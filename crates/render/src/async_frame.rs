//! Asynchronous selective readback for repeated offscreen rendering (PERF-R04).
//!
//! [`OffscreenSession`](crate::OffscreenSession) submission methods encode and
//! enqueue GPU work and return a [`FrameTicket`] immediately, without waiting
//! for GPU execution or CPU readback. Completion is explicit via
//! [`OffscreenSession::poll_frame`] (non-blocking) or
//! [`OffscreenSession::wait_frame`] (blocking). This separates submission from
//! completion so callers can overlap CPU work (tessellation, encoding later
//! frames) with GPU execution of earlier frames.
//!
//! # Ownership and completion semantics
//!
//! - A ticket is bound to the session that issued it. Each ticket carries the
//!   issuing [`OffscreenSession`](crate::OffscreenSession)'s id; using it on
//!   any other session fails with [`RenderError::WrongSession`](crate::RenderError::WrongSession)
//!   before touching the GPU. The refusing session stays usable.
//! - Submission captures camera and frame options by value (globals upload +
//!   scene encode happen synchronously inside `submit_*`). Mutating the
//!   caller's [`Camera`](crate::Camera) or options afterwards does not affect
//!   the pending frame.
//! - Each pending frame owns exclusive offscreen targets (color, id, depth
//!   textures) and exclusive staging buffers for the requested outputs. A
//!   later submission never overwrites resources still needed by an earlier
//!   frame; there is no shared mutable target between in-flight frames.
//! - Prepared-asset lifetime: submission clones the asset's GPU buffer handles
//!   into the pending frame, so dropping or replacing the
//!   [`PreparedSolid`](crate::PreparedSolid) after submission does not affect
//!   the pending frame. The direct (`submit_render`) path likewise retains its
//!   freshly uploaded geometry until completion.
//! - Output selection ([`ReadbackSelection`]) controls which copies, maps, and
//!   CPU decodes happen. Unrequested outputs are skipped entirely: no
//!   `copy_texture_to_buffer` is encoded, no buffer is mapped, and no bytes
//!   are decoded for them. [`ReadbackSelection::None`] encodes the render
//!   without any readback (useful for warmup/timing) and completes with empty
//!   outputs.
//! - Completion order is caller-chosen: tickets may be collected in any order,
//!   including a different order from submission. Each [`AsyncFrameOutput`]
//!   carries its [`FrameTicket`], so color and face-id data cannot be
//!   mismatched across frames.
//! - The existing synchronous APIs (`render`, `draw_prepared`,
//!   [`crate::render_solid_offscreen`]) are preserved verbatim and share the
//!   device/pipelines but never touch pending async resources.
//!
//! # Bounded in-flight resources
//!
//! At most [`MAX_ASYNC_FRAMES`] frames may be pending per session. Submitting
//! beyond the bound fails with [`RenderError::QueueFull`](crate::RenderError::QueueFull)
//! without allocating and without poisoning the session. Retained memory is
//! therefore bounded by `MAX_ASYNC_FRAMES` times the per-frame allocation
//! (targets + staging + retained geometry), even when the caller stops
//! consuming results.
//!
//! - Full queue: `submit_*` returns `QueueFull`; collect or cancel a pending
//!   frame and retry.
//! - Cancellation: [`OffscreenSession::cancel_frame`](crate::OffscreenSession::cancel_frame)
//!   releases a pending frame's resources without mapping or decoding. It is
//!   idempotent (`Ok(false)` when the ticket is already gone) and works even
//!   on a poisoned session so callers can always reclaim memory.
//! - Abandoned tickets: dropping a [`FrameTicket`] without collecting leaves
//!   the frame pending (it still occupies a queue slot) until cancelled or
//!   until the session is dropped. Queue-full is the backpressure signal to
//!   drain.
//! - Resize: each pending frame owns its size; submitting a different size
//!   never invalidates earlier frames. The synchronous cached targets are
//!   untouched by async submissions.
//! - Asset replacement: pending frames hold buffer-handle clones, so preparing
//!   a replacement asset (or dropping the old one) never affects them.
//! - Session shutdown: dropping the session drops the device plus every
//!   pending frame's textures, buffers, and retained geometry. The GPU may
//!   defer physical reclamation until submitted work completes.
//!
//! # Failure behavior
//!
//! Validation failures (`InvalidSize`, `PixelBudgetExceeded`, `SizeTooLarge`,
//! `MeshData`, `Operations`, `Topology`, `WrongSession` on submit, `QueueFull`)
//! never poison the session. GPU failures (`BufferMap`, `Poll`) poison it: the
//! failing completion returns the error as-is and records the reason; every
//! later `submit_*`, `poll_frame`, and `wait_frame` returns
//! [`RenderError::DeviceLost`](crate::RenderError::DeviceLost) without touching
//! the GPU (pending frames are released as they are observed). `cancel_frame`
//! still releases resources on a poisoned session. Validation errors never
//! become a poisoned session.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Maximum in-flight async frames per session.
///
/// Bounds retained memory: at most this many frames' targets, staging buffers,
/// and retained geometry exist at once, even when the caller stops consuming.
pub const MAX_ASYNC_FRAMES: usize = 8;

/// Which outputs an async frame reads back to the CPU.
///
/// Unrequested outputs skip every stage: no texture-to-buffer copy is encoded,
/// no buffer is mapped, and no bytes are decoded. This saves copy bandwidth,
/// mapping latency, and CPU decode for callers that need only color (preview),
/// only face ids (picking), or neither (warmup/timing).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReadbackSelection {
    /// Read back only the shaded color image.
    Color,
    /// Read back only the face-id buffer (picking).
    FaceIds,
    /// Read back both color and face ids (same as the synchronous path).
    Both,
    /// No CPU readback: render executes on the GPU without any copy, map, or
    /// decode. Completion signals GPU execution only.
    None,
}

impl ReadbackSelection {
    /// Whether the color image is requested.
    #[must_use]
    pub fn wants_color(self) -> bool {
        matches!(self, Self::Color | Self::Both)
    }

    /// Whether the face-id buffer is requested.
    #[must_use]
    pub fn wants_ids(self) -> bool {
        matches!(self, Self::FaceIds | Self::Both)
    }
}

/// A frame ticket identifying one submitted async frame.
///
/// Tickets are `Copy` handles, not RAII guards: dropping a ticket does not
/// cancel its frame (see the module docs for abandoned-ticket semantics).
/// Each ticket records the issuing session's id, so misuse on another session
/// is refused with `WrongSession`. The ticket also snapshots the frame's size
/// and output selection for introspection and reporting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FrameTicket {
    session_id: u64,
    frame_id: u64,
    width: u32,
    height: u32,
    selection: ReadbackSelection,
}

impl FrameTicket {
    /// Create a ticket. Only the issuing session calls this.
    #[must_use]
    pub fn new(
        session_id: u64,
        frame_id: u64,
        width: u32,
        height: u32,
        selection: ReadbackSelection,
    ) -> Self {
        Self {
            session_id,
            frame_id,
            width,
            height,
            selection,
        }
    }

    /// Id of the session that issued this ticket.
    #[must_use]
    pub fn session_id(self) -> u64 {
        self.session_id
    }

    /// Per-session unique frame id.
    #[must_use]
    pub fn frame_id(self) -> u64 {
        self.frame_id
    }

    /// Frame width in pixels.
    #[must_use]
    pub fn width(self) -> u32 {
        self.width
    }

    /// Frame height in pixels.
    #[must_use]
    pub fn height(self) -> u32 {
        self.height
    }

    /// Output selection captured at submission.
    #[must_use]
    pub fn selection(self) -> ReadbackSelection {
        self.selection
    }
}

/// Phase timings for one completed async frame.
///
/// `submit` is CPU encoding/enqueue time measured inside `submit_*` (it never
/// waits for the GPU). `wait` covers GPU execution plus mapping (the blocking
/// or polling section of `wait_frame`/`poll_frame`). `decode` covers CPU
/// unpadding and image/buffer construction from mapped bytes. These are
/// host-observed durations, not separate GPU execution timestamps.
#[derive(Debug, Clone, Copy)]
pub struct AsyncFrameTimings {
    /// Time spent uploading globals, encoding, and submitting the frame.
    pub submit: Duration,
    /// Time waiting for GPU completion and buffer mapping.
    pub wait: Duration,
    /// Time decoding mapped bytes into CPU outputs.
    pub decode: Duration,
}

/// The completed outputs of one async frame.
///
/// Only requested outputs are present: [`ReadbackSelection::Color`] yields
/// `color` without `id_buffer`, [`ReadbackSelection::FaceIds`] yields
/// `id_buffer` without `color`, [`ReadbackSelection::Both`] yields both, and
/// [`ReadbackSelection::None`] yields neither. The [`ticket`](Self::ticket)
/// field keeps frame identity attached to both color and picking output.
pub struct AsyncFrameOutput {
    /// The ticket this output completes.
    pub ticket: FrameTicket,
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// Shaded color image (sRGB, RGBA8) when requested, else `None`.
    pub color: Option<image::RgbaImage>,
    /// Per-pixel face id, row-major (`width * height` entries) when requested,
    /// else `None`. `0` is background; otherwise `FaceId.index() + 1`.
    pub id_buffer: Option<Vec<u32>>,
    /// Padded staging bytes transferred GPU -> CPU for the requested outputs.
    pub transferred_bytes: u64,
    /// Phase timings (submit measured at submission; wait/decode at completion).
    pub timings: AsyncFrameTimings,
}

impl AsyncFrameOutput {
    /// Face id at pixel `(x, y)`, or `None` for background, out-of-bounds, or
    /// when ids were not requested ([`ReadbackSelection::Color`] / `None`).
    #[must_use]
    pub fn face_id_at(&self, x: u32, y: u32) -> Option<u32> {
        let ids = self.id_buffer.as_ref()?;
        if x >= self.width || y >= self.height {
            return None;
        }
        let idx = (y * self.width + x) as usize;
        match ids.get(idx).copied() {
            Some(0) | None => None,
            Some(v) => Some(v),
        }
    }
}

/// Shared mapping result written by a `map_async` callback and read without
/// consuming by polls, so non-blocking readiness checks never lose a partial
/// completion. `None` means the callback has not fired yet; `Some(Ok(()))`
/// means mapped; `Some(Err(msg))` means the mapping failed.
pub type SharedMapResult = Arc<Mutex<Option<Result<(), String>>>>;

/// Create a fresh shared mapping slot.
pub fn shared_map_slot() -> SharedMapResult {
    Arc::new(Mutex::new(None))
}

/// Read a shared mapping slot without consuming it.
pub fn read_map_slot(slot: &SharedMapResult) -> Option<Result<(), String>> {
    slot.lock().ok().and_then(|guard| guard.clone())
}

/// Write a shared mapping slot from a `map_async` callback (never panics).
pub fn write_map_slot(slot: &SharedMapResult, res: Result<(), String>) {
    if let Ok(mut guard) = slot.lock() {
        *guard = Some(res);
    }
}

/// Mapping state for a pending frame.
///
/// Submission never maps. The first `poll_frame`/`wait_frame` initiates mapping
/// (one `map_async` per requested buffer); later polls read the same shared
/// slots so buffers are never double-mapped and partial completion is never
/// lost. [`ReadbackSelection::None`] frames never map: completion is the
/// `done_flag` set by the `on_submitted_work_done` callback registered at
/// submission.
pub enum PendingMapping {
    /// Nothing mapped yet.
    Unmapped,
    /// Mapping initiated; shared slots receive completion.
    Mapping {
        color: Option<SharedMapResult>,
        ids: Option<SharedMapResult>,
    },
}

/// One in-flight async frame: exclusive targets, staging, and retained geometry.
///
/// Textures and staging buffers are owned here until collection or
/// cancellation, so no later submission can overwrite them. `retained_geometry`
/// clones the vertex/index/edge buffer handles referenced by the encoded draw,
/// keeping them alive even when the caller drops or replaces the prepared
/// asset. `retained_bytes` accounts textures + staging + geometry for
/// [`crate::OffscreenSession::in_flight_bytes`]. `done_flag` (only for
/// [`ReadbackSelection::None`]) is set by the submission's
/// `on_submitted_work_done` callback.
#[allow(dead_code)]
pub struct PendingFrame {
    // Textures, geometry handles, and extents are retained to keep GPU
    // resources alive until completion; they are never read on the CPU.
    pub ticket: FrameTicket,
    pub selection: ReadbackSelection,
    pub width: u32,
    pub height: u32,
    pub extent: wgpu::Extent3d,
    pub color_tex: wgpu::Texture,
    pub depth_tex: wgpu::Texture,
    pub id_tex: wgpu::Texture,
    pub color_buf: Option<wgpu::Buffer>,
    pub id_buf: Option<wgpu::Buffer>,
    pub color_padded_bpr: u32,
    pub id_padded_bpr: u32,
    pub retained_geometry: Vec<wgpu::Buffer>,
    pub geometry_bytes: usize,
    pub retained_bytes: u64,
    pub submit_duration: Duration,
    pub mapping: PendingMapping,
    pub done_flag: Option<Arc<AtomicBool>>,
}

/// Mark a `None`-selection frame done from its `on_submitted_work_done`
/// callback (never panics).
pub fn mark_done(flag: &Arc<AtomicBool>) {
    flag.store(true, Ordering::Release);
}

/// Read a `None`-selection frame's done flag.
pub fn is_done(flag: &Arc<AtomicBool>) -> bool {
    flag.load(Ordering::Acquire)
}
