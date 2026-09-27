//! Explicit prepared-render assets — partial PERF-R02 delivery.
//!
//! [`PreparedSolid`] is an immutable snapshot of one solid's tessellated
//! geometry with its GPU buffers already uploaded to a specific
//! [`OffscreenSession`](crate::OffscreenSession)'s device. Preparing is an
//! explicit operation ([`OffscreenSession::prepare`]); drawing a prepared
//! asset ([`OffscreenSession::draw_prepared`]) skips tessellation and geometry
//! upload entirely, so camera-only frames pay only uniforms + draw + readback.
//!
//! # What this covers (and what it does not)
//!
//! Covered: one explicit prepare (tessellate + upload), arbitrarily many
//! camera-only draws with zero tessellation/upload, explicit replacement
//! (prepare again, drop the old asset), and resource release (drop the asset).
//!
//! Explicitly out of scope — no claim is made here: automatic topology
//! revision tracking, dirty-face invalidation, incremental tessellation, and
//! asynchronous readback. Those remain future PERF-R02/R04 work; a prepared
//! asset never refreshes itself.
//!
//! # Asset lifetime and session association
//!
//! - A prepared asset is bound to the session (and GPU device) that prepared
//!   it. Each session owns a unique id; the asset records it at prepare time
//!   and [`OffscreenSession::draw_prepared`] rejects foreign assets with
//!   [`RenderError::WrongSession`](crate::RenderError::WrongSession) before
//!   touching the GPU. The refusing session stays usable.
//! - Dropping the asset releases its GPU vertex/index/edge buffers. Dropping
//!   the session releases the device; assets cannot outlive useful drawing
//!   because drawing is always a session method.
//!
//! # What an asset captures
//!
//! At prepare time the asset snapshots, for one `deflection` value:
//! - mesh vertices: center-relative (RTC) f32 positions, f32 normals, and the
//!   per-vertex face id (`FaceId.index() + 1`; `0` stays the background
//!   sentinel), exactly as the render mesh builder produces them;
//! - the triangle index buffer;
//! - the topological edge line-list (RTC f32), when the solid has edges;
//! - the f64 RTC center folded into the per-frame view matrix;
//! - counts and uploaded byte sizes ([`PreparedStats`]).
//!
//! The asset retains no [`Topology`](remus_topology::Topology) reference and
//! no [`SolidId`](remus_topology::solid::SolidId): there is deliberately no
//! global cache keyed by numeric id, so two documents with identical numerical
//! ids can never alias.
//!
//! # Frame options (change freely — no rebuild)
//!
//! [`PreparedDrawOpts`] carries every per-frame knob: camera (any orbit, pan,
//! zoom, or projection change), render size (served by the session's cached
//! targets, rebuilt on size change exactly as in
//! [`OffscreenSession::render`](crate::OffscreenSession::render)), the edge
//! overlay toggle (both pipeline variants are prebuilt; the id buffer is
//! unaffected by the toggle), background clear color, and ambient light.
//! Tessellation quality (`deflection`) is baked at prepare time and is not a
//! frame option — changing quality means preparing a new asset.
//!
//! # Topology edits, replacement, and staleness
//!
//! The asset is a snapshot, not a live view: edits to the source topology
//! after preparation do not affect it, and drawing it renders the geometry as
//! prepared. Replacement is explicit — call
//! [`OffscreenSession::prepare`](crate::OffscreenSession::prepare) again and
//! drop (or overwrite) the old asset. Because the draw API takes the asset
//! itself rather than a solid id, a prepared draw can never silently present
//! stale geometry *as the current topology*; staleness is always the caller's
//! explicit choice to keep drawing an older snapshot.
//!
//! # Device loss
//!
//! A poisoned session fails closed: both `prepare` and `draw_prepared` return
//! [`RenderError::DeviceLost`](crate::RenderError::DeviceLost) without
//! touching the GPU. Create a new session (and re-prepare) to recover.

use remus_math::vec::Point3;

use crate::RenderOpts;
use crate::pipeline::GeometryBuffers;

/// Per-frame options for drawing a [`PreparedSolid`].
///
/// Every field may change between draws without rebuilding the asset:
/// camera moves are the intended steady state (zero tessellation, zero
/// upload). Tessellation quality is deliberately absent — it is baked into
/// the asset at prepare time.
#[derive(Debug, Clone, Copy)]
pub struct PreparedDrawOpts {
    /// Output width in pixels (must be non-zero).
    pub width: u32,
    /// Output height in pixels (must be non-zero).
    pub height: u32,
    /// Draw the prepared topological edge overlay.
    pub edges: bool,
    /// Background clear color as linear RGBA in `[0, 1]`.
    pub background: [f32; 4],
    /// Ambient light fraction in `[0, 1]`.
    pub ambient: f32,
}

impl PreparedDrawOpts {
    /// Create frame options for a `width` x `height` draw with sensible
    /// defaults (edges on, light-gray background, modest ambient term).
    #[must_use]
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            edges: true,
            background: [0.11, 0.12, 0.14, 1.0],
            ambient: 0.25,
        }
    }
}

impl From<&RenderOpts> for PreparedDrawOpts {
    /// Convert render options to frame options, dropping `deflection` (baked
    /// into the asset at prepare time).
    fn from(opts: &RenderOpts) -> Self {
        Self {
            width: opts.width,
            height: opts.height,
            edges: opts.edges,
            background: opts.background,
            ambient: opts.ambient,
        }
    }
}

/// Geometry census of a [`PreparedSolid`]: what was tessellated and uploaded.
#[derive(Debug, Clone, Copy)]
pub struct PreparedStats {
    /// Triangle count (index buffer length / 3).
    pub triangles: usize,
    /// Mesh vertex count (non-indexed-per-face expansion).
    pub vertices: usize,
    /// Edge-overlay line segments (line-list vertex pairs).
    pub edge_segments: usize,
    /// Uploaded mesh-vertex bytes.
    pub vertex_bytes: usize,
    /// Uploaded index bytes.
    pub index_bytes: usize,
    /// Uploaded edge-vertex bytes (`0` when the solid has no edges).
    pub edge_bytes: usize,
    /// Linear chord tolerance the asset was tessellated at.
    pub deflection: f64,
}

/// CPU-side phase timings of the last
/// [`prepare`](crate::OffscreenSession::prepare), for benchmarking.
///
/// `tessellate` covers render-mesh construction
/// (grouped tessellation + edge sampling + RTC conversion);
/// `upload` covers GPU buffer creation from that mesh.
#[derive(Debug, Clone, Copy)]
pub struct PrepareTimings {
    /// Time spent tessellating and converting to center-relative geometry.
    pub tessellate: std::time::Duration,
    /// Time spent creating GPU vertex/index/edge buffers.
    pub upload: std::time::Duration,
}

/// GPU-side phase timings of the last
/// [`draw_prepared`](crate::OffscreenSession::draw_prepared), for benchmarking.
///
/// `submit` covers globals upload, scene encoding, and queue submission;
/// `readback` covers mapping both readback buffers and copying them out
/// (synchronous; block-on-poll — async readback remains PERF-R04 work).
#[derive(Debug, Clone, Copy)]
pub struct PreparedDrawTimings {
    /// Time spent uploading globals, encoding, and submitting the frame.
    pub submit: std::time::Duration,
    /// Time spent mapping + copying the color and face-id readback buffers.
    pub readback: std::time::Duration,
}

/// An immutable prepared-render asset: tessellated geometry plus GPU buffers
/// bound to one session's device.
///
/// Construct via [`OffscreenSession::prepare`](crate::OffscreenSession::prepare)
/// and draw via
/// [`draw_prepared`](crate::OffscreenSession::draw_prepared). The asset is a
/// snapshot — see the module docs for lifetime, capture, frame-option,
/// staleness, and device-loss contracts.
///
/// Dropping the asset releases its GPU buffers; the session itself is
/// unaffected and keeps serving other assets and direct renders.
pub struct PreparedSolid {
    pub(crate) geometry: GeometryBuffers,
    pub(crate) center: Point3,
    pub(crate) session_id: u64,
    pub(crate) has_edges: bool,
    pub(crate) stats: PreparedStats,
}

impl PreparedSolid {
    /// Id of the session (and GPU device) this asset was prepared on.
    ///
    /// [`draw_prepared`](crate::OffscreenSession::draw_prepared) on any other
    /// session fails with
    /// [`RenderError::WrongSession`](crate::RenderError::WrongSession).
    #[must_use]
    pub fn session_id(&self) -> u64 {
        self.session_id
    }

    /// Linear chord tolerance this asset was tessellated at.
    #[must_use]
    pub fn deflection(&self) -> f64 {
        self.stats.deflection
    }

    /// Whether the asset carries a non-empty edge overlay.
    #[must_use]
    pub fn has_edges(&self) -> bool {
        self.has_edges
    }

    /// Geometry census: what was tessellated and uploaded.
    #[must_use]
    pub fn stats(&self) -> PreparedStats {
        self.stats
    }
}
