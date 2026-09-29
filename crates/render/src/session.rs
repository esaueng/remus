//! Reusable offscreen rendering session (PERF-R01) with asynchronous selective
//! readback (PERF-R04).
//!
//! [`OffscreenSession`] owns one GPU device plus the compatible pipelines and
//! reuses them across frames. The convenience path
//! ([`crate::render_solid_offscreen`]) creates a fresh device and pipelines
//! per render; a session pays that cold cost once and then only
//! tessellates, uploads per-frame geometry, renders, and reads back.
//!
//! # Scope
//!
//! - Reused across frames: adapter/device/queue, pipeline layout, globals
//!   uniform + bind group, mesh + edge pipelines (both variants, so edge
//!   toggles never rebuild), and offscreen targets + readback buffers while
//!   the size stays constant (synchronous path).
//! - Rebuilt per frame by [`OffscreenSession::render`]: tessellation, GPU
//!   vertex/index/edge buffers, globals contents (camera, ambient), and the
//!   clear color. Targets are recreated when `width`/`height` change.
//! - Skipped by prepared draws: [`OffscreenSession::draw_prepared`] reuses a
//!   [`PreparedSolid`](crate::PreparedSolid)'s uploaded geometry, so
//!   camera-only frames tessellate and upload nothing (partial PERF-R02; see
//!   [`PreparedSolid`](crate::PreparedSolid) for the asset contract).
//! - Asynchronous selective readback (PERF-R04): [`OffscreenSession::submit_render`]
//!   and [`OffscreenSession::submit_prepared`] encode and enqueue GPU work and
//!   return a [`FrameTicket`](crate::FrameTicket) immediately, without waiting
//!   for GPU execution or CPU readback. Completion is explicit via
//!   [`OffscreenSession::poll_frame`] (non-blocking) or
//!   [`OffscreenSession::wait_frame`] (blocking), with output selection via
//!   [`ReadbackSelection`](crate::ReadbackSelection). See
//!   [`ReadbackSelection`](crate::ReadbackSelection) for the full ownership, bounding, and failure
//!   contract.
//! - Explicitly out of scope: automatic topology revision tracking,
//!   dirty-face invalidation, incremental tessellation, compute-mesher
//!   changes, and window-viewer changes.
//!
//! # Failure policy
//!
//! Validation and tessellation failures (`InvalidSize`,
//! `PixelBudgetExceeded`, `SizeTooLarge`, `MeshData`, `Operations`,
//! `Topology`) leave the session usable: cached targets are untouched and a
//! later valid render succeeds. Async validation failures (`QueueFull`,
//! `UnknownTicket`, `WrongSession` on submit) likewise never poison.
//!
//! GPU execution failures (`BufferMap`, `Poll`) poison the session: the first
//! failure is returned as-is and the session records the reason; every later
//! [`OffscreenSession::render`], `submit_*`, `poll_frame`, and `wait_frame`
//! returns [`RenderError::DeviceLost`] without touching the GPU
//! (`cancel_frame` still releases resources so callers can reclaim memory).
//! Create a new session to recover. Real device loss is not synthesized in
//! tests; use [`OffscreenSession::inject_device_loss_for_test`] to exercise
//! the poisoned path. No claim is made about recovering a physically lost
//! device.
//!
//! # Resource release
//!
//! Dropping the session drops its device, pipelines, globals, cached sync
//! target handles, and every pending async frame's textures, buffers, and
//! retained geometry. On a sync size change, old target handles are dropped
//! before allocating replacements. The GPU may defer physical reclamation
//! until submitted work completes.
//!
//! # No exact-kernel speedup claim
//!
//! Session reuse, prepared assets, and async readback reduce renderer overhead
//! (device setup, tessellation/upload repetition, CPU/GPU serialization) for
//! repeated offscreen rendering. They do not accelerate exact kernel geometry.

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::Instant;

use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

use crate::async_frame::{
    AsyncFrameOutput, AsyncFrameTimings, FrameTicket, MAX_ASYNC_FRAMES, MAX_ASYNC_IN_FLIGHT_BYTES,
    PendingFrame, PendingMapping, ReadbackSelection, is_done, mark_done, read_map_slot,
    shared_map_slot, write_map_slot,
};
use crate::camera::Camera;
use crate::error::RenderError;
use crate::mesh::{EdgeVertex, RenderMesh, Vertex};
use crate::pipeline::{
    COLOR_FORMAT_OFFSCREEN, DEPTH_FORMAT, GeometryBuffers, Globals, GlobalsBinding, GpuContext,
    ID_FORMAT, PassTargets, Pipelines, build_globals, encode_scene, map_and_read,
    padded_bytes_per_row, unpad_to_rgba, unpad_to_u32,
};
use crate::prepared::{
    PrepareTimings, PreparedDrawOpts, PreparedDrawTimings, PreparedSolid, PreparedStats,
};
use crate::{MAX_OFFSCREEN_PIXELS, RenderOpts, RenderOutput};

/// Source of unique [`OffscreenSession`] ids (prepared-asset affinity tokens).
static NEXT_SESSION_ID: AtomicU64 = AtomicU64::new(1);

/// Cached offscreen targets for one `width` x `height` size (synchronous path).
///
/// Textures, views, and readback buffers are recreated only when the requested
/// size changes; every frame clears them (`LoadOp::Clear` in
/// [`encode_scene`]), so no previous frame's color, ids, or depth leak into
/// the next render. Async submissions never touch these targets: each pending
/// frame owns exclusive textures and staging buffers (see
/// [`ReadbackSelection`](crate::ReadbackSelection).
struct CachedTargets {
    width: u32,
    height: u32,
    extent: wgpu::Extent3d,
    // Retained to keep the targets alive (the views borrow them on some
    // backends); readback buffers are the live handles per frame.
    #[allow(dead_code)]
    color_tex: wgpu::Texture,
    #[allow(dead_code)]
    depth_tex: wgpu::Texture,
    #[allow(dead_code)]
    id_tex: wgpu::Texture,
    color_view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
    id_view: wgpu::TextureView,
    color_readback: wgpu::Buffer,
    id_readback: wgpu::Buffer,
    color_padded_bpr: u32,
    id_padded_bpr: u32,
}

impl CachedTargets {
    fn build(device: &wgpu::Device, width: u32, height: u32) -> Self {
        let extent = wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        };
        let color_tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("session color target"),
            size: extent,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: COLOR_FORMAT_OFFSCREEN,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let depth_tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("session depth target"),
            size: extent,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let id_tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("session id target"),
            size: extent,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: ID_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let color_view = color_tex.create_view(&wgpu::TextureViewDescriptor::default());
        let depth_view = depth_tex.create_view(&wgpu::TextureViewDescriptor::default());
        let id_view = id_tex.create_view(&wgpu::TextureViewDescriptor::default());

        let color_padded_bpr = padded_bytes_per_row(width, 4);
        let id_padded_bpr = padded_bytes_per_row(width, 4);
        let color_readback = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("session color readback"),
            size: u64::from(color_padded_bpr) * u64::from(height),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let id_readback = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("session id readback"),
            size: u64::from(id_padded_bpr) * u64::from(height),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        Self {
            width,
            height,
            extent,
            color_tex,
            depth_tex,
            id_tex,
            color_view,
            depth_view,
            id_view,
            color_readback,
            id_readback,
            color_padded_bpr,
            id_padded_bpr,
        }
    }
}

/// A reusable offscreen rendering session.
///
/// Owns one GPU device and the compatible pipelines; call
/// [`OffscreenSession::render`] repeatedly to avoid the per-render
/// adapter/device/pipeline setup the convenience path pays every time. For
/// repeated rendering with overlapped CPU/GPU work, use the async submission
/// methods ([`OffscreenSession::submit_render`],
/// [`OffscreenSession::submit_prepared`]) plus explicit completion
/// ([`OffscreenSession::poll_frame`], [`OffscreenSession::wait_frame`]); see
/// [`ReadbackSelection`](crate::ReadbackSelection) for the ticket, selection, bounding, and failure
/// contract.
///
/// A session is bound to a single device and the offscreen color format. It
/// is `!Sync` (it takes `&mut self` per frame) and must be created on the
/// thread that renders; do not share one across threads.
pub struct OffscreenSession {
    ctx: GpuContext,
    /// Unique affinity token: prepared assets record it and refuse to draw on
    /// any other session (see [`RenderError::WrongSession`]). Async tickets
    /// carry it as well.
    session_id: u64,
    adapter_info: String,
    max_texture_dimension_2d: u32,
    // Retained to keep the pipeline layout alive for the session lifetime;
    // the pipelines were built from it.
    #[allow(dead_code)]
    pipeline_layout: wgpu::PipelineLayout,
    globals: GlobalsBinding,
    pipelines_with_edges: Pipelines,
    pipelines_without_edges: Pipelines,
    targets: Option<CachedTargets>,
    target_rebuilds: u64,
    /// Successful [`OffscreenSession::prepare`] calls (each tessellates and
    /// uploads exactly once).
    prepare_calls: u64,
    /// Successful [`OffscreenSession::draw_prepared`] calls (each skips
    /// tessellation and upload).
    prepared_draw_calls: u64,
    /// CPU-side phase timings of the last `prepare`, for benchmarking.
    last_prepare_timings: Option<PrepareTimings>,
    /// Host wall-clock phase timings of the last `draw_prepared`.
    last_draw_timings: Option<PreparedDrawTimings>,
    failed: Option<String>,
    next_frame_id: u64,
    pending: HashMap<u64, PendingFrame>,
    in_flight_bytes: u64,
    peak_in_flight_bytes: u64,
    total_transferred_bytes: u64,
}

impl OffscreenSession {
    /// Create a session: one cold adapter/device setup plus both pipeline
    /// variants (edges on/off) for the offscreen color format.
    ///
    /// No targets are allocated until the first [`OffscreenSession::render`]:
    /// the size comes from [`RenderOpts`], so construction never fails on
    /// dimensions.
    ///
    /// # Errors
    ///
    /// [`RenderError::NoAdapter`] if no adapter (GPU or software fallback)
    /// exists, or [`RenderError::DeviceRequest`] if the device cannot be
    /// created. The software-adapter fallback is the same
    /// GPU context setup path the convenience API uses.
    pub fn new() -> Result<Self, RenderError> {
        let ctx = GpuContext::new()?;
        let info = ctx.adapter.get_info();
        let adapter_info = format!(
            "{:?} / {} ({:?})",
            info.backend, info.name, info.device_type
        );
        let max_texture_dimension_2d = ctx.device.limits().max_texture_dimension_2d;

        // The uniform contents are uploaded per frame; start from zero.
        let initial = Globals {
            view_proj: [0.0; 16],
            view_dir: [0.0; 4],
            ambient: 0.0,
            selected_id: 0,
            _pad: [0.0; 2],
        };
        let globals = GlobalsBinding::new(&ctx.device, &initial);
        let pipeline_layout = ctx
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("session pipeline layout"),
                bind_group_layouts: &[Some(&globals.layout)],
                immediate_size: 0,
            });
        let pipelines_with_edges =
            Pipelines::new(&ctx.device, &pipeline_layout, COLOR_FORMAT_OFFSCREEN, true);
        let pipelines_without_edges =
            Pipelines::new(&ctx.device, &pipeline_layout, COLOR_FORMAT_OFFSCREEN, false);

        Ok(Self {
            ctx,
            session_id: NEXT_SESSION_ID.fetch_add(1, Ordering::Relaxed),
            adapter_info,
            max_texture_dimension_2d,
            pipeline_layout,
            globals,
            pipelines_with_edges,
            pipelines_without_edges,
            targets: None,
            target_rebuilds: 0,
            prepare_calls: 0,
            prepared_draw_calls: 0,
            last_prepare_timings: None,
            last_draw_timings: None,
            failed: None,
            next_frame_id: 1,
            pending: HashMap::new(),
            in_flight_bytes: 0,
            peak_in_flight_bytes: 0,
            total_transferred_bytes: 0,
        })
    }

    /// Unique id of this session, bound at creation.
    ///
    /// Prepared assets record the id of their preparing session; drawing an
    /// asset on any other session fails with [`RenderError::WrongSession`].
    /// Async tickets carry the issuing session's id and are refused elsewhere.
    #[must_use]
    pub fn session_id(&self) -> u64 {
        self.session_id
    }

    /// Adapter backend/name/type selected at creation (real GPU or software
    /// fallback), e.g. `"Vulkan / NVIDIA GeForce RTX 3090 (DiscreteGpu)"`.
    #[must_use]
    pub fn adapter_info(&self) -> &str {
        &self.adapter_info
    }

    /// The device's maximum 2D texture dimension, cached at creation.
    #[must_use]
    pub fn max_texture_dimension_2d(&self) -> u32 {
        self.max_texture_dimension_2d
    }

    /// Currently cached target size, or `None` before the first render.
    ///
    /// This covers the synchronous path only; async frames own exclusive
    /// per-frame targets and never touch these.
    #[must_use]
    pub fn cached_size(&self) -> Option<(u32, u32)> {
        self.targets.as_ref().map(|t| (t.width, t.height))
    }

    /// How many times cached targets have been (re)built. Stays at 1 across
    /// repeated same-size renders; increments on each size change.
    #[must_use]
    pub fn target_rebuilds(&self) -> u64 {
        self.target_rebuilds
    }

    /// Successful [`OffscreenSession::prepare`] calls.
    ///
    /// Each prepare tessellates and uploads exactly once; with
    /// [`OffscreenSession::prepared_draw_calls`] this proves skipped work: N
    /// prepared draws after one prepare performed N frames with one
    /// tessellation and one upload.
    #[must_use]
    pub fn prepare_calls(&self) -> u64 {
        self.prepare_calls
    }

    /// Successful [`OffscreenSession::draw_prepared`] calls.
    ///
    /// Each of these skipped tessellation and geometry upload entirely.
    #[must_use]
    pub fn prepared_draw_calls(&self) -> u64 {
        self.prepared_draw_calls
    }

    /// CPU-side phase timings of the last successful `prepare`
    /// (tessellation vs. upload split), or `None` before the first prepare.
    #[must_use]
    pub fn last_prepare_timings(&self) -> Option<PrepareTimings> {
        self.last_prepare_timings
    }

    /// Host wall-clock timings of the last successful `draw_prepared`.
    ///
    /// `submit` is CPU encoding/enqueue time; `readback` also waits for GPU
    /// render/copy completion. Returns `None` before the first prepared draw.
    #[must_use]
    pub fn last_draw_timings(&self) -> Option<PreparedDrawTimings> {
        self.last_draw_timings
    }

    /// Maximum in-flight async frames per session.
    ///
    /// Submitting beyond this bound fails with [`RenderError::QueueFull`]
    /// without allocating. See [`ReadbackSelection`](crate::ReadbackSelection) for the bounding contract.
    #[must_use]
    #[allow(clippy::unused_self)]
    pub fn max_in_flight(&self) -> usize {
        MAX_ASYNC_FRAMES
    }

    /// Current number of pending (submitted, not yet collected/cancelled)
    /// async frames.
    #[must_use]
    pub fn pending_frame_count(&self) -> usize {
        self.pending.len()
    }

    /// Tickets of all pending async frames, in arbitrary order.
    #[must_use]
    pub fn pending_tickets(&self) -> Vec<FrameTicket> {
        self.pending.values().map(|f| f.ticket).collect()
    }

    /// Current retained bytes across all pending async frames (targets +
    /// staging + retained geometry).
    #[must_use]
    pub fn in_flight_bytes(&self) -> u64 {
        self.in_flight_bytes
    }

    /// Peak [`OffscreenSession::in_flight_bytes`] observed on this session.
    #[must_use]
    pub fn peak_in_flight_bytes(&self) -> u64 {
        self.peak_in_flight_bytes
    }

    /// Cumulative padded staging bytes transferred GPU -> CPU across all
    /// completed async frames (requested outputs only).
    #[must_use]
    pub fn total_transferred_bytes(&self) -> u64 {
        self.total_transferred_bytes
    }

    /// Whether the session is poisoned by a device-level failure.
    #[must_use]
    pub fn is_failed(&self) -> bool {
        self.failed.is_some()
    }

    /// The recorded device-failure reason, if [`OffscreenSession::is_failed`].
    #[must_use]
    pub fn failure_reason(&self) -> Option<&str> {
        self.failed.as_deref()
    }

    /// Poison the session without touching the GPU, exercising the
    /// [`RenderError::DeviceLost`] path without a physical device loss.
    ///
    /// After this call, [`OffscreenSession::render`], `submit_*`,
    /// `poll_frame`, and `wait_frame` return `DeviceLost` until the session is
    /// dropped and recreated (`cancel_frame` still releases pending resources
    /// so callers can reclaim memory). This is a test hook: it makes no claim
    /// about recovering a physically lost device.
    #[doc(hidden)]
    pub fn inject_device_loss_for_test(&mut self, reason: impl Into<String>) {
        self.failed = Some(reason.into());
    }

    /// Render `solid` offscreen, reusing the session's device and pipelines.
    ///
    /// Tessellates at `opts.deflection`, uploads fresh per-frame geometry,
    /// draws with the prebuilt pipelines, and reads back color + face ids.
    /// See the module docs for what is reused versus rebuilt, and for the
    /// failure policy.
    ///
    /// # Errors
    ///
    /// - [`RenderError::DeviceLost`] if the session is poisoned.
    /// - [`RenderError::InvalidSize`] / [`RenderError::PixelBudgetExceeded`] /
    ///   [`RenderError::SizeTooLarge`] on bad dimensions (session stays usable).
    /// - [`RenderError::Operations`] / [`RenderError::Topology`] /
    ///   [`RenderError::MeshData`] on tessellation failure (session stays usable).
    /// - [`RenderError::BufferMap`] / [`RenderError::Poll`] on GPU readback
    ///   failure (poisons the session; the next call returns `DeviceLost`).
    pub fn render(
        &mut self,
        topo: &Topology,
        solid: SolidId,
        cam: &Camera,
        opts: &RenderOpts,
    ) -> Result<RenderOutput, RenderError> {
        if let Some(reason) = self.failed.as_ref() {
            return Err(RenderError::DeviceLost(reason.clone()));
        }
        validate_session_size(opts.width, opts.height, self.max_texture_dimension_2d)?;
        // Tessellation runs before any GPU work so a tessellation failure
        // never touches (or poisons) GPU state.
        let mesh = RenderMesh::build(topo, solid, opts.deflection)?;
        let geometry = GeometryBuffers::new(&self.ctx.device, &mesh);
        let (output, _submit, _readback) = self.draw_frame(
            mesh.center,
            &geometry,
            !mesh.edge_vertices.is_empty(),
            cam,
            opts.width,
            opts.height,
            opts.edges,
            opts.background,
            opts.ambient,
        )?;
        Ok(output)
    }

    /// Prepare `solid` for repeated drawing: tessellate once at `deflection`
    /// and upload the geometry to this session's device.
    ///
    /// Returns an immutable [`PreparedSolid`] snapshot bound to this session.
    /// Preparing replacement geometry is always explicit — call `prepare`
    /// again after topology edits and drop (or overwrite) the old asset; the
    /// old asset keeps drawing its snapshot until released and never refreshes
    /// itself. See [`PreparedSolid`](crate::PreparedSolid) for the full asset contract.
    ///
    /// # Errors
    ///
    /// - [`RenderError::DeviceLost`] if the session is poisoned.
    /// - [`RenderError::Operations`] / [`RenderError::Topology`] /
    ///   [`RenderError::MeshData`] on tessellation failure (session stays usable).
    pub fn prepare(
        &mut self,
        topo: &Topology,
        solid: SolidId,
        deflection: f64,
    ) -> Result<PreparedSolid, RenderError> {
        if let Some(reason) = self.failed.as_ref() {
            return Err(RenderError::DeviceLost(reason.clone()));
        }
        // Tessellation runs before any GPU work so a tessellation failure
        // never touches (or poisons) GPU state.
        let t0 = Instant::now();
        let mesh = RenderMesh::build(topo, solid, deflection)?;
        let tessellate = t0.elapsed();
        let t0 = Instant::now();
        let geometry = GeometryBuffers::new(&self.ctx.device, &mesh);
        let upload = t0.elapsed();

        let has_edges = !mesh.edge_vertices.is_empty();
        let stats = PreparedStats {
            triangles: mesh.indices.len() / 3,
            vertices: mesh.vertices.len(),
            edge_segments: mesh.edge_vertices.len() / 2,
            vertex_bytes: mesh.vertices.len() * std::mem::size_of::<Vertex>(),
            index_bytes: mesh.indices.len() * std::mem::size_of::<u32>(),
            edge_bytes: mesh.edge_vertices.len() * std::mem::size_of::<EdgeVertex>(),
            deflection,
        };
        let asset = PreparedSolid {
            geometry,
            center: mesh.center,
            session_id: self.session_id,
            has_edges,
            stats,
        };
        self.prepare_calls += 1;
        self.last_prepare_timings = Some(PrepareTimings { tessellate, upload });
        Ok(asset)
    }

    /// Draw a [`PreparedSolid`] without tessellating or uploading geometry.
    ///
    /// The camera, size, edge toggle, background, and ambient may all differ
    /// from previous draws — none of them rebuilds the asset. The asset must
    /// have been prepared on this session; a foreign asset is refused with
    /// [`RenderError::WrongSession`] before any GPU work. See
    /// [`PreparedSolid`](crate::PreparedSolid) for the asset contract.
    ///
    /// # Errors
    ///
    /// - [`RenderError::DeviceLost`] if the session is poisoned.
    /// - [`RenderError::WrongSession`] if `asset` was prepared on another
    ///   session (session stays usable).
    /// - [`RenderError::InvalidSize`] / [`RenderError::PixelBudgetExceeded`] /
    ///   [`RenderError::SizeTooLarge`] on bad dimensions (session stays usable).
    /// - [`RenderError::BufferMap`] / [`RenderError::Poll`] on GPU readback
    ///   failure (poisons the session; the next call returns `DeviceLost`).
    pub fn draw_prepared(
        &mut self,
        asset: &PreparedSolid,
        cam: &Camera,
        opts: &PreparedDrawOpts,
    ) -> Result<RenderOutput, RenderError> {
        if let Some(reason) = self.failed.as_ref() {
            return Err(RenderError::DeviceLost(reason.clone()));
        }
        // Device-bound buffers must never cross devices: refuse before any
        // GPU work so a foreign asset can neither trip GPU validation nor
        // poison this session.
        if asset.session_id != self.session_id {
            return Err(RenderError::WrongSession {
                expected: asset.session_id,
                actual: self.session_id,
            });
        }
        validate_session_size(opts.width, opts.height, self.max_texture_dimension_2d)?;
        let (output, submit, readback) = self.draw_frame(
            asset.center,
            &asset.geometry,
            asset.has_edges,
            cam,
            opts.width,
            opts.height,
            opts.edges,
            opts.background,
            opts.ambient,
        )?;
        self.prepared_draw_calls += 1;
        self.last_draw_timings = Some(PreparedDrawTimings { submit, readback });
        Ok(output)
    }

    /// Submit a direct render asynchronously: tessellate and upload now (CPU),
    /// encode GPU work, and return a ticket without waiting for GPU execution
    /// or readback.
    ///
    /// The camera and `opts` are captured by value at submission (globals
    /// upload + scene encode happen inside this call); later mutation does not
    /// affect the pending frame. Only the outputs selected by `selection` are
    /// copied, mapped, and decoded on completion.
    ///
    /// # Errors
    ///
    /// - [`RenderError::DeviceLost`] if the session is poisoned.
    /// - [`RenderError::QueueFull`] when [`MAX_ASYNC_FRAMES`] frames are
    ///   already pending (session stays usable; collect or cancel one first).
    /// - [`RenderError::AsyncBudgetExceeded`] when retained bytes would exceed
    ///   [`MAX_ASYNC_IN_FLIGHT_BYTES`] (session stays usable).
    /// - [`RenderError::InvalidSize`] / [`RenderError::PixelBudgetExceeded`] /
    ///   [`RenderError::SizeTooLarge`] on bad dimensions (session stays usable).
    /// - [`RenderError::Operations`] / [`RenderError::Topology`] /
    ///   [`RenderError::MeshData`] on tessellation failure (session stays usable).
    pub fn submit_render(
        &mut self,
        topo: &Topology,
        solid: SolidId,
        cam: &Camera,
        opts: &RenderOpts,
        selection: ReadbackSelection,
    ) -> Result<FrameTicket, RenderError> {
        if let Some(reason) = self.failed.as_ref() {
            return Err(RenderError::DeviceLost(reason.clone()));
        }
        validate_session_size(opts.width, opts.height, self.max_texture_dimension_2d)?;
        self.check_queue_capacity()?;
        ensure_async_budget(self.in_flight_bytes, opts.width, opts.height, selection, 0)?;
        // Tessellation runs before any GPU work so a tessellation failure
        // never touches GPU state and never consumes a queue slot.
        let mesh = RenderMesh::build(topo, solid, opts.deflection)?;
        let geometry_bytes = mesh.vertices.len() * std::mem::size_of::<Vertex>()
            + mesh.indices.len() * std::mem::size_of::<u32>()
            + mesh.edge_vertices.len() * std::mem::size_of::<EdgeVertex>();
        ensure_async_budget(
            self.in_flight_bytes,
            opts.width,
            opts.height,
            selection,
            geometry_bytes,
        )?;
        let geometry = GeometryBuffers::new(&self.ctx.device, &mesh);
        Ok(self.submit_frame_inner(
            mesh.center,
            &geometry,
            !mesh.edge_vertices.is_empty(),
            geometry_bytes,
            cam,
            opts.width,
            opts.height,
            opts.edges,
            opts.background,
            opts.ambient,
            selection,
        ))
    }

    /// Submit a prepared-asset draw asynchronously: no tessellation or upload,
    /// encode GPU work from the asset's buffers, and return a ticket without
    /// waiting for GPU execution or readback.
    ///
    /// The camera and `opts` are captured by value at submission. The asset's
    /// GPU buffer handles are cloned into the pending frame, so dropping or
    /// replacing the asset afterwards does not affect the pending frame.
    ///
    /// # Errors
    ///
    /// - [`RenderError::DeviceLost`] if the session is poisoned.
    /// - [`RenderError::WrongSession`] if `asset` was prepared on another
    ///   session (session stays usable).
    /// - [`RenderError::QueueFull`] when [`MAX_ASYNC_FRAMES`] frames are
    ///   already pending (session stays usable).
    /// - [`RenderError::AsyncBudgetExceeded`] when retained bytes would exceed
    ///   [`MAX_ASYNC_IN_FLIGHT_BYTES`] (session stays usable).
    /// - [`RenderError::InvalidSize`] / [`RenderError::PixelBudgetExceeded`] /
    ///   [`RenderError::SizeTooLarge`] on bad dimensions (session stays usable).
    pub fn submit_prepared(
        &mut self,
        asset: &PreparedSolid,
        cam: &Camera,
        opts: &PreparedDrawOpts,
        selection: ReadbackSelection,
    ) -> Result<FrameTicket, RenderError> {
        if let Some(reason) = self.failed.as_ref() {
            return Err(RenderError::DeviceLost(reason.clone()));
        }
        if asset.session_id != self.session_id {
            return Err(RenderError::WrongSession {
                expected: asset.session_id,
                actual: self.session_id,
            });
        }
        validate_session_size(opts.width, opts.height, self.max_texture_dimension_2d)?;
        self.check_queue_capacity()?;
        let stats = asset.stats();
        let geometry_bytes = stats.vertex_bytes + stats.index_bytes + stats.edge_bytes;
        ensure_async_budget(
            self.in_flight_bytes,
            opts.width,
            opts.height,
            selection,
            geometry_bytes,
        )?;
        Ok(self.submit_frame_inner(
            asset.center,
            &asset.geometry,
            asset.has_edges,
            geometry_bytes,
            cam,
            opts.width,
            opts.height,
            opts.edges,
            opts.background,
            opts.ambient,
            selection,
        ))
    }

    /// Poll a submitted frame without blocking.
    ///
    /// Returns `Ok(None)` when the frame is still pending (GPU work or mapping
    /// not yet complete). Returns `Ok(Some(output))` once its requested
    /// outputs are decoded, removing the frame from the queue and releasing
    /// all but the decoded CPU bytes. The output carries its ticket, so color
    /// and face-id data stay attached to the right frame.
    ///
    /// This never waits: submission never concealed a synchronous readback
    /// wait, and neither does polling. Use [`OffscreenSession::wait_frame`]
    /// to block until a specific frame completes.
    ///
    /// # Errors
    ///
    /// - [`RenderError::WrongSession`] if `ticket` was issued by another
    ///   session (both sessions stay usable).
    /// - [`RenderError::UnknownTicket`] if `ticket` was already
    ///   collected/cancelled or was never issued on this session.
    /// - [`RenderError::DeviceLost`] if the session is poisoned (the pending
    ///   frame, if present, is released).
    /// - [`RenderError::BufferMap`] / [`RenderError::Poll`] on GPU completion
    ///   failure (poisons the session; the frame is released).
    pub fn poll_frame(
        &mut self,
        ticket: FrameTicket,
    ) -> Result<Option<AsyncFrameOutput>, RenderError> {
        self.check_ticket_session(ticket)?;
        if let Some(reason) = self.failed.clone() {
            // Fail closed but release the frame so callers can reclaim memory.
            if let Some(frame) = self.pending.remove(&ticket.frame_id()) {
                self.in_flight_bytes = self.in_flight_bytes.saturating_sub(frame.retained_bytes);
            }
            return Err(RenderError::DeviceLost(reason));
        }
        if !self.pending.contains_key(&ticket.frame_id()) {
            return Err(RenderError::UnknownTicket {
                ticket: ticket.frame_id(),
            });
        }
        // Initiate mapping on first poll (submission never maps).
        self.ensure_mapping_initiated(ticket.frame_id())?;
        // Non-blocking device poll drives mapping callbacks without waiting.
        // `wait` for a polled completion covers this poll only; the blocking
        // `wait_frame` path measures its own blocking wait separately.
        let t_wait = Instant::now();
        if let Err(e) = self.ctx.device.poll(wgpu::PollType::Poll) {
            let msg = e.to_string();
            self.failed = Some(msg.clone());
            if let Some(frame) = self.pending.remove(&ticket.frame_id()) {
                self.release_frame_bytes(frame.retained_bytes);
            }
            return Err(RenderError::Poll(msg));
        }
        let waited = t_wait.elapsed();
        if self.check_mapping_ready(ticket.frame_id())? {
            let mut output = self.decode_ready_frame(ticket.frame_id(), waited)?;
            // Preserve the non-blocking wait actually spent polling.
            output.timings.wait = waited;
            Ok(Some(output))
        } else {
            Ok(None)
        }
    }

    /// Block until a submitted frame completes, then decode and return its
    /// requested outputs.
    ///
    /// The frame is removed from the queue and its GPU resources released;
    /// only decoded CPU bytes are retained in the output. Tickets may be
    /// waited on in any order, including a different order from submission.
    ///
    /// # Errors
    ///
    /// Same as [`OffscreenSession::poll_frame`], but blocks instead of
    /// returning `Ok(None)`.
    pub fn wait_frame(&mut self, ticket: FrameTicket) -> Result<AsyncFrameOutput, RenderError> {
        self.check_ticket_session(ticket)?;
        if let Some(reason) = self.failed.clone() {
            if let Some(frame) = self.pending.remove(&ticket.frame_id()) {
                self.in_flight_bytes = self.in_flight_bytes.saturating_sub(frame.retained_bytes);
            }
            return Err(RenderError::DeviceLost(reason));
        }
        if !self.pending.contains_key(&ticket.frame_id()) {
            return Err(RenderError::UnknownTicket {
                ticket: ticket.frame_id(),
            });
        }
        self.ensure_mapping_initiated(ticket.frame_id())?;
        let t_wait = Instant::now();
        // Wait for GPU completion; mapping callbacks (and the `None` done
        // callback) complete as part of the poll. This waits for the most
        // recent submission, which is conservative when collecting out of
        // order (it also waits for later frames), but remains correct because
        // every pending frame owns exclusive resources.
        let poll_result = self.ctx.device.poll(wgpu::PollType::Wait {
            submission_index: None,
            timeout: None,
        });
        if let Err(e) = poll_result {
            let msg = e.to_string();
            self.failed = Some(msg.clone());
            if let Some(frame) = self.pending.remove(&ticket.frame_id()) {
                self.in_flight_bytes = self.in_flight_bytes.saturating_sub(frame.retained_bytes);
            }
            return Err(RenderError::Poll(msg));
        }
        let wait = t_wait.elapsed();
        // The blocking poll above drove the mapping callbacks (and the done
        // callback for `None`), so readiness here means success barring device
        // failure. Re-check shared slots without consuming: a mapping failure
        // poisons the session inside `check_mapping_ready`.
        if !self.check_mapping_ready(ticket.frame_id())? {
            // `Wait` on the frame's own submission index should have completed
            // it; a still-pending frame here means the device did not make
            // progress. Poison to fail closed rather than spinning.
            let msg = "async frame still pending after blocking wait".to_string();
            self.failed = Some(msg.clone());
            if let Some(frame) = self.pending.remove(&ticket.frame_id()) {
                self.release_frame_bytes(frame.retained_bytes);
            }
            return Err(RenderError::Poll(msg));
        }
        self.decode_ready_frame(ticket.frame_id(), wait)
    }

    /// Cancel a pending frame, releasing its textures, staging buffers, and
    /// retained geometry without mapping or decoding.
    ///
    /// Returns `Ok(true)` when a pending frame was released, `Ok(false)` when
    /// the ticket was already collected/cancelled or never issued (idempotent).
    /// Cancellation works even on a poisoned session so callers can always
    /// reclaim memory.
    ///
    /// # Errors
    ///
    /// [`RenderError::WrongSession`] if `ticket` was issued by another session.
    pub fn cancel_frame(&mut self, ticket: FrameTicket) -> Result<bool, RenderError> {
        self.check_ticket_session(ticket)?;
        if let Some(frame) = self.pending.remove(&ticket.frame_id()) {
            self.in_flight_bytes = self.in_flight_bytes.saturating_sub(frame.retained_bytes);
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn check_queue_capacity(&self) -> Result<(), RenderError> {
        if self.pending.len() >= MAX_ASYNC_FRAMES {
            return Err(RenderError::QueueFull {
                pending: self.pending.len(),
                max: MAX_ASYNC_FRAMES,
            });
        }
        Ok(())
    }

    fn check_ticket_session(&self, ticket: FrameTicket) -> Result<(), RenderError> {
        if ticket.session_id() != self.session_id {
            return Err(RenderError::WrongSession {
                expected: ticket.session_id(),
                actual: self.session_id,
            });
        }
        Ok(())
    }

    /// Encode one async frame from already-uploaded `geometry` and enqueue it.
    ///
    /// Never waits: no `device.poll`, no `map_async` (except the
    /// `on_submitted_work_done` registration for `None`), no CPU decode. Each
    /// frame owns exclusive textures and staging buffers, so later submissions
    /// cannot overwrite earlier frames. Returns the ticket immediately.
    #[allow(clippy::too_many_arguments)]
    fn submit_frame_inner(
        &mut self,
        center: Point3,
        geometry: &GeometryBuffers,
        has_edges: bool,
        geometry_bytes: usize,
        cam: &Camera,
        width: u32,
        height: u32,
        edges: bool,
        background: [f32; 4],
        ambient: f32,
        selection: ReadbackSelection,
    ) -> FrameTicket {
        let t0 = Instant::now();
        let extent = wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        };
        let color_tex = self.ctx.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("async color target"),
            size: extent,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: COLOR_FORMAT_OFFSCREEN,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let depth_tex = self.ctx.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("async depth target"),
            size: extent,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let id_tex = self.ctx.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("async id target"),
            size: extent,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: ID_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let color_view = color_tex.create_view(&wgpu::TextureViewDescriptor::default());
        let depth_view = depth_tex.create_view(&wgpu::TextureViewDescriptor::default());
        let id_view = id_tex.create_view(&wgpu::TextureViewDescriptor::default());

        let color_padded_bpr = padded_bytes_per_row(width, 4);
        let id_padded_bpr = padded_bytes_per_row(width, 4);
        let color_buf = selection.wants_color().then(|| {
            self.ctx.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("async color readback"),
                size: u64::from(color_padded_bpr) * u64::from(height),
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            })
        });
        let id_buf = selection.wants_ids().then(|| {
            self.ctx.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("async id readback"),
                size: u64::from(id_padded_bpr) * u64::from(height),
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            })
        });

        let with_edges = edges && has_edges;
        let pipelines = if with_edges {
            &self.pipelines_with_edges
        } else {
            &self.pipelines_without_edges
        };

        let globals = build_globals(cam, center, ambient);
        self.globals.upload(&self.ctx.queue, &globals);

        let mut encoder = self
            .ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("async encoder"),
            });
        encode_scene(
            &mut encoder,
            pipelines,
            &self.globals,
            geometry,
            &PassTargets {
                color: &color_view,
                id: &id_view,
                depth: &depth_view,
                background,
            },
        );
        if let Some(buf) = color_buf.as_ref() {
            encoder.copy_texture_to_buffer(
                wgpu::TexelCopyTextureInfo {
                    texture: &color_tex,
                    mip_level: 0,
                    origin: wgpu::Origin3d::ZERO,
                    aspect: wgpu::TextureAspect::All,
                },
                wgpu::TexelCopyBufferInfo {
                    buffer: buf,
                    layout: wgpu::TexelCopyBufferLayout {
                        offset: 0,
                        bytes_per_row: Some(color_padded_bpr),
                        rows_per_image: Some(height),
                    },
                },
                extent,
            );
        }
        if let Some(buf) = id_buf.as_ref() {
            encoder.copy_texture_to_buffer(
                wgpu::TexelCopyTextureInfo {
                    texture: &id_tex,
                    mip_level: 0,
                    origin: wgpu::Origin3d::ZERO,
                    aspect: wgpu::TextureAspect::All,
                },
                wgpu::TexelCopyBufferInfo {
                    buffer: buf,
                    layout: wgpu::TexelCopyBufferLayout {
                        offset: 0,
                        bytes_per_row: Some(id_padded_bpr),
                        rows_per_image: Some(height),
                    },
                },
                extent,
            );
        }
        let _submission_index = self.ctx.queue.submit(Some(encoder.finish()));
        let submit = t0.elapsed();

        // Retain geometry handles so the GPU draw stays valid even when the
        // caller drops or replaces the prepared asset.
        let mut retained_geometry = vec![geometry.vertex.clone(), geometry.index.clone()];
        if let Some(edge) = geometry.edge.as_ref() {
            retained_geometry.push(edge.clone());
        }

        let frame_id = self.next_frame_id;
        self.next_frame_id += 1;
        let ticket = FrameTicket::new(self.session_id, frame_id, width, height, selection);

        let retained_bytes = frame_retained_bytes(width, height, selection, geometry_bytes);

        // For `None`, completion is GPU execution only: register a done
        // callback now (submission never waits for it).
        let done_flag = if matches!(selection, ReadbackSelection::None) {
            let flag = Arc::new(AtomicBool::new(false));
            let flag_clone = Arc::clone(&flag);
            self.ctx.queue.on_submitted_work_done(move || {
                mark_done(&flag_clone);
            });
            Some(flag)
        } else {
            None
        };

        let frame = PendingFrame {
            ticket,
            selection,
            width,
            height,
            extent,
            color_tex,
            depth_tex,
            id_tex,
            color_buf,
            id_buf,
            color_padded_bpr,
            id_padded_bpr,
            retained_geometry,
            geometry_bytes,
            retained_bytes,
            submit_duration: submit,
            mapping: PendingMapping::Unmapped,
            done_flag,
        };
        self.in_flight_bytes += retained_bytes;
        if self.in_flight_bytes > self.peak_in_flight_bytes {
            self.peak_in_flight_bytes = self.in_flight_bytes;
        }
        self.pending.insert(frame_id, frame);
        ticket
    }

    /// Initiate `map_async` for a pending frame's requested buffers, once.
    ///
    /// Submission never maps; the first poll/wait does. Later polls read the
    /// same shared slots so buffers are never double-mapped and partial
    /// completion is never lost (slots are peekable, unlike channels).
    fn ensure_mapping_initiated(&mut self, frame_id: u64) -> Result<(), RenderError> {
        let Some(frame) = self.pending.get_mut(&frame_id) else {
            return Err(RenderError::UnknownTicket { ticket: frame_id });
        };
        if !matches!(frame.mapping, PendingMapping::Unmapped) {
            return Ok(());
        }
        if matches!(frame.selection, ReadbackSelection::None) {
            // Done callback was registered at submission; nothing to map.
            return Ok(());
        }
        let mut color_slot = None;
        let mut ids_slot = None;
        if let Some(buf) = frame.color_buf.as_ref() {
            let slot = shared_map_slot();
            let slot_clone = Arc::clone(&slot);
            buf.slice(..).map_async(wgpu::MapMode::Read, move |res| {
                write_map_slot(&slot_clone, res.map_err(|e| e.to_string()));
            });
            color_slot = Some(slot);
        }
        if let Some(buf) = frame.id_buf.as_ref() {
            let slot = shared_map_slot();
            let slot_clone = Arc::clone(&slot);
            buf.slice(..).map_async(wgpu::MapMode::Read, move |res| {
                write_map_slot(&slot_clone, res.map_err(|e| e.to_string()));
            });
            ids_slot = Some(slot);
        }
        frame.mapping = PendingMapping::Mapping {
            color: color_slot,
            ids: ids_slot,
        };
        Ok(())
    }

    /// Release a pending frame's retained-memory accounting (textures, staging,
    /// geometry) after removal.
    fn release_frame_bytes(&mut self, retained_bytes: u64) {
        self.in_flight_bytes = self.in_flight_bytes.saturating_sub(retained_bytes);
    }

    /// Decode a ready frame: copy mapped ranges, unmap, build CPU outputs,
    /// release GPU resources, and account transferred bytes.
    ///
    /// The caller guarantees every requested mapping succeeded (slots hold
    /// `Some(Ok(()))`, or the frame is `None`-selection with its done flag
    /// set). Any `get_mapped_range` failure poisons the session.
    fn decode_ready_frame(
        &mut self,
        frame_id: u64,
        wait: std::time::Duration,
    ) -> Result<AsyncFrameOutput, RenderError> {
        let Some(frame) = self.pending.remove(&frame_id) else {
            return Err(RenderError::UnknownTicket { ticket: frame_id });
        };
        let retained = frame.retained_bytes;
        let ticket = frame.ticket;
        let width = frame.width;
        let height = frame.height;
        let color_padded_bpr = frame.color_padded_bpr;
        let id_padded_bpr = frame.id_padded_bpr;
        let submit = frame.submit_duration;
        let t_decode = Instant::now();

        let mut transferred_bytes: u64 = 0;
        let color = if let Some(buf) = frame.color_buf.as_ref() {
            let data = match buf.slice(..).get_mapped_range() {
                Ok(range) => range.to_vec(),
                Err(e) => {
                    let msg = e.to_string();
                    self.failed = Some(msg.clone());
                    self.release_frame_bytes(retained);
                    return Err(RenderError::BufferMap(msg));
                }
            };
            buf.unmap();
            transferred_bytes += u64::from(color_padded_bpr) * u64::from(height);
            Some(unpad_to_rgba(&data, width, height, color_padded_bpr))
        } else {
            None
        };
        let id_buffer = if let Some(buf) = frame.id_buf.as_ref() {
            let data = match buf.slice(..).get_mapped_range() {
                Ok(range) => range.to_vec(),
                Err(e) => {
                    let msg = e.to_string();
                    self.failed = Some(msg.clone());
                    self.release_frame_bytes(retained);
                    return Err(RenderError::BufferMap(msg));
                }
            };
            buf.unmap();
            transferred_bytes += u64::from(id_padded_bpr) * u64::from(height);
            Some(unpad_to_u32(&data, width, height, id_padded_bpr))
        } else {
            None
        };
        // `None`-selection frames have neither buffer; they complete with
        // empty outputs once the done flag is set (checked by the caller).
        let decode = t_decode.elapsed();
        let timings = AsyncFrameTimings {
            submit,
            wait,
            decode,
        };
        let output = AsyncFrameOutput {
            ticket,
            width,
            height,
            color,
            id_buffer,
            transferred_bytes,
            timings,
        };
        self.release_frame_bytes(retained);
        self.total_transferred_bytes += transferred_bytes;
        Ok(output)
    }

    /// Check a pending frame's shared mapping slots without consuming them.
    ///
    /// Returns `Ok(true)` when every requested buffer mapped successfully,
    /// `Ok(false)` when at least one callback has not fired yet, and
    /// `Err` (poisoning the session) when any mapping failed.
    fn check_mapping_ready(&mut self, frame_id: u64) -> Result<bool, RenderError> {
        let Some(frame) = self.pending.get(&frame_id) else {
            return Err(RenderError::UnknownTicket { ticket: frame_id });
        };
        // `None`-selection frames never map; readiness is the done flag,
        // checked by the caller (needs no slot read).
        if matches!(frame.selection, ReadbackSelection::None) {
            return Ok(frame.done_flag.as_ref().is_some_and(is_done));
        }
        let (color_state, ids_state) = match &frame.mapping {
            PendingMapping::Unmapped => (None, None),
            PendingMapping::Mapping { color, ids } => (
                color.as_ref().map(read_map_slot),
                ids.as_ref().map(read_map_slot),
            ),
        };
        // A requested buffer with no slot means it was never initiated, which
        // cannot happen after `ensure_mapping_initiated`; treat as pending.
        // `None` outer (no buffer requested) is trivially ready.
        let mut pending = false;
        if let Some(slot) = frame.color_buf.as_ref().map(|_| color_state) {
            match slot {
                Some(Some(Ok(()))) => {}
                Some(Some(Err(msg))) => {
                    self.failed = Some(msg.clone());
                    if let Some(f) = self.pending.remove(&frame_id) {
                        self.release_frame_bytes(f.retained_bytes);
                    }
                    return Err(RenderError::BufferMap(msg));
                }
                Some(None) | None => pending = true,
            }
        }
        if let Some(slot) = frame.id_buf.as_ref().map(|_| ids_state) {
            match slot {
                Some(Some(Ok(()))) => {}
                Some(Some(Err(msg))) => {
                    self.failed = Some(msg.clone());
                    if let Some(f) = self.pending.remove(&frame_id) {
                        self.release_frame_bytes(f.retained_bytes);
                    }
                    return Err(RenderError::BufferMap(msg));
                }
                Some(None) | None => pending = true,
            }
        }
        Ok(!pending)
    }

    /// Draw one frame from already-uploaded `geometry` (synchronous path).
    ///
    /// Shared verbatim by [`OffscreenSession::render`] (which tessellates and
    /// uploads first) and [`OffscreenSession::draw_prepared`] (which reuses a
    /// prepared asset), so the two paths cannot drift. Returns the output plus
    /// the submit-phase and readback-phase durations for benchmarking.
    ///
    /// `SizeTooLarge` leaves the session usable; GPU readback failures poison
    /// it (the next call on any path returns `DeviceLost`).
    #[allow(clippy::too_many_arguments)]
    fn draw_frame(
        &mut self,
        center: Point3,
        geometry: &GeometryBuffers,
        has_edges: bool,
        cam: &Camera,
        width: u32,
        height: u32,
        edges: bool,
        background: [f32; 4],
        ambient: f32,
    ) -> Result<(RenderOutput, std::time::Duration, std::time::Duration), RenderError> {
        let needs_rebuild = self
            .targets
            .as_ref()
            .is_none_or(|t| t.width != width || t.height != height);
        if needs_rebuild {
            // Release the old textures and readback buffers before allocating
            // a replacement near the adapter's memory limit.
            drop(self.targets.take());
            self.targets = Some(CachedTargets::build(&self.ctx.device, width, height));
            self.target_rebuilds += 1;
        }
        let targets = self.targets.as_ref().ok_or_else(|| {
            RenderError::DeviceLost("session targets missing after rebuild".into())
        })?;

        let with_edges = edges && has_edges;
        let pipelines = if with_edges {
            &self.pipelines_with_edges
        } else {
            &self.pipelines_without_edges
        };

        let t0 = Instant::now();
        let globals = build_globals(cam, center, ambient);
        self.globals.upload(&self.ctx.queue, &globals);

        let mut encoder = self
            .ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("session encoder"),
            });
        encode_scene(
            &mut encoder,
            pipelines,
            &self.globals,
            geometry,
            &PassTargets {
                color: &targets.color_view,
                id: &targets.id_view,
                depth: &targets.depth_view,
                background,
            },
        );
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &targets.color_tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &targets.color_readback,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(targets.color_padded_bpr),
                    rows_per_image: Some(height),
                },
            },
            targets.extent,
        );
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &targets.id_tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &targets.id_readback,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(targets.id_padded_bpr),
                    rows_per_image: Some(height),
                },
            },
            targets.extent,
        );
        self.ctx.queue.submit(Some(encoder.finish()));
        let submit = t0.elapsed();

        // Map + read. A device-level failure poisons the session; validation
        // and tessellation failures above never reach here, so they never poison.
        let t0 = Instant::now();
        let color_bytes = match map_and_read(&self.ctx.device, &targets.color_readback) {
            Ok(bytes) => bytes,
            Err(e) => {
                self.failed = Some(e.to_string());
                return Err(e);
            }
        };
        let id_bytes = match map_and_read(&self.ctx.device, &targets.id_readback) {
            Ok(bytes) => bytes,
            Err(e) => {
                self.failed = Some(e.to_string());
                return Err(e);
            }
        };

        let color = unpad_to_rgba(&color_bytes, width, height, targets.color_padded_bpr);
        let id_buffer = unpad_to_u32(&id_bytes, width, height, targets.id_padded_bpr);
        let readback = t0.elapsed();

        Ok((
            RenderOutput {
                color,
                id_buffer,
                width,
                height,
            },
            submit,
            readback,
        ))
    }
}

fn frame_retained_bytes(
    width: u32,
    height: u32,
    selection: ReadbackSelection,
    geometry_bytes: usize,
) -> u64 {
    let pixels = u64::from(width) * u64::from(height);
    let staging_per_output = u64::from(padded_bytes_per_row(width, 4)) * u64::from(height);
    let outputs = u64::from(selection.wants_color()) + u64::from(selection.wants_ids());
    pixels
        .saturating_mul(12)
        .saturating_add(staging_per_output.saturating_mul(outputs))
        .saturating_add(u64::try_from(geometry_bytes).unwrap_or(u64::MAX))
}

fn ensure_async_budget(
    in_flight: u64,
    width: u32,
    height: u32,
    selection: ReadbackSelection,
    geometry_bytes: usize,
) -> Result<(), RenderError> {
    let requested = frame_retained_bytes(width, height, selection, geometry_bytes);
    if requested > MAX_ASYNC_IN_FLIGHT_BYTES || in_flight > MAX_ASYNC_IN_FLIGHT_BYTES - requested {
        return Err(RenderError::AsyncBudgetExceeded {
            in_flight,
            requested,
            max: MAX_ASYNC_IN_FLIGHT_BYTES,
        });
    }
    Ok(())
}

fn validate_session_size(
    width: u32,
    height: u32,
    max_texture_dimension_2d: u32,
) -> Result<(), RenderError> {
    if width == 0 || height == 0 {
        return Err(RenderError::InvalidSize { width, height });
    }
    let pixels = u64::from(width) * u64::from(height);
    if pixels > MAX_OFFSCREEN_PIXELS {
        return Err(RenderError::PixelBudgetExceeded {
            pixels,
            max: MAX_OFFSCREEN_PIXELS,
        });
    }
    if width > max_texture_dimension_2d || height > max_texture_dimension_2d {
        return Err(RenderError::SizeTooLarge {
            width,
            height,
            max: max_texture_dimension_2d,
        });
    }
    Ok(())
}

#[cfg(test)]
mod async_budget_tests {
    use super::*;

    #[test]
    fn cumulative_budget_rejects_before_a_second_large_frame() {
        let first = frame_retained_bytes(4096, 4096, ReadbackSelection::Both, 0);
        assert_eq!(first, 320 * 1024 * 1024);
        assert!(ensure_async_budget(0, 4096, 4096, ReadbackSelection::Both, 0).is_ok());
        assert!(matches!(
            ensure_async_budget(first, 4096, 4096, ReadbackSelection::Both, 0),
            Err(RenderError::AsyncBudgetExceeded {
                in_flight,
                requested,
                max,
            }) if in_flight == first && requested == first && max == MAX_ASYNC_IN_FLIGHT_BYTES
        ));
        assert!(ensure_async_budget(0, 4096, 4096, ReadbackSelection::Both, 0).is_ok());
    }

    #[test]
    fn selection_and_geometry_are_counted_without_overflow() {
        let none = frame_retained_bytes(4096, 4096, ReadbackSelection::None, 0);
        assert_eq!(none, 192 * 1024 * 1024);
        assert!(matches!(
            ensure_async_budget(none * 2, 4096, 4096, ReadbackSelection::None, 0),
            Err(RenderError::AsyncBudgetExceeded { .. })
        ));
        assert!(matches!(
            ensure_async_budget(0, 1, 1, ReadbackSelection::None, usize::MAX),
            Err(RenderError::AsyncBudgetExceeded { .. })
        ));
    }
}
