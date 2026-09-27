//! Reusable offscreen rendering session (PERF-R01).
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
//!   the size stays constant.
//! - Rebuilt per frame: tessellation, GPU vertex/index/edge buffers, globals
//!   contents (camera, ambient), and the clear color. Targets are recreated
//!   when `width`/`height` change.
//! - Explicitly out of scope (see PERF-R02/R04): persistent geometry caching,
//!   incremental tessellation, asynchronous readback, compute-mesher changes,
//!   and window-viewer changes.
//!
//! # Failure policy
//!
//! Validation and tessellation failures (`InvalidSize`,
//! `PixelBudgetExceeded`, `SizeTooLarge`, `MeshData`, `Operations`,
//! `Topology`) leave the session usable: cached targets are untouched and a
//! later valid render succeeds.
//!
//! GPU execution failures (`BufferMap`, `Poll`) poison the session: the first
//! failure is returned as-is and the session records the reason; every later
//! [`OffscreenSession::render`] returns [`RenderError::DeviceLost`] without
//! touching the GPU. Create a new session to recover. Real device loss is not
//! synthesized in tests; use
//! [`OffscreenSession::inject_device_loss_for_test`] to exercise the poisoned
//! path. No claim is made about recovering a physically lost device.
//!
//! # Resource release
//!
//! Dropping the session drops its device, pipelines, globals, and cached
//! target handles. On a size change, old target handles are dropped before
//! allocating replacements. The GPU may defer physical reclamation until
//! submitted work completes.

use remus_topology::Topology;
use remus_topology::solid::SolidId;

use crate::camera::Camera;
use crate::error::RenderError;
use crate::mesh::RenderMesh;
use crate::pipeline::{
    COLOR_FORMAT_OFFSCREEN, DEPTH_FORMAT, GeometryBuffers, Globals, GlobalsBinding, GpuContext,
    ID_FORMAT, PassTargets, Pipelines, build_globals, encode_scene, map_and_read,
    padded_bytes_per_row, unpad_to_rgba, unpad_to_u32,
};
use crate::{MAX_OFFSCREEN_PIXELS, RenderOpts, RenderOutput};

/// Cached offscreen targets for one `width` x `height` size.
///
/// Textures, views, and readback buffers are recreated only when the requested
/// size changes; every frame clears them (`LoadOp::Clear` in
/// [`encode_scene`]), so no previous frame's color, ids, or depth leak into
/// the next render.
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
/// adapter/device/pipeline setup the convenience path pays every time.
///
/// A session is bound to a single device and the offscreen color format. It
/// is `!Sync` (it takes `&mut self` per frame) and must be created on the
/// thread that renders; do not share one across threads.
pub struct OffscreenSession {
    ctx: GpuContext,
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
    failed: Option<String>,
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
    /// [`GpuContext::new`] path the convenience API uses.
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
            adapter_info,
            max_texture_dimension_2d,
            pipeline_layout,
            globals,
            pipelines_with_edges,
            pipelines_without_edges,
            targets: None,
            target_rebuilds: 0,
            failed: None,
        })
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
    /// After this call, [`OffscreenSession::render`] returns `DeviceLost`
    /// until the session is dropped and recreated. This is a test hook: it
    /// makes no claim about recovering a physically lost device.
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
        validate_session_size(opts.width, opts.height)?;
        // Tessellation runs before any GPU work so a tessellation failure
        // never touches (or poisons) GPU state.
        let mesh = RenderMesh::build(topo, solid, opts.deflection)?;
        let (width, height) = (opts.width, opts.height);
        if width > self.max_texture_dimension_2d || height > self.max_texture_dimension_2d {
            return Err(RenderError::SizeTooLarge {
                width,
                height,
                max: self.max_texture_dimension_2d,
            });
        }

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

        let with_edges = opts.edges && !mesh.edge_vertices.is_empty();
        let pipelines = if with_edges {
            &self.pipelines_with_edges
        } else {
            &self.pipelines_without_edges
        };

        let globals = build_globals(cam, mesh.center, opts.ambient);
        self.globals.upload(&self.ctx.queue, &globals);
        let geometry = GeometryBuffers::new(&self.ctx.device, &mesh);

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
            &geometry,
            &PassTargets {
                color: &targets.color_view,
                id: &targets.id_view,
                depth: &targets.depth_view,
                background: opts.background,
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

        // Map + read. A device-level failure poisons the session; validation
        // and tessellation failures above never reach here, so they never poison.
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

        Ok(RenderOutput {
            color,
            id_buffer,
            width,
            height,
        })
    }
}

fn validate_session_size(width: u32, height: u32) -> Result<(), RenderError> {
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
    Ok(())
}
