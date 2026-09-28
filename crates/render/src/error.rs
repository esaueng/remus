//! Error type for the offscreen renderer.

/// Errors that can occur while rendering a solid offscreen.
#[derive(Debug, thiserror::Error)]
pub enum RenderError {
    /// No wgpu adapter could be obtained (neither a real GPU nor a software
    /// fallback). The crate is still usable on machines that do provide one.
    #[error("no wgpu adapter available (tried real GPU then software fallback): {0}")]
    NoAdapter(String),

    /// The adapter could not provide a device/queue matching the request.
    #[error("failed to request wgpu device: {0}")]
    DeviceRequest(String),

    /// A GPU buffer could not be mapped for readback.
    #[error("failed to map GPU buffer for readback: {0}")]
    BufferMap(String),

    /// Polling the device for readback completion failed.
    #[error("failed to poll wgpu device: {0}")]
    Poll(String),

    /// The session's device was lost (or a readback failed) and the session is
    /// poisoned: subsequent renders on this session fail without touching the
    /// GPU. Create a new session to recover. The first failure surfaces as
    /// [`RenderError::BufferMap`] / [`RenderError::Poll`]; later calls surface
    /// this variant. Real device loss is not synthesized in tests — see
    /// [`crate::OffscreenSession::inject_device_loss_for_test`].
    #[error("render session device lost (session poisoned): {0}")]
    DeviceLost(String),

    /// A prepared-render asset was drawn on a different session than the one
    /// that prepared it. GPU buffers are device-bound, so the draw is refused
    /// before touching the GPU; the refusing session stays usable. Prepare on
    /// the drawing session (or draw on the preparing session) instead.
    /// `expected` is the preparing session's id
    /// ([`OffscreenSession::session_id`](crate::OffscreenSession::session_id));
    /// `actual` is the drawing session's id.
    #[error(
        "prepared asset belongs to session {expected}, not this session ({actual}); prepare on the drawing session instead"
    )]
    WrongSession {
        /// Id of the session that prepared the asset.
        expected: u64,
        /// Id of the session asked to draw it.
        actual: u64,
    },

    /// The requested render dimensions were invalid (zero width or height).
    #[error("invalid render size: width and height must be non-zero, got {width}x{height}")]
    InvalidSize {
        /// Requested width in pixels.
        width: u32,
        /// Requested height in pixels.
        height: u32,
    },

    /// The requested render dimensions exceed the adapter's maximum 2D texture
    /// size, so a render would fail GPU validation.
    #[error(
        "render size {width}x{height} exceeds the device limit of {max}x{max} (max 2D texture dimension)"
    )]
    SizeTooLarge {
        /// Requested width in pixels.
        width: u32,
        /// Requested height in pixels.
        height: u32,
        /// The adapter's `max_texture_dimension_2d`.
        max: u32,
    },

    /// The requested render dimensions exceed the renderer's total-pixel
    /// budget, so the target buffers are never allocated.
    #[error("render size {pixels} pixels exceeds the offscreen budget of {max} pixels")]
    PixelBudgetExceeded {
        /// Requested total pixel count (`width * height`).
        pixels: u64,
        /// The renderer's offscreen pixel budget.
        max: u64,
    },

    /// The tessellation produced a mesh that violates a renderer invariant
    /// (e.g. an index buffer length not divisible by 3, an out-of-range vertex
    /// index, or grouped face offsets that do not cover every triangle).
    #[error("malformed tessellation mesh: {0}")]
    MeshData(String),

    /// The windowing event loop could not be created or run (viewer only).
    #[error("windowing event loop error: {0}")]
    EventLoop(String),

    /// A window surface could not be created or configured (viewer only).
    #[error("failed to create or configure window surface: {0}")]
    SurfaceConfig(String),

    /// Tessellation of the input solid failed.
    #[error(transparent)]
    Operations(#[from] remus_operations::OperationsError),

    /// Topology traversal of the input solid failed.
    #[error(transparent)]
    Topology(#[from] remus_topology::TopologyError),
}
