# Rendering

`brepkit-render` is an optional native wgpu renderer. It can render a solid to
an RGBA buffer, produce a face-ID buffer for picking, and generate analytic
cylinder meshes with screen-space level of detail.

Offscreen rendering requires either a hardware adapter or a software backend.
Linux CI uses Mesa lavapipe through Vulkan. Applications should surface
`RenderError::NoAdapter` rather than treating an absent adapter as a blank
image.

Rendering is downstream of exact geometry: it tessellates a solid for display
but does not modify its B-Rep. Camera, output dimensions, and render options are
explicit. Validate requested image dimensions because they are bounded by the
adapter's maximum texture size.

The renderer is not part of `brepkit-wasm`; browser consumers provide their
own WebGL/WebGPU scene integration from tessellated geometry.
