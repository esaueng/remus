# Data Exchange

| Format | Geometry | Import | Export | Notes |
| --- | --- | --- | --- | --- |
| STEP | Exact B-Rep | Yes | Yes | Preferred for analytic/NURBS interchange |
| IGES | B-Rep | Preview | Lossy | Experimental subset |
| STL | Triangle mesh | Yes | Yes | Binary and ASCII |
| 3MF | Triangle mesh | Yes | Yes | ZIP container; multiple objects on import |
| OBJ | Triangle mesh | Yes | Yes | Geometry only |
| PLY | Triangle mesh | Yes | Yes | ASCII and binary little-endian |
| GLB | Triangle mesh | Yes | Yes | No materials or scene graph |

STEP preserves planes, cylinders, cones, spheres, tori, NURBS surfaces, and
supported curve types. Mesh imports reconstruct planar triangle faces; they do
not recover the original analytic surfaces.

Every public reader uses `ImportLimits::default()` to bound encoded input and
entity counts. Use the corresponding `*_with_limits` entry point for a stricter
service boundary. Arena deserialization has the same production limits and
should be treated as a debug/replay format, not long-term interchange.

## Round-trip verification

For exact formats, export, import into a new `Topology`, then validate the
solid and compare volume, bounding box, and surface types. For mesh formats,
also check index bounds, manifoldness, and a deflection-appropriate geometric
error. A successful writer call alone is not evidence of a valid round trip.
