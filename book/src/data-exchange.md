# Data Exchange

| Format | Geometry | Import | Export | Notes |
| --- | --- | --- | --- | --- |
| STEP | Exact B-Rep | Yes | Yes | Analytic and rational NURBS interchange; configurable file/product metadata |
| IGES | B-Rep | Preview | Lossy | Experimental subset |
| STL | Triangle mesh | Yes | Yes | Binary and ASCII |
| 3MF | Triangle mesh | Yes | Yes | ZIP container; multiple objects on import |
| OBJ | Triangle mesh | Yes | Yes | Geometry only |
| PLY | Triangle mesh | Yes | Yes | ASCII and binary little-endian |
| GLB | Triangle mesh | Yes | Yes | No materials or scene graph |

STEP preserves planes, cylinders, cones, spheres, tori, NURBS surfaces, and
supported curve types, including rational curve and surface weights. Rust
callers can use `write_step_with_options` with `StepWriteOptions` to set the
product name, file name, and timestamp. WASM callers use
`exportStepWithOptions` or `exportStepMultiWithOptions` and pass the same fields
as a JSON string. Mesh imports reconstruct planar triangle faces; they do not
recover the original analytic surfaces.

Every public reader uses `ImportLimits::default()` to bound encoded input and
entity counts. Use the corresponding `*_with_limits` entry point for a stricter
service boundary. Arena deserialization has the same production limits and
should be treated as a debug/replay format, not long-term interchange.

## Edge orientation on STEP import

remus's topology has no edge "sense" flag. An `Edge` owns its curve outright,
and every consumer assumes the stored parameterization runs start → end; the
STEP writer relies on the same invariant, which is why it always emits
`same_sense = .T.`.

ISO 10303-42 does not require that. An `EDGE_CURVE` may run against its curve's
parameterization and say so with `same_sense = .F.`, which real CATIA, NX and
SolidWorks exports use freely. The reader canonicalizes those edges on import by
reversing the curve itself, so the invariant above holds for imported geometry
too.

Reversal is only load-bearing where the two vertices do not pin down the
traversal on their own:

- **Circles and ellipses** are periodic. Both endpoints lie on the curve twice
  over, so a `.F.` arc is indistinguishable from its complement without the
  flag. These are reversed.
- **NURBS curves** are reversed as well. An open sub-span recovers its direction
  by projecting both endpoints, but an edge spanning the curve's whole domain
  matches its natural ends in either orientation, so a `.F.` edge would
  otherwise be sampled backwards.
- **Lines** are interpolated between the vertices and carry no direction of
  their own.
- **Hyperbolas and parabolas** are unbounded and never closed. Their endpoint
  projection is exact and returns a reversed span as-is, so they already trace
  start → end.

Reversing a circle or ellipse negates its `v_axis` and normal but leaves
`u_axis` alone, so `evaluate(0.0)` — the seam of a closed edge — does not move.
A closed `.F.` edge therefore changes its winding without changing its phase.

Files written by remus are unaffected, since the writer never emits `.F.`.
Files from other CAD systems that contain `.F.` conic edges import differently
than they did before this behaviour existed: previously such arcs were built as
the complement sweep, which left face, edge and vertex counts correct while
deforming the solid.

## Round-trip verification

For exact formats, export, import into a new `Topology`, then validate the
solid and compare volume, bounding box, and surface types. For mesh formats,
also check index bounds, manifoldness, and a deflection-appropriate geometric
error. A successful writer call alone is not evidence of a valid round trip.
