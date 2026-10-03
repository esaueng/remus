# P-Class 7.4: directional curve projection — contract

Status: qualified M5 implementation in `crates/operations/src/project_curve.rs`,
with acceptance oracles in `crates/operations/tests/qualify_project_curve.rs`
and regressions in `crates/operations/tests/regress_project_curve.rs`.
Expected values, refusal codes and thresholds remain fixed; the current
qualification limits are stated in §§5, 6 and 8.

Consumers: OpenZCAD S-6 (project model edges into the active sketch), 6.6
linked sketch references, and later B73 wrap/emboss. 7.4's other half,
surface extension, is not covered here.

## 1. Scope of slice 1

| Source edge | Plane target | Cylinder / cone / sphere target | Torus, NURBS target |
|---|---|---|---|
| Line segment | exact (line) | exact (conic) | refused |
| Circle / arc | exact (circle or ellipse) | exact if coaxial, else approximate on request | refused |
| Ellipse | exact (ellipse or circle) | refused | refused |
| NURBS, parabola, hyperbola | refused for faces; exact on sketch planes (§3.3) | refused | refused |

"Approximate on request" means `allow_approximate = true`, within the
certified cylinder-slab trim cell in §6; unqualified trims refuse. The default
refuses (§6). Torus and NURBS targets are refused in every combination:
a plane section of a torus is quartic and a NURBS section needs marching,
and neither has a consumer in this slice.

## 2. Definitions

**Direction.** `d` is any finite non-zero vector; `d̂ = d/|d|`. Projection
is one-sided: a source point `p` maps to `q = p + λ·d̂` with `λ ≥ 0`.
Callers who want the other side negate `d`.

**First hit.** For a face `F`, `q(p) = p + λ*·d̂` where `λ*` is the smallest
`λ ≥ 0` with `p + λ·d̂ ∈ F` (the trimmed region, boundary included). Points
whose ray never meets `F` have no image. On a solid, `F` is the union of all
faces (outer and inner shells, `solid_faces`); each image point belongs to
the face it first hits. Hidden sheets (the back of a cylinder) never appear.

**Source parameter and order.** The source edge is traversed start → end in
its own domain: `[0, 1]` for a line, its trim interval for a circle or
ellipse. Result pieces are ordered by increasing source parameter. Each
piece records `source_range = (s0, s1)`, `s0 < s1`, and its start vertex is
the image of `s0`. For a closed source, ranges are taken modulo the period:
`s0` lies in `[t0, t0 + 2π)` and `s1 − s0 ≤ 2π`.

**Scale.** `scale = max(source extent, target extent)`. Source extent:
segment length, circle or arc `2r` (whole diameter), ellipse `2a`. Target
extent: plane, the diagonal of the face boundary vertices' bounding box;
cylinder and sphere, the radius; cone, the largest distance from the apex
to a face boundary vertex.

**Residuals** (all checked by the oracles):

- *On-face*: distance from an image point to the support surface, with the
  closest point inside the trimmed region. Exact cells: `≤ 1e-7·scale`.
  Approximate cell: `≤ max_deviation`.
- *Direction*: the angle between `q − p` and `d̂`, where `p` is the source
  point the image came from. Exact cells: `≤ 1e-9`.
- *Carrier*: centres, vertices and semi-axes within `1e-9·scale` of the
  closed forms below; axis directions parallel within `1e-9`.

The oracle does not use the kernel's trim tests, so it stays independent
of the code under test: each oracle face carries a closed-form region in
fixture-local coordinates, and the surface distance comes from
`remus_geometry::extrema`. (While this contract was written, both
`remus_check::distance::point_to_face` and
`remus_check::classify::surface_point_in_face` misjudged points on these
full-turn primitive faces; #909 fixed them.)

## 3. API

### 3.1 Functions

```rust
pub fn project_curve_onto_face(
    topo: &mut Topology, source: EdgeId, direction: Vec3, face: FaceId,
    options: &ProjectCurveOptions,
) -> Result<ProjectedCurves, ProjectCurveError>;

pub fn project_curves_onto_solid(
    topo: &mut Topology, sources: &[EdgeId], direction: Vec3, solid: SolidId,
    options: &ProjectCurveOptions,
) -> Result<SolidProjection, ProjectCurveError>;

pub fn project_curves_onto_plane(
    topo: &Topology, sources: &[EdgeId], direction: Vec3, frame: &Frame3,
) -> Result<Vec<PCurve>, ProjectCurveError>;
```

The public types are defined alongside the implementation. Summary:

- `ProjectCurveOptions { allow_approximate: false, approximation_tolerance:
  None (= 1e-6·scale), max_control_points: 512, plane_frame: None }`.
- `ProjectedEdge { edge, face, source_range, plane_curve: Option<PCurve> }`.
- `ProjectedCurves { edges, face, quality, clipped }`. `clipped` is true
  exactly when the pieces' source ranges do not cover the source domain.
- `SolidProjection { sources: Vec<SourceProjection { source, edges, clipped }>,
  quality }`, one entry per input source in input order. `quality` is the
  worst over all pieces.
- `ProjectionQuality::{Exact, Approximate { max_deviation }}`.

### 3.2 Ownership, transactions, journal

- Results are **free edges**: not in any wire, face, shell or solid, no
  coedges, no p-curves in the registry (p-curves live on coedges, and free
  edges have none). No wire is built: the pieces are in general
  disconnected.
- Vertex sharing: consecutive pieces whose ends coincide within
  `Tolerance::linear` share one vertex (a seam split, a piece crossing onto
  an adjacent face). Pieces separated by a gap never share. A closed image
  is one edge with `start == end`.
- New vertices carry `Tolerance::linear`.
- Every face/solid entry point runs inside `run_append_only`. Success adds
  only vertices and edges. A refusal leaves every count (vertices, edges,
  wires, faces, shells, solids, p-curves) and the journal length unchanged.
  The source edge and the target are never modified.
- **Scoped allocation history.** A successful face/solid call records one
  barrier over only its new free edges and vertices; input entities remain
  outside its scope. The operation opens history after all fallible
  computation and before allocation, so image creation does not cause a
  later global mutation-gap barrier that severs unrelated references.
  An earlier unjournaled mutation still records its usual global barrier.
  Refusals preserve journal entries, the live index, ID counters and mutation
  ticks. A later operation that consumes the edges (an imprint, §9) journals
  its own lineage. Sketch-plane projection remains read-only.
  A containing transaction that rolls back after a successful projection
  retires its image handles and issued journal IDs; those IDs are never reused.
- Multi-source calls are atomic. A per-source refusal returns
  `SourceRefused { index, error }` (also for a single source) and creates
  nothing. Call-level refusals (invalid direction, options or frame,
  grazing a sketch plane, all sources empty) come back bare.

### 3.3 2D output

**On a planar face** with `options.plane_frame = Some(frame)`: every edge
carries `plane_curve` in frame coordinates, `(x, y) = ((q − o)·x̂, (q − o)·ŷ)`.
The frame must satisfy `|ẑ × n| ≤ 1e-9` and `|n·o − δ| ≤ 1e-7·scale` for
the face plane `n·x = δ`, otherwise `PlaneFrameMismatch`; a frame on a
non-planar target is also `PlaneFrameMismatch`. Without a frame
`plane_curve` is `None`. Contract: lifting the 2D curve at `t_start` and
`t_end` gives the edge's start and end vertices, and every lifted sample
lies on the 3D edge within `1e-9·scale`. Lines map to `Line2D`, circles to
`Circle2D`, ellipses to `Ellipse2D`. A circle traversed clockwise in the
frame uses `t_start > t_end`.

**On a sketch plane** (`project_curves_onto_plane`, read-only, unbounded,
no clipping): the plane is `(frame.origin, frame.z)`; the image of each
source is returned in frame coordinates, one `PCurve` per source, input
order. All edge-curve types are exact, because directional projection onto
a plane is affine (§4.1):

| Source | 2D result |
|---|---|
| line | `Line2D` |
| circle, ellipse | `Circle2D` when the image semi-axes agree within `1e-12` relative, else `Ellipse2D` |
| NURBS | `NurbsCurve2D`: same degree, knots and weights, control points mapped by the affine map, **same parameterization** (`curve2d(t)` is the image of `source(t)`) |
| parabola | `NurbsCurve2D`, the exact degree-2 Bézier of the trimmed arc |
| hyperbola | `NurbsCurve2D`, the exact rational quadratic of the trimmed arc |

Refusals here: `InvalidDirection`, `GrazingDirection` (`|d̂·ẑ| ≤ 1e-9`),
and per source `DegenerateImage` (a conic whose plane contains `d̂`, or a
NURBS image with a stationary point).

## 4. Cells: closed forms and numeric instances

Fixtures (unit scale, before placement):

- **Box** `make_box(10, 8, 4)`; top face `z = 4`.
- **Bored plate**: the box cut by `make_cylinder(1, 8)` at `(5, 4, −2)`.
- **Cylinder** `make_cylinder(2, 10)`: axis `+z`, base `z = 0`, seam edge at
  `+x`. (Its parametric `u = 0` is at `+y`; "the seam" in this note always
  means the face's seam *edge*.)
- **Sphere** `make_sphere(3, 64)`: north hemisphere face `z ≥ 0`, no seam
  edge, no pole vertex.
- **Cone** `make_cone(3, 0, 3)`: apex `(0, 0, 3)`, axis `a = −z`,
  `half_angle = 45°` (measured from the radial plane, so
  `k = sin²(half_angle) = 1/2`), seam edge at `+x`.

Every cell runs at scales `1e-3`, `1`, `1e3` and once under the rigid motion
`T(12.5, −7.25, 3.125)·Rz(0.7)·Rx(−0.4)·Ry(1.1)`; expected values transform
with the fixture.

### 4.1 Plane targets: the affine rule

Onto the plane `n·x = δ` along `d̂` (requires `|n·d̂| > 1e-9`):

```
P(x) = x + ((δ − n·x)/(n·d̂))·d̂,     linear part L = I − d̂ nᵀ/(n·d̂).
```

`P` is affine, so lines map to lines and conics to conics of the same
affine class. A circle `c + r(cos t·u + sin t·v)` maps to
`P(c) + cos t·a⃗ + sin t·b⃗` with conjugate semi-diameters `a⃗ = rLu`,
`b⃗ = rLv` (for an ellipse source, `a⃗ = aLu`, `b⃗ = bLv`). The image
semi-axes are the square roots of the eigenvalues of the Gram matrix
`G = [[a⃗·a⃗, a⃗·b⃗], [a⃗·b⃗, b⃗·b⃗]]`:

```
σ₁,₂² = (tr G ± √(tr² G − 4 det G))/2,     σ₁σ₂ = |a⃗ × b⃗|,
```

with the major direction `w₁a⃗ + w₂b⃗` for the `σ₁²` eigenvector `(w₁, w₂)`.
Equal semi-axes (relative `1e-12`) give a circle. Special cases:

- source plane parallel to the target (`N × n = 0`): `Lu = u`, `Lv = v`,
  a congruent, translated circle;
- source plane tilted by `α` about an in-plane axis, `d̂ = −n`: semi-axes
  `r` and `r·cos α`.

| Cell | Source | `d` | Expected |
|---|---|---|---|
| P1 | segment `(1,2,7) → (8,5,9)` | `(0.3, −0.2, −1)` | line `(1.9, 1.4, 4) → (9.5, 4, 4)` (`λ = 3, 5`), range `(0, 1)` |
| P2 | arc centre `(4,4,7)`, `N = +z`, `r = 2`, ref `+x`, `t ∈ [0, π]` | `(0.3, −0.2, −1)` | circle centre `(4.9, 3.4, 4)`, `r = 2`, `(6.9, 3.4, 4) → (2.9, 3.4, 4)`, range `(0, π)` |
| P3 | full circle centre `(5,4,8)`, `N = (0, sin 60°, cos 60°)`, `r = 2`, ref `+x` | `(0, 0, −1)` | closed ellipse centre `(5, 4, 4)`, major `+x`, `a = 2`, `b = 1`, vertex `(7, 4, 4)`, range `(0, 2π)` |
| P4 | as P3 | `(0.3, −0.2, −1)` | closed ellipse centre `(6.2, 3.2, 4)`; `a⃗ = (2, 0, 0)`, `b⃗ = (−0.5196…, 1.3464…, 0)`; `a = 2.1107422412426584`, `b = 1.2757693812211794` (`ab = 2.6928… = |a⃗ × b⃗|`); vertex `(8.2, 3.2, 4)` |
| P5 | full ellipse centre `(5,4,7)`, `N = +z`, `a = 3`, `b = 1.5`, major `(1,1,0)/√2` | `(0.3, −0.2, −1)` | translated ellipse centre `(5.9, 3.4, 4)`, same axes, vertex `(5.9 + 1.5√2, 3.4 + 1.5√2, 4)` |

### 4.2 Segment onto a curved quadric: the sweep plane

A segment `A → B` (unit direction `ê`) swept along `d̂` fills part of the
plane `Π` through `A` with unit normal `m = (ê × d̂)/|ê × d̂|`. Every image
point lies in `Π`, so the image is an arc of the conic `Π ∩ S`, and on it
the **front** arc: the points first reached along `d̂`. Front-arc ends are
either images of the segment ends or **silhouette points**, where `d̂` is
tangent to the conic in `Π`. Within the front arc the map from source
parameter to conic parameter is strictly monotone.

**Cylinder** (axis point `o`, unit axis `a`, radius `r`):

- `|m·a| ≤ 1e-12` (Π ∥ axis): `ρ = |m·(o − A)|`. `ρ < r`: two rulings at
  in-plane offset `±√(r² − ρ²)` from the axis trace; `ρ` within
  `1e-9·scale` of `r`: `TangentSection`; `ρ > r`: no image.
- `|m × a| ≤ 1e-12` (Π ⟂ axis): circle of radius `r`, centre `a`-line ∩ Π.
- otherwise: ellipse, centre `a`-line ∩ Π, semi-minor `r` along
  `(a × m)/|a × m|`, semi-major `r/cos θ` along the in-plane projection of
  `a`, `cos θ = |m·a|`.
- `|a × d̂| ≤ 1e-9` (direction along the rulings): `GrazingDirection`.

| Cell | Segment | `d` | Expected |
|---|---|---|---|
| C1 | `(1,5,2) → (1,5,8)` | `(0, −1, 0)` | ruling line `(1, √3, 2) → (1, √3, 8)` |
| C2 | `(−1,5,4) → (1.5,5,4)` | `(0, −1, 0)` | circle centre `(0,0,4)`, `r = 2`, `(−1, √3, 4) → (1.5, √1.75, 4)` |
| C3 | `(−1,5,6) → (1,5,6)` | `(0, −1, −1)` | `m = (0,1,−1)/√2`, `cos θ = 1/√2`; ellipse centre `(0,0,1)`, major `(0,1,1)/√2`, `a = 2√2`, `b = 2`, `(−1, √3, 1+√3) → (1, √3, 1+√3)` |
| C4 seam | `(5,−1,3) → (5,1.5,3)` | `(−1, 0, 0)` | two circle pieces (centre `(0,0,3)`, `r = 2`) split at the seam point `(2,0,3)`: `(√3,−1,3) → (2,0,3)`, range `(0, 0.4)`; `(2,0,3) → (√1.75,1.5,3)`, range `(0.4, 1)`; shared vertex |
| C5 silhouette | `(−3,5,4) → (3,5,4)`, cylinder turned `−90°` about `z` (seam at `−y`) | `(0, −1, 0)` | front half circle `(−2,0,4) → (2,0,4)`, range `(1/6, 5/6)`, `clipped` |

**Sphere** (centre `c`, radius `R`): `h = |m·(c − A)|`. `h < R`: circle in
`Π`, centre `c − (m·(c − A))·m`, radius `√(R² − h²)`. `h` within
`1e-9·scale` of `R`: `TangentSection`. `h > R`: no image.

| Cell | Segment | `d` | Expected |
|---|---|---|---|
| S1 | `(−1,1,6) → (2,1,6)` | `(0,0,−1)` | `h = 1`: circle centre `(0,1,0)`, normal `y`, `r = √8`, `(−1,1,√7) → (2,1,2)` |
| S2 pole | `(0,−1,6) → (0,1.5,6)` | `(0,0,−1)` | great circle `r = 3`, normal `x`, `(0,−1,√8) → (0,1.5,√6.75)`, through the pole; **one edge** (no seam edge on this face, see §5) |

**Cone** (apex `P₀`, unit axis `a` into the real nappe, `k = sin²(half_angle)`).
With `c = m·a`, `p² = 1 − c²`, `e = m·(A − P₀)` (signed plane offset from the
apex along `m`), `e₁ = (a − c·m)/|a − c·m|` (the axis projected into `Π`),
`e₂ = m × e₁`, a point of `Π` is `P₀ + e·m + s·e₁ + t·e₂` and the cone
`(w·a)² = k|w|²` becomes

```
A·s² + 2ecp·s + e²(c² − k) = k·t²,     A = p² − k = cos²β − cos²γ,
```

where `β` is the plane–axis angle and `γ` the semi-vertical angle; the
eccentricity is `cos β / cos γ`. Every formula below is invariant under
`m → −m` (`c` and `e` change sign together, `e₁` is unchanged); the K rows
quote one choice. Classification (thresholds are part of the contract):

| Condition | Section |
|---|---|
| `|e| ≤ 1e-9·scale` | `SectionThroughApex` (point, one or two generators) |
| `|m × a| ≤ 1e-12` | circle, centre `P₀ + τa`, `τ = e/c`, radius `τ·cot(half_angle)`; `τ ≤ 0` → no image |
| `A < −1e-6` | ellipse (real nappe iff `ec > 0`): centre `P₀ + e·m + s_c·e₁`, `s_c = ecp/|A|`; with `R = e²k(1 − k)/|A|`, semi-axis `√(R/|A|)` along `e₁` and `√(R/k)` along `e₂` |
| `A > 1e-6` | hyperbola, real branch: centre `P₀ + e·m − (ecp/A)·e₁`, real semi-axis `|e|√(k(1 − k))/A` along **`+e₁`** (always: `p·a > |eck/A|` because `c² + k − 1 = −A < 0`), imaginary `|e|√((1 − k)/A)` along `e₂` |
| `|A| ≤ 1e-12` | parabola (real iff `ec > 0`): vertex `P₀ + e·m + s_v·e₁`, `s_v = −e(1 − 2k)/(2cp)`, axis `+e₁`, focal length `ecp/(2k)` |
| `1e-12 < |A| ≤ 1e-6` | `NearParabolicSection { a_coefficient: A }` |

The near-parabolic band is wide on purpose: the ellipse and hyperbola
centres sit `≈ ecp/|A|` away, so at `|A| = 1e-6` evaluation loses about six
digits, which still leaves `1e-7·scale` on-face residual at `1e-16`
arithmetic; closer to the boundary it would not. A parabola for `|A| ≤ 1e-12`
deviates from the true conic far below `1e-7·scale`; the post-check (§7)
still verifies it.

| Cell | Segment | `d` | Expected |
|---|---|---|---|
| K1 | `(−1,5,1) → (1,5,1)` | `(0,−1,0)` | `τ = 2`: circle centre `(0,0,1)`, `r = 2`, `(−1,√3,1) → (1,√3,1)` |
| K2 | `(0.5, y₀, z₀) → (−0.6, y₀, z₀)`, `y₀ = −¼ − 3√3/2`, `z₀ = 9/2 − √3/4` | `(0, √3/2, −½)` | `m = (0, ½, √3/2)`, `c = −√3/2`, `e = −½`, `A = −¼`; ellipse centre `(0, ½, 3 − √3/2)`, major `e₁ = (0, √3/2, −½)`, `a = 1`, `b = √½`; image of `x`: centre `− √(1 − 2x²)·e₁ + (x,0,0)` |
| K3 | `(−1,1,5) → (1.5,1,5)` | `(0,0,−1)` | `c = 0`, `e = 1`, `A = ½`; hyperbola centre `(0,1,3)`, real axis `(0,0,−1)`, `a = b = 1` (in `Π`: `(3 − z)² − x² = 1`), `(−1, 1, 3 − √2) → (1.5, 1, 3 − √3.25)` |
| K4 | `(−1.2,−2,4.5) → (0.3,−2,4.5)` | `(0,1,−1)` | `m = (0,1,1)/√2`, `c = −1/√2`, `e = −1/(2√2)`, `A ≈ 0`; parabola vertex `(0, −¼, 11/4)`, axis `(0,1,−1)/√2`, `f = √2/8` (`y = x² − ¼`, `z = 11/4 − x²`), `(−1.2, 1.19, 1.31) → (0.3, −0.16, 2.66)` |

### 4.3 Arc onto a curved quadric

The sweep of a circle along `d̂` is an elliptic cylinder. Its intersection
with a cylinder, cone or sphere is a degree-4 space curve in general, with
no rational parameterization in the kernel's curve types. **Decision:**
exact only in the coaxial cases below; everything else is the approximate
cell (§6), and refused unless the caller opts in. Rationale: exact
downstream operations (booleans, imprint) must not consume fitted edges by
default, and the sketch-onto-face consumer (S-6) only needs plane targets,
which are always exact.

**Coaxial** means: the arc normal `N ∥ d̂` (`|N × d̂| ≤ 1e-9`) and the arc
centre lies on the line through the sphere centre along `d̂`, or on the cone
axis with `d̂ ∥ a` (each within `1e-9·scale`). The sweep is then a circular
cylinder about that line, and the image is a circle about the same axis:

- sphere: the circle of radius `r` at axial offset `√(R² − r²)` on the side
  facing the source; `r` within `1e-9·scale` of `R`: `TangentSection`;
  `r > R`: no image.
- cone: the circle where the cone radius equals `r`, at axial distance
  `r·tan(half_angle)` from the apex.
- cylinder: `d̂ ∥ axis` is `GrazingDirection`.

| Cell | Arc | `d` | Expected |
|---|---|---|---|
| S3 | centre `(0,0,6)`, `N = +z`, `r = √5`, ref `+x`, `t ∈ [π/4, 3π/4]` | `(0,0,−1)` | circle centre `(0,0,2)`, `r = √5`, `(√2.5, √2.5, 2) → (−√2.5, √2.5, 2)` |
| K5 | full circle centre `(0,0,6)`, `N = +z`, `r = 1.5`, ref `+y` | `(0,0,−1)` | closed circle centre `(0,0,1.5)`, `r = 1.5`, vertex **at the seam** `(1.5, 0, 1.5)`, range `(3π/2, 7π/2)` |

## 5. Clipping and seams

**Clipping.** Pieces are the maximal source intervals whose images lie in
the trimmed region. The current implementation qualifies planar line
polygons and whole circle/ellipse loops (including holes), primitive
cylinder/cone slabs with coaxial circular rims, and qualified spherical
caps or hemispheres. Mixed conic loops, oblique curved rims and other
unqualified trims refuse before publishing edges. Periodic face/solid
sources require ascending strict trims; descending sources refuse.
For solid calls, an exact image covering the whole source on one planar
face bounded by a single full circle or ellipse, with no holes, remains
exact when interval bounds certify that the whole image lies inside the
rim and exclude every deferred curved carrier along each ray up to that
face. A later curved hit cannot force an approximation. Clipped images,
polygonal or holed caps, and unresolved competing hits refuse this shortcut.
Within these cells, M5 computes candidate split parameters as follows:

- segment sources and coaxial arcs (image in a known plane `Π`): each face
  boundary edge ∩ `Π` — line: one root; circle/ellipse: `α cos t + β sin t = γ`,
  two roots. Parabola/hyperbola and NURBS boundaries remain refused;
- arc and ellipse sources on plane targets: roots of the image conic's
  implicit `Q` composed with each boundary edge, `Q(B(t)) = 0` — quadratic
  for lines, bounded quartic root isolation in `tan(t/2)` for
  circles/ellipses;
- silhouette points (§4.2) and the ends of the source.

Candidates are mapped to source parameters, sorted, and each open interval
between consecutive candidates is classified by its midpoint with
`remus_check::classify::surface_point_in_face`. Inside intervals merge into
pieces; a piece shorter than `1e-9·scale` is dropped. An image running
along a boundary edge over a positive length is refused
(`ImageAlongBoundary`): the midpoint test is meaningless on the boundary.
The CLIP cell: segment `(−2,4,6) → (12,4,6)`, `d = −z`, bored plate → lines
`(0,4,4) → (4,4,4)` range `(1/7, 3/7)` and `(6,4,4) → (10,4,4)` range
`(4/7, 6/7)`, `clipped`, no shared vertex.

**Seams.** An image is split wherever it meets a **seam edge** of the target
face (an edge the face's boundary uses twice), including the seam edge's
end vertices; the pieces share the split vertex. A face without a seam edge
is never split, even where the curve crosses the surface's parametric
`u = 0` or passes over a pole (S2). Consequence: each piece stays inside
one period of the face's own chart, which is what a later imprint needs to
build its p-curves. A closed image on a face with a seam edge is one closed
edge whose vertex is the seam crossing (K5); without a seam edge, the
vertex is the image of the source start (P3–P5).

## 6. The approximate cell

Non-coaxial circular source → full-period cylinder slab,
`allow_approximate = true`. The target must have two full circular rims,
one seam and no inner wires. The current certificate encloses the entire
source circle, including the carrier beyond an open source arc; an
unresolved enclosure refuses conservatively. It requires an exterior
source and two distinct forward support roots over that circle; interior
or tangent source configurations remain uncertified. Cone and sphere approximate
trim cells remain refused until whole-image containment is certified.

- Image points are exact: the first root of the ray `p(t) + λd̂` against the
  quadric, closed form.
- The edge is a clamped NURBS (degree 3) fitted through them, refined until
  the deviation is within `approximation_tolerance` or the
  `max_control_points` budget is spent (`ToleranceUnattainable { requested,
  achieved }`). A closed source gives a closed edge.
- **Deviation metric** at an edge point `q`: `max(dist(q, S), | |q₀ − c| − r |)`,
  where `q₀ = q − ((q − c)·N/(d̂·N))·d̂` pushes `q` back onto the source
  plane. `max_deviation` is a **sampled** maximum, labelled as such, not a
  certified bound. Contract: `0 < max_deviation ≤ tolerance`, and the oracle
  measuring the same metric at 4096 uniform parameter samples must not
  exceed it, so M5 samples more densely than that.
- Options: `tolerance` must be finite and `≥ 1e-9·scale`;
  `max_control_points ≥ 4`; otherwise `InvalidOptions`.
- No clipping in slice 1: outward interval bounds certify every first
  positive infinite-support root and enclose the whole exact image strictly
  between the rim height bounds. Positive rational weights and an axial
  control hull certify the entire fitted edge within the same slab. A hole,
  an exterior sliver or either unresolved certificate returns
  `ApproximateClipUnsupported`; trim probes and post-check samples cannot
  establish containment.
- A solid call accepts the fitted image only when one face contains the
  whole image and conservative bounds prove that every other face cannot
  meet any source ray. Competing faces or unresolved visibility return
  `ApproximateClipUnsupported`; sampled probes do not decide occlusion.
  Mixed exact and approximate sources report the largest sampled deviation
  in the solid result's `quality`.
- Instance A1: unit circle, centre `(0,5,5)`, `N = +y`, ref `+x`, `d = −y`,
  onto the cylinder: `q(t) = (cos t, √(4 − cos²t), 5 + sin t)`. Default
  tolerance at `scale = 2` is `2e-6`; `max_control_points = 4` is
  `ToleranceUnattainable`; centre `z = 9.5` leaves the face (`z > 10`).

## 7. Refusals

Every refusal leaves the topology unchanged (§3.2). Codes are API.

| Code | Variant | Condition | Test |
|---|---|---|---|
| `invalid-direction` | `InvalidDirection` | `d` zero or non-finite | `invalid_direction_is_refused`, `sketch_plane_refusals` |
| `invalid-options` | `InvalidOptions` | tolerance `≤ 0`, non-finite or `< 1e-9·scale`; `max_control_points < 4` | `invalid_options_are_refused` |
| `degenerate-source` | `DegenerateSource` | segment shorter than `Tolerance::linear`; invalid source geometry | `zero_length_source_is_refused`, solid atomicity |
| `unsupported-source-curve` | `UnsupportedSourceCurve { curve, surface }` | §1 table (tags from `type_tag()`) | `unsupported_source_curves_are_refused` |
| `unsupported-target-surface` | `UnsupportedTargetSurface { surface }` | torus, NURBS | `torus_and_nurbs_targets_are_refused` |
| `source-parallel-to-direction` | `SourceParallelToDirection` | `|ê × d̂| ≤ 1e-9` | `segment_parallel_to_direction_is_refused` |
| `degenerate-image` | `DegenerateImage` | conic source with `|N·d̂| ≤ 1e-9`; NURBS image with a stationary point | `edge_on_arc_is_a_degenerate_image`, `sketch_plane_refusals` |
| `grazing-direction` | `GrazingDirection` | plane: `|n·d̂| ≤ 1e-9`; cylinder: `|a × d̂| ≤ 1e-9` | `grazing_directions_are_refused`, `sketch_plane_refusals` |
| `tangent-section` | `TangentSection` | §4.2/§4.3 tangency bands | `tangent_sweep_planes_are_refused` |
| `section-through-apex` | `SectionThroughApex` | `|e| ≤ 1e-9·scale` | `sweep_plane_through_the_cone_apex_is_refused` |
| `near-parabolic-section` | `NearParabolicSection` | `1e-12 < |A| ≤ 1e-6` | `near_parabolic_cone_section_is_refused` (K4 with `d` tilted `1e-8` rad, `A ≈ −1e-8`) |
| `approximation-required` | `ApproximationRequired` | approximate cell, default options | `approximate_cell_is_refused_by_default` |
| `tolerance-unattainable` | `ToleranceUnattainable` | budget exhausted | `approximate_cell_budget_exhaustion_is_typed` |
| `approximate-clip-unsupported` | `ApproximateClipUnsupported` | approximate image leaves the face | `approximate_image_crossing_the_face_boundary_is_refused` |
| `image-along-boundary` | `ImageAlongBoundary` | image overlaps a boundary edge | `image_along_a_face_boundary_is_refused` |
| `empty-projection` | `EmptyProjection` | no image at all (face call, or every source of a solid call) | `empty_projections_are_refused`, `solid_level_all_sources_missing_is_empty` |
| `plane-frame-mismatch` | `PlaneFrameMismatch` | §3.3 | `plane_frame_mismatch_is_refused` |
| `residual-exceeded` | `ResidualExceeded { measured, bound }` | post-check failed | M5 unit test (below) |
| `source-refused` | `SourceRefused { index, error }` | multi-source wrapper | `solid_level_is_atomic_and_names_the_refused_source`, `sketch_plane_refusals` |
| `operations` | `Operations(OperationsError)` | stale handles, kernel failures | `stale_handles_surface_as_topology_errors` |

**Post-check.** Before commit, every piece is sampled (33 points) for the
on-face residual against its face (support distance plus region test) and
compared with `1e-7·scale` (exact) or `max_deviation` (approximate). A
failure is `ResidualExceeded` with rollback. No input reaches it in a
correct implementation, so M5 covers it with a unit test that feeds the
post-check a perturbed curve.

## 8. Layer placement and M5 work plan

**`remus-geometry` (L1, math only)** — new `geometry/src/projection/`,
pure geometry, no topology:

- `affine.rs`: the plane map `P`, its linear part, the Gram rule
  (line/circle/ellipse images), NURBS control-point mapping, exact 2D
  rational forms for parabola and hyperbola arcs;
- `sections.rs`: sweep plane, plane ∩ cylinder/sphere, the cone classifier
  with the bands of §4.2 and real-branch selection, coaxial circles;
- `front.rs`: ray–quadric first roots, silhouette parameters, the monotone
  source-to-image parameter maps;
- `approx.rs`: the §6 fit and deviation metric, on `math/nurbs/fitting.rs`.

These do not go into `math/analytic_intersection.rs`: its plane–cone path
deliberately returns sampled chains for parabolas and hyperbolas and the
boolean engine depends on that; changing it is out of scope.

**`remus-operations` (L3)** — `project_curve.rs` becomes a directory
(`project_curve/{mod,clip,seam,tests}.rs`): handle lookup and refusals,
dispatch to geometry, clipping (§5), seam splitting, vertex sharing,
`run_append_only` allocation, post-check, solid-level visibility and
atomicity. Allowed deps: `geometry`, `check` (midpoint classification),
`topology`, `math` — no new crate edges.

**Prerequisite (met).** M5's midpoint classification depends on
`surface_point_in_face` being right on pointed-cone lateral faces; #909
fixed the pole handling that made it reject them.

**`remus-wasm` (L4)** — `bindings/project_curve.rs`, results as tsify types
in `types.rs`, errors as structured errors whose code is
`project-curve:<code>`:

```ts
projectCurveOntoFace(edge: number, dirX: number, dirY: number, dirZ: number,
                     face: number, options?: ProjectCurveOptions): ProjectedCurves
projectCurvesOntoSolid(edges: Uint32Array, dirX: number, dirY: number, dirZ: number,
                       solid: number, options?: ProjectCurveOptions): SolidProjection
projectCurvesOntoSketchPlane(edges: Uint32Array, dirX: number, dirY: number, dirZ: number,
                             frame: SketchFrame): PlaneCurve2d[]

interface ProjectCurveOptions { allowApproximate?: boolean; approximationTolerance?: number;
  maxControlPoints?: number; planeFrame?: SketchFrame }
interface SketchFrame { origin: [number, number, number]; xAxis: [number, number, number];
  normal: [number, number, number] }            // y = normal × xAxis
interface ProjectedEdge { edge: number; face: number; sourceStart: number; sourceEnd: number;
  planeCurve?: PlaneCurve2d }
type ProjectionQuality = { kind: "exact" } | { kind: "approximate"; maxDeviation: number }
interface ProjectedCurves { edges: ProjectedEdge[]; face: number; quality: ProjectionQuality;
  clipped: boolean }
interface SourceProjection { source: number; edges: ProjectedEdge[]; clipped: boolean }
interface SolidProjection { sources: SourceProjection[]; quality: ProjectionQuality }
type PlaneCurve2d =
  | { kind: "line"; start: [number, number]; end: [number, number] }
  | { kind: "circle"; center: [number, number]; radius: number; startAngle: number; endAngle: number }
  | { kind: "ellipse"; center: [number, number]; semiMajor: number; semiMinor: number;
      rotation: number; startAngle: number; endAngle: number }
  | { kind: "nurbs"; degree: number; knots: number[]; controlPoints: [number, number][];
      weights: number[]; tStart: number; tEnd: number }
```

`executeBatch` gets the same three ops with these names and JSON shapes.
For NURBS, the serialized control net is restricted to the traversed interval
while retaining the source knot parameterization. `tStart` and `tEnd` specify
the traversal, including descending intervals; clients evaluate the carrier
from `tStart` toward `tEnd` instead of assuming increasing knot order means
increasing traversal.
Contract tests go through `execute_batch()` per the wasm-bindings skill.
Projection is in the shipped kernel package (no `io` feature).
Sketch-plane NURBS currently require an already clamped, single-span
Bézier carrier whose projected derivative and emitted control points can
be qualified with outward-rounded arithmetic. Multispan carriers or an
unresolved certificate refuse with `degenerate-image`; descending sketch
traversal remains supported.

## 9. Phase 3 — imprint composition (deferred, not M5)

Projected free edges are not yet imprintable. `imprint::imprint` takes a
tool **solid** and imprints every section of the tool with the target, so
an open projected curve cannot be expressed as a tool. Two routes, for a
later row (4.5 / B73), not for M5:

- **Closed planar source wire `W`, planar or curved target face, exact
  cells only.** Tool = `extrude(face(W), d̂·L)` from the source plane, with
  `L = λ_max + ε`, `λ_max` the largest first-hit distance over the
  projected images and `ε = 1e-3·scale`, so the prism passes the target
  face and ends inside the body before reaching a second face along `d̂`
  (verified by a first-hit probe through the cap; otherwise refuse). The
  prism's lateral faces cut the target face along exactly the projected
  closed curve; `imprint(target, prism)` then splits the face, and the
  result is checked against §4 by matching the new edges to the projected
  ones. Fails when the prism also crosses other target faces (deep
  pockets) — refuse there.
- **Open curves.** Needs a "split face by free edges" operation on the
  face splitter (`algo/builder/face_splitter/`), consuming the projected
  edges with their single-period guarantee (§5). That is the general
  route and the one B73 wrap/emboss composes on.

## 10. Test map

The 48 API acceptance tests in `qualify_project_curve.rs` run without
`ignore` attributes. Two additional self-checks,
`oracle_self_check_every_exact_cell_at_every_placement` and
`oracle_self_check_solid_level_box_pieces` build every expected answer as
edges and pass it through the acceptance checker, proving that the closed
forms above, the placements and the independent ray-cast oracle agree. The
checker rejects a wrong radius or focal length, the wrong hyperbola branch,
a wrong clip range, a closed vertex off the seam, a reversed traversal and
the back-sheet image (each mutation tried by hand).

Per piece the checker asserts: face; source range; start/end vertices;
closedness and vertex sharing; curve type and closed-form parameters;
explicit domain whose ends evaluate to the vertices; and at 33 samples the
on-face residual, the direction residual, a forward pre-image inside the
source range that increases monotonically, first-hit agreement with the
oracle, and no crossing of a face seam edge.

M5 completion requires all 48 API acceptance tests, both self-checks,
the projection regressions, and the gates in `AGENTS.md` to pass.
