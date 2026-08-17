# Architecture

Remus is a modeling kernel: geometry evaluation, boolean operations,
blends, tessellation, and data exchange. It exposes a Rust API and a
JavaScript API over the same engine, and leaves presentation to the caller.

Remus uses a strict layered architecture. Each layer may only depend on
layers below it, never above or sideways. The layer DAG is a program
invariant, not a convention: preserving it is a constraint on every change.

```
┌──────────────────────────────────────────────────────┐
│  L4: remus-wasm          remus-render            │  JS API / offscreen GPU
├──────────────────────────────────────────────────────┤
│  L3: remus-operations    remus-io                │  Modeling ops / exchange
├──────────────────────────────────────────────────────┤
│  L2: algo  blend  check  heal  offset  sketch        │  Engines
├──────────────────────────────────────────────────────┤
│  L1: remus-topology      remus-geometry          │  B-Rep structures
├──────────────────────────────────────────────────────┤
│  L0: remus-math                                    │  Vectors, NURBS, predicates
└──────────────────────────────────────────────────────┘
```

## Layer Rules

| Crate | Layer | Allowed Dependencies |
|-------|-------|---------------------|
| `remus-math` | L0 | External crates only |
| `remus-geometry` | L1 | `remus-math` |
| `remus-topology` | L1 | `remus-math` |
| `remus-algo` | L2 | `math`, `topology` |
| `remus-blend` | L2 | `math`, `topology` |
| `remus-check` | L2 | `math`, `topology`, `geometry` |
| `remus-heal` | L2 | `math`, `topology`, `geometry` |
| `remus-offset` | L2 | `math`, `topology`, `geometry` |
| `remus-sketch` | L2 | External crates only |
| `remus-operations` | L3 | All L0 to L2 crates |
| `remus-io` | L3 | `math`, `topology`, `operations` |
| `remus-wasm` | L4 | All workspace crates |
| `remus-render` | L4 | `math`, `topology`, `operations` |

These rules are enforced by `scripts/check-boundaries.sh`, which runs in CI.
`remus-render` is a leaf: nothing may depend on it.

## Arena-Based Topology

All topological entities (vertices, edges, faces, etc.) are stored in a
central `Arena` and referenced by typed index handles. This approach:

- Avoids reference counting overhead (`Rc`/`Arc`)
- Enables cache-friendly traversal (data locality)
- Makes ownership clear (the arena owns everything)
- Provides O(1) entity lookup

## Analytic Geometry, with NURBS as the General Case

Curves and surfaces are enums, not a single universal representation.
`FaceSurface` is one of `Plane`, `Cylinder`, `Cone`, `Sphere`, `Torus`, or
`Nurbs`; `EdgeCurve` is one of `Line`, `Circle`, `Ellipse`, or `NurbsCurve`.

Analytic types are deliberately special-cased rather than collapsed into
NURBS:

- Operations preserve them. A cylinder cut by a plane stays a `Cylinder`,
  so face counts stay low across chained booleans instead of growing with
  every step.
- Intersections take exact closed-form paths where a pair allows one
  (see `math/src/analytic_intersection.rs`), falling back to NURBS
  marching only when no analytic solution exists.
- STEP export writes them as native surface entities, so a round-trip is
  lossless rather than an approximation.

NURBS is the general representation that everything can convert into, and
the one used for free-form geometry. It is the fallback, not the default.
