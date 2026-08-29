# P-Class Program — running ledger

Issue → state → PR, so any session can resume the program cold. States:
`open` → `in-progress` → `in-review (PR #N)` → `merged (PR #N)` /
`deferred (reason)`. Update in each issue's final PR.

Program plan: [p-class-program.md](p-class-program.md) (PR #119 at program
start; move to a permanent link once merged).

## M2 — General curved booleans

| Issue | Title | State | PR |
|-------|-------|-------|----|
| 2.0 | RFC 0002 completion: trims & p-curves | in-progress | 2.0a: trim writers |
| 2.1 | Honest-failure hygiene (typed refusals) | open | — |
| 2.2 | Sphere in general position | open | — |
| 2.3 | Steinmetz ellipses (equal-radius cyl×cyl) | open | — |
| 2.4 | Quadric×quadric transversal, NURBS seams | open | — |
| 2.5 | NURBS×NURBS booleans | open | — |
| 2.6 | Scale-relative band audit | open | — |
| 2.7 | Tangency & sliver contacts (stretch) | open | — |
| 2.8 | OperationContext budgets & cancellation | open | — |

## M3 — Tolerant modeling

| Issue | Title | State | PR |
|-------|-------|-------|----|
| 3.1 | RFC 0004: per-entity tolerance semantics | in-review (draft landed on branch docs/rfc-0004-draft) | — |
| 3.2–3.6 | Substrate / predicates / GFA / import+sew / disclosure | open (blocked on 3.1) | — |

## M4 — Body taxonomy

| Issue | Title | State | PR |
|-------|-------|-------|----|
| 4.1 | RFC 0005: body classes & cellular results | in-review (draft landed on branch docs/rfc-0005-draft) | — |
| 4.2–4.7 | Sheet bodies / split / trim / imprint / multi-region / wire | open (blocked on 4.1) | — |

## M5–M8

Not started. M7.5 and 8.2 are dependency-free filler for idle capacity.

## Measured survey (issue 2.0 baseline, main @ abcbdc67)

- `domain_with_endpoints`: 132 production call sites — 87 trim-aware
  (`&Edge` delegate), 45 trim-blind (`EdgeCurve::` direct), of which 40 are
  reader-reconstruction risks (34 in algo, 1 STEP-reader fallback, 5
  legitimate new-geometry sites). No grep gate exists.
- Trim writers: GFA (algo) result-assembly chain carries trims end-to-end.
  Open gaps were: `merge_result_vertices` (operations/boolean/mod.rs), the
  analytic fast paths (SphereCapFace / CylindricalFace / box-sphere octant
  arcs), coaxial-cone rim trims, `copy_and_transform_solid`, extrude top
  edges, loft ring edges, blend trimmer `split_edge_at`, blend/chamfer
  vertex-substitute rebuilds, `unify_faces`, `unify_same_domain`, wasm
  `reverseShape`.
- SameParameter/SameRange validators (topology/validation.rs) have zero
  external callers; no boolean-output CI coverage.
- Transaction machinery exists and is adopted at 10 sites; the boundary
  authority flip precondition (sanctioned mutation) is met at boolean/GFA/
  blend entry points, not in heal/offset (79 uncontrolled in-place mutation
  sites across five crates).
- make_torus builds the minimal CW complex (degenerate Line seams) — no
  circle rims to trim on a rebuilt torus; torus-side rim trims are moot
  until M2.4's splitters.
