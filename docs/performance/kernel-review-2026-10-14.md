# Kernel review 2026-10-14: instruction-count leads

The performance leads from the 2026-10-14 kernel review, re-measured as
cachegrind instruction counts (`Ir`) on main `0186a84`. Each lead that cleared
the bar (at least 5% of a real workload, with output byte- or digest-identical
or oracle-equivalent) shipped as its own PR. The others are recorded below with
their numbers.

## Method

- Driver: [`crates/operations/examples/ir_probe.rs`](../../crates/operations/examples/ir_probe.rs).
  One workload per run, printing an output digest.
- WASM-3 driver: `crates/wasm/examples/readonly_batch_ir.rs`, added by
  [#1137](https://github.com/esaueng/remus/pull/1137).
- Build: `cargo rustc --profile profiling -p remus-operations --example ir_probe -- -C link-arg=-fuse-ld=bfd`.
  The `profiling` profile is release without LTO, with debug info. Valgrind
  cannot read lld-linked symbols, hence bfd.
- Run: `RAYON_NUM_THREADS=1 valgrind --tool=cachegrind --cache-sim=no`.
  Attribution comes from callgrind (`callgrind_annotate --inclusive=yes`).
  Base and candidate runs were alternated.
- Digest: each shell with its role (outer or cavity), and per face in shell
  order the exact surface `Debug`, orientation and every wire's traversal
  (vertex position bits, curve, trim, tolerance, and each use's coedge edge,
  stored pcurve and periodic winding). It also covers connectivity: which edge
  and vertices each use names, how many uses share each, and which faces each
  edge and vertex bounds. Lazily filled caches in a `Debug` rendering (the
  NURBS weight `OnceLock`s) are left out.
  A `canonical` digest of the same content ignores face order, cavity order and
  the start of each closed wire. Mesh workloads digest position bits, normal
  bits and indices.
- glibc selects libm variants by CPU feature, so digests are compared native
  with native and valgrind with valgrind.

| workload | what it runs | base `Ir` |
|---|---|---:|
| `box_fillet` | `cad_operations` `box+fillet`: box(20), rolling-ball fillet on all 12 edges, r = 1 | 81.1–81.5M |
| `box_fillet_v2` | the same box through `blend_ops::fillet_v2` | 156.6–156.8M |
| `fuse_overlap_12` | `fuse_balanced/balanced_N=144`: `fuse_all`, 12×12 overlapping box grid | 2,861.8M |
| `fuse_touch_12` | `fuse_touching/balanced_N=144`: `fuse_all`, 12×12 touching box grid | 1,184.4M |
| `seq_fuse_16` | `fuse_balanced/sequential_N=16` | 410.5M |
| `cut_box_cyl_x10` | `cad_operations` `cut(box,cyl) x10` | 87.2M |
| `mesh_sphere` | `cad_operations` `mesh sphere (tol=0.01)` | 297.3–297.7M |
| `seq_cuts_64` | `boolean_perf` `sequential_cylinder_cuts/N=64` (local-cut path) | 135.2M |
| `mesh_plate_64` | whole-solid tessellation of a 64-hole plate, deflection 0.01 | 262.4M |

The ranges come from the rolling-ball and v2 fillets, which iterate std
`HashMap`s; see [Determinism](#determinism).

`Ir` counts the whole process, including setup and the digest. Callgrind's
inclusive count for the measured operation (`fuse_all`, `boolean`,
`tessellate`, or the fillet call) is 92.4% of the total on `seq_cuts_64`,
93.8% on `mesh_sphere`, and 97.2–99.3% on the rest. Setup and the digest do
the same work in base and candidate, because their outputs are identical. So
they cancel out of each absolute saving and only shrink the percentages.
Against the operation alone, every shipped Δ is the same or larger: `seq_cuts_64`
−21.9% and `mesh_sphere` −45.1%; the others change by at most 0.7 points. The
below-bar leads keep their side of the line: the largest operation-only share
is `deep_copy_solid_with_maps` at 4.9% of `seq_fuse_16`.

## Shipped

| lead | PR | workload | base → candidate | Δ |
|---|---|---|---:|---:|
| `validate_blend_volume`: shared bucketed edge census | [#1111](https://github.com/esaueng/remus/pull/1111) | `box_fillet` | 81.1M → 54.1M | −33.4% |
| | | `box_fillet_v2` | 156.6M → 102.5M | −34.5% |
| `phase_ff_coplanar::perform`: face-keyed curve and FF index | [#1119](https://github.com/esaueng/remus/pull/1119) | `fuse_overlap_12` | 2,861.8M → 2,470.9M | −13.7% |
| | | `fuse_touch_12` | 1,184.4M → 1,072.8M | −9.4% |
| `JunctionRegistry` resolve + clone: copy-on-write cells, shared per-face caches | [#1128](https://github.com/esaueng/remus/pull/1128) | `fuse_overlap_12` | 2,861.8M → 2,703.0M | −5.5% |
| | | `fuse_touch_12` | 1,184.4M → 1,140.7M | −3.7% |
| `sample_wire_outline` (touching grid): EF containment cached per N-way pass | [#1130](https://github.com/esaueng/remus/pull/1130) | `fuse_touch_12` | 1,184.4M → 1,039.9M | −12.2% |
| | | `fuse_overlap_12` | 2,861.8M → 2,716.1M | −5.1% |
| `(i64,i64,i64)` mesh keys on mesh sphere: pre-size, plus a per-vertex band reference | [#1135](https://github.com/esaueng/remus/pull/1135) | `mesh_sphere` | 297.6M → 171.8M | −42.3% |
| `copy_solid_spliced`: authority aligned past spliced hole loops | [#1136](https://github.com/esaueng/remus/pull/1136) | `seq_cuts_64` | 135.2M → 107.8M | −20.3% |
| WASM-3: no rollback for a read-only op that never wrote | [#1137](https://github.com/esaueng/remus/pull/1137) | 32-query batch on a 64-hole plate | 581.9M → 1.1M (batch) | −99.8% |

Every PR measured its candidate against base `0186a84` alone. With the
digest extended to coedge authority, shells and incidence, #1119, #1128, #1130 and #1136 still
match base on every boolean workload, and #1111 keeps the fillets' canonical
digest. The fuse PRs
touch different phases of the same workloads, so their gains compound.

## Below the bar

| lead | workload: share of base `Ir` | finding |
|---|---|---|
| `builder_solid::build_solids_with_origins` | `cut_box_cyl_x10`: 8.1% (7.1M). `seq_fuse_16`: 12.7% (52.1M) | Spread across passes, none worth 5% alone. On `seq_fuse_16`: `clean_selected_faces` 7.8% (of which `merge_duplicate_edges` 1.6%, `split_arc_edges_at_collinear_vertices` 0.74%, `PointGrid::box_candidates` 0.58%, `remove_doubled_faces` 0.30%, the rest inlined cleanup passes at a few tenths each), `perform_loops` 2.9% (`face_adjacency_keys` 0.71%), `assemble_regions` 1.7% (`prune_dangling_edges` 0.99%). |
| Shape-store `deep_copy_solid_with_maps` | `seq_fuse_16`: 4.84% (19.9M). `cut_box_cyl_x10`: 2.42% (2.1M). `fuse_touch_12`: 1.31% | Below 5% even if removed entirely. |
| det-hash of `(i64,i64,i64)` keys | `cut_box_cyl_x10`: `hash_one` 3.41%, `reserve_rehash` 3.72%, over all key types | Below the bar on this workload. The mesh-sphere share shipped in #1135. |
| `FaceExtents::ensure` | `cut_box_cyl_x10`: 2.01% (1.75M). `fuse_touch_12`: 1.08% | Below the bar. |
| `PCurveRegistry` scans | under 1% on every workload | Below the bar. |
| `clip_line_to_face` (touching grid) | `fuse_touch_12`: 6.43% (75.3M), of which `chain_edge_vertices` 3.2% (37.6M) | Rebuilds each face's polygon from its wire on every call, across all N-way FF pairs. A per-face cache across pairs is exact (FF never edits a face boundary on the shared path) but belongs with the shared per-face caches #1128 introduces. Follow-up once #1128 lands; expected gain about 4%. |
| `distance_to_polygon_boundary` (touching grid) | `fuse_touch_12`: 4.31% (50.5M), 42.7M of it from EF `FaceContainment::accepts` | Below the bar. Each test walks the sampled outline (33 points per edge, collinear on line edges). Merging collinear samples would change distance bits, so it is not identity-preserving. |
| `find_nearby_pave_vertex` (touching grid) | `fuse_touch_12`: 3.91% (45.9M), 29.4M from phase EE | Below the bar. |
| Phase-FF-coplanar `compute_face_bboxes` (observed) | `fuse_touch_12`: 3.24% (37.9M) | Boxes recomputed per solid pair. Same follow-up shape as `clip_line_to_face`. |
| `normalize_hole_windings` (64 sequential cuts) | `seq_cuts_64`: 17.9% (24.1M) | Handled by the separate review PR named in the review. Not touched here. |

### PLAUSIBLE items

| item | measurement | finding |
|---|---|---|
| heal ReShape recording, O(merged × V) (`crates/heal/src/fix/solid.rs`) | No listed workload reaches it | The `vertex_ids.iter().find` per merged vertex is quadratic, but only an input with coincident duplicate vertices merges anything. STL import welds before healing, and booleans and primitives share vertices. It needs a triangle-soup workload before it can be measured. |
| `independent_blend_groups` O(N²·m²) (`blend_ops.rs`) | `box_fillet_v2`: below 0.01% (12 edges, one group) | Below the bar at real edge counts. |
| `g1_chains` rebuilds adjacency per seed (`crates/blend/src/g1_chain.rs`) | `box_fillet_v2`: 0.72% (1.12M, 2 calls) | Below the bar. |
| `edge_concavity_from_faces` one-shot classifiers | `box_fillet`: 3.32% (2.70M). `box_fillet_v2`: 1.72% | Below the bar. |
| Phase VF / `fill_boundary_on` dead writes | `phase_vf::perform` total: `fuse_touch_12` 3.37% (39.4M), `seq_fuse_16` 1.39%, `fuse_overlap_12` 1.22% | The dead writes are a subset of a phase already under 5%. |

## Determinism

Both fillet engines produce a different face order and closed-wire start from
run to run, though the geometry is the same:

- `box_fillet`: 8 of 8 native runs gave 8 strict digests and one canonical digest.
- `box_fillet_v2`: the same pattern.
- Cause: the rolling-ball engine keys std `HashMap`s (`RandomState`) by index
  in `fillet/rolling_ball.rs`.

The boolean and tessellation workloads are deterministic. Downstream consumers
that diff topology order, such as golden files or persistent naming by
position, will see churn from the fillets. This is not a performance item; it
is recorded here for a separate fix.
