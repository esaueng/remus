# remus-check mutation triage — 2026-09-18

One row per survivor for the three triaged files; file-level summary for the rest.

## Method

- Tool: `cargo-mutants 27.0.0` (the weekly workflow's pinned version), Rust 1.96.0 toolchain family.
- `.cargo/mutants.toml` `examine_globs` cover only math/algo/blend/offset/operations, so
  `cargo mutants -p remus-check` examines **zero** mutants. This campaign overrides discovery
  with `--no-config` (local runs only; the committed CI scope is unchanged).
- Test scope is crate-only: `-- -p remus-check`. Downstream crates' suites (operations,
  algo, io, wasm) were NOT run per mutant, so these survivor counts over-approximate the
  workspace-scope survivors the weekly CI would report. In particular the B20 scale-matrix
  (`operations/tests/b20_exact_measurement_scale_matrix.rs`) asserts several of these closed
  forms downstream; the new tests here pin them at the defining layer instead.
- `--baseline skip` (the crate suite was verified green first).
- The first full-crate run died at 3544/3972 on a transient disk-full (`No space left on device`); the remaining 7 `validate/*` files were run with `-f` filters to a separate output dir on the pristine tree and merged by mutant name (verified total == 3972, the exact `--list` count).
- After-runs per triaged file re-test every mutant in that file on the final tree.
- Verdicts: (a) missing assertion — new unit test fails under the mutant, passes on real code;
  (b) equivalent — one-line reason; (c) needs geometry judgment; timeout — mutant hangs the suite.

## Crate baseline (before, measured)

- 3972 mutants: 2324 missed, 1261 caught, 386 unviable, 1 timeout (miss rate 58.5%).
- remus-check has far more than 10 survivors, so no remus-geometry second PR follows from this run.

## Before / after per triaged file (measured)

| File | Mutants | Missed before | Missed after | Killed | Residual |
| --- | ---: | ---: | ---: | ---: | --- |
| `properties/analytic.rs` | 266 | 162 | 0 | 162 | 0 |
| `validate/wire.rs` | 181 | 93 (+1 timeout) | 16 (+1 timeout) | 77 | 16 equiv/boundary + 1 hang |
| `validate/face.rs` | 114 | 74 | 18 | 56 | 18 equivalent |
| **Triaged total** | **561** | **329** | **35** | **295** | **35** |

295 killed by 57 new unit tests (12 analytic + 31 wire + 14 face; lib suite 87 -> 144). No production code changed; no existing
 assertion weakened. Projected crate remainder: 2324 - 295 = 2029 missed (arithmetic, not re-measured).

## crates/check/src/properties/analytic.rs

- before: 162 missed; after: 0 missed, 266 caught, 0 unviable, 0 timeout.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/check/src/properties/analytic.rs:22:30: replace * with / in box_props | (a) | box_exact_props |
| crates/check/src/properties/analytic.rs:22:40: replace * with / in box_props | (a) | box_exact_props |
| crates/check/src/properties/analytic.rs:23:30: replace * with / in box_props | (a) | box_exact_props |
| crates/check/src/properties/analytic.rs:23:40: replace * with / in box_props | (a) | box_exact_props |
| crates/check/src/properties/analytic.rs:36:28: replace * with / in sphere_props | (a) | sphere_exact_props |
| crates/check/src/properties/analytic.rs:39:17: replace / with % in sphere_props | (a) | sphere_exact_props |
| crates/check/src/properties/analytic.rs:39:17: replace / with * in sphere_props | (a) | sphere_exact_props |
| crates/check/src/properties/analytic.rs:39:23: replace * with + in sphere_props | (a) | sphere_exact_props |
| crates/check/src/properties/analytic.rs:39:23: replace * with / in sphere_props | (a) | sphere_exact_props |
| crates/check/src/properties/analytic.rs:39:27: replace * with + in sphere_props | (a) | sphere_exact_props |
| crates/check/src/properties/analytic.rs:39:27: replace * with / in sphere_props | (a) | sphere_exact_props |
| crates/check/src/properties/analytic.rs:39:36: replace * with + in sphere_props | (a) | sphere_exact_props |
| crates/check/src/properties/analytic.rs:39:36: replace * with / in sphere_props | (a) | sphere_exact_props |
| crates/check/src/properties/analytic.rs:52:16: replace * with / in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:52:25: replace * with / in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:55:17: replace / with % in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:55:17: replace / with * in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:55:24: replace * with + in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:55:24: replace * with / in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:55:31: replace * with + in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:55:31: replace * with / in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:55:40: replace * with + in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:55:40: replace * with / in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:55:49: replace + with - in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:55:49: replace + with * in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:55:58: replace * with + in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:55:58: replace * with / in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:58:17: replace / with % in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:58:17: replace / with * in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:58:23: replace * with + in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:58:23: replace * with / in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:58:32: replace * with + in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:58:32: replace * with / in cylinder_props | (a) | cylinder_exact_props |
| crates/check/src/properties/analytic.rs:73:24: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:74:21: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:76:22: replace + with - in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:76:29: replace + with - in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:76:29: replace + with * in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:79:15: replace < with == in cone_props | (a) | cone_degenerate_zero_radii |
| crates/check/src/properties/analytic.rs:79:15: replace < with <= in cone_props | (a) | cone_degenerate_threshold_boundary_takes_normal_path |
| crates/check/src/properties/analytic.rs:82:50: replace / with % in cone_props | (a) | cone_degenerate_zero_radii |
| crates/check/src/properties/analytic.rs:82:50: replace / with * in cone_props | (a) | cone_degenerate_zero_radii |
| crates/check/src/properties/analytic.rs:87:31: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:90:24: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:90:31: replace + with - in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:90:44: replace + with - in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:90:44: replace + with * in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:90:64: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:22: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:22: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:28: replace + with - in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:28: replace + with * in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:34: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:34: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:41: replace + with - in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:41: replace + with * in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:47: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:47: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:53: replace + with - in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:53: replace + with * in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:60: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:60: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:66: replace + with - in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:66: replace + with * in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:72: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:96:72: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:97:19: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:97:19: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:97:23: replace / with % in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:97:23: replace / with * in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:97:30: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:97:30: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:97:39: replace / with % in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:97:39: replace / with * in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:102:30: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:102:30: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:102:34: replace / with % in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:102:34: replace / with * in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:102:41: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:102:41: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:102:50: replace / with % in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:102:50: replace / with * in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:9: replace + with - in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:9: replace + with * in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:13: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:13: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:22: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:22: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:31: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:31: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:38: replace + with - in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:38: replace + with * in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:44: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:44: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:51: replace + with - in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:51: replace + with * in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:57: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:57: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:64: replace / with % in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:64: replace / with * in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:72: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:103:72: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:105:30: replace - with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:105:30: replace - with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:105:34: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:105:34: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:105:42: replace * with + in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:105:42: replace * with / in cone_props | (a) | cone_frustum_exact_props |
| crates/check/src/properties/analytic.rs:121:37: replace * with / in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:121:47: replace * with / in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:124:17: replace * with + in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:124:17: replace * with / in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:124:28: replace * with + in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:124:28: replace * with / in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:124:38: replace + with - in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:124:38: replace + with * in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:124:45: replace * with + in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:124:45: replace * with / in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:124:55: replace * with + in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:124:55: replace * with / in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:126:17: replace * with + in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:126:17: replace * with / in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:126:28: replace * with + in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:126:28: replace * with / in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:126:38: replace / with % in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:126:38: replace / with * in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:126:44: replace + with - in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:126:44: replace + with * in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:126:50: replace / with % in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:126:50: replace / with * in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:126:56: replace * with + in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:126:56: replace * with / in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:126:66: replace * with + in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:126:66: replace * with / in torus_props | (a) | torus_exact_props |
| crates/check/src/properties/analytic.rs:150:5: replace cylinder_area -> f64 with 0.0 | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:150:5: replace cylinder_area -> f64 with 1.0 | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:150:5: replace cylinder_area -> f64 with -1.0 | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:150:9: replace * with + in cylinder_area | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:150:9: replace * with / in cylinder_area | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:150:14: replace * with + in cylinder_area | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:150:14: replace * with / in cylinder_area | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:150:23: replace * with + in cylinder_area | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:150:23: replace * with / in cylinder_area | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:150:32: replace + with - in cylinder_area | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:150:32: replace + with * in cylinder_area | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:150:38: replace * with + in cylinder_area | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:150:38: replace * with / in cylinder_area | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:150:43: replace * with + in cylinder_area | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:150:43: replace * with / in cylinder_area | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:150:52: replace * with + in cylinder_area | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:150:52: replace * with / in cylinder_area | (a) | cylinder_area_exact |
| crates/check/src/properties/analytic.rs:156:5: replace torus_area -> f64 with 0.0 | (a) | torus_area_exact |
| crates/check/src/properties/analytic.rs:156:5: replace torus_area -> f64 with 1.0 | (a) | torus_area_exact |
| crates/check/src/properties/analytic.rs:156:5: replace torus_area -> f64 with -1.0 | (a) | torus_area_exact |
| crates/check/src/properties/analytic.rs:156:9: replace * with + in torus_area | (a) | torus_area_exact |
| crates/check/src/properties/analytic.rs:156:9: replace * with / in torus_area | (a) | torus_area_exact |
| crates/check/src/properties/analytic.rs:156:14: replace * with + in torus_area | (a) | torus_area_exact |
| crates/check/src/properties/analytic.rs:156:14: replace * with / in torus_area | (a) | torus_area_exact |
| crates/check/src/properties/analytic.rs:156:19: replace * with + in torus_area | (a) | torus_area_exact |
| crates/check/src/properties/analytic.rs:156:19: replace * with / in torus_area | (a) | torus_area_exact |
| crates/check/src/properties/analytic.rs:156:29: replace * with + in torus_area | (a) | torus_area_exact |
| crates/check/src/properties/analytic.rs:156:29: replace * with / in torus_area | (a) | torus_area_exact |

## crates/check/src/validate/wire.rs

- before: 94 missed; after: 16 missed, 144 caught, 20 unviable, 1 timeout.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/check/src/validate/wire.rs:17:5: replace check_wire_empty -> Result<Vec<ValidationIssue>, CheckError> with Ok(vec![]) | (b) | unreachable: Wire::new rejects empty edge lists (only constructor) |
| crates/check/src/validate/wire.rs:35:5: replace check_wire_connected -> Result<Vec<ValidationIssue>, CheckError> with Ok(vec![]) | (a) | connected_gap_reported_and_triangle_clean |
| crates/check/src/validate/wire.rs:37:20: replace < with == in check_wire_connected | (a) | connected_two_edge_gap_reported |
| crates/check/src/validate/wire.rs:37:20: replace < with > in check_wire_connected | (a) | connected_gap_reported_and_triangle_clean |
| crates/check/src/validate/wire.rs:37:20: replace < with <= in check_wire_connected | (a) | connected_two_edge_gap_reported |
| crates/check/src/validate/wire.rs:65:5: replace check_wire_closure -> Result<Vec<ValidationIssue>, CheckError> with Ok(vec![]) | (a) | closure_mismatch_reported_and_open_wire_ignored |
| crates/check/src/validate/wire.rs:66:8: delete ! in check_wire_closure | (a) | closure_mismatch_reported_and_open_wire_ignored |
| crates/check/src/validate/wire.rs:96:5: replace check_wire_redundant -> Result<Vec<ValidationIssue>, CheckError> with Ok(vec![]) | (a) | redundant_triple_use_reported_and_double_use_clean |
| crates/check/src/validate/wire.rs:99:47: replace += with *= in check_wire_redundant | (a) | redundant_triple_use_reported_and_double_use_clean |
| crates/check/src/validate/wire.rs:145:5: replace check_wire_self_intersection_on_periodic_surface -> Result<Vec<ValidationIssue>, CheckError> with Ok(vec![]) | (a) | periodic_bowtie_self_intersection_reported |
| crates/check/src/validate/wire.rs:156:20: replace < with == in check_wire_self_intersection_impl | (b) | same outcome for wires with fewer than 3 edges (loop body empty) |
| crates/check/src/validate/wire.rs:156:20: replace < with <= in check_wire_self_intersection_impl | (b) | same outcome for wires with fewer than 3 edges (loop body empty) |
| crates/check/src/validate/wire.rs:164:61: replace += with *= in check_wire_self_intersection_impl | (a) | periodic_duplicate_seam_exempt_but_plain_reports |
| crates/check/src/validate/wire.rs:181:35: replace + with - in check_wire_self_intersection_impl | (b) | reversed full-circle span yields the identical segment set |
| crates/check/src/validate/wire.rs:181:35: replace + with * in check_wire_self_intersection_impl | (a) | circle_closed_loop_crossing_reported |
| crates/check/src/validate/wire.rs:189:50: replace - with / in check_wire_self_intersection_impl | (a) | circle_short_arc_span_stays_minor |
| crates/check/src/validate/wire.rs:190:32: replace <= with > in check_wire_self_intersection_impl | (a) | circle_short_arc_span_stays_minor |
| crates/check/src/validate/wire.rs:191:39: replace + with - in check_wire_self_intersection_impl | (a) | circle_ccw_arc_crossing_reported |
| crates/check/src/validate/wire.rs:191:39: replace + with * in check_wire_self_intersection_impl | (a) | circle_ccw_arc_crossing_reported |
| crates/check/src/validate/wire.rs:193:39: replace - with / in check_wire_self_intersection_impl | (a) | circle_wrapped_tip_crossing_reported |
| crates/check/src/validate/wire.rs:193:64: replace - with / in check_wire_self_intersection_impl | (a) | circle_wrapped_tip_crossing_reported |
| crates/check/src/validate/wire.rs:198:32: replace + with * in check_wire_self_intersection_impl | (a) | circle_short_arc_span_stays_minor |
| crates/check/src/validate/wire.rs:198:38: replace - with / in check_wire_self_intersection_impl | (a) | circle_short_arc_span_stays_minor |
| crates/check/src/validate/wire.rs:198:44: replace * with + in check_wire_self_intersection_impl | (a) | circle_short_arc_span_stays_minor |
| crates/check/src/validate/wire.rs:198:44: replace * with / in check_wire_self_intersection_impl | (a) | circle_short_arc_crossing_reported |
| crates/check/src/validate/wire.rs:198:57: replace / with * in check_wire_self_intersection_impl | (a) | circle_short_arc_span_stays_minor |
| crates/check/src/validate/wire.rs:201:20: delete ! in check_wire_self_intersection_impl | (b) | uniform sampling is reversal-symmetric |
| crates/check/src/validate/wire.rs:207:46: replace == with != in check_wire_self_intersection_impl | (a) | ellipse_lower_arc_clean |
| crates/check/src/validate/wire.rs:213:24: delete ! in check_wire_self_intersection_impl | (a) | ellipse_lower_arc_clean / ellipse_upper_arc_crossing_reported |
| crates/check/src/validate/wire.rs:216:27: replace <= with > in check_wire_self_intersection_impl | (a) | ellipse_lower_arc_clean / ellipse_upper_arc_crossing_reported |
| crates/check/src/validate/wire.rs:217:28: replace += with -= in check_wire_self_intersection_impl | (a) | ellipse_lower_arc_clean / ellipse_upper_arc_crossing_reported |
| crates/check/src/validate/wire.rs:217:28: replace += with *= in check_wire_self_intersection_impl | (a) | ellipse_lower_arc_clean / ellipse_upper_arc_crossing_reported |
| crates/check/src/validate/wire.rs:223:32: replace + with - in check_wire_self_intersection_impl | (a) | ellipse_lower_arc_clean / ellipse_upper_arc_crossing_reported |
| crates/check/src/validate/wire.rs:223:32: replace + with * in check_wire_self_intersection_impl | (a) | ellipse_lower_arc_clean / ellipse_upper_arc_crossing_reported |
| crates/check/src/validate/wire.rs:223:38: replace - with + in check_wire_self_intersection_impl | (a) | ellipse_lower_arc_clean / ellipse_upper_arc_crossing_reported |
| crates/check/src/validate/wire.rs:223:38: replace - with / in check_wire_self_intersection_impl | (a) | ellipse_lower_arc_clean / ellipse_upper_arc_crossing_reported |
| crates/check/src/validate/wire.rs:223:44: replace * with + in check_wire_self_intersection_impl | (a) | ellipse_lower_arc_clean / ellipse_upper_arc_crossing_reported |
| crates/check/src/validate/wire.rs:223:44: replace * with / in check_wire_self_intersection_impl | (a) | ellipse_lower_arc_clean / ellipse_upper_arc_crossing_reported |
| crates/check/src/validate/wire.rs:223:57: replace / with % in check_wire_self_intersection_impl | (a) | ellipse_lower_arc_clean / ellipse_upper_arc_crossing_reported |
| crates/check/src/validate/wire.rs:223:57: replace / with * in check_wire_self_intersection_impl | (a) | ellipse_lower_arc_clean / ellipse_upper_arc_crossing_reported |
| crates/check/src/validate/wire.rs:226:20: delete ! in check_wire_self_intersection_impl | (b) | uniform sampling is reversal-symmetric |
| crates/check/src/validate/wire.rs:239:32: replace + with - in check_wire_self_intersection_impl | (a) | hyperbola_shallow_arc_clean / hyperbola_arc_crossing_reported |
| crates/check/src/validate/wire.rs:239:32: replace + with * in check_wire_self_intersection_impl | (a) | hyperbola_half_arc_crossing_localized |
| crates/check/src/validate/wire.rs:239:38: replace - with + in check_wire_self_intersection_impl | (a) | hyperbola_shallow_arc_clean / hyperbola_arc_crossing_reported |
| crates/check/src/validate/wire.rs:239:38: replace - with / in check_wire_self_intersection_impl | (a) | hyperbola_shallow_arc_clean / hyperbola_arc_crossing_reported |
| crates/check/src/validate/wire.rs:239:44: replace * with + in check_wire_self_intersection_impl | (a) | hyperbola_shallow_arc_clean / hyperbola_arc_crossing_reported |
| crates/check/src/validate/wire.rs:239:44: replace * with / in check_wire_self_intersection_impl | (a) | hyperbola_shallow_arc_clean / hyperbola_arc_crossing_reported |
| crates/check/src/validate/wire.rs:239:57: replace / with % in check_wire_self_intersection_impl | (a) | hyperbola_shallow_arc_clean / hyperbola_arc_crossing_reported |
| crates/check/src/validate/wire.rs:239:57: replace / with * in check_wire_self_intersection_impl | (a) | hyperbola_shallow_arc_clean / hyperbola_arc_crossing_reported |
| crates/check/src/validate/wire.rs:242:20: delete ! in check_wire_self_intersection_impl | (b) | uniform sampling is reversal-symmetric |
| crates/check/src/validate/wire.rs:251:32: replace + with - in check_wire_self_intersection_impl | (a) | parabola_arc_clean / parabola_arc_crossing_reported |
| crates/check/src/validate/wire.rs:251:32: replace + with * in check_wire_self_intersection_impl | (a) | parabola_half_dip_crossing_localized |
| crates/check/src/validate/wire.rs:251:38: replace - with + in check_wire_self_intersection_impl | (a) | parabola_arc_clean / parabola_arc_crossing_reported |
| crates/check/src/validate/wire.rs:251:38: replace - with / in check_wire_self_intersection_impl | (a) | parabola_arc_clean / parabola_arc_crossing_reported |
| crates/check/src/validate/wire.rs:251:44: replace * with + in check_wire_self_intersection_impl | (a) | parabola_arc_clean / parabola_arc_crossing_reported |
| crates/check/src/validate/wire.rs:251:44: replace * with / in check_wire_self_intersection_impl | (a) | parabola_arc_clean / parabola_arc_crossing_reported |
| crates/check/src/validate/wire.rs:251:57: replace / with % in check_wire_self_intersection_impl | (a) | parabola_chord_crossing_tripwire_stays_clear |
| crates/check/src/validate/wire.rs:251:57: replace / with * in check_wire_self_intersection_impl | (a) | parabola_arc_clean / parabola_arc_crossing_reported |
| crates/check/src/validate/wire.rs:254:20: delete ! in check_wire_self_intersection_impl | (b) | uniform sampling is reversal-symmetric |
| crates/check/src/validate/wire.rs:263:32: replace + with - in check_wire_self_intersection_impl | (a) | nurbs_shallow_arc_clean / nurbs_dip_crossing_reported |
| crates/check/src/validate/wire.rs:263:32: replace + with * in check_wire_self_intersection_impl | (a) | nurbs_shallow_arc_clean / nurbs_dip_crossing_reported |
| crates/check/src/validate/wire.rs:263:38: replace - with + in check_wire_self_intersection_impl | (c) | identical iff the NURBS domain starts at t0=0 (all suite fixtures); full suite passes under the mutant (verified); killing needs a custom non-zero-start domain fixture |
| crates/check/src/validate/wire.rs:263:38: replace - with / in check_wire_self_intersection_impl | (a) | nurbs_shallow_arc_clean / nurbs_dip_crossing_reported |
| crates/check/src/validate/wire.rs:263:44: replace * with + in check_wire_self_intersection_impl | (a) | nurbs_shallow_arc_clean / nurbs_dip_crossing_reported |
| crates/check/src/validate/wire.rs:263:44: replace * with / in check_wire_self_intersection_impl | (a) | nurbs_shallow_arc_clean / nurbs_dip_crossing_reported |
| crates/check/src/validate/wire.rs:263:57: replace / with % in check_wire_self_intersection_impl | (a) | nurbs_shallow_arc_clean / nurbs_dip_crossing_reported |
| crates/check/src/validate/wire.rs:263:57: replace / with * in check_wire_self_intersection_impl | (a) | nurbs_shallow_arc_clean / nurbs_dip_crossing_reported |
| crates/check/src/validate/wire.rs:266:20: delete ! in check_wire_self_intersection_impl | (b) | uniform sampling is reversal-symmetric |
| crates/check/src/validate/wire.rs:277:21: replace + with * in check_wire_self_intersection_impl | (a) | late_index_pair_crossing_reported |
| crates/check/src/validate/wire.rs:286:17: replace && with \|\| in check_wire_self_intersection_impl | (a) | periodic_bowtie_self_intersection_reported |
| crates/check/src/validate/wire.rs:286:36: replace == with != in check_wire_self_intersection_impl | (a) | periodic_bowtie_self_intersection_reported |
| crates/check/src/validate/wire.rs:287:17: replace && with \|\| in check_wire_self_intersection_impl | (a) | periodic_reused_pair_crossing_reported |
| crates/check/src/validate/wire.rs:287:53: replace == with != in check_wire_self_intersection_impl | (a) | periodic_reused_pair_crossing_reported |
| crates/check/src/validate/wire.rs:288:17: replace && with \|\| in check_wire_self_intersection_impl | (a) | periodic_mixed_orientation_crossing_still_reports |
| crates/check/src/validate/wire.rs:288:42: replace != with == in check_wire_self_intersection_impl | (a) | periodic_mixed_orientation_crossing_still_reports |
| crates/check/src/validate/wire.rs:298:44: replace == with != in check_wire_self_intersection_impl | (a) | pinched_vertex_contact_reports_plain_but_exempt_periodic |
| crates/check/src/validate/wire.rs:298:62: replace \|\| with && in check_wire_self_intersection_impl | (a) | pinched_vertex_contact_reports_plain_but_exempt_periodic |
| crates/check/src/validate/wire.rs:298:73: replace == with != in check_wire_self_intersection_impl | (a) | pinched_vertex_contact_reports_plain_but_exempt_periodic |
| crates/check/src/validate/wire.rs:312:29: replace < with <= in check_wire_self_intersection_impl | (c) | exact dist==tol boundary; no stable fixture |
| crates/check/src/validate/wire.rs:319:61: replace < with == in check_wire_self_intersection_impl | (a) | pinched_vertex_contact_reports_plain_but_exempt_periodic |
| crates/check/src/validate/wire.rs:319:61: replace < with > in check_wire_self_intersection_impl | (a) | pinched_vertex_contact_reports_plain_but_exempt_periodic |
| crates/check/src/validate/wire.rs:319:61: replace < with <= in check_wire_self_intersection_impl | (c) | exact-threshold boundary class |
| crates/check/src/validate/wire.rs:320:33: replace && with \|\| in check_wire_self_intersection_impl | (c) | widened exemption unobservable on line-only geometry |
| crates/check/src/validate/wire.rs:320:65: replace < with == in check_wire_self_intersection_impl | (a) | pinched_vertex_contact_reports_plain_but_exempt_periodic |
| crates/check/src/validate/wire.rs:320:65: replace < with > in check_wire_self_intersection_impl | (a) | pinched_vertex_contact_reports_plain_but_exempt_periodic |
| crates/check/src/validate/wire.rs:320:65: replace < with <= in check_wire_self_intersection_impl | (c) | exact-threshold boundary class |
| crates/check/src/validate/wire.rs:352:25: replace != with == in collinear_boundary_groups::root | (timeout) | union-find root loops forever under ==; suite hangs |
| crates/check/src/validate/wire.rs:364:13: replace \|\| with && in collinear_boundary_groups | (a) | collinear_groups_keep_arc_edge_separate |
| crates/check/src/validate/wire.rs:365:13: replace \|\| with && in collinear_boundary_groups | (a) | collinear_groups_keep_disconnected_gap_separate |
| crates/check/src/validate/wire.rs:374:33: replace \|\| with && in collinear_boundary_groups | (b) | degenerate-length guard reconverges via the non-finite-fraction check |
| crates/check/src/validate/wire.rs:377:53: replace / with % in collinear_boundary_groups | (a) | collinear_midpoint_subdivision_stays_single_group |
| crates/check/src/validate/wire.rs:377:53: replace / with * in collinear_boundary_groups | (a) | collinear_midpoint_subdivision_stays_single_group |
| crates/check/src/validate/wire.rs:385:44: replace * with / in collinear_boundary_groups | (a) | collinear_near_roundoff_bump_still_merges |
| crates/check/src/validate/wire.rs:386:60: replace > with >= in collinear_boundary_groups | (b) | exact deviation==roundoff float equality unreachable |

## crates/check/src/validate/face.rs

- before: 74 missed; after: 18 missed, 87 caught, 9 unviable, 0 timeout.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/check/src/validate/face.rs:16:5: replace check_face_has_surface -> Result<Vec<ValidationIssue>, CheckError> with Ok(vec![]) | (a) | face_has_surface_errors_for_unknown_face |
| crates/check/src/validate/face.rs:58:20: replace < with <= in check_face_orientation | (a) | face_orientation_threshold_is_strict_at_minus_tenth / face_orientation_passes_for_near_perpendicular_normal |
| crates/check/src/validate/face.rs:58:22: delete - in check_face_orientation | (a) | face_orientation_threshold_is_strict_at_minus_tenth / face_orientation_passes_for_near_perpendicular_normal |
| crates/check/src/validate/face.rs:134:44: replace / with * in check_planar_inner_wire_orientation | (a) | planar_inner_wire_tiny_hole_same_wound_fires |
| crates/check/src/validate/face.rs:134:69: replace * with + in check_planar_inner_wire_orientation | (a) | planar_inner_wire_tiny_hole_same_wound_fires |
| crates/check/src/validate/face.rs:134:69: replace * with / in check_planar_inner_wire_orientation | (a) | planar_inner_wire_tiny_hole_same_wound_fires |
| crates/check/src/validate/face.rs:135:22: replace > with >= in check_planar_inner_wire_orientation | (b) | alignment exactly 0.1 is a measure-zero float boundary |
| crates/check/src/validate/face.rs:182:9: delete match arm remus_topology::face::FaceSurface::Torus(_) in check_periodic_inner_wire_orientation | (a) | torus_hole_crossing_v_seam_keeps_its_winding_verdict |
| crates/check/src/validate/face.rs:202:52: replace - with + in check_periodic_inner_wire_orientation | (a) | torus_hole_crossing_v_seam_keeps_its_winding_verdict |
| crates/check/src/validate/face.rs:202:52: replace - with / in check_periodic_inner_wire_orientation | (a) | torus_hole_crossing_v_seam_keeps_its_winding_verdict |
| crates/check/src/validate/face.rs:218:35: replace - with + in check_periodic_inner_wire_orientation | (a) | cylinder_same_direction_rings_fire(_shifted_start) |
| crates/check/src/validate/face.rs:218:35: replace - with / in check_periodic_inner_wire_orientation | (a) | cylinder_same_direction_rings_fire(_shifted_start) |
| crates/check/src/validate/face.rs:219:38: replace - with + in check_periodic_inner_wire_orientation | (a) | cylinder_same_direction_rings_fire(_shifted_start) |
| crates/check/src/validate/face.rs:219:38: replace - with / in check_periodic_inner_wire_orientation | (a) | cylinder_same_direction_rings_fire(_shifted_start) |
| crates/check/src/validate/face.rs:220:23: replace > with == in check_periodic_inner_wire_orientation | (a) | cylinder_opposite_direction_rings_pass |
| crates/check/src/validate/face.rs:220:23: replace > with < in check_periodic_inner_wire_orientation | (a) | cylinder_opposite_direction_rings_pass |
| crates/check/src/validate/face.rs:220:23: replace > with >= in check_periodic_inner_wire_orientation | (b) | ring metric 5.50 vs bar 4.71, never equal |
| crates/check/src/validate/face.rs:220:30: replace * with + in check_periodic_inner_wire_orientation | (a) | cylinder_opposite_direction_rings_pass |
| crates/check/src/validate/face.rs:220:30: replace * with / in check_periodic_inner_wire_orientation | (b) | provably equivalent: \|closing\|==\|progress\| always, lowering one bar never flips a verdict |
| crates/check/src/validate/face.rs:220:41: replace && with \|\| in check_periodic_inner_wire_orientation | (b) | both operands identical |
| crates/check/src/validate/face.rs:220:61: replace > with == in check_periodic_inner_wire_orientation | (a) | cylinder_opposite_direction_rings_pass |
| crates/check/src/validate/face.rs:220:61: replace > with < in check_periodic_inner_wire_orientation | (a) | cylinder_opposite_direction_rings_pass |
| crates/check/src/validate/face.rs:220:61: replace > with >= in check_periodic_inner_wire_orientation | (b) | never equal (same argument) |
| crates/check/src/validate/face.rs:220:68: replace * with + in check_periodic_inner_wire_orientation | (a) | cylinder_opposite_direction_rings_pass |
| crates/check/src/validate/face.rs:220:68: replace * with / in check_periodic_inner_wire_orientation | (b) | same \|closing\|==\|progress\| equivalence |
| crates/check/src/validate/face.rs:221:47: replace > with == in check_periodic_inner_wire_orientation | (a) | cylinder_opposite_direction_rings_pass |
| crates/check/src/validate/face.rs:221:47: replace > with < in check_periodic_inner_wire_orientation | (b) | flips both ring directions, pairwise equality preserved |
| crates/check/src/validate/face.rs:221:47: replace > with >= in check_periodic_inner_wire_orientation | (b) | differs only at progress == 0, impossible for a ring |
| crates/check/src/validate/face.rs:224:39: replace - with + in check_periodic_inner_wire_orientation | (a) | torus_same_direction_rings_fire(_shifted_start) |
| crates/check/src/validate/face.rs:224:39: replace - with / in check_periodic_inner_wire_orientation | (a) | torus_same_direction_rings_fire(_shifted_start) |
| crates/check/src/validate/face.rs:224:46: replace - with + in check_periodic_inner_wire_orientation | (a) | torus_same_direction_rings_fire(_shifted_start) |
| crates/check/src/validate/face.rs:224:46: replace - with / in check_periodic_inner_wire_orientation | (a) | torus_same_direction_rings_fire(_shifted_start) |
| crates/check/src/validate/face.rs:225:35: replace - with + in check_periodic_inner_wire_orientation | (a) | torus_same_direction_rings_fire(_shifted_start) |
| crates/check/src/validate/face.rs:225:35: replace - with / in check_periodic_inner_wire_orientation | (a) | torus_same_direction_rings_fire(_shifted_start) |
| crates/check/src/validate/face.rs:225:42: replace - with + in check_periodic_inner_wire_orientation | (a) | torus_same_direction_rings_fire(_shifted_start) |
| crates/check/src/validate/face.rs:225:42: replace - with / in check_periodic_inner_wire_orientation | (a) | torus_same_direction_rings_fire(_shifted_start) |
| crates/check/src/validate/face.rs:226:27: replace > with == in check_periodic_inner_wire_orientation | (a) | torus_opposite_direction_rings_pass |
| crates/check/src/validate/face.rs:226:27: replace > with < in check_periodic_inner_wire_orientation | (a) | torus_opposite_direction_rings_pass |
| crates/check/src/validate/face.rs:226:27: replace > with >= in check_periodic_inner_wire_orientation | (b) | never equal (same argument) |
| crates/check/src/validate/face.rs:226:34: replace * with + in check_periodic_inner_wire_orientation | (a) | torus_opposite_direction_rings_pass |
| crates/check/src/validate/face.rs:226:34: replace * with / in check_periodic_inner_wire_orientation | (b) | same \|closing\|==\|progress\| equivalence |
| crates/check/src/validate/face.rs:226:43: replace && with \|\| in check_periodic_inner_wire_orientation | (b) | identical operands |
| crates/check/src/validate/face.rs:226:63: replace > with == in check_periodic_inner_wire_orientation | (a) | torus_opposite_direction_rings_pass |
| crates/check/src/validate/face.rs:226:63: replace > with < in check_periodic_inner_wire_orientation | (a) | torus_opposite_direction_rings_pass |
| crates/check/src/validate/face.rs:226:63: replace > with >= in check_periodic_inner_wire_orientation | (b) | never equal (same argument) |
| crates/check/src/validate/face.rs:226:70: replace * with + in check_periodic_inner_wire_orientation | (a) | torus_opposite_direction_rings_pass |
| crates/check/src/validate/face.rs:226:70: replace * with / in check_periodic_inner_wire_orientation | (b) | same \|closing\|==\|progress\| equivalence |
| crates/check/src/validate/face.rs:227:51: replace > with == in check_periodic_inner_wire_orientation | (a) | torus_opposite_direction_rings_pass |
| crates/check/src/validate/face.rs:227:51: replace > with < in check_periodic_inner_wire_orientation | (b) | double-flip preserves equality |
| crates/check/src/validate/face.rs:227:51: replace > with >= in check_periodic_inner_wire_orientation | (b) | == 0 impossible |
| crates/check/src/validate/face.rs:230:52: replace > with < in check_periodic_inner_wire_orientation | (b) | global Area-sign flip preserves every pairwise verdict |
| crates/check/src/validate/face.rs:230:52: replace > with >= in check_periodic_inner_wire_orientation | (b) | differs only at double_area == 0.0, already Degenerate |
| crates/check/src/validate/face.rs:279:20: replace < with == in raw_newell_normal | (a) | raw_newell_normal_rejects_short_and_degenerate_polygons |
| crates/check/src/validate/face.rs:279:20: replace < with <= in raw_newell_normal | (a) | raw_newell_normal_rejects_short_and_degenerate_polygons |
| crates/check/src/validate/face.rs:288:16: replace += with -= in raw_newell_normal | (a) | raw_newell_normal_matches_hand_computed_values |
| crates/check/src/validate/face.rs:289:26: replace - with + in raw_newell_normal | (a) | raw_newell_normal_matches_hand_computed_values |
| crates/check/src/validate/face.rs:289:38: replace * with + in raw_newell_normal | (a) | raw_newell_normal_matches_hand_computed_values |
| crates/check/src/validate/face.rs:289:53: replace + with - in raw_newell_normal | (a) | raw_newell_normal_matches_hand_computed_values |
| crates/check/src/validate/face.rs:289:53: replace + with * in raw_newell_normal | (a) | raw_newell_normal_matches_hand_computed_values |
| crates/check/src/validate/face.rs:290:26: replace - with + in raw_newell_normal | (a) | raw_newell_normal_matches_hand_computed_values |
| crates/check/src/validate/face.rs:290:38: replace * with + in raw_newell_normal | (a) | raw_newell_normal_matches_hand_computed_values |
| crates/check/src/validate/face.rs:290:53: replace + with - in raw_newell_normal | (a) | raw_newell_normal_matches_hand_computed_values |
| crates/check/src/validate/face.rs:290:53: replace + with * in raw_newell_normal | (a) | raw_newell_normal_matches_hand_computed_values |
| crates/check/src/validate/face.rs:291:26: replace - with + in raw_newell_normal | (a) | raw_newell_normal_matches_hand_computed_values |
| crates/check/src/validate/face.rs:291:38: replace * with + in raw_newell_normal | (a) | raw_newell_normal_matches_hand_computed_values |
| crates/check/src/validate/face.rs:291:53: replace + with * in raw_newell_normal | (a) | raw_newell_normal_matches_hand_computed_values |
| crates/check/src/validate/face.rs:295:25: replace && with \|\| in raw_newell_normal | (a) | raw_newell_normal_rejects_short_and_degenerate_polygons |
| crates/check/src/validate/face.rs:295:35: replace > with >= in raw_newell_normal | (b) | length exactly 1e-30 unreachable |
| crates/check/src/validate/face.rs:304:20: replace / with % in polygon_centroid | (a) | polygon_centroid_matches_hand_computed_average |
| crates/check/src/validate/face.rs:304:20: replace / with * in polygon_centroid | (a) | polygon_centroid_matches_hand_computed_average |
| crates/check/src/validate/face.rs:304:28: replace / with % in polygon_centroid | (a) | polygon_centroid_matches_hand_computed_average |
| crates/check/src/validate/face.rs:304:28: replace / with * in polygon_centroid | (a) | polygon_centroid_matches_hand_computed_average |
| crates/check/src/validate/face.rs:304:36: replace / with % in polygon_centroid | (a) | polygon_centroid_matches_hand_computed_average |
| crates/check/src/validate/face.rs:304:36: replace / with * in polygon_centroid | (a) | polygon_centroid_matches_hand_computed_average |

## Untouched files (not in this PR's scope)

| File | Mutants | Missed before |
| --- | ---: | ---: |
| `crates/check/src/analyze/curvature.rs` | 159 | 93 |
| `crates/check/src/classify/boundary.rs` | 286 | 163 |
| `crates/check/src/classify/mod.rs` | 82 | 49 |
| `crates/check/src/classify/ray_surface.rs` | 235 | 114 |
| `crates/check/src/classify/winding.rs` | 92 | 32 |
| `crates/check/src/distance/analytic.rs` | 24 | 1 |
| `crates/check/src/distance/edge.rs` | 3 | 0 |
| `crates/check/src/distance/mod.rs` | 165 | 130 |
| `crates/check/src/error.rs` | 1 | 0 |
| `crates/check/src/properties/accumulator.rs` | 169 | 90 |
| `crates/check/src/properties/bbox.rs` | 1 | 0 |
| `crates/check/src/properties/face_integrator.rs` | 1476 | 942 |
| `crates/check/src/properties/mod.rs` | 120 | 32 |
| `crates/check/src/util.rs` | 310 | 202 |
| `crates/check/src/validate/checks.rs` | 10 | 4 |
| `crates/check/src/validate/edge.rs` | 70 | 34 |
| `crates/check/src/validate/finite.rs` | 21 | 9 |
| `crates/check/src/validate/mod.rs` | 85 | 67 |
| `crates/check/src/validate/shell.rs` | 56 | 22 |
| `crates/check/src/validate/solid.rs` | 14 | 4 |
| `crates/check/src/validate/vertex.rs` | 32 | 7 |

## Notes

- `analytic.rs` had zero unit tests; all 162 survivors were distinguishable arithmetic, killed by
  exact closed-form assertions (1e-12 relative). Fixture-value traps avoided: radii != 0,1 and
  torus R=4 (at R=3, `R*R/2` -> `R+R/2` is a numerical coincidence, caught during verification).
- `wire.rs` 207:46 (`==` -> `!=`): first called equivalent, refuted by the after-run — killed by
  `ellipse_lower_arc_clean`. Four agent-claimed kills did not reproduce in the after-run; three were
  re-killed with localizing fixtures (`hyperbola_half_arc_crossing_localized`,
  `parabola_half_dip_crossing_localized`, `parabola_chord_crossing_tripwire_stays_clear`), the fourth
  (263:38) is residual: identical iff the NURBS domain starts at t0=0.
- `face.rs` after-run reproduces the triage exactly (18 predicted equivalents survive, all 56
  claimed kills caught). Two candidate triangular-hole tests were removed before landing after
  analysis proved their targets equivalent.
- The `//`-threshold and exact-equality residuals (wire 312/319/320/386, face 135/220/221/226/227/230/295)
  are measure-zero float boundaries no stable fixture can pin; recorded, not killed.
- Raw logs: crate run + per-file after-runs (worktree `mutants.out*` dirs, untracked, not committed).
  Reproduce: `cargo mutants -p remus-check --no-config --in-place --baseline skip -f <file> -- -p remus-check`.
