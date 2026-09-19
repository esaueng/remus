| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| `crates/geometry/src/sampling/arc_length.rs:45:21: replace + with - in sample_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:45:32: replace * with + in sample_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:45:41: replace - with + in sample_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:51:28: replace - with + in sample_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle (needs a fixture whose z varies) |
| `crates/geometry/src/sampling/arc_length.rs:52:32: replace + with - in sample_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle (needs dz != 0) |
| `crates/geometry/src/sampling/arc_length.rs:52:17: replace * with / in sample_arc_length` | (a) | arc_length_beats_uniform_parameter_on_an_ellipse (chord becomes ~constant, degrading to uniform-parameter sampling, which is invisible on a circle) |
| `crates/geometry/src/sampling/arc_length.rs:52:27: replace * with / in sample_arc_length` | (a) | arc_length_beats_uniform_parameter_on_an_ellipse (chord becomes ~constant, degrading to uniform-parameter sampling, which is invisible on a circle) |
| `crates/geometry/src/sampling/arc_length.rs:52:37: replace * with + in sample_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle (needs dz != 0) |
| `crates/geometry/src/sampling/arc_length.rs:66:32: replace - with + in sample_arc_length` | (b) | `i == n + 1` is never true, so `target` becomes `total_len*(n-1)/(n-1)` == `total_len`; and `t_final` at `i == n-1` is snapped to `t_end` by a separate comparison (line 97) regardless. |
| `crates/geometry/src/sampling/arc_length.rs:66:32: replace - with / in sample_arc_length` | (b) | `i == n / 1` is never true for `i in 0..n`; same reasoning as the `-` -> `+` variant at this site. |
| `crates/geometry/src/sampling/arc_length.rs:74:37: replace < with <= in sample_arc_length` | (b) | `seg_idx` shifts only when a target lands exactly on a chord-table knot; both brackets bisect to that same parameter, and the `i == 0` / `i == n-1` cases are clamped and snapped identically. |
| `crates/geometry/src/sampling/arc_length.rs:76:23: replace - with + in sample_arc_length` | (b) | The `.min(segs - 1)` clamp never binds: `target <= total_len == arc_table[segs]`, so `partition_point <= segs` and `seg_idx <= segs - 1` already. |
| `crates/geometry/src/sampling/arc_length.rs:76:23: replace - with / in sample_arc_length` | (b) | Same dead clamp: `.min(segs)` and `.min(segs - 1)` both leave the already-bounded `seg_idx` unchanged. |
| `crates/geometry/src/sampling/arc_length.rs:79:36: replace + with * in sample_arc_length` | (b) | `s1` is read only by the `t_approx` computation, which is discarded at line 101. |
| `crates/geometry/src/sampling/arc_length.rs:81:34: replace + with * in sample_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle (`t1` collapses to `t0`, so bisection returns the left edge of the chord-table segment: a ~1.2% angle-step error) |
| `crates/geometry/src/sampling/arc_length.rs:85:43: replace < with == in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:85:43: replace < with > in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:85:43: replace < with <= in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:85:31: replace - with + in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:85:31: replace - with / in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:88:16: replace + with - in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:88:16: replace + with * in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:88:44: replace * with + in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:88:44: replace * with / in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:88:32: replace / with % in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:88:32: replace / with * in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:88:26: replace - with + in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:88:26: replace - with / in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:88:38: replace - with + in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:88:38: replace - with / in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:88:50: replace - with + in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:88:50: replace - with / in sample_arc_length` | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| `crates/geometry/src/sampling/arc_length.rs:133:28: replace - with + in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle (off-origin centre makes `x + x` differ from `x - x`) |
| `crates/geometry/src/sampling/arc_length.rs:133:28: replace - with / in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:134:28: replace - with + in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle (off-origin centre makes `y + y` differ from `y - y`) |
| `crates/geometry/src/sampling/arc_length.rs:134:28: replace - with / in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:135:28: replace - with + in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle (off-origin centre makes `z + z` differ from `z - z`) |
| `crates/geometry/src/sampling/arc_length.rs:135:28: replace - with / in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:136:40: replace + with - in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:136:40: replace + with * in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:136:30: replace + with - in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:136:30: replace + with * in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:136:25: replace * with + in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:136:25: replace * with / in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:136:35: replace * with + in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:136:35: replace * with / in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:136:45: replace * with + in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle (needs dz != 0) |
| `crates/geometry/src/sampling/arc_length.rs:136:45: replace * with / in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle (needs dz != 0) |
| `crates/geometry/src/sampling/arc_length.rs:137:36: replace + with - in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:137:36: replace + with * in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:139:23: replace < with == in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:139:23: replace < with > in bisect_arc_length` | (a) | arc_length_on_tilted_circle_is_equal_angle |
| `crates/geometry/src/sampling/arc_length.rs:139:23: replace < with <= in bisect_arc_length` | (b) | Flips only on the exact float equality `arc_at_mid == target_arc` (measure zero); the bracket converges to the same root either way. |
