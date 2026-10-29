# Preliminary torus raw-sample reuse qualification

Tested working-tree source cfe5977992fb4ae2580844c2140a9e0f41892b4590316c9308ef7c9db130d235. This receipt deliberately stops before the root owner's subsequent test-only lint fixes, source commit, full workspace tests and canonical package qualification.

The baseline-equivalent uncached control passed the same polygon/logical-order comparisons but failed the desired work guard with1788/1788 evaluations (expected exit101). The carried candidate passed the exact1023/1788 work guard. Default full classifier47/47 passed its production-identical predecessor6098; final perf-counters47/47 and default final domain differential1/1 passed sourcecfe. Rustfmt check passed. Independent reviewer reported no correctness findings.

The4 actual holder torus boundaries compare bit-identically against literal original algorithms, including polygon, centre, margin, and every logical parameter/anchor call. Raw evaluations changed45482→26129,44033→25301,876→612,876→612: total91267→52654. These counts do not establish elapsed or shipping instruction speedups. Tests also cover signed-zero/ULP parameter tags, finite non-dyadic and overflowed midpoint mismatches, reversed/close/extreme domains, periodic anchors, and unchanged refusal/partial polygon/margin behavior at resource limits.

Only crates/algo/src/classifier/torus_patch.rs was edited by this agent. No dependency, default, tolerance, resource-budget or public API changes. Cargo target was released; no further work is queued. Final-source and packaged performance evidence belongs to the coordinating task.
