# Fixed-order curved-face area: native evidence

The area getter for analytic curved faces now uses the existing face dispatch,
trim preparation, quadrature order, sample order, and orientation rule, but
accumulates only area at each Gauss point. Full mass properties still accumulate
all moments. Planar and NURBS `face_area` routing is unchanged. This is a
fixed-order path; it does not substitute adaptive mass-property output for the
area getter.

Base source is `fe3c8efaef2d041fbc3303ac6a1ac6841aa7ac62`. The candidate
is the `codex/geometry-latency` source diff in `face_integrator.rs`,
`face_integrator/adaptive.rs`, and `measure/area.rs`. Measurements used Rust
1.96.0, Linux x86_64, AMD Ryzen 9 5900XT, the release profile,
`CARGO_BUILD_JOBS=2`, and `RAYON_NUM_THREADS=1`. The previous diagnostic archive
had left instrumented `remus-topology` and `remus-algo` objects in the shared
release target, so those two packages were explicitly cleaned before this
qualifying run. The [raw clean-run log](face-area-criterion.txt) has no
diagnostic timer lines.

```sh
cargo clean --release -p remus-topology -p remus-algo
CARGO_BUILD_JOBS=2 RAYON_NUM_THREADS=1 cargo bench --offline --locked \
  -p remus-operations --bench cad_operations -- \
  'fillet curved face area' --noplot
```

The maintained Criterion benchmark builds one 20 mm box and fillets one edge
before timing. Both functions measure the same cylindrical blend face with
Gauss order 8. Its 20-sample estimates and confidence bounds were:

| Getter path | Criterion estimate and bounds |
| --- | ---: |
| Full face contribution, selecting area | 1.9670 ms [1.9369, 2.0002] |
| Area-only contribution | 0.79225 ms [0.78292, 0.80016] |

The area-only estimate is about 60% lower for this getter. This is a paired
native microbenchmark, not a measured fillet-operation or browser speedup.
Criterion's `change` lines in the raw log compare each path with a preceding
run that had diagnostic dependencies; the valid comparison here is between the
two paths in the clean run. CPU affinity and host activity were not controlled.

The focused parity tests compare exact `f64` bits against the old full area
result on cylinder, cone, sphere, and torus primitives, large translations,
face reversal, a trimmed cylinder patch with a hole, a full-revolution wall
with a hole, closed-edge bands, and torus tube-wrapping bands. The excessive
4,096-point trim case retains its error text. Existing mass-property adaptive
integration remains on the full contribution path. These tests and the
benchmark do not establish every possible imported trim or a whole-operation
speedup.

The separate complex-plate diagnostic identified PaveFiller and unassigned
boolean healing/acceptance work as future attribution targets. Its Builder
timing includes classification and cannot establish that classification itself
is a hotspot. No Boolean acceptance, validation, tolerance, or rollback change
is justified by that coarse breakdown.
