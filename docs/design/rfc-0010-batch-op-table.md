# RFC 0010 Declarative batch op table

Status: proposed (design only, no production code in this PR).
Baseline: `origin/main` at `c00e940` (#1358, the kernel class rename to
`RemusKernel`) on 2026-10-30. Line numbers identify that baseline; the named
arm or function is authoritative when later edits move them.

Tier: core. This changes how `RemusKernel` dispatches `executeBatch` and
`executeBatchV2`, and both read and write the live session (handles,
journal, rollback scopes). No new capability, no JS-visible behaviour change
in the migration itself.

## Summary

`executeBatch` / `executeBatchV2` reach 216 operations. Each one is
hand-written up to four times: a direct `#[wasm_bindgen]` method, a match arm
in `dispatch_op` (or one of six sub-dispatchers), an entry in
`batch_op_kind`, and, for some, a name in the rollback-scope selection in
`dispatch_with_rollback`. About half of the arms re-implement argument
decoding, validation and handle resolution instead of calling the direct
method's body. The copies have drifted. This RFC:

1. surveys every batch arm against its direct twin and sorts the differences
   into deliberate ones (with the test or doc that pins each) and accidental
   drift (with a concrete input and the output observed today);
2. proposes a `macro_rules!` op table that generates the batch registry,
   op-kind classification, rollback selection and, for twins that are really
   the same operation, the direct method shim, with JS-visible behaviour kept
   byte-identical;
3. proves each migration stage with a differential oracle that runs the
   hand-written dispatcher, kept verbatim, in lockstep with the generated one;
4. estimates size and speed effects;
5. stages the migration by op family.

Fixing the drift is **out of scope** for the migration. Each fix is a
deliberate JS-visible change and gets its own PR, owner sign-off and oracle
exemption (stage 11).

## 1. Survey

### 1.1 Inventory

Counted on the baseline with the scripts noted below (to reproduce: list the
quoted names in `batch_op_kind`, the top-level arms of `dispatch_op`, and the
arms of each `dispatch_*_op` it falls through to).

| Quantity | Count |
| --- | ---: |
| `js_name` exports on the kernel | 414 |
| Ops reachable through `executeBatch` | 216 (212 shipped + 4 behind the `io` feature) |
| Top-level `dispatch_op` arms | 152 ops in 126 arms |
| Ops in the six sub-dispatchers (`naming`, `evolution`, `gcs`, `linked_sketch`, `project_curve`, `nurbs_reuse`) | 64 |
| Batch ops with a same-named direct export | 213 |
| Batch ops without one | 3: `transform` (direct `transformSolid`, which returns `()`), `solidEdges`, `getFaceVertexPositions` |
| Batch ops whose arm calls the direct body (`*_impl`, `*_json`, a shared result builder) | about 102: all 64 sub-dispatched ops and 38 top-level arms |
| Batch ops whose arm re-implements decode, validation and resolution | about 114 |

The four name lists (`batch_op_kind`, the arms, the sub-dispatcher arms and
the rollback-scope selection) agree today: every name in `batch_op_kind`
has a handler and every handler is classified. Nothing enforces that except
review and the unknown-operation fallthrough.

### 1.2 Axes of difference

For each batch arm and its direct twin, the survey compared:

| Axis | Batch side | Direct side |
| --- | --- | --- |
| Argument decoding | JSON object, named keys, `get_f64` / `get_u32` / `get_u32_array(_optional)` / `get_optional_*` (`helpers.rs`); first failing key wins, in the arm's read order | Positional typed parameters converted by wasm-bindgen; no decode errors reach Rust |
| Optional-arg defaults | Applied in the arm (`unwrap_or(16)` etc.); absent or `null` means default, a present wrong type is refused (WASM-5) | Usually a required parameter, sometimes `Option<T>` |
| Validation | Often only what the engine checks | Often `validate_positive` / `validate_finite` / work caps before the engine |
| Handle resolution | `resolve_*` interleaved with decoding in arm order | `resolve_*` after validation |
| Error mapping | `StructuredWasmError`: legacy contract emits `message`; V2 emits `code`, `category`, `message`, `details` (+`operation`, `operationIndex`) | `JsError` from `WasmError` / engine error `Display`; blends prefix the kernel code into the message |
| Rollback scope | `dispatch_with_rollback`: read-only `Rc` share; `Exact` for `fuse`/`cut`/`intersect`; `Restored` for every other mutating op; extra checkpoint-sharing wrappers for the sweep family, `imprintPlanarPolygon`, `projectCurveOntoFace` | Per method: none (the engine's own native rollback), `with_topology_transaction` (`Restored`), or the same sweep wrapper |
| Result encoding | `serde_json::Value`; `json!` objects serialize with **sorted keys** (`serde_json` without `preserve_order`) | `u32`, `Vec<…>`, `JsValue` JSON strings built by `format!` or `serde_json::to_string`, tsify types |

### 1.3 Deliberate differences

Each is pinned by a test or written down; the op table must be able to
express every one (section 2.4).

| # | Difference | Ops | Pinned by |
| --- | --- | --- | --- |
| K1 | Batch addresses curves by edge handle where direct takes raw NURBS arrays (`pathEdge`, `spineEdge`, `auxEdge`) | `sweep`, `pipe`, `guidedSweep`, `multiSectionSweep` | `batch_optional_args_tests.rs` fixture (`spineEdge`), `sweep_transaction_tests.rs` |
| K2 | Batch defaults for arguments the direct method requires: `segments` 16, extrude direction (0,0,1)·1, revolve axis origin/Z, mirror/section/split plane, pattern directions, draft pull/neutral, `classifyPoint`/`healSolid`/`repairSolid` tolerance 1e-7, `convertToElementary` tolerance `TOL`, `offsetFace` samples 16, `projectEdges`/`projectSilhouettes` `hiddenLines` true, `deflection` 0.1 | about 30 | `batch_optional_args_tests.rs` (WASM-5: absent → default, present-but-malformed → refused); `batch_deflection_defaults_only_when_omitted` |
| K3 | Legacy batch errors are bare strings; V2 adds category, code and details; messages identical across the two | all | `legacy_errors_remain_strings_with_unchanged_messages`, `both_contracts_share_successes_and_error_messages`, `typed_mapping_never_changes_the_existing_display_message` (batch.rs tests) |
| K4 | Blend refusals: batch carries `details.kernelCode`; direct prefixes the code into the message (`fillet_failure_js_error`) | `fillet*`, `chamfer*`, `faceFaceBlend*` | `fillet_fail_closed_tests.rs` module doc and assertions |
| K5 | `getFaceMinRadius` returns `{minRadius: null, isInfinite}` (JSON has no `Infinity`); direct returns `f64` | `getFaceMinRadius` | comment in the arm (batch.rs:2699) |
| K6 | Typed twins: a batch decode failure becomes the twin's typed error **envelope** under `ok`, not a batch `error` | `*Detailed` (11 ops) | comments at batch.rs:1648 and 1760; `qualify_ops_tests.rs` |
| K7 | `gcsAddConstraint` takes the constraint as an inline object, the direct method as a JSON string | `gcsAddConstraint` | `dispatch_gcs_op` doc; B75 parity tests at batch.rs:5101 |
| K8 | Rollback: only `fuse`/`cut`/`intersect` rewind exactly; every other mutating op leaves what the old full-copy restore left (`Refusal::Restored`) | all mutating | `batch_rollback_oracle_tests.rs` (verbatim legacy dispatcher), `Refusal` doc (kernel.rs:123) |
| K9 | A failed item never stops later items; caps of 16 MiB / 10,000 ops apply before parsing | all | `malformed_batch_corpus_returns_json_and_never_stops_later_items`, `rejects_too_many_operations_before_dispatch` |
| K10 | `ReadOnly` is a performance hint; an op that does mutate is still restored | read-only set | `read_only_hint_still_restores_an_op_that_mutated` |
| K11 | Batch `split` returns `{positive, negative}`; direct returns `[positive, negative]`. Batch `*WithEvolution` returns an object with sorted keys; direct a `format!` JSON string with `solid` first | `split`, `*WithEvolution` | `qualify_section_split_tests.rs` (batch shape); the direct shapes are the shipped `.d.ts` |
| K12 | Non-finite numbers cannot reach a batch arm: `serde_json` refuses `1e400` as `invalid JSON: number out of range`. Direct `validate_finite` calls are therefore no-ops on batch input, and sharing them is behaviour-neutral | all | observed (probe below); `malformed_batch_corpus_…` |

### 1.4 Suspected accidental drift

Observed on the baseline through `executeBatch` / `executeBatchV2` (native,
`cargo test -p remus-wasm`, with a throwaway probe test that is not part of
this PR). Direct-side behaviour is read from the code. Where the direct call
succeeds it was also run natively; direct error paths cannot run natively
(`JsError::new` panics off-wasm). Box fixtures are `makeBox` 2×2×2 at the
origin.

| # | Severity | Ops | Input | Batch today | Direct today |
| --- | --- | --- | --- | --- | --- |
| D1 | **High**: wrong answer | `getFaceNormal` | Box 2³ `cut` by box 1×1×4; query each result face | Two inner walls report the **stored** plane normal, e.g. `[1,0,0]` and `[0,1,0]`, which points into the material (the face is reversed) | `effective_plane_normal`: `[-1,0,0]`, `[0,-1,0]` |
| D1b | Medium | `getFaceNormal` | `makeCylinder` r=1 h=2, lateral face | `ok: [0,1,0]` (surface normal at (u,v)=(0,0)) | Refuses: "invalid input: getFaceNormal only works on planar faces" |
| D2 | Medium | `revolve` (legacy arm) | `{"face": f, "angle": 360.00000000005, "originX": 10}` | `ok`: a full revolve (the engine accepts it within angular tolerance) | Refuses: "invalid input: angle_degrees must be in (0, 360], got 360.00000000005"; so does batch `revolveDetailed` |
| D2b | Low | `revolve` (legacy arm) | `"angle": 720` or `-90` | Refuses with the angle in **radians** for a degrees argument: "revolve angle must be in (0, 2π], got 12.566370614359172" | "invalid input: angle_degrees must be in (0, 360], got 720" |
| D3 | Medium (direct side fails open) | `loftWithOptions` | two unit squares at z=0 and z=1, `options: "{not json"` | Refuses: "missing or invalid 'options' object" | `Ok` with default options: `serde_json::from_str(options).unwrap_or(Null)` (operations.rs:433) |
| D4 | Low (message only) | `makeBox`, `makeCylinder`, `makeSphere`, `makeCone`, `makeTorus`, `fillet`, `filletV2`, `chamfer`, `chamferV2`, `chamferDistanceAngle`, `resizeCylindricalFace`, `linearPattern`, `gridPattern` | e.g. `makeBox` width −1; `fillet` radius −1; `linearPattern` spacing 0 | The engine refuses: "invalid input: box dimensions must be positive, got (-1, 1, 1)", "invalid input: radius must be finite and positive, got -1", "invalid input: pattern spacing must be finite and positive, got 0" | The binding refuses first with `validate_positive`: "invalid input: dx must be positive, got -1", "invalid input: radius must be positive, got -1", "invalid input: spacing must be positive, got 0" |
| D5 | Medium (native consumers only) | `compoundCut`, `fillet`, `chamfer` | an engine panic | `compoundCut`: no `catch_unwind`, a panic unwinds out of `executeBatch`. `fillet`/`chamfer`: catch and set `poisoned`, but no batch arm ever **checks** `poisoned`, so a poisoned kernel keeps serving batch items | Both check `poisoned` first and catch the panic. On wasm32 (`panic=abort`) none of this applies. The batch `chamfer` comment claims the direct method's panic guard, but direct `chamfer` has none |
| D6 | Low | `unifyFaces`, `healSolid`, `convertToBspline`, `convertToElementary`, `repairSolid` | any solid | `ok`: the solid handle (`unifyFaces`, `healSolid`) or `{solid, converted}` / `{solid, errorsBefore, errorsAfter, totalRepairs}` | A count: faces removed, repairs made, surfaces converted, errors after. Same name, different meaning; nothing pins the batch shape |
| D7 | Low | `transform`, `transformWire`, `transformFace`, `copyAndTransformSolid` | 16-element matrix with a string in slot 15 | `transformFace`: "matrix element 15 is not a number"; the other three: "matrix[15] is not a number"; length error "matrix must have 16 elements" (direct: "transform matrix must have 16 elements"). V2 carries `details.argument` for the length error but not the element error | — |
| D8 | Low | `circularPattern`, `offsetWire2DWithJoin`, `offsetWireWithJoinType` | two bad arguments at once (bad handle and too-large `count`; bad handle and bad `joinType`) | Reports the count (resp. join-type) error | Reports the handle error first |
| D9 | Low | `makeCompound` | > `MAX_WASM_WORK_ITEMS` handles | Refused by `validate_work_count` | No cap |
| D10 | Note | `sewFaces` | no `tolerance` | Defaults to 1e-6; every other batch tolerance default is 1e-7 (`TOL`). Undocumented; may be deliberate | `tolerance` required |

Not drift: `offsetSolid` and `offsetSolidV2` are the same operation under
two names on both surfaces; `validateSolid`'s batch-only "error count exceeds
u32" is unreachable.

D1 is decided (owner, 2026-10-30): the outward normal is correct, so
`getFaceNormal` returns `effective_plane_normal` on both surfaces. The batch
fix is esaueng/remus#1371, a breaking JS-visible change for reversed planar
faces only; faces that are not reversed, every primitive face included, keep
their normal, so `qualify_ops_tests.rs` (`wall_by_normal`, updated in that PR)
and `project_curve.rs` (`box_top_face`, a primitive face) still pick the same
faces. D1b is fixed the same way: the batch arm now calls the direct body,
so a face that is not planar refuses on both surfaces.

D6 is decided (owner, 2026-10-30): the batch heal family's result shapes are
not a contract. Stage 11 may align them with the direct methods without a
deprecation period; the change still gets a release note.

D10 is decided (owner, 2026-10-30): the 1e-6 `sewFaces` default is not
intended. The batch default becomes 1e-7 (`TOL`), like every other batch
tolerance, in its own stage-11 PR.

## 2. Proposal: a `macro_rules!` op table

### 2.1 Choosing the mechanism

| | `macro_rules!` table | proc-macro crate | `build.rs` from a data file |
| --- | --- | --- | --- |
| New dependency | none | a new workspace crate (`syn`/`quote` are already in the graph via wasm-bindgen and serde, so no new external crate); a row in the layer table and `check-boundaries.sh` | none, but a data format (TOML/JSON) and a generator |
| Compile time | expansion only; negligible for 216 rows split per family | proc-macro crate build, cached after the first build | build script rebuilds on table changes |
| Readability | rows are Rust; bodies are ordinary Rust in the same file | best syntax, e.g. an attribute on the direct method | table is data; bodies live elsewhere, joined by name |
| Diagnostics | poor for grammar mistakes in a row, normal inside bodies | best (spanned errors) | errors point into generated code |
| Debuggability | each row names its body `fn`, so backtraces and breakpoints are ordinary | `cargo expand` needed to see output | generated file via `include!`, invisible to rust-analyzer without extra setup |
| WASM size | same machine code as hand-written arms | same | same |
| Tooling (`check-wasm-o47-coverage.py`) | parses uniform row headers with a regex, no build | parses attributes, no build | reads the data file directly (best) |

**Recommendation: `macro_rules!`.** It needs no new crate, the generated code
is the code we write today, and it keeps the bodies where reviewers already
read them. If generating direct shims (section 2.5) proves valuable for many
families, an attribute proc-macro over the direct method is the natural
follow-up; that is a separate RFC because it adds a workspace crate.

### 2.2 What gets generated

Rows live in `crates/wasm/src/bindings/batch_ops/<family>.rs`, one
`batch_ops! { … }` invocation per family (no deep macro recursion). Each row
generates:

1. A named body `fn` (`fn op_make_box(k: &mut RemusKernel, args: &Value) -> BatchItemResult`)
   that decodes arguments **in row order**, then runs the body.
2. An `OpMeta` entry: `name`, `kind` (`ReadOnly` / `Mutating`), `rollback`
   (`ReadOnly`, `Exact`, `Restored`, or `Restored` + one of the
   checkpoint-sharing wrappers `Sweep` / `Imprint` / `FaceProjection`),
   `gate` (a `cfg` attribute passed through), `family`, `twin`
   (`Shared { direct }` / `Divergent { direct, reasons }` / `BatchOnly`), and
   `run: fn(&mut RemusKernel, &Value) -> BatchItemResult`.
3. For `Shared` rows only, the direct `#[wasm_bindgen]` shim (section 2.5).

From the family tables the crate builds once:

* `fn lookup(op: &str) -> Option<&'static OpMeta>`: a single generated
  `match op { "makeBox" => Some(&FAMILY_PRIMITIVES[0]), … }`. It replaces
  `batch_op_kind`, the `match` in `dispatch_op`, the six sub-dispatcher
  fallthroughs and the name tests in `dispatch_with_rollback`.
* `dispatch_with_rollback` keeps its body and reads `meta.kind` /
  `meta.rollback` instead of matching names; unknown names still never reach
  `topo_mut` (the copy-on-write invariant in the `batch_op_kind` doc).

Both batch contracts are untouched: rows return
`Result<Value, StructuredWasmError>` and `execute_batch_contract` projects to
legacy strings or V2 objects exactly as now.

### 2.3 Row grammar

```rust
batch_ops! {
    family primitives;

    /// Optional doc, copied onto the generated body fn.
    op make_box = "makeBox" {
        kind: Mutating,
        rollback: Restored,
        twin: Divergent { direct: "makeBox", reasons: [D4] },
        // Decoded in this order; the first failing key wins.
        args {
            width:  f64 = "width",
            height: f64 = "height",
            depth:  f64 = "depth",
        }
        body(k) {
            let solid = remus_operations::primitives::make_box(k.topo_mut(), width, height, depth)
                .map_err(StructuredWasmError::from)?;
            Ok(json!(solid_id_to_u32(solid)))
        }
    }
}
```

Argument types select a decoder through a small trait (`BatchArg`), which
monomorphizes to the helper calls arms make today:

| Declared type | Decoder (unchanged helper) | Absent / `null` |
| --- | --- | --- |
| `f64`, `u32`, `bool` | `get_f64`, `get_u32`, `get_bool` | refused |
| `Option<f64>`, `Option<u32>`, `Option<bool>` | `get_optional_f64`, `get_optional_u32`, `get_optional_bool` | `None` |
| `T ?? default` | the `Option<T>` decoder, then `unwrap_or(default)` | default (K2) |
| `Vec<u32>`, `Vec<f64>` | `get_u32_array`, `get_f64_array` | refused |
| `Handles<u32>` | `get_u32_array_optional` | empty |
| `Raw` | `&Value` passthrough, for arms with bespoke parsing (`matrix`, `options`, `specs`) | arm decides |

Arms whose current order interleaves decoding with resolution or validation
(chamfer decodes `solid`, `distance`, resolves `solid`, then decodes `edges`)
use a `steps { … }` block instead of `args { … }`: ordinary `let` statements
evaluated in order, so the first error stays the first error. The table
does not force a canonical order during the migration; that would change
which error a malformed item reports (D8).

### 2.4 Expressing the deliberate differences

| Difference | Row construct |
| --- | --- |
| K1 edge handle vs NURBS arrays | `Divergent`; batch decodes `pathEdge` then `extract_nurbs_curve`, sharing the engine call with the direct body |
| K2 batch-only defaults | `name: T ?? default` in `args`; a shared shim takes the direct parameter as `T` (required) |
| K3 legacy vs V2 | not per row: `execute_batch_contract`, unchanged |
| K4 blend error mapping | `errors { batch: blend_failure, direct: kernel_code_prefix }` |
| K5, K11 result shapes | `encode { batch: …, direct: … }` on a `Shared` row, or `Divergent` |
| K6 typed twins | `encode: envelope(op = "moveFaces")`: decode failures become `SolidOperationDetailedResult::error(e.with_direct_operation(op))` under `ok` |
| K7 inline constraint | `Divergent` |
| K8 rollback | `rollback:` per row; the direct shim takes its own `direct_rollback:` (default: none) |
| panic guard + poison | `guard: panic("Fillet")` generates the `catch_unwind` / `poisoned = true` / `panic_message` block the arms carry today, byte for byte |
| `io` feature | `#[cfg(feature = "io")]` on the row, passed through to the body, `OpMeta` and `lookup` arm |

### 2.5 Direct shims

A `Shared` row lists its arguments with types, so it can also generate the
direct method:

```rust
#[wasm_bindgen(js_name = "fuseWithOptions")]
pub fn fuse_with_options(&mut self, solid_a: u32, solid_b: u32, unify_faces: Option<bool>)
    -> Result<u32, JsError>
```

The shim calls the row's body with typed values (no JSON). The batch side
decodes JSON into the same typed values, then calls the same body. Shims are
generated **only** where both surfaces already agree on validation, defaults
(other than K2), rollback and errors; every other direct method stays
hand-written and its row says `Divergent` with the drift ids. A
`Divergent` row becomes `Shared` only through a stage-11 PR.

Direct error text must not change. `JsError` from a `StructuredWasmError`
uses its `message`, which `From<WasmError>` and the engine-error `From`
impls set to the source error's `Display`. The oracle (section 3) proves
this per row instead of assuming it.

### 2.6 Four representative rows

**Primitive (`makeBox`)**: shown in 2.3. `Divergent` because of D4: the
direct method validates `dx/dy/dz > 0` before the engine. After the D4 fix
it becomes:

```rust
op make_box = "makeBox" {
    kind: Mutating, rollback: Restored,
    twin: Shared { direct: "makeBox" },
    args { dx: f64 = "width", dy: f64 = "height", dz: f64 = "depth" }
    body(k) {
        validate_positive(dx, "dx")?;
        validate_positive(dy, "dy")?;
        validate_positive(dz, "dz")?;
        let solid = remus_operations::primitives::make_box(k.topo_mut(), dx, dy, dz)?;
        Ok(handle(solid))
    }
    encode { batch: json_handle, direct: u32 }
}
```

**Boolean with options**: one row for three names, with the op as a row
parameter:

```rust
op boolean_with_options = ["fuseWithOptions" => BooleanOp::Fuse,
                           "cutWithOptions" => BooleanOp::Cut,
                           "intersectWithOptions" => BooleanOp::Intersect] (bool_op) {
    kind: Mutating,
    rollback: Restored,            // batch scope, K8
    direct_rollback: None,         // the engine rolls back natively
    twin: Shared { direct: ["fuseWithOptions", "cutWithOptions", "intersectWithOptions"] },
    args {
        solid_a: u32 = "solidA",
        solid_b: u32 = "solidB",
        unify_faces: Option<bool> = "unifyFaces",
    }
    body(k) {
        let a = k.resolve_solid(solid_a)?;
        let b = k.resolve_solid(solid_b)?;
        let opts = BooleanOptions { unify_faces: unify_faces.unwrap_or(true), ..Default::default() };
        let solid = boolean_with_options(k.topo_mut(), bool_op, a, b, opts)?;
        Ok(handle(solid))
    }
    encode { batch: json_handle, direct: u32 }
}
```

Today's arm decodes all three keys and then resolves; the direct method
resolves and calls. The row keeps both orders, and the oracle confirms it
(both handles bad plus a malformed `unifyFaces` reports `unifyFaces`, as
now).

**Optional arguments with batch defaults (`extrude`)**:

```rust
op extrude = "extrude" {
    kind: Mutating,
    rollback: Restored,
    direct_rollback: None,         // `extrude` owns its native transaction
    twin: Shared { direct: "extrude" },
    args {
        face: u32 = "face",
        dir_x: f64 ?? 0.0 = "dx",
        dir_y: f64 ?? 0.0 = "dy",
        dir_z: f64 ?? 1.0 = "dz",
        distance: f64 ?? 1.0 = "distance",
    }
    body(k) {
        // No-ops on batch input (K12); the direct method's existing checks.
        validate_finite(dir_x, "dir_x")?;
        validate_finite(dir_y, "dir_y")?;
        validate_finite(dir_z, "dir_z")?;
        validate_finite(distance, "distance")?;
        let face = k.resolve_face(face)?;
        let solid = extrude(k.topo_mut(), face, Vec3::new(dir_x, dir_y, dir_z), distance)?;
        Ok(handle(solid))
    }
    encode { batch: json_handle, direct: u32 }
}
```

The generated direct signature is
`extrude(face: u32, dir_x: f64, dir_y: f64, dir_z: f64, distance: f64)`,
with no defaults, as today; the `?? d` defaults apply to batch decoding only.

**Typed V2 refusal (`fillet`)**: `Divergent` (D4, D5) during the migration,
with the error mapping and panic guard the arm has now:

```rust
op fillet = "fillet" {
    kind: Mutating,
    rollback: Restored,
    twin: Divergent { direct: "fillet", reasons: [D4, D5] },
    guard: panic("Fillet"),        // catch_unwind, poisoned = true, operation_failed(panic_message)
    errors: blend_failure,         // V2: category from the kernel code, details.kernelCode (K4)
    steps {
        let solid = get_u32(args, "solid")?;
        let radius = get_f64(args, "radius")?;
        let solid_id = k.resolve_solid(solid)?;
        let edges = get_u32_array_optional(args, "edges")?;
        let edge_ids = k.resolve_edges(&edges)?;
    }
    body(k) {
        let solid = crate::helpers::fillet_whole_selection(k.topo_mut(), solid_id, &edge_ids, radius)?;
        Ok(handle(solid))
    }
}
```

V2 output stays, for example,
`{"code":"invalid_argument","category":"invalid_input","message":"invalid input: radius must be finite and positive, got -1","details":{"kernelCode":"invalid-input","operation":"fillet","operationIndex":0}}`.

## 3. Verification

### 3.1 Batch oracle

New `crates/wasm/src/bindings/batch_ops/oracle_tests.rs`, modelled on
`batch_rollback_oracle_tests.rs`:

* `legacy_dispatch_op` and `legacy_batch_op_kind`: today's `dispatch_op`,
  sub-dispatchers and `batch_op_kind`, **kept verbatim** (copied at the
  stage-0 commit, private names only renamed). Arms already migrated keep
  their legacy copy here until the last stage.
* Two kernels with one history (the `fixture` / `observe` / `live_state` /
  `high_water` / `session_state` helpers from `transaction_tests.rs`), with
  and without a retained checkpoint, run every corpus item through
  `execute_batch_results` on one and the legacy path on the other, in
  lockstep.
* Compared per item: the legacy JSON string and the V2 JSON string,
  **byte for byte** (`execute_batch_contract` output, which includes key
  order and float formatting), plus `live_state`, `high_water`,
  `session_state` and the journal after the item.
* Corpus, generated from the table so it cannot miss a row:
  * one success fixture per op (or an explicit `no_success_fixture` list
    with a reason, e.g. io-gated ops in a non-io build);
  * each declared key removed, set to `null`, set to each wrong JSON type,
    set to `-1`, `0`, `1e-9`, `4294967296` and a float where an integer is
    expected;
  * each handle argument stale, out of range, and the wrong entity class;
  * pairs of simultaneous faults, to pin first-error order (D8);
  * the mutation fuzzer of `crates/io/src/json/mutation.rs` over the
    success fixtures, seeded and bounded.
* Coverage assertions: every `OpMeta` name appears; every op has at least
  one success and one refusal, or a listed exemption; `lookup` and
  `legacy_batch_op_kind` agree on every name in the table and on a set of
  unknown names (including names that differ by case or a trailing space).

### 3.2 Direct shims

Native tests cannot build a `JsError`, so a shim's error path is not
reachable under `cargo test`. Two layers:

* Natively, each generated shim is a one-line wrapper over the row body;
  the oracle drives that body with the direct argument values and compares
  successes and error **messages** against the current direct method's
  logic, refactored into a test-only copy that returns the message.
* In node, a differential script (new, in the style of
  `scripts/test-wasm-smoke.mjs`) loads the kernel package built from the
  stage's merge base and the one built from the stage head, calls every
  `Shared` direct method on the corpus, and compares return values and
  thrown messages exactly. Run in CI on stages that add shims.

### 3.3 O4.7 coverage script

`scripts/check-wasm-o47-coverage.py` discovers batch ops by regex over the
top-level arms of `dispatch_op`. It therefore cannot see the 64
sub-dispatched ops today. Changes:

* `batch_ops()` reads the table row headers (`op <ident> = "<name>"`, and
  the bracketed multi-name form) under `bindings/batch_ops/`, plus any
  remaining hand-written `dispatch_op` arms during migration. The union must
  have no duplicates (a new check).
* A new check: every row's `twin` names existing exports, and every
  `Divergent` row lists drift ids that exist in a new
  `scripts/wasm-batch-drift.json` (id, ops, description, witness test). A
  closed drift id with a remaining reference fails as `STALE`.
* `scripts/test-wasm-o47-coverage.py` gets fixtures for the table form,
  multi-name rows, `cfg` gates on rows, and the mixed (partly migrated)
  state.
* `docs/kernel-maturity/o47-coverage.md` documents the table as the source
  of batch dispatch paths.

The script stays build-free and network-free.

## 4. Size and speed

### 4.1 Size (estimate, unmeasured)

Starting points from the W3 work: `dispatch_op` is about 144 KB of the
shipped kernel; deleting every `executeBatch` arm saves 157 KB raw / 29 KB
gzip; W3.1 projected about 34 KB from sharing implementations.

| Step | Raw | Gzip | Reasoning |
| --- | ---: | ---: | --- |
| Byte-identical migration (stages 0–10) | about −5 to −15 KB | about −1 to −4 KB | Bodies compile to the same code. One `lookup` replaces 3–9 string-match chains per name (each comparison is a length check plus `memcmp`, roughly 20–30 bytes, × 216 names × the extra chains); the `OpMeta` table adds about 216 × 32 B ≈ 7 KB of data |
| After the drift fixes turn rows `Shared` (stage 11) | a further −15 to −30 KB | −4 to −8 KB | Duplicated validation and resolution removed from about 114 arms, consistent with W3.1's projection |

No new code lands in the kernel for any capability, so no `WASM-Size:` line
is expected. Each stage PR reads the CI "WASM Binary Size" comment; growth
above 8 KB raw in a byte-identical stage is a defect to explain before
merging (for example, function pointers stopping inlining that today's
arms get).

### 4.2 Speed

Per batch item, today's path is: `serde_json` parse of the whole array, then
per item `batch_op_kind` (one string-match chain), `dispatch_with_rollback`
(up to five more name tests), `dispatch_op` (up to 126 arm comparisons) and,
for 64 ops, up to six sub-dispatcher chains; then result serialization.

Measured natively on the baseline (release, best of 7 runs of 10,000 items
in one `executeBatch` call; throwaway timing test, not in this PR):

| Item (each a refusal unless noted) | Where the name matches | ns / item |
| --- | --- | ---: |
| `storeLocalCutsEnabled` (success) | early top-level arm | 653 |
| unknown op | `batch_op_kind` only | 1,231 |
| `makeBox` with no args | early arm, mutating scope | 1,311 |
| `getNurbsSurfaceDataParity`, bad handle | last top-level arm | 1,773 |
| `getFaceName`, bad handle | 1st sub-dispatcher | 1,793 |
| `gcsDof`, bad sketch | 3rd sub-dispatcher | 1,954 |
| `fitCubicCurve`, no args | 6th sub-dispatcher | 2,195 |
| `boundingBox` (success) | early arm | 2,513 |

The build was the parent commit `cc7b260`, whose `batch.rs` differs from the
baseline only by the class rename. The rows are confounded: each item builds
a different error and output, and the per-item cost includes JSON parsing,
the error's `details` map and result serialization. They do bound name
matching: an item that falls through every top-level arm and all six
sub-dispatchers costs at most a few hundred nanoseconds more than an early
arm. Real operations cost microseconds to seconds.

The generated path does one `lookup` and one indirect call, so dispatch
cost can only shrink; the parse/serialize cost is unchanged. The repo rule
(no speed regression) is checked per stage with the same harness on
native and with a node run of the built package on a mixed real batch (the
`gridfinity_tests.rs` recipes), comparing instruction counts as W3.2 did.

## 5. Migration plan

Each stage is one PR, byte-identical in JS-visible behaviour, proven by the
oracle over its families plus the whole existing `cargo test -p remus-wasm`
suite, the O4.7 gate, and `node scripts/test-wasm-smoke.mjs` on a fresh
`cargo xtask wasm-build`.

| Stage | Content | Ops | Extra proof |
| --- | --- | ---: | --- |
| 0 | `batch_ops!` macro, `OpMeta`, `lookup`; `batch_op_kind` and rollback selection derived from the table while every row is still a hand-written legacy arm (`Legacy` rows that point at the existing arm); verbatim legacy copy and oracle harness; O4.7 script reads both | 0 moved | `lookup` vs `legacy_batch_op_kind` on all 216 names and unknown names; rollback selection vs the verbatim name tests |
| 1 | Read-only measurement and query: `volume`, `surfaceArea`, `boundingBox`, `centerOfMass`, `massProperties`, `meshQuality`, `sheet*`, `tessellateSheet`, `validate*`, `solidEdges`, `getSolidFaces`, `getFace*`, `getEdgeParamSpan`, distances, draft, convexity, `classifyPoint` | about 35 | read-only rollback: no copy, no generation bump (existing `failing_read_only_op_neither_copies_…`) |
| 2 | Primitives and construction: `make*`, `makeLineEdge`, `makeNurbsEdge`, `makeWire`, faces, sheets, holes, `makeTangentArc3d`, `liftCurve2dToPlane` | about 18 | |
| 3 | Transforms, copies, mirror, patterns | about 14 | matrix messages (D7) pinned as they are |
| 4 | Booleans and sheet trims, including `Exact` rollback rows | about 22 | `transaction_tests.rs` coordinated-boolean cases |
| 5 | Sweep, pipe, loft, extrude, revolve, helix, Minkowski (sweep wrapper) | about 15 | `sweep_transaction_tests.rs` |
| 6 | Blends: fillet, chamfer, face-face (panic guard, `blend_failure`) | about 9 | `fillet_fail_closed_tests.rs` |
| 7 | Heal, offset, shell, draft, section, split, defeature, projection, 2D polygon | about 30 | |
| 8 | Typed twins (`*Detailed`): metadata only, bodies already shared | 13 | `qualify_ops_tests.rs` envelope parity |
| 9 | Sub-dispatchers: naming, evolution, GCS, linked sketch, curve projection, NURBS reuse; delete the six `dispatch_*_op` fallthroughs | 64 | B75 GCS parity tests; projection and imprint wrapper tests |
| 10 | `io`-gated ops; delete `dispatch_op` and `batch_op_kind`; the legacy copy stays in the oracle | 4 | `--features io` and default builds both |
| 11 | Drift fixes, **one PR per drift id**, each a deliberate JS-visible change: an oracle exemption naming the op and the corpus cases it changes, a regression test, owner sign-off, release note. D1 is decided (outward normal) and fixed by esaueng/remus#1371 | — | after the release that ships it, the legacy copy is pruned from the oracle row by row |

Direct shims are introduced only in stage 11, when a fix makes a row
`Shared`, or for rows that are already `Shared`, as a separate follow-up
after stage 10, so a byte-identical stage never changes both surfaces at
once.

## 6. Risks and open questions

* **Order fidelity.** The first error a malformed item reports depends on
  the arm's statement order. `steps { … }` keeps it, and the corpus pins it
  with paired faults; the cost is that some rows stay less declarative until
  stage 11.
* **Key order and float text.** Batch results serialize through
  `serde_json::Value` (sorted object keys, `float_roundtrip`). Rows must keep
  building `Value`s, not typed structs serialized another way, until a
  deliberate change says otherwise.
* **`macro_rules!` diagnostics.** A malformed row produces a poor error.
  Mitigation: the grammar stays small, and the macro module carries one
  compiled example row per construct as a test.
* **Owner decisions:** all taken (2026-10-30). D1: `getFaceNormal`
  returns the outward normal (esaueng/remus#1371). D6: the batch heal
  family's result shapes are not a contract. D10: the batch `sewFaces`
  default becomes 1e-7.
