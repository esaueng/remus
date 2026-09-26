//! P-Class 8.1, first bounded slice: operation-sequence generation and shrinking.
//!
//! Roadmap row 8.1 asks for a differential harness over *randomized operation
//! sequences* with automatic shrinking/replay. This file is the first bounded
//! slice of that parent, built directly on the B26 oracles:
//!
//! - **Grammar (first slice only):** `makeBox` / `makeCylinder` creation,
//!   rigid `transform`, and exact-only `booleanWithQuality` (`fuse` / `cut` /
//!   `intersect` with `exactOnly: true`). Nothing else. Broader families
//!   (cones, spheres, tori, blends, offsets, sweeps) stay with their owning
//!   rows; the nightly schedule and the first-ten-defect exit stay open.
//! - **Sequences, not pairs:** short sequences (3–6 ops) whose later
//!   operations consume earlier results, so defects that need two composed
//!   booleans can surface. Boolean operands are *consumed*: once used, an
//!   operand handle is dead and later ops must reference live results.
//! - **One format:** a sequence IS a schema-1 reproduction bundle operations
//!   array ([`BrepKernel::execute_batch_v2`][remus_wasm] compatible, same
//!   shape as `crates/wasm/tests/repro/*.json`). Generation, execution,
//!   shrinking, and export all operate on that array — no second,
//!   incompatible reproduction format is introduced. The pre-execution copy
//!   is persisted before the kernel runs, so a crash or timeout cannot lose
//!   the inputs that caused it.
//! - **Outcome taxonomy:** every boolean attempt is exactly one of
//!   `exact_ok` / `supported_empty` / `refused` / `incorrect_success` /
//!   `crash` / `timeout` / `invalid_handle`, reported separately. Each
//!   successful result runs its full oracle battery *before* any sibling
//!   refusal is classified, so a permitted refusal never masks an incorrect
//!   success (the B26 re-review lesson). Execution stops at the first
//!   operation that yields no solid (refusal, supported-empty refusal,
//!   invalid handle, failed construction): a dependent chain cannot proceed
//!   past a missing link, so the verdict always comes from the executed
//!   prefix — the same rule committed schema-1 bundles follow (success
//!   chains with terminal expectations). Refused booleans burn no arena
//!   slots (measured during development: a refused exact boolean leaves
//!   `num_solids` unchanged and the next solid takes the predicted slot),
//!   which is why positional handles stay meaningful only on the executed
//!   prefix.
//! - **Oracles (independent of the paths under test):** hand closed forms
//!   for the two primitives, disjoint-operand exact algebra, volume bounds,
//!   complementary boolean identities recomputed on scratch clones
//!   (inclusion–exclusion and cut complement), material probes via the
//!   analytic ray-cast classifier, both validators' topology gates, mesh
//!   watertightness, translation invariance, and transactional rollback
//!   checks on every refusal.
//! - **Shrinking:** greedy deletion with dependent-cascade removal and
//!   handle repair, then parameter simplification toward lattice origins. A
//!   candidate is accepted only when it reproduces the *same* failure key
//!   (outcome kind + oracle tag): an `invalid_handle`, an unrelated refusal,
//!   or a timeout never replaces an `incorrect_success` witness.
//! - **CI mode:** `ci_regression_matrix` replays 8 fixed sequences
//!   deterministically (no environment input). The larger `bounded_campaign`
//!   is bounded by `OPSEQ81_CASES` (default 64) and a per-sequence wall-clock
//!   budget. Neither claims the nightly schedule or the parent exit.
//!
//! ## Known limitations of this slice (not claimed)
//!
//! - Position-quantized free-edge supplement (B26/5) is not yet ported; the
//!   edge-id census plus mesh watertightness stand in.
//! - Material probes cover fuse centers always and disjoint cuts; overlapping
//!   cut/intersect material expectations are skipped (no oracle volunteers
//!   exact overlap volumes for composed results).
//! - Timeout shrinking is attempt-bounded (see [`shrink_sequence`]).
//! - Complementary-identity oracles need all three legs; a refused leg skips
//!   the identities for that boolean (the independent oracles still judge it).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
#![allow(clippy::too_many_lines, clippy::cast_possible_truncation)]
// Campaign diagnostics print case/tallies to stdout by design (seed replay);
// the workspace denies `print_stdout`, so allow it file-wide here.
#![allow(clippy::print_stdout)]
#![allow(
    clippy::type_complexity,
    clippy::collapsible_if,
    clippy::single_element_loop,
    clippy::too_many_arguments,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::items_after_statements
)]

use std::collections::BTreeMap;
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, FRAC_PI_6, PI};
use std::time::Instant;

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::copy::copy_solid;
use remus_operations::measure::{solid_bounding_box, solid_volume};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::explorer;
use remus_topology::solid::SolidId;
use serde_json::{Value, json};

// ── Slice constants ──────────────────────────────────────────────────

/// Repro-bundle schema version sequences are generated and exported as.
/// Must match `remus-wasm`'s `repro::SCHEMA_VERSION`; a mismatch means the
/// exported bundles no longer replay through `executeBatchV2`.
const SCHEMA_VERSION: u32 = 1;
/// Source identity stamped into every persisted bundle description.
const HARNESS_SOURCE: &str = "op_seq_81/v1";
/// Longest sequence this slice generates (primitives + transforms + bools).
const MAX_SEQ_OPS: usize = 6;
/// Default per-sequence wall-clock budget in milliseconds.
///
/// Calibrated 2026-09-26: an initial 30s budget timed out two exploratory
/// sequences whose kernel mains measured 32–1574ms — the overrun was
/// harness-side tessellation at fine deflection in debug builds (the mesh
/// and translation oracles re-mesh big curved results), not a kernel hang.
/// 120s keeps genuine hangs (no output, no refusal) detectable while the
/// debug-build oracle battery fits. Release builds run the same battery in
/// a fraction of the time.
const DEFAULT_SEQ_TIMEOUT_MS: u64 = 120_000;
/// Face-count budget: results above this skip the mesh oracle (a timeout
/// report teaches nothing) but every other oracle still judges them.
const FACE_BUDGET: usize = 240;
/// Relative slack for volume comparisons (gross-disagreement detector).
const VOL_SLACK: f64 = 1e-2;
/// Absolute floor so near-zero volumes do not divide the relative test.
const VOL_FLOOR: f64 = 1e-6;
/// Volume reading deflection (matches the B26 precedent).
const READ_DEFLECTION: f64 = 0.1;
/// Translation vector for the doubled-boundary probe.
const PROBE_DX: f64 = 13.0;
const PROBE_DY: f64 = -7.0;
const PROBE_DZ: f64 = 5.0;
/// Minimum exact successes for a non-vacuous campaign matrix.
const MIN_EXACT_OK: usize = 1;

// ── Deterministic stream (SplitMix64; no external RNG dependency) ────

struct Stream(u64);

impl Stream {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

// ── Quantized magnitude lattices (shared rationale with fuzz shapegen) ─

/// Positive dimension on a half-unit lattice in `[1.0, 8.0]`.
fn dim_lattice(b: u64) -> f64 {
    1.0 + (b % 15) as f64 * 0.5
}

/// Signed placement offset on a half-unit lattice in `[-4.0, 4.0]`.
fn off_lattice(b: u64) -> f64 {
    ((b % 17) as f64 - 8.0) * 0.5
}

/// Rotation angle: mostly quarter turns (coplanar tangency configurations),
/// occasionally oblique.
fn angle_lattice(b: u64) -> f64 {
    match b % 8 {
        0 | 5 => 0.0,
        1 | 4 => FRAC_PI_2,
        2 => PI,
        3 => 3.0 * FRAC_PI_2,
        6 => FRAC_PI_4,
        _ => FRAC_PI_6,
    }
}

// ── Operation builders (schema-1 batch operations) ───────────────────

fn op_make_box(dx: f64, dy: f64, dz: f64) -> Value {
    json!({"op": "makeBox", "args": {"width": dx, "height": dy, "depth": dz}})
}

fn op_make_cylinder(r: f64, h: f64) -> Value {
    json!({"op": "makeCylinder", "args": {"radius": r, "height": h}})
}

fn op_transform(handle: u32, mat: &Mat4) -> Value {
    let flat: Vec<f64> = mat.0.iter().flat_map(|row| row.iter().copied()).collect();
    json!({"op": "transform", "args": {"solid": handle, "matrix": flat}})
}

fn op_bool(kind: &str, a: u32, b: u32) -> Value {
    json!({"op": "booleanWithQuality",
           "args": {"operation": kind, "solidA": a, "solidB": b, "exactOnly": true}})
}

fn op_name(op: &Value) -> &str {
    op.get("op").and_then(Value::as_str).unwrap_or("<missing>")
}

/// `'static` projection of an operation name for verdict notes.
fn static_op_name(op: &Value) -> &'static str {
    match op_name(op) {
        "makeBox" => "makeBox",
        "makeCylinder" => "makeCylinder",
        "transform" => "transform",
        "booleanWithQuality" => "bool",
        _ => "unknown",
    }
}

fn get_f64(args: &Value, key: &str) -> Option<f64> {
    args.get(key)?.as_f64()
}

fn get_u32(args: &Value, key: &str) -> Option<u32> {
    args.get(key)?.as_u64()?.try_into().ok()
}

// ── Sequence generation ──────────────────────────────────────────────
//
// Handles are predicted densely (`next` counter): each `makeBox`,
// `makeCylinder`, and boolean consumes one fresh solid index. Transform
// mutates in place and consumes nothing. Boolean operands are consumed:
// they leave the live set and later operations must reference live results,
// which is what makes the dependencies between operations explicit.

/// Generate one short dependent sequence as a schema-1 operations array.
///
/// Pure function of `(seed, len)`; `len` is clamped to `[3, MAX_SEQ_OPS]`.
/// The first two operations are always primitives (a boolean needs two
/// operands); every later boolean uses the most recent live result as one
/// operand, so each sequence is a genuine chain rather than disjoint pairs.
///
/// Excluded cells (retained as ignored ready-repros, not fixed here):
/// - X1: a boolean whose operands are both same-axis unmoved cylinders
///   (or unmoved boolean results, which may still be coaxial) — the
///   coaxial lattice cell carries an order-dependent face-orientation
///   defect (`coax_*` repros; B33 family affinity).
/// - X2: an unmoved box–cylinder CUT pair in either prim order
///   (`finding_box_cyl_cut_orientation`).
/// - X3: a CUT whose tool is a rotated prim cylinder
///   (`finding_rotated_cyl_cut_material`).
/// - X4: an INTERSECT consuming a rotated prim cylinder
///   (`finding_rotated_cyl_intersect_volumes`).
///
/// Translated cylinders, every box-involving fuse/intersect, and composed
/// results in the X2–X4 cells stay in generation as tripwires for wider
/// instances of each class.
#[derive(Clone, Copy, PartialEq)]
enum GenKind {
    Box,
    Cyl,
    Composed,
}

struct GenLive {
    handle: u32,
    kind: GenKind,
    /// Whether any rigid placement has been applied since creation.
    /// Unplaced cylinders all share the +Z axis at the origin, so two
    /// unplaced non-box operands are coaxial by construction.
    placed: bool,
    /// Whether a non-zero rotation was ever applied (tracked through
    /// composition by OR-ing the operands). Pure translations preserve
    /// coaxiality; rotations break it — and rotated cylinders are exactly
    /// what findings F3/F4 implicate.
    rotated: bool,
}

/// Coaxial-lattice risk: anything that is not a proven box and has never
/// been moved may still sit on the primitive +Z axis.
fn risky(live: &GenLive) -> bool {
    live.kind != GenKind::Box && !live.placed
}

/// Narrow excluded cells X2–X4 (prim-level only; composed results stay in
/// generation as tripwires). Returns the cell id when the boolean must not
/// generate.
fn excluded_cell(kind: BooleanOp, a: &GenLive, b: &GenLive) -> Option<&'static str> {
    // X2: unmoved box–cylinder CUT pair, either prim order.
    if kind == BooleanOp::Cut {
        let box_cut = |x: &GenLive, y: &GenLive| {
            x.kind == GenKind::Box && !x.placed && y.kind == GenKind::Cyl && !y.placed
        };
        if box_cut(a, b) || box_cut(b, a) {
            return Some("X2");
        }
        // X3: CUT whose tool is a rotated prim cylinder.
        if b.kind == GenKind::Cyl && b.rotated {
            return Some("X3");
        }
    }
    // X4: INTERSECT consuming a rotated prim cylinder on either side.
    if kind == BooleanOp::Intersect
        && ((a.kind == GenKind::Cyl && a.rotated) || (b.kind == GenKind::Cyl && b.rotated))
    {
        return Some("X4");
    }
    None
}

/// Push a random rigid placement of a random live solid, recording placement
/// and rotation on its generation record.
fn push_placed_transform(rng: &mut Stream, ops: &mut Vec<Value>, live: &mut [GenLive]) {
    let pick = (rng.below(live.len() as u64)) as usize;
    let target = live[pick].handle;
    live[pick].placed = true;
    let (tx, ty, tz) = (
        off_lattice(rng.next()),
        off_lattice(rng.next()),
        off_lattice(rng.next()),
    );
    let axis = rng.below(3);
    let angle_draw = rng.next();
    let angle = angle_lattice(angle_draw);
    if !matches!(angle_draw % 8, 0 | 5) {
        live[pick].rotated = true;
    }
    let rot = match axis {
        0 => Mat4::rotation_x(angle),
        1 => Mat4::rotation_y(angle),
        _ => Mat4::rotation_z(angle),
    };
    ops.push(op_transform(target, &(Mat4::translation(tx, ty, tz) * rot)));
}

fn generate_sequence(seed: u64, len: usize) -> Vec<Value> {
    let len = len.clamp(3, MAX_SEQ_OPS);
    let mut rng = Stream(seed);
    let mut ops: Vec<Value> = Vec::with_capacity(len);
    let mut live: Vec<GenLive> = Vec::new();
    let mut next: u32 = 0;

    let push_prim =
        |rng: &mut Stream, ops: &mut Vec<Value>, live: &mut Vec<GenLive>, next: &mut u32| {
            if rng.below(2) == 0 {
                ops.push(op_make_box(
                    dim_lattice(rng.next()),
                    dim_lattice(rng.next()),
                    dim_lattice(rng.next()),
                ));
                live.push(GenLive {
                    handle: *next,
                    kind: GenKind::Box,
                    placed: false,
                    rotated: false,
                });
            } else {
                ops.push(op_make_cylinder(
                    dim_lattice(rng.next()),
                    dim_lattice(rng.next()),
                ));
                live.push(GenLive {
                    handle: *next,
                    kind: GenKind::Cyl,
                    placed: false,
                    rotated: false,
                });
            }
            *next += 1;
        };

    push_prim(&mut rng, &mut ops, &mut live, &mut next);
    push_prim(&mut rng, &mut ops, &mut live, &mut next);

    while ops.len() < len {
        let choice = rng.below(10);
        if choice <= 2 && live.len() < 4 {
            push_prim(&mut rng, &mut ops, &mut live, &mut next);
        } else if choice <= 4 && !live.is_empty() {
            push_placed_transform(&mut rng, &mut ops, &mut live);
        } else if live.len() >= 2 {
            // Chain bias: one operand is always the most recent live result,
            // so every boolean extends the sequence rather than forking it.
            let a_pos = live.len() - 1;
            let mut b_pos = (rng.below(live.len() as u64)) as usize;
            if b_pos == a_pos {
                b_pos = live
                    .iter()
                    .position(|l| l.handle != live[a_pos].handle)
                    .expect("distinct live handle");
            }
            let kind = match rng.below(3) {
                0 => BooleanOp::Fuse,
                1 => BooleanOp::Cut,
                _ => BooleanOp::Intersect,
            };
            if (risky(&live[a_pos]) && risky(&live[b_pos]))
                || excluded_cell(kind, &live[a_pos], &live[b_pos]).is_some()
            {
                // Excluded cell (X1–X4): steer into a placement instead,
                // keeping the chain (the placement may unlock a later retry
                // outside the cell).
                push_placed_transform(&mut rng, &mut ops, &mut live);
                continue;
            }
            let (a, b) = (live[a_pos].handle, live[b_pos].handle);
            let rot = live[a_pos].rotated || live[b_pos].rotated;
            ops.push(op_bool(bool_kind_name(kind), a, b));
            live.retain(|l| l.handle != a && l.handle != b);
            live.push(GenLive {
                handle: next,
                kind: GenKind::Composed,
                placed: false,
                rotated: rot,
            });
            next += 1;
        } else {
            push_prim(&mut rng, &mut ops, &mut live, &mut next);
        }
    }
    ops
}

// ── Outcome taxonomy ─────────────────────────────────────────────────

/// Every boolean attempt — and every sequence overall — lands in exactly
/// one bucket. `InvalidHandle` is its own bucket (not a refusal): a
/// dangling reference is a harness/sequence defect, never a kernel verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum SeqKind {
    ExactOk,
    Empty,
    Refused,
    Incorrect,
    Crash,
    Timeout,
    InvalidHandle,
}

impl SeqKind {
    /// Rollup priority: a sequence reports the worst outcome it contains.
    /// `Incorrect` outranks `Refused` (a permitted refusal must never mask
    /// an incorrect success) and `InvalidHandle` outranks `Refused` (a
    /// broken sequence must never read as a clean refusal).
    fn severity(self) -> u8 {
        match self {
            Self::ExactOk => 0,
            Self::Empty => 1,
            Self::Refused => 2,
            Self::InvalidHandle => 3,
            Self::Incorrect => 4,
            Self::Timeout => 5,
            Self::Crash => 6,
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::ExactOk => "exact_ok",
            Self::Empty => "supported_empty",
            Self::Refused => "refused",
            Self::Incorrect => "incorrect_success",
            Self::Crash => "crash",
            Self::Timeout => "timeout",
            Self::InvalidHandle => "invalid_handle",
        }
    }
}

/// One executed operation's verdict.
struct OpNote {
    index: usize,
    op: &'static str,
    kind: SeqKind,
    /// Oracle that decided the verdict (`ok`, `empty`, `refused`, or the
    /// failing oracle tag: `prim_pin`, `topology`, `mesh`, …).
    oracle: &'static str,
    detail: String,
}

/// Whole-sequence verdict plus per-operation notes.
struct SeqReport {
    kind: SeqKind,
    /// Oracle tag of the deciding note (the failure predicate's second half).
    oracle: &'static str,
    notes: Vec<OpNote>,
    bool_ops: usize,
    exact_ok_bools: usize,
}

/// Failure predicate: outcome kind plus the deciding oracle tag. Shrinking
/// preserves exactly this pair — a candidate that answers a different
/// oracle, refuses, dangles, or times out is rejected.
fn failure_key(report: &SeqReport) -> (SeqKind, &'static str) {
    (report.kind, report.oracle)
}

// ── Resource limits, faults, live set ────────────────────────────────

/// Resource limits declared (and persisted) before execution.
#[derive(Debug, Clone)]
struct SeqLimits {
    timeout_ms: u64,
    face_budget: usize,
    source: &'static str,
}

impl Default for SeqLimits {
    fn default() -> Self {
        Self {
            timeout_ms: DEFAULT_SEQ_TIMEOUT_MS,
            face_budget: FACE_BUDGET,
            source: HARNESS_SOURCE,
        }
    }
}

/// Deliberately injected wrong results (tests only). The harness must flag
/// each armed fault as `Incorrect`, and shrinking must retain the predicate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Fault {
    None,
    /// Replace the fuse result at `op_index` with an unchanged copy of its
    /// first operand: the classic silently-dropped-operand wrong success.
    DropOperandFuseAt {
        op_index: usize,
    },
    /// Panic inside the operation to prove the crash bucket works.
    PanicAt {
        op_index: usize,
    },
}

/// Liveness plus the hand-derived closed-form volume where the construction
/// sequence determines one (primitives, rigid placements, disjoint booleans).
type LiveInfo = Option<f64>;

// ── Small measurement helpers (independent readings) ─────────────────

fn rel_err(a: f64, b: f64) -> f64 {
    if !a.is_finite() || !b.is_finite() {
        return f64::INFINITY;
    }
    (a - b).abs() / a.abs().max(b.abs()).max(VOL_FLOOR)
}

enum Measured {
    Value(f64),
    /// Successful measurement returning NaN/Inf: malformed output, a finding.
    NonFinite,
    /// Typed measurement refusal: a pass for the calling oracle.
    Refused,
}

fn measure_vol(topo: &Topology, solid: SolidId) -> Measured {
    match solid_volume(topo, solid, READ_DEFLECTION) {
        Ok(v) if v.is_finite() => Measured::Value(v),
        Ok(v) => {
            let _ = v;
            Measured::NonFinite
        }
        Err(_) => Measured::Refused,
    }
}

fn mesh_deflection(diag: f64) -> f64 {
    if diag.is_finite() && diag > 0.0 {
        (diag * 1e-5).max(1e-7)
    } else {
        READ_DEFLECTION
    }
}

fn result_diag(topo: &Topology, solid: SolidId) -> f64 {
    solid_bounding_box(topo, solid)
        .map(|aabb| (aabb.max - aabb.min).length())
        .unwrap_or(0.0)
}

/// Interior-disjoint bounding boxes imply interior-disjoint solids (the
/// sound direction). Positive epsilon admits exactly-tangent lattice
/// configurations as disjoint, with exact algebraic answers.
fn boxes_interior_disjoint(a: &remus_math::aabb::Aabb3, b: &remus_math::aabb::Aabb3) -> bool {
    const EPS: f64 = 1e-9;
    let sep = |amin: f64, amax: f64, bmin: f64, bmax: f64| amax <= bmin + EPS || bmax <= amin + EPS;
    sep(a.min.x(), a.max.x(), b.min.x(), b.max.x())
        || sep(a.min.y(), a.max.y(), b.min.y(), b.max.y())
        || sep(a.min.z(), a.max.z(), b.min.z(), b.max.z())
}

fn parse_bool_kind(op: &str) -> Option<BooleanOp> {
    match op {
        "fuse" | "union" => Some(BooleanOp::Fuse),
        "cut" | "difference" => Some(BooleanOp::Cut),
        "intersect" | "intersection" => Some(BooleanOp::Intersect),
        _ => None,
    }
}

fn bool_kind_name(op: BooleanOp) -> &'static str {
    match op {
        BooleanOp::Fuse => "fuse",
        BooleanOp::Cut => "cut",
        BooleanOp::Intersect => "intersect",
    }
}

fn exact_boolean(
    topo: &mut Topology,
    op: BooleanOp,
    a: SolidId,
    b: SolidId,
) -> Result<remus_operations::boolean::BooleanOutcome, OperationsError> {
    boolean_with_context(
        topo,
        op,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
}

/// Classify a kernel boolean error: supported-empty, typed refusal, or an
/// untyped error (an incorrect-success witness at the harness level: the
/// engine answered outside its typed contract).
enum RefusalClass {
    Empty,
    Refused,
    Untyped,
}

fn classify_refusal(e: &OperationsError) -> RefusalClass {
    if matches!(e, OperationsError::EmptyResult { .. }) {
        RefusalClass::Empty
    } else if matches!(
        e,
        OperationsError::ExactOnlyUnattainable
            | OperationsError::NonManifoldResult
            | OperationsError::Unsupported { .. }
            | OperationsError::BodyClassOperationUnsupported { .. }
            | OperationsError::BodyClassMeasureMismatch { .. }
            | OperationsError::BodyValidationFailed { .. }
    ) {
        RefusalClass::Refused
    } else {
        RefusalClass::Untyped
    }
}

// ── Execution ──────────────────────────────────────────────────────

struct ExecState<'a> {
    topo: Topology,
    live: BTreeMap<u32, LiveInfo>,
    limits: &'a SeqLimits,
    fault: Fault,
    start: Instant,
    notes: Vec<OpNote>,
    bool_ops: usize,
    exact_ok_bools: usize,
    aborted: bool,
}

impl<'a> ExecState<'a> {
    fn new(limits: &'a SeqLimits, fault: Fault) -> Self {
        Self {
            topo: Topology::new(),
            live: BTreeMap::new(),
            limits,
            fault,
            start: Instant::now(),
            notes: Vec::new(),
            bool_ops: 0,
            exact_ok_bools: 0,
            aborted: false,
        }
    }

    fn elapsed_ms(&self) -> u64 {
        self.start
            .elapsed()
            .as_millis()
            .try_into()
            .unwrap_or(u64::MAX)
    }
}

fn note(
    st: &mut ExecState<'_>,
    index: usize,
    op: &'static str,
    kind: SeqKind,
    oracle: &'static str,
    detail: String,
) {
    st.notes.push(OpNote {
        index,
        op,
        kind,
        oracle,
        detail,
    });
}

/// Transactional rollback check after a refused boolean: live-solid count
/// must not shrink, and both operands must still resolve with unchanged
/// volumes. A torn arena is an `Incorrect` witness, never a pass.
fn check_rollback(
    topo: &Topology,
    solids_before: usize,
    a: SolidId,
    b: SolidId,
    va0: f64,
    vb0: f64,
) -> Result<(), String> {
    if topo.num_solids() < solids_before {
        return Err(format!(
            "live-solid count shrank across a refusal ({} -> {})",
            solids_before,
            topo.num_solids()
        ));
    }
    for (solid, v0, tag) in [(a, va0, "A"), (b, vb0, "B")] {
        let v1 = match measure_vol(topo, solid) {
            Measured::Value(v) => v,
            Measured::NonFinite => {
                return Err(format!("operand {tag} measures non-finite after refusal"));
            }
            Measured::Refused => return Err(format!("operand {tag} unmeasurable after refusal")),
        };
        if rel_err(v0, v1) > VOL_SLACK {
            return Err(format!(
                "operand {tag} volume moved across a refusal ({v0:.9} -> {v1:.9})"
            ));
        }
    }
    Ok(())
}

/// Independent oracle battery for one successful exact boolean result.
/// Returns the failing oracle tag, or `None` when every oracle passes.
/// `vf` is the main result's measured volume; `legs` carries the
/// complementary scratch-clone outcomes for the identity oracles.
#[allow(clippy::too_many_lines)]
fn battery_main(
    st: &ExecState<'_>,
    kind_name: &'static str,
    result: SolidId,
    vf: f64,
    va: f64,
    vb: f64,
    exact_a: Option<f64>,
    exact_b: Option<f64>,
    disjoint: bool,
    probe_a: Option<remus_math::vec::Point3>,
    probe_b: Option<remus_math::vec::Point3>,
) -> Option<(&'static str, String)> {
    // Oracle 1: operand closed forms pinned before any identity runs.
    if let Some(ea) = exact_a
        && rel_err(va, ea) > VOL_SLACK
    {
        return Some((
            "operand_pin",
            format!("operand A {va:.9} != closed form {ea:.9}"),
        ));
    }
    if let Some(eb) = exact_b
        && rel_err(vb, eb) > VOL_SLACK
    {
        return Some((
            "operand_pin",
            format!("operand B {vb:.9} != closed form {eb:.9}"),
        ));
    }
    // Oracle 2: topology — the operations validator plus the edge-use census.
    match remus_operations::validate::validate_solid(&st.topo, result) {
        Ok(report) if report.is_valid() => {}
        Ok(report) => {
            let first = report
                .issues
                .first()
                .map(|i| i.description.clone())
                .unwrap_or_default();
            return Some((
                "topology",
                format!(
                    "validator reports {} error(s); first: {first}",
                    report.error_count()
                ),
            ));
        }
        Err(e) => return Some(("topology", format!("validator lookup failed: {e:?}"))),
    }
    match explorer::edge_to_face_map(&st.topo, result) {
        Ok(map) => {
            let mut free = 0;
            let mut non_manifold = 0;
            for uses in map.values() {
                match uses.len() {
                    0 | 1 => free += 1,
                    2 => {}
                    _ => non_manifold += 1,
                }
            }
            if free != 0 || non_manifold != 0 {
                return Some((
                    "topology_census",
                    format!("{free} free / {non_manifold} non-manifold edge uses"),
                ));
            }
        }
        Err(e) => return Some(("topology", format!("edge census failed: {e:?}"))),
    }
    // Oracle 3: mesh watertightness (skipped past the face budget, never
    // weakened to pass).
    let faces = explorer::solid_faces(&st.topo, result).map(|f| f.len());
    if faces.is_ok_and(|n| n <= st.limits.face_budget) {
        let deflection = mesh_deflection(result_diag(&st.topo, result));
        // A tessellation refusal is a pass for this oracle.
        if let Ok(mesh) = tessellate_solid(&st.topo, result, deflection) {
            let b = boundary_edge_count(&mesh);
            let n = non_manifold_edge_count(&mesh);
            if b != 0 || n != 0 {
                return Some((
                    "mesh",
                    format!("{b} boundary / {n} non-manifold mesh edges"),
                ));
            }
        }
    }
    // Oracle 4: volume bounds, sharpened to exact algebra on disjoint pairs.
    let slack = |v: f64| v.abs().mul_add(VOL_SLACK, VOL_FLOOR);
    match kind_name {
        "cut" => {
            if vf > va + slack(va) {
                return Some((
                    "volume_bounds",
                    format!("cut {vf:.9} exceeds target {va:.9}: material invented"),
                ));
            }
        }
        "fuse" => {
            if vf > va + vb + slack(va + vb) {
                return Some((
                    "volume_bounds",
                    format!("fuse {vf:.9} exceeds operand sum {:.9}", va + vb),
                ));
            }
            if vf < va.max(vb) - slack(va.max(vb)) {
                return Some((
                    "volume_bounds",
                    format!(
                        "fuse {vf:.9} below larger operand {:.9}: material lost",
                        va.max(vb)
                    ),
                ));
            }
        }
        _ => {
            if vf > va.min(vb) + slack(va.min(vb)) {
                return Some((
                    "volume_bounds",
                    format!(
                        "intersect {vf:.9} exceeds smaller operand {:.9}",
                        va.min(vb)
                    ),
                ));
            }
        }
    }
    if disjoint && let (Some(ea), Some(eb)) = (exact_a, exact_b) {
        let expected = match kind_name {
            "fuse" => ea + eb,
            "cut" => ea,
            _ => 0.0,
        };
        if rel_err(vf, expected) > VOL_SLACK {
            return Some((
                "disjoint_exact",
                format!("disjoint {kind_name} {vf:.9} != exact {expected:.9}"),
            ));
        }
    }
    // Oracle 4b: the two volume routes must agree about whatever was built.
    // They share their face integrator, so agreement proves little on its
    // own — but a disagreement means one route alone misreads the geometry
    // (the recorded precedent is a 39% split on a valid-seeming intersect).
    // A refused second route skips (pass); a non-finite one is a finding.
    match remus_operations::measure::mass_properties(&st.topo, result) {
        Ok(props) if props.mass.is_finite() => {
            if rel_err(vf, props.mass) > VOL_SLACK {
                return Some((
                    "mass_agreement",
                    format!("solid_volume {vf:.9} vs mass_properties {:.9}", props.mass),
                ));
            }
        }
        Ok(props) => {
            let _ = props;
            return Some((
                "non_finite",
                "mass_properties returned non-finite".to_owned(),
            ));
        }
        Err(_) => {}
    }
    // Oracle 5: material probes with the analytic ray-cast classifier.
    // Fuse centers are always inside the union; disjoint-cut centers have
    // known sides. Overlapping cut/intersect probes are skipped: no oracle
    // volunteers their exact answers (documented limitation).
    let want: Vec<(Option<remus_math::vec::Point3>, bool, &str)> = match kind_name {
        "fuse" => vec![(probe_a, true, "A-center"), (probe_b, true, "B-center")],
        "cut" if disjoint => vec![(probe_a, true, "A-center"), (probe_b, false, "B-center")],
        _ => vec![],
    };
    if !want.is_empty() {
        let opts = remus_check::classify::ClassifyOptions::default();
        for (point, inside, tag) in want {
            let Some(p) = point else { continue };
            match remus_check::classify::classify_point(&st.topo, result, p, &opts) {
                Ok(remus_check::classify::PointClassification::Inside) if inside => {}
                Ok(remus_check::classify::PointClassification::Outside) if !inside => {}
                Ok(remus_check::classify::PointClassification::OnBoundary) => {}
                Ok(other) => {
                    return Some((
                        "material_probe",
                        format!("{tag} classifies {other:?}, expected inside={inside}"),
                    ));
                }
                Err(_) => {} // Classifier refusal skips the probe, not the verdict.
            }
        }
    }
    // Oracle 6: translation invariance (the doubled-boundary detector).
    {
        let mut moved = st.topo.clone();
        if remus_operations::transform::transform_solid(
            &mut moved,
            result,
            &Mat4::translation(PROBE_DX, PROBE_DY, PROBE_DZ),
        )
        .is_ok()
        {
            let diag = result_diag(&moved, result);
            if let Ok(v1) = solid_volume(&moved, result, mesh_deflection(diag))
                && v1.is_finite()
                && rel_err(vf, v1) > VOL_SLACK
            {
                return Some((
                    "translation",
                    format!("volume moved {vf:.9} -> {v1:.9} under rigid translation"),
                ));
            }
        }
    }
    None
}

/// Scratch-clone complementary legs for the identity oracles. Each leg runs
/// on its own clone so operand handles stay valid across legs. Returns the
/// three leg volumes when all succeed (an `EmptyResult` intersect counts as
/// `Some(0.0)`), or `None` when any leg refuses — with one exception: a leg
/// that fails *untyped* is itself an `Incorrect` witness and is returned as
/// an error instead of a quiet skip.
#[allow(clippy::too_many_lines)]
fn complementary_legs(
    topo: &Topology,
    a: SolidId,
    b: SolidId,
) -> Result<Option<(f64, f64, f64)>, (&'static str, String)> {
    let mut vols: [Option<f64>; 3] = [None, None, None];
    for (slot, leg) in [BooleanOp::Fuse, BooleanOp::Intersect, BooleanOp::Cut]
        .iter()
        .enumerate()
    {
        let mut scratch = topo.clone();
        match exact_boolean(&mut scratch, *leg, a, b) {
            Ok(outcome) => {
                if outcome.quality != BooleanQuality::Exact {
                    return Err((
                        "identity_leg",
                        format!(
                            "{} leg escaped ExactOnly with approximate quality",
                            bool_kind_name(*leg)
                        ),
                    ));
                }
                match remus_operations::validate::validate_solid(&scratch, outcome.solid) {
                    Ok(report) if report.is_valid() => {}
                    Ok(report) => {
                        let first = report
                            .issues
                            .first()
                            .map(|i| i.description.clone())
                            .unwrap_or_default();
                        return Err((
                            "identity_leg",
                            format!(
                                "{} leg topology invalid ({} error(s); first: {first})",
                                bool_kind_name(*leg),
                                report.error_count()
                            ),
                        ));
                    }
                    Err(e) => {
                        return Err((
                            "identity_leg",
                            format!(
                                "{} leg validator lookup failed: {e:?}",
                                bool_kind_name(*leg)
                            ),
                        ));
                    }
                }
                match measure_vol(&scratch, outcome.solid) {
                    Measured::Value(v) => vols[slot] = Some(v),
                    Measured::NonFinite => {
                        return Err((
                            "identity_leg",
                            format!("{} leg measured non-finite", bool_kind_name(*leg)),
                        ));
                    }
                    Measured::Refused => return Ok(None),
                }
            }
            Err(e) => match classify_refusal(&e) {
                // An empty intersect is the disjoint signature: volume zero.
                RefusalClass::Empty if *leg == BooleanOp::Intersect => vols[slot] = Some(0.0),
                RefusalClass::Empty | RefusalClass::Refused => return Ok(None),
                RefusalClass::Untyped => {
                    return Err((
                        "identity_leg",
                        format!("{} leg untyped error: {e:?}", bool_kind_name(*leg)),
                    ));
                }
            },
        }
    }
    match vols {
        [Some(vf), Some(vi), Some(vc)] => Ok(Some((vf, vi, vc))),
        _ => Ok(None),
    }
}

/// Execute one operation inside the sequence. Returns `true` when the loop
/// must stop: the timeout fired, or the chain broke (an operation that
/// should produce a solid yielded none — a refusal, a supported empty
/// refusal, an invalid handle, or a failed construction). A dependent chain
/// cannot proceed past a missing link, so the verdict always comes from the
/// executed prefix; unexecutable tails contribute no verdict. (Committed
/// schema-1 bundles follow the same rule: success chains with terminal
/// expectations, never operations past a refusal.)
#[allow(clippy::too_many_lines)]
fn exec_one(st: &mut ExecState<'_>, index: usize, op: &Value) -> bool {
    let name = op_name(op);
    let args = op.get("args").unwrap_or(&Value::Null);
    // Static op-name table keeps the grammar closed: anything outside the
    // first-slice set is an incorrect sequence, never silently skipped.
    let known = matches!(
        name,
        "makeBox" | "makeCylinder" | "transform" | "booleanWithQuality"
    );
    if !known {
        note(
            st,
            index,
            "unknown",
            SeqKind::Incorrect,
            "unknown_op",
            format!("op {name} is outside the 8.1 first-slice grammar"),
        );
        return false;
    }
    assert!(
        st.fault != (Fault::PanicAt { op_index: index }),
        "injected panic at op {index} (Fault::PanicAt)"
    );

    match name {
        "makeBox" | "makeCylinder" => {
            let (closed, built): (Option<f64>, Result<SolidId, OperationsError>) = if name
                == "makeBox"
            {
                let (Some(dx), Some(dy), Some(dz)) = (
                    get_f64(args, "width"),
                    get_f64(args, "height"),
                    get_f64(args, "depth"),
                ) else {
                    note(
                        st,
                        index,
                        "prim",
                        SeqKind::Incorrect,
                        "prim_args",
                        "non-numeric box dims".to_owned(),
                    );
                    return false;
                };
                if !(dx.is_finite()
                    && dy.is_finite()
                    && dz.is_finite()
                    && dx > 0.0
                    && dy > 0.0
                    && dz > 0.0)
                {
                    note(
                        st,
                        index,
                        "prim",
                        SeqKind::Incorrect,
                        "prim_args",
                        format!("invalid box dims {dx}/{dy}/{dz}"),
                    );
                    return false;
                }
                (
                    Some(dx * dy * dz),
                    remus_operations::primitives::make_box(&mut st.topo, dx, dy, dz),
                )
            } else {
                let (Some(r), Some(h)) = (get_f64(args, "radius"), get_f64(args, "height")) else {
                    note(
                        st,
                        index,
                        "prim",
                        SeqKind::Incorrect,
                        "prim_args",
                        "non-numeric cylinder dims".to_owned(),
                    );
                    return false;
                };
                if !(r.is_finite() && h.is_finite() && r > 0.0 && h > 0.0) {
                    note(
                        st,
                        index,
                        "prim",
                        SeqKind::Incorrect,
                        "prim_args",
                        format!("invalid cylinder dims {r}/{h}"),
                    );
                    return false;
                }
                (
                    Some(PI * r * r * h),
                    remus_operations::primitives::make_cylinder(&mut st.topo, r, h),
                )
            };
            match built {
                Err(e) => {
                    // No solid produced: the chain breaks here.
                    note(
                        st,
                        index,
                        "prim",
                        SeqKind::Incorrect,
                        "prim",
                        format!("primitive constructor failed on valid dims: {e:?}"),
                    );
                    return true;
                }
                Ok(solid) => match measure_vol(&st.topo, solid) {
                    Measured::NonFinite => note(
                        st,
                        index,
                        "prim",
                        SeqKind::Incorrect,
                        "non_finite",
                        "fresh primitive measures non-finite".to_owned(),
                    ),
                    Measured::Refused => note(
                        st,
                        index,
                        "prim",
                        SeqKind::Incorrect,
                        "prim_measure",
                        "fresh primitive unmeasurable".to_owned(),
                    ),
                    Measured::Value(v) => {
                        let expected = closed.expect("closed form set for both prim kinds");
                        if rel_err(v, expected) > VOL_SLACK {
                            note(
                                st,
                                index,
                                "prim",
                                SeqKind::Incorrect,
                                "prim_pin",
                                format!("measured {v:.9} != closed form {expected:.9}"),
                            );
                        } else {
                            let handle = solid.index() as u32;
                            st.live.insert(handle, Some(expected));
                            note(
                                st,
                                index,
                                "prim",
                                SeqKind::ExactOk,
                                "ok",
                                format!("closed form {expected:.6} pinned"),
                            );
                        }
                    }
                },
            }
            false
        }
        "transform" => {
            let (Some(handle), matrix) = (get_u32(args, "solid"), args.get("matrix")) else {
                note(
                    st,
                    index,
                    "transform",
                    SeqKind::Incorrect,
                    "xform_args",
                    "non-numeric transform handle".to_owned(),
                );
                return false;
            };
            let elems: Option<Vec<f64>> = matrix.and_then(|m| {
                m.as_array().and_then(|a| {
                    if a.len() == 16 {
                        a.iter().map(Value::as_f64).collect()
                    } else {
                        None
                    }
                })
            });
            let Some(elems) = elems else {
                note(
                    st,
                    index,
                    "transform",
                    SeqKind::Incorrect,
                    "xform_args",
                    "matrix must hold 16 numbers".to_owned(),
                );
                return false;
            };
            if !elems.iter().all(|v| v.is_finite()) {
                note(
                    st,
                    index,
                    "transform",
                    SeqKind::Incorrect,
                    "xform_args",
                    "non-finite matrix entry".to_owned(),
                );
                return false;
            }
            let rows = std::array::from_fn(|i| std::array::from_fn(|j| elems[i * 4 + j]));
            let mat = Mat4(rows);
            let Some(solid) = st.topo.solid_id_from_index(handle as usize) else {
                note(
                    st,
                    index,
                    "transform",
                    SeqKind::InvalidHandle,
                    "invalid_handle",
                    format!("solid {handle} does not resolve"),
                );
                return true;
            };
            let v0 = match measure_vol(&st.topo, solid) {
                Measured::Value(v) => v,
                Measured::NonFinite => {
                    note(
                        st,
                        index,
                        "transform",
                        SeqKind::Incorrect,
                        "non_finite",
                        "pre-transform volume non-finite".to_owned(),
                    );
                    return false;
                }
                Measured::Refused => {
                    note(
                        st,
                        index,
                        "transform",
                        SeqKind::Incorrect,
                        "xform_measure",
                        "pre-transform volume refused".to_owned(),
                    );
                    return false;
                }
            };
            let faces0 = explorer::solid_entity_counts(&st.topo, solid).map(|(f, _, _)| f);
            if let Err(e) = remus_operations::transform::transform_solid(&mut st.topo, solid, &mat)
            {
                note(
                    st,
                    index,
                    "transform",
                    SeqKind::Incorrect,
                    "xform",
                    format!("rigid transform refused on valid solid: {e:?}"),
                );
            } else {
                let bad = match measure_vol(&st.topo, solid) {
                    Measured::Value(v1) if rel_err(v0, v1) > VOL_SLACK => Some((
                        "xform_volume",
                        format!("volume moved {v0:.9} -> {v1:.9} under rigid motion"),
                    )),
                    Measured::Value(_) => None,
                    _ => Some((
                        "xform_volume",
                        "post-transform volume unmeasurable".to_owned(),
                    )),
                }
                .or_else(|| match explorer::solid_entity_counts(&st.topo, solid) {
                    Ok((f1, _, _)) if faces0.as_ref().is_ok_and(|&f0| f0 == f1) => None,
                    Ok((f1, _, _)) => Some((
                        "xform_census",
                        format!("face count moved under rigid motion: {faces0:?} -> {f1}"),
                    )),
                    Err(e) => Some((
                        "xform_census",
                        format!("post-transform census failed: {e:?}"),
                    )),
                });
                if let Some((oracle, detail)) = bad {
                    note(st, index, "transform", SeqKind::Incorrect, oracle, detail);
                } else {
                    note(
                        st,
                        index,
                        "transform",
                        SeqKind::ExactOk,
                        "ok",
                        "rigid motion preserves volume and census".to_owned(),
                    );
                }
            }
            false
        }
        "booleanWithQuality" => {
            let op_str = args.get("operation").and_then(Value::as_str).unwrap_or("");
            let Some(kind) = parse_bool_kind(op_str) else {
                note(
                    st,
                    index,
                    "bool",
                    SeqKind::Incorrect,
                    "bool_args",
                    format!("unknown boolean operation {op_str}"),
                );
                return true;
            };
            if args.get("exactOnly").and_then(Value::as_bool) != Some(true) {
                note(
                    st,
                    index,
                    "bool",
                    SeqKind::Incorrect,
                    "policy",
                    "first-slice booleans must set exactOnly: true".to_owned(),
                );
                return true;
            }
            let (Some(ha), Some(hb)) = (get_u32(args, "solidA"), get_u32(args, "solidB")) else {
                note(
                    st,
                    index,
                    "bool",
                    SeqKind::Incorrect,
                    "bool_args",
                    "non-numeric boolean handles".to_owned(),
                );
                return true;
            };
            let kind_name = bool_kind_name(kind);
            let (Some(a), Some(b)) = (
                st.topo.solid_id_from_index(ha as usize),
                st.topo.solid_id_from_index(hb as usize),
            ) else {
                note(
                    st,
                    index,
                    "bool",
                    SeqKind::InvalidHandle,
                    "invalid_handle",
                    format!("boolean handles {ha}/{hb} do not both resolve"),
                );
                return true;
            };
            st.bool_ops += 1;
            // Pre-read everything the oracles need: booleans may rebuild
            // operand entities, so operand-relative facts are snapshotted now.
            let (ma, mb) = (measure_vol(&st.topo, a), measure_vol(&st.topo, b));
            let (Measured::Value(va), Measured::Value(vb)) = (ma, mb) else {
                note(
                    st,
                    index,
                    "bool",
                    SeqKind::Incorrect,
                    "operand_measure",
                    "operand volume unmeasurable or non-finite".to_owned(),
                );
                return false;
            };
            let boxes = solid_bounding_box(&st.topo, a)
                .ok()
                .zip(solid_bounding_box(&st.topo, b).ok());
            let disjoint = boxes
                .as_ref()
                .is_some_and(|(x, y)| boxes_interior_disjoint(x, y));
            let probe_a = boxes.as_ref().map(|(x, _)| x.center());
            let probe_b = boxes.as_ref().map(|(_, y)| y.center());
            let exact_a = st.live.get(&ha).copied().flatten();
            let exact_b = st.live.get(&hb).copied().flatten();
            let solids_before = st.topo.num_solids();

            // Main attempt on the live topology.
            enum Main {
                Outcome(remus_operations::boolean::BooleanOutcome),
                FaultCopy(SolidId),
            }
            let main: Result<Main, OperationsError> = if st.fault
                == (Fault::DropOperandFuseAt { op_index: index })
                && kind == BooleanOp::Fuse
            {
                // Injected wrong success: the fuse silently returns its
                // first operand unchanged. The oracle battery below — kept
                // fault-free — must catch it.
                copy_solid(&mut st.topo, a).map(Main::FaultCopy)
            } else {
                exact_boolean(&mut st.topo, kind, a, b).map(Main::Outcome)
            };
            let mut main_vol: Option<f64> = None;
            let mut main_solid: Option<SolidId> = None;
            match main {
                Err(e) => match classify_refusal(&e) {
                    RefusalClass::Empty => {
                        note(
                            st,
                            index,
                            "bool",
                            SeqKind::Empty,
                            "empty",
                            format!("supported empty result: {e}"),
                        );
                    }
                    RefusalClass::Refused => {
                        if let Err(detail) = check_rollback(&st.topo, solids_before, a, b, va, vb) {
                            note(st, index, "bool", SeqKind::Incorrect, "txn", detail);
                        } else {
                            note(
                                st,
                                index,
                                "bool",
                                SeqKind::Refused,
                                "refused",
                                format!("typed refusal, rollback intact: {e}"),
                            );
                        }
                    }
                    RefusalClass::Untyped => note(
                        st,
                        index,
                        "bool",
                        SeqKind::Incorrect,
                        "untyped_err",
                        format!("untyped boolean error: {e:?}"),
                    ),
                },
                Ok(Main::Outcome(outcome)) => {
                    if outcome.quality == BooleanQuality::Exact {
                        main_solid = Some(outcome.solid);
                    } else {
                        note(
                            st,
                            index,
                            "bool",
                            SeqKind::Incorrect,
                            "exact_policy",
                            "ExactOnly returned approximate quality".to_owned(),
                        );
                    }
                }
                Ok(Main::FaultCopy(c)) => {
                    main_solid = Some(c);
                }
            }

            // The main result's independent battery runs before any sibling
            // (complementary-leg) outcome is classified.
            if let Some(result) = main_solid {
                match explorer::solid_faces(&st.topo, result).map(|f| f.len()) {
                    Err(_) => note(
                        st,
                        index,
                        "bool",
                        SeqKind::Incorrect,
                        "topology",
                        "result faces unreadable".to_owned(),
                    ),
                    Ok(0) => match measure_vol(&st.topo, result) {
                        Measured::Value(v) if v.abs() <= VOL_FLOOR.mul_add(10.0, 0.0) => {
                            note(
                                st,
                                index,
                                "bool",
                                SeqKind::Empty,
                                "empty",
                                "faceless result measures ~0".to_owned(),
                            );
                        }
                        Measured::Value(v) => note(
                            st,
                            index,
                            "bool",
                            SeqKind::Incorrect,
                            "empty_volume",
                            format!("faceless result measures {v:.9}, expected ~0"),
                        ),
                        Measured::NonFinite => note(
                            st,
                            index,
                            "bool",
                            SeqKind::Incorrect,
                            "non_finite",
                            "empty result measures non-finite".to_owned(),
                        ),
                        Measured::Refused => note(
                            st,
                            index,
                            "bool",
                            SeqKind::Empty,
                            "empty",
                            "faceless result, measurement declined".to_owned(),
                        ),
                    },
                    Ok(_) => match measure_vol(&st.topo, result) {
                        Measured::NonFinite => note(
                            st,
                            index,
                            "bool",
                            SeqKind::Incorrect,
                            "non_finite",
                            "result measures non-finite".to_owned(),
                        ),
                        Measured::Refused => note(
                            st,
                            index,
                            "bool",
                            SeqKind::Incorrect,
                            "result_measure",
                            "result unmeasurable".to_owned(),
                        ),
                        Measured::Value(vf) => {
                            main_vol = Some(vf);
                            if let Some((oracle, detail)) = battery_main(
                                st, kind_name, result, vf, va, vb, exact_a, exact_b, disjoint,
                                probe_a, probe_b,
                            ) {
                                note(st, index, "bool", SeqKind::Incorrect, oracle, detail);
                            } else {
                                st.exact_ok_bools += 1;
                                note(
                                    st,
                                    index,
                                    "bool",
                                    SeqKind::ExactOk,
                                    "ok",
                                    format!("exact {kind_name} {vf:.6}"),
                                );
                            }
                        }
                    },
                }
            }

            // Complementary identities on scratch clones (fault-free by
            // construction: `st.fault` never fires inside this helper).
            if st.elapsed_ms() >= st.limits.timeout_ms {
                note(
                    st,
                    index,
                    "bool",
                    SeqKind::Timeout,
                    "timeout",
                    "budget exhausted before identity legs".to_owned(),
                );
                st.aborted = true;
                return true;
            }
            // Re-resolve operand handles in a pre-op snapshot: the main
            // attempt may have rebuilt entities, so legs run on clones of
            // the CURRENT topology only when both operands still resolve.
            // (They always do here: operands are never deleted, only
            // retired from the harness live set.)
            match complementary_legs(&st.topo, a, b) {
                Err((oracle, detail)) => {
                    note(st, index, "bool", SeqKind::Incorrect, oracle, detail);
                }
                Ok(None) => {}
                Ok(Some((vf_leg, vi_leg, vc_leg))) => {
                    let sum = va + vb;
                    if rel_err(vf_leg + vi_leg, sum) > VOL_SLACK {
                        note(
                            st,
                            index,
                            "bool",
                            SeqKind::Incorrect,
                            "inclusion_exclusion",
                            format!("fuse {vf_leg:.9} + intersect {vi_leg:.9} != A+B {sum:.9}"),
                        );
                    } else if rel_err(vc_leg + vi_leg, va) > VOL_SLACK {
                        note(
                            st,
                            index,
                            "bool",
                            SeqKind::Incorrect,
                            "cut_complement",
                            format!("cut {vc_leg:.9} + intersect {vi_leg:.9} != A {va:.9}"),
                        );
                    } else if let Some(vf) = main_vol
                        && rel_err(
                            vf,
                            match kind {
                                BooleanOp::Fuse => vf_leg,
                                BooleanOp::Cut => vc_leg,
                                BooleanOp::Intersect => vi_leg,
                            },
                        ) > VOL_SLACK
                    {
                        note(
                            st,
                            index,
                            "bool",
                            SeqKind::Incorrect,
                            "identity_agreement",
                            format!("main {kind_name} {vf:.9} disagrees with its identity leg"),
                        );
                    }
                }
            }

            // Operand consumption bookkeeping: successful main attempts retire
            // their operands (later ops must use live results). Refusals keep
            // every operand live — the transactional contract.
            if main_solid.is_some() {
                let exact = match (disjoint, exact_a, exact_b) {
                    (true, Some(x), Some(y)) => Some(match kind {
                        BooleanOp::Fuse => x + y,
                        BooleanOp::Cut => x,
                        BooleanOp::Intersect => 0.0,
                    }),
                    _ => None,
                };
                st.live.remove(&ha);
                st.live.remove(&hb);
                if let Some(handle) = main_solid.map(|s| s.index() as u32) {
                    st.live.insert(handle, exact);
                }
            }
            // A boolean that yields no solid breaks the chain; one that
            // yields a (possibly wrong) solid continues so every downstream
            // oracle still runs.
            main_solid.is_none()
        }
        _ => false,
    }
}

/// Execute a whole sequence. Never panics across the harness boundary:
/// kernel panics are caught per operation and reported as `Crash`.
fn execute_sequence(ops: &[Value], limits: &SeqLimits, fault: Fault) -> SeqReport {
    let mut st = ExecState::new(limits, fault);
    if ops.is_empty() {
        return SeqReport {
            kind: SeqKind::ExactOk,
            oracle: "empty_sequence",
            notes: Vec::new(),
            bool_ops: 0,
            exact_ok_bools: 0,
        };
    }
    for (index, op) in ops.iter().enumerate() {
        if st.elapsed_ms() >= st.limits.timeout_ms {
            let budget = st.limits.timeout_ms;
            note(
                &mut st,
                index,
                "sequence",
                SeqKind::Timeout,
                "timeout",
                format!("{budget}ms budget exhausted"),
            );
            st.aborted = true;
            break;
        }
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            exec_one(&mut st, index, op)
        }));
        match outcome {
            Err(_) => {
                note(
                    &mut st,
                    index,
                    static_op_name(op),
                    SeqKind::Crash,
                    "panic",
                    "operation panicked (caught at the harness boundary)".to_owned(),
                );
                st.aborted = true;
                break;
            }
            Ok(abort) => {
                if abort || st.aborted {
                    break;
                }
            }
        }
    }
    let kind = st
        .notes
        .iter()
        .map(|n| n.kind)
        .max_by_key(|k| k.severity())
        .unwrap_or(SeqKind::ExactOk);
    let oracle = st
        .notes
        .iter()
        .find(|n| n.kind == kind)
        .map(|n| n.oracle)
        .unwrap_or("ok");
    SeqReport {
        kind,
        oracle,
        notes: st.notes,
        bool_ops: st.bool_ops,
        exact_ok_bools: st.exact_ok_bools,
    }
}

// ── Schema-1 export and pre-execution persistence ────────────────────

/// Wrap an operations array as a schema-1 bundle document replayable via
/// `BrepKernel::execute_batch_v2` (native runner: `execute_sequence`).
fn to_bundle(name: &str, description: String, ops: &[Value]) -> Value {
    let revision = std::env::var("OPSEQ81_REVISION").unwrap_or_else(|_| "unknown".to_owned());
    json!({
        "schema": SCHEMA_VERSION,
        "name": name,
        "description": description,
        "revision": revision,
        "operations": ops,
        "expect": [],
    })
}

fn bundle_description(seed: u64, report: Option<&SeqReport>, limits: &SeqLimits) -> String {
    let verdict = report.map_or_else(
        || "inputs (persisted before execution)".to_owned(),
        |r| format!("{} / {} after execution", r.kind.label(), r.oracle),
    );
    format!(
        "P-Class 8.1 first-slice op sequence: seed={seed} source={} timeout_ms={} face_budget={} verdict={verdict}",
        limits.source, limits.timeout_ms, limits.face_budget,
    )
}

/// Findings directory: `$OPSEQ81_OUT` when set, else the process temp dir.
/// Never inside the repo tree, so campaign artifacts cannot pollute commits.
fn findings_dir() -> std::path::PathBuf {
    std::env::var("OPSEQ81_OUT")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| std::env::temp_dir().join("opseq81-findings"))
}

/// Persist the inputs BEFORE execution: a later crash or timeout still
/// leaves the exact bundle that caused it, with seed, source identity, and
/// resource limits stamped into the description.
fn persist_inputs(name: &str, seed: u64, ops: &[Value], limits: &SeqLimits) -> std::path::PathBuf {
    let dir = findings_dir();
    let _ = std::fs::create_dir_all(&dir);
    let path = dir.join(format!("{name}.json"));
    let bundle = to_bundle(name, bundle_description(seed, None, limits), ops);
    let text = serde_json::to_string_pretty(&bundle).expect("bundle serializes");
    std::fs::write(&path, text).expect("findings dir is writable");
    path
}

// ── Shrinking ────────────────────────────────────────────────────────
//
// Deletion with dependent-cascade removal and dense handle repair, then
// parameter simplification toward lattice origins. A candidate replaces the
// incumbent only when its failure key (outcome kind + oracle tag) is
// identical: an invalid handle, an unrelated refusal, or a timeout never
// replaces an incorrect-success witness.

/// Simulate dense handle assignment: every `makeBox`/`makeCylinder` and
/// every boolean produces one fresh solid slot; `transform` produces none.
fn produced_handles(ops: &[Value]) -> Vec<Option<u32>> {
    let mut next: u32 = 0;
    ops.iter()
        .map(|op| match op_name(op) {
            "makeBox" | "makeCylinder" | "booleanWithQuality" => {
                let h = next;
                next += 1;
                Some(h)
            }
            _ => None,
        })
        .collect()
}

fn referenced_handles(op: &Value) -> Vec<u32> {
    let args = op.get("args").unwrap_or(&Value::Null);
    let mut out = Vec::new();
    for key in ["solid", "solidA", "solidB"] {
        if let Some(h) = get_u32(args, key) {
            out.push(h);
        }
    }
    out
}

/// Delete `del` plus every transitively dependent operation, then repair
/// handles densely. Returns `None` when nothing would remain.
fn delete_cascade_remap(ops: &[Value], del: usize) -> Option<Vec<Value>> {
    if ops.len() <= 1 {
        return None;
    }
    let produced = produced_handles(ops);
    let producer_of: BTreeMap<u32, usize> = produced
        .iter()
        .enumerate()
        .filter_map(|(i, h)| h.map(|handle| (handle, i)))
        .collect();
    let mut keep = vec![true; ops.len()];
    keep[del] = false;
    loop {
        let mut changed = false;
        for (i, op) in ops.iter().enumerate() {
            if !keep[i] {
                continue;
            }
            let dangling = referenced_handles(op)
                .iter()
                .any(|h| match producer_of.get(h) {
                    None => true,
                    Some(&p) => !keep[p],
                });
            if dangling {
                keep[i] = false;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    if !keep.iter().any(|&k| k) {
        return None;
    }
    // Dense repair: old produced handle -> new dense handle.
    let mut map: BTreeMap<u32, u32> = BTreeMap::new();
    let mut next: u32 = 0;
    for (i, h) in produced.iter().enumerate() {
        if keep[i]
            && let Some(handle) = h
        {
            map.insert(*handle, next);
            next += 1;
        }
    }
    let mut out = Vec::new();
    for (i, op) in ops.iter().enumerate() {
        if !keep[i] {
            continue;
        }
        let mut rewritten = op.clone();
        if produced[i].is_some() || !referenced_handles(op).is_empty() {
            let args = rewritten
                .get_mut("args")
                .and_then(Value::as_object_mut)
                .expect("args object");
            for key in ["solid", "solidA", "solidB"] {
                if let Some(Value::Number(_)) = args.get(key) {
                    let old: u32 =
                        get_u32(&Value::Object(args.clone()), key).expect("numeric handle");
                    let new = map.get(&old).expect("closure keeps references valid");
                    args.insert(key.to_owned(), Value::from(*new));
                }
            }
        }
        out.push(rewritten);
    }
    Some(out)
}

/// One-step parameter simplifications for a single operation: lattice steps
/// toward origins (dims down to 1.0, translations/rotations to identity).
fn simplify_candidates(op: &Value) -> Vec<Value> {
    let mut out = Vec::new();
    let name = op_name(op);
    let mut args = op.get("args").cloned().unwrap_or(Value::Null);
    let Some(obj) = args.as_object_mut() else {
        return out;
    };
    if name == "makeBox" {
        for key in ["width", "height", "depth"] {
            if let Some(v) = get_f64(&Value::Object(obj.clone()), key) {
                for cand in [v - 0.5, 1.0] {
                    if cand >= 1.0 && cand < v {
                        let mut next = obj.clone();
                        next.insert(key.to_owned(), json!(cand));
                        out.push(json!({"op": name, "args": next}));
                    }
                }
            }
        }
    } else if name == "makeCylinder" {
        for key in ["radius", "height"] {
            if let Some(v) = get_f64(&Value::Object(obj.clone()), key) {
                for cand in [v - 0.5, 1.0] {
                    if cand >= 1.0 && cand < v {
                        let mut next = obj.clone();
                        next.insert(key.to_owned(), json!(cand));
                        out.push(json!({"op": name, "args": next}));
                    }
                }
            }
        }
    } else if name == "transform" {
        // Identity, then rotation-only (translation zeroed): either may
        // preserve the failure while deleting a whole degree of freedom.
        out.push(json!({"op": name, "args": {"solid": get_u32(&Value::Object(obj.clone()), "solid"), "matrix": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]}}));
        if let Some(matrix) = obj.get("matrix").and_then(Value::as_array)
            && matrix.len() == 16
        {
            let mut flat: Vec<f64> = matrix.iter().filter_map(Value::as_f64).collect();
            if flat.len() == 16 {
                let translated = flat[3] != 0.0 || flat[7] != 0.0 || flat[11] != 0.0;
                if translated {
                    flat[3] = 0.0;
                    flat[7] = 0.0;
                    flat[11] = 0.0;
                    out.push(
                        json!({"op": name, "args": {"solid": get_u32(&Value::Object(obj.clone()), "solid"), "matrix": flat}}),
                    );
                }
            }
        }
    }
    out
}

/// Shrink `ops` while preserving `key`. Returns the minimized sequence plus
/// before/after operation counts. Timeout witnesses shrink under an
/// attempt cap (each candidate costs the full budget); every other failure
/// class shrinks to fixpoint.
fn shrink_sequence(
    ops: &[Value],
    limits: &SeqLimits,
    fault: Fault,
    key: (SeqKind, &'static str),
) -> (Vec<Value>, usize, usize) {
    let before = ops.len();
    let mut current = ops.to_vec();
    let timeout_cap = if key.0 == SeqKind::Timeout {
        12
    } else {
        usize::MAX
    };
    let mut attempts = 0;
    // Phase A: greedy deletion to fixpoint.
    loop {
        let mut accepted = false;
        for i in 0..current.len() {
            if attempts >= timeout_cap {
                break;
            }
            let Some(candidate) = delete_cascade_remap(&current, i) else {
                continue;
            };
            attempts += 1;
            if failure_key(&execute_sequence(&candidate, limits, fault)) == key {
                current = candidate;
                accepted = true;
                break;
            }
        }
        if !accepted || attempts >= timeout_cap {
            break;
        }
    }
    // Phase B: parameter simplification to fixpoint.
    loop {
        let mut accepted = false;
        for i in 0..current.len() {
            if attempts >= timeout_cap {
                break;
            }
            for candidate in simplify_candidates(&current[i]) {
                let mut trial = current.clone();
                trial[i] = candidate;
                attempts += 1;
                if failure_key(&execute_sequence(&trial, limits, fault)) == key {
                    current = trial;
                    accepted = true;
                    break;
                }
                if attempts >= timeout_cap {
                    break;
                }
            }
            if accepted {
                break;
            }
        }
        if !accepted || attempts >= timeout_cap {
            break;
        }
    }
    // Phase C: deletion again (simplification may unlock new deletions).
    loop {
        let mut accepted = false;
        for i in 0..current.len() {
            if attempts >= timeout_cap {
                break;
            }
            let Some(candidate) = delete_cascade_remap(&current, i) else {
                continue;
            };
            attempts += 1;
            if failure_key(&execute_sequence(&candidate, limits, fault)) == key {
                current = candidate;
                accepted = true;
                break;
            }
        }
        if !accepted || attempts >= timeout_cap {
            break;
        }
    }
    let after = current.len();
    (current, before, after)
}

// ── Campaign ─────────────────────────────────────────────────────────

#[derive(Default)]
struct Tally {
    exact_ok: usize,
    empty: usize,
    refused: usize,
    incorrect: usize,
    crash: usize,
    timeout: usize,
    invalid_handle: usize,
    exact_ok_bools: usize,
    bool_ops: usize,
}

impl Tally {
    fn record(&mut self, report: &SeqReport) {
        match report.kind {
            SeqKind::ExactOk => self.exact_ok += 1,
            SeqKind::Empty => self.empty += 1,
            SeqKind::Refused => self.refused += 1,
            SeqKind::Incorrect => self.incorrect += 1,
            SeqKind::Crash => self.crash += 1,
            SeqKind::Timeout => self.timeout += 1,
            SeqKind::InvalidHandle => self.invalid_handle += 1,
        }
        self.exact_ok_bools += report.exact_ok_bools;
        self.bool_ops += report.bool_ops;
    }

    fn bad(&self) -> usize {
        self.incorrect + self.crash + self.timeout + self.invalid_handle
    }
}

fn campaign_size() -> (usize, u64) {
    let n: usize = std::env::var("OPSEQ81_CASES")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(64);
    let seed: u64 = std::env::var("OPSEQ81_SEED")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0x0081_5EED);
    (n.clamp(1, 4096), seed)
}

// ── Tests ────────────────────────────────────────────────────────────

#[test]
fn op_names_are_exact_only_grammar() {
    // The first-slice grammar is closed: generation emits nothing outside
    // makeBox / makeCylinder / transform / exact-only booleanWithQuality.
    for seed in [0x81, 0x82, 0x83, 0x84] {
        for len in [3, 4, 5, 6] {
            let ops = generate_sequence(seed, len);
            assert_eq!(ops.len(), len, "seed={seed} len={len}");
            for op in &ops {
                let name = op_name(op);
                assert!(
                    matches!(
                        name,
                        "makeBox" | "makeCylinder" | "transform" | "booleanWithQuality"
                    ),
                    "seed={seed}: op {name} outside the first-slice grammar"
                );
                if name == "booleanWithQuality" {
                    assert_eq!(
                        op.get("args")
                            .and_then(|a| a.get("exactOnly"))
                            .and_then(Value::as_bool),
                        Some(true),
                        "seed={seed}: boolean without exactOnly: true"
                    );
                    let bop = op
                        .get("args")
                        .and_then(|a| a.get("operation"))
                        .and_then(Value::as_str)
                        .unwrap_or("");
                    assert!(
                        matches!(bop, "fuse" | "cut" | "intersect"),
                        "seed={seed}: boolean op {bop} outside fuse/cut/intersect"
                    );
                }
            }
            // Explicit dependencies: every boolean consumes live results, so
            // each boolean handle pair must be produced by earlier operations.
            let produced = produced_handles(&ops);
            let table: BTreeMap<u32, usize> = produced
                .iter()
                .enumerate()
                .filter_map(|(i, h)| h.map(|handle| (handle, i)))
                .collect();
            for (i, op) in ops.iter().enumerate() {
                for h in referenced_handles(op) {
                    let p = table.get(&h).copied().unwrap_or(usize::MAX);
                    assert!(
                        p < i,
                        "seed={seed}: op {i} references handle {h} with no earlier producer"
                    );
                }
            }
        }
    }
}

#[test]
fn exported_bundles_are_schema1_and_roundtrip() {
    // The persisted artifact is the executed artifact: export a generated
    // sequence as a schema-1 bundle, parse it back, and re-execute — the
    // failure key must be identical.
    let limits = SeqLimits::default();
    for (seed, len) in [(0x81, 5), (0x82, 4), (0x83, 6), (0x84, 5)] {
        let ops = generate_sequence(seed, len);
        let bundle = to_bundle(
            &format!("opseq81-seed-{seed}"),
            bundle_description(seed, None, &limits),
            &ops,
        );
        assert_eq!(bundle.get("schema").and_then(Value::as_u64), Some(1));
        assert_eq!(
            bundle
                .get("operations")
                .and_then(Value::as_array)
                .map(Vec::len),
            Some(len)
        );
        assert!(bundle.get("expect").and_then(Value::as_array).is_some());
        let text = serde_json::to_string(&bundle).expect("bundle serializes");
        let parsed: Value = serde_json::from_str(&text).expect("bundle parses");
        let back: Vec<Value> = parsed
            .get("operations")
            .and_then(Value::as_array)
            .expect("operations array")
            .clone();
        let direct = execute_sequence(&ops, &limits, Fault::None);
        let replayed = execute_sequence(&back, &limits, Fault::None);
        assert_eq!(
            failure_key(&direct),
            failure_key(&replayed),
            "seed={seed}: bundle round-trip changed the verdict"
        );
    }
}

#[test]
fn ci_regression_matrix() {
    // Small deterministic CI mode: 8 fixed sequences, no environment input.
    // Green means exact success, supported empty, or typed refusal only;
    // the non-vacuity gate requires at least one exact boolean overall.
    let limits = SeqLimits::default();
    let matrix = [
        (0xC101, 5),
        (0xC102, 4),
        (0xC103, 6),
        (0xC104, 5),
        (0xC105, 4),
        (0xC106, 6),
        (0xC107, 5),
        (0xC108, 4),
    ];
    let mut tally = Tally::default();
    for (seed, len) in matrix {
        let ops = generate_sequence(seed, len);
        let report = execute_sequence(&ops, &limits, Fault::None);
        println!(
            "B81CASE seed={seed} len={len} kind={} oracle={} bools={}/{}",
            report.kind.label(),
            report.oracle,
            report.exact_ok_bools,
            report.bool_ops,
        );
        for n in &report.notes {
            if n.kind.severity() >= SeqKind::Incorrect.severity() {
                println!(
                    "B81CASE op {} {}: {}: {}",
                    n.index, n.op, n.oracle, n.detail
                );
            }
        }
        tally.record(&report);
    }
    println!(
        "B81TALLY exact_ok={} empty={} refused={} incorrect={} crash={} timeout={} invalid_handle={} exact_ok_bools={}/{}",
        tally.exact_ok,
        tally.empty,
        tally.refused,
        tally.incorrect,
        tally.crash,
        tally.timeout,
        tally.invalid_handle,
        tally.exact_ok_bools,
        tally.bool_ops,
    );
    assert_eq!(
        tally.bad(),
        0,
        "CI matrix must hold no incorrect/crash/timeout/invalid-handle witness"
    );
    assert!(
        tally.exact_ok_bools >= MIN_EXACT_OK,
        "CI matrix is vacuous: no exact boolean success"
    );
}

/// Hand-built partial-overlap witness: A is a 4×2×2 box, B a 2×2×2 box
/// shifted +2.5 in x (overlap 1.5×2×2, neither center on the other's
/// boundary), fused. Fault-free it must not read Incorrect; with the
/// dropped-operand fault armed on the fuse it must read Incorrect.
fn overlap_witness() -> Vec<Value> {
    let mat = Mat4::translation(2.5, 0.0, 0.0);
    vec![
        op_make_box(4.0, 2.0, 2.0),
        op_make_box(2.0, 2.0, 2.0),
        op_transform(1, &mat),
        op_bool("fuse", 0, 1),
    ]
}

#[test]
fn injected_drop_operand_fuse_is_incorrect() {
    // The harness detects a deliberately injected wrong success: with the
    // fault armed the fuse returns its first operand unchanged, and the
    // complementary identity oracle must reject it.
    let limits = SeqLimits::default();
    let ops = overlap_witness();
    let clean = execute_sequence(&ops, &limits, Fault::None);
    println!(
        "B81CASE clean witness: kind={} oracle={}",
        clean.kind.label(),
        clean.oracle
    );
    assert_ne!(
        clean.kind,
        SeqKind::Incorrect,
        "clean witness must not read Incorrect"
    );
    assert_ne!(
        clean.kind,
        SeqKind::InvalidHandle,
        "clean witness must resolve every handle"
    );

    let injected = execute_sequence(&ops, &limits, Fault::DropOperandFuseAt { op_index: 3 });
    println!(
        "B81CASE injected witness: kind={} oracle={}",
        injected.kind.label(),
        injected.oracle
    );
    for n in &injected.notes {
        println!(
            "B81CASE op {} {}: {}: {}",
            n.index, n.op, n.oracle, n.detail
        );
    }
    assert_eq!(
        injected.kind,
        SeqKind::Incorrect,
        "dropped-operand fuse must read incorrect_success"
    );
}

#[test]
fn shrink_retains_injected_failure() {
    // A longer faulty chain shrinks to a smaller witness with the identical
    // failure key (before/after sizes prove the reduction).
    let limits = SeqLimits::default();
    let mat_b = Mat4::translation(2.5, 0.0, 0.0);
    let mat_c = Mat4::translation(-1.0, 3.0, 0.5) * Mat4::rotation_z(FRAC_PI_2);
    let ops = vec![
        op_make_box(4.0, 2.0, 2.0), // 0
        op_make_box(2.0, 2.0, 2.0), // 1
        op_transform(1, &mat_b),    // 2
        op_bool("fuse", 0, 1),      // 3 <- fault here
        op_make_cylinder(1.0, 2.0), // 4
        op_transform(3, &mat_c),    // 5
        op_bool("cut", 2, 3),       // 6 (uses both chain results)
    ];
    let fault = Fault::DropOperandFuseAt { op_index: 3 };
    let report = execute_sequence(&ops, &limits, fault);
    let key = failure_key(&report);
    assert_eq!(
        key.0,
        SeqKind::Incorrect,
        "faulty chain must read Incorrect before shrinking"
    );
    let (shrunk, before, after) = shrink_sequence(&ops, &limits, fault, key);
    let rereport = execute_sequence(&shrunk, &limits, fault);
    println!(
        "B81SHRINK before={before} after={after} key={:?}",
        failure_key(&rereport)
    );
    assert_eq!(
        failure_key(&rereport),
        key,
        "shrunk witness must preserve the failure key"
    );
    assert!(
        after < before,
        "shrinking must reduce the sequence ({before} -> {after})"
    );
    // Replay the minimized case repeatedly: determinism across runs.
    for _ in 0..5 {
        assert_eq!(
            failure_key(&execute_sequence(&shrunk, &limits, fault)),
            key,
            "minimized witness must replay deterministically"
        );
    }
}

#[test]
fn shrink_rejects_substitute_outcomes() {
    // The failure predicate discriminates: the same operations under no
    // fault, under a dangling handle, and under an exhausted budget produce
    // different keys from the injected incorrect-success witness — so the
    // shrinker can never accept one as a substitute.
    let limits = SeqLimits::default();
    let ops = overlap_witness();
    let fault = Fault::DropOperandFuseAt { op_index: 3 };
    let key_incorrect = failure_key(&execute_sequence(&ops, &limits, fault));

    let key_clean = failure_key(&execute_sequence(&ops, &limits, Fault::None));
    assert_ne!(
        key_clean, key_incorrect,
        "clean run must key differently from the injected witness"
    );

    let mut dangling = ops.clone();
    dangling[3] = op_bool("fuse", 0, 99);
    let key_dangling = failure_key(&execute_sequence(&dangling, &limits, fault));
    assert_eq!(key_dangling.0, SeqKind::InvalidHandle);
    assert_ne!(
        key_dangling, key_incorrect,
        "invalid-handle must not substitute the witness"
    );

    let rushed = SeqLimits {
        timeout_ms: 0,
        ..SeqLimits::default()
    };
    let key_timeout = failure_key(&execute_sequence(&ops, &rushed, fault));
    assert_eq!(key_timeout.0, SeqKind::Timeout);
    assert_ne!(
        key_timeout, key_incorrect,
        "timeout must not substitute the witness"
    );

    // And shrinking the injected witness keeps its exact key (oracle tag included).
    let (shrunk, _, _) = shrink_sequence(&ops, &limits, fault, key_incorrect);
    assert_eq!(
        failure_key(&execute_sequence(&shrunk, &limits, fault)),
        key_incorrect,
        "shrink must retain the exact failure key"
    );
}

#[test]
fn replay_minimized_cases_repeatedly() {
    // Minimized witnesses replay identically across repeated runs, and a
    // primitive-only sequence is deterministically exact.
    let limits = SeqLimits::default();
    let ops = overlap_witness();
    let fault = Fault::DropOperandFuseAt { op_index: 3 };
    let key = failure_key(&execute_sequence(&ops, &limits, fault));
    let (shrunk, _, _) = shrink_sequence(&ops, &limits, fault, key);
    for _ in 0..5 {
        assert_eq!(
            failure_key(&execute_sequence(&shrunk, &limits, fault)),
            key,
            "minimized incorrect witness must replay its key"
        );
    }
    let prims = vec![op_make_box(2.0, 2.0, 2.0), op_make_cylinder(1.0, 2.0)];
    for _ in 0..5 {
        let report = execute_sequence(&prims, &limits, Fault::None);
        assert_eq!(
            report.kind,
            SeqKind::ExactOk,
            "primitive-only sequences replay exact"
        );
    }
}

#[test]
fn timeout_budget_is_enforced() {
    // A zero budget times out before the first operation: the timeout bucket
    // is reachable and reported separately from refusals.
    let tight = SeqLimits {
        timeout_ms: 0,
        ..SeqLimits::default()
    };
    let ops = vec![op_make_box(2.0, 2.0, 2.0)];
    let report = execute_sequence(&ops, &tight, Fault::None);
    assert_eq!(report.kind, SeqKind::Timeout);
}

#[test]
fn crash_taxonomy_catches_panics() {
    // A panic inside an operation is caught at the harness boundary and
    // reported as a crash — it never aborts the suite.
    let limits = SeqLimits::default();
    let ops = vec![op_make_box(2.0, 2.0, 2.0)];
    let report = execute_sequence(&ops, &limits, Fault::PanicAt { op_index: 0 });
    assert_eq!(report.kind, SeqKind::Crash);
}

#[test]
fn bounded_campaign() {
    // Bounded real campaign: `OPSEQ81_CASES` sequences from `OPSEQ81_SEED`
    // (defaults: 64 from 0x815EED), lengths 3..=MAX_SEQ_OPS. Every input is
    // persisted before execution; every non-clean finding is retained as a
    // schema-1 bundle. The denominator and the six-bucket split print as
    // B81TALLY; findings print as B81CASE with replay paths.
    let limits = SeqLimits::default();
    let (n, seed) = campaign_size();
    let mut tally = Tally::default();
    let mut finding_paths: Vec<String> = Vec::new();
    for i in 0..n {
        let case_seed = seed
            .wrapping_add(i as u64)
            .wrapping_mul(0x9E37_79B9_7F4A_7C15);
        let len = 3 + (i % (MAX_SEQ_OPS - 2));
        let ops = generate_sequence(case_seed, len);
        let name = format!("opseq81-seed-{case_seed}");
        let input_path = persist_inputs(&name, case_seed, &ops, &limits);
        let report = execute_sequence(&ops, &limits, Fault::None);
        if report.kind == SeqKind::InvalidHandle
            || report.kind.severity() >= SeqKind::Incorrect.severity()
        {
            let finding = to_bundle(
                &format!("{name}-{}", report.kind.label()),
                bundle_description(case_seed, Some(&report), &limits),
                &ops,
            );
            let path = findings_dir().join(format!("{name}-{}.json", report.kind.label()));
            std::fs::write(
                &path,
                serde_json::to_string_pretty(&finding).expect("bundle serializes"),
            )
            .expect("findings dir is writable");
            finding_paths.push(path.display().to_string());
            println!(
                "B81CASE seed={case_seed} kind={} oracle={}",
                report.kind.label(),
                report.oracle
            );
            for note in &report.notes {
                if note.kind == report.kind {
                    println!(
                        "B81CASE op {} {}: {}: {}",
                        note.index, note.op, note.oracle, note.detail
                    );
                }
            }
            println!("B81CASE inputs: {}", input_path.display());
        } else if report.kind == SeqKind::Refused {
            println!(
                "B81CASE seed={case_seed} kind=refused oracle={}",
                report.oracle
            );
            for note in &report.notes {
                if note.kind == SeqKind::Refused {
                    println!(
                        "B81CASE op {} {}: {}: {}",
                        note.index, note.op, note.oracle, note.detail
                    );
                }
            }
        }
        tally.record(&report);
    }
    println!(
        "B81TALLY n={n} seed={seed} exact_ok={} empty={} refused={} incorrect={} crash={} timeout={} invalid_handle={} exact_ok_bools={}/{}",
        tally.exact_ok,
        tally.empty,
        tally.refused,
        tally.incorrect,
        tally.crash,
        tally.timeout,
        tally.invalid_handle,
        tally.exact_ok_bools,
        tally.bool_ops,
    );
    assert_eq!(
        tally.bad(),
        0,
        "campaign holds {} finding(s): {}",
        finding_paths.len(),
        finding_paths.join(", ")
    );
    assert!(
        tally.exact_ok_bools >= MIN_EXACT_OK,
        "campaign is vacuous: no exact boolean success in {n} sequences"
    );
}

/// 8.1 finding F1 (retained, kernel untouched): swapped-operand coaxial
/// cylinder intersect carries an inconsistent-orientation edge.
///
/// Minimized from bounded-campaign seeds `10006000559475869461`
/// (cylinders r=2.5/h=4.0 × r=2.0/h=5.0) and `12304682496488374415`
/// (r=5.0/h=2.5 × r=2.0/h=5.0) — both unmoved coaxial pairs — via
/// delete-cascade + parameter simplification (3 ops stay 3 ops; dims shrink
/// to r=1.5/h=1.0 × r=1.0/h=1.5). The same shapes in the opposite operand
/// order validate clean with identical volumes, so this is an
/// order-dependent assembly-orientation defect, B33-family affinity (the
/// row whose cylinder families fail the supplement while the ops gate stays
/// clean — here even the ops gate fails).
///
/// Acceptance: exact intersect, valid topology, volume π (r=1/h=1 overlap).
/// No geometry fix in this harness PR; un-ignore when green.
#[test]
#[ignore = "8.1 finding F1: swapped coaxial cylinder intersect misorients one shared edge; route to the B33 orientation family"]
fn finding_coax_cylinder_intersect_orientation() {
    let ops: Vec<Value> = serde_json::from_str(
        r#"[{"args":{"height":1.0,"radius":1.5},"op":"makeCylinder"},{"args":{"height":1.5,"radius":1.0},"op":"makeCylinder"},{"args":{"exactOnly":true,"operation":"intersect","solidA":1,"solidB":0},"op":"booleanWithQuality"}]"#,
    )
    .expect("finding bundle parses");
    let limits = SeqLimits::default();
    let report = execute_sequence(&ops, &limits, Fault::None);
    for n in &report.notes {
        println!(
            "B81FINDING op {} {}: {}: {}",
            n.index, n.op, n.oracle, n.detail
        );
    }
    assert_eq!(
        report.kind,
        SeqKind::ExactOk,
        "coaxial intersect must be exact and valid in either operand order"
    );
    assert!(
        (report.exact_ok_bools >= 1),
        "the intersect itself must succeed exactly"
    );
}

/// 8.1 finding F2 (retained, kernel untouched): the same swapped coaxial
/// pair fused instead of intersected carries the same inconsistent
/// shared-edge orientation (verified by direct native replay of both
/// operand orders: small-first invalid, big-first valid, identical volumes
/// and face counts).
///
/// Acceptance: exact fuse, valid topology, volume 8.639379797
/// (7.068583 + 4.712389 − π overlap). No geometry fix in this harness PR.
#[test]
#[ignore = "8.1 finding F2: swapped coaxial cylinder fuse misorients one shared edge; route to the B33 orientation family"]
fn finding_coax_cylinder_fuse_orientation() {
    let ops: Vec<Value> = serde_json::from_str(
        r#"[{"args":{"height":1.0,"radius":1.5},"op":"makeCylinder"},{"args":{"height":1.5,"radius":1.0},"op":"makeCylinder"},{"args":{"exactOnly":true,"operation":"fuse","solidA":1,"solidB":0},"op":"booleanWithQuality"}]"#,
    )
    .expect("finding bundle parses");
    let limits = SeqLimits::default();
    let report = execute_sequence(&ops, &limits, Fault::None);
    for n in &report.notes {
        println!(
            "B81FINDING op {} {}: {}: {}",
            n.index, n.op, n.oracle, n.detail
        );
    }
    assert_eq!(
        report.kind,
        SeqKind::ExactOk,
        "coaxial fuse must be exact and valid in either operand order"
    );
}

/// 8.1 finding F3 (retained, kernel untouched): a cut whose tool is a
/// 30°-rotated cylinder returns a degenerate 3-face solid measuring 0.0
/// while the paired fuse (170.19 against stock+tool 186.53) implies a ~27.6
/// remainder.
///
/// Minimized from campaign seed `17450404173350609447` (4 ops stay 4 ops;
/// dims simplify). The result validates clean yet tessellates open at every
/// deflection (36 boundary edges even at 0.1) and the independent Gauss
/// route declines to measure it — a material-loss wrong success, not a
/// tessellation-density artifact.
///
/// Acceptance: the harness `ExactOk` verdict (valid topology, watertight
/// mesh, mass agreement, translation invariance, identities). No geometry
/// fix in this harness PR.
#[test]
#[ignore = "8.1 finding F3: rotated-cylinder-tool cut drops its remainder (valid 3-face 0-volume result, open mesh, refused mass route)"]
fn finding_rotated_cyl_cut_material() {
    let ops: Vec<Value> = serde_json::from_str(
        r#"[{"args":{"height":1.5,"radius":5.5},"op":"makeCylinder"},{"args":{"height":3.5,"radius":2.0},"op":"makeCylinder"},{"args":{"matrix":[1.0,0.0,0.0,4.0,0.0,0.8660254037844386,-0.5,-2.0,0.0,0.5,0.8660254037844386,-0.5,0.0,0.0,0.0,1.0],"solid":0},"op":"transform"},{"args":{"exactOnly":true,"operation":"cut","solidA":1,"solidB":0},"op":"booleanWithQuality"}]"#,
    )
    .expect("finding bundle parses");
    let limits = SeqLimits::default();
    let report = execute_sequence(&ops, &limits, Fault::None);
    for n in &report.notes {
        println!(
            "B81FINDING op {} {}: {}: {}",
            n.index, n.op, n.oracle, n.detail
        );
    }
    assert_eq!(
        report.kind,
        SeqKind::ExactOk,
        "rotated-tool cut must keep an exact, valid, watertight remainder"
    );
}

/// 8.1 finding F4 (retained, kernel untouched): an intersect over a
/// −90°-rotated cylinder validates clean and meshes watertight, yet three
/// volume readings disagree wildly on the same body — tessellated 42.74,
/// Gauss 26.07, moved-tessellated 18.18.
///
/// Minimized from campaign seed `4329058900930561401` (6 ops shrink to 4).
/// No closed form exists for oblique cylinder pairs, so the acceptance is
/// cross-route agreement rather than an exact value — documented as weaker
/// than the closed-form oracles, and the available ground truth for this
/// configuration.
///
/// Acceptance: the harness `ExactOk` verdict (includes mass agreement and
/// translation invariance). No geometry fix in this harness PR.
#[test]
#[ignore = "8.1 finding F4: rotated-cylinder intersect measures 42.74/26.07/18.18 across routes (no closed form; cross-route agreement required)"]
fn finding_rotated_cyl_intersect_volumes() {
    let ops: Vec<Value> = serde_json::from_str(
        r#"[{"args":{"height":3.5,"radius":4.5},"op":"makeCylinder"},{"args":{"height":6.0,"radius":6.5},"op":"makeCylinder"},{"args":{"matrix":[1.0,0.0,0.0,-1.5,0.0,-1.8369701987210297e-16,1.0,3.0,0.0,-1.0,-1.8369701987210297e-16,-2.5,0.0,0.0,0.0,1.0],"solid":0},"op":"transform"},{"args":{"exactOnly":true,"operation":"intersect","solidA":1,"solidB":0},"op":"booleanWithQuality"}]"#,
    )
    .expect("finding bundle parses");
    let limits = SeqLimits::default();
    let report = execute_sequence(&ops, &limits, Fault::None);
    for n in &report.notes {
        println!(
            "B81FINDING op {} {}: {}: {}",
            n.index, n.op, n.oracle, n.detail
        );
    }
    assert_eq!(
        report.kind,
        SeqKind::ExactOk,
        "rotated intersect must measure consistently across all three volume routes"
    );
}

/// 8.1 finding F5 (retained, kernel untouched): an unmoved
/// cylinder-minus-box cut carries 8 shared edges with inconsistent face
/// orientations while reading a plausible tessellated volume (36.98) —
/// the Gauss route collapses to 7.86 on the same body.
///
/// Minimized from campaign seed `13916448401417584040` (5 ops shrink to 3).
/// Orientation-family affinity with F1/F2 (B33), but on a box–cylinder cut
/// pair rather than a coaxial cylinder pair, so it owns a distinct
/// exclusion cell (X2) and repro.
///
/// Acceptance: the harness `ExactOk` verdict (valid topology with agreeing
/// volumes). No geometry fix in this harness PR.
#[test]
#[ignore = "8.1 finding F5: unmoved cyl-minus-box cut misorients 8 shared edges (tess 36.98 vs Gauss 7.86)"]
fn finding_box_cyl_cut_orientation() {
    let ops: Vec<Value> = serde_json::from_str(
        r#"[{"args":{"depth":6.5,"height":1.0,"width":1.5},"op":"makeBox"},{"args":{"height":6.5,"radius":1.5},"op":"makeCylinder"},{"args":{"exactOnly":true,"operation":"cut","solidA":1,"solidB":0},"op":"booleanWithQuality"}]"#,
    )
    .expect("finding bundle parses");
    let limits = SeqLimits::default();
    let report = execute_sequence(&ops, &limits, Fault::None);
    for n in &report.notes {
        println!(
            "B81FINDING op {} {}: {}: {}",
            n.index, n.op, n.oracle, n.detail
        );
    }
    assert_eq!(
        report.kind,
        SeqKind::ExactOk,
        "unmoved cyl-minus-box cut must validate clean with agreeing volumes"
    );
}
