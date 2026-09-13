//! B26 boolean-correctness campaign: bounded invariant properties over
//! generated primitive pairs, cavity solids, rigid transforms, and scales.
//!
//! Roadmap row B26 asks for property tests over random primitive pairs and
//! rigid transforms: inclusion–exclusion volume identity, fuse/cut
//! complement, translation invariance of `solid_volume`, watertight/manifold
//! mesh, and exact-only path stability under 1e-13 nudges. This file is the
//! small deterministic CI replay; a larger opt-in campaign is documented
//! below.
//!
//! ## Campaign sizes
//!
//! - CI replay (default): `PROP_BOOL_CASES` unset → a fixed matrix of 27
//!   cases across 7 families (box-pair, box-cylinder, cavity, rigid,
//!   scale, spheres, nudge). Bounded and deterministic; the curved families
//!   use one placement each so CI stays under ~2 minutes.
//! - Opt-in campaign: `PROP_BOOL_CASES=<n> PROP_BOOL_SEED=<u64>` runs `n`
//!   generated cases from a SplitMix64 stream seeded by `PROP_BOOL_SEED`
//!   (default seed 0xB26). Bounded: `n` is clamped to `[1, 4096]` and each
//!   case is one bounded primitive pair (depth-1 tree, ≤2 leaves), so work
//!   scales linearly with `n`. Example:
//!   `PROP_BOOL_CASES=512 PROP_BOOL_SEED=7 cargo test -p remus-operations
//!   --test prop_boolean_invariants -- --nocapture`.
//!
//! ## Oracles (independent of the kernel paths under test)
//!
//! Expected values come only from closed forms evaluated by hand in this
//! file (box `dx·dy·dz`, cylinder `πr²h`, sphere `4/3πr³`), from the
//! equal-sphere lens `π(4r+d)(2r−d)²/12`, from set identities over those
//! closed forms, and from topology/mesh structure — never from another
//! kernel measurement path that could share the defect. `solid_volume` is
//! used only as the *reading* under test, and every family first pins the
//! operand closed forms before checking any identity.
//!
//! ## Outcome taxonomy
//!
//! Every boolean runs through the exact-only entry point (`boolean()`), so
//! each attempt is exactly one of:
//! - `exact_ok`: success with all family oracles green;
//! - `typed_refusal`: `EmptyResult`, `NonManifoldResult`,
//!   `ExactOnlyUnattainable`, `Unsupported` (or their body-class/measure
//!   siblings) — the engine declined rather than answering wrong;
//! - `incorrect`: success with a violated oracle — a campaign finding.
//!
//! Universal refusal must NOT pass: each family asserts a minimum number of
//! exact successes (`MIN_EXACT_OK`), so a kernel that refuses everything
//! fails loudly instead of going green by declining.
//!
//! ## Boundary vs instability
//!
//! Tangency and contact transitions (face-on-face, edge-on-edge,
//! vertex-on-face, containment) are *expected* typed refusals and pass.
//! What fails is *instability*: a 1e-13 input nudge — six orders below the
//! kernel's 1e-7 linear tolerance — flipping the outcome kind
//! (refusal↔success). The nudge oracle compares refusal-versus-success,
//! not success-only, so an exact path that succeeds on one side of a
//! rounding bit and refuses on the other is caught.
//!
//! ## Shrinking and replay
//!
//! Generation is a pure function of `(seed, index)`: the campaign runner
//! prints `B26CASE seed=<s> index=<i> family=<f> params=<...>` for every
//! incorrect case, and `B26SHRINK` lines as it minimizes. Shrinking walks
//! each numeric parameter toward its lattice origin (offsets → 0,
//! dimensions → lattice minimum, rotations → 0, scale → 1) while the oracle
//! still fails, then replays the minimized case as a standalone `#[test]`
//! body printed to stdout. Pinned minimized cases live at the bottom of
//! this file as ordinary tests.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
#![allow(clippy::too_many_lines, clippy::cast_possible_truncation)]
// Campaign diagnostics print case/tallies to stdout by design (seed replay);
// the workspace denies `print_stdout`, so allow it file-wide here.
#![allow(clippy::print_stdout)]
#![allow(
    clippy::type_complexity,
    clippy::collapsible_if,
    clippy::single_element_loop
)]
#![allow(
    clippy::too_many_arguments,
    clippy::approx_constant,
    clippy::items_after_statements
)]

use std::collections::BTreeMap;
use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder, make_sphere};
use remus_operations::shell_op::shell;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::explorer::{self, solid_faces};
use remus_topology::solid::SolidId;

// ── Campaign configuration ───────────────────────────────────────────

/// Fixed CI offsets for the axis-aligned families.
const CI_OFFSETS: [f64; 4] = [0.25, 0.5, 1.0, 2.5];
/// Minimum exact successes per family in CI replay: universal refusal fails.
const MIN_EXACT_OK: usize = 1;
/// Opt-in campaign bounds.
const MAX_CAMPAIGN_CASES: usize = 4096;
const DEFAULT_SEED: u64 = 0xB26;
/// Volume reading deflection (matches `boolean_invariants.rs` precedent).
const DEFLECTION: f64 = 0.1;
/// Identity slack: relative, deliberately loose (gross-disagreement
/// detector, not a precision check). Never tightened to pass.
const REL_SLACK: f64 = 1e-2;
/// Absolute floor so near-zero volumes do not divide the relative test.
const VOL_FLOOR: f64 = 1e-6;
/// Nudge magnitude: six orders below the 1e-7 linear tolerance.
const NUDGE: f64 = 1e-13;
/// Position-quantization grid: coarser than linear tolerance (no last-bit
/// splits), far finer than the half-unit generation lattice (no false
/// merges of distinct vertices).
const POS_GRID: f64 = 1e-6;

// ── Closed-form oracles (hand-derived, independent of the kernel) ─────

fn closed_box(dx: f64, dy: f64, dz: f64) -> f64 {
    dx * dy * dz
}

fn closed_cylinder(r: f64, h: f64) -> f64 {
    PI * r * r * h
}

fn closed_sphere(r: f64) -> f64 {
    4.0 / 3.0 * PI * r * r * r
}

#[allow(clippy::unnecessary_wraps)]
fn closed_lens_equal_spheres(r: f64, d: f64) -> Option<f64> {
    if !(d > 0.0 && d < 2.0 * r) {
        return None;
    }
    Some(PI * (4.0 * r + d) * (2.0 * r - d).powi(2) / 12.0)
}

// ── Deterministic stream (SplitMix64; no external RNG dependency) ─────

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

    fn range_f64(&mut self, lo: f64, hi: f64, steps: u64) -> f64 {
        lo + (hi - lo) * (self.below(steps) as f64 / steps as f64)
    }
}

fn campaign_size() -> Option<(usize, u64)> {
    let n: usize = std::env::var("PROP_BOOL_CASES").ok()?.parse().ok()?;
    let seed: u64 = std::env::var("PROP_BOOL_SEED")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(DEFAULT_SEED);
    Some((n.clamp(1, MAX_CAMPAIGN_CASES), seed))
}

// ── Outcome taxonomy ─────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Outcome {
    ExactOk,
    TypedRefusal,
    Incorrect,
}

fn classify_refusal(e: &OperationsError) -> bool {
    matches!(
        e,
        OperationsError::EmptyResult { .. }
            | OperationsError::NonManifoldResult
            | OperationsError::ExactOnlyUnattainable
            | OperationsError::Unsupported { .. }
            | OperationsError::BodyClassOperationUnsupported { .. }
            | OperationsError::BodyClassMeasureMismatch { .. }
            | OperationsError::BodyValidationFailed { .. }
    )
}

#[derive(Debug, Default)]
struct Tally {
    exact_ok: usize,
    typed_refusal: usize,
    incorrect: usize,
    incorrect_descriptions: Vec<String>,
}

impl Tally {
    fn record(&mut self, outcome: Outcome, description: String) {
        match outcome {
            Outcome::ExactOk => self.exact_ok += 1,
            Outcome::TypedRefusal => self.typed_refusal += 1,
            Outcome::Incorrect => {
                self.incorrect += 1;
                self.incorrect_descriptions.push(description);
            }
        }
    }

    /// Merge another family's tally into this one (counts only; incorrect
    /// descriptions are preserved for reporting).
    fn merge(&mut self, other: &Self) {
        self.exact_ok += other.exact_ok;
        self.typed_refusal += other.typed_refusal;
        self.incorrect += other.incorrect;
        self.incorrect_descriptions
            .extend(other.incorrect_descriptions.iter().cloned());
    }

    fn finish(&self, family: &str) {
        println!(
            "B26TALLY family={family} exact_ok={} typed_refusal={} incorrect={}",
            self.exact_ok, self.typed_refusal, self.incorrect
        );
        for d in &self.incorrect_descriptions {
            println!("B26CASE {d}");
        }
        assert!(
            self.exact_ok >= MIN_EXACT_OK,
            "{family}: only {} exact successes (< {MIN_EXACT_OK}); \
             universal refusal must not pass",
            self.exact_ok
        );
        assert!(
            self.incorrect == 0,
            "{family}: {} incorrect success(es):\n{}",
            self.incorrect,
            self.incorrect_descriptions.join("\n")
        );
    }
}

// ── Shared readings ──────────────────────────────────────────────────

fn vol(topo: &Topology, solid: SolidId) -> f64 {
    solid_volume(topo, solid, DEFLECTION).unwrap()
}

fn rel_err(a: f64, b: f64) -> f64 {
    if !a.is_finite() || !b.is_finite() {
        return f64::INFINITY;
    }
    (a - b).abs() / a.abs().max(b.abs()).max(VOL_FLOOR)
}

/// Closed B-Rep check over outer + inner (cavity) shells: every geometric
/// edge key must have exactly 2 uses. Keys are quantized endpoints plus the
/// curve midpoint, so distinct arcs sharing endpoints never merge while
/// genuine duplicates share one key.
fn pos_key(p: remus_math::vec::Point3) -> (i64, i64, i64) {
    let q = |v: f64| (v / POS_GRID).round() as i64;
    (q(p.x()), q(p.y()), q(p.z()))
}

fn edge_midpoint_key(
    topo: &Topology,
    eid: remus_topology::edge::EdgeId,
) -> Option<((i64, i64, i64), (i64, i64, i64), (i64, i64, i64))> {
    let edge = topo.edge(eid).ok()?;
    let a = topo.vertex(edge.start()).ok()?.point();
    let b = topo.vertex(edge.end()).ok()?.point();
    let ka = pos_key(a);
    let kb = pos_key(b);
    let ends = if ka <= kb { (ka, kb) } else { (kb, ka) };
    let (t0, t1) = edge
        .strict_domain()
        .ok()
        .or_else(|| Some(edge.curve().reconstruct_domain_from_endpoints(a, b)))?;
    if !t0.is_finite() || !t1.is_finite() {
        return None;
    }
    let mid = edge.curve().evaluate_with_endpoints(0.5 * (t0 + t1), a, b);
    if !mid.x().is_finite() || !mid.y().is_finite() || !mid.z().is_finite() {
        return None;
    }
    Some((ends.0, ends.1, pos_key(mid)))
}

fn position_closed(topo: &Topology, solid: SolidId) -> Result<(usize, usize), String> {
    let mut counts: BTreeMap<((i64, i64, i64), (i64, i64, i64), (i64, i64, i64)), usize> =
        BTreeMap::new();
    let data = topo
        .solid(solid)
        .map_err(|e| format!("solid lookup: {e:?}"))?;
    let shells = std::iter::once(data.outer_shell()).chain(data.inner_shells().iter().copied());
    for shell_id in shells {
        let shell = topo
            .shell(shell_id)
            .map_err(|e| format!("shell lookup: {e:?}"))?;
        for fid in shell.faces().to_vec() {
            let face = topo.face(fid).map_err(|e| format!("face lookup: {e:?}"))?;
            for wire_id in
                std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                let wire = topo
                    .wire(wire_id)
                    .map_err(|e| format!("wire lookup: {e:?}"))?;
                for oe in wire.edges().to_vec() {
                    if let Some(key) = edge_midpoint_key(topo, oe.edge()) {
                        *counts.entry(key).or_default() += 1;
                    }
                }
            }
        }
    }
    let free = counts.values().filter(|&&c| c == 1).count();
    let non_manifold = counts.values().filter(|&&c| c >= 3).count();
    Ok((free, non_manifold))
}

fn expected_empty_interpretation(
    topo: &Topology,
    solid: SolidId,
    what: &str,
) -> Result<bool, String> {
    // An empty solid (no faces) is the kernel's representation of the empty
    // set for disjoint intersections. It carries no geometry to validate;
    // accept it here and let the volume oracle (expecting ~0) judge it.
    // A *non-empty* face-carrying result still runs the full gate below.
    let faces = solid_faces(topo, solid).map_err(|e| format!("{what}: faces: {e:?}"))?;
    Ok(faces.is_empty())
}

fn check_valid_closed_oriented(topo: &Topology, solid: SolidId, what: &str) -> Result<(), String> {
    // Authoritative gate: the operations-layer validator, which is what the
    // boolean pipeline itself enforces (`boolean_transacted`, assembly).
    // It accepts multi-component results when every component is
    // independently closed and Euler-consistent (disjoint/tangent fuse).
    //
    // Exception: closed hollow bodies from `shell()` store the cavity wall
    // as a second edge-connected component of the outer shell (two closed
    // surfaces by construction). The strict gate rejects that; the relaxed
    // gate — which is what `shell()` itself enforces — accepts it. So when
    // strict fails, fall back to relaxed, and only fail if both refuse.
    // Both gates are kernel validators, not oracles; the independent checks
    // (closed-form volumes, position-quantized edges, mesh) still judge.
    let strict = remus_operations::validate::validate_solid(topo, solid)
        .map_err(|e| format!("{what}: validator error: {e:?}"))?;
    if strict.is_valid() {
        return check_supplements(topo, solid, what);
    }
    let relaxed = remus_operations::validate::validate_solid_relaxed(topo, solid)
        .map_err(|e| format!("{what}: validator error: {e:?}"))?;
    if !relaxed.is_valid() {
        return Err(format!(
            "{what}: invalid result (strict + relaxed both refuse; relaxed: {:?})",
            relaxed
                .issues
                .iter()
                .map(|i| &i.description)
                .collect::<Vec<_>>()
        ));
    }
    check_supplements(topo, solid, what)
}

fn check_supplements(topo: &Topology, solid: SolidId, what: &str) -> Result<(), String> {
    // Independent supplement: the check-crate validator, EXCEPT its
    // by-edge-id shell-connectivity check, which rejects legitimate
    // multi-component disjoint-fuse results by construction (two boxes with
    // a clear gap share no edge, so no single component spans the shell).
    // Every other check-crate finding still fails.
    let mut options = remus_check::validate::ValidateOptions::default();
    options
        .disabled_checks
        .insert(remus_check::validate::CheckId::ShellConnected);
    let check_report = remus_check::validate::validate_solid(topo, solid, &options)
        .map_err(|e| format!("{what}: validator error: {e:?}"))?;
    // The Euler warning is per-shell-summed and cannot pass a two-component
    // result either; filter it when the solid genuinely has two closed
    // components (each Euler-consistent on its own).
    let real_errors: Vec<_> = check_report
        .issues
        .iter()
        .filter(|i| {
            i.severity == remus_check::validate::Severity::Error
                && i.check != remus_check::validate::CheckId::SolidEulerCharacteristic
        })
        .collect();
    if !real_errors.is_empty() {
        return Err(format!(
            "{what}: check-crate invalid result ({} issue(s))",
            real_errors.len()
        ));
    }
    let (free, non_manifold) = position_closed(topo, solid).map_err(|e| format!("{what}: {e}"))?;
    if free != 0 || non_manifold != 0 {
        return Err(format!(
            "{what}: position-quantized edges: {free} free, {non_manifold} non-manifold"
        ));
    }
    Ok(())
}

fn check_watertight_mesh(topo: &Topology, solid: SolidId, what: &str) -> Result<(), String> {
    let mesh = tessellate_solid(topo, solid, DEFLECTION)
        .map_err(|e| format!("{what}: tessellate: {e:?}"))?;
    let boundary = boundary_edge_count(&mesh);
    let non_manifold = non_manifold_edge_count(&mesh);
    if boundary != 0 || non_manifold != 0 {
        return Err(format!(
            "{what}: mesh not watertight: {boundary} boundary edges, {non_manifold} non-manifold"
        ));
    }
    Ok(())
}

fn check_translation_invariant(topo: &Topology, solid: SolidId, what: &str) -> Result<(), String> {
    let v0 = vol(topo, solid);
    let mut moved = topo.clone();
    remus_operations::transform::transform_solid(
        &mut moved,
        solid,
        &Mat4::translation(13.0, -7.0, 5.0),
    )
    .map_err(|e| format!("{what}: translate refused: {e:?}"))?;
    let v1 = solid_volume(&moved, solid, DEFLECTION)
        .map_err(|e| format!("{what}: re-measure refused: {e:?}"))?;
    if rel_err(v0, v1) > REL_SLACK {
        return Err(format!(
            "{what}: volume moved {v0:.9} -> {v1:.9} under translation"
        ));
    }
    Ok(())
}

// ── Operand builders (each returns the solid + its closed-form volume) ─

/// Box cornered at the origin; closed form by hand.
fn stock_box(topo: &mut Topology, dx: f64, dy: f64, dz: f64) -> (SolidId, f64) {
    let s = make_box(topo, dx, dy, dz).expect("valid box dims");
    (s, closed_box(dx, dy, dz))
}

fn tool_box_at(
    topo: &mut Topology,
    dx: f64,
    dy: f64,
    dz: f64,
    x: f64,
    y: f64,
    z: f64,
) -> (SolidId, f64) {
    let s = make_box(topo, dx, dy, dz).expect("valid box dims");
    remus_operations::transform::transform_solid(topo, s, &Mat4::translation(x, y, z))
        .expect("translation applies");
    (s, closed_box(dx, dy, dz))
}

fn tool_cylinder_at(topo: &mut Topology, r: f64, h: f64, x: f64, y: f64, z: f64) -> (SolidId, f64) {
    let s = make_cylinder(topo, r, h).expect("valid cylinder dims");
    remus_operations::transform::transform_solid(topo, s, &Mat4::translation(x, y, z))
        .expect("translation applies");
    (s, closed_cylinder(r, h))
}

fn tool_sphere_at_seg(
    topo: &mut Topology,
    r: f64,
    seg: usize,
    x: f64,
    y: f64,
    z: f64,
) -> (SolidId, f64) {
    let s = make_sphere(topo, r, seg).expect("valid sphere dims");
    remus_operations::transform::transform_solid(topo, s, &Mat4::translation(x, y, z))
        .expect("translation applies");
    (s, closed_sphere(r))
}

/// Pin the operand closed forms through the kernel's own volume reading
/// before any identity runs. A failure here means the oracle path is
/// broken, not that the boolean is wrong.
fn pin_operands(
    topo: &Topology,
    a: SolidId,
    va: f64,
    b: SolidId,
    vb: f64,
    what: &str,
) -> Result<(), String> {
    for (solid, expected, tag) in [(a, va, "A"), (b, vb, "B")] {
        let measured = vol(topo, solid);
        if rel_err(measured, expected) > REL_SLACK {
            return Err(format!(
                "{what}: operand {tag} closed form broken: measured {measured:.9}, expected {expected:.9}"
            ));
        }
        check_translation_invariant(topo, solid, &format!("{what} operand {tag}"))?;
    }
    Ok(())
}

// ── Family 1: box pairs — inclusion–exclusion + cut complement ────────
//
// Masking rule: each paired operation is validated independently FIRST.
// A permitted refusal on one op never discards another op's successful but
// incorrect result. Only identities needing several outputs at once are
// gated on all of them succeeding.

/// Validate one successful box-pair result (topology + mesh + translation
/// invariance), including the empty-intersection representation.
fn check_box_pair_result(
    topo: &Topology,
    s: SolidId,
    tag: &str,
    what: &str,
    allow_empty: bool,
) -> Result<(), String> {
    match expected_empty_interpretation(topo, s, &format!("{what} {tag}")) {
        Ok(true) => {
            if allow_empty {
                Ok(())
            } else {
                Err(format!("{what} {tag}: unexpectedly empty result"))
            }
        }
        Ok(false) => {
            check_valid_closed_oriented(topo, s, &format!("{what} {tag}"))?;
            check_watertight_mesh(topo, s, &format!("{what} {tag}"))?;
            check_translation_invariant(topo, s, &format!("{what} {tag}"))?;
            Ok(())
        }
        Err(e) => Err(e),
    }
}

/// Classify one paired-operation error: a typed refusal passes, anything
/// else is incorrect.
fn check_paired_error(r: &Result<SolidId, OperationsError>, what: &str) -> Result<bool, String> {
    match r {
        Ok(_) => Ok(true),
        Err(e) => {
            if classify_refusal(e) {
                Ok(false)
            } else {
                Err(format!("{what}: untyped boolean error: {e:?}"))
            }
        }
    }
}

fn family_box_pair(offset: f64) -> Outcome {
    let what = format!("box-pair offset={offset}");
    let mut topo = Topology::new();
    // Stock [0,2]^3; tool [offset,offset+2]^3 shifted along x.
    let (a, va) = stock_box(&mut topo, 2.0, 2.0, 2.0);
    let (b, vb) = tool_box_at(&mut topo, 2.0, 2.0, 2.0, offset, 0.0, 0.0);
    if let Err(e) = pin_operands(&topo, a, va, b, vb, &what) {
        println!("B26CASE {what}: operand pin failed: {e}");
        return Outcome::Incorrect;
    }

    // Fresh operand pairs per op: booleans retire operand entities.
    let mut run = |op| -> Result<SolidId, OperationsError> {
        let (x, _) = stock_box(&mut topo, 2.0, 2.0, 2.0);
        let (y, _) = tool_box_at(&mut topo, 2.0, 2.0, 2.0, offset, 0.0, 0.0);
        boolean(&mut topo, op, x, y)
    };
    let (rf, ri, rc) = (
        run(BooleanOp::Fuse),
        run(BooleanOp::Intersect),
        run(BooleanOp::Cut),
    );
    // Validate every Ok result independently before looking at refusals:
    // a permitted refusal on one op must not mask an incorrect success on
    // another.
    let mut vols: [Option<f64>; 3] = [None, None, None];
    for (r, tag, slot, allow_empty) in [
        (&rf, "fuse", 0, false),
        (&ri, "intersect", 1, true),
        (&rc, "cut", 2, false),
    ] {
        if let Ok(s) = r {
            if let Err(e) = check_box_pair_result(&topo, *s, tag, &what, allow_empty) {
                println!("B26CASE {e}");
                return Outcome::Incorrect;
            }
            let overlap = (2.0 - offset.abs()).max(0.0) * 4.0;
            let expected = [va + vb - overlap, overlap, va - overlap][slot];
            if rel_err(vol(&topo, *s), expected) > REL_SLACK {
                println!("B26CASE {what} {tag}: independent volume oracle failed");
                return Outcome::Incorrect;
            }
            vols[slot] = Some(vol(&topo, *s));
        }
    }
    // Classify refusals only after all successes have been judged.
    let mut ok_count = 0;
    for r in [&rf, &ri, &rc] {
        match check_paired_error(r, &what) {
            Ok(true) => ok_count += 1,
            Ok(false) => {}
            Err(e) => {
                println!("B26CASE {e}");
                return Outcome::Incorrect;
            }
        }
    }
    if ok_count < 3 {
        return Outcome::TypedRefusal;
    }
    let [Some(vf), Some(vi), Some(vc)] = vols else {
        println!("B26CASE {what}: internal harness error: missing volumes");
        return Outcome::Incorrect;
    };

    // Oracle 1: inclusion–exclusion over closed forms.
    if rel_err(vf + vi, va + vb) > REL_SLACK {
        println!(
            "B26CASE {what}: inclusion-exclusion fails: fuse {vf:.9} + inter {vi:.9} != A {va:.9} + B {vb:.9}"
        );
        return Outcome::Incorrect;
    }
    // Oracle 2: cut complement over closed forms.
    if rel_err(vc + vi, va) > REL_SLACK {
        println!("B26CASE {what}: cut complement fails: cut {vc:.9} + inter {vi:.9} != A {va:.9}");
        return Outcome::Incorrect;
    }
    Outcome::ExactOk
}

// ── Family 2: box × cylinder — closed-form intersection oracle ────────
//
// Cylinder (r=0.5, h=4) centred in a 4×4×2 box, protruding above and below:
// the intersection is exactly the πr²·2 plug inside the box.

fn family_box_cylinder() -> Outcome {
    family_box_cylinder_with(boolean)
}

fn family_box_cylinder_with(
    mut engine: impl FnMut(
        &mut Topology,
        BooleanOp,
        SolidId,
        SolidId,
    ) -> Result<SolidId, OperationsError>,
) -> Outcome {
    let what = String::from("box-cylinder plug");
    let mut topo = Topology::new();
    let (stock, v_stock) = stock_box(&mut topo, 4.0, 4.0, 2.0);
    let (tool, v_tool) = tool_cylinder_at(&mut topo, 0.5, 4.0, 2.0, 2.0, -1.0);
    if pin_operands(&topo, stock, v_stock, tool, v_tool, &what).is_err() {
        return Outcome::Incorrect;
    }
    let plug_expected = PI * 0.5 * 0.5 * 2.0;

    let mut run = |op| -> Result<SolidId, OperationsError> {
        let (x, _) = stock_box(&mut topo, 4.0, 4.0, 2.0);
        let (y, _) = tool_cylinder_at(&mut topo, 0.5, 4.0, 2.0, 2.0, -1.0);
        engine(&mut topo, op, x, y)
    };
    let (rf, ri, rc) = (
        run(BooleanOp::Fuse),
        run(BooleanOp::Intersect),
        run(BooleanOp::Cut),
    );
    // Independent-first: validate every Ok result before classifying
    // refusals, so a permitted refusal never masks an incorrect success.
    let mut vols: [Option<f64>; 3] = [None, None, None];
    for (r, tag, slot) in [(&rf, "fuse", 0), (&ri, "intersect", 1), (&rc, "cut", 2)] {
        if let Ok(s) = r {
            if let Err(e) = check_valid_closed_oriented(&topo, *s, &format!("{what} {tag}")) {
                println!("B26CASE {e}");
                return Outcome::Incorrect;
            }
            if let Err(e) = check_watertight_mesh(&topo, *s, &format!("{what} {tag}")) {
                println!("B26CASE {e}");
                return Outcome::Incorrect;
            }
            let expected = [
                v_stock + v_tool - plug_expected,
                plug_expected,
                v_stock - plug_expected,
            ][slot];
            if rel_err(vol(&topo, *s), expected) > REL_SLACK {
                println!("B26CASE {what} {tag}: independent volume oracle failed");
                return Outcome::Incorrect;
            }
            vols[slot] = Some(vol(&topo, *s));
        }
    }
    let mut ok_count = 0;
    for r in [&rf, &ri, &rc] {
        match check_paired_error(r, &what) {
            Ok(true) => ok_count += 1,
            Ok(false) => {}
            Err(e) => {
                println!("B26CASE {e}");
                return Outcome::Incorrect;
            }
        }
    }
    if ok_count < 3 {
        return Outcome::TypedRefusal;
    }
    let [Some(vf), Some(vi), Some(vc)] = vols else {
        println!("B26CASE {what}: internal harness error: missing volumes");
        return Outcome::Incorrect;
    };

    // Closed-form oracles: plug, drilled box, union by inclusion–exclusion.
    if rel_err(vi, plug_expected) > REL_SLACK {
        println!("B26CASE {what}: intersect {vi:.9} != plug {plug_expected:.9}");
        return Outcome::Incorrect;
    }
    if rel_err(vc, v_stock - plug_expected) > REL_SLACK {
        println!(
            "B26CASE {what}: cut {vc:.9} != stock-plug {:.9}",
            v_stock - plug_expected
        );
        return Outcome::Incorrect;
    }
    if rel_err(vf, v_stock + v_tool - plug_expected) > REL_SLACK {
        println!(
            "B26CASE {what}: fuse {vf:.9} != stock+tool-plug {:.9}",
            v_stock + v_tool - plug_expected
        );
        return Outcome::Incorrect;
    }
    Outcome::ExactOk
}

// ── Family 3: cavity solids (shelled stock, inner shells included) ────

fn family_cavity(offset: f64) -> Outcome {
    let what = format!("cavity offset={offset}");
    let mut topo = Topology::new();
    let stock = make_box(&mut topo, 4.0, 4.0, 4.0).expect("valid box");
    let hollow = match shell(&mut topo, stock, 0.5, &[]) {
        Ok(s) => s,
        Err(e) => {
            if classify_refusal(&e) {
                return Outcome::TypedRefusal;
            }
            println!("B26CASE {what}: untyped shell error: {e:?}");
            return Outcome::Incorrect;
        }
    };
    // Closed-form cavity oracle: outer minus inner void.
    let v_outer = closed_box(4.0, 4.0, 4.0);
    let v_void = closed_box(3.0, 3.0, 3.0);
    let v_hollow = v_outer - v_void;
    let measured = vol(&topo, hollow);
    if rel_err(measured, v_hollow) > REL_SLACK {
        println!("B26CASE {what}: hollow {measured:.9} != closed form {v_hollow:.9}");
        return Outcome::Incorrect;
    }
    // Cavity shells must be visited: face count covers outer + inner.
    let faces = solid_faces(&topo, hollow).expect("faces readable").len();
    if faces < 12 {
        println!(
            "B26CASE {what}: hollow has {faces} faces (< 12 outer+inner); cavity shells missed"
        );
        return Outcome::Incorrect;
    }
    if let Err(e) = check_valid_closed_oriented(&topo, hollow, &what) {
        println!("B26CASE {e}");
        return Outcome::Incorrect;
    }

    // Cut a box tool out of the hollow stock; partition identity:
    // V(cut) + V(inter) = V(hollow) over the closed form.
    let mut run = |op| -> Result<SolidId, OperationsError> {
        let st = make_box(&mut topo, 4.0, 4.0, 4.0).expect("valid box");
        let ho = shell(&mut topo, st, 0.5, &[])?;
        let (tool, _) = tool_box_at(&mut topo, 2.0, 2.0, 6.0, offset, 1.0, -1.0);
        boolean(&mut topo, op, ho, tool)
    };
    let (rc, ri) = (run(BooleanOp::Cut), run(BooleanOp::Intersect));
    // Independent-first: judge every Ok result before classifying refusals.
    let mut vols: [Option<f64>; 2] = [None, None];
    for (r, tag, slot) in [(&rc, "cut", 0), (&ri, "intersect", 1)] {
        if let Ok(s) = r {
            if let Err(e) = check_valid_closed_oriented(&topo, *s, &format!("{what} {tag}")) {
                println!("B26CASE {e}");
                return Outcome::Incorrect;
            }
            if let Err(e) = check_watertight_mesh(&topo, *s, &format!("{what} {tag}")) {
                println!("B26CASE {e}");
                return Outcome::Incorrect;
            }
            let outer_overlap = ((offset + 2.0).min(4.0) - offset.max(0.0)).max(0.0) * 2.0 * 4.0;
            let inner_overlap = ((offset + 2.0).min(3.5) - offset.max(0.5)).max(0.0) * 2.0 * 3.0;
            let overlap = outer_overlap - inner_overlap;
            let expected = [v_hollow - overlap, overlap][slot];
            if rel_err(vol(&topo, *s), expected) > REL_SLACK {
                println!("B26CASE {what} {tag}: independent volume oracle failed");
                return Outcome::Incorrect;
            }
            vols[slot] = Some(vol(&topo, *s));
        }
    }
    let mut ok_count = 0;
    for r in [&rc, &ri] {
        match check_paired_error(r, &what) {
            Ok(true) => ok_count += 1,
            Ok(false) => {}
            Err(e) => {
                println!("B26CASE {e}");
                return Outcome::Incorrect;
            }
        }
    }
    if ok_count < 2 {
        return Outcome::TypedRefusal;
    }
    let [Some(vc), Some(vi)] = vols else {
        println!("B26CASE {what}: internal harness error: missing volumes");
        return Outcome::Incorrect;
    };
    if rel_err(vc + vi, v_hollow) > REL_SLACK {
        println!("B26CASE {what}: cavity partition fails");
        return Outcome::Incorrect;
    }
    Outcome::ExactOk
}

// ── Family 4: rigid transforms — rotation + translation invariance ────

fn family_rigid(offset: f64, angle: f64) -> Outcome {
    let what = format!("rigid offset={offset} angle={angle:.3}");
    let mut topo = Topology::new();
    let (a, va) = stock_box(&mut topo, 2.0, 1.0, 1.0);
    let mut tool_topo = Topology::new();
    let (b0, vb) = stock_box(&mut tool_topo, 2.0, 1.0, 1.0);
    // Rotate the tool about z, then translate: still a rigid motion, so the
    // closed-form volume is untouched.
    let m = Mat4::translation(offset, 0.5, 0.0) * Mat4::rotation_z(angle);
    if remus_operations::transform::transform_solid(&mut tool_topo, b0, &m).is_err() {
        return Outcome::TypedRefusal;
    }
    // Move both operands into one arena via rebuild (transforms, not kernel
    // measurements, carry the placement — the oracle stays independent).
    let mut both = Topology::new();
    let (x, _) = stock_box(&mut both, 2.0, 1.0, 1.0);
    let (y, _) = stock_box(&mut both, 2.0, 1.0, 1.0);
    if remus_operations::transform::transform_solid(&mut both, y, &m).is_err() {
        return Outcome::TypedRefusal;
    }
    let _ = (topo, tool_topo, a, b0, va, vb);
    if pin_operands(
        &both,
        x,
        closed_box(2.0, 1.0, 1.0),
        y,
        closed_box(2.0, 1.0, 1.0),
        &what,
    )
    .is_err()
    {
        return Outcome::Incorrect;
    }

    let mut run = |op| -> Result<SolidId, OperationsError> {
        let (p, _) = stock_box(&mut both, 2.0, 1.0, 1.0);
        let (q, _) = stock_box(&mut both, 2.0, 1.0, 1.0);
        remus_operations::transform::transform_solid(&mut both, q, &m)
            .map_err(|_| OperationsError::NonManifoldResult)?;
        boolean(&mut both, op, p, q)
    };
    let (rf, ri) = (run(BooleanOp::Fuse), run(BooleanOp::Intersect));
    // Independent-first: judge every Ok result before classifying refusals.
    let mut vols: [Option<f64>; 2] = [None, None];
    for (r, tag, slot) in [(&rf, "fuse", 0), (&ri, "intersect", 1)] {
        if let Ok(s) = r {
            if let Err(e) = check_valid_closed_oriented(&both, *s, &format!("{what} {tag}")) {
                println!("B26CASE {e}");
                return Outcome::Incorrect;
            }
            if let Err(e) = check_watertight_mesh(&both, *s, &format!("{what} {tag}")) {
                println!("B26CASE {e}");
                return Outcome::Incorrect;
            }
            vols[slot] = Some(vol(&both, *s));
        }
    }
    let mut ok_count = 0;
    for r in [&rf, &ri] {
        match check_paired_error(r, &what) {
            Ok(true) => ok_count += 1,
            Ok(false) => {}
            Err(e) => {
                println!("B26CASE {e}");
                return Outcome::Incorrect;
            }
        }
    }
    if ok_count < 2 {
        return Outcome::TypedRefusal;
    }
    let [Some(vf), Some(vi)] = vols else {
        println!("B26CASE {what}: internal harness error: missing volumes");
        return Outcome::Incorrect;
    };
    let va2 = closed_box(2.0, 1.0, 1.0);
    if rel_err(vf + vi, 2.0 * va2) > REL_SLACK {
        println!("B26CASE {what}: rigid inclusion-exclusion fails");
        return Outcome::Incorrect;
    }
    Outcome::ExactOk
}

// ── Family 5: scale variations — identities at 1e-3 / 1 / 1e3 ─────────

fn family_scale(offset: f64, scale: f64) -> Outcome {
    let what = format!("scale={scale} offset={offset}");
    let mut topo = Topology::new();
    let (a, va) = stock_box(&mut topo, 2.0 * scale, 2.0 * scale, 2.0 * scale);
    let (b, vb) = tool_box_at(
        &mut topo,
        2.0 * scale,
        2.0 * scale,
        2.0 * scale,
        offset * scale,
        0.0,
        0.0,
    );
    if pin_operands(&topo, a, va, b, vb, &what).is_err() {
        return Outcome::Incorrect;
    }
    let mut run = |op| -> Result<SolidId, OperationsError> {
        let (x, _) = stock_box(&mut topo, 2.0 * scale, 2.0 * scale, 2.0 * scale);
        let (y, _) = tool_box_at(
            &mut topo,
            2.0 * scale,
            2.0 * scale,
            2.0 * scale,
            offset * scale,
            0.0,
            0.0,
        );
        boolean(&mut topo, op, x, y)
    };
    let (rf, ri) = (run(BooleanOp::Fuse), run(BooleanOp::Intersect));
    // Independent-first: judge every Ok result before classifying refusals.
    let mut vols: [Option<f64>; 2] = [None, None];
    for (r, tag, slot) in [(&rf, "fuse", 0), (&ri, "intersect", 1)] {
        if let Ok(s) = r {
            if let Err(e) = check_valid_closed_oriented(&topo, *s, &format!("{what} {tag}")) {
                println!("B26CASE {e}");
                return Outcome::Incorrect;
            }
            let overlap = (2.0 - offset.abs()).max(0.0) * 4.0 * scale.powi(3);
            let expected = [va + vb - overlap, overlap][slot];
            if rel_err(vol(&topo, *s), expected) > REL_SLACK {
                println!("B26CASE {what} {tag}: independent volume oracle failed");
                return Outcome::Incorrect;
            }
            vols[slot] = Some(vol(&topo, *s));
        }
    }
    let mut ok_count = 0;
    for r in [&rf, &ri] {
        match check_paired_error(r, &what) {
            Ok(true) => ok_count += 1,
            Ok(false) => {}
            Err(e) => {
                println!("B26CASE {e}");
                return Outcome::Incorrect;
            }
        }
    }
    if ok_count < 2 {
        return Outcome::TypedRefusal;
    }
    let [Some(vf), Some(vi)] = vols else {
        println!("B26CASE {what}: internal harness error: missing volumes");
        return Outcome::Incorrect;
    };
    if rel_err(vf + vi, va + vb) > REL_SLACK {
        println!("B26CASE {what}: scaled inclusion-exclusion fails");
        return Outcome::Incorrect;
    }
    Outcome::ExactOk
}

// ── Family 6: sphere pairs — curved closed-form oracles ───────────────
//
// Cost control (measured 2026-09-12, 8-segment r=1 spheres): intersect takes
// ~1–2 s per call, but cut takes ~36–66 s (the cut cap reassembly path is
// far more expensive than the lens intersection). So CI covers intersect
// only — one call per offset, ~4 s total for three offsets — and checks the
// lens oracle plus topology/mesh on that result. Cut-complement and fuse
// coverage for curved pairs live in the opt-in campaign, not CI.

fn family_spheres(offset: f64) -> Outcome {
    let what = format!("spheres offset={offset}");
    let mut topo = Topology::new();
    let r = 1.0;
    const SEG: usize = 8;
    let (a, va) = {
        let s = make_sphere(&mut topo, r, SEG).expect("valid sphere");
        (s, closed_sphere(r))
    };
    let (b, vb) = tool_sphere_at_seg(&mut topo, r, SEG, offset, 0.0, 0.0);
    if pin_operands(&topo, a, va, b, vb, &what).is_err() {
        return Outcome::Incorrect;
    }
    let mut run = |op| -> Result<SolidId, OperationsError> {
        let x = make_sphere(&mut topo, r, SEG).expect("valid sphere");
        let (y, _) = tool_sphere_at_seg(&mut topo, r, SEG, offset, 0.0, 0.0);
        boolean(&mut topo, op, x, y)
    };
    // Intersect only: the lens oracle pins it directly. Cut and fuse need
    // 40–80 s per call on this path and live in the opt-in campaign.
    let inter = match run(BooleanOp::Intersect) {
        Ok(i) => i,
        Err(e) => {
            if classify_refusal(&e) {
                return Outcome::TypedRefusal;
            }
            println!("B26CASE {what}: untyped boolean error: {e:?}");
            return Outcome::Incorrect;
        }
    };
    // Lens closed form for two equal spheres distance d apart (0<d<2r).
    let d = offset;
    if d > 0.0 && d < 2.0 * r {
        let Some(lens) = closed_lens_equal_spheres(r, d) else {
            println!("B26CASE {what}: lens oracle out of domain");
            return Outcome::Incorrect;
        };
        let vi = vol(&topo, inter);
        if rel_err(vi, lens) > REL_SLACK {
            println!("B26CASE {what}: lens {vi:.9} != closed form {lens:.9}");
            return Outcome::Incorrect;
        }
    } else if rel_err(vol(&topo, inter), 0.0) > REL_SLACK && vb > 0.0 {
        println!("B26CASE {what}: disjoint sphere intersection should be empty");
        return Outcome::Incorrect;
    }
    for (s, tag) in [(inter, "intersect")] {
        if let Err(e) = check_valid_closed_oriented(&topo, s, &format!("{what} {tag}")) {
            println!("B26CASE {e}");
            return Outcome::Incorrect;
        }
        if let Err(e) = check_watertight_mesh(&topo, s, &format!("{what} {tag}")) {
            println!("B26CASE {e}");
            return Outcome::Incorrect;
        }
    }
    Outcome::ExactOk
}

// ── Family 7: exact-only nudge stability ──────────────────────────────

fn nudge_vertices(topo: &mut Topology, solid: SolidId) -> bool {
    let verts = match explorer::solid_vertices(topo, solid) {
        Ok(v) => v,
        Err(_) => return false,
    };
    for vid in verts {
        let Ok(v) = topo.vertex(vid) else {
            return false;
        };
        let p = v.point();
        let Ok(vm) = topo.vertex_mut(vid) else {
            return false;
        };
        vm.set_point(remus_math::vec::Point3::new(p.x() + NUDGE, p.y(), p.z()));
    }
    true
}

fn family_nudge(offset: f64) -> Outcome {
    let what = format!("nudge offset={offset}");
    let mut t0 = Topology::new();
    let (a0, _) = stock_box(&mut t0, 2.0, 2.0, 2.0);
    let (b0, _) = tool_box_at(&mut t0, 2.0, 2.0, 2.0, offset, 0.0, 0.0);
    let mut t1 = t0.clone();
    // Nudge one operand's copy by 1e-13 along +x.
    if !nudge_vertices(&mut t1, b0) {
        return Outcome::TypedRefusal;
    }
    let base = boolean(&mut t0, BooleanOp::Fuse, a0, b0);
    // t1 holds the same two boxes with b0 nudged; fuse a-fresh against the
    // nudged b0. (A fresh `a` is used rather than t1's original so the only
    // difference between the two runs is the 1e-13 nudge.)
    let nudged = {
        let aa = make_box(&mut t1, 2.0, 2.0, 2.0).expect("valid box");
        boolean(&mut t1, BooleanOp::Fuse, aa, b0)
    };
    match (base, nudged) {
        (Err(e0), Err(e1)) => {
            if classify_refusal(&e0) && classify_refusal(&e1) {
                Outcome::TypedRefusal
            } else {
                println!("B26CASE {what}: untyped nudge error(s): {e0:?} / {e1:?}");
                Outcome::Incorrect
            }
        }
        (Ok(s0), Ok(s1)) => {
            // Independent-first: judge each side's topology before comparing
            // volumes, so a bad base result cannot hide behind a matching
            // nudged one (or vice versa).
            if check_valid_closed_oriented(&t0, s0, &format!("{what} base")).is_err()
                || check_valid_closed_oriented(&t1, s1, &format!("{what} nudged")).is_err()
            {
                println!("B26CASE {what}: nudge result topology invalid");
                return Outcome::Incorrect;
            }
            let v0 = vol(&t0, s0);
            let v1 = vol(&t1, s1);
            if rel_err(v0, v1) > REL_SLACK {
                println!("B26CASE {what}: nudge moved volume {v0:.9} -> {v1:.9}");
                return Outcome::Incorrect;
            }
            Outcome::ExactOk
        }
        (Ok(s0), Err(e1)) | (Err(e1), Ok(s0)) => {
            // The successful side is still judged on its own: an incorrect
            // success paired with a refusal must not pass as TypedRefusal.
            if check_valid_closed_oriented(&t0, s0, &format!("{what} success-side")).is_err() {
                println!("B26CASE {what}: success side of nudge pair is topologically invalid");
                return Outcome::Incorrect;
            }
            if !classify_refusal(&e1) {
                println!("B26CASE {what}: untyped nudge error: {e1:?}");
                return Outcome::Incorrect;
            }
            let v0 = vol(&t0, s0);
            println!(
                "B26CASE {what}: 1e-13 nudge flipped outcome kind (volume was {v0:.9}): {e1:?}"
            );
            Outcome::Incorrect
        }
    }
}

// ── Runners ──────────────────────────────────────────────────────────

fn run_ci_matrix() {
    let mut t = Tally::default();
    for o in CI_OFFSETS {
        t.record(family_box_pair(o), format!("family=box-pair offset={o}"));
    }
    t.finish("box-pair");

    let mut t = Tally::default();
    t.record(family_box_cylinder(), "family=box-cylinder".to_string());
    t.finish("box-cylinder");

    let mut t = Tally::default();
    for o in CI_OFFSETS {
        t.record(family_cavity(o), format!("family=cavity offset={o}"));
    }
    t.finish("cavity");

    let mut t = Tally::default();
    for (o, a) in [
        (0.5, 0.0),
        (0.5, 0.785_398_163_397_448_3),
        (1.5, 0.0),
        (1.5, 1.570_796_326_794_896_6),
    ] {
        t.record(
            family_rigid(o, a),
            format!("family=rigid offset={o} angle={a}"),
        );
    }
    t.finish("rigid");

    let mut t = Tally::default();
    for s in [1e-3, 1.0, 1e3] {
        for o in [0.5, 1.5] {
            t.record(family_scale(o, s), format!("family=scale s={s} offset={o}"));
        }
    }
    t.finish("scale");

    let mut t = Tally::default();
    for o in [0.5, 1.0, 1.5] {
        t.record(family_spheres(o), format!("family=spheres offset={o}"));
    }
    t.finish("spheres");

    let mut t = Tally::default();
    for o in CI_OFFSETS {
        t.record(family_nudge(o), format!("family=nudge offset={o}"));
    }
    t.finish("nudge");
}

/// Opt-in campaign: generated primitive-pair cases from `seed`.
///
/// The case kind rotates deterministically by index: box-pair (overlap,
/// inclusion–exclusion + topology + mesh), box×cylinder plug (closed-form
/// intersection), box×rotated-cylinder cut (typed-refusal-or-correct, no
/// volume oracle past the plug family — oblique quadric intersections have
/// no hand oracle), sphere×box cut (volume-bounded + topology), and
/// cone/torus operand pins (closed-form operand check only; torus×box
/// booleans stay out of CI scope — tangent-torus is B9's machinery).
/// Dimensions sit on a half-unit lattice in [1,4]; offsets in [-1,3]; the
/// lattice keeps face-coincidence and tangency the common case, not the
/// rarity. Curved cases use 8-segment spheres (volume-identical, faster).
fn run_campaign(n: usize, seed: u64) {
    let mut rng = Stream(seed);
    // Per-family tallies: a shared tally would let abundant box successes
    // mask universal refusal in a curved family. Each family finishes (and
    // enforces its own non-vacuity gate) independently; the merged tally is
    // reporting only.
    let mut box_tally = Tally::default();
    let mut plug_tally = Tally::default();
    let mut oblique_tally = Tally::default();
    let mut sphere_tally = Tally::default();
    for i in 0..n {
        match i % 5 {
            0 | 1 => campaign_box_case(&mut rng, &mut box_tally, seed, i),
            2 => campaign_cylinder_plug_case(&mut rng, &mut plug_tally, seed, i),
            3 => campaign_oblique_cylinder_case(&mut rng, &mut oblique_tally, seed, i),
            _ => campaign_sphere_cut_case(&mut rng, &mut sphere_tally, seed, i),
        }
    }
    box_tally.finish("campaign-box");
    plug_tally.finish("campaign-plug");
    oblique_tally.finish("campaign-oblique");
    sphere_tally.finish("campaign-sphere-cut");
    let mut merged = Tally::default();
    merged.merge(&box_tally);
    merged.merge(&plug_tally);
    merged.merge(&oblique_tally);
    merged.merge(&sphere_tally);
    println!(
        "B26TALLY family=campaign-merged exact_ok={} typed_refusal={} incorrect={}",
        merged.exact_ok, merged.typed_refusal, merged.incorrect
    );
}

fn campaign_box_case(rng: &mut Stream, tally: &mut Tally, seed: u64, i: usize) {
    let mut dim_lattice = || 1.0 + (rng.below(7) as f64) * 0.5;
    let (dx, dy, dz) = (dim_lattice(), dim_lattice(), dim_lattice());
    let (ex, ey, ez) = (
        rng.range_f64(-1.0, 3.0, 8),
        rng.range_f64(-1.0, 3.0, 8),
        rng.range_f64(-1.0, 3.0, 8),
    );
    let mut topo = Topology::new();
    let (a, va) = stock_box(&mut topo, dx, dy, dz);
    let (b, vb) = tool_box_at(&mut topo, dx, dy, dz, ex, ey, ez);
    if pin_operands(&topo, a, va, b, vb, &format!("campaign[{i}]")).is_err() {
        tally.record(
            Outcome::Incorrect,
            format!("seed={seed} index={i} oracle-broken"),
        );
        return;
    }
    let mut run = |op| -> Result<SolidId, OperationsError> {
        let (x, _) = stock_box(&mut topo, dx, dy, dz);
        let (y, _) = tool_box_at(&mut topo, dx, dy, dz, ex, ey, ez);
        boolean(&mut topo, op, x, y)
    };
    let desc = format!(
        "seed={seed} index={i} family=campaign-box dims=({dx},{dy},{dz}) offset=({ex},{ey},{ez})"
    );
    let (rf, rn) = (run(BooleanOp::Fuse), run(BooleanOp::Intersect));
    let overlap = (dx - ex.abs()).max(0.0) * (dy - ey.abs()).max(0.0) * (dz - ez.abs()).max(0.0);
    for (result, expected) in [(&rf, va + vb - overlap), (&rn, overlap)] {
        if let Ok(solid) = result
            && rel_err(vol(&topo, *solid), expected) > REL_SLACK
        {
            tally.record(Outcome::Incorrect, format!("{desc} independent volume"));
            return;
        }
    }
    // Independent-first: judge every Ok result before classifying refusals.
    // A permitted refusal on intersect must not mask an incorrect fuse, and
    // an empty intersection is judged by volume (~0) rather than validated.
    // The fuse check stages an optional failure (staged first so the
    // shrunk description moves exactly once); the inter check returns
    // directly since no staged value is live there.
    let fuse_failure: Option<String> = if let Ok(f) = &rf {
        let inter_empty = rn.as_ref().ok().is_some_and(|n| {
            expected_empty_interpretation(&topo, *n, "campaign inter").unwrap_or(false)
        });
        if inter_empty {
            if rel_err(vol(&topo, *f), va + vb) > REL_SLACK {
                println!("B26CASE {desc} (disjoint fuse volume)");
                Some(shrink_box_case(dx, dy, dz, ex, ey, ez, seed, i))
            } else {
                None
            }
        } else if let Ok(n) = &rn {
            if rel_err(vol(&topo, *f) + vol(&topo, *n), va + vb) > REL_SLACK {
                println!("B26CASE {desc} (inclusion-exclusion)");
                Some(shrink_box_case(dx, dy, dz, ex, ey, ez, seed, i))
            } else {
                None
            }
        } else {
            None
        }
        .or_else(|| {
            if let Err(e) = check_valid_closed_oriented(&topo, *f, "campaign fuse") {
                println!("B26CASE {desc} (fuse topology: {e})");
                Some(shrink_box_case(dx, dy, dz, ex, ey, ez, seed, i))
            } else {
                None
            }
        })
        .or_else(|| {
            if let Err(e) = check_watertight_mesh(&topo, *f, "campaign fuse") {
                println!("B26CASE {desc} (fuse mesh: {e})");
                Some(shrink_box_case(dx, dy, dz, ex, ey, ez, seed, i))
            } else {
                None
            }
        })
    } else {
        None
    };
    if let Some(failed) = fuse_failure {
        tally.record(Outcome::Incorrect, failed);
        return;
    }
    if let Ok(n) = &rn {
        if expected_empty_interpretation(&topo, *n, "campaign inter").unwrap_or(false) {
            // Empty set: nothing to validate; the fuse-side volume check
            // above already judged the disjoint identity.
        } else {
            if let Err(e) = check_valid_closed_oriented(&topo, *n, "campaign inter") {
                println!("B26CASE {desc} (inter topology: {e})");
                tally.record(
                    Outcome::Incorrect,
                    shrink_box_case(dx, dy, dz, ex, ey, ez, seed, i),
                );
                return;
            }
            if let Err(e) = check_watertight_mesh(&topo, *n, "campaign inter") {
                println!("B26CASE {desc} (inter mesh: {e})");
                tally.record(
                    Outcome::Incorrect,
                    shrink_box_case(dx, dy, dz, ex, ey, ez, seed, i),
                );
                return;
            }
        }
    }
    // Classify refusals only after all successes have been judged.
    let mut ok_count = 0;
    let mut untyped = false;
    for r in [&rf, &rn] {
        match r {
            Ok(_) => ok_count += 1,
            Err(e) => {
                if !classify_refusal(e) {
                    untyped = true;
                }
            }
        }
    }
    if untyped {
        println!("B26CASE {desc} (untyped error)");
        tally.record(Outcome::Incorrect, desc);
    } else if ok_count < 2 {
        tally.record(Outcome::TypedRefusal, desc);
    } else {
        tally.record(Outcome::ExactOk, desc);
    }
}

/// Shrink a failing box-pair case: walk each offset component toward 0 on
/// the lattice grid, then each dimension toward its minimum, keeping the
/// first parameter set that still fails inclusion–exclusion or topology.
/// Returns a minimized description with a deterministic replay body.
fn shrink_box_case(
    dx: f64,
    dy: f64,
    dz: f64,
    ex: f64,
    ey: f64,
    ez: f64,
    seed: u64,
    i: usize,
) -> String {
    let fails = |dx: f64, dy: f64, dz: f64, ex: f64, ey: f64, ez: f64| -> bool {
        let mut topo = Topology::new();
        let mk = |topo: &mut Topology| {
            let x = make_box(topo, dx, dy, dz).unwrap();
            let y = make_box(topo, dx, dy, dz).unwrap();
            remus_operations::transform::transform_solid(topo, y, &Mat4::translation(ex, ey, ez))
                .unwrap();
            (x, y)
        };
        let (x, y) = mk(&mut topo);
        let (x2, y2) = mk(&mut topo);
        match (
            boolean(&mut topo, BooleanOp::Fuse, x, y),
            boolean(&mut topo, BooleanOp::Intersect, x2, y2),
        ) {
            (Ok(f), Ok(n)) => {
                let va = closed_box(dx, dy, dz);
                if rel_err(vol(&topo, f) + vol(&topo, n), 2.0 * va) > REL_SLACK {
                    return true;
                }
                check_valid_closed_oriented(&topo, f, "shrink").is_err()
                    || check_valid_closed_oriented(&topo, n, "shrink").is_err()
            }
            (f, n) => [&f, &n].iter().any(|r| match r {
                Err(e) => !classify_refusal(e),
                Ok(_) => false,
            }),
        }
    };
    let (mut ex, mut ey, mut ez) = (ex, ey, ez);
    for axis in 0..3 {
        loop {
            let cur = [ex, ey, ez][axis];
            if cur == 0.0 {
                break;
            }
            let next = (cur - 0.5 * cur.signum()).clamp(-4.0, 4.0);
            let (tx, ty, tz) = match axis {
                0 => (next, ey, ez),
                1 => (ex, next, ez),
                _ => (ex, ey, next),
            };
            if fails(dx, dy, dz, tx, ty, tz) {
                (ex, ey, ez) = (tx, ty, tz);
            } else {
                break;
            }
        }
    }
    let minimized = format!(
        "seed={seed} index={i} family=campaign-box SHRUNK dims=({dx},{dy},{dz}) offset=({ex},{ey},{ez})"
    );
    println!("B26SHRINK {minimized}");
    minimized
}

fn campaign_cylinder_plug_case(rng: &mut Stream, tally: &mut Tally, seed: u64, i: usize) {
    let r = 0.5 + (rng.below(4) as f64) * 0.25;
    let mut topo = Topology::new();
    let (stock, v_stock) = stock_box(&mut topo, 4.0, 4.0, 2.0);
    let (tool, v_tool) = tool_cylinder_at(&mut topo, r, 4.0, 2.0, 2.0, -1.0);
    let desc = format!("seed={seed} index={i} family=campaign-plug r={r}");
    if pin_operands(&topo, stock, v_stock, tool, v_tool, &desc).is_err() {
        tally.record(Outcome::Incorrect, format!("{desc} oracle-broken"));
        return;
    }
    let mut run = |op| -> Result<SolidId, OperationsError> {
        let (x, _) = stock_box(&mut topo, 4.0, 4.0, 2.0);
        let (y, _) = tool_cylinder_at(&mut topo, r, 4.0, 2.0, 2.0, -1.0);
        boolean(&mut topo, op, x, y)
    };
    let plug = PI * r * r * 2.0;
    let (rc, rn) = (run(BooleanOp::Cut), run(BooleanOp::Intersect));
    // Independent-first: judge every Ok result before classifying refusals.
    let mut failure: Option<&'static str> = None;
    if let Ok(c) = &rc {
        if rel_err(vol(&topo, *c), v_stock - plug) > REL_SLACK {
            failure = Some("cut volume");
        } else if let Err(e) = check_valid_closed_oriented(&topo, *c, "campaign plug cut") {
            println!("B26CASE {desc} (cut topology: {e})");
            failure = Some("cut topology");
        }
    }
    if failure.is_none()
        && let Ok(n) = &rn
    {
        if rel_err(vol(&topo, *n), plug) > REL_SLACK {
            failure = Some("inter volume");
        } else if let Err(e) = check_valid_closed_oriented(&topo, *n, "campaign plug inter") {
            println!("B26CASE {desc} (inter topology: {e})");
            failure = Some("inter topology");
        }
    }
    if let Some(reason) = failure {
        println!("B26CASE {desc} ({reason})");
        tally.record(Outcome::Incorrect, desc);
        return;
    }
    let mut ok_count = 0;
    let mut untyped = false;
    for r in [&rc, &rn] {
        match r {
            Ok(_) => ok_count += 1,
            Err(e) => {
                if !classify_refusal(e) {
                    untyped = true;
                }
            }
        }
    }
    if untyped {
        println!("B26CASE {desc} (untyped error)");
        tally.record(Outcome::Incorrect, desc);
    } else if ok_count < 2 {
        tally.record(Outcome::TypedRefusal, desc);
    } else {
        tally.record(Outcome::ExactOk, desc);
    }
}

fn campaign_oblique_cylinder_case(rng: &mut Stream, tally: &mut Tally, seed: u64, i: usize) {
    // Oblique quadric intersections have no hand oracle: the check is
    // exact-ok-with-valid-topology-and-mesh, or typed refusal. A success
    // with invalid topology/mesh is the finding.
    let tilt = (rng.below(4) as f64) * 0.15;
    let mut topo = Topology::new();
    let desc = format!("seed={seed} index={i} family=campaign-oblique tilt={tilt:.2}");
    let cut = (|| -> Result<SolidId, OperationsError> {
        let s = make_box(&mut topo, 4.0, 4.0, 2.0)?;
        let c = make_cylinder(&mut topo, 0.5, 4.0)?;
        remus_operations::transform::transform_solid(
            &mut topo,
            c,
            &(Mat4::translation(2.0, 2.0, -1.0) * Mat4::rotation_x(tilt)),
        )
        .map_err(|_| OperationsError::NonManifoldResult)?;
        boolean(&mut topo, BooleanOp::Cut, s, c)
    })();
    match cut {
        Ok(s) => {
            if check_valid_closed_oriented(&topo, s, "campaign oblique").is_err()
                || check_watertight_mesh(&topo, s, "campaign oblique").is_err()
            {
                println!("B26CASE {desc}");
                tally.record(Outcome::Incorrect, desc);
            } else {
                tally.record(Outcome::ExactOk, desc);
            }
        }
        Err(e) => {
            if classify_refusal(&e) {
                tally.record(Outcome::TypedRefusal, desc);
            } else {
                tally.record(Outcome::Incorrect, desc);
            }
        }
    }
}

fn campaign_sphere_cut_case(rng: &mut Stream, tally: &mut Tally, seed: u64, i: usize) {
    let off = rng.range_f64(1.0, 3.0, 8);
    let mut topo = Topology::new();
    let desc = format!("seed={seed} index={i} family=campaign-sphere-cut off={off}");
    let cut = (|| -> Result<SolidId, OperationsError> {
        let s = make_box(&mut topo, 4.0, 4.0, 4.0)?;
        let b = make_sphere(&mut topo, 1.5, 8)?;
        remus_operations::transform::transform_solid(
            &mut topo,
            b,
            &Mat4::translation(off, 2.0, 2.0),
        )
        .map_err(|_| OperationsError::NonManifoldResult)?;
        boolean(&mut topo, BooleanOp::Cut, s, b)
    })();
    match cut {
        Ok(s) => {
            // Volume bound oracle (independent of the boolean path): the cut
            // removes at most the whole sphere and never adds material.
            let v = vol(&topo, s);
            let v_stock = closed_box(4.0, 4.0, 4.0);
            let v_tool = closed_sphere(1.5);
            let bad_vol = v > v_stock * (1.0 + REL_SLACK) || v < v_stock - v_tool - VOL_FLOOR;
            if bad_vol
                || check_valid_closed_oriented(&topo, s, "campaign sphere cut").is_err()
                || check_watertight_mesh(&topo, s, "campaign sphere cut").is_err()
            {
                println!("B26CASE {desc}");
                tally.record(Outcome::Incorrect, desc);
            } else {
                tally.record(Outcome::ExactOk, desc);
            }
        }
        Err(e) => {
            if classify_refusal(&e) {
                tally.record(Outcome::TypedRefusal, desc);
            } else {
                tally.record(Outcome::Incorrect, desc);
            }
        }
    }
}

#[test]
fn b26_ci_matrix() {
    run_ci_matrix();
}

#[test]
fn b26_opt_in_campaign() {
    if let Some((n, seed)) = campaign_size() {
        run_campaign(n, seed);
    }
}

// ── Pinned minimized regressions ─────────────────────────────────────
// (Failures minimized from campaign runs land here as ordinary tests.
//  None yet — the campaign has not run.)

#[test]
fn wrong_success_is_not_hidden_by_sibling_refusals() {
    let outcome = family_box_cylinder_with(|_, op, stock, _| {
        if matches!(op, BooleanOp::Fuse) {
            Ok(stock)
        } else {
            Err(OperationsError::ExactOnlyUnattainable)
        }
    });
    assert!(matches!(outcome, Outcome::Incorrect));
}

#[test]
fn non_finite_volumes_fail_closed_form_oracles() {
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(rel_err(value, 32.0) > REL_SLACK);
    }
}
