//! K-S1 fail-closed contract for every public fillet/blend mutation path.
//!
//! The defect this file pins: `fillet_variable` (and the deprecated v1
//! engines when called directly) could answer with silent wrongness —
//! a variable fillet of radius 50 on a 10 mm box returned `Ok` and a solid
//! measuring 3242 mm³ (a fillet must *remove* material), the same call on a
//! cylinder's edges returned an *invalid* solid as success, and a selection
//! naming another solid's edge returned a fresh clone of the input as if the
//! blend had happened. Failures could also leave the arena carrying the
//! half-built attempt.
//!
//! The contract these tests prove on every path:
//! - success means a *different*, valid, watertight solid whose volume change
//!   is one a blend of that size can physically produce;
//! - anything else is a typed refusal (`blend_failure_code` is non-empty)
//!   that leaves the input handle, the arena counts, the topology, the face
//!   attributes, and the geometry exactly as they were.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, deprecated)]

use remus_check::validate::{ValidateOptions, validate_solid};
use remus_operations::blend_ops::blend_failure_code;
use remus_operations::chamfer::chamfer;
use remus_operations::fillet::{FilletRadiusLaw, fillet, fillet_rolling_ball, fillet_variable};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_topology::Topology;
use remus_topology::attributes::EntityAttributes;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const DEFLECTION: f64 = 0.01;

/// Every live-entity counter plus the journal length: the complete observable
/// state a failed operation must not disturb. A handle slot handed out before
/// the call must still resolve afterwards, so counts must match exactly.
#[derive(Debug, PartialEq)]
struct ArenaCounts {
    vertices: usize,
    edges: usize,
    wires: usize,
    faces: usize,
    shells: usize,
    solids: usize,
    loops: usize,
    coedges: usize,
    pcurves: usize,
    attributes: usize,
}

fn arena_counts(topo: &Topology) -> ArenaCounts {
    ArenaCounts {
        vertices: topo.num_vertices(),
        edges: topo.num_edges(),
        wires: topo.num_wires(),
        faces: topo.num_faces(),
        shells: topo.num_shells(),
        solids: topo.num_solids(),
        loops: topo.num_loops(),
        coedges: topo.num_coedges(),
        pcurves: topo.num_pcurves(),
        attributes: topo.attributes().len(),
    }
}

/// The observable shape of a solid: counts, volume (quantised so the check
/// does not hinge on float equality), and whether it validates.
#[derive(Debug, PartialEq)]
struct Fingerprint {
    faces: usize,
    edges: usize,
    vertices: usize,
    volume_nano: i128,
    valid: bool,
}

fn fingerprint(topo: &Topology, solid: SolidId) -> Fingerprint {
    let volume = solid_volume(topo, solid, DEFLECTION).unwrap();
    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    Fingerprint {
        faces: solid_faces(topo, solid).unwrap().len(),
        edges: solid_edges(topo, solid).unwrap().len(),
        vertices: remus_topology::explorer::solid_vertices(topo, solid)
            .unwrap()
            .len(),
        #[allow(clippy::cast_possible_truncation)]
        volume_nano: (volume * 1e9).round() as i128,
        valid: report.is_valid(),
    }
}

/// Assert that `op` fails with a machine-readable blend code and that
/// afterwards the input solid, the arena, and the journal read back exactly
/// as before.
fn assert_typed_noop_failure<F>(label: &str, topo: &mut Topology, solid: SolidId, op: F)
where
    F: FnOnce(&mut Topology, SolidId) -> Result<SolidId, remus_operations::OperationsError>,
{
    let counts_before = arena_counts(topo);
    let before = fingerprint(topo, solid);
    assert!(
        before.valid,
        "{label}: the fixture itself must start valid, got {before:?}"
    );

    let error = match op(topo, solid) {
        Ok(s) => panic!(
            "{label}: expected a typed refusal, got Ok (same handle: {})",
            s == solid
        ),
        Err(error) => error,
    };
    assert!(
        !blend_failure_code(&error).is_empty(),
        "{label}: every refusal must map to a machine-readable code, got {error}"
    );

    let counts_after = arena_counts(topo);
    assert_eq!(
        counts_before, counts_after,
        "{label}: the failed operation left arena state behind"
    );
    let after = fingerprint(topo, solid);
    assert_eq!(
        before, after,
        "{label}: the failed operation changed the input solid"
    );

    // The input handle still resolves to a watertight shell.
    let shell = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    validate_shell_closed(shell, topo)
        .unwrap_or_else(|e| panic!("{label}: input shell must stay watertight: {e}"));
}

/// Closed-form material a convex fillet removes: (1−π/4)·r² per unit length.
fn convex_fillet_removed(radius: f64, length: f64) -> f64 {
    (1.0 - std::f64::consts::FRAC_PI_4) * radius * radius * length
}

// ── 1. Supported fillet: a genuinely changed, valid result ──────────

/// A variable-radius fillet that succeeds must return a *different* solid
/// whose volume dropped by the closed-form sliver, never the input handle.
#[test]
fn variable_fillet_success_is_a_different_valid_solid() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    let before = fingerprint(&topo, solid);

    let result = fillet_variable(
        &mut topo,
        solid,
        &[(edges[0], FilletRadiusLaw::Constant(1.0))],
    )
    .expect("a supported variable fillet must succeed");
    assert_ne!(
        result, solid,
        "a fillet must not answer with its input handle"
    );

    let after = fingerprint(&topo, result);
    assert!(after.valid, "the result must validate");
    assert!(
        after.faces > before.faces && after.edges > before.edges,
        "a fillet adds blend faces: {before:?} -> {after:?}"
    );

    // Volume oracle: one 10 mm convex edge at r=1 removes ≈(1−π/4)·1²·10.
    let expected = 1000.0 - convex_fillet_removed(1.0, 10.0);
    let volume = solid_volume(&topo, result, DEFLECTION).unwrap();
    assert!(
        (volume - expected).abs() < 0.5,
        "expected ≈{expected:.3} mm³ (closed form 997.854), got {volume:.3}"
    );

    let shell = topo
        .shell(topo.solid(result).unwrap().outer_shell())
        .unwrap();
    validate_shell_closed(shell, &topo).expect("the result must be watertight");

    // And the input is still there, itself unchanged.
    assert_eq!(
        fingerprint(&topo, solid),
        before,
        "a successful fillet still must not touch the input solid"
    );
}

// ── 2. Unsupported/failing fillets: typed refusal ───────────────────

/// Baseline defect: `fillet_variable` with r=50 on a 10 mm box returned `Ok`
/// and a solid measuring 3242.011 mm³ — success, with the volume *growing*.
/// Now the volume-sign oracle refuses it, and the refusal is a true no-op.
#[test]
fn variable_fillet_oversized_radius_is_a_typed_refusal_and_true_noop() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();

    assert_typed_noop_failure(
        "variable fillet r=50 on a 10mm box",
        &mut topo,
        solid,
        |t, s| fillet_variable(t, s, &[(edges[0], FilletRadiusLaw::Constant(50.0))]),
    );
}

/// Baseline defect: `fillet_variable` on a cylinder's edges returned `Ok` with
/// an *invalid* solid (458.8 mm³ from a 785.4 mm³ cylinder). The closed rim is
/// a legitimate blend (the v2 engine rounds it); the variable engine cannot —
/// it must say so by name.
#[test]
fn variable_fillet_on_cylinder_edges_is_typed_and_clean() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    let laws: Vec<_> = edges
        .iter()
        .map(|&e| (e, FilletRadiusLaw::Constant(0.5)))
        .collect();

    assert_typed_noop_failure(
        "variable fillet on every cylinder edge",
        &mut topo,
        solid,
        |t, s| fillet_variable(t, s, &laws),
    );
}

/// Baseline defect: naming another solid's edge returned a fresh clone of the
/// input (same volume, new handle) as if the blend had happened. Now the edge
/// is named in an `edges-not-blended` refusal.
#[test]
fn variable_fillet_foreign_edge_is_refused_and_leaves_no_trace() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let other = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let other_edges = solid_edges(&topo, other).unwrap();

    let counts_before = arena_counts(&topo);
    let before = fingerprint(&topo, solid);
    let error = fillet_variable(
        &mut topo,
        solid,
        &[(other_edges[0], FilletRadiusLaw::Constant(0.5))],
    )
    .expect_err("a foreign edge must be refused");
    assert!(
        matches!(
            error,
            remus_operations::OperationsError::Blend(
                remus_blend::BlendError::EdgesNotBlended { .. }
            )
        ),
        "the refusal must name the unblended edge, got {error}"
    );
    assert_eq!(arena_counts(&topo), counts_before);
    assert_eq!(fingerprint(&topo, solid), before);
    assert_eq!(
        fingerprint(&topo, other).volume_nano,
        (8.0_f64 * 1e9) as i128
    );
}

/// The deprecated flat-bevel engine had the same defect (r=50 returned
/// 3833.333 mm³ as a "valid" solid). It now refuses through the same oracle.
#[test]
fn legacy_flat_fillet_oversized_radius_is_refused_closed() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();

    assert_typed_noop_failure(
        "flat-bevel fillet r=50 on a 10mm box",
        &mut topo,
        solid,
        |t, s| fillet(t, s, &edges[..1], 50.0),
    );
}

/// The rolling-ball engine already refused an oversized radius by name; pin
/// the other half of the contract: the refusal leaves no arena state behind.
#[test]
fn legacy_rolling_ball_oversized_radius_is_refused_closed() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();

    assert_typed_noop_failure(
        "rolling-ball fillet r=50 on a 10mm box",
        &mut topo,
        solid,
        |t, s| fillet_rolling_ball(t, s, &edges[..1], 50.0),
    );
}

/// The flat-bevel chamfer validates its result already; pin that its refusal
/// is transactional too (it assembles the invalid solid before the closing
/// gate fires, so without the transaction the arena would keep it).
#[test]
fn legacy_chamfer_failure_leaves_the_arena_unchanged() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();

    assert_typed_noop_failure("chamfer d=50 on a 10mm box", &mut topo, solid, |t, s| {
        chamfer(t, s, &edges[..1], 50.0)
    });
}

/// A rolling-ball failure *after assembly* — the blend-adjacent second pass
/// whose historical "success" carried orientation-inconsistent shared edges —
/// must roll the arena all the way back, not just the input solid.
#[test]
fn rolling_ball_post_assembly_refusal_rolls_back_everything() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    let first = fillet_rolling_ball(&mut topo, solid, &edges[0..1], 0.1)
        .expect("first fillet must succeed");

    let filletable = remus_operations::query::filter_filletable_edges(
        &topo,
        first,
        &solid_edges(&topo, first).unwrap(),
    )
    .unwrap();
    let target = *filletable.first().expect("a blendable edge remains");

    let counts_before = arena_counts(&topo);
    let before = fingerprint(&topo, first);
    // Whether this particular target succeeds or is refused, the arena must
    // only ever change through a committed success.
    if let Err(error) = fillet_rolling_ball(&mut topo, first, &[target], 0.05) {
        assert!(
            !blend_failure_code(&error).is_empty(),
            "typed refusal expected, got {error}"
        );
        assert_eq!(
            arena_counts(&topo),
            counts_before,
            "a post-assembly refusal must roll the arena back exactly"
        );
        assert_eq!(fingerprint(&topo, first), before);
    }
}

// ── 3. Attributes survive a failed blend ────────────────────────────

#[test]
fn failed_fillet_preserves_face_attributes() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    let face = solid_faces(&topo, solid).unwrap()[0];
    topo.set_face_attributes(
        face,
        EntityAttributes {
            name: Some("datum face".to_string()),
            color: None,
        },
    )
    .unwrap();

    let counts_before = arena_counts(&topo);
    let error = fillet_variable(
        &mut topo,
        solid,
        &[(edges[0], FilletRadiusLaw::Constant(50.0))],
    )
    .expect_err("the oversized fillet must be refused");

    assert!(!blend_failure_code(&error).is_empty());
    assert_eq!(
        arena_counts(&topo),
        counts_before,
        "a failed fillet must not touch the attribute store either"
    );
    assert_eq!(
        topo.attributes().face(face).and_then(|a| a.name.as_deref()),
        Some("datum face"),
        "the face name must survive the rolled-back fillet"
    );
}

// ── 4. Radius/scale/law sweeps — not one convenient point ───────────

/// The same relative fillet at 1e-3/1/1e3 model scales: each succeeds, each
/// removes the closed-form sliver scaled by s³, and an oversized radius at
/// each scale is refused without a trace.
#[test]
fn variable_fillet_scale_sweep() {
    for scale in [1e-3_f64, 1.0, 1e3] {
        let edge_len = 10.0 * scale;
        let radius = 0.1 * edge_len;

        let mut topo = Topology::new();
        let solid = make_box(&mut topo, edge_len, edge_len, edge_len).unwrap();
        let edges = solid_edges(&topo, solid).unwrap();
        let input_volume = solid_volume(&topo, solid, DEFLECTION * scale).unwrap();

        let result = fillet_variable(
            &mut topo,
            solid,
            &[(edges[0], FilletRadiusLaw::Constant(radius))],
        )
        .unwrap_or_else(|e| panic!("scale {scale}: a supported fillet must succeed: {e}"));
        assert_ne!(result, solid, "scale {scale}: must not echo the input");

        let expected = input_volume - convex_fillet_removed(radius, edge_len);
        let volume = solid_volume(&topo, result, DEFLECTION * scale).unwrap();
        let slack = convex_fillet_removed(radius, edge_len) * 0.2 + 1e-9 * scale.powi(3);
        assert!(
            (volume - expected).abs() < slack,
            "scale {scale}: expected ≈{expected:.6}, got {volume:.6}"
        );
        let report = validate_solid(&topo, result, &ValidateOptions::default()).unwrap();
        assert!(report.is_valid(), "scale {scale}: result must validate");

        // Oversized at the same scale: refused, nothing left behind.
        assert_typed_noop_failure(
            &format!("scale {scale} oversized variable fillet"),
            &mut topo,
            solid,
            |t, s| {
                fillet_variable(
                    t,
                    s,
                    &[(edges[1], FilletRadiusLaw::Constant(50.0 * edge_len))],
                )
            },
        );
    }
}

/// Every radius law evaluates on the supported case: constant, linear, and
/// s-curve each produce a valid, watertight, distinct solid that removed
/// material.
#[test]
fn variable_fillet_law_sweep_all_remove_material() {
    let laws = [
        FilletRadiusLaw::Constant(1.0),
        FilletRadiusLaw::Linear {
            start: 0.5,
            end: 1.5,
        },
        FilletRadiusLaw::SCurve {
            start: 0.5,
            end: 1.5,
        },
    ];
    for law in laws {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edges(&topo, solid).unwrap();

        let result = fillet_variable(&mut topo, solid, &[(edges[0], law.clone())])
            .unwrap_or_else(|e| panic!("{law:?}: must succeed: {e}"));
        assert_ne!(result, solid);
        let volume = solid_volume(&topo, result, 0.05).unwrap();
        assert!(
            volume < 1000.0 && volume > 990.0,
            "{law:?}: a single-edge fillet removes only a sliver, got {volume}"
        );
        let report = validate_solid(&topo, result, &ValidateOptions::default()).unwrap();
        assert!(report.is_valid(), "{law:?}: result must validate");
    }
}

/// A radius law that crosses zero is invalid input — and must be as clean a
/// no-op as an engine failure.
#[test]
fn variable_fillet_nonpositive_law_is_rejected_clean() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();

    assert_typed_noop_failure("zero-crossing radius law", &mut topo, solid, |t, s| {
        fillet_variable(
            t,
            s,
            &[(
                edges[0],
                FilletRadiusLaw::Linear {
                    start: 1.0,
                    end: -1.0,
                },
            )],
        )
    });
}

/// A repeated seed must blend the edge once — not emit two coincident canal
/// surfaces. The answer must equal the single-spec answer.
#[test]
fn variable_fillet_duplicate_specs_blend_once() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();

    let single = fillet_variable(
        &mut topo,
        solid,
        &[(edges[0], FilletRadiusLaw::Constant(1.0))],
    )
    .unwrap();
    let single_volume = solid_volume(&topo, single, DEFLECTION).unwrap();

    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    let doubled = fillet_variable(
        &mut topo,
        solid,
        &[
            (edges[0], FilletRadiusLaw::Constant(1.0)),
            (edges[0], FilletRadiusLaw::Constant(1.0)),
        ],
    )
    .expect("a duplicated seed is still one fillet");
    let doubled_volume = solid_volume(&topo, doubled, DEFLECTION).unwrap();

    assert!(
        (single_volume - doubled_volume).abs() < 1e-6,
        "duplicate specs must not blend twice: {single_volume} vs {doubled_volume}"
    );
    let report = validate_solid(&topo, doubled, &ValidateOptions::default()).unwrap();
    assert!(report.is_valid(), "the deduped result must validate");
}
