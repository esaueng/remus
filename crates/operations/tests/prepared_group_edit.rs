//! Prepared blend-group edits (`PreparedGroupEdit`): the reference video's
//! live drags of a band radius (A2) and the boss diameter (A7) on the
//! filleted Gentle-Gecko part and on the cumulative body (A2 R1.9, A3 R1,
//! A4 0.7 and A6 0.8 chamfer chains).
//!
//! Every preview frame is compared with the unprepared edit at the same
//! value from the same source (`resize_blend`, `resize_cylindrical_face` on
//! a copy of the session): entity counts, face census and volume to 1e-9.
//! Every frame and commit meets the verification bar (both validators, a
//! watertight manifold mesh at two deflections, API vs mesh volume within
//! 1e-4), a commit equals the unprepared edit (journaled too), cancelling
//! leaves the source byte-identical, and refusals keep the unprepared
//! call's typed code.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::blend_cluster::{
    Convexity, GroupEditKind, PreparedGroupEdit, recognize_blend_cluster,
};
use remus_operations::blend_ops::{chamfer_v2_tangent_chain, fillet_cascade};
use remus_operations::journal_ops::{resize_blend_journaled, resize_cylindrical_face_journaled};
use remus_operations::measure::solid_volume;
use remus_operations::push_pull::resize_cylindrical_face;
use remus_operations::resize_blend::{resize_blend, resize_blend_failure_code};
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

// ---------------------------------------------------------------------------
// Fixtures.

fn gecko() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let text = include_str!("data/gentle_gecko_filleted.step");
    let solid = remus_io::step::reader::read_step(text, &mut topo).unwrap()[0];
    (topo, solid)
}

/// The boss wall: the cylinder on the Z axis.
fn boss_of(topo: &Topology, solid: SolidId) -> FaceId {
    let walls: Vec<FaceId> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
                if c.axis().cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9
                    && c.origin().x().abs() < 1e-9
                    && c.origin().y().abs() < 1e-9)
        })
        .collect();
    assert_eq!(walls.len(), 1, "one boss wall");
    walls[0]
}

/// The group member of the boss's group on the given side: the convex top
/// band along the x = 0 face, or the concave seam round there.
fn x0_member(topo: &Topology, solid: SolidId, convexity: Convexity) -> FaceId {
    let cluster = recognize_blend_cluster(topo, solid, boss_of(topo, solid)).unwrap();
    let found: Vec<FaceId> = cluster
        .members
        .iter()
        .filter(|m| m.convexity == convexity)
        .flat_map(|m| m.faces.iter().copied())
        .filter(|&face| match topo.face(face).unwrap().surface() {
            FaceSurface::Cylinder(c) if convexity == Convexity::Convex => {
                c.axis().cross(Vec3::new(0.0, 1.0, 0.0)).length() < 1e-9
            }
            FaceSurface::Cylinder(c) => c.origin().x() < -1.0,
            _ => false,
        })
        .collect();
    assert_eq!(found.len(), 1, "one {convexity:?} x = 0 member");
    found[0]
}

fn band_radius(topo: &Topology, face: FaceId) -> f64 {
    match topo.face(face).unwrap().surface() {
        FaceSurface::Cylinder(c) => c.radius(),
        other => panic!("band is a {}", other.type_tag()),
    }
}

fn line_where(topo: &Topology, solid: SolidId, on: impl Fn(Point3) -> bool) -> EdgeId {
    let found: Vec<EdgeId> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Line))
        .filter(|&edge| {
            let data = topo.edge(edge).unwrap();
            on(topo.vertex(data.start()).unwrap().point())
                && on(topo.vertex(data.end()).unwrap().point())
        })
        .collect();
    assert_eq!(found.len(), 1);
    found[0]
}

/// The cumulative body: A2 R1.9 → A3 R1 → A4 0.7 → A6 0.8 on the filleted
/// Gecko (each step verified in `blend_group_cumulative.rs`).
fn cumulative() -> (Topology, SolidId) {
    let (mut topo, mut solid) = gecko();
    let band = x0_member(&topo, solid, Convexity::Convex);
    solid = resize_blend(&mut topo, solid, band, 4.0, 1.9)
        .unwrap()
        .solid;
    let a3 = line_where(&topo, solid, |p| {
        p.x().abs() < 1e-7 && (p.y() - 18.0).abs() < 1e-7
    });
    solid = fillet_cascade(&mut topo, solid, &[a3], 1.0).unwrap().solid;
    let a4 = line_where(&topo, solid, |p| {
        (p.x() - 30.0).abs() < 1e-7 && (p.z() - 24.0).abs() < 1e-7
    });
    solid = chamfer_v2_tangent_chain(&mut topo, solid, &[a4], 0.7, 0.7)
        .unwrap()
        .solid;
    let a6 = line_where(&topo, solid, |p| p.y().abs() < 1e-7 && p.z().abs() < 1e-7);
    solid = chamfer_v2_tangent_chain(&mut topo, solid, &[a6], 0.8, 0.8)
        .unwrap()
        .solid;
    (topo, solid)
}

// ---------------------------------------------------------------------------
// Verification.

fn bbox_diagonal(topo: &Topology, solid: SolidId) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in &tessellate_solid(topo, solid, 0.1).unwrap().positions {
        for (axis, value) in [p.x(), p.y(), p.z()].into_iter().enumerate() {
            lo[axis] = lo[axis].min(value);
            hi[axis] = hi[axis].max(value);
        }
    }
    ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt()
}

fn mesh_volume(mesh: &TriangleMesh) -> f64 {
    let origin = mesh.positions[0];
    mesh.indices
        .chunks(3)
        .map(|tri| {
            let a = mesh.positions[tri[0] as usize] - origin;
            let b = mesh.positions[tri[1] as usize] - origin;
            let c = mesh.positions[tri[2] as usize] - origin;
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

/// PLAN.md's bar: both validators clean, a watertight manifold mesh at two
/// deflections at or below `bbox_diag * 5e-5`, and the API volume within
/// 1e-4 of the independent mesh volume.
fn verify(topo: &Topology, solid: SolidId, label: &str) {
    let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {:?}", report.issues);
    let strict = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(strict.is_valid(), "{label}: {:?}", strict.issues);
    let diagonal = bbox_diagonal(topo, solid);
    let volume = solid_volume(topo, solid, diagonal * 1e-5).unwrap();
    for deflection in [diagonal * 5e-5, diagonal * 2e-5] {
        let mesh = tessellate_solid(topo, solid, deflection).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "{label} at {deflection}");
        assert_eq!(non_manifold_edge_count(&mesh), 0, "{label} at {deflection}");
        let independent = mesh_volume(&mesh);
        assert!(
            (independent - volume).abs() <= 1e-4 * volume,
            "{label}: mesh volume {independent} vs {volume} at {deflection}"
        );
    }
}

fn census(topo: &Topology, solid: SolidId) -> BTreeMap<&'static str, usize> {
    let mut census = BTreeMap::new();
    for face in solid_faces(topo, solid).unwrap() {
        *census
            .entry(topo.face(face).unwrap().surface().type_tag())
            .or_insert(0) += 1;
    }
    census
}

/// Entity counts, face census and volume to 1e-9 relative.
fn assert_same_body(
    topo: &Topology,
    solid: SolidId,
    other: &Topology,
    expected: SolidId,
    label: &str,
) {
    assert_eq!(
        solid_entity_counts(topo, solid).unwrap(),
        solid_entity_counts(other, expected).unwrap(),
        "{label}: F/E/V"
    );
    assert_eq!(
        census(topo, solid),
        census(other, expected),
        "{label}: census"
    );
    let volume = solid_volume(topo, solid, 1e-3).unwrap();
    let reference = solid_volume(other, expected, 1e-3).unwrap();
    assert!(
        (volume - reference).abs() <= 1e-9 * reference,
        "{label}: volume {volume} vs unprepared {reference}"
    );
}

fn fingerprint(topo: &Topology, solid: SolidId) -> String {
    remus_io::step::writer::write_step(topo, &[solid]).unwrap()
}

/// The code a refusal carries across the WASM boundary.
fn code(error: &OperationsError) -> String {
    match error {
        OperationsError::BlendCluster(inner) => inner.code().to_string(),
        other => resize_blend_failure_code(other).to_string(),
    }
}

/// What a prepared edit is compared with: the unprepared edit at `value`.
fn unprepared(
    topo: &Topology,
    solid: SolidId,
    face: FaceId,
    kind: GroupEditKind,
    value: f64,
) -> (Topology, Result<SolidId, OperationsError>) {
    let mut topo = topo.clone();
    let result = match kind {
        GroupEditKind::BandRadius { expected_radius } => {
            resize_blend(&mut topo, solid, face, expected_radius, value).map(|r| r.solid)
        }
        GroupEditKind::CylinderRadius { .. } => {
            resize_cylindrical_face(&mut topo, solid, face, value)
        }
    };
    (topo, result)
}

/// Preview each of `values` (source-relative, as a drag does), checking
/// every frame against the unprepared edit and the verification bar, then
/// cancel: the source is byte-identical.
fn drag(topo: &mut Topology, prepared: &PreparedGroupEdit, values: &[f64], label: &str) {
    let (solid, face, kind) = (prepared.solid(), prepared.face(), prepared.kind());
    let before = fingerprint(topo, solid);
    for &value in values {
        let label = format!("{label}: frame {value}");
        let frame = prepared.preview(topo, value).unwrap();
        verify(topo, frame, &label);
        let (reference_topo, reference) = unprepared(topo, solid, face, kind, value);
        assert_same_body(topo, frame, &reference_topo, reference.unwrap(), &label);
        assert_eq!(fingerprint(topo, solid), before, "{label}: source");
    }
}

/// Commit `value`: the result equals the unprepared edit and meets the bar;
/// the journaled commit equals the unprepared journaled edit.
fn commit(topo: &mut Topology, prepared: &PreparedGroupEdit, value: f64, label: &str) -> SolidId {
    let (solid, face, kind) = (prepared.solid(), prepared.face(), prepared.kind());
    let label = format!("{label}: commit {value}");
    let before = fingerprint(topo, solid);
    let (reference_topo, reference) = unprepared(topo, solid, face, kind, value);
    let reference = reference.unwrap();

    let mut journaled_topo = topo.clone();
    let journaled = prepared
        .commit_journaled(&mut journaled_topo, value)
        .unwrap();
    let mut unprepared_journal_topo = topo.clone();
    let unprepared_journaled = match kind {
        GroupEditKind::BandRadius { expected_radius } => resize_blend_journaled(
            &mut unprepared_journal_topo,
            solid,
            face,
            expected_radius,
            value,
        ),
        GroupEditKind::CylinderRadius { .. } => {
            resize_cylindrical_face_journaled(&mut unprepared_journal_topo, solid, face, value)
        }
    }
    .unwrap();
    assert_same_body(
        &journaled_topo,
        journaled.solid,
        &unprepared_journal_topo,
        unprepared_journaled.solid,
        &format!("{label} (journaled)"),
    );
    assert_eq!(
        journaled.map.modified.len(),
        unprepared_journaled.map.modified.len(),
        "{label}: journaled modified faces"
    );
    assert_eq!(
        journaled.map.deleted, unprepared_journaled.map.deleted,
        "{label}: journaled deleted faces"
    );

    let result = prepared.commit(topo, value).unwrap();
    assert_eq!(fingerprint(topo, solid), before, "{label}: source");
    verify(topo, result, &label);
    assert_same_body(topo, result, &reference_topo, reference, &label);
    result
}

// ---------------------------------------------------------------------------
// A2: a band radius drag.

fn band_drag(mut topo: Topology, solid: SolidId, label: &str) {
    let band = x0_member(&topo, solid, Convexity::Convex);
    let radius = band_radius(&topo, band);
    let prepared = PreparedGroupEdit::prepare_band(&topo, solid, band, radius).unwrap();
    assert!(prepared.learned_refusal(), "{label}: {prepared:?}");
    drag(&mut topo, &prepared, &[1.5, 1.9, 1.5], label);
    commit(&mut topo, &prepared, 1.5, label);
}

#[test]
fn band_radius_drag_on_the_filleted_gecko() {
    let (topo, solid) = gecko();
    band_drag(topo, solid, "gecko A2");
}

#[test]
fn band_radius_drag_on_the_cumulative_body() {
    let (topo, solid) = cumulative();
    band_drag(topo, solid, "cumulative A2");
}

// ---------------------------------------------------------------------------
// A7: the boss diameter, dragged and committed in the video's order
// (14.6, 17, a 21.1 preview that is cancelled, 14.7).

fn boss_drags(mut topo: Topology, mut solid: SolidId, label: &str) {
    for (diameter, keep) in [(14.6, true), (17.0, true), (21.1, false), (14.7, true)] {
        let label = format!("{label} Ø{diameter}");
        let boss = boss_of(&topo, solid);
        let prepared = PreparedGroupEdit::prepare_cylinder(&topo, solid, boss).unwrap();
        assert!(prepared.learned_refusal(), "{label}: {prepared:?}");
        drag(&mut topo, &prepared, &[diameter / 2.0], &label);
        if keep {
            solid = commit(&mut topo, &prepared, diameter / 2.0, &label);
        }
    }
}

#[test]
fn boss_diameter_drags_on_the_filleted_gecko() {
    let (topo, solid) = gecko();
    boss_drags(topo, solid, "gecko A7");
}

#[test]
fn boss_diameter_drags_on_the_cumulative_body() {
    let (topo, solid) = cumulative();
    boss_drags(topo, solid, "cumulative A7");
}

// ---------------------------------------------------------------------------
// Refusals, staleness, values the group route does not take.

#[test]
fn prepared_refusals_keep_the_unprepared_codes() {
    let (mut topo, solid) = gecko();
    let before = fingerprint(&topo, solid);
    let boss = boss_of(&topo, solid);
    let wall = PreparedGroupEdit::prepare_cylinder(&topo, solid, boss).unwrap();
    // Ø34: the sharp resize fits but the 1 mm top edge cannot take R4. Ø40:
    // the wall meets the y = 18 face. Both are refused alike by frames,
    // commits and the unprepared edit, and leave the body alone.
    for radius in [17.0, 20.0] {
        let (_, reference) = unprepared(&topo, solid, boss, wall.kind(), radius);
        let expected = code(&reference.unwrap_err());
        assert_eq!(expected, "cylinder-diameter-unbuildable");
        assert_eq!(
            code(&wall.preview(&mut topo, radius).unwrap_err()),
            expected
        );
        assert_eq!(code(&wall.commit(&mut topo, radius).unwrap_err()), expected);
        assert_eq!(fingerprint(&topo, solid), before);
    }
    // The current radius is not an edit; the unprepared refusal answers.
    let (_, reference) = unprepared(&topo, solid, boss, wall.kind(), 6.0);
    assert_eq!(
        wall.preview(&mut topo, 6.0).unwrap_err().to_string(),
        reference.unwrap_err().to_string()
    );

    // A concave seam round R4 → 20 under the R4 shoulder has no
    // construction (its band leaves no valid corner): prepared and
    // unprepared refuse it with one code.
    let seam = x0_member(&topo, solid, Convexity::Concave);
    let seam_edit = PreparedGroupEdit::prepare_band(&topo, solid, seam, 4.0).unwrap();
    let (_, reference) = unprepared(&topo, solid, seam, seam_edit.kind(), 20.0);
    let expected = code(&reference.unwrap_err());
    assert_eq!(expected, "blend-cluster-reblend-failed");
    assert_eq!(
        code(&seam_edit.preview(&mut topo, 20.0).unwrap_err()),
        expected
    );
    assert_eq!(fingerprint(&topo, solid), before);

    // A wrong witness and a plain block face are typed refusals at prepare.
    let band = x0_member(&topo, solid, Convexity::Convex);
    let error = PreparedGroupEdit::prepare_band(&topo, solid, band, 3.0).unwrap_err();
    assert_eq!(resize_blend_failure_code(&error), "blend-radius-mismatch");
    let plane = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Plane { .. }
            )
        })
        .unwrap();
    let error = PreparedGroupEdit::prepare_cylinder(&topo, solid, plane).unwrap_err();
    assert_eq!(code(&error), "blend-cluster-not-prepared");
    let error = PreparedGroupEdit::prepare_band(&topo, solid, plane, 4.0).unwrap_err();
    assert_eq!(code(&error), "blend-cluster-not-prepared");
    assert_eq!(fingerprint(&topo, solid), before);
}

#[test]
fn a_changed_source_makes_the_preparation_stale() {
    let (mut topo, solid) = gecko();
    let boss = boss_of(&topo, solid);
    let prepared = PreparedGroupEdit::prepare_cylinder(&topo, solid, boss).unwrap();
    assert!(prepared.is_current(&topo));
    // A restored checkpoint is the same source, as a drag restores it every
    // frame.
    let checkpoint = topo.clone();
    prepared.preview(&mut topo, 7.3).unwrap();
    topo.restore_preserving_handle_slots(&checkpoint);
    assert!(prepared.is_current(&topo));
    prepared.preview(&mut topo, 7.3).unwrap();

    // Editing the source in place is not.
    remus_operations::transform::transform_solid(
        &mut topo,
        solid,
        &remus_math::mat::Mat4::translation(0.0, 0.0, 1.0),
    )
    .unwrap();
    assert!(!prepared.is_current(&topo));
    let before = fingerprint(&topo, solid);
    let error = prepared.preview(&mut topo, 7.3).unwrap_err();
    assert!(
        matches!(error, OperationsError::InvalidInput { .. }),
        "{error}"
    );
    assert!(error.to_string().contains("stale"), "{error}");
    assert!(prepared.commit(&mut topo, 7.3).is_err());
    assert_eq!(fingerprint(&topo, solid), before);
}
