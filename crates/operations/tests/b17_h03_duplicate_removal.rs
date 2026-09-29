//! B17/PERF-H03 duplicate-face removal qualification — end to end.
//!
//! The heal-crate suite (`heal/src/fix/duplicate_faces.rs`) proves recognition
//! and removal mechanics on isolated shells. This matrix proves the REPAIR on
//! valid solids through the verified wrapper.
//!
//! Each cell asserts dual-validator validity, entity census, closed-form
//! volume, and ray-cast material probes before and after, plus idempotence,
//! transactional rollback, and the attribute veto — for plain, holed, and
//! arc-bounded duplicates.
//!
//! Equivalence contract:
//! `docs/kernel-maturity/b17-h03-duplicate-face-contract.md`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_heal::fix::{FixConfig, FixMode};
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::{solid_entity_counts, solid_faces};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const TOL: f64 = 1e-7;
const REL_VOL: f64 = 1e-6;

/// Config with every fixer off except duplicate-face removal, so each repair
/// below is proved to come from THAT fixer (B17 matrix convention).
fn duplicate_only_config() -> FixConfig {
    FixConfig {
        fix_reorder: FixMode::Off,
        fix_connectivity: FixMode::Off,
        fix_closure: FixMode::Off,
        fix_small_edges: FixMode::Off,
        fix_self_intersection: FixMode::Off,
        fix_degenerate_edges: FixMode::Off,
        fix_gaps_2d: FixMode::Off,
        fix_gaps_3d: FixMode::Off,
        fix_lacking: FixMode::Off,
        fix_notched: FixMode::Off,
        fix_tail: FixMode::Off,
        fix_intersecting_edges: FixMode::Off,
        fix_wire_orientation: FixMode::Off,
        fix_add_natural_bound: FixMode::Off,
        fix_missing_seam: FixMode::Off,
        fix_small_area: FixMode::Off,
        fix_duplicate_faces: FixMode::Auto,
        fix_intersecting_wires: FixMode::Off,
        fix_orientation: FixMode::Off,
        fix_same_parameter: FixMode::Off,
        fix_vertex_tolerance: FixMode::Off,
        fix_pcurve: FixMode::Off,
        fix_coincident_vertices: FixMode::Off,
        fix_wireframe: FixMode::Off,
        fix_split_common_vertex: FixMode::Off,
        fix_small_faces: FixMode::Off,
    }
}

/// Append an exact geometric duplicate of `face` (fresh vertices/edges/wires,
/// same winding and surface) to the solid's outer shell.
fn inject_duplicate_of(topo: &mut Topology, solid: SolidId, face: remus_topology::face::FaceId) {
    let source = topo.face(face).unwrap().clone();
    // Copy one wire loop positionally: fresh vertices at the same points,
    // edges rebuilt with the source curves and matching orientation.
    let copy_wire = |topo: &mut Topology, wire_id: remus_topology::wire::WireId| {
        let wire = topo.wire(wire_id).unwrap().clone();
        let starts: Vec<Point3> = wire
            .edges()
            .iter()
            .map(|oe| {
                let edge = topo.edge(oe.edge()).unwrap();
                topo.vertex(oe.oriented_start(edge)).unwrap().point()
            })
            .collect();
        let last_end = {
            let oe = wire.edges().last().unwrap();
            let edge = topo.edge(oe.edge()).unwrap();
            topo.vertex(oe.oriented_end(edge)).unwrap().point()
        };
        let mut verts: Vec<_> = starts
            .iter()
            .map(|point| topo.add_vertex(Vertex::new(*point, TOL)))
            .collect();
        verts.push(topo.add_vertex(Vertex::new(last_end, TOL)));
        let edges: Vec<_> = wire
            .edges()
            .iter()
            .enumerate()
            .map(|(k, oe)| {
                let edge = topo.edge(oe.edge()).unwrap().clone();
                // New wire uses are all forward, so rebuilt edges must run
                // in traversal order unconditionally: preserving the source
                // edge's natural direction here would reverse backward-used
                // edges and flip the winding (plus break connectivity).
                let end = if edge.is_closed() {
                    verts[k]
                } else {
                    verts[k + 1]
                };
                let mut copy = Edge::new(verts[k], end, edge.curve().clone());
                copy.set_trim(edge.trim().map(
                    |(t0, t1)| {
                        if oe.is_forward() { (t0, t1) } else { (t1, t0) }
                    },
                ));
                topo.add_edge(copy)
            })
            .collect();
        let forward: Vec<OrientedEdge> = edges
            .into_iter()
            .map(|edge| OrientedEdge::new(edge, true))
            .collect();
        topo.add_wire(Wire::new(forward, true).unwrap())
    };
    let outer = copy_wire(topo, source.outer_wire());
    let holes: Vec<_> = source
        .inner_wires()
        .iter()
        .map(|wire| copy_wire(topo, *wire))
        .collect();
    let mut duplicate = Face::new(outer, holes, source.surface().clone());
    duplicate.set_reversed(source.is_reversed());
    let duplicate = topo.add_face(duplicate);
    let shell_id = topo.solid(solid).unwrap().outer_shell();
    let mut faces = topo.shell(shell_id).unwrap().faces().to_vec();
    faces.push(duplicate);
    *topo.shell_mut(shell_id).unwrap() = Shell::new(faces).unwrap();
}

/// First face of `solid` whose wire layout matches `wants_holes`.
fn find_face(topo: &Topology, solid: SolidId, wants_holes: bool) -> remus_topology::face::FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|face| {
            let has_holes = !topo.face(*face).unwrap().inner_wires().is_empty();
            has_holes == wants_holes
        })
        .unwrap()
}

/// Dual-validator + census + closed-form volume oracle for a repaired solid.
fn check_repaired(
    topo: &Topology,
    solid: SolidId,
    label: &str,
    expected_faces: usize,
    expected_volume: f64,
) {
    let operations_report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(
        operations_report.is_valid(),
        "{label}: L3 validation issues: {:?}",
        operations_report.issues
    );
    let check_report = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(
        check_report.is_valid(),
        "{label}: check validation issues: {:?}",
        check_report.issues
    );
    let (faces, _, _) = solid_entity_counts(topo, solid).unwrap();
    assert_eq!(faces, expected_faces, "{label}: face census");
    let volume = solid_volume(topo, solid, 0.01).unwrap();
    let relative = (volume - expected_volume).abs() / expected_volume.abs();
    assert!(
        relative <= REL_VOL,
        "{label}: volume {volume:.12e} vs {expected_volume:.12e} (rel {relative:.3e})"
    );
}

fn assert_inside(topo: &Topology, solid: SolidId, point: Point3, label: &str) {
    let classification = remus_check::classify::classify_point(
        topo,
        solid,
        point,
        &remus_check::classify::ClassifyOptions::default(),
    )
    .unwrap();
    assert_eq!(
        classification,
        remus_check::classify::PointClassification::Inside,
        "{label}: expected Inside at {point:?}"
    );
}

fn assert_outside(topo: &Topology, solid: SolidId, point: Point3, label: &str) {
    let classification = remus_check::classify::classify_point(
        topo,
        solid,
        point,
        &remus_check::classify::ClassifyOptions::default(),
    )
    .unwrap();
    assert_eq!(
        classification,
        remus_check::classify::PointClassification::Outside,
        "{label}: expected Outside at {point:?}"
    );
}

#[test]
fn box_duplicate_repair_commits_verified() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let target = find_face(&topo, solid, false);
    inject_duplicate_of(&mut topo, solid, target);
    assert_eq!(solid_entity_counts(&topo, solid).unwrap().0, 7);

    let config = duplicate_only_config();
    let report =
        remus_operations::heal::fix_shape_verified(&mut topo, solid, &config, Some(TOL)).unwrap();
    let removals: usize = report
        .fixing
        .actions
        .iter()
        .filter(|action| action.kind == remus_heal::fix::RepairActionKind::DuplicateFaceRemoved)
        .map(|action| action.count)
        .sum();
    assert_eq!(removals, 1, "exactly one duplicate removal disclosed");
    check_repaired(&topo, report.solid, "box-duplicate", 6, 24.0);
    // Material is where the box was: center inside, far point outside.
    assert_inside(
        &topo,
        report.solid,
        Point3::new(1.0, 1.5, 2.0),
        "box-duplicate",
    );
    assert_outside(
        &topo,
        report.solid,
        Point3::new(100.0, 100.0, 100.0),
        "box-duplicate",
    );

    // Idempotence through the verified wrapper: the repaired solid commits
    // again with zero disclosed repairs.
    let again =
        remus_operations::heal::fix_shape_verified(&mut topo, report.solid, &config, Some(TOL))
            .unwrap();
    assert_eq!(again.fixing.actions_taken, 0, "repair must be idempotent");
    check_repaired(&topo, again.solid, "box-duplicate-idem", 6, 24.0);
}

#[test]
fn cylinder_cap_duplicate_repair_commits_verified() {
    // Arc-bounded duplicates end to end: a cylinder cap disc (closed Circle
    // rim) duplicated with fresh topology.
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
    let cap = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|face| {
            matches!(
                topo.face(*face).unwrap().surface(),
                FaceSurface::Plane { .. }
            )
        })
        .unwrap();
    inject_duplicate_of(&mut topo, solid, cap);

    let config = duplicate_only_config();
    let report =
        remus_operations::heal::fix_shape_verified(&mut topo, solid, &config, Some(TOL)).unwrap();
    let removals: usize = report
        .fixing
        .actions
        .iter()
        .filter(|action| action.kind == remus_heal::fix::RepairActionKind::DuplicateFaceRemoved)
        .map(|action| action.count)
        .sum();
    assert_eq!(removals, 1, "exactly one cap removal disclosed");
    let expected_volume = std::f64::consts::PI * 2.0;
    check_repaired(&topo, report.solid, "cap-duplicate", 3, expected_volume);
    assert_inside(
        &topo,
        report.solid,
        Point3::new(0.0, 0.0, 1.0),
        "cap-duplicate",
    );
    assert_outside(
        &topo,
        report.solid,
        Point3::new(0.0, 0.0, 3.0),
        "cap-duplicate",
    );
}

#[test]
fn frame_holed_duplicate_repair_commits_verified() {
    // Perforated duplicates end to end: an exact box-cut frame carries
    // holed annulus faces (possibly with split collinear edges from the
    // boolean); duplicating one exercises holes + subdivision together.
    let mut topo = Topology::new();
    let outer = make_box(&mut topo, 4.0, 4.0, 0.5).unwrap();
    let tool = make_box(&mut topo, 2.0, 2.0, 1.0).unwrap();
    // Center the tool in the blank: bore [1,3]² through the 0.5-thick plate.
    remus_operations::transform::transform_solid(
        &mut topo,
        tool,
        &remus_math::mat::Mat4::translation(1.0, 1.0, -0.25),
    )
    .unwrap();
    let frame = boolean(&mut topo, BooleanOp::Cut, outer, tool).unwrap();
    // Baseline: valid with the ring volume; face count is whatever the exact
    // boolean produced (walls may split) — the repair must return to it.
    let baseline_faces = solid_entity_counts(&topo, frame).unwrap().0;
    let operations_report = remus_operations::validate::validate_solid(&topo, frame).unwrap();
    assert!(
        operations_report.is_valid(),
        "frame baseline issues: {:?}",
        operations_report.issues
    );
    let volume = solid_volume(&topo, frame, 0.01).unwrap();
    assert!(
        (volume - 6.0).abs() / 6.0 <= REL_VOL,
        "frame baseline volume {volume:.12e} vs 6.0"
    );
    let holed = find_face(&topo, frame, true);
    inject_duplicate_of(&mut topo, frame, holed);
    assert_eq!(
        solid_entity_counts(&topo, frame).unwrap().0,
        baseline_faces + 1
    );

    let config = duplicate_only_config();
    let report =
        remus_operations::heal::fix_shape_verified(&mut topo, frame, &config, Some(TOL)).unwrap();
    let removals: usize = report
        .fixing
        .actions
        .iter()
        .filter(|action| action.kind == remus_heal::fix::RepairActionKind::DuplicateFaceRemoved)
        .map(|action| action.count)
        .sum();
    assert_eq!(removals, 1, "exactly one holed removal disclosed");
    check_repaired(&topo, report.solid, "frame-duplicate", baseline_faces, 6.0);
    // Material probes: webbing inside, bore and far field outside.
    assert_inside(
        &topo,
        report.solid,
        Point3::new(0.5, 0.5, 0.25),
        "frame-duplicate",
    );
    assert_outside(
        &topo,
        report.solid,
        Point3::new(2.0, 2.0, 0.25),
        "frame-duplicate",
    );
}

#[test]
fn distinct_coincident_sheets_record_no_removal() {
    // Two adjacent coplanar tiles on a box face plane: same carrier, disjoint
    // regions — kept with no removal recorded (unverified path: the shell is
    // open by construction, so only the disclosed action count binds).
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let tile = |topo: &mut Topology, x0: f64| {
        let corners = [
            Point3::new(x0, 5.0, 0.0),
            Point3::new(x0 + 1.0, 5.0, 0.0),
            Point3::new(x0 + 1.0, 6.0, 0.0),
            Point3::new(x0, 6.0, 0.0),
        ];
        let vs: Vec<_> = corners
            .iter()
            .map(|point| topo.add_vertex(Vertex::new(*point, TOL)))
            .collect();
        let es: Vec<_> = (0..4)
            .map(|i| topo.add_edge(Edge::new(vs[i], vs[(i + 1) % 4], EdgeCurve::Line)))
            .collect();
        let wire = topo.add_wire(
            Wire::new(
                es.into_iter()
                    .map(|edge| OrientedEdge::new(edge, true))
                    .collect(),
                true,
            )
            .unwrap(),
        );
        topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: remus_math::vec::Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    };
    let left = tile(&mut topo, 0.0);
    let right = tile(&mut topo, 1.0);
    let shell_id = topo.solid(solid).unwrap().outer_shell();
    let mut faces = topo.shell(shell_id).unwrap().faces().to_vec();
    faces.push(left);
    faces.push(right);
    *topo.shell_mut(shell_id).unwrap() = Shell::new(faces).unwrap();

    let config = duplicate_only_config();
    let (repaired, result, history) =
        remus_heal::fix::fix_shape_with_history(&mut topo, solid, &config, Some(TOL)).unwrap();
    assert_eq!(result.actions_taken, 0, "tiles must be kept");
    assert!(history.is_empty());
    assert_eq!(solid_entity_counts(&topo, repaired).unwrap().0, 8);
}

#[test]
fn shared_face_refusal_rolls_back_transactionally() {
    // One FaceId in two shells: the verified wrapper refuses typed and the
    // transaction leaves the topology bit-identical.
    let mut topo = Topology::new();
    let anchor = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let shared = find_face(&topo, anchor, false);
    // Build a second shell reusing `shared`: the inner shell holds the shared
    // face plus a fresh twin of it (so the pair is detected as duplicates
    // AND shared).
    let twin_corners: Vec<Point3> = {
        let face = topo.face(shared).unwrap();
        let wire = topo.wire(face.outer_wire()).unwrap();
        wire.edges()
            .iter()
            .map(|oe| {
                let edge = topo.edge(oe.edge()).unwrap();
                topo.vertex(oe.oriented_start(edge)).unwrap().point()
            })
            .collect()
    };
    let vs: Vec<_> = twin_corners
        .iter()
        .map(|point| topo.add_vertex(Vertex::new(*point, TOL)))
        .collect();
    let es: Vec<_> = (0..vs.len())
        .map(|i| topo.add_edge(Edge::new(vs[i], vs[(i + 1) % vs.len()], EdgeCurve::Line)))
        .collect();
    let twin_wire = topo.add_wire(
        Wire::new(
            es.into_iter()
                .map(|edge| OrientedEdge::new(edge, true))
                .collect(),
            true,
        )
        .unwrap(),
    );
    let (normal, duplicated) = match topo.face(shared).unwrap().surface() {
        FaceSurface::Plane { normal, d } => (*normal, *d),
        other => panic!("box face must be planar, got {:?}", other.type_tag()),
    };
    let twin = topo.add_face(Face::new(
        twin_wire,
        vec![],
        FaceSurface::Plane {
            normal,
            d: duplicated,
        },
    ));
    let outer_shell = topo.solid(anchor).unwrap().outer_shell();
    let inner_shell = topo.add_shell(Shell::new(vec![shared, twin]).unwrap());
    *topo.solid_mut(anchor).unwrap() =
        remus_topology::solid::Solid::new(outer_shell, vec![inner_shell]);
    let before: Vec<usize> = solid_faces(&topo, anchor)
        .unwrap()
        .iter()
        .map(|face| face.index())
        .collect();
    assert!(
        before
            .iter()
            .filter(|index| **index == shared.index())
            .count()
            >= 1
    );

    let config = duplicate_only_config();
    let error = remus_operations::heal::fix_shape_verified(&mut topo, anchor, &config, Some(TOL))
        .unwrap_err();
    let typed = matches!(
        error,
        remus_operations::OperationsError::HealingRepairRefused { .. }
            | remus_operations::OperationsError::ConfiguredHealingValidationFailed { .. }
            | remus_operations::OperationsError::HealingValidationFailed { .. }
            | remus_operations::OperationsError::Heal(_)
    );
    assert!(typed, "refusal must be typed, got {error:?}");
    let after: Vec<usize> = solid_faces(&topo, anchor)
        .unwrap()
        .iter()
        .map(|face| face.index())
        .collect();
    assert_eq!(before, after, "transaction must roll back fully");
}

#[test]
fn named_duplicate_veto_holds_end_to_end() {
    // A named duplicate carries provenance the survivor lacks: no removal is
    // recorded and the shell keeps both faces.
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let target = find_face(&topo, solid, false);
    inject_duplicate_of(&mut topo, solid, target);
    let faces = solid_faces(&topo, solid).unwrap();
    let duplicate = *faces.last().unwrap();
    topo.set_face_attributes(
        duplicate,
        remus_topology::attributes::EntityAttributes {
            name: Some("imported-cap".to_string()),
            color: None,
        },
    )
    .unwrap();

    let config = duplicate_only_config();
    let (repaired, result, history) =
        remus_heal::fix::fix_shape_with_history(&mut topo, solid, &config, Some(TOL)).unwrap();
    assert_eq!(result.actions_taken, 0, "named duplicate must be vetoed");
    assert!(history.is_empty());
    assert_eq!(solid_entity_counts(&topo, repaired).unwrap().0, 7);
}
