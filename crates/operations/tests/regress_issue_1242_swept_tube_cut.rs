//! OpenZCAD issue 1242: Subtract consumed the tool and returned the target
//! unchanged for an annular revolve cutting a swept tube.
//!
//! The tube's side is one torus face bounded by its two end circles and a
//! seam along one equator (see `data/issue_1242_1244_fixtures.md`). Three
//! defects lined up. Its broad-phase box was taken from those edges alone and
//! stopped 3.3 mm below the crown, so the cutter's top plane was never
//! intersected with it. Its `v` band was read from five samples per edge and
//! missed the quarter of the tube where the projection wraps, so every torus
//! section stopped at the tube's extreme along the torus axis. And no face
//! splitter could partition a tube whose sections wrap it, cross its seam
//! and end on each other, so the splitter dropped them all as orphaned arcs
//! and kept the tube whole.
//!
//! The pair now cuts exactly: the sections are traced to the cutter's rims,
//! the tube is partitioned as a certified arrangement in its cut-open chart,
//! and no section chain is dropped on the way. The checks here are material
//! witnesses, the independent reference volumes and the analytic face
//! census, through every entry point a replay can take, in both orders for
//! Fuse and Intersect, and for the same tube built by Remus's own revolve.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean, boolean_with_entity_evolution};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::journal_ops::boolean_journaled;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const TUBE: &str = include_str!("data/issue_1242_swept_tube_target.step");
const ANNULUS: &str = include_str!("data/issue_1242_annular_tool.step");

/// Strictly inside both operands, more than 1 mm from either boundary.
const WITNESS: [f64; 3] = [6.175_785_17, 1.031_013_20, 4.870_290_36];
/// Inside the tube's left leg, inside the cutter's bore: material the cut
/// keeps.
const KEPT: [f64; 3] = [2.0, 1.0, 4.14];
/// Independent references on the same inputs: a second B-rep kernel at a
/// 1e-9 integration tolerance (data file notes).
const TUBE_VOLUME: f64 = 219.538_848;
const ANNULUS_VOLUME: f64 = 3_166.725_395;
const CUT_VOLUME: f64 = 175.430_430;
const COMMON_VOLUME: f64 = 44.108_443;
const FUSE_VOLUME: f64 = 3_342.155_805;
/// The face integrator agrees with the references to 2.2e-6 relative here;
/// this bound is 3e-5 of the cut's volume.
const VOLUME_TOL: f64 = 0.005;

fn import(topo: &mut Topology, text: &str) -> SolidId {
    let solids = remus_io::step::reader::read_step(text, topo).unwrap();
    assert_eq!(solids.len(), 1);
    solids[0]
}

fn operands() -> (Topology, SolidId, SolidId) {
    let mut topo = Topology::new();
    let tube = import(&mut topo, TUBE);
    let annulus = import(&mut topo, ANNULUS);
    (topo, tube, annulus)
}

/// The same tube built by Remus: the profile circle revolved about the
/// arch's axis, then scaled and placed as the fixture was. Its seam lies on
/// the tube's extreme along the torus axis, not on its inner equator.
fn native_operands() -> (Topology, SolidId, SolidId) {
    use remus_math::curves::Circle3D;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    let mut topo = Topology::new();
    let scale = 0.82;
    // Arc (-8, 0, -5) -> (0, 0, 6) -> (8, 0, -5): centre (0, 0, -53/22).
    let start = Point3::new(-8.0 * scale + 6.0, 1.0, -5.0 * scale + 0.5);
    let axis_point = Point3::new(6.0, 1.0, (-53.0 / 22.0) * scale + 0.5);
    let radial = start - axis_point;
    let tangent = Vec3::new(radial.z(), 0.0, -radial.x()).normalize().unwrap();
    let tangent = if tangent.z() > 0.0 { tangent } else { -tangent };
    let circle = Circle3D::new(start, tangent, 2.0 * scale).unwrap();
    let vertex = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let mut edge = Edge::new(vertex, vertex, EdgeCurve::Circle(circle));
    edge.set_trim(Some((0.0, std::f64::consts::TAU)));
    let edge = topo.add_edge(edge);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let d = tangent.dot(Vec3::new(start.x(), start.y(), start.z()));
    let profile = topo.add_face(Face::new(
        wire,
        Vec::new(),
        FaceSurface::Plane { normal: tangent, d },
    ));
    let sweep = 215.890_506_459_585_6_f64.to_radians();
    let tube = remus_operations::revolve::revolve(
        &mut topo,
        profile,
        axis_point,
        Vec3::new(0.0, 1.0, 0.0),
        sweep,
    )
    .unwrap();
    let annulus = import(&mut topo, ANNULUS);
    (topo, tube, annulus)
}

fn point(p: [f64; 3]) -> Point3 {
    Point3::new(p[0], p[1], p[2])
}

fn class(topo: &Topology, solid: SolidId, p: [f64; 3]) -> PointClassification {
    classify_point(topo, solid, point(p), 0.01, 1e-7).unwrap()
}

fn live_counts(topo: &Topology) -> [usize; 4] {
    [
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_faces(),
        topo.num_solids(),
    ]
}

/// An exact analytic B-rep: planes, cylinders and tori only (a mesh fallback
/// is hundreds of planes), at least one torus face, and every edge shared by
/// exactly two face uses.
fn assert_exact_brep(topo: &Topology, solid: SolidId, what: &str) {
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    assert!(faces.len() < 20, "{what}: {} faces", faces.len());
    let mut tori = 0;
    let mut uses: std::collections::BTreeMap<usize, usize> = std::collections::BTreeMap::new();
    for &face in &faces {
        let face = topo.face(face).unwrap();
        let tag = face.surface().type_tag();
        assert!(
            matches!(tag, "plane" | "cylinder" | "torus"),
            "{what}: {tag} face"
        );
        tori += usize::from(tag == "torus");
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wire).unwrap().edges() {
                *uses.entry(oe.edge().index()).or_default() += 1;
            }
        }
    }
    assert!(tori >= 1, "{what}: the tube's torus face is gone");
    for (edge, count) in uses {
        assert_eq!(count, 2, "{what}: edge {edge} used {count} times");
    }
}

/// The imported tube's planar end caps carry pcurves whose SameParameter
/// proof the validator cannot make (see the data notes); a result keeps
/// those caps, so it may report exactly that, and nothing else, as an error.
fn assert_valid_but_for_inherited_caps(topo: &Topology, solid: SolidId, what: &str) {
    let report = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    for issue in &report.issues {
        if issue.severity == remus_check::validate::Severity::Error {
            assert!(
                matches!(
                    issue.check,
                    remus_check::validate::CheckId::EdgeSameParameter
                ) && issue.description.contains("plane surface / circle edge"),
                "{what}: {issue:?}"
            );
        }
    }
}

/// The operands as read, and the witnesses.
#[test]
fn fixture_operands_hold_the_witness() {
    let (topo, tube, annulus) = operands();
    let v = solid_volume(&topo, tube, 0.005).unwrap();
    assert!((v - TUBE_VOLUME).abs() < 0.05, "tube volume {v}");
    let v = solid_volume(&topo, annulus, 0.005).unwrap();
    assert!((v - ANNULUS_VOLUME).abs() < 0.05, "annulus volume {v}");
    assert_eq!(class(&topo, tube, WITNESS), PointClassification::Inside);
    assert_eq!(class(&topo, annulus, WITNESS), PointClassification::Inside);
    assert_eq!(class(&topo, tube, KEPT), PointClassification::Inside);
    assert_eq!(class(&topo, annulus, KEPT), PointClassification::Outside);
}

type Entry = fn(&mut Topology, SolidId, SolidId) -> Result<SolidId, OperationsError>;

const CUT_ENTRIES: [(&str, Entry); 3] = [
    ("boolean", |t, a, b| boolean(t, BooleanOp::Cut, a, b)),
    ("entity evolution", |t, a, b| {
        boolean_with_entity_evolution(t, BooleanOp::Cut, a, b).map(|(s, _)| s)
    }),
    ("journaled", |t, a, b| {
        boolean_journaled(t, remus_algo::bop::BooleanOp::Cut, a, b).map(|r| r.solid)
    }),
];

/// Exact Cut through every entry point a replay can take, with no section
/// chain dropped: the overlap is removed, the bore-side leg is kept, the
/// volume is the reference's, and the sources survive.
#[test]
fn cut_is_exact_through_every_entry_point() {
    for (name, entry) in CUT_ENTRIES {
        let (mut topo, tube, annulus) = operands();
        let drops = remus_algo::gfa::SectionDropScope::enter();
        let result = entry(&mut topo, tube, annulus)
            .unwrap_or_else(|e| panic!("{name}: the cut refused: {e:?}"));
        assert_eq!(drops.dropped(), 0, "{name}: a section chain was dropped");
        assert_ne!(
            class(&topo, result, WITNESS),
            PointClassification::Inside,
            "{name}: the cut kept material inside both operands"
        );
        assert_eq!(
            class(&topo, result, KEPT),
            PointClassification::Inside,
            "{name}: the cut lost the tube's leg in the bore"
        );
        let v = solid_volume(&topo, result, 0.005).unwrap();
        assert!(
            (v - CUT_VOLUME).abs() < VOLUME_TOL,
            "{name}: cut volume {v}, reference {CUT_VOLUME}"
        );
        assert_exact_brep(&topo, result, name);
        assert_valid_but_for_inherited_caps(&topo, result, name);
        assert_eq!(class(&topo, tube, WITNESS), PointClassification::Inside);
        assert_eq!(class(&topo, annulus, WITNESS), PointClassification::Inside);
        let v = solid_volume(&topo, tube, 0.005).unwrap();
        assert!((v - TUBE_VOLUME).abs() < 0.05, "{name}: tube changed: {v}");
    }
}

/// Fuse and Intersect in both orders: the witness is material of both, so
/// it is inside both results, and the volumes are the references'. Cut and
/// Intersect partition the tube.
#[test]
fn fuse_and_intersect_are_exact_in_both_orders() {
    for (op, reference) in [
        (BooleanOp::Fuse, FUSE_VOLUME),
        (BooleanOp::Intersect, COMMON_VOLUME),
    ] {
        for swap in [false, true] {
            let what = format!("{op:?} swap={swap}");
            let (mut topo, tube, annulus) = operands();
            let (a, b) = if swap {
                (annulus, tube)
            } else {
                (tube, annulus)
            };
            let drops = remus_algo::gfa::SectionDropScope::enter();
            let result =
                boolean(&mut topo, op, a, b).unwrap_or_else(|e| panic!("{what}: refused: {e:?}"));
            assert_eq!(drops.dropped(), 0, "{what}: a section chain was dropped");
            assert_eq!(
                class(&topo, result, WITNESS),
                PointClassification::Inside,
                "{what}: lost material both operands hold"
            );
            let kept = class(&topo, result, KEPT);
            let expected = if op == BooleanOp::Fuse {
                PointClassification::Inside
            } else {
                PointClassification::Outside
            };
            assert_eq!(kept, expected, "{what}: tube-only material");
            let v = solid_volume(&topo, result, 0.005).unwrap();
            assert!(
                (v - reference).abs() < VOLUME_TOL,
                "{what}: volume {v}, reference {reference}"
            );
            assert_exact_brep(&topo, result, &what);
            assert_valid_but_for_inherited_caps(&topo, result, &what);
        }
    }
    let (mut topo, tube, annulus) = operands();
    let cut = boolean(&mut topo, BooleanOp::Cut, tube, annulus).unwrap();
    let common = boolean(&mut topo, BooleanOp::Intersect, tube, annulus).unwrap();
    let total =
        solid_volume(&topo, cut, 0.005).unwrap() + solid_volume(&topo, common, 0.005).unwrap();
    assert!(
        (total - TUBE_VOLUME).abs() < VOLUME_TOL,
        "cut + common = {total}, tube {TUBE_VOLUME}"
    );
}

/// The tube built by Remus's own revolve cuts, fuses and intersects exactly
/// too (its seam is on another parallel; a whole-tube band's strict extent
/// and the marcher's rim exit used to stop its sections short there).
#[test]
fn native_revolve_tube_is_exact_too() {
    for (op, reference) in [
        (BooleanOp::Cut, CUT_VOLUME),
        (BooleanOp::Fuse, FUSE_VOLUME),
        (BooleanOp::Intersect, COMMON_VOLUME),
    ] {
        let what = format!("native {op:?}");
        let (mut topo, tube, annulus) = native_operands();
        let v = solid_volume(&topo, tube, 0.005).unwrap();
        assert!((v - TUBE_VOLUME).abs() < 0.01, "{what}: tube volume {v}");
        let drops = remus_algo::gfa::SectionDropScope::enter();
        let result = boolean(&mut topo, op, tube, annulus)
            .unwrap_or_else(|e| panic!("{what}: refused: {e:?}"));
        assert_eq!(drops.dropped(), 0, "{what}: a section chain was dropped");
        let inside = class(&topo, result, WITNESS) == PointClassification::Inside;
        assert_eq!(inside, op != BooleanOp::Cut, "{what}: witness");
        let v = solid_volume(&topo, result, 0.005).unwrap();
        assert!(
            (v - reference).abs() < 2.0 * VOLUME_TOL,
            "{what}: volume {v}, reference {reference}"
        );
        assert_exact_brep(&topo, result, &what);
        let report = remus_check::validate::validate_solid(
            &topo,
            result,
            &remus_check::validate::ValidateOptions::default(),
        )
        .unwrap();
        assert!(report.is_valid(), "{what}: {report:?}");
    }
}

/// A layout the chart splitter does not qualify still fails closed: a cutter
/// grazing the tube's seam line refuses or cuts correctly, and a refusal is
/// typed and leaves the arena as it was.
#[test]
fn unqualified_layout_refuses_transactionally_or_cuts_correctly() {
    let (mut topo, tube, _) = operands();
    // A thin slab whose top face touches the tube's inner-equator seam line
    // from below along the arch's apex: the tangency the chart refuses.
    let slab = make_box(&mut topo, 4.0, 4.0, 1.0).unwrap();
    // The seam runs on the inner equator, at z = -1.4755 + 6.8955 - 1.64
    // under the crown.
    let apex = -1.475_454_545_455 + 6.895_454_545_455 - 1.64;
    transform_solid(&mut topo, slab, &Mat4::translation(4.0, -1.0, apex - 1.0)).unwrap();
    let before = live_counts(&topo);
    match boolean(&mut topo, BooleanOp::Cut, tube, slab) {
        Ok(result) => {
            let v = solid_volume(&topo, result, 0.005).unwrap();
            assert!(
                (v - TUBE_VOLUME).abs() < VOLUME_TOL,
                "a tangent slab removes no volume: {v}"
            );
            assert_eq!(class(&topo, result, WITNESS), PointClassification::Inside);
        }
        Err(e) => {
            assert!(
                matches!(
                    e,
                    OperationsError::ExactOnlyUnattainable | OperationsError::Algo(_)
                ),
                "unexpected refusal {e:?}"
            );
            assert_eq!(live_counts(&topo), before, "refusal left entities");
        }
    }
}

/// Controls: a cut by the annulus moved clear of the tube is the tube, and
/// so is a cut by a box inside the tube's arch, clear of the tube but well
/// inside its bounding box.
#[test]
fn unchanged_cuts_that_are_correct_still_succeed() {
    let (mut topo, tube, annulus) = operands();
    transform_solid(&mut topo, annulus, &Mat4::translation(100.0, 0.0, 0.0)).unwrap();
    let result = boolean(&mut topo, BooleanOp::Cut, tube, annulus).unwrap();
    let v = solid_volume(&topo, result, 0.005).unwrap();
    assert!((v - TUBE_VOLUME).abs() < 0.05, "disjoint cut volume {v}");
    assert_eq!(class(&topo, result, WITNESS), PointClassification::Inside);

    // The arch's hollow: the tube's centre circle has radius 6.8955 about
    // (6, 1, -1.4755) and the tube radius is 1.64, so the 3 mm cube centred
    // at (6, 1, 0.5) stays more than 1 mm clear of it.
    let (mut topo, tube, _) = operands();
    let cube = make_box(&mut topo, 3.0, 3.0, 3.0).unwrap();
    transform_solid(&mut topo, cube, &Mat4::translation(4.5, -0.5, -1.0)).unwrap();
    let result = boolean(&mut topo, BooleanOp::Cut, tube, cube).unwrap();
    let v = solid_volume(&topo, result, 0.005).unwrap();
    assert!(
        (v - TUBE_VOLUME).abs() < 0.05,
        "hollow-clearance cut volume {v}"
    );
    assert_eq!(class(&topo, result, WITNESS), PointClassification::Inside);
}
