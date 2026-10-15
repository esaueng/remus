//! OPS-3 (kernel review 2026-10-14) — orientation-reversing transforms left
//! every boundary wound against its face.
//!
//! A mirror is an affine map with a negative determinant. `transform_solid`
//! rebuilt every carrier with its outward normal (plane normals as
//! covectors, analytic carriers right-handed, NURBS u-reversed) but kept
//! each wire's traversal, which the reflection had turned clockwise. A
//! mirrored unit box reported six `FaceOrientationConsistency` issues, and
//! intersecting it with a cylinder under `ExactOnly` returned `Ok` with six
//! same-sense shared edges: the exact gate accepted a mis-oriented result
//! (op_seq_81 finding F16).
//!
//! The fix reverses every boundary traversal of the transformed faces after
//! a det < 0 map, in place (`Topology::reverse_face_boundaries`), on the
//! in-place, copying and single-face transform paths.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_4, PI};

use remus_check::validate::{ValidateOptions, ValidationIssue, validate_solid};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::mat::Mat4;
use remus_math::vec::{Point2, Point3, Vec2, Vec3};
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::copy::copy_and_transform_solid;
use remus_operations::measure::solid_volume;
use remus_operations::mirror::mirror;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::{transform_face, transform_solid};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::pcurve::PCurve;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_face_loops;

const DEFLECTION: f64 = 0.01;

fn issues(topo: &Topology, solid: SolidId) -> Vec<ValidationIssue> {
    validate_solid(topo, solid, &ValidateOptions::default())
        .unwrap()
        .issues
}

fn assert_valid(topo: &Topology, solid: SolidId, label: &str) {
    let issues = issues(topo, solid);
    assert!(issues.is_empty(), "{label}: {issues:?}");
    for face in solid_faces(topo, solid).unwrap() {
        validate_face_loops(topo, face).unwrap();
    }
}

fn assert_volume(topo: &Topology, solid: SolidId, expected: f64, label: &str) {
    assert_volume_within(topo, solid, expected, 1e-9, label);
}

fn assert_volume_within(
    topo: &Topology,
    solid: SolidId,
    expected: f64,
    relative: f64,
    label: &str,
) {
    let volume = solid_volume(topo, solid, DEFLECTION).unwrap();
    assert!(
        (volume - expected).abs() <= relative * expected.abs().max(1.0),
        "{label}: volume {volume}, expected {expected}"
    );
}

/// Unit Newell normal of a wire, sampled through its curved edges.
fn wire_normal(topo: &Topology, wire: remus_topology::wire::WireId) -> Vec3 {
    let polygon = remus_check::util::wire_polygon(topo, wire).unwrap();
    remus_check::util::polygon_normal(&polygon)
}

/// Newell vector of a planar face's outer wire, through its vertices.
fn outer_newell(topo: &Topology, face: FaceId) -> Vec3 {
    let wire = topo.wire(topo.face(face).unwrap().outer_wire()).unwrap();
    let points: Vec<Point3> = wire
        .edges()
        .iter()
        .map(|used| {
            let edge = topo.edge(used.edge()).unwrap();
            let start = if used.is_forward() {
                edge.start()
            } else {
                edge.end()
            };
            topo.vertex(start).unwrap().point()
        })
        .collect();
    let mut n = Vec3::new(0.0, 0.0, 0.0);
    for (i, a) in points.iter().enumerate() {
        let b = points[(i + 1) % points.len()];
        n += Vec3::new(
            (a.y() - b.y()) * (a.z() + b.z()),
            (a.z() - b.z()) * (a.x() + b.x()),
            (a.x() - b.x()) * (a.y() + b.y()),
        );
    }
    n
}

fn exact_only() -> OperationContext {
    OperationContext::new().with_fallback(FallbackPolicy::ExactOnly)
}

/// The review repro: every face of a mirrored unit box wound against its
/// stored normal.
#[test]
fn mirrored_box_winds_with_its_faces() {
    let planes = [
        (Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)),
        (Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)),
        (Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
        (Point3::new(3.0, -2.0, 7.0), Vec3::new(1.0, 2.0, -2.0)),
    ];
    for (point, normal) in planes {
        let mut topo = Topology::new();
        let source = make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
        let mirrored = mirror(&mut topo, source, point, normal).unwrap();
        let label = format!("mirror across {normal:?} through {point:?}");
        assert_valid(&topo, mirrored, &label);
        assert_volume(&topo, mirrored, 6.0, &label);
        for face in solid_faces(&topo, mirrored).unwrap() {
            let FaceSurface::Plane { normal, .. } = *topo.face(face).unwrap().surface() else {
                panic!("box faces stay planar");
            };
            assert!(
                outer_newell(&topo, face).dot(normal) > 0.0,
                "{label}: face {face:?} winds against its normal"
            );
        }
        // The source is untouched.
        assert_valid(&topo, source, "source box");
    }
}

/// op_seq_81 F16: the mirrored box intersected with a unit cylinder was
/// accepted exact with six same-sense shared edges.
#[test]
fn mirrored_box_intersect_cylinder_is_valid() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 1.0, 1.0).unwrap();
    let source = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let mirrored = mirror(
        &mut topo,
        source,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();

    let outcome = boolean_with_context(
        &mut topo,
        BooleanOp::Intersect,
        mirrored,
        cylinder,
        &exact_only(),
    )
    .unwrap();

    assert_eq!(outcome.quality, BooleanQuality::Exact);
    let result = outcome.solid;
    assert_valid(&topo, result, "mirrored box ∩ cylinder");
    // The quarter of the unit disc in x ≥ 0, y ≤ 0, one unit tall.
    assert_volume(&topo, result, FRAC_PI_4, "mirrored box ∩ cylinder");
    let probes = [
        (Point3::new(0.5, -0.5, 0.5), PointClassification::Inside),
        (Point3::new(0.1, -0.9, 0.9), PointClassification::Inside),
        (Point3::new(0.5, 0.5, 0.5), PointClassification::Outside),
        (Point3::new(0.9, -0.9, 0.5), PointClassification::Outside),
        (Point3::new(-0.5, -0.5, 0.5), PointClassification::Outside),
        (Point3::new(0.5, -0.5, 1.5), PointClassification::Outside),
    ];
    for (point, expected) in probes {
        assert_eq!(
            classify_point(&topo, result, point, DEFLECTION, 1e-7).unwrap(),
            expected,
            "probe {point:?}"
        );
    }
}

/// The other two operators on the same pair, against their closed forms.
#[test]
fn mirrored_box_cut_and_fuse_with_cylinder_are_valid() {
    let cases = [
        (BooleanOp::Cut, true, 1.0 - FRAC_PI_4),
        (BooleanOp::Cut, false, PI - FRAC_PI_4),
        (BooleanOp::Fuse, true, PI + 1.0 - FRAC_PI_4),
    ];
    for (op, box_first, expected) in cases {
        let mut topo = Topology::new();
        let cylinder = make_cylinder(&mut topo, 1.0, 1.0).unwrap();
        let source = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let mirrored = mirror(
            &mut topo,
            source,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let (a, b) = if box_first {
            (mirrored, cylinder)
        } else {
            (cylinder, mirrored)
        };
        let outcome = boolean_with_context(&mut topo, op, a, b, &exact_only()).unwrap();
        let label = format!("{op:?} box_first={box_first}");
        assert_eq!(outcome.quality, BooleanQuality::Exact, "{label}");
        let errors: Vec<_> = issues(&topo, outcome.solid)
            .into_iter()
            .filter(|issue| issue.severity == remus_check::validate::Severity::Error)
            .collect();
        assert!(errors.is_empty(), "{label}: {errors:?}");
        assert_volume(&topo, outcome.solid, expected, &label);
    }
}

/// Inner wires reverse with their outer wire: a drilled plate mirrored
/// keeps every planar face wound with its normal and every hole opposite.
#[test]
fn mirrored_drilled_plate_keeps_holes_opposite() {
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 4.0, 4.0, 1.0).unwrap();
    let bore = make_cylinder(&mut topo, 1.0, 3.0).unwrap();
    transform_solid(&mut topo, bore, &Mat4::translation(2.0, 2.0, -1.0)).unwrap();
    let drilled = boolean_with_context(&mut topo, BooleanOp::Cut, plate, bore, &exact_only())
        .unwrap()
        .solid;
    let mirrored = mirror(
        &mut topo,
        drilled,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();

    // The wrapped bore wall's Newell polygon is degenerate (its vector area
    // is zero to rounding), so the whole-face winding check reads noise
    // there on the drilled source too; planar faces are judged below.
    let unexpected: Vec<_> = issues(&topo, mirrored)
        .into_iter()
        .filter(|issue| {
            !matches!(
                issue.entity,
                remus_check::validate::EntityRef::Face(face)
                    if matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_))
            )
        })
        .collect();
    assert!(unexpected.is_empty(), "{unexpected:?}");
    assert_volume(&topo, mirrored, 16.0 - PI, "mirrored drilled plate");
    let mut holed = 0;
    for face in solid_faces(&topo, mirrored).unwrap() {
        validate_face_loops(&topo, face).unwrap();
        let data = topo.face(face).unwrap();
        let FaceSurface::Plane { normal, .. } = *data.surface() else {
            continue;
        };
        assert!(wire_normal(&topo, data.outer_wire()).dot(normal) > 0.9);
        for &inner in data.inner_wires() {
            assert!(wire_normal(&topo, inner).dot(normal) < -0.9);
            holed += 1;
        }
    }
    assert_eq!(holed, 2, "both plate faces keep their hole");
}

/// Reflections that are not `mirror`: an in-place reflection matrix, a
/// negative uniform scale through the copying path, and an anisotropic
/// reflection.
#[test]
fn every_negative_determinant_path_winds_with_its_faces() {
    let reflect_x = Mat4([
        [-1.0, 0.0, 0.0, 5.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    let negative_scale = Mat4([
        [-2.0, 0.0, 0.0, 0.0],
        [0.0, -2.0, 0.0, 0.0],
        [0.0, 0.0, -2.0, 1.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    let anisotropic = Mat4([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, -3.0, 0.0, 0.0],
        [0.0, 0.0, 0.5, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);

    let mut topo = Topology::new();
    let in_place = make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
    transform_solid(&mut topo, in_place, &reflect_x).unwrap();
    assert_valid(&topo, in_place, "in-place reflection");
    assert_volume(&topo, in_place, 6.0, "in-place reflection");

    let source = make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
    let scaled = copy_and_transform_solid(&mut topo, source, &negative_scale).unwrap();
    assert_valid(&topo, scaled, "copied negative scale");
    assert_volume(&topo, scaled, 48.0, "copied negative scale");

    let stretched = make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
    transform_solid(&mut topo, stretched, &anisotropic).unwrap();
    assert_valid(&topo, stretched, "anisotropic reflection");
    assert_volume(&topo, stretched, 9.0, "anisotropic reflection");

    // A cylinder converted to NURBS by an anisotropic reflection still
    // bounds positive material, and its caps wind with their normals. Its
    // volume is read through the mesh route for the NURBS wall, hence the
    // looser bound.
    let cylinder = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
    transform_solid(&mut topo, cylinder, &anisotropic).unwrap();
    assert_volume_within(
        &topo,
        cylinder,
        PI * 2.0 * 3.0 * 0.5,
        1e-4,
        "anisotropic cylinder",
    );
    for face in solid_faces(&topo, cylinder).unwrap() {
        validate_face_loops(&topo, face).unwrap();
        let data = topo.face(face).unwrap();
        if let FaceSurface::Plane { normal, .. } = *data.surface() {
            assert!(
                wire_normal(&topo, data.outer_wire()).dot(normal) > 0.9,
                "cap {face:?} winds against its normal"
            );
        }
    }
    let errors: Vec<_> = issues(&topo, cylinder)
        .into_iter()
        .filter(|issue| issue.severity == remus_check::validate::Severity::Error)
        .collect();
    assert!(errors.is_empty(), "{errors:?}");
}

/// A single planar face reflected on its own keeps its winding with its
/// normal.
#[test]
fn reflected_free_face_winds_with_its_normal() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
    let face = solid_faces(&topo, solid).unwrap()[0];
    let copy = remus_operations::copy::copy_face(&mut topo, face).unwrap();
    transform_face(
        &mut topo,
        copy,
        &Mat4([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]),
    )
    .unwrap();
    let FaceSurface::Plane { normal, .. } = *topo.face(copy).unwrap().surface() else {
        panic!("box faces are planar");
    };
    assert!(outer_newell(&topo, copy).dot(normal) > 0.0);
    validate_face_loops(&topo, copy).unwrap();
}

/// The reflected UV charts make stored traces stale: the mirrored copy
/// carries none, and the source keeps its own.
#[test]
fn reflection_clears_stale_pcurves() {
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let face = solid_faces(&topo, source).unwrap()[0];
    let first = topo
        .wire(topo.face(face).unwrap().outer_wire())
        .unwrap()
        .edges()[0];
    let trace = PCurve::new(
        Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
        0.0,
        1.0,
    );
    topo.set_pcurve_oriented(first.edge(), face, first.is_forward(), trace)
        .unwrap();

    let mirrored = mirror(
        &mut topo,
        source,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();

    for copied in solid_faces(&topo, mirrored).unwrap() {
        assert!(topo.pcurves_for_face(copied).is_empty());
    }
    assert!(
        topo.pcurve_oriented(first.edge(), face, first.is_forward())
            .is_some()
    );
    assert_valid(&topo, mirrored, "mirrored box with a source trace");

    transform_solid(
        &mut topo,
        source,
        &Mat4([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]),
    )
    .unwrap();
    assert!(topo.pcurves_for_face(face).is_empty());
    assert_valid(&topo, source, "box reflected in place");
}
