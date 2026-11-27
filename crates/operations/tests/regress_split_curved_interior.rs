//! Regression: `split(plane)` refused a plane that clearly crossed a curved
//! solid as if it missed ("entirely on one side", `InvalidInput`).
//!
//! The side judgement read only the body's edges. A sphere's only edges are
//! its equator arcs and a primitive torus's are two degenerate seams, so a
//! plane through the bulge of either face, clear of every edge, looked like a
//! miss. The side of a face is now read from its surface too: the edges, plus
//! the carrier's interior critical points along the plane normal (a sphere's
//! two poles along it, a torus's four Gauss-map points or its top and bottom
//! rings) where the face's trim holds them.
//!
//! Every build case here checks both halves against closed-form volumes, and
//! requires each to be a valid closed solid on its own side of the plane: a
//! watertight mesh and material probes on either side of the cut. The
//! tangent and miss planes keep their refusals and categories.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder, make_sphere, make_torus};
use remus_operations::split::{SplitResult, split, split_with_evolution};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_operations::validate::{Severity, validate_solid};
use remus_topology::Topology;
use remus_topology::explorer::solid_entity_counts;
use remus_topology::solid::SolidId;

/// `solid_volume` integrates analytic faces in closed form; this only bounds
/// sampled paths.
const DEFLECTION: f64 = 0.01;

/// Relative agreement with the closed forms. The halves keep the input's
/// exact sphere, cylinder and torus carriers, so only rounding separates them.
const RELATIVE: f64 = 1e-9;

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    solid_volume(topo, solid, DEFLECTION).expect("volume")
}

fn assert_relative(what: &str, actual: f64, expected: f64) {
    let err = (actual - expected).abs() / expected.abs().max(1.0);
    assert!(
        err <= RELATIVE,
        "{what}: got {actual}, expected {expected} (relative error {err:e})"
    );
}

/// A valid solid whose display mesh is closed and manifold.
fn assert_closed_valid(topo: &Topology, solid: SolidId, what: &str) {
    let report = validate_solid(topo, solid).expect("validate");
    let errors: Vec<_> = report
        .issues
        .iter()
        .filter(|i| i.severity == Severity::Error)
        .map(|i| i.description.clone())
        .collect();
    assert!(errors.is_empty(), "{what} is not a valid solid: {errors:?}");
    let mesh = tessellate_solid(topo, solid, 0.05).expect("tessellate");
    assert_eq!(boundary_edge_count(&mesh), 0, "{what}: mesh has open edges");
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "{what}: mesh has non-manifold edges"
    );
}

fn classify(topo: &Topology, solid: SolidId, p: Point3) -> PointClassification {
    classify_point(topo, solid, p, DEFLECTION, 1e-7).expect("classify")
}

/// Split and hold both halves to the closed forms, validity, closure and the
/// side contract: each probe in `pos_probes` is material of the positive half
/// only, each in `neg_probes` of the negative half only.
#[allow(clippy::too_many_arguments)]
fn check_split(
    topo: &mut Topology,
    solid: SolidId,
    point: Point3,
    normal: Vec3,
    expected_pos: f64,
    expected_neg: f64,
    pos_probes: &[Point3],
    neg_probes: &[Point3],
    what: &str,
) -> SplitResult {
    let input = volume(topo, solid);
    assert_relative(
        &format!("{what}: input"),
        input,
        expected_pos + expected_neg,
    );

    let halves =
        split(topo, solid, point, normal).unwrap_or_else(|e| panic!("{what}: split refused: {e}"));
    for (half, side) in [(halves.positive, "positive"), (halves.negative, "negative")] {
        assert_closed_valid(topo, half, &format!("{what}: {side} half"));
    }
    let pos = volume(topo, halves.positive);
    let neg = volume(topo, halves.negative);
    assert_relative(&format!("{what}: positive half"), pos, expected_pos);
    assert_relative(&format!("{what}: negative half"), neg, expected_neg);
    assert_relative(&format!("{what}: halves add up"), pos + neg, input);

    for &p in pos_probes {
        assert_eq!(
            classify(topo, halves.positive, p),
            PointClassification::Inside,
            "{what}: {p:?} must be material of the positive half"
        );
        assert_eq!(
            classify(topo, halves.negative, p),
            PointClassification::Outside,
            "{what}: {p:?} must not be material of the negative half"
        );
    }
    for &p in neg_probes {
        assert_eq!(
            classify(topo, halves.negative, p),
            PointClassification::Inside,
            "{what}: {p:?} must be material of the negative half"
        );
        assert_eq!(
            classify(topo, halves.positive, p),
            PointClassification::Outside,
            "{what}: {p:?} must not be material of the positive half"
        );
    }
    halves
}

/// Volume of a sphere cap of height `h` on a sphere of radius `r`.
fn sphere_cap(r: f64, h: f64) -> f64 {
    PI * h * h * (3.0 * r - h) / 3.0
}

/// Area of the part of a disc of radius `r` beyond a chord at distance `a`
/// from its centre.
fn disc_segment(r: f64, a: f64) -> f64 {
    r * r * (a / r).acos() - a * (r * r - a * a).sqrt()
}

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// Builds one input solid.
type Build = fn(&mut Topology) -> SolidId;

#[test]
fn sphere_split_off_its_equator_builds() {
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, 5.0, 16).unwrap();
    check_split(
        &mut topo,
        sphere,
        Point3::new(0.0, 0.0, 1.0),
        Z,
        sphere_cap(5.0, 4.0),
        sphere_cap(5.0, 6.0),
        &[Point3::new(0.0, 0.0, 4.0), Point3::new(1.0, 1.0, 1.5)],
        &[Point3::new(0.0, 0.0, 0.5), Point3::new(0.0, 0.0, -4.0)],
        "R5 sphere at z = 1",
    );
}

#[test]
fn sphere_split_below_its_equator_builds() {
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, 5.0, 16).unwrap();
    check_split(
        &mut topo,
        sphere,
        Point3::new(0.0, 0.0, -2.5),
        Z,
        sphere_cap(5.0, 7.5),
        sphere_cap(5.0, 2.5),
        &[Point3::new(0.0, 0.0, -2.0), Point3::new(0.0, 0.0, 4.0)],
        &[Point3::new(0.0, 0.0, -3.0), Point3::new(0.5, 0.0, -4.5)],
        "R5 sphere at z = -2.5",
    );
}

#[test]
fn sphere_split_through_its_equator_builds() {
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, 5.0, 16).unwrap();
    let half = 2.0 * PI * 125.0 / 3.0;
    check_split(
        &mut topo,
        sphere,
        Point3::new(0.0, 0.0, 0.0),
        Z,
        half,
        half,
        &[Point3::new(0.0, 0.0, 2.0)],
        &[Point3::new(0.0, 0.0, -2.0)],
        "R5 sphere at z = 0",
    );
}

#[test]
fn cylinder_split_along_its_axis_builds() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    let half = PI * 25.0 * 10.0 / 2.0;
    check_split(
        &mut topo,
        cylinder,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        half,
        half,
        &[Point3::new(0.0, 2.0, 5.0)],
        &[Point3::new(0.0, -2.0, 5.0)],
        "R5 x 10 cylinder through its axis",
    );
}

#[test]
fn cylinder_split_across_its_axis_builds() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    check_split(
        &mut topo,
        cylinder,
        Point3::new(0.0, 0.0, 3.0),
        Z,
        PI * 25.0 * 7.0,
        PI * 25.0 * 3.0,
        &[Point3::new(0.0, 0.0, 4.0)],
        &[Point3::new(0.0, 0.0, 2.0)],
        "R5 x 10 cylinder across its axis",
    );
}

/// The lateral face crossed away from its seam (`x = 5`): the plane meets the
/// wall only between the seam's vertices, through the rims' interiors.
#[test]
fn cylinder_wall_split_between_its_seam_vertices_builds() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    let beyond = 10.0 * disc_segment(5.0, 3.0);
    check_split(
        &mut topo,
        cylinder,
        Point3::new(-3.0, 0.0, 0.0),
        Vec3::new(-1.0, 0.0, 0.0),
        beyond,
        PI * 25.0 * 10.0 - beyond,
        &[Point3::new(-4.0, 0.0, 5.0)],
        &[Point3::new(-2.0, 0.0, 5.0), Point3::new(4.0, 0.0, 5.0)],
        "R5 x 10 cylinder wall at x = -3",
    );
}

/// A cylinder lying on its side, its wall bulging past every rim point: the
/// plane `z = 4` crosses only the wall's interior between the two rims'
/// crossings.
#[test]
fn lying_cylinder_split_through_its_wall_builds() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    // Axis along +x: rotate +z onto +x.
    transform_solid(
        &mut topo,
        cylinder,
        &Mat4([
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]),
    )
    .unwrap();
    let beyond = 10.0 * disc_segment(5.0, 4.0);
    check_split(
        &mut topo,
        cylinder,
        Point3::new(0.0, 0.0, 4.0),
        Z,
        beyond,
        PI * 25.0 * 10.0 - beyond,
        &[Point3::new(5.0, 0.0, 4.5)],
        &[Point3::new(5.0, 0.0, 3.5)],
        "R5 x 10 cylinder lying along x, at z = 4",
    );
}

/// Torus R10 r3: the part of the tube above `z = c` is a disc segment swept
/// about the axis at its centroid radius `R` (Pappus).
#[test]
fn torus_split_through_its_tube_builds() {
    let mut topo = Topology::new();
    let torus = make_torus(&mut topo, 10.0, 3.0, 16).unwrap();
    let total = 2.0 * PI * PI * 10.0 * 9.0;
    let above = 2.0 * PI * 10.0 * disc_segment(3.0, 1.0);
    check_split(
        &mut topo,
        torus,
        Point3::new(0.0, 0.0, 1.0),
        Z,
        above,
        total - above,
        &[Point3::new(10.0, 0.0, 2.0), Point3::new(0.0, -10.0, 2.0)],
        &[Point3::new(10.0, 0.0, 0.0), Point3::new(-10.0, 0.0, -2.0)],
        "R10 r3 torus at z = 1",
    );
}

#[test]
fn torus_split_through_its_axis_builds() {
    let mut topo = Topology::new();
    let torus = make_torus(&mut topo, 10.0, 3.0, 16).unwrap();
    let half = PI * PI * 10.0 * 9.0;
    check_split(
        &mut topo,
        torus,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        half,
        half,
        &[Point3::new(10.0, 0.0, 0.0), Point3::new(1.0, 10.0, 0.0)],
        &[Point3::new(-10.0, 0.0, 0.0), Point3::new(-1.0, -10.0, 0.0)],
        "R10 r3 torus through its axis",
    );
}

/// Unchanged refusals: a plane tangent to the solid at one point stays a
/// one-sided `InvalidInput`, as does a plane that misses it. Neither leaves
/// anything behind.
#[test]
fn tangent_and_missing_planes_stay_refused() {
    let sphere: Build = |t| make_sphere(t, 5.0, 16).unwrap();
    let torus: Build = |t| make_torus(t, 10.0, 3.0, 16).unwrap();
    let cylinder: Build = |t| make_cylinder(t, 5.0, 10.0).unwrap();
    let cases: [(&str, Build, Point3, Vec3); 8] = [
        (
            "sphere tangent at z = 5",
            sphere,
            Point3::new(0.0, 0.0, 5.0),
            Z,
        ),
        (
            "sphere tangent at z = -5",
            sphere,
            Point3::new(0.0, 0.0, -5.0),
            Z,
        ),
        (
            "sphere missed at z = 6",
            sphere,
            Point3::new(0.0, 0.0, 6.0),
            Z,
        ),
        (
            "sphere tangent along (1, 1, 1)",
            sphere,
            Point3::new(0.0, 0.0, 0.0) + Vec3::new(1.0, 1.0, 1.0).normalize().unwrap() * 5.0,
            Vec3::new(1.0, 1.0, 1.0),
        ),
        (
            "torus tangent at z = 3",
            torus,
            Point3::new(0.0, 0.0, 3.0),
            Z,
        ),
        (
            "torus missed at z = 4",
            torus,
            Point3::new(0.0, 0.0, 4.0),
            Z,
        ),
        (
            "cylinder tangent along a ruling at y = 5",
            cylinder,
            Point3::new(0.0, 5.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ),
        (
            "cylinder missed at z = 11",
            cylinder,
            Point3::new(0.0, 0.0, 11.0),
            Z,
        ),
    ];
    for (what, build, point, normal) in cases {
        let mut topo = Topology::new();
        let solid = build(&mut topo);
        let before = solid_entity_counts(&topo, solid).unwrap();
        let journal = topo.journal().snapshot();
        let err = split_with_evolution(&mut topo, solid, point, normal)
            .err()
            .unwrap_or_else(|| panic!("{what}: split must refuse"));
        assert!(
            matches!(err, OperationsError::InvalidInput { .. }),
            "{what}: expected InvalidInput, got {err:?}"
        );
        assert_eq!(solid_entity_counts(&topo, solid).unwrap(), before, "{what}");
        assert_eq!(topo.journal().snapshot(), journal, "{what}");
    }
}

/// Unchanged refusals: a plane coincident with a face, or lying along an
/// edge, touches the body without crossing its interior. Those keep their
/// typed `Unsupported` refusal naming the contained edge.
#[test]
fn planes_along_a_face_or_edge_keep_their_refusal() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    let cube = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let cases = [
        ("cylinder base", cylinder, Point3::new(0.0, 0.0, 0.0), Z),
        ("cylinder top", cylinder, Point3::new(0.0, 0.0, 10.0), Z),
        ("box top", cube, Point3::new(0.0, 0.0, 2.0), Z),
        (
            "box edge",
            cube,
            Point3::new(2.0, 2.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0),
        ),
    ];
    for (what, solid, point, normal) in cases {
        let err = split(&mut topo, solid, point, normal)
            .err()
            .unwrap_or_else(|| panic!("{what}: split must refuse"));
        assert!(
            matches!(&err, OperationsError::Unsupported { operation, reason }
                if *operation == "split" && reason.contains("contains edge")),
            "{what}: expected the contained-edge refusal, got {err:?}"
        );
    }
}

/// The R5 sphere with both hemispheres converted to B-spline faces, each on a
/// whole-sphere carrier trimmed back to its hemisphere.
fn nurbs_sphere(topo: &mut Topology) -> SolidId {
    let sphere = make_sphere(topo, 5.0, 16).unwrap();
    let converted = remus_operations::heal::convert_to_bspline(topo, sphere).unwrap();
    assert!(converted > 0, "the hemispheres must convert to B-spline");
    sphere
}

/// A NURBS face is read through its Bezier control nets, and a carrier that
/// reaches past the plane only outside the face's trim proves nothing: the
/// tangent and missing planes stay one-sided on the B-spline sphere.
#[test]
fn nurbs_sphere_tangent_and_missing_planes_stay_refused() {
    for z in [5.0, -5.0, 6.0, -6.0] {
        let mut topo = Topology::new();
        let sphere = nurbs_sphere(&mut topo);
        let err = split(&mut topo, sphere, Point3::new(0.0, 0.0, z), Z)
            .err()
            .unwrap_or_else(|| panic!("NURBS sphere at z = {z}: split must refuse"));
        assert!(
            matches!(err, OperationsError::InvalidInput { .. }),
            "NURBS sphere at z = {z}: expected InvalidInput, got {err:?}"
        );
    }
}

/// Planes through the interior of a face whose edges all lie on one side, on
/// shapes the exact half-space booleans cannot build yet: they are crossings,
/// not misses. Each split either builds both halves to their closed forms or
/// refuses by name as an interior crossing; neither reads as "entirely on one
/// side".
#[test]
fn interior_crossings_the_fallback_cannot_build_are_not_misses() {
    let n = Vec3::new(1.0, 1.0, 1.0).normalize().unwrap();
    let analytic: Build = |t| make_sphere(t, 5.0, 16).unwrap();
    let cases: [(&str, Build, Point3, Vec3, f64); 3] = [
        (
            "NURBS sphere at z = 1",
            nurbs_sphere,
            Point3::new(0.0, 0.0, 1.0),
            Z,
            1.0,
        ),
        (
            "NURBS sphere at z = -4",
            nurbs_sphere,
            Point3::new(0.0, 0.0, -4.0),
            Z,
            -4.0,
        ),
        (
            "R5 sphere at 4.5 along (1, 1, 1), clear of its equator",
            analytic,
            Point3::new(0.0, 0.0, 0.0) + n * 4.5,
            n,
            4.5,
        ),
    ];
    for (what, build, point, normal, offset) in cases {
        let mut topo = Topology::new();
        let solid = build(&mut topo);
        match split(&mut topo, solid, point, normal) {
            Ok(halves) => {
                assert_relative(
                    &format!("{what}: positive half"),
                    volume(&topo, halves.positive),
                    sphere_cap(5.0, 5.0 - offset),
                );
                assert_relative(
                    &format!("{what}: negative half"),
                    volume(&topo, halves.negative),
                    sphere_cap(5.0, 5.0 + offset),
                );
            }
            Err(OperationsError::Unsupported { reason, .. }) => assert!(
                reason.contains("crosses the interior of face"),
                "{what}: refused for another reason: {reason}"
            ),
            Err(err) => panic!("{what}: expected the halves or a named refusal, got {err:?}"),
        }
    }
}

/// The prepared trim the side judgement reads answers every point exactly as
/// the one-shot `surface_point_in_face` does, abstentions included: on the
/// analytic sphere, torus and cylinder faces and the B-spline hemispheres, at
/// carrier points across each surface's whole parameter range.
#[test]
fn prepared_face_trim_matches_the_one_shot_trim_test() {
    use remus_check::classify::{PreparedFaceTrim, surface_point_in_face};
    use remus_topology::explorer::solid_faces;
    use remus_topology::face::FaceSurface;

    let mut topo = Topology::new();
    let solids = [
        make_sphere(&mut topo, 5.0, 16).unwrap(),
        make_torus(&mut topo, 10.0, 3.0, 16).unwrap(),
        make_cylinder(&mut topo, 5.0, 10.0).unwrap(),
        nurbs_sphere(&mut topo),
    ];
    let mut compared = 0usize;
    for solid in solids {
        for face in solid_faces(&topo, solid).unwrap() {
            let prepared = PreparedFaceTrim::new(&topo, face).unwrap();
            let surface = topo.face(face).unwrap().surface().clone();
            // The one-shot NURBS trim test rebuilds its trim per point: keep
            // that grid coarse.
            let steps = if matches!(surface, FaceSurface::Nurbs(_)) {
                3
            } else {
                13
            };
            for i in 0..=steps {
                for j in 0..=steps {
                    let (s, t) = (
                        f64::from(i) / f64::from(steps),
                        f64::from(j) / f64::from(steps),
                    );
                    let p = match &surface {
                        FaceSurface::Sphere(sphere) => {
                            let (lon, lat) = (2.0 * PI * s, PI * (t - 0.5));
                            sphere.center()
                                + Vec3::new(lat.cos() * lon.cos(), lat.cos() * lon.sin(), lat.sin())
                                    * sphere.radius()
                        }
                        FaceSurface::Torus(torus) => torus.evaluate(2.0 * PI * s, 2.0 * PI * t),
                        FaceSurface::Cylinder(cylinder) => {
                            cylinder.evaluate(2.0 * PI * s, 12.0 * t - 1.0)
                        }
                        FaceSurface::Nurbs(nurbs) => {
                            let ((u0, u1), (v0, v1)) = (nurbs.domain_u(), nurbs.domain_v());
                            nurbs.evaluate(u0 + (u1 - u0) * s, v0 + (v1 - v0) * t)
                        }
                        FaceSurface::Plane { .. } => {
                            Point3::new(10.0 * s - 5.0, 10.0 * t - 5.0, 0.0)
                                + Vec3::new(0.0, 0.0, 10.0) * f64::from(i % 2)
                        }
                        FaceSurface::Cone(_) => unreachable!("no cone faces here"),
                    };
                    let one_shot = surface_point_in_face(&topo, face, p).map_err(|e| e.to_string());
                    let reused = prepared.contains(p).map_err(|e| e.to_string());
                    assert_eq!(reused, one_shot, "face {} at {p:?}", face.index());
                    compared += 1;
                }
            }
        }
    }
    assert!(compared > 1000, "{compared} points compared");
}

/// A refusal that is not a miss: the halves were built, or the split refused
/// by name as an interior crossing.
fn assert_crossing(result: Result<SplitResult, OperationsError>, what: &str) {
    match result {
        Ok(_) => {}
        Err(OperationsError::Unsupported { reason, .. }) => assert!(
            reason.contains("crosses the interior of face"),
            "{what}: refused for another reason: {reason}"
        ),
        Err(err) => panic!("{what}: a crossing must not read as {err:?}"),
    }
}

/// A plane a hair off the axis of a large torus: the normal's radial part
/// `|w| = 5e-13` is below the Gauss-map helper's cutoff, but `R·|w| = 5e-7`
/// on an `R = 1e6` torus is past the tolerance, so a plane 2e-7 under the top
/// ring still crosses (by 3e-7). The rings are read where the normal points.
#[test]
fn near_axial_plane_on_a_large_torus_still_crosses() {
    let mut topo = Topology::new();
    let torus = make_torus(&mut topo, 1.0e6, 3.0, 16).unwrap();
    let normal = Vec3::new(0.0, 5.0e-13, 1.0);
    let crossing = split(&mut topo, torus, Point3::new(0.0, 0.0, 3.000_000_2), normal);
    assert_crossing(
        crossing,
        "R1e6 r3 torus, near-axial plane under the top ring",
    );
    let missing = split(&mut topo, torus, Point3::new(0.0, 0.0, 3.000_001), normal);
    assert!(
        matches!(missing, Err(OperationsError::InvalidInput { .. })),
        "a plane 1e-6 over the reach must stay a miss, got {missing:?}"
    );
}

/// A unit cube on `z ∈ [-1, 0]` whose top is the bicubic Bezier roof
/// `z = q(u)·q(v)`, `q(t) = 27/4·t(1−t)²`: zero along its four straight
/// edges, reaching exactly 1 at `(1/3, 1/3)`. Volume `1 + (9/16)²`.
fn bump_roofed_cube(topo: &mut Topology) -> SolidId {
    use remus_math::nurbs::surface::NurbsSurface;
    use remus_topology::explorer::solid_faces;
    use remus_topology::face::FaceSurface;

    let cube = make_box(topo, 1.0, 1.0, 1.0).unwrap();
    transform_solid(topo, cube, &Mat4::translation(0.0, 0.0, -1.0)).unwrap();
    let top = solid_faces(topo, cube)
        .unwrap()
        .into_iter()
        .find(|&f| {
            topo.face(f)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.z() > 0.5)
        })
        .expect("top face");
    let reversed = topo.face(top).unwrap().is_reversed();
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let net: Vec<Vec<Point3>> = (0..4)
        .map(|i| {
            (0..4)
                .map(|j| {
                    let z = if i == 1 && j == 1 { 81.0 / 16.0 } else { 0.0 };
                    let (x, y) = (f64::from(i) / 3.0, f64::from(j) / 3.0);
                    // A reversed face flips the carrier's normal: swap u and
                    // v so the roof's effective normal still points up.
                    if reversed {
                        Point3::new(y, x, z)
                    } else {
                        Point3::new(x, y, z)
                    }
                })
                .collect()
        })
        .collect();
    let roof = NurbsSurface::new(3, 3, knots.clone(), knots, net, vec![vec![1.0; 4]; 4]).unwrap();
    topo.face_mut(top)
        .unwrap()
        .set_surface(FaceSurface::Nurbs(roof));
    assert_relative("bump-roofed cube", volume(topo, cube), 1.0 + 81.0 / 256.0);
    cube
}

/// The roof crosses `z = 0.999999` by 1e-6 at a peak between every dyadic
/// witness the patch search reaches; the witnesses are polished onto the
/// peak. The tangent plane at the peak and a plane past it stay misses.
#[test]
fn nurbs_roof_crossed_below_a_narrow_peak_is_a_crossing() {
    for (z, crossing) in [
        (0.5, true),
        (0.999_999, true),
        (1.0, false),
        (1.000_001, false),
    ] {
        let mut topo = Topology::new();
        let cube = bump_roofed_cube(&mut topo);
        let result = split(&mut topo, cube, Point3::new(0.0, 0.0, z), Z);
        if crossing {
            assert_crossing(result, &format!("roof at z = {z}"));
        } else {
            assert!(
                matches!(result, Err(OperationsError::InvalidInput { .. })),
                "roof at z = {z} must stay a miss, got {result:?}"
            );
        }
    }
}

/// A torus whose carrier axis is the unnormalised `(0, 0, 49)`, cut square to
/// that axis below the tube: the normalised normal and axis agree to the last
/// bit but leave a rounding residual along the axis. The ring probes must
/// stay on the surface, so the miss stays a miss.
#[test]
fn exactly_axial_miss_on_a_torus_with_a_scaled_axis_stays_a_miss() {
    use remus_math::surfaces::ToroidalSurface;
    use remus_topology::explorer::solid_faces;
    use remus_topology::face::FaceSurface;

    let axis = Vec3::new(0.0, 0.0, 49.0);
    for z in [-2.0, 2.0] {
        let mut topo = Topology::new();
        let torus = make_torus(&mut topo, 10.0, 1.0, 16).unwrap();
        let face = solid_faces(&topo, torus).unwrap()[0];
        let carrier =
            ToroidalSurface::with_axis(Point3::new(0.0, 0.0, 0.0), 10.0, 1.0, axis).unwrap();
        topo.face_mut(face)
            .unwrap()
            .set_surface(FaceSurface::Torus(carrier));
        let result = split(&mut topo, torus, Point3::new(0.0, 0.0, z), axis);
        assert!(
            matches!(result, Err(OperationsError::InvalidInput { .. })),
            "torus missed at z = {z} along its scaled axis, got {result:?}"
        );
        assert_crossing(
            split(&mut topo, torus, Point3::new(0.0, 0.0, 0.5), axis),
            "torus at z = 0.5 along its scaled axis",
        );
    }
}
