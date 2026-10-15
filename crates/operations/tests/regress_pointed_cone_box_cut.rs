//! Regression: an exact box − pointed-cone cut returned the wrong volume.
//!
//! Box 2×3×4 minus `make_cone(0.5, 0, 4)` (volume π/3) placed so its base cap
//! lies inside the box and its apex pokes out through the top face. The
//! removed volume is the frustum below the top face, 7/8 of the cone. Under
//! `ExactOnly` the cut returned `Ok`/Exact reading `24 − π/24` (the apex piece
//! outside the box) with one same-sense shared edge. The apex-down twin did
//! the same.
//!
//! Two defects, both about the cone's doubled seam to its apex:
//!
//! 1. The face splitter's band path (`split_periodic_face_into_bands`) only
//!    took laterals with two rim circles. A pointed cone has one rim and a
//!    seam run to the apex and back, so it declined, and the generic wire
//!    builder mistraced the doubled seam: it dropped the rim-side band and
//!    emitted one sub-face with no rim edge. Assembly then dropped the
//!    orphaned base cap.
//! 2. Shell assembly (`excise_out_and_back_spurs`) and the operations heal
//!    (`remove_wire_spurs`) stripped the seam pair as an out-and-back spur.
//!    The two uses are the u = 0 and u = 2π sides of the seam meeting at the
//!    apex, so a kept band reaching the apex was left bounded by its rim
//!    alone: with the apex inside the box the shell came back open and
//!    uncarved, which is why those placements refused.
//!
//! The oracle is the closed form: the part of a cone inside the slab
//! `0 ≤ z ≤ 4` is the whole cone less the similar cone cut off past the
//! plane, so it scales with the cube of the apex distance.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_check::validate::{CheckId, Severity, ValidateOptions, validate_solid};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cone};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const R: f64 = 0.5;
const H: f64 = 4.0;
/// Cone axis position: inside the 2×3 footprint, clear of every side wall.
const AXIS: (f64, f64) = (1.0, 1.5);

/// π r² h / 3 = π/3.
fn cone_volume() -> f64 {
    PI * R * R * H / 3.0
}

/// Cut the 2×3×4 box by a pointed cone of radius `R`, height `H`.
///
/// `apex_up` picks `make_cone(R, 0, H)` (base at the bottom) or
/// `make_cone(0, R, H)` (apex at the bottom); `z0` is the cone's lowest z.
fn cut(apex_up: bool, z0: f64) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let cone = if apex_up {
        make_cone(&mut topo, R, 0.0, H).unwrap()
    } else {
        make_cone(&mut topo, 0.0, R, H).unwrap()
    };
    transform_solid(&mut topo, cone, &Mat4::translation(AXIS.0, AXIS.1, z0)).unwrap();
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let outcome = boolean_with_context(&mut topo, BooleanOp::Cut, block, cone, &context)
        .expect("exact cut must succeed");
    assert_eq!(outcome.quality, BooleanQuality::Exact);
    (topo, outcome.solid)
}

/// Fraction of the cone's volume inside the slab `0 ≤ z ≤ 4`.
///
/// The part of the cone between its apex and a plane at distance `d` from it
/// is a similar cone, `(d / H)³` of the whole.
fn inside_fraction(apex_z: f64, base_z: f64) -> f64 {
    let (lo, hi) = (apex_z.min(base_z), apex_z.max(base_z));
    let tip = |z: f64| ((z.clamp(lo, hi) - apex_z).abs() / H).powi(3);
    (tip(4.0) - tip(0.0)).abs()
}

/// Cone radius at height `z` for a cone whose apex is at `apex_z`.
fn radius_at(apex_z: f64, z: f64) -> f64 {
    R * (z - apex_z).abs() / H
}

/// Check `solid` against the closed form for a cone with its apex at
/// `apex_z` and its base at `base_z`.
fn assert_cut(topo: &Topology, solid: SolidId, apex_z: f64, base_z: f64, what: &str) {
    let removed_fraction = inside_fraction(apex_z, base_z);
    let expected = 24.0 - removed_fraction * cone_volume();
    let volume = solid_volume(topo, solid, 1e-3).unwrap();
    assert!(
        (volume - expected).abs() < 1e-9,
        "{what}: volume {volume:.15} != closed form {expected:.15} (error {:.3e})",
        volume - expected
    );

    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    assert_eq!(
        report.error_count(),
        0,
        "{what}: validation errors {:?}",
        report.issues
    );
    // The only finding allowed is `check_euler`'s documented blind spot: the
    // top/bottom face carries the cone's opening as an inner wire, which its
    // V − E + F count cannot see.
    for issue in &report.issues {
        assert!(
            issue.check == CheckId::SolidEulerCharacteristic && issue.severity == Severity::Warning,
            "{what}: unexpected issue {issue:?}"
        );
    }
    assert!(
        report
            .issues
            .iter()
            .all(|i| i.check != CheckId::FaceOrientationConsistency),
        "{what}: misoriented shared edge"
    );

    let mesh = tessellate_solid(topo, solid, 1e-3).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0, "{what}: open mesh");
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "{what}: non-manifold mesh"
    );

    let probe = |p: Point3| {
        classify_point(topo, solid, p, &ClassifyOptions::default())
            .unwrap_or_else(|e| panic!("{what}: classify {p:?}: {e}"))
    };
    let (cx, cy) = AXIS;
    // Box material well clear of the cone stays.
    assert_eq!(
        probe(Point3::new(0.2, 0.3, 2.0)),
        PointClassification::Inside,
        "{what}"
    );
    assert_eq!(
        probe(Point3::new(1.8, 2.7, 0.5)),
        PointClassification::Inside,
        "{what}"
    );
    // Outside the box stays outside, including next to the protruding part.
    assert_eq!(
        probe(Point3::new(cx, cy, 4.5)),
        PointClassification::Outside,
        "{what}"
    );
    assert_eq!(
        probe(Point3::new(cx, cy, -0.5)),
        PointClassification::Outside,
        "{what}"
    );
    // Inside the box, on and off the cone's axis, at heights the cone spans.
    // Heights off the 0.5 grid the apex and cap planes sit on.
    for z in [0.3, 1.1, 2.6, 3.7, 3.95] {
        let r = radius_at(apex_z, z);
        let in_cone = z > apex_z.min(base_z) && z < apex_z.max(base_z);
        if in_cone {
            assert_eq!(
                probe(Point3::new(cx + 0.5 * r, cy, z)),
                PointClassification::Outside,
                "{what}: cone interior at z={z} kept"
            );
            assert_eq!(
                probe(Point3::new(cx + r + 0.05, cy, z)),
                PointClassification::Inside,
                "{what}: box material beside the cone at z={z} removed"
            );
        } else {
            assert_eq!(
                probe(Point3::new(cx, cy, z)),
                PointClassification::Inside,
                "{what}: material outside the cone's span at z={z} removed"
            );
        }
    }
}

/// The reported case: base cap inside at z = 2, apex out through the top at
/// z = 6. Removes 7/8 of the cone; `main` removed 1/8.
#[test]
fn apex_up_cone_with_base_cap_inside_box() {
    let (topo, solid) = cut(true, 2.0);
    assert!((inside_fraction(6.0, 2.0) - 0.875).abs() < 1e-15);
    assert_cut(&topo, solid, 6.0, 2.0, "apex-up, base z=2");
}

/// The apex-down twin: base cap inside at z = 2, apex out through the bottom
/// at z = −2.
#[test]
fn apex_down_cone_with_base_cap_inside_box() {
    let (topo, solid) = cut(false, -2.0);
    assert!((inside_fraction(-2.0, 2.0) - 0.875).abs() < 1e-15);
    assert_cut(&topo, solid, -2.0, 2.0, "apex-down, base z=2");
}

/// The rest of the base-inside family, both orientations: the base cap at
/// z = 0.5 … 3.5 and the apex out through the top (apex up) or the bottom
/// (apex down).
#[test]
fn base_cap_inside_family() {
    for k in 1..=7 {
        let base = 0.5 * f64::from(k);
        let (topo, solid) = cut(true, base);
        assert_cut(
            &topo,
            solid,
            base + H,
            base,
            &format!("apex-up, base z={base}"),
        );
        let (topo, solid) = cut(false, base - H);
        assert_cut(
            &topo,
            solid,
            base - H,
            base,
            &format!("apex-down, base z={base}"),
        );
    }
}

/// Apex inside the box, base cap out through the bottom: the kept cone band
/// reaches the apex. Shell assembly used to excise its seam pair, leaving an
/// open, uncarved shell, so `ExactOnly` refused. Removes (3.5/4)³ of the cone.
///
/// Apex-up only: on `main` the apex-down primitive itself carries a
/// misoriented rim (`make_cone(0, r, h)`, fixed separately), which an
/// apex-inside cut keeps.
#[test]
fn apex_up_cone_with_apex_inside_box() {
    for apex in [0.5, 2.0, 3.5] {
        let (topo, solid) = cut(true, apex - H);
        assert_cut(
            &topo,
            solid,
            apex,
            apex - H,
            &format!("apex-up, apex z={apex}"),
        );
    }
}

/// A genuine seam reaches the singularity; an ordinary retraced excursion
/// ending several vertex tolerances before it is still removable debris.
#[test]
fn healing_removes_a_near_apex_spur_without_stripping_the_real_seam() {
    use remus_operations::heal::remove_wire_spurs;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::FaceSurface;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    for apex_up in [true, false] {
        for rotation in 0..5 {
            let mut topo = Topology::new();
            let solid = if apex_up {
                make_cone(&mut topo, R, 0.0, H).unwrap()
            } else {
                make_cone(&mut topo, 0.0, R, H).unwrap()
            };
            let face = remus_topology::explorer::solid_faces(&topo, solid)
                .unwrap()
                .into_iter()
                .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cone(_)))
                .unwrap();
            let FaceSurface::Cone(cone) = topo.face(face).unwrap().surface() else {
                panic!("cone")
            };
            let apex = cone.apex();
            let old = topo.face(face).unwrap().outer_wire();
            let mut uses = topo.wire(old).unwrap().edges().to_vec();
            let rim_index = uses
                .iter()
                .position(|use_| {
                    matches!(
                        topo.edge(use_.edge()).unwrap().curve(),
                        EdgeCurve::Circle(_)
                    )
                })
                .unwrap();
            uses.rotate_left(rim_index);
            let rim = topo.edge(uses[0].edge()).unwrap().start();
            let rim_point = topo.vertex(rim).unwrap().point();
            let direction = rim_point - apex;
            let near = apex + direction * (5e-6 / direction.length());
            let near = topo.add_vertex(Vertex::new(near, 1e-7));
            let spur = topo.add_edge(Edge::new(rim, near, EdgeCurve::Line));
            uses.splice(
                0..0,
                [
                    OrientedEdge::new(spur, true),
                    OrientedEdge::new(spur, false),
                ],
            );
            uses.rotate_left(rotation);
            let wire = topo.add_wire(Wire::new(uses, true).unwrap());
            topo.set_face_boundary_wires(face, wire, Vec::new())
                .unwrap();
            assert_eq!(
                remove_wire_spurs(&mut topo, solid).unwrap(),
                2,
                "apex_up={apex_up}, rotation={rotation}"
            );
            let wire = topo.face(face).unwrap().outer_wire();
            assert_eq!(topo.wire(wire).unwrap().edges().len(), 3);
            assert!(
                topo.wire(wire)
                    .unwrap()
                    .edges()
                    .iter()
                    .all(|use_| use_.edge() != spur)
            );
        }
    }
}

/// The integrated apex-down primitive repair lets the opposite apex-inside
/// family retain the same boundary and material contracts.
#[test]
fn apex_down_cone_with_apex_inside_box() {
    for apex in [0.5, 2.0, 3.5] {
        let (topo, solid) = cut(false, apex);
        assert_cut(
            &topo,
            solid,
            apex,
            apex + H,
            &format!("apex-down, apex z={apex}"),
        );
    }
}

/// The cavity cone's apex coincides with the body's integration reference.
/// Its volume flux is exactly zero, while its area and moments are nonzero.
#[test]
fn apex_inside_mass_properties_converge_without_cancelling_cone_flux() {
    use remus_check::properties::face_integrator::integrate_face_about;
    use remus_check::properties::{PropertiesOptions, integration_reference, solid_properties};
    use remus_math::vec::Vec3;
    use remus_topology::face::FaceSurface;

    let close = |actual: f64, expected: f64, scale: f64| {
        assert!(
            (actual - expected).abs() <= 1e-9 * scale.abs(),
            "{actual} != {expected}"
        );
    };
    for up in [true, false] {
        for apex in [0.5, 2.0, 3.5] {
            let height = if up { apex } else { 4.0 - apex };
            let radius = height / 8.0;
            let removed = PI * radius * radius * height / 3.0;
            let remaining = 24.0 - removed;
            let cone_z = if up { height / 4.0 } else { 4.0 - height / 4.0 };
            let displacement = -removed * (cone_z - 2.0) / remaining;
            let transverse = -removed
                * (0.15 * radius * radius + 0.0375 * height * height + (cone_z - 2.0).powi(2))
                - remaining * displacement.powi(2);
            let inertia = [
                50.0 + transverse,
                40.0 + transverse,
                26.0 - 0.3 * radius * radius * removed,
            ];
            let area = 52.0 - PI * radius * radius + PI * radius * height.hypot(radius);
            for scale in [0.5_f64, 1.0, 2.0] {
                for offset in [Vec3::new(0.0, 0.0, 0.0), Vec3::new(37.0, -12.0, 9.0)] {
                    let (mut topo, solid) = cut(up, if up { apex - H } else { apex });
                    let matrix = Mat4::translation(offset.x(), offset.y(), offset.z())
                        * Mat4::scale(scale, scale, scale);
                    transform_solid(&mut topo, solid, &matrix).unwrap();
                    let options = PropertiesOptions::default();
                    let props = solid_properties(&topo, solid, &options)
                        .expect("apex-relative cone flux converges");
                    close(
                        props.mass,
                        remaining * scale.powi(3),
                        remaining * scale.powi(3),
                    );
                    let z = 2.0 + displacement;
                    let center = matrix.mul_point(Point3::new(1.0, 1.5, z));
                    close((props.center - center).length(), 0.0, scale);
                    for (actual, expected) in props.inertia[..3].iter().zip(inertia) {
                        close(*actual, expected * scale.powi(5), expected * scale.powi(5));
                    }
                    for &product in &props.inertia[3..] {
                        close(product, 0.0, 50.0 * scale.powi(5));
                    }
                    let reference = integration_reference(&topo, solid).unwrap();
                    let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
                    let measured_area: f64 = faces
                        .iter()
                        .map(|&face| {
                            integrate_face_about(&topo, face, &options, reference)
                                .unwrap()
                                .area
                        })
                        .sum();
                    close(measured_area, area * scale.powi(2), area * scale.powi(2));
                    let cone = faces
                        .into_iter()
                        .find(|&face| {
                            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cone(_))
                        })
                        .unwrap();
                    let FaceSurface::Cone(carrier) = topo.face(cone).unwrap().surface() else {
                        panic!("cone")
                    };
                    for delta in [
                        Vec3::new(0.0, 0.0, 0.0),
                        Vec3::new(0.0, 0.0, 1e-9 * scale),
                        Vec3::new(0.125 * scale, -0.25 * scale, 0.5 * scale),
                    ] {
                        let reference = carrier.apex() + delta;
                        let options = PropertiesOptions {
                            adaptive_eps: 1e-9,
                            max_depth: 12,
                            ..options
                        };
                        let contribution =
                            integrate_face_about(&topo, cone, &options, reference).unwrap();
                        let expected =
                            (reference.z() - carrier.apex().z()) * PI * (radius * scale).powi(2)
                                / 3.0
                                * if up { 1.0 } else { -1.0 };
                        close(contribution.volume, expected, scale.powi(3));
                        close(
                            contribution.area,
                            PI * radius * height.hypot(radius) * scale.powi(2),
                            scale.powi(2),
                        );
                    }
                }
            }
        }
    }
}
