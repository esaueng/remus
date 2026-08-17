//! Rolling-ball (arc) joint offsets, checked against the Minkowski closed form.
//!
//! Every figure here is derived, not recorded. Offsetting a convex polyhedron
//! outward by `d` with rounded joints is the Minkowski sum with a ball of
//! radius `d`, so a box's rounded offset decomposes into the box, six slabs,
//! twelve quarter-cylinders and eight ball octants — the Steiner formula.
//!
//! The failure this file exists to catch is a silent fall back to the mitred
//! joint. A mitred `2×2×2` box offset by `0.5` is the `3×3×3` box, 27.0; the
//! rounded one is 25.2359..., 7% smaller. Each volume assertion names the
//! mitred figure so a fallback cannot pass unnoticed.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_check::properties::{PropertiesOptions, solid_area, solid_volume};
use remus_check::validate::{Severity, ValidateOptions, validate_solid};
use remus_math::mat::Mat4;
use remus_offset::{JointType, OffsetOptions, offset_solid};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

fn arc_opts() -> OffsetOptions {
    OffsetOptions {
        joint: JointType::Arc,
        remove_self_intersections: false,
        ..Default::default()
    }
}

fn mitre_opts() -> OffsetOptions {
    OffsetOptions {
        joint: JointType::Intersection,
        remove_self_intersections: false,
        ..Default::default()
    }
}

/// Volume of `box(a, b, c) ⊕ ball(d)`, the body a rolling-ball offset
/// produces. Steiner's decomposition:
///
/// * the box itself: `abc`
/// * one slab of height `d` on each of the six faces: `2(ab + bc + ca)·d`
/// * one quarter-cylinder of radius `d` along each of the twelve edges:
///   `4 · ¼π d² (a + b + c) = π(a + b + c)d²`
/// * one ball octant at each of the eight corners, which reassemble into a
///   whole ball: `(4/3)π d³`
fn minkowski_box_volume(a: f64, b: f64, c: f64, d: f64) -> f64 {
    a * b * c
        + 2.0 * (a * b + b * c + c * a) * d
        + PI * (a + b + c) * d * d
        + 4.0 / 3.0 * PI * d * d * d
}

/// Surface area of the same body: the six faces keep their area, each edge
/// grows a quarter-cylinder wall `¼·2πd·len` and a box has four edges of each
/// length, and the eight corner patches reassemble into one whole sphere.
fn minkowski_box_area(a: f64, b: f64, c: f64, d: f64) -> f64 {
    2.0 * (a * b + b * c + c * a) + 2.0 * PI * d * (a + b + c) + 4.0 * PI * d * d
}

/// Volume of the mitred offset of the same box — the answer a fallback to
/// `JointType::Intersection` would produce.
fn mitred_box_volume(a: f64, b: f64, c: f64, d: f64) -> f64 {
    (a + 2.0 * d) * (b + 2.0 * d) * (c + 2.0 * d)
}

/// Volume by per-face divergence-theorem quadrature on the analytic surfaces.
///
/// The sign is kept, not taken away. The construction's own
/// `check_oriented_manifold` proves the joint faces agree with each other,
/// which a skin turned entirely inside out would satisfy just as well; only the
/// sign of the boundary integral says they agree with the outside.
fn measured_volume(topo: &Topology, solid: SolidId) -> f64 {
    let volume = solid_volume(topo, solid, &PropertiesOptions::default()).unwrap();
    assert!(
        volume > 0.0,
        "the rounded skin measures {volume:.10}; a negative boundary integral means every \
         face of it is oriented inward"
    );
    volume
}

/// How close a measurement of a rounded body can get to its closed form.
///
/// The geometry is exact — every patch is an analytic quadric and every corner
/// sits on all of its surfaces to the last bit — and the translated faces and
/// the edge cylinders measure exact, because their boundaries are straight
/// lines and constant-`v` arcs in their own parameterisations. The residual is
/// entirely the corner patches: a spherical triangle's sides are great-circle
/// arcs that bow in `(u, v)`, and the integrator trims on a chord polygon of
/// them. At its 128 samples per arc that leaves the corner family ~1.5e-5 low
/// and the whole body ~1e-6 low, inscribed rather than scattered. A mitred
/// fallback is 7e-2 out — four orders of magnitude away — so the threshold
/// still separates the two decisively.
const CLOSED_FORM_TOLERANCE: f64 = 1e-5;

fn assert_watertight(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    let errors: Vec<_> = report
        .issues
        .iter()
        .filter(|issue| issue.severity == Severity::Error)
        .map(|issue| format!("{:?}: {}", issue.check, issue.description))
        .collect();
    assert!(
        errors.is_empty(),
        "rounded offset must be valid, got: {errors:?}"
    );
}

// ── The Minkowski closed form ──────────────────────────────────

#[test]
fn a_rounded_box_is_the_minkowski_sum_with_a_ball() {
    let (a, b, c, d) = (2.0, 2.0, 2.0, 0.5);
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, a, b, c).unwrap();
    let result = offset_solid(&mut topo, solid, d, arc_opts()).unwrap();

    let expected = minkowski_box_volume(a, b, c, d);
    let measured = measured_volume(&topo, result);
    let error = ((measured - expected) / expected).abs();
    assert!(
        error < CLOSED_FORM_TOLERANCE,
        "rounded volume {measured:.10} should be the Minkowski sum {expected:.10} \
         (relative error {error:.3e}); a mitred fallback would read {:.10}",
        mitred_box_volume(a, b, c, d)
    );
}

#[test]
fn a_rounded_rectangular_box_is_the_minkowski_sum_with_a_ball() {
    let (a, b, c, d) = (3.0, 5.0, 7.0, 0.75);
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, a, b, c).unwrap();
    let result = offset_solid(&mut topo, solid, d, arc_opts()).unwrap();

    let expected = minkowski_box_volume(a, b, c, d);
    let measured = measured_volume(&topo, result);
    let error = ((measured - expected) / expected).abs();
    assert!(
        error < CLOSED_FORM_TOLERANCE,
        "rounded volume {measured:.10} should be {expected:.10} (relative error {error:.3e}); \
         a mitred fallback would read {:.10}",
        mitred_box_volume(a, b, c, d)
    );
}

#[test]
fn a_rounded_box_has_the_minkowski_surface_area() {
    let (a, b, c, d) = (2.0, 3.0, 4.0, 0.5);
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, a, b, c).unwrap();
    let result = offset_solid(&mut topo, solid, d, arc_opts()).unwrap();

    let expected = minkowski_box_area(a, b, c, d);
    let measured = solid_area(&topo, result, &PropertiesOptions::default()).unwrap();
    let error = ((measured - expected) / expected).abs();
    assert!(
        error < CLOSED_FORM_TOLERANCE,
        "rounded area {measured:.10} should be {expected:.10} (relative error {error:.3e})"
    );
}

#[test]
fn a_rounded_box_carries_the_minkowski_face_partition() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let result = offset_solid(&mut topo, solid, 0.5, arc_opts()).unwrap();

    let shell = topo
        .shell(topo.solid(result).unwrap().outer_shell())
        .unwrap();
    let mut partition = (0, 0, 0);
    for &face_id in shell.faces() {
        match topo.face(face_id).unwrap().surface() {
            FaceSurface::Plane { .. } => partition.0 += 1,
            FaceSurface::Cylinder(_) => partition.1 += 1,
            FaceSurface::Sphere(_) => partition.2 += 1,
            other => panic!("unexpected joint surface {other:?}"),
        }
    }
    assert_eq!(
        partition,
        (6, 12, 8),
        "a rounded box is 6 translated faces, 12 edge cylinders and 8 corner ball octants; \
         the mitred answer would be (6, 0, 0)"
    );
}

#[test]
fn a_rounded_box_is_a_closed_two_manifold() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let result = offset_solid(&mut topo, solid, 0.4, arc_opts()).unwrap();
    assert_watertight(&topo, result);
}

/// The geometry itself, independent of any integrator.
///
/// Every joint surface must have exactly the offset radius, and every vertex
/// of every wire must lie on the surface of the face that wire bounds — three
/// surfaces at a corner, all agreeing to the last few bits. This is what says
/// the patches actually meet rather than merely come close, and no chording
/// tolerance enters it.
#[test]
fn every_joint_patch_carries_the_offset_radius_and_holds_its_own_corners() {
    let d = 0.4;
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let result = offset_solid(&mut topo, solid, d, arc_opts()).unwrap();

    let shell = topo
        .shell(topo.solid(result).unwrap().outer_shell())
        .unwrap();
    for &face_id in shell.faces() {
        let face = topo.face(face_id).unwrap();
        match face.surface() {
            FaceSurface::Cylinder(c) => assert!(
                (c.radius() - d).abs() < 1e-15,
                "edge joint radius {} should be the offset distance {d}",
                c.radius()
            ),
            FaceSurface::Sphere(s) => assert!(
                (s.radius() - d).abs() < 1e-15,
                "corner joint radius {} should be the offset distance {d}",
                s.radius()
            ),
            FaceSurface::Plane { .. } => {}
            other => panic!("unexpected joint surface {other:?}"),
        }
        for oriented in topo.wire(face.outer_wire()).unwrap().edges() {
            let edge = topo.edge(oriented.edge()).unwrap();
            for vertex in [edge.start(), edge.end()] {
                let p = topo.vertex(vertex).unwrap().point();
                let deviation = match face.surface() {
                    FaceSurface::Plane { normal, d: dist } => {
                        (normal.x() * p.x() + normal.y() * p.y() + normal.z() * p.z() - dist).abs()
                    }
                    FaceSurface::Cylinder(c) => {
                        let rel = p - c.origin();
                        let radial = rel - c.axis() * c.axis().dot(rel);
                        (radial.length() - c.radius()).abs()
                    }
                    FaceSurface::Sphere(s) => ((p - s.center()).length() - s.radius()).abs(),
                    other => panic!("unexpected joint surface {other:?}"),
                };
                assert!(
                    deviation < 1e-13,
                    "a corner of face {} sits {deviation:e} off its own surface",
                    face_id.index()
                );
            }
        }
    }
}

/// The public measurement API has to reach the same answer.
///
/// `remus-offset` measures through `remus-check`'s per-face quadrature,
/// but callers reach a rounded body through `remus-operations`, which routes
/// a solid with no bored quadric to a tessellated boundary integral. An
/// inscribed mesh under-counts a convex patch, so that route is held to a
/// looser figure than the quadrature — but to the same closed form, and from
/// below.
#[test]
fn the_operations_measure_api_reaches_the_same_closed_form() {
    let (a, b, c, d) = (3.0, 5.0, 7.0, 0.75);
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, a, b, c).unwrap();
    let result = offset_solid(&mut topo, solid, d, arc_opts()).unwrap();

    let expected = minkowski_box_volume(a, b, c, d);
    let tessellated = remus_operations::measure::solid_volume(&topo, result, 0.01).unwrap();
    let mass = remus_operations::measure::mass_properties(&topo, result)
        .unwrap()
        .mass;

    assert!(
        ((tessellated - expected) / expected).abs() < 1e-3,
        "tessellated volume {tessellated:.9} should be {expected:.9}"
    );
    assert!(
        tessellated <= expected,
        "an inscribed mesh cannot exceed the convex body it approximates: \
         {tessellated:.9} vs {expected:.9}"
    );
    assert!(
        ((mass - expected) / expected).abs() < CLOSED_FORM_TOLERANCE,
        "mass_properties {mass:.9} should be {expected:.9}"
    );
}

#[test]
fn a_rounded_offset_holds_its_closed_form_across_six_decades_of_scale() {
    for scale in [1.0_f64, 1000.0, 0.001] {
        let (a, b, c, d) = (2.0 * scale, 3.0 * scale, 4.0 * scale, 0.5 * scale);
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, a, b, c).unwrap();
        let result = offset_solid(&mut topo, solid, d, arc_opts()).unwrap();
        assert_watertight(&topo, result);

        let expected = minkowski_box_volume(a, b, c, d);
        let measured = measured_volume(&topo, result);
        let error = ((measured - expected) / expected).abs();
        assert!(
            error < CLOSED_FORM_TOLERANCE,
            "at {scale}x the rounded volume {measured:.10} should be {expected:.10} \
             (relative error {error:.3e})"
        );
    }
}

#[test]
fn rounding_removes_exactly_the_corner_material_the_mitre_keeps() {
    let (a, b, c, d) = (2.0, 2.0, 2.0, 0.5);
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, a, b, c).unwrap();
    let rounded = offset_solid(&mut topo, solid, d, arc_opts()).unwrap();
    let mitred = offset_solid(&mut topo, solid, d, mitre_opts()).unwrap();

    let gap = measured_volume(&topo, mitred) - measured_volume(&topo, rounded);
    let expected = mitred_box_volume(a, b, c, d) - minkowski_box_volume(a, b, c, d);
    // Normalised by the body, not by the gap: the gap is a difference of two
    // measurements, so scaling its own residual by the gap would report the
    // corner patches' chord error amplified by however small the difference
    // happens to be, which says nothing about either measurement.
    let residual = (gap - expected).abs() / minkowski_box_volume(a, b, c, d);
    assert!(
        residual < CLOSED_FORM_TOLERANCE,
        "the mitre keeps {gap:.10} more material than the rounded joint; closed form \
         {expected:.10} (residual {residual:.3e} of the body)"
    );
}

#[test]
fn a_rounded_offset_is_translation_invariant() {
    let (a, b, c, d) = (2.0, 3.0, 4.0, 0.6);
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, a, b, c).unwrap();
    transform_solid(&mut topo, solid, &Mat4::translation(-17.0, 41.0, -3.5)).unwrap();
    let result = offset_solid(&mut topo, solid, d, arc_opts()).unwrap();

    let expected = minkowski_box_volume(a, b, c, d);
    let measured = measured_volume(&topo, result);
    assert!(
        ((measured - expected) / expected).abs() < CLOSED_FORM_TOLERANCE,
        "moving the box off the origin must not change its rounded volume: \
         {measured:.10} vs {expected:.10}"
    );
}

// ── What still refuses, and says so ────────────────────────────

#[test]
fn an_inward_rounded_offset_refuses() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    let error = offset_solid(&mut topo, solid, -0.5, arc_opts()).unwrap_err();
    assert!(
        error.to_string().contains("outward offset only"),
        "an inward rounded offset must refuse, got {error}"
    );
}

#[test]
fn a_curved_source_face_refuses_rather_than_mitres() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 2.0, 5.0).unwrap();
    let error = offset_solid(&mut topo, solid, 0.5, arc_opts()).unwrap_err();
    assert!(
        error.to_string().contains("planar faces only"),
        "a curved source face must refuse, got {error}"
    );
}

#[test]
fn a_concave_edge_refuses_rather_than_mitres() {
    // An L-prism: a box with a corner cut away, leaving concave edges the
    // rolling ball cannot round.
    let mut topo = Topology::new();
    let big = make_box(&mut topo, 4.0, 4.0, 2.0).unwrap();
    let notch = make_box(&mut topo, 3.0, 3.0, 4.0).unwrap();
    transform_solid(&mut topo, notch, &Mat4::translation(2.0, 2.0, -1.0)).unwrap();
    let ell = boolean(&mut topo, BooleanOp::Cut, big, notch).unwrap();

    let error = offset_solid(&mut topo, ell, 0.25, arc_opts()).unwrap_err();
    assert!(
        error.to_string().contains("convex edges only"),
        "a concave edge must refuse rather than be mitred silently, got {error}"
    );
}

#[test]
fn a_cavity_refuses_a_rounded_offset() {
    let mut topo = Topology::new();
    let outer = make_box(&mut topo, 6.0, 6.0, 6.0).unwrap();
    let void = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    transform_solid(&mut topo, void, &Mat4::translation(2.0, 2.0, 2.0)).unwrap();
    let hollow = boolean(&mut topo, BooleanOp::Cut, outer, void).unwrap();
    assert_eq!(
        topo.solid(hollow).unwrap().inner_shells().len(),
        1,
        "fixture must be a solid with exactly one cavity"
    );

    let error = offset_solid(&mut topo, hollow, 0.5, arc_opts()).unwrap_err();
    assert!(
        error.to_string().contains("cavity shells"),
        "a hollow part must refuse a rounded offset for now, got {error}"
    );
}

#[test]
fn excluding_a_face_refuses_a_rounded_offset() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    let faces = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap()
        .faces()
        .to_vec();
    let error =
        remus_offset::thick_solid(&mut topo, solid, 0.5, &faces[..1], arc_opts()).unwrap_err();
    assert!(
        error.to_string().contains("cannot exclude faces"),
        "an open face has no joint to roll into; must refuse, got {error}"
    );
}
