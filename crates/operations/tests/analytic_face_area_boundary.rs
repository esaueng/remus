//! The boundary-reduced area of analytic curved faces
//! (`face_integrator::analytic_face_area`, behind `measure::face_area`)
//! against closed forms and against the trimmed Gauss quadrature it replaces.
//!
//! The quadrature is limited by the chording of its UV trim polygons, so it
//! agrees with the reduction only to that chording; closed forms are matched
//! to rounding. Every face either reduces or falls back to the quadrature
//! bit for bit, and the census records which, so a qualified family cannot
//! silently drop back to the slow path.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout
)]

use std::f64::consts::{PI, TAU};

use remus_check::properties::face_integrator::{analytic_face_area, integrate_face_area};
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::{blend_ops, measure, primitives, transform};
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

/// Relative agreement the order-8 trimmed quadrature reaches against the
/// reduction on these bodies: the chord error of its 128-sample trims.
const QUADRATURE_AGREEMENT: f64 = 2e-4;

/// Relative agreement with a closed form.
const EXACT: f64 = 1e-11;

fn translate(topo: &mut Topology, solid: SolidId, x: f64, y: f64, z: f64) {
    transform::transform_solid(topo, solid, &Mat4::translation(x, y, z)).unwrap();
}

fn is_analytic_curved(topo: &Topology, face: FaceId) -> bool {
    matches!(
        topo.face(face).unwrap().surface(),
        FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_)
    )
}

#[derive(Debug, Default)]
struct Census {
    reduced: usize,
    fallback: usize,
    max_relative_difference: f64,
}

/// Compares every analytic curved face of `solid`: the reduction against the
/// quadrature, and `measure::face_area` against the reduction.
fn census(topo: &Topology, solid: SolidId) -> Census {
    let mut census = Census::default();
    for face in solid_faces(topo, solid).unwrap() {
        if !is_analytic_curved(topo, face) {
            continue;
        }
        let quadrature = integrate_face_area(topo, face, 8).unwrap();
        let area = analytic_face_area(topo, face, 8).unwrap();
        assert_eq!(
            measure::face_area(topo, face, 0.1).unwrap().to_bits(),
            area.abs().to_bits(),
            "face_area routes through the reduction"
        );
        if area.to_bits() == quadrature.to_bits() {
            census.fallback += 1;
            continue;
        }
        census.reduced += 1;
        assert!(area > 0.0);
        let relative = (area - quadrature.abs()).abs() / area;
        assert!(
            relative <= QUADRATURE_AGREEMENT,
            "face {} ({}): reduced {area} vs quadrature {quadrature}",
            face.index(),
            topo.face(face).unwrap().surface().type_tag()
        );
        census.max_relative_difference = census.max_relative_difference.max(relative);
    }
    census
}

fn faces_of(topo: &Topology, solid: SolidId, tag: &str) -> Vec<FaceId> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| topo.face(f).unwrap().surface().type_tag() == tag)
        .collect()
}

fn assert_exact(actual: f64, expected: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= EXACT * expected.abs(),
        "{what}: {actual} vs closed form {expected}"
    );
}

#[test]
fn full_cylinder_wall_is_exact() {
    let mut topo = Topology::new();
    let solid = primitives::make_cylinder(&mut topo, 3.0, 7.0).unwrap();
    let [wall] = faces_of(&topo, solid, "cylinder")[..] else {
        panic!("one wall");
    };
    assert_exact(
        analytic_face_area(&topo, wall, 8).unwrap(),
        TAU * 3.0 * 7.0,
        "wall",
    );
    let c = census(&topo, solid);
    assert_eq!((c.reduced, c.fallback), (1, 0), "{c:?}");
}

#[test]
fn reversed_bore_wall_is_exact() {
    let mut topo = Topology::new();
    let block = primitives::make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
    let tool = primitives::make_cylinder(&mut topo, 4.0, 30.0).unwrap();
    translate(&mut topo, tool, 10.0, 10.0, -10.0);
    let solid = boolean(&mut topo, BooleanOp::Cut, block, tool).unwrap();
    let walls = faces_of(&topo, solid, "cylinder");
    assert!(!walls.is_empty());
    let total: f64 = walls
        .iter()
        .map(|&w| {
            assert!(topo.face(w).unwrap().is_reversed());
            analytic_face_area(&topo, w, 8).unwrap()
        })
        .sum();
    assert_exact(total, TAU * 4.0 * 10.0, "bore");
    let c = census(&topo, solid);
    assert_eq!(c.fallback, 0, "{c:?}");
}

#[test]
fn quarter_cylinders_of_an_intersection_are_exact() {
    let mut topo = Topology::new();
    let cylinder = primitives::make_cylinder(&mut topo, 5.0, 8.0).unwrap();
    let quadrant = primitives::make_box(&mut topo, 10.0, 10.0, 20.0).unwrap();
    translate(&mut topo, quadrant, 0.0, 0.0, -5.0);
    let solid = boolean(&mut topo, BooleanOp::Intersect, cylinder, quadrant).unwrap();
    let walls = faces_of(&topo, solid, "cylinder");
    let total: f64 = walls
        .iter()
        .map(|&w| analytic_face_area(&topo, w, 8).unwrap())
        .sum();
    assert_exact(total, 0.25 * TAU * 5.0 * 8.0, "quarter wall");
    let c = census(&topo, solid);
    assert_eq!(c.fallback, 0, "{c:?}");
}

#[test]
fn cone_frustum_and_pointed_cone_are_exact() {
    for (bottom, top) in [(4.0, 1.5), (1.5, 4.0), (3.0, 0.0)] {
        let mut topo = Topology::new();
        let height = 6.0;
        let solid = primitives::make_cone(&mut topo, bottom, top, height).unwrap();
        let [wall] = faces_of(&topo, solid, "cone")[..] else {
            panic!("one cone wall");
        };
        let slant = f64::hypot(height, bottom - top);
        assert_exact(
            analytic_face_area(&topo, wall, 8).unwrap(),
            PI * (bottom + top) * slant,
            &format!("cone {bottom} -> {top}"),
        );
        let c = census(&topo, solid);
        assert_eq!(c.fallback, 0, "{c:?}");
    }
}

#[test]
fn sphere_caps_are_exact() {
    let r = 5.0;
    for cut in [-3.0, -1.0, 1.5, 2.0] {
        // Cut away z >= cut: a cap of height r + cut, area 2 pi r h.
        let mut topo = Topology::new();
        let sphere = primitives::make_sphere(&mut topo, r, 32).unwrap();
        let above = primitives::make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
        translate(&mut topo, above, -10.0, -10.0, cut);
        let Ok(solid) = boolean(&mut topo, BooleanOp::Cut, sphere, above) else {
            println!("cut at {cut} refused by the exact boolean");
            continue;
        };
        let total: f64 = faces_of(&topo, solid, "sphere")
            .iter()
            .map(|&f| analytic_face_area(&topo, f, 8).unwrap())
            .sum();
        assert_exact(total, TAU * r * (r + cut), &format!("cap below {cut}"));
        let c = census(&topo, solid);
        println!("sphere cap below {cut}: {c:?}");
    }
}

#[test]
fn spherical_cavity_cap_is_exact() {
    // A reversed spherical face: a ball pocket in the top of a block.
    let r = 4.0;
    let mut topo = Topology::new();
    let block = primitives::make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
    let ball = primitives::make_sphere(&mut topo, r, 32).unwrap();
    translate(&mut topo, ball, 10.0, 10.0, 8.0);
    let solid = boolean(&mut topo, BooleanOp::Cut, block, ball).unwrap();
    let caps = faces_of(&topo, solid, "sphere");
    let total: f64 = caps
        .iter()
        .map(|&f| {
            assert!(topo.face(f).unwrap().is_reversed());
            analytic_face_area(&topo, f, 8).unwrap()
        })
        .sum();
    // The pocket is the cap below z = 10, height r + 2.
    assert_exact(total, TAU * r * (r + 2.0), "cavity cap");
    let c = census(&topo, solid);
    assert_eq!(c.fallback, 0, "{c:?}");
}

#[test]
fn filleted_box_faces_reduce_and_sum_to_the_closed_form() {
    // Every edge of a box rounded: 12 quarter cylinders and 8 sphere octants.
    let (a, b, c, r) = (40.0, 30.0, 20.0, 3.0);
    let mut topo = Topology::new();
    let solid = primitives::make_box(&mut topo, a, b, c).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    let solid = blend_ops::fillet_cascade(&mut topo, solid, &edges, r)
        .unwrap()
        .solid;
    let cylinders: f64 = faces_of(&topo, solid, "cylinder")
        .iter()
        .map(|&f| analytic_face_area(&topo, f, 8).unwrap())
        .sum();
    let octants = faces_of(&topo, solid, "sphere");
    assert_eq!(octants.len(), 8);
    for &octant in &octants {
        // A corner patch is bounded by three quarter great circles, so one
        // of the quadrature's candidate polar axes runs through a vertex.
        assert_exact(
            analytic_face_area(&topo, octant, 8).unwrap(),
            PI * r * r / 2.0,
            "corner octant",
        );
    }
    let spheres: f64 = octants
        .iter()
        .map(|&f| analytic_face_area(&topo, f, 8).unwrap())
        .sum();
    let straight = 4.0 * ((a - 2.0 * r) + (b - 2.0 * r) + (c - 2.0 * r));
    assert_exact(cylinders, 0.25 * TAU * r * straight, "fillet bands");
    assert_exact(spheres, 2.0 * TAU * r * r, "corner octants");
    let census = census(&topo, solid);
    assert_eq!((census.reduced, census.fallback), (20, 0), "{census:?}");
}

#[test]
fn torus_rim_band_is_exact() {
    // A rounded rim on a cylinder: a quarter torus band.
    let (radius, height, fillet) = (6.0, 5.0, 1.5);
    let mut topo = Topology::new();
    let solid = primitives::make_cylinder(&mut topo, radius, height).unwrap();
    let rims: Vec<_> = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            matches!(edge.curve(), remus_topology::edge::EdgeCurve::Circle(_))
                && topo.vertex(edge.start()).unwrap().point().z() > height - 1e-9
        })
        .collect();
    assert_eq!(rims.len(), 1);
    let solid = blend_ops::fillet_cascade(&mut topo, solid, &rims, fillet)
        .unwrap()
        .solid;
    let [band] = faces_of(&topo, solid, "torus")[..] else {
        panic!("one torus band");
    };
    // Quarter tube swept around the axis at major radius R = radius - fillet:
    // r ∫ (R + r cos v) dv over a quarter turn, times 2 pi.
    let major = radius - fillet;
    let expected = TAU * fillet * major.mul_add(PI / 2.0, fillet);
    assert_exact(
        analytic_face_area(&topo, band, 8).unwrap(),
        expected,
        "rim band",
    );
    let c = census(&topo, solid);
    assert_eq!(c.fallback, 0, "{c:?}");
}

#[test]
fn cross_drilled_wall_keeps_its_holes() {
    // A shaft with a perpendicular through hole: the shaft wall carries two
    // inner wires (the hole's rims), each a saddle curve on the cylinder.
    let mut topo = Topology::new();
    let shaft = primitives::make_cylinder(&mut topo, 5.0, 30.0).unwrap();
    let drill = primitives::make_cylinder(&mut topo, 1.5, 20.0).unwrap();
    let turn = Mat4::rotation_x(std::f64::consts::FRAC_PI_2);
    transform::transform_solid(&mut topo, drill, &turn).unwrap();
    translate(&mut topo, drill, 0.0, 10.0, 15.0);
    let solid = boolean(&mut topo, BooleanOp::Cut, shaft, drill).unwrap();
    let walls = faces_of(&topo, solid, "cylinder");
    let shaft_wall = walls
        .iter()
        .copied()
        .find(|&w| match topo.face(w).unwrap().surface() {
            FaceSurface::Cylinder(c) => (c.radius() - 5.0).abs() < 1e-9,
            _ => false,
        })
        .unwrap();
    let area = analytic_face_area(&topo, shaft_wall, 8).unwrap();
    // The rims are fitted intersection curves, so the reference is the wall
    // the boolean actually built: a cylinder's area element is the constant
    // `r`, so each hole removes `r` times its `(θ, z)` area, here a shoelace
    // over 20 000 points of the rim curve itself.
    let face = topo.face(shaft_wall).unwrap();
    assert_eq!(face.inner_wires().len(), 2);
    let mut holes = 0.0;
    for &wire in face.inner_wires() {
        let mut uv = Vec::new();
        for oriented in topo.wire(wire).unwrap().edges() {
            let edge = topo.edge(oriented.edge()).unwrap();
            let (start, end) = (
                topo.vertex(edge.start()).unwrap().point(),
                topo.vertex(edge.end()).unwrap().point(),
            );
            let (mut t0, mut t1) = edge.strict_domain().unwrap();
            if !oriented.is_forward() {
                std::mem::swap(&mut t0, &mut t1);
            }
            let n = 20_000;
            for k in 0..n {
                let t = t0 + (t1 - t0) * f64::from(k) / f64::from(n);
                let p = edge.curve().evaluate_with_endpoints(t, start, end);
                uv.push((p.y().atan2(p.x()), p.z()));
            }
        }
        // Unwrap θ along the rim; the rim does not wind around the shaft.
        for k in 1..uv.len() {
            let previous = uv[k - 1].0;
            uv[k].0 = previous + (uv[k].0 - previous + PI).rem_euclid(TAU) - PI;
        }
        let shoelace: f64 = (0..uv.len())
            .map(|k| {
                let (a, b) = (uv[k], uv[(k + 1) % uv.len()]);
                a.0.mul_add(b.1, -(b.0 * a.1))
            })
            .sum();
        holes += 5.0 * 0.5 * shoelace.abs();
    }
    let reference = TAU * 5.0 * 30.0 - holes;
    let quadrature = integrate_face_area(&topo, shaft_wall, 8).unwrap().abs();
    println!("drilled wall: reduced {area}, quadrature {quadrature}, rim shoelace {reference}");
    // The shoelace's own chord error over 20 000 points is ~1e-9 relative.
    assert!((area - reference).abs() <= 1e-8 * reference);
    let c = census(&topo, solid);
    assert_eq!(c.fallback, 0, "{c:?}");
}

#[test]
fn sphere_primitive_hemispheres_are_exact() {
    let mut topo = Topology::new();
    let sphere = primitives::make_sphere(&mut topo, 4.0, 32).unwrap();
    let total: f64 = faces_of(&topo, sphere, "sphere")
        .iter()
        .map(|&f| analytic_face_area(&topo, f, 8).unwrap())
        .sum();
    assert_exact(total, 2.0 * TAU * 16.0, "sphere");
    let c = census(&topo, sphere);
    assert_eq!(c.fallback, 0, "{c:?}");
}

#[test]
fn unqualified_boundaries_keep_the_quadrature_bit_for_bit() {
    // A full torus is bounded by seams that wind around its tube: the
    // reduction refuses it and reads the quadrature.
    let mut topo = Topology::new();
    let torus = primitives::make_torus(&mut topo, 10.0, 2.0, 32).unwrap();
    let c = census(&topo, torus);
    assert_eq!(c.reduced, 0, "{c:?}");
    assert!(c.fallback > 0, "{c:?}");
}
