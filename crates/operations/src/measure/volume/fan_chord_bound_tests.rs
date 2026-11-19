//! Planar faces read by the chord-polygon fan in `solid_volume`'s face
//! integrals.
//!
//! The fan reads a planar face the exact reader declines as its sampled
//! boundary's polygon, whatever the Gauss order, so the order comparison
//! gave such a face no disagreement. Here the decline comes from the
//! partial revolve's on-axis point edge widened into a `1e-8·s` loop: too
//! wide to be read as a point chord, its own moments rounding noise off the
//! origin. Every sector cap then falls to the fan and loses its arc's chord
//! slivers (32 chords per edge).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use super::*;
use remus_math::nurbs::curve::NurbsCurve;
use remus_topology::builder::make_polygon_wire;
use remus_topology::face::Face;

/// The sector `{0 ≤ φ ≤ angle, r ≤ 2s, 0 ≤ z ≤ 2s}` about the `z` axis
/// through `axis`, revolved from its rectangular profile.
fn sector(topo: &mut Topology, s: f64, axis: Point3, angle: f64) -> SolidId {
    let (e1, e2, d) = (
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    );
    let (r, h) = (2.0 * s, 2.0 * s);
    let at = |x: f64, z: f64| axis + e1 * x + d * z;
    let wire = make_polygon_wire(
        topo,
        &[at(0.0, 0.0), at(r, 0.0), at(r, h), at(0.0, h)],
        1e-7,
    )
    .unwrap();
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: -e2,
            d: -e2.dot(axis - Point3::new(0.0, 0.0, 0.0)),
        },
    ));
    crate::revolve::revolve(topo, face, axis, d, angle).unwrap()
}

/// Move the middle control point of every point-sized NURBS edge (the
/// zero-radius arcs at the on-axis vertices) `delta` along `+x`, keeping
/// its trim: a loop too wide to be read as a point chord. Returns how many
/// edges were widened.
fn widen_point_edges(topo: &mut Topology, solid: SolidId, delta: f64) -> usize {
    let mut edges = Vec::new();
    for face in remus_topology::explorer::solid_faces(topo, solid).unwrap() {
        for e in remus_topology::explorer::face_edges(topo, face).unwrap() {
            if let EdgeCurve::NurbsCurve(nc) = topo.edge(e).unwrap().curve() {
                let cps = nc.control_points();
                if cps.iter().all(|p| (*p - cps[0]).length() <= 1e-9) && !edges.contains(&e) {
                    edges.push(e);
                }
            }
        }
    }
    for &e in &edges {
        let edge = topo.edge(e).unwrap();
        let trim = edge.trim();
        let EdgeCurve::NurbsCurve(nc) = edge.curve().clone() else {
            continue;
        };
        let mut cps = nc.control_points().to_vec();
        let mid = cps.len() / 2;
        cps[mid] = cps[mid] + Vec3::new(delta, 0.0, 0.0);
        let curve =
            NurbsCurve::new(nc.degree(), nc.knots().to_vec(), cps, nc.weights().to_vec()).unwrap();
        let edge = topo.edge_mut(e).unwrap();
        edge.set_curve(EdgeCurve::NurbsCurve(curve));
        edge.set_trim(trim);
    }
    edges.len()
}

/// The sector about an axis through `s·(5, 3, 1)` with its point edges
/// widened by `1e-8·s`, and its volume `angle·(2s)³/2`.
fn fan_read_sector(topo: &mut Topology, s: f64, angle: f64) -> (SolidId, f64) {
    let solid = sector(topo, s, Point3::new(5.0 * s, 3.0 * s, s), angle);
    // One point edge per sector cap: two caps per revolve segment.
    let widened = widen_point_edges(topo, solid, 1e-8 * s);
    assert!(widened >= 2);
    let mut fan = 0;
    for face in remus_topology::explorer::solid_faces(topo, solid).unwrap() {
        if matches!(
            topo.face(face).unwrap().surface(),
            FaceSurface::Plane { .. }
        ) && remus_check::properties::face_integrator::planar_face_exact_area(topo, face)
            .unwrap()
            .is_none()
        {
            fan += 1;
        }
    }
    assert_eq!(fan, widened, "every sector cap falls to the fan");
    (solid, angle * (2.0 * s).powi(3) / 2.0)
}

/// The face integrals' volume and summed disagreement for `solid`'s shell.
fn shell_reading(topo: &Topology, solid: SolidId) -> (f64, f64) {
    let order = remus_check::properties::PropertiesOptions::default().gauss_order;
    let shell = topo.solid(solid).unwrap().outer_shell();
    let (volume, disagreement) = qualified_shell_volume(topo, shell, order, false)
        .unwrap()
        .expect("the shell closes: its caps' chord errors cancel");
    (volume.abs(), disagreement)
}

/// The half cylinder's two caps lose the same chord slivers, so the shell
/// still closes, and `solid_volume` returned the face integrals 1.347e-4 of
/// the volume short at every scale. The fan's chord bound now counts as
/// those caps' disagreement: it covers the error and declines the reading.
#[test]
fn a_fan_read_half_cylinder_whose_cap_errors_cancel_is_declined() {
    for s in [1e-2, 1.0, 1e2] {
        let mut topo = Topology::new();
        let (solid, exact) = fan_read_sector(&mut topo, s, PI);
        let (volume, disagreement) = shell_reading(&topo, solid);
        let error = (volume - exact).abs();
        assert!(
            error > 1e-4 * exact,
            "s={s}: the fan reads {:.3e} of the volume short",
            error / exact
        );
        assert!(
            error <= disagreement && disagreement <= 2.0 * error,
            "s={s}: error {error:.3e} against the bound {disagreement:.3e}"
        );
        assert_eq!(
            qualified_face_integral_volume(&topo, solid).unwrap(),
            None,
            "s={s}: the face integrals are declined"
        );
        // The closed mesh at the clamp deflection measures it instead, to
        // its own chord accuracy, never the face integrals' short reading.
        let measured = solid_volume(&topo, solid, 1e-3 * s).unwrap();
        assert!(
            (measured - exact).abs() <= 1e-4 * exact && (measured - volume).abs() > 1e-6 * exact,
            "s={s}: solid_volume {measured}, closed form {exact}, face integrals {volume}"
        );
    }
}

/// A narrow sector's caps lose slivers within the order-agreement budget:
/// the bound accepts the face integrals, within it of the closed form.
#[test]
fn a_fan_read_sector_within_the_budget_keeps_its_face_integrals() {
    for s in [1e-2, 1.0, 1e2] {
        let mut topo = Topology::new();
        let (solid, exact) = fan_read_sector(&mut topo, s, 0.05);
        let (volume, disagreement) = shell_reading(&topo, solid);
        assert!(
            (volume - exact).abs() <= disagreement && disagreement <= ORDER_AGREEMENT * exact,
            "s={s}: error {:.3e} against the bound {disagreement:.3e}",
            (volume - exact).abs()
        );
        let qualified = qualified_face_integral_volume(&topo, solid)
            .unwrap()
            .unwrap_or_else(|| panic!("s={s}: the face integrals are accepted"));
        assert_eq!(qualified.to_bits(), volume.to_bits(), "s={s}");
        let measured = solid_volume(&topo, solid, 1e-3 * s).unwrap();
        assert!(
            (measured - exact).abs() <= disagreement,
            "s={s}: solid_volume {measured}, closed form {exact}"
        );
    }
}
