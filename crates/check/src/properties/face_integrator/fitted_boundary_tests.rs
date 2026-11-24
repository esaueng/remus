//! The certificate a reading on a fitted boundary carries
//! (`FittedBoundaryReading`), against quantities read off the stored
//! boundary directly (tests).
//!
//! A cylinder or cone face with a fitted rim is read at the rim's projection
//! onto the surface. The closure allowance bounds how far that moves the
//! face's vector area from the stored rim's own, `½ ∮ (C − O) × dC` (any
//! surface spanning the stored loops has it), which these tests take by
//! quadrature on the stored curves. The rims are exact sections pushed off
//! the surface by a non-uniform radial pattern, so the projection does move
//! the vector area: the allowance must hold it, and stays within a hundredfold
//! of it (24 to 81 times it here, twice the derived bound included).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::cell::Cell;
use std::f64::consts::TAU;

use remus_geometry::convert::curve_to_nurbs::ellipse_to_nurbs;
use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::nurbs::NurbsCurve;
use remus_math::quadrature::gauss_legendre_points;
use remus_math::surfaces::CylindricalSurface;
use remus_math::traits::ParametricCurve;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use super::cone_green_tests::{Section, closed_nurbs_edge, reversed, sections};
use super::{IntegrationRule, MomentRequest, cone_green, cylinder_green};

/// Radial push of each control point, as a fraction of the scale: the first
/// and last points of a closed rim coincide, and so do their pushes.
const PATTERN: [f64; 9] = [1.0, -0.6, 0.3, 0.8, -1.0, 0.2, -0.4, 0.9, 1.0];

/// `curve` with control point `i` moved `PATTERN[i] · push` away from the
/// axis through `origin` along `axis`.
fn pushed(curve: &NurbsCurve, origin: Point3, axis: Vec3, push: f64) -> NurbsCurve {
    assert_eq!(curve.control_points().len(), PATTERN.len());
    let points: Vec<_> = curve
        .control_points()
        .iter()
        .zip(PATTERN)
        .map(|(&p, k)| {
            let d = p - origin;
            let radial = (d - axis * d.dot(axis)).normalize().unwrap();
            p + radial * (k * push)
        })
        .collect();
    NurbsCurve::new(
        curve.degree(),
        curve.knots().to_vec(),
        points,
        curve.weights().to_vec(),
    )
    .unwrap()
}

/// `½ ∮ (C − O) × dC` over the face's stored boundary, each edge run as its
/// wire uses it, by composite Gauss–Legendre on its own parameter.
fn stored_vector_area(topo: &Topology, face: FaceId) -> Vec3 {
    let face = topo.face(face).unwrap();
    let gauss = gauss_legendre_points(16);
    let mut sum = Vec3::new(0.0, 0.0, 0.0);
    for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oriented in topo.wire(wire).unwrap().edges() {
            let edge = topo.edge(oriented.edge()).unwrap();
            let (start, end) = (
                topo.vertex(edge.start()).unwrap().point(),
                topo.vertex(edge.end()).unwrap().point(),
            );
            let (t0, t1) = edge.strict_domain().unwrap();
            let sense = if oriented.is_forward() { 0.5 } else { -0.5 };
            let pieces = 256;
            let step = (t1 - t0) / f64::from(pieces);
            for k in 0..pieces {
                let (a, b) = (t0 + step * f64::from(k), t0 + step * f64::from(k + 1));
                let half = (b - a) / 2.0;
                for gp in gauss {
                    let t = a + half + half * gp.x;
                    let c = edge.curve().evaluate_with_endpoints(t, start, end);
                    let dc = match edge.curve() {
                        EdgeCurve::Line => end - start,
                        EdgeCurve::Circle(c) => ParametricCurve::derivative_pair(c, t).unwrap().0,
                        EdgeCurve::Ellipse(e) => ParametricCurve::derivative_pair(e, t).unwrap().0,
                        EdgeCurve::NurbsCurve(n) => {
                            ParametricCurve::derivative_pair(n, t).unwrap().0
                        }
                        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
                            panic!("no conic rims here")
                        }
                    };
                    let p = c - Point3::new(0.0, 0.0, 0.0);
                    sum += p.cross(dc) * (sense * gp.w * half);
                }
            }
        }
    }
    sum
}

fn collecting_rule(sink: &Cell<Option<Vec3>>) -> IntegrationRule<'_> {
    IntegrationRule {
        order: 5,
        adaptive: None,
        knots: None,
        reference: Point3::new(0.0, 0.0, 0.0),
        moments: MomentRequest::VolumeAreaAndFlux,
        ruled_guard: false,
        flux: Some(sink),
        cone_apex: None,
        boundary: None,
    }
}

/// The pointed face from the apex up to `rim` on `section`'s cone.
fn pointed_face(topo: &mut Topology, section: &Section, rim: NurbsCurve) -> FaceId {
    let (rim, rim_vertex) = closed_nurbs_edge(topo, rim);
    let apex = topo.add_vertex(Vertex::new(section.apex, 1e-9));
    let seam = topo.add_edge(Edge::new(rim_vertex, apex, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(rim, true),
                OrientedEdge::new(seam, true),
                OrientedEdge::new(seam, false),
            ],
            true,
        )
        .unwrap(),
    );
    topo.add_face(Face::new(wire, vec![], FaceSurface::Cone(section.cone())))
}

/// `(|N − N_stored|, allowance, volume disagreement, area^(3/2))` of a reading.
fn fitted_terms(reading: &cylinder_green::GreenReading, stored: Vec3) -> (f64, f64, f64, f64) {
    let fitted_reading = reading.fitted.expect("a fitted boundary reading");
    (
        (reading.flux - stored).length(),
        fitted_reading.closure_allowance,
        fitted_reading.span_disagreement,
        // The volume's own scale (a term about the apex vanishes).
        reading.contribution.area * reading.contribution.area.sqrt(),
    )
}

/// On cones: three oblique sections at three scales, each rim pushed up to
/// 2e-4 of the scale off the cone.
#[test]
fn the_closure_allowance_holds_what_projection_moves_on_a_cone() {
    for scale in [1e-3, 1.0, 1e3] {
        for (index, section) in sections(scale).iter().enumerate() {
            let rim = pushed(&section.rim(), section.apex, section.axis, 2e-4 * scale);
            let mut topo = Topology::new();
            let face = pointed_face(&mut topo, section, rim);
            let data = topo.face(face).unwrap().clone();
            let sink = Cell::new(None);
            let reading = cone_green::integrate::<false>(
                &topo,
                &data,
                &section.cone(),
                &collecting_rule(&sink),
                1.0,
            )
            .unwrap()
            .expect("the boundary reading");
            let (moved, allowance, disagreement, volume) =
                fitted_terms(&reading, stored_vector_area(&topo, face));
            let label = format!("section {index} at scale {scale}");
            assert!(
                moved <= allowance,
                "{label}: moved {moved:e} > {allowance:e}"
            );
            assert!(
                moved >= allowance / 100.0,
                "{label}: moved {moved:e} ≪ {allowance:e}"
            );
            assert!(disagreement <= 1e-9 * volume, "{label}: {disagreement:e}");
        }
    }
}

/// On cylinders: the band between a circle and an oblique section of the
/// cylinder, the section a rational quadratic pushed up to 2e-4 of the
/// radius off the wall, at three scales.
#[test]
fn the_closure_allowance_holds_what_projection_moves_on_a_cylinder() {
    for scale in [1e-3, 1.0, 1e3] {
        let (r, h, s) = (scale, 3.0 * scale, 0.5);
        let axis = Vec3::new(0.0, 0.0, 1.0);
        let origin = Point3::new(0.0, 0.0, 0.0);
        let ellipse = Ellipse3D::new_with_ref(
            Point3::new(0.0, 0.0, h),
            Vec3::new(-s, 0.0, 1.0),
            r * s.hypot(1.0),
            r,
            Vec3::new(1.0, 0.0, s),
        )
        .unwrap();
        let section = ellipse_to_nurbs(&ellipse, 0.0, TAU).unwrap();
        let top = pushed(&section, origin, axis, 2e-4 * scale);
        let mut topo = Topology::new();
        // Counter-clockwise in `(u, v)` about the outward normal: the top
        // rim against `u`, the bottom one along it.
        let (top, _) = closed_nurbs_edge(&mut topo, reversed(&top));
        let top = topo.add_wire(Wire::new(vec![OrientedEdge::new(top, true)], true).unwrap());
        let circle = Circle3D::new(origin, axis, r).unwrap();
        let vertex = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-9 * scale));
        let mut edge = Edge::new(vertex, vertex, EdgeCurve::Circle(circle));
        edge.set_trim(Some((0.0, TAU)));
        let edge = topo.add_edge(edge);
        let bottom = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
        let cylinder = CylindricalSurface::new(origin, axis, r).unwrap();
        let face = topo.add_face(Face::new(
            top,
            vec![bottom],
            FaceSurface::Cylinder(cylinder.clone()),
        ));
        let data = topo.face(face).unwrap().clone();
        let sink = Cell::new(None);
        let reading = cylinder_green::integrate::<false>(
            &topo,
            &data,
            &cylinder,
            &collecting_rule(&sink),
            1.0,
        )
        .unwrap()
        .expect("the boundary reading");
        let (moved, allowance, disagreement, volume) =
            fitted_terms(&reading, stored_vector_area(&topo, face));
        assert!(
            moved <= allowance,
            "scale {scale}: moved {moved:e} > {allowance:e}"
        );
        assert!(
            moved >= allowance / 100.0,
            "scale {scale}: moved {moved:e} ≪ {allowance:e}"
        );
        assert!(
            disagreement <= 1e-9 * volume,
            "scale {scale}: {disagreement:e}"
        );
    }
}

/// A fitted reading's certificate does not depend on whether the rule reads
/// every moment: a volume-only reading takes the full reading's path and its
/// area, volume and vector area bit for bit.
#[test]
fn a_volume_only_reading_takes_the_full_readings_path_bit_for_bit() {
    for scale in [1e-3, 1.0, 1e3] {
        for section in &sections(scale) {
            let rim = pushed(&section.rim(), section.apex, section.axis, 2e-4 * scale);
            let mut topo = Topology::new();
            let face = pointed_face(&mut topo, section, rim);
            let data = topo.face(face).unwrap().clone();
            let sink = Cell::new(None);
            let light = collecting_rule(&sink);
            let full = IntegrationRule {
                moments: MomentRequest::Full,
                ..light
            };
            let read = |rule: &IntegrationRule<'_>| {
                cone_green::integrate::<false>(&topo, &data, &section.cone(), rule, 1.0)
                    .unwrap()
                    .unwrap()
            };
            let (light, full) = (read(&light), read(&full));
            assert_eq!(
                light.contribution.area.to_bits(),
                full.contribution.area.to_bits()
            );
            assert_eq!(
                light.contribution.volume.to_bits(),
                full.contribution.volume.to_bits()
            );
            assert_eq!(
                light.flux.0.map(f64::to_bits),
                full.flux.0.map(f64::to_bits)
            );
            assert_eq!(light.fitted, full.fitted);
        }
    }
}

/// The face-integral cache serves a fitted face's volume term (the reading
/// `solid_volume`'s certificates take) bit for bit as the uncached
/// integrator reads it, certificate included: cold or warm, enabled or
/// disabled, and through the thread's cache. About another reference, or
/// for a translated copy of the face, it integrates afresh rather than
/// re-expressing a retained reading, since whether the boundary reading is
/// taken turns on the reference.
#[test]
fn the_face_cache_serves_fitted_volume_terms_bit_for_bit() {
    use crate::properties::face_cache::{
        FaceIntegralCache, FaceVolumeTerm, clear_thread_face_cache, enable_thread_face_cache,
        integrate_face_volume_about_memoized, set_thread_face_cache_limits,
        thread_face_cache_stats,
    };
    let bits = |term: &FaceVolumeTerm| {
        (
            term.area.to_bits(),
            term.volume.to_bits(),
            term.vector_area.map(|n| n.0.map(f64::to_bits)),
            term.fitted_boundary
                .map(|c| (c.span_disagreement.to_bits(), c.closure_allowance.to_bits())),
        )
    };
    let uncached = |topo: &Topology, face, reference| {
        let reading =
            super::integrate_face_fixed_flux_with_boundary_about(topo, face, 5, reference, true)
                .unwrap();
        FaceVolumeTerm {
            area: reading.contribution.area,
            volume: reading.contribution.volume,
            vector_area: reading.vector_area,
            fitted_boundary: reading.fitted_boundary,
            planar_fan: reading.planar_fan,
        }
    };
    let section = &sections(1.0)[1];
    let shift = Vec3::new(3.25, -1.5, 0.75);
    let moved = Section {
        apex: section.apex + shift,
        axis: section.axis,
        tilt_dir: section.tilt_dir,
        a: section.a,
        theta: section.theta,
        d: section.d,
    };
    let mut topo = Topology::new();
    let rim = pushed(&section.rim(), section.apex, section.axis, 2e-4);
    let face = pointed_face(&mut topo, section, rim);
    let rim = pushed(&moved.rim(), moved.apex, moved.axis, 2e-4);
    let copy = pointed_face(&mut topo, &moved, rim);
    let (r0, r1) = (
        section.apex + Vec3::new(0.5, 0.25, -1.0),
        section.apex + Vec3::new(-2.0, 1.0, 0.5),
    );
    let expected = [
        (face, r0, uncached(&topo, face, r0)),
        (face, r1, uncached(&topo, face, r1)),
        (copy, r0 + shift, uncached(&topo, copy, r0 + shift)),
    ];
    for (_, _, term) in &expected {
        assert!(term.fitted_boundary.is_some(), "the boundary reading");
    }
    // The volume-only term is the full reading's.
    let full = super::integrate_face_fixed_about(&topo, face, 5, r0).unwrap();
    assert_eq!(expected[0].2.volume.to_bits(), full.volume.to_bits());
    assert_eq!(expected[0].2.area.to_bits(), full.area.to_bits());

    for (enabled, mut cache) in [
        (true, FaceIntegralCache::new()),
        (false, FaceIntegralCache::disabled()),
    ] {
        for _ in 0..2 {
            for (f, reference, term) in &expected {
                let served = cache
                    .integrate_face_volume_about(&topo, *f, 5, *reference)
                    .unwrap();
                assert_eq!(bits(&served), bits(term), "enabled {enabled}");
            }
        }
        let stats = cache.stats();
        assert_eq!((stats.rereferenced, stats.translated), (0, 0));
        // Warm, each request finds its own identical entry.
        if enabled {
            assert_eq!((stats.misses, stats.hits), (3, 3));
        }
    }

    set_thread_face_cache_limits(0, 0);
    enable_thread_face_cache();
    clear_thread_face_cache();
    for _ in 0..2 {
        for (f, reference, term) in &expected {
            let served = integrate_face_volume_about_memoized(&topo, *f, 5, *reference).unwrap();
            assert_eq!(bits(&served), bits(term), "thread cache");
        }
    }
    let stats = thread_face_cache_stats();
    set_thread_face_cache_limits(0, 0);
    assert_eq!((stats.rereferenced, stats.translated), (0, 0));
    assert_eq!((stats.misses, stats.hits), (3, 3));
}
