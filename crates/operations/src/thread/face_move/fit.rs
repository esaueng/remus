//! Recover the fitting tolerance from the existing cubic Hermite spans.
//! A uniform fit with span width h records the smallest count for which
//! sqrt(3) M h^4 / 384 fits the requested tolerance. The largest such
//! remainder across outline, screw and chamfer fits reproduces all counts.
use super::Bore;
use remus_math::vec::Point3;
use remus_topology::{Topology, edge::EdgeCurve, explorer::face_edges, face::FaceSurface};
use std::f64::consts::TAU;

fn span(knots: &[f64]) -> f64 {
    knots.windows(2).map(|k| k[1] - k[0]).fold(0.0, f64::max)
}

pub(super) fn tolerance(topo: &Topology, bore: &Bore) -> Option<f64> {
    let radial = |p: Point3| {
        let v = p - bore.frame.origin;
        v - bore.frame.axis * v.dot(bore.frame.axis)
    };
    let mut estimate = 0.0_f64;
    let mut profile_radius = bore.root;
    let mut axial_step = 0.0_f64;
    for &face in &bore.wall {
        let FaceSurface::Nurbs(s) = topo.face(face).ok()?.surface() else {
            continue;
        };
        if s.degree_u() != 3 {
            return None;
        }
        match s.degree_v() {
            3 => {
                let transverse = s.knots_v();
                let width = transverse.last()? - transverse.first()?;
                if width <= 0.0 {
                    return None;
                }
                let fourth = bore.root.hypot(4.0 * bore.form.depth / width);
                estimate = estimate.max(fourth * span(transverse).powi(4));
            }
            // A land wrapping beyond one turn in a chamfer zone is a screw
            // sweep of an exact rational circular arc. Only its axial fit counts.
            2 => {}
            _ => return None,
        }
        axial_step = axial_step.max(span(s.knots_u()));
        for &p in s.control_points().first()? {
            profile_radius = profile_radius.max(radial(p).length());
        }
    }
    let rate = TAU / bore.form.lead();
    estimate = estimate.max(profile_radius * (axial_step * rate).powi(4));
    for mouth in &bore.ends {
        for id in face_edges(topo, mouth.cone).ok()? {
            let edge = topo.edge(id).ok()?;
            let EdgeCurve::NurbsCurve(s) = edge.curve() else {
                continue;
            };
            let (start, end) = s.domain();
            let r = radial(*s.control_points().first()?);
            let end_radius = radial(*s.control_points().last()?).length();
            let dr = (end_radius - r.length()) / (end - start);
            let tangent = *s.derivatives(start, 1).get(1)?;
            let omega = r.cross(tangent).dot(bore.frame.axis).abs() / r.length_squared();
            let fourth = omega.powi(3) * (omega * r.length().max(end_radius)).hypot(4.0 * dr);
            estimate = estimate.max(fourth * span(s.knots()).powi(4));
        }
    }
    // Keep floating-point reconstruction off the span-count ceil boundary.
    let bound = estimate * (3.0_f64.sqrt() / 384.0) * (1.0 + 1e-9);
    (bound.is_finite() && bound > 0.0).then_some(bound)
}
