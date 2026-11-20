//! Compare every regenerated wall carrier and oriented wire to the input bore.
//! Index faces by their lowest vertex along the thread axis. Each half turn
//! contributes just one section of candidates at that height.
use super::{Bore, Ring};
use remus_math::compact_sort::CompactSort;
use remus_math::{tolerance::Tolerance, vec::Point3};
use remus_topology::{
    Topology,
    edge::{EdgeCurve, EdgeId},
    explorer::face_vertices,
    face::{FaceId, FaceSurface},
};
use std::collections::BTreeSet;

pub(super) fn matches(
    topo: &Topology,
    bore: &Bore,
    template: &Topology,
    ring: &Ring,
) -> Option<()> {
    if bore.wall.len() != ring.face_roles.len() {
        return None;
    }
    let level = |t: &Topology, f| {
        face_vertices(t, f)
            .ok()?
            .into_iter()
            .try_fold(f64::INFINITY, |lowest, v| {
                Some(
                    lowest
                        .min((t.vertex(v).ok()?.point() - bore.frame.origin).dot(bore.frame.axis)),
                )
            })
    };
    let mut levels = bore
        .wall
        .iter()
        .map(|&f| Some((level(topo, f)?, f)))
        .collect::<Option<Vec<_>>>()?;
    levels.compact_sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut used = BTreeSet::new();
    for &(face, _) in &ring.face_roles {
        let z = level(template, face)?;
        let tol = Tolerance::new().linear;
        let lo = levels.partition_point(|p| p.0 < z - tol);
        let hi = levels.partition_point(|p| p.0 <= z + tol);
        let mut candidates = levels[lo..hi]
            .iter()
            .map(|p| p.1)
            .filter(|f| !used.contains(f) && same_face(template, face, topo, *f));
        let found = candidates.next()?;
        if candidates.next().is_some() {
            return None;
        }
        used.insert(found);
    }
    Some(())
}
fn same_face(template: &Topology, a: FaceId, topo: &Topology, existing: FaceId) -> bool {
    let (Ok(f), Ok(g)) = (template.face(a), topo.face(existing)) else {
        return false;
    };
    if f.is_reversed() != g.is_reversed()
        || !f.inner_wires().is_empty()
        || !g.inner_wires().is_empty()
    {
        return false;
    }
    let same_carrier = match f.surface() {
        FaceSurface::Nurbs(a) => {
            let FaceSurface::Nurbs(b) = g.surface() else {
                return false;
            };
            a.degree_u() == b.degree_u()
                && a.degree_v() == b.degree_v()
                && knots(a.knots_u(), b.knots_u())
                && knots(a.knots_v(), b.knots_v())
                && a.control_points().len() == b.control_points().len()
                && a.control_points()
                    .iter()
                    .zip(b.control_points())
                    .all(|(x, y)| points(x, y))
                && a.weights().len() == b.weights().len()
                && a.weights()
                    .iter()
                    .zip(b.weights())
                    .all(|(x, y)| weights(x, y))
        }
        FaceSurface::Cylinder(_) | FaceSurface::Cone(_) => {
            crate::heal::surfaces_equivalent_pub(f.surface(), g.surface())
        }
        FaceSurface::Plane { .. } | FaceSurface::Sphere(_) | FaceSurface::Torus(_) => false,
    };
    if !same_carrier {
        return false;
    }
    let (Ok(wa), Ok(wb)) = (template.wire(f.outer_wire()), topo.wire(g.outer_wire())) else {
        return false;
    };
    let x = wa.edges();
    let y = wb.edges();
    x.len() == y.len()
        && (0..x.len()).any(|shift| {
            x.iter().zip(y.iter().cycle().skip(shift)).all(|(a, b)| {
                a.is_forward() == b.is_forward()
                    && topo
                        .pcurve_oriented(b.edge(), existing, b.is_forward())
                        .is_none()
                    && edge_match(template, a.edge(), topo, b.edge()).unwrap_or(false)
            })
        })
}
// A change of angular reference translates the spline parameter origin.
fn knots(x: &[f64], y: &[f64]) -> bool {
    x.len() == y.len()
        && x.iter()
            .zip(y)
            .all(|(a, b)| ((a - x[0]) - (b - y[0])).abs() < 1e-9)
}
// Equivalent rational circle sectors can differ by rounding in cos(sweep/2).
fn weights(x: &[f64], y: &[f64]) -> bool {
    x.len() == y.len()
        && x.iter()
            .zip(y)
            .all(|(&a, &b)| (a - b).abs() <= 64.0 * f64::EPSILON * a.abs().min(b.abs()))
}
fn scalars(x: &[f64], y: &[f64]) -> bool {
    x.len() == y.len() && x.iter().zip(y).all(|(a, b)| (a - b).abs() < 1e-9)
}
fn points(x: &[Point3], y: &[Point3]) -> bool {
    x.len() == y.len() && x.iter().zip(y).all(|(&a, &b)| (a - b).length() < 1e-8)
}
fn edge_match(template: &Topology, a: EdgeId, topo: &Topology, b: EdgeId) -> Option<bool> {
    let (a, b) = (template.edge(a).ok()?, topo.edge(b).ok()?);
    if (template.vertex(a.start()).ok()?.point() - topo.vertex(b.start()).ok()?.point()).length()
        > 1e-8
        || (template.vertex(a.end()).ok()?.point() - topo.vertex(b.end()).ok()?.point()).length()
            > 1e-8
    {
        return Some(false);
    }
    let (s, t) = a.strict_domain().ok()?;
    let (u, v) = b.strict_domain().ok()?;
    Some(match a.curve() {
        EdgeCurve::Line => matches!(b.curve(), EdgeCurve::Line),
        EdgeCurve::Circle(x) => {
            let EdgeCurve::Circle(y) = b.curve() else {
                return Some(false);
            };
            (x.center() - y.center()).length() < 1e-8
                && (x.radius() - y.radius()).abs() < 1e-8
                && (x.normal() - y.normal()).length() < 1e-9
                && ((t - s) - (v - u)).abs() < 1e-9
                && (x.evaluate(s.midpoint(t)) - y.evaluate(u.midpoint(v))).length() < 1e-8
        }
        EdgeCurve::NurbsCurve(x) => {
            let EdgeCurve::NurbsCurve(y) = b.curve() else {
                return Some(false);
            };
            x.degree() == y.degree()
                && scalars(
                    &[s - x.domain().0, t - x.domain().0],
                    &[u - y.domain().0, v - y.domain().0],
                )
                && knots(x.knots(), y.knots())
                && points(x.control_points(), y.control_points())
                && weights(x.weights(), y.weights())
        }
        EdgeCurve::Ellipse(_) | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => false,
    })
}
