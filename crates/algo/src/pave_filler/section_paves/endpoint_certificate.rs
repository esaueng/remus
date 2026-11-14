//! One-sided exclusion at a resolved trimming contact.
//!
//! In a small cylinder chart, isolate ONE native trimming trace and prove it
//! is a graph. Whole-interval velocity bounds then prove that the section
//! leaves strictly into the excluded side of that graph. A weld ball only
//! selects where to try this proof; it never supplies an exclusion decision.
//!
//! With graph `z = f(q)` and normalized section parameter `s`, interval
//! arithmetic bounds `d(z - f(q))/ds = z' - f'(q) q'` strictly on the
//! excluded side. Isolation and graph coverage make that a whole-prefix
//! certificate. If the native contact and the section endpoint differ, the
//! negative initial gap divided by the minimum escape rate, times the
//! maximum model speed, bounds contact uncertainty in ARC LENGTH. It must
//! fit the caller's linear tolerance; the much larger weld never pays for it.
//! A zero escape rate (including tangency) cannot authorize exclusion.

use remus_math::context::OperationContext;
use remus_math::nurbs::NurbsCurve;
use remus_math::nurbs::reuse::bound_curve_difference;
use remus_math::surfaces::CylindricalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;
use remus_topology::vertex::VertexId;
use remus_topology::wire::WireId;

use crate::ds::{GfaArena, Pave};

use super::{TrimmedFace, WELD_FACTOR, section_ball};

pub(super) struct Boundary {
    curve: EdgeCurve,
    range: (f64, f64),
    ends: (Point3, Point3),
    contacts: Vec<Pave>,
}

pub(super) fn boundaries(topo: &Topology, arena: &GfaArena, wire: WireId) -> Option<Vec<Boundary>> {
    let wire = topo.wire(wire).ok()?;
    wire.edges()
        .iter()
        .filter(|oe| {
            wire.edges()
                .iter()
                .filter(|e| e.edge() == oe.edge())
                .count()
                == 1
        })
        .map(|oe| {
            let edge = topo.edge(oe.edge()).ok()?;
            let (a, b) = edge.strict_domain().ok()?;
            let range = (a.min(b), a.max(b));
            if range.0 >= range.1 {
                return None;
            }
            let ends = (
                topo.vertex(edge.start()).ok()?.point(),
                topo.vertex(edge.end()).ok()?.point(),
            );
            let mut contacts = Vec::new();
            if let Some(roots) = arena.edge_pave_blocks.get(&oe.edge()) {
                for leaf in arena.collect_leaf_pave_blocks(roots) {
                    let pb = arena.pave_blocks.get(leaf)?;
                    for pave in [pb.start, pb.end] {
                        contacts.push(Pave::new(arena.resolve_vertex(pave.vertex), pave.parameter));
                    }
                }
            }
            Some(Boundary {
                curve: edge.curve().clone(),
                range,
                ends: if a < b { ends } else { (ends.1, ends.0) },
                contacts,
            })
        })
        .collect()
}

/// Small outward-rounded intervals. Nonfinite or singular arithmetic can
/// never satisfy a certificate's strict inequalities.
#[derive(Clone, Copy, Debug)]
struct Iv(f64, f64);

impl Iv {
    const fn point(x: f64) -> Self {
        Self(x, x)
    }
    fn around(x: f64, r: f64) -> Self {
        Self((x - r).next_down(), (x + r).next_up())
    }
    fn sub(self, b: Self) -> Self {
        Self((self.0 - b.1).next_down(), (self.1 - b.0).next_up())
    }
    fn mul(self, b: Self) -> Self {
        let p = [self.0 * b.0, self.0 * b.1, self.1 * b.0, self.1 * b.1];
        Self(
            p.into_iter().fold(f64::INFINITY, f64::min).next_down(),
            p.into_iter().fold(f64::NEG_INFINITY, f64::max).next_up(),
        )
    }
    fn div(self, b: Self) -> Option<Self> {
        if b.0 <= 0. && b.1 >= 0. {
            return None;
        }
        Some(self.mul(Self((1. / b.1).next_down(), (1. / b.0).next_up())))
    }
    fn abs(self) -> f64 {
        self.0.abs().max(self.1.abs())
    }
    fn finite(self) -> bool {
        self.0.is_finite() && self.1.is_finite()
    }
}

/// Derivative with respect to the NORMALIZED interval, enclosed by a ball.
/// The Bernstein certificate covers every knot, however compressed its span.
fn velocity(curve: &EdgeCurve, range: (f64, f64), ends: (Point3, Point3)) -> Option<(Vec3, f64)> {
    let dt = range.1 - range.0;
    if dt <= 0. {
        return None;
    }
    let (v, error) = match curve {
        EdgeCurve::Line => (ends.1 - ends.0, 0.),
        EdgeCurve::NurbsCurve(n) => {
            let chord = NurbsCurve::new(
                1,
                vec![range.0, range.0, range.1, range.1],
                vec![ends.0, ends.1],
                vec![1.; 2],
            )
            .ok()?;
            let bound =
                bound_curve_difference(n, &chord, range, 1_000_000, &OperationContext::default())
                    .ok()?;
            (ends.1 - ends.0, (bound.derivative * dt).next_up())
        }
        EdgeCurve::Circle(c) => (
            c.tangent(f64::midpoint(range.0, range.1)) * (c.radius() * dt),
            (c.radius() * dt * dt * 0.5).next_up(),
        ),
        EdgeCurve::Ellipse(e) => (
            e.tangent(f64::midpoint(range.0, range.1)) * dt,
            (e.semi_major().max(e.semi_minor()) * dt * dt * 0.5).next_up(),
        ),
        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => return None,
    };
    let error = (error + 1e-14 * v.length().max(f64::MIN_POSITIVE)).next_up();
    (error.is_finite() && v.0.iter().all(|x| x.is_finite())).then_some((v, error))
}

/// Chart velocity `(R*u', v')` of the radially projected curve. Its
/// interval extension includes off-carrier marched traces and rotations.
fn chart_velocity(
    c: &CylindricalSurface,
    curve: &EdgeCurve,
    range: (f64, f64),
    ends: (Point3, Point3),
) -> Option<([Iv; 2], f64)> {
    let (p, r) = section_ball(curve, range, ends)?;
    let (v, e) = velocity(curve, range, ends)?;
    let q = p - c.origin();
    let slack = 1e-14
        * p.0
            .iter()
            .chain(c.origin().0.iter())
            .fold(c.radius().max(1.), |s, x| s.max(x.abs()));
    let x = Iv::around(q.dot(c.x_axis()), r + slack);
    let y = Iv::around(q.dot(c.y_axis()), r + slack);
    let radial = q - c.axis() * q.dot(c.axis());
    let lo = (radial.length() - r - slack).next_down();
    let hi = (radial.length() + r + slack).next_up();
    if lo <= 0. {
        return None;
    }
    let dv = |axis| Iv::around(v.dot(axis), e + 1e-14 * v.length());
    let angle = x
        .mul(dv(c.y_axis()))
        .sub(y.mul(dv(c.x_axis())))
        .div(Iv((lo * lo).next_down(), (hi * hi).next_up()))?
        .mul(Iv::point(c.radius()));
    let out = [angle, dv(c.axis())];
    out.iter()
        .all(|x| x.finite())
        .then_some((out, (v.length() + e).next_up()))
}

fn chart(c: &CylindricalSurface, p: Point3, near: f64) -> [f64; 2] {
    let (u, v) = c.project_point(p);
    [crate::periodic_chart::unwrap_near(u, near) * c.radius(), v]
}

fn projected_ball(
    c: &CylindricalSurface,
    curve: &EdgeCurve,
    range: (f64, f64),
    ends: (Point3, Point3),
) -> Option<(Point3, f64)> {
    let (p, r) = section_ball(curve, range, ends)?;
    let q = p - c.origin();
    let radial = q - c.axis() * q.dot(c.axis());
    let slack = 1e-14
        * p.0
            .iter()
            .chain(c.origin().0.iter())
            .fold(c.radius().max(1.), |s, x| s.max(x.abs()));
    let minimum = (radial.length() - r - slack).next_down();
    if minimum <= 0. {
        return None;
    }
    let (u, v) = c.project_point(p);
    Some((
        c.evaluate(u, v),
        (r * (c.radius() / minimum).max(1.) + slack).next_up(),
    ))
}

fn at(boundary: &Boundary, t: f64) -> Point3 {
    match &boundary.curve {
        EdgeCurve::Line => {
            boundary.ends.0
                + (boundary.ends.1 - boundary.ends.0)
                    * ((t - boundary.range.0) / (boundary.range.1 - boundary.range.0))
        }
        curve @ (EdgeCurve::Circle(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_)) => {
            curve.evaluate_with_endpoints(t, boundary.ends.0, boundary.ends.1)
        }
    }
}

/// The contact graph is the ONLY boundary in this neighborhood. All other
/// native intervals need strict separation, not a sampled distance check.
fn isolated(
    face: &TrimmedFace,
    c: &CylindricalSurface,
    index: usize,
    patch: (f64, f64),
    p: Point3,
    reach: f64,
) -> bool {
    let mut work = 0;
    for (i, boundary) in face.boundaries.iter().enumerate() {
        let mut pending = if i == index {
            vec![
                (boundary.range.0, patch.0, 0),
                (patch.1, boundary.range.1, 0),
            ]
        } else {
            vec![(boundary.range.0, boundary.range.1, 0)]
        };
        while let Some((a, b, depth)) = pending.pop() {
            if a >= b {
                continue;
            }
            work += 1;
            if work > 2048 {
                return false;
            }
            if let Some((center, radius)) = projected_ball(
                c,
                &boundary.curve,
                (a, b),
                (at(boundary, a), at(boundary, b)),
            ) && ((center - p).length() - radius).next_down() > reach
            {
                continue;
            }
            let m = f64::midpoint(a, b);
            if depth >= 32 || m <= a || m >= b {
                return false;
            }
            pending.extend([(a, m, depth + 1), (m, b, depth + 1)]);
        }
    }
    true
}

#[allow(clippy::too_many_lines)]
pub(super) fn excludes(
    face: &TrimmedFace,
    curve: &EdgeCurve,
    range: (f64, f64),
    ends: (Point3, Point3),
    id: VertexId,
    reverse: bool,
    tolerance: f64,
) -> bool {
    let FaceSurface::Cylinder(c) = &face.surface else {
        return false;
    };
    let result = || -> Option<bool> {
        let endpoint = if reverse { ends.1 } else { ends.0 };
        let (u, v) = c.project_point(endpoint);
        let origin = c.evaluate(u, v);
        let reach = tolerance * WELD_FACTOR * 8.;
        if reach >= c.radius() * 0.125 {
            return None;
        }
        let (center, radius) = projected_ball(c, curve, range, ends)?;
        if ((center - origin).length() + radius).next_up() >= reach * 0.25 {
            return None;
        }
        let (mut flow, speed) = chart_velocity(c, curve, range, ends)?;
        if reverse {
            flow = flow.map(|x| x.mul(Iv::point(-1.)));
        }
        let start = chart(c, endpoint, u);
        for (index, boundary) in face.boundaries.iter().enumerate() {
            let Some(contact) = boundary.contacts.iter().find(|p| p.vertex == id) else {
                continue;
            };
            let t = contact.parameter;
            if t <= boundary.range.0 || t >= boundary.range.1 {
                continue;
            }
            let point = at(boundary, t);
            let (pu, pv) = c.project_point(point);
            if (c.evaluate(pu, pv) - origin).length() >= reach * 0.125 {
                continue;
            }
            let base = chart(c, point, u);
            let scale = boundary.range.1 - boundary.range.0;
            for depth in 0..24 {
                let half = scale * 0.5_f64.powi(depth);
                let patch = (
                    (t - half).max(boundary.range.0),
                    (t + half).min(boundary.range.1),
                );
                let Some((d, _)) = chart_velocity(
                    c,
                    &boundary.curve,
                    patch,
                    (at(boundary, patch.0), at(boundary, patch.1)),
                ) else {
                    continue;
                };
                let q = usize::from(d[1].abs() > d[0].abs());
                let z = 1 - q;
                let Some(slope) = d[z].div(d[q]) else {
                    continue;
                };
                if !slope.finite() {
                    continue;
                }
                let left = chart(c, at(boundary, patch.0), u)[q];
                let right = chart(c, at(boundary, patch.1), u)[q];
                if left.min(right) >= base[q] - 2. * reach
                    || left.max(right) <= base[q] + 2. * reach
                {
                    continue;
                }
                // Select the excluded side using a point clear of the
                // certified outline. Isolation makes this a local side
                // decision about the exact graph, not sampled membership.
                for side in [-1., 1.] {
                    let mut witness = base;
                    witness[z] += side * reach * 0.5;
                    let witness = c.evaluate(witness[0] / c.radius(), witness[1]);
                    if face.clearance(witness).is_none() {
                        continue;
                    }
                    let rate = flow[z].sub(slope.mul(flow[q])).mul(Iv::point(side));
                    if !(rate.0 > 0. && rate.finite()) {
                        continue;
                    }
                    let pad = 1e-13
                        * origin
                            .0
                            .iter()
                            .fold(c.radius().max(1.), |s, x| s.max(x.abs()));
                    // A shared exact endpoint cancels identically. Otherwise
                    // account for the chart arithmetic AND contact mismatch.
                    // Exact equality cancels the same chart computation;
                    // this is an arithmetic identity, not a proximity test.
                    #[allow(clippy::float_cmp)]
                    let delta = if endpoint.0 == point.0 {
                        [Iv::point(0.); 2]
                    } else {
                        [
                            Iv::around(start[0] - base[0], pad),
                            Iv::around(start[1] - base[1], pad),
                        ]
                    };
                    let gap = delta[z].sub(slope.mul(delta[q])).mul(Iv::point(side));
                    if !gap.finite() {
                        continue;
                    }
                    // Bound contact-location uncertainty by ARC LENGTH at
                    // the caller's linear tolerance, never by the weld band.
                    // The strict rate also refuses every grazing contact.
                    let uncertainty = ((-gap.0).max(0.) / rate.0 * speed).next_up();
                    if !uncertainty.is_finite() || uncertainty > tolerance {
                        continue;
                    }
                    if isolated(face, c, index, patch, origin, reach) {
                        return Some(true);
                    }
                }
            }
        }
        Some(false)
    };
    result().unwrap_or(false)
}
