//! Bounded native Circle latitude domains, independent of trim sampling.
//!
//! A complete one-carrier ring selects a spherical cap by its directed native
//! turn. One oppositely wound contained latitude ring clips that cap to a band.
//! Unknown carrier/trim/endpoint readings keep the historical integration path.

// These comparisons establish stored carrier/native-domain identity.
#![allow(clippy::float_cmp)]

use crate::CheckError;
use remus_math::curves::Circle3D;
use remus_math::surfaces::SphericalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::Face;
use remus_topology::wire::WireId;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

#[derive(Clone, Copy)]
struct I {
    lo: f64,
    hi: f64,
}
impl I {
    const fn point(value: f64) -> Self {
        Self {
            lo: value,
            hi: value,
        }
    }
    const fn unknown() -> Self {
        Self {
            lo: f64::NEG_INFINITY,
            hi: f64::INFINITY,
        }
    }
    fn finite(self) -> bool {
        self.lo.is_finite() && self.hi.is_finite() && self.lo <= self.hi
    }
    fn add(self, b: Self) -> Self {
        if !self.finite() || !b.finite() {
            return Self::unknown();
        }
        Self {
            lo: (self.lo + b.lo).next_down(),
            hi: (self.hi + b.hi).next_up(),
        }
    }
    fn neg(self) -> Self {
        Self {
            lo: -self.hi,
            hi: -self.lo,
        }
    }
    fn sub(self, b: Self) -> Self {
        self.add(b.neg())
    }
    fn mul(self, b: Self) -> Self {
        if !self.finite() || !b.finite() {
            return Self::unknown();
        }
        let products = [
            self.lo * b.lo,
            self.lo * b.hi,
            self.hi * b.lo,
            self.hi * b.hi,
        ];
        if products.iter().any(|x| !x.is_finite()) {
            return Self::unknown();
        }
        Self {
            lo: products
                .into_iter()
                .fold(f64::INFINITY, f64::min)
                .next_down(),
            hi: products
                .into_iter()
                .fold(f64::NEG_INFINITY, f64::max)
                .next_up(),
        }
    }
    fn div(self, b: f64) -> Self {
        if !self.finite() || !b.is_finite() || b <= 0.0 {
            return Self::unknown();
        }
        Self {
            lo: (self.lo / b).next_down(),
            hi: (self.hi / b).next_up(),
        }
    }
    fn abs(self) -> Self {
        if !self.finite() {
            return Self::unknown();
        }
        Self {
            lo: if self.lo <= 0.0 && self.hi >= 0.0 {
                0.0
            } else {
                self.lo.abs().min(self.hi.abs())
            },
            hi: self.lo.abs().max(self.hi.abs()),
        }
    }
    fn square(self) -> Self {
        let absolute = self.abs();
        if !absolute.finite() {
            return Self::unknown();
        }
        Self {
            lo: (absolute.lo * absolute.lo).next_down().max(0.0),
            hi: (absolute.hi * absolute.hi).next_up(),
        }
    }
    fn sqrt(self) -> Self {
        if !self.finite() || self.lo < 0.0 {
            return Self::unknown();
        }
        Self {
            lo: self.lo.sqrt().next_down().max(0.0),
            hi: self.hi.sqrt().next_up(),
        }
    }
    fn midpoint(self) -> f64 {
        f64::midpoint(self.lo, self.hi)
    }
}
type V = [I; 3];
fn vector(v: Vec3) -> V {
    v.0.map(I::point)
}
fn difference(a: Point3, b: Point3) -> V {
    std::array::from_fn(|j| I::point(a.0[j]).sub(I::point(b.0[j])))
}
fn add(a: V, b: V) -> V {
    std::array::from_fn(|j| a[j].add(b[j]))
}
fn sub(a: V, b: V) -> V {
    std::array::from_fn(|j| a[j].sub(b[j]))
}
fn scale(a: V, b: I) -> V {
    a.map(|x| x.mul(b))
}
fn dot(a: V, b: V) -> I {
    a.into_iter()
        .zip(b)
        .map(|(x, y)| x.mul(y))
        .fold(I::point(0.0), I::add)
}
fn norm1(a: V) -> I {
    a.into_iter().map(I::abs).fold(I::point(0.0), I::add)
}
fn cross(a: V, b: V) -> V {
    std::array::from_fn(|j| {
        let k = (j + 1) % 3;
        let l = (j + 2) % 3;
        a[k].mul(b[l]).sub(a[l].mul(b[k]))
    })
}
fn tau() -> I {
    I {
        lo: TAU.next_down(),
        hi: TAU.next_up(),
    }
}

fn sin_cos(parameter: f64) -> Option<(I, I)> {
    if !parameter.is_finite() || parameter.abs() > 64.0 * PI {
        return None;
    }
    let turns = (parameter / FRAC_PI_2).round();
    let x = I::point(parameter).sub(
        I {
            lo: FRAC_PI_2.next_down(),
            hi: FRAC_PI_2.next_up(),
        }
        .mul(I::point(turns)),
    );
    if !x.finite() || x.abs().hi > 1.0 {
        return None;
    }
    let square = x.square();
    let (mut st, mut ct) = (x, I::point(1.0));
    let (mut sine, mut cosine) = (st, ct);
    // Degree39/38 on |x|<=1: omitted terms<2e-48, enclosed by1e-30.
    for k in 1..20 {
        st = st.mul(square).neg().div(f64::from((2 * k) * (2 * k + 1)));
        ct = ct.mul(square).neg().div(f64::from((2 * k) * (2 * k - 1)));
        sine = sine.add(st);
        cosine = cosine.add(ct);
    }
    let remainder = I {
        lo: -1e-30,
        hi: 1e-30,
    };
    sine = sine.add(remainder);
    cosine = cosine.add(remainder);
    let q = turns.rem_euclid(4.0);
    let result = if q < 0.5 {
        (sine, cosine)
    } else if q < 1.5 {
        (cosine, sine.neg())
    } else if q < 2.5 {
        (sine.neg(), cosine.neg())
    } else {
        (cosine.neg(), sine)
    };
    (result.0.finite() && result.1.finite()).then_some(result)
}
fn native_point(c: &Circle3D, t: f64) -> Option<V> {
    let (sine, cosine) = sin_cos(t)?;
    Some(add(
        c.center().0.map(I::point),
        scale(
            add(
                scale(vector(c.u_axis()), cosine),
                scale(vector(c.v_axis()), sine),
            ),
            I::point(c.radius()),
        ),
    ))
}

#[derive(Clone, Copy)]
struct Latitude {
    height: I,
    winding: f64,
}

fn ring(
    topo: &Topology,
    wire: WireId,
    sphere: &SphericalSurface,
) -> Result<Option<Latitude>, CheckError> {
    let wire = topo.wire(wire)?;
    let edges = wire.edges();
    if !wire.is_closed() || edges.is_empty() || edges.len() > super::MAX_TRIM_POINTS {
        return Ok(None);
    }
    let EdgeCurve::Circle(circle) = topo.edge(edges[0].edge())?.curve() else {
        return Ok(None);
    };
    if !circle.radius().is_finite() || circle.radius() <= 0.0 {
        return Ok(None);
    }
    let budget = remus_math::tolerance::Tolerance::new().linear / 64.0;
    let u = vector(circle.u_axis());
    let v = vector(circle.v_axis());
    let z = vector(sphere.z_axis());
    let gram = dot(u, u)
        .sub(I::point(1.0))
        .abs()
        .add(dot(v, v).sub(I::point(1.0)).abs())
        .add(dot(u, v).abs().mul(I::point(2.0)));
    let center = difference(circle.center(), sphere.center());
    let height = dot(center, z);
    let radial = I::point(sphere.radius())
        .square()
        .sub(height.square())
        .sqrt();
    let perpendicular = norm1(sub(center, scale(z, height)));
    let latitude_error = dot(u, z)
        .abs()
        .add(dot(v, z).abs())
        .mul(I::point(circle.radius()));
    let error = perpendicular
        .add(gram.mul(I::point(circle.radius())))
        .add(I::point(circle.radius()).sub(radial).abs())
        .add(latitude_error);
    let alignment = dot(cross(u, v), z);
    if !gram.finite()
        || gram.hi > 1e-12
        || !error.finite()
        || error.hi > budget
        || !alignment.finite()
    {
        return Ok(None);
    }
    let frame_sign = if alignment.lo > 0.5 {
        1.0
    } else if alignment.hi < -0.5 {
        -1.0
    } else {
        return Ok(None);
    };
    let endpoint_budget = budget / edges.len() as f64;
    let mut direction = 0.0;
    let mut turn = I::point(0.0);
    let mut first_parameter = None;
    let mut previous = None;
    for (index, oriented) in edges.iter().enumerate() {
        let edge = topo.edge(oriented.edge())?;
        let next = topo.edge(edges[(index + 1) % edges.len()].edge())?;
        let EdgeCurve::Circle(c) = edge.curve() else {
            return Ok(None);
        };
        if c.center() != circle.center()
            || c.radius() != circle.radius()
            || c.u_axis() != circle.u_axis()
            || c.v_axis() != circle.v_axis()
            || oriented.oriented_end(edge) != edges[(index + 1) % edges.len()].oriented_start(next)
        {
            return Ok(None);
        }
        let Ok((a, b)) = edge.strict_domain() else {
            return Ok(None);
        };
        let (a, b) = if oriented.is_forward() {
            (a, b)
        } else {
            (b, a)
        };
        let delta = I::point(b).sub(I::point(a));
        if !delta.finite() || delta.abs().hi > TAU.next_up().next_up() {
            return Ok(None);
        }
        let sign = if delta.lo > 0.0 {
            1.0
        } else if delta.hi < 0.0 {
            -1.0
        } else {
            return Ok(None);
        };
        if direction != 0.0 && sign != direction {
            return Ok(None);
        }
        direction = sign;
        turn = turn.add(delta);
        if let Some(previous) = previous {
            if !periodic_continuity(previous, a, circle.radius(), endpoint_budget) {
                return Ok(None);
            }
        } else {
            first_parameter = Some(a);
        }
        previous = Some(b);
        for (t, vertex) in [
            (a, oriented.oriented_start(edge)),
            (b, oriented.oriented_end(edge)),
        ] {
            let Some(point) = native_point(circle, t) else {
                return Ok(None);
            };
            let distance = norm1(sub(point, topo.vertex(vertex)?.point().0.map(I::point)));
            if !distance.finite() || distance.hi > endpoint_budget {
                return Ok(None);
            }
        }
    }
    let turn_angle_error = turn.sub(tau().mul(I::point(direction))).abs();
    let turn_error = turn_angle_error.mul(I::point(circle.radius()));
    if !turn_angle_error.finite()
        || turn_angle_error.hi > 128.0 * f64::EPSILON * TAU * edges.len() as f64
        || !turn_error.finite()
        || turn_error.hi > budget
        || !periodic_continuity(
            previous.unwrap_or(0.0),
            first_parameter.unwrap_or(0.0),
            circle.radius(),
            endpoint_budget,
        )
    {
        return Ok(None);
    }
    Ok(Some(Latitude {
        height,
        winding: direction * frame_sign,
    }))
}

fn periodic_continuity(a: f64, b: f64, radius: f64, budget: f64) -> bool {
    let difference = I::point(a).sub(I::point(b));
    let turns = (difference.midpoint() / TAU).round();
    if !turns.is_finite() || turns.abs() > 66.0 {
        return false;
    }
    let angle_error = difference.sub(tau().mul(I::point(turns))).abs();
    let error = angle_error.mul(I::point(radius));
    angle_error.finite()
        && angle_error.hi <= 128.0 * f64::EPSILON * TAU * (1.0 + turns.abs())
        && error.finite()
        && error.hi <= budget
}

pub(super) fn domain(
    topo: &Topology,
    face: &Face,
    sphere: &SphericalSurface,
) -> Result<Option<(f64, f64)>, CheckError> {
    if face.inner_wires().len() > 1 || !sphere.radius().is_finite() || sphere.radius() <= 0.0 {
        return Ok(None);
    }
    let mut count = topo.wire(face.outer_wire())?.edges().len();
    for hole in face.inner_wires() {
        count = count.saturating_add(topo.wire(*hole)?.edges().len());
    }
    if count > super::MAX_TRIM_POINTS {
        return Ok(None);
    }
    let axes = [
        vector(sphere.x_axis()),
        vector(sphere.y_axis()),
        vector(sphere.z_axis()),
    ];
    let mut gram = I::point(0.0);
    for j in 0..3 {
        gram = gram.add(dot(axes[j], axes[j]).sub(I::point(1.0)).abs());
        for k in j + 1..3 {
            gram = gram.add(dot(axes[j], axes[k]).abs().mul(I::point(2.0)));
        }
    }
    let error = gram.mul(I::point(sphere.radius()));
    if !gram.finite()
        || gram.hi > 1e-12
        || !error.finite()
        || error.hi > remus_math::tolerance::Tolerance::new().linear / 64.0
        || dot(cross(axes[0], axes[1]), axes[2]).lo < 0.5
    {
        return Ok(None);
    }
    let Some(outer) = ring(topo, face.outer_wire(), sphere)? else {
        return Ok(None);
    };
    let start = outer.height.midpoint();
    let far = if let Some(hole) = face.inner_wires().first() {
        let Some(hole) = ring(topo, *hole, sphere)? else {
            return Ok(None);
        };
        if hole.winding != -outer.winding
            || if outer.winding > 0.0 {
                hole.height.lo <= outer.height.hi || hole.height.hi >= sphere.radius()
            } else {
                hole.height.hi >= outer.height.lo || hole.height.lo <= -sphere.radius()
            }
        {
            return Ok(None);
        }
        hole.height.midpoint()
    } else {
        outer.winding * sphere.radius()
    };
    let start = (start / sphere.radius()).asin();
    let far = (far / sphere.radius()).asin();
    if !start.is_finite() || !far.is_finite() || start == far {
        return Ok(None);
    }
    Ok(Some((start.min(far), start.max(far))))
}

#[cfg(test)]
mod tests;

mod partition_sampling;

pub(super) struct PartitionSamples {
    pub(super) outer: Vec<Point3>,
    pub(super) holes: Vec<Vec<Point3>>,
}

pub(super) fn partition_samples(
    topo: &Topology,
    face: &Face,
    sphere: &SphericalSurface,
) -> Result<Option<PartitionSamples>, CheckError> {
    partition_sampling::build(topo, face, sphere)
}
