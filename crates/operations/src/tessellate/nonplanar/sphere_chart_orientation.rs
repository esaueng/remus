//! Signed native vector-area qualification for a hemisphere chart.
//!
//! Boundary containment alone also describes the complementary patch. For
//! native central conics the directed area integral, rather than an endpoint
//! polygon, determines whether the bounded side has the selected orientation.

use remus_math::surfaces::SphericalSurface;
use remus_topology::Topology;
use remus_topology::wire::OrientedEdge;

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
    fn add(self, other: Self) -> Self {
        if !self.finite() || !other.finite() {
            return Self::unknown();
        }
        Self {
            lo: (self.lo + other.lo).next_down(),
            hi: (self.hi + other.hi).next_up(),
        }
    }
    fn neg(self) -> Self {
        Self {
            lo: -self.hi,
            hi: -self.lo,
        }
    }
    fn sub(self, other: Self) -> Self {
        self.add(other.neg())
    }
    fn mul(self, other: Self) -> Self {
        if !self.finite() || !other.finite() {
            return Self::unknown();
        }
        let products = [
            self.lo * other.lo,
            self.lo * other.hi,
            self.hi * other.lo,
            self.hi * other.hi,
        ];
        if products.iter().any(|v| !v.is_finite()) {
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
    fn abs_max(self) -> f64 {
        self.lo.abs().max(self.hi.abs())
    }
}

pub(super) fn qualifies(
    topo: &Topology,
    sphere: &SphericalSurface,
    edges: &[OrientedEdge],
    side: f64,
) -> Result<bool, crate::OperationsError> {
    let mut area = I::point(0.0);
    let mut uncertainty = I::point(0.0);
    for oriented in edges {
        let edge = topo.edge(oriented.edge())?;
        let Some(conic) = super::sphere_boundary_conic(edge.curve()) else {
            return Ok(false);
        };
        let (a, b) = crate::authoritative_edge_domain(edge, "spherical chart orientation")?;
        let mut delta = I::point(b).sub(I::point(a));
        if !oriented.is_forward() {
            delta = delta.neg();
        }
        let u = conic.u.0.map(I::point);
        let v = conic.v.0.map(I::point);
        let axis = sphere.z_axis().0.map(I::point);
        let cross: [I; 3] = std::array::from_fn(|j| {
            let k = (j + 1) % 3;
            let l = (j + 2) % 3;
            u[k].mul(v[l]).sub(u[l].mul(v[k]))
        });
        let projection = (0..3)
            .map(|j| cross[j].mul(axis[j]))
            .fold(I::point(0.0), I::add);
        area = area.add(
            I::point(conic.a)
                .mul(I::point(conic.b))
                .mul(delta)
                .mul(projection),
        );

        // Relative to the sphere centre, the remaining integral is
        // C × (P(b)-P(a)). Bound it uniformly using the actual coefficients;
        // this includes noncentral-but-qualified supports without libm
        // endpoint evaluations or a hidden edge-count summation allowance.
        let center: [I; 3] = std::array::from_fn(|j| {
            I::point(conic.center.0[j]).sub(I::point(sphere.center().0[j]))
        });
        let span: [I; 3] = std::array::from_fn(|j| {
            I::point(2.0).mul(
                I::point(I::point(conic.a).mul(u[j]).abs_max())
                    .add(I::point(I::point(conic.b).mul(v[j]).abs_max())),
            )
        });
        for j in 0..3 {
            let k = (j + 1) % 3;
            let l = (j + 2) % 3;
            uncertainty = uncertainty.add(
                I::point(axis[j].abs_max()).mul(
                    I::point(center[k].abs_max())
                        .mul(span[l])
                        .add(I::point(center[l].abs_max()).mul(span[k])),
                ),
            );
        }
    }
    let directed = area.mul(I::point(side));
    Ok(directed.finite() && uncertainty.finite() && directed.lo > uncertainty.hi)
}
