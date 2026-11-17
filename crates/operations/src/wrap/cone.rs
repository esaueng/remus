//! Wrap a planar profile onto a cone, preserving arc length (OpenZCAD M12).
//!
//! A cone is developable: cut along a generator and laid flat about its
//! apex, it is a plane sector, and the development keeps every length on the
//! surface. The profile is read in that development. It lies in a plane
//! parallel to the cone's axis; a point `p` has an arc-length coordinate
//! `s = (p − O)·b` (with `b = a × n`, `n` the plane's normal) and a
//! coordinate `z = (p − O)·a` taken as length along the generator, where `O`
//! is a point on the axis. The developed apex sits at `(0, −ρ₀)`, `ρ₀` being
//! the slant length from the apex to the circle at `O`'s height, so a
//! profile point lies at distance `ρ = |(s, z + ρ₀)|` from it and at angle
//! `ψ = atan2(s, z + ρ₀)` from the generator through `e` (the reference
//! direction, angle zero). On the cone, with the remus half-angle `β`
//! (generator `cos β·r + sin β·a`) and `θ = ψ / cos β`:
//!
//! ```text
//! W_d(s, z) = A − (d / cos β)·a + (ρ + d·tan β)·(cos β·r(θ) + sin β·a)
//! ```
//!
//! is the point at normal offset `d` from the cone (`r(θ) = cos θ·e + sin θ·c`,
//! `c = a × e`); every `W_d` lies on the cone offset by `d`, itself an exact
//! cone with the same axis and half-angle. The solid is every `W_d(s, z)`
//! with `(s, z)` in the profile and `d` between `inner` and `outer`:
//!
//! - its inner and outer faces are exact cones;
//! - every profile edge wraps to a **certified** cubic B-spline at each
//!   offset: the fit's distance from the exact wrapped curve is proven from a
//!   bound on its fourth derivative. The map is homogeneous in the developed
//!   plane, so its `j`-th derivatives are bounded in closed form by
//!   `ρ^(1−j)` (and `ρ^(−j)` for the offset term) from Wirtinger derivatives
//!   of `w^p·w̄^q` with `w = (z + ρ₀) + i·s`; `ρ` is bounded below on each
//!   edge from its samples and its first-derivative bound. The two fits of an
//!   edge share their spans, and its wall is the exact ruled surface between
//!   them: every ruling is the normal segment, so the wall is within the
//!   larger fit bound of the true wall.
//!
//! A profile that reaches the developed apex's level (`z + ρ₀ ≤ 0`), one
//! that spans a full turn of the cone, an offset that passes an apex, and a
//! degenerate cone or offset range are refused by name; every refusal
//! leaves the topology unchanged.

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::surfaces::ConicalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use super::{
    EXTENT_SAMPLES, PARALLEL_TOLERANCE, Planar, RELATIVE_FIT_TOLERANCE, SPAN_SAMPLES, Span, Unroll,
    Unrolled, certified_range, fit_spans, invalid, unroll_piece,
};
use crate::OperationsError;
use crate::helix::in_place::Loop;

/// The cone a profile is wrapped onto.
#[derive(Debug, Clone, Copy)]
pub struct WrapCone {
    /// The cone's apex.
    pub apex: Point3,
    /// The axis direction, from the apex into the cone.
    pub axis: Vec3,
    /// The direction across the axis at which arc length zero lies.
    pub reference: Vec3,
    /// The remus half-angle `β` (radians, in `(0, π/2)`): the angle from the
    /// radial plane to a generator.
    pub half_angle: f64,
    /// A point on the axis beyond the apex: the profile's `z = 0` is the
    /// circle at its height.
    pub origin: Point3,
}

/// The wrap's frame and its map from the profile plane onto the cone.
#[derive(Debug, Clone, Copy)]
struct ConeFrame {
    apex: Point3,
    a: Vec3,
    e: Vec3,
    c: Vec3,
    sin_b: f64,
    cos_b: f64,
    /// `ρ₀`: the slant length from the apex to `origin`'s circle.
    slant: f64,
    origin: Point3,
    across: Vec3,
}

impl Unroll for ConeFrame {
    fn unroll(&self, p: Point3) -> Planar {
        let d = p - self.origin;
        (d.dot(self.across), d.dot(self.a))
    }

    fn unroll_vec(&self, v: Vec3) -> Planar {
        (v.dot(self.across), v.dot(self.a))
    }
}

impl ConeFrame {
    /// `(ρ, ψ)` of an unrolled point about the developed apex.
    fn develop(&self, (s, z): Planar) -> (f64, f64) {
        let y = z + self.slant;
        (s.hypot(y), s.atan2(y))
    }

    fn angle(&self, psi: f64) -> f64 {
        psi / self.cos_b
    }

    fn radial(&self, theta: f64) -> Vec3 {
        let (sin, cos) = theta.sin_cos();
        self.e * cos + self.c * sin
    }

    fn generator(&self, theta: f64) -> Vec3 {
        self.radial(theta) * self.cos_b + self.a * self.sin_b
    }

    /// The apex of the cone offset by `d` along the outward normal.
    fn apex_at(&self, d: f64) -> Point3 {
        self.apex - self.a * (d / self.cos_b)
    }

    /// `W_d(s, z)`.
    fn wrap(&self, q: Planar, d: f64) -> Point3 {
        let (rho, psi) = self.develop(q);
        let tan_b = self.sin_b / self.cos_b;
        self.apex_at(d) + self.generator(self.angle(psi)) * (rho + d * tan_b)
    }

    /// `W_d` and its derivative along a curve with `(s, z)` and `(s', z')`.
    fn wrap_with_derivative(&self, q: Planar, dq: Planar, d: f64) -> (Point3, Vec3) {
        let (x, y) = (q.0, q.1 + self.slant);
        let (rho, psi) = self.develop(q);
        let theta = self.angle(psi);
        let d_rho = (x * dq.0 + y * dq.1) / rho;
        let d_psi = (y * dq.0 - x * dq.1) / (rho * rho);
        let (sin, cos) = theta.sin_cos();
        let turn = self.c * cos - self.e * sin;
        let tan_b = self.sin_b / self.cos_b;
        // dθ = dψ / cos β, and the generator turns by cos β · r'(θ).
        (
            self.wrap(q, d),
            self.generator(theta) * d_rho + turn * ((rho + d * tan_b) * d_psi),
        )
    }

    /// A bound on `‖D^j W_d‖` (as a `j`-linear map on unit vectors) where the
    /// developed distance from the apex is at least `rho_min`.
    fn derivative_bound(&self, j: u32, rho_min: f64, d: f64) -> f64 {
        let k = 1.0 / self.cos_b;
        let rho = rho_min;
        let r1 = rho.powi(1 - i32::try_from(j).unwrap_or(i32::MAX));
        let r0 = rho.powi(-i32::try_from(j).unwrap_or(i32::MAX));
        self.sin_b * wirtinger(j, 0.5, 0.5) * r1
            + self.cos_b * wirtinger(j, 0.5 * (1.0 + k), 0.5 * (1.0 - k)) * r1
            + d.abs() * self.sin_b * wirtinger(j, 0.5 * k, -0.5 * k) * r0
    }
}

/// `Σ_p C(j, p)·|(a)_p|·|(b)_(j−p)|` with falling factorials: the bound on
/// a `j`-th real derivative of `w^a·w̄^b`, over `|w|^(a+b−j)`.
fn wirtinger(j: u32, a: f64, b: f64) -> f64 {
    let falling = |x: f64, n: u32| (0..n).map(|i| x - f64::from(i)).product::<f64>().abs();
    (0..=j)
        .map(|p| {
            let binomial = (0..p).fold(1.0, |acc, i| acc * f64::from(j - i) / f64::from(i + 1));
            binomial * falling(a, p) * falling(b, j - p)
        })
        .sum()
}

/// A lower bound on `z + ρ₀` along a span (and so on its distance from the
/// developed apex): its least sample less the most its first-derivative
/// bound allows between samples.
#[allow(clippy::cast_precision_loss)]
fn least_height(span: &Span<'_>, slant: f64) -> f64 {
    let n = SPAN_SAMPLES;
    let least = (0..=n)
        .map(|i| (span.sample)(i as f64 / n as f64).0.1 + slant)
        .fold(f64::INFINITY, f64::min);
    least - span.bounds[0] / (2.0 * n as f64)
}

/// One wrapped profile edge: its fits at the inner and outer offsets, with
/// the same spans, and the larger of their certified bounds.
struct Fitted {
    inner: remus_math::nurbs::curve::NurbsCurve,
    outer: remus_math::nurbs::curve::NurbsCurve,
    bound: f64,
}

fn fit_piece(
    unrolled: &Unrolled<'_>,
    frame: &ConeFrame,
    inner: f64,
    outer: f64,
    tolerance: f64,
) -> Result<Fitted, OperationsError> {
    // `‖q^(i)‖ ≤ √2·bounds[i−1]`, then Faà di Bruno at order four, on each
    // span with its own least distance from the apex. One bound serves both
    // offsets, so both fits take the same spans.
    let fourth = |span: &Span<'_>| {
        let rho_min = least_height(span, frame.slant);
        let [d1, d2, d3, d4] = span.bounds.map(|b| std::f64::consts::SQRT_2 * b);
        let at = |d: f64| {
            let w = |j| frame.derivative_bound(j, rho_min, d);
            w(1) * d4
                + w(2) * (4.0 * d1 * d3 + 3.0 * d2 * d2)
                + w(3) * 6.0 * d1 * d1 * d2
                + w(4) * d1.powi(4)
        };
        at(inner).max(at(outer))
    };
    let fit = |d: f64| {
        fit_spans(
            unrolled,
            fourth,
            |q, dq| frame.wrap_with_derivative(q, dq, d),
            tolerance,
        )
    };
    let ((inner, bi), (outer, bo)) = (fit(inner)?, fit(outer)?);
    Ok(Fitted {
        bound: bi.max(bo),
        inner,
        outer,
    })
}

/// Wrap the planar face `profile` onto `cone` between normal offsets
/// `inner` and `outer` (negative is inside the cone).
///
/// The profile is read in the cone's development: its arc length along the
/// plane (`axis × normal`) and its height along the generator from the
/// circle at `origin`. The profile's plane must be parallel to the
/// axis.
///
/// # Errors
///
/// [`OperationsError::InvalidInput`] for a non-finite or degenerate cone or
/// offset range, an origin not beyond the apex, a profile plane not
/// parallel to the axis, a profile that reaches the developed apex's level or
/// spans a full turn, or an offset that passes an apex under the profile;
/// [`OperationsError::Unsupported`] for a profile curve that is not a line,
/// circle, ellipse or non-rational spline, or one whose wrap cannot be fitted
/// within the tolerance. Every error leaves the topology unchanged.
pub fn wrap_profile_on_cone(
    topo: &mut Topology,
    profile: FaceId,
    cone: &WrapCone,
    inner: f64,
    outer: f64,
) -> Result<SolidId, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        wrap_impl(topo, profile, cone, inner, outer)
    })
}

#[allow(clippy::too_many_lines)]
fn wrap_impl(
    topo: &mut Topology,
    profile: FaceId,
    cone: &WrapCone,
    inner: f64,
    outer: f64,
) -> Result<SolidId, OperationsError> {
    let linear = Tolerance::new().linear;
    let beta = cone.half_angle;
    if !(beta.is_finite() && beta > 1e-6 && beta < std::f64::consts::FRAC_PI_2 - 1e-6) {
        return Err(invalid(format!(
            "a cone's half-angle must lie strictly between 0 and 90°, got {:.6}°",
            beta.to_degrees()
        )));
    }
    if !(inner.is_finite() && outer.is_finite() && outer > inner + linear) {
        return Err(invalid(format!(
            "a cone wrap needs inner < outer offset, got {inner} and {outer}"
        )));
    }
    let a = cone.axis.normalize()?;
    let e = (cone.reference - a * cone.reference.dot(a)).normalize()?;
    let (sin_b, cos_b) = beta.sin_cos();
    let slant = (cone.origin - cone.apex).dot(a) / sin_b;
    if !(slant.is_finite() && slant > linear) {
        return Err(invalid(
            "a cone wrap's origin must lie on the axis beyond the apex",
        ));
    }

    let face = topo.face(profile)?;
    let FaceSurface::Plane { normal, d } = *face.surface() else {
        return Err(invalid("a wrapped profile must be planar"));
    };
    // The plane's normal turned to face away from the axis, so a region
    // lands under where it is drawn whichever way its sketch faces.
    let length = normal.length();
    let normal = normal.normalize()?;
    let normal = if d / length
        - normal.dot(Vec3::new(cone.origin.x(), cone.origin.y(), cone.origin.z()))
        < 0.0
    {
        -normal
    } else {
        normal
    };
    if normal.dot(a).abs() > PARALLEL_TOLERANCE {
        return Err(invalid(
            "a wrapped profile must lie in a plane parallel to the cone's axis",
        ));
    }
    let frame = ConeFrame {
        apex: cone.apex,
        a,
        e,
        c: a.cross(e),
        sin_b,
        cos_b,
        slant,
        origin: cone.origin,
        across: a.cross(normal).normalize()?,
    };
    let outer_wire = face.outer_wire();
    let inner_wires = face.inner_wires().to_vec();
    let mut loops: Vec<Loop> = vec![super::read_wrap_loop(topo, outer_wire, true, "cone wrap")?];
    for wire in inner_wires {
        loops.push(super::read_wrap_loop(topo, wire, false, "cone wrap")?);
    }
    let unrolled: Vec<Vec<Unrolled<'_>>> = loops
        .iter()
        .map(|lp| {
            lp.pieces
                .iter()
                .map(|piece| unroll_piece(piece, &frame))
                .collect::<Result<_, _>>()
        })
        .collect::<Result<_, _>>()?;

    // Beyond the developed apex's level, within one turn, and clear of
    // every offset cone's apex.
    let lowest = unrolled
        .iter()
        .flatten()
        .flat_map(|u| u.spans.iter())
        .map(|span| least_height(span, slant))
        .fold(f64::INFINITY, f64::min);
    if lowest.is_nan() || lowest <= linear {
        return Err(invalid(
            "the wrapped profile reaches the cone's apex; keep it beyond the apex",
        ));
    }
    let tan_b = sin_b / cos_b;
    if lowest + inner.min(0.0) * tan_b <= linear {
        return Err(invalid(format!(
            "the inner offset {inner} passes the apex of the offset cone under the profile"
        )));
    }
    // Certified from samples and a bound on how fast the angle can move
    // between them: `|dθ/dt| ≤ |q'| / (ρ cos β)`.
    let all: Vec<&Unrolled<'_>> = unrolled.iter().flatten().collect();
    let (low, high) = certified_range(
        &all,
        |q| q.0.atan2(q.1 + slant) / cos_b,
        |span| std::f64::consts::SQRT_2 * span.bounds[0] / (least_height(span, slant) * cos_b),
    );
    let extent = high - low;
    if extent.is_nan() || extent >= std::f64::consts::TAU - 1e-6 {
        return Err(invalid(format!(
            "the wrapped profile spans up to {:.3}° around the cone; it must stay within one turn",
            extent.to_degrees()
        )));
    }
    let seam_dir = frame.radial(low - 0.5 * (std::f64::consts::TAU - extent));

    let scale = (slant + outer.abs()).max(1.0);
    let fit_tolerance = (RELATIVE_FIT_TOLERANCE * scale).max(linear);
    let fitted: Vec<Vec<Fitted>> = unrolled
        .iter()
        .map(|pieces| {
            pieces
                .iter()
                .map(|u| fit_piece(u, &frame, inner, outer, fit_tolerance))
                .collect::<Result<_, _>>()
        })
        .collect::<Result<_, _>>()?;

    let mut walls = Vec::new();
    let mut outer_wires: Vec<WireId> = Vec::new();
    let mut inner_loop_wires: Vec<WireId> = Vec::new();
    for (l, lp) in loops.iter().enumerate() {
        let points: Vec<Planar> = lp.points.iter().map(|&(p, _)| frame.unroll(p)).collect();
        // Winding in (s, z): right-handed about the outward normal.
        #[allow(clippy::cast_precision_loss)]
        let samples: Vec<Planar> = unrolled[l]
            .iter()
            .flat_map(|u| {
                (0..EXTENT_SAMPLES).map(move |k| {
                    let x = k as f64 / EXTENT_SAMPLES as f64;
                    (u.sample)(if u.piece.forward { x } else { 1.0 - x }).0
                })
            })
            .collect();
        let area: f64 = samples
            .iter()
            .zip(samples.iter().cycle().skip(1))
            .map(|(p, q)| p.0 * q.1 - q.0 * p.1)
            .sum();
        let ccw = area > 0.0;
        let side = if ccw == lp.outer { 1.0 } else { -1.0 };

        let bound_at = |i: usize| -> f64 {
            fitted[l]
                .iter()
                .enumerate()
                .filter(|&(j, _)| {
                    let (ns, ne) = lp.natural_ends(j);
                    ns == i || ne == i
                })
                .map(|(_, f)| f.bound)
                .fold(0.0, f64::max)
        };
        let vertex = |topo: &mut Topology, i: usize, d: f64| {
            let tol = lp.points[i].1.max(linear).max(2.0 * bound_at(i));
            topo.add_vertex(Vertex::new(frame.wrap(points[i], d), tol))
        };
        let inner_v: Vec<VertexId> = (0..points.len()).map(|i| vertex(topo, i, inner)).collect();
        let outer_v: Vec<VertexId> = (0..points.len()).map(|i| vertex(topo, i, outer)).collect();
        let normals: Vec<EdgeId> = (0..points.len())
            .map(|i| topo.add_edge(Edge::new(inner_v[i], outer_v[i], EdgeCurve::Line)))
            .collect();

        let mut inner_edges = Vec::with_capacity(lp.pieces.len());
        let mut outer_edges = Vec::with_capacity(lp.pieces.len());
        for (j, (piece, fit)) in lp.pieces.iter().zip(&fitted[l]).enumerate() {
            let (ns, ne) = lp.natural_ends(j);
            for (curve, store, vs) in [
                (&fit.inner, &mut inner_edges, &inner_v),
                (&fit.outer, &mut outer_edges, &outer_v),
            ] {
                let domain = curve.domain();
                let mut edge = Edge::new(vs[ns], vs[ne], EdgeCurve::NurbsCurve(curve.clone()));
                edge.set_trim(Some(domain));
                edge.set_tolerance(Some(fit.bound.max(linear)))?;
                store.push(topo.add_edge(edge));
            }

            // The wall: ruled along the normal between the two fits,
            // oriented away from the material.
            let u = &unrolled[l][j];
            let (qm, dqm) = (u.sample)(0.5);
            let dir = if piece.forward { 1.0 } else { -1.0 };
            let (ts, tz) = (dqm.0 * dir, dqm.1 * dir);
            let away = (tz * side, -ts * side);
            let mid = 0.5 * (inner + outer);
            let outward = frame.wrap_with_derivative(qm, away, mid).1;
            let rows = fit
                .inner
                .control_points()
                .iter()
                .zip(fit.outer.control_points())
                .map(|(&p, &q)| vec![p, q])
                .collect();
            let weights = fit.inner.weights().iter().map(|&w| vec![w, w]).collect();
            let surface = NurbsSurface::new(
                fit.inner.degree(),
                1,
                fit.inner.knots().to_vec(),
                vec![0.0, 0.0, 1.0, 1.0],
                rows,
                weights,
            )?;
            let reversed = surface.normal(0.5, 0.5)?.dot(outward) < 0.0;
            let (bottom, top) = (inner_edges[j], outer_edges[j]);
            let wire = topo.add_wire(Wire::new(
                vec![
                    OrientedEdge::new(bottom, true),
                    OrientedEdge::new(normals[ne], true),
                    OrientedEdge::new(top, false),
                    OrientedEdge::new(normals[ns], false),
                ],
                true,
            )?);
            let mut face = Face::new(wire, vec![], FaceSurface::Nurbs(surface));
            if reversed {
                face.set_reversed(true);
            }
            walls.push(topo.add_face(face));
        }

        let cap_wire = |topo: &mut Topology, edges: &[EdgeId]| -> Result<WireId, OperationsError> {
            let oriented: Vec<OrientedEdge> = if ccw == lp.outer {
                edges
                    .iter()
                    .zip(&lp.pieces)
                    .map(|(&e, piece)| OrientedEdge::new(e, piece.forward))
                    .collect()
            } else {
                edges
                    .iter()
                    .zip(&lp.pieces)
                    .rev()
                    .map(|(&e, piece)| OrientedEdge::new(e, !piece.forward))
                    .collect()
            };
            Ok(topo.add_wire(Wire::new(oriented, true)?))
        };
        outer_wires.push(cap_wire(topo, &outer_edges)?);
        inner_loop_wires.push(cap_wire(topo, &inner_edges)?);
    }

    let cone_face = |topo: &mut Topology,
                     d: f64,
                     wires: &[WireId],
                     reversed: bool|
     -> Result<FaceId, OperationsError> {
        let surface = ConicalSurface::with_ref_dir(frame.apex_at(d), a, beta, seam_dir)?;
        let mut face = Face::new(wires[0], wires[1..].to_vec(), FaceSurface::Cone(surface));
        if reversed {
            face.set_reversed(true);
        }
        Ok(topo.add_face(face))
    };
    let mut faces = walls;
    faces.push(cone_face(topo, outer, &outer_wires, false)?);
    faces.push(cone_face(topo, inner, &inner_loop_wires, true)?);
    let shell = topo.add_shell(Shell::new(faces)?);
    Ok(topo.add_solid(Solid::new(shell, vec![])))
}

#[cfg(test)]
mod tests;
