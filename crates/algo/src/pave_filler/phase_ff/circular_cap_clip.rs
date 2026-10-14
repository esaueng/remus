//! Boundary-event clipping for a full ellipse on an oblique cylinder section.
//!
//! Qualified domain: one full circular planar cap without holes, and a full
//! cylindrical lateral bounded by two full circular rims and one repeated
//! ruling seam. Roots of the cap's radial quartic and the cylinder's axial
//! quadratics partition the entire native ellipse. Interval bounds prove
//! exclusion and monotonic root isolation; unresolved contacts refuse.

use std::f64::consts::{PI, TAU};

use remus_math::compact_sort::CompactSort;
use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::surfaces::CylindricalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};

use super::super::curved_section_clip::bezier_cylinder_residual_bound;
use super::{AlgoError, JunctionRegistry, OperationContext, RawCurve, Tolerance};

const MAX_ROOT_NODES: usize = 4096;
const MAX_ROOT_DEPTH: usize = 64;

fn unresolved(reason: &str) -> AlgoError {
    AlgoError::FaceSplitFailed(format!(
        "circular-cap section clipping unresolved: {reason}"
    ))
}

fn finite_vec(v: Vec3) -> bool {
    v.0.iter().all(|x| x.is_finite())
}

fn position_vector(p: Point3) -> Vec3 {
    Vec3::new(p.x(), p.y(), p.z())
}

fn orthonormal(u: Vec3, v: Vec3, n: Vec3) -> bool {
    finite_vec(u)
        && finite_vec(v)
        && finite_vec(n)
        && (u.length_squared() - 1.0).abs() <= 1e-12
        && (v.length_squared() - 1.0).abs() <= 1e-12
        && (n.length_squared() - 1.0).abs() <= 1e-12
        && u.dot(v).abs() <= 1e-12
        && u.dot(n).abs() <= 1e-12
        && v.dot(n).abs() <= 1e-12
}

fn full_circle(edge: &Edge) -> Option<&Circle3D> {
    let EdgeCurve::Circle(circle) = edge.curve() else {
        return None;
    };
    let (start, end) = edge.strict_domain().ok()?;
    (edge.is_closed()
        && ((end - start).abs() - TAU).abs() <= 32.0 * f64::EPSILON * TAU
        && circle.center().0.iter().all(|x| x.is_finite())
        && circle.radius().is_finite()
        && circle.radius() > 0.0
        && orthonormal(circle.u_axis(), circle.v_axis(), circle.normal()))
    .then_some(circle)
}

/// Qualify a planar disc from its authoritative single full-circle boundary.
/// A carrier-only circle, partial trim, hole, or off-plane circle declines.
pub(super) fn closed_disc(topo: &Topology, face_id: FaceId, tol: Tolerance) -> Option<Circle3D> {
    if !tol.linear.is_finite() || tol.linear <= 0.0 {
        return None;
    }
    let face = topo.face(face_id).ok()?;
    let FaceSurface::Plane { normal, d } = face.surface() else {
        return None;
    };
    let wire = topo.wire(face.outer_wire()).ok()?;
    if !face.inner_wires().is_empty() || !wire.is_closed() || wire.edges().len() != 1 {
        return None;
    }
    let edge = topo.edge(wire.edges()[0].edge()).ok()?;
    let circle = full_circle(edge)?;
    let amplitude = circle.radius()
        * normal
            .dot(circle.u_axis())
            .hypot(normal.dot(circle.v_axis()));
    let displacement = normal.dot(position_vector(circle.center())) - d;
    let vertex = topo.vertex(edge.start()).ok()?.point();
    if !finite_vec(*normal)
        || !d.is_finite()
        || (normal.length_squared() - 1.0).abs() > 1e-12
        || displacement.abs() + amplitude > tol.linear
        || (circle.evaluate(circle.project(vertex)) - vertex).length() > tol.linear
    {
        return None;
    }
    Some(circle.clone())
}

pub(super) fn cylinder_limits(
    topo: &Topology,
    face_id: FaceId,
    cyl: &CylindricalSurface,
    tol: Tolerance,
) -> Option<(f64, f64)> {
    let face = topo.face(face_id).ok()?;
    let wire = topo.wire(face.outer_wire()).ok()?;
    if !face.inner_wires().is_empty()
        || !wire.is_closed()
        || wire.edges().len() != 4
        || !orthonormal(cyl.x_axis(), cyl.y_axis(), cyl.axis())
        || !cyl.radius().is_finite()
        || cyl.radius() <= 0.0
    {
        return None;
    }
    let mut rims = Vec::new();
    let mut seams = Vec::new();
    for usage in wire.edges() {
        let edge = topo.edge(usage.edge()).ok()?;
        match edge.curve() {
            EdgeCurve::Circle(_) => {
                let circle = full_circle(edge)?;
                let delta = circle.center() - cyl.origin();
                let v = delta.dot(cyl.axis());
                let radial = delta - cyl.axis() * v;
                if radial.length() > tol.linear
                    || (circle.radius() - cyl.radius()).abs() > tol.linear
                    || circle.radius() * circle.normal().cross(cyl.axis()).length() > tol.linear
                {
                    return None;
                }
                let vertex = topo.vertex(edge.start()).ok()?.point();
                if (circle.evaluate(circle.project(vertex)) - vertex).length() > tol.linear {
                    return None;
                }
                rims.push((v, edge.start()));
            }
            EdgeCurve::Line => seams.push((usage.edge(), usage.is_forward())),
            EdgeCurve::Ellipse(_)
            | EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => return None,
        }
    }
    if rims.len() != 2 || seams.len() != 2 || seams[0].0 != seams[1].0 || seams[0].1 == seams[1].1 {
        return None;
    }
    let seam = topo.edge(seams[0].0).ok()?;
    if !((seam.start() == rims[0].1 && seam.end() == rims[1].1)
        || (seam.start() == rims[1].1 && seam.end() == rims[0].1))
    {
        return None;
    }
    let seam_delta =
        topo.vertex(seam.end()).ok()?.point() - topo.vertex(seam.start()).ok()?.point();
    if (seam_delta - cyl.axis() * seam_delta.dot(cyl.axis())).length() > tol.linear {
        return None;
    }
    for vertex in [seam.start(), seam.end()] {
        let delta = topo.vertex(vertex).ok()?.point() - cyl.origin();
        let radial = delta - cyl.axis() * delta.dot(cyl.axis());
        if (radial.length() - cyl.radius()).abs() > tol.linear {
            return None;
        }
    }
    let limits = (rims[0].0.min(rims[1].0), rims[0].0.max(rims[1].0));
    (limits.0.is_finite() && limits.1.is_finite() && limits.1 - limits.0 > tol.linear)
        .then_some(limits)
}

/// Find the actual full planar cap that owns each lateral rim. Shared edge
/// identity is required: an unrelated coplanar disc is not an endpoint source.
fn owned_caps(
    topo: &Topology,
    lateral: FaceId,
    context: &OperationContext,
) -> Result<Option<Vec<FaceId>>, AlgoError> {
    let face = topo.face(lateral)?;
    let wire = topo.wire(face.outer_wire())?;
    let mut caps = Vec::new();
    for usage in wire.edges() {
        let rim = usage.edge();
        if !matches!(topo.edge(rim)?.curve(), EdgeCurve::Circle(_)) {
            continue;
        }
        let mut owner = None;
        for (candidate, face) in topo.faces().iter() {
            context.check_cancelled()?;
            if candidate == lateral || !matches!(face.surface(), FaceSurface::Plane { .. }) {
                continue;
            }
            let boundary = topo.wire(face.outer_wire())?;
            if boundary.edges().len() != 1
                || boundary.edges()[0].edge() != rim
                || closed_disc(topo, candidate, context.tolerance).is_none()
            {
                continue;
            }
            if owner.replace(candidate).is_some() {
                return Ok(None);
            }
        }
        let Some(owner) = owner else {
            return Ok(None);
        };
        caps.push(owner);
    }
    Ok((caps.len() == 2).then_some(caps))
}

/// Independently solve cap-rim/other-cylinder junctions before the lateral
/// marcher needs them. This pool is separate from approximate earlier paves;
/// every point comes from a uniquely owned cap and an isolated radial event.
/// The consumer still proves both cylinder carriers and endpoint identity.
pub(super) fn early_cylinder_cap_junctions(
    topo: &Topology,
    first_id: FaceId,
    second_id: FaceId,
    context: &OperationContext,
) -> Result<Option<Vec<Point3>>, AlgoError> {
    let ids = [first_id, second_id];
    let surfaces = [
        topo.face(first_id)?.surface(),
        topo.face(second_id)?.surface(),
    ];
    let (FaceSurface::Cylinder(first), FaceSurface::Cylinder(second)) = (surfaces[0], surfaces[1])
    else {
        return Ok(None);
    };
    let limits = [
        cylinder_limits(topo, first_id, first, context.tolerance),
        cylinder_limits(topo, second_id, second, context.tolerance),
    ];
    let [Some(first_limits), Some(second_limits)] = limits else {
        return Ok(None);
    };
    let limits = [first_limits, second_limits];
    let cosine = first.axis().dot(second.axis()).abs();
    if !cosine.is_finite() || cosine <= 1e-10 || cosine >= 1.0 - 1e-10 {
        return Ok(None);
    }
    let [Some(first_caps), Some(second_caps)] = [
        owned_caps(topo, first_id, context)?,
        owned_caps(topo, second_id, context)?,
    ] else {
        return Ok(None);
    };
    let caps = [first_caps, second_caps];
    let mut points = Vec::new();
    for owner in 0..2 {
        let other = 1 - owner;
        let other_box = super::compute_face_bbox(topo, ids[other], context.tolerance)?;
        for &cap in &caps[owner] {
            context.check_cancelled()?;
            let cap_surface = topo.face(cap)?.surface();
            let cap_box = super::compute_face_bbox(topo, cap, context.tolerance)?;
            let raw = super::compute_raw_curves(
                cap_surface,
                surfaces[other],
                &cap_box,
                &other_box,
                None,
                Some(limits[other]),
                context,
            )?;
            let mut junctions = JunctionRegistry::default();
            for curve in raw {
                let mut events = Vec::new();
                if clip_closed_ellipse_with_events(
                    topo,
                    cap,
                    ids[other],
                    cap_surface,
                    surfaces[other],
                    &curve,
                    context.tolerance,
                    &mut junctions,
                    &mut events,
                )?
                .is_none()
                {
                    return Ok(None);
                }
                // Events are emitted once per radial root, before lens
                // midpoint splitting or junction-registry coordinate reuse.
                points.extend(events.into_iter().filter_map(|event| {
                    matches!(event.boundary, Boundary::DiscRim).then_some(event.point)
                }));
            }
        }
    }
    // Keep distinct roots distinct. The fitter must refuse if more than one
    // independently qualified root remains in its local endpoint window.
    Ok(Some(points))
}

/// Outward-rounded intervals. Inputs are the stored binary geometry, treated
/// as real coefficients; every algebraic operation encloses its roundoff.
#[derive(Clone, Copy, Debug)]
struct I {
    lo: f64,
    hi: f64,
}

impl I {
    const fn unknown() -> Self {
        Self {
            lo: f64::NEG_INFINITY,
            hi: f64::INFINITY,
        }
    }
    const fn point(x: f64) -> Self {
        Self { lo: x, hi: x }
    }
    fn add(self, b: Self) -> Self {
        if !self.finite() || !b.finite() {
            return Self::unknown();
        }
        let result = Self {
            lo: (self.lo + b.lo).next_down(),
            hi: (self.hi + b.hi).next_up(),
        };
        if result.finite() {
            result
        } else {
            Self::unknown()
        }
    }
    fn neg(self) -> Self {
        if !self.finite() {
            return Self::unknown();
        }
        Self {
            lo: -self.hi,
            hi: -self.lo,
        }
    }
    fn sub(self, b: Self) -> Self {
        self.add(b.neg())
    }
    fn abs(self) -> Self {
        if !self.finite() {
            return Self::unknown();
        }
        if self.lo >= 0.0 {
            self
        } else if self.hi <= 0.0 {
            self.neg()
        } else {
            Self {
                lo: 0.0,
                hi: self.lo.abs().max(self.hi.abs()),
            }
        }
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
        let result = Self {
            lo: products
                .iter()
                .copied()
                .fold(f64::INFINITY, f64::min)
                .next_down(),
            hi: products
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, f64::max)
                .next_up(),
        };
        if result.finite() {
            result
        } else {
            Self::unknown()
        }
    }
    fn scale(self, x: f64) -> Self {
        self.mul(Self::point(x))
    }
    fn div_positive(self, b: Self) -> Self {
        if !b.finite() || b.lo <= 0.0 {
            return Self::unknown();
        }
        self.mul(Self {
            lo: (1.0 / b.hi).next_down(),
            hi: (1.0 / b.lo).next_up(),
        })
    }
    fn sign(self) -> Option<bool> {
        if self.lo > 0.0 {
            Some(true)
        } else if self.hi < 0.0 {
            Some(false)
        } else {
            None
        }
    }
    fn finite(self) -> bool {
        self.lo.is_finite() && self.hi.is_finite() && self.lo <= self.hi
    }
}

type V = [I; 3];
type Poly = [I; 5];

fn vector(v: Vec3) -> V {
    v.0.map(I::point)
}
fn difference(a: Point3, b: Point3) -> V {
    std::array::from_fn(|i| I::point(a.0[i]).sub(I::point(b.0[i])))
}
fn vadd(a: V, b: V) -> V {
    std::array::from_fn(|i| a[i].add(b[i]))
}
fn vsub(a: V, b: V) -> V {
    std::array::from_fn(|i| a[i].sub(b[i]))
}
fn vscale(a: V, x: I) -> V {
    a.map(|v| v.mul(x))
}
fn dot(a: V, b: V) -> I {
    a[0].mul(b[0]).add(a[1].mul(b[1])).add(a[2].mul(b[2]))
}

fn absolute_coefficient_sum(coefficients: impl IntoIterator<Item = I>) -> Option<I> {
    let mut total = I::point(0.0);
    for coefficient in coefficients {
        if !coefficient.finite() {
            return None;
        }
        total = total.add(coefficient.abs());
    }
    total.finite().then_some(total)
}

fn ellipse_plane_residual(ellipse: &Ellipse3D, normal: Vec3, d: f64) -> Option<I> {
    let normal = vector(normal);
    absolute_coefficient_sum([
        dot(normal, vector(position_vector(ellipse.center()))).sub(I::point(d)),
        dot(normal, vector(ellipse.u_axis())).scale(ellipse.semi_major()),
        dot(normal, vector(ellipse.v_axis())).scale(ellipse.semi_minor()),
    ])
}

/// A whole-period bound on the cylinder radial equation. Each Fourier
/// coefficient and the sum of their absolute values enclose rounding from
/// the stored geometry, including cancellation in the constant coefficient.
fn ellipse_cylinder_residual(ellipse: &Ellipse3D, cyl: &CylindricalSurface) -> Option<I> {
    let axis = vector(cyl.axis());
    let radial = |v: V| vsub(v, vscale(axis, dot(v, axis)));
    let dd = radial(difference(ellipse.center(), cyl.origin()));
    let aa = radial(vscale(
        vector(ellipse.u_axis()),
        I::point(ellipse.semi_major()),
    ));
    let bb = radial(vscale(
        vector(ellipse.v_axis()),
        I::point(ellipse.semi_minor()),
    ));
    let a2 = dot(aa, aa);
    let b2 = dot(bb, bb);
    let r2 = I::point(cyl.radius()).mul(I::point(cyl.radius()));
    absolute_coefficient_sum([
        dot(dd, dd).add(a2.add(b2).scale(0.5)).sub(r2),
        dot(dd, aa).scale(2.0),
        dot(dd, bb).scale(2.0),
        a2.sub(b2).scale(0.5),
        dot(aa, bb),
    ])
}

fn evaluate(p: &Poly, x: I) -> I {
    p.iter().rev().fold(I::point(0.0), |y, c| y.mul(x).add(*c))
}
fn derivative(p: &Poly) -> Poly {
    [
        p[1],
        p[2].scale(2.0),
        p[3].scale(3.0),
        p[4].scale(4.0),
        I::point(0.0),
    ]
}
fn linear_event(d: I, a: I, b: I) -> Poly {
    [
        d.add(a),
        b.scale(2.0),
        d.sub(a),
        I::point(0.0),
        I::point(0.0),
    ]
}
fn radial_event(d: V, a: V, b: V, radius: f64) -> Poly {
    let r2 = I::point(radius).mul(I::point(radius));
    [
        dot(vadd(d, a), vadd(d, a)).sub(r2),
        dot(vadd(d, a), b).scale(4.0),
        dot(d, d)
            .sub(r2)
            .scale(2.0)
            .sub(dot(a, a).scale(2.0))
            .add(dot(b, b).scale(4.0)),
        dot(vsub(d, a), b).scale(4.0),
        dot(vsub(d, a), vsub(d, a)).sub(r2),
    ]
}

/// Isolate every simple root on a bounded chart. An ambiguous endpoint,
/// unresolved multiple root, or exhausted work bound is an explicit refusal.
fn roots(p: &Poly, target: f64) -> Result<Vec<I>, AlgoError> {
    let dp = derivative(p);
    let mut stack = vec![(-1.0, 1.0, 0_usize)];
    let mut result = Vec::new();
    let mut nodes = 0;
    while let Some((mut lo, mut hi, depth)) = stack.pop() {
        nodes += 1;
        if nodes > MAX_ROOT_NODES || depth > MAX_ROOT_DEPTH {
            return Err(unresolved("root isolation work limit or multiple contact"));
        }
        if evaluate(p, I { lo, hi }).sign().is_some() {
            continue;
        }
        if evaluate(&dp, I { lo, hi }).sign().is_some() {
            let s0 = evaluate(p, I::point(lo))
                .sign()
                .ok_or_else(|| unresolved("root endpoint"))?;
            let s1 = evaluate(p, I::point(hi))
                .sign()
                .ok_or_else(|| unresolved("root endpoint"))?;
            if s0 == s1 {
                continue;
            }
            for _ in 0..80 {
                if hi - lo <= target {
                    break;
                }
                let mid = f64::midpoint(lo, hi);
                let value = evaluate(p, I::point(mid));
                if let Some(sign) = value.sign() {
                    if sign == s0 {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                } else {
                    let slope = evaluate(&dp, I { lo, hi });
                    let lower = slope.lo.abs().min(slope.hi.abs());
                    let reach = (value.lo.abs().max(value.hi.abs()) / lower).next_up();
                    let next_lo = lo.max((mid - reach).next_down());
                    let next_hi = hi.min((mid + reach).next_up());
                    if next_hi - next_lo >= (hi - lo) * 0.99 {
                        break;
                    }
                    lo = next_lo;
                    hi = next_hi;
                }
            }
            // Opposite endpoint signs and a nonzero whole-interval
            // derivative already prove exactly one root here. Coefficient
            // uncertainty can prevent the requested parameter contraction;
            // the caller must judge the retained bracket in physical space.
            result.push(I { lo, hi });
            continue;
        }
        // Keep subdivision endpoints away from roots, so both children have
        // certified endpoint signs. These probes choose a split; they never
        // decide geometric exclusion or root count.
        let split = [0.5, 0.375, 0.625, 0.25, 0.75]
            .into_iter()
            .find_map(|fraction| {
                let x = lo + (hi - lo) * fraction;
                evaluate(p, I::point(x)).sign().map(|_| x)
            })
            .ok_or_else(|| unresolved("unresolved multiple contact"))?;
        stack.push((split, hi, depth + 1));
        stack.push((lo, split, depth + 1));
    }
    Ok(result)
}

struct Chart {
    phase: f64,
    a: V,
    b: V,
    events: [Poly; 3],
}

/// Bound the distance from the emitted native-ellipse point to every point
/// in an isolated root's exact rational chart image. This includes chart
/// rotation coefficient uncertainty and native parameter conversion error.
fn root_image_radius(chart: &Chart, center: Point3, root: I, emitted: Point3) -> Option<I> {
    let squared = root.mul(root);
    let denominator = I::point(1.0).add(squared);
    let cosine = I::point(1.0).sub(squared).div_positive(denominator);
    let sine = root.scale(2.0).div_positive(denominator);
    let relative = vadd(vscale(chart.a, cosine), vscale(chart.b, sine));
    absolute_coefficient_sum(vadd(difference(center, emitted), relative))
}

fn check_root_image(
    chart: &Chart,
    ellipse: &Ellipse3D,
    root: I,
    parameter: f64,
    tolerance: f64,
) -> Result<(), AlgoError> {
    let allocation = I::point(tolerance).scale(0.125);
    if !allocation.finite()
        || root_image_radius(chart, ellipse.center(), root, ellipse.evaluate(parameter))
            .is_none_or(|radius| radius.hi > allocation.lo)
    {
        return Err(unresolved(
            "root physical-image uncertainty exceeds tolerance",
        ));
    }
    Ok(())
}

#[derive(Clone, Copy)]
enum Boundary {
    Chart,
    DiscRim,
    AxialRim,
}

struct Event {
    range: I,
    boundary: Boundary,
}

struct Window {
    range: (f64, f64),
    start: Boundary,
    end: Boundary,
}

struct NativeEvent {
    point: Point3,
    boundary: Boundary,
}

/// An isolated native event independently certified on the current supports
/// and an authoritative source rim. Its source pair identifies an EF pave.
pub(super) struct CapAnchor {
    pub(super) point: Point3,
    pub(super) source_edge: EdgeId,
    pub(super) opposing_face: FaceId,
    pub(super) parameter: f64,
}

type AnchoredArcs = (Vec<RawCurve>, Vec<CapAnchor>);

#[allow(clippy::too_many_arguments)]
pub(super) fn clip_closed_ellipse_with_anchors(
    topo: &Topology,
    fa: FaceId,
    fb: FaceId,
    surf_a: &FaceSurface,
    surf_b: &FaceSurface,
    raw: &RawCurve,
    tol: Tolerance,
    junctions: &mut JunctionRegistry,
) -> Result<Option<AnchoredArcs>, AlgoError> {
    let mut events = Vec::new();
    let Some(arcs) = clip_closed_ellipse_with_events(
        topo,
        fa,
        fb,
        surf_a,
        surf_b,
        raw,
        tol,
        junctions,
        &mut events,
    )?
    else {
        return Ok(None);
    };
    let (plane_id, cyl_id, cylinder, normal, d) =
        if let (FaceSurface::Plane { normal, d }, FaceSurface::Cylinder(cylinder)) =
            (surf_a, surf_b)
        {
            (fa, fb, cylinder, *normal, *d)
        } else if let (FaceSurface::Cylinder(cylinder), FaceSurface::Plane { normal, d }) =
            (surf_a, surf_b)
        {
            (fb, fa, cylinder, *normal, *d)
        } else {
            return Err(unresolved("event support identity"));
        };
    let mut anchors = Vec::new();
    for event in events {
        let (owner, opposing_face) = if matches!(event.boundary, Boundary::DiscRim) {
            (plane_id, cyl_id)
        } else {
            (cyl_id, plane_id)
        };
        let plane_error = dot(vector(normal), vector(position_vector(event.point)))
            .sub(I::point(d))
            .abs();
        if !plane_error.finite()
            || plane_error.hi > tol.linear * 0.0625
            || bezier_cylinder_residual_bound(
                &[event.point, event.point],
                &[1.0; 2],
                cylinder,
                cylinder.axis(),
            )
            .is_none_or(|(bound, _)| bound > tol.linear * 0.125)
        {
            return Err(unresolved("native boundary event support certificate"));
        }
        let wire = topo.wire(topo.face(owner)?.outer_wire())?;
        let mut anchor = None;
        for usage in wire.edges() {
            let edge = topo.edge(usage.edge())?;
            let Some(circle) = full_circle(edge) else {
                continue;
            };
            let (start, end) = edge
                .strict_domain()
                .map_err(|_| unresolved("source rim range"))?;
            let raw_parameter = circle.project(event.point);
            let parameter = if end > start {
                start + (raw_parameter - start).rem_euclid(TAU)
            } else {
                start - (start - raw_parameter).rem_euclid(TAU)
            };
            // Projection only chooses a stored-circle witness; the outward
            // coordinate bound proves that witness is within the allocation.
            let error =
                absolute_coefficient_sum(difference(event.point, circle.evaluate(parameter)));
            if error.is_none_or(|bound| bound.hi > tol.linear * 0.125) {
                continue;
            }
            if anchor.is_some() {
                return Err(unresolved("native event has multiple source rims"));
            }
            anchor = Some(CapAnchor {
                point: event.point,
                source_edge: usage.edge(),
                opposing_face,
                parameter,
            });
        }
        anchors
            .push(anchor.ok_or_else(|| unresolved("native event lacks a certified source rim"))?);
    }
    Ok(Some((arcs, anchors)))
}

fn charts(
    ellipse: &Ellipse3D,
    disc: &Circle3D,
    cyl: &CylindricalSurface,
    limits: (f64, f64),
) -> Result<[Chart; 2], AlgoError> {
    let a = vscale(vector(ellipse.u_axis()), I::point(ellipse.semi_major()));
    let b = vscale(vector(ellipse.v_axis()), I::point(ellipse.semi_minor()));
    let d = difference(ellipse.center(), disc.center());
    let axis = vector(cyl.axis());
    let height = dot(difference(ellipse.center(), cyl.origin()), axis);
    for q in [0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1.0] {
        let q = I::point(q);
        let den = I::point(1.0).add(q.mul(q));
        let cos = I::point(1.0).sub(q.mul(q)).div_positive(den);
        let sin = q.scale(2.0).div_positive(den);
        let aa = vadd(vscale(a, cos), vscale(b, sin));
        let bb = vsub(vscale(b, cos), vscale(a, sin));
        let phase = 2.0 * q.lo.atan();
        let pair = std::array::from_fn(|index| {
            let sign = if index == 0 { 1.0 } else { -1.0 };
            let aa = vscale(aa, I::point(sign));
            let bb = vscale(bb, I::point(sign));
            Chart {
                phase: phase + if index == 0 { 0.0 } else { PI },
                a: aa,
                b: bb,
                events: [
                    radial_event(d, aa, bb, disc.radius()),
                    linear_event(height.sub(I::point(limits.0)), dot(aa, axis), dot(bb, axis)),
                    linear_event(height.sub(I::point(limits.1)), dot(aa, axis), dot(bb, axis)),
                ],
            }
        });
        if pair.iter().all(|chart| {
            chart.events.iter().all(|poly| {
                poly.iter().all(|c| c.finite())
                    && evaluate(poly, I::point(-1.0)).sign().is_some()
                    && evaluate(poly, I::point(1.0)).sign().is_some()
            })
        }) {
            return Ok(pair);
        }
    }
    Err(unresolved("all chart boundaries meet unresolved contacts"))
}

/// Clip only closed ellipse sections of an oblique full cylinder by a disc.
/// `None` means unsupported; `Some([])` is a certified empty finite section.
#[cfg(test)]
#[allow(clippy::too_many_arguments)]
pub(super) fn clip_closed_ellipse_to_circular_cap(
    topo: &Topology,
    fa: FaceId,
    fb: FaceId,
    surf_a: &FaceSurface,
    surf_b: &FaceSurface,
    raw: &RawCurve,
    tol: Tolerance,
    junctions: &mut JunctionRegistry,
) -> Result<Option<Vec<RawCurve>>, AlgoError> {
    clip_closed_ellipse_with_events(
        topo,
        fa,
        fb,
        surf_a,
        surf_b,
        raw,
        tol,
        junctions,
        &mut Vec::new(),
    )
}

#[allow(clippy::too_many_arguments)]
fn clip_closed_ellipse_with_events(
    topo: &Topology,
    fa: FaceId,
    fb: FaceId,
    surf_a: &FaceSurface,
    surf_b: &FaceSurface,
    raw: &RawCurve,
    tol: Tolerance,
    junctions: &mut JunctionRegistry,
    native_events: &mut Vec<NativeEvent>,
) -> Result<Option<Vec<RawCurve>>, AlgoError> {
    let EdgeCurve::Ellipse(ellipse) = &raw.curve else {
        return Ok(None);
    };
    if !orthonormal(ellipse.u_axis(), ellipse.v_axis(), ellipse.normal())
        || !ellipse.semi_major().is_finite()
        || !ellipse.semi_minor().is_finite()
        || ellipse.semi_minor() <= 0.0
        || ellipse.semi_major() <= ellipse.semi_minor()
        || ((raw.t_range.1 - raw.t_range.0).abs() - TAU).abs() > 32.0 * f64::EPSILON * TAU
        || (raw.p_start - raw.p_end).length() > tol.linear
    {
        return Ok(None);
    }
    let (plane_id, cyl_id, cyl, normal, d) =
        if let (FaceSurface::Plane { normal, d }, FaceSurface::Cylinder(cyl)) = (surf_a, surf_b) {
            (fa, fb, cyl, *normal, *d)
        } else if let (FaceSurface::Cylinder(cyl), FaceSurface::Plane { normal, d }) =
            (surf_a, surf_b)
        {
            (fb, fa, cyl, *normal, *d)
        } else {
            return Ok(None);
        };
    let Some(disc) = closed_disc(topo, plane_id, tol) else {
        return Ok(None);
    };
    let Some(limits) = cylinder_limits(topo, cyl_id, cyl, tol) else {
        return Ok(None);
    };
    let cos = normal.dot(cyl.axis()).abs();
    if cos <= 1e-10 || cos >= 1.0 - 1e-10 {
        return Ok(None);
    }
    if ellipse_plane_residual(ellipse, normal, d).is_none_or(|bound| bound.hi > tol.linear) {
        return Ok(None);
    }
    let limit = I::point(tol.linear).mul(I::point(cyl.radius()));
    if !limit.finite()
        || ellipse_cylinder_residual(ellipse, cyl).is_none_or(|bound| bound.hi > limit.lo)
    {
        return Ok(None);
    }
    let charts = charts(ellipse, &disc, cyl, limits)?;
    let target = (tol.linear / (128.0 * (ellipse.semi_major() + ellipse.semi_minor()))).min(1e-6);
    if !target.is_finite() || target <= 0.0 {
        return Err(unresolved("invalid root tolerance"));
    }
    let mut pieces: Vec<Window> = Vec::new();
    let mut boundary_events = 0;
    for chart in charts {
        let mut events = vec![
            Event {
                range: I::point(-1.0),
                boundary: Boundary::Chart,
            },
            Event {
                range: I::point(1.0),
                boundary: Boundary::Chart,
            },
        ];
        for (index, polynomial) in chart.events.iter().enumerate() {
            let boundary = if index == 0 {
                Boundary::DiscRim
            } else {
                Boundary::AxialRim
            };
            events.extend(
                roots(polynomial, target)?
                    .into_iter()
                    .map(|range| Event { range, boundary }),
            );
        }
        boundary_events += events.len() - 2;
        events.compact_sort_by(|a, b| a.range.lo.total_cmp(&b.range.lo));
        for pair in events.windows(2) {
            if pair[0].range.hi >= pair[1].range.lo {
                return Err(unresolved("coincident boundary events"));
            }
            let mid = I::point(f64::midpoint(pair[0].range.hi, pair[1].range.lo));
            let signs: Option<Vec<_>> = chart
                .events
                .iter()
                .map(|p| evaluate(p, mid).sign())
                .collect();
            let Some(signs) = signs else {
                return Err(unresolved("interval material sign"));
            };
            if signs.as_slice() != [false, true, false] {
                continue;
            }
            let start =
                chart.phase + 2.0 * f64::midpoint(pair[0].range.lo, pair[0].range.hi).atan();
            let end = chart.phase + 2.0 * f64::midpoint(pair[1].range.lo, pair[1].range.hi).atan();
            for (event, parameter) in [(&pair[0], start), (&pair[1], end)] {
                if !matches!(event.boundary, Boundary::Chart) {
                    check_root_image(&chart, ellipse, event.range, parameter, tol.linear)?;
                }
            }
            pieces.push(Window {
                range: (start, end),
                start: pair[0].boundary,
                end: pair[1].boundary,
            });
        }
    }
    if boundary_events == 0 && !pieces.is_empty() {
        return Ok(Some(vec![raw.clone()]));
    }
    // Chart boundaries are proven away from every geometric event. Remove
    // those artificial cuts, including the periodic join, before emission.
    let mut merged: Vec<Window> = Vec::new();
    for piece in pieces {
        if let Some(last) = merged.last_mut()
            && matches!(last.end, Boundary::Chart)
            && matches!(piece.start, Boundary::Chart)
        {
            if (last.range.1 - piece.range.0).abs() > 64.0 * f64::EPSILON * TAU {
                return Err(unresolved("chart boundary mismatch"));
            }
            last.range.1 = piece.range.1;
            last.end = piece.end;
        } else {
            merged.push(piece);
        }
    }
    if merged.len() > 1
        && matches!(merged[0].start, Boundary::Chart)
        && merged
            .last()
            .is_some_and(|last| matches!(last.end, Boundary::Chart))
    {
        let first = merged.remove(0);
        if let Some(last) = merged.last_mut() {
            last.range.1 = first.range.1 + TAU;
            last.end = first.end;
        }
    }
    let mut output = Vec::new();
    for window in merged {
        if matches!(window.start, Boundary::DiscRim) {
            native_events.push(NativeEvent {
                point: ellipse.evaluate(window.range.0),
                boundary: window.start,
            });
        } else if matches!(window.start, Boundary::AxialRim) {
            native_events.push(NativeEvent {
                point: ellipse.evaluate(window.range.0),
                boundary: window.start,
            });
        }
        if matches!(window.end, Boundary::DiscRim) {
            native_events.push(NativeEvent {
                point: ellipse.evaluate(window.range.1),
                boundary: window.end,
            });
        } else if matches!(window.end, Boundary::AxialRim) {
            native_events.push(NativeEvent {
                point: ellipse.evaluate(window.range.1),
                boundary: window.end,
            });
        }
        // Two disc-rim crossings bound a real ellipse/circle lens. Give its
        // ellipse side an exact interior vertex so assembly cannot identify
        // the two different curves solely by their common endpoint pair.
        let rim_to_rim =
            matches!(window.start, Boundary::DiscRim) && matches!(window.end, Boundary::DiscRim);
        let interval_start = window.range.0;
        let interval_span = window.range.1 - window.range.0;
        let divisions = if rim_to_rim || interval_span >= PI * 0.999 {
            2
        } else {
            1
        };
        for index in 0..divisions {
            let fraction = f64::from(index) / f64::from(divisions);
            let next = f64::from(index + 1) / f64::from(divisions);
            let start = interval_start + interval_span * fraction;
            let end = interval_start + interval_span * next;
            let p_start = if index == 0 || !rim_to_rim {
                junctions.resolve(topo, fa, fb, ellipse.evaluate(start), tol)
            } else {
                ellipse.evaluate(start)
            };
            let p_end = if index + 1 == divisions || !rim_to_rim {
                junctions.resolve(topo, fa, fb, ellipse.evaluate(end), tol)
            } else {
                ellipse.evaluate(end)
            };
            output.push(RawCurve {
                curve: raw.curve.clone(),
                bbox: raw.bbox,
                t_range: (start, end),
                p_start,
                p_end,
            });
        }
    }
    Ok(Some(output))
}

#[cfg(test)]
mod tests;
