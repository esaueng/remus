//! Exact boundary reduction of analytic curved-face areas (Green's theorem).
//!
//! On a cylinder, cone, sphere or torus the area element depends on the `v`
//! parameter alone, `dA = J(v) du dv`. With `G' = J`, Green's theorem turns
//! the area of a trimmed `(u, v)` region `D` into a line integral over its
//! boundary:
//!
//! ```text
//! A = ∬_D J(v) du dv = −∮_∂D G(v) du      (∂D counter-clockwise in (u, v))
//! ```
//!
//! | Surface  | `J(v)`             | `G(v)` in local coordinates `(x, y, z)` |
//! |----------|--------------------|-----------------------------------------|
//! | cylinder | `r`                | `r z`                                   |
//! | cone     | `v cos a`          | `z² cos a / (2 sin² a)` (`z = v sin a`) |
//! | sphere   | `r² cos v`         | `r z` (`z = r sin v`)                   |
//! | torus    | `r (R + r cos v)`  | `r (R v + z)`                           |
//!
//! `u = atan2(y, x)` around the axis, so along a boundary curve `C(t)`,
//! `du = (x y' − y x') / (x² + y²) dt` comes from the curve's own derivative:
//! no projection, unwrapping or seam bookkeeping is needed for `u`. Each edge
//! is integrated over its authoritative parameter span by adaptive
//! Gauss–Legendre quadrature (split on NURBS knots and into arcs of at most
//! a quarter turn), which is exact for iso-parametric lines and circles (a
//! constant or vanishing integrand) and converges to rounding elsewhere.
//!
//! The material side comes from the wire orientation. Every surface here is
//! parameterized so that `∂P/∂u × ∂P/∂v` is its own outward normal, and a
//! face's loops run counter-clockwise in `(u, v)` about that normal whatever
//! the face's `reversed` flag, which flips only the face's outward normal
//! (a bore keeps its cylinder's loops). The trimmed quadrature relies on the
//! same convention to pick a sphere cap's pole. A region containing a pole (a
//! sphere cap, a pointed cone) has a boundary whose net `u` winding is a full
//! turn; the pole closes it at `G(pole)`.
//!
//! Every assumption is checked rather than trusted, and any doubt answers
//! `None` so the caller keeps the quadrature path: a boundary through a pole
//! or the cone apex, a torus that is not a ring torus or a wire that winds
//! around its tube, a curve with no analytic derivative, a wire whose winding
//! is not a whole number of turns, a closed loop wound against its role, a
//! non-positive total, or a quadrature that does not converge within its
//! work budget.

use remus_math::frame::Frame3;
use remus_math::quadrature::gauss_legendre_points;
use remus_math::surfaces::SphericalSurface;
use remus_math::traits::ParametricCurve;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::wire::WireId;

use crate::CheckError;

const TAU: f64 = std::f64::consts::TAU;

/// Gauss–Legendre order of one quadrature panel.
const PANEL_ORDER: usize = 10;

/// Longest parameter sweep of one circle or ellipse panel before refinement.
const MAX_ARC_SWEEP: f64 = std::f64::consts::FRAC_PI_4;

/// Deepest bisection of one panel.
const MAX_DEPTH: u32 = 24;

/// Panels one face may integrate before the reduction gives up.
const PANEL_BUDGET: usize = 1 << 14;

/// Accepted panel error, relative to the face's area scale (`L²`).
const RELATIVE_PANEL_TOLERANCE: f64 = 1e-14;

/// A sphere's polar axis must stay this far (relative to the radius) from
/// every boundary sample to be used.
const SPHERE_AXIS_CLEARANCE: f64 = 1e-3;

/// Accepted panel error of the `u` winding, in radians.
const WINDING_PANEL_TOLERANCE: f64 = 1e-13;

/// Largest gap between consecutive edge curves, relative to the face scale,
/// that [`bridge`] closes with a chord.
const MAX_BRIDGED_GAP: f64 = 1e-4;

/// How far a panel's `∫ du` may sit from its endpoints' change of `u`, modulo
/// whole turns, in radians (see [`continuous_in_u`]).
const PANEL_CONTINUITY_SLACK: f64 = 1e-7;

/// How far a wire's winding may sit from a whole number of turns.
const WINDING_SLACK: f64 = 1e-8;

/// A boundary point closer than this fraction of the surface's own scale to
/// the axis (a sphere pole, the cone apex) refuses the reduction: `u` jumps
/// there, and the jump carries area wherever `G` is non-zero.
const AXIS_CLEARANCE: f64 = 1e-6;

/// Integrand of one analytic surface in its local frame.
#[derive(Debug, Clone, Copy)]
enum Kernel {
    /// `G = r z`.
    Cylinder { r: f64 },
    /// `G = k z²`, `k = cos a / (2 sin² a)`; `z ≥ 0` on the cone's nappe.
    Cone { k: f64 },
    /// `G = r z`; the poles sit at `z = ±r`. `quadrature_axis` records
    /// whether `z` is the polar axis the trimmed quadrature itself uses.
    Sphere { r: f64, quadrature_axis: bool },
    /// `G = r (R v + z)`, `v = atan2(z, ρ − R)` continued along the wire.
    Torus { major: f64, minor: f64 },
}

#[derive(Debug, Clone, Copy)]
struct Chart {
    origin: Point3,
    x: Vec3,
    y: Vec3,
    z: Vec3,
    kernel: Kernel,
    /// Characteristic length: area errors are judged against its square.
    scale: f64,
}

impl Chart {
    fn local(&self, p: Point3) -> (f64, f64, f64) {
        let q = p - self.origin;
        (q.dot(self.x), q.dot(self.y), q.dot(self.z))
    }

    /// Distance from the axis below which `u` is ill-defined.
    fn axis_clearance(&self) -> f64 {
        match self.kernel {
            Kernel::Sphere { r, .. } => AXIS_CLEARANCE * r,
            Kernel::Cone { .. } => AXIS_CLEARANCE * self.scale,
            // A cylinder's and a ring torus's points all keep their radius
            // from the axis; nothing on them reaches it.
            Kernel::Cylinder { .. } | Kernel::Torus { .. } => 0.0,
        }
    }
}

/// Running state of one wire's integration.
struct WireSum {
    /// `∮ G du` along the wire as stored.
    area_term: f64,
    /// `∮ du` along the wire as stored.
    winding: f64,
    /// The torus's tube angle at the last evaluated point, continued along
    /// the wire so `R v` stays on one branch.
    tube_angle: Option<f64>,
    panels: usize,
}

/// Area of an analytic curved face from its boundary alone, or `None` when
/// the face lies outside the qualified family (see the [module docs](self)).
///
/// The value is the unsigned area of the trimmed region.
///
/// # Errors
///
/// Returns an error only when a topology lookup fails.
pub(super) fn analytic_face_area(
    topo: &Topology,
    face_id: FaceId,
) -> Result<Option<f64>, CheckError> {
    let face = topo.face(face_id)?;
    let mut wire_ids = Vec::with_capacity(1 + face.inner_wires().len());
    wire_ids.push(face.outer_wire());
    wire_ids.extend(face.inner_wires().iter().copied());

    let mut vertices = Vec::new();
    for &wire_id in &wire_ids {
        for oriented in topo.wire(wire_id)?.edges() {
            let edge = topo.edge(oriented.edge())?;
            vertices.push(topo.vertex(edge.start())?.point());
            vertices.push(topo.vertex(edge.end())?.point());
        }
    }
    let Some(&anchor) = vertices.first() else {
        return Ok(None);
    };

    let Some(chart) = chart_for(topo, face.surface(), &wire_ids, anchor, &vertices)? else {
        return Ok(None);
    };

    let tolerance = RELATIVE_PANEL_TOLERANCE * chart.scale * chart.scale;
    // `(∮ G du, turns)` per wire, outer first.
    let mut loops = Vec::with_capacity(wire_ids.len());
    let mut panels = 0;
    for &wire_id in &wire_ids {
        let mut sum = WireSum {
            area_term: 0.0,
            winding: 0.0,
            tube_angle: None,
            panels,
        };
        if !integrate_wire(topo, &chart, wire_id, tolerance, &mut sum)? {
            return Ok(None);
        }
        panels = sum.panels;
        let turns = (sum.winding / TAU).round();
        if (sum.winding - turns * TAU).abs() > WINDING_SLACK || turns.abs() > 1.0 {
            return Ok(None);
        }
        loops.push((sum.area_term, turns));
    }
    let area = combine(&chart, &loops);
    Ok(area.filter(|area| area.is_finite() && *area > 0.0))
}

/// The face's area from its loops' `(∮ G du, turns)`, outer loop first.
///
/// Loop orientation is source-dependent (kernel builders wind loops about
/// the surface's own normal whatever the face's `reversed` flag; imported
/// faces need not), so the region is read off orientation-free wherever the
/// trimmed quadrature does the same:
///
/// * a closed loop bounds the patch `|∮ G du|` inside it — the finite side
///   on a cylinder or cone, the side away from the polar axis on a sphere,
///   as the quadrature's UV polygon does; inner loops are holes wound against
///   the outer one;
/// * two loops that each wind once around the axis bound the band between
///   them, `|∮₁ G du − ∮₂ G du|` with both taken in `+u`;
/// * one winding loop on a cone bounds the patch towards the apex (`G = 0`),
///   `|∮ G du|`.
///
/// The one place the quadrature reads orientation is a sphere cap: a loop
/// winding `+u` keeps the north pole, as it does in the quadrature's chart.
/// In the fallback clearance chart, used when the quadrature's own axis
/// touches the boundary, only a region within a hemisphere qualifies, read
/// as the side the loop encloses (the quadrature integrates over its
/// boundary's own UV extent).
fn combine(chart: &Chart, loops: &[(f64, f64)]) -> Option<f64> {
    let (&(outer_term, outer_turns), inner) = loops.split_first()?;
    let mut holes = 0.0;
    // `(∮ G du in +u, turns)` of each loop that winds around the axis.
    let mut winding_terms = Vec::new();
    if outer_turns != 0.0 {
        winding_terms.push((outer_term * outer_turns, outer_turns));
    }
    for &(term, turns) in inner {
        if turns == 0.0 {
            // Holes run against a closed outer loop under either convention.
            if outer_turns == 0.0 && term * outer_term >= 0.0 {
                return None;
            }
            holes += term.abs();
        } else {
            winding_terms.push((term * turns, turns));
        }
    }
    let region = match (winding_terms.as_slice(), chart.kernel) {
        (
            [],
            Kernel::Sphere {
                r,
                quadrature_axis: false,
            },
        ) => {
            // Only the quadrature's own chart says which side of a loop on a
            // sphere it measures; this one can vouch only for a patch the
            // quadrature reads the same way from any chart.
            let inside = outer_term.abs();
            if inside > TAU * r * r {
                return None;
            }
            inside
        }
        ([], _) => outer_term.abs(),
        (
            &[(cap, _)],
            Kernel::Sphere {
                r,
                quadrature_axis: false,
            },
        ) => {
            // The quadrature integrates over its boundary's own UV extent, so
            // it measures the side the loop encloses: of the two caps, the
            // one within a hemisphere.
            let smaller = TAU.mul_add(r * r, -cap.abs());
            if smaller > TAU * r * r {
                return None;
            }
            smaller
        }
        (
            &[(cap, turns)],
            Kernel::Sphere {
                r,
                quadrature_axis: true,
            },
        ) => {
            // A loop that runs in `+u` keeps the north pole (`G = r²`) on its
            // left about the outward normal, one that runs in `−u` the south
            // pole (`G = −r²`).
            if turns > 0.0 {
                TAU.mul_add(r * r, -cap)
            } else {
                TAU.mul_add(r * r, cap)
            }
        }
        (&[(cap, _)], Kernel::Cone { .. }) => cap.abs(),
        (&[(first, _), (second, _)], _) => (first - second).abs(),
        _ => return None,
    };
    Some(region - holes)
}

/// The local frame and integrand of `surface`, or `None` for a surface or
/// boundary the reduction does not qualify.
fn chart_for(
    topo: &Topology,
    surface: &FaceSurface,
    wire_ids: &[WireId],
    anchor: Point3,
    vertices: &[Point3],
) -> Result<Option<Chart>, CheckError> {
    let reach = |from: Point3| {
        vertices
            .iter()
            .map(|p| (*p - from).length())
            .fold(0.0_f64, f64::max)
    };
    let chart = match surface {
        FaceSurface::Cylinder(s) => {
            // `G = r z` is shift-invariant on a cylinder (its regions have no
            // net winding), so measure `z` from the face, not from a distant
            // origin, and keep the cancelling terms small.
            let origin = s.origin() + s.axis() * (anchor - s.origin()).dot(s.axis());
            Chart {
                origin,
                x: s.x_axis(),
                y: s.y_axis(),
                z: s.axis(),
                kernel: Kernel::Cylinder { r: s.radius() },
                scale: s.radius().max(reach(anchor)),
            }
        }
        FaceSurface::Cone(s) => {
            let (sin_a, cos_a) = s.half_angle().sin_cos();
            if sin_a <= 1e-9 {
                return Ok(None);
            }
            Chart {
                origin: s.apex(),
                x: s.x_axis(),
                y: s.y_axis(),
                z: s.axis(),
                kernel: Kernel::Cone {
                    k: cos_a / (2.0 * sin_a * sin_a),
                },
                scale: reach(s.apex()),
            }
        }
        FaceSurface::Sphere(s) => {
            // Any polar axis parameterizes a sphere, but only the boundary
            // must avoid its poles: the quadrature's own axis when the
            // boundary keeps clear of it, else the clearest candidate.
            let Some((z, quadrature_axis)) = sphere_polar_axis(topo, s, wire_ids)? else {
                return Ok(None);
            };
            let Ok(frame) = Frame3::from_normal(s.center(), z) else {
                return Ok(None);
            };
            Chart {
                origin: s.center(),
                x: frame.x,
                y: frame.y,
                z: frame.z,
                kernel: Kernel::Sphere {
                    r: s.radius(),
                    quadrature_axis,
                },
                scale: s.radius(),
            }
        }
        FaceSurface::Torus(s) => {
            let (major, minor) = (s.major_radius(), s.minor_radius());
            // Only a ring torus keeps `R + r cos v > 0`, so `J > 0`.
            if !(minor > 0.0 && major > minor * (1.0 + 1e-9)) {
                return Ok(None);
            }
            Chart {
                origin: s.center(),
                x: s.x_axis(),
                y: s.y_axis(),
                z: s.z_axis(),
                kernel: Kernel::Torus { major, minor },
                scale: major + minor,
            }
        }
        FaceSurface::Plane { .. } | FaceSurface::Nurbs(_) => return Ok(None),
    };
    // The chart must be right-handed for `∂P/∂u × ∂P/∂v` to be outward.
    if chart.x.cross(chart.y).dot(chart.z) <= 0.0
        || chart.scale.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater)
    {
        return Ok(None);
    }
    Ok(Some(chart))
}

/// The polar axis to integrate a sphere face about, and whether it is the
/// one the trimmed quadrature uses (see [`quadrature_sphere_axis`]).
///
/// That axis is kept when every boundary sample stays clear of its poles;
/// otherwise the candidate the samples keep farthest from is used.
fn sphere_polar_axis(
    topo: &Topology,
    sphere: &SphericalSurface,
    wire_ids: &[WireId],
) -> Result<Option<(Vec3, bool)>, CheckError> {
    let (center, radius) = (sphere.center(), sphere.radius());
    let mut samples = Vec::new();
    for &wire_id in wire_ids {
        let edges = topo.wire(wire_id)?.edges();
        // A doubled seam makes the face wrap the seam's own axis, which the
        // quadrature's chart rule does not follow (it frames such a face by
        // the seam's great circle): keep the quadrature's reading there.
        if edges
            .iter()
            .enumerate()
            .any(|(i, a)| edges[i + 1..].iter().any(|b| b.edge() == a.edge()))
        {
            return Ok(None);
        }
        for oriented in edges {
            let edge = topo.edge(oriented.edge())?;
            let start = topo.vertex(edge.start())?.point();
            let end = topo.vertex(edge.end())?.point();
            let Ok((t0, t1)) = edge.strict_domain() else {
                return Ok(None);
            };
            for k in 0..=8 {
                let t = t0 + (t1 - t0) * f64::from(k) / 8.0;
                samples.push(edge.curve().evaluate_with_endpoints(t, start, end) - center);
            }
        }
    }
    let clearance = |axis: Vec3| {
        samples
            .iter()
            .map(|q| (*q - axis * q.dot(axis)).length())
            .fold(f64::INFINITY, f64::min)
    };
    let Some(&outer) = wire_ids.first() else {
        return Ok(None);
    };
    let quadrature = quadrature_sphere_axis(topo, sphere, outer)?;
    if clearance(quadrature) > SPHERE_AXIS_CLEARANCE * radius {
        return Ok(Some((quadrature, true)));
    }
    let s = 1.0 / 3.0_f64.sqrt();
    let candidates = [
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(s, s, s),
        Vec3::new(-s, s, s),
        Vec3::new(s, -s, s),
        Vec3::new(s, s, -s),
    ];
    let mut best: Option<(f64, Vec3)> = None;
    for axis in candidates {
        let clear = clearance(axis);
        if best.is_none_or(|(c, _)| clear > c) {
            best = Some((clear, axis));
        }
    }
    Ok(best.and_then(|(clear, axis)| {
        (clear > SPHERE_AXIS_CLEARANCE * radius).then_some((axis, false))
    }))
}

/// The polar axis `face_integrator::integrate_face_impl` measures a sphere
/// face about, by the same rule: the axis of an outer-wire great circle,
/// else the normal of an all-line outer wire through the centre, else the
/// surface's own polar axis.
fn quadrature_sphere_axis(
    topo: &Topology,
    sphere: &SphericalSurface,
    outer: WireId,
) -> Result<Vec3, CheckError> {
    let r = sphere.radius();
    let wire = topo.wire(outer)?;
    for oriented in wire.edges() {
        let (center, axis, radius) = match topo.edge(oriented.edge())?.curve() {
            EdgeCurve::Circle(c) => (c.center(), c.normal(), c.radius()),
            EdgeCurve::Ellipse(c) if (c.semi_major() - c.semi_minor()).abs() <= r * 1e-10 => {
                (c.center(), c.normal(), c.semi_major())
            }
            EdgeCurve::Ellipse(_)
            | EdgeCurve::Line
            | EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => continue,
        };
        if (center - sphere.center()).length() <= r * 1e-10
            && (radius - r).abs() <= r * 1e-10
            && let Ok(axis) = axis.normalize()
        {
            return Ok(axis);
        }
    }
    let mut points = Vec::new();
    let mut all_lines = true;
    for oriented in wire.edges() {
        let edge = topo.edge(oriented.edge())?;
        all_lines &= matches!(edge.curve(), EdgeCurve::Line);
        points.push(topo.vertex(oriented.oriented_start(edge))?.point() - sphere.center());
    }
    if all_lines {
        let mut axis = Vec3::new(0.0, 0.0, 0.0);
        for i in 0..points.len() {
            axis += points[i].cross(points[(i + 1) % points.len()]);
        }
        if let Ok(axis) = axis.normalize()
            && points.iter().all(|p| p.dot(axis).abs() < r * 1e-9)
        {
            return Ok(axis);
        }
    }
    Ok(sphere.z_axis())
}

/// Integrates one wire, edge by edge in wire order. `false` refuses.
fn integrate_wire(
    topo: &Topology,
    chart: &Chart,
    wire_id: WireId,
    tolerance: f64,
    sum: &mut WireSum,
) -> Result<bool, CheckError> {
    let wire = topo.wire(wire_id)?;
    let mut first: Option<Point3> = None;
    let mut last: Option<Point3> = None;
    let mut tube_start = None;
    for oriented in wire.edges() {
        let edge = topo.edge(oriented.edge())?;
        let start = topo.vertex(edge.start())?.point();
        let end = topo.vertex(edge.end())?.point();
        // The stored trim is the edge's authority (RFC 0002); an edge without
        // one keeps the quadrature.
        let Ok((d0, d1)) = edge.strict_domain() else {
            return Ok(false);
        };
        let (from, to) = if oriented.is_forward() {
            (d0, d1)
        } else {
            (d1, d0)
        };
        let curve = edge.curve();
        if !supported_curve(curve) {
            return Ok(false);
        }
        let (Some((head, _)), Some((tail, _))) = (
            point_and_derivative(curve, from, start, end),
            point_and_derivative(curve, to, start, end),
        ) else {
            return Ok(false);
        };
        if let Some(previous) = last {
            if !bridge(chart, previous, head, tolerance, sum) {
                return Ok(false);
            }
        } else {
            first = Some(head);
            // The torus tube angle starts on the principal branch at the
            // wire's first point and must come back to it: a wire that winds
            // around the tube bounds no patch in one chart.
            if let Kernel::Torus { major, .. } = chart.kernel {
                let (x, y, z) = chart.local(head);
                let v = z.atan2(x.hypot(y) - major);
                sum.tube_angle = Some(v);
                tube_start = Some(v);
            }
        }
        let breaks = panel_breaks(curve, from, to);
        for pair in breaks.windows(2) {
            if !integrate_panel(
                chart, curve, start, end, pair[0], pair[1], tolerance, 0, sum,
            ) {
                return Ok(false);
            }
        }
        last = Some(tail);
    }
    let (Some(first), Some(last)) = (first, last) else {
        return Ok(false);
    };
    if !bridge(chart, last, first, tolerance, sum) {
        return Ok(false);
    }
    if let (Some(start), Some(end)) = (tube_start, sum.tube_angle)
        && (continue_angle(end, start) - start).abs() > 1e-6
    {
        return Ok(false);
    }
    Ok(true)
}

/// Closes the gap a fitted edge curve leaves at a shared vertex with its
/// chord, as the trimmed quadrature's boundary polygon does. Only a
/// tolerance-sized gap is bridged; a larger one means the wire is not a
/// closed loop of its curves.
fn bridge(chart: &Chart, from: Point3, to: Point3, tolerance: f64, sum: &mut WireSum) -> bool {
    let gap = (to - from).length();
    if gap == 0.0 {
        return true;
    }
    if gap > MAX_BRIDGED_GAP * chart.scale {
        return false;
    }
    integrate_panel(
        chart,
        &EdgeCurve::Line,
        from,
        to,
        0.0,
        1.0,
        tolerance,
        0,
        sum,
    )
}

/// `raw` on the branch nearest `previous`.
fn continue_angle(previous: f64, raw: f64) -> f64 {
    previous + (raw - previous + std::f64::consts::PI).rem_euclid(TAU) - std::f64::consts::PI
}

const fn supported_curve(curve: &EdgeCurve) -> bool {
    matches!(
        curve,
        EdgeCurve::Line | EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) | EdgeCurve::NurbsCurve(_)
    )
}

/// Panel boundaries from `from` to `to` (either direction): NURBS knots and
/// arcs of at most [`MAX_ARC_SWEEP`].
fn panel_breaks(curve: &EdgeCurve, from: f64, to: f64) -> Vec<f64> {
    let (lo, hi) = if from <= to { (from, to) } else { (to, from) };
    let mut breaks = vec![lo];
    match curve {
        EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) => {
            let pieces = ((hi - lo) / MAX_ARC_SWEEP).ceil().max(1.0);
            // Bounded by the edge domain; a non-finite span was refused.
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let pieces = pieces.min(1024.0) as usize;
            for k in 1..pieces {
                #[allow(clippy::cast_precision_loss)]
                breaks.push(lo + (hi - lo) * (k as f64) / (pieces as f64));
            }
        }
        EdgeCurve::NurbsCurve(nurbs) => {
            let span = hi - lo;
            for &knot in nurbs.knots() {
                if knot > lo + 1e-12 * span
                    && knot < hi - 1e-12 * span
                    && breaks.last().is_none_or(|&last| knot > last)
                {
                    breaks.push(knot);
                }
            }
        }
        EdgeCurve::Line | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {}
    }
    breaks.push(hi);
    if from > to {
        breaks.reverse();
    }
    breaks
}

/// Point and parameter derivative of `curve` at `t`.
fn point_and_derivative(
    curve: &EdgeCurve,
    t: f64,
    start: Point3,
    end: Point3,
) -> Option<(Point3, Vec3)> {
    match curve {
        EdgeCurve::Line => Some((start + (end - start) * t, end - start)),
        EdgeCurve::Circle(c) => Some((c.evaluate(t), c.derivative_pair(t)?.0)),
        EdgeCurve::Ellipse(e) => Some((e.evaluate(t), e.derivative_pair(t)?.0)),
        EdgeCurve::NurbsCurve(n) => {
            let ders = n.derivatives(t, 1);
            Some((Point3::new(0.0, 0.0, 0.0) + *ders.first()?, *ders.get(1)?))
        }
        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => None,
    }
}

/// `(G du/dt, du/dt)` at one point, or `None` at an axis crossing.
fn integrand(
    chart: &Chart,
    p: Point3,
    d: Vec3,
    tube_angle: &mut Option<f64>,
) -> Option<(f64, f64)> {
    let (x, y, z) = chart.local(p);
    let (dx, dy) = (d.dot(chart.x), d.dot(chart.y));
    let rho2 = x.mul_add(x, y * y);
    let clearance = chart.axis_clearance();
    // Also refuses a NaN distance.
    if rho2.partial_cmp(&(clearance * clearance)) != Some(std::cmp::Ordering::Greater) {
        return None;
    }
    let du = x.mul_add(dy, -(y * dx)) / rho2;
    let g = match chart.kernel {
        Kernel::Cylinder { r } | Kernel::Sphere { r, .. } => r * z,
        Kernel::Cone { k } => {
            if z < -clearance {
                return None;
            }
            k * z * z
        }
        Kernel::Torus { major, minor } => {
            let raw = z.atan2(rho2.sqrt() - major);
            // Continue on the branch nearest the previous point; panels are
            // refined until the tube angle moves far less than π across one.
            let v = tube_angle.map_or(raw, |previous| continue_angle(previous, raw));
            *tube_angle = Some(v);
            minor * major.mul_add(v, z)
        }
    };
    (g.is_finite() && du.is_finite()).then_some((g * du, du))
}

/// One Gauss panel from `a` to `b`: `(∫ G du, ∫ du)`.
fn gauss_panel(
    chart: &Chart,
    curve: &EdgeCurve,
    start: Point3,
    end: Point3,
    a: f64,
    b: f64,
    tube_angle: &mut Option<f64>,
) -> Option<(f64, f64)> {
    let half = 0.5 * (b - a);
    let mid = 0.5 * (a + b);
    let mut term = 0.0;
    let mut winding = 0.0;
    // The abscissae ascend, so the nodes run from `a` to `b` whichever way
    // the panel is traversed and the torus tube angle is continued along the
    // curve.
    for point in gauss_legendre_points(PANEL_ORDER) {
        let t = half.mul_add(point.x, mid);
        let (p, d) = point_and_derivative(curve, t, start, end)?;
        let (g_du, du) = integrand(chart, p, d, tube_angle)?;
        term = (point.w * half).mul_add(g_du, term);
        winding = (point.w * half).mul_add(du, winding);
    }
    Some((term, winding))
}

/// Adaptive bisection of one panel, accumulated into `sum`. `false` refuses.
#[allow(clippy::too_many_arguments)]
fn integrate_panel(
    chart: &Chart,
    curve: &EdgeCurve,
    start: Point3,
    end: Point3,
    a: f64,
    b: f64,
    tolerance: f64,
    depth: u32,
    sum: &mut WireSum,
) -> bool {
    sum.panels += 1;
    if sum.panels > PANEL_BUDGET || depth > MAX_DEPTH {
        return false;
    }
    let mid = 0.5 * (a + b);
    let entry_angle = sum.tube_angle;
    let mut angle = entry_angle;
    let Some(whole) = gauss_panel(chart, curve, start, end, a, b, &mut angle) else {
        return false;
    };
    let mut angle = entry_angle;
    let Some(left) = gauss_panel(chart, curve, start, end, a, mid, &mut angle) else {
        return false;
    };
    let Some(right) = gauss_panel(chart, curve, start, end, mid, b, &mut angle) else {
        return false;
    };
    let halves = (left.0 + right.0, left.1 + right.1);
    // A torus panel whose tube angle moves by a half turn or more could have
    // been continued onto the wrong branch: refine it.
    let tube_jump = match (entry_angle, angle) {
        (Some(from), Some(to)) => (to - from).abs() >= std::f64::consts::FRAC_PI_2,
        _ => false,
    };
    if !tube_jump
        && (whole.0 - halves.0).abs() <= tolerance
        && (whole.1 - halves.1).abs() <= WINDING_PANEL_TOLERANCE
    {
        // Only a sphere's boundary can reach its axis where `G` is non-zero:
        // a cylinder's and a ring torus's never reach it, and a cone's only
        // at the apex, where `G = 0` and a jump in `u` carries no area.
        if matches!(chart.kernel, Kernel::Sphere { .. })
            && !continuous_in_u(chart, curve, start, end, a, b, halves.1)
        {
            return false;
        }
        sum.area_term += halves.0;
        sum.winding += halves.1;
        sum.tube_angle = angle;
        return true;
    }
    integrate_panel(chart, curve, start, end, a, mid, tolerance, depth + 1, sum)
        && integrate_panel(chart, curve, start, end, mid, b, tolerance, depth + 1, sum)
}

/// Whether the panel's integrated `∫ du` is the change of `u` between its
/// ends, modulo whole turns. A curve that crosses the polar axis between
/// quadrature nodes makes `u` jump by a half turn there, a jump `du` never
/// sees and which carries `G(pole) · Δu` of area; this refuses it.
fn continuous_in_u(
    chart: &Chart,
    curve: &EdgeCurve,
    start: Point3,
    end: Point3,
    a: f64,
    b: f64,
    winding: f64,
) -> bool {
    let u_at = |t: f64| {
        point_and_derivative(curve, t, start, end).map(|(p, _)| {
            let (x, y, _) = chart.local(p);
            y.atan2(x)
        })
    };
    let (Some(from), Some(to)) = (u_at(a), u_at(b)) else {
        return false;
    };
    let residual = winding - (to - from);
    (residual - TAU * (residual / TAU).round()).abs() <= PANEL_CONTINUITY_SLACK
}

#[cfg(test)]
mod tests;
