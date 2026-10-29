//! Isolated P-Class 2.5 clipping for regular rational ruled Bezier patches.
//!
//! The clipper consumes a supplied section and its two constant-v traces.
//! The discovery companion proposes one bounded shared source before proof;
//! neither invokes recognition, fitting, GFA, or the arrangement.
//! Whole-span homogeneous residual and normal bounds precede trim decisions.

use remus_math::compact_sort::CompactSort;
use remus_math::context::OperationContext;
use remus_math::curves2d::Curve2D;
use remus_math::nurbs::{curve::NurbsCurve, surface::NurbsSurface};
use remus_math::vec::{Point2, Point3};
use remus_topology::coedge::{CoedgeId, PeriodicWinding};
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::face_loop::LoopId;
use remus_topology::{Topology, TopologyError};

mod bernstein;
pub mod discovery;
use bernstein::{H, I, V};
#[cfg(test)]
pub(in crate::pave_filler) mod tests;

/// Typed refusal of the bounded rational ruled-section component.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum ClipError {
    /// A referenced topology entity is unavailable.
    #[error(transparent)]
    Topology(#[from] TopologyError),
    /// Geometry or charts are outside the qualified ruled-Bezier domain.
    #[error("unsupported NURBS chart, section, or boundary curve")]
    UnsupportedDomain,
    /// A required input is nonfinite or malformed.
    #[error("non-finite or invalid clipping input")]
    InvalidInput,
    /// A boundary has no authoritative pcurve or native edge trim.
    #[error("missing authoritative pcurve or edge trim")]
    MissingAuthority,
    /// Physical loops do not form the qualified disjoint rectangle domain.
    #[error("disconnected, nonrectangular, touching, or nested trim loops")]
    InvalidBoundary,
    /// Whole-span regularity or transversality could not be proved.
    #[error("regularity or transverse intersection could not be bounded")]
    UnresolvedContact,
    /// A trim event cannot be ordered or distinguished from a contact.
    #[error("section touches a trim corner, overlaps a boundary, or has unresolved event order")]
    AmbiguousBoundary,
    /// The entire stored curve could not be bounded on original geometry.
    #[error("original-geometry residual bound {bound} exceeds tolerance")]
    Residual {
        /// Conservative distance bound, in model units.
        bound: f64,
    },
    /// The caller's deterministic section-work allowance is insufficient.
    #[error("clipping work budget exceeded")]
    WorkBudgetExceeded,
    /// The caller cancelled the operation before completion.
    #[error("clipping cancelled")]
    Cancelled,
}
type Result<T> = std::result::Result<T, ClipError>;

/// The full native u domain maps affinely to the section's native domain.
#[derive(Debug, Clone, Copy)]
pub(super) struct FaceTrace {
    /// Original face whose native u domain maps to the source domain.
    pub face: FaceId,
    /// Constant coordinate in that face's original native v chart.
    pub v: f64,
}
/// One crossing of a source section with an original physical boundary use.
#[derive(Debug, Clone)]
pub struct BoundaryEvent {
    /// Original face containing this use.
    pub face: FaceId,
    /// Original physical outer or hole loop.
    pub boundary_loop: LoopId,
    /// Original directed boundary use, retained independently of proximity.
    pub coedge: CoedgeId,
    /// Original trimmed 3D edge.
    pub edge: EdgeId,
    /// Traversal direction of the original coedge.
    pub forward: bool,
    /// Crossing coordinate in the unchanged source curve's native domain.
    pub section_parameter: f64,
    /// Outward-rounded enclosure of the native source crossing coordinate.
    pub section_parameter_bound: [f64; 2],
    /// Coordinate on the original use's pcurve.
    pub pcurve_parameter: f64,
    /// Coordinate on the original 3D edge's authoritative trim.
    pub edge_parameter: f64,
    /// Evaluated, not welded; both original surfaces are checked at this event.
    pub surface_residuals: [f64; 2],
    /// Evaluated distance to the original trimmed 3D boundary edge.
    pub boundary_residual: f64,
    chart_u: f64,
    chart_domain: (f64, f64),
}
/// A complete contiguous material interval of the unchanged source section.
#[derive(Debug)]
pub struct RetainedInterval {
    /// Increasing native parameters on the source curve.
    pub source_range: [f64; 2],
    /// Empty only at an interior source endpoint. Equal parameters do not
    /// identify boundary uses; every use retains its own certificate.
    pub endpoints: [Vec<BoundaryEvent>; 2],
}
/// Certified trim result; these records describe the topology at call time.
#[derive(Debug)]
pub struct ClippedSection {
    /// Every retained material interval, in increasing native source order.
    pub intervals: Vec<RetainedInterval>,
    /// All trim events, including events bounding discarded material.
    pub events: Vec<BoundaryEvent>,
    /// Conservative whole-source-span distance bounds in model units.
    pub surface_residual_bounds: [f64; 2],
}
struct Patch<'a> {
    surface: &'a NurbsSurface,
    ends: [H; 2],
    section: H,
    normal: V,
    v: f64,
}
struct Boundary {
    coedge: CoedgeId,
    edge: EdgeId,
    forward: bool,
    uv: [Point2; 2],
    pcurve_range: [f64; 2],
    edge_range: [f64; 2],
}
struct Rectangle {
    id: LoopId,
    uses: Vec<Boundary>,
    bounds: [f64; 4],
}

fn finite(x: f64) -> Result<()> {
    if x.is_finite() {
        Ok(())
    } else {
        Err(ClipError::InvalidInput)
    }
}
fn fraction(x: f64, range: (f64, f64)) -> Result<f64> {
    finite(x)?;
    if x < range.0 || x > range.1 || range.0 >= range.1 {
        return Err(ClipError::UnsupportedDomain);
    }
    Ok((x - range.0) / (range.1 - range.0))
}
fn fraction_bound(x: f64, range: (f64, f64)) -> I {
    if same(x, range.0) {
        return I::exact(0.0);
    }
    if same(x, range.1) {
        return I::exact(1.0);
    }
    I::exact(x)
        .sub(I::exact(range.0))
        .div(I::exact(range.1).sub(I::exact(range.0)))
}
fn same(a: f64, b: f64) -> bool {
    (a - b).abs() <= 0.0
}
// Structural equality and event order use no model-unit tolerance.
fn exact_sum(a: f64, b: f64) -> bool {
    let sum = a + b;
    let b_virtual = sum - a;
    same((a - (sum - b_virtual)) + (b - b_virtual), 0.0)
}
fn bezier(knots: &[f64], degree: usize, count: usize) -> Result<()> {
    if !(1..=3).contains(&degree) || count != degree + 1 || knots.len() != 2 * (degree + 1) {
        return Err(ClipError::UnsupportedDomain);
    }
    let (lo, hi) = (knots[0], knots[knots.len() - 1]);
    if !lo.is_finite()
        || !hi.is_finite()
        || lo >= hi
        || !(hi - lo).is_finite()
        || knots[..=degree].iter().any(|x| !same(*x, lo))
        || knots[degree + 1..].iter().any(|x| !same(*x, hi))
    {
        return Err(ClipError::UnsupportedDomain);
    }
    Ok(())
}
fn curve_h(curve: &NurbsCurve, origin: Point3) -> Result<H> {
    bezier(curve.knots(), curve.degree(), curve.control_points().len())?;
    if curve.weights().len() != curve.control_points().len()
        || curve.weights().iter().any(|w| !w.is_finite() || *w <= 0.0)
        || curve
            .control_points()
            .iter()
            .any(|p| p.0.iter().any(|x| !x.is_finite()))
    {
        return Err(ClipError::InvalidInput);
    }

    Ok(bernstein::homogeneous(
        curve.control_points(),
        curve.weights(),
        origin,
    ))
}
fn check_residual(a: &H, b: &H, tolerance: f64) -> Result<f64> {
    let bound = bernstein::residual(a, b);
    if !bound.is_finite() || bound > tolerance {
        return Err(ClipError::Residual { bound });
    }
    Ok(bound)
}
fn patch(topo: &Topology, trace: FaceTrace, origin: Point3) -> Result<Patch<'_>> {
    let FaceSurface::Nurbs(s) = topo.face(trace.face)?.surface() else {
        return Err(ClipError::UnsupportedDomain);
    };
    bezier(s.knots_u(), s.degree_u(), s.control_points().len())?;
    if s.control_points()
        .iter()
        .any(|row| row.len() != 2 || row.iter().any(|p| p.0.iter().any(|x| !x.is_finite())))
        || s.weights().len() != s.control_points().len()
        || s.weights()
            .iter()
            .any(|row| row.len() != 2 || row.iter().any(|w| !w.is_finite() || *w <= 0.0))
    {
        return Err(ClipError::InvalidInput);
    }

    bezier(
        s.knots_v(),
        s.degree_v(),
        s.control_points()
            .first()
            .ok_or(ClipError::InvalidInput)?
            .len(),
    )?;
    if s.degree_v() != 1
        || s.is_periodic_u()
        || s.is_periodic_v()
        || s.weights().iter().any(|w| !same(w[0], w[1]))
    {
        return Err(ClipError::UnsupportedDomain);
    }
    fraction(trace.v, s.domain_v())?;
    let v = fraction_bound(trace.v, s.domain_v());
    let weights: Vec<_> = s.weights().iter().map(|w| w[0]).collect();
    let ends: [H; 2] = std::array::from_fn(|j| {
        bernstein::homogeneous(
            &s.control_points().iter().map(|r| r[j]).collect::<Vec<_>>(),
            &weights,
            origin,
        )
    });
    let direction = std::array::from_fn(|j| bernstein::sub(&ends[1][j], &ends[0][j]));
    let normals = [
        bernstein::normals(&ends[0], &direction),
        bernstein::normals(&ends[1], &direction),
    ];
    if !bernstein::hemisphere(&[&normals[0], &normals[1]]) {
        return Err(ClipError::UnresolvedContact);
    }
    let section = bernstein::blend(&ends[0], &ends[1], v);
    let normal = bernstein::normals(&section, &direction);
    Ok(Patch {
        surface: s,
        ends,
        section,
        normal,
        v: trace.v,
    })
}
fn boundary_h(patch: &Patch<'_>, uv: [Point2; 2]) -> Result<H> {
    let s = patch.surface;
    let _u = [
        fraction(uv[0].x(), s.domain_u())?,
        fraction(uv[1].x(), s.domain_u())?,
    ];
    let _v = [
        fraction(uv[0].y(), s.domain_v())?,
        fraction(uv[1].y(), s.domain_v())?,
    ];
    if same(uv[0].y(), uv[1].y()) && !same(uv[0].x(), uv[1].x()) {
        let h = bernstein::blend(
            &patch.ends[0],
            &patch.ends[1],
            fraction_bound(uv[0].y(), s.domain_v()),
        );
        Ok(bernstein::restrict(
            &h,
            fraction_bound(uv[0].x(), s.domain_u()),
            fraction_bound(uv[1].x(), s.domain_u()),
        ))
    } else if same(uv[0].x(), uv[1].x()) && !same(uv[0].y(), uv[1].y()) {
        let points: [[I; 4]; 2] = std::array::from_fn(|k| {
            bernstein::value(
                &bernstein::blend(
                    &patch.ends[0],
                    &patch.ends[1],
                    fraction_bound(uv[k].y(), s.domain_v()),
                ),
                fraction_bound(uv[0].x(), s.domain_u()),
            )
        });
        Ok(std::array::from_fn(|j| vec![points[0][j], points[1][j]]))
    } else {
        Err(ClipError::UnsupportedDomain)
    }
}
fn budget(context: &OperationContext, used: usize) -> Result<()> {
    if context
        .cancellation
        .as_ref()
        .is_some_and(remus_math::context::CancellationToken::is_cancelled)
    {
        return Err(ClipError::Cancelled);
    }
    if used > context.budgets.segments.min(256) {
        return Err(ClipError::WorkBudgetExceeded);
    }
    Ok(())
}
#[allow(clippy::too_many_lines)]
fn rectangles(
    topo: &Topology,
    trace: FaceTrace,
    patch: &Patch<'_>,
    origin: Point3,
    context: &OperationContext,
    used: &mut usize,
) -> Result<Vec<Rectangle>> {
    let loops = topo
        .loops_of_face(trace.face)
        .ok_or(ClipError::MissingAuthority)?;
    if loops.is_empty() {
        return Err(ClipError::InvalidBoundary);
    }
    let mut out = Vec::new();
    let mut edges = std::collections::BTreeSet::new();
    for &id in loops {
        *used = used.saturating_add(4);
        budget(context, *used)?;
        let boundary = topo.face_loop(id)?;
        if boundary.face() != trace.face || !boundary.is_closed() || boundary.coedges().len() != 4 {
            return Err(ClipError::InvalidBoundary);
        }
        let mut uses = Vec::new();
        for &cid in boundary.coedges() {
            let coedge = topo.coedge(cid)?;
            if !edges.insert(coedge.edge()) {
                return Err(ClipError::InvalidBoundary);
            }
            if coedge.parent_loop() != id || coedge.periodic_winding() != PeriodicWinding::ZERO {
                return Err(ClipError::UnsupportedDomain);
            }
            let pc = coedge.pcurve().ok_or(ClipError::MissingAuthority)?;
            let Curve2D::Line(line) = pc.curve() else {
                return Err(ClipError::UnsupportedDomain);
            };
            let pr = [pc.t_start(), pc.t_end()];
            finite(pr[0])?;
            finite(pr[1])?;
            if same(pr[0], pr[1]) {
                return Err(ClipError::InvalidBoundary);
            }
            // This first adapter accepts exact axis-aligned line charts. A
            // rounded endpoint is not a certificate of loop connectivity.
            let d = line.direction();
            if !same(pr[0], 0.0)
                || !((same(d.x(), 0.0) && same(d.y().abs(), 1.0))
                    || (same(d.y(), 0.0) && same(d.x().abs(), 1.0)))
                || !exact_sum(line.origin().x(), d.x() * pr[1])
                || !exact_sum(line.origin().y(), d.y() * pr[1])
            {
                return Err(ClipError::UnsupportedDomain);
            }
            let uv = [pc.evaluate(pr[0]), pc.evaluate(pr[1])];
            let exact = boundary_h(patch, uv)?;
            let edge = topo.edge(coedge.edge())?;
            let range = edge
                .strict_domain()
                .map_err(|_| ClipError::MissingAuthority)?;
            let range = if coedge.is_forward() {
                [range.0, range.1]
            } else {
                [range.1, range.0]
            };
            let pts = [
                topo.vertex(edge.start())?.point(),
                topo.vertex(edge.end())?.point(),
            ];
            let mut h = match edge.curve() {
                EdgeCurve::Line => bernstein::homogeneous(&pts, &[1.0, 1.0], origin),
                EdgeCurve::NurbsCurve(c) => {
                    fraction(range[0], c.domain())?;
                    let a = fraction_bound(range[0], c.domain());
                    fraction(range[1], c.domain())?;
                    let b = fraction_bound(range[1], c.domain());
                    bernstein::restrict(&curve_h(c, origin)?, a, b)
                }
                EdgeCurve::Circle(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => return Err(ClipError::UnsupportedDomain),
            };
            if matches!(edge.curve(), EdgeCurve::Line) && !coedge.is_forward() {
                for p in &mut h {
                    p.reverse();
                }
            }
            check_residual(&h, &exact, context.tolerance.linear)?;
            // Endpoints are certified against their stored topology vertices too.
            for k in 0..2 {
                let vertex = pts[if coedge.is_forward() { k } else { 1 - k }];
                let value =
                    bernstein::value(&h, I::exact(f64::from(u32::try_from(k).unwrap_or(0))));
                let point: H = std::array::from_fn(|j| vec![value[j]]);
                check_residual(
                    &point,
                    &bernstein::homogeneous(&[vertex], &[1.0], origin),
                    context.tolerance.linear,
                )?;
            }
            uses.push(Boundary {
                coedge: cid,
                edge: coedge.edge(),
                forward: coedge.is_forward(),
                uv,
                pcurve_range: pr,
                edge_range: range,
            });
        }
        for i in 0..4 {
            let a = &uses[i];
            let b = &uses[(i + 1) % 4];
            let ea = topo.edge(a.edge)?;
            let eb = topo.edge(b.edge)?;
            let end = if a.forward { ea.end() } else { ea.start() };
            let start = if b.forward { eb.start() } else { eb.end() };
            if end != start || !same(a.uv[1].x(), b.uv[0].x()) || !same(a.uv[1].y(), b.uv[0].y()) {
                return Err(ClipError::InvalidBoundary);
            }
            let vertical_a = same(a.uv[0].x(), a.uv[1].x());
            let vertical_b = same(b.uv[0].x(), b.uv[1].x());
            if vertical_a == vertical_b {
                return Err(ClipError::InvalidBoundary);
            }
            for other in &uses[..i] {
                let e = topo.edge(other.edge)?;
                let other_start = if other.forward { e.start() } else { e.end() };
                let a_start = if a.forward { ea.start() } else { ea.end() };
                if a_start == other_start {
                    return Err(ClipError::InvalidBoundary);
                }
            }
        }
        let bounds = [
            uses.iter()
                .map(|u| u.uv[0].x())
                .fold(f64::INFINITY, f64::min),
            uses.iter()
                .map(|u| u.uv[0].x())
                .fold(f64::NEG_INFINITY, f64::max),
            uses.iter()
                .map(|u| u.uv[0].y())
                .fold(f64::INFINITY, f64::min),
            uses.iter()
                .map(|u| u.uv[0].y())
                .fold(f64::NEG_INFINITY, f64::max),
        ];
        out.push(Rectangle { id, uses, bounds });
    }
    let outer = out[0].bounds;
    for (i, r) in out.iter().enumerate().skip(1) {
        let b = r.bounds;
        if b[0] <= outer[0] || b[1] >= outer[1] || b[2] <= outer[2] || b[3] >= outer[3] {
            return Err(ClipError::InvalidBoundary);
        }
        for prev in &out[1..i] {
            let a = prev.bounds;
            if b[0] <= a[1] && b[1] >= a[0] && b[2] <= a[3] && b[3] >= a[2] {
                return Err(ClipError::InvalidBoundary);
            }
        }
    }
    Ok(out)
}
fn inside(loops: &[Rectangle], u: I, v: f64) -> Result<bool> {
    let contains = |r: &Rectangle| -> Result<bool> {
        let b = r.bounds;
        if v <= b[2] || v >= b[3] || u.hi < b[0] || u.lo > b[1] {
            return Ok(false);
        }
        if u.lo <= b[0] || u.hi >= b[1] {
            return Err(ClipError::AmbiguousBoundary);
        }
        Ok(true)
    };
    let mut material = contains(&loops[0])?;
    for hole in &loops[1..] {
        material &= !contains(hole)?;
    }
    Ok(material)
}

/// Retain the authoritative stored endpoints instead of recomposing them
/// through a rounded span. Clamping an interior map into its native domain
/// cannot increase its distance from the exact in-domain affine image.
fn source_parameter(domain: (f64, f64), fraction: f64) -> f64 {
    if same(fraction, 0.0) {
        domain.0
    } else if same(fraction, 1.0) {
        domain.1
    } else {
        (domain.0 + (domain.1 - domain.0) * fraction).clamp(domain.0, domain.1)
    }
}

/// Read-only and atomic: a refusal cannot expose intervals or mutate topology.
#[allow(clippy::too_many_lines)]
pub(super) fn clip_section(
    topo: &Topology,
    traces: [FaceTrace; 2],
    section: &NurbsCurve,
    context: &OperationContext,
) -> Result<ClippedSection> {
    budget(context, 1)?;
    if !context.tolerance.linear.is_finite() || context.tolerance.linear <= 0.0 {
        return Err(ClipError::InvalidInput);
    }
    let origin = *section
        .control_points()
        .first()
        .ok_or(ClipError::InvalidInput)?;
    let source = curve_h(section, origin)?;
    let patches = [
        patch(topo, traces[0], origin)?,
        patch(topo, traces[1], origin)?,
    ];
    let transverse = bernstein::cross(&patches[0].normal, &patches[1].normal);
    if !bernstein::hemisphere(&[&transverse]) {
        return Err(ClipError::UnresolvedContact);
    }
    let bounds = [
        check_residual(&source, &patches[0].section, context.tolerance.linear)?,
        check_residual(&source, &patches[1].section, context.tolerance.linear)?,
    ];
    let mut used = 0;
    let loops = [
        rectangles(topo, traces[0], &patches[0], origin, context, &mut used)?,
        rectangles(topo, traces[1], &patches[1], origin, context, &mut used)?,
    ];
    let (t0, t1) = section.domain();
    let mut events: Vec<(f64, I, BoundaryEvent)> = Vec::new();
    for face in 0..2 {
        let s = patches[face].surface;
        for r in &loops[face] {
            budget(context, used)?;
            // Corner and boundary-overlap cases require the future contact owner.
            if same(traces[face].v, r.bounds[2]) || same(traces[face].v, r.bounds[3]) {
                return Err(ClipError::AmbiguousBoundary);
            }
            if traces[face].v < r.bounds[2] || traces[face].v > r.bounds[3] {
                continue;
            }
            for b in &r.uses {
                if !same(b.uv[0].x(), b.uv[1].x()) {
                    continue;
                }
                let f = fraction(b.uv[0].x(), s.domain_u())?;
                let fb = fraction_bound(b.uv[0].x(), s.domain_u());
                let g = (traces[face].v - b.uv[0].y()) / (b.uv[1].y() - b.uv[0].y());
                let t = source_parameter((t0, t1), f);
                let tb = I::exact(t0).add(I::exact(t1).sub(I::exact(t0)).mul(fb));
                let point = section.evaluate(t);
                let residuals = std::array::from_fn(|j| {
                    let domain = patches[j].surface.domain_u();
                    (point
                        - patches[j]
                            .surface
                            .evaluate(domain.0 + (domain.1 - domain.0) * f, patches[j].v))
                    .length()
                });
                if residuals
                    .iter()
                    .any(|r| !r.is_finite() || *r > context.tolerance.linear)
                {
                    return Err(ClipError::Residual {
                        bound: residuals.into_iter().fold(0.0, f64::max),
                    });
                }
                let edge_parameter = b.edge_range[0] + g * (b.edge_range[1] - b.edge_range[0]);
                let edge = topo.edge(b.edge)?;
                let boundary_residual = (point
                    - edge.curve().evaluate_with_endpoints(
                        edge_parameter,
                        topo.vertex(edge.start())?.point(),
                        topo.vertex(edge.end())?.point(),
                    ))
                .length();
                if !boundary_residual.is_finite() || boundary_residual > context.tolerance.linear {
                    return Err(ClipError::Residual {
                        bound: boundary_residual,
                    });
                }
                events.push((
                    f,
                    fb,
                    BoundaryEvent {
                        face: traces[face].face,
                        boundary_loop: r.id,
                        coedge: b.coedge,
                        edge: b.edge,
                        forward: b.forward,
                        section_parameter: t,
                        section_parameter_bound: [tb.lo, tb.hi],
                        pcurve_parameter: b.pcurve_range[0]
                            + g * (b.pcurve_range[1] - b.pcurve_range[0]),
                        edge_parameter,
                        surface_residuals: residuals,
                        boundary_residual,
                        chart_u: b.uv[0].x(),
                        chart_domain: s.domain_u(),
                    },
                ));
            }
        }
    }
    events.compact_sort_by(|a, b| a.0.total_cmp(&b.0));
    for pair in events.windows(2) {
        // Different normalized crossings must remain different ordered
        // native source parameters, including crossings later discarded as
        // outside material. Multiple original uses at one cut stay valid.
        if !same(pair[0].0, pair[1].0) && pair[0].2.section_parameter >= pair[1].2.section_parameter
        {
            return Err(ClipError::AmbiguousBoundary);
        }
        if pair[0].1.hi >= pair[1].1.lo {
            let (a, b) = (&pair[0], &pair[1]);
            let same_chart_coordinate = same(a.2.chart_u, b.2.chart_u)
                && same(a.2.chart_domain.0, b.2.chart_domain.0)
                && same(a.2.chart_domain.1, b.2.chart_domain.1);
            let same_source_end =
                (same(a.1.lo, 0.0) && same(a.1.hi, 0.0) && same(b.1.lo, 0.0) && same(b.1.hi, 0.0))
                    || (same(a.1.lo, 1.0)
                        && same(a.1.hi, 1.0)
                        && same(b.1.lo, 1.0)
                        && same(b.1.hi, 1.0));
            if !same_chart_coordinate && !same_source_end {
                return Err(ClipError::AmbiguousBoundary);
            }
        }
    }
    let mut cuts = vec![0.0, 1.0];
    cuts.extend(events.iter().map(|e| e.0));
    cuts.compact_sort_by(f64::total_cmp);
    cuts.dedup_by(|a, b| same(*a, *b));
    let mut intervals = Vec::new();
    for w in cuts.windows(2) {
        let mid = w[0] + (w[1] - w[0]) * 0.5;
        if mid <= w[0] || mid >= w[1] {
            return Err(ClipError::AmbiguousBoundary);
        }
        let mut material = true;
        for j in 0..2 {
            let d = patches[j].surface.domain_u();
            let u = I::exact(d.0).add(I::exact(d.1).sub(I::exact(d.0)).mul(I::exact(mid)));
            material &= inside(&loops[j], u, traces[j].v)?;
        }
        if material {
            let range = [w[0], w[1]].map(|f| source_parameter((t0, t1), f));
            if range[0] >= range[1] {
                return Err(ClipError::AmbiguousBoundary);
            }
            intervals.push(RetainedInterval {
                source_range: range,
                endpoints: std::array::from_fn(|j| {
                    events
                        .iter()
                        .filter(|e| same(e.0, w[j]))
                        .map(|e| e.2.clone())
                        .collect()
                }),
            });
        }
    }
    Ok(ClippedSection {
        intervals,
        events: events.into_iter().map(|e| e.2).collect(),
        surface_residual_bounds: bounds,
    })
}

/// Outward-rounded whole-span distance to a cylindrical carrier and the
/// range of its radial normal projected on `direction`. The latter's sign
/// distinguishes regular cylinder-cylinder quadratic branches. Coefficients
/// describe the actual stored Bezier span; no numerical knot refinement runs.
pub(super) fn bezier_cylinder_residual_bound(
    points: &[Point3],
    weights: &[f64],
    cylinder: &remus_math::surfaces::CylindricalSurface,
    direction: remus_math::vec::Vec3,
) -> Option<(f64, [f64; 2])> {
    if points.len() < 2
        || points.len() > 4
        || weights.len() != points.len()
        || points.iter().any(|p| p.0.iter().any(|x| !x.is_finite()))
        || weights.iter().any(|w| !w.is_finite() || *w <= 0.0)
        || direction.0.iter().any(|x| !x.is_finite())
        || !cylinder.radius().is_finite()
        || cylinder.radius() <= 0.0
    {
        return None;
    }
    let h = bernstein::homogeneous(points, weights, cylinder.origin());
    let degree = points.len() - 1;
    let axis = cylinder.axis().0.map(I::exact);
    let direction = direction.0.map(I::exact);
    let axis_squared = axis.into_iter().fold(I::exact(0.0), |s, x| s.add(x.mul(x)));
    let axis_direction = (0..3).fold(I::exact(0.0), |s, j| s.add(axis[j].mul(direction[j])));
    if !axis_squared.lo.is_finite() || axis_squared.lo <= 0.0 {
        return None;
    }
    let mut squared = vec![I::exact(0.0); 2 * degree + 1];
    let mut along = vec![I::exact(0.0); degree + 1];
    let mut toward = vec![I::exact(0.0); degree + 1];
    for j in 0..3 {
        for (sum, value) in squared.iter_mut().zip(bernstein::product(&h[j], &h[j])) {
            *sum = sum.add(value);
        }
        for k in 0..=degree {
            along[k] = along[k].add(h[j][k].mul(axis[j]));
            toward[k] = toward[k].add(h[j][k].mul(direction[j]));
        }
    }
    let along_squared = bernstein::product(&along, &along);
    let weight_squared = bernstein::product(&h[3], &h[3]);
    let radius_squared = I::exact(cylinder.radius()).mul(I::exact(cylinder.radius()));
    let bound = squared
        .iter()
        .zip(&along_squared)
        .zip(&weight_squared)
        .map(|((s, a), w)| {
            s.mul(axis_squared)
                .sub(*a)
                .sub(w.mul(radius_squared).mul(axis_squared))
                .abs_max()
        })
        .fold(0.0, f64::max);
    let weight_lo = h[3].iter().map(|w| w.lo).fold(f64::INFINITY, f64::min);
    let weight_hi = h[3].iter().map(|w| w.hi).fold(f64::NEG_INFINITY, f64::max);
    let denominator = I::exact(axis_squared.lo)
        .mul(I::exact(weight_lo))
        .mul(I::exact(weight_lo))
        .lo;
    if !denominator.is_finite() || denominator <= 0.0 || weight_lo <= 0.0 {
        return None;
    }
    // Stored frame axes are normalized in floating point. Bound the defect
    // of their projected Gram matrix from the identity, rather than assuming
    // an exactly orthonormal radial basis. Its maximum absolute row sum d
    // bounds each squared singular-value defect; for d < 1, |sigma-1| <= d.
    // Thus radius*d bounds the distance between the stored parametric carrier
    // and the ideal axis/radius cylinder used by the radial polynomial.
    let radial_axes = [
        cylinder.x_axis().0.map(I::exact),
        cylinder.y_axis().0.map(I::exact),
    ];
    let dot = |a: [I; 3], b: [I; 3]| (0..3).fold(I::exact(0.0), |sum, j| sum.add(a[j].mul(b[j])));
    let axial = radial_axes.map(|x| dot(x, axis));
    let gxx = dot(radial_axes[0], radial_axes[0]).sub(axial[0].mul(axial[0]).div(axis_squared));
    let gyy = dot(radial_axes[1], radial_axes[1]).sub(axial[1].mul(axial[1]).div(axis_squared));
    let gxy = dot(radial_axes[0], radial_axes[1]).sub(axial[0].mul(axial[1]).div(axis_squared));
    let off_diagonal = I::exact(gxy.abs_max());
    let defect = I::exact(gxx.sub(I::exact(1.0)).abs_max())
        .add(off_diagonal)
        .hi
        .max(
            I::exact(gyy.sub(I::exact(1.0)).abs_max())
                .add(off_diagonal)
                .hi,
        );
    if !defect.is_finite() || defect >= 1.0 {
        return None;
    }
    // |rho-r| = |rho²-r²|/(rho+r) <= |rho²-r²|/r.
    let radial = I::exact(bound)
        .div(I::exact(denominator))
        .div(I::exact(cylinder.radius()))
        .add(I::exact(cylinder.radius()).mul(I::exact(defect)))
        .hi;
    let projected: Vec<_> = toward
        .iter()
        .zip(&along)
        .map(|(a, b)| a.sub(b.mul(axis_direction).div(axis_squared)))
        .collect();
    let projected = I {
        lo: projected.iter().map(|x| x.lo).fold(f64::INFINITY, f64::min),
        hi: projected
            .iter()
            .map(|x| x.hi)
            .fold(f64::NEG_INFINITY, f64::max),
    }
    .div(I {
        lo: weight_lo,
        hi: weight_hi,
    });
    (radial.is_finite() && projected.lo.is_finite() && projected.hi.is_finite())
        .then_some((radial, [projected.lo, projected.hi]))
}

/// Sign range of `(curve-origin) × curve' · axis` over a stored Bezier
/// span. Positive weights leave this sign unchanged in homogeneous form;
/// a strict sign proves angular monotonicity around the cylinder axis.
pub(super) fn bezier_cylinder_angular_direction(
    points: &[Point3],
    weights: &[f64],
    cylinder: &remus_math::surfaces::CylindricalSurface,
) -> Option<[f64; 2]> {
    if !(2..=4).contains(&points.len())
        || points.len() != weights.len()
        || points.iter().any(|p| p.0.iter().any(|x| !x.is_finite()))
        || weights.iter().any(|w| !w.is_finite() || *w <= 0.0)
    {
        return None;
    }
    let h = bernstein::homogeneous(points, weights, cylinder.origin());
    let position: bernstein::V = std::array::from_fn(|j| h[j].clone());
    let degree = I::exact(f64::from(u32::try_from(points.len() - 1).ok()?));
    let derivative: bernstein::V = std::array::from_fn(|j| {
        h[j].windows(2)
            .map(|w| w[1].sub(w[0]).mul(degree))
            .collect()
    });
    let cross = bernstein::cross(&position, &derivative);
    let axis = cylinder.axis().0.map(I::exact);
    let dot: Vec<_> = (0..cross[0].len())
        .map(|k| (0..3).fold(I::exact(0.0), |sum, j| sum.add(cross[j][k].mul(axis[j]))))
        .collect();
    let lo = dot.iter().map(|x| x.lo).fold(f64::INFINITY, f64::min);
    let hi = dot.iter().map(|x| x.hi).fold(f64::NEG_INFINITY, f64::max);
    (lo.is_finite() && hi.is_finite()).then_some([lo, hi])
}

/// Whole-point distance to a cylinder's axial cap plane, including arithmetic
/// and axis-normalization roundoff. `cap` is its stored axial dot coordinate.
pub(super) fn point_cylinder_cap_distance_bound(
    point: Point3,
    cylinder: &remus_math::surfaces::CylindricalSurface,
    cap: f64,
) -> Option<f64> {
    if !cap.is_finite() || point.0.iter().any(|x| !x.is_finite()) {
        return None;
    }
    let axis = cylinder.axis().0.map(I::exact);
    let origin = cylinder.origin();
    let coordinate = (0..3).fold(I::exact(0.0), |sum, j| {
        sum.add(I::exact(point.0[j]).sub(I::exact(origin.0[j])).mul(axis[j]))
    });
    let norm_squared = axis
        .into_iter()
        .fold(I::exact(0.0), |sum, x| sum.add(x.mul(x)));
    let norm_lo = norm_squared.lo.sqrt().next_down();
    if !norm_lo.is_finite() || norm_lo <= 0.0 {
        return None;
    }
    let distance = I::exact(coordinate.sub(I::exact(cap)).abs_max())
        .div(I::exact(norm_lo))
        .hi;
    distance.is_finite().then_some(distance)
}

/// Whole-span upper bound on distance outside a finite cylinder's axial
/// interval. Positive rational weights preserve the control-point hull of
/// this linear coordinate. Limits use the cylinder's stored axial dot
/// coordinate; dividing by a lower axis norm gives physical distance.
pub(super) fn bezier_cylinder_finite_distance_bound(
    points: &[Point3],
    weights: &[f64],
    cylinder: &remus_math::surfaces::CylindricalSurface,
    limits: (f64, f64),
) -> Option<f64> {
    if !(2..=4).contains(&points.len())
        || points.len() != weights.len()
        || points.iter().any(|p| p.0.iter().any(|x| !x.is_finite()))
        || weights.iter().any(|w| !w.is_finite() || *w <= 0.0)
        || !limits.0.is_finite()
        || !limits.1.is_finite()
        || limits.0 > limits.1
    {
        return None;
    }
    let axis = cylinder.axis().0.map(I::exact);
    let origin = cylinder.origin();
    let coordinate = |point: Point3| {
        (0..3).fold(I::exact(0.0), |sum, j| {
            sum.add(I::exact(point.0[j]).sub(I::exact(origin.0[j])).mul(axis[j]))
        })
    };
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for &point in points {
        let value = coordinate(point);
        lo = lo.min(value.lo);
        hi = hi.max(value.hi);
    }
    let norm_squared = axis
        .into_iter()
        .fold(I::exact(0.0), |sum, x| sum.add(x.mul(x)));
    let norm_lo = norm_squared.lo.sqrt().next_down();
    if !lo.is_finite() || !hi.is_finite() || !norm_lo.is_finite() || norm_lo <= 0.0 {
        return None;
    }
    let outside = I::exact(limits.0)
        .sub(I::exact(lo))
        .hi
        .max(I::exact(hi).sub(I::exact(limits.1)).hi)
        .max(0.0);
    let distance = I::exact(outside).div(I::exact(norm_lo)).hi;
    distance.is_finite().then_some(distance)
}
