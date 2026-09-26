//! Isolated P-Class 2.5 clipping for regular rational ruled Bezier patches.
//!
//! This consumes a supplied section and its two constant-v traces. It does not
//! discover sections or invoke recognition, fitting, GFA, or the arrangement.
//! Whole-span homogeneous residual and normal bounds precede trim decisions.

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
use bernstein::{H, I, V};
#[cfg(test)]
pub(in crate::pave_filler) mod tests;

#[derive(Debug, thiserror::Error)]
pub(super) enum ClipError {
    #[error(transparent)]
    Topology(#[from] TopologyError),
    #[error("unsupported NURBS chart, section, or boundary curve")]
    UnsupportedDomain,
    #[error("non-finite or invalid clipping input")]
    InvalidInput,
    #[error("missing authoritative pcurve or edge trim")]
    MissingAuthority,
    #[error("disconnected, nonrectangular, touching, or nested trim loops")]
    InvalidBoundary,
    #[error("regularity or transverse intersection could not be bounded")]
    UnresolvedContact,
    #[error("section touches a trim corner, overlaps a boundary, or has unresolved event order")]
    AmbiguousBoundary,
    #[error("original-geometry residual bound {bound} exceeds tolerance")]
    Residual { bound: f64 },
    #[error("clipping work budget exceeded")]
    WorkBudgetExceeded,
    #[error("clipping cancelled")]
    Cancelled,
}
type Result<T> = std::result::Result<T, ClipError>;

/// The full native u domain maps affinely to the section's native domain.
#[derive(Debug, Clone, Copy)]
pub(super) struct FaceTrace {
    pub face: FaceId,
    pub v: f64,
}
#[derive(Debug, Clone)]
pub(super) struct BoundaryEvent {
    pub face: FaceId,
    pub boundary_loop: LoopId,
    pub coedge: CoedgeId,
    pub edge: EdgeId,
    pub forward: bool,
    pub section_parameter: f64,
    pub section_parameter_bound: [f64; 2],
    pub pcurve_parameter: f64,
    pub edge_parameter: f64,
    /// Evaluated, not welded; both original surfaces are checked at this event.
    pub surface_residuals: [f64; 2],
    pub boundary_residual: f64,
    chart_u: f64,
    chart_domain: (f64, f64),
}
#[derive(Debug)]
pub(super) struct RetainedInterval {
    pub source_range: [f64; 2],
    /// Empty only at an interior source endpoint. Equal parameters do not
    /// identify boundary uses; every use retains its own certificate.
    pub endpoints: [Vec<BoundaryEvent>; 2],
}
#[derive(Debug)]
pub(super) struct ClippedSection {
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
                _ => return Err(ClipError::UnsupportedDomain),
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
                let t = t0 + (t1 - t0) * f;
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
    events.sort_by(|a, b| a.0.total_cmp(&b.0));
    for pair in events.windows(2) {
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
    cuts.sort_by(f64::total_cmp);
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
            let range = [t0 + (t1 - t0) * w[0], t0 + (t1 - t0) * w[1]];
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
