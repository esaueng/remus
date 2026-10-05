//! Edge curve sampling and parametrization.

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;

const MAX_EDGE_SAMPLE_POINTS: usize = 16_384;

/// Reject sampling requests that could exhaust memory or CPU on hostile trims.
pub(super) fn enforce_edge_sample_limit(n: usize) -> Result<(), crate::OperationsError> {
    if n > MAX_EDGE_SAMPLE_POINTS {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!(
                "edge sampling needs {n} points; limit is {MAX_EDGE_SAMPLE_POINTS}; increase tolerances"
            ),
        });
    }
    Ok(())
}

/// Combined linear+angular segment count for a circular arc.
///
/// Delegates to [`remus_math::chord::segments_for_chord_deviation_with_angle`]
/// with no minimum-edge-length clamp. `apply_curvature_floor` is forwarded:
/// constant-curvature circles pass `false` (the chord formula is exact),
/// variable/doubly-curved geometry passes `true`.
pub(super) fn segments_for_chord_deviation_a(
    radius: f64,
    arc_range: f64,
    deflection: f64,
    angular_tol: f64,
    apply_curvature_floor: bool,
) -> usize {
    remus_math::chord::segments_for_chord_deviation_with_angle(
        radius,
        arc_range,
        deflection,
        angular_tol,
        0.0,
        apply_curvature_floor,
    )
}

/// Segment count for an *open* conic (hyperbola or parabola) sub-arc.
///
/// These curves have no angular parameter, so the circular
/// `segments_for_chord_deviation_a` cannot be fed their raw parameter span.
/// Instead the arc is treated as an equivalent circular arc of the TIGHTEST
/// osculating circle on the span: radius `min_radius`, swept angle
/// `arc_len / min_radius`. That is conservative (every other point of the
/// arc is flatter than the tightest one) and it is dimensionless in the
/// right way — both inputs carry units of length, so the count is invariant
/// when the model and `deflection` are scaled together.
pub(super) fn open_conic_segments(
    min_radius: f64,
    arc_len: f64,
    deflection: f64,
    angular_tol: f64,
) -> usize {
    if !min_radius.is_finite() || min_radius <= 0.0 || !arc_len.is_finite() || arc_len <= 0.0 {
        return 1;
    }
    segments_for_chord_deviation_a(
        min_radius,
        arc_len / min_radius,
        deflection,
        angular_tol,
        false,
    )
}

/// Compute orthogonal axes for a plane given its normal.
///
/// Falls back to identity axes if the normal is degenerate (should not
/// happen for valid face data).
pub(super) fn plane_axes(normal: Vec3) -> (Vec3, Vec3) {
    let up = if normal.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u_axis = normal
        .cross(up)
        .normalize()
        .unwrap_or(Vec3::new(1.0, 0.0, 0.0));
    let v_axis = normal
        .cross(u_axis)
        .normalize()
        .unwrap_or(Vec3::new(0.0, 1.0, 0.0));
    (u_axis, v_axis)
}

/// Compute the number of sample points for an edge based on deflection.
///
/// Uses edge length and curvature to determine sampling density.
///
/// `circle_floor` selects whether a circular edge keeps the curvature floor.
/// Display callers pass `false` (the chord count is exact for a constant-
/// curvature circle); the boolean mesh-fallback passes `true` because its
/// co-refinement robustness depends on the denser floored sampling.
///
/// # Errors
///
/// Returns an error when a curved edge lacks valid stored parameter authority.
pub fn edge_sample_count(
    edge: &remus_topology::edge::Edge,
    deflection: f64,
    angular_tol: f64,
    circle_floor: bool,
) -> Result<usize, crate::OperationsError> {
    use remus_topology::edge::EdgeCurve;

    let count = match edge.curve() {
        EdgeCurve::Line => 2,
        EdgeCurve::Circle(c) => {
            let radius = c.radius();
            // Use the same segments_for_chord_deviation formula that
            // tessellate_analytic uses for the grid density. This ensures
            // edge sample points align with the analytic grid boundary,
            // allowing the snap path to achieve watertight stitching.
            let (t_start, t_end) = circle_param_range(edge)?;
            let arc_range = (t_end - t_start).abs();
            segments_for_chord_deviation_a(radius, arc_range, deflection, angular_tol, circle_floor)
                + 1
        }
        EdgeCurve::Hyperbola(h) => {
            let (t0, t1) = crate::authoritative_edge_domain(edge, "edge sample count")?;
            open_conic_segments(
                h.min_curvature_radius(t0, t1),
                h.arc_length(t0, t1),
                deflection,
                angular_tol,
            ) + 1
        }
        EdgeCurve::Parabola(p) => {
            let (t0, t1) = crate::authoritative_edge_domain(edge, "edge sample count")?;
            open_conic_segments(
                p.min_curvature_radius(t0, t1),
                p.arc_length(t0, t1),
                deflection,
                angular_tol,
            ) + 1
        }
        EdgeCurve::Ellipse(ellipse) => {
            // Density is driven by the LARGEST radius of curvature (a^2/b, at the
            // minor-axis ends). Under uniform-parameter sampling the per-segment
            // chord deviation is set by how far the parameter sweeps in arc length,
            // which peaks where curvature is lowest; the small-radius criterion
            // (b^2/a) satisfies pointwise sag but lets the integrated (area/volume)
            // error grow ~15x. Using a^2/b keeps both bounded.
            let a = ellipse.semi_major();
            let b = ellipse.semi_minor();
            let max_curv_radius = a * a / b;
            let (ts, te) = crate::authoritative_edge_domain(edge, "edge sample count")?;
            let arc_range = (te - ts).abs();
            segments_for_chord_deviation_a(
                max_curv_radius,
                arc_range,
                deflection,
                angular_tol,
                true,
            )
            .min(4096)
        }
        EdgeCurve::NurbsCurve(nurbs) => {
            // Adaptive: coarse-pass deviation measurement, then refine if the
            // chord sag OR the per-segment turn exceeds tolerance.
            // Endpoint-trimmed convention: a section edge can be a validated
            // sub-span of its stored curve; measuring the FULL knot domain
            // would size (and later sample) the whole parent curve.
            let (u0, u1) = crate::authoritative_edge_domain(edge, "edge sample count")?;
            let n_spans = nurbs
                .control_points()
                .len()
                .saturating_sub(nurbs.degree())
                .max(1);
            let coarse_n = (n_spans * 4).clamp(8, 128);
            let max_dev = measure_max_chord_deviation(nurbs, u0, u1, coarse_n);
            let max_turn = measure_max_segment_turn(nurbs, u0, u1, coarse_n);
            let sag_ok = max_dev <= deflection;
            let turn_ok = angular_tol <= 0.0 || max_turn <= angular_tol * 0.5;
            if sag_ok && turn_ok {
                coarse_n
            } else {
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                let sag_n = if sag_ok {
                    coarse_n
                } else {
                    ((coarse_n as f64) * (max_dev / deflection).sqrt()).ceil() as usize
                };
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                let turn_n = if turn_ok {
                    coarse_n
                } else {
                    ((coarse_n as f64) * (max_turn / (angular_tol * 0.5))).ceil() as usize
                };
                sag_n.max(turn_n).clamp(8, 4096)
            }
        }
    };
    Ok(count)
}

/// Measure the maximum midpoint chord deviation across `n` segments of a NURBS curve.
///
/// For each segment `[u_i, u_{i+1}]`, evaluates the curve at the midpoint and
/// measures its distance from the chord midpoint. Returns the maximum deviation.
pub(super) fn measure_max_chord_deviation(
    nurbs: &remus_math::nurbs::curve::NurbsCurve,
    u0: f64,
    u1: f64,
    n: usize,
) -> f64 {
    let mut max_dev: f64 = 0.0;
    #[allow(clippy::cast_precision_loss)]
    for i in 0..n {
        let t0 = u0 + (u1 - u0) * (i as f64) / (n as f64);
        let t1 = u0 + (u1 - u0) * ((i + 1) as f64) / (n as f64);
        let p0 = nurbs.evaluate(t0);
        let p1 = nurbs.evaluate(t1);
        let mid_chord = Point3::new(
            (p0.x() + p1.x()) * 0.5,
            (p0.y() + p1.y()) * 0.5,
            (p0.z() + p1.z()) * 0.5,
        );
        let mid_curve = nurbs.evaluate((t0 + t1) * 0.5);
        let dev = (mid_curve - mid_chord).length();
        max_dev = max_dev.max(dev);
    }
    max_dev
}

/// Measure the maximum tangent turn angle (radians) at segment midpoints of a
/// NURBS curve sampled over `n` uniform segments.
///
/// For each segment the curve tangent is compared at the segment endpoints; the
/// angle between them is the swing across that segment.
pub(super) fn measure_max_segment_turn(
    nurbs: &remus_math::nurbs::curve::NurbsCurve,
    u0: f64,
    u1: f64,
    n: usize,
) -> f64 {
    let mut max_turn: f64 = 0.0;
    #[allow(clippy::cast_precision_loss)]
    for i in 0..n {
        let t0 = u0 + (u1 - u0) * (i as f64) / (n as f64);
        let t1 = u0 + (u1 - u0) * ((i + 1) as f64) / (n as f64);
        if let (Ok(a), Ok(b)) = (nurbs.tangent(t0), nurbs.tangent(t1)) {
            let dot = a.dot(b).clamp(-1.0, 1.0);
            max_turn = max_turn.max(dot.acos());
        }
    }
    max_turn
}

/// Get the parameter range for a circle edge.
///
/// The stored interval is returned verbatim, including a lifted seam end,
/// major arc, or reversed traversal. Closed-circle producers must therefore
/// anchor their stored full turn at the topology seam vertex.
///
/// # Errors
///
/// Returns an error if the circle lacks valid stored parameter authority.
pub(super) fn circle_param_range(
    edge: &remus_topology::edge::Edge,
) -> Result<(f64, f64), crate::OperationsError> {
    crate::authoritative_edge_domain(edge, "circle sampling")
}

/// The parameter span an edge actually covers on its stored curve.
///
/// Curved edges report their stored
/// authoritative interval verbatim. Lines report `(0, length)` to match the
/// query surface's parameterization of line edges.
///
/// This is the authoritative span for consumers that rebuild edge geometry
/// outside the kernel (e.g. a 2D drawing export choosing between the two
/// arcs a circle edge's endpoints subtend) — reconstructing it from
/// endpoints alone flips intentional major arcs.
///
/// # Errors
///
/// Returns an error if vertex lookup fails for line endpoints or a curved edge
/// lacks valid stored parameter authority.
pub fn edge_param_span(
    topo: &Topology,
    edge: &remus_topology::edge::Edge,
) -> Result<(f64, f64), crate::OperationsError> {
    use remus_topology::edge::EdgeCurve;

    match edge.curve() {
        EdgeCurve::Line => {
            let sp = topo.vertex(edge.start())?.point();
            let ep = topo.vertex(edge.end())?.point();
            Ok((0.0, (ep - sp).length()))
        }
        EdgeCurve::Circle(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_)
        | EdgeCurve::NurbsCurve(_) => crate::authoritative_edge_domain(edge, "edge parameter span"),
    }
}

/// Translation-exact uniform samples of a circle (PERF-D01).
///
/// Same parameters as `sample_uniform` (`t_start + i·step`, the last one
/// pinned to `t_end`), but each point is evaluated as
/// `center + (u·r·cos t + v·r·sin t)`: the offset is formed in the circle's
/// own frame and added to the centre once. A rigid translation of the
/// circle by `δ` therefore moves every sample by exactly `δ` whenever the
/// translated coordinates stay in the same binade (one rounding at world
/// magnitude instead of two), which is what lets a translated face's chart
/// input stay bit-identical.
pub(super) fn sample_circle_uniform(
    circle: &remus_math::curves::Circle3D,
    t_start: f64,
    t_end: f64,
    n: usize,
) -> Vec<Point3> {
    let (u, v, r, c) = (
        circle.u_axis(),
        circle.v_axis(),
        circle.radius(),
        circle.center(),
    );
    uniform_params_like_sampler(t_start, t_end, n)
        .map(|t| c + (u * (r * t.cos()) + v * (r * t.sin())))
        .collect()
}

/// Translation-exact uniform samples of an ellipse (see
/// [`sample_circle_uniform`]).
pub(super) fn sample_ellipse_uniform(
    ellipse: &remus_math::curves::Ellipse3D,
    t_start: f64,
    t_end: f64,
    n: usize,
) -> Vec<Point3> {
    let (u, v, c) = (ellipse.u_axis(), ellipse.v_axis(), ellipse.center());
    let (a, b) = (ellipse.semi_major(), ellipse.semi_minor());
    uniform_params_like_sampler(t_start, t_end, n)
        .map(|t| c + (u * (a * t.cos()) + v * (b * t.sin())))
        .collect()
}

/// The parameters `remus_geometry::sampling::sample_uniform` evaluates.
fn uniform_params_like_sampler(t_start: f64, t_end: f64, n: usize) -> impl Iterator<Item = f64> {
    #[allow(clippy::cast_precision_loss)]
    let step = if n >= 2 {
        (t_end - t_start) / (n - 1) as f64
    } else {
        0.0
    };
    #[allow(clippy::cast_precision_loss)]
    (0..n).map(move |i| {
        if n >= 2 && i == n - 1 {
            t_end
        } else {
            t_start + i as f64 * step
        }
    })
}

/// A NURBS curve re-expressed relative to its first control point, for
/// translation-exact evaluation: `anchor + rel.evaluate(t)` moves by exactly
/// `δ` under a rigid translation of the control points whenever the
/// translated coordinates stay in the same binade (see
/// [`sample_circle_uniform`]).
pub(super) struct AnchoredNurbsCurve {
    anchor: Point3,
    rel: remus_math::nurbs::curve::NurbsCurve,
}

impl AnchoredNurbsCurve {
    pub(super) fn new(curve: &remus_math::nurbs::curve::NurbsCurve) -> Option<Self> {
        let anchor = *curve.control_points().first()?;
        let rel_points: Vec<Point3> = curve
            .control_points()
            .iter()
            .map(|&p| Point3::new(p.x() - anchor.x(), p.y() - anchor.y(), p.z() - anchor.z()))
            .collect();
        let rel = remus_math::nurbs::curve::NurbsCurve::new(
            curve.degree(),
            curve.knots().to_vec(),
            rel_points,
            curve.weights().to_vec(),
        )
        .ok()?;
        Some(Self { anchor, rel })
    }

    pub(super) fn evaluate(&self, t: f64) -> Point3 {
        let r = self.rel.evaluate(t);
        Point3::new(
            self.anchor.x() + r.x(),
            self.anchor.y() + r.y(),
            self.anchor.z() + r.z(),
        )
    }

    /// Project `p` onto the curve in the anchor's frame.
    fn project(&self, p: Point3, tol: f64) -> Option<f64> {
        let rel = Point3::new(
            p.x() - self.anchor.x(),
            p.y() - self.anchor.y(),
            p.z() - self.anchor.z(),
        );
        remus_math::nurbs::projection::project_point_to_curve(&self.rel, rel, tol)
            .ok()
            .map(|proj| proj.parameter)
    }
}

/// Curve parameter of a closed NURBS edge's start vertex, where its
/// full-period sampling begins; projected in the curve's own frame when the
/// anchored copy exists (translation-exact, see [`AnchoredNurbsCurve`]).
fn closed_start_param(
    nurbs: &remus_math::nurbs::curve::NurbsCurve,
    anchored: Option<&AnchoredNurbsCurve>,
    start: Point3,
    fallback: f64,
) -> f64 {
    match anchored {
        Some(a) => a.project(start, 1e-9),
        None => remus_math::nurbs::projection::project_point_to_curve(nurbs, start, 1e-9)
            .ok()
            .map(|proj| proj.parameter),
    }
    .unwrap_or(fallback)
}

/// Sample an edge curve to produce a list of 3D points in stored start-to-end order.
///
/// # Errors
///
/// Returns an error if endpoint lookup fails, stored curved authority is
/// missing or invalid, or the sampling budget is exceeded.
pub(super) fn sample_edge(
    topo: &Topology,
    edge: &remus_topology::edge::Edge,
    deflection: f64,
    angular_tol: f64,
    circle_floor: bool,
) -> Result<Vec<Point3>, crate::OperationsError> {
    sample_edge_with_params(topo, edge, deflection, angular_tol, circle_floor)
        .map(|(points, _)| points)
}

/// Uniform parameter steps over `[t0, t1]` for `n` samples.
///
/// Shared by the sampler and the boundary plan's density-synchronization
/// stages so resampled chains carry the same authoritative parameters the
/// sampler would have produced directly.
pub(super) fn uniform_edge_params(t0: f64, t1: f64, n: usize) -> Vec<Option<f64>> {
    (0..n)
        .map(|i| {
            #[allow(clippy::cast_precision_loss)]
            let f = i as f64 / (n.max(2) - 1) as f64;
            Some((t1 - t0).mul_add(f, t0))
        })
        .collect()
}

/// Sample an edge curve with authoritative parameters retained per sample.
///
/// Returns the 3D polyline (identical to [`sample_edge`]) plus the curve
/// parameter of each sample, parallel to the points. Lines report arclength
/// from the start vertex (`0.0` to length, matching [`edge_param_span`]);
/// curved edges report their stored authoritative parameter, including the
/// wrapped full-period walk of a closed NURBS edge. Endpoint samples carry
/// the domain ends even though their positions are overwritten with the
/// exact vertex positions. The shared-boundary plan (PERF-D03) retains these
/// so every incident face can observe the same subdivision with its curve
/// identity intact; geometrically inserted refinements (seam crossings,
/// contact subdivisions, CDT Steiner points) carry `None`.
///
/// # Errors
///
/// Returns an error under the same conditions as [`sample_edge`].
pub(super) fn sample_edge_with_params(
    topo: &Topology,
    edge: &remus_topology::edge::Edge,
    deflection: f64,
    angular_tol: f64,
    circle_floor: bool,
) -> Result<(Vec<Point3>, Vec<Option<f64>>), crate::OperationsError> {
    use remus_topology::edge::EdgeCurve;

    let n = edge_sample_count(edge, deflection, angular_tol, circle_floor)?;
    enforce_edge_sample_limit(n)?;

    let mut points = match edge.curve() {
        EdgeCurve::Line => {
            vec![
                topo.vertex(edge.start())?.point(),
                topo.vertex(edge.end())?.point(),
            ]
        }
        EdgeCurve::Circle(circle) => {
            let (t_start, t_end) = circle_param_range(edge)?;
            sample_circle_uniform(circle, t_start, t_end, n)
        }
        EdgeCurve::Ellipse(ellipse) => {
            let (t_start, t_end) = edge_param_span(topo, edge)?;
            sample_ellipse_uniform(ellipse, t_start, t_end, n)
        }
        EdgeCurve::Hyperbola(h) => {
            let (t0, t1) = crate::authoritative_edge_domain(edge, "hyperbola sampling")?;
            (0..n)
                .map(|i| {
                    #[allow(clippy::cast_precision_loss)]
                    let f = i as f64 / (n.max(2) - 1) as f64;
                    h.evaluate((t1 - t0).mul_add(f, t0))
                })
                .collect()
        }
        EdgeCurve::Parabola(p) => {
            let (t0, t1) = crate::authoritative_edge_domain(edge, "parabola sampling")?;
            (0..n)
                .map(|i| {
                    #[allow(clippy::cast_precision_loss)]
                    let f = i as f64 / (n.max(2) - 1) as f64;
                    p.evaluate((t1 - t0).mul_add(f, t0))
                })
                .collect()
        }
        EdgeCurve::NurbsCurve(nurbs) => {
            // Endpoint-trimmed convention: a validated sub-span samples only
            // the edge's own piece of the stored curve (already start→end);
            // sampling the full knot domain traces the whole parent section
            // curve and rips a crack along the un-shared part.
            let sp = topo.vertex(edge.start())?.point();
            let (t0, t1) = crate::authoritative_edge_domain(edge, "NURBS sampling")?;
            let (u0, u1) = nurbs.domain();
            let is_subspan = (t0 - u0).abs() > 1e-12 || (t1 - u1).abs() > 1e-12;
            let anchored = AnchoredNurbsCurve::new(nurbs);
            let eval = |t: f64| {
                anchored
                    .as_ref()
                    .map_or_else(|| nurbs.evaluate(t), |a| a.evaluate(t))
            };
            if !is_subspan && edge.is_closed() {
                // A CLOSED NURBS edge still has a start vertex, and the
                // polyline has to begin there (the circle arm's
                // `circle_param_range` rationale). The curve's own parameter
                // origin can sit anywhere on the ring — e.g. a converted
                // rim circle whose NURBS origin is a quarter turn from the
                // seam vertex — and the endpoint overwrite below would then
                // replace the first and last samples with a point far off
                // the sampled arc, folding the ring back across itself.
                // Rotate the sampling to start at the vertex's parameter and
                // walk one full period, wrapping at the knot-domain seam.
                let width = u1 - u0;
                let t_v = closed_start_param(nurbs, anchored.as_ref(), sp, t0);
                #[allow(clippy::cast_precision_loss)]
                (0..n)
                    .map(|i| {
                        let offset = width * (i as f64) / ((n - 1).max(1) as f64);
                        let t = u0 + (t_v - u0 + offset).rem_euclid(width.max(1e-300));
                        eval(t)
                    })
                    .collect()
            } else {
                let mut pts: Vec<Point3> =
                    uniform_params_like_sampler(t0, t1, n).map(eval).collect();
                // Normalize to edge (start→end vertex) order so every
                // consumer's `is_forward` walk holds even for section edges
                // whose stored curve runs end→start. Sub-spans are already
                // endpoint-ordered.
                if !is_subspan && nurbs_runs_end_to_start(topo, edge, nurbs)? {
                    pts.reverse();
                }
                pts
            }
        }
    };

    // Authoritative parameters parallel to the polyline, derived from the
    // same spans the points above were sampled over. Endpoint positions are
    // overwritten with exact vertices below; their parameters stay pinned to
    // the domain ends.
    let mut params: Vec<Option<f64>> = match edge.curve() {
        EdgeCurve::Line => {
            let sp = topo.vertex(edge.start())?.point();
            let ep = topo.vertex(edge.end())?.point();
            let length = (ep - sp).length();
            vec![Some(0.0), Some(length)]
        }
        EdgeCurve::Circle(_) => {
            let (t_start, t_end) = circle_param_range(edge)?;
            uniform_edge_params(t_start, t_end, points.len())
        }
        EdgeCurve::Ellipse(_) => {
            let (t_start, t_end) = edge_param_span(topo, edge)?;
            uniform_edge_params(t_start, t_end, points.len())
        }
        EdgeCurve::Hyperbola(_) => {
            let (t0, t1) = crate::authoritative_edge_domain(edge, "hyperbola sampling")?;
            uniform_edge_params(t0, t1, points.len())
        }
        EdgeCurve::Parabola(_) => {
            let (t0, t1) = crate::authoritative_edge_domain(edge, "parabola sampling")?;
            uniform_edge_params(t0, t1, points.len())
        }
        EdgeCurve::NurbsCurve(nurbs) => {
            let (t0, t1) = crate::authoritative_edge_domain(edge, "NURBS sampling")?;
            let (u0, u1) = nurbs.domain();
            let is_subspan = (t0 - u0).abs() > 1e-12 || (t1 - u1).abs() > 1e-12;
            if !is_subspan && edge.is_closed() {
                let sp = topo.vertex(edge.start())?.point();
                let width = u1 - u0;
                let t_v =
                    closed_start_param(nurbs, AnchoredNurbsCurve::new(nurbs).as_ref(), sp, t0);
                (0..points.len())
                    .map(|i| {
                        #[allow(clippy::cast_precision_loss)]
                        let offset = width * (i as f64) / ((points.len() - 1).max(1) as f64);
                        Some(u0 + (t_v - u0 + offset).rem_euclid(width.max(1e-300)))
                    })
                    .collect()
            } else {
                let mut ps = uniform_edge_params(t0, t1, points.len());
                if !is_subspan && nurbs_runs_end_to_start(topo, edge, nurbs).unwrap_or(false) {
                    ps.reverse();
                }
                ps
            }
        }
    };

    if !matches!(edge.curve(), EdgeCurve::Line) {
        if let Some(first) = points.first_mut() {
            *first = topo.vertex(edge.start())?.point();
        }
        if let Some(last) = points.last_mut() {
            *last = topo.vertex(edge.end())?.point();
        }
    }
    debug_assert_eq!(points.len(), params.len());
    params.truncate(points.len());
    while params.len() < points.len() {
        params.push(None);
    }

    Ok((points, params))
}

/// Whether an open NURBS edge's stored curve runs from the edge's END vertex
/// back to its START vertex. GFA section edges can store traversal-order
/// vertices over an unreversed curve, so a sampler that walks the knot domain
/// trusting `oe.is_forward()` alone folds the boundary polyline back on
/// itself (a double-covered strip along the shared section curve).
pub(super) fn nurbs_runs_end_to_start(
    topo: &Topology,
    edge: &remus_topology::edge::Edge,
    nurbs: &remus_math::nurbs::curve::NurbsCurve,
) -> Result<bool, crate::OperationsError> {
    if edge.start() == edge.end() {
        return Ok(false);
    }
    let s = topo.vertex(edge.start())?.point();
    let e = topo.vertex(edge.end())?.point();
    let (u0, u1) = nurbs.domain();
    let p0 = nurbs.evaluate(u0);
    let p1 = nurbs.evaluate(u1);
    let aligned = (p0 - s).length() + (p1 - e).length();
    let flipped = (p0 - e).length() + (p1 - s).length();
    Ok(flipped < aligned)
}

/// Sample a wire into a list of 3D positions, skipping consecutive duplicates.
pub(super) fn sample_wire_positions(
    topo: &Topology,
    wire: &remus_topology::wire::Wire,
    tol: f64,
    deflection: f64,
    angular_tol: f64,
) -> Result<Vec<Point3>, crate::OperationsError> {
    use remus_topology::edge::EdgeCurve;

    let mut positions = Vec::new();

    // Half-open in TRAVERSAL order: emit the vertex the wire arrives at and
    // stop one step short of the vertex it leaves at, which the next edge
    // supplies. `t_for_index` maps 0 -> the curve's natural start and
    // `n_samples` -> its natural end, so a forward edge walks `0..n` and a
    // REVERSED one must walk `n..=1` — not `(0..n).rev()`, which starts one
    // step inside the arc and drops the vertex the wire arrives at. That
    // dropped vertex leaves a chord running from the previous edge's last
    // sample straight into the arc's interior; where the previous edge is a
    // long straight side, the chord slices a large triangle off the face
    // (a 74 mm run into an r = 3 arc loses 5.4 mm² — see the volume tests).
    let sample_curve_into = |evaluate: &dyn Fn(f64) -> Point3,
                             t_for_index: &dyn Fn(usize) -> f64,
                             n_samples: usize,
                             forward: bool,
                             positions: &mut Vec<Point3>| {
        let indices: Box<dyn Iterator<Item = usize>> = if forward {
            Box::new(0..n_samples)
        } else {
            // Reversed traversal walks t_end -> t_start; the [traversal
            // start, traversal end) convention therefore needs indices
            // n..=1, not (0..n).rev(): excluding t_end here dropped the
            // junction vertex with the PREVIOUS edge (nobody else supplies
            // it), and the CDT outline then shortcut the polygon corner
            // with a chord whose area bite scales with the neighbour
            // edge's length. t_start is excluded instead - the next edge
            // supplies it, same as the forward case.
            Box::new((1..=n_samples).rev())
        };
        for i in indices {
            #[allow(clippy::cast_precision_loss)]
            let t = t_for_index(i);
            let pt = evaluate(t);
            if positions
                .last()
                .is_none_or(|p: &Point3| (*p - pt).length() > tol)
            {
                positions.push(pt);
            }
        }
    };

    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        match edge.curve() {
            EdgeCurve::Circle(circle) => {
                let (t_start, t_end) = circle_param_range(edge)?;
                let arc_range = (t_end - t_start).abs();
                let n_samples = segments_for_chord_deviation_a(
                    circle.radius(),
                    arc_range,
                    deflection,
                    angular_tol,
                    false,
                );
                enforce_edge_sample_limit(n_samples)?;
                #[allow(clippy::cast_precision_loss)]
                sample_curve_into(
                    &|t| circle.evaluate(t),
                    &|i| t_start + (t_end - t_start) * (i as f64) / (n_samples as f64),
                    n_samples,
                    oe.is_forward(),
                    &mut positions,
                );
            }
            EdgeCurve::Ellipse(ellipse) => {
                let (t_start, t_end) =
                    crate::authoritative_edge_domain(edge, "ellipse wire sampling")?;
                let arc_range = t_end - t_start;
                // Largest radius of curvature (a^2/b) governs uniform-parameter
                // sampling density; see edge_sample_count for the rationale.
                let max_curv_radius =
                    ellipse.semi_major() * ellipse.semi_major() / ellipse.semi_minor();
                let n_samples = segments_for_chord_deviation_a(
                    max_curv_radius,
                    arc_range,
                    deflection,
                    angular_tol,
                    true,
                );
                enforce_edge_sample_limit(n_samples)?;
                #[allow(clippy::cast_precision_loss)]
                sample_curve_into(
                    &|t| ellipse.evaluate(t),
                    &|i| t_start + (t_end - t_start) * (i as f64) / (n_samples as f64),
                    n_samples,
                    oe.is_forward(),
                    &mut positions,
                );
            }
            // Unbounded branches: `project` inverts the parameterization
            // exactly, so the arc is the straight parameter interval between
            // the two vertices. Density comes from the tightest osculating
            // circle on that span (see `open_conic_segments`), never a chord.
            EdgeCurve::Hyperbola(h) => {
                let (t0, t1) = crate::authoritative_edge_domain(edge, "hyperbola wire sampling")?;
                let n_samples = open_conic_segments(
                    h.min_curvature_radius(t0, t1),
                    h.arc_length(t0, t1),
                    deflection,
                    angular_tol,
                );
                enforce_edge_sample_limit(n_samples)?;
                #[allow(clippy::cast_precision_loss)]
                sample_curve_into(
                    &|t| h.evaluate(t),
                    &|i| t0 + (t1 - t0) * (i as f64) / (n_samples as f64),
                    n_samples,
                    oe.is_forward(),
                    &mut positions,
                );
            }
            EdgeCurve::Parabola(p) => {
                let (t0, t1) = crate::authoritative_edge_domain(edge, "parabola wire sampling")?;
                let n_samples = open_conic_segments(
                    p.min_curvature_radius(t0, t1),
                    p.arc_length(t0, t1),
                    deflection,
                    angular_tol,
                );
                enforce_edge_sample_limit(n_samples)?;
                #[allow(clippy::cast_precision_loss)]
                sample_curve_into(
                    &|t| p.evaluate(t),
                    &|i| t0 + (t1 - t0) * (i as f64) / (n_samples as f64),
                    n_samples,
                    oe.is_forward(),
                    &mut positions,
                );
            }
            EdgeCurve::NurbsCurve(nurbs) => {
                // Endpoint-trimmed convention: sample only the edge's own
                // sub-span of the stored curve (see `sample_edge`).
                let (u0, u1) = crate::authoritative_edge_domain(edge, "NURBS wire sampling")?;
                let full = nurbs.domain();
                let is_subspan = (u0 - full.0).abs() > 1e-12 || (u1 - full.1).abs() > 1e-12;
                let n_spans = nurbs
                    .control_points()
                    .len()
                    .saturating_sub(nurbs.degree())
                    .max(1);
                let coarse_n = (n_spans * 4).clamp(8, 128);
                let max_dev = measure_max_chord_deviation(nurbs, u0, u1, coarse_n);
                let max_turn = measure_max_segment_turn(nurbs, u0, u1, coarse_n);
                let sag_ok = max_dev <= deflection;
                let turn_ok = angular_tol <= 0.0 || max_turn <= angular_tol * 0.5;
                #[allow(clippy::cast_sign_loss)]
                let n_samples = if sag_ok && turn_ok {
                    coarse_n
                } else {
                    let sag_n = if sag_ok {
                        coarse_n
                    } else {
                        ((coarse_n as f64) * (max_dev / deflection).sqrt()).ceil() as usize
                    };
                    let turn_n = if turn_ok {
                        coarse_n
                    } else {
                        ((coarse_n as f64) * (max_turn / (angular_tol * 0.5))).ceil() as usize
                    };
                    sag_n.max(turn_n)
                }
                .clamp(8, 4096);
                let forward = if is_subspan {
                    // Sub-spans are already endpoint-ordered start→end.
                    oe.is_forward()
                } else {
                    oe.is_forward() != nurbs_runs_end_to_start(topo, edge, nurbs)?
                };
                #[allow(clippy::cast_precision_loss)]
                sample_curve_into(
                    &|t| nurbs.evaluate(t),
                    &|i| u0 + (u1 - u0) * (i as f64) / (n_samples as f64),
                    n_samples,
                    forward,
                    &mut positions,
                );
            }
            EdgeCurve::Line => {
                let vid = if oe.is_forward() {
                    edge.start()
                } else {
                    edge.end()
                };
                let pt = topo.vertex(vid)?.point();
                if positions
                    .last()
                    .is_none_or(|p: &Point3| (*p - pt).length() > tol)
                {
                    positions.push(pt);
                }
            }
        }
    }

    if positions.len() > 2
        && let (Some(first), Some(last)) = (positions.first(), positions.last())
        && (*last - *first).length() < tol
    {
        positions.pop();
    }

    Ok(positions)
}
