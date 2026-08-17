//! Wire and surface sampling functions for UV space.

use remus_math::vec::Point2;

use super::super::split_types::OrientedPCurveEdge;

/// Sample UV points along a wire loop, interpolating along curved edges.
///
/// For line edges, uses only the start point. For curved edges (Circle,
/// Ellipse, NurbsCurve), samples N intermediate points to approximate the
/// true curve shape in UV. This is critical for signed area computation
/// and point-in-polygon tests on loops with curved edges.
pub(super) fn sample_wire_loop_uv(wire: &[OrientedPCurveEdge]) -> Vec<Point2> {
    sample_wire_loop_uv_periodic(wire, None, None)
}

/// Sample a plane-face wire loop by evaluating each edge's 3D curve and
/// projecting through the face's `PlaneFrame` — never the stored pcurve.
///
/// Pcurves reach a wire under two orientation conventions (sections carry the
/// curve's natural parameterization plus a traversal flag; boundary edges are
/// fit in traversal order but keep the topology orientation flag), so a
/// pcurve-driven sampler can trace a reversed-boundary arc backwards and fold
/// the loop polygon into a self-crossing zig-zag. The 3D curve with the
/// traversal endpoints is orientation-unambiguous. Mirrors the arc-true hole
/// polygons in `find_point_outside_holes`.
pub(super) fn sample_wire_loop_uv_via_frame(
    wire: &[OrientedPCurveEdge],
    frame: &super::super::plane_frame::PlaneFrame,
) -> Vec<Point2> {
    use remus_topology::edge::EdgeCurve;
    const CURVE_SAMPLES: usize = 8;

    let mut pts = Vec::with_capacity(wire.len() * CURVE_SAMPLES);
    for e in wire {
        pts.push(e.start_uv);
        if matches!(e.curve_3d, EdgeCurve::Line) {
            continue;
        }
        // Sample in the edge's NATIVE orientation: `domain_with_endpoints`
        // always takes the positive parametric span, so swapped endpoints
        // would select the COMPLEMENTARY arc. Restore wire order afterwards.
        let (s3, e3) = if e.forward {
            (e.start_3d, e.end_3d)
        } else {
            (e.end_3d, e.start_3d)
        };
        let (t0, t1) = e.native_domain();
        #[allow(clippy::cast_precision_loss)]
        let mut samples: Vec<Point2> = (1..CURVE_SAMPLES)
            .map(|k| {
                let t = (t1 - t0).mul_add(k as f64 / CURVE_SAMPLES as f64, t0);
                frame.project(e.curve_3d.evaluate_with_endpoints(t, s3, e3))
            })
            .collect();
        if !e.forward {
            samples.reverse();
        }
        pts.extend(samples);
    }
    pts
}

/// Sample UV points along a wire loop with optional periodic unwrapping.
///
/// When `u_period`/`v_period` is set, unwraps the UV path so it is continuous
/// (no jumps of ~2pi between edges connected via periodic quantization).
///
/// Unwrapping is done PER EDGE, never across the whole flattened point list.
/// An edge's own stored span is authoritative: a boundary arc on a cylinder is
/// a Line2D pcurve whose two endpoints already carry the true parametric
/// delta. Re-deriving that delta by rounding `du / period` is a coin flip for a
/// SEMICIRCLE, whose step is exactly half a period — and `f64::round` breaks
/// the tie away from zero, so it always folds a genuine +pi to -pi (or the
/// reverse) regardless of which way the arc actually runs. A cylinder rim built
/// from two semicircular edges hits that tie exactly, not approximately, so no
/// epsilon avoids it: the 3/4 band left by a quarter-overlap cut then folds
/// onto the quarter's u-range, and its interior sample lands in the
/// neighbouring sub-face — misclassifying the wall and dropping it from the
/// result.
///
/// Between edges the step is unambiguous (consecutive edges share a vertex, so
/// the raw delta is a whole number of periods), which is exactly where the
/// period-copy reconciliation belongs.
pub(super) fn sample_wire_loop_uv_periodic(
    wire: &[OrientedPCurveEdge],
    u_period: Option<f64>,
    v_period: Option<f64>,
) -> Vec<Point2> {
    use remus_math::curves2d::Curve2D;
    const CURVE_SAMPLES: usize = 8;

    let mut pts: Vec<Point2> = Vec::new();
    let has_period = u_period.is_some() || v_period.is_some();
    for edge in wire {
        let mut group: Vec<Point2> = Vec::new();
        match &edge.pcurve {
            Curve2D::Line(_) => {
                // For periodic surfaces, push both start and end to enable
                // proper unwrapping across periodic jumps at seam vertices.
                group.push(edge.start_uv);
                if has_period {
                    group.push(edge.end_uv);
                }
            }
            Curve2D::Nurbs(nurbs) => {
                let knots = nurbs.knots();
                if knots.len() >= 2 {
                    let t0 = knots[0];
                    let tn = knots[knots.len() - 1];
                    // For reverse edges, the pcurve was computed for the forward
                    // direction. Evaluate from tn->t0 to trace the reverse path.
                    #[allow(clippy::cast_precision_loss)]
                    for i in 0..CURVE_SAMPLES {
                        let frac = i as f64 / CURVE_SAMPLES as f64;
                        let t = if edge.forward {
                            t0 + (tn - t0) * frac
                        } else {
                            tn - (tn - t0) * frac
                        };
                        group.push(nurbs.evaluate(t));
                    }
                } else {
                    group.push(edge.start_uv);
                }
            }
            Curve2D::Circle(_) | Curve2D::Ellipse(_) => {
                // Circle2D/Ellipse2D pcurves: interpolate between start_uv
                // and end_uv. This is approximate (chord, not arc) but these
                // pcurve types are rare in the pipeline -- section edges use
                // NURBS and boundary edges use Line2D.
                #[allow(clippy::cast_precision_loss)]
                for i in 0..CURVE_SAMPLES {
                    let t = i as f64 / CURVE_SAMPLES as f64;
                    group.push(Point2::new(
                        edge.start_uv.x() + (edge.end_uv.x() - edge.start_uv.x()) * t,
                        edge.start_uv.y() + (edge.end_uv.y() - edge.start_uv.y()) * t,
                    ));
                }
            }
        }

        if group.is_empty() {
            continue;
        }
        if !has_period {
            pts.append(&mut group);
            continue;
        }

        // Densely-sampled groups come from evaluating one pcurve, so successive
        // samples are a small fraction of a period apart and rounding is
        // unambiguous — unwrap them so a pcurve stored across the seam is made
        // continuous. A two-point group is the edge's own stored span and is
        // left exactly as recorded.
        if group.len() > 2 {
            super::super::pcurve_compute::unwrap_periodic_params_pub(
                &mut group, u_period, v_period,
            );
        }

        // Shift the whole group onto the period copy nearest the previous
        // edge's end, preserving every delta inside the group.
        if let Some(prev) = pts.last().copied() {
            let du = u_period.map_or(0.0, |p| -p * ((group[0].x() - prev.x()) / p).round());
            let dv = v_period.map_or(0.0, |p| -p * ((group[0].y() - prev.y()) / p).round());
            if du != 0.0 || dv != 0.0 {
                for g in &mut group {
                    *g = Point2::new(g.x() + du, g.y() + dv);
                }
            }
        }
        pts.append(&mut group);
    }

    pts
}

/// Normalize an angle into the `[0, 1]` parameter range of an edge span.
///
/// `t0` is the start angle, `span = t1 - t0` is the signed angular range.
/// Returns `(angle - t0) / span`, wrapping by 2pi to stay within the arc.
pub(super) fn normalize_angle_in_span(angle: f64, t0: f64, span: f64) -> f64 {
    use std::f64::consts::TAU;
    let mut delta = angle - t0;
    if span > 0.0 {
        // CCW arc: delta should be in [0, span].
        // At most 2 wraps needed (angle is in (-pi, pi]).
        for _ in 0..3 {
            if delta >= -1e-10 {
                break;
            }
            delta += TAU;
        }
        for _ in 0..3 {
            if delta <= span + 1e-10 {
                break;
            }
            delta -= TAU;
        }
    } else {
        // CW arc: delta should be in [span, 0].
        for _ in 0..3 {
            if delta <= 1e-10 {
                break;
            }
            delta -= TAU;
        }
        for _ in 0..3 {
            if delta >= span - 1e-10 {
                break;
            }
            delta += TAU;
        }
    }
    delta / span
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;
    use remus_math::curves2d::{Curve2D, Line2D};
    use remus_math::vec::{Point3, Vec2};
    use remus_topology::edge::EdgeCurve;
    use std::f64::consts::{PI, TAU};

    /// A UV-space edge is all `sample_wire_loop_uv_periodic` reads; the 3D
    /// fields only have to be present.
    fn uv_edge(su: f64, sv: f64, eu: f64, ev: f64) -> OrientedPCurveEdge {
        let start_uv = Point2::new(su, sv);
        let end_uv = Point2::new(eu, ev);
        let dir = Vec2::new(eu - su, ev - sv);
        OrientedPCurveEdge {
            curve_3d: EdgeCurve::Line,
            trim: None,
            pcurve: Curve2D::Line(
                Line2D::new(start_uv, dir)
                    .unwrap_or_else(|_| Line2D::new(start_uv, Vec2::new(1.0, 0.0)).unwrap()),
            ),
            start_uv,
            end_uv,
            start_3d: Point3::new(0.0, 0.0, 0.0),
            end_3d: Point3::new(0.0, 0.0, 0.0),
            forward: true,
            source_edge_idx: None,
            pave_block_id: None,
            source_topo_edge: None,
        }
    }

    /// The 3/4 lateral band left when a box takes the first quadrant out of a
    /// cylinder whose seam sits exactly on the cut plane. Both bounding rims
    /// are built from two SEMICIRCLES, so two of the wire's steps are exactly
    /// half a period — the tie that `f64::round` breaks away from zero.
    ///
    /// The band must come back spanning its true 3pi/2 of u. Folding either
    /// semicircle the wrong way collapses it onto the quarter's u-range, and
    /// the interior sample then lands in the neighbouring sub-face.
    #[test]
    fn a_semicircle_step_keeps_its_own_direction() {
        let (q1, q3) = (PI / 2.0, 3.0 * PI / 2.0);
        // Right side up; top rim right-to-left as a semicircle then a quarter;
        // left side down; bottom rim back left-to-right, stored one period up.
        let wire = vec![
            uv_edge(q3, 0.0, q3, 2.0),
            uv_edge(q3, 2.0, q1, 2.0),  // -pi, the ambiguous step
            uv_edge(q1, 2.0, 0.0, 2.0), // -pi/2
            uv_edge(0.0, 2.0, 0.0, 0.0),
            uv_edge(TAU, 0.0, TAU + q1, 0.0), // +pi/2, a period copy up
            uv_edge(TAU + q1, 0.0, TAU + q3, 0.0), // +pi, the ambiguous step
        ];

        let pts = sample_wire_loop_uv_periodic(&wire, Some(TAU), None);

        let u_min = pts.iter().map(|p| p.x()).fold(f64::INFINITY, f64::min);
        let u_max = pts.iter().map(|p| p.x()).fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (u_max - u_min - q3).abs() < 1e-9,
            "3/4 band must span 3pi/2 of u, got {:.6} over [{u_min:.4}, {u_max:.4}]",
            u_max - u_min
        );

        // The centroid is the interior sample the classifier consumes: it has
        // to land in the band's own u-range, not the quarter's.
        #[allow(clippy::cast_precision_loss)]
        let cu = pts.iter().map(|p| p.x()).sum::<f64>() / pts.len() as f64;
        assert!(
            cu > u_min + 1e-9 && cu < u_max - 1e-9,
            "interior u {cu:.6} escaped the band [{u_min:.4}, {u_max:.4}]"
        );
    }

    /// The quarter piece has no half-period step, so per-edge unwrapping must
    /// leave it exactly where whole-list unwrapping did.
    #[test]
    fn a_quarter_band_is_unchanged() {
        let q3 = 3.0 * PI / 2.0;
        let wire = vec![
            uv_edge(q3, 2.0, q3, 0.0),
            uv_edge(q3, 0.0, TAU, 0.0),
            uv_edge(0.0, 0.0, 0.0, 2.0),
            uv_edge(0.0, 2.0, -PI / 2.0, 2.0),
        ];

        let pts = sample_wire_loop_uv_periodic(&wire, Some(TAU), None);

        let u_min = pts.iter().map(|p| p.x()).fold(f64::INFINITY, f64::min);
        let u_max = pts.iter().map(|p| p.x()).fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (u_min - q3).abs() < 1e-9 && (u_max - TAU).abs() < 1e-9,
            "quarter must stay on [3pi/2, 2pi], got [{u_min:.4}, {u_max:.4}]"
        );
    }
}
