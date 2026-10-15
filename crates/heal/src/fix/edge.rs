//! Edge fixing — vertex-curve alignment, degenerate removal, same-parameter.

use remus_math::curves2d::{Curve2D, NurbsCurve2D};
use remus_math::frame::Frame3;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::pcurve::PCurve;

use super::FixResult;
use super::config::{FixConfig, FixMode};
use crate::HealError;
use crate::context::HealContext;
use crate::status::Status;

/// Number of sample points for PCurve deviation analysis.
const SAME_PARAM_SAMPLES: usize = 20;

/// Fix a single edge: vertex tolerance, degenerate removal, `SameParameter` stub.
///
/// # Fixes applied
///
/// 1. **Vertex tolerance** (`fix_vertex_tolerance`): if a vertex position
///    deviates from the 3D curve endpoint by more than tolerance, a warning
///    is logged.
/// 2. **Degenerate edge** (`fix_degenerate_edges`): if the edge is closed
///    (`start == end`) and the curve length is approximately zero, the edge
///    is marked for removal via [`ReShape`](crate::reshape::ReShape).
/// 3. **SameParameter** (`fix_same_parameter`): logs a TODO warning.
///    For face-aware same-parameter fixing, use [`fix_same_parameter_on_face`].
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups fail.
pub fn fix_edge(
    topo: &Topology,
    edge_id: EdgeId,
    ctx: &mut HealContext,
    config: &FixConfig,
) -> Result<FixResult, HealError> {
    let mut result = FixResult::ok();

    if config.fix_vertex_tolerance != FixMode::Off {
        let r = fix_vertex_tolerance(topo, edge_id, ctx, config)?;
        result.merge(&r);
    }

    if config.fix_degenerate_edges != FixMode::Off {
        let r = fix_degenerate(topo, edge_id, ctx, config)?;
        result.merge(&r);
    }

    if config.fix_same_parameter != FixMode::Off {
        let r = fix_same_parameter_stub(ctx, config);
        result.merge(&r);
    }

    Ok(result)
}

/// Fix SameParameter for an edge on a specific face.
///
/// Ensures the edge's PCurve on the given face accurately represents the
/// 3D curve projected onto the face surface.
///
/// # Algorithm
///
/// 1. Check if the edge has a PCurve on the given face (via pcurve registry).
/// 2. If a PCurve exists, sample both the 3D curve and the PCurve at
///    `SAME_PARAM_SAMPLES` points and compute the maximum deviation
///    between the surface evaluation at the PCurve's UV coordinates and
///    the 3D curve position.
/// 3. If deviation > tolerance (or no PCurve exists), rebuild the PCurve
///    via [`project_edge_to_pcurve`](crate::construct::project_curve::project_edge_to_pcurve).
/// 4. Register the new PCurve in the topology's pcurve registry.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups fail or PCurve projection fails.
#[allow(clippy::too_many_lines)]
pub fn fix_same_parameter_on_face(
    topo: &mut Topology,
    edge_id: EdgeId,
    face_id: FaceId,
    ctx: &mut HealContext,
    config: &FixConfig,
) -> Result<FixResult, HealError> {
    // How does this face use the edge? A SEAM edge appears in the face's wires
    // twice — once forward, once reversed — and each use carries its own
    // p-curve, so `(edge, face)` does not name one of them. The registry is
    // right to refuse that (RFC 0002 fails closed rather than picking a side),
    // but this function used to let the refusal escape, and `fix_shape` aborts
    // on the first error. Every cylinder, cone, sphere, revolve and most
    // imported STEP carries a seam, so the whole heal pipeline was unusable on
    // them: a plain cylinder and a box with a through-hole both came back
    // `seam_pcurve_ambiguous`.
    //
    // Declining the seam use is the honest repair. Rebuilding one needs to know
    // WHICH side of the seam it is, and `project_edge_to_pcurve` projects the
    // 3D curve without that information — it cannot tell u = 0 from u = 2*pi.
    // Fabricating one would be worse than leaving it. The rest of the face, and
    // the rest of the pipeline, now proceed.
    let senses = face_edge_senses(topo, face_id, edge_id)?;
    let forward = match senses.as_slice() {
        [] => return Ok(FixResult::ok()),
        [only] => *only,
        _ => {
            let needs_fix = senses.iter().any(|&sense| {
                topo.pcurve_oriented(edge_id, face_id, sense)
                    .is_none_or(|_| {
                        compute_pcurve_deviation(topo, edge_id, face_id, sense)
                            .map_or(true, |deviation| deviation > ctx.tolerance.linear)
                    })
            });
            if !config.fix_same_parameter.should_fix(needs_fix) || !needs_fix {
                return Ok(FixResult::ok());
            }
            ctx.info(format!(
                "Edge {edge_id:?} on Face {face_id:?}: seam edge (used in both senses);                  SameParameter needs a side the projection cannot infer, skipping",
            ));
            return Ok(FixResult::refused(
                Status::FAIL1,
                super::RepairRefusalKind::AmbiguousSeamPcurve,
                senses.len(),
            ));
        }
    };

    let has_pcurve = topo.pcurve_oriented(edge_id, face_id, forward).is_some();

    if has_pcurve {
        let max_dev = compute_pcurve_deviation(topo, edge_id, face_id, forward)?;
        let tol = ctx.tolerance.linear;
        let needs_fix = max_dev > tol;

        if !config.fix_same_parameter.should_fix(needs_fix) {
            return Ok(FixResult::ok());
        }

        if !needs_fix {
            return Ok(FixResult::ok());
        }

        ctx.info(format!(
            "Edge {edge_id:?} on Face {face_id:?}: PCurve deviation {max_dev:.2e} exceeds tolerance {tol:.2e}, rebuilding",
        ));
    } else {
        // Absence is not itself a SameParameter defect: many valid native
        // solids intentionally carry no pcurves. Auto mode repairs a measured
        // deviation only; explicit On mode requests construction of a missing
        // representation and the high-level boundary verifies the result.
        if config.fix_same_parameter != FixMode::On {
            return Ok(FixResult::ok());
        }

        ctx.info(format!(
            "Edge {edge_id:?} on Face {face_id:?}: no PCurve found, creating via projection",
        ));
    }

    // A plane carries a line or circle exactly in its canonical chart, and
    // strict validation certifies only those analytic forms there: a NURBS
    // fit of either would read as repaired here and fail that proof.
    let plane = if let FaceSurface::Plane { normal, d } = topo.face(face_id)?.surface() {
        Some((*normal, *d))
    } else {
        None
    };
    let exact = match plane {
        Some((normal, d)) => exact_plane_pcurve(topo, edge_id, forward, normal, d)?,
        None => None,
    };
    let pcurve = match exact {
        Some(pcurve) => pcurve,
        None => fitted_pcurve(topo, edge_id, face_id, forward, ctx)?,
    };
    let original = topo.pcurve_oriented(edge_id, face_id, forward).cloned();
    topo.set_pcurve_oriented(edge_id, face_id, forward, pcurve)?;

    // A plane's chart is exact, so a deviation left after the rebuild is the
    // edge's own distance from the plane (or a fit that cannot follow it):
    // no pcurve repairs that, so keep the caller's and say so.
    if plane.is_some() {
        let after = compute_pcurve_deviation(topo, edge_id, face_id, forward)?;
        if after.is_nan() || after > ctx.tolerance.linear {
            match original {
                Some(pcurve) => topo.set_pcurve_oriented(edge_id, face_id, forward, pcurve)?,
                None => {
                    topo.remove_pcurve_oriented(edge_id, face_id, forward)?;
                }
            }
            ctx.info(format!(
                "Edge {edge_id:?} on Face {face_id:?}: rebuilt PCurve still deviates {after:.2e} from the plane, keeping the original",
            ));
            return Ok(FixResult::refused(
                Status::FAIL1,
                super::RepairRefusalKind::PcurveDeviationRemains,
                1,
            ));
        }
    }

    Ok(FixResult::changed(
        Status::DONE3,
        super::RepairActionKind::PcurveRebuilt,
        1,
    ))
}

/// The senses in which `face` uses `edge`, deduplicated.
///
/// Empty when the face does not use the edge at all; one entry for an ordinary
/// edge; two for a seam, which is the case every caller here has to decide
/// about explicitly rather than let the p-curve registry refuse.
fn face_edge_senses(
    topo: &Topology,
    face_id: FaceId,
    edge_id: EdgeId,
) -> Result<Vec<bool>, HealError> {
    let face = topo.face(face_id)?;
    let mut senses: Vec<bool> = Vec::new();
    for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oe in topo.wire(wid)?.edges() {
            if oe.edge() == edge_id && !senses.contains(&oe.is_forward()) {
                senses.push(oe.is_forward());
            }
        }
    }
    Ok(senses)
}

/// Check vertex-curve deviation and warn if it exceeds tolerance.
fn fix_vertex_tolerance(
    topo: &Topology,
    edge_id: EdgeId,
    ctx: &mut HealContext,
    config: &FixConfig,
) -> Result<FixResult, HealError> {
    let (start_dev, end_dev) = crate::analysis::edge::vertex_curve_deviation(topo, edge_id)?;

    let tol = ctx.tolerance.linear;
    let has_issue = start_dev > tol || end_dev > tol;

    if !config.fix_vertex_tolerance.should_fix(has_issue) {
        return Ok(FixResult::ok());
    }

    if start_dev > tol {
        ctx.warn(format!(
            "Edge {edge_id:?}: start vertex deviates from curve by {start_dev:.2e} (tol={tol:.2e})",
        ));
    }
    if end_dev > tol {
        ctx.warn(format!(
            "Edge {edge_id:?}: end vertex deviates from curve by {end_dev:.2e} (tol={tol:.2e})",
        ));
    }

    Ok(FixResult::refused(
        Status::FAIL2,
        super::RepairRefusalKind::VertexToleranceRepairUnsupported,
        usize::from(start_dev > tol) + usize::from(end_dev > tol),
    ))
}

/// Detect and remove degenerate edges (closed + zero-length curve).
fn fix_degenerate(
    topo: &Topology,
    edge_id: EdgeId,
    ctx: &mut HealContext,
    config: &FixConfig,
) -> Result<FixResult, HealError> {
    let analysis = crate::analysis::edge::analyze_edge(topo, edge_id, &ctx.tolerance)?;

    if !config
        .fix_degenerate_edges
        .should_fix(analysis.is_degenerate)
    {
        return Ok(FixResult::ok());
    }

    ctx.info(format!(
        "Edge {edge_id:?}: degenerate (closed, length={:.2e}), marking for removal",
        analysis.curve_length_approx
    ));
    ctx.reshape.remove_edge(edge_id);

    Ok(FixResult::changed(
        Status::DONE2,
        super::RepairActionKind::DegenerateEdgeRemoved,
        1,
    ))
}

/// Stub for SameParameter when no face context is available.
fn fix_same_parameter_stub(ctx: &mut HealContext, config: &FixConfig) -> FixResult {
    // SameParameter requires a face context to compute PCurve deviation.
    // Without a face_id, we can only log a warning.
    if !config.fix_same_parameter.should_fix(false) {
        return FixResult::ok();
    }

    ctx.warn(
        "SameParameter fix: requires face context, use fix_same_parameter_on_face()".to_string(),
    );

    FixResult::refused(
        Status::FAIL3,
        super::RepairRefusalKind::SameParameterNeedsFace,
        1,
    )
}

/// Compute the maximum deviation between a 3D edge curve and its PCurve
/// on a given face.
///
/// Samples both curves at [`SAME_PARAM_SAMPLES`] points and returns the
/// maximum 3D distance between the edge curve point and the surface point
/// evaluated at the PCurve's UV coordinates.
fn compute_pcurve_deviation(
    topo: &Topology,
    edge_id: EdgeId,
    face_id: FaceId,
    forward: bool,
) -> Result<f64, HealError> {
    let pcurve = topo
        .pcurve_oriented(edge_id, face_id, forward)
        .ok_or_else(|| {
            HealError::FixFailed(format!(
                "no PCurve found for edge {edge_id:?} on face {face_id:?}"
            ))
        })?;

    let edge = topo.edge(edge_id)?;
    let start_pos = topo.vertex(edge.start())?.point();
    let end_pos = topo.vertex(edge.end())?.point();
    let curve = edge.curve();

    let face = topo.face(face_id)?;
    let surface = face.surface();
    // A plane carries no chart of its own: its pcurves live in the canonical
    // frame the projector writes and the STEP writer and strict validator
    // read. `FaceSurface::evaluate` has no answer for a plane, and skipping
    // those samples read every planar pcurve as deviation 0.
    let plane_frame = match surface {
        FaceSurface::Plane { normal, d } => Some(canonical_plane_frame(*normal, *d)?),
        FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_)
        | FaceSurface::Nurbs(_) => None,
    };

    let (t0_3d, t1_3d) = edge
        .strict_domain()
        .map_err(crate::error::fix_edge_domain)?;
    let t0_pc = pcurve.t_start();
    let t1_pc = pcurve.t_end();

    let mut max_dev = 0.0_f64;

    for i in 0..=SAME_PARAM_SAMPLES {
        #[allow(clippy::cast_precision_loss)]
        let frac = i as f64 / SAME_PARAM_SAMPLES as f64;

        // The pcurve traces the USE: a reversed use starts at the edge's end.
        let t_3d = if forward {
            t0_3d + (t1_3d - t0_3d) * frac
        } else {
            t1_3d - (t1_3d - t0_3d) * frac
        };
        let pt_3d = curve.evaluate_with_endpoints(t_3d, start_pos, end_pos);

        let t_pc = t0_pc + (t1_pc - t0_pc) * frac;
        let uv = pcurve.evaluate(t_pc);

        let pt_surf = match &plane_frame {
            Some(frame) => frame.origin + frame.x * uv.x() + frame.y * uv.y(),
            None => surface.evaluate(uv.x(), uv.y()).ok_or_else(|| {
                HealError::AnalysisFailed(format!(
                    "face {face_id:?} cannot evaluate its {} surface",
                    surface.type_tag()
                ))
            })?,
        };
        let dev = (pt_3d - pt_surf).length();
        // NaN must not compare its way out of the maximum.
        if dev.is_nan() {
            return Ok(f64::INFINITY);
        }
        max_dev = max_dev.max(dev);
    }

    Ok(max_dev)
}

/// The canonical chart of the plane `normal · p = d`: origin at the foot of
/// the perpendicular from the world origin, axes from
/// [`Frame3::from_normal`](remus_math::frame::Frame3::from_normal). The
/// in-plane axes are the ones
/// [`project_points_to_surface`](crate::construct::project_curve::project_points_to_surface)
/// projects with; the origin is exact for a non-unit `normal` too.
fn canonical_plane_frame(normal: Vec3, d: f64) -> Result<Frame3, HealError> {
    let scale = d / normal.dot(normal);
    let origin = Point3::new(normal.x() * scale, normal.y() * scale, normal.z() * scale);
    Frame3::from_normal(origin, normal)
        .map_err(|e| HealError::AnalysisFailed(format!("plane frame: {e}")))
}

/// The pcurve of `edge` on `face` fitted by projection, oriented along the
/// use.
fn fitted_pcurve(
    topo: &Topology,
    edge_id: EdgeId,
    face_id: FaceId,
    forward: bool,
    ctx: &HealContext,
) -> Result<PCurve, HealError> {
    let nurbs_3d = crate::construct::project_curve::project_edge_to_pcurve(
        topo,
        edge_id,
        face_id,
        SAME_PARAM_SAMPLES,
        &ctx.tolerance,
    )?;

    // Convert the 3D NURBS (with z=0) to a NurbsCurve2D.
    let cp_2d: Vec<Point2> = nurbs_3d
        .control_points()
        .iter()
        .map(|p| Point2::new(p.x(), p.y()))
        .collect();
    let weights = nurbs_3d.weights().to_vec();
    let knots = nurbs_3d.knots().to_vec();
    let degree = nurbs_3d.degree();

    let nurbs_2d = NurbsCurve2D::new(degree, knots.clone(), cp_2d, weights).map_err(|e| {
        HealError::FixFailed(format!(
            "failed to construct NurbsCurve2D for edge {edge_id:?}: {e}"
        ))
    })?;

    let t_start = knots[degree];
    let t_end = knots[knots.len() - degree - 1];

    // The fit runs along the edge's own direction; a reversed use traverses
    // it backwards, so its pcurve takes the parameter range reversed.
    let pcurve = if forward {
        PCurve::new(Curve2D::Nurbs(nurbs_2d), t_start, t_end)
    } else {
        PCurve::new(Curve2D::Nurbs(nurbs_2d), t_end, t_start)
    };
    Ok(pcurve)
}

/// The exact pcurve of a line or circle edge lying in a plane, in the
/// plane's canonical chart, oriented along the use; `None` for any other
/// curve, or a circle whose own plane is not the face's.
///
/// A line maps to the chart segment between its use's end points over
/// `(0, length)`. A circle maps to a chart circle whose angle is the edge
/// parameter shifted by the circle's phase in the chart, negated when the
/// circle's normal opposes the plane's; the range follows the edge domain
/// along the use.
fn exact_plane_pcurve(
    topo: &Topology,
    edge_id: EdgeId,
    forward: bool,
    normal: Vec3,
    d: f64,
) -> Result<Option<PCurve>, HealError> {
    use remus_math::curves2d::{Circle2D, Line2D};
    use remus_topology::edge::EdgeCurve;

    let frame = canonical_plane_frame(normal, d)?;
    let to_chart = |p: Point3| {
        let rel = p - frame.origin;
        Point2::new(rel.dot(frame.x), rel.dot(frame.y))
    };
    let edge = topo.edge(edge_id)?;
    match edge.curve() {
        EdgeCurve::Line => {
            let start = topo.vertex(edge.start())?.point();
            let end = topo.vertex(edge.end())?.point();
            let (from, to) = if forward { (start, end) } else { (end, start) };
            let (uv0, uv1) = (to_chart(from), to_chart(to));
            let Ok(line) = Line2D::new(uv0, uv1 - uv0) else {
                return Ok(None);
            };
            Ok(Some(PCurve::new(
                Curve2D::Line(line),
                0.0,
                (uv1 - uv0).length(),
            )))
        }
        EdgeCurve::Circle(circle) => {
            let along = circle.normal().dot(frame.z);
            // Off-plane centers and tilted circles stay with the fitted
            // projection; strict validation then reports them as they are.
            let off_plane = (circle.center() - frame.origin).dot(frame.z);
            if along.abs() < 1.0 - 1e-12 || off_plane.abs() > 1e-9 * circle.radius().max(1.0) {
                return Ok(None);
            }
            let Ok(chart_circle) = Circle2D::new(to_chart(circle.center()), circle.radius()) else {
                return Ok(None);
            };
            let u = circle.u_axis();
            let phase = u.dot(frame.y).atan2(u.dot(frame.x));
            let angle = |t: f64| if along > 0.0 { phase + t } else { phase - t };
            let (t0, t1) = edge
                .strict_domain()
                .map_err(|e| HealError::AnalysisFailed(format!("edge domain: {e}")))?;
            let (a, b) = if forward { (t0, t1) } else { (t1, t0) };
            Ok(Some(PCurve::new(
                Curve2D::Circle(chart_circle),
                angle(a),
                angle(b),
            )))
        }
        EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => Ok(None),
    }
}

/// Outcome of a budget-bound pcurve repair.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PcurveRepairReport {
    /// Maximum deviation before the repair (`f64::INFINITY` when no pcurve
    /// existed).
    pub deviation_before: f64,
    /// Maximum deviation after the repair.
    pub deviation_after: f64,
}

/// Rebuilds the pcurve of `edge` on `face` by projection, **within a
/// declared error budget** (RFC 0002, Stage 3 controlled repair).
///
/// The repair is never silent and never commits a result that misses its
/// budget: when the rebuilt pcurve still deviates beyond `budget`, the
/// original pcurve (or its absence) is restored and a typed error reports
/// the achieved deviation. On success the report discloses the deviation
/// before and after, for the caller's tolerance ledger.
///
/// # Errors
///
/// Returns [`HealError::RepairBudgetExceeded`] when the rebuilt pcurve
/// cannot meet `budget` (topology unchanged), or other [`HealError`]s when
/// projection or topology access fails.
pub fn repair_pcurve_within_budget(
    topo: &mut Topology,
    edge_id: EdgeId,
    face_id: FaceId,
    ctx: &mut HealContext,
    budget: f64,
) -> Result<PcurveRepairReport, HealError> {
    // Same seam reasoning as `fix_same_parameter_on_face`: a face that uses the
    // edge twice does not name one p-curve, and this entry point has no side to
    // repair. Refuse with the budget error rather than the registry's.
    let forward = match face_edge_senses(topo, face_id, edge_id)?.as_slice() {
        [only] => *only,
        _ => {
            return Err(HealError::FixFailed(format!(
                "edge {edge_id:?} is a seam of face {face_id:?}; \
                 pcurve repair needs a single edge use"
            )));
        }
    };
    let deviation_before = if topo.pcurve_oriented(edge_id, face_id, forward).is_some() {
        compute_pcurve_deviation(topo, edge_id, face_id, forward)?
    } else {
        f64::INFINITY
    };
    let original = topo.pcurve_oriented(edge_id, face_id, forward).cloned();

    let config = crate::fix::config::FixConfig {
        fix_same_parameter: crate::fix::config::FixMode::On,
        ..Default::default()
    };
    fix_same_parameter_on_face(topo, edge_id, face_id, ctx, &config)?;

    let deviation_after = compute_pcurve_deviation(topo, edge_id, face_id, forward)?;
    if deviation_after > budget {
        // Roll back: a repair that misses its budget must not look like
        // success, and must not replace the caller's data.
        match original {
            Some(pcurve) => {
                topo.set_pcurve_oriented(edge_id, face_id, forward, pcurve)?;
            }
            None => {
                topo.remove_pcurve_oriented(edge_id, face_id, forward)?;
            }
        }
        return Err(HealError::RepairBudgetExceeded {
            achieved: deviation_after,
            budget,
        });
    }
    ctx.info(format!(
        "Edge {edge_id:?} on Face {face_id:?}: pcurve repaired, deviation \
         {deviation_before:.2e} -> {deviation_after:.2e} (budget {budget:.2e})",
    ));
    Ok(PcurveRepairReport {
        deviation_before,
        deviation_after,
    })
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod repair_budget_tests {
    use remus_math::curves::Circle3D;
    use remus_math::curves2d::{Curve2D, Line2D};
    use remus_math::surfaces::CylindricalSurface;
    use remus_math::traits::ParametricCurve;
    use remus_math::vec::{Point2, Point3, Vec2, Vec3};
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
    use remus_topology::face::{Face, FaceId, FaceSurface};
    use remus_topology::pcurve::PCurve;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    use crate::context::HealContext;
    use crate::error::HealError;

    use super::{compute_pcurve_deviation, repair_pcurve_within_budget};

    fn cylinder_rim() -> (Topology, EdgeId, FaceId) {
        let mut topo = Topology::new();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let p0 = ParametricCurve::evaluate(&circle, 0.0);
        let v = topo.add_vertex(Vertex::new(p0, 1e-7));
        let mut rim_edge = Edge::new(v, v, EdgeCurve::Circle(circle));
        rim_edge.set_trim(Some((0.0, std::f64::consts::TAU)));
        let rim = topo.add_edge(rim_edge);
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true).unwrap());
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Cylinder(
                CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0)
                    .unwrap(),
            ),
        ));
        (topo, rim, face)
    }

    #[test]
    fn repair_within_budget_reports_before_and_after() {
        let (mut topo, rim, face) = cylinder_rim();
        let mut ctx = HealContext::new();
        let report = repair_pcurve_within_budget(&mut topo, rim, face, &mut ctx, 1e-3).unwrap();
        assert!(report.deviation_before.is_infinite(), "no pcurve existed");
        assert!(
            report.deviation_after <= 1e-3,
            "repaired deviation {} must meet the budget",
            report.deviation_after
        );
        assert!(topo.has_pcurve(rim, face).unwrap());
    }

    /// The square `[0, 2]² × {0}` on the plane `z = 0`, and its edge from
    /// `(0, 0, 0)` to `(2, 0, 0)`.
    fn square_face() -> (Topology, EdgeId, FaceId) {
        let mut topo = Topology::new();
        let corners = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)];
        let vertices: Vec<_> = corners
            .iter()
            .map(|&(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), 1e-7)))
            .collect();
        let edges: Vec<_> = (0..4)
            .map(|i| {
                topo.add_edge(Edge::new(
                    vertices[i],
                    vertices[(i + 1) % 4],
                    EdgeCurve::Line,
                ))
            })
            .collect();
        let wire = topo.add_wire(
            Wire::new(
                edges.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        (topo, edges[0], face)
    }

    /// A pcurve for the square's first edge in the canonical frame of
    /// `z = 0` (`u = y`, `v = -x`), shifted by `offset` along `u`.
    fn bottom_pcurve(offset: f64) -> PCurve {
        let line = Line2D::new(Point2::new(offset, 0.0), Vec2::new(0.0, -1.0)).unwrap();
        PCurve::new(Curve2D::Line(line), 0.0, 2.0)
    }

    #[test]
    fn plane_pcurve_deviation_is_measured_in_the_canonical_frame() {
        for (offset, expected) in [(0.0, 0.0), (0.5, 0.5), (-1e-3, 1e-3)] {
            let (mut topo, edge, face) = square_face();
            topo.set_pcurve_oriented(edge, face, true, bottom_pcurve(offset))
                .unwrap();
            let deviation = compute_pcurve_deviation(&topo, edge, face, true).unwrap();
            assert!(
                (deviation - expected).abs() < 1e-12,
                "offset {offset}: deviation {deviation}, expected {expected}"
            );
        }
    }

    /// Strict validation's verdict on one use: the certified deviation.
    fn strictly_certified_deviation(
        topo: &Topology,
        edge: EdgeId,
        face: FaceId,
        forward: bool,
    ) -> f64 {
        remus_topology::validation::check_same_parameter_strict(topo, edge, face, forward, 0)
            .unwrap()
            .unwrap()
            .max_deviation
    }

    #[test]
    fn plane_repair_measures_an_offset_pcurve_against_its_budget() {
        let (mut topo, edge, face) = square_face();
        topo.set_pcurve_oriented(edge, face, true, bottom_pcurve(0.5))
            .unwrap();
        let mut ctx = HealContext::new();

        // The rebuild is exact in the plane's chart, so even a 1e-30 budget
        // holds, and strict validation certifies the analytic line it wrote.
        let report = repair_pcurve_within_budget(&mut topo, edge, face, &mut ctx, 1e-30).unwrap();
        assert!((report.deviation_before - 0.5).abs() < 1e-12, "{report:?}");
        assert!(report.deviation_after <= 1e-30, "{report:?}");
        let rebuilt = topo.pcurve_oriented(edge, face, true).unwrap();
        assert!(matches!(rebuilt.curve(), Curve2D::Line(_)), "{rebuilt:?}");
        assert!(strictly_certified_deviation(&topo, edge, face, true) <= 1e-9);
    }

    /// An edge lifted 1e-3 off its plane has no pcurve within tolerance: the
    /// rebuild must keep the caller's pcurve and refuse, not report a repair.
    #[test]
    fn off_plane_edge_keeps_its_pcurve_and_refuses() {
        let (mut topo, edge, face) = square_face();
        let (start, end) = {
            let e = topo.edge(edge).unwrap();
            (e.start(), e.end())
        };
        for v in [start, end] {
            let p = topo.vertex(v).unwrap().point();
            topo.vertex_mut(v)
                .unwrap()
                .set_point(Point3::new(p.x(), p.y(), 1e-3));
        }
        topo.set_pcurve_oriented(edge, face, true, bottom_pcurve(0.0))
            .unwrap();
        let mut ctx = HealContext::new();
        let config = crate::fix::config::FixConfig::default();
        let result =
            super::fix_same_parameter_on_face(&mut topo, edge, face, &mut ctx, &config).unwrap();
        assert_eq!(result.actions_taken, 0, "{result:?}");
        assert_eq!(
            result.refusals.first().map(|r| r.kind),
            Some(crate::fix::RepairRefusalKind::PcurveDeviationRemains),
            "{result:?}"
        );
        let kept = topo.pcurve_oriented(edge, face, true).unwrap();
        assert!((kept.evaluate(0.0) - Point2::new(0.0, 0.0)).length() < 1e-15);
        assert!((compute_pcurve_deviation(&topo, edge, face, true).unwrap() - 1e-3).abs() < 1e-12);
    }

    /// A disc on `z = 0` bounded by one full circle about `(1, 2, 0)` of
    /// radius 1.5, its normal along `normal_z` and its seam at `phase`.
    fn disc_face(normal_z: f64, phase: f64) -> (Topology, EdgeId, FaceId) {
        let mut topo = Topology::new();
        let circle = Circle3D::new_with_ref(
            Point3::new(1.0, 2.0, 0.0),
            Vec3::new(0.0, 0.0, normal_z),
            1.5,
            Vec3::new(phase.cos(), phase.sin(), 0.0),
        )
        .unwrap();
        let p0 = ParametricCurve::evaluate(&circle, 0.0);
        let v = topo.add_vertex(Vertex::new(p0, 1e-7));
        let mut rim_edge = Edge::new(v, v, EdgeCurve::Circle(circle));
        rim_edge.set_trim(Some((0.0, std::f64::consts::TAU)));
        let rim = topo.add_edge(rim_edge);
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true).unwrap());
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        (topo, rim, face)
    }

    /// A planar circle rebuilds as a chart circle, phase and winding
    /// included, which strict validation certifies; a NURBS fit has no
    /// strict proof on a plane.
    #[test]
    fn planar_circle_repair_writes_a_certified_chart_circle() {
        use remus_math::curves2d::Circle2D;
        for (normal_z, phase) in [(1.0, 0.0), (1.0, 0.7), (-1.0, 2.0)] {
            let (mut topo, rim, face) = disc_face(normal_z, phase);
            // Right radius, centre 0.25 off.
            let offset = Circle2D::new(Point2::new(2.25, -1.0), 1.5).unwrap();
            topo.set_pcurve_oriented(
                rim,
                face,
                true,
                PCurve::new(Curve2D::Circle(offset), 0.0, std::f64::consts::TAU),
            )
            .unwrap();
            let mut ctx = HealContext::new();
            let report = repair_pcurve_within_budget(&mut topo, rim, face, &mut ctx, 1e-9).unwrap();
            assert!(
                report.deviation_before > 0.2,
                "{normal_z} {phase}: {report:?}"
            );
            assert!(
                report.deviation_after <= 1e-9,
                "{normal_z} {phase}: {report:?}"
            );
            let rebuilt = topo.pcurve_oriented(rim, face, true).unwrap();
            assert!(matches!(rebuilt.curve(), Curve2D::Circle(_)), "{rebuilt:?}");
            let certified = strictly_certified_deviation(&topo, rim, face, true);
            assert!(
                certified <= 1e-9,
                "{normal_z} {phase}: certified {certified}"
            );
        }
    }

    /// [`square_face`] traversed the other way round: every use reversed, so
    /// the bottom edge's use runs from `(2, 0, 0)` to `(0, 0, 0)`.
    fn reversed_square_face() -> (Topology, EdgeId, FaceId) {
        let (mut topo, edge, face) = square_face();
        let wire = topo.face(face).unwrap().outer_wire();
        let mut uses: Vec<_> = topo
            .wire(wire)
            .unwrap()
            .edges()
            .iter()
            .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
            .collect();
        uses.reverse();
        let reversed = topo.add_wire(Wire::new(uses, true).unwrap());
        topo.set_face_boundary_wires(face, reversed, vec![])
            .unwrap();
        (topo, edge, face)
    }

    #[test]
    fn reversed_planar_use_is_measured_and_rebuilt_along_the_use() {
        // Canonical pcurve of the reversed use: uv (0, -2) → (0, 0).
        let (mut topo, edge, face) = reversed_square_face();
        let line = Line2D::new(Point2::new(0.0, -2.0), Vec2::new(0.0, 1.0)).unwrap();
        topo.set_pcurve_oriented(
            edge,
            face,
            false,
            PCurve::new(Curve2D::Line(line), 0.0, 2.0),
        )
        .unwrap();
        let deviation = compute_pcurve_deviation(&topo, edge, face, false).unwrap();
        assert!(
            deviation < 1e-12,
            "a correct reversed pcurve read {deviation}"
        );

        // Missing pcurve on the reversed use: the rebuild must run along it.
        let (mut topo, edge, face) = reversed_square_face();
        let mut ctx = HealContext::new();
        let report = repair_pcurve_within_budget(&mut topo, edge, face, &mut ctx, 1e-9).unwrap();
        assert!(report.deviation_after <= 1e-9, "{report:?}");
        let rebuilt = topo.pcurve_oriented(edge, face, false).unwrap();
        let start = rebuilt.evaluate(rebuilt.t_start());
        assert!(
            (start - Point2::new(0.0, -2.0)).length() < 1e-9,
            "the reversed use starts at (2, 0, 0), uv (0, -2): {start:?}"
        );
    }

    #[test]
    fn auto_same_parameter_rebuilds_only_the_offset_planar_pcurve() {
        let config = crate::fix::config::FixConfig::default();
        for (offset, rebuilt) in [(0.0, 0), (0.5, 1)] {
            let (mut topo, edge, face) = square_face();
            topo.set_pcurve_oriented(edge, face, true, bottom_pcurve(offset))
                .unwrap();
            let mut ctx = HealContext::new();
            let result =
                super::fix_same_parameter_on_face(&mut topo, edge, face, &mut ctx, &config)
                    .unwrap();
            assert_eq!(result.actions_taken, rebuilt, "offset {offset}");
            assert!(compute_pcurve_deviation(&topo, edge, face, true).unwrap() < 1e-9);
        }
    }

    #[test]
    fn repair_missing_an_impossible_budget_fails_typed_and_rolls_back() {
        let (mut topo, rim, face) = cylinder_rim();
        let mut ctx = HealContext::new();
        let err = repair_pcurve_within_budget(&mut topo, rim, face, &mut ctx, 1e-30).unwrap_err();
        assert!(matches!(err, HealError::RepairBudgetExceeded { .. }));
        assert!(
            !topo.has_pcurve(rim, face).unwrap(),
            "a repair that misses its budget must not leave its result behind"
        );
    }
}
