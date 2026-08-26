//! Edge fixing — vertex-curve alignment, degenerate removal, same-parameter.

use remus_math::curves2d::{Curve2D, NurbsCurve2D};
use remus_math::vec::Point2;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::FaceId;
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
            ctx.info(format!(
                "Edge {edge_id:?} on Face {face_id:?}: seam edge (used in both senses);                  SameParameter needs a side the projection cannot infer, skipping",
            ));
            return Ok(FixResult {
                status: Status::FAIL1,
                actions_taken: 0,
            });
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
        // No PCurve exists -- always needs fixing unless mode is Off.
        if !config.fix_same_parameter.should_fix(true) {
            return Ok(FixResult::ok());
        }

        ctx.info(format!(
            "Edge {edge_id:?} on Face {face_id:?}: no PCurve found, creating via projection",
        ));
    }

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

    let pcurve = PCurve::new(Curve2D::Nurbs(nurbs_2d), t_start, t_end);
    topo.set_pcurve_oriented(edge_id, face_id, forward, pcurve);

    Ok(FixResult {
        status: Status::DONE3,
        actions_taken: 1,
    })
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

    Ok(FixResult {
        status: Status::DONE1,
        actions_taken: 1,
    })
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

    Ok(FixResult {
        status: Status::DONE2,
        actions_taken: 1,
    })
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

    FixResult {
        status: Status::DONE3,
        actions_taken: 0,
    }
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

    let (t0_3d, t1_3d) = edge.domain_with_endpoints(start_pos, end_pos);
    let t0_pc = pcurve.t_start();
    let t1_pc = pcurve.t_end();

    let mut max_dev = 0.0_f64;

    for i in 0..=SAME_PARAM_SAMPLES {
        #[allow(clippy::cast_precision_loss)]
        let frac = i as f64 / SAME_PARAM_SAMPLES as f64;

        let t_3d = t0_3d + (t1_3d - t0_3d) * frac;
        let pt_3d = curve.evaluate_with_endpoints(t_3d, start_pos, end_pos);

        let t_pc = t0_pc + (t1_pc - t0_pc) * frac;
        let uv = pcurve.evaluate(t_pc);

        if let Some(pt_surf) = surface.evaluate(uv.x(), uv.y()) {
            let dev = (pt_3d - pt_surf).length();
            max_dev = max_dev.max(dev);
        }
        // If surface.evaluate returns None (plane), skip that sample --
        // plane PCurves are validated differently.
    }

    Ok(max_dev)
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
            Some(pcurve) => topo.set_pcurve_oriented(edge_id, face_id, forward, pcurve),
            None => {
                let _ = topo.remove_pcurve_oriented(edge_id, face_id, forward);
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
    use remus_math::surfaces::CylindricalSurface;
    use remus_math::traits::ParametricCurve;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
    use remus_topology::face::{Face, FaceId, FaceSurface};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    use crate::context::HealContext;
    use crate::error::HealError;

    use super::repair_pcurve_within_budget;

    fn cylinder_rim() -> (Topology, EdgeId, FaceId) {
        let mut topo = Topology::new();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let p0 = ParametricCurve::evaluate(&circle, 0.0);
        let v = topo.add_vertex(Vertex::new(p0, 1e-7));
        let rim = topo.add_edge(Edge::new(v, v, EdgeCurve::Circle(circle)));
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
