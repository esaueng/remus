//! Recover a knot-budget refusal by sweeping a nonperiodic trim the other way.

use super::{
    CheckError, FaceContribution, IntegrationRule, MAX_KNOT_ALIGNED_CELLS, NurbsSurface,
    ParametricSurface, PatchScale, Point2, UvBounds, UvLoop, UvTrim, integrate_knot_aligned,
};

/// Retain every admitted quadrature unchanged. Only a preflight refusal may
/// try a different slicing direction, and that attempt has the same cell cap.
/// Periodic bands retain their existing refusal: their domain semantics are
/// tied to the original u direction. The dense axis moves outwards only once,
/// so a failed retry cannot recurse back to the original sweep.
#[allow(clippy::too_many_arguments)]
pub(super) fn retry<S: ParametricSurface, const AREA_ONLY: bool>(
    surface: &S,
    range: UvBounds,
    rule: IntegrationRule<'_>,
    sign: f64,
    trim: &UvTrim<'_>,
    scale: PatchScale,
) -> Result<Option<FaceContribution>, CheckError> {
    let refused = || {
        CheckError::IntegrationFailed(format!(
            "knot-aligned quadrature needs more than {MAX_KNOT_ALIGNED_CELLS} cells"
        ))
    };
    // The generator certificate is tied to linear v. Its caller can retry
    // the ordinary rule, including this transpose, after a budget refusal.
    if rule.ruled_guard {
        return Err(refused());
    }
    let Some(s) = surface.as_nurbs() else {
        return Err(refused());
    };
    if s.is_periodic_u()
        || s.is_periodic_v()
        || trim.u_periodic
        || !trim.bands.is_empty()
        || s.knots_v().len() <= s.knots_u().len()
    {
        return Err(refused());
    }
    let points = s.control_points();
    let weights = s.weights();
    let transposed = NurbsSurface::new(
        s.degree_v(),
        s.degree_u(),
        s.knots_v().to_vec(),
        s.knots_u().to_vec(),
        (0..points[0].len())
            .map(|v| points.iter().map(|row| row[v]).collect())
            .collect(),
        (0..weights[0].len())
            .map(|v| weights.iter().map(|row| row[v]).collect())
            .collect(),
    )?;
    let flip = |loop_: &UvLoop| {
        UvLoop::new(
            loop_
                .points
                .iter()
                .map(|p| Point2::new(p.y(), p.x()))
                .collect(),
            false,
            false,
        )
    };
    let outer = trim.outer.map(flip);
    let pockets: Vec<_> = trim.pockets.iter().map(flip).collect();
    let transposed_trim = UvTrim::new(outer.as_ref(), &pockets, &[], false);
    let knots = (transposed.knots_u(), transposed.knots_v());
    let rule = IntegrationRule {
        knots: Some(knots),
        ..rule
    };
    // Swapping partial derivatives reverses the carrier's normal; reverse its
    // sign too so signed volume, vector area and all moments retain orientation.
    integrate_knot_aligned::<_, AREA_ONLY>(
        &transposed,
        range.1,
        range.0,
        rule,
        -sign,
        &transposed_trim,
        PatchScale {
            u: scale.v,
            v: scale.u,
        },
        knots,
    )
}
