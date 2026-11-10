//! One-sided material contradiction check for a GFA result whose section
//! set was incomplete.
//!
//! The face splitter drops an open section chain whose loose ends lie inside
//! a face and keeps the face whole (see
//! [`remus_algo::gfa::SectionDropScope`]). Usually the face classifies the
//! same on either side of the stray arc and the result is right — the hammer
//! holder's replays drop dozens and match their references. It also returned
//! a swept tube untouched under an annular cutter (OpenZCAD issue 1242) and
//! kept a holed block's hole in a torus intersection (issue 1244), each a
//! closed, valid solid that every structural gate passed. Those families are
//! now partitioned without a drop (the torus chart splitter); this check is
//! what remains for the established paths.
//!
//! It is a contradiction DETECTOR, not a certificate. On a fixed grid over
//! the region the result can occupy, a point clearly inside or outside each
//! operand must be inside the result exactly when the operation says, and a
//! clear disagreement refuses. Agreement proves nothing: a wrong result whose
//! error lies between grid points passes. A 200 mm blank cut by a thin
//! annular tool (inner radius 6.75, outer 7.25, height 20) returned
//! unchanged passes every grid here although the point (7, 0, 0) is 0.25
//! inside the tool's material; no finite sample set certifies a result built
//! on an incomplete section set. Results built without a drop never pay for
//! the check.
use remus_math::aabb::Aabb3;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

use super::BooleanOp;
use crate::classify::{PointClassification, RobustClassifier};

/// Points per axis of the verification grid.
const GRID: usize = 7;
/// The classifiers' deflection is this fraction of the region's diagonal,
/// capped at [`MAX_DEFLECTION`], so the check scales with the model.
const DEFLECTION_FRACTION: f64 = 2e-3;
/// Largest deflection used, in model units.
const MAX_DEFLECTION: f64 = 0.1;

/// Whether no unambiguous grid point contradicts `results` (their union)
/// being `op` of `a` and `b`. Unmeasurable boxes and unclassifiable points
/// are not witnesses; only a clear contradiction rejects. An empty `results`
/// is the empty set.
pub(super) fn material_witnesses_agree(
    topo: &Topology,
    op: BooleanOp,
    a: SolidId,
    b: SolidId,
    results: &[SolidId],
    tol: Tolerance,
) -> bool {
    let (Ok(a_box), Ok(b_box)) = (
        crate::measure::solid_bounding_box(topo, a),
        crate::measure::solid_bounding_box(topo, b),
    ) else {
        return true;
    };
    // Where the result may hold material: a union within both operands'
    // boxes, a difference within its blank's, an intersection within their
    // overlap. Points outside those are outside every operand that matters,
    // and the result's vertices are already held there by
    // `result_within_operand_bounds`. The overlap is always sampled on its
    // own grid as well: it is where the operation decides anything, and it
    // can be far smaller than the union of the boxes (a tube under an
    // annular cutter).
    let common = overlap(a_box, b_box);
    let regions: Vec<Aabb3> = match op {
        BooleanOp::Fuse => [Some(a_box), Some(b_box), common]
            .into_iter()
            .flatten()
            .collect(),
        BooleanOp::Cut => [Some(a_box), common].into_iter().flatten().collect(),
        BooleanOp::Intersect => common.into_iter().collect(),
    };
    let Some(scale) = regions
        .iter()
        .map(|r| (r.max - r.min).length())
        .reduce(f64::max)
    else {
        return true;
    };
    let deflection = (scale * DEFLECTION_FRACTION)
        .min(MAX_DEFLECTION)
        .max(tol.linear);
    let margin = 2.0 * deflection;
    let mut in_a = RobustClassifier::new(topo, a, deflection, margin);
    let mut in_b = RobustClassifier::new(topo, b, deflection, margin);
    let mut in_results: Vec<RobustClassifier<'_>> = results
        .iter()
        .map(|&result| RobustClassifier::new(topo, result, deflection, margin))
        .collect();
    let decided = |classifier: &mut RobustClassifier<'_>, p: Point3| match classifier.classify(p) {
        Ok(PointClassification::Inside) => Some(true),
        Ok(PointClassification::Outside) => Some(false),
        Ok(PointClassification::OnBoundary) | Err(_) => None,
    };
    #[allow(clippy::cast_precision_loss)]
    let at = |i: usize| (i as f64 + 0.5) / GRID as f64;
    for region in regions {
        let extent = region.max - region.min;
        if extent.x() <= tol.linear || extent.y() <= tol.linear || extent.z() <= tol.linear {
            continue;
        }
        for i in 0..GRID {
            for j in 0..GRID {
                for k in 0..GRID {
                    let p = Point3::new(
                        extent.x().mul_add(at(i), region.min.x()),
                        extent.y().mul_add(at(j), region.min.y()),
                        extent.z().mul_add(at(k), region.min.z()),
                    );
                    let (Some(pa), Some(pb)) = (decided(&mut in_a, p), decided(&mut in_b, p))
                    else {
                        continue;
                    };
                    // Inside the union of the results when inside any one.
                    // The winding classifier counts a point inside two
                    // overlapping shells as inside (winding 2), which is how
                    // a union that kept both operands' whole shells looks
                    // right to it, so each result must also agree under the
                    // analytic ray cast.
                    let mut pr = Some(false);
                    let mut parity = Some(false);
                    for in_result in &mut in_results {
                        match decided(in_result, p) {
                            Some(true) => pr = Some(true),
                            Some(false) => {}
                            None => pr = pr.filter(|&inside| inside),
                        }
                        match in_result.fallback_ray_cast(p) {
                            Ok(PointClassification::Inside) => parity = Some(true),
                            Ok(PointClassification::Outside) => {}
                            Ok(PointClassification::OnBoundary) | Err(_) => {
                                parity = parity.filter(|&inside| inside);
                            }
                        }
                    }
                    let Some(pr) = pr else {
                        continue;
                    };
                    let expected = match op {
                        BooleanOp::Fuse => pa || pb,
                        BooleanOp::Cut => pa && !pb,
                        BooleanOp::Intersect => pa && pb,
                    };
                    if pr != expected || parity.is_some_and(|inside| inside != expected) {
                        log::debug!(
                            "material witness {op:?} at {p:?}: a={pa} b={pb} result={pr} \
                             (ray parity {parity:?}), expected {expected}"
                        );
                        return false;
                    }
                }
            }
        }
    }
    true
}

/// The overlap of two boxes, `None` when they do not overlap.
fn overlap(a: Aabb3, b: Aabb3) -> Option<Aabb3> {
    let min = Point3::new(
        a.min.x().max(b.min.x()),
        a.min.y().max(b.min.y()),
        a.min.z().max(b.min.z()),
    );
    let max = Point3::new(
        a.max.x().min(b.max.x()),
        a.max.y().min(b.max.y()),
        a.max.z().min(b.max.z()),
    );
    (min.x() <= max.x() && min.y() <= max.y() && min.z() <= max.z()).then_some(Aabb3 { min, max })
}
