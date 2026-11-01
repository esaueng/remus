//! A cylinder/block seam followed by its adjoining convex straight edge.
//!
//! All three seeds are one user operation. The seam must be built together,
//! before the convex edge; its construction history identifies the shortened
//! edge without proximity guessing. The caller transactions the whole attempt.

use remus_math::tolerance::Tolerance;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::solid_edges;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use super::{BlendError, BlendResult, BlendSize, OperationsError};

struct Plan {
    seam: [EdgeId; 2],
    straight: EdgeId,
    supports: [FaceId; 2],
    corner: Point3,
    far: Point3,
}

fn plan(
    topo: &Topology,
    solid: SolidId,
    edges: &[EdgeId],
) -> Result<Option<Plan>, OperationsError> {
    if edges.len() != 3 {
        return Ok(None);
    }
    let adjacency = topo.build_adjacency(solid)?;
    // The one non-seam seed must be a line between the two planes that meet
    // along the seam's corner. This excludes unrelated, curved and ambiguous
    // remnants before any topology is changed.
    let mut plans = Vec::new();
    for (i, &straight) in edges.iter().enumerate() {
        let line = topo.edge(straight)?;
        let supports = adjacency.faces_for_edge(straight);
        if !matches!(line.curve(), EdgeCurve::Line)
            || supports.len() != 2
            || supports[0] == supports[1]
            || supports.iter().any(|&f| {
                !topo
                    .face(f)
                    .is_ok_and(|face| face.effective_plane_normal().is_some())
            })
        {
            continue;
        }
        let seam: Vec<_> = edges
            .iter()
            .enumerate()
            .filter_map(|(j, &e)| (i != j).then_some(e))
            .collect();
        let first = topo.edge(seam[0])?;
        let second = topo.edge(seam[1])?;
        let common = adjacency
            .faces_for_edge(seam[0])
            .iter()
            .copied()
            .filter(|&f| {
                adjacency.faces_for_edge(seam[1]).contains(&f)
                    && topo.face(f).is_ok_and(|face| {
                        matches!(face.surface(), FaceSurface::Cylinder(_)) && !face.is_reversed()
                    })
            })
            .count();
        if common != 1
            || seam.iter().any(|&e| {
                let faces = adjacency.faces_for_edge(e);
                faces.len() != 2 || !faces.iter().any(|f| supports.contains(f))
            })
        {
            continue;
        }
        let corners: Vec<_> = [first.start(), first.end()]
            .into_iter()
            .filter(|v| {
                (*v == second.start() || *v == second.end())
                    && (*v == line.start() || *v == line.end())
            })
            .collect();
        if let [corner] = corners.as_slice() {
            let far = if line.start() == *corner {
                line.end()
            } else {
                line.start()
            };
            plans.push(Plan {
                seam: [seam[0], seam[1]],
                straight,
                supports: [supports[0], supports[1]],
                corner: topo.vertex(*corner)?.point(),
                far: topo.vertex(far)?.point(),
            });
        }
    }
    Ok(if plans.len() == 1 { plans.pop() } else { None })
}

fn remap(topo: &Topology, step: &BlendResult, plan: &Plan) -> Result<EdgeId, OperationsError> {
    let refusal = || {
        OperationsError::Blend(BlendError::EdgesNotBlended {
            edges: vec![plan.straight],
            reason: "the seam did not leave a unique construction-derived straight edge".into(),
        })
    };
    let origins = step.face_origins.as_ref().ok_or_else(refusal)?;
    let mut supports = Vec::new();
    for input in plan.supports {
        let mapped: Vec<_> = origins
            .survived
            .iter()
            .filter_map(|&(from, to)| (from == input).then_some(to))
            .collect();
        let [output] = mapped.as_slice() else {
            return Err(refusal());
        };
        supports.push(*output);
    }
    let axis = plan.far - plan.corner;
    let length = axis.length();
    let tol = Tolerance::new().linear * 10.0;
    if length <= tol {
        return Err(refusal());
    }
    let direction = axis * (1.0 / length);
    let on_span = |p: Point3| {
        let along = (p - plan.corner).dot(direction);
        along >= -tol
            && along <= length + tol
            && (p - (plan.corner + direction * along)).length() <= tol
    };
    let adjacency = topo.build_adjacency(step.solid)?;
    let mut candidates = Vec::new();
    for edge_id in solid_edges(topo, step.solid)? {
        let edge = topo.edge(edge_id)?;
        let faces = adjacency.faces_for_edge(edge_id);
        if !matches!(edge.curve(), EdgeCurve::Line)
            || faces.len() != 2
            || supports.iter().any(|f| !faces.contains(f))
        {
            continue;
        }
        let a = topo.vertex(edge.start())?.point();
        let b = topo.vertex(edge.end())?.point();
        if (a - b).length() > tol
            && on_span(a)
            && on_span(b)
            && ((a - plan.far).length() <= tol || (b - plan.far).length() <= tol)
        {
            candidates.push(edge_id);
        }
    }
    match candidates.as_slice() {
        [edge] => Ok(*edge),
        _ => Err(refusal()),
    }
}

pub(super) fn fillet(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    radius: f64,
) -> Result<Option<BlendResult>, OperationsError> {
    let Some(plan) = plan(topo, solid, edges)? else {
        return Ok(None);
    };
    let seam = super::fillet_group(topo, solid, &plan.seam, radius)?;
    let straight = remap(topo, &seam, &plan)?;
    let mut result = super::fillet_cascade_attempts(topo, seam.solid, &[straight], radius)?;
    result.succeeded = edges.to_vec();
    // Face histories from successive trims need composition. Do not claim the
    // second stage's intermediate faces were faces of the original input.
    result.face_origins = None;
    if result.engine != seam.engine {
        result.engine = super::BlendEngine::Mixed;
    }
    super::validate_complete_blend(
        topo,
        "fillet",
        solid,
        &result,
        edges,
        BlendSize::Fillet { radius },
    )?;
    super::reject_open_shell(topo, result.solid)?;
    Ok(Some(result))
}
