//! Face analysis — small faces, degeneracy, wire count.

use remus_math::tolerance::Tolerance;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::face::FaceId;

use crate::HealError;
use crate::status::Status;

/// Result of analyzing a single face.
#[derive(Debug, Clone)]
pub struct FaceAnalysis {
    /// Whether the face's bounding box diagonal is below tolerance.
    pub is_small: bool,
    /// Diagonal length of the axis-aligned box of the face's sampled boundary.
    pub bbox_diagonal: f64,
    /// Total number of wires (outer + inner).
    pub wire_count: usize,
    /// Whether the face is degenerate (its whole boundary collapses to a point).
    pub is_degenerate: bool,
    /// Outcome status flags.
    pub status: Status,
}

/// Analyze a face for size, degeneracy, and wire structure.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups fail.
pub fn analyze_face(
    topo: &Topology,
    face_id: FaceId,
    tolerance: &Tolerance,
) -> Result<FaceAnalysis, HealError> {
    /// Points sampled per boundary edge when measuring a face's extent.
    const BOUNDARY_SAMPLES: usize = 16;

    let face = topo.face(face_id)?;
    let wire_count = 1 + face.inner_wires().len();

    let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
        .chain(face.inner_wires().iter().copied())
        .collect();

    let mut min = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut vertex_count = 0usize;

    // Sample ALONG each boundary curve, never just its endpoints. A face bounded
    // by ONE closed curve — a cylinder cap, a disc, a full-circle hole rim — has
    // start == end on that edge, so an endpoint-only box collapses to a point
    // and the face measures zero. Every such face was being reported
    // `is_degenerate`, and `fix_shape`'s small-face pass removes those: a plain
    // cylinder healed from 3 faces to 1, losing both caps and 33% of its volume.
    //
    // This is the closed-edge AABB collapse the boolean-debugging skill records
    // ("a closed circular edge has start == end so its box collapses to a
    // point"), and its doctrine is the fix: closed boundary edges must be
    // sampled along the curve.
    for wid in &wire_ids {
        let wire = topo.wire(*wid)?;
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            let start = topo.vertex(edge.start())?.point();
            let end = topo.vertex(edge.end())?.point();
            let (t0, t1) = edge
                .strict_domain()
                .map_err(crate::error::analysis_edge_domain)?;
            for i in 0..=BOUNDARY_SAMPLES {
                #[allow(clippy::cast_precision_loss)]
                let t = t0 + (t1 - t0) * (i as f64) / (BOUNDARY_SAMPLES as f64);
                let p = edge.curve().evaluate_with_endpoints(t, start, end);
                min = Point3::new(min.x().min(p.x()), min.y().min(p.y()), min.z().min(p.z()));
                max = Point3::new(max.x().max(p.x()), max.y().max(p.y()), max.z().max(p.z()));
                vertex_count += 1;
            }
        }
    }

    let bbox_diagonal = if vertex_count > 0 {
        (max - min).length()
    } else {
        0.0
    };

    let is_small = bbox_diagonal < tolerance.linear;
    let is_degenerate = is_small && vertex_count > 0;

    let mut status = Status::OK;
    if is_small {
        status = status.merge(Status::DONE1);
    }
    if is_degenerate {
        status = status.merge(Status::DONE2);
    }

    Ok(FaceAnalysis {
        is_small,
        bbox_diagonal,
        wire_count,
        is_degenerate,
        status,
    })
}
