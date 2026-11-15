//! Rims that touch the outline of their planar face at one point.
//!
//! A closed section curve (a drilled hole's rim) can touch the boundary of
//! the face it cuts at one point. When the curve's own vertex is that point
//! (the rim starts on the tool's seam, and the seam is where the tool
//! touches), the pave filler splits the boundary edge there, so the rim and
//! the outline share the vertex in the assembled face. Emitted as a hole,
//! that rim is a hole wire sharing a vertex with its outer wire: one
//! boundary component written as two. The Euler–Poincaré count of inner
//! loops is then one too many and the result is refused, while the
//! mirror-image cut, whose rim is split at the touch point by an ordinary
//! crossing, traces the same region as one outline passing through the
//! vertex twice and is accepted (#1739).
//!
//! [`join_touching_rims`] writes the first form as the second, whichever
//! emitter produced the sub-faces.
//!
//! # What qualifies
//!
//! Only a circular rim whose vertex is a point of the outer wire within the
//! linear tolerance, and which stays clear of every other edge of the
//! sub-face, outer or inner, by more than the weld band. A rim that merely
//! comes near the outline is a hole. One that touches the boundary at
//! several points (a circle inscribed in a square within the tolerance, or
//! a cutter as wide as a shell's wall touching its outer edge and its
//! cavity's opening) pinches the face into pieces that one outline through
//! one vertex does not describe. Both are left as emitted. Clearance is read
//! exactly, on straight and circular edges; a sub-face carrying any other
//! boundary curve is left alone.

use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

use super::super::classify_2d::signed_area_2d;
use super::super::plane_frame::PlaneFrame;
use super::super::split_types::{OrientedPCurveEdge, SectionEdge, SplitSubFace};
use super::edge_splitting::split_boundary_edges_at_3d_points;
use super::sampling::{sample_wire_loop_uv_via_frame, sample_wire_loop_uv_via_frame_closed};

/// Points sampled on a closed rim to read its winding.
const RIM_SAMPLES: usize = 16;

/// A rim to move into its sub-face's outer wire.
struct Join {
    /// The outer wire split at the rim's vertex, when it was not already.
    split: Option<Vec<OrientedPCurveEdge>>,
    /// Index in the (split) outer wire of the edge starting at the vertex.
    at: usize,
    /// Whether the rim winds with the outline and must be reversed.
    reverse: bool,
}

/// Whether `hole` is one closed circular section of this cut: never a hole
/// the face already had.
fn is_new_rim(hole: &[OrientedPCurveEdge], sections: &[SectionEdge], weld: f64) -> bool {
    let [edge] = hole else {
        return false;
    };
    matches!(edge.curve_3d, EdgeCurve::Circle(_))
        && (edge.start_3d - edge.end_3d).length() < weld
        && sections.iter().any(|section| {
            (section.start - section.end).length() < weld
                && (section.start - edge.start_3d).length() < weld
        })
}

/// Whether the new rim at hole `index` of `part` joins the outline, and how.
fn plan_join(
    part: &SplitSubFace,
    index: usize,
    frame: &PlaneFrame,
    outer_area: f64,
    tol: f64,
    weld: f64,
) -> Option<Join> {
    let [edge] = part.inner_wires[index].as_slice() else {
        return None;
    };
    let EdgeCurve::Circle(circle) = &edge.curve_3d else {
        return None;
    };
    let vertex: Point3 = edge.start_3d;
    let (center, radius) = (circle.center(), circle.radius());
    let outer = &part.outer_wire;
    // The vertex is a point of the outline, not merely near it.
    if !outer.iter().any(|boundary| {
        super::distance_to_boundary_edge(vertex, boundary).is_some_and(|d| d <= tol)
    }) {
        return None;
    }
    let junction = |wire: &[OrientedPCurveEdge]| {
        wire.iter()
            .position(|boundary| (boundary.start_3d - vertex).length() < weld)
    };
    let (split, at) = if let Some(at) = junction(outer) {
        (None, at)
    } else {
        let split = split_boundary_edges_at_3d_points(
            outer.clone(),
            &[vertex],
            Some(frame),
            &part.surface,
            tol,
        )
        .ok()?;
        let at = junction(&split)?;
        (Some(split), at)
    };
    // The rim meets the boundary nowhere else: every other edge of the
    // sub-face, outer or inner, that does not end at the vertex stays clear
    // of the rim's circle. An edge whose distance cannot be read exactly
    // counts as touching.
    let clear = |boundary: &OrientedPCurveEdge| {
        (boundary.start_3d - vertex).length() < weld
            || (boundary.end_3d - vertex).length() < weld
            || super::distance_to_boundary_edge(center, boundary).is_some_and(|d| d - radius > weld)
    };
    let other_holes = part
        .inner_wires
        .iter()
        .enumerate()
        .filter(|&(other, _)| other != index)
        .flat_map(|(_, hole)| hole.iter());
    if !split
        .as_deref()
        .unwrap_or(outer)
        .iter()
        .chain(other_holes)
        .all(clear)
    {
        return None;
    }
    let rim_area = signed_area_2d(&sample_wire_loop_uv_via_frame_closed(
        std::slice::from_ref(edge),
        frame,
        RIM_SAMPLES,
    ));
    if !rim_area.is_finite() || rim_area == 0.0 {
        return None;
    }
    Some(Join {
        split,
        at,
        reverse: rim_area * outer_area > 0.0,
    })
}

/// Move each hole of `parts` that is one closed circular section of
/// `sections`, touching the sub-face's outer wire at its own vertex and its
/// boundary nowhere else, into that outer wire at the vertex.
///
/// The outer wire can reach here unsplit, so the edge the vertex lies on is
/// split at it first, as the general path splits a boundary at every section
/// end. The rim is wound against the outline, both read from the 3D curves
/// through the face's frame, so the region stays on the same side of every
/// edge. A sub-face without a closed section among its holes is not read
/// past one point comparison per hole.
pub(super) fn join_touching_rims(
    topo: &Topology,
    face_id: FaceId,
    parts: &mut [SplitSubFace],
    sections: &[SectionEdge],
    tol: f64,
) {
    if !sections
        .iter()
        .any(|section| matches!(section.curve_3d, EdgeCurve::Circle(_)))
    {
        return;
    }
    let mut frame: Option<PlaneFrame> = None;
    for part in parts {
        let FaceSurface::Plane { normal, .. } = &part.surface else {
            continue;
        };
        if !part
            .inner_wires
            .iter()
            .any(|hole| matches!(hole.as_slice(), [edge] if matches!(edge.curve_3d, EdgeCurve::Circle(_))))
        {
            continue;
        }
        if frame.is_none() {
            let Ok(face) = topo.face(face_id) else {
                return;
            };
            frame = Some(PlaneFrame::from_plane_face(
                *normal,
                &super::collect_wire_points(topo, face.outer_wire()),
            ));
        }
        let Some(frame) = frame.as_ref() else {
            return;
        };
        let weld = super::boundary_weld_band(&part.outer_wire, tol);
        let mut index = 0;
        while index < part.inner_wires.len() {
            if !is_new_rim(&part.inner_wires[index], sections, weld) {
                index += 1;
                continue;
            }
            let outer_area =
                signed_area_2d(&sample_wire_loop_uv_via_frame(&part.outer_wire, frame));
            let join = (outer_area.is_finite() && outer_area != 0.0)
                .then(|| plan_join(part, index, frame, outer_area, tol, weld))
                .flatten();
            let Some(join) = join else {
                index += 1;
                continue;
            };
            let mut rim = part.inner_wires.remove(index);
            if join.reverse {
                for edge in &mut rim {
                    std::mem::swap(&mut edge.start_uv, &mut edge.end_uv);
                    std::mem::swap(&mut edge.start_3d, &mut edge.end_3d);
                    edge.forward = !edge.forward;
                }
            }
            if let Some(split) = join.split {
                part.outer_wire = split;
            }
            part.outer_wire.splice(join.at..join.at, rim);
        }
    }
}
