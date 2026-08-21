//! Shared loft/sweep/pipe/revolve end-cap construction.
//!
//! A swept/lofted/revolved end is closed by filling its section *boundary* — a
//! ring of chord edges — rather than by reusing the section's own surface.
//! remus tessellates and integrates a non-planar face over its full u/v
//! extent rather than clipping to a chord-polygon wire, so a reused parent
//! surface would overfill past the section. Instead: a planar ring gets an exact
//! `Plane` cap (the planar tessellator clips it to the polygon), and a
//! non-planar 4-sided ring is filled by a bilinear patch whose boundary
//! iso-curves are exactly the ring chords (`domain == wire`, so it can't
//! overfill).

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::VertexId;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use crate::dot_normal_point;

/// Collect the 3D positions of a ring of vertices (a cap's outer boundary).
///
/// # Errors
///
/// Returns an error if a vertex id is missing from the arena.
pub fn ring_point_positions(
    topo: &Topology,
    ring: &[VertexId],
) -> Result<Vec<Point3>, crate::OperationsError> {
    ring.iter()
        .map(|&vid| -> Result<Point3, crate::OperationsError> { Ok(topo.vertex(vid)?.point()) })
        .collect()
}

/// A cap ring whose vertices deviate from their best-fit plane by less than this
/// fraction of the ring's size is treated as planar (capped by an exact
/// `Plane`); a larger deviation is filled by a bilinear patch.
const CAP_PLANARITY_TOL: f64 = 1e-6;

/// Characteristic size of a cap ring: the largest distance from its first
/// vertex to any other, used to scale the planarity test.
fn ring_scale(verts: &[Point3]) -> f64 {
    let c = verts[0];
    verts.iter().map(|p| (*p - c).length()).fold(0.0, f64::max)
}

/// Whether the ring lies (within tolerance) in the plane through `cap_verts[0]`
/// with normal `outward`.
fn ring_is_planar(cap_verts: &[Point3], outward: Vec3) -> bool {
    let plane_pt = cap_verts[0];
    let max_dev = cap_verts
        .iter()
        .map(|p| (*p - plane_pt).dot(outward).abs())
        .fold(0.0, f64::max);
    max_dev <= CAP_PLANARITY_TOL * ring_scale(cap_verts)
}

/// Bilinear (degree-1) NURBS patch through a 4-corner ring, in ring order.
///
/// Its four boundary iso-curves are the straight segments between consecutive
/// corners — exactly the ring's chord edges — so the cap shares its boundary
/// with the side faces and tessellates/integrates clipped to the section.
///
/// # Errors
///
/// Returns an error if the four corners cannot define a valid bilinear NURBS
/// patch.
pub fn bilinear_cap_patch(corners: &[Point3]) -> Result<NurbsSurface, remus_math::MathError> {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![vec![corners[0], corners[1]], vec![corners[3], corners[2]]],
        vec![vec![1.0, 1.0], vec![1.0, 1.0]],
    )
}

/// Bilinear (4 corners) or Coons (5 or more) patch through a non-planar
/// ring, exact on the ring's chords.
///
/// # Errors
///
/// Returns an error if the ring cannot form a valid patch.
pub(crate) fn nonplanar_ring_surface(
    cap_verts: &[Point3],
) -> Result<NurbsSurface, crate::OperationsError> {
    if cap_verts.len() == 4 {
        bilinear_cap_patch(cap_verts).map_err(crate::OperationsError::Math)
    } else {
        coons_cap_patch(cap_verts)
    }
}

/// Coons patch through an n-corner ring (n ≥ 5), in ring order.
///
/// The ring is split into four chord chains at quarter points; opposite
/// chains are refined to a common breakpoint count by splitting their
/// longest segments at midpoints — collinear insertions, so the boundary
/// image stays exactly the ring's chords — and the four polylines are
/// blended by [`crate::fill_face::coons_surface`]. Every boundary iso-curve
/// of the result is exactly a run of ring chords, so the cap shares its
/// boundary with the side faces.
///
/// # Errors
///
/// Returns an error if the ring cannot form a valid Coons net.
fn coons_cap_patch(cap_verts: &[Point3]) -> Result<NurbsSurface, crate::OperationsError> {
    let n = cap_verts.len();
    let i1 = n.div_ceil(4).max(1);
    let i2 = (n / 2).max(i1 + 1);
    let i3 = (3 * n / 4).max(i2 + 1);
    if i3 >= n {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("cap ring with {n} edges cannot be split into four chains"),
        });
    }

    let bottom: Vec<Point3> = cap_verts[0..=i1].to_vec();
    let right: Vec<Point3> = cap_verts[i1..=i2].to_vec();
    let mut top: Vec<Point3> = cap_verts[i2..=i3].to_vec();
    top.reverse();
    let mut left: Vec<Point3> = cap_verts[i3..].to_vec();
    left.push(cap_verts[0]);
    left.reverse();

    let m_u = bottom.len().max(top.len());
    let m_v = right.len().max(left.len());
    let bottom = refine_polyline(bottom, m_u);
    let top = refine_polyline(top, m_u);
    let right = refine_polyline(right, m_v);
    let left = refine_polyline(left, m_v);

    crate::fill_face::coons_surface(&bottom, &right, &top, &left)
}

/// Insert midpoints of the longest segments until the polyline has `target`
/// points. Insertions are collinear, so the polyline's image is unchanged.
fn refine_polyline(mut pts: Vec<Point3>, target: usize) -> Vec<Point3> {
    while pts.len() < target {
        let mut best = 0;
        let mut best_len = -1.0;
        for i in 0..pts.len() - 1 {
            let len = (pts[i + 1] - pts[i]).length();
            if len > best_len {
                best_len = len;
                best = i;
            }
        }
        let a = pts[best];
        let b = pts[best + 1];
        let mid = Point3::new(
            f64::midpoint(a.x(), b.x()),
            f64::midpoint(a.y(), b.y()),
            f64::midpoint(a.z(), b.z()),
        );
        pts.insert(best + 1, mid);
    }
    pts
}

/// The ring's outward cap normal: its Newell normal, flipped to agree with
/// `toward` (the side the cap should face, e.g. away from the swept body).
///
/// # Errors
///
/// Returns an error if the ring is degenerate (zero-area Newell normal).
pub fn outward_normal(verts: &[Point3], toward: Vec3) -> Result<Vec3, crate::OperationsError> {
    let n = crate::winding::newell_normal(verts).normalize()?;
    Ok(if n.dot(toward) < 0.0 { -n } else { n })
}

/// Build one end-cap face that fills the ring boundary (and any holes).
///
/// `outer_ring_edges` is the section's outer boundary in ring order;
/// `inner_wires` are pre-built hole loops (empty for hole-free sections, and
/// only supported on a planar cap). `outward` is the section's outward normal;
/// `start_role` builds the reversed-ring wire so the cap faces away from the
/// body.
///
/// A planar ring → exact `Plane` cap. A non-planar 4-sided hole-free ring →
/// bilinear patch. A non-planar ring with more than four edges, or with holes,
/// is unsupported.
///
/// # Errors
///
/// Returns an error if the wire is invalid, the bilinear patch cannot be built,
/// or the ring is an unsupported non-planar shape.
pub fn build_cap_face(
    topo: &mut Topology,
    outer_ring_edges: &[EdgeId],
    inner_wires: Vec<WireId>,
    cap_verts: &[Point3],
    outward: Vec3,
    start_role: bool,
) -> Result<FaceId, crate::OperationsError> {
    let n = outer_ring_edges.len();
    // `cap_verts` must be one position per ring edge (and a ring needs ≥ 3) —
    // later indexing (`cap_verts[0]`, the 4 bilinear corners) relies on this.
    if n < 3 || cap_verts.len() != n {
        return Err(crate::OperationsError::InvalidInput {
            reason: "cap ring must have at least 3 vertices matching its edge count".into(),
        });
    }
    // The two role patterns are each other's exact reversals; `flip` selects
    // the opposite pattern when the face will carry is_reversed=true, so the
    // EFFECTIVE traversal (is_forward XOR is_reversed) stays opposed to the
    // side walls (an end cap built reversed with the unreversed wire traversed
    // its ring in the same effective sense as the walls).
    let ring_wire = |topo: &mut Topology, flip: bool| -> Result<_, crate::OperationsError> {
        let edges: Vec<OrientedEdge> = if start_role == flip {
            (0..n)
                .map(|i| OrientedEdge::new(outer_ring_edges[i], true))
                .collect()
        } else {
            (0..n)
                .rev()
                .map(|i| OrientedEdge::new(outer_ring_edges[i], false))
                .collect()
        };
        Ok(topo.add_wire(Wire::new(edges, true).map_err(crate::OperationsError::Topology)?))
    };

    if ring_is_planar(cap_verts, outward) {
        let wid = ring_wire(topo, false)?;
        let surface = FaceSurface::Plane {
            normal: outward,
            d: dot_normal_point(outward, cap_verts[0]),
        };
        return Ok(topo.add_face(Face::new(wid, inner_wires, surface)));
    }

    if !inner_wires.is_empty() {
        return Err(crate::OperationsError::InvalidInput {
            reason: "cap with holes on a non-planar section boundary is not supported".into(),
        });
    }

    // 4-sided: single bilinear span. 5-or-more-sided: Coons patch of the
    // ring's chord chains — its boundary iso-curves are exactly the ring
    // chords, so like the bilinear case it cannot overfill past the section.
    // (A 3-ring of chords always lies in a plane and took the branch above.)
    let surf = if n == 4 {
        bilinear_cap_patch(cap_verts).map_err(crate::OperationsError::Math)?
    } else {
        coons_cap_patch(cap_verts)?
    };
    // A near-flat bilinear lid: its center normal is stable and aligned with the
    // ring axis, so probe there and flip if it opposes `outward`.
    let reversed = surf
        .normal(0.5, 0.5)
        .map(|nrm| nrm.dot(outward) < 0.0)
        .unwrap_or(false);
    let wid = ring_wire(topo, reversed)?;
    let mut face = Face::new(wid, vec![], FaceSurface::Nurbs(surf));
    if reversed {
        face.set_reversed(true);
    }
    Ok(topo.add_face(face))
}
