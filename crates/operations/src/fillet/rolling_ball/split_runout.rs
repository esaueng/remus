//! A lone straight stripe ending at a strictly convex planar corner.
//!
//! When every face at the stripe's end vertex is a plane and every edge
//! there is convex, the material near the vertex is the intersection of
//! those planes' half-spaces, so the band is the rolling-ball cylinder cut
//! by them and each contact line runs until the first side plane it meets —
//! wherever along its line that is, before the vertex's own section or past
//! it. Two cells:
//!
//! - **Three planes** (`f1`, `f2` and one side `P`): both contacts run out on
//!   `P`. The pre-pass already does this when both reach `P` within the
//!   edge's own span; [`qualify_single`] covers the contacts that reach it
//!   beyond the vertex — a chamfer's upper contact line, whose chamfer
//!   contact runs on to the mitre, or a fillet of the mitre, whose contacts
//!   run on to the top.
//! - **Four planes** ([`qualify`]): a chamfered box's mitred corner bounds the
//!   chamfer's lower contact line by the stripe's own supports `f1` (wall)
//!   and `f2` (chamfer), the neighbouring wall `Pa` (meeting `f1`) and the
//!   neighbouring chamfer `Pb` (meeting `f2` along the mitre), `Pa` and `Pb`
//!   meeting along a third edge `Ek`. Contact 1 runs out on `Pa`, contact 2
//!   on `Pb`, and the end boundary is the cylinder's section by `Pa` up to
//!   the point `kink` where the cylinder crosses `Ek`, then its section by
//!   `Pb`. Each side face trades the vertex for its two points of that
//!   boundary. The mitre's own lower end, bounded by the two walls, is the
//!   same cell.
//!
//! Anything else returns `None` and the caller keeps its previous handling.

use remus_math::compact_sort::CompactSort;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::FaceId;

use super::{FacePolygon, HashMap, intersect_contact_line_with_plane, on_segment_past_start};
use crate::dot_normal_point;

/// The two-plane end of one stripe at one vertex.
#[derive(Clone, Copy, Debug)]
pub(super) struct SplitRunout {
    /// The selected edge this end belongs to.
    pub(super) edge: usize,
    /// The side faces `[Pa, Pb]` that end contact 1 and contact 2.
    pub(super) faces: [usize; 2],
    /// Their outward planes `(n, d)`, `n · p = d`, `n` unit.
    pub(super) planes: [(Vec3, f64); 2],
    /// The stripe's end contacts on `f1 ∩ Pa` and `f2 ∩ Pb`.
    pub(super) contacts: [Point3; 2],
    /// Where the stripe's end boundary crosses `Ek = Pa ∩ Pb`.
    pub(super) kink: Point3,
}

impl SplitRunout {
    /// The two points side face `face` takes in place of the vertex at
    /// `pos`, ordered along its wire (`prev_pos → pos → next_pos`); `None`
    /// when `face` is not one of this end's side faces.
    pub(super) fn side_corner(
        &self,
        face: usize,
        pos: Point3,
        prev_pos: Point3,
        next_pos: Point3,
        tol: Tolerance,
    ) -> Option<Result<[Point3; 2], crate::OperationsError>> {
        let contact = if face == self.faces[0] {
            self.contacts[0]
        } else if face == self.faces[1] {
            self.contacts[1]
        } else {
            return None;
        };
        let on = |p: Point3, toward: Point3| on_segment_past_start(p, pos, toward, tol);
        Some(if on(contact, prev_pos) && on(self.kink, next_pos) {
            Ok([contact, self.kink])
        } else if on(self.kink, prev_pos) && on(contact, next_pos) {
            Ok([self.kink, contact])
        } else {
            Err(crate::OperationsError::InvalidInput {
                reason: "two-plane fillet runout points do not lie on the side face's corner edges"
                    .into(),
            })
        })
    }
}

/// One edge at the vertex: unit direction away from the vertex, length, and
/// the two faces it separates.
struct CornerEdge {
    id: usize,
    dir: Vec3,
    len: f64,
    faces: [usize; 2],
}

/// The planar faces and straight edges around one vertex, qualified as a
/// strictly convex cone.
struct Cone {
    vertex: Point3,
    /// `(face, outward unit normal, d)`.
    planes: Vec<(usize, Vec3, f64)>,
    edges: Vec<CornerEdge>,
}

impl Cone {
    fn plane(&self, face: usize) -> Option<(Vec3, f64)> {
        self.planes
            .iter()
            .find(|(id, ..)| *id == face)
            .map(|&(_, n, d)| (n, d))
    }

    fn edge_between(&self, a: usize, b: usize) -> Option<&CornerEdge> {
        (a != b)
            .then(|| {
                self.edges
                    .iter()
                    .find(|e| e.faces.contains(&a) && e.faces.contains(&b))
            })
            .flatten()
    }

    /// The edge other than `not` on `face`, and the face across it.
    fn other_edge(&self, face: usize, not: usize) -> Option<(&CornerEdge, usize)> {
        let edge = self
            .edges
            .iter()
            .find(|e| e.id != not && e.faces.contains(&face))?;
        let across = if edge.faces[0] == face {
            edge.faces[1]
        } else {
            edge.faces[0]
        };
        Some((edge, across))
    }
}

/// Every face at the vertex planar and visited once, every edge straight,
/// `count` of each, and the cone strictly convex: each edge leaves the vertex
/// strictly on the material side of every plane not containing it, and lies
/// in the two that do. The material near the vertex is then exactly the
/// intersection of the planes' half-spaces.
fn convex_cone(
    topo: &Topology,
    vertex_index: usize,
    count: usize,
    vertex_faces: &HashMap<usize, Vec<FaceId>>,
    edge_to_faces: &HashMap<usize, Vec<FaceId>>,
    face_polygons: &HashMap<usize, FacePolygon>,
) -> Result<Option<Cone>, crate::OperationsError> {
    let mut faces: Vec<usize> = vertex_faces
        .get(&vertex_index)
        .map_or(&[][..], Vec::as_slice)
        .iter()
        .map(|f| f.index())
        .collect();
    let listed = faces.len();
    faces.compact_sort_by_key(|f| *f);
    faces.dedup();
    // A face touching the vertex twice is a pinched corner, not a cone.
    if faces.len() != count || listed != count {
        return Ok(None);
    }
    let mut planes = Vec::with_capacity(count);
    let mut edges: Vec<CornerEdge> = Vec::with_capacity(count);
    let mut vertex: Option<Point3> = None;
    for &f in &faces {
        let Some(poly) = face_polygons.get(&f) else {
            return Ok(None);
        };
        let length = poly.normal.length();
        let Ok(normal) = poly.normal.normalize() else {
            return Ok(None);
        };
        planes.push((f, normal, poly.d / length));
        let n = poly.vertex_ids.len();
        let at: Vec<usize> = (0..n)
            .filter(|&i| poly.vertex_ids[i].index() == vertex_index)
            .collect();
        let &[i] = at.as_slice() else {
            return Ok(None);
        };
        let pos = poly.positions[i];
        vertex = Some(pos);
        let prev = (i + n - 1) % n;
        let next = (i + 1) % n;
        for (edge_id, far) in [
            (poly.wire_edge_ids[prev], poly.positions[prev]),
            (poly.wire_edge_ids[i], poly.positions[next]),
        ] {
            if edges.iter().any(|e| e.id == edge_id.index()) {
                continue;
            }
            if !matches!(topo.edge(edge_id)?.curve(), EdgeCurve::Line) {
                return Ok(None);
            }
            let Some(&[a, b]) = edge_to_faces.get(&edge_id.index()).map(Vec::as_slice) else {
                return Ok(None);
            };
            let Ok(dir) = (far - pos).normalize() else {
                return Ok(None);
            };
            edges.push(CornerEdge {
                id: edge_id.index(),
                dir,
                len: (far - pos).length(),
                faces: [a.index(), b.index()],
            });
        }
    }
    let Some(vertex) = vertex else {
        return Ok(None);
    };
    if edges.len() != count {
        return Ok(None);
    }
    // Unit normals against unit directions: a dimensionless slack.
    let slack = 1e-9;
    for e in &edges {
        for &(f, normal, _) in &planes {
            let along = normal.dot(e.dir);
            if e.faces.contains(&f) {
                if along.abs() > slack {
                    return Ok(None);
                }
            } else if along >= -slack {
                return Ok(None);
            }
        }
    }
    Ok(Some(Cone {
        vertex,
        planes,
        edges,
    }))
}

/// Run a contact line along the stripe (`reach`, the whole edge from the
/// vertex) to `plane`, and accept the point only if it lies on `side_edge`,
/// strictly between the vertex and that edge's far end, and short of the
/// stripe's far end.
fn run_out(
    vertex: Point3,
    contact: Point3,
    reach: Vec3,
    plane: (Vec3, f64),
    side_edge: &CornerEdge,
    tol: Tolerance,
) -> Option<Point3> {
    let (end, delta) = intersect_contact_line_with_plane(contact, reach, plane, tol)?;
    let along = (end - vertex).dot(side_edge.dir);
    let eps = tol.linear * 100.0;
    (delta.abs() < 1.0
        && along > eps
        && along < side_edge.len - eps
        && (vertex + side_edge.dir * along - end).length() <= eps)
        .then_some(end)
}

/// The three-plane end of the lone straight convex stripe `edge` at vertex
/// `vertex_index`: both contacts (`contacts`, on `f1`, `f2` at the vertex
/// section) run out on the one side plane, wherever they meet it. `None`
/// outside the cell.
#[allow(clippy::too_many_arguments)]
pub(super) fn qualify_single(
    topo: &Topology,
    vertex_index: usize,
    edge: EdgeId,
    supports: [FaceId; 2],
    contacts: [Point3; 2],
    vertex_faces: &HashMap<usize, Vec<FaceId>>,
    edge_to_faces: &HashMap<usize, Vec<FaceId>>,
    face_polygons: &HashMap<usize, FacePolygon>,
    tol: Tolerance,
) -> Result<Option<[Point3; 2]>, crate::OperationsError> {
    let [f1, f2] = [supports[0].index(), supports[1].index()];
    let Some(cone) = convex_cone(
        topo,
        vertex_index,
        3,
        vertex_faces,
        edge_to_faces,
        face_polygons,
    )?
    else {
        return Ok(None);
    };
    let Some(stripe) = cone.edge_between(f1, f2).filter(|e| e.id == edge.index()) else {
        return Ok(None);
    };
    let (Some((edge_a, side_a)), Some((edge_b, side_b))) = (
        cone.other_edge(f1, stripe.id),
        cone.other_edge(f2, stripe.id),
    ) else {
        return Ok(None);
    };
    let Some(plane) = cone.plane(side_a).filter(|_| side_a == side_b) else {
        return Ok(None);
    };
    let reach = stripe.dir * stripe.len;
    let (Some(c1), Some(c2)) = (
        run_out(cone.vertex, contacts[0], reach, plane, edge_a, tol),
        run_out(cone.vertex, contacts[1], reach, plane, edge_b, tol),
    ) else {
        return Ok(None);
    };
    Ok(Some([c1, c2]))
}

/// The four-plane end of the lone straight convex stripe `edge` at vertex
/// `vertex_index`, given its pre-pass contacts `contacts` (on `f1`, `f2`) at
/// the vertex section. `None` outside the cell.
#[allow(clippy::too_many_arguments)]
pub(super) fn qualify(
    topo: &Topology,
    vertex_index: usize,
    edge: EdgeId,
    supports: [FaceId; 2],
    contacts: [Point3; 2],
    vertex_faces: &HashMap<usize, Vec<FaceId>>,
    edge_to_faces: &HashMap<usize, Vec<FaceId>>,
    face_polygons: &HashMap<usize, FacePolygon>,
    radius: f64,
    tol: Tolerance,
) -> Result<Option<SplitRunout>, crate::OperationsError> {
    let [f1, f2] = [supports[0].index(), supports[1].index()];
    let Some(cone) = convex_cone(
        topo,
        vertex_index,
        4,
        vertex_faces,
        edge_to_faces,
        face_polygons,
    )?
    else {
        return Ok(None);
    };
    let vertex = cone.vertex;
    let Some(stripe) = cone.edge_between(f1, f2).filter(|e| e.id == edge.index()) else {
        return Ok(None);
    };
    let (Some((edge_a, pa)), Some((edge_b, pb))) = (
        cone.other_edge(f1, stripe.id),
        cone.other_edge(f2, stripe.id),
    ) else {
        return Ok(None);
    };
    if pa == pb || [pa, pb].iter().any(|p| *p == f1 || *p == f2) {
        return Ok(None);
    }
    let (Some(edge_k), Some(plane_a), Some(plane_b), Some(plane_1)) = (
        cone.edge_between(pa, pb),
        cone.plane(pa),
        cone.plane(pb),
        cone.plane(f1),
    ) else {
        return Ok(None);
    };

    // Each contact meets its own side plane on that side's edge; convexity
    // puts that point inside the other side's half-space, checked anyway.
    let inside =
        |p: Point3, plane: (Vec3, f64)| dot_normal_point(plane.0, p) - plane.1 <= tol.linear;
    let reach = stripe.dir * stripe.len;
    let (Some(c1), Some(c2)) = (
        run_out(vertex, contacts[0], reach, plane_a, edge_a, tol),
        run_out(vertex, contacts[1], reach, plane_b, edge_b, tol),
    ) else {
        return Ok(None);
    };
    if !inside(c1, plane_b) || !inside(c2, plane_a) {
        return Ok(None);
    }

    // The ball's axis, through the vertex section where both pre-pass
    // contacts lie: one radius in from contact 1 along f1's inward normal,
    // and one radius from contact 2.
    let centre = contacts[0] - plane_1.0 * radius;
    if ((contacts[1] - centre).length() - radius).abs() > tol.linear * 100.0 {
        return Ok(None);
    }
    let axis = stripe.dir;
    let radial = |p: Point3| {
        let w = p - centre;
        w - axis * w.dot(axis)
    };

    // The kink: the nearer root where `Ek` enters the cylinder. The vertex
    // is outside it (`qc > 0`), so both roots share a sign.
    let w = radial(vertex);
    let d = edge_k.dir - axis * edge_k.dir.dot(axis);
    let qa = d.dot(d);
    let qb = w.dot(d);
    let qc = w.dot(w) - radius * radius;
    let disc = qb * qb - qa * qc;
    if qa <= tol.angular || qc <= 0.0 || disc <= 0.0 {
        return Ok(None);
    }
    let q = -(qb + qb.signum() * disc.sqrt());
    let mut roots = [q / qa, qc / q];
    roots.compact_sort_by(f64::total_cmp);
    let eps = tol.linear * 100.0;
    let Some(s) = roots.into_iter().find(|s| *s > eps) else {
        return Ok(None);
    };
    if s >= edge_k.len - eps {
        return Ok(None);
    }
    let kink = vertex + edge_k.dir * s;

    // It must lie on the band's own arc, strictly between the contacts.
    let angle = |a: Vec3, b: Vec3| {
        let (Ok(a), Ok(b)) = (a.normalize(), b.normalize()) else {
            return None;
        };
        Some(a.cross(b).length().atan2(a.dot(b)))
    };
    let (Some(whole), Some(first), Some(second)) = (
        angle(radial(c1), radial(c2)),
        angle(radial(c1), radial(kink)),
        angle(radial(kink), radial(c2)),
    ) else {
        return Ok(None);
    };
    if first <= tol.angular || second <= tol.angular || (first + second - whole).abs() > 1e-9 {
        return Ok(None);
    }

    Ok(Some(SplitRunout {
        edge: edge.index(),
        faces: [pa, pb],
        planes: [plane_a, plane_b],
        contacts: [c1, c2],
        kink,
    }))
}
