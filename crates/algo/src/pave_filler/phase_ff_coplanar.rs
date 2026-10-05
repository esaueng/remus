//! Phase FF-Coplanar: coplanar face splitting.
//!
//! Handles the case where two faces from different solids lie on the same
//! plane and partially overlap. Phase FF skips these because parallel planes
//! have no intersection line. This phase runs after FF and creates section
//! edges by projecting one face's boundary edges into the other face's
//! interior.

use remus_math::aabb::Aabb3;
use remus_math::det_hash::DetHashSet;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;

use crate::ds::{GfaArena, Interference, IntersectionCurveDS, Pave, PaveBlock, PaveBlockId};
use crate::error::AlgoError;

use super::helpers::find_nearby_pave_vertex;

/// Detect coplanar face pairs between two solids and create section edges
/// for boundary edges of one face that lie inside the other.
///
/// # Errors
///
/// Returns [`AlgoError`] if any topology lookup fails.
#[allow(clippy::too_many_lines)]
pub fn perform(
    topo: &mut Topology,
    solid_a: SolidId,
    solid_b: SolidId,
    tol: Tolerance,
    arena: &mut GfaArena,
) -> Result<(), AlgoError> {
    let faces_a = remus_topology::explorer::solid_faces(topo, solid_a)?;
    let faces_b = remus_topology::explorer::solid_faces(topo, solid_b)?;

    let qualified_splines = ruled_profile_edges(topo, faces_a.iter().chain(&faces_b).copied())?;
    let planes_a = collect_plane_faces(topo, &faces_a)?;
    let planes_b = collect_plane_faces(topo, &faces_b)?;

    if planes_a.is_empty() || planes_b.is_empty() {
        return Ok(());
    }

    let bboxes_a = compute_face_bboxes(topo, &planes_a)?;
    let bboxes_b = compute_face_bboxes(topo, &planes_b)?;

    log::debug!(
        "FF-coplanar: checking {} × {} plane face pairs",
        planes_a.len(),
        planes_b.len()
    );

    for (idx_a, &(fa, na, da)) in planes_a.iter().enumerate() {
        let bbox_a = &bboxes_a[idx_a];

        for (idx_b, &(fb, nb, db)) in planes_b.iter().enumerate() {
            let bbox_b = &bboxes_b[idx_b];

            let dot = na.dot(nb);
            if dot.abs() < 1.0 - tol.angular {
                continue;
            }

            // Coplanar test accounts for normal direction: anti-parallel
            // normals describe the same plane when da == -db.
            let sign = if dot > 0.0 { 1.0 } else { -1.0 };
            if (da - db * sign).abs() > tol.linear {
                continue;
            }

            if !bbox_a
                .expanded(tol.linear)
                .intersects(bbox_b.expanded(tol.linear))
            {
                continue;
            }

            if has_existing_ff_interference(arena, fa, fb) {
                continue;
            }

            process_coplanar_pair(topo, fa, na, fb, tol, arena, &qualified_splines)?;
        }
    }

    Ok(())
}

/// Strict whole-span witnesses belong to the single-span Bezier ruled-profile cell.
/// Fitted edges from general surface intersections use the established dispatch.
#[allow(clippy::float_cmp)] // Degree, knot, weight and coefficient identities qualify the cell.
fn ruled_profile_edges(
    topo: &Topology,
    faces: impl Iterator<Item = FaceId>,
) -> Result<DetHashSet<remus_topology::edge::EdgeId>, AlgoError> {
    let mut qualified = DetHashSet::default();
    for fid in faces {
        let FaceSurface::Nurbs(surface) = topo.face(fid)?.surface() else {
            continue;
        };
        let points = surface.control_points();
        let weights = surface.weights();
        let knots = surface.knots_u();
        if surface.degree_u() != 1
            || points.len() != 2
            || points[0].len() != surface.degree_v() + 1
            || weights[0] != weights[1]
            || knots.len() != 4
            || knots[0] != knots[1]
            || knots[2] != knots[3]
        {
            continue;
        }
        for eid in remus_topology::explorer::face_edges(topo, fid)? {
            let EdgeCurve::NurbsCurve(curve) = topo.edge(eid)?.curve() else {
                continue;
            };
            let matches_row = |curve: &remus_math::nurbs::curve::NurbsCurve| {
                curve.degree() == surface.degree_v()
                    && curve.knots() == surface.knots_v()
                    && (0..2).any(|row| {
                        curve.control_points() == points[row] && curve.weights() == weights[row]
                    })
            };
            if matches_row(curve) || matches_row(&curve.reversed()) {
                qualified.insert(eid);
            }
        }
    }
    Ok(qualified)
}

/// Collect `(FaceId, normal, d)` for all plane faces in the list.
fn collect_plane_faces(
    topo: &Topology,
    faces: &[FaceId],
) -> Result<Vec<(FaceId, Vec3, f64)>, AlgoError> {
    let mut result = Vec::new();
    for &fid in faces {
        let face = topo.face(fid)?;
        if let FaceSurface::Plane { normal, d } = face.surface() {
            result.push((fid, *normal, *d));
        }
    }
    Ok(result)
}

/// Compute AABBs for plane faces by sampling boundary edges.
fn compute_face_bboxes(
    topo: &Topology,
    planes: &[(FaceId, Vec3, f64)],
) -> Result<Vec<Aabb3>, AlgoError> {
    let mut bboxes = Vec::with_capacity(planes.len());
    for &(fid, _, _) in planes {
        bboxes.push(compute_face_bbox(topo, fid)?);
    }
    Ok(bboxes)
}

/// Compute AABB for a face by sampling its boundary edges.
fn compute_face_bbox(topo: &Topology, face_id: FaceId) -> Result<Aabb3, AlgoError> {
    let edges = remus_topology::explorer::face_edges(topo, face_id)?;
    let mut points = Vec::new();

    for eid in edges {
        let edge = topo.edge(eid)?;
        let start_pos = topo.vertex(edge.start())?.point();
        let end_pos = topo.vertex(edge.end())?.point();
        let (t0, t1) =
            super::helpers::authoritative_edge_domain(edge, eid, "coplanar face bounding box")?;

        let n: usize = 8;
        for i in 0..=n {
            let t = t0 + (t1 - t0) * (i as f64 / n as f64);
            let pt = edge.curve().evaluate_with_endpoints(t, start_pos, end_pos);
            points.push(pt);
        }
    }

    if points.is_empty() {
        Ok(Aabb3 {
            min: Point3::new(0.0, 0.0, 0.0),
            max: Point3::new(0.0, 0.0, 0.0),
        })
    } else {
        Ok(Aabb3::from_points(points))
    }
}

/// Check if a section curve already exists at this position for either face.
///
/// Searches `arena.curves` for any existing intersection curve involving
/// `face_a` or `face_b` whose endpoints match `p_start`/`p_end` within
/// tolerance. This prevents the coplanar phase from creating duplicate
/// section edges that already exist from the regular FF phase.
fn has_existing_section_at(
    arena: &GfaArena,
    face_a: FaceId,
    face_b: FaceId,
    p_start: Point3,
    p_end: Point3,
    tol: Tolerance,
) -> bool {
    for curve in &arena.curves {
        if curve.face_a != face_a
            && curve.face_a != face_b
            && curve.face_b != face_a
            && curve.face_b != face_b
        {
            continue;
        }

        let edge_min = Point3::new(
            p_start.x().min(p_end.x()),
            p_start.y().min(p_end.y()),
            p_start.z().min(p_end.z()),
        );
        let edge_max = Point3::new(
            p_start.x().max(p_end.x()),
            p_start.y().max(p_end.y()),
            p_start.z().max(p_end.z()),
        );
        let expanded = curve.bbox.expanded(tol.linear);
        if edge_min.x() > expanded.max.x()
            || edge_max.x() < expanded.min.x()
            || edge_min.y() > expanded.max.y()
            || edge_max.y() < expanded.min.y()
            || edge_min.z() > expanded.max.z()
            || edge_max.z() < expanded.min.z()
        {
            continue;
        }

        // Check endpoint match: midpoint of proposed edge must be near the
        // existing curve's midpoint. Use midpoint instead of endpoint to
        // handle reversed-direction curves.
        let mid = Point3::new(
            (p_start.x() + p_end.x()) * 0.5,
            (p_start.y() + p_end.y()) * 0.5,
            (p_start.z() + p_end.z()) * 0.5,
        );
        let curve_mid = Point3::new(
            (curve.bbox.min.x() + curve.bbox.max.x()) * 0.5,
            (curve.bbox.min.y() + curve.bbox.max.y()) * 0.5,
            (curve.bbox.min.z() + curve.bbox.max.z()) * 0.5,
        );
        if (mid - curve_mid).length() < tol.linear * 10.0 {
            return true;
        }
    }
    false
}

/// Check if an FF interference already exists for this face pair.
fn has_existing_ff_interference(arena: &GfaArena, fa: FaceId, fb: FaceId) -> bool {
    arena.interference.ff.iter().any(|interf| {
        matches!(interf,
            Interference::FF { f1, f2, .. } if (*f1 == fa && *f2 == fb) || (*f1 == fb && *f2 == fa)
        )
    })
}

/// Process a single coplanar face pair: project boundary edges of each face
/// into the other and create section edges for edges that lie inside.
#[allow(clippy::too_many_lines)]
fn process_coplanar_pair(
    topo: &mut Topology,
    face_a: FaceId,
    normal: Vec3,
    face_b: FaceId,
    tol: Tolerance,
    arena: &mut GfaArena,
    qualified_splines: &DetHashSet<remus_topology::edge::EdgeId>,
) -> Result<(), AlgoError> {
    let origin = first_wire_vertex(topo, face_a)?;
    let frame = PlaneFrame2D::new(normal, origin);

    let poly_a = face_boundary_polygon_2d(topo, face_a, &frame)?;
    let poly_b = face_boundary_polygon_2d(topo, face_b, &frame)?;

    let edges_a = face_boundary_edges_2d(topo, face_a, &frame)?;
    let edges_b = face_boundary_edges_2d(topo, face_b, &frame)?;

    // For each boundary edge of face_b, create a section for the part inside
    // face_a. Clipping to face_a's polygon lands a straddling edge's endpoint
    // exactly on the boundary, so a faceted chain (e.g. a scoop ramp leaving the
    // cavity wall) reaches the wall edge and the wall partitions; a fully-inside
    // edge is kept whole. Skip true shared-boundary edges (both endpoints on the
    // same target edge) and edges already sectioned by the regular FF phase.
    //
    // A curved boundary edge projects here as its straight CHORD — wrong
    // geometry whenever the sagitta exceeds tolerance. When the true arc is
    // already present as a section (the barrel face sharing that arc meets
    // the coplanar partner plane in exactly this circle, so the regular FF
    // phase emits it split at the same operand vertices), emitting the chord
    // too would hand the splitter a co-endpoint chord/arc lens that the
    // endpoint-keyed edge merge cannot reconcile — the weave then routes the
    // face boundary along the chord and orphans the true arc (the rounded-
    // corner cap defect). Skip the chord when its exact arc section exists.
    // Spline matches require identical coefficients and parameter spans;
    // unresolved spline boundaries refuse instead of becoming chords.
    create_boundary_sections(
        topo,
        arena,
        (face_a, face_b),
        &edges_b,
        CoplanarTarget {
            face: face_a,
            edges: &edges_a,
            polygon: &poly_a,
        },
        tol,
        qualified_splines,
    )?;
    create_boundary_sections(
        topo,
        arena,
        (face_a, face_b),
        &edges_a,
        CoplanarTarget {
            face: face_b,
            edges: &edges_b,
            polygon: &poly_b,
        },
        tol,
        qualified_splines,
    )?;

    // For each boundary edge of face_b that coincides with a boundary edge
    // of face_a (both endpoints on the SAME target edge), create a CommonBlock
    // linking their PaveBlocks. This enables edge sharing for flush-face
    // (touching) booleans where the faces share a boundary segment.
    for &(b_eid, p2d_start, p2d_end, _, _) in &edges_b {
        let start_edge = which_boundary_edge(p2d_start, &edges_a, tol.linear);
        let end_edge = which_boundary_edge(p2d_end, &edges_a, tol.linear);
        if let (Some(si), Some(ei)) = (start_edge, end_edge)
            && si == ei
        {
            let a_eid = edges_a[si].0;
            if (!qualified_splines.contains(&a_eid) && !qualified_splines.contains(&b_eid))
                || spline_pair_compatible(topo, a_eid, b_eid)?
            {
                create_coplanar_common_block(arena, a_eid, b_eid);
            }
        }
    }

    Ok(())
}

/// Both directed coplanar passes share this code, keeping operand-B then
/// operand-A traversal and every exclusion/witness check in the same order.
struct CoplanarTarget<'a> {
    face: FaceId,
    edges: &'a [BoundaryEdge],
    polygon: &'a [Point2],
}

#[cfg_attr(target_arch = "wasm32", inline(never))]
fn create_boundary_sections(
    topo: &mut Topology,
    arena: &mut GfaArena,
    faces: (FaceId, FaceId),
    edges: &[BoundaryEdge],
    target: CoplanarTarget<'_>,
    tol: Tolerance,
    qualified_splines: &DetHashSet<remus_topology::edge::EdgeId>,
) -> Result<(), AlgoError> {
    let (face_a, face_b) = faces;
    for &(eid, p2d_start, p2d_end, p3d_start, p3d_end) in edges {
        if shared_spline_boundary(topo, eid, target.face)?
            || spline_hull_disjoint_from_line_face(topo, eid, target.face, tol)?
            || matching_boundary_section_exists(
                topo,
                arena,
                face_a,
                face_b,
                eid,
                tol,
                qualified_splines.contains(&eid),
            )?
        {
            continue;
        }
        if !is_shared_boundary_edge(p2d_start, p2d_end, target.edges, tol.linear) {
            for (c_start, c_end) in clip_section_to_polygon(
                p2d_start,
                p2d_end,
                p3d_start,
                p3d_end,
                target.polygon,
                tol.linear,
            ) {
                if !has_existing_section_at(arena, face_a, face_b, c_start, c_end, tol) {
                    create_section_edge(topo, arena, face_a, face_b, c_start, c_end, tol)?;
                }
            }
        }
    }
    Ok(())
}

/// Get the first vertex position of a face's outer wire.
fn first_wire_vertex(topo: &Topology, face_id: FaceId) -> Result<Point3, AlgoError> {
    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;
    if let Some(oe) = wire.edges().first() {
        let edge = topo.edge(oe.edge())?;
        Ok(topo.vertex(edge.start())?.point())
    } else {
        Ok(Point3::new(0.0, 0.0, 0.0))
    }
}

/// Collect the outer wire boundary as a 2D polygon.
fn face_boundary_polygon_2d(
    topo: &Topology,
    face_id: FaceId,
    frame: &PlaneFrame2D,
) -> Result<Vec<Point2>, AlgoError> {
    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;
    let mut polygon = Vec::new();

    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        // Use oriented start to respect wire traversal direction.
        let vid = oe.oriented_start(edge);
        let pos = topo.vertex(vid)?.point();
        polygon.push(frame.project(pos));
    }

    Ok(polygon)
}

/// Boundary edge info: `(EdgeId, 2D start, 2D end, 3D start, 3D end)`.
type BoundaryEdge = (remus_topology::edge::EdgeId, Point2, Point2, Point3, Point3);

/// Collect boundary edges with 2D and 3D endpoint positions.
///
/// Respects oriented edge direction so start/end match wire traversal.
fn face_boundary_edges_2d(
    topo: &Topology,
    face_id: FaceId,
    frame: &PlaneFrame2D,
) -> Result<Vec<BoundaryEdge>, AlgoError> {
    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;
    let mut edges = Vec::new();

    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let (p3_start, p3_end) = if oe.is_forward() {
            (
                topo.vertex(edge.start())?.point(),
                topo.vertex(edge.end())?.point(),
            )
        } else {
            (
                topo.vertex(edge.end())?.point(),
                topo.vertex(edge.start())?.point(),
            )
        };
        let p2_start = frame.project(p3_start);
        let p2_end = frame.project(p3_end);
        edges.push((oe.edge(), p2_start, p2_end, p3_start, p3_end));
    }

    Ok(edges)
}

/// A shared spline seam is certified by both boundary carriers and spans.
fn shared_spline_boundary(
    topo: &Topology,
    eid: remus_topology::edge::EdgeId,
    target: FaceId,
) -> Result<bool, AlgoError> {
    if !matches!(topo.edge(eid)?.curve(), EdgeCurve::NurbsCurve(_)) {
        return Ok(false);
    }
    for other in remus_topology::explorer::face_edges(topo, target)? {
        if matches!(topo.edge(other)?.curve(), EdgeCurve::NurbsCurve(_))
            && spline_pair_compatible(topo, eid, other)?
        {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Endpoint matches alone must not join a spline to a chord or another lens.
fn spline_pair_compatible(
    topo: &Topology,
    a: remus_topology::edge::EdgeId,
    b: remus_topology::edge::EdgeId,
) -> Result<bool, AlgoError> {
    let a_edge = topo.edge(a)?;
    let b_edge = topo.edge(b)?;
    match (a_edge.curve(), b_edge.curve()) {
        (EdgeCurve::NurbsCurve(a_curve), EdgeCurve::NurbsCurve(b_curve)) => {
            let a_span =
                super::helpers::authoritative_edge_domain(a_edge, a, "shared spline seam")?;
            let b_span =
                super::helpers::authoritative_edge_domain(b_edge, b, "shared spline seam")?;
            Ok(super::helpers::identical_nurbs_span(
                a_curve, a_span, b_curve, b_span,
            ))
        }
        (
            EdgeCurve::NurbsCurve(_),
            EdgeCurve::Line
            | EdgeCurve::Circle(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_),
        )
        | (
            EdgeCurve::Line
            | EdgeCurve::Circle(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_),
            EdgeCurve::NurbsCurve(_),
        ) => Ok(false),
        (
            EdgeCurve::Line
            | EdgeCurve::Circle(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_),
            EdgeCurve::Line
            | EdgeCurve::Circle(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_),
        ) => Ok(true),
    }
}

/// Exclude an entire positive-weight spline hull from a polygonal partner.
/// A distant spline needs no section witness. Endpoint chords cannot establish
/// this exclusion, and curved partner boundaries retain the refusing path.
fn spline_hull_disjoint_from_line_face(
    topo: &Topology,
    eid: remus_topology::edge::EdgeId,
    target: FaceId,
    tol: Tolerance,
) -> Result<bool, AlgoError> {
    let EdgeCurve::NurbsCurve(curve) = topo.edge(eid)?.curve() else {
        return Ok(false);
    };
    if curve.validate_weights().is_err() {
        return Ok(false);
    }
    let edges = remus_topology::explorer::face_edges(topo, target)?;
    let mut points = Vec::with_capacity(edges.len() * 2);
    for id in edges {
        let edge = topo.edge(id)?;
        if !matches!(edge.curve(), EdgeCurve::Line) {
            return Ok(false);
        }
        points.push(topo.vertex(edge.start())?.point());
        points.push(topo.vertex(edge.end())?.point());
    }
    if points.is_empty() {
        return Ok(false);
    }
    if !curve
        .aabb()
        .expanded(tol.linear)
        .intersects(Aabb3::from_points(points).expanded(tol.linear))
    {
        return Ok(true);
    }
    let FaceSurface::Plane { normal, .. } = topo.face(target)?.surface() else {
        return Ok(false);
    };
    let normal = normal.normalize()?;
    let frame = PlaneFrame2D::new(normal, curve.control_points()[0]);
    let mut min = Point2::new(f64::INFINITY, f64::INFINITY);
    let mut max = Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    for &p in curve.control_points() {
        let q = frame.project(p);
        min = Point2::new(min.x().min(q.x()), min.y().min(q.y()));
        max = Point2::new(max.x().max(q.x()), max.y().max(q.y()));
    }
    min = Point2::new(min.x() - tol.linear, min.y() - tol.linear);
    max = Point2::new(max.x() + tol.linear, max.y() + tol.linear);
    let face = topo.face(target)?;
    let mut polygons = Vec::new();
    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        let wire = topo.wire(wire_id)?;
        let mut polygon = Vec::with_capacity(wire.edges().len());
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            let a = frame.project(topo.vertex(oe.oriented_start(edge))?.point());
            let b = frame.project(topo.vertex(oe.oriented_end(edge))?.point());
            if segment_intersects_box(a, b, min, max) {
                return Ok(false);
            }
            polygon.push(a);
        }
        if polygon.len() < 3 {
            return Ok(false);
        }
        polygons.push(polygon);
    }
    // No boundary crosses the hull box, so its region membership is constant.
    let center = Point2::new(0.5 * (min.x() + max.x()), 0.5 * (min.y() + max.y()));
    Ok(!point_in_line_polygon_exact(center, &polygons[0])
        || polygons[1..]
            .iter()
            .any(|p| point_in_line_polygon_exact(center, p)))
}

/// Separating axes for a segment and the expanded convex hull box.
/// Exact orientation signs avoid rounded slab-division exclusions at corners.
fn segment_intersects_box(a: Point2, b: Point2, min: Point2, max: Point2) -> bool {
    if a.x().max(b.x()) < min.x()
        || a.x().min(b.x()) > max.x()
        || a.y().max(b.y()) < min.y()
        || a.y().min(b.y()) > max.y()
    {
        return false;
    }
    let corners = [
        min,
        Point2::new(max.x(), min.y()),
        max,
        Point2::new(min.x(), max.y()),
    ];
    let signs = corners.map(|p| remus_math::predicates::orient2d(a, b, p));
    !(signs.iter().all(|&s| s > 0.) || signs.iter().all(|&s| s < 0.))
}

fn point_in_line_polygon_exact(point: Point2, polygon: &[Point2]) -> bool {
    let mut inside = false;
    for i in 0..polygon.len() {
        let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
        let sign = remus_math::predicates::orient2d(a, b, point);
        if (a.y() <= point.y() && b.y() > point.y() && sign > 0.)
            || (b.y() <= point.y() && a.y() > point.y() && sign < 0.)
        {
            inside = !inside;
        }
    }
    inside
}

/// True when a boundary's exact curve already exists as a section.
/// Spline identity requires equal coefficients and parameter spans; a missing
/// spline witness refuses because endpoint chords are not exact sections.
/// A Circle boundary edge matches its exact arc as
/// a section curve involving either face of the coplanar pair: same circle
/// (center + radius within tolerance) and same endpoints (either orientation).
///
/// Such an edge's chord projection must NOT become a section — the chord and
/// the true arc share both endpoints, and that lens breaks the downstream
/// wire weave (see the rounded-corner cap comment at the call sites). Line
/// boundary edges always return `false`; a co-endpoint line/arc pair can be a
/// genuine lens with material between the two (the in-tube torus-box case) and
/// must keep both curves.
fn matching_boundary_section_exists(
    topo: &Topology,
    arena: &GfaArena,
    face_a: FaceId,
    face_b: FaceId,
    eid: remus_topology::edge::EdgeId,
    tol: Tolerance,
    require_exact_witness: bool,
) -> Result<bool, AlgoError> {
    let edge = topo.edge(eid)?;
    if let EdgeCurve::NurbsCurve(boundary) = edge.curve() {
        let domain =
            super::helpers::authoritative_edge_domain(edge, eid, "coplanar spline section")?;
        let matched = arena.curves.iter().any(|section| {
            (section.face_a == face_a
                || section.face_a == face_b
                || section.face_b == face_a
                || section.face_b == face_b)
                && matches!(&section.curve, EdgeCurve::NurbsCurve(existing)
                    if super::helpers::identical_nurbs_span(existing, section.t_range, boundary, domain))
        });
        if !matched && require_exact_witness {
            return Err(AlgoError::IntersectionFailed(
                "coplanar spline boundary has no certified whole-span section".into(),
            ));
        }
        return Ok(matched);
    }
    let EdgeCurve::Circle(circle) = edge.curve() else {
        return Ok(false);
    };
    let (sv, ev) = (topo.vertex(edge.start())?, topo.vertex(edge.end())?);
    let (p_start, p_end) = (sv.point(), ev.point());
    // Arc midpoint of the boundary edge, so the COMPLEMENTARY arc between the
    // same two endpoints on the same circle does not count as a match (its
    // chord section would then be wrongly suppressed while the true span has
    // no section at all).
    let (d0, d1) =
        super::helpers::authoritative_edge_domain(edge, eid, "coplanar matching-arc check")?;
    let edge_mid = edge
        .curve()
        .evaluate_with_endpoints(0.5 * (d0 + d1), p_start, p_end);
    let close = |a: Point3, b: Point3| (a - b).length() < tol.linear * 10.0;
    Ok(arena.curves.iter().any(|c| {
        let EdgeCurve::Circle(existing) = &c.curve else {
            return false;
        };
        let shares_face =
            c.face_a == face_a || c.face_a == face_b || c.face_b == face_a || c.face_b == face_b;
        // Same geometric circle: center + radius + carrier plane (normals
        // parallel either way — cross-product test). Same-center same-radius
        // circles in DIFFERENT planes (two great circles of a sphere) can
        // still share two endpoints and must not match.
        if !shares_face
            || (existing.center() - circle.center()).length() > tol.linear * 10.0
            || (existing.radius() - circle.radius()).abs() > tol.linear * 10.0
            || existing.normal().cross(circle.normal()).length() > tol.angular.max(1e-9)
        {
            return false;
        }
        let s = existing.evaluate(c.t_range.0);
        let e = existing.evaluate(c.t_range.1);
        let m = existing.evaluate(0.5 * (c.t_range.0 + c.t_range.1));
        close(m, edge_mid)
            && ((close(s, p_start) && close(e, p_end)) || (close(s, p_end) && close(e, p_start)))
    }))
}

/// True when a boundary edge is a shared boundary segment of the target face:
/// both endpoints lie on the SAME target boundary edge (collinear with it).
/// Such an edge is the faces' common boundary, not a dividing section. An edge
/// whose endpoints sit on DIFFERENT target edges crosses the interior and is a
/// genuine section.
fn is_shared_boundary_edge(
    p2d_start: Point2,
    p2d_end: Point2,
    target_edges: &[BoundaryEdge],
    tol: f64,
) -> bool {
    let start_edge_idx = which_boundary_edge(p2d_start, target_edges, tol);
    let end_edge_idx = which_boundary_edge(p2d_end, target_edges, tol);
    matches!((start_edge_idx, end_edge_idx), (Some(si), Some(ei)) if si == ei)
}

/// Clip a coplanar section edge to the target face polygon, returning the
/// 3D endpoints of each connected sub-segment inside the polygon. Returns
/// an empty list when the whole segment is outside.
///
/// A face-b boundary edge that straddles the target boundary (one endpoint
/// inside the wall, the other outside, e.g. a faceted scoop ramp leaving the
/// cavity wall) would otherwise contribute a section that overshoots the wall
/// or — if its midpoint falls outside — none at all, leaving the section chain
/// dangling at an interior vertex so the wall never splits. Clipping at the
/// boundary crossing lands the chain endpoint exactly on the wall edge, so the
/// face partitions. A fully-inside edge is returned unchanged.
fn clip_section_to_polygon(
    p2d_start: Point2,
    p2d_end: Point2,
    p3d_start: Point3,
    p3d_end: Point3,
    polygon: &[Point2],
    tol: f64,
) -> Vec<(Point3, Point3)> {
    let inside = |p: Point2| point_in_polygon_2d(p, polygon);

    // Parameter(s) along the segment where it crosses a polygon edge.
    let d = Point2::new(p2d_end.x() - p2d_start.x(), p2d_end.y() - p2d_start.y());
    let seg_len = d.x().hypot(d.y());
    if seg_len < tol {
        return Vec::new();
    }
    let mut ts: Vec<f64> = Vec::new();
    let n = polygon.len();
    for i in 0..n {
        let a = polygon[i];
        let b = polygon[(i + 1) % n];
        let e = Point2::new(b.x() - a.x(), b.y() - a.y());
        let denom = d.x() * e.y() - d.y() * e.x();
        if denom.abs() < 1e-15 {
            continue;
        }
        let t = ((a.x() - p2d_start.x()) * e.y() - (a.y() - p2d_start.y()) * e.x()) / denom;
        let u = ((a.x() - p2d_start.x()) * d.y() - (a.y() - p2d_start.y()) * d.x()) / denom;
        if (-1e-9..=1.0 + 1e-9).contains(&t) && (-1e-9..=1.0 + 1e-9).contains(&u) {
            ts.push(t.clamp(0.0, 1.0));
        }
    }
    ts.push(0.0);
    ts.push(1.0);
    ts.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
    ts.dedup_by(|x, y| (*x - *y).abs() < 1e-9);

    // Keep disconnected interior intervals separate. Even when both endpoints
    // are inside, the segment can cross the exterior of a concave face.
    let mut intervals: Vec<(f64, f64)> = Vec::new();
    for w in ts.windows(2) {
        let (ta, tb) = (w[0], w[1]);
        if tb - ta < 1e-9 {
            continue;
        }
        let tm = 0.5 * (ta + tb);
        let mid = Point2::new(p2d_start.x() + d.x() * tm, p2d_start.y() + d.y() * tm);
        if inside(mid) {
            if let Some(last) = intervals.last_mut()
                && (last.1 - ta).abs() < 1e-9
            {
                last.1 = tb;
            } else {
                intervals.push((ta, tb));
            }
        }
    }
    let lerp = |t: f64| -> Point3 {
        Point3::new(
            p3d_start.x() + (p3d_end.x() - p3d_start.x()) * t,
            p3d_start.y() + (p3d_end.y() - p3d_start.y()) * t,
            p3d_start.z() + (p3d_end.z() - p3d_start.z()) * t,
        )
    };
    intervals
        .into_iter()
        .filter(|(ta, tb)| (tb - ta) * seg_len >= tol)
        .map(|(ta, tb)| (lerp(ta), lerp(tb)))
        .collect()
}

/// Create a section edge and register it in the GFA arena.
/// Create a CommonBlock linking leaf PaveBlocks of two coincident boundary edges.
///
/// For flush-face (touching) booleans, A's boundary edge and B's boundary edge
/// overlap at the shared face boundary. Linking their PaveBlocks via a
/// CommonBlock ensures they share the same split edge, enabling
/// `merge_duplicate_edges` to recognize them as the same geometric edge.
fn create_coplanar_common_block(
    arena: &mut GfaArena,
    a_edge: remus_topology::edge::EdgeId,
    b_edge: remus_topology::edge::EdgeId,
) {
    let get_leaves = |edge: remus_topology::edge::EdgeId| -> Vec<PaveBlockId> {
        arena
            .edge_pave_blocks
            .get(&edge)
            .map(|pbs| {
                pbs.iter()
                    .copied()
                    .filter(|&pb_id| {
                        arena
                            .pave_blocks
                            .get(pb_id)
                            .is_some_and(|pb| pb.children.is_empty())
                    })
                    .collect()
            })
            .unwrap_or_default()
    };

    let a_leaves = get_leaves(a_edge);
    let b_leaves = get_leaves(b_edge);

    // For now, handle the simple case: both edges have exactly 1 leaf PB.
    // More complex cases (split edges with multiple children) need position
    // matching to pair the correct leaf PBs.
    if a_leaves.len() == 1 && b_leaves.len() == 1 {
        let a_pb = a_leaves[0];
        let b_pb = b_leaves[0];

        // Skip if both PBs are already in the same CB, or either
        // is in a different CB (merging CBs deferred to Phase 5).
        let a_cb = arena.pb_to_cb.get(&a_pb).copied();
        let b_cb = arena.pb_to_cb.get(&b_pb).copied();
        if (a_cb.is_some() && a_cb == b_cb) || a_cb.is_some() || b_cb.is_some() {
            return;
        }

        arena.create_common_block(vec![a_pb, b_pb]);

        log::debug!("coplanar CommonBlock: edge {a_edge:?} + {b_edge:?} (PBs {a_pb:?} + {b_pb:?})");
    }
}

#[allow(clippy::unnecessary_wraps)]
fn create_section_edge(
    topo: &mut Topology,
    arena: &mut GfaArena,
    face_a: FaceId,
    face_b: FaceId,
    p3d_start: Point3,
    p3d_end: Point3,
    tol: Tolerance,
) -> Result<(), AlgoError> {
    let edge_length = (p3d_end - p3d_start).length();
    if edge_length < tol.linear {
        // Degenerate edge, skip
        return Ok(());
    }

    let start_vid = find_or_create_vertex(topo, arena, p3d_start, tol);
    let end_vid = find_or_create_vertex(topo, arena, p3d_end, tol);

    let edge = Edge::new(start_vid, end_vid, EdgeCurve::Line);
    let edge_id = topo.add_edge(edge);

    // EdgeCurve::Line uses normalized parameter space [0, 1].
    let start_pave = Pave::new(start_vid, 0.0);
    let end_pave = Pave::new(end_vid, 1.0);
    let pb = PaveBlock::new(edge_id, start_pave, end_pave);
    let pb_id = arena.pave_blocks.alloc(pb);

    // Register in edge_pave_blocks so ForceInterfEE can detect overlaps
    // between this section PB and boundary-edge PBs with the same
    // endpoints. This creates CommonBlocks → shared split edges →
    // manifold shell connectivity between coplanar sub-faces.
    arena
        .edge_pave_blocks
        .entry(edge_id)
        .or_default()
        .push(pb_id);

    let bbox = Aabb3 {
        min: Point3::new(
            p3d_start.x().min(p3d_end.x()),
            p3d_start.y().min(p3d_end.y()),
            p3d_start.z().min(p3d_end.z()),
        ),
        max: Point3::new(
            p3d_start.x().max(p3d_end.x()),
            p3d_start.y().max(p3d_end.y()),
            p3d_start.z().max(p3d_end.z()),
        ),
    };

    let curve_index = arena.curves.len();
    arena.curves.push(IntersectionCurveDS {
        curve: EdgeCurve::Line,
        face_a,
        face_b,
        bbox,
        pave_blocks: vec![pb_id],
        t_range: (0.0, 1.0),
    });

    arena.interference.ff.push(Interference::FF {
        f1: face_a,
        f2: face_b,
        curve_index,
    });

    log::debug!(
        "FF-coplanar: faces {face_a:?} and {face_b:?} section edge \
         (curve_index={curve_index}, edge={edge_id:?}, pb={pb_id:?})",
    );

    Ok(())
}

/// Find an existing vertex near the point, or create a new one.
fn find_or_create_vertex(
    topo: &mut Topology,
    arena: &GfaArena,
    point: Point3,
    tol: Tolerance,
) -> remus_topology::vertex::VertexId {
    if let Some(vid) = find_nearby_pave_vertex(topo, arena, point, tol) {
        return vid;
    }
    topo.add_vertex(Vertex::new(point, tol.linear))
}

// ---------------------------------------------------------------------------
// 2D geometry helpers
// ---------------------------------------------------------------------------

/// Minimal plane frame for 3D ↔ 2D projection (same logic as
/// `builder::plane_frame::PlaneFrame` but kept local to avoid coupling).
struct PlaneFrame2D {
    origin: Point3,
    u_axis: Vec3,
    v_axis: Vec3,
}

impl PlaneFrame2D {
    fn new(normal: Vec3, origin: Point3) -> Self {
        let seed = if normal.x().abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let u_raw = normal.cross(seed);
        let u_axis = u_raw.normalize().unwrap_or(Vec3::new(1.0, 0.0, 0.0));
        let v_axis = normal.cross(u_axis);
        Self {
            origin,
            u_axis,
            v_axis,
        }
    }

    fn project(&self, p: Point3) -> Point2 {
        let d = p - self.origin;
        Point2::new(d.dot(self.u_axis), d.dot(self.v_axis))
    }
}

/// Ray-casting point-in-polygon test.
///
/// Returns `true` if `pt` is strictly inside `polygon` (CCW or CW vertex order).
fn point_in_polygon_2d(pt: Point2, polygon: &[Point2]) -> bool {
    if polygon.len() < 3 {
        return false;
    }

    let mut inside = false;
    let n = polygon.len();
    let mut j = n - 1;

    for i in 0..n {
        let pi = polygon[i];
        let pj = polygon[j];

        let yi = pi.y();
        let yj = pj.y();
        let xi = pi.x();
        let xj = pj.x();

        if ((yi > pt.y()) != (yj > pt.y())) && (pt.x() < (xj - xi) * (pt.y() - yi) / (yj - yi) + xi)
        {
            inside = !inside;
        }

        j = i;
    }

    inside
}

/// Return the index of the boundary edge that the point lies on, if any.
fn which_boundary_edge(pt: Point2, edges: &[BoundaryEdge], tol: f64) -> Option<usize> {
    edges
        .iter()
        .position(|&(_, a, b, _, _)| point_on_segment_2d(pt, a, b, tol))
}

/// Check if a 2D point lies on a line segment within tolerance.
fn point_on_segment_2d(pt: Point2, a: Point2, b: Point2, tol: f64) -> bool {
    let ab = Point2::new(b.x() - a.x(), b.y() - a.y());
    let ap = Point2::new(pt.x() - a.x(), pt.y() - a.y());

    let ab_len_sq = ab.x() * ab.x() + ab.y() * ab.y();
    if ab_len_sq < tol * tol {
        // Degenerate segment — just check distance to endpoint
        return ap.x() * ap.x() + ap.y() * ap.y() <= tol * tol;
    }

    let t = (ap.x() * ab.x() + ap.y() * ab.y()) / ab_len_sq;
    if t < -tol || t > 1.0 + tol {
        return false;
    }

    let closest_x = a.x() + t.clamp(0.0, 1.0) * ab.x();
    let closest_y = a.y() + t.clamp(0.0, 1.0) * ab.y();
    let dx = pt.x() - closest_x;
    let dy = pt.y() - closest_y;

    dx * dx + dy * dy <= tol * tol
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;

    fn oracle_polygon(topo: &mut Topology, coordinates: &[(f64, f64)]) -> FaceId {
        use remus_topology::wire::{OrientedEdge, Wire};
        let vertices: Vec<_> = coordinates
            .iter()
            .map(|&(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, 0.), 1e-7)))
            .collect();
        let edges = (0..vertices.len())
            .map(|i| {
                OrientedEdge::new(
                    topo.add_edge(Edge::new(
                        vertices[i],
                        vertices[(i + 1) % vertices.len()],
                        EdgeCurve::Line,
                    )),
                    true,
                )
            })
            .collect();
        let wire = topo.add_wire(Wire::new(edges, true).unwrap());
        remus_topology::builder::make_planar_face_from_wire(topo, wire).unwrap()
    }

    fn oracle_spline(topo: &mut Topology, points: [Point3; 3]) -> remus_topology::edge::EdgeId {
        let a = topo.add_vertex(Vertex::new(points[0], 1e-7));
        let b = topo.add_vertex(Vertex::new(points[2], 1e-7));
        let curve = remus_math::nurbs::curve::NurbsCurve::new(
            2,
            vec![0., 0., 0., 1., 1., 1.],
            points.to_vec(),
            vec![1.; 3],
        )
        .unwrap();
        let mut edge = Edge::new(a, b, EdgeCurve::NurbsCurve(curve));
        edge.set_trim(Some((0., 1.)));
        topo.add_edge(edge)
    }

    #[test]
    fn concave_face_exclusion_never_uses_a_spline_chord() {
        let mut topo = Topology::new();
        let target = oracle_polygon(
            &mut topo,
            &[
                (0., 0.),
                (4., 0.),
                (4., 4.),
                (3., 4.),
                (3., 1.),
                (1., 1.),
                (1., 4.),
                (0., 4.),
            ],
        );
        let outside = oracle_spline(
            &mut topo,
            [
                Point3::new(1.5, 2., 0.),
                Point3::new(2., 3., 0.),
                Point3::new(2.5, 2., 0.),
            ],
        );
        let possible_crossing = oracle_spline(
            &mut topo,
            [
                Point3::new(1.5, 2., 0.),
                Point3::new(2., -1., 0.),
                Point3::new(2.5, 2., 0.),
            ],
        );
        assert!(
            spline_hull_disjoint_from_line_face(&topo, outside, target, Tolerance::new()).unwrap()
        );
        assert!(
            !spline_hull_disjoint_from_line_face(
                &topo,
                possible_crossing,
                target,
                Tolerance::new()
            )
            .unwrap()
        );
        let square = oracle_polygon(&mut topo, &[(0., 0.), (4., 0.), (4., 4.), (0., 4.)]);
        let hole = oracle_polygon(&mut topo, &[(1., 1.), (3., 1.), (3., 3.5), (1., 3.5)]);
        let inner = topo.face(hole).unwrap().outer_wire();
        let outer = topo.face(square).unwrap().outer_wire();
        topo.set_face_boundary_wires(square, outer, vec![inner])
            .unwrap();
        assert!(
            spline_hull_disjoint_from_line_face(&topo, outside, square, Tolerance::new()).unwrap()
        );
        assert!(
            !spline_hull_disjoint_from_line_face(
                &topo,
                possible_crossing,
                square,
                Tolerance::new()
            )
            .unwrap()
        );
    }

    #[test]
    fn shared_spline_caps_need_no_regular_ff_section_witness() {
        let mut topo = Topology::new();
        let face = oracle_polygon(&mut topo, &[(0., 0.), (2., 0.), (2., 2.), (0., 2.)]);
        let boundary = remus_topology::explorer::face_edges(&topo, face).unwrap()[0];
        let curved = oracle_spline(
            &mut topo,
            [
                Point3::new(0., 0., 0.),
                Point3::new(1., -1., 0.),
                Point3::new(2., 0., 0.),
            ],
        );
        let curve = topo.edge(curved).unwrap().curve().clone();
        topo.edge_mut(boundary).unwrap().set_curve(curve);
        topo.edge_mut(boundary).unwrap().set_trim(Some((0., 1.)));
        let partner = topo.add_face(topo.face(face).unwrap().clone());
        assert!(shared_spline_boundary(&topo, boundary, partner).unwrap());
        let mut arena = GfaArena::new();
        process_coplanar_pair(
            &mut topo,
            face,
            Vec3::new(0., 0., 1.),
            partner,
            Tolerance::new(),
            &mut arena,
            &DetHashSet::default(),
        )
        .unwrap();
        let a = Point3::new(0., 0., 0.);
        let b = Point3::new(2., 0., 0.);
        assert!(
            arena.curves.iter().all(|section| {
                (section.bbox.min - a).length() > 1e-7 || (section.bbox.max - b).length() > 1e-7
            }),
            "a shared spline seam must not mint its chord"
        );
        let lens = oracle_spline(
            &mut topo,
            [
                Point3::new(0., 0., 0.),
                Point3::new(1., 1., 0.),
                Point3::new(2., 0., 0.),
            ],
        );
        assert!(!spline_pair_compatible(&topo, boundary, lens).unwrap());
        let line = topo.add_edge(Edge::new(
            topo.edge(boundary).unwrap().start(),
            topo.edge(boundary).unwrap().end(),
            EdgeCurve::Line,
        ));
        assert!(!spline_pair_compatible(&topo, boundary, line).unwrap());
    }

    #[test]
    fn spline_exclusion_uses_the_whole_hull_and_a_polygonal_partner() {
        use remus_math::nurbs::curve::NurbsCurve;
        let mut topo = Topology::new();
        let face = remus_topology::test_utils::make_unit_square_face(&mut topo);
        let mut edge = |middle_x| {
            let a = Point3::new(2.0, 0.0, 0.0);
            let b = Point3::new(2.0, 1.0, 0.0);
            let va = topo.add_vertex(Vertex::new(a, 1e-7));
            let vb = topo.add_vertex(Vertex::new(b, 1e-7));
            let curve = remus_topology::edge::EdgeCurve::NurbsCurve(
                NurbsCurve::new(
                    2,
                    vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                    vec![a, Point3::new(middle_x, 0.5, 0.0), b],
                    vec![1.0; 3],
                )
                .unwrap(),
            );
            topo.add_edge(Edge::new(va, vb, curve))
        };
        let outside = edge(3.0);
        let crossing = edge(-2.0);
        assert!(
            spline_hull_disjoint_from_line_face(&topo, outside, face, Tolerance::new()).unwrap()
        );
        assert!(
            !spline_hull_disjoint_from_line_face(&topo, crossing, face, Tolerance::new()).unwrap()
        );
        // An endpoint box cannot enclose this curved partner boundary.
        let boundary = remus_topology::explorer::face_edges(&topo, face).unwrap()[0];
        let replacement = topo.edge(crossing).unwrap().curve().clone();
        topo.edge_mut(boundary).unwrap().set_curve(replacement);
        assert!(
            !spline_hull_disjoint_from_line_face(&topo, outside, face, Tolerance::new()).unwrap()
        );
    }

    #[test]
    fn strict_profile_guard_qualifies_bezier_rows_but_not_fitted_multispans() {
        use remus_math::nurbs::{NurbsCurve, NurbsSurface};
        for (knots, count, expected) in [
            (vec![0., 0., 0., 0., 1., 1., 1., 1.], 4, true),
            (vec![0., 0., 0., 0., 0.5, 1., 1., 1., 1.], 5, false),
        ] {
            let mut topo = Topology::new();
            let face = remus_topology::test_utils::make_unit_square_face(&mut topo);
            let edge = remus_topology::explorer::face_edges(&topo, face).unwrap()[0];
            let row: Vec<_> = (0..count)
                .map(|i| Point3::new(f64::from(i), 0., 0.))
                .collect();
            let translated = row.iter().map(|&p| p + Vec3::new(0., 0., 2.)).collect();
            let curve =
                NurbsCurve::new(3, knots.clone(), row.clone(), vec![1.; row.len()]).unwrap();
            topo.edge_mut(edge)
                .unwrap()
                .set_curve(EdgeCurve::NurbsCurve(curve.clone()));
            let surface = NurbsSurface::new(
                1,
                3,
                vec![0., 0., 1., 1.],
                knots,
                vec![row, translated],
                vec![vec![1.; count as usize]; 2],
            )
            .unwrap();
            topo.face_mut(face)
                .unwrap()
                .set_surface(FaceSurface::Nurbs(surface));
            assert_eq!(
                ruled_profile_edges(&topo, std::iter::once(face))
                    .unwrap()
                    .contains(&edge),
                expected
            );
            topo.edge_mut(edge)
                .unwrap()
                .set_curve(EdgeCurve::NurbsCurve(curve.reversed()));
            assert_eq!(
                ruled_profile_edges(&topo, std::iter::once(face))
                    .unwrap()
                    .contains(&edge),
                expected
            );
            topo.edge_mut(edge).unwrap().set_trim(Some(curve.domain()));
            let no_sections = GfaArena::new();
            assert!(
                matching_boundary_section_exists(
                    &topo,
                    &no_sections,
                    face,
                    face,
                    edge,
                    Tolerance::new(),
                    expected
                )
                .is_err()
                    == expected
            );
        }
    }

    #[test]
    fn spline_chord_is_suppressed_only_with_a_whole_span_coefficient_witness() {
        use remus_math::nurbs::curve::NurbsCurve;
        use remus_topology::wire::{OrientedEdge, Wire};

        let mut topo = Topology::new();
        let points = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.0, 2.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
        ];
        let vertices = points.map(|p| topo.add_vertex(Vertex::new(p, 1e-7)));
        let spline = |y| {
            NurbsCurve::new(
                2,
                vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                vec![points[0], Point3::new(1.0, y, 0.0), points[1]],
                vec![1.0; 3],
            )
            .unwrap()
        };
        let boundary = spline(-1.0);
        let mut boundary_edge = Edge::new(
            vertices[0],
            vertices[1],
            EdgeCurve::NurbsCurve(boundary.clone()),
        );
        boundary_edge.set_trim(Some((0.0, 1.0)));
        let eid = topo.add_edge(boundary_edge);
        let mut edges = vec![OrientedEdge::new(eid, true)];
        for i in 1..4 {
            edges.push(OrientedEdge::new(
                topo.add_edge(Edge::new(
                    vertices[i],
                    vertices[(i + 1) % 4],
                    EdgeCurve::Line,
                )),
                true,
            ));
        }
        let wire = topo.add_wire(Wire::new(edges, true).unwrap());
        let face = remus_topology::builder::make_planar_face_from_wire(&mut topo, wire).unwrap();
        let mut arena = GfaArena::new();
        assert!(
            matching_boundary_section_exists(
                &topo,
                &arena,
                face,
                face,
                eid,
                Tolerance::new(),
                true
            )
            .is_err()
        );
        arena.curves.push(IntersectionCurveDS {
            curve: EdgeCurve::NurbsCurve(spline(1.0)),
            face_a: face,
            face_b: face,
            bbox: boundary.aabb(),
            pave_blocks: vec![],
            t_range: (0.0, 1.0),
        });
        assert!(
            matching_boundary_section_exists(
                &topo,
                &arena,
                face,
                face,
                eid,
                Tolerance::new(),
                true
            )
            .is_err(),
            "co-endpoint lens is a different curve"
        );
        arena.curves[0].curve = EdgeCurve::NurbsCurve(boundary.clone());
        arena.curves[0].t_range = (0.25, 0.75);
        assert!(
            matching_boundary_section_exists(
                &topo,
                &arena,
                face,
                face,
                eid,
                Tolerance::new(),
                true
            )
            .is_err(),
            "a subspan cannot replace the whole boundary"
        );
        arena.curves[0].t_range = (0.0, 1.0);
        assert!(
            matching_boundary_section_exists(
                &topo,
                &arena,
                face,
                face,
                eid,
                Tolerance::new(),
                true
            )
            .unwrap()
        );
        arena.curves[0].curve = EdgeCurve::NurbsCurve(boundary.reversed());
        assert!(
            matching_boundary_section_exists(
                &topo,
                &arena,
                face,
                face,
                eid,
                Tolerance::new(),
                true
            )
            .unwrap()
        );
        arena.curves[0].t_range = (0.75, 0.25);
        assert!(
            matching_boundary_section_exists(
                &topo,
                &arena,
                face,
                face,
                eid,
                Tolerance::new(),
                true
            )
            .is_err()
        );
    }

    #[test]
    fn concave_clipping_preserves_disconnected_intervals() {
        // U-shaped face: the horizontal section must never bridge its opening.
        let polygon = [
            (0.0, 0.0),
            (4.0, 0.0),
            (4.0, 4.0),
            (3.0, 4.0),
            (3.0, 1.0),
            (1.0, 1.0),
            (1.0, 4.0),
            (0.0, 4.0),
        ]
        .map(|(x, y)| Point2::new(x, y));
        for (start, end, expected) in [
            (-1.0, 5.0, [(0.0, 1.0), (3.0, 4.0)]),
            (0.5, 3.5, [(0.5, 1.0), (3.0, 3.5)]),
            (3.5, 0.5, [(3.5, 3.0), (1.0, 0.5)]),
        ] {
            let sections = clip_section_to_polygon(
                Point2::new(start, 2.0),
                Point2::new(end, 2.0),
                Point3::new(start, 2.0, 7.0),
                Point3::new(end, 2.0, 7.0),
                &polygon,
                1e-7,
            );
            assert_eq!(sections.len(), 2, "{sections:?}");
            for ((a, b), (x0, x1)) in sections.into_iter().zip(expected) {
                assert!((a.x() - x0).abs() < 1e-10);
                assert!((b.x() - x1).abs() < 1e-10);
                assert!((a.y() - 2.0).abs() < 1e-10 && (b.y() - 2.0).abs() < 1e-10);
                assert!((a.z() - 7.0).abs() < 1e-10 && (b.z() - 7.0).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn point_in_unit_square() {
        let square = vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 1.0),
        ];
        assert!(point_in_polygon_2d(Point2::new(0.5, 0.5), &square));
        assert!(!point_in_polygon_2d(Point2::new(2.0, 0.5), &square));
        assert!(!point_in_polygon_2d(Point2::new(-0.1, 0.5), &square));
    }

    #[test]
    fn point_on_segment() {
        let a = Point2::new(0.0, 0.0);
        let b = Point2::new(1.0, 0.0);
        assert!(point_on_segment_2d(Point2::new(0.5, 0.0), a, b, 1e-7));
        assert!(!point_on_segment_2d(Point2::new(0.5, 1.0), a, b, 1e-7));
        assert!(point_on_segment_2d(Point2::new(0.0, 0.0), a, b, 1e-7));
        assert!(point_on_segment_2d(Point2::new(1.0, 0.0), a, b, 1e-7));
    }
}
