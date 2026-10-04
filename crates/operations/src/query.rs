//! Shape query utilities.

use std::collections::{HashMap, HashSet};

use remus_math::polygon_boolean::{BooleanOp as PolygonBooleanOp, polygon_boolean_with_budget};
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{face_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use crate::OperationsError;
use crate::boolean::{face_polygon, wire_polygon};
use crate::classify::{PointClassification, classify_point_robust};
use crate::measure::face_area;

/// An opposing pair of parallel planar faces with a non-zero projected overlap.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpposingPlanarFacePair {
    /// First face in stable topology order.
    pub face_a: FaceId,
    /// Second face in stable topology order.
    pub face_b: FaceId,
    /// Perpendicular distance between the two planes.
    pub distance: f64,
    /// Area covered by both trimmed faces after projection onto either plane.
    pub overlap_area: f64,
    /// Trimmed area of `face_a`.
    pub face_area_a: f64,
    /// Trimmed area of `face_b`.
    pub face_area_b: f64,
    /// Effective outward normal of `face_a`.
    pub normal: Vec3,
    /// Whether `face_a` has a tangent contact with a curved blend face.
    pub face_a_borders_blend: bool,
    /// Whether `face_b` has a tangent contact with a curved blend face.
    pub face_b_borders_blend: bool,
}

struct PlanarFaceRegion {
    face: FaceId,
    normal: Vec3,
    outer: Vec<Point3>,
    holes: Vec<Vec<Point3>>,
    area: f64,
    borders_blend: bool,
    /// Points across the outer ring and every hole ring.
    ring_points: usize,
}

// Query-local limits apply to native, scalar WASM, and batch entry points.
// Preparation counts references (including repeated uses), not unique entities.
#[derive(Clone, Copy)]
struct PlanarQueryLimits {
    faces: usize,
    topology_uses: usize,
    samples: usize,
    work: usize,
}

impl Default for PlanarQueryLimits {
    fn default() -> Self {
        Self {
            faces: 512,
            topology_uses: 16_384,
            samples: 262_144,
            work: 1_000_000_000,
        }
    }
}

fn query_budget_error(resource: &str) -> OperationsError {
    OperationsError::InvalidInput {
        reason: format!("opposing planar face-pair query {resource} budget exceeded"),
    }
}

fn reserve_query_budget(
    remaining: &mut usize,
    amount: Option<usize>,
    resource: &str,
) -> Result<(), OperationsError> {
    *remaining = amount
        .and_then(|amount| remaining.checked_sub(amount))
        .ok_or_else(|| query_budget_error(resource))?;
    Ok(())
}

fn preflight_planar_query(
    topo: &Topology,
    solid: SolidId,
    limits: PlanarQueryLimits,
    remaining_work: &mut usize,
) -> Result<Vec<FaceId>, OperationsError> {
    let solid_data = topo.solid(solid)?;
    let mut remaining_uses = limits.topology_uses;
    let mut remaining_faces = limits.faces;
    let mut remaining_samples = limits.samples;
    reserve_query_budget(
        &mut remaining_uses,
        solid_data.inner_shells().len().checked_add(1),
        "topology",
    )?;
    // Count borrowed slices before solid_faces or adjacency can allocate.
    for shell in
        std::iter::once(solid_data.outer_shell()).chain(solid_data.inner_shells().iter().copied())
    {
        let count = topo.shell(shell)?.faces().len();
        reserve_query_budget(
            &mut remaining_faces,
            Some(count),
            "face count (maximum 512)",
        )?;
        reserve_query_budget(&mut remaining_uses, Some(count), "topology")?;
    }
    let faces = solid_faces(topo, solid)?;
    let mut edge_uses = 0_usize;
    for &face in &faces {
        let face_data = topo.face(face)?;
        reserve_query_budget(
            &mut remaining_uses,
            face_data.inner_wires().len().checked_add(1),
            "topology",
        )?;
        for wire in
            std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
        {
            let edges = topo.wire(wire)?.edges();
            reserve_query_budget(&mut remaining_uses, Some(edges.len()), "topology")?;
            edge_uses = edge_uses
                .checked_add(edges.len())
                .ok_or_else(|| query_budget_error("topology"))?;
            if face_data.surface().is_planar() {
                for oriented in edges {
                    let edge = topo.edge(oriented.edge())?;
                    // Reserve both materializations: area fallback emits at
                    // most 257 curved-edge points and wire_polygon at most 32.
                    // Line edges contribute one point to each.
                    let samples = if matches!(edge.curve(), EdgeCurve::Line) {
                        2
                    } else {
                        289
                    };
                    reserve_query_budget(
                        &mut remaining_samples,
                        Some(samples),
                        "boundary samples",
                    )?;
                }
            }
        }
    }
    // Adjacency and blend-border traversal also process repeated edge uses.
    reserve_query_budget(
        remaining_work,
        edge_uses.checked_mul(edge_uses),
        "preparation work",
    )?;
    Ok(faces)
}

fn plane_frame(normal: Vec3) -> Option<(Vec3, Vec3)> {
    let seed = if normal.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u = normal.cross(seed).normalize().ok()?;
    let v = normal.cross(u).normalize().ok()?;
    Some((u, v))
}

fn project_loop(points: &[Point3], origin: Point3, u: Vec3, v: Vec3) -> Vec<Point2> {
    points
        .iter()
        .map(|point| {
            let delta = *point - origin;
            Point2::new(delta.dot(u), delta.dot(v))
        })
        .collect()
}

/// Axis-aligned extent of a projected ring; empty rings and rings with
/// non-finite points get no box and are never skipped.
fn ring_box(ring: &[Point2]) -> Option<[f64; 4]> {
    let first = ring.first()?;
    let mut extent = [first.x(), first.y(), first.x(), first.y()];
    for point in ring {
        if !point.x().is_finite() || !point.y().is_finite() {
            return None;
        }
        extent[0] = extent[0].min(point.x());
        extent[1] = extent[1].min(point.y());
        extent[2] = extent[2].max(point.x());
        extent[3] = extent[3].max(point.y());
    }
    Some(extent)
}

/// Whether two ring boxes are farther apart than the polygon boolean can
/// ever join. Snapping moves a point by at most half a `tolerance` cell per
/// axis, and splitting and boundary tests reach `tolerance`, so rings whose
/// boxes are separated by more than twice that cannot share an edge, a
/// split point, or an interior point: their intersection is empty and its
/// area exactly zero, which is what the skipped call would have returned.
fn rings_disjoint(a: Option<[f64; 4]>, b: Option<[f64; 4]>, tolerance: Tolerance) -> bool {
    let (Some(a), Some(b)) = (a, b) else {
        return false;
    };
    let gap = 2.0 * tolerance.linear;
    b[0] - a[2] > gap || a[0] - b[2] > gap || b[1] - a[3] > gap || a[1] - b[3] > gap
}

fn polygon_overlap_area(
    a: &[Point2],
    b: &[Point2],
    tolerance: Tolerance,
    remaining_work: &mut usize,
) -> Result<f64, OperationsError> {
    let overlap = polygon_boolean_with_budget(
        a,
        b,
        PolygonBooleanOp::Intersection,
        tolerance.linear,
        remaining_work,
    )
    .map_err(|_| query_budget_error("polygon work"))?;
    Ok(overlap.area().max(0.0))
}

/// Projected overlap of two trimmed planar faces, by inclusion-exclusion
/// over their outer and hole rings.
///
/// Work is metered as it is done against the query's shared budget: the
/// projection and ring boxes per point, one unit per ring pair, and each
/// polygon boolean by the arrangement it actually builds (see
/// [`polygon_boolean_with_budget`]). Ring pairs whose boxes are disjoint
/// contribute exactly zero and are skipped, so a plate drilled with `k`
/// through holes pays for its `k` coincident hole pairs, not all `k²`.
fn projected_overlap_area(
    a: &PlanarFaceRegion,
    b: &PlanarFaceRegion,
    tolerance: Tolerance,
    remaining_work: &mut usize,
) -> Result<f64, OperationsError> {
    // Projecting every ring point, then boxing it.
    reserve_query_budget(
        remaining_work,
        a.ring_points
            .checked_add(b.ring_points)
            .and_then(|points| points.checked_mul(2)),
        "polygon work",
    )?;
    let Some(&origin) = a.outer.first() else {
        return Ok(0.0);
    };
    let Some((u, v)) = plane_frame(a.normal) else {
        return Ok(0.0);
    };
    let project = |ring: &[Point3]| {
        let projected = project_loop(ring, origin, u, v);
        let extent = ring_box(&projected);
        (projected, extent)
    };
    let (outer_a, box_a) = project(&a.outer);
    let (outer_b, box_b) = project(&b.outer);
    let holes_a: Vec<_> = a.holes.iter().map(|hole| project(hole)).collect();
    let holes_b: Vec<_> = b.holes.iter().map(|hole| project(hole)).collect();
    let mut ring_overlap = |ring_a: &(Vec<Point2>, Option<[f64; 4]>),
                            ring_b: &(Vec<Point2>, Option<[f64; 4]>)|
     -> Result<f64, OperationsError> {
        reserve_query_budget(remaining_work, Some(1), "polygon work")?;
        if rings_disjoint(ring_a.1, ring_b.1, tolerance) {
            return Ok(0.0);
        }
        polygon_overlap_area(&ring_a.0, &ring_b.0, tolerance, remaining_work)
    };
    let outer_a = (outer_a, box_a);
    let outer_b = (outer_b, box_b);

    let mut overlap = ring_overlap(&outer_a, &outer_b)?;
    for hole in &holes_a {
        overlap -= ring_overlap(hole, &outer_b)?;
    }
    for hole in &holes_b {
        overlap -= ring_overlap(hole, &outer_a)?;
    }
    for hole_a in &holes_a {
        for hole_b in &holes_b {
            overlap += ring_overlap(hole_a, hole_b)?;
        }
    }
    Ok(overlap.clamp(0.0, a.area.min(b.area)))
}

fn borders_blend(
    topo: &Topology,
    adjacency: &remus_topology::adjacency::AdjacencyIndex,
    face: FaceId,
) -> Result<bool, OperationsError> {
    for edge in face_edges(topo, face)? {
        for &neighbour in adjacency.faces_for_edge(edge) {
            if neighbour == face || topo.face(neighbour)?.surface().is_planar() {
                continue;
            }
            if edge_is_g1(topo, edge, face, neighbour)? {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// Find pairs of opposing parallel planar faces with overlapping projections.
///
/// Faces are returned in stable topology order. The pair direction is chosen
/// so the effective outward normals point away from the material between the
/// planes; parallel faces whose normals point into the intervening gap are not
/// thickness candidates.
///
/// # Errors
///
/// Returns an error if the solid topology cannot be traversed or measured,
/// or if the query exceeds 512 face references, 16,384 topology references,
/// 262,144 boundary samples, or 1,000,000,000 work units. Work is metered
/// as it is done, each step charged before it runs: quadratic preparation
/// over edge uses, then every face pair's projection and polygon booleans
/// by the arrangement they actually build. A 100 x 100 x 6 plate drilled
/// with seventy through holes spends about 1,030,000 units.
/// Refusal returns no partial pairs and does not modify the topology.
pub fn opposing_planar_face_pairs(
    topo: &Topology,
    solid: SolidId,
    tolerance: Tolerance,
) -> Result<Vec<OpposingPlanarFacePair>, OperationsError> {
    opposing_planar_face_pairs_with_limits(topo, solid, tolerance, PlanarQueryLimits::default())
}

fn opposing_planar_face_pairs_with_limits(
    topo: &Topology,
    solid: SolidId,
    tolerance: Tolerance,
    limits: PlanarQueryLimits,
) -> Result<Vec<OpposingPlanarFacePair>, OperationsError> {
    opposing_planar_face_pairs_metered(topo, solid, tolerance, limits).map(|(pairs, _)| pairs)
}

/// The query, also returning the work units it spent.
fn opposing_planar_face_pairs_metered(
    topo: &Topology,
    solid: SolidId,
    tolerance: Tolerance,
    limits: PlanarQueryLimits,
) -> Result<(Vec<OpposingPlanarFacePair>, usize), OperationsError> {
    let mut remaining_work = limits.work;
    let faces = preflight_planar_query(topo, solid, limits, &mut remaining_work)?;
    let adjacency = remus_topology::adjacency::AdjacencyIndex::build_from_faces(topo, &faces)?;
    let mut regions = Vec::new();
    for face in faces {
        let face_data = topo.face(face)?;
        let Some(normal) = face_data.effective_plane_normal() else {
            continue;
        };
        let outer = face_polygon(topo, face)?;
        if outer.len() < 3 {
            continue;
        }
        let holes: Vec<Vec<Point3>> = face_data
            .inner_wires()
            .iter()
            .map(|&wire| wire_polygon(topo, wire))
            .collect::<Result<Vec<_>, _>>()?;
        let ring_points = holes
            .iter()
            .try_fold(outer.len(), |sum, hole| sum.checked_add(hole.len()))
            .ok_or_else(|| query_budget_error("polygon work"))?;
        regions.push(PlanarFaceRegion {
            ring_points,
            face,
            normal,
            outer,
            holes,
            area: face_area(topo, face, tolerance.linear)?,
            borders_blend: borders_blend(topo, &adjacency, face)?,
        });
    }
    regions.sort_by_key(|region| region.face.index());

    let mut pairs = Vec::new();
    for (index, a) in regions.iter().enumerate() {
        for b in &regions[index + 1..] {
            let dot = a.normal.dot(b.normal);
            if (dot + 1.0).abs() > tolerance.angular {
                continue;
            }
            let signed_distance = (b.outer[0] - a.outer[0]).dot(a.normal);
            if signed_distance >= -tolerance.linear {
                continue;
            }
            let overlap_area = projected_overlap_area(a, b, tolerance, &mut remaining_work)?;
            if overlap_area <= tolerance.linear_sq() {
                continue;
            }
            pairs.push(OpposingPlanarFacePair {
                face_a: a.face,
                face_b: b.face,
                distance: -signed_distance,
                overlap_area,
                face_area_a: a.area,
                face_area_b: b.area,
                normal: a.normal,
                face_a_borders_blend: a.borders_blend,
                face_b_borders_blend: b.borders_blend,
            });
        }
    }
    Ok((pairs, limits.work - remaining_work))
}

/// Geometric relation between the two faces meeting at a manifold edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeConcavity {
    /// The edge rounds off material: a probe opposite the outward-normal
    /// bisector lands inside the solid.
    Convex,
    /// The edge is re-entrant: the same probe lands outside the solid.
    Concave,
    /// The faces meet with aligned outward normals along the sampled edge.
    Tangent,
    /// The edge is a self-seam, non-manifold, degenerate, or too ambiguous to
    /// classify without guessing.
    Unknown,
}

/// Effective outward normal of `face` at `point`.
///
/// Planar faces use the face orientation directly. Curved faces are projected
/// into their own UV domain before evaluating the surface normal, and the
/// `reversed` flag flips the result. Returns `None` when projection or
/// normalization cannot produce a finite unit normal.
#[must_use]
pub fn effective_face_normal(topo: &Topology, face: FaceId, point: Point3) -> Option<Vec3> {
    let face_data = topo.face(face).ok()?;
    let normal = if let Some(normal) = face_data.effective_plane_normal() {
        // `effective_plane_normal` has already applied the reversed flag.
        normal
    } else {
        let (u, v) = face_data.surface().project_point(point)?;
        let normal = face_data.surface().normal(u, v);
        if face_data.is_reversed() {
            -normal
        } else {
            normal
        }
    };
    normal.normalize().ok()
}

fn edge_samples(topo: &Topology, edge: EdgeId) -> Result<Vec<Point3>, OperationsError> {
    let edge_data = topo.edge(edge)?;
    let start = topo.vertex(edge_data.start())?.point();
    let end = topo.vertex(edge_data.end())?.point();
    let (t0, t1) = crate::authoritative_edge_domain(edge_data, "edge query sampling")?;
    Ok([0.25, 0.5, 0.75]
        .into_iter()
        .map(|fraction| {
            let t = (t1 - t0).mul_add(fraction, t0);
            edge_data.curve().evaluate_with_endpoints(t, start, end)
        })
        .collect())
}

fn face_vertex_span(topo: &Topology, face: FaceId) -> Result<f64, OperationsError> {
    let face_data = topo.face(face)?;
    let mut bounds: Option<(Point3, Point3)> = None;
    for wire_id in
        std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
    {
        for oriented in topo.wire(wire_id)?.edges() {
            let edge = topo.edge(oriented.edge())?;
            for vertex in [edge.start(), edge.end()] {
                let point = topo.vertex(vertex)?.point();
                bounds = Some(match bounds {
                    None => (point, point),
                    Some((lo, hi)) => (
                        Point3::new(
                            lo.x().min(point.x()),
                            lo.y().min(point.y()),
                            lo.z().min(point.z()),
                        ),
                        Point3::new(
                            hi.x().max(point.x()),
                            hi.y().max(point.y()),
                            hi.z().max(point.z()),
                        ),
                    ),
                });
            }
        }
    }
    Ok(bounds.map_or(0.0, |(lo, hi)| (hi - lo).length()))
}

fn edge_curve_span(topo: &Topology, edge: EdgeId) -> Result<f64, OperationsError> {
    let edge_data = topo.edge(edge)?;
    let start = topo.vertex(edge_data.start())?.point();
    let end = topo.vertex(edge_data.end())?.point();
    let (t0, t1) = crate::authoritative_edge_domain(edge_data, "edge span query")?;
    let mut bounds: Option<(Point3, Point3)> = None;
    for i in 0..=16 {
        let t = t0 + (t1 - t0) * f64::from(i) / 16.0;
        let point = edge_data.curve().evaluate_with_endpoints(t, start, end);
        bounds = Some(match bounds {
            None => (point, point),
            Some((lo, hi)) => (
                Point3::new(
                    lo.x().min(point.x()),
                    lo.y().min(point.y()),
                    lo.z().min(point.z()),
                ),
                Point3::new(
                    hi.x().max(point.x()),
                    hi.y().max(point.y()),
                    hi.z().max(point.z()),
                ),
            ),
        });
    }
    Ok(bounds.map_or(0.0, |(lo, hi)| (hi - lo).length()))
}

/// One interior edge sample with both faces' effective outward normals.
type NormalSample = (Point3, Vec3, Vec3);

fn sampled_normals(
    topo: &Topology,
    edge: EdgeId,
    face_a: FaceId,
    face_b: FaceId,
) -> Result<Vec<NormalSample>, OperationsError> {
    let mut samples = Vec::new();
    for point in edge_samples(topo, edge)? {
        if let (Some(na), Some(nb)) = (
            effective_face_normal(topo, face_a, point),
            effective_face_normal(topo, face_b, point),
        ) {
            samples.push((point, na, nb));
        }
    }
    Ok(samples)
}

/// The [`edge_is_g1`] verdict over already-sampled normals.
fn samples_are_g1(samples: &[NormalSample]) -> bool {
    samples.len() == 3 && samples.iter().all(|(_, a, b)| 1.0 - a.dot(*b) <= 1.0e-10)
}

/// The [`edge_normal_angle`] mean over already-sampled normals.
#[allow(clippy::cast_precision_loss)]
fn samples_normal_angle(samples: &[NormalSample]) -> Option<f64> {
    if samples.len() != 3 {
        return None;
    }
    let sum = samples
        .iter()
        .map(|(_, a, b)| a.cross(*b).length().atan2(a.dot(*b)))
        .sum::<f64>();
    Some(sum / samples.len() as f64)
}

/// Whether two distinct faces meet with aligned effective outward normals
/// throughout the edge's interior samples.
///
/// This is the G1 blend-contact convention: smooth contacts have a normal
/// angle near zero, not near pi. A projection/normal failure is not tangent.
///
/// # Errors
///
/// Returns `OperationsError::Topology` if any referenced entity is invalid.
pub fn edge_is_g1(
    topo: &Topology,
    edge: EdgeId,
    face_a: FaceId,
    face_b: FaceId,
) -> Result<bool, OperationsError> {
    if face_a == face_b {
        return Ok(false);
    }
    let samples = sampled_normals(topo, edge, face_a, face_b)?;
    Ok(samples_are_g1(&samples))
}

/// Angle between the effective outward normals, sampled along the edge and
/// reported in `[0, pi]`. This is deliberately not a signed 0..2pi dihedral.
///
/// # Errors
///
/// Returns `OperationsError::Topology` if any referenced entity is invalid.
pub fn edge_normal_angle(
    topo: &Topology,
    edge: EdgeId,
    face_a: FaceId,
    face_b: FaceId,
) -> Result<Option<f64>, OperationsError> {
    if face_a == face_b {
        return Ok(None);
    }
    let samples = sampled_normals(topo, edge, face_a, face_b)?;
    Ok(samples_normal_angle(&samples))
}

/// Classify the geometric relation between the two distinct faces at `edge`.
///
/// Tangent edges are decided by sampled normals. Sharp edges use four
/// material quadrant probes at the edge midpoint. The probe must be local:
/// above 25% of the local edge/face scale the result is
/// [`EdgeConcavity::Unknown`] rather than a confident answer from another
/// feature's neighbourhood. Boundary samples, self-seams, degenerate normals,
/// and non-manifold edges are also unknown rather than a guess.
///
/// # Errors
///
/// Returns `OperationsError::InvalidInput` for a non-positive/non-finite probe
/// step, or propagates topology/classification errors.
pub fn edge_concavity(
    topo: &Topology,
    solid: SolidId,
    edge: EdgeId,
    probe: f64,
) -> Result<EdgeConcavity, OperationsError> {
    if !probe.is_finite() || probe <= 0.0 {
        return Err(OperationsError::InvalidInput {
            reason: "edge concavity probe must be positive and finite".into(),
        });
    }
    let adjacency = topo.build_adjacency(solid)?;
    let faces = adjacency.faces_for_edge(edge);
    if faces.len() != 2 || faces[0] == faces[1] {
        return Ok(EdgeConcavity::Unknown);
    }
    let (face_a, face_b) = (faces[0], faces[1]);
    let samples = sampled_normals(topo, edge, face_a, face_b)?;
    edge_concavity_with_samples(topo, edge, face_a, face_b, probe, &samples, |point| {
        classify_point_robust(topo, solid, point, 0.01, 1.0e-7)
    })
}

/// One-shot variant for callers that already built edge-to-face adjacency.
///
/// Uses the analytic classifier rather than rebuilding a full-solid
/// tessellation for each probe. The supplied faces must be the two incident
/// faces of `edge`. Callers that classify many edges of one solid should use
/// [`EdgeRelationContext`], which prepares the classifier once and reaches
/// the same verdicts.
pub(crate) fn edge_concavity_from_faces(
    topo: &Topology,
    solid: SolidId,
    edge: EdgeId,
    face_a: FaceId,
    face_b: FaceId,
    probe: f64,
) -> Result<EdgeConcavity, OperationsError> {
    if !probe.is_finite() || probe <= 0.0 {
        return Err(OperationsError::InvalidInput {
            reason: "edge concavity probe must be positive and finite".into(),
        });
    }
    if face_a == face_b {
        return Ok(EdgeConcavity::Unknown);
    }
    let samples = sampled_normals(topo, edge, face_a, face_b)?;
    edge_concavity_with_samples(topo, edge, face_a, face_b, probe, &samples, |point| {
        crate::classify::classify_point(topo, solid, point, 0.01, 1.0e-7)
    })
}

/// Operation-local context for classifying many edges of one solid.
///
/// Feature recognition and the bulk edge-relation queries classify four
/// quadrant probes per manifold edge. The one-shot `classify_point` rebuilt
/// every face bound, the face BVH and the candidate trim polygons for every
/// ray of every probe, which made a 400-edge import pay ~1 600 full-solid
/// preparations. This context prepares the solid once
/// ([`remus_check::classify::PreparedSolid`], PERF-Q01) and runs the same
/// shared ray-vote loop over the same faces, bounds and trims, so every probe
/// reaches the verdict the one-shot path reached (O06).
///
/// It also samples each edge's normals once and derives the G1 test, the
/// normal angle and the concavity probe point from that one sample set, where
/// the separate helpers used to project the same three points onto both faces
/// three times.
///
/// The borrow on the topology does the invalidation: the context cannot
/// outlive the state it was prepared from.
pub(crate) struct EdgeRelationContext<'a> {
    topo: &'a Topology,
    prepared: remus_check::classify::PreparedSolid<'a>,
}

impl<'a> EdgeRelationContext<'a> {
    /// Prepare `solid` for repeated edge classification.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle or its faces are invalid.
    pub(crate) fn prepare(topo: &'a Topology, solid: SolidId) -> Result<Self, OperationsError> {
        Ok(Self {
            topo,
            prepared: remus_check::classify::PreparedSolid::prepare(topo, solid)?,
        })
    }

    /// Concavity plus the `[0, pi]` normal angle of one manifold edge.
    ///
    /// Same verdict as the analytic-classifier bulk path and the same angle as
    /// [`edge_normal_angle`]. `face_a` and `face_b` must be the two incident
    /// faces of `edge`; equal faces are a self-seam and report
    /// [`EdgeConcavity::Unknown`] with no angle.
    ///
    /// # Errors
    ///
    /// Returns `InvalidInput` for a non-positive or non-finite probe, or
    /// propagates topology and classification errors.
    pub(crate) fn concavity_and_angle(
        &self,
        edge: EdgeId,
        face_a: FaceId,
        face_b: FaceId,
        probe: f64,
    ) -> Result<(EdgeConcavity, Option<f64>), OperationsError> {
        if !probe.is_finite() || probe <= 0.0 {
            return Err(OperationsError::InvalidInput {
                reason: "edge concavity probe must be positive and finite".into(),
            });
        }
        if face_a == face_b {
            return Ok((EdgeConcavity::Unknown, None));
        }
        let samples = sampled_normals(self.topo, edge, face_a, face_b)?;
        let options = remus_check::classify::ClassifyOptions {
            tolerance: 1.0e-7,
            ..Default::default()
        };
        let concavity = edge_concavity_with_samples(
            self.topo,
            edge,
            face_a,
            face_b,
            probe,
            &samples,
            |point| Ok(self.prepared.classify_point(point, &options)?.into()),
        )?;
        Ok((concavity, samples_normal_angle(&samples)))
    }
}

/// Concavity of a non-self-seam edge from its already-sampled normals.
///
/// `classify` answers one quadrant probe; the robust single-call path and the
/// prepared bulk path differ only there.
fn edge_concavity_with_samples(
    topo: &Topology,
    edge: EdgeId,
    face_a: FaceId,
    face_b: FaceId,
    probe: f64,
    samples: &[NormalSample],
    mut classify: impl FnMut(Point3) -> Result<PointClassification, OperationsError>,
) -> Result<EdgeConcavity, OperationsError> {
    if samples_are_g1(samples) {
        return Ok(EdgeConcavity::Tangent);
    }

    let Some(&(point, normal_a, normal_b)) = samples.get(1).or_else(|| samples.first()) else {
        return Ok(EdgeConcavity::Unknown);
    };
    let face_scale = face_vertex_span(topo, face_a)?.min(face_vertex_span(topo, face_b)?);
    let local_scale = face_scale.max(edge_curve_span(topo, edge)?);
    if probe > local_scale * 0.25 {
        return Ok(EdgeConcavity::Unknown);
    }

    // The four normal-halfspace quadrants distinguish the two local shapes
    // without depending on face orientation bookkeeping: a convex edge is an
    // intersection of inward halfspaces (exactly one quadrant is material),
    // while a concave edge is their union (exactly three quadrants are).
    let inward_a = -normal_a * probe;
    let inward_b = -normal_b * probe;
    let quadrants = [
        inward_a + inward_b,
        inward_a - inward_b,
        -inward_a + inward_b,
        -inward_a - inward_b,
    ];
    let mut inside = 0;
    for offset in quadrants {
        match classify(point + offset)? {
            PointClassification::Inside => inside += 1,
            PointClassification::Outside => {}
            PointClassification::OnBoundary => return Ok(EdgeConcavity::Unknown),
        }
    }
    Ok(match inside {
        1 => EdgeConcavity::Convex,
        3 => EdgeConcavity::Concave,
        _ => EdgeConcavity::Unknown,
    })
}

/// The parameter domain an edge ACTUALLY covers on its stored curve.
///
/// Curved edges report their stored authoritative trim interval verbatim; a
/// closed edge reports one full period anchored at its start vertex, and an
/// open edge uses the endpoint-trimmed convention. Lines report
/// `(0, length)` to match the query surface's parameterization of line edges.
///
/// This differs from the raw curve domain (`[0, TAU]` for every circle): a
/// circle edge's endpoints subtend TWO arcs, and only this span says which
/// one the edge is — reconstructing it from endpoints alone flips intentional
/// major arcs.
///
/// # Errors
///
/// Returns an error if the edge handle is invalid, if vertex lookup fails for
/// line endpoints, or if a curved edge lacks valid stored parameter
/// authority.
pub fn trimmed_edge_domain(topo: &Topology, edge: EdgeId) -> Result<(f64, f64), OperationsError> {
    let edge_data = topo.edge(edge)?;
    crate::tessellate::edge_param_span(topo, edge_data)
}

/// Filter edges to only those shared by two planar faces in a solid.
///
/// Given a solid and a set of edge IDs, returns only the edges
/// where both adjacent faces have a planar surface.
///
/// # Errors
///
/// Returns `OperationsError::Topology` if any entity ID is invalid.
pub fn filter_planar_edges(
    topo: &Topology,
    solid_id: SolidId,
    edge_ids: &[EdgeId],
) -> Result<Vec<EdgeId>, OperationsError> {
    // Solid-scoped: a hollow body's cavity faces carry filletable/planar edges
    // too, so walk outer + inner shells (CLAUDE.md, "Walking faces in a solid").
    let mut edge_faces: HashMap<usize, Vec<FaceId>> = HashMap::new();
    for fid in solid_faces(topo, solid_id)? {
        let face = topo.face(fid)?;
        let wire = topo.wire(face.outer_wire())?;
        for oe in wire.edges() {
            edge_faces.entry(oe.edge().index()).or_default().push(fid);
        }
    }

    let mut result = Vec::new();
    for &eid in edge_ids {
        if let Some(adj_faces) = edge_faces.get(&eid.index()) {
            let all_planar = adj_faces.iter().all(|&fid| {
                topo.face(fid)
                    .map(|f| matches!(f.surface(), FaceSurface::Plane { .. }))
                    .unwrap_or(false)
            });
            if all_planar {
                result.push(eid);
            }
        }
    }
    Ok(result)
}

/// Filter edges to only those the blend engine can fillet: manifold edges
/// (shared by exactly two distinct faces) that meet at a real (non-tangent)
/// angle.
///
/// Edges bordering a curved neighbour — including a previous fillet's NURBS
/// blend face — ARE filletable: the rolling-ball engine solves the true
/// ball-tangent contacts against any surface. The cases that genuinely have no
/// fillet are **tangent / G1** edges (the two faces meet smoothly, e.g. a
/// fillet face's contact line with its planar neighbour) and degenerate folds;
/// those are excluded here so callers never feed them to the engine.
///
/// `try_fillet` additionally guards each result with a manifold check, so a
/// permissive filter here cannot let a malformed solid through.
///
/// # Errors
///
/// Returns `OperationsError::Topology` if any entity ID is invalid.
pub fn filter_filletable_edges(
    topo: &Topology,
    solid_id: SolidId,
    edge_ids: &[EdgeId],
) -> Result<Vec<EdgeId>, OperationsError> {
    // Solid-scoped: a hollow body's cavity faces carry filletable/planar edges
    // too, so walk outer + inner shells (CLAUDE.md, "Walking faces in a solid").
    // Map each edge to its set of *distinct* adjacent faces, walking both outer
    // and inner (hole-boundary) wires — the same adjacency the fillet engine
    // sees. The set dedups a seam edge that a single face's wire lists twice.
    let mut edge_faces: HashMap<usize, HashSet<FaceId>> = HashMap::new();
    for fid in solid_faces(topo, solid_id)? {
        let face = topo.face(fid)?;
        let mut wires = vec![face.outer_wire()];
        wires.extend(face.inner_wires().iter().copied());
        for wid in wires {
            for oe in topo.wire(wid)?.edges() {
                edge_faces.entry(oe.edge().index()).or_default().insert(fid);
            }
        }
    }

    let mut result = Vec::new();
    for &eid in edge_ids {
        let Some(adj_faces) = edge_faces.get(&eid.index()) else {
            continue;
        };
        if adj_faces.len() != 2 {
            continue;
        }
        if edge_is_tangent(topo, eid, adj_faces)? {
            continue;
        }
        result.push(eid);
    }
    Ok(result)
}

/// Whether the two faces of `eid` meet tangentially (G1) — their effective
/// outward normals stay aligned at every interior edge sample, so there is no
/// real dihedral to round.
pub(crate) fn edge_is_tangent(
    topo: &Topology,
    eid: EdgeId,
    faces: &HashSet<FaceId>,
) -> Result<bool, OperationsError> {
    let mut it = faces.iter().copied();
    let (Some(face_a), Some(face_b)) = (it.next(), it.next()) else {
        return Ok(true);
    };
    edge_is_g1(topo, eid, face_a, face_b)
}

/// Material side of an analytic face, determined without any probe.
///
/// `Outward` is a boss-like wall whose effective outward normal points away
/// from the surface's axis (cylinder, cone) or centre (sphere, tube centre
/// for torus). `Inward` is a bore- or pocket-like wall whose effective
/// outward normal points toward that axis or centre. Planes and NURBS have
/// no axis or centre and are a typed refusal, never a guess.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MaterialSense {
    /// Boss-like wall: effective outward normal points away from the axis
    /// or centre.
    Outward,
    /// Bore- or pocket-like wall: effective outward normal points toward
    /// the axis or centre.
    Inward,
}

impl MaterialSense {
    /// Stable lowercase wire spelling (`"outward"` / `"inward"`).
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Outward => "outward",
            Self::Inward => "inward",
        }
    }
}

/// Per-edge convexity plus its signed dihedral.
///
/// `dihedral_angle` is the angle between the two effective outward normals
/// in `[0, pi]`, signed by the convexity verdict: positive for
/// [`EdgeConcavity::Convex`], negative for [`EdgeConcavity::Concave`], and
/// the unsigned near-zero angle for [`EdgeConcavity::Tangent`]. It is `None`
/// when the edge is [`EdgeConcavity::Unknown`] or the normal angle is
/// unavailable — an unknown edge never guesses, not even its angle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EdgeRelation {
    /// The classified edge.
    pub edge: EdgeId,
    /// Convex, concave, tangent, or unknown (never a guess).
    pub concavity: EdgeConcavity,
    /// Signed normal angle in radians, or `None` for unknown edges.
    pub dihedral_angle: Option<f64>,
}

impl EdgeConcavity {
    /// Stable lowercase wire spelling for the WASM boundary.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Convex => "convex",
            Self::Concave => "concave",
            Self::Tangent => "tangent",
            Self::Unknown => "unknown",
        }
    }
}

/// Default concavity probe for one manifold edge.
///
/// The formula is `0.05 * local_scale`, where `local_scale` is
/// `max(min(face_vertex_span(a), face_vertex_span(b)), edge_curve_span(edge))`.
/// Five percent sits well inside the classifier's 25 % local-scale cap,
/// leaving a 5x margin against reading a neighbouring feature, while staying
/// scale-relative from 1e-3 through 1e3. Returns an error when the local
/// scale is non-positive or non-finite.
///
/// # Errors
///
/// Returns `OperationsError::Topology` when the edge or its faces cannot be
/// traversed, or `InvalidInput` when the local scale is degenerate.
pub fn default_concavity_probe(
    topo: &Topology,
    edge: EdgeId,
    face_a: FaceId,
    face_b: FaceId,
) -> Result<f64, OperationsError> {
    let face_scale = face_vertex_span(topo, face_a)?.min(face_vertex_span(topo, face_b)?);
    let local_scale = face_scale.max(edge_curve_span(topo, edge)?);
    if !local_scale.is_finite() || local_scale <= 0.0 {
        return Err(OperationsError::InvalidInput {
            reason: "edge convexity default probe has degenerate local scale".into(),
        });
    }
    Ok(local_scale * 0.05)
}

fn signed_dihedral(concavity: EdgeConcavity, angle: Option<f64>) -> Option<f64> {
    match (concavity, angle) {
        (EdgeConcavity::Convex, Some(a)) => Some(a),
        (EdgeConcavity::Concave, Some(a)) => Some(-a),
        (EdgeConcavity::Tangent, Some(a)) => Some(a),
        _ => None,
    }
}

fn edge_relation_with_faces_and_probe(
    context: &EdgeRelationContext<'_>,
    edge: EdgeId,
    face_a: FaceId,
    face_b: FaceId,
    probe: f64,
) -> Result<EdgeRelation, OperationsError> {
    let (concavity, angle) = context.concavity_and_angle(edge, face_a, face_b, probe)?;
    Ok(EdgeRelation {
        edge,
        concavity,
        dihedral_angle: signed_dihedral(concavity, angle),
    })
}

/// Classify one edge of `solid`, with an optional caller probe.
///
/// This is the single-edge bulk path: adjacency is built once for the lookup
/// and the bulk classifier (one prepared point classifier per call) runs, so
/// a per-edge loop over [`solid_edge_relations`] stays consistent with this
/// call. `probe = None` selects [`default_concavity_probe`]
/// (`0.05 * local_scale`); `Some(p)` must be positive and finite. A
/// non-manifold edge, a self-seam, or a probe above 25 % of the local scale
/// reports [`EdgeConcavity::Unknown`] with no dihedral rather than a guess.
///
/// `edge_concavity`'s robust single-call semantics are untouched; this is the
/// bulk-path sibling the WASM bindings share.
///
/// # Errors
///
/// Returns `InvalidInput` for a non-positive or non-finite caller probe or
/// for an edge that is not part of `solid`, or propagates topology and
/// classification errors.
pub fn edge_relation(
    topo: &Topology,
    solid: SolidId,
    edge: EdgeId,
    probe: Option<f64>,
) -> Result<EdgeRelation, OperationsError> {
    if probe.is_some_and(|p| !p.is_finite() || p <= 0.0) {
        return Err(OperationsError::InvalidInput {
            reason: "edge concavity probe must be positive and finite".into(),
        });
    }
    // Ownership first: adjacency alone gives another solid's edge no faces,
    // which would read as the conservative `Unknown` meant for this solid's
    // own seams and non-manifold edges.
    if !remus_topology::explorer::solid_edges(topo, solid)?.contains(&edge) {
        return Err(OperationsError::InvalidInput {
            reason: format!("edge {} is not part of the solid", edge.index()),
        });
    }
    let adjacency = topo.build_adjacency(solid)?;
    let faces = adjacency.faces_for_edge(edge);
    if faces.len() != 2 || faces[0] == faces[1] {
        return Ok(EdgeRelation {
            edge,
            concavity: EdgeConcavity::Unknown,
            dihedral_angle: None,
        });
    }
    let (face_a, face_b) = (faces[0], faces[1]);
    let probe_eff = if let Some(p) = probe {
        p
    } else if let Ok(p) = default_concavity_probe(topo, edge, face_a, face_b) {
        p
    } else {
        return Ok(EdgeRelation {
            edge,
            concavity: EdgeConcavity::Unknown,
            dihedral_angle: None,
        });
    };
    let context = EdgeRelationContext::prepare(topo, solid)?;
    edge_relation_with_faces_and_probe(&context, edge, face_a, face_b, probe_eff)
}

/// Classify every edge of `solid` in one pass.
///
/// Adjacency and the point classifier are prepared once and reused for every
/// edge, so a 2 000-edge import costs one adjacency and one classifier
/// preparation plus one classification per edge — never the quadratic
/// rebuild a per-edge loop over the single-edge binding would pay.
/// `probe = None` selects the per-edge [`default_concavity_probe`]
/// (`0.05 * local_scale`, documented there); `Some(p)` applies one caller
/// probe to every edge and must be positive and finite. Edges that are non-manifold, self-seams, degenerate,
/// or probed above 25 % of their local scale report
/// [`EdgeConcavity::Unknown`] with no dihedral rather than a guess.
///
/// # Errors
///
/// Returns `InvalidInput` for a non-positive or non-finite caller probe, or
/// propagates topology and classification errors.
pub fn solid_edge_relations(
    topo: &Topology,
    solid: SolidId,
    probe: Option<f64>,
) -> Result<Vec<EdgeRelation>, OperationsError> {
    if probe.is_some_and(|p| !p.is_finite() || p <= 0.0) {
        return Err(OperationsError::InvalidInput {
            reason: "edge concavity probe must be positive and finite".into(),
        });
    }
    let adjacency = topo.build_adjacency(solid)?;
    let edges = remus_topology::explorer::solid_edges(topo, solid)?;
    let context = EdgeRelationContext::prepare(topo, solid)?;
    let mut out = Vec::with_capacity(edges.len());
    for edge in edges {
        let faces = adjacency.faces_for_edge(edge);
        if faces.len() != 2 || faces[0] == faces[1] {
            out.push(EdgeRelation {
                edge,
                concavity: EdgeConcavity::Unknown,
                dihedral_angle: None,
            });
            continue;
        }
        let (face_a, face_b) = (faces[0], faces[1]);
        let probe_eff = if let Some(p) = probe {
            p
        } else if let Ok(p) = default_concavity_probe(topo, edge, face_a, face_b) {
            p
        } else {
            out.push(EdgeRelation {
                edge,
                concavity: EdgeConcavity::Unknown,
                dihedral_angle: None,
            });
            continue;
        };
        out.push(edge_relation_with_faces_and_probe(
            &context, edge, face_a, face_b, probe_eff,
        )?);
    }
    Ok(out)
}

/// Material sense of one analytic face of `solid`, without any probe.
///
/// The verdict compares the face's effective outward normal
/// ([`effective_face_normal`], orientation-aware) against the surface's
/// radial direction at a boundary sample: cylinder and cone walls against
/// the axis-perpendicular radial, spheres against the centre radial, tori
/// against the tube-centre radial (closest point on the major circle). A
/// positive projection is [`MaterialSense::Outward`] (boss-like), a negative
/// one [`MaterialSense::Inward`] (bore- or pocket-like). This is a different
/// question from edge convexity: a bore's top rim is convex while its wall
/// is inward — report both, never collapse them.
///
/// Planes and NURBS have no axis or centre and are a typed
/// [`OperationsError::Unsupported`] refusal. A face that does not belong to
/// `solid`, a missing effective normal, or a degenerate radial is
/// [`OperationsError::InvalidInput`].
///
/// # Errors
///
/// Returns `Unsupported` for plane and NURBS faces, `InvalidInput` when the
/// face is not part of `solid` or its sense cannot be established, or
/// propagates topology errors.
pub fn face_material_sense(
    topo: &Topology,
    solid: SolidId,
    face: FaceId,
) -> Result<MaterialSense, OperationsError> {
    let owned = remus_topology::explorer::solid_faces(topo, solid)?;
    if !owned.contains(&face) {
        return Err(OperationsError::InvalidInput {
            reason: format!("face {} is not part of the solid", face.index()),
        });
    }
    let face_data = topo.face(face)?;
    match face_data.surface() {
        FaceSurface::Plane { .. } | FaceSurface::Nurbs(_) => {
            return Err(OperationsError::Unsupported {
                operation: "face_material_sense",
                reason: "material sense is defined only for cylinder, cone, sphere and torus faces"
                    .into(),
            });
        }
        FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => {}
    }
    let sample = {
        let verts = remus_topology::explorer::face_vertices(topo, face)?;
        let Some(first) = verts.first() else {
            return Err(OperationsError::InvalidInput {
                reason: "face has no vertices for material-sense sampling".into(),
            });
        };
        topo.vertex(*first)?.point()
    };
    let Some(normal) = effective_face_normal(topo, face, sample) else {
        return Err(OperationsError::InvalidInput {
            reason: "face material sense needs a finite effective outward normal".into(),
        });
    };
    let radial = match face_data.surface() {
        FaceSurface::Cylinder(cyl) => {
            let axis = cyl
                .axis()
                .normalize()
                .map_err(|_| OperationsError::InvalidInput {
                    reason: "cylindrical face has a degenerate axis".into(),
                })?;
            let to_point = sample - cyl.origin();
            let closest = cyl.origin() + axis * axis.dot(to_point);
            sample - closest
        }
        FaceSurface::Cone(cone) => {
            let axis = cone
                .axis()
                .normalize()
                .map_err(|_| OperationsError::InvalidInput {
                    reason: "conical face has a degenerate axis".into(),
                })?;
            let to_point = sample - cone.apex();
            let closest = cone.apex() + axis * axis.dot(to_point);
            sample - closest
        }
        FaceSurface::Sphere(sphere) => sample - sphere.center(),
        FaceSurface::Torus(torus) => {
            let axis = torus
                .z_axis()
                .normalize()
                .map_err(|_| OperationsError::InvalidInput {
                    reason: "toroidal face has a degenerate axis".into(),
                })?;
            let to_point = sample - torus.center();
            let along = axis * axis.dot(to_point);
            let in_plane = to_point - along;
            let planar_len = in_plane.length();
            if !planar_len.is_finite() || planar_len <= 0.0 {
                return Err(OperationsError::InvalidInput {
                    reason: "toroidal face sample lies on its symmetry axis".into(),
                });
            }
            let scale = torus.major_radius() / planar_len;
            if !scale.is_finite() {
                return Err(OperationsError::InvalidInput {
                    reason: "toroidal tube centre is not finite".into(),
                });
            }
            let circle_point = torus.center() + in_plane * scale;
            if !circle_point.x().is_finite()
                || !circle_point.y().is_finite()
                || !circle_point.z().is_finite()
            {
                return Err(OperationsError::InvalidInput {
                    reason: "toroidal tube centre is not finite".into(),
                });
            }
            sample - circle_point
        }
        FaceSurface::Plane { .. } | FaceSurface::Nurbs(_) => {
            return Err(OperationsError::Unsupported {
                operation: "face_material_sense",
                reason: "material sense is defined only for cylinder, cone, sphere and torus faces"
                    .into(),
            });
        }
    };
    if !radial.x().is_finite() || !radial.y().is_finite() || !radial.z().is_finite() {
        return Err(OperationsError::InvalidInput {
            reason: "face material sense radial is not finite".into(),
        });
    }
    let radial_len = radial.length();
    if !radial_len.is_finite() || radial_len <= 0.0 {
        return Err(OperationsError::InvalidInput {
            reason: "face material sense radial is degenerate".into(),
        });
    }
    let dot = normal.dot(radial);
    if !dot.is_finite() {
        return Err(OperationsError::InvalidInput {
            reason: "face material sense projection is not finite".into(),
        });
    }
    // Scale-relative zero: the dot is |radial| for cylinders and spheres,
    // sin(a)*|radial| for cones, |tube| for tori. Anything at or below
    // 1e-12 of the radial length is perpendicular, not a sense.
    if dot.abs() <= 1e-12 * radial_len {
        return Err(OperationsError::InvalidInput {
            reason: "face material sense is perpendicular to its radial".into(),
        });
    }
    Ok(if dot > 0.0 {
        MaterialSense::Outward
    } else {
        MaterialSense::Inward
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, deprecated)]

    use remus_math::mat::Mat4;
    use remus_topology::edge::EdgeCurve;
    use remus_topology::explorer::{solid_edges, solid_faces};

    use super::*;
    use crate::boolean::{BooleanOp, boolean};
    use crate::primitives::{make_box, make_cylinder};
    use crate::transform::transform_solid;

    #[test]
    fn opposing_planar_pairs_measure_box_dimensions() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();

        let pairs = opposing_planar_face_pairs(&topo, solid, Tolerance::default()).unwrap();
        assert_eq!(pairs.len(), 3);

        let mut measurements: Vec<(f64, f64)> = pairs
            .iter()
            .map(|pair| (pair.distance, pair.overlap_area))
            .collect();
        measurements.sort_by(|a, b| a.0.total_cmp(&b.0));
        assert!((measurements[0].0 - 2.0).abs() < 1.0e-9);
        assert!((measurements[0].1 - 12.0).abs() < 1.0e-9);
        assert!((measurements[1].0 - 3.0).abs() < 1.0e-9);
        assert!((measurements[1].1 - 8.0).abs() < 1.0e-9);
        assert!((measurements[2].0 - 4.0).abs() < 1.0e-9);
        assert!((measurements[2].1 - 6.0).abs() < 1.0e-9);
        assert!(
            pairs
                .iter()
                .all(|pair| { !pair.face_a_borders_blend && !pair.face_b_borders_blend })
        );
    }

    fn assert_query_budget_refusal(
        result: Result<Vec<OpposingPlanarFacePair>, OperationsError>,
        resource: &str,
    ) {
        assert!(
            matches!(result, Err(OperationsError::InvalidInput { reason }) if reason.contains(resource)),
            "expected {resource} budget refusal"
        );
    }

    #[test]
    fn planar_query_preserves_512_face_reference_ceiling_before_collection() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
        let face = solid_faces(&topo, solid).unwrap()[0];
        let shell = topo.add_shell(remus_topology::shell::Shell::new(vec![face; 512]).unwrap());
        let bounded = topo.add_solid(remus_topology::solid::Solid::new(shell, vec![]));
        assert!(
            opposing_planar_face_pairs(&topo, bounded, Tolerance::default())
                .unwrap()
                .is_empty()
        );
        let extra = topo.add_shell(remus_topology::shell::Shell::new(vec![face]).unwrap());
        topo.solid_mut(bounded).unwrap().add_inner_shell(extra);
        assert_query_budget_refusal(
            opposing_planar_face_pairs(&topo, bounded, Tolerance::default()),
            "face count",
        );
    }

    #[test]
    fn planar_query_bounds_topology_before_materializing_rings() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
        let face = solid_faces(&topo, solid).unwrap()[0];
        let original = topo.face(face).unwrap().outer_wire();
        let oriented = topo.wire(original).unwrap().edges()[0];
        let repeated =
            topo.add_wire(remus_topology::wire::Wire::new(vec![oriented; 32], true).unwrap());
        topo.set_face_boundary_wires(face, repeated, vec![])
            .unwrap();
        assert_eq!(solid_faces(&topo, solid).unwrap().len(), 6);
        assert_query_budget_refusal(
            opposing_planar_face_pairs_with_limits(
                &topo,
                solid,
                Tolerance::default(),
                PlanarQueryLimits {
                    topology_uses: 40,
                    ..PlanarQueryLimits::default()
                },
            ),
            "topology",
        );
        assert_eq!(topo.wire(repeated).unwrap().edges().len(), 32);
    }

    #[test]
    fn planar_query_bounds_hole_references_even_when_reused() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
        let face = solid_faces(&topo, solid).unwrap()[0];
        let outer = topo.face(face).unwrap().outer_wire();
        topo.set_face_boundary_wires(face, outer, vec![outer; 32])
            .unwrap();
        assert_query_budget_refusal(
            opposing_planar_face_pairs_with_limits(
                &topo,
                solid,
                Tolerance::default(),
                PlanarQueryLimits {
                    topology_uses: 40,
                    ..PlanarQueryLimits::default()
                },
            ),
            "topology",
        );
        assert_eq!(topo.face(face).unwrap().inner_wires().len(), 32);
    }

    #[test]
    fn planar_query_bounds_area_samples_before_curved_wire_sampling() {
        let mut topo = Topology::new();
        let solid = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
        assert_query_budget_refusal(
            opposing_planar_face_pairs_with_limits(
                &topo,
                solid,
                Tolerance::default(),
                PlanarQueryLimits {
                    samples: 256,
                    ..PlanarQueryLimits::default()
                },
            ),
            "boundary samples",
        );
        let pairs = opposing_planar_face_pairs(&topo, solid, Tolerance::default()).unwrap();
        assert_eq!(pairs.len(), 1);
        assert!((pairs[0].distance - 2.0).abs() < 1e-9);
        assert!((pairs[0].face_area_a - std::f64::consts::PI).abs() < 1e-9);
        let inscribed_area = 16.0 * (std::f64::consts::TAU / 32.0).sin();
        assert!((pairs[0].overlap_area - inscribed_area).abs() < 1e-6);
    }

    /// Box `size` drilled through Z by real boolean cuts of radius `r`.
    fn drilled_box(topo: &mut Topology, size: [f64; 3], r: f64, holes: &[(f64, f64)]) -> SolidId {
        let mut solid = make_box(topo, size[0], size[1], size[2]).unwrap();
        for &(x, y) in holes {
            let tool = make_cylinder(topo, r, size[2] + 4.0).unwrap();
            transform_solid(topo, tool, &Mat4::translation(x, y, -2.0)).unwrap();
            solid = boolean(topo, BooleanOp::Cut, solid, tool).unwrap();
        }
        solid
    }

    /// The work the query spends on `solid` under the default limits.
    fn default_spend(topo: &Topology, solid: SolidId) -> (Vec<OpposingPlanarFacePair>, usize) {
        opposing_planar_face_pairs_metered(
            topo,
            solid,
            Tolerance::default(),
            PlanarQueryLimits::default(),
        )
        .unwrap()
    }

    /// Asserts `limit = spent` admits the query with the default-limit result
    /// and `spent - 1` refuses it on polygon work.
    fn assert_exact_work_boundary(topo: &Topology, solid: SolidId) -> usize {
        let (pairs, spent) = default_spend(topo, solid);
        let limits = PlanarQueryLimits {
            work: spent,
            ..PlanarQueryLimits::default()
        };
        assert_eq!(
            opposing_planar_face_pairs_with_limits(topo, solid, Tolerance::default(), limits)
                .unwrap(),
            pairs
        );
        assert_query_budget_refusal(
            opposing_planar_face_pairs_with_limits(
                topo,
                solid,
                Tolerance::default(),
                PlanarQueryLimits {
                    work: spent - 1,
                    ..limits
                },
            ),
            "polygon work",
        );
        spent
    }

    #[test]
    fn planar_query_charges_all_face_pairs_and_accepts_exact_work_boundary() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
        let expected = opposing_planar_face_pairs(&topo, solid, Tolerance::default()).unwrap();
        assert_eq!(expected.len(), 3);
        // Each opposing pair projects onto one shared rectangle: 2 * 8 to
        // project and box both rings, 1 for the ring pair, and the metered
        // boolean of two coincident squares: 3 * 8 input scans, 2 * 4 * (4 + 1)
        // split tests with no split parameters, 2 * 4 * (3 * 4 + 1)
        // midpoint classifications, and 2 * 4 + 4 * 4 + 4 tracing four
        // selected edges whose junctions each leave once.
        let per_pair = 16 + 1 + 24 + 40 + 104 + 8 + 20;
        let spent = assert_exact_work_boundary(&topo, solid);
        assert_eq!(spent, 24 * 24 + 3 * per_pair);
    }

    #[test]
    fn planar_query_preserves_hole_overlap_and_cumulative_hole_budget() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 2.0).unwrap();
        let hole_solid = make_box(&mut topo, 2.0, 3.0, 2.0).unwrap();
        transform_solid(&mut topo, hole_solid, &Mat4::translation(4.0, 4.0, 0.0)).unwrap();
        let hole_faces = solid_faces(&topo, hole_solid).unwrap();
        for face in solid_faces(&topo, solid).unwrap() {
            let normal = topo.face(face).unwrap().effective_plane_normal().unwrap();
            if normal.z().abs() < 0.9 {
                continue;
            }
            let hole_face = hole_faces
                .iter()
                .copied()
                .find(|&f| {
                    topo.face(f)
                        .unwrap()
                        .effective_plane_normal()
                        .unwrap()
                        .dot(normal)
                        > 0.9
                })
                .unwrap();
            let outer = topo.face(face).unwrap().outer_wire();
            let hole = topo.face(hole_face).unwrap().outer_wire();
            topo.set_face_boundary_wires(face, outer, vec![hole])
                .unwrap();
        }
        let pairs = opposing_planar_face_pairs(&topo, solid, Tolerance::default()).unwrap();
        let cap_pair = pairs.iter().find(|p| p.normal.z().abs() > 0.9).unwrap();
        assert!((cap_pair.overlap_area - 94.0).abs() < 1e-6);
        assert!((cap_pair.face_area_a - 94.0).abs() < 1e-9);
        // All four cap ring pairs (outer/outer, both hole/outer, hole/hole)
        // draw on the one shared budget, down to the last unit.
        assert_exact_work_boundary(&topo, solid);
    }

    /// OpenZCAD's planar-distance proof fixture: a 40 x 30 x 20 box with four
    /// Ø3 through holes. The worst-case pair reservation charged its cap pair
    /// 1,125,909,576 units and refused it; metered, the whole query spends
    /// 51,564 of the default 1,000,000,000.
    #[test]
    fn planar_query_admits_a_box_with_four_through_holes() {
        let mut topo = Topology::new();
        let holes: Vec<(f64, f64)> = (0..4).map(|i| (5.0 + 5.0 * f64::from(i), 5.0)).collect();
        let solid = drilled_box(&mut topo, [40.0, 30.0, 20.0], 1.5, &holes);
        let (pairs, spent) = default_spend(&topo, solid);
        assert_eq!(pairs.len(), 3);
        let caps = pairs
            .iter()
            .find(|p| p.normal.z().abs() > 0.9)
            .expect("the drilled caps still oppose each other");
        assert!((caps.distance - 20.0).abs() < 1e-9);
        let drilled_area = 4.0f64.mul_add(-std::f64::consts::PI * 1.5 * 1.5, 1200.0);
        assert!((caps.face_area_a - drilled_area).abs() < 1e-6);
        assert!(caps.overlap_area > 1171.0 && caps.overlap_area <= caps.face_area_a);
        assert!(
            spent < PlanarQueryLimits::default().work / 10_000,
            "four through holes spent {spent} work units"
        );
        assert_exact_work_boundary(&topo, solid);
    }

    /// OpenZCAD's dense imported plate: 100 x 100 x 6 with a 7 x 10 grid of
    /// Ø1 through holes. Every one of the 4,900 hole/hole ring pairs used to
    /// be reserved as if all 32-point rings crossed everywhere (341,942,754,616
    /// units in all); now the 4,830 disjoint pairs cost one unit each and only
    /// the 70 coincident ones run the polygon boolean.
    #[test]
    fn planar_query_admits_a_plate_with_seventy_through_holes() {
        let mut topo = Topology::new();
        let plate = make_box(&mut topo, 100.0, 100.0, 6.0).unwrap();
        let mut top = Vec::new();
        let mut bottom = Vec::new();
        for x in 0..7 {
            for y in 0..10 {
                let pin = make_cylinder(&mut topo, 0.5, 6.0).unwrap();
                let at =
                    Mat4::translation(10.0 + 12.0 * f64::from(x), 5.0 + 9.0 * f64::from(y), 0.0);
                transform_solid(&mut topo, pin, &at).unwrap();
                for face in solid_faces(&topo, pin).unwrap() {
                    let data = topo.face(face).unwrap();
                    match data.effective_plane_normal() {
                        Some(n) if n.z() > 0.9 => top.push(data.outer_wire()),
                        Some(n) if n.z() < -0.9 => bottom.push(data.outer_wire()),
                        _ => {}
                    }
                }
            }
        }
        for face in solid_faces(&topo, plate).unwrap() {
            let data = topo.face(face).unwrap();
            let holes = match data.effective_plane_normal() {
                Some(n) if n.z() > 0.9 => top.clone(),
                Some(n) if n.z() < -0.9 => bottom.clone(),
                _ => continue,
            };
            let outer = data.outer_wire();
            topo.set_face_boundary_wires(face, outer, holes).unwrap();
        }
        let (pairs, spent) = default_spend(&topo, plate);
        let caps = pairs
            .iter()
            .find(|p| p.normal.z().abs() > 0.9)
            .expect("the drilled caps still oppose each other");
        assert!((caps.distance - 6.0).abs() < 1e-9);
        let drilled_area = 70.0f64.mul_add(-std::f64::consts::PI * 0.25, 10_000.0);
        assert!((caps.face_area_a - drilled_area).abs() < 1e-6);
        assert!(caps.overlap_area > 9_940.0 && caps.overlap_area <= caps.face_area_a);
        assert!(
            spent < PlanarQueryLimits::default().work / 100,
            "seventy through holes spent {spent} work units"
        );
    }

    /// Two closed ring pairs whose boxes are just over twice the tolerance
    /// apart intersect in nothing, so skipping them is exact; rings within
    /// the tolerance are never skipped.
    #[test]
    fn planar_query_skips_only_ring_pairs_the_boolean_cannot_join() {
        let tolerance = Tolerance::default();
        let square = |x: f64| {
            vec![
                Point2::new(x, 0.0),
                Point2::new(x + 1.0, 0.0),
                Point2::new(x + 1.0, 1.0),
                Point2::new(x, 1.0),
            ]
        };
        let a = square(0.0);
        for (offset, skipped) in [
            (1.0 + 2.5 * tolerance.linear, true),
            (1.0 + 0.5 * tolerance.linear, false),
            (0.5, false),
        ] {
            let b = square(offset);
            assert_eq!(
                rings_disjoint(ring_box(&a), ring_box(&b), tolerance),
                skipped
            );
            let mut budget = usize::MAX;
            let area = polygon_overlap_area(&a, &b, tolerance, &mut budget).unwrap();
            if skipped {
                assert!(area.to_bits() == 0.0f64.to_bits());
            }
        }
        let poisoned = vec![
            Point2::new(f64::NAN, 0.0),
            Point2::new(5.0, 0.0),
            Point2::new(5.0, 1.0),
        ];
        assert!(ring_box(&poisoned).is_none());
        assert!(!rings_disjoint(
            ring_box(&a),
            ring_box(&poisoned),
            tolerance
        ));
    }

    /// Wire `points` (in XY) at height `z` as the outer boundary of `face`.
    fn rewire_face(topo: &mut Topology, face: FaceId, points: &[(f64, f64)], z: f64) {
        let points: Vec<Point3> = points.iter().map(|&(x, y)| Point3::new(x, y, z)).collect();
        let wire = remus_topology::builder::make_polygon_wire(topo, &points, 1e-7).unwrap();
        topo.set_face_boundary_wires(face, wire, vec![]).unwrap();
    }

    /// A comb in XY: a base strip along X with `teeth` columns rising in Y.
    fn comb(teeth: u32) -> Vec<(f64, f64)> {
        let span = 2.0 * f64::from(teeth);
        let mut points = vec![(0.0, 0.0), (span, 0.0), (span, 1.0)];
        for i in (0..teeth).rev() {
            let x = 2.0 * f64::from(i);
            points.extend([
                (x + 1.5, 1.0),
                (x + 1.5, span),
                (x + 0.5, span),
                (x + 0.5, 1.0),
            ]);
        }
        points.push((0.0, 1.0));
        points
    }

    /// Hostile caps: a comb of columns over a comb of rows crosses at four
    /// points per tooth pair, so 400 teeth each (1,604 edges per cap, well
    /// inside the topology and sample limits) build a 640,000-crossing
    /// arrangement whose midpoint classification alone would cost about
    /// 3 * 10^9 units. The default budget refuses it at that metered charge,
    /// before the phase runs, having spent only the preparation and split
    /// tests.
    #[test]
    fn planar_query_refuses_crossing_comb_caps_under_default_limits() {
        let mut topo = Topology::new();
        let teeth = 400;
        let span = 2.0 * f64::from(teeth);
        let solid = make_box(&mut topo, span, span, 1.0).unwrap();
        let columns = comb(teeth);
        let rows: Vec<(f64, f64)> = columns.iter().map(|&(x, y)| (y, x)).collect();
        for face in solid_faces(&topo, solid).unwrap() {
            match topo.face(face).unwrap().effective_plane_normal() {
                Some(n) if n.z() > 0.9 => rewire_face(&mut topo, face, &columns, 1.0),
                Some(n) if n.z() < -0.9 => rewire_face(&mut topo, face, &rows, 0.0),
                _ => {}
            }
        }
        assert_query_budget_refusal(
            opposing_planar_face_pairs(&topo, solid, Tolerance::default()),
            "polygon work",
        );

        // The same caps with eight teeth are an ordinary query.
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 16.0, 16.0, 1.0).unwrap();
        let columns = comb(8);
        let rows: Vec<(f64, f64)> = columns.iter().map(|&(x, y)| (y, x)).collect();
        for face in solid_faces(&topo, solid).unwrap() {
            match topo.face(face).unwrap().effective_plane_normal() {
                Some(n) if n.z() > 0.9 => rewire_face(&mut topo, face, &columns, 1.0),
                Some(n) if n.z() < -0.9 => rewire_face(&mut topo, face, &rows, 0.0),
                _ => {}
            }
        }
        let pairs = opposing_planar_face_pairs(&topo, solid, Tolerance::default()).unwrap();
        assert!(
            pairs
                .iter()
                .any(|p| p.normal.z().abs() > 0.9 && p.overlap_area > 0.0)
        );
    }

    #[test]
    fn planar_query_preflight_includes_cavity_and_curved_face_wires() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let cavity = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
        let shell = topo.solid(cavity).unwrap().outer_shell();
        topo.solid_mut(solid).unwrap().add_inner_shell(shell);
        let wall = solid_faces(&topo, cavity)
            .unwrap()
            .into_iter()
            .find(|&f| !topo.face(f).unwrap().surface().is_planar())
            .unwrap();
        let edge = topo
            .wire(topo.face(wall).unwrap().outer_wire())
            .unwrap()
            .edges()[0];
        let repeated =
            topo.add_wire(remus_topology::wire::Wire::new(vec![edge; 64], true).unwrap());
        topo.set_face_boundary_wires(wall, repeated, vec![])
            .unwrap();
        assert_query_budget_refusal(
            opposing_planar_face_pairs_with_limits(
                &topo,
                solid,
                Tolerance::default(),
                PlanarQueryLimits {
                    topology_uses: 64,
                    ..PlanarQueryLimits::default()
                },
            ),
            "topology",
        );
    }

    #[test]
    fn planar_query_keeps_cavity_pairs_and_bounds_empty_shell_references() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 8.0, 8.0, 8.0).unwrap();
        let cavity = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        transform_solid(&mut topo, cavity, &Mat4::translation(3.0, 3.0, 3.0)).unwrap();
        for face in solid_faces(&topo, cavity).unwrap() {
            topo.face_mut(face).unwrap().set_reversed(true);
        }
        let shell = topo.solid(cavity).unwrap().outer_shell();
        topo.solid_mut(solid).unwrap().add_inner_shell(shell);
        let pairs = opposing_planar_face_pairs(&topo, solid, Tolerance::default()).unwrap();
        assert_eq!(pairs.len(), 9);
        assert_eq!(
            pairs
                .iter()
                .filter(|p| (p.distance - 3.0).abs() < 1e-9 && (p.overlap_area - 4.0).abs() < 1e-9)
                .count(),
            6
        );
        let empty = topo.add_shell(remus_topology::shell::Shell::empty());
        for _ in 0..32 {
            topo.solid_mut(solid).unwrap().add_inner_shell(empty);
        }
        assert_query_budget_refusal(
            opposing_planar_face_pairs_with_limits(
                &topo,
                solid,
                Tolerance::default(),
                PlanarQueryLimits {
                    topology_uses: 32,
                    ..PlanarQueryLimits::default()
                },
            ),
            "topology",
        );
    }

    #[test]
    fn filletable_edges_all_planar_box() {
        let mut topo = Topology::new();
        let cube = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edges(&topo, cube).unwrap();
        let filletable = filter_filletable_edges(&topo, cube, &edges).unwrap();
        assert_eq!(
            filletable.len(),
            edges.len(),
            "every box edge is plane↔plane and filletable"
        );
        assert_eq!(edges.len(), 12);
    }

    #[test]
    fn filletable_edges_keep_nontangent_blend_edges_drop_tangent() {
        // A single rolling-ball fillet makes a watertight solid with a
        // cylindrical blend face. Its blend-border edges split into tangent/G1
        // contact lines (degenerate → excluded) and real-angle end-caps (→ kept).
        let mut topo = Topology::new();
        let cube = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edges(&topo, cube).unwrap();
        let filleted =
            crate::fillet::fillet_rolling_ball(&mut topo, cube, &[edges[0]], 1.0).unwrap();
        let r_edges = solid_edges(&topo, filleted).unwrap();
        let filletable: HashSet<usize> = filter_filletable_edges(&topo, filleted, &r_edges)
            .unwrap()
            .iter()
            .map(|e| e.index())
            .collect();

        let sh = topo
            .shell(topo.solid(filleted).unwrap().outer_shell())
            .unwrap();
        // The blend face, whatever surface type it carries. A straight box
        // edge blends to an exact cylinder; only curved neighbours give NURBS.
        let blend_faces: HashSet<usize> = sh
            .faces()
            .iter()
            .filter(|&&f| !topo.face(f).unwrap().surface().is_planar())
            .map(|f| f.index())
            .collect();
        assert!(
            !blend_faces.is_empty(),
            "first fillet must create a blend face"
        );

        let mut ef: HashMap<usize, HashSet<FaceId>> = HashMap::new();
        for &fid in sh.faces() {
            for oe in topo
                .wire(topo.face(fid).unwrap().outer_wire())
                .unwrap()
                .edges()
            {
                ef.entry(oe.edge().index()).or_default().insert(fid);
            }
        }

        let (mut saw_kept, mut saw_dropped_tangent) = (false, false);
        for &e in &r_edges {
            let Some(fs) = ef.get(&e.index()) else {
                continue;
            };
            if fs.len() != 2 || !fs.iter().any(|f| blend_faces.contains(&f.index())) {
                continue;
            }
            if edge_is_tangent(&topo, e, fs).unwrap() {
                assert!(
                    !filletable.contains(&e.index()),
                    "tangent blend-contact edge {} must be excluded",
                    e.index()
                );
                saw_dropped_tangent = true;
            } else {
                assert!(
                    filletable.contains(&e.index()),
                    "non-tangent blend-adjacent edge {} must stay filletable",
                    e.index()
                );
                saw_kept = true;
            }
        }
        assert!(saw_kept, "expected a kept non-tangent NURBS-blend edge");
        assert!(
            saw_dropped_tangent,
            "expected an excluded tangent contact edge"
        );
    }

    #[test]
    fn reversed_plane_effective_normal_flips_once() {
        let mut topo = Topology::new();
        let cube = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let face = solid_faces(&topo, cube).unwrap()[0];
        let point = Point3::new(1.0, 1.0, 1.0);
        let outward = effective_face_normal(&topo, face, point).unwrap();
        topo.face_mut(face).unwrap().set_reversed(true);
        let reversed = effective_face_normal(&topo, face, point).unwrap();
        assert!((reversed + outward).length() < 1e-12);
    }

    #[test]
    fn l_notch_reflex_edge_is_concave() {
        let mut topo = Topology::new();
        let base = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
        let tool = make_box(&mut topo, 12.0, 12.0, 6.0).unwrap();
        transform_solid(&mut topo, tool, &Mat4::translation(10.0, 10.0, 5.0)).unwrap();
        let notched = boolean(&mut topo, BooleanOp::Cut, base, tool).unwrap();
        let reflex = solid_edges(&topo, notched)
            .unwrap()
            .into_iter()
            .find(|&edge| {
                let data = topo.edge(edge).unwrap();
                [data.start(), data.end()].iter().all(|vertex| {
                    let point = topo.vertex(*vertex).unwrap().point();
                    (point.x() - 10.0).abs() < 1e-9 && (point.y() - 10.0).abs() < 1e-9
                })
            })
            .expect("inner vertical edge");

        assert_eq!(
            edge_concavity(&topo, notched, reflex, 0.01).unwrap(),
            EdgeConcavity::Concave
        );
    }

    #[test]
    fn post_base_and_hole_rim_have_opposite_convexity() {
        let mut post_topo = Topology::new();
        let plate = make_box(&mut post_topo, 80.0, 40.0, 8.0).unwrap();
        let post = make_cylinder(&mut post_topo, 10.0, 32.0).unwrap();
        transform_solid(&mut post_topo, post, &Mat4::translation(40.0, 20.0, 8.0)).unwrap();
        let posted = boolean(&mut post_topo, BooleanOp::Fuse, plate, post).unwrap();
        let post_rim = solid_edges(&post_topo, posted)
            .unwrap()
            .into_iter()
            .find(|&edge| {
                let data = post_topo.edge(edge).unwrap();
                matches!(data.curve(), EdgeCurve::Circle(_))
                    && (post_topo.vertex(data.start()).unwrap().point().z() - 8.0).abs() < 1e-9
            })
            .expect("post-base rim");
        assert_eq!(
            edge_concavity(&post_topo, posted, post_rim, 0.05).unwrap(),
            EdgeConcavity::Concave
        );
        assert_eq!(
            edge_concavity(&post_topo, posted, post_rim, 100.0).unwrap(),
            EdgeConcavity::Unknown,
            "a probe far outside the local faces must not produce a verdict"
        );

        let mut bore_topo = Topology::new();
        let plate = make_box(&mut bore_topo, 20.0, 20.0, 6.0).unwrap();
        let drill = make_cylinder(&mut bore_topo, 3.0, 10.0).unwrap();
        transform_solid(&mut bore_topo, drill, &Mat4::translation(10.0, 10.0, -2.0)).unwrap();
        let bored = boolean(&mut bore_topo, BooleanOp::Cut, plate, drill).unwrap();
        let bore_rim = solid_edges(&bore_topo, bored)
            .unwrap()
            .into_iter()
            .find(|&edge| {
                let data = bore_topo.edge(edge).unwrap();
                matches!(data.curve(), EdgeCurve::Circle(_))
                    && (bore_topo.vertex(data.start()).unwrap().point().z() - 6.0).abs() < 1e-9
            })
            .expect("top bore rim");
        assert_eq!(
            edge_concavity(&bore_topo, bored, bore_rim, 0.05).unwrap(),
            EdgeConcavity::Convex
        );
    }

    #[test]
    fn cylinder_self_seam_is_unknown_not_an_adjacency() {
        let mut topo = Topology::new();
        let cylinder = make_cylinder(&mut topo, 2.0, 4.0).unwrap();
        let adjacency = topo.build_adjacency(cylinder).unwrap();
        let seam = solid_edges(&topo, cylinder)
            .unwrap()
            .into_iter()
            .find(|&edge| {
                let faces = adjacency.faces_for_edge(edge);
                faces.len() == 2 && faces[0] == faces[1]
            })
            .expect("periodic wall seam");

        assert_eq!(
            edge_concavity(&topo, cylinder, seam, 0.01).unwrap(),
            EdgeConcavity::Unknown
        );
        let face = adjacency.faces_for_edge(seam)[0];
        assert!(!edge_is_g1(&topo, seam, face, face).unwrap());
    }

    #[test]
    fn closed_disc_cap_rim_still_classifies_convex() {
        let mut topo = Topology::new();
        let cylinder = make_cylinder(&mut topo, 2.0, 4.0).unwrap();
        let rim = solid_edges(&topo, cylinder)
            .unwrap()
            .into_iter()
            .find(|&edge| {
                let data = topo.edge(edge).unwrap();
                matches!(data.curve(), EdgeCurve::Circle(_))
                    && topo.vertex(data.start()).unwrap().point().z().abs() < 1e-9
            })
            .expect("bottom cap rim");

        assert_eq!(
            edge_concavity(&topo, cylinder, rim, 0.05).unwrap(),
            EdgeConcavity::Convex
        );
    }

    #[test]
    fn blend_spring_edges_are_tangent_by_aligned_normals() {
        let mut topo = Topology::new();
        let cube = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edge = solid_edges(&topo, cube).unwrap()[0];
        let filleted = crate::fillet::fillet_rolling_ball(&mut topo, cube, &[edge], 1.0).unwrap();
        let band = solid_faces(&topo, filleted)
            .unwrap()
            .into_iter()
            .find(|&face| !topo.face(face).unwrap().surface().is_planar())
            .expect("blend band");
        let adjacency = topo.build_adjacency(filleted).unwrap();
        let tangent = solid_edges(&topo, filleted)
            .unwrap()
            .into_iter()
            .filter(|&edge| {
                let faces = adjacency.faces_for_edge(edge);
                faces.len() == 2
                    && faces.contains(&band)
                    && edge_concavity(&topo, filleted, edge, 0.01).unwrap()
                        == EdgeConcavity::Tangent
            })
            .count();

        assert_eq!(tangent, 2, "one spring contact on each side of the band");
    }

    #[test]
    fn imported_step_springs_are_tangent_and_a_bore_is_negative() {
        let fillet_step = include_str!("../../io/tests/data/openzcad_e_analytic_fillet_plate.step");
        let mut fillet_topo = Topology::new();
        let fillet_solid =
            remus_io::step::reader::read_step(fillet_step, &mut fillet_topo).unwrap()[0];
        let fillet_adjacency = fillet_topo.build_adjacency(fillet_solid).unwrap();
        let tangent: Vec<EdgeId> = solid_edges(&fillet_topo, fillet_solid)
            .unwrap()
            .into_iter()
            .filter(|&edge| {
                edge_concavity(&fillet_topo, fillet_solid, edge, 0.01).unwrap()
                    == EdgeConcavity::Tangent
            })
            .collect();
        assert_eq!(tangent.len(), 8, "four bands with two spring contacts each");
        assert!(tangent.iter().all(|&edge| {
            fillet_adjacency.faces_for_edge(edge).iter().any(|&face| {
                matches!(
                    fillet_topo.face(face).unwrap().surface(),
                    FaceSurface::Cylinder(cylinder)
                        if (cylinder.radius() - 3.0).abs() < 1e-9
                )
            })
        }));

        let bore_step = include_str!("../../io/tests/data/openzcad_a_export_bored_plate.step");
        let mut bore_topo = Topology::new();
        let bore_solid = remus_io::step::reader::read_step(bore_step, &mut bore_topo).unwrap()[0];
        let bore_tangent = solid_edges(&bore_topo, bore_solid)
            .unwrap()
            .into_iter()
            .filter(|&edge| {
                edge_concavity(&bore_topo, bore_solid, edge, 0.01).unwrap()
                    == EdgeConcavity::Tangent
            })
            .count();
        assert_eq!(bore_tangent, 0, "a plain bore has no G1 spring contacts");
    }

    // ── B16: bulk edge relations + face material sense ──────────────
    //
    // Closed-form fixtures at 1e-3, 1 and 1e3 plus one rigid placement.
    // The default probe is 0.05 * local_scale per edge; every cell below
    // runs with `None` except the oversized-probe Unknown pin.

    fn circle_rim_at_z(
        topo: &Topology,
        solid: remus_topology::solid::SolidId,
        z: f64,
        tol: f64,
    ) -> EdgeId {
        solid_edges(topo, solid)
            .unwrap()
            .into_iter()
            .find(|&edge| {
                let data = topo.edge(edge).unwrap();
                matches!(data.curve(), EdgeCurve::Circle(_))
                    && (topo.vertex(data.start()).unwrap().point().z() - z).abs() < tol
            })
            .unwrap_or_else(|| panic!("no circle rim at z={z}"))
    }

    fn cylinder_wall_faces(topo: &Topology, solid: remus_topology::solid::SolidId) -> Vec<FaceId> {
        solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
            .collect()
    }

    fn planar_faces(topo: &Topology, solid: remus_topology::solid::SolidId) -> Vec<FaceId> {
        solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .filter(|&face| topo.face(face).unwrap().surface().is_planar())
            .collect()
    }

    #[test]
    fn b16_box_edges_convex_with_pi_half_dihedral_across_scales() {
        for scale in [1e-3, 1.0, 1e3] {
            let mut topo = Topology::new();
            let solid = make_box(&mut topo, scale, scale, scale).unwrap();
            let relations = solid_edge_relations(&topo, solid, None).unwrap();
            assert_eq!(relations.len(), 12, "box has 12 edges at scale {scale:e}");
            for rel in &relations {
                assert_eq!(
                    rel.concavity,
                    EdgeConcavity::Convex,
                    "box edge must be convex at scale {scale:e}"
                );
                let angle = rel.dihedral_angle.expect("convex carries a signed angle");
                assert!(
                    (angle - std::f64::consts::FRAC_PI_2).abs() < 1e-9,
                    "box dihedral must be +pi/2, got {angle} at scale {scale:e}"
                );
                // Bulk agrees with the single-edge bulk path.
                let single = edge_relation(&topo, solid, rel.edge, None).unwrap();
                assert_eq!(single.concavity, rel.concavity);
                assert_eq!(single.dihedral_angle, rel.dihedral_angle);
            }
            // Oversized probe never guesses.
            let huge = solid_edge_relations(&topo, solid, Some(scale * 10.0)).unwrap();
            assert!(
                huge.iter()
                    .all(|r| r.concavity == EdgeConcavity::Unknown && r.dihedral_angle.is_none()),
                "a probe far above local scale must be Unknown at scale {scale:e}"
            );
            // Zero and negative probes are typed refusals, not Unknown.
            assert!(solid_edge_relations(&topo, solid, Some(0.0)).is_err());
            assert!(solid_edge_relations(&topo, solid, Some(-1.0)).is_err());
            assert!(edge_relation(&topo, solid, relations[0].edge, Some(0.0)).is_err());
        }
    }

    #[test]
    fn b16_box_convexity_survives_rigid_placement() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 4.0, 2.0, 1.0).unwrap();
        let placed =
            Mat4::translation(17.0, -23.0, 31.0) * Mat4::rotation_y(0.37) * Mat4::rotation_x(0.61);
        transform_solid(&mut topo, solid, &placed).unwrap();
        let relations = solid_edge_relations(&topo, solid, None).unwrap();
        assert_eq!(relations.len(), 12);
        for rel in &relations {
            assert_eq!(rel.concavity, EdgeConcavity::Convex);
            let angle = rel.dihedral_angle.unwrap();
            assert!((angle - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
        }
    }

    #[test]
    fn b16_pocket_floor_wall_edges_concave() {
        for scale in [1e-3, 1.0, 1e3] {
            let mut topo = Topology::new();
            let base = make_box(&mut topo, 20.0 * scale, 20.0 * scale, 10.0 * scale).unwrap();
            let tool = make_box(&mut topo, 10.0 * scale, 10.0 * scale, 6.0 * scale).unwrap();
            transform_solid(
                &mut topo,
                tool,
                &Mat4::translation(5.0 * scale, 5.0 * scale, 4.0 * scale),
            )
            .unwrap();
            let pocket = boolean(&mut topo, BooleanOp::Cut, base, tool).unwrap();
            // Floor-wall rim: circle-free rectangular loop at the pocket floor.
            let floor_z = 4.0 * scale;
            let rim: Vec<EdgeRelation> = solid_edge_relations(&topo, pocket, None)
                .unwrap()
                .into_iter()
                .filter(|rel| {
                    let data = topo.edge(rel.edge).unwrap();
                    [data.start(), data.end()].iter().all(|v| {
                        (topo.vertex(*v).unwrap().point().z() - floor_z).abs()
                            < 1e-9 * scale.max(1.0)
                    }) && matches!(data.curve(), EdgeCurve::Line)
                })
                .collect();
            assert_eq!(rim.len(), 4, "pocket floor must have 4 wall edges");
            for rel in &rim {
                assert_eq!(rel.concavity, EdgeConcavity::Concave);
                let angle = rel.dihedral_angle.expect("concave carries a signed angle");
                assert!(
                    (angle + std::f64::consts::FRAC_PI_2).abs() < 1e-6,
                    "pocket dihedral must be -pi/2, got {angle}"
                );
            }
        }
    }

    #[test]
    fn b16_boss_rims_and_wall_sense() {
        for scale in [1e-3, 1.0, 1e3] {
            let mut topo = Topology::new();
            let plate = make_box(&mut topo, 80.0 * scale, 40.0 * scale, 8.0 * scale).unwrap();
            let post = make_cylinder(&mut topo, 10.0 * scale, 32.0 * scale).unwrap();
            transform_solid(
                &mut topo,
                post,
                &Mat4::translation(40.0 * scale, 20.0 * scale, 8.0 * scale),
            )
            .unwrap();
            let posted = boolean(&mut topo, BooleanOp::Fuse, plate, post).unwrap();
            let tol = 1e-9 * scale.max(1.0);
            let foot = circle_rim_at_z(&topo, posted, 8.0 * scale, tol);
            let top = circle_rim_at_z(&topo, posted, 40.0 * scale, tol);
            // Foot rim is re-entrant (plate top meets boss wall): concave.
            // Top rim is the boss cap meeting its wall: a 90-degree convex edge.
            assert_eq!(
                edge_relation(&topo, posted, foot, None).unwrap().concavity,
                EdgeConcavity::Concave
            );
            assert_eq!(
                edge_relation(&topo, posted, top, None).unwrap().concavity,
                EdgeConcavity::Convex
            );
            let walls = cylinder_wall_faces(&topo, posted);
            assert!(!walls.is_empty(), "boss must keep a cylindrical wall");
            for wall in walls {
                assert_eq!(
                    face_material_sense(&topo, posted, wall).unwrap(),
                    MaterialSense::Outward,
                    "boss wall must read outward"
                );
            }
        }
    }

    #[test]
    fn b16_through_bore_rims_convex_wall_inward() {
        for scale in [1e-3, 1.0, 1e3] {
            let mut topo = Topology::new();
            let plate = make_box(&mut topo, 20.0 * scale, 20.0 * scale, 6.0 * scale).unwrap();
            let drill = make_cylinder(&mut topo, 3.0 * scale, 10.0 * scale).unwrap();
            transform_solid(
                &mut topo,
                drill,
                &Mat4::translation(10.0 * scale, 10.0 * scale, -2.0 * scale),
            )
            .unwrap();
            let bored = boolean(&mut topo, BooleanOp::Cut, plate, drill).unwrap();
            let tol = 1e-9 * scale.max(1.0);
            // Both rims of a through-bore are 90-degree material wedges:
            // convex by the quadrant probe, while the wall itself is inward.
            for z in [0.0, 6.0 * scale] {
                let rim = circle_rim_at_z(&topo, bored, z, tol);
                let rel = edge_relation(&topo, bored, rim, None).unwrap();
                assert_eq!(rel.concavity, EdgeConcavity::Convex, "bore rim at z={z}");
                let angle = rel.dihedral_angle.unwrap();
                assert!((angle - std::f64::consts::FRAC_PI_2).abs() < 1e-6);
            }
            let walls = cylinder_wall_faces(&topo, bored);
            assert!(!walls.is_empty());
            for wall in walls {
                assert_eq!(
                    face_material_sense(&topo, bored, wall).unwrap(),
                    MaterialSense::Inward,
                    "bore wall must read inward"
                );
            }
        }
    }

    #[test]
    fn b16_blind_hole_floor_rim_concave_wall_inward() {
        let scale = 1.0;
        let mut topo = Topology::new();
        let plate = make_box(&mut topo, 20.0 * scale, 20.0 * scale, 6.0 * scale).unwrap();
        let drill = make_cylinder(&mut topo, 3.0 * scale, 4.0 * scale).unwrap();
        transform_solid(
            &mut topo,
            drill,
            &Mat4::translation(10.0 * scale, 10.0 * scale, 2.0 * scale),
        )
        .unwrap();
        let blind = boolean(&mut topo, BooleanOp::Cut, plate, drill).unwrap();
        let tol = 1e-9;
        // Opening rim convex, floor rim (wall meets hole floor) concave.
        let opening = circle_rim_at_z(&topo, blind, 6.0 * scale, tol);
        let floor = circle_rim_at_z(&topo, blind, 2.0 * scale, tol);
        assert_eq!(
            edge_relation(&topo, blind, opening, None)
                .unwrap()
                .concavity,
            EdgeConcavity::Convex
        );
        let floor_rel = edge_relation(&topo, blind, floor, None).unwrap();
        assert_eq!(floor_rel.concavity, EdgeConcavity::Concave);
        assert!(floor_rel.dihedral_angle.unwrap() < 0.0);
        for wall in cylinder_wall_faces(&topo, blind) {
            assert_eq!(
                face_material_sense(&topo, blind, wall).unwrap(),
                MaterialSense::Inward
            );
        }
    }

    #[test]
    fn b16_filleted_box_band_edges_tangent() {
        let mut topo = Topology::new();
        let cube = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edge = solid_edges(&topo, cube).unwrap()[0];
        let filleted = crate::fillet::fillet_rolling_ball(&mut topo, cube, &[edge], 1.0).unwrap();
        let relations = solid_edge_relations(&topo, filleted, None).unwrap();
        let tangent = relations
            .iter()
            .filter(|r| r.concavity == EdgeConcavity::Tangent)
            .count();
        assert_eq!(tangent, 2, "one spring contact on each side of the band");
        for rel in relations
            .iter()
            .filter(|r| r.concavity == EdgeConcavity::Tangent)
        {
            let angle = rel.dihedral_angle.expect("tangent carries ~0 angle");
            assert!(
                angle.abs() < 1e-6,
                "tangent dihedral must be ~0, got {angle}"
            );
        }
    }

    #[test]
    fn b16_cone_plane_rim_convex_and_wall_outward() {
        for scale in [1e-3, 1.0, 1e3] {
            let mut topo = Topology::new();
            let cone =
                crate::primitives::make_cone(&mut topo, 2.0 * scale, 1.0 * scale, 2.0 * scale)
                    .unwrap();
            let tol = 1e-9 * scale.max(1.0);
            let rim = circle_rim_at_z(&topo, cone, 0.0, tol);
            let rel = edge_relation(&topo, cone, rim, None).unwrap();
            assert_eq!(rel.concavity, EdgeConcavity::Convex);
            let angle = rel.dihedral_angle.expect("convex carries +angle");
            assert!(
                angle > 0.0 && angle < std::f64::consts::PI,
                "cone rim dihedral must be a positive angle below pi, got {angle}"
            );
            let wall = solid_faces(&topo, cone)
                .unwrap()
                .into_iter()
                .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cone(_)))
                .expect("cone must have a conical wall");
            assert_eq!(
                face_material_sense(&topo, cone, wall).unwrap(),
                MaterialSense::Outward
            );
        }
    }

    #[test]
    fn b16_seam_and_degenerate_edges_are_unknown() {
        let mut topo = Topology::new();
        let cylinder = make_cylinder(&mut topo, 2.0, 4.0).unwrap();
        let adjacency = topo.build_adjacency(cylinder).unwrap();
        let seam = solid_edges(&topo, cylinder)
            .unwrap()
            .into_iter()
            .find(|&edge| {
                let faces = adjacency.faces_for_edge(edge);
                faces.len() == 2 && faces[0] == faces[1]
            })
            .expect("periodic wall seam");
        let rel = edge_relation(&topo, cylinder, seam, None).unwrap();
        assert_eq!(rel.concavity, EdgeConcavity::Unknown);
        assert_eq!(rel.dihedral_angle, None, "unknown never carries an angle");
        let bulk = solid_edge_relations(&topo, cylinder, None).unwrap();
        let seam_bulk = bulk.iter().find(|r| r.edge == seam).expect("seam in bulk");
        assert_eq!(seam_bulk.concavity, EdgeConcavity::Unknown);
        assert_eq!(seam_bulk.dihedral_angle, None);
    }

    /// An existing edge owned by another solid is a typed refusal, not the
    /// conservative `Unknown` reserved for this solid's seams and
    /// non-manifold edges: the adjacency lookup alone would find no faces
    /// for it and report a successful `Unknown`.
    #[test]
    fn b16_edge_relation_refuses_an_edge_of_another_solid() {
        let mut topo = Topology::new();
        let boxed = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let cylinder = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
        let box_edge = solid_edges(&topo, boxed).unwrap()[0];
        let cylinder_edge = solid_edges(&topo, cylinder).unwrap()[0];
        for (solid, edge) in [(cylinder, box_edge), (boxed, cylinder_edge)] {
            for probe in [None, Some(0.01)] {
                match edge_relation(&topo, solid, edge, probe) {
                    Err(crate::OperationsError::InvalidInput { reason }) => assert!(
                        reason.contains("not part of the solid"),
                        "refusal must say why: {reason}"
                    ),
                    other => panic!("foreign edge {edge:?} probe {probe:?}: {other:?}"),
                }
            }
        }
        // The owners still classify their own edges.
        assert_eq!(
            edge_relation(&topo, boxed, box_edge, None)
                .unwrap()
                .concavity,
            EdgeConcavity::Convex
        );
        assert!(edge_relation(&topo, cylinder, cylinder_edge, None).is_ok());
    }

    #[test]
    fn b16_material_sense_refusals_and_primitives() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let plane = planar_faces(&topo, solid)[0];
        assert!(matches!(
            face_material_sense(&topo, solid, plane),
            Err(crate::OperationsError::Unsupported { .. })
        ));
        // NURBS faces refuse the same way.
        crate::heal::convert_to_bspline(&mut topo, solid).unwrap();
        let nurbs = solid_faces(&topo, solid).unwrap()[0];
        assert!(matches!(
            face_material_sense(&topo, solid, nurbs),
            Err(crate::OperationsError::Unsupported { .. })
        ));
        // A face from another solid is not part of this solid.
        let mut topo2 = Topology::new();
        let a = make_box(&mut topo2, 1.0, 1.0, 1.0).unwrap();
        let b = make_cylinder(&mut topo2, 1.0, 2.0).unwrap();
        let wall = cylinder_wall_faces(&topo2, b)[0];
        assert!(matches!(
            face_material_sense(&topo2, a, wall),
            Err(crate::OperationsError::InvalidInput { .. })
        ));
        // Sphere and torus primitives read outward.
        let sphere = crate::primitives::make_sphere(&mut topo2, 2.0, 16).unwrap();
        for face in solid_faces(&topo2, sphere).unwrap() {
            assert_eq!(
                face_material_sense(&topo2, sphere, face).unwrap(),
                MaterialSense::Outward
            );
        }
        let torus = crate::primitives::make_torus(&mut topo2, 4.0, 1.0, 16).unwrap();
        for face in solid_faces(&topo2, torus).unwrap() {
            assert_eq!(
                face_material_sense(&topo2, torus, face).unwrap(),
                MaterialSense::Outward
            );
        }
    }

    #[test]
    fn b16_default_probe_formula_is_five_percent_of_local_scale() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let adjacency = topo.build_adjacency(solid).unwrap();
        let edge = solid_edges(&topo, solid).unwrap()[0];
        let faces = adjacency.faces_for_edge(edge);
        let probe = default_concavity_probe(&topo, edge, faces[0], faces[1]).unwrap();
        // A 2-unit box face spans sqrt(8) ~= 2.828 (face diagonal); the edge
        // spans 2, so local scale is the face diagonal and 5 % is ~0.1414.
        let expected = 8.0_f64.sqrt() * 0.05;
        assert!(
            (probe - expected).abs() < 1e-9,
            "default probe must be 0.05*local_scale, got {probe} vs {expected}"
        );
        // And it classifies: the box edge is convex with the default.
        assert_eq!(
            edge_relation(&topo, solid, edge, None).unwrap().concavity,
            EdgeConcavity::Convex
        );
    }
}
