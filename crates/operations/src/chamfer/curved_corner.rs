//! Exact clipping of orthogonal planar ridges after earlier blends.
//!
//! The polygon chamfer cannot rebuild a curved boundary. For three straight
//! ridges sharing a trihedral corner, the same bevels and triangular corner
//! cap are four planes. Exact GFA cuts trim the existing carriers against
//! those planes, retaining circles, ellipses, cylinders and hole wires.
//! An isolated ridge uses one such plane, including its cylindrical end
//! rounds. This is a bounded addition: other selections keep their route.

use std::collections::{BTreeMap, BTreeSet};

use remus_blend::BlendFaceOrigins;
use remus_math::compact_sort::CompactSort;
use remus_math::mat::Mat4;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use crate::OperationsError;
use crate::boolean::{BooleanOp, require_exact_boolean_result};
use crate::measure::{conservative_solid_bounding_box, solid_volume};

#[derive(Clone)]
enum Origin {
    Input(FaceId),
    Bevel(Vec<FaceId>),
}

struct Corner {
    point: Point3,
    supports: [(FaceId, Vec3); 3],
    allowed: BTreeSet<usize>,
    lengths: [f64; 3],
}

pub(super) fn try_chamfer(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    distance: f64,
) -> Result<Option<(SolidId, BlendFaceOrigins)>, OperationsError> {
    if edges.len() == 1 {
        return try_single_edge(topo, solid, edges[0], distance);
    }
    let Some(corner) = qualify(topo, solid, edges, distance)? else {
        return Ok(None);
    };
    remus_topology::transaction::run_transacted(topo, |t| {
        build(t, solid, &corner, distance).map(Some)
    })
}

/// A whole-body support proof makes the plane a convex bevel rather than a
/// cut through a pocket. Only the two supports, endpoint faces and the planes
/// tangent to endpoint cylinders may change; the shared builder verifies all
/// other face boundaries and requires every original face to survive.
#[allow(clippy::too_many_lines)]
fn try_single_edge(
    topo: &mut Topology,
    solid: SolidId,
    edge_id: EdgeId,
    distance: f64,
) -> Result<Option<(SolidId, BlendFaceOrigins)>, OperationsError> {
    let edge = topo.edge(edge_id)?;
    if !matches!(edge.curve(), EdgeCurve::Line) {
        return Ok(None);
    }
    let endpoints = [edge.start(), edge.end()];
    let point = topo.vertex(endpoints[0])?.point();
    let direction = topo.vertex(endpoints[1])?.point() - point;
    if direction.length() <= 2.0 * distance {
        return Ok(None);
    }
    let adjacency = topo.build_adjacency(solid)?;
    let incident = adjacency.faces_for_edge(edge_id);
    if incident.len() != 2 {
        return Ok(None);
    }
    let mut supports = Vec::new();
    for &face_id in incident {
        let face = topo.face(face_id)?;
        let Some(normal) = face.effective_plane_normal() else {
            return Ok(None);
        };
        if !topo
            .wire(face.outer_wire())?
            .edges()
            .iter()
            .any(|e| e.edge() == edge_id)
        {
            return Ok(None);
        }
        supports.push((face_id, normal.normalize()?));
    }
    let eps = Tolerance::new().linear;
    let angular = Tolerance::new().angular;
    if supports[0].1.dot(supports[1].1).abs() > angular {
        return Ok(None);
    }
    let mut allowed: BTreeSet<_> = incident.iter().map(|f| f.index()).collect();
    let mut cylinders = BTreeSet::new();
    for id in solid_edges(topo, solid)? {
        let edge = topo.edge(id)?;
        if !endpoints.contains(&edge.start()) && !endpoints.contains(&edge.end()) {
            continue;
        }
        for &face_id in adjacency.faces_for_edge(id) {
            let face = topo.face(face_id)?;
            match face.surface() {
                FaceSurface::Plane { .. } => {}
                FaceSurface::Cylinder(_) if !face.is_reversed() => {
                    cylinders.insert(face_id);
                }
                FaceSurface::Cylinder(_)
                | FaceSurface::Cone(_)
                | FaceSurface::Sphere(_)
                | FaceSurface::Torus(_)
                | FaceSurface::Nurbs(_) => return Ok(None),
            }
            allowed.insert(face_id.index());
        }
    }
    if cylinders.is_empty() {
        return Ok(None);
    }
    // A bevel larger than an endpoint round may reach its other tangent
    // wall. Admit that wall by its exact cylinder/plane tangency, never by
    // proximity; unrelated bores and curved faces remain outside the set.
    for cylinder_id in cylinders {
        let face = topo.face(cylinder_id)?;
        let FaceSurface::Cylinder(cylinder) = face.surface() else {
            return Ok(None);
        };
        for oe in topo.wire(face.outer_wire())?.edges() {
            for &neighbour in adjacency.faces_for_edge(oe.edge()) {
                let FaceSurface::Plane { normal, d } = topo.face(neighbour)?.surface() else {
                    continue;
                };
                let length = normal.length();
                let unit = normal.normalize()?;
                let separation =
                    (unit.dot(cylinder.origin() - Point3::new(0.0, 0.0, 0.0)) - d / length).abs();
                if unit.dot(cylinder.axis()).abs() <= angular
                    && (separation - cylinder.radius()).abs() <= eps
                {
                    allowed.insert(neighbour.index());
                }
            }
        }
    }
    let axis = [-supports[0].1, -supports[1].1, direction.normalize()?];
    let row = |n: Vec3| {
        [
            n.x(),
            n.y(),
            n.z(),
            -n.dot(point - Point3::new(0.0, 0.0, 0.0)),
        ]
    };
    let mut framed = topo.clone();
    crate::transform::transform_solid(
        &mut framed,
        solid,
        &Mat4([
            row(axis[0]),
            row(axis[1]),
            row(axis[2]),
            [0.0, 0.0, 0.0, 1.0],
        ]),
    )?;
    let bounds = conservative_solid_bounding_box(&framed, solid)?;
    if [bounds.min.x(), bounds.min.y()]
        .iter()
        .any(|v| !v.is_finite() || *v < -eps)
        || [
            bounds.max.x(),
            bounds.max.y(),
            bounds.min.z(),
            bounds.max.z(),
        ]
        .iter()
        .any(|v| !v.is_finite())
    {
        return Ok(None);
    }
    let cut = (
        point - supports[0].1 * distance,
        (supports[0].1 + supports[1].1).normalize()?,
        supports.iter().map(|(f, _)| *f).collect(),
        0.0,
    );
    remus_topology::transaction::run_transacted(topo, |t| {
        build_cuts(t, solid, &allowed, vec![cut], distance).map(Some)
    })
}

/// The three planes must support the whole body on their inward side. This
/// excludes re-entrant corners and prevents interpreting a pocket's corner
/// as an outer one. The complete boundary bounds, not just vertex samples,
/// establish that condition in the corner's own orthogonal frame.
#[allow(clippy::too_many_lines)]
fn qualify(
    topo: &Topology,
    solid: SolidId,
    edges: &[EdgeId],
    distance: f64,
) -> Result<Option<Corner>, OperationsError> {
    if edges.len() != 3 || edges.iter().collect::<BTreeSet<_>>().len() != 3 {
        return Ok(None);
    }
    let first = topo.edge(edges[0])?;
    let common = [first.start(), first.end()].into_iter().find(|&v| {
        edges
            .iter()
            .all(|&id| topo.edge(id).is_ok_and(|e| e.start() == v || e.end() == v))
    });
    let Some(vertex) = common else {
        return Ok(None);
    };
    let point = topo.vertex(vertex)?.point();
    let adjacency = topo.build_adjacency(solid)?;
    let mut supports = BTreeMap::new();
    let mut endpoints = BTreeSet::new();
    for &id in edges {
        let edge = topo.edge(id)?;
        if !matches!(edge.curve(), EdgeCurve::Line) {
            return Ok(None);
        }
        let faces = adjacency.faces_for_edge(id);
        if faces.len() != 2 {
            return Ok(None);
        }
        for &fid in faces {
            let face = topo.face(fid)?;
            let Some(normal) = face.effective_plane_normal() else {
                return Ok(None);
            };
            if topo
                .wire(face.outer_wire())?
                .edges()
                .iter()
                .all(|oe| oe.edge() != id)
            {
                return Ok(None);
            }
            supports.insert(fid, normal.normalize()?);
        }
        let end = if edge.start() == vertex {
            edge.end()
        } else {
            edge.start()
        };
        endpoints.insert(end);
        if (topo.vertex(end)?.point() - point).length() <= 2.0 * distance {
            return Ok(None);
        }
    }
    if supports.len() != 3 {
        return Ok(None);
    }
    let supports: Vec<_> = supports.into_iter().collect();
    let supports = [supports[0], supports[1], supports[2]];
    let angular = Tolerance::new().angular;
    for i in 0..3 {
        for j in i + 1..3 {
            if supports[i].1.dot(supports[j].1).abs() > angular {
                return Ok(None);
            }
        }
    }
    // No unselected fourth ridge may share the corner. At the far ends,
    // only planar caps and convex cylindrical rounds are qualified here.
    let mut allowed: BTreeSet<_> = supports.iter().map(|(f, _)| f.index()).collect();
    for eid in solid_edges(topo, solid)? {
        let edge = topo.edge(eid)?;
        if (edge.start() == vertex || edge.end() == vertex) && !edges.contains(&eid) {
            return Ok(None);
        }
        if endpoints.contains(&edge.start()) || endpoints.contains(&edge.end()) {
            for &fid in adjacency.faces_for_edge(eid) {
                let face = topo.face(fid)?;
                match face.surface() {
                    FaceSurface::Plane { .. } => {}
                    FaceSurface::Cylinder(cylinder)
                        if !face.is_reversed() && distance <= cylinder.radius() => {}
                    FaceSurface::Cylinder(_)
                    | FaceSurface::Cone(_)
                    | FaceSurface::Sphere(_)
                    | FaceSurface::Torus(_)
                    | FaceSurface::Nurbs(_) => return Ok(None),
                }
                allowed.insert(fid.index());
            }
        }
    }
    let mut framed = topo.clone();
    let axis = supports.map(|(_, n)| -n);
    let row = |n: Vec3| {
        [
            n.x(),
            n.y(),
            n.z(),
            -n.dot(point - Point3::new(0.0, 0.0, 0.0)),
        ]
    };
    crate::transform::transform_solid(
        &mut framed,
        solid,
        &Mat4([
            row(axis[0]),
            row(axis[1]),
            row(axis[2]),
            [0.0, 0.0, 0.0, 1.0],
        ]),
    )?;
    let bounds = conservative_solid_bounding_box(&framed, solid)?;
    let eps = Tolerance::new().linear;
    if [bounds.min.x(), bounds.min.y(), bounds.min.z()]
        .iter()
        .any(|v| !v.is_finite() || *v < -eps)
        || [bounds.max.x(), bounds.max.y(), bounds.max.z()]
            .iter()
            .any(|v| !v.is_finite())
    {
        return Ok(None);
    }
    let mut lengths = [0.0f64; 3];
    for endpoint in endpoints {
        let direction = topo.vertex(endpoint)?.point() - point;
        for (i, (_, normal)) in supports.iter().enumerate() {
            lengths[i] = lengths[i].max(-direction.dot(*normal));
        }
    }
    Ok(Some(Corner {
        point,
        supports,
        allowed,
        lengths,
    }))
}

fn build(
    topo: &mut Topology,
    solid: SolidId,
    corner: &Corner,
    distance: f64,
) -> Result<(SolidId, BlendFaceOrigins), OperationsError> {
    let mut cuts = Vec::new();
    for i in 0..3 {
        for j in i + 1..3 {
            let (a, na) = corner.supports[i];
            let (b, nb) = corner.supports[j];
            cuts.push((
                corner.point - na * distance,
                (na + nb).normalize()?,
                vec![a, b],
                corner.lengths[3 - i - j],
            ));
        }
    }
    cuts.compact_sort_by(|a, b| a.3.total_cmp(&b.3));
    let normal = corner
        .supports
        .iter()
        .fold(Vec3::new(0.0, 0.0, 0.0), |sum, (_, n)| sum + *n)
        .normalize()?;
    cuts.push((
        corner.point - corner.supports[0].1 * (2.0 * distance),
        normal,
        corner.supports.iter().map(|(f, _)| *f).collect(),
        0.0,
    ));
    build_cuts(topo, solid, &corner.allowed, cuts, distance)
}

type Cut = (Point3, Vec3, Vec<FaceId>, f64);

fn build_cuts(
    topo: &mut Topology,
    solid: SolidId,
    allowed: &BTreeSet<usize>,
    cuts: Vec<Cut>,
    distance: f64,
) -> Result<(SolidId, BlendFaceOrigins), OperationsError> {
    let input_faces = solid_faces(topo, solid)?;
    let mut origins: BTreeMap<usize, Origin> = input_faces
        .iter()
        .map(|&f| (f.index(), Origin::Input(f)))
        .collect();
    let before = solid_volume(topo, solid, super::VOLUME_DEFLECTION)?;
    let bounds = conservative_solid_bounding_box(topo, solid)?;
    let center = bounds.min + (bounds.max - bounds.min) * 0.5;
    let span = (bounds.max - bounds.min).length() * 2.0 + distance;
    if !span.is_finite() || span <= 0.0 {
        return Err(super::unsupported(
            "the chamfer has no finite clipping extent",
        ));
    }
    let mut current = solid;
    for (point, normal, sources, _) in cuts {
        let (result, face_sources) = clip(topo, current, point, normal, center, span)?;
        let mut next = BTreeMap::new();
        for (out, source) in face_sources {
            let origin = match source {
                None => Origin::Bevel(sources.clone()),
                Some(source) => origins.get(&source).cloned().ok_or_else(|| {
                    super::unsupported("the corner cut has an unknown input face")
                })?,
            };
            next.insert(out, origin);
        }
        origins = next;
        current = result;
    }
    let mut history = BlendFaceOrigins::default();
    let mut survived = BTreeSet::new();
    for face in solid_faces(topo, current)? {
        match origins
            .get(&face.index())
            .ok_or_else(|| super::unsupported("the corner result has incomplete face origins"))?
        {
            Origin::Input(source) => {
                if !survived.insert(source.index()) {
                    return Err(super::unsupported(
                        "the corner cut split an existing face into multiple regions",
                    ));
                }
                if !allowed.contains(&source.index()) && !unchanged_boundary(topo, *source, face)? {
                    return Err(super::unsupported(
                        "the corner cut would alter a face outside its edge neighbourhood",
                    ));
                }
                history.survived.push((*source, face));
            }
            Origin::Bevel(sources) => history.created.push((face, sources.clone())),
        }
    }
    if input_faces.iter().any(|f| !survived.contains(&f.index())) {
        return Err(super::unsupported(
            "the corner cut would consume an existing face",
        ));
    }
    let after = solid_volume(topo, current, super::VOLUME_DEFLECTION)?;
    if !after.is_finite()
        || after <= 0.0
        || after >= before
        || !crate::validate::validate_solid(topo, current)?.is_valid()
    {
        return Err(super::unsupported(
            "the exact corner cut failed its solid postconditions",
        ));
    }
    Ok((current, history))
}

type FaceSources = Vec<(usize, Option<usize>)>;

/// Both branches are exact and transactional. The topology plane trimmer
/// retains untouched arcs without running intersections at all. Oblique cuts
/// through cylindrical rounds use GFA and the ordinary exact-result gate.
pub(super) fn clip(
    topo: &mut Topology,
    solid: SolidId,
    point: Point3,
    normal: Vec3,
    center: Point3,
    span: f64,
) -> Result<(SolidId, FaceSources), OperationsError> {
    if let Ok((halves, evolution)) = crate::split::split_with_evolution(topo, solid, point, normal)
    {
        let mut faces = Vec::new();
        for (source, outputs) in evolution.negative.modified {
            faces.extend(outputs.into_iter().map(|out| (out, Some(source))));
        }
        // A split creates exactly one cap; it is now a chamfer plane whose
        // two (or three) construction supports are known by the caller.
        if evolution.negative.unresolved.len() != 1 || !evolution.negative.generated.is_empty() {
            return Err(super::unsupported(
                "the corner plane split has unexpected cap origins",
            ));
        }
        faces.extend(
            evolution
                .negative
                .unresolved
                .into_keys()
                .map(|out| (out, None)),
        );
        return Ok((halves.negative, faces));
    }
    let plane_center = center + normal * normal.dot(point - center);
    let reference = if normal.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u = normal.cross(reference).normalize()?;
    let v = normal.cross(u);
    let p = plane_center - (u + v) * span;
    let tool = crate::primitives::make_box(topo, 2.0 * span, 2.0 * span, 2.0 * span)?;
    crate::transform::transform_solid(
        topo,
        tool,
        &Mat4([
            [u.x(), v.x(), normal.x(), p.x()],
            [u.y(), v.y(), normal.y(), p.y()],
            [u.z(), v.z(), normal.z(), p.z()],
            [0.0, 0.0, 0.0, 1.0],
        ]),
    )?;
    // Only this plane of the constructed tool can meet the body's complete
    // conservative box. Its other five faces are strictly outside that box.
    let cap = solid_faces(topo, tool)?
        .into_iter()
        .find(|&f| {
            topo.face(f).is_ok_and(|face| {
                face.effective_plane_normal()
                    .is_some_and(|n| n.dot(normal) < -0.999_999_999_999)
            })
        })
        .ok_or_else(|| super::unsupported("missing clipping-tool cap"))?;
    let (result, origins) = remus_algo::gfa::boolean_with_face_origins(
        topo,
        remus_algo::bop::BooleanOp::Cut,
        solid,
        tool,
    )?;
    crate::heal::remove_degenerate_edges(topo, result, Tolerance::new().linear)?;
    crate::heal::remove_wire_spurs(topo, result)?;
    require_exact_boolean_result(topo, BooleanOp::Cut, solid, tool, result)?;
    let input: BTreeSet<_> = solid_faces(topo, solid)?
        .into_iter()
        .map(remus_topology::arena::Id::index)
        .collect();
    let mut faces = Vec::new();
    for (out, source) in origins {
        let source = source.ok_or_else(|| {
            super::unsupported("the exact corner cut has an unresolved face origin")
        })?;
        if source == cap.index() {
            faces.push((out, None));
        } else if input.contains(&source) {
            faces.push((out, Some(source)));
        } else {
            return Err(super::unsupported(
                "a clipping-tool side entered the corner",
            ));
        }
    }
    Ok((result, faces))
}

/// This compares complete analytic geometry and authoritative trim domains,
/// not samples. The result's face identity already comes from construction;
/// this predicate only certifies that an unrelated patch stayed unchanged.
fn same_curve(a: &EdgeCurve, b: &EdgeCurve) -> bool {
    match a {
        EdgeCurve::Line => matches!(b, EdgeCurve::Line),
        EdgeCurve::Circle(a) => {
            let EdgeCurve::Circle(b) = b else {
                return false;
            };
            a.center() == b.center()
                && a.normal() == b.normal()
                && a.radius().to_bits() == b.radius().to_bits()
                && a.u_axis() == b.u_axis()
                && a.v_axis() == b.v_axis()
        }
        EdgeCurve::Ellipse(a) => {
            let EdgeCurve::Ellipse(b) = b else {
                return false;
            };
            a.center() == b.center()
                && a.normal() == b.normal()
                && a.semi_major().to_bits() == b.semi_major().to_bits()
                && a.semi_minor().to_bits() == b.semi_minor().to_bits()
                && a.u_axis() == b.u_axis()
                && a.v_axis() == b.v_axis()
        }
        EdgeCurve::NurbsCurve(a) => matches!(b, EdgeCurve::NurbsCurve(b) if a == b),
        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => false,
    }
}

fn same_edge_use(
    topo: &Topology,
    a: &remus_topology::wire::OrientedEdge,
    b: &remus_topology::wire::OrientedEdge,
) -> Result<bool, OperationsError> {
    let ea = topo.edge(a.edge())?;
    let eb = topo.edge(b.edge())?;
    if topo.vertex(a.oriented_start(ea))?.point() != topo.vertex(b.oriented_start(eb))?.point()
        || topo.vertex(a.oriented_end(ea))?.point() != topo.vertex(b.oriented_end(eb))?.point()
        || !same_curve(ea.curve(), eb.curve())
    {
        return Ok(false);
    }
    if !matches!(ea.curve(), EdgeCurve::Line) {
        let (Ok((a0, a1)), Ok((b0, b1))) = (ea.strict_domain(), eb.strict_domain()) else {
            return Ok(false);
        };
        if [a0.to_bits(), a1.to_bits()] != [b0.to_bits(), b1.to_bits()] {
            return Ok(false);
        }
    }
    Ok(true)
}

fn unchanged_boundary(
    topo: &Topology,
    input: FaceId,
    output: FaceId,
) -> Result<bool, OperationsError> {
    let a = topo.face(input)?;
    let b = topo.face(output)?;
    if a.is_reversed() != b.is_reversed() || a.inner_wires().len() != b.inner_wires().len() {
        return Ok(false);
    }
    for (old, new) in std::iter::once(a.outer_wire())
        .chain(a.inner_wires().iter().copied())
        .zip(std::iter::once(b.outer_wire()).chain(b.inner_wires().iter().copied()))
    {
        let old = topo.wire(old)?.edges();
        let new = topo.wire(new)?.edges();
        if old.len() != new.len() {
            return Ok(false);
        }
        let mut matched = false;
        for offset in 0..old.len() {
            let mut all = true;
            for (i, edge) in new.iter().enumerate() {
                if !same_edge_use(topo, &old[(i + offset) % old.len()], edge)? {
                    all = false;
                    break;
                }
            }
            if all {
                matched = true;
                break;
            }
        }
        if !matched {
            return Ok(false);
        }
    }
    Ok(true)
}
