//! Exact analytic chamfer cutters for ridges the boundary rebuild refuses.
//!
//! Straight plane pairs use prismatic cutters; circular plane/cylinder and
//! plane/cone rims use revolved triangles. GFA retains the existing carriers
//! and exact trim curves. Tier: core (edits B-Rep and construction origins).

use crate::{
    OperationsError,
    boolean::{BooleanOp, require_exact_boolean_result},
};
use remus_blend::query::edge_material_frames_at;
use remus_blend::{BlendEngine, BlendFaceOrigins, BlendResult};
use remus_math::{
    compact_sort::CompactSort,
    mat::Mat4,
    tolerance::Tolerance,
    vec::{Point3, Vec3},
};
use remus_topology::{
    Topology,
    builder::make_polygon_wire,
    edge::{EdgeCurve, EdgeId},
    explorer::solid_faces,
    face::{Face, FaceId, FaceSurface},
    solid::SolidId,
};
use std::collections::{BTreeMap, BTreeSet};

mod joined_sweeps;

#[derive(Clone)]
enum Origin {
    Input(FaceId),
    Bevel(Vec<FaceId>),
    Unresolved,
}
#[derive(Clone)]
struct Tool {
    solid: SolidId,
    concave: bool,
    supports: Vec<FaceId>,
    radial: bool,
    plane: Option<(Point3, Vec3)>,
}

/// Try qualified analytic cutters after the primary chamfer engines refuse.
///
/// # Errors
/// Returns an error when construction, exact Boolean trimming or validation fails.
pub fn try_chamfer(
    topo: &mut Topology,
    solid: SolidId,
    seeds: &[EdgeId],
    d1: f64,
    d2: f64,
) -> Result<Option<BlendResult>, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |t| build(t, solid, seeds, d1, d2))
}

fn build(
    topo: &mut Topology,
    solid: SolidId,
    seeds: &[EdgeId],
    d1: f64,
    d2: f64,
) -> Result<Option<BlendResult>, OperationsError> {
    let adjacency = topo.build_adjacency(solid)?;
    let mut work = BTreeMap::new();
    for &seed in seeds {
        let pair = adjacency.faces_for_edge(seed);
        if pair.len() != 2 || pair[0] == pair[1] {
            return Ok(None);
        }
        work.insert(seed, [d1, d2]);
    }
    let mut tools = Vec::new();
    for (edge, distances) in work {
        let pair = adjacency.faces_for_edge(edge);
        let Some(tool) = make_tool(topo, solid, edge, [pair[0], pair[1]], distances, true)? else {
            return Ok(None);
        };
        tools.push(tool);
    }
    tools.extend(corner_tools(topo, solid, seeds, d1, d2)?);
    if seeds.len() > 1 && seeds.len() <= 32 {
        match remus_topology::transaction::run_transacted(topo, |t| {
            joined_sweeps::build(t, solid, seeds, d1, d2)
        }) {
            Ok(result) => return Ok(Some(result)),
            Err(error) => log::debug!("joined analytic sweeps refused: {error}"),
        }
    }
    let mut refusal = None;
    for order in 0..4 {
        let mut ordered = tools.clone();
        if order % 2 == 1 {
            ordered.reverse();
        }
        ordered.compact_sort_by_key(|tool| {
            (
                !tool.concave,
                if order < 2 {
                    tool.supports.len() == 3
                } else {
                    tool.supports.len() != 3
                },
                if order < 2 { tool.radial } else { !tool.radial },
            )
        });
        let attempt = remus_topology::transaction::run_transacted(topo, |t| {
            apply_tools(t, solid, seeds, &ordered)
        });
        match attempt {
            Ok(result) => return Ok(Some(result)),
            Err(error) => {
                log::debug!("analytic cutter order {order} refused: {error}");
                refusal = Some(error);
            }
        }
    }
    match refusal {
        Some(error) => Err(error),
        None => Ok(None),
    }
}

fn apply_tools(
    topo: &mut Topology,
    solid: SolidId,
    seeds: &[EdgeId],
    tools: &[Tool],
) -> Result<BlendResult, OperationsError> {
    let input = solid_faces(topo, solid)?;
    let mut origins: BTreeMap<usize, Origin> = input
        .iter()
        .map(|&f| (f.index(), Origin::Input(f)))
        .collect();
    let mut current = solid;
    for tool in tools {
        if let Some((point, normal)) = tool.plane {
            let bounds = crate::measure::conservative_solid_bounding_box(topo, current)?;
            let center = bounds.min + (bounds.max - bounds.min) * 0.5;
            let span = (bounds.max - bounds.min).length() * 2.0;
            let (result, sources) =
                super::curved_corner::clip(topo, current, point, normal, center, span)?;
            let mut next = BTreeMap::new();
            for (out, source) in sources {
                let origin = match source {
                    Some(source) => origins
                        .get(&source)
                        .cloned()
                        .ok_or_else(|| super::unsupported("corner cap has an unknown source"))?,
                    None => Origin::Bevel(tool.supports.clone()),
                };
                next.insert(out, origin);
            }
            origins = next;
            current = result;
            continue;
        }
        let op = if tool.concave {
            BooleanOp::Fuse
        } else {
            BooleanOp::Cut
        };
        let algo_op = if tool.concave {
            remus_algo::bop::BooleanOp::Fuse
        } else {
            remus_algo::bop::BooleanOp::Cut
        };
        let tool_origins: BTreeMap<usize, Origin> = solid_faces(topo, tool.solid)?
            .into_iter()
            .map(|face| {
                if tool.supports.is_empty() {
                    return (face.index(), Origin::Unresolved);
                }
                let source = tool
                    .supports
                    .iter()
                    .copied()
                    .find(|&s| same_carrier(topo, face, s));
                (
                    face.index(),
                    source.map_or_else(|| Origin::Bevel(tool.supports.clone()), Origin::Input),
                )
            })
            .collect();
        let faithful = remus_topology::transaction::run_transacted(topo, |t| {
            let (result, sources) =
                remus_algo::gfa::boolean_with_face_origins(t, algo_op, current, tool.solid)?;
            crate::heal::remove_degenerate_edges(t, result, Tolerance::new().linear)?;
            crate::heal::remove_wire_spurs(t, result)?;
            normalize_new_planar_wires(t, current, result)?;
            orient_new_faces(t, current, result, &sources)?;
            require_exact_boolean_result(t, op, current, tool.solid, result)?;
            Ok::<_, OperationsError>((result, sources))
        });
        let (result, sources) = match faithful {
            Ok(result) => result,
            Err(error) => {
                log::debug!("analytic cutter provenance route refused: {error}");
                // The regular exact boolean also handles coincident planar
                // caps. If that route rebuilds faces without origins, keep
                // the absence explicit rather than inventing ancestry.
                let result = crate::boolean::boolean(topo, op, current, tool.solid)?;
                let sources = solid_faces(topo, result)?
                    .into_iter()
                    .map(|face| {
                        let index = face.index();
                        let source = (origins.contains_key(&index)
                            || tool_origins.contains_key(&index))
                        .then_some(index);
                        (index, source)
                    })
                    .collect();
                (result, sources)
            }
        };
        let mut next = BTreeMap::new();
        for (out, source) in sources {
            let origin = match source {
                None => Origin::Unresolved,
                Some(source) => origins
                    .get(&source)
                    .or_else(|| tool_origins.get(&source))
                    .ok_or_else(|| {
                        super::unsupported("the analytic chamfer cut has an unknown face origin")
                    })?
                    .clone(),
            };
            next.insert(out, origin);
        }
        origins = next;
        current = result;
    }
    let mut history = BlendFaceOrigins::default();
    let mut survived = BTreeSet::new();
    for face in solid_faces(topo, current)? {
        match origins.get(&face.index()).ok_or_else(|| {
            super::unsupported("the analytic chamfer result has incomplete origins")
        })? {
            Origin::Input(source) => {
                history.survived.push((*source, face));
                survived.insert(source.index());
            }
            Origin::Bevel(sources) => history.created.push((face, sources.clone())),
            Origin::Unresolved => history.created_unattributed.push(face),
        }
    }
    history.deleted = input
        .into_iter()
        .filter(|f| !survived.contains(&f.index()))
        .collect();
    let report = crate::validate::validate_solid(topo, current)?;
    if !report.is_valid() {
        return Err(super::unsupported("analytic cutters left an invalid solid"));
    }
    Ok(BlendResult {
        solid: current,
        succeeded: seeds.to_vec(),
        failed: Vec::new(),
        is_partial: false,
        face_origins: Some(history),
        engine: BlendEngine::AnalyticCut,
    })
}

pub(super) fn normalize_new_planar_wires(
    topo: &mut Topology,
    input: SolidId,
    result: SolidId,
) -> Result<(), OperationsError> {
    let original: BTreeSet<_> = solid_faces(topo, input)?.into_iter().collect();
    for face in solid_faces(topo, result)? {
        if original.contains(&face) {
            continue;
        }
        let data = topo.face(face)?;
        let FaceSurface::Plane { normal, .. } = data.surface() else {
            continue;
        };
        let normal = *normal;
        let mut wires: Vec<_> = std::iter::once(data.outer_wire())
            .chain(data.inner_wires().iter().copied())
            .collect();
        let mut changed = false;
        for (index, wire) in wires.iter_mut().enumerate() {
            let edges = topo.wire(*wire)?.edges();
            if edges.iter().any(|edge| {
                topo.edge(edge.edge())
                    .is_ok_and(|e| !matches!(e.curve(), EdgeCurve::Line))
            }) {
                continue;
            }
            let mut area = Vec3::new(0.0, 0.0, 0.0);
            for edge in edges {
                let data = topo.edge(edge.edge())?;
                let a =
                    topo.vertex(edge.oriented_start(data))?.point() - Point3::new(0.0, 0.0, 0.0);
                let b = topo.vertex(edge.oriented_end(data))?.point() - Point3::new(0.0, 0.0, 0.0);
                area += a.cross(b);
            }
            let sign = area.dot(normal);
            if (index == 0 && sign < 0.0) || (index != 0 && sign > 0.0) {
                let reversed = edges
                    .iter()
                    .rev()
                    .map(|edge| {
                        remus_topology::wire::OrientedEdge::new(edge.edge(), !edge.is_forward())
                    })
                    .collect();
                *wire = topo.add_wire(remus_topology::wire::Wire::new(reversed, true)?);
                changed = true;
            }
        }
        if changed {
            topo.set_face_boundary_wires(face, wires[0], wires[1..].to_vec())?;
        }
    }
    Ok(())
}

/// Propagate boundary orientation from retained input faces onto new faces.
/// A retained face is an anchor, never a repair target. Contradictory edge
/// constraints refuse the construction; the ordinary solid/material gates
/// still validate the completed cut.
fn orient_new_faces(
    topo: &mut Topology,
    input: SolidId,
    result: SolidId,
    sources: &[(usize, Option<usize>)],
) -> Result<(), OperationsError> {
    let retained: BTreeSet<_> = solid_faces(topo, input)?.into_iter().collect();
    let faces = solid_faces(topo, result)?;
    let mut uses: BTreeMap<usize, Vec<(usize, bool)>> = BTreeMap::new();
    for (i, &id) in faces.iter().enumerate() {
        let face = topo.face(id)?;
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for edge in topo.wire(wire)?.edges() {
                uses.entry(edge.edge().index())
                    .or_default()
                    .push((i, edge.is_forward() != face.is_reversed()));
            }
        }
    }
    let mut neighbors = vec![Vec::new(); faces.len()];
    for incidences in uses.values() {
        let [(a, forward_a), (b, forward_b)] = incidences.as_slice() else {
            return Err(super::unsupported(
                "analytic cutter did not produce a closed manifold",
            ));
        };
        if a != b {
            let different = forward_a == forward_b;
            neighbors[*a].push((*b, different));
            neighbors[*b].push((*a, different));
        }
    }
    let mut flips = vec![None; faces.len()];
    let mut queue = std::collections::VecDeque::new();
    for (i, face) in faces.iter().enumerate() {
        if retained.contains(face) {
            flips[i] = Some(false);
            queue.push_back(i);
        } else if let Some(source) = sources
            .iter()
            .find_map(|&(out, source)| (out == face.index()).then_some(source).flatten())
            .and_then(|source| retained.iter().find(|face| face.index() == source))
        {
            // A split face keeps its certified analytic carrier. Its physical
            // outward side therefore follows that source, independently of
            // the wire direction chosen by the splitter.
            if same_carrier(topo, *face, *source) {
                let output = topo.face(*face)?;
                let original = topo.face(*source)?;
                let opposite = match output.surface() {
                    FaceSurface::Plane { normal: a, .. } => {
                        if let FaceSurface::Plane { normal: b, .. } = original.surface() {
                            a.dot(*b) < 0.0
                        } else {
                            false
                        }
                    }
                    FaceSurface::Nurbs(_)
                    | FaceSurface::Cylinder(_)
                    | FaceSurface::Cone(_)
                    | FaceSurface::Sphere(_)
                    | FaceSurface::Torus(_) => false,
                };
                flips[i] = Some(output.is_reversed() != (original.is_reversed() != opposite));
                queue.push_back(i);
            }
        }
    }
    if queue.is_empty() {
        return Err(super::unsupported(
            "analytic cutter has no retained orientation anchor",
        ));
    }
    while let Some(a) = queue.pop_front() {
        let Some(flip_a) = flips[a] else { continue };
        for &(b, different) in &neighbors[a] {
            let wanted = flip_a != different;
            match flips[b] {
                Some(actual) if actual != wanted => {
                    return Err(super::unsupported(
                        "analytic cutter has contradictory face orientations",
                    ));
                }
                Some(_) => {}
                None => {
                    flips[b] = Some(wanted);
                    queue.push_back(b);
                }
            }
        }
    }
    for (face, flip) in faces.into_iter().zip(flips) {
        match flip {
            Some(true) => {
                let data = topo.face_mut(face)?;
                data.set_reversed(!data.is_reversed());
            }
            Some(false) => {}
            None => {
                return Err(super::unsupported(
                    "analytic cutter left an unanchored component",
                ));
            }
        }
    }
    Ok(())
}

fn make_tool(
    topo: &mut Topology,
    solid: SolidId,
    edge: EdgeId,
    supports: [FaceId; 2],
    distances: [f64; 2],
    extend: bool,
) -> Result<Option<Tool>, OperationsError> {
    let Some(frames) = edge_material_frames_at(topo, edge, supports[0], supports[1])? else {
        return Ok(None);
    };
    let Some(concave) = frames.concave() else {
        return Ok(None);
    };
    let data = topo.edge(edge)?.clone();
    let a = topo.vertex(data.start())?.point();
    let b = topo.vertex(data.end())?.point();
    let surfaces = [
        topo.face(supports[0])?.surface().clone(),
        topo.face(supports[1])?.surface().clone(),
    ];
    let (lo, hi) = crate::authoritative_edge_domain(&data, "analytic chamfer cutter")?;
    let mid = f64::midpoint(lo, hi);
    let middle = data.curve().evaluate_with_endpoints(mid, a, b);
    let contact = [
        middle + frames.into1 * distances[0],
        middle + frames.into2 * distances[1],
    ];
    // An extended cutter must not turn an oversized setback into a valid
    // half-space cut through an unrelated part of the body. Both requested
    // contacts must still lie on their original trimmed support faces.
    for (support, point) in supports.into_iter().zip(contact) {
        let Some((distance, _)) = remus_check::distance::point_to_face(topo, point, support)?
        else {
            return Ok(None);
        };
        if distance > Tolerance::new().linear {
            return Ok(None);
        }
    }
    let solid = match data.curve() {
        EdgeCurve::Line if surfaces.iter().all(FaceSurface::is_planar) => {
            let direction = (b - a).normalize()?;
            let bounds = crate::measure::conservative_solid_bounding_box(topo, solid)?;
            let global = extend
                && supporting_pair(
                    topo,
                    solid,
                    middle,
                    frames.outward1,
                    frames.outward2,
                    direction,
                )?;
            let extension = if global {
                (bounds.max - bounds.min).length()
            } else {
                0.0
            };
            let shift = a - middle - direction * extension;
            let first = contact[0] + shift;
            let second = contact[1] + shift;
            let (apex, first, second) = if global {
                // The other two profile sides and both prism caps stay
                // strictly outside the complete body bounds. Only the bevel
                // plane can intersect it; its contacts are inside tool edges,
                // avoiding coincident tool vertices at earlier chamfers.
                let reach = 2.0 * extension;
                let factor = reach / distances[0].min(distances[1]);
                (
                    a - direction * extension - frames.into1 * reach - frames.into2 * reach,
                    first + (first - second) * factor,
                    second + (second - first) * factor,
                )
            } else {
                (
                    a - frames.into1 * distances[0] - frames.into2 * distances[1],
                    first,
                    second,
                )
            };
            let profile = triangle_along(topo, apex, first, second, direction)?;
            crate::extrude::extrude(topo, profile, direction, (b - a).length() + 2.0 * extension)?
        }
        EdgeCurve::Circle(circle) if analytic_rim(&surfaces, circle.normal()) => {
            let axis = circle.normal();
            let rotate = |p: Point3| {
                let v = p - circle.center();
                let angle = lo - mid;
                circle.center()
                    + v * angle.cos()
                    + axis.cross(v) * angle.sin()
                    + axis * axis.dot(v) * (1.0 - angle.cos())
            };
            let profile = triangle_along(
                topo,
                rotate(middle - frames.into1 * distances[0] - frames.into2 * distances[1]),
                rotate(contact[0]),
                rotate(contact[1]),
                axis.cross(circle.evaluate(lo) - circle.center()),
            )?;
            crate::revolve::revolve(topo, profile, circle.center(), axis, hi - lo)?
        }
        EdgeCurve::Line
        | EdgeCurve::Circle(_)
        | EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => return Ok(None),
    };
    Ok(Some(Tool {
        solid,
        concave,
        supports: supports.to_vec(),
        radial: matches!(data.curve(), EdgeCurve::Circle(_)),
        plane: None,
    }))
}

/// At a complete convex orthogonal corner the three bevels meet a triangular
/// cap. Its points are the intersections of the two setback lines on each
/// support face, just as in the planar boundary rebuild.
fn corner_tools(
    topo: &Topology,
    solid: SolidId,
    seeds: &[EdgeId],
    d1: f64,
    d2: f64,
) -> Result<Vec<Tool>, OperationsError> {
    let distances = seeds.iter().map(|&edge| (edge, [d1, d2])).collect();
    corner_tools_with_distances(topo, solid, seeds, &distances)
}

fn corner_tools_with_distances(
    topo: &Topology,
    solid: SolidId,
    seeds: &[EdgeId],
    distances: &BTreeMap<EdgeId, [f64; 2]>,
) -> Result<Vec<Tool>, OperationsError> {
    let adjacency = topo.build_adjacency(solid)?;
    let edges = remus_topology::explorer::solid_edges(topo, solid)?;
    let mut vertices = BTreeMap::new();
    for &edge in &edges {
        let data = topo.edge(edge)?;
        vertices
            .entry(data.start())
            .or_insert_with(Vec::new)
            .push(edge);
        if data.end() != data.start() {
            vertices
                .entry(data.end())
                .or_insert_with(Vec::new)
                .push(edge);
        }
    }
    let mut result = Vec::new();
    for (vertex, incident) in vertices {
        if incident.len() != 3 || incident.iter().any(|edge| !seeds.contains(edge)) {
            continue;
        }
        let mut supports = BTreeMap::new();
        let mut pairs = Vec::new();
        let mut qualified = true;
        for edge in incident {
            if !matches!(topo.edge(edge)?.curve(), EdgeCurve::Line) {
                qualified = false;
                break;
            }
            let pair = adjacency.faces_for_edge(edge);
            if pair.len() != 2 || pair[0] == pair[1] {
                qualified = false;
                break;
            }
            for &face in pair {
                let Some(normal) = topo.face(face)?.effective_plane_normal() else {
                    qualified = false;
                    break;
                };
                supports.insert(face, normal);
            }
            pairs.push(([pair[0], pair[1]], distances[&edge]));
        }
        if !qualified || supports.len() != 3 {
            continue;
        }
        let faces: Vec<_> = supports.keys().copied().collect();
        let normals: Vec<_> = supports.values().copied().collect();
        if (0..3).any(|i| {
            (i + 1..3).any(|j| normals[i].dot(normals[j]).abs() > Tolerance::new().angular)
        }) {
            continue;
        }
        let point = topo.vertex(vertex)?.point();
        if !supporting_pair(
            topo,
            solid,
            point,
            normals[0],
            normals[1],
            normals[0].cross(normals[1]),
        )? || !supporting_pair(
            topo,
            solid,
            point,
            normals[0],
            normals[2],
            normals[0].cross(normals[2]),
        )? {
            continue;
        }
        let mut points = vec![point];
        for &face in &faces {
            let mut offset = Vec3::new(0.0, 0.0, 0.0);
            for (pair, distance) in &pairs {
                if pair[0] == face {
                    offset -= supports[&pair[1]] * distance[0];
                } else if pair[1] == face {
                    offset -= supports[&pair[0]] * distance[1];
                }
            }
            points.push(point + offset);
        }
        let mut normal = (points[2] - points[1])
            .cross(points[3] - points[1])
            .normalize()?;
        let outward = normals
            .iter()
            .copied()
            .fold(Vec3::new(0.0, 0.0, 0.0), |a, b| a + b);
        if normal.dot(outward) < 0.0 {
            normal = -normal;
        }
        result.push(Tool {
            solid,
            concave: false,
            supports: faces,
            radial: false,
            plane: Some((points[1], normal)),
        });
    }
    Ok(result)
}

fn triangle(
    topo: &mut Topology,
    a: Point3,
    b: Point3,
    c: Point3,
) -> Result<FaceId, OperationsError> {
    let normal = (b - a).cross(c - a).normalize()?;
    let wire = make_polygon_wire(topo, &[a, b, c], Tolerance::new().linear)?;
    Ok(topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal,
            d: normal.dot(a - Point3::new(0.0, 0.0, 0.0)),
        },
    )))
}

fn triangle_along(
    topo: &mut Topology,
    a: Point3,
    b: Point3,
    c: Point3,
    direction: Vec3,
) -> Result<FaceId, OperationsError> {
    if (b - a).cross(c - a).dot(direction) < 0.0 {
        triangle(topo, a, c, b)
    } else {
        triangle(topo, a, b, c)
    }
}

fn analytic_rim(surfaces: &[FaceSurface; 2], axis: Vec3) -> bool {
    let (plane, wall) = match &surfaces[0] {
        FaceSurface::Plane { .. } => (&surfaces[0], &surfaces[1]),
        FaceSurface::Nurbs(_)
        | FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => (&surfaces[1], &surfaces[0]),
    };
    let FaceSurface::Plane { normal, .. } = plane else {
        return false;
    };
    let wall_axis = match wall {
        FaceSurface::Cylinder(c) => c.axis(),
        FaceSurface::Cone(c) => c.axis(),
        FaceSurface::Plane { .. }
        | FaceSurface::Nurbs(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => return false,
    };
    normal.dot(axis).abs() > 1.0 - Tolerance::new().angular
        && wall_axis.dot(axis).abs() > 1.0 - Tolerance::new().angular
}

/// Complete curved-boundary bounds certify that extending the cutter cannot
/// enter a remote concave lobe. The temporary frame is rigid and keeps curves.
fn supporting_pair(
    topo: &Topology,
    solid: SolidId,
    point: Point3,
    n1: Vec3,
    n2: Vec3,
    direction: Vec3,
) -> Result<bool, OperationsError> {
    let tol = Tolerance::new();
    if n1.dot(n2).abs() > tol.angular {
        return Ok(false);
    }
    let mut shadow = topo.clone();
    let row = |n: Vec3| {
        [
            n.x(),
            n.y(),
            n.z(),
            -n.dot(point - Point3::new(0.0, 0.0, 0.0)),
        ]
    };
    crate::transform::transform_solid(
        &mut shadow,
        solid,
        &Mat4([row(n1), row(n2), row(direction), [0.0, 0.0, 0.0, 1.0]]),
    )?;
    let bounds = crate::measure::conservative_solid_bounding_box(&shadow, solid)?;
    Ok(bounds.max.x() <= tol.linear && bounds.max.y() <= tol.linear)
}

fn same_carrier(topo: &Topology, first: FaceId, second: FaceId) -> bool {
    let (Ok(a), Ok(b)) = (topo.face(first), topo.face(second)) else {
        return false;
    };
    let tol = Tolerance::new();
    match a.surface() {
        FaceSurface::Plane { normal: a, d: da } => {
            if let FaceSurface::Plane { normal: b, d: db } = b.surface() {
                (a.dot(*b) > 1.0 - tol.angular && (da - db).abs() <= tol.linear)
                    || (a.dot(*b) < -1.0 + tol.angular && (da + db).abs() <= tol.linear)
            } else {
                false
            }
        }
        FaceSurface::Cylinder(a) => {
            if let FaceSurface::Cylinder(b) = b.surface() {
                a.axis().dot(b.axis()).abs() > 1.0 - tol.angular
                    && (a.radius() - b.radius()).abs() <= tol.linear
                    && (b.origin() - a.origin()).cross(a.axis()).length() <= tol.linear
            } else {
                false
            }
        }
        FaceSurface::Cone(a) => {
            if let FaceSurface::Cone(b) = b.surface() {
                a.axis().dot(b.axis()) > 1.0 - tol.angular
                    && (a.apex() - b.apex()).length() <= tol.linear
                    && (a.half_angle() - b.half_angle()).abs() <= tol.angular
            } else {
                false
            }
        }
        FaceSurface::Nurbs(_) | FaceSurface::Sphere(_) | FaceSurface::Torus(_) => false,
    }
}
