//! Exact complete chamfers of trivalent orthogonal planar shells.
//!
//! Rebuild every wire, including a boss footprint in a plate face. At mixed
//! convex/concave vertices the corner plane passes through the old vertex;
//! its intersection with a support can contribute two contacts. Tier: core.

use std::collections::{BTreeMap, BTreeSet};

use remus_blend::{BlendEngine, BlendFaceOrigins, BlendResult, query::edge_material_frames_at};
use remus_math::{
    compact_sort::CompactSort,
    tolerance::Tolerance,
    vec::{Point3, Vec3},
};
use remus_topology::{
    Topology,
    edge::{EdgeCurve, EdgeId},
    explorer::{solid_edges, solid_faces},
    face::FaceId,
    solid::SolidId,
    vertex::VertexId,
    wire::WireId,
};

use crate::{
    OperationsError,
    boolean::{FaceSpec, assemble_solid_mixed_with_history},
    dot_normal_point,
};

type Contacts = BTreeMap<(EdgeId, VertexId, FaceId), Point3>;

pub(super) fn has_complete_mixed_corner(
    topo: &Topology,
    solid: SolidId,
    seeds: &[EdgeId],
) -> Result<bool, OperationsError> {
    let adjacency = topo.build_adjacency(solid)?;
    let mut incident: BTreeMap<VertexId, BTreeSet<EdgeId>> = BTreeMap::new();
    for edge in solid_edges(topo, solid)? {
        let data = topo.edge(edge)?;
        incident.entry(data.start()).or_default().insert(edge);
        incident.entry(data.end()).or_default().insert(edge);
    }
    for at in incident.values() {
        if at.len() != 3 || at.iter().any(|edge| !seeds.contains(edge)) {
            continue;
        }
        let mut kinds = BTreeSet::new();
        for &edge in at {
            let pair = adjacency.faces_for_edge(edge);
            if pair.len() == 2
                && pair[0] != pair[1]
                && topo.face(pair[0])?.surface().is_planar()
                && topo.face(pair[1])?.surface().is_planar()
                && let Some(frame) = edge_material_frames_at(topo, edge, pair[0], pair[1])?
                && let Some(kind) = frame.concave()
            {
                kinds.insert(kind);
            }
        }
        if kinds.len() == 2 {
            return Ok(true);
        }
    }
    Ok(false)
}

pub(super) fn build(
    topo: &mut Topology,
    solid: SolidId,
    seeds: &[EdgeId],
    d1: f64,
    d2: f64,
) -> Result<Option<BlendResult>, OperationsError> {
    if d1.to_bits() != d2.to_bits() {
        return Ok(None);
    }
    let edges = solid_edges(topo, solid)?;
    if edges.iter().copied().collect::<BTreeSet<_>>() != seeds.iter().copied().collect() {
        return Ok(None);
    }
    let faces = solid_faces(topo, solid)?;
    let adjacency = topo.build_adjacency(solid)?;
    let mut offsets = BTreeMap::new();
    let mut concave = BTreeMap::new();
    let mut incident: BTreeMap<VertexId, BTreeSet<EdgeId>> = BTreeMap::new();
    for &edge in &edges {
        let data = topo.edge(edge)?;
        let pair = adjacency.faces_for_edge(edge);
        if !matches!(data.curve(), EdgeCurve::Line) || pair.len() != 2 || pair[0] == pair[1] {
            return Ok(None);
        }
        let Some(a) = topo.face(pair[0])?.effective_plane_normal() else {
            return Ok(None);
        };
        let Some(b) = topo.face(pair[1])?.effective_plane_normal() else {
            return Ok(None);
        };
        if a.dot(b).abs() > Tolerance::new().angular {
            return Ok(None);
        }
        let Some(frame) = edge_material_frames_at(topo, edge, pair[0], pair[1])? else {
            return Ok(None);
        };
        let Some(is_concave) = frame.concave() else {
            return Ok(None);
        };
        concave.insert(edge, is_concave);
        let middle = topo.vertex(data.start())?.point()
            + (topo.vertex(data.end())?.point() - topo.vertex(data.start())?.point()) * 0.5;
        for (face, into) in [(pair[0], frame.into1), (pair[1], frame.into2)] {
            let offset = into * d1;
            let Some((distance, _)) =
                remus_check::distance::point_to_face(topo, middle + offset, face)?
            else {
                return Ok(None);
            };
            if distance > Tolerance::new().linear {
                return Ok(None);
            }
            offsets.insert((edge, face), offset);
        }
        incident.entry(data.start()).or_default().insert(edge);
        incident.entry(data.end()).or_default().insert(edge);
    }
    let mut caps = BTreeMap::new();
    for (&vertex, at) in &incident {
        let support: BTreeSet<_> = at
            .iter()
            .flat_map(|e| adjacency.faces_for_edge(*e).iter().copied())
            .collect();
        if at.len() != 3 || support.len() != 3 {
            return Ok(None);
        }
        let normals: Vec<_> = support
            .iter()
            .map(|f| {
                topo.face(*f)
                    .map(remus_topology::face::Face::effective_plane_normal)
            })
            .collect::<Result<_, _>>()?;
        let Some(normals) = normals.into_iter().collect::<Option<Vec<_>>>() else {
            return Ok(None);
        };
        if (0..3).any(|i| {
            (i + 1..3).any(|j| normals[i].dot(normals[j]).abs() > Tolerance::new().angular)
        }) {
            return Ok(None);
        }
        let sum = normals
            .into_iter()
            .fold(Vec3::new(0.0, 0.0, 0.0), |a, b| a + b);
        let normal = sum.normalize()?;
        let count = at.iter().filter(|e| concave[e]).count();
        let shift = if count == 0 {
            -2.0 * d1 / sum.length()
        } else if count == 3 {
            2.0 * d1 / sum.length()
        } else {
            0.0
        };
        caps.insert(
            vertex,
            (
                normal,
                dot_normal_point(normal, topo.vertex(vertex)?.point()) + shift,
                support,
            ),
        );
    }
    let mut specs = Vec::new();
    let mut sources = Vec::new();
    let mut contacts = Contacts::new();
    for &face in &faces {
        let data = topo.face(face)?;
        let outer = trim_wire(
            topo,
            face,
            data.outer_wire(),
            &offsets,
            &caps,
            &mut contacts,
        )?;
        let mut inner = Vec::new();
        for &wire in data.inner_wires() {
            inner.push(trim_wire(topo, face, wire, &offsets, &caps, &mut contacts)?);
        }
        require_disjoint_wires(topo, face, &outer, &inner)?;
        specs.push(FaceSpec::ExistingTrimmedInners {
            face,
            outer: Some(outer),
            inner: inner.into_iter().map(Some).collect(),
        });
        sources.push(vec![face]);
    }
    for &edge in &edges {
        let data = topo.edge(edge)?;
        let pair = adjacency.faces_for_edge(edge);
        let a = contacts[&(edge, data.start(), pair[0])];
        let b = contacts[&(edge, data.end(), pair[0])];
        let c = contacts[&(edge, data.end(), pair[1])];
        let d = contacts[&(edge, data.start(), pair[1])];
        let direction =
            (topo.vertex(data.end())?.point() - topo.vertex(data.start())?.point()).normalize()?;
        if (b - a).dot(direction) <= Tolerance::new().linear
            || (c - d).dot(direction) <= Tolerance::new().linear
        {
            return Err(super::unsupported(
                "orthogonal chamfer collapses a contact edge",
            ));
        }
        let normal = (topo
            .face(pair[0])?
            .effective_plane_normal()
            .ok_or_else(|| super::unsupported("missing plane"))?
            + topo
                .face(pair[1])?
                .effective_plane_normal()
                .ok_or_else(|| super::unsupported("missing plane"))?)
        .normalize()?;
        let mut vertices = vec![a, b, c, d];
        if (b - a).cross(c - a).dot(normal) < 0.0 {
            vertices.reverse();
        }
        specs.push(FaceSpec::Planar {
            vertices,
            normal,
            d: dot_normal_point(normal, a),
            inner_wires: vec![],
        });
        sources.push(pair.to_vec());
    }
    for (vertex, (normal, plane, support)) in caps {
        let mut points = Vec::new();
        for (&(_, at, _), &point) in &contacts {
            if at == vertex
                && !points
                    .iter()
                    .any(|p: &Point3| (*p - point).length() <= Tolerance::new().linear)
            {
                points.push(point);
            }
        }
        if points.len() < 3
            || points
                .iter()
                .any(|p| (dot_normal_point(normal, *p) - plane).abs() > Tolerance::new().linear)
        {
            return Err(super::unsupported("orthogonal corner contacts disagree"));
        }
        let center = points.iter().fold(Vec3::new(0.0, 0.0, 0.0), |a, p| {
            a + (*p - Point3::new(0.0, 0.0, 0.0))
        }) * (1.0 / points.len() as f64);
        let center = Point3::new(center.x(), center.y(), center.z());
        let axis = (points[0] - center).normalize()?;
        let binormal = normal.cross(axis);
        points.compact_sort_by(|a, b| {
            let a = *a - center;
            let b = *b - center;
            a.dot(binormal)
                .atan2(a.dot(axis))
                .total_cmp(&b.dot(binormal).atan2(b.dot(axis)))
        });
        specs.push(FaceSpec::Planar {
            vertices: points,
            normal,
            d: plane,
            inner_wires: vec![],
        });
        sources.push(support.into_iter().collect());
    }
    let assembly = assemble_solid_mixed_with_history(topo, &specs, Tolerance::new())?;
    super::analytic_tools::normalize_new_planar_wires(topo, solid, assembly.solid)?;
    let report = crate::validate::validate_solid(topo, assembly.solid)?;
    if !report.is_valid() {
        return Err(super::unsupported(
            "orthogonal chamfer left an invalid shell",
        ));
    }
    let mut origins = BlendFaceOrigins::default();
    for (index, face) in assembly.faces_by_spec.iter().enumerate() {
        let Some(face) = *face else {
            return Err(super::unsupported("orthogonal chamfer lost a face"));
        };
        if index < faces.len() {
            origins.survived.push((faces[index], face));
        } else {
            origins.created.push((face, sources[index].clone()));
        }
    }
    Ok(Some(BlendResult {
        solid: assembly.solid,
        succeeded: seeds.to_vec(),
        failed: vec![],
        is_partial: false,
        face_origins: Some(origins),
        engine: BlendEngine::AnalyticCut,
    }))
}

type Caps = BTreeMap<VertexId, (Vec3, f64, BTreeSet<FaceId>)>;

/// Fixed trim loops cannot express a collision or a hole merge. Prove their
/// planar embedding before assembly; a closed manifold graph alone can still
/// enclose overlapping holes that do not describe a valid geometric face.
fn require_disjoint_wires(
    topo: &Topology,
    face: FaceId,
    outer: &[Point3],
    inner: &[Vec<Point3>],
) -> Result<(), OperationsError> {
    let normal = topo
        .face(face)?
        .effective_plane_normal()
        .ok_or_else(|| super::unsupported("orthogonal trim has no plane"))?
        .normalize()?;
    let (u, v) = super::plane_basis(normal)?;
    let Some(&origin) = outer.first() else {
        return Err(super::unsupported("orthogonal trim is empty"));
    };
    let loops: Vec<Vec<(f64, f64)>> = std::iter::once(outer)
        .chain(inner.iter().map(Vec::as_slice))
        .map(|ring| {
            ring.iter()
                .map(|p| {
                    let delta = *p - origin;
                    (delta.dot(u), delta.dot(v))
                })
                .collect()
        })
        .collect();
    let eps = Tolerance::new().linear;
    for ring in &loops {
        let n = ring.len();
        if n < 3 {
            return Err(super::unsupported("orthogonal trim collapses a wire"));
        }
        for i in 0..n {
            let a = ring[i];
            let b = ring[(i + 1) % n];
            let c = ring[(i + 2) % n];
            if (b.0 - a.0).hypot(b.1 - a.1) <= eps
                || super::point_segment_distance(a, b, c) <= eps
                || super::point_segment_distance(c, a, b) <= eps
            {
                return Err(super::unsupported(
                    "orthogonal trim collapses or retraces an edge",
                ));
            }
            for j in i + 1..n {
                if j == (i + 1) % n || i == (j + 1) % n {
                    continue;
                }
                if segments_meet(a, b, ring[j], ring[(j + 1) % n], eps) {
                    return Err(super::unsupported("orthogonal trim intersects itself"));
                }
            }
        }
    }
    for i in 1..loops.len() {
        if loops[i]
            .iter()
            .any(|p| !super::point_in_polygon(*p, &loops[0]))
        {
            return Err(super::unsupported(
                "orthogonal trim moves a hole outside its face",
            ));
        }
        for j in 0..i {
            let a = &loops[i];
            let b = &loops[j];
            let touching = (0..a.len()).any(|ai| {
                (0..b.len()).any(|bi| {
                    segments_meet(
                        a[ai],
                        a[(ai + 1) % a.len()],
                        b[bi],
                        b[(bi + 1) % b.len()],
                        eps,
                    )
                })
            });
            let nested =
                j > 0 && (super::point_in_polygon(a[0], b) || super::point_in_polygon(b[0], a));
            if touching || nested {
                return Err(super::unsupported(
                    "orthogonal chamfer trim boundaries collide",
                ));
            }
        }
    }
    Ok(())
}

fn segments_meet(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64), eps: f64) -> bool {
    super::polylines_cross(&[a, b], &[c, d])
        || super::point_segment_distance(a, c, d) <= eps
        || super::point_segment_distance(b, c, d) <= eps
        || super::point_segment_distance(c, a, b) <= eps
        || super::point_segment_distance(d, a, b) <= eps
}

fn trim_wire(
    topo: &Topology,
    face: FaceId,
    wire: WireId,
    offsets: &BTreeMap<(EdgeId, FaceId), Vec3>,
    caps: &Caps,
    contacts: &mut Contacts,
) -> Result<Vec<Point3>, OperationsError> {
    let wire = topo.wire(wire)?;
    let oriented = wire.edges();
    let mut result = Vec::new();
    for (index, current) in oriented.iter().enumerate() {
        let previous = &oriented[(index + oriented.len() - 1) % oriented.len()];
        let data = topo.edge(current.edge())?;
        let vertex = current.oriented_start(data);
        let point = topo.vertex(vertex)?.point();
        let next = topo.vertex(current.oriented_end(data))?.point();
        let prev_data = topo.edge(previous.edge())?;
        let prev = topo.vertex(previous.oriented_start(prev_data))?.point();
        let u = (next - point).normalize()?;
        let v = (prev - point).normalize()?;
        if u.dot(v).abs() > Tolerance::new().angular {
            return Err(super::unsupported("orthogonal wire has a non-right corner"));
        }
        let a = point + offsets[&(current.edge(), face)];
        let b = point + offsets[&(previous.edge(), face)];
        let intersection = a + u * (b - a).dot(u);
        let (normal, plane, _) = &caps[&vertex];
        let (before, after) =
            if (dot_normal_point(*normal, intersection) - plane).abs() <= Tolerance::new().linear {
                (intersection, intersection)
            } else {
                (
                    b + v * ((plane - dot_normal_point(*normal, b)) / normal.dot(v)),
                    a + u * ((plane - dot_normal_point(*normal, a)) / normal.dot(u)),
                )
            };
        result.push(before);
        if (after - before).length() > Tolerance::new().linear {
            result.push(after);
        }
        contacts.insert((previous.edge(), vertex, face), before);
        contacts.insert((current.edge(), vertex, face), after);
    }
    Ok(result)
}
