//! Feature recognition: detect geometric features from B-Rep topology.
//!
//! Analyzes face adjacency, surface types, and geometry to identify
//! manufacturing features like holes, pockets, fillets, and chamfers.
//! Useful for CAM path planning and simulation simplification.

#![allow(
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::suboptimal_flops,
    clippy::needless_range_loop,
    clippy::cast_precision_loss,
    clippy::doc_markdown,
    clippy::module_name_repetitions,
    clippy::manual_let_else,
    clippy::missing_const_for_fn,
    clippy::option_if_let_else,
    clippy::derivable_impls,
    clippy::bool_to_int_with_if,
    clippy::if_same_then_else,
    clippy::tuple_array_conversions,
    clippy::match_same_arms,
    clippy::derive_partial_eq_without_eq,
    clippy::suspicious_operation_groupings,
    clippy::too_many_lines,
    clippy::iter_over_hash_type,
    clippy::map_unwrap_or,
    clippy::unused_self,
    clippy::used_underscore_binding
)]

use std::collections::{HashMap, HashSet};

use brepkit_math::tolerance::Tolerance;
use brepkit_math::vec::{Point3, Vec3};
use brepkit_topology::Topology;
use brepkit_topology::edge::EdgeId;
use brepkit_topology::explorer::solid_faces;
use brepkit_topology::face::{FaceId, FaceSurface};
use brepkit_topology::solid::SolidId;

use crate::OperationsError;
pub use crate::query::EdgeConcavity;

/// Compatibility alias for the corrected edge-concavity classifier.
pub type ConcavityType = EdgeConcavity;

/// Surface classification for a face.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SurfaceClass {
    /// Planar surface.
    Planar,
    /// Cylindrical surface.
    Cylindrical,
    /// Conical surface.
    Conical,
    /// Spherical surface.
    Spherical,
    /// Toroidal surface.
    Toroidal,
    /// NURBS (free-form) surface.
    FreeForm,
}

/// A node in the face adjacency graph.
#[derive(Debug, Clone)]
pub struct FagNode {
    /// The face ID.
    pub face: FaceId,
    /// Surface classification.
    pub surface_class: SurfaceClass,
    /// Face area (approximate).
    pub area: f64,
}

/// An edge in the face adjacency graph.
#[derive(Debug, Clone)]
pub struct FagEdge {
    /// The shared topology edge ID.
    pub edge: EdgeId,
    /// Geometric convexity, G1 contact, or an explicit unknown.
    pub concavity: EdgeConcavity,
    /// Mean angle between effective outward normals in `[0, pi]`, when it can
    /// be evaluated at every interior sample.
    pub normal_angle: Option<f64>,
}

/// Face adjacency graph with typed nodes and edges.
pub struct FaceAdjacencyGraph {
    /// Nodes indexed by face index.
    pub nodes: HashMap<usize, FagNode>,
    /// Adjacency: `face_index -> [(neighbor_face_index, edge_info)]`.
    pub adjacency: HashMap<usize, Vec<(usize, FagEdge)>>,
}

/// Type of a detected pattern.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PatternType {
    /// Features arranged in a line.
    Linear,
    /// Features arranged in a circle.
    Circular,
}

/// A recognized geometric feature.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum Feature {
    /// A through-hole or blind hole.
    Hole {
        /// Faces forming the hole.
        faces: Vec<FaceId>,
        /// Estimated diameter (if detectable).
        diameter: Option<f64>,
        /// Whether the hole opens through two opposite sides of the solid.
        through: bool,
    },
    /// A chamfer (bevel) face between two adjacent faces.
    Chamfer {
        /// The chamfer face.
        face: FaceId,
        /// The two faces adjacent to the chamfer.
        adjacent: (FaceId, FaceId),
        /// Angle between the chamfer and each adjacent face.
        angle: f64,
    },
    /// A small face that may be a fillet approximation.
    FilletLike {
        /// The fillet face.
        face: FaceId,
        /// Area of the face.
        area: f64,
    },
    /// A pocket (depression bounded by walls and a floor).
    Pocket {
        /// The floor face.
        floor: FaceId,
        /// The wall faces.
        walls: Vec<FaceId>,
    },
    /// A detected pattern of repeated features.
    Pattern {
        /// Indices into the feature list of the pattern members.
        feature_indices: Vec<usize>,
        /// Pattern type (linear or circular).
        pattern_type: PatternType,
        /// Number of instances.
        count: usize,
        /// Center-to-center pitch (linear) or arc pitch (circular).
        spacing: Option<f64>,
    },
}

/// Recognize features in a solid.
///
/// Analyzes the solid's face adjacency and geometry to identify
/// common manufacturing features.
///
/// # Errors
///
/// Returns an error if topology lookups fail.
pub fn recognize_features(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
) -> Result<Vec<Feature>, OperationsError> {
    let face_ids = solid_faces(topo, solid)?;

    let mut features = Vec::new();

    let fag = build_face_adjacency_graph(topo, solid, &face_ids, deflection)?;

    detect_chamfers_fag(topo, &fag, &mut features)?;
    detect_fillet_like_fag(&fag, &mut features);
    detect_holes(topo, &fag, &mut features)?;
    detect_pockets_fag(&fag, &mut features);
    detect_patterns(topo, &mut features)?;

    Ok(features)
}

/// Build a typed face adjacency graph from a set of face IDs.
fn build_face_adjacency_graph(
    topo: &Topology,
    solid: SolidId,
    face_ids: &[FaceId],
    deflection: f64,
) -> Result<FaceAdjacencyGraph, OperationsError> {
    let mut nodes = HashMap::new();
    for &fid in face_ids {
        let face = topo.face(fid)?;
        let surface_class = classify_surface(face.surface());
        let area = crate::measure::face_area(topo, fid, deflection).unwrap_or(0.0);
        nodes.insert(
            fid.index(),
            FagNode {
                face: fid,
                surface_class,
                area,
            },
        );
    }

    let mut edge_to_faces: HashMap<usize, (EdgeId, Vec<FaceId>)> = HashMap::new();
    for &fid in face_ids {
        let face = topo.face(fid)?;
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            let wire = topo.wire(wire_id)?;
            for oe in wire.edges() {
                let entry = edge_to_faces
                    .entry(oe.edge().index())
                    .or_insert_with(|| (oe.edge(), Vec::new()));
                if !entry.1.contains(&fid) {
                    entry.1.push(fid);
                }
            }
        }
    }

    let mut adjacency: HashMap<usize, Vec<(usize, FagEdge)>> = HashMap::new();
    for (eid, faces) in edge_to_faces.values() {
        if faces.len() == 2 {
            let probe = local_concavity_probe(topo, *eid, faces[0], faces[1])?;
            let normal_angle = crate::query::edge_normal_angle(topo, *eid, faces[0], faces[1])?;
            // Reuse the adjacency assembled above and avoid the robust
            // classifier's full-solid tessellation for every edge probe.
            let concavity = crate::query::edge_concavity_from_faces(
                topo, solid, *eid, faces[0], faces[1], probe,
            )?;

            let edge_info = FagEdge {
                edge: *eid,
                concavity,
                normal_angle,
            };
            adjacency
                .entry(faces[0].index())
                .or_default()
                .push((faces[1].index(), edge_info.clone()));
            adjacency
                .entry(faces[1].index())
                .or_default()
                .push((faces[0].index(), edge_info));
        }
    }

    Ok(FaceAdjacencyGraph { nodes, adjacency })
}

/// Classify a `FaceSurface` into a `SurfaceClass`.
fn classify_surface(surface: &FaceSurface) -> SurfaceClass {
    match surface {
        FaceSurface::Plane { .. } => SurfaceClass::Planar,
        FaceSurface::Cylinder(_) => SurfaceClass::Cylindrical,
        FaceSurface::Cone(_) => SurfaceClass::Conical,
        FaceSurface::Sphere(_) => SurfaceClass::Spherical,
        FaceSurface::Torus(_) => SurfaceClass::Toroidal,
        FaceSurface::Nurbs(_) => SurfaceClass::FreeForm,
    }
}

/// Probe distance for edge convexity, chosen from the local edge and its two
/// incident faces rather than from the whole model's bounding box.
fn local_concavity_probe(
    topo: &Topology,
    edge_id: EdgeId,
    face_a: FaceId,
    face_b: FaceId,
) -> Result<f64, OperationsError> {
    let edge_scale = geometric_edge_length(topo, edge_id)?;
    let face_scale = face_vertex_span(topo, face_a)?.min(face_vertex_span(topo, face_b)?);
    let scale = edge_scale.min(face_scale);
    Ok((scale * 1.0e-3).max(Tolerance::new().linear * 10.0))
}

fn geometric_edge_length(topo: &Topology, edge_id: EdgeId) -> Result<f64, OperationsError> {
    let edge = topo.edge(edge_id)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    let (t0, t1) = edge.curve().domain_with_endpoints(start, end);
    let mut previous = edge.curve().evaluate_with_endpoints(t0, start, end);
    let mut length = 0.0;
    for i in 1..=16 {
        let t = t0 + (t1 - t0) * f64::from(i) / 16.0;
        let point = edge.curve().evaluate_with_endpoints(t, start, end);
        length += (point - previous).length();
        previous = point;
    }
    Ok(length)
}

fn face_vertex_span(topo: &Topology, face_id: FaceId) -> Result<f64, OperationsError> {
    let face = topo.face(face_id)?;
    let mut bounds: Option<(Point3, Point3)> = None;
    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oe in topo.wire(wire_id)?.edges() {
            let edge = topo.edge(oe.edge())?;
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

/// Detect chamfer faces using the face adjacency graph.
///
/// A chamfer is a small planar face whose normal is at an intermediate
/// angle (neither parallel nor perpendicular) to both neighboring faces.
fn detect_chamfers_fag(
    topo: &Topology,
    fag: &FaceAdjacencyGraph,
    features: &mut Vec<Feature>,
) -> Result<(), OperationsError> {
    let mut seen_chamfers: HashSet<usize> = HashSet::new();

    for (&idx, node) in &fag.nodes {
        if seen_chamfers.contains(&idx) {
            continue;
        }
        if node.surface_class != SurfaceClass::Planar {
            continue;
        }

        let face = topo.face(node.face)?;
        let Some(normal) = face.effective_plane_normal() else {
            continue;
        };

        let neighbors = fag
            .adjacency
            .get(&idx)
            .map_or(&[] as &[_], |v| v.as_slice());
        if neighbors.len() < 2 {
            continue;
        }

        for i in 0..neighbors.len() {
            for j in (i + 1)..neighbors.len() {
                let (ni, _) = &neighbors[i];
                let (nj, _) = &neighbors[j];

                let n1 = get_node_planar_normal(topo, fag, *ni);
                let n2 = get_node_planar_normal(topo, fag, *nj);

                if let (Some(n1), Some(n2)) = (n1, n2) {
                    let dot1 = normal.dot(n1).abs();
                    let dot2 = normal.dot(n2).abs();

                    // Chamfer face is at an angle (not parallel/perpendicular)
                    // to both adjacent faces.
                    if dot1 > 0.1 && dot1 < 0.95 && dot2 > 0.1 && dot2 < 0.95 {
                        let angle = normal.dot(n1).acos();
                        let f1 = fag.nodes.get(ni).map(|n| n.face);
                        let f2 = fag.nodes.get(nj).map(|n| n.face);
                        if let (Some(f1), Some(f2)) = (f1, f2) {
                            seen_chamfers.insert(idx);
                            features.push(Feature::Chamfer {
                                face: node.face,
                                adjacent: (f1, f2),
                                angle,
                            });
                        }
                    }
                }
            }
        }
    }

    Ok(())
}

/// Get the planar normal for a FAG node, or `None` if non-planar.
fn get_node_planar_normal(
    topo: &Topology,
    fag: &FaceAdjacencyGraph,
    node_idx: usize,
) -> Option<Vec3> {
    let node = fag.nodes.get(&node_idx)?;
    if node.surface_class != SurfaceClass::Planar {
        return None;
    }
    topo.face(node.face).ok()?.effective_plane_normal()
}

/// Detect fillet-like faces by small area relative to the average.
fn detect_fillet_like_fag(fag: &FaceAdjacencyGraph, features: &mut Vec<Feature>) {
    if fag.nodes.is_empty() {
        return;
    }

    let total_area: f64 = fag.nodes.values().map(|n| n.area).sum();
    #[allow(clippy::cast_precision_loss)]
    let avg_area = total_area / fag.nodes.len() as f64;
    let threshold = avg_area * 0.25;

    for node in fag.nodes.values() {
        if node.area < threshold && node.area > 0.0 {
            features.push(Feature::FilletLike {
                face: node.face,
                area: node.area,
            });
        }
    }
}

/// Detect holes by finding cylindrical faces in the FAG.
///
/// A through-hole connects to two or more distinct planar faces;
/// a blind hole connects to fewer.
fn detect_holes(
    topo: &Topology,
    fag: &FaceAdjacencyGraph,
    features: &mut Vec<Feature>,
) -> Result<(), OperationsError> {
    let tolerance = Tolerance::new();
    let mut cylinders = Vec::new();

    for (&idx, node) in &fag.nodes {
        if node.surface_class != SurfaceClass::Cylindrical {
            continue;
        }

        let face = topo.face(node.face)?;
        // A hole wall is oriented into the removed volume. An ordinary
        // cylinder or boss has the analytic cylinder's outward orientation
        // and must not be reported as a hole.
        if !face.is_reversed() {
            continue;
        }
        let cyl = match face.surface() {
            FaceSurface::Cylinder(c) => c,
            _ => continue,
        };
        let axis = canonical_axis(cyl.axis());
        let (axial_min, axial_max) = cylinder_axial_extent(topo, node.face, axis)?;
        cylinders.push(CylinderFaceInfo {
            node_index: idx,
            face: node.face,
            origin: cyl.origin(),
            axis,
            radius: cyl.radius(),
            axial_min,
            axial_max,
        });
    }
    cylinders.sort_unstable_by_key(|info| info.face.index());

    let mut groups: Vec<Vec<CylinderFaceInfo>> = Vec::new();
    for cylinder in cylinders {
        let mut merged = vec![cylinder];
        let mut group_index = 0;
        while group_index < groups.len() {
            let matches = groups[group_index].iter().any(|member| {
                merged.iter().any(|candidate| {
                    axes_are_collinear(member, candidate, tolerance)
                        && (cylinder_faces_are_connected(
                            fag,
                            member.node_index,
                            candidate.node_index,
                        ) || (tolerance.approx_eq(member.radius, candidate.radius)
                            && axial_extents_are_continuous(
                                (member.axial_min, member.axial_max),
                                (candidate.axial_min, candidate.axial_max),
                                tolerance,
                            )))
                })
            });
            if matches {
                merged.extend(groups.swap_remove(group_index));
            } else {
                group_index += 1;
            }
        }
        groups.push(merged);
    }

    for mut group in groups {
        group.sort_unstable_by_key(|info| info.face.index());
        let axis = group[0].axis;
        let mut positive_cap = false;
        let mut negative_cap = false;
        let mut cap_faces = HashSet::new();
        for cylinder in &group {
            for (neighbor_index, _) in fag
                .adjacency
                .get(&cylinder.node_index)
                .map_or(&[] as &[_], Vec::as_slice)
            {
                let Some(neighbor) = fag.nodes.get(neighbor_index) else {
                    continue;
                };
                if neighbor.surface_class != SurfaceClass::Planar
                    || !cap_faces.insert(neighbor.face)
                {
                    continue;
                }
                let Some(normal) = topo.face(neighbor.face)?.effective_plane_normal() else {
                    continue;
                };
                let alignment = normal.dot(axis);
                if alignment > 1.0 - 1e-8 {
                    positive_cap = true;
                } else if alignment < -1.0 + 1e-8 {
                    negative_cap = true;
                }
            }
        }

        let diameter = group.iter().map(|info| info.radius * 2.0).reduce(f64::min);
        features.push(Feature::Hole {
            faces: group.iter().map(|info| info.face).collect(),
            diameter,
            through: positive_cap && negative_cap,
        });
    }

    Ok(())
}

#[derive(Debug, Clone, Copy)]
struct CylinderFaceInfo {
    node_index: usize,
    face: FaceId,
    origin: Point3,
    axis: Vec3,
    radius: f64,
    axial_min: f64,
    axial_max: f64,
}

/// Give parallel cylinder axes one deterministic orientation.
fn canonical_axis(axis: Vec3) -> Vec3 {
    let components = [axis.x(), axis.y(), axis.z()];
    let dominant = components
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.abs().total_cmp(&b.abs()))
        .map_or_else(|| axis.z(), |(_, value)| *value);
    if dominant < 0.0 { -axis } else { axis }
}

fn axes_are_collinear(a: &CylinderFaceInfo, b: &CylinderFaceInfo, tolerance: Tolerance) -> bool {
    if a.axis.dot(b.axis).abs() < 1.0 - tolerance.angular {
        return false;
    }
    let offset = (b.origin - a.origin).cross(a.axis).length();
    tolerance.approx_eq(offset, 0.0)
}

/// Project a cylindrical face's boundary onto its canonical axis.
fn cylinder_axial_extent(
    topo: &Topology,
    face_id: FaceId,
    axis: Vec3,
) -> Result<(f64, f64), OperationsError> {
    let face = topo.face(face_id)?;
    let mut axial_min = f64::INFINITY;
    let mut axial_max = f64::NEG_INFINITY;

    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        let wire = topo.wire(wire_id)?;
        for oriented_edge in wire.edges() {
            let edge = topo.edge(oriented_edge.edge())?;
            for vertex_id in [edge.start(), edge.end()] {
                let point = topo.vertex(vertex_id)?.point();
                let axial = point.x() * axis.x() + point.y() * axis.y() + point.z() * axis.z();
                axial_min = axial_min.min(axial);
                axial_max = axial_max.max(axial);
            }
        }
    }

    if axial_min.is_finite() && axial_max.is_finite() {
        Ok((axial_min, axial_max))
    } else {
        Err(OperationsError::InvalidInput {
            reason: format!(
                "cylindrical face {} has no finite axial boundary extent",
                face_id.index()
            ),
        })
    }
}

/// Whether two projected axial intervals overlap or touch within tolerance.
fn axial_extents_are_continuous(a: (f64, f64), b: (f64, f64), tolerance: Tolerance) -> bool {
    (a.0 <= b.1 || tolerance.approx_eq(a.0, b.1)) && (b.0 <= a.1 || tolerance.approx_eq(b.0, a.1))
}

/// Two coaxial cylindrical bands belong to one stepped hole when they touch
/// directly or meet the same planar annulus/cap.
fn cylinder_faces_are_connected(fag: &FaceAdjacencyGraph, a_index: usize, b_index: usize) -> bool {
    let neighbors = |index| fag.adjacency.get(&index).map_or(&[] as &[_], Vec::as_slice);
    if neighbors(a_index)
        .iter()
        .any(|(neighbor, _)| *neighbor == b_index)
    {
        return true;
    }

    let planar_a: HashSet<usize> = neighbors(a_index)
        .iter()
        .filter_map(|(neighbor, _)| {
            fag.nodes
                .get(neighbor)
                .filter(|node| node.surface_class == SurfaceClass::Planar)
                .map(|_| *neighbor)
        })
        .collect();
    neighbors(b_index).iter().any(|(neighbor, _)| {
        planar_a.contains(neighbor)
            && fag
                .nodes
                .get(neighbor)
                .is_some_and(|node| node.surface_class == SurfaceClass::Planar)
    })
}

/// Detect pockets using concave-connected components in the FAG.
///
/// A pocket is a set of faces connected by concave edges, with at
/// least one planar floor face and two or more wall faces.
fn detect_pockets_fag(fag: &FaceAdjacencyGraph, features: &mut Vec<Feature>) {
    let mut visited: HashSet<usize> = HashSet::new();

    for &idx in fag.nodes.keys() {
        if visited.contains(&idx) {
            continue;
        }

        let node = match fag.nodes.get(&idx) {
            Some(n) => n,
            None => continue,
        };

        if node.surface_class != SurfaceClass::Planar {
            continue;
        }

        let mut component = HashSet::new();
        let mut stack = vec![idx];

        while let Some(current) = stack.pop() {
            if !component.insert(current) {
                continue;
            }

            if let Some(adj) = fag.adjacency.get(&current) {
                for (neighbor, edge) in adj {
                    if edge.concavity == EdgeConcavity::Concave && !component.contains(neighbor) {
                        stack.push(*neighbor);
                    }
                }
            }
        }

        // Classify component: floor = planar, walls = non-planar or
        // perpendicular planar faces.
        let mut floor = None;
        let mut walls = Vec::new();

        for &ci in &component {
            if let Some(n) = fag.nodes.get(&ci) {
                if n.surface_class == SurfaceClass::Planar {
                    if floor.is_none() {
                        floor = Some(n.face);
                    }
                } else {
                    walls.push(n.face);
                }
            }
        }

        if let Some(floor_face) = floor
            && walls.len() >= 2
        {
            features.push(Feature::Pocket {
                floor: floor_face,
                walls,
            });
            visited.extend(&component);
        }
    }
}

/// Detect patterns (linear or circular) among already-recognized features.
///
/// Groups holes by similar diameter, then tests whether their centroids
/// are collinear (linear pattern) or cocircular (circular pattern).
fn detect_patterns(topo: &Topology, features: &mut Vec<Feature>) -> Result<(), OperationsError> {
    let mut hole_info = Vec::new();
    for (feature_index, feature) in features.iter().enumerate() {
        let Feature::Hole {
            faces,
            diameter: Some(diameter),
            ..
        } = feature
        else {
            continue;
        };
        let Some(&face_id) = faces.first() else {
            continue;
        };
        let face = topo.face(face_id)?;
        let FaceSurface::Cylinder(cylinder) = face.surface() else {
            continue;
        };
        let axis = canonical_axis(cylinder.axis());
        let origin = cylinder.origin();
        let origin_vector = Vec3::new(origin.x(), origin.y(), origin.z());
        let position = origin + axis * -origin_vector.dot(axis);
        hole_info.push(HolePatternInfo {
            feature_index,
            diameter: *diameter,
            position,
            axis,
        });
    }

    if hole_info.len() < 3 {
        return Ok(());
    }

    // Group by diameter (within 1% tolerance) and parallel drilling axes.
    let groups = group_holes_by_diameter(&hole_info);

    let mut new_patterns = Vec::new();

    for group in &groups {
        if group.len() < 3 {
            continue;
        }

        if let Some((indices, spacing)) = fit_linear_pattern(group) {
            new_patterns.push(Feature::Pattern {
                count: indices.len(),
                feature_indices: indices,
                pattern_type: PatternType::Linear,
                spacing: Some(spacing),
            });
        } else if group.len() >= 4
            && let Some((indices, spacing)) = fit_circular_pattern(group)
        {
            new_patterns.push(Feature::Pattern {
                count: indices.len(),
                feature_indices: indices,
                pattern_type: PatternType::Circular,
                spacing: Some(spacing),
            });
        }
    }

    features.extend(new_patterns);
    Ok(())
}

#[derive(Debug, Clone, Copy)]
struct HolePatternInfo {
    feature_index: usize,
    diameter: f64,
    position: Point3,
    axis: Vec3,
}

/// Group holes by similar diameter (1% relative tolerance) and parallel axes.
fn group_holes_by_diameter(items: &[HolePatternInfo]) -> Vec<Vec<HolePatternInfo>> {
    let mut groups: Vec<Vec<HolePatternInfo>> = Vec::new();

    for &item in items {
        let mut found = false;
        for group in &mut groups {
            let representative = group[0];
            if (item.diameter - representative.diameter).abs()
                < representative.diameter * 0.01 + Tolerance::new().linear
                && item.axis.dot(representative.axis).abs() > 1.0 - 1e-8
            {
                group.push(item);
                found = true;
                break;
            }
        }
        if !found {
            groups.push(vec![item]);
        }
    }

    groups
}

/// Fit collinear, evenly-spaced hole centers and return their ordered feature
/// indices plus center-to-center pitch.
fn fit_linear_pattern(group: &[HolePatternInfo]) -> Option<(Vec<usize>, f64)> {
    if group.len() < 3 {
        return None;
    }
    let mut farthest = None;
    for i in 0..group.len() {
        for j in (i + 1)..group.len() {
            let distance_sq = (group[j].position - group[i].position).length_squared();
            if farthest.is_none_or(|(_, _, best)| distance_sq > best) {
                farthest = Some((i, j, distance_sq));
            }
        }
    }
    let (start, end, span_sq) = farthest?;
    let tolerance = Tolerance::new();
    if span_sq <= tolerance.linear_sq() {
        return None;
    }
    let direction = (group[end].position - group[start].position)
        .normalize()
        .ok()?;
    let origin = group[start].position;
    let span = span_sq.sqrt();
    let position_tolerance = tolerance.linear.max(span * 1e-6);
    let mut ordered = Vec::with_capacity(group.len());
    for hole in group {
        let offset = hole.position - origin;
        let parameter = offset.dot(direction);
        let nearest = origin + direction * parameter;
        if (hole.position - nearest).length() > position_tolerance {
            return None;
        }
        ordered.push((parameter, hole.feature_index));
    }
    ordered.sort_by(|a, b| a.0.total_cmp(&b.0));

    let spacing = (ordered.last()?.0 - ordered.first()?.0) / (ordered.len() - 1) as f64;
    if spacing <= tolerance.linear {
        return None;
    }
    let spacing_tolerance = tolerance.linear.max(spacing.abs() * 0.01);
    if ordered
        .windows(2)
        .any(|pair| ((pair[1].0 - pair[0].0) - spacing).abs() > spacing_tolerance)
    {
        return None;
    }

    Some((
        ordered.into_iter().map(|(_, index)| index).collect(),
        spacing,
    ))
}

/// Fit evenly-spaced centers on a common circle and return arc pitch.
fn fit_circular_pattern(group: &[HolePatternInfo]) -> Option<(Vec<usize>, f64)> {
    if group.len() < 4 {
        return None;
    }
    let tolerance = Tolerance::new();
    // Use a well-separated pair and the point farthest from their line. This
    // finds a non-collinear triple in linear time instead of exhaustively
    // examining every triple for collinear, unevenly-spaced inputs.
    let origin = group[0].position;
    let (_, a) = group[1..]
        .iter()
        .map(|hole| {
            let offset = hole.position - origin;
            (offset.length_squared(), offset)
        })
        .max_by(|left, right| left.0.total_cmp(&right.0))?;
    let (_, b, normal) = group[1..]
        .iter()
        .map(|hole| {
            let offset = hole.position - origin;
            let cross = a.cross(offset);
            (cross.length_squared(), offset, cross)
        })
        .max_by(|left, right| left.0.total_cmp(&right.0))?;
    let denominator = 2.0 * normal.length_squared();
    if denominator <= tolerance.linear_sq() {
        return None;
    }
    let offset = (a.length_squared() * b.cross(normal) + b.length_squared() * normal.cross(a))
        * (1.0 / denominator);
    let center = origin + offset;
    let radius = offset.length();
    let radial_tolerance = tolerance.linear.max(radius * 1e-6);
    if radius <= tolerance.linear
        || group
            .iter()
            .any(|hole| ((hole.position - center).length() - radius).abs() > radial_tolerance)
    {
        return None;
    }

    let radial_axis = (group[0].position - center).normalize().ok()?;
    let tangent_axis = group[0].axis.cross(radial_axis).normalize().ok()?;
    let mut ordered = group
        .iter()
        .map(|hole| {
            let radial = hole.position - center;
            let angle = radial
                .dot(tangent_axis)
                .atan2(radial.dot(radial_axis))
                .rem_euclid(std::f64::consts::TAU);
            (angle, hole.feature_index)
        })
        .collect::<Vec<_>>();
    ordered.sort_by(|a, b| a.0.total_cmp(&b.0));

    let expected_angle = std::f64::consts::TAU / ordered.len() as f64;
    let angle_tolerance = tolerance.angular.max(expected_angle * 0.01);
    for i in 0..ordered.len() {
        let next_angle = if i + 1 < ordered.len() {
            ordered[i + 1].0
        } else {
            ordered[0].0 + std::f64::consts::TAU
        };
        if ((next_angle - ordered[i].0) - expected_angle).abs() > angle_tolerance {
            return None;
        }
    }

    Some((
        ordered.into_iter().map(|(_, index)| index).collect(),
        radius * expected_angle,
    ))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic)]
mod tests {
    use super::*;
    use crate::boolean::{BooleanOp, boolean};
    use crate::primitives::{make_box, make_cylinder, make_sphere};
    use crate::transform::transform_solid;
    use brepkit_math::mat::Mat4;
    use brepkit_topology::validation::{validate_shell_closed, validate_shell_manifold};

    fn translated_cylinder(
        topo: &mut Topology,
        radius: f64,
        height: f64,
        x: f64,
        y: f64,
        z: f64,
    ) -> SolidId {
        let cylinder = make_cylinder(topo, radius, height).unwrap();
        transform_solid(topo, cylinder, &Mat4::translation(x, y, z)).unwrap();
        cylinder
    }

    fn assert_verified_solid(topo: &Topology, solid: SolidId, expected_volume: f64) {
        let solid_data = topo.solid(solid).unwrap();
        for shell_id in std::iter::once(solid_data.outer_shell())
            .chain(solid_data.inner_shells().iter().copied())
        {
            let shell = topo.shell(shell_id).unwrap();
            assert!(
                validate_shell_closed(shell, topo).is_ok(),
                "fixture shell must be closed"
            );
            assert!(
                validate_shell_manifold(shell, topo).is_ok(),
                "fixture shell must be manifold"
            );
        }

        let coarse = crate::measure::solid_volume(topo, solid, 0.2).unwrap();
        let medium = crate::measure::solid_volume(topo, solid, 0.05).unwrap();
        let fine = crate::measure::solid_volume(topo, solid, 0.01).unwrap();
        let allowed = expected_volume.abs().max(1.0) * 0.005;
        assert!(
            (fine - expected_volume).abs() <= allowed,
            "fixture volume {fine} should be within {allowed} of {expected_volume}"
        );
        assert!(
            (fine - medium).abs() <= (medium - coarse).abs() + allowed * 0.01,
            "fixture volume should converge: coarse={coarse}, medium={medium}, fine={fine}"
        );
    }

    fn cut_through_holes(
        topo: &mut Topology,
        mut solid: SolidId,
        centers: &[(f64, f64)],
        radius: f64,
        height: f64,
    ) -> SolidId {
        for &(x, y) in centers {
            let drill = translated_cylinder(topo, radius, height + 2.0, x, y, -1.0);
            solid = boolean(topo, BooleanOp::Cut, solid, drill).unwrap();
        }
        solid
    }

    #[test]
    fn box_has_no_chamfers() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let features = recognize_features(&topo, solid, 0.1).unwrap();

        let chamfer_count = features
            .iter()
            .filter(|f| matches!(f, Feature::Chamfer { .. }))
            .count();
        assert_eq!(chamfer_count, 0, "box should have no chamfers");
    }

    #[test]
    fn box_has_no_fillet_like() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let features = recognize_features(&topo, solid, 0.1).unwrap();

        let fillet_count = features
            .iter()
            .filter(|f| matches!(f, Feature::FilletLike { .. }))
            .count();
        assert_eq!(
            fillet_count, 0,
            "uniform box should have no fillet-like faces"
        );
    }

    #[test]
    fn chamfered_box_has_chamfer_features() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let solid_data = topo.solid(solid).unwrap();
        let shell = topo.shell(solid_data.outer_shell()).unwrap();
        let face_ids: Vec<FaceId> = shell.faces().to_vec();

        let mut edge_set = HashSet::new();
        for &fid in &face_ids {
            let face = topo.face(fid).unwrap();
            let wire = topo.wire(face.outer_wire()).unwrap();
            for oe in wire.edges() {
                edge_set.insert(oe.edge());
            }
        }
        let mut edges: Vec<_> = edge_set.into_iter().collect();
        edges.sort_unstable_by_key(|edge| edge.index());

        let chamfered = crate::chamfer::chamfer(&mut topo, solid, &[edges[0]], 0.2).unwrap();
        let features = recognize_features(&topo, chamfered, 0.1).unwrap();
        // The chamfered solid should have at least one chamfer feature.
        let chamfer_count = features
            .iter()
            .filter(|f| matches!(f, Feature::Chamfer { .. }))
            .count();
        assert!(
            chamfer_count > 0,
            "chamfered box should have chamfer features, got {chamfer_count}"
        );
    }

    #[test]
    fn feature_count_is_reasonable() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let features = recognize_features(&topo, solid, 0.1).unwrap();

        // A simple box might have pocket features (faces with 4 perpendicular neighbors)
        // but shouldn't have an excessive number
        assert!(
            features.len() <= 12,
            "box should have reasonable feature count, got {}",
            features.len()
        );
    }

    #[test]
    fn fag_nodes_match_face_count() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let solid_data = topo.solid(solid).unwrap();
        let shell = topo.shell(solid_data.outer_shell()).unwrap();
        let face_ids: Vec<FaceId> = shell.faces().to_vec();

        let fag = build_face_adjacency_graph(&topo, solid, &face_ids, 0.1).unwrap();
        assert_eq!(fag.nodes.len(), 6, "box has 6 faces");
    }

    #[test]
    fn fag_box_all_planar() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let solid_data = topo.solid(solid).unwrap();
        let shell = topo.shell(solid_data.outer_shell()).unwrap();
        let face_ids: Vec<FaceId> = shell.faces().to_vec();

        let fag = build_face_adjacency_graph(&topo, solid, &face_ids, 0.1).unwrap();
        for node in fag.nodes.values() {
            assert_eq!(node.surface_class, SurfaceClass::Planar);
        }
    }

    #[test]
    fn fag_box_adjacency_exists() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let solid_data = topo.solid(solid).unwrap();
        let shell = topo.shell(solid_data.outer_shell()).unwrap();
        let face_ids: Vec<FaceId> = shell.faces().to_vec();

        let fag = build_face_adjacency_graph(&topo, solid, &face_ids, 0.1).unwrap();
        // Each face of a box shares edges with 4 other faces.
        for node in fag.nodes.values() {
            let adj = fag.adjacency.get(&node.face.index());
            assert!(adj.is_some(), "face should have adjacency");
            let neighbors = adj.unwrap();
            assert!(
                neighbors.len() >= 2,
                "each box face should have at least 2 neighbors, got {}",
                neighbors.len()
            );
        }
    }

    #[test]
    fn box_has_no_holes() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let features = recognize_features(&topo, solid, 0.1).unwrap();
        let hole_count = features
            .iter()
            .filter(|f| matches!(f, Feature::Hole { .. }))
            .count();
        assert_eq!(hole_count, 0, "box should have no holes");
    }

    #[test]
    fn box_has_no_patterns() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let features = recognize_features(&topo, solid, 0.1).unwrap();
        let pattern_count = features
            .iter()
            .filter(|f| matches!(f, Feature::Pattern { .. }))
            .count();
        assert_eq!(pattern_count, 0, "box should have no patterns");
    }

    #[test]
    fn classify_surface_variants() {
        assert_eq!(
            classify_surface(&FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            }),
            SurfaceClass::Planar
        );
    }

    #[test]
    fn box_edges_are_convex_with_measured_normal_angles() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let faces = solid_faces(&topo, solid).unwrap();
        let fag = build_face_adjacency_graph(&topo, solid, &faces, 0.1).unwrap();

        assert_eq!(fag.adjacency.values().map(Vec::len).sum::<usize>(), 24);
        for neighbors in fag.adjacency.values() {
            for (_, edge) in neighbors {
                assert_eq!(edge.concavity, EdgeConcavity::Convex);
                assert!(
                    (edge.normal_angle.unwrap() - std::f64::consts::FRAC_PI_2).abs() < 1e-12,
                    "box outward normals meet at a right angle"
                );
            }
        }
    }

    #[test]
    fn group_by_diameter_groups_similar() {
        let make = |feature_index, diameter| HolePatternInfo {
            feature_index,
            diameter,
            position: Point3::new(feature_index as f64, 0.0, 0.0),
            axis: Vec3::new(0.0, 0.0, 1.0),
        };
        let items = vec![make(0, 10.0), make(1, 10.05), make(2, 20.0), make(3, 10.02)];
        let groups = group_holes_by_diameter(&items);
        assert_eq!(groups.len(), 2, "should form 2 groups");
    }

    #[test]
    fn pattern_detection_needs_three() {
        let group = [
            HolePatternInfo {
                feature_index: 0,
                diameter: 5.0,
                position: Point3::new(0.0, 0.0, 0.0),
                axis: Vec3::new(0.0, 0.0, 1.0),
            },
            HolePatternInfo {
                feature_index: 1,
                diameter: 5.0,
                position: Point3::new(1.0, 0.0, 0.0),
                axis: Vec3::new(0.0, 0.0, 1.0),
            },
        ];
        assert!(fit_linear_pattern(&group).is_none());
        assert!(fit_circular_pattern(&group).is_none());
    }

    #[test]
    fn arbitrary_same_diameter_triple_is_not_a_pattern() {
        let group = [
            HolePatternInfo {
                feature_index: 0,
                diameter: 5.0,
                position: Point3::new(0.0, 0.0, 0.0),
                axis: Vec3::new(0.0, 0.0, 1.0),
            },
            HolePatternInfo {
                feature_index: 1,
                diameter: 5.0,
                position: Point3::new(2.0, 1.0, 0.0),
                axis: Vec3::new(0.0, 0.0, 1.0),
            },
            HolePatternInfo {
                feature_index: 2,
                diameter: 5.0,
                position: Point3::new(5.0, 0.25, 0.0),
                axis: Vec3::new(0.0, 0.0, 1.0),
            },
        ];
        assert!(fit_linear_pattern(&group).is_none());
        // Three arbitrary non-collinear points always define a circle, so
        // circular recognition deliberately requires at least four holes.
        assert!(group.len() < 4);
    }

    #[test]
    fn circular_fitting_rejects_large_collinear_group() {
        let group = (0..1_000)
            .map(|feature_index| HolePatternInfo {
                feature_index,
                diameter: 5.0,
                position: Point3::new((feature_index * feature_index) as f64, 0.0, 0.0),
                axis: Vec3::new(0.0, 0.0, 1.0),
            })
            .collect::<Vec<_>>();

        assert!(fit_circular_pattern(&group).is_none());
    }

    #[test]
    fn circular_fitting_preserves_evenly_spaced_patterns() -> Result<(), &'static str> {
        let group = (0..8)
            .map(|feature_index| {
                let angle = std::f64::consts::TAU * feature_index as f64 / 8.0;
                HolePatternInfo {
                    feature_index,
                    diameter: 5.0,
                    position: Point3::new(10.0 * angle.cos(), 10.0 * angle.sin(), 0.0),
                    axis: Vec3::new(0.0, 0.0, 1.0),
                }
            })
            .collect::<Vec<_>>();

        let (indices, spacing) = fit_circular_pattern(&group).ok_or("pattern should fit")?;
        assert_eq!(indices.len(), group.len());
        assert!((spacing - 10.0 * std::f64::consts::TAU / 8.0).abs() < 1e-10);
        Ok(())
    }

    #[test]
    fn ordinary_cylinder_is_not_a_hole() {
        let mut topo = Topology::new();
        let solid = make_cylinder(&mut topo, 2.0, 5.0).unwrap();
        assert_verified_solid(&topo, solid, std::f64::consts::PI * 4.0 * 5.0);

        let features = recognize_features(&topo, solid, 0.05).unwrap();
        assert!(
            !features
                .iter()
                .any(|feature| matches!(feature, Feature::Hole { .. })),
            "an exterior cylindrical wall is not a hole"
        );
    }

    #[test]
    fn through_hole_uses_inner_wire_adjacency() {
        let mut topo = Topology::new();
        let body = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
        let drill = translated_cylinder(&mut topo, 2.0, 12.0, 10.0, 10.0, -1.0);
        let result = boolean(&mut topo, BooleanOp::Cut, body, drill).unwrap();
        let expected = 4000.0 - std::f64::consts::PI * 4.0 * 10.0;
        assert_verified_solid(&topo, result, expected);

        let features = recognize_features(&topo, result, 0.05).unwrap();
        let holes = features
            .iter()
            .filter_map(|feature| match feature {
                Feature::Hole {
                    diameter, through, ..
                } => Some((*diameter, *through)),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(holes.len(), 1);
        assert!(holes[0].1, "the bore opens through opposite planar faces");
        assert!((holes[0].0.unwrap() - 4.0).abs() < 1e-12);
    }

    #[test]
    fn blind_hole_has_same_side_planar_caps() {
        let mut topo = Topology::new();
        let body = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
        let drill = translated_cylinder(&mut topo, 2.0, 7.0, 10.0, 10.0, 4.0);
        let result = boolean(&mut topo, BooleanOp::Cut, body, drill).unwrap();
        let expected = 4000.0 - std::f64::consts::PI * 4.0 * 6.0;
        assert_verified_solid(&topo, result, expected);

        let features = recognize_features(&topo, result, 0.05).unwrap();
        let through = features.iter().find_map(|feature| match feature {
            Feature::Hole { through, .. } => Some(*through),
            _ => None,
        });
        assert_eq!(through, Some(false));
    }

    #[test]
    fn opposing_blind_holes_with_an_axial_gap_stay_distinct() {
        let mut topo = Topology::new();
        let body = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
        let lower_drill = translated_cylinder(&mut topo, 2.0, 4.0, 10.0, 10.0, -1.0);
        let lower_cut = boolean(&mut topo, BooleanOp::Cut, body, lower_drill).unwrap();
        let upper_drill = translated_cylinder(&mut topo, 2.0, 4.0, 10.0, 10.0, 7.0);
        let result = boolean(&mut topo, BooleanOp::Cut, lower_cut, upper_drill).unwrap();
        let expected = 4000.0 - 2.0 * std::f64::consts::PI * 4.0 * 3.0;
        assert_verified_solid(&topo, result, expected);

        let features = recognize_features(&topo, result, 0.05).unwrap();
        let hole_classifications = features
            .iter()
            .filter_map(|feature| match feature {
                Feature::Hole {
                    diameter, through, ..
                } => Some((*diameter, *through)),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(
            hole_classifications.len(),
            2,
            "the four-unit material web must separate the coaxial cuts"
        );
        assert!(hole_classifications.iter().all(|(diameter, through)| {
            !through && diameter.is_some_and(|value| (value - 4.0).abs() < 1e-12)
        }));
    }

    #[test]
    fn axial_extent_continuity_accepts_touching_bands_but_not_a_gap() {
        let tolerance = Tolerance::new();
        assert!(axial_extents_are_continuous(
            (0.0, 3.0),
            (3.0 + tolerance.linear * 0.5, 6.0),
            tolerance,
        ));
        assert!(!axial_extents_are_continuous(
            (0.0, 3.0),
            (7.0, 10.0),
            tolerance,
        ));
    }

    #[test]
    fn counterbore_is_one_grouped_through_hole() {
        let mut topo = Topology::new();
        let body = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
        let small = translated_cylinder(&mut topo, 1.5, 12.0, 10.0, 10.0, -1.0);
        let through_cut = boolean(&mut topo, BooleanOp::Cut, body, small).unwrap();
        let large = translated_cylinder(&mut topo, 3.0, 4.0, 10.0, 10.0, 7.0);
        let result = boolean(&mut topo, BooleanOp::Cut, through_cut, large).unwrap();
        let expected = 4000.0
            - std::f64::consts::PI * (1.5_f64.powi(2) * 10.0 + (9.0 - 1.5_f64.powi(2)) * 3.0);
        assert_verified_solid(&topo, result, expected);

        let features = recognize_features(&topo, result, 0.05).unwrap();
        let holes = features
            .iter()
            .filter_map(|feature| match feature {
                Feature::Hole {
                    faces,
                    diameter,
                    through,
                } => Some((faces, *diameter, *through)),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(holes.len(), 1, "coaxial counterbore bands are one hole");
        assert!(holes[0].0.len() >= 2, "both counterbore bands are retained");
        assert!((holes[0].1.unwrap() - 3.0).abs() < 1e-12);
        assert!(holes[0].2);
    }

    #[test]
    fn three_hole_linear_array_reports_pitch() {
        let mut topo = Topology::new();
        let body = make_box(&mut topo, 30.0, 12.0, 6.0).unwrap();
        let centers = [(6.0, 6.0), (15.0, 6.0), (24.0, 6.0)];
        let result = cut_through_holes(&mut topo, body, &centers, 1.0, 6.0);
        let expected = 30.0 * 12.0 * 6.0 - 3.0 * std::f64::consts::PI * 6.0;
        assert_verified_solid(&topo, result, expected);

        let features = recognize_features(&topo, result, 0.05).unwrap();
        assert_eq!(
            features
                .iter()
                .filter(|feature| matches!(feature, Feature::Hole { .. }))
                .count(),
            3
        );
        let linear = features.iter().find_map(|feature| match feature {
            Feature::Pattern {
                pattern_type: PatternType::Linear,
                count,
                spacing,
                ..
            } => Some((*count, *spacing)),
            _ => None,
        });
        let (count, spacing) = linear.unwrap();
        assert_eq!(count, 3);
        assert!((spacing.unwrap() - 9.0).abs() < 1e-9);
    }

    #[test]
    fn four_hole_bolt_circle_reports_arc_pitch() {
        let mut topo = Topology::new();
        let body = make_box(&mut topo, 30.0, 30.0, 6.0).unwrap();
        let centers = [(23.0, 15.0), (15.0, 23.0), (7.0, 15.0), (15.0, 7.0)];
        let result = cut_through_holes(&mut topo, body, &centers, 1.2, 6.0);
        let expected = 30.0 * 30.0 * 6.0 - 4.0 * std::f64::consts::PI * 1.2_f64.powi(2) * 6.0;
        assert_verified_solid(&topo, result, expected);

        let features = recognize_features(&topo, result, 0.05).unwrap();
        assert_eq!(
            features
                .iter()
                .filter(|feature| matches!(feature, Feature::Hole { .. }))
                .count(),
            4
        );
        let circular = features.iter().find_map(|feature| match feature {
            Feature::Pattern {
                pattern_type: PatternType::Circular,
                count,
                spacing,
                ..
            } => Some((*count, *spacing)),
            _ => None,
        });
        let (count, spacing) = circular.unwrap();
        assert_eq!(count, 4);
        assert!((spacing.unwrap() - 4.0 * std::f64::consts::PI).abs() < 1e-9);
    }

    #[test]
    fn sphere_tunnel_normals_are_local_surface_normals() {
        let mut topo = Topology::new();
        let sphere = make_sphere(&mut topo, 6.0, 24).unwrap();
        let drill = translated_cylinder(&mut topo, 3.0, 30.0, 0.0, 0.0, -15.0);
        let result = boolean(&mut topo, BooleanOp::Cut, sphere, drill).unwrap();
        let expected = 4.0 / 3.0 * std::f64::consts::PI * (36.0_f64 - 9.0).powf(1.5);
        assert_verified_solid(&topo, result, expected);

        let faces = solid_faces(&topo, result).unwrap();
        let fag = build_face_adjacency_graph(&topo, result, &faces, 0.05).unwrap();
        let (cylinder_face, sphere_face, edge_id) = fag
            .nodes
            .values()
            .filter(|node| node.surface_class == SurfaceClass::Cylindrical)
            .find_map(|cylinder| {
                fag.adjacency.get(&cylinder.face.index())?.iter().find_map(
                    |(neighbor_index, edge)| {
                        let neighbor = fag.nodes.get(neighbor_index)?;
                        (neighbor.surface_class == SurfaceClass::Spherical).then_some((
                            cylinder.face,
                            neighbor.face,
                            edge.edge,
                        ))
                    },
                )
            })
            .unwrap();
        let edge = topo.edge(edge_id).unwrap();
        let start = topo.vertex(edge.start()).unwrap().point();
        let end = topo.vertex(edge.end()).unwrap().point();
        let (t0, t1) = edge.curve().domain_with_endpoints(start, end);
        let midpoint = edge
            .curve()
            .evaluate_with_endpoints(f64::midpoint(t0, t1), start, end);
        let cylinder_normal =
            crate::query::effective_face_normal(&topo, cylinder_face, midpoint).unwrap();
        let sphere_normal =
            crate::query::effective_face_normal(&topo, sphere_face, midpoint).unwrap();
        let FaceSurface::Cylinder(cylinder) = topo.face(cylinder_face).unwrap().surface() else {
            unreachable!();
        };
        assert!(
            cylinder_normal.dot(cylinder.axis()).abs() < 1e-9,
            "a cylinder normal is radial, not its axis"
        );
        let radial = Vec3::new(midpoint.x(), midpoint.y(), midpoint.z())
            .normalize()
            .unwrap();
        assert!(
            sphere_normal.dot(radial).abs() > 1.0 - 1e-9,
            "a sphere normal follows the projected surface point"
        );
    }
}
