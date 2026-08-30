//! Rolling-ball arc joint construction at convex edges.
//!
//! With [`JointType::Arc`](crate::JointType::Arc) an outward offset does not
//! extend adjacent faces until they meet. Each face translates along its own
//! outward normal and stops there; the gap left at every convex edge is filled
//! by the surface a ball of the offset radius sweeps as it rolls along that
//! edge, and the gap left at every convex vertex by the piece of that ball's
//! own surface it occupies while pivoting there.
//!
//! For a convex polyhedron this is exactly the Minkowski sum with a ball, so
//! the result's volume is the Steiner formula `V + A·d + M·d² + (4/3)π·d³`,
//! where `M` sums `½·len·(π − dihedral)` over the edges and the vertex term
//! collects into a whole ball because the vertex normal cones tile the sphere.
//!
//! # What is refused
//!
//! Only the case above is built. A non-planar source face, a concave or
//! tangent edge, a face with a hole, a cavity shell, an excluded face, or an
//! inward distance all return [`OffsetError::InvalidInput`] rather than fall
//! back to the mitred joint, which would silently return a noticeably larger
//! body than the rounded one that was asked for.

use std::collections::{BTreeMap, BTreeSet};

use remus_math::curves::Circle3D;
use remus_math::surfaces::{CylindricalSurface, SphericalSurface};
use remus_math::tolerance::Tolerance;
use remus_math::vec::Vec3;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use crate::data::{EdgeClass, OffsetData};
use crate::error::OffsetError;

/// Below this sine the two faces meeting at an edge have (anti-)parallel
/// normals, so the rolling ball sweeps either nothing or a half turn about a
/// knife edge. Dimensionless — both normals and the edge direction are unit
/// vectors — so it means the same angle at every model scale.
const MIN_JOINT_SINE: f64 = 1e-9;

/// Below this the edge direction is perpendicular to an adjacent face's
/// normal, i.e. the edge lies in the plane that face claims. Also
/// dimensionless, for the same reason.
const MAX_EDGE_PLANE_SINE: f64 = 1e-7;

/// One convex edge of the source solid, with its two faces ordered so that
/// rotating `face_a`'s outward normal about `axis` by the positive right-hand
/// angle reaches `face_b`'s. That ordering is what makes every joint face
/// below wind the same way, so it is established once here rather than
/// re-derived at each use.
struct JointEdge {
    edge: EdgeId,
    start: VertexId,
    end: VertexId,
    face_a: FaceId,
    face_b: FaceId,
    /// Unit direction from `start` to `end`.
    axis: Vec3,
}

/// Build the rounded (rolling-ball) offset of a convex polyhedron.
///
/// Returns the assembled solid: one translated copy of each source face, one
/// cylindrical patch per source edge, and one spherical patch per source
/// vertex.
///
/// # Errors
///
/// Returns [`OffsetError::InvalidInput`] when the source solid is outside the
/// supported class (see the module docs), and [`OffsetError::AssemblyFailed`]
/// when the joint faces do not close into an oriented 2-manifold.
pub fn build_arc_offset(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
    data: &OffsetData,
) -> Result<SolidId, OffsetError> {
    if distance <= 0.0 {
        return Err(OffsetError::InvalidInput {
            reason: "arc joints are built for an outward offset only: an inward offset rounds \
                     nothing, it mitres at the concave edges the source's convex ones become"
                .into(),
        });
    }
    if !data.excluded_faces.is_empty() {
        return Err(OffsetError::InvalidInput {
            reason: "arc joints cannot exclude faces: an open face has no joint to roll into"
                .into(),
        });
    }
    if !topo.solid(solid)?.inner_shells().is_empty() {
        return Err(OffsetError::InvalidInput {
            reason: "arc joints on a solid with cavity shells are not supported: a cavity's \
                     convex edges face into the void and would need the concave-side joint"
                .into(),
        });
    }

    let faces = topo
        .shell(topo.solid(solid)?.outer_shell())?
        .faces()
        .to_vec();
    let normals = collect_planar_normals(topo, &faces)?;
    let joint_edges = collect_joint_edges(topo, solid, data, &normals)?;

    let corners = build_corner_vertices(topo, &faces, &normals, distance)?;
    let face_lines = build_face_line_edges(topo, &joint_edges, &corners)?;
    let arcs = build_joint_arcs(topo, &joint_edges, &normals, &corners, distance)?;

    let mut result_faces = Vec::with_capacity(faces.len() + joint_edges.len() * 2);
    result_faces.extend(build_translated_faces(
        topo,
        &faces,
        &corners,
        distance,
        &face_lines,
    )?);
    result_faces.extend(build_edge_cylinders(
        topo,
        &joint_edges,
        &normals,
        &face_lines,
        &arcs,
        distance,
    )?);
    result_faces.extend(build_vertex_spheres(
        topo,
        &joint_edges,
        &normals,
        &arcs,
        distance,
    )?);

    check_oriented_manifold(topo, &result_faces)?;

    let shell_id = topo.add_shell(Shell::new(result_faces)?);
    Ok(topo.add_solid(Solid::new(shell_id, vec![])))
}

/// The outward normal of every face, refusing anything that is not a plane
/// bounded by a single wire.
fn collect_planar_normals(
    topo: &Topology,
    faces: &[FaceId],
) -> Result<BTreeMap<usize, Vec3>, OffsetError> {
    let mut normals = BTreeMap::new();
    for &face_id in faces {
        let face = topo.face(face_id)?;
        if !face.inner_wires().is_empty() {
            return Err(OffsetError::InvalidInput {
                reason: format!(
                    "arc joints require faces without holes; face {} has {} inner wires",
                    face_id.index(),
                    face.inner_wires().len()
                ),
            });
        }
        let FaceSurface::Plane { normal, .. } = face.surface() else {
            return Err(OffsetError::InvalidInput {
                reason: format!(
                    "arc joints are implemented for planar faces only; face {} is curved",
                    face_id.index()
                ),
            });
        };
        let outward = if face.is_reversed() {
            -*normal
        } else {
            *normal
        };
        normals.insert(face_id.index(), outward);
    }
    Ok(normals)
}

/// Every edge of the solid as a [`JointEdge`], refusing any edge that is not a
/// straight convex two-face edge.
fn collect_joint_edges(
    topo: &Topology,
    solid: SolidId,
    data: &OffsetData,
    normals: &BTreeMap<usize, Vec3>,
) -> Result<Vec<JointEdge>, OffsetError> {
    let edge_faces = remus_topology::explorer::edge_to_face_map(topo, solid)?;
    let mut edge_indices: Vec<usize> = edge_faces.keys().copied().collect();
    edge_indices.sort_unstable();

    let mut joint_edges = Vec::with_capacity(edge_indices.len());
    for edge_index in edge_indices {
        let face_ids = &edge_faces[&edge_index];
        if face_ids.len() != 2 {
            return Err(OffsetError::InvalidInput {
                reason: format!(
                    "arc joints require a manifold solid; edge {edge_index} is used by {} faces",
                    face_ids.len()
                ),
            });
        }
        match data.edge_class.get(&edge_index) {
            Some(EdgeClass::Convex { .. }) => {}
            other => {
                return Err(OffsetError::InvalidInput {
                    reason: format!(
                        "arc joints are implemented for convex edges only; edge {edge_index} is {}",
                        match other {
                            Some(EdgeClass::Concave { .. }) => "concave",
                            Some(EdgeClass::Tangent) => "tangent",
                            _ => "unclassified",
                        }
                    ),
                });
            }
        }

        let edge_id =
            topo.edge_id_from_index(edge_index)
                .ok_or_else(|| OffsetError::InvalidInput {
                    reason: format!("edge index {edge_index} not found in arena"),
                })?;
        let edge = topo.edge(edge_id)?;
        if !matches!(edge.curve(), EdgeCurve::Line) {
            return Err(OffsetError::InvalidInput {
                reason: format!(
                    "arc joints require straight edges; edge {edge_index} carries a curve"
                ),
            });
        }
        let start = edge.start();
        let end = edge.end();
        let axis = (topo.vertex(end)?.point() - topo.vertex(start)?.point()).normalize()?;

        let (mut face_a, mut face_b) = (face_ids[0], face_ids[1]);
        let n_a = normal_of(normals, face_a)?;
        let n_b = normal_of(normals, face_b)?;
        for (face_id, normal) in [(face_a, n_a), (face_b, n_b)] {
            if axis.dot(normal).abs() > MAX_EDGE_PLANE_SINE {
                return Err(OffsetError::InvalidInput {
                    reason: format!(
                        "edge {edge_index} is not perpendicular to the normal of its face {}; a \
                         straight edge of a planar face must lie in that face's plane",
                        face_id.index()
                    ),
                });
            }
        }
        // Sine of the turn from n_a to n_b about `axis`. Its sign says which of
        // the two faces the ball reaches first when it rolls the positive way.
        let sine = axis.cross(n_a).dot(n_b);
        if sine.abs() < MIN_JOINT_SINE {
            return Err(OffsetError::InvalidInput {
                reason: format!(
                    "edge {edge_index} joins faces whose normals are parallel; the rolling ball \
                     sweeps no well-defined joint there"
                ),
            });
        }
        if sine < 0.0 {
            std::mem::swap(&mut face_a, &mut face_b);
        }

        joint_edges.push(JointEdge {
            edge: edge_id,
            start,
            end,
            face_a,
            face_b,
            axis,
        });
    }
    Ok(joint_edges)
}

fn normal_of(normals: &BTreeMap<usize, Vec3>, face: FaceId) -> Result<Vec3, OffsetError> {
    normals
        .get(&face.index())
        .copied()
        .ok_or_else(|| OffsetError::AssemblyFailed {
            reason: format!("face {} has no recorded outward normal", face.index()),
        })
}

/// Key for the offset image of source vertex `v` on source face `f`.
type Corner = (usize, usize);

/// One offset vertex per (source vertex, incident source face) pair, at
/// `p + d·n_f`. It lies on the translated face, on the cylinder of every
/// incident edge, and on the sphere at that vertex, all exactly — which is
/// what lets the three patch families share vertices instead of merely
/// touching.
fn build_corner_vertices(
    topo: &mut Topology,
    faces: &[FaceId],
    normals: &BTreeMap<usize, Vec3>,
    distance: f64,
) -> Result<BTreeMap<Corner, VertexId>, OffsetError> {
    let mut corners = BTreeMap::new();
    for &face_id in faces {
        let normal = normal_of(normals, face_id)?;
        for vertex_id in face_wire_vertices(topo, face_id)? {
            let key = (vertex_id.index(), face_id.index());
            if corners.contains_key(&key) {
                continue;
            }
            let source = topo.vertex(vertex_id)?;
            let point = source.point() + normal * distance;
            let tolerance = source.tolerance();
            let new_id = topo.add_vertex(Vertex::new(point, tolerance));
            corners.insert(key, new_id);
        }
    }
    Ok(corners)
}

/// The vertices of a face's outer wire, in traversal order.
fn face_wire_vertices(topo: &Topology, face_id: FaceId) -> Result<Vec<VertexId>, OffsetError> {
    let wire_id = topo.face(face_id)?.outer_wire();
    let mut vertices = Vec::new();
    for oriented in topo.wire(wire_id)?.edges() {
        let edge = topo.edge(oriented.edge())?;
        vertices.push(if oriented.is_forward() {
            edge.start()
        } else {
            edge.end()
        });
    }
    Ok(vertices)
}

/// Key for the translated image of source edge `e` on source face `f`.
type FaceLine = (usize, usize);

/// The translated image of each source edge on each of its two faces. The one
/// on `face_a` is shared by that face and the edge's cylinder, and likewise on
/// `face_b`: two edges, two uses each, which is the manifold count.
fn build_face_line_edges(
    topo: &mut Topology,
    joint_edges: &[JointEdge],
    corners: &BTreeMap<Corner, VertexId>,
) -> Result<BTreeMap<FaceLine, EdgeId>, OffsetError> {
    let mut lines = BTreeMap::new();
    for joint in joint_edges {
        for face_id in [joint.face_a, joint.face_b] {
            let from = corner_of(corners, joint.start, face_id)?;
            let to = corner_of(corners, joint.end, face_id)?;
            let edge_id = topo.add_edge(Edge::new(from, to, EdgeCurve::Line));
            lines.insert((joint.edge.index(), face_id.index()), edge_id);
        }
    }
    Ok(lines)
}

fn corner_of(
    corners: &BTreeMap<Corner, VertexId>,
    vertex: VertexId,
    face: FaceId,
) -> Result<VertexId, OffsetError> {
    corners
        .get(&(vertex.index(), face.index()))
        .copied()
        .ok_or_else(|| OffsetError::AssemblyFailed {
            reason: format!(
                "vertex {} has no offset image on face {}; the face's wire does not visit it",
                vertex.index(),
                face.index()
            ),
        })
}

/// Key for the joint arc of source edge `e` at source vertex `v`.
type ArcKey = (usize, usize);

/// Build one positively oriented minor arc of a source edge's normal cone.
///
/// The circle's explicit reference direction puts `from` at parameter zero;
/// the second face normal therefore determines the positive cone angle.
/// Certify both corners and the radial-bisector midpoint before allocating the
/// shared edge.
fn add_certified_joint_arc(
    topo: &mut Topology,
    center: remus_math::vec::Point3,
    circle: Circle3D,
    from: VertexId,
    to: VertexId,
    to_direction: Vec3,
) -> Result<EdgeId, OffsetError> {
    let from_vertex = topo.vertex(from)?;
    let to_vertex = topo.vertex(to)?;
    let from_point = from_vertex.point();
    let to_point = to_vertex.point();
    let from_tolerance = from_vertex.tolerance();
    let to_tolerance = to_vertex.tolerance();
    if !from_tolerance.is_finite()
        || from_tolerance < 0.0
        || !to_tolerance.is_finite()
        || to_tolerance < 0.0
    {
        return Err(OffsetError::AssemblyFailed {
            reason: format!(
                "joint arc corners have invalid tolerances {from_tolerance} and {to_tolerance}"
            ),
        });
    }
    let tolerance = from_tolerance
        .max(to_tolerance)
        .max(Tolerance::new().linear);

    let end_parameter = to_direction
        .dot(circle.v_axis())
        .atan2(to_direction.dot(circle.u_axis()))
        .rem_euclid(std::f64::consts::TAU);
    let angular_roundoff = 32.0 * f64::EPSILON * std::f64::consts::PI;
    if !end_parameter.is_finite()
        || end_parameter <= angular_roundoff
        || end_parameter > std::f64::consts::PI + angular_roundoff
    {
        return Err(OffsetError::AssemblyFailed {
            reason: format!(
                "joint arc does not define a positive minor normal-cone angle: {end_parameter}"
            ),
        });
    }

    let from_radial = (from_point - center).normalize()?;
    let to_radial = (to_point - center).normalize()?;
    let midpoint_radial = (from_radial + to_radial).normalize()?;
    let midpoint = center + midpoint_radial * circle.radius();
    let range = (0.0, end_parameter);
    for (label, parameter, expected) in [
        ("start", range.0, from_point),
        ("midpoint", end_parameter * 0.5, midpoint),
        ("end", range.1, to_point),
    ] {
        let residual = (circle.evaluate(parameter) - expected).length();
        if !residual.is_finite() || residual > tolerance {
            return Err(OffsetError::AssemblyFailed {
                reason: format!(
                    "joint arc {label} misses its exact normal-cone oracle by {residual} \
                     (tolerance {tolerance})"
                ),
            });
        }
    }

    let mut edge = Edge::with_tolerance(from, to, EdgeCurve::Circle(circle), Some(tolerance));
    edge.set_trim(Some(range));
    edge.strict_domain()
        .map_err(|error| OffsetError::AssemblyFailed {
            reason: format!("joint arc has invalid parameter authority: {error}"),
        })?;
    Ok(topo.add_edge(edge))
}

/// The end arcs of each edge's cylinder: at each of the edge's two vertices, a
/// circular arc of radius `d` about the edge direction, running from the
/// `face_a` corner to the `face_b` corner the short way.
///
/// Each arc is shared by exactly one cylinder and one sphere, which is what
/// stitches the two joint families into a closed skin.
fn build_joint_arcs(
    topo: &mut Topology,
    joint_edges: &[JointEdge],
    normals: &BTreeMap<usize, Vec3>,
    corners: &BTreeMap<Corner, VertexId>,
    distance: f64,
) -> Result<BTreeMap<ArcKey, EdgeId>, OffsetError> {
    let mut arcs = BTreeMap::new();
    for joint in joint_edges {
        let n_a = normal_of(normals, joint.face_a)?;
        let n_b = normal_of(normals, joint.face_b)?;
        for vertex_id in [joint.start, joint.end] {
            let center = topo.vertex(vertex_id)?.point();
            // `face_a` was ordered so the turn about `axis` from its normal to
            // `face_b`'s is positive, and the circle's own u-axis is `n_a`, so
            // the edge spans the minor arc from start to end — the angle of
            // the source's normal cone at this edge.
            let circle = Circle3D::new_with_ref(center, joint.axis, distance, n_a)?;
            let from = corner_of(corners, vertex_id, joint.face_a)?;
            let to = corner_of(corners, vertex_id, joint.face_b)?;
            let edge_id = add_certified_joint_arc(topo, center, circle, from, to, n_b)?;
            arcs.insert((joint.edge.index(), vertex_id.index()), edge_id);
        }
    }
    Ok(arcs)
}

fn arc_of(
    arcs: &BTreeMap<ArcKey, EdgeId>,
    edge: EdgeId,
    vertex: VertexId,
) -> Result<EdgeId, OffsetError> {
    arcs.get(&(edge.index(), vertex.index()))
        .copied()
        .ok_or_else(|| OffsetError::AssemblyFailed {
            reason: format!(
                "edge {} has no joint arc at vertex {}",
                edge.index(),
                vertex.index()
            ),
        })
}

/// Each source face translated by `d` along its own outward normal, keeping
/// its wire order and its reversal flag, so the offset face is congruent to
/// the source and oriented the same way.
fn build_translated_faces(
    topo: &mut Topology,
    faces: &[FaceId],
    corners: &BTreeMap<Corner, VertexId>,
    distance: f64,
    face_lines: &BTreeMap<FaceLine, EdgeId>,
) -> Result<Vec<FaceId>, OffsetError> {
    let mut built = Vec::with_capacity(faces.len());
    for &face_id in faces {
        let face = topo.face(face_id)?;
        let FaceSurface::Plane { normal, d } = *face.surface() else {
            return Err(OffsetError::AssemblyFailed {
                reason: format!("face {} is not planar", face_id.index()),
            });
        };
        let reversed = face.is_reversed();
        // A reversed face's stored normal points inward, so its plane constant
        // moves the other way; this is the rule `offset.rs` uses.
        let surface = FaceSurface::Plane {
            normal,
            d: d + if reversed { -distance } else { distance },
        };

        let source_wire = topo.wire(face.outer_wire())?.edges().to_vec();
        let mut loop_edges = Vec::with_capacity(source_wire.len());
        for oriented in &source_wire {
            let line = *face_lines
                .get(&(oriented.edge().index(), face_id.index()))
                .ok_or_else(|| OffsetError::AssemblyFailed {
                    reason: format!(
                        "edge {} of face {} has no translated image",
                        oriented.edge().index(),
                        face_id.index()
                    ),
                })?;
            loop_edges.push(OrientedEdge::new(line, oriented.is_forward()));
        }

        // Every corner the wire visits must exist, or the translated face is
        // not the source face's image.
        for vertex_id in face_wire_vertices(topo, face_id)? {
            corner_of(corners, vertex_id, face_id)?;
        }

        let wire_id = topo.add_wire(Wire::new(loop_edges, true)?);
        let new_face = if reversed {
            Face::new_reversed(wire_id, vec![], surface)
        } else {
            Face::new(wire_id, vec![], surface)
        };
        built.push(topo.add_face(new_face));
    }
    Ok(built)
}

/// The surface a ball of radius `d` sweeps rolling along each convex edge: a
/// cylindrical patch of that radius about the edge, spanning the angle between
/// the two adjacent outward normals.
fn build_edge_cylinders(
    topo: &mut Topology,
    joint_edges: &[JointEdge],
    normals: &BTreeMap<usize, Vec3>,
    face_lines: &BTreeMap<FaceLine, EdgeId>,
    arcs: &BTreeMap<ArcKey, EdgeId>,
    distance: f64,
) -> Result<Vec<FaceId>, OffsetError> {
    let mut built = Vec::with_capacity(joint_edges.len());
    for joint in joint_edges {
        let n_a = normal_of(normals, joint.face_a)?;
        let n_b = normal_of(normals, joint.face_b)?;
        let origin = topo.vertex(joint.start)?.point();
        // Put the parametric seam (u = 0) diametrically opposite the patch:
        // `project_point` returns u in [0, 2π), so a patch straddling u = 0
        // would come back as two pieces at opposite ends of the domain and
        // measure as the complement of itself.
        let seam_ref = -(n_a + n_b).normalize()?;
        let surface = CylindricalSurface::with_ref_dir(origin, joint.axis, distance, seam_ref)?;

        let line_a = line_of(face_lines, joint.edge, joint.face_a)?;
        let line_b = line_of(face_lines, joint.edge, joint.face_b)?;
        let arc_start = arc_of(arcs, joint.edge, joint.start)?;
        let arc_end = arc_of(arcs, joint.edge, joint.end)?;

        // face_b's translated edge forward, then the far arc backwards from
        // face_b to face_a, then face_a's edge backwards, then the near arc
        // forwards. In the surface's own (u, v) — u rising from n_a to n_b, v
        // rising along the edge — this traverses the patch counter-clockwise,
        // which for the outward-pointing cylinder normal is the outward
        // orientation. `check_oriented_manifold` re-derives this against every
        // neighbouring face rather than taking it on trust.
        let wire_id = topo.add_wire(Wire::new(
            vec![
                OrientedEdge::new(line_b, true),
                OrientedEdge::new(arc_end, false),
                OrientedEdge::new(line_a, false),
                OrientedEdge::new(arc_start, true),
            ],
            true,
        )?);
        built.push(topo.add_face(Face::new(wire_id, vec![], FaceSurface::Cylinder(surface))));
    }
    Ok(built)
}

fn line_of(
    face_lines: &BTreeMap<FaceLine, EdgeId>,
    edge: EdgeId,
    face: FaceId,
) -> Result<EdgeId, OffsetError> {
    face_lines
        .get(&(edge.index(), face.index()))
        .copied()
        .ok_or_else(|| OffsetError::AssemblyFailed {
            reason: format!(
                "edge {} has no translated image on face {}",
                edge.index(),
                face.index()
            ),
        })
}

/// The piece of the ball's own surface it occupies while pivoting at each
/// convex vertex: a spherical patch of radius `d` centred on the vertex,
/// bounded by the end arcs of the cylinders of every edge meeting there.
fn build_vertex_spheres(
    topo: &mut Topology,
    joint_edges: &[JointEdge],
    normals: &BTreeMap<usize, Vec3>,
    arcs: &BTreeMap<ArcKey, EdgeId>,
    distance: f64,
) -> Result<Vec<FaceId>, OffsetError> {
    // Which edges meet at each source vertex, in a deterministic order.
    let mut vertex_edges: BTreeMap<usize, Vec<&JointEdge>> = BTreeMap::new();
    for joint in joint_edges {
        vertex_edges
            .entry(joint.start.index())
            .or_default()
            .push(joint);
        vertex_edges
            .entry(joint.end.index())
            .or_default()
            .push(joint);
    }

    let mut built = Vec::with_capacity(vertex_edges.len());
    for (vertex_index, incident) in vertex_edges {
        let vertex_id =
            topo.vertex_id_from_index(vertex_index)
                .ok_or_else(|| OffsetError::AssemblyFailed {
                    reason: format!("vertex index {vertex_index} not found in arena"),
                })?;
        let center = topo.vertex(vertex_id)?.point();

        // Each cylinder traverses its near arc forwards and its far arc
        // backwards, so the sphere sharing them must do the opposite. That is
        // the manifold rule, and it fixes this patch's orientation without a
        // second orientation argument.
        let mut oriented: Vec<OrientedEdge> = Vec::with_capacity(incident.len());
        let mut patch_faces: BTreeSet<usize> = BTreeSet::new();
        for joint in &incident {
            let arc = arc_of(arcs, joint.edge, vertex_id)?;
            oriented.push(OrientedEdge::new(arc, joint.end == vertex_id));
            patch_faces.insert(joint.face_a.index());
            patch_faces.insert(joint.face_b.index());
        }

        let loop_edges = chain_arc_loop(topo, &oriented, vertex_index)?;

        // Keep both parametric singularities off the patch: the poles at
        // ±z_axis and the u = 0 seam. The patch is the spherical convex hull
        // of its faces' outward normals, so pointing the seam at its antipode
        // and the pole across it puts the patch around (u, v) = (π, 0), where
        // the projection that trims the integration domain is single-valued
        // and well conditioned. A pole inside or on the patch would collapse a
        // whole boundary sample row onto u = 0 and unwrap into a spiral.
        let mut center_dir = Vec3::new(0.0, 0.0, 0.0);
        for &face_index in &patch_faces {
            center_dir += *normals
                .get(&face_index)
                .ok_or_else(|| OffsetError::AssemblyFailed {
                    reason: format!("face {face_index} has no recorded outward normal"),
                })?;
        }
        let center_dir = center_dir.normalize()?;
        let polar = pick_perpendicular(center_dir)?;
        let surface = SphericalSurface::with_frame(center, distance, polar, -center_dir)?;

        let wire_id = topo.add_wire(Wire::new(loop_edges, true)?);
        built.push(topo.add_face(Face::new(wire_id, vec![], FaceSurface::Sphere(surface))));
    }
    Ok(built)
}

/// Order a vertex's oriented joint arcs into a single closed chain.
///
/// Every corner of the patch is the offset image of the vertex on one incident
/// face, and that face contributes exactly two of the vertex's edges, so each
/// corner has one arc arriving and one leaving. A vertex where that fails is
/// not a simple manifold corner and is refused rather than closed by guesswork.
fn chain_arc_loop(
    topo: &Topology,
    oriented: &[OrientedEdge],
    vertex_index: usize,
) -> Result<Vec<OrientedEdge>, OffsetError> {
    if oriented.len() < 3 {
        return Err(OffsetError::AssemblyFailed {
            reason: format!(
                "vertex {vertex_index} has {} joint arcs; a spherical corner patch needs at least 3",
                oriented.len()
            ),
        });
    }

    let mut endpoints = Vec::with_capacity(oriented.len());
    for entry in oriented {
        let edge = topo.edge(entry.edge())?;
        let (from, to) = if entry.is_forward() {
            (edge.start(), edge.end())
        } else {
            (edge.end(), edge.start())
        };
        endpoints.push((from.index(), to.index()));
    }

    let mut used = vec![false; oriented.len()];
    let mut chain = vec![oriented[0]];
    used[0] = true;
    let first = endpoints[0].0;
    let mut current = endpoints[0].1;
    while current != first {
        let next = endpoints
            .iter()
            .enumerate()
            .find(|(index, (from, _))| !used[*index] && *from == current)
            .map(|(index, _)| index)
            .ok_or_else(|| OffsetError::AssemblyFailed {
                reason: format!(
                    "the joint arcs at vertex {vertex_index} do not chain into a closed corner \
                     loop; no unused arc leaves offset corner {current}"
                ),
            })?;
        used[next] = true;
        chain.push(oriented[next]);
        current = endpoints[next].1;
    }
    if !used.iter().all(|&flag| flag) {
        return Err(OffsetError::AssemblyFailed {
            reason: format!(
                "the joint arcs at vertex {vertex_index} form more than one loop; the corner is \
                 not a simple manifold vertex"
            ),
        });
    }
    Ok(chain)
}

/// Any unit vector perpendicular to `dir`, taken against the axis `dir` leans
/// on least so the cross product is well conditioned.
fn pick_perpendicular(dir: Vec3) -> Result<Vec3, OffsetError> {
    let axis = if dir.x().abs() <= dir.y().abs() && dir.x().abs() <= dir.z().abs() {
        Vec3::new(1.0, 0.0, 0.0)
    } else if dir.y().abs() <= dir.z().abs() {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        Vec3::new(0.0, 0.0, 1.0)
    };
    Ok(dir.cross(axis).normalize()?)
}

/// Every edge of the assembled skin must be traversed by exactly two faces,
/// once each way.
///
/// This is the invariant the whole construction rests on. The translated
/// faces, the cylinders and the spheres each settle their own winding from a
/// separate argument, and this is what checks those three arguments agree. A
/// shell that fails it is refused rather than returned as a body that
/// validates as closed while measuring inside out.
fn check_oriented_manifold(topo: &Topology, faces: &[FaceId]) -> Result<(), OffsetError> {
    let mut uses: BTreeMap<usize, (usize, usize)> = BTreeMap::new();
    for &face_id in faces {
        let face = topo.face(face_id)?;
        let flip = face.is_reversed();
        for oriented in topo.wire(face.outer_wire())?.edges() {
            let entry = uses.entry(oriented.edge().index()).or_insert((0, 0));
            if oriented.is_forward() == flip {
                entry.1 += 1;
            } else {
                entry.0 += 1;
            }
        }
    }
    for (edge_index, (forward, backward)) in uses {
        if forward != 1 || backward != 1 {
            return Err(OffsetError::AssemblyFailed {
                reason: format!(
                    "arc-joint edge {edge_index} is traversed {forward} times forward and \
                     {backward} times backward; a closed oriented skin needs exactly one of each"
                ),
            });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::unwrap_used)]

    use super::*;
    use remus_math::vec::Point3;
    use remus_topology::vertex::Vertex;

    #[test]
    fn joint_arc_records_the_positive_minor_cone_range() {
        let mut topo = Topology::new();
        let center = Point3::new(1.0e6, -2.0e6, 3.0e6);
        let circle = Circle3D::new_with_ref(
            center,
            Vec3::new(0.0, 0.0, 1.0),
            0.75,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let expected_end = 1.25;
        let from = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1.0e-10));
        let to = topo.add_vertex(Vertex::new(circle.evaluate(expected_end), 1.0e-10));

        let end_direction = Vec3::new(expected_end.cos(), expected_end.sin(), 0.0);
        let edge_id =
            add_certified_joint_arc(&mut topo, center, circle.clone(), from, to, end_direction)
                .expect("normal-cone arc");
        let edge = topo.edge(edge_id).unwrap();
        let range = edge.strict_domain().expect("explicit arc authority");
        assert!(range.0.abs() <= f64::EPSILON);
        assert!((range.1 - expected_end).abs() <= 4.0 * f64::EPSILON);
        assert!(range.1 > range.0 && range.1 < std::f64::consts::PI);

        let start = topo.vertex(edge.start()).unwrap().point();
        let end = topo.vertex(edge.end()).unwrap().point();
        let midpoint = edge
            .curve()
            .evaluate_with_endpoints((range.0 + range.1) * 0.5, start, end);
        assert!(
            (edge.curve().evaluate_with_endpoints(range.0, start, end) - start).length() < 1e-9
        );
        assert!((edge.curve().evaluate_with_endpoints(range.1, start, end) - end).length() < 1e-9);
        assert!((midpoint - circle.evaluate(expected_end * 0.5)).length() < 1e-9);
    }
}
