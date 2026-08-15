//! Exact constant-radius editing for analytic blend bands.
//!
//! The operation never trusts caller classification. It re-walks the selected
//! face's tangent band, re-measures the radius, identifies the two exact
//! supports, restores their sharp intersection, and only then runs the normal
//! fillet construction at the requested radius. Any ambiguity is a refusal and
//! the topology arena is restored to its pre-call state.

use std::collections::{HashMap, HashSet, VecDeque};

use brepkit_math::curves::Circle3D;
use brepkit_math::tolerance::Tolerance;
use brepkit_math::vec::Vec3;
use brepkit_topology::Topology;
use brepkit_topology::edge::{Edge, EdgeCurve, EdgeId};
use brepkit_topology::face::{FaceId, FaceSurface};
use brepkit_topology::shell::Shell;
use brepkit_topology::solid::{Solid, SolidId};
use brepkit_topology::vertex::Vertex;
use brepkit_topology::wire::{OrientedEdge, Wire, WireId};

use crate::OperationsError;
use crate::blend_ops::{BlendFaceOrigins, fillet_v2};
use crate::evolution::EvolutionMap;

/// Stable, machine-readable refusal from [`resize_blend`].
#[derive(Debug, thiserror::Error)]
pub enum ResizeBlendError {
    /// Caller arguments or handles cannot name a resize request.
    #[error("invalid input: {reason}")]
    InvalidInput {
        /// Refusal detail.
        reason: String,
    },
    /// The selected face is not a supported analytic blend surface.
    #[error("selected face is not an analytic torus/cylinder blend: {surface}")]
    BandNotAnalytic {
        /// Selected surface type.
        surface: &'static str,
    },
    /// Caller radius does not match the exact band radius.
    #[error("expected radius {expected} mm, but the exact band radius is {actual} mm")]
    RadiusMismatch {
        /// Caller-provided witness.
        expected: f64,
        /// Exact topology measurement.
        actual: f64,
    },
    /// The band closes into or is supported by freeform geometry.
    #[error("blend band touches a freeform face")]
    BandTouchesFreeform,
    /// The support surfaces cannot be reconstructed exactly in this phase.
    #[error("unsupported analytic support pair: {first} x {second}")]
    UnsupportedSupportPair {
        /// First support surface type.
        first: &'static str,
        /// Second support surface type.
        second: &'static str,
    },
    /// The requested radius cannot fit on the recovered sharp feature.
    #[error("radius {radius} mm does not fit on the recovered sharp feature")]
    RadiusTooLarge {
        /// Requested radius.
        radius: f64,
    },
    /// Exact topology analysis or reconstruction could not produce one answer.
    #[error("exact reconstruction refused: {reason}")]
    ReconstructionFailed {
        /// Refusal detail.
        reason: String,
    },
}

impl ResizeBlendError {
    /// Stable code used across the WASM boundary.
    #[must_use]
    pub const fn code(&self) -> &'static str {
        match self {
            Self::InvalidInput { .. } => "invalid-input",
            Self::BandNotAnalytic { .. } => "blend-band-not-analytic",
            Self::RadiusMismatch { .. } => "blend-radius-mismatch",
            Self::BandTouchesFreeform => "band-touches-freeform",
            Self::UnsupportedSupportPair { .. } => "unsupported-support-pair",
            Self::RadiusTooLarge { .. } => "radius-too-large",
            Self::ReconstructionFailed { .. } => "resize-blend-failed",
        }
    }
}

/// Result of an exact band resize, including source-to-result face evolution.
#[derive(Debug)]
pub struct ResizeBlendResult {
    /// Edited solid.
    pub solid: SolidId,
    /// Face evolution from the caller's input solid to [`Self::solid`].
    pub evolution: EvolutionMap,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BlendKind {
    Cylinder,
    Torus,
}

#[derive(Debug)]
struct BandDescription {
    faces: Vec<FaceId>,
    supports: [FaceId; 2],
    radius: f64,
}

struct SharpResult {
    solid: SolidId,
    edges: Vec<EdgeId>,
    face_map: HashMap<usize, FaceId>,
}

/// Resize or remove a constant-radius analytic blend band.
///
/// The selected face is only a seed. Band membership, supports and the current
/// radius are re-derived from exact topology. `expected_radius` is a replay
/// witness and must match that measurement through [`Tolerance::approx_eq`].
/// A zero `new_radius` restores the sharp support intersection; a positive
/// value rebuilds the band from that sharp topology. Negative/non-finite
/// values, freeform closure, ambiguous supports, invalid output and implausible
/// volume changes all refuse. On every refusal the arena, including existing
/// handle slots, is restored exactly.
///
/// # Errors
///
/// Returns [`OperationsError::ResizeBlend`] with a stable reason when the
/// request has no exact construction, or another typed operations error when
/// input topology cannot be read. Failure is a true no-op.
pub fn resize_blend(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    expected_radius: f64,
    new_radius: f64,
) -> Result<ResizeBlendResult, OperationsError> {
    let snapshot = topo.clone();
    let result = resize_blend_impl(topo, solid, face, expected_radius, new_radius);
    if result.is_err() {
        topo.restore_preserving_handle_slots(&snapshot);
    }
    result
}

fn resize_blend_impl(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    expected_radius: f64,
    new_radius: f64,
) -> Result<ResizeBlendResult, OperationsError> {
    if !expected_radius.is_finite() || expected_radius <= 0.0 {
        return Err(invalid("expected_radius must be finite and positive"));
    }
    if !new_radius.is_finite() || new_radius < 0.0 {
        return Err(invalid("new_radius must be finite and non-negative"));
    }
    if !brepkit_topology::explorer::solid_faces(topo, solid)?.contains(&face) {
        return Err(invalid(format!(
            "face {} is not part of solid {}",
            face.index(),
            solid.index()
        )));
    }

    let band = describe_band(topo, solid, face)?;
    let tol = Tolerance::new();
    if !tol.approx_eq(expected_radius, band.radius) {
        return Err(ResizeBlendError::RadiusMismatch {
            expected: expected_radius,
            actual: band.radius,
        }
        .into());
    }

    if tol.approx_eq(new_radius, band.radius) {
        return copy_unchanged(topo, solid);
    }

    let input_volume = crate::measure::solid_volume(topo, solid, 0.05)?;
    let support_types = [
        topo.face(band.supports[0])?.surface().type_tag(),
        topo.face(band.supports[1])?.surface().type_tag(),
    ];
    let sharp = match support_types {
        ["plane", "plane"] => heal_planar_band(topo, solid, &band)?,
        ["plane", "cylinder"] | ["cylinder", "plane"] => {
            heal_plane_cylinder_band(topo, solid, &band)?
        }
        ["cylinder", "cone"] | ["cone", "cylinder"] => {
            if !tol.approx_eq(new_radius, 0.0) {
                return Err(ResizeBlendError::UnsupportedSupportPair {
                    first: support_types[0],
                    second: support_types[1],
                }
                .into());
            }
            heal_cylinder_cone_band(topo, solid, &band)?
        }
        [first, second] => {
            return Err(ResizeBlendError::UnsupportedSupportPair { first, second }.into());
        }
    };

    validate_exact_result(topo, sharp.solid, "sharp support reconstruction")?;
    let sharp_volume = crate::measure::solid_volume(topo, sharp.solid, 0.05)?;
    if tol.approx_eq(new_radius, 0.0) {
        validate_volume_progress(input_volume, sharp_volume, sharp_volume, band.radius, 0.0)?;
        return Ok(ResizeBlendResult {
            solid: sharp.solid,
            evolution: heal_evolution(&sharp.face_map, &band.faces),
        });
    }

    let chains =
        brepkit_blend::g1_chain::g1_chains(topo, sharp.solid, &sharp.edges, Tolerance::new())
            .map_err(|error| {
                reconstruction(format!("recovered sharp-edge walk failed: {error}"))
            })?;
    if chains.len() != 1 || chains[0].is_empty() {
        return Err(reconstruction(format!(
            "recovered support intersection produced {} sharp-edge chains",
            chains.len()
        )));
    }

    let rebuilt = match fillet_v2(topo, sharp.solid, &chains[0], new_radius) {
        Ok(result) => result,
        Err(OperationsError::Blend(brepkit_blend::BlendError::RadiusTooLarge { .. })) => {
            return Err(ResizeBlendError::RadiusTooLarge { radius: new_radius }.into());
        }
        Err(OperationsError::Blend(brepkit_blend::BlendError::TrimmingFailure { .. }))
            if new_radius > band.radius =>
        {
            return Err(ResizeBlendError::RadiusTooLarge { radius: new_radius }.into());
        }
        Err(error) => {
            return Err(reconstruction(format!(
                "fillet reconstruction at {new_radius} mm failed: {error}"
            )));
        }
    };
    validate_exact_result(topo, rebuilt.solid, "resized blend")?;
    let result_volume = crate::measure::solid_volume(topo, rebuilt.solid, 0.05)?;
    validate_volume_progress(
        input_volume,
        sharp_volume,
        result_volume,
        band.radius,
        new_radius,
    )?;
    let evolution = compose_evolution(&sharp.face_map, &band.faces, rebuilt.face_origins.as_ref());
    Ok(ResizeBlendResult {
        solid: rebuilt.solid,
        evolution,
    })
}

fn invalid(reason: impl Into<String>) -> OperationsError {
    ResizeBlendError::InvalidInput {
        reason: reason.into(),
    }
    .into()
}

fn reconstruction(reason: impl Into<String>) -> OperationsError {
    ResizeBlendError::ReconstructionFailed {
        reason: reason.into(),
    }
    .into()
}

fn blend_surface(surface: &FaceSurface) -> Option<(BlendKind, f64)> {
    match surface {
        FaceSurface::Cylinder(cylinder) => Some((BlendKind::Cylinder, cylinder.radius())),
        FaceSurface::Torus(torus) => Some((BlendKind::Torus, torus.minor_radius())),
        _ => None,
    }
}

fn face_edges(topo: &Topology, face: FaceId) -> Result<Vec<EdgeId>, OperationsError> {
    let face = topo.face(face)?;
    let mut edges = Vec::new();
    for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        edges.extend(topo.wire(wire)?.edges().iter().map(OrientedEdge::edge));
    }
    edges.sort_unstable_by_key(|edge| edge.index());
    edges.dedup();
    Ok(edges)
}

fn distinct_faces(faces: &[FaceId]) -> Vec<FaceId> {
    let mut result = faces.to_vec();
    result.sort_unstable_by_key(|face| face.index());
    result.dedup();
    result
}

fn tangent_across(
    topo: &Topology,
    edge: EdgeId,
    first: FaceId,
    second: FaceId,
) -> Result<bool, OperationsError> {
    let pair = HashSet::from([first, second]);
    crate::query::edge_is_tangent(topo, edge, &pair)
}

fn describe_band(
    topo: &Topology,
    solid: SolidId,
    seed: FaceId,
) -> Result<BandDescription, OperationsError> {
    let seed_surface = topo.face(seed)?.surface();
    let Some((_, radius)) = blend_surface(seed_surface) else {
        return Err(ResizeBlendError::BandNotAnalytic {
            surface: seed_surface.type_tag(),
        }
        .into());
    };
    let tol = Tolerance::new();
    let adjacency = topo.build_adjacency(solid)?;
    let mut band = HashSet::from([seed]);
    let mut queue = VecDeque::from([seed]);

    while let Some(current) = queue.pop_front() {
        for edge in face_edges(topo, current)? {
            let adjacent = distinct_faces(adjacency.faces_for_edge(edge));
            let others: Vec<FaceId> = adjacent
                .into_iter()
                .filter(|candidate| *candidate != current)
                .collect();
            let [other] = others.as_slice() else {
                continue;
            };
            if !tangent_across(topo, edge, current, *other)? {
                continue;
            }
            if let Some((_, other_radius)) = blend_surface(topo.face(*other)?.surface())
                && tol.approx_eq(radius, other_radius)
                && band.insert(*other)
            {
                queue.push_back(*other);
            }
        }
    }

    let mut supports = HashSet::new();
    for &band_face in &band {
        for edge in face_edges(topo, band_face)? {
            let adjacent = distinct_faces(adjacency.faces_for_edge(edge));
            let others: Vec<FaceId> = adjacent
                .into_iter()
                .filter(|candidate| *candidate != band_face && !band.contains(candidate))
                .collect();
            for other in others {
                let surface = topo.face(other)?.surface();
                let tangent = tangent_across(topo, edge, band_face, other)?;
                if matches!(surface, FaceSurface::Nurbs(_)) {
                    return Err(ResizeBlendError::BandTouchesFreeform.into());
                }
                if tangent {
                    supports.insert(other);
                }
            }
        }
    }

    let mut supports: Vec<FaceId> = supports.into_iter().collect();
    supports.sort_unstable_by_key(|support| support.index());
    let [first, second] = supports.as_slice() else {
        return Err(reconstruction(format!(
            "band has {} tangent support faces; exactly two are required",
            supports.len()
        )));
    };

    let mut faces: Vec<FaceId> = band.into_iter().collect();
    faces.sort_unstable_by_key(|face| face.index());
    Ok(BandDescription {
        faces,
        supports: [*first, *second],
        radius,
    })
}

fn copy_unchanged(
    topo: &mut Topology,
    solid: SolidId,
) -> Result<ResizeBlendResult, OperationsError> {
    let (copy, map) = crate::copy::copy_solid_with_face_map(topo, solid)?;
    let mut evolution = EvolutionMap::exact();
    for (source, result) in map {
        evolution.add_modified(source, result);
    }
    Ok(ResizeBlendResult {
        solid: copy,
        evolution,
    })
}

fn heal_planar_band(
    topo: &mut Topology,
    solid: SolidId,
    band: &BandDescription,
) -> Result<SharpResult, OperationsError> {
    let outcome = crate::defeature::defeature_blend_band(topo, solid, &band.faces)
        .map_err(|error| reconstruction(format!("planar support heal failed: {error}")))?;
    let support_a = outcome
        .face_map
        .get(&band.supports[0].index())
        .copied()
        .ok_or_else(|| reconstruction("first support was consumed by planar heal"))?;
    let support_b = outcome
        .face_map
        .get(&band.supports[1].index())
        .copied()
        .ok_or_else(|| reconstruction("second support was consumed by planar heal"))?;
    let adjacency = topo.build_adjacency(outcome.solid)?;
    let mut edges = Vec::new();
    for edge in face_edges(topo, support_a)? {
        let adjacent = distinct_faces(adjacency.faces_for_edge(edge));
        if adjacent.contains(&support_a) && adjacent.contains(&support_b) {
            edges.push(edge);
        }
    }
    if edges.is_empty() {
        return Err(reconstruction(
            "healed planar supports do not share a sharp edge",
        ));
    }
    Ok(SharpResult {
        solid: outcome.solid,
        edges,
        face_map: outcome.face_map,
    })
}

fn shared_edges(
    topo: &Topology,
    solid: SolidId,
    first: FaceId,
    second: FaceId,
) -> Result<Vec<EdgeId>, OperationsError> {
    let adjacency = topo.build_adjacency(solid)?;
    let mut edges = Vec::new();
    for edge in face_edges(topo, first)? {
        let adjacent = distinct_faces(adjacency.faces_for_edge(edge));
        if adjacent.contains(&first) && adjacent.contains(&second) {
            edges.push(edge);
        }
    }
    Ok(edges)
}

fn wire_containing_edge(
    topo: &Topology,
    face: FaceId,
    edge: EdgeId,
) -> Result<(WireId, Vec<OrientedEdge>), OperationsError> {
    let face_id = face;
    let face_data = topo.face(face_id)?;
    for wire_id in
        std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
    {
        let wire = topo.wire(wire_id)?;
        if wire.edges().iter().any(|oriented| oriented.edge() == edge) {
            return Ok((wire_id, wire.edges().to_vec()));
        }
    }
    Err(reconstruction(format!(
        "face {} lost contact edge {}",
        face_id.index(),
        edge.index()
    )))
}

fn replace_face_wire(
    topo: &mut Topology,
    face: FaceId,
    old_wire: WireId,
    new_wire: WireId,
) -> Result<(), OperationsError> {
    let face = topo.face_mut(face)?;
    if face.outer_wire() == old_wire {
        face.set_outer_wire(new_wire);
        return Ok(());
    }
    let Some(slot) = face
        .inner_wires()
        .iter()
        .position(|candidate| *candidate == old_wire)
    else {
        return Err(reconstruction(format!(
            "face {} lost wire {}",
            face.outer_wire().index(),
            old_wire.index()
        )));
    };
    face.inner_wires_mut()[slot] = new_wire;
    Ok(())
}

fn oriented_replacement(
    topo: &Topology,
    old: OrientedEdge,
    new_edge: EdgeId,
    new_curve_normal: Vec3,
) -> Result<OrientedEdge, OperationsError> {
    let old_edge = topo.edge(old.edge())?;
    let EdgeCurve::Circle(old_circle) = old_edge.curve() else {
        return Ok(OrientedEdge::new(new_edge, old.is_forward()));
    };
    let aligned = old_circle.normal().dot(new_curve_normal) >= 0.0;
    Ok(OrientedEdge::new(
        new_edge,
        if aligned {
            old.is_forward()
        } else {
            !old.is_forward()
        },
    ))
}

#[allow(clippy::too_many_lines)]
fn heal_plane_cylinder_band(
    topo: &mut Topology,
    solid: SolidId,
    band: &BandDescription,
) -> Result<SharpResult, OperationsError> {
    if band.faces.len() != 1 {
        return Err(reconstruction(
            "plane/cylinder reconstruction requires one closed analytic band face",
        ));
    }
    if !topo.solid(solid)?.inner_shells().is_empty() {
        return Err(reconstruction(
            "plane/cylinder reconstruction does not edit cavity shells",
        ));
    }
    let (plane_source, cylinder_source) = match (
        topo.face(band.supports[0])?.surface(),
        topo.face(band.supports[1])?.surface(),
    ) {
        (FaceSurface::Plane { .. }, FaceSurface::Cylinder(_)) => {
            (band.supports[0], band.supports[1])
        }
        (FaceSurface::Cylinder(_), FaceSurface::Plane { .. }) => {
            (band.supports[1], band.supports[0])
        }
        _ => return Err(reconstruction("support classification changed during heal")),
    };

    let (copy, mut face_map_indices) = crate::copy::copy_solid_with_face_map(topo, solid)?;
    let copied = |map: &HashMap<usize, usize>, source: FaceId| {
        map.get(&source.index())
            .and_then(|index| topo.face_id_from_index(*index))
            .ok_or_else(|| reconstruction(format!("face {} was not copied", source.index())))
    };
    let plane = copied(&face_map_indices, plane_source)?;
    let cylinder = copied(&face_map_indices, cylinder_source)?;
    let band_face = copied(&face_map_indices, band.faces[0])?;

    let plane_contacts = shared_edges(topo, copy, plane, band_face)?;
    let cylinder_contacts = shared_edges(topo, copy, cylinder, band_face)?;
    let ([plane_contact], [cylinder_contact]) =
        (plane_contacts.as_slice(), cylinder_contacts.as_slice())
    else {
        return Err(reconstruction(format!(
            "closed rim needs one contact on each support; found {} and {}",
            plane_contacts.len(),
            cylinder_contacts.len()
        )));
    };
    if !topo.edge(*plane_contact)?.is_closed() || !topo.edge(*cylinder_contact)?.is_closed() {
        return Err(reconstruction(
            "plane/cylinder support contacts are not full closed loops",
        ));
    }

    let (plane_wire_id, plane_wire) = wire_containing_edge(topo, plane, *plane_contact)?;
    if plane_wire.len() != 1 {
        return Err(reconstruction(
            "plane contact loop contains more than the analytic rim edge",
        ));
    }
    let (cylinder_wire_id, cylinder_wire) =
        wire_containing_edge(topo, cylinder, *cylinder_contact)?;

    let (plane_normal, plane_d) = match topo.face(plane)?.surface() {
        FaceSurface::Plane { normal, d } => (*normal, *d),
        _ => return Err(reconstruction("plane support lost its surface")),
    };
    let cylinder_surface = match topo.face(cylinder)?.surface() {
        FaceSurface::Cylinder(surface) => surface.clone(),
        _ => return Err(reconstruction("cylinder support lost its surface")),
    };
    let axis = cylinder_surface
        .axis()
        .normalize()
        .map_err(|error| reconstruction(format!("invalid cylinder axis: {error}")))?;
    let normal = plane_normal
        .normalize()
        .map_err(|error| reconstruction(format!("invalid plane normal: {error}")))?;
    let denominator = normal.dot(axis);
    if denominator.abs() < 1.0 - Tolerance::new().angular {
        return Err(ResizeBlendError::UnsupportedSupportPair {
            first: "oblique-plane",
            second: "cylinder",
        }
        .into());
    }
    let origin = cylinder_surface.origin();
    let plane_origin_dot = normal.dot(Vec3::new(origin.x(), origin.y(), origin.z()));
    let center = origin + axis * ((plane_d - plane_origin_dot) / denominator);

    let old_contact_vertex = topo.edge(*cylinder_contact)?.start();
    let old_vertex = topo.vertex(old_contact_vertex)?.point();
    let offset = old_vertex - cylinder_surface.origin();
    let radial = (offset - axis * offset.dot(axis))
        .normalize()
        .map_err(|error| reconstruction(format!("invalid cylinder seam: {error}")))?;
    let sharp_point = center + radial * cylinder_surface.radius();
    let sharp_vertex = topo.add_vertex(Vertex::new(sharp_point, Tolerance::new().linear));

    let plane_circle_normal = match topo.edge(*plane_contact)?.curve() {
        EdgeCurve::Circle(circle) => circle.normal(),
        _ => return Err(reconstruction("plane contact is not an exact circle")),
    };
    let circle = Circle3D::new_with_ref(
        center,
        plane_circle_normal,
        cylinder_surface.radius(),
        radial,
    )
    .map_err(|error| reconstruction(format!("sharp circle construction failed: {error}")))?;
    let circle_normal = circle.normal();
    let sharp_edge = topo.add_edge(Edge::new(
        sharp_vertex,
        sharp_vertex,
        EdgeCurve::Circle(circle),
    ));

    let plane_oriented = oriented_replacement(topo, plane_wire[0], sharp_edge, circle_normal)?;
    let plane_new_wire = topo.add_wire(Wire::new(vec![plane_oriented], true)?);
    replace_face_wire(topo, plane, plane_wire_id, plane_new_wire)?;

    let mut seam_candidates: Vec<EdgeId> = cylinder_wire
        .iter()
        .map(OrientedEdge::edge)
        .filter(|edge| *edge != *cylinder_contact)
        .filter(|edge| {
            matches!(
                topo.edge(*edge).map(brepkit_topology::edge::Edge::curve),
                Ok(EdgeCurve::Line)
            )
        })
        .collect();
    seam_candidates.sort_unstable_by_key(|edge| edge.index());
    seam_candidates.dedup();
    let [old_seam] = seam_candidates.as_slice() else {
        return Err(reconstruction(format!(
            "closed cylinder support has {} seam edges; exactly one is required",
            seam_candidates.len()
        )));
    };
    let seam = topo.edge(*old_seam)?;
    let (seam_start, seam_end) = if seam.start() == old_contact_vertex {
        (sharp_vertex, seam.end())
    } else if seam.end() == old_contact_vertex {
        (seam.start(), sharp_vertex)
    } else {
        return Err(reconstruction(
            "cylinder seam does not terminate on the band contact",
        ));
    };
    let sharp_seam = topo.add_edge(Edge::new(seam_start, seam_end, EdgeCurve::Line));
    let mut cylinder_edges = Vec::with_capacity(cylinder_wire.len());
    for oriented in cylinder_wire {
        if oriented.edge() == *cylinder_contact {
            cylinder_edges.push(oriented_replacement(
                topo,
                oriented,
                sharp_edge,
                circle_normal,
            )?);
        } else if oriented.edge() == *old_seam {
            cylinder_edges.push(OrientedEdge::new(sharp_seam, oriented.is_forward()));
        } else {
            cylinder_edges.push(oriented);
        }
    }
    let cylinder_new_wire = topo.add_wire(Wire::new(cylinder_edges, true)?);
    replace_face_wire(topo, cylinder, cylinder_wire_id, cylinder_new_wire)?;

    let old_shell = topo.solid(copy)?.outer_shell();
    let kept_faces: Vec<FaceId> = topo
        .shell(old_shell)?
        .faces()
        .iter()
        .copied()
        .filter(|face| *face != band_face)
        .collect();
    let shell = topo.add_shell(Shell::new(kept_faces)?);
    let sharp_solid = topo.add_solid(Solid::new(shell, Vec::new()));
    face_map_indices.remove(&band.faces[0].index());
    let face_map = face_map_indices
        .into_iter()
        .filter_map(|(source, result)| topo.face_id_from_index(result).map(|face| (source, face)))
        .collect();
    Ok(SharpResult {
        solid: sharp_solid,
        edges: vec![sharp_edge],
        face_map,
    })
}

fn contact_wire(
    topo: &Topology,
    face: FaceId,
    contacts: &HashSet<EdgeId>,
) -> Result<(WireId, Vec<OrientedEdge>), OperationsError> {
    let face_data = topo.face(face)?;
    let mut matches = Vec::new();
    for wire_id in
        std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
    {
        let wire = topo.wire(wire_id)?;
        if wire
            .edges()
            .iter()
            .any(|oriented| contacts.contains(&oriented.edge()))
        {
            matches.push((wire_id, wire.edges().to_vec()));
        }
    }
    let [result] = matches.as_slice() else {
        return Err(reconstruction(format!(
            "support face {} has band contacts on {} wires; exactly one is required",
            face.index(),
            matches.len()
        )));
    };
    let found: HashSet<EdgeId> = result
        .1
        .iter()
        .map(OrientedEdge::edge)
        .filter(|edge| contacts.contains(edge))
        .collect();
    if found != *contacts {
        return Err(reconstruction(format!(
            "support face {} does not carry every band contact edge",
            face.index()
        )));
    }
    Ok(result.clone())
}

fn other_circle_edges(
    topo: &Topology,
    wire: &[OrientedEdge],
    contacts: &HashSet<EdgeId>,
) -> Result<HashSet<EdgeId>, OperationsError> {
    let circles: HashSet<EdgeId> = wire
        .iter()
        .map(OrientedEdge::edge)
        .filter(|edge| !contacts.contains(edge))
        .filter(|edge| {
            matches!(
                topo.edge(*edge).map(brepkit_topology::edge::Edge::curve),
                Ok(EdgeCurve::Circle(_))
            )
        })
        .collect();
    if circles.is_empty() {
        return Err(reconstruction(
            "periodic support has no opposite circular boundary",
        ));
    }
    Ok(circles)
}

fn boundary_directions(
    topo: &Topology,
    circles: &HashSet<EdgeId>,
) -> Result<Vec<(Vec3, brepkit_topology::vertex::VertexId)>, OperationsError> {
    let mut directions = Vec::new();
    let mut seen = HashSet::new();
    for edge_id in circles {
        let edge = topo.edge(*edge_id)?;
        let EdgeCurve::Circle(circle) = edge.curve() else {
            continue;
        };
        for vertex in [edge.start(), edge.end()] {
            if !seen.insert(vertex.index()) {
                continue;
            }
            let point = topo.vertex(vertex)?.point();
            let direction = (point - circle.center())
                .normalize()
                .map_err(|error| reconstruction(format!("invalid periodic rim: {error}")))?;
            directions.push((direction, vertex));
        }
    }
    Ok(directions)
}

fn common_boundary_direction(
    topo: &Topology,
    first: &HashSet<EdgeId>,
    second: &HashSet<EdgeId>,
) -> Result<
    (
        Vec3,
        brepkit_topology::vertex::VertexId,
        brepkit_topology::vertex::VertexId,
    ),
    OperationsError,
> {
    let first = boundary_directions(topo, first)?;
    let second = boundary_directions(topo, second)?;
    let mut candidates = Vec::new();
    for &(first_direction, first_vertex) in &first {
        for &(second_direction, second_vertex) in &second {
            if (first_direction - second_direction).length() <= Tolerance::new().linear {
                candidates.push((first_direction, first_vertex, second_vertex));
            }
        }
    }
    candidates.sort_by(|left, right| {
        left.0
            .x()
            .total_cmp(&right.0.x())
            .then_with(|| left.0.y().total_cmp(&right.0.y()))
            .then_with(|| left.0.z().total_cmp(&right.0.z()))
    });
    candidates.dedup_by(|left, right| (left.0 - right.0).length() <= Tolerance::new().linear);
    let Some(candidate) = candidates.first().copied() else {
        return Err(reconstruction(
            "support seams have no common analytic radial direction",
        ));
    };
    Ok(candidate)
}

fn ordered_circle_boundary(
    topo: &Topology,
    wire: &[OrientedEdge],
    circles: &HashSet<EdgeId>,
    start_vertex: brepkit_topology::vertex::VertexId,
) -> Result<Vec<OrientedEdge>, OperationsError> {
    let mut remaining: Vec<OrientedEdge> = wire
        .iter()
        .copied()
        .filter(|oriented| circles.contains(&oriented.edge()))
        .collect();
    let mut result = Vec::with_capacity(remaining.len());
    let mut current = start_vertex;
    while !remaining.is_empty() {
        let Some(index) = remaining.iter().position(|oriented| {
            topo.edge(oriented.edge())
                .is_ok_and(|edge| oriented.oriented_start(edge) == current)
        }) else {
            return Err(reconstruction(
                "opposite circular boundary cannot be ordered from the selected seam",
            ));
        };
        let oriented = remaining.remove(index);
        current = oriented.oriented_end(topo.edge(oriented.edge())?);
        result.push(oriented);
    }
    if result.len() != circles.len() || current != start_vertex {
        return Err(reconstruction(
            "opposite circular boundary is not one closed analytic cycle",
        ));
    }
    Ok(result)
}

fn contact_direction(
    topo: &Topology,
    wire: &[OrientedEdge],
    contacts: &HashSet<EdgeId>,
    axis: Vec3,
) -> Result<bool, OperationsError> {
    let Some(oriented) = wire.iter().find(|edge| contacts.contains(&edge.edge())) else {
        return Err(reconstruction("support contact is empty"));
    };
    let EdgeCurve::Circle(circle) = topo.edge(oriented.edge())?.curve() else {
        return Err(reconstruction("support contact is not circular"));
    };
    Ok(if circle.normal().dot(axis) >= 0.0 {
        oriented.is_forward()
    } else {
        !oriented.is_forward()
    })
}

#[allow(clippy::too_many_arguments)]
fn rebuild_closed_periodic_support(
    topo: &mut Topology,
    face: FaceId,
    contacts: &HashSet<EdgeId>,
    sharp_edge: EdgeId,
    sharp_vertex: brepkit_topology::vertex::VertexId,
    far_vertex: brepkit_topology::vertex::VertexId,
    far_circles: &HashSet<EdgeId>,
    axis: Vec3,
) -> Result<(), OperationsError> {
    let (wire_id, old_wire) = contact_wire(topo, face, contacts)?;
    let forward = contact_direction(topo, &old_wire, contacts, axis)?;
    let far_boundary = ordered_circle_boundary(topo, &old_wire, far_circles, far_vertex)?;
    let seam = topo.add_edge(Edge::new(far_vertex, sharp_vertex, EdgeCurve::Line));
    let mut edges = Vec::with_capacity(far_boundary.len() + 3);
    edges.push(OrientedEdge::new(seam, true));
    edges.push(OrientedEdge::new(sharp_edge, forward));
    edges.push(OrientedEdge::new(seam, false));
    edges.extend(far_boundary);
    let new_wire = topo.add_wire(Wire::new(edges, true)?);
    replace_face_wire(topo, face, wire_id, new_wire)
}

#[allow(clippy::too_many_lines)]
fn heal_cylinder_cone_band(
    topo: &mut Topology,
    solid: SolidId,
    band: &BandDescription,
) -> Result<SharpResult, OperationsError> {
    if band.faces.len() != 1 {
        return Err(reconstruction(
            "cylinder/cone reconstruction requires one closed analytic band face",
        ));
    }
    if !topo.solid(solid)?.inner_shells().is_empty() {
        return Err(reconstruction(
            "cylinder/cone reconstruction does not edit cavity shells",
        ));
    }
    let (cylinder_source, cone_source) = match (
        topo.face(band.supports[0])?.surface(),
        topo.face(band.supports[1])?.surface(),
    ) {
        (FaceSurface::Cylinder(_), FaceSurface::Cone(_)) => (band.supports[0], band.supports[1]),
        (FaceSurface::Cone(_), FaceSurface::Cylinder(_)) => (band.supports[1], band.supports[0]),
        _ => return Err(reconstruction("support classification changed during heal")),
    };

    let (copy, mut face_map_indices) = crate::copy::copy_solid_with_face_map(topo, solid)?;
    let copied = |map: &HashMap<usize, usize>, source: FaceId| {
        map.get(&source.index())
            .and_then(|index| topo.face_id_from_index(*index))
            .ok_or_else(|| reconstruction(format!("face {} was not copied", source.index())))
    };
    let cylinder = copied(&face_map_indices, cylinder_source)?;
    let cone = copied(&face_map_indices, cone_source)?;
    let band_face = copied(&face_map_indices, band.faces[0])?;
    let cylinder_contacts: HashSet<EdgeId> = shared_edges(topo, copy, cylinder, band_face)?
        .into_iter()
        .collect();
    let cone_contacts: HashSet<EdgeId> = shared_edges(topo, copy, cone, band_face)?
        .into_iter()
        .collect();
    if cylinder_contacts.is_empty() || cone_contacts.is_empty() {
        return Err(reconstruction(
            "cylinder/cone band has no exact contact on one support",
        ));
    }

    let cylinder_surface = match topo.face(cylinder)?.surface() {
        FaceSurface::Cylinder(surface) => surface.clone(),
        _ => return Err(reconstruction("cylinder support lost its surface")),
    };
    let cone_surface = match topo.face(cone)?.surface() {
        FaceSurface::Cone(surface) => surface.clone(),
        _ => return Err(reconstruction("cone support lost its surface")),
    };
    let cylinder_axis = cylinder_surface
        .axis()
        .normalize()
        .map_err(|error| reconstruction(format!("invalid cylinder axis: {error}")))?;
    let cone_axis = cone_surface
        .axis()
        .normalize()
        .map_err(|error| reconstruction(format!("invalid cone axis: {error}")))?;
    if cylinder_axis.dot(cone_axis).abs() < 1.0 - Tolerance::new().angular {
        return Err(ResizeBlendError::UnsupportedSupportPair {
            first: "non-coaxial-cylinder",
            second: "cone",
        }
        .into());
    }
    let axis = if cylinder_axis.dot(cone_axis) >= 0.0 {
        cone_axis
    } else {
        -cone_axis
    };
    let axis_offset = cylinder_surface.origin() - cone_surface.apex();
    let radial_offset = axis_offset - axis * axis_offset.dot(axis);
    if radial_offset.length() > Tolerance::new().linear {
        return Err(ResizeBlendError::UnsupportedSupportPair {
            first: "non-coaxial-cylinder",
            second: "cone",
        }
        .into());
    }

    let sample_edge = *cone_contacts
        .iter()
        .next()
        .ok_or_else(|| reconstruction("cone contact disappeared"))?;
    let sample = topo.vertex(topo.edge(sample_edge)?.start())?.point();
    let sample_height = (sample - cone_surface.apex()).dot(axis);
    if sample_height.abs() <= Tolerance::new().linear {
        return Err(reconstruction("cone contact lies at the cone apex"));
    }
    let cone_slope = cone_surface.half_angle().tan();
    if cone_slope.abs() <= Tolerance::new().angular {
        return Err(reconstruction("cone support has no radial slope"));
    }
    let sharp_height = sample_height.signum() * cylinder_surface.radius() / cone_slope;
    let center = cone_surface.apex() + axis * sharp_height;

    let (_, cylinder_wire) = contact_wire(topo, cylinder, &cylinder_contacts)?;
    let (_, cone_wire) = contact_wire(topo, cone, &cone_contacts)?;
    let cylinder_far = other_circle_edges(topo, &cylinder_wire, &cylinder_contacts)?;
    let cone_far = other_circle_edges(topo, &cone_wire, &cone_contacts)?;
    let (direction, cylinder_far_vertex, cone_far_vertex) =
        common_boundary_direction(topo, &cylinder_far, &cone_far)?;
    let sharp_point = center + direction * cylinder_surface.radius();
    let sharp_vertex = topo.add_vertex(Vertex::new(sharp_point, Tolerance::new().linear));
    let circle = Circle3D::new_with_ref(center, axis, cylinder_surface.radius(), direction)
        .map_err(|error| reconstruction(format!("sharp circle failed: {error}")))?;
    let sharp_edge = topo.add_edge(Edge::new(
        sharp_vertex,
        sharp_vertex,
        EdgeCurve::Circle(circle),
    ));
    rebuild_closed_periodic_support(
        topo,
        cylinder,
        &cylinder_contacts,
        sharp_edge,
        sharp_vertex,
        cylinder_far_vertex,
        &cylinder_far,
        axis,
    )?;
    rebuild_closed_periodic_support(
        topo,
        cone,
        &cone_contacts,
        sharp_edge,
        sharp_vertex,
        cone_far_vertex,
        &cone_far,
        axis,
    )?;

    let old_shell = topo.solid(copy)?.outer_shell();
    let kept_faces: Vec<FaceId> = topo
        .shell(old_shell)?
        .faces()
        .iter()
        .copied()
        .filter(|face| *face != band_face)
        .collect();
    let shell = topo.add_shell(Shell::new(kept_faces)?);
    let sharp_solid = topo.add_solid(Solid::new(shell, Vec::new()));
    face_map_indices.remove(&band.faces[0].index());
    let face_map = face_map_indices
        .into_iter()
        .filter_map(|(source, result)| topo.face_id_from_index(result).map(|face| (source, face)))
        .collect();
    Ok(SharpResult {
        solid: sharp_solid,
        edges: vec![sharp_edge],
        face_map,
    })
}

fn validate_exact_result(
    topo: &Topology,
    solid: SolidId,
    label: &str,
) -> Result<(), OperationsError> {
    let report = brepkit_check::validate::validate_solid(
        topo,
        solid,
        &brepkit_check::validate::ValidateOptions::default(),
    )?;
    if report.is_valid() {
        return Ok(());
    }
    let detail = report
        .issues
        .iter()
        .filter(|issue| issue.severity == brepkit_check::validate::Severity::Error)
        .take(3)
        .map(|issue| issue.description.as_str())
        .collect::<Vec<_>>()
        .join("; ");
    Err(reconstruction(format!(
        "{label} failed validation with {} error(s): {detail}",
        report.error_count()
    )))
}

fn validate_volume_progress(
    input: f64,
    sharp: f64,
    result: f64,
    old_radius: f64,
    new_radius: f64,
) -> Result<(), OperationsError> {
    if result <= 0.0 {
        return Err(reconstruction("resized blend encloses no positive volume"));
    }
    let tol = Tolerance::new();
    let old_effect = input - sharp;
    let new_effect = result - sharp;
    if !tol.approx_eq(old_effect, 0.0)
        && !tol.approx_eq(new_effect, 0.0)
        && old_effect.is_sign_positive() != new_effect.is_sign_positive()
    {
        return Err(reconstruction(
            "resized blend moved material to the opposite side of the sharp feature",
        ));
    }
    let old_magnitude = old_effect.abs();
    let new_magnitude = new_effect.abs();
    if new_radius < old_radius
        && new_magnitude > old_magnitude
        && !tol.approx_eq(new_magnitude, old_magnitude)
    {
        return Err(reconstruction(
            "shrinking the radius increased the blend's volume effect",
        ));
    }
    if new_radius > old_radius
        && new_magnitude < old_magnitude
        && !tol.approx_eq(new_magnitude, old_magnitude)
    {
        return Err(reconstruction(
            "growing the radius decreased the blend's volume effect",
        ));
    }
    Ok(())
}

fn heal_evolution(face_map: &HashMap<usize, FaceId>, removed: &[FaceId]) -> EvolutionMap {
    let mut evolution = EvolutionMap::exact();
    for (&source, &result) in face_map {
        evolution.add_modified(source, result.index());
    }
    for face in removed {
        evolution.add_deleted(face.index());
    }
    evolution
}

fn compose_evolution(
    face_map: &HashMap<usize, FaceId>,
    removed: &[FaceId],
    origins: Option<&BlendFaceOrigins>,
) -> EvolutionMap {
    let Some(origins) = origins else {
        return EvolutionMap::new();
    };
    let healed_to_source: HashMap<usize, usize> = face_map
        .iter()
        .map(|(&source, result)| (result.index(), source))
        .collect();
    let mut evolution = EvolutionMap::exact();
    for &(healed, result) in &origins.survived {
        if let Some(source) = healed_to_source.get(&healed.index()) {
            evolution.add_modified(*source, result.index());
        }
    }
    for healed in &origins.deleted {
        if let Some(source) = healed_to_source.get(&healed.index()) {
            evolution.add_deleted(*source);
        }
    }
    for (result, sources) in &origins.created {
        let mapped: Vec<usize> = sources
            .iter()
            .filter_map(|source| healed_to_source.get(&source.index()).copied())
            .collect();
        if mapped.len() != sources.len() || mapped.is_empty() {
            evolution.add_unresolved(result.index(), mapped);
        } else {
            for source in mapped {
                evolution.add_generated(source, result.index());
            }
        }
    }
    for result in &origins.created_unattributed {
        evolution.add_unresolved(result.index(), Vec::new());
    }
    for face in removed {
        evolution.add_deleted(face.index());
    }
    evolution
}

/// Stable failure code for a resize-blend error.
#[must_use]
pub fn resize_blend_failure_code(error: &OperationsError) -> &'static str {
    match error {
        OperationsError::ResizeBlend(error) => error.code(),
        _ => "resize-blend-failed",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn refusal_codes_are_stable() {
        let cases = [
            (
                ResizeBlendError::InvalidInput {
                    reason: String::new(),
                },
                "invalid-input",
            ),
            (
                ResizeBlendError::BandNotAnalytic { surface: "plane" },
                "blend-band-not-analytic",
            ),
            (
                ResizeBlendError::RadiusMismatch {
                    expected: 1.0,
                    actual: 2.0,
                },
                "blend-radius-mismatch",
            ),
            (
                ResizeBlendError::BandTouchesFreeform,
                "band-touches-freeform",
            ),
            (
                ResizeBlendError::UnsupportedSupportPair {
                    first: "cone",
                    second: "cylinder",
                },
                "unsupported-support-pair",
            ),
            (
                ResizeBlendError::RadiusTooLarge { radius: 100.0 },
                "radius-too-large",
            ),
            (
                ResizeBlendError::ReconstructionFailed {
                    reason: String::new(),
                },
                "resize-blend-failed",
            ),
        ];
        for (error, expected) in cases {
            assert_eq!(error.code(), expected);
        }
    }
}
