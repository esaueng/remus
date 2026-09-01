//! Exact constant-radius editing for analytic blend bands.
//!
//! The operation never trusts caller classification. It re-walks the selected
//! face's tangent band, re-measures the radius, identifies the two exact
//! supports, restores their sharp intersection, and only then runs the normal
//! fillet construction at the requested radius. Any ambiguity is a refusal and
//! the topology arena is restored to its pre-call state.

use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};

use remus_blend::BlendResult;
use remus_math::curves::Circle3D;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Vec3;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

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
    #[error("selected face is not an analytic torus/cylinder/sphere blend: {surface}")]
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
    Sphere,
}

/// Tangency-connected analytic faces that carry one constant blend radius.
///
/// A region may contain cylindrical edge bands, toroidal curved-edge bands,
/// and spherical corner patches. Membership is derived from exact shared-edge
/// tangency and radius equality; the seed is never trusted as a classification.
#[derive(Debug, Clone, PartialEq)]
pub struct BlendRegion {
    /// Region faces in deterministic arena-index order.
    pub faces: Vec<FaceId>,
    /// Exact rolling-ball radius shared by every region face.
    pub radius: f64,
}

#[derive(Debug)]
struct BandDescription {
    faces: Vec<FaceId>,
    supports: Vec<FaceId>,
    radius: f64,
}

struct SharpResult {
    solid: SolidId,
    edges: Vec<EdgeId>,
    face_map: HashMap<usize, FaceId>,
}

#[derive(Debug)]
struct SupportPair {
    first: FaceId,
    second: FaceId,
    edge_count: usize,
}

#[derive(Debug)]
struct BlendMovePlan {
    radius: f64,
    volume_effect: f64,
    support_pairs: Vec<SupportPair>,
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
    remus_topology::transaction::run_transacted(topo, |topo| {
        resize_blend_impl(topo, solid, face, expected_radius, new_radius)
    })
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
    if !remus_topology::explorer::solid_faces(topo, solid)?.contains(&face) {
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
    let sharp = remove_blend_region(topo, solid, &band, new_radius)?;

    validate_exact_result(topo, sharp.solid, "sharp support reconstruction")?;
    let sharp_volume = crate::measure::solid_volume(topo, sharp.solid, 0.05)?;
    if tol.approx_eq(new_radius, 0.0) {
        validate_volume_progress(input_volume, sharp_volume, sharp_volume, band.radius, 0.0)?;
        return Ok(ResizeBlendResult {
            solid: sharp.solid,
            evolution: heal_evolution(&sharp.face_map, &band.faces),
        });
    }

    let rebuilt = rebuild_blend_edges(topo, sharp.solid, &sharp.edges, band.radius, new_radius)?;
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

fn remove_blend_region(
    topo: &mut Topology,
    solid: SolidId,
    band: &BandDescription,
    rebuild_radius: f64,
) -> Result<SharpResult, OperationsError> {
    let support_types: Vec<&'static str> = band
        .supports
        .iter()
        .map(|support| topo.face(*support).map(|face| face.surface().type_tag()))
        .collect::<Result<_, _>>()?;
    if support_types.iter().all(|surface| *surface == "plane") {
        return heal_planar_band(topo, solid, band);
    }
    match support_types.as_slice() {
        ["plane", "cylinder"] | ["cylinder", "plane"] => {
            heal_plane_cylinder_band(topo, solid, band)
        }
        ["cylinder", "cone"] | ["cone", "cylinder"] => {
            if !Tolerance::new().approx_eq(rebuild_radius, 0.0) {
                // The sharp circle is exact, but the closed-rim assembler
                // cannot reconstruct this support pair at positive radius.
                return Err(ResizeBlendError::UnsupportedSupportPair {
                    first: support_types[0],
                    second: support_types[1],
                }
                .into());
            }
            heal_cylinder_cone_band(topo, solid, band)
        }
        [first, second] => Err(ResizeBlendError::UnsupportedSupportPair { first, second }.into()),
        _ => Err(reconstruction(format!(
            "blend region has unsupported support surfaces {support_types:?}; expected all planes or one supported analytic pair"
        ))),
    }
}

fn rebuild_blend_edges(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    old_radius: f64,
    new_radius: f64,
) -> Result<BlendResult, OperationsError> {
    match fillet_v2(topo, solid, edges, new_radius) {
        Ok(result) => Ok(result),
        Err(OperationsError::Blend(remus_blend::BlendError::RadiusTooLarge { .. })) => {
            Err(ResizeBlendError::RadiusTooLarge { radius: new_radius }.into())
        }
        Err(OperationsError::Blend(remus_blend::BlendError::TrimmingFailure { .. }))
            if new_radius > old_radius =>
        {
            Err(ResizeBlendError::RadiusTooLarge { radius: new_radius }.into())
        }
        Err(error) => Err(reconstruction(format!(
            "fillet reconstruction at {new_radius} mm failed: {error}"
        ))),
    }
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
        FaceSurface::Sphere(sphere) => Some((BlendKind::Sphere, sphere.radius())),
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

/// Find the complete equal-radius analytic blend region containing `seed`.
///
/// Faces are connected only across exact shared edges where their effective
/// outward normals are tangent. At least two tangent non-region supports must
/// bound the result, which prevents an ordinary cylinder or sphere from being
/// reported as a blend solely because it has a radius.
///
/// # Errors
///
/// Returns a typed resize-blend refusal when the seed is outside `solid`, is
/// not analytic blend geometry, touches freeform support, or cannot be proven
/// to have at least two tangent support faces.
pub fn blend_region(
    topo: &Topology,
    solid: SolidId,
    seed: FaceId,
) -> Result<BlendRegion, OperationsError> {
    if !remus_topology::explorer::solid_faces(topo, solid)?.contains(&seed) {
        return Err(invalid(format!(
            "face {} is not part of solid {}",
            seed.index(),
            solid.index()
        )));
    }
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

    let mut faces: Vec<FaceId> = band.into_iter().collect();
    faces.sort_unstable_by_key(|face| face.index());
    let supports = blend_region_supports(topo, solid, &faces)?;
    if supports.len() < 2 {
        return Err(reconstruction(format!(
            "blend region has {} tangent support faces; at least two are required",
            supports.len()
        )));
    }
    Ok(BlendRegion { faces, radius })
}

fn blend_region_supports(
    topo: &Topology,
    solid: SolidId,
    faces: &[FaceId],
) -> Result<Vec<FaceId>, OperationsError> {
    let adjacency = topo.build_adjacency(solid)?;
    let band: HashSet<FaceId> = faces.iter().copied().collect();
    let mut supports = HashSet::new();
    for &band_face in faces {
        for edge in face_edges(topo, band_face)? {
            let adjacent = distinct_faces(adjacency.faces_for_edge(edge));
            let others: Vec<FaceId> = adjacent
                .into_iter()
                .filter(|candidate| *candidate != band_face && !band.contains(candidate))
                .collect();
            for other in others {
                let surface = topo.face(other)?.surface();
                let tangent = tangent_across(topo, edge, band_face, other)?;
                if tangent && matches!(surface, FaceSurface::Nurbs(_)) {
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
    Ok(supports)
}

fn describe_band(
    topo: &Topology,
    solid: SolidId,
    seed: FaceId,
) -> Result<BandDescription, OperationsError> {
    let region = blend_region(topo, solid, seed)?;
    let supports = blend_region_supports(topo, solid, &region.faces)?;
    Ok(BandDescription {
        faces: region.faces,
        supports,
        radius: region.radius,
    })
}

/// Move planar support faces through their tangent analytic blend neighborhood.
///
/// The primary path temporarily restores every incident sharp edge, moves the
/// sharp support, and rebuilds the same constant-radius blend regions. Imported
/// prismatic regions that reach freeform carrier patches may instead be moved as
/// one rigid, proof-gated patch when every boundary extension stays exact.
///
/// `Ok(None)` means the selection has no tangent analytic blend neighbor and
/// the ordinary planar path should run. Once a blend is recognized, every
/// refusal is returned rather than falling back to an approximation.
pub(crate) fn move_planar_faces_with_blends(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
) -> Result<Option<SolidId>, OperationsError> {
    if adjacent_analytic_blend_seed(topo, solid, faces)?.is_none() {
        return Ok(None);
    }

    let snapshot = topo.clone();
    let remove_rebuild_error =
        match move_planar_faces_with_blends_remove_rebuild(topo, solid, faces, distance) {
            Ok(Some(result)) => return Ok(Some(result)),
            Ok(None) => {
                reconstruction("remove/rebuild could not derive a removable incident blend region")
            }
            Err(error) => error,
        };
    topo.restore_preserving_handle_slots(&snapshot);
    match move_translation_invariant_blend_region(topo, solid, faces, distance) {
        Ok(result) => Ok(Some(result)),
        Err(translation_error) => Err(reconstruction(format!(
            "remove/rebuild failed ({remove_rebuild_error}); exact prismatic blend translation failed ({translation_error})"
        ))),
    }
}

fn move_planar_faces_with_blends_remove_rebuild(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
) -> Result<Option<SolidId>, OperationsError> {
    if faces.is_empty() {
        return Ok(None);
    }
    let source_faces = remus_topology::explorer::solid_faces(topo, solid)?;
    let source_face_set: HashSet<usize> = source_faces.iter().map(|face| face.index()).collect();
    if faces.iter().any(|face| {
        !source_face_set.contains(&face.index())
            || topo
                .face(*face)
                .is_ok_and(|face| !matches!(face.surface(), FaceSurface::Plane { .. }))
    }) {
        return Ok(None);
    }
    if adjacent_blend_seed(topo, solid, faces)?.is_none() {
        return Ok(None);
    }

    let source_counts = remus_topology::explorer::solid_entity_counts(topo, solid)?;
    let source_volume = crate::measure::solid_volume(topo, solid, 0.05)?;
    let mut current_solid = solid;
    let mut selected_faces = faces.to_vec();
    let mut plans = Vec::new();

    while let Some(seed) = adjacent_blend_seed(topo, current_solid, &selected_faces)? {
        let band = describe_band(topo, current_solid, seed)?;
        let stage_volume = crate::measure::solid_volume(topo, current_solid, 0.05)?;
        let sharp = remove_blend_region(topo, current_solid, &band, band.radius)?;
        validate_exact_result(topo, sharp.solid, "sharp support reconstruction")?;
        let sharp_volume = crate::measure::solid_volume(topo, sharp.solid, 0.05)?;
        validate_volume_progress(stage_volume, sharp_volume, sharp_volume, band.radius, 0.0)?;

        selected_faces = remap_faces(&selected_faces, &sharp.face_map, "selected planar support")?;
        for plan in &mut plans {
            remap_plan(plan, &sharp.face_map)?;
        }
        let mapped_supports = remap_faces(&band.supports, &sharp.face_map, "blend support")?;
        plans.push(BlendMovePlan {
            radius: band.radius,
            volume_effect: stage_volume - sharp_volume,
            support_pairs: support_pairs_for_edges(
                topo,
                sharp.solid,
                &sharp.edges,
                &mapped_supports,
            )?,
        });
        current_solid = sharp.solid;
    }

    if !crate::push_pull::move_is_prismatic(topo, current_solid, &selected_faces)? {
        return Err(reconstruction(
            "blend-aware planar move requires a prismatic sharp support neighborhood",
        ));
    }
    let moved_area = selected_faces.iter().try_fold(0.0, |area, &face| {
        crate::measure::face_area(topo, face, 0.01).map(|value| area + value)
    })?;
    let sharp_before = crate::measure::solid_volume(topo, current_solid, 0.05)?;
    let moved =
        remus_offset::move_faces_with_face_map(topo, current_solid, &selected_faces, distance)?;
    let sharp_after = crate::measure::solid_volume(topo, moved.solid, 0.05)?;
    validate_expected_volume(
        sharp_before + moved_area * distance,
        sharp_after,
        "sharp planar move",
    )?;
    for plan in &mut plans {
        remap_plan(plan, &moved.face_map)?;
    }
    current_solid = moved.solid;

    for index in (0..plans.len()).rev() {
        let before_rebuild = crate::measure::solid_volume(topo, current_solid, 0.05)?;
        let edges = resolve_support_pair_edges(topo, current_solid, &plans[index])?;
        let rebuilt = rebuild_blend_edges(
            topo,
            current_solid,
            &edges,
            plans[index].radius,
            plans[index].radius,
        )?;
        validate_exact_result(topo, rebuilt.solid, "moved blend reconstruction")?;
        let rebuilt_volume = crate::measure::solid_volume(topo, rebuilt.solid, 0.05)?;
        validate_expected_volume(
            before_rebuild + plans[index].volume_effect,
            rebuilt_volume,
            "moved blend volume effect",
        )?;
        let origins = rebuilt.face_origins.as_ref().ok_or_else(|| {
            reconstruction("blend reconstruction did not report construction face origins")
        })?;
        let survivor_map: HashMap<usize, FaceId> = origins
            .survived
            .iter()
            .map(|(source, result)| (source.index(), *result))
            .collect();
        for plan in &mut plans[..index] {
            remap_plan(plan, &survivor_map)?;
        }
        current_solid = rebuilt.solid;
    }

    validate_exact_result(topo, current_solid, "blend-aware planar move")?;
    let final_counts = remus_topology::explorer::solid_entity_counts(topo, current_solid)?;
    if final_counts != source_counts {
        return Err(reconstruction(format!(
            "blend-aware planar move changed (faces, edges, vertices) from {source_counts:?} to {final_counts:?}"
        )));
    }
    let final_volume = crate::measure::solid_volume(topo, current_solid, 0.05)?;
    validate_expected_volume(
        source_volume + moved_area * distance,
        final_volume,
        "blend-aware planar move",
    )?;

    Ok(Some(current_solid))
}

fn adjacent_blend_seed(
    topo: &Topology,
    solid: SolidId,
    selected_faces: &[FaceId],
) -> Result<Option<FaceId>, OperationsError> {
    let selected: HashSet<FaceId> = selected_faces.iter().copied().collect();
    let adjacency = topo.build_adjacency(solid)?;
    let mut ordered = selected_faces.to_vec();
    ordered.sort_unstable_by_key(|face| face.index());
    for selected_face in ordered {
        for edge in face_edges(topo, selected_face)? {
            let mut adjacent = distinct_faces(adjacency.faces_for_edge(edge));
            adjacent.sort_unstable_by_key(|face| face.index());
            for candidate in adjacent {
                if selected.contains(&candidate)
                    || blend_surface(topo.face(candidate)?.surface()).is_none()
                {
                    continue;
                }
                if tangent_across(topo, edge, selected_face, candidate)?
                    && describe_band(topo, solid, candidate).is_ok()
                {
                    return Ok(Some(candidate));
                }
            }
        }
    }
    Ok(None)
}

fn adjacent_analytic_blend_seed(
    topo: &Topology,
    solid: SolidId,
    selected_faces: &[FaceId],
) -> Result<Option<FaceId>, OperationsError> {
    let selected: HashSet<FaceId> = selected_faces.iter().copied().collect();
    let adjacency = topo.build_adjacency(solid)?;
    let mut ordered = selected_faces.to_vec();
    ordered.sort_unstable_by_key(|face| face.index());
    for selected_face in ordered {
        for edge in face_edges(topo, selected_face)? {
            let mut adjacent = distinct_faces(adjacency.faces_for_edge(edge));
            adjacent.sort_unstable_by_key(|face| face.index());
            for candidate in adjacent {
                if selected.contains(&candidate)
                    || blend_surface(topo.face(candidate)?.surface()).is_none()
                {
                    continue;
                }
                if tangent_across(topo, edge, selected_face, candidate)?
                    && is_local_blend_face(topo, &adjacency, candidate)?
                {
                    return Ok(Some(candidate));
                }
            }
        }
    }
    Ok(None)
}

fn is_local_blend_face(
    topo: &Topology,
    adjacency: &remus_topology::adjacency::AdjacencyIndex,
    face: FaceId,
) -> Result<bool, OperationsError> {
    if blend_surface(topo.face(face)?.surface()).is_none() {
        return Ok(false);
    }
    let mut tangent_neighbors = HashSet::new();
    for edge in face_edges(topo, face)? {
        for adjacent in distinct_faces(adjacency.faces_for_edge(edge)) {
            if adjacent != face && tangent_across(topo, edge, face, adjacent)? {
                tangent_neighbors.insert(adjacent);
            }
        }
    }
    Ok(tangent_neighbors.len() >= 2)
}

fn surface_translation_invariant(surface: &FaceSurface, direction: Vec3) -> bool {
    let tol = Tolerance::new();
    match surface {
        FaceSurface::Plane { normal, .. } => normal.dot(direction).abs() <= tol.angular,
        FaceSurface::Cylinder(cylinder) => {
            cylinder.axis().dot(direction).abs() >= 1.0 - tol.angular
        }
        FaceSurface::Nurbs(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => false,
    }
}

fn invariant_boundary_carriers(
    topo: &Topology,
    adjacency: &remus_topology::adjacency::AdjacencyIndex,
    moved_faces: &HashSet<FaceId>,
    direction: Vec3,
) -> Result<HashSet<FaceId>, OperationsError> {
    let mut carriers = HashSet::new();
    for &moved_face in moved_faces {
        for edge in face_edges(topo, moved_face)? {
            for adjacent in distinct_faces(adjacency.faces_for_edge(edge)) {
                if !moved_faces.contains(&adjacent)
                    && surface_translation_invariant(topo.face(adjacent)?.surface(), direction)
                {
                    carriers.insert(adjacent);
                }
            }
        }
    }
    Ok(carriers)
}

struct BlendTranslationRegion {
    translated_faces: HashSet<FaceId>,
    translated_nurbs_supports: HashSet<FaceId>,
}

fn translation_invariant_blend_region(
    topo: &Topology,
    solid: SolidId,
    selected_faces: &[FaceId],
    direction: Vec3,
) -> Result<BlendTranslationRegion, OperationsError> {
    let selected: HashSet<FaceId> = selected_faces.iter().copied().collect();
    let adjacency = topo.build_adjacency(solid)?;
    let mut region = HashSet::new();
    let mut queue = VecDeque::new();

    for &selected_face in selected_faces {
        for edge in face_edges(topo, selected_face)? {
            for candidate in distinct_faces(adjacency.faces_for_edge(edge)) {
                if selected.contains(&candidate)
                    || surface_translation_invariant(topo.face(candidate)?.surface(), direction)
                    || !tangent_across(topo, edge, selected_face, candidate)?
                    || !is_local_blend_face(topo, &adjacency, candidate)?
                {
                    continue;
                }
                if region.insert(candidate) {
                    queue.push_back(candidate);
                }
            }
        }
    }

    while let Some(current) = queue.pop_front() {
        for edge in face_edges(topo, current)? {
            for candidate in distinct_faces(adjacency.faces_for_edge(edge)) {
                if candidate == current
                    || selected.contains(&candidate)
                    || region.contains(&candidate)
                {
                    continue;
                }
                if !tangent_across(topo, edge, current, candidate)?
                    || surface_translation_invariant(topo.face(candidate)?.surface(), direction)
                    || !is_local_blend_face(topo, &adjacency, candidate)?
                {
                    continue;
                }
                region.insert(candidate);
                queue.push_back(candidate);
            }
        }
    }

    if region.is_empty() {
        return Err(reconstruction(
            "no translation-dependent analytic blend region bounds the selected plane",
        ));
    }

    let mut translated_nurbs_supports = HashSet::new();
    for &blend_face in &region {
        for edge in face_edges(topo, blend_face)? {
            for adjacent in distinct_faces(adjacency.faces_for_edge(edge)) {
                if adjacent == blend_face
                    || selected.contains(&adjacent)
                    || region.contains(&adjacent)
                {
                    continue;
                }
                let surface = topo.face(adjacent)?.surface();
                if matches!(surface, FaceSurface::Nurbs(_)) {
                    translated_nurbs_supports.insert(adjacent);
                } else if !surface_translation_invariant(surface, direction) {
                    return Err(reconstruction(format!(
                        "blend face {} reaches non-invariant {} support face {} across edge {}",
                        blend_face.index(),
                        topo.face(adjacent)?.surface().type_tag(),
                        adjacent.index(),
                        edge.index()
                    )));
                }
            }
        }
    }

    Ok(BlendTranslationRegion {
        translated_faces: region,
        translated_nurbs_supports,
    })
}

fn translate_face_surface(
    topo: &mut Topology,
    face: FaceId,
    delta: Vec3,
) -> Result<(), OperationsError> {
    let translated = match topo.face(face)?.surface().clone() {
        FaceSurface::Plane { normal, d } => FaceSurface::Plane {
            normal,
            d: normal.dot(delta).mul_add(1.0, d),
        },
        FaceSurface::Cylinder(surface) => FaceSurface::Cylinder(surface.translated(delta)),
        FaceSurface::Cone(surface) => FaceSurface::Cone(surface.translated(delta)),
        FaceSurface::Sphere(surface) => FaceSurface::Sphere(surface.translated(delta)),
        FaceSurface::Torus(surface) => FaceSurface::Torus(surface.translated(delta)),
        FaceSurface::Nurbs(surface) => {
            let control_points = surface
                .control_points()
                .iter()
                .map(|row| row.iter().map(|point| *point + delta).collect())
                .collect();
            FaceSurface::Nurbs(remus_math::nurbs::surface::NurbsSurface::new(
                surface.degree_u(),
                surface.degree_v(),
                surface.knots_u().to_vec(),
                surface.knots_v().to_vec(),
                control_points,
                surface.weights().to_vec(),
            )?)
        }
    };
    topo.face_mut(face)?.set_surface(translated);
    Ok(())
}

fn face_anchor(topo: &Topology, face: FaceId) -> Result<remus_math::vec::Point3, OperationsError> {
    let wire = topo.wire(topo.face(face)?.outer_wire())?;
    let oriented = wire
        .edges()
        .first()
        .ok_or_else(|| reconstruction(format!("face {} has an empty outer wire", face.index())))?;
    let edge = topo.edge(oriented.edge())?;
    Ok(topo.vertex(oriented.oriented_start(edge))?.point())
}

#[allow(clippy::too_many_lines)]
fn move_translation_invariant_blend_region(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
) -> Result<SolidId, OperationsError> {
    if !distance.is_finite() || distance.abs() <= Tolerance::new().linear {
        return Err(remus_offset::OffsetError::InvalidInput {
            reason: "move-face distance must be non-zero and finite".into(),
        }
        .into());
    }
    let Some(&reference) = faces.first() else {
        return Err(remus_offset::OffsetError::InvalidInput {
            reason: "move-face requires at least one selected face".into(),
        }
        .into());
    };
    let source_faces = remus_topology::explorer::solid_faces(topo, solid)?;
    let source_face_set: HashSet<FaceId> = source_faces.iter().copied().collect();
    if faces.iter().any(|face| !source_face_set.contains(face)) {
        return Err(remus_offset::OffsetError::FaceNotInSolid {
            face: faces
                .iter()
                .copied()
                .find(|face| !source_face_set.contains(face))
                .unwrap_or(reference),
            solid,
        }
        .into());
    }
    let (direction, reference_d) = match topo.face(reference)?.surface() {
        FaceSurface::Plane { .. } => {
            let normal = topo
                .face(reference)?
                .effective_plane_normal()
                .ok_or_else(|| reconstruction("selected planar face has no effective normal"))?;
            let anchor = face_anchor(topo, reference)?;
            (
                normal,
                normal.dot(anchor - remus_math::vec::Point3::new(0.0, 0.0, 0.0)),
            )
        }
        surface => {
            return Err(remus_offset::OffsetError::UnsupportedMoveFace {
                face: reference,
                surface_type: surface.type_tag(),
                reason: "blend-aware translation requires planar selected faces".into(),
            }
            .into());
        }
    };
    for &face in faces.iter().skip(1) {
        let FaceSurface::Plane { .. } = topo.face(face)?.surface() else {
            return Err(remus_offset::OffsetError::UnsupportedMoveFace {
                face,
                surface_type: topo.face(face)?.surface().type_tag(),
                reason: "blend-aware translation requires planar selected faces".into(),
            }
            .into());
        };
        let normal = topo
            .face(face)?
            .effective_plane_normal()
            .ok_or_else(|| reconstruction("selected planar face has no effective normal"))?;
        let anchor = face_anchor(topo, face)?;
        let d = normal.dot(anchor - remus_math::vec::Point3::new(0.0, 0.0, 0.0));
        if normal.dot(direction) < 1.0 - Tolerance::new().angular
            || !Tolerance::new().approx_eq(d, reference_d)
        {
            return Err(remus_offset::OffsetError::MoveGroupMismatch {
                reference,
                face,
                reason: "selected faces are not coplanar with the same outward normal".into(),
            }
            .into());
        }
    }

    let region = translation_invariant_blend_region(topo, solid, faces, direction)?;
    let delta = direction * distance;
    let source_counts = remus_topology::explorer::solid_entity_counts(topo, solid)?;
    let source_volume = crate::measure::solid_volume(topo, solid, 0.05)?;
    let mut work = topo.clone();
    let mut moved_faces: HashSet<FaceId> = faces.iter().copied().collect();
    moved_faces.extend(region.translated_faces);
    moved_faces.extend(region.translated_nurbs_supports);
    let adjacency = work.build_adjacency(solid)?;
    let invariant_carriers =
        invariant_boundary_carriers(&work, &adjacency, &moved_faces, direction)?;
    let swept_faces = faces.iter().copied().collect();
    let excluded_candidates = moved_faces.union(&invariant_carriers).copied().collect();
    crate::push_pull::refuse_swept_region_intersections(
        topo,
        solid,
        &swept_faces,
        &excluded_candidates,
        direction,
        distance,
    )?;

    let mut moved_vertices = HashSet::new();
    for &face in &moved_faces {
        moved_vertices.extend(remus_topology::explorer::face_vertices(&work, face)?);
    }
    let mut translated_edges = HashSet::new();
    for edge in remus_topology::explorer::solid_edges(&work, solid)? {
        let edge_data = work.edge(edge)?;
        let start_moves = moved_vertices.contains(&edge_data.start());
        let end_moves = moved_vertices.contains(&edge_data.end());
        if start_moves || end_moves {
            for adjacent in distinct_faces(adjacency.faces_for_edge(edge)) {
                if !moved_faces.contains(&adjacent)
                    && !surface_translation_invariant(work.face(adjacent)?.surface(), direction)
                {
                    return Err(reconstruction(format!(
                        "boundary edge {} reaches non-invariant unmoved face {}",
                        edge.index(),
                        adjacent.index()
                    )));
                }
            }
        }
        match (start_moves, end_moves) {
            (true, true) => {
                translated_edges.insert(edge);
            }
            (true, false) | (false, true) => {
                if !matches!(edge_data.curve(), EdgeCurve::Line) {
                    return Err(reconstruction(format!(
                        "boundary edge {} would change one endpoint of a curved edge",
                        edge.index()
                    )));
                }
                let span =
                    work.vertex(edge_data.end())?.point() - work.vertex(edge_data.start())?.point();
                if span.cross(direction).length() > Tolerance::new().linear {
                    return Err(reconstruction(format!(
                        "boundary edge {} is not parallel to the planar move",
                        edge.index()
                    )));
                }
            }
            (false, false) => {}
        }
    }

    for vertex in moved_vertices {
        let point = work.vertex(vertex)?.point();
        work.vertex_mut(vertex)?.set_point(point + delta);
    }
    let matrix = remus_math::mat::Mat4::translation(delta.x(), delta.y(), delta.z());
    crate::transform::transform_edges(&mut work, &translated_edges, &matrix)?;
    for face in moved_faces {
        translate_face_surface(&mut work, face, delta)?;
    }
    validate_exact_result(&work, solid, "translation-invariant blend move")?;
    let result_counts = remus_topology::explorer::solid_entity_counts(&work, solid)?;
    if result_counts != source_counts {
        return Err(reconstruction(format!(
            "blend translation changed (faces, edges, vertices) from {source_counts:?} to {result_counts:?}"
        )));
    }
    let result_volume = crate::measure::solid_volume(&work, solid, 0.05)?;
    let volume_change = result_volume - source_volume;
    let volume_slack = source_volume.abs().mul_add(1e-9, 1e-7);
    if volume_change.abs() <= volume_slack
        || volume_change.is_sign_positive() != distance.is_sign_positive()
    {
        return Err(reconstruction(format!(
            "blend-aware move changed volume by {volume_change}, inconsistent with distance {distance}"
        )));
    }

    let result = crate::copy::copy_solid_between(&work, topo, solid)?;
    validate_exact_result(topo, result, "accepted blend-aware planar move")?;
    Ok(result)
}

fn remap_faces(
    faces: &[FaceId],
    face_map: &HashMap<usize, FaceId>,
    label: &str,
) -> Result<Vec<FaceId>, OperationsError> {
    faces
        .iter()
        .map(|face| {
            face_map.get(&face.index()).copied().ok_or_else(|| {
                reconstruction(format!(
                    "{label} face {} did not survive exact reconstruction",
                    face.index()
                ))
            })
        })
        .collect()
}

fn remap_plan(
    plan: &mut BlendMovePlan,
    face_map: &HashMap<usize, FaceId>,
) -> Result<(), OperationsError> {
    for pair in &mut plan.support_pairs {
        pair.first = face_map.get(&pair.first.index()).copied().ok_or_else(|| {
            reconstruction(format!(
                "blend support face {} did not survive exact reconstruction",
                pair.first.index()
            ))
        })?;
        pair.second = face_map.get(&pair.second.index()).copied().ok_or_else(|| {
            reconstruction(format!(
                "blend support face {} did not survive exact reconstruction",
                pair.second.index()
            ))
        })?;
    }
    Ok(())
}

fn support_pairs_for_edges(
    topo: &Topology,
    solid: SolidId,
    edges: &[EdgeId],
    supports: &[FaceId],
) -> Result<Vec<SupportPair>, OperationsError> {
    let adjacency = topo.build_adjacency(solid)?;
    let support_set: HashSet<FaceId> = supports.iter().copied().collect();
    let mut pairs: BTreeMap<(usize, usize), (FaceId, FaceId, usize)> = BTreeMap::new();
    for &edge in edges {
        let adjacent = distinct_faces(adjacency.faces_for_edge(edge));
        let [first, second] = adjacent.as_slice() else {
            return Err(reconstruction(format!(
                "sharp blend edge {} has {} adjacent faces; exactly two are required",
                edge.index(),
                adjacent.len()
            )));
        };
        if !support_set.contains(first) || !support_set.contains(second) {
            return Err(reconstruction(format!(
                "sharp blend edge {} is not bounded by the recognized supports",
                edge.index()
            )));
        }
        let (first, second) = if first.index() <= second.index() {
            (*first, *second)
        } else {
            (*second, *first)
        };
        pairs
            .entry((first.index(), second.index()))
            .and_modify(|entry| entry.2 += 1)
            .or_insert((first, second, 1));
    }
    Ok(pairs
        .into_values()
        .map(|(first, second, edge_count)| SupportPair {
            first,
            second,
            edge_count,
        })
        .collect())
}

fn resolve_support_pair_edges(
    topo: &Topology,
    solid: SolidId,
    plan: &BlendMovePlan,
) -> Result<Vec<EdgeId>, OperationsError> {
    let mut edges = Vec::new();
    for pair in &plan.support_pairs {
        let mut shared = shared_edges(topo, solid, pair.first, pair.second)?;
        shared.sort_unstable_by_key(|edge| edge.index());
        shared.dedup();
        if shared.len() != pair.edge_count {
            return Err(reconstruction(format!(
                "moved blend supports {} and {} share {} sharp edges; expected {}",
                pair.first.index(),
                pair.second.index(),
                shared.len(),
                pair.edge_count
            )));
        }
        edges.extend(shared);
    }
    edges.sort_unstable_by_key(|edge| edge.index());
    edges.dedup();
    Ok(edges)
}

fn validate_expected_volume(
    expected: f64,
    actual: f64,
    label: &str,
) -> Result<(), OperationsError> {
    let slack = expected.abs().mul_add(2e-3, 1e-6);
    if (actual - expected).abs() <= slack {
        return Ok(());
    }
    Err(reconstruction(format!(
        "{label} volume is {actual}, expected {expected}"
    )))
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
    let expected_edges = band
        .faces
        .iter()
        .filter(|face| {
            topo.face(**face)
                .ok()
                .and_then(|face| blend_surface(face.surface()))
                .is_some_and(|(kind, _)| kind != BlendKind::Sphere)
        })
        .count();
    let outcome = crate::defeature::defeature_blend_band(topo, solid, &band.faces)
        .map_err(|error| reconstruction(format!("planar support heal failed: {error}")))?;
    let supports: HashSet<FaceId> = band
        .supports
        .iter()
        .map(|support| {
            outcome
                .face_map
                .get(&support.index())
                .copied()
                .ok_or_else(|| {
                    reconstruction(format!(
                        "support face {} was consumed by planar heal",
                        support.index()
                    ))
                })
        })
        .collect::<Result<_, _>>()?;
    let adjacency = topo.build_adjacency(outcome.solid)?;
    let mut edges = Vec::new();
    for edge in remus_topology::explorer::solid_edges(topo, outcome.solid)? {
        let adjacent = distinct_faces(adjacency.faces_for_edge(edge));
        if adjacent.len() == 2 && adjacent.iter().all(|face| supports.contains(face)) {
            edges.push(edge);
        }
    }
    edges.sort_unstable_by_key(|edge| edge.index());
    edges.dedup();
    if edges.len() != expected_edges {
        return Err(reconstruction(format!(
            "healed planar region produced {} sharp edges for {expected_edges} edge-band faces",
            edges.len()
        )));
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
    let face_data = topo.face(face)?;
    if face_data.outer_wire() == old_wire {
        let inner = face_data.inner_wires().to_vec();
        topo.set_face_boundary_wires(face, new_wire, inner)?;
        return Ok(());
    }
    let Some(slot) = face_data
        .inner_wires()
        .iter()
        .position(|candidate| *candidate == old_wire)
    else {
        return Err(reconstruction(format!(
            "face {} lost wire {}",
            face_data.outer_wire().index(),
            old_wire.index()
        )));
    };
    let outer = face_data.outer_wire();
    let mut inner = face_data.inner_wires().to_vec();
    inner[slot] = new_wire;
    topo.set_face_boundary_wires(face, outer, inner)?;
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

/// Add the exact full-circle edge recovered where two periodic supports meet.
///
/// The explicit reference direction places the seam vertex at parameter zero,
/// so one positive turn is the authoritative traversal.  Certify the two seam
/// evaluations and the antipodal midpoint before the edge enters topology.
fn add_certified_closed_circle_edge(
    topo: &mut Topology,
    seam_vertex: remus_topology::vertex::VertexId,
    circle: Circle3D,
) -> Result<EdgeId, OperationsError> {
    let vertex = topo.vertex(seam_vertex)?;
    let seam = vertex.point();
    let vertex_tolerance = vertex.tolerance();
    if !vertex_tolerance.is_finite() || vertex_tolerance < 0.0 {
        return Err(reconstruction(format!(
            "sharp circle seam has invalid tolerance {vertex_tolerance}"
        )));
    }
    let tolerance = vertex_tolerance.max(Tolerance::new().linear);

    let range = (0.0, std::f64::consts::TAU);
    let antipode = circle.center() - circle.u_axis() * circle.radius();
    for (label, parameter, expected) in [
        ("start seam", range.0, seam),
        ("antipodal midpoint", std::f64::consts::PI, antipode),
        ("end seam", range.1, seam),
    ] {
        let residual = (circle.evaluate(parameter) - expected).length();
        if !residual.is_finite() || residual > tolerance {
            return Err(reconstruction(format!(
                "sharp circle {label} misses its exact oracle by {residual} mm \
                 (tolerance {tolerance} mm)"
            )));
        }
    }

    let mut edge = Edge::with_tolerance(
        seam_vertex,
        seam_vertex,
        EdgeCurve::Circle(circle),
        Some(tolerance),
    );
    edge.set_trim(Some(range));
    edge.strict_domain().map_err(|error| {
        reconstruction(format!(
            "sharp circle does not have an exportable full-turn domain: {error}"
        ))
    })?;
    Ok(topo.add_edge(edge))
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
    let sharp_edge = add_certified_closed_circle_edge(topo, sharp_vertex, circle)?;

    let plane_oriented = oriented_replacement(topo, plane_wire[0], sharp_edge, circle_normal)?;
    let plane_new_wire = topo.add_wire(Wire::new(vec![plane_oriented], true)?);
    replace_face_wire(topo, plane, plane_wire_id, plane_new_wire)?;

    let mut seam_candidates: Vec<EdgeId> = cylinder_wire
        .iter()
        .map(OrientedEdge::edge)
        .filter(|edge| *edge != *cylinder_contact)
        .filter(|edge| {
            matches!(
                topo.edge(*edge).map(remus_topology::edge::Edge::curve),
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
                topo.edge(*edge).map(remus_topology::edge::Edge::curve),
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
) -> Result<Vec<(Vec3, remus_topology::vertex::VertexId)>, OperationsError> {
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
        remus_topology::vertex::VertexId,
        remus_topology::vertex::VertexId,
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
    start_vertex: remus_topology::vertex::VertexId,
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
    sharp_vertex: remus_topology::vertex::VertexId,
    far_vertex: remus_topology::vertex::VertexId,
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
    let sharp_edge = add_certified_closed_circle_edge(topo, sharp_vertex, circle)?;
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
    let report = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )?;
    if report.is_valid() {
        return Ok(());
    }
    let detail = report
        .issues
        .iter()
        .filter(|issue| issue.severity == remus_check::validate::Severity::Error)
        .take(3)
        .map(|issue| format!("{} ({:?})", issue.description, issue.entity))
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
    #![allow(clippy::panic, clippy::unwrap_used)]

    use super::*;

    #[test]
    fn recovered_closed_circle_has_certified_full_turn_authority() {
        let mut topo = Topology::new();
        let center = remus_math::vec::Point3::new(1.0e6, -2.0e6, 3.0e6);
        let circle = Circle3D::new_with_ref(
            center,
            Vec3::new(0.0, 0.0, 1.0),
            25.0,
            Vec3::new(0.6, 0.8, 0.0),
        )
        .unwrap();
        let seam = circle.evaluate(0.0);
        let seam_vertex = topo.add_vertex(Vertex::new(seam, Tolerance::new().linear));

        let edge_id = add_certified_closed_circle_edge(&mut topo, seam_vertex, circle).unwrap();
        let edge = topo.edge(edge_id).unwrap();
        let range = edge.strict_domain().unwrap();
        assert_eq!(range.0.to_bits(), 0.0_f64.to_bits());
        assert_eq!(range.1.to_bits(), std::f64::consts::TAU.to_bits());

        let EdgeCurve::Circle(circle) = edge.curve() else {
            panic!("expected exact circle");
        };
        let antipode = circle.center() - circle.u_axis() * circle.radius();
        for (parameter, expected) in [
            (range.0, seam),
            (std::f64::consts::PI, antipode),
            (range.1, seam),
        ] {
            assert!(
                (circle.evaluate(parameter) - expected).length() <= Tolerance::new().linear,
                "parameter {parameter} missed its closed-form circle oracle"
            );
        }
    }

    #[test]
    fn sweep_exemption_only_includes_actual_boundary_carriers() {
        let mut topo = Topology::new();
        let outer = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let cavity = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let outer_shell = topo.solid(outer).unwrap().outer_shell();
        let cavity_shell = topo.solid(cavity).unwrap().outer_shell();
        let hollow = topo.add_solid(remus_topology::solid::Solid::new(
            outer_shell,
            vec![cavity_shell],
        ));
        let direction = Vec3::new(0.0, 0.0, 1.0);
        let selected = remus_topology::explorer::solid_faces(&topo, outer)
            .unwrap()
            .into_iter()
            .find(|face| {
                topo.face(*face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|normal| normal.dot(direction) > 1.0 - Tolerance::new().angular)
            })
            .unwrap();
        let moved_faces = HashSet::from([selected]);
        let adjacency = topo.build_adjacency(hollow).unwrap();

        let carriers =
            invariant_boundary_carriers(&topo, &adjacency, &moved_faces, direction).unwrap();
        let cavity_faces: HashSet<_> = remus_topology::explorer::solid_faces(&topo, cavity)
            .unwrap()
            .into_iter()
            .collect();

        assert_eq!(carriers.len(), 4);
        assert!(carriers.is_disjoint(&cavity_faces));
    }

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
