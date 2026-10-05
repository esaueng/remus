//! Exact constant-radius editing for analytic blend bands.
//!
//! The operation never trusts caller classification. It re-walks the selected
//! face's tangent band, re-measures the radius, identifies the two exact
//! supports, restores their sharp intersection, and only then runs the normal
//! fillet construction at the requested radius. Any ambiguity is a refusal and
//! the topology arena is restored to its pre-call state.

use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};

use remus_blend::BlendResult;
use remus_blend::fillet_builder::FilletBuilder;
use remus_math::curves::Circle3D;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::journal::EntityKey;
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use crate::OperationsError;
use crate::blend_ops::{BlendFaceOrigins, fillet_v2};
use crate::dot_normal_point;
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
    boundary_history: Option<Vec<(EntityKey, Option<EntityKey>)>>,
}

#[derive(Debug)]
struct SupportPair {
    first: FaceId,
    second: FaceId,
    edge_count: usize,
}

#[derive(Debug)]
struct BandFaceLineage {
    source: usize,
    supports: Vec<FaceId>,
}

#[derive(Debug)]
struct BlendMovePlan {
    radius: f64,
    volume_effect: f64,
    support_pairs: Vec<SupportPair>,
    band_faces: Vec<BandFaceLineage>,
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

type BlendBoundaryHistory = Vec<(EntityKey, Option<EntityKey>)>;

pub(crate) fn resize_blend_with_entity_evolution(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    expected_radius: f64,
    new_radius: f64,
) -> Result<(ResizeBlendResult, BlendBoundaryHistory), OperationsError> {
    let band = describe_band(topo, solid, face)?;
    if band.faces != [face]
        || !matches!(topo.face(face)?.surface(), FaceSurface::Cylinder(_))
        || band.supports.len() != 2
        || band.supports.iter().any(|&support| {
            !topo
                .face(support)
                .is_ok_and(|data| data.surface().is_planar())
        })
        || !new_radius.is_finite()
        || new_radius < 0.0
        || (new_radius > 0.0 && new_radius <= Tolerance::new().linear)
    {
        return Err(reconstruction(
            "journaled resize requires one cylindrical band, two planar supports and a zero or supported positive radius",
        ));
    }
    if new_radius <= 0.0 {
        return remove_blend_with_entity_evolution(topo, solid, face, expected_radius, &band);
    }
    let mut result = resize_blend(topo, solid, face, expected_radius, new_radius)?;
    let mut map = result.evolution.clone();
    if map.deleted.contains(&face.index()) {
        // The rebuilt band's generating supports identify its predecessor only
        // when one output has exactly this band's two construction supports.
        let mut origins = BTreeMap::<usize, Vec<usize>>::new();
        for (&source, outputs) in &map.generated {
            for &output in outputs {
                origins.entry(output).or_default().push(source);
            }
        }
        let supports = canonical_faces(&band.supports);
        let candidates: Vec<_> = origins
            .iter()
            .filter_map(|(&output, sources)| {
                let mut sources = sources.clone();
                sources.sort_unstable();
                sources.dedup();
                (sources == supports).then_some(output)
            })
            .collect();
        let [target] = candidates.as_slice() else {
            return Err(reconstruction(
                "resized band has no unique construction successor",
            ));
        };
        map.generated
            .values_mut()
            .for_each(|outputs| outputs.retain(|output| output != target));
        map.generated.retain(|_, outputs| !outputs.is_empty());
        map.deleted.remove(&face.index());
        map.add_modified(face.index(), *target);
    }
    if !map.origin.is_exact()
        || !map.is_complete()
        || !map.generated.is_empty()
        || !map.deleted.is_empty()
    {
        return Err(reconstruction(
            "resized band does not have total one-to-one face history",
        ));
    }
    let face_map = map
        .modified
        .iter()
        .map(|(&source, outputs)| {
            let [output] = outputs.as_slice() else {
                return None;
            };
            topo.face_id_from_index(*output)
                .map(|target| (source, target))
        })
        .collect::<Option<BTreeMap<_, _>>>()
        .ok_or_else(|| reconstruction("resized band has ambiguous face history"))?;
    let pairs = construction_boundary_pairs(topo, solid, result.solid, &face_map)?;
    if pairs.is_empty() {
        return Err(reconstruction(
            "resized band has ambiguous boundary history",
        ));
    }
    result.evolution = map;
    Ok((
        result,
        pairs
            .into_iter()
            .map(|(source, target)| (source, Some(target)))
            .collect(),
    ))
}

fn remove_blend_with_entity_evolution(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    expected_radius: f64,
    band: &BandDescription,
) -> Result<(ResizeBlendResult, BlendBoundaryHistory), OperationsError> {
    if !expected_radius.is_finite() || expected_radius <= 0.0 {
        return Err(invalid("expected_radius must be finite and positive"));
    }
    if !remus_topology::explorer::solid_faces(topo, solid)?.contains(&face) {
        return Err(invalid("blend face is not part of the input solid"));
    }
    if !Tolerance::new().approx_eq(expected_radius, band.radius) {
        return Err(ResizeBlendError::RadiusMismatch {
            expected: expected_radius,
            actual: band.radius,
        }
        .into());
    }
    let input_volume = crate::measure::solid_volume(topo, solid, 0.05)?;
    let sharp = remove_blend_region(topo, solid, band)?;
    validate_exact_result(topo, sharp.solid, "journaled sharp support reconstruction")?;
    let sharp_volume = crate::measure::solid_volume(topo, sharp.solid, 0.05)?;
    validate_volume_progress(input_volume, sharp_volume, sharp_volume, band.radius, 0.0)?;
    let history = sharp
        .boundary_history
        .ok_or_else(|| reconstruction("blend removal has no construction boundary history"))?;
    let source_faces: HashSet<_> = remus_topology::explorer::solid_faces(topo, solid)?
        .into_iter()
        .filter(|source| *source != face)
        .map(remus_topology::arena::Id::index)
        .collect();
    let target_faces: HashSet<_> = remus_topology::explorer::solid_faces(topo, sharp.solid)?
        .into_iter()
        .collect();
    if sharp.face_map.keys().copied().collect::<HashSet<_>>() != source_faces
        || sharp.face_map.values().copied().collect::<HashSet<_>>() != target_faces
        || sharp.face_map.len() != target_faces.len()
    {
        return Err(reconstruction(
            "blend removal has incomplete surviving face history",
        ));
    }
    let boundaries = |solid| -> Result<HashSet<EntityKey>, OperationsError> {
        Ok(crate::journal_ops::solid_entity_keys(topo, solid)?
            .into_iter()
            .filter(|key| key.kind != remus_topology::journal::EntityKind::Face)
            .collect())
    };
    let sources: HashSet<_> = history.iter().map(|(source, _)| *source).collect();
    let targets: HashSet<_> = history.iter().filter_map(|(_, target)| *target).collect();
    // Partial construction records must not silently sever a surviving boundary.
    // A split boundary has multiple distinct live targets. It must not also
    // be marked deleted, and duplicate source/target records are invalid.
    let records: HashSet<_> = history.iter().copied().collect();
    let deleted_sources: HashSet<_> = history
        .iter()
        .filter_map(|(source, target)| target.is_none().then_some(*source))
        .collect();
    if records.len() != history.len()
        || history
            .iter()
            .any(|(source, target)| target.is_some() && deleted_sources.contains(source))
        || sources != boundaries(solid)?
        || targets != boundaries(sharp.solid)?
        || history
            .iter()
            .any(|(source, target)| target.is_some_and(|target| target.kind != source.kind))
    {
        return Err(reconstruction(
            "blend removal has incomplete or ambiguous boundary history",
        ));
    }
    Ok((
        ResizeBlendResult {
            solid: sharp.solid,
            evolution: heal_evolution(&sharp.face_map, &band.faces),
        },
        history,
    ))
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
    let external_support_side = reconstructs_external_cylinder_cone(topo, &band)?;
    let sharp = remove_blend_region(topo, solid, &band)?;

    validate_exact_result(topo, sharp.solid, "sharp support reconstruction")?;
    let sharp_volume = crate::measure::solid_volume(topo, sharp.solid, 0.05)?;
    if tol.approx_eq(new_radius, 0.0) {
        validate_volume_progress(input_volume, sharp_volume, sharp_volume, band.radius, 0.0)?;
        return Ok(ResizeBlendResult {
            solid: sharp.solid,
            evolution: heal_evolution(&sharp.face_map, &band.faces),
        });
    }

    let rebuilt = rebuild_blend_edges(
        topo,
        sharp.solid,
        &sharp.edges,
        band.radius,
        new_radius,
        external_support_side,
    )?;
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

pub(crate) fn defeature_curved_band(
    topo: &mut Topology,
    solid: SolidId,
    selected: &[FaceId],
) -> Result<Option<crate::defeature::DefeatureOutcome>, OperationsError> {
    let Some(&seed) = selected.first() else {
        return Ok(None);
    };
    if matches!(topo.face(seed)?.surface(), FaceSurface::Cylinder(_)) {
        // Plane-to-plane cylindrical bands heal through the surgical path so
        // unrelated curved geometry survives. Anything outside the surgical
        // scope (multi-face bands, non-planar supports, bores and other plain
        // cylinders) declines here and keeps the established defeature logic.
        let band = match describe_band(topo, solid, seed) {
            Ok(band) => band,
            Err(_) => return Ok(None),
        };
        if band.faces.len() != 1
            || band.supports.len() != 2
            || band.supports.iter().any(|support| {
                topo.face(*support)
                    .is_ok_and(|face| !face.surface().is_planar())
            })
        {
            return Ok(None);
        }
        let selection: std::collections::BTreeSet<_> = selected.iter().copied().collect();
        let members: std::collections::BTreeSet<_> = band.faces.iter().copied().collect();
        if selection != members {
            return Err(reconstruction(
                "delete-face selection must contain exactly the complete analytic blend band",
            ));
        }
        let Some(sharp) = heal_cylinder_plane_band_surgical(topo, solid, &band)? else {
            return Ok(None);
        };
        validate_exact_result(topo, sharp.solid, "deleted planar blend reconstruction")?;
        return Ok(Some(sharp));
    }
    if !matches!(topo.face(seed)?.surface(), FaceSurface::Torus(_)) {
        return Ok(None);
    }
    let band = describe_band(topo, solid, seed)?;
    let selection: std::collections::BTreeSet<_> = selected.iter().copied().collect();
    let members: std::collections::BTreeSet<_> = band.faces.iter().copied().collect();
    if selection != members {
        return Err(reconstruction(
            "delete-face selection must contain exactly the complete analytic blend band",
        ));
    }
    let sharp = remove_blend_region(topo, solid, &band)?;
    validate_exact_result(topo, sharp.solid, "deleted curved blend reconstruction")?;
    Ok(Some(crate::defeature::DefeatureOutcome {
        solid: sharp.solid,
        face_map: sharp.face_map,
        boundary_history: sharp.boundary_history,
    }))
}

fn remove_blend_region(
    topo: &mut Topology,
    solid: SolidId,
    band: &BandDescription,
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
        ["cylinder", "cone"] | ["cone", "cylinder"] => heal_cylinder_cone_band(topo, solid, band),
        [first, second] => Err(ResizeBlendError::UnsupportedSupportPair { first, second }.into()),
        // Zero, one, or three-or-more supports cannot be a two-support blend
        // reconstruction. These arms match on the string dispatch (not on
        // `EdgeCurve`/`FaceSurface`), so they carry no wildcard-arm audit
        // obligation; they are spelled out so a support-count change is
        // explicit rather than a reworded catch-all.
        [] | [_] | [_, _, _, ..] => Err(reconstruction(format!(
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
    external_support_side: bool,
) -> Result<BlendResult, OperationsError> {
    if external_support_side {
        let mut builder = FilletBuilder::new(topo, solid);
        builder.use_external_support_side();
        builder.add_edges(edges, new_radius);
        let result = builder.build()?;
        if result.is_partial {
            return Err(OperationsError::PartialResult {
                operation: "resize blend",
                succeeded: result.succeeded.len(),
                failed: result.failed.len(),
            });
        }
        return Ok(result);
    }
    match fillet_v2(topo, solid, edges, new_radius) {
        Ok(result) => Ok(result),
        Err(OperationsError::Blend(
            remus_blend::BlendError::RadiusTooLarge { .. }
            | remus_blend::BlendError::CliffEncountered { .. },
        )) => Err(ResizeBlendError::RadiusTooLarge { radius: new_radius }.into()),
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

fn reconstructs_external_cylinder_cone(
    topo: &Topology,
    band: &BandDescription,
) -> Result<bool, OperationsError> {
    if band.faces.len() != 1 || band.supports.len() != 2 {
        return Ok(false);
    }
    let cylinder_radius = band.supports.iter().find_map(|support| {
        let face = topo.face(*support).ok()?;
        match face.surface() {
            FaceSurface::Cylinder(cylinder) => Some(cylinder.radius()),
            // Only a cylinder support carries the radius this probe compares
            // against the torus major radius. Every other carrier declines
            // explicitly so a future variant cannot silently inherit `None`.
            FaceSurface::Plane { .. }
            | FaceSurface::Nurbs(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => None,
        }
    });
    let has_cone = band.supports.iter().any(|support| {
        topo.face(*support)
            .is_ok_and(|face| matches!(face.surface(), FaceSurface::Cone(_)))
    });
    let torus_major_radius = match topo.face(band.faces[0])?.surface() {
        FaceSurface::Torus(torus) => Some(torus.major_radius()),
        // The external-branch probe only applies to a torus band; any other
        // carrier (including a future variant) declines explicitly.
        FaceSurface::Plane { .. }
        | FaceSurface::Nurbs(_)
        | FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_) => None,
    };
    let (Some(cylinder), true, Some(major)) = (cylinder_radius, has_cone, torus_major_radius)
    else {
        return Ok(false);
    };
    let tol = Tolerance::new();
    if tol.approx_eq(major, cylinder + band.radius) {
        return Ok(true);
    }
    if cylinder > band.radius && tol.approx_eq(major, cylinder - band.radius) {
        return Ok(false);
    }
    Err(reconstruction(format!(
        "cylinder/cone torus major radius {major} does not prove either reconstruction branch around cylinder radius {cylinder} and blend radius {}",
        band.radius
    )))
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
        // Only constant-radius rolling-ball carriers are blend geometry.
        // Planes, cones, and NURBS never carry a blend radius, so they
        // decline explicitly; a future variant is a compile error here,
        // not a silent `None`.
        FaceSurface::Plane { .. } | FaceSurface::Cone(_) | FaceSurface::Nurbs(_) => None,
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

pub(crate) struct BlendMoveEntities {
    pub solid: SolidId,
    pub evolution: EvolutionMap,
    pub boundary_pairs: Vec<(EntityKey, EntityKey)>,
}

/// Exact construction data for a caller-proven explicit blend-face union.
///
/// This stays crate-private so higher-level removal can journal one atomic
/// reconstruction without exposing the internal band description publicly.
pub(crate) struct RecognizedBlendRemoval {
    pub solid: SolidId,
    pub face_map: HashMap<usize, FaceId>,
    pub boundary_history: BlendBoundaryHistory,
}

/// Remove an explicit union of already-recognized analytic blend faces in one
/// wound reconstruction.
///
/// The explicit union must have one common radius. Mixed-radius group healing
/// remains unqualified even when every support is planar: the planar healer's
/// unused radius witness is not evidence that such a wound reconstructs with
/// exact carriers and total construction history. The caller owns the outer
/// transaction.
pub(crate) fn remove_recognized_blend_faces(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
) -> Result<RecognizedBlendRemoval, OperationsError> {
    if faces.is_empty() {
        return Err(invalid(
            "recognized blend removal requires at least one face",
        ));
    }
    let solid_faces: HashSet<FaceId> = remus_topology::explorer::solid_faces(topo, solid)?
        .into_iter()
        .collect();
    let mut explicit = faces.to_vec();
    explicit.sort_unstable_by_key(|face| face.index());
    explicit.dedup();
    if explicit.iter().any(|face| !solid_faces.contains(face)) {
        return Err(invalid(
            "recognized blend removal contains a face outside the input solid",
        ));
    }
    let mut radii = Vec::with_capacity(explicit.len());
    for &face in &explicit {
        let Some((_, radius)) = blend_surface(topo.face(face)?.surface()) else {
            return Err(reconstruction(format!(
                "face {} is not constant-radius analytic blend geometry",
                face.index()
            )));
        };
        radii.push(radius);
    }
    let supports = blend_region_supports(topo, solid, &explicit)?;
    if supports.len() < 2 {
        return Err(reconstruction(format!(
            "recognized blend union has {} tangent supports; at least two are required",
            supports.len()
        )));
    }
    let tol = Tolerance::new();
    if radii.iter().any(|radius| !tol.approx_eq(*radius, radii[0])) {
        return Err(reconstruction(
            "mixed-radius recognized blend groups are not qualified".to_string(),
        ));
    }
    let band = BandDescription {
        faces: explicit,
        supports,
        // Every selected carrier was proven common-radius immediately above.
        radius: radii[0],
    };
    let sharp = remove_blend_region(topo, solid, &band)?;
    validate_exact_result(topo, sharp.solid, "recognized blend union removal")?;
    let mut boundary_history = sharp.boundary_history.ok_or_else(|| {
        reconstruction("recognized blend union has no construction boundary history")
    })?;
    boundary_history.sort_unstable();
    boundary_history.dedup();
    Ok(RecognizedBlendRemoval {
        solid: sharp.solid,
        face_map: sharp.face_map,
        boundary_history,
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
) -> Result<Option<BlendMoveEntities>, OperationsError> {
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
) -> Result<Option<BlendMoveEntities>, OperationsError> {
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
    // Whole-body mesh volume of `current_solid`, read at most once per stage.
    // Every stage below allocates its result as a new solid and leaves its
    // input in the arena, so one reading serves as a stage's "after" and the
    // next stage's "before". On NURBS-heavy imports one such mesh costs more
    // than the whole reconstruction, and the band removal is what refuses on
    // the bodies this path cannot serve, so nothing is meshed before a band
    // has actually come off.
    let mut current_volume: Option<f64> = None;
    let mut source_volume: Option<f64> = None;
    let mut current_solid = solid;
    let mut selected_faces = faces.to_vec();
    let mut plans = Vec::new();
    let mut construction_map: HashMap<usize, FaceId> = source_faces
        .iter()
        .map(|face| (face.index(), *face))
        .collect();
    let mut lineage_is_exact = true;

    while let Some(seed) = adjacent_blend_seed(topo, current_solid, &selected_faces)? {
        let band = describe_band(topo, current_solid, seed)?;
        let (band_faces, exact) =
            describe_band_face_lineage(topo, current_solid, &band, &construction_map)?;
        lineage_is_exact &= exact;
        let sharp = remove_blend_region(topo, current_solid, &band)?;
        validate_exact_result(topo, sharp.solid, "sharp support reconstruction")?;
        let stage_volume = stage_volume_of(topo, current_solid, current_volume)?;
        source_volume.get_or_insert(stage_volume);
        let sharp_volume = crate::measure::solid_volume(topo, sharp.solid, 0.05)?;
        current_volume = Some(sharp_volume);
        validate_volume_progress(stage_volume, sharp_volume, sharp_volume, band.radius, 0.0)?;

        selected_faces = remap_faces(&selected_faces, &sharp.face_map, "selected planar support")?;
        for plan in &mut plans {
            remap_plan(plan, &sharp.face_map)?;
        }
        remap_construction_lineage(&mut construction_map, &sharp.face_map, &band.faces)?;
        let mapped_supports = remap_faces(&band.supports, &sharp.face_map, "blend support")?;
        let band_faces = remap_band_face_lineage(band_faces, &sharp.face_map)?;
        plans.push(BlendMovePlan {
            radius: band.radius,
            volume_effect: stage_volume - sharp_volume,
            support_pairs: support_pairs_for_edges(
                topo,
                sharp.solid,
                &sharp.edges,
                &mapped_supports,
            )?,
            band_faces,
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
    let sharp_before = stage_volume_of(topo, current_solid, current_volume)?;
    let source_volume = source_volume.unwrap_or(sharp_before);
    let moved =
        remus_offset::move_faces_with_face_map(topo, current_solid, &selected_faces, distance)?;
    let sharp_after = crate::measure::solid_volume(topo, moved.solid, 0.05)?;
    current_volume = Some(sharp_after);
    validate_expected_volume(
        sharp_before + moved_area * distance,
        sharp_after,
        "sharp planar move",
    )?;
    for plan in &mut plans {
        remap_plan(plan, &moved.face_map)?;
    }
    remap_construction_lineage(&mut construction_map, &moved.face_map, &[])?;
    current_solid = moved.solid;

    for index in (0..plans.len()).rev() {
        let before_rebuild = stage_volume_of(topo, current_solid, current_volume)?;
        let edges = resolve_support_pair_edges(topo, current_solid, &plans[index])?;
        let rebuilt = rebuild_blend_edges(
            topo,
            current_solid,
            &edges,
            plans[index].radius,
            plans[index].radius,
            false,
        )?;
        validate_exact_result(topo, rebuilt.solid, "moved blend reconstruction")?;
        let rebuilt_volume = crate::measure::solid_volume(topo, rebuilt.solid, 0.05)?;
        current_volume = Some(rebuilt_volume);
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
        remap_construction_lineage(&mut construction_map, &survivor_map, &[])?;
        lineage_is_exact &=
            restore_band_face_lineage(&mut construction_map, &plans[index].band_faces, origins);
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
    let final_volume = stage_volume_of(topo, current_solid, current_volume)?;
    validate_expected_volume(
        source_volume + moved_area * distance,
        final_volume,
        "blend-aware planar move",
    )?;

    let result_faces = remus_topology::explorer::solid_faces(topo, current_solid)?;
    let boundary_pairs =
        if lineage_is_exact && lineage_is_total(&source_faces, &result_faces, &construction_map) {
            construction_boundary_pairs(
                topo,
                solid,
                current_solid,
                &construction_map
                    .iter()
                    .map(|(&source, &target)| (source, target))
                    .collect(),
            )?
        } else {
            Vec::new()
        };
    let evolution =
        if lineage_is_exact && lineage_is_total(&source_faces, &result_faces, &construction_map) {
            crate::push_pull::exact_face_evolution(
                topo,
                &source_faces,
                current_solid,
                construction_map,
            )?
        } else {
            conservative_move_evolution(&source_faces, &result_faces, &construction_map)
        };

    // Every stage volume above is a `solid_volume` reading, memoized as
    // measured when the application enabled the memo; the final one is the
    // result's, so there is nothing to derive.
    Ok(Some(BlendMoveEntities {
        solid: current_solid,
        evolution,
        boundary_pairs,
    }))
}

// Rebuilt boundaries are attributable only when the construction face map
// determines one complete incidence isomorphism, including outer/hole loops.
// Ambiguous seams retain faces-only history instead of guessed identities.
#[allow(clippy::too_many_lines)]
pub(crate) fn construction_boundary_pairs(
    topo: &Topology,
    source: SolidId,
    result: SolidId,
    face_map: &BTreeMap<usize, FaceId>,
) -> Result<Vec<(EntityKey, EntityKey)>, OperationsError> {
    use remus_topology::explorer::{edge_to_face_map, solid_edges, solid_faces, solid_vertices};
    let source_faces: HashSet<_> = solid_faces(topo, source)?
        .iter()
        .map(|face| face.index())
        .collect();
    let result_faces: HashSet<_> = solid_faces(topo, result)?.into_iter().collect();
    if face_map.keys().copied().collect::<HashSet<_>>() != source_faces
        || face_map.values().copied().collect::<HashSet<_>>() != result_faces
        || face_map.len() != result_faces.len()
    {
        return Ok(Vec::new());
    }
    let old_edges = edge_to_face_map(topo, source)?;
    let new_edges = edge_to_face_map(topo, result)?;
    let mut groups = BTreeMap::<Vec<usize>, Vec<usize>>::new();
    for (edge, faces) in &new_edges {
        let mut key: Vec<_> = faces.iter().map(|face| face.index()).collect();
        key.sort_unstable();
        key.dedup();
        groups.entry(key).or_default().push(*edge);
    }
    let mut edge_map = HashMap::new();
    let mut used_edges = HashSet::new();
    for (edge, faces) in &old_edges {
        let Some(mut key) = faces
            .iter()
            .map(|face| face_map.get(&face.index()).map(|id| id.index()))
            .collect::<Option<Vec<_>>>()
        else {
            return Ok(Vec::new());
        };
        key.sort_unstable();
        key.dedup();
        let Some(candidates) = groups.get(&key) else {
            return Ok(Vec::new());
        };
        let [target] = candidates.as_slice() else {
            return Ok(Vec::new());
        };
        if !used_edges.insert(*target) {
            return Ok(Vec::new());
        }
        edge_map.insert(*edge, *target);
    }
    if used_edges.len() != new_edges.len() {
        return Ok(Vec::new());
    }

    let mut target_vertices = BTreeMap::<usize, Vec<usize>>::new();
    for edge in solid_edges(topo, result)? {
        let data = topo.edge(edge)?;
        for vertex in [data.start(), data.end()] {
            target_vertices
                .entry(vertex.index())
                .or_default()
                .push(edge.index());
        }
    }
    let mut vertex_groups = BTreeMap::<Vec<usize>, Vec<usize>>::new();
    for (vertex, mut edges) in target_vertices {
        edges.sort_unstable();
        vertex_groups.entry(edges).or_default().push(vertex);
    }
    let mut source_vertices = BTreeMap::<usize, Vec<usize>>::new();
    for edge in solid_edges(topo, source)? {
        let Some(&target) = edge_map.get(&edge.index()) else {
            return Ok(Vec::new());
        };
        let data = topo.edge(edge)?;
        for vertex in [data.start(), data.end()] {
            source_vertices
                .entry(vertex.index())
                .or_default()
                .push(target);
        }
    }
    let mut vertex_map = HashMap::new();
    let mut used_vertices = HashSet::new();
    for (vertex, mut edges) in source_vertices {
        edges.sort_unstable();
        let Some(candidates) = vertex_groups.get(&edges) else {
            return Ok(Vec::new());
        };
        let [target] = candidates.as_slice() else {
            return Ok(Vec::new());
        };
        if !used_vertices.insert(*target) {
            return Ok(Vec::new());
        }
        vertex_map.insert(vertex, *target);
    }
    if used_vertices.len() != solid_vertices(topo, result)?.len() {
        return Ok(Vec::new());
    }
    for (&source_face, &result_face) in face_map {
        let Some(source_face) = topo.face_id_from_index(source_face) else {
            return Ok(Vec::new());
        };
        let before = topo.face(source_face)?;
        let after = topo.face(result_face)?;
        if mapped_wire_cycle(topo, before.outer_wire(), Some(&edge_map))?
            != mapped_wire_cycle(topo, after.outer_wire(), None)?
        {
            return Ok(Vec::new());
        }
        let mut before_holes = before
            .inner_wires()
            .iter()
            .map(|&wire| mapped_wire_cycle(topo, wire, Some(&edge_map)))
            .collect::<Result<Vec<_>, _>>()?;
        let mut after_holes = after
            .inner_wires()
            .iter()
            .map(|&wire| mapped_wire_cycle(topo, wire, None))
            .collect::<Result<Vec<_>, _>>()?;
        before_holes.sort_unstable();
        after_holes.sort_unstable();
        if before_holes != after_holes {
            return Ok(Vec::new());
        }
    }
    Ok(edge_map
        .into_iter()
        .map(|(source, target)| (EntityKey::edge(source), EntityKey::edge(target)))
        .chain(
            vertex_map
                .into_iter()
                .map(|(source, target)| (EntityKey::vertex(source), EntityKey::vertex(target))),
        )
        .collect())
}

fn mapped_wire_cycle(
    topo: &Topology,
    wire: WireId,
    edge_map: Option<&HashMap<usize, usize>>,
) -> Result<Vec<usize>, OperationsError> {
    let edges =
        topo.wire(wire)?
            .edges()
            .iter()
            .map(|edge| {
                let index = edge.edge().index();
                match edge_map {
                    Some(map) => map.get(&index).copied().ok_or_else(|| {
                        reconstruction("boundary correspondence omitted a wire edge")
                    }),
                    None => Ok(index),
                }
            })
            .collect::<Result<Vec<_>, _>>()?;
    let Some(&minimum) = edges.iter().min() else {
        return Ok(edges);
    };
    let mut candidates = Vec::new();
    for (position, &edge) in edges.iter().enumerate() {
        if edge == minimum {
            candidates.push(
                (0..edges.len())
                    .map(|offset| edges[(position + offset) % edges.len()])
                    .collect::<Vec<_>>(),
            );
            candidates.push(
                (0..edges.len())
                    .map(|offset| edges[(position + edges.len() - offset) % edges.len()])
                    .collect::<Vec<_>>(),
            );
        }
    }
    Ok(candidates.into_iter().min().unwrap_or_default())
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

/// Whole-body mesh volume of `solid` for one reconstruction stage, reusing the
/// reading the previous stage took of the same solid.
fn stage_volume_of(
    topo: &Topology,
    solid: SolidId,
    known: Option<f64>,
) -> Result<f64, OperationsError> {
    match known {
        Some(volume) => Ok(volume),
        None => crate::measure::solid_volume(topo, solid, 0.05),
    }
}

/// What [`translated_faces_volume_change`] read off the translated faces.
struct TranslatedFacesChange {
    /// Whole-body volume change of the translation.
    volume: f64,
    /// Area of the translated faces: the scale the change is judged against.
    area: f64,
}

/// Volume change of a rigid translation of `faces`, read from those faces
/// alone.
///
/// Translating a face set S by δ while every face that bounds S is invariant
/// under δ sweeps a prism of volume δ · ∫_S n dA: on S the boundary moves with
/// velocity δ, on the invariant carriers it slides tangentially, everywhere
/// else it stands still. The divergence theorem splits that volume three
/// ways: the moved faces' own flux (1/3)∮ P·n dA changes by exactly one third
/// of it, and the lateral band the carriers grow or lose supplies the other
/// two thirds (a box pulled up by d, integrated about a bottom corner: the lid
/// gains d·A/3, the two walls away from that corner gain d·A/3 each, the two
/// through it nothing). Faces that are neither moved nor carriers keep their
/// geometry and cancel exactly; the carriers' share is fixed by closure, which
/// `validate_exact_result` proves topologically, and carrier invariance is the
/// construction's own premise (`surface_translation_invariant`). So integrating
/// S before and after about one reference point and tripling the difference is
/// the whole-body change without a whole-body mesh.
///
/// Before and after share one surface parameterization and one trim — the
/// surface and its boundary translate together — so the fixed-order quadrature
/// sees the same sample set on both sides and the difference is δ · Σ wᵢ nᵢ
/// over those samples: its error is relative to the moved faces' projected
/// area, never to the body. That premise is checked, not assumed: a rigid
/// translation preserves each face's area, so a face whose quadrature area
/// reads differently after the move did not resample the same trim.
///
/// Both readings go through the thread's face-integral cache when an
/// application enabled it. The "before" side is then usually the source
/// body's own orientation probe re-expressed about this reference, and the
/// "after" side, whose content is the translation of the "before" content,
/// is read from the same entry shifted by `(1/3) δ · N` — exactly the swept
/// prism this function is after, from the same samples, so the area check
/// holds by construction rather than by resampling. With the cache off both
/// sides integrate as before.
///
/// `None` — the caller then compares whole-body meshes instead of refusing —
/// when a moved face declines to integrate, reads back a different area,
/// yields a non-finite number, or belongs to a family whose UV outline the
/// trimmed quadrature can get wrong (`gauss_unqualified_face`) and its
/// boundary-derived shadow along the move disagrees with the quadrature's
/// (`boundary_shadow_along`).
fn translated_faces_volume_change(
    before: &Topology,
    after: &Topology,
    solid: SolidId,
    faces: &HashSet<FaceId>,
    delta: Vec3,
) -> Result<Option<TranslatedFacesChange>, OperationsError> {
    use remus_check::properties::face_cache::integrate_face_volume_about_memoized;

    let reference = remus_check::properties::integration_reference(before, solid)?;
    let order = remus_check::properties::PropertiesOptions::default().gauss_order;
    let mut ordered: Vec<FaceId> = faces.iter().copied().collect();
    ordered.sort_unstable_by_key(|face| face.index());
    let mut change = 0.0;
    let mut area = 0.0;
    for face in ordered {
        let (Ok(was), Ok(now)) = (
            integrate_face_volume_about_memoized(before, face, order, reference),
            integrate_face_volume_about_memoized(after, face, order, reference),
        ) else {
            return Ok(None);
        };
        let readings = [was.volume, now.volume, was.area, now.area];
        if !readings.iter().all(|value| value.is_finite())
            || (now.area - was.area).abs() > was.area.abs().mul_add(1e-9, 1e-12)
        {
            return Ok(None);
        }
        // A wrong UV outline is consistent before and after, so the area
        // check above cannot see it. The families `gauss_unqualified_face`
        // names (a scalloped sphere collar, a torus trim outside the two-rim
        // band family — the generic integrator is on record over-reading a
        // bore's rim torus — and a quadric wall whose rim winds the period)
        // must therefore agree with a reading that does not go through the UV
        // outline at all: the face's shadow along the move taken from its
        // boundary (Stokes). The torus corners of ordinary fillets land here
        // and pass; a misread outline whose extra or missing region casts a
        // shadow along the move does not, and the body falls back to the
        // whole-body comparison.
        let face_change = 3.0 * (now.volume - was.volume);
        if crate::measure::gauss_unqualified_face(before, face)?.is_some() {
            let Some(shadow) = boundary_shadow_along(before, face, delta)? else {
                return Ok(None);
            };
            let slack = (0.02 * delta.length()).mul_add(was.area.abs(), 1e-9);
            if (face_change.abs() - shadow).abs() > slack {
                return Ok(None);
            }
        }
        change += face_change;
        area += was.area.abs();
    }
    Ok(change.is_finite().then_some(TranslatedFacesChange {
        volume: change,
        area,
    }))
}

/// `|δ · ∫_f n dA|` read from the face's boundary alone.
///
/// By Stokes the vector area of a closed loop is `(1/2) ∮ P × dP` whatever
/// surface spans it, so this never consults the face's UV outline and is an
/// independent check on a trimmed quadrature that may have one wrong. Only the
/// magnitude is read — the loop's stored winding is not relied on for a sign —
/// which is all the cross-check needs. Lines contribute exactly; every other
/// edge is sampled densely on its own authoritative domain (2048 chords per
/// edge: a full circle reads within 2e-6 relative), far below the tolerance
/// the caller applies.
///
/// `None` for a face with inner wires (the loops' relative winding would then
/// matter) or an edge without a usable domain.
fn boundary_shadow_along(
    topo: &Topology,
    face: FaceId,
    delta: Vec3,
) -> Result<Option<f64>, OperationsError> {
    const CURVE_SAMPLES: usize = 2048;

    let face_data = topo.face(face)?;
    if !face_data.inner_wires().is_empty() {
        return Ok(None);
    }
    let wire = topo.wire(face_data.outer_wire())?;
    let mut points: Vec<Point3> = Vec::new();
    let mut previous_end: Option<VertexId> = None;
    for oriented in wire.edges() {
        let edge = topo.edge(oriented.edge())?;
        let (start_id, end_id) = (edge.start(), edge.end());
        // Wires store edges in loop order, but per-edge orientation flags are
        // not guaranteed to chain head-to-tail; re-derive the traversal from
        // vertex connectivity with the previous edge (the integrators'
        // convention), falling back to the stored flag for closed edges.
        let forward = match previous_end {
            Some(previous) if start_id == previous && end_id != previous => true,
            Some(previous) if end_id == previous && start_id != previous => false,
            _ => oriented.is_forward(),
        };
        previous_end = Some(if forward { end_id } else { start_id });
        let start = topo.vertex(start_id)?.point();
        let end = topo.vertex(end_id)?.point();
        let mut samples: Vec<Point3> = if matches!(edge.curve(), EdgeCurve::Line) {
            vec![start, end]
        } else {
            let Ok((t0, t1)) = edge.strict_domain() else {
                return Ok(None);
            };
            (0..=CURVE_SAMPLES)
                .map(|index| {
                    let t = (t1 - t0).mul_add(index as f64 / CURVE_SAMPLES as f64, t0);
                    edge.curve().evaluate_with_endpoints(t, start, end)
                })
                .collect()
        };
        if !forward {
            samples.reverse();
        }
        points.extend(samples);
    }
    if points.len() < 3 {
        return Ok(None);
    }
    // The vector area of a closed loop is translation-invariant, so take the
    // cross products about a point on the loop: about the world origin a
    // millimetre loop a metre away loses its area to cancellation.
    let origin = points[0];
    let mut twice_vector_area = Vec3::new(0.0, 0.0, 0.0);
    for pair in points.windows(2) {
        twice_vector_area += (pair[0] - origin).cross(pair[1] - origin);
    }
    if let (Some(&last), Some(&first)) = (points.last(), points.first()) {
        twice_vector_area += (last - origin).cross(first - origin);
    }
    let shadow = delta.dot(twice_vector_area * 0.5).abs();
    Ok(shadow.is_finite().then_some(shadow))
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
) -> Result<BlendMoveEntities, OperationsError> {
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
    for &face in &moved_faces {
        translate_face_surface(&mut work, face, delta)?;
    }
    validate_exact_result(&work, solid, "translation-invariant blend move")?;
    let result_counts = remus_topology::explorer::solid_entity_counts(&work, solid)?;
    if result_counts != source_counts {
        return Err(reconstruction(format!(
            "blend translation changed (faces, edges, vertices) from {source_counts:?} to {result_counts:?}"
        )));
    }
    // The body's volume change is read off the translated faces alone; meshing
    // the whole body twice for this one sign was most of the cost of a planar
    // move on NURBS-heavy imports. On that path the change is `distance` times
    // the region's shadow along the move, so the slack is that shadow against
    // the region's own area: only a degenerate region, whose shadow is
    // negligible next to its area, is refused, independent of body size. A
    // region the integral cannot vouch for keeps the whole-body mesh
    // comparison with its original slack, so those bodies behave as before.
    let (volume_change, volume_slack) = if let Some(read) =
        translated_faces_volume_change(topo, &work, solid, &moved_faces, delta)?
    {
        (
            read.volume,
            (read.area * 1e-9).mul_add(distance.abs(), 1e-7),
        )
    } else {
        let source_volume = crate::measure::solid_volume(topo, solid, 0.05)?;
        let result_volume = crate::measure::solid_volume(&work, solid, 0.05)?;
        (
            result_volume - source_volume,
            source_volume.abs().mul_add(1e-9, 1e-7),
        )
    };
    if volume_change.abs() <= volume_slack
        || volume_change.is_sign_positive() != distance.is_sign_positive()
    {
        return Err(reconstruction(format!(
            "blend-aware move changed volume by {volume_change}, inconsistent with distance {distance}"
        )));
    }

    let copied = crate::copy::copy_solid_between_with_entity_map(&work, topo, solid)?;
    let result = copied.solid;
    validate_exact_result(topo, result, "accepted blend-aware planar move")?;
    let boundary_pairs =
        copied
            .edge_map
            .into_iter()
            .map(|(source, target)| (EntityKey::edge(source), EntityKey::edge(target.index())))
            .chain(copied.vertex_map.into_iter().map(|(source, target)| {
                (EntityKey::vertex(source), EntityKey::vertex(target.index()))
            }))
            .collect();
    Ok(BlendMoveEntities {
        solid: result,
        evolution: crate::push_pull::exact_face_evolution(
            topo,
            &source_faces,
            result,
            copied.face_map,
        )?,
        boundary_pairs,
    })
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

fn describe_band_face_lineage(
    topo: &Topology,
    solid: SolidId,
    band: &BandDescription,
    construction_map: &HashMap<usize, FaceId>,
) -> Result<(Vec<BandFaceLineage>, bool), OperationsError> {
    let current_to_source: HashMap<usize, usize> = construction_map
        .iter()
        .map(|(&source, current)| (current.index(), source))
        .collect();
    let support_set: HashSet<FaceId> = band.supports.iter().copied().collect();
    let adjacency = topo.build_adjacency(solid)?;
    let mut exact = true;
    let mut lineages = Vec::new();
    for &band_face in &band.faces {
        let Some(&source) = current_to_source.get(&band_face.index()) else {
            exact = false;
            continue;
        };
        let mut supports = HashSet::new();
        for edge in face_edges(topo, band_face)? {
            for adjacent in distinct_faces(adjacency.faces_for_edge(edge)) {
                if support_set.contains(&adjacent)
                    && tangent_across(topo, edge, band_face, adjacent)?
                {
                    supports.insert(adjacent);
                }
            }
        }
        let mut supports: Vec<_> = supports.into_iter().collect();
        supports.sort_unstable_by_key(|face| face.index());
        if supports.is_empty() {
            exact = false;
        }
        lineages.push(BandFaceLineage { source, supports });
    }
    if lineages.len() != band.faces.len() {
        exact = false;
    }
    Ok((lineages, exact))
}

fn remap_band_face_lineage(
    mut lineages: Vec<BandFaceLineage>,
    face_map: &HashMap<usize, FaceId>,
) -> Result<Vec<BandFaceLineage>, OperationsError> {
    for lineage in &mut lineages {
        lineage.supports = remap_faces(&lineage.supports, face_map, "blend lineage support")?;
    }
    Ok(lineages)
}

fn remap_construction_lineage(
    construction_map: &mut HashMap<usize, FaceId>,
    stage_map: &HashMap<usize, FaceId>,
    removed: &[FaceId],
) -> Result<(), OperationsError> {
    let removed: HashSet<usize> = removed.iter().map(|face| face.index()).collect();
    let current = std::mem::take(construction_map);
    for (source, face) in current {
        if let Some(&result) = stage_map.get(&face.index()) {
            construction_map.insert(source, result);
        } else if !removed.contains(&face.index()) {
            return Err(reconstruction(format!(
                "face {} disappeared without a construction record",
                face.index()
            )));
        }
    }
    Ok(())
}

fn canonical_faces(faces: &[FaceId]) -> Vec<usize> {
    let mut indices: Vec<_> = faces.iter().map(|face| face.index()).collect();
    indices.sort_unstable();
    indices.dedup();
    indices
}

fn restore_band_face_lineage(
    construction_map: &mut HashMap<usize, FaceId>,
    lineages: &[BandFaceLineage],
    origins: &BlendFaceOrigins,
) -> bool {
    let mut exact = origins.created_unattributed.is_empty();
    let mut used = HashSet::new();
    for lineage in lineages {
        let supports = canonical_faces(&lineage.supports);
        let matches: Vec<_> = origins
            .created
            .iter()
            .filter(|(result, sources)| {
                !used.contains(result) && canonical_faces(sources) == supports
            })
            .map(|(result, _)| *result)
            .collect();
        if let [result] = matches.as_slice() {
            construction_map.insert(lineage.source, *result);
            used.insert(*result);
        } else {
            exact = false;
        }
    }
    exact && used.len() == origins.created.len()
}

fn lineage_is_total(
    source_faces: &[FaceId],
    result_faces: &[FaceId],
    construction_map: &HashMap<usize, FaceId>,
) -> bool {
    let sources: HashSet<_> = source_faces.iter().map(|face| face.index()).collect();
    let mapped_sources: HashSet<_> = construction_map.keys().copied().collect();
    let results: HashSet<_> = result_faces.iter().map(|face| face.index()).collect();
    let mapped_results: HashSet<_> = construction_map.values().map(|face| face.index()).collect();
    sources == mapped_sources
        && results == mapped_results
        && construction_map.len() == result_faces.len()
}

fn conservative_move_evolution(
    source_faces: &[FaceId],
    result_faces: &[FaceId],
    construction_map: &HashMap<usize, FaceId>,
) -> EvolutionMap {
    let mut evolution = EvolutionMap::exact();
    let result_set: HashSet<_> = result_faces.iter().map(|face| face.index()).collect();
    let mut candidates_by_result: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (&source, result) in construction_map {
        if result_set.contains(&result.index()) {
            candidates_by_result
                .entry(result.index())
                .or_default()
                .push(source);
        }
    }
    let mut proven_sources = HashSet::new();
    let mut proven_results = HashSet::new();
    for (&result, candidates) in &mut candidates_by_result {
        candidates.sort_unstable();
        candidates.dedup();
        if let [source] = candidates.as_slice() {
            evolution.add_modified(*source, result);
            proven_sources.insert(*source);
            proven_results.insert(result);
        }
    }
    let mut uncertain_sources: Vec<_> = source_faces
        .iter()
        .map(|face| face.index())
        .filter(|source| !proven_sources.contains(source))
        .collect();
    uncertain_sources.sort_unstable();
    for result in result_faces {
        if !proven_results.contains(&result.index()) {
            evolution.add_unresolved(result.index(), uncertain_sources.clone());
        }
    }
    evolution
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
    for lineage in &mut plan.band_faces {
        lineage.supports = remap_faces(&lineage.supports, face_map, "blend lineage support")?;
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

/// Solve `n·p = d` for three unit-normal planes, or `None` when they are too
/// near-parallel to define a corner. Same gate as
/// [`crate::defeature::MIN_PLANE_TRIPLE_DET`]: below it the corner position
/// is meaningless and the heal is refused rather than emitting a far-away
/// intersection point.
fn sharp_triple_corner(a: (Vec3, f64), b: (Vec3, f64), c: (Vec3, f64)) -> Option<Point3> {
    let bc = b.0.cross(c.0);
    let det = a.0.dot(bc);
    if det.abs() < crate::defeature::MIN_PLANE_TRIPLE_DET {
        return None;
    }
    let ca = c.0.cross(a.0);
    let ab = a.0.cross(b.0);
    let v = (bc * a.1 + ca * b.1 + ab * c.1) * (1.0 / det);
    Some(Point3::new(v.x(), v.y(), v.z()))
}

/// Unit plane equation of a planar face: normalized normal with matching
/// offset, so corner solves and containment tests share one convention.
fn unit_plane_of(face: &remus_topology::face::Face) -> Result<(Vec3, f64), OperationsError> {
    match face.surface() {
        FaceSurface::Plane { normal, d } => {
            let unit = normal
                .normalize()
                .map_err(|error| reconstruction(format!("invalid plane normal: {error}")))?;
            let scale = unit.dot(*normal);
            if scale.abs() <= Tolerance::new().angular {
                return Err(reconstruction("degenerate plane normal".to_string()));
            }
            Ok((unit, *d / scale))
        }
        // Callers gate on planarity first; any other carrier here (or a
        // future variant) is classification drift, refused explicitly.
        FaceSurface::Nurbs(_)
        | FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => Err(reconstruction("support face lost its plane".to_string())),
    }
}

/// Distance from a point to the infinite line through two points.
fn point_line_distance(point: Point3, line_a: Point3, line_b: Point3) -> f64 {
    let direction = line_b - line_a;
    let length = direction.length();
    if length <= Tolerance::new().linear {
        return (point - line_a).length();
    }
    (direction.cross(point - line_a)).length() / length
}

/// Orient an edge to traverse from one corner point to another, matching
/// stored endpoints within tolerance.
fn orient_corners(
    topo: &Topology,
    edge: EdgeId,
    from: Point3,
    to: Point3,
) -> Result<OrientedEdge, OperationsError> {
    let tol = Tolerance::new();
    let data = topo.edge(edge)?;
    let start = topo.vertex(data.start())?.point();
    let end = topo.vertex(data.end())?.point();
    if (start - from).length() <= tol.linear && (end - to).length() <= tol.linear {
        return Ok(OrientedEdge::new(edge, true));
    }
    if (start - to).length() <= tol.linear && (end - from).length() <= tol.linear {
        return Ok(OrientedEdge::new(edge, false));
    }
    Err(reconstruction(format!(
        "edge {} does not span the requested corners",
        edge.index()
    )))
}

/// A verified transverse-cylinder strip end: the R8 contact (N) and oblique
/// contact (C) share band vertex B; A reaches the sharp corner P2 along the
/// kept R8/support generatrix; D reaches Q* along the kept oblique/support
/// line; Q* is the transverse generatrix/support-plane crossing.
struct CompoundEnd {
    r8: FaceId,
    n_edge: EdgeId,
    c_edge: EdgeId,
    a: VertexId,
    b: VertexId,
    d: VertexId,
    sy: FaceId,
    p2: Point3,
    qstar: Point3,
    west: FaceId,
    west_point: Point3,
}

/// One topologically split contact between the cylindrical band and a planar
/// support.  The edges are allowed to be split, but only when they form one
/// connected, non-branching line chain on the same exact generatrix.
struct SpringChain {
    edges: Vec<EdgeId>,
    endpoints: [VertexId; 2],
    interior_vertices: Vec<VertexId>,
}

fn face_contains_contiguous_chain(
    topo: &Topology,
    face: FaceId,
    ordered_edges: &[EdgeId],
    ordered_vertices: &[VertexId],
) -> Result<bool, OperationsError> {
    let face = topo.face(face)?;
    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        let wire = topo.wire(wire_id)?;
        let uses = wire.edges();
        if ordered_edges.len() > uses.len() {
            continue;
        }
        for start in 0..uses.len() {
            let forward = ordered_edges.iter().enumerate().all(|(offset, edge)| {
                let oriented = uses[(start + offset) % uses.len()];
                let Ok(data) = topo.edge(oriented.edge()) else {
                    return false;
                };
                oriented.edge() == *edge
                    && oriented.oriented_start(data) == ordered_vertices[offset]
                    && oriented.oriented_end(data) == ordered_vertices[offset + 1]
            });
            let reverse = ordered_edges
                .iter()
                .rev()
                .enumerate()
                .all(|(offset, edge)| {
                    let oriented = uses[(start + offset) % uses.len()];
                    let Ok(data) = topo.edge(oriented.edge()) else {
                        return false;
                    };
                    let vertex = ordered_vertices.len() - 1 - offset;
                    oriented.edge() == *edge
                        && oriented.oriented_start(data) == ordered_vertices[vertex]
                        && oriented.oriented_end(data) == ordered_vertices[vertex - 1]
                });
            if forward || reverse {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// Prove that all contacts with one support are one collinear cylindrical
/// generatrix.  A multi-edge contact is claimed by the surgical path once it
/// has line geometry; malformed split topology is therefore a typed refusal
/// rather than a fall-through to the positional healer.
fn prove_spring_chain(
    topo: &Topology,
    support: FaceId,
    band: FaceId,
    contacts: Vec<EdgeId>,
) -> Result<Option<SpringChain>, OperationsError> {
    if contacts.is_empty() {
        return Ok(None);
    }
    let split = contacts.len() > 1;
    if contacts.iter().any(|edge| {
        !matches!(
            topo.edge(*edge).map(remus_topology::edge::Edge::curve),
            Ok(EdgeCurve::Line)
        )
    }) {
        return if split {
            Err(reconstruction(
                "split spring chain contains a non-line edge".to_string(),
            ))
        } else {
            Ok(None)
        };
    }

    let fail = |reason: String| {
        if split {
            Err(reconstruction(reason))
        } else {
            Ok(None)
        }
    };
    let mut incidence: HashMap<VertexId, Vec<EdgeId>> = HashMap::new();
    for &edge_id in &contacts {
        let edge = topo.edge(edge_id)?;
        if edge.start() == edge.end() {
            return fail("spring chain contains a closed line edge".to_string());
        }
        incidence.entry(edge.start()).or_default().push(edge_id);
        incidence.entry(edge.end()).or_default().push(edge_id);
    }
    let mut endpoints: Vec<VertexId> = incidence
        .iter()
        .filter_map(|(&vertex, edges)| (edges.len() == 1).then_some(vertex))
        .collect();
    if endpoints.len() != 2 || incidence.values().any(|edges| edges.len() > 2) {
        return fail(format!(
            "spring chain is branched or closed ({} endpoints)",
            endpoints.len()
        ));
    }

    endpoints.sort_unstable_by_key(|vertex| vertex.index());
    // Walk the topology from one terminal to the other. The resulting vertex
    // order is later proved strictly monotone on the common line, excluding
    // overlapping/backtracking chains such as 0→2→1→3.
    let mut ordered_edges = Vec::with_capacity(contacts.len());
    let mut ordered_vertices = Vec::with_capacity(contacts.len() + 1);
    let mut visited = HashSet::new();
    let mut current = endpoints[0];
    ordered_vertices.push(current);
    while current != endpoints[1] {
        let candidates: Vec<EdgeId> = incidence
            .get(&current)
            .map_or(&[][..], Vec::as_slice)
            .iter()
            .copied()
            .filter(|edge| !visited.contains(edge))
            .collect();
        let [edge_id] = candidates.as_slice() else {
            return fail("spring chain is disconnected or has an ambiguous walk".to_string());
        };
        visited.insert(*edge_id);
        ordered_edges.push(*edge_id);
        let edge = topo.edge(*edge_id)?;
        current = if edge.start() == current {
            edge.end()
        } else {
            edge.start()
        };
        ordered_vertices.push(current);
    }
    if visited.len() != contacts.len() {
        return fail("spring chain is disconnected".to_string());
    }

    let start = topo.vertex(endpoints[0])?.point();
    let end = topo.vertex(endpoints[1])?.point();
    let span = (end - start).length();
    let direction = (end - start)
        .normalize()
        .map_err(|error| reconstruction(format!("degenerate spring chain: {error}")))?;
    let tol = Tolerance::new();
    let support_plane = unit_plane_of(topo.face(support)?)?;
    let FaceSurface::Cylinder(cylinder) = topo.face(band)?.surface() else {
        return Err(reconstruction(
            "spring-chain proof lost its cylindrical band".to_string(),
        ));
    };
    let axis = cylinder
        .axis()
        .normalize()
        .map_err(|error| reconstruction(format!("invalid blend cylinder axis: {error}")))?;
    if 1.0 - direction.dot(axis).abs() > tol.angular {
        return fail("spring chain is not a cylinder generatrix".to_string());
    }
    let mut previous_parameter: Option<f64> = None;
    for &vertex in &ordered_vertices {
        let parameter = (topo.vertex(vertex)?.point() - start).dot(direction);
        if !parameter.is_finite()
            || previous_parameter.is_some_and(|previous| parameter <= previous + tol.linear)
            || parameter < -tol.linear
            || parameter > span + tol.linear
        {
            return fail("spring chain backtracks or overlaps on its carrier".to_string());
        }
        previous_parameter = Some(parameter);
    }
    for &vertex in incidence.keys() {
        let point = topo.vertex(vertex)?.point();
        if point_line_distance(point, start, end) > tol.linear {
            return fail("spring chain segments are not collinear".to_string());
        }
        if (dot_normal_point(support_plane.0, point) - support_plane.1).abs() > tol.linear {
            return fail("spring chain leaves its planar support".to_string());
        }
        let radial = (point - cylinder.origin()) - axis * (point - cylinder.origin()).dot(axis);
        if (radial.length() - cylinder.radius()).abs() > tol.linear {
            return fail("spring chain leaves the cylindrical band carrier".to_string());
        }
    }

    if !face_contains_contiguous_chain(topo, support, &ordered_edges, &ordered_vertices)?
        || !face_contains_contiguous_chain(topo, band, &ordered_edges, &ordered_vertices)?
    {
        return fail(
            "spring chain is not one contiguous boundary run on both incident faces".to_string(),
        );
    }

    let mut interior_vertices: Vec<VertexId> = incidence
        .into_iter()
        .filter_map(|(vertex, edges)| (edges.len() == 2).then_some(vertex))
        .collect();
    interior_vertices.sort_unstable_by_key(|vertex| vertex.index());
    Ok(Some(SpringChain {
        edges: ordered_edges,
        endpoints: [endpoints[0], endpoints[1]],
        interior_vertices,
    }))
}

/// Intersection of two coplanar lines, or `None` when parallel. Coplanarity
/// itself is verified by the caller comparing the result against both lines.
fn line_line_intersection(
    point_a: Point3,
    direction_a: Vec3,
    point_b: Point3,
    direction_b: Vec3,
) -> Option<Point3> {
    let cross = direction_a.cross(direction_b);
    if cross.length() <= 1e-9 {
        return None;
    }
    // Solve point_a + t * direction_a == point_b + s * direction_b in the
    // least-squares sense; exact for coplanar lines.
    let difference = point_b - point_a;
    let t = difference.cross(direction_b).dot(cross) / cross.dot(cross);
    Some(point_a + direction_a * t)
}

/// Intersection of a line with a unit plane, or `None` when parallel.
fn line_plane_intersection(point: Point3, direction: Vec3, normal: Vec3, d: f64) -> Option<Point3> {
    let denominator = normal.dot(direction);
    if denominator.abs() <= 1e-9 {
        return None;
    }
    Some(point + direction * ((d - crate::dot_normal_point(normal, point)) / denominator))
}

/// Surgical removal of one cylindrical plane-to-plane blend band.
///
/// The copied solid's wound wires are edited in place: each collapsing cross
/// arc is deleted, each spring contact becomes one shared sharp edge between
/// the two supports, and surviving boundary edges are re-anchored as new
/// lines only when the recovered corner lies exactly on their carrier and on
/// every adjacent face. A strip end capped by one plane resolves to its sharp
/// triple; a transverse-cylinder compound end (one R8 contact plus one
/// oblique contact sharing a band vertex) resolves to the R8 piercing P2 and
/// the transverse generatrix/support crossing Q*, with an analytic circle
/// arc P2→Q* on the support/R8 intersection. Sibling arcs, holes, cone
/// shoulders, and every face away from the wound keep their entities
/// untouched, so unrelated analytic geometry (and any pcurves registered on
/// surviving uses) survives exactly.
///
/// Scope and fallback contract: `Ok(None)` declines anything outside the
/// isolated-strip scope (non-cylindrical bands, support counts, non-line or
/// unprovable spring chains, extensions such as off-carrier corners or
/// point contacts) so callers fall back to the positional healer, which
/// still owns those shapes with identical outcomes to before. Definitive
/// `Err` is reserved for configurations the fallback cannot heal either:
/// bands ending on curved geometry outside the compound scope, non-planar
/// wound neighbors, and malformed compound ends.
#[allow(clippy::too_many_lines)]
fn heal_cylinder_plane_band_surgical(
    topo: &mut Topology,
    solid: SolidId,
    band: &BandDescription,
) -> Result<Option<crate::defeature::DefeatureOutcome>, OperationsError> {
    if band.faces.len() != 1 {
        return Ok(None);
    }
    let band_source = band.faces[0];
    if !matches!(topo.face(band_source)?.surface(), FaceSurface::Cylinder(_)) {
        return Ok(None);
    }
    if band.supports.len() != 2
        || band.supports.iter().any(|support| {
            topo.face(*support)
                .is_ok_and(|face| !face.surface().is_planar())
        })
    {
        return Ok(None);
    }
    if !topo.solid(solid)?.inner_shells().is_empty() {
        // The positional fallback owns the cavity refusal.
        return Ok(None);
    }
    if let Some(outcome) = crate::affine_blend_caps::heal_cylinder_plane_band_affine_cap(
        topo,
        solid,
        band_source,
        [band.supports[0], band.supports[1]],
    )? {
        return Ok(Some(outcome));
    }
    if let Some(outcome) = crate::local_wound::heal_cylinder_plane_band_sphere_end(
        topo,
        solid,
        band_source,
        [band.supports[0], band.supports[1]],
    )? {
        return Ok(Some(outcome));
    }

    // A scope decline must not allocate even tombstones in the caller's
    // arena: transaction rollback preserves those slots and their payloads.
    // Isolate only this solid, then publish only an accepted exact result.
    let mut work = Topology::new();
    let input = crate::copy::copy_solid_between_with_entity_map(topo, &mut work, solid)?;
    let Some(outcome) = heal_cylinder_plane_strip(&mut work, input, band)? else {
        return Ok(None);
    };
    let output = crate::copy::copy_solid_between_with_entity_map(&work, topo, outcome.solid)?;
    let mut face_map = HashMap::new();
    for (source, result) in outcome.face_map {
        let target = output
            .face_map
            .get(&result.index())
            .copied()
            .ok_or_else(|| reconstruction("accepted surgical face did not survive exact copy"))?;
        face_map.insert(source, target);
    }
    let output_keys: HashMap<_, _> =
        output
            .edge_map
            .into_iter()
            .map(|(source, target)| (EntityKey::edge(source), EntityKey::edge(target.index())))
            .chain(output.vertex_map.into_iter().map(|(source, target)| {
                (EntityKey::vertex(source), EntityKey::vertex(target.index()))
            }))
            .collect();
    let boundary_history = outcome
        .boundary_history
        .map(|history| {
            history
                .into_iter()
                .map(|(source, target)| {
                    let target = target
                        .map(|target| {
                            output_keys.get(&target).copied().ok_or_else(|| {
                                reconstruction("surgical boundary history lost its result")
                            })
                        })
                        .transpose()?;
                    Ok((source, target))
                })
                .collect::<Result<_, OperationsError>>()
        })
        .transpose()?;
    Ok(Some(crate::defeature::DefeatureOutcome {
        solid: output.solid,
        face_map,
        boundary_history,
    }))
}

/// Speculative strip construction. All scope declines and partial splice
/// products remain in the disposable topology supplied by the caller.
#[allow(clippy::too_many_lines)]
fn heal_cylinder_plane_strip(
    topo: &mut Topology,
    copied_entities: crate::copy::CopiedSolidEntities,
    band: &BandDescription,
) -> Result<Option<crate::defeature::DefeatureOutcome>, OperationsError> {
    let tol = Tolerance::new();
    let band_source = band.faces[0];
    let copy = copied_entities.solid;
    let mut face_map_indices: HashMap<_, _> = copied_entities
        .face_map
        .iter()
        .map(|(&source, face)| (source, face.index()))
        .collect();
    let copied = |map: &HashMap<usize, usize>, source: FaceId| {
        map.get(&source.index())
            .and_then(|index| topo.face_id_from_index(*index))
            .ok_or_else(|| reconstruction(format!("face {} was not copied", source.index())))
    };
    let band_face = copied(&face_map_indices, band_source)?;
    let supports = [
        copied(&face_map_indices, band.supports[0])?,
        copied(&face_map_indices, band.supports[1])?,
    ];
    let support_set: HashSet<FaceId> = supports.iter().copied().collect();
    let (support_plane0, support_plane1) = (
        unit_plane_of(topo.face(supports[0])?)?,
        unit_plane_of(topo.face(supports[1])?)?,
    );

    // Spring contacts: one proven line chain per support. STEP importers and
    // prior exact operations may retain harmless split vertices along a
    // generatrix, so edge count alone is not a geometric ambiguity.
    let mut spring_chains = Vec::new();
    for &support in &supports {
        let contacts = shared_edges(topo, copy, support, band_face)?;
        let Some(chain) = prove_spring_chain(topo, support, band_face, contacts)? else {
            return Ok(None);
        };
        spring_chains.push(chain);
    }
    let springs: Vec<EdgeId> = spring_chains
        .iter()
        .flat_map(|chain| chain.edges.iter().copied())
        .collect();
    let spring_set: HashSet<EdgeId> = springs.iter().copied().collect();
    let spring_chain_by_edge: HashMap<EdgeId, usize> = spring_chains
        .iter()
        .enumerate()
        .flat_map(|(index, chain)| chain.edges.iter().copied().map(move |edge| (edge, index)))
        .collect();
    let spring_interior_vertices: HashSet<VertexId> = spring_chains
        .iter()
        .flat_map(|chain| chain.interior_vertices.iter().copied())
        .collect();

    // Cross arcs: every remaining band outer edge. The isolated strip carries
    // exactly two, each meeting exactly one end face.
    let band_data = topo.face(band_face)?;
    if !band_data.inner_wires().is_empty() {
        return Ok(None);
    }
    let band_edges: Vec<OrientedEdge> = topo.wire(band_data.outer_wire())?.edges().to_vec();
    let crosses: Vec<EdgeId> = band_edges
        .iter()
        .map(OrientedEdge::edge)
        .filter(|edge| !spring_set.contains(edge))
        .collect();
    for chain in &spring_chains {
        let malformed = chain.endpoints.iter().any(|endpoint| {
            crosses
                .iter()
                .filter(|edge| {
                    topo.edge(**edge)
                        .is_ok_and(|edge| edge.start() == *endpoint || edge.end() == *endpoint)
                })
                .count()
                != 1
        }) || chain.interior_vertices.iter().any(|interior| {
            crosses.iter().any(|edge| {
                topo.edge(*edge)
                    .is_ok_and(|edge| edge.start() == *interior || edge.end() == *interior)
            })
        });
        if malformed {
            if chain.edges.len() > 1 {
                return Err(reconstruction(
                    "split spring chain terminals do not meet exactly one end contact".to_string(),
                ));
            }
            return Ok(None);
        }
    }
    let adjacency = topo.build_adjacency(copy)?;
    // A cross edge ending on curved geometry outside the transverse-cylinder
    // compound scope names the missing construction precisely and refuses: no
    // fallback healer owns curved-carrier sharp terminations either. A single
    // cylindrical neighbor diverts to the compound-end reconstruction below.
    let mut curved_cylinder_endfaces: Vec<FaceId> = Vec::new();
    for &cross in &crosses {
        for face in adjacency
            .faces_for_edge(cross)
            .iter()
            .copied()
            .filter(|face| *face != band_face)
        {
            let neighbor = topo.face(face)?;
            if neighbor.surface().is_planar() {
                continue;
            }
            if matches!(neighbor.surface(), FaceSurface::Cylinder(_)) {
                if !curved_cylinder_endfaces.contains(&face) {
                    curved_cylinder_endfaces.push(face);
                }
                continue;
            }
            return Err(curved_end_refusal(face, topo));
        }
    }
    // End grouping. Planar crosses group by neighbor face; each R8-cylinder
    // cross joins the planar crosses sharing its band vertices into one
    // compound end. Crosses touching any other curved surface refuse above.
    let mut cross_neighbor: HashMap<EdgeId, FaceId> = HashMap::new();
    for &cross in &crosses {
        let mut neighbors: Vec<FaceId> = adjacency
            .faces_for_edge(cross)
            .iter()
            .copied()
            .filter(|face| *face != band_face)
            .collect();
        neighbors.sort_unstable_by_key(|face| face.index());
        neighbors.dedup();
        if neighbors.len() != 1 {
            return Ok(None);
        }
        if support_set.contains(&neighbors[0]) {
            return Ok(None);
        }
        cross_neighbor.insert(cross, neighbors[0]);
    }
    let mut endfaces: Vec<FaceId> = cross_neighbor.values().copied().collect();
    endfaces.sort_unstable_by_key(|face| face.index());
    endfaces.dedup();
    // A compound R8 end groups one cylindrical cross-neighbor with the planar
    // crosses sharing its band vertices; every other end must be a single
    // planar face. Anything else declines to the positional fallback, except
    // a second curved neighbor, which no fallback healer owns either.
    let mut curved_endfaces: Vec<FaceId> = Vec::new();
    let mut planar_endfaces: Vec<FaceId> = Vec::new();
    for &end in &endfaces {
        if topo.face(end)?.surface().is_planar() {
            planar_endfaces.push(end);
        } else {
            curved_endfaces.push(end);
        }
    }
    // Supported end counts: two planar ends (isolated strip), or planar ends
    // plus one transverse-cylinder compound end.
    let compound_r8 = match curved_endfaces.as_slice() {
        [] => None,
        [r8] => {
            if !matches!(topo.face(*r8)?.surface(), FaceSurface::Cylinder(_)) {
                return Err(curved_end_refusal(*r8, topo));
            }
            Some(*r8)
        }
        [first, ..] => {
            return Err(curved_end_refusal(*first, topo));
        }
    };
    // Compound-end reconstruction. The R8 cross (N) and the oblique cross (C)
    // must share exactly one band vertex (B); the remaining planar crosses
    // belong to plane-capped ends. Every structural violation below is
    // definitive: the positional fallback refuses R8-ending bands as well.
    let compound: Option<CompoundEnd> = if let Some(r8) = compound_r8 {
        // R8 crosses: exactly one contact with the cylinder.
        let r8_crosses: Vec<EdgeId> = crosses
            .iter()
            .copied()
            .filter(|cross| cross_neighbor.get(cross).is_some_and(|face| *face == r8))
            .collect();
        if r8_crosses.len() != 1 {
            return Err(reconstruction(format!(
                "compound R8 end needs exactly one R8 contact edge, found {}",
                r8_crosses.len()
            )));
        }
        let n_edge = r8_crosses[0];
        let n_data = topo.edge(n_edge)?;
        if n_data.start() == n_data.end() {
            return Err(reconstruction(
                "compound R8 contact edge is a closed loop".to_string(),
            ));
        }
        // Oblique cross: exactly one planar cross sharing a band vertex with N.
        let n_vertices = [n_data.start(), n_data.end()];
        let mut obl_crosses: Vec<EdgeId> = Vec::new();
        for &cross in &crosses {
            if cross == n_edge {
                continue;
            }
            let edge = topo.edge(cross)?;
            if [edge.start(), edge.end()]
                .iter()
                .any(|vertex| n_vertices.contains(vertex))
                && matches!(
                    topo.face(cross_neighbor[&cross])?.surface(),
                    FaceSurface::Plane { .. }
                )
            {
                obl_crosses.push(cross);
            }
        }
        if obl_crosses.len() != 1 {
            return Err(reconstruction(format!(
                "compound R8 end needs exactly one oblique contact edge, found {}",
                obl_crosses.len()
            )));
        }
        let c_edge = obl_crosses[0];
        let c_data = topo.edge(c_edge)?;
        if c_data.start() == c_data.end() {
            return Err(reconstruction(
                "compound oblique contact edge is a closed loop".to_string(),
            ));
        }
        // Shared vertex B; A on N with a spring; D on C with a spring.
        let b = [c_data.start(), c_data.end()]
            .into_iter()
            .find(|vertex| n_vertices.contains(vertex))
            .ok_or_else(|| reconstruction("compound contacts share no band vertex".to_string()))?;
        let a = [n_data.start(), n_data.end()]
            .into_iter()
            .find(|vertex| *vertex != b)
            .ok_or_else(|| reconstruction("compound R8 contact is degenerate".to_string()))?;
        let d = [c_data.start(), c_data.end()]
            .into_iter()
            .find(|vertex| *vertex != b)
            .ok_or_else(|| reconstruction("compound oblique contact is degenerate".to_string()))?;
        if a == d {
            return Err(reconstruction(
                "compound end contacts share both band vertices".to_string(),
            ));
        }
        // Role verification by incidence: A meets a spring and a kept
        // R8/support generatrix; B meets no spring; D meets a spring and a
        // kept oblique/support line.
        let springs_at = |vertex: VertexId| -> Vec<EdgeId> {
            band_edges
                .iter()
                .map(OrientedEdge::edge)
                .filter(|edge| spring_set.contains(edge))
                .filter(|edge| {
                    topo.edge(*edge)
                        .is_ok_and(|data| data.start() == vertex || data.end() == vertex)
                })
                .collect()
        };
        if springs_at(a).len() != 1 || !springs_at(b).is_empty() {
            return Err(reconstruction(
                "compound end spring incidence is not one spring at A, none at B".to_string(),
            ));
        }
        if springs_at(d).len() != 1 {
            return Err(reconstruction(
                "compound end spring incidence is not one spring at D".to_string(),
            ));
        }
        let kept_lines_at = |vertex: VertexId| -> Vec<EdgeId> {
            let mut lines = Vec::new();
            for &face in &remus_topology::explorer::solid_faces(topo, copy).unwrap_or_default() {
                if face == band_face {
                    continue;
                }
                let face_data = match topo.face(face) {
                    Ok(face_data) => face_data,
                    Err(_) => continue,
                };
                for wire_id in std::iter::once(face_data.outer_wire())
                    .chain(face_data.inner_wires().iter().copied())
                {
                    let wire = match topo.wire(wire_id) {
                        Ok(wire) => wire,
                        Err(_) => continue,
                    };
                    for oriented in wire.edges() {
                        let edge_id = oriented.edge();
                        if spring_set.contains(&edge_id) || crosses.contains(&edge_id) {
                            continue;
                        }
                        if topo.edge(edge_id).is_ok_and(|data| {
                            matches!(data.curve(), EdgeCurve::Line)
                                && (data.start() == vertex || data.end() == vertex)
                        }) {
                            lines.push(edge_id);
                        }
                    }
                }
            }
            lines.sort_unstable_by_key(|edge| edge.index());
            lines.dedup();
            lines
        };
        // E_z at A: kept line shared by R8 and a support. Exactly one: a
        // split collinear generatrix (two edges where one would do) is a
        // conservative refusal — fail-closed and fixture-correct, documented
        // as a known limitation, not a silent merge.
        let ez_candidates: Vec<EdgeId> = kept_lines_at(a)
            .into_iter()
            .filter(|edge| {
                adjacency
                    .faces_for_edge(*edge)
                    .iter()
                    .copied()
                    .filter(|face| *face != band_face)
                    .any(|face| face == r8)
                    && adjacency
                        .faces_for_edge(*edge)
                        .iter()
                        .any(|face| supports.contains(face) && *face != band_face)
            })
            .collect();
        if ez_candidates.len() != 1 {
            return Err(reconstruction(format!(
                "compound end needs exactly one kept R8/support generatrix at A, found {}",
                ez_candidates.len()
            )));
        }
        let ez = ez_candidates[0];
        let ez_support = adjacency
            .faces_for_edge(ez)
            .iter()
            .copied()
            .find(|face| supports.contains(face))
            .ok_or_else(|| reconstruction("kept generatrix lost its support".to_string()))?;
        // E_o at B: kept line shared by R8 and the oblique face.
        let obl = cross_neighbor[&c_edge];
        let eo_candidates: Vec<EdgeId> = kept_lines_at(b)
            .into_iter()
            .filter(|edge| {
                let mut adjacent: Vec<FaceId> = adjacency
                    .faces_for_edge(*edge)
                    .iter()
                    .copied()
                    .filter(|face| *face != band_face)
                    .collect();
                adjacent.sort_unstable_by_key(|face| face.index());
                adjacent.dedup();
                adjacent == vec![obl, r8] || adjacent == vec![r8, obl]
            })
            .collect();
        if eo_candidates.len() != 1 {
            return Err(reconstruction(format!(
                "compound end needs exactly one kept R8/oblique generatrix at B, found {}",
                eo_candidates.len()
            )));
        }
        let eo = eo_candidates[0];
        // E_yo at D: kept line shared by the oblique face and a support.
        let eyo_candidates: Vec<EdgeId> = kept_lines_at(d)
            .into_iter()
            .filter(|edge| {
                adjacency.faces_for_edge(*edge).contains(&obl)
                    && adjacency
                        .faces_for_edge(*edge)
                        .iter()
                        .any(|face| supports.contains(face))
            })
            .collect();
        if eyo_candidates.len() != 1 {
            return Err(reconstruction(format!(
                "compound end needs exactly one kept oblique/support line at D, found {}",
                eyo_candidates.len()
            )));
        }
        let eyo = eyo_candidates[0];
        // S_y: the support containing Q*; S_z: E_z's support. Distinct.
        let sy = supports
            .iter()
            .copied()
            .find(|support| *support != ez_support)
            .ok_or_else(|| reconstruction("compound end supports coincide".to_string()))?;
        // Generatrix proofs: kept lines run along the R8 axis with both
        // endpoints on the carrier, so extensions stay exact.
        let FaceSurface::Cylinder(r8_surface) = topo.face(r8)?.surface().clone() else {
            return Err(reconstruction(
                "compound R8 face lost its cylinder".to_string(),
            ));
        };
        let r8_axis = r8_surface
            .axis()
            .normalize()
            .map_err(|error| reconstruction(format!("invalid R8 axis: {error}")))?;
        for (label, edge) in [("E_z", ez), ("E_o", eo)] {
            let data = topo.edge(edge)?;
            let direction = (topo.vertex(data.end())?.point() - topo.vertex(data.start())?.point())
                .normalize()
                .map_err(|error| reconstruction(format!("degenerate {label}: {error}")))?;
            if 1.0 - direction.dot(r8_axis).abs() > 1e-9 {
                return Err(reconstruction(format!(
                    "{label} is not an R8 generatrix; the compound end is not transverse"
                )));
            }
            for vertex in [data.start(), data.end()] {
                let point = topo.vertex(vertex)?.point();
                let radial = (point - r8_surface.origin())
                    - r8_axis * (point - r8_surface.origin()).dot(r8_axis);
                if (radial.length() - r8_surface.radius()).abs() > tol.linear {
                    return Err(reconstruction(format!(
                        "{label} leaves the R8 carrier; the compound end is not exact"
                    )));
                }
            }
        }
        // P2: sharp line meets E_z's line inside the support plane; the
        // piercing must land on R8.
        let (sy_plane, sz_plane) = (
            unit_plane_of(topo.face(sy)?)?,
            unit_plane_of(topo.face(ez_support)?)?,
        );
        let sharp_direction = (sy_plane.0.cross(sz_plane.0))
            .normalize()
            .map_err(|error| reconstruction(format!("compound supports are parallel: {error}")))?;
        let ez_data = topo.edge(ez)?;
        let ez_a = topo.vertex(ez_data.start())?.point();
        let ez_b = topo.vertex(ez_data.end())?.point();
        let ez_direction = (ez_b - ez_a)
            .normalize()
            .map_err(|error| reconstruction(format!("degenerate kept generatrix: {error}")))?;
        if sharp_direction.cross(ez_direction).length() <= 1e-9 {
            return Err(reconstruction(
                "sharp line runs parallel to the kept generatrix".to_string(),
            ));
        }
        // Both lines lie in the support plane; solve there and certify.
        // The west cap triple anchors the sharp line for the solve; the
        // plane-capped west end is resolved once, here.
        let west_planar: Vec<FaceId> = planar_endfaces
            .iter()
            .copied()
            .filter(|face| *face != obl)
            .collect();
        if west_planar.len() != 1 {
            return Err(reconstruction(
                "compound strip needs exactly one plane-capped end".to_string(),
            ));
        }
        let west_plane = unit_plane_of(topo.face(west_planar[0])?)?;
        let Some(sharp_point) = sharp_triple_corner(sy_plane, sz_plane, west_plane) else {
            return Err(reconstruction(
                "compound west cap is parallel to the supports".to_string(),
            ));
        };
        let p2 = line_line_intersection(sharp_point, sharp_direction, ez_a, ez_direction)
            .ok_or_else(|| reconstruction("sharp line misses the kept generatrix".to_string()))?;
        // Both lines lie in the support plane, but the generatrix endpoints
        // are only known on it to validation tolerance: certify P2 back on
        // both lines so a skewed solve cannot smuggle in a wrong piercing.
        if point_line_distance(p2, ez_a, ez_b) > tol.linear
            || point_line_distance(p2, sharp_point, sharp_point + sharp_direction) > tol.linear
        {
            return Err(reconstruction(
                "sharp/generatrix solve is skewed; no exact piercing".to_string(),
            ));
        }
        let r8_residual = ((p2 - r8_surface.origin())
            - r8_axis * (p2 - r8_surface.origin()).dot(r8_axis))
        .length()
            - r8_surface.radius();
        if r8_residual.abs() > tol.linear {
            return Err(reconstruction(format!(
                "sharp/R8 piercing misses the R8 carrier by {r8_residual:.3e} mm"
            )));
        }
        // Q*: E_o meets the Sy support plane transversely; the corner must
        // lie on R8 and the oblique face.
        let eo_data = topo.edge(eo)?;
        let eo_a = topo.vertex(eo_data.start())?.point();
        let eo_b = topo.vertex(eo_data.end())?.point();
        let eo_direction = (eo_b - eo_a)
            .normalize()
            .map_err(|error| reconstruction(format!("degenerate oblique generatrix: {error}")))?;
        if eo_direction.dot(sy_plane.0).abs() <= 1e-6 {
            return Err(reconstruction(
                "kept R8/oblique generatrix runs parallel to the support plane".to_string(),
            ));
        }
        let qstar = line_plane_intersection(eo_a, eo_direction, sy_plane.0, sy_plane.1)
            .ok_or_else(|| reconstruction("generatrix misses the support plane".to_string()))?;
        for (label, residual) in [
            ("R8", {
                let offset = qstar - r8_surface.origin();
                (offset - r8_axis * offset.dot(r8_axis)).length() - r8_surface.radius()
            }),
            (
                "oblique face",
                dot_normal_point(unit_plane_of(topo.face(obl)?)?.0, qstar)
                    - unit_plane_of(topo.face(obl)?)?.1,
            ),
        ] {
            if residual.abs() > tol.linear {
                return Err(reconstruction(format!(
                    "recovered Q* misses {label} by {residual:.3e} mm"
                )));
            }
        }
        if (p2 - qstar).length() <= tol.linear {
            return Err(reconstruction(
                "compound end corners coincide; the strip has no R8 boundary".to_string(),
            ));
        }
        // The R8 section in the Sy plane must be a circle (transverse
        // cylinder); an oblique section would need ellipse machinery.
        if 1.0 - sy_plane.0.dot(r8_axis).abs() > 1e-9 {
            return Err(reconstruction(
                "support plane is not transverse to the R8 axis".to_string(),
            ));
        }
        // E_yo must join exactly the oblique face and the Q* support;
        // otherwise D's corner has no consistent carrier pair.
        {
            let mut adjacent: Vec<FaceId> = adjacency
                .faces_for_edge(eyo)
                .iter()
                .copied()
                .filter(|face| *face != band_face)
                .collect();
            adjacent.sort_unstable_by_key(|face| face.index());
            adjacent.dedup();
            let mut expected = [obl, sy];
            expected.sort_unstable_by_key(|face| face.index());
            if adjacent != expected {
                return Err(reconstruction(format!(
                    "kept line at D joins faces {:?}, not the oblique/support pair",
                    adjacent.iter().map(|face| face.index()).collect::<Vec<_>>()
                )));
            }
        }
        Some(CompoundEnd {
            r8,
            n_edge,
            c_edge,
            a,
            b,
            d,
            sy,
            p2,
            qstar,
            west: west_planar[0],
            west_point: sharp_point,
        })
    } else {
        None
    };
    // End corners: plane-capped ends get support/support/endface triples;
    // a compound end contributes its R8 piercing P2 (Q* joins the mapping
    // separately below). Parallel triples decline to the positional fallback,
    // which owns those refusals. End keys/points stay aligned for the sharp
    // edge; keys are face indices (R8 for a compound end), sorted for
    // determinism.
    let mut end_keys: Vec<usize> = Vec::new();
    let mut end_corners: Vec<Point3> = Vec::new();
    let mut triples_by_endface: HashMap<usize, Point3> = HashMap::new();
    if compound.is_none() {
        if crosses.len() != 2 || endfaces.len() != 2 {
            return Ok(None);
        }
        let mut planes = Vec::new();
        for &end in &endfaces {
            planes.push(unit_plane_of(topo.face(end)?)?);
        }
        for (i, &end) in endfaces.iter().enumerate() {
            let Some(point) = sharp_triple_corner(support_plane0, support_plane1, planes[i]) else {
                return Ok(None);
            };
            end_keys.push(end.index());
            end_corners.push(point);
            triples_by_endface.insert(end.index(), point);
        }
    } else {
        let entry = compound
            .as_ref()
            .ok_or_else(|| reconstruction("compound end disappeared".to_string()))?;
        triples_by_endface.insert(entry.west.index(), entry.west_point);
        let mut keys = [
            (entry.west.index(), entry.west_point),
            (entry.r8.index(), entry.p2),
        ];
        keys.sort_unstable_by_key(|(key, _)| *key);
        for (key, point) in keys {
            end_keys.push(key);
            end_corners.push(point);
        }
    }
    if (end_corners[0] - end_corners[1]).length() <= tol.linear {
        return Ok(None);
    }

    // Every terminal band vertex maps to a recovered corner: plane-capped
    // crosses through their endface triple, compound roles explicitly.
    // Proven interior spring-chain vertices disappear with the split contact
    // edges and therefore intentionally have no result vertex.
    let mut corner_of_vertex: HashMap<VertexId, Point3> = HashMap::new();
    for &cross in &crosses {
        if compound
            .as_ref()
            .is_some_and(|entry| cross == entry.n_edge || cross == entry.c_edge)
        {
            continue;
        }
        let neighbor = cross_neighbor[&cross];
        let Some(&point) = triples_by_endface.get(&neighbor.index()) else {
            return Ok(None);
        };
        let edge = topo.edge(cross)?;
        let mut consistent = true;
        for vertex in [edge.start(), edge.end()] {
            match corner_of_vertex.entry(vertex) {
                std::collections::hash_map::Entry::Vacant(slot) => {
                    slot.insert(point);
                }
                std::collections::hash_map::Entry::Occupied(slot) => {
                    if (*slot.get() - point).length() > tol.linear {
                        consistent = false;
                    }
                }
            }
        }
        if !consistent {
            return Ok(None);
        }
    }
    if let Some(entry) = compound.as_ref() {
        // Compound roles: A reaches P2 along the kept generatrix, B and D
        // reach Q* (B is abandoned when its generatrix re-anchors, D rides
        // the re-anchored oblique/support line). A colliding earlier mapping
        // means the strip ends overlap, which has no exact closure.
        for (vertex, point) in [
            (entry.a, entry.p2),
            (entry.b, entry.qstar),
            (entry.d, entry.qstar),
        ] {
            match corner_of_vertex.entry(vertex) {
                std::collections::hash_map::Entry::Vacant(slot) => {
                    slot.insert(point);
                }
                std::collections::hash_map::Entry::Occupied(slot) => {
                    if (*slot.get() - point).length() > tol.linear {
                        return Err(reconstruction(format!(
                            "compound end vertex {} maps to two corners",
                            vertex.index()
                        )));
                    }
                }
            }
        }
    }
    let band_vertices: HashSet<VertexId> = band_edges
        .iter()
        .filter_map(|oriented| {
            topo.edge(oriented.edge())
                .ok()
                .map(|edge| [edge.start(), edge.end()])
        })
        .flatten()
        .collect();
    let terminal_band_vertices: HashSet<VertexId> = band_vertices
        .iter()
        .copied()
        .filter(|vertex| !spring_interior_vertices.contains(vertex))
        .collect();
    if corner_of_vertex.keys().copied().collect::<HashSet<_>>() != terminal_band_vertices {
        return Ok(None);
    }

    // Displacement bound: a healed corner moves by roughly the feature size.
    let (mut patch_lo, mut patch_hi): (Option<Point3>, Option<Point3>) = (None, None);
    for &vertex in &band_vertices {
        let p = topo.vertex(vertex)?.point();
        patch_lo = Some(match patch_lo {
            None => p,
            Some(lo) => Point3::new(lo.x().min(p.x()), lo.y().min(p.y()), lo.z().min(p.z())),
        });
        patch_hi = Some(match patch_hi {
            None => p,
            Some(hi) => Point3::new(hi.x().max(p.x()), hi.y().max(p.y()), hi.z().max(p.z())),
        });
    }
    let max_displacement = patch_lo.zip(patch_hi).map_or(0.0, |(lo, hi)| {
        (hi - lo).length() * crate::defeature::MAX_HEAL_DISPLACEMENT_FACTOR
    });
    for &vertex in &terminal_band_vertices {
        let original = topo.vertex(vertex)?.point();
        let Some(&target) = corner_of_vertex.get(&vertex) else {
            return Ok(None);
        };
        if (target - original).length() > max_displacement {
            return Ok(None);
        }
    }

    // Wound-adjacent kept faces share a wound edge with the band. Any other
    // kept face containing a band vertex is a corner-touch the surgery cannot
    // close exactly (rebuilding it would bend it off its own surface, leaving
    // it would crack the shell), so it is refused rather than corrupted.
    let wound_edges: HashSet<EdgeId> = springs.iter().chain(crosses.iter()).copied().collect();
    let mut wound_adjacent: HashSet<FaceId> = HashSet::new();
    for &edge in &wound_edges {
        for face in adjacency.faces_for_edge(edge).iter().copied() {
            if face != band_face {
                wound_adjacent.insert(face);
            }
        }
    }
    let mut vertex_users: HashMap<VertexId, Vec<FaceId>> = HashMap::new();
    for &face in &remus_topology::explorer::solid_faces(topo, copy)? {
        let face_data = topo.face(face)?;
        for wire_id in
            std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire_id)?.edges() {
                let edge = topo.edge(oriented.edge())?;
                for vertex in [edge.start(), edge.end()] {
                    vertex_users.entry(vertex).or_default().push(face);
                }
            }
        }
    }
    // A kept face that only touches the band at a point (no shared wound
    // edge) declines to the positional fallback: rebuilding it would bend it
    // off its own surface, and leaving it would crack the shell.
    for &vertex in &band_vertices {
        let empty: &[FaceId] = &[];
        for &user in vertex_users.get(&vertex).map_or(empty, Vec::as_slice) {
            if user != band_face && !wound_adjacent.contains(&user) {
                return Ok(None);
            }
        }
    }
    for &face in &wound_adjacent {
        // The compound R8 face is rebuilt by the dedicated re-cut below, not
        // by planar extension.
        if compound.as_ref().is_some_and(|entry| face == entry.r8) {
            continue;
        }
        let face_data = topo.face(face)?;
        if !face_data.surface().is_planar() {
            return Err(OperationsError::Unsupported {
                operation: "resize blend",
                reason: format!(
                    "wound-neighbor face {} is a {} surface; extending the shell \
                     to close the gap is only implemented for planar wound \
                     neighbors",
                    face.index(),
                    face_data.surface().type_tag()
                ),
            });
        }
    }

    // New corner vertices, one per strip end, plus Q* vertices for compound
    // ends, plus the shared sharp edge spanning the two end corners.
    let mut corner_vertices: HashMap<usize, VertexId> = HashMap::new();
    for (key, point) in end_keys.iter().zip(end_corners.iter()) {
        corner_vertices.insert(*key, topo.add_vertex(Vertex::new(*point, tol.linear)));
    }
    // Q* vertices, one per compound end: point plus vertex by R8 key.
    let mut qstars: HashMap<usize, (Point3, VertexId)> = HashMap::new();
    if let Some(entry) = compound.as_ref() {
        qstars.insert(
            entry.r8.index(),
            (
                entry.qstar,
                topo.add_vertex(Vertex::new(entry.qstar, tol.linear)),
            ),
        );
    }
    let corner_vertex_for = |vertex: VertexId,
                             corner_of_vertex: &HashMap<VertexId, Point3>,
                             end_keys: &[usize],
                             end_corners: &[Point3],
                             corner_vertices: &HashMap<usize, VertexId>,
                             qstars: &HashMap<usize, (Point3, VertexId)>|
     -> Result<VertexId, OperationsError> {
        let Some(&target) = corner_of_vertex.get(&vertex) else {
            return Err(reconstruction(format!(
                "wound vertex {} has no recovered corner",
                vertex.index()
            )));
        };
        for (i, &key) in end_keys.iter().enumerate() {
            if (end_corners[i] - target).length() <= tol.linear {
                return corner_vertices
                    .get(&key)
                    .copied()
                    .ok_or_else(|| reconstruction("recovered corner has no vertex".to_string()));
            }
        }
        for (point, vertex_id) in qstars.values() {
            if (*point - target).length() <= tol.linear {
                return Ok(*vertex_id);
            }
        }
        Err(reconstruction(
            "recovered corner matches no end face".to_string(),
        ))
    };
    let sharp_edge = topo.add_edge(Edge::new(
        corner_vertices[&end_keys[0]],
        corner_vertices[&end_keys[1]],
        EdgeCurve::Line,
    ));

    // Compound R8 boundary arcs: analytic circle P2→Q* on the support/R8
    // intersection, with certified trim and a minor-arc gate. One per
    // compound end, shared by its support and R8 wires. The endpoints are
    // already proven on both carriers during classification; the trim
    // certification below re-proves curve-on-surface consistency.
    let mut compound_circles: HashMap<usize, EdgeId> = HashMap::new();
    if let Some(entry) = compound.as_ref() {
        let FaceSurface::Cylinder(r8c) = topo.face(entry.r8)?.surface().clone() else {
            return Err(reconstruction(
                "compound R8 face lost its cylinder".to_string(),
            ));
        };
        let (sy_n, sy_d) = unit_plane_of(topo.face(entry.sy)?)?;
        let r8_axis = r8c
            .axis()
            .normalize()
            .map_err(|error| reconstruction(format!("invalid R8 axis: {error}")))?;
        let axial = sy_n.dot(r8_axis);
        if axial.abs() < 1e-9 {
            return Err(reconstruction(
                "support plane is parallel to the R8 axis".to_string(),
            ));
        }
        let center =
            r8c.origin() + r8_axis * ((sy_d - crate::dot_normal_point(sy_n, r8c.origin())) / axial);
        let p2_vertex = corner_vertices[&entry.r8.index()];
        let (qstar_point, qstar_vertex) = qstars[&entry.r8.index()];
        let ref_direction = entry.p2 - center;
        if ref_direction.length() <= tol.linear {
            return Err(reconstruction(
                "R8 piercing coincides with the circle center".to_string(),
            ));
        }
        let circle = Circle3D::new_with_ref(center, sy_n, r8c.radius(), ref_direction)
            .map_err(|error| reconstruction(format!("sharp circle failed: {error}")))?;
        // Minor arc between the two corners; an antipodal span has no unique
        // minor arc and is refused. The trim range must start at the edge's
        // start vertex: swapped order flips the stored endpoints to match.
        let (t0, t1) = crate::boolean::assembly::ccw_arc_trim(&circle, entry.p2, qstar_point, tol)?;
        let (trim, start_is_p2) = if t1 - t0 <= std::f64::consts::PI + 1e-9 {
            if (t1 - t0 - std::f64::consts::PI).abs() <= 1e-9 {
                return Err(reconstruction(
                    "R8 boundary arc is antipodal; the minor arc is ambiguous".to_string(),
                ));
            }
            ((t0, t1), true)
        } else {
            let (s0, s1) =
                crate::boolean::assembly::ccw_arc_trim(&circle, qstar_point, entry.p2, tol)?;
            if s1 - s0 >= std::f64::consts::PI - 1e-9 {
                return Err(reconstruction(
                    "R8 boundary arc is antipodal; the minor arc is ambiguous".to_string(),
                ));
            }
            ((s0, s1), false)
        };
        let (first_vertex, second_vertex) = if start_is_p2 {
            (p2_vertex, qstar_vertex)
        } else {
            (qstar_vertex, p2_vertex)
        };
        let mut arc_edge = Edge::new(first_vertex, second_vertex, EdgeCurve::Circle(circle));
        arc_edge.set_trim(Some(trim));
        arc_edge.strict_domain().map_err(|error| {
            reconstruction(format!("R8 boundary arc has no exportable domain: {error}"))
        })?;
        compound_circles.insert(entry.r8.index(), topo.add_edge(arc_edge));
    }

    // Splice every wound-adjacent wire that carries a band contact. Untouched
    // wires keep their entities, which also keeps registered pcurves valid.
    // Rebuilt wire IDs are tracked for the abandonment scan below.
    let mut rebuilt_wires: HashSet<WireId> = HashSet::new();
    let mut edge_replacements: HashMap<EdgeId, EdgeId> = HashMap::new();
    let mut ordered_adjacent: Vec<FaceId> = wound_adjacent.iter().copied().collect();
    ordered_adjacent.sort_unstable_by_key(|face| face.index());
    for &face in &ordered_adjacent {
        let face_data = topo.face(face)?;
        let wires: Vec<WireId> = std::iter::once(face_data.outer_wire())
            .chain(face_data.inner_wires().iter().copied())
            .collect();
        for wire_id in wires {
            let old_sequence = topo.wire(wire_id)?.edges().to_vec();
            if !old_sequence
                .iter()
                .any(|oriented| wound_edges.contains(&oriented.edge()))
            {
                continue;
            }
            let mut new_sequence = Vec::with_capacity(old_sequence.len());
            let mut emitted_spring_chains = HashSet::new();
            for oriented in &old_sequence {
                if wound_edges.contains(&oriented.edge()) {
                    let edge = topo.edge(oriented.edge())?;
                    if spring_set.contains(&oriented.edge()) {
                        if !support_set.contains(&face) {
                            return Ok(None);
                        }
                        let chain_index = spring_chain_by_edge[&oriented.edge()];
                        if !emitted_spring_chains.insert(chain_index) {
                            // Every segment of a proven split chain is replaced
                            // by the same single result boundary.
                            continue;
                        }
                        let chain = &spring_chains[chain_index];
                        let chain_start = topo.vertex(chain.endpoints[0])?.point();
                        let chain_end = topo.vertex(chain.endpoints[1])?.point();
                        let oriented_start = topo.vertex(oriented.oriented_start(edge))?.point();
                        let oriented_end = topo.vertex(oriented.oriented_end(edge))?.point();
                        let forward =
                            (oriented_end - oriented_start).dot(chain_end - chain_start) > 0.0;
                        let (first, second) = if forward {
                            (chain.endpoints[0], chain.endpoints[1])
                        } else {
                            (chain.endpoints[1], chain.endpoints[0])
                        };
                        let Some(&t0) = corner_of_vertex.get(&first) else {
                            return Ok(None);
                        };
                        let Some(&t1) = corner_of_vertex.get(&second) else {
                            return Ok(None);
                        };

                        let sharp_data = topo.edge(sharp_edge)?;
                        let sharp_start_point = topo.vertex(sharp_data.start())?.point();
                        let sharp_end_point = topo.vertex(sharp_data.end())?.point();
                        let at_end = |point: Point3| {
                            (point - sharp_start_point).length() <= tol.linear
                                || (point - sharp_end_point).length() <= tol.linear
                        };
                        if at_end(t0) && at_end(t1) {
                            new_sequence.push(orient_corners(topo, sharp_edge, t0, t1)?);
                            continue;
                        }

                        // Compound oblique-side spring: the complete chain
                        // spans a sharp corner to Q*. It contributes the sharp
                        // edge and R8 arc once, regardless of contact splits.
                        if let Some(entry) = compound.as_ref() {
                            let touches_d = chain.endpoints.contains(&entry.d);
                            let Some(&circle) = compound_circles.get(&entry.r8.index()) else {
                                return Err(reconstruction("compound R8 arc missing".to_string()));
                            };
                            let west = end_corners
                                .iter()
                                .copied()
                                .find(|corner| (*corner - entry.p2).length() > tol.linear)
                                .ok_or_else(|| {
                                    reconstruction("sharp chain has no far corner".to_string())
                                })?;
                            let goes_forward = touches_d
                                && (t0 - west).length() <= tol.linear
                                && (t1 - entry.qstar).length() <= tol.linear;
                            let goes_reverse = touches_d
                                && (t0 - entry.qstar).length() <= tol.linear
                                && (t1 - west).length() <= tol.linear;
                            if goes_forward {
                                new_sequence.push(orient_corners(topo, sharp_edge, t0, entry.p2)?);
                                new_sequence.push(orient_corners(topo, circle, entry.p2, t1)?);
                                continue;
                            }
                            if goes_reverse {
                                new_sequence.push(orient_corners(topo, circle, t0, entry.p2)?);
                                new_sequence.push(orient_corners(topo, sharp_edge, entry.p2, t1)?);
                                continue;
                            }
                        }
                        return Ok(None);
                    }

                    let Some(&start_corner) = corner_of_vertex.get(&edge.start()) else {
                        return Ok(None);
                    };
                    let Some(&end_corner) = corner_of_vertex.get(&edge.end()) else {
                        return Ok(None);
                    };
                    if (start_corner - end_corner).length() <= tol.linear {
                        // Collapsing cross arc: deleted with the band.
                        continue;
                    }
                    // Compound R8 contact: replaced by the certified R8
                    // boundary arc, oriented to the old traversal.
                    if let Some(entry) = compound.as_ref()
                        && oriented.edge() == entry.n_edge
                    {
                        let Some(&circle) = compound_circles.get(&entry.r8.index()) else {
                            return Err(reconstruction("compound R8 arc missing".to_string()));
                        };
                        // Old traversal corners, ordered.
                        let (t0, t1) = if oriented.is_forward() {
                            (start_corner, end_corner)
                        } else {
                            (end_corner, start_corner)
                        };
                        // The arc spans P2 to Q* in either order; any other
                        // corner pair means the mapping is broken, which
                        // is definitive this far into verified scope.
                        let matches_p2_qstar = (t0 - entry.p2).length() <= tol.linear
                            && (t1 - entry.qstar).length() <= tol.linear;
                        let matches_qstar_p2 = (t0 - entry.qstar).length() <= tol.linear
                            && (t1 - entry.p2).length() <= tol.linear;
                        if !matches_p2_qstar && !matches_qstar_p2 {
                            return Err(reconstruction(
                                "compound R8 contact does not span P2 to Q*".to_string(),
                            ));
                        }
                        new_sequence.push(if matches_p2_qstar {
                            orient_corners(topo, circle, entry.p2, entry.qstar)?
                        } else {
                            orient_corners(topo, circle, entry.qstar, entry.p2)?
                        });
                        continue;
                    }
                    // Every wound edge left here is a cross contact.
                    return Ok(None);
                }
                let edge = topo.edge(oriented.edge())?;
                let remap = |vertex: VertexId| -> Result<VertexId, OperationsError> {
                    if corner_of_vertex.contains_key(&vertex) {
                        corner_vertex_for(
                            vertex,
                            &corner_of_vertex,
                            &end_keys,
                            &end_corners,
                            &corner_vertices,
                            &qstars,
                        )
                    } else {
                        Ok(vertex)
                    }
                };
                let new_start = remap(edge.start())?;
                let new_end = remap(edge.end())?;
                if new_start == new_end {
                    // Both corners merged: dropping the zero-length survivor
                    // is exact.
                    continue;
                }
                if new_start == edge.start() && new_end == edge.end() {
                    new_sequence.push(*oriented);
                    continue;
                }
                // Surviving boundary extended to the recovered corner. Exact
                // only for lines whose carrier and every adjacent face still
                // contain the corner; anything else declines to the
                // positional fallback, which owns those topologies.
                if !matches!(edge.curve(), EdgeCurve::Line) {
                    return Ok(None);
                }
                let old_start = topo.vertex(edge.start())?.point();
                let old_end = topo.vertex(edge.end())?.point();
                let new_start_point = topo.vertex(new_start)?.point();
                let new_end_point = topo.vertex(new_end)?.point();
                if point_line_distance(new_start_point, old_start, old_end) > tol.linear
                    || point_line_distance(new_end_point, old_start, old_end) > tol.linear
                {
                    return Ok(None);
                }
                for other in adjacency.faces_for_edge(oriented.edge()).iter().copied() {
                    if other == face || other == band_face {
                        continue;
                    }
                    if !wound_adjacent.contains(&other) {
                        return Ok(None);
                    }
                    // The re-anchored corner must stay on every adjacent
                    // face's carrier: planes directly, cylinders radially.
                    // Any other carrier declines.
                    match topo.face(other)?.surface() {
                        FaceSurface::Plane { .. } => {
                            let (normal, d) = unit_plane_of(topo.face(other)?)?;
                            for point in [new_start_point, new_end_point] {
                                if (dot_normal_point(normal, point) - d).abs() > tol.linear {
                                    return Ok(None);
                                }
                            }
                        }
                        FaceSurface::Cylinder(cylinder) => {
                            let axis = cylinder.axis().normalize().map_err(|error| {
                                reconstruction(format!("invalid cylinder axis: {error}"))
                            })?;
                            for point in [new_start_point, new_end_point] {
                                let radial = (point - cylinder.origin())
                                    - axis * (point - cylinder.origin()).dot(axis);
                                if (radial.length() - cylinder.radius()).abs() > tol.linear {
                                    return Ok(None);
                                }
                            }
                        }
                        FaceSurface::Cone(_)
                        | FaceSurface::Sphere(_)
                        | FaceSurface::Torus(_)
                        | FaceSurface::Nurbs(_) => return Ok(None),
                    }
                }
                let replacement = *edge_replacements.entry(oriented.edge()).or_insert_with(|| {
                    topo.add_edge(Edge::new(new_start, new_end, EdgeCurve::Line))
                });
                new_sequence.push(OrientedEdge::new(replacement, oriented.is_forward()));
            }
            if new_sequence.is_empty() {
                return Ok(None);
            }
            let new_wire = match Wire::new(new_sequence, true) {
                Ok(wire) => topo.add_wire(wire),
                Err(_) => return Ok(None),
            };
            rebuilt_wires.insert(new_wire);
            replace_face_wire(topo, face, wire_id, new_wire)?;
        }
    }

    // Abandoned vertices (mapped olds superseded by new corner vertices)
    // must not leak into any rebuilt wire; such a leak would duplicate or
    // split a boundary. Untouched wires keep old vertices by design.
    {
        let mut referenced: HashSet<VertexId> = HashSet::new();
        for &wire in &rebuilt_wires {
            for oriented in topo.wire(wire)?.edges() {
                let edge = topo.edge(oriented.edge())?;
                referenced.insert(edge.start());
                referenced.insert(edge.end());
            }
        }
        for &vertex in &band_vertices {
            if referenced.contains(&vertex) {
                return Err(reconstruction(format!(
                    "rebuilt wires still reference superseded vertex {}",
                    vertex.index()
                )));
            }
        }
    }

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
    face_map_indices.remove(&band_source.index());
    let face_map = face_map_indices
        .into_iter()
        .filter_map(|(source, result)| topo.face_id_from_index(result).map(|face| (source, face)))
        .collect();
    let live_edges: HashSet<_> = remus_topology::explorer::solid_edges(topo, sharp_solid)?
        .into_iter()
        .collect();
    let live_vertices: HashSet<_> = remus_topology::explorer::solid_vertices(topo, sharp_solid)?
        .into_iter()
        .collect();
    // Every copied boundary maps: springs to the sharp edge, the compound
    // R8 contact to the R8 arc, extended edges to their replacements,
    // vanished wound edges to nothing, and untouched entities to themselves.
    // New edges are therefore all covered, which the journaled path requires.
    // The oblique-side spring splits into the sharp edge and the arc.
    // Preserve both descendants; output coverage alone is not enough to
    // resolve a reference to that source boundary correctly.
    let mut replaced_edges: HashMap<EdgeId, EdgeId> = HashMap::new();
    for &spring in &springs {
        replaced_edges.insert(spring, sharp_edge);
    }
    if let Some(entry) = compound.as_ref()
        && let Some(&circle) = compound_circles.get(&entry.r8.index())
    {
        replaced_edges.insert(entry.n_edge, circle);
    }
    for (&old, &new) in &edge_replacements {
        replaced_edges.insert(old, new);
    }
    let split_spring = compound.as_ref().and_then(|entry| {
        springs.iter().copied().find(|edge| {
            topo.edge(*edge)
                .is_ok_and(|data| data.start() == entry.d || data.end() == entry.d)
        })
    });
    let mut boundary_history = Vec::new();
    for (source, copied) in copied_entities.edge_map {
        let target = replaced_edges.get(&copied).copied().unwrap_or(copied);
        boundary_history.push((
            EntityKey::edge(source),
            live_edges
                .contains(&target)
                .then_some(EntityKey::edge(target.index())),
        ));
        if Some(copied) == split_spring {
            let entry = compound
                .as_ref()
                .ok_or_else(|| reconstruction("split spring lost compound end"))?;
            let arc = compound_circles[&entry.r8.index()];
            boundary_history.push((EntityKey::edge(source), Some(EntityKey::edge(arc.index()))));
        }
    }
    // Wound vertices merge into their recovered corners; every other copied
    // vertex survives on its own face.
    let mut merged_vertices: HashMap<VertexId, VertexId> = HashMap::new();
    for &vertex in corner_of_vertex.keys() {
        if let Ok(target) = corner_vertex_for(
            vertex,
            &corner_of_vertex,
            &end_keys,
            &end_corners,
            &corner_vertices,
            &qstars,
        ) {
            merged_vertices.insert(vertex, target);
        }
    }
    // Abandoned vertices (mapped olds superseded by new corner vertices)
    // must not leak into any rebuilt wire; such a leak would duplicate or
    // split a boundary, so it is a definitive internal error.
    {
        let mut referenced: HashSet<VertexId> = HashSet::new();
        for &face in &remus_topology::explorer::solid_faces(topo, sharp_solid)? {
            let face_data = topo.face(face)?;
            for wire_id in std::iter::once(face_data.outer_wire())
                .chain(face_data.inner_wires().iter().copied())
            {
                for oriented in topo.wire(wire_id)?.edges() {
                    let edge = topo.edge(oriented.edge())?;
                    referenced.insert(edge.start());
                    referenced.insert(edge.end());
                }
            }
        }
        for &vertex in &band_vertices {
            if referenced.contains(&vertex) {
                return Err(reconstruction(format!(
                    "rebuilt wires still reference superseded vertex {}",
                    vertex.index()
                )));
            }
        }
    }
    for (source, copied) in copied_entities.vertex_map {
        let target = merged_vertices.get(&copied).copied().unwrap_or(copied);
        boundary_history.push((
            EntityKey::vertex(source),
            live_vertices
                .contains(&target)
                .then_some(EntityKey::vertex(target.index())),
        ));
    }
    Ok(Some(crate::defeature::DefeatureOutcome {
        solid: sharp_solid,
        face_map,
        boundary_history: Some(boundary_history),
    }))
}

/// Typed refusal for a blend band ending on curved geometry outside the
/// transverse-cylinder compound scope: no fallback healer owns
/// curved-carrier sharp terminations either.
fn curved_end_refusal(face: FaceId, topo: &Topology) -> OperationsError {
    let surface = topo
        .face(face)
        .map(|face| face.surface().type_tag())
        .unwrap_or("unknown");
    OperationsError::Unsupported {
        operation: "resize blend",
        reason: format!(
            "blend end face {} is a {surface} surface; exact removal of a band \
             ending on curved geometry needs a curved-carrier sharp \
             termination (line-surface piercing plus curved wire re-cut), \
             which is not implemented",
            face.index(),
        ),
    }
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
    // Prefer the surgical healer: it edits wound wires in place and carries
    // every surviving edge, wire, face, and pcurve untouched. Multi-face
    // bands and non-cylindrical carriers decline (`Ok(None)`) and fall back
    // to the positional defeature healer, which still owns those shapes on
    // all-planar bodies.
    let outcome = match heal_cylinder_plane_band_surgical(topo, solid, band)? {
        Some(outcome) => outcome,
        None => crate::defeature::defeature_blend_band(topo, solid, &band.faces)
            .map_err(|error| reconstruction(format!("planar support heal failed: {error}")))?,
    };
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
        boundary_history: outcome.boundary_history,
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

/// Normal about which an exact circle edge travels from its start vertex to
/// its end vertex.  A decreasing trim runs the stored circle clockwise, so its
/// travel normal is the opposite of the stored one; comparing stored normals
/// alone would flip the replacement of such an edge.
fn circle_travel_normal(edge: &Edge, circle: &Circle3D) -> Vec3 {
    match edge.strict_domain() {
        Ok((start, end)) if end < start => circle.normal() * -1.0,
        _ => circle.normal(),
    }
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
    let aligned = circle_travel_normal(old_edge, old_circle).dot(new_curve_normal) >= 0.0;
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

#[allow(clippy::too_many_lines, clippy::unnested_or_patterns)]
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
        // Supports were already classified as a plane/cylinder pair by
        // `remove_blend_region`'s dispatch; any other pairing here (a cone,
        // sphere, torus, or NURBS support, or a future variant) means the
        // classification changed under the copy, so refuse rather than
        // rebuild against the wrong carriers. The two accepted orders above
        // are excluded arm by arm, so a future variant is a compile error.
        (FaceSurface::Plane { .. }, FaceSurface::Plane { .. })
        | (
            FaceSurface::Plane { .. },
            FaceSurface::Nurbs(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_),
        )
        | (FaceSurface::Cylinder(_), FaceSurface::Cylinder(_))
        | (
            FaceSurface::Cylinder(_),
            FaceSurface::Nurbs(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_),
        )
        | (
            FaceSurface::Nurbs(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_),
            _,
        ) => {
            return Err(reconstruction("support classification changed during heal"));
        }
    };

    let copied_entities = crate::copy::copy_solid_with_entity_map(topo, solid)?;
    let copy = copied_entities.solid;
    let mut face_map_indices: HashMap<_, _> = copied_entities
        .face_map
        .iter()
        .map(|(&source, face)| (source, face.index()))
        .collect();
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
        // The plane handle was resolved from a plane/cylinder pair above; a
        // non-plane carrier here (or a future variant) is the same
        // classification drift, refused with the same typed error.
        FaceSurface::Nurbs(_)
        | FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => {
            return Err(reconstruction("plane support lost its surface"));
        }
    };
    let cylinder_surface = match topo.face(cylinder)?.surface() {
        FaceSurface::Cylinder(surface) => surface.clone(),
        // Symmetric to the plane re-check above: any non-cylinder carrier
        // (or a future variant) is classification drift, not a rebuild case.
        FaceSurface::Plane { .. }
        | FaceSurface::Nurbs(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => {
            return Err(reconstruction("cylinder support lost its surface"));
        }
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
        // The plane contact of a closed plane/cylinder rim is an exact
        // circle by construction. Lines, conic arcs, and NURBS (or a future
        // curve variant) mean the rim is no longer that analytic contact,
        // so refuse with the same typed error rather than rebuilding.
        EdgeCurve::Line
        | EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => {
            return Err(reconstruction("plane contact is not an exact circle"));
        }
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
    let live_edges: HashSet<_> = remus_topology::explorer::solid_edges(topo, sharp_solid)?
        .into_iter()
        .collect();
    let live_vertices: HashSet<_> = remus_topology::explorer::solid_vertices(topo, sharp_solid)?
        .into_iter()
        .collect();
    let plane_contact_vertex = topo.edge(*plane_contact)?.start();
    let mut boundary_history = Vec::new();
    for (source, copied) in copied_entities.edge_map {
        let target = if copied == *plane_contact || copied == *cylinder_contact {
            sharp_edge
        } else if copied == *old_seam {
            sharp_seam
        } else {
            copied
        };
        boundary_history.push((
            EntityKey::edge(source),
            live_edges
                .contains(&target)
                .then_some(EntityKey::edge(target.index())),
        ));
    }
    for (source, copied) in copied_entities.vertex_map {
        let target = if copied == plane_contact_vertex || copied == old_contact_vertex {
            sharp_vertex
        } else {
            copied
        };
        boundary_history.push((
            EntityKey::vertex(source),
            live_vertices
                .contains(&target)
                .then_some(EntityKey::vertex(target.index())),
        ));
    }
    Ok(SharpResult {
        solid: sharp_solid,
        edges: vec![sharp_edge],
        face_map,
        boundary_history: Some(boundary_history),
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
    let edge = topo.edge(oriented.edge())?;
    let EdgeCurve::Circle(circle) = edge.curve() else {
        return Err(reconstruction("support contact is not circular"));
    };
    Ok(if circle_travel_normal(edge, circle).dot(axis) >= 0.0 {
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
) -> Result<(EdgeId, Option<(EdgeId, remus_topology::VertexId)>), OperationsError> {
    let (wire_id, old_wire) = contact_wire(topo, face, contacts)?;
    let forward = contact_direction(topo, &old_wire, contacts, axis)?;
    let far_boundary = ordered_circle_boundary(topo, &old_wire, far_circles, far_vertex)?;
    let mut contact_vertices = HashSet::new();
    for &contact in contacts {
        let edge = topo.edge(contact)?;
        contact_vertices.extend([edge.start(), edge.end()]);
    }
    let mut candidates = Vec::new();
    for oriented in &old_wire {
        let edge = topo.edge(oriented.edge())?;
        if !matches!(edge.curve(), EdgeCurve::Line) {
            continue;
        }
        let contact = if edge.start() == far_vertex && contact_vertices.contains(&edge.end()) {
            Some(edge.end())
        } else if edge.end() == far_vertex && contact_vertices.contains(&edge.start()) {
            Some(edge.start())
        } else {
            None
        };
        if let Some(contact) = contact {
            candidates.push((oriented.edge(), contact));
        }
    }
    candidates.sort_unstable();
    candidates.dedup();
    let source = match candidates.as_slice() {
        // Zero or several ambiguous seam sources mean no single seam to
        // carry forward; the caller re-derives the seam. This matches on a
        // slice of candidate pairs (not on `EdgeCurve`/`FaceSurface`), so it
        // carries no wildcard-arm audit obligation.
        [] | [_, _, ..] => None,
        [source] => Some(*source),
    };
    let seam = topo.add_edge(Edge::new(far_vertex, sharp_vertex, EdgeCurve::Line));
    let mut edges = Vec::with_capacity(far_boundary.len() + 3);
    edges.push(OrientedEdge::new(seam, true));
    edges.push(OrientedEdge::new(sharp_edge, forward));
    edges.push(OrientedEdge::new(seam, false));
    edges.extend(far_boundary);
    let new_wire = topo.add_wire(Wire::new(edges, true)?);
    replace_face_wire(topo, face, wire_id, new_wire)?;
    Ok((seam, source))
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
        // As in the plane/cylinder healer: supports were dispatched as a
        // cylinder/cone pair, so any other pairing (including a future
        // variant) is classification drift, refused with the same error.
        (
            FaceSurface::Plane { .. }
            | FaceSurface::Nurbs(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_),
            _,
        )
        | (
            FaceSurface::Cylinder(_),
            FaceSurface::Plane { .. }
            | FaceSurface::Nurbs(_)
            | FaceSurface::Cylinder(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_),
        )
        | (
            FaceSurface::Cone(_),
            FaceSurface::Plane { .. }
            | FaceSurface::Nurbs(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_),
        ) => {
            return Err(reconstruction("support classification changed during heal"));
        }
    };

    let copied_entities = crate::copy::copy_solid_with_entity_map(topo, solid)?;
    let copy = copied_entities.solid;
    let mut face_map_indices: HashMap<_, _> = copied_entities
        .face_map
        .iter()
        .map(|(&source, face)| (source, face.index()))
        .collect();
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
        // Same drift guard as the plane/cylinder healer: any non-cylinder
        // carrier (or a future variant) refuses with the same typed error.
        FaceSurface::Plane { .. }
        | FaceSurface::Nurbs(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => {
            return Err(reconstruction("cylinder support lost its surface"));
        }
    };
    let cone_surface = match topo.face(cone)?.surface() {
        FaceSurface::Cone(surface) => surface.clone(),
        // Symmetric cone re-check: only a cone carrier may rebuild here.
        FaceSurface::Plane { .. }
        | FaceSurface::Nurbs(_)
        | FaceSurface::Cylinder(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => {
            return Err(reconstruction("cone support lost its surface"));
        }
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
    // `ConicalSurface::half_angle` is measured from the radial plane, so its
    // tangent is axial/radial (not radial/axial).
    let sharp_height = sample_height.signum() * cylinder_surface.radius() * cone_slope;
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
    let cylinder_history = rebuild_closed_periodic_support(
        topo,
        cylinder,
        &cylinder_contacts,
        sharp_edge,
        sharp_vertex,
        cylinder_far_vertex,
        &cylinder_far,
        axis,
    )?;
    let cone_history = rebuild_closed_periodic_support(
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
    let live_edges: HashSet<_> = remus_topology::explorer::solid_edges(topo, sharp_solid)?
        .into_iter()
        .collect();
    let live_vertices: HashSet<_> = remus_topology::explorer::solid_vertices(topo, sharp_solid)?
        .into_iter()
        .collect();
    let mut replaced_edges = HashMap::new();
    let mut replaced_vertices = HashMap::new();
    for &contact in cylinder_contacts.iter().chain(&cone_contacts) {
        replaced_edges.insert(contact, sharp_edge);
    }
    for (seam, source) in [cylinder_history, cone_history] {
        if let Some((old_seam, old_contact)) = source {
            replaced_edges.insert(old_seam, seam);
            replaced_vertices.insert(old_contact, sharp_vertex);
        }
    }
    let mut boundary_history = Vec::new();
    for (source, copied) in copied_entities.edge_map {
        let target = replaced_edges.get(&copied).copied().unwrap_or(copied);
        boundary_history.push((
            EntityKey::edge(source),
            live_edges
                .contains(&target)
                .then_some(EntityKey::edge(target.index())),
        ));
    }
    for (source, copied) in copied_entities.vertex_map {
        let target = replaced_vertices.get(&copied).copied().unwrap_or(copied);
        boundary_history.push((
            EntityKey::vertex(source),
            live_vertices
                .contains(&target)
                .then_some(EntityKey::vertex(target.index())),
        ));
    }
    Ok(SharpResult {
        solid: sharp_solid,
        edges: vec![sharp_edge],
        face_map,
        boundary_history: Some(boundary_history),
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
        return Err(reconstruction(format!(
            "shrinking the radius from {old_radius} to {new_radius} mm increased the blend's volume effect from {old_magnitude} to {new_magnitude}"
        )));
    }
    if new_radius > old_radius
        && new_magnitude < old_magnitude
        && !tol.approx_eq(new_magnitude, old_magnitude)
    {
        return Err(reconstruction(format!(
            "growing the radius from {old_radius} to {new_radius} mm decreased the blend's volume effect from {old_magnitude} to {new_magnitude}"
        )));
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
        // Dispatches on the typed error enum, not on `EdgeCurve`/`FaceSurface`:
        // every non-resize-blend error maps to the generic code. Spelled per
        // variant so a new `OperationsError` variant is a compile error here
        // rather than an inherited string.
        OperationsError::ExactOnlyUnattainable
        | OperationsError::InvalidInput { .. }
        | OperationsError::NonManifoldResult
        | OperationsError::EmptyResult { .. }
        | OperationsError::Unsupported { .. }
        | OperationsError::BodyClassMeasureMismatch { .. }
        | OperationsError::BodyClassOperationUnsupported { .. }
        | OperationsError::BodyValidationFailed { .. }
        | OperationsError::HealingValidationFailed { .. }
        | OperationsError::HealingVerificationUnavailable { .. }
        | OperationsError::HealingRepairRefused { .. }
        | OperationsError::ConfiguredHealingValidationFailed { .. }
        | OperationsError::ConfiguredHealingVerificationUnavailable { .. }
        | OperationsError::PatternInstancesOverlap { .. }
        | OperationsError::Topology(_)
        | OperationsError::Math(_)
        | OperationsError::Algo(_)
        | OperationsError::Blend(_)
        | OperationsError::Check(_)
        | OperationsError::Geometry(_)
        | OperationsError::Heal(_)
        | OperationsError::Offset(_)
        | OperationsError::PartialResult { .. } => "resize-blend-failed",
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

    use super::*;

    #[test]
    fn recognized_group_refuses_mixed_radii_before_mutation() {
        use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};

        fn vertical_edge_at(topo: &Topology, solid: SolidId, x: f64, y: f64) -> EdgeId {
            solid_edges(topo, solid)
                .unwrap()
                .into_iter()
                .find(|&edge| {
                    let edge = topo.edge(edge).unwrap();
                    let start = topo.vertex(edge.start()).unwrap().point();
                    let end = topo.vertex(edge.end()).unwrap().point();
                    matches!(edge.curve(), EdgeCurve::Line)
                        && (start.x() - x).abs() <= Tolerance::new().linear
                        && (end.x() - x).abs() <= Tolerance::new().linear
                        && (start.y() - y).abs() <= Tolerance::new().linear
                        && (end.y() - y).abs() <= Tolerance::new().linear
                        && (start.z() - end.z()).abs() > 1.0
                })
                .unwrap()
        }

        let mut topo = Topology::new();
        let sharp = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let first_edge = vertical_edge_at(&topo, sharp, 0.0, 0.0);
        let first = fillet_v2(&mut topo, sharp, &[first_edge], 1.0)
            .unwrap()
            .solid;
        let second_edge = vertical_edge_at(&topo, first, 10.0, 10.0);
        let combined = fillet_v2(&mut topo, first, &[second_edge], 2.0)
            .unwrap()
            .solid;
        assert!(
            crate::validate::validate_solid(&topo, combined)
                .unwrap()
                .is_valid()
        );
        let mut bands: Vec<_> = solid_faces(&topo, combined)
            .unwrap()
            .into_iter()
            .filter_map(|face| match topo.face(face).unwrap().surface() {
                FaceSurface::Cylinder(cylinder)
                    if Tolerance::new().approx_eq(cylinder.radius(), 1.0)
                        || Tolerance::new().approx_eq(cylinder.radius(), 2.0) =>
                {
                    Some((cylinder.radius(), face))
                }
                FaceSurface::Plane { .. }
                | FaceSurface::Nurbs(_)
                | FaceSurface::Cylinder(_)
                | FaceSurface::Cone(_)
                | FaceSurface::Sphere(_)
                | FaceSurface::Torus(_) => None,
            })
            .collect();
        bands.sort_by(|left, right| left.0.total_cmp(&right.0));
        let [(first_radius, first_band), (second_radius, second_band)] = bands.as_slice() else {
            panic!("expected two retained distinct-radius bands, got {bands:?}");
        };
        assert!(Tolerance::new().approx_eq(*first_radius, 1.0));
        assert!(Tolerance::new().approx_eq(*second_radius, 2.0));
        let before_arena = (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
            topo.num_loops(),
            topo.num_coedges(),
            topo.num_pcurves(),
        );
        let before_entities = solid_entity_counts(&topo, combined).unwrap();

        let error = match remove_recognized_blend_faces(
            &mut topo,
            combined,
            &[*first_band, *second_band],
        ) {
            Err(error) => error,
            Ok(_) => panic!("mixed-radius recognized group unexpectedly succeeded"),
        };
        assert!(
            format!("{error}").contains("mixed-radius recognized blend groups are not qualified"),
            "unexpected refusal: {error}"
        );
        assert_eq!(
            (
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_wires(),
                topo.num_faces(),
                topo.num_shells(),
                topo.num_solids(),
                topo.num_loops(),
                topo.num_coedges(),
                topo.num_pcurves(),
            ),
            before_arena,
            "mixed-radius refusal must not allocate"
        );
        assert_eq!(
            solid_entity_counts(&topo, combined).unwrap(),
            before_entities,
            "mixed-radius refusal preserves the source topology"
        );
    }

    /// The remove/rebuild path is the primary blend-aware planar move; its
    /// caller silently falls back to the rigid translation path when it
    /// returns `Ok(None)`, so mutation testing found that disabling it
    /// outright went unnoticed by every test. Pin that it succeeds, on its
    /// own, for the canonical filleted box.
    #[test]
    fn remove_rebuild_moves_a_planar_support_through_its_box_blend() {
        use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};

        let mut topo = Topology::new();
        let sharp = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edge = solid_edges(&topo, sharp).unwrap()[0];
        let solid = fillet_v2(&mut topo, sharp, &[edge], 1.0).unwrap().solid;
        let band = solid_faces(&topo, solid)
            .unwrap()
            .into_iter()
            .find(|&face| {
                matches!(
                    topo.face(face).unwrap().surface(),
                    FaceSurface::Cylinder(cylinder)
                        if Tolerance::new().approx_eq(cylinder.radius(), 1.0)
                )
            })
            .unwrap();
        // One of the band's two tangent supports — not an end cap, which
        // shares an edge with the band but is not tangent to it and is
        // therefore an ordinary planar move.
        let support = describe_band(&topo, solid, band).unwrap().supports[0];
        assert!(
            matches!(
                topo.face(support).unwrap().surface(),
                FaceSurface::Plane { .. }
            ),
            "box blend supports are planar"
        );
        let source_counts = solid_entity_counts(&topo, solid).unwrap();
        // Closed forms: a 10-cube minus one r=1 edge fillet, then the same
        // fillet on an 11 × 10 × 10 block once the support has moved by 1.
        let fillet_removed = (1.0 - std::f64::consts::FRAC_PI_4) * 10.0;
        let source_volume = crate::measure::solid_volume(&topo, solid, 0.05).unwrap();
        assert!(
            (source_volume - (1000.0 - fillet_removed)).abs() < 1e-3 * 1000.0,
            "fixture volume {source_volume} is not a 10-cube minus one edge fillet"
        );

        let moved = move_planar_faces_with_blends_remove_rebuild(&mut topo, solid, &[support], 1.0)
            .unwrap()
            .unwrap();
        let translated =
            move_translation_invariant_blend_region(&mut topo, solid, &[support], 1.0).unwrap();
        for result in [&moved, &translated] {
            assert_eq!(
                result.boundary_pairs.len(),
                source_counts.1 + source_counts.2
            );
            let mapped: BTreeMap<_, _> = result.boundary_pairs.iter().copied().collect();
            for edge in solid_edges(&topo, solid).unwrap() {
                let source_edge = topo.edge(edge).unwrap();
                let target = mapped[&EntityKey::edge(edge.index())];
                let target_edge = topo
                    .edge(topo.edge_id_from_index(target.index).unwrap())
                    .unwrap();
                let expected: HashSet<_> = [source_edge.start(), source_edge.end()]
                    .into_iter()
                    .map(|vertex| mapped[&EntityKey::vertex(vertex.index())].index)
                    .collect();
                let actual: HashSet<_> = [target_edge.start().index(), target_edge.end().index()]
                    .into_iter()
                    .collect();
                assert_eq!(actual, expected);
            }
        }

        assert_eq!(
            solid_entity_counts(&topo, moved.solid).unwrap(),
            source_counts
        );
        assert!(moved.evolution.origin.is_exact());
        assert!(moved.evolution.is_complete());
        let moved_volume = crate::measure::solid_volume(&topo, moved.solid, 0.05).unwrap();
        let expected = 1100.0 - fillet_removed;
        assert!(
            (moved_volume - expected).abs() < 1e-3 * expected,
            "moving the support by 1 must lengthen the filleted block: {moved_volume} vs {expected}"
        );
        let translated_volume =
            crate::measure::solid_volume(&topo, translated.solid, 0.05).unwrap();
        assert!(
            (translated_volume - expected).abs() < 1e-3 * expected,
            "the rigid translation builds the same block: {translated_volume} vs {expected}"
        );
        let shrunk =
            move_translation_invariant_blend_region(&mut topo, solid, &[support], -0.5).unwrap();
        let shrunk_volume = crate::measure::solid_volume(&topo, shrunk.solid, 0.05).unwrap();
        let expected = 950.0 - fillet_removed;
        assert!(
            (shrunk_volume - expected).abs() < 1e-3 * expected,
            "moving the support by −0.5 must shorten the filleted block: {shrunk_volume} vs {expected}"
        );
    }

    /// Translate `faces` rigidly by `delta` the way the translation-invariant
    /// move does: their vertices, the edges joining two moved vertices, then
    /// their surfaces. Lines with one moved endpoint follow their vertex.
    fn translate_faces(topo: &mut Topology, solid: SolidId, faces: &[FaceId], delta: Vec3) {
        let mut vertices = HashSet::new();
        for &face in faces {
            vertices.extend(remus_topology::explorer::face_vertices(topo, face).unwrap());
        }
        let mut edges = HashSet::new();
        for edge in remus_topology::explorer::solid_edges(topo, solid).unwrap() {
            let data = topo.edge(edge).unwrap();
            if vertices.contains(&data.start()) && vertices.contains(&data.end()) {
                edges.insert(edge);
            }
        }
        for vertex in vertices {
            let point = topo.vertex(vertex).unwrap().point();
            topo.vertex_mut(vertex).unwrap().set_point(point + delta);
        }
        let matrix = remus_math::mat::Mat4::translation(delta.x(), delta.y(), delta.z());
        crate::transform::transform_edges(topo, &edges, &matrix).unwrap();
        for &face in faces {
            translate_face_surface(topo, face, delta).unwrap();
        }
    }

    fn face_with_outward_normal(topo: &Topology, solid: SolidId, axis: Vec3) -> FaceId {
        remus_topology::explorer::solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .find(|&face| {
                topo.face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|normal| normal.dot(axis) > 0.99)
            })
            .unwrap()
    }

    fn read_change(
        before: &Topology,
        after: &Topology,
        solid: SolidId,
        faces: &[FaceId],
        delta: Vec3,
    ) -> TranslatedFacesChange {
        translated_faces_volume_change(
            before,
            after,
            solid,
            &faces.iter().copied().collect(),
            delta,
        )
        .unwrap()
        .expect("every face of this fixture integrates")
    }

    /// The boundary-only shadow matches closed forms on a rectangle, a
    /// circle and a quarter-cylinder band, and ignores the move direction's
    /// tangential components.
    #[test]
    fn boundary_shadow_along_matches_closed_forms() {
        let mut topo = Topology::new();
        let bx = crate::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
        let lid = face_with_outward_normal(&topo, bx, Vec3::new(0.0, 0.0, 1.0));
        let shadow = boundary_shadow_along(&topo, lid, Vec3::new(0.0, 0.0, 0.5))
            .unwrap()
            .unwrap();
        assert!((shadow - 3.0).abs() < 1e-12, "lid shadow {shadow}");
        let sideways = boundary_shadow_along(&topo, lid, Vec3::new(1.0, 0.0, 0.0))
            .unwrap()
            .unwrap();
        assert!(
            sideways < 1e-12,
            "a lid casts no shadow along its own plane"
        );

        let cyl = crate::primitives::make_cylinder(&mut topo, 1.5, 4.0).unwrap();
        let cap = face_with_outward_normal(&topo, cyl, Vec3::new(0.0, 0.0, 1.0));
        let shadow = boundary_shadow_along(&topo, cap, Vec3::new(0.0, 0.0, 2.0))
            .unwrap()
            .unwrap();
        let expected = 2.0 * std::f64::consts::PI * 1.5 * 1.5;
        // Chord residual of the dense sampling on a full circle.
        assert!(
            (shadow - expected).abs() < 1e-5 * expected,
            "cap shadow {shadow} vs {expected}"
        );

        let (fixture, _solid, _support, band, direction) = filleted_cube_support_and_band();
        let shadow = boundary_shadow_along(&fixture, band, direction * 2.0)
            .unwrap()
            .unwrap();
        // The quarter band's shadow along the support normal is r × 10.
        assert!(
            (shadow - 20.0).abs() < 1e-5 * 20.0,
            "band shadow {shadow} vs 20"
        );

        // Far from the world origin the cross products about the origin would
        // cancel to nothing; about a loop point the shadow is still exact.
        let far = crate::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
        crate::transform::transform_solid(
            &mut topo,
            far,
            &remus_math::mat::Mat4::translation(1e8, -1e8, 5e7),
        )
        .unwrap();
        let far_lid = face_with_outward_normal(&topo, far, Vec3::new(0.0, 0.0, 1.0));
        let shadow = boundary_shadow_along(&topo, far_lid, Vec3::new(0.0, 0.0, 0.5))
            .unwrap()
            .unwrap();
        assert!(
            (shadow - 3.0).abs() < 1e-6,
            "lid shadow a hundred kilometres out {shadow}"
        );
    }

    /// Closed form: pulling the 2 × 3 lid of a box by d sweeps 6d, in either
    /// direction, and the closed-form box volume agrees. Lid and floor pulled
    /// together is the degenerate region the gate exists to refuse: its
    /// shadow along the move is zero.
    #[test]
    fn translated_faces_volume_change_is_the_swept_prism_on_a_box() {
        let mut before = Topology::new();
        let solid = crate::primitives::make_box(&mut before, 2.0, 3.0, 4.0).unwrap();
        let lid = face_with_outward_normal(&before, solid, Vec3::new(0.0, 0.0, 1.0));
        let floor = face_with_outward_normal(&before, solid, Vec3::new(0.0, 0.0, -1.0));
        for distance in [0.5, -0.25] {
            let mut after = before.clone();
            translate_faces(&mut after, solid, &[lid], Vec3::new(0.0, 0.0, distance));
            let read = read_change(
                &before,
                &after,
                solid,
                &[lid],
                Vec3::new(0.0, 0.0, distance),
            );
            assert!(
                (read.volume - 6.0 * distance).abs() < 1e-9,
                "lid moved by {distance}: change {}, expected {}",
                read.volume,
                6.0 * distance
            );
            assert!((read.area - 6.0).abs() < 1e-9, "lid area {}", read.area);
            let measured = crate::measure::solid_volume(&after, solid, 0.05).unwrap()
                - crate::measure::solid_volume(&before, solid, 0.05).unwrap();
            assert!(
                (read.volume - measured).abs() < 1e-6,
                "face integral {} disagrees with the measured volume {measured}",
                read.volume
            );
        }
        let mut after = before.clone();
        translate_faces(&mut after, solid, &[lid, floor], Vec3::new(0.0, 0.0, 0.5));
        let read = read_change(
            &before,
            &after,
            solid,
            &[lid, floor],
            Vec3::new(0.0, 0.0, 0.5),
        );
        assert!(
            read.volume.abs() < 1e-9 && (read.area - 12.0).abs() < 1e-9,
            "lid and floor together have no shadow: change {} area {}",
            read.volume,
            read.area
        );
        assert!(
            read.volume.abs() <= (read.area * 1e-9).mul_add(0.5, 1e-7),
            "the gate's slack must refuse the shadowless region"
        );
    }

    /// Closed form on a curved boundary: the r = 1.5 cap of a cylinder pulled
    /// along its axis by d sweeps π r² d, either way. The circle integrates
    /// exactly.
    #[test]
    fn translated_faces_volume_change_is_the_swept_prism_on_a_cylinder_cap() {
        let mut before = Topology::new();
        let solid = crate::primitives::make_cylinder(&mut before, 1.5, 4.0).unwrap();
        let cap = face_with_outward_normal(&before, solid, Vec3::new(0.0, 0.0, 1.0));
        for distance in [0.75, -0.4] {
            let mut after = before.clone();
            translate_faces(&mut after, solid, &[cap], Vec3::new(0.0, 0.0, distance));
            let read = read_change(
                &before,
                &after,
                solid,
                &[cap],
                Vec3::new(0.0, 0.0, distance),
            );
            let expected = std::f64::consts::PI * 1.5 * 1.5 * distance;
            assert!(
                (read.volume - expected).abs() < 1e-9 * expected.abs(),
                "cap moved by {distance}: change {}, expected {expected}",
                read.volume
            );
        }
    }

    /// Closed form with an inner wire: the top of a 6 × 6 × 3 plate with an
    /// r = 1 through-hole moved by d sweeps (36 − π) d. The hole wall is a
    /// carrier (axis along the move); the hole's circle integrates exactly as
    /// a subtracted loop.
    #[test]
    fn translated_faces_volume_change_subtracts_an_inner_wire() {
        let mut before = Topology::new();
        let plate = crate::primitives::make_box(&mut before, 6.0, 6.0, 3.0).unwrap();
        let drill = crate::primitives::make_cylinder(&mut before, 1.0, 5.0).unwrap();
        crate::transform::transform_solid(
            &mut before,
            drill,
            &remus_math::mat::Mat4::translation(3.0, 3.0, -1.0),
        )
        .unwrap();
        let solid =
            crate::boolean::boolean(&mut before, crate::boolean::BooleanOp::Cut, plate, drill)
                .unwrap();
        let top = face_with_outward_normal(&before, solid, Vec3::new(0.0, 0.0, 1.0));
        assert_eq!(
            before.face(top).unwrap().inner_wires().len(),
            1,
            "the drilled top carries the hole as an inner wire"
        );
        for distance in [0.5, -0.25] {
            let mut after = before.clone();
            translate_faces(&mut after, solid, &[top], Vec3::new(0.0, 0.0, distance));
            let read = read_change(
                &before,
                &after,
                solid,
                &[top],
                Vec3::new(0.0, 0.0, distance),
            );
            let expected = (36.0 - std::f64::consts::PI) * distance;
            assert!(
                (read.volume - expected).abs() < 1e-9 * expected.abs(),
                "drilled top moved by {distance}: change {}, expected {expected}",
                read.volume
            );
        }
    }

    /// Closed form on a reversed face: the ceiling of a 4-cube cavity inside a
    /// 10-cube faces into the cavity, so its outward normal is −Z. Moving it
    /// along that normal by d shrinks the void and grows the body by 16 d.
    #[test]
    fn translated_faces_volume_change_follows_a_cavity_ceiling() {
        let mut before = Topology::new();
        let block = crate::primitives::make_box(&mut before, 10.0, 10.0, 10.0).unwrap();
        let void = crate::primitives::make_box(&mut before, 4.0, 4.0, 4.0).unwrap();
        crate::transform::transform_solid(
            &mut before,
            void,
            &remus_math::mat::Mat4::translation(3.0, 3.0, 3.0),
        )
        .unwrap();
        let solid =
            crate::boolean::boolean(&mut before, crate::boolean::BooleanOp::Cut, block, void)
                .unwrap();
        assert_eq!(
            before.solid(solid).unwrap().inner_shells().len(),
            1,
            "the cut leaves one cavity shell"
        );
        let ceiling = remus_topology::explorer::solid_faces(&before, solid)
            .unwrap()
            .into_iter()
            .find(|&face| {
                let data = before.face(face).unwrap();
                data.effective_plane_normal()
                    .is_some_and(|normal| normal.z() < -0.99)
                    && remus_topology::explorer::face_vertices(&before, face)
                        .unwrap()
                        .iter()
                        .all(|&vertex| {
                            (before.vertex(vertex).unwrap().point().z() - 7.0).abs() < 1e-9
                        })
            })
            .expect("the cavity ceiling is the −Z plane at z = 7");
        for distance in [0.5, -0.25] {
            let mut after = before.clone();
            translate_faces(
                &mut after,
                solid,
                &[ceiling],
                Vec3::new(0.0, 0.0, -distance),
            );
            let read = read_change(
                &before,
                &after,
                solid,
                &[ceiling],
                Vec3::new(0.0, 0.0, -distance),
            );
            assert!(
                (read.volume - 16.0 * distance).abs() < 1e-9,
                "ceiling moved by {distance} into the void: change {}, expected {}",
                read.volume,
                16.0 * distance
            );
        }
    }

    /// The filleted 10-cube of the end-to-end test: its support plane (10 × 9
    /// after the r = 1 fillet) and quarter-cylinder band move together.
    fn filleted_cube_support_and_band() -> (Topology, SolidId, FaceId, FaceId, Vec3) {
        use remus_topology::explorer::{solid_edges, solid_faces};

        let mut topo = Topology::new();
        let sharp = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edge = solid_edges(&topo, sharp).unwrap()[0];
        let solid = fillet_v2(&mut topo, sharp, &[edge], 1.0).unwrap().solid;
        let band = solid_faces(&topo, solid)
            .unwrap()
            .into_iter()
            .find(|&face| {
                matches!(
                    topo.face(face).unwrap().surface(),
                    FaceSurface::Cylinder(cylinder)
                        if Tolerance::new().approx_eq(cylinder.radius(), 1.0)
                )
            })
            .unwrap();
        let support = describe_band(&topo, solid, band).unwrap().supports[0];
        let direction = topo
            .face(support)
            .unwrap()
            .effective_plane_normal()
            .unwrap();
        (topo, solid, support, band, direction)
    }

    /// Closed form with a translated blend face: the band's shadow along the
    /// move is r × 10, so the prism is (90 + 10) d — the same +100 the
    /// remove/rebuild path measures on this fixture — and the region's area is
    /// 90 + π/2 · 10.
    #[test]
    fn translated_faces_volume_change_counts_a_translated_blend_band() {
        let (before, solid, support, band, direction) = filleted_cube_support_and_band();
        for distance in [1.0, -0.5] {
            let mut after = before.clone();
            translate_faces(&mut after, solid, &[support, band], direction * distance);
            let read = read_change(
                &before,
                &after,
                solid,
                &[support, band],
                direction * distance,
            );
            let expected = 100.0 * distance;
            assert!(
                (read.volume - expected).abs() < 1e-6 * expected.abs(),
                "support and band moved by {distance}: change {}, expected {expected}",
                read.volume
            );
            let expected_area = 10.0f64.mul_add(std::f64::consts::FRAC_PI_2, 90.0);
            assert!(
                (read.area - expected_area).abs() < 1e-6 * expected_area,
                "region area {} vs {expected_area}",
                read.area
            );
        }
    }

    /// Two filleted top edges meeting at a corner give the moved region a
    /// vertex patch (sphere or torus) besides the two bands. Lifting the whole
    /// top assembly by d builds a 10 × 10 × (10 + d) block with the same
    /// fillets, so the prism is exactly 100 d whatever the corner patch is —
    /// and a torus corner is the family the independent area check vets.
    #[test]
    fn translated_faces_volume_change_carries_a_fillet_corner_patch() {
        use remus_topology::explorer::{face_vertices, solid_edges, solid_faces};

        let mut before = Topology::new();
        let sharp = crate::primitives::make_box(&mut before, 10.0, 10.0, 10.0).unwrap();
        let top_edges: Vec<_> = solid_edges(&before, sharp)
            .unwrap()
            .into_iter()
            .filter(|&edge| {
                let data = before.edge(edge).unwrap();
                [data.start(), data.end()]
                    .iter()
                    .all(|&vertex| (before.vertex(vertex).unwrap().point().z() - 10.0).abs() < 1e-9)
            })
            .collect();
        let shared = |a: EdgeId, b: EdgeId| {
            let (ea, eb) = (before.edge(a).unwrap(), before.edge(b).unwrap());
            [ea.start(), ea.end()]
                .iter()
                .any(|v| *v == eb.start() || *v == eb.end())
        };
        let first = top_edges[0];
        let second = top_edges[1..]
            .iter()
            .copied()
            .find(|&edge| shared(first, edge))
            .expect("two top edges share a corner");
        let solid = fillet_v2(&mut before, sharp, &[first, second], 1.0)
            .unwrap()
            .solid;
        // The top assembly: every face whose vertices all sit within the
        // fillet radius of the top. The walls reach z = 0 and stay behind.
        let moved: Vec<FaceId> = solid_faces(&before, solid)
            .unwrap()
            .into_iter()
            .filter(|&face| {
                face_vertices(&before, face)
                    .unwrap()
                    .iter()
                    .all(|&vertex| before.vertex(vertex).unwrap().point().z() >= 9.0 - 1e-9)
            })
            .collect();
        assert!(
            moved.iter().any(|&face| matches!(
                before.face(face).unwrap().surface(),
                FaceSurface::Torus(_) | FaceSurface::Sphere(_)
            )),
            "the two fillets meet in a corner patch"
        );
        for distance in [0.5, -0.25] {
            let mut after = before.clone();
            translate_faces(&mut after, solid, &moved, Vec3::new(0.0, 0.0, distance));
            let read = read_change(
                &before,
                &after,
                solid,
                &moved,
                Vec3::new(0.0, 0.0, distance),
            );
            let expected = 100.0 * distance;
            assert!(
                (read.volume - expected).abs() < 1e-6 * expected.abs(),
                "top assembly lifted by {distance}: change {}, expected {expected}",
                read.volume
            );
        }
    }

    /// The production hot path: a translated NURBS support. The band's
    /// cylinder is replaced by its exact rational NURBS (trimmed by the same
    /// edges, projected onto the new surface), so the quadrature runs the
    /// NURBS arm with a non-rectangular trim before and after. Same closed
    /// form, and the same-trim self-check must accept the rigid translation.
    #[test]
    fn translated_faces_volume_change_reads_a_translated_nurbs_support() {
        let (mut before, solid, support, band, direction) = filleted_cube_support_and_band();
        let FaceSurface::Cylinder(cylinder) = before.face(band).unwrap().surface().clone() else {
            unreachable!("the band is a cylinder");
        };
        let axial: Vec<f64> = remus_topology::explorer::face_vertices(&before, band)
            .unwrap()
            .iter()
            .map(|&vertex| {
                (before.vertex(vertex).unwrap().point() - cylinder.origin()).dot(cylinder.axis())
            })
            .collect();
        let v_min = axial.iter().copied().fold(f64::INFINITY, f64::min) - 0.5;
        let v_max = axial.iter().copied().fold(f64::NEG_INFINITY, f64::max) + 0.5;
        let nurbs =
            remus_geometry::convert::surface_to_nurbs::cylinder_to_nurbs(&cylinder, (v_min, v_max))
                .unwrap();
        // Keep the face outward: flip its orientation flag if the NURBS
        // parameterization's normal points the other way than the cylinder's.
        let (ku, kv) = (nurbs.knots_u(), nurbs.knots_v());
        let (u_mid, v_mid) = (
            f64::midpoint(ku[0], ku[ku.len() - 1]),
            f64::midpoint(kv[0], kv[kv.len() - 1]),
        );
        let point = nurbs.evaluate(u_mid, v_mid);
        let radial = {
            let offset = point - cylinder.origin();
            offset - cylinder.axis() * offset.dot(cylinder.axis())
        };
        let flip = nurbs.normal(u_mid, v_mid).unwrap().dot(radial) < 0.0;
        {
            let face = before.face_mut(band).unwrap();
            face.set_surface(FaceSurface::Nurbs(nurbs));
            face.compose_orientation(flip);
        }
        for distance in [1.0, -0.5] {
            let mut after = before.clone();
            translate_faces(&mut after, solid, &[support, band], direction * distance);
            let read = read_change(
                &before,
                &after,
                solid,
                &[support, band],
                direction * distance,
            );
            let expected = 100.0 * distance;
            assert!(
                (read.volume - expected).abs() < 1e-6 * expected.abs(),
                "NURBS band moved by {distance}: change {}, expected {expected}",
                read.volume
            );
            let expected_area = 10.0f64.mul_add(std::f64::consts::FRAC_PI_2, 90.0);
            assert!(
                (read.area - expected_area).abs() < 1e-6 * expected_area,
                "region area {} vs {expected_area}",
                read.area
            );
        }
    }

    /// The filleted box with its blend band carried as the exact rational
    /// NURBS cylinder (the production hot path's carrier), outward-wound.
    fn filleted_cube_with_nurbs_band() -> (Topology, SolidId, FaceId, FaceId, Vec3) {
        let (mut topo, solid, support, band, direction) = filleted_cube_support_and_band();
        let FaceSurface::Cylinder(cylinder) = topo.face(band).unwrap().surface().clone() else {
            unreachable!("the band is a cylinder");
        };
        let axial: Vec<f64> = remus_topology::explorer::face_vertices(&topo, band)
            .unwrap()
            .iter()
            .map(|&vertex| {
                (topo.vertex(vertex).unwrap().point() - cylinder.origin()).dot(cylinder.axis())
            })
            .collect();
        let v_min = axial.iter().copied().fold(f64::INFINITY, f64::min) - 0.5;
        let v_max = axial.iter().copied().fold(f64::NEG_INFINITY, f64::max) + 0.5;
        let nurbs =
            remus_geometry::convert::surface_to_nurbs::cylinder_to_nurbs(&cylinder, (v_min, v_max))
                .unwrap();
        let (ku, kv) = (nurbs.knots_u(), nurbs.knots_v());
        let (u_mid, v_mid) = (
            f64::midpoint(ku[0], ku[ku.len() - 1]),
            f64::midpoint(kv[0], kv[kv.len() - 1]),
        );
        let point = nurbs.evaluate(u_mid, v_mid);
        let radial = {
            let offset = point - cylinder.origin();
            offset - cylinder.axis() * offset.dot(cylinder.axis())
        };
        let flip = nurbs.normal(u_mid, v_mid).unwrap().dot(radial) < 0.0;
        let face = topo.face_mut(band).unwrap();
        face.set_surface(FaceSurface::Nurbs(nurbs));
        face.compose_orientation(flip);
        (topo, solid, support, band, direction)
    }

    /// Turns both thread memos off when dropped, so a failing test cannot
    /// leave them on for another test on the same thread.
    struct MemosOff;

    impl MemosOff {
        fn new() -> Self {
            remus_check::properties::face_cache::set_thread_face_cache_limits(0, 0);
            crate::measure::set_thread_volume_memo_capacity(0);
            Self
        }
    }

    impl Drop for MemosOff {
        fn drop(&mut self) {
            remus_check::properties::face_cache::set_thread_face_cache_limits(0, 0);
            crate::measure::set_thread_volume_memo_capacity(0);
        }
    }

    /// With the face cache on and the source already probed, the swept
    /// prism is read from the cache — the "before" side re-expressed about
    /// the move's reference, the "after" side as a translation — including
    /// the NURBS band, and it agrees with the uncached reading to rounding.
    #[test]
    fn translated_faces_volume_change_reads_translations_from_the_face_cache() {
        use remus_check::properties::face_cache::{
            enable_thread_face_cache, thread_face_cache_stats,
        };
        let _off = MemosOff::new();
        let (before, solid, support, band, direction) = filleted_cube_with_nurbs_band();
        for distance in [1.0, -0.5] {
            let mut after = before.clone();
            translate_faces(&mut after, solid, &[support, band], direction * distance);
            let uncached = read_change(
                &before,
                &after,
                solid,
                &[support, band],
                direction * distance,
            );

            enable_thread_face_cache();
            // The source's own strict validation primes the cache, about
            // its shell reference.
            let report = crate::validate::validate_solid(&before, solid).unwrap();
            assert!(report.is_valid());
            let primed = thread_face_cache_stats();
            let cached = read_change(
                &before,
                &after,
                solid,
                &[support, band],
                direction * distance,
            );
            let stats = thread_face_cache_stats();
            assert_eq!(stats.misses, primed.misses, "every reading is a hit");
            assert_eq!(stats.translated - primed.translated, 2, "support and band");
            assert!(
                (cached.volume - uncached.volume).abs() <= uncached.volume.abs() * 1e-12,
                "moved by {distance}: cached {} vs uncached {}",
                cached.volume,
                uncached.volume
            );
            assert!((cached.area - uncached.area).abs() <= uncached.area * 1e-12);
            remus_check::properties::face_cache::set_thread_face_cache_limits(0, 0);
        }
    }

    /// The remove/rebuild path measures its result with `solid_volume`, so
    /// with the memo on that reading is what the next request — at any
    /// deflection clamping to the same mesh — returns, bit for bit.
    #[test]
    fn remove_rebuild_leaves_its_measured_result_volume() {
        use crate::measure::{
            enable_thread_volume_memo, set_thread_volume_memo_capacity, solid_volume,
            thread_volume_memo_stats,
        };
        let _off = MemosOff::new();
        let (mut topo, solid, support, _, _) = filleted_cube_support_and_band();
        enable_thread_volume_memo();
        let moved = move_planar_faces_with_blends_remove_rebuild(&mut topo, solid, &[support], 1.0)
            .unwrap()
            .unwrap();
        let stats = thread_volume_memo_stats();
        let read = solid_volume(&topo, moved.solid, 0.08).unwrap();
        assert_eq!(thread_volume_memo_stats().hits, stats.hits + 1);
        set_thread_volume_memo_capacity(0);
        let fresh = solid_volume(&topo, moved.solid, 0.08).unwrap();
        assert_eq!(read.to_bits(), fresh.to_bits());
    }

    #[test]
    fn boundary_correspondence_requires_a_total_face_bijection() {
        let mut topo = Topology::new();
        let source = crate::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
        let faces = remus_topology::explorer::solid_faces(&topo, source).unwrap();
        let mut map: BTreeMap<_, _> = faces.iter().map(|&face| (face.index(), face)).collect();
        assert_eq!(
            construction_boundary_pairs(&topo, source, source, &map)
                .unwrap()
                .len(),
            20
        );
        map.remove(&faces[0].index());
        assert!(
            construction_boundary_pairs(&topo, source, source, &map)
                .unwrap()
                .is_empty()
        );
        map.insert(faces[0].index(), faces[1]);
        assert!(
            construction_boundary_pairs(&topo, source, source, &map)
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn boundary_correspondence_refuses_ambiguous_periodic_edges() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_torus(&mut topo, 5.0, 1.0, 32).unwrap();
        let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
        let map = faces.into_iter().map(|face| (face.index(), face)).collect();
        assert!(
            construction_boundary_pairs(&topo, solid, solid, &map)
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn conservative_move_lineage_refuses_duplicate_construction_claims() {
        use remus_topology::explorer::solid_faces;

        let mut topo = Topology::new();
        let source = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let result = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let source_faces = solid_faces(&topo, source).unwrap();
        let result_faces = solid_faces(&topo, result).unwrap();
        let mut construction = HashMap::new();
        construction.insert(source_faces[1].index(), result_faces[0]);
        construction.insert(source_faces[0].index(), result_faces[0]);
        for index in 2..source_faces.len() {
            construction.insert(source_faces[index].index(), result_faces[index]);
        }

        let evolution = conservative_move_evolution(&source_faces, &result_faces, &construction);
        let uncertain = vec![source_faces[0].index(), source_faces[1].index()];
        assert_eq!(evolution.unresolved[&result_faces[0].index()], uncertain);
        assert_eq!(evolution.unresolved[&result_faces[1].index()], uncertain);
        assert!(!evolution.modified.contains_key(&source_faces[0].index()));
        assert!(!evolution.modified.contains_key(&source_faces[1].index()));
        assert!(!evolution.is_complete());
        assert!(evolution.origin.is_exact());
    }

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

    // ---- B19 survivor tranche: split spring-chain proof, contiguity, and
    // helper certificates.  The chain fixture is a band carrier x^2 + y^2 = 1
    // with the tangent support plane x = 1; every perturbation below moves a
    // vertex by a stated amount against exactly one clause of the proof.

    use remus_math::surfaces::CylindricalSurface;
    use remus_topology::face::Face;

    struct ChainFixture {
        topo: Topology,
        support: FaceId,
        band: FaceId,
        contacts: Vec<EdgeId>,
        chain_vertices: Vec<VertexId>,
    }

    fn line_edge(topo: &mut Topology, start: VertexId, end: VertexId) -> EdgeId {
        topo.add_edge(Edge::new(start, end, EdgeCurve::Line))
    }

    /// Band/support pair sharing the open chain through `points`.  The
    /// support wire runs the chain forward; the band wire runs it backward.
    fn chain_fixture(points: &[Point3]) -> ChainFixture {
        let mut topo = Topology::new();
        let vertices: Vec<_> = points
            .iter()
            .map(|point| topo.add_vertex(Vertex::new(*point, Tolerance::new().linear)))
            .collect();
        let contacts: Vec<_> = vertices
            .windows(2)
            .map(|pair| line_edge(&mut topo, pair[0], pair[1]))
            .collect();
        let (first, last) = (vertices[0], *vertices.last().unwrap());
        let off_support = topo.add_vertex(Vertex::new(
            Point3::new(1.0, 5.0, 5.0),
            Tolerance::new().linear,
        ));
        let off_band = topo.add_vertex(Vertex::new(
            Point3::new(0.0, 1.0, 5.0),
            Tolerance::new().linear,
        ));
        let mut support_uses: Vec<_> = contacts
            .iter()
            .map(|edge| OrientedEdge::new(*edge, true))
            .collect();
        let closing = [
            line_edge(&mut topo, last, off_support),
            line_edge(&mut topo, off_support, first),
        ];
        support_uses.extend(closing.map(|edge| OrientedEdge::new(edge, true)));
        let mut band_uses: Vec<_> = contacts
            .iter()
            .rev()
            .map(|edge| OrientedEdge::new(*edge, false))
            .collect();
        let closing = [
            line_edge(&mut topo, first, off_band),
            line_edge(&mut topo, off_band, last),
        ];
        band_uses.extend(closing.map(|edge| OrientedEdge::new(edge, true)));
        let support_wire = topo.add_wire(Wire::new(support_uses, true).unwrap());
        let band_wire = topo.add_wire(Wire::new(band_uses, true).unwrap());
        let support = topo.add_face(Face::new(
            support_wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(1.0, 0.0, 0.0),
                d: 1.0,
            },
        ));
        let band = topo.add_face(Face::new(
            band_wire,
            Vec::new(),
            FaceSurface::Cylinder(
                CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0)
                    .unwrap(),
            ),
        ));
        ChainFixture {
            topo,
            support,
            band,
            contacts,
            chain_vertices: vertices,
        }
    }

    fn prove(fixture: &ChainFixture) -> Result<Option<SpringChain>, OperationsError> {
        prove_spring_chain(
            &fixture.topo,
            fixture.support,
            fixture.band,
            fixture.contacts.clone(),
        )
    }

    fn generatrix(z: &[f64]) -> Vec<Point3> {
        z.iter().map(|z| Point3::new(1.0, 0.0, *z)).collect()
    }

    #[test]
    fn split_generatrix_chain_is_proved_in_walk_order() {
        let fixture = chain_fixture(&generatrix(&[0.0, 4.0, 10.0]));
        let chain = prove(&fixture)
            .unwrap()
            .expect("collinear split generatrix");
        assert_eq!(chain.edges, fixture.contacts);
        assert_eq!(
            chain.endpoints,
            [fixture.chain_vertices[0], fixture.chain_vertices[2]]
        );
        assert_eq!(chain.interior_vertices, vec![fixture.chain_vertices[1]]);

        // A split vertex 5e-8 off the line (inside the 1e-7 modeling
        // tolerance) is still the same generatrix.
        let mut points = generatrix(&[0.0, 4.0, 10.0]);
        points[1] = Point3::new(1.0, 5e-8, 4.0);
        assert!(prove(&chain_fixture(&points)).unwrap().is_some());
    }

    #[test]
    fn split_chain_proof_refuses_each_violated_clause() {
        let refuses = |points: &[Point3], clause: &str| {
            let error = match prove(&chain_fixture(points)) {
                Err(error) => error,
                Ok(chain) => panic!(
                    "{clause}: expected a typed refusal, got {:?}",
                    chain.map(|c| c.edges)
                ),
            };
            assert!(
                error.to_string().contains(clause),
                "expected '{clause}', got {error}"
            );
        };
        // In-plane drift of the split vertex by 1e-5: still on the support
        // plane and within 5e-11 of the carrier, but off the common line.
        let mut points = generatrix(&[0.0, 4.0, 10.0]);
        points[1] = Point3::new(1.0, 1e-5, 4.0);
        refuses(&points, "not collinear");
        // The whole chain turned 0.01 rad about the band axis: a collinear
        // generatrix on the carrier, 5e-5 off the support plane.
        let (sin, cos) = (0.01_f64).sin_cos();
        let turned: Vec<_> = [0.0, 4.0, 10.0]
            .iter()
            .map(|z| Point3::new(cos, sin, *z))
            .collect();
        refuses(&turned, "leaves its planar support");
        // The whole chain slid 1e-3 along the support plane: collinear,
        // parallel to the axis, on the plane, but 5e-7 off the carrier.
        let slid: Vec<_> = [0.0, 4.0, 10.0]
            .iter()
            .map(|z| Point3::new(1.0, 1e-3, *z))
            .collect();
        refuses(&slid, "leaves the cylindrical band carrier");
        // A zero-length middle segment (two vertices at z = 4) overlaps.
        refuses(
            &generatrix(&[0.0, 4.0, 4.0, 10.0]),
            "backtracks or overlaps",
        );
    }

    #[test]
    fn single_contact_outside_the_proof_declines_to_the_fallback() {
        // One contact tilted 1e-5 rad inside the support plane: every vertex
        // is on the plane and within 5e-9 of the carrier, but the line is not
        // a generatrix.  A single edge declines (`Ok(None)`) rather than
        // raising the split-chain refusal.
        let fixture = chain_fixture(&[Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1e-4, 10.0)]);
        assert!(prove(&fixture).unwrap().is_none());
    }

    fn declined_surgical_fixture(offset: f64) -> (Topology, SolidId, FaceId, BandDescription) {
        // A single spring tilted within the carrier's positional tolerance
        // declines the generatrix proof without requiring a large fixture.
        let (mut topo, solid) = crate::test_helpers::blended_box(&[0]);
        let face = crate::test_helpers::cylinder_faces(&topo, solid)[0];
        let band = describe_band(&topo, solid, face).unwrap();
        let support = band.supports[0];
        let spring = shared_edges(&topo, solid, support, face).unwrap()[0];
        let vertex = topo.edge(spring).unwrap().end();
        let FaceSurface::Cylinder(cylinder) = topo.face(face).unwrap().surface() else {
            unreachable!();
        };
        let normal = topo
            .face(support)
            .unwrap()
            .effective_plane_normal()
            .unwrap();
        let tangent = normal.cross(cylinder.axis()).normalize().unwrap();
        let point = topo.vertex(vertex).unwrap().point();
        topo.vertex_mut(vertex)
            .unwrap()
            .set_point(point + tangent * offset);
        assert!(
            prove_spring_chain(&topo, support, face, vec![spring])
                .unwrap()
                .is_none()
        );
        (topo, solid, face, band)
    }

    #[test]
    fn declined_surgical_attempts_do_not_allocate_caller_slots() {
        for offset in [-1e-4, 1e-4] {
            let (mut topo, solid, _, band) = declined_surgical_fixture(offset);
            let slots = topo.allocated_slot_count();
            let ticks = topo.mutation_ticks();
            let journal = format!("{:?}", topo.journal());
            for _ in 0..3 {
                assert!(
                    heal_cylinder_plane_band_surgical(&mut topo, solid, &band)
                        .unwrap()
                        .is_none()
                );
                assert_eq!(topo.allocated_slot_count(), slots);
                assert_eq!(topo.mutation_ticks(), ticks);
                assert_eq!(format!("{:?}", topo.journal()), journal);
            }
        }
    }

    #[test]
    fn declined_surgical_public_fallbacks_allocate_only_their_results() {
        for fraction in [0.25, 0.75] {
            for resize in [false, true] {
                let (mut topo, solid) = crate::test_helpers::blended_box(&[0]);
                let face = crate::test_helpers::cylinder_faces(&topo, solid)[0];
                // Split one exact end arc without changing any geometry.
                // The strip has three cross edges, outside surgical scope,
                // but the positional fallback still reconstructs the box.
                let cross = face_edges(&topo, face)
                    .unwrap()
                    .into_iter()
                    .find(|edge| matches!(topo.edge(*edge).unwrap().curve(), EdgeCurve::Circle(_)))
                    .unwrap();
                let edge = topo.edge(cross).unwrap().clone();
                let (start, end) = edge.strict_domain().unwrap();
                let middle_parameter = start + (end - start) * fraction;
                let EdgeCurve::Circle(circle) = edge.curve() else {
                    unreachable!()
                };
                let middle = topo.add_vertex(Vertex::new(
                    circle.evaluate(middle_parameter),
                    Tolerance::new().linear,
                ));
                let mut first = Edge::new(edge.start(), middle, edge.curve().clone());
                first.set_trim(Some((start, middle_parameter)));
                let mut second = Edge::new(middle, edge.end(), edge.curve().clone());
                second.set_trim(Some((middle_parameter, end)));
                let halves = [topo.add_edge(first), topo.add_edge(second)];
                let mut wires = HashSet::new();
                for face in remus_topology::explorer::solid_faces(&topo, solid).unwrap() {
                    wires.insert(topo.face(face).unwrap().outer_wire());
                }
                for wire in wires {
                    let old = topo.wire(wire).unwrap();
                    let mut sequence = Vec::new();
                    for oriented in old.edges() {
                        if oriented.edge() != cross {
                            sequence.push(*oriented);
                        } else if oriented.is_forward() {
                            sequence.extend(halves.map(|edge| OrientedEdge::new(edge, true)));
                        } else {
                            sequence.extend(
                                halves
                                    .into_iter()
                                    .rev()
                                    .map(|edge| OrientedEdge::new(edge, false)),
                            );
                        }
                    }
                    topo.replace_boundary_wire(wire, Wire::new(sequence, old.is_closed()).unwrap())
                        .unwrap();
                }
                let band = describe_band(&topo, solid, face).unwrap();
                validate_exact_result(&topo, solid, "split end-arc input").unwrap();
                let mut control = topo.clone();
                let source_volume = crate::measure::solid_volume(&topo, solid, 0.01).unwrap();
                for _ in 0..3 {
                    let before = topo.allocated_slot_count();
                    let control_before = control.allocated_slot_count();
                    let expected =
                        crate::defeature::defeature_blend_band(&mut control, solid, &band.faces)
                            .unwrap();
                    let result = if resize {
                        resize_blend(&mut topo, solid, face, 1.0, 0.0)
                            .unwrap()
                            .solid
                    } else {
                        crate::defeature::defeature(&mut topo, solid, &[face]).unwrap()
                    };
                    assert_eq!(
                        topo.allocated_slot_count() - before,
                        control.allocated_slot_count() - control_before
                    );
                    validate_exact_result(&topo, result, "fallback regression").unwrap();
                    let actual_volume = crate::measure::solid_volume(&topo, result, 0.01).unwrap();
                    let expected_volume =
                        crate::measure::solid_volume(&control, expected.solid, 0.01).unwrap();
                    assert!((actual_volume - expected_volume).abs() < 1e-9);
                    assert!((actual_volume - 1000.0).abs() < 1e-6);
                    assert_eq!(
                        crate::measure::solid_volume(&topo, solid, 0.01)
                            .unwrap()
                            .to_bits(),
                        source_volume.to_bits()
                    );
                }
            }
        }
    }

    #[test]
    fn chain_must_be_the_same_contiguous_run_on_both_faces() {
        // The band carries a parallel duplicate of the second chain edge
        // (same endpoints, different edge), in either traversal direction.
        for reverse in [false, true] {
            let mut fixture = chain_fixture(&generatrix(&[0.0, 4.0, 10.0]));
            let [a, m, b] = fixture.chain_vertices[..] else {
                unreachable!("three chain vertices");
            };
            let duplicate = line_edge(&mut fixture.topo, m, b);
            let side = fixture.topo.add_vertex(Vertex::new(
                Point3::new(0.0, 1.0, 5.0),
                Tolerance::new().linear,
            ));
            let uses = if reverse {
                vec![
                    OrientedEdge::new(duplicate, false),
                    OrientedEdge::new(fixture.contacts[0], false),
                    OrientedEdge::new(line_edge(&mut fixture.topo, a, side), true),
                    OrientedEdge::new(line_edge(&mut fixture.topo, side, b), true),
                ]
            } else {
                vec![
                    OrientedEdge::new(fixture.contacts[0], true),
                    OrientedEdge::new(duplicate, true),
                    OrientedEdge::new(line_edge(&mut fixture.topo, b, side), true),
                    OrientedEdge::new(line_edge(&mut fixture.topo, side, a), true),
                ]
            };
            let wire = fixture.topo.add_wire(Wire::new(uses, true).unwrap());
            fixture
                .topo
                .set_face_boundary_wires(fixture.band, wire, Vec::new())
                .unwrap();
            assert!(
                !face_contains_contiguous_chain(
                    &fixture.topo,
                    fixture.band,
                    &fixture.contacts,
                    &fixture.chain_vertices
                )
                .unwrap()
            );
            assert!(
                face_contains_contiguous_chain(
                    &fixture.topo,
                    fixture.support,
                    &fixture.contacts,
                    &fixture.chain_vertices
                )
                .unwrap()
            );
            let Err(error) = prove(&fixture) else {
                panic!("band lacks the chain: expected a typed refusal");
            };
            assert!(
                error.to_string().contains("contiguous boundary run"),
                "{error}"
            );
        }
    }

    #[test]
    fn point_line_distance_matches_the_perpendicular_distance() {
        let a = Point3::new(1.0, 2.0, 3.0);
        let b = Point3::new(1.0, 2.0, 13.0);
        for (offset, expected) in [(3e-5, 3e-5), (0.0, 0.0), (2.5, 2.5)] {
            let point = Point3::new(1.0 + offset, 2.0, 7.0);
            assert!((point_line_distance(point, a, b) - expected).abs() < 1e-15);
        }
    }

    #[test]
    fn closed_circle_certificate_checks_seam_and_vertex_tolerance() {
        let circle = || {
            Circle3D::new_with_ref(
                Point3::new(0.0, 0.0, 2.0),
                Vec3::new(0.0, 0.0, 1.0),
                3.0,
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap()
        };
        let seam = Point3::new(3.0, 0.0, 2.0);
        let attempt = |point: Point3, tolerance: f64| {
            let mut topo = Topology::new();
            let vertex = topo.add_vertex(Vertex::new(point, tolerance));
            add_certified_closed_circle_edge(&mut topo, vertex, circle())
        };
        // A zero vertex tolerance is valid; the kernel default applies.
        assert!(attempt(seam, 0.0).is_ok());
        assert!(attempt(seam + Vec3::new(0.0, 0.0, 5e-8), 0.0).is_ok());
        // Negative or non-finite vertex tolerances are malformed.
        assert!(attempt(seam, -1.0).is_err());
        assert!(attempt(seam, f64::NAN).is_err());
        // A seam vertex 1e-3 off the circle is not the recovered sharp circle.
        assert!(attempt(seam + Vec3::new(0.0, 0.0, 1e-3), 0.0).is_err());
    }

    #[test]
    fn line_solvers_are_invariant_to_direction_length() {
        // Oblique, non-unit directions whose true crossing is known.
        let crossing = Point3::new(2.0, -1.0, 4.0);
        let da = Vec3::new(3.0, 1.0, -2.0);
        let db = Vec3::new(-0.5, 2.0, 0.25);
        let hit = line_line_intersection(crossing - da * 1.7, da, crossing + db * 0.6, db)
            .expect("transverse lines");
        assert!((hit - crossing).length() < 1e-12, "{hit:?}");

        let normal = Vec3::new(2.0, 3.0, 6.0) * (1.0 / 7.0);
        let d = normal.dot(Vec3::new(crossing.x(), crossing.y(), crossing.z()));
        let direction = Vec3::new(0.3, -2.0, 5.0);
        let hit = line_plane_intersection(crossing + direction * 2.3, direction, normal, d)
            .expect("transverse line");
        assert!((hit - crossing).length() < 1e-12, "{hit:?}");
    }

    #[test]
    fn unit_plane_of_normalizes_the_stored_equation() {
        let mut topo = Topology::new();
        let a = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 3.0), 1e-7));
        let b = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 3.0), 1e-7));
        let c = topo.add_vertex(Vertex::new(Point3::new(0.0, 1.0, 3.0), 1e-7));
        let uses = [(a, b), (b, c), (c, a)]
            .map(|(start, end)| OrientedEdge::new(line_edge(&mut topo, start, end), true))
            .to_vec();
        let wire = topo.add_wire(Wire::new(uses, true).unwrap());
        let face = Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 2.5),
                d: 7.5,
            },
        );
        let (normal, d) = unit_plane_of(&face).unwrap();
        assert!((normal - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-15);
        assert!((d - 3.0).abs() < 1e-15, "z = 3 has unit offset 3, got {d}");
    }

    #[test]
    fn orient_corners_requires_both_endpoints() {
        let mut topo = Topology::new();
        let a = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let b = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let edge = line_edge(&mut topo, a, b);
        let (pa, pb) = (Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let elsewhere = Point3::new(2.0, 0.0, 0.0);
        assert!(orient_corners(&topo, edge, pa, pb).unwrap().is_forward());
        assert!(!orient_corners(&topo, edge, pb, pa).unwrap().is_forward());
        // Exactly one endpoint matching, in either traversal, is not a span.
        for (from, to) in [
            (pa, elsewhere),
            (elsewhere, pb),
            (pb, elsewhere),
            (elsewhere, pa),
        ] {
            assert!(orient_corners(&topo, edge, from, to).is_err());
        }
    }

    #[test]
    fn multi_face_band_selection_keeps_the_established_defeature_logic() {
        // A cylindrical band split into two faces on one carrier is outside the
        // surgical scope: deleting one half must reach the established
        // defeature logic, not the single-face surgical selection rule.
        let (mut topo, blended) = crate::test_helpers::blended_box(&[0]);
        let (split, halves) = crate::test_helpers::split_band(&mut topo, blended);
        let error = crate::defeature::defeature(&mut topo, split, &[halves[0]])
            .expect_err("a half band has no exact sharp closure");
        assert!(
            !error
                .to_string()
                .contains("must contain exactly the complete analytic blend band"),
            "multi-face bands decline the surgical selection rule: {error}"
        );
    }

    /// How a band's exact cylinder-side contact circles are stored; every
    /// variant is the same point set traversed the same way by every face.
    #[derive(Clone, Copy, Debug)]
    enum CircleStorage {
        AsBuilt,
        /// Opposite stored normal with a decreasing trim.
        DecreasingTrim,
        /// The same arcs stored end-to-start on the opposite normal with an
        /// increasing trim; every face use flips.
        ReversedEdge,
    }

    fn restore_band_circles(
        topo: &mut Topology,
        solid: SolidId,
        band: FaceId,
        storage: CircleStorage,
    ) {
        use remus_topology::explorer::solid_faces;
        // Only the contact with the cylindrical support changes storage; its
        // sibling contact keeps the fillet's own convention, so the rebuilt
        // sharp edge must be oriented from each contact's actual travel.
        let adjacency = topo.build_adjacency(solid).unwrap();
        let circles: Vec<_> = face_edges(topo, band)
            .unwrap()
            .into_iter()
            .filter(|edge| {
                matches!(topo.edge(*edge).unwrap().curve(), EdgeCurve::Circle(_))
                    && adjacency.faces_for_edge(*edge).iter().any(|face| {
                        *face != band
                            && matches!(
                                topo.face(*face).unwrap().surface(),
                                FaceSurface::Cylinder(_)
                            )
                    })
            })
            .collect();
        assert!(!circles.is_empty());
        for edge_id in circles {
            let data = topo.edge(edge_id).unwrap();
            let (start, end, tolerance) = (data.start(), data.end(), data.tolerance());
            let EdgeCurve::Circle(circle) = data.curve().clone() else {
                unreachable!("filtered circles");
            };
            let (t0, t1) = data.strict_domain().unwrap();
            // flipped(-t) == circle(t)
            let flipped = EdgeCurve::Circle(
                Circle3D::new_with_ref(
                    circle.center(),
                    circle.normal() * -1.0,
                    circle.radius(),
                    circle.u_axis(),
                )
                .unwrap(),
            );
            let uses: Vec<_> = topo
                .pcurves_for_edge(edge_id)
                .into_iter()
                .map(|(face, forward, _)| (face, forward))
                .collect();
            match storage {
                CircleStorage::AsBuilt => {}
                CircleStorage::DecreasingTrim => {
                    for (face, forward) in uses {
                        topo.remove_pcurve_oriented(edge_id, face, forward).unwrap();
                    }
                    let edge = topo.edge_mut(edge_id).unwrap();
                    edge.set_curve(flipped);
                    edge.set_trim(Some((-t0, -t1)));
                }
                CircleStorage::ReversedEdge => {
                    // The same arc from `end` to `start`, increasing on the
                    // flipped circle; every face use flips.
                    let mut reversed = Edge::with_tolerance(end, start, flipped, tolerance);
                    reversed.set_trim(Some((-t1, -t0)));
                    let reversed = topo.add_edge(reversed);
                    for face in solid_faces(topo, solid).unwrap() {
                        let data = topo.face(face).unwrap();
                        let wires: Vec<_> = std::iter::once(data.outer_wire())
                            .chain(data.inner_wires().iter().copied())
                            .collect();
                        let mut rebuilt = Vec::new();
                        let mut touched = false;
                        for wire in wires {
                            let uses: Vec<_> = topo
                                .wire(wire)
                                .unwrap()
                                .edges()
                                .iter()
                                .map(|oriented| {
                                    if oriented.edge() == edge_id {
                                        touched = true;
                                        OrientedEdge::new(reversed, !oriented.is_forward())
                                    } else {
                                        *oriented
                                    }
                                })
                                .collect();
                            rebuilt.push(topo.add_wire(Wire::new(uses, true).unwrap()));
                        }
                        if touched {
                            topo.set_face_boundary_wires(face, rebuilt[0], rebuilt[1..].to_vec())
                                .unwrap();
                        }
                    }
                }
            }
        }
        let report = crate::validate::validate_solid(topo, solid).unwrap();
        assert!(report.is_valid(), "{storage:?} input: {:?}", report.issues);
    }

    fn torus_band(topo: &Topology, solid: SolidId) -> FaceId {
        remus_topology::explorer::solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .find(|face| matches!(topo.face(*face).unwrap().surface(), FaceSurface::Torus(_)))
            .unwrap()
    }

    fn assert_rim_removal(
        topo: &mut Topology,
        solid: SolidId,
        band: FaceId,
        radius: f64,
        sharp_volume: f64,
        what: String,
    ) {
        let result = resize_blend(topo, solid, band, radius, 0.0)
            .unwrap_or_else(|error| panic!("{what}: {error}"));
        let report = crate::validate::validate_solid(topo, result.solid).unwrap();
        assert!(report.is_valid(), "{what}: {:?}", report.issues);
        let volume = crate::measure::solid_volume(topo, result.solid, 0.001).unwrap();
        assert!(
            (volume - sharp_volume).abs() < sharp_volume * 1e-6,
            "{what}: volume {volume} vs sharp {sharp_volume}"
        );
    }

    #[test]
    fn plane_cylinder_rim_removal_is_independent_of_circle_storage() {
        use remus_topology::explorer::solid_edges;
        for storage in [
            CircleStorage::AsBuilt,
            CircleStorage::DecreasingTrim,
            CircleStorage::ReversedEdge,
        ] {
            let mut topo = Topology::new();
            let sharp = crate::primitives::make_cylinder(&mut topo, 10.0, 20.0).unwrap();
            let rim = solid_edges(&topo, sharp)
                .unwrap()
                .into_iter()
                .find(|edge| matches!(topo.edge(*edge).unwrap().curve(), EdgeCurve::Circle(_)))
                .unwrap();
            let solid = fillet_v2(&mut topo, sharp, &[rim], 2.0).unwrap().solid;
            let band = torus_band(&topo, solid);
            restore_band_circles(&mut topo, solid, band, storage);
            // Closed-form sharp body: the plain 10 x 20 cylinder.
            let sharp_volume = std::f64::consts::PI * 100.0 * 20.0;
            assert_rim_removal(
                &mut topo,
                solid,
                band,
                2.0,
                sharp_volume,
                format!("{storage:?}"),
            );
        }
    }

    #[test]
    fn cylinder_cone_rim_removal_is_independent_of_circle_storage() {
        use remus_topology::explorer::solid_edges;
        for storage in [
            CircleStorage::AsBuilt,
            CircleStorage::DecreasingTrim,
            CircleStorage::ReversedEdge,
        ] {
            let mut topo = Topology::new();
            // Radius-3 cylinder (height 5) fused to a 3 -> 1 cone of height 4:
            // the sharp shoulder volume is pi*9*5 + pi*4*(9 + 3 + 1)/3.
            let cylinder = crate::primitives::make_cylinder(&mut topo, 3.0, 5.0).unwrap();
            let cone = crate::primitives::make_cone(&mut topo, 3.0, 1.0, 4.0).unwrap();
            crate::transform::transform_solid(
                &mut topo,
                cone,
                &remus_math::mat::Mat4::translation(0.0, 0.0, 5.0),
            )
            .unwrap();
            let sharp =
                crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Fuse, cylinder, cone)
                    .unwrap();
            let adjacency = topo.build_adjacency(sharp).unwrap();
            let shoulder = solid_edges(&topo, sharp)
                .unwrap()
                .into_iter()
                .find(|edge| {
                    let faces = adjacency.faces_for_edge(*edge);
                    faces.len() == 2
                        && faces.iter().any(|face| {
                            matches!(topo.face(*face).unwrap().surface(), FaceSurface::Cone(_))
                        })
                        && faces.iter().any(|face| {
                            matches!(
                                topo.face(*face).unwrap().surface(),
                                FaceSurface::Cylinder(_)
                            )
                        })
                })
                .unwrap();
            let solid = fillet_v2(&mut topo, sharp, &[shoulder], 0.25)
                .unwrap()
                .solid;
            let band = torus_band(&topo, solid);
            restore_band_circles(&mut topo, solid, band, storage);
            let pi = std::f64::consts::PI;
            let sharp_volume = pi * 9.0 * 5.0 + pi * 4.0 * (9.0 + 3.0 + 1.0) / 3.0;
            assert_rim_removal(
                &mut topo,
                solid,
                band,
                0.25,
                sharp_volume,
                format!("{storage:?}"),
            );
        }
    }
}
