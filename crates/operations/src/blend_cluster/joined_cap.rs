//! Group edits on a body whose rounded cap was lowered into its shoulder.
//!
//! A boss cap moved down past its shoulder with sampled permission
//! ([`crate::face_move_quality::move_faces_with_quality`]) keeps the shoulder
//! and trims it, and the corner patches beside it, by the cap plane; its rim
//! re-rolls over them, as exact tori on the boss and shoulder and as sampled
//! bands over the corner patches. Those rim bands roll on sampled supports,
//! so no closed form explains them and the group recognizer refuses the
//! whole group.
//!
//! The group is edited with the cap lifted clear of its shoulder instead: on
//! the private copy the cap is moved up by the same sampled route until its
//! rim is one full circle above the shoulder again (the move rebuilds the
//! shoulder, the corner patches and the rim as the body had them before the
//! cap came down), the ordinary group route edits the lifted body, and the
//! cap is lowered back by the same distance with the same permission. The
//! edit is thus the construction the body came from — the blend network,
//! the rim, the lowered cap — with the edited value, the planner's order
//! with the cap descent last.
//!
//! The permission is the one the cap was lowered with: its station count is
//! read off the body's own sampled rim bands, so the edit re-rolls the
//! sampled faces the earlier move made and adds none of another kind.
//! Planes, cylinders and tori stay analytic. Either move refusing, or the
//! lifted body carrying no recognized group, means the route does not apply
//! and the caller keeps its own refusal.
//!
//! WASM module tier: `core` (reached from `resizeBlend` and
//! `resizeCylindricalFace`).

use remus_blend::cap_reblend::{CapReblendApproximation, sampled_section_radius};
use remus_math::vec::Vec3;
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::explorer::{face_edges, face_vertices, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use super::reblend::compose_evolution;
use crate::OperationsError;
use crate::evolution::EvolutionMap;

/// Radius agreement (absolute, mm) between a joined cap's rim bands.
const RIM_RADIUS: f64 = 1e-7;
/// Normal agreement between the cap plane and its rim tori's axes.
const AXIS: f64 = 1e-9;

/// A joined cap lifted clear of its shoulder on a private copy.
#[derive(Clone, Debug)]
pub(super) struct CapLift {
    /// The cap on the lifted body.
    pub(super) cap: FaceId,
    /// Axial distance the cap was lifted by.
    pub(super) distance: f64,
    /// The sampled permission the joined cap was built with.
    pub(super) permission: CapReblendApproximation,
    /// Face evolution from the copied body to the lifted one.
    pub(super) evolution: EvolutionMap,
}

/// A planar cap whose rim rolls over sampled supports, with its rim radius,
/// the station count of its sampled rim bands, and how far it must rise for
/// the rim to clear every support it was cut into.
struct JoinedCap {
    cap: FaceId,
    stations: usize,
    lift: f64,
}

/// Lift the joined cap of `solid` (on a private `work` arena) clear of its
/// shoulder, returning the lifted solid, `seed`'s image on it and the lift.
///
/// `Ok(None)` when `solid` has no joined cap, the lift refuses, or `seed`
/// has no unique image on the lifted body.
pub(super) fn lift_joined_cap(
    work: &mut Topology,
    solid: SolidId,
    seed: FaceId,
) -> Result<Option<(SolidId, FaceId, CapLift)>, OperationsError> {
    let Some(joined) = joined_cap(work, solid)? else {
        return Ok(None);
    };
    let permission = CapReblendApproximation {
        stations: joined.stations,
    };
    let Ok(((moved, _), _)) = crate::face_move_quality::edit_with_quality(
        work,
        solid,
        &[joined.cap],
        joined.lift,
        Some(permission),
    ) else {
        return Ok(None);
    };
    let (Some(cap), Some(lifted_seed)) = (
        unique_image(&moved.evolution, joined.cap),
        unique_image(&moved.evolution, seed),
    ) else {
        return Ok(None);
    };
    let (Some(cap), Some(lifted_seed)) = (
        work.face_id_from_index(cap),
        work.face_id_from_index(lifted_seed),
    ) else {
        return Ok(None);
    };
    Ok(Some((
        moved.solid,
        lifted_seed,
        CapLift {
            cap,
            distance: joined.lift,
            permission,
            evolution: moved.evolution,
        },
    )))
}

/// Lower the lifted cap of an edited body back by the lift, with the same
/// permission. `evolution` runs from the lifted body to `solid`; the
/// returned one runs from the copied body (before the lift) to the lowered
/// solid.
pub(super) fn lower_joined_cap(
    work: &mut Topology,
    solid: SolidId,
    lift: &CapLift,
    evolution: &EvolutionMap,
) -> Result<(SolidId, EvolutionMap), OperationsError> {
    // The cap survives the group edit as a modified support of its rim.
    let cap = unique_image(evolution, lift.cap)
        .and_then(|index| work.face_id_from_index(index))
        .ok_or_else(|| super::BlendClusterError::ReblendFailed {
            reason: "the lifted cap has no unique image on the edited body".into(),
        })?;
    let ((lowered, _), _) = crate::face_move_quality::edit_with_quality(
        work,
        solid,
        &[cap],
        -lift.distance,
        Some(lift.permission),
    )
    .map_err(|error| super::BlendClusterError::ReblendFailed {
        reason: format!("the edited body's cap does not come back down into its shoulder: {error}"),
    })?;
    let edited = compose_evolution(&lift.evolution, evolution);
    Ok((
        lowered.solid,
        compose_evolution(&edited, &lowered.evolution),
    ))
}

/// The unique face `face` became under `evolution`, by index.
fn unique_image(evolution: &EvolutionMap, face: FaceId) -> Option<usize> {
    match evolution.modified.get(&face.index())?.as_slice() {
        [image] => Some(*image),
        _ => None,
    }
}

/// The joined cap of `solid`: a planar face whose every neighbour is a rim
/// band of one radius (axial tori and sampled circular-section bands), at
/// least one of them sampled.
fn joined_cap(topo: &Topology, solid: SolidId) -> Result<Option<JoinedCap>, OperationsError> {
    let adjacency = topo.build_adjacency(solid)?;
    let mut found = None;
    for face in solid_faces(topo, solid)? {
        let Some(normal) = topo.face(face)?.effective_plane_normal() else {
            continue;
        };
        if let Some(joined) = joined_at(topo, &adjacency, face, normal)? {
            if found.is_some() {
                // Two joined caps: which one the edit belongs to is not ours
                // to guess.
                return Ok(None);
            }
            found = Some(joined);
        }
    }
    Ok(found)
}

fn joined_at(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    cap: FaceId,
    normal: Vec3,
) -> Result<Option<JoinedCap>, OperationsError> {
    let mut rims: Vec<FaceId> = face_edges(topo, cap)?
        .into_iter()
        .flat_map(|edge| adjacency.faces_for_edge(edge).iter().copied())
        .filter(|&face| face != cap)
        .collect();
    rims.sort_unstable_by_key(|face| face.index());
    rims.dedup();
    if rims.len() < 2 {
        return Ok(None);
    }
    let mut radius: Option<f64> = None;
    let mut stations = None;
    for &rim in &rims {
        let data = topo.face(rim)?;
        let section = match data.surface() {
            FaceSurface::Torus(torus) if torus.z_axis().cross(normal).length() < AXIS => {
                torus.minor_radius()
            }
            FaceSurface::Nurbs(surface) => {
                let Ok(section) = sampled_section_radius(data) else {
                    return Ok(None);
                };
                let count = surface.control_points()[0].len();
                if stations.is_some_and(|known| known != count) {
                    return Ok(None);
                }
                stations = Some(count);
                section
            }
            FaceSurface::Plane { .. }
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => return Ok(None),
        };
        if radius.is_some_and(|known| (known - section).abs() > RIM_RADIUS) {
            return Ok(None);
        }
        radius = Some(section);
    }
    let (Some(radius), Some(stations)) = (radius, stations) else {
        return Ok(None);
    };
    if crate::face_move_quality::check_permission(Some(CapReblendApproximation { stations }))
        .is_err()
    {
        return Ok(None);
    }
    // Every support the rim was cut into must end below the rim's contact
    // plane once the cap has risen: a concave shoulder torus at its ring's
    // height, any other support at its highest vertex.
    let height = |point: remus_math::vec::Point3| {
        (point - remus_math::vec::Point3::new(0.0, 0.0, 0.0)).dot(normal)
    };
    let cap_height = face_vertices(topo, cap)?
        .first()
        .map(|&vertex| topo.vertex(vertex).map(|data| height(data.point())))
        .transpose()?
        .ok_or_else(|| OperationsError::InvalidInput {
            reason: "joined cap has no vertex".into(),
        })?;
    let mut top = f64::NEG_INFINITY;
    for &rim in &rims {
        for &support in adjacency.neighbors_of_face(rim) {
            if support == cap || rims.contains(&support) {
                continue;
            }
            let data = topo.face(support)?;
            let reach = match data.surface() {
                FaceSurface::Torus(torus) if torus.z_axis().cross(normal).length() < AXIS => {
                    height(torus.center()) + torus.minor_radius()
                }
                _ => face_vertices(topo, support)?
                    .into_iter()
                    .map(|vertex| topo.vertex(vertex).map(|data| height(data.point())))
                    .collect::<Result<Vec<_>, _>>()?
                    .into_iter()
                    .fold(f64::NEG_INFINITY, f64::max),
            };
            top = top.max(reach);
        }
    }
    if !top.is_finite() {
        return Ok(None);
    }
    // The cap rises until its rim's lower contact is a rim radius above
    // every support.
    let lift = top + 2.0 * radius - cap_height;
    if lift <= radius {
        return Ok(None);
    }
    Ok(Some(JoinedCap {
        cap,
        stations,
        lift,
    }))
}
