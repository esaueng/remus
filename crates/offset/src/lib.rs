//! # remus-offset
//!
//! Solid offset engine for remus.
//!
//! This is layer L2, depending on `remus-math`, `remus-topology`,
//! and `remus-geometry`.
//!
//! # Pipeline
//!
//! [`JointType::Intersection`], the default, extends adjacent offset faces
//! until they meet, through an 8-phase pipeline:
//!
//! 1. **Analyse** — classify edges as convex/concave/tangent, derive vertex
//!    classes.
//! 2. **Offset** — construct the offset surface for each face (translate
//!    planes, adjust cylinder radii, etc.).
//! 3. **Intersect 3D** — intersect adjacent offset faces in 3D to find new
//!    edge curves.
//! 4. **Intersect 2D** — intersect offset PCurves in parameter space to find
//!    edge split points.
//! 5. **Split edges** — split original edges at intersection parameters.
//! 6. **Build loops** — assemble trimmed edges into closed wire loops for each
//!    offset face.
//! 7. **Assemble** — build the final shell and solid from offset faces and
//!    wire loops.
//! 8. **Self-intersection removal** — detect and excise global
//!    self-intersections if enabled.
//!
//! # Joints
//!
//! [`JointType::Arc`] instead leaves each face at its own translated boundary
//! and fills the gaps with the surface a rolling ball sweeps — a cylindrical
//! patch along every convex edge and a spherical one at every convex vertex.
//! For a convex polyhedron that is exactly the Minkowski sum with a ball. It
//! is a different construction rather than a post-pass on the one above, so it
//! runs after phase 1 and returns. Anything outside that class — a curved
//! face, a concave or tangent edge, a face with a hole, a cavity, an excluded
//! face, or an inward distance — is refused rather than quietly mitred.
//!
//! # Cavities
//!
//! A solid may bound its volume with an outer shell plus inner shells, each
//! enclosing a void. Every shell is offset by the same signed distance along
//! its own outward normal — the direction pointing away from material — and
//! a cavity's outward normals point into the void. A positive distance
//! therefore grows the outer boundary and **shrinks** every cavity; a
//! negative distance does the reverse. The result keeps the same shell
//! partition, so a hollow part stays hollow.
//!
//! Whether the shells still avoid one another afterwards is a global
//! property this engine does not compute. A necessary condition — each
//! cavity's spatial extent staying strictly inside the outer shell's, before
//! and after — is checked, and a solid that fails it is refused rather than
//! offset into a well-formed answer with the wrong volume.

pub(crate) mod analyse;
pub(crate) mod arc_joint;
pub(crate) mod assemble;
pub(crate) mod cavity;
pub(crate) mod data;
pub mod error;
pub(crate) mod inter2d;
pub(crate) mod inter3d;
pub(crate) mod loops;
mod move_faces;
pub(crate) mod offset;
pub(crate) mod self_int;

pub use data::{JointType, OffsetOptions};
pub use error::OffsetError;
pub use move_faces::{MoveFacesResult, move_faces, move_faces_with_face_map};

use remus_math::det_hash::{DetHashMap, DetHashSet};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

use crate::data::OffsetData;

/// A solid offset and its construction-derived source-face correspondence.
#[derive(Debug)]
pub struct OffsetResult {
    /// Offset solid.
    pub solid: SolidId,
    /// Source face index to the one result face derived from it.
    pub face_map: DetHashMap<usize, FaceId>,
}

struct ThickSolidResult {
    solid: SolidId,
    face_map: Option<DetHashMap<usize, FaceId>>,
}

/// Offset all faces of a solid by the given signed distance.
///
/// Positive distance offsets outward (enlarges), negative inward (shrinks).
///
/// # Errors
///
/// Returns [`OffsetError`] if the offset collapses the solid, any
/// intersection fails, or the result cannot be assembled into a valid solid.
pub fn offset_solid(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
    options: OffsetOptions,
) -> Result<SolidId, OffsetError> {
    thick_solid(topo, solid, distance, &[], options)
}

/// Offset every face while retaining the exact source-to-result face map.
///
/// The default intersection-joint construction derives exactly one result
/// face from every source face. Arc joints and self-intersection removal can
/// synthesize or replace faces after that construction, so this entry point
/// refuses those options instead of returning stale or incomplete provenance.
///
/// # Errors
///
/// Returns the same errors as [`offset_solid`], or
/// [`OffsetError::InvalidInput`] when the requested options do not preserve
/// the one-to-one face contract. Any failure rolls the topology back.
pub fn offset_solid_with_face_map(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
    options: OffsetOptions,
) -> Result<OffsetResult, OffsetError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        if options.joint != JointType::Intersection {
            return Err(OffsetError::InvalidInput {
                reason: "face provenance is only available for intersection-joint offsets".into(),
            });
        }
        if options.remove_self_intersections {
            return Err(OffsetError::InvalidInput {
                reason: "face provenance is unavailable when self-intersection removal may replace faces"
                    .into(),
            });
        }

        let source_faces = solid_faces(topo, solid)?;
        let result = thick_solid_impl(topo, solid, distance, &[], options)?;
        let face_map = result.face_map.ok_or_else(|| OffsetError::AssemblyFailed {
            reason: "offset assembly did not retain source-face provenance".into(),
        })?;
        validate_face_map(topo, &source_faces, result.solid, &face_map)?;
        Ok(OffsetResult {
            solid: result.solid,
            face_map,
        })
    })
}

/// Offset a solid while excluding specific faces, producing a thick
/// (hollowed) solid.
///
/// Excluded faces are left at their original positions, and side walls
/// connect them to the offset faces.
///
/// # Errors
///
/// Returns [`OffsetError`] if the offset collapses the solid, any
/// intersection fails, or the result cannot be assembled into a valid solid.
#[allow(clippy::too_many_lines)]
pub fn thick_solid(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
    exclude: &[FaceId],
    options: OffsetOptions,
) -> Result<SolidId, OffsetError> {
    Ok(thick_solid_impl(topo, solid, distance, exclude, options)?.solid)
}

#[allow(clippy::too_many_lines)]
fn thick_solid_impl(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
    exclude: &[FaceId],
    options: OffsetOptions,
) -> Result<ThickSolidResult, OffsetError> {
    if !distance.is_finite() || distance.abs() < options.tolerance.linear {
        return Err(OffsetError::InvalidInput {
            reason: "offset distance must be non-zero and finite".into(),
        });
    }
    let has_cavities = !topo.solid(solid)?.inner_shells().is_empty();
    if has_cavities {
        if !exclude.is_empty() {
            return Err(OffsetError::InvalidInput {
                reason: "offset of solids with cavity shells cannot exclude faces: the wall \
                         builder that closes a thick solid's openings only knows the outer shell"
                    .into(),
            });
        }
        let scale = cavity::solid_extent_scale(topo, solid)?;
        let clearance = cavity::required_clearance(distance, scale, options.tolerance.linear);
        cavity::check_cavity_extents(topo, solid, clearance, cavity::Stage::Input)?;
        let clearance_floor = options.tolerance.linear.max(scale * 1e-9);
        cavity::check_cavity_survival(topo, solid, distance, clearance_floor)?;
    }

    let linear_tol = options.tolerance.linear;
    let mut data = OffsetData::new(distance, options, exclude.to_vec());

    analyse::analyse_edges(topo, solid, &mut data)?;

    if data.options.joint == JointType::Arc {
        // A rounded offset is not the mitred one with fillets bolted on: its
        // faces stop at their own translated boundary instead of being
        // extended until they meet, so it is built whole rather than patched
        // into the phases below.
        let result = arc_joint::build_arc_offset(topo, solid, distance, &data)?;
        validate_offset_result(topo, result)?;
        return Ok(ThickSolidResult {
            solid: result,
            face_map: None,
        });
    }

    offset::build_offset_faces(topo, solid, &mut data)?;

    inter3d::intersect_faces_3d(topo, solid, &mut data)?;

    inter2d::intersect_pcurves_2d(topo, solid, &mut data)?;

    // Edge splitting (phase 5) is integrated into inter2d for now.

    loops::build_wire_loops(topo, &mut data)?;

    let assembled = assemble::assemble_solid_with_face_map(topo, &data)?;
    let mut face_map = Some(assembled.face_map.into_iter().collect());

    let result = if data.options.remove_self_intersections {
        face_map = None;
        self_int::remove_self_intersections(topo, assembled.solid)?
    } else {
        assembled.solid
    };
    validate_offset_result(topo, result)?;
    if has_cavities {
        // The cavities moved; re-check that they still sit inside the outer
        // boundary rather than trusting the input check to have covered it.
        let scale = cavity::solid_extent_scale(topo, result)?;
        let clearance = linear_tol.max(scale * 1e-9);
        cavity::check_cavity_extents(topo, result, clearance, cavity::Stage::Result)?;
    }
    Ok(ThickSolidResult {
        solid: result,
        face_map,
    })
}

fn validate_face_map(
    topo: &Topology,
    source_faces: &[FaceId],
    result: SolidId,
    face_map: &DetHashMap<usize, FaceId>,
) -> Result<(), OffsetError> {
    let source_indices: DetHashSet<usize> =
        source_faces.iter().copied().map(FaceId::index).collect();
    let result_faces = solid_faces(topo, result)?;
    let result_indices: DetHashSet<usize> =
        result_faces.iter().copied().map(FaceId::index).collect();
    let mapped_indices: DetHashSet<usize> = face_map.values().copied().map(FaceId::index).collect();
    if face_map.len() != source_indices.len()
        || face_map.keys().copied().collect::<DetHashSet<_>>() != source_indices
        || mapped_indices.len() != face_map.len()
        || mapped_indices != result_indices
    {
        return Err(OffsetError::AssemblyFailed {
            reason: format!(
                "offset face provenance is not one-to-one ({} source faces, {} map entries, {} distinct mapped faces, {} result faces)",
                source_indices.len(),
                face_map.len(),
                mapped_indices.len(),
                result_indices.len()
            ),
        });
    }
    Ok(())
}

/// Every shell of the result must be a closed 2-manifold — the outer skin
/// and each cavity alike.
fn validate_offset_result(topo: &Topology, solid: SolidId) -> Result<(), OffsetError> {
    let solid_data = topo.solid(solid)?;
    for shell in
        std::iter::once(solid_data.outer_shell()).chain(solid_data.inner_shells().iter().copied())
    {
        remus_topology::validation::validate_shell_closed(topo.shell(shell)?, topo)?;
    }
    Ok(())
}
