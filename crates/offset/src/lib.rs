//! # brepkit-offset
//!
//! Solid offset engine for brepkit.
//!
//! This is layer L2, depending on `brepkit-math`, `brepkit-topology`,
//! and `brepkit-geometry`.
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
pub(crate) mod offset;
pub(crate) mod self_int;

pub use data::{JointType, OffsetOptions};
pub use error::OffsetError;

use brepkit_topology::Topology;
use brepkit_topology::face::FaceId;
use brepkit_topology::solid::SolidId;

use crate::data::OffsetData;

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
        return Ok(result);
    }

    offset::build_offset_faces(topo, solid, &mut data)?;

    inter3d::intersect_faces_3d(topo, solid, &mut data)?;

    inter2d::intersect_pcurves_2d(topo, solid, &mut data)?;

    // Edge splitting (phase 5) is integrated into inter2d for now.

    loops::build_wire_loops(topo, &mut data)?;

    let result = assemble::assemble_solid(topo, &data)?;

    let result = if data.options.remove_self_intersections {
        self_int::remove_self_intersections(topo, result)?
    } else {
        result
    };
    validate_offset_result(topo, result)?;
    if has_cavities {
        // The cavities moved; re-check that they still sit inside the outer
        // boundary rather than trusting the input check to have covered it.
        let scale = cavity::solid_extent_scale(topo, result)?;
        let clearance = linear_tol.max(scale * 1e-9);
        cavity::check_cavity_extents(topo, result, clearance, cavity::Stage::Result)?;
    }
    Ok(result)
}

/// Every shell of the result must be a closed 2-manifold — the outer skin
/// and each cavity alike.
fn validate_offset_result(topo: &Topology, solid: SolidId) -> Result<(), OffsetError> {
    let solid_data = topo.solid(solid)?;
    for shell in
        std::iter::once(solid_data.outer_shell()).chain(solid_data.inner_shells().iter().copied())
    {
        brepkit_topology::validation::validate_shell_closed(topo.shell(shell)?, topo)?;
    }
    Ok(())
}
