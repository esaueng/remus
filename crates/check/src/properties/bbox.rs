//! Axis-aligned bounding box computation for solids.

use remus_math::aabb::Aabb3;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

use crate::CheckError;
use crate::util::face_aabb;

/// Compute the axis-aligned bounding box of a solid.
///
/// Unions the conservative bounds of every face in the outer shell. Per-face
/// bounds account for curved edges as well as surface curvature, using exact
/// full-curve extents for circles and ellipses and conservative hulls for
/// other curve types.
///
/// # Errors
///
/// Returns an error if any topology entity referenced by the solid is missing,
/// or if the solid has no vertices.
pub fn bounding_box(topo: &Topology, solid: SolidId) -> Result<Aabb3, CheckError> {
    let solid_data = topo.solid(solid)?;
    let shell = topo.shell(solid_data.outer_shell())?;

    let mut faces = shell.faces().iter();
    let first = faces
        .next()
        .ok_or_else(|| CheckError::ClassificationFailed("solid has no vertices".into()))?;
    let mut aabb = face_aabb(topo, *first)?;
    for &face in faces {
        aabb = aabb.union(face_aabb(topo, face)?);
    }

    Ok(aabb)
}
