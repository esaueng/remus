//! The face a cone wall is charted as: on the nappe where projection and
//! evaluation invert each other.
//!
//! `ConicalSurface` is the double cone `A + v (cos a ρ(u) + sin a â)`. At
//! `v < 0` the point at longitude `u` lies at `−ρ(u)`, half a turn round the
//! axis from the longitude `project_point` answers (the point's own). Every
//! cone mesher charts boundary points by projection and emits interior
//! points by evaluation, so a wall below its carrier's apex meshed as its
//! own mirror image across the axis. That nappe is the `v > 0` nappe of the
//! carrier with its axis reversed, with the same parametric normal
//! direction, so the face's sense carries over unchanged; the measurement
//! readers chart such faces on it already
//! ([`cone_chart_carrier`](remus_check::properties::face_integrator::cone_chart_carrier)).

use std::borrow::Cow;

use remus_topology::Topology;
use remus_topology::face::{Face, FaceId, FaceSurface};

/// `face_id` as the tessellator charts it: the face itself, or for a cone
/// wall below its carrier's apex a copy on the reversed-axis carrier.
///
/// Only a face the measurement readers move to the reversed carrier is
/// copied; every other face (upper-nappe cones included) is borrowed and
/// meshes exactly as before. A face whose boundary reaches both nappes, or
/// whose carrier is too flat to tell them apart, has no single chart; it
/// keeps its own carrier and meshes as it always did, since display and
/// mesh-fallback callers have no refusal to report and the exact readers
/// refuse such a face themselves.
///
/// # Errors
///
/// Only a missing face.
pub(super) fn chart_face(
    topo: &Topology,
    face_id: FaceId,
) -> Result<Cow<'_, Face>, crate::OperationsError> {
    let face = topo.face(face_id)?;
    let FaceSurface::Cone(cone) = face.surface() else {
        return Ok(Cow::Borrowed(face));
    };
    match remus_check::properties::face_integrator::cone_chart_carrier(topo, face_id, cone) {
        // The chart decision returns the carrier itself or its axis-reversed
        // copy; only the latter changes the face.
        Ok(carrier) if carrier.axis().dot(cone.axis()) < 0.0 => {
            let mut charted = face.clone();
            charted.set_surface(FaceSurface::Cone(carrier));
            Ok(Cow::Owned(charted))
        }
        Ok(_) | Err(_) => Ok(Cow::Borrowed(face)),
    }
}
