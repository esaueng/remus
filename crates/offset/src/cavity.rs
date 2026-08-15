//! Cavity (inner shell) extent checks.
//!
//! A solid may bound its volume with an outer shell plus any number of inner
//! shells, each enclosing a void. Offsetting such a solid offsets every shell
//! by the same signed distance along its own outward normal, which is the
//! direction pointing away from material. A cavity's outward normals point
//! *into* the void, so a positive (outward) offset grows the outer boundary
//! and **shrinks** each cavity, and a negative offset does the reverse.
//!
//! Shell-to-shell interference is a global property this engine does not
//! compute — the same limitation that keeps
//! [`remove_self_intersections`](crate::self_int::remove_self_intersections)
//! unimplemented. What this module contributes is a cheap *necessary*
//! condition: a cavity's spatial extent must stay strictly inside the outer
//! shell's extent, before and after the offset. It catches the gross case
//! (a cavity that reaches or crosses the outer boundary) and refuses loudly
//! rather than returning a well-formed solid with the wrong volume.
//!
//! Every extent here is an *over*-approximation of the real geometry: planar
//! faces use their exact vertex hull, quadric faces are widened to cover the
//! bulge between boundary vertices, and NURBS faces use the control-point
//! hull (the convex-hull property makes that a bound). Over-approximating
//! both extents keeps the containment test from passing on geometry it has
//! not actually looked at.

use brepkit_math::aabb::Aabb3;
use brepkit_math::vec::Point3;
use brepkit_topology::Topology;
use brepkit_topology::face::{FaceId, FaceSurface};
use brepkit_topology::shell::ShellId;
use brepkit_topology::solid::SolidId;

use crate::error::OffsetError;

/// Upper bound on cavity shells accepted by the pairwise disjointness check.
///
/// The check below intentionally compares every pair. Keeping this limit near
/// one thousand bounds that work to roughly half a million comparisons even
/// when a solid came from an untrusted importer.
const MAX_CAVITY_SHELLS: usize = 1_024;

/// Where a cavity check is being applied, for error wording.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stage {
    /// Before the pipeline runs, on the caller's solid.
    Input,
    /// After assembly, on the offset result.
    Result,
}

impl Stage {
    const fn as_str(self) -> &'static str {
        match self {
            Self::Input => "input solid",
            Self::Result => "offset result",
        }
    }
}

/// Check that every cavity shell of `solid` stays strictly inside the outer
/// shell's extent, with at least `clearance` to spare on all six sides, and
/// that no two cavities overlap.
///
/// # Errors
///
/// Returns [`OffsetError::InvalidInput`] naming cavity shells when a cavity
/// reaches or crosses the outer boundary, when two cavities meet, when the
/// solid exceeds the cavity work budget, or when a shell has no geometry to
/// bound.
pub fn check_cavity_extents(
    topo: &Topology,
    solid: SolidId,
    clearance: f64,
    stage: Stage,
) -> Result<(), OffsetError> {
    let solid_data = topo.solid(solid)?;
    let inner_shells = solid_data.inner_shells().to_vec();
    if inner_shells.is_empty() {
        return Ok(());
    }
    if inner_shells.len() > MAX_CAVITY_SHELLS {
        return Err(OffsetError::InvalidInput {
            reason: format!(
                "offset of solids with cavity shells supports at most {MAX_CAVITY_SHELLS} \
                 cavities; the {} has {}",
                stage.as_str(),
                inner_shells.len(),
            ),
        });
    }
    let outer = shell_extent(topo, solid_data.outer_shell(), stage)?;

    let mut cavity_extents = Vec::with_capacity(inner_shells.len());
    for &shell_id in &inner_shells {
        let cavity = shell_extent(topo, shell_id, stage)?;
        if !strictly_inside(cavity, outer, clearance) {
            return Err(OffsetError::InvalidInput {
                reason: format!(
                    "offset of solids with cavity shells requires every cavity to stay strictly \
                     inside the outer shell: on the {}, cavity shell {} spans {} while the outer \
                     shell spans {} (needs {clearance:.6e} clearance on every side)",
                    stage.as_str(),
                    shell_id.index(),
                    format_extent(cavity),
                    format_extent(outer),
                ),
            });
        }
        cavity_extents.push((shell_id, cavity));
    }

    for i in 0..cavity_extents.len() {
        for j in (i + 1)..cavity_extents.len() {
            let (id_a, a) = cavity_extents[i];
            let (id_b, b) = cavity_extents[j];
            if a.expanded(clearance * 0.5)
                .intersects(b.expanded(clearance * 0.5))
            {
                return Err(OffsetError::InvalidInput {
                    reason: format!(
                        "offset of solids with cavity shells requires the cavities to stay \
                         disjoint: on the {}, cavity shells {} ({}) and {} ({}) meet",
                        stage.as_str(),
                        id_a.index(),
                        format_extent(a),
                        id_b.index(),
                        format_extent(b),
                    ),
                });
            }
        }
    }

    Ok(())
}

/// Reject an outward offset that would collapse a cavity's spatial extent.
///
/// A positive offset moves both opposing cavity walls inward by `distance`,
/// so every axis of the cavity's bounding extent must remain larger than the
/// scale-aware clearance after losing `2 * distance`.
///
/// # Errors
///
/// Returns [`OffsetError::InvalidInput`] when a cavity would collapse along
/// at least one axis.
pub fn check_cavity_survival(
    topo: &Topology,
    solid: SolidId,
    distance: f64,
    clearance: f64,
) -> Result<(), OffsetError> {
    if distance <= 0.0 {
        return Ok(());
    }

    let minimum_span = 2.0 * distance + clearance;
    for &shell_id in topo.solid(solid)?.inner_shells() {
        let cavity = shell_extent(topo, shell_id, Stage::Input)?;
        let spans = [
            cavity.max.x() - cavity.min.x(),
            cavity.max.y() - cavity.min.y(),
            cavity.max.z() - cavity.min.z(),
        ];
        if spans.into_iter().any(|span| span <= minimum_span) {
            return Err(OffsetError::InvalidInput {
                reason: format!(
                    "offset of solids with cavity shells requires every cavity to survive: \
                     cavity shell {} spans {} and an outward offset of {distance:.6e} needs \
                     more than {minimum_span:.6e} on every axis",
                    shell_id.index(),
                    format_extent(cavity),
                ),
            });
        }
    }
    Ok(())
}

/// The clearance a cavity must keep from the outer shell for an offset of
/// `distance` to be attempted.
///
/// Each shell moves by exactly `|distance|` along its own outward normal.
/// An inward offset (`distance < 0`) moves the outer boundary in and the
/// cavity walls out, closing the wall between them by `2 * |distance|`; an
/// outward offset opens it. The requirement is expressed relative to the
/// model's own size so it survives a change of units — see
/// `scale_invariance` in the integration tests.
#[must_use]
pub fn required_clearance(distance: f64, extent_scale: f64, linear_tol: f64) -> f64 {
    let closing = 2.0 * (-distance).max(0.0);
    closing + linear_tol.max(extent_scale * RELATIVE_CLEARANCE)
}

/// Clearance floor as a fraction of the model's own diagonal, so the check
/// means the same thing at 0.001x and 1000x.
const RELATIVE_CLEARANCE: f64 = 1e-9;

/// The longest side of a solid's outer shell extent — the scale against
/// which relative clearances are measured.
///
/// # Errors
///
/// Returns [`OffsetError::InvalidInput`] if the outer shell has no geometry.
pub fn solid_extent_scale(topo: &Topology, solid: SolidId) -> Result<f64, OffsetError> {
    let outer = shell_extent(topo, topo.solid(solid)?.outer_shell(), Stage::Input)?;
    Ok((outer.max.x() - outer.min.x())
        .max(outer.max.y() - outer.min.y())
        .max(outer.max.z() - outer.min.z()))
}

fn strictly_inside(inner: Aabb3, outer: Aabb3, clearance: f64) -> bool {
    inner.min.x() - outer.min.x() > clearance
        && inner.min.y() - outer.min.y() > clearance
        && inner.min.z() - outer.min.z() > clearance
        && outer.max.x() - inner.max.x() > clearance
        && outer.max.y() - inner.max.y() > clearance
        && outer.max.z() - inner.max.z() > clearance
}

fn format_extent(b: Aabb3) -> String {
    format!(
        "[{:.6}, {:.6}, {:.6}]..[{:.6}, {:.6}, {:.6}]",
        b.min.x(),
        b.min.y(),
        b.min.z(),
        b.max.x(),
        b.max.y(),
        b.max.z()
    )
}

/// An over-approximating axis-aligned extent for every face of a shell.
fn shell_extent(topo: &Topology, shell: ShellId, stage: Stage) -> Result<Aabb3, OffsetError> {
    let faces = topo.shell(shell)?.faces().to_vec();
    let mut extent: Option<Aabb3> = None;
    for face_id in faces {
        let face_extent = face_extent(topo, face_id)?;
        extent = Some(match extent {
            None => face_extent,
            Some(acc) => merge(acc, face_extent),
        });
    }
    extent.ok_or_else(|| OffsetError::InvalidInput {
        reason: format!(
            "cavity extent check: shell {} on the {} bounds no geometry",
            shell.index(),
            stage.as_str()
        ),
    })
}

fn merge(a: Aabb3, b: Aabb3) -> Aabb3 {
    Aabb3 {
        min: Point3::new(
            a.min.x().min(b.min.x()),
            a.min.y().min(b.min.y()),
            a.min.z().min(b.min.z()),
        ),
        max: Point3::new(
            a.max.x().max(b.max.x()),
            a.max.y().max(b.max.y()),
            a.max.z().max(b.max.z()),
        ),
    }
}

/// An over-approximating extent for a single face.
///
/// The trimmed patch is bounded by its boundary vertices plus whatever the
/// surface bulges between them; each arm widens the vertex hull by a bound on
/// that bulge, or replaces it with a bound on the whole surface.
fn face_extent(topo: &Topology, face_id: FaceId) -> Result<Aabb3, OffsetError> {
    let hull = vertex_hull(topo, face_id)?;
    let face = topo.face(face_id)?;
    Ok(match face.surface() {
        // A planar patch never leaves the hull of its own boundary.
        FaceSurface::Plane { .. } => hull,
        // A patch of a sphere cannot leave the sphere.
        FaceSurface::Sphere(sph) => {
            let c = sph.center();
            let r = sph.radius();
            Aabb3 {
                min: Point3::new(c.x() - r, c.y() - r, c.z() - r),
                max: Point3::new(c.x() + r, c.y() + r, c.z() + r),
            }
        }
        // A torus lies within `major + minor` of its centre in every
        // direction, whatever its axis.
        FaceSurface::Torus(tor) => {
            let c = tor.center();
            let r = tor.major_radius() + tor.minor_radius();
            Aabb3 {
                min: Point3::new(c.x() - r, c.y() - r, c.z() - r),
                max: Point3::new(c.x() + r, c.y() + r, c.z() + r),
            }
        }
        // A cylinder is unbounded along its axis, so the vertex hull carries
        // the axial extent; the surface can bow out from the chord between
        // two boundary vertices by at most the radius.
        FaceSurface::Cylinder(cyl) => hull.expanded(cyl.radius()),
        // Same argument for a cone, with the widest circle the patch reaches
        // standing in for the radius.
        FaceSurface::Cone(cone) => {
            let axis = cone.axis();
            let apex = cone.apex();
            let mut max_radius: f64 = 0.0;
            for vid in brepkit_topology::explorer::face_vertices(topo, face_id)? {
                let p = topo.vertex(vid)?.point();
                let d = p - apex;
                let along = d.dot(axis);
                let radial = d - axis * along;
                max_radius = max_radius.max(radial.length());
            }
            hull.expanded(max_radius)
        }
        // The convex-hull property bounds a NURBS patch by its control net.
        FaceSurface::Nurbs(nurbs) => nurbs.aabb(),
    })
}

fn vertex_hull(topo: &Topology, face_id: FaceId) -> Result<Aabb3, OffsetError> {
    let mut points = Vec::new();
    for vid in brepkit_topology::explorer::face_vertices(topo, face_id)? {
        points.push(topo.vertex(vid)?.point());
    }
    Aabb3::try_from_points(points).ok_or_else(|| OffsetError::InvalidInput {
        reason: format!(
            "cavity extent check: face {} has no boundary vertices to bound",
            face_id.index()
        ),
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;

    #[test]
    fn inward_offset_demands_twice_the_distance_in_clearance() {
        let clearance = required_clearance(-0.5, 10.0, 1e-7);
        assert!(
            (clearance - 1.0).abs() < 1e-6,
            "an inward offset of 0.5 closes the wall by 1.0, got {clearance}"
        );
    }

    #[test]
    fn outward_offset_demands_only_a_relative_floor() {
        let clearance = required_clearance(0.5, 10.0, 0.0);
        assert!(
            clearance > 0.0 && clearance < 1e-6,
            "an outward offset opens the wall, got {clearance}"
        );
    }

    #[test]
    fn clearance_floor_tracks_model_scale() {
        let small = required_clearance(1e-6, 1e-3, 0.0);
        let large = required_clearance(1.0, 1e3, 0.0);
        assert!(
            (large / small - 1e6).abs() < 1.0,
            "the floor must scale with the model: {small} vs {large}"
        );
    }
}
