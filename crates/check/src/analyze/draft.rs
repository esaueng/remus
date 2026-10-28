//! Signed draft angles relative to a caller-supplied pull direction.
//!
//! The angle is `asin(outward_normal · unit_pull)`: a vertical wall has zero
//! draft, a face directed toward pull has positive draft, and a face directed
//! away has negative draft. A negative angle alone is not a global undercut
//! or accessibility test. All angles are in radians.

use remus_math::vec::Vec3;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

/// A draft query cannot provide a meaningful angle.
#[derive(Debug, thiserror::Error)]
pub enum DraftAnalysisError {
    /// The referenced face or solid is unavailable.
    #[error(transparent)]
    Topology(#[from] remus_topology::TopologyError),
    /// Pull must be finite and nonzero; its magnitude is otherwise irrelevant.
    #[error("draft analysis requires a finite nonzero pull direction")]
    InvalidPullDirection,
    /// Parameters must be finite and within the carrier's declared domain.
    #[error("draft analysis parameters are outside the surface domain")]
    InvalidParameters,
    /// The surface has no qualified finite, uniquely determined normal.
    #[error("draft analysis has no regular surface normal")]
    UndefinedNormal,
    /// The constant-per-face report cannot represent a curved face.
    #[error("constant draft report requires planar faces; face {face:?} is curved")]
    NonPlanarFace {
        /// The first curved face encountered.
        face: FaceId,
    },
}

/// One local draft angle, using the face's effective outward orientation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DraftSample {
    /// Signed angle in `[-pi/2, pi/2]`, in radians.
    pub angle_radians: f64,
    /// Unit surface normal, negated for a reversed face.
    pub outward_normal: Vec3,
}

/// Constant draft on one planar face of a solid.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarFaceDraft {
    /// Source face identity; no topology is copied or mutated.
    pub face: FaceId,
    /// Exact constant-angle evaluation, up to floating-point roundoff.
    pub sample: DraftSample,
}

/// Evaluate a face carrier's signed draft at `(u, v)`.
///
/// This does **not** test the trimmed face's membership and does not claim a
/// minimum or maximum over that face. Plane parameters are ignored after
/// checking finiteness. Analytic angular parameters are periodic; sphere
/// latitude is restricted to `[-pi/2, pi/2]`, cone slant distance must be
/// positive, and NURBS parameters must lie in their native knot domains.
/// Sphere poles have a unique analytic normal; a collapsed NURBS chart refuses.
/// Internal NURBS knots permitting discontinuous first derivatives also refuse;
/// no arbitrary one-sided span normal is promoted to a unique surface normal.
///
/// # Errors
///
/// Returns a typed error for invalid handles, parameters, pull direction or
/// undefined normals. The topology, journal and mutation tick remain untouched.
pub fn surface_draft_angle(
    topo: &Topology,
    face: FaceId,
    u: f64,
    v: f64,
    pull: Vec3,
) -> Result<DraftSample, DraftAnalysisError> {
    let pull = unit(pull).ok_or(DraftAnalysisError::InvalidPullDirection)?;
    if !u.is_finite() || !v.is_finite() {
        return Err(DraftAnalysisError::InvalidParameters);
    }
    let face = topo.face(face)?;
    let raw = match face.surface() {
        FaceSurface::Nurbs(surface) => {
            let (u0, u1) = surface.domain_u();
            let (v0, v1) = surface.domain_v();
            if !(u0..=u1).contains(&u) || !(v0..=v1).contains(&v) {
                return Err(DraftAnalysisError::InvalidParameters);
            }
            if unqualified_knot(surface.knots_u(), surface.degree_u(), u, (u0, u1))
                || unqualified_knot(surface.knots_v(), surface.degree_v(), v, (v0, v1))
            {
                return Err(DraftAnalysisError::UndefinedNormal);
            }
            let derivatives = surface.derivatives(u, v, 1);
            let du = derivatives.get(1).and_then(|row| row.first()).copied();
            let dv = derivatives.first().and_then(|row| row.get(1)).copied();
            // Normalize separately to avoid squaring geometric scale in the
            // cross product; do not inherit normal()'s synthetic +Z fallback.
            let du = du
                .and_then(unit)
                .ok_or(DraftAnalysisError::UndefinedNormal)?;
            let dv = dv
                .and_then(unit)
                .ok_or(DraftAnalysisError::UndefinedNormal)?;
            du.cross(dv)
        }
        FaceSurface::Plane { normal, .. } => *normal,
        FaceSurface::Cylinder(cylinder) => cylinder.normal(u, v),
        FaceSurface::Cone(cone) => {
            if v <= 0.0 {
                return Err(DraftAnalysisError::InvalidParameters);
            }
            cone.normal(u, v)
        }
        FaceSurface::Sphere(sphere) => {
            if v.abs() > std::f64::consts::FRAC_PI_2 {
                return Err(DraftAnalysisError::InvalidParameters);
            }
            sphere.normal(u, v)
        }
        FaceSurface::Torus(torus) => {
            if torus.major_radius() + torus.minor_radius() * v.cos() <= 0.0 {
                return Err(DraftAnalysisError::UndefinedNormal);
            }
            torus.normal(u, v)
        }
    };
    let normal = unit(raw).ok_or(DraftAnalysisError::UndefinedNormal)?;
    let normal = if face.is_reversed() { -normal } else { normal };
    Ok(sample(normal, pull))
}

/// Return the constant draft angle of every face of an all-planar solid.
///
/// Includes both outer and cavity shells in explorer order. This is not a
/// sampled curved-face report: encountering a curved face returns a typed
/// error, with no partial success or topology mutation.
///
/// # Errors
///
/// Returns a typed error for an invalid handle, pull direction, non-planar
/// face or undefined normal.
pub fn planar_solid_draft(
    topo: &Topology,
    solid: SolidId,
    pull: Vec3,
) -> Result<Vec<PlanarFaceDraft>, DraftAnalysisError> {
    let pull = unit(pull).ok_or(DraftAnalysisError::InvalidPullDirection)?;
    solid_faces(topo, solid)?
        .into_iter()
        .map(|id| {
            let face = topo.face(id)?;
            let FaceSurface::Plane { normal, .. } = face.surface() else {
                return Err(DraftAnalysisError::NonPlanarFace { face: id });
            };
            let normal = unit(*normal).ok_or(DraftAnalysisError::UndefinedNormal)?;
            let normal = if face.is_reversed() { -normal } else { normal };
            Ok(PlanarFaceDraft {
                face: id,
                sample: sample(normal, pull),
            })
        })
        .collect()
}

fn sample(normal: Vec3, pull: Vec3) -> DraftSample {
    DraftSample {
        // The equivalent atan2 form retains accuracy near +/- pi/2,
        // where rounding a dot product just below 1 amplifies asin's error.
        angle_radians: normal.dot(pull).atan2(normal.cross(pull).length()),
        outward_normal: normal,
    }
}

fn unqualified_knot(knots: &[f64], degree: usize, parameter: f64, domain: (f64, f64)) -> bool {
    // Exact knot identity is intentional. Merely being close to a knot does
    // not remove the unique normal of a regular open span. At multiplicity
    // >= degree the carrier promises at most C0; coplanar coincident normals
    // could be admitted later with a separate continuity certificate.
    parameter > domain.0
        && parameter < domain.1
        && knots
            .iter()
            .filter(|&&knot| knot.partial_cmp(&parameter) == Some(std::cmp::Ordering::Equal))
            .count()
            >= degree
}

fn unit(vector: Vec3) -> Option<Vec3> {
    if !vector.0.iter().all(|x| x.is_finite()) {
        return None;
    }
    let scale = vector.x().abs().max(vector.y().abs()).max(vector.z().abs());
    if scale == 0.0 {
        return None;
    }
    // Component division also handles subnormal directions without forming
    // an overflowing reciprocal.
    let scaled = Vec3::new(vector.x() / scale, vector.y() / scale, vector.z() / scale);
    Some(scaled * (1.0 / scaled.length()))
}

#[cfg(test)]
mod tests;
