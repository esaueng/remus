//! Automatic discovery of one bounded shared constant-v ruled section.
//!
//! A control-row line intersection only proposes native v coordinates. The
//! original component subsequently proves both entire rational traces,
//! regularity, transversality and every physical boundary before returning.
//! This is not a complete face-face intersector or a GFA dispatch path.

use remus_math::context::OperationContext;
use remus_math::nurbs::{curve::NurbsCurve, surface::NurbsSurface};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};

use super::{ClipError, ClippedSection, FaceTrace, I, Result, budget, clip_section, patch};

/// An affine native trace of the discovered source on an original face.
#[derive(Debug, Clone, Copy)]
pub struct RuledSurfaceTrace {
    /// Original face; handles and certificates belong to the input topology.
    pub face: FaceId,
    /// Native source domain mapped affinely to `u_range`.
    pub source_range: [f64; 2],
    /// Increasing original native u coordinates at the source endpoints.
    pub u_range: [f64; 2],
    /// Original native constant-v coordinate.
    pub v: f64,
}

/// One certified shared section and all of its retained physical trim windows.
///
/// The curve's domain is the first face's unchanged native u domain. Both
/// original faces, their traces and per-use events remain explicit. This
/// component does not claim that other intersection components are absent,
/// assemble topology, or authorize an unsupported Boolean.
#[derive(Debug)]
pub struct RuledFaceSection {
    /// Rational section constructed from the first face without fitting.
    pub curve: NurbsCurve,
    /// Two original native surface traces, in the caller's face order.
    pub traces: [RuledSurfaceTrace; 2],
    /// Complete interval clipping and original boundary-use certificates.
    pub clipping: ClippedSection,
}

fn surface(topo: &Topology, face: FaceId) -> Result<&NurbsSurface> {
    let FaceSurface::Nurbs(surface) = topo.face(face)?.surface() else {
        return Err(ClipError::UnsupportedDomain);
    };
    // This performs the existing complete chart/weight/dimension and
    // whole-patch regularity checks before any control-row indexing below.
    let origin = surface
        .control_points()
        .first()
        .and_then(|row| row.first())
        .copied()
        .ok_or(ClipError::InvalidInput)?;
    patch(
        topo,
        FaceTrace {
            face,
            v: surface.domain_v().0,
        },
        origin,
    )?;
    Ok(surface)
}

fn scaled(direction: Vec3) -> Option<(Vec3, f64)> {
    let scale = direction.0.into_iter().map(f64::abs).fold(0.0, f64::max);
    (scale.is_finite() && scale > 0.0).then(|| {
        (
            Vec3::new(
                direction.x() / scale,
                direction.y() / scale,
                direction.z() / scale,
            ),
            scale,
        )
    })
}

/// Propose the intersection fractions of two control-row lines. Certified
/// nonzero 2x2 minors avoid division by an unresolved determinant. Neither
/// this solve nor agreeing control-row endpoints authorizes a section.
fn propose(first: &[Point3], second: &[Point3]) -> Option<[f64; 2]> {
    let (a, a_scale) = scaled(first[1] - first[0])?;
    let (b, b_scale) = scaled(second[0] - second[1])?;
    let delta = second[0] - first[0];
    let mut best: Option<(usize, usize, f64)> = None;
    for (i, j) in [(0, 1), (0, 2), (1, 2)] {
        let determinant = I::exact(a.0[i])
            .mul(I::exact(b.0[j]))
            .sub(I::exact(a.0[j]).mul(I::exact(b.0[i])));
        if determinant.lo <= 0.0 && determinant.hi >= 0.0 {
            continue;
        }
        let value = a.0[i] * b.0[j] - a.0[j] * b.0[i];
        if value.is_finite() && best.is_none_or(|(_, _, previous)| value.abs() > previous.abs()) {
            best = Some((i, j, value));
        }
    }
    let (i, j, determinant) = best?;
    let fractions = [
        (delta.0[i] * b.0[j] - delta.0[j] * b.0[i]) / determinant / a_scale,
        (a.0[i] * delta.0[j] - a.0[j] * delta.0[i]) / determinant / b_scale,
    ];
    fractions
        .iter()
        .all(|f| f.is_finite() && (0.0..=1.0).contains(f))
        .then_some(fractions)
}

/// Discover and clip one shared constant-v rational ruled-Bezier section.
///
/// Both faces must be nonperiodic positive-weight single-span ruled patches
/// of matching u degree 1--3, degree one in v, with equal weights along each
/// ruling. At least one pair of corresponding control-row lines must propose
/// finite in-chart v coordinates. The resulting rational section must pass
/// the original whole-span residual, regularity and transversality proofs
/// and rectangular physical outer/hole trim checks. Native u domains may
/// differ; they map affinely in increasing order without refitting.
///
/// This read-only prerequisite returns no partial record on refusal and
/// never mutates topology, history, budgets or public Boolean dispatch.
/// Certificates refer to the input geometry at call time. In particular an
/// empty `clipping.intervals` means this *section* misses the trimmed material;
/// it does not prove the faces disjoint or exclude other intersections.
///
/// # Errors
///
/// Returns a typed [`ClipError`] for unsupported/unproved geometry, missing
/// boundary authority, cancellation or insufficient segment budget. A failed
/// proposal is not interpreted as an empty intersection.
pub fn discover_ruled_face_section(
    topo: &Topology,
    faces: [FaceId; 2],
    context: &OperationContext,
) -> Result<RuledFaceSection> {
    budget(context, 1)?;
    let surfaces = [surface(topo, faces[0])?, surface(topo, faces[1])?];
    if surfaces[0].degree_u() != surfaces[1].degree_u()
        || surfaces[0].control_points().len() != surfaces[1].control_points().len()
    {
        return Err(ClipError::UnsupportedDomain);
    }
    let fractions = surfaces[0]
        .control_points()
        .iter()
        .zip(surfaces[1].control_points())
        .find_map(|(first, second)| propose(first, second))
        .ok_or(ClipError::UnresolvedContact)?;
    let native: [f64; 2] = std::array::from_fn(|j| {
        let (lo, hi) = surfaces[j].domain_v();
        lo + (hi - lo) * fractions[j]
    });
    let traces = std::array::from_fn(|j| FaceTrace {
        face: faces[j],
        v: native[j],
    });
    // The actual stored native v is the authority; rounding of a large or
    // narrow chart may change the proposed fraction. The full-span proof
    // below checks this construction against both original evaluations.
    let (v0, v1) = surfaces[0].domain_v();
    let fraction = (native[0] - v0) / (v1 - v0);
    let curve = NurbsCurve::new(
        surfaces[0].degree_u(),
        surfaces[0].knots_u().to_vec(),
        surfaces[0]
            .control_points()
            .iter()
            .map(|row| row[0] + (row[1] - row[0]) * fraction)
            .collect(),
        surfaces[0].weights().iter().map(|row| row[0]).collect(),
    )
    .map_err(|_| ClipError::InvalidInput)?;
    let clipping = clip_section(topo, traces, &curve, context)?;
    let (lo, hi) = curve.domain();
    Ok(RuledFaceSection {
        curve,
        traces: std::array::from_fn(|j| {
            let (u0, u1) = surfaces[j].domain_u();
            RuledSurfaceTrace {
                face: faces[j],
                source_range: [lo, hi],
                u_range: [u0, u1],
                v: native[j],
            }
        }),
        clipping,
    })
}

#[cfg(test)]
mod tests;
