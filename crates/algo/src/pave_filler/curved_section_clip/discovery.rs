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

use super::{
    ClipError, ClippedSection, FaceTrace, I, Result, bernstein, budget, clip_section, curve_h,
    patch,
};

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

/// A full ULP safely bounds correctly rounded arithmetic, including
/// subnormals and binade boundaries. Infinite radii deliberately refuse.
fn roundoff_radius(value: f64) -> f64 {
    let value = value.abs();
    (value.next_up() - value).max(value - value.next_down())
}

/// Roundoff in f = fl(fl(parameter - lo) / fl(hi - lo)), uniformly for
/// stored parameters in the domain. Both subtraction results are bounded by
/// one ULP of the positive rounded span; the exact fraction lies in [0,1].
/// Thus the two subtraction errors contribute at most 2*ulp(span)/span,
/// and the final division contributes at most ulp(1). All bounds round out.
fn fraction_roundoff(domain: (f64, f64)) -> I {
    let span = domain.1 - domain.0;
    if !span.is_finite() || span <= 0.0 {
        return I::exact(f64::INFINITY);
    }
    I::exact(roundoff_radius(span))
        .mul(I::exact(2.0))
        .div(I::exact(span))
        .add(I::exact(roundoff_radius(1.0)))
}

/// Normalized target-u error in the caller's native affine trace map:
/// f=(parameter-source_lo)/(source_hi-source_lo);
/// u=target_lo+(target_hi-target_lo)*f. The allowance includes source
/// normalization, target span subtraction/multiplication/addition, and the
/// surface evaluator's final normalization of that stored native u. Its
/// native-domain clamp cannot increase distance from the exact in-domain u.
fn affine_trace_roundoff(source: (f64, f64), target: (f64, f64)) -> I {
    let span = target.1 - target.0;
    let exact_span = I::exact(target.1).sub(I::exact(target.0));
    if !span.is_finite() || span <= 0.0 || exact_span.lo <= 0.0 {
        return I::exact(f64::INFINITY);
    }
    let end = I::exact(target.0).add(I::exact(span));
    let largest_sum = target.0.abs().max(end.abs_max());
    let native_roundoff = I::exact(roundoff_radius(span))
        .mul(I::exact(2.0))
        .add(I::exact(roundoff_radius(largest_sum)));
    fraction_roundoff(source)
        .add(native_roundoff.div(I::exact(exact_span.lo)))
        .add(fraction_roundoff(target))
}

/// Absolute error of the existing degree<=3 curve / degree<=3-by-1 surface
/// position evaluators, including their exponent-scaled tiny-term branch.
/// Per coordinate the ordinary surface path has <=43 basis operations,
/// 16 scale products and 41 contraction/division operations. The exceptional
/// path has <=43 basis +75 mantissa/scale/sum operations. Counting an FMA as
/// two operations, gamma_128 bounds either path (curve paths are shorter).
/// Positive weights bound quotient conditioning by kappa; kappa^2 also
/// covers absolute basis perturbations. The numerator and denominator each
/// contribute gamma_128*kappa^2, hence 2*c/(1-c) times the absolute control
/// magnitude. A separate subnormal floor covers coordinate underflow.
fn evaluator_roundoff(
    points: impl Iterator<Item = Point3>,
    weights: impl Iterator<Item = f64>,
    domains: &[(f64, f64)],
) -> f64 {
    // Keep recurrence divisors, raw weight products and ordinary contraction
    // intermediates inside the bounded arithmetic regime. Uniform extreme
    // weights can be evaluated by the math crate but are not all qualified
    // by this component's operational certificate.
    let safe_range = 64.0 * f64::MIN_POSITIVE..=f64::MAX / 64.0;
    if domains.iter().any(|d| !safe_range.contains(&(d.1 - d.0))) {
        return f64::INFINITY;
    }
    let (mut minimum, mut maximum) = (f64::INFINITY, 0.0_f64);
    for weight in weights {
        if !safe_range.contains(&weight) {
            return f64::INFINITY;
        }
        minimum = minimum.min(weight);
        maximum = maximum.max(weight);
    }
    let kappa = I::exact(maximum).div(I::exact(minimum));
    let kappa_squared = kappa.mul(kappa);
    let operation_error = I::exact(128.0).mul(I::exact(f64::EPSILON / 2.0));
    let gamma = operation_error.div(I::exact(1.0).sub(operation_error));
    let condition = gamma.mul(kappa_squared);
    if !condition.hi.is_finite() || condition.hi >= 0.25 {
        return f64::INFINITY;
    }
    let factor = I::exact(2.0)
        .mul(condition)
        .div(I::exact(1.0).sub(condition));
    let mut magnitudes = [0.0_f64; 3];
    for point in points {
        for (magnitude, value) in magnitudes.iter_mut().zip(point.0) {
            if !value.is_finite() {
                return f64::INFINITY;
            }
            *magnitude = magnitude.max(value.abs());
        }
    }
    let floor = I::exact(128.0 * f64::from_bits(1)).mul(kappa_squared);
    let mut squared = I::exact(0.0);
    for magnitude in magnitudes {
        if I::exact(128.0).mul(kappa).mul(I::exact(magnitude)).hi > f64::MAX / 4.0 {
            return f64::INFINITY;
        }
        let coordinate = factor.mul(I::exact(magnitude)).add(floor);
        squared = squared.add(coordinate.mul(coordinate));
    }
    squared.hi.sqrt().next_up()
}

fn qualify_native_traces(
    topo: &Topology,
    traces: [FaceTrace; 2],
    curve: &NurbsCurve,
    evaluation_bounds: [f64; 2],
    clipping: &mut ClippedSection,
    context: &OperationContext,
) -> Result<()> {
    let origin = curve.control_points()[0];
    let source = curve_h(curve, origin)?;
    let source_error =
        I::exact(bernstein::derivative_bound(&source)).mul(fraction_roundoff(curve.domain()));
    for (index, trace) in traces.into_iter().enumerate() {
        budget(context, 1)?;
        let patch = patch(topo, trace, origin)?;
        let u_error = I::exact(bernstein::derivative_bound(&patch.section)).mul(
            affine_trace_roundoff(curve.domain(), patch.surface.domain_u()),
        );
        // The proposed v was rounded before construction. Existing residual
        // proofs already use that actual stored v, not the proposal. Only
        // its evaluator normalization remains to be accounted for here.
        // Equal ruling weights make the normalized-v derivative exactly the
        // difference of the two rational end curves over the entire u span.
        let v_error = I::exact(bernstein::residual(&patch.ends[0], &patch.ends[1]))
            .mul(fraction_roundoff(patch.surface.domain_v()));
        let bound = I::exact(clipping.surface_residual_bounds[index])
            .add(I::exact(evaluation_bounds[index]))
            .add(source_error)
            .add(u_error)
            .add(v_error)
            .hi;
        if !bound.is_finite() || bound > context.tolerance.linear {
            return Err(ClipError::Residual { bound });
        }
        clipping.surface_residual_bounds[index] = bound;
    }
    Ok(())
}

/// Discover and clip one shared constant-v rational ruled-Bezier section.
///
/// Both faces must be nonperiodic positive-weight single-span ruled patches
/// of matching u degree 1--3, degree one in v, with equal weights along each
/// ruling. At least one pair of corresponding control-row lines must propose
/// finite in-chart v coordinates. The resulting rational section must pass
/// the original whole-span residual, regularity and transversality proofs
/// and rectangular physical outer/hole trim checks. Native u domains may
/// differ; they map affinely in increasing order without refitting. Native
/// map and normalization roundoff are propagated through whole-section
/// derivative bounds; insufficient chart precision refuses at the unchanged
/// linear tolerance rather than publishing an understated residual bound.
/// Absolute coordinate magnitude and positive-weight conditioning also bound
/// the original position evaluators; unsafe arithmetic regimes refuse.
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
    if !context.tolerance.linear.is_finite() || context.tolerance.linear <= 0.0 {
        return Err(ClipError::InvalidInput);
    }
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
    let source_evaluation = evaluator_roundoff(
        curve.control_points().iter().copied(),
        curve.weights().iter().copied(),
        &[curve.domain()],
    );
    let evaluation_bounds = surfaces.map(|s| {
        I::exact(source_evaluation)
            .add(I::exact(evaluator_roundoff(
                s.control_points().iter().flatten().copied(),
                s.weights().iter().flatten().copied(),
                &[s.domain_u(), s.domain_v()],
            )))
            .hi
    });
    // Refuse unsafe evaluator regimes before the clipper evaluates physical
    // boundary events. The final certificate also includes the native maps.
    for bound in evaluation_bounds {
        if !bound.is_finite() || bound > context.tolerance.linear {
            return Err(ClipError::Residual { bound });
        }
    }
    let mut clipping = clip_section(topo, traces, &curve, context)?;
    qualify_native_traces(
        topo,
        traces,
        &curve,
        evaluation_bounds,
        &mut clipping,
        context,
    )?;
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
