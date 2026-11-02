//! A thread's transverse outline: the section of the thread across its axis.
//!
//! The thread form is the axial profile `f(u)`, the radius of the boundary
//! between material and air at axial position `u`, periodic with the pitch.
//! A screw motion turning `k` radians per unit length about the axis maps the
//! transverse outline `r(θ) = f(−θ/k)` onto the thread surface
//! `R(θ, z) = f(z − θ/k)`, so the thread is the twisted extrusion of that
//! outline. Over one turn the outline covers one lead, so a thread of `n`
//! starts has `n` lobes.
//!
//! The form is trapezoidal: a crest flat, the upper flank (facing the end of
//! the thread), a root flat and the lower flank (facing its start). A flat
//! is a constant radius, so its outline piece is an arc concentric with the
//! axis. A flank is linear in `u`, so its piece is an Archimedean spiral
//! `r = r₀ + (r₁ − r₀)·(θ − θ₀)/(θ₁ − θ₀)`, fitted with certified cubic
//! Hermite spans; a flank at zero angle (a square thread) is a radial line.

use std::f64::consts::TAU;

use remus_math::nurbs::NurbsCurve;
use remus_math::nurbs::hermite_fit::fit_cubic_hermite;
use remus_math::vec::{Point3, Vec3};

use super::{ThreadError, ThreadForm, ThreadHand};

/// Hermite spans allowed per flank across the profile.
const MAX_SPANS_PER_FLANK: usize = 512;

/// One piece of the outline in the local frame (`z = 0`, axis at the
/// origin), traversed counter-clockwise about the axis.
#[derive(Debug, Clone)]
pub(super) enum Segment {
    /// An arc of the circle of `radius` about the axis from angle `from` to
    /// `to > from`.
    Arc { radius: f64, from: f64, to: f64 },
    /// A straight radial segment from `start` (it ends where the next
    /// piece starts).
    Line { start: Point3 },
    /// A fitted spiral flank, parameterized counter-clockwise.
    Flank(NurbsCurve),
}

impl Segment {
    pub(super) fn start(&self) -> Point3 {
        match self {
            Self::Arc { radius, from, .. } => polar(*radius, *from),
            Self::Line { start, .. } => *start,
            Self::Flank(curve) => curve.control_points()[0],
        }
    }
}

/// The outline and its certified fit report.
pub(super) struct Outline {
    pub(super) segments: Vec<Segment>,
    /// Proven bound on any flank piece's distance from its exact spiral.
    pub(super) remainder_bound: f64,
    /// Most Hermite spans used by one flank piece.
    pub(super) spans_per_flank: usize,
}

pub(super) fn polar(radius: f64, angle: f64) -> Point3 {
    let (sin, cos) = angle.sin_cos();
    Point3::new(radius * cos, radius * sin, 0.0)
}

/// A piece of the axial profile over `[u0, u1]`: the radius goes linearly
/// from `r0` to `r1` (equal on a flat).
#[derive(Debug, Clone, Copy)]
struct Piece {
    u0: f64,
    u1: f64,
    r0: f64,
    r1: f64,
}

/// The axial profile over one lead, from the start of a crest flat, in
/// increasing `u`. Zero-width pieces are kept as steps (a square flank).
fn axial_pieces(form: &ThreadForm, crest_radius: f64, root_radius: f64) -> Vec<Piece> {
    let depth = (crest_radius - root_radius).abs();
    let upper = depth * form.upper_flank_angle.tan();
    let root = form.root_width();
    let mut pieces = Vec::new();
    for start in 0..form.starts {
        let u = f64::from(start) * form.pitch;
        let crest_end = u + form.crest_width;
        let upper_end = crest_end + upper;
        let root_end = upper_end + root;
        // The lower flank fills the rest of the pitch: the root width is the
        // pitch less the crest and both flanks.
        let next = u + form.pitch;
        pieces.push(Piece {
            u0: u,
            u1: crest_end,
            r0: crest_radius,
            r1: crest_radius,
        });
        pieces.push(Piece {
            u0: crest_end,
            u1: upper_end,
            r0: crest_radius,
            r1: root_radius,
        });
        pieces.push(Piece {
            u0: upper_end,
            u1: root_end,
            r0: root_radius,
            r1: root_radius,
        });
        pieces.push(Piece {
            u0: root_end,
            u1: next,
            r0: root_radius,
            r1: crest_radius,
        });
    }
    pieces
}

/// The transverse outline of a thread of `form` with its crest at
/// `crest_radius` and its root at `root_radius`, turned by `phase`
/// radians, for a screw motion of the form's hand and lead.
///
/// `tolerance` bounds every flank fit's distance from its exact spiral.
pub(super) fn outline(
    form: &ThreadForm,
    crest_radius: f64,
    root_radius: f64,
    phase: f64,
    tolerance: f64,
) -> Result<Outline, ThreadError> {
    let lead = form.lead();
    // `θ = −k·u` with `k = ±2π/lead`: a right hand runs the axial profile
    // backwards as the outline turns counter-clockwise.
    let scale = TAU / lead;
    let reversed = form.hand == ThreadHand::Right;
    let mut pieces = axial_pieces(form, crest_radius, root_radius);
    if reversed {
        pieces.reverse();
    }
    let angle = |u: f64| {
        if reversed {
            phase + scale * (lead - u)
        } else {
            phase + scale * u
        }
    };
    let mut segments = Vec::with_capacity(pieces.len());
    let mut remainder_bound: f64 = 0.0;
    let mut spans_per_flank = 0;
    let width_tol = 1e-12 * lead;
    for piece in pieces {
        let (mut from, mut to) = (angle(piece.u0), angle(piece.u1));
        let (mut r_from, mut r_to) = (piece.r0, piece.r1);
        if reversed {
            std::mem::swap(&mut from, &mut to);
            std::mem::swap(&mut r_from, &mut r_to);
        }
        let width = piece.u1 - piece.u0;
        if width <= width_tol {
            // A step: a radial line at one angle, unless the radius does not
            // change (a flat of zero width, which leaves nothing).
            if (r_from - r_to).abs() > width_tol {
                segments.push(Segment::Line {
                    start: polar(r_from, from),
                });
            }
            continue;
        }
        #[allow(clippy::float_cmp)]
        if r_from == r_to {
            segments.push(Segment::Arc {
                radius: r_from,
                from,
                to,
            });
            continue;
        }
        let slope = (r_to - r_from) / (to - from);
        let radius = move |t: f64| slope.mul_add(t - from, r_from);
        // ‖g''''‖ = √(r² + 16 r'²) for g = r·e^{iθ} with r linear in θ.
        let r_max = r_from.max(r_to);
        let bound = r_max.hypot(4.0 * slope);
        let fit = fit_cubic_hermite(
            |t| {
                let r = radius(t);
                let (sin, cos) = t.sin_cos();
                (
                    Point3::new(r * cos, r * sin, 0.0),
                    Vec3::new(
                        slope.mul_add(cos, -r * sin),
                        slope.mul_add(sin, r * cos),
                        0.0,
                    ),
                )
            },
            from,
            to,
            bound,
            tolerance,
            MAX_SPANS_PER_FLANK,
        )
        .map_err(|error| match error {
            remus_math::MathError::ResourceLimitExceeded { .. } => ThreadError::refused(
                super::ThreadRefusal::ToleranceTooTight,
                format!(
                    "a flank needs more than {MAX_SPANS_PER_FLANK} spans for a tolerance of {tolerance}"
                ),
            ),
            other => other.into(),
        })?;
        // Snap the ends onto the exact vertices the neighbouring pieces use.
        let mut points = fit.curve.control_points().to_vec();
        let last = points.len() - 1;
        points[0] = polar(r_from, from);
        points[last] = polar(r_to, to);
        let curve = NurbsCurve::new(
            fit.curve.degree(),
            fit.curve.knots().to_vec(),
            points,
            fit.curve.weights().to_vec(),
        )?;
        remainder_bound = remainder_bound.max(fit.remainder_bound);
        spans_per_flank = spans_per_flank.max(fit.spans);
        segments.push(Segment::Flank(curve));
    }
    Ok(Outline {
        segments,
        remainder_bound,
        spans_per_flank,
    })
}
