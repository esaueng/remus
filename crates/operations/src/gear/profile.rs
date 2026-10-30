//! Planar gear outlines in the gear's local frame.
//!
//! Every outline is a closed loop of [`Segment`]s in the local `z = 0`
//! plane, wound counter-clockwise about `+z`. External gears put a tooth
//! centered on `+x` (before the caller's phase), internal gears do the same
//! for one of their teeth, and a rack runs along `+x` with its teeth
//! pointing along `+y` and its pitch line on `y = 0`.
//!
//! ## Involute flank
//!
//! The involute of the base circle `r_b` is
//! `P(t) = r_b (cos t + t sin t, sin t − t cos t)` for roll angle `t ≥ 0`,
//! at radius `r_b √(1 + t²)` and polar angle `t − atan t = inv α_r`. A
//! tooth's upper flank is `U(t) = R(ψ₀) · M · P(t)` with `M` the reflection
//! `y ↦ −y` and `ψ₀ = s/(2r) + inv α` (`s` the pitch-circle thickness), so
//! its polar angle `ψ₀ − inv α_r` falls with radius. The lower flank is the
//! mirror image `y ↦ −y`.
//!
//! `‖P''''(t)‖ = r_b √(9 + t²)` exactly, and rotation and reflection keep
//! norms, so the certified Hermite fit gets a closed-form fourth-derivative
//! bound.
//!
//! ## Root fillet
//!
//! The fillet is a circle of radius `ρ` tangent to the root circle and to
//! the flank on the tooth-space side. Below the base circle the flank
//! continues as a radial line. With `B(t) = r_b (cos t, −sin t)` the mirrored
//! base-circle contact point, the flank normal through `U(t)` is the line of
//! action `(sin t, cos t)` (before `R(ψ₀)`), orthogonal to `B`, so the
//! fillet center `B + (r_b t ± ρ)(sin t, cos t)` sits at radius
//! `√(r_b² + (r_b t ± ρ)²)`. Equating that to the root-side center radius
//! gives the tangency roll angle in closed form.

use std::f64::consts::PI;

use remus_math::nurbs::NurbsCurve;
use remus_math::nurbs::hermite_fit::fit_cubic_hermite;
use remus_math::vec::{Point3, Vec3};

use super::{GearEnd, GearError, GearFaceRole, GearRefusal, ToothSide};

/// Uniform Hermite spans allowed per flank before the tolerance refuses.
pub(super) const MAX_SPANS_PER_FLANK: usize = 256;

/// Smallest angular gap, relative to a full pitch, still treated as a land.
const MIN_LAND_FRACTION: f64 = 1e-9;

/// One boundary piece of a local outline.
#[derive(Debug, Clone)]
pub(super) enum Segment {
    /// A straight piece.
    Line { start: Point3, end: Point3 },
    /// A circular arc shorter than half a turn, swept from `start` to `end`
    /// on the side that keeps it shorter than half a turn.
    Arc {
        center: Point3,
        start: Point3,
        end: Point3,
    },
    /// An involute flank, parameterized outward; `reversed` walks it inward.
    Flank { curve: NurbsCurve, reversed: bool },
}

impl Segment {
    pub(super) fn start(&self) -> Point3 {
        match self {
            Self::Line { start, .. } | Self::Arc { start, .. } => *start,
            Self::Flank { curve, reversed } => curve_end(curve, *reversed),
        }
    }

    pub(super) fn end(&self) -> Point3 {
        match self {
            Self::Line { end, .. } | Self::Arc { end, .. } => *end,
            Self::Flank { curve, reversed } => curve_end(curve, !*reversed),
        }
    }

    /// The same piece walked the other way.
    pub(super) fn reversed(self) -> Self {
        match self {
            Self::Line { start, end } => Self::Line {
                start: end,
                end: start,
            },
            Self::Arc { center, start, end } => Self::Arc {
                center,
                start: end,
                end: start,
            },
            Self::Flank { curve, reversed } => Self::Flank {
                curve,
                reversed: !reversed,
            },
        }
    }

    fn mapped(&self, map: impl Fn(Point3) -> Point3) -> Result<Self, GearError> {
        Ok(match self {
            Self::Line { start, end } => Self::Line {
                start: map(*start),
                end: map(*end),
            },
            Self::Arc { center, start, end } => Self::Arc {
                center: map(*center),
                start: map(*start),
                end: map(*end),
            },
            Self::Flank { curve, reversed } => Self::Flank {
                curve: map_curve(curve, &map)?,
                reversed: *reversed,
            },
        })
    }
}

/// First (`false`) or last (`true`) control point: the curve's end points.
fn curve_end(curve: &NurbsCurve, last: bool) -> Point3 {
    let points = curve.control_points();
    if last {
        points[points.len() - 1]
    } else {
        points[0]
    }
}

pub(super) fn map_curve(
    curve: &NurbsCurve,
    map: impl Fn(Point3) -> Point3,
) -> Result<NurbsCurve, GearError> {
    Ok(NurbsCurve::new(
        curve.degree(),
        curve.knots().to_vec(),
        curve.control_points().iter().map(|&p| map(p)).collect(),
        curve.weights().to_vec(),
    )?)
}

fn mirror(p: Point3) -> Point3 {
    Point3::new(p.x(), -p.y(), 0.0)
}

fn rotate(p: Point3, cos: f64, sin: f64) -> Point3 {
    Point3::new(
        cos.mul_add(p.x(), -sin * p.y()),
        sin.mul_add(p.x(), cos * p.y()),
        0.0,
    )
}

fn polar(radius: f64, angle: f64) -> Point3 {
    let (sin, cos) = angle.sin_cos();
    Point3::new(radius * cos, radius * sin, 0.0)
}

fn angle_of(p: Point3) -> f64 {
    p.y().atan2(p.x())
}

fn radius_of(p: Point3) -> f64 {
    p.x().hypot(p.y())
}

/// `inv α_r = tan α_r − α_r` at radius `r ≥ r_b`, as `t − atan t`.
pub(super) fn involute_function(base_radius: f64, radius: f64) -> f64 {
    let t = roll_angle(base_radius, radius);
    t - t.atan()
}

/// Roll angle at which the involute of `r_b` reaches `r ≥ r_b`.
fn roll_angle(base_radius: f64, radius: f64) -> f64 {
    ((radius / base_radius).powi(2) - 1.0).max(0.0).sqrt()
}

/// Upper flank `U(t) = R(ψ₀)·M·P(t)` and its derivative.
fn upper_flank(base_radius: f64, psi0: f64) -> impl Fn(f64) -> (Point3, Vec3) {
    let (s0, c0) = psi0.sin_cos();
    move |t: f64| {
        let (s, c) = t.sin_cos();
        let x = base_radius * t.mul_add(s, c);
        let y = -base_radius * (-t).mul_add(c, s);
        let dx = base_radius * t * c;
        let dy = -base_radius * t * s;
        (
            Point3::new(c0.mul_add(x, -s0 * y), s0.mul_add(x, c0 * y), 0.0),
            Vec3::new(c0.mul_add(dx, -s0 * dy), s0.mul_add(dx, c0 * dy), 0.0),
        )
    }
}

/// Upper fillet center for a roll angle `t` and signed offset
/// `r_b t + offset` along the line of action.
fn fillet_center(base_radius: f64, psi0: f64, t: f64, along_line: f64) -> Point3 {
    let (s, c) = t.sin_cos();
    let local = Point3::new(
        base_radius.mul_add(c, along_line * s),
        (-base_radius).mul_add(s, along_line * c),
        0.0,
    );
    let (s0, c0) = psi0.sin_cos();
    rotate(local, c0, s0)
}

/// The fitted upper flank over `[t0, t1]`.
pub(super) struct FittedFlank {
    pub(super) curve: NurbsCurve,
    pub(super) remainder_bound: f64,
    pub(super) spans: usize,
}

fn fit_upper_flank(
    base_radius: f64,
    psi0: f64,
    t0: f64,
    t1: f64,
    tolerance: f64,
) -> Result<FittedFlank, GearError> {
    let bound = base_radius * t1.mul_add(t1, 9.0).sqrt();
    let fit = fit_cubic_hermite(
        upper_flank(base_radius, psi0),
        t0,
        t1,
        bound,
        tolerance,
        MAX_SPANS_PER_FLANK,
    )
    .map_err(|error| match error {
        remus_math::MathError::ResourceLimitExceeded { .. } => GearError::refused(
            GearRefusal::ToleranceTooTight,
            format!(
                "meeting the flank tolerance {tolerance:e} needs more than \
                 {MAX_SPANS_PER_FLANK} spans per flank"
            ),
        ),
        other => GearError::from(other),
    })?;
    Ok(FittedFlank {
        curve: fit.curve,
        remainder_bound: fit.remainder_bound,
        spans: fit.spans,
    })
}

/// A full outline plus what the flank fit proved.
pub(super) struct Outline {
    pub(super) segments: Vec<Segment>,
    /// The face role each segment's extruded side takes, parallel to
    /// `segments`.
    pub(super) roles: Vec<GearFaceRole>,
    pub(super) flank: Option<FittedFlank>,
}

/// Radii and tooth angle for one circular gear's tooth (or tooth space).
pub(super) struct CircularTooth {
    pub(super) teeth: u32,
    pub(super) base_radius: f64,
    /// External: tip circle. Internal: the inner (tooth tip) circle.
    pub(super) inner_or_tip_radius: f64,
    /// External: root circle. Internal: the outer (root) circle.
    pub(super) root_radius: f64,
    pub(super) fillet_radius: f64,
    /// Half-angle term `s/(2r) + inv α` of the tooth (external) or tooth
    /// space (internal).
    pub(super) psi0: f64,
    pub(super) phase: f64,
    pub(super) tolerance: f64,
}

/// External outline: one tooth per period, root lands between them.
pub(super) fn external_outline(tooth: &CircularTooth) -> Result<Outline, GearError> {
    let CircularTooth {
        teeth,
        base_radius: rb,
        inner_or_tip_radius: ra,
        root_radius: rf,
        fillet_radius: rho,
        psi0,
        ..
    } = *tooth;
    let half_pitch = PI / f64::from(teeth);
    let t_tip = roll_angle(rb, ra);
    let has_fillet = rho > 0.0;

    // Where the flank leaves the fillet (or the root circle).
    let involute_meets_fillet = rf.mul_add(rf, 2.0 * rf * rho) >= rb * rb;
    let (t_start, center) = if involute_meets_fillet {
        let along = (rf + rho).mul_add(rf + rho, -rb * rb).sqrt();
        let t = (along - rho) / rb;
        (t, fillet_center(rb, psi0, t, along))
    } else {
        let center_radius = rf + rho;
        (
            0.0,
            polar(center_radius, psi0 + (rho / center_radius).asin()),
        )
    };
    if t_tip - t_start <= MIN_LAND_FRACTION {
        return Err(GearError::refused(
            GearRefusal::FlankTooShort,
            "the root fillet reaches the tip circle, leaving no involute flank",
        ));
    }
    let flank = fit_upper_flank(rb, psi0, t_start, t_tip, tooth.tolerance)?;
    let start_point = curve_end(&flank.curve, false);
    let tip_point = curve_end(&flank.curve, true);
    let tangent_point = if involute_meets_fillet {
        start_point
    } else {
        let tangent_radius = (rf + rho).mul_add(rf + rho, -rho * rho).sqrt();
        polar(tangent_radius, psi0)
    };
    let root_point = if has_fillet {
        let scale = rf / radius_of(center);
        Point3::new(center.x() * scale, center.y() * scale, 0.0)
    } else {
        tangent_point
    };
    if angle_of(root_point) >= half_pitch * (1.0 - MIN_LAND_FRACTION) {
        return Err(GearError::refused(
            GearRefusal::RootLandClosed,
            "adjacent teeth meet at the root circle; reduce the root fillet, \
             dedendum or backlash",
        ));
    }
    if angle_of(tip_point) <= half_pitch * MIN_LAND_FRACTION {
        return Err(GearError::refused(
            GearRefusal::PointedTip,
            "the flanks cross before the tip circle",
        ));
    }

    let mut period = Vec::with_capacity(7);
    if has_fillet {
        period.push((
            Segment::Arc {
                center: mirror(center),
                start: mirror(root_point),
                end: mirror(tangent_point),
            },
            GearFaceRole::Fillet {
                tooth: 0,
                side: ToothSide::Minus,
            },
        ));
    }
    if !involute_meets_fillet {
        period.push((
            Segment::Line {
                start: mirror(tangent_point),
                end: mirror(start_point),
            },
            GearFaceRole::FlankBase {
                tooth: 0,
                side: ToothSide::Minus,
            },
        ));
    }
    period.push((
        Segment::Flank {
            curve: map_curve(&flank.curve, mirror)?,
            reversed: false,
        },
        GearFaceRole::Flank {
            tooth: 0,
            side: ToothSide::Minus,
        },
    ));
    period.push((
        Segment::Arc {
            center: Point3::new(0.0, 0.0, 0.0),
            start: mirror(tip_point),
            end: tip_point,
        },
        GearFaceRole::Tip { tooth: 0 },
    ));
    period.push((
        Segment::Flank {
            curve: flank.curve.clone(),
            reversed: true,
        },
        GearFaceRole::Flank {
            tooth: 0,
            side: ToothSide::Plus,
        },
    ));
    if !involute_meets_fillet {
        period.push((
            Segment::Line {
                start: start_point,
                end: tangent_point,
            },
            GearFaceRole::FlankBase {
                tooth: 0,
                side: ToothSide::Plus,
            },
        ));
    }
    if has_fillet {
        period.push((
            Segment::Arc {
                center,
                start: tangent_point,
                end: root_point,
            },
            GearFaceRole::Fillet {
                tooth: 0,
                side: ToothSide::Plus,
            },
        ));
    }
    let (segments, roles) = replicate(&period, GearFaceRole::Root { gap: 0 }, teeth, tooth.phase)?;
    Ok(Outline {
        segments,
        roles,
        flank: Some(flank),
    })
}

/// Internal outline (the hole boundary, wound counter-clockwise): one tooth
/// space per period, tooth tip lands between them.
pub(super) fn internal_outline(space: &CircularTooth) -> Result<Outline, GearError> {
    let CircularTooth {
        teeth,
        base_radius: rb,
        inner_or_tip_radius: r_tip,
        root_radius: r_root,
        fillet_radius: rho,
        psi0,
        ..
    } = *space;
    let half_pitch = PI / f64::from(teeth);
    let has_fillet = rho > 0.0;
    if r_root - rho < rb {
        return Err(GearError::refused(
            GearRefusal::FlankTooShort,
            "the root fillet does not reach the involute above the base circle",
        ));
    }
    let along = (r_root - rho).mul_add(r_root - rho, -rb * rb).sqrt();
    let t_end = (rho + along) / rb;
    let center = fillet_center(rb, psi0, t_end, along);
    let involute_at_tip = r_tip >= rb;
    let t_begin = if involute_at_tip {
        roll_angle(rb, r_tip)
    } else {
        0.0
    };
    if t_end - t_begin <= MIN_LAND_FRACTION {
        return Err(GearError::refused(
            GearRefusal::FlankTooShort,
            "the root fillet reaches the tip circle, leaving no involute flank",
        ));
    }
    let flank = fit_upper_flank(rb, psi0, t_begin, t_end, space.tolerance)?;
    let start_point = curve_end(&flank.curve, false);
    let tangent_point = curve_end(&flank.curve, true);
    let tip_corner = if involute_at_tip {
        start_point
    } else {
        polar(r_tip, psi0)
    };
    let root_point = if has_fillet {
        let scale = r_root / radius_of(center);
        Point3::new(center.x() * scale, center.y() * scale, 0.0)
    } else {
        tangent_point
    };
    if angle_of(root_point) <= half_pitch * MIN_LAND_FRACTION {
        return Err(GearError::refused(
            GearRefusal::RootLandClosed,
            "the tooth space closes before the root circle; reduce the root \
             fillet or dedendum",
        ));
    }
    if angle_of(tip_corner) >= half_pitch * (1.0 - MIN_LAND_FRACTION) {
        return Err(GearError::refused(
            GearRefusal::PointedTip,
            "adjacent tooth spaces meet before the tip circle",
        ));
    }

    let mut period = Vec::with_capacity(7);
    if !involute_at_tip {
        period.push((
            Segment::Line {
                start: mirror(tip_corner),
                end: mirror(start_point),
            },
            GearFaceRole::FlankBase {
                tooth: 0,
                side: ToothSide::Plus,
            },
        ));
    }
    period.push((
        Segment::Flank {
            curve: map_curve(&flank.curve, mirror)?,
            reversed: false,
        },
        GearFaceRole::Flank {
            tooth: 0,
            side: ToothSide::Plus,
        },
    ));
    if has_fillet {
        period.push((
            Segment::Arc {
                center: mirror(center),
                start: mirror(tangent_point),
                end: mirror(root_point),
            },
            GearFaceRole::Fillet {
                tooth: 0,
                side: ToothSide::Plus,
            },
        ));
    }
    period.push((
        Segment::Arc {
            center: Point3::new(0.0, 0.0, 0.0),
            start: mirror(root_point),
            end: root_point,
        },
        GearFaceRole::Root { gap: 0 },
    ));
    if has_fillet {
        period.push((
            Segment::Arc {
                center,
                start: root_point,
                end: tangent_point,
            },
            GearFaceRole::Fillet {
                tooth: 1,
                side: ToothSide::Minus,
            },
        ));
    }
    period.push((
        Segment::Flank {
            curve: flank.curve.clone(),
            reversed: true,
        },
        GearFaceRole::Flank {
            tooth: 1,
            side: ToothSide::Minus,
        },
    ));
    if !involute_at_tip {
        period.push((
            Segment::Line {
                start: start_point,
                end: tip_corner,
            },
            GearFaceRole::FlankBase {
                tooth: 1,
                side: ToothSide::Minus,
            },
        ));
    }
    // Spaces sit half a pitch off the teeth, so a tooth lands on the phase.
    let (segments, roles) = replicate(
        &period,
        GearFaceRole::Tip { tooth: 1 },
        teeth,
        space.phase + half_pitch,
    )?;
    Ok(Outline {
        segments,
        roles,
        flank: Some(flank),
    })
}

/// Rotate one period `teeth` times and join consecutive periods with an arc
/// about the center (a root land for external gears, a tip land for
/// internal ones). Role tooth indices in `period` and `between` are offsets
/// from the period's own index.
fn replicate(
    period: &[(Segment, GearFaceRole)],
    between: GearFaceRole,
    teeth: u32,
    phase: f64,
) -> Result<(Vec<Segment>, Vec<GearFaceRole>), GearError> {
    let pitch = 2.0 * PI / f64::from(teeth);
    let rotation = |k: u32| {
        let (sin, cos) = f64::from(k).mul_add(pitch, phase).sin_cos();
        move |p: Point3| rotate(p, cos, sin)
    };
    let first_start = period
        .first()
        .map(|(piece, _)| piece.start())
        .ok_or_else(|| GearError::refused(GearRefusal::InvalidParameter, "empty tooth period"))?;
    let last_end = period
        .last()
        .map(|(piece, _)| piece.end())
        .ok_or_else(|| GearError::refused(GearRefusal::InvalidParameter, "empty tooth period"))?;
    let capacity = (period.len() + 1) * teeth as usize;
    let mut segments = Vec::with_capacity(capacity);
    let mut roles = Vec::with_capacity(capacity);
    for k in 0..teeth {
        let here = rotation(k);
        for (piece, role) in period {
            segments.push(piece.mapped(here)?);
            roles.push(role.shifted(k, teeth));
        }
        let next = rotation((k + 1) % teeth);
        segments.push(Segment::Arc {
            center: Point3::new(0.0, 0.0, 0.0),
            start: here(last_end),
            end: next(first_start),
        });
        roles.push(between.shifted(k, teeth));
    }
    Ok((segments, roles))
}

/// Straight-sided rack tooth data in the rack's local frame.
pub(super) struct RackTooth {
    pub(super) teeth: u32,
    pub(super) pitch: f64,
    pub(super) thickness: f64,
    pub(super) tip_height: f64,
    pub(super) root_depth: f64,
    pub(super) back_depth: f64,
    pub(super) pressure_angle: f64,
    pub(super) fillet_radius: f64,
}

/// Rack outline, counter-clockwise: back, right end, the toothed top from
/// right to left, left end. Both ends cut through the middle of a root space.
pub(super) fn rack_outline(rack: &RackTooth) -> Result<Outline, GearError> {
    let RackTooth {
        teeth,
        pitch,
        thickness,
        tip_height: ya,
        root_depth,
        back_depth,
        pressure_angle,
        fillet_radius: rho,
    } = *rack;
    let yf = -root_depth;
    let yb = yf - back_depth;
    let (sin_a, cos_a) = pressure_angle.sin_cos();
    let tan_a = sin_a / cos_a;
    let half_tip = thickness.mul_add(0.5, -ya * tan_a);
    if half_tip <= pitch * MIN_LAND_FRACTION {
        return Err(GearError::refused(
            GearRefusal::PointedTip,
            "the rack flanks cross below the tip line",
        ));
    }
    let has_fillet = rho > 0.0;
    // Right flank of a tooth centered on x = 0: x = s/2 − y·tan α.
    let flank_x = |y: f64| thickness.mul_add(0.5, -y * tan_a);
    let y_tangent = rho.mul_add(1.0 - sin_a, yf);
    if ya - y_tangent <= pitch * MIN_LAND_FRACTION {
        return Err(GearError::refused(
            GearRefusal::FlankTooShort,
            "the root fillet reaches the tip line, leaving no flank",
        ));
    }
    let tangent = Point3::new(flank_x(y_tangent), y_tangent, 0.0);
    let center = Point3::new(rho.mul_add(cos_a, tangent.x()), yf + rho, 0.0);
    let root = if has_fillet {
        Point3::new(center.x(), yf, 0.0)
    } else {
        tangent
    };
    if root.x() >= 0.5 * pitch * (1.0 - MIN_LAND_FRACTION) {
        return Err(GearError::refused(
            GearRefusal::RootLandClosed,
            "adjacent rack teeth meet at the root line; reduce the root fillet, \
             dedendum or backlash",
        ));
    }
    let tip = Point3::new(half_tip, ya, 0.0);
    let left = |p: Point3| Point3::new(-p.x(), p.y(), 0.0);
    let shift = |p: Point3, dx: f64| Point3::new(p.x() + dx, p.y(), 0.0);

    let length = f64::from(teeth) * pitch;
    let mut segments = Vec::with_capacity(4 + 6 * teeth as usize);
    let mut roles = Vec::with_capacity(4 + 6 * teeth as usize);
    let mut push = |segment: Segment, role: GearFaceRole| {
        segments.push(segment);
        roles.push(role);
    };
    push(
        Segment::Line {
            start: Point3::new(0.0, yb, 0.0),
            end: Point3::new(length, yb, 0.0),
        },
        GearFaceRole::RackBack,
    );
    push(
        Segment::Line {
            start: Point3::new(length, yb, 0.0),
            end: Point3::new(length, yf, 0.0),
        },
        GearFaceRole::RackEnd(GearEnd::End),
    );
    let mut cursor = Point3::new(length, yf, 0.0);
    for k in (0..teeth).rev() {
        let dx = (f64::from(k) + 0.5) * pitch;
        let right_root = shift(root, dx);
        push(
            Segment::Line {
                start: cursor,
                end: right_root,
            },
            if k + 1 == teeth {
                GearFaceRole::RackRootEnd(GearEnd::End)
            } else {
                GearFaceRole::Root { gap: k }
            },
        );
        if has_fillet {
            push(
                Segment::Arc {
                    center: shift(center, dx),
                    start: right_root,
                    end: shift(tangent, dx),
                },
                GearFaceRole::Fillet {
                    tooth: k,
                    side: ToothSide::Plus,
                },
            );
        }
        push(
            Segment::Line {
                start: shift(tangent, dx),
                end: shift(tip, dx),
            },
            GearFaceRole::Flank {
                tooth: k,
                side: ToothSide::Plus,
            },
        );
        push(
            Segment::Line {
                start: shift(tip, dx),
                end: shift(left(tip), dx),
            },
            GearFaceRole::Tip { tooth: k },
        );
        push(
            Segment::Line {
                start: shift(left(tip), dx),
                end: shift(left(tangent), dx),
            },
            GearFaceRole::Flank {
                tooth: k,
                side: ToothSide::Minus,
            },
        );
        if has_fillet {
            push(
                Segment::Arc {
                    center: shift(left(center), dx),
                    start: shift(left(tangent), dx),
                    end: shift(left(root), dx),
                },
                GearFaceRole::Fillet {
                    tooth: k,
                    side: ToothSide::Minus,
                },
            );
        }
        cursor = shift(left(root), dx);
    }
    push(
        Segment::Line {
            start: cursor,
            end: Point3::new(0.0, yf, 0.0),
        },
        GearFaceRole::RackRootEnd(GearEnd::Start),
    );
    push(
        Segment::Line {
            start: Point3::new(0.0, yf, 0.0),
            end: Point3::new(0.0, yb, 0.0),
        },
        GearFaceRole::RackEnd(GearEnd::Start),
    );
    Ok(Outline {
        segments,
        roles,
        flank: None,
    })
}
