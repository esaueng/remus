//! Straight bevel gears: teeth on a pitch cone, tapering to its apex.
//!
//! [`make_bevel_gear`] builds a gear whose pitch cone meets a mate's at a
//! shaft angle Σ. The pitch angle follows from the tooth ratio,
//! `tan δ = sin Σ / (z₂/z₁ + cos Σ)`, and every tooth surface is a cone with
//! the gear's apex as its vertex, so the teeth taper to the apex.
//!
//! - **Flanks** are the exact straight-bevel tooth form: cones over a
//!   *spherical involute*, the curve a great circle traces rolling on the
//!   base cone (`sin δ_b = sin δ · cos α`). Like the spur involute it is not
//!   rational, so each flank is a certified cubic Hermite fit of the
//!   spherical involute, centrally projected (exactly, as a rational curve)
//!   onto the heel plane and swept toward the apex.
//! - **Depth** tapers to the apex: the tip and root cones are the pitch cone
//!   opened and closed by the addendum and dedendum angles
//!   `atan(h_a·m / A)` and `atan(h_f·m / A)` at the outer cone distance `A`.
//! - **Heel and toe** are planes perpendicular to the axis, not back and
//!   front cones, so the outer ends of the teeth run out to a flat back; the
//!   working flanks are unaffected.
//!
//! The body is the heel-plane outline extruded toward the apex
//! ([tapered extrusion](crate::extrude::taper)): lands become exact cones,
//! radial flank pieces planes through the apex, and flanks ruled NURBS that
//! are exact sweeps of the projected fit. The root is sharp (a radial flank
//! below the base cone, no fillet). Module, pitch and tip diameters are
//! measured in the heel plane; face width along the pitch cone.

use std::f64::consts::{FRAC_PI_2, PI};

use remus_math::nurbs::NurbsCurve;
use remus_math::nurbs::hermite_fit::fit_cubic_hermite;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

use super::profile::{
    FittedFlank, MAX_SPANS_PER_FLANK, Outline, Segment, curve_end, map_curve, mirror, polar,
    replicate,
};
use super::{
    DEFAULT_TOLERANCE_PER_MODULE, Frame, GearError, GearFace, GearFaceRole, GearKind,
    GearPlacement, GearRefusal, GearWarning, LocalOutline, Sweep, THIN_TIP_MODULES, ToothSide,
    gear_body, non_negative, positive, require,
};

/// Largest face (tip cone) angle the flat-heel construction builds: beyond
/// it the heel plane meets the tip cone too far out (toward a crown gear).
pub const MAX_FACE_ANGLE_DEG: f64 = 80.0;

/// Smallest tooth count on either gear of the pair.
const MIN_TEETH: u32 = 4;

/// A land narrower than this fraction of its period is treated as closed.
const MIN_LAND_FRACTION: f64 = 1e-9;

/// How far a fitted point may stray from the unit sphere when bounding the
/// projection's Lipschitz constant; the fit tolerance is kept below it.
const SPHERE_TUBE: f64 = 1e-3;

/// A straight bevel gear's parameters (ISO 23509 terms, heel values).
#[derive(Debug, Clone, PartialEq)]
pub struct BevelGearParams {
    /// Module at the heel (outer) pitch circle.
    pub module: f64,
    /// Tooth count.
    pub teeth: u32,
    /// The mating gear's tooth count, which with the shaft angle fixes the
    /// pitch angle.
    pub mate_teeth: u32,
    /// Angle between the two gears' axes, radians in `(0, π)`; `π/2` for
    /// the usual right-angle pair.
    pub shaft_angle: f64,
    /// Tooth length along the pitch cone, below the outer cone distance.
    pub face_width: f64,
    /// Pressure angle α in radians, in `(0, π/4]`.
    pub pressure_angle: f64,
    /// Addendum coefficient `h_a` at the heel (1.0).
    pub addendum: f64,
    /// Dedendum coefficient `h_f` at the heel (1.25).
    pub dedendum: f64,
    /// Circumferential tooth thinning at the heel pitch circle (length).
    pub backlash: f64,
    /// Flank deviation tolerance; `None` uses
    /// [`DEFAULT_TOLERANCE_PER_MODULE`]·m.
    pub tolerance: Option<f64>,
    /// Optional bore diameter, a straight cylinder through both ends.
    pub bore_diameter: Option<f64>,
}

impl BevelGearParams {
    /// A right-angle pair member with ISO defaults (α = 20°, `h_a` 1.0,
    /// `h_f` 1.25).
    #[must_use]
    pub fn standard(module: f64, teeth: u32, mate_teeth: u32, face_width: f64) -> Self {
        Self {
            module,
            teeth,
            mate_teeth,
            shaft_angle: FRAC_PI_2,
            face_width,
            pressure_angle: 20.0_f64.to_radians(),
            addendum: 1.0,
            dedendum: 1.25,
            backlash: 0.0,
            tolerance: None,
            bore_diameter: None,
        }
    }
}

/// A straight bevel gear's closed-form dimensions.
#[derive(Debug, Clone, PartialEq)]
pub struct BevelGearGeometry {
    /// Heel module.
    pub module: f64,
    /// Pitch angle δ (radians).
    pub pitch_angle: f64,
    /// The mate's pitch angle `Σ − δ`.
    pub mate_pitch_angle: f64,
    /// Base cone angle `δ_b`, `sin δ_b = sin δ cos α`.
    pub base_cone_angle: f64,
    /// Face (tip cone) angle `δ + atan(h_a·m/A)`.
    pub face_angle: f64,
    /// Root cone angle `δ − atan(h_f·m/A)`.
    pub root_angle: f64,
    /// Outer cone distance `A = r/sin δ`, apex to heel pitch circle.
    pub outer_cone_distance: f64,
    /// Inner cone distance `A − b`, apex to toe pitch circle.
    pub inner_cone_distance: f64,
    /// Heel pitch diameter `m·z`.
    pub pitch_diameter: f64,
    /// Tip circle diameter in the heel plane.
    pub tip_diameter: f64,
    /// Root circle diameter in the heel plane.
    pub root_diameter: f64,
    /// Axial distance from the heel plane to the apex, `A cos δ`.
    pub apex_distance: f64,
    /// Axial length from heel to toe, `b cos δ`.
    pub height: f64,
    /// Heel circular pitch `π·m`.
    pub circular_pitch: f64,
    /// Tooth thickness along the heel pitch circle.
    pub tooth_thickness: f64,
    /// Tooth thickness along the tip circle in the heel plane; non-positive
    /// means pointed.
    pub tip_thickness: f64,
    /// Tooth count of the back-cone equivalent spur gear, `z/cos δ`.
    pub virtual_teeth: f64,
    /// Guideline violations.
    pub warnings: Vec<GearWarning>,
}

/// A built straight bevel gear.
#[derive(Debug, Clone, PartialEq)]
pub struct BevelGearOutcome {
    /// The gear body.
    pub solid: SolidId,
    /// Its dimensions and warnings.
    pub geometry: BevelGearGeometry,
    /// Proven bound on any flank's distance from the exact spherical-involute
    /// cone, including a floating-point allowance.
    pub flank_deviation_bound: f64,
    /// Hermite spans per flank.
    pub spans_per_flank: usize,
    /// The role of every face. `cap.start` is the heel, `cap.end` the toe.
    pub face_roles: Vec<(FaceId, GearFace)>,
}

/// The spherical involute of the base cone, on the unit sphere centered at
/// the apex with `z` along the axis toward the heel: rolling angle `φ`,
/// `ψ = φ·sin δ_b`.
#[derive(Debug, Clone, Copy)]
struct SphericalInvolute {
    sin_base: f64,
    cos_base: f64,
}

impl SphericalInvolute {
    fn new(base_cone_angle: f64) -> Self {
        let (sin_base, cos_base) = base_cone_angle.sin_cos();
        Self { sin_base, cos_base }
    }

    /// The point and derivative at rolling angle `φ`, starting at
    /// `(sin δ_b, 0, cos δ_b)` and winding counter-clockwise about `z`.
    ///
    /// `(x, y) = A·e(ω₁φ) + B·e(ω₂φ)` with `A = (1 + s)/2`, `B = (s − 1)/2`,
    /// `ω₁ = 1 − s`, `ω₂ = 1 + s`, `s = sin δ_b`, and
    /// `z = cos δ_b · cos(sφ)`.
    fn sample(self, phi: f64) -> (Point3, Vec3) {
        let s = self.sin_base;
        let (a, b) = (0.5 * (1.0 + s), 0.5 * (s - 1.0));
        let (w1, w2) = (1.0 - s, 1.0 + s);
        let (s1, c1) = (w1 * phi).sin_cos();
        let (s2, c2) = (w2 * phi).sin_cos();
        let (sz, cz) = (s * phi).sin_cos();
        (
            Point3::new(
                a.mul_add(c1, b * c2),
                a.mul_add(s1, b * s2),
                self.cos_base * cz,
            ),
            Vec3::new(
                -(a * w1).mul_add(s1, b * w2 * s2),
                (a * w1).mul_add(c1, b * w2 * c2),
                -self.cos_base * s * sz,
            ),
        )
    }

    /// `‖P⁗‖` over all `φ`: `(|A|ω₁⁴ + |B|ω₂⁴)` across, `cos δ_b·s⁴` along.
    fn fourth_derivative_bound(self) -> f64 {
        let s = self.sin_base;
        let across = 0.5f64.mul_add(
            (1.0 + s) * (1.0 - s).powi(4),
            0.5 * (1.0 - s) * (1.0 + s).powi(4),
        );
        across.hypot(self.cos_base * s.powi(4))
    }

    /// Rolling angle at which the involute reaches polar angle `ϑ ≥ δ_b`
    /// from the axis.
    fn roll_at(self, polar: f64) -> f64 {
        (polar.cos() / self.cos_base).clamp(-1.0, 1.0).acos() / self.sin_base
    }

    /// Azimuth of the involute at polar angle `ϑ ≥ δ_b`: `φ − atan2(sin ψ,
    /// sin δ_b cos ψ)`, the spherical counterpart of `inv α`.
    fn azimuth_at(self, polar: f64) -> f64 {
        let phi = self.roll_at(polar);
        let (sin_psi, cos_psi) = (phi * self.sin_base).sin_cos();
        phi - sin_psi.atan2(self.sin_base * cos_psi)
    }
}

/// Pitch-cone quantities shared by [`bevel_gear_geometry`] and the build.
struct Cones {
    pitch: f64,
    base: f64,
    face: f64,
    root: f64,
    outer_distance: f64,
    apex_distance: f64,
    scale: f64,
    /// Half azimuth of the tooth at its base cone (or root, if deeper).
    psi0: f64,
    involute: SphericalInvolute,
}

fn validate(params: &BevelGearParams) -> Result<(), GearError> {
    positive(params.module, "module")?;
    positive(params.face_width, "face width")?;
    require(
        params.teeth >= MIN_TEETH && params.mate_teeth >= MIN_TEETH,
        || format!("both gears need at least {MIN_TEETH} teeth"),
    )?;
    require(
        params.teeth <= super::MAX_TEETH && params.mate_teeth <= super::MAX_TEETH,
        || format!("tooth counts are limited to {}", super::MAX_TEETH),
    )?;
    require(
        params.shaft_angle.is_finite() && params.shaft_angle > 0.0 && params.shaft_angle < PI,
        || "shaft angle must be in (0°, 180°)".to_owned(),
    )?;
    require(
        params.pressure_angle.is_finite()
            && params.pressure_angle > 0.0
            && params.pressure_angle <= PI / 4.0,
        || "pressure angle must be in (0°, 45°]".to_owned(),
    )?;
    positive(params.addendum, "addendum")?;
    positive(params.dedendum, "dedendum")?;
    non_negative(params.backlash, "backlash")?;
    if let Some(tolerance) = params.tolerance {
        positive(tolerance, "tolerance")?;
    }
    if let Some(bore) = params.bore_diameter {
        positive(bore, "bore diameter")?;
    }
    Ok(())
}

fn cones(params: &BevelGearParams) -> Result<Cones, GearError> {
    let m = params.module;
    let z = f64::from(params.teeth);
    let ratio = f64::from(params.mate_teeth) / z;
    let pitch = params
        .shaft_angle
        .sin()
        .atan2(ratio + params.shaft_angle.cos());
    let pitch_radius = 0.5 * m * z;
    let outer_distance = pitch_radius / pitch.sin();
    require(params.face_width < outer_distance, || {
        format!(
            "face width {} reaches the apex at outer cone distance {outer_distance}",
            params.face_width
        )
    })?;
    let face = pitch + (params.addendum * m / outer_distance).atan();
    let root = pitch - (params.dedendum * m / outer_distance).atan();
    require(face < MAX_FACE_ANGLE_DEG.to_radians(), || {
        format!(
            "the face angle {:.2}° exceeds {MAX_FACE_ANGLE_DEG}°: a flat-backed bevel gear \
             this close to a crown gear is not supported",
            face.to_degrees()
        )
    })?;
    require(root > 0.0, || {
        "the root cone closes on the axis; use more teeth or a smaller dedendum".to_owned()
    })?;
    let base = (pitch.sin() * params.pressure_angle.cos()).asin();
    let involute = SphericalInvolute::new(base);
    // Half the heel pitch-circle tooth thickness as an azimuth.
    let half_tooth = (0.5 * PI).mul_add(0.5 * m, -0.5 * params.backlash) / pitch_radius;
    Ok(Cones {
        pitch,
        base,
        face,
        root,
        outer_distance,
        apex_distance: outer_distance * pitch.cos(),
        scale: 1.0 - params.face_width / outer_distance,
        psi0: half_tooth + involute.azimuth_at(pitch),
        involute,
    })
}

/// Closed-form dimensions and guideline warnings.
///
/// # Errors
///
/// [`GearError::Refused`] with [`GearRefusal::InvalidParameter`] for
/// out-of-range parameters.
pub fn bevel_gear_geometry(params: &BevelGearParams) -> Result<BevelGearGeometry, GearError> {
    validate(params)?;
    let c = cones(params)?;
    let m = params.module;
    let z = f64::from(params.teeth);
    let h = c.apex_distance;
    let tip_radius = h * c.face.tan();
    // Azimuths survive the central projection onto the heel plane.
    let tip_half = if c.face > c.base {
        c.psi0 - c.involute.azimuth_at(c.face)
    } else {
        c.psi0
    };
    let tip_thickness = 2.0 * tip_radius * tip_half;
    let mut warnings = Vec::new();
    if tip_thickness > 0.0 && tip_thickness < THIN_TIP_MODULES * m {
        warnings.push(GearWarning::ThinTip {
            thickness: tip_thickness,
        });
    }
    Ok(BevelGearGeometry {
        module: m,
        pitch_angle: c.pitch,
        mate_pitch_angle: params.shaft_angle - c.pitch,
        base_cone_angle: c.base,
        face_angle: c.face,
        root_angle: c.root,
        outer_cone_distance: c.outer_distance,
        inner_cone_distance: c.outer_distance - params.face_width,
        pitch_diameter: m * z,
        tip_diameter: 2.0 * tip_radius,
        root_diameter: 2.0 * h * c.root.tan(),
        apex_distance: h,
        height: params.face_width * c.pitch.cos(),
        circular_pitch: PI * m,
        tooth_thickness: PI.mul_add(0.5 * m, -params.backlash),
        tip_thickness,
        virtual_teeth: z / c.pitch.cos(),
        warnings,
    })
}

/// Build a straight bevel gear.
///
/// The heel cap is centered on the placement origin, the gear tapers along
/// the placement axis toward its apex at `apex_distance`, and its first
/// tooth points along the x direction turned `phase` about the axis.
///
/// The whole construction runs in one transaction: a refusal or failure
/// leaves the topology as it was.
///
/// # Errors
///
/// [`GearError::Refused`] for out-of-range parameters or an unbuildable
/// tooth form; [`GearError::Operations`] when the extrusion fails.
pub fn make_bevel_gear(
    topo: &mut Topology,
    params: &BevelGearParams,
    placement: &GearPlacement,
) -> Result<BevelGearOutcome, GearError> {
    let geometry = bevel_gear_geometry(params)?;
    let frame = Frame::new(placement, GearKind::External)?;
    remus_topology::transaction::run_transacted(topo, |topo| {
        build(topo, params, &frame, placement.phase, &geometry)
    })
}

fn build(
    topo: &mut Topology,
    params: &BevelGearParams,
    frame: &Frame,
    phase: f64,
    geometry: &BevelGearGeometry,
) -> Result<BevelGearOutcome, GearError> {
    let c = cones(params)?;
    let tolerance = params
        .tolerance
        .unwrap_or(DEFAULT_TOLERANCE_PER_MODULE * params.module);
    let (outline, profile_bound) = heel_outline(params, &c, phase, tolerance)?;
    let bore_radius = params.bore_diameter.map(|bore| 0.5 * bore);
    let toe_root = c.scale * 0.5 * geometry.root_diameter;
    if let Some(radius) = bore_radius
        && radius >= toe_root
    {
        return Err(GearError::refused(
            GearRefusal::BoreTooLarge,
            format!(
                "bore diameter {} reaches the toe root diameter {}",
                2.0 * radius,
                2.0 * toe_root
            ),
        ));
    }
    let spans_per_flank = outline.flank.as_ref().map_or(0, |flank| flank.spans);
    let (solid, face_roles, _) = gear_body(
        topo,
        LocalOutline {
            outline,
            rim_radius: None,
            bore_radius,
        },
        frame,
        frame.x,
        geometry.height,
        Sweep::Taper {
            apex_distance: c.apex_distance,
        },
    )?;
    Ok(BevelGearOutcome {
        solid,
        geometry: geometry.clone(),
        flank_deviation_bound: profile_bound,
        spans_per_flank,
        face_roles,
    })
}

/// The heel-plane outline and the proven deviation of its flanks from the
/// exact spherical-involute cones, in model units.
///
/// The upper flank of tooth 0 is the spherical involute mirrored and turned
/// by `ψ₀` (its azimuth falls from `ψ₀` at the base cone), fitted on the
/// unit sphere and projected from the apex onto the heel plane at distance
/// `h`: `(x, y, z) ↦ h·(x/z, y/z)`, exact on control points with weights
/// `z`. The projection's Lipschitz constant on the sphere's tube below the
/// face angle, `h·(1/c + ρ/c²)` with `c` the least `z` and `ρ` the largest
/// `|(x, y)|`, turns the fit's bound into the plane's.
fn heel_outline(
    params: &BevelGearParams,
    c: &Cones,
    phase: f64,
    tolerance: f64,
) -> Result<(Outline, f64), GearError> {
    let h = c.apex_distance;
    let teeth = params.teeth;
    let half_pitch = PI / f64::from(teeth);
    if c.face <= c.base {
        return Err(GearError::refused(
            GearRefusal::TipBelowBaseCircle,
            "the face cone lies inside the base cone, so the teeth have no involute",
        ));
    }
    let start_polar = c.root.max(c.base);
    if c.psi0 - c.involute.azimuth_at(c.face) <= half_pitch * MIN_LAND_FRACTION {
        return Err(GearError::refused(
            GearRefusal::PointedTip,
            "the flanks cross before the face cone",
        ));
    }
    let root_half = c.psi0 - c.involute.azimuth_at(start_polar);
    if root_half >= half_pitch * (1.0 - MIN_LAND_FRACTION) {
        return Err(GearError::refused(
            GearRefusal::RootLandClosed,
            "adjacent teeth meet at the root cone; reduce the dedendum or backlash",
        ));
    }

    let lipschitz = {
        let least_z = c.face.cos() - SPHERE_TUBE;
        let largest_radius = c.face.sin() + SPHERE_TUBE;
        h * (1.0 / least_z + largest_radius / (least_z * least_z))
    };
    let sphere_tolerance = (tolerance / lipschitz).min(SPHERE_TUBE);
    let (s0, c0) = c.psi0.sin_cos();
    let involute = c.involute;
    let upper = move |phi: f64| {
        let (p, d) = involute.sample(phi);
        // Mirror (y → −y), then turn by ψ₀.
        let (x, y, dx, dy) = (p.x(), -p.y(), d.x(), -d.y());
        (
            Point3::new(c0.mul_add(x, -s0 * y), s0.mul_add(x, c0 * y), p.z()),
            Vec3::new(c0.mul_add(dx, -s0 * dy), s0.mul_add(dx, c0 * dy), d.z()),
        )
    };
    let fit = fit_cubic_hermite(
        upper,
        involute.roll_at(start_polar),
        involute.roll_at(c.face),
        involute.fourth_derivative_bound(),
        sphere_tolerance,
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
    let flank_curve = project(&fit.curve, h)?;
    // The bound is exact-arithmetic; allow for rounding at the tip radius.
    let tip_radius = h * c.face.tan();
    let bound = lipschitz.mul_add(fit.remainder_bound, 64.0 * f64::EPSILON * tip_radius);

    let start_point = curve_end(&flank_curve, false);
    let tip_point = curve_end(&flank_curve, true);
    let below_base = c.root < c.base;
    let root_point = if below_base {
        polar(h * c.root.tan(), c.psi0)
    } else {
        start_point
    };
    let mut period = Vec::with_capacity(5);
    if below_base {
        period.push((
            Segment::Line {
                start: mirror(root_point),
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
            curve: map_curve(&flank_curve, mirror)?,
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
            curve: flank_curve.clone(),
            reversed: true,
        },
        GearFaceRole::Flank {
            tooth: 0,
            side: ToothSide::Plus,
        },
    ));
    if below_base {
        period.push((
            Segment::Line {
                start: start_point,
                end: root_point,
            },
            GearFaceRole::FlankBase {
                tooth: 0,
                side: ToothSide::Plus,
            },
        ));
    }
    let (segments, roles) = replicate(&period, GearFaceRole::Root { gap: 0 }, teeth, phase)?;
    Ok((
        Outline {
            segments,
            roles,
            flank: Some(FittedFlank {
                curve: flank_curve,
                remainder_bound: bound,
                spans: fit.spans,
            }),
        },
        bound,
    ))
}

/// The central projection from the apex of a polynomial curve on (near) the
/// unit sphere onto the plane `z = h`: control point `P` maps to
/// `h·(P_x/P_z, P_y/P_z, 0)` with weight `P_z`, exactly.
fn project(curve: &NurbsCurve, h: f64) -> Result<NurbsCurve, GearError> {
    let mut points = Vec::with_capacity(curve.control_points().len());
    let mut weights = Vec::with_capacity(points.capacity());
    for (&p, &w) in curve.control_points().iter().zip(curve.weights()) {
        if p.z() <= 0.0 {
            return Err(GearError::invalid(
                "a flank control point lies behind the apex",
            ));
        }
        points.push(Point3::new(h * p.x() / p.z(), h * p.y() / p.z(), 0.0));
        weights.push(w * p.z());
    }
    Ok(NurbsCurve::new(
        curve.degree(),
        curve.knots().to_vec(),
        points,
        weights,
    )?)
}

#[cfg(test)]
mod tests;
