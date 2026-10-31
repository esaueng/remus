//! Cylindrical involute (ZI) worms and the helical wheels that mesh with
//! them.
//!
//! A ZI worm's thread flanks are involute helicoids, so the worm is an
//! involute helical gear with `z₁` teeth (the starts) and helix angle
//! `β = 90° − γ`, where `γ` is the lead angle. [`make_worm`] builds it
//! through the helical construction (one transverse outline, one twisted
//! extrusion), and its flanks carry the same certified bound against the
//! exact involute helicoid.
//!
//! Conventions (ISO/TR 10828, DIN 3975), in the worm's axial section:
//!
//! - axial module `m_x`, diameter factor `q`, reference diameter
//!   `d₁ = q·m_x`;
//! - lead angle `tan γ = z₁/q`, axial pitch `p_x = π·m_x`, lead
//!   `p_z = z₁·p_x`;
//! - normal module `m_n = m_x·cos γ`; `pressure_angle` is the normal one
//!   `α_n`;
//! - addendum `h_a·m_x`, dedendum `h_f·m_x`, root fillet radius `ρ·m_x`
//!   (the fillet lies in the transverse section, as on a helical gear);
//! - `backlash` `j_x` thins the thread along the axis at the reference
//!   cylinder, so the axial thread thickness there is `p_x/2 − j_x`.
//!
//! The matching wheel ([`worm_wheel`]) is the involute helical gear of the
//! worm's hand with helix angle `γ` and normal module `m_n`, so its
//! transverse module is `m_x`. With the axes crossed at 90° and
//! `(d₁ + d₂)/2` apart, worm and wheel are a crossed helical pair: their
//! flanks are conjugate, touching at a point. A hobbed (throated, enveloping)
//! wheel touches the worm along lines and carries more load; this wheel has
//! the right kinematics, not a hobbed wheel's contact.
//!
//! A throated wheel ([`make_throated_worm_wheel`]) wraps its tips around the
//! worm: over the throat they follow the torus that keeps a constant
//! clearance from the worm's root, and outside it they stay on an outside
//! cylinder. Its flanks are still the helical wheel's, so it is the same
//! crossed helical drive; the throat only gives the worm more wheel to run
//! in.
//!
//! The thread ends where the end caps cut it, perpendicular to the axis.

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

use remus_topology::Topology;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

use super::throat::{Throat, cut_throat};
use super::{
    DEFAULT_TOLERANCE_PER_MODULE, Frame, GearError, GearFace, GearGeometry, GearKind, GearOutcome,
    GearParams, GearPlacement, GearWarning, Hand, MAX_TEETH, THIN_RIM_MODULES, THIN_TIP_MODULES,
    circular, flank_bound, flank_tolerance, gear_body, gear_geometry, helical_geometry,
    local_outline, non_negative, positive, require, twist,
};

/// Largest supported number of thread starts.
pub const MAX_WORM_STARTS: u32 = 10;

/// Largest supported lead angle, degrees: the matching wheel's helix angle,
/// which the helical construction caps at
/// [`MAX_HELIX_ANGLE_DEG`](super::MAX_HELIX_ANGLE_DEG).
pub const MAX_LEAD_ANGLE_DEG: f64 = super::MAX_HELIX_ANGLE_DEG;

/// Uniform Hermite spans allowed along a worm's axis before the tolerance
/// refuses. A worm turns many times along its length, so it needs more than
/// a helical gear.
const MAX_WORM_SPANS: usize = 4096;

/// A cylindrical involute (ZI) worm's parameters. Lengths are model units;
/// coefficients are multiples of the axial module.
#[derive(Debug, Clone, PartialEq)]
pub struct WormParams {
    /// Axial module `m_x`.
    pub module: f64,
    /// Number of thread starts `z₁`.
    pub starts: u32,
    /// Diameter factor `q = d₁/m_x`.
    pub diameter_factor: f64,
    /// Normal pressure angle `α_n` in radians, in `(0, π/4]`.
    pub pressure_angle: f64,
    /// Length along the worm axis.
    pub length: f64,
    /// Addendum coefficient `h_a` (1.0).
    pub addendum: f64,
    /// Dedendum coefficient `h_f` (1.25).
    pub dedendum: f64,
    /// Root fillet radius coefficient (0.38); zero for a sharp root.
    pub root_fillet: f64,
    /// Axial thinning of the thread at the reference cylinder (length).
    pub backlash: f64,
    /// Hand of the thread.
    pub hand: Hand,
    /// Flank deviation tolerance; `None` uses
    /// [`DEFAULT_TOLERANCE_PER_MODULE`]·`m_x`.
    pub tolerance: Option<f64>,
    /// Optional concentric bore diameter.
    pub bore_diameter: Option<f64>,
}

impl WormParams {
    /// A right-hand worm with α_n = 20°, `h_a` 1.0, `h_f` 1.25 and a root
    /// fillet of 0.38.
    #[must_use]
    pub fn standard(module: f64, starts: u32, diameter_factor: f64, length: f64) -> Self {
        Self {
            module,
            starts,
            diameter_factor,
            pressure_angle: 20.0_f64.to_radians(),
            length,
            addendum: 1.0,
            dedendum: 1.25,
            root_fillet: 0.38,
            backlash: 0.0,
            hand: Hand::Right,
            tolerance: None,
            bore_diameter: None,
        }
    }

    /// Lead angle `γ = atan(z₁/q)`.
    fn lead_angle(&self) -> f64 {
        f64::from(self.starts).atan2(self.diameter_factor)
    }
}

/// A worm's closed-form dimensions.
#[derive(Debug, Clone, PartialEq)]
pub struct WormGeometry {
    /// Axial module `m_x`.
    pub axial_module: f64,
    /// Normal module `m_x·cos γ`.
    pub normal_module: f64,
    /// Transverse module `m_x/tan γ` (of the equivalent helical gear).
    pub transverse_module: f64,
    /// Lead angle γ at the reference cylinder, radians.
    pub lead_angle: f64,
    /// Axial pitch `π·m_x`, between neighbouring threads.
    pub axial_pitch: f64,
    /// Lead `z₁·π·m_x`: one thread's advance over a full turn.
    pub lead: f64,
    /// Reference diameter `q·m_x`.
    pub pitch_diameter: f64,
    /// Tip diameter `d₁ + 2·h_a·m_x`.
    pub tip_diameter: f64,
    /// Root diameter `d₁ − 2·h_f·m_x`.
    pub root_diameter: f64,
    /// Base cylinder diameter of the involute helicoids.
    pub base_diameter: f64,
    /// Axial thread thickness at the reference cylinder, `p_x/2 − j_x`.
    pub axial_thickness: f64,
    /// Axial thread thickness at the tip cylinder; non-positive means
    /// pointed.
    pub axial_tip_thickness: f64,
    /// Guideline violations: a thin tip (axial, under 0.25·`m_x`) or a thin
    /// rim between the root and the bore (under 2·`m_x`).
    pub warnings: Vec<GearWarning>,
}

/// A built worm.
#[derive(Debug, Clone, PartialEq)]
pub struct WormOutcome {
    /// The worm body.
    pub solid: SolidId,
    /// Its dimensions and warnings.
    pub geometry: WormGeometry,
    /// Proven bound on any flank's distance from the exact involute
    /// helicoid, profile and twist fits together, with a floating-point
    /// allowance.
    pub flank_deviation_bound: f64,
    /// Hermite spans per flank across the profile.
    pub spans_per_flank: usize,
    /// Hermite spans along the axis.
    pub spans_along_axis: usize,
    /// The role of every face. Thread start `k` takes the tooth roles of
    /// index `k` (`tooth[k].flank.plus`, `tooth[k].tip`, `root[k]`, …).
    pub face_roles: Vec<(FaceId, GearFace)>,
}

/// The helical wheel that meshes with a worm.
#[derive(Debug, Clone, PartialEq)]
pub struct WormWheel {
    /// Gear parameters for [`make_gear`](super::make_gear).
    pub params: GearParams,
    /// Distance between the worm and wheel axes, `(d₁ + d₂)/2`.
    pub center_distance: f64,
}

fn validate(params: &WormParams) -> Result<(), GearError> {
    positive(params.module, "module")?;
    require((1..=MAX_WORM_STARTS).contains(&params.starts), || {
        format!(
            "starts must be in [1, {MAX_WORM_STARTS}], got {}",
            params.starts
        )
    })?;
    positive(params.diameter_factor, "diameter factor")?;
    require(
        params.pressure_angle.is_finite()
            && params.pressure_angle > 0.0
            && params.pressure_angle <= FRAC_PI_4,
        || {
            format!(
                "pressure angle must be in (0°, 45°], got {}°",
                params.pressure_angle.to_degrees()
            )
        },
    )?;
    positive(params.length, "length")?;
    positive(params.addendum, "addendum coefficient")?;
    positive(params.dedendum, "dedendum coefficient")?;
    non_negative(params.root_fillet, "root fillet coefficient")?;
    non_negative(params.backlash, "backlash")?;
    if let Some(tolerance) = params.tolerance {
        positive(tolerance, "tolerance")?;
    }
    if let Some(bore) = params.bore_diameter {
        positive(bore, "bore diameter")?;
    }
    let lead_angle = params.lead_angle();
    require(lead_angle <= MAX_LEAD_ANGLE_DEG.to_radians(), || {
        format!(
            "the lead angle {:.2}° exceeds {MAX_LEAD_ANGLE_DEG}°; use a larger diameter \
             factor or fewer starts",
            lead_angle.to_degrees()
        )
    })?;
    require(params.diameter_factor > 2.0 * params.dedendum, || {
        "the root cylinder has no positive diameter; use a larger diameter factor or a \
         smaller dedendum"
            .to_owned()
    })?;
    require(params.backlash < 0.5 * PI * params.module, || {
        "backlash must be less than half the axial pitch".to_owned()
    })?;
    Ok(())
}

/// The involute helical gear the worm is (see the [module docs](self)):
/// `z₁` teeth, normal module `m_x·cos γ`, helix angle `90° − γ`. Every
/// coefficient is rescaled so its length stays a multiple of `m_x`, and the
/// axial backlash becomes the transverse thinning `j_x/tan γ`.
pub(super) fn equivalent(params: &WormParams) -> GearParams {
    let lead_angle = params.lead_angle();
    let cos = lead_angle.cos();
    GearParams {
        kind: GearKind::External,
        module: params.module * cos,
        teeth: params.starts,
        pressure_angle: params.pressure_angle,
        face_width: params.length,
        profile_shift: 0.0,
        backlash: params.backlash / lead_angle.tan(),
        addendum: params.addendum / cos,
        dedendum: params.dedendum / cos,
        root_fillet: params.root_fillet / cos,
        tolerance: Some(
            params
                .tolerance
                .unwrap_or(DEFAULT_TOLERANCE_PER_MODULE * params.module),
        ),
        bore_diameter: params.bore_diameter,
        rim_diameter: None,
        back_depth: None,
        helix_angle: FRAC_PI_2 - lead_angle,
        hand: params.hand,
        herringbone: false,
    }
}

/// Closed-form dimensions and guideline warnings.
///
/// # Errors
///
/// [`GearError::Refused`] with
/// [`GearRefusal::InvalidParameter`](super::GearRefusal::InvalidParameter)
/// for out-of-range parameters.
pub fn worm_geometry(params: &WormParams) -> Result<WormGeometry, GearError> {
    validate(params)?;
    Ok(geometry_of(params, &helical_geometry(&equivalent(params))))
}

/// The worm's dimensions read from its equivalent helical gear's.
fn geometry_of(params: &WormParams, helical: &GearGeometry) -> WormGeometry {
    let m = params.module;
    let z = f64::from(params.starts);
    let lead_angle = params.lead_angle();
    let pitch_diameter = params.diameter_factor * m;
    let tip_diameter = helical.tip_diameter.unwrap_or(0.0);
    // The thread turns `2/(z₁·m_x)` radians per unit of advance, so a
    // transverse arc of `θ` radians at radius `r` (thickness `r·θ`) spans
    // `θ·z₁·m_x/2` along the axis.
    let axial_tip_thickness = helical.tip_thickness / tip_diameter * z * m;
    let root_diameter = helical.root_diameter.unwrap_or(0.0);
    let mut warnings = Vec::new();
    if axial_tip_thickness > 0.0 && axial_tip_thickness < THIN_TIP_MODULES * m {
        warnings.push(GearWarning::ThinTip {
            thickness: axial_tip_thickness,
        });
    }
    if let Some(bore) = params.bore_diameter {
        let thickness = 0.5 * (root_diameter - bore);
        if thickness > 0.0 && thickness < THIN_RIM_MODULES * m {
            warnings.push(GearWarning::ThinRim { thickness });
        }
    }
    WormGeometry {
        axial_module: m,
        normal_module: m * lead_angle.cos(),
        transverse_module: helical.transverse_module,
        lead_angle,
        axial_pitch: PI * m,
        lead: z * PI * m,
        pitch_diameter,
        tip_diameter,
        root_diameter,
        base_diameter: helical.base_diameter.unwrap_or(0.0),
        axial_thickness: (0.5 * PI).mul_add(m, -params.backlash),
        axial_tip_thickness,
        warnings,
    }
}

/// Build a worm.
///
/// The first cap is centered on the placement origin, the worm runs along
/// the placement axis, and thread start 0 is centered on the x direction
/// turned `phase` about the axis at the first cap. A right-hand thread turns
/// counter-clockwise about the axis as it advances along it.
///
/// The whole construction runs in one transaction: a refusal or failure
/// leaves the topology as it was.
///
/// # Errors
///
/// [`GearError::Refused`] for out-of-range parameters or an unbuildable
/// thread form (a bore reaching the root, a tolerance needing too many
/// spans); [`GearError::Operations`] when the extrusion fails.
pub fn make_worm(
    topo: &mut Topology,
    params: &WormParams,
    placement: &GearPlacement,
) -> Result<WormOutcome, GearError> {
    let geometry = worm_geometry(params)?;
    let frame = Frame::new(placement, GearKind::External)?;
    remus_topology::transaction::run_transacted(topo, |topo| {
        build(topo, params, &frame, placement.phase, geometry)
    })
}

fn build(
    topo: &mut Topology,
    params: &WormParams,
    frame: &Frame,
    phase: f64,
    geometry: WormGeometry,
) -> Result<WormOutcome, GearError> {
    let gear = equivalent(params);
    let helical = helical_geometry(&gear);
    let local = local_outline(&gear, &helical, phase)?;
    let (profile_bound, spans_per_flank) = flank_bound(&local.outline, &helical);
    let twist = twist(&gear, &helical).map(|twist| super::Twist {
        max_spans: MAX_WORM_SPANS,
        ..twist
    });
    let (solid, face_roles, twisted) =
        gear_body(topo, local, frame, frame.x, params.length, twist.into())?;
    Ok(WormOutcome {
        solid,
        geometry,
        flank_deviation_bound: profile_bound + twisted.bound,
        spans_per_flank,
        spans_along_axis: twisted.spans,
        face_roles,
    })
}

/// The involute helical wheel with `teeth` teeth and `face_width` that
/// meshes with `worm`, a crossed helical partner (see [`make_worm`]).
///
/// It takes the worm's hand, helix angle `γ`, normal module `m_x·cos γ` and
/// pressure angle, and addendum, dedendum and root fillet of the same
/// lengths as the worm's.
///
/// The wheel's axis crosses the worm's at 90° and `center_distance` from
/// it; its transverse module is `m_x`, so its pitch diameter is `z₂·m_x`.
/// The returned parameters have no backlash, tolerance or bore; set them
/// before [`make_gear`](super::make_gear).
///
/// # Errors
///
/// [`GearError::Refused`] with
/// [`GearRefusal::InvalidParameter`](super::GearRefusal::InvalidParameter)
/// when the worm or the wheel's tooth count or face width is out of range.
pub fn worm_wheel(worm: &WormParams, teeth: u32, face_width: f64) -> Result<WormWheel, GearError> {
    validate(worm)?;
    require((3..=MAX_TEETH).contains(&teeth), || {
        format!("a worm wheel needs 3 to {MAX_TEETH} teeth, got {teeth}")
    })?;
    let lead_angle = worm.lead_angle();
    let cos = lead_angle.cos();
    let params = GearParams {
        module: worm.module * cos,
        pressure_angle: worm.pressure_angle,
        addendum: worm.addendum / cos,
        dedendum: worm.dedendum / cos,
        root_fillet: worm.root_fillet / cos,
        helix_angle: lead_angle,
        hand: worm.hand,
        ..GearParams::standard(GearKind::External, worm.module * cos, teeth, face_width)
    };
    // Rejects a face width the gear would refuse.
    super::gear_geometry(&params)?;
    Ok(WormWheel {
        params,
        center_distance: 0.5 * worm.module * (worm.diameter_factor + f64::from(teeth)),
    })
}

/// Options for [`make_throated_worm_wheel`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ThroatedWheelOptions {
    /// Outside diameter `d_e2`, where the tips stand outside the throat;
    /// `None` is the throat diameter plus one axial module.
    pub outside_diameter: Option<f64>,
    /// Circumferential tooth thinning at the pitch circle (length).
    pub backlash: f64,
    /// Flank and throat curve tolerance; `None` uses
    /// [`DEFAULT_TOLERANCE_PER_MODULE`]·`m_x·cos γ`.
    pub tolerance: Option<f64>,
    /// Optional concentric bore diameter.
    pub bore_diameter: Option<f64>,
}

/// A built throated worm wheel.
#[derive(Debug, Clone, PartialEq)]
pub struct ThroatedWheelOutcome {
    /// The wheel body and its tooth form. `geometry.tip_diameter` is the
    /// outside diameter; `face_roles` names the throat patches
    /// `tooth[k].throat` and the tip strips either side of it
    /// `tooth[k].tip.start-half` and `tooth[k].tip.end-half`.
    pub gear: GearOutcome,
    /// Proven bound on the distance from each curve where a flank meets the
    /// throat to the exact intersection of the involute helicoid with the
    /// torus, with a floating-point allowance.
    pub throat_curve_bound: f64,
    /// Distance between the worm and wheel axes, `(d₁ + d₂)/2`.
    pub center_distance: f64,
    /// Throat diameter `d_a2 = d₂ + 2·h_a·m_x`, the tips' smallest diameter,
    /// half way across the face.
    pub throat_diameter: f64,
    /// Outside diameter `d_e2`.
    pub outside_diameter: f64,
    /// Radius of the throat torus's circle about the worm axis,
    /// `a − d_a2/2`.
    pub throat_radius: f64,
    /// Width of the throat along the wheel axis, where the tips run below
    /// the outside diameter.
    pub throat_width: f64,
}

/// Build the helical wheel that meshes with `worm` (see [`worm_wheel`]) on a
/// throated blank.
///
/// Across the throat its tips follow the torus that keeps the worm's root
/// clearance, centered on the worm axis half way across the face width;
/// either side of it they stay on the outside cylinder.
///
/// The worm's axis is perpendicular to the wheel's, `center_distance` from
/// it and `face_width/2` along it from the first cap. The construction is
/// the helical wheel with tips out to the outside diameter, each tooth tip
/// rebuilt as two cylinder strips and an exact torus patch, and each flank
/// stopped at the throat along a certified fit of its intersection with the
/// torus. No boolean is involved.
///
/// The whole construction runs in one transaction: a refusal or failure
/// leaves the topology as it was.
///
/// # Errors
///
/// [`GearError::Refused`] with
/// [`GearRefusal::InvalidParameter`](super::GearRefusal::InvalidParameter)
/// when the worm, tooth count or face width is out of range, the outside
/// diameter is not above the throat diameter, or the throat spans the whole
/// face width (a narrower face needs a smaller outside diameter); the
/// gear's own refusals for the tooth form (a pointed tip at the outside
/// diameter, a bore reaching the root); [`GearError::Operations`] when the
/// construction fails.
pub fn make_throated_worm_wheel(
    topo: &mut Topology,
    worm: &WormParams,
    teeth: u32,
    face_width: f64,
    options: &ThroatedWheelOptions,
    placement: &GearPlacement,
) -> Result<ThroatedWheelOutcome, GearError> {
    let matched = worm_wheel(worm, teeth, face_width)?;
    non_negative(options.backlash, "backlash")?;
    let throat_diameter = gear_geometry(&matched.params)?.tip_diameter.unwrap_or(0.0);
    let throat_radius = matched.center_distance - 0.5 * throat_diameter;
    require(throat_radius > 0.0, || {
        "the worm is too thin to wrap the wheel's throat around it".to_owned()
    })?;
    let outside_diameter = options
        .outside_diameter
        .unwrap_or(throat_diameter + worm.module);
    positive(outside_diameter, "outside diameter")?;
    let outside_radius = 0.5 * outside_diameter;
    require(outside_diameter > throat_diameter * (1.0 + 1e-9), || {
        format!(
            "the outside diameter {outside_diameter} must be above the throat diameter \
             {throat_diameter}"
        )
    })?;
    // Where the torus meets the outside cylinder: |s − c| = w.
    let reach = matched.center_distance - outside_radius;
    let half_width = throat_radius.mul_add(throat_radius, -reach * reach).sqrt();
    require(2.0 * half_width < face_width * (1.0 - 1e-9), || {
        format!(
            "the throat ({:.4} wide) spans the whole face width {face_width}; use a wider face \
             or a smaller outside diameter",
            2.0 * half_width
        )
    })?;
    // The matching wheel with its tips raised to the outside diameter.
    let pitch_radius = 0.5 * worm.module * f64::from(teeth);
    let params = GearParams {
        addendum: (outside_radius - pitch_radius) / matched.params.module,
        backlash: options.backlash,
        tolerance: options.tolerance,
        bore_diameter: options.bore_diameter,
        ..matched.params
    };
    let geometry = gear_geometry(&params)?;
    let frame = Frame::new(placement, GearKind::External)?;
    let throat = Throat {
        center_distance: matched.center_distance,
        tube_radius: throat_radius,
        mid_height: 0.5 * face_width,
        half_width,
        outside_radius,
        face_width,
    };
    remus_topology::transaction::run_transacted(topo, |topo| {
        let gear = super::build(topo, &params, &frame, placement.phase, &geometry)?;
        let rate = twist(&params, &geometry).map_or(0.0, |twist| twist.rate);
        let throated = cut_throat(
            topo,
            gear.solid,
            &gear.face_roles,
            &frame,
            &throat,
            circular(&params).base_radius,
            rate,
            0.5 * flank_tolerance(&params),
            gear.flank_deviation_bound,
        )?;
        Ok(ThroatedWheelOutcome {
            gear: GearOutcome {
                solid: throated.solid,
                face_roles: throated.face_roles,
                ..gear
            },
            throat_curve_bound: throated.curve_bound,
            center_distance: matched.center_distance,
            throat_diameter,
            outside_diameter,
            throat_radius,
            throat_width: 2.0 * half_width,
        })
    })
}
