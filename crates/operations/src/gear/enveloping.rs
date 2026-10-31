//! Double-enveloping (Hindley) worms.
//!
//! A double-enveloping worm wraps round the wheel it drives: its blank is an
//! hourglass, and its thread is cut by the wheel's own tooth form. In the
//! Hindley form that tooth is straight-sided in the wheel's mid-plane (the
//! plane through the worm axis perpendicular to the wheel axis): each flank
//! line is tangent to the base circle of radius `r_b = r₂·sin α` about the
//! wheel center, so it meets the pitch circle `r₂` at the pressure angle `α`.
//! As the wheel turns by `φ`, the worm turns by `i·φ` (`i = z₂/z₁`), and the
//! worm's thread is the surface the flank lines sweep.
//!
//! In the worm's frame each position of a flank line is a straight ruling,
//! so every flank is a ruled surface. The circles about the wheel center at
//! the worm's tip and root distances (`ρ_t = r₂ − h_a·m`,
//! `ρ_f = r₂ + h_f·m`) turn into themselves as the wheel turns, so each
//! ruling ends on the two tori the worm's axis sweeps those circles round:
//! the tip hourglass and the root hourglass. On a torus with angle `u` about
//! the worm axis and `v` round the tube, a ruling's end sits at
//!
//! ```text
//! u = π/2 − σ·i·φ (+ 2π·k/z₁ for thread start k),   v = θ(ρ) + φ,
//! ```
//!
//! a straight line in `(u, v)` (`σ` is +1 for a right-hand worm, −1 for a
//! left-hand one; `θ(ρ)` is the flank line's polar angle at radius `ρ`).
//! So the thread lands and root floors are exact torus patches bounded by
//! lines in their own parameters, cut every half turn so no patch crosses
//! the torus seam, and the flanks are the ruled surfaces between their two
//! rails.
//!
//! The rails are not rational: each is a certified cubic Hermite fit whose
//! fourth derivative is bounded in closed form, and the flank is the ruled
//! B-spline between the two rail fits, so it is within the rails' bound of
//! the true flank everywhere. The worm ends on planes perpendicular to its
//! axis at `±L/2` from the throat. Each meets the tori in exact circles and
//! each flank along a planar curve whose radius is
//! `a + (r_b − z·sin w)/cos w` (`w` the flank line's normal angle), also a
//! certified fit.
//!
//! Every axial section of the worm is the hourglass blank less the wheel's
//! straight-sided teeth: the classical Hindley property, and the tests'
//! oracle.
//!
//! Conventions, in the mid-plane at the throat:
//!
//! - module `m` (the wheel's transverse module, the worm's axial module at
//!   the throat), worm starts `z₁`, wheel teeth `z₂`;
//! - throat pitch diameter `d₁ = q·m`, wheel pitch diameter `d₂ = z₂·m`,
//!   center distance `a = (d₁ + d₂)/2`;
//! - throat lead angle `tan γ = z₁/q`;
//! - `backlash` `j` thins the thread along the wheel's pitch circle, so the
//!   thread is `π·m/2 − j` thick there;
//! - the flanks meet the root floor at a sharp corner (no fillet).
//!
//! The wheel that meshes with this worm is the wheel the worm envelops; it
//! is not built here.

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

use remus_math::compact_sort::CompactSort;
use remus_math::curves::Circle3D;
use remus_math::nurbs::NurbsSurface;
use remus_math::nurbs::hermite_fit::{HermiteFit, fit_cubic_hermite};
use remus_math::surfaces::{CylindricalSurface, ToroidalSurface};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use super::jet::{Jet, compose, product};
use super::worm::MAX_WORM_STARTS;
use super::{
    DEFAULT_TOLERANCE_PER_MODULE, GearEnd, GearError, GearFace, GearFaceRole, GearPlacement,
    GearRefusal, GearWarning, Hand, MAX_TEETH, THIN_RIM_MODULES, THIN_TIP_MODULES, ToothSide,
    non_negative, positive, require,
};

/// Uniform Hermite spans allowed along one fitted curve before the
/// tolerance refuses.
const MAX_ENVELOPING_SPANS: usize = 4096;

/// Fewest wheel teeth an enveloping worm is built for.
pub const MIN_ENVELOPED_TEETH: u32 = 10;

/// A double-enveloping (Hindley) worm's parameters. Lengths are model units;
/// coefficients are multiples of the module.
#[derive(Debug, Clone, PartialEq)]
pub struct EnvelopingWormParams {
    /// Module `m`: the wheel's transverse module, and the worm's axial
    /// module at the throat.
    pub module: f64,
    /// Number of thread starts `z₁`.
    pub starts: u32,
    /// Teeth `z₂` of the wheel the worm envelops.
    pub wheel_teeth: u32,
    /// Diameter factor `q = d₁/m` at the throat.
    pub diameter_factor: f64,
    /// Pressure angle `α` of the wheel's straight flanks at its pitch
    /// circle, radians, in `(0, π/4]`.
    pub pressure_angle: f64,
    /// Length along the worm axis, centered on the throat.
    pub length: f64,
    /// Addendum coefficient `h_a` (1.0).
    pub addendum: f64,
    /// Dedendum coefficient `h_f` (1.25).
    pub dedendum: f64,
    /// Thinning of the thread along the wheel's pitch circle (length).
    pub backlash: f64,
    /// Hand of the thread.
    pub hand: Hand,
    /// Thread surface deviation tolerance; `None` uses
    /// [`DEFAULT_TOLERANCE_PER_MODULE`]·`m`.
    pub tolerance: Option<f64>,
    /// Optional concentric bore diameter.
    pub bore_diameter: Option<f64>,
}

impl EnvelopingWormParams {
    /// A right-hand worm with α = 20°, `h_a` 1.0 and `h_f` 1.25.
    #[must_use]
    pub fn standard(
        module: f64,
        starts: u32,
        wheel_teeth: u32,
        diameter_factor: f64,
        length: f64,
    ) -> Self {
        Self {
            module,
            starts,
            wheel_teeth,
            diameter_factor,
            pressure_angle: 20.0_f64.to_radians(),
            length,
            addendum: 1.0,
            dedendum: 1.25,
            backlash: 0.0,
            hand: Hand::Right,
            tolerance: None,
            bore_diameter: None,
        }
    }
}

/// An enveloping worm's closed-form dimensions.
#[derive(Debug, Clone, PartialEq)]
pub struct EnvelopingWormGeometry {
    /// Center distance `a = (d₁ + d₂)/2` to the wheel's axis.
    pub center_distance: f64,
    /// Transmission ratio `i = z₂/z₁`.
    pub ratio: f64,
    /// Throat lead angle γ, `tan γ = z₁/q`, radians.
    pub lead_angle: f64,
    /// Throat pitch diameter `d₁ = q·m`.
    pub throat_pitch_diameter: f64,
    /// Throat tip diameter `d₁ + 2·h_a·m`, the narrowest point of the tip
    /// hourglass.
    pub throat_tip_diameter: f64,
    /// Throat root diameter `d₁ − 2·h_f·m`.
    pub throat_root_diameter: f64,
    /// Tip diameter at the end caps.
    pub end_tip_diameter: f64,
    /// Root diameter at the end caps.
    pub end_root_diameter: f64,
    /// The wheel's pitch diameter `z₂·m`.
    pub wheel_pitch_diameter: f64,
    /// Diameter of the base circle the wheel's flank lines are tangent to,
    /// `d₂·sin α`.
    pub base_diameter: f64,
    /// Wheel angle the worm's tip spans, `2·asin(L/(2·ρ_t))`, radians.
    pub wrap_angle: f64,
    /// Thread thickness along the wheel's pitch circle, `π·m/2 − j`.
    pub thread_thickness: f64,
    /// Thread thickness along the tip circle in the mid-plane; non-positive
    /// means pointed.
    pub tip_thickness: f64,
    /// Width of the root floor along the root circle in the mid-plane;
    /// non-positive means the wheel's tooth closes before the root.
    pub root_width: f64,
    /// Guideline violations: a thin tip (under 0.25·`m`) or a thin rim
    /// between the throat root and the bore (under 2·`m`).
    pub warnings: Vec<GearWarning>,
}

/// A built enveloping worm.
#[derive(Debug, Clone, PartialEq)]
pub struct EnvelopingWormOutcome {
    /// The worm body.
    pub solid: SolidId,
    /// Its dimensions and warnings.
    pub geometry: EnvelopingWormGeometry,
    /// Proven bound on any thread surface's distance from the exact Hindley
    /// flank (the rail fits; the end curves are within the same bound of
    /// the flanks' sections), with a floating-point allowance.
    pub flank_deviation_bound: f64,
    /// Hermite spans along each flank.
    pub spans_along_flank: usize,
    /// Center of the wheel the worm envelops: on the placement's x direction
    /// at the center distance from the throat.
    pub wheel_center: Point3,
    /// Axis of that wheel, perpendicular to the worm's.
    pub wheel_axis: Vec3,
    /// The role of every face. Thread `k`'s flanks are
    /// `tooth[k].flank.plus` and `.minus` (the plus flank at the larger
    /// angle about the worm axis), its tip land the torus patches
    /// `tooth[k].tip[p]`, and the root floor between threads `k` and `k + 1`
    /// the patches `root[k][p]`, with `p` counted from the start cap.
    pub face_roles: Vec<(FaceId, GearFace)>,
}

/// Derived construction quantities, in the worm's local frame: worm axis
/// `z`, throat at the origin, wheel center at `(0, a, 0)`, wheel axis `x`.
#[derive(Debug, Clone)]
pub(super) struct Hindley {
    pub(super) a: f64,
    /// Base circle radius `r_b`.
    pub(super) rb: f64,
    /// Ratio `i`.
    pub(super) ratio: f64,
    /// +1 right hand, −1 left hand.
    pub(super) sigma: f64,
    pub(super) starts: u32,
    pub(super) wheel_teeth: u32,
    /// Half length `h`.
    pub(super) half: f64,
    pub(super) rho_tip: f64,
    pub(super) rho_root: f64,
    /// Polar angles about the wheel center of the tangent points of tooth
    /// 0's plus and minus flank lines.
    pub(super) tangent_plus: f64,
    pub(super) tangent_minus: f64,
    pub(super) phase: f64,
}

impl Hindley {
    pub(super) fn new(params: &EnvelopingWormParams, phase: f64) -> Self {
        let m = params.module;
        let r2 = 0.5 * m * f64::from(params.wheel_teeth);
        let r1 = 0.5 * m * params.diameter_factor;
        let alpha = params.pressure_angle;
        // Half the wheel tooth's angle at its pitch circle: the thread is
        // thinned by the backlash, so the tooth that cuts it is thickened.
        let delta = (0.5 * PI).mul_add(m, params.backlash) / (2.0 * r2);
        Self {
            a: r1 + r2,
            rb: r2 * alpha.sin(),
            ratio: f64::from(params.wheel_teeth) / f64::from(params.starts),
            sigma: match params.hand {
                Hand::Right => 1.0,
                Hand::Left => -1.0,
            },
            starts: params.starts,
            wheel_teeth: params.wheel_teeth,
            half: 0.5 * params.length,
            rho_tip: params.addendum.mul_add(-m, r2),
            rho_root: params.dedendum.mul_add(m, r2),
            tangent_plus: PI + delta + FRAC_PI_2 - alpha,
            tangent_minus: PI - delta - FRAC_PI_2 + alpha,
            phase,
        }
    }

    fn tangent(&self, side: ToothSide) -> f64 {
        match side {
            ToothSide::Plus => self.tangent_plus,
            ToothSide::Minus => self.tangent_minus,
        }
    }

    /// Polar angle about the wheel center of tooth 0's flank line at
    /// radius `rho ≥ r_b`.
    pub(super) fn line_angle(&self, side: ToothSide, rho: f64) -> f64 {
        let off = rho
            .mul_add(rho, -self.rb * self.rb)
            .max(0.0)
            .sqrt()
            .atan2(self.rb);
        match side {
            ToothSide::Plus => self.tangent_plus - off,
            ToothSide::Minus => self.tangent_minus + off,
        }
    }

    /// Angle about the worm axis of start `start`'s rulings at wheel angle
    /// `phi`.
    fn u(&self, start: u32, phi: f64) -> f64 {
        self.sigma.mul_add(
            -self.ratio * phi,
            TAU.mul_add(
                f64::from(start) / f64::from(self.starts),
                FRAC_PI_2 + self.phase,
            ),
        )
    }

    /// Torus angle `v` at which the tube of radius `rho` meets the plane at
    /// height `z` on the hourglass side.
    fn latitude(rho: f64, z: f64) -> f64 {
        PI - (z / rho).asin()
    }

    /// Wheel angle at which a flank's rail at radius `rho` crosses height
    /// `z`.
    fn rail_end(&self, side: ToothSide, rho: f64, z: f64) -> f64 {
        Self::latitude(rho, z) - self.line_angle(side, rho)
    }

    /// Point and derivative of a rail: the flank's ruling end at radius
    /// `rho`, start `start`, wheel angle `phi`.
    fn rail(&self, start: u32, side: ToothSide, rho: f64, phi: f64) -> (Point3, Vec3) {
        let u = self.u(start, phi);
        let v = self.line_angle(side, rho) + phi;
        let (su, cu) = u.sin_cos();
        let (sv, cv) = v.sin_cos();
        let r = rho.mul_add(cv, self.a);
        let du = -self.sigma * self.ratio;
        (
            Point3::new(r * cu, r * su, rho * sv),
            Vec3::new(
                (-rho * sv).mul_add(cu, -r * su * du),
                (-rho * sv).mul_add(su, r * cu * du),
                rho * cv,
            ),
        )
    }

    /// Point and derivative of a flank's section by the plane at height
    /// `z`, at wheel angle `phi`.
    fn end_curve(&self, start: u32, side: ToothSide, z: f64, phi: f64) -> (Point3, Vec3) {
        let u = self.u(start, phi);
        let w = self.tangent(side) + phi;
        let (sw, cw) = w.sin_cos();
        let r = self.a + z.mul_add(-sw, self.rb) / cw;
        let dr = self.rb.mul_add(sw, -z) / (cw * cw);
        let (su, cu) = u.sin_cos();
        let du = -self.sigma * self.ratio;
        (
            Point3::new(r * cu, r * su, z),
            Vec3::new(
                dr.mul_add(cu, -r * su * du),
                dr.mul_add(su, r * cu * du),
                0.0,
            ),
        )
    }

    /// Distance from the wheel center (in the rotated mid-plane) of a flank
    /// section point at height `z`, wheel angle `phi`.
    fn end_radius(&self, side: ToothSide, z: f64, phi: f64) -> f64 {
        let w = self.tangent(side) + phi;
        let (sw, cw) = w.sin_cos();
        let y = z.mul_add(-sw, self.rb) / cw;
        y.hypot(z)
    }

    /// A signed margin of a local point inside the worm's material: positive
    /// inside the hourglass blank and clear of every wheel tooth that cut the
    /// groove, negative outside; roughly the distance to the nearest of
    /// those boundaries.
    pub(super) fn material_margin(&self, p: Point3) -> f64 {
        let u = p.y().atan2(p.x());
        let r = p.x().hypot(p.y());
        let (y, z) = (r - self.a, p.z());
        let rho = y.hypot(z);
        let blank = (self.half - z.abs()).min(rho - self.rho_tip);
        let phi = (FRAC_PI_2 + self.phase - u) / (self.sigma * self.ratio);
        let pitch = TAU / f64::from(self.wheel_teeth);
        let angle = z.atan2(y) - phi;
        let radius = rho.max(self.rb);
        let lo = self.line_angle(ToothSide::Minus, radius);
        let width = self.line_angle(ToothSide::Plus, radius) - lo;
        let offset = (angle - lo).rem_euclid(pitch);
        let angular = if offset <= width {
            -offset.min(width - offset)
        } else {
            (offset - width).min(pitch - offset)
        };
        blank.min((angular * rho).max(rho - self.rho_root))
    }

    /// Whether a local point lies inside one of the wheel teeth that cut
    /// the worm: the worm's groove.
    fn in_groove(&self, p: Point3) -> bool {
        let u = p.y().atan2(p.x());
        let r = p.x().hypot(p.y());
        let (y, z) = (r - self.a, p.z());
        let rho = y.hypot(z);
        if rho > self.rho_root || rho < self.rb {
            return false;
        }
        // The wheel angle at which start 0's ruling plane passes `u`; every
        // other wheel tooth is this one moved by whole tooth pitches.
        let phi = (FRAC_PI_2 + self.phase - u) / (self.sigma * self.ratio);
        let pitch = TAU / f64::from(self.wheel_teeth);
        let angle = z.atan2(y) - phi;
        let lo = self.line_angle(ToothSide::Minus, rho);
        let hi = self.line_angle(ToothSide::Plus, rho);
        let offset = (angle - lo).rem_euclid(pitch);
        offset <= hi - lo
    }
}

fn validate(params: &EnvelopingWormParams) -> Result<(), GearError> {
    positive(params.module, "module")?;
    require((1..=MAX_WORM_STARTS).contains(&params.starts), || {
        format!(
            "starts must be in [1, {MAX_WORM_STARTS}], got {}",
            params.starts
        )
    })?;
    require(
        (MIN_ENVELOPED_TEETH..=MAX_TEETH).contains(&params.wheel_teeth),
        || {
            format!(
                "an enveloping worm's wheel needs {MIN_ENVELOPED_TEETH} to {MAX_TEETH} teeth, got {}",
                params.wheel_teeth
            )
        },
    )?;
    require(params.wheel_teeth >= 2 * params.starts, || {
        "the wheel needs at least two teeth per worm start".to_owned()
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
    non_negative(params.backlash, "backlash")?;
    if let Some(tolerance) = params.tolerance {
        positive(tolerance, "tolerance")?;
    }
    if let Some(bore) = params.bore_diameter {
        positive(bore, "bore diameter")?;
    }
    require(params.diameter_factor > 2.0 * params.dedendum, || {
        "the root hourglass has no positive throat diameter; use a larger diameter factor or a \
         smaller dedendum"
            .to_owned()
    })?;
    require(params.backlash < 0.5 * PI * params.module, || {
        "backlash must be less than half the circular pitch".to_owned()
    })?;
    let h = Hindley::new(params, 0.0);
    require(h.rho_tip > h.rb, || {
        "the worm's tip reaches inside the wheel's base circle; use a smaller pressure angle or \
         addendum"
            .to_owned()
    })?;
    require(h.half < h.rho_tip, || {
        format!(
            "the worm is longer than its tip hourglass allows: length {} must be under {}",
            params.length,
            2.0 * h.rho_tip
        )
    })?;
    Ok(())
}

/// Closed-form dimensions and guideline warnings.
///
/// # Errors
///
/// [`GearError::Refused`] with [`GearRefusal::InvalidParameter`] for
/// out-of-range parameters.
pub fn enveloping_worm_geometry(
    params: &EnvelopingWormParams,
) -> Result<EnvelopingWormGeometry, GearError> {
    validate(params)?;
    let m = params.module;
    let h = Hindley::new(params, 0.0);
    let d1 = params.diameter_factor * m;
    let pitch = TAU / f64::from(params.wheel_teeth);
    let tip_angle = h.line_angle(ToothSide::Minus, h.rho_tip) + pitch
        - h.line_angle(ToothSide::Plus, h.rho_tip);
    let root_angle =
        h.line_angle(ToothSide::Plus, h.rho_root) - h.line_angle(ToothSide::Minus, h.rho_root);
    let tip_thickness = tip_angle * h.rho_tip;
    let end = |rho: f64| 2.0 * (h.a - rho.mul_add(rho, -h.half * h.half).sqrt());
    let throat_root_diameter = params.dedendum.mul_add(-2.0 * m, d1);
    let mut warnings = Vec::new();
    if tip_thickness > 0.0 && tip_thickness < THIN_TIP_MODULES * m {
        warnings.push(GearWarning::ThinTip {
            thickness: tip_thickness,
        });
    }
    if let Some(bore) = params.bore_diameter {
        let thickness = 0.5 * (throat_root_diameter - bore);
        if thickness > 0.0 && thickness < THIN_RIM_MODULES * m {
            warnings.push(GearWarning::ThinRim { thickness });
        }
    }
    Ok(EnvelopingWormGeometry {
        center_distance: h.a,
        ratio: h.ratio,
        lead_angle: f64::from(params.starts).atan2(params.diameter_factor),
        throat_pitch_diameter: d1,
        throat_tip_diameter: params.addendum.mul_add(2.0 * m, d1),
        throat_root_diameter,
        end_tip_diameter: end(h.rho_tip),
        end_root_diameter: end(h.rho_root),
        wheel_pitch_diameter: m * f64::from(params.wheel_teeth),
        base_diameter: 2.0 * h.rb,
        wrap_angle: 2.0 * (h.half / h.rho_tip).asin(),
        thread_thickness: (0.5 * PI).mul_add(m, -params.backlash),
        tip_thickness,
        root_width: root_angle * h.rho_root,
        warnings,
    })
}

/// Build a double-enveloping (Hindley) worm.
///
/// The first cap is centered on the placement origin and the worm runs
/// `length` along the placement axis, so the throat is at its middle. The
/// wheel it envelops sits on the placement's x direction at the center
/// distance from the throat, its axis along `axis × x` reversed (reported
/// as [`EnvelopingWormOutcome::wheel_center`] and `wheel_axis`); `phase`
/// turns the thread about the worm axis. A right-hand thread turns
/// counter-clockwise about the axis as it advances along it.
///
/// The whole construction runs in one transaction: a refusal or failure
/// leaves the topology as it was.
///
/// # Errors
///
/// [`GearError::Refused`] for out-of-range parameters or an unbuildable
/// thread: a pointed thread tip ([`GearRefusal::PointedTip`]), a wheel
/// tooth that closes before the worm's root
/// ([`GearRefusal::RootLandClosed`]), a bore reaching the throat root
/// ([`GearRefusal::BoreTooLarge`]), a flank the end planes cannot cut
/// cleanly ([`GearRefusal::InvalidParameter`]) or a tolerance needing too
/// many spans ([`GearRefusal::ToleranceTooTight`]).
pub fn make_enveloping_worm(
    topo: &mut Topology,
    params: &EnvelopingWormParams,
    placement: &GearPlacement,
) -> Result<EnvelopingWormOutcome, GearError> {
    let geometry = enveloping_worm_geometry(params)?;
    if geometry.tip_thickness <= 0.0 {
        return Err(GearError::refused(
            GearRefusal::PointedTip,
            format!(
                "the worm's thread comes to a point before its tip (tip thickness {:.6})",
                geometry.tip_thickness
            ),
        ));
    }
    if geometry.root_width <= 0.0 {
        return Err(GearError::refused(
            GearRefusal::RootLandClosed,
            "the wheel's tooth comes to a point before the worm's root".to_owned(),
        ));
    }
    if let Some(bore) = params.bore_diameter
        && bore >= geometry.throat_root_diameter
    {
        return Err(GearError::refused(
            GearRefusal::BoreTooLarge,
            format!(
                "the bore {bore} reaches the throat root diameter {}",
                geometry.throat_root_diameter
            ),
        ));
    }
    let frame = super::Frame::new(placement, super::GearKind::External)?;
    let world = World::new(&frame, params.length);
    let hindley = Hindley::new(params, placement.phase);
    let tolerance = params
        .tolerance
        .unwrap_or(DEFAULT_TOLERANCE_PER_MODULE * params.module);
    remus_topology::transaction::run_transacted(topo, |topo| {
        build(topo, params, &hindley, &world, tolerance, geometry)
    })
}

/// The worm's local frame placed in the model: local `x` is the placement's
/// `−y`, local `y` (toward the wheel) its `x`, local `z` its axis, and the
/// local origin the throat.
struct World {
    origin: Point3,
    e1: Vec3,
    e2: Vec3,
    e3: Vec3,
}

impl World {
    fn new(frame: &super::Frame, length: f64) -> Self {
        Self {
            origin: frame.origin + frame.axis * (0.5 * length),
            e1: -frame.y,
            e2: frame.x,
            e3: frame.axis,
        }
    }

    fn point(&self, p: Point3) -> Point3 {
        self.origin + self.e1 * p.x() + self.e2 * p.y() + self.e3 * p.z()
    }

    fn vector(&self, v: Vec3) -> Vec3 {
        self.e1 * v.x() + self.e2 * v.y() + self.e3 * v.z()
    }
}

/// `[1, n, n², n³, n⁴]`: the derivatives of a rotation at rate `n`.
fn rotation_jet(n: f64) -> Jet {
    [1.0, n, n * n, n * n * n, n * n * n * n]
}

/// Bound on a rail's fourth derivative: `(a + ρ·cos v)·e^{iu}` turns at
/// rate `i` while its radius swings by `ρ` at rate one, and the height
/// `ρ·sin v` adds at most `ρ`.
fn rail_bound(h: &Hindley, rho: f64) -> f64 {
    let radius: Jet = [h.a + rho, rho, rho, rho, rho];
    product(radius, rotation_jet(h.ratio))[4] + rho
}

/// Bound on an end curve's fourth derivative over `w ∈ [w0, w1]` (the flank
/// line's normal angle), where `|cos w| ≥ c > 0`: its radius is
/// `a + r_b·sec w − z·tan w`, turning at rate `i`.
fn end_bound(h: &Hindley, z: f64, c: f64) -> f64 {
    // 1/x over |x| ≥ c, composed with cos (every derivative at most one).
    let reciprocal: Jet = [
        1.0 / c,
        1.0 / (c * c),
        2.0 / c.powi(3),
        6.0 / c.powi(4),
        24.0 / c.powi(5),
    ];
    let sec = compose(reciprocal, [1.0; 5]);
    let tan = product([1.0; 5], sec);
    let y: Jet = std::array::from_fn(|k| h.rb.mul_add(sec[k], z.abs() * tan[k]));
    let radius: Jet = [h.a + y[0], y[1], y[2], y[3], y[4]];
    product(radius, rotation_jet(h.ratio))[4]
}

fn fit_error(error: remus_math::MathError, tolerance: f64) -> GearError {
    match error {
        remus_math::MathError::ResourceLimitExceeded { .. } => GearError::refused(
            GearRefusal::ToleranceTooTight,
            format!(
                "the thread needs more than {MAX_ENVELOPING_SPANS} spans along a flank for a \
                 tolerance of {tolerance}"
            ),
        ),
        other => other.into(),
    }
}

fn unbuildable(reason: impl Into<String>) -> GearError {
    GearError::refused(GearRefusal::InvalidParameter, reason)
}

/// Which hourglass a rail or band lies on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Level {
    Tip,
    Root,
}

/// One flank: start `start`'s rulings of tooth 0's `side` flank line.
struct Flank {
    start: u32,
    side: ToothSide,
    /// The flank's patches in increasing wheel angle, each under half a turn
    /// of the worm, split along whole rulings.
    pieces: Vec<FlankPiece>,
    /// Ruling edges between consecutive patches, at each patch boundary.
    cuts: Vec<EdgeId>,
    /// Rail vertices: `(phi, vertex)` sorted by `phi`, per level.
    tip_vertices: Vec<(f64, VertexId)>,
    root_vertices: Vec<(f64, VertexId)>,
    /// Rail breakpoints before their vertices exist, per level.
    tip_breaks: Vec<f64>,
    root_breaks: Vec<f64>,
    /// Rail edges between consecutive vertices, per level.
    tip_edges: Vec<EdgeId>,
    root_edges: Vec<EdgeId>,
    /// End curve edges at the start (`−h`) and end (`+h`) caps.
    ends: [Option<EdgeId>; 2],
}

/// One patch of a flank: the ruled surface between its two rail fits over a
/// wheel-angle interval.
struct FlankPiece {
    domain: (f64, f64),
    tip: HermiteFit,
    root: HermiteFit,
}

impl FlankPiece {
    const fn rail(&self, level: Level) -> &HermiteFit {
        match level {
            Level::Tip => &self.tip,
            Level::Root => &self.root,
        }
    }
}

impl Flank {
    /// The patch holding wheel angle `phi`.
    fn piece_at(&self, phi: f64) -> &FlankPiece {
        self.pieces
            .iter()
            .find(|piece| phi <= piece.domain.1)
            .unwrap_or_else(|| &self.pieces[self.pieces.len() - 1])
    }

    fn vertices_mut(&mut self, level: Level) -> &mut Vec<(f64, VertexId)> {
        match level {
            Level::Tip => &mut self.tip_vertices,
            Level::Root => &mut self.root_vertices,
        }
    }

    fn breaks_mut(&mut self, level: Level) -> &mut Vec<f64> {
        match level {
            Level::Tip => &mut self.tip_breaks,
            Level::Root => &mut self.root_breaks,
        }
    }

    fn vertices(&self, level: Level) -> &[(f64, VertexId)] {
        match level {
            Level::Tip => &self.tip_vertices,
            Level::Root => &self.root_vertices,
        }
    }
}

/// What a band polygon's side runs along.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Carrier {
    /// The band's lower rail (smaller `v`).
    Lower,
    /// Its upper rail.
    Upper,
    /// The cap at height `+h` (`end` 1) or `−h` (`end` 0).
    Cap(usize),
    /// The meridian at `u = k·π`.
    Meridian(i64),
}

/// A land or root floor: the strip of one torus between two rails.
struct Band {
    level: Level,
    start: u32,
    /// Flank indices of the lower and upper rails.
    lower: usize,
    upper: usize,
    /// `v = offset + φ` along each rail, in the band's wheel angle `φ`.
    lower_offset: f64,
    upper_offset: f64,
    /// Rail wheel angle minus band wheel angle, for each rail.
    lower_shift: f64,
    upper_shift: f64,
    rho: f64,
}

/// A clipped band piece: polygon corners in the band's `(φ, v)` chart, each
/// with the carrier of the side that starts there.
struct Piece {
    corners: Vec<((f64, f64), Carrier)>,
}

/// Clip a convex polygon to `φ ≥ lo` (`keep_above`) or `φ ≤ lo`.
fn clip(
    corners: &[((f64, f64), Carrier)],
    at: f64,
    keep_above: bool,
    cut: Carrier,
) -> Vec<((f64, f64), Carrier)> {
    let inside = |p: (f64, f64)| if keep_above { p.0 >= at } else { p.0 <= at };
    let mut out = Vec::new();
    let n = corners.len();
    for i in 0..n {
        let (p, carrier) = corners[i];
        let (q, _) = corners[(i + 1) % n];
        let crossing = || {
            let t = (at - p.0) / (q.0 - p.0);
            (at, t.mul_add(q.1 - p.1, p.1))
        };
        match (inside(p), inside(q)) {
            (true, true) => out.push((p, carrier)),
            (true, false) => {
                out.push((p, carrier));
                out.push((crossing(), cut));
            }
            (false, true) => out.push((crossing(), carrier)),
            (false, false) => {}
        }
    }
    // Drop zero-length sides a cut through a corner leaves.
    let mut kept: Vec<((f64, f64), Carrier)> = Vec::with_capacity(out.len());
    for corner in out {
        if let Some(last) = kept.last()
            && (last.0.0 - corner.0.0).abs() < 1e-12
            && (last.0.1 - corner.0.1).abs() < 1e-12
        {
            continue;
        }
        kept.push(corner);
    }
    while kept.len() > 1 {
        let (first, last) = (kept[0].0, kept[kept.len() - 1].0);
        if (first.0 - last.0).abs() < 1e-12 && (first.1 - last.1).abs() < 1e-12 {
            kept.pop();
        } else {
            break;
        }
    }
    kept
}

#[allow(clippy::too_many_lines)]
fn build(
    topo: &mut Topology,
    params: &EnvelopingWormParams,
    h: &Hindley,
    world: &World,
    tolerance: f64,
    geometry: EnvelopingWormGeometry,
) -> Result<EnvelopingWormOutcome, GearError> {
    let z = [-h.half, h.half];
    let sides = [ToothSide::Plus, ToothSide::Minus];
    let pitch = TAU / f64::from(h.wheel_teeth);

    // --- Rails: every start's plus and minus flank, tip and root. ---
    let rail_fourth = rail_bound(h, h.rho_tip).max(rail_bound(h, h.rho_root));
    let mut flanks: Vec<Flank> = Vec::new();
    let mut deviation: f64 = 0.0;
    let mut spans = 0;
    for start in 0..h.starts {
        for side in sides {
            let ends = |rho: f64| (h.rail_end(side, rho, z[1]), h.rail_end(side, rho, z[0]));
            let (tip_lo, tip_hi) = ends(h.rho_tip);
            let (root_lo, root_hi) = ends(h.rho_root);
            let domain = (tip_lo.min(root_lo), tip_hi.max(root_hi));
            // Split along the rulings at u = k·π where the whole ruling lies
            // between the caps, the same meridians that cut the bands, so no
            // patch winds a turn: a projection onto a many-turn patch can
            // start on the wrong turn.
            let (inner_lo, inner_hi) = (tip_lo.max(root_lo), tip_hi.min(root_hi));
            let (u_a, u_b) = (h.u(start, inner_lo), h.u(start, inner_hi));
            let (k_lo, k_hi) = (
                (u_a.min(u_b) / PI).floor() as i64,
                (u_a.max(u_b) / PI).ceil() as i64,
            );
            let mut breaks: Vec<f64> = (k_lo..=k_hi)
                .map(|k| inner_lo + (k as f64 * PI - u_a) / (u_b - u_a) * (inner_hi - inner_lo))
                .filter(|&phi| phi > inner_lo + 1e-9 && phi < inner_hi - 1e-9)
                .collect();
            breaks.compact_sort_by(f64::total_cmp);
            let bounds: Vec<f64> = std::iter::once(domain.0)
                .chain(breaks)
                .chain(std::iter::once(domain.1))
                .collect();
            let mut pieces = Vec::with_capacity(bounds.len() - 1);
            for pair in bounds.windows(2) {
                let fit = |rho: f64| {
                    fit_cubic_hermite(
                        |phi| {
                            let (p, d) = h.rail(start, side, rho, phi);
                            (world.point(p), world.vector(d))
                        },
                        pair[0],
                        pair[1],
                        rail_fourth,
                        tolerance,
                        MAX_ENVELOPING_SPANS,
                    )
                    .map_err(|error| fit_error(error, tolerance))
                };
                let (tip, root) = (fit(h.rho_tip)?, fit(h.rho_root)?);
                if tip.curve.knots() != root.curve.knots() {
                    return Err(unbuildable(
                        "a flank's rails were fitted to different spans",
                    ));
                }
                deviation = deviation.max(tip.remainder_bound).max(root.remainder_bound);
                pieces.push(FlankPiece {
                    domain: (pair[0], pair[1]),
                    tip,
                    root,
                });
            }
            spans = pieces.iter().map(|piece| piece.tip.spans).sum();
            flanks.push(Flank {
                start,
                side,
                pieces,
                cuts: Vec::new(),
                tip_vertices: Vec::new(),
                root_vertices: Vec::new(),
                tip_breaks: Vec::new(),
                root_breaks: Vec::new(),
                tip_edges: Vec::new(),
                root_edges: Vec::new(),
                ends: [None, None],
            });
        }
    }
    let flank_index =
        |start: u32, side: ToothSide| 2 * start as usize + usize::from(side == ToothSide::Minus);
    let scale = h.a + h.rho_root;
    let deviation = (64.0 * f64::EPSILON).mul_add(scale, deviation);
    let vertex_tolerance = deviation;

    // --- Bands: each start's land (plus tip rail to the next tooth's minus
    // tip rail) and root floor (its minus to its plus root rail). ---
    let next = |start: u32| {
        let z1 = i64::from(h.starts);
        u32::try_from((i64::from(start) + h.sigma as i64).rem_euclid(z1)).unwrap_or(0)
    };
    let mut bands = Vec::new();
    for start in 0..h.starts {
        bands.push(Band {
            level: Level::Tip,
            start,
            lower: flank_index(start, ToothSide::Plus),
            upper: flank_index(next(start), ToothSide::Minus),
            lower_offset: h.line_angle(ToothSide::Plus, h.rho_tip),
            upper_offset: h.line_angle(ToothSide::Minus, h.rho_tip) + pitch,
            lower_shift: 0.0,
            upper_shift: pitch,
            rho: h.rho_tip,
        });
        bands.push(Band {
            level: Level::Root,
            start,
            lower: flank_index(start, ToothSide::Minus),
            upper: flank_index(start, ToothSide::Plus),
            lower_offset: h.line_angle(ToothSide::Minus, h.rho_root),
            upper_offset: h.line_angle(ToothSide::Plus, h.rho_root),
            lower_shift: 0.0,
            upper_shift: 0.0,
            rho: h.rho_root,
        });
    }

    // Clip each band into pieces between meridians `u = k·π`.
    let mut band_pieces: Vec<Vec<Piece>> = Vec::new();
    for band in &bands {
        let (v_end, v_start) = (
            Hindley::latitude(band.rho, z[1]),
            Hindley::latitude(band.rho, z[0]),
        );
        let (l, u) = (band.lower_offset, band.upper_offset);
        // Counter-clockwise in (φ, v): bottom (+h cap) rightward, lower rail
        // up, top (−h cap) leftward, upper rail down.
        let polygon = vec![
            ((v_end - u, v_end), Carrier::Cap(1)),
            ((v_end - l, v_end), Carrier::Lower),
            ((v_start - l, v_start), Carrier::Cap(0)),
            ((v_start - u, v_start), Carrier::Upper),
        ];
        let (phi_lo, phi_hi) = (v_end - u, v_start - l);
        let u_at = |phi: f64| h.u(band.start, phi);
        let (u_a, u_b) = (u_at(phi_lo), u_at(phi_hi));
        let (k_lo, k_hi) = (
            (u_a.min(u_b) / PI).floor() as i64,
            (u_a.max(u_b) / PI).ceil() as i64,
        );
        let mut cuts: Vec<(f64, i64)> = (k_lo..=k_hi)
            .map(|k| {
                let target = k as f64 * PI;
                // u(φ) is affine in φ.
                let phi = phi_lo + (target - u_a) / (u_b - u_a) * (phi_hi - phi_lo);
                (phi, k)
            })
            .filter(|&(phi, _)| phi > phi_lo + 1e-12 && phi < phi_hi - 1e-12)
            .collect();
        cuts.compact_sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut pieces = Vec::new();
        let mut rest = polygon;
        for &(phi, k) in &cuts {
            let piece = clip(&rest, phi, false, Carrier::Meridian(k));
            rest = clip(&rest, phi, true, Carrier::Meridian(k));
            if piece.len() >= 3 {
                pieces.push(Piece { corners: piece });
            }
        }
        if rest.len() >= 3 {
            pieces.push(Piece { corners: rest });
        }
        band_pieces.push(pieces);
    }

    // --- Vertices on the rails, from every piece corner that lies on one. ---
    let rail_of = |band: &Band, carrier: Carrier| match carrier {
        Carrier::Lower => Some((band.lower, band.lower_shift)),
        Carrier::Upper => Some((band.upper, band.upper_shift)),
        Carrier::Cap(_) | Carrier::Meridian(_) => None,
    };
    let rail_point = |flank: &Flank, level: Level, phi: f64| {
        let rho = match level {
            Level::Tip => h.rho_tip,
            Level::Root => h.rho_root,
        };
        world.point(h.rail(flank.start, flank.side, rho, phi).0)
    };
    let merge = 1e-9;
    for (band, pieces) in bands.iter().zip(&band_pieces) {
        for piece in pieces {
            let n = piece.corners.len();
            for i in 0..n {
                let ((phi, _), outgoing) = piece.corners[i];
                let incoming = piece.corners[(i + n - 1) % n].1;
                for carrier in [incoming, outgoing] {
                    if let Some((flank, shift)) = rail_of(band, carrier) {
                        let rail_phi = phi + shift;
                        let list = flanks[flank].breaks_mut(band.level);
                        if !list.iter().any(|&p| (p - rail_phi).abs() < merge) {
                            list.push(rail_phi);
                        }
                    }
                }
            }
        }
    }
    for flank in &mut flanks {
        for level in [Level::Tip, Level::Root] {
            let mut breaks = std::mem::take(flank.breaks_mut(level));
            breaks.compact_sort_by(f64::total_cmp);
            let list = breaks
                .into_iter()
                .map(|phi| {
                    let vertex = topo
                        .add_vertex(Vertex::new(rail_point(flank, level, phi), vertex_tolerance));
                    (phi, vertex)
                })
                .collect();
            *flank.vertices_mut(level) = list;
        }
    }
    let rail_vertex = |flank: &Flank, level: Level, phi: f64| -> Result<VertexId, GearError> {
        flank
            .vertices(level)
            .iter()
            .find(|&&(p, _)| (p - phi).abs() < merge)
            .map(|entry| entry.1)
            .ok_or_else(|| unbuildable("a band corner lies on no rail vertex"))
    };

    // --- Rail edges between consecutive vertices. ---
    for flank in &mut flanks {
        for level in [Level::Tip, Level::Root] {
            let vertices = flank.vertices(level).to_vec();
            if vertices.len() < 2 {
                return Err(unbuildable("a rail has fewer than two vertices"));
            }
            let mut edges = Vec::with_capacity(vertices.len() - 1);
            for pair in vertices.windows(2) {
                let piece = flank.piece_at(0.5 * (pair[0].0 + pair[1].0));
                let curve = &piece.rail(level).curve;
                // Breakpoints come from the bands' corners, which can sit a
                // rounding error past a fit's domain ends.
                let (d0, d1) = curve.domain();
                let mut edge = Edge::with_tolerance(
                    pair[0].1,
                    pair[1].1,
                    EdgeCurve::NurbsCurve(curve.clone()),
                    Some(deviation),
                );
                edge.set_trim(Some((pair[0].0.clamp(d0, d1), pair[1].0.clamp(d0, d1))));
                edges.push(topo.add_edge(edge));
            }
            match level {
                Level::Tip => flank.tip_edges = edges,
                Level::Root => flank.root_edges = edges,
            }
        }
    }

    // --- End curves: each flank's section by both cap planes. ---
    let mut end_deviation: f64 = 0.0;
    for flank in &mut flanks {
        for (end, &height) in z.iter().enumerate() {
            let tip_phi = h.rail_end(flank.side, h.rho_tip, height);
            let root_phi = h.rail_end(flank.side, h.rho_root, height);
            let (lo, hi) = (tip_phi.min(root_phi), tip_phi.max(root_phi));
            let tangent = h.tangent(flank.side);
            let (w0, w1) = (tangent + lo, tangent + hi);
            // |cos w| is unimodal between its zeros, so its least value on a
            // zero-free interval is at an end.
            let zero = ((w0 - FRAC_PI_2) / PI).ceil() * PI + FRAC_PI_2;
            if zero <= w1 {
                return Err(unbuildable(
                    "a flank line runs parallel to an end plane; shorten the worm",
                ));
            }
            let c = w0.cos().abs().min(w1.cos().abs());
            for step in 0..=32 {
                let phi = lo + (hi - lo) * f64::from(step) / 32.0;
                let rho = h.end_radius(flank.side, height, phi);
                if rho < h.rho_tip - 1e-9 * scale || rho > h.rho_root + 1e-9 * scale {
                    return Err(unbuildable(
                        "an end plane leaves a flank's section outside its tip and root; \
                         shorten the worm",
                    ));
                }
            }
            let fit = fit_cubic_hermite(
                |phi| {
                    let (p, d) = h.end_curve(flank.start, flank.side, height, phi);
                    (world.point(p), world.vector(d))
                },
                lo,
                hi,
                end_bound(h, height, c),
                tolerance,
                MAX_ENVELOPING_SPANS,
            )
            .map_err(|error| fit_error(error, tolerance))?;
            end_deviation = end_deviation.max(fit.remainder_bound);
            let (v_lo, v_hi) = if tip_phi <= root_phi {
                (
                    rail_vertex(flank, Level::Tip, tip_phi)?,
                    rail_vertex(flank, Level::Root, root_phi)?,
                )
            } else {
                (
                    rail_vertex(flank, Level::Root, root_phi)?,
                    rail_vertex(flank, Level::Tip, tip_phi)?,
                )
            };
            let domain = fit.curve.domain();
            let mut edge = Edge::with_tolerance(
                v_lo,
                v_hi,
                EdgeCurve::NurbsCurve(fit.curve),
                Some(deviation.max(fit.remainder_bound)),
            );
            edge.set_trim(Some(domain));
            flank.ends[end] = Some(topo.add_edge(edge));
        }
    }
    let deviation = deviation.max(end_deviation);

    // --- Band edges: meridians and cap arcs; then band faces. ---
    let e1 = world.e1;
    let e2 = world.e2;
    let e3 = world.e3;
    let torus_of =
        |rho: f64| ToroidalSurface::with_axis_and_ref_dir(world.origin, h.a, rho, e3, e1);
    let tip_torus = torus_of(h.rho_tip)?;
    let root_torus = torus_of(h.rho_root)?;
    let torus_point = |rho: f64, u: f64, v: f64| {
        let (su, cu) = u.sin_cos();
        let (sv, cv) = v.sin_cos();
        let r = rho.mul_add(cv, h.a);
        world.point(Point3::new(r * cu, r * su, rho * sv))
    };
    // Cap arcs, keyed by cap, for the cap loops.
    let mut cap_edges: [Vec<EdgeId>; 2] = [Vec::new(), Vec::new()];
    let mut face_roles: Vec<(FaceId, GearFace)> = Vec::new();
    let mut faces: Vec<FaceId> = Vec::new();
    for (band, pieces) in bands.iter().zip(&band_pieces) {
        // Vertices where a meridian meets a cap, shared by the two pieces on
        // either side of it.
        let mut cap_cut_vertices: Vec<((usize, i64), VertexId)> = Vec::new();
        let mut meridians: Vec<(i64, EdgeId)> = Vec::new();
        let torus = match band.level {
            Level::Tip => tip_torus.clone(),
            Level::Root => root_torus.clone(),
        };
        // Order pieces from the start cap (largest φ) for their roles.
        let mut order: Vec<usize> = (0..pieces.len()).collect();
        let centroid = |piece: &Piece| {
            piece.corners.iter().map(|c| c.0.0).sum::<f64>() / piece.corners.len() as f64
        };
        order.compact_sort_by(|&a, &b| centroid(&pieces[b]).total_cmp(&centroid(&pieces[a])));
        for (index, &which) in order.iter().enumerate() {
            let piece = &pieces[which];
            let n = piece.corners.len();
            // Each corner's vertex.
            let mut corner_vertices = Vec::with_capacity(n);
            for i in 0..n {
                let ((phi, v), outgoing) = piece.corners[i];
                let incoming = piece.corners[(i + n - 1) % n].1;
                let on_rail = [incoming, outgoing]
                    .into_iter()
                    .find_map(|carrier| rail_of(band, carrier));
                let vertex = if let Some((flank, shift)) = on_rail {
                    rail_vertex(&flanks[flank], band.level, phi + shift)?
                } else {
                    let (end, k) = match (incoming, outgoing) {
                        (Carrier::Cap(end), Carrier::Meridian(k))
                        | (Carrier::Meridian(k), Carrier::Cap(end)) => (end, k),
                        _ => return Err(unbuildable("a band corner is on no rail or cap cut")),
                    };
                    if let Some(entry) = cap_cut_vertices.iter().find(|e| e.0 == (end, k)) {
                        entry.1
                    } else {
                        let id = topo.add_vertex(Vertex::new(
                            torus_point(band.rho, h.u(band.start, phi), v),
                            vertex_tolerance,
                        ));
                        cap_cut_vertices.push(((end, k), id));
                        id
                    }
                };
                corner_vertices.push(vertex);
            }
            // Each side's edge, walked from corner i to corner i + 1.
            let mut sides = Vec::with_capacity(n);
            for i in 0..n {
                let ((phi0, v0), carrier) = piece.corners[i];
                let ((phi1, v1), _) = piece.corners[(i + 1) % n];
                let (a, b) = (corner_vertices[i], corner_vertices[(i + 1) % n]);
                let oriented = match carrier {
                    Carrier::Lower | Carrier::Upper => {
                        let (flank, shift) = rail_of(band, carrier)
                            .ok_or_else(|| unbuildable("a rail side has no rail"))?;
                        let flank = &flanks[flank];
                        let (p0, p1) = (phi0 + shift, phi1 + shift);
                        let (lo, hi) = (p0.min(p1), p0.max(p1));
                        let vertices = flank.vertices(band.level);
                        let position =
                            vertices
                                .iter()
                                .position(|&(p, _)| (p - lo).abs() < merge)
                                .ok_or_else(|| unbuildable("a rail side starts at no vertex"))?;
                        if vertices
                            .get(position + 1)
                            .is_none_or(|&(p, _)| (p - hi).abs() >= merge)
                        {
                            return Err(unbuildable("a rail side spans more than one edge"));
                        }
                        let edges = match band.level {
                            Level::Tip => &flank.tip_edges,
                            Level::Root => &flank.root_edges,
                        };
                        OrientedEdge::new(edges[position], p0 < p1)
                    }
                    Carrier::Meridian(k) => {
                        if let Some(entry) = meridians.iter().find(|e| e.0 == k) {
                            let edge = topo.edge(entry.1)?;
                            OrientedEdge::new(entry.1, edge.start() == a)
                        } else {
                            let angle = k as f64 * PI;
                            let radial = e1 * angle.cos() + e2 * angle.sin();
                            let center = world.origin + radial * h.a;
                            let circle = Circle3D::with_axes(
                                center,
                                radial.cross(e3),
                                band.rho,
                                radial,
                                e3,
                            )?;
                            let (start, end, lo, hi) = if v0 <= v1 {
                                (a, b, v0, v1)
                            } else {
                                (b, a, v1, v0)
                            };
                            let mut edge = Edge::with_tolerance(
                                start,
                                end,
                                EdgeCurve::Circle(circle),
                                Some(vertex_tolerance),
                            );
                            edge.set_trim(Some((lo, hi)));
                            let id = topo.add_edge(edge);
                            meridians.push((k, id));
                            OrientedEdge::new(id, start == a)
                        }
                    }
                    Carrier::Cap(end) => {
                        let (u0, u1) = (h.u(band.start, phi0), h.u(band.start, phi1));
                        let (start, finish, lo, hi) = if u0 <= u1 {
                            (a, b, u0, u1)
                        } else {
                            (b, a, u1, u0)
                        };
                        let shift = (lo / TAU).floor() * TAU;
                        let height = z[end];
                        let v_cap = Hindley::latitude(band.rho, height);
                        let radius = band.rho.mul_add(v_cap.cos(), h.a);
                        let circle =
                            Circle3D::with_axes(world.origin + e3 * height, e3, radius, e1, e2)?;
                        let mut edge = Edge::with_tolerance(
                            start,
                            finish,
                            EdgeCurve::Circle(circle),
                            Some(vertex_tolerance),
                        );
                        edge.set_trim(Some((lo - shift, hi - shift)));
                        let id = topo.add_edge(edge);
                        cap_edges[end].push(id);
                        OrientedEdge::new(id, start == a)
                    }
                };
                sides.push(oriented);
            }
            // The corners run counter-clockwise in (φ, v); u falls as φ rises
            // when σ·i > 0, so the wire is turned round to run
            // counter-clockwise in the torus's (u, v).
            if h.sigma > 0.0 {
                sides = sides
                    .into_iter()
                    .rev()
                    .map(|e| OrientedEdge::new(e.edge(), !e.is_forward()))
                    .collect();
            }
            let wire = topo.add_wire(Wire::new(sides, true)?);
            // Material lies on the worm axis's side of both hourglasses, so
            // the outward normal points into the tube: against the torus's.
            let face = topo.add_face(Face::new_reversed(
                wire,
                vec![],
                FaceSurface::Torus(torus.clone()),
            ));
            let piece = u32::try_from(index).unwrap_or(u32::MAX);
            let role = match band.level {
                Level::Tip => GearFaceRole::TipPiece {
                    tooth: band.start,
                    piece,
                },
                Level::Root => GearFaceRole::RootPiece {
                    gap: root_gap(h, band.start),
                    piece,
                },
            };
            faces.push(face);
            face_roles.push((face, role.into()));
        }
    }

    // --- Flank faces: one ruled patch per piece, the pieces joined along
    // straight rulings. ---
    for flank in &mut flanks {
        let mut cuts = Vec::with_capacity(flank.pieces.len().saturating_sub(1));
        for piece in &flank.pieces[..flank.pieces.len() - 1] {
            let phi = piece.domain.1;
            let tip = rail_vertex(flank, Level::Tip, phi)?;
            let root = rail_vertex(flank, Level::Root, phi)?;
            cuts.push(topo.add_edge(Edge::new(tip, root, EdgeCurve::Line)));
        }
        flank.cuts = cuts;
    }
    for flank in &flanks {
        let [Some(start_end), Some(end_end)] = flank.ends else {
            return Err(unbuildable("a flank has no end curve"));
        };
        let count = flank.pieces.len();
        let mut patches = Vec::with_capacity(count);
        for (index, piece) in flank.pieces.iter().enumerate() {
            let (lo, hi) = piece.domain;
            let within = |vertices: &[(f64, VertexId)], edges: &[EdgeId]| -> Vec<EdgeId> {
                vertices
                    .windows(2)
                    .zip(edges)
                    .filter(|(pair, _)| pair[0].0 >= lo - merge && pair[1].0 <= hi + merge)
                    .map(|(_, &edge)| edge)
                    .collect()
            };
            let tip_edges = within(&flank.tip_vertices, &flank.tip_edges);
            let root_edges = within(&flank.root_vertices, &flank.root_edges);
            let tip_hi = rail_vertex(
                flank,
                Level::Tip,
                if index + 1 == count {
                    h.rail_end(flank.side, h.rho_tip, z[0])
                } else {
                    hi
                },
            )?;
            let root_lo = rail_vertex(
                flank,
                Level::Root,
                if index == 0 {
                    h.rail_end(flank.side, h.rho_root, z[1])
                } else {
                    lo
                },
            )?;
            // Counter-clockwise in (φ, s): the tip rail (s = 0) toward −h,
            // the far side (the −h end curve, or a ruling) to the root, the
            // root rail back toward +h, the near side (the +h end curve, or a
            // ruling) to the tip.
            let mut sides: Vec<OrientedEdge> = tip_edges
                .iter()
                .map(|&edge| OrientedEdge::new(edge, true))
                .collect();
            let far = if index + 1 == count {
                start_end
            } else {
                flank.cuts[index]
            };
            sides.push(OrientedEdge::new(far, topo.edge(far)?.start() == tip_hi));
            sides.extend(
                root_edges
                    .iter()
                    .rev()
                    .map(|&edge| OrientedEdge::new(edge, false)),
            );
            let near = if index == 0 {
                end_end
            } else {
                flank.cuts[index - 1]
            };
            sides.push(OrientedEdge::new(near, topo.edge(near)?.start() == root_lo));

            let tip = piece.tip.curve.control_points();
            let root = piece.root.curve.control_points();
            let net: Vec<Vec<Point3>> = tip.iter().zip(root).map(|(&t, &r)| vec![t, r]).collect();
            let weights = vec![vec![1.0; 2]; net.len()];
            let surface = NurbsSurface::new(
                3,
                1,
                piece.tip.curve.knots().to_vec(),
                vec![0.0, 0.0, 1.0, 1.0],
                net,
                weights,
            )?;
            // The surface normal ∂φ × ∂s faces out of the material where a
            // step along it enters the wheel tooth that cut the groove.
            let mid = 0.5 * (lo + hi);
            let (p, d) = h.rail(flank.start, flank.side, h.rho_tip, mid);
            let (q, e) = h.rail(flank.start, flank.side, h.rho_root, mid);
            let point = Point3::new(
                0.5 * (p.x() + q.x()),
                0.5 * (p.y() + q.y()),
                0.5 * (p.z() + q.z()),
            );
            let normal = ((d + e) * 0.5)
                .cross(q - p)
                .normalize()
                .map_err(|_| unbuildable("a flank is degenerate"))?;
            let step = 1e-3 * (h.rho_root - h.rho_tip);
            let ahead = h.in_groove(point + normal * step);
            let behind = h.in_groove(point - normal * step);
            if ahead == behind {
                return Err(unbuildable("a flank's material side is ambiguous"));
            }
            let wire = topo.add_wire(Wire::new(sides, true)?);
            let surface = FaceSurface::Nurbs(surface);
            let face = topo.add_face(if ahead {
                Face::new(wire, vec![], surface)
            } else {
                Face::new_reversed(wire, vec![], surface)
            });
            patches.push(face);
        }
        // Patches are numbered from the start cap: the largest wheel angle.
        let GearFaceRole::Flank { tooth, side } = flank_role(h, flank.start, flank.side) else {
            return Err(unbuildable("a flank has no flank role"));
        };
        for (piece, &face) in patches.iter().rev().enumerate() {
            faces.push(face);
            let role = GearFaceRole::FlankPiece {
                tooth,
                side,
                piece: u32::try_from(piece).unwrap_or(u32::MAX),
            };
            face_roles.push((face, role.into()));
        }
    }

    // --- Caps and the bore. ---
    let bore = params.bore_diameter.map(|d| 0.5 * d);
    let mut bore_circles: [Option<EdgeId>; 2] = [None, None];
    if let Some(radius) = bore {
        let mut seam_vertices = Vec::with_capacity(2);
        for (end, &height) in z.iter().enumerate() {
            let center = world.origin + e3 * height;
            let vertex = topo.add_vertex(Vertex::new(center + e1 * radius, vertex_tolerance));
            let circle = Circle3D::with_axes(center, e3, radius, e1, e2)?;
            let mut edge = Edge::new(vertex, vertex, EdgeCurve::Circle(circle));
            edge.set_trim(Some((0.0, TAU)));
            bore_circles[end] = Some(topo.add_edge(edge));
            seam_vertices.push(vertex);
        }
        let seam = topo.add_edge(Edge::new(
            seam_vertices[0],
            seam_vertices[1],
            EdgeCurve::Line,
        ));
        let [Some(low), Some(high)] = bore_circles else {
            return Err(unbuildable("the bore has no circles"));
        };
        let wire = topo.add_wire(Wire::new(
            vec![
                OrientedEdge::new(low, true),
                OrientedEdge::new(seam, true),
                OrientedEdge::new(high, false),
                OrientedEdge::new(seam, false),
            ],
            true,
        )?);
        let cylinder = CylindricalSurface::with_ref_dir(world.origin + e3 * z[0], e3, radius, e1)?;
        // The bore's material lies outside the cylinder.
        let face = topo.add_face(Face::new_reversed(
            wire,
            vec![],
            FaceSurface::Cylinder(cylinder),
        ));
        faces.push(face);
        face_roles.push((face, GearFaceRole::Bore.into()));
    }
    for (end, &height) in z.iter().enumerate() {
        let mut edges = cap_edges[end].clone();
        edges.extend(flanks.iter().filter_map(|flank| flank.ends[end]));
        let outward = if end == 1 { e3 } else { -e3 };
        let loop_edges = cap_loop(topo, &edges, outward)?;
        let wire = topo.add_wire(Wire::new(loop_edges, true)?);
        let mut inner = Vec::new();
        if let Some(circle) = bore_circles[end] {
            // A hole runs clockwise about the outward normal.
            inner.push(topo.add_wire(Wire::new(vec![OrientedEdge::new(circle, end == 0)], true)?));
        }
        let center = world.origin + e3 * height;
        let face = topo.add_face(Face::new(
            wire,
            inner,
            FaceSurface::Plane {
                normal: outward,
                d: outward.dot(Vec3::new(center.x(), center.y(), center.z())),
            },
        ));
        faces.push(face);
        let cap = if end == 0 {
            GearEnd::Start
        } else {
            GearEnd::End
        };
        face_roles.push((face, GearFaceRole::Cap(cap).into()));
    }

    let shell = topo.add_shell(Shell::new(faces)?);
    let solid = topo.add_solid(Solid::new(shell, vec![]));
    let wheel_center = world.point(Point3::new(0.0, h.a, 0.0));
    Ok(EnvelopingWormOutcome {
        solid,
        geometry,
        flank_deviation_bound: deviation,
        spans_along_flank: spans,
        wheel_center,
        wheel_axis: world.vector(Vec3::new(1.0, 0.0, 0.0)),
        face_roles,
    })
}

/// The root floor of start `start`'s groove lies between threads `k` and
/// `k + 1`; this is `k`.
fn root_gap(h: &Hindley, start: u32) -> u32 {
    if h.sigma > 0.0 {
        (start + h.starts - 1) % h.starts
    } else {
        start
    }
}

/// Thread role of a flank. The land of thread `k` runs from start `k`'s
/// plus flank (of the wheel tooth) to start `k + σ`'s minus flank, and for a
/// right-hand worm the second lies at the larger angle about the axis.
fn flank_role(h: &Hindley, start: u32, side: ToothSide) -> GearFaceRole {
    let right = h.sigma > 0.0;
    match side {
        ToothSide::Plus => GearFaceRole::Flank {
            tooth: start,
            side: if right {
                ToothSide::Minus
            } else {
                ToothSide::Plus
            },
        },
        ToothSide::Minus => GearFaceRole::Flank {
            tooth: if right {
                (start + h.starts - 1) % h.starts
            } else {
                (start + 1) % h.starts
            },
            side: if right {
                ToothSide::Plus
            } else {
                ToothSide::Minus
            },
        },
    }
}

/// Chain a planar cap's edges into one loop running counter-clockwise about
/// its outward normal.
pub(super) fn cap_loop(
    topo: &Topology,
    edges: &[EdgeId],
    outward: Vec3,
) -> Result<Vec<OrientedEdge>, GearError> {
    let ends: Vec<(VertexId, VertexId)> = edges
        .iter()
        .map(|&edge| topo.edge(edge).map(|e| (e.start(), e.end())))
        .collect::<Result<_, _>>()?;
    let mut used = vec![false; edges.len()];
    let mut chain = Vec::with_capacity(edges.len());
    let first = 0;
    used[first] = true;
    chain.push(OrientedEdge::new(edges[first], true));
    let origin = ends[first].0;
    let mut at = ends[first].1;
    while at != origin {
        let next = (0..edges.len())
            .find(|&i| !used[i] && (ends[i].0 == at || ends[i].1 == at))
            .ok_or_else(|| unbuildable("an end cap's outline does not close"))?;
        used[next] = true;
        let forward = ends[next].0 == at;
        chain.push(OrientedEdge::new(edges[next], forward));
        at = if forward { ends[next].1 } else { ends[next].0 };
    }
    if used.iter().any(|&u| !u) {
        return Err(unbuildable("an end cap's outline is not one loop"));
    }
    // Signed area in a basis whose cross product is the outward normal, from
    // points along every edge.
    let normal = outward
        .normalize()
        .map_err(|_| unbuildable("an end cap has no normal"))?;
    let helper = if normal.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let ea = normal
        .cross(helper)
        .normalize()
        .map_err(|_| unbuildable("an end cap has no plane basis"))?;
    let eb = normal.cross(ea);
    let mut points = Vec::new();
    for oriented in &chain {
        let edge = topo.edge(oriented.edge())?;
        let start = topo.vertex(edge.start())?.point();
        let finish = topo.vertex(edge.end())?.point();
        let (t0, t1) = edge.curve().domain_with_endpoints(start, finish);
        let (t0, t1) = edge.trim().unwrap_or((t0, t1));
        for i in 0..8 {
            let s = f64::from(i) / 8.0;
            let s = if oriented.is_forward() { s } else { 1.0 - s };
            let t = s.mul_add(t1 - t0, t0);
            let p = edge.curve().evaluate_with_endpoints(t, start, finish);
            let v = Vec3::new(p.x(), p.y(), p.z());
            points.push((v.dot(ea), v.dot(eb)));
        }
    }
    let mut area = 0.0;
    for i in 0..points.len() {
        let (p, q) = (points[i], points[(i + 1) % points.len()]);
        area += p.0.mul_add(q.1, -q.0 * p.1);
    }
    if area < 0.0 {
        chain = chain
            .into_iter()
            .rev()
            .map(|e| OrientedEdge::new(e.edge(), !e.is_forward()))
            .collect();
    }
    Ok(chain)
}
