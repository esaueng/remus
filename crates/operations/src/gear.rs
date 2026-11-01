//! Involute spur gears, internal (ring) gears and racks as exact B-Rep
//! solids.
//!
//! [`make_gear`] builds one planar outline in the gear's transverse plane and
//! extrudes it across the face width, so a gear costs one extrusion and no
//! boolean. The faces are planar caps, `Cylinder` tip and root lands and root
//! fillets, and ruled-NURBS flanks (racks: planes and cylinders only).
//!
//! The involute is not rational, so each flank is a C¹ cubic B-spline from
//! [`fit_cubic_hermite`](remus_math::nurbs::hermite_fit::fit_cubic_hermite)
//! with a proven deviation bound, reported as
//! [`GearOutcome::flank_deviation_bound`]. Every tooth is the first tooth
//! with its control points rotated, so all teeth are the same curve.
//!
//! Conventions (ISO 21771 / ISO 53 basic rack):
//!
//! - `module` m, `teeth` z, `pressure_angle` α (radians);
//! - pitch radius `r = m·z/2`, base radius `r_b = r·cos α`;
//! - external: tip radius `r + m(h_a + x)`, root radius `r − m(h_f − x)`,
//!   pitch-circle tooth thickness `m(π/2 + 2x·tan α) − j`;
//! - internal: inner tip radius `r − m(h_a − x)`, outer root radius
//!   `r + m(h_f + x)`, pitch-circle space width `m(π/2 + 2x·tan α) + j`
//!   (the space is shaped like an external tooth shifted by `x`). Positive
//!   `x` moves an internal profile away from the axis, into the rim; this
//!   sign is this module's convention, so check it against the convention a
//!   drawing or standard uses for internal gears;
//! - `backlash` j is the circumferential thinning of this gear's teeth at the
//!   pitch circle (a widening of an internal gear's spaces). A meshing pair's
//!   play is the sum of both gears' values.
//!
//! The root is a circular fillet of radius `root_fillet·m` tangent to the
//! flank and the root circle, with a radial flank below the base circle. It
//! is not the trochoid a generating cutter leaves, so an undercut gear is
//! reported with [`GearWarning::Undercut`] rather than modelled.

use std::f64::consts::{FRAC_PI_4, PI, TAU};

use remus_math::compact_sort::CompactSort;
use remus_math::curves::Circle3D;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::make_circle_edge_with_ref;
use remus_topology::edge::EdgeId;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use crate::OperationsError;

mod bevel;
mod cylinder;
mod enveloping;
mod hindley_wheel;
mod jet;
mod profile;
mod throat;
mod worm;

pub use bevel::{
    BevelGearGeometry, BevelGearOutcome, BevelGearParams, MAX_FACE_ANGLE_DEG, bevel_gear_geometry,
    make_bevel_gear,
};
pub use cylinder::{CylinderGearOptions, GearFit, cut_gear_teeth};
pub use enveloping::{
    EnvelopingWormGeometry, EnvelopingWormOutcome, EnvelopingWormParams, MIN_ENVELOPED_TEETH,
    enveloping_worm_geometry, make_enveloping_worm,
};
pub use hindley_wheel::{
    EnvelopingWheelOptions, EnvelopingWheelOutcome, make_enveloping_worm_wheel,
};
pub use worm::{
    MAX_LEAD_ANGLE_DEG, MAX_WORM_STARTS, ThroatedWheelOptions, ThroatedWheelOutcome, WormGeometry,
    WormOutcome, WormParams, WormWheel, make_throated_worm_wheel, make_worm, worm_geometry,
    worm_wheel,
};

use profile::{CircularTooth, RackTooth, Segment};

/// Largest supported tooth count.
pub const MAX_TEETH: u32 = 1000;

/// Default flank tolerance as a fraction of the module.
pub const DEFAULT_TOLERANCE_PER_MODULE: f64 = 1e-5;

/// Largest supported helix angle, degrees.
pub const MAX_HELIX_ANGLE_DEG: f64 = 45.0;

/// Uniform Hermite spans allowed along the axis of a helical gear before
/// the tolerance refuses.
const MAX_TWIST_SPANS: usize = 1024;

/// Default rack back depth (root line to back face) in modules.
pub const DEFAULT_RACK_BACK_DEPTH_PER_MODULE: f64 = 2.0;

/// Tip thickness below this many modules is reported as thin.
const THIN_TIP_MODULES: f64 = 0.25;

/// Rim (root to bore, or root to rim) below this many modules is reported
/// as thin.
const THIN_RIM_MODULES: f64 = 2.0;

/// Which gear body to build.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GearKind {
    /// External spur gear.
    External,
    /// Internal (ring) spur gear: teeth on the inside of a rim.
    Internal,
    /// Straight spur rack.
    Rack,
}

/// Hand of a helical gear's teeth.
///
/// Right-handed teeth turn counter-clockwise about the placement axis as
/// they advance along it (a right-handed screw); left-handed teeth turn
/// clockwise. External gears mesh with the opposite hand, an internal gear
/// with the same hand.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Hand {
    /// Right-handed helix.
    #[default]
    Right,
    /// Left-handed helix.
    Left,
}

/// Gear parameters. Lengths are model units; coefficients are multiples of
/// the module.
///
/// A non-zero `helix_angle` makes a helical gear (ISO 21771): `module`,
/// `pressure_angle` and the coefficients are then normal-section values,
/// while `backlash` and the root fillet apply in the transverse section
/// (perpendicular to the axis).
#[derive(Debug, Clone, PartialEq)]
pub struct GearParams {
    /// Body kind.
    pub kind: GearKind,
    /// Module m (pitch diameter per tooth).
    pub module: f64,
    /// Tooth count (rack: teeth along its length).
    pub teeth: u32,
    /// Pressure angle α in radians, in `(0, π/4]`.
    pub pressure_angle: f64,
    /// Extrusion length along the gear axis.
    pub face_width: f64,
    /// Profile shift coefficient x (zero for a rack).
    pub profile_shift: f64,
    /// Circumferential tooth thinning at the pitch circle (length).
    pub backlash: f64,
    /// Addendum coefficient `h_a` (ISO 53: 1.0).
    pub addendum: f64,
    /// Dedendum coefficient `h_f` (ISO 53: 1.25).
    pub dedendum: f64,
    /// Root fillet radius coefficient (ISO 53: 0.38); zero for a sharp root.
    pub root_fillet: f64,
    /// Flank deviation tolerance; `None` uses
    /// [`DEFAULT_TOLERANCE_PER_MODULE`]·m.
    pub tolerance: Option<f64>,
    /// External gears: optional concentric bore diameter.
    pub bore_diameter: Option<f64>,
    /// Internal gears: outer rim diameter (required).
    pub rim_diameter: Option<f64>,
    /// Racks: root line to back face; `None` uses
    /// [`DEFAULT_RACK_BACK_DEPTH_PER_MODULE`]·m.
    pub back_depth: Option<f64>,
    /// Helix angle β at the pitch cylinder, radians in
    /// `[0, MAX_HELIX_ANGLE_DEG°]`; zero is a spur gear. Circular gears only.
    pub helix_angle: f64,
    /// Hand of the helix; ignored for a spur gear. A herringbone gear's hand
    /// is that of its start half.
    pub hand: Hand,
    /// Herringbone (double helical): the helix reverses hand at the middle
    /// of the face width, so the far cap's profile lines up with the start
    /// cap's. Needs a helix angle.
    pub herringbone: bool,
}

impl GearParams {
    /// ISO 53 defaults (α = 20°, `h_a` 1.0, `h_f` 1.25, root fillet 0.38) for
    /// `kind` with the given module, tooth count and face width.
    #[must_use]
    pub fn standard(kind: GearKind, module: f64, teeth: u32, face_width: f64) -> Self {
        Self {
            kind,
            module,
            teeth,
            pressure_angle: 20.0_f64.to_radians(),
            face_width,
            profile_shift: 0.0,
            backlash: 0.0,
            addendum: 1.0,
            dedendum: 1.25,
            root_fillet: 0.38,
            tolerance: None,
            bore_diameter: None,
            rim_diameter: None,
            back_depth: None,
            helix_angle: 0.0,
            hand: Hand::Right,
            herringbone: false,
        }
    }
}

/// Where the gear sits.
///
/// Circular gears: `origin` is the center of the first cap face, `axis` the
/// extrusion direction, and the tooth at `phase` (radians, counter-clockwise
/// about `axis`) from `x_direction` is the first tooth. Racks: `origin` is
/// the start of the pitch line, `x_direction` runs along the rack, the teeth
/// point along `axis × x_direction`, and `phase` must be zero.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GearPlacement {
    /// Frame origin.
    pub origin: Point3,
    /// Extrusion axis (normalized internally).
    pub axis: Vec3,
    /// Reference direction, projected perpendicular to `axis`.
    pub x_direction: Vec3,
    /// Rotation of the teeth about `axis`, radians.
    pub phase: f64,
}

impl Default for GearPlacement {
    fn default() -> Self {
        Self {
            origin: Point3::new(0.0, 0.0, 0.0),
            axis: Vec3::new(0.0, 0.0, 1.0),
            x_direction: Vec3::new(1.0, 0.0, 0.0),
            phase: 0.0,
        }
    }
}

/// A design guideline the gear violates without being unbuildable.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GearWarning {
    /// A generating cutter would undercut this external gear; the modelled
    /// root is a fillet, not the undercut trochoid. `min_profile_shift` is
    /// the smallest shift that avoids it.
    Undercut {
        /// Smallest profile shift coefficient without undercut.
        min_profile_shift: f64,
    },
    /// Tip thickness under 0.25 modules.
    ThinTip {
        /// Tip thickness (arc length at the tip circle, or width at a rack's
        /// tip line).
        thickness: f64,
    },
    /// Rim under 2 modules between the root circle and the bore or rim.
    ThinRim {
        /// Radial rim thickness.
        thickness: f64,
    },
}

impl GearWarning {
    /// Stable machine-readable code.
    #[must_use]
    pub const fn code(&self) -> &'static str {
        match self {
            Self::Undercut { .. } => "undercut",
            Self::ThinTip { .. } => "thin-tip",
            Self::ThinRim { .. } => "thin-rim",
        }
    }
}

/// Which side of a tooth a face lies on: `Plus` is the side at the larger
/// polar angle about the gear axis (counter-clockwise looking down the
/// axis), or the `+x` side of a rack tooth.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum ToothSide {
    /// Smaller polar angle (rack: `−x`).
    Minus,
    /// Larger polar angle (rack: `+x`).
    Plus,
}

/// Start or end of the extrusion (caps) or of a rack's length.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum GearEnd {
    /// The placement origin's end.
    Start,
    /// The far end.
    End,
}

/// What a face of a gear body is, from the construction itself (never from
/// matching coordinates).
///
/// Tooth `k` of a circular gear is centered at `phase + k·2π/z`; root land
/// `k` lies between teeth `k` and `k + 1` (mod z). An internal gear uses the
/// same indices for its own (inward-pointing) teeth. Rack tooth `k` is
/// centered at `(k + ½)·p` along the rack.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum GearFaceRole {
    /// Involute flank of a tooth (rack: straight flank).
    Flank {
        /// Tooth index.
        tooth: u32,
        /// Which flank.
        side: ToothSide,
    },
    /// Radial flank piece below the base circle.
    FlankBase {
        /// Tooth index.
        tooth: u32,
        /// Which flank.
        side: ToothSide,
    },
    /// Root fillet at the foot of a tooth flank.
    Fillet {
        /// Tooth index.
        tooth: u32,
        /// Which flank.
        side: ToothSide,
    },
    /// Tip land of a tooth.
    Tip {
        /// Tooth index.
        tooth: u32,
    },
    /// Torus patch a throated worm wheel's tooth tip takes under the worm.
    Throat {
        /// Tooth index.
        tooth: u32,
    },
    /// Root land between teeth `gap` and `gap + 1`.
    Root {
        /// Index of the tooth before the land.
        gap: u32,
    },
    /// One ruled patch of an enveloping worm's thread flank, counted from
    /// the start cap.
    FlankPiece {
        /// Thread index.
        tooth: u32,
        /// Which flank.
        side: ToothSide,
        /// Patch index from the start cap.
        piece: u32,
    },
    /// One torus patch of an enveloping worm's thread tip, counted from the
    /// start cap.
    TipPiece {
        /// Thread index.
        tooth: u32,
        /// Patch index from the start cap.
        piece: u32,
    },
    /// One torus patch of an enveloping worm's root floor between threads
    /// `gap` and `gap + 1`, counted from the start cap.
    RootPiece {
        /// Index of the thread before the floor.
        gap: u32,
        /// Patch index from the start cap.
        piece: u32,
    },
    /// Planar end cap.
    Cap(GearEnd),
    /// Bore of an external gear.
    Bore,
    /// Outer rim of an internal gear.
    Rim,
    /// Back face of a rack.
    RackBack,
    /// End face of a rack.
    RackEnd(GearEnd),
    /// Half root land at a rack's end.
    RackRootEnd(GearEnd),
}

/// A gear face's role and, on a herringbone gear, the half of the face width
/// it lies in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct GearFace {
    /// What the face is.
    pub role: GearFaceRole,
    /// Herringbone gears: the half (from the start or the end cap) of a face
    /// swept from the profile; `None` on every other gear and on the caps.
    pub half: Option<GearEnd>,
}

impl GearFace {
    /// Stable name: the role's name, with `.start-half` or `.end-half` on a
    /// herringbone gear's halves, e.g. `tooth[3].flank.plus.end-half`.
    #[must_use]
    pub fn name(self) -> String {
        match self.half {
            None => self.role.name(),
            Some(GearEnd::Start) => format!("{}.start-half", self.role.name()),
            Some(GearEnd::End) => format!("{}.end-half", self.role.name()),
        }
    }
}

impl From<GearFaceRole> for GearFace {
    fn from(role: GearFaceRole) -> Self {
        Self { role, half: None }
    }
}

impl GearFaceRole {
    /// Stable name, e.g. `tooth[3].flank.plus`, `root[0]`, `cap.start`.
    #[must_use]
    pub fn name(self) -> String {
        let side = |side: ToothSide| match side {
            ToothSide::Minus => "minus",
            ToothSide::Plus => "plus",
        };
        let end = |end: GearEnd| match end {
            GearEnd::Start => "start",
            GearEnd::End => "end",
        };
        match self {
            Self::Flank { tooth, side: s } => format!("tooth[{tooth}].flank.{}", side(s)),
            Self::FlankBase { tooth, side: s } => {
                format!("tooth[{tooth}].flank-base.{}", side(s))
            }
            Self::Fillet { tooth, side: s } => format!("tooth[{tooth}].fillet.{}", side(s)),
            Self::Tip { tooth } => format!("tooth[{tooth}].tip"),
            Self::Throat { tooth } => format!("tooth[{tooth}].throat"),
            Self::Root { gap } => format!("root[{gap}]"),
            Self::FlankPiece {
                tooth,
                side: s,
                piece,
            } => format!("tooth[{tooth}].flank.{}[{piece}]", side(s)),
            Self::TipPiece { tooth, piece } => format!("tooth[{tooth}].tip[{piece}]"),
            Self::RootPiece { gap, piece } => format!("root[{gap}][{piece}]"),
            Self::Cap(e) => format!("cap.{}", end(e)),
            Self::Bore => "bore".to_owned(),
            Self::Rim => "rim".to_owned(),
            Self::RackBack => "back".to_owned(),
            Self::RackEnd(e) => format!("end.{}", end(e)),
            Self::RackRootEnd(e) => format!("root.{}", end(e)),
        }
    }

    /// The same role `k` periods on, with tooth indices taken mod `teeth`.
    fn shifted(self, k: u32, teeth: u32) -> Self {
        let shift = |index: u32| (index + k) % teeth;
        match self {
            Self::Flank { tooth, side } => Self::Flank {
                tooth: shift(tooth),
                side,
            },
            Self::FlankBase { tooth, side } => Self::FlankBase {
                tooth: shift(tooth),
                side,
            },
            Self::Fillet { tooth, side } => Self::Fillet {
                tooth: shift(tooth),
                side,
            },
            Self::Tip { tooth } => Self::Tip {
                tooth: shift(tooth),
            },
            Self::Throat { tooth } => Self::Throat {
                tooth: shift(tooth),
            },
            Self::Root { gap } => Self::Root { gap: shift(gap) },
            Self::FlankPiece { tooth, side, piece } => Self::FlankPiece {
                tooth: shift(tooth),
                side,
                piece,
            },
            Self::TipPiece { tooth, piece } => Self::TipPiece {
                tooth: shift(tooth),
                piece,
            },
            Self::RootPiece { gap, piece } => Self::RootPiece {
                gap: shift(gap),
                piece,
            },
            other => other,
        }
    }
}

/// Closed-form gear dimensions, available before any topology is built.
///
/// Diameters are `None` for a rack. For an internal gear `tip_diameter` is
/// the inner (tooth tip) circle and `root_diameter` the outer root circle.
#[derive(Debug, Clone, PartialEq)]
pub struct GearGeometry {
    /// Module the dimensions use (solved from the diameter when fitting teeth
    /// to an existing cylinder); the normal module of a helical gear.
    pub module: f64,
    /// Transverse module `m/cos β` (equal to `module` for a spur gear or a
    /// rack). Diameters are `transverse_module·z` based.
    pub transverse_module: f64,
    /// Helix angle β at the pitch cylinder, radians (zero for a spur gear).
    pub helix_angle: f64,
    /// Axial advance of one tooth over a full turn, `π·d/tan β`; `None` for
    /// a spur gear or a rack.
    pub lead: Option<f64>,
    /// Pitch circle diameter `m_t·z`.
    pub pitch_diameter: Option<f64>,
    /// Base circle diameter.
    pub base_diameter: Option<f64>,
    /// Tip circle diameter.
    pub tip_diameter: Option<f64>,
    /// Root circle diameter.
    pub root_diameter: Option<f64>,
    /// Circular pitch `π·m_t` along the pitch circle.
    pub circular_pitch: f64,
    /// The gear's own tooth thickness at the pitch circle (transverse arc)
    /// or line.
    pub tooth_thickness: f64,
    /// Tooth thickness at the tip circle (transverse arc) or tip line;
    /// non-positive means pointed.
    pub tip_thickness: f64,
    /// External gears: tooth count below which a zero-shift gear undercuts
    /// (`2·h_a·cos β/sin² α_t` for a helical gear).
    pub min_teeth_without_undercut: Option<f64>,
    /// External gears: smallest profile shift without undercut.
    pub min_profile_shift_without_undercut: Option<f64>,
    /// Guideline violations.
    pub warnings: Vec<GearWarning>,
}

/// A built gear.
#[derive(Debug, Clone, PartialEq)]
pub struct GearOutcome {
    /// The extruded gear body.
    pub solid: SolidId,
    /// Its dimensions and warnings.
    pub geometry: GearGeometry,
    /// Proven bound on any flank's distance from the true involute (a
    /// helical gear: the involute helicoid, profile and twist fits
    /// together), including a floating-point allowance; zero for a rack,
    /// whose flanks are exact planes.
    pub flank_deviation_bound: f64,
    /// Hermite spans per flank across the profile (zero for a rack).
    pub spans_per_flank: usize,
    /// Hermite spans along the axis of a helical gear (zero for a spur gear
    /// or a rack).
    pub spans_along_axis: usize,
    /// The role of every gear face, from the construction record. Faces of
    /// a body the gear was cut into carry no role.
    pub face_roles: Vec<(FaceId, GearFace)>,
}

/// Why a gear was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GearRefusal {
    /// A parameter is out of range.
    InvalidParameter,
    /// The tip circle lies on or inside the base circle.
    TipBelowBaseCircle,
    /// The flanks meet before the tip.
    PointedTip,
    /// Adjacent teeth (or a tooth space) close up at the root.
    RootLandClosed,
    /// The root fillet leaves no flank below the tip.
    FlankTooShort,
    /// The bore reaches the root circle.
    BoreTooLarge,
    /// The rim does not enclose the root circle.
    RimTooSmall,
    /// The flank tolerance needs more spans than allowed.
    ToleranceTooTight,
    /// The picked face is not a cylinder.
    NotACylinder,
    /// The picked cylinder has material on neither or both sides.
    NotABossOrBore,
    /// The teeth would reach past an end of the picked face.
    SpanOutsideFace,
    /// Internal teeth would cover only part of the picked bore. No longer
    /// returned: internal teeth over part of a bore build.
    PartialSpan,
    /// The tip circle is larger than the picked boss.
    CylinderTooSmall,
    /// A hole, slot or other feature crosses the band the teeth rebuild.
    FeaturesInToothRegion,
}

impl GearRefusal {
    /// Stable machine-readable code.
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::InvalidParameter => "invalid-parameter",
            Self::TipBelowBaseCircle => "tip-below-base-circle",
            Self::PointedTip => "pointed-tip",
            Self::RootLandClosed => "root-land-closed",
            Self::FlankTooShort => "flank-too-short",
            Self::BoreTooLarge => "bore-too-large",
            Self::RimTooSmall => "rim-too-small",
            Self::ToleranceTooTight => "tolerance-too-tight",
            Self::NotACylinder => "not-a-cylinder",
            Self::NotABossOrBore => "not-a-boss-or-bore",
            Self::SpanOutsideFace => "span-outside-face",
            Self::PartialSpan => "partial-span-unsupported",
            Self::CylinderTooSmall => "cylinder-too-small",
            Self::FeaturesInToothRegion => "features-in-tooth-region",
        }
    }
}

/// Error from [`make_gear`] or [`gear_geometry`].
#[derive(Debug, thiserror::Error)]
pub enum GearError {
    /// The parameters describe no buildable gear. `geometry` carries the
    /// dimensions when they could be computed.
    #[error("gear:{}: {reason}", code.code())]
    Refused {
        /// Refusal kind.
        code: GearRefusal,
        /// Human-readable reason.
        reason: String,
        /// Dimensions of the refused gear, when the parameters were valid.
        geometry: Option<Box<GearGeometry>>,
    },
    /// The underlying construction failed.
    #[error(transparent)]
    Operations(#[from] OperationsError),
}

impl GearError {
    fn refused(code: GearRefusal, reason: impl Into<String>) -> Self {
        Self::Refused {
            code,
            reason: reason.into(),
            geometry: None,
        }
    }

    fn invalid(reason: impl Into<String>) -> Self {
        Self::refused(GearRefusal::InvalidParameter, reason)
    }

    fn with_geometry(self, value: &GearGeometry) -> Self {
        match self {
            Self::Refused {
                code,
                reason,
                geometry: None,
            } => Self::Refused {
                code,
                reason,
                geometry: Some(Box::new(value.clone())),
            },
            other => other,
        }
    }
}

impl From<remus_math::MathError> for GearError {
    fn from(error: remus_math::MathError) -> Self {
        Self::Operations(error.into())
    }
}

impl From<remus_topology::TopologyError> for GearError {
    fn from(error: remus_topology::TopologyError) -> Self {
        Self::Operations(error.into())
    }
}

fn require(condition: bool, reason: impl FnOnce() -> String) -> Result<(), GearError> {
    if condition {
        Ok(())
    } else {
        Err(GearError::invalid(reason()))
    }
}

fn positive(value: f64, name: &str) -> Result<(), GearError> {
    require(value.is_finite() && value > 0.0, || {
        format!("{name} must be positive and finite, got {value}")
    })
}

fn non_negative(value: f64, name: &str) -> Result<(), GearError> {
    require(value.is_finite() && value >= 0.0, || {
        format!("{name} must be non-negative and finite, got {value}")
    })
}

// A rack's shift must be exactly zero, not merely small.
#[allow(clippy::float_cmp)]
fn validate(params: &GearParams) -> Result<(), GearError> {
    let kind = params.kind;
    positive(params.module, "module")?;
    let min_teeth = match kind {
        GearKind::External => 3,
        GearKind::Internal => 6,
        GearKind::Rack => 1,
    };
    require((min_teeth..=MAX_TEETH).contains(&params.teeth), || {
        format!(
            "teeth must be in [{min_teeth}, {MAX_TEETH}] for this gear kind, got {}",
            params.teeth
        )
    })?;
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
    positive(params.face_width, "face width")?;
    require(params.profile_shift.is_finite(), || {
        "profile shift must be finite".to_owned()
    })?;
    require(
        kind != GearKind::Rack || params.profile_shift == 0.0,
        || "a rack has no profile shift".to_owned(),
    )?;
    non_negative(params.backlash, "backlash")?;
    positive(params.addendum, "addendum coefficient")?;
    positive(params.dedendum, "dedendum coefficient")?;
    non_negative(params.root_fillet, "root fillet coefficient")?;
    if let Some(tolerance) = params.tolerance {
        positive(tolerance, "tolerance")?;
    }
    match (kind, params.bore_diameter) {
        (GearKind::External, Some(bore)) => positive(bore, "bore diameter")?,
        (_, Some(_)) => return Err(GearError::invalid("only an external gear takes a bore")),
        _ => {}
    }
    match (kind, params.rim_diameter) {
        (GearKind::Internal, Some(rim)) => positive(rim, "rim diameter")?,
        (GearKind::Internal, None) => {
            return Err(GearError::invalid("an internal gear needs a rim diameter"));
        }
        (_, Some(_)) => return Err(GearError::invalid("only an internal gear takes a rim")),
        _ => {}
    }
    match (kind, params.back_depth) {
        (GearKind::Rack, Some(depth)) => positive(depth, "back depth")?,
        (_, Some(_)) => return Err(GearError::invalid("only a rack takes a back depth")),
        _ => {}
    }
    require(
        params.helix_angle.is_finite()
            && params.helix_angle >= 0.0
            && params.helix_angle <= MAX_HELIX_ANGLE_DEG.to_radians(),
        || {
            format!(
                "helix angle must be in [0°, {MAX_HELIX_ANGLE_DEG}°], got {}°",
                params.helix_angle.to_degrees()
            )
        },
    )?;
    require(kind != GearKind::Rack || params.helix_angle <= 0.0, || {
        "helical racks are not supported yet".to_owned()
    })?;
    require(!params.herringbone || params.helix_angle > 0.0, || {
        "a herringbone gear needs a helix angle".to_owned()
    })?;
    Ok(())
}

/// The flank tolerance in model units.
fn flank_tolerance(params: &GearParams) -> f64 {
    params
        .tolerance
        .unwrap_or(DEFAULT_TOLERANCE_PER_MODULE * params.module)
}

/// The transverse-section spur equivalent of a helical gear (ISO 21771):
/// module `m/cos β`, pressure angle `atan(tan α/cos β)`, and every
/// coefficient rescaled so its length in model units is unchanged (the root
/// fillet radius stays `ρ·m` in the transverse section). Half the flank
/// tolerance goes to the profile, the other half to the twist. A spur gear
/// is its own equivalent.
fn transverse(params: &GearParams) -> GearParams {
    if params.helix_angle <= 0.0 {
        return params.clone();
    }
    let cos = params.helix_angle.cos();
    GearParams {
        module: params.module / cos,
        pressure_angle: (params.pressure_angle.tan() / cos).atan(),
        profile_shift: params.profile_shift * cos,
        addendum: params.addendum * cos,
        dedendum: params.dedendum * cos,
        root_fillet: params.root_fillet * cos,
        tolerance: Some(0.5 * flank_tolerance(params)),
        helix_angle: 0.0,
        ..params.clone()
    }
}

/// A built gear body: the solid, every face's role, and its twist report.
type GearBody = (SolidId, Vec<(FaceId, GearFace)>, TwistReport);

/// A twist's certified deviation bound and spans along the axis.
#[derive(Debug, Clone, Copy, Default)]
struct TwistReport {
    bound: f64,
    spans: usize,
}

/// How a gear's outline is swept along its axis.
#[derive(Debug, Clone, Copy)]
enum Sweep {
    /// A straight extrusion (spur gears, racks).
    Straight,
    /// A twisted extrusion (helical and herringbone gears).
    Twist(Twist),
    /// A tapered extrusion toward the apex this far along the axis from the
    /// outline (bevel gears).
    Taper { apex_distance: f64 },
}

impl From<Option<Twist>> for Sweep {
    fn from(twist: Option<Twist>) -> Self {
        twist.map_or(Self::Straight, Self::Twist)
    }
}

/// How a helical gear's transverse profile turns along its axis.
#[derive(Debug, Clone, Copy)]
struct Twist {
    /// Radians per unit of advance, positive for a right hand (a herringbone
    /// gear: over its start half).
    rate: f64,
    /// The twist's share of the flank tolerance.
    tolerance: f64,
    /// The rate reverses at the middle of the face width.
    herringbone: bool,
    /// Uniform Hermite spans allowed along the axis before the tolerance
    /// refuses.
    max_spans: usize,
}

/// The twist of a helical gear (`None` for a spur gear or a rack): the
/// teeth follow helices of angle β on the pitch cylinder, `tan β / r` radians
/// per unit length.
fn twist(params: &GearParams, geometry: &GearGeometry) -> Option<Twist> {
    if params.helix_angle <= 0.0 {
        return None;
    }
    let pitch_radius = 0.5 * geometry.pitch_diameter?;
    let sign = match params.hand {
        Hand::Right => 1.0,
        Hand::Left => -1.0,
    };
    Some(Twist {
        rate: sign * params.helix_angle.tan() / pitch_radius,
        tolerance: 0.5 * flank_tolerance(params),
        herringbone: params.herringbone,
        max_spans: MAX_TWIST_SPANS,
    })
}

/// Radii and tooth angles shared by [`gear_geometry`] and [`make_gear`].
struct Circular {
    pitch_radius: f64,
    base_radius: f64,
    /// External: tip circle. Internal: inner tip circle.
    tip_radius: f64,
    /// External and internal: root circle.
    root_radius: f64,
    /// Half-angle term of the external tooth or the internal tooth space.
    psi0: f64,
}

fn circular(params: &GearParams) -> Circular {
    let params = &transverse(params);
    let m = params.module;
    let x = params.profile_shift;
    let alpha = params.pressure_angle;
    let pitch_radius = 0.5 * m * f64::from(params.teeth);
    let base_radius = pitch_radius * alpha.cos();
    let involute_alpha = alpha.tan() - alpha;
    // External tooth thickness, or internal space width, at the pitch circle.
    let nominal = m * (2.0 * x).mul_add(alpha.tan(), PI / 2.0);
    let (tip_radius, root_radius, width) = match params.kind {
        GearKind::Internal => (
            m.mul_add(-(params.addendum - x), pitch_radius),
            m.mul_add(params.dedendum + x, pitch_radius),
            nominal + params.backlash,
        ),
        GearKind::External | GearKind::Rack => (
            m.mul_add(params.addendum + x, pitch_radius),
            m.mul_add(-(params.dedendum - x), pitch_radius),
            nominal - params.backlash,
        ),
    };
    Circular {
        pitch_radius,
        base_radius,
        tip_radius,
        root_radius,
        psi0: width / (2.0 * pitch_radius) + involute_alpha,
    }
}

/// Closed-form dimensions and guideline warnings for valid parameters.
///
/// A geometry with a non-positive `tip_thickness` is still returned; only
/// [`make_gear`] refuses unbuildable profiles.
///
/// # Errors
///
/// [`GearError::Refused`] with [`GearRefusal::InvalidParameter`] when a
/// parameter is out of range.
pub fn gear_geometry(params: &GearParams) -> Result<GearGeometry, GearError> {
    validate(params)?;
    Ok(helical_geometry(params))
}

/// [`gear_geometry`] without validation, for a spur or helical gear (or a
/// worm's helical equivalent, whose helix angle exceeds the gear limit).
fn helical_geometry(params: &GearParams) -> GearGeometry {
    let mut geometry = spur_geometry(&transverse(params));
    if params.helix_angle > 0.0 {
        // Report the normal module and convert the transverse shift
        // coefficients back to normal ones (lengths are unchanged).
        let cos = params.helix_angle.cos();
        geometry.module = params.module;
        geometry.helix_angle = params.helix_angle;
        geometry.lead = geometry
            .pitch_diameter
            .map(|diameter| PI * diameter / params.helix_angle.tan());
        geometry.min_profile_shift_without_undercut = geometry
            .min_profile_shift_without_undercut
            .map(|shift| shift / cos);
        for warning in &mut geometry.warnings {
            if let GearWarning::Undercut { min_profile_shift } = warning {
                *min_profile_shift /= cos;
            }
        }
    }
    geometry
}

/// [`gear_geometry`] of a spur gear (or a helical gear's transverse
/// equivalent), without validation.
fn spur_geometry(params: &GearParams) -> GearGeometry {
    let m = params.module;
    let circular_pitch = PI * m;
    let mut warnings = Vec::new();
    let geometry = if params.kind == GearKind::Rack {
        let thickness = circular_pitch.mul_add(0.5, -params.backlash);
        let tip_thickness =
            (2.0 * params.addendum * m).mul_add(-params.pressure_angle.tan(), thickness);
        GearGeometry {
            module: m,
            transverse_module: m,
            helix_angle: 0.0,
            lead: None,
            pitch_diameter: None,
            base_diameter: None,
            tip_diameter: None,
            root_diameter: None,
            circular_pitch,
            tooth_thickness: thickness,
            tip_thickness,
            min_teeth_without_undercut: None,
            min_profile_shift_without_undercut: None,
            warnings: Vec::new(),
        }
    } else {
        let c = circular(params);
        let half_pitch = PI / f64::from(params.teeth);
        // Half-angle of the flank (tooth or space) at radius ρ.
        let flank_angle = |radius: f64| {
            if radius >= c.base_radius {
                c.psi0 - profile::involute_function(c.base_radius, radius)
            } else {
                c.psi0
            }
        };
        let (tooth_thickness, tip_thickness, undercut) = if params.kind == GearKind::External {
            let sin2 = params.pressure_angle.sin().powi(2);
            let min_teeth = 2.0 * params.addendum / sin2;
            let min_shift = (0.5 * f64::from(params.teeth)).mul_add(-sin2, params.addendum);
            (
                2.0 * c.pitch_radius
                    * (c.psi0 - (params.pressure_angle.tan() - params.pressure_angle)),
                2.0 * c.tip_radius * flank_angle(c.tip_radius),
                Some((min_teeth, min_shift)),
            )
        } else {
            let space = 2.0
                * c.pitch_radius
                * (c.psi0 - (params.pressure_angle.tan() - params.pressure_angle));
            (
                circular_pitch - space,
                2.0 * c.tip_radius * (half_pitch - flank_angle(c.tip_radius)),
                None,
            )
        };
        if let Some((_, min_shift)) = undercut
            && params.profile_shift < min_shift - 1e-12
        {
            warnings.push(GearWarning::Undercut {
                min_profile_shift: min_shift,
            });
        }
        let rim = match params.kind {
            GearKind::External => params.bore_diameter.map(|bore| c.root_radius - 0.5 * bore),
            _ => params.rim_diameter.map(|rim| 0.5 * rim - c.root_radius),
        };
        if let Some(thickness) = rim
            && thickness > 0.0
            && thickness < THIN_RIM_MODULES * m
        {
            warnings.push(GearWarning::ThinRim { thickness });
        }
        GearGeometry {
            module: m,
            transverse_module: m,
            helix_angle: 0.0,
            lead: None,
            pitch_diameter: Some(2.0 * c.pitch_radius),
            base_diameter: Some(2.0 * c.base_radius),
            tip_diameter: Some(2.0 * c.tip_radius),
            root_diameter: Some(2.0 * c.root_radius),
            circular_pitch,
            tooth_thickness,
            tip_thickness,
            min_teeth_without_undercut: undercut.map(|(teeth, _)| teeth),
            min_profile_shift_without_undercut: undercut.map(|(_, shift)| shift),
            warnings: Vec::new(),
        }
    };
    if geometry.tip_thickness > 0.0 && geometry.tip_thickness < THIN_TIP_MODULES * m {
        warnings.push(GearWarning::ThinTip {
            thickness: geometry.tip_thickness,
        });
    }
    GearGeometry {
        warnings,
        ..geometry
    }
}

/// Build a gear body.
///
/// The whole construction runs in one transaction: a refusal or failure
/// leaves the topology as it was.
///
/// # Errors
///
/// [`GearError::Refused`] when the parameters are out of range or describe
/// an unbuildable profile (with the dimensions attached when available);
/// [`GearError::Operations`] when the extrusion itself fails.
pub fn make_gear(
    topo: &mut Topology,
    params: &GearParams,
    placement: &GearPlacement,
) -> Result<GearOutcome, GearError> {
    let geometry = gear_geometry(params)?;
    let frame = Frame::new(placement, params.kind)?;
    remus_topology::transaction::run_transacted(topo, |topo| {
        build(topo, params, &frame, placement.phase, &geometry)
    })
    .map_err(|error| error.with_geometry(&geometry))
}

/// The local outline plus the circular rim (internal) or bore (external).
struct LocalOutline {
    outline: profile::Outline,
    rim_radius: Option<f64>,
    bore_radius: Option<f64>,
}

fn local_outline(
    params: &GearParams,
    geometry: &GearGeometry,
    phase: f64,
) -> Result<LocalOutline, GearError> {
    let params = &transverse(params);
    let m = params.module;
    let tolerance = params.tolerance.unwrap_or(DEFAULT_TOLERANCE_PER_MODULE * m);
    let fillet_radius = params.root_fillet * m;
    if params.kind == GearKind::Rack {
        let outline = profile::rack_outline(&RackTooth {
            teeth: params.teeth,
            pitch: geometry.circular_pitch,
            thickness: geometry.tooth_thickness,
            tip_height: params.addendum * m,
            root_depth: params.dedendum * m,
            back_depth: params
                .back_depth
                .unwrap_or(DEFAULT_RACK_BACK_DEPTH_PER_MODULE * m),
            pressure_angle: params.pressure_angle,
            fillet_radius,
        })?;
        return Ok(LocalOutline {
            outline,
            rim_radius: None,
            bore_radius: None,
        });
    }
    let c = circular(params);
    if params.kind == GearKind::External && c.root_radius <= 0.0 {
        return Err(GearError::invalid(
            "the root circle has no positive radius; use more teeth or a smaller dedendum",
        ));
    }
    if params.kind == GearKind::Internal && c.tip_radius <= 0.0 {
        return Err(GearError::invalid(
            "the inner tip circle has no positive radius; use more teeth or a smaller addendum",
        ));
    }
    if params.kind == GearKind::External && c.tip_radius <= c.base_radius {
        return Err(GearError::refused(
            GearRefusal::TipBelowBaseCircle,
            "the tip circle lies inside the base circle, so the teeth have no involute",
        ));
    }
    let tooth = CircularTooth {
        teeth: params.teeth,
        base_radius: c.base_radius,
        inner_or_tip_radius: c.tip_radius,
        root_radius: c.root_radius,
        fillet_radius,
        psi0: c.psi0,
        phase,
        tolerance,
    };
    if params.kind == GearKind::External {
        if let Some(bore) = params.bore_diameter
            && 0.5 * bore >= c.root_radius
        {
            return Err(GearError::refused(
                GearRefusal::BoreTooLarge,
                format!(
                    "bore diameter {bore} reaches the root diameter {}",
                    2.0 * c.root_radius
                ),
            ));
        }
        Ok(LocalOutline {
            outline: profile::external_outline(&tooth)?,
            rim_radius: None,
            bore_radius: params.bore_diameter.map(|bore| 0.5 * bore),
        })
    } else {
        let rim = params.rim_diameter.unwrap_or(0.0);
        if 0.5 * rim <= c.root_radius {
            return Err(GearError::refused(
                GearRefusal::RimTooSmall,
                format!(
                    "rim diameter {rim} does not enclose the root diameter {}",
                    2.0 * c.root_radius
                ),
            ));
        }
        Ok(LocalOutline {
            outline: profile::internal_outline(&tooth)?,
            rim_radius: Some(0.5 * rim),
            bore_radius: None,
        })
    }
}

fn build(
    topo: &mut Topology,
    params: &GearParams,
    frame: &Frame,
    phase: f64,
    geometry: &GearGeometry,
) -> Result<GearOutcome, GearError> {
    let local = local_outline(params, geometry, phase)?;
    let (profile_bound, spans_per_flank) = flank_bound(&local.outline, geometry);
    let (solid, face_roles, twisted) = gear_body(
        topo,
        local,
        frame,
        frame.x,
        params.face_width,
        twist(params, geometry).into(),
    )?;
    Ok(GearOutcome {
        solid,
        geometry: geometry.clone(),
        flank_deviation_bound: profile_bound + twisted.bound,
        spans_per_flank,
        spans_along_axis: twisted.spans,
        face_roles,
    })
}

/// Proven flank deviation bound (with a rounding allowance) and span count.
fn flank_bound(outline: &profile::Outline, geometry: &GearGeometry) -> (f64, usize) {
    outline.flank.as_ref().map_or((0.0, 0), |f| {
        // The remainder bound is exact-arithmetic; allow for rounding of the
        // control points at the gear's largest radius.
        let scale = geometry
            .root_diameter
            .unwrap_or(0.0)
            .max(geometry.tip_diameter.unwrap_or(0.0));
        (
            (64.0 * f64::EPSILON).mul_add(scale.max(geometry.module), f.remainder_bound),
            f.spans,
        )
    })
}

/// Extrude a local outline into a gear body (twisted for a helical gear)
/// and name every face from the extrusion's construction record: each side
/// face is matched to the profile edge it was swept from, by edge identity.
/// Also reports the twist's deviation bound and spans (zero without one).
fn gear_body(
    topo: &mut Topology,
    local: LocalOutline,
    frame: &Frame,
    circle_ref: Vec3,
    face_width: f64,
    sweep: Sweep,
) -> Result<GearBody, GearError> {
    let vertex_tol = Tolerance::new().linear;
    let LocalOutline {
        outline,
        rim_radius,
        bore_radius,
    } = local;
    // Profile edge index -> role, for every edge of every wire.
    let mut edge_roles: Vec<(usize, GearFaceRole)> = Vec::new();
    let (outer, inner) = if let Some(rim_radius) = rim_radius {
        let rim = make_circle_edge_with_ref(
            topo,
            frame.origin,
            frame.axis,
            rim_radius,
            circle_ref,
            vertex_tol,
        )?;
        edge_roles.push((rim.index(), GearFaceRole::Rim));
        let rim_wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true)?);
        let hole: Vec<Segment> = outline
            .segments
            .into_iter()
            .rev()
            .map(Segment::reversed)
            .collect();
        let (wire, edges) = build_wire(topo, &hole, frame, vertex_tol)?;
        edge_roles.extend(
            edges
                .iter()
                .zip(outline.roles.iter().rev())
                .map(|(edge, role)| (edge.index(), *role)),
        );
        (rim_wire, vec![wire])
    } else {
        let (outer, edges) = build_wire(topo, &outline.segments, frame, vertex_tol)?;
        edge_roles.extend(
            edges
                .iter()
                .zip(&outline.roles)
                .map(|(edge, role)| (edge.index(), *role)),
        );
        let mut inner = Vec::new();
        if let Some(radius) = bore_radius {
            // Wound clockwise about the axis, opposite the outer boundary.
            let bore = make_circle_edge_with_ref(
                topo,
                frame.origin,
                -frame.axis,
                radius,
                circle_ref,
                vertex_tol,
            )?;
            edge_roles.push((bore.index(), GearFaceRole::Bore));
            inner.push(topo.add_wire(Wire::new(vec![OrientedEdge::new(bore, true)], true)?));
        }
        (outer, inner)
    };
    edge_roles.compact_sort_by_key(|entry| entry.0);

    let d = frame.axis.dot(Vec3::new(
        frame.origin.x(),
        frame.origin.y(),
        frame.origin.z(),
    ));
    let face = topo.add_face(Face::new(
        outer,
        inner,
        FaceSurface::Plane {
            normal: frame.axis,
            d,
        },
    ));
    let tolerance_refusal = |twist: Twist| {
        move |error| match error {
            OperationsError::Math(remus_math::MathError::ResourceLimitExceeded { .. }) => {
                GearError::refused(
                    GearRefusal::ToleranceTooTight,
                    format!(
                        "the helix needs more than {} spans for a tolerance of {}",
                        twist.max_spans, twist.tolerance
                    ),
                )
            }
            other => other.into(),
        }
    };
    // The extruded halves, each with the half its sides are named for.
    let (solid, halves, twist_bound) = match sweep {
        Sweep::Straight => {
            let (solid, construction) =
                crate::extrude::extrude_impl(topo, face, frame.axis, face_width)?;
            (solid, vec![(construction, None)], TwistReport::default())
        }
        Sweep::Twist(twist) if twist.herringbone => {
            let built = crate::extrude::twist::herringbone_extrude_impl(
                topo,
                face,
                frame.origin,
                frame.axis,
                face_width,
                twist.rate,
                twist.tolerance,
                twist.max_spans,
            )
            .map_err(tolerance_refusal(twist))?;
            (
                built.solid,
                vec![
                    (built.lower, Some(GearEnd::Start)),
                    (built.upper, Some(GearEnd::End)),
                ],
                TwistReport {
                    bound: built.deviation_bound,
                    spans: built.spans,
                },
            )
        }
        Sweep::Twist(twist) => {
            let twisted = crate::extrude::twist::twisted_extrude_impl(
                topo,
                face,
                frame.origin,
                frame.axis,
                face_width,
                twist.rate,
                twist.tolerance,
                twist.max_spans,
            )
            .map_err(tolerance_refusal(twist))?;
            (
                twisted.solid,
                vec![(twisted.construction, None)],
                TwistReport {
                    bound: twisted.deviation_bound,
                    spans: twisted.spans,
                },
            )
        }
        Sweep::Taper { apex_distance } => {
            let tapered = crate::extrude::taper::tapered_extrude_impl(
                topo,
                face,
                frame.origin + frame.axis * apex_distance,
                frame.axis,
                face_width,
            )?;
            (
                tapered.solid,
                vec![(tapered.construction, None)],
                TwistReport::default(),
            )
        }
    };

    let role_of = |edge: EdgeId| {
        edge_roles
            .binary_search_by_key(&edge.index(), |entry| entry.0)
            .ok()
            .map(|i| edge_roles[i].1)
    };
    let (Some((first, _)), Some((last, _))) = (halves.first(), halves.last()) else {
        return Err(OperationsError::InvalidInput {
            reason: "gear extrusion has no construction record".to_owned(),
        }
        .into());
    };
    let mut face_roles = vec![
        (first.bottom, GearFaceRole::Cap(GearEnd::Start).into()),
        (last.top, GearFaceRole::Cap(GearEnd::End).into()),
    ];
    // A herringbone gear's end half is swept from its start half's top
    // edges, so those carry the roles across the mid-plane.
    let mut carried: Vec<(usize, GearFaceRole)> = Vec::new();
    for (construction, half) in &halves {
        let wires = std::iter::once((&construction.outer, &construction.outer_sides)).chain(
            construction
                .holes
                .iter()
                .map(|hole| (&hole.inner, &hole.sides)),
        );
        for (wire, sides) in wires {
            for ((&edge, &top), &side) in wire
                .source_edges()
                .iter()
                .zip(wire.top_edge_ids())
                .zip(sides)
            {
                let role = role_of(edge)
                    .or_else(|| {
                        carried
                            .iter()
                            .find(|entry| entry.0 == edge.index())
                            .map(|entry| entry.1)
                    })
                    .ok_or_else(|| OperationsError::InvalidInput {
                        reason: "gear side face has no profile edge in its construction record"
                            .to_owned(),
                    })?;
                carried.push((top.index(), role));
                face_roles.push((side, GearFace { role, half: *half }));
            }
        }
    }
    Ok((solid, face_roles, twist_bound))
}

/// Orthonormal placement frame: local `(x, y, 0)` maps to
/// `origin + x·X + y·Y`, with `Y = axis × X`.
struct Frame {
    origin: Point3,
    axis: Vec3,
    x: Vec3,
    y: Vec3,
}

impl Frame {
    // A rack's phase must be exactly zero, not merely small.
    #[allow(clippy::float_cmp)]
    fn new(placement: &GearPlacement, kind: GearKind) -> Result<Self, GearError> {
        let finite = |v: Vec3| v.0.iter().all(|c| c.is_finite());
        require(
            placement.origin.0.iter().all(|c| c.is_finite())
                && finite(placement.axis)
                && finite(placement.x_direction)
                && placement.phase.is_finite(),
            || "placement must be finite".to_owned(),
        )?;
        require(kind != GearKind::Rack || placement.phase == 0.0, || {
            "a rack takes no phase; place it with its origin instead".to_owned()
        })?;
        let axis = placement
            .axis
            .normalize()
            .map_err(|_| GearError::invalid("placement axis must be non-zero"))?;
        let projected = placement.x_direction - axis * placement.x_direction.dot(axis);
        let x = projected.normalize().map_err(|_| {
            GearError::invalid("placement x direction must not be parallel to the axis")
        })?;
        Ok(Self {
            origin: placement.origin,
            axis,
            x,
            y: axis.cross(x),
        })
    }

    fn point(&self, p: Point3) -> Point3 {
        self.origin + self.x * p.x() + self.y * p.y()
    }
}

/// Turn a closed local outline into a wire with shared vertices.
fn build_wire(
    topo: &mut Topology,
    segments: &[Segment],
    frame: &Frame,
    tolerance: f64,
) -> Result<(WireId, Vec<EdgeId>), GearError> {
    let vertices: Vec<_> = segments
        .iter()
        .map(|segment| topo.add_vertex(Vertex::new(frame.point(segment.start()), tolerance)))
        .collect();
    let mut edges = Vec::with_capacity(segments.len());
    for (i, segment) in segments.iter().enumerate() {
        let start = vertices[i];
        let end = vertices[(i + 1) % vertices.len()];
        let oriented = match segment {
            Segment::Line { .. } => {
                OrientedEdge::new(topo.add_edge(Edge::new(start, end, EdgeCurve::Line)), true)
            }
            Segment::Arc {
                center,
                start: from,
                end: to,
            } => {
                let local_turn = (from.x() - center.x()).mul_add(
                    to.y() - center.y(),
                    -(from.y() - center.y()) * (to.x() - center.x()),
                );
                let normal = if local_turn > 0.0 {
                    frame.axis
                } else {
                    -frame.axis
                };
                let center = frame.point(*center);
                let from = frame.point(*from);
                let to = frame.point(*to);
                let radial = from - center;
                let circle = Circle3D::new_with_ref(center, normal, radial.length(), radial)?;
                let t0 = circle.project(from);
                let span = (circle.project(to) - t0).rem_euclid(TAU);
                if span <= 0.0 || span >= PI {
                    return Err(OperationsError::InvalidInput {
                        reason: format!("gear outline arc spans {span} rad"),
                    }
                    .into());
                }
                let mut edge =
                    Edge::with_tolerance(start, end, EdgeCurve::Circle(circle), Some(tolerance));
                edge.set_trim(Some((t0, t0 + span)));
                OrientedEdge::new(topo.add_edge(edge), true)
            }
            Segment::Flank { curve, reversed } => {
                let curve = profile::map_curve(curve, |p| frame.point(p))?;
                let domain = curve.domain();
                let (curve_start, curve_end) = if *reversed {
                    (end, start)
                } else {
                    (start, end)
                };
                let mut edge = Edge::with_tolerance(
                    curve_start,
                    curve_end,
                    EdgeCurve::NurbsCurve(curve),
                    Some(tolerance),
                );
                edge.set_trim(Some(domain));
                OrientedEdge::new(topo.add_edge(edge), !*reversed)
            }
        };
        edges.push(oriented);
    }
    let ids = edges.iter().map(OrientedEdge::edge).collect();
    Ok((topo.add_wire(Wire::new(edges, true)?), ids))
}

#[cfg(test)]
mod tests;
