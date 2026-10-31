//! Involute spur, helical, herringbone, internal, rack, straight bevel and
//! worm gear construction (`remus_operations::gear`).
//!
//! `makeGearDetailed(spec)` builds a new gear body,
//! `makeBevelGearDetailed(spec)` a straight bevel gear,
//! `makeWormDetailed(spec)` a worm and `makeWormWheelDetailed(spec)` its
//! wheel, `makeEnvelopingWormDetailed(spec)` a double-enveloping worm and
//! `makeEnvelopingWormWheelDetailed(spec)` its wheel, and
//! `cutGearTeethDetailed(solid, face, spec)` cuts teeth along the whole
//! length of a solid's cylindrical face (external teeth on a boss, internal
//! teeth in a bore). Both return the typed [`SolidOperationDetailedResult`]
//! envelope; the `executeBatch` / `executeBatchV2` ops of the same names take
//! the spec as their `args` (`makeGearDetailed`, `makeBevelGearDetailed`,
//! `makeWormDetailed`, `makeWormWheelDetailed`, `makeEnvelopingWormDetailed`,
//! `makeEnvelopingWormWheelDetailed`) or as `args.spec` beside
//! `args.solid` and `args.face` (`cutGearTeethDetailed`), so direct and batch
//! results are identical by construction.
//!
//! `makeGearDetailed` spec:
//!
//! ```json
//! {
//!   "kind": "external" | "internal" | "rack",
//!   "module": 2, "teeth": 20, "faceWidth": 5,
//!   "pressureAngle": 20,          // degrees, default 20
//!   "profileShift": 0, "backlash": 0,
//!   "addendum": 1, "dedendum": 1.25, "rootFillet": 0.38,
//!   "tolerance": 2e-5,            // default 1e-5 · module
//!   "boreDiameter": 10,           // external only
//!   "rimDiameter": 60,            // internal, required
//!   "backDepth": 4,               // rack only, default 2 · module
//!   "origin": [0, 0, 0], "axis": [0, 0, 1], "xDirection": [1, 0, 0],
//!   "phase": 0,                   // degrees; circular gears only
//!   "helixAngle": 0,              // degrees in [0, 45]; circular gears only
//!   "hand": "right" | "left",     // default "right"
//!   "herringbone": false          // needs a helixAngle
//! }
//! ```
//!
//! A non-zero `helixAngle` makes a helical gear (ISO 21771): `module`,
//! `pressureAngle` and the coefficients are normal-section values, and the
//! teeth turn about `axis` as they advance (counter-clockwise for a right
//! hand). `phase` places the first tooth on the start cap. `herringbone`
//! reverses the hand at the middle of the face width (`hand` is the start
//! half's); each half's swept faces carry `.start-half` or `.end-half` on
//! their role, e.g. `tooth[3].flank.plus.end-half`.
//!
//! `cutGearTeethDetailed` spec: `kind` (`"external"` for a boss,
//! `"internal"` for a bore), `teeth`, `faceWidth` (the face's length:
//! partial spans refuse), `fit` (`"keepDiameter"`, the default, solves the
//! module so the tip (inner tip) diameter is the cylinder's; `"module"` uses
//! `module`), the same tooth-form fields as above (including `helixAngle`,
//! `hand` and `herringbone`), and optional `xDirection` and `phase` (default: the face's
//! seam direction).
//!
//! `makeBevelGearDetailed` spec (heel values; the apex lies along `axis`
//! from the heel plane at `origin`, and `cap.start` is the heel):
//!
//! ```json
//! {
//!   "module": 2, "teeth": 20, "mateTeeth": 40,
//!   "faceWidth": 8,               // along the pitch cone
//!   "shaftAngle": 90,             // degrees, default 90
//!   "pressureAngle": 20, "addendum": 1, "dedendum": 1.25, "backlash": 0,
//!   "tolerance": 2e-5, "boreDiameter": 10,
//!   "origin": [0, 0, 0], "axis": [0, 0, 1], "xDirection": [1, 0, 0],
//!   "phase": 0
//! }
//! ```
//!
//! Its flanks are certified fits of the spherical involute
//! (`details.flankDeviationBound`), and `details.geometry` carries the cone
//! angles in degrees.
//!
//! `makeWormDetailed` spec (a cylindrical involute (ZI) worm; axial-section
//! values, coefficients in axial modules):
//!
//! ```json
//! {
//!   "module": 2,                  // axial module
//!   "starts": 1, "diameterFactor": 10, "length": 40,
//!   "pressureAngle": 20,          // normal, degrees
//!   "addendum": 1, "dedendum": 1.25, "rootFillet": 0.38,
//!   "backlash": 0,                // axial thread thinning
//!   "hand": "right" | "left",
//!   "tolerance": 2e-5, "boreDiameter": 8,
//!   "origin": [0, 0, 0], "axis": [0, 0, 1], "xDirection": [1, 0, 0],
//!   "phase": 0
//! }
//! ```
//!
//! `makeWormWheelDetailed` takes the worm's tooth-form fields (`module`,
//! `starts`, `diameterFactor`, `pressureAngle`, `addendum`, `dedendum`,
//! `rootFillet`, `hand`) with the wheel's own `teeth`, `faceWidth` and
//! optional `backlash` (circumferential, at its pitch circle),
//! `tolerance`, `boreDiameter` and placement, and builds the helical wheel
//! that meshes with that worm: the worm's hand, helix angle equal to the
//! lead angle. Its `details.geometry` is a helical gear's plus
//! `centerDistance`, the distance between the crossed axes. The pair is a
//! crossed helical drive (point contact), not a hobbed throated wheel.
//! `"throated": true` builds it on a throated blank: across the throat (half
//! way across the face, where the worm sits) the tips follow the torus that
//! keeps the worm's root clearance, either side of it they stand at
//! `outsideDiameter` (default the throat diameter plus one module). Its
//! geometry adds `throatDiameter`, `outsideDiameter`, `throatRadius` and
//! `throatWidth`, its details `throatCurveBound` (the certified fit of the
//! curves where the flanks meet the throat), and its roles
//! `tooth[k].throat` and `tooth[k].tip.start-half` / `.end-half`.
//! Worm flanks are certified fits of the involute helicoid, and its
//! `details.geometry` carries the lead angle in degrees.
//!
//! `makeEnvelopingWormDetailed` spec (a double-enveloping Hindley worm,
//! whose hourglass wraps the wheel of `wheelTeeth` teeth; values at the
//! throat, coefficients in modules):
//!
//! ```json
//! {
//!   "module": 2,                  // the wheel's transverse module
//!   "starts": 1, "wheelTeeth": 30, "diameterFactor": 10, "length": 24,
//!   "pressureAngle": 20,          // the wheel's straight flanks, degrees
//!   "addendum": 1, "dedendum": 1.25,
//!   "backlash": 0,                // thread thinning on the wheel's pitch circle
//!   "hand": "right" | "left",
//!   "tolerance": 2e-5, "boreDiameter": 8,
//!   "origin": [0, 0, 0], "axis": [0, 0, 1], "xDirection": [1, 0, 0],
//!   "phase": 0
//! }
//! ```
//!
//! Its thread is the surface the wheel's straight flank lines sweep, so its
//! lands and root floors are exact torus patches (`tooth[k].tip[p]`,
//! `root[k][p]`, numbered from the start cap) and its flanks certified ruled
//! patches (`tooth[k].flank.plus[p]`, `details.flankDeviationBound`).
//! `details.geometry` carries the center distance and throat and end
//! diameters, the lead and wrap angles in degrees, and `details.wheelCenter`
//! and `details.wheelAxis` place the wheel it envelops.
//!
//! `makeEnvelopingWormWheelDetailed` takes the worm's tooth-form fields
//! (`module`, `starts`, `wheelTeeth`, `diameterFactor`, `length`,
//! `pressureAngle`, `addendum`, `dedendum`, `hand`) with the wheel's own
//! `faceWidth` and optional `backlash` (on its pitch circle),
//! `outsideDiameter` (beside the throat, default the throat diameter plus
//! one module), `tolerance`, `boreDiameter` and placement, and builds the
//! wheel that worm drives. In the wheel's mid-plane its teeth are exactly
//! the straight-sided teeth the worm was swept by; off it each flank stays
//! straight in every section, turned and moved so the tooth stays inside
//! the tooth a hob shaped like the worm (without its backlash, so the two
//! backlashes add) cuts, with a crease along the mid-plane line where the
//! hobbed tooth has one. That fit is sampled, not proven:
//! `details.relief` is the largest sampled gap it leaves inside the hobbed
//! tooth. Its tips follow the throat torus across the throat and stand at
//! the outside diameter either side (`tooth[k].throat`,
//! `tooth[k].tip.start-half` / `.end-half`), and its flank roles are
//! `tooth[k].flank.plus.start-half` / `.end-half`. `details.geometry`
//! carries the center distance and the pitch, throat, root and outside
//! diameters, the throat radius and width, and the flank laws
//! (`flankTurn`, `flankOffset`, below then above the mid-plane);
//! `details.wormPlacement` is the placement (`origin`, `axis`,
//! `xDirection`, `phase` in degrees) that meshes the worm with it.
//!
//! Unknown keys refuse, so a misspelled option never silently falls back to
//! its default. Angles are degrees at this boundary and radians natively,
//! as for `revolve`.
//!
//! Success carries `details.quality`: `"approximate"` for circular gears,
//! whose involute flanks are certified cubic B-splines (the proven bound is
//! `details.flankDeviationBound`; for a helical gear it bounds the distance
//! to the exact involute helicoid, and `details.spansAlongAxis` counts the
//! twist's spans), and `"exact"` for a rack, whose flanks
//! are planes. `details.geometry` holds the closed-form dimensions
//! (including the solved `module`), `details.warnings` the guideline
//! warnings, and `details.faceRoles` a `{face, role}` entry per gear face
//! (`tooth[3].flank.plus`, `root[0]`, `cap.start`, …) from the construction
//! record. A refusal's native `code` is `gear:<reason>` and, when the
//! parameters were valid, its `details.geometry` carries the same dimensions
//! so a caller can show them while an input is out of range. Each call runs
//! in one native transaction, so a refusal leaves the session unchanged.

#![allow(clippy::missing_errors_doc)]

use serde::Deserialize;
use serde_json::{Map, Value, json};
use tsify::Tsify as _;
use wasm_bindgen::prelude::*;

use remus_math::vec::{Point3, Vec3};
use remus_operations::gear::{
    BevelGearGeometry, BevelGearOutcome, BevelGearParams, CylinderGearOptions,
    EnvelopingWheelOptions, EnvelopingWheelOutcome, EnvelopingWormGeometry, EnvelopingWormOutcome,
    EnvelopingWormParams, GearError, GearFit, GearGeometry, GearKind, GearOutcome, GearParams,
    GearPlacement, GearRefusal, GearWarning, Hand, ThroatedWheelOptions, WormGeometry, WormOutcome,
    WormParams, bevel_gear_geometry, cut_gear_teeth, enveloping_worm_geometry, make_bevel_gear,
    make_enveloping_worm, make_enveloping_worm_wheel, make_gear, make_throated_worm_wheel,
    make_worm, worm_geometry, worm_wheel,
};

use crate::error::StructuredWasmError;
use crate::handles::{face_id_to_u32, solid_id_to_u32};
use crate::kernel::RemusKernel;
use crate::types::SolidOperationDetailedResult;

#[derive(Debug, Default, Clone, Copy, Deserialize)]
#[serde(rename_all = "lowercase")]
enum HandSpec {
    #[default]
    Right,
    Left,
}

impl From<HandSpec> for Hand {
    fn from(hand: HandSpec) -> Self {
        match hand {
            HandSpec::Right => Self::Right,
            HandSpec::Left => Self::Left,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "lowercase")]
enum KindSpec {
    External,
    Internal,
    Rack,
}

/// The JSON gear specification (see the module docs).
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
struct GearSpec {
    kind: KindSpec,
    module: f64,
    teeth: u32,
    face_width: f64,
    #[serde(default = "default_pressure_angle")]
    pressure_angle: f64,
    #[serde(default)]
    profile_shift: f64,
    #[serde(default)]
    backlash: f64,
    #[serde(default = "default_addendum")]
    addendum: f64,
    #[serde(default = "default_dedendum")]
    dedendum: f64,
    #[serde(default = "default_root_fillet")]
    root_fillet: f64,
    tolerance: Option<f64>,
    bore_diameter: Option<f64>,
    rim_diameter: Option<f64>,
    back_depth: Option<f64>,
    #[serde(default)]
    origin: Option<[f64; 3]>,
    #[serde(default)]
    axis: Option<[f64; 3]>,
    #[serde(default)]
    x_direction: Option<[f64; 3]>,
    #[serde(default)]
    phase: f64,
    #[serde(default)]
    helix_angle: f64,
    #[serde(default)]
    hand: HandSpec,
    #[serde(default)]
    herringbone: bool,
}

const fn default_pressure_angle() -> f64 {
    20.0
}

const fn default_addendum() -> f64 {
    1.0
}

const fn default_dedendum() -> f64 {
    1.25
}

const fn default_root_fillet() -> f64 {
    0.38
}

impl GearSpec {
    fn split(self) -> (GearParams, GearPlacement) {
        let defaults = GearPlacement::default();
        let point = |p: [f64; 3]| Point3::new(p[0], p[1], p[2]);
        let vector = |v: [f64; 3]| Vec3::new(v[0], v[1], v[2]);
        let params = GearParams {
            kind: match self.kind {
                KindSpec::External => GearKind::External,
                KindSpec::Internal => GearKind::Internal,
                KindSpec::Rack => GearKind::Rack,
            },
            module: self.module,
            teeth: self.teeth,
            pressure_angle: self.pressure_angle.to_radians(),
            face_width: self.face_width,
            profile_shift: self.profile_shift,
            backlash: self.backlash,
            addendum: self.addendum,
            dedendum: self.dedendum,
            root_fillet: self.root_fillet,
            tolerance: self.tolerance,
            bore_diameter: self.bore_diameter,
            rim_diameter: self.rim_diameter,
            back_depth: self.back_depth,
            helix_angle: self.helix_angle.to_radians(),
            hand: self.hand.into(),
            herringbone: self.herringbone,
        };
        let placement = GearPlacement {
            origin: self.origin.map_or(defaults.origin, point),
            axis: self.axis.map_or(defaults.axis, vector),
            x_direction: self.x_direction.map_or(defaults.x_direction, vector),
            phase: self.phase.to_radians(),
        };
        (params, placement)
    }
}

/// The JSON bevel gear specification (see the module docs).
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
struct BevelGearSpec {
    module: f64,
    teeth: u32,
    mate_teeth: u32,
    face_width: f64,
    #[serde(default = "default_shaft_angle")]
    shaft_angle: f64,
    #[serde(default = "default_pressure_angle")]
    pressure_angle: f64,
    #[serde(default = "default_addendum")]
    addendum: f64,
    #[serde(default = "default_dedendum")]
    dedendum: f64,
    #[serde(default)]
    backlash: f64,
    tolerance: Option<f64>,
    bore_diameter: Option<f64>,
    #[serde(default)]
    origin: Option<[f64; 3]>,
    #[serde(default)]
    axis: Option<[f64; 3]>,
    #[serde(default)]
    x_direction: Option<[f64; 3]>,
    #[serde(default)]
    phase: f64,
}

const fn default_shaft_angle() -> f64 {
    90.0
}

impl BevelGearSpec {
    fn split(self) -> (BevelGearParams, GearPlacement) {
        let defaults = GearPlacement::default();
        let point = |p: [f64; 3]| Point3::new(p[0], p[1], p[2]);
        let vector = |v: [f64; 3]| Vec3::new(v[0], v[1], v[2]);
        let params = BevelGearParams {
            module: self.module,
            teeth: self.teeth,
            mate_teeth: self.mate_teeth,
            shaft_angle: self.shaft_angle.to_radians(),
            face_width: self.face_width,
            pressure_angle: self.pressure_angle.to_radians(),
            addendum: self.addendum,
            dedendum: self.dedendum,
            backlash: self.backlash,
            tolerance: self.tolerance,
            bore_diameter: self.bore_diameter,
        };
        let placement = GearPlacement {
            origin: self.origin.map_or(defaults.origin, point),
            axis: self.axis.map_or(defaults.axis, vector),
            x_direction: self.x_direction.map_or(defaults.x_direction, vector),
            phase: self.phase.to_radians(),
        };
        (params, placement)
    }
}

/// The JSON worm specification (see the module docs).
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
struct WormSpec {
    module: f64,
    starts: u32,
    diameter_factor: f64,
    length: f64,
    #[serde(default = "default_pressure_angle")]
    pressure_angle: f64,
    #[serde(default = "default_addendum")]
    addendum: f64,
    #[serde(default = "default_dedendum")]
    dedendum: f64,
    #[serde(default = "default_root_fillet")]
    root_fillet: f64,
    #[serde(default)]
    backlash: f64,
    #[serde(default)]
    hand: HandSpec,
    tolerance: Option<f64>,
    bore_diameter: Option<f64>,
    #[serde(default)]
    origin: Option<[f64; 3]>,
    #[serde(default)]
    axis: Option<[f64; 3]>,
    #[serde(default)]
    x_direction: Option<[f64; 3]>,
    #[serde(default)]
    phase: f64,
}

/// The JSON enveloping worm specification (see the module docs).
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
struct EnvelopingWormSpec {
    module: f64,
    starts: u32,
    wheel_teeth: u32,
    diameter_factor: f64,
    length: f64,
    #[serde(default = "default_pressure_angle")]
    pressure_angle: f64,
    #[serde(default = "default_addendum")]
    addendum: f64,
    #[serde(default = "default_dedendum")]
    dedendum: f64,
    #[serde(default)]
    backlash: f64,
    #[serde(default)]
    hand: HandSpec,
    tolerance: Option<f64>,
    bore_diameter: Option<f64>,
    #[serde(default)]
    origin: Option<[f64; 3]>,
    #[serde(default)]
    axis: Option<[f64; 3]>,
    #[serde(default)]
    x_direction: Option<[f64; 3]>,
    #[serde(default)]
    phase: f64,
}

impl EnvelopingWormSpec {
    fn split(self) -> (EnvelopingWormParams, GearPlacement) {
        let params = EnvelopingWormParams {
            module: self.module,
            starts: self.starts,
            wheel_teeth: self.wheel_teeth,
            diameter_factor: self.diameter_factor,
            pressure_angle: self.pressure_angle.to_radians(),
            length: self.length,
            addendum: self.addendum,
            dedendum: self.dedendum,
            backlash: self.backlash,
            hand: self.hand.into(),
            tolerance: self.tolerance,
            bore_diameter: self.bore_diameter,
        };
        let placement = placement_of(self.origin, self.axis, self.x_direction, self.phase);
        (params, placement)
    }
}

/// The JSON double-enveloping wheel specification (see the module docs).
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
struct EnvelopingWheelSpec {
    module: f64,
    starts: u32,
    wheel_teeth: u32,
    diameter_factor: f64,
    length: f64,
    #[serde(default = "default_pressure_angle")]
    pressure_angle: f64,
    #[serde(default = "default_addendum")]
    addendum: f64,
    #[serde(default = "default_dedendum")]
    dedendum: f64,
    #[serde(default)]
    hand: HandSpec,
    face_width: f64,
    #[serde(default)]
    backlash: f64,
    outside_diameter: Option<f64>,
    tolerance: Option<f64>,
    bore_diameter: Option<f64>,
    #[serde(default)]
    origin: Option<[f64; 3]>,
    #[serde(default)]
    axis: Option<[f64; 3]>,
    #[serde(default)]
    x_direction: Option<[f64; 3]>,
    #[serde(default)]
    phase: f64,
}

impl EnvelopingWheelSpec {
    fn split(self) -> (EnvelopingWormParams, EnvelopingWheelOptions, GearPlacement) {
        let worm = EnvelopingWormParams {
            module: self.module,
            starts: self.starts,
            wheel_teeth: self.wheel_teeth,
            diameter_factor: self.diameter_factor,
            pressure_angle: self.pressure_angle.to_radians(),
            length: self.length,
            addendum: self.addendum,
            dedendum: self.dedendum,
            backlash: 0.0,
            hand: self.hand.into(),
            tolerance: None,
            bore_diameter: None,
        };
        let options = EnvelopingWheelOptions {
            face_width: self.face_width,
            backlash: self.backlash,
            outside_diameter: self.outside_diameter,
            tolerance: self.tolerance,
            bore_diameter: self.bore_diameter,
        };
        let placement = placement_of(self.origin, self.axis, self.x_direction, self.phase);
        (worm, options, placement)
    }
}

/// A placement from optional JSON fields (degrees for the phase).
fn placement_of(
    origin: Option<[f64; 3]>,
    axis: Option<[f64; 3]>,
    x_direction: Option<[f64; 3]>,
    phase: f64,
) -> GearPlacement {
    let defaults = GearPlacement::default();
    let point = |p: [f64; 3]| Point3::new(p[0], p[1], p[2]);
    let vector = |v: [f64; 3]| Vec3::new(v[0], v[1], v[2]);
    GearPlacement {
        origin: origin.map_or(defaults.origin, point),
        axis: axis.map_or(defaults.axis, vector),
        x_direction: x_direction.map_or(defaults.x_direction, vector),
        phase: phase.to_radians(),
    }
}

impl WormSpec {
    fn split(self) -> (WormParams, GearPlacement) {
        let params = WormParams {
            module: self.module,
            starts: self.starts,
            diameter_factor: self.diameter_factor,
            pressure_angle: self.pressure_angle.to_radians(),
            length: self.length,
            addendum: self.addendum,
            dedendum: self.dedendum,
            root_fillet: self.root_fillet,
            backlash: self.backlash,
            hand: self.hand.into(),
            tolerance: self.tolerance,
            bore_diameter: self.bore_diameter,
        };
        let placement = placement_of(self.origin, self.axis, self.x_direction, self.phase);
        (params, placement)
    }
}

/// The JSON worm wheel specification (see the module docs).
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
struct WormWheelSpec {
    module: f64,
    starts: u32,
    diameter_factor: f64,
    teeth: u32,
    face_width: f64,
    #[serde(default = "default_pressure_angle")]
    pressure_angle: f64,
    #[serde(default = "default_addendum")]
    addendum: f64,
    #[serde(default = "default_dedendum")]
    dedendum: f64,
    #[serde(default = "default_root_fillet")]
    root_fillet: f64,
    #[serde(default)]
    hand: HandSpec,
    #[serde(default)]
    backlash: f64,
    tolerance: Option<f64>,
    bore_diameter: Option<f64>,
    #[serde(default)]
    throated: bool,
    outside_diameter: Option<f64>,
    #[serde(default)]
    origin: Option<[f64; 3]>,
    #[serde(default)]
    axis: Option<[f64; 3]>,
    #[serde(default)]
    x_direction: Option<[f64; 3]>,
    #[serde(default)]
    phase: f64,
}

impl WormWheelSpec {
    /// The mating worm (its length is irrelevant to the wheel), the wheel's
    /// tooth count, face width, backlash, tolerance, bore and placement.
    fn split(self) -> (WormParams, GearParams, GearPlacement) {
        let worm = WormParams {
            module: self.module,
            starts: self.starts,
            diameter_factor: self.diameter_factor,
            pressure_angle: self.pressure_angle.to_radians(),
            length: 1.0,
            addendum: self.addendum,
            dedendum: self.dedendum,
            root_fillet: self.root_fillet,
            backlash: 0.0,
            hand: self.hand.into(),
            tolerance: None,
            bore_diameter: None,
        };
        let wheel = GearParams {
            backlash: self.backlash,
            tolerance: self.tolerance,
            bore_diameter: self.bore_diameter,
            ..GearParams::standard(GearKind::External, self.module, self.teeth, self.face_width)
        };
        let placement = placement_of(self.origin, self.axis, self.x_direction, self.phase);
        (worm, wheel, placement)
    }
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "lowercase")]
enum CylinderKindSpec {
    External,
    Internal,
}

#[derive(Debug, Default, Deserialize)]
#[serde(rename_all = "camelCase")]
enum FitSpec {
    #[default]
    KeepDiameter,
    Module,
}

/// The JSON specification for teeth on an existing cylinder (see the
/// module docs).
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
struct CylinderGearSpec {
    kind: CylinderKindSpec,
    teeth: u32,
    face_width: f64,
    #[serde(default)]
    fit: FitSpec,
    module: Option<f64>,
    #[serde(default = "default_pressure_angle")]
    pressure_angle: f64,
    #[serde(default)]
    profile_shift: f64,
    #[serde(default)]
    backlash: f64,
    #[serde(default = "default_addendum")]
    addendum: f64,
    #[serde(default = "default_dedendum")]
    dedendum: f64,
    #[serde(default = "default_root_fillet")]
    root_fillet: f64,
    tolerance: Option<f64>,
    #[serde(default)]
    x_direction: Option<[f64; 3]>,
    #[serde(default)]
    phase: f64,
    #[serde(default)]
    helix_angle: f64,
    #[serde(default)]
    hand: HandSpec,
    #[serde(default)]
    herringbone: bool,
}

impl CylinderGearSpec {
    fn split(self) -> Result<(GearParams, CylinderGearOptions), StructuredWasmError> {
        let (fit, module) = match (self.fit, self.module) {
            (FitSpec::KeepDiameter, None) => (GearFit::KeepDiameter, 1.0),
            (FitSpec::KeepDiameter, Some(_)) => {
                return Err(StructuredWasmError::invalid_argument(
                    "the keepDiameter fit solves the module; omit it or use fit \"module\"",
                    Some("module"),
                ));
            }
            (FitSpec::Module, Some(module)) => (GearFit::UseModule, module),
            (FitSpec::Module, None) => {
                return Err(StructuredWasmError::invalid_argument(
                    "fit \"module\" needs a module",
                    Some("module"),
                ));
            }
        };
        let params = GearParams {
            kind: match self.kind {
                CylinderKindSpec::External => GearKind::External,
                CylinderKindSpec::Internal => GearKind::Internal,
            },
            module,
            teeth: self.teeth,
            pressure_angle: self.pressure_angle.to_radians(),
            face_width: self.face_width,
            profile_shift: self.profile_shift,
            backlash: self.backlash,
            addendum: self.addendum,
            dedendum: self.dedendum,
            root_fillet: self.root_fillet,
            tolerance: self.tolerance,
            bore_diameter: None,
            rim_diameter: None,
            back_depth: None,
            helix_angle: self.helix_angle.to_radians(),
            hand: self.hand.into(),
            herringbone: self.herringbone,
        };
        let options = CylinderGearOptions {
            fit,
            x_direction: self.x_direction.map(|v| Vec3::new(v[0], v[1], v[2])),
            phase: self.phase.to_radians(),
        };
        Ok((params, options))
    }
}

/// Success details shared by both gear entry points.
fn outcome_details(outcome: &GearOutcome, kind: GearKind) -> Map<String, Value> {
    let mut details = Map::new();
    let exact = kind == GearKind::Rack;
    details.insert(
        "quality".into(),
        Value::from(if exact { "exact" } else { "approximate" }),
    );
    if !exact {
        details.insert("approximated".into(), Value::from("involute flanks"));
    }
    details.insert(
        "flankDeviationBound".into(),
        Value::from(outcome.flank_deviation_bound),
    );
    details.insert("spansPerFlank".into(), Value::from(outcome.spans_per_flank));
    details.insert(
        "spansAlongAxis".into(),
        Value::from(outcome.spans_along_axis),
    );
    details.insert("geometry".into(), geometry_json(&outcome.geometry));
    details.insert("warnings".into(), warnings_json(&outcome.geometry.warnings));
    details.insert(
        "faceRoles".into(),
        Value::Array(
            outcome
                .face_roles
                .iter()
                .map(|(face, role)| json!({ "face": face_id_to_u32(*face), "role": role.name() }))
                .collect(),
        ),
    );
    details
}

fn geometry_json(geometry: &GearGeometry) -> Value {
    json!({
        "module": geometry.module,
        "transverseModule": geometry.transverse_module,
        "helixAngle": geometry.helix_angle.to_degrees(),
        "lead": geometry.lead,
        "pitchDiameter": geometry.pitch_diameter,
        "baseDiameter": geometry.base_diameter,
        "tipDiameter": geometry.tip_diameter,
        "rootDiameter": geometry.root_diameter,
        "circularPitch": geometry.circular_pitch,
        "toothThickness": geometry.tooth_thickness,
        "tipThickness": geometry.tip_thickness,
        "minTeethWithoutUndercut": geometry.min_teeth_without_undercut,
        "minProfileShiftWithoutUndercut": geometry.min_profile_shift_without_undercut,
    })
}

fn bevel_details(outcome: &BevelGearOutcome) -> Map<String, Value> {
    let mut details = Map::new();
    details.insert("quality".into(), Value::from("approximate"));
    details.insert(
        "approximated".into(),
        Value::from("spherical involute flanks"),
    );
    details.insert(
        "flankDeviationBound".into(),
        Value::from(outcome.flank_deviation_bound),
    );
    details.insert("spansPerFlank".into(), Value::from(outcome.spans_per_flank));
    details.insert("geometry".into(), bevel_geometry_json(&outcome.geometry));
    details.insert("warnings".into(), warnings_json(&outcome.geometry.warnings));
    details.insert(
        "faceRoles".into(),
        Value::Array(
            outcome
                .face_roles
                .iter()
                .map(|(face, role)| json!({ "face": face_id_to_u32(*face), "role": role.name() }))
                .collect(),
        ),
    );
    details
}

fn bevel_geometry_json(geometry: &BevelGearGeometry) -> Value {
    json!({
        "module": geometry.module,
        "pitchAngle": geometry.pitch_angle.to_degrees(),
        "matePitchAngle": geometry.mate_pitch_angle.to_degrees(),
        "baseConeAngle": geometry.base_cone_angle.to_degrees(),
        "faceAngle": geometry.face_angle.to_degrees(),
        "rootAngle": geometry.root_angle.to_degrees(),
        "outerConeDistance": geometry.outer_cone_distance,
        "innerConeDistance": geometry.inner_cone_distance,
        "pitchDiameter": geometry.pitch_diameter,
        "tipDiameter": geometry.tip_diameter,
        "rootDiameter": geometry.root_diameter,
        "apexDistance": geometry.apex_distance,
        "height": geometry.height,
        "circularPitch": geometry.circular_pitch,
        "toothThickness": geometry.tooth_thickness,
        "tipThickness": geometry.tip_thickness,
        "virtualTeeth": geometry.virtual_teeth,
    })
}

fn worm_details(outcome: &WormOutcome) -> Map<String, Value> {
    let mut details = Map::new();
    details.insert("quality".into(), Value::from("approximate"));
    details.insert(
        "approximated".into(),
        Value::from("involute helicoid flanks"),
    );
    details.insert(
        "flankDeviationBound".into(),
        Value::from(outcome.flank_deviation_bound),
    );
    details.insert("spansPerFlank".into(), Value::from(outcome.spans_per_flank));
    details.insert(
        "spansAlongAxis".into(),
        Value::from(outcome.spans_along_axis),
    );
    details.insert("geometry".into(), worm_geometry_json(&outcome.geometry));
    details.insert("warnings".into(), warnings_json(&outcome.geometry.warnings));
    details.insert(
        "faceRoles".into(),
        Value::Array(
            outcome
                .face_roles
                .iter()
                .map(|(face, role)| json!({ "face": face_id_to_u32(*face), "role": role.name() }))
                .collect(),
        ),
    );
    details
}

fn worm_geometry_json(geometry: &WormGeometry) -> Value {
    json!({
        "axialModule": geometry.axial_module,
        "normalModule": geometry.normal_module,
        "transverseModule": geometry.transverse_module,
        "leadAngle": geometry.lead_angle.to_degrees(),
        "axialPitch": geometry.axial_pitch,
        "lead": geometry.lead,
        "pitchDiameter": geometry.pitch_diameter,
        "tipDiameter": geometry.tip_diameter,
        "rootDiameter": geometry.root_diameter,
        "baseDiameter": geometry.base_diameter,
        "axialThickness": geometry.axial_thickness,
        "axialTipThickness": geometry.axial_tip_thickness,
    })
}

fn enveloping_details(outcome: &EnvelopingWormOutcome) -> Map<String, Value> {
    let mut details = Map::new();
    details.insert("quality".into(), Value::from("approximate"));
    details.insert("approximated".into(), Value::from("Hindley thread flanks"));
    details.insert(
        "flankDeviationBound".into(),
        Value::from(outcome.flank_deviation_bound),
    );
    details.insert(
        "spansAlongFlank".into(),
        Value::from(outcome.spans_along_flank),
    );
    details.insert(
        "geometry".into(),
        enveloping_geometry_json(&outcome.geometry),
    );
    details.insert("warnings".into(), warnings_json(&outcome.geometry.warnings));
    let c = outcome.wheel_center;
    let w = outcome.wheel_axis;
    details.insert("wheelCenter".into(), json!([c.x(), c.y(), c.z()]));
    details.insert("wheelAxis".into(), json!([w.x(), w.y(), w.z()]));
    details.insert(
        "faceRoles".into(),
        Value::Array(
            outcome
                .face_roles
                .iter()
                .map(|(face, role)| json!({ "face": face_id_to_u32(*face), "role": role.name() }))
                .collect(),
        ),
    );
    details
}

fn enveloping_wheel_details(outcome: &EnvelopingWheelOutcome) -> Map<String, Value> {
    let mut details = Map::new();
    details.insert("quality".into(), Value::from("approximate"));
    details.insert(
        "approximated".into(),
        Value::from("Hindley wheel flanks off the mid-plane"),
    );
    details.insert(
        "flankDeviationBound".into(),
        Value::from(outcome.flank_deviation_bound),
    );
    details.insert("relief".into(), Value::from(outcome.relief));
    details.insert(
        "geometry".into(),
        json!({
            "centerDistance": outcome.center_distance,
            "pitchDiameter": outcome.pitch_diameter,
            "throatDiameter": outcome.throat_diameter,
            "rootDiameter": outcome.root_diameter,
            "outsideDiameter": outcome.outside_diameter,
            "throatRadius": outcome.throat_radius,
            "throatWidth": outcome.throat_width,
            "flankTurn": outcome.flank_turn,
            "flankOffset": outcome.flank_offset,
        }),
    );
    let placement = &outcome.worm_placement;
    let (o, a, x) = (placement.origin, placement.axis, placement.x_direction);
    details.insert(
        "wormPlacement".into(),
        json!({
            "origin": [o.x(), o.y(), o.z()],
            "axis": [a.x(), a.y(), a.z()],
            "xDirection": [x.x(), x.y(), x.z()],
            "phase": placement.phase.to_degrees(),
        }),
    );
    details.insert(
        "faceRoles".into(),
        Value::Array(
            outcome
                .face_roles
                .iter()
                .map(|(face, role)| json!({ "face": face_id_to_u32(*face), "role": role.name() }))
                .collect(),
        ),
    );
    details
}

fn enveloping_geometry_json(geometry: &EnvelopingWormGeometry) -> Value {
    json!({
        "centerDistance": geometry.center_distance,
        "ratio": geometry.ratio,
        "leadAngle": geometry.lead_angle.to_degrees(),
        "throatPitchDiameter": geometry.throat_pitch_diameter,
        "throatTipDiameter": geometry.throat_tip_diameter,
        "throatRootDiameter": geometry.throat_root_diameter,
        "endTipDiameter": geometry.end_tip_diameter,
        "endRootDiameter": geometry.end_root_diameter,
        "wheelPitchDiameter": geometry.wheel_pitch_diameter,
        "baseDiameter": geometry.base_diameter,
        "wrapAngle": geometry.wrap_angle.to_degrees(),
        "threadThickness": geometry.thread_thickness,
        "tipThickness": geometry.tip_thickness,
        "rootWidth": geometry.root_width,
    })
}

fn warnings_json(warnings: &[GearWarning]) -> Value {
    Value::Array(
        warnings
            .iter()
            .map(|warning| match *warning {
                GearWarning::Undercut { min_profile_shift } => json!({
                    "code": warning.code(),
                    "minProfileShift": min_profile_shift,
                }),
                GearWarning::ThinTip { thickness } | GearWarning::ThinRim { thickness } => {
                    json!({ "code": warning.code(), "thickness": thickness })
                }
            })
            .collect(),
    )
}

fn gear_failure(error: GearError) -> StructuredWasmError {
    match error {
        GearError::Refused {
            code,
            reason,
            geometry,
        } => {
            let argument = match code {
                GearRefusal::PartialSpan => Some("faceWidth"),
                GearRefusal::BoreTooLarge => Some("boreDiameter"),
                GearRefusal::RimTooSmall => Some("rimDiameter"),
                GearRefusal::ToleranceTooTight => Some("tolerance"),
                _ => None,
            };
            let mut structured = StructuredWasmError::invalid_argument(
                format!("gear:{}: {reason}", code.code()),
                argument,
            )
            .with_detail("kernelCode", format!("gear:{}", code.code()));
            if let Some(geometry) = geometry {
                structured = structured
                    .with_detail("geometry", geometry_json(&geometry))
                    .with_detail("warnings", warnings_json(&geometry.warnings));
            }
            structured
        }
        GearError::Operations(error) => StructuredWasmError::from(error),
    }
}

#[wasm_bindgen]
impl RemusKernel {
    /// Build an involute spur gear, internal gear or rack from a JSON
    /// specification, as typed data (see the module docs for the fields).
    ///
    /// Never throws on a refusal: the envelope's `status` is `"error"` with
    /// a `gear:<reason>` code and the session unchanged.
    #[wasm_bindgen(js_name = "makeGearDetailed")]
    pub fn make_gear_detailed(
        &mut self,
        spec: &str,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        let result = match serde_json::from_str::<Value>(spec) {
            Ok(value) => self.make_gear_detailed_impl(&value),
            Err(error) => SolidOperationDetailedResult::error(
                StructuredWasmError::invalid_json(&error).with_direct_operation("makeGear"),
            ),
        };
        Ok(result.into_ts()?)
    }

    /// Build a straight bevel gear from a JSON specification, as typed data
    /// (see the module docs for the fields).
    ///
    /// Never throws on a refusal: the envelope's `status` is `"error"` with
    /// a `gear:<reason>` code and the session unchanged.
    #[wasm_bindgen(js_name = "makeBevelGearDetailed")]
    pub fn make_bevel_gear_detailed(
        &mut self,
        spec: &str,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        let result = match serde_json::from_str::<Value>(spec) {
            Ok(value) => self.make_bevel_gear_detailed_impl(&value),
            Err(error) => SolidOperationDetailedResult::error(
                StructuredWasmError::invalid_json(&error).with_direct_operation("makeBevelGear"),
            ),
        };
        Ok(result.into_ts()?)
    }

    /// Build a cylindrical involute (ZI) worm from a JSON specification, as
    /// typed data (see the module docs for the fields).
    ///
    /// Never throws on a refusal: the envelope's `status` is `"error"` with
    /// a `gear:<reason>` code and the session unchanged.
    #[wasm_bindgen(js_name = "makeWormDetailed")]
    pub fn make_worm_detailed(
        &mut self,
        spec: &str,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        let result = match serde_json::from_str::<Value>(spec) {
            Ok(value) => self.make_worm_detailed_impl(&value),
            Err(error) => SolidOperationDetailedResult::error(
                StructuredWasmError::invalid_json(&error).with_direct_operation("makeWorm"),
            ),
        };
        Ok(result.into_ts()?)
    }

    /// Build a double-enveloping (Hindley) worm from a JSON specification,
    /// as typed data (see the module docs for the fields).
    ///
    /// Never throws on a refusal: the envelope's `status` is `"error"` with
    /// a `gear:<reason>` code and the session unchanged.
    #[wasm_bindgen(js_name = "makeEnvelopingWormDetailed")]
    pub fn make_enveloping_worm_detailed(
        &mut self,
        spec: &str,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        let result = match serde_json::from_str::<Value>(spec) {
            Ok(value) => self.make_enveloping_worm_detailed_impl(&value),
            Err(error) => SolidOperationDetailedResult::error(
                StructuredWasmError::invalid_json(&error)
                    .with_direct_operation("makeEnvelopingWorm"),
            ),
        };
        Ok(result.into_ts()?)
    }

    /// Build the wheel a double-enveloping (Hindley) worm drives from a JSON
    /// specification, as typed data (see the module docs for the fields).
    ///
    /// Never throws on a refusal: the envelope's `status` is `"error"` with
    /// a `gear:<reason>` code and the session unchanged.
    #[wasm_bindgen(js_name = "makeEnvelopingWormWheelDetailed")]
    pub fn make_enveloping_worm_wheel_detailed(
        &mut self,
        spec: &str,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        let result = match serde_json::from_str::<Value>(spec) {
            Ok(value) => self.make_enveloping_worm_wheel_detailed_impl(&value),
            Err(error) => SolidOperationDetailedResult::error(
                StructuredWasmError::invalid_json(&error)
                    .with_direct_operation("makeEnvelopingWormWheel"),
            ),
        };
        Ok(result.into_ts()?)
    }

    /// Build the helical wheel that meshes with a worm from a JSON
    /// specification, as typed data (see the module docs for the fields).
    ///
    /// Never throws on a refusal: the envelope's `status` is `"error"` with
    /// a `gear:<reason>` code and the session unchanged.
    #[wasm_bindgen(js_name = "makeWormWheelDetailed")]
    pub fn make_worm_wheel_detailed(
        &mut self,
        spec: &str,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        let result = match serde_json::from_str::<Value>(spec) {
            Ok(value) => self.make_worm_wheel_detailed_impl(&value),
            Err(error) => SolidOperationDetailedResult::error(
                StructuredWasmError::invalid_json(&error).with_direct_operation("makeWormWheel"),
            ),
        };
        Ok(result.into_ts()?)
    }

    /// Cut gear teeth along the whole length of a solid's cylindrical face,
    /// as typed data: external teeth on a boss, internal teeth in a bore
    /// (see the module docs for the fields).
    ///
    /// The input solid is left in place and a new solid is returned. Never
    /// throws on a refusal: the envelope's `status` is `"error"` with a
    /// `gear:<reason>` code and the session unchanged.
    #[wasm_bindgen(js_name = "cutGearTeethDetailed")]
    pub fn cut_gear_teeth_detailed(
        &mut self,
        solid: u32,
        face: u32,
        spec: &str,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        let result = match serde_json::from_str::<Value>(spec) {
            Ok(value) => self.cut_gear_teeth_detailed_impl(solid, face, &value),
            Err(error) => SolidOperationDetailedResult::error(
                StructuredWasmError::invalid_json(&error).with_direct_operation("cutGearTeeth"),
            ),
        };
        Ok(result.into_ts()?)
    }
}

impl RemusKernel {
    pub(crate) fn cut_gear_teeth_detailed_impl(
        &mut self,
        solid: u32,
        face: u32,
        spec: &Value,
    ) -> SolidOperationDetailedResult {
        let result = (|| -> Result<(u32, Map<String, Value>), StructuredWasmError> {
            let spec = CylinderGearSpec::deserialize(spec).map_err(|error| {
                StructuredWasmError::invalid_argument(format!("invalid gear spec: {error}"), None)
            })?;
            let (params, options) = spec.split()?;
            let solid = self
                .resolve_solid(solid)
                .map_err(StructuredWasmError::from)?;
            let face = self.resolve_face(face).map_err(StructuredWasmError::from)?;
            let outcome = cut_gear_teeth(self.topo_mut(), solid, face, &params, &options)
                .map_err(gear_failure)?;
            Ok((
                solid_id_to_u32(outcome.solid),
                outcome_details(&outcome, params.kind),
            ))
        })();
        match result {
            Ok((solid, details)) => {
                SolidOperationDetailedResult::success_with_details(solid, details)
            }
            Err(error) => {
                SolidOperationDetailedResult::error(error.with_direct_operation("cutGearTeeth"))
            }
        }
    }

    pub(crate) fn make_bevel_gear_detailed_impl(
        &mut self,
        spec: &Value,
    ) -> SolidOperationDetailedResult {
        let result = (|| -> Result<(u32, Map<String, Value>), StructuredWasmError> {
            let spec = BevelGearSpec::deserialize(spec).map_err(|error| {
                StructuredWasmError::invalid_argument(
                    format!("invalid bevel gear spec: {error}"),
                    None,
                )
            })?;
            let (params, placement) = spec.split();
            let outcome =
                make_bevel_gear(self.topo_mut(), &params, &placement).map_err(|error| {
                    // Bevel refusals carry no spur geometry; attach the cone
                    // dimensions whenever the parameters admit them.
                    let dimensions = bevel_gear_geometry(&params).ok();
                    let mut structured = gear_failure(error);
                    if let Some(geometry) = dimensions {
                        structured = structured
                            .with_detail("geometry", bevel_geometry_json(&geometry))
                            .with_detail("warnings", warnings_json(&geometry.warnings));
                    }
                    structured
                })?;
            Ok((solid_id_to_u32(outcome.solid), bevel_details(&outcome)))
        })();
        match result {
            Ok((solid, details)) => {
                SolidOperationDetailedResult::success_with_details(solid, details)
            }
            Err(error) => {
                SolidOperationDetailedResult::error(error.with_direct_operation("makeBevelGear"))
            }
        }
    }

    pub(crate) fn make_worm_detailed_impl(&mut self, spec: &Value) -> SolidOperationDetailedResult {
        let result = (|| -> Result<(u32, Map<String, Value>), StructuredWasmError> {
            let spec = WormSpec::deserialize(spec).map_err(|error| {
                StructuredWasmError::invalid_argument(format!("invalid worm spec: {error}"), None)
            })?;
            let (params, placement) = spec.split();
            let outcome = make_worm(self.topo_mut(), &params, &placement).map_err(|error| {
                // Worm refusals carry no gear geometry; attach the worm's
                // dimensions whenever the parameters admit them.
                let dimensions = worm_geometry(&params).ok();
                let mut structured = gear_failure(error);
                if let Some(geometry) = dimensions {
                    structured = structured
                        .with_detail("geometry", worm_geometry_json(&geometry))
                        .with_detail("warnings", warnings_json(&geometry.warnings));
                }
                structured
            })?;
            Ok((solid_id_to_u32(outcome.solid), worm_details(&outcome)))
        })();
        match result {
            Ok((solid, details)) => {
                SolidOperationDetailedResult::success_with_details(solid, details)
            }
            Err(error) => {
                SolidOperationDetailedResult::error(error.with_direct_operation("makeWorm"))
            }
        }
    }

    pub(crate) fn make_enveloping_worm_detailed_impl(
        &mut self,
        spec: &Value,
    ) -> SolidOperationDetailedResult {
        let result = (|| -> Result<(u32, Map<String, Value>), StructuredWasmError> {
            let spec = EnvelopingWormSpec::deserialize(spec).map_err(|error| {
                StructuredWasmError::invalid_argument(
                    format!("invalid enveloping worm spec: {error}"),
                    None,
                )
            })?;
            let (params, placement) = spec.split();
            let outcome =
                make_enveloping_worm(self.topo_mut(), &params, &placement).map_err(|error| {
                    // Attach the worm's dimensions whenever the parameters
                    // admit them.
                    let dimensions = enveloping_worm_geometry(&params).ok();
                    let mut structured = gear_failure(error);
                    if let Some(geometry) = dimensions {
                        structured = structured
                            .with_detail("geometry", enveloping_geometry_json(&geometry))
                            .with_detail("warnings", warnings_json(&geometry.warnings));
                    }
                    structured
                })?;
            Ok((solid_id_to_u32(outcome.solid), enveloping_details(&outcome)))
        })();
        match result {
            Ok((solid, details)) => {
                SolidOperationDetailedResult::success_with_details(solid, details)
            }
            Err(error) => SolidOperationDetailedResult::error(
                error.with_direct_operation("makeEnvelopingWorm"),
            ),
        }
    }

    pub(crate) fn make_enveloping_worm_wheel_detailed_impl(
        &mut self,
        spec: &Value,
    ) -> SolidOperationDetailedResult {
        let result = (|| -> Result<(u32, Map<String, Value>), StructuredWasmError> {
            let spec = EnvelopingWheelSpec::deserialize(spec).map_err(|error| {
                StructuredWasmError::invalid_argument(
                    format!("invalid enveloping worm wheel spec: {error}"),
                    None,
                )
            })?;
            let (worm, options, placement) = spec.split();
            let outcome = make_enveloping_worm_wheel(self.topo_mut(), &worm, &options, &placement)
                .map_err(|error| {
                    // Attach the worm's dimensions whenever its parameters
                    // admit them.
                    let dimensions = enveloping_worm_geometry(&worm).ok();
                    let mut structured = gear_failure(error);
                    if let Some(geometry) = dimensions {
                        structured = structured
                            .with_detail("wormGeometry", enveloping_geometry_json(&geometry));
                    }
                    structured
                })?;
            Ok((
                solid_id_to_u32(outcome.solid),
                enveloping_wheel_details(&outcome),
            ))
        })();
        match result {
            Ok((solid, details)) => {
                SolidOperationDetailedResult::success_with_details(solid, details)
            }
            Err(error) => SolidOperationDetailedResult::error(
                error.with_direct_operation("makeEnvelopingWormWheel"),
            ),
        }
    }

    pub(crate) fn make_worm_wheel_detailed_impl(
        &mut self,
        spec: &Value,
    ) -> SolidOperationDetailedResult {
        let result = (|| -> Result<(u32, Map<String, Value>), StructuredWasmError> {
            let spec = WormWheelSpec::deserialize(spec).map_err(|error| {
                StructuredWasmError::invalid_argument(
                    format!("invalid worm wheel spec: {error}"),
                    None,
                )
            })?;
            let (throated, outside_diameter) = (spec.throated, spec.outside_diameter);
            let (worm, wheel, placement) = spec.split();
            if throated {
                let options = ThroatedWheelOptions {
                    outside_diameter,
                    backlash: wheel.backlash,
                    tolerance: wheel.tolerance,
                    bore_diameter: wheel.bore_diameter,
                };
                let outcome = make_throated_worm_wheel(
                    self.topo_mut(),
                    &worm,
                    wheel.teeth,
                    wheel.face_width,
                    &options,
                    &placement,
                )
                .map_err(gear_failure)?;
                let mut details = outcome_details(&outcome.gear, GearKind::External);
                details.insert(
                    "throatCurveBound".into(),
                    Value::from(outcome.throat_curve_bound),
                );
                if let Some(Value::Object(geometry)) = details.get_mut("geometry") {
                    for (key, value) in [
                        ("centerDistance", outcome.center_distance),
                        ("throatDiameter", outcome.throat_diameter),
                        ("outsideDiameter", outcome.outside_diameter),
                        ("throatRadius", outcome.throat_radius),
                        ("throatWidth", outcome.throat_width),
                    ] {
                        geometry.insert(key.into(), Value::from(value));
                    }
                }
                return Ok((solid_id_to_u32(outcome.gear.solid), details));
            }
            if outside_diameter.is_some() {
                return Err(StructuredWasmError::invalid_argument(
                    "only a throated worm wheel takes an outside diameter",
                    Some("outsideDiameter"),
                ));
            }
            let matched = worm_wheel(&worm, wheel.teeth, wheel.face_width).map_err(gear_failure)?;
            let params = GearParams {
                backlash: wheel.backlash,
                tolerance: wheel.tolerance,
                bore_diameter: wheel.bore_diameter,
                ..matched.params
            };
            let center_distance = Value::from(matched.center_distance);
            let outcome = make_gear(self.topo_mut(), &params, &placement).map_err(gear_failure)?;
            let mut details = outcome_details(&outcome, params.kind);
            if let Some(Value::Object(geometry)) = details.get_mut("geometry") {
                geometry.insert("centerDistance".into(), center_distance);
            }
            Ok((solid_id_to_u32(outcome.solid), details))
        })();
        match result {
            Ok((solid, details)) => {
                SolidOperationDetailedResult::success_with_details(solid, details)
            }
            Err(error) => {
                SolidOperationDetailedResult::error(error.with_direct_operation("makeWormWheel"))
            }
        }
    }

    pub(crate) fn make_gear_detailed_impl(&mut self, spec: &Value) -> SolidOperationDetailedResult {
        let result = (|| -> Result<(u32, Map<String, Value>), StructuredWasmError> {
            let spec = GearSpec::deserialize(spec).map_err(|error| {
                StructuredWasmError::invalid_argument(format!("invalid gear spec: {error}"), None)
            })?;
            let (params, placement) = spec.split();
            let outcome = make_gear(self.topo_mut(), &params, &placement).map_err(gear_failure)?;
            Ok((
                solid_id_to_u32(outcome.solid),
                outcome_details(&outcome, params.kind),
            ))
        })();
        match result {
            Ok((solid, details)) => {
                SolidOperationDetailedResult::success_with_details(solid, details)
            }
            Err(error) => {
                SolidOperationDetailedResult::error(error.with_direct_operation("makeGear"))
            }
        }
    }
}

#[cfg(test)]
mod tests;
