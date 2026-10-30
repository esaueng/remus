//! Involute spur, internal and rack gear construction
//! (`remus_operations::gear`).
//!
//! `makeGearDetailed(spec)` builds a new gear body and
//! `cutGearTeethDetailed(solid, face, spec)` cuts teeth along the whole
//! length of a solid's cylindrical face (external teeth on a boss, internal
//! teeth in a bore). Both return the typed [`SolidOperationDetailedResult`]
//! envelope; the `executeBatch` / `executeBatchV2` ops of the same names take
//! the spec as their `args` (`makeGearDetailed`) or as `args.spec` beside
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
//!   "hand": "right" | "left"      // default "right"
//! }
//! ```
//!
//! A non-zero `helixAngle` makes a helical gear (ISO 21771): `module`,
//! `pressureAngle` and the coefficients are normal-section values, and the
//! teeth turn about `axis` as they advance (counter-clockwise for a right
//! hand). `phase` places the first tooth on the start cap.
//!
//! `cutGearTeethDetailed` spec: `kind` (`"external"` for a boss,
//! `"internal"` for a bore), `teeth`, `faceWidth` (the face's length:
//! partial spans refuse), `fit` (`"keepDiameter"`, the default, solves the
//! module so the tip (inner tip) diameter is the cylinder's; `"module"` uses
//! `module`), the same tooth-form fields as above (including `helixAngle`
//! and `hand`), and optional `xDirection` and `phase` (default: the face's
//! seam direction).
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
    CylinderGearOptions, GearError, GearFit, GearGeometry, GearKind, GearOutcome, GearParams,
    GearPlacement, GearRefusal, GearWarning, Hand, cut_gear_teeth, make_gear,
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
