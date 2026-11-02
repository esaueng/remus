//! Screw threads on an existing cylindrical face
//! (`remus_operations::thread`).
//!
//! `cutThreadDetailed(solid, face, spec)` cuts a thread along a solid's
//! cylindrical face: external on a boss, internal in a bore (the kind comes
//! from the face). It returns the typed [`SolidOperationDetailedResult`]
//! envelope; the `executeBatch` / `executeBatchV2` op of the same name takes
//! the spec as `args.spec` beside `args.solid` and `args.face`, so direct and
//! batch results are identical by construction.
//!
//! Spec (lengths in model units, angles in degrees):
//!
//! ```json
//! {
//!   "pitch": 1,                       // axial distance between threads
//!   "depth": 0.541,                   // crest to root, radial
//!   "crestWidth": 0.125,              // axial crest flat, 0 for sharp
//!   "upperFlankAngle": 30,            // flank facing the thread's end,
//!   "lowerFlankAngle": 30,            //   from the plane across the axis
//!   "starts": 1, "hand": "right" | "left",
//!   "majorDiameter": 6,               // optional; default fits the face
//!   "offset": 0,                      // from the face's lower end
//!   "length": 8,                      // optional; default to the face end
//!   "xDirection": [1, 0, 0], "phase": 0,
//!   "tolerance": 1e-4                 // default 1e-4 · pitch
//! }
//! ```
//!
//! The root flat is what the crest and flanks leave of the pitch. An
//! external thread's crest is its major diameter, an internal thread's its
//! minor diameter; `majorDiameter` is the outer surface either way
//! (external crest, internal root). Without it an external crest sits on the
//! boss and an internal crest on the bore.
//!
//! Details on success: `quality` (`"approximate"`, `approximated`
//! `"helical thread surfaces"`), `deviationBound`, `spansAlongAxis`,
//! `spansPerFlank` and
//! `geometry` (`kind`, `majorDiameter`, `minorDiameter`, `pitch`, `lead`,
//! `crestWidth`, `rootWidth`, `leadAngle` in degrees, `length`, `turns`).
//! Refusals carry a `thread:<reason>` code (see
//! [`ThreadRefusal::code`]) and leave the session unchanged.

use serde::Deserialize;
use serde_json::{Map, Value, json};
use tsify::Tsify as _;
use wasm_bindgen::prelude::*;

use remus_math::vec::Vec3;
use remus_operations::thread::{
    CylinderThreadOptions, ThreadError, ThreadForm, ThreadGeometry, ThreadHand, ThreadKind,
    ThreadOutcome, ThreadRefusal, cut_thread,
};

use crate::error::StructuredWasmError;
use crate::handles::solid_id_to_u32;
use crate::kernel::RemusKernel;
use crate::types::SolidOperationDetailedResult;

#[derive(Debug, Default, Clone, Copy, Deserialize)]
#[serde(rename_all = "lowercase")]
enum HandSpec {
    #[default]
    Right,
    Left,
}

const fn default_starts() -> u32 {
    1
}

/// The JSON specification (see the module docs).
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
struct ThreadSpec {
    pitch: f64,
    depth: f64,
    crest_width: f64,
    upper_flank_angle: f64,
    lower_flank_angle: f64,
    #[serde(default = "default_starts")]
    starts: u32,
    #[serde(default)]
    hand: HandSpec,
    major_diameter: Option<f64>,
    #[serde(default)]
    offset: f64,
    length: Option<f64>,
    x_direction: Option<[f64; 3]>,
    #[serde(default)]
    phase: f64,
    tolerance: Option<f64>,
}

impl ThreadSpec {
    fn split(self) -> (ThreadForm, CylinderThreadOptions) {
        let form = ThreadForm {
            pitch: self.pitch,
            depth: self.depth,
            crest_width: self.crest_width,
            upper_flank_angle: self.upper_flank_angle.to_radians(),
            lower_flank_angle: self.lower_flank_angle.to_radians(),
            starts: self.starts,
            hand: match self.hand {
                HandSpec::Right => ThreadHand::Right,
                HandSpec::Left => ThreadHand::Left,
            },
        };
        let options = CylinderThreadOptions {
            major_diameter: self.major_diameter,
            offset: self.offset,
            length: self.length,
            x_direction: self.x_direction.map(|v| Vec3::new(v[0], v[1], v[2])),
            phase: self.phase.to_radians(),
            tolerance: self.tolerance,
        };
        (form, options)
    }
}

fn geometry_json(geometry: &ThreadGeometry) -> Value {
    json!({
        "kind": match geometry.kind {
            ThreadKind::External => "external",
            ThreadKind::Internal => "internal",
        },
        "majorDiameter": geometry.major_diameter,
        "minorDiameter": geometry.minor_diameter,
        "pitch": geometry.pitch,
        "lead": geometry.lead,
        "crestWidth": geometry.crest_width,
        "rootWidth": geometry.root_width,
        "leadAngle": geometry.lead_angle.to_degrees(),
        "length": geometry.length,
        "turns": geometry.turns,
    })
}

fn outcome_details(outcome: &ThreadOutcome) -> Map<String, Value> {
    let mut details = Map::new();
    details.insert("quality".into(), Value::from("approximate"));
    details.insert(
        "approximated".into(),
        Value::from("helical thread surfaces"),
    );
    details.insert(
        "deviationBound".into(),
        Value::from(outcome.deviation_bound),
    );
    details.insert(
        "spansAlongAxis".into(),
        Value::from(outcome.spans_along_axis),
    );
    details.insert("spansPerFlank".into(), Value::from(outcome.spans_per_flank));
    details.insert("geometry".into(), geometry_json(&outcome.geometry));
    details
}

fn thread_failure(error: ThreadError) -> StructuredWasmError {
    match error {
        ThreadError::Refused { code, reason } => {
            let argument = match code {
                ThreadRefusal::SpanOutsideFace => Some("length"),
                ThreadRefusal::CylinderTooSmall | ThreadRefusal::BoreTooLarge => {
                    Some("majorDiameter")
                }
                ThreadRefusal::RootClosed => Some("crestWidth"),
                ThreadRefusal::TooDeep => Some("depth"),
                ThreadRefusal::ToleranceTooTight => Some("tolerance"),
                _ => None,
            };
            StructuredWasmError::invalid_argument(
                format!("thread:{}: {reason}", code.code()),
                argument,
            )
            .with_detail("kernelCode", format!("thread:{}", code.code()))
        }
        ThreadError::Operations(error) => StructuredWasmError::from(error),
    }
}

#[wasm_bindgen]
impl RemusKernel {
    /// Cut a screw thread along a solid's cylindrical face, as typed data:
    /// external on a boss, internal in a bore (see the module docs for the
    /// fields).
    ///
    /// The input solid is left in place and a new solid is returned. Never
    /// throws on a refusal: the envelope's `status` is `"error"` with a
    /// `thread:<reason>` code and the session unchanged.
    ///
    /// # Errors
    ///
    /// Only when the envelope cannot be converted to a JavaScript value.
    #[wasm_bindgen(js_name = "cutThreadDetailed")]
    pub fn cut_thread_detailed(
        &mut self,
        solid: u32,
        face: u32,
        spec: &str,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        let result = match serde_json::from_str::<Value>(spec) {
            Ok(value) => self.cut_thread_detailed_impl(solid, face, &value),
            Err(error) => SolidOperationDetailedResult::error(
                StructuredWasmError::invalid_json(&error).with_direct_operation("cutThread"),
            ),
        };
        Ok(result.into_ts()?)
    }
}

impl RemusKernel {
    pub(crate) fn cut_thread_detailed_impl(
        &mut self,
        solid: u32,
        face: u32,
        spec: &Value,
    ) -> SolidOperationDetailedResult {
        let result = (|| -> Result<(u32, Map<String, Value>), StructuredWasmError> {
            let spec = ThreadSpec::deserialize(spec).map_err(|error| {
                StructuredWasmError::invalid_argument(format!("invalid thread spec: {error}"), None)
            })?;
            let (form, options) = spec.split();
            let solid = self
                .resolve_solid(solid)
                .map_err(StructuredWasmError::from)?;
            let face = self.resolve_face(face).map_err(StructuredWasmError::from)?;
            let outcome = cut_thread(self.topo_mut(), solid, face, &form, &options)
                .map_err(thread_failure)?;
            Ok((solid_id_to_u32(outcome.solid), outcome_details(&outcome)))
        })();
        match result {
            Ok((solid, details)) => {
                SolidOperationDetailedResult::success_with_details(solid, details)
            }
            Err(error) => {
                SolidOperationDetailedResult::error(error.with_direct_operation("cutThread"))
            }
        }
    }
}

#[cfg(test)]
mod tests;
