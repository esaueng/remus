//! Read-only global maximum-distance measurement (core tier).

use remus_operations::distance::MaximumDistanceError;
use wasm_bindgen::prelude::*;

use crate::error::StructuredWasmError;
use crate::kernel::BrepKernel;

fn failure(error: MaximumDistanceError) -> StructuredWasmError {
    let code = error.kernel_code();
    let message = format!("{code}: {error}");
    let structured = match error {
        MaximumDistanceError::Topology(error) => {
            StructuredWasmError::from(error).with_message(message)
        }
        MaximumDistanceError::InvalidGeometry(_) => {
            StructuredWasmError::invalid_argument(message, None)
        }
        MaximumDistanceError::Unsupported(_) => StructuredWasmError::unsupported(message),
        MaximumDistanceError::Unresolved(_) => {
            StructuredWasmError::exact_only_unattainable(message)
        }
        MaximumDistanceError::ResourceLimit {
            resource,
            required,
            limit,
        } => StructuredWasmError::resource_limit(message, resource, limit, required),
        MaximumDistanceError::Cancelled => {
            StructuredWasmError::from(remus_math::MathError::Cancelled).with_message(message)
        }
    };
    structured.with_detail("kernelCode", code)
}

impl BrepKernel {
    pub(crate) fn maximum_distance_values(
        &self,
        a: u32,
        b: u32,
    ) -> Result<Vec<f64>, StructuredWasmError> {
        let a = self.resolve_solid(a).map_err(StructuredWasmError::from)?;
        let b = self.resolve_solid(b).map_err(StructuredWasmError::from)?;
        let result = remus_operations::distance::solid_to_solid_maximum_distance(&self.topo, a, b)
            .map_err(failure)?;
        Ok(vec![
            result.distance,
            result.point_a.x(),
            result.point_a.y(),
            result.point_a.z(),
            result.point_b.x(),
            result.point_b.y(),
            result.point_b.z(),
        ])
    }
}

#[wasm_bindgen]
impl BrepKernel {
    /// Global maximum distance and witnesses as `[distance, ax, ay, az, bx, by, bz]`.
    ///
    /// Supports straight-edged planar polyhedra and complete native spheres,
    /// including certified affine NURBS planar faces and mixed pairs.
    /// All cavity boundaries are qualified; unsupported
    /// geometry returns a typed refusal. This query changes no session state.
    /// Inputs require valid BReps with closed shells and simple, nested trim loops.
    ///
    /// # Errors
    /// Returns an error for invalid handles/boundaries, unsupported geometry,
    /// unresolved arithmetic or exhausted bounded work.
    #[wasm_bindgen(js_name = "solidToSolidMaximumDistance")]
    pub fn solid_to_solid_maximum_distance(&self, a: u32, b: u32) -> Result<Vec<f64>, JsError> {
        self.maximum_distance_values(a, b).map_err(|error| {
            let message = error.message().to_owned();
            let (prefix, _, _) = error.into_direct_parts();
            if message.starts_with(&format!("{prefix}:")) {
                JsError::new(&message)
            } else {
                JsError::new(&format!("{prefix}: {message}"))
            }
        })
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use crate::kernel::BrepKernel;
    use serde_json::{Value, json};

    fn call(kernel: &mut BrepKernel, v2: bool, op: &str, args: Value) -> Value {
        let input = json!([{ "op":op,"args":args }]).to_string();
        let output = if v2 {
            kernel.execute_batch_v2(&input)
        } else {
            kernel.execute_batch(&input)
        };
        serde_json::from_str::<Value>(&output).unwrap()[0].clone()
    }

    #[test]
    #[allow(deprecated)] // Deliberate mismatch of compatibility and authoritative boundaries.
    fn maximum_distance_topology_refusals_keep_query_code_and_topology_details() {
        for v2 in [false, true] {
            let mut kernel = BrepKernel::new();
            let solid = u32::try_from(
                call(
                    &mut kernel,
                    v2,
                    "makeBox",
                    json!({"width":2,"height":3,"depth":4}),
                )["ok"]
                    .as_u64()
                    .unwrap(),
            )
            .unwrap();
            let id = kernel.resolve_solid(solid).unwrap();
            let faces = remus_topology::explorer::solid_faces(&kernel.topo, id).unwrap();
            let other = kernel.topo.face(faces[1]).unwrap().outer_wire();
            kernel
                .topo_mut()
                .face_mut(faces[0])
                .unwrap()
                .set_outer_wire(other);
            kernel.checkpoint_impl().unwrap();
            let owner = std::rc::Rc::clone(&kernel.topo);
            let ticks = kernel.topo.mutation_ticks();
            let direct = kernel.maximum_distance_values(solid, solid).unwrap_err();
            let (code, category, details) = direct.into_direct_parts();
            assert_eq!(code, "maximum-distance:invalid-topology");
            assert_eq!(category, "invalid_topology");
            assert!(
                details["message"]
                    .as_str()
                    .unwrap()
                    .starts_with("maximum-distance:invalid-topology:")
            );
            let result = call(
                &mut kernel,
                v2,
                "solidToSolidMaximumDistance",
                json!({"solidA":solid,"solidB":solid}),
            );
            if v2 {
                assert_eq!(result["error"]["category"], "invalid_topology");
                assert_eq!(
                    result["error"]["details"]["kernelCode"],
                    "maximum-distance:invalid-topology"
                );
                assert_eq!(result["error"]["details"]["entity"], "face");
                assert_eq!(result["error"]["details"]["index"], faces[0].index());
            } else {
                assert!(
                    result["error"]
                        .as_str()
                        .unwrap()
                        .starts_with("maximum-distance:invalid-topology:"),
                    "{result}"
                );
            }
            assert!(std::rc::Rc::ptr_eq(&owner, &kernel.topo));
            assert_eq!(kernel.topo.mutation_ticks(), ticks);
        }
    }

    #[test]
    fn maximum_distance_batch_oracles_and_read_only_session_contract() {
        for v2 in [false, true] {
            let mut kernel = BrepKernel::new();
            let a = call(
                &mut kernel,
                v2,
                "makeBox",
                json!({"width":2,"height":3,"depth":4}),
            )["ok"]
                .as_u64()
                .unwrap();
            let b = call(
                &mut kernel,
                v2,
                "makeBox",
                json!({"width":5,"height":2,"depth":1}),
            )["ok"]
                .as_u64()
                .unwrap();
            let before = (
                kernel.topo.mutation_ticks(),
                kernel.topo.journal().len(),
                kernel.topo.num_solids(),
                kernel.topo.num_vertices(),
            );
            kernel.checkpoint_impl().unwrap();
            let owner = std::rc::Rc::clone(&kernel.topo);
            let row = call(
                &mut kernel,
                v2,
                "solidToSolidMaximumDistance",
                json!({"solidA":a,"solidB":b}),
            );
            let values = row["ok"].as_array().unwrap();
            // Overlapping origin boxes: per-axis farthest separations5,3,4.
            assert!((values[0].as_f64().unwrap() - 50.0_f64.sqrt()).abs() < 1e-12);
            assert_eq!(values.len(), 7);
            assert_eq!(
                before,
                (
                    kernel.topo.mutation_ticks(),
                    kernel.topo.journal().len(),
                    kernel.topo.num_solids(),
                    kernel.topo.num_vertices()
                )
            );
            assert!(std::rc::Rc::ptr_eq(&owner, &kernel.topo));
            let cylinder = call(
                &mut kernel,
                v2,
                "makeCylinder",
                json!({"radius":1,"height":2,"segments":16}),
            )["ok"]
                .as_u64()
                .unwrap();
            let tick = kernel.topo.mutation_ticks();
            let failed_owner = std::rc::Rc::clone(&kernel.topo);
            let refusal = call(
                &mut kernel,
                v2,
                "solidToSolidMaximumDistance",
                json!({"solidA":a,"solidB":cylinder}),
            );
            if v2 {
                assert_eq!(refusal["error"]["category"], "unsupported");
                assert_eq!(
                    refusal["error"]["details"]["kernelCode"],
                    "maximum-distance:unsupported"
                );
            } else {
                assert!(
                    refusal["error"]
                        .as_str()
                        .unwrap()
                        .starts_with("maximum-distance:unsupported:")
                );
            }
            assert_eq!(kernel.topo.mutation_ticks(), tick);
            assert!(std::rc::Rc::ptr_eq(&failed_owner, &kernel.topo));
            let invalid = call(
                &mut kernel,
                v2,
                "solidToSolidMaximumDistance",
                json!({"solidA":a,"solidB":4_294_967_294_u32}),
            );
            assert!(invalid.get("error").is_some());
            if v2 {
                assert_eq!(invalid["error"]["category"], "invalid_input");
            }
        }
    }
}
