//! Persistent topological reference resolution (RFC 0003 Stage 2).

use wasm_bindgen::prelude::*;

use crate::kernel::BrepKernel;
use crate::types::{PersistentRefResolutionV1, PersistentRefV1};

#[wasm_bindgen]
impl BrepKernel {
    /// Resolve a version-1 persistent reference over this kernel's evolution
    /// journal.
    ///
    /// Stage 2 accepts `operationOutput` and `lineageOf` anchors with
    /// surface/curve-type discriminators. The tagged result is always returned as data:
    /// dangling, ambiguous, and journal-gap outcomes are not thrown JS errors.
    /// Malformed or unsupported reference schemas are rejected.
    ///
    /// # Errors
    ///
    /// Returns an error if the value object has an unsupported schema version,
    /// a non-canonical operation id, mismatched lineage kinds, or a result
    /// handle outside the WASM `u32` range.
    #[wasm_bindgen(js_name = "resolvePersistentRef")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn resolve_persistent_ref(
        &self,
        reference: PersistentRefV1,
    ) -> Result<PersistentRefResolutionV1, JsError> {
        let reference = reference
            .into_native()
            .map_err(|error| JsError::new(&error))?;
        let outcome = brepkit_topology::naming::resolve(self.topo(), &reference);
        PersistentRefResolutionV1::from_native(&outcome).map_err(|error| JsError::new(&error))
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use brepkit_topology::journal::{EntityKey, EventDraft, EvolutionDraft, OpId};
    use brepkit_topology::naming::Resolution;

    use crate::types::{
        PersistentRefAnchorV1, PersistentRefDiscriminatorV1, PersistentRefEntityKindV1,
        PersistentRefResolutionV1, PersistentRefV1,
    };

    use super::*;

    fn output(operation: u64, output: u32) -> PersistentRefV1 {
        PersistentRefV1 {
            schema_version: 1,
            anchor: PersistentRefAnchorV1::OperationOutput {
                operation: operation.to_string(),
                output,
            },
            discriminators: Vec::new(),
            entity_kind: PersistentRefEntityKindV1::Face,
        }
    }

    fn deterministic_split() -> PersistentRefResolutionV1 {
        let mut kernel = BrepKernel::new();
        let pending = kernel.topo_mut().journal_begin("seed");
        let mut seed = EvolutionDraft::construction();
        seed.push(
            EntityKey::face(10),
            EventDraft::Modified {
                from: EntityKey::face(1),
            },
        );
        let op = kernel
            .topo_mut()
            .journal_record_evolution(pending, seed)
            .unwrap();

        let pending = kernel.topo_mut().journal_begin("split");
        let mut split = EvolutionDraft::construction();
        for result in [30, 31] {
            split.push(
                EntityKey::face(result),
                EventDraft::Modified {
                    from: EntityKey::face(10),
                },
            );
        }
        kernel
            .topo_mut()
            .journal_record_evolution(pending, split)
            .unwrap();

        kernel
            .resolve_persistent_ref(PersistentRefV1 {
                schema_version: 1,
                anchor: PersistentRefAnchorV1::LineageOf {
                    base: Box::new(output(op.value(), 0)),
                },
                discriminators: Vec::new(),
                entity_kind: PersistentRefEntityKindV1::Face,
            })
            .unwrap()
    }

    #[test]
    fn wasm_outcome_is_tagged_and_deterministic() {
        let first = deterministic_split();
        let second = deterministic_split();
        assert_eq!(first, second);
        let value = serde_json::to_value(first).unwrap();
        assert_eq!(value["status"], "boundMany");
        assert_eq!(value["provenance"], "construction");
        assert_eq!(value["entities"][0]["handle"], 30);
        assert_eq!(value["entities"][1]["handle"], 31);
    }

    #[test]
    fn wasm_failure_carries_stable_diagnostic() {
        let kernel = BrepKernel::new();
        let outcome = kernel.resolve_persistent_ref(output(42, 0)).unwrap();
        let value = serde_json::to_value(outcome).unwrap();
        assert_eq!(value["status"], "unknownOperation");
        assert_eq!(value["operation"], "42");
        assert_eq!(value["diagnostic"]["code"], "ref_unknown_operation");
        assert_eq!(value["diagnostic"]["category"], "invalid_input");
    }

    #[test]
    fn every_native_failure_has_a_typed_wasm_diagnostic() {
        let cases = [
            (
                Resolution::Ambiguous {
                    candidates: vec![EntityKey::face(2), EntityKey::face(1)],
                    reason: "same signature".to_owned(),
                },
                "ambiguous",
                "ref_ambiguous",
                "invalid_input",
            ),
            (
                Resolution::Dangling {
                    deleted_at: OpId::from_value(8),
                },
                "dangling",
                "ref_dangling",
                "invalid_input",
            ),
            (
                Resolution::UnresolvedAcrossOperation {
                    op: OpId::from_value(9),
                    kind: "offset".to_owned(),
                },
                "unresolvedAcrossOperation",
                "ref_unresolved_across_operation",
                "unsupported",
            ),
            (
                Resolution::UnknownOperation {
                    op: OpId::from_value(10),
                },
                "unknownOperation",
                "ref_unknown_operation",
                "invalid_input",
            ),
            (
                Resolution::NoMatch {
                    reason: "surface_type:plane".to_owned(),
                },
                "noMatch",
                "ref_no_match",
                "invalid_input",
            ),
        ];

        for (outcome, status, code, category) in cases {
            let typed = PersistentRefResolutionV1::from_native(&outcome).unwrap();
            let value = serde_json::to_value(typed).unwrap();
            assert_eq!(value["status"], status);
            assert_eq!(value["diagnostic"]["code"], code);
            assert_eq!(value["diagnostic"]["category"], category);
        }
    }

    #[test]
    fn wasm_reference_validation_fails_before_resolution() {
        let mut typed_discriminator = output(0, 0);
        typed_discriminator.discriminators = vec![PersistentRefDiscriminatorV1::SurfaceType {
            tag: "plane".to_owned(),
        }];
        let native = typed_discriminator.into_native().unwrap();
        assert_eq!(
            native.discriminators,
            vec![brepkit_topology::naming::Discriminator::SurfaceType(
                "plane".to_owned()
            )]
        );

        let mut wrong_version = output(0, 0);
        wrong_version.schema_version = 2;
        assert!(wrong_version.into_native().is_err());

        let noncanonical = PersistentRefV1 {
            schema_version: 1,
            anchor: PersistentRefAnchorV1::OperationOutput {
                operation: "01".to_owned(),
                output: 0,
            },
            discriminators: Vec::new(),
            entity_kind: PersistentRefEntityKindV1::Face,
        };
        assert!(noncanonical.into_native().is_err());

        let mismatched_lineage = PersistentRefV1 {
            schema_version: 1,
            anchor: PersistentRefAnchorV1::LineageOf {
                base: Box::new(output(0, 0)),
            },
            discriminators: Vec::new(),
            entity_kind: PersistentRefEntityKindV1::Edge,
        };
        assert!(mismatched_lineage.into_native().is_err());

        let mut empty_tag = output(0, 0);
        empty_tag.discriminators =
            vec![PersistentRefDiscriminatorV1::SurfaceType { tag: String::new() }];
        assert!(empty_tag.into_native().is_err());

        let mut too_many_discriminators = output(0, 0);
        too_many_discriminators.discriminators = (0..65)
            .map(|_| PersistentRefDiscriminatorV1::CurveType {
                tag: "line".to_owned(),
            })
            .collect();
        assert!(too_many_discriminators.into_native().is_err());
    }
}
