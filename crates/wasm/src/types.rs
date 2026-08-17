//! Typed result structs for structured WASM returns.
//!
//! Types annotated with `Tsify` automatically generate TypeScript definitions
//! and can be serialized via `serde-wasm-bindgen` for zero-copy JS interop.

use std::collections::HashSet;

use remus_operations::evolution::{EvolutionMap, EvolutionOrigin};
use tsify::Tsify;

/// Typed result for `tessellateSolidGrouped`.
#[derive(serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
#[tsify(into_wasm_abi)]
pub struct GroupedMeshResult {
    pub positions: Vec<f64>,
    pub normals: Vec<f64>,
    pub indices: Vec<u32>,
    pub face_offsets: Vec<u32>,
}

/// Typed result for `tessellateSolidUV`.
#[derive(serde::Serialize, Tsify)]
#[tsify(into_wasm_abi)]
pub struct UvMeshResult {
    pub positions: Vec<f64>,
    pub normals: Vec<f64>,
    pub indices: Vec<u32>,
    pub uvs: Vec<f64>,
}

/// Typed result for `boundingBox`.
#[derive(serde::Serialize, Tsify)]
#[tsify(into_wasm_abi)]
pub struct BoundingBoxResult {
    pub min_x: f64,
    pub min_y: f64,
    pub min_z: f64,
    pub max_x: f64,
    pub max_y: f64,
    pub max_z: f64,
}

/// Current version of the WASM face-evolution payload.
pub const FACE_EVOLUTION_SCHEMA_VERSION: u32 = 1;

/// A solid and the complete set of face handles relevant to one side of an
/// evolution operation.
#[derive(Debug, Clone, PartialEq, Eq, serde::Deserialize, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
#[tsify(into_wasm_abi)]
pub struct EvolutionShapeV1 {
    pub solid: u32,
    pub faces: Vec<u32>,
}

/// One source face and the final-result faces related to it.
#[derive(Debug, Clone, PartialEq, Eq, serde::Deserialize, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
#[tsify(into_wasm_abi)]
pub struct EvolutionRelationV1 {
    pub source: u32,
    pub results: Vec<u32>,
}

/// A final-result face whose source could not be established.
#[derive(Debug, Clone, PartialEq, Eq, serde::Deserialize, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
#[tsify(into_wasm_abi)]
pub struct UnresolvedEvolutionResultV1 {
    pub result: u32,
    pub candidates: Vec<u32>,
}

/// Whether the payload contains construction history or an explicit refusal.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Deserialize, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub enum EvolutionProvenanceV1 {
    Construction,
    Unavailable,
}

/// Version 1 face-evolution claims.
#[derive(Debug, Clone, PartialEq, Eq, serde::Deserialize, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
#[tsify(into_wasm_abi)]
pub struct FaceEvolutionClaimsV1 {
    pub provenance: EvolutionProvenanceV1,
    pub modified: Vec<EvolutionRelationV1>,
    pub generated: Vec<EvolutionRelationV1>,
    pub deleted: Vec<u32>,
    pub unresolved_results: Vec<UnresolvedEvolutionResultV1>,
    pub unresolved_sources: Vec<u32>,
}

/// Stable, versioned WASM contract returned by fillet/chamfer evolution APIs.
///
/// `source.faces` and `result.faces` are the complete handle domains. A valid
/// payload accounts for every source as modified, deleted, or unresolved and
/// every result as modified, generated, or unresolved. The decoder rejects
/// handles outside those domains, duplicate pairs, overlaps between claim
/// kinds, and incomplete coverage.
#[derive(Debug, Clone, PartialEq, Eq, serde::Deserialize, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
#[tsify(into_wasm_abi)]
pub struct FaceEvolutionPayloadV1 {
    /// Contract version; currently always `1`.
    pub schema_version: u32,
    /// Input solid and its complete source-face handle set.
    pub source: EvolutionShapeV1,
    /// Final solid and its complete final-face handle set.
    pub result: EvolutionShapeV1,
    /// Validated evolution claims between the two handle domains.
    pub evolution: FaceEvolutionClaimsV1,
}

impl FaceEvolutionPayloadV1 {
    /// Build and validate a payload from a kernel evolution map.
    pub(crate) fn from_map(
        source_solid: u32,
        result_solid: u32,
        source_faces: Vec<u32>,
        result_faces: Vec<u32>,
        map: &EvolutionMap,
    ) -> Result<Self, String> {
        let mut source_faces = sorted_unique(source_faces);
        let mut result_faces = sorted_unique(result_faces);

        let evolution = if map.origin == EvolutionOrigin::Construction {
            let modified = relations(&map.modified)?;
            let generated = relations(&map.generated)?;
            let deleted = sorted_unique(
                map.deleted
                    .iter()
                    .copied()
                    .map(index_to_u32)
                    .collect::<Result<Vec<_>, _>>()?,
            );
            let mut unresolved_results: Vec<UnresolvedEvolutionResultV1> = map
                .unresolved
                .iter()
                .map(|(&result, candidates)| {
                    Ok(UnresolvedEvolutionResultV1 {
                        result: index_to_u32(result)?,
                        candidates: sorted_unique(
                            candidates
                                .iter()
                                .copied()
                                .map(index_to_u32)
                                .collect::<Result<Vec<_>, _>>()?,
                        ),
                    })
                })
                .collect::<Result<_, String>>()?;

            let claimed_results: HashSet<u32> = modified
                .iter()
                .chain(&generated)
                .flat_map(|claim| claim.results.iter().copied())
                .chain(unresolved_results.iter().map(|claim| claim.result))
                .collect();
            for &result in &result_faces {
                if !claimed_results.contains(&result) {
                    unresolved_results.push(UnresolvedEvolutionResultV1 {
                        result,
                        candidates: Vec::new(),
                    });
                }
            }
            unresolved_results.sort_by_key(|claim| claim.result);

            let accounted_sources: HashSet<u32> = modified
                .iter()
                .map(|claim| claim.source)
                .chain(deleted.iter().copied())
                .collect();
            let unresolved_sources = source_faces
                .iter()
                .copied()
                .filter(|source| !accounted_sources.contains(source))
                .collect();

            FaceEvolutionClaimsV1 {
                provenance: EvolutionProvenanceV1::Construction,
                modified,
                generated,
                deleted,
                unresolved_results,
                unresolved_sources,
            }
        } else {
            // A geometric match is deliberately not promoted into the stable
            // contract. Proximity, traversal order and approximate surface
            // matching are not construction history, so expose uncertainty
            // explicitly while preserving the successful solid.
            FaceEvolutionClaimsV1 {
                provenance: EvolutionProvenanceV1::Unavailable,
                modified: Vec::new(),
                generated: Vec::new(),
                deleted: Vec::new(),
                unresolved_results: result_faces
                    .iter()
                    .copied()
                    .map(|result| UnresolvedEvolutionResultV1 {
                        result,
                        candidates: Vec::new(),
                    })
                    .collect(),
                unresolved_sources: source_faces.clone(),
            }
        };

        let payload = Self {
            schema_version: FACE_EVOLUTION_SCHEMA_VERSION,
            source: EvolutionShapeV1 {
                solid: source_solid,
                faces: std::mem::take(&mut source_faces),
            },
            result: EvolutionShapeV1 {
                solid: result_solid,
                faces: std::mem::take(&mut result_faces),
            },
            evolution,
        };
        payload.validate()?;
        Ok(payload)
    }

    /// Decode JSON and enforce every version-1 structural invariant.
    ///
    /// # Errors
    ///
    /// Returns an error for malformed JSON, unsupported schema versions, or
    /// any duplicate, contradictory, out-of-domain, or incomplete claim.
    pub fn decode(json: &str) -> Result<Self, String> {
        let payload: Self = serde_json::from_str(json)
            .map_err(|error| format!("malformed face evolution payload: {error}"))?;
        payload.validate()?;
        Ok(payload)
    }

    fn validate(&self) -> Result<(), String> {
        if self.schema_version != FACE_EVOLUTION_SCHEMA_VERSION {
            return Err(format!(
                "unsupported face evolution schema version {}",
                self.schema_version
            ));
        }

        let source_faces = unique_set("source.faces", &self.source.faces)?;
        let result_faces = unique_set("result.faces", &self.result.faces)?;
        let modified = validate_relations(
            "modified",
            &self.evolution.modified,
            &source_faces,
            &result_faces,
        )?;
        let generated = validate_relations(
            "generated",
            &self.evolution.generated,
            &source_faces,
            &result_faces,
        )?;
        let deleted = unique_set("deleted", &self.evolution.deleted)?;
        ensure_subset("deleted", &deleted, "source.faces", &source_faces)?;
        let unresolved_sources =
            unique_set("unresolvedSources", &self.evolution.unresolved_sources)?;
        ensure_subset(
            "unresolvedSources",
            &unresolved_sources,
            "source.faces",
            &source_faces,
        )?;

        let mut unresolved_results = HashSet::new();
        for claim in &self.evolution.unresolved_results {
            if !result_faces.contains(&claim.result) {
                return Err(format!(
                    "unresolved result {} is not in result.faces",
                    claim.result
                ));
            }
            if !unresolved_results.insert(claim.result) {
                return Err(format!(
                    "unresolved result {} is claimed more than once",
                    claim.result
                ));
            }
            let candidates = unique_set("unresolved result candidates", &claim.candidates)?;
            ensure_subset(
                "unresolved result candidates",
                &candidates,
                "source.faces",
                &source_faces,
            )?;
        }

        ensure_disjoint(
            "modified results",
            &modified.results,
            "generated results",
            &generated.results,
        )?;
        ensure_disjoint(
            "modified results",
            &modified.results,
            "unresolved results",
            &unresolved_results,
        )?;
        ensure_disjoint(
            "generated results",
            &generated.results,
            "unresolved results",
            &unresolved_results,
        )?;
        let claimed_results: HashSet<u32> = modified
            .results
            .union(&generated.results)
            .copied()
            .chain(unresolved_results.iter().copied())
            .collect();
        ensure_equal(
            "result claims",
            &claimed_results,
            "result.faces",
            &result_faces,
        )?;

        ensure_disjoint("modified sources", &modified.sources, "deleted", &deleted)?;
        ensure_disjoint(
            "modified sources",
            &modified.sources,
            "unresolvedSources",
            &unresolved_sources,
        )?;
        ensure_disjoint(
            "deleted",
            &deleted,
            "unresolvedSources",
            &unresolved_sources,
        )?;
        let accounted_sources: HashSet<u32> = modified
            .sources
            .union(&deleted)
            .copied()
            .chain(unresolved_sources.iter().copied())
            .collect();
        ensure_equal(
            "source claims",
            &accounted_sources,
            "source.faces",
            &source_faces,
        )?;

        if self.evolution.provenance == EvolutionProvenanceV1::Unavailable
            && (!self.evolution.modified.is_empty()
                || !self.evolution.generated.is_empty()
                || !self.evolution.deleted.is_empty())
        {
            return Err(
                "unavailable provenance cannot contain modified, generated, or deleted claims"
                    .into(),
            );
        }

        Ok(())
    }
}

struct RelationSets {
    sources: HashSet<u32>,
    results: HashSet<u32>,
}

fn relations(
    map: &std::collections::HashMap<usize, Vec<usize>>,
) -> Result<Vec<EvolutionRelationV1>, String> {
    let mut claims: Vec<EvolutionRelationV1> = map
        .iter()
        .map(|(&source, results)| {
            Ok(EvolutionRelationV1 {
                source: index_to_u32(source)?,
                results: sorted_unique(
                    results
                        .iter()
                        .copied()
                        .map(index_to_u32)
                        .collect::<Result<Vec<_>, _>>()?,
                ),
            })
        })
        .collect::<Result<_, String>>()?;
    claims.sort_by_key(|claim| claim.source);
    Ok(claims)
}

fn validate_relations(
    label: &str,
    claims: &[EvolutionRelationV1],
    source_faces: &HashSet<u32>,
    result_faces: &HashSet<u32>,
) -> Result<RelationSets, String> {
    let mut sources = HashSet::new();
    let mut results = HashSet::new();
    let mut pairs = HashSet::new();
    for claim in claims {
        if !source_faces.contains(&claim.source) {
            return Err(format!(
                "{label} source {} is not in source.faces",
                claim.source
            ));
        }
        if !sources.insert(claim.source) {
            return Err(format!(
                "{label} source {} has more than one relation entry",
                claim.source
            ));
        }
        if claim.results.is_empty() {
            return Err(format!("{label} source {} has no results", claim.source));
        }
        let relation_results = unique_set(&format!("{label} results"), &claim.results)?;
        ensure_subset(
            &format!("{label} results"),
            &relation_results,
            "result.faces",
            result_faces,
        )?;
        for &result in &relation_results {
            if !pairs.insert((claim.source, result)) {
                return Err(format!(
                    "duplicate {label} claim {} -> {result}",
                    claim.source
                ));
            }
            results.insert(result);
        }
    }
    Ok(RelationSets { sources, results })
}

fn unique_set(label: &str, values: &[u32]) -> Result<HashSet<u32>, String> {
    let set: HashSet<u32> = values.iter().copied().collect();
    if set.len() != values.len() {
        return Err(format!("{label} contains duplicate handles"));
    }
    Ok(set)
}

fn ensure_subset(
    label: &str,
    values: &HashSet<u32>,
    domain_label: &str,
    domain: &HashSet<u32>,
) -> Result<(), String> {
    let mut outside: Vec<u32> = values.difference(domain).copied().collect();
    outside.sort_unstable();
    if outside.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "{label} contains handles outside {domain_label}: {outside:?}"
        ))
    }
}

fn ensure_disjoint(
    left_label: &str,
    left: &HashSet<u32>,
    right_label: &str,
    right: &HashSet<u32>,
) -> Result<(), String> {
    let mut overlap: Vec<u32> = left.intersection(right).copied().collect();
    overlap.sort_unstable();
    if overlap.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "{left_label} and {right_label} make contradictory claims for {overlap:?}"
        ))
    }
}

fn ensure_equal(
    left_label: &str,
    left: &HashSet<u32>,
    right_label: &str,
    right: &HashSet<u32>,
) -> Result<(), String> {
    let mut missing: Vec<u32> = right.difference(left).copied().collect();
    let mut extra: Vec<u32> = left.difference(right).copied().collect();
    missing.sort_unstable();
    extra.sort_unstable();
    if missing.is_empty() && extra.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "{left_label} do not equal {right_label}: missing {missing:?}, extra {extra:?}"
        ))
    }
}

fn sorted_unique(mut values: Vec<u32>) -> Vec<u32> {
    values.sort_unstable();
    values.dedup();
    values
}

fn index_to_u32(index: usize) -> Result<u32, String> {
    u32::try_from(index).map_err(|_| format!("face handle {index} exceeds the u32 range"))
}

#[cfg(test)]
mod evolution_payload_tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    fn valid_payload() -> serde_json::Value {
        serde_json::json!({
            "schemaVersion": 1,
            "source": { "solid": 1, "faces": [10, 11, 12] },
            "result": { "solid": 2, "faces": [20, 21] },
            "evolution": {
                "provenance": "construction",
                "modified": [{ "source": 10, "results": [20] }],
                "generated": [{ "source": 11, "results": [21] }],
                "deleted": [11],
                "unresolvedResults": [],
                "unresolvedSources": [12]
            }
        })
    }

    fn reject(mutator: impl FnOnce(&mut serde_json::Value)) {
        let mut value = valid_payload();
        mutator(&mut value);
        assert!(
            FaceEvolutionPayloadV1::decode(&value.to_string()).is_err(),
            "malformed payload was accepted: {value}"
        );
    }

    #[test]
    fn decoder_accepts_complete_modified_generated_deleted_payload() {
        let value = valid_payload();
        let payload = FaceEvolutionPayloadV1::decode(&value.to_string()).unwrap();
        assert_eq!(payload.schema_version, 1);
        assert_eq!(payload.evolution.deleted, vec![11]);
    }

    #[test]
    fn decoder_rejects_unknown_version_and_fields() {
        reject(|value| value["schemaVersion"] = 2.into());
        reject(|value| value["unexpected"] = true.into());
        reject(|value| value["evolution"]["unexpected"] = true.into());
    }

    #[test]
    fn decoder_rejects_duplicate_and_out_of_domain_handles() {
        reject(|value| value["source"]["faces"] = serde_json::json!([10, 10, 11, 12]));
        reject(|value| value["evolution"]["modified"][0]["results"] = serde_json::json!([20, 20]));
        reject(|value| {
            let duplicate = value["evolution"]["modified"][0].clone();
            value["evolution"]["modified"]
                .as_array_mut()
                .unwrap()
                .push(duplicate);
        });
        reject(|value| value["evolution"]["modified"][0]["results"] = serde_json::json!([99]));
        reject(|value| value["evolution"]["generated"][0]["source"] = 99.into());
    }

    #[test]
    fn decoder_rejects_contradictory_claims() {
        reject(|value| value["evolution"]["generated"][0]["results"] = serde_json::json!([20]));
        reject(|value| value["evolution"]["deleted"] = serde_json::json!([10, 11]));
        reject(|value| {
            value["evolution"]["unresolvedResults"] = serde_json::json!([
                { "result": 20, "candidates": [10] }
            ]);
        });
    }

    #[test]
    fn decoder_rejects_incomplete_source_and_result_coverage() {
        reject(|value| value["evolution"]["unresolvedSources"] = serde_json::json!([]));
        reject(|value| value["evolution"]["generated"] = serde_json::json!([]));
    }

    #[test]
    fn decoder_rejects_claims_disguised_as_unavailable_provenance() {
        reject(|value| value["evolution"]["provenance"] = "unavailable".into());
    }

    #[test]
    fn encoder_preserves_explicit_deletions_and_validates_itself() {
        let mut map = EvolutionMap::exact();
        map.add_modified(10, 20);
        map.add_generated(11, 21);
        map.add_deleted(11);
        let payload =
            FaceEvolutionPayloadV1::from_map(1, 2, vec![10, 11], vec![20, 21], &map).unwrap();
        assert_eq!(payload.evolution.deleted, vec![11]);
        assert!(payload.evolution.unresolved_sources.is_empty());
        assert!(payload.evolution.unresolved_results.is_empty());
    }

    #[test]
    fn encoder_refuses_to_promote_geometric_inference() {
        let mut inferred = EvolutionMap::new();
        inferred.add_modified(10, 20);
        let payload =
            FaceEvolutionPayloadV1::from_map(1, 2, vec![10], vec![20], &inferred).unwrap();

        assert_eq!(
            payload.evolution.provenance,
            EvolutionProvenanceV1::Unavailable
        );
        assert!(payload.evolution.modified.is_empty());
        assert_eq!(payload.evolution.unresolved_sources, vec![10]);
        assert_eq!(payload.evolution.unresolved_results[0].result, 20);
    }
}

/// Typed result for `massProperties`.
#[derive(serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct MassPropertiesResult {
    /// Solid volume (mass at unit density).
    pub volume: f64,
    /// Center of mass `[x, y, z]`.
    pub center_of_mass: Vec<f64>,
    /// Inertia tensor about the center of mass, global axes:
    /// `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]` (unit density).
    pub inertia: Vec<f64>,
    /// Principal moments of inertia, ascending.
    pub principal_moments: Vec<f64>,
    /// Principal axes as three unit vectors, row-major
    /// `[x0, y0, z0, x1, y1, z1, x2, y2, z2]`, matching `principalMoments`.
    pub principal_axes: Vec<f64>,
}

/// Typed result for `meshQuality`.
#[derive(serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct MeshQualityResult {
    /// Edges used by exactly one triangle after position welding (0 for a
    /// watertight mesh).
    pub boundary_edges: u32,
    /// Edges used by more than two triangles after position welding.
    pub non_manifold_edges: u32,
    /// Euler characteristic `V - E + F` of the welded mesh (2 for a single
    /// closed genus-0 shell).
    pub euler_characteristic: i32,
    /// True when the welded mesh has no boundary and no non-manifold edges.
    pub is_watertight: bool,
}

/// One issue reported by detailed solid validation.
#[derive(Debug, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct ValidationIssueResult {
    /// Issue severity: `error` or `warning`.
    pub severity: String,
    /// Human-readable description supplied by the operations validator.
    pub description: String,
}

/// Typed result for `validateSolidDetailed` and
/// `validateSolidDetailedWithOptions`.
#[derive(Debug, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct ValidationReportResult {
    /// Number of error-severity issues.
    pub error_count: u32,
    /// Number of warning-severity issues.
    pub warning_count: u32,
    /// All validation issues in validator order.
    pub issues: Vec<ValidationIssueResult>,
}

/// Typed result for `sketchSolve`.
#[derive(serde::Serialize, Tsify)]
#[tsify(into_wasm_abi)]
pub struct SketchSolveResult {
    pub converged: bool,
    pub points: Vec<f64>,
    pub residual: f64,
}

/// Per-step entry in a `HealPipelineResult`.
#[derive(Debug, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct HealStepResult {
    /// Operator name that ran.
    pub step: String,
    /// Number of individual repair actions taken.
    pub actions_taken: u32,
    /// At least one fix was applied.
    pub done: bool,
    /// At least one fix could not be applied.
    pub failed: bool,
}

/// Typed result for `fixShapeWithConfig`.
#[derive(Debug, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct HealFixResult {
    /// Handle of the healed solid (may differ from the input).
    pub solid: u32,
    /// Number of individual repair actions taken.
    pub actions_taken: u32,
    /// At least one fix was applied.
    pub done: bool,
    /// At least one fix could not be applied.
    pub failed: bool,
}

/// Typed result for `runHealPipeline`.
#[derive(Debug, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct HealPipelineResult {
    /// Handle of the healed solid (may differ from the input).
    pub solid: u32,
    /// One entry per executed step, in order.
    pub steps: Vec<HealStepResult>,
}

/// Typed result for `gcsSolve`.
#[derive(Debug, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct GcsSolveResult {
    /// Whether the solver converged within tolerance.
    pub converged: bool,
    /// Number of DogLeg iterations used.
    pub iterations: u32,
    /// Maximum absolute residual after solving.
    pub max_residual: f64,
}

/// Residual magnitude attributed to one constraint in a `gcsSolveDetailed`
/// report.
///
/// A large magnitude is evidence about *where* a system is unsatisfied, not
/// proof that this constraint is at fault — one bad constraint pushes error
/// into every constraint sharing its parameters.
#[derive(Debug, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct GcsConstraintResidual {
    /// The `gcsAddConstraint` handle this magnitude belongs to.
    pub constraint: u32,
    /// Largest absolute residual across the constraint's equations, measured
    /// at the solver's final iterate — its best attempt, before any rollback.
    /// Constraints the system could satisfy read ~0 here, so a magnitude that
    /// survives marks where it could not.
    pub max_residual: f64,
}

/// Typed result for `gcsSolveDetailed`.
///
/// Kernel-internal constraints (an arc's centre–endpoint tie) carry no
/// `gcsAddConstraint` handle. They are excluded from `constraintResiduals`
/// entirely and summarised by `internalMaxResidual` instead, so no internal
/// equation is ever attributed to a caller's constraint.
#[derive(Debug, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct GcsSolveDiagnostics {
    /// Whether the solver reached the requested tolerance.
    pub converged: bool,
    /// Number of DogLeg iterations used.
    pub iterations: u32,
    /// Maximum absolute residual at the solver's final iterate.
    pub max_residual: f64,
    /// Maximum absolute residual at the state now published in the sketch.
    /// Differs from `maxResidual` only when `rolledBack` is set.
    pub published_max_residual: f64,
    /// Degrees of freedom remaining (`numParams - rank`).
    pub dof: u32,
    /// Rank of the constraint Jacobian.
    pub rank: u32,
    /// Total free solver parameters.
    pub num_params: u32,
    /// Total residual equations, kernel-internal constraints included.
    pub num_equations: u32,
    /// Per-constraint residuals for caller-added constraints only.
    pub constraint_residuals: Vec<GcsConstraintResidual>,
    /// Largest residual over kernel-internal constraints alone.
    pub internal_max_residual: f64,
    /// Whether the attempt was discarded and the pre-solve geometry restored.
    /// A rejected solve never leaves partially moved geometry published.
    pub rolled_back: bool,
    /// Whether some equation is linearly dependent on the others
    /// (`rank < numEquations`). Reported independently of `classification`,
    /// which can only name one state.
    pub redundant: bool,
    /// One of `solved`, `underConstrained`, `redundant`, `unsatisfied`.
    ///
    /// `unsatisfied` means the solver did not converge — it does **not**
    /// identify a conflicting constraint. Non-convergence is equally
    /// consistent with contradictory constraints, a poor starting point, or
    /// too small an iteration budget.
    pub classification: String,
}

/// Typed result for `polygonUnion2d` and `polygonBoolean2d`.
///
/// Each loop is a flat `[x0, y0, x1, y1, ...]` array of 2D coordinates.
/// `outer` loops are counter-clockwise, `holes` are clockwise; a loop is
/// implicitly closed (the last point is not repeated). Keeping the two
/// lists separate is the whole point of this type — a downstream consumer
/// building a face with holes must know which loops bound material and
/// which remove it.
#[derive(Debug, Default, serde::Serialize, Tsify)]
#[tsify(into_wasm_abi)]
pub struct PolygonBoolean2dResult {
    /// Counter-clockwise outer boundary loops, each a flat `[x, y, ...]` array.
    pub outer: Vec<Vec<f64>>,
    /// Clockwise hole loops, each a flat `[x, y, ...]` array.
    pub holes: Vec<Vec<f64>>,
}

/// Typed result for `gcsDof`.
#[derive(Debug, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct GcsDofResult {
    /// Degrees of freedom remaining (under-constrained dimensions).
    pub dof: u32,
    /// Rank of the constraint Jacobian.
    pub rank: u32,
    /// Total solver parameters.
    pub num_params: u32,
    /// Total constraint equations.
    pub num_equations: u32,
}
