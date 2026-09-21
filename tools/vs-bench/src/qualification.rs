//! First executable Remus qualification runner (native surface only).
//!
//! This module executes a bounded set of qualification cases through the
//! native [`remus::Model`] facade and maps each real execution result to one
//! [O1.2d](super) observation. It does not judge: judging stays in
//! [`crate::evaluate_json`]. Feeding this module's job output through the
//! existing `remus-vs-bench` CLI yields the versioned scorecard report.
//!
//! Design notes, all load-bearing for honest evidence:
//!
//! * The independent oracle for every case is closed-form arithmetic (or the
//!   algebraic empty set) computed from the case inputs without touching
//!   kernel geometry. The kernel measurement is one route; the oracle is the
//!   other. Native/WASM agreement is never claimed and no second kernel runs,
//!   so this module cannot produce a head-to-head ranking or a speed claim.
//! * Each case runs in a fresh child process with a wall-clock ceiling. The
//!   ceiling is a harness safety bound, not a measured latency band.
//! * A timed-out, crashed, or otherwise incomplete case yields **no**
//!   observation: the O1.2d schema requires measured values for every
//!   applicable metric, so an incomplete case cannot become a row. The parent
//!   records the attempt and refuses to emit a partial job instead.
//! * A typed kernel refusal (for example the algebraic empty set) is kept as
//!   a refusal observation. Refusals are admissible evidence, never relabeled
//!   as successes.
//!
//! There is no production fault injection in this module: every constructor
//! takes caller-supplied values, so tests exercise the mapping with synthetic
//! inputs without touching the execution path.

use std::collections::BTreeMap;
use std::io::Write;
use std::process::{Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};

use remus::{BooleanOp, BooleanQuality, Model};
use remus_math::mat::Mat4;
use remus_operations::journal_ops::solid_entity_keys;
use remus_topology::explorer::solid_faces;
use serde::{Deserialize, Serialize};

/// STEP fixture: a radius-4, height-10 cylinder with omitted OPTIONAL
/// placement attributes, as written by real CAD exports.
///
/// Closed forms (independent of the STEP reader): volume `160*pi`,
/// whole-boundary area `112*pi`.
const STEP_CYLINDER: &str =
    include_str!("../../../crates/io/tests/data/axis2_optional_attrs_cylinder.step");

/// Schema version this runner targets. Mirrors [`crate::SCHEMA_VERSION`]
/// without depending on the private schema types.
const SCHEMA_VERSION: u32 = 1;
/// Declared tessellation deflection in millimetres. Every mesh-derived check
/// (watertightness, tessellation-route volume) runs at this deflection.
const DEFLECTION: f64 = 0.01;
/// Declared tolerance model. Matches the kernel defaults
/// (linear `1e-7`, angular `1e-12`); units are millimetres and radians.
const TOLERANCE_MODEL: &str = "linear=1e-7;angular=1e-12";
/// Default per-case wall-clock ceiling. A harness safety bound, not a
/// performance band.
const DEFAULT_TIMEOUT_MS: u64 = 60_000;
/// Gauntlet manifest this run references. Content is pinned by hash at
/// runtime; no corpus bytes are read.
const MANIFEST_NAME: &str = "tools/gauntlet/manifests/smoke.json";
/// Nibble table for manifest hashing.
const HEX_NIBBLES: &[u8; 16] = b"0123456789abcdef";

/// A qualification case: scope, oracle, tolerance intent, resource bound and
/// reproduction identity, all declared up front.
#[derive(Debug, Clone, Copy)]
pub struct CaseSpec {
    /// Stable case identity, also used as the scenario id.
    pub id: &'static str,
    /// Supported scope: what configuration this case exercises.
    pub scope: &'static str,
    /// Independent oracle: the non-kernel route the result is checked against.
    pub oracle: &'static str,
    /// Tolerance and error intent for the oracle comparison.
    pub tolerance_intent: &'static str,
    /// Wall-clock ceiling per repetition, in milliseconds.
    pub timeout_ms: u64,
    /// Pinned reproduction identity: inputs plus fixture provenance.
    pub repro: &'static str,
}

/// The bounded case set. See the module docs for what each family proves.
#[must_use]
pub fn case_specs() -> Vec<CaseSpec> {
    vec![
        CaseSpec {
            id: "box-fuse-half-overlap",
            scope: "axis-aligned planar box/box fuse through the exact journaled path",
            oracle: "closed-form volume 1.5 = 1 + 1 - 0.5 from input dimensions",
            tolerance_intent: "relative volume error <= 1e-6; exact representation, zero error budget",
            timeout_ms: DEFAULT_TIMEOUT_MS,
            repro: "make_box(1,1,1) + make_box(1,1,1)@(0.5,0,0), Fuse",
        },
        CaseSpec {
            id: "box-fuse-identical",
            scope: "coincident-everything fuse (unchanged-stock guard: oracles still apply)",
            oracle: "closed-form volume 1.0 from input dimensions",
            tolerance_intent: "relative volume error <= 1e-6; exact representation, zero error budget",
            timeout_ms: DEFAULT_TIMEOUT_MS,
            repro: "make_box(1,1,1) + make_box(1,1,1), Fuse",
        },
        CaseSpec {
            id: "box-cut-contained-cavity",
            scope: "full-containment cut producing a hollow solid with an inner cavity shell",
            oracle: "closed-form volume 7.0 = 8 - 1 and whole-boundary area 30.0 = 24 + 6 (outer plus cavity material)",
            tolerance_intent: "relative volume and area error <= 1e-6; exact representation, zero error budget",
            timeout_ms: DEFAULT_TIMEOUT_MS,
            repro: "make_box(2,2,2) - make_box(1,1,1)@(0.5,0.5,0.5), Cut",
        },
        CaseSpec {
            id: "box-cut-identical-empty",
            scope: "algebraic empty set via subtraction on the plain exact facade path (Model::boolean)",
            oracle: "algebraic empty set A - A = {}; only a typed EmptyResult refusal agrees",
            tolerance_intent: "no numeric tolerance: outcome identity only",
            timeout_ms: DEFAULT_TIMEOUT_MS,
            repro: "make_box(1,1,1) - make_box(1,1,1), Cut via Model::boolean",
        },
        CaseSpec {
            id: "box-intersect-disjoint-empty",
            scope: "algebraic empty set via disjoint intersection on the plain exact facade path (Model::boolean)",
            oracle: "algebraic empty set; agrees only with zero faces and ~0 volume, or typed EmptyResult refusal",
            tolerance_intent: "volume <= 1e-6 and zero faces for the empty-solid outcome; no tolerance on refusals",
            timeout_ms: DEFAULT_TIMEOUT_MS,
            repro: "make_box(1,1,1) n make_box(1,1,1)@(5,5,5), Intersect via Model::boolean",
        },
        CaseSpec {
            id: "step-cylinder-preservation",
            scope: "STEP import of the committed axis2-optional-attrs cylinder, exact re-export and reimport",
            oracle: "closed-form cylinder volume 160*pi (r=4,h=10 from the fixture header) plus write/reimport volume stability <= 1e-9",
            tolerance_intent: "relative import volume error <= 1e-6; round-trip stability <= 1e-9; exact representation, zero error budget",
            timeout_ms: DEFAULT_TIMEOUT_MS,
            repro: "crates/io/tests/data/axis2_optional_attrs_cylinder.step",
        },
    ]
}

/// Looks up a case by id.
#[must_use]
pub fn case_spec(id: &str) -> Option<CaseSpec> {
    case_specs().into_iter().find(|c| c.id == id)
}

/// Runner-level failure. Kernel-typed refusals and wrong results are data,
/// carried inside [`CaseEvidence`], never raised as errors.
#[derive(Debug, thiserror::Error)]
pub enum QualificationError {
    /// Unknown case id.
    #[error("unknown case: {0}")]
    UnknownCase(String),
    /// A kernel or measurement step failed in a way the case does not expect.
    #[error("worker failure in {case}: {detail}")]
    Worker {
        /// Case under execution.
        case: String,
        /// What failed.
        detail: String,
    },
    /// A value needed for the evidence contract is missing or non-finite.
    #[error("incomplete evidence in {case}: {detail}")]
    Incomplete {
        /// Case under execution.
        case: String,
        /// What is missing.
        detail: String,
    },
    /// Child-process handling failed.
    #[error("child process failure: {0}")]
    Child(String),
    /// Output serialisation failed.
    #[error("serialisation failure: {0}")]
    Serialise(String),
}

/// Real execution evidence for one case repetition.
///
/// `status` mirrors the kernel's claim: `"success"` (a solid was built),
/// `"empty_success"` (a zero-face empty solid), `"refused"` (typed kernel
/// refusal) or `"error"` (anything else). The parent maps this claim plus the
/// independent oracle verdict to the reported observation; a claimed success
/// that misses its oracle becomes `oracle_agrees: false`, never a relabeled
/// refusal.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CaseEvidence {
    /// Case id.
    pub case: String,
    /// Kernel claim: `success`, `empty_success`, `refused` or `error`.
    pub status: String,
    /// Stable diagnostic for `refused`/`error` (for example `EmptyResult`).
    pub diagnostic: Option<String>,
    /// Exact-integrated volume (`mass_properties`), where measurable.
    pub volume: Option<f64>,
    /// Whole-boundary area over outer plus cavity shells, where measured.
    pub area: Option<f64>,
    /// Face count over outer plus inner shells, where measurable.
    pub faces: Option<usize>,
    /// Tessellation watertightness at the configured deflection, where measured.
    pub watertight: Option<bool>,
    /// Strict validation verdict, where measured.
    pub valid: Option<bool>,
    /// Fraction of result entities carrying a journal evolution event.
    pub evolution_completeness: Option<f64>,
    /// Provenance note for paths that record no construction lineage
    /// (for example STEP import, which transcribes foreign topology).
    /// Carried in the attempts log; never a scorecard metric.
    pub journal_note: Option<String>,
    /// STEP import validity, where applicable.
    pub import_validity: Option<bool>,
    /// Post-import operation success, where applicable.
    pub post_import_success: Option<bool>,
    /// Round-trip fidelity errors (volume, area, centroid, bounds).
    pub round_trip: Option<RoundTripFidelity>,
}

/// Write/read stability measurements for the STEP case: relative errors for
/// size-like quantities, absolute millimetre drift for positions.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RoundTripFidelity {
    /// Relative volume change across write and reimport.
    pub volume_rel: f64,
    /// Relative area change across write and reimport.
    pub area_rel: f64,
    /// Absolute centroid displacement across write and reimport, in mm.
    pub centroid_abs: f64,
    /// Maximum corner displacement of the bounding box, in mm.
    pub bounds_abs: f64,
}

/// Relative error with a floor of 1.0 so near-zero oracles stay meaningful.
fn rel_err(got: f64, expected: f64) -> f64 {
    let denom = got.abs().max(expected.abs()).max(1.0);
    (got - expected).abs() / denom
}

/// Requires a finite f64 for the evidence contract.
fn require_finite(value: f64, what: &str, case: &str) -> Result<f64, QualificationError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(QualificationError::Incomplete {
            case: case.to_string(),
            detail: format!("non-finite {what}"),
        })
    }
}

/// Closed triangle mesh edge-closure check: every undirected edge must appear
/// exactly twice, in opposite directions. An empty mesh is vacuously closed.
#[must_use]
pub fn mesh_watertight(positions_len: usize, indices: &[u32]) -> bool {
    if !indices.len().is_multiple_of(3) {
        return false;
    }
    let mut edges: BTreeMap<(u32, u32), i32> = BTreeMap::new();
    for tri in indices.chunks_exact(3) {
        let [a, b, c] = [tri[0], tri[1], tri[2]];
        if a >= positions_len as u32 || b >= positions_len as u32 || c >= positions_len as u32 {
            return false;
        }
        for (from, to) in [(a, b), (b, c), (c, a)] {
            if from == to {
                return false;
            }
            let key = (from.min(to), from.max(to));
            let sign = if from < to { 1 } else { -1 };
            *edges.entry(key).or_insert(0) += sign;
        }
    }
    edges.values().all(|&balance| balance == 0)
}

/// Fraction of a solid's faces, edges and vertices carrying at least one
/// journal evolution event.
fn evolution_fraction(
    model: &Model,
    solid: remus::SolidId,
    case: &str,
) -> Result<f64, QualificationError> {
    let keys =
        solid_entity_keys(model.topology(), solid).map_err(|e| QualificationError::Worker {
            case: case.to_string(),
            detail: format!("entity keys: {e:?}"),
        })?;
    if keys.is_empty() {
        return Err(QualificationError::Incomplete {
            case: case.to_string(),
            detail: "result solid has no entities".to_string(),
        });
    }
    let journal = model.journal();
    let mut covered = 0_usize;
    for key in &keys {
        let has_events = journal
            .ordinal_of(*key)
            .is_some_and(|ord| !journal.events_for(ord).is_empty());
        covered += usize::from(has_events);
    }
    require_finite(
        covered as f64 / keys.len() as f64,
        "evolution fraction",
        case,
    )
}

fn place_box(
    model: &mut Model,
    dx: f64,
    dy: f64,
    dz: f64,
    at: (f64, f64, f64),
    case: &str,
) -> Result<remus::SolidId, QualificationError> {
    let solid = model
        .make_box(dx, dy, dz)
        .map_err(|e| QualificationError::Worker {
            case: case.to_string(),
            detail: format!("make_box: {e:?}"),
        })?;
    if at != (0.0, 0.0, 0.0) {
        model
            .transform(solid, &Mat4::translation(at.0, at.1, at.2))
            .map_err(|e| QualificationError::Worker {
                case: case.to_string(),
                detail: format!("transform: {e:?}"),
            })?;
    }
    Ok(solid)
}

fn stable_diagnostic(error: &remus::OperationsError) -> String {
    match error {
        remus::OperationsError::EmptyResult { reason } => format!("EmptyResult: {reason}"),
        remus::OperationsError::ExactOnlyUnattainable => "ExactOnlyUnattainable".to_string(),
        remus::OperationsError::Unsupported { operation, reason } => {
            format!("Unsupported({operation}): {reason}")
        }
        other => format!("{other:?}"),
    }
}

/// Measures one built solid: exact volume, whole-boundary area, face census,
/// mesh watertightness, validation verdict and journal evolution coverage.
#[allow(clippy::too_many_lines)]
fn measure_solid(
    model: &Model,
    solid: remus::SolidId,
    case: &str,
) -> Result<CaseEvidence, QualificationError> {
    let props = model
        .mass_properties(solid)
        .map_err(|e| QualificationError::Worker {
            case: case.to_string(),
            detail: format!("mass_properties: {e:?}"),
        })?;
    let volume = require_finite(props.mass, "volume", case)?;
    let area = require_finite(
        model
            .surface_area(solid, DEFLECTION)
            .map_err(|e| QualificationError::Worker {
                case: case.to_string(),
                detail: format!("surface_area: {e:?}"),
            })?,
        "area",
        case,
    )?;
    let faces = solid_faces(model.topology(), solid)
        .map_err(|e| QualificationError::Worker {
            case: case.to_string(),
            detail: format!("solid_faces: {e:?}"),
        })?
        .len();
    let mesh = model
        .tessellate(solid, DEFLECTION)
        .map_err(|e| QualificationError::Worker {
            case: case.to_string(),
            detail: format!("tessellate: {e:?}"),
        })?;
    let watertight = mesh_watertight(mesh.positions.len(), &mesh.indices);
    let valid = model
        .validate(solid)
        .map_err(|e| QualificationError::Worker {
            case: case.to_string(),
            detail: format!("validate: {e:?}"),
        })?
        .is_valid();
    let evolution_completeness = evolution_fraction(model, solid, case)?;
    Ok(CaseEvidence {
        case: case.to_string(),
        status: "success".to_string(),
        diagnostic: None,
        volume: Some(volume),
        area: Some(area),
        faces: Some(faces),
        watertight: Some(watertight),
        valid: Some(valid),
        evolution_completeness: Some(evolution_completeness),
        journal_note: None,
        import_validity: None,
        post_import_success: None,
        round_trip: None,
    })
}

fn run_box_boolean(
    case: &str,
    op: BooleanOp,
    journaled: bool,
    a_dims: (f64, f64, f64),
    a_at: (f64, f64, f64),
    b_dims: (f64, f64, f64),
    b_at: (f64, f64, f64),
) -> Result<CaseEvidence, QualificationError> {
    let mut model = Model::new();
    let a = place_box(&mut model, a_dims.0, a_dims.1, a_dims.2, a_at, case)?;
    let b = place_box(&mut model, b_dims.0, b_dims.1, b_dims.2, b_at, case)?;
    // The journaled (persistent-naming) path is exact-only by construction
    // and records construction lineage; the plain facade path discloses its
    // quality, which must be exact for these planar cases. Empty inputs run
    // on the plain path, where the empty-outcome contract (EmptyResult /
    // zero-face solid) is qualified; the journaled path bypasses the
    // trivial-relation short-circuits and surfaces Algo(AssemblyFailed)
    // instead (retained as a prose witness for the Boolean owner, not a
    // case in this set).
    let solid = if journaled {
        match model.boolean_journaled(op, a, b) {
            Ok(outcome) => outcome.solid,
            Err(e) => return Ok(empty_outcome(case, e)),
        }
    } else {
        match model.boolean(op, a, b) {
            Ok(outcome) => {
                if outcome.quality != BooleanQuality::Exact {
                    return Ok(CaseEvidence {
                        case: case.to_string(),
                        status: "error".to_string(),
                        diagnostic: Some(format!(
                            "unexpected-approximate-fallback: {:?}",
                            outcome.quality
                        )),
                        volume: None,
                        area: None,
                        faces: None,
                        watertight: None,
                        valid: None,
                        evolution_completeness: None,
                        journal_note: None,
                        import_validity: None,
                        post_import_success: None,
                        round_trip: None,
                    });
                }
                outcome.solid
            }
            Err(e) => return Ok(empty_outcome(case, e)),
        }
    };
    measure_solid(&model, solid, case)
}

/// Maps a boolean error to refused/error evidence.
///
/// Only the typed [`remus::OperationsError::EmptyResult`] agrees with the
/// algebraic empty set; every other error is recorded as-is for the oracle
/// to reject.
#[must_use]
pub fn empty_outcome(case: &str, error: remus::OperationsError) -> CaseEvidence {
    let diagnostic = stable_diagnostic(&error);
    let empty_refusal = matches!(error, remus::OperationsError::EmptyResult { .. });
    CaseEvidence {
        case: case.to_string(),
        status: if empty_refusal { "refused" } else { "error" }.to_string(),
        diagnostic: Some(diagnostic),
        volume: None,
        area: None,
        faces: None,
        watertight: None,
        valid: None,
        evolution_completeness: None,
        journal_note: None,
        import_validity: None,
        post_import_success: None,
        round_trip: None,
    }
}

fn run_disjoint_intersect(case: &str) -> Result<CaseEvidence, QualificationError> {
    let mut model = Model::new();
    let a = place_box(&mut model, 1.0, 1.0, 1.0, (0.0, 0.0, 0.0), case)?;
    let b = place_box(&mut model, 1.0, 1.0, 1.0, (5.0, 5.0, 5.0), case)?;
    // Plain facade path: the disjoint-intersection empty-solid contract is
    // qualified there (zero faces, ~0 volume) or refused as EmptyResult.
    // The disclosed quality must be exact for this planar case.
    match model.boolean(BooleanOp::Intersect, a, b) {
        Ok(outcome) => {
            if outcome.quality != BooleanQuality::Exact {
                return Ok(CaseEvidence {
                    case: case.to_string(),
                    status: "error".to_string(),
                    diagnostic: Some(format!(
                        "unexpected-approximate-fallback: {:?}",
                        outcome.quality
                    )),
                    volume: None,
                    area: None,
                    faces: None,
                    watertight: None,
                    valid: None,
                    evolution_completeness: None,
                    journal_note: None,
                    import_validity: None,
                    post_import_success: None,
                    round_trip: None,
                });
            }
            let faces = solid_faces(model.topology(), outcome.solid)
                .map_err(|e| QualificationError::Worker {
                    case: case.to_string(),
                    detail: format!("solid_faces: {e:?}"),
                })?
                .len();
            if faces != 0 {
                // Non-empty result for a disjoint intersection: still measure
                // so the parent can record oracle disagreement honestly.
                let mut evidence = measure_solid(&model, outcome.solid, case)?;
                evidence.case = case.to_string();
                return Ok(evidence);
            }
            let volume = require_finite(
                model.volume(outcome.solid, DEFLECTION).map_err(|e| {
                    QualificationError::Worker {
                        case: case.to_string(),
                        detail: format!("volume: {e:?}"),
                    }
                })?,
                "empty volume",
                case,
            )?;
            let valid = model
                .validate(outcome.solid)
                .map(|r| r.is_valid())
                .unwrap_or(false);
            Ok(CaseEvidence {
                case: case.to_string(),
                status: "empty_success".to_string(),
                diagnostic: None,
                volume: Some(volume),
                area: None,
                faces: Some(0),
                watertight: Some(true),
                valid: Some(valid),
                evolution_completeness: None,
                journal_note: None,
                import_validity: None,
                post_import_success: None,
                round_trip: None,
            })
        }
        Err(e) => Ok(empty_outcome(case, e)),
    }
}

#[allow(clippy::too_many_lines)]
fn run_step_cylinder(case: &str) -> Result<CaseEvidence, QualificationError> {
    let worker = |detail: String| QualificationError::Worker {
        case: case.to_string(),
        detail,
    };
    let mut model = Model::new();
    let ids = model
        .read_step(STEP_CYLINDER)
        .map_err(|e| worker(format!("read_step: {e:?}")))?;
    if ids.len() != 1 {
        return Err(worker(format!(
            "expected exactly one imported solid, got {}",
            ids.len()
        )));
    }
    let solid = ids[0];
    let valid = model
        .validate(solid)
        .map_err(|e| worker(format!("validate: {e:?}")))?
        .is_valid();
    let import_validity = valid;
    let props = model
        .mass_properties(solid)
        .map_err(|e| worker(format!("mass_properties: {e:?}")))?;
    let volume = require_finite(props.mass, "import volume", case)?;
    let area = require_finite(
        model
            .surface_area(solid, DEFLECTION)
            .map_err(|e| worker(format!("surface_area: {e:?}")))?,
        "import area",
        case,
    )?;
    let center = props.center;
    let bounds = model
        .bounding_box(solid)
        .map_err(|e| worker(format!("bounding_box: {e:?}")))?;
    // Post-import operation: tessellate the imported body. Success proves the
    // imported topology is operable, not merely present.
    let mesh = model
        .tessellate(solid, DEFLECTION)
        .map_err(|e| worker(format!("post-import tessellate: {e:?}")))?;
    let post_import_success = true;
    let watertight = mesh_watertight(mesh.positions.len(), &mesh.indices);
    let faces = solid_faces(model.topology(), solid)
        .map_err(|e| worker(format!("solid_faces: {e:?}")))?
        .len();
    let evolution_completeness = evolution_fraction(&model, solid, case).ok();
    let text = model
        .write_step(&[solid])
        .map_err(|e| worker(format!("write_step: {e:?}")))?;
    let mut model2 = Model::new();
    let ids2 = model2
        .read_step(&text)
        .map_err(|e| worker(format!("reimport: {e:?}")))?;
    if ids2.len() != 1 {
        return Err(worker(format!(
            "expected exactly one reimported solid, got {}",
            ids2.len()
        )));
    }
    let props2 = model2
        .mass_properties(ids2[0])
        .map_err(|e| worker(format!("reimport mass_properties: {e:?}")))?;
    let area2 = model2
        .surface_area(ids2[0], DEFLECTION)
        .map_err(|e| worker(format!("reimport surface_area: {e:?}")))?;
    let bounds2 = model2
        .bounding_box(ids2[0])
        .map_err(|e| worker(format!("reimport bounding_box: {e:?}")))?;
    let dc = props2.center - center;
    let centroid_error = require_finite(
        (dc.x() * dc.x() + dc.y() * dc.y() + dc.z() * dc.z()).sqrt(),
        "centroid drift",
        case,
    )?;
    let bounds_error = require_finite(
        (bounds2.min.x() - bounds.min.x())
            .abs()
            .max((bounds2.min.y() - bounds.min.y()).abs())
            .max((bounds2.min.z() - bounds.min.z()).abs())
            .max((bounds2.max.x() - bounds.max.x()).abs())
            .max((bounds2.max.y() - bounds.max.y()).abs())
            .max((bounds2.max.z() - bounds.max.z()).abs()),
        "bounds drift",
        case,
    )?;
    // Import transcribes foreign topology rather than constructing it, so
    // the journal may carry no construction lineage. Attest completeness
    // only when events exist; otherwise record the observation as a note
    // for the attempts log and leave the scorecard metric undeclared.
    let (evolution_completeness, journal_note) = match evolution_completeness {
        Some(fraction) if fraction > 0.0 => (Some(fraction), None),
        _ => (
            None,
            Some(
                "step import records no journal evolution events; import transcribes \
                 foreign topology rather than constructing it"
                    .to_string(),
            ),
        ),
    };
    Ok(CaseEvidence {
        case: case.to_string(),
        status: "success".to_string(),
        diagnostic: None,
        volume: Some(volume),
        area: Some(area),
        faces: Some(faces),
        watertight: Some(watertight),
        valid: Some(valid),
        evolution_completeness,
        journal_note,
        import_validity: Some(import_validity),
        post_import_success: Some(post_import_success),
        round_trip: Some(RoundTripFidelity {
            volume_rel: rel_err(props2.mass, volume),
            area_rel: rel_err(area2, area),
            centroid_abs: centroid_error,
            bounds_abs: bounds_error,
        }),
    })
}

/// Executes one case once, in-process. The parent runs this in a child
/// process so a hang or crash cannot take down the run.
///
/// # Errors
///
/// Returns [`QualificationError`] for unknown cases or worker-internal
/// failures. Kernel-typed refusals are returned as evidence, not errors.
pub fn execute_case(id: &str) -> Result<CaseEvidence, QualificationError> {
    match id {
        "box-fuse-half-overlap" => run_box_boolean(
            id,
            BooleanOp::Fuse,
            true,
            (1.0, 1.0, 1.0),
            (0.0, 0.0, 0.0),
            (1.0, 1.0, 1.0),
            (0.5, 0.0, 0.0),
        ),
        "box-fuse-identical" => run_box_boolean(
            id,
            BooleanOp::Fuse,
            true,
            (1.0, 1.0, 1.0),
            (0.0, 0.0, 0.0),
            (1.0, 1.0, 1.0),
            (0.0, 0.0, 0.0),
        ),
        "box-cut-contained-cavity" => run_box_boolean(
            id,
            BooleanOp::Cut,
            true,
            (2.0, 2.0, 2.0),
            (0.0, 0.0, 0.0),
            (1.0, 1.0, 1.0),
            (0.5, 0.5, 0.5),
        ),
        "box-cut-identical-empty" => run_box_boolean(
            id,
            BooleanOp::Cut,
            false,
            (1.0, 1.0, 1.0),
            (0.0, 0.0, 0.0),
            (1.0, 1.0, 1.0),
            (0.0, 0.0, 0.0),
        ),
        "box-intersect-disjoint-empty" => run_disjoint_intersect(id),
        "step-cylinder-preservation" => run_step_cylinder(id),
        other => Err(QualificationError::UnknownCase(other.to_string())),
    }
}

/// How one child repetition ended.
#[derive(Debug, Clone, PartialEq)]
pub enum ChildOutcome {
    /// The child printed evidence JSON and exited zero.
    Evidence(CaseEvidence),
    /// The wall-clock ceiling expired; the child was killed.
    Timeout,
    /// The child died abnormally (signal or nonzero exit without evidence).
    Crash(String),
}

/// Classifies an already-reaped exit status without evidence output.
#[must_use]
pub fn classify_status(status: ExitStatus, stderr_excerpt: &str) -> ChildOutcome {
    ChildOutcome::Crash(format!("exit {status}: {stderr_excerpt}"))
}

/// Runs one case repetition in a fresh child process with a wall-clock
/// ceiling. `child_argv0` is the runner binary; `extra_args` are appended
/// after the `--worker <case>` pair.
///
/// # Errors
///
/// Returns [`QualificationError::Child`] when the child cannot be spawned.
/// Timeout and crash outcomes are returned as data, never raised.
pub fn run_case_in_child(
    child_argv0: &str,
    case: &str,
    timeout: Duration,
    extra_args: &[String],
) -> Result<ChildOutcome, QualificationError> {
    let mut command = Command::new(child_argv0);
    command
        .arg("--worker")
        .arg(case)
        .args(extra_args)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut child = command
        .spawn()
        .map_err(|e| QualificationError::Child(format!("spawn worker for {case}: {e}")))?;
    let start = Instant::now();
    loop {
        match child.try_wait() {
            Err(e) => {
                return Err(QualificationError::Child(format!(
                    "poll worker for {case}: {e}"
                )));
            }
            Ok(Some(_)) => {
                let output = child.wait_with_output().map_err(|e| {
                    QualificationError::Child(format!("reap worker for {case}: {e}"))
                })?;
                if !output.status.success() {
                    let stderr = String::from_utf8_lossy(&output.stderr);
                    let excerpt: String = stderr.chars().take(300).collect();
                    return Ok(classify_status(output.status, &excerpt));
                }
                match serde_json::from_slice::<CaseEvidence>(&output.stdout) {
                    Ok(evidence) => return Ok(ChildOutcome::Evidence(evidence)),
                    Err(e) => {
                        return Ok(ChildOutcome::Crash(format!(
                            "evidence parse after success exit: {e}"
                        )));
                    }
                }
            }
            Ok(None) => {
                if start.elapsed() >= timeout {
                    let _ = child.kill();
                    let _ = child.wait();
                    return Ok(ChildOutcome::Timeout);
                }
                std::thread::sleep(Duration::from_millis(5));
            }
        }
    }
}

/// One recorded repetition for the attempts log.
#[derive(Debug, Clone, Serialize)]
pub struct AttemptRecord {
    /// Case id.
    pub case: String,
    /// Repetition index (zero-based).
    pub repetition: u32,
    /// `evidence`, `timeout` or `crash`.
    pub outcome: String,
    /// Wall-clock seconds for this repetition.
    pub elapsed_secs: f64,
    /// Evidence, when the child produced any.
    pub evidence: Option<CaseEvidence>,
    /// Crash detail, when applicable.
    pub detail: Option<String>,
}

/// Oracle expectation used when mapping evidence to an observation.
#[derive(Debug, Clone, Copy)]
enum OracleKind {
    /// Closed-form volume oracle with relative tolerance.
    Volume(f64, f64),
    /// Closed-form volume plus whole-boundary area oracle.
    VolumeArea(f64, f64, f64),
    /// Algebraic empty set via expected refusal.
    EmptyRefusal,
    /// Algebraic empty set via zero-face solid or refusal.
    EmptyFlexible,
    /// STEP preservation: closed-form import volume plus stability bound.
    StepPreservation,
}

fn oracle_kind(case: &str) -> OracleKind {
    match case {
        "box-fuse-half-overlap" => OracleKind::Volume(1.5, 1e-6),
        "box-fuse-identical" => OracleKind::Volume(1.0, 1e-6),
        "box-cut-contained-cavity" => OracleKind::VolumeArea(7.0, 30.0, 1e-6),
        "box-cut-identical-empty" => OracleKind::EmptyRefusal,
        "box-intersect-disjoint-empty" => OracleKind::EmptyFlexible,
        _ => OracleKind::StepPreservation,
    }
}

/// Scenario oracle text: the named independent route, not kernel output.
fn oracle_text(case: &str) -> &'static str {
    match case {
        "box-fuse-half-overlap" => "closed-form: 1 + 1 - 0.5 overlap arithmetic",
        "box-fuse-identical" => "closed-form: coincident fuse preserves unit volume",
        "box-cut-contained-cavity" => {
            "closed-form: outer-minus-cavity volume 8 - 1, whole-boundary area 24 + 6"
        }
        "box-cut-identical-empty" => "algebraic empty set: A - A = {}; typed EmptyResult only",
        "box-intersect-disjoint-empty" => {
            "algebraic empty set: zero faces with ~0 volume, or typed EmptyResult"
        }
        _ => "closed-form cylinder 160*pi plus write/reimport stability",
    }
}

fn applicable_metrics(case: &str, evidence: &CaseEvidence) -> Vec<&'static str> {
    match case {
        "box-fuse-half-overlap" | "box-fuse-identical" => vec![
            "evolution_completeness",
            "tessellation_watertight",
            "volume_error",
            "runtime_median",
            "runtime_p95",
        ],
        "box-cut-contained-cavity" => vec![
            "evolution_completeness",
            "tessellation_watertight",
            "volume_error",
            "area_error",
            "runtime_median",
            "runtime_p95",
        ],
        "step-cylinder-preservation" => {
            let mut metrics = vec![
                "import_validity",
                "post_import_operation_success",
                "round_trip_geometry_fidelity",
                "tessellation_watertight",
                "volume_error",
                "runtime_median",
                "runtime_p95",
            ];
            if evidence.evolution_completeness.is_some() {
                metrics.push("evolution_completeness");
            }
            metrics
        }
        _ => vec!["runtime_median", "runtime_p95"],
    }
}

fn topology_producing(case: &str, evidence: &CaseEvidence) -> bool {
    match case {
        "box-fuse-half-overlap" | "box-fuse-identical" | "box-cut-contained-cavity" => true,
        "step-cylinder-preservation" => evidence.evolution_completeness.is_some(),
        _ => false,
    }
}

fn quality_block() -> serde_json::Value {
    serde_json::json!({
        "representation": "exact",
        "deflection": DEFLECTION,
        "tolerance_model": TOLERANCE_MODEL,
        "error_budget": 0.0,
    })
}

fn null_metric_groups() -> BTreeMap<String, serde_json::Value> {
    let groups: &[(&str, &[&str])] = &[
        (
            "history",
            &["evolution_completeness", "persistent_ref_survival"],
        ),
        (
            "interchange",
            &[
                "import_validity",
                "post_import_operation_success",
                "round_trip_geometry_fidelity",
                "assembly_metadata_fidelity",
            ],
        ),
        (
            "geometry_quality",
            &[
                "tessellation_watertight",
                "volume_error",
                "area_error",
                "centroid_error",
                "inertia_error",
            ],
        ),
        (
            "resources",
            &[
                "runtime_median",
                "runtime_p95",
                "peak_memory",
                "entity_growth",
                "cancellation_latency",
            ],
        ),
        (
            "browser",
            &[
                "wasm_cold_init",
                "module_size_raw",
                "module_size_gzip",
                "module_size_brotli",
                "native_wasm_agreement",
            ],
        ),
        ("concurrency", &["thread_scaling_efficiency"]),
    ];
    groups
        .iter()
        .map(|(group, columns)| {
            (
                (*group).to_string(),
                serde_json::Value::Object(
                    columns
                        .iter()
                        .map(|c| ((*c).to_string(), serde_json::Value::Null))
                        .collect(),
                ),
            )
        })
        .collect()
}

fn set_metric(
    metrics: &mut BTreeMap<String, serde_json::Value>,
    group: &str,
    column: &str,
    value: serde_json::Value,
) {
    if let Some(serde_json::Value::Object(map)) = metrics.get_mut(group) {
        map.insert(column.to_string(), value);
    }
}

/// Evaluates the independent oracle against one repetition's evidence.
/// Returns `(oracle_agrees, volume_error, area_error)`; errors are `None`
/// when the evidence cannot answer the oracle.
fn evaluate_oracle(
    spec: &CaseSpec,
    evidence: &CaseEvidence,
) -> (Option<bool>, Option<f64>, Option<f64>) {
    match oracle_kind(spec.id) {
        OracleKind::Volume(expected, tol) => match evidence.volume {
            Some(v) if evidence.status == "success" => {
                let err = rel_err(v, expected);
                (Some(err <= tol), Some(err), None)
            }
            _ => (Some(false), None, None),
        },
        OracleKind::VolumeArea(expected_v, expected_a, tol) => {
            match (evidence.volume, evidence.area) {
                (Some(v), _) if evidence.status == "success" => {
                    let verr = rel_err(v, expected_v);
                    let aerr = evidence.area.map(|a| rel_err(a, expected_a));
                    let agrees = verr <= tol && aerr.is_some_and(|e| e <= tol);
                    (Some(agrees), Some(verr), aerr)
                }
                _ => (Some(false), None, None),
            }
        }
        OracleKind::EmptyRefusal => {
            if evidence.status == "refused" {
                (None, None, None)
            } else {
                // A claimed non-refusal for A - A disagrees with the empty set.
                (Some(false), None, None)
            }
        }
        OracleKind::EmptyFlexible => match evidence.status.as_str() {
            "refused" => (None, None, None),
            "empty_success" => {
                let agrees =
                    evidence.faces == Some(0) && evidence.volume.is_some_and(|v| v.abs() <= 1e-6);
                (Some(agrees), None, None)
            }
            _ => (Some(false), None, None),
        },
        OracleKind::StepPreservation => {
            const CYLINDER_VOLUME: f64 = 160.0 * std::f64::consts::PI;
            match evidence.volume {
                Some(v) if evidence.status == "success" => {
                    let import_err = rel_err(v, CYLINDER_VOLUME);
                    let stable = evidence
                        .round_trip
                        .as_ref()
                        .is_some_and(|rt| rt.volume_rel <= 1e-9);
                    (Some(import_err <= 1e-6 && stable), Some(import_err), None)
                }
                _ => (Some(false), None, None),
            }
        }
    }
}

/// Maps two repetitions plus parent-measured runtimes to one O1.2d
/// observation value.
///
/// The kernel claim selects the reported status; the independent oracle
/// verdict selects agreement. A claimed success that misses its oracle keeps
/// `reported` success with `oracle_agrees: false` (the scorecard then derives
/// `silent_wrong`); nothing is relabeled.
///
/// # Errors
///
/// Returns [`QualificationError::Incomplete`] when required evidence fields
/// are missing for the observation contract.
#[allow(clippy::too_many_lines)]
pub fn map_to_observation(
    spec: &CaseSpec,
    first: &CaseEvidence,
    second: &CaseEvidence,
    kernel: &str,
    runtime_median: f64,
    runtime_p95: f64,
    harness_sha_short: &str,
) -> Result<serde_json::Value, QualificationError> {
    let incomplete = |detail: &str| QualificationError::Incomplete {
        case: spec.id.to_string(),
        detail: detail.to_string(),
    };
    if first.status != second.status {
        return Err(incomplete("repetitions disagree on kernel claim"));
    }
    let (oracle_agrees, volume_error, area_error) = evaluate_oracle(spec, first);
    let (oracle_agrees2, _, _) = evaluate_oracle(spec, second);
    let repeat_volumes_agree = match (first.volume, second.volume) {
        (Some(a), Some(b)) => rel_err(a, b) <= 1e-9,
        (None, None) => true,
        _ => false,
    };
    let repeat_agrees = oracle_agrees == oracle_agrees2 && repeat_volumes_agree;
    let defect_repro = || {
        serde_json::Value::String(format!(
            "native-qualification:{}@{harness_sha_short}",
            spec.id
        ))
    };
    let (reported, validator_accepts, needs_repro_on_failure) = match first.status.as_str() {
        "success" => {
            let valid = first
                .valid
                .ok_or_else(|| incomplete("missing validation verdict"))?;
            ("exact_success", Some(valid), true)
        }
        "empty_success" => {
            let valid = first
                .valid
                .ok_or_else(|| incomplete("missing validation verdict"))?;
            ("exact_success", Some(valid), true)
        }
        "refused" => ("refusal", None, false),
        _ => ("error", None, false),
    };
    let diagnostic = first
        .diagnostic
        .clone()
        .map(serde_json::Value::String)
        .unwrap_or(serde_json::Value::Null);
    let metrics_applicable = applicable_metrics(spec.id, first);
    let mut metrics = null_metric_groups();
    if metrics_applicable.contains(&"runtime_median") {
        set_metric(
            &mut metrics,
            "resources",
            "runtime_median",
            serde_json::json!(require_finite(runtime_median, "runtime median", spec.id)?),
        );
        set_metric(
            &mut metrics,
            "resources",
            "runtime_p95",
            serde_json::json!(require_finite(runtime_p95, "runtime p95", spec.id)?),
        );
    }
    if metrics_applicable.contains(&"evolution_completeness") {
        let completeness = first
            .evolution_completeness
            .ok_or_else(|| incomplete("missing evolution completeness"))?;
        set_metric(
            &mut metrics,
            "history",
            "evolution_completeness",
            serde_json::json!(require_finite(
                completeness,
                "evolution completeness",
                spec.id
            )?),
        );
    }
    if metrics_applicable.contains(&"tessellation_watertight") {
        let watertight = first
            .watertight
            .ok_or_else(|| incomplete("missing watertight"))?;
        set_metric(
            &mut metrics,
            "geometry_quality",
            "tessellation_watertight",
            serde_json::Value::Bool(watertight),
        );
    }
    if metrics_applicable.contains(&"volume_error") {
        let verr = volume_error.ok_or_else(|| incomplete("missing volume error"))?;
        set_metric(
            &mut metrics,
            "geometry_quality",
            "volume_error",
            serde_json::json!(require_finite(verr, "volume error", spec.id)?),
        );
    }
    if metrics_applicable.contains(&"area_error") {
        let aerr = area_error.ok_or_else(|| incomplete("missing area error"))?;
        set_metric(
            &mut metrics,
            "geometry_quality",
            "area_error",
            serde_json::json!(require_finite(aerr, "area error", spec.id)?),
        );
    }
    if metrics_applicable.contains(&"import_validity") {
        let validity = first
            .import_validity
            .ok_or_else(|| incomplete("missing import validity"))?;
        set_metric(
            &mut metrics,
            "interchange",
            "import_validity",
            serde_json::Value::Bool(validity),
        );
        let post = first
            .post_import_success
            .ok_or_else(|| incomplete("missing post-import success"))?;
        set_metric(
            &mut metrics,
            "interchange",
            "post_import_operation_success",
            serde_json::Value::Bool(post),
        );
        let rt = first
            .round_trip
            .clone()
            .ok_or_else(|| incomplete("missing round-trip"))?;
        for (field, value) in [
            ("volume_rel", rt.volume_rel),
            ("area_rel", rt.area_rel),
            ("centroid_abs", rt.centroid_abs),
            ("bounds_abs", rt.bounds_abs),
        ] {
            require_finite(value, field, spec.id)?;
            if value < 0.0 {
                return Err(incomplete("negative round-trip error"));
            }
        }
        set_metric(
            &mut metrics,
            "interchange",
            "round_trip_geometry_fidelity",
            serde_json::json!({
                "volume_error": rt.volume_rel,
                "area_error": rt.area_rel,
                "centroid_error": rt.centroid_abs,
                "bounds_error": rt.bounds_abs,
            }),
        );
    }
    // A failed absolute gate needs a permanent minimized repro reference;
    // the case identity plus source SHA replays it exactly.
    let refusal_undiagnosed = first.status == "refused"
        && first
            .diagnostic
            .as_deref()
            .is_none_or(|d| d.trim().is_empty());
    let gate_fails = oracle_agrees == Some(false)
        || validator_accepts == Some(false)
        || !repeat_agrees
        || refusal_undiagnosed
        || (needs_repro_on_failure && first.status == "error");
    Ok(serde_json::json!({
        "scenario": spec.id,
        "kernel": kernel,
        "reported": reported,
        "diagnostic": diagnostic,
        "oracle_agrees": oracle_agrees,
        "validator_accepts": validator_accepts,
        "repeat_agrees": repeat_agrees,
        "quality": quality_block(),
        "approximation": serde_json::Value::Null,
        "repairs": [],
        "repair_occurred": false,
        "evolution_explicit": first.evolution_completeness.is_some_and(|c| c >= 1.0)
            || !topology_producing(spec.id, first),
        "defect_repro": if gate_fails { defect_repro() } else { serde_json::Value::Null },
        "metrics": metrics,
    }))
}

/// Scenario block for the job: oracle text, declared quality, applicable
/// metrics and topology flags.
#[must_use]
pub fn scenario_block(spec: &CaseSpec, representative: &CaseEvidence) -> serde_json::Value {
    serde_json::json!({
        "id": spec.id,
        "oracle": oracle_text(spec.id),
        "quality": quality_block(),
        "applicable_metrics": applicable_metrics(spec.id, representative),
        "topology_producing": topology_producing(spec.id, representative),
        "native_wasm_required": false,
    })
}

/// Computes the SHA-256 of the referenced gauntlet manifest.
///
/// # Errors
///
/// Returns [`QualificationError::Child`] when the manifest cannot be read.
/// (Categorised as harness failure: without manifest identity no report may
/// be emitted.)
pub fn manifest_sha256() -> Result<String, QualificationError> {
    let path = format!("{}/../../{MANIFEST_NAME}", env!("CARGO_MANIFEST_DIR"));
    let bytes = std::fs::read(&path)
        .map_err(|e| QualificationError::Child(format!("read {MANIFEST_NAME}: {e}")))?;
    let digest = <sha2::Sha256 as sha2::Digest>::digest(&bytes);
    let mut hex = String::with_capacity(digest.len() * 2);
    for byte in digest {
        hex.push(char::from(HEX_NIBBLES[usize::from(byte >> 4)]));
        hex.push(char::from(HEX_NIBBLES[usize::from(byte & 0x0f)]));
    }
    Ok(hex)
}

/// Validates a full-length commit SHA for the harness identity.
///
/// # Errors
///
/// Returns [`QualificationError::Child`] for anything that is not 40 hex
/// digits.
pub fn check_harness_sha(sha: &str) -> Result<(), QualificationError> {
    let ok = sha.len() == 40 && sha.bytes().all(|b| b.is_ascii_hexdigit());
    if ok {
        Ok(())
    } else {
        Err(QualificationError::Child(format!(
            "harness SHA must be 40 hex digits, got {sha:?}"
        )))
    }
}

/// Assembles the complete O1.2d job from mapped observations.
///
/// Each entry pairs a case with its first-repetition evidence so the
/// scenario block declares the same applicable metrics and topology flags
/// the observations were measured against.
///
/// # Errors
///
/// Returns [`QualificationError::Incomplete`] unless every declared case has
/// exactly one observation for the single native kernel.
pub fn assemble_job(
    specs: &[(&CaseSpec, &CaseEvidence)],
    observations: Vec<serde_json::Value>,
    run_id: &str,
    harness_sha: &str,
    manifest_sha: &str,
    kernel: &str,
    repetitions: u32,
) -> Result<serde_json::Value, QualificationError> {
    check_harness_sha(harness_sha)?;
    if observations.len() != specs.len() {
        return Err(QualificationError::Incomplete {
            case: "<job>".to_string(),
            detail: format!(
                "expected {} observations, got {}",
                specs.len(),
                observations.len()
            ),
        });
    }
    let mut seen = BTreeMap::new();
    for observation in &observations {
        let scenario = observation
            .get("scenario")
            .and_then(serde_json::Value::as_str)
            .unwrap_or_default();
        let obs_kernel = observation
            .get("kernel")
            .and_then(serde_json::Value::as_str)
            .unwrap_or_default();
        if obs_kernel != kernel || !specs.iter().any(|(s, _)| s.id == scenario) {
            return Err(QualificationError::Incomplete {
                case: "<job>".to_string(),
                detail: format!("unexpected observation {scenario}/{obs_kernel}"),
            });
        }
        seen.insert(scenario.to_string(), 1_u32);
    }
    if seen.len() != specs.len() {
        return Err(QualificationError::Incomplete {
            case: "<job>".to_string(),
            detail: "duplicate or missing scenario observations".to_string(),
        });
    }
    Ok(serde_json::json!({
        "schema_version": SCHEMA_VERSION,
        "run_id": run_id,
        "repetitions": repetitions,
        "harness_sha": harness_sha,
        "manifest_sha256": manifest_sha,
        "kernels": [kernel],
        "scenarios": specs.iter().map(|(s, evidence)| scenario_block(s, evidence)).collect::<Vec<_>>(),
        "observations": observations,
    }))
}

/// Writes `value` as pretty JSON to `path`.
///
/// # Errors
///
/// Returns [`QualificationError::Serialise`] on IO or serialisation failure.
pub fn write_json(path: &str, value: &serde_json::Value) -> Result<(), QualificationError> {
    let mut file =
        std::fs::File::create(path).map_err(|e| QualificationError::Serialise(e.to_string()))?;
    serde_json::to_writer_pretty(&mut file, value)
        .map_err(|e| QualificationError::Serialise(e.to_string()))?;
    file.write_all(b"\n")
        .map_err(|e| QualificationError::Serialise(e.to_string()))?;
    Ok(())
}
