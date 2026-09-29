//! P-Class 8.1 execution-surface cross-check: native vs batch contract.
//!
//! Every committed `opseq81-*.json` bundle replays natively (see
//! `crates/operations/tests/op_seq_81.rs`, whose ignored finding tests pin
//! the verdicts) and here through the actual `executeBatchV2` dispatch path
//! — the same contract the built WASM package serves. For each bundle the
//! test appends read-only probes (`validateSolid`, `volume`, `meshQuality`)
//! on the final produced solid and checks the pinned defect signature:
//!
//! - orientation findings (F1/F2/F5) must show validation errors through
//!   batch with matching volumes: an engine defect, not a binding one. A
//!   batch-side refusal or a clean validation would mean the defect moved
//!   or the binding diverges — either fails loudly here.
//! - material/mesh findings (F3/F6) must validate clean with their
//!   degenerate/open signatures intact through batch.
//!
//! A mismatch between the native verdict and the batch signature separates
//! engine defects from binding, package-provenance, and harness defects:
//! the native battery is the reference, batch is the surface under test.
//! Exact/approximate quality distinctions are preserved by pinning volumes,
//! never just success.

#![cfg(not(target_arch = "wasm32"))]
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
// Campaign diagnostics print per-bundle signatures by design; the workspace
// denies `print_stdout`, so allow it file-wide here.
#![allow(clippy::print_stdout)]

use remus_wasm::repro::ReproBundle;
use serde_json::Value;

/// Producer-op names, shared with the response parser below.
fn is_producer(op: &Value) -> bool {
    matches!(
        op.get("op").and_then(Value::as_str).unwrap_or(""),
        "makeBox"
            | "makeCylinder"
            | "makeSphere"
            | "makeCone"
            | "makeTorus"
            | "booleanWithQuality"
            | "copySolid"
            | "copyAndTransformSolid"
            | "mirror"
            | "offsetSolidV2"
    )
}

/// Read the produced handle back out of a batch response envelope: bare
/// numbers (`{"ok": 2}`) or quality envelopes (`{"ok": {"solid": 5,
/// ...}}`). Never predict when the surface tells you: some booleans
/// retain internal intermediate solids (F11's fuse lands on 5, not the
/// dense 3), so dense prediction misprobes them. Returns `None` for
/// read-only ops.
fn produced_handle(result: &Value) -> Option<u32> {
    let ok = result.get("ok")?;
    if let Some(h) = ok.as_u64() {
        return u32::try_from(h).ok();
    }
    ok.get("solid")?.as_u64()?.try_into().ok()
}

#[derive(Clone, Copy)]
enum ValidateExpect {
    /// The engine defect must be visible: nonzero error count.
    Defect,
    /// The result must validate clean.
    Clean,
}

struct CrossCheck {
    file: &'static str,
    signature: Signature,
}

enum Signature {
    /// A produced solid carries the pinned defect signature (volume,
    /// validation, watertightness).
    Probes {
        volume_near: (f64, f64),
        validate: ValidateExpect,
        /// When set, the mesh watertightness flag must equal it.
        watertight: Option<bool>,
    },
    /// The sequence ends in a batch-side error at `op_index` with every
    /// earlier op succeeding: the engine defect (here: a mistyped refusal)
    /// is visible through the binding with the same code the native
    /// executor classifies.
    TerminalError { op_index: usize },
}

/// Pinned batch signatures for the retained 8.1 findings. Volumes use the
/// native `READ_DEFLECTION` (0.1); tolerances are gross-agreement bands —
/// precise oracles live in the native battery, this file separates engine
/// from surface.
fn cross_checks() -> Vec<CrossCheck> {
    vec![
        CrossCheck {
            file: "opseq81-coax-intersect-orientation.json",
            signature: Signature::Probes {
                volume_near: (std::f64::consts::PI, 0.05),
                validate: ValidateExpect::Defect,
                watertight: None,
            },
        },
        CrossCheck {
            file: "opseq81-coax-fuse-orientation.json",
            signature: Signature::Probes {
                volume_near: (8.639_379_797, 0.05),
                validate: ValidateExpect::Defect,
                watertight: None,
            },
        },
        CrossCheck {
            file: "opseq81-rotated-cut-material.json",
            signature: Signature::Probes {
                volume_near: (0.0, 0.05),
                validate: ValidateExpect::Clean,
                watertight: Some(false),
            },
        },
        CrossCheck {
            file: "opseq81-box-cyl-cut-orientation.json",
            signature: Signature::Probes {
                volume_near: (36.98, 0.5),
                validate: ValidateExpect::Defect,
                watertight: None,
            },
        },
        CrossCheck {
            file: "opseq81-rotated-intersect-volumes.json",
            signature: Signature::Probes {
                volume_near: (32.210_818, 0.5),
                validate: ValidateExpect::Clean,
                watertight: Some(true),
            },
        },
        CrossCheck {
            file: "opseq81-sphere-minus-box-mesh.json",
            signature: Signature::Probes {
                volume_near: (3.671_139, 0.05),
                validate: ValidateExpect::Clean,
                watertight: Some(false),
            },
        },
        CrossCheck {
            file: "opseq81-inward-offset-fuse-untyped.json",
            signature: Signature::TerminalError { op_index: 3 },
        },
        CrossCheck {
            file: "opseq81-sphere-box-fuse-mesh.json",
            signature: Signature::Probes {
                // Closed-form guess 4.66519 (sphere + box − octant ball);
                // the open mesh underreads by its hole, so keep the band
                // wide: this pin separates surfaces, it is not a volume
                // oracle (that lives natively).
                volume_near: (4.665_19, 0.1),
                validate: ValidateExpect::Clean,
                watertight: Some(false),
            },
        },
        CrossCheck {
            file: "opseq81-pointed-cone-offset-mesh.json",
            signature: Signature::Probes {
                volume_near: (4.317_963, 0.05),
                validate: ValidateExpect::Clean,
                watertight: Some(false),
            },
        },
        CrossCheck {
            file: "opseq81-sphere-cyl-fuse-orientation.json",
            signature: Signature::Probes {
                // Tessellated reading; the Gauss route splits 24% lower
                // (4.18879) on the same body — see the finding doc.
                volume_near: (5.218_487, 0.05),
                validate: ValidateExpect::Defect,
                watertight: None,
            },
        },
        CrossCheck {
            file: "opseq81-translated-cyl-cut-orientation.json",
            signature: Signature::Probes {
                // Sliver remainder of the r6/h2 cylinder after the moved
                // r8/h5 tool passes through.
                volume_near: (2.011_78, 0.05),
                validate: ValidateExpect::Defect,
                watertight: None,
            },
        },
        CrossCheck {
            file: "opseq81-pointed-cone-fuse-orientation.json",
            signature: Signature::Probes {
                volume_near: (4.187_30, 0.05),
                validate: ValidateExpect::Defect,
                watertight: None,
            },
        },
        CrossCheck {
            file: "opseq81-placed-sphere-box-cut-mesh.json",
            signature: Signature::Probes {
                volume_near: (112.711_52, 0.5),
                validate: ValidateExpect::Clean,
                watertight: Some(false),
            },
        },
        CrossCheck {
            file: "opseq81-frustum-cone-offset-mesh.json",
            signature: Signature::Probes {
                volume_near: (124.764_117, 0.5),
                validate: ValidateExpect::Clean,
                // No watertight pin: the batch probe welds before counting
                // and reports this weldable-crack body clean, while the
                // native unwelded oracle fails it. That blindness IS the
                // surface result here — a coverage gap for T-junction-class
                // defects, documented in the finding.
                watertight: None,
            },
        },
        CrossCheck {
            file: "opseq81-cylinder-offset-mesh.json",
            signature: Signature::Probes {
                volume_near: (0.883_573, 0.05),
                validate: ValidateExpect::Clean,
                // Same weldable-crack nuance as F14: the batch probe welds
                // and reports clean; the native unwelded oracle fails it.
                watertight: None,
            },
        },
        CrossCheck {
            file: "opseq81-mirrored-box-intersect-orientation.json",
            signature: Signature::Probes {
                // Quarter-cylinder bite reads correctly; the defect is
                // orientation-only.
                volume_near: (std::f64::consts::FRAC_PI_4, 0.05),
                validate: ValidateExpect::Defect,
                watertight: None,
            },
        },
    ]
}

fn ok_at(results: &[Value], index: usize) -> &Value {
    results
        .get(index)
        .unwrap_or_else(|| panic!("batch returned no result for op {index}"))
}

#[test]
fn opseq81_native_batch_crosscheck() {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/repro");
    for check in cross_checks() {
        // Fresh kernel per bundle: handles are dense from zero only in a
        // fresh arena (reusing one kernel across bundles aliases solid 2
        // of bundle N onto bundle 1's result — a harness defect that once
        // read F1's pi as F2's fuse volume).
        let mut kernel = remus_wasm::kernel::BrepKernel::new();
        let path = format!("{dir}/{}", check.file);
        let json = std::fs::read_to_string(&path).unwrap_or_else(|_| {
            panic!(
                "{}: bundle must exist — commit it with the finding",
                check.file
            )
        });
        let bundle =
            ReproBundle::from_json(&json).unwrap_or_else(|e| panic!("{}: {e}", check.file));
        // Deterministic replay through the batch contract (two fresh-kernel
        // runs must agree byte-for-byte before any signature is checked).
        bundle
            .run()
            .unwrap_or_else(|e| panic!("{}: {e}", check.file));

        let ops = bundle.operations.clone();
        match check.signature {
            Signature::TerminalError { op_index } => {
                // No probes: nothing is produced past the terminal op. The
                // prefix must succeed exactly as natively; the terminal op
                // must error (never silently succeed, never crash the
                // batch); nothing may execute past it with a fabricated ok.
                let request = serde_json::to_string(&ops).expect("batch serializes");
                let response = kernel.execute_batch_v2(&request);
                let results: Vec<Value> =
                    serde_json::from_str(&response).expect("batch response must parse");
                assert_eq!(
                    results.len(),
                    ops.len(),
                    "{}: batch must answer every op",
                    check.file
                );
                for (i, result) in results.iter().enumerate().take(op_index) {
                    assert!(
                        result.get("ok").is_some(),
                        "{}: prefix op {i} errors through batch but succeeds natively: {result}",
                        check.file
                    );
                }
                let terminal = ok_at(&results, op_index);
                let code = terminal
                    .get("error")
                    .and_then(|e| e.get("code"))
                    .and_then(Value::as_str)
                    .unwrap_or_else(|| {
                        panic!(
                            "{}: terminal op must error through batch (mistyped refusal), got {terminal}",
                            check.file
                        )
                    });
                // The native executor classifies this InvalidInput as
                // untyped (outside the refusal contract); the batch code
                // must name the same invalid-argument family, proving one
                // shared mistyping rather than two divergent defects.
                assert!(
                    code.contains("invalid"),
                    "{}: terminal error code `{code}` must name the invalid-argument family",
                    check.file
                );
                println!(
                    "B81XCHECK {}: terminal error `{code}` at op {op_index}",
                    check.file
                );
                continue;
            }
            Signature::Probes { .. } => {}
        }
        let Signature::Probes {
            volume_near,
            validate,
            watertight,
        } = check.signature
        else {
            unreachable!("terminal-error bundles continue above");
        };
        // Run the prefix alone first: every op must succeed through batch
        // exactly as it does natively (a batch-side error where native
        // succeeds is a binding/package defect, never the retained engine
        // defect).
        let request = serde_json::to_string(&bundle.operations).expect("bundle serializes");
        let response = kernel.execute_batch_v2(&request);
        let prefix: Vec<Value> =
            serde_json::from_str(&response).expect("batch response must parse");
        assert_eq!(
            prefix.len(),
            bundle.operations.len(),
            "{}: batch must answer every op",
            check.file
        );
        for (i, result) in prefix.iter().enumerate() {
            assert!(
                result.get("ok").is_some(),
                "{}: op {i} errors through batch but succeeds natively: {result}",
                check.file
            );
        }
        // Read the final produced handle back out of its response envelope
        // — never predict it. Some booleans retain internal intermediate
        // solids (F11's fuse lands on 5, not the dense 3); probing a
        // predicted slot reads whatever the intermediate happens to be.
        let solid = bundle
            .operations
            .iter()
            .zip(prefix.iter())
            .filter(|(op, _)| is_producer(op))
            .filter_map(|(_, res)| produced_handle(res))
            .next_back()
            .unwrap_or_else(|| {
                panic!(
                    "{}: last producer op reports no handle through batch",
                    check.file
                )
            });
        let probes = vec![
            serde_json::json!({"op": "validateSolid", "args": {"solid": solid}}),
            serde_json::json!({"op": "volume", "args": {"solid": solid, "deflection": 0.1}}),
            serde_json::json!({"op": "meshQuality", "args": {"solid": solid, "deflection": 0.01}}),
        ];
        let request = serde_json::to_string(&probes).expect("probe batch serializes");
        let response = kernel.execute_batch_v2(&request);
        let results: Vec<Value> =
            serde_json::from_str(&response).expect("batch response must parse");
        assert_eq!(
            results.len(),
            3,
            "{}: batch must answer every probe",
            check.file
        );
        let base = 0;
        let validation = ok_at(&results, base);
        let volume = ok_at(&results, base + 1);
        let mesh = ok_at(&results, base + 2);

        match validate {
            ValidateExpect::Defect => {
                let errors = validation
                    .get("ok")
                    .and_then(Value::as_u64)
                    .unwrap_or_else(|| {
                        panic!(
                            "{}: validateSolid must return a count, got {validation}",
                            check.file
                        )
                    });
                assert!(
                    errors > 0,
                    "{}: native reports misoriented edges but batch validates clean \
                     (defect moved or binding diverges)",
                    check.file
                );
            }
            ValidateExpect::Clean => {
                assert_eq!(
                    validation.get("ok").and_then(Value::as_u64),
                    Some(0),
                    "{}: native validates clean but batch reports errors: {validation}",
                    check.file
                );
            }
        }
        let v = volume.get("ok").and_then(Value::as_f64).unwrap_or_else(|| {
            panic!(
                "{}: volume must read through batch, got {volume}",
                check.file
            )
        });
        assert!(
            (v - volume_near.0).abs() <= volume_near.1,
            "{}: batch volume {v} outside {:?} (engine/binding divergence)",
            check.file,
            volume_near
        );
        if let Some(want) = watertight {
            let got = mesh
                .get("ok")
                .and_then(|m| m.get("isWatertight"))
                .and_then(Value::as_bool)
                .unwrap_or_else(|| {
                    panic!(
                        "{}: meshQuality must report watertightness, got {mesh}",
                        check.file
                    )
                });
            assert_eq!(
                got, want,
                "{}: batch watertight={got}, native battery says {want}",
                check.file
            );
        }
        println!(
            "B81XCHECK {}: batch signature holds (volume {v:.6})",
            check.file
        );
    }
}
