//! P-Class 8.1 execution-surface cross-check: native vs batch contract.
//!
//! Every committed `opseq81-*.json` bundle replays natively (see
//! `crates/operations/tests/op_seq_81.rs`, whose finding tests pin
//! the verdicts) and here through the actual `executeBatchV2` dispatch path
//! — the same contract the built WASM package serves. For each bundle the
//! test appends read-only probes (`validateSolid`, `volume`, `meshQuality`)
//! on the final produced solid and checks the pinned acceptance or defect signature:
//!
//! - fixed orientation findings F1/F2/F5/F11/F12/F16 must validate clean and mesh
//!   watertight with their closed-form volumes through batch.
//! - open orientation finding F10 must show validation errors through
//!   batch with matching volumes: an engine defect, not a binding one. A
//!   batch-side refusal or a clean validation would mean the defect moved
//!   or the binding diverges — either fails loudly here.
//! - fixed material findings F3/F6/F8 must retain their independently derived
//!   remainders and mesh watertight through batch.
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
                volume_near: (std::f64::consts::PI, 1e-9),
                validate: ValidateExpect::Clean,
                watertight: Some(true),
            },
        },
        CrossCheck {
            file: "opseq81-coax-fuse-orientation.json",
            signature: Signature::Probes {
                volume_near: (2.75 * std::f64::consts::PI, 1e-9),
                validate: ValidateExpect::Clean,
                watertight: Some(true),
            },
        },
        CrossCheck {
            file: "opseq81-rotated-cut-material.json",
            signature: Signature::Probes {
                volume_near: (30.240_105_421_226_68, 1e-4),
                validate: ValidateExpect::Clean,
                watertight: Some(true),
            },
        },
        CrossCheck {
            file: "opseq81-box-cyl-cut-orientation.json",
            signature: Signature::Probes {
                // Cylinder minus the first-quadrant rectangle [0,1.5] x [0,1].
                // The overlap area is sqrt(5)/4 + 9 asin(2/3)/8.
                volume_near: (
                    6.5 * (2.25 * std::f64::consts::PI
                        - (5.0_f64.sqrt() / 4.0 + 1.125 * (2.0_f64 / 3.0).asin())),
                    1e-4,
                ),
                validate: ValidateExpect::Clean,
                watertight: Some(true),
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
                volume_near: (
                    7.0 * std::f64::consts::PI / 6.0,
                    1e-4 * 7.0 * std::f64::consts::PI / 6.0,
                ),
                validate: ValidateExpect::Clean,
                watertight: Some(true),
            },
        },
        CrossCheck {
            file: "opseq81-inward-offset-fuse-untyped.json",
            signature: Signature::TerminalError { op_index: 3 },
        },
        CrossCheck {
            file: "opseq81-sphere-box-fuse-mesh.json",
            signature: Signature::Probes {
                volume_near: (
                    1.0 + 7.0 * std::f64::consts::PI / 6.0,
                    1e-4 * (1.0 + 7.0 * std::f64::consts::PI / 6.0),
                ),
                validate: ValidateExpect::Clean,
                watertight: Some(true),
            },
        },
        CrossCheck {
            file: "opseq81-pointed-cone-offset-mesh.json",
            signature: Signature::Probes {
                volume_near: (
                    std::f64::consts::PI * (1.25 + std::f64::consts::SQRT_2 / 4.0).powi(3) / 3.0,
                    1e-6,
                ),
                validate: ValidateExpect::Clean,
                watertight: Some(true),
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
                // Height-2 crescent: r6 stock minus r8 tool at distance sqrt5.
                // Independent circle-sector overlap, not a kernel reading.
                volume_near: (
                    2.0 * (36.0 * (23.0 / (12.0 * 5.0_f64.sqrt())).acos()
                        - 64.0 * (33.0 / (16.0 * 5.0_f64.sqrt())).acos()
                        + 191.0_f64.sqrt() / 2.0),
                    1e-4,
                ),
                validate: ValidateExpect::Clean,
                watertight: Some(true),
            },
        },
        CrossCheck {
            file: "opseq81-pointed-cone-fuse-orientation.json",
            signature: Signature::Probes {
                // A placed unit cylinder and a disjoint unit apex-down cone:
                // pi + pi/3 exactly. The batch `volume` probe meshes this
                // two-lump shell and reads 4.18730 (-3.6e-4).
                volume_near: (std::f64::consts::PI * 4.0 / 3.0, 2e-3),
                validate: ValidateExpect::Clean,
                watertight: Some(true),
            },
        },
        CrossCheck {
            file: "opseq81-placed-sphere-box-cut-mesh.json",
            signature: Signature::Probes {
                volume_near: (112.711_520_498_865_48, 1e-4),
                validate: ValidateExpect::Clean,
                watertight: Some(true),
            },
        },
        CrossCheck {
            file: "opseq81-frustum-cone-offset-mesh.json",
            signature: Signature::Probes {
                volume_near: (
                    {
                        let a = 4.0 + (13.0_f64.sqrt() + 2.0) / 12.0;
                        let b = 2.0 + (13.0_f64.sqrt() - 2.0) / 12.0;
                        std::f64::consts::PI * 3.5 * (a * a + a * b + b * b) / 3.0
                    },
                    2e-5,
                ),
                validate: ValidateExpect::Clean,
                watertight: Some(true),
            },
        },
        CrossCheck {
            file: "opseq81-cylinder-offset-mesh.json",
            signature: Signature::Probes {
                volume_near: (9.0 * std::f64::consts::PI / 32.0, 1e-9),
                validate: ValidateExpect::Clean,
                watertight: Some(true),
            },
        },
        CrossCheck {
            file: "opseq81-mirrored-box-intersect-orientation.json",
            signature: Signature::Probes {
                // The quarter of the unit disc in x >= 0, y <= 0, one tall.
                volume_near: (std::f64::consts::FRAC_PI_4, 1e-9),
                validate: ValidateExpect::Clean,
                watertight: Some(true),
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

#[test]
fn f13_both_batches_keep_holed_sphere_mesh_closed_on_cache_replay() {
    let bundle = ReproBundle::from_json(include_str!(
        "repro/opseq81-placed-sphere-box-cut-mesh.json"
    ))
    .unwrap();
    for v2 in [false, true] {
        for (operation, reverse, expected) in [
            ("cut", false, 112.711_520_498_865_48),
            ("fuse", false, 113.711_520_498_865_48),
            ("fuse", true, 113.711_520_498_865_48),
            ("intersect", false, 0.385_815_030_367_076_43),
        ] {
            let mut kernel = remus_wasm::kernel::BrepKernel::new();
            let mut ops = bundle.operations.clone();
            ops[3]["args"]["operation"] = serde_json::json!(operation);
            if reverse {
                ops[3]["args"]["solidA"] = serde_json::json!(0);
                ops[3]["args"]["solidB"] = serde_json::json!(1);
            }
            let run = |kernel: &mut remus_wasm::kernel::BrepKernel, request: &str| {
                if v2 {
                    kernel.execute_batch_v2(request)
                } else {
                    kernel.execute_batch(request)
                }
            };
            let prefix: Vec<Value> =
                serde_json::from_str(&run(&mut kernel, &serde_json::to_string(&ops).unwrap()))
                    .unwrap();
            assert!(
                prefix.iter().all(|cell| cell.get("ok").is_some()),
                "{prefix:?}"
            );
            assert_eq!(prefix[3]["ok"]["quality"], "exact");
            let solid = produced_handle(&prefix[3]).unwrap();
            for deflection in [0.1, 0.01, 0.003] {
                let probes = serde_json::json!([
                    {"op":"validateSolid","args":{"solid":solid}},
                    {"op":"volume","args":{"solid":solid,"deflection":0.0001}},
                    {"op":"meshQuality","args":{"solid":solid,"deflection":deflection}}
                ])
                .to_string();
                let cold = run(&mut kernel, &probes);
                let warm = run(&mut kernel, &probes);
                assert_eq!(cold, warm, "F13 {operation} batchv2={v2}: cache replay");
                let cells: Vec<Value> = serde_json::from_str(&cold).unwrap();
                assert_eq!(cells[0]["ok"], 0);
                assert!((cells[1]["ok"].as_f64().unwrap() - expected).abs() < expected * 2e-4);
                assert_eq!(
                    cells[2]["ok"]["isWatertight"], true,
                    "F13 {operation} d={deflection}: {cells:?}"
                );
                assert_eq!(cells[2]["ok"]["boundaryEdges"], 0);
                assert_eq!(cells[2]["ok"]["nonManifoldEdges"], 0);
            }
        }
    }
}
