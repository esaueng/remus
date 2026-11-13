//! Session snapshot contracts: handles, stale-handle refusals, the journal
//! and later edits behave identically in a restored kernel, and every
//! malformed snapshot is refused before a kernel exists.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout
)]

use serde_json::{Value, json};

use crate::kernel::RemusKernel;

fn run(k: &mut RemusKernel, ops: &Value) -> Vec<Value> {
    let out: Value = serde_json::from_str(&k.execute_batch(&ops.to_string())).unwrap();
    out.as_array().unwrap().clone()
}

fn ok(k: &mut RemusKernel, op: &str, args: &Value) -> Value {
    let out = run(k, &json!([{"op": op, "args": args}])).remove(0);
    assert!(out.get("ok").is_some(), "{op} {args}: {out}");
    out["ok"].clone()
}

fn handle(value: &Value) -> u32 {
    u32::try_from(value.as_u64().unwrap()).unwrap()
}

fn snapshot(k: &RemusKernel) -> Vec<u8> {
    k.snapshot_impl().unwrap()
}

fn restore(bytes: &[u8]) -> RemusKernel {
    RemusKernel::from_snapshot_impl(bytes).unwrap()
}

/// Every evolution query a consumer reads after an operation.
fn evolution(k: &mut RemusKernel, op: u64) -> Value {
    let mut outputs = Vec::new();
    for kind in ["face", "edge", "vertex"] {
        for index in 0..64 {
            outputs.push(
                run(
                    k,
                    &json!([{"op": "resolveOperationOutput",
                        "args": {"op": op, "kind": kind, "index": index}}]),
                )
                .remove(0),
            );
        }
    }
    let summary = ok(k, "journalSummary", &json!({}));
    json!({"summary": summary, "outputs": outputs})
}

/// A session with history: journaled booleans, a named face, a checkpoint
/// restore that retires a box and a GCS sketch, and an unjournaled edit.
fn session() -> (RemusKernel, Value) {
    let mut k = RemusKernel::new();
    let block = handle(&ok(
        &mut k,
        "makeBox",
        &json!({"width": 10.0, "height": 10.0, "depth": 10.0}),
    ));
    let post = handle(&ok(
        &mut k,
        "makeCylinder",
        &json!({"radius": 3.0, "height": 15.0}),
    ));
    let fused = ok(
        &mut k,
        "fuseJournaled",
        &json!({"solidA": block, "solidB": post}),
    );
    let solid = handle(&fused["solid"]);
    let faces = ok(&mut k, "getSolidFaces", &json!({"solid": solid}));
    ok(
        &mut k,
        "setFaceName",
        &json!({"face": faces[0], "name": "datum"}),
    );

    let checkpoint = k.checkpoint_impl().unwrap();
    let doomed = handle(&ok(
        &mut k,
        "makeBox",
        &json!({"width": 1.0, "height": 1.0, "depth": 1.0}),
    ));
    let sketch = handle(&ok(&mut k, "gcsNew", &json!({})));
    k.restore_checkpoint_impl(checkpoint).unwrap();
    assert!(k.resolve_solid(doomed).is_err());
    ok(&mut k, "makeSphere", &json!({"radius": 2.0}));
    let state = json!({
        "solid": solid, "doomed": doomed, "sketch": sketch,
        "checkpoint": checkpoint, "fuseOp": fused["op"],
    });
    (k, state)
}

#[test]
fn round_trip_preserves_handles_counters_and_stale_refusals() {
    let (mut original, state) = session();
    let bytes = snapshot(&original);
    assert_eq!(bytes, snapshot(&original), "snapshots are deterministic");
    let mut restored = restore(&bytes);
    assert_eq!(
        snapshot(&restored),
        bytes,
        "a restored kernel snapshots to the same bytes"
    );

    for k in [&original, &restored] {
        assert!(k.resolve_solid(handle(&state["solid"])).is_ok());
        assert!(
            k.resolve_solid(handle(&state["doomed"])).is_err(),
            "retired stays stale"
        );
    }
    // The restored kernel carries no checkpoint contents, only the counter.
    assert!(
        restored
            .restore_checkpoint_impl(handle(&state["checkpoint"]))
            .is_err()
    );
    assert_eq!(
        restored.checkpoint_impl().unwrap(),
        original.checkpoint_impl().unwrap(),
        "the next checkpoint takes the same id"
    );

    // The next allocation of every kind takes the same handle.
    for (op, args) in [
        (
            "makeBox",
            json!({"width": 2.0, "height": 3.0, "depth": 4.0}),
        ),
        ("gcsNew", json!({})),
    ] {
        let a = ok(&mut original, op, &args);
        let b = ok(&mut restored, op, &args);
        assert_eq!(a, b, "{op}");
    }
    assert_eq!(
        ok(&mut restored, "gcsNew", &json!({})),
        json!(handle(&state["sketch"]) + 2),
        "the retired sketch handle is never reissued"
    );
    ok(&mut original, "gcsNew", &json!({}));

    let fuse_op = state["fuseOp"].as_u64().unwrap();
    assert_eq!(
        evolution(&mut original, fuse_op),
        evolution(&mut restored, fuse_op)
    );
}

#[test]
fn journaled_edits_after_restore_match_the_original_byte_for_byte() {
    let (mut original, state) = session();
    let mut restored = restore(&snapshot(&original));
    let solid = handle(&state["solid"]);
    let mut ops = Vec::new();
    for k in [&mut original, &mut restored] {
        let tool = handle(&ok(
            k,
            "makeBox",
            &json!({"width": 4.0, "height": 4.0, "depth": 4.0}),
        ));
        let edges = ok(k, "solidEdges", &json!({"solid": tool}));
        let filleted = ok(
            k,
            "filletJournaled",
            &json!({"solid": tool, "edges": [edges[0]], "radius": 0.5}),
        );
        let rounded = handle(&filleted["solid"]);
        let faces = ok(k, "getSolidFaces", &json!({"solid": rounded}));
        let planar: Vec<u32> = faces
            .as_array()
            .unwrap()
            .iter()
            .map(handle)
            .filter(|&face| {
                let id = k.topo().face_id_from_index(face as usize).unwrap();
                k.topo().face(id).unwrap().surface().is_planar()
            })
            .collect();
        let moved = ok(
            k,
            "moveFacesJournaled",
            &json!({"solid": rounded, "faces": [planar[planar.len() - 1]], "distance": 0.25}),
        );
        // Edit the restored history too: the fused body's top cap.
        let top = ok(k, "getSolidFaces", &json!({"solid": solid}))
            .as_array()
            .unwrap()
            .iter()
            .map(handle)
            .find(|&face| {
                let id = k.topo().face_id_from_index(face as usize).unwrap();
                matches!(k.topo().face(id).unwrap().surface(),
                    remus_topology::face::FaceSurface::Plane { normal, d }
                        if normal.z() > 0.99 && (d - 10.0).abs() < 1e-9)
            })
            .unwrap();
        let raised = ok(
            k,
            "moveFacesJournaled",
            &json!({"solid": solid, "faces": [top], "distance": 1.5}),
        );
        let volume = ok(
            k,
            "volume",
            &json!({"solid": handle(&raised["solid"]), "deflection": 0.01}),
        );
        ops.push((filleted, moved, raised, volume));
    }
    assert_eq!(ops[0], ops[1], "results and their handles agree");
    for op in [&ops[0].0, &ops[0].1, &ops[0].2].map(|result| result["op"].as_u64().unwrap()) {
        assert_eq!(evolution(&mut original, op), evolution(&mut restored, op));
    }
    assert_eq!(
        snapshot(&original),
        snapshot(&restored),
        "the whole session, journal included, is identical after the edits"
    );
}

/// A restored kernel's handle stores start past the carried counters; their
/// checkpoint restores still retire later handles and never reissue them.
#[test]
fn restored_handle_stores_keep_checkpoint_semantics() {
    let (original, state) = session();
    let mut k = restore(&snapshot(&original));
    let retired = handle(&state["sketch"]);
    let point = json!({"sketch": retired, "x": 0.0, "y": 0.0, "fixed": false});
    assert!(
        run(&mut k, &json!([{"op": "gcsAddPoint", "args": point}]))[0]
            .get("error")
            .is_some()
    );

    let checkpoint = k.checkpoint_impl().unwrap();
    let first = handle(&ok(&mut k, "gcsNew", &json!({})));
    assert_eq!(first, retired + 1);
    ok(
        &mut k,
        "gcsAddPoint",
        &json!({"sketch": first, "x": 1.0, "y": 2.0, "fixed": true}),
    );
    k.restore_checkpoint_impl(checkpoint).unwrap();
    let point = json!({"sketch": first, "x": 0.0, "y": 0.0, "fixed": false});
    assert!(
        run(&mut k, &json!([{"op": "gcsAddPoint", "args": point}]))[0]
            .get("error")
            .is_some()
    );
    let second = handle(&ok(&mut k, "gcsNew", &json!({})));
    assert_eq!(second, first + 1, "a retired handle is never reissued");
    ok(
        &mut k,
        "gcsAddPoint",
        &json!({"sketch": second, "x": 1.0, "y": 2.0, "fixed": true}),
    );
    let error = k.snapshot_impl().unwrap_err();
    assert_eq!(error.code(), "transient_state");
}

#[test]
fn transient_and_poisoned_sessions_are_refused() {
    let mut k = RemusKernel::new();
    ok(
        &mut k,
        "makeBox",
        &json!({"width": 1.0, "height": 1.0, "depth": 1.0}),
    );
    ok(&mut k, "gcsNew", &json!({}));
    let error = k.snapshot_impl().unwrap_err();
    assert_eq!(error.code(), "transient_state");
    assert!(
        error.to_string().starts_with("snapshot:transient_state:"),
        "{error}"
    );

    let mut k = RemusKernel::new();
    k.poisoned = true;
    assert_eq!(k.snapshot_impl().unwrap_err().code(), "poisoned");
}

fn refused(bytes: &[u8], code: &str) {
    match RemusKernel::from_snapshot_impl(bytes) {
        Ok(_) => panic!("expected {code}"),
        Err(error) => {
            assert_eq!(error.code(), code, "{error}");
            assert!(
                error.to_string().starts_with(&format!("snapshot:{code}:")),
                "{error}"
            );
        }
    }
}

#[test]
fn malformed_snapshots_are_refused_atomically() {
    let (original, _) = session();
    let bytes = snapshot(&original);
    let before = snapshot(&original);

    refused(b"definitely not a snapshot", "not_a_snapshot");
    refused(&bytes[..bytes.len() / 2], "corrupt");
    refused(&bytes[..bytes.len() - 1], "corrupt");
    let mut flipped = bytes.clone();
    let middle = flipped.len() / 2;
    flipped[middle] ^= 0x01;
    refused(&flipped, "corrupt");

    let mut version = bytes.clone();
    version[8] = version[8].wrapping_add(1);
    refused(&version, "unsupported_format");

    // Another build: same bytes, different identity, valid digest.
    let build = super::snapshot::build_identity();
    let foreign = build.replace("remus-wasm", "remus-wasX");
    let mut other = bytes.clone();
    other.splice(13..13 + build.len(), foreign.bytes());
    let content = other.len() - 16;
    let digest = remus_io::snapshot::snapshot_digest(&bytes).unwrap();
    assert_eq!(
        RemusKernel::snapshot_digest(&bytes).ok(),
        Some(digest.to_hex())
    );
    reseal(&mut other, content);
    refused(&other, "build_mismatch");

    // Refusals leave the source untouched.
    assert_eq!(snapshot(&original), before);
}

/// Recomputes the trailer the way the writer does, through a fresh write of
/// the same layout: the digest of every byte before it.
fn reseal(bytes: &mut [u8], content: usize) {
    let digest = digest_of(&bytes[..content]);
    bytes[content..content + 8].copy_from_slice(&digest[0].to_le_bytes());
    bytes[content + 8..].copy_from_slice(&digest[1].to_le_bytes());
}

/// The snapshot digest (FNV-1a 64, SplitMix64 word chain), written out
/// independently of the io crate.
fn digest_of(content: &[u8]) -> [u64; 2] {
    let mut fnv = 0xcbf2_9ce4_8422_2325_u64;
    for &byte in content {
        fnv ^= u64::from(byte);
        fnv = fnv.wrapping_mul(0x0000_0100_0000_01b3);
    }
    let mix = |state: u64| {
        let mut z = state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    };
    let mut mixed = 0x243f_6a88_85a3_08d3_u64 ^ content.len() as u64;
    for chunk in content.chunks(8) {
        let mut word = [0u8; 8];
        word[..chunk.len()].copy_from_slice(chunk);
        mixed = mix(mixed ^ u64::from_le_bytes(word));
    }
    [fnv, mixed]
}

#[cfg(feature = "io")]
mod gecko {
    use super::*;
    use crate::handles::{face_id_to_u32, solid_id_to_u32};
    use remus_topology::explorer::solid_faces;
    use remus_topology::face::FaceSurface;

    fn fixture(k: &mut RemusKernel) -> u32 {
        let solid = remus_io::step::reader::read_step(
            include_str!("../../../operations/tests/data/gentle_gecko_filleted.step"),
            k.topo_mut(),
        )
        .unwrap()[0];
        let keys = remus_operations::journal_ops::solid_entity_keys(k.topo(), solid).unwrap();
        let pending = k.topo_mut().journal_begin("import_step");
        let mut draft = remus_topology::journal::EvolutionDraft::construction();
        for key in keys {
            draft.push(
                key,
                remus_topology::journal::EventDraft::Generated { sources: vec![] },
            );
        }
        k.topo_mut()
            .journal_record_evolution(pending, draft)
            .unwrap();
        solid_id_to_u32(solid)
    }

    fn face(k: &RemusKernel, solid: u32, pick: impl Fn(&FaceSurface) -> bool) -> u32 {
        face_id_to_u32(
            solid_faces(k.topo(), k.resolve_solid(solid).unwrap())
                .unwrap()
                .into_iter()
                .find(|&f| pick(k.topo().face(f).unwrap().surface()))
                .unwrap(),
        )
    }

    /// The planar cap at height `z`.
    fn cap(z: f64) -> impl Fn(&FaceSurface) -> bool {
        move |surface| {
            matches!(surface, FaceSurface::Plane { normal, d }
                if normal.z().abs() > 0.99 && (d.abs() - z).abs() < 1e-7)
        }
    }

    fn boss(surface: &FaceSurface) -> bool {
        matches!(surface, FaceSurface::Cylinder(c)
            if c.axis().z().abs() > 0.99 && c.origin().x().abs() < 1e-7 && c.origin().y().abs() < 1e-7)
    }

    /// A filleted STEP body with journaled edits, checkpoint-retired
    /// previews, then identical edits in the original and restored kernels.
    #[test]
    fn gecko_history_survives_a_snapshot() {
        let mut original = RemusKernel::new();
        let source = fixture(&mut original);
        let top_face = face(&original, source, cap(36.0));
        let moved = ok(
            &mut original,
            "moveFacesJournaled",
            &json!({"solid": source, "faces": [top_face], "distance": 2.0}),
        );
        let solid = handle(&moved["solid"]);
        // A drag preview: applied, then rolled back to a checkpoint.
        let checkpoint = original.checkpoint_impl().unwrap();
        let preview_face = face(&original, solid, cap(38.0));
        ok(
            &mut original,
            "moveFacesJournaled",
            &json!({"solid": solid, "faces": [preview_face], "distance": -1.0}),
        );
        original.restore_checkpoint_impl(checkpoint).unwrap();

        let bytes = snapshot(&original);
        let mut restored = restore(&bytes);
        assert_eq!(snapshot(&restored), bytes);

        let mut results = Vec::new();
        for k in [&mut original, &mut restored] {
            let boss_face = face(k, solid, boss);
            let resized = ok(
                k,
                "resizeCylindricalFaceJournaled",
                &json!({"solid": solid, "face": boss_face, "radius": 7.3}),
            );
            let next = handle(&resized["solid"]);
            let top_face = face(k, next, cap(38.0));
            let lowered = ok(
                k,
                "moveFacesJournaled",
                &json!({"solid": next, "faces": [top_face], "distance": -0.5}),
            );
            let volume = ok(
                k,
                "volume",
                &json!({"solid": handle(&lowered["solid"]), "deflection": 0.01}),
            );
            results.push((resized, lowered, volume));
        }
        assert_eq!(results[0], results[1]);
        for op in [
            moved["op"].as_u64().unwrap(),
            results[0].0["op"].as_u64().unwrap(),
            results[0].1["op"].as_u64().unwrap(),
        ] {
            assert_eq!(evolution(&mut original, op), evolution(&mut restored, op));
        }
        assert_eq!(snapshot(&original), snapshot(&restored));
    }

    /// Local measurements: `cargo test -p remus-wasm --release
    /// snapshot_measurements -- --ignored --nocapture`.
    #[test]
    #[ignore = "measurement; prints sizes and timings"]
    fn snapshot_measurements() {
        let build = std::time::Instant::now();
        let mut k = RemusKernel::new();
        let mut solid = fixture(&mut k);
        let mut height = 36.0;
        for distance in [2.0, -1.0, 0.5] {
            let top_face = face(&k, solid, cap(height));
            height += distance;
            let moved = ok(
                &mut k,
                "moveFacesJournaled",
                &json!({"solid": solid, "faces": [top_face], "distance": distance}),
            );
            solid = handle(&moved["solid"]);
            let checkpoint = k.checkpoint_impl().unwrap();
            let top_face = face(&k, solid, cap(height));
            ok(
                &mut k,
                "moveFacesJournaled",
                &json!({"solid": solid, "faces": [top_face], "distance": 0.1}),
            );
            k.restore_checkpoint_impl(checkpoint).unwrap();
        }
        let boss_face = face(&k, solid, boss);
        let resized = ok(
            &mut k,
            "resizeCylindricalFaceJournaled",
            &json!({"solid": solid, "face": boss_face, "radius": 7.3}),
        );
        let _ = resized;
        let history = build.elapsed();
        let rounds = 20;
        let start = std::time::Instant::now();
        let mut bytes = Vec::new();
        for _ in 0..rounds {
            bytes = snapshot(&k);
        }
        let write = start.elapsed() / rounds;
        let start = std::time::Instant::now();
        for _ in 0..rounds {
            drop(restore(&bytes));
        }
        let read = start.elapsed() / rounds;
        // For restoring the same bytes in the browser module.
        if let Some(path) = std::env::var_os("REMUS_SNAPSHOT_DUMP") {
            std::fs::write(path, &bytes).unwrap();
        }
        let topo = k.topo();
        println!(
            "snapshot: {} bytes, {} faces / {} edges / {} vertices live of {} / {} / {} slots, {} journal entries; history {history:?}, write {write:?}, restore {read:?}",
            bytes.len(),
            topo.num_faces(),
            topo.num_edges(),
            topo.num_vertices(),
            topo.faces().slot_len(),
            topo.edges().slot_len(),
            topo.vertices().slot_len(),
            topo.journal().len(),
        );
    }
}
