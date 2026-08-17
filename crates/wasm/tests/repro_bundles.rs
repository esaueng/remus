//! Replays every committed reproduction bundle under `tests/repro/`.
//!
//! Each `.json` file is a versioned repro bundle (see `remus_wasm::repro`).
//! Adding a regression means adding a bundle file; this suite picks it up
//! with no code change.

#![cfg(not(target_arch = "wasm32"))]
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_wasm::repro::ReproBundle;

#[test]
fn all_committed_bundles_replay() {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/repro");
    let mut replayed = Vec::new();
    let mut entries: Vec<_> = std::fs::read_dir(dir)
        .expect("tests/repro must exist")
        .map(|e| e.expect("readable dir entry").path())
        .filter(|p| p.extension().is_some_and(|ext| ext == "json"))
        .collect();
    entries.sort();

    for path in entries {
        let json = std::fs::read_to_string(&path).expect("readable bundle");
        let bundle =
            ReproBundle::from_json(&json).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        bundle
            .run()
            .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        replayed.push(path);
    }

    assert!(
        replayed.len() >= 2,
        "expected at least the two seed bundles, replayed {replayed:?}"
    );
}
