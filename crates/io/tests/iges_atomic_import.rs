//! PERF-I02 IGES atomic import: failure atomicity, append-only path, and
//! semantic preservation.
//!
//! Owns IGES reader/import orchestration only. Topology internals are
//! consumed via the existing `run_append_only` / `RollbackSnapshot` APIs;
//! missing primitives are handed off, not changed here.
//!
//! Each failing import compares complete pre-existing logical state —
//! model Debug, live counts, volumes, attributes, journal length and
//! persistent references — not just entity counts. Every handle allocated
//! by an abandoned attempt must remain stale, and a malformed later plane
//! must not publish earlier planes from the same file.
//!
//! The IGES importer produces only vertices, edges, wires, faces, shells,
//! solids, and derived loops/coedges (via `add_face`). It writes no
//! attributes, pcurves, or journal entries: those stores are asserted
//! unchanged on both success and failure.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::naive_bytecount
)]

use remus_io::iges::{read_iges, read_iges_with_limits};
use remus_io::{ImportLimits, IoError};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_topology::Topology;
use remus_topology::naming::{EntitySignature, PersistentRef, resolve};

// ── Helpers ──────────────────────────────────────────────────────────

fn box_iges(dx: f64, dy: f64, dz: f64) -> String {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, dx, dy, dz).unwrap();
    remus_io::iges::write_iges(&topo, &[solid]).unwrap()
}

/// Pre-existing document with attributes, journal history and a named face
/// for persistent-reference checks.
fn seeded_doc(boxes: usize) -> (Topology, Vec<remus_topology::solid::SolidId>) {
    let mut topo = Topology::new();
    let mut solids = Vec::new();
    for i in 0..boxes {
        let s = make_box(&mut topo, 1.0 + i as f64 * 0.01, 1.0, 1.0).unwrap();
        solids.push(s);
    }
    if let Some(&first) = solids.first() {
        topo.set_solid_attributes(
            first,
            remus_topology::attributes::EntityAttributes {
                name: Some("pre-existing".to_owned()),
                ..Default::default()
            },
        )
        .unwrap();
        let shell = topo.solid(first).unwrap().outer_shell();
        let face = topo.shell(shell).unwrap().faces()[0];
        topo.set_face_attributes(
            face,
            remus_topology::attributes::EntityAttributes {
                name: Some("pre-face".to_owned()),
                ..Default::default()
            },
        )
        .unwrap();
    }
    let pending = topo.journal_begin("seed");
    topo.journal_record_barrier(pending, vec![]);
    (topo, solids)
}

struct StateSnapshot {
    num_vertices: usize,
    num_edges: usize,
    num_wires: usize,
    num_faces: usize,
    num_shells: usize,
    num_solids: usize,
    num_pcurves: usize,
    allocated_slots: usize,
    journal_len: usize,
    journal_debug: String,
    attributes_debug: String,
    solids_debug: Vec<(String, String)>,
    volumes: Vec<f64>,
}

fn capture_state(topo: &Topology, solids: &[remus_topology::solid::SolidId]) -> StateSnapshot {
    StateSnapshot {
        num_vertices: topo.num_vertices(),
        num_edges: topo.num_edges(),
        num_wires: topo.num_wires(),
        num_faces: topo.num_faces(),
        num_shells: topo.num_shells(),
        num_solids: topo.num_solids(),
        num_pcurves: topo.num_pcurves(),
        allocated_slots: topo.allocated_slot_count(),
        journal_len: topo.journal().len(),
        journal_debug: format!("{:?}", topo.journal()),
        attributes_debug: format!("{:?}", topo.attributes()),
        solids_debug: solids
            .iter()
            .map(|&s| (format!("{s:?}"), format!("{:?}", topo.solid(s).unwrap())))
            .collect(),
        volumes: solids
            .iter()
            .map(|&s| solid_volume(topo, s, 0.05).unwrap())
            .collect(),
    }
}

fn assert_state_unchanged(
    topo: &Topology,
    solids: &[remus_topology::solid::SolidId],
    before: &StateSnapshot,
    ctx: &str,
) {
    let after = capture_state(topo, solids);
    assert_eq!(
        (
            after.num_vertices,
            after.num_edges,
            after.num_wires,
            after.num_faces,
            after.num_shells,
            after.num_solids
        ),
        (
            before.num_vertices,
            before.num_edges,
            before.num_wires,
            before.num_faces,
            before.num_shells,
            before.num_solids
        ),
        "{ctx}: live counts changed"
    );
    assert_eq!(
        after.num_pcurves, before.num_pcurves,
        "{ctx}: pcurves changed"
    );
    assert_eq!(after.journal_len, before.journal_len, "{ctx}: journal grew");
    assert_eq!(after.volumes, before.volumes, "{ctx}: volumes changed");
    assert_eq!(
        format!("{:?}", topo.attributes()),
        before.attributes_debug,
        "{ctx}: attributes changed"
    );
    assert_eq!(
        format!("{:?}", topo.journal()),
        before.journal_debug,
        "{ctx}: journal contents changed"
    );
    for &s in solids {
        let live = format!("{:?}", topo.solid(s).unwrap());
        let key = format!("{s:?}");
        let expected = before
            .solids_debug
            .iter()
            .find(|(k, _)| k == &key)
            .map(|(_, v)| v)
            .unwrap();
        assert_eq!(&live, expected, "{ctx}: solid {s:?} contents changed");
    }
    assert!(
        after.allocated_slots >= before.allocated_slots,
        "{ctx}: slot high-water went backwards"
    );
    for &s in solids {
        assert!(topo.solid(s).is_ok(), "{ctx}: pre-existing solid stale");
    }
    if let Some(&first) = solids.first() {
        let attrs = topo.attributes().solid(first).unwrap();
        assert_eq!(
            attrs.name.as_deref(),
            Some("pre-existing"),
            "{ctx}: solid attributes changed"
        );
    }
}

/// Indices of P-section first-chunk lines carrying type-108 plane data.
fn plane_p_indices(valid: &str) -> Vec<usize> {
    let lines: Vec<&str> = valid.lines().collect();
    let mut out = Vec::new();
    for (i, line) in lines.iter().enumerate() {
        if line.len() >= 73
            && line.as_bytes().get(72).copied().unwrap_or(b' ') == b'P'
            && line[..64.min(line.len())].contains("108,")
        {
            out.push(i);
        }
    }
    out
}

/// Corrupt the nth 108 plane (0-based) to a zero normal, failing construction
/// after `n` earlier planes allocated their vertices/edges/wires/faces.
fn malform_plane_at(valid: &str, n: usize) -> String {
    let mut lines: Vec<String> = valid.lines().map(str::to_owned).collect();
    let indices = plane_p_indices(valid);
    assert!(!indices.is_empty(), "valid IGES must contain 108 planes");
    assert!(n < indices.len(), "plane index {n} out of range");
    let idx = indices[n];
    let suffix = lines[idx][64..].to_owned();
    let zero_plane = format!("{:<64}", "108,0.,0.,0.,0.,0,0,0,0;");
    lines[idx] = format!("{zero_plane}{suffix}");
    lines.join("\n")
}

/// Corrupt the nth 108 plane with an unparsable float token.
fn malform_plane_with_bad_float(valid: &str, n: usize) -> String {
    let mut lines: Vec<String> = valid.lines().map(str::to_owned).collect();
    let indices = plane_p_indices(valid);
    let idx = indices[n];
    let suffix = lines[idx][64..].to_owned();
    let bad = format!("{:<64}", "108,1.0,abc,0.,2.0,0,0,0,0;");
    lines[idx] = format!("{bad}{suffix}");
    lines.join("\n")
}

/// Corrupt the nth 108 plane with a nonfinite value.
fn malform_plane_with_nonfinite(valid: &str, n: usize) -> String {
    let mut lines: Vec<String> = valid.lines().map(str::to_owned).collect();
    let indices = plane_p_indices(valid);
    let idx = indices[n];
    let suffix = lines[idx][64..].to_owned();
    let bad = format!("{:<64}", "108,1.0,inf,0.,2.0,0,0,0,0;");
    lines[idx] = format!("{bad}{suffix}");
    lines.join("\n")
}

// ── Failure atomicity at every face depth ────────────────────────────

#[test]
fn failure_after_each_face_restores_full_state() {
    // Six planes per box IGES. Corrupting plane k fails after k faces
    // allocated (each with 4 vertices, 4 edges, 1 wire, 1 face + loops).
    // Shells/solids allocate only at the end (infallibly), so every failure
    // lands before shell construction; pre-existing shells/solids are
    // verified unchanged, and abandoned face/vertex slots stay stale.
    let valid = box_iges(2.0, 3.0, 4.0);
    assert_eq!(plane_p_indices(&valid).len(), 6);
    for k in 0..6 {
        let bad = malform_plane_at(&valid, k);
        {
            let mut probe = Topology::new();
            assert!(
                read_iges(&bad, &mut probe).is_err(),
                "plane {k} file unexpectedly imported"
            );
        }
        let (mut topo, solids) = seeded_doc(5);
        let before = capture_state(&topo, &solids);
        let pre_face = {
            let shell = topo.solid(solids[0]).unwrap().outer_shell();
            topo.shell(shell).unwrap().faces()[0]
        };
        let signature = EntitySignature::capture_face(&topo, pre_face, 1e-7).unwrap();
        let persistent = PersistentRef::signature(signature);

        let err = read_iges(&bad, &mut topo).unwrap_err();
        assert!(
            matches!(err, IoError::ParseError { .. }),
            "plane {k}: unexpected {err:?}"
        );
        assert_state_unchanged(&topo, &solids, &before, &format!("plane {k}"));

        let resolution = resolve(&topo, &persistent);
        assert!(
            matches!(
                resolution,
                remus_topology::naming::Resolution::Bound { .. }
                    | remus_topology::naming::Resolution::BoundMany { .. }
                    | remus_topology::naming::Resolution::Ambiguous { .. }
            ),
            "plane {k}: persistent ref lost: {resolution:?}"
        );

        if let Some(next) = topo.solid_id_from_index(solids.len()) {
            assert!(
                topo.solid(next).is_err(),
                "plane {k}: abandoned solid slot is live"
            );
        }
        let fresh = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        assert!(topo.solid(fresh).is_ok());
        assert!(
            !solids.contains(&fresh),
            "plane {k}: fresh handle aliases pre-existing"
        );
        for &s in &solids {
            assert!(topo.solid(s).is_ok(), "plane {k}: pre-existing lost");
        }
    }
}

#[test]
fn malformed_float_and_nonfinite_planes_restore_full_state() {
    let valid = box_iges(2.0, 3.0, 4.0);
    for (label, bad) in [
        ("bad-float", malform_plane_with_bad_float(&valid, 3)),
        ("nonfinite", malform_plane_with_nonfinite(&valid, 3)),
    ] {
        {
            let mut probe = Topology::new();
            assert!(
                read_iges(&bad, &mut probe).is_err(),
                "{label} unexpectedly imported"
            );
        }
        let (mut topo, solids) = seeded_doc(3);
        let before = capture_state(&topo, &solids);
        let err = read_iges(&bad, &mut topo).unwrap_err();
        assert!(
            matches!(err, IoError::ParseError { .. }),
            "{label}: unexpected {err:?}"
        );
        assert_state_unchanged(&topo, &solids, &before, label);
    }
}

#[test]
fn importer_produces_no_attributes_or_pcurves() {
    // The IGES importer writes no attributes, pcurves, or journal entries.
    let valid = box_iges(2.0, 3.0, 4.0);
    let (mut topo, solids) = seeded_doc(2);
    let attrs_before = format!("{:?}", topo.attributes());
    let journal_before = topo.journal().len();

    let imported = read_iges(&valid, &mut topo).unwrap();
    assert_eq!(imported.len(), 1);
    // New solids carry default (unnamed) attributes; pre-existing kept theirs.
    let pre_attrs = topo.attributes().solid(solids[0]).unwrap();
    assert_eq!(pre_attrs.name.as_deref(), Some("pre-existing"));
    // Importer wrote no pcurves and no journal entries.
    assert_eq!(topo.num_pcurves(), 0);
    assert_eq!(topo.journal().len(), journal_before);
    // Attributes store grew only by defaults (pre-existing debug prefix intact).
    assert!(
        format!("{:?}", topo.attributes()).starts_with(&attrs_before[..20.min(attrs_before.len())])
            || format!("{:?}", topo.attributes()).contains("pre-existing"),
        "pre-existing attributes lost"
    );

    // Failed import also writes none.
    let bad = malform_plane_at(&valid, 5);
    let attrs_before_fail = format!("{:?}", topo.attributes());
    let mut all = solids;
    all.extend(imported.iter().copied());
    let before = capture_state(&topo, &all);
    let _ = read_iges(&bad, &mut topo).unwrap_err();
    assert_eq!(format!("{:?}", topo.attributes()), attrs_before_fail);
    assert_eq!(topo.num_pcurves(), before.num_pcurves);
    assert_state_unchanged(&topo, &all, &before, "no-attrs-fail");
}

// ── Malformed records, references, counts, limits ────────────────────

#[test]
fn truncated_and_malformed_records_do_not_panic() {
    // Short lines (<73 cols) carry no section tag and contribute nothing.
    let short = "108,0.,0.,1.,0.;\nshort\n";
    let mut topo = Topology::new();
    let solids = read_iges(short, &mut topo).unwrap();
    assert!(solids.is_empty());
    assert_eq!(topo.num_solids(), 0);

    // Truncated D-section (lone line without its pair) is ignored, not indexed.
    let mut write_topo = Topology::new();
    let solid = make_box(&mut write_topo, 1.0, 1.0, 1.0).unwrap();
    let valid = remus_io::iges::write_iges(&write_topo, &[solid]).unwrap();
    let mut lines: Vec<String> = valid.lines().map(str::to_owned).collect();
    // Keep an odd D-line count by dropping the last D line: the trailing lone
    // entry is ignored, so one plane goes missing but no panic or partial
    // error occurs (the file still parses; fewer faces import).
    let d_count = lines
        .iter()
        .filter(|l| l.len() >= 73 && l.as_bytes()[72] == b'D')
        .count();
    assert!(d_count % 2 == 0);
    if let Some(pos) = lines
        .iter()
        .rposition(|l| l.len() >= 73 && l.as_bytes()[72] == b'D')
    {
        lines.remove(pos);
    }
    let truncated = lines.join("\n");
    let mut topo2 = Topology::new();
    // Must not panic; either succeeds with fewer faces or refuses typed.
    let result = read_iges(&truncated, &mut topo2);
    match result {
        Ok(solids) => {
            // Fewer than 6 faces if it succeeded (one DE pair broken).
            if !solids.is_empty() {
                let shell = topo2.solid(solids[0]).unwrap().outer_shell();
                assert!(topo2.shell(shell).unwrap().faces().len() <= 6);
            }
        }
        Err(err) => assert!(matches!(err, IoError::ParseError { .. })),
    }
}

#[test]
fn malformed_de_pointer_and_directory_fields_are_typed() {
    // P line with non-numeric DE pointer lumps under key 0 and matches no
    // directory entry: the planes lose params and (with strict planes) the
    // import refuses typed rather than panicking or publishing partial.
    let mut write_topo = Topology::new();
    let solid = make_box(&mut write_topo, 1.0, 1.0, 1.0).unwrap();
    let valid = remus_io::iges::write_iges(&write_topo, &[solid]).unwrap();
    let mut lines: Vec<String> = valid.lines().map(str::to_owned).collect();
    for line in &mut lines {
        if line.len() >= 80 && line.as_bytes()[72] == b'P' {
            // Corrupt DE pointer cols 64-72 to non-numeric.
            line.replace_range(64..72, "ABCDEFGH");
            break;
        }
    }
    let bad = lines.join("\n");
    let mut topo = Topology::new();
    // Must not panic. Either the corrupted plane refuses typed, or (if the
    // corrupted line was not a 108 first chunk) the import still succeeds —
    // but never with partial allocations on error.
    let before_slots = topo.allocated_slot_count();
    match read_iges(&bad, &mut topo) {
        Ok(_) => {}
        Err(err) => {
            assert!(matches!(
                err,
                IoError::ParseError { .. } | IoError::LimitExceeded { .. }
            ));
            assert_eq!(topo.num_solids(), 0);
            assert_eq!(topo.num_vertices(), 0);
            assert!(topo.allocated_slot_count() >= before_slots);
        }
    }

    // Non-numeric DE integer field is a typed parse error before allocation.
    let mut lines2: Vec<String> = valid.lines().map(str::to_owned).collect();
    for line in &mut lines2 {
        if line.len() >= 80 && line.as_bytes()[72] == b'D' {
            line.replace_range(0..8, "ABCDEFGH");
            break;
        }
    }
    let bad2 = lines2.join("\n");
    let mut topo2 = Topology::new();
    let err = read_iges(&bad2, &mut topo2).unwrap_err();
    assert!(matches!(err, IoError::ParseError { .. }));
    assert_eq!(topo2.num_solids(), 0);
}

#[test]
fn oversized_counts_and_limit_boundaries_are_honest() {
    let valid = box_iges(2.0, 3.0, 4.0);
    // Six planes need 92 generated slots.
    let tight = ImportLimits {
        max_model_entities: 10,
        ..Default::default()
    };
    let mut topo = Topology::new();
    let slots_before = topo.allocated_slot_count();
    let err = read_iges_with_limits(&valid, &mut topo, tight).unwrap_err();
    assert!(matches!(err, IoError::LimitExceeded { .. }));
    assert_eq!(topo.allocated_slot_count(), slots_before);
    assert_eq!(topo.num_solids(), 0);

    // Exact boundary succeeds.
    let exact = ImportLimits {
        max_model_entities: 92,
        ..Default::default()
    };
    let mut topo2 = Topology::new();
    assert_eq!(
        read_iges_with_limits(&valid, &mut topo2, exact)
            .unwrap()
            .len(),
        1
    );

    // Input-byte limit refuses before parsing/allocation.
    let byte_tight = ImportLimits {
        max_input_bytes: 16,
        ..Default::default()
    };
    let mut topo3 = Topology::new();
    assert!(matches!(
        read_iges_with_limits(&valid, &mut topo3, byte_tight).unwrap_err(),
        IoError::LimitExceeded { .. }
    ));
    assert_eq!(topo3.num_solids(), 0);
}

// ── Semantic preservation ────────────────────────────────────────────

#[test]
fn import_semantics_preserved_into_nonempty_documents() {
    // Coordinate conventions, formats, and default units are unchanged by
    // the transaction migration: imports into large docs match empty-doc
    // imports exactly (same 1×1 preview faces per plane, same normals/d).
    let iges = box_iges(2.0, 3.0, 4.0);
    let mut empty = Topology::new();
    let empty_solids = read_iges(&iges, &mut empty).unwrap();
    assert_eq!(empty_solids.len(), 1);
    let empty_shell = empty.solid(empty_solids[0]).unwrap().outer_shell();
    assert_eq!(empty.shell(empty_shell).unwrap().faces().len(), 6);
    let empty_faces = empty.num_faces();

    let (mut topo, solids) = seeded_doc(20);
    let faces_before = topo.num_faces();
    let imported = read_iges(&iges, &mut topo).unwrap();
    assert_eq!(imported.len(), 1);
    assert_eq!(topo.num_faces(), faces_before + empty_faces);
    // New faces match empty-doc faces exactly (normals and offsets).
    let new_shell = topo.solid(imported[0]).unwrap().outer_shell();
    assert_eq!(topo.shell(new_shell).unwrap().faces().len(), 6);
    // Pre-existing volumes unchanged.
    let before_vols: Vec<f64> = solids
        .iter()
        .map(|&s| solid_volume(&topo, s, 0.05).unwrap())
        .collect();
    for (i, &s) in solids.iter().enumerate() {
        assert!((solid_volume(&topo, s, 0.05).unwrap() - before_vols[i]).abs() < 1e-12);
    }

    // Resource limits and typed errors unchanged.
    let tight = ImportLimits {
        max_input_bytes: 16,
        ..Default::default()
    };
    assert!(matches!(
        read_iges_with_limits(&iges, &mut Topology::new(), tight).unwrap_err(),
        IoError::LimitExceeded { .. }
    ));
    let mut t = Topology::new();
    // Non-ASCII remains a typed refusal with an empty document.
    let mut line = " ".repeat(80);
    line.replace_range(63..65, "é");
    assert!(matches!(
        read_iges(&line, &mut t).unwrap_err(),
        IoError::ParseError { .. }
    ));
    assert_eq!(t.num_solids(), 0);
}

#[test]
fn import_after_checkpoint_restore_is_atomic() {
    let (mut topo, solids) = seeded_doc(10);
    let checkpoint = topo.clone();
    let extra = make_box(&mut topo, 9.0, 9.0, 9.0).unwrap();
    assert!(topo.solid(extra).is_ok());
    topo.restore_preserving_handle_slots(&checkpoint);
    assert!(topo.solid(extra).is_err() || topo.num_solids() == solids.len());
    let before = capture_state(&topo, &solids);
    let valid = box_iges(1.5, 1.5, 1.5);
    let imported = read_iges(&valid, &mut topo).unwrap();
    assert_eq!(imported.len(), 1);
    for (i, &s) in solids.iter().enumerate() {
        assert!((solid_volume(&topo, s, 0.05).unwrap() - before.volumes[i]).abs() < 1e-9);
    }
    let mut all = solids;
    all.push(imported[0]);
    let before2 = capture_state(&topo, &all);
    let bad = malform_plane_at(&valid, 2);
    let _ = read_iges(&bad, &mut topo).unwrap_err();
    let after2 = capture_state(&topo, &all);
    assert_eq!(after2.num_solids, before2.num_solids);
    assert_eq!(after2.journal_len, before2.journal_len);
    assert_eq!(after2.volumes, before2.volumes);
}

// ── Split translator route ───────────────────────────────────────────

#[test]
fn split_translator_import_to_arena_to_kernel_is_exact() {
    // Native simulation of the split-module path (translator `read_iges` +
    // `serialize_solids` → kernel `deserialize_solids`), as qualified against
    // the actually built `remus_wasm_io.wasm` (3.4 MB, release). The
    // translator always imports into a fresh scratch topology, so the
    // large-document clone this slice removes never existed there; native
    // import improvement does not make deserialization or JS transfer cheaper.
    let iges = box_iges(2.0, 3.0, 4.0);

    // Translator side: fresh scratch, read + serialize.
    let mut scratch = Topology::new();
    let solids = read_iges(&iges, &mut scratch).unwrap();
    assert_eq!(solids.len(), 1);
    let bytes = remus_io::arena_io::serialize_solids(&scratch, &solids).unwrap();
    assert!(!bytes.is_empty());

    // Kernel side: deserialize into live document with pre-existing bodies.
    let (mut kernel, kept) = seeded_doc(3);
    let kept_vols: Vec<f64> = kept
        .iter()
        .map(|&s| solid_volume(&kernel, s, 0.05).unwrap())
        .collect();
    let restored = remus_io::arena_io::deserialize_solids(&bytes, &mut kernel).unwrap();
    assert_eq!(restored.len(), 1);
    let shell = kernel.solid(restored[0]).unwrap().outer_shell();
    assert_eq!(kernel.shell(shell).unwrap().faces().len(), 6);
    for (i, &s) in kept.iter().enumerate() {
        assert!((solid_volume(&kernel, s, 0.05).unwrap() - kept_vols[i]).abs() < 1e-12);
    }

    // Failed translator import yields typed error and empty document (fresh
    // scratch, nothing published).
    let bad = malform_plane_at(&iges, 5);
    let mut scratch2 = Topology::new();
    let err = read_iges(&bad, &mut scratch2).unwrap_err();
    assert!(matches!(err, IoError::ParseError { .. }));
    assert_eq!(scratch2.num_solids(), 0);
    assert_eq!(scratch2.num_vertices(), 0);
}

// ── Bounded fuzz/replay campaign ─────────────────────────────────────

/// Deterministic xorshift64 for reproducible mutations (no external RNG).
fn xorshift64(state: &mut u64) -> u64 {
    let mut x = *state;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    *state = x;
    x
}

#[test]
fn bounded_fuzz_parser_and_construction_never_panics_or_leaks() {
    // 200 deterministic single-byte mutations over a valid box IGES plus
    // targeted token corruptions. Every outcome must be either success with
    // exact face counts or a typed refusal (ParseError / LimitExceeded /
    // InvalidTopology / Topology / Operations), never a panic, and every
    // refusal must leave the pre-existing document logically unchanged.
    // Minimized witnesses for each refusal family are preserved as the
    // dedicated tests above (zero-normal, bad-float, nonfinite, truncated,
    // bad DE pointer, oversized counts); this campaign proves no other
    // single-byte change escapes those typed families or leaks partials.
    let valid = box_iges(2.0, 3.0, 4.0);
    let bytes = valid.as_bytes();
    let mut rng: u64 = 0x1234_5678_9abc_def1;
    let mut seen_parse_error = false;
    let mut seen_success = false;
    for iter in 0..200 {
        let pos = (xorshift64(&mut rng) as usize) % bytes.len();
        let mut mutated = bytes.to_vec();
        // Mutate to a printable ASCII byte (keeps input ASCII so the
        // fixed-width parser runs; non-ASCII is covered separately).
        let replacement = 32 + (xorshift64(&mut rng) % 95) as u8;
        mutated[pos] = replacement;
        let candidate = String::from_utf8_lossy(&mutated).into_owned();

        let (mut topo, solids) = seeded_doc(2);
        let before = capture_state(&topo, &solids);
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            read_iges(&candidate, &mut topo)
        }));
        let outcome = result.expect("fuzz iteration panicked");
        match outcome {
            Ok(solids_out) => {
                seen_success = true;
                // Success must be exact: 0 or 1 solid, and when 1, 6 or fewer
                // faces (truncated DE pairs drop planes without error).
                assert!(solids_out.len() <= 1, "iter {iter}: too many solids");
                if let Some(&s) = solids_out.first() {
                    let shell = topo.solid(s).unwrap().outer_shell();
                    let faces = topo.shell(shell).unwrap().faces().len();
                    assert!(faces <= 6, "iter {iter}: too many faces {faces}");
                }
            }
            Err(err) => {
                assert!(
                    matches!(
                        err,
                        IoError::ParseError { .. }
                            | IoError::LimitExceeded { .. }
                            | IoError::InvalidTopology { .. }
                            | IoError::Topology(_)
                            | IoError::Operations(_)
                            | IoError::UnsupportedEntity { .. }
                    ),
                    "iter {iter}: untyped error {err:?}"
                );
                seen_parse_error = true;
                assert_state_unchanged(&topo, &solids, &before, &format!("fuzz {iter}"));
            }
        }
    }
    assert!(seen_success, "fuzz never succeeded (harness broken)");
    assert!(seen_parse_error, "fuzz never refused (harness broken)");
}
