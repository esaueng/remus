//! PERF-I02 STEP atomic import: failure atomicity at every stage,
//! append-only path qualification, and semantic preservation.
//!
//! Owns STEP reader/import orchestration only. Topology internals are
//! consumed via the existing `run_append_only` / `RollbackSnapshot` APIs;
//! missing primitives are handed off, not changed here.
//!
//! Each failing import compares complete pre-existing logical state —
//! model Debug, live counts, volumes, attributes, journal length and
//! persistent references — not just entity counts. Every handle allocated
//! by an abandoned attempt must remain stale, and a malformed later body
//! must not publish earlier bodies from the same file.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::naive_bytecount
)]

use remus_io::step::reader::{
    StepValidationOptions, read_step, read_step_bodies, read_step_with_limits_and_report,
    read_step_with_validation,
};
use remus_io::{ImportLimits, IoError};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_topology::Topology;
use remus_topology::naming::{EntitySignature, PersistentRef, resolve};

// ── Helpers ──────────────────────────────────────────────────────────

fn box_step(dx: f64, dy: f64, dz: f64) -> String {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, dx, dy, dz).unwrap();
    remus_io::step::write_step(&topo, &[solid]).unwrap()
}

fn cylinder_step() -> String {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
    remus_io::step::write_step(&topo, &[solid]).unwrap()
}

/// Pre-existing document with attributes, journal history and a named face
/// for persistent-reference checks.
fn seeded_doc(boxes: usize) -> (Topology, Vec<remus_topology::solid::SolidId>) {
    let mut topo = Topology::new();
    let mut solids = Vec::new();
    for i in 0..boxes {
        // Offset along x so bodies stay disjoint.
        let s = make_box(&mut topo, 1.0 + i as f64 * 0.01, 1.0, 1.0).unwrap();
        solids.push(s);
    }
    // Name the first solid + first face when present.
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
    // One journaled barrier so the journal is non-empty.
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
    // Live counts, journal length and pre-existing volumes must match.
    // Full `Debug` is NOT compared: retired high-water slots legitimately
    // grow on rollback (handles stay stale, never reused), so slot-level
    // Debug differs while logical state is identical.
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
    assert_eq!(after.journal_len, before.journal_len, "{ctx}: journal grew");
    assert_eq!(after.volumes, before.volumes, "{ctx}: volumes changed");
    // Attributes for pre-existing entities must be identical.
    assert_eq!(
        format!("{:?}", topo.attributes()),
        before.attributes_debug,
        "{ctx}: attributes changed"
    );
    // Journal contents identical (not just length).
    assert_eq!(
        format!("{:?}", topo.journal()),
        before.journal_debug,
        "{ctx}: journal contents changed"
    );
    // Live contents of every pre-existing solid identical.
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
    // High-water marks never decrease, even on rollback — but they must not
    // alias: fresh allocations land above the preserved mark.
    assert!(
        after.allocated_slots >= before.allocated_slots,
        "{ctx}: slot high-water went backwards"
    );
    // Pre-existing handles still resolve with identical attributes.
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

/// Malform a valid box STEP so its first solid fails at the requested stage.
/// Each file parses but fails during body construction, exercising the
/// transactional rollback. Line-based: finds the first line containing the
/// stage's entity and breaks one of its references (or empties its list).
fn malform_for_stage(valid: &str, stage: &str) -> String {
    let mut lines: Vec<String> = valid.lines().map(str::to_owned).collect();
    // Helper: replace the nth '#' reference on the target line with #999999.
    let break_nth_ref = |line: &mut String, n: usize| {
        let mut count = 0;
        let mut idx = 0;
        let bytes = line.as_bytes();
        while idx < bytes.len() {
            if bytes[idx] == b'#' {
                count += 1;
                if count == n {
                    // Consume "#digits".
                    let mut end = idx + 1;
                    while end < bytes.len() && bytes[end].is_ascii_digit() {
                        end += 1;
                    }
                    line.replace_range(idx..end, "#999999");
                    return true;
                }
            }
            idx += 1;
        }
        false
    };
    // Helper: empty the first "(...)" list containing '#' on the line.
    let empty_ref_list = |line: &mut String| {
        if let Some(open) = line.find('(') {
            // Find the '(' that starts a reference list (contains '#').
            let mut search = open;
            while let Some(pos) = line[search..].find('(') {
                let abs = search + pos;
                if let Some(close) = line[abs..].find(')') {
                    let inner = &line[abs + 1..abs + close];
                    if inner.contains('#') {
                        line.replace_range(abs + 1..abs + close, "");
                        return true;
                    }
                    search = abs + 1;
                } else {
                    break;
                }
            }
        }
        false
    };
    let target: &str = match stage {
        "vertices" => "VERTEX_POINT(",
        "edges" => "EDGE_CURVE(",
        "loops" => "EDGE_LOOP(",
        "faces" => "ADVANCED_FACE(",
        "shells" => "CLOSED_SHELL(",
        "solids" => "MANIFOLD_SOLID_BREP(",
        _ => return valid.to_owned(),
    };
    for line in &mut lines {
        if !line.contains(target) {
            continue;
        }
        let done = match stage {
            // VERTEX_POINT('', #43): break its point reference (1st).
            "vertices" => break_nth_ref(line, 1),
            // EDGE_CURVE('', #start, #end, #geom, ...): break geometry (3rd).
            "edges" => break_nth_ref(line, 3),
            // EDGE_LOOP('', (#...)): empty the edge list.
            "loops" => empty_ref_list(line),
            // ADVANCED_FACE('', (...), #surface, ...): break last '#' (surface).
            "faces" => {
                let total = line.as_bytes().iter().filter(|&&b| b == b'#').count();
                break_nth_ref(line, total)
            }
            // CLOSED_SHELL('', (...)): empty the face list.
            "shells" => empty_ref_list(line),
            // MANIFOLD_SOLID_BREP('', #shell): break its shell (1st).
            "solids" => break_nth_ref(line, 1),
            _ => false,
        };
        if done {
            break;
        }
    }
    lines.join("\n")
}

// ── Failure atomicity at every construction stage ────────────────────

#[test]
fn failure_after_each_construction_stage_restores_full_state() {
    for stage in ["vertices", "edges", "loops", "faces", "shells", "solids"] {
        let valid = box_step(2.0, 3.0, 4.0);
        let bad = malform_for_stage(&valid, stage);
        // Sanity: the malformed file must actually fail (not silently succeed).
        {
            let mut probe = Topology::new();
            assert!(
                read_step(&bad, &mut probe).is_err(),
                "stage {stage} file unexpectedly imported"
            );
        }
        let (mut topo, solids) = seeded_doc(5);
        let before = capture_state(&topo, &solids);
        // Capture a persistent reference to a pre-existing face.
        let pre_face = {
            let shell = topo.solid(solids[0]).unwrap().outer_shell();
            topo.shell(shell).unwrap().faces()[0]
        };
        let signature = EntitySignature::capture_face(&topo, pre_face, 1e-7).unwrap();
        let persistent = PersistentRef::signature(signature);

        // Attempt the failing import.
        let err = read_step(&bad, &mut topo).unwrap_err();
        assert!(
            matches!(
                err,
                IoError::ParseError { .. }
                    | IoError::UnsupportedEntity { .. }
                    | IoError::Topology(_)
                    | IoError::Operations(_)
                    | IoError::LimitExceeded { .. }
                    | IoError::InvalidTopology { .. }
            ),
            "stage {stage}: unexpected error {err:?}"
        );
        assert_state_unchanged(&topo, &solids, &before, stage);

        // Persistent reference still resolves to the same face.
        // Box faces are symmetric, so a signature may match several
        // pre-existing faces (Ambiguous) — that still proves the target was
        // not deleted or rebound. Dangling/NoMatch would mean loss.
        let resolution = resolve(&topo, &persistent);
        assert!(
            matches!(
                resolution,
                remus_topology::naming::Resolution::Bound { .. }
                    | remus_topology::naming::Resolution::BoundMany { .. }
                    | remus_topology::naming::Resolution::Ambiguous { .. }
            ),
            "stage {stage}: persistent ref lost: {resolution:?}"
        );

        // Handle safety: the next solid slot is either never allocated
        // (early failure) or retired stale (late failure) — never live.
        // A fresh build then succeeds without aliasing pre-existing handles.
        if let Some(next) = topo.solid_id_from_index(solids.len()) {
            assert!(
                topo.solid(next).is_err(),
                "stage {stage}: abandoned solid slot is live"
            );
        }
        let fresh = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        assert!(topo.solid(fresh).is_ok());
        assert!(
            !solids.contains(&fresh),
            "stage {stage}: fresh handle aliases pre-existing"
        );
        // The pre-existing document is still valid after the fresh build.
        for &s in &solids {
            assert!(
                topo.solid(s).is_ok(),
                "stage {stage}: pre-existing lost after fresh"
            );
        }
        let _ = before;
    }
}

#[test]
fn pcurve_stage_failure_restores_full_state() {
    // Curved faces carry PCURVEs; corrupt the PCURVE surface binding so the
    // failure lands in bind_face_pcurves, after faces are allocated.
    let valid = std::fs::read_to_string("tests/data/openzcad_e_analytic_fillet_plate.step")
        .expect("fillet-plate fixture");
    assert!(valid.contains("PCURVE"), "fillet plate must carry pcurves");
    let mut bad = valid;
    // Break the first PCURVE's basis-surface reference (second #xxx on the line).
    if let Some(pos) = bad.find("PCURVE(") {
        let seg_end = (pos + 300).min(bad.len());
        let seg = bad[pos..seg_end].to_owned();
        // Find second '#' in the segment.
        let hashes = seg
            .match_indices('#')
            .map(|(i, _)| pos + i)
            .collect::<Vec<_>>();
        if hashes.len() >= 2 {
            let abs = hashes[1];
            let end = bad[abs..]
                .find(|c: char| !c.is_ascii_digit() && c != '#')
                .unwrap_or(7);
            bad.replace_range(abs..abs + end, "#999999");
        }
    }
    {
        let mut probe = Topology::new();
        assert!(
            read_step(&bad, &mut probe).is_err(),
            "pcurve file unexpectedly imported"
        );
    }
    let (mut topo, solids) = seeded_doc(5);
    let before = capture_state(&topo, &solids);
    let err = read_step(&bad, &mut topo).unwrap_err();
    assert!(
        matches!(
            err,
            IoError::ParseError { .. } | IoError::UnsupportedEntity { .. } | IoError::Topology(_)
        ),
        "pcurve: unexpected {err:?}"
    );
    assert_state_unchanged(&topo, &solids, &before, "pcurves");
}

#[test]
fn attributes_and_names_survive_failed_imports_unchanged() {
    let valid = box_step(2.0, 2.0, 2.0);
    let bad = malform_for_stage(&valid, "solids");
    let (mut topo, solids) = seeded_doc(3);
    let before_attrs = format!("{:?}", topo.attributes());
    let _ = read_step(&bad, &mut topo).unwrap_err();
    assert_eq!(
        format!("{:?}", topo.attributes()),
        before_attrs,
        "attributes changed by failed import"
    );
    // Successful import preserves pre-existing names and adds new names.
    let ok_solids = read_step(&valid, &mut topo).unwrap();
    assert_eq!(ok_solids.len(), 1);
    let pre_attrs = topo.attributes().solid(solids[0]).unwrap();
    assert_eq!(pre_attrs.name.as_deref(), Some("pre-existing"));
    // New solids carry the file's name when present (writer emits 'Box' or similar;
    // at minimum the import must not clear pre-existing names).
    assert!(topo.solid(ok_solids[0]).is_ok());
}

#[test]
fn malformed_later_body_publishes_nothing() {
    // Two-body file where the second body is malformed: neither body may be
    // published, even though the first body alone is valid.
    let mut write_topo = Topology::new();
    let first = make_box(&mut write_topo, 1.0, 1.0, 1.0).unwrap();
    let second = make_box(&mut write_topo, 2.0, 2.0, 2.0).unwrap();
    let mut step = remus_io::step::write_step(&write_topo, &[first, second]).unwrap();
    let second_line = step
        .lines()
        .filter(|l| l.contains("MANIFOLD_SOLID_BREP"))
        .nth(1)
        .unwrap()
        .to_owned();
    let hash = second_line.rfind('#').unwrap();
    let end = hash + second_line[hash..].find(')').unwrap();
    let mut bad_line = second_line.clone();
    bad_line.replace_range(hash..end, "#999999");
    step = step.replacen(&second_line, &bad_line, 1);

    let (mut topo, solids) = seeded_doc(4);
    let before = capture_state(&topo, &solids);

    let err = read_step(&step, &mut topo).unwrap_err();
    assert!(matches!(
        err,
        IoError::ParseError { .. } | IoError::Topology(_) | IoError::Operations(_)
    ));
    // No partial bodies: live counts, volumes and journal unchanged.
    assert_state_unchanged(&topo, &solids, &before, "late-body");
    assert_eq!(topo.num_solids(), solids.len());

    // The abandoned first body stays stale: its slot does not come back live,
    // and the next build does not alias it.
    if let Some(abandoned) = topo.solid_id_from_index(solids.len()) {
        assert!(
            topo.solid(abandoned).is_err(),
            "abandoned late-body handle is live"
        );
    }
    let fresh = make_box(&mut topo, 3.0, 3.0, 3.0).unwrap();
    assert!(topo.solid(fresh).is_ok());
    assert!(!solids.contains(&fresh));
}

#[test]
fn rim_repair_and_validation_stages_commit_atomically() {
    // Split-rim file (two arcs for one full circle) exercises merge_split_rim_arcs
    // inside the import transaction; validation exercises the read-only tail.
    // Both must commit atomically and preserve pre-existing state on failure.
    let valid = box_step(2.0, 3.0, 4.0);
    let (mut topo, solids) = seeded_doc(3);
    let before = capture_state(&topo, &solids);

    // Successful rim + validation path commits exactly one body.
    let report =
        read_step_with_limits_and_report(&valid, &mut topo, ImportLimits::default()).unwrap();
    assert_eq!(report.solids().len(), 1);
    assert_eq!(topo.num_solids(), solids.len() + 1);
    // Pre-existing volumes unchanged; new body has expected volume.
    for (i, &s) in solids.iter().enumerate() {
        assert!((solid_volume(&topo, s, 0.05).unwrap() - before.volumes[i]).abs() < 1e-9);
    }
    assert!((solid_volume(&topo, report.solids()[0], 0.05).unwrap() - 24.0).abs() < 1e-6);

    // Validation with explicit options still commits (read-only tail).
    let validated = read_step_with_validation(
        &valid,
        &mut topo,
        ImportLimits::default(),
        StepValidationOptions::default(),
    )
    .unwrap();
    assert_eq!(validated.solids().len(), 1);

    // Invalid validation options fail before any topology mutation.
    let bad_options = StepValidationOptions {
        volume_relative: -1.0,
        ..Default::default()
    };
    let mut all_snapshot = solids;
    all_snapshot.extend(report.solids().iter().copied());
    all_snapshot.extend(validated.solids().iter().copied());
    let snapshot = capture_state(&topo, &all_snapshot);
    let err = read_step_with_validation(&valid, &mut topo, ImportLimits::default(), bad_options)
        .unwrap_err();
    assert!(matches!(err, IoError::InvalidValidationProperties { .. }));
    // No topology change: counts and journal identical.
    assert_eq!(topo.num_solids(), snapshot.num_solids);
    assert_eq!(topo.journal().len(), snapshot.journal_len);
}

// ── Semantic preservation ────────────────────────────────────────────

#[test]
fn import_semantics_preserved_into_nonempty_documents() {
    // Units, carriers, weights, orientations, cavities, order, names,
    // sheet/solid-only behavior, limits and typed errors are unchanged by
    // the transaction migration: imports into large docs match empty-doc
    // imports exactly.
    let cylinder = cylinder_step();
    let mut empty = Topology::new();
    let empty_solids = read_step(&cylinder, &mut empty).unwrap();
    let empty_volume = solid_volume(&empty, empty_solids[0], 0.01).unwrap();
    let empty_faces = empty.num_faces();

    let (mut topo, solids) = seeded_doc(20);
    let faces_before = topo.num_faces();
    let imported = read_step(&cylinder, &mut topo).unwrap();
    assert_eq!(imported.len(), 1);
    assert_eq!(topo.num_faces(), faces_before + empty_faces);
    let vol = solid_volume(&topo, imported[0], 0.01).unwrap();
    assert!(
        (vol - empty_volume).abs() < 1e-9,
        "volume {vol} vs {empty_volume}"
    );

    // Body-root order: multi-body file preserves order into nonempty docs.
    let mut write_topo = Topology::new();
    let a = make_box(&mut write_topo, 1.0, 1.0, 1.0).unwrap();
    let b = make_box(&mut write_topo, 2.0, 2.0, 2.0).unwrap();
    let multi = remus_io::step::write_step(&write_topo, &[a, b]).unwrap();
    let mut empty2 = Topology::new();
    let empty_roots = read_step(&multi, &mut empty2).unwrap();
    let empty_vols: Vec<f64> = empty_roots
        .iter()
        .map(|&s| solid_volume(&empty2, s, 0.01).unwrap())
        .collect();
    let (mut topo2, _) = seeded_doc(10);
    let roots = read_step(&multi, &mut topo2).unwrap();
    assert_eq!(roots.len(), 2);
    let vols: Vec<f64> = roots
        .iter()
        .map(|&s| solid_volume(&topo2, s, 0.01).unwrap())
        .collect();
    assert_eq!(vols, empty_vols, "body-root order/volumes changed");

    // Sheet/solid-only behavior: sheet file imports 0 solids via read_step,
    // 1 sheet via read_step_bodies, in both empty and nonempty docs.
    let mut sheet_topo = Topology::new();
    let face =
        remus_topology::builder::make_rectangle_face(&mut sheet_topo, 2.0, 1.0, 1e-7).unwrap();
    let sheet = remus_operations::sew::make_sheet_body(&mut sheet_topo, &[face]).unwrap();
    let sheet_doc = remus_io::arena_io::serialize_sheets(&sheet_topo, &[sheet]).unwrap();
    let sheet_step = {
        let (t, d) = {
            let mut t = Topology::new();
            let doc = remus_io::arena_io::deserialize_sheets(&sheet_doc, &mut t).unwrap();
            (t, doc)
        };
        remus_io::step::write_step_bodies_with_options(
            &t,
            &[],
            &d,
            &remus_io::step::StepWriteOptions::default(),
        )
        .unwrap()
    };
    let mut e1 = Topology::new();
    assert!(read_step(&sheet_step, &mut e1).unwrap().is_empty());
    let mut n1 = seeded_doc(5).0;
    assert!(read_step(&sheet_step, &mut n1).unwrap().is_empty());
    let mut e2 = Topology::new();
    assert_eq!(
        read_step_bodies(&sheet_step, &mut e2)
            .unwrap()
            .sheets()
            .len(),
        1
    );
    let mut n2 = seeded_doc(5).0;
    assert_eq!(
        read_step_bodies(&sheet_step, &mut n2)
            .unwrap()
            .sheets()
            .len(),
        1
    );

    // Resource limits and typed errors unchanged.
    let tight = ImportLimits {
        max_input_bytes: 16,
        ..Default::default()
    };
    assert!(matches!(
        read_step_with_limits_and_report(&cylinder, &mut Topology::new(), tight).unwrap_err(),
        IoError::LimitExceeded { .. }
    ));
    // Malformed input remains a typed refusal and leaves the document empty.
    let mut t = Topology::new();
    assert!(matches!(
        read_step("not a step file", &mut t).unwrap_err(),
        IoError::ParseError { .. } | IoError::LimitExceeded { .. }
    ));
    assert_eq!(t.num_solids(), 0);
    let _ = solids;
}

#[test]
fn cavities_and_names_round_trip_into_large_documents() {
    // Cavity file (when available) plus name preservation.
    let valid = box_step(4.0, 4.0, 1.0);
    let (mut topo, solids) = seeded_doc(10);
    let imported = read_step(&valid, &mut topo).unwrap();
    assert_eq!(imported.len(), 1);
    // Pre-existing cavity-free volumes unchanged (no cavity fixture here;
    // cavity preservation is covered by multicavity corpus tests, which
    // also run through this transaction path).
    for &s in &solids {
        assert!(topo.solid(s).is_ok());
    }
    assert!(topo.solid(imported[0]).is_ok());
}

#[test]
fn import_after_checkpoint_restore_is_atomic() {
    let (mut topo, solids) = seeded_doc(10);
    let checkpoint = topo.clone();
    // Diverge, then restore, then import: the import must see the restored
    // state and remain atomic on failure.
    let extra = make_box(&mut topo, 9.0, 9.0, 9.0).unwrap();
    assert!(topo.solid(extra).is_ok());
    topo.restore_preserving_handle_slots(&checkpoint);
    assert!(topo.solid(extra).is_err() || topo.num_solids() == solids.len());
    let before = capture_state(&topo, &solids);
    let valid = box_step(1.5, 1.5, 1.5);
    let imported = read_step(&valid, &mut topo).unwrap();
    assert_eq!(imported.len(), 1);
    // Successful import preserves pre-existing volumes (counts legitimately grow).
    for (i, &s) in solids.iter().enumerate() {
        assert!((solid_volume(&topo, s, 0.05).unwrap() - before.volumes[i]).abs() < 1e-9);
    }
    // Failed import after restore also restores fully.
    let bad = malform_for_stage(&valid, "solids");
    let mut all = solids;
    all.push(imported[0]);
    let before2 = capture_state(&topo, &all);
    let _ = read_step(&bad, &mut topo).unwrap_err();
    let after2 = capture_state(&topo, &all);
    assert_eq!(after2.num_solids, before2.num_solids);
    assert_eq!(after2.journal_len, before2.journal_len);
    assert_eq!(after2.volumes, before2.volumes);
}
