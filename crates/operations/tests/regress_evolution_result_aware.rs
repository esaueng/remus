//! Result-aware history completeness: the map against the actual result.
//!
//! [`EvolutionMap::is_complete`] checks only that the `unresolved` bucket is
//! empty. That limited contract cannot see a result face the map never
//! mentions (omitted) or a claimed face that is not in the result (phantom):
//! an empty map for a nonempty result reports `true` there. The result-aware
//! check — [`EvolutionMap::completeness_for_result`] and its
//! [`EvolutionMap::accounts_for_result`] /
//! [`EvolutionMap::is_resolved_for_result`] /
//! [`EvolutionMap::is_construction_resolved_for_result`] wrappers — compares
//! attributed output identities with the actual result-entity set and reports
//! omitted and phantom entities explicitly.
//!
//! This file does not rediscover the set-equality coverage in
//! `regress_evolution_completeness.rs`: that file's
//! `assert_lineage_accounts_for_everything` remains the lineage invariant for
//! fillets, chamfers, booleans and patterns, and is reused here by reference
//! (same sets, same direction). What it adds is the checker itself, proved on
//! synthetic malformed maps, plus one real operation qualified through it.
//!
//! A synthetic malformed map proves the checker, not a production naming
//! defect. Whether any real producer or consumer fails is reported separately
//! below: the `plane_split_*` tests qualify
//! [`remus_operations::split::split_with_evolution`] through the new API at
//! three modelling units, under rigid placements, and with rollback evidence.
//! Broader producer changes are out of scope and handed off with a witness.
//!
//! Complete accounting (every result face claimed in `modified`, `generated`
//! or explicitly `unresolved`) is separated from resolved construction
//! provenance (accounted, no unresolved, [`EvolutionOrigin::Construction`]).
//! Split caps are the honest-`unresolved` case that exercises the separation.
//!
//! No test here imposes a one-parent or one-record rule: merges (several
//! inputs into one output), splits (one input into several outputs) and
//! generated bands with several legitimate parents are all accepted.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::{BTreeSet, HashSet};

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::evolution::{EvolutionMap, EvolutionOrigin};
use remus_operations::{primitives, split};
use remus_topology::Topology;
use remus_topology::arena::Id;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

fn faces_of(topo: &Topology, solid: SolidId) -> BTreeSet<usize> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(Id::index)
        .collect()
}

fn hash_of(set: &BTreeSet<usize>) -> HashSet<usize> {
    set.iter().copied().collect()
}

// ─── The helper's limited contract ──────────────────────────────────────

/// An empty map for a nonempty result: the legacy helper certifies it
/// complete, the result-aware check does not.
#[test]
fn empty_map_for_nonempty_result_is_not_accounted() {
    let map = EvolutionMap::exact();
    assert!(
        map.is_complete(),
        "the legacy helper sees only the empty unresolved bucket"
    );
    let result: BTreeSet<usize> = [7, 8, 9].into_iter().collect();
    let report = map.completeness_for_result(result.iter().copied());
    assert_eq!(report.omitted, vec![7, 8, 9]);
    assert!(report.phantom.is_empty());
    assert!(!report.is_accounted());
    assert!(!report.is_resolved());
    assert!(!map.accounts_for_result(result.iter().copied()));
    assert!(!map.is_resolved_for_result(result.iter().copied()));
    assert!(!map.is_construction_resolved_for_result(result.iter().copied()));
}

/// Empty against empty is vacuously accounted and resolved.
#[test]
fn empty_map_for_empty_result_is_vacuously_accounted() {
    let map = EvolutionMap::exact();
    let empty: BTreeSet<usize> = BTreeSet::new();
    let report = map.completeness_for_result(empty.iter().copied());
    assert!(report.is_accounted());
    assert!(report.is_resolved());
    assert!(map.accounts_for_result(empty.iter().copied()));
}

// ─── Synthetic malformed maps prove the checker ─────────────────────────

/// Deliberate record removal: one result face loses its only claim.
#[test]
fn deliberate_record_removal_is_reported_as_omitted() {
    let mut map = EvolutionMap::exact();
    map.add_modified(0, 10);
    map.add_modified(1, 11);
    map.add_modified(2, 12);
    let result: BTreeSet<usize> = [10, 11, 12].into_iter().collect();
    assert!(map.accounts_for_result(result.iter().copied()));

    // Remove the claim on 11 the way a dropped record would.
    let mut broken = EvolutionMap::exact();
    broken.add_modified(0, 10);
    broken.add_modified(2, 12);
    assert!(
        broken.is_complete(),
        "no unresolved entry was added, so the legacy helper still passes"
    );
    let report = broken.completeness_for_result(result.iter().copied());
    assert_eq!(report.omitted, vec![11]);
    assert!(report.phantom.is_empty());
    assert!(!broken.accounts_for_result(result.iter().copied()));
}

/// A claimed face that is not in the result is phantom.
#[test]
fn phantom_output_is_reported() {
    let mut map = EvolutionMap::exact();
    map.add_modified(0, 10);
    map.add_modified(1, 11);
    map.add_modified(1, 99);
    let result: BTreeSet<usize> = [10, 11].into_iter().collect();
    let report = map.completeness_for_result(result.iter().copied());
    assert!(report.omitted.is_empty());
    assert_eq!(report.phantom, vec![99]);
    assert!(!map.accounts_for_result(result.iter().copied()));
}

/// Explicit unresolved records count as accounted but not resolved.
#[test]
fn unresolved_counts_as_accounted_but_not_resolved() {
    let mut map = EvolutionMap::exact();
    map.add_modified(0, 10);
    map.add_unresolved(11, vec![0]);
    let result: BTreeSet<usize> = [10, 11].into_iter().collect();
    assert!(
        !map.is_complete(),
        "the legacy helper refuses on the unresolved entry"
    );
    let report = map.completeness_for_result(result.iter().copied());
    assert!(report.omitted.is_empty());
    assert!(report.phantom.is_empty());
    assert_eq!(report.unresolved_outputs, vec![11]);
    assert!(report.is_accounted());
    assert!(!report.is_resolved());
    assert!(map.accounts_for_result(result.iter().copied()));
    assert!(!map.is_resolved_for_result(result.iter().copied()));
    assert!(!map.is_construction_resolved_for_result(result.iter().copied()));

    // Same accounting with inferred provenance is not construction-resolved
    // even when fully resolved: provenance and resolution are separate axes.
    let mut inferred = EvolutionMap::new();
    assert_eq!(inferred.origin, EvolutionOrigin::Geometry);
    inferred.add_modified(0, 10);
    inferred.add_modified(1, 11);
    assert!(inferred.is_resolved_for_result(result.iter().copied()));
    assert!(!inferred.is_construction_resolved_for_result(result.iter().copied()));
    let mut exact = EvolutionMap::exact();
    exact.add_modified(0, 10);
    exact.add_modified(1, 11);
    assert!(exact.is_construction_resolved_for_result(result.iter().copied()));
}

// ─── Many-to-many history is legitimate ─────────────────────────────────

/// One input fanning out to several outputs (a split) is not a conflict.
#[test]
fn split_fan_out_is_not_a_conflict() {
    let mut map = EvolutionMap::exact();
    map.add_modified(0, 10);
    map.add_modified(0, 11);
    let result: BTreeSet<usize> = [10, 11].into_iter().collect();
    let report = map.completeness_for_result(result.iter().copied());
    assert!(report.is_accounted());
    assert!(report.is_resolved());
}

/// Several inputs flowing into one output (a same-domain merge) is not a
/// conflict.
#[test]
fn merge_fan_in_is_not_a_conflict() {
    let mut map = EvolutionMap::exact();
    map.add_modified(0, 10);
    map.add_modified(1, 10);
    let result: BTreeSet<usize> = std::iter::once(10).collect();
    let report = map.completeness_for_result(result.iter().copied());
    assert!(report.is_accounted());
    assert!(report.is_resolved());
    // The merge attribution itself is retained, not collapsed away.
    assert_eq!(map.modified.get(&0), Some(&vec![10]));
    assert_eq!(map.modified.get(&1), Some(&vec![10]));
}

/// A generated band built between two base faces names both parents.
#[test]
fn generated_band_with_two_parents_is_legitimate() {
    let mut map = EvolutionMap::exact();
    map.add_modified(0, 10);
    map.add_modified(1, 11);
    map.add_modified(2, 12);
    map.add_generated(0, 20);
    map.add_generated(2, 20);
    let result: BTreeSet<usize> = [10, 11, 12, 20].into_iter().collect();
    let report = map.completeness_for_result(result.iter().copied());
    assert!(report.is_accounted(), "report: {report:?}");
    assert!(report.is_resolved());
    let mut parents: Vec<usize> = map
        .generated
        .iter()
        .filter(|(_, outs)| outs.contains(&20))
        .map(|(src, _)| *src)
        .collect();
    parents.sort_unstable();
    assert_eq!(parents, vec![0, 2]);
}

// ─── Retained and deleted sources ───────────────────────────────────────

/// Retained inputs stay in `modified`, consumed inputs are `deleted`; the
/// result-aware check covers outputs while the source side stays explicit.
#[test]
fn retained_and_deleted_sources_are_distinguished() {
    let mut map = EvolutionMap::exact();
    map.add_modified(0, 10);
    map.add_modified(1, 11);
    map.add_deleted(2);
    let result: BTreeSet<usize> = [10, 11].into_iter().collect();
    assert!(map.accounts_for_result(result.iter().copied()));
    assert!(map.is_resolved_for_result(result.iter().copied()));
    assert!(map.modified.contains_key(&0) && map.modified.contains_key(&1));
    assert!(map.deleted.contains(&2));
    // A contested input that only ever lost an unresolved tie is neither
    // retained nor deleted; that unknown state is asserted by the geometry
    // matcher's own tests and is not re-decided here.
}

// ─── One real operation through the new API: plane split ────────────────

/// The modelling units the same split is built in. 1000x and 0.001x are the
/// axis a scale-blind rule fails on; the split's own slack is scale-relative
/// so the history must read the same at each of them.
const SCALES: [f64; 3] = [1.0, 1000.0, 0.001];

/// Split one box and check both halves through the result-aware API.

#[test]
fn plane_split_halves_are_accounted_with_honest_caps_at_every_scale() {
    for scale in SCALES {
        let mut topo = Topology::new();
        let edge = 10.0 * scale;
        let cut_z = 4.0 * scale;
        // Capture the input before the split: the operation trims the input's
        // faces in place, so post-split reads of the source solid would not
        // be the pre-operation set.
        let cube = primitives::make_box(&mut topo, edge, edge, edge).unwrap();
        let inputs = faces_of(&topo, cube);
        assert_eq!(inputs.len(), 6, "a box has six faces");
        let input_volume =
            remus_operations::measure::solid_volume(&topo, cube, 0.01 * scale.max(1.0)).unwrap();
        let (result, evo) = split::split_with_evolution(
            &mut topo,
            cube,
            Point3::new(0.0, 0.0, cut_z),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();

        for (name, half, map) in [
            ("positive", result.positive, &evo.positive),
            ("negative", result.negative, &evo.negative),
        ] {
            let label = format!("{name} split at {scale}x");
            let after = faces_of(&topo, half);
            // Six faces per half: the untouched cap, four trimmed walls, the cap.
            assert_eq!(after.len(), 6, "{label}: unexpected face count");

            assert_eq!(
                map.origin,
                EvolutionOrigin::Construction,
                "{label}: split history is construction-derived"
            );
            // The honest cap is the one unresolved face, with no candidates:
            // it was synthesised, not derived from any one input.
            assert_eq!(
                map.unresolved.len(),
                1,
                "{label}: exactly the cap is unresolved: {:?}",
                map.unresolved
            );
            for candidates in map.unresolved.values() {
                assert!(
                    candidates.is_empty(),
                    "{label}: the cap has no single input source"
                );
            }
            // Legacy helper refuses (correctly, but opaquely).
            assert!(
                !map.is_complete(),
                "{label}: the cap keeps the legacy helper refused"
            );
            // Result-aware accounting passes; resolved provenance does not.
            let report = map.completeness_for_result(after.iter().copied());
            assert!(
                report.omitted.is_empty(),
                "{label}: omitted faces {:?}: {}",
                report.omitted,
                map.to_json()
            );
            assert!(
                report.phantom.is_empty(),
                "{label}: phantom faces {:?}: {}",
                report.phantom,
                map.to_json()
            );
            assert_eq!(
                report.unresolved_outputs.len(),
                1,
                "{label}: the cap is the accounted-but-unresolved face"
            );
            assert!(map.accounts_for_result(after.iter().copied()));
            assert!(!map.is_resolved_for_result(after.iter().copied()));
            assert!(!map.is_construction_resolved_for_result(after.iter().copied()));

            // Every modified source is a real input face.
            for src in map.modified.keys() {
                assert!(
                    inputs.contains(src),
                    "{label}: source {src} is not an input face"
                );
            }
            // No input is silently lost: the four straddled walls appear in
            // both halves, the two whole caps in exactly one.
            let _ = label;
        }

        // Source-side union: all six inputs survive in at least one half;
        // a split deletes nothing.
        let mut union_sources: BTreeSet<usize> = BTreeSet::new();
        union_sources.extend(evo.positive.modified.keys().copied());
        union_sources.extend(evo.negative.modified.keys().copied());
        assert_eq!(
            union_sources, inputs,
            "split at {scale}x: every input must survive in some half"
        );
        assert!(evo.positive.deleted.is_empty() && evo.negative.deleted.is_empty());

        // Independent oracles, not a second wrapper over the same journal:
        // both halves validate, and their volumes add back to the input.
        for half in [result.positive, result.negative] {
            let report = remus_check::validate::validate_solid(
                &topo,
                half,
                &remus_check::validate::ValidateOptions::default(),
            )
            .unwrap();
            assert!(report.is_valid(), "{:#?}", report.issues);
        }
        let pos_volume =
            remus_operations::measure::solid_volume(&topo, result.positive, 0.01 * scale.max(1.0))
                .unwrap();
        let neg_volume =
            remus_operations::measure::solid_volume(&topo, result.negative, 0.01 * scale.max(1.0))
                .unwrap();
        let expected_pos = edge * edge * (edge - cut_z);
        let expected_neg = edge * edge * cut_z;
        let rel = |a: f64, b: f64| (a - b).abs() / b.abs().max(1e-30);
        assert!(
            rel(pos_volume, expected_pos) < 1e-6,
            "positive half volume {pos_volume} != closed form {expected_pos} at {scale}x"
        );
        assert!(
            rel(neg_volume, expected_neg) < 1e-6,
            "negative half volume {neg_volume} != closed form {expected_neg} at {scale}x"
        );
        assert!(
            rel(pos_volume + neg_volume, input_volume) < 1e-9,
            "halves {pos_volume}+{neg_volume} != input {input_volume} at {scale}x"
        );
        // The four straddled walls are the intersection of the two halves'
        // source sets; the whole caps are disjoint. This is the split
        // fan-out the checker must accept, not flag.
        let pos_sources: BTreeSet<usize> = evo.positive.modified.keys().copied().collect();
        let neg_sources: BTreeSet<usize> = evo.negative.modified.keys().copied().collect();
        assert_eq!(pos_sources.intersection(&neg_sources).count(), 4);
        assert_eq!(pos_sources.union(&neg_sources).count(), 6);
        // Phantom guard on the combined scope: the union of both halves'
        // attributed faces is exactly the union of both halves' faces.
        let mut combined_attributed = evo.positive.attributed_outputs();
        combined_attributed.extend(evo.negative.attributed_outputs());
        let mut combined_result = faces_of(&topo, result.positive);
        combined_result.extend(faces_of(&topo, result.negative));
        assert_eq!(combined_attributed, combined_result);
        let _ = hash_of(&combined_result);
    }
}

/// The same split under rigid placements: a translated body and a body
/// rotated 0.7 rad about Z with its cutting plane carried along.
#[test]
fn plane_split_history_survives_translation_and_rotation() {
    for (placement, matrix, plane_point) in [
        (
            "translated",
            Mat4::translation(100.0, -50.0, 30.0),
            Point3::new(100.0, -50.0, 30.0 + 4.0),
        ),
        (
            "rotated",
            Mat4::translation(100.0, -50.0, 30.0) * Mat4::rotation_z(0.7),
            Point3::new(100.0, -50.0, 30.0 + 4.0),
        ),
    ] {
        let mut topo = Topology::new();
        let cube = primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        remus_operations::transform::transform_solid(&mut topo, cube, &matrix).unwrap();
        let inputs = faces_of(&topo, cube);
        // The cutting plane stays horizontal (normal +Z), so rotation about
        // Z leaves the section unchanged while moving every vertex: a
        // placement-sensitive rule would answer differently here.
        let (result, evo) =
            split::split_with_evolution(&mut topo, cube, plane_point, Vec3::new(0.0, 0.0, 1.0))
                .unwrap();
        for (half, map) in [
            (result.positive, &evo.positive),
            (result.negative, &evo.negative),
        ] {
            let after = faces_of(&topo, half);
            assert_eq!(after.len(), 6, "{placement}: unexpected face count");
            assert!(map.origin.is_exact());
            assert!(map.accounts_for_result(after.iter().copied()));
            assert!(!map.is_resolved_for_result(after.iter().copied()));
            assert_eq!(map.unresolved.len(), 1);
        }
        let pos_volume =
            remus_operations::measure::solid_volume(&topo, result.positive, 0.01).unwrap();
        let neg_volume =
            remus_operations::measure::solid_volume(&topo, result.negative, 0.01).unwrap();
        assert!(
            (pos_volume - 600.0).abs() < 1e-6,
            "{placement}: positive volume {pos_volume} != 600"
        );
        assert!(
            (neg_volume - 400.0).abs() < 1e-6,
            "{placement}: negative volume {neg_volume} != 400"
        );
        let _ = inputs;
    }
}

/// A split that misses the solid refuses transactionally: source faces,
/// source volume and closedness are unchanged. This is the rollback half of
/// the qualification — history that fails to build must not move geometry.
#[test]
fn failed_split_leaves_source_topology_unchanged() {
    let mut topo = Topology::new();
    let cube = primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let before_faces = faces_of(&topo, cube);
    let before_volume = remus_operations::measure::solid_volume(&topo, cube, 0.01).unwrap();
    let failed = split::split_with_evolution(
        &mut topo,
        cube,
        Point3::new(0.0, 0.0, 50.0),
        Vec3::new(0.0, 0.0, 1.0),
    );
    assert!(
        failed.is_err(),
        "a plane missing the solid must be rejected"
    );
    assert_eq!(
        faces_of(&topo, cube),
        before_faces,
        "failed split changed the source face set"
    );
    let after_volume = remus_operations::measure::solid_volume(&topo, cube, 0.01).unwrap();
    assert!(
        (after_volume - before_volume).abs() < 1e-9,
        "failed split changed source volume: {before_volume} -> {after_volume}"
    );
    let shell = topo.solid(cube).unwrap().outer_shell();
    remus_topology::validation::validate_shell_closed(topo.shell(shell).unwrap(), &topo).unwrap();
}
