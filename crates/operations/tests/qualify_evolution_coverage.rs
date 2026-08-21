//! Qualification evidence for construction-derived face evolution on the
//! previously barrier-only operations: draft, defeature, and plane split
//! (stabilization plan item B3).
//!
//! Each test asserts the full source→result domain is enumerated — every
//! result face claimed exactly once, every input face accounted for — and
//! that the map's origin is `Construction`, never geometric inference.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeSet;

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::defeature::defeature_with_evolution;
use remus_operations::draft::draft_with_evolution;
use remus_operations::evolution::EvolutionMap;
use remus_operations::primitives::make_box;
use remus_operations::split::split_with_evolution;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

fn outer_faces(topo: &Topology, solid: SolidId) -> Vec<FaceId> {
    let s = topo.solid(solid).unwrap();
    topo.shell(s.outer_shell()).unwrap().faces().to_vec()
}

/// The set of output faces the map claims, over modified + generated +
/// unresolved. Panics if any output is claimed twice.
fn claimed_outputs(map: &EvolutionMap) -> BTreeSet<usize> {
    let mut seen = BTreeSet::new();
    let mut claim = |out: usize| {
        assert!(seen.insert(out), "output face {out} claimed twice: {map:?}");
    };
    for outs in map.modified.values() {
        for &o in outs {
            claim(o);
        }
    }
    for outs in map.generated.values() {
        for &o in outs {
            claim(o);
        }
    }
    for &o in map.unresolved.keys() {
        claim(o);
    }
    seen
}

/// Draft: every result face is `modified` from exactly one input face; no
/// deletions, nothing unresolved, origin is construction.
#[test]
fn draft_evolution_is_total_and_exact() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let inputs: BTreeSet<usize> = outer_faces(&topo, cube).iter().map(|f| f.index()).collect();
    let wall = outer_faces(&topo, cube)
        .into_iter()
        .filter(|&f| {
            matches!(
                topo.face(f).unwrap().surface(),
                remus_topology::face::FaceSurface::Plane { normal, .. }
                    if (*normal - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-9
            )
        })
        .collect::<Vec<_>>();

    let (result, map) = draft_with_evolution(
        &mut topo,
        cube,
        &wall,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        5.0_f64.to_radians(),
    )
    .unwrap();

    assert!(
        map.origin.is_exact(),
        "draft evolution must be construction-derived"
    );
    assert!(map.deleted.is_empty());
    assert!(map.unresolved.is_empty());

    let outputs: BTreeSet<usize> = outer_faces(&topo, result)
        .iter()
        .map(|f| f.index())
        .collect();
    assert_eq!(
        claimed_outputs(&map),
        outputs,
        "every result face must be claimed exactly once"
    );
    let sources: BTreeSet<usize> = map.modified.keys().copied().collect();
    assert_eq!(sources, inputs, "every input face must appear as a source");
}

/// Split: both halves' maps claim every face of their half; carried and
/// trimmed faces are modified from real inputs, the cap is the single
/// honestly-unresolved face.
#[test]
fn split_evolution_accounts_for_both_halves() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let inputs: BTreeSet<usize> = outer_faces(&topo, cube).iter().map(|f| f.index()).collect();

    let (result, evo) = split_with_evolution(
        &mut topo,
        cube,
        Point3::new(0.0, 0.0, 0.4),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();

    for (half, map) in [
        (result.positive, &evo.positive),
        (result.negative, &evo.negative),
    ] {
        assert!(map.origin.is_exact());
        let outputs: BTreeSet<usize> = outer_faces(&topo, half).iter().map(|f| f.index()).collect();
        assert_eq!(claimed_outputs(map), outputs);
        assert_eq!(
            map.unresolved.len(),
            1,
            "exactly the cap is synthesised: {map:?}"
        );
        for src in map.modified.keys() {
            assert!(inputs.contains(src), "source {src} is not an input face");
        }
    }

    // The four straddled walls appear as sources in BOTH halves; the whole
    // top/bottom faces in exactly one.
    let pos_sources: BTreeSet<usize> = evo.positive.modified.keys().copied().collect();
    let neg_sources: BTreeSet<usize> = evo.negative.modified.keys().copied().collect();
    assert_eq!(pos_sources.intersection(&neg_sources).count(), 4);
    assert_eq!(pos_sources.union(&neg_sources).count(), 6);
}

/// Defeature: removed feature faces are `deleted`, kept faces are
/// `modified` onto the healed result, and the whole result is claimed.
#[test]
fn defeature_evolution_reports_deletions() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let cutter = make_box(&mut topo, 0.4, 0.4, 2.0).unwrap();
    transform_solid(&mut topo, cutter, &Mat4::translation(0.3, 0.3, -0.5)).unwrap();
    let holed = boolean(&mut topo, BooleanOp::Cut, cube, cutter).unwrap();

    let walls: Vec<FaceId> = outer_faces(&topo, holed)
        .into_iter()
        .filter(|&f| {
            let face = topo.face(f).unwrap();
            let mut inside = true;
            for oe in topo.wire(face.outer_wire()).unwrap().edges() {
                let e = topo.edge(oe.edge()).unwrap();
                for v in [e.start(), e.end()] {
                    let p = topo.vertex(v).unwrap().point();
                    inside &= p.x() > 0.29 && p.x() < 0.71 && p.y() > 0.29 && p.y() < 0.71;
                }
            }
            inside
        })
        .collect();
    assert_eq!(walls.len(), 4);

    let (result, map) = defeature_with_evolution(&mut topo, holed, &walls).unwrap();

    assert!(map.origin.is_exact());
    for w in &walls {
        assert!(
            map.deleted.contains(&w.index()),
            "removed wall {} must be reported deleted",
            w.index()
        );
    }
    let outputs: BTreeSet<usize> = outer_faces(&topo, result)
        .iter()
        .map(|f| f.index())
        .collect();
    assert_eq!(claimed_outputs(&map), outputs);
    assert!(map.unresolved.is_empty(), "the heal is fully attributed");
}

/// Shell: kept outer faces are `modified`, inner-skin faces `generated`
/// from the face they offset, opened faces `deleted`, and every result
/// face of a closed hollow is claimed by one of them.
#[test]
fn shell_evolution_is_total() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let inputs: BTreeSet<usize> = outer_faces(&topo, cube).iter().map(|f| f.index()).collect();

    let (hollow, map) =
        remus_operations::shell_op::shell_with_evolution(&mut topo, cube, 0.1, &[]).unwrap();

    assert!(map.origin.is_exact());
    assert!(map.deleted.is_empty(), "no faces were opened");
    assert!(map.unresolved.is_empty(), "a closed hollow has no rim");

    let outputs: BTreeSet<usize> = outer_faces(&topo, hollow)
        .iter()
        .map(|f| f.index())
        .collect();
    assert_eq!(claimed_outputs(&map), outputs);
    // Every input face is both carried (modified) and offset (generated).
    let modified_sources: BTreeSet<usize> = map.modified.keys().copied().collect();
    let generated_sources: BTreeSet<usize> = map.generated.keys().copied().collect();
    assert_eq!(modified_sources, inputs);
    assert_eq!(generated_sources, inputs);
}

/// Shell with an opened face: the opened face is deleted and the rim
/// annulus is honestly unresolved with the opened face as candidate.
#[test]
fn opened_shell_evolution_reports_rim() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let top: Vec<FaceId> = outer_faces(&topo, cube)
        .into_iter()
        .filter(|&f| {
            matches!(
                topo.face(f).unwrap().surface(),
                remus_topology::face::FaceSurface::Plane { normal, .. }
                    if (*normal - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9
            )
        })
        .collect();
    assert_eq!(top.len(), 1);

    let (open_cup, map) =
        remus_operations::shell_op::shell_with_evolution(&mut topo, cube, 0.1, &top).unwrap();

    assert!(map.deleted.contains(&top[0].index()));
    assert!(
        !map.unresolved.is_empty(),
        "the rim annulus must be reported"
    );
    for candidates in map.unresolved.values() {
        assert_eq!(candidates, &vec![top[0].index()]);
    }
    let outputs: BTreeSet<usize> = outer_faces(&topo, open_cup)
        .iter()
        .map(|f| f.index())
        .collect();
    assert_eq!(claimed_outputs(&map), outputs);
}

/// The journaled wrappers record the same evolution into the topology
/// journal as real entries (not barriers): a journaled draft leaves a
/// journal whose latest entry is kind `draft`.
#[test]
fn journaled_wrappers_record_entries() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let wall = outer_faces(&topo, cube)
        .into_iter()
        .filter(|&f| {
            matches!(
                topo.face(f).unwrap().surface(),
                remus_topology::face::FaceSurface::Plane { normal, .. }
                    if (*normal - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-9
            )
        })
        .collect::<Vec<_>>();

    let out = remus_operations::journal_ops::draft_journaled(
        &mut topo,
        cube,
        &wall,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        5.0_f64.to_radians(),
    )
    .unwrap();
    assert!(out.map.origin.is_exact());
    assert!(!out.map.modified.is_empty());

    let mut topo2 = Topology::new();
    let cube2 = make_box(&mut topo2, 1.0, 1.0, 1.0).unwrap();
    let js = remus_operations::journal_ops::split_journaled(
        &mut topo2,
        cube2,
        Point3::new(0.0, 0.0, 0.5),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    assert!(!js.positive_map.modified.is_empty());
    assert!(!js.negative_map.modified.is_empty());
}
