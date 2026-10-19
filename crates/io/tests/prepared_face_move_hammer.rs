//! A prepared face move on the hammer holder's largest +X face against the
//! unprepared journaled move: the blend-aware translation path, its
//! remove/rebuild refusal (a torus support) and its own refusals past −7.2.
//!
//! Each distance is moved from a restored checkpoint in two copies of the
//! document, one unprepared and one prepared once, and the results must be
//! the same arena document byte for byte, with the same face history and
//! journal growth, or the same refusal text.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use remus_operations::journal_ops::{JournaledSolidOp, move_faces_journaled};
use remus_operations::measure;
use remus_operations::prepared_move::PreparedFaceMove;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

fn hammer() -> (Topology, SolidId, FaceId) {
    let mut topo = Topology::new();
    let solids = remus_io::step::reader::read_step(
        include_str!("data/shapr3d_hammer_holder.step"),
        &mut topo,
    )
    .unwrap();
    let solid = solids[0];
    let face = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            topo.face(f)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.x() > 0.999)
        })
        .max_by(|&a, &b| {
            let area = |f| measure::face_area(&topo, f, 0.08).unwrap();
            area(a).total_cmp(&area(b))
        })
        .unwrap();
    (topo, solid, face)
}

/// The result document, its face history with outputs by position in the
/// result, and how many journal entries the move added.
fn observed(
    topo: &Topology,
    entries_before: usize,
    moved: Result<JournaledSolidOp, remus_operations::OperationsError>,
) -> Result<(Vec<u8>, String, usize), String> {
    let moved = moved.map_err(|error| error.to_string())?;
    let position: BTreeMap<usize, usize> = solid_faces(topo, moved.solid)
        .unwrap()
        .iter()
        .enumerate()
        .map(|(i, f)| (f.index(), i))
        .collect();
    let mut modified: Vec<_> = moved
        .map
        .modified
        .iter()
        .map(|(input, outputs)| {
            (
                *input,
                outputs.iter().map(|o| position[o]).collect::<Vec<_>>(),
            )
        })
        .collect();
    modified.sort_unstable();
    Ok((
        remus_io::arena_io::serialize_solid(topo, moved.solid).unwrap(),
        format!(
            "{:?} {modified:?} unresolved {:?}",
            moved.map.origin,
            moved.map.unresolved.len()
        ),
        topo.journal().entries().len() - entries_before,
    ))
}

#[test]
fn prepared_moves_match_the_unprepared_journaled_move() {
    let (topo, solid, face) = hammer();
    let checkpoint = topo.clone();
    let source = remus_io::arena_io::serialize_solid(&topo, solid).unwrap();
    let mut plain = topo.clone();
    let mut prepared_topo = topo.clone();
    let mut prepared = PreparedFaceMove::prepare(&prepared_topo, solid, &[face]).unwrap();

    let distances = [-6.0, -6.8, -7.25, -7.6, -7.2, 0.5, 2.0, 0.0, f64::NAN, -6.4];
    let mut accepted = 0;
    let mut refused = Vec::new();
    for distance in distances {
        plain.restore_preserving_handle_slots(&checkpoint);
        prepared_topo.restore_preserving_handle_slots(&checkpoint);
        let entries = plain.journal().entries().len();
        assert_eq!(prepared_topo.journal().entries().len(), entries);

        let moved = move_faces_journaled(&mut plain, solid, &[face], distance);
        let expected = observed(&plain, entries, moved);
        assert!(prepared.is_current(&prepared_topo));
        let moved = prepared.move_faces_journaled(&mut prepared_topo, distance);
        let actual = observed(&prepared_topo, entries, moved);
        assert!(
            actual == expected,
            "distance {distance}: {actual:?} vs {expected:?}"
        );

        // A refusal leaves the source document exactly as it was (an
        // accepted move adds its journal entry, which the document carries).
        if expected.is_err() {
            for topo in [&plain, &prepared_topo] {
                assert!(remus_io::arena_io::serialize_solid(topo, solid).unwrap() == source);
            }
        }
        match expected {
            Ok(_) => accepted += 1,
            Err(reason) => refused.push((distance, reason)),
        }
    }
    assert!(accepted >= 5, "{accepted} accepted, refused {refused:?}");
    // The blend translation refuses past about −7.2 on this face, after the
    // remove/rebuild attempt refused on its torus support.
    assert!(
        refused.iter().any(|(d, reason)| *d <= -7.25
            && reason.contains("remove/rebuild failed")
            && reason.contains("torus")),
        "{refused:?}"
    );
}
