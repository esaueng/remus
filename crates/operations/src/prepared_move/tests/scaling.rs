//! Work-count guards for interactive face moves (PERF-M08 Phase 0).
//!
//! A face drag should pay for the edited body, not for the document around
//! it, and each tick of a drag should allocate the same amount. Both are
//! asserted with exact counts (undo records and arena slots), not timings, so
//! they are deterministic. They run in the ordinary suite and in the
//! `test(scaling_)` step.

use remus_topology::Topology;
use remus_topology::solid::SolidId;
use remus_topology::transaction::run_transacted;

use super::{PreparedFaceMove, filleted_box, pocketed_block, top_face};
use crate::{OperationsError, primitives, push_pull};

const BACKGROUND_BODIES: usize = 64;

type Fixture = fn() -> (Topology, SolidId);

/// The two fixtures cover the planar re-limitation path (a pocketed, bored
/// block) and the blend translation path (a fully filleted box).
fn fixtures() -> [(&'static str, Fixture); 2] {
    [
        ("pocketed block", pocketed_block),
        ("filleted box", filleted_box),
    ]
}

fn add_background(topo: &mut Topology, bodies: usize) {
    for _ in 0..bodies {
        primitives::make_box(topo, 10.0, 8.0, 6.0).unwrap();
    }
}

/// Undo records written and arena slots allocated by one move of `body`'s
/// top face, measured inside an enclosing scope so every write is retained.
fn move_writes(
    topo: &mut Topology,
    body: SolidId,
    distance: f64,
    prepared: bool,
) -> (usize, usize) {
    let face = top_face(topo, body);
    let mut session = prepared.then(|| PreparedFaceMove::prepare(topo, body, &[face]).unwrap());
    let slots = topo.allocated_slot_count();
    run_transacted(topo, |topo| -> Result<_, OperationsError> {
        match session.as_mut() {
            Some(session) => session.move_faces(topo, distance)?,
            None => push_pull::move_faces_with_evolution(topo, body, &[face], distance)?,
        };
        Ok((
            topo.undo_record_count(),
            topo.allocated_slot_count() - slots,
        ))
    })
    .unwrap()
}

#[test]
fn scaling_face_moves_write_only_the_edited_body() {
    for (name, fixture) in fixtures() {
        for prepared in [false, true] {
            let counts = |background: usize| {
                let (mut topo, body) = fixture();
                add_background(&mut topo, background);
                move_writes(&mut topo, body, 2.5, prepared)
            };
            let (alone, crowded) = (counts(0), counts(BACKGROUND_BODIES));
            assert!(alone.0 > 0, "{name}: no undo records were observed");
            assert_eq!(
                alone, crowded,
                "{name} (prepared {prepared}): a face move's writes grow with unrelated bodies"
            );
        }
    }
}

#[test]
fn scaling_prepared_drag_ticks_allocate_a_constant_number_of_slots() {
    for (name, fixture) in fixtures() {
        let ticks = |background: usize| {
            let (mut topo, body) = fixture();
            add_background(&mut topo, background);
            let face = top_face(&topo, body);
            let checkpoint = topo.clone();
            let mut session = PreparedFaceMove::prepare(&topo, body, &[face]).unwrap();
            (0..6)
                .map(|tick| {
                    let before_restore = topo.allocated_slot_count();
                    topo.restore_preserving_handle_slots(&checkpoint);
                    // Restore retires the previous tick's slots but never
                    // allocates: handles are not reused.
                    assert_eq!(topo.allocated_slot_count(), before_restore);
                    let distance = if tick % 2 == 0 { 2.0 } else { 2.5 };
                    session.move_faces(&mut topo, distance).unwrap();
                    topo.allocated_slot_count() - before_restore
                })
                .collect::<Vec<_>>()
        };
        let (alone, crowded) = (ticks(0), ticks(BACKGROUND_BODIES));
        assert!(alone[0] > 0, "{name}: the move allocated nothing");
        assert_eq!(
            alone,
            vec![alone[0]; alone.len()],
            "{name}: per-tick allocation drifts across a drag"
        );
        assert_eq!(
            alone, crowded,
            "{name}: per-tick allocation grows with unrelated bodies"
        );
    }
}
