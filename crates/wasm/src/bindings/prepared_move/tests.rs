#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::Vec3;
use remus_topology::explorer::solid_faces;

use super::MAX_PREPARED_MOVES;
use crate::handles::face_id_to_u32;
use crate::kernel::BrepKernel;

/// A box whose top face a drag moves: its handles are the same in every
/// kernel built the same way.
fn fixture(kernel: &mut BrepKernel) -> (u32, u32) {
    let solid = kernel.make_box_solid(10.0, 6.0, 4.0).unwrap();
    let id = kernel.resolve_solid(solid).unwrap();
    let face = solid_faces(kernel.topo(), id)
        .unwrap()
        .into_iter()
        .find(|&face| {
            kernel
                .topo()
                .face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.dot(Vec3::new(0.0, 0.0, 1.0)) > 0.999)
        })
        .unwrap();
    (solid, face_id_to_u32(face))
}

/// A drag in two kernels — `moveFacesJournaled` in one, the prepared move in
/// the other — restoring the same checkpoint before every frame: the same
/// JSON, the same result document, the same refusal text.
#[test]
fn prepared_drag_matches_move_faces_journaled_across_restores() {
    let mut plain = BrepKernel::new();
    let (source, face) = fixture(&mut plain);
    let plain_checkpoint = plain.checkpoint_impl().unwrap();
    let mut kernel = BrepKernel::new();
    assert_eq!(fixture(&mut kernel), (source, face));
    let checkpoint = kernel.checkpoint_impl().unwrap();
    let prepared = kernel.prepare_move_faces_impl(source, &[face]).unwrap();

    for distance in [2.0, -1.5, 0.5, -20.0, 0.0, 3.25, -3.999, -4.0] {
        plain.restore_checkpoint_impl(plain_checkpoint).unwrap();
        kernel.restore_checkpoint_impl(checkpoint).unwrap();
        let expected = plain
            .move_faces_journaled_json(source, &[face], distance)
            .map_err(|error| error.message().to_owned());
        let actual = kernel
            .move_faces_prepared_journaled_json(prepared, distance)
            .map_err(|error| error.message().to_owned());
        assert_eq!(actual, expected, "distance {distance}");
        if let Ok(value) = &expected {
            let solid = u32::try_from(value["solid"].as_u64().unwrap()).unwrap();
            assert_eq!(
                kernel.serialize_solids(&[solid]).unwrap(),
                plain.serialize_solids(&[solid]).unwrap()
            );
        }
        assert_eq!(
            kernel.serialize_solids(&[source]).unwrap(),
            plain.serialize_solids(&[source]).unwrap()
        );
    }

    // The unjournaled twin returns `moveFaces`' handle and document.
    plain.restore_checkpoint_impl(plain_checkpoint).unwrap();
    kernel.restore_checkpoint_impl(checkpoint).unwrap();
    let moved = kernel.move_faces_prepared_impl(prepared, 1.25).unwrap();
    let expected = plain.move_faces_binding(source, vec![face], 1.25).unwrap();
    assert_eq!(moved, expected);
    assert_eq!(
        kernel.serialize_solids(&[moved]).unwrap(),
        plain.serialize_solids(&[expected]).unwrap()
    );
}

#[test]
fn a_changed_source_refuses_until_the_drag_restores_it() {
    let mut kernel = BrepKernel::new();
    let (source, face) = fixture(&mut kernel);
    let checkpoint = kernel.checkpoint_impl().unwrap();
    let prepared = kernel.prepare_move_faces_impl(source, &[face]).unwrap();

    // Commit a move in place of the source: an edit of the source itself.
    let id = kernel.resolve_solid(source).unwrap();
    remus_operations::transform::transform_solid(
        kernel.topo_mut(),
        id,
        &remus_math::mat::Mat4::translation(0.0, 0.0, 1.0),
    )
    .unwrap();
    let before = kernel.serialize_solids(&[source]).unwrap();
    let stale = kernel
        .move_faces_prepared_journaled_json(prepared, 1.0)
        .unwrap_err();
    assert!(stale.message().contains("stale"), "{}", stale.message());
    assert!(kernel.move_faces_prepared_impl(prepared, 1.0).is_err());
    assert_eq!(kernel.serialize_solids(&[source]).unwrap(), before);

    kernel.restore_checkpoint_impl(checkpoint).unwrap();
    assert!(kernel.move_faces_prepared_impl(prepared, 1.0).is_ok());
}

#[test]
fn handles_are_bounded_released_and_never_reused() {
    let mut kernel = BrepKernel::new();
    let (source, face) = fixture(&mut kernel);
    let handles: Vec<u32> = (0..MAX_PREPARED_MOVES)
        .map(|_| kernel.prepare_move_faces_impl(source, &[face]).unwrap())
        .collect();
    assert!(kernel.prepare_move_faces_impl(source, &[face]).is_err());
    assert!(kernel.release_prepared_move(handles[3]));
    assert!(!kernel.release_prepared_move(handles[3]));
    assert!(kernel.move_faces_prepared_impl(handles[3], 1.0).is_err());
    let next = kernel.prepare_move_faces_impl(source, &[face]).unwrap();
    assert!(!handles.contains(&next));
    assert!(kernel.move_faces_prepared_impl(next, 1.0).is_ok());

    // Inputs are validated as `moveFaces` validates them.
    assert!(kernel.prepare_move_faces_impl(u32::MAX, &[face]).is_err());
    assert!(kernel.prepare_move_faces_impl(source, &[u32::MAX]).is_err());
    assert!(kernel.move_faces_prepared_impl(next, f64::NAN).is_err());
}
