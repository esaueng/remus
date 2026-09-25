//! Native mirror of the WASM smoke's hammer-holder opening replay
//! (`scripts/test-wasm-smoke.mjs`, "hammer opening replay"), step for step,
//! in ONE kernel topology like the smoke's single `BrepKernel`:
//! the STEP fixture enters through the translator's arena document
//! (`read_step` → `serialize_solids` → `deserialize_solids`), every STEP round
//! trip re-enters the same topology, and the shifted operand is a fresh
//! deserialization of the source bytes. Arena layout therefore matches the
//! smoke's, which is what the boolean's section order depends on.
//!
//! On `8539b266` (B39) the smoke's `shiftedCommon` intersect refused on
//! wasm32 (`ExactOnlyUnattainable`: 104 faces, 4 free edges) while this
//! native sequence stayed exact. The operands were bit-identical across
//! platforms (the WASM-computed arena bytes replay exact natively); the
//! difference was wasm32 libm rounding inside the intersect. The cause was
//! marched torus-FILLET-patch × cylinder traces landing exactly on
//! patch-boundary corners. The discriminating native check is
//! `remus_math::analytic_intersection::tests::torus_patch_marches_keep_their_band_exit`,
//! and the WASM smoke runs the real thing. This mirror pins the full smoke
//! sequence and its exact counts natively so neither side drifts silently.
#![allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]

use remus_io::arena_io::{deserialize_solids, serialize_solids};
use remus_io::step::reader::read_step;
use remus_io::step::{StepWriteOptions, write_step_bodies_with_options};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::{tessellate_solid_with_tolerance, welded_mesh_quality};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const FIXTURE: &str = include_str!("data/shapr3d_hammer_holder.step");

/// `io.importStep(text)` then `kernel.deserializeSolids(bytes)`.
fn import_step_into(kernel: &mut Topology, text: &str) -> Vec<SolidId> {
    let mut io = Topology::new();
    let solids = read_step(text, &mut io).expect("STEP import");
    let bytes = serialize_solids(&io, &solids).expect("arena document");
    deserialize_solids(&bytes, kernel).expect("deserialize")
}

/// `io.exportStep(kernel.serializeSolids([solid]))`.
fn export_step(kernel: &Topology, solid: SolidId) -> String {
    let bytes = serialize_solids(kernel, &[solid]).expect("serialize");
    let mut io = Topology::new();
    let solids = deserialize_solids(&bytes, &mut io).expect("translator load");
    write_step_bodies_with_options(&io, &solids, &[], &StepWriteOptions::default())
        .expect("STEP export")
}

/// STEP round trip back into the kernel topology.
fn round_trip(kernel: &mut Topology, solid: SolidId) -> SolidId {
    let step = export_step(kernel, solid);
    let restored = import_step_into(kernel, &step);
    assert_eq!(restored.len(), 1, "one body after the round trip");
    restored[0]
}

fn exact(kernel: &mut Topology, op: BooleanOp, a: SolidId, b: SolidId, what: &str) -> SolidId {
    let outcome = boolean_with_context(
        kernel,
        op,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap_or_else(|e| panic!("{what}: exact boolean refused: {e:?}"));
    assert_eq!(outcome.quality, BooleanQuality::Exact, "{what}");
    outcome.solid
}

fn translate(kernel: &mut Topology, solid: SolidId, x: f64, y: f64) {
    transform_solid(kernel, solid, &Mat4::translation(x, y, 0.0)).expect("translate");
}

/// `validateSolidDetailed(...).errorCount == 0` and
/// `meshQuality(0.05, 0.1)` watertight, plus the check crate's strict
/// validator (errors only).
fn assert_valid_watertight(kernel: &Topology, solid: SolidId, what: &str) {
    let ops = remus_operations::validate::validate_solid(kernel, solid).expect("validate");
    assert!(
        ops.is_valid(),
        "{what}: {:?}",
        ops.issues
            .iter()
            .map(|i| &i.description)
            .collect::<Vec<_>>()
    );
    let report = remus_check::validate::validate_solid(
        kernel,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .expect("check validate");
    let errors: Vec<_> = report
        .issues
        .iter()
        .filter(|i| i.severity == remus_check::validate::Severity::Error)
        .map(|i| (i.check, i.description.clone()))
        .collect();
    assert!(errors.is_empty(), "{what}: check crate: {errors:?}");
    let mesh = tessellate_solid_with_tolerance(kernel, solid, 0.05, 0.1).expect("mesh");
    let quality = welded_mesh_quality(&mesh);
    assert_eq!(quality.boundary_edges, 0, "{what}: boundary edges");
    assert_eq!(quality.non_manifold_edges, 0, "{what}: non-manifold edges");
    assert!(quality.is_watertight(), "{what}: watertight");
}

fn face_count(kernel: &Topology, solid: SolidId) -> usize {
    solid_faces(kernel, solid).expect("faces").len()
}

/// The smoke's `assertCut`: valid, watertight, both r = 2.5 mounting bores.
fn assert_cut(kernel: &Topology, solid: SolidId, what: &str) {
    assert_valid_watertight(kernel, solid, what);
    let bores = solid_faces(kernel, solid)
        .expect("faces")
        .into_iter()
        .filter(|&f| {
            matches!(kernel.face(f).expect("face").surface(),
                FaceSurface::Cylinder(c) if (c.radius() - 2.5).abs() < 1e-7)
        })
        .count();
    assert_eq!(bores, 2, "{what}: mounting bores");
}

fn volume(kernel: &Topology, solid: SolidId) -> f64 {
    solid_volume(kernel, solid, 0.01).expect("volume")
}

#[test]
#[allow(clippy::too_many_lines)]
fn wasm_smoke_hammer_opening_replay_sequence() {
    let mut k = Topology::new();
    let source = import_step_into(&mut k, FIXTURE)[0];
    let source_bytes = serialize_solids(&k, &[source]).expect("source bytes");

    let mask = make_box(&mut k, 29.0, 53.0, 70.0).expect("mask");
    translate(&mut k, mask, -18.0, -10.0);
    let result = exact(&mut k, BooleanOp::Cut, source, mask, "cut");
    assert_eq!(serialize_solids(&k, &[source]).unwrap(), source_bytes);
    assert_cut(&k, result, "cut");
    let imported = round_trip(&mut k, result);
    assert_cut(&k, imported, "cut round trip");

    let common = exact(&mut k, BooleanOp::Intersect, source, mask, "common");
    let restored_common = round_trip(&mut k, common);
    for (solid, what) in [(common, "common"), (restored_common, "common round trip")] {
        assert_eq!(face_count(&k, solid), 101, "{what}");
        assert_valid_watertight(&k, solid, what);
    }

    // The case that refused on 8539b266.
    let shifted = deserialize_solids(&source_bytes, &mut k).expect("shifted")[0];
    translate(&mut k, shifted, -2.0, 0.0);
    let shifted_common = exact(
        &mut k,
        BooleanOp::Intersect,
        common,
        shifted,
        "shifted common",
    );
    assert_eq!(serialize_solids(&k, &[source]).unwrap(), source_bytes);
    let restored_shifted = round_trip(&mut k, shifted_common);
    for (solid, what) in [
        (shifted_common, "shifted common"),
        (restored_shifted, "shifted common round trip"),
    ] {
        assert_eq!(face_count(&k, solid), 104, "{what}");
        assert_valid_watertight(&k, solid, what);
    }
    let shifted_volume = volume(&k, shifted_common);
    assert!(shifted_volume > 0.0 && shifted_volume < volume(&k, common));
    assert!((volume(&k, restored_shifted) - shifted_volume).abs() < shifted_volume * 1e-6);

    let fused = exact(&mut k, BooleanOp::Fuse, result, shifted_common, "fuse");
    let restored_fuse = round_trip(&mut k, fused);
    for (solid, what) in [(fused, "fuse"), (restored_fuse, "fuse round trip")] {
        assert_eq!(face_count(&k, solid), 177, "{what}");
        assert_cut(&k, solid, what);
    }
    let fused_volume = volume(&k, fused);
    assert!(
        (fused_volume - volume(&k, result) - shifted_volume).abs() < fused_volume * 1e-5,
        "fuse volume accounting"
    );

    let right_mask = make_box(&mut k, 29.0, 53.0, 70.0).expect("right mask");
    translate(&mut k, right_mask, 11.0, -10.0);
    let right_cut = exact(&mut k, BooleanOp::Cut, fused, right_mask, "right cut");
    let restored_right = round_trip(&mut k, right_cut);
    for (solid, what) in [
        (right_cut, "right cut"),
        (restored_right, "right round trip"),
    ] {
        assert_eq!(face_count(&k, solid), 162, "{what}");
        assert_cut(&k, solid, what);
    }
    let right_volume = volume(&k, right_cut);
    assert!(right_volume > 0.0 && right_volume < fused_volume);

    let right_inside = exact(
        &mut k,
        BooleanOp::Intersect,
        fused,
        right_mask,
        "right inside",
    );
    assert_eq!(face_count(&k, right_inside), 33);
    let shifted_right = deserialize_solids(&source_bytes, &mut k).expect("shifted right")[0];
    translate(&mut k, shifted_right, 2.0, 0.0);
    let right_common = exact(
        &mut k,
        BooleanOp::Intersect,
        right_inside,
        shifted_right,
        "right common",
    );
    assert_eq!(face_count(&k, right_common), 36);
    let completed = exact(
        &mut k,
        BooleanOp::Fuse,
        right_cut,
        right_common,
        "completed",
    );
    let restored_completed = round_trip(&mut k, completed);
    for (solid, what) in [
        (completed, "completed"),
        (restored_completed, "completed round trip"),
    ] {
        assert_eq!(face_count(&k, solid), 194, "{what}");
        assert_cut(&k, solid, what);
    }
    let completed_volume = volume(&k, completed);
    assert!(completed_volume > 0.0 && completed_volume < fused_volume);
    assert!(
        (completed_volume - right_volume - volume(&k, right_common)).abs()
            < completed_volume * 1e-5
    );
    assert_eq!(serialize_solids(&k, &[source]).unwrap(), source_bytes);
}
