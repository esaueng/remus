#![allow(clippy::unwrap_used, clippy::expect_used)]
use crate::kernel::test_fixtures::{kernel_with_box, kernel_with_cylinder};
use remus_math::vec::Vec3;
use serde_json::{Value, json};

#[test]
fn draft_box_closed_form_batch_and_helper_preserve_checkpoint_state() {
    for v2 in [false, true] {
        let (mut kernel, solid) = kernel_with_box();
        let checkpoint = kernel.checkpoint().unwrap();
        let ticks = kernel.topo.mutation_ticks();
        let before = kernel.serialize_solids(&[solid]).unwrap();
        let journal = kernel.journal_summary();
        let pull = Vec3::new(1.0, 2.0, 3.0);
        let expected = kernel.planar_draft_result(solid, pull).unwrap();
        assert_eq!(expected.len(), 6);
        for row in &expected {
            let n = &row.outward_normal;
            let dot = (n[0] + 2.0 * n[1] + 3.0 * n[2]) / 14.0_f64.sqrt();
            assert!((row.angle_radians - dot.asin()).abs() < 1e-14);
            let sample = kernel
                .face_draft_result(row.face, 19.0, -21.0, pull)
                .unwrap();
            assert_eq!(sample.angle_radians.to_bits(), row.angle_radians.to_bits());
            assert_eq!(sample.outward_normal, row.outward_normal);
        }
        let input = json!([{ "op":"getPlanarSolidDraft", "args":{
            "solid":solid,"pullX":1,"pullY":2,"pullZ":3
        }}])
        .to_string();
        let output = if v2 {
            kernel.execute_batch_v2(&input)
        } else {
            kernel.execute_batch(&input)
        };
        let rows: Value = serde_json::from_str(&output).unwrap();
        assert_eq!(rows[0]["ok"], serde_json::to_value(&expected).unwrap());
        assert_eq!(kernel.serialize_solids(&[solid]).unwrap(), before);
        assert_eq!(kernel.journal_summary(), journal);
        assert_eq!(kernel.topo.mutation_ticks(), ticks);
        assert_eq!(kernel.checkpoint_count(), 1);
        kernel.restore(checkpoint).unwrap();
        assert_eq!(kernel.serialize_solids(&[solid]).unwrap(), before);
        kernel.discard_checkpoint(checkpoint).unwrap();
    }
}

#[test]
fn draft_invalid_inputs_and_curved_report_are_typed_refusals() {
    let (mut kernel, solid) = kernel_with_cylinder();
    let before = kernel.serialize_solids(&[solid]).unwrap();
    let error = kernel
        .planar_draft_result(solid, Vec3::new(0.0, 0.0, 1.0))
        .err()
        .unwrap();
    assert!(
        error
            .message()
            .starts_with("draft-analysis:non-planar-face:")
    );
    let face = kernel.get_solid_faces(solid).unwrap()[0];
    for pull in [Vec3::new(0.0, 0.0, 0.0), Vec3::new(f64::NAN, 0.0, 1.0)] {
        assert!(
            kernel
                .face_draft_result(face, 0.0, 0.0, pull)
                .err()
                .unwrap()
                .message()
                .starts_with("draft-analysis:invalid-pull:")
        );
    }
    assert!(
        kernel
            .face_draft_result(u32::MAX, 0.0, 0.0, Vec3::new(0.0, 0.0, 1.0))
            .is_err()
    );
    for v2 in [false, true] {
        let input = json!([{ "op":"getPlanarSolidDraft", "args":{
            "solid":solid,"pullX":0,"pullY":0,"pullZ":1
        }}])
        .to_string();
        let output = if v2 {
            kernel.execute_batch_v2(&input)
        } else {
            kernel.execute_batch(&input)
        };
        let rows: Value = serde_json::from_str(&output).unwrap();
        assert!(rows[0].get("error").is_some());
        if v2 {
            assert_eq!(
                rows[0]["error"]["details"]["kernelCode"],
                "draft-analysis:non-planar-face"
            );
        }
    }
    assert_eq!(kernel.serialize_solids(&[solid]).unwrap(), before);
}
