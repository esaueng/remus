#![allow(clippy::unwrap_used)]
use crate::kernel::BrepKernel;

#[test]
fn coordinated_boolean_items_commit_independently_and_keep_session_state() {
    let mut k = BrepKernel::new();
    let a = k.make_box_solid(1.0, 1.0, 1.0).unwrap();
    let b = k.make_box_solid(1.0, 1.0, 1.0).unwrap();
    k.transform_solid_binding(
        b,
        vec![
            1.0, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ],
    )
    .unwrap();
    k.assembly_new("retained");
    let sketch = k.sketch_new();
    k.sketch_add_point(sketch, 1.0, 2.0, false).unwrap();
    k.gcs_new();
    let checkpoint = k.checkpoint();
    let input = k.serialize_solids(&[a, b]).unwrap();
    let session = format!(
        "{:?}{:?}{:?}{:?}{}",
        k.assemblies, k.sketches, k.gcs_sketches, k.checkpoints, k.poisoned
    );
    let mut retired = Vec::new();
    for _ in 0..3 {
        let rows: serde_json::Value = serde_json::from_str(&k.execute_batch(&format!(
            r#"[
            {{"op":"fuse","args":{{"solidA":{a},"solidB":{b}}}}},
            {{"op":"cut","args":{{"solidA":{a},"solidB":{a}}}}},
            {{"op":"fuse","args":{{"solidA":{a},"solidB":{b}}}}}
        ]"#
        )))
        .unwrap();
        assert!(rows[1]["error"].is_string());
        for row in [&rows[0], &rows[2]] {
            let id = u32::try_from(row["ok"].as_u64().unwrap()).unwrap();
            assert!((k.volume(id, 0.01).unwrap() - 1.5).abs() < 1e-8);
            retired.push(id);
        }
        assert_eq!(k.serialize_solids(&[a, b]).unwrap(), input);
        assert_eq!(
            format!(
                "{:?}{:?}{:?}{:?}{}",
                k.assemblies, k.sketches, k.gcs_sketches, k.checkpoints, k.poisoned
            ),
            session
        );
        k.restore(checkpoint).unwrap();
        k.make_box_solid(2.0, 2.0, 2.0).unwrap();
        for &id in &retired {
            assert!(k.resolve_solid(id).is_err());
        }
        k.restore(checkpoint).unwrap();
    }
    assert_eq!(k.serialize_solids(&[a, b]).unwrap(), input);
}
