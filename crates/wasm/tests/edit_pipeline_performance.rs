//! Checkpoint accounting and consume-once outputs retain modeling behavior.
#![allow(clippy::unwrap_used)]
use remus_wasm::kernel::BrepKernel;

#[test]
fn checkpoint_accounting_distinguishes_shared_and_copied_arenas() {
    let mut kernel = BrepKernel::new();
    kernel.make_box_solid(2., 3., 4.).unwrap();
    let stats = |kernel: &BrepKernel| -> serde_json::Value {
        serde_json::from_str(&kernel.checkpoint_memory_stats().unwrap()).unwrap()
    };
    let original = stats(&kernel);
    let saved = kernel.checkpoint().unwrap();
    let shared = stats(&kernel);
    assert_eq!(original["estimatedBytes"], shared["estimatedBytes"]);
    assert_eq!(shared["uniqueTopologies"], 1);
    kernel.make_box_solid(4., 3., 2.).unwrap();
    let copied = stats(&kernel);
    assert_eq!(copied["uniqueTopologies"], 2);
    assert!(copied["estimatedBytes"].as_u64() > shared["estimatedBytes"].as_u64());
    kernel.restore(saved).unwrap();
    assert!(stats(&kernel)["retiredSlots"].as_u64().unwrap() > 0);
}

#[test]
fn grouped_output_is_consumed_once_without_changing_existing_getters() {
    let mut kernel = BrepKernel::new();
    let solid = kernel.make_box_solid(2., 3., 4.).unwrap();
    let mut mesh = kernel
        .tessellate_solid_grouped_binary(solid, 0.08, Some(0.06))
        .unwrap();
    let positions = mesh.positions();
    let indices = mesh.indices();
    let offsets = mesh.face_offsets();
    assert_eq!(mesh.take_positions(), positions);
    assert_eq!(mesh.take_indices(), indices);
    assert_eq!(mesh.take_face_offsets(), offsets);
    assert!(mesh.positions().is_empty());
    assert!(mesh.take_positions().is_empty());
    assert_eq!(
        kernel.volume(solid, 0.08).unwrap().to_bits(),
        24_f64.to_bits()
    );
}
