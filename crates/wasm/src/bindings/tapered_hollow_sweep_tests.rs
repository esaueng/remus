//! Direct and both batch contracts for the bounded P-Class 7.1 hollow taper.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::{Point3, Vec3};
use remus_topology::face::{Face, FaceSurface};

use crate::handles::{face_id_to_u32, solid_id_to_u32};
use crate::kernel::BrepKernel;

fn fixture(kernel: &mut BrepKernel) -> (u32, u32) {
    let topo = kernel.topo_mut();
    let square = |half: f64| {
        vec![
            Point3::new(-half, -half, 0.0),
            Point3::new(half, -half, 0.0),
            Point3::new(half, half, 0.0),
            Point3::new(-half, half, 0.0),
        ]
    };
    let outer = remus_topology::builder::make_polygon_wire(topo, &square(2.0), 1e-7).unwrap();
    let mut inner = square(1.0);
    inner.reverse();
    let inner = remus_topology::builder::make_polygon_wire(topo, &inner, 1e-7).unwrap();
    let profile = topo.add_face(Face::new(
        outer,
        vec![inner],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    let path = kernel.make_line_edge(0.0, 0.0, 0.0, 0.0, 0.0, 6.0).unwrap();
    (face_id_to_u32(profile), path)
}

fn counts(kernel: &BrepKernel) -> [usize; 6] {
    let topo = kernel.topo();
    [
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.faces().len(),
        topo.num_shells(),
        topo.num_solids(),
    ]
}

fn assert_frustum(kernel: &BrepKernel, handle: u32) {
    let solid = kernel.resolve_solid(handle).unwrap();
    assert_eq!(solid_id_to_u32(solid), handle);
    let volume = remus_operations::measure::solid_volume(kernel.topo(), solid, 0.01).unwrap();
    assert!((volume - 168.0).abs() < 1e-8, "{volume} != 168");
    for (x, inside) in [(1.25, false), (2.5, true), (4.0, false)] {
        let state = remus_check::classify::classify_point(
            kernel.topo(),
            solid,
            Point3::new(x, 0.0, 4.5),
            &remus_check::classify::ClassifyOptions::default(),
        )
        .unwrap();
        assert_eq!(
            state == remus_check::classify::PointClassification::Inside,
            inside
        );
    }
    assert!(
        remus_operations::validate::validate_solid(kernel.topo(), solid)
            .unwrap()
            .is_valid()
    );
    let mesh = remus_operations::tessellate::tessellate_solid(kernel.topo(), solid, 0.01).unwrap();
    assert!(remus_operations::tessellate::is_watertight(&mesh));
    assert_eq!(
        remus_operations::tessellate::non_manifold_edge_count(&mesh),
        0
    );
}

#[test]
fn tapered_hollow_sweep_direct_and_both_batch_contracts() {
    let mut direct = BrepKernel::new();
    let (profile, path) = fixture(&mut direct);
    let result = direct
        .sweep_with_options(
            profile,
            path,
            "fixed",
            vec![0.0, 1.0, 1.0, 2.0],
            4,
            "smooth",
        )
        .unwrap();
    assert_frustum(&direct, result);

    for version_two in [false, true] {
        let mut kernel = BrepKernel::new();
        let (profile, path) = fixture(&mut kernel);
        let program = serde_json::json!([{"op": "sweepWithOptions", "args": {
            "profile": profile, "pathEdge": path, "contactMode": "fixed",
            "scaleValues": [0.0, 1.0, 1.0, 2.0], "segments": 4,
        }}])
        .to_string();
        let text = if version_two {
            kernel.execute_batch_v2(&program)
        } else {
            kernel.execute_batch(&program)
        };
        let response: serde_json::Value = serde_json::from_str(&text).unwrap();
        let handle = &response[0]["ok"];
        let handle =
            u32::try_from(handle.as_u64().unwrap_or_else(|| panic!("{response}"))).unwrap();
        assert_frustum(&kernel, handle);
    }
}

#[test]
fn tapered_hollow_sweep_refusal_keeps_live_state_and_history() {
    for version_two in [false, true] {
        let mut kernel = BrepKernel::new();
        let (profile, path) = fixture(&mut kernel);
        let before = counts(&kernel);
        let history = kernel.topo().journal().snapshot();
        let program = serde_json::json!([{"op": "sweepWithOptions", "args": {
            "profile": profile, "pathEdge": path,
            "scaleValues": [0.0, 1.0, 1.0, 0.0], "segments": 4,
        }}])
        .to_string();
        let text = if version_two {
            kernel.execute_batch_v2(&program)
        } else {
            kernel.execute_batch(&program)
        };
        let response: serde_json::Value = serde_json::from_str(&text).unwrap();
        let entry = &response[0];
        assert!(entry.get("error").is_some(), "{response}");
        if version_two {
            assert_eq!(entry["error"]["code"], "invalid_argument");
            assert_eq!(entry["error"]["category"], "invalid_input");
        }
        assert!(text.contains("finite positive scales"), "{text}");
        assert_eq!(counts(&kernel), before);
        assert_eq!(kernel.topo().journal().snapshot(), history);
    }
}

#[test]
fn tapered_hollow_sweep_scale_law_pairs_are_validated() {
    for values in [
        vec![0.0, 1.0],
        vec![0.0, 1.0, 1.0],
        vec![0.0, 1.0, 0.0, 2.0],
        vec![1.0, 1.0, 0.0, 2.0],
        vec![-1e308, 1.0, 1e308, 2.0],
        vec![0.0, 1.0, f64::NAN, 2.0],
        vec![0.0, 1.0, 1.0, f64::INFINITY],
        vec![0.0, 1.0, 1.0, 0.0],
    ] {
        assert!(
            super::parse_sweep_options("rmf", values.clone(), 4, "smooth").is_err(),
            "accepted malformed scale law {values:?}"
        );
    }
    assert!(super::parse_sweep_options("rmf", Vec::new(), 4, "smooth").is_ok());
    // Endpoint clamping allows finite parameter knots outside [0,1].
    let options =
        super::parse_sweep_options("rmf", vec![-1.0, 1.0, 2.0, 4.0], 4, "smooth").unwrap();
    assert!((options.scale_law.unwrap()(0.5) - 2.5).abs() < f64::EPSILON);
}

#[test]
fn sweep_native_domains_direct_and_both_batch_contracts() {
    for (a, b, segments) in [
        (10.0, 20.0, 4),
        (-20.0, -10.0, 4),
        (-2.0, 3.0, 4),
        (0.0, 2.0, 4),
        (-1.0, 1.0, 4),
        (1e16, 1e16 + 16.0, 16),
        (1e16, 1e16 + 64.0, 64),
        (0.0, 1e308, 16),
    ] {
        for route in 0..3 {
            let mut kernel = BrepKernel::new();
            let (profile, _) = fixture(&mut kernel);
            let edge = kernel
                .make_nurbs_edge_impl(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    6.0,
                    1,
                    vec![a, a, b, b],
                    vec![0.0, 0.0, 0.0, 0.0, 0.0, 6.0],
                    vec![1.0; 2],
                )
                .unwrap();
            let original = kernel.extract_nurbs_curve(edge).unwrap();
            let handle = if route == 0 {
                kernel
                    .sweep_with_options(
                        profile,
                        edge,
                        "fixed",
                        vec![0.0, 1.0, 1.0, 2.0],
                        segments,
                        "smooth",
                    )
                    .unwrap()
            } else {
                let program = serde_json::json!([{"op":"sweepWithOptions","args":{
                    "profile":profile,"pathEdge":edge,"contactMode":"fixed",
                    "scaleValues":[0,1,1,2],"segments":segments}}])
                .to_string();
                let response = if route == 1 {
                    kernel.execute_batch(&program)
                } else {
                    kernel.execute_batch_v2(&program)
                };
                let row: serde_json::Value = serde_json::from_str(&response).unwrap();
                u32::try_from(row[0]["ok"].as_u64().unwrap_or_else(|| panic!("{row}"))).unwrap()
            };
            assert_frustum(&kernel, handle);
            assert_eq!(kernel.extract_nurbs_curve(edge).unwrap(), original);
        }
    }
}

#[test]
fn guided_native_domain_mismatch_is_typed_and_keeps_checkpoint_ownership() {
    let mut kernel = BrepKernel::new();
    let (profile, spine_edge) = fixture(&mut kernel);
    let aux = kernel
        .make_nurbs_edge_impl(
            3.0,
            0.0,
            0.0,
            3.0,
            3.0,
            6.0,
            1,
            vec![10.0, 10.0, 20.0, 20.0],
            vec![3.0, 0.0, 0.0, 3.0, 3.0, 6.0],
            vec![1.0; 2],
        )
        .unwrap();
    let checkpoint = kernel.checkpoint().unwrap();
    let before = counts(&kernel);
    let memory = kernel.checkpoint_memory_stats().unwrap();
    let history = kernel.topo().journal().snapshot();
    let expected = "sweep_guided: unsupported configuration: main and auxiliary spines must have matching native parameter domains";
    let error = kernel
        .guided_sweep_impl(
            profile,
            (
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                &[0.0, 0.0, 0.0, 0.0, 0.0, 6.0],
                vec![1.0, 1.0],
            ),
            (
                1,
                vec![10.0, 10.0, 20.0, 20.0],
                &[3.0, 0.0, 0.0, 3.0, 3.0, 6.0],
                vec![1.0, 1.0],
            ),
        )
        .unwrap_err();
    assert_eq!(error.to_string(), expected);
    assert_eq!(kernel.checkpoint_memory_stats().unwrap(), memory);
    let structured = serde_json::to_value(crate::error::StructuredWasmError::from(error)).unwrap();
    assert_eq!(structured["code"], "operation_failed");
    assert_eq!(structured["category"], "unsupported");
    assert_eq!(structured["message"], expected);
    for v2 in [false, true] {
        let program = serde_json::json!([{"op":"guidedSweep", "args":{
            "face":profile,"spineEdge":spine_edge,"auxEdge":aux}}])
        .to_string();
        let response = if v2 {
            kernel.execute_batch_v2(&program)
        } else {
            kernel.execute_batch(&program)
        };
        let row: serde_json::Value = serde_json::from_str(&response).unwrap();
        if v2 {
            assert_eq!(row[0]["error"]["code"], "operation_failed");
            assert_eq!(row[0]["error"]["category"], "unsupported");
            assert_eq!(row[0]["error"]["message"], expected);
        } else {
            assert_eq!(row[0]["error"], expected);
        }
        assert_eq!(counts(&kernel), before);
        assert_eq!(kernel.checkpoint_memory_stats().unwrap(), memory);
        assert_eq!(kernel.topo().journal().snapshot(), history);
        assert_eq!(kernel.checkpoint_count(), 1);
    }
    kernel.restore(checkpoint).unwrap();
    assert_eq!(counts(&kernel), before);
    kernel.discard_checkpoint(checkpoint).unwrap();
    assert_eq!(kernel.checkpoint_count(), 0);
}

/// The law is positive at the k/64 preflight probes but refuses an actual
/// section. A single closed profile edge forces private splitting allocations
/// before that late refusal, so this exercises more than parser validation.
#[test]
fn sweep_late_refusal_preserves_shared_ownership_and_unshared_private_slot_safety() {
    use remus_topology::wire::{OrientedEdge, Wire};
    for checkpointed in [false, true] {
        let mut kernel = BrepKernel::new();
        let edge = remus_topology::builder::make_circle_edge(
            kernel.topo_mut(),
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            1e-7,
        )
        .unwrap();
        let wire = kernel
            .topo_mut()
            .add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
        let profile = kernel.topo_mut().add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let retained =
            remus_operations::primitives::make_box(kernel.topo_mut(), 1.0, 1.0, 1.0).unwrap();
        let handle = solid_id_to_u32(retained);
        let checkpoint = checkpointed.then(|| kernel.checkpoint().unwrap());
        let before = counts(&kernel);
        let history = kernel.topo().journal().snapshot();
        let memory = kernel.checkpoint_memory_stats().unwrap();
        let slots = kernel.topo().allocated_slot_count();
        let path = remus_math::nurbs::curve::NurbsCurve::new(
            1,
            vec![1e16, 1e16, 1e16 + 16.0, 1e16 + 16.0],
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 6.0)],
            vec![1.0; 2],
        )
        .unwrap();
        let options = remus_operations::sweep::SweepOptions {
            segments: 3,
            scale_law: Some(Box::new(|q| {
                if (q - 1.0 / 3.0).abs() < 1e-12 {
                    0.0
                } else {
                    1.0
                }
            })),
            ..Default::default()
        };
        let error = kernel
            .with_sweep_rollback(|kernel| {
                kernel.with_topology_transaction(|topo| {
                    remus_operations::sweep::sweep_with_options(topo, profile, &path, &options)
                })
            })
            .unwrap_err();
        assert!(matches!(
            error,
            remus_operations::OperationsError::InvalidInput { .. }
        ));
        assert_eq!(counts(&kernel), before);
        assert_eq!(kernel.topo().journal().snapshot(), history);
        assert_eq!(kernel.resolve_solid(handle).unwrap(), retained);
        assert_eq!(
            kernel.resolve_face(face_id_to_u32(profile)).unwrap(),
            profile
        );
        assert!(
            (remus_operations::measure::solid_volume(kernel.topo(), retained, 0.01).unwrap() - 1.0)
                .abs()
                < 1e-10
        );
        if let Some(checkpoint) = checkpoint {
            // No result handle was published on Err; the old checkpoint's
            // exact document owner and all live/retired-slot metrics survive.
            assert_eq!(kernel.checkpoint_memory_stats().unwrap(), memory);
            kernel.restore(checkpoint).unwrap();
            assert_eq!(kernel.resolve_solid(handle).unwrap(), retained);
            kernel.discard_checkpoint(checkpoint).unwrap();
        } else {
            // Unshared calls retain existing mutation-local high-water safety:
            // private failed allocation slots stay retired rather than reused.
            assert!(kernel.topo().allocated_slot_count() > slots);
            let old_memory: serde_json::Value = serde_json::from_str(&memory).unwrap();
            let new_memory: serde_json::Value =
                serde_json::from_str(&kernel.checkpoint_memory_stats().unwrap()).unwrap();
            assert!(
                new_memory["retiredSlots"].as_u64().unwrap()
                    > old_memory["retiredSlots"].as_u64().unwrap()
            );
            assert_eq!(kernel.checkpoint_count(), 0);
        }
    }
}
