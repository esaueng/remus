//! Contract tests for the O4.7 modifier twins.
//!
//! The direct side runs the natively-testable `*_detailed_impl` bodies (a
//! `JsError` cannot be built off-wasm); the batch side goes through
//! `execute_batch` / `execute_batch_v2`. Every refusal is checked against the
//! legacy op's `executeBatchV2` code and category and for a byte-for-byte
//! unchanged topology.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashSet;

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};
use serde_json::{Value, json};

use crate::handles::{edge_id_to_u32, face_id_to_u32, solid_id_to_u32};
use crate::kernel::BrepKernel;
use crate::types::SolidOperationDetailedResult;

type Counts = (usize, usize, usize, usize, usize);

fn counts(kernel: &BrepKernel) -> Counts {
    let topo = kernel.topo();
    (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_faces(),
        topo.num_solids(),
        topo.allocated_slot_count(),
    )
}

/// Live entity counts, without the handle high-water mark: a rollback after
/// an attempt that allocated keeps those slots retired so no stale handle
/// can alias a later entity.
fn live_counts(kernel: &BrepKernel) -> (usize, usize, usize, usize) {
    let (vertices, edges, faces, solids, _) = counts(kernel);
    (vertices, edges, faces, solids)
}

fn envelope(result: SolidOperationDetailedResult) -> Value {
    serde_json::to_value(result).unwrap()
}

fn batch_v2(kernel: &mut BrepKernel, ops: &Value) -> Value {
    serde_json::from_str(&kernel.execute_batch_v2(&ops.to_string())).unwrap()
}

fn batch_legacy(kernel: &mut BrepKernel, ops: &Value) -> Value {
    serde_json::from_str(&kernel.execute_batch(&ops.to_string())).unwrap()
}

/// Native code a legacy `executeBatchV2` error projects: `kernelCode` when
/// present, the wire code otherwise (the O4.7 direct-result rule).
fn v2_code(error: &Value) -> &str {
    error["details"]["kernelCode"]
        .as_str()
        .or_else(|| error["code"].as_str())
        .unwrap()
}

fn volume(kernel: &BrepKernel, solid: u32) -> f64 {
    kernel.volume(solid, 0.1).unwrap()
}

fn handle(value: &Value) -> u32 {
    u32::try_from(value.as_u64().unwrap()).unwrap()
}

/// The four modifier twins, their legacy batch ops, and the argument shapes
/// both take, over a 10³ box whose first edge and first face are selected.
#[derive(Clone, Copy, Debug)]
enum Twin {
    Fillet,
    Chamfer,
    Shell,
    Offset,
}

impl Twin {
    const ALL: [Self; 4] = [Self::Fillet, Self::Chamfer, Self::Shell, Self::Offset];

    const fn detailed_op(self) -> &'static str {
        match self {
            Self::Fillet => "filletDetailed",
            Self::Chamfer => "chamferDetailed",
            Self::Shell => "shellDetailed",
            Self::Offset => "offsetDetailed",
        }
    }

    const fn legacy_op(self) -> &'static str {
        match self {
            Self::Fillet => "fillet",
            Self::Chamfer => "chamfer",
            Self::Shell => "shell",
            Self::Offset => "offsetSolid",
        }
    }

    const fn operation(self) -> &'static str {
        match self {
            Self::Fillet => "fillet",
            Self::Chamfer => "chamfer",
            Self::Shell => "shell",
            Self::Offset => "offset",
        }
    }

    fn args(self, solid: u32, edge: u32, face: u32) -> Value {
        match self {
            Self::Fillet => json!({"solid": solid, "edges": [edge], "radius": 1.0}),
            Self::Chamfer => json!({"solid": solid, "edges": [edge], "distance": 1.0}),
            Self::Shell => json!({"solid": solid, "thickness": 1.0, "faces": [face]}),
            Self::Offset => json!({"solid": solid, "distance": 1.0}),
        }
    }

    fn direct(self, kernel: &mut BrepKernel, solid: u32, edge: u32, face: u32) -> Value {
        envelope(match self {
            Self::Fillet => kernel.fillet_detailed_impl(solid, &[edge], 1.0, false),
            Self::Chamfer => kernel.chamfer_detailed_impl(solid, &[edge], 1.0, false),
            Self::Shell => kernel.shell_detailed_impl(solid, 1.0, &[face], None),
            Self::Offset => kernel.offset_detailed_impl(solid, 1.0, false),
        })
    }

    /// Closed-form volume of the twin applied to the 10³ box.
    fn expected_volume(self) -> f64 {
        match self {
            Self::Fillet => (1.0 - std::f64::consts::FRAC_PI_4).mul_add(-10.0, 1000.0),
            Self::Chamfer => 995.0,
            Self::Shell => 1000.0 - 8.0 * 8.0 * 9.0,
            Self::Offset => 1728.0,
        }
    }
}

/// A fresh kernel holding one 10³ box and its first edge and face.
fn box_kernel() -> (BrepKernel, u32, u32, u32) {
    let mut kernel = BrepKernel::new();
    let solid = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
    let solid_id = kernel.resolve_solid(solid).unwrap();
    let edge =
        edge_id_to_u32(remus_topology::explorer::solid_edges(kernel.topo(), solid_id).unwrap()[0]);
    let face =
        face_id_to_u32(remus_topology::explorer::solid_faces(kernel.topo(), solid_id).unwrap()[0]);
    (kernel, solid, edge, face)
}

#[test]
fn exact_twins_match_both_batch_contracts_and_the_legacy_geometry() {
    for twin in Twin::ALL {
        let (mut direct_kernel, solid, edge, face) = box_kernel();
        let direct = twin.direct(&mut direct_kernel, solid, edge, face);
        assert_eq!(direct["status"], "ok", "{twin:?}: {direct}");
        assert!(direct["code"].is_null());
        assert!(direct["category"].is_null());
        assert_eq!(direct["details"]["quality"], "exact", "{twin:?}: {direct}");
        let expected_engine = match twin {
            Twin::Fillet => Some("rollingBall"),
            Twin::Chamfer => Some("planarBevel"),
            Twin::Shell | Twin::Offset => None,
        };
        assert_eq!(
            direct["details"]["engine"].as_str(),
            expected_engine,
            "{twin:?}: {direct}"
        );
        let result = handle(&direct["value"]);
        let direct_volume = volume(&direct_kernel, result);
        assert!(
            (direct_volume - twin.expected_volume()).abs() < 1e-6,
            "{twin:?}: volume {direct_volume} vs closed form {}",
            twin.expected_volume()
        );

        // The batch op of the same name carries the identical envelope as
        // its `ok` value under both batch contracts.
        let ops = json!([
            {"op": "makeBox", "args": {"width": 10, "height": 10, "depth": 10}},
            {"op": twin.detailed_op(), "args": twin.args(solid, edge, face)},
        ]);
        let mut v2_kernel = BrepKernel::new();
        let v2 = batch_v2(&mut v2_kernel, &ops);
        assert_eq!(
            v2[1]["ok"], direct,
            "{twin:?}: direct/executeBatchV2 parity"
        );
        let legacy = batch_legacy(&mut BrepKernel::new(), &ops);
        assert_eq!(
            legacy[1]["ok"], direct,
            "{twin:?}: direct/executeBatch parity"
        );

        // Same engine path as the legacy op: identical committed geometry.
        let (mut legacy_kernel, ..) = box_kernel();
        let legacy = batch_v2(
            &mut legacy_kernel,
            &json!([{"op": twin.legacy_op(), "args": twin.args(solid, edge, face)}]),
        );
        let legacy_solid = handle(&legacy[0]["ok"]);
        assert_eq!(legacy_solid, result, "{twin:?}: same handle allocation");
        assert!(
            (volume(&legacy_kernel, legacy_solid) - direct_volume).abs() < 1e-9,
            "{twin:?}: legacy and detailed geometry diverge"
        );
    }
}

#[test]
fn invalid_handle_refusals_match_legacy_batch_v2_and_mutate_nothing() {
    for twin in Twin::ALL {
        let (mut kernel, solid, edge, face) = box_kernel();
        let invalid = u32::MAX;
        // An invalid solid for every twin, plus an invalid selected entity.
        let mut cases = vec![("solid", invalid, edge, face)];
        match twin {
            Twin::Fillet | Twin::Chamfer => cases.push(("edge", solid, invalid, face)),
            Twin::Shell => cases.push(("face", solid, edge, invalid)),
            Twin::Offset => {}
        }
        for (entity, s, e, f) in cases {
            let before = counts(&kernel);
            let direct = twin.direct(&mut kernel, s, e, f);
            assert_eq!(direct["status"], "error", "{twin:?}/{entity}: {direct}");
            assert_eq!(direct["code"], "invalid_handle");
            assert_eq!(direct["category"], "invalid_input");
            assert!(direct["value"].is_null());
            assert_eq!(direct["details"]["operation"], twin.operation());
            assert_eq!(direct["details"]["entity"], entity);
            assert_eq!(direct["details"]["index"], u64::from(invalid));
            assert!(direct["details"]["message"].is_string());
            assert_eq!(counts(&kernel), before, "{twin:?}/{entity}: mutated");

            let legacy = batch_v2(
                &mut kernel,
                &json!([{"op": twin.legacy_op(), "args": twin.args(s, e, f)}]),
            );
            let legacy_error = &legacy[0]["error"];
            assert_eq!(direct["code"].as_str().unwrap(), v2_code(legacy_error));
            assert_eq!(direct["category"], legacy_error["category"]);

            let batch = batch_v2(
                &mut kernel,
                &json!([{"op": twin.detailed_op(), "args": twin.args(s, e, f)}]),
            );
            assert_eq!(
                batch[0]["ok"], direct,
                "{twin:?}/{entity}: batch twin parity"
            );
            assert_eq!(counts(&kernel), before, "{twin:?}/{entity}: batch mutated");
        }
    }
}

#[test]
fn blend_engine_refusals_match_legacy_batch_v2_and_roll_back() {
    for (twin, op, size_key) in [
        (Twin::Fillet, "fillet", "radius"),
        (Twin::Chamfer, "chamfer", "distance"),
    ] {
        let (mut kernel, solid, edge, _) = box_kernel();
        let before = counts(&kernel);
        let before_volume = volume(&kernel, solid);
        // Larger than the box: every engine refuses.
        let direct = envelope(match twin {
            Twin::Fillet => kernel.fillet_detailed_impl(solid, &[edge], 20.0, false),
            _ => kernel.chamfer_detailed_impl(solid, &[edge], 20.0, false),
        });
        assert_eq!(direct["status"], "error", "{op}: {direct}");
        assert!(direct["value"].is_null());
        assert_eq!(direct["details"]["operation"], op);
        assert_eq!(counts(&kernel), before, "{op}: refusal must roll back");
        assert!((volume(&kernel, solid) - before_volume).abs() < 1e-12);

        let args = json!({"solid": solid, "edges": [edge], size_key: 20.0});
        let legacy = batch_v2(&mut kernel, &json!([{"op": op, "args": args}]));
        let legacy_error = &legacy[0]["error"];
        assert_eq!(
            direct["code"].as_str().unwrap(),
            v2_code(legacy_error),
            "{op}: direct={direct} legacy={legacy_error}"
        );
        assert_eq!(direct["category"], legacy_error["category"]);

        let batch = batch_v2(
            &mut kernel,
            &json!([{"op": twin.detailed_op(), "args": args}]),
        );
        assert_eq!(batch[0]["ok"], direct, "{op}: batch twin parity");
        assert_eq!(counts(&kernel), before);
    }
}

#[test]
fn argument_refusals_are_typed_data() {
    let (mut kernel, solid, edge, face) = box_kernel();
    let before = counts(&kernel);
    for (label, direct) in [
        (
            "fillet radius",
            envelope(kernel.fillet_detailed_impl(solid, &[edge], 0.0, false)),
        ),
        (
            "chamfer distance",
            envelope(kernel.chamfer_detailed_impl(solid, &[edge], -1.0, false)),
        ),
        (
            "shell thickness",
            envelope(kernel.shell_detailed_impl(solid, f64::NAN, &[face], None)),
        ),
        (
            "shell spacing",
            envelope(kernel.shell_detailed_impl(solid, 1.0, &[face], Some(0.0))),
        ),
        (
            "offset distance",
            envelope(kernel.offset_detailed_impl(solid, f64::INFINITY, false)),
        ),
    ] {
        assert_eq!(direct["status"], "error", "{label}: {direct}");
        assert_eq!(direct["code"], "invalid_argument", "{label}: {direct}");
        assert_eq!(direct["category"], "invalid_input", "{label}: {direct}");
    }
    assert_eq!(counts(&kernel), before);

    // Batch argument parsing refuses a mistyped option before dispatch.
    let response = batch_v2(
        &mut kernel,
        &json!([{"op": "offsetDetailed", "args": {"solid": solid, "distance": 1, "exactOnly": "yes"}}]),
    );
    assert_eq!(response[0]["error"]["code"], "invalid_argument");
    assert_eq!(response[0]["error"]["details"]["argument"], "exactOnly");
    let response = batch_v2(
        &mut kernel,
        &json!([{"op": "shellDetailed", "args": {"solid": solid, "thickness": 1, "approximationSpacing": "fine"}}]),
    );
    assert_eq!(response[0]["error"]["code"], "invalid_argument");
    assert_eq!(
        response[0]["error"]["details"]["argument"],
        "approximationSpacing"
    );
    assert_eq!(counts(&kernel), before);
}

#[test]
fn poisoned_kernel_refuses_every_twin_as_data() {
    let (mut kernel, solid, edge, face) = box_kernel();
    kernel.poisoned = true;
    let before = counts(&kernel);
    for twin in Twin::ALL {
        let direct = twin.direct(&mut kernel, solid, edge, face);
        assert_eq!(direct["status"], "error", "{twin:?}: {direct}");
        assert_eq!(direct["code"], "operation_failed");
        assert_eq!(direct["category"], "internal");
        assert!(
            direct["details"]["message"]
                .as_str()
                .unwrap()
                .contains("poisoned")
        );
    }
    assert_eq!(counts(&kernel), before);
}

// ── Approximation: disclosure and exact-only refusal ─────────────────────

/// Two crossed cylinders (radii 3 and 2, so the saddle is a simple loop, not
/// the equal-radius figure eight). Rounding the saddle intersection needs a
/// walking-engine NURBS wall with no closed form.
fn crossed_cylinders(kernel: &mut BrepKernel) -> (u32, u32) {
    let response = batch_v2(
        kernel,
        &json!([
            {"op": "makeCylinder", "args": {"radius": 3, "height": 10}},
            {"op": "makeCylinder", "args": {"radius": 2, "height": 16}},
            {"op": "transform", "args": {"solid": 1, "matrix": [
                1, 0, 0, 0,
                0, 0, -1, 8,
                0, 1, 0, 5,
                0, 0, 0, 1
            ]}},
            {"op": "fuse", "args": {"solidA": 0, "solidB": 1}},
        ]),
    );
    let solid = handle(&response[3]["ok"]);
    let solid_id = kernel.resolve_solid(solid).unwrap();
    let topo = kernel.topo();
    let saddle = remus_topology::explorer::solid_edges(topo, solid_id)
        .unwrap()
        .into_iter()
        .find(|&edge| matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::NurbsCurve(_)))
        .expect("the fused cylinders carry a NURBS saddle edge");
    (solid, edge_id_to_u32(saddle))
}

#[test]
fn saddle_fillet_discloses_its_nurbs_wall_and_exact_only_refuses_it() {
    let mut kernel = BrepKernel::new();
    let (solid, saddle) = crossed_cylinders(&mut kernel);
    let solid_id = kernel.resolve_solid(solid).unwrap();
    let input_faces: HashSet<FaceId> =
        remus_topology::explorer::solid_faces(kernel.topo(), solid_id)
            .unwrap()
            .into_iter()
            .collect();

    // Exact-only: the NURBS wall is built, found, rolled back and refused.
    let before = live_counts(&kernel);
    let before_faces = remus_topology::explorer::solid_faces(kernel.topo(), solid_id).unwrap();
    let before_volume = volume(&kernel, solid);
    let refused = envelope(kernel.fillet_detailed_impl(solid, &[saddle], 0.5, true));
    assert_eq!(refused["status"], "error", "{refused}");
    assert_eq!(refused["code"], "exact_only_unattainable");
    assert_eq!(refused["category"], "quality_refused");
    assert_eq!(refused["details"]["operation"], "fillet");
    assert_eq!(refused["details"]["engine"], "walking");
    assert!(refused["details"]["approximateFaceCount"].as_u64().unwrap() >= 1);
    assert!(refused["value"].is_null());
    assert_eq!(
        live_counts(&kernel),
        before,
        "exact-only refusal must roll back"
    );
    assert_eq!(
        remus_topology::explorer::solid_faces(kernel.topo(), solid_id).unwrap(),
        before_faces,
        "the input solid keeps its exact face list"
    );
    assert!((volume(&kernel, solid) - before_volume).abs() < 1e-12);

    // Permissive: the same call commits and names every new NURBS face.
    let disclosed = envelope(kernel.fillet_detailed_impl(solid, &[saddle], 0.5, false));
    assert_eq!(disclosed["status"], "ok", "{disclosed}");
    assert_eq!(disclosed["details"]["quality"], "approximate");
    assert_eq!(disclosed["details"]["engine"], "walking");
    let result = kernel.resolve_solid(handle(&disclosed["value"])).unwrap();
    let approximate: HashSet<u32> = disclosed["details"]["approximateFaces"]
        .as_array()
        .unwrap()
        .iter()
        .map(handle)
        .collect();
    let new_nurbs: HashSet<u32> = remus_topology::explorer::solid_faces(kernel.topo(), result)
        .unwrap()
        .into_iter()
        .filter(|face| {
            !input_faces.contains(face)
                && matches!(
                    kernel.topo().face(*face).unwrap().surface(),
                    FaceSurface::Nurbs(_)
                )
        })
        .map(face_id_to_u32)
        .collect();
    assert!(!approximate.is_empty());
    assert_eq!(approximate, new_nurbs, "every new NURBS face is disclosed");

    // The batch twin, replayed on an identical kernel, is the same envelope.
    let mut batch_kernel = BrepKernel::new();
    let (batch_solid, batch_saddle) = crossed_cylinders(&mut batch_kernel);
    assert_eq!((batch_solid, batch_saddle), (solid, saddle));
    let batch = batch_v2(
        &mut batch_kernel,
        &json!([
            {"op": "filletDetailed", "args": {"solid": solid, "edges": [saddle], "radius": 0.5, "exactOnly": true}},
            {"op": "filletDetailed", "args": {"solid": solid, "edges": [saddle], "radius": 0.5}},
        ]),
    );
    assert_eq!(batch[0]["ok"], refused);
    assert_eq!(batch[1]["ok"], disclosed);
}

/// A `loft_smooth` solid with four curved NURBS side walls and planar caps
/// (the fixture `shell_op`'s own NURBS-policy test uses).
fn nurbs_loft(kernel: &mut BrepKernel) -> u32 {
    fn square_at(topo: &mut Topology, size: f64, z: f64) -> FaceId {
        let half = size / 2.0;
        let vertices: Vec<_> = [(-half, -half), (half, -half), (half, half), (-half, half)]
            .iter()
            .map(|&(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, z), 1e-7)))
            .collect();
        let edges: Vec<_> = (0..4)
            .map(|i| {
                topo.add_edge(Edge::new(
                    vertices[i],
                    vertices[(i + 1) % 4],
                    EdgeCurve::Line,
                ))
            })
            .collect();
        let wire = topo.add_wire(
            Wire::new(
                edges
                    .iter()
                    .map(|&edge| OrientedEdge::new(edge, true))
                    .collect(),
                true,
            )
            .unwrap(),
        );
        topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: z,
            },
        ))
    }

    let topo = kernel.topo_mut();
    let profiles = [
        square_at(topo, 6.0, 0.0),
        square_at(topo, 3.0, 5.0),
        square_at(topo, 6.0, 10.0),
    ];
    solid_id_to_u32(remus_operations::loft::loft_smooth(topo, &profiles).unwrap())
}

fn nurbs_face_handles(kernel: &BrepKernel, solid: u32) -> Vec<u32> {
    let solid_id = kernel.resolve_solid(solid).unwrap();
    remus_topology::explorer::solid_faces(kernel.topo(), solid_id)
        .unwrap()
        .into_iter()
        .filter(|face| {
            matches!(
                kernel.topo().face(*face).unwrap().surface(),
                FaceSurface::Nurbs(_)
            )
        })
        .map(face_id_to_u32)
        .collect()
}

#[test]
fn shell_twin_keeps_the_legacy_exact_only_refusal_and_discloses_opt_in() {
    let mut kernel = BrepKernel::new();
    let solid = nurbs_loft(&mut kernel);
    let nurbs = nurbs_face_handles(&kernel, solid);
    assert_eq!(nurbs.len(), 4);

    // Exact-only by default, like `shell`: the same refusal, rolled back.
    let before = counts(&kernel);
    let refused = envelope(kernel.shell_detailed_impl(solid, 0.3, &[], None));
    assert_eq!(refused["status"], "error", "{refused}");
    assert!(
        refused["details"]["message"]
            .as_str()
            .unwrap()
            .contains("NURBS")
    );
    assert_eq!(counts(&kernel), before);
    let legacy = batch_v2(
        &mut kernel,
        &json!([{"op": "shell", "args": {"solid": solid, "thickness": 0.3}}]),
    );
    let legacy_error = &legacy[0]["error"];
    assert_eq!(refused["code"].as_str().unwrap(), v2_code(legacy_error));
    assert_eq!(refused["category"], legacy_error["category"]);

    // Opt-in spacing commits the sampled inner skin and discloses it.
    let disclosed = envelope(kernel.shell_detailed_impl(solid, 0.3, &[], Some(0.1)));
    assert_eq!(disclosed["status"], "ok", "{disclosed}");
    assert_eq!(disclosed["details"]["quality"], "approximate");
    assert_eq!(disclosed["details"]["deflection"], 0.1);
    let sampled: Vec<u32> = disclosed["details"]["sampledFaces"]
        .as_array()
        .unwrap()
        .iter()
        .map(handle)
        .collect();
    assert_eq!(
        sampled, nurbs,
        "sampledFaces are the input NURBS face handles"
    );

    let mut batch_kernel = BrepKernel::new();
    assert_eq!(nurbs_loft(&mut batch_kernel), solid);
    let batch = batch_v2(
        &mut batch_kernel,
        &json!([
            {"op": "shellDetailed", "args": {"solid": solid, "thickness": 0.3}},
            {"op": "shellDetailed", "args": {"solid": solid, "thickness": 0.3, "approximationSpacing": 0.1}},
        ]),
    );
    assert_eq!(batch[0]["ok"], refused);
    assert_eq!(batch[1]["ok"], disclosed);
}

#[test]
fn offset_exact_only_refuses_sampled_nurbs_before_touching_topology() {
    let mut kernel = BrepKernel::new();
    let solid = nurbs_loft(&mut kernel);
    let nurbs = nurbs_face_handles(&kernel, solid);
    let before = counts(&kernel);

    let refused = envelope(kernel.offset_detailed_impl(solid, 0.3, true));
    assert_eq!(refused["status"], "error", "{refused}");
    assert_eq!(refused["code"], "exact_only_unattainable");
    assert_eq!(refused["category"], "quality_refused");
    assert_eq!(refused["details"]["operation"], "offset");
    let sampled: Vec<u32> = refused["details"]["sampledFaces"]
        .as_array()
        .unwrap()
        .iter()
        .map(handle)
        .collect();
    assert_eq!(sampled, nurbs);
    assert_eq!(counts(&kernel), before);

    // Without exactOnly the engine runs and refuses on its own terms (the
    // offset intersector has no NURBS pair yet); that refusal is the legacy
    // op's and is rolled back too.
    let engine = envelope(kernel.offset_detailed_impl(solid, 0.3, false));
    assert_eq!(engine["status"], "error", "{engine}");
    assert_eq!(counts(&kernel), before);
    let legacy = batch_v2(
        &mut kernel,
        &json!([{"op": "offsetSolid", "args": {"solid": solid, "distance": 0.3}}]),
    );
    let legacy_error = &legacy[0]["error"];
    assert_eq!(engine["code"].as_str().unwrap(), v2_code(legacy_error));
    assert_eq!(engine["category"], legacy_error["category"]);

    let batch = batch_v2(
        &mut kernel,
        &json!([
            {"op": "offsetDetailed", "args": {"solid": solid, "distance": 0.3, "exactOnly": true}},
            {"op": "offsetDetailed", "args": {"solid": solid, "distance": 0.3}},
        ]),
    );
    assert_eq!(batch[0]["ok"], refused);
    assert_eq!(batch[1]["ok"], engine);
    assert_eq!(counts(&kernel), before);
}

// ── Blend variants: filletV2, chamferV2, chamferDistanceAngle, filletVariable ──

/// The walking-engine blend variants over the 10³ box's first edge.
#[derive(Clone, Copy, Debug)]
enum Variant {
    FilletV2,
    ChamferV2,
    ChamferDistanceAngle,
}

impl Variant {
    const ALL: [Self; 3] = [Self::FilletV2, Self::ChamferV2, Self::ChamferDistanceAngle];
    const ANGLE: f64 = 0.5;

    const fn detailed_op(self) -> &'static str {
        match self {
            Self::FilletV2 => "filletV2Detailed",
            Self::ChamferV2 => "chamferV2Detailed",
            Self::ChamferDistanceAngle => "chamferDistanceAngleDetailed",
        }
    }

    const fn legacy_op(self) -> &'static str {
        match self {
            Self::FilletV2 => "filletV2",
            Self::ChamferV2 => "chamferV2",
            Self::ChamferDistanceAngle => "chamferDistanceAngle",
        }
    }

    fn args(self, solid: u32, edge: u32) -> Value {
        match self {
            Self::FilletV2 => json!({"solid": solid, "edges": [edge], "radius": 1.0}),
            Self::ChamferV2 => json!({"solid": solid, "edges": [edge], "d1": 1.0, "d2": 2.0}),
            Self::ChamferDistanceAngle => {
                json!({"solid": solid, "edges": [edge], "distance": 1.0, "angle": Self::ANGLE})
            }
        }
    }

    fn direct(self, kernel: &mut BrepKernel, solid: u32, edge: u32, exact_only: bool) -> Value {
        envelope(match self {
            Self::FilletV2 => kernel.fillet_v2_detailed_impl(solid, &[edge], 1.0, exact_only),
            Self::ChamferV2 => {
                kernel.chamfer_v2_detailed_impl(solid, &[edge], 1.0, 2.0, exact_only)
            }
            Self::ChamferDistanceAngle => kernel.chamfer_distance_angle_detailed_impl(
                solid,
                &[edge],
                1.0,
                Self::ANGLE,
                exact_only,
            ),
        })
    }

    const fn engine(self) -> &'static str {
        match self {
            Self::FilletV2 => "rollingBall",
            Self::ChamferV2 | Self::ChamferDistanceAngle => "planarBevel",
        }
    }

    /// Closed-form volume on the 10³ box.
    fn expected_volume(self) -> f64 {
        match self {
            Self::FilletV2 => (1.0 - std::f64::consts::FRAC_PI_4).mul_add(-10.0, 1000.0),
            Self::ChamferV2 => 1000.0 - 0.5 * 1.0 * 2.0 * 10.0,
            Self::ChamferDistanceAngle => 1000.0 - 0.5 * Self::ANGLE.tan() * 10.0,
        }
    }
}

#[test]
fn exact_blend_variants_match_both_batch_contracts_and_the_legacy_geometry() {
    for variant in Variant::ALL {
        for exact_only in [false, true] {
            let (mut kernel, solid, edge, _) = box_kernel();
            let direct = variant.direct(&mut kernel, solid, edge, exact_only);
            assert_eq!(direct["status"], "ok", "{variant:?}: {direct}");
            assert!(direct["code"].is_null());
            assert_eq!(
                direct["details"]["quality"], "exact",
                "{variant:?}: {direct}"
            );
            assert_eq!(direct["details"]["engine"], variant.engine(), "{variant:?}");
            let result = handle(&direct["value"]);
            let direct_volume = volume(&kernel, result);
            assert!(
                (direct_volume - variant.expected_volume()).abs() < 1e-6,
                "{variant:?}: volume {direct_volume} vs {}",
                variant.expected_volume()
            );

            let mut args = variant.args(solid, edge);
            if exact_only {
                args["exactOnly"] = json!(true);
            }
            let ops = json!([
                {"op": "makeBox", "args": {"width": 10, "height": 10, "depth": 10}},
                {"op": variant.detailed_op(), "args": args},
            ]);
            let v2 = batch_v2(&mut BrepKernel::new(), &ops);
            assert_eq!(
                v2[1]["ok"], direct,
                "{variant:?}: direct/executeBatchV2 parity"
            );
            let legacy = batch_legacy(&mut BrepKernel::new(), &ops);
            assert_eq!(
                legacy[1]["ok"], direct,
                "{variant:?}: direct/executeBatch parity"
            );

            let (mut legacy_kernel, ..) = box_kernel();
            let legacy = batch_v2(
                &mut legacy_kernel,
                &json!([{"op": variant.legacy_op(), "args": variant.args(solid, edge)}]),
            );
            let legacy_solid = handle(&legacy[0]["ok"]);
            assert_eq!(legacy_solid, result, "{variant:?}: same handle allocation");
            assert!((volume(&legacy_kernel, legacy_solid) - direct_volume).abs() < 1e-9);
        }
    }
}

#[test]
fn blend_variant_refusals_match_legacy_batch_v2_and_mutate_nothing() {
    for variant in Variant::ALL {
        let (mut kernel, solid, edge, _) = box_kernel();
        let mut cases = vec![
            ("invalid solid", variant.args(u32::MAX, edge)),
            ("invalid edge", variant.args(solid, u32::MAX)),
            ("oversized", {
                let mut args = variant.args(solid, edge);
                for key in ["radius", "d1", "d2", "distance"] {
                    if args.get(key).is_some() {
                        args[key] = json!(20.0);
                    }
                }
                args
            }),
        ];
        if matches!(variant, Variant::ChamferDistanceAngle) {
            let mut args = variant.args(solid, edge);
            args["angle"] = json!(std::f64::consts::FRAC_PI_2);
            cases.push(("right angle", args));
        }
        for (label, args) in cases {
            let before = counts(&kernel);
            let batch = batch_v2(
                &mut kernel,
                &json!([{"op": variant.detailed_op(), "args": args}]),
            );
            let direct = &batch[0]["ok"];
            assert_eq!(direct["status"], "error", "{variant:?}/{label}: {batch}");
            assert!(direct["value"].is_null());
            assert_eq!(direct["details"]["operation"], variant.legacy_op());
            assert_eq!(counts(&kernel), before, "{variant:?}/{label}: mutated");

            let legacy = batch_v2(
                &mut kernel,
                &json!([{"op": variant.legacy_op(), "args": args}]),
            );
            let legacy_error = &legacy[0]["error"];
            assert_eq!(
                direct["code"].as_str().unwrap(),
                v2_code(legacy_error),
                "{variant:?}/{label}: detailed={direct} legacy={legacy_error}"
            );
            assert_eq!(
                direct["category"], legacy_error["category"],
                "{variant:?}/{label}"
            );
            assert_eq!(counts(&kernel), before);
        }
    }
}

#[test]
fn walking_variant_exact_only_refuses_the_saddle_wall_with_rollback() {
    let mut kernel = BrepKernel::new();
    let (solid, saddle) = crossed_cylinders(&mut kernel);
    let solid_id = kernel.resolve_solid(solid).unwrap();
    let before = live_counts(&kernel);
    let before_faces = remus_topology::explorer::solid_faces(kernel.topo(), solid_id).unwrap();
    let refused = envelope(kernel.fillet_v2_detailed_impl(solid, &[saddle], 0.5, true));
    assert_eq!(refused["status"], "error", "{refused}");
    assert_eq!(refused["code"], "exact_only_unattainable");
    assert_eq!(refused["category"], "quality_refused");
    assert_eq!(refused["details"]["operation"], "filletV2");
    assert_eq!(refused["details"]["engine"], "walking");
    assert_eq!(live_counts(&kernel), before);
    assert_eq!(
        remus_topology::explorer::solid_faces(kernel.topo(), solid_id).unwrap(),
        before_faces
    );
}

/// A constant-law variable fillet runs the variable engine, whose wall is a
/// NURBS fit even where `filletDetailed` builds an exact cylinder.
fn variable_spec(edge: u32) -> Value {
    json!({"edge": edge, "law": "constant", "start": 1.0, "end": 1.0})
}

#[test]
fn variable_fillet_discloses_its_nurbs_wall_and_exact_only_refuses_it() {
    let (mut kernel, solid, edge, _) = box_kernel();
    let solid_id = kernel.resolve_solid(solid).unwrap();
    let input_faces: HashSet<FaceId> =
        remus_topology::explorer::solid_faces(kernel.topo(), solid_id)
            .unwrap()
            .into_iter()
            .collect();
    let before = live_counts(&kernel);
    let before_volume = volume(&kernel, solid);

    let refused =
        envelope(kernel.fillet_variable_detailed_impl(solid, &[variable_spec(edge)], true));
    assert_eq!(refused["status"], "error", "{refused}");
    assert_eq!(refused["code"], "exact_only_unattainable");
    assert_eq!(refused["category"], "quality_refused");
    assert_eq!(refused["details"]["operation"], "filletVariable");
    assert!(
        refused["details"].get("engine").is_none(),
        "no engine tag: {refused}"
    );
    assert_eq!(
        live_counts(&kernel),
        before,
        "exact-only refusal must roll back"
    );
    assert!((volume(&kernel, solid) - before_volume).abs() < 1e-12);

    let disclosed =
        envelope(kernel.fillet_variable_detailed_impl(solid, &[variable_spec(edge)], false));
    assert_eq!(disclosed["status"], "ok", "{disclosed}");
    assert_eq!(disclosed["details"]["quality"], "approximate");
    assert!(disclosed["details"].get("engine").is_none());
    let result = kernel.resolve_solid(handle(&disclosed["value"])).unwrap();
    let approximate: HashSet<u32> = disclosed["details"]["approximateFaces"]
        .as_array()
        .unwrap()
        .iter()
        .map(handle)
        .collect();
    let new_nurbs: HashSet<u32> = remus_topology::explorer::solid_faces(kernel.topo(), result)
        .unwrap()
        .into_iter()
        .filter(|face| {
            !input_faces.contains(face)
                && matches!(
                    kernel.topo().face(*face).unwrap().surface(),
                    FaceSurface::Nurbs(_)
                )
        })
        .map(face_id_to_u32)
        .collect();
    assert!(!approximate.is_empty());
    assert_eq!(approximate, new_nurbs);
    // The fit stays close to the exact rounded box it approximates.
    let exact = (1.0 - std::f64::consts::FRAC_PI_4).mul_add(-10.0, 1000.0);
    assert!((volume(&kernel, handle(&disclosed["value"])) - exact).abs() < 0.05);

    // Same body behind the batch op, and the same solid as legacy
    // `filletVariable` when replayed on identical kernels.
    let (mut batch_kernel, ..) = box_kernel();
    let batch = batch_v2(
        &mut batch_kernel,
        &json!([
            {"op": "filletVariableDetailed", "args": {"solid": solid, "specs": [variable_spec(edge)], "exactOnly": true}},
            {"op": "filletVariableDetailed", "args": {"solid": solid, "specs": [variable_spec(edge)]}},
        ]),
    );
    assert_eq!(batch[0]["ok"], refused);
    assert_eq!(batch[1]["ok"], disclosed);

    let (mut fresh, ..) = box_kernel();
    let fresh_result =
        envelope(fresh.fillet_variable_detailed_impl(solid, &[variable_spec(edge)], false));
    let (mut legacy_kernel, ..) = box_kernel();
    let legacy = batch_v2(
        &mut legacy_kernel,
        &json!([{"op": "filletVariable", "args": {"solid": solid, "specs": [variable_spec(edge)]}}]),
    );
    let legacy_solid = handle(&legacy[0]["ok"]);
    assert_eq!(legacy_solid, handle(&fresh_result["value"]));
    assert!((volume(&legacy_kernel, legacy_solid) - volume(&fresh, legacy_solid)).abs() < 1e-9);
}

#[test]
fn variable_fillet_spec_refusals_match_legacy_batch_v2() {
    let (mut kernel, solid, edge, _) = box_kernel();
    for (label, specs) in [
        (
            "missing edge",
            json!([{"law": "constant", "start": 1.0, "end": 1.0}]),
        ),
        ("invalid edge", json!([variable_spec(u32::MAX)])),
        (
            "malformed setback",
            json!([{"edge": edge, "law": "constant", "start": 1.0, "end": 1.0, "startSetback": "far"}]),
        ),
        // B63 closed: the variable engine now refuses a radius wider than
        // its support faces with the walking engine's cliff diagnostic.
        (
            "oversized radius",
            json!([{"edge": edge, "law": "constant", "start": 20.0, "end": 20.0}]),
        ),
    ] {
        let before = counts(&kernel);
        let direct =
            envelope(kernel.fillet_variable_detailed_impl(solid, specs.as_array().unwrap(), false));
        assert_eq!(direct["status"], "error", "{label}: {direct}");
        if label == "oversized radius" {
            assert_eq!(direct["code"], "cliff-encountered", "{direct}");
        }
        assert_eq!(counts(&kernel), before, "{label}: mutated");
        let legacy = batch_v2(
            &mut kernel,
            &json!([{"op": "filletVariable", "args": {"solid": solid, "specs": specs}}]),
        );
        let legacy_error = &legacy[0]["error"];
        assert_eq!(
            direct["code"].as_str().unwrap(),
            v2_code(legacy_error),
            "{label}"
        );
        assert_eq!(direct["category"], legacy_error["category"], "{label}");
        let batch = batch_v2(
            &mut kernel,
            &json!([{"op": "filletVariableDetailed", "args": {"solid": solid, "specs": specs}}]),
        );
        assert_eq!(batch[0]["ok"], direct, "{label}: batch twin parity");
    }
    // A missing spec array is a batch argument error, like the legacy op.
    let response = batch_v2(
        &mut kernel,
        &json!([{"op": "filletVariableDetailed", "args": {"solid": solid}}]),
    );
    assert_eq!(response[0]["error"]["code"], "invalid_argument");
}
