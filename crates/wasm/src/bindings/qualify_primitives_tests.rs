//! Direct/batch WASM qualification for the B6 primitive matrix.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::collections::BTreeMap;
use std::f64::consts::PI;

use remus_math::tolerance::Tolerance;
use remus_operations::tessellate::{tessellate_solid_with_tolerance, welded_mesh_quality};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;

use crate::kernel::BrepKernel;

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];

#[derive(Clone, Copy, Debug)]
enum Primitive {
    Box,
    Cylinder,
    PointedCone,
    Frustum,
    Sphere,
    Torus,
    Ellipsoid,
}

impl Primitive {
    const QUALIFIED: [Self; 7] = [
        Self::Box,
        Self::Cylinder,
        Self::PointedCone,
        Self::Frustum,
        Self::Sphere,
        Self::Torus,
        Self::Ellipsoid,
    ];

    fn make_direct(
        self,
        kernel: &mut BrepKernel,
        scale: f64,
    ) -> Result<u32, wasm_bindgen::JsError> {
        match self {
            Self::Box => kernel.make_box_solid(2.0 * scale, 3.0 * scale, 4.0 * scale),
            Self::Cylinder => kernel.make_cylinder_solid(2.0 * scale, 5.0 * scale),
            Self::PointedCone => kernel.make_cone_solid(3.0 * scale, 0.0, 4.0 * scale),
            Self::Frustum => kernel.make_cone_solid(3.0 * scale, scale, 4.0 * scale),
            Self::Sphere => kernel.make_sphere_solid(2.0 * scale, 16),
            Self::Torus => kernel.make_torus_solid(4.0 * scale, scale, 16),
            Self::Ellipsoid => kernel.make_ellipsoid_solid(1.5 * scale, 2.0 * scale, 2.5 * scale),
        }
    }

    fn batch_make(self, scale: f64) -> serde_json::Value {
        match self {
            Self::Box => serde_json::json!({
                "op": "makeBox",
                "args": {"width": 2.0 * scale, "height": 3.0 * scale, "depth": 4.0 * scale}
            }),
            Self::Cylinder => serde_json::json!({
                "op": "makeCylinder",
                "args": {"radius": 2.0 * scale, "height": 5.0 * scale}
            }),
            Self::PointedCone => serde_json::json!({
                "op": "makeCone",
                "args": {"bottomRadius": 3.0 * scale, "topRadius": 0.0, "height": 4.0 * scale}
            }),
            Self::Frustum => serde_json::json!({
                "op": "makeCone",
                "args": {"bottomRadius": 3.0 * scale, "topRadius": scale, "height": 4.0 * scale}
            }),
            Self::Sphere => serde_json::json!({
                "op": "makeSphere",
                "args": {"radius": 2.0 * scale, "segments": 16}
            }),
            Self::Torus => serde_json::json!({
                "op": "makeTorus",
                "args": {"majorRadius": 4.0 * scale, "minorRadius": scale, "segments": 16}
            }),
            Self::Ellipsoid => serde_json::json!({
                "op": "makeEllipsoid",
                "args": {"rx": 1.5 * scale, "ry": 2.0 * scale, "rz": 2.5 * scale}
            }),
        }
    }

    fn expected_volume(self, scale: f64) -> f64 {
        let unit = match self {
            Self::Box => 24.0,
            Self::Cylinder => 20.0 * PI,
            Self::PointedCone => 12.0 * PI,
            Self::Frustum => 52.0 * PI / 3.0,
            Self::Sphere => 32.0 * PI / 3.0,
            Self::Torus => 8.0 * PI * PI,
            Self::Ellipsoid => 10.0 * PI,
        };
        unit * scale.powi(3)
    }

    fn expected_bounds(self, scale: f64) -> [f64; 6] {
        let unit = match self {
            Self::Box => [0.0, 0.0, 0.0, 2.0, 3.0, 4.0],
            Self::Cylinder => [-2.0, -2.0, 0.0, 2.0, 2.0, 5.0],
            Self::PointedCone | Self::Frustum => [-3.0, -3.0, 0.0, 3.0, 3.0, 4.0],
            Self::Sphere => [-2.0, -2.0, -2.0, 2.0, 2.0, 2.0],
            Self::Torus => [-5.0, -5.0, -1.0, 5.0, 5.0, 1.0],
            Self::Ellipsoid => [-1.5, -2.0, -2.5, 1.5, 2.0, 2.5],
        };
        unit.map(|value| value * scale)
    }

    fn expected_surfaces(self) -> BTreeMap<&'static str, usize> {
        let entries: &[(&str, usize)] = match self {
            Self::Box => &[("plane", 6)],
            Self::Cylinder => &[("cylinder", 1), ("plane", 2)],
            Self::PointedCone => &[("cone", 1), ("plane", 1)],
            Self::Frustum => &[("cone", 1), ("plane", 2)],
            Self::Sphere => &[("sphere", 2)],
            Self::Torus => &[("torus", 1)],
            Self::Ellipsoid => &[("nurbs", 2)],
        };
        entries.iter().copied().collect()
    }

    const fn expected_euler(self) -> i64 {
        match self {
            Self::Torus => 0,
            _ => 2,
        }
    }

    const fn volume_tolerance(self) -> f64 {
        match self {
            Self::Ellipsoid => 1e-3,
            _ => 1e-9,
        }
    }
}

fn assert_relative(label: &str, actual: f64, expected: f64, limit: f64) {
    let relative = (actual - expected).abs() / expected.abs();
    assert!(
        relative <= limit,
        "{label}: expected {expected:.12e}, got {actual:.12e}, relative error {relative:.3e} > {limit:.3e}"
    );
}

fn signed_mesh_volume(mesh: &remus_operations::tessellate::TriangleMesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|triangle| {
            let a = mesh.positions[triangle[0] as usize];
            let b = mesh.positions[triangle[1] as usize];
            let c = mesh.positions[triangle[2] as usize];
            let determinant = a.x() * (b.y() * c.z() - b.z() * c.y())
                - a.y() * (b.x() * c.z() - b.z() * c.x())
                + a.z() * (b.x() * c.y() - b.y() * c.x());
            determinant / 6.0
        })
        .sum::<f64>()
        .abs()
}

fn parse(response: &str) -> serde_json::Value {
    serde_json::from_str(response).expect("batch response must be valid JSON")
}

fn batch_program(case: Primitive, scale: f64) -> String {
    serde_json::Value::Array(vec![
        case.batch_make(scale),
        serde_json::json!({
            "op": "volume",
            "args": {"solid": 0, "deflection": 0.01 * scale}
        }),
        serde_json::json!({"op": "boundingBox", "args": {"solid": 0}}),
        serde_json::json!({"op": "validateSolid", "args": {"solid": 0}}),
        serde_json::json!({
            "op": "meshQuality",
            "args": {"solid": 0, "deflection": 0.01 * scale, "angularTolerance": 0.1}
        }),
    ])
    .to_string()
}

#[test]
fn primitive_matrix_has_direct_batch_parity_across_scales() {
    for case in Primitive::QUALIFIED {
        for scale in SCALES {
            let label = format!("{case:?} at scale {scale}");
            let expected_volume = case.expected_volume(scale);
            let expected_bounds = case.expected_bounds(scale);

            let mut direct = BrepKernel::new();
            let handle = case.make_direct(&mut direct, scale).unwrap();
            let direct_volume = direct.volume(handle, 0.01 * scale).unwrap();
            assert_relative(
                &format!("{label}: direct volume"),
                direct_volume,
                expected_volume,
                case.volume_tolerance(),
            );
            let direct_bounds = direct.bounding_box(handle).unwrap();
            for (axis, (&actual, expected)) in direct_bounds.iter().zip(expected_bounds).enumerate()
            {
                let limit = 1e-10 * scale.max(1.0);
                assert!(
                    (actual - expected).abs() <= limit,
                    "{label}: direct bound {axis} expected {expected:.12e}, got {actual:.12e}"
                );
            }
            assert_eq!(direct.validate_solid(handle).unwrap(), 0, "{label}");

            let solid = direct.resolve_solid(handle).unwrap();
            let mut surfaces = BTreeMap::new();
            for face_id in solid_faces(direct.topo(), solid).unwrap() {
                let tag = direct.topo().face(face_id).unwrap().surface().type_tag();
                *surfaces.entry(tag).or_default() += 1;
            }
            assert_eq!(surfaces, case.expected_surfaces(), "{label}");

            let mesh =
                tessellate_solid_with_tolerance(direct.topo(), solid, 0.01 * scale, 0.1).unwrap();
            let direct_quality = welded_mesh_quality(&mesh);
            assert!(
                direct_quality.is_watertight(),
                "{label}: {direct_quality:?}"
            );
            assert_eq!(direct_quality.euler_characteristic, case.expected_euler());
            assert_relative(
                &format!("{label}: direct independent mesh volume"),
                signed_mesh_volume(&mesh),
                expected_volume,
                0.02,
            );

            let program = batch_program(case, scale);
            let first_response = BrepKernel::new().execute_batch_v2(&program);
            let second_response = BrepKernel::new().execute_batch_v2(&program);
            assert_eq!(
                first_response, second_response,
                "{label}: batch replay must be deterministic"
            );
            let batch = parse(&first_response);
            assert_eq!(batch[0]["ok"], 0, "{label}: primitive handle");
            let batch_volume = batch[1]["ok"].as_f64().expect("batch volume");
            assert_eq!(
                batch_volume.to_bits(),
                direct_volume.to_bits(),
                "{label}: direct and batch volume parity"
            );
            let batch_bounds = batch[2]["ok"].as_array().expect("batch bounds");
            for (axis, (&direct_value, batch_value)) in
                direct_bounds.iter().zip(batch_bounds).enumerate()
            {
                assert_eq!(
                    batch_value.as_f64().expect("batch bound").to_bits(),
                    direct_value.to_bits(),
                    "{label}: direct/batch bound {axis}"
                );
            }
            assert_eq!(batch[3]["ok"], 0, "{label}: batch validation");
            assert_eq!(batch[4]["ok"]["isWatertight"], true, "{label}");
            assert_eq!(batch[4]["ok"]["boundaryEdges"], 0, "{label}");
            assert_eq!(batch[4]["ok"]["nonManifoldEdges"], 0, "{label}");
            assert_eq!(
                batch[4]["ok"]["eulerCharacteristic"],
                case.expected_euler(),
                "{label}: batch mesh Euler"
            );
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ArenaCounts {
    vertices: usize,
    edges: usize,
    wires: usize,
    faces: usize,
    shells: usize,
    solids: usize,
    loops: usize,
    coedges: usize,
}

fn arena_counts(topo: &Topology) -> ArenaCounts {
    ArenaCounts {
        vertices: topo.num_vertices(),
        edges: topo.num_edges(),
        wires: topo.num_wires(),
        faces: topo.num_faces(),
        shells: topo.num_shells(),
        solids: topo.num_solids(),
        loops: topo.num_loops(),
        coedges: topo.num_coedges(),
    }
}

#[test]
fn batch_invalid_input_matrix_has_stable_codes_and_rolls_back() {
    let operations = serde_json::json!([
        {"op":"makeBox","args":{"width":1.0,"height":2.0,"depth":3.0}},
        {"op":"makeBox","args":{"width":0.0,"height":1.0,"depth":1.0}},
        {"op":"makeCylinder","args":{"radius":-1.0,"height":1.0}},
        {"op":"makeSphere","args":{"radius":0.0,"segments":16}},
        {"op":"makeSphere","args":{"radius":1.0,"segments":3}},
        {"op":"makeSphere","args":{"radius":1.0,"segments":10001}},
        {"op":"makeCone","args":{"bottomRadius":0.0,"topRadius":0.0,"height":1.0}},
        {"op":"makeCone","args":{"bottomRadius":-1.0,"topRadius":1.0,"height":1.0}},
        {"op":"makeTorus","args":{"majorRadius":2.0,"minorRadius":2.0,"segments":16}},
        {"op":"makeTorus","args":{"majorRadius":3.0,"minorRadius":1.0,"segments":3}},
        {"op":"makeEllipsoid","args":{"rx":1.0,"ry":0.0,"rz":1.0}},
        {"op":"makeBox","args":{"height":1.0,"depth":1.0}},
        {"op":"volume","args":{"solid":0,"deflection":0.01}}
    ]);

    let mut kernel = BrepKernel::new();
    let response = parse(&kernel.execute_batch_v2(&operations.to_string()));
    assert_eq!(response[0]["ok"], 0);
    for (index, item) in response
        .as_array()
        .expect("batch result array")
        .iter()
        .enumerate()
        .take(12)
        .skip(1)
    {
        assert_eq!(item["error"]["code"], "invalid_argument", "item {index}");
        assert_eq!(item["error"]["category"], "invalid_input", "item {index}");
        assert_eq!(item["error"]["details"]["operationIndex"], index);
    }
    assert_relative(
        "existing batch box after refusals",
        response[12]["ok"].as_f64().expect("box volume"),
        6.0,
        1e-12,
    );

    let mut baseline = BrepKernel::new();
    baseline.make_box_solid(1.0, 2.0, 3.0).unwrap();
    assert_eq!(
        arena_counts(kernel.topo()),
        arena_counts(baseline.topo()),
        "failed batch primitives must leave no live topology"
    );
}

#[test]
fn batch_boundary_values_are_tested_from_both_sides() {
    let minimum = Tolerance::new().linear;
    let cases = [
        (
            "box below",
            serde_json::json!({"op":"makeBox","args":{"width":minimum,"height":1.0,"depth":1.0}}),
            false,
        ),
        (
            "box above",
            serde_json::json!({"op":"makeBox","args":{"width":2.0 * minimum,"height":1.0,"depth":1.0}}),
            true,
        ),
        (
            "sphere segments below",
            serde_json::json!({"op":"makeSphere","args":{"radius":1.0,"segments":3}}),
            false,
        ),
        (
            "sphere segments at boundary",
            serde_json::json!({"op":"makeSphere","args":{"radius":1.0,"segments":4}}),
            true,
        ),
        (
            "torus radii equal",
            serde_json::json!({"op":"makeTorus","args":{"majorRadius":1.0,"minorRadius":1.0,"segments":16}}),
            false,
        ),
        (
            "torus radii separated",
            serde_json::json!({"op":"makeTorus","args":{"majorRadius":1.0,"minorRadius":1.0 - 2.0 * minimum,"segments":16}}),
            true,
        ),
    ];

    for (label, operation, accepted) in cases {
        let program = serde_json::Value::Array(vec![operation]).to_string();
        let result = parse(&BrepKernel::new().execute_batch_v2(&program));
        assert_eq!(result[0].get("ok").is_some(), accepted, "{label}: {result}");
        if !accepted {
            assert_eq!(result[0]["error"]["code"], "invalid_argument", "{label}");
            assert_eq!(result[0]["error"]["category"], "invalid_input", "{label}");
        }
    }
}
