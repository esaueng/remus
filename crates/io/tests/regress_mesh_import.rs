//! Mesh importer regressions for winding, accessor layouts, JSON and normals.
#![allow(clippy::unwrap_used)]
use remus_operations::measure;
use remus_topology::{Topology, face::FaceSurface};
fn glb(json: &serde_json::Value, bin: &[u8]) -> Vec<u8> {
    let mut j = serde_json::to_vec(json).unwrap();
    while !j.len().is_multiple_of(4) {
        j.push(b' ');
    }
    let mut b = bin.to_vec();
    while !b.len().is_multiple_of(4) {
        b.push(0);
    }
    let mut out = Vec::new();
    for v in [
        0x4654_6c67_u32,
        2,
        (12 + 8 + j.len() + 8 + b.len()) as u32,
        j.len() as u32,
        0x4e4f_534a,
    ] {
        out.extend(v.to_le_bytes());
    }
    out.extend(j);
    out.extend((b.len() as u32).to_le_bytes());
    out.extend(0x004e_4942_u32.to_le_bytes());
    out.extend(b);
    out
}

fn simple_glb_json() -> serde_json::Value {
    serde_json::json!({"asset":{"version":"2.0"},"meshes":[{"primitives":[{"attributes":{"POSITION":0}}]}],"accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"}],"bufferViews":[{"buffer":0,"byteLength":36}],"buffers":[{"byteLength":36}]})
}

fn triangle_bytes() -> Vec<u8> {
    [0f32, 0., 0., 1., 0., 0., 0., 1., 0.]
        .into_iter()
        .flat_map(f32::to_le_bytes)
        .collect()
}

#[test]
fn glb_valid_name_string_must_not_break_json_parser() {
    let mut j = simple_glb_json();
    let bytes = triangle_bytes();
    assert!(remus_io::gltf::read_glb(&glb(&j, &bytes)).is_ok());
    j["accessors"][0]["name"] = serde_json::json!("points ]");
    let r = remus_io::gltf::read_glb(&glb(&j, &bytes));
    assert!(r.is_ok(), "valid optional name string rejected: {r:?}");
}

#[test]
fn glb_valid_accessor_subrange_must_import() {
    let mut j = simple_glb_json();
    let mut bytes = vec![0u8; 12];
    bytes.extend(triangle_bytes());
    j["accessors"][0]["byteOffset"] = serde_json::json!(12);
    j["bufferViews"][0]["byteLength"] = serde_json::json!(48);
    j["buffers"][0]["byteLength"] = serde_json::json!(48);
    let r = remus_io::gltf::read_glb(&glb(&j, &bytes));
    assert!(r.is_ok(), "valid accessor offset/subrange rejected: {r:?}");
}

#[test]
fn obj_face_normal_references_are_preserved() {
    let text = "v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 -1\nf 1//1 2//1 3//1\n";
    let m = remus_io::obj::read_obj(text).unwrap();
    assert!(
        m.normals.iter().all(|n| (n.z() + 1.).abs() < 1e-12),
        "expected -Z at every face corner, got {:?}",
        m.normals
    );
}

#[test]
fn glb_omitted_normals_must_not_reverse_closed_mesh_faces() {
    let p = [
        [10f32, 20., 30.],
        [11., 20., 30.],
        [10., 21., 30.],
        [10., 20., 31.],
    ];
    let corners = [0usize, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];
    let bytes: Vec<u8> = corners
        .into_iter()
        .flat_map(|i| p[i])
        .flat_map(f32::to_le_bytes)
        .collect();
    let mut j = simple_glb_json();
    j["accessors"][0]["count"] = serde_json::json!(12);
    j["bufferViews"][0]["byteLength"] = serde_json::json!(144);
    j["buffers"][0]["byteLength"] = serde_json::json!(144);
    let mut t = Topology::new();
    let s = remus_io::gltf::read_glb_solid(&mut t, &glb(&j, &bytes), 1e-7).unwrap();
    let volume = measure::solid_volume(&t, s, 0.1).unwrap();
    assert!((volume - 1. / 6.).abs() < 1e-10, "volume {volume}");
    let f = remus_topology::explorer::solid_faces(&t, s).unwrap()[0];
    let normal = match t.face(f).unwrap().surface() {
        FaceSurface::Plane { normal, .. } => *normal,
        _ => unreachable!(),
    };
    assert!(
        normal.z() < 0.,
        "closed tetrahedron bottom face was reversed to {:?}; volume {}",
        normal,
        volume
    );
}

#[test]
fn glb_interleaved_attributes_share_a_view_and_decode_only_their_ranges() {
    let mut j = simple_glb_json();
    let bytes: Vec<u8> = [
        0f32, 0., 0., 0., 0., -1., 1., 0., 0., 0., 0., -1., 0., 1., 0., 0., 0., -1.,
    ]
    .into_iter()
    .flat_map(f32::to_le_bytes)
    .collect();
    j["bufferViews"][0]["byteLength"] = serde_json::json!(72);
    j["bufferViews"][0]["byteStride"] = serde_json::json!(24);
    j["buffers"][0]["byteLength"] = serde_json::json!(72);
    j["accessors"].as_array_mut().unwrap().push(serde_json::json!({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","byteOffset":12}));
    j["meshes"][0]["primitives"][0]["attributes"]["NORMAL"] = serde_json::json!(1);
    let mesh = remus_io::gltf::read_glb(&glb(&j, &bytes)).unwrap();
    assert_eq!(mesh.positions.len(), 3);
    assert!((mesh.positions[1] - remus_math::vec::Point3::new(1., 0., 0.)).length() < 1e-12);
    assert!(mesh.normals.iter().all(|n| (n.z() + 1.).abs() < 1e-12));
    j["accessors"][1]["byteOffset"] = serde_json::json!(16);
    assert!(remus_io::gltf::read_glb(&glb(&j, &bytes)).is_err());
    j["accessors"][1]["byteOffset"] = serde_json::json!(12);
    for stride in [0, 8, 14, 256] {
        j["bufferViews"][0]["byteStride"] = serde_json::json!(stride);
        assert!(remus_io::gltf::read_glb(&glb(&j, &bytes)).is_err());
    }
}

#[test]
fn glb_index_accessor_offset_and_unused_view_bytes_are_respected() {
    let mut j = simple_glb_json();
    let mut bytes = triangle_bytes();
    bytes.extend([0u8; 4]);
    for index in [0u16, 1, 2] {
        bytes.extend(index.to_le_bytes());
    }
    bytes.extend([0u8; 2]);
    j["accessors"].as_array_mut().unwrap().push(serde_json::json!({"bufferView":1,"componentType":5123,"count":3,"type":"SCALAR","byteOffset":4}));
    j["bufferViews"]
        .as_array_mut()
        .unwrap()
        .push(serde_json::json!({"buffer":0,"byteOffset":36,"byteLength":12}));
    j["buffers"][0]["byteLength"] = serde_json::json!(48);
    j["meshes"][0]["primitives"][0]["indices"] = serde_json::json!(1);
    let mesh = remus_io::gltf::read_glb(&glb(&j, &bytes)).unwrap();
    assert_eq!(mesh.indices, [0, 1, 2]);
    j["accessors"][1]["byteOffset"] = serde_json::json!(8);
    assert!(remus_io::gltf::read_glb(&glb(&j, &bytes)).is_err());
}

#[test]
fn malformed_glb_accessor_slots_cannot_shift_later_references() {
    let mut j = simple_glb_json();
    j["accessors"]
        .as_array_mut()
        .unwrap()
        .insert(0, serde_json::json!({"name":"unused but malformed"}));
    assert!(remus_io::gltf::read_glb(&glb(&j, &triangle_bytes())).is_err());
}

#[test]
fn obj_separate_normal_indices_split_shared_positions_and_validate_references() {
    let text =
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 1\nvn 0 0 -1\nf 1//1 2//1 3//1\nf 1//2 3//2 2//2\n";
    let mesh = remus_io::obj::read_obj(text).unwrap();
    assert_eq!(mesh.positions.len(), 6);
    for (triangle, sign) in mesh.indices.chunks_exact(3).zip([1., -1.]) {
        assert!(
            triangle
                .iter()
                .all(|&index| (mesh.normals[index as usize].z() - sign).abs() < 1e-12)
        );
    }
    let limits = remus_io::limits::ImportLimits {
        max_model_entities: 5,
        ..Default::default()
    };
    assert!(matches!(
        remus_io::obj::read_obj_with_limits(text, limits),
        Err(remus_io::IoError::LimitExceeded { .. })
    ));
    for bad in ["1//9", "1//0", "9//1"] {
        let invalid = text.replace("1//1", bad);
        assert!(remus_io::obj::read_obj(&invalid).is_err());
    }
}

#[test]
fn invalid_explicit_glb_indices_cannot_become_an_unindexed_primitive() {
    for index in [
        serde_json::Value::Null,
        serde_json::json!(-1),
        serde_json::json!(0.5),
        serde_json::json!("1"),
    ] {
        let mut j = simple_glb_json();
        j["meshes"][0]["primitives"][0]["indices"] = index;
        assert!(remus_io::gltf::read_glb(&glb(&j, &triangle_bytes())).is_err());
    }
}
