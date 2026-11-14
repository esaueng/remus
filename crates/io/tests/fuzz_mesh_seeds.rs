//! The `glb_reader` and `threemf_reader` fuzz seeds reach mesh import.
//!
//! Both targets mutate a whole binary container, so a campaign that starts
//! from a seed the reader refuses spends its time on the container layer and
//! may never hand `import_mesh` a mesh. Each seed is a 1 x 2 x 3 box; this
//! test pins that both still parse and import as one closed 12-triangle box.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::path::Path;

use remus_operations::measure;
use remus_operations::tessellate::TriangleMesh;
use remus_topology::Topology;

/// The weld tolerance the fuzz harness and the wasm mesh imports pass.
const WELD_TOLERANCE: f64 = 1e-7;

fn seed(relative: &str) -> Vec<u8> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../fuzz/corpus")
        .join(relative);
    std::fs::read(&path).expect("fuzz seed is committed")
}

fn assert_imports_as_the_box(mesh: &TriangleMesh) {
    assert_eq!(mesh.indices.len(), 36, "a box seed carries 12 triangles");
    let mut topo = Topology::new();
    let solid = remus_io::stl::import_mesh(&mut topo, mesh, WELD_TOLERANCE).unwrap();
    let volume = measure::solid_volume(&topo, solid, 0.01).unwrap();
    assert!(
        (volume - 6.0).abs() < 1e-9,
        "box volume {volume}, expected 6"
    );
}

#[test]
fn glb_seed_reaches_mesh_import() {
    let mesh = remus_io::gltf::read_glb(&seed("glb_reader/box.glb")).unwrap();
    assert_imports_as_the_box(&mesh);
}

#[test]
fn threemf_seed_reaches_mesh_import() {
    let meshes = remus_io::threemf::read_threemf(&seed("threemf_reader/box.3mf")).unwrap();
    assert_eq!(meshes.len(), 1, "one object in the seed");
    assert_imports_as_the_box(&meshes[0]);
}
