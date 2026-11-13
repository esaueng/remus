use remus_io::ImportLimits;
use remus_operations::tessellate::TriangleMesh;
use remus_topology::Topology;

pub fn limits() -> ImportLimits {
    ImportLimits {
        max_input_bytes: 1024 * 1024,
        max_archive_entry_bytes: 2 * 1024 * 1024,
        max_model_entities: 10_000,
    }
}

/// The weld tolerance the wasm mesh imports (`remus-wasm-io`) pass to
/// `import_mesh`.
#[allow(dead_code)]
pub const MESH_WELD_TOLERANCE: f64 = 1e-7;

/// Upper bound on the arena entities `import_mesh` allocates for `mesh`: one
/// vertex per position and nine entities per triangle, plus shell and solid.
/// Mirrors the guard the wasm mesh imports apply before importing.
#[allow(dead_code)]
pub fn mesh_entity_bound(mesh: &TriangleMesh) -> Option<usize> {
    (mesh.indices.len() / 3)
        .checked_mul(9)
        .and_then(|count| count.checked_add(mesh.positions.len()))
        .and_then(|count| count.checked_add(2))
}

/// Feed parsed meshes through the B-Rep import every mesh format ends in, the
/// way the wasm `import*` bindings do: refuse the set when its generated
/// entity bound exceeds the model-entity limit, otherwise import each mesh
/// into a fresh topology.
///
/// Oracle: an accepted mesh yields one resolvable solid with at least one and
/// at most one face per input triangle (degenerate triangles are dropped,
/// never split). A refusal is a pass.
#[allow(dead_code)]
pub fn import_meshes(meshes: &[TriangleMesh]) {
    let bound = meshes.iter().try_fold(0usize, |total, mesh| {
        mesh_entity_bound(mesh).and_then(|count| total.checked_add(count))
    });
    if bound.is_none_or(|bound| bound > limits().max_model_entities) {
        return;
    }
    for mesh in meshes {
        let mut topo = Topology::new();
        let Ok(solid) = remus_io::stl::import_mesh(&mut topo, mesh, MESH_WELD_TOLERANCE) else {
            continue;
        };
        assert!(topo.solid(solid).is_ok(), "imported solid does not resolve");
        let faces = remus_topology::explorer::solid_faces(&topo, solid)
            .expect("imported solid has resolvable faces");
        assert!(!faces.is_empty(), "imported solid has no faces");
        assert!(
            faces.len() <= mesh.indices.len() / 3,
            "more faces than input triangles"
        );
    }
}

/// A refused atomic import must leave a fresh topology with no live entity.
#[allow(dead_code)]
pub fn assert_untouched(topo: &Topology, path: &str) {
    assert_eq!(
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
            topo.num_loops(),
            topo.num_coedges(),
            topo.num_pcurves(),
        ),
        (0, 0, 0, 0, 0, 0, 0, 0, 0),
        "a refused {path} import left live entities behind"
    );
}
