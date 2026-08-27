//! `mesh_boolean` must refuse a malformed triangle mesh, not panic on it.
//!
//! Both operands are caller-supplied: the WASM `meshBoolean` binding hands
//! JS arrays straight through `build_triangle_mesh`, which copies the index
//! array verbatim. Every downstream stage then indexes `positions`,
//! `normals` and `indices` raw, so anything malformed used to abort the
//! kernel — and a panic that unwinds across the wasm-bindgen boundary leaves
//! the kernel's `RefCell` borrowed, breaking every later JS call.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::boolean::BooleanOp;
use remus_operations::mesh_boolean::mesh_boolean;
use remus_operations::tessellate::TriangleMesh;

/// A closed tetrahedron — the smallest well-formed operand.
fn tetrahedron() -> TriangleMesh {
    TriangleMesh {
        positions: vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ],
        normals: vec![Vec3::new(0.0, 0.0, 1.0); 4],
        indices: vec![0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
    }
}

/// Run a fuse with operand A mutated, asserting it returns rather than
/// panicking, and that the error names the offending mesh.
fn assert_refused(case: &str, mutate: impl FnOnce(&mut TriangleMesh)) {
    let mut a = tetrahedron();
    mutate(&mut a);
    let b = tetrahedron();

    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        mesh_boolean(&a, &b, BooleanOp::Fuse, 1e-6)
    }));

    let result = outcome.unwrap_or_else(|_| panic!("{case}: mesh_boolean panicked"));
    match result {
        Err(OperationsError::InvalidInput { reason }) => {
            assert!(
                reason.contains("mesh A"),
                "{case}: error should name the offending operand, got {reason:?}"
            );
        }
        Err(other) => panic!("{case}: expected InvalidInput, got {other:?}"),
        Ok(_) => panic!("{case}: malformed input was accepted"),
    }
}

#[test]
fn vertex_index_past_the_end_is_refused() {
    // The original crash: `index out of bounds: the len is 4 but the index
    // is 99` in `get_triangle`.
    assert_refused("index out of range", |m| m.indices[2] = 99);
}

#[test]
fn index_count_that_is_not_whole_triangles_is_refused() {
    // Previously accepted: every triangle count is `indices.len() / 3`, so a
    // trailing partial triangle was dropped without a word.
    assert_refused("indices not a multiple of 3", |m| {
        m.indices.pop();
    });
}

#[test]
fn missing_normals_are_refused() {
    // Assembly reads `normals[i]` for a vertex index `i`.
    assert_refused("empty normals", |m| m.normals.clear());
}

#[test]
fn short_normals_are_refused() {
    assert_refused("short normals", |m| {
        m.normals.pop();
    });
}

#[test]
fn indices_into_an_empty_vertex_array_are_refused() {
    assert_refused("empty positions", |m| {
        m.positions.clear();
        m.normals.clear();
    });
}

#[test]
fn a_well_formed_pair_still_fuses() {
    // The control: the guard must not reject valid input.
    let a = tetrahedron();
    let b = tetrahedron();
    let result =
        mesh_boolean(&a, &b, BooleanOp::Fuse, 1e-6).expect("well-formed fuse must succeed");
    assert!(!result.mesh.positions.is_empty());
    assert_eq!(result.mesh.indices.len() % 3, 0);
    assert_eq!(
        result.mesh.normals.len(),
        result.mesh.positions.len(),
        "the result must itself satisfy the contract it enforces on inputs"
    );
}
