//! The CDT mesher's NURBS chart frame shares its carrier's knots and weights
//! instead of copying them (`NurbsSurface::with_control_points`), so the
//! per-sample position-and-normal solve skips a knot-vector comparison.
//! These tests build the frame from copies, as it was, and compare whole
//! display meshes bit for bit: an in-place helical sweep, a thread, a
//! helical gear, a cross-drilled shaft and ordinary curved bodies.

#![allow(clippy::unwrap_used)]

use std::cell::Cell;

use remus_topology::Topology;

use crate::tessellate::swept_fixtures as fixtures;
use crate::tessellate::tessellate_solid;

thread_local! {
    static COPY_STORAGE: Cell<bool> = const { Cell::new(false) };
}

/// Whether `NurbsFrame::new` must build its carrier from copied knots and
/// weights, as it did before it shared them.
pub(super) fn copy_storage() -> bool {
    COPY_STORAGE.with(Cell::get)
}

fn with_copied_storage<T>(run: impl FnOnce() -> T) -> T {
    COPY_STORAGE.with(|flag| flag.set(true));
    let out = run();
    COPY_STORAGE.with(|flag| flag.set(false));
    out
}

#[test]
fn display_meshes_are_bit_identical_with_shared_and_copied_frame_storage() {
    let mut topo = Topology::new();
    let mut bodies = vec![
        ("coil", fixtures::coil(&mut topo, 3.0), 0.05),
        (
            "external thread",
            fixtures::external_thread(&mut topo, 2.0),
            0.02,
        ),
        ("helical gear", fixtures::helical_gear(&mut topo), 0.05),
        (
            "cross-drilled shaft",
            fixtures::cross_drilled_shaft(&mut topo),
            0.05,
        ),
    ];
    for solid in fixtures::ordinary(&mut topo) {
        bodies.push(("ordinary", solid, 0.05));
    }
    let mut triangles = 0;
    for (name, solid, deflection) in bodies {
        let mesh = tessellate_solid(&topo, solid, deflection).unwrap();
        let copied = with_copied_storage(|| tessellate_solid(&topo, solid, deflection).unwrap());
        fixtures::assert_same_bits(&mesh, &copied, name);
        triangles += mesh.indices.len() / 3;
    }
    assert!(triangles > 20_000, "{triangles} triangles");
}
