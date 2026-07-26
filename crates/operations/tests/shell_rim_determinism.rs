//! Regression gate for shell-rim determinism.
//!
//! `shell` collected its open-boundary edges by iterating the std `HashMap`
//! returned by `edge_to_face_map`, so their order was seed-dependent. That
//! order decides where `sort_edges_into_loops` starts each chain, and a
//! different starting edge splits the rim into a different NUMBER of loops —
//! the cup's rim came back with two or three inner wires depending on the
//! process, moving its measured volume between roughly 900 and 2800.
//!
//! This test pins the resulting structure. It cannot observe cross-process
//! variance from inside one process, but it fails in most runs if the
//! collection order goes back to being seed-dependent: only one decomposition
//! matches the constants below.
//!
//! Keep it active alongside `perf_64cut_determinism` — divergence means
//! topology construction has become order-dependent again. To check across
//! processes directly, run `cargo run --release --example determinism_sweep -p
//! brepkit-operations` several times and diff the output.

#![allow(clippy::unwrap_used)]

use brepkit_operations::measure::solid_volume;
use brepkit_operations::primitives::make_cylinder;
use brepkit_topology::Topology;
use brepkit_topology::explorer::solid_faces;

#[test]
fn shelled_cylinder_rim_is_deterministic() {
    let (r, h, wall) = (10.0, 16.0, 1.2);

    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, r, h).unwrap();
    let top: Vec<_> = solid_faces(&topo, cyl)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            topo.face(f)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| (n.z() - 1.0).abs() < 1e-6)
        })
        .collect();
    let shelled = brepkit_operations::shell_op::shell(&mut topo, cyl, wall, &top).unwrap();

    let faces = solid_faces(&topo, shelled).unwrap();
    assert_eq!(faces.len(), 5, "shelled cup face count");

    // The field that used to vary: how the rim boundary decomposed into loops.
    let mut inner_counts: Vec<usize> = faces
        .iter()
        .map(|&f| topo.face(f).unwrap().inner_wires().len())
        .collect();
    inner_counts.sort_unstable();
    assert_eq!(
        inner_counts,
        vec![0, 0, 0, 0, 3],
        "rim loop decomposition changed — the open-boundary edge order is \
         order-dependent again (or the rim decomposition was deliberately fixed, \
         in which case update this and the volume below together)"
    );

    // A stability gate, NOT a correctness one. The analytic cup volume is
    // pi*(r^2*h - (r-wall)^2*(h-wall)) = 1425.93; this is ~20% under.
    //
    // The rim is genuinely mis-traced, not merely mis-ordered: the boundary
    // handed to `sort_edges_into_loops` also contains free edges from the
    // BOTTOM faces (points at z=0 and z=wall), because the assembled outer and
    // inner faces are not edge-shared there. Two of the four loops it returns
    // jump across the solid instead of ringing the opening. Ordering only
    // decides WHICH wrong decomposition comes out; fixing it means making the
    // bottom faces share edges. Pinned so the number cannot drift silently
    // meanwhile.
    let vol = solid_volume(&topo, shelled, 0.05).unwrap();
    assert!(
        (vol - 1_133.391_160_851_742_6).abs() < 1e-3,
        "shelled cup volume drifted: {vol} (expected 1133.3911608517426; analytic is 1425.93)"
    );
}
