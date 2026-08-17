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
//! matches the constants below — and that one is now the right one, so the
//! volume below is the analytic cup rather than a pinned wrong number.
//!
//! Keep it active alongside `perf_64cut_determinism` — divergence means
//! topology construction has become order-dependent again. To check across
//! processes directly, run `cargo run --release --example determinism_sweep -p
//! remus-operations` several times and diff the output.

#![allow(clippy::unwrap_used)]

use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_cylinder;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;

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
    let shelled = remus_operations::shell_op::shell(&mut topo, cyl, wall, &top).unwrap();

    let faces = solid_faces(&topo, shelled).unwrap();
    assert_eq!(faces.len(), 5, "shelled cup face count");

    // The field that used to vary: how the rim boundary decomposed into loops.
    // A cup's rim is ONE annulus — the wall's outer circle with the inner
    // circle as its hole — so exactly one face carries exactly one inner wire.
    let mut inner_counts: Vec<usize> = faces
        .iter()
        .map(|&f| topo.face(f).unwrap().inner_wires().len())
        .collect();
    inner_counts.sort_unstable();
    assert_eq!(
        inner_counts,
        vec![0, 0, 0, 0, 1],
        "rim loop decomposition changed — the open-boundary edge order is \
         order-dependent again"
    );

    // Now a correctness gate as well. This used to read 1133.39 against the
    // analytic 1425.93, 20% under, because the boundary handed to
    // `sort_edges_into_loops` also carried free edges from the BOTTOM faces:
    // the wall's polygon closed neither of its rim circles, so the bottom cap
    // had nothing to share them with, and two of the four loops that came back
    // jumped across the solid instead of ringing the opening.
    let analytic = std::f64::consts::PI * (r * r * h - (r - wall).powi(2) * (h - wall));
    let vol = solid_volume(&topo, shelled, 0.05).unwrap();
    assert!(
        (vol - analytic).abs() < 1e-4 * analytic,
        "shelled cup volume {vol} is not the analytic {analytic}"
    );
}
