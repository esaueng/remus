//! Regression: the v2 blend trimmer must propagate boundary-edge splits into
//! neighbor faces that are not themselves trimmed.
//!
//! A fillet's contact curve crosses the trimmed face's boundary mid-edge; the
//! crossed edge is shared with a cap face (here: the box end faces). Before
//! the fix, `split_edge_at` rebuilt only the trimmed face's wire, so the end
//! faces kept referencing the stale unsplit edge: the stale edge and the kept
//! sub-edge were each used by a single face, opening the shell along the
//! shared span (16 free B-Rep edges, 28 boundary mesh edges at export
//! tolerance for this configuration).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_blend::fillet_builder::FilletBuilder;
use remus_math::vec::Point3;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::{boundary_edge_count, tessellate_solid_with_tolerance};
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

fn edge_use_counts(topo: &Topology, solid: SolidId) -> HashMap<EdgeId, usize> {
    let mut counts: HashMap<EdgeId, usize> = HashMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        let mut wires = vec![face.outer_wire()];
        wires.extend(face.inner_wires().iter().copied());
        for wid in wires {
            for oe in topo.wire(wid).unwrap().edges() {
                *counts.entry(oe.edge()).or_insert(0) += 1;
            }
        }
    }
    counts
}

fn assert_wire_connected(topo: &Topology, fid: FaceId) {
    let wire = topo.wire(topo.face(fid).unwrap().outer_wire()).unwrap();
    let oes = wire.edges();
    for i in 0..oes.len() {
        let cur = topo.edge(oes[i].edge()).unwrap();
        let next_oe = oes[(i + 1) % oes.len()];
        let next = topo.edge(next_oe.edge()).unwrap();
        assert_eq!(
            oes[i].oriented_end(cur),
            next_oe.oriented_start(next),
            "wire of face {fid:?} is disconnected at position {i}"
        );
    }
}

fn wire_has_vertex_at(topo: &Topology, fid: FaceId, p: Point3) -> bool {
    let wire = topo.wire(topo.face(fid).unwrap().outer_wire()).unwrap();
    wire.edges().iter().any(|oe| {
        let e = topo.edge(oe.edge()).unwrap();
        [e.start(), e.end()]
            .iter()
            .any(|&vid| (topo.vertex(vid).unwrap().point() - p).length() < 1e-9)
    })
}

#[test]
fn fillet_v2_box_edge_propagates_boundary_splits() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();

    // Top-front edge: (0,0,10) -> (10,0,10).
    let fillet_edge = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&eid| {
            let e = topo.edge(eid).unwrap();
            let s = topo.vertex(e.start()).unwrap().point();
            let t = topo.vertex(e.end()).unwrap().point();
            (s.z() - 10.0).abs() < 1e-9
                && (t.z() - 10.0).abs() < 1e-9
                && s.y().abs() < 1e-9
                && t.y().abs() < 1e-9
        })
        .expect("top front edge");

    // The four edges sharing exactly one endpoint with the fillet edge are
    // the ones the contact curves cross mid-edge (at distance r = 1 from the
    // corner). Each is shared between a trimmed face and an untouched end
    // face.
    let fe = topo.edge(fillet_edge).unwrap();
    let fe_verts = [fe.start(), fe.end()];
    let split_candidates: Vec<EdgeId> = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&eid| {
            let e = topo.edge(eid).unwrap();
            let shared = usize::from(fe_verts.contains(&e.start()))
                + usize::from(fe_verts.contains(&e.end()));
            eid != fillet_edge && shared == 1
        })
        .collect();
    assert_eq!(split_candidates.len(), 4, "box corner adjacency");

    // Exercise the walking builder directly. The production operations API
    // tries its polygon-rebuilding implementation first for planar line
    // blends, so driving the builder here pins the walking trimmer and its
    // stitched assembly on their own.
    let mut builder = FilletBuilder::new(&mut topo, solid);
    builder.add_edges(&[fillet_edge], 1.0);
    let result = builder.build().unwrap();
    assert_eq!(result.succeeded, vec![fillet_edge]);
    assert!(result.failed.is_empty());

    let counts = edge_use_counts(&topo, result.solid);

    // The stale unsplit edges must not be referenced by ANY face of the
    // result: every wire that referenced them (including the untouched end
    // faces) must have been rebuilt onto the two sub-edges.
    for eid in &split_candidates {
        assert_eq!(
            counts.get(eid).copied().unwrap_or(0),
            0,
            "stale pre-split edge {eid:?} still referenced by a result face"
        );
    }

    // No edge may be over-shared, and every face wire must remain a
    // head-to-tail connected loop after the in-place neighbor rebuilds.
    assert!(
        counts.values().all(|&c| c <= 2),
        "over-shared edge after split propagation"
    );
    let faces = solid_faces(&topo, result.solid).unwrap();
    for &fid in &faces {
        assert_wire_connected(&topo, fid);
    }

    // Each end face (planes x=0 and x=10) gained both split vertices —
    // (x, 1, 10) from the top-face trim and (x, 0, 9) from the front-face
    // trim — and lost the sharp corner (x, 0, 10) to the fillet's end arc.
    // Net: 4 boundary edges become 5 (two splits add two, the arc replaces
    // the two sub-edges that met at the corner).
    //
    // The end-face notch used to be left open here: the splits landed but
    // nothing closed the wedge between them, so the shell had a hole at each
    // end. The stitched assembly now replaces that sub-edge pair in place
    // with the exact quarter-arc the blend wall terminates on.
    for x in [0.0, 10.0] {
        let end_face = faces
            .iter()
            .copied()
            .find(|&fid| {
                let f = topo.face(fid).unwrap();
                matches!(
                    f.surface(),
                    FaceSurface::Plane { normal, .. } if normal.x().abs() > 0.99
                ) && wire_has_vertex_at(&topo, fid, Point3::new(x, 10.0, 0.0))
            })
            .expect("end face");
        let n_edges = topo
            .wire(topo.face(end_face).unwrap().outer_wire())
            .unwrap()
            .edges()
            .len();
        assert_eq!(n_edges, 5, "end face at x={x} should gain a net edge");
        assert!(
            wire_has_vertex_at(&topo, end_face, Point3::new(x, 1.0, 10.0)),
            "end face at x={x} missing top-trim split vertex"
        );
        assert!(
            wire_has_vertex_at(&topo, end_face, Point3::new(x, 0.0, 9.0)),
            "end face at x={x} missing front-trim split vertex"
        );
        assert!(
            !wire_has_vertex_at(&topo, end_face, Point3::new(x, 0.0, 10.0)),
            "end face at x={x} still carries the corner the fillet removed"
        );
    }

    // Six box faces plus one blend wall — no stray patches.
    assert_eq!(faces.len(), 7, "expected 6 box faces + 1 blend wall");

    // Export-tolerance mesh (0.01 mm / 5 deg). Pre-fix this configuration
    // produced 28 boundary mesh edges: the shared spans tessellated as
    // unwelded T-junction cracks on both sides. With split propagation AND
    // the stitched end closure the mesh is fully watertight.
    let mesh =
        tessellate_solid_with_tolerance(&topo, result.solid, 0.01, 5.0_f64.to_radians()).unwrap();
    let bnd = boundary_edge_count(&mesh);
    assert_eq!(bnd, 0, "filleted box mesh must be watertight; bnd = {bnd}");

    // A rolled convex edge removes exactly (1 - pi/4)·r²·L = 2.146 mm³.
    let vol = remus_operations::measure::solid_volume(&topo, result.solid, 0.01).unwrap();
    assert!(
        (vol - 997.854).abs() < 0.01,
        "expected the exact quarter-round volume 997.854, got {vol}"
    );
}
