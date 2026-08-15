//! Regression: phase EF must not pave an edge that crosses a planar face's
//! HOLE rather than its material.
//!
//! `build_face_containment` sampled a face's inner wires into the AABB but
//! built the planar containment polygon from the OUTER wire alone, so
//! `accepts()` treated an annulus as the full disc its outer rim bounds.
//!
//! Fusing the flange rim (r24..45, z0..10) with the hub (r12..24, z0..26) put
//! the hub's r12 bore seam — a line at radius 12, in open space as far as the
//! rim is concerned — against the rim's z=10 cap, an annulus spanning r24..45.
//! The cap accepted the crossing at (12, 0, 10) and paved a vertex there,
//! splitting the full-height bore seam into two Line edges.
//!
//! Every B-Rep gate still passed: each half is used exactly twice, so edge
//! usage reads 0 free / 0 non-manifold and validation is clean. Only the
//! cylinder band mesher noticed, mis-stitching the split seam into 8 unmatched
//! mesh edges — an STL with a hole in it, from a solid that every topological
//! check called watertight. That is why this test asserts on the MESH as well
//! as the B-Rep.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::heal::unify_faces;
use remus_operations::revolve::revolve;
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_topology::Topology;
use remus_topology::builder::{make_planar_face_from_wire, make_polygon_wire};
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

const TOL: f64 = 1e-7;

/// `(boundary, non_manifold)` mesh edge counts after welding by position.
fn mesh_edge_counts(topo: &Topology, solid: SolidId) -> (usize, usize) {
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01, 0.1).unwrap();
    let q = 1e6;
    let mut canon: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut remap = vec![0u32; mesh.positions.len()];
    for (i, p) in mesh.positions.iter().enumerate() {
        let key = (
            (p.x() * q).round() as i64,
            (p.y() * q).round() as i64,
            (p.z() * q).round() as i64,
        );
        let next = canon.len() as u32;
        remap[i] = *canon.entry(key).or_insert(next);
    }
    let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let v = [
            remap[tri[0] as usize],
            remap[tri[1] as usize],
            remap[tri[2] as usize],
        ];
        for &(a, b) in &[(v[0], v[1]), (v[1], v[2]), (v[2], v[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    (
        edges.values().filter(|&&c| c == 1).count(),
        edges.values().filter(|&&c| c >= 3).count(),
    )
}

fn revolved_annulus(
    topo: &mut Topology,
    r_inner: f64,
    r_outer: f64,
    z_lo: f64,
    z_hi: f64,
) -> SolidId {
    let pts = [
        Point3::new(r_inner, 0.0, z_lo),
        Point3::new(r_outer, 0.0, z_lo),
        Point3::new(r_outer, 0.0, z_hi),
        Point3::new(r_inner, 0.0, z_hi),
    ];
    let wire = make_polygon_wire(topo, &pts, TOL).unwrap();
    let face = make_planar_face_from_wire(topo, wire).unwrap();
    revolve(
        topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::TAU,
    )
    .unwrap()
}

fn flange_blank(topo: &mut Topology) -> SolidId {
    let rim = revolved_annulus(topo, 24.0, 45.0, 0.0, 10.0);
    let hub = revolved_annulus(topo, 12.0, 24.0, 0.0, 26.0);
    let blank = boolean(topo, BooleanOp::Fuse, rim, hub).expect("blank fuse must succeed");
    unify_faces(topo, blank).unwrap();
    blank
}

/// Count the Line (seam) edges on the r12 bore face. One seam, traversed
/// twice, is 2; a seam split at z=10 gives 4.
fn bore_seam_edge_count(topo: &Topology, solid: SolidId) -> usize {
    let mut n = 0;
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        if face.surface().type_tag() != "cylinder" {
            continue;
        }
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                let e = topo.edge(oe.edge()).unwrap();
                let a = topo.vertex(e.start()).unwrap().point();
                if (a.x().hypot(a.y()) - 12.0).abs() < 1e-6 && e.curve().type_tag() == "line" {
                    n += 1;
                }
            }
        }
    }
    n
}

#[test]
fn bore_seam_is_not_split_by_a_face_hole() {
    let mut topo = Topology::new();
    let blank = flange_blank(&mut topo);

    // The rim cap's hole must not pave the bore seam at (12, 0, 10).
    assert_eq!(
        bore_seam_edge_count(&topo, blank),
        2,
        "the r12 bore seam must stay one full-height edge (used twice), not be \
         split at z=10 by the rim cap's hole"
    );

    // No vertex may exist at bore radius, mid-height.
    for fid in solid_faces(&topo, blank).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                let e = topo.edge(oe.edge()).unwrap();
                for v in [e.start(), e.end()] {
                    let p = topo.vertex(v).unwrap().point();
                    let r = p.x().hypot(p.y());
                    assert!(
                        !((r - 12.0).abs() < 1e-6 && (p.z() - 10.0).abs() < 1e-6),
                        "spurious vertex at the bore radius, rim-cap height: {p:?}"
                    );
                }
            }
        }
    }

    // The payoff: the mesh is watertight. The B-Rep passed even when split.
    assert_eq!(
        mesh_edge_counts(&topo, blank),
        (0, 0),
        "the fused blank must tessellate watertight"
    );
}

/// A tool edge that genuinely meets a hole RIM must still pave — the fix
/// rejects only the strict interior of a hole, not its boundary.
#[test]
fn hole_rim_contact_still_paves() {
    let mut topo = Topology::new();
    let blank = flange_blank(&mut topo);

    // The hub's r24 wall meets the rim cap exactly on that cap's inner rim.
    // If hole-rim contacts stopped paving, this junction would come apart.
    let (free, nonmanifold) = {
        let mut usage: HashMap<usize, usize> = HashMap::new();
        for fid in solid_faces(&topo, blank).unwrap() {
            let face = topo.face(fid).unwrap();
            for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                for oe in topo.wire(wid).unwrap().edges() {
                    *usage.entry(oe.edge().index()).or_insert(0) += 1;
                }
            }
        }
        (
            usage.values().filter(|&&c| c == 1).count(),
            usage.values().filter(|&&c| c >= 3).count(),
        )
    };
    assert_eq!(
        (free, nonmanifold),
        (0, 0),
        "the r24 hub wall / rim cap inner-rim junction must stay closed"
    );

    // And the r24 circle at z=10 must still exist as a real shared edge.
    let mut found = false;
    for fid in solid_faces(&topo, blank).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                let e = topo.edge(oe.edge()).unwrap();
                let p = topo.vertex(e.start()).unwrap().point();
                if e.curve().type_tag() == "circle"
                    && (p.x().hypot(p.y()) - 24.0).abs() < 1e-6
                    && (p.z() - 10.0).abs() < 1e-6
                {
                    found = true;
                }
            }
        }
    }
    assert!(
        found,
        "the r24 circle at z=10 must survive as a shared edge"
    );
}
