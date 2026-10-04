//! Byte-level digest of the Hammer Holder display mesh (PERF-D07).
//!
//! The whole-mesh passes of the solid tessellator (circle contact
//! refinement, weld, coincident-triangle dedupe, gap fill) were rewritten
//! for speed without changing their output. This pins the complete mesh
//! (indices, face offsets, position and normal bits) of the source body and
//! of the OpenZCAD offset edit at two deflections against digests recorded
//! from the implementation before the rewrite.
//!
//! The digests depend on the platform's `libm` (`sin`, `cos`, `atan2` are
//! not correctly rounded and differ in the last bit between C libraries), so
//! they are asserted only on the platform they were recorded on
//! (aarch64 macOS). Everywhere else the test still meshes both bodies and
//! checks them watertight and deterministic. The pass rewrites are pinned
//! on every platform by the reference-implementation comparisons in
//! `remus-operations` (`tessellate::tests::mesh_passes`).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::step::reader::read_step;
use remus_operations::measure::face_area;
use remus_operations::push_pull::move_faces;
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, disable_face_mesh_cache, non_manifold_edge_count,
    tessellate_solid_grouped_with_tolerance,
};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const HAMMER_HOLDER: &str = include_str!("data/shapr3d_hammer_holder.step");
const ANGULAR: f64 = 0.06;

/// `(deflection, source digest, moved digest)` recorded on aarch64 macOS.
const EXPECTED: [(f64, u64, u64); 2] = [
    (0.0148, 0x1ff3_1ab7_f5f5_088f, 0x3b42_10f1_b7d6_4e89),
    (0.1, 0xd7e9_273d_6634_8587, 0x3751_1c78_4c0f_6155),
];

/// FNV-1a over the mesh bytes: deterministic on every platform.
fn digest(mesh: &TriangleMesh, offsets: &[u32]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut eat = |bytes: &[u8]| {
        for &b in bytes {
            h ^= u64::from(b);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    eat(&(mesh.indices.len() as u64).to_le_bytes());
    for &i in &mesh.indices {
        eat(&i.to_le_bytes());
    }
    eat(&(offsets.len() as u64).to_le_bytes());
    for &o in offsets {
        eat(&o.to_le_bytes());
    }
    eat(&(mesh.positions.len() as u64).to_le_bytes());
    for p in &mesh.positions {
        for c in [p.x(), p.y(), p.z()] {
            eat(&c.to_bits().to_le_bytes());
        }
    }
    eat(&(mesh.normals.len() as u64).to_le_bytes());
    for n in &mesh.normals {
        for c in [n.x(), n.y(), n.z()] {
            eat(&c.to_bits().to_le_bytes());
        }
    }
    h
}

/// The +X planar face OpenZCAD's offset harness edits (area 1045.93).
fn edited_face(topo: &Topology, solid: SolidId) -> FaceId {
    let mut best = None;
    for face in solid_faces(topo, solid).unwrap() {
        if let FaceSurface::Plane { normal, .. } = topo.face(face).unwrap().surface()
            && normal.x() > 0.99
        {
            let error = (face_area(topo, face, 0.05).unwrap() - 1045.93).abs();
            if best.is_none_or(|(_, e)| error < e) {
                best = Some((face, error));
            }
        }
    }
    best.expect("a +X planar face").0
}

fn mesh_digest(topo: &Topology, solid: SolidId, deflection: f64) -> u64 {
    let (mesh, offsets) =
        tessellate_solid_grouped_with_tolerance(topo, solid, deflection, ANGULAR).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0, "open mesh at {deflection}");
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "branching mesh at {deflection}"
    );
    let again = tessellate_solid_grouped_with_tolerance(topo, solid, deflection, ANGULAR).unwrap();
    let d = digest(&mesh, &offsets);
    assert_eq!(d, digest(&again.0, &again.1), "nondeterministic mesh");
    d
}

#[test]
fn hammer_holder_display_mesh_digest_is_pinned() {
    disable_face_mesh_cache();
    let mut topo = Topology::new();
    let solid = read_step(HAMMER_HOLDER, &mut topo).unwrap()[0];
    let face = edited_face(&topo, solid);
    let moved = move_faces(&mut topo, solid, &[face], -6.0).unwrap();

    let mut actual = Vec::new();
    for (deflection, _, _) in EXPECTED {
        actual.push((
            deflection,
            mesh_digest(&topo, solid, deflection),
            mesh_digest(&topo, moved, deflection),
        ));
    }
    if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
        for (expected, got) in EXPECTED.iter().zip(&actual) {
            assert_eq!(
                (expected.1, expected.2),
                (got.1, got.2),
                "Hammer Holder mesh digest changed at deflection {}: {actual:#x?}",
                expected.0
            );
        }
    }
}
