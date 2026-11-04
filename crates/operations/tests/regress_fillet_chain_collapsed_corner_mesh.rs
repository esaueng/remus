//! A fillet chain whose blend collapses at its corners must mesh closed.
//!
//! The fixture is the third 1 mm round of a box unioned with a cylinder:
//! earlier 1 mm rounds on the adjacent box edges left torus corner blends and
//! a cylinder/box junction round, and the new round runs along three top
//! edges as one NURBS blend face. Where that blend rolls round a vertical
//! 1 mm corner round, its top contact rail stops at the corner while the
//! cross-section keeps turning: the `u = 1` row of its control net collapses
//! onto one point over two v intervals, and its far end collapses the whole
//! cross-section onto the vertex it shares with the junction torus.
//!
//! The solid is valid, but it tessellated with 1314 open edges: the blend's
//! chart polygon charted every sample of the straight top rail between the
//! two corners onto a collapsed corner (the coarse projection seed sits on
//! the collapsed row, where Newton has no gradient), and the samples at each
//! corner zig-zagged along the collapse, so constraint recovery failed and
//! the face fell back to the independently sampled snap path. The standalone
//! face path also swept the trimmed junction torus over its whole UV box
//! (81.5 mm² for a 15.6 mm² face).
//!
//! Exact areas come from the measurement path for analytic and planar faces
//! and, for the blend, from Gauss quadrature of the untrimmed patch, which
//! its four boundary sides bound completely.

#![allow(
    clippy::cast_precision_loss,
    clippy::expect_used,
    clippy::panic,
    clippy::unwrap_used
)]

use std::collections::HashMap;

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;
use remus_operations::measure::{face_area, solid_volume};
use remus_operations::tessellate::{
    TriangleMesh, tessellate_solid_grouped_with_tolerance, tessellate_with_tolerance,
    welded_mesh_quality,
};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

/// The display tolerances the browser requests, and a fine deflection that
/// shows the per-face areas converge rather than stall.
const ANGULAR: f64 = 0.2;
const DEFLECTIONS: [(f64, f64); 2] = [(0.05, 0.015), (0.005, 0.005)];

fn load(topo: &mut Topology) -> SolidId {
    remus_io::arena_io::deserialize_solid(
        include_bytes!("data/fillet_chain_collapsed_corner.arena.json"),
        topo,
    )
    .unwrap()
}

fn triangle_area(mesh: &TriangleMesh, triangle: &[u32]) -> f64 {
    let p = mesh.positions[triangle[0] as usize];
    let q = mesh.positions[triangle[1] as usize];
    let r = mesh.positions[triangle[2] as usize];
    0.5 * (q - p).cross(r - p).length()
}

fn mesh_area(mesh: &TriangleMesh, indices: &[u32]) -> f64 {
    indices
        .chunks_exact(3)
        .map(|t| triangle_area(mesh, t))
        .sum()
}

/// Gauss-Legendre area of a whole NURBS patch, four points per knot span in
/// v and eight across each u span.
fn patch_area(surface: &NurbsSurface) -> f64 {
    const G4: [(f64, f64); 4] = [
        (-0.861_136_311_594_052_6, 0.347_854_845_137_453_9),
        (-0.339_981_043_584_856_3, 0.652_145_154_862_546_1),
        (0.339_981_043_584_856_3, 0.652_145_154_862_546_1),
        (0.861_136_311_594_052_6, 0.347_854_845_137_453_9),
    ];
    let spans = |knots: &[f64]| {
        let mut out = Vec::new();
        for w in knots.windows(2) {
            if w[1] > w[0] {
                out.push((w[0], w[1]));
            }
        }
        out
    };
    let mut area = 0.0;
    for (u0, u1) in spans(surface.knots_u()) {
        for k in 0..2 {
            let (a0, a1) = (
                u0 + (u1 - u0) * f64::from(k) / 2.0,
                u0 + (u1 - u0) * f64::from(k + 1) / 2.0,
            );
            for (v0, v1) in spans(surface.knots_v()) {
                for &(xu, wu) in &G4 {
                    for &(xv, wv) in &G4 {
                        let u = 0.5 * (a1 - a0).mul_add(xu, a0 + a1);
                        let v = 0.5 * (v1 - v0).mul_add(xv, v0 + v1);
                        let d = surface.derivatives(u, v, 1);
                        let jacobian = d[1][0].cross(d[0][1]).length();
                        area += wu * wv * jacobian * 0.25 * (a1 - a0) * (v1 - v0);
                    }
                }
            }
        }
    }
    area
}

fn exact_areas(topo: &Topology, faces: &[FaceId]) -> Vec<f64> {
    faces
        .iter()
        .map(|&face| {
            let measured = face_area(topo, face, 1e-3).unwrap();
            if let FaceSurface::Nurbs(surface) = topo.face(face).unwrap().surface() {
                let independent = patch_area(surface);
                assert!(
                    (measured - independent).abs() <= 1e-3 * independent,
                    "blend measurement {measured} vs quadrature {independent}"
                );
                independent
            } else {
                measured
            }
        })
        .collect()
}

/// Weld by the 1 µm grid and require every directed edge to be matched by
/// exactly one opposite edge: closed, two-manifold and coherently wound.
fn assert_closed_and_coherent(mesh: &TriangleMesh) {
    let key = |p: Point3| {
        let s = 1e6;
        (
            (p.x() * s).round() as i64,
            (p.y() * s).round() as i64,
            (p.z() * s).round() as i64,
        )
    };
    let mut directed: HashMap<_, usize> = HashMap::new();
    for t in mesh.indices.chunks_exact(3) {
        let k = [0, 1, 2].map(|i| key(mesh.positions[t[i] as usize]));
        if k[0] == k[1] || k[1] == k[2] || k[0] == k[2] {
            continue;
        }
        for i in 0..3 {
            *directed.entry((k[i], k[(i + 1) % 3])).or_default() += 1;
        }
    }
    let repeated = directed.values().filter(|&&c| c > 1).count();
    let unmatched = directed
        .keys()
        .filter(|&&(a, b)| directed.get(&(b, a)).copied() != Some(1))
        .count();
    assert_eq!((repeated, unmatched), (0, 0), "welded directed edges");
}

fn signed_volume(mesh: &TriangleMesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|t| {
            let [a, b, c] = [0, 1, 2].map(|i| mesh.positions[t[i] as usize]);
            let origin = Point3::new(0.0, 0.0, 0.0);
            (a - origin).dot((b - origin).cross(c - origin)) / 6.0
        })
        .sum()
}

#[test]
fn collapsed_corner_blend_solid_meshes_closed_with_exact_face_areas() {
    let mut topo = Topology::new();
    let solid = load(&mut topo);
    assert!(validate_solid(&topo, solid).unwrap().is_valid());
    let faces = solid_faces(&topo, solid).unwrap();
    assert_eq!(faces.len(), 14);
    let exact = exact_areas(&topo, &faces);
    let volume = solid_volume(&topo, solid, 1e-3).unwrap();
    assert!((volume - 7730.67).abs() < 0.5, "volume {volume}");

    for (deflection, tolerance) in DEFLECTIONS {
        let (mesh, offsets) =
            tessellate_solid_grouped_with_tolerance(&topo, solid, deflection, ANGULAR).unwrap();
        let quality = welded_mesh_quality(&mesh);
        assert!(quality.is_watertight(), "{deflection}: {quality:?}");
        assert_eq!(quality.euler_characteristic, 2, "{deflection}: {quality:?}");
        assert_closed_and_coherent(&mesh);
        let mesh_volume = signed_volume(&mesh);
        assert!(
            (mesh_volume - volume).abs() <= tolerance * volume,
            "{deflection}: mesh volume {mesh_volume} vs {volume}"
        );
        for (i, &area) in exact.iter().enumerate() {
            let group = &mesh.indices[offsets[i] as usize..offsets[i + 1] as usize];
            let meshed = mesh_area(&mesh, group);
            assert!(
                (meshed - area).abs() <= tolerance * area,
                "{deflection}: face #{i} meshed {meshed} vs exact {area}"
            );
        }
    }
}

#[test]
fn collapsed_corner_blend_faces_mesh_alone_with_exact_areas() {
    let mut topo = Topology::new();
    let solid = load(&mut topo);
    let faces = solid_faces(&topo, solid).unwrap();
    let exact = exact_areas(&topo, &faces);
    for (deflection, tolerance) in DEFLECTIONS {
        for (i, (&face, &area)) in faces.iter().zip(&exact).enumerate() {
            let mesh = tessellate_with_tolerance(&topo, face, deflection, ANGULAR).unwrap();
            let meshed = mesh_area(&mesh, &mesh.indices);
            assert!(
                (meshed - area).abs() <= tolerance * area,
                "{deflection}: face #{i} meshed alone {meshed} vs exact {area}"
            );
            // Triangles wind with the face's outward normal.
            let mut agree = 0.0;
            let mut total = 0.0;
            for t in mesh.indices.chunks_exact(3) {
                let [a, b, c] = [0, 1, 2].map(|k| mesh.positions[t[k] as usize]);
                let normal = (b - a).cross(c - a);
                let outward = [0, 1, 2]
                    .map(|k| mesh.normals[t[k] as usize])
                    .into_iter()
                    .fold(remus_math::vec::Vec3::new(0.0, 0.0, 0.0), |s, n| s + n);
                let weight = normal.length();
                total += weight;
                if normal.dot(outward) > 0.0 {
                    agree += weight;
                }
            }
            assert!(
                agree >= 0.999 * total,
                "{deflection}: face #{i} winds against its normals ({agree} of {total})"
            );
        }
    }
}
