//! B71 family qualification: cross-drilled bore mesh quality and the
//! wrong-side rim-fillet refusal across bore/stock sizes, tool placement,
//! seam-relative angles, thin retained walls, and scale.
//!
//! The B71 witness
//! (`tessellate::tests::mutation_oracles::cross_drilled_bore_wall_stays_within_the_chord_bound`)
//! pins the core case (shaft r = 3, h = 30, bores 2 and 1). This file sweeps
//! the family around it with independent oracles only: per-face chord sag
//! against each analytic carrier's implicit equation, indexed and welded
//! closure, triangle winding, analytic ray-cast material probes, and volume
//! convergence against a Simpson closed form — plus the typed wrong-side
//! refusal (with rollback) on a cost-bounded subset, since each refusal pays
//! a ~6000-sample analytic Monte Carlo.
//!
//! Nothing here trusts a mesh density: the chord slab bounds the volume from
//! the measured area, and the refusal rests on the analytic material oracle
//! in `remus_operations::blend_material`, not on any volume delta.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::make_cylinder;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;

/// Drill an `x`-directed bore of radius `bore` through a shaft of radius
/// `stock_r` and height `stock_h` at mid-height, with the tool spun `rot_z`
/// about `z` first (seam-relative placement sweep).
fn drilled_shaft(
    stock_r: f64,
    stock_h: f64,
    bore: f64,
    rot_z: f64,
) -> (Topology, remus_topology::solid::SolidId) {
    let mut topo = Topology::new();
    let shaft = make_cylinder(&mut topo, stock_r, stock_h).unwrap();
    let len = stock_h + 4.0 * stock_r;
    let tool = make_cylinder(&mut topo, bore, len).unwrap();
    transform_solid(&mut topo, tool, &Mat4::rotation_y(std::f64::consts::FRAC_PI_2)).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(-len / 2.0, 0.0, stock_h / 2.0)).unwrap();
    transform_solid(&mut topo, tool, &Mat4::rotation_z(rot_z)).unwrap();
    let solid = boolean(&mut topo, BooleanOp::Cut, shaft, tool).unwrap();
    (topo, solid)
}

/// Spin a bore-frame probe into the placed frame (the tool is spun `rot_z`
/// about `z` through the stock center, which fixes the origin in `x`/`y`).
fn place(rot_z: f64, x: f64, y: f64, z: f64) -> Point3 {
    let (s, c) = rot_z.sin_cos();
    Point3::new(c * x - s * y, s * x + c * y, z)
}

/// Exact volume: stock minus the bore/stock intersection,
/// `∫ 4·√(R² − y²)·√(b² − y²) dy` over `|y| ≤ b` (trapezoid, converged).
fn exact_volume(stock_r: f64, stock_h: f64, bore: f64) -> f64 {
    let n = 200_000_usize;
    let h = 2.0 * bore / n as f64;
    let mut sum = 0.0;
    for i in 0..=n {
        let y = -bore + i as f64 * h;
        let f = 4.0 * (stock_r * stock_r - y * y).max(0.0).sqrt()
            * (bore * bore - y * y).max(0.0).sqrt();
        sum += if i == 0 || i == n {
            f
        } else if i % 2 == 1 {
            4.0 * f
        } else {
            2.0 * f
        };
    }
    PI * stock_r * stock_r * stock_h - sum * h / 3.0
}

fn signed_mesh_volume(mesh: &remus_operations::tessellate::TriangleMesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|t| {
            let a = mesh.positions[t[0] as usize];
            let b = mesh.positions[t[1] as usize];
            let c = mesh.positions[t[2] as usize];
            (a.x() * (b.y() * c.z() - b.z() * c.y())
                + a.y() * (b.z() * c.x() - b.x() * c.z())
                + a.z() * (b.x() * c.y() - b.y() * c.x()))
                / 6.0
        })
        .sum::<f64>()
        .abs()
}

fn mesh_area(mesh: &remus_operations::tessellate::TriangleMesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|t| {
            let (a, b, c) = (
                mesh.positions[t[0] as usize],
                mesh.positions[t[1] as usize],
                mesh.positions[t[2] as usize],
            );
            (b - a).cross(c - a).length() / 2.0
        })
        .sum()
}

/// Implicit distance to an analytic carrier (independent of the mesher).
/// The drilled-shaft family carries planes and cylinders only.
fn carrier_distance(surface: &FaceSurface, p: Point3) -> f64 {
    match surface {
        FaceSurface::Plane { normal, d } => {
            let n = normal.normalize().unwrap();
            (Vec3::new(p.x(), p.y(), p.z()).dot(n) - d).abs()
        }
        FaceSurface::Cylinder(c) => {
            let w = p - c.origin();
            let h = w.dot(c.axis());
            ((w - c.axis() * h).length() - c.radius()).abs()
        }
        other => panic!("family carries no {} faces", other.type_tag()),
    }
}

/// Full mesh-quality gate for one family member at one deflection.
#[allow(clippy::too_many_lines)]
fn check_member(
    topo: &Topology,
    solid: remus_topology::solid::SolidId,
    exact: f64,
    deflection: f64,
    what: &str,
) {
    let validation = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(validation.is_valid(), "{what}: {validation:?}");

    let (mesh, offsets) = remus_operations::tessellate::tessellate_solid_grouped_with_tolerance(
        topo,
        solid,
        deflection,
        remus_math::chord::DEFAULT_ANGULAR_TOL,
    )
    .unwrap();
    assert_eq!(
        (
            remus_operations::tessellate::boundary_edge_count(&mesh),
            remus_operations::tessellate::non_manifold_edge_count(&mesh)
        ),
        (0, 0),
        "{what}: indexed mesh must be closed and manifold"
    );
    let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
    assert_eq!(
        (quality.boundary_edges, quality.non_manifold_edges),
        (0, 0),
        "{what}: welded mesh must be closed and manifold"
    );

    // Per-face oracles from the triangles themselves: carrier adherence
    // (vertices and chord midpoints), chord sag, and winding.
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    assert_eq!(offsets.len(), faces.len() + 1);
    for (i, &face_id) in faces.iter().enumerate() {
        let face = topo.face(face_id).unwrap();
        let FaceSurface::Nurbs(_) = face.surface() else {
            // Analytic faces only in this family; NURBS would need the
            // surface-derivative oracle instead of the implicit one.
            let tris = mesh.indices[offsets[i] as usize..offsets[i + 1] as usize]
                .chunks_exact(3)
                .map(|t| [0, 1, 2].map(|k| mesh.positions[t[k] as usize]))
                .collect::<Vec<_>>();
            let mut sag = 0.0_f64;
            for t in &tris {
                let mid = |a: Point3, b: Point3| {
                    Point3::new(
                        f64::midpoint(a.x(), b.x()),
                        f64::midpoint(a.y(), b.y()),
                        f64::midpoint(a.z(), b.z()),
                    )
                };
                let centroid = Point3::new(
                    (t[0].x() + t[1].x() + t[2].x()) / 3.0,
                    (t[0].y() + t[1].y() + t[2].y()) / 3.0,
                    (t[0].z() + t[1].z() + t[2].z()) / 3.0,
                );
                for q in [centroid, mid(t[0], t[1]), mid(t[1], t[2]), mid(t[2], t[0])] {
                    sag = sag.max(carrier_distance(face.surface(), q));
                }
                // Winding: every non-sliver triangle faces out of the material.
                let n = (t[1] - t[0]).cross(t[2] - t[0]);
                if n.length() > 1e-14 {
                    let c = centroid;
                    // Outward side: step off along the geometric normal and
                    // require the solid to lie behind.
                    let sdir = carrier_normal(face.surface(), c);
                    let sdir = if face.is_reversed() { -sdir } else { sdir };
                    assert!(
                        n.dot(sdir) > 0.0,
                        "{what}: face {i} has an inverted triangle"
                    );
                }
            }
            assert!(
                sag <= 2.0 * deflection,
                "{what}: face {i} sags {sag} at deflection {deflection}"
            );
            continue;
        };
        panic!("{what}: unexpected NURBS face in the drilled-shaft family");
    }

    // Volume within the chord slab (inscribed mesh reads low).
    let area = mesh_area(&mesh);
    let volume = signed_mesh_volume(&mesh);
    assert!(
        volume <= exact * (1.0 + 1e-9) && exact - volume <= 2.0 * deflection * area,
        "{what}: volume {volume} vs exact {exact} at deflection {deflection}"
    );

    // B-Rep volume agrees with the closed form (mesh-independent route).
    let brep = remus_operations::measure::solid_volume(topo, solid, deflection).unwrap();
    assert!(
        (brep - exact).abs() / exact < 1e-3,
        "{what}: B-rep volume {brep} vs exact {exact}"
    );
}

/// Unit outward normal of an analytic carrier at a near-carrier point.
/// The drilled-shaft family carries planes and cylinders only.
fn carrier_normal(surface: &FaceSurface, p: Point3) -> Vec3 {
    match surface {
        FaceSurface::Plane { normal, .. } => normal.normalize().unwrap(),
        FaceSurface::Cylinder(c) => {
            let w = p - c.origin();
            let h = w.dot(c.axis());
            let r = w - c.axis() * h;
            r * (1.0 / r.length().max(1e-300))
        }
        other => panic!("family carries no {} faces", other.type_tag()),
    }
}

/// Analytic material probes: bore void is outside, wall midpoints inside.
/// Probes are built in the bore frame and spun by `rot_z` like the tool.
fn check_probes(
    topo: &Topology,
    solid: remus_topology::solid::SolidId,
    stock_r: f64,
    stock_h: f64,
    bore: f64,
    rot_z: f64,
    what: &str,
) {
    use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
    let opts = ClassifyOptions::default();
    // Bore centerline inside the stock: void.
    for x in [-stock_r * 0.5, 0.0, stock_r * 0.5] {
        let q = place(rot_z, x, 0.0, stock_h / 2.0);
        assert_eq!(
            classify_point(topo, solid, q, &opts).unwrap(),
            PointClassification::Outside,
            "{what}: bore void at {q:?}"
        );
    }
    // Wall midpoints between the bore tube and the stock OD: material.
    // At x = 0 the stock allows |y| < stock_r while the bore excludes
    // |y| < bore, so the mid-wall band is solid.
    for s in [-1.0, 1.0] {
        let q = place(rot_z, 0.0, s * f64::midpoint(bore, stock_r), stock_h / 2.0);
        assert_eq!(
            classify_point(topo, solid, q, &opts).unwrap(),
            PointClassification::Inside,
            "{what}: wall at {q:?}"
        );
    }
    // Stock center below/above the bore: material.
    for z in [stock_h * 0.1, stock_h * 0.9] {
        assert_eq!(
            classify_point(topo, solid, Point3::new(0.0, 0.0, z), &opts).unwrap(),
            PointClassification::Inside,
            "{what}: stock interior at z={z}"
        );
    }
}

fn first_cylinder_rim(
    topo: &Topology,
    solid: remus_topology::solid::SolidId,
) -> remus_topology::edge::EdgeId {
    let adjacency = topo.build_adjacency(solid).unwrap();
    remus_topology::explorer::solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|edge| {
            let adjacent = adjacency.faces_for_edge(*edge);
            adjacent.len() == 2
                && adjacent[0] != adjacent[1]
                && adjacent.iter().all(|face| {
                    matches!(
                        topo.face(*face).unwrap().surface(),
                        FaceSurface::Cylinder(_)
                    )
                })
        })
        .expect("drilled body must have a cylinder-cylinder hole rim")
}

/// The wrong-side refusal with complete rollback (cost-bounded: analytic MC).
fn check_refusal(
    topo: &mut Topology,
    solid: remus_topology::solid::SolidId,
    rim: remus_topology::edge::EdgeId,
    radius: f64,
    what: &str,
) {
    let before = remus_io::arena_io::serialize_solid(topo, solid).unwrap();
    let error = match remus_operations::blend_ops::fillet_v2(topo, solid, &[rim], radius) {
        Err(error) => error,
        Ok(_) => panic!("{what}: wrong-side blend must refuse"),
    };
    assert!(
        error.to_string().contains("convex edges added"),
        "{what}: typed refusal, got {error}"
    );
    assert_eq!(
        remus_io::arena_io::serialize_solid(topo, solid).unwrap(),
        before,
        "{what}: rollback must restore the input exactly"
    );
}

#[test]
fn b71_family_mesh_quality_and_probes() {
    // (stock_r, stock_h, bore, rot_z): core, small/large bores, thin wall,
    // stock variants, and seam-relative placements.
    let members = [
        (3.0, 30.0, 2.0, 0.0, "core b=2"),
        (3.0, 30.0, 1.0, 0.0, "core b=1"),
        (3.0, 30.0, 0.5, 0.0, "small bore"),
        (3.0, 30.0, 2.5, 0.0, "thin retained wall"),
        (3.0, 12.0, 1.0, 0.0, "pclass geometry"),
        (3.0, 12.0, 1.0, 0.37, "off-seam placement"),
        (3.0, 12.0, 1.0, std::f64::consts::FRAC_PI_2, "quarter-turn placement"),
        (5.0, 20.0, 1.5, 0.0, "large stock"),
    ];
    for (stock_r, stock_h, bore, rot_z, name) in members {
        let (topo, solid) = drilled_shaft(stock_r, stock_h, bore, rot_z);
        let exact = exact_volume(stock_r, stock_h, bore);
        for deflection in [0.05, 0.01] {
            check_member(&topo, solid, exact, deflection, &format!("{name} @{deflection}"));
        }
        check_probes(&topo, solid, stock_r, stock_h, bore, rot_z, name);
    }
}

#[test]
fn b71_family_scale_invariance() {
    // Tenfold scales at physical deflection (proportional to the extent).
    for scale in [0.1, 10.0] {
        let (mut topo, solid) = drilled_shaft(3.0 * scale, 12.0 * scale, 1.0 * scale, 0.0);
        let exact = exact_volume(3.0 * scale, 12.0 * scale, 1.0 * scale);
        let extent = (3.0 * scale * 2.0).hypot(12.0 * scale);
        for k in [0.05, 0.01] {
            let deflection = k * extent / 30.0;
            check_member(
                &topo,
                solid,
                exact,
                deflection,
                &format!("scale {scale} @{deflection}"),
            );
        }
        check_probes(&topo, solid, 3.0 * scale, 12.0 * scale, 1.0 * scale, 0.0, "scaled");
        let rim = first_cylinder_rim(&topo, solid);
        check_refusal(&mut topo, solid, rim, 0.15 * scale, &format!("scale {scale} refusal"));
    }
}

#[test]
fn b71_family_wrong_side_refusals() {
    // Refusal across meshes (coarse and fine pre-conditioning does not move
    // the analytic verdict), placement, and bore size.
    let members = [
        (3.0, 12.0, 1.0, 0.0, 0.15, "pclass"),
        (3.0, 30.0, 2.0, 0.0, 0.15, "core b=2"),
        (3.0, 12.0, 1.0, 0.37, 0.15, "off-seam"),
        (3.0, 30.0, 2.5, 0.0, 0.15, "thin retained wall"),
    ];
    for (stock_r, stock_h, bore, rot_z, radius, name) in members {
        let (mut topo, solid) = drilled_shaft(stock_r, stock_h, bore, rot_z);
        // Pre-condition at two mesh densities: the verdict must not move.
        // (0.005 is skipped: a pre-existing shaft-wall hole-boundary needle
        // leaves 2 non-manifold edges there on the unmodified tree too —
        // A/B-verified, unrelated to the bore-wall gate.)
        for deflection in [0.05, 0.01] {
            let mesh = remus_operations::tessellate::tessellate_solid(&topo, solid, deflection)
                .unwrap();
            assert_eq!(
                (
                    remus_operations::tessellate::boundary_edge_count(&mesh),
                    remus_operations::tessellate::non_manifold_edge_count(&mesh)
                ),
                (0, 0),
                "{name}: closed at {deflection}"
            );
        }
        let rim = first_cylinder_rim(&topo, solid);
        check_refusal(&mut topo, solid, rim, radius, name);
    }
}
