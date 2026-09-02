//! Regression: `unify_faces` on a wall a boolean split AT its seam.
//!
//! A boss standing flush on a shaft's base cuts a notch into the shaft wall
//! that reaches the bottom rim. The fuse hands back the wall in three pieces
//! — two below the boss's top, split from each other along the wall's seam,
//! and the band above — plus the full ring the notch's ceiling severs. Those
//! pieces lie on one cylinder, so `unify_faces` merges them, and it used to:
//!
//! - drop the seam edge below the notch's ceiling as an internal shared edge
//!   (it is shared between the two lower pieces like any other), so the
//!   merged face kept a seam that ran only part of the height; and
//! - walk the remaining boundary into a notch loop plus a loop of the top
//!   rim with the seam stub, filing the latter as an inner wire.
//!
//! The result validated at 90° and 180° round the shaft and failed only in
//! the periodic tessellator, which produced an open mesh from it (72 boundary
//! edges); at 0° it failed the Euler check. OpenZCAD adopted the unified copy
//! on validation alone and then refused the whole union as open.
//!
//! The merged wall must come back as one face with the conventional band
//! wire — notched bottom, seam up, top rim, seam down — that validates,
//! tessellates closed at several deflections, and keeps the exact volume.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean_with_context};
use remus_operations::heal::unify_faces;
use remus_operations::measure::mass_properties;
use remus_operations::primitives::make_cylinder;
use remus_operations::tessellate::{TriangleMesh, tessellate_solid_with_tolerance};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;

/// Open and non-manifold or inconsistently wound edges after welding
/// positions, the way a display or export consumer reads the mesh.
fn mesh_defects(mesh: &TriangleMesh) -> (usize, usize) {
    let key = |p: Point3| {
        (
            (p.x() * 1e6).round() as i64,
            (p.y() * 1e6).round() as i64,
            (p.z() * 1e6).round() as i64,
        )
    };
    let mut ids: HashMap<(i64, i64, i64), usize> = HashMap::new();
    let remap: Vec<usize> = mesh
        .positions
        .iter()
        .map(|p| {
            let next = ids.len();
            *ids.entry(key(*p)).or_insert(next)
        })
        .collect();
    let mut uses: HashMap<(usize, usize), (usize, usize)> = HashMap::new();
    for tri in mesh.indices.chunks(3) {
        let v = [
            remap[tri[0] as usize],
            remap[tri[1] as usize],
            remap[tri[2] as usize],
        ];
        for k in 0..3 {
            let (a, b) = (v[k], v[(k + 1) % 3]);
            if a == b {
                continue;
            }
            let entry = uses.entry((a.min(b), a.max(b))).or_insert((0, 0));
            if a < b {
                entry.0 += 1;
            } else {
                entry.1 += 1;
            }
        }
    }
    let open = uses.values().filter(|(f, b)| f + b == 1).count();
    let bad = uses
        .values()
        .filter(|(f, b)| f + b > 2 || (f + b == 2 && f != b))
        .count();
    (open, bad)
}

#[test]
fn unified_seam_split_wall_stays_one_closed_band() {
    let exact = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    for deg in [0.0_f64, 90.0, 180.0] {
        let a = deg.to_radians();
        let mut topo = Topology::new();
        let shaft = make_cylinder(&mut topo, 15.0, 60.0).unwrap();
        let boss = make_cylinder(&mut topo, 7.5, 20.0).unwrap();
        transform_solid(
            &mut topo,
            boss,
            &Mat4::translation(15.0 * a.cos(), 15.0 * a.sin(), 0.0),
        )
        .unwrap();
        let fused = boolean_with_context(&mut topo, BooleanOp::Fuse, shaft, boss, &exact)
            .unwrap()
            .solid;
        let volume_before = mass_properties(&topo, fused).unwrap().mass;
        let walls_before = cylinder_faces(&topo, fused);

        let merged = unify_faces(&mut topo, fused).unwrap();
        assert!(merged > 0, "{deg}°: nothing merged");

        let report = validate_solid(&topo, fused).unwrap();
        assert!(report.is_valid(), "{deg}°: {:?}", report.issues);
        let walls_after = cylinder_faces(&topo, fused);
        assert!(
            walls_after < walls_before,
            "{deg}°: the wall pieces did not merge ({walls_before} -> {walls_after} cylindrical faces)"
        );
        let volume_after = mass_properties(&topo, fused).unwrap().mass;
        assert!(
            ((volume_after - volume_before) / volume_before).abs() < 1e-9,
            "{deg}°: unify changed the exact volume {volume_before} -> {volume_after}"
        );
        for deflection in [0.1, 0.02, 0.005] {
            let mesh = tessellate_solid_with_tolerance(&topo, fused, deflection, 0.1).unwrap();
            let (open, bad) = mesh_defects(&mesh);
            assert_eq!(
                (open, bad),
                (0, 0),
                "{deg}° at deflection {deflection}: {open} open, {bad} non-manifold edges"
            );
        }
    }
}

fn cylinder_faces(topo: &Topology, solid: remus_topology::solid::SolidId) -> usize {
    remus_topology::explorer::solid_faces(topo, solid)
        .unwrap()
        .iter()
        .filter(|f| matches!(topo.face(**f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .count()
}
