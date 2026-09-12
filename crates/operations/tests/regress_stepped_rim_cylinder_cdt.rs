//! A boolean-stepped cylinder wall must tessellate without chord triangles.
//!
//! Fusing a 30x18x24 box at the origin with an r6 h28 cylinder at the origin
//! leaves a cylindrical wall whose outer wire carries rim circles at three
//! axial levels (z=0, z=24, z=28). The non-planar CDT used to seed a single
//! interior row at mid-v, so the band between the step (z=24) and the top rim
//! triangulated from boundary points alone: long sliver triangles fanned from
//! the notch corners across the free arc, up to ~56 degrees off the true
//! radial normal. Seeding an interior row at every rim level keeps every wall
//! triangle within twice the angular tolerance of its radial normal.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::tessellate::{
    tessellate_solid_grouped_with_tolerance, tessellate_solid_with_tolerance, welded_mesh_quality,
};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const DEFLECTION: f64 = 0.006;
const ANGULAR_TOL: f64 = 0.06;

fn build_fused() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let box_solid = make_box(&mut topo, 30.0, 18.0, 24.0).unwrap();
    let cylinder = make_cylinder(&mut topo, 6.0, 28.0).unwrap();
    let fused = boolean(&mut topo, BooleanOp::Fuse, box_solid, cylinder).unwrap();
    (topo, fused)
}

/// Axial levels of the rim circles on a cylinder face's outer wire.
fn rim_levels(topo: &Topology, face: remus_topology::face::FaceId) -> Vec<f64> {
    let face_data = topo.face(face).unwrap();
    let FaceSurface::Cylinder(cyl) = face_data.surface() else {
        return Vec::new();
    };
    let wire = topo.wire(face_data.outer_wire()).unwrap();
    let mut levels = Vec::new();
    for oriented in wire.edges() {
        if let EdgeCurve::Circle(circle) = topo.edge(oriented.edge()).unwrap().curve() {
            let level = (circle.center() - cyl.origin()).dot(cyl.axis());
            if !levels
                .iter()
                .any(|&existing: &f64| (existing - level).abs() <= 1e-7)
            {
                levels.push(level);
            }
        }
    }
    levels
}

#[test]
fn stepped_rim_cylinder_wall_has_no_chord_triangles() {
    let (topo, fused) = build_fused();
    assert!(
        remus_operations::validate::validate_solid(&topo, fused)
            .unwrap()
            .is_valid()
    );

    // The fixture must actually exercise the stepped-rim path: one cylinder
    // wall with rim circles at three axial levels.
    let faces = remus_topology::explorer::solid_faces(&topo, fused).unwrap();
    let stepped_count = faces
        .iter()
        .filter(|f| rim_levels(&topo, **f).len() > 2)
        .count();
    assert_eq!(
        stepped_count,
        1,
        "expected one stepped-rim wall, levels per cylinder face: {:?}",
        faces
            .iter()
            .map(|f| rim_levels(&topo, *f))
            .collect::<Vec<_>>()
    );

    let mesh = tessellate_solid_with_tolerance(&topo, fused, DEFLECTION, ANGULAR_TOL).unwrap();
    let quality = welded_mesh_quality(&mesh);
    assert!(quality.is_watertight(), "{quality:?}");

    // Box (30*18*24) + full cylinder (pi*36*28) minus the quarter-cylinder
    // of height 24 the box swallows.
    let expected = 30.0 * 18.0 * 24.0 + 792.0 * std::f64::consts::PI;
    let exact = solid_volume(&topo, fused, 0.01).unwrap();
    assert!(
        (exact - expected).abs() / expected < 1e-3,
        "B-rep volume {exact} vs {expected}"
    );

    // Per-face triangles attributed to their owning face, so only the stepped
    // wall's triangles are measured against the cylinder's radial normal.
    let (grouped, offsets) =
        tessellate_solid_grouped_with_tolerance(&topo, fused, DEFLECTION, ANGULAR_TOL).unwrap();
    assert_eq!(offsets.len(), faces.len() + 1);

    let bound = 2.0 * ANGULAR_TOL;
    let mut wall_triangles = 0_usize;
    let mut worst = 0.0_f64;
    for (i, &face) in faces.iter().enumerate() {
        let face_data = topo.face(face).unwrap();
        let FaceSurface::Cylinder(cyl) = face_data.surface() else {
            continue;
        };
        let (start, end) = (offsets[i] as usize, offsets[i + 1] as usize);
        for tri in grouped.indices[start..end].chunks_exact(3) {
            let (pa, pb, pc) = (
                grouped.positions[tri[0] as usize],
                grouped.positions[tri[1] as usize],
                grouped.positions[tri[2] as usize],
            );
            let geo = (pb - pa).cross(pc - pa);
            let area2 = geo.length();
            if area2 < 1e-20 {
                continue;
            }
            let normal = geo * (1.0 / area2);
            let centroid = remus_math::vec::Point3::new(
                (pa.x() + pb.x() + pc.x()) / 3.0,
                (pa.y() + pb.y() + pc.y()) / 3.0,
                (pa.z() + pb.z() + pc.z()) / 3.0,
            );
            let axial = cyl.origin() + cyl.axis() * cyl.axis().dot(centroid - cyl.origin());
            let radial = centroid - axial;
            let radius = radial.length();
            assert!(radius > 1e-9, "wall centroid on the cylinder axis");
            let deviation = normal
                .dot(radial * (1.0 / radius))
                .abs()
                .clamp(-1.0, 1.0)
                .acos();
            wall_triangles += 1;
            worst = worst.max(deviation);
            assert!(
                deviation <= bound,
                "wall triangle normal {deviation:.3} rad off radial (bound {bound:.3}): \
                 [{pa:?}, {pb:?}, {pc:?}]"
            );
        }
    }
    assert!(
        wall_triangles > 1000,
        "stepped wall tessellated to only {wall_triangles} triangles"
    );
    assert!(worst > 0.0, "no wall triangles measured");
}
