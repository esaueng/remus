//! A stepped cylinder wall whose seam carries vertices between rim levels
//! must tessellate without sliver triangles fanning from those vertices.
//!
//! The fixture is a real modelling result: a box grown by face offsets, fused
//! with an r58 cylinder whose top cap was then offset twice. The wall runs
//! z 45..170, the box top steps its rim at z 53, and the seam edge (u = 0)
//! keeps vertices at z 70 and z 85 — the two former cap heights. #399 seeds
//! interior rows around rim levels, but a boundary vertex sitting on the seam
//! between two levels still fans to the interior row above it across up to
//! ~27 degrees of arc, up to ~49 degrees off the radial normal.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_operations::tessellate::{
    tessellate_solid_grouped_with_tolerance, tessellate_solid_with_tolerance, welded_mesh_quality,
};
use remus_topology::Topology;
use remus_topology::face::FaceSurface;

const DEFLECTION: f64 = 0.006;
const ANGULAR_TOL: f64 = 0.06;

#[test]
fn stepped_rim_wall_with_seam_vertices_has_no_sliver_triangles() {
    let step = include_str!("data/stepped_rim_seam_vertices_wall.step");
    let mut topo = Topology::new();
    let solids = remus_io::step::reader::read_step(step, &mut topo).unwrap();
    assert_eq!(solids.len(), 1);
    let solid = solids[0];
    assert!(
        remus_operations::validate::validate_solid(&topo, solid)
            .unwrap()
            .is_valid()
    );

    let mesh = tessellate_solid_with_tolerance(&topo, solid, DEFLECTION, ANGULAR_TOL).unwrap();
    let quality = welded_mesh_quality(&mesh);
    assert!(quality.is_watertight(), "{quality:?}");

    let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
    let (grouped, offsets) =
        tessellate_solid_grouped_with_tolerance(&topo, solid, DEFLECTION, ANGULAR_TOL).unwrap();
    assert_eq!(offsets.len(), faces.len() + 1);

    let bound = 2.0 * ANGULAR_TOL;
    let mut wall_triangles = 0_usize;
    let mut worst = 0.0_f64;
    let mut violations = Vec::new();
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
            if deviation > bound {
                violations.push(format!("{deviation:.3} rad: [{pa:?}, {pb:?}, {pc:?}]"));
            }
        }
    }
    assert!(
        wall_triangles > 1000,
        "wall tessellated to only {wall_triangles} triangles"
    );
    assert!(
        violations.is_empty(),
        "{} of {wall_triangles} wall triangles exceed {bound:.3} rad off radial (worst {worst:.3}); first: {}",
        violations.len(),
        violations
            .iter()
            .take(8)
            .cloned()
            .collect::<Vec<_>>()
            .join("\n")
    );
}
