//! A STEP cone wall whose `CONICAL_SURFACE` axis points away from its
//! material (the face below the carrier's apex) meshes on its own wall.
//!
//! The half frustum (bottom radius `2s`, top radius `s`, height `2s`, an
//! inner bore of radius `s/2`, `y ≥ 0`) has its wall written on the
//! reversed-axis carrier; the STEP writer states that carrier as it is and
//! the reader rebuilds it, so the imported wall lies on the `v < 0` nappe
//! like a foreign file's would. It meshed as its mirror image across the
//! axis (1.23 enclosed for 6.54 at scale 1). Oracles independent of
//! `ConicalSurface`: the physical wall `r = 2s − z/2` in the half space
//! `y ≥ 0`, and the volume `solid_volume` reads.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::PI;

use remus_io::step::reader::read_step;
use remus_io::step::writer::write_step;
use remus_math::vec::{Point3, Vec3};
use remus_operations::measure::solid_volume;
use remus_operations::revolve::revolve;
use remus_operations::tessellate::{edge_census, tessellate_solid_grouped_with_tolerance};
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceSurface};
use remus_topology::solid::SolidId;

/// The half frustum, its cone wall on the reversed-axis carrier, through a
/// STEP round trip.
fn imported_half_frustum(s: f64) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let profile = make_polygon_wire(
        &mut topo,
        &[
            Point3::new(0.5 * s, 0.0, 0.0),
            Point3::new(2.0 * s, 0.0, 0.0),
            Point3::new(s, 0.0, 2.0 * s),
            Point3::new(0.5 * s, 0.0, 2.0 * s),
        ],
        1e-7,
    )
    .unwrap();
    let face = topo.add_face(Face::new(
        profile,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, -1.0, 0.0),
            d: 0.0,
        },
    ));
    let solid = revolve(
        &mut topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        PI,
    )
    .unwrap();
    for wall in solid_faces(&topo, solid).unwrap() {
        if let FaceSurface::Cone(cone) = topo.face(wall).unwrap().surface().clone() {
            topo.face_mut(wall)
                .unwrap()
                .set_surface(FaceSurface::Cone(cone.with_reversed_axis()));
        }
    }
    let step = write_step(&topo, &[solid]).unwrap();
    let mut imported = Topology::new();
    let solids = read_step(&step, &mut imported).unwrap();
    assert_eq!(solids.len(), 1);
    (imported, solids[0])
}

#[test]
fn an_imported_lower_nappe_cone_wall_meshes_on_its_own_wall() {
    let mut failures = Vec::new();
    for s in [1e-2, 1.0, 1e2] {
        let (topo, solid) = imported_half_frustum(s);
        let faces = solid_faces(&topo, solid).unwrap();
        let walls: Vec<_> = faces
            .iter()
            .map(|&face| match topo.face(face).unwrap().surface() {
                FaceSurface::Cone(cone) => {
                    let probe = Point3::new(1.5 * s, 0.0, s);
                    assert!(cone.project_point(probe).1 < 0.0, "below the apex");
                    true
                }
                _ => false,
            })
            .collect();
        assert!(walls.iter().any(|&wall| wall), "a cone wall");
        let expected = solid_volume(&topo, solid, 1e-3 * s).unwrap();
        for deflection in [1e-2 * s, 1e-3 * s] {
            let label = format!("scale {s}, deflection {deflection}");
            let (mesh, offsets) =
                tessellate_solid_grouped_with_tolerance(&topo, solid, deflection, 0.5).unwrap();
            let census = edge_census(&mesh);
            if census.boundary_edges != 0 || census.non_manifold_edges != 0 {
                failures.push(format!("{label}: {census:?}"));
            }
            let origin = Point3::new(0.0, 0.0, 0.0);
            let (mut six, mut twice_area) = (0.0, 0.0);
            for tri in mesh.indices.chunks_exact(3) {
                let [a, b, c] = [0, 1, 2].map(|k| mesh.positions[tri[k] as usize] - origin);
                six += a.dot(b.cross(c));
                twice_area += (b - a).cross(c - a).length();
            }
            let (volume, area) = (six / 6.0, twice_area / 2.0);
            // Below 1e-2·s at scale 1e2 a cone's interior grid is sized for
            // radius 1 on either nappe (`estimate_surface_radius`) and keeps
            // a sparse middle row: that case is not held to the chord slab.
            let sparse_grid = s > 1.0 && deflection < 1e-2 * s;
            if !sparse_grid && (volume - expected).abs() > deflection.mul_add(area, 1e-9 * expected)
            {
                failures.push(format!("{label}: mesh volume {volume} vs {expected}"));
            }
            for (index, &wall) in walls.iter().enumerate() {
                if !wall {
                    continue;
                }
                let range = offsets[index] as usize..offsets[index + 1] as usize;
                for &vertex in &mesh.indices[range] {
                    let p = mesh.positions[vertex as usize];
                    let off = (p.x().hypot(p.y()) - (2.0 * s - p.z() / 2.0)).abs();
                    if off > 1e-9 * s || p.y() < -1e-9 * s {
                        failures.push(format!("{label}: wall vertex {p:?} off the wall"));
                        break;
                    }
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
