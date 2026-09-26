//! Independent mesh oracles for the B19 tessellation mutation tranche.
//!
//! Nothing here calls the mesher's own geometry helpers. Distances and
//! normals come from each analytic surface's implicit equation, and areas and
//! volumes from the triangles themselves, so a mutant that moves, drops or
//! flips triangles cannot also move the yardstick.

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

use super::super::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count,
    tessellate_solid_grouped_with_tolerance,
};

/// One face's triangles, as positions, with its carrier and orientation.
pub(super) struct FaceTris {
    pub surface: FaceSurface,
    pub reversed: bool,
    pub tris: Vec<[Point3; 3]>,
}

/// Tessellate `solid` (the display path's flags) and split the triangles by
/// face. Panics unless the mesh is closed and 2-manifold.
pub(super) fn closed_mesh_by_face(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
) -> (TriangleMesh, Vec<FaceTris>) {
    let (mesh, offsets) = tessellate_solid_grouped_with_tolerance(
        topo,
        solid,
        deflection,
        remus_math::chord::DEFAULT_ANGULAR_TOL,
    )
    .unwrap();
    assert_eq!(
        (boundary_edge_count(&mesh), non_manifold_edge_count(&mesh)),
        (0, 0),
        "mesh at deflection {deflection} is not closed and manifold"
    );
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    let mut out = Vec::with_capacity(faces.len());
    for (i, &face_id) in faces.iter().enumerate() {
        let face = topo.face(face_id).unwrap();
        let tris = mesh.indices[offsets[i] as usize..offsets[i + 1] as usize]
            .chunks_exact(3)
            .map(|t| [0, 1, 2].map(|k| mesh.positions[t[k] as usize]))
            .collect();
        out.push(FaceTris {
            surface: face.surface().clone(),
            reversed: face.is_reversed(),
            tris,
        });
    }
    (mesh, out)
}

/// Divergence-theorem volume of a closed mesh.
pub(super) fn signed_volume(mesh: &TriangleMesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|t| {
            let [a, b, c] = [0, 1, 2].map(|k| {
                let p = mesh.positions[t[k] as usize];
                Vec3::new(p.x(), p.y(), p.z())
            });
            a.dot(b.cross(c))
        })
        .sum::<f64>()
        / 6.0
}

pub(super) fn triangle_area(t: &[Point3; 3]) -> f64 {
    (t[1] - t[0]).cross(t[2] - t[0]).length() / 2.0
}

/// Distance from `p` to the carrier and the carrier's unit normal at the
/// foot, both from the implicit equation. NURBS carriers return `None`.
pub(super) fn surface_distance_and_normal(surface: &FaceSurface, p: Point3) -> Option<(f64, Vec3)> {
    let radial = |w: Vec3, axis: Vec3| {
        let h = w.dot(axis);
        let r = w - axis * h;
        let rho = r.length();
        let dir = if rho > 0.0 {
            r * (1.0 / rho)
        } else {
            Vec3::new(0.0, 0.0, 0.0)
        };
        (h, rho, dir)
    };
    Some(match surface {
        FaceSurface::Plane { normal, d } => {
            let n = normal.normalize().ok()?;
            let o = Vec3::new(p.x(), p.y(), p.z());
            ((o.dot(n) - d).abs(), n)
        }
        FaceSurface::Cylinder(c) => {
            let (_, rho, dir) = radial(p - c.origin(), c.axis());
            ((rho - c.radius()).abs(), dir)
        }
        FaceSurface::Cone(c) => {
            // `half_angle` is measured from the radial plane: the generator
            // is cos(a)·radial + sin(a)·axis, so the surface is
            // ρ·sin(a) = |h|·cos(a) on either nappe.
            let (h, rho, dir) = radial(p - c.apex(), c.axis());
            let (s, co) = c.half_angle().sin_cos();
            let along = c.axis() * h.signum();
            ((rho * s - h.abs() * co).abs(), dir * s - along * co)
        }
        FaceSurface::Sphere(s) => {
            let w = p - s.center();
            let len = w.length();
            ((len - s.radius()).abs(), w * (1.0 / len))
        }
        FaceSurface::Torus(t) => {
            let (h, rho, dir) = radial(p - t.center(), t.z_axis());
            let tube = dir * (rho - t.major_radius()) + t.z_axis() * h;
            let len = tube.length();
            ((len - t.minor_radius()).abs(), tube * (1.0 / len))
        }
        FaceSurface::Nurbs(_) => return None,
    })
}

/// Worst vertex distance to the carrier, and worst chord sag measured at
/// every triangle's centroid and edge midpoints.
pub(super) fn vertex_and_sag(face: &FaceTris) -> (f64, f64) {
    let mut vertex = 0.0_f64;
    let mut sag = 0.0_f64;
    for t in &face.tris {
        for p in t {
            let (d, _) = surface_distance_and_normal(&face.surface, *p).unwrap();
            vertex = vertex.max(d);
        }
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
            let (d, _) = surface_distance_and_normal(&face.surface, q).unwrap();
            sag = sag.max(d);
        }
    }
    (vertex, sag)
}

/// Every non-sliver triangle's geometric normal must point along the
/// carrier's normal on the material's outside: the carrier normal, reversed
/// for a reversed face. Returns the count of triangles that disagree.
pub(super) fn inverted_triangles(face: &FaceTris) -> usize {
    face.tris
        .iter()
        .filter(|t| {
            let n = (t[1] - t[0]).cross(t[2] - t[0]);
            if n.length() <= 1e-14 {
                return false;
            }
            let c = Point3::new(
                (t[0].x() + t[1].x() + t[2].x()) / 3.0,
                (t[0].y() + t[1].y() + t[2].y()) / 3.0,
                (t[0].z() + t[1].z() + t[2].z()) / 3.0,
            );
            let (_, s) = surface_distance_and_normal(&face.surface, c).unwrap();
            let s = if face.reversed { -s } else { s };
            n.dot(s) <= 0.0
        })
        .count()
}
