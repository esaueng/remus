//! Display-mesh audit shared by the tessellation witnesses (tests).
//!
//! Reads one solid's display mesh the way the acceptance bar does: its
//! directed edge census, triangles wound against their face's outward
//! normal, and the volume it encloses next to the slab its chords may cut
//! off (deflection times mesh area).

#![allow(clippy::unwrap_used)]

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

use super::surface_normal::face_surface_normal_at;
use super::{edge_census, tessellate_solid_grouped_with_tolerance};

/// What [`audit_display_mesh`] read off one mesh.
#[derive(Debug, Clone, Copy)]
pub struct MeshAudit {
    /// Half-edges whose reverse no triangle runs.
    pub boundary_edges: usize,
    /// Edges three or more triangles use.
    pub non_manifold_edges: usize,
    /// Triangles on planar, cylindrical, spherical or toroidal faces whose
    /// winding opposes the face's outward normal at their centroid.
    pub inverted: usize,
    /// The same on conical faces, counted apart: the non-planar CDT can
    /// leave a sliver between two boundary samples and a grid point a few
    /// 1e-8 off their chord in the chart, which the cone's curvature turns
    /// over in space (see `thread::tests`). NURBS faces are not read (no
    /// closed-form normal).
    pub inverted_on_cones: usize,
    /// Signed volume the mesh encloses.
    pub volume: f64,
    /// Mesh surface area.
    pub area: f64,
}

impl MeshAudit {
    /// Closed, edge-manifold, enclosing `expected` to within the chord slab
    /// `deflection · area` (plus a relative 1e-9 of rounding), and with no
    /// inverted triangle off the cones.
    pub fn sound_off_cones(&self, deflection: f64, expected: f64) -> bool {
        self.boundary_edges == 0
            && self.non_manifold_edges == 0
            && self.inverted == 0
            && (self.volume - expected).abs() <= deflection * self.area + 1e-9 * expected.abs()
    }

    /// [`Self::sound_off_cones`], and no inverted triangle on a cone either.
    pub fn sound(&self, deflection: f64, expected: f64) -> bool {
        self.sound_off_cones(deflection, expected) && self.inverted_on_cones == 0
    }
}

/// Audit the display mesh of `solid` at `deflection` (angular tolerance
/// `angular`).
pub fn audit_display_mesh(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
    angular: f64,
) -> MeshAudit {
    let (mesh, offsets) =
        tessellate_solid_grouped_with_tolerance(topo, solid, deflection, angular).unwrap();
    let census = edge_census(&mesh);
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    let corners = |t: usize| [0, 1, 2].map(|k| mesh.positions[mesh.indices[3 * t + k] as usize]);
    let origin = Point3::new(0.0, 0.0, 0.0);
    let (mut inverted, mut inverted_on_cones) = (0, 0);
    for (index, &face_id) in faces.iter().enumerate() {
        let face = topo.face(face_id).unwrap();
        let on_cone = match face.surface() {
            FaceSurface::Plane { .. }
            | FaceSurface::Cylinder(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => false,
            FaceSurface::Cone(_) => true,
            FaceSurface::Nurbs(_) => continue,
        };
        let sign = if face.is_reversed() { -1.0 } else { 1.0 };
        let (start, end) = (offsets[index] as usize / 3, offsets[index + 1] as usize / 3);
        for t in start..end {
            let [a, b, c] = corners(t);
            let winding: Vec3 = (b - a).cross(c - a);
            let centroid = origin + ((a - origin) + (b - origin) + (c - origin)) * (1.0 / 3.0);
            if face_surface_normal_at(face.surface(), centroid)
                .is_some_and(|normal| sign * winding.dot(normal) < 0.0)
            {
                if on_cone {
                    inverted_on_cones += 1;
                } else {
                    inverted += 1;
                }
            }
        }
    }
    let (mut six, mut twice_area) = (0.0, 0.0);
    for t in 0..mesh.indices.len() / 3 {
        let [a, b, c] = corners(t).map(|p| p - origin);
        six += a.dot(b.cross(c));
        twice_area += (b - a).cross(c - a).length();
    }
    MeshAudit {
        boundary_edges: census.boundary_edges,
        non_manifold_edges: census.non_manifold_edges,
        inverted,
        inverted_on_cones,
        volume: six / 6.0,
        area: twice_area / 2.0,
    }
}
