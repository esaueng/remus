//! A thread flank is the twisted extrusion of a transverse spiral: its chart
//! runs `u` up the helix and `v` along a spiral at constant height (127° for
//! an ISO M12 flank), two directions only a few degrees apart in space.
//! Gridding that chart axis by axis gave every flank a sheared 100 × 70 grid
//! whose cells split into slivers: 1.38 M triangles on an M12 × 28 mm thread
//! at display deflection, with facet normals 6–13° off the surface they
//! sample, which a viewport that shades from facets renders as grain.

use remus_math::vec::Vec3;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;

use crate::primitives::make_cylinder;
use crate::thread::{CylinderThreadOptions, ThreadEnd, ThreadForm, ThreadHand, cut_thread};

use super::super::{is_watertight, tessellate_solid_grouped_with_tolerance};

/// Facet-normal deviation from the exact vertex normals over the interior
/// triangles of a solid's NURBS faces, plus triangle counts.
struct FlankShading {
    triangles: usize,
    nurbs_triangles: usize,
    /// Degrees, sorted ascending.
    deviations: Vec<f64>,
    watertight: bool,
}

impl FlankShading {
    fn quantile(&self, q: f64) -> f64 {
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss
        )]
        let index = ((self.deviations.len() - 1) as f64 * q).round() as usize;
        self.deviations[index]
    }
}

/// The ISO M12 × 1.75 basic profile (30° flanks).
fn iso_m12() -> ThreadForm {
    ThreadForm {
        pitch: 1.75,
        depth: 1.0734,
        crest_width: 0.21875,
        upper_flank_angle: 30_f64.to_radians(),
        lower_flank_angle: 30_f64.to_radians(),
        starts: 1,
        hand: ThreadHand::Right,
    }
}

/// The printable M12 × 1.75 profile a CAD front end offers for upright
/// printing: 45° flanks, depth and crest a quarter pitch.
fn printable_m12() -> ThreadForm {
    ThreadForm {
        pitch: 1.75,
        depth: 1.75 / 4.0,
        crest_width: 1.75 / 4.0,
        upper_flank_angle: 45_f64.to_radians(),
        lower_flank_angle: 45_f64.to_radians(),
        starts: 1,
        hand: ThreadHand::Right,
    }
}

/// `form` cut `length` mm along a 6 mm boss to a 12 mm major diameter.
fn flank_shading(form: &ThreadForm, length: f64, ends: ThreadEnd) -> FlankShading {
    // The display deflection an M12 × 28 mm body gets (2e-4 of its extent)
    // and the display angular tolerance. A shorter thread keeps the test
    // quick: every flank face is the same half turn either way.
    let (deflection, angular) = (28.0 * 2e-4, 0.06);
    let mut topo = Topology::new();
    let body = make_cylinder(&mut topo, 6.0, length).unwrap();
    let wall = remus_topology::explorer::solid_faces(&topo, body)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let options = CylinderThreadOptions {
        major_diameter: Some(12.0),
        length: Some(length),
        start: ends,
        end: ends,
        ..CylinderThreadOptions::default()
    };
    let solid = cut_thread(&mut topo, body, wall, form, &options)
        .unwrap()
        .solid;
    let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
    let (mesh, offsets) =
        tessellate_solid_grouped_with_tolerance(&topo, solid, deflection, angular).unwrap();

    // A vertex on a face boundary carries one neighbour's normal; keep only
    // triangles whose three vertices belong to a single face.
    let mut owner = vec![usize::MAX; mesh.positions.len()];
    let mut shared = vec![false; mesh.positions.len()];
    for face in 0..faces.len() {
        for &v in &mesh.indices[offsets[face] as usize..offsets[face + 1] as usize] {
            let v = v as usize;
            if owner[v] == usize::MAX {
                owner[v] = face;
            } else if owner[v] != face {
                shared[v] = true;
            }
        }
    }

    let mut deviations = Vec::new();
    let mut nurbs_triangles = 0;
    for (face, &face_id) in faces.iter().enumerate() {
        if !matches!(topo.face(face_id).unwrap().surface(), FaceSurface::Nurbs(_)) {
            continue;
        }
        let range = offsets[face] as usize..offsets[face + 1] as usize;
        for tri in mesh.indices[range].chunks_exact(3) {
            nurbs_triangles += 1;
            if tri.iter().any(|&v| shared[v as usize]) {
                continue;
            }
            let [a, b, c] = [0, 1, 2].map(|i| mesh.positions[tri[i] as usize]);
            let facet = (b - a).cross(c - a);
            let mean = tri.iter().fold(Vec3::new(0.0, 0.0, 0.0), |sum, &v| {
                sum + mesh.normals[v as usize]
            });
            let (fl, ml) = (facet.length(), mean.length());
            if fl <= 0.0 || ml <= 0.0 {
                continue;
            }
            let cos = (facet.dot(mean) / (fl * ml)).abs().clamp(0.0, 1.0);
            deviations.push(cos.acos().to_degrees());
        }
    }
    deviations.sort_by(f64::total_cmp);
    FlankShading {
        triangles: mesh.indices.len() / 3,
        nurbs_triangles,
        deviations,
        watertight: is_watertight(&mesh),
    }
}

fn assert_facets_follow_flanks(shading: &FlankShading, max_nurbs_triangles: usize) {
    let (p50, p90, max) = (
        shading.quantile(0.5),
        shading.quantile(0.9),
        shading.quantile(1.0),
    );
    let report = format!(
        "{} triangles ({} on flanks), facet deviation p50 {p50:.3}° p90 {p90:.3}° max {max:.3}°",
        shading.triangles, shading.nurbs_triangles
    );
    assert!(shading.watertight, "{report}");
    assert!(!shading.deviations.is_empty(), "{report}");
    // A facet whose vertices sit on the surface tilts from it by about the
    // turning across the facet: under 0.5° at this deflection. The sheared
    // grid measured p90 12.4° and max 13.6°.
    assert!(p90 < 1.0, "{report}");
    assert!(max < 3.0, "{report}");
    // The sheared grid spent 223 392 flank triangles on the ISO four turns;
    // the metric chart needs about a tenth.
    assert!(shading.nurbs_triangles <= max_nurbs_triangles, "{report}");
}

#[test]
fn iso_thread_flank_facets_follow_the_surface_at_display_deflection() {
    let shading = flank_shading(&iso_m12(), 7.0, ThreadEnd::Plain);
    assert_facets_follow_flanks(&shading, 40_000);
}

#[test]
fn printable_chamfered_thread_flank_facets_follow_the_surface_at_display_deflection() {
    let chamfer = ThreadEnd::Chamfer {
        angle: 45_f64.to_radians(),
    };
    let shading = flank_shading(&printable_m12(), 7.0, chamfer);
    assert_facets_follow_flanks(&shading, 40_000);
}
