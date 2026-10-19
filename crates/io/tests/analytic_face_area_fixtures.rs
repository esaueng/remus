//! The boundary-reduced area of analytic curved faces against the trimmed
//! Gauss quadrature on real imported bodies: every cylinder, cone, sphere and
//! torus face of each fixture either reduces, agreeing with the quadrature to
//! its trim chording, or falls back to the quadrature bit for bit.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout
)]

use remus_check::properties::face_integrator::{analytic_face_area, integrate_face_area};
use remus_operations::measure;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;

/// Relative agreement the order-8 trimmed quadrature reaches against the
/// reduction: the chord error of its 128-sample trims.
const QUADRATURE_AGREEMENT: f64 = 2e-4;

#[derive(Debug, Default)]
struct Census {
    reduced: [usize; 4],
    fallback: [usize; 4],
    max_relative_difference: f64,
}

fn kind(surface: &FaceSurface) -> Option<usize> {
    match surface {
        FaceSurface::Cylinder(_) => Some(0),
        FaceSurface::Cone(_) => Some(1),
        FaceSurface::Sphere(_) => Some(2),
        FaceSurface::Torus(_) => Some(3),
        FaceSurface::Plane { .. } | FaceSurface::Nurbs(_) => None,
    }
}

fn census(step: &str) -> Census {
    let mut topo = Topology::new();
    let solids = remus_io::step::reader::read_step(step, &mut topo).unwrap();
    let mut census = Census::default();
    for solid in solids {
        for face in solid_faces(&topo, solid).unwrap() {
            let Some(kind) = kind(topo.face(face).unwrap().surface()) else {
                continue;
            };
            let quadrature = integrate_face_area(&topo, face, 8).unwrap();
            let area = analytic_face_area(&topo, face, 8).unwrap();
            assert_eq!(
                measure::face_area(&topo, face, 0.1).unwrap().to_bits(),
                area.abs().to_bits()
            );
            if area.to_bits() == quadrature.to_bits() {
                census.fallback[kind] += 1;
                continue;
            }
            census.reduced[kind] += 1;
            let relative = (area - quadrature.abs()).abs() / area;
            assert!(
                area > 0.0 && relative <= QUADRATURE_AGREEMENT,
                "face {}: reduced {area} vs quadrature {quadrature}",
                face.index()
            );
            census.max_relative_difference = census.max_relative_difference.max(relative);
        }
    }
    census
}

macro_rules! fixture {
    ($name:literal) => {
        include_str!(concat!("data/", $name, ".step"))
    };
}

#[test]
fn hammer_holder_fillets_reduce() {
    let c = census(fixture!("shapr3d_hammer_holder"));
    println!("hammer holder (cyl, cone, sphere, torus): {c:?}");
    // Every analytic face of the hammer holder qualifies.
    assert_eq!(c.fallback, [0; 4], "{c:?}");
    assert!(c.reduced.iter().sum::<usize>() > 40, "{c:?}");
}

#[test]
fn fixture_corpus_agrees_with_the_quadrature() {
    // (fixture, faces that reduce per (cylinder, cone, sphere, torus), and
    // faces that keep the quadrature).
    for (name, step, reduced, fallback) in [
        (
            "analytic fillet plate",
            fixture!("openzcad_e_analytic_fillet_plate"),
            [4, 0, 0, 0],
            [0; 4],
        ),
        (
            "bored plate",
            fixture!("openzcad_a_export_bored_plate"),
            [1, 0, 0, 0],
            [0; 4],
        ),
        (
            "walking stick foot",
            fixture!("shapr3d_walking_stick_foot"),
            [3, 2, 0, 3],
            [0; 4],
        ),
        (
            "partial cylinder",
            fixture!("jolly_fox_partial_cylinder"),
            [1, 0, 0, 0],
            [0; 4],
        ),
        (
            "cone frustum",
            fixture!("conical_surface_base_radius_frustum"),
            [0, 2, 0, 0],
            [0; 4],
        ),
        (
            "period winding torus",
            fixture!("mambo_b12_period_winding_torus"),
            [0, 0, 0, 1],
            [0; 4],
        ),
        (
            "wide sphere cap",
            fixture!("wide_sphere_cap"),
            [0, 0, 1, 0],
            [0; 4],
        ),
        // A doubled great-circle seam: the quadrature frames the face by the
        // seam and reads the small cap, which the reduction does not mirror.
        (
            "wide sphere cap with seam",
            fixture!("wide_sphere_cap_with_seam"),
            [0; 4],
            [0, 0, 1, 0],
        ),
        (
            "o-ring nested holes",
            fixture!("oring_nested_holes"),
            [24, 0, 0, 0],
            [0; 4],
        ),
        (
            "lip fuse body",
            fixture!("lipfuse_body"),
            [8, 0, 0, 0],
            [0; 4],
        ),
    ] {
        let c = census(step);
        println!("{name} (cyl, cone, sphere, torus): {c:?}");
        assert_eq!((c.reduced, c.fallback), (reduced, fallback), "{name}");
    }
}
