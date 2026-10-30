//! Regression: resizing a cylinder wall of a revolved part whose fillet
//! rims a STEP writer split into arcs.
//!
//! `walking_stick_foot_body.bin` is an imported revolved part (a 45° cone
//! foot, an R20 band and an R16.5 neck joined by R4 fillets, a blind R12.5
//! bore from z = 9 to the z = 50 top). Its writer split every fillet rim into
//! two half-circle arcs where the neighbouring face's seam meets it, so each
//! full-revolution fillet band has five or six boundary edges instead of
//! four.
//!
//! `rectangular_torus_domain` accepted only four-edge patches, so the ray-cast
//! classifier fell back to a flat boundary polygon for those bands and the
//! horizontal rays never crossed them. A point inside the neck level with the
//! concave fillet (z ≈ 27–30) classified Outside; the resize's new wall,
//! sampled at mid-height z = 29.5, was dropped from the Cut; the result had a
//! free edge and every resize of the bore was refused as exact-only.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;
use std::path::Path;

use remus_algo::FaceClass;
use remus_algo::classifier::classify_point;
use remus_io::arena_io::deserialize_solid;
use remus_math::vec::Point3;
use remus_operations::measure::solid_volume;
use remus_operations::push_pull::resize_cylindrical_face;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const DEFLECTION: f64 = 0.01;

fn load(topo: &mut Topology) -> SolidId {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/walking_stick_foot_body.bin");
    deserialize_solid(&std::fs::read(path).unwrap(), topo).unwrap()
}

fn wall(topo: &Topology, solid: SolidId, radius: f64) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Cylinder(c) if (c.radius() - radius).abs() < 1e-9
            )
        })
        .unwrap_or_else(|| panic!("no cylinder wall of radius {radius}"))
}

#[test]
fn fixture_fillet_rims_are_split() {
    let mut topo = Topology::new();
    let solid = load(&mut topo);
    let tori: Vec<_> = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Torus(_)))
        .collect();
    assert_eq!(tori.len(), 3);
    for face in tori {
        let edges = topo
            .wire(topo.face(face).unwrap().outer_wire())
            .unwrap()
            .edges()
            .len();
        assert!(edges > 4, "torus face {face:?} has {edges} edges");
    }
}

#[test]
fn points_level_with_the_fillets_classify_inside() {
    let mut topo = Topology::new();
    let solid = load(&mut topo);
    // Between the bore (R12.5) and the neck or fillet surface (R ≥ 16.5) at
    // every height from the bore floor to the top.
    for z in [
        10.0, 20.0, 24.0, 26.0, 27.0, 28.0, 29.0, 29.5, 30.0, 31.0, 40.0, 49.0,
    ] {
        for (x, y) in [(-14.0, 0.0), (0.0, 14.0), (9.9, -9.9)] {
            let class = classify_point(&topo, solid, Point3::new(x, y, z)).unwrap();
            assert_eq!(class, FaceClass::Inside, "({x}, {y}, {z})");
        }
    }
}

#[test]
fn bore_resizes_through_the_fillet_band() {
    for new_radius in [10.0, 14.0, 16.0] {
        let mut topo = Topology::new();
        let solid = load(&mut topo);
        let before = solid_volume(&topo, solid, DEFLECTION).unwrap();
        let bore = wall(&topo, solid, 12.5);
        let resized = resize_cylindrical_face(&mut topo, solid, bore, new_radius)
            .unwrap_or_else(|error| panic!("bore -> R{new_radius}: {error}"));
        let shell = topo
            .shell(topo.solid(resized).unwrap().outer_shell())
            .unwrap();
        validate_shell_closed(shell, &topo).unwrap();
        wall(&topo, resized, new_radius);
        // The bore runs from the floor at z = 9 to the top at z = 50.
        let expected = before - PI * (new_radius * new_radius - 12.5 * 12.5) * 41.0;
        let after = solid_volume(&topo, resized, DEFLECTION).unwrap();
        assert!(
            (after - expected).abs() <= expected * 1e-3,
            "R{new_radius}: volume {after}, expected {expected}"
        );
    }
}
