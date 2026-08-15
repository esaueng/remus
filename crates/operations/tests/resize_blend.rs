//! Exact resize/removal coverage for analytic blend bands.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use brepkit_check::validate::{ValidateOptions, validate_solid};
use brepkit_math::nurbs::surface::NurbsSurface;
use brepkit_math::tolerance::Tolerance;
use brepkit_math::vec::{Point3, Vec3};
use brepkit_operations::blend_ops::fillet_v2;
use brepkit_operations::measure::solid_volume;
use brepkit_operations::primitives::{make_box, make_cylinder};
use brepkit_operations::resize_blend::{resize_blend, resize_blend_failure_code};
use brepkit_topology::Topology;
use brepkit_topology::edge::{EdgeCurve, EdgeId};
use brepkit_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use brepkit_topology::face::{FaceId, FaceSurface};
use brepkit_topology::solid::SolidId;

const DEFLECTION: f64 = 0.01;

fn assert_valid(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    assert!(report.is_valid(), "validation issues: {:?}", report.issues);
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    solid_volume(topo, solid, DEFLECTION).unwrap()
}

fn box_fixture(radius: f64) -> (Topology, SolidId, FaceId) {
    let mut topo = Topology::new();
    let sharp = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edge = solid_edges(&topo, sharp).unwrap()[0];
    let solid = fillet_v2(&mut topo, sharp, &[edge], radius).unwrap().solid;
    let bands: Vec<FaceId> = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Cylinder(cylinder)
                    if Tolerance::new().approx_eq(cylinder.radius(), radius)
            )
        })
        .collect();
    assert_eq!(bands.len(), 1);
    (topo, solid, bands[0])
}

fn cylinder_fixture(radius: f64) -> (Topology, SolidId, FaceId) {
    let mut topo = Topology::new();
    let sharp = make_cylinder(&mut topo, 10.0, 20.0).unwrap();
    let rim: EdgeId = solid_edges(&topo, sharp)
        .unwrap()
        .into_iter()
        .find(|&edge| matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Circle(_)))
        .expect("cylinder rim");
    let solid = fillet_v2(&mut topo, sharp, &[rim], radius).unwrap().solid;
    let bands: Vec<FaceId> = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Torus(torus)
                    if Tolerance::new().approx_eq(torus.minor_radius(), radius)
            )
        })
        .collect();
    assert_eq!(bands.len(), 1);
    (topo, solid, bands[0])
}

fn blend_radius(topo: &Topology, solid: SolidId) -> Option<f64> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find_map(|face| match topo.face(face).unwrap().surface() {
            FaceSurface::Torus(torus) => Some(torus.minor_radius()),
            FaceSurface::Cylinder(cylinder) if cylinder.radius() < 5.0 => Some(cylinder.radius()),
            _ => None,
        })
}

#[test]
fn box_band_grows_shrinks_and_removes_with_monotonic_volume() {
    let (mut grow_topo, grow_input, grow_band) = box_fixture(1.0);
    let before_grow = volume(&grow_topo, grow_input);
    let grown = resize_blend(&mut grow_topo, grow_input, grow_band, 1.0, 2.0)
        .unwrap()
        .solid;
    assert_valid(&grow_topo, grown);
    assert!(Tolerance::new().approx_eq(blend_radius(&grow_topo, grown).unwrap(), 2.0));
    assert!(volume(&grow_topo, grown) < before_grow);

    let (mut shrink_topo, shrink_input, shrink_band) = box_fixture(2.0);
    let before_shrink = volume(&shrink_topo, shrink_input);
    let shrunk = resize_blend(&mut shrink_topo, shrink_input, shrink_band, 2.0, 0.5)
        .unwrap()
        .solid;
    assert_valid(&shrink_topo, shrunk);
    assert!(Tolerance::new().approx_eq(blend_radius(&shrink_topo, shrunk).unwrap(), 0.5));
    assert!(volume(&shrink_topo, shrunk) > before_shrink);

    let (mut remove_topo, remove_input, remove_band) = box_fixture(1.0);
    let removed = resize_blend(&mut remove_topo, remove_input, remove_band, 1.0, 0.0)
        .unwrap()
        .solid;
    assert_valid(&remove_topo, removed);
    assert_eq!(solid_entity_counts(&remove_topo, removed).unwrap().0, 6);
    assert!(Tolerance::new().approx_eq(volume(&remove_topo, removed), 1000.0));
}

#[test]
fn closed_cylinder_rim_grows_shrinks_and_removes() {
    for (old_radius, new_radius) in [(2.0, 3.0), (2.0, 0.75)] {
        let (mut topo, input, band) = cylinder_fixture(old_radius);
        let result = resize_blend(&mut topo, input, band, old_radius, new_radius)
            .unwrap()
            .solid;
        assert_valid(&topo, result);
        assert!(Tolerance::new().approx_eq(blend_radius(&topo, result).unwrap(), new_radius));
    }

    let (mut topo, input, band) = cylinder_fixture(2.0);
    let result = resize_blend(&mut topo, input, band, 2.0, 0.0)
        .unwrap()
        .solid;
    assert_valid(&topo, result);
    assert_eq!(solid_entity_counts(&topo, result).unwrap().0, 3);
    assert!(blend_radius(&topo, result).is_none());
}

#[test]
fn expected_radius_mismatch_refuses_without_mutating_input() {
    let (mut topo, input, band) = box_fixture(1.0);
    let counts = solid_entity_counts(&topo, input).unwrap();
    let before = volume(&topo, input);
    let error = resize_blend(&mut topo, input, band, 1.25, 2.0).unwrap_err();
    assert_eq!(resize_blend_failure_code(&error), "blend-radius-mismatch");
    assert_eq!(solid_entity_counts(&topo, input).unwrap(), counts);
    assert!(Tolerance::new().approx_eq(volume(&topo, input), before));
    assert_valid(&topo, input);
}

#[test]
fn non_band_and_invalid_radius_have_stable_codes() {
    let (mut topo, input, _) = box_fixture(1.0);
    let plane = solid_faces(&topo, input)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Plane { .. }
            )
        })
        .unwrap();
    let error = resize_blend(&mut topo, input, plane, 1.0, 2.0).unwrap_err();
    assert_eq!(resize_blend_failure_code(&error), "blend-band-not-analytic");

    let error = resize_blend(&mut topo, input, plane, 1.0, -1.0).unwrap_err();
    assert_eq!(resize_blend_failure_code(&error), "invalid-input");
}

#[test]
fn ordinary_cylinder_is_not_guessed_to_be_a_blend() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 10.0, 20.0).unwrap();
    let cylinder = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let error = resize_blend(&mut topo, solid, cylinder, 10.0, 8.0).unwrap_err();
    assert_eq!(resize_blend_failure_code(&error), "resize-blend-failed");
}

#[test]
fn oversized_radius_refuses_and_preserves_input() {
    let (mut topo, input, band) = box_fixture(1.0);
    let counts = solid_entity_counts(&topo, input).unwrap();
    let before = volume(&topo, input);
    let error = resize_blend(&mut topo, input, band, 1.0, 50.0).unwrap_err();
    assert_eq!(
        resize_blend_failure_code(&error),
        "radius-too-large",
        "unexpected refusal: {error}"
    );
    assert_eq!(solid_entity_counts(&topo, input).unwrap(), counts);
    assert!(Tolerance::new().approx_eq(volume(&topo, input), before));
}

fn planar_nurbs(anchor: Point3, normal: Vec3) -> NurbsSurface {
    let normal = normal.normalize().unwrap();
    let seed = if normal.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u = normal.cross(seed).normalize().unwrap() * 20.0;
    let v = normal.cross(u).normalize().unwrap() * 20.0;
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![anchor - u - v, anchor - u + v],
            vec![anchor + u - v, anchor + u + v],
        ],
        vec![vec![1.0, 1.0], vec![1.0, 1.0]],
    )
    .unwrap()
}

#[test]
fn band_touching_freeform_support_refuses_without_mutating() {
    let (mut topo, input, band) = box_fixture(1.0);
    for face in solid_faces(&topo, input).unwrap() {
        let FaceSurface::Plane { normal, .. } = topo.face(face).unwrap().surface() else {
            continue;
        };
        let normal = *normal;
        let wire = topo.face(face).unwrap().outer_wire();
        let oriented = topo.wire(wire).unwrap().edges()[0];
        let edge = topo.edge(oriented.edge()).unwrap();
        let anchor = topo.vertex(oriented.oriented_start(edge)).unwrap().point();
        topo.face_mut(face)
            .unwrap()
            .set_surface(FaceSurface::Nurbs(planar_nurbs(anchor, normal)));
    }
    let counts = solid_entity_counts(&topo, input).unwrap();

    let error = resize_blend(&mut topo, input, band, 1.0, 0.5).unwrap_err();
    assert_eq!(resize_blend_failure_code(&error), "band-touches-freeform");
    assert_eq!(solid_entity_counts(&topo, input).unwrap(), counts);
}
