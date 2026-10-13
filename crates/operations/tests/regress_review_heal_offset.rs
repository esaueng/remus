//! Healing, placed torus offset and failed-operation atomicity regressions.
#![allow(clippy::unwrap_used, clippy::panic)]
use remus_math::{
    mat::Mat4,
    vec::{Point3, Vec3},
};
use remus_operations::{extrude, heal, measure, offset_face, offset_v2, primitives, transform};
use remus_topology::{Topology, face::FaceSurface};

#[test]
fn rotated_torus_face_offset_preserves_axis() {
    let mut t = Topology::new();
    let s = primitives::make_torus(&mut t, 5., 1., 16).unwrap();
    transform::transform_solid(&mut t, s, &Mat4::rotation_x(std::f64::consts::FRAC_PI_2)).unwrap();
    let f = remus_topology::explorer::solid_faces(&t, s)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(t.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .unwrap();
    let old_carrier = match t.face(f).unwrap().surface() {
        FaceSurface::Torus(c) => c.clone(),
        _ => panic!("expected torus"),
    };
    let old_axis = match t.face(f).unwrap().surface() {
        FaceSurface::Torus(c) => c.z_axis(),
        _ => unreachable!(),
    };
    let out = offset_face::offset_face_with_quality(&mut t, f, 0.2, None).unwrap();
    let new_axis = match t.face(out.face).unwrap().surface() {
        FaceSurface::Torus(c) => c.z_axis(),
        _ => unreachable!(),
    };
    assert!(
        old_axis.dot(new_axis) > 1. - 1e-10,
        "old axis {:?}, new axis {:?}, quality {:?}",
        old_axis,
        new_axis,
        out.quality
    );
    let carrier = match t.face(out.face).unwrap().surface() {
        FaceSurface::Torus(c) => c,
        _ => panic!("expected torus"),
    };
    for (u, v) in [(0., 0.), (0.3, 1.7), (2.4, 4.1)] {
        let source = old_carrier.evaluate(u, v);
        let expected = source + old_carrier.normal(u, v) * 0.2;
        assert!((carrier.evaluate(u, v) - expected).length() < 1e-10);
    }
}

#[test]
fn valid_concave_solid_healing_keeps_volume() {
    let mut t = Topology::new();
    let pts = [
        (0., 0.),
        (10., 0.),
        (10., 10.),
        (7., 10.),
        (7., 3.),
        (3., 3.),
        (3., 10.),
        (0., 10.),
    ]
    .map(|(x, y)| Point3::new(x, y, 0.));
    let f = remus_topology::builder::make_planar_face(&mut t, &pts, 1e-7).unwrap();
    let s = extrude::extrude(&mut t, f, Vec3::new(0., 0., 1.), 2.).unwrap();
    let before = measure::solid_volume(&t, s, 0.1).unwrap();
    let l3 = remus_operations::validate::validate_solid(&t, s).unwrap();
    assert!(l3.is_valid());
    assert!((before - 144.).abs() < 1e-8);
    let report = heal::heal_solid(&mut t, s, 1e-7);
    let after = measure::solid_volume(&t, s, 0.1).unwrap();

    assert!(
        report.is_ok() && (after - before).abs() < 1e-8,
        "healing a valid U-prism changed volume from {before} to {after}, report {report:?}"
    );
}

#[test]
fn failed_native_offset_does_not_leave_live_entities() {
    let mut t = Topology::new();
    let s = primitives::make_box(&mut t, 10., 10., 10.).unwrap();
    let before = (t.num_solids(), t.num_faces(), t.num_vertices());
    let out = offset_v2::offset_solid_v2(&mut t, s, -6.);
    let after = (t.num_solids(), t.num_faces(), t.num_vertices());
    assert!(out.is_err(), "expected collapse refusal, got {out:?}");
    assert_eq!(
        before, after,
        "failed offset leaked live topology, error {out:?}"
    );
}

#[test]
fn healing_preserves_cavity_material_sense_and_repairs_only_a_wrong_carrier() {
    let mut t = Topology::new();
    let outer = primitives::make_box(&mut t, 4., 4., 4.).unwrap();
    let inner = primitives::make_box(&mut t, 2., 2., 2.).unwrap();
    transform::transform_solid(&mut t, inner, &Mat4::translation(1., 1., 1.)).unwrap();
    for face in remus_topology::explorer::solid_faces(&t, inner).unwrap() {
        t.face_mut(face).unwrap().set_reversed(true);
    }
    let cavity = t.add_solid(remus_topology::solid::Solid::new(
        t.solid(outer).unwrap().outer_shell(),
        vec![t.solid(inner).unwrap().outer_shell()],
    ));
    let report = heal::heal_solid(&mut t, cavity, 1e-7).unwrap();
    assert_eq!(report.orientations_fixed, 0);
    assert!((measure::solid_volume(&t, cavity, 0.1).unwrap() - 56.).abs() < 1e-8);
    let face = remus_topology::explorer::solid_faces(&t, cavity).unwrap()[0];
    let surface = t.face(face).unwrap().surface().clone();
    if let FaceSurface::Plane { normal, d } = surface {
        t.face_mut(face).unwrap().set_surface(FaceSurface::Plane {
            normal: -normal,
            d: -d,
        });
    } else {
        panic!("box face must be planar");
    }
    assert_eq!(heal::fix_face_orientations(&mut t, cavity).unwrap(), 1);
    assert_eq!(heal::fix_face_orientations(&mut t, cavity).unwrap(), 0);
    assert!((measure::solid_volume(&t, cavity, 0.1).unwrap() - 56.).abs() < 1e-8);
}

#[test]
fn failed_offset_variants_preserve_live_topology_and_input_volume() {
    for arc in [false, true] {
        let mut t = Topology::new();
        let solid = primitives::make_box(&mut t, 10., 10., 10.).unwrap();
        let counts = |t: &Topology| {
            (
                t.num_vertices(),
                t.num_edges(),
                t.num_wires(),
                t.num_faces(),
                t.num_shells(),
                t.num_solids(),
            )
        };
        let before = counts(&t);
        let result = if arc {
            offset_v2::offset_solid_arc_v2(&mut t, solid, -6.)
        } else {
            offset_v2::shell_v2(&mut t, solid, -6., &[])
        };
        assert!(result.is_err(), "expected collapse refusal: {result:?}");
        assert_eq!(counts(&t), before);
        assert!((measure::solid_volume(&t, solid, 0.1).unwrap() - 1000.).abs() < 1e-8);
    }
}
