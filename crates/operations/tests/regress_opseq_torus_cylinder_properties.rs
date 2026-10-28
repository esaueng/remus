//! A torus pocket must remove the same region in every tube-angle branch.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::classify::{PointClassification, classify_points};
use remus_operations::measure::{mass_properties, mass_properties_with_options};
use remus_operations::primitives::{make_cylinder, make_sphere, make_torus};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::wire::Wire;

/// Exact overlap area of two discs; none of the kernel's measurement paths
/// participates in this independent horizontal-section oracle.
fn disc_overlap(a: f64, b: f64, distance: f64) -> f64 {
    if distance >= a + b {
        return 0.0;
    }
    if distance <= (a - b).abs() {
        return PI * a.min(b).powi(2);
    }
    let alpha = ((distance * distance + a * a - b * b) / (2.0 * distance * a))
        .clamp(-1.0, 1.0)
        .acos();
    let beta = ((distance * distance + b * b - a * a) / (2.0 * distance * b))
        .clamp(-1.0, 1.0)
        .acos();
    let four_triangle_area_sq =
        (-distance + a + b) * (distance + a - b) * (distance - a + b) * (distance + a + b);
    a * a * alpha + b * b * beta - 0.5 * four_triangle_area_sq.max(0.0).sqrt()
}

fn independent_cut_integrals(panels: usize) -> (f64, f64) {
    assert!(panels.is_multiple_of(2));
    let distance = 3.0_f64.hypot(1.5);
    // z = -0.5 + 2 sin(theta): only 0 <= z <= 1.5 reaches the
    // cylinder. Each section is an annulus minus its overlap with a disc.
    let (lo, hi) = (0.25_f64.asin(), PI / 2.0);
    let section = |theta: f64| {
        let half_width = 2.0 * theta.cos();
        let area_dz = (disc_overlap(5.0 + half_width, 5.5, distance)
            - disc_overlap(5.0 - half_width, 5.5, distance))
            * half_width;
        (area_dz, (-0.5 + 2.0 * theta.sin()) * area_dz)
    };
    let step = (hi - lo) / panels as f64;
    let ends = (section(lo), section(hi));
    let mut integral = (ends.0.0 + ends.1.0, ends.0.1 + ends.1.1);
    for i in 1..panels {
        let weight = if i % 2 == 0 { 2.0 } else { 4.0 };
        let value = section(lo + i as f64 * step);
        integral.0 += weight * value.0;
        integral.1 += weight * value.1;
    }
    let full_volume = 40.0 * PI * PI;
    (
        full_volume - integral.0 * step / 3.0,
        -0.5 * full_volume - integral.1 * step / 3.0,
    )
}

fn independent_cut_volume(panels: usize) -> f64 {
    independent_cut_integrals(panels).0
}

fn placement(scale: f64, placed: bool) -> Mat4 {
    if placed {
        Mat4::translation(10.0 * scale, -7.0 * scale, 3.0 * scale)
            * Mat4::rotation_z(0.37)
            * Mat4::rotation_x(0.41)
    } else {
        Mat4::identity()
    }
}

fn retained_cut(topo: &mut Topology, scale: f64, placed: bool) -> SolidId {
    let torus = make_torus(topo, 5.0, 2.0, 16).unwrap();
    let cylinder = make_cylinder(topo, 5.5, 7.5).unwrap();
    transform_solid(topo, torus, &Mat4::translation(3.0, 1.5, -0.5)).unwrap();
    let operands = remus_io::arena_io::serialize_solids(topo, &[torus, cylinder]).unwrap();
    let result = boolean_with_context(
        topo,
        BooleanOp::Cut,
        torus,
        cylinder,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap();
    assert_eq!(result.quality, BooleanQuality::Exact);
    // Qualify measurement under similarity transforms of the retained exact
    // body. Direct booleans built at 1e-3/1e3 scales already refuse on the
    // frozen source; broadening that independent GFA family is outside this fix.
    if scale.to_bits() != 1.0_f64.to_bits() || placed {
        let matrix = placement(scale, placed) * Mat4::scale(scale, scale, scale);
        transform_solid(topo, result.solid, &matrix).unwrap();
    }
    assert_eq!(
        remus_io::arena_io::serialize_solids(topo, &[torus, cylinder]).unwrap(),
        operands
    );
    let validation = remus_operations::validate::validate_solid(topo, result.solid).unwrap();
    assert!(validation.is_valid(), "{validation:?}");
    let mut carriers = [0; 3];
    for face in remus_topology::explorer::solid_faces(topo, result.solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => carriers[0] += 1,
            FaceSurface::Cylinder(_) => carriers[1] += 1,
            FaceSurface::Torus(_) => carriers[2] += 1,
            other => panic!("unexpected retained carrier: {other:?}"),
        }
    }
    assert_eq!(carriers, [1, 2, 1]);
    result.solid
}

#[test]
fn independent_annulus_disc_sections_converge() {
    let fine = independent_cut_volume(2048);
    assert!((fine - 336.286_642_360_514_2).abs() < 1e-10);
    assert!((independent_cut_volume(512) - fine).abs() < 1e-8);
    assert!((independent_cut_volume(1024) - fine).abs() < 1e-9);
}

#[test]
fn retained_torus_cylinder_mass_removes_its_periodic_pocket() {
    let mut topo = Topology::new();
    let solid = retained_cut(&mut topo, 1.0, false);
    let expected = independent_cut_volume(2048);
    let measured = mass_properties(&topo, solid).unwrap().mass;
    assert!(
        (measured - expected).abs() < expected * 1e-5,
        "periodic torus pocket: mass={measured}, independent volume={expected}"
    );
    // More Gauss points cannot recover an omitted hole. Every order must now
    // integrate the resolved pocket, with unchanged trim chording as residual.
    for gauss_order in [3, 5, 12, 20] {
        let options = remus_check::properties::PropertiesOptions {
            gauss_order,
            ..Default::default()
        };
        let value = mass_properties_with_options(&topo, solid, &options)
            .unwrap()
            .mass;
        assert!(
            (value - expected).abs() < expected * 1e-5,
            "order={gauss_order}, mass={value}"
        );
    }
}

#[test]
fn periodic_pocket_properties_survive_placement_and_scale() {
    let (volume, first_z) = independent_cut_integrals(2048);
    for scale in [1.0, 0.001, 1000.0] {
        for placed in [false, true] {
            let mut topo = Topology::new();
            let solid = retained_cut(&mut topo, scale, placed);
            let before = remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap();
            let measured = mass_properties(&topo, solid).unwrap();
            assert!(
                (measured.mass / scale.powi(3) - volume).abs() < volume * 1e-5,
                "scale={scale}, placed={placed}, mass={}",
                measured.mass
            );
            let center = placement(scale, placed)
                .inverse()
                .unwrap()
                .mul_point(measured.center);
            assert!((center.z() / scale - first_z / volume).abs() < 1e-5);

            let mut points = Vec::new();
            let mut expected = Vec::new();
            for x in [-3.7_f64, -0.7, 2.3, 5.3, 8.3, 10.3] {
                for y in [-4.7_f64, -1.7, 1.3, 4.3, 7.3] {
                    for z in [-2.3_f64, -1.1, 0.3, 1.1, 2.3] {
                        let in_torus =
                            ((x - 3.0).hypot(y - 1.5) - 5.0).powi(2) + (z + 0.5).powi(2) < 4.0;
                        let in_cutter = x * x + y * y < 5.5_f64.powi(2) && z > 0.0 && z < 7.5;
                        points.push(placement(scale, placed).mul_point(Point3::new(
                            x * scale,
                            y * scale,
                            z * scale,
                        )));
                        expected.push(if in_torus && !in_cutter {
                            PointClassification::Inside
                        } else {
                            PointClassification::Outside
                        });
                    }
                }
            }
            assert_eq!(
                classify_points(&topo, solid, &points, 0.1, 1e-7).unwrap(),
                expected
            );
            assert_eq!(
                remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap(),
                before
            );
        }
    }
}

#[test]
fn cyclic_torus_seam_starts_preserve_the_pocket_measurement() {
    let mut topo = Topology::new();
    let solid = retained_cut(&mut topo, 1.0, false);
    let shell = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap()
        .clone();
    let face_id = shell
        .faces()
        .iter()
        .copied()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .unwrap();
    let face = topo.face(face_id).unwrap().clone();
    let uses = topo.wire(face.outer_wire()).unwrap().edges().to_vec();
    assert_eq!(uses.len(), 16);
    let expected = independent_cut_volume(2048);
    for start in 0..uses.len() {
        let mut rotated = uses.clone();
        rotated.rotate_left(start);
        let outer = topo.add_wire(Wire::new(rotated, true).unwrap());
        // Construct new valid wire/face authority, rather than altering an
        // existing wire behind its stored coedges or persistent references.
        let mut shifted = Face::new(outer, face.inner_wires().to_vec(), face.surface().clone());
        shifted.set_reversed(face.is_reversed());
        let shifted = topo.add_face(shifted);
        let faces = shell
            .faces()
            .iter()
            .map(|&f| if f == face_id { shifted } else { f })
            .collect();
        let new_shell = topo.add_shell(Shell::new(faces).unwrap());
        let variant = topo.add_solid(Solid::new(new_shell, vec![]));
        assert!(
            remus_operations::validate::validate_solid(&topo, variant)
                .unwrap()
                .is_valid()
        );
        let measured = mass_properties(&topo, variant).unwrap().mass;
        assert!(
            (measured - expected).abs() < expected * 1e-5,
            "start={start}, mass={measured}"
        );
    }
}

#[test]
fn cylinder_and_sphere_closed_forms_remain_unchanged() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 5.5, 7.5).unwrap();
    let sphere = make_sphere(&mut topo, 2.0, 16).unwrap();
    for (solid, volume) in [
        (cylinder, PI * 5.5_f64.powi(2) * 7.5),
        (sphere, 4.0 * PI * 8.0 / 3.0),
    ] {
        let measured = mass_properties(&topo, solid).unwrap().mass;
        assert!((measured - volume).abs() < volume * 1e-10);
    }
}
