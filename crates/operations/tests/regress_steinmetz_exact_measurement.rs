//! Measured volume and moments of equal-radius perpendicular cylinders
//! (consumer K10 combination matrix on Remus 2026.1.122, 2026-11-05).
//!
//! The Steinmetz bicylinder of two r = 10 cylinders read 5333.2664 against
//! 16r³/3 = 5333.3333 (1.25e-5 low), and the cross-drill subtract and
//! union were off by the same 0.067. The B-rep was already exact: the
//! sections are exact ellipses and every face area matches its closed form.
//! Only the measurement was wrong. The face integrator sampled each trim
//! edge into a 128-point `(u, v)` polygon and integrated inside it, which
//! costs O(1/N²) on a curved trim.
//!
//! A cylinder face now integrates on its own boundary by Green's theorem
//! (`face_integrator/cylinder_green.rs`). That is exact, because every
//! moment integrand on a cylinder is a cubic in the axial coordinate.
//!
//! Oracles, independent of the kernel: V = 16r³/3. With the axes along Z and
//! X crossing at the origin, each y-slice is a square of half-side
//! √(r² − y²), so I_y = 128r⁵/45, I_x = I_z = 112r⁵/45, the products of
//! inertia vanish, and the centroid is the crossing point. Cut and fuse
//! follow from the cylinder volume πr²h.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::make_cylinder;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const HEIGHT_RATIO: f64 = 6.0;
const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];
const TOL: f64 = 1e-10;

/// Two cylinders of radius `r` and length `HEIGHT_RATIO · r`, along Z and X,
/// centred on the origin, then moved rigidly by `motion`.
fn pair(topo: &mut Topology, r: f64, motion: &Mat4) -> (SolidId, SolidId) {
    let h = HEIGHT_RATIO * r;
    let along_z = make_cylinder(topo, r, h).unwrap();
    transform_solid(
        topo,
        along_z,
        &(*motion * Mat4::translation(0.0, 0.0, -h / 2.0)),
    )
    .unwrap();
    let along_x = make_cylinder(topo, r, h).unwrap();
    let place = *motion * Mat4::translation(-h / 2.0, 0.0, 0.0) * Mat4::rotation_y(PI / 2.0);
    transform_solid(topo, along_x, &place).unwrap();
    (along_z, along_x)
}

fn close(measured: f64, expected: f64, scale: f64, what: &str) {
    assert!(
        (measured - expected).abs() <= TOL * scale,
        "{what}: {measured} vs {expected}"
    );
}

fn motions() -> [(&'static str, Mat4); 2] {
    [
        ("origin", Mat4::identity()),
        (
            "moved",
            Mat4::translation(7.0, -4.0, 11.0) * Mat4::rotation_z(0.61) * Mat4::rotation_x(-0.37),
        ),
    ]
}

#[test]
fn steinmetz_volumes_are_exact_for_every_operation() {
    for r in [6.0, 10.0] {
        for scale in SCALES {
            let r = r * scale;
            let steinmetz = 16.0 * r.powi(3) / 3.0;
            let cylinder = PI * r * r * HEIGHT_RATIO * r;
            for (place, motion) in motions() {
                let motion = if place == "origin" {
                    motion
                } else {
                    // Keep the rigid offset proportional to the model.
                    Mat4::translation(7.0 * scale, -4.0 * scale, 11.0 * scale)
                        * Mat4::rotation_z(0.61)
                        * Mat4::rotation_x(-0.37)
                };
                for (op, expected) in [
                    (BooleanOp::Intersect, steinmetz),
                    (BooleanOp::Cut, cylinder - steinmetz),
                    (BooleanOp::Fuse, 2.0 * cylinder - steinmetz),
                ] {
                    for swap in [false, true] {
                        let what = format!("r {r} {place} {op:?} swap {swap}");
                        let mut topo = Topology::new();
                        let (a, b) = pair(&mut topo, r, &motion);
                        let (x, y) = if swap { (b, a) } else { (a, b) };
                        let solid =
                            boolean(&mut topo, op, x, y).unwrap_or_else(|e| panic!("{what}: {e}"));
                        assert!(validate_solid(&topo, solid).unwrap().is_valid(), "{what}");
                        let mass = mass_properties(&topo, solid).unwrap().mass;
                        close(mass, expected, expected, &format!("{what} mass_properties"));
                        for deflection in [0.1 * r, 0.01 * r] {
                            let volume = solid_volume(&topo, solid, deflection).unwrap();
                            close(volume, expected, expected, &format!("{what} solid_volume"));
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn steinmetz_centroid_and_inertia_are_exact() {
    for scale in SCALES {
        let r = 10.0 * scale;
        let mut topo = Topology::new();
        let (a, b) = pair(&mut topo, r, &Mat4::identity());
        let solid = boolean(&mut topo, BooleanOp::Intersect, a, b).unwrap();
        let props = mass_properties(&topo, solid).unwrap();
        let what = format!("scale {scale}");
        let origin = Point3::new(0.0, 0.0, 0.0);
        assert!(
            (props.center - origin).length() <= TOL * r,
            "{what}: centroid {:?}",
            props.center
        );
        let r5 = r.powi(5);
        let [ixx, iyy, izz, ixy, ixz, iyz] = props.inertia;
        close(ixx, 112.0 * r5 / 45.0, r5, &format!("{what} Ixx"));
        close(iyy, 128.0 * r5 / 45.0, r5, &format!("{what} Iyy"));
        close(izz, 112.0 * r5 / 45.0, r5, &format!("{what} Izz"));
        for (name, value) in [("Ixy", ixy), ("Ixz", ixz), ("Iyz", iyz)] {
            close(value, 0.0, r5, &format!("{what} {name}"));
        }
    }
}
