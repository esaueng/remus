//! A primitive sphere against a box whose walls cross the upper hemisphere
//! and whose lid sits below the pole: the upper hemisphere splits into a
//! collar (inside the walls, holed by the lid's latitude circle), the lid's
//! cap and four wall caps — the face splitter's box–sphere collar route
//! (B19 face-splitter tranche).
//!
//! Oracles: exact success, validity, the analytic face census, ray-cast
//! probes, and the volume against an independent numerical integral of the
//! sphere ∩ box column heights (cut checked by `sphere − intersect`).
//!
//! Two open defects ride here as ready-repros:
//! - B61: the cut drops the lid-cap lump (the sphere above the lid), so it
//!   returns only the four wall caps — a silent wrong volume on `main`.
//! - B62: with the walls close under the lid (`a = 0.75·r`) the collar's
//!   classification sample, the lid latitude nudged a fixed amount toward
//!   the equator, lands beyond a wall, and the exact boolean is refused.
//!   Bounding the nudge by the first wall arc on its meridian fixes the
//!   intersect but turns the cut from refused into B61's wrong result, so it
//!   waits on B61.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_sphere};
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// Sphere of radius `r` at the origin and the box `|x|, |y| < a`,
/// `-r - 5 < z < c`.
fn operands(r: f64, a: f64, c: f64) -> (Topology, SolidId, SolidId) {
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, r, 64).unwrap();
    let b = make_box(&mut topo, 2.0 * a, 2.0 * a, r + 5.0 + c).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        b,
        &Mat4::translation(-a, -a, -r - 5.0),
    )
    .unwrap();
    (topo, sphere, b)
}

/// `∬_{|x|,|y|<a, x²+y²<r²} (min(c, √(r²−x²−y²)) + √(r²−x²−y²)) dx dy` by the
/// midpoint rule: the volume of sphere ∩ box, independent of the kernel.
fn reference_volume(r: f64, a: f64, c: f64) -> f64 {
    let n = 1200;
    let h = 2.0 * a / f64::from(n);
    let mut sum = 0.0;
    for i in 0..n {
        let x = h.mul_add(f64::from(i) + 0.5, -a);
        for j in 0..n {
            let y = h.mul_add(f64::from(j) + 0.5, -a);
            let q = r.mul_add(r, -x.mul_add(x, y * y));
            if q > 0.0 {
                let t = q.sqrt();
                sum += t.min(c) + t;
            }
        }
    }
    sum * h * h
}

/// Box half-width `frac·r`, lid halfway between the walls' arc apex and the
/// pole (so its latitude circle, of radius < a, stays inside the walls).
fn lidded(r: f64, frac: f64) -> (f64, f64, f64) {
    let a = frac * r;
    let rho = a.mul_add(-a, r * r).sqrt();
    (a, rho, 0.5 * (rho + r))
}

fn check(r: f64, frac: f64, op: BooleanOp) {
    let (a, rho, c) = lidded(r, frac);
    let ctx = format!("r={r} a={a} {op:?}");
    let want = reference_volume(r, a, c);
    let (mut topo, s, b) = operands(r, a, c);
    let result = boolean(&mut topo, op, s, b)
        .unwrap_or_else(|e| panic!("{ctx}: exact boolean refused: {e:?}"));
    assert!(
        remus_operations::validate::validate_solid(&topo, result)
            .unwrap()
            .is_valid(),
        "{ctx}: invalid result"
    );
    let faces = remus_topology::explorer::solid_faces(&topo, result).unwrap();
    let spheres = faces
        .iter()
        .filter(|&&f| topo.face(f).unwrap().surface().type_tag() == "sphere")
        .count();
    // Intersect: collar + lower hemisphere | 4 walls + lid. Cut: each wall
    // cap in two hemisphere halves + the lid cap | 4 walls + lid.
    let want_counts = match op {
        BooleanOp::Intersect => (2, 5),
        _ => (9, 5),
    };
    assert_eq!(
        (spheres, faces.len() - spheres),
        want_counts,
        "{ctx}: face census"
    );
    let opts = ClassifyOptions::default();
    let in_box = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 0.95 * c),
        Point3::new(0.0, 0.9 * a, 0.5 * rho),
        Point3::new(0.6 * a, 0.6 * a, 0.3 * rho),
    ];
    let out_of_box = [
        Point3::new(0.0, 0.0, 0.5 * (c + r)),
        Point3::new(0.5 * (a + r), 0.0, 0.2 * rho),
    ];
    for (points, is_in_box) in [(&in_box[..], true), (&out_of_box[..], false)] {
        for &p in points {
            let inside = match op {
                BooleanOp::Intersect => is_in_box,
                _ => !is_in_box,
            };
            let want_class = if inside {
                PointClassification::Inside
            } else {
                PointClassification::Outside
            };
            assert_eq!(
                classify_point(&topo, result, p, &opts).unwrap(),
                want_class,
                "{ctx}: {p:?}"
            );
        }
    }
    let got = remus_operations::measure::solid_volume(&topo, result, 1e-3 * r).unwrap();
    let want_op = match op {
        BooleanOp::Intersect => want,
        _ => (4.0 / 3.0 * std::f64::consts::PI).mul_add(r.powi(3), -want),
    };
    assert!(
        (got - want_op).abs() < 5e-4 * want_op,
        "{ctx}: volume {got}, want {want_op}"
    );
}

/// Walls far enough out that the collar sample stays inside them: the exact
/// intersect keeps the holed collar and the lower hemisphere.
#[test]
fn lidded_box_intersects_a_sphere_exactly() {
    for r in [1.0, 50.0] {
        check(r, 0.85, BooleanOp::Intersect);
    }
}

#[test]
#[ignore = "open: B61 — the cut drops the lid-cap lump and returns only the wall caps"]
fn b61_lidded_box_cut_keeps_the_lid_cap_lump() {
    for r in [1.0, 50.0] {
        check(r, 0.9, BooleanOp::Cut);
    }
}

#[test]
#[ignore = "open: B62 — the collar sample overshoots a wall close under the lid; exact boolean refused"]
fn b62_lidded_box_with_walls_close_under_the_lid_intersects_exactly() {
    for r in [1.0, 50.0] {
        check(r, 0.75, BooleanOp::Intersect);
    }
}
