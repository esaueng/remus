//! Point classification must be exact on spherical faces.
//!
//! `classify_point` is what the solid-verification guidance designates as
//! ground truth for point-in-solid, and `remus_operations::classify` delegates
//! to it, so a wrong answer here quietly weakens every verification claim that
//! rests on it.
//!
//! It was wrong on spheres. The trim tested a ray's exit point against
//! `face_polygon`, which samples a closed boundary edge at a fixed 32 points —
//! so a hemisphere's equator became a 32-gon INSCRIBED in the true circle. A
//! ray leaving the sphere in the scalloped band between chord and arc (angular
//! half-width ~pi/n) was inside NEITHER hemisphere's polygon: the near face
//! rejected it on containment, the far face on the half-space test. The
//! crossing was counted by no face at all, parity flipped, and an interior
//! point came back `Outside`.
//!
//! Measured before the fix, `make_sphere(1.0, segments)`, points at 0.9r:
//! 32.4% wrong at segments = 8, 3.3% at 32, still 0.25% at 128 — every failure
//! Inside -> Outside. Volume, area, watertightness and `validate_solid` all
//! passed on the same solid, so nothing else in the verification ladder caught
//! it.
//!
//! These tests measure a RATE over many points. That is the whole reason the
//! defect survived: the existing single-probe test uses the sphere's centre,
//! which is classified correctly at every segment count.

#![allow(clippy::unwrap_used)]

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::primitives::{make_box, make_cylinder, make_sphere, make_torus};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

/// Deterministic low-discrepancy sequence — a fixed seed, not `rand`, so a
/// failure reproduces exactly.
fn halton(index: usize, base: usize) -> f64 {
    let (mut f, mut r, mut n) = (1.0 / base as f64, 0.0, index);
    while n > 0 {
        r += f * (n % base) as f64;
        n /= base;
        f /= base as f64;
    }
    r
}

/// `count` points spread over the sphere of radius `frac` about the origin.
fn ball_shell_points(count: usize, frac: f64) -> impl Iterator<Item = Point3> {
    (1..=count).map(move |i| {
        let theta = std::f64::consts::TAU * halton(i, 2);
        let phi = (2.0f64.mul_add(halton(i, 3), -1.0)).acos();
        Point3::new(
            frac * phi.sin() * theta.cos(),
            frac * phi.sin() * theta.sin(),
            frac * phi.cos(),
        )
    })
}

/// The load-bearing case. `segments = 8` is deliberate: it failed at 32% before
/// the fix, so this cannot pass by luck the way a 3% signal at 32 might.
#[test]
fn sphere_interior_points_classify_inside_at_every_segment_count() {
    for segments in [8usize, 12, 16, 32, 64] {
        let mut topo = Topology::new();
        let sphere = make_sphere(&mut topo, 1.0, segments).unwrap();

        for frac in [0.5, 0.9, 0.99] {
            let mut wrong = 0usize;
            let mut total = 0usize;
            for p in ball_shell_points(2000, frac) {
                total += 1;
                if !matches!(
                    classify_point(&topo, sphere, p, 0.01, 1e-7),
                    Ok(PointClassification::Inside)
                ) {
                    wrong += 1;
                }
            }
            assert_eq!(
                wrong, 0,
                "segments={segments}, points at {frac}r: {wrong}/{total} interior points \
                 classified as anything but Inside"
            );
        }
    }
}

/// Derived geometry: the cavity must read Outside and the surrounding material
/// Inside. Before the fix this was 2.7% wrong in the cavity while the solid's
/// volume was accurate to five decimal places — the exact pairing the
/// verification guidance warns about when it says never to sign off on volume
/// alone.
#[test]
fn cavity_of_a_bored_out_sphere_classifies_outside() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    let tool = make_sphere(&mut topo, 1.0, 32).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(2.0, 2.0, 2.0)).unwrap();
    let carved = boolean(&mut topo, BooleanOp::Cut, block, tool).unwrap();

    let centre = Point3::new(2.0, 2.0, 2.0);
    let (mut cavity_wrong, mut material_wrong) = (0usize, 0usize);
    for i in 1..=40000 {
        let p = Point3::new(4.0 * halton(i, 2), 4.0 * halton(i, 3), 4.0 * halton(i, 5));
        let d = (p - centre).length();
        let inset = [p.x(), p.y(), p.z()].iter().all(|c| *c > 0.2 && *c < 3.8);

        // The two radial bands are disjoint, so the collapsed form keeps the
        // original meaning: a point that classifies correctly in the first band
        // falls through to a second test that its own radius rules out.
        if d < 0.85
            && !matches!(
                classify_point(&topo, carved, p, 0.01, 1e-7),
                Ok(PointClassification::Outside)
            )
        {
            cavity_wrong += 1;
        } else if d > 1.15
            && inset
            && !matches!(
                classify_point(&topo, carved, p, 0.01, 1e-7),
                Ok(PointClassification::Inside)
            )
        {
            material_wrong += 1;
        }
    }
    assert_eq!(cavity_wrong, 0, "points inside the cavity read as material");
    assert_eq!(material_wrong, 0, "points in the material read as cavity");
}

/// The case that separates a correct fix from a plausible one.
///
/// Here the sphere breaks the block's top face, so the carved region is bounded
/// by an ANNULAR spherical face whose complementary cap is not a face of the
/// same solid. A trim whose cap side is taken from the boundary polygon's
/// winding gets this wrong — the winding-derived sign is only accidentally
/// right on a whole sphere, where two complementary hemispheres tile it and the
/// errors cancel. Measured on this solid: 47.1% of the carved region wrong
/// before the fix, 34.3% with the winding-derived sign, 0% when the side comes
/// from the surface normal.
#[test]
fn carved_region_of_a_surface_breaking_sphere_classifies_outside() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    let tool = make_sphere(&mut topo, 1.0, 32).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(2.0, 2.0, 3.5)).unwrap();
    let carved = boolean(&mut topo, BooleanOp::Cut, block, tool).unwrap();

    let centre = Point3::new(2.0, 2.0, 3.5);
    let (mut wrong, mut total) = (0usize, 0usize);
    for i in 1..=40000 {
        let p = Point3::new(4.0 * halton(i, 2), 4.0 * halton(i, 3), 4.0 * halton(i, 5));
        // Inside the tool sphere and still within the block's extent.
        if (p - centre).length() < 0.85 && p.z() < 4.0 {
            total += 1;
            if !matches!(
                classify_point(&topo, carved, p, 0.01, 1e-7),
                Ok(PointClassification::Outside)
            ) {
                wrong += 1;
            }
        }
    }
    assert!(total > 500, "test needs a meaningful sample, got {total}");
    assert_eq!(wrong, 0, "{wrong}/{total} carved points read as material");
}

/// The other curved surface types must be untouched. They already used the
/// analytic UV trim rather than the chorded polygon, and measured 0% both
/// before and after — so a regression here means the fix leaked out of the
/// sphere arm.
#[test]
fn other_primitives_are_unaffected() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
    let torus = make_torus(&mut topo, 2.0, 0.5, 32).unwrap();
    let block = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();

    for i in 1..=2000 {
        let (a, b, c) = (halton(i, 2), halton(i, 3), halton(i, 5));

        let radial = 0.8 * a.sqrt();
        let angle = std::f64::consts::TAU * b;
        let in_cylinder = Point3::new(radial * angle.cos(), radial * angle.sin(), 0.2 + 1.6 * c);
        assert!(
            matches!(
                classify_point(&topo, cylinder, in_cylinder, 0.01, 1e-7),
                Ok(PointClassification::Inside)
            ),
            "cylinder interior point {in_cylinder:?} misclassified"
        );

        let tube = std::f64::consts::TAU * b;
        let around = 0.35 * c.sqrt();
        let in_torus = Point3::new(
            (2.0 + around * tube.cos()) * angle.cos(),
            (2.0 + around * tube.cos()) * angle.sin(),
            around * tube.sin(),
        );
        assert!(
            matches!(
                classify_point(&topo, torus, in_torus, 0.01, 1e-7),
                Ok(PointClassification::Inside)
            ),
            "torus interior point {in_torus:?} misclassified"
        );

        let in_box = Point3::new(0.4 + 3.2 * a, 0.4 + 3.2 * b, 0.4 + 3.2 * c);
        assert!(
            matches!(
                classify_point(&topo, block, in_box, 0.01, 1e-7),
                Ok(PointClassification::Inside)
            ),
            "box interior point {in_box:?} misclassified"
        );
    }
}

/// A spherical face carrying an INNER wire — the case that exercises the hole
/// test on the new trim rather than just the outer boundary.
///
/// Boring a cylinder through a ball leaves two spherical faces each with a
/// circular hole where the bore exits. Measured before the fix: 3.85% of the
/// remaining material read as empty and 0.24% of the bore read as solid.
#[test]
fn a_spherical_face_with_a_hole_classifies_correctly() {
    let mut topo = Topology::new();
    let ball = make_sphere(&mut topo, 3.0, 32).unwrap();
    let bore = make_cylinder(&mut topo, 0.8, 12.0).unwrap();
    transform_solid(&mut topo, bore, &Mat4::translation(0.0, 0.0, -6.0)).unwrap();
    let drilled = boolean(&mut topo, BooleanOp::Cut, ball, bore).unwrap();

    let holed = remus_topology::explorer::solid_faces(&topo, drilled)
        .unwrap()
        .iter()
        .filter(|f| {
            topo.face(**f).is_ok_and(|face| {
                matches!(face.surface(), remus_topology::face::FaceSurface::Sphere(_))
                    && !face.inner_wires().is_empty()
            })
        })
        .count();
    assert_eq!(holed, 2, "test setup: expected two holed spherical faces");

    let (mut material_wrong, mut bore_wrong) = (0usize, 0usize);
    let origin = Point3::new(0.0, 0.0, 0.0);
    for i in 1..=40000 {
        let p = Point3::new(
            6.0f64.mul_add(halton(i, 2), -3.0),
            6.0f64.mul_add(halton(i, 3), -3.0),
            6.0f64.mul_add(halton(i, 5), -3.0),
        );
        let axial = p.x().hypot(p.y());
        let radial = (p - origin).length();
        // Disjoint again: `axial > 0.95` and `axial < 0.65` cannot both hold.
        if radial < 2.85
            && axial > 0.95
            && !matches!(
                classify_point(&topo, drilled, p, 0.01, 1e-7),
                Ok(PointClassification::Inside)
            )
        {
            material_wrong += 1;
        } else if axial < 0.65
            && radial < 2.85
            && !matches!(
                classify_point(&topo, drilled, p, 0.01, 1e-7),
                Ok(PointClassification::Outside)
            )
        {
            bore_wrong += 1;
        }
    }
    assert_eq!(material_wrong, 0, "material around the bore read as empty");
    assert_eq!(bore_wrong, 0, "the bore read as solid");
}
