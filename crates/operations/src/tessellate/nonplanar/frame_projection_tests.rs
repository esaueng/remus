//! A boundary sample the coarse seed grid cannot project (Newton fails to
//! converge from the nearest of its 81 nodes) is retried from a node in
//! every knot span, on a strip swept many turns round an axis like a worm's
//! root fillet.

#![allow(clippy::unwrap_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::nurbs::NurbsCurve;
use remus_math::nurbs::projection::project_point_to_surface_with_grid;
use remus_math::nurbs::screw::ScrewMotion;
use remus_math::vec::{Point3, Vec3};
use remus_topology::face::FaceSurface;

use super::{NurbsFrame, self_relative};

/// A 0.4 rad arc of radius 0.76 facing the axis at 8.26 from it, swept 20
/// radians over a height of 20 (fitted for radii up to 12): the shape of a
/// single-start worm's root fillet.
fn swept_strip() -> remus_math::nurbs::NurbsSurface {
    let (rho, half) = (0.76, 0.2_f64);
    let at = |angle: f64, radius: f64| {
        Point3::new(8.26 + radius * angle.cos(), radius * angle.sin(), 0.0)
    };
    let arc = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            at(PI - half, rho),
            at(PI, rho / half.cos()),
            at(PI + half, rho),
        ],
        vec![1.0, half.cos(), 1.0],
    )
    .unwrap();
    let motion = ScrewMotion::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        1.0,
        20.0,
    )
    .unwrap();
    let plan = motion.plan(12.0, 1e-5, 4096).unwrap();
    motion.surface(&plan, &arc).unwrap()
}

#[test]
fn a_strip_winding_many_turns_projects_every_sample() {
    let surface = swept_strip();
    let frame = NurbsFrame::new(&surface).unwrap();
    let FaceSurface::Nurbs(relative) = frame.surface() else {
        panic!("a NURBS frame charts a NURBS carrier");
    };
    let (u0, u1) = surface.domain_u();
    let (v0, v1) = surface.domain_v();
    let mut coarse_failures = 0;
    for i in 0..=400 {
        for j in 0..=4 {
            let u = u0 + (u1 - u0) * f64::from(i) / 400.0;
            let v = v0 + (v1 - v0) * f64::from(j) / 4.0;
            let point = surface.evaluate(u, v);
            let coarse = project_point_to_surface_with_grid(
                relative,
                self_relative(point, frame.anchor),
                1e-6,
                &frame.grid,
            );
            // A coarse result in the wrong basin is the mesher's to reseed
            // (it checks every chart point against its sample); a failure
            // must now come back on the point.
            if coarse.is_err() {
                coarse_failures += 1;
                let (pu, pv) = frame.project(point).unwrap();
                let miss = (surface.evaluate(pu, pv) - point).length();
                assert!(miss < 1e-6, "({u}, {v}) projects {miss:e} away");
            }
        }
    }
    // The retry is what this test exercises.
    assert!(coarse_failures > 0);
}
