//! Fuzz Smoke `modifier_ops` finding (scheduled run 2026-10-04,
//! `crash-acf0cccab6858f3d22affdd4c13c2c7349d7c3a9`): a 1 x 3 x 1 box fused
//! with a torus (R = 6.5, r = 2.5) turned pi/6 about X and moved to
//! (0, -1, -1.5), then filleted, read 1457.8 by `mass_properties` against
//! 804.3 by `solid_volume`.
//!
//! The base body was already wrong; the fillet only inherited it. The torus
//! bites into two box faces; their section arcs are NURBS recognized as a
//! circle (the x = 0 face holds the torus axis, so its section is two
//! meridian circles) and as ellipses (the z = 1 face), and both run clockwise
//! in the face frame. The exact planar
//! path integrated the circles' complements and measured the ellipse span
//! about the wrong point, reading those faces at 41.8 and 664.0 square units.
//!
//! The reference is independent of the kernel: the torus by Pappus plus the
//! box, less their overlap, integrated over the box's x-z face by
//! Gauss-Legendre with each column's y-interval taken from the exact roots of
//! the line-torus quartic (converged to 13 digits at 100, 200 and 400 points
//! per axis).
#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::f64::consts::{FRAC_PI_6, PI};

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{mass_properties, solid_bounding_box, solid_volume};
use remus_operations::primitives;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// Volume of the box-torus overlap, from the external quartic-root quadrature.
const OVERLAP: f64 = 0.558_159_753_304_908;

fn exact_union() -> f64 {
    2.0 * PI * PI * 6.5 * 2.5 * 2.5 + 3.0 - OVERLAP
}

fn box_torus_fuse(topo: &mut Topology) -> SolidId {
    let stock = primitives::make_box(topo, 1.0, 3.0, 1.0).unwrap();
    let tool = primitives::make_torus(topo, 6.5, 2.5, 16).unwrap();
    let place = Mat4::translation(0.0, -1.0, -1.5) * Mat4::rotation_x(FRAC_PI_6);
    transform_solid(topo, tool, &place).unwrap();
    boolean(topo, BooleanOp::Fuse, stock, tool).unwrap()
}

fn tessellated_volume(topo: &Topology, solid: SolidId) -> f64 {
    let aabb = solid_bounding_box(topo, solid).unwrap();
    let diag = (aabb.max - aabb.min).length();
    solid_volume(topo, solid, diag * 1e-4).unwrap()
}

#[test]
fn box_torus_fuse_mass_properties_match_the_closed_form() {
    let mut topo = Topology::new();
    let body = box_torus_fuse(&mut topo);
    let exact = exact_union();
    let mass = mass_properties(&topo, body).unwrap().mass;
    // The z = 1 section is a spiric curve its recognized ellipse matches to
    // the recognizer's 1e-6 relative fit check, which bounds this residual.
    assert!(
        (mass - exact).abs() <= exact * 1e-6,
        "mass_properties {mass} vs closed form {exact}"
    );
    let tess = tessellated_volume(&topo, body);
    assert!(
        (tess - exact).abs() <= exact * 1e-3,
        "solid_volume {tess} vs closed form {exact}"
    );
}
