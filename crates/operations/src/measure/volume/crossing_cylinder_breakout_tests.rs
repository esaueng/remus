//! A drill crossing a bar off its axis, large enough to break out through
//! the bar's wall: the cut and the union measure their closed-form volumes
//! wherever the bar's seam lies (esaueng/OpenZCAD K10).
//!
//! The bar is a cylinder r 10 along x over [-30, 30]; the drill a cylinder
//! along z, its axis 3 mm off the bar's. While the drill stays inside the
//! bar's top (r < 7) the two cut two closed loops; from r > 7 it breaks out
//! and they cut one. Both trimmed walls then carry a single fitted NURBS
//! loop, and the sampled trim integral read each about 1.1 mm³ high, by an
//! amount that moved with the bar's seam (2.16, 2.03 and 2.07 mm³ in total
//! at r 7.3 for the three seam positions below). These walls now integrate
//! on their own boundary (see `remus_check`'s `cylinder_green`). The
//! OpenZCAD K10 matrix met this as a union of the two grown to r 7 reading
//! 2.145 mm³ high.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::*;
use crate::boolean::{BooleanOp, boolean};
use remus_math::mat::Mat4;
use std::f64::consts::{FRAC_PI_2, PI};

const BAR_R: f64 = 10.0;
const BAR_LEN: f64 = 60.0;
const OFFSET: f64 = 3.0;
/// The trims are fitted NURBS sections that lie up to 1.4e-3 off the walls,
/// so the B-rep itself is only that close to the closed form: the readings
/// land within 0.022 mm³ of it (they were 2.0–2.7 mm³ high once the drill
/// reached the bar's top). The K10 matrix allows 0.05 at unit scale.
const VOLUME_TOLERANCE: f64 = 0.03;

/// Volume common to the bar and a drill of radius `r`: over the drill's
/// footprint `x² + (y − OFFSET)² < r²`, the bar's chord `2√(R² − y²)`
/// integrated in `y` in closed form, then in `x = r sin t` by composite
/// Gauss–Legendre (the substitution removes the endpoint square root).
fn overlap(r: f64) -> f64 {
    let chord = |y: f64| {
        let y = y.clamp(-BAR_R, BAR_R);
        y * (BAR_R * BAR_R - y * y).max(0.0).sqrt() + BAR_R * BAR_R * (y / BAR_R).asin()
    };
    let rule = remus_math::quadrature::gauss_legendre_points(16);
    let pieces = 512;
    let step = PI / f64::from(pieces);
    let mut total = 0.0;
    for k in 0..pieces {
        let mid = (f64::from(k) + 0.5).mul_add(step, -FRAC_PI_2);
        for point in rule {
            let t = (0.5 * step).mul_add(point.x, mid);
            let half = r * t.cos();
            total +=
                point.w * 0.5 * step * r * t.cos() * (chord(OFFSET + half) - chord(OFFSET - half));
        }
    }
    total
}

fn bar_and_drill(topo: &mut Topology, r: f64, seam_turn: f64) -> (SolidId, SolidId) {
    let bar = crate::primitives::make_cylinder(topo, BAR_R, BAR_LEN).unwrap();
    // About its own axis first, which moves only the seam.
    let along_x = Mat4::translation(-BAR_LEN / 2.0, 0.0, 0.0)
        * Mat4::rotation_y(FRAC_PI_2)
        * Mat4::rotation_z(seam_turn.to_radians());
    crate::transform::transform_solid(topo, bar, &along_x).unwrap();
    let drill = crate::primitives::make_cylinder(topo, r, BAR_LEN).unwrap();
    crate::transform::transform_solid(topo, drill, &Mat4::translation(0.0, OFFSET, -BAR_LEN / 2.0))
        .unwrap();
    (bar, drill)
}

#[test]
fn breakout_cross_drill_measures_its_closed_form_at_any_seam() {
    let bar_volume = PI * BAR_R * BAR_R * BAR_LEN;
    // r 6 and 6.9 stay inside the bar (two loops, the control), r 7 is
    // internally tangent to the bar's top line, and the rest break out.
    for r in [6.0, 6.9, 6.999, 7.0, 7.001, 7.05, 7.3, 8.0, 9.0] {
        let common = overlap(r);
        let drill_volume = PI * r * r * BAR_LEN;
        for seam_turn in [0.0, 45.0, 90.0] {
            for (op, expected) in [
                (BooleanOp::Cut, bar_volume - common),
                (BooleanOp::Fuse, bar_volume + drill_volume - common),
            ] {
                let mut topo = Topology::new();
                let (bar, drill) = bar_and_drill(&mut topo, r, seam_turn);
                let result = boolean(&mut topo, op, bar, drill).unwrap();
                let volume = solid_volume(&topo, result, 1e-4).unwrap();
                assert!(
                    (volume - expected).abs() <= VOLUME_TOLERANCE,
                    "r {r}, seam {seam_turn}°, {op:?}: volume {volume} vs closed form {expected} \\
                     ({:+e})",
                    volume - expected
                );
            }
        }
    }
}
