//! Differential oracle for the cylinder × NURBS carrier-clearance gate.
//!
//! `nurbs_net_clear_of_cylinder` lets phase FF skip the surface marcher for
//! a pair whose NURBS control net is clear of the cylinder's carrier. The
//! gate is sound when it only ever skips a pair the marcher returns nothing
//! for, so every cleared pair here is also run through the unchanged
//! marcher (`nurbs_nurbs_intersection` on the converted cylinder, exactly
//! what the arm did before the gate), and its surface is sampled against
//! the closed-form distance to the carrier. Pairs that cross, touch or wrap
//! the cylinder must not be cleared.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::context::OperationContext;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::surfaces::CylindricalSurface;

use super::{
    CARRIER_CLEARANCE, Point3, Vec3, nurbs_net_clear_of_cylinder, nurbs_nurbs_intersection,
};

/// `splitmix64`.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    #[allow(clippy::cast_precision_loss)]
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        (hi - lo).mul_add((self.next() >> 11) as f64 / (1_u64 << 53) as f64, lo)
    }
}

/// A bicubic patch through a 4 × 4 net, rational when `weights` is given.
fn patch(net: &[[Point3; 4]; 4], weights: Option<[[f64; 4]; 4]>) -> NurbsSurface {
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    NurbsSurface::new(
        3,
        3,
        knots.clone(),
        knots,
        net.iter().map(|row| row.to_vec()).collect(),
        weights.map_or_else(
            || vec![vec![1.0; 4]; 4],
            |w| w.iter().map(|row| row.to_vec()).collect(),
        ),
    )
    .unwrap()
}

/// A net over radius `r0..r1`, angle `a0..a1` and height `z0..z1` about
/// `cylinder`'s axis, each point jittered by up to `jitter`.
fn polar_net(
    cylinder: &CylindricalSurface,
    (r0, r1): (f64, f64),
    (a0, a1): (f64, f64),
    (z0, z1): (f64, f64),
    jitter: f64,
    rng: &mut Rng,
) -> [[Point3; 4]; 4] {
    let (x, z) = (cylinder.x_axis(), cylinder.axis());
    let y = z.cross(x);
    std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            #[allow(clippy::cast_precision_loss)]
            let (s, t) = (i as f64 / 3.0, j as f64 / 3.0);
            let radius = (r1 - r0).mul_add(s, r0);
            // The angle drifts with height, as a helical flank's does.
            let angle = (a1 - a0).mul_add(0.5 * (s + t), a0);
            let height = (z1 - z0).mul_add(t, z0);
            cylinder.origin()
                + x * (radius * angle.cos())
                + y * (radius * angle.sin())
                + z * height
                + Vec3::new(
                    rng.range(-jitter, jitter),
                    rng.range(-jitter, jitter),
                    rng.range(-jitter, jitter),
                )
        })
    })
}

/// Least distance from a 33 × 33 sampling of `surface` to the carrier.
fn sampled_gap(cylinder: &CylindricalSurface, surface: &NurbsSurface) -> f64 {
    let mut gap = f64::INFINITY;
    for i in 0..=32 {
        for j in 0..=32 {
            let point = surface.evaluate(f64::from(i) / 32.0, f64::from(j) / 32.0);
            let offset = point - cylinder.origin();
            let radial = offset - cylinder.axis() * offset.dot(cylinder.axis());
            gap = gap.min((radial.length() - cylinder.radius()).abs());
        }
    }
    gap
}

/// The arm as it ran before the gate: the cylinder converted over a band
/// holding the patch, through the marcher.
fn marched_curves(cylinder: &CylindricalSurface, surface: &NurbsSurface) -> usize {
    let heights: Vec<f64> = surface
        .control_points()
        .iter()
        .flatten()
        .map(|&p| (p - cylinder.origin()).dot(cylinder.axis()))
        .collect();
    let lo = heights.iter().copied().fold(f64::INFINITY, f64::min);
    let hi = heights.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let converted = cylinder.to_nurbs(lo - 0.5, hi + 0.5).unwrap();
    nurbs_nurbs_intersection(&converted, surface, &OperationContext::new())
        .unwrap()
        .len()
}

fn tilted_cylinder(rng: &mut Rng, radius: f64) -> CylindricalSurface {
    let axis = Vec3::new(
        rng.range(-1.0, 1.0),
        rng.range(-1.0, 1.0),
        rng.range(0.2, 1.0),
    );
    let origin = Point3::new(
        rng.range(-20.0, 20.0),
        rng.range(-20.0, 20.0),
        rng.range(-20.0, 20.0),
    );
    CylindricalSurface::new(origin, axis, radius).unwrap()
}

/// Random flank-like patches beside, inside and across tilted cylinders:
/// whatever the gate clears, the marcher finds nothing on and the sampled
/// surface stays clear of the carrier.
#[test]
fn cleared_pairs_are_pairs_the_marcher_finds_nothing_on() {
    let mut rng = Rng(0x5EED_CA11);
    let (mut cleared_outside, mut cleared_inside, mut kept) = (0, 0, 0);
    for case in 0..60 {
        let radius = rng.range(2.0, 12.0);
        let cylinder = tilted_cylinder(&mut rng, radius);
        // A third outside, a third inside, a third straddling the wall.
        let radii = match case % 3 {
            0 => {
                let r0 = radius + rng.range(0.0, 2.0);
                (r0, r0 + rng.range(0.5, 3.0))
            }
            1 => {
                let r1 = radius - rng.range(0.0, 0.4);
                (rng.range(0.1, 0.5) * r1, r1)
            }
            _ => (radius - rng.range(0.1, 1.0), radius + rng.range(0.1, 1.0)),
        };
        let start = rng.range(0.0, 6.0);
        let net = polar_net(
            &cylinder,
            radii,
            (start, start + rng.range(0.05, 0.5)),
            (rng.range(-3.0, 0.0), rng.range(1.0, 6.0)),
            0.02,
            &mut rng,
        );
        let weights = (case % 2 == 0)
            .then(|| std::array::from_fn(|_| std::array::from_fn(|_| rng.range(0.3, 3.0))));
        let surface = patch(&net, weights);
        if nurbs_net_clear_of_cylinder(&cylinder, &surface) {
            let gap = sampled_gap(&cylinder, &surface);
            assert!(gap > CARRIER_CLEARANCE, "case {case}: sampled gap {gap}");
            assert_eq!(marched_curves(&cylinder, &surface), 0, "case {case}");
            match case % 3 {
                0 => cleared_outside += 1,
                1 => cleared_inside += 1,
                _ => panic!("case {case}: a patch across the wall was cleared"),
            }
        } else {
            kept += 1;
        }
    }
    // Not vacuous: each side is cleared often, and the straddling third
    // (at least) is kept.
    assert!(cleared_outside >= 5, "outside cleared {cleared_outside}");
    assert!(cleared_inside >= 5, "inside cleared {cleared_inside}");
    assert!(kept >= 20, "kept {kept}");
}

fn z_cylinder(radius: f64) -> CylindricalSurface {
    CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), radius).unwrap()
}

/// A flat net at height `z` whose corners are `points` (x, y), bilinear in
/// between.
fn flat_patch(corners: [(f64, f64); 4], z: f64) -> NurbsSurface {
    let [a, b, c, d] = corners;
    let net = std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            #[allow(clippy::cast_precision_loss)]
            let (s, t) = (i as f64 / 3.0, j as f64 / 3.0);
            let lerp = |p: f64, q: f64, w: f64| (q - p).mul_add(w, p);
            Point3::new(
                lerp(lerp(a.0, b.0, s), lerp(d.0, c.0, s), t),
                lerp(lerp(a.1, b.1, s), lerp(d.1, c.1, s), t),
                z,
            )
        })
    });
    patch(&net, None)
}

/// The gap that decides: a patch `CARRIER_CLEARANCE` and more from the wall
/// is cleared on either side, one closer is kept.
#[test]
fn the_clearance_is_the_decision_boundary() {
    let cylinder = z_cylinder(5.0);
    let beside = |gap: f64| {
        let x = 5.0 + gap;
        flat_patch([(x, -1.0), (x + 2.0, -1.0), (x + 2.0, 1.0), (x, 1.0)], 0.0)
    };
    assert!(nurbs_net_clear_of_cylinder(&cylinder, &beside(0.1)));
    assert!(nurbs_net_clear_of_cylinder(
        &cylinder,
        &beside(2.0 * CARRIER_CLEARANCE)
    ));
    assert!(!nurbs_net_clear_of_cylinder(
        &cylinder,
        &beside(0.5 * CARRIER_CLEARANCE)
    ));
    assert!(!nurbs_net_clear_of_cylinder(&cylinder, &beside(0.0)));

    // Inside: the farthest corner sets the gap.
    let within = |gap: f64| {
        let reach = (5.0 - gap) / 2.0_f64.sqrt();
        flat_patch(
            [(0.5, 0.5), (reach, 0.5), (reach, reach), (0.5, reach)],
            1.0,
        )
    };
    assert!(nurbs_net_clear_of_cylinder(&cylinder, &within(0.1)));
    assert!(!nurbs_net_clear_of_cylinder(
        &cylinder,
        &within(0.5 * CARRIER_CLEARANCE)
    ));
}

/// A patch crossing the wall is kept, and the marcher does find its section.
#[test]
fn a_patch_through_the_wall_is_kept_and_marched() {
    let cylinder = z_cylinder(5.0);
    let through = flat_patch([(3.0, -1.0), (7.0, -1.0), (7.0, 1.0), (3.0, 1.0)], 0.0);
    assert!(!nurbs_net_clear_of_cylinder(&cylinder, &through));
    assert!(marched_curves(&cylinder, &through) > 0);
}

/// The gate is conservative, not exact. A net wrapped around the axis is
/// kept, and so is a long strip beside the wall that a half-space does
/// separate but the one the gate tries (along the net's mean radial
/// direction) does not, though the marcher finds nothing on it.
#[test]
fn nets_the_mean_direction_does_not_separate_are_kept() {
    let cylinder = z_cylinder(5.0);
    let mut rng = Rng(7);
    let sector = patch(
        &polar_net(&cylinder, (7.0, 9.0), (0.0, 4.7), (0.0, 1.0), 0.0, &mut rng),
        None,
    );
    assert!(!nurbs_net_clear_of_cylinder(&cylinder, &sector));

    let strip = flat_patch([(5.5, -1.0), (6.5, -1.0), (6.5, 30.0), (5.5, 30.0)], 0.0);
    assert!(sampled_gap(&cylinder, &strip) > 0.4);
    assert!(!nurbs_net_clear_of_cylinder(&cylinder, &strip));
    assert_eq!(marched_curves(&cylinder, &strip), 0);
}

/// The clearance grows with the coordinates, so rounding in the offsets
/// never clears a pair: at 1e9 from the origin a gap of twice
/// `CARRIER_CLEARANCE` is kept and a 100-unit gap cleared.
#[test]
fn far_from_the_origin_the_clearance_is_relative() {
    let centre = 1.0e9;
    let cylinder = CylindricalSurface::new(
        Point3::new(centre, centre, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        5.0,
    )
    .unwrap();
    let beside = |gap: f64| {
        let x = centre + 5.0 + gap;
        flat_patch(
            [
                (x, centre - 1.0),
                (x + 2.0, centre - 1.0),
                (x + 2.0, centre + 1.0),
                (x, centre + 1.0),
            ],
            0.0,
        )
    };
    assert!(!nurbs_net_clear_of_cylinder(
        &cylinder,
        &beside(2.0 * CARRIER_CLEARANCE)
    ));
    assert!(nurbs_net_clear_of_cylinder(&cylinder, &beside(100.0)));
}
