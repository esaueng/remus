//! Differential oracles for the torus line cull of the ray-cast vote loop.
//!
//! The cull claims that the quartic solver returns no root at all for a line
//! it clears, so a torus face contributes no crossing and no suspicion. The
//! first test checks that claim on random ring tori against random lines and
//! against lines that graze offset tori at distances on both sides of the
//! cull margin. The second compares whole ray triples, cardinal and generic,
//! with the vote loop before the cull (`cast_rays_unculled`) over solids'
//! worth of torus faces of every kind (banded, whole, trimmed). The third
//! does both from 1e8 to 1e50 away, across the reach cap past which the
//! solver overflows and nothing is culled. The fourth casts lines just past
//! small and thin tori, where the solver keeps near-tangent roots outside the
//! carrier box: only the cull's margin leaves those lines to the solver.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::suboptimal_flops
)]

use std::f64::consts::TAU;

use super::*;
use remus_math::analytic_intersection::intersect_line_torus;
use remus_math::surfaces::ToroidalSurface;

/// Xorshift64*, deterministic across platforms.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1_u64 << 53) as f64
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        (hi - lo).mul_add(self.next(), lo)
    }

    fn unit(&mut self) -> Vec3 {
        loop {
            let v = Vec3::new(
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
            );
            if let Ok(u) = v.normalize() {
                return u;
            }
        }
    }
}

fn random_torus(rng: &mut Rng, scale: f64) -> ToroidalSurface {
    let major = rng.range(0.5, 40.0) * scale;
    let ratio = if rng.next() < 0.1 {
        1.0
    } else {
        rng.range(0.02, 0.98)
    };
    let center = Point3::new(
        rng.range(-200.0, 200.0) * scale,
        rng.range(-200.0, 200.0) * scale,
        rng.range(-200.0, 200.0) * scale,
    );
    let axis = if rng.next() < 0.3 {
        [
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
        ][(rng.next() * 3.0) as usize % 3]
    } else {
        rng.unit()
    };
    ToroidalSurface::with_axis(center, major, major * ratio, axis).unwrap()
}

/// Every line the cull clears gets no root from the solver, so neither torus
/// crossing test can report a crossing or a suspicion on it.
#[test]
fn cleared_lines_get_no_torus_roots() {
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    let tol = Tolerance::default();
    let (mut lines, mut cleared, mut near_cleared) = (0_usize, 0_usize, 0_usize);
    for scale in [1e-3, 1.0, 1e3] {
        for _ in 0..60 {
            let torus = random_torus(&mut rng, scale);
            let (big_r, small_r) = (torus.major_radius(), torus.minor_radius());
            let extent = big_r + small_r;
            let mut cases: Vec<(Point3, Vec3, bool)> = Vec::new();
            // Lines through random points around the torus, in random and
            // classifier directions.
            for _ in 0..200 {
                let reach = extent * [1.0, 3.0, 10.0, 60.0][(rng.next() * 4.0) as usize % 4];
                let origin = torus.center()
                    + Vec3::new(
                        rng.range(-reach, reach),
                        rng.range(-reach, reach),
                        rng.range(-reach, reach),
                    );
                let dir = if rng.next() < 0.5 {
                    RAY_DIRS[(rng.next() * 6.0) as usize % 6]
                } else {
                    rng.unit()
                };
                cases.push((origin, dir, false));
            }
            // Lines tangent to the torus offset by a gap, the solver's
            // near-double-root regime, with gaps on both sides of the margin.
            for _ in 0..200 {
                let (u, v) = (rng.range(0.0, TAU), rng.range(0.0, TAU));
                let point = torus.evaluate(u, v);
                let normal = torus.normal(u, v);
                let reach = 1.0 + (point - torus.center()).length() + extent;
                let margin = small_r.mul_add(small_r, 0.0125 * reach * reach).sqrt() - small_r;
                let gap = margin * rng.range(0.0, 3.0);
                let tangent = normal.cross(rng.unit()).normalize().unwrap();
                let origin = point + normal * gap - tangent * rng.range(-5.0, 5.0) * extent;
                cases.push((origin, tangent, true));
            }
            for (origin, dir, near) in cases {
                lines += 1;
                if !torus_line_is_clear(&torus, origin, dir) {
                    continue;
                }
                cleared += 1;
                near_cleared += usize::from(near);
                let unit = dir.normalize().unwrap();
                assert!(
                    intersect_line_torus(&torus, origin, unit).is_empty(),
                    "cleared line has roots: {torus:?} from {origin:?} along {dir:?}"
                );
                assert_eq!(
                    ray_torus_crossings(origin, dir, &torus, None, None, tol),
                    (0, false)
                );
            }
        }
    }
    // The cull must fire, including on near-tangent lines past the margin,
    // or the oracle proves nothing.
    assert!(
        cleared * 3 > lines && near_cleared > 1000,
        "{cleared} of {lines} lines cleared, {near_cleared} near-tangent"
    );
}

/// The faces of one solid's worth of torus geometry: banded and whole tori,
/// and trimmed patches, on carriers in general position.
fn torus_faces(rng: &mut Rng) -> Vec<FaceGeom> {
    let mut faces = Vec::new();
    for k in 0..12 {
        let torus = random_torus(rng, 1.0);
        let geom = match k % 4 {
            0 => FaceGeom::Torus {
                surface: torus,
                v_band: None,
                u_band: None,
            },
            1 => FaceGeom::Torus {
                surface: torus,
                v_band: Some((rng.range(0.0, TAU), rng.range(0.5, 3.0))),
                u_band: Some((rng.range(0.0, TAU), rng.range(0.5, 3.0))),
            },
            _ => {
                let (u0, v0) = (rng.range(0.0, TAU), rng.range(0.0, TAU));
                let (du, dv) = (rng.range(0.3, 2.0), rng.range(0.3, 2.0));
                FaceGeom::TrimmedTorus {
                    surface: torus,
                    trim: super::super::torus_patch::TorusTrim {
                        polygon: vec![
                            Point2::new(u0, v0),
                            Point2::new(u0 + du, v0),
                            Point2::new(u0 + du * 0.5, v0 + dv),
                        ],
                        centre: Point2::new(u0 + du * 0.5, v0 + dv * 0.5),
                        margin: 1e-8,
                    },
                }
            }
        };
        faces.push(geom);
    }
    faces
}

/// Whole ray triples equal the vote loop's before the cull, and every face
/// the cull skips reports nothing when tested anyway.
#[test]
fn culled_rays_match_the_unculled_vote_loop() {
    let mut rng = Rng(0x2545_f491_4f6c_dd1d);
    let tol = Tolerance::default();
    let mut culls = 0_u64;
    for _ in 0..8 {
        let mut geoms = torus_faces(&mut rng);
        geoms.push(FaceGeom::planar(
            vec![
                Point3::new(-300.0, -300.0, 0.0),
                Point3::new(300.0, -300.0, 0.0),
                Point3::new(300.0, 300.0, 0.0),
                Point3::new(-300.0, 300.0, 0.0),
            ],
            vec![],
            Vec3::new(0.0, 0.0, 1.0),
            0.0,
        ));
        let faces = FaceGeoms::new(geoms);
        for _ in 0..400 {
            let point = if rng.next() < 0.5 {
                Point3::new(
                    rng.range(-260.0, 260.0),
                    rng.range(-260.0, 260.0),
                    rng.range(-260.0, 260.0),
                )
            } else {
                // On or next to a carrier.
                let geom = &faces.others[(rng.next() * 12.0) as usize % 12];
                let (FaceGeom::Torus { surface, .. } | FaceGeom::TrimmedTorus { surface, .. }) =
                    geom
                else {
                    unreachable!("only torus faces are collected here")
                };
                let (u, v) = (rng.range(0.0, TAU), rng.range(0.0, TAU));
                surface.evaluate(u, v) + surface.normal(u, v) * rng.range(-1e-3, 1e-3)
            };
            let _ = crate::perf::take_ray_work();
            for base in [0, 3] {
                assert_eq!(
                    cast_rays(&faces, point, base, tol, false),
                    cast_rays_unculled(&faces, point, base, tol, false),
                    "rays {base}.. from {point:?}"
                );
            }
            culls += crate::perf::take_ray_work().torus_culls;
            for dir in RAY_DIRS {
                for geom in &faces.others {
                    if line_clears_carrier(geom, point, dir) {
                        assert_eq!(ray_geom_crossings(point, dir, geom, tol), (0, false));
                    }
                }
            }
        }
    }
    assert!(culls > 10_000, "only {culls} torus faces culled");
}

/// Points from 1e8 to 1e50 away, in quarter decades. Up to the reach cap
/// every cleared line gets no root at all (none NaN either); past it nothing
/// is culled, so the solver's overflow there reaches the vote exactly as it
/// did before the cull. Ray triples equal the unculled loop's throughout.
#[test]
fn far_field_rays_match_the_unculled_vote_loop() {
    let mut rng = Rng(0x5851_f42d_4c95_7f2d);
    let tol = Tolerance::default();
    let faces = FaceGeoms::new(torus_faces(&mut rng));
    let carriers: Vec<&ToroidalSurface> = faces
        .others
        .iter()
        .map(|geom| match geom {
            FaceGeom::Torus { surface, .. } | FaceGeom::TrimmedTorus { surface, .. } => surface,
            _ => unreachable!("only torus faces are collected here"),
        })
        .collect();
    assert!(
        faces
            .others
            .iter()
            .any(|geom| matches!(geom, FaceGeom::TrimmedTorus { .. }))
            && faces
                .others
                .iter()
                .any(|geom| matches!(geom, FaceGeom::Torus { .. }))
    );
    let (mut capped_lines, mut culls_below_cap, mut cleared_checked) = (0_u64, 0_u64, 0_u64);
    for quarter in 0..=168_u32 {
        let distance = 10_f64.powf(f64::from(quarter).mul_add(0.25, 8.0));
        for k in 0..24 {
            let point = if k % 3 == 0 {
                // On a classifier axis through a carrier's centre, so some
                // rays head straight at it.
                let j = k / 3;
                let centre = carriers[j % carriers.len()].center();
                centre + RAY_DIRS[j % 3] * (distance * if j % 2 == 0 { 1.0 } else { -1.0 })
            } else {
                Point3::new(0.0, 0.0, 0.0) + rng.unit() * distance
            };
            let _ = crate::perf::take_ray_work();
            for base in [0, 3] {
                assert_eq!(
                    cast_rays(&faces, point, base, tol, false),
                    cast_rays_unculled(&faces, point, base, tol, false),
                    "rays {base}.. from {point:?}"
                );
            }
            let culls = crate::perf::take_ray_work().torus_culls;
            // Every carrier lies within 400 of the origin, so its reach
            // `1 + |p0| + R + r` passes the cap once the point is that much
            // farther than the cap.
            if distance > TORUS_CULL_MAX_REACH + 1e3 {
                capped_lines += 1;
                assert_eq!(culls, 0, "culled past the reach cap from {point:?}");
            } else {
                culls_below_cap += culls;
            }
            for dir in RAY_DIRS {
                let unit = dir.normalize().unwrap();
                for (geom, carrier) in faces.others.iter().zip(&carriers) {
                    if line_clears_carrier(geom, point, dir) {
                        cleared_checked += 1;
                        let roots = intersect_line_torus(carrier, point, unit);
                        assert!(
                            roots.is_empty(),
                            "cleared line has roots {roots:?}: {carrier:?} from {point:?} along {dir:?}"
                        );
                    }
                }
            }
        }
    }
    assert!(
        culls_below_cap > 5_000 && cleared_checked == culls_below_cap && capped_lines > 3_000,
        "{culls_below_cap} culls below the cap, {cleared_checked} cleared lines, {capped_lines} past it"
    );
}

/// Whether the line misses the carrier box grown only by the cull's rounding
/// allowance `1e-9 Λ`, not by its margin.
fn misses_bare_box(torus: &ToroidalSurface, origin: Point3, dir: Vec3) -> bool {
    let unit = dir.normalize().unwrap();
    let o = origin - torus.center();
    let frame = [torus.x_axis(), torus.y_axis(), torus.z_axis()];
    let p0 = frame.map(|axis| axis.dot(o));
    let v = frame.map(|axis| axis.dot(unit));
    let (big_r, small_r) = (torus.major_radius(), torus.minor_radius());
    let reach = 1.0 + o.length() + big_r + small_r;
    let half = [big_r + small_r, big_r + small_r, small_r].map(|h| h + 1e-9 * reach);
    let (mut lo, mut hi) = (f64::NEG_INFINITY, f64::INFINITY);
    for k in 0..3 {
        if v[k] == 0.0 {
            if p0[k].abs() > half[k] {
                return true;
            }
            continue;
        }
        let (a, b) = ((-half[k] - p0[k]) / v[k], (half[k] - p0[k]) / v[k]);
        lo = lo.max(a.min(b));
        hi = hi.min(a.max(b));
    }
    lo > hi
}

/// Lines just past small and thin tori, outside the carrier box. On many the
/// solver keeps a root: a near-tangent pair whose imaginary parts fall under
/// its `1e-7` filter, with a residual its filter accepts. The box's own
/// rounding allowance does not reach those lines, so only the cull's margin
/// keeps them uncleared; a cull without it fails here.
#[test]
fn margin_leaves_near_miss_roots_to_the_solver() {
    let mut rng = Rng(0x94d0_49bb_1331_11eb);
    let (mut lines, mut rooted, mut past_bare_box) = (0_usize, 0_usize, 0_usize);
    for k in 0..300 {
        let centre = Point3::new(
            rng.range(-5.0, 5.0),
            rng.range(-5.0, 5.0),
            rng.range(-5.0, 5.0),
        );
        let (big_r, small_r) = if k % 2 == 0 {
            // Thin: a unit-scale ring of a hair-thin tube.
            let major = rng.range(0.5, 2.0);
            (major, major * 10_f64.powf(rng.range(-12.0, -8.0)))
        } else {
            // Small: a whole torus far below the unit scale `Λ` starts at.
            let major = 10_f64.powf(rng.range(-7.0, -5.0));
            (major, major * rng.range(0.01, 0.5))
        };
        let torus = ToroidalSurface::new(centre, big_r, small_r).unwrap();
        for _ in 0..40 {
            let gap = 10_f64.powf(rng.range(-9.0, -7.0));
            let side = if rng.next() < 0.5 { 1.0 } else { -1.0 };
            let (origin, dir) = match (rng.next() * 3.0) as usize {
                // Level, over the top or bottom of the tube.
                0 => {
                    let (phi, theta) = (rng.range(0.0, TAU), rng.range(0.0, TAU));
                    let dir = Vec3::new(theta.cos(), theta.sin(), 0.0);
                    let over = centre
                        + Vec3::new(big_r * phi.cos(), big_r * phi.sin(), side * (small_r + gap));
                    (over - dir * (rng.range(-3.0, 3.0) * big_r), dir)
                }
                // Along the axis, past the outer equator.
                1 => {
                    let offset = side * (big_r + small_r + gap);
                    let foot = if rng.next() < 0.5 {
                        Vec3::new(offset, 0.0, 0.0)
                    } else {
                        Vec3::new(0.0, offset, 0.0)
                    };
                    let along = Vec3::new(0.0, 0.0, side);
                    (centre + foot - along * rng.range(-3.0, 3.0) * big_r, along)
                }
                // Level and square to the axis, past the outer equator.
                _ => {
                    let offset = side * (big_r + small_r + gap);
                    let (foot, along) = if rng.next() < 0.5 {
                        (Vec3::new(offset, 0.0, 0.0), Vec3::new(0.0, side, 0.0))
                    } else {
                        (Vec3::new(0.0, offset, 0.0), Vec3::new(side, 0.0, 0.0))
                    };
                    (centre + foot - along * rng.range(-3.0, 3.0) * big_r, along)
                }
            };
            lines += 1;
            let roots = intersect_line_torus(&torus, origin, dir.normalize().unwrap());
            if roots.is_empty() {
                continue;
            }
            rooted += 1;
            assert!(
                !torus_line_is_clear(&torus, origin, dir),
                "cleared a line with roots {roots:?}: {torus:?} from {origin:?} along {dir:?}"
            );
            past_bare_box += usize::from(misses_bare_box(&torus, origin, dir));
        }
    }
    assert!(
        past_bare_box > 500,
        "{past_bare_box} rooted lines past the bare box ({rooted} rooted of {lines})"
    );
}
