//! `intersect_plane_torus` must reproduce its pre-tabulation self bit for
//! bit. The oracles below are verbatim copies of the old grid scan, Newton
//! refinement and greedy chaining. They inline the old torus formula instead
//! of calling `ToroidalSurface::evaluate`, which now runs through the shared
//! `tube_terms` / `evaluate_tube`, so a fault there cannot move the oracle
//! and the code together.

#![allow(
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::suboptimal_flops
)]

use super::*;

/// The pre-change `ToroidalSurface::evaluate`, inlined.
fn ref_evaluate(torus: &ToroidalSurface, u: f64, v: f64) -> Point3 {
    let (sin_u, cos_u) = u.sin_cos();
    let (sin_v, cos_v) = v.sin_cos();
    let tube_radius = torus.minor_radius().fma(cos_v, torus.major_radius());
    torus.center()
        + torus.x_axis() * (tube_radius * cos_u)
        + torus.y_axis() * (tube_radius * sin_u)
        + torus.z_axis() * (torus.minor_radius() * sin_v)
}

/// The pre-change `newton_refine_torus`, verbatim but for [`ref_evaluate`].
fn ref_newton(torus: &ToroidalSurface, normal: Vec3, d: f64, mut u: f64, mut v: f64) -> (f64, f64) {
    let eps = 1e-6;
    for _ in 0..10 {
        let f = dot_np(normal, ref_evaluate(torus, u, v)) - d;
        if f.abs() < 1e-12 {
            break;
        }
        // Numerical gradient via central differences.
        let fu = (dot_np(normal, ref_evaluate(torus, u + eps, v))
            - dot_np(normal, ref_evaluate(torus, u - eps, v)))
            / (2.0 * eps);
        let fv = (dot_np(normal, ref_evaluate(torus, u, v + eps))
            - dot_np(normal, ref_evaluate(torus, u, v - eps)))
            / (2.0 * eps);

        let grad_sq = fu.fma(fu, fv * fv);
        if grad_sq < 1e-20 {
            break;
        }
        let step = f / grad_sq;
        u -= step * fu;
        v -= step * fv;
    }
    (u, v)
}

/// The pre-change grid scan of `intersect_plane_torus`, verbatim but for
/// [`ref_evaluate`] and [`ref_newton`].
fn ref_grid_crossings(torus: &ToroidalSurface, normal: Vec3, d: f64) -> Vec<(f64, f64, Point3)> {
    let n_grid = 128_usize;

    // Signed distance to plane for a torus point.
    let sdf = |u: f64, v: f64| -> f64 { dot_np(normal, ref_evaluate(torus, u, v)) - d };

    // Collect zero-crossing points by scanning edges of a (u,v) grid.
    let mut crossing_pts: Vec<(f64, f64, Point3)> = Vec::new();

    let du = TAU / (n_grid as f64);
    let dv = TAU / (n_grid as f64);

    // Offset grid by half a cell to avoid landing exactly on zero crossings
    // (e.g. sin(0) = 0.0 exactly in IEEE 754, which defeats sign-change detection).
    let u_off = du * 0.5;
    let v_off = dv * 0.5;

    for iu in 0..n_grid {
        for iv in 0..n_grid {
            let u0 = (iu as f64).fma(du, u_off);
            let v0 = (iv as f64).fma(dv, v_off);
            let u1 = u0 + du;
            let v1 = v0 + dv;

            let f00 = sdf(u0, v0);
            let f10 = sdf(u1, v0);
            let f01 = sdf(u0, v1);

            // Check horizontal edge (u0,v0)-(u1,v0).
            if f00 * f10 < 0.0 {
                let t = f00 / (f00 - f10);
                let u = t.fma(u1 - u0, u0);
                let (u_r, v_r) = ref_newton(torus, normal, d, u, v0);
                crossing_pts.push((u_r, v_r, ref_evaluate(torus, u_r, v_r)));
            }

            // Check vertical edge (u0,v0)-(u0,v1).
            if f00 * f01 < 0.0 {
                let t = f00 / (f00 - f01);
                let v = t.fma(v1 - v0, v0);
                let (u_r, v_r) = ref_newton(torus, normal, d, u0, v);
                crossing_pts.push((u_r, v_r, ref_evaluate(torus, u_r, v_r)));
            }
        }
    }
    crossing_pts
}

/// The pre-change `intersect_plane_torus`: [`ref_grid_crossings`], then the
/// greedy chaining and fitting, verbatim.
fn ref_intersect_plane_torus(
    torus: &ToroidalSurface,
    normal: Vec3,
    d: f64,
) -> Result<Vec<IntersectionCurve>, MathError> {
    let crossing_pts = ref_grid_crossings(torus, normal, d);

    if crossing_pts.is_empty() {
        return Ok(vec![]);
    }

    // Group nearby points into connected curves via greedy chaining.
    let mut used = vec![false; crossing_pts.len()];
    let mut curves = Vec::new();

    for start in 0..crossing_pts.len() {
        if used[start] {
            continue;
        }
        used[start] = true;
        let mut chain = vec![start];

        loop {
            let last = chain[chain.len() - 1];
            let last_pt = crossing_pts[last].2;
            let mut best_idx = None;
            let mut best_dist = torus.minor_radius() / 3.0;

            for (j, &is_used) in used.iter().enumerate() {
                if is_used {
                    continue;
                }
                let dist = (crossing_pts[j].2 - last_pt).length();
                if dist < best_dist {
                    best_dist = dist;
                    best_idx = Some(j);
                }
            }

            if let Some(j) = best_idx {
                used[j] = true;
                chain.push(j);
            } else {
                break;
            }
        }

        if chain.len() >= 4 {
            if (crossing_pts[chain[1]].2 - crossing_pts[chain[0]].2)
                .dot(crossing_pts[chain[2]].2 - crossing_pts[chain[1]].2)
                < 0.0
            {
                chain.swap(0, 1);
            }
            let mut pts: Vec<Point3> = chain.iter().map(|&i| crossing_pts[i].2).collect();
            let mut ipts: Vec<IntersectionPoint> = chain
                .iter()
                .map(|&i| {
                    Ok::<_, MathError>(IntersectionPoint {
                        point: crossing_pts[i].2,
                        param1: (crossing_pts[i].0, crossing_pts[i].1),
                        param2: plane_uv(normal, d, crossing_pts[i].2)?,
                    })
                })
                .collect::<Result<Vec<_>, _>>()?;

            let closing_gap = (pts[pts.len() - 1] - pts[0]).length();
            let median_spacing = {
                let mut spac: Vec<f64> = pts.windows(2).map(|w| (w[1] - w[0]).length()).collect();
                spac.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                spac.get(spac.len() / 2).copied().unwrap_or(0.0)
            };
            let wrapped =
                closing_gap > 1e-9 && median_spacing > 1e-12 && closing_gap <= 2.0 * median_spacing;
            if wrapped {
                pts.push(pts[0]);
                ipts.push(ipts[0]);
            }

            if let Ok(curve) = interpolate(&pts, 3.min(pts.len() - 1)) {
                curves.push(IntersectionCurve {
                    curve,
                    points: ipts,
                });
            }
        }
    }

    Ok(curves)
}

/// SplitMix64: a fixed-seed generator, so every run sees the same cases.
struct SplitMix(u64);

impl SplitMix {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[lo, hi)`.
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        let unit = (self.next_u64() >> 11) as f64 / (1_u64 << 53) as f64;
        (hi - lo).mul_add(unit, lo)
    }

    fn direction(&mut self) -> Vec3 {
        loop {
            let v = Vec3::new(
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
            );
            let len = v.length();
            if (0.1..=1.0).contains(&len) {
                return v * (1.0 / len);
            }
        }
    }
}

struct Case {
    label: String,
    torus: ToroidalSurface,
    normal: Vec3,
    d: f64,
}

fn axis_torus(major: f64, minor: f64) -> ToroidalSurface {
    ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), major, minor).unwrap()
}

fn case(label: &str, torus: &ToroidalSurface, normal: Vec3, d: f64) -> Case {
    Case {
        label: label.to_owned(),
        torus: torus.clone(),
        normal,
        d,
    }
}

/// Hand-picked planes: the three box faces of `torus_notch_*` that reach the
/// torus (and three that miss it), equatorial and near-equatorial-tangent
/// planes, meridians, the inner/outer tangents (the inner one cuts a
/// figure-eight), and a spindle torus.
fn named_cases() -> Vec<Case> {
    let bench = axis_torus(10.0, 3.0);
    let (x, y, z) = (
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    );
    let mut cases = vec![
        case("bench x=6", &bench, -x, -6.0),
        case("bench y=-4", &bench, -y, 4.0),
        case("bench y=4", &bench, y, 4.0),
        case("bench x=14 (miss)", &bench, x, 14.0),
        case("bench z=4 (miss)", &bench, z, 4.0),
        case("bench z=-4 (miss)", &bench, -z, 4.0),
        case("meridian x=0", &bench, x, 0.0),
        case("meridian y=0", &bench, y, 0.0),
        case(
            "meridian oblique",
            &bench,
            Vec3::new(0.3_f64.cos(), 0.3_f64.sin(), 0.0),
            0.0,
        ),
        case("inner tangent x=7", &bench, -x, -7.0),
        case("inner tangent x=-7", &bench, x, -7.0),
        case("outer tangent x=13", &bench, x, 13.0),
        case("oblique notch", &bench, Vec3::new(-0.9, 0.1, 0.42), -5.0),
    ];
    for c in [0.0, 1.5, 3.0, -3.0] {
        for scale in [1.0, 1.0 + 1e-12, 1.0 - 1e-12] {
            cases.push(case(&format!("z={c}*{scale}"), &bench, z, c * scale));
        }
    }
    let spindle = axis_torus(2.0, 3.0);
    cases.push(case("spindle z=0", &spindle, z, 0.0));
    cases.push(case("spindle x=1", &spindle, x, 1.0));
    cases.push(case("spindle x=4", &spindle, x, 4.0));
    cases.push(case(
        "spindle oblique",
        &spindle,
        Vec3::new(0.6, 0.0, 0.8),
        1.2,
    ));
    cases
}

/// Seeded oblique planes through tori with random frames, centres out to
/// `1e4`, scales `1e-3 ..= 1e3`, and non-unit and reversed normals.
fn random_cases(count: usize) -> Vec<Case> {
    let mut rng = SplitMix(0x5EED_70A5_0000_0001);
    (0..count)
        .map(|k| {
            let scale = 10_f64.powf(rng.range(-3.0, 3.0));
            let centre_span = if k % 2 == 0 { 1e4 } else { 10.0 * scale };
            let center = Point3::new(
                rng.range(-centre_span, centre_span),
                rng.range(-centre_span, centre_span),
                rng.range(-centre_span, centre_span),
            );
            let major = scale * rng.range(1.0, 10.0);
            let minor = major * rng.range(0.05, if k % 8 == 0 { 1.5 } else { 0.95 });
            let torus = if k % 3 == 0 {
                ToroidalSurface::with_axis(center, major, minor, rng.direction()).unwrap()
            } else {
                ToroidalSurface::with_axis_and_ref_dir(
                    center,
                    major,
                    minor,
                    rng.direction(),
                    rng.direction(),
                )
                .unwrap()
            };
            let mut normal = rng.direction();
            let reach = major + minor;
            let through = center
                + Vec3::new(
                    rng.range(-reach, reach),
                    rng.range(-reach, reach),
                    rng.range(-reach, reach),
                );
            let mut d = dot_np(normal, through);
            if k % 4 == 1 {
                normal = normal * 3.7;
                d *= 3.7;
            }
            if k % 5 == 2 {
                normal = -normal;
                d = -d;
            }
            Case {
                label: format!("random #{k}"),
                torus,
                normal,
                d,
            }
        })
        .collect()
}

fn crossing_bits(crossings: &[(f64, f64, Point3)]) -> Vec<[u64; 5]> {
    crossings
        .iter()
        .map(|(u, v, p)| {
            [
                u.to_bits(),
                v.to_bits(),
                p.x().to_bits(),
                p.y().to_bits(),
                p.z().to_bits(),
            ]
        })
        .collect()
}

fn curve_bits(curves: &[IntersectionCurve]) -> Vec<Vec<u64>> {
    curves
        .iter()
        .map(|c| {
            let mut bits = vec![c.curve.degree() as u64];
            bits.extend(c.curve.knots().iter().map(|k| k.to_bits()));
            for p in c.curve.control_points() {
                bits.extend([p.x(), p.y(), p.z()].map(f64::to_bits));
            }
            bits.extend(c.curve.weights().iter().map(|w| w.to_bits()));
            for ip in &c.points {
                bits.extend(
                    [
                        ip.point.x(),
                        ip.point.y(),
                        ip.point.z(),
                        ip.param1.0,
                        ip.param1.1,
                        ip.param2.0,
                        ip.param2.1,
                    ]
                    .map(f64::to_bits),
                );
            }
            bits
        })
        .collect()
}

#[test]
fn evaluate_matches_the_pre_change_formula() {
    let mut rng = SplitMix(0x5EED_70A5_0000_0002);
    let tori = [
        axis_torus(10.0, 3.0),
        ToroidalSurface::with_axis(
            Point3::new(1e4, -3e3, 7.5),
            4.0,
            1.25,
            Vec3::new(0.2, -0.5, 0.9),
        )
        .unwrap(),
        ToroidalSurface::with_axis_and_ref_dir(
            Point3::new(-0.3, 2e-3, 1e6),
            2.0e-3,
            7.0e-4,
            Vec3::new(-0.7, 0.1, 0.3),
            Vec3::new(0.4, 0.9, -0.2),
        )
        .unwrap(),
        axis_torus(2.0, 3.0),
    ];
    for torus in &tori {
        for k in 0..5_000 {
            let span = [TAU, 1e3, 1e6, 1e12][k % 4];
            let (u, v) = (rng.range(-span, span), rng.range(-span, span));
            let want = ref_evaluate(torus, u, v);
            let (sin_u, cos_u) = u.sin_cos();
            let (sin_v, cos_v) = v.sin_cos();
            let (tube, height) = torus.tube_terms(sin_v, cos_v);
            for got in [
                torus.evaluate(u, v),
                torus.evaluate_tube(tube, height, sin_u, cos_u),
            ] {
                assert_eq!(
                    [got.x(), got.y(), got.z()].map(f64::to_bits),
                    [want.x(), want.y(), want.z()].map(f64::to_bits),
                    "u={u} v={v}"
                );
            }
        }
    }
}

#[test]
fn grid_crossings_match_the_pre_change_scan() {
    let random = random_cases(64);
    let mut hits = 0;
    for c in named_cases().iter().chain(&random) {
        let want = crossing_bits(&ref_grid_crossings(&c.torus, c.normal, c.d));
        let grid = plane_torus_grid_crossings(&c.torus, c.normal, c.d);
        assert_eq!(crossing_bits(&grid.crossings), want, "{}", c.label);
        hits += usize::from(!want.is_empty());
    }
    assert!(hits >= 60, "only {hits} cases crossed the torus");
}

#[test]
fn intersect_plane_torus_matches_the_pre_change_pipeline() {
    let mut nonempty = 0;
    for c in named_cases().iter().chain(&random_cases(64)[..8]) {
        let want = ref_intersect_plane_torus(&c.torus, c.normal, c.d).unwrap();
        let got = intersect_plane_torus(&c.torus, c.normal, c.d).unwrap();
        assert_eq!(curve_bits(&got), curve_bits(&want), "{}", c.label);
        nonempty += usize::from(!want.is_empty());
    }
    assert!(nonempty >= 20, "only {nonempty} cases produced curves");
}

#[test]
fn grid_trig_is_tabulated() {
    let c = &named_cases()[0];
    let grid = plane_torus_grid_crossings(&c.torus, c.normal, c.d);
    // Both ends of each of the 128 cells; `u` and `v` share the table.
    assert_eq!(grid.grid_sin_cos, 2 * 128);
}
