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
/// figure-eight), a spindle torus, and inputs outside the range the line
/// certificates accept.
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
    cases.push(case(
        "subnormal normal component",
        &bench,
        Vec3::new(-1.0, 1e-310, 0.0),
        -6.0,
    ));
    cases.push(case("tiny torus", &axis_torus(1e-59, 3e-60), -x, -6e-60));
    cases.push(case("huge torus", &axis_torus(1e59, 3e58), -x, -6e58));
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
        for certify in [false, true] {
            let grid = plane_torus_grid_crossings(&c.torus, c.normal, c.d, certify);
            assert_eq!(
                crossing_bits(&grid.crossings),
                want,
                "{} (certify: {certify})",
                c.label
            );
        }
        hits += usize::from(!want.is_empty());
    }
    assert!(hits >= 60, "only {hits} cases crossed the torus");
}

/// Every computed sample on a certified line has the certified sign. The
/// samples come from the oracle's own formula, which matches the scan's
/// bit for bit (`evaluate_matches_the_pre_change_formula`).
#[test]
fn certified_lines_hold_on_every_sample() {
    let (mut certified, mut lines) = (0, 0);
    for c in named_cases().iter().chain(&random_cases(64)) {
        let nodes: Vec<GridNode> = plane_torus_grid_nodes(&c.torus, c.normal, c.d, true)
            .into_iter()
            .flat_map(|(lo, hi)| [lo, hi])
            .collect();
        for u in &nodes {
            for v in &nodes {
                let f = dot_np(c.normal, ref_evaluate(&c.torus, u.angle, v.angle)) - c.d;
                for sign in [u.col, v.row] {
                    assert!(
                        sign == 0 || f * f64::from(sign) > 0.0,
                        "{}: f={f} at u={} v={} certified {sign}",
                        c.label,
                        u.angle,
                        v.angle
                    );
                }
            }
        }
        lines += 2 * nodes.len();
        certified += nodes
            .iter()
            .map(|n| usize::from(n.col != 0) + usize::from(n.row != 0))
            .sum::<usize>();
    }
    assert!(
        certified * 3 >= lines,
        "only {certified} of {lines} lines certified"
    );
}

#[test]
fn bounds_refuse_inputs_outside_their_error_budget() {
    let bench = axis_torus(10.0, 3.0);
    let x = Vec3::new(1.0, 0.0, 0.0);
    let nodes = plane_torus_grid_nodes(&bench, -x, -6.0, false);
    assert!(GridLineBounds::new(&bench, -x, -6.0, &nodes).is_some());
    for (torus, normal, d) in [
        (&bench, Vec3::new(-1.0, 1e-310, 0.0), -6.0),
        (&bench, Vec3::new(-1.0, 1e-51, 0.0), -6.0),
        (&bench, Vec3::new(-1e51, 0.0, 0.0), -6.0),
        (&bench, -x, f64::INFINITY),
        (&bench, Vec3::new(f64::NAN, 0.0, 0.0), -6.0),
        (&axis_torus(1e-59, 3e-60), -x, -6e-60),
        (&axis_torus(1e59, 3e58), -x, -6e58),
        // In range term by term, but `S = 0`.
        (&bench, Vec3::new(0.0, 0.0, 0.0), 0.0),
    ] {
        assert!(
            GridLineBounds::new(torus, normal, d, &nodes).is_none(),
            "normal={normal:?} d={d}"
        );
    }
    // A table whose trig left the unit circle.
    let mut off_circle = nodes;
    off_circle[7].1.cos *= 1.000_001;
    assert!(GridLineBounds::new(&bench, -x, -6.0, &off_circle).is_none());
}

/// Work guard: the certificates leave under a quarter of the cells to
/// sample on the `torus_notch_*` box faces, and the trig is tabulated.
#[test]
fn certificates_skip_most_cells_of_the_notch_planes() {
    for c in &named_cases()[..3] {
        let grid = plane_torus_grid_crossings(&c.torus, c.normal, c.d, true);
        assert!(
            grid.sampled_cells * 4 <= 128 * 128,
            "{}: {} cells sampled",
            c.label,
            grid.sampled_cells
        );
        // Both ends of each of the 128 cells; `u` and `v` share the table.
        assert_eq!(grid.grid_sin_cos, 2 * 128);
        let full = plane_torus_grid_crossings(&c.torus, c.normal, c.d, false);
        assert_eq!(full.sampled_cells, 128 * 128);
    }
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

/// The pre-change chaining loop of `intersect_plane_torus`, verbatim, with
/// every chain kept.
fn ref_greedy_chains(crossing_pts: &[(f64, f64, Point3)], radius: f64) -> Vec<Vec<usize>> {
    let mut used = vec![false; crossing_pts.len()];
    let mut chains = Vec::new();

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
            let mut best_dist = radius;

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
        chains.push(chain);
    }
    chains
}

/// Crossings `(u, v, point)` as the chaining reads them.
type Cloud = Vec<(f64, f64, Point3)>;

fn cloud(points: impl IntoIterator<Item = [f64; 3]>) -> Cloud {
    points
        .into_iter()
        .map(|[x, y, z]| (0.0, 0.0, Point3::new(x, y, z)))
        .collect()
}

fn lattice(spacing: f64) -> Vec<[f64; 3]> {
    (0..72)
        .map(|i| {
            let (a, b, c) = (i % 6, (i / 6) % 6, i / 36);
            [
                f64::from(a) * spacing,
                f64::from(b) * spacing,
                f64::from(c) * spacing,
            ]
        })
        .collect()
}

/// Seeded point clouds (uniform, clustered, on circles, with duplicates and
/// exact-tie lattices) and adversarial ones: points exactly at the radius
/// and at the prefilter gate and their neighbouring floats, non-finite
/// coordinates, tiny scales where the gate is off (including squares that
/// underflow to zero), and zero, negative, NaN and infinite radii.
fn chain_clouds() -> Vec<(String, Cloud, f64)> {
    let mut rng = SplitMix(0x5EED_70A5_0000_0003);
    let mut clouds = Vec::new();
    for n in [0, 1, 2, 3, 17, 500] {
        let pts = (0..n)
            .map(|_| [(); 3].map(|()| rng.range(-1.0, 1.0)))
            .collect::<Vec<_>>();
        clouds.push((format!("uniform {n}"), cloud(pts), 0.2));
    }
    let clustered = (0..300)
        .map(|i| {
            let centre = f64::from(i % 10);
            [(); 3].map(|()| centre + rng.range(-1e-3, 1e-3))
        })
        .collect::<Vec<_>>();
    clouds.push(("clustered".to_owned(), cloud(clustered), 0.05));
    for (scale, centre) in [(1e-3, 0.0), (1.0, 0.0), (1e3, 0.0), (1.0, 1e4)] {
        let circles = (0..300)
            .map(|i| {
                let t = TAU * f64::from(i % 100) / 100.0 + rng.range(0.0, 1e-3);
                let ring = 1.0 + f64::from(i / 100);
                [
                    scale * ring * t.cos() + centre,
                    scale * ring * t.sin() + centre,
                    scale * ring * 0.1 + centre,
                ]
            })
            .collect::<Vec<_>>();
        clouds.push((
            format!("circles at {scale} around {centre}"),
            cloud(circles),
            scale * 0.1,
        ));
    }
    let duplicates = (0..60)
        .map(|i| [f64::from(i / 3) * 0.1, 0.0, 0.0])
        .collect::<Vec<_>>();
    clouds.push(("duplicates".to_owned(), cloud(duplicates), 0.3));
    clouds.push(("lattice ties".to_owned(), cloud(lattice(1.0)), 1.5));
    clouds.push(("lattice at radius".to_owned(), cloud(lattice(1.0)), 1.0));

    let radius = 0.75_f64;
    let gate = radius * (1.0 + 1e-12);
    let mut edges = vec![[0.0; 3]];
    for v in [
        gate,
        gate.next_up(),
        gate.next_down(),
        radius,
        radius.next_up(),
        radius.next_down(),
    ] {
        edges.extend([[v, 0.0, 0.0], [0.0, -v, 0.0], [0.0, 0.0, v]]);
    }
    clouds.push(("radius and gate edges".to_owned(), cloud(edges), radius));

    let mut non_finite = lattice(0.5);
    non_finite[3] = [f64::NAN, 0.0, 0.0];
    non_finite[10] = [0.5, f64::INFINITY, 0.5];
    non_finite[20] = [f64::NEG_INFINITY, f64::NAN, 1.0];
    non_finite[40] = [1.0, 1.0, f64::NAN];
    clouds.push(("non-finite".to_owned(), cloud(non_finite), 0.8));

    for scale in [1e-160, 1e-163] {
        let tiny = lattice(1.0).into_iter().map(|p| p.map(|c| c * scale));
        clouds.push((format!("lattice at {scale}"), cloud(tiny), 1.5 * scale));
    }
    // Neighbours' squared differences round to zero: their distance is 0.
    let underflow = (0..20).map(|i| [f64::from(i) * 1.2e-162, 0.0, 0.0]);
    clouds.push(("subnormal squares".to_owned(), cloud(underflow), 1e-162));

    let uniform = clouds[5].1.clone();
    for radius in [0.0, -0.2, f64::NAN, f64::INFINITY] {
        clouds.push((format!("radius {radius}"), uniform.clone(), radius));
    }
    clouds
}

#[test]
fn greedy_chains_match_the_pre_change_scan() {
    for (label, pts, radius) in chain_clouds() {
        assert_eq!(
            greedy_chains(&pts, radius),
            ref_greedy_chains(&pts, radius),
            "{label}"
        );
    }
}
