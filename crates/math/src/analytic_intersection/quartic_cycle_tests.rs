//! The Durand–Kerner cycle jump in `real_roots_quartic` must reproduce the
//! previous solver bit for bit: `intersect_line_torus` is compared against a
//! verbatim copy of the pre-jump algorithm (which always runs the sweep
//! budget out) on rays that hit, graze and miss tori of several proportions
//! and scales.

#![allow(
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::suboptimal_flops
)]

use super::*;

/// Deterministic low-discrepancy sample in [0, 1).
fn halton(mut i: u32, base: u32) -> f64 {
    let (mut f, mut r) = (1.0_f64, 0.0_f64);
    while i > 0 {
        f /= f64::from(base);
        r += f * f64::from(i % base);
        i /= base;
    }
    r
}

#[derive(Default)]
struct ReferenceStats {
    solves: usize,
    /// Solves that ran the whole sweep budget.
    exhausted: usize,
    /// Exhausted solves that revisited a state with period one.
    fixed: usize,
    /// Exhausted solves that revisited a state with a longer period.
    cycled: usize,
}

/// The complex arithmetic of the pre-jump solver, verbatim.
#[derive(Clone, Copy)]
struct RefComplex {
    re: f64,
    im: f64,
}

impl RefComplex {
    const fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }
    fn norm(self) -> f64 {
        self.re.hypot(self.im)
    }
    fn add(self, o: Self) -> Self {
        Self::new(self.re + o.re, self.im + o.im)
    }
    fn sub(self, o: Self) -> Self {
        Self::new(self.re - o.re, self.im - o.im)
    }
    fn mul(self, o: Self) -> Self {
        Self::new(
            self.re.mul_add(o.re, -(self.im * o.im)),
            self.re.mul_add(o.im, self.im * o.re),
        )
    }
    fn div(self, o: Self) -> Self {
        let den = o.re.mul_add(o.re, o.im * o.im);
        Self::new(
            self.re.mul_add(o.re, self.im * o.im) / den,
            self.im.mul_add(o.re, -(self.re * o.im)) / den,
        )
    }
    fn bits(self) -> (u64, u64) {
        (self.re.to_bits(), self.im.to_bits())
    }
}

/// `real_roots_quartic` before the cycle jump, verbatim apart from the
/// statistics it records.
fn reference_real_roots_quartic(
    c4: f64,
    c3: f64,
    c2: f64,
    c1: f64,
    c0: f64,
    stats: &mut ReferenceStats,
) -> Vec<f64> {
    if c4.abs() < 1e-14 {
        return real_roots_cubic(c3, c2, c1, c0);
    }
    stats.solves += 1;
    let (a, b, c, d) = (c3 / c4, c2 / c4, c1 / c4, c0 / c4);
    let eval = |z: RefComplex| -> RefComplex {
        let mut acc = RefComplex::new(1.0, 0.0);
        acc = acc.mul(z).add(RefComplex::new(a, 0.0));
        acc = acc.mul(z).add(RefComplex::new(b, 0.0));
        acc = acc.mul(z).add(RefComplex::new(c, 0.0));
        acc.mul(z).add(RefComplex::new(d, 0.0))
    };
    let seed = RefComplex::new(0.4, 0.9);
    let mut r = [
        RefComplex::new(1.0, 0.0),
        seed,
        seed.mul(seed),
        seed.mul(seed).mul(seed),
    ];
    let mut seen = vec![r.map(RefComplex::bits)];
    let mut period = None;
    let mut converged = false;
    for _ in 0..100 {
        let mut max_step = 0.0_f64;
        for i in 0..4 {
            let mut denom = RefComplex::new(1.0, 0.0);
            for j in 0..4 {
                if i != j {
                    denom = denom.mul(r[i].sub(r[j]));
                }
            }
            if denom.norm() < 1e-300 {
                continue;
            }
            let step = eval(r[i]).div(denom);
            r[i] = r[i].sub(step);
            max_step = max_step.max(step.norm());
        }
        if max_step < 1e-14 {
            converged = true;
            break;
        }
        let state = r.map(RefComplex::bits);
        if period.is_none()
            && let Some(first) = seen.iter().position(|s| *s == state)
        {
            period = Some(seen.len() - first);
        }
        seen.push(state);
    }
    if !converged {
        stats.exhausted += 1;
        match period {
            Some(1) => stats.fixed += 1,
            Some(_) => stats.cycled += 1,
            None => {}
        }
    }
    let p_real = |x: f64| -> f64 { (((x + a) * x + b) * x + c) * x + d };
    let mut out: Vec<f64> = Vec::new();
    for z in r {
        if z.im.abs() >= 1e-7 {
            continue;
        }
        let x = z.re;
        let scale = 1.0 + a.abs() + b.abs() + c.abs() + d.abs() + x.abs().powi(4);
        if p_real(x).abs() > 1e-6 * scale {
            continue;
        }
        if out.iter().any(|&y| (y - x).abs() < 1e-9 * (1.0 + x.abs())) {
            continue;
        }
        out.push(x);
    }
    out
}

/// `intersect_line_torus` before the cycle jump, verbatim.
fn reference_intersect_line_torus(
    torus: &ToroidalSurface,
    origin: Point3,
    dir: Vec3,
    stats: &mut ReferenceStats,
) -> Vec<f64> {
    let c = torus.center();
    let (xa, ya, za) = (torus.x_axis(), torus.y_axis(), torus.z_axis());
    let big_r = torus.major_radius();
    let small_r = torus.minor_radius();
    let o = Vec3::new(origin.x() - c.x(), origin.y() - c.y(), origin.z() - c.z());
    let (a0, a1) = (xa.dot(o), xa.dot(dir));
    let (b0, b1) = (ya.dot(o), ya.dot(dir));
    let (c0, c1) = (za.dot(o), za.dot(dir));
    let g2 = a1.mul_add(a1, b1.mul_add(b1, c1 * c1));
    let g1 = 2.0 * a1.mul_add(a0, b1.mul_add(b0, c1 * c0));
    let g0 = a0.mul_add(
        a0,
        b0.mul_add(b0, c0.mul_add(c0, big_r.mul_add(big_r, -small_r * small_r))),
    );
    let four_rr = 4.0 * big_r * big_r;
    let h2 = four_rr * a1.mul_add(a1, b1 * b1);
    let h1 = four_rr * (2.0 * a1.mul_add(a0, b1 * b0));
    let h0 = four_rr * a0.mul_add(a0, b0 * b0);
    let e4 = g2 * g2;
    let e3 = 2.0 * g2 * g1;
    let e2 = g1.mul_add(g1, 2.0 * g2 * g0) - h2;
    let e1 = 2.0f64.mul_add(g1 * g0, -h1);
    let e0 = g0.mul_add(g0, -h0);
    let mut roots = reference_real_roots_quartic(e4, e3, e2, e1, e0, stats);
    let impl_f = |t: f64| -> f64 {
        let p = origin + dir * t;
        let q = Vec3::new(p.x() - c.x(), p.y() - c.y(), p.z() - c.z());
        let (a, b, cc) = (xa.dot(q), ya.dot(q), za.dot(q));
        (a.hypot(b) - big_r).hypot(cc) - small_r
    };
    for t in &mut roots {
        let eps = 1e-7;
        let f = impl_f(*t);
        let df = (impl_f(*t + eps) - impl_f(*t - eps)) / (2.0 * eps);
        if df.abs() > 1e-12 {
            *t -= f / df;
        }
    }
    roots.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    roots
}

fn bits(ts: &[f64]) -> Vec<u64> {
    ts.iter().map(|t| t.to_bits()).collect()
}

/// A unit vector from two samples in [0, 1).
fn unit(s: f64, t: f64) -> Vec3 {
    let z = 2.0f64.mul_add(s, -1.0);
    let phi = std::f64::consts::TAU * t;
    let rho = (1.0 - z * z).max(0.0).sqrt();
    Vec3::new(rho * phi.cos(), rho * phi.sin(), z)
}

#[test]
fn cycle_jump_matches_the_full_budget_solver_bit_for_bit() {
    // (major, minor, centre, axis): ring tori from fillet-corner proportions
    // to fat ones, at the origin and at the hammer holder's model scale.
    let tori = [
        (
            10.0,
            2.0,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ),
        (
            3.0,
            2.5,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ),
        (
            40.0,
            1.5,
            Point3::new(120.0, -35.0, 60.0),
            Vec3::new(0.3, -0.5, 0.81),
        ),
        (
            6.0,
            0.75,
            Point3::new(-80.0, 210.0, 15.0),
            Vec3::new(1.0, 0.0, 0.0),
        ),
    ];
    let mut stats = ReferenceStats::default();
    let mut hits = 0_usize;
    let mut sample = 0_u32;
    for &(major, minor, center, axis) in &tori {
        let torus =
            ToroidalSurface::with_axis(center, major, minor, axis.normalize().unwrap()).unwrap();
        let reach = major + minor;
        for k in 0..900_u32 {
            sample += 1;
            let i = sample;
            // Origins in a box around the torus; directions towards a point
            // on (or near) the tube so many rays hit or graze, plus
            // unconstrained directions that mostly miss.
            let origin = center
                + Vec3::new(
                    (halton(i, 2) - 0.5) * 6.0 * reach,
                    (halton(i, 3) - 0.5) * 6.0 * reach,
                    (halton(i, 5) - 0.5) * 6.0 * reach,
                );
            let (u, v) = (
                std::f64::consts::TAU * halton(i, 7),
                std::f64::consts::TAU * halton(i, 11),
            );
            let dir = match k % 4 {
                // Through a surface point.
                0 => torus.evaluate(u, v) - origin,
                // Through a point just off the tube: grazes and near-misses.
                1 => {
                    let p = torus.evaluate(u, v);
                    let n = torus.normal(u, v);
                    (p + n * (minor * 1e-3 * (halton(i, 13) - 0.5))) - origin
                }
                // Tangent to the tube at a surface point.
                2 => {
                    let n = torus.normal(u, v);
                    let t = unit(halton(i, 13), halton(i, 17));
                    let tangent = t - n * t.dot(n);
                    match tangent.normalize() {
                        Ok(tangent) => {
                            let p = torus.evaluate(u, v);
                            // Start on the tangent line, one tube width away.
                            let start = p - tangent * (2.0 * minor);
                            let out = intersect_line_torus(&torus, start, tangent);
                            let reference =
                                reference_intersect_line_torus(&torus, start, tangent, &mut stats);
                            assert_eq!(bits(&out), bits(&reference), "tangent ray {i}");
                            hits += usize::from(!out.is_empty());
                            continue;
                        }
                        Err(_) => unit(halton(i, 13), halton(i, 17)),
                    }
                }
                // Any direction, at a non-unit length.
                _ => unit(halton(i, 13), halton(i, 17)) * (0.25 + 4.0 * halton(i, 19)),
            };
            let out = intersect_line_torus(&torus, origin, dir);
            let reference = reference_intersect_line_torus(&torus, origin, dir, &mut stats);
            assert_eq!(
                bits(&out),
                bits(&reference),
                "ray {i}: {origin:?} + t {dir:?}"
            );
            hits += usize::from(!out.is_empty());
        }
    }
    // Non-vacuous: rays hit, and solves ran the budget out both in fixed
    // points and longer cycles (the states the jump shortcuts) and without
    // ever repeating (which the jump must leave alone).
    assert!(hits > 1000, "only {hits} rays hit");
    assert!(
        stats.fixed > 0,
        "no fixed-point Durand-Kerner run exercised"
    );
    assert!(
        stats.cycled > 0,
        "no longer-period Durand-Kerner cycle exercised"
    );
    assert!(
        stats.exhausted > stats.fixed + stats.cycled,
        "no budget-exhausting run without a repeat exercised"
    );
}

/// The cycle jump lands where running out the budget would: simulate every
/// `(first, updates)` cycle shape step by step.
#[test]
fn durand_kerner_budget_end_matches_running_out_the_budget() {
    for updates in 1..=DURAND_KERNER_SWEEPS {
        for first in 0..updates {
            // Sequence of state indices: k for k < updates, then periodic.
            let period = updates - first;
            let mut state = 0_usize;
            for _ in 0..DURAND_KERNER_SWEEPS {
                state = if state + 1 == updates {
                    first
                } else {
                    state + 1
                };
            }
            let expected = state;
            assert_eq!(
                durand_kerner_budget_end(first, updates),
                expected,
                "first {first}, period {period}"
            );
        }
    }
}
