//! Differential oracle for the hob sweep: the depth and hobbed-edge search
//! as they were before the sweep stopped at the first cutting wheel angle,
//! skipped samples under their blank margin and kept only its four largest
//! samples, kept verbatim. The edges, depths and cut answers must match bit
//! for bit, and the search must evaluate fewer margins.

#![allow(clippy::unwrap_used, clippy::panic)]

use std::cell::Cell;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

use remus_math::compact_sort::CompactSort;

use super::super::Hand;
use super::*;

thread_local! {
    /// Material margins the verbatim sweep evaluated on this thread.
    static OLD_EVALS: Cell<u64> = const { Cell::new(0) };
}

/// `Hindley::line_angle` before the change, verbatim.
fn old_line_angle(h: &Hindley, side: ToothSide, rho: f64) -> f64 {
    let off = rho.mul_add(rho, -h.rb * h.rb).max(0.0).sqrt().atan2(h.rb);
    match side {
        ToothSide::Plus => h.tangent_plus - off,
        ToothSide::Minus => h.tangent_minus + off,
    }
}

/// `Hindley::material_margin` before the change, verbatim.
fn old_material_margin(h: &Hindley, p: Point3) -> f64 {
    OLD_EVALS.with(|evals| evals.set(evals.get() + 1));
    let u = p.y().atan2(p.x());
    let r = p.x().hypot(p.y());
    let (y, z) = (r - h.a, p.z());
    let rho = y.hypot(z);
    let blank = (h.half - z.abs()).min(rho - h.rho_tip);
    let phi = (FRAC_PI_2 + h.phase - u) / (h.sigma * h.ratio);
    let pitch = TAU / f64::from(h.wheel_teeth);
    let angle = z.atan2(y) - phi;
    let radius = rho.max(h.rb);
    let lo = old_line_angle(h, ToothSide::Minus, radius);
    let width = old_line_angle(h, ToothSide::Plus, radius) - lo;
    let offset = (angle - lo).rem_euclid(pitch);
    let angular = if offset <= width {
        -offset.min(width - offset)
    } else {
        (offset - width).min(pitch - offset)
    };
    blank.min((angular * rho).max(rho - h.rho_root))
}

/// `HobSweep::depth` before the change, verbatim.
fn old_depth(hob: &HobSweep, w: Point3) -> f64 {
    let theta = w.z().atan2(w.y()).rem_euclid(2.0 * PI);
    let center = PI - theta;
    let (lo, hi) = (center - hob.reach, center + hob.reach);
    let count = ((hi - lo) * hob.density).ceil().max(8.0) as usize;
    let step = (hi - lo) / count as f64;
    let margin = |phi: f64| old_material_margin(&hob.hindley, hob.to_worm(w, phi));
    let mut samples: Vec<(f64, f64)> = (0..=count)
        .map(|i| {
            let phi = lo + step * i as f64;
            (phi, margin(phi))
        })
        .collect();
    // Refine around the best few samples by golden-section search: the
    // margin is piecewise smooth, so the maximum sits near one of them.
    samples.compact_sort_by(|a, b| b.1.total_cmp(&a.1));
    let mut best = samples[0].1;
    for &(phi, _) in samples.iter().take(4) {
        let (mut a, mut b) = (phi - step, phi + step);
        let ratio = 0.5 * (5.0_f64.sqrt() - 1.0);
        let mut c = b - ratio * (b - a);
        let mut d = a + ratio * (b - a);
        let (mut fc, mut fd) = (margin(c), margin(d));
        for _ in 0..40 {
            if fc > fd {
                b = d;
                d = c;
                fd = fc;
                c = b - ratio * (b - a);
                fc = margin(c);
            } else {
                a = c;
                c = d;
                fc = fd;
                d = a + ratio * (b - a);
                fd = margin(d);
            }
        }
        best = best.max(fc).max(fd);
    }
    best
}

/// `HobSweep::edge` before the change, verbatim.
fn old_edge(hob: &HobSweep, side: ToothSide, s: f64, rho: f64) -> HobEdge {
    let at = |theta: f64| {
        let (sin, cos) = theta.sin_cos();
        old_depth(hob, Point3::new(s, rho * cos, rho * sin))
    };
    // Bracket between the tooth's center (θ = π), which the hob must
    // leave, and the center of the space beside this flank, which it
    // cuts wherever it reaches at all.
    let pitch = 2.0 * PI / f64::from(hob.hindley.wheel_teeth);
    let (mut cut, mut kept) = match side {
        ToothSide::Minus => (PI - 0.5 * pitch, PI),
        ToothSide::Plus => (PI + 0.5 * pitch, PI),
    };
    if at(kept) > 0.0 {
        return HobEdge::Blocked;
    }
    if at(cut) <= 0.0 {
        return HobEdge::Clear;
    }
    for _ in 0..50 {
        let mid = 0.5 * (cut + kept);
        if at(mid) > 0.0 {
            cut = mid;
        } else {
            kept = mid;
        }
    }
    HobEdge::At(0.5 * (cut + kept))
}

/// An edge as comparable bits.
fn edge_bits(edge: HobEdge) -> (u8, u64) {
    match edge {
        HobEdge::At(angle) => (0, angle.to_bits()),
        HobEdge::Clear => (1, 0),
        HobEdge::Blocked => (2, 0),
    }
}

/// The probe worm and variants: two starts, left hand, another module and
/// tooth count, and a worm short enough to leave edges clear.
fn worms() -> Vec<EnvelopingWormParams> {
    vec![
        EnvelopingWormParams::standard(2.0, 1, 30, 10.0, 24.0),
        EnvelopingWormParams::standard(2.0, 2, 30, 10.0, 24.0),
        EnvelopingWormParams {
            hand: Hand::Left,
            ..EnvelopingWormParams::standard(2.0, 1, 30, 10.0, 24.0)
        },
        EnvelopingWormParams::standard(1.5, 1, 41, 8.0, 14.0),
        EnvelopingWormParams::standard(2.0, 1, 30, 10.0, 6.0),
    ]
}

/// Heights and radii as fractions of half the face width and of the tooth
/// height: on the mid-plane (where every wheel angle's margin is near
/// zero), beside it, across the throat and out where the hob runs into the
/// tooth.
const HEIGHTS: [f64; 6] = [-1.0, -0.4375, 0.0, 0.125, 0.75, 1.5];
const RADII: [f64; 3] = [0.0, 0.5, 1.0];

/// Compares every edge of one worm's grid with the verbatim search; the
/// margins the old and new searches evaluated, and the edges by kind.
fn compare_edges(worm: &EnvelopingWormParams, heights: &[f64]) -> (u64, u64, [usize; 3]) {
    let hob = HobSweep::new(worm);
    let r2 = 0.5 * worm.module * f64::from(worm.wheel_teeth);
    let root = worm.dedendum.mul_add(-worm.module, r2);
    let outside = (worm.addendum + 0.5).mul_add(worm.module, r2);
    let half = 2.0 * worm.module;
    OLD_EVALS.with(|evals| evals.set(0));
    MARGIN_EVALS.with(|evals| evals.set(0));
    let mut kinds = [0; 3];
    for &height in heights {
        for fraction in RADII {
            let (s, rho) = (half * height, root + (outside - root) * fraction);
            for side in [ToothSide::Minus, ToothSide::Plus] {
                let old = edge_bits(old_edge(&hob, side, s, rho));
                let new = edge_bits(hob.edge(side, s, rho));
                assert_eq!(new, old, "{worm:?} {side:?} s {s} rho {rho}");
                kinds[usize::from(new.0)] += 1;
            }
        }
    }
    (
        OLD_EVALS.with(Cell::get),
        MARGIN_EVALS.with(Cell::get),
        kinds,
    )
}

#[test]
fn hobbed_edges_match_the_full_sweep_bit_for_bit() {
    let mut kinds = [0; 3];
    for worm in worms() {
        let (_, _, found) = compare_edges(&worm, &HEIGHTS);
        for (total, count) in kinds.iter_mut().zip(found) {
            *total += count;
        }
    }
    // Every outcome of the search is exercised.
    assert!(kinds.iter().all(|&count| count > 0), "{kinds:?}");
}

#[test]
fn hob_depths_and_cut_answers_match_the_full_sweep_bit_for_bit() {
    for worm in worms() {
        let hob = HobSweep::new(&worm);
        let r2 = 0.5 * worm.module * f64::from(worm.wheel_teeth);
        let pitch = TAU / f64::from(worm.wheel_teeth);
        for height in HEIGHTS {
            for radial in [-1.25_f64, 0.0, 1.0, 1.5] {
                for turn in [-0.5_f64, -0.26, -0.24, -0.05, 0.0, 0.1, 0.25, 0.4] {
                    let (s, rho) = (2.0 * worm.module * height, radial.mul_add(worm.module, r2));
                    let theta = turn.mul_add(pitch, PI);
                    let w = Point3::new(s, rho * theta.cos(), rho * theta.sin());
                    let old = old_depth(&hob, w);
                    assert!(!old.is_nan());
                    assert_eq!(
                        hob.depth(w).to_bits(),
                        old.to_bits(),
                        "{worm:?} s {s} rho {rho} theta {theta}"
                    );
                    assert_eq!(hob.cuts(w), old > 0.0, "{worm:?} s {s} rho {rho} {theta}");
                }
            }
        }
    }
}

#[test]
fn the_edge_search_evaluates_a_quarter_fewer_margins() {
    // Deterministic work counts, not wall time: off the mid-plane on the
    // probe worm, the heights the flank fit samples.
    let (old, new, _) = compare_edges(&worms()[0], &[-1.0, -0.4375, 0.125, 0.75]);
    assert!(new * 4 <= old * 3, "{new} of {old} margins");
}

/// Every edge the flank fit of the probe wheel (face width 8) asks for.
#[test]
#[ignore = "about ten seconds: the verbatim search over the whole fit grid"]
fn the_probe_wheels_fit_grid_matches_the_full_sweep_bit_for_bit() {
    let worm = &worms()[0];
    let hob = HobSweep::new(worm);
    let (a, half, root, tip, outside) = (40.0_f64, 4.0, 27.5, 32.0_f64, 33.0_f64);
    let tube = a - tip;
    let half_throat = tube.mul_add(tube, -(a - outside).powi(2)).sqrt();
    let mut edges = 0;
    for i in 0..=16 {
        let s = half * (2.0 * f64::from(i) / 16.0 - 1.0);
        if s.abs() < 1e-9 * half {
            continue;
        }
        let top = if s.abs() < half_throat {
            a - s.mul_add(-s, tube * tube).sqrt()
        } else {
            outside
        };
        for j in 0..=8 {
            let rho = root + (top - root) * f64::from(j) / 8.0;
            for side in [ToothSide::Minus, ToothSide::Plus] {
                let old = edge_bits(old_edge(&hob, side, s, rho));
                assert_eq!(edge_bits(hob.edge(side, s, rho)), old, "{side:?} {s} {rho}");
                edges += usize::from(old.0 == 0);
            }
        }
    }
    assert!(edges > 200, "{edges} edges");
}
