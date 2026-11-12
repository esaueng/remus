//! Differential oracle for the hob sweep: the depth and hobbed-edge search
//! as they were before the sweep skipped samples under their blank margin,
//! kept only its four largest samples and was tabulated once per edge, kept
//! verbatim.
//!
//! A point's depth still matches bit for bit. The hobbed edges cannot: the
//! verbatim search lays its wheel-angle grid `lo + step·i` out again for
//! every candidate polar angle `θ` from `lo = (π − θ) − reach`, while the
//! table lays one grid out from `π − reach` and adds `θ` to each sample's
//! lead. The two reach the same angles `ψ = θ + φ` only up to rounding, so
//! margins differ in their last bits, and a bisection round whose depth is
//! within that of zero can go the other way. The edges are held to
//! [`MAX_EDGE_ULPS`], and the built wheel to [`MAX_DISPLACEMENT`].

#![allow(clippy::unwrap_used, clippy::panic)]

use std::cell::Cell;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

use remus_math::compact_sort::CompactSort;

use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};

use super::super::Hand;
use super::*;
use crate::measure::solid_volume;

thread_local! {
    /// Material margins the verbatim sweep evaluated on this thread.
    static OLD_EVALS: Cell<u64> = const { Cell::new(0) };
}

thread_local! {
    /// Whether `HobSweep::edge` runs the verbatim search on this thread.
    static VERBATIM: Cell<bool> = const { Cell::new(false) };
}

/// Whether `HobSweep::edge` is to run [`old_edge`] instead.
pub(super) fn verbatim_sweep() -> bool {
    VERBATIM.with(Cell::get)
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
pub(super) fn old_edge(hob: &HobSweep, side: ToothSide, s: f64, rho: f64) -> HobEdge {
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

/// The largest distance, in units in the last place, between a tabulated
/// edge angle and the verbatim one. Measured: 2 on the five worms' grids
/// and 4 on the probe wheel's whole fit grid (edge angles are
/// near π, where one unit is 4.4e-16).
const MAX_EDGE_ULPS: u64 = 16;

/// The largest displacement of a vertex or NURBS control point of a built
/// wheel between the tabulated and the verbatim search, in model units.
/// Measured: 1.2e-13 to 1.5e-13 on the four wheels below, with flank law
/// coefficients moving by at most 8.9e-14, the relief by 1.5e-13 and the
/// volume by 2.2e-14 of itself; the kernel's linear tolerance is 1e-7.
const MAX_DISPLACEMENT: f64 = 1e-12;

/// The outcome of the two searches must be the same; the distance between
/// their edge angles in units in the last place.
fn edge_ulps(new: HobEdge, old: HobEdge, what: &str) -> u64 {
    match (new, old) {
        (HobEdge::At(new), HobEdge::At(old)) => new.to_bits().abs_diff(old.to_bits()),
        (HobEdge::Clear, HobEdge::Clear) | (HobEdge::Blocked, HobEdge::Blocked) => 0,
        _ => panic!("{what}: {new:?} against the verbatim {old:?}"),
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

/// What comparing one grid of edges with the verbatim search found.
struct EdgeComparison {
    /// Margins the verbatim and the tabulated searches evaluated.
    old_evals: u64,
    new_evals: u64,
    /// Edges found, clear and blocked.
    kinds: [usize; 3],
    max_ulps: u64,
}

/// Compares every edge of one worm's grid with the verbatim search.
fn compare_edges(worm: &EnvelopingWormParams, heights: &[f64]) -> EdgeComparison {
    let hob = HobSweep::new(worm);
    let r2 = 0.5 * worm.module * f64::from(worm.wheel_teeth);
    let root = worm.dedendum.mul_add(-worm.module, r2);
    let outside = (worm.addendum + 0.5).mul_add(worm.module, r2);
    let half = 2.0 * worm.module;
    OLD_EVALS.with(|evals| evals.set(0));
    MARGIN_EVALS.with(|evals| evals.set(0));
    let mut kinds = [0; 3];
    let mut max_ulps = 0;
    for &height in heights {
        for fraction in RADII {
            let (s, rho) = (half * height, root + (outside - root) * fraction);
            for side in [ToothSide::Minus, ToothSide::Plus] {
                let old = old_edge(&hob, side, s, rho);
                let new = hob.edge(side, s, rho);
                let what = format!("{worm:?} {side:?} s {s} rho {rho}");
                max_ulps = max_ulps.max(edge_ulps(new, old, &what));
                kinds[match new {
                    HobEdge::At(_) => 0,
                    HobEdge::Clear => 1,
                    HobEdge::Blocked => 2,
                }] += 1;
            }
        }
    }
    EdgeComparison {
        old_evals: OLD_EVALS.with(Cell::get),
        new_evals: MARGIN_EVALS.with(Cell::get),
        kinds,
        max_ulps,
    }
}

#[test]
fn hobbed_edges_stay_within_ulps_of_the_full_sweep() {
    let mut kinds = [0; 3];
    let mut max_ulps = 0;
    for worm in worms() {
        let found = compare_edges(&worm, &HEIGHTS);
        for (total, count) in kinds.iter_mut().zip(found.kinds) {
            *total += count;
        }
        max_ulps = max_ulps.max(found.max_ulps);
    }
    // Every outcome of the search is exercised.
    assert!(kinds.iter().all(|&count| count > 0), "{kinds:?}");
    assert!(max_ulps <= MAX_EDGE_ULPS, "{max_ulps} ulps");
}

#[test]
fn hob_depths_match_the_full_sweep_bit_for_bit() {
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
                }
            }
        }
    }
}

/// The table's sample count is the one the verbatim sweep takes at every
/// candidate polar angle of an edge search: `(hi − lo)·density` rounded up,
/// with `hi − lo` twice the reach either way up to rounding, and that
/// product nowhere near a whole number on these worms.
#[test]
fn the_table_takes_the_samples_a_point_sweep_takes() {
    for worm in worms() {
        let hob = HobSweep::new(&worm);
        let product = 2.0 * hob.reach * hob.density;
        assert!(
            (product - product.round()).abs() > 1e-6,
            "{worm:?} {product}"
        );
        let r2 = 0.5 * worm.module * f64::from(worm.wheel_teeth);
        let pitch = TAU / f64::from(worm.wheel_teeth);
        let table = hob.tabulate(worm.module, r2);
        for k in 0..=200 {
            // Across the whole bracket of both sides' searches.
            let theta = (f64::from(k) / 200.0 - 0.5).mul_add(pitch, PI);
            let w = Point3::new(worm.module, r2 * theta.cos(), r2 * theta.sin());
            // As the verbatim sweep counts.
            let theta = w.z().atan2(w.y()).rem_euclid(2.0 * PI);
            let center = PI - theta;
            let (lo, hi) = (center - hob.reach, center + hob.reach);
            let count = ((hi - lo) * hob.density).ceil().max(8.0) as usize;
            assert_eq!(table.samples.len(), count + 1, "{worm:?} theta {theta}");
            let step = (hi - lo) / count as f64;
            assert!((table.step - step).abs() <= 4.0 * f64::EPSILON * step);
            // And the same wheel positions, first to last.
            assert!((theta + lo - table.lo).abs() <= 8.0 * f64::EPSILON);
        }
    }
}

#[test]
fn the_edge_search_evaluates_an_eighth_of_the_margins() {
    // Deterministic work counts, not wall time: off the mid-plane on the
    // probe worm, the heights the flank fit samples. Each table sample
    // counts as one margin.
    let found = compare_edges(&worms()[0], &[-1.0, -0.4375, 0.125, 0.75]);
    assert!(
        found.new_evals * 8 <= found.old_evals,
        "{} of {} margins",
        found.new_evals,
        found.old_evals
    );
}

/// Every edge the flank fit of the probe wheel (face width 8) asks for.
#[test]
#[ignore = "about ten seconds: the verbatim search over the whole fit grid"]
fn the_probe_wheels_fit_grid_stays_within_ulps_of_the_full_sweep() {
    let worm = &worms()[0];
    let hob = HobSweep::new(worm);
    let (a, half, root, tip, outside) = (40.0_f64, 4.0, 27.5, 32.0_f64, 33.0_f64);
    let tube = a - tip;
    let half_throat = tube.mul_add(tube, -(a - outside).powi(2)).sqrt();
    let mut edges = 0;
    let mut max_ulps = 0;
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
                let old = old_edge(&hob, side, s, rho);
                let what = format!("{side:?} {s} {rho}");
                max_ulps = max_ulps.max(edge_ulps(hob.edge(side, s, rho), old, &what));
                edges += usize::from(matches!(old, HobEdge::At(_)));
            }
        }
    }
    assert!(edges > 200, "{edges} edges");
    assert!(max_ulps <= MAX_EDGE_ULPS, "{max_ulps} ulps");
}

/// A wheel built with the tabulated or the verbatim search.
fn build(
    worm: &EnvelopingWormParams,
    options: &EnvelopingWheelOptions,
    verbatim: bool,
) -> (Topology, EnvelopingWheelOutcome) {
    let mut topo = Topology::new();
    VERBATIM.with(|flag| flag.set(verbatim));
    let outcome = make_enveloping_worm_wheel(&mut topo, worm, options, &GearPlacement::default());
    VERBATIM.with(|flag| flag.set(false));
    (topo, outcome.unwrap())
}

/// Every vertex and every NURBS control point of a solid, in the order its
/// faces, edges and vertices are met; and the three counts.
fn geometry(topo: &Topology, solid: SolidId) -> (Vec<Point3>, [usize; 3]) {
    let faces = solid_faces(topo, solid).unwrap();
    let edges = solid_edges(topo, solid).unwrap();
    let vertices = solid_vertices(topo, solid).unwrap();
    let mut points: Vec<Point3> = vertices
        .iter()
        .map(|&v| topo.vertex(v).unwrap().point())
        .collect();
    for &edge in &edges {
        if let EdgeCurve::NurbsCurve(curve) = topo.edge(edge).unwrap().curve() {
            points.extend_from_slice(curve.control_points());
        }
    }
    for &face in &faces {
        if let FaceSurface::Nurbs(surface) = topo.face(face).unwrap().surface() {
            points.extend(surface.control_points().iter().flatten().copied());
        }
    }
    (points, [faces.len(), edges.len(), vertices.len()])
}

/// Builds one wheel both ways and holds the tabulated one to the verbatim
/// one: the same faces, edges, vertices and control points, none displaced
/// by more than [`MAX_DISPLACEMENT`], and flank laws, relief and volume to
/// match.
fn compare_wheels(worm: &EnvelopingWormParams, options: &EnvelopingWheelOptions) {
    let (old_topo, old) = build(worm, options, true);
    let (new_topo, new) = build(worm, options, false);
    let (old_points, old_counts) = geometry(&old_topo, old.solid);
    let (new_points, new_counts) = geometry(&new_topo, new.solid);
    assert_eq!(new_counts, old_counts, "{options:?}");
    assert_eq!(new_points.len(), old_points.len(), "{options:?}");
    assert_eq!(new.face_roles.len(), old.face_roles.len());
    let displacement = new_points
        .iter()
        .zip(&old_points)
        .map(|(a, b)| (*a - *b).length())
        .fold(0.0, f64::max);
    let laws = |outcome: &EnvelopingWheelOutcome| {
        let mut all = outcome.flank_turn.concat();
        all.extend(outcome.flank_offset.concat());
        all
    };
    let law = laws(&new)
        .iter()
        .zip(laws(&old))
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max);
    let relief = (new.relief - old.relief).abs();
    let bound = (new.flank_deviation_bound - old.flank_deviation_bound).abs();
    let old_volume = solid_volume(&old_topo, old.solid, 1e-4).unwrap();
    let new_volume = solid_volume(&new_topo, new.solid, 1e-4).unwrap();
    let volume = ((new_volume - old_volume) / old_volume).abs();
    let measured = format!(
        "{options:?}: displacement {displacement:e}, law {law:e}, relief {relief:e}, \
         deviation bound {bound:e}, volume {volume:e} of {old_volume}, counts {old_counts:?}"
    );
    assert!(displacement <= MAX_DISPLACEMENT, "{measured}");
    assert!(law <= MAX_DISPLACEMENT, "{measured}");
    assert!(relief <= MAX_DISPLACEMENT, "{measured}");
    assert!(bound <= MAX_DISPLACEMENT, "{measured}");
    assert!(volume <= 1e-12, "{measured}");
}

#[test]
fn the_probe_wheel_is_the_verbatim_wheel_to_a_displacement_bound() {
    compare_wheels(&worms()[0], &EnvelopingWheelOptions::new(8.0));
}

/// The other wheels of `gear/tests/hindley_wheel.rs`, and a left-hand one.
#[test]
#[ignore = "about a minute: three wheels built with the verbatim search"]
fn other_wheels_are_the_verbatim_wheels_to_a_displacement_bound() {
    let probe = &worms()[0];
    compare_wheels(
        probe,
        &EnvelopingWheelOptions {
            backlash: 0.05,
            bore_diameter: Some(12.0),
            outside_diameter: Some(64.4),
            ..EnvelopingWheelOptions::new(7.0)
        },
    );
    compare_wheels(
        probe,
        &EnvelopingWheelOptions {
            bore_diameter: Some(12.0),
            ..EnvelopingWheelOptions::new(8.0)
        },
    );
    compare_wheels(&worms()[2], &EnvelopingWheelOptions::new(8.0));
}
