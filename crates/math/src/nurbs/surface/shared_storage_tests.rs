//! The cached periodicity flags and the knot-sharing constructor change no
//! answer, and they remove the per-call work they were added to remove.
//!
//! - `is_periodic_u` / `is_periodic_v` against the control-net comparisons
//!   they cache, kept verbatim below, on closed, open and degenerate nets,
//!   and through clones;
//! - `with_control_points` against `NurbsSurface::new` on copies of the same
//!   knots and weights: equal surfaces, and `point_and_normal_with` between
//!   a surface and its moved net returns the same bits either way;
//! - work counts: one net comparison per surface however often it is asked
//!   or projected onto, and no knot-vector comparison per sample between a
//!   surface and a net built by `with_control_points`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]

use std::cell::Cell;

use super::*;
use crate::nurbs::projection::project_point_to_surface;

thread_local! {
    static PERIODIC_COMPARES: Cell<usize> = const { Cell::new(0) };
    static KNOT_COMPARES: Cell<usize> = const { Cell::new(0) };
}

pub(super) fn record_periodic_compare() {
    PERIODIC_COMPARES.with(|count| count.set(count.get() + 1));
}

#[cfg_attr(feature = "simd", allow(dead_code))]
pub(super) fn record_knot_compare() {
    KNOT_COMPARES.with(|count| count.set(count.get() + 1));
}

fn take(counter: &'static std::thread::LocalKey<Cell<usize>>) -> usize {
    counter.with(|count| count.replace(0))
}

/// `is_periodic_u` before the flags were cached, verbatim.
fn legacy_is_periodic_u(surface: &NurbsSurface) -> bool {
    let n = surface.control_points.len();
    if n < 2 {
        return false;
    }
    let first = &surface.control_points[0];
    let last = &surface.control_points[n - 1];
    if first.len() != last.len() {
        return false;
    }
    // (1e-7)^2 matching Tolerance::default().linear
    first.iter().zip(last.iter()).all(|(a, b)| {
        let d = *a - *b;
        d.x() * d.x() + d.y() * d.y() + d.z() * d.z() < 1e-14
    })
}

/// `is_periodic_v` before the flags were cached, verbatim.
fn legacy_is_periodic_v(surface: &NurbsSurface) -> bool {
    if surface.control_points.is_empty() {
        return false;
    }
    // (1e-7)^2 matching Tolerance::default().linear
    surface.control_points.iter().all(|row| {
        if row.len() < 2 {
            return false;
        }
        let d = row[0] - row[row.len() - 1];
        d.x() * d.x() + d.y() * d.y() + d.z() * d.z() < 1e-14
    })
}

fn clamped_knots(degree: usize, count: usize) -> Vec<f64> {
    let spans = count - degree;
    let mut knots = vec![0.0; degree + 1];
    knots.extend((1..spans).map(|k| k as f64 / spans as f64));
    knots.extend(std::iter::repeat_n(1.0, degree + 1));
    knots
}

/// A rational tube swept `turns` times round the z axis: `rows` control
/// rows round the tube and `7 * turns + 3` columns along it. `close_u` and
/// `close_v` make the first and last row (column) coincide; `gap` opens
/// the closed direction by that distance instead.
fn tube(rows: usize, turns: usize, close_u: bool, close_v: bool, gap: f64) -> NurbsSurface {
    let columns = 7 * turns + 3;
    let mut net = Vec::with_capacity(rows);
    let mut weights = Vec::with_capacity(rows);
    for i in 0..rows {
        let around = if close_u {
            (i % (rows - 1)) as f64 / (rows - 1) as f64
        } else {
            i as f64 / rows as f64
        } * std::f64::consts::TAU;
        let open = if i == rows - 1 { gap } else { 0.0 };
        let mut row = Vec::with_capacity(columns);
        let mut weight_row = Vec::with_capacity(columns);
        for j in 0..columns {
            let along = if close_v {
                (j % (columns - 1)) as f64 / (columns - 1) as f64 * std::f64::consts::TAU
            } else {
                j as f64 / 7.0
            };
            let radius = 10.0 + 1.5 * around.cos();
            row.push(Point3::new(
                radius * along.cos() + open,
                radius * along.sin(),
                if close_v { 0.0 } else { 0.8 * along } + 1.5 * around.sin(),
            ));
            weight_row.push(1.0 + 0.25 * ((i + 2 * j) % 3) as f64);
        }
        net.push(row);
        weights.push(weight_row);
    }
    NurbsSurface::new(
        2,
        3,
        clamped_knots(2, rows),
        clamped_knots(3, columns),
        net,
        weights,
    )
    .expect("valid tube")
}

fn fixtures() -> Vec<(&'static str, NurbsSurface)> {
    vec![
        ("closed u", tube(9, 3, true, false, 0.0)),
        ("closed v", tube(9, 3, false, true, 0.0)),
        ("closed both", tube(9, 2, true, true, 0.0)),
        ("open", tube(9, 3, false, false, 0.0)),
        // Either side of the 1e-7 closure tolerance.
        ("u within tolerance", tube(9, 3, true, false, 0.9e-7)),
        ("u past tolerance", tube(9, 3, true, false, 1.1e-7)),
        ("long closed u", tube(5, 40, true, false, 0.0)),
    ]
}

#[test]
fn cached_periodicity_is_the_net_comparison() {
    let mut closed = [0, 0];
    for (name, surface) in fixtures() {
        for copy in [surface.clone(), surface] {
            let expected = (legacy_is_periodic_u(&copy), legacy_is_periodic_v(&copy));
            for _ in 0..3 {
                assert_eq!(
                    (copy.is_periodic_u(), copy.is_periodic_v()),
                    expected,
                    "{name}"
                );
            }
            // A clone made after the flags were read carries them.
            let later = copy.clone();
            assert_eq!(
                (later.is_periodic_v(), later.is_periodic_u()),
                (expected.1, expected.0),
                "{name} clone"
            );
            closed[0] += usize::from(expected.0);
            closed[1] += usize::from(expected.1);
        }
    }
    // The corpus holds closed and open nets in both directions.
    assert!(closed[0] >= 8 && closed[1] >= 4, "{closed:?}");
}

#[test]
fn periodicity_is_compared_once_per_surface() {
    let surface = tube(5, 40, true, false, 0.0);
    take(&PERIODIC_COMPARES);
    for k in 0..200 {
        let t = f64::from(k) / 200.0;
        assert!(surface.is_periodic_u());
        assert!(!surface.is_periodic_v());
        // Every projection asks for both flags.
        let point = surface.evaluate(0.3, t) + Vec3::new(0.01, 0.0, 0.0);
        let _ = project_point_to_surface(&surface, point, 1e-6);
    }
    assert_eq!(take(&PERIODIC_COMPARES), 1);
    // A clone carries the answer; a new surface over the same net asks once.
    let clone = surface.clone();
    drop(surface);
    assert!(clone.is_periodic_u());
    assert_eq!(take(&PERIODIC_COMPARES), 0);
    let rebuilt = clone
        .with_control_points(clone.control_points().to_vec())
        .unwrap();
    assert!(rebuilt.is_periodic_u() && rebuilt.is_periodic_u());
    assert_eq!(take(&PERIODIC_COMPARES), 1);
}

/// The net of `surface` moved by `shift`, built both ways: sharing the
/// surface's knots and weights, and from copies of them.
fn moved(surface: &NurbsSurface, shift: Vec3) -> (NurbsSurface, NurbsSurface) {
    let net: Vec<Vec<Point3>> = surface
        .control_points()
        .iter()
        .map(|row| row.iter().map(|&p| p + shift).collect())
        .collect();
    let copied = NurbsSurface::new(
        surface.degree_u(),
        surface.degree_v(),
        surface.knots_u().to_vec(),
        surface.knots_v().to_vec(),
        net.clone(),
        surface.weights().to_vec(),
    )
    .unwrap();
    (surface.with_control_points(net).unwrap(), copied)
}

fn bits(v: Vec3) -> [u64; 3] {
    [v.x().to_bits(), v.y().to_bits(), v.z().to_bits()]
}

#[test]
fn a_shared_net_is_the_copied_one_bit_for_bit() {
    let shift = Vec3::new(-11.5, 0.25, 3.0);
    for (name, surface) in fixtures() {
        let (shared, copied) = moved(&surface, shift);
        assert_eq!(shared, copied, "{name}");
        assert_eq!(shared.max_weight().to_bits(), copied.max_weight().to_bits());
        for i in 0..=12 {
            for j in 0..=12 {
                // Past both ends of the domain too.
                let (u, v) = (f64::from(i) / 10.0 - 0.1, f64::from(j) / 10.0 - 0.1);
                let (p_shared, n_shared) = surface.point_and_normal_with(&shared, u, v);
                let (p_copied, n_copied) = surface.point_and_normal_with(&copied, u, v);
                assert_eq!(
                    bits(p_shared - Point3::new(0.0, 0.0, 0.0)),
                    bits(p_copied - Point3::new(0.0, 0.0, 0.0)),
                    "{name} point ({u}, {v})"
                );
                assert_eq!(
                    n_shared.ok().map(bits),
                    n_copied.ok().map(bits),
                    "{name} normal ({u}, {v})"
                );
                // The fused call is the two separate ones.
                assert_eq!(
                    bits(p_shared - surface.evaluate(u, v)),
                    [0.0_f64.to_bits(); 3]
                );
                assert_eq!(
                    surface
                        .point_and_normal_with(&shared, u, v)
                        .1
                        .ok()
                        .map(bits),
                    shared.normal(u, v).ok().map(bits),
                    "{name} carrier normal ({u}, {v})"
                );
            }
        }
    }
}

#[test]
fn a_net_of_another_shape_is_refused() {
    let surface = tube(9, 3, true, false, 0.0);
    let mut net = surface.control_points().to_vec();
    net.pop();
    assert!(surface.with_control_points(net).is_err());
    let mut net = surface.control_points().to_vec();
    net[4].pop();
    assert!(surface.with_control_points(net).is_err());
    assert!(surface.with_control_points(Vec::new()).is_err());
}

#[cfg(not(feature = "simd"))]
#[test]
fn a_shared_net_skips_the_knot_comparison() {
    let surface = tube(5, 40, true, false, 0.0);
    let (shared, copied) = moved(&surface, Vec3::new(1.0, 2.0, 3.0));
    let sample = |carrier: &NurbsSurface| {
        take(&KNOT_COMPARES);
        for k in 0..100 {
            let _ = surface.point_and_normal_with(carrier, 0.4, f64::from(k) / 100.0);
        }
        take(&KNOT_COMPARES)
    };
    assert_eq!(sample(&shared), 0);
    assert_eq!(sample(&surface), 0);
    // Copies are still recognized, by comparing both knot vectors per call.
    assert_eq!(sample(&copied), 200);
}
