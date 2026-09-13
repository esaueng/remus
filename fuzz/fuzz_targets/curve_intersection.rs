//! Structured fuzzing of NURBS curve-surface intersection.
//!
//! Builds a bounded NURBS curve and a bounded NURBS surface from the fuzzer's
//! bytes (small degree, few control points, coordinates on a coarse lattice
//! so near-degeneracy is common) and runs `intersect_curve_surface`. The
//! oracle is independent of the solver under test: every reported hit must
//! satisfy BOTH geometries —
//!
//! * `curve.evaluate(hit.t)` is within tolerance of `hit.point`, and
//! * `surface.evaluate(hit.uv.0, hit.uv.1)` is within tolerance of
//!   `hit.point`.
//!
//! A hit that lies on only one of the two geometries (or on neither) is a
//! finding. Finiteness is enforced the same way: a non-finite hit point,
//! parameter, or evaluation is malformed successful output, not a refusal.
//!
//! **A typed refusal is a pass.** Constructor or solver `Err` stops the case
//! silently. Panics only fire on malformed-but-successful output.

#![no_main]

use arbitrary::{Arbitrary, Unstructured};
use libfuzzer_sys::fuzz_target;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::intersection::intersect_curve_surface;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;

/// Residual tolerance: hits must re-evaluate onto both geometries within
/// this 3D distance. Loose on purpose (gross-disagreement detector): the
/// solver refines by Newton, so healthy hits sit at ~1e-9; anything above
/// 1e-4 is a wrong answer, not a precision wobble.
const HIT_TOL: f64 = 1e-4;

/// Lattice coordinate in [-4.0, 4.0] on a half-unit grid.
fn coord(b: u8) -> f64 {
    (f64::from(b % 17) - 8.0) * 0.5
}

#[derive(Debug, Clone, Copy, Arbitrary)]
struct Ctrl {
    x: u8,
    y: u8,
    z: u8,
    w: u8,
}

fn point(c: Ctrl) -> Point3 {
    Point3::new(coord(c.x), coord(c.y), coord(c.z))
}

/// Strictly positive weight in [0.25, 2.0]; zero/negative weights are a
/// constructor refusal, not fuzz material.
fn weight(b: u8) -> f64 {
    0.25 + f64::from(b % 8) * 0.25
}

fn knots(count: usize, degree: usize) -> Vec<f64> {
    // Clamped uniform knots: deterministic given (count, degree), so the
    // fuzzer's bytes go to control data rather than knot validity.
    let interior = count.saturating_sub(degree + 1);
    let mut knots = vec![0.0; degree + 1];
    for i in 1..=interior {
        knots.push(i as f64 / (interior + 1) as f64);
    }
    knots.extend(std::iter::repeat_n(1.0, degree + 1));
    knots
}

fn build_curve(u: &mut Unstructured<'_>) -> arbitrary::Result<NurbsCurve> {
    let n = 2 + (u.bytes(1)?[0] % 3) as usize; // 2..=4 control points
    let degree = (1 + (u.bytes(1)?[0] % 2) as usize).min(n - 1); // 1..=2
    let mut pts = Vec::with_capacity(n);
    let mut wts = Vec::with_capacity(n);
    for _ in 0..n {
        let c = Ctrl::arbitrary(u)?;
        pts.push(point(c));
        wts.push(weight(c.w));
    }
    NurbsCurve::new(degree, knots(n, degree), pts, wts)
        .map_err(|_| arbitrary::Error::IncorrectFormat)
}

fn build_surface(u: &mut Unstructured<'_>) -> arbitrary::Result<NurbsSurface> {
    let nu = 2 + (u.bytes(1)?[0] % 2) as usize; // 2..=3
    let nv = 2 + (u.bytes(1)?[0] % 2) as usize;
    let du = (1 + (u.bytes(1)?[0] % 2) as usize).min(nu - 1);
    let dv = (1 + (u.bytes(1)?[0] % 2) as usize).min(nv - 1);
    let mut pts = Vec::with_capacity(nu);
    let mut wts = Vec::with_capacity(nu);
    for _ in 0..nu {
        let mut row = Vec::with_capacity(nv);
        let mut wrow = Vec::with_capacity(nv);
        for _ in 0..nv {
            let c = Ctrl::arbitrary(u)?;
            row.push(point(c));
            wrow.push(weight(c.w));
        }
        pts.push(row);
        wts.push(wrow);
    }
    NurbsSurface::new(du, dv, knots(nu, du), knots(nv, dv), pts, wts)
        .map_err(|_| arbitrary::Error::IncorrectFormat)
}

fuzz_target!(|data: &[u8]| {
    let mut u = Unstructured::new(data);
    let Ok(curve) = build_curve(&mut u) else {
        return;
    };
    let Ok(surface) = build_surface(&mut u) else {
        return;
    };
    let tolerance = 1e-7;
    let Ok(hits) = intersect_curve_surface(&curve, &surface, tolerance) else {
        return; // typed refusal is a pass
    };
    for hit in &hits {
        assert!(
            hit.point.x().is_finite()
                && hit.point.y().is_finite()
                && hit.point.z().is_finite(),
            "hit point is non-finite: {:?}",
            hit.point,
        );
        assert!(
            hit.t.is_finite() && hit.uv.0.is_finite() && hit.uv.1.is_finite(),
            "hit parameters are non-finite: t={}, uv={:?}",
            hit.t,
            hit.uv,
        );
        // Independent oracle, leg 1: the hit lies on the curve.
        let on_curve = curve.evaluate(hit.t);
        let dc = ((on_curve.x() - hit.point.x()).powi(2)
            + (on_curve.y() - hit.point.y()).powi(2)
            + (on_curve.z() - hit.point.z()).powi(2))
        .sqrt();
        assert!(
            dc <= HIT_TOL,
            "hit fails the curve leg: re-evaluated C(t) misses the reported point by {dc:.3e} (> {HIT_TOL:.0e})",
        );
        // Independent oracle, leg 2: the hit lies on the surface.
        let on_surface = surface.evaluate(hit.uv.0, hit.uv.1);
        let ds = ((on_surface.x() - hit.point.x()).powi(2)
            + (on_surface.y() - hit.point.y()).powi(2)
            + (on_surface.z() - hit.point.z()).powi(2))
        .sqrt();
        assert!(
            ds <= HIT_TOL,
            "hit fails the surface leg: re-evaluated S(u,v) misses the reported point by {ds:.3e} (> {HIT_TOL:.0e})",
        );
    }
});
