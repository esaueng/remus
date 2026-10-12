//! Opt-in per-thread memo of planar arc pcurves.
//!
//! The face splitter computes a pcurve for every boundary edge of every face a
//! boolean touches. For an arc on a plane that is sixteen projected samples
//! and a NURBS interpolation solve. A plate drilled one hole at a time keeps
//! its earlier hole rims unchanged from cut to cut, yet the GFA refits all of
//! them on every cut, because the splitter re-reads the whole face (stage 3
//! of `docs/performance/perf-local-boolean.md`).
//!
//! The pcurve of a circle or ellipse on a plane is a pure function of the
//! curve, its traversal endpoints and domain, the plane, and the plane
//! frame. This memo keys on the exact bits of all of them and returns a
//! clone of the curve computed the first time, so every hit is bit-identical
//! to recomputing. Failures are never stored.
//!
//! It is off unless a thread enables it (`enable_thread_pcurve_memo`), like
//! the other content-keyed memos the WASM kernel turns on module-wide. It is
//! bounded: past its capacity, the oldest entries are dropped first.

use std::cell::RefCell;

use remus_math::curves2d::Curve2D;

use crate::memo::BoundedMemo;

/// Default number of pcurves a thread keeps once enabled.
///
/// Enough for both faces of a plate carrying about two thousand holes. A
/// full-circle pcurve holds about twenty control points, so the memo stays
/// near a few MB.
pub const DEFAULT_PCURVE_MEMO_CAPACITY: usize = 4096;

/// Exact bits of everything a planar arc pcurve depends on.
pub(crate) type PcurveKey = [u64; PCURVE_KEY_WORDS];

/// Words in a [`PcurveKey`]: curve (kind, centre, normal, two radii, two
/// axes), endpoints, domain, plane (normal, offset) and frame (origin, two
/// axes).
pub(crate) const PCURVE_KEY_WORDS: usize = 1 + 3 + 3 + 2 + 3 + 3 + 3 + 3 + 2 + 4 + 3 + 3 + 3;

thread_local! {
    static MEMO: RefCell<BoundedMemo<PcurveKey, Curve2D>> = RefCell::new(BoundedMemo::default());
}

/// Statistics of this thread's memo.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PcurveMemoStats {
    /// Lookups answered from the memo.
    pub hits: u64,
    /// Lookups the enabled memo could not answer (it computed and stored).
    pub misses: u64,
    /// Pcurves held.
    pub len: usize,
    /// Most pcurves held; zero when the memo is off.
    pub capacity: usize,
}

/// Enable this thread's memo at [`DEFAULT_PCURVE_MEMO_CAPACITY`] unless it
/// is already enabled (then its capacity and contents are kept).
pub fn enable_thread_pcurve_memo() {
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            memo.enable(DEFAULT_PCURVE_MEMO_CAPACITY);
        }
    });
}

/// Set how many pcurves this thread's memo keeps; zero disables it and
/// drops its contents. Shrinking drops the oldest first.
pub fn set_thread_pcurve_memo_capacity(capacity: usize) {
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            memo.set_capacity(capacity);
        }
    });
}

/// This thread's memo statistics.
#[must_use]
pub fn thread_pcurve_memo_stats() -> PcurveMemoStats {
    let stats = MEMO.with(|memo| {
        memo.try_borrow()
            .map(|memo| memo.stats())
            .unwrap_or_default()
    });
    PcurveMemoStats {
        hits: stats.hits,
        misses: stats.misses,
        len: stats.len,
        capacity: stats.capacity,
    }
}

/// The pcurve for `key`: a clone of the stored one, or `compute`'s result,
/// stored when the memo is on. `compute` runs outside the memo's borrow.
pub(crate) fn memoized<E>(
    key: &PcurveKey,
    compute: impl FnOnce() -> Result<Curve2D, E>,
) -> Result<Curve2D, E> {
    let stored = MEMO.with(|memo| memo.try_borrow_mut().ok()?.get(key));
    if let Some(curve) = stored {
        return Ok(curve);
    }
    let curve = compute()?;
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            memo.insert(key, &curve);
        }
    });
    Ok(curve)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use remus_math::curves::{Circle3D, Ellipse3D};
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::edge::EdgeCurve;
    use remus_topology::face::FaceSurface;

    use super::*;
    use crate::builder::pcurve_compute::compute_pcurve_on_surface_in_domain;
    use crate::builder::plane_frame::PlaneFrame;

    type Case = (
        EdgeCurve,
        Point3,
        Point3,
        (f64, f64),
        FaceSurface,
        PlaneFrame,
    );

    /// Arcs on two planes: full circles, open circle arcs both ways, an
    /// ellipse, and a circle one ulp larger than another.
    fn cases() -> Vec<Case> {
        let mut out = Vec::new();
        for (normal, origin) in [
            (Vec3::new(0.0, 0.0, 1.0), Point3::new(0.0, 0.0, 10.0)),
            (
                Vec3::new(0.3, -0.4, 0.866).normalize().unwrap(),
                Point3::new(2.0, -1.0, 3.0),
            ),
        ] {
            let surface = FaceSurface::Plane {
                normal,
                d: normal.dot(origin - Point3::new(0.0, 0.0, 0.0)),
            };
            let frame = PlaneFrame::from_normal_and_point(normal, origin);
            for (k, radius) in [2.0, 3.5, f64::from_bits(2.0_f64.to_bits() + 1)]
                .into_iter()
                .enumerate()
            {
                // The ulp-larger circle shares the first circle's centre.
                let center = frame.evaluate(if k == 1 { 5.0 } else { 0.0 }, 1.5);
                let circle = Circle3D::new(center, normal, radius).unwrap();
                let p0 = circle.evaluate(0.0);
                out.push((
                    EdgeCurve::Circle(circle.clone()),
                    p0,
                    p0,
                    (0.0, std::f64::consts::TAU),
                    surface.clone(),
                    frame.clone(),
                ));
                for domain in [(0.4, 2.1), (2.1, 0.4)] {
                    out.push((
                        EdgeCurve::Circle(circle.clone()),
                        circle.evaluate(domain.0),
                        circle.evaluate(domain.1),
                        domain,
                        surface.clone(),
                        frame.clone(),
                    ));
                }
            }
            let ellipse = Ellipse3D::new(frame.evaluate(-4.0, -3.0), normal, 3.0, 1.25).unwrap();
            let p0 = ellipse.evaluate(0.0);
            out.push((
                EdgeCurve::Ellipse(ellipse),
                p0,
                p0,
                (0.0, std::f64::consts::TAU),
                surface,
                frame,
            ));
        }
        out
    }

    fn compute_all() -> Vec<String> {
        cases()
            .iter()
            .map(|(curve, start, end, domain, surface, frame)| {
                let pcurve = compute_pcurve_on_surface_in_domain(
                    curve,
                    *start,
                    *end,
                    *domain,
                    surface,
                    &[],
                    Some(frame),
                )
                .unwrap();
                // `Debug` prints every f64 exactly.
                format!("{pcurve:?}")
            })
            .collect()
    }

    #[test]
    fn memoized_pcurves_are_bit_identical_to_computed_ones() {
        set_thread_pcurve_memo_capacity(0);
        let computed = compute_all();
        enable_thread_pcurve_memo();
        let stored = compute_all();
        let recalled = compute_all();
        let stats = thread_pcurve_memo_stats();
        set_thread_pcurve_memo_capacity(0);

        assert_eq!(computed, stored);
        assert_eq!(computed, recalled);
        // Every case has its own key, the ulp-larger circle included (a
        // collision would answer it from the memo while storing): one miss
        // each while storing, then one hit each.
        let n = computed.len() as u64;
        assert_eq!((stats.misses, stats.hits), (n, n), "{stats:?}");
    }

    #[test]
    fn memo_is_bounded_and_off_by_default() {
        set_thread_pcurve_memo_capacity(0);
        let _ = compute_all();
        assert_eq!(
            thread_pcurve_memo_stats().len,
            0,
            "a disabled memo stored pcurves"
        );
        set_thread_pcurve_memo_capacity(3);
        let _ = compute_all();
        assert_eq!(thread_pcurve_memo_stats().len, 3);
        set_thread_pcurve_memo_capacity(1);
        assert_eq!(thread_pcurve_memo_stats().len, 1);
        set_thread_pcurve_memo_capacity(0);
        assert_eq!(thread_pcurve_memo_stats().len, 0);
    }
}
