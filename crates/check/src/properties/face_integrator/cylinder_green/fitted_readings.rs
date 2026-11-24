//! Reblend plans can integrate the same fitted cylindrical trim on successive
//! private solids. Reuse a converged pair only within the operation that asked
//! for it, including nested reads. No topology handles enter the key.

use std::cell::RefCell;
use std::collections::VecDeque;

use remus_math::nurbs::curve::NurbsCurve;

use super::{Accumulator, EdgeMeasure, Reading};

const MAX_ENTRIES: usize = 16;
// At most 512 KiB of curve and frame data per active thread.
const MAX_KEY_WORDS: usize = 4096;

pub(super) type Pair = ((Accumulator, EdgeMeasure), (Accumulator, EdgeMeasure));
type Entries = VecDeque<(Vec<u64>, Pair)>;

thread_local! {
    static ENTRIES: RefCell<Option<Entries>> = const { RefCell::new(None) };
}

struct Scope;

impl Drop for Scope {
    fn drop(&mut self) {
        ENTRIES.with_borrow_mut(|entries| *entries = None);
    }
}

/// Share successful coarse and fine cylinder-trim readings for the duration
/// of an operation. Nested calls share the outer scope.
pub fn with_fitted_edge_readings<T>(run: impl FnOnce() -> T) -> T {
    if ENTRIES.with_borrow(Option::is_some) {
        return run();
    }
    ENTRIES.with_borrow_mut(|entries| *entries = Some(Entries::new()));
    let _scope = Scope;
    run()
}

// These are the inputs to Reading::edge for a fitted trim. Its normalized axis,
// off-surface bound and convergence scale derive from the stored cylinder and
// reference. NURBS evaluation uses the control points, not the edge's vertices.
fn key(
    reading: &Reading<'_>,
    curve: &NurbsCurve,
    lo: f64,
    hi: f64,
    direction: f64,
) -> Option<Vec<u64>> {
    let words = curve
        .control_points()
        .len()
        .checked_mul(3)?
        .checked_add(curve.weights().len())?
        .checked_add(curve.knots().len())?
        .checked_add(23)?;
    if words > MAX_KEY_WORDS {
        return None;
    }
    let mut key = Vec::with_capacity(words);
    key.extend(
        [
            curve.degree(),
            curve.knots().len(),
            curve.control_points().len(),
            curve.weights().len(),
        ]
        .map(|n| n as u64),
    );
    key.extend([lo, hi, direction, reading.cylinder.radius()].map(f64::to_bits));
    for v in [
        reading.cylinder.origin().0,
        reading.cylinder.axis().0,
        reading.cylinder.x_axis().0,
        reading.cylinder.y_axis().0,
        reading.reference.0,
    ] {
        key.extend(v.map(f64::to_bits));
    }
    key.extend(curve.knots().iter().map(|x| x.to_bits()));
    key.extend(curve.weights().iter().map(|x| x.to_bits()));
    for p in curve.control_points() {
        key.extend(p.0.map(f64::to_bits));
    }
    Some(key)
}

pub(super) fn read(
    reading: &Reading<'_>,
    curve: &NurbsCurve,
    lo: f64,
    hi: f64,
    direction: f64,
    compute: impl FnOnce() -> Option<Pair>,
) -> Option<Pair> {
    if !ENTRIES.with_borrow(Option::is_some) {
        return compute();
    }
    let Some(key) = key(reading, curve, lo, hi, direction) else {
        return compute();
    };
    let hit = ENTRIES.with_borrow_mut(|slot| {
        let entries = slot.as_mut()?;
        let index = entries.iter().position(|(stored, _)| stored == &key)?;
        let entry = entries.remove(index)?;
        let pair = entry.1.clone();
        entries.push_back(entry);
        Some(pair)
    });
    if hit.is_some() {
        return hit;
    }
    let pair = compute()?;
    ENTRIES.with_borrow_mut(|slot| {
        if let Some(entries) = slot {
            if entries.len() == MAX_ENTRIES {
                entries.pop_front();
            }
            entries.push_back((key, pair.clone()));
        }
    });
    Some(pair)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use remus_math::surfaces::CylindricalSurface;
    use remus_math::vec::{Point3, Vec3};

    fn curve(knot: f64, weight: f64, x: f64) -> NurbsCurve {
        NurbsCurve::new(
            1,
            vec![0.0, 0.0, knot, 1.0, 1.0],
            vec![
                Point3::new(x, 0.0, 0.0),
                Point3::new(1.0, 0.0, 1.0),
                Point3::new(1.0, 1.0, 2.0),
            ],
            vec![1.0, weight, 1.0],
        )
        .unwrap()
    }

    fn reading(cylinder: &CylindricalSurface, reference: Point3) -> Reading<'_> {
        Reading {
            cylinder,
            reference,
            axis: cylinder.axis().normalize().unwrap(),
            off_surface_limit: (super::super::NURBS_ON_SURFACE * cylinder.radius())
                .max(super::super::NURBS_ON_SURFACE_ABS),
        }
    }

    #[test]
    fn distinguishes_frame_parameterization_reference_and_curve_bits() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let axis = Vec3::new(0.0, 0.0, 1.0);
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let cylinder = CylindricalSurface::with_ref_dir(origin, axis, 1.0, x).unwrap();
        let input = reading(&cylinder, origin);
        let trim = curve(0.5, 1.0, 0.);
        let original = key(&input, &trim, 0.0, 1.0, 1.).unwrap();
        for other in [curve(0.25, 1., 0.), curve(0.5, 2., 0.), curve(0.5, 1., -0.)] {
            assert_ne!(key(&input, &other, 0., 1., 1.).unwrap(), original);
        }
        for (lo, hi, direction) in [(0.25, 1., 1.), (0., 0.75, 1.), (0., 1., -1.)] {
            assert_ne!(key(&input, &trim, lo, hi, direction).unwrap(), original);
        }
        let shift = Point3::new(0., 0., 1.);
        for (o, a, r, rd) in [
            (origin, axis, 1., y),
            (origin, y, 1., x),
            (origin, axis, 2., x),
            (shift, axis, 1., x),
        ] {
            let other = CylindricalSurface::with_ref_dir(o, a, r, rd).unwrap();
            assert_ne!(
                key(&reading(&other, origin), &trim, 0., 1., 1.).unwrap(),
                original
            );
        }
        assert_ne!(
            key(&reading(&cylinder, shift), &trim, 0., 1., 1.).unwrap(),
            original
        );
    }

    #[test]
    fn oversized_trims_still_compute_without_retention() {
        let origin = Point3::new(0., 0., 0.);
        let cylinder = CylindricalSurface::new(origin, Vec3::new(0., 0., 1.), 2.).unwrap();
        let input = reading(&cylinder, origin);
        let knots = std::iter::once(0.)
            .chain((0..1024).map(f64::from))
            .chain([1023.])
            .collect();
        let trim = NurbsCurve::new(1, knots, vec![origin; 1024], vec![1.; 1024]).unwrap();
        with_fitted_edge_readings(|| {
            assert!(read(&input, &trim, 0., 1., 1., || Some(Pair::default())).is_some());
            assert!(ENTRIES.with_borrow(|entries| entries.as_ref().unwrap().is_empty()));
        });
    }

    #[test]
    fn keeps_only_successes_within_one_operation_and_evicts_old_entries() {
        let origin = Point3::new(0., 0., 0.);
        let cylinder = CylindricalSurface::new(origin, Vec3::new(0., 0., 1.), 2.).unwrap();
        let input = reading(&cylinder, origin);
        let trim = curve(0.5, 1., 1.);
        let lookup = |lo: f64| read(&input, &trim, lo, 1., 1., || None);
        let fill = |lo: f64| read(&input, &trim, lo, 1., 1., || Some(Pair::default()));
        assert!(fill(0.).is_some());
        with_fitted_edge_readings(|| {
            assert!(lookup(0.).is_none());
            assert!(fill(0.).is_some());
            assert!(with_fitted_edge_readings(|| lookup(0.)).is_some());
            for index in 1..=16 {
                assert!(fill(f64::from(index) / 32.).is_some());
            }
            assert!(lookup(0.).is_none());
            assert!(lookup(0.5).is_some());
        });
        assert!(lookup(0.5).is_none());
        assert!(
            std::panic::catch_unwind(|| with_fitted_edge_readings(|| {
                assert!(fill(0.).is_some());
                std::panic::resume_unwind(Box::new("unwind the operation"));
            }))
            .is_err()
        );
        assert!(ENTRIES.with_borrow(Option::is_none));
    }
}
