//! Interior chart points that would fold a triangle over a boundary chord.
//!
//! The non-planar CDT triangulates a face in its parameter chart, where each
//! boundary chord is a straight segment between two consecutive shared
//! boundary samples. On a cone the `(u, v)` chart is not a development: a
//! straight chart segment maps to a curve that runs beside the 3D chord
//! between the same two samples, not under it. An interior grid point a few
//! 1e-8 inside the chart chord can therefore lie beyond the 3D chord on the
//! surface, and the triangle the CDT builds from the chord and that point
//! turns over in space (B289: the chamfer and tapered-thread cones kept 1 to
//! 44 such slivers, closed and edge-manifold but wound against the face).
//!
//! [`ChordFoldGuard`] reads each boundary chord once, with a reference
//! normal: the chart's orientation at the chord's middle (the surface's
//! chart Jacobian applied to the chord's direction and its left normal). An
//! interior candidate in
//! the band beside a chord folds when the 3D triangle it makes with that chord
//! is not within 60 degrees of the reference (negated for a candidate on the
//! chord's right). The test is side-agnostic, so holes need no orientation of
//! their own, and it reads only the samples' 3D positions: it never moves a
//! point. Dropping an interior point never invalidates the triangulation, so
//! a fold the guard finds is resolved by leaving the point out.

use remus_math::vec::{Point2, Point3, Vec3};

/// How far beside a chord, as a fraction of the chord's chart length, a
/// candidate is tested against it. The lens a chart chord's image leaves
/// beside its 3D chord is far thinner than this (second order in the chord's
/// length); the band only bounds the work.
const BAND: f64 = 0.25;

/// A candidate folds when the cosine between its triangle's 3D normal and the
/// chord's reference normal is at most this: the triangle is turned over, or
/// stands more than 60 degrees off the surface.
const MIN_COSINE: f64 = 0.5;

/// Chords per broad-phase block, as in `PreparedBoundarySegments`.
const CHORDS_PER_BLOCK: usize = 16;

/// One boundary chord with its chart band and 3D reference normal.
#[derive(Debug, Clone, Copy)]
struct Chord {
    start: Point2,
    /// Chart vector from `start` to the chord's other end.
    edge: (f64, f64),
    length_squared: f64,
    p0: Point3,
    p1: Point3,
    /// Unit 3D chart orientation normal at the chord's middle, for a
    /// triangle on the chord's chart left.
    reference: Vec3,
    lower: [f64; 2],
    upper: [f64; 2],
}

/// Boundary chords of one chart, read for the fold test.
#[derive(Debug, Clone)]
pub(super) struct ChordFoldGuard {
    chords: Vec<Chord>,
    blocks: Vec<Option<([f64; 2], [f64; 2])>>,
}

impl ChordFoldGuard {
    /// Read the closed `wires` (chart points and the 3D samples they chart,
    /// index for index). `surface_point` evaluates the surface at a chart
    /// point; it is called four times per chord. A chord whose reference cannot be
    /// read (degenerate, non-finite) is left out: it constrains nothing.
    pub(super) fn new<'a>(
        wires: impl IntoIterator<Item = (&'a [Point2], &'a [Point3])>,
        mut surface_point: impl FnMut(Point2) -> Point3,
    ) -> Self {
        let mut chords = Vec::new();
        for (chart, space) in wires {
            let n = chart.len().min(space.len());
            if n < 3 {
                continue;
            }
            for i in 0..n {
                let j = (i + 1) % n;
                if let Some(chord) =
                    Chord::read(chart[i], chart[j], space[i], space[j], &mut surface_point)
                {
                    chords.push(chord);
                }
            }
        }
        let blocks = chords
            .chunks(CHORDS_PER_BLOCK)
            .map(|chunk| {
                chunk
                    .iter()
                    .fold(None, |acc: Option<([f64; 2], [f64; 2])>, c| {
                        Some(acc.map_or((c.lower, c.upper), |(lo, hi)| {
                            (
                                [lo[0].min(c.lower[0]), lo[1].min(c.lower[1])],
                                [hi[0].max(c.upper[0]), hi[1].max(c.upper[1])],
                            )
                        }))
                    })
            })
            .collect();
        Self { chords, blocks }
    }

    /// Number of chords read (for tests).
    #[cfg(test)]
    pub(super) fn chord_count(&self) -> usize {
        self.chords.len()
    }

    /// Whether the interior candidate at chart point `chart` (surface point
    /// `point`) folds a triangle over any chord whose band holds it.
    pub(super) fn folds(&self, chart: Point2, point: Point3) -> bool {
        let (x, y) = (chart.x(), chart.y());
        if !x.is_finite() || !y.is_finite() {
            return false;
        }
        let outside =
            |lo: &[f64; 2], hi: &[f64; 2]| x < lo[0] || x > hi[0] || y < lo[1] || y > hi[1];
        self.chords
            .chunks(CHORDS_PER_BLOCK)
            .zip(&self.blocks)
            .any(|(chunk, block)| {
                block.is_some_and(|(lo, hi)| !outside(&lo, &hi))
                    && chunk
                        .iter()
                        .any(|c| !outside(&c.lower, &c.upper) && c.folds(chart, point))
            })
    }
}

impl Chord {
    fn read(
        a: Point2,
        b: Point2,
        p0: Point3,
        p1: Point3,
        surface_point: &mut impl FnMut(Point2) -> Point3,
    ) -> Option<Self> {
        let edge = (b.x() - a.x(), b.y() - a.y());
        let length_squared = edge.0.mul_add(edge.0, edge.1 * edge.1);
        let length = length_squared.sqrt();
        if !(length.is_finite() && length > 0.0) {
            return None;
        }
        // The chart's orientation normal at the chord's middle, `J e₁ × J e₂`
        // for the chord's unit direction `e₁` and its left normal `e₂`, by
        // central differences a thousandth of the chord long: close enough
        // to the chord to stay on the face's sheet of the surface (never
        // across a cone's apex), long enough to stay clear of rounding.
        let step = 1e-3;
        let middle = (0.5f64.mul_add(edge.0, a.x()), 0.5f64.mul_add(edge.1, a.y()));
        let at = |dx: f64, dy: f64| Point2::new(middle.0 + dx, middle.1 + dy);
        let (ax, ay) = (step * edge.0, step * edge.1);
        let along = surface_point(at(ax, ay)) - surface_point(at(-ax, -ay));
        let across = surface_point(at(-ay, ax)) - surface_point(at(ay, -ax));
        let reference = along.cross(across);
        let norm = reference.length();
        if !(norm.is_finite() && norm > 0.0) {
            return None;
        }
        let reference = reference * (1.0 / norm);
        let pad = BAND * length;
        let lower = [a.x().min(b.x()) - pad, a.y().min(b.y()) - pad];
        let upper = [a.x().max(b.x()) + pad, a.y().max(b.y()) + pad];
        lower
            .into_iter()
            .chain(upper)
            .all(f64::is_finite)
            .then_some(Self {
                start: a,
                edge,
                length_squared,
                p0,
                p1,
                reference,
                lower,
                upper,
            })
    }

    fn folds(&self, chart: Point2, point: Point3) -> bool {
        let rel = (chart.x() - self.start.x(), chart.y() - self.start.y());
        let along = self.edge.0.mul_add(rel.0, self.edge.1 * rel.1);
        // Only a candidate beside the chord's span can take it as an edge of
        // a thin triangle; one off its ends is the neighbouring chord's.
        if along <= 0.0 || along >= self.length_squared {
            return false;
        }
        let side = self.edge.0.mul_add(rel.1, -(self.edge.1 * rel.0));
        if side.abs() > BAND * self.length_squared {
            return false;
        }
        let normal = (self.p1 - self.p0).cross(point - self.p0);
        let length = normal.length();
        if side == 0.0 || length.is_nan() || length <= 0.0 {
            // On the chart chord, or collinear in space: a degenerate triangle.
            return true;
        }
        side.signum() * normal.dot(self.reference) <= MIN_COSINE * length
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use remus_math::surfaces::ConicalSurface;
    use remus_math::vec::Vec3;

    fn cone(half_angle: f64) -> ConicalSurface {
        ConicalSurface::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            half_angle,
        )
        .unwrap()
    }

    /// The `(u, v)` chart wire of a quarter-turn cone band between slant
    /// lengths `v0 < v1`, `n` chords per rim, counter-clockwise in the chart
    /// (the face on the left of every chord), and its 3D samples. The far
    /// rim's first chord runs from index `n + 1` to `n + 2`, `u` decreasing.
    fn band(surface: &ConicalSurface, v0: f64, v1: f64, n: usize) -> (Vec<Point2>, Vec<Point3>) {
        let span = std::f64::consts::FRAC_PI_2;
        let mut chart = Vec::new();
        for k in 0..=n {
            chart.push(Point2::new(span * k as f64 / n as f64, v0));
        }
        for k in (0..=n).rev() {
            chart.push(Point2::new(span * k as f64 / n as f64, v1));
        }
        let space = chart
            .iter()
            .map(|p| surface.evaluate(p.x(), p.y()))
            .collect();
        (chart, space)
    }

    fn guard(surface: &ConicalSurface, chart: &[Point2], space: &[Point3]) -> ChordFoldGuard {
        ChordFoldGuard::new([(chart, space)], |p| surface.evaluate(p.x(), p.y()))
    }

    /// Whether `(p0, p1, q)`, counter-clockwise in the chart, is wound against
    /// the chart's orientation normal `S_u × S_v` at `q`, by closed form: with
    /// the carrier's frame `(x, y)` read off its `v = 1` circle and `â` its
    /// axis, `P = v (c ρ(u) + s â)`, `S_u = v c ρ'(u)`, `S_v = c ρ(u) + s â`,
    /// `ρ(u) = cos u x + sin u y`.
    fn turned_over(surface: &ConicalSurface, p0: Point3, p1: Point3, q: Point2) -> bool {
        let (u, v) = (q.x(), q.y());
        let (s, c) = surface.half_angle().sin_cos();
        let axis = Vec3::new(0.0, 0.0, 1.0);
        let apex = Point3::new(0.0, 0.0, 0.0);
        let x = (surface.evaluate(0.0, 1.0) - apex - axis * s) * (1.0 / c);
        let y = (surface.evaluate(std::f64::consts::FRAC_PI_2, 1.0) - apex - axis * s) * (1.0 / c);
        let rho = x * u.cos() + y * u.sin();
        let d_rho = y * u.cos() - x * u.sin();
        let su = d_rho * (v * c);
        let sv = rho * c + axis * s;
        (p1 - p0)
            .cross(surface.evaluate(u, v) - p0)
            .dot(su.cross(sv))
            < 0.0
    }

    /// The far rim chord's circular sagitta and the lens it leaves along the
    /// ruling: a chart point closer than the lens to the chord in `v` lies
    /// past the 3D chord.
    fn sagitta_and_lens(half_angle: f64, v1: f64, du: f64) -> (f64, f64) {
        let c = half_angle.cos();
        let sagitta = v1 * c * (1.0 - (0.5 * du).cos());
        (sagitta, sagitta * c)
    }

    /// A point inside the chart chord of the rim away from the apex, but
    /// within the lens, lies past the 3D chord: the closed form turns its
    /// triangle over, and the guard rejects it. Well inside, both keep it.
    #[test]
    fn a_point_inside_a_far_rim_chord_lens_folds() {
        let a = std::f64::consts::FRAC_PI_4;
        let n = 4;
        let surface = cone(a);
        let (chart, space) = band(&surface, 1.0, 3.0, n);
        let guard = guard(&surface, &chart, &space);
        assert_eq!(guard.chord_count(), chart.len());
        let du = std::f64::consts::FRAC_PI_2 / n as f64;
        let (_, lens) = sagitta_and_lens(a, 3.0, du);
        let u = chart[n + 1].x() - 0.5 * du;
        let (p0, p1) = (space[n + 1], space[n + 2]);
        let inside = Point2::new(u, 3.0 - 0.5 * lens);
        assert!(turned_over(&surface, p0, p1, inside));
        assert!(guard.folds(inside, surface.evaluate(u, inside.y())));
        let sound = Point2::new(u, 3.0 - 10.0 * lens);
        assert!(!turned_over(&surface, p0, p1, sound));
        assert!(!guard.folds(sound, surface.evaluate(u, sound.y())));
        // The rim nearer the apex leaves no lens on the face's side.
        let near = Point2::new(0.5 * du, 1.0 + 1e-9);
        assert!(!turned_over(&surface, space[0], space[1], near));
    }

    /// Over half angles, scales and chord counts: every candidate beside a
    /// far rim chord whose triangle the closed form turns over is rejected,
    /// and every one clear of the lens by twice the sagitta is kept.
    #[test]
    fn the_guard_rejects_every_turned_over_candidate_and_keeps_clear_ones() {
        let mut folded = 0;
        for a in [0.1, 0.5, std::f64::consts::FRAC_PI_4, 1.2, 1.5] {
            let surface = cone(a);
            for (v1, n) in [(1e-3, 3), (1.0, 2), (3.0, 4), (1e3, 7)] {
                let (chart, space) = band(&surface, 0.5 * v1, v1, n);
                let guard = guard(&surface, &chart, &space);
                let du = std::f64::consts::FRAC_PI_2 / n as f64;
                let (sagitta, lens) = sagitta_and_lens(a, v1, du);
                let (p0, p1) = (space[n + 1], space[n + 2]);
                let band_depth = (BAND * du).min(0.5 * v1);
                for k in 1..40 {
                    let u = chart[n + 1].x() - du * f64::from(k) / 40.0;
                    for step in 1..=60 {
                        let dv = band_depth * 0.7f64.powi(step);
                        let q = Point2::new(u, v1 - dv);
                        let point = surface.evaluate(u, q.y());
                        let verdict = guard.folds(q, point);
                        if turned_over(&surface, p0, p1, q) {
                            folded += 1;
                            assert!(verdict, "kept a fold: a {a} v1 {v1} n {n} u {u} dv {dv}");
                        } else if dv >= lens + 2.0 * sagitta {
                            assert!(
                                !verdict,
                                "dropped a sound point: a {a} v1 {v1} n {n} u {u} dv {dv}"
                            );
                        }
                    }
                }
            }
        }
        assert!(folded > 1000, "the sweep reached the lens {folded} times");
    }

    /// The verdict does not depend on which way a wire runs, so hole wires
    /// need no orientation of their own.
    #[test]
    fn the_verdict_does_not_depend_on_the_wire_direction() {
        let a = 0.7;
        let n = 3;
        let surface = cone(a);
        let (chart, space) = band(&surface, 2.0, 2.5, n);
        let forward = guard(&surface, &chart, &space);
        let (mut back_chart, mut back_space) = (chart, space);
        back_chart.reverse();
        back_space.reverse();
        let backward = guard(&surface, &back_chart, &back_space);
        let du = std::f64::consts::FRAC_PI_2 / n as f64;
        let (_, lens) = sagitta_and_lens(a, 2.5, du);
        let mut folds = 0;
        for k in 1..30 {
            let u = du * f64::from(k) / 10.0;
            for dv in [0.25 * lens, 0.9 * lens, 3.0 * lens, 0.05, 0.2] {
                for v in [2.5 - dv, 2.0 + dv] {
                    let q = Point2::new(u, v);
                    let point = surface.evaluate(u, v);
                    let verdict = forward.folds(q, point);
                    folds += usize::from(verdict);
                    assert_eq!(verdict, backward.folds(q, point), "u {u} v {v}");
                }
            }
        }
        assert!(folds > 0);
    }

    /// Off a chord's band or span the guard reads nothing; a candidate on the
    /// chart chord itself is degenerate and folds; a non-finite chart point is
    /// left to the other filters; a wire of two samples is no loop.
    #[test]
    fn the_band_and_span_bound_the_test() {
        let surface = cone(0.6);
        let (chart, space) = band(&surface, 1.0, 3.0, 4);
        let guard = guard(&surface, &chart, &space);
        let middle = Point2::new(std::f64::consts::FRAC_PI_4, 2.0);
        assert!(!guard.folds(middle, surface.evaluate(middle.x(), middle.y())));
        let on = Point2::new(std::f64::consts::PI / 16.0, 1.0);
        assert!(guard.folds(on, surface.evaluate(on.x(), on.y())));
        assert!(!guard.folds(Point2::new(f64::NAN, 1.0), space[0]));
        let single = ChordFoldGuard::new([(&chart[..2], &space[..2])], |p| {
            surface.evaluate(p.x(), p.y())
        });
        assert_eq!(single.chord_count(), 0, "a wire of two samples is no loop");
    }

    /// Degenerate and non-finite chords are left out.
    #[test]
    fn degenerate_chords_are_left_out() {
        let surface = cone(0.4);
        let p = Point2::new(0.0, 1.0);
        let chart = [p, p, Point2::new(f64::INFINITY, 1.0)];
        let point = surface.evaluate(0.0, 1.0);
        let space = [point, point, point];
        let guard = ChordFoldGuard::new([(&chart[..], &space[..])], |q| {
            surface.evaluate(q.x(), q.y())
        });
        assert_eq!(guard.chord_count(), 0);
        assert!(!guard.folds(Point2::new(0.0, 1.0 + 1e-9), point));
    }
}
