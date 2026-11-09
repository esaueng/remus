//! A metric chart for an open NURBS face whose parameter directions run
//! nearly parallel in space.
//!
//! A twisted extrusion charts `u` up its twist and `v` along the transverse
//! profile. On a thread flank the profile is a 127° spiral at constant height
//! and the twist a helix, two directions a few degrees apart in space, so a
//! grid that is uniform in `(u, v)` is a grid of thin parallelograms: an
//! M12 × 1.75 flank took 100 × 70 cells that split into 0.25 × 0.015 mm
//! slivers, whose facets tilt 6–13° off the surface they sample (a viewport
//! that shades from facets shows grain) and number 14 000 where the
//! deflection needs a tenth of that.
//!
//! The fix is the first fundamental form. With `E = Su·Su`, `F = Su·Sv`,
//! `G = Sv·Sv` averaged over the face's parameter box, the Cholesky factor
//! `R = [[√E, F/√E], [0, √(G − F²/E)]]` maps `(u, v)` to `(s, t) = R·(u, v)`,
//! a chart where a parameter step's length is its length in space (exactly
//! at the averaged metric, to first order elsewhere). The CDT and its
//! interior grid run in `(s, t)`, so cells and Delaunay triangles are shaped
//! by distances on the surface rather than by the parameterization. `R` is
//! upper triangular: rows of constant `t` are rows of constant `v`, so the
//! interior grid's row-wise containment scan works on the `(u, v)` boundary
//! unchanged.
//!
//! A chart whose directions meet at more than `acos(SHEAR_COS)` keeps the
//! established per-axis grid: every converted primitive (orthogonal
//! parameter lines) keeps its grid points.
//!
//! Such a chart still triangulates in surface lengths along each axis:
//! [`metric_v_scale`] is the diagonal of the same factor, `√(G/E)`, and the
//! CDT scales `v` by it instead of squaring up the parameter box. Knot units
//! say nothing about length: an in-place helical sweep's side face spans
//! `[0, 1]` across a unit half circle (π long) and `[0, 1]` along five turns
//! about an axis 10 away (≈ 314 long), and the squared-up box compressed
//! the surface's `v` 94-fold. Delaunay in that chart joined each cap sample
//! to the interior-grid column a hair beside it, up to nine rows along, and
//! those slivers (0.004 wide, up to 2 long) folded inside out on the bending
//! tube. Scaling an axis moves no grid or boundary point, so only the
//! connectivity changes (the coil keeps its exact triangle counts).

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point2, Vec3};

use super::{isotropic_rows, nurbs_open_axis_segments};

/// The chart is sheared when its iso-directions meet at under ~37°.
///
/// A parallelogram cell with interior angle `α` splits into triangles whose
/// short side is `sin α` of the long one; at 37° that is 0.6, about what a
/// uniform grid already gives a 1.7:1 cell, and below it the slivers grow
/// quickly (the thread flank's 6° gives 0.1). Conformal and orthogonal charts
/// (`F = 0`) never qualify.
const SHEAR_COS: f64 = 0.8;

/// Samples per direction for the averaged metric and the curvature bound.
const METRIC_SAMPLES: usize = 3;
const CURVATURE_SAMPLES: usize = 5;

/// The affine map `(s, t) = (a·u + b·v, c·v)` with `a, c > 0`.
#[derive(Clone, Copy, Debug)]
pub(super) struct ShearedChart {
    a: f64,
    b: f64,
    c: f64,
}

/// The interior grid of a sheared chart: `n_s` columns evenly spaced in `s`
/// over `[s_min, s_min + s_span]`, `n_v` rows evenly spaced in `v` (and so in
/// `t`).
#[derive(Clone, Copy, Debug)]
pub(super) struct ShearedGrid {
    pub(super) s_min: f64,
    pub(super) s_span: f64,
    pub(super) n_s: usize,
    pub(super) n_v: usize,
}

#[allow(clippy::cast_precision_loss)]
fn lattice(lo: f64, hi: f64, n: usize) -> impl Iterator<Item = f64> {
    (0..n).map(move |i| (hi - lo).mul_add((i as f64 + 0.5) / n as f64, lo))
}

impl ShearedChart {
    /// The metric chart of `surface` over the parameter box
    /// `(u_min, u_max, v_min, v_max)`, or `None` when the surface is closed
    /// in either direction, its metric degenerates, or its parameter lines
    /// are not sheared enough to need one.
    pub(super) fn detect(surface: &NurbsSurface, bounds: (f64, f64, f64, f64)) -> Option<Self> {
        if surface.is_periodic_u() || surface.is_periodic_v() {
            return None;
        }
        let (u_min, u_max, v_min, v_max) = bounds;
        let (mut e, mut f, mut g) = (0.0_f64, 0.0_f64, 0.0_f64);
        for u in lattice(u_min, u_max, METRIC_SAMPLES) {
            for v in lattice(v_min, v_max, METRIC_SAMPLES) {
                let d = surface.derivatives(u, v, 1);
                let (su, sv) = (d[1][0], d[0][1]);
                e += su.dot(su);
                f += su.dot(sv);
                g += sv.dot(sv);
            }
        }
        #[allow(clippy::cast_precision_loss)]
        let n = (METRIC_SAMPLES * METRIC_SAMPLES) as f64;
        let (e, f, g) = (e / n, f / n, g / n);
        if !(e.is_finite() && f.is_finite() && g.is_finite()) || e <= 0.0 || g <= 0.0 {
            return None;
        }
        if f.abs() <= SHEAR_COS * (e * g).sqrt() {
            return None;
        }
        let a = e.sqrt();
        let b = f / a;
        let c2 = b.mul_add(-b, g);
        // EG − F² > 0 on a regular surface; a vanishing determinant is a
        // degenerate chart the per-axis grid already handles.
        if c2 <= 1e-12 * g {
            return None;
        }
        Some(Self { a, b, c: c2.sqrt() })
    }

    pub(super) fn forward(self, u: f64, v: f64) -> Point2 {
        Point2::new(self.a.mul_add(u, self.b * v), self.c * v)
    }

    pub(super) fn inverse(self, p: Point2) -> (f64, f64) {
        let v = p.y() / self.c;
        (self.u_at(p.x(), v), v)
    }

    /// The `u` of chart point `(s, c·v)`.
    pub(super) fn u_at(self, s: f64, v: f64) -> f64 {
        self.b.mul_add(-v, s) / self.a
    }

    /// Chart bounding box of the parameter box.
    pub(super) fn bounds(self, u: (f64, f64), v: (f64, f64)) -> (Point2, Point2) {
        let corners = [
            self.forward(u.0, v.0),
            self.forward(u.1, v.0),
            self.forward(u.0, v.1),
            self.forward(u.1, v.1),
        ];
        corners.iter().fold(
            (
                Point2::new(f64::INFINITY, f64::INFINITY),
                Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
            ),
            |(lo, hi), p| {
                (
                    Point2::new(lo.x().min(p.x()), lo.y().min(p.y())),
                    Point2::new(hi.x().max(p.x()), hi.y().max(p.y())),
                )
            },
        )
    }

    /// Size the interior grid in chart units, which are lengths on the
    /// surface: each direction takes the chord count its curvature needs over
    /// the boundary's chart extent, and a flat direction matches the other's
    /// spacing (the NURBS arm of `interior_grid_resolution`, with the chart's
    /// directions in place of the parameter lines).
    pub(super) fn grid(
        self,
        surface: &NurbsSurface,
        boundary_uv: &[(f64, f64)],
        bounds: (f64, f64, f64, f64),
        deflection: f64,
        angular_tol: f64,
    ) -> ShearedGrid {
        let (u_min, u_max, v_min, v_max) = bounds;
        let (s_min, s_max) = boundary_uv
            .iter()
            .map(|&(u, v)| self.forward(u, v).x())
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), s| {
                (lo.min(s), hi.max(s))
            });
        let s_span = s_max - s_min;
        let t_span = self.c * (v_max - v_min);
        // Unit chart steps along s and t, in (u, v).
        let along_s = (1.0 / self.a, 0.0);
        let along_t = (-self.b / (self.a * self.c), 1.0 / self.c);
        let (mut k_s, mut k_t) = (0.0_f64, 0.0_f64);
        for u in lattice(u_min, u_max, CURVATURE_SAMPLES) {
            for v in lattice(v_min, v_max, CURVATURE_SAMPLES) {
                let d = surface.derivatives(u, v, 2);
                k_s = k_s.max(curvature(&d, along_s));
                k_t = k_t.max(curvature(&d, along_t));
            }
        }
        let own =
            |k: f64, len: f64| nurbs_open_axis_segments(k * len, len, deflection, angular_tol);
        let (own_s, own_t) = (own(k_s, s_span), own(k_t, t_span));
        let n_s = own_s.unwrap_or_else(|| isotropic_rows(s_span, own_t, t_span));
        let n_v = own_t.unwrap_or_else(|| isotropic_rows(t_span, own_s, s_span));
        ShearedGrid {
            s_min,
            s_span,
            n_s,
            n_v,
        }
    }
}

/// The CDT's `v` scale that makes a parameter step along `v` as long, in
/// chart units, as it is on the surface relative to a step along `u`:
/// `√(G/E)` of the first fundamental form averaged over the parameter box
/// `(u_min, u_max, v_min, v_max)`. `None` when the averaged metric is
/// degenerate (a direction of zero speed everywhere sampled), where the
/// caller keeps the squared-up parameter box.
pub(super) fn metric_v_scale(surface: &NurbsSurface, bounds: (f64, f64, f64, f64)) -> Option<f64> {
    let (u_min, u_max, v_min, v_max) = bounds;
    let (mut e, mut g) = (0.0_f64, 0.0_f64);
    for u in lattice(u_min, u_max, METRIC_SAMPLES) {
        for v in lattice(v_min, v_max, METRIC_SAMPLES) {
            let d = surface.derivatives(u, v, 1);
            let (su, sv) = (d[1][0], d[0][1]);
            e += su.dot(su);
            g += sv.dot(sv);
        }
    }
    let scale = (g / e).sqrt();
    (e > 0.0 && g > 0.0 && scale.is_finite() && scale > 0.0).then_some(scale)
}

/// Curvature of the surface curve through a point in parameter direction
/// `(du, dv)`, from the derivative table `d` (orders up to two) there.
fn curvature(d: &[Vec<Vec3>], (du, dv): (f64, f64)) -> f64 {
    let first = d[1][0] * du + d[0][1] * dv;
    let second = d[2][0] * (du * du) + d[1][1] * (2.0 * du * dv) + d[0][2] * (dv * dv);
    let speed = first.length();
    if speed.is_nan() || speed <= 0.0 {
        return 0.0;
    }
    let k = first.cross(second).length() / (speed * speed * speed);
    if k.is_finite() { k } else { 0.0 }
}
