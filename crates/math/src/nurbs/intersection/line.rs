//! Line (ray) - NURBS surface intersection.

use crate::MathError;
use crate::nurbs::surface::{DerivativeScratch, NurbsSurface};
use crate::vec::{Point3, Vec3};

use super::{IntersectionPoint, MAX_NEWTON_ITER};
use crate::fma::FusedMulAdd;

mod subdivision;

#[cfg(test)]
mod bump_oracle_tests;

/// Intersect a line (ray) with a NURBS surface.
///
/// Returns all intersection points along the ray (the whole line, behind
/// the origin too).
///
/// Newton refines seeds from an `n × n` sample grid first; those roots come
/// first, in grid order. A completion pass then isolates crossings by
/// Bézier subdivision of the surface against the line and appends any the
/// grid missed (a crossing whose Newton basin holds no grid seed, such as
/// the far flank of a bump narrower than about two grid cells). It accepts
/// only residuals at the round-off floor of the coordinates involved, so it
/// adds genuine crossings, never near-misses, whatever the surface's size,
/// shape or position.
///
/// # Parameters
///
/// - `surface`: The NURBS surface
/// - `ray_origin`: Starting point of the ray
/// - `ray_dir`: Direction of the ray
/// - `samples`: Grid resolution for finding seed points
///
/// # Errors
///
/// Returns an error if evaluation or refinement fails.
pub fn intersect_line_nurbs(
    surface: &NurbsSurface,
    ray_origin: Point3,
    ray_dir: Vec3,
    samples: usize,
) -> Result<Vec<IntersectionPoint>, MathError> {
    if ray_dir.dot(ray_dir) < 1e-20 {
        return Err(MathError::ZeroVector);
    }
    let grid = LineSurfaceSeedGrid::for_surface(surface, samples);
    intersect_line_nurbs_with_grid(surface, &grid, ray_origin, ray_dir)
}

/// The seed grid [`intersect_line_nurbs`] samples, evaluated once and
/// reusable across many lines.
///
/// The grid nodes and each node's local spacing (the largest distance to a
/// grid neighbour) are properties of the surface alone; only the
/// node-to-line distances depend on the line. [`intersect_line_nurbs`]
/// rebuilds all `n²` surface evaluations on every call, so a caller casting
/// many rays at one surface (point classification casts several per query
/// point) can build this once and hand it to
/// [`intersect_line_nurbs_with_grid`], which then costs `n²` distance
/// comparisons plus the Newton refinements — the same answer, bit for bit,
/// because it is the same grid, scanned in the same order, with the same
/// spacing thresholds. It also holds the completion pass's per-surface
/// span-block bounds, about `2·⌈S/16⌉` boxes for `S` knot spans.
#[derive(Debug, Clone)]
pub struct LineSurfaceSeedGrid {
    /// Resolution per direction (`samples.max(10)`).
    n: usize,
    u_min: f64,
    u_step: f64,
    v_min: f64,
    v_step: f64,
    /// `(S(u_i, v_j), local spacing)` in row-major `(i, j)` order.
    nodes: Vec<(Point3, f64)>,
    /// Largest coordinate magnitude over the non-NaN nodes, the scale of the
    /// seed scan's prefilter margin (see [`SeedPrefilter`]).
    max_abs: f64,
    /// Span-hull data of the completion pass ([`subdivision`]).
    hull: subdivision::SpanHull,
}

impl LineSurfaceSeedGrid {
    /// Evaluate the seed grid [`intersect_line_nurbs`] uses for `samples`.
    #[must_use]
    pub fn for_surface(surface: &NurbsSurface, samples: usize) -> Self {
        let n = samples.max(10);

        let (u_min, u_max) = surface.domain_u();
        let (v_min, v_max) = surface.domain_v();

        #[allow(clippy::cast_precision_loss)]
        let u_step = (u_max - u_min) / (n - 1) as f64;
        #[allow(clippy::cast_precision_loss)]
        let v_step = (v_max - v_min) / (n - 1) as f64;

        // Evaluate the seed grid once and keep it: deciding whether a sample
        // is close enough to the ray requires knowing how far apart the
        // samples actually are.
        #[allow(clippy::cast_precision_loss)]
        let grid: Vec<Vec<Point3>> = (0..n)
            .map(|i| {
                let u = u_min + i as f64 * u_step;
                (0..n)
                    .map(|j| surface.evaluate(u, v_min + j as f64 * v_step))
                    .collect()
            })
            .collect();

        let mut nodes = Vec::with_capacity(n * n);
        for i in 0..n {
            for j in 0..n {
                let pt = grid[i][j];
                // A ray piercing the surface lands somewhere inside a grid
                // cell, so the nearest sample to it can be half a cell
                // diagonal away. The test therefore has to be against the
                // LOCAL sample spacing.
                //
                // It used to be the corner-to-corner diagonal over `n`, which
                // is not a spacing at all: it is unrelated to the grid's
                // actual density, and it COLLAPSES for any surface whose two
                // corners coincide -- precisely what a closed one does --
                // landing on the `.max(0.1)` floor. A torus then seeded at 0.1
                // against a real spacing of 0.657 and missed 74% of its
                // intersections; a cylinder seeded at 0.500 against 1.567 and
                // missed 30%. Both failed silently, as an empty result. A box
                // happened to work only because its diagonal came out larger
                // than its spacing (0.849 vs 0.632).
                //
                // Half a cell diagonal is at most `sqrt(2)/2` of the larger of
                // the two adjacent spacings, so the neighbour distance covers
                // it with ~40% to spare, and adapts where the grid stretches
                // or collapses (a sphere's poles).
                let mut spacing = 0.0_f64;
                if i > 0 {
                    spacing = spacing.max((pt - grid[i - 1][j]).length());
                }
                if i + 1 < n {
                    spacing = spacing.max((pt - grid[i + 1][j]).length());
                }
                if j > 0 {
                    spacing = spacing.max((pt - grid[i][j - 1]).length());
                }
                if j + 1 < n {
                    spacing = spacing.max((pt - grid[i][j + 1]).length());
                }
                nodes.push((pt, spacing));
            }
        }

        let max_abs = nodes.iter().fold(0.0_f64, |m, (p, _)| {
            m.max(p.x().abs()).max(p.y().abs()).max(p.z().abs())
        });
        Self {
            n,
            u_min,
            u_step,
            v_min,
            v_step,
            nodes,
            max_abs,
            hull: subdivision::SpanHull::for_surface(surface),
        }
    }
}

/// An exact, conservative pre-test for the seed scan's candidate test.
///
/// The scan keeps a node when `dist < spacing`, with `dist` the length of
/// `pt - closest_on_ray` computed through fused multiply-adds (a software
/// call on `wasm32`). Nearly every node lies far from the ray, so this
/// discards those first with plain arithmetic: it skips a node when
/// `|d × r|² > (spacing + m)² · |r|²`, where `d = pt - origin` (the same
/// rounded difference the scan uses), `r` the direction and `m` the margin
/// below. A skipped node is one the scan's own test rejects, so the
/// candidates, their order and everything after are unchanged.
///
/// Why: with `u = 2⁻⁵³`, `M = max |pt|∞ + |origin|∞` and `e` the exact
/// distance from `pt` to the line, rounding in the cross product, its
/// squared norm and the bound's two products keeps the skip condition
/// within `e ≥ (spacing + m)(1 - 5u) - 13u·M`, while the scan's
/// `t = d·r / |r|²`, `closest = origin + r·t` (with `|t|·|r|∞ ≤ 1.74 M`),
/// `pt - closest` and the fused length give `dist ≥ e(1 - 4u) - 8u·M`.
/// Together `dist ≥ spacing + m - 9u(spacing + m) - 21u·M`, and since
/// `spacing ≤ 2√3 M` (both nodes of a spacing are grid nodes) it suffices
/// that `m ≥ 52u·M ≈ 5.8e-15·M`. The margin is `1e-9·M`, five orders of
/// magnitude more, plus `1e-100` absolute so that every quantity compared
/// stays far above the subnormal range, where gradual underflow's absolute
/// errors (at most `2⁻¹⁰⁷⁵` per operation) are negligible.
///
/// The bound assumes no overflow: the filter is off unless `M ≤ 1e100` and
/// `|r|² ≤ 1e100` (`|r|² ≥ 1e-20` holds already), which keeps every product
/// below `1e302`. A NaN anywhere fails a comparison, which also leaves the
/// node to the scan's own test.
#[derive(Clone, Copy)]
struct SeedPrefilter {
    enabled: bool,
    margin: f64,
    direction: Vec3,
    dir_len_sq: f64,
}

impl SeedPrefilter {
    /// Largest `M` and `|r|²` the error bound above covers.
    const MAX_SCALE: f64 = 1e100;
    /// Margin relative to `M`, five orders of magnitude above the bound.
    const RELATIVE_MARGIN: f64 = 1e-9;
    /// Absolute margin that keeps the compared squares normal.
    const ABSOLUTE_MARGIN: f64 = 1e-100;

    fn new(grid: &LineSurfaceSeedGrid, origin: Point3, direction: Vec3, dir_len_sq: f64) -> Self {
        let scale = grid.max_abs + origin.x().abs().max(origin.y().abs()).max(origin.z().abs());
        // `f64::max` skips a NaN coordinate, so test the origin itself.
        let finite_origin =
            origin.x().is_finite() && origin.y().is_finite() && origin.z().is_finite();
        Self {
            enabled: finite_origin && scale <= Self::MAX_SCALE && dir_len_sq <= Self::MAX_SCALE,
            margin: scale.fma(Self::RELATIVE_MARGIN, Self::ABSOLUTE_MARGIN),
            direction,
            dir_len_sq,
        }
    }

    /// Whether the scan's test certainly rejects the node at `diff = pt -
    /// origin` with local `spacing`.
    // Plain products and sums on purpose: fused ones are what this avoids,
    // and the margin covers the rounding of either.
    #[allow(clippy::suboptimal_flops)]
    #[inline]
    fn rejects(&self, diff: Vec3, spacing: f64) -> bool {
        if !self.enabled {
            return false;
        }
        let (dx, dy, dz) = (diff.x(), diff.y(), diff.z());
        let (rx, ry, rz) = (self.direction.x(), self.direction.y(), self.direction.z());
        let cx = dy * rz - dz * ry;
        let cy = dz * rx - dx * rz;
        let cz = dx * ry - dy * rx;
        let bound = spacing + self.margin;
        cx * cx + cy * cy + cz * cz > bound * bound * self.dir_len_sq
    }
}

/// The seed scan of [`intersect_line_nurbs_with_grid`]: every grid node
/// within its local spacing of the line, as `(u, v, node)` in scan order.
fn seed_candidates(
    grid: &LineSurfaceSeedGrid,
    ray_origin: Point3,
    ray_dir: Vec3,
    dir_len_sq: f64,
) -> Vec<(f64, f64, Point3)> {
    let mut candidates = Vec::new();
    let prefilter = SeedPrefilter::new(grid, ray_origin, ray_dir, dir_len_sq);
    let n = grid.n;
    #[allow(clippy::cast_precision_loss)]
    for i in 0..n {
        let u = grid.u_min + i as f64 * grid.u_step;
        for j in 0..n {
            let v = grid.v_min + j as f64 * grid.v_step;
            let (pt, spacing) = grid.nodes[i * n + j];
            let diff = pt - ray_origin;
            let diff_vec = Vec3::new(diff.x(), diff.y(), diff.z());
            if prefilter.rejects(diff_vec, spacing) {
                continue;
            }

            // Project onto ray.
            let t = diff_vec.dot(ray_dir) / dir_len_sq;
            // `mul_add` by one, not `fma`: LLVM folds it to the plain
            // (exact) addition, which an out-of-line `fma` call would hide.
            let closest_on_ray = Point3::new(
                ray_origin.x().mul_add(1.0, ray_dir.x() * t),
                ray_origin.y().mul_add(1.0, ray_dir.y() * t),
                ray_origin.z().mul_add(1.0, ray_dir.z() * t),
            );

            let dist = (pt - closest_on_ray).length();

            // Compare against the node's local spacing (see
            // `LineSurfaceSeedGrid::for_surface`).
            if dist < spacing {
                // Rough candidate.
                candidates.push((u, v, pt));
            }
        }
    }
    candidates
}

/// [`intersect_line_nurbs`] with the seed grid supplied rather than rebuilt.
///
/// Returns exactly what [`intersect_line_nurbs`] would, provided `grid` was
/// built from `surface` with the same `samples`: same nodes, same scan
/// order, same spacing thresholds, so the same candidates, the same Newton
/// refinements and the same deduplication. A grid built from a different
/// surface is not unsafe but is meaningless.
///
/// # Errors
///
/// Returns [`MathError::ZeroVector`] for a degenerate ray direction.
pub fn intersect_line_nurbs_with_grid(
    surface: &NurbsSurface,
    grid: &LineSurfaceSeedGrid,
    ray_origin: Point3,
    ray_dir: Vec3,
) -> Result<Vec<IntersectionPoint>, MathError> {
    let dir_len_sq = ray_dir.dot(ray_dir);
    if dir_len_sq < 1e-20 {
        return Err(MathError::ZeroVector);
    }
    // Sample surface points and compute distance to ray.
    let candidates = seed_candidates(grid, ray_origin, ray_dir, dir_len_sq);

    // Refinements share only immutable surface evaluations, never Newton
    // outcomes: the remaining iteration budget can differ at the same UV.
    let mut workspace = NewtonWorkspace::default();
    let line = LineQuery {
        origin: ray_origin,
        direction: ray_dir,
        dir_len_sq,
    };
    let (u_min, u_max) = surface.domain_u();
    let (v_min, v_max) = surface.domain_v();
    let domain = [u_min, u_max, v_min, v_max];
    let mut results: Vec<IntersectionPoint> = Vec::new();
    for &(u_guess, v_guess, seed_point) in &candidates {
        if let Some(pt) = refine_line_surface_point(
            surface,
            line,
            (u_guess, v_guess),
            seed_point,
            domain,
            GRID_CONVERGENCE,
            GRID_ROOT_TOLERANCE,
            &mut workspace,
        ) {
            // Deduplicate: skip if close to an existing result.
            let dominated = results
                .iter()
                .any(|existing| (existing.point - pt.point).length() < 1e-6);
            if !dominated {
                results.push(pt);
            }
        }
    }

    // Roots whose Newton basin no grid seed reached (a narrow bump's far
    // flank). Appended after the grid roots, which keep their bits and order.
    subdivision::complete_line_roots(surface, &grid.hull, line, &mut workspace, &mut results);

    Ok(results)
}

/// Convergence target and final-check residual of a grid-seeded refinement.
const GRID_CONVERGENCE: f64 = 1e-10;
const GRID_ROOT_TOLERANCE: f64 = 1e-5;

/// Bound on how far from the line a reported hit can lie.
///
/// Covers every hit [`intersect_line_nurbs_with_grid`] reports on a surface
/// whose control points have no coordinate larger than `magnitude` in
/// absolute value, for a line through `origin`. Every hit is a surface
/// evaluation at an in-domain parameter: seeds and Newton iterates are
/// clamped to the domain. The grid pass keeps a hit closer than
/// `GRID_ROOT_TOLERANCE` to the line, and the completion pass one within its
/// round-off allowance, which grows with the coordinates involved
/// (`subdivision::hit_reach`). The sum covers both, with the rounding of
/// their residuals, so a caller that bounds the surface may skip a line that
/// stays farther than this from the bound.
#[must_use]
pub fn line_hit_reach(magnitude: f64, origin: Point3) -> f64 {
    GRID_ROOT_TOLERANCE + subdivision::hit_reach(magnitude, origin)
}

#[derive(Clone, Copy)]
struct LineQuery {
    origin: Point3,
    direction: Vec3,
    dir_len_sq: f64,
}

// Bounded, call-local storage. UV identity is exact; a collision only causes
// another evaluation. No topology/surface identity or invalidation is needed:
// one workspace is created for one immutable surface and one line query.
const EVALUATION_CACHE_SIZE: usize = 64;

#[derive(Clone, Copy)]
struct SurfaceEvaluation {
    u_bits: u64,
    v_bits: u64,
    occupied: bool,
    has_partials: bool,
    point: Point3,
    su: Vec3,
    sv: Vec3,
}

impl SurfaceEvaluation {
    const EMPTY: Self = Self {
        u_bits: 0,
        v_bits: 0,
        occupied: false,
        has_partials: false,
        point: Point3::new(0.0, 0.0, 0.0),
        su: Vec3::new(0.0, 0.0, 0.0),
        sv: Vec3::new(0.0, 0.0, 0.0),
    };

    fn matches(&self, u: f64, v: f64) -> bool {
        self.occupied && self.u_bits == u.to_bits() && self.v_bits == v.to_bits()
    }
}

/// Reusable surface-position/first-partial evaluations for the refinements.
/// Position and partials retain their separate arithmetic: the derivative
/// solve's position does not have the same bits as `surface.evaluate`, so a
/// position solve computes `evaluate`'s own bits and the partials from one
/// shared basis ([`DerivativeScratch::evaluate_and_partials_from`]).
struct NewtonWorkspace {
    scratch: DerivativeScratch,
    evaluations: [SurfaceEvaluation; EVALUATION_CACHE_SIZE],
    #[cfg(test)]
    stats: EvaluationStats,
}

impl Default for NewtonWorkspace {
    fn default() -> Self {
        Self {
            scratch: DerivativeScratch::default(),
            evaluations: [SurfaceEvaluation::EMPTY; EVALUATION_CACHE_SIZE],
            #[cfg(test)]
            stats: EvaluationStats::default(),
        }
    }
}

impl NewtonWorkspace {
    fn index(u: f64, v: f64) -> usize {
        let hash = u
            .to_bits()
            .wrapping_mul(0x9e37_79b9_7f4a_7c15)
            .rotate_left(17)
            ^ v.to_bits().wrapping_mul(0xbf58_476d_1ce4_e5b9);
        // Six high bits also support the measured 16/32/64-entry variants.
        ((hash >> 58) as usize) & (EVALUATION_CACHE_SIZE - 1)
    }

    fn store_point(&mut self, index: usize, u: f64, v: f64, point: Point3) {
        if self.evaluations[index].matches(u, v) {
            return;
        }
        self.evaluations[index] = SurfaceEvaluation {
            u_bits: u.to_bits(),
            v_bits: v.to_bits(),
            occupied: true,
            point,
            ..SurfaceEvaluation::EMPTY
        };
    }

    fn seed_point(&mut self, u: f64, v: f64, point: Point3) {
        // The grid evaluated exactly these parameter bits on this surface.
        // Reusing that point needs neither a surface solve nor a cache hit.
        self.store_point(Self::index(u, v), u, v, point);
        #[cfg(test)]
        {
            self.stats.seed_points += 1;
        }
    }

    fn point(&mut self, surface: &NurbsSurface, u: f64, v: f64) -> Point3 {
        let index = Self::index(u, v);
        let cached = &self.evaluations[index];
        if cached.matches(u, v) {
            #[cfg(test)]
            {
                self.stats.point_hits += 1;
            }
            return cached.point;
        }
        // A missed position is nearly always followed by the partials at the
        // same UV (every iteration that does not converge), so solve both
        // from one basis and keep both.
        let (point, su, sv) = self.scratch.evaluate_and_partials_from(surface, u, v);
        self.evaluations[index] = SurfaceEvaluation {
            u_bits: u.to_bits(),
            v_bits: v.to_bits(),
            occupied: true,
            has_partials: true,
            point,
            su,
            sv,
        };
        #[cfg(test)]
        {
            self.stats.point_evaluations += 1;
        }
        point
    }

    fn partials(&mut self, surface: &NurbsSurface, u: f64, v: f64) -> (Vec3, Vec3) {
        let index = Self::index(u, v);
        let cached = &self.evaluations[index];
        if cached.matches(u, v) && cached.has_partials {
            #[cfg(test)]
            {
                self.stats.partial_hits += 1;
            }
            return (cached.su, cached.sv);
        }
        let partials = self.scratch.partials_from(surface, u, v);
        // The caller has just obtained the position at this UV. Keep the
        // distinction explicit instead of using the derivative solve's point.
        if self.evaluations[index].matches(u, v) {
            self.evaluations[index].su = partials.0;
            self.evaluations[index].sv = partials.1;
            self.evaluations[index].has_partials = true;
        }
        #[cfg(test)]
        {
            self.stats.partial_evaluations += 1;
        }
        partials
    }
}

#[cfg(test)]
#[derive(Default)]
struct EvaluationStats {
    seed_points: usize,
    /// Position solves; each also yields the partials at its UV.
    point_evaluations: usize,
    /// Partial-only solves (at a UV whose position came from the seed grid).
    partial_evaluations: usize,
    point_hits: usize,
    partial_hits: usize,
}

/// Where the iteration budget ends on a cycle: the index into the visited
/// states of the state after `MAX_NEWTON_ITER` updates, for a sequence whose
/// state after `updates` updates repeats the state after `first`.
///
/// From `first` on the sequence is periodic with period `updates - first`, so
/// the state after `MAX_NEWTON_ITER` updates is the one after
/// `first + (MAX_NEWTON_ITER - first) % period`, which is below `updates`.
const fn budget_end_index(first: usize, updates: usize) -> usize {
    first + (MAX_NEWTON_ITER - first) % (updates - first)
}

/// Refine a line-surface intersection point using Newton iteration.
///
/// Each iteration is a pure function of the current `(u, v)` bits. Once an
/// update reproduces a state the loop already passed through (a fixed point,
/// or a cycle of any period), every remaining iteration only repeats that
/// cycle — each of its states already failed the convergence test — so the
/// state the iteration budget would end on is the cycle member at the same
/// phase. The loop jumps there and goes straight to the final check, which
/// then sees exactly the `(u, v)` exhausting the budget would have left.
///
/// Iterates stay within `bounds` (`[u_min, u_max, v_min, v_max]`). Iteration
/// stops once the distance to the line is below the smallest of `1e-10`,
/// `converge` and `final_tolerance`; a point that has not converged within
/// the budget is accepted when its distance is below `final_tolerance`. The
/// completion pass converges to its strict perpendicular bound while keeping
/// a looser final check, which far from the origin must admit rounding
/// along the line.
#[allow(clippy::too_many_arguments)]
fn refine_line_surface_point(
    surface: &NurbsSurface,
    line: LineQuery,
    guess: (f64, f64),
    seed_point: Point3,
    bounds: [f64; 4],
    converge: f64,
    final_tolerance: f64,
    workspace: &mut NewtonWorkspace,
) -> Option<IntersectionPoint> {
    let LineQuery {
        origin: ray_origin,
        direction: ray_dir,
        dir_len_sq,
    } = line;
    let (mut u, mut v) = guess;
    workspace.seed_point(u, v, seed_point);
    let [u_min, u_max, v_min, v_max] = bounds;
    // The grid pass leaves this at 1e-10; a tighter target must not be
    // bypassed by an early exit.
    let converged = converge.min(final_tolerance).min(1e-10);
    // `visited[k]`: the `(u, v)` bits after `k` updates.
    let mut visited = [(0_u64, 0_u64); MAX_NEWTON_ITER + 1];
    visited[0] = (u.to_bits(), v.to_bits());

    for iteration in 0..MAX_NEWTON_ITER {
        let pt = if iteration == 0 {
            seed_point
        } else {
            workspace.point(surface, u, v)
        };
        let diff = pt - ray_origin;
        let diff_vec = Vec3::new(diff.x(), diff.y(), diff.z());

        // Closest t on ray.
        let t = diff_vec.dot(ray_dir) / dir_len_sq;
        let ray_pt = Point3::new(
            ray_dir.x().fma(t, ray_origin.x()),
            ray_dir.y().fma(t, ray_origin.y()),
            ray_dir.z().fma(t, ray_origin.z()),
        );

        let residual = pt - ray_pt;
        if residual.length() < converged {
            return Some(IntersectionPoint {
                point: pt,
                param1: (u, v),
                param2: (t, 0.0),
            });
        }

        // Keep `evaluate` above for the position and residual: the derivative
        // solve's position uses a different arithmetic order. Only substitute
        // the partials, whose bits match the general derivative-table solve.
        let (su, sv) = workspace.partials(surface, u, v);

        let r = Vec3::new(residual.x(), residual.y(), residual.z());

        // The quantity being driven to zero is the distance to the ray LINE, so
        // only the ray-PERPENDICULAR part of a tangent reduces it -- sliding the
        // surface point along the ray moves it without getting it any closer.
        // Build the Gauss-Newton system from those projected tangents. Using the
        // raw su/sv inflates the matrix by the ray-parallel component and
        // under-relaxes every step: on a plane, where the projected system lands
        // exactly in one iteration, the raw one still has not converged after
        // 100 and so gives up at MAX_NEWTON_ITER with the intersection
        // undiscovered. `r` is already perpendicular to the ray, so the
        // right-hand side is unchanged in exact arithmetic; it is projected here
        // too to keep the system self-consistent.
        let ju = su - ray_dir * (su.dot(ray_dir) / dir_len_sq);
        let jv = sv - ray_dir * (sv.dot(ray_dir) / dir_len_sq);

        // Solve 2x2 system: [ju*ju, ju*jv; jv*ju, jv*jv] * [du, dv] = [ju*r, jv*r]
        let a11 = ju.dot(ju);
        let a12 = ju.dot(jv);
        let a22 = jv.dot(jv);
        let b1 = ju.dot(r);
        let b2 = jv.dot(r);

        let det = a11.fma(a22, -(a12 * a12));
        // Relative singularity threshold -- catches surface poles/apex where
        // derivatives shrink to zero (making absolute 1e-20 too lenient).
        let (du, dv) = if det.abs() < (a11 + a22).max(1e-30) * 1e-12 {
            // Near-degenerate: Tikhonov regularization.
            let lambda = (a11 + a22).max(1e-10) * 1e-4;
            let a11r = a11 + lambda;
            let a22r = a22 + lambda;
            let det_r = a11r.fma(a22r, -(a12 * a12));
            if det_r.abs() < 1e-30 {
                // Truly singular -- step along the non-degenerate direction only.
                if a11 > a22 {
                    (b1 / a11.max(1e-30), 0.0)
                } else if a22 > 1e-30 {
                    (0.0, b2 / a22.max(1e-30))
                } else {
                    break;
                }
            } else {
                (
                    b1.fma(a22r, -(b2 * a12)) / det_r,
                    a11r.fma(b2, -(a12 * b1)) / det_r,
                )
            }
        } else {
            (
                b1.fma(a22, -(b2 * a12)) / det,
                a11.fma(b2, -(a12 * b1)) / det,
            )
        };

        u -= du;
        v -= dv;
        u = u.clamp(u_min, u_max);
        v = v.clamp(v_min, v_max);

        let updates = iteration + 1;
        let state = (u.to_bits(), v.to_bits());
        if let Some(first) = visited[..updates].iter().position(|&seen| seen == state) {
            let (u_end, v_end) = visited[budget_end_index(first, updates)];
            u = f64::from_bits(u_end);
            v = f64::from_bits(v_end);
            break;
        }
        visited[updates] = state;
    }

    // Final check.
    let pt = workspace.point(surface, u, v);
    let diff = pt - ray_origin;
    let diff_vec = Vec3::new(diff.x(), diff.y(), diff.z());
    let t = diff_vec.dot(ray_dir) / dir_len_sq;
    let ray_pt = Point3::new(
        ray_dir.x().fma(t, ray_origin.x()),
        ray_dir.y().fma(t, ray_origin.y()),
        ray_dir.z().fma(t, ray_origin.z()),
    );

    if (pt - ray_pt).length() < final_tolerance {
        Some(IntersectionPoint {
            point: pt,
            param1: (u, v),
            param2: (t, 0.0),
        })
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    //! The seeded-grid entry point, the reused derivative workspace and the
    //! Newton cycle skip must reproduce the pre-O06 algorithm bit for bit.
    //! `reference_*` below is that algorithm verbatim: a fresh grid per call,
    //! an allocating `derivatives` per iteration, and the full iteration
    //! budget on every candidate.

    #![allow(clippy::unwrap_used, clippy::cast_precision_loss)]

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
        exhausted: usize,
        /// Exhausted runs that revisited a state with period one.
        fixed: usize,
        /// Exhausted runs that revisited a state with a longer period.
        cycled: usize,
    }

    fn reference_refine(
        surface: &NurbsSurface,
        ray_origin: Point3,
        ray_dir: Vec3,
        u_guess: f64,
        v_guess: f64,
        stats: &mut ReferenceStats,
    ) -> Option<IntersectionPoint> {
        let mut u = u_guess;
        let mut v = v_guess;
        let (u_min, u_max) = surface.domain_u();
        let (v_min, v_max) = surface.domain_v();
        let mut seen = vec![(u.to_bits(), v.to_bits())];
        let mut period = None;

        for _ in 0..MAX_NEWTON_ITER {
            let pt = surface.evaluate(u, v);
            let diff = pt - ray_origin;
            let diff_vec = Vec3::new(diff.x(), diff.y(), diff.z());
            let t = diff_vec.dot(ray_dir) / ray_dir.dot(ray_dir);
            let ray_pt = Point3::new(
                ray_dir.x().mul_add(t, ray_origin.x()),
                ray_dir.y().mul_add(t, ray_origin.y()),
                ray_dir.z().mul_add(t, ray_origin.z()),
            );
            let residual = pt - ray_pt;
            if residual.length() < 1e-10 {
                return Some(IntersectionPoint {
                    point: pt,
                    param1: (u, v),
                    param2: (t, 0.0),
                });
            }
            let derivs = surface.derivatives(u, v, 1);
            let su = derivs[1][0];
            let sv = derivs[0][1];
            let r = Vec3::new(residual.x(), residual.y(), residual.z());
            let dir_dot_dir = ray_dir.dot(ray_dir);
            let ju = su - ray_dir * (su.dot(ray_dir) / dir_dot_dir);
            let jv = sv - ray_dir * (sv.dot(ray_dir) / dir_dot_dir);
            let a11 = ju.dot(ju);
            let a12 = ju.dot(jv);
            let a22 = jv.dot(jv);
            let b1 = ju.dot(r);
            let b2 = jv.dot(r);
            let det = a11.mul_add(a22, -(a12 * a12));
            let (du, dv) = if det.abs() < (a11 + a22).max(1e-30) * 1e-12 {
                let lambda = (a11 + a22).max(1e-10) * 1e-4;
                let a11r = a11 + lambda;
                let a22r = a22 + lambda;
                let det_r = a11r.mul_add(a22r, -(a12 * a12));
                if det_r.abs() < 1e-30 {
                    if a11 > a22 {
                        (b1 / a11.max(1e-30), 0.0)
                    } else if a22 > 1e-30 {
                        (0.0, b2 / a22.max(1e-30))
                    } else {
                        break;
                    }
                } else {
                    (
                        b1.mul_add(a22r, -(b2 * a12)) / det_r,
                        a11r.mul_add(b2, -(a12 * b1)) / det_r,
                    )
                }
            } else {
                (
                    b1.mul_add(a22, -(b2 * a12)) / det,
                    a11.mul_add(b2, -(a12 * b1)) / det,
                )
            };
            u -= du;
            v -= dv;
            u = u.clamp(u_min, u_max);
            v = v.clamp(v_min, v_max);
            let state = (u.to_bits(), v.to_bits());
            if period.is_none()
                && let Some(first) = seen.iter().position(|&s| s == state)
            {
                period = Some(seen.len() - first);
            }
            seen.push(state);
        }
        stats.exhausted += 1;
        match period {
            Some(1) => stats.fixed += 1,
            Some(_) => stats.cycled += 1,
            None => {}
        }

        let pt = surface.evaluate(u, v);
        let diff = pt - ray_origin;
        let diff_vec = Vec3::new(diff.x(), diff.y(), diff.z());
        let t = diff_vec.dot(ray_dir) / ray_dir.dot(ray_dir);
        let ray_pt = Point3::new(
            ray_dir.x().mul_add(t, ray_origin.x()),
            ray_dir.y().mul_add(t, ray_origin.y()),
            ray_dir.z().mul_add(t, ray_origin.z()),
        );
        ((pt - ray_pt).length() < 1e-5).then_some(IntersectionPoint {
            point: pt,
            param1: (u, v),
            param2: (t, 0.0),
        })
    }

    fn reference_intersect(
        surface: &NurbsSurface,
        ray_origin: Point3,
        ray_dir: Vec3,
        samples: usize,
        stats: &mut ReferenceStats,
    ) -> Vec<IntersectionPoint> {
        let n = samples.max(10);
        let (u_min, u_max) = surface.domain_u();
        let (v_min, v_max) = surface.domain_v();
        let u_step = (u_max - u_min) / (n - 1) as f64;
        let v_step = (v_max - v_min) / (n - 1) as f64;
        let dir_len_sq = ray_dir.dot(ray_dir);
        let grid: Vec<Vec<Point3>> = (0..n)
            .map(|i| {
                let u = u_min + i as f64 * u_step;
                (0..n)
                    .map(|j| surface.evaluate(u, v_min + j as f64 * v_step))
                    .collect()
            })
            .collect();
        let mut candidates = Vec::new();
        for i in 0..n {
            let u = u_min + i as f64 * u_step;
            for j in 0..n {
                let v = v_min + j as f64 * v_step;
                let pt = grid[i][j];
                let diff = pt - ray_origin;
                let diff_vec = Vec3::new(diff.x(), diff.y(), diff.z());
                let t = diff_vec.dot(ray_dir) / dir_len_sq;
                let closest_on_ray = Point3::new(
                    ray_origin.x().mul_add(1.0, ray_dir.x() * t),
                    ray_origin.y().mul_add(1.0, ray_dir.y() * t),
                    ray_origin.z().mul_add(1.0, ray_dir.z() * t),
                );
                let dist = (pt - closest_on_ray).length();
                let mut spacing = 0.0_f64;
                if i > 0 {
                    spacing = spacing.max((pt - grid[i - 1][j]).length());
                }
                if i + 1 < n {
                    spacing = spacing.max((pt - grid[i + 1][j]).length());
                }
                if j > 0 {
                    spacing = spacing.max((pt - grid[i][j - 1]).length());
                }
                if j + 1 < n {
                    spacing = spacing.max((pt - grid[i][j + 1]).length());
                }
                if dist < spacing {
                    candidates.push((u, v));
                }
            }
        }
        let mut results: Vec<IntersectionPoint> = Vec::new();
        for (u_guess, v_guess) in candidates {
            if let Some(pt) =
                reference_refine(surface, ray_origin, ray_dir, u_guess, v_guess, stats)
                && !results
                    .iter()
                    .any(|existing| (existing.point - pt.point).length() < 1e-6)
            {
                results.push(pt);
            }
        }
        results
    }

    fn bits(points: &[IntersectionPoint]) -> Vec<[u64; 7]> {
        points
            .iter()
            .map(|p| {
                [
                    p.point.x().to_bits(),
                    p.point.y().to_bits(),
                    p.point.z().to_bits(),
                    p.param1.0.to_bits(),
                    p.param1.1.to_bits(),
                    p.param2.0.to_bits(),
                    p.param2.1.to_bits(),
                ]
            })
            .collect()
    }

    /// A wavy bicubic patch over a non-unit knot domain with an interior knot.
    fn wavy_bicubic() -> NurbsSurface {
        let knots = vec![0.0, 0.0, 0.0, 0.0, 1.5, 3.0, 3.0, 3.0, 3.0];
        let control_points = (0..5)
            .map(|i| {
                (0..5)
                    .map(|j| {
                        let (x, y) = (f64::from(i), f64::from(j));
                        Point3::new(x, y, 0.6 * (x * 1.3).sin() * (y * 0.9).cos())
                    })
                    .collect()
            })
            .collect();
        NurbsSurface::new(
            3,
            3,
            knots.clone(),
            knots,
            control_points,
            vec![vec![1.0; 5]; 5],
        )
        .unwrap()
    }

    /// A rational quarter-cylinder sheet (radius 2, height 3).
    fn rational_quarter_cylinder() -> NurbsSurface {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let ring = [
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.0, 2.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
        ];
        let control_points = ring
            .iter()
            .map(|p| vec![*p, Point3::new(p.x(), p.y(), 3.0)])
            .collect();
        NurbsSurface::new(
            2,
            1,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            control_points,
            vec![vec![1.0, 1.0], vec![w, w], vec![1.0, 1.0]],
        )
        .unwrap()
    }

    /// A closed rational cylinder with repeated quarter knots and a non-unit
    /// axial domain. Its two line intersections have a Cartesian oracle.
    fn rational_cylinder() -> NurbsSurface {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let ring = [
            (2.0, 0.0, 1.0),
            (2.0, 2.0, w),
            (0.0, 2.0, 1.0),
            (-2.0, 2.0, w),
            (-2.0, 0.0, 1.0),
            (-2.0, -2.0, w),
            (0.0, -2.0, 1.0),
            (2.0, -2.0, w),
            (2.0, 0.0, 1.0),
        ];
        NurbsSurface::new(
            2,
            1,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 4.0],
            vec![2.0, 2.0, 5.0, 5.0],
            ring.iter()
                .map(|&(x, y, _)| vec![Point3::new(x, y, 0.0), Point3::new(x, y, 3.0)])
                .collect(),
            ring.iter().map(|&(_, _, weight)| vec![weight; 2]).collect(),
        )
        .unwrap()
    }

    fn high_degree_patch() -> NurbsSurface {
        let knots = [vec![-2.0; 10], vec![3.0; 10]].concat();
        let points = (0..10)
            .map(|i| {
                (0..10)
                    .map(|j| {
                        let (x, y) = (f64::from(i) / 9.0, f64::from(j) / 9.0);
                        Point3::new(x, y, 0.2 * x - 0.3 * y)
                    })
                    .collect()
            })
            .collect();
        NurbsSurface::new(9, 9, knots.clone(), knots, points, vec![vec![1.0; 10]; 10]).unwrap()
    }

    fn rational_pole_patch() -> NurbsSurface {
        let surface = rational_quarter_cylinder();
        let mut points = surface.control_points().to_vec();
        for row in &mut points {
            row[1] = Point3::new(0.0, 0.0, 3.0);
        }
        NurbsSurface::new(
            surface.degree_u(),
            surface.degree_v(),
            surface.knots_u().to_vec(),
            surface.knots_v().to_vec(),
            points,
            surface.weights().to_vec(),
        )
        .unwrap()
    }

    fn placed(surface: &NurbsSurface, scale: f64) -> NurbsSurface {
        let (s, c) = 0.37_f64.sin_cos();
        let mut points = surface.control_points().to_vec();
        for row in &mut points {
            for point in row {
                *point = Point3::new(
                    scale * (c * point.x() + s * point.z() + 37.0),
                    scale * (point.y() - 91.0),
                    scale * (-s * point.x() + c * point.z() + 13.0),
                );
            }
        }
        NurbsSurface::new(
            surface.degree_u(),
            surface.degree_v(),
            surface.knots_u().to_vec(),
            surface.knots_v().to_vec(),
            points,
            surface.weights().to_vec(),
        )
        .unwrap()
    }

    fn vector_bits(value: Vec3) -> [u64; 3] {
        [
            value.x().to_bits(),
            value.y().to_bits(),
            value.z().to_bits(),
        ]
    }

    #[test]
    fn evaluation_reuse_keeps_exact_uv_identity_and_collision_eviction() {
        let surface = wavy_bicubic();
        let mut workspace = NewtonWorkspace::default();
        let first = (0.8_f64, 1.3_f64);
        let expected = surface.evaluate(first.0, first.1);
        let expected_partials = surface.derivatives(first.0, first.1, 1);
        for _ in 0..2 {
            assert_eq!(workspace.point(&surface, first.0, first.1), expected);
            let (su, sv) = workspace.partials(&surface, first.0, first.1);
            assert_eq!(vector_bits(su), vector_bits(expected_partials[1][0]));
            assert_eq!(vector_bits(sv), vector_bits(expected_partials[0][1]));
        }
        // The position solve also kept the partials, so both partial reads
        // hit.
        assert_eq!(workspace.stats.point_evaluations, 1);
        assert_eq!(workspace.stats.partial_evaluations, 0);
        assert_eq!(workspace.stats.point_hits, 1);
        assert_eq!(workspace.stats.partial_hits, 2);

        // A distinct native UV forced into the same slot must run the actual
        // surface solves, including after returning to the evicted first UV.
        let second = (1..2000)
            .map(|i| (3.0 * f64::from(i) / 2000.0, first.1))
            .find(|&(u, v)| {
                u.to_bits() != first.0.to_bits()
                    && NewtonWorkspace::index(u, v) == NewtonWorkspace::index(first.0, first.1)
            })
            .unwrap();
        for (u, v) in [second, first] {
            let expected = surface.evaluate(u, v);
            let expected_partials = surface.derivatives(u, v, 1);
            let point = workspace.point(&surface, u, v);
            assert_eq!(point.x().to_bits(), expected.x().to_bits());
            assert_eq!(point.y().to_bits(), expected.y().to_bits());
            assert_eq!(point.z().to_bits(), expected.z().to_bits());
            let (su, sv) = workspace.partials(&surface, u, v);
            assert_eq!(vector_bits(su), vector_bits(expected_partials[1][0]));
            assert_eq!(vector_bits(sv), vector_bits(expected_partials[0][1]));
        }
        assert_eq!(workspace.stats.point_evaluations, 3);
        assert_eq!(workspace.stats.partial_evaluations, 0);

        // A seed point carries no partials: they take their own solve, then
        // hit.
        let seed = (0.35_f64, 0.95_f64);
        workspace.seed_point(seed.0, seed.1, surface.evaluate(seed.0, seed.1));
        let expected_partials = surface.derivatives(seed.0, seed.1, 1);
        for _ in 0..2 {
            let (su, sv) = workspace.partials(&surface, seed.0, seed.1);
            assert_eq!(vector_bits(su), vector_bits(expected_partials[1][0]));
            assert_eq!(vector_bits(sv), vector_bits(expected_partials[0][1]));
        }
        assert_eq!(workspace.stats.partial_evaluations, 1);
        assert_eq!(workspace.stats.point_evaluations, 3);
        let mut entry = SurfaceEvaluation::EMPTY;
        entry.occupied = true;
        assert!(entry.matches(0.0, 0.0));
        assert!(!entry.matches(-0.0, 0.0));
        assert!(!entry.matches(0.0_f64.next_up(), 0.0));
        assert!(std::mem::size_of_val(&workspace.evaluations) <= 8 * 1024);
    }

    #[test]
    fn repeated_refinement_reuses_values_without_reusing_newton_outcome() {
        let surface = wavy_bicubic();
        let line = LineQuery {
            origin: Point3::new(2.0, 2.0, 3.0),
            direction: Vec3::new(0.1, -0.2, -1.0),
            dir_len_sq: Vec3::new(0.1, -0.2, -1.0).dot(Vec3::new(0.1, -0.2, -1.0)),
        };
        let guess = (1.2, 1.8);
        let seed = surface.evaluate(guess.0, guess.1);
        let (u0, u1) = surface.domain_u();
        let (v0, v1) = surface.domain_v();
        let refine = |workspace: &mut NewtonWorkspace| {
            refine_line_surface_point(
                &surface,
                line,
                guess,
                seed,
                [u0, u1, v0, v1],
                GRID_CONVERGENCE,
                GRID_ROOT_TOLERANCE,
                workspace,
            )
        };
        let mut workspace = NewtonWorkspace::default();
        let first = refine(&mut workspace).unwrap();
        let evaluations = workspace.stats.point_evaluations + workspace.stats.partial_evaluations;
        // Within one refinement, partial reads hit the partials each position
        // solve kept; the second refinement also hits the first one's values.
        let first_hits = (workspace.stats.point_hits, workspace.stats.partial_hits);
        let second = refine(&mut workspace).unwrap();
        assert_eq!(bits(&[first]), bits(&[second]));
        assert!(workspace.stats.point_hits > first_hits.0);
        assert!(workspace.stats.partial_hits > first_hits.1);
        assert!(
            workspace.stats.point_evaluations + workspace.stats.partial_evaluations
                < 2 * evaluations
        );
        assert_eq!(workspace.stats.seed_points, 2);
        let expected = reference_refine(
            &surface,
            line.origin,
            line.direction,
            guess.0,
            guess.1,
            &mut ReferenceStats::default(),
        )
        .unwrap();
        assert_eq!(bits(&[second]), bits(&[expected]));

        // A new surface/ray call starts with no retained evaluations.
        let mut fresh = NewtonWorkspace::default();
        let actual = refine(&mut fresh).unwrap();
        assert_eq!(bits(&[actual]), bits(&[expected]));
        assert_eq!(
            (fresh.stats.point_hits, fresh.stats.partial_hits),
            first_hits
        );
        assert_eq!(
            fresh.stats.point_evaluations + fresh.stats.partial_evaluations,
            evaluations
        );
    }

    #[test]
    fn closed_cylinder_roots_match_cartesian_oracle() {
        let surface = rational_cylinder();
        let grid = LineSurfaceSeedGrid::for_surface(&surface, 20);
        for y in [0.3, 0.9, 1.7] {
            let hits = intersect_line_nurbs_with_grid(
                &surface,
                &grid,
                Point3::new(3.0, y, 1.3),
                Vec3::new(-1.0, 0.0, 0.0),
            )
            .unwrap();
            assert_eq!(hits.len(), 2);
            let x = (4.0 - y * y).sqrt();
            for expected_x in [-x, x] {
                assert!(hits.iter().any(|hit| {
                    (hit.point.x() - expected_x).abs() < 1e-8
                        && (hit.point.y() - y).abs() < 1e-8
                        && (hit.point.z() - 1.3).abs() < 1e-8
                        && (hit.param2.0 - (3.0 - expected_x)).abs() < 1e-8
                }));
            }
        }
        assert!(
            intersect_line_nurbs_with_grid(
                &surface,
                &grid,
                Point3::new(3.0, 2.01, 1.3),
                Vec3::new(-1.0, 0.0, 0.0),
            )
            .unwrap()
            .is_empty()
        );
    }

    #[test]
    fn reuse_preserves_root_bits_on_closed_pole_high_degree_and_placed_surfaces() {
        let mut stats = ReferenceStats::default();
        let mut hits = 0;
        for source in [
            rational_cylinder(),
            rational_pole_patch(),
            high_degree_patch(),
        ] {
            for scale in [0.001, 1.0, 1000.0] {
                let surface = placed(&source, scale);
                let grid = LineSurfaceSeedGrid::for_surface(&surface, 13);
                for i in 1..=40_u32 {
                    let origin = Point3::new(
                        scale * (37.0 + 6.0 * halton(i, 2) - 3.0),
                        scale * (-91.0 + 6.0 * halton(i, 3) - 3.0),
                        scale * (13.0 + 6.0 * halton(i, 5) - 2.0),
                    );
                    let z = 2.0 * halton(i, 7) - 1.0;
                    let phi = std::f64::consts::TAU * halton(i, 11);
                    let r = (1.0 - z * z).max(0.0).sqrt();
                    // Non-unit directions exercise reuse of the actual norm.
                    let dir = Vec3::new(r * phi.cos(), r * phi.sin(), z) * 2.3;
                    let expected = reference_intersect(&surface, origin, dir, 13, &mut stats);
                    let actual =
                        intersect_line_nurbs_with_grid(&surface, &grid, origin, dir).unwrap();
                    assert_eq!(bits(&actual), bits(&expected), "scale={scale}, ray={i}");
                    hits += actual.len();
                }
            }
        }
        assert!(hits > 30, "non-vacuous intersections: {hits}");
    }

    #[test]
    fn grid_and_newton_shortcuts_match_reference_bit_for_bit() {
        let mut stats = ReferenceStats::default();
        let mut hits = 0usize;
        for surface in [wavy_bicubic(), rational_quarter_cylinder()] {
            let grid = LineSurfaceSeedGrid::for_surface(&surface, 20);
            for i in 1..=600_u32 {
                // Origins scattered around the patch; directions over the
                // whole sphere, so rays pierce, graze and miss.
                let origin = Point3::new(
                    8.0f64.mul_add(halton(i, 2), -2.0),
                    8.0f64.mul_add(halton(i, 3), -2.0),
                    6.0f64.mul_add(halton(i, 5), -3.0),
                );
                let z = 2.0f64.mul_add(halton(i, 7), -1.0);
                let phi = std::f64::consts::TAU * halton(i, 11);
                let r = (1.0 - z * z).max(0.0).sqrt();
                let dir = Vec3::new(r * phi.cos(), r * phi.sin(), z);

                let expected = bits(&reference_intersect(&surface, origin, dir, 20, &mut stats));
                let one_shot = bits(&intersect_line_nurbs(&surface, origin, dir, 20).unwrap());
                let seeded =
                    bits(&intersect_line_nurbs_with_grid(&surface, &grid, origin, dir).unwrap());
                assert_eq!(one_shot, expected, "ray {i}");
                assert_eq!(seeded, expected, "ray {i}");
                hits += expected.len();
            }
        }
        // Non-vacuous: rays hit, and Newton runs exhausted their budget in
        // cycles (the case the skip shortcuts) as well as without one.
        assert!(hits > 100, "hits {hits}");
        assert!(stats.fixed > 0, "no fixed-point Newton run exercised");
        assert!(stats.cycled > 0, "no longer-period Newton cycle exercised");
        assert!(
            stats.exhausted > stats.fixed + stats.cycled,
            "no non-cycling exhausted run exercised"
        );
    }

    /// The cycle jump lands where running out the budget would: simulate
    /// every `(first, updates)` cycle shape step by step.
    /// A tolerance tighter than the in-loop convergence threshold is not
    /// bypassed by an early exit: a line 5e-11 above a plane patch is a
    /// root at the grid pass's 1e-5 but not at 1e-12.
    #[test]
    fn tight_final_tolerance_rejects_sub_threshold_near_miss() {
        let surface = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1e-3, 0.0)],
                vec![Point3::new(1e-3, 0.0, 0.0), Point3::new(1e-3, 1e-3, 0.0)],
            ],
            vec![vec![1.0; 2]; 2],
        )
        .unwrap();
        let direction = Vec3::new(1.0, 0.0, 0.0);
        let line = LineQuery {
            origin: Point3::new(0.0, 5e-4, 5e-11),
            direction,
            dir_len_sq: 1.0,
        };
        let guess = (0.5, 0.5);
        let seed = surface.evaluate(guess.0, guess.1);
        let refine = |tolerance: f64| {
            refine_line_surface_point(
                &surface,
                line,
                guess,
                seed,
                [0.0, 1.0, 0.0, 1.0],
                GRID_CONVERGENCE,
                tolerance,
                &mut NewtonWorkspace::default(),
            )
        };
        assert!(refine(GRID_ROOT_TOLERANCE).is_some());
        assert!(refine(1e-12).is_none());
    }

    #[test]
    fn budget_end_index_matches_running_out_the_budget() {
        for updates in 1..=MAX_NEWTON_ITER {
            for first in 0..updates {
                // States are labelled by the update count that first reached
                // them; the update after `updates - 1` returns to `first`.
                let next = |state: usize| {
                    if state + 1 < updates {
                        state + 1
                    } else {
                        first
                    }
                };
                let mut state = 0;
                for _ in 0..MAX_NEWTON_ITER {
                    state = next(state);
                }
                assert_eq!(
                    budget_end_index(first, updates),
                    state,
                    "first {first} updates {updates}"
                );
            }
        }
    }

    /// The seed scan as it stood before the prefilter, verbatim over the
    /// grid's nodes.
    fn seed_candidates_reference(
        grid: &LineSurfaceSeedGrid,
        ray_origin: Point3,
        ray_dir: Vec3,
    ) -> Vec<(f64, f64, Point3)> {
        let mut candidates: Vec<(f64, f64, Point3)> = Vec::new();
        let dir_len_sq = ray_dir.dot(ray_dir);
        let n = grid.n;
        for i in 0..n {
            let u = grid.u_min + i as f64 * grid.u_step;
            for j in 0..n {
                let v = grid.v_min + j as f64 * grid.v_step;
                let (pt, spacing) = grid.nodes[i * n + j];
                let diff = pt - ray_origin;
                let diff_vec = Vec3::new(diff.x(), diff.y(), diff.z());
                let t = diff_vec.dot(ray_dir) / dir_len_sq;
                let closest_on_ray = Point3::new(
                    ray_origin.x().mul_add(1.0, ray_dir.x() * t),
                    ray_origin.y().mul_add(1.0, ray_dir.y() * t),
                    ray_origin.z().mul_add(1.0, ray_dir.z() * t),
                );
                let dist = (pt - closest_on_ray).length();
                if dist < spacing {
                    candidates.push((u, v, pt));
                }
            }
        }
        candidates
    }

    fn candidate_bits(candidates: &[(f64, f64, Point3)]) -> Vec<[u64; 5]> {
        candidates
            .iter()
            .map(|&(u, v, p)| {
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

    /// The prefiltered scan keeps exactly the reference candidates, in
    /// order: random rays over the sphere of directions at unit, tiny,
    /// huge and over-the-guard direction norms, on curved, closed, pole,
    /// high-degree and far-placed (`1e5`, `1e95`, `1e120`) surfaces, and rays
    /// aimed to pass each sampled node at its spacing to within a few ulps.
    #[test]
    fn seed_prefilter_keeps_exactly_the_reference_candidates() {
        let mut filtered = 0usize;
        let mut scanned = 0usize;
        let mut kept = 0usize;
        let mut near_threshold = 0usize;
        let (mut aimed_kept, mut aimed_dropped) = (0usize, 0usize);
        let surfaces = [
            wavy_bicubic(),
            rational_cylinder(),
            rational_pole_patch(),
            high_degree_patch(),
            placed(&wavy_bicubic(), 1e5),
            placed(&rational_quarter_cylinder(), 1e-3),
            placed(&wavy_bicubic(), 1e95),
            placed(&wavy_bicubic(), 1e120),
        ];
        for (index, surface) in surfaces.iter().enumerate() {
            let grid = LineSurfaceSeedGrid::for_surface(surface, 20);
            let scale = grid.max_abs.max(1e-300);
            let center = grid.nodes[grid.nodes.len() / 2 + 3].0;
            let check = |origin: Point3, dir: Vec3, label: &str| {
                let dir_len_sq = dir.dot(dir);
                let actual = seed_candidates(&grid, origin, dir, dir_len_sq);
                let expected = seed_candidates_reference(&grid, origin, dir);
                assert_eq!(
                    candidate_bits(&actual),
                    candidate_bits(&expected),
                    "surface {index}, {label}: origin {origin:?}, dir {dir:?}"
                );
                let prefilter = SeedPrefilter::new(&grid, origin, dir, dir_len_sq);
                let rejected = grid
                    .nodes
                    .iter()
                    .filter(|(pt, spacing)| {
                        let d = *pt - origin;
                        prefilter.rejects(Vec3::new(d.x(), d.y(), d.z()), *spacing)
                    })
                    .count();
                (rejected, grid.nodes.len(), expected.len())
            };
            for i in 1..=200_u32 {
                let origin = Point3::new(
                    center.x() + scale * (2.0 * halton(i, 2) - 1.0) * 0.3,
                    center.y() + scale * (2.0 * halton(i, 3) - 1.0) * 0.3,
                    center.z() + scale * (2.0 * halton(i, 5) - 1.0) * 0.3,
                );
                let z = 2.0f64.mul_add(halton(i, 7), -1.0);
                let phi = std::f64::consts::TAU * halton(i, 11);
                let r = (1.0 - z * z).max(0.0).sqrt();
                let unit = Vec3::new(r * phi.cos(), r * phi.sin(), z);
                for norm in [1.0, 1e-9, 1e40, 1e60] {
                    let (rejected, total, found) = check(origin, unit * norm, "random");
                    filtered += rejected;
                    scanned += total;
                    kept += found;
                }
            }
            // Aim a ray past a node at its spacing, perturbed by a few ulps
            // and by tiny relative amounts either way, along random
            // directions perpendicular to the offset.
            for (k, &(pt, spacing)) in grid.nodes.iter().enumerate().step_by(7) {
                if spacing.is_nan() || spacing <= 0.0 {
                    continue;
                }
                let k = u32::try_from(k).unwrap() + 1;
                let a = Vec3::new(halton(k, 2) - 0.5, halton(k, 3) - 0.5, halton(k, 5) - 0.5);
                let b = Vec3::new(halton(k, 7) - 0.5, halton(k, 11) - 0.5, halton(k, 13) - 0.5);
                let Ok(offset) = a.normalize() else { continue };
                let Ok(dir) = (b - offset * b.dot(offset)).normalize() else {
                    continue;
                };
                for factor in [
                    1.0,
                    1.0f64.next_up(),
                    1.0f64.next_down(),
                    1.0 + 4.0 * f64::EPSILON,
                    1.0 - 4.0 * f64::EPSILON,
                    1.0 + 1e-12,
                    1.0 - 1e-12,
                    1.0 + 1e-9,
                    1.0 - 1e-9,
                ] {
                    let origin = pt + offset * (spacing * factor) - dir * (3.0 * spacing);
                    check(origin, dir, "near threshold");
                    check(origin, dir * 1e-9, "near threshold, tiny direction");
                    near_threshold += 1;
                    let aimed = seed_candidates_reference(&grid, origin, dir)
                        .iter()
                        .any(|&(_, _, p)| p == pt);
                    if aimed {
                        aimed_kept += 1;
                    } else {
                        aimed_dropped += 1;
                    }
                }
            }
        }
        // Non-vacuous: rays found candidates, near-threshold rays ran, and
        // the filter discarded most nodes.
        assert!(kept > 1_000, "kept {kept}");
        assert!(
            near_threshold > 1_000,
            "near-threshold rays {near_threshold}"
        );
        // The aimed node lands on both sides of its own threshold.
        assert!(
            aimed_kept > 100 && aimed_dropped > 100,
            "aimed nodes kept {aimed_kept}, dropped {aimed_dropped}"
        );
        assert!(filtered * 2 > scanned, "filtered {filtered} of {scanned}");
    }

    /// The filter switches off outside the range its error bound covers.
    #[test]
    fn seed_prefilter_is_off_beyond_its_error_bound() {
        let grid = LineSurfaceSeedGrid::for_surface(&wavy_bicubic(), 20);
        let origin = Point3::new(100.0, 100.0, 100.0);
        let dir = Vec3::new(0.0, 0.0, 1.0);
        let far = Vec3::new(1e6, 0.0, 0.0);
        assert!(SeedPrefilter::new(&grid, origin, dir, 1.0).rejects(far, 1.0));
        assert!(!SeedPrefilter::new(&grid, origin, dir, 1e101).rejects(far, 1.0));
        let huge = Point3::new(1e101, 0.0, 0.0);
        assert!(!SeedPrefilter::new(&grid, huge, dir, 1.0).rejects(far, 1.0));
        let nan = Point3::new(f64::NAN, 0.0, 0.0);
        assert!(!SeedPrefilter::new(&grid, nan, dir, 1.0).rejects(far, 1.0));
        let filter = SeedPrefilter::new(&grid, origin, dir, 1.0);
        assert!(!filter.rejects(Vec3::new(f64::NAN, 0.0, 0.0), 1.0));
        assert!(!filter.rejects(far, f64::NAN));
    }

    #[test]
    fn degenerate_direction_is_refused_by_both_entry_points() {
        let surface = wavy_bicubic();
        let grid = LineSurfaceSeedGrid::for_surface(&surface, 20);
        let origin = Point3::new(1.0, 1.0, 1.0);
        let zero = Vec3::new(0.0, 0.0, 0.0);
        assert!(intersect_line_nurbs(&surface, origin, zero, 20).is_err());
        assert!(intersect_line_nurbs_with_grid(&surface, &grid, origin, zero).is_err());
    }
}
