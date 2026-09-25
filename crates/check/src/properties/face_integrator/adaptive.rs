//! Bounded h-refinement on an already resolved quadrature domain.
//!
//! Two domain shapes are refined. A rectangle (an untrimmed analytic or NURBS
//! patch) is split into quadrants. A sliced domain — one axis cut at break
//! points, the other covered by spans that move smoothly between breaks, which
//! is what a polyline-trimmed face or a two-rim torus band is — has each outer
//! interval halved while the inner span tiling doubles. Either way the coarse
//! rule is compared with its refinement on every property component, and the
//! resolved domain itself (including any sampled trim) is fixed input: the
//! estimator measures quadrature convergence over it, not boundary error.

use super::{
    Accumulator, CheckError, DerivativeScratch, FaceContribution, ParametricSurface, PatchScale,
    Point3, PropertiesOptions, gauss_legendre_points, patch_count,
};

// Counts tensor-rule evaluations, independently of caller depth; each rule has
// `order²` surface evaluations. Sliced domains charge the same currency in
// surface evaluations. No recursion can exceed either budget.
const MAX_RULES: usize = 32_768;

// A sliced domain (level-zero pass and refinement together) may spend this
// many rectangular budgets. Its base pass alone walks every trim window and
// knot piece, which a rectangle never does; the knot-aligned hammer holder's
// worst face used ~4% of it at `adaptive_eps = 1e-9`.
const SLICED_BUDGET_FACTOR: usize = 64;
type Components = [f64; 14];
type Rect = ((f64, f64), (f64, f64));

struct Sample {
    value: Components,
    magnitude: Components,
}

impl Sample {
    fn zero() -> Self {
        Self {
            value: [0.0; 14],
            magnitude: [0.0; 14],
        }
    }

    fn add(&mut self, other: &Self) {
        for i in 0..14 {
            self.value[i] += other.value[i];
            self.magnitude[i] += other.magnitude[i];
        }
    }

    fn ensure_finite(&self) -> Result<(), CheckError> {
        if self
            .value
            .iter()
            .chain(&self.magnitude)
            .any(|x| !x.is_finite())
        {
            return Err(CheckError::IntegrationFailed(
                "non-finite adaptive integral".into(),
            ));
        }
        Ok(())
    }

    fn budgets(&self, eps: f64) -> Components {
        let mut budgets = [0.0; 14];
        // Compare quantities with like units: mm², mm³, mm⁴, mm⁵, mm³.
        for range in [0..1, 1..2, 2..5, 5..11, 11..14] {
            let scale = self.magnitude[range.clone()]
                .iter()
                .copied()
                .fold(0.0_f64, f64::max);
            for i in range {
                budgets[i] = eps * scale;
            }
        }
        budgets
    }
}

fn values(a: &Accumulator) -> Components {
    [
        a.area, a.vol, a.mx, a.my, a.mz, a.qxx, a.qyy, a.qzz, a.qxy, a.qxz, a.qyz, a.cx, a.cy, a.cz,
    ]
}

fn contribution(a: Components, sign: f64) -> FaceContribution {
    Accumulator {
        area: a[0],
        vol: a[1],
        mx: a[2],
        my: a[3],
        mz: a[4],
        qxx: a[5],
        qyy: a[6],
        qzz: a[7],
        qxy: a[8],
        qxz: a[9],
        qyz: a[10],
        cx: a[11],
        cy: a[12],
        cz: a[13],
    }
    .finish(sign)
}

fn quadrants((u, v): Rect) -> Result<[Rect; 4], CheckError> {
    let um = f64::midpoint(u.0, u.1);
    let vm = f64::midpoint(v.0, v.1);
    if !(u.0 < um && um < u.1 && v.0 < vm && vm < v.1) {
        return Err(CheckError::IntegrationFailed(
            "adaptive parameter interval cannot be subdivided".into(),
        ));
    }
    Ok([
        ((u.0, um), (v.0, vm)),
        ((um, u.1), (v.0, vm)),
        ((u.0, um), (vm, v.1)),
        ((um, u.1), (vm, v.1)),
    ])
}

/// Per-face surface-evaluation budget shared by both domain shapes.
struct Work {
    used: usize,
    limit: usize,
}

impl Work {
    const fn new(order: usize) -> Self {
        Self {
            used: 0,
            limit: MAX_RULES.saturating_mul(order.saturating_mul(order)),
        }
    }

    const fn sliced(order: usize) -> Self {
        let rectangular = Self::new(order);
        Self {
            used: 0,
            limit: rectangular.limit.saturating_mul(SLICED_BUDGET_FACTOR),
        }
    }

    fn charge(&mut self, evaluations: usize) -> Result<(), CheckError> {
        match self.used.checked_add(evaluations) {
            Some(used) if used <= self.limit => {
                self.used = used;
                Ok(())
            }
            _ => Err(CheckError::IntegrationFailed(
                "adaptive per-face work budget exhausted".into(),
            )),
        }
    }
}

struct Integrator<'a, S> {
    surface: &'a S,
    options: &'a PropertiesOptions,
    reference: Point3,
    scratch: DerivativeScratch,
    work: Work,
}

impl<S: ParametricSurface> Integrator<'_, S> {
    fn sample(&mut self, (u, v): Rect) -> Result<Sample, CheckError> {
        let order = self.options.gauss_order;
        self.work.charge(order * order)?;
        let uh = (u.1 - u.0) * 0.5;
        let vh = (v.1 - v.0) * 0.5;
        let mut result = Sample::zero();
        for gu in gauss_legendre_points(self.options.gauss_order) {
            for gv in gauss_legendre_points(self.options.gauss_order) {
                let mut a = Accumulator::default();
                a.add(
                    self.surface,
                    uh.mul_add(gu.x, f64::midpoint(u.0, u.1)),
                    vh.mul_add(gv.x, f64::midpoint(v.0, v.1)),
                    gu.w * gv.w * uh * vh,
                    self.reference,
                    &mut self.scratch,
                );
                for (i, x) in values(&a).into_iter().enumerate() {
                    result.value[i] += x;
                    result.magnitude[i] += x.abs();
                }
            }
        }
        result.ensure_finite()?;
        Ok(result)
    }

    fn refine(
        &mut self,
        rect: Rect,
        coarse: &Sample,
        depth: usize,
        budget: Option<Components>,
    ) -> Result<Sample, CheckError> {
        let rects = quadrants(rect)?;
        let mut children = Vec::with_capacity(4);
        let mut fine = Sample::zero();
        for r in rects {
            let sample = self.sample(r)?;
            fine.add(&sample);
            children.push(sample);
        }
        fine.ensure_finite()?;
        let budget = budget.unwrap_or_else(|| fine.budgets(self.options.adaptive_eps));
        if fine
            .value
            .iter()
            .zip(coarse.value)
            .zip(budget)
            .all(|((&f, c), b)| (f - c).abs() <= b)
        {
            return Ok(fine);
        }
        if depth >= self.options.max_depth.min(32) {
            return Err(CheckError::IntegrationFailed(
                "adaptive max_depth (or safety depth 32) exhausted before convergence".into(),
            ));
        }
        let child_budget = budget.map(|b| b * 0.25);
        let mut refined = Sample::zero();
        for (r, sample) in rects.into_iter().zip(children) {
            refined.add(&self.refine(r, &sample, depth + 1, Some(child_budget))?);
        }
        refined.ensure_finite()?;
        Ok(refined)
    }
}

#[allow(clippy::cast_precision_loss)]
pub(super) fn integrate<S: ParametricSurface>(
    surface: &S,
    (u, v): Rect,
    sign: f64,
    scale: PatchScale,
    options: &PropertiesOptions,
    reference: Point3,
) -> Result<FaceContribution, CheckError> {
    if ![u.0, u.1, v.0, v.1].iter().all(|x| x.is_finite()) || u.0 >= u.1 || v.0 >= v.1 {
        return Err(CheckError::IntegrationFailed(
            "invalid adaptive parameter domain".into(),
        ));
    }
    let nu = patch_count(u.1 - u.0, scale.u);
    let nv = patch_count(v.1 - v.0, scale.v);
    let mut integrator = Integrator {
        surface,
        options,
        reference,
        scratch: DerivativeScratch::new(),
        work: Work::new(options.gauss_order),
    };
    let mut total = Sample::zero();
    for iu in 0..nu {
        for iv in 0..nv {
            let rect = (
                (
                    u.0 + (u.1 - u.0) * iu as f64 / nu as f64,
                    u.0 + (u.1 - u.0) * (iu + 1) as f64 / nu as f64,
                ),
                (
                    v.0 + (v.1 - v.0) * iv as f64 / nv as f64,
                    v.0 + (v.1 - v.0) * (iv + 1) as f64 / nv as f64,
                ),
            );
            let coarse = integrator.sample(rect)?;
            total.add(&integrator.refine(rect, &coarse, 0, None)?);
        }
    }
    total.ensure_finite()?;
    Ok(contribution(total.value, sign))
}

/// A domain cut into outer-axis intervals whose inner-axis spans vary smoothly
/// inside each interval.
pub(super) struct Sliced<'a> {
    /// Sorted outer-axis break points; each adjacent pair is one interval.
    pub breaks: &'a [f64],
    /// Sorted inner-axis values every span is split at (a NURBS carrier's
    /// interior knots), so no inner Gauss rule straddles a polynomial join.
    pub inner_knots: &'a [f64],
    /// Initial patch length on the outer axis (`f64::INFINITY` for none).
    pub outer_scale: f64,
    /// Initial patch length on the inner axis.
    pub inner_scale: f64,
    /// Inner spans that belong to the face at an outer coordinate. The span
    /// ends may be given in either order.
    pub spans: &'a dyn Fn(f64) -> Vec<(f64, f64)>,
    /// `false`: outer is `u`, inner is `v`. `true`: outer is `v`, inner is `u`.
    pub outer_is_v: bool,
}

/// How the inner spans of one break window are produced.
///
/// Between two consecutive breaks no outline vertex intervenes, so every span
/// end moves affinely with the outer coordinate (a polyline edge crossing a
/// line of constant outer coordinate). Refinement evaluates spans at many
/// abscissae inside one window, and scanning the whole outline each time
/// dominated the cost on sampled NURBS trims; the affine model replaces that
/// scan with two evaluations, checked against a third before it is trusted.
enum SpanModel {
    Affine {
        origin: f64,
        base: Vec<(f64, f64)>,
        slope: Vec<(f64, f64)>,
    },
    Direct,
}

impl SpanModel {
    fn fit(spans: &dyn Fn(f64) -> Vec<(f64, f64)>, (a, b): (f64, f64)) -> Self {
        let width = b - a;
        let (u1, um, u2) = (
            width.mul_add(0.25, a),
            f64::midpoint(a, b),
            width.mul_add(0.75, a),
        );
        if !(a < u1 && u1 < um && um < u2 && u2 < b) {
            return Self::Direct;
        }
        let (s1, sm, s2) = (spans(u1), spans(um), spans(u2));
        if s1.len() != s2.len() || s1.len() != sm.len() {
            return Self::Direct;
        }
        let run = u2 - u1;
        let slope: Vec<_> = s1
            .iter()
            .zip(&s2)
            .map(|(p, q)| ((q.0 - p.0) / run, (q.1 - p.1) / run))
            .collect();
        let scale = s1
            .iter()
            .chain(&sm)
            .chain(&s2)
            .flat_map(|&(lo, hi)| [lo.abs(), hi.abs()])
            .fold(1.0_f64, f64::max);
        let tolerance = 1e-11 * scale;
        let reproduces = s1.iter().zip(&slope).zip(&sm).all(|((p, d), m)| {
            let lo = d.0.mul_add(um - u1, p.0);
            let hi = d.1.mul_add(um - u1, p.1);
            (lo - m.0).abs() <= tolerance && (hi - m.1).abs() <= tolerance
        });
        if !reproduces || slope.iter().any(|d| !(d.0.is_finite() && d.1.is_finite())) {
            return Self::Direct;
        }
        Self::Affine {
            origin: u1,
            base: s1,
            slope,
        }
    }
}

struct SlicedIntegrator<'a> {
    surface: &'a dyn ParametricSurface,
    reference: Point3,
    domain: &'a Sliced<'a>,
    options: &'a PropertiesOptions,
    scratch: DerivativeScratch,
    work: Work,
    models: Vec<SpanModel>,
}

impl SlicedIntegrator<'_> {
    /// One outer Gauss rule on `(a, b)`; each inner span is tiled as at level
    /// zero and then split `2^level` times.
    #[allow(clippy::cast_precision_loss)]
    fn sample(
        &mut self,
        (a, b): (f64, f64),
        level: usize,
        window: usize,
    ) -> Result<Sample, CheckError> {
        let gauss = gauss_legendre_points(self.options.gauss_order);
        let half = (b - a) * 0.5;
        let mid = f64::midpoint(a, b);
        let split = u32::try_from(level)
            .ok()
            .and_then(|level| 1usize.checked_shl(level))
            .ok_or_else(|| {
                CheckError::IntegrationFailed("adaptive per-face work budget exhausted".into())
            })?;
        let mut columns = Vec::with_capacity(gauss.len());
        let mut evaluations = 0usize;
        for g in gauss {
            let outer = half.mul_add(g.x, mid);
            let mut tiles = Vec::new();
            let spans = match self.models.get(window) {
                Some(SpanModel::Affine {
                    origin,
                    base,
                    slope,
                }) => base
                    .iter()
                    .zip(slope)
                    .map(|(p, d)| {
                        (
                            d.0.mul_add(outer - origin, p.0),
                            d.1.mul_add(outer - origin, p.1),
                        )
                    })
                    .collect(),
                _ => (self.domain.spans)(outer),
            };
            for (lo, hi) in spans {
                let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
                if !(lo.is_finite() && hi.is_finite()) {
                    return Err(CheckError::IntegrationFailed(
                        "non-finite adaptive integral".into(),
                    ));
                }
                let first = self.domain.inner_knots.partition_point(|&k| k <= lo);
                let last = self.domain.inner_knots.partition_point(|&k| k < hi);
                let cuts = self.domain.inner_knots.get(first..last).unwrap_or(&[]);
                let mut start = lo;
                for &end in cuts.iter().chain(std::iter::once(&hi)) {
                    let count =
                        patch_count(end - start, self.domain.inner_scale).saturating_mul(split);
                    evaluations = evaluations.saturating_add(count.saturating_mul(gauss.len()));
                    tiles.push((start, end, count));
                    start = end;
                }
            }
            columns.push((g, outer, tiles));
        }
        self.work.charge(evaluations)?;
        let mut result = Sample::zero();
        for (g, outer, tiles) in columns {
            for (lo, hi, count) in tiles {
                let step = (hi - lo) / count as f64;
                for k in 0..count {
                    let tile_mid = step.mul_add(k as f64 + 0.5, lo);
                    for h in gauss {
                        let inner = (step * 0.5).mul_add(h.x, tile_mid);
                        let (u, v) = if self.domain.outer_is_v {
                            (inner, outer)
                        } else {
                            (outer, inner)
                        };
                        let mut acc = Accumulator::default();
                        acc.add(
                            self.surface,
                            u,
                            v,
                            g.w * h.w * half.abs() * step * 0.5,
                            self.reference,
                            &mut self.scratch,
                        );
                        for (i, x) in values(&acc).into_iter().enumerate() {
                            result.value[i] += x;
                            result.magnitude[i] += x.abs();
                        }
                    }
                }
            }
        }
        result.ensure_finite()?;
        Ok(result)
    }

    /// Refine `interval` once against its `coarse` rule.
    ///
    /// `depth` is also the interval's level: each subdivision halves the outer
    /// interval and doubles the inner tiling once.
    fn cell(
        &mut self,
        (a, b): (f64, f64),
        coarse: &Sample,
        depth: usize,
        window: usize,
    ) -> Result<Cell, CheckError> {
        let m = f64::midpoint(a, b);
        let halves = [
            self.sample((a, m), depth + 1, window)?,
            self.sample((m, b), depth + 1, window)?,
        ];
        let mut error = [0.0; 14];
        for (i, e) in error.iter_mut().enumerate() {
            *e = (halves[0].value[i] + halves[1].value[i] - coarse.value[i]).abs();
        }
        Ok(Cell {
            interval: (a, b),
            depth,
            window,
            halves,
            error,
        })
    }
}

/// One outer interval of a sliced domain, with its coarse rule and the
/// refinement that estimates the coarse rule's error.
struct Cell {
    interval: (f64, f64),
    depth: usize,
    window: usize,
    halves: [Sample; 2],
    error: Components,
}

impl Cell {
    fn fine(&self) -> Components {
        let mut value = self.halves[0].value;
        for (v, h) in value.iter_mut().zip(self.halves[1].value) {
            *v += h;
        }
        value
    }
}

/// Largest ratio of a cell's error to the face budget over the components.
fn priority(error: &Components, budget: &Components) -> f64 {
    error
        .iter()
        .zip(budget)
        .map(|(&e, &b)| {
            if e <= b * f64::EPSILON {
                0.0
            } else if b > 0.0 {
                e / b
            } else {
                f64::INFINITY
            }
        })
        .fold(0.0, f64::max)
}

/// Adaptive counterpart of the fixed sliced quadrature.
///
/// Level zero tiles each break interval by `outer_scale` and each inner span
/// (between inner knots) by `inner_scale` — the fixed rule's tiling on an
/// analytic face, one Gauss rule per knot span on a NURBS carrier. A refinement of an
/// interval halves it and doubles its inner tiling together, so every step
/// shrinks both cell dimensions as the rectangular quadrant split does.
///
/// The error budget is global over the face: `adaptive_eps` times the face's
/// own magnitude per dimensional group, against the SUM of the per-interval
/// coarse-versus-refined differences. The interval with the largest share is
/// refined first. A per-interval relative test would instead demand relative
/// accuracy from sub-ulp slivers that polyline trim vertices create next to a
/// seam, where the integrand jumps by a steep outline segment yet the sliver
/// contributes ~1e-15 of the face.
///
/// The surface is taken as a trait object on purpose: this path runs only on
/// non-default controls, and one out-of-line copy instead of one per carrier
/// type keeps ~55 KB out of each WASM module, at the cost of a virtual call
/// that is negligible next to a surface evaluation.
#[inline(never)]
#[allow(clippy::cast_precision_loss)]
pub(super) fn integrate_sliced(
    surface: &dyn ParametricSurface,
    domain: &Sliced<'_>,
    sign: f64,
    options: &PropertiesOptions,
    reference: Point3,
) -> Result<FaceContribution, CheckError> {
    if domain.breaks.iter().any(|x| !x.is_finite()) {
        return Err(CheckError::IntegrationFailed(
            "invalid adaptive parameter domain".into(),
        ));
    }
    let mut integrator = SlicedIntegrator {
        surface,
        reference,
        domain,
        options,
        scratch: DerivativeScratch::new(),
        work: Work::sliced(options.gauss_order),
        models: Vec::new(),
    };
    let lo = domain.breaks.first().copied().unwrap_or(0.0);
    let hi = domain.breaks.last().copied().unwrap_or(0.0);
    let eps = (hi - lo).abs() * 1e-12;
    let mut coarse = Vec::new();
    for window in domain.breaks.windows(2) {
        let (a, b) = (window[0], window[1]);
        if b - a <= eps {
            continue;
        }
        let window = integrator.models.len();
        integrator.models.push(SpanModel::fit(domain.spans, (a, b)));
        let n = patch_count(b - a, domain.outer_scale);
        for i in 0..n {
            let interval = (
                (b - a).mul_add(i as f64 / n as f64, a),
                (b - a).mul_add((i + 1) as f64 / n as f64, a),
            );
            coarse.push((interval, window, integrator.sample(interval, 0, window)?));
        }
    }

    let mut cells = Vec::with_capacity(coarse.len());
    let mut scale = Sample::zero();
    for (interval, window, sample) in coarse {
        let cell = integrator.cell(interval, &sample, 0, window)?;
        for half in &cell.halves {
            scale.add(half);
        }
        cells.push(Some(cell));
    }
    scale.ensure_finite()?;
    let budget = scale.budgets(options.adaptive_eps);

    let mut total_error = [0.0; 14];
    let mut queue = std::collections::BinaryHeap::new();
    for (index, cell) in cells.iter().enumerate() {
        if let Some(cell) = cell {
            for (t, e) in total_error.iter_mut().zip(cell.error) {
                *t += e;
            }
            queue.push(Ranked(priority(&cell.error, &budget), index));
        }
    }
    let max_depth = options.max_depth.min(32);
    loop {
        if total_error.iter().zip(budget).all(|(&e, b)| e <= b) {
            // Re-sum exactly: the running total is only a steering estimate.
            let mut exact = [0.0; 14];
            for cell in cells.iter().flatten() {
                for (t, e) in exact.iter_mut().zip(cell.error) {
                    *t += e;
                }
            }
            total_error = exact;
            if total_error.iter().zip(budget).all(|(&e, b)| e <= b) {
                break;
            }
        }
        let Some(Ranked(_, index)) = queue.pop() else {
            break;
        };
        let Some(cell) = cells.get_mut(index).and_then(Option::take) else {
            continue;
        };
        if cell.depth >= max_depth {
            return Err(CheckError::IntegrationFailed(
                "adaptive max_depth (or safety depth 32) exhausted before convergence".into(),
            ));
        }
        let (a, b) = cell.interval;
        let m = f64::midpoint(a, b);
        if !(a < m && m < b) {
            return Err(CheckError::IntegrationFailed(
                "adaptive parameter interval cannot be subdivided".into(),
            ));
        }
        for (t, e) in total_error.iter_mut().zip(cell.error) {
            *t -= e;
        }
        for (interval, sample) in [(a, m), (m, b)].into_iter().zip(cell.halves) {
            let child = integrator.cell(interval, &sample, cell.depth + 1, cell.window)?;
            for (t, e) in total_error.iter_mut().zip(child.error) {
                *t += e;
            }
            queue.push(Ranked(priority(&child.error, &budget), cells.len()));
            cells.push(Some(child));
        }
    }
    if total_error.iter().zip(budget).any(|(&e, b)| e > b) {
        return Err(CheckError::IntegrationFailed(
            "adaptive estimator did not converge".into(),
        ));
    }
    let mut total = [0.0; 14];
    for cell in cells.iter().flatten() {
        for (t, f) in total.iter_mut().zip(cell.fine()) {
            *t += f;
        }
    }
    let total = Sample {
        value: total,
        magnitude: scale.magnitude,
    };
    total.ensure_finite()?;
    Ok(contribution(total.value, sign))
}

/// Heap entry ordering intervals by their error share.
struct Ranked(f64, usize);

impl PartialEq for Ranked {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == std::cmp::Ordering::Equal
    }
}

impl Eq for Ranked {}

impl PartialOrd for Ranked {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Ranked {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Ties break toward the older interval so the order is deterministic.
        self.0
            .total_cmp(&other.0)
            .then_with(|| other.1.cmp(&self.1))
    }
}
