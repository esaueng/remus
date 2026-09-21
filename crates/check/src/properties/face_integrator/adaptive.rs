//! Bounded h-refinement on an already resolved rectangular analytic domain.

use super::{
    Accumulator, CheckError, DerivativeScratch, FaceContribution, ParametricSurface, PatchScale,
    PropertiesOptions, gauss_legendre_points, patch_count,
};

// Counts rule evaluations, independently of caller depth; each rule has at most
// MAX_ORDER² surface evaluations. No recursion can exceed the work budget.
const MAX_RULES: usize = 32_768;
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

struct Integrator<'a, S> {
    surface: &'a S,
    options: &'a PropertiesOptions,
    scratch: DerivativeScratch,
    rules: usize,
}

impl<S: ParametricSurface> Integrator<'_, S> {
    fn sample(&mut self, (u, v): Rect) -> Result<Sample, CheckError> {
        if self.rules >= MAX_RULES {
            return Err(CheckError::IntegrationFailed(
                "adaptive per-face work budget exhausted".into(),
            ));
        }
        self.rules += 1;
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
    u: (f64, f64),
    v: (f64, f64),
    sign: f64,
    scale: PatchScale,
    options: &PropertiesOptions,
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
        scratch: DerivativeScratch::new(),
        rules: 0,
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
