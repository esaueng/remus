//! NURBS surface evaluation via tensor-product De Boor.

use crate::MathError;
use crate::aabb::Aabb3;
use crate::nurbs::basis;
use crate::nurbs::evaluator::SurfaceEvaluator;
use crate::vec::{Point3, Vec3};

/// A Non-Uniform Rational B-Spline (NURBS) surface in 3D space.
///
/// The surface is defined by degrees in the u and v directions, two knot
/// vectors, a 2D grid of control points, and matching weights.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NurbsSurface {
    /// Polynomial degree in the u direction.
    degree_u: usize,
    /// Polynomial degree in the v direction.
    degree_v: usize,
    /// Knot vector in the u direction.
    knots_u: Vec<f64>,
    /// Knot vector in the v direction.
    knots_v: Vec<f64>,
    /// Control point grid indexed as `control_points[row_u][col_v]`.
    control_points: Vec<Vec<Point3>>,
    /// Weight grid matching `control_points` dimensions.
    weights: Vec<Vec<f64>>,
    /// Largest weight, cached: `derivatives` divides every weight by it so a
    /// common factor (e.g. 1e-300) cannot destabilize the perspective divide.
    /// The struct is immutable after construction, so the cache never goes
    /// stale; it is filled in `new` and lazily after deserialization.
    #[cfg_attr(feature = "serde", serde(skip))]
    max_weight: std::sync::OnceLock<f64>,
}

impl PartialEq for NurbsSurface {
    fn eq(&self, other: &Self) -> bool {
        self.degree_u == other.degree_u
            && self.degree_v == other.degree_v
            && self.knots_u == other.knots_u
            && self.knots_v == other.knots_v
            && self.control_points == other.control_points
            && self.weights == other.weights
    }
}

/// Reusable scratch storage for [`NurbsSurface::derivatives_into`].
///
/// Holds the basis-derivative buffers and the homogeneous quotient table so a
/// hot loop (e.g. one Gauss abscissa after another) pays their allocation at
/// most once, when the buffer first grows to the requested size. All state is
/// overwritten on every call; nothing is read before it is written.
///
/// Re-exported for quadrature callers in other crates; see
/// [`NurbsSurface::derivatives_into`].
#[derive(Debug, Default)]
pub struct DerivativeScratch {
    basis: Vec<f64>,
    sk: Vec<Vec3>,
    point_and_partials_out: Vec<Vec<Vec3>>,
    /// Last knot spans the derivative solve ran in, if any.
    ///
    /// [`Self::span_hinted_point_and_partials_from`] consults the hinted
    /// spans first and falls back to the binary search on a miss, so
    /// quadrature callers must not read these fields directly — only through
    /// that method, which verifies before trusting.
    last_span_u: Option<usize>,
    last_span_v: Option<usize>,
}

impl DerivativeScratch {
    /// Empty scratch; buffers grow on first use.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    fn ensure_basis(&mut self, len: usize) {
        if self.basis.len() < len {
            self.basis.resize(len, 0.0);
        }
    }

    fn ensure_sk(&mut self, len: usize) {
        if self.sk.len() < len {
            self.sk.resize(len, Vec3::new(0.0, 0.0, 0.0));
        }
    }

    /// Size the basis buffer for a `derivatives_into` call at order `d`.
    fn ensure_basis_for(&mut self, surface: &NurbsSurface, d: usize) {
        let du = d.min(surface.degree_u);
        let dv = d.min(surface.degree_v);
        self.ensure_basis((du + 1) * (surface.degree_u + 1) + (dv + 1) * (surface.degree_v + 1));
    }

    /// Size the quotient table for a `derivatives_into` call at order `d`.
    fn ensure_sk_for(&mut self, d: usize) {
        self.ensure_sk((d + 1) * (d + 1));
    }

    /// Grow the reusable 2x2 `derivatives_into` output table.
    #[doc(hidden)]
    pub fn ensure_point_and_partials_out(&mut self) {
        if self.point_and_partials_out.len() < 2 || self.point_and_partials_out[0].len() < 2 {
            self.point_and_partials_out = vec![vec![Vec3::new(0.0, 0.0, 0.0); 2]; 2];
        }
    }

    /// Run one fused position-and-partials solve into the reusable buffers.
    ///
    /// Equivalent to `point_and_partials` on this surface, without any
    /// per-call allocation once the buffers have grown. Callers must not
    /// share one scratch across threads.
    #[doc(hidden)]
    pub fn point_and_partials_from(
        &mut self,
        surface: &NurbsSurface,
        u: f64,
        v: f64,
    ) -> (Vec3, Vec3, Vec3) {
        let (p, du, dv, _, _) = self.span_hinted_point_and_partials_from(surface, u, v);
        (p, du, dv)
    }

    /// [`Self::point_and_partials_from`], reusing the previous call's knot
    /// spans when the parameters still lie in them.
    ///
    /// Gauss abscissae arrive in ascending `u` (and usually ascending `v`
    /// within one patch), so the solve usually stays in the same span; a
    /// verified hint then replaces each axis's `find_span` binary search.
    /// Verification is `knots[span] <= t < knots[span + 1]` — the exact
    /// postcondition [`basis::find_span`] establishes, including at repeated
    /// knots and the clamped domain ends — so a hit runs bit-identically to
    /// the search (the same indices feed the same basis solve), and a miss
    /// takes the search, which also refreshes the hint. The returned `bool`s
    /// report whether each axis hit, for census only; results do not depend
    /// on them. Callers must use one scratch per surface: spans are knot
    /// indices, meaningless on any other knot vector. Callers must not share
    /// one scratch across threads.
    #[doc(hidden)]
    pub fn span_hinted_point_and_partials_from(
        &mut self,
        surface: &NurbsSurface,
        u: f64,
        v: f64,
    ) -> (Vec3, Vec3, Vec3, bool, bool) {
        self.ensure_point_and_partials_out();
        self.ensure_basis_for(surface, 1);
        self.ensure_sk_for(1);
        // Split the borrows: the output table, the scratch buffers, and the
        // span hints live in disjoint fields, so all can be borrowed at once.
        let Self {
            basis,
            sk,
            point_and_partials_out: out,
            last_span_u,
            last_span_v,
        } = self;
        let (span_u, hit_u) = surface.find_span_hinted_u(u, last_span_u.unwrap_or(usize::MAX));
        let (span_v, hit_v) = surface.find_span_hinted_v(v, last_span_v.unwrap_or(usize::MAX));
        *last_span_u = Some(span_u);
        *last_span_v = Some(span_v);
        surface.derivatives_into_with_spans(u, v, 1, span_u, span_v, basis, sk, out);
        (out[0][0], out[1][0], out[0][1], hit_u, hit_v)
    }

    /// The reusable 2x2 `derivatives_into` output table.
    ///
    /// Valid only after [`Self::ensure_point_and_partials_out`]; callers must
    /// not retain the borrow across calls.
    #[doc(hidden)]
    #[must_use]
    pub fn point_and_partials_out_mut(&mut self) -> &mut [Vec<Vec3>] {
        &mut self.point_and_partials_out
    }
}

impl NurbsSurface {
    /// The largest control-point weight (cached; computed once per surface).
    #[must_use]
    pub fn max_weight(&self) -> f64 {
        *self.max_weight.get_or_init(|| {
            self.weights
                .iter()
                .flatten()
                .copied()
                .fold(0.0_f64, f64::max)
        })
    }

    /// Construct a new NURBS surface with validation.
    ///
    /// # Errors
    ///
    /// Returns [`MathError::InvalidControlPointGrid`] if the control point
    /// rows have inconsistent lengths.
    ///
    /// Returns [`MathError::InvalidKnotVector`] if either knot vector has the
    /// wrong length for the given degree and control point count, or
    /// [`MathError::InvalidKnotValue`] if a knot is non-finite or a vector is
    /// not non-decreasing.
    ///
    /// Returns [`MathError::InvalidWeights`] if the weights grid dimensions
    /// do not match the control point grid, or
    /// [`MathError::InvalidWeightValue`] if a weight is non-finite or
    /// non-positive.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        degree_u: usize,
        degree_v: usize,
        knots_u: Vec<f64>,
        knots_v: Vec<f64>,
        control_points: Vec<Vec<Point3>>,
        weights: Vec<Vec<f64>>,
    ) -> Result<Self, MathError> {
        let n_rows = control_points.len();

        // Validate that all rows have the same length.
        let n_cols = control_points.first().map_or(0, Vec::len);
        for row in &control_points {
            if row.len() != n_cols {
                return Err(MathError::InvalidControlPointGrid {
                    expected_rows: n_rows,
                    expected_cols: n_cols,
                });
            }
        }

        // Same degree contract as `NurbsCurve::new`, applied per direction.
        // Degree 0 yields discontinuous basis functions whose derivatives —
        // and therefore surface normals — are identically zero, so it cannot
        // produce usable face geometry.
        if degree_u == 0 || degree_u >= n_rows {
            return Err(MathError::InvalidDegree {
                degree: degree_u,
                control_points: n_rows,
            });
        }
        if degree_v == 0 || degree_v >= n_cols {
            return Err(MathError::InvalidDegree {
                degree: degree_v,
                control_points: n_cols,
            });
        }

        // Validate knot vectors.
        let expected_knots_u = n_rows + degree_u + 1;
        if knots_u.len() != expected_knots_u {
            return Err(MathError::InvalidKnotVector {
                expected: expected_knots_u,
                got: knots_u.len(),
            });
        }
        super::validate_knot_values(&knots_u)?;
        super::validate_knot_domain(&knots_u, degree_u, n_rows)?;

        let expected_knots_v = n_cols + degree_v + 1;
        if knots_v.len() != expected_knots_v {
            return Err(MathError::InvalidKnotVector {
                expected: expected_knots_v,
                got: knots_v.len(),
            });
        }
        super::validate_knot_values(&knots_v)?;
        super::validate_knot_domain(&knots_v, degree_v, n_cols)?;

        // Validate weights grid dimensions.
        if weights.len() != n_rows {
            return Err(MathError::InvalidWeights {
                expected: n_rows,
                got: weights.len(),
            });
        }
        for row in &weights {
            if row.len() != n_cols {
                return Err(MathError::InvalidWeights {
                    expected: n_cols,
                    got: row.len(),
                });
            }
        }
        validate_weight_values(&weights)?;
        super::validate_control_point_values(control_points.iter().flatten().copied())?;

        let surface = Self {
            degree_u,
            degree_v,
            knots_u,
            knots_v,
            control_points,
            weights,
            max_weight: std::sync::OnceLock::new(),
        };
        let _ = surface.max_weight();
        Ok(surface)
    }

    /// Polynomial degree in the u direction.
    #[must_use]
    pub const fn degree_u(&self) -> usize {
        self.degree_u
    }

    /// Polynomial degree in the v direction.
    #[must_use]
    pub const fn degree_v(&self) -> usize {
        self.degree_v
    }

    /// Whether the surface is rational (any weight differs from 1.0).
    #[must_use]
    pub fn is_rational(&self) -> bool {
        self.weights
            .iter()
            .flatten()
            .any(|&weight| weight.to_bits() != 1.0f64.to_bits())
    }

    /// Return the valid parameter domain in u: `[u_min, u_max]`.
    #[must_use]
    pub fn domain_u(&self) -> (f64, f64) {
        let u_min = self.knots_u[self.degree_u];
        let u_max = self.knots_u[self.knots_u.len() - self.degree_u - 1];
        (u_min, u_max)
    }

    /// Return the valid parameter domain in v: `[v_min, v_max]`.
    #[must_use]
    pub fn domain_v(&self) -> (f64, f64) {
        let v_min = self.knots_v[self.degree_v];
        let v_max = self.knots_v[self.knots_v.len() - self.degree_v - 1];
        (v_min, v_max)
    }

    /// Whether the surface is periodic (closed) in u.
    ///
    /// A NURBS surface is considered periodic in u if the first and last
    /// control point rows coincide within a tight tolerance. This is true
    /// for surfaces converted from analytic periodic types (cylinder, cone,
    /// sphere, torus).
    #[must_use]
    pub fn is_periodic_u(&self) -> bool {
        let n = self.control_points.len();
        if n < 2 {
            return false;
        }
        let first = &self.control_points[0];
        let last = &self.control_points[n - 1];
        if first.len() != last.len() {
            return false;
        }
        // (1e-7)^2 matching Tolerance::default().linear
        first.iter().zip(last.iter()).all(|(a, b)| {
            let d = *a - *b;
            d.x() * d.x() + d.y() * d.y() + d.z() * d.z() < 1e-14
        })
    }

    /// Whether the surface is periodic (closed) in v.
    ///
    /// A NURBS surface is considered periodic in v if the first and last
    /// control point columns coincide within a tight tolerance.
    #[must_use]
    pub fn is_periodic_v(&self) -> bool {
        if self.control_points.is_empty() {
            return false;
        }
        // (1e-7)^2 matching Tolerance::default().linear
        self.control_points.iter().all(|row| {
            if row.len() < 2 {
                return false;
            }
            let d = row[0] - row[row.len() - 1];
            d.x() * d.x() + d.y() * d.y() + d.z() * d.z() < 1e-14
        })
    }

    /// Knot vector in the u direction.
    #[must_use]
    pub fn knots_u(&self) -> &[f64] {
        &self.knots_u
    }

    /// Knot vector in the v direction.
    #[must_use]
    pub fn knots_v(&self) -> &[f64] {
        &self.knots_v
    }

    /// Reference to the control point grid.
    #[must_use]
    pub fn control_points(&self) -> &[Vec<Point3>] {
        &self.control_points
    }

    /// Reference to the weights grid.
    #[must_use]
    pub fn weights(&self) -> &[Vec<f64>] {
        &self.weights
    }

    /// Validate the stored rational weights after construction or
    /// deserialization.
    ///
    /// # Errors
    ///
    /// Returns [`MathError::InvalidWeightValue`] for a non-finite or
    /// non-positive weight.
    pub fn validate_weights(&self) -> Result<(), MathError> {
        validate_weight_values(&self.weights)
    }

    /// Validate all structural invariants normally enforced by [`Self::new`].
    ///
    /// This is intended for serialization formats that populate the private
    /// fields directly in order to preserve their exact floating-point values.
    ///
    /// # Errors
    ///
    /// Returns the same validation errors as [`Self::new`].
    pub fn validate(&self) -> Result<(), MathError> {
        Self::new(
            self.degree_u,
            self.degree_v,
            self.knots_u.clone(),
            self.knots_v.clone(),
            self.control_points.clone(),
            self.weights.clone(),
        )?;
        Ok(())
    }

    /// Evaluate the surface at parameters `(u, v)`.
    ///
    /// Uses tensor-product basis function evaluation (NURBS Book A3.5).
    #[must_use]
    pub fn evaluate(&self, u: f64, v: f64) -> Point3 {
        let pu = self.degree_u;
        let pv = self.degree_v;
        let n_rows = self.control_points.len();
        let n_cols = self.control_points[0].len();
        let u = u.clamp(self.knots_u[pu], self.knots_u[n_rows]);
        let v = v.clamp(self.knots_v[pv], self.knots_v[n_cols]);

        let span_u = basis::find_span(n_rows, pu, u, &self.knots_u);
        let span_v = basis::find_span(n_cols, pv, v, &self.knots_v);
        let mut nu_stack = [0.0f64; basis::MAX_STACK_OUTPUT + 1];
        let mut nu_heap;
        let nu: &mut [f64] = if pu <= basis::MAX_STACK_OUTPUT {
            &mut nu_stack[..=pu]
        } else {
            nu_heap = vec![0.0; pu + 1];
            &mut nu_heap
        };
        basis::basis_funs_into(span_u, u, pu, &self.knots_u, nu);
        let mut nv_stack = [0.0f64; basis::MAX_STACK_OUTPUT + 1];
        let mut nv_heap;
        let nv: &mut [f64] = if pv <= basis::MAX_STACK_OUTPUT {
            &mut nv_stack[..=pv]
        } else {
            nv_heap = vec![0.0; pv + 1];
            &mut nv_heap
        };
        basis::basis_funs_into(span_v, v, pv, &self.knots_v, nv);

        // Contract along v first for each relevant u-row, then along u.
        let scale = nu
            .iter()
            .enumerate()
            .take(pu + 1)
            .flat_map(|(i, &nu_i)| {
                nv.iter().enumerate().take(pv + 1).map(move |(j, &nv_j)| {
                    let u_idx = span_u - pu + i;
                    let v_idx = span_v - pv + j;
                    (nu_i * nv_j * self.weights[u_idx][v_idx]).abs()
                })
            })
            .fold(0.0_f64, f64::max);
        let mut wx = 0.0;
        let mut wy = 0.0;
        let mut wz = 0.0;
        let mut ww = 0.0;

        for (i, &nu_i) in nu.iter().enumerate().take(pu + 1) {
            let u_idx = span_u - pu + i;
            // Evaluate the v-direction for this row.
            let mut row_x = 0.0;
            let mut row_y = 0.0;
            let mut row_z = 0.0;
            let mut row_w = 0.0;
            for (j, &nv_j) in nv.iter().enumerate().take(pv + 1) {
                let v_idx = span_v - pv + j;
                let pt = &self.control_points[u_idx][v_idx];
                let w = self.weights[u_idx][v_idx];
                let bw = nv_j * w / scale;
                row_x += bw * pt.x();
                row_y += bw * pt.y();
                row_z += bw * pt.z();
                row_w += bw;
            }
            wx += nu_i * row_x;
            wy += nu_i * row_y;
            wz += nu_i * row_z;
            ww += nu_i * row_w;
        }

        debug_assert!(scale.is_finite() && scale > 0.0);
        debug_assert!(ww.is_finite() && ww > 0.0);
        Point3::new(wx / ww, wy / ww, wz / ww)
    }

    /// Compute surface derivatives up to order `d` at parameters `(u, v)`.
    ///
    /// Returns a 2D vector `ders[k][l]` representing the mixed partial
    /// derivative `∂^(k+l)S / ∂u^k ∂v^l` as a `Vec3`.
    ///
    /// Uses NURBS Book A3.6 + A4.4 (rational quotient rule).
    ///
    /// The returned table costs one heap allocation per row plus the outer
    /// vector. Hot loops that call this per quadrature abscissa should use
    /// [`Self::derivatives_into`] with a reused scratch buffer instead.
    #[must_use]
    #[allow(clippy::many_single_char_names, clippy::cast_precision_loss)]
    pub fn derivatives(&self, u: f64, v: f64, d: usize) -> Vec<Vec<Vec3>> {
        let mut scratch = DerivativeScratch::new();
        let mut out = vec![vec![Vec3::new(0.0, 0.0, 0.0); d + 1]; d + 1];
        self.derivatives_into(u, v, d, &mut scratch, &mut out);
        out
    }

    /// Compute surface derivatives up to order `d`, writing into caller-owned
    /// storage: `out[k][l]` receives `∂^(k+l)S / ∂u^k ∂v^l`.
    ///
    /// `out` must have at least `d + 1` rows of at least `d + 1` entries; only
    /// entries with `k + l <= d` (clamped by each axis degree) are written.
    /// `scratch` carries the reusable basis/homogeneous buffers so repeated
    /// calls (e.g. per quadrature abscissa) perform no allocation for the
    /// orders every hot path uses; larger orders take a heap path sized by the
    /// request. Results are bit-identical to [`Self::derivatives`]: same
    /// spans, same basis values, same contraction and quotient order.
    #[allow(clippy::many_single_char_names, clippy::cast_precision_loss)]
    pub fn derivatives_into(
        &self,
        u: f64,
        v: f64,
        d: usize,
        scratch: &mut DerivativeScratch,
        out: &mut [Vec<Vec3>],
    ) {
        scratch.ensure_basis_for(self, d);
        scratch.ensure_sk_for(d);
        self.derivatives_into_with_buffers(u, v, d, &mut scratch.basis, &mut scratch.sk, out);
    }

    /// [`Self::derivatives_into`] with the scratch buffers passed explicitly,
    /// so a caller holding both the output table and the scratch (e.g. two
    /// disjoint fields of one struct) can borrow both at once.
    #[allow(clippy::many_single_char_names, clippy::cast_precision_loss)]
    fn derivatives_into_with_buffers(
        &self,
        u: f64,
        v: f64,
        d: usize,
        basis_buf: &mut [f64],
        sk_buf: &mut [Vec3],
        out: &mut [Vec<Vec3>],
    ) {
        let n_rows = self.control_points.len();
        let n_cols = self.control_points[0].len();
        let u = u.clamp(self.knots_u[self.degree_u], self.knots_u[n_rows]);
        let v = v.clamp(self.knots_v[self.degree_v], self.knots_v[n_cols]);
        let span_u = basis::find_span(n_rows, self.degree_u, u, &self.knots_u);
        let span_v = basis::find_span(n_cols, self.degree_v, v, &self.knots_v);
        self.derivatives_into_with_spans(u, v, d, span_u, span_v, basis_buf, sk_buf, out);
    }

    /// Verify a hinted span the way [`basis::find_span`] defines it: the index
    /// `i` with `knots[i] <= t < knots[i + 1]`, clamped to
    /// `[degree, n - 1]`. Returns `None` when the hint misses, including when
    /// it is out of range for this knot vector (e.g. a fresh scratch).
    fn verify_span_hint(
        knots: &[f64],
        n: usize,
        degree: usize,
        t: f64,
        hint: usize,
    ) -> Option<usize> {
        if hint < degree || hint >= n {
            return None;
        }
        // `find_span` clamps both domain ends before searching, so the hint
        // check must accept the same clamped outcomes: at or past either end
        // the answer is the end span regardless of the knot comparison.
        if t >= knots[n] {
            return (hint == n - 1).then_some(hint);
        }
        if t <= knots[degree] {
            return (hint == degree).then_some(hint);
        }
        (knots[hint] <= t && t < knots[hint + 1]).then_some(hint)
    }

    /// [`basis::find_span`] on the u axis, consulting `hint` first.
    fn find_span_hinted_u(&self, u: f64, hint: usize) -> (usize, bool) {
        let n_rows = self.control_points.len();
        if let Some(span) = Self::verify_span_hint(&self.knots_u, n_rows, self.degree_u, u, hint) {
            return (span, true);
        }
        let u = u.clamp(self.knots_u[self.degree_u], self.knots_u[n_rows]);
        (
            basis::find_span(n_rows, self.degree_u, u, &self.knots_u),
            false,
        )
    }

    /// [`basis::find_span`] on the v axis, consulting `hint` first.
    fn find_span_hinted_v(&self, v: f64, hint: usize) -> (usize, bool) {
        let n_cols = self.control_points[0].len();
        if let Some(span) = Self::verify_span_hint(&self.knots_v, n_cols, self.degree_v, v, hint) {
            return (span, true);
        }
        let v = v.clamp(self.knots_v[self.degree_v], self.knots_v[n_cols]);
        (
            basis::find_span(n_cols, self.degree_v, v, &self.knots_v),
            false,
        )
    }

    /// [`Self::derivatives_into_with_buffers`] with pre-resolved knot spans.
    ///
    /// `span_u`/`span_v` must be what [`basis::find_span`] returns for the
    /// (clamped) `(u, v)` — which is exactly what the hinted lookup above
    /// guarantees, hit or miss. Splitting span resolution out of the solve
    /// lets a hot loop pay the binary search only when the abscissa leaves
    /// the previous span; the arithmetic below is untouched.
    #[allow(
        clippy::many_single_char_names,
        clippy::cast_precision_loss,
        clippy::too_many_arguments
    )]
    fn derivatives_into_with_spans(
        &self,
        u: f64,
        v: f64,
        d: usize,
        span_u: usize,
        span_v: usize,
        basis_buf: &mut [f64],
        sk_buf: &mut [Vec3],
        out: &mut [Vec<Vec3>],
    ) {
        let pu = self.degree_u;
        let pv = self.degree_v;
        let u = u.clamp(self.knots_u[pu], self.knots_u[self.control_points.len()]);
        let v = v.clamp(self.knots_v[pv], self.knots_v[self.control_points[0].len()]);
        let du = d.min(pu);
        let dv = d.min(pv);
        let stride_u = pu + 1;
        let required_u = (du + 1) * stride_u;
        let stride_v = pv + 1;
        let required_v = (dv + 1) * stride_v;
        let (ders_u, ders_v) = basis_buf.split_at_mut(required_u);
        let ders_v = &mut ders_v[..required_v];
        basis::ders_basis_funs_into(span_u, u, pu, du, &self.knots_u, ders_u);
        basis::ders_basis_funs_into(span_v, v, pv, dv, &self.knots_v, ders_v);

        // Compute homogeneous derivatives Aw[k][l] = (wx, wy, wz, w), stored
        // row-major with stride `d + 1` in a stack buffer for the orders every
        // hot path uses (d <= MAX_STACK_OUTPUT), heap otherwise.
        let n = d + 1;
        let mut aw_stack =
            [[0.0f64; 4]; (basis::MAX_STACK_OUTPUT + 1) * (basis::MAX_STACK_OUTPUT + 1)];
        let mut aw_heap;
        let aw: &mut [[f64; 4]] = if n * n <= aw_stack.len() {
            &mut aw_stack[..n * n]
        } else {
            aw_heap = vec![[0.0f64; 4]; n * n];
            &mut aw_heap
        };
        let weight_scale = self.max_weight();
        debug_assert!(weight_scale.is_finite() && weight_scale > 0.0);
        for k in 0..=du {
            for l in 0..=dv {
                if k + l > d {
                    continue;
                }
                for i in 0..=pu {
                    let du_ki = ders_u[k * stride_u + i];
                    let u_idx = span_u - pu + i;
                    for j in 0..=pv {
                        let dv_lj = ders_v[l * stride_v + j];
                        let v_idx = span_v - pv + j;
                        let pt = &self.control_points[u_idx][v_idx];
                        let w = self.weights[u_idx][v_idx] / weight_scale;
                        let coeff = du_ki * dv_lj;
                        let cell = &mut aw[k * n + l];
                        cell[0] += coeff * pt.x() * w;
                        cell[1] += coeff * pt.y() * w;
                        cell[2] += coeff * pt.z() * w;
                        cell[3] += coeff * w;
                    }
                }
            }
        }

        // Apply rational quotient rule (A4.4).
        let zero = Vec3::new(0.0, 0.0, 0.0);
        for entry in sk_buf.iter_mut() {
            *entry = zero;
        }
        let sk = &mut *sk_buf;
        let skl = |sk: &[Vec3], k: usize, l: usize| sk[k * n + l];
        let w0 = aw[0][3];

        for k in 0..=du {
            for l in 0..=dv {
                if k + l > d {
                    continue;
                }
                let mut v3 = [aw[k * n + l][0], aw[k * n + l][1], aw[k * n + l][2]];

                for j in 1..=l {
                    let bin = binomial(l, j) as f64;
                    let wj = aw[j][3];
                    let s = skl(sk, k, l - j);
                    v3[0] -= bin * wj * s.x();
                    v3[1] -= bin * wj * s.y();
                    v3[2] -= bin * wj * s.z();
                }

                for i in 1..=k {
                    let bin = binomial(k, i) as f64;
                    let wi = aw[i * n][3];
                    let s = skl(sk, k - i, l);
                    v3[0] -= bin * wi * s.x();
                    v3[1] -= bin * wi * s.y();
                    v3[2] -= bin * wi * s.z();

                    let mut v2 = [0.0f64; 3];
                    for j in 1..=l {
                        let bin2 = binomial(l, j) as f64;
                        let wij = aw[i * n + j][3];
                        let s = skl(sk, k - i, l - j);
                        v2[0] += bin2 * wij * s.x();
                        v2[1] += bin2 * wij * s.y();
                        v2[2] += bin2 * wij * s.z();
                    }
                    v3[0] -= bin * v2[0];
                    v3[1] -= bin * v2[1];
                    v3[2] -= bin * v2[2];
                }

                debug_assert!(w0.is_finite() && w0 > 0.0);
                sk[k * n + l] = Vec3::new(v3[0] / w0, v3[1] / w0, v3[2] / w0);
            }
        }

        for k in 0..=du {
            for l in 0..=dv {
                if k + l > d {
                    continue;
                }
                out[k][l] = sk[k * n + l];
            }
        }
    }

    /// Compute the unit normal vector at parameters `(u, v)`.
    ///
    /// The normal is the cross product of the u- and v-partial derivatives,
    /// normalized. At degenerate points (poles, collapsed edges) where
    /// `du × dv ≈ 0`, falls back to perturbing the parameter slightly
    /// in each direction and retrying — an L'Hôpital-style approach.
    ///
    /// # Errors
    ///
    /// Returns [`MathError::ZeroVector`] if the surface is degenerate at
    /// this point and all fallback perturbations also fail.
    pub fn normal(&self, u: f64, v: f64) -> Result<Vec3, MathError> {
        let d = self.derivatives(u, v, 1);
        let du = d[1][0];
        let dv = d[0][1];
        let cross = du.cross(dv);

        if cross.length_squared() > 1e-30 {
            return cross.normalize();
        }

        // Degenerate point — try perturbing the parameter slightly.
        let (u0, u1) = self.domain_u();
        let (v0, v1) = self.domain_v();
        let eps_u = (u1 - u0) * 1e-6;
        let eps_v = (v1 - v0) * 1e-6;

        let perturbations = [
            (u + eps_u, v),
            (u - eps_u, v),
            (u, v + eps_v),
            (u, v - eps_v),
        ];

        for (pu, pv) in perturbations {
            let pu = pu.clamp(u0, u1);
            let pv = pv.clamp(v0, v1);
            let pd = self.derivatives(pu, pv, 1);
            let pdu = pd[1][0];
            let pdv = pd[0][1];
            let pcross = pdu.cross(pdv);
            if pcross.length_squared() > 1e-30 {
                return pcross.normalize();
            }
        }

        Err(MathError::ZeroVector)
    }

    /// Compute an axis-aligned bounding box from control point extrema.
    #[must_use]
    pub fn aabb(&self) -> Aabb3 {
        Aabb3::from_points(
            self.control_points
                .iter()
                .flat_map(|row| row.iter().copied()),
        )
    }

    /// Create a cached evaluator for repeated evaluation.
    ///
    /// The evaluator lazily precomputes polynomial coefficients for Horner
    /// evaluation, amortising the setup cost over many evaluations.
    #[must_use]
    pub fn evaluator(&self) -> SurfaceEvaluator<'_> {
        SurfaceEvaluator::new(self)
    }
}

fn validate_weight_values(weights: &[Vec<f64>]) -> Result<(), MathError> {
    let mut index = 0;
    for row in weights {
        for &value in row {
            if !value.is_finite() || value <= 0.0 {
                return Err(MathError::InvalidWeightValue { index, value });
            }
            index += 1;
        }
    }
    Ok(())
}

use super::basis::binomial;

#[cfg(test)]
#[allow(clippy::expect_used, clippy::cast_lossless, clippy::suboptimal_flops)]
mod tests {
    use super::*;

    /// A valid bilinear patch, with one control point substituted.
    fn bilinear_with(point: Point3) -> Result<NurbsSurface, MathError> {
        NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
                vec![Point3::new(1.0, 0.0, 0.0), point],
            ],
            vec![vec![1.0, 1.0], vec![1.0, 1.0]],
        )
    }

    #[test]
    fn rejects_nonfinite_control_points() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(
                matches!(
                    bilinear_with(Point3::new(1.0, bad, 0.0)),
                    // Row-major flattening: the substituted point is index 3.
                    Err(MathError::InvalidControlPointValue { index: 3, .. })
                ),
                "expected rejection for {bad}"
            );
        }
        assert!(bilinear_with(Point3::new(1.0, 1.0, 0.0)).is_ok());
    }

    #[test]
    fn rejects_degree_zero_in_either_direction() {
        let points = vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        ];
        let weights = vec![vec![1.0, 1.0], vec![1.0, 1.0]];
        // Degree 0 has a knot vector one shorter than degree 1, so both the
        // count and the degree are legal-looking in isolation.
        assert!(matches!(
            NurbsSurface::new(
                0,
                1,
                vec![0.0, 0.5, 1.0],
                vec![0.0, 0.0, 1.0, 1.0],
                points.clone(),
                weights.clone(),
            ),
            Err(MathError::InvalidDegree { degree: 0, .. })
        ));
        assert!(matches!(
            NurbsSurface::new(
                1,
                0,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![0.0, 0.5, 1.0],
                points,
                weights,
            ),
            Err(MathError::InvalidDegree { degree: 0, .. })
        ));
    }

    #[test]
    fn rejects_degree_without_enough_control_points() {
        assert!(matches!(
            NurbsSurface::new(
                2,
                1,
                vec![0.0, 0.0, 0.0, 1.0, 1.0],
                vec![0.0, 0.0, 1.0, 1.0],
                vec![
                    vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
                    vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
                ],
                vec![vec![1.0, 1.0], vec![1.0, 1.0]],
            ),
            Err(MathError::InvalidDegree {
                degree: 2,
                control_points: 2
            })
        ));
    }

    #[test]
    fn rationality_follows_the_weight_grid() {
        let points = vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        ];
        let make = |weights| {
            NurbsSurface::new(
                1,
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![0.0, 0.0, 1.0, 1.0],
                points.clone(),
                weights,
            )
            .expect("test surface should be valid")
        };

        assert!(!make(vec![vec![1.0; 2]; 2]).is_rational());
        assert!(make(vec![vec![1.0, 0.5], vec![1.0, 1.0]]).is_rational());
    }

    #[test]
    fn rejects_nonpositive_and_nonfinite_weights() {
        let points = vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        ];
        for bad in [0.0, -1.0, f64::INFINITY] {
            let mut weights = vec![vec![1.0; 2]; 2];
            weights[1][1] = bad;
            assert!(matches!(
                NurbsSurface::new(
                    1,
                    1,
                    vec![0.0, 0.0, 1.0, 1.0],
                    vec![0.0, 0.0, 1.0, 1.0],
                    points.clone(),
                    weights,
                ),
                Err(MathError::InvalidWeightValue { .. })
            ));
        }
    }

    #[test]
    fn rejects_nonfinite_and_decreasing_knots() {
        let points = vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        ];
        let make = |knots_u, knots_v| {
            NurbsSurface::new(
                1,
                1,
                knots_u,
                knots_v,
                points.clone(),
                vec![vec![1.0; 2]; 2],
            )
        };

        assert!(matches!(
            make(vec![1.0, 1.0, 0.0, 0.0], vec![0.0, 0.0, 1.0, 1.0]),
            Err(MathError::InvalidKnotValue { .. })
        ));
        assert!(matches!(
            make(vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, f64::INFINITY, 1.0]),
            Err(MathError::InvalidKnotValue { .. })
        ));
    }

    #[test]
    fn common_tiny_weight_scale_evaluates_stably() {
        let surface = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
                vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
            ],
            vec![vec![1e-300; 2]; 2],
        )
        .expect("tiny positive weights are projectively valid");
        let point = surface.evaluate(0.5, 0.5);
        let derivatives = surface.derivatives(0.5, 0.5, 1);
        let mut evaluator = surface.evaluator();
        let cached = evaluator.point(0.5, 0.5);
        assert!((point.x() - 0.5).abs() < 1e-12);
        assert!((point.y() - 0.5).abs() < 1e-12);
        assert!((cached - point).length() < 1e-12);
        assert!(
            derivatives
                .iter()
                .flatten()
                .all(|v| v.x().is_finite() && v.y().is_finite() && v.z().is_finite())
        );
    }

    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn degree_nine_surface_and_cached_evaluator_do_not_panic() {
        let degree = 9;
        let mut knots = vec![0.0; degree + 1];
        knots.extend(std::iter::repeat_n(1.0, degree + 1));
        let control_points: Vec<Vec<_>> = (0..=degree)
            .map(|i| {
                (0..=degree)
                    .map(|j| Point3::new(i as f64, j as f64, 0.0))
                    .collect()
            })
            .collect();
        let surface = NurbsSurface::new(
            degree,
            degree,
            knots.clone(),
            knots,
            control_points,
            vec![vec![1.0; degree + 1]; degree + 1],
        )
        .expect("valid degree-nine Bezier surface");

        let direct = surface.evaluate(0.5, 0.5);
        let derivatives = surface.derivatives(0.5, 0.5, degree);
        let mut evaluator = surface.evaluator();
        let cached = evaluator.point(0.5, 0.5);
        let normal = evaluator.normal(0.5, 0.5);

        assert!((direct.x() - 4.5).abs() < 1e-10);
        assert!((direct.y() - 4.5).abs() < 1e-10);
        assert!((cached - direct).length() < 1e-8);
        assert!(normal.length_squared().is_finite());
        assert!(derivatives.iter().flatten().all(|derivative| {
            derivative.x().is_finite() && derivative.y().is_finite() && derivative.z().is_finite()
        }));
    }

    /// A bilinear surface (degree 1x1): a flat quadrilateral.
    fn bilinear_surface() -> NurbsSurface {
        NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
                vec![Point3::new(0.0, 1.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
            ],
            vec![vec![1.0, 1.0], vec![1.0, 1.0]],
        )
        .expect("valid bilinear surface")
    }

    /// A bicubic surface patch.
    fn bicubic_surface() -> NurbsSurface {
        let mut cps = Vec::new();
        let mut ws = Vec::new();
        for i in 0..4 {
            let mut row = Vec::new();
            let mut wrow = Vec::new();
            for j in 0..4 {
                row.push(Point3::new(
                    j as f64,
                    i as f64,
                    ((i + j) as f64 * 0.5).sin(),
                ));
                wrow.push(1.0);
            }
            cps.push(row);
            ws.push(wrow);
        }
        NurbsSurface::new(
            3,
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            cps,
            ws,
        )
        .expect("valid bicubic surface")
    }

    #[test]
    fn bilinear_corners() {
        let s = bilinear_surface();
        let p00 = s.evaluate(0.0, 0.0);
        let p10 = s.evaluate(1.0, 0.0);
        let p01 = s.evaluate(0.0, 1.0);
        let p11 = s.evaluate(1.0, 1.0);

        assert!((p00.x()).abs() < 1e-14);
        assert!((p00.y()).abs() < 1e-14);
        assert!((p10.x() - 0.0).abs() < 1e-14);
        assert!((p10.y() - 1.0).abs() < 1e-14);
        assert!((p01.x() - 1.0).abs() < 1e-14);
        assert!((p01.y() - 0.0).abs() < 1e-14);
        assert!((p11.x() - 1.0).abs() < 1e-14);
        assert!((p11.y() - 1.0).abs() < 1e-14);
    }

    #[test]
    fn bilinear_midpoint() {
        let s = bilinear_surface();
        let mid = s.evaluate(0.5, 0.5);
        assert!((mid.x() - 0.5).abs() < 1e-14);
        assert!((mid.y() - 0.5).abs() < 1e-14);
        assert!((mid.z()).abs() < 1e-14);
    }

    #[test]
    fn bilinear_normal() {
        let s = bilinear_surface();
        let n = s.normal(0.5, 0.5).expect("non-degenerate");
        // Flat surface in XY plane, normal should be (0, 0, ±1).
        assert!((n.x()).abs() < 1e-12);
        assert!((n.y()).abs() < 1e-12);
        assert!((n.z().abs() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn bicubic_endpoint_interpolation() {
        let s = bicubic_surface();
        let p = s.evaluate(0.0, 0.0);
        let cp = &s.control_points()[0][0];
        assert!((p.x() - cp.x()).abs() < 1e-14);
        assert!((p.y() - cp.y()).abs() < 1e-14);
        assert!((p.z() - cp.z()).abs() < 1e-14);
    }

    #[test]
    fn derivatives_zeroth_matches_evaluate() {
        let s = bicubic_surface();
        let p = s.evaluate(0.5, 0.5);
        let d = s.derivatives(0.5, 0.5, 1);
        assert!((d[0][0].x() - p.x()).abs() < 1e-12);
        assert!((d[0][0].y() - p.y()).abs() < 1e-12);
        assert!((d[0][0].z() - p.z()).abs() < 1e-12);
    }

    #[test]
    fn aabb_contains_all_control_points() {
        let s = bicubic_surface();
        let bb = s.aabb();
        for row in s.control_points() {
            for pt in row {
                assert!(bb.contains_point(*pt));
            }
        }
    }

    #[test]
    fn nurbs_partial_matches_finite_difference() {
        use crate::traits::ParametricSurface;

        let s = bicubic_surface();
        let u = 0.5;
        let v = 0.5;
        let h = 1e-6;

        // Central finite difference for du
        let p_plus = s.evaluate(u + h, v);
        let p_minus = s.evaluate(u - h, v);
        let fd_u = Vec3::new(
            (p_plus.x() - p_minus.x()) / (2.0 * h),
            (p_plus.y() - p_minus.y()) / (2.0 * h),
            (p_plus.z() - p_minus.z()) / (2.0 * h),
        );
        let du = ParametricSurface::partial_u(&s, u, v);
        assert!(
            (du.x() - fd_u.x()).abs() < 1e-4,
            "du.x: {} vs {}",
            du.x(),
            fd_u.x()
        );
        assert!(
            (du.y() - fd_u.y()).abs() < 1e-4,
            "du.y: {} vs {}",
            du.y(),
            fd_u.y()
        );
        assert!(
            (du.z() - fd_u.z()).abs() < 1e-4,
            "du.z: {} vs {}",
            du.z(),
            fd_u.z()
        );

        // Central finite difference for dv
        let p_plus = s.evaluate(u, v + h);
        let p_minus = s.evaluate(u, v - h);
        let fd_v = Vec3::new(
            (p_plus.x() - p_minus.x()) / (2.0 * h),
            (p_plus.y() - p_minus.y()) / (2.0 * h),
            (p_plus.z() - p_minus.z()) / (2.0 * h),
        );
        let dv = ParametricSurface::partial_v(&s, u, v);
        assert!(
            (dv.x() - fd_v.x()).abs() < 1e-4,
            "dv.x: {} vs {}",
            dv.x(),
            fd_v.x()
        );
        assert!(
            (dv.y() - fd_v.y()).abs() < 1e-4,
            "dv.y: {} vs {}",
            dv.y(),
            fd_v.y()
        );
        assert!(
            (dv.z() - fd_v.z()).abs() < 1e-4,
            "dv.z: {} vs {}",
            dv.z(),
            fd_v.z()
        );
    }

    use proptest::prelude::*;

    proptest! {
        #[test]
        fn prop_bilinear_linear_interpolation(u in 0.0f64..=1.0, v in 0.0f64..=1.0) {
            let s = bilinear_surface();
            let p = s.evaluate(u, v);
            // Bilinear: S(u,v) = (v, u, 0) for our test surface
            prop_assert!((p.x() - v).abs() < 1e-12, "x: {} vs {}", p.x(), v);
            prop_assert!((p.y() - u).abs() < 1e-12, "y: {} vs {}", p.y(), u);
            prop_assert!(p.z().abs() < 1e-12);
        }
    }
}

#[cfg(test)]
mod weight_cache_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use crate::traits::ParametricSurface;

    /// A rational biquadratic patch with a spread of weights and a tiny
    /// common factor, the case the global weight scale exists for.
    fn rational_patch(factor: f64) -> NurbsSurface {
        let pts: Vec<Vec<Point3>> = (0..4)
            .map(|i| {
                (0..3)
                    .map(|j| {
                        let (x, y) = (f64::from(i), f64::from(j));
                        Point3::new(x, y, (x * y).sin() * 0.5 + 0.1 * x * x)
                    })
                    .collect()
            })
            .collect();
        let weights: Vec<Vec<f64>> = (0..4)
            .map(|i| {
                (0..3)
                    .map(|j| factor * (1.0 + 0.3 * f64::from(i) + 0.2 * f64::from(j)))
                    .collect()
            })
            .collect();
        NurbsSurface::new(
            2,
            2,
            vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            pts,
            weights,
        )
        .unwrap()
    }

    fn bits(v: Vec3) -> [u64; 3] {
        [v.x().to_bits(), v.y().to_bits(), v.z().to_bits()]
    }

    #[test]
    fn max_weight_is_the_global_maximum_and_is_cached_once() {
        let s = rational_patch(1e-200);
        let expected = s
            .weights()
            .iter()
            .flatten()
            .copied()
            .fold(0.0_f64, f64::max);
        assert_eq!(s.max_weight().to_bits(), expected.to_bits());
        // Evaluations must not recompute the scale: the cell is filled by `new`
        // and stays the same object afterwards.
        let before = s.max_weight.get().copied();
        for k in 0..25 {
            let t = f64::from(k) / 24.0;
            let _ = s.derivatives(t, 1.0 - t, 2);
        }
        assert_eq!(s.max_weight.get().copied(), before);
        assert!(before.is_some());
    }

    #[test]
    fn partials_are_bitwise_the_separate_partials() {
        let s = rational_patch(1.0);
        for k in 0..30 {
            let (u, v) = (f64::from(k) / 29.0, (f64::from(k) * 0.37) % 1.0);
            let (du, dv) = ParametricSurface::partials(&s, u, v);
            assert_eq!(bits(du), bits(ParametricSurface::partial_u(&s, u, v)));
            assert_eq!(bits(dv), bits(ParametricSurface::partial_v(&s, u, v)));
        }
    }

    #[test]
    fn point_and_partials_matches_separate_evaluations_to_rounding() {
        // The position takes a different summation path than `evaluate`
        // (homogeneous quotient vs. scaled perspective divide), so it agrees
        // to rounding, not bit-identically; the partials are the same table
        // entries either way.
        let s = rational_patch(1.0);
        for k in 0..30 {
            let (u, v) = (f64::from(k) / 29.0, (f64::from(k) * 0.37) % 1.0);
            let (p, du, dv) = ParametricSurface::point_and_partials(&s, u, v);
            let q = ParametricSurface::evaluate(&s, u, v);
            let drift = (p.x() - q.x()).hypot(p.y() - q.y()).hypot(p.z() - q.z());
            assert!(
                drift < 1e-12,
                "position drift {drift:.3e} at ({u}, {v}) exceeds rounding"
            );
            assert_eq!(bits(du), bits(ParametricSurface::partial_u(&s, u, v)));
            assert_eq!(bits(dv), bits(ParametricSurface::partial_v(&s, u, v)));
        }
    }

    #[test]
    fn stack_and_heap_derivative_paths_agree_bitwise() {
        // d beyond the stack budget takes the heap buffer; the shared entries
        // must match the stack path exactly.
        let s = rational_patch(1e-120);
        let (u, v) = (0.31, 0.77);
        let low = s.derivatives(u, v, 2);
        let high = s.derivatives(u, v, basis::MAX_STACK_OUTPUT + 1);
        for k in 0..=2 {
            for l in 0..=2 {
                if k + l <= 2 {
                    assert_eq!(bits(low[k][l]), bits(high[k][l]), "S^({k},{l})");
                }
            }
        }
    }

    #[test]
    fn derivatives_into_matches_derivatives_bitwise() {
        use crate::vec::Vec3;
        // Scratch-buffer path must reproduce the allocating path exactly:
        // same spans, basis values, contraction and quotient order. Cover
        // d = 0..3 (including the degree-clamped d > pu/pv arms) and an order
        // that forces the heap `aw` path.
        let s = rational_patch(1.0);
        let mut scratch = DerivativeScratch::new();
        for d in 0..=3 {
            for k in 0..30 {
                let (u, v) = (f64::from(k) / 29.0, (f64::from(k) * 0.37) % 1.0);
                let mut out = vec![vec![Vec3::new(0.0, 0.0, 0.0); d + 1]; d + 1];
                s.derivatives_into(u, v, d, &mut scratch, &mut out);
                let expected = s.derivatives(u, v, d);
                for a in 0..=d {
                    for b in 0..=d {
                        // Entries outside the written triangle keep whatever
                        // the caller put there; only compare written cells.
                        if a + b > d || a > s.degree_u().min(d) || b > s.degree_v().min(d) {
                            continue;
                        }
                        assert_eq!(bits(out[a][b]), bits(expected[a][b]), "S^({a},{b}) d={d}");
                    }
                }
            }
        }
        // Heap `aw` path (d = MAX_STACK_OUTPUT + 1) agrees too.
        let (u, v) = (0.31, 0.77);
        let d = basis::MAX_STACK_OUTPUT + 1;
        let mut out = vec![vec![Vec3::new(0.0, 0.0, 0.0); d + 1]; d + 1];
        s.derivatives_into(u, v, d, &mut scratch, &mut out);
        let expected = s.derivatives(u, v, d);
        for a in 0..=2 {
            for b in 0..=2 {
                if a + b <= 2 {
                    assert_eq!(bits(out[a][b]), bits(expected[a][b]), "S^({a},{b}) heap");
                }
            }
        }
        // Reusing one scratch across many calls stays exact (no stale state).
        let mut out2 = vec![vec![Vec3::new(0.0, 0.0, 0.0); 2]; 2];
        for k in 0..50 {
            let (u, v) = (f64::from(k) / 49.0, 1.0 - f64::from(k) / 49.0);
            s.derivatives_into(u, v, 1, &mut scratch, &mut out2);
            let expected = s.derivatives(u, v, 1);
            assert_eq!(bits(out2[0][0]), bits(expected[0][0]));
            assert_eq!(bits(out2[1][0]), bits(expected[1][0]));
            assert_eq!(bits(out2[0][1]), bits(expected[0][1]));
        }
    }

    #[test]
    fn span_hinted_solve_matches_unhinted_bitwise() {
        use crate::traits::ParametricSurface;
        // Ascending walks (the quadrature order), span crossings, domain ends,
        // repeated knots, and out-of-domain clamping must all reproduce the
        // unhinted solve bit for bit — a hit feeds the same indices, a miss
        // takes the search. The bicubic single-span case covers the hint that
        // never misses; the rational two-span case covers crossings.
        let single = NurbsSurface::new(
            3,
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            (0..4)
                .map(|i| {
                    (0..4)
                        .map(|j| {
                            Point3::new(f64::from(j), f64::from(i), (f64::from(i + j) * 0.5).sin())
                        })
                        .collect()
                })
                .collect(),
            vec![vec![1.0; 4]; 4],
        )
        .unwrap();
        let two_span = rational_patch(1.0);
        for s in [&single, &two_span] {
            // Dense ascending grid plus exact knots, ends, and out-of-domain.
            let (u0, u1) = s.domain_u();
            let (v0, v1) = s.domain_v();
            let mut params = Vec::new();
            for k in 0..200 {
                params.push((
                    u0 + (u1 - u0) * f64::from(k) / 199.0,
                    v0 + (v1 - v0) * (f64::from(k) * 0.618_033_988_7 % 1.0),
                ));
            }
            params.extend(
                s.knots_u()
                    .iter()
                    .flat_map(|&u| s.knots_v().iter().map(move |&v| (u, v))),
            );
            params.extend([
                (u0, v0),
                (u1, v1),
                (u0 - 0.25, v0 - 0.25),
                (u1 + 0.25, v1 + 0.25),
                (f64::midpoint(u0, u1), v1 + 1.0),
            ]);
            // One shared scratch per surface, walked twice: ascending (high
            // hit rate) then shuffled (miss-heavy). Both must match.
            for order in [false, true] {
                let mut scratch = DerivativeScratch::new();
                let mut seq = params.clone();
                if order {
                    seq.reverse();
                }
                for &(u, v) in &seq {
                    let (p, du, dv, _, _) =
                        ParametricSurface::span_hinted_point_and_partials_with_scratch(
                            s,
                            u,
                            v,
                            &mut scratch,
                        );
                    let (ep, edu, edv) = ParametricSurface::point_and_partials_with_scratch(
                        s,
                        u,
                        v,
                        &mut DerivativeScratch::new(),
                    );
                    assert_eq!(
                        bits(p - ep),
                        bits(Vec3::new(0.0, 0.0, 0.0)),
                        "p at ({u}, {v})"
                    );
                    assert_eq!(bits(du), bits(edu), "du at ({u}, {v})");
                    assert_eq!(bits(dv), bits(edv), "dv at ({u}, {v})");
                }
            }
        }
    }

    #[test]
    fn span_hint_reports_hits_on_quadrature_like_walks() {
        use crate::traits::ParametricSurface;
        // Guard against a vacuous hint: on an ascending walk inside one span
        // nearly every abscissa must hit. A single-span bicubic patch walked
        // at 200 ascending points hits 199/200 per axis (the first call has
        // no hint yet).
        let s = NurbsSurface::new(
            3,
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            (0..4)
                .map(|i| {
                    (0..4)
                        .map(|j| {
                            Point3::new(f64::from(j), f64::from(i), (f64::from(i + j) * 0.5).sin())
                        })
                        .collect()
                })
                .collect(),
            vec![vec![1.0; 4]; 4],
        )
        .unwrap();
        let mut scratch = DerivativeScratch::new();
        let (mut hit_u, mut hit_v) = (0usize, 0usize);
        for k in 0..200 {
            let t = f64::from(k) / 199.0;
            let (_, _, _, hu, hv) = ParametricSurface::span_hinted_point_and_partials_with_scratch(
                &s,
                t,
                t,
                &mut scratch,
            );
            hit_u += usize::from(hu);
            hit_v += usize::from(hv);
        }
        assert_eq!((hit_u, hit_v), (199, 199));
    }

    #[test]
    fn equality_ignores_the_cache_cell() {
        let a = rational_patch(0.5);
        let mut b = a.clone();
        b.max_weight = std::sync::OnceLock::new();
        assert!(b.max_weight.get().is_none());
        assert_eq!(a, b);
        assert_eq!(a.max_weight().to_bits(), b.max_weight().to_bits());
        let c = rational_patch(0.25);
        assert_ne!(a, c);
    }
}
