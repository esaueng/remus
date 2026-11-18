//! Exact moments of a cylinder face trimmed by exact curves, by Green's
//! theorem on its own boundary.
//!
//! The general trimmed path samples each trim edge into a 128-point
//! polygon in `(u, v)` and integrates the cells inside it. A curved trim
//! (the ellipses two equal crossing cylinders cut into each other) then
//! costs O(1/N²): a Steinmetz bicylinder r = 10 read 5333.2664 against
//! 16r³/3 = 5333.3333 (1.25e-5 low) although its B-rep is exact.
//!
//! Exact trims (lines, circles, ellipses) take this path, and so does a
//! fitted NURBS trim (the quartic section of unequal crossing cylinders)
//! whose every quadrature point lies on the cylinder within
//! [`NURBS_ON_SURFACE`] of its radius (and at least [`NURBS_ON_SURFACE_ABS`]).
//! Such a trim is read on the surface: each point is projected onto the
//! cylinder and its angular rate taken at the projected radius, so the face
//! is measured on the trim the B-rep stores. A fitted trim further off the
//! surface has no boundary on the cylinder to integrate, and the sampled
//! path keeps measuring it.
//!
//! Fitted trims are read here only for a moment (not area-only) reading. An
//! area-only request on a fitted face (`integrate_face_area`) is answered by
//! the moment reading, so it takes this path under the same guard.
//!
//! A caller that collects the vector area (the shell-closure and
//! order-convergence certificates of `solid_volume`) qualifies the reading it
//! is handed, so a fitted reading also reports what certifies it
//! ([`FittedBoundaryReading`]): how far its span-by-span and halved-span
//! volume terms differ (the convergence certificate an order comparison
//! cannot give, since this reading does not depend on the order), and how
//! far its vector area can stand from the one the stored boundary bounds:
//! every fitted point is read at its projection onto this surface, up to its
//! fit error off the neighbouring face's own reading of the edge (see
//! [`FittedEdges::add`]).
//!
//! The sampled path cannot be trusted on such trims either: when two
//! unequal crossing cylinders cut a single closed loop (a drill r 7.3 at
//! 3 mm off the axis of a bar r 10, so the drill breaks out through the
//! bar's top), both trimmed walls read about 1.1 mm³ high and the total
//! moved with the bar's seam (2.16, 2.03 or 2.07 mm³ high, esaueng/OpenZCAD
//! K10). On its boundary the same faces read within 0.01 mm³.
//!
//! On a cylinder `P(u, v) = O + r ρ(u) + v a` every integrand the
//! [`Accumulator`] sums is a polynomial of degree at most three in `v`
//! times a function of `u`. With `F(u, v) = ∫₀ᵛ f(u, s) ds`,
//! `∬_D f du dv = −∮_{∂D} F du` (Green, the boundary run counter-clockwise
//! in `(u, v)`, which is how the kernel orients a face's wires on its own
//! surface). `F` is single-valued and periodic in `u`, so the identity also
//! holds for bands whose loops wind the axis, and a seam traversed both
//! ways cancels. Each edge is integrated by composite Gauss–Legendre on its
//! own parameter, `F` by three Gauss points in `s` (exact for cubics), and
//! `du/dt` is the edge velocity projected on `∂P/∂u`. No `u` unwrapping is
//! needed: `u` enters only through the periodic surface point.

use remus_math::nurbs::surface::DerivativeScratch;
use remus_math::quadrature::gauss_legendre_points;
use remus_math::surfaces::CylindricalSurface;
use remus_math::traits::{ParametricCurve, ParametricSurface};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::Face;

use super::{Accumulator, FaceContribution, FittedBoundaryReading, IntegrationRule};
use crate::CheckError;

/// Composite pieces per edge and Gauss points per piece. A conic trim's
/// integrand is a trigonometric polynomial of low order in its parameter (a
/// cubic on a line), resolved to round-off well before this.
pub(super) const EDGE_PIECES: usize = 16;
pub(super) const EDGE_ORDER: usize = 16;
/// Gauss points in the axial integration variable `s ∈ [0, 1]`.
pub(super) const AXIAL_ORDER: usize = 3;
/// How far a fitted NURBS trim's quadrature points may lie off the cylinder
/// for the face to integrate on its boundary: the larger of
/// [`NURBS_ON_SURFACE_ABS`] and this fraction of the radius. Every point is
/// projected onto the cylinder before it is integrated, so the bound only
/// rejects trims that are not curves on this surface; the sampled path reads
/// the same projected trim. The kernel's crossing-cylinder sections sit up
/// to 1.4e-3 off a r 10 wall (1.4e-4 of the radius).
pub(super) const NURBS_ON_SURFACE: f64 = 1e-3;
/// Absolute floor of the on-surface bound, in model units.
pub(super) const NURBS_ON_SURFACE_ABS: f64 = 1e-3;
/// Relative agreement a fitted trim's span-by-span and halved-span readings
/// must reach in every term (see `readings_agree`).
const FITTED_CONVERGENCE: f64 = 1e-9;
/// Margin [`FittedEdges::add`] puts on its vector-area bound: the offsets it
/// is built from are read at the quadrature points only, and twice the bound
/// covers an offset between them up to twice the largest one read. It is no
/// certificate between the points (an excursion a parameterization crowds
/// between them goes unseen, as it does by the sampled trim and the
/// boundary-gap check); see [`FittedEdges::add`] for why an underestimate is
/// safe.
const CLOSURE_ALLOWANCE_MARGIN: f64 = 2.0;

/// A face's reading on its own boundary.
pub(super) struct GreenReading {
    pub(super) contribution: FaceContribution,
    /// The vector area `∫ n dA` from the same quadrature points.
    pub(super) flux: Vec3,
    /// What certifies the reading of its fitted edges; `None` when the face
    /// has none (exact trims only).
    pub(super) fitted: Option<FittedBoundaryReading>,
}

/// What one reading of an edge measured of it besides its integrals.
#[derive(Debug, Default, Clone, Copy)]
pub(super) struct EdgeMeasure {
    /// Largest distance of a quadrature point from its projection onto the
    /// surface (zero for an exact trim, which is read where it lies).
    pub(super) offset: f64,
    /// The stored edge's arc length, by the same quadrature.
    pub(super) length: f64,
    /// The arc length of its projection onto the surface.
    pub(super) projected_length: f64,
}

impl EdgeMeasure {
    /// Record one quadrature point: its weight `w` (the interval map
    /// included), its distance `offset` off the surface, and the speeds of
    /// the stored point and of its projection.
    pub(super) fn add(&mut self, w: f64, offset: f64, speed: f64, projected_speed: f64) {
        self.offset = self.offset.max(offset);
        self.length += w * speed;
        self.projected_length += w * projected_speed;
    }
}

/// The certificate of a face's fitted edges, summed edge by edge.
#[derive(Debug, Default)]
pub(super) struct FittedEdges {
    span_disagreement: f64,
    closure_allowance: f64,
    count: usize,
}

impl FittedEdges {
    /// Add one fitted edge from its span-by-span (`coarse`) and halved-span
    /// (`fine`) readings, what they measured of it, and the distances of its
    /// two end points from the integration reference.
    ///
    /// The volume disagreement is `|V_coarse − V_fine|`.
    ///
    /// The closure allowance bounds how far this face's vector area moves
    /// because the edge is read at its projection `C + a` onto the surface
    /// rather than where it is stored, `C`. A vector area is
    /// `½ ∮ (P − R) × dP` over the boundary, and with `∫ (C − R) × da`
    /// integrated by parts the edge moves it by
    /// `∫ a × dC + ½ [(C − R) × a] + ½ ∫ a × da`, which is at most
    /// `|a| (ℓ + ½ (ℓ + ℓₐ) + ½ (|C₀ − R| + |C₁ − R|))`, `ℓ` and `ℓₐ` being
    /// the stored and projected arc lengths (`∫ |da| ≤ ℓ + ℓₐ`). The stored
    /// edges bound a closed shell, whose vector areas cancel exactly, so a
    /// shell read this way closes to within the sum of its faces' allowances
    /// plus whatever its other faces' readings leave. `|a|` is the largest
    /// offset read, and the bound carries [`CLOSURE_ALLOWANCE_MARGIN`].
    ///
    /// It is an estimate from the offsets read, not a certified bound: an
    /// excursion between quadrature points, and an exact trim (line, circle,
    /// ellipse) that stands off the surface, are not counted. Exact trims are
    /// read where they lie, as the exact-trim boundary path always read them,
    /// and are left to the base closure bound as before. The allowance only
    /// ever adds to that bound, for faces read on a fitted boundary, so where
    /// it falls short the closure check is no looser than it was without
    /// it.
    pub(super) fn add(
        &mut self,
        coarse: (&Accumulator, EdgeMeasure),
        fine: (&Accumulator, EdgeMeasure),
        end_reach: [f64; 2],
    ) {
        let offset = coarse.1.offset.max(fine.1.offset);
        let (length, projected) = (fine.1.length, fine.1.projected_length);
        let bound = offset
            * 0.5f64.mul_add(
                end_reach[0] + end_reach[1],
                1.5f64.mul_add(length, 0.5 * projected),
            );
        self.span_disagreement += (coarse.0.vol - fine.0.vol).abs();
        self.closure_allowance += CLOSURE_ALLOWANCE_MARGIN * bound;
        self.count += 1;
    }

    /// The face's certificate, `None` when it has no fitted edge or a
    /// measure is not finite.
    pub(super) fn summary(&self) -> Option<FittedBoundaryReading> {
        (self.count > 0 && self.span_disagreement.is_finite() && self.closure_allowance.is_finite())
            .then_some(FittedBoundaryReading {
                span_disagreement: self.span_disagreement,
                closure_allowance: self.closure_allowance,
            })
    }
}

/// The edge's velocity `dP/dt` at `t` for an exact trim (line, circle,
/// ellipse); `None` for anything else.
pub(super) fn velocity(curve: &EdgeCurve, t: f64, start: Point3, end: Point3) -> Option<Vec3> {
    match curve {
        EdgeCurve::Line => Some(end - start),
        EdgeCurve::Circle(c) => ParametricCurve::derivative_pair(c, t).map(|(d1, _)| d1),
        EdgeCurve::Ellipse(e) => ParametricCurve::derivative_pair(e, t).map(|(d1, _)| d1),
        // A fitted trim is read on the surface; `integrate` checks it lies
        // there (see the module docs).
        EdgeCurve::NurbsCurve(n) => ParametricCurve::derivative_pair(n, t).map(|(d1, _)| d1),
        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => None,
    }
}

/// `[lo, hi]` cut at every distinct interior knot, each knot span then into
/// `per_span` equal pieces.
pub(super) fn knot_pieces(knots: &[f64], lo: f64, hi: f64, per_span: usize) -> Vec<(f64, f64)> {
    let mut breaks = vec![lo];
    for &knot in knots {
        if knot > lo && knot < hi && breaks.last().is_none_or(|&last| knot > last) {
            breaks.push(knot);
        }
    }
    breaks.push(hi);
    breaks
        .windows(2)
        .flat_map(|w| pieces(w[0], w[1], per_span))
        .collect()
}

/// `count` equal quadrature pieces of `[lo, hi]`.
pub(super) fn pieces(lo: f64, hi: f64, count: usize) -> impl Iterator<Item = (f64, f64)> {
    let step = (hi - lo) / count as f64;
    (0..count).map(move |k| (step.mul_add(k as f64, lo), step.mul_add((k + 1) as f64, lo)))
}

/// The face's contribution and vector area by Green's theorem, with the
/// certificate of its fitted edges.
///
/// `None` when a boundary edge is not a line, circle or ellipse (or, for a
/// moment reading, an on-surface NURBS curve; see the module docs), has no
/// stored domain, a fitted edge's two readings disagree or measure something
/// non-finite, or the boundary does not enclose positive area (an
/// orientation outside the convention this relies on): the caller keeps its
/// sampled path then.
pub(super) fn integrate<const AREA_ONLY: bool>(
    topo: &Topology,
    face: &Face,
    cylinder: &CylindricalSurface,
    rule: &IntegrationRule<'_>,
    sign: f64,
) -> Result<Option<GreenReading>, CheckError> {
    let fresh = || Accumulator {
        volume_only: rule.volume_only,
        ..Accumulator::default()
    };
    let mut acc = fresh();
    let Ok(axis) = cylinder.axis().normalize() else {
        return Ok(None);
    };
    let reading = Reading {
        cylinder,
        axis,
        reference: rule.reference,
        off_surface_limit: (NURBS_ON_SURFACE * cylinder.radius()).max(NURBS_ON_SURFACE_ABS),
    };
    // Fitted trims only for a moment reading (module docs).
    // `integrate_face_area` never asks for one area-only: it reads a fitted
    // face with its moments, whose volume the guard needs.
    let fitted_trims_allowed = !AREA_ONLY;
    let mut fitted = FittedEdges::default();
    // Every term compared at its own dimension of this length: the radius,
    // plus how far the integration reference lies from the cylinder.
    let length_scale = cylinder.radius() + (cylinder.origin() - rule.reference).length();

    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oriented in topo.wire(wire_id)?.edges() {
            let edge = topo.edge(oriented.edge())?;
            let span = EdgeSpan {
                curve: edge.curve(),
                start: topo.vertex(edge.start())?.point(),
                end: topo.vertex(edge.end())?.point(),
            };
            let Ok((t0, t1)) = edge.strict_domain() else {
                return Ok(None);
            };
            let (lo, hi) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
            // A reversed use runs the domain backwards.
            let direction = if oriented.is_forward() == (t0 <= t1) {
                1.0
            } else {
                -1.0
            };
            if !matches!(span.curve, EdgeCurve::NurbsCurve(_)) {
                let intervals: Vec<_> = pieces(lo, hi, EDGE_PIECES).collect();
                if !reading.edge::<AREA_ONLY>(
                    &span,
                    direction,
                    &intervals,
                    &mut acc,
                    &mut EdgeMeasure::default(),
                ) {
                    return Ok(None);
                }
                continue;
            }
            if !fitted_trims_allowed {
                return Ok(None);
            }
            // A fitted trim is a spline: the rule is exact-to-round-off only
            // inside one knot span, and a parameterization that crowds its
            // turn into a few spans is under-resolved by equal pieces. Read
            // it span by span, then with every span halved, and keep it only
            // where the two agree.
            let EdgeCurve::NurbsCurve(nurbs) = span.curve else {
                return Ok(None);
            };
            let (coarse_pieces, fine_pieces) = (
                knot_pieces(nurbs.knots(), lo, hi, 1),
                knot_pieces(nurbs.knots(), lo, hi, 2),
            );
            // Every term, even for a volume-only rule: the edge is kept on
            // the same agreement whatever the request, so a volume-only
            // reading takes the path the full one takes, bit for bit.
            let (mut coarse, mut fine) = (Accumulator::default(), Accumulator::default());
            let (mut coarse_measure, mut fine_measure) =
                (EdgeMeasure::default(), EdgeMeasure::default());
            if !reading.edge::<AREA_ONLY>(
                &span,
                direction,
                &coarse_pieces,
                &mut coarse,
                &mut coarse_measure,
            ) || !reading.edge::<AREA_ONLY>(
                &span,
                direction,
                &fine_pieces,
                &mut fine,
                &mut fine_measure,
            ) {
                return Ok(None);
            }
            let converged = readings_agree(&coarse, &fine, length_scale);
            if !converged {
                return Ok(None);
            }
            fitted.add(
                (&coarse, coarse_measure),
                (&fine, fine_measure),
                span.end_reach(lo, hi, rule.reference),
            );
            acc.absorb(&fine);
        }
    }
    Ok(finish(acc, sign, &fitted))
}

/// The reading `acc` sums, `None` when it encloses no positive area or its
/// fitted edges measured something non-finite.
pub(super) fn finish(acc: Accumulator, sign: f64, fitted: &FittedEdges) -> Option<GreenReading> {
    if !acc.area.is_finite() || acc.area <= 0.0 {
        return None;
    }
    let fitted_reading = fitted.summary();
    if fitted.count > 0 && fitted_reading.is_none() {
        return None;
    }
    let flux = acc.flux(sign);
    Some(GreenReading {
        contribution: acc.finish(sign),
        flux,
        fitted: fitted_reading,
    })
}

/// Whether two readings of one fitted edge agree in every term they carry
/// to [`FITTED_CONVERGENCE`], each against its own dimension of `length`:
/// area and vector area as length², volume and the surface-centroid sums as
/// length³, the first volume moments as length⁴ and the second as length⁵.
/// Comparing every term, not only area and volume, keeps a reading whose
/// higher moments are under-resolved from passing on a lucky cancellation.
pub(super) fn readings_agree(coarse: &Accumulator, fine: &Accumulator, length: f64) -> bool {
    let close = |a: f64, b: f64, power: i32| {
        (a - b).abs() <= FITTED_CONVERGENCE * (b.abs() + length.powi(power))
    };
    close(coarse.area, fine.area, 2)
        && close(coarse.nx, fine.nx, 2)
        && close(coarse.ny, fine.ny, 2)
        && close(coarse.nz, fine.nz, 2)
        && close(coarse.vol, fine.vol, 3)
        && close(coarse.cx, fine.cx, 3)
        && close(coarse.cy, fine.cy, 3)
        && close(coarse.cz, fine.cz, 3)
        && close(coarse.mx, fine.mx, 4)
        && close(coarse.my, fine.my, 4)
        && close(coarse.mz, fine.mz, 4)
        && close(coarse.qxx, fine.qxx, 5)
        && close(coarse.qyy, fine.qyy, 5)
        && close(coarse.qzz, fine.qzz, 5)
        && close(coarse.qxy, fine.qxy, 5)
        && close(coarse.qxz, fine.qxz, 5)
        && close(coarse.qyz, fine.qyz, 5)
}

/// One boundary edge's curve and end points.
pub(super) struct EdgeSpan<'a> {
    pub(super) curve: &'a EdgeCurve,
    pub(super) start: Point3,
    pub(super) end: Point3,
}

impl EdgeSpan<'_> {
    /// How far the edge's points at `lo` and `hi` lie from `reference`.
    pub(super) fn end_reach(&self, lo: f64, hi: f64, reference: Point3) -> [f64; 2] {
        [lo, hi].map(|t| {
            (self.curve.evaluate_with_endpoints(t, self.start, self.end) - reference).length()
        })
    }
}

/// What every edge of one face is read against.
struct Reading<'a> {
    cylinder: &'a CylindricalSurface,
    axis: Vec3,
    reference: Point3,
    off_surface_limit: f64,
}

impl Reading<'_> {
    /// Add `−∮ F du` along `span` over `intervals` (run in `direction`), one
    /// Gauss–Legendre rule per interval, and what its points read of the edge
    /// to `measure`. `false` when the edge is not a curve this path reads, or
    /// a fitted point lies off the surface.
    fn edge<const AREA_ONLY: bool>(
        &self,
        span: &EdgeSpan<'_>,
        direction: f64,
        intervals: &[(f64, f64)],
        acc: &mut Accumulator,
        measure: &mut EdgeMeasure,
    ) -> bool {
        let mut scratch = DerivativeScratch::new();
        let edge_points = gauss_legendre_points(EDGE_ORDER);
        let axial_points = gauss_legendre_points(AXIAL_ORDER);
        let cylinder = self.cylinder;
        for &(p0, p1) in intervals {
            let half = (p1 - p0) / 2.0;
            let mid = p0 + half;
            for gp in edge_points {
                let t = half.mul_add(gp.x, mid);
                let point = span.curve.evaluate_with_endpoints(t, span.start, span.end);
                let Some(dp_dt) = velocity(span.curve, t, span.start, span.end) else {
                    return false;
                };
                // The angular rate of the projected point: ∂P/∂u has the
                // radius's length, so a point at radial distance ρ moves in u
                // at r/ρ times `dp·∂P/∂u / |∂P/∂u|²`. Exact trims lie on the
                // surface (ρ = r) and keep the plain quotient.
                let (rate_scale, off) = if matches!(span.curve, EdgeCurve::NurbsCurve(_)) {
                    let offset = point - cylinder.origin();
                    let rho = (offset - self.axis * offset.dot(self.axis)).length();
                    let off = (rho - cylinder.radius()).abs();
                    if off > self.off_surface_limit {
                        return false;
                    }
                    (cylinder.radius() / rho, off)
                } else {
                    (1.0, 0.0)
                };
                let (u, v) = cylinder.project_point(point);
                let (_, du_vec, dv_vec, _, _) =
                    cylinder.span_hinted_point_and_partials_with_scratch(u, v, &mut scratch);
                let du_dt = rate_scale * dp_dt.dot(du_vec) / du_vec.dot(du_vec);
                // The radial projection keeps the axial rate.
                let dv_dt = dp_dt.dot(dv_vec) / dv_vec.dot(dv_vec);
                measure.add(
                    gp.w * half,
                    off,
                    dp_dt.length(),
                    (du_vec * du_dt + dv_vec * dv_dt).length(),
                );
                // −∮ F du, F(u, v) = v ∫₀¹ f(u, v s) ds.
                let line_weight = -(direction * gp.w * half) * du_dt * v;
                for sp in axial_points {
                    let s = 0.5 * (sp.x + 1.0);
                    acc.add::<_, AREA_ONLY>(
                        cylinder,
                        u,
                        v * s,
                        line_weight * 0.5 * sp.w,
                        self.reference,
                        &mut scratch,
                    );
                }
            }
        }
        true
    }
}
