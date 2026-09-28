//! Affine transforms applied to topological shapes.

use std::collections::HashSet;

use remus_math::context::DEFAULT_MAX_ENTITY_TOLERANCE;
use remus_math::mat::Mat4;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Vec3;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::transaction::run_transacted;
use remus_topology::vertex::VertexId;
use remus_topology::wire::WireId;

/// Dimensionless floor below which a transform is treated as collapsing the
/// model rather than moving it.
///
/// The quantity compared against it is the linear part's **Hadamard ratio**,
/// `|det| / (‖c₀‖·‖c₁‖·‖c₂‖)` over the three columns — equivalently the
/// determinant of the same matrix with every column normalized to unit
/// length. It is the volume of the unit cube's image divided by the volume
/// those columns would span if they were mutually orthogonal, so it is `1`
/// for any similarity (rotation, uniform scale, reflection), `0` for a matrix
/// that flattens space onto a plane, line or point, and — being a quotient of
/// two volumes — dimensionless. It is therefore identical for a model in
/// metres and the same model in nanometres, which is the whole point: a
/// transform is degenerate because of its *shape*, never because of its size.
///
/// `1e-12` sits far below any transform a user means (every non-degenerate
/// matrix in the suite below, uniform or not, measures between 0.14 and 1.0)
/// and far above the `f64::EPSILON`-relative algebraic test inside
/// `Mat4::inverse`, which remains the backstop. This guard exists to name the
/// failure before the inverse-transpose normal update starts amplifying
/// round-off.
const DEGENERATE_SHAPE_RATIO: f64 = 1e-12;

/// Half-width of the ulp-scale band around `[0, 0, 0, 1]` inside which a
/// bottom row still counts as affine.
///
/// This is deliberately a tolerance and not an exact comparison. `Mat4::inverse`
/// is an adjugate inversion, so inverting a perfectly rigid frame legitimately
/// returns a bottom row like `[0, 0, -0.0, 1.0000000000000002]`: the `w` entry
/// is a sum of 2×2 minors divided by the determinant, and that division does
/// not have to land on exactly `1.0`. Rejecting those rows would refuse every
/// caller that feeds a `Mat4::inverse()` result back into a transform.
///
/// A genuinely projective row is nowhere near this band — perspective entries
/// are on the order of the reciprocal of the model's size, some twelve or more
/// orders of magnitude above `8·f64::EPSILON`.
const AFFINE_ROW_WOBBLE: f64 = 8.0 * f64::EPSILON;

/// Relative error budget for recognizing an analytic image as exact.
///
/// The tests below only forgive the rounding accumulated by a handful of
/// `f64` products, sums, square roots, and normalizations.  A looser geometric
/// tolerance is unsafe here: its absolute error grows with the curve radius,
/// so a coefficient that looks "close" at unit scale can move a point a large
/// distance on a large model.  Sixteen ulps covers the arithmetic above while
/// keeping any accepted discrepancy at the same scale as evaluating the
/// transformed coordinates themselves.
const ANALYTIC_ROUNDOFF_REL: f64 = 16.0 * f64::EPSILON;

fn equal_within_analytic_roundoff(a: f64, b: f64) -> bool {
    let scale = a.abs().max(b.abs());
    scale.is_finite() && scale > 0.0 && (a - b).abs() <= scale * ANALYTIC_ROUNDOFF_REL
}

// ── B74 quality-aware contract ─────────────────────────────────────────────
// Entry-point inventory (native):
//   - `transform_solid` — in-place solid transform (legacy, fitting default).
//   - `transform_solid_detailed` — additive twin: same engine, typed report,
//     caller-chosen [`TransformPolicy`], transacted.
//   - `transform_wire`, `transform_face` — in-place wire/face subsets.
//   - `copy::copy_and_transform_solid` — copy + transform in one pass.
//   - `copy::copy_and_transform_solid_detailed` — additive twin with report.
//   - `mirror::mirror` — copy + reflection (a similarity, always exact).
//   - `pattern::*` — rigid placements only, unaffected by anisotropy.
// WASM/batch routes live in `remus-wasm`: `transformSolid`, `transformWire`,
// `transformFace`, `copyAndTransformSolid` (direct + `executeBatch`/`V2`),
// plus the additive `transformDetailed` / `copyAndTransformSolidDetailed`
// twins. `composeTransforms` is pure matrix math with no topology.
//
// Supported domain: finite affine matrices whose linear part has a
// dimensionless Hadamard ratio above [`DEGENERATE_SHAPE_RATIO`] (accepts a
// uniform scale of any magnitude in either direction; refuses near-collapsed
// matrices) and that [`Mat4::inverse`] still inverts. Similarity maps
// (conformal linear part within [`ANALYTIC_ROUNDOFF_REL`]) preserve every
// analytic carrier; anisotropic maps convert carriers as the report records.
// Volume scales by `|det(linear)|`; orientation is witnessed separately
// (classification + normal adherence), never inferred from the volume sign.
//
// Legacy compatibility limits (unchanged by this contract): the legacy
// entry points keep their signatures, their fitting default for sphere
// patches outside the exact class, and their void/copy returns. They gain
// transacted atomicity and share the engine fixes below, but they do not
// disclose quality — use the detailed twins for that. A detailed `Exact`
// report never covers fitted output; fitted output requires
// [`TransformPolicy::AllowApproximate`] and carries sampled evidence that
// is explicitly not a certified bound.

/// Caller-chosen fallback policy for the quality-aware transform entry
/// points.
///
/// This is the transform-local form of the operation contract's fallback
/// policy. There is no error budget: the sampled sphere fit this policy
/// unlocks has no computable bound, so the report carries measured sampled
/// evidence instead (see [`FittedFaceEvidence`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransformPolicy {
    /// Refuse with [`crate::OperationsError::ExactOnlyUnattainable`] before
    /// touching topology when a face would need sampled fitting. Never
    /// publishes fitted output.
    ExactOnly,
    /// Perform the sampled sphere fit where the exact class does not reach,
    /// and disclose it per face with sampled evidence.
    AllowApproximate,
}

/// Quality of a committed transform result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransformQuality {
    /// Every face is the exact affine image of its source: analytic
    /// carriers preserved under similarity, exact rational NURBS
    /// conversions otherwise. No representation was degraded.
    Exact,
    /// At least one face is a sampled interpolation (see
    /// [`TransformReport::fitted_faces`]). Permitted only under
    /// [`TransformPolicy::AllowApproximate`].
    Approximate,
}

/// How one face's surface survived the transform.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceMethod {
    /// The carrier is unchanged (planes under any affine map, NURBS control
    /// nets under any affine map, every analytic carrier under similarity).
    AnalyticPreserved,
    /// The carrier changed family through an exact rational construction:
    /// cylinder/cone/torus to NURBS, or an exact-class sphere patch (full
    /// sphere, hemisphere) to NURBS. The affine image of a rational NURBS is exact as a point set; 3D edge trims remap exactly
    /// with it. The surface parameterization is NOT preserved — converters
    /// use their own knot frames (fractional-circle u for cylinders/cones,
    /// rational-vs-trigonometric circle maps for spheres) — so surface-UV
    /// traces (stored pcurves) do not survive conversion. Downstream
    /// consumers that trim from 3D wires are unaffected; UV-trace consumers
    /// must re-derive. Meshing and classification fall back to projection
    /// through a tolerance gate wherever a stale trace misses.
    ExactRational,
    /// The carrier changed family through sampled interpolation (sphere
    /// patches outside the exact class: 33 × 17 grid, cubic, non-rational).
    /// Approximate: see [`FittedFaceEvidence`].
    SampledFit,
}

impl SurfaceMethod {
    /// Stable wire string for reports and WASM details.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::AnalyticPreserved => "analyticPreserved",
            Self::ExactRational => "exactRational",
            Self::SampledFit => "sampledFit",
        }
    }
}

/// One face whose surface carrier changed family.
#[derive(Debug, Clone, Copy)]
pub struct FaceCarrierChange {
    /// Face as found after the transform (in-place entry points keep the id).
    pub face: FaceId,
    /// Source carrier (`FaceSurface::type_tag`, e.g. `"cylinder"`).
    pub from: &'static str,
    /// Result carrier (e.g. `"nurbs"`).
    pub to: &'static str,
    /// How the conversion was built.
    pub method: SurfaceMethod,
    /// Control points of the result surface when it is NURBS, else `0`.
    /// Deterministic output-size measure for conversion cost tracking.
    pub control_points: usize,
}

/// One edge whose curve carrier changed family (e.g. circle to ellipse).
#[derive(Debug, Clone, Copy)]
pub struct EdgeCarrierChange {
    /// Edge as found after the transform.
    pub edge: EdgeId,
    /// Source carrier (`EdgeCurve::type_tag`, e.g. `"circle"`).
    pub from: &'static str,
    /// Result carrier (e.g. `"ellipse"`).
    pub to: &'static str,
}

/// Sampled evidence for one fitted face.
///
/// Measured as a symmetric sampled Hausdorff estimate between uniform
/// samples of the fitted NURBS domain and a fine analytic grid of the true
/// affine image: every sample cloud's farthest point from the other cloud.
/// This is actual measured evidence, not a certified bound — the true
/// maximum deviation between the sample clouds is unobserved.
#[derive(Debug, Clone, Copy)]
pub struct FittedFaceEvidence {
    /// Face as found after the transform.
    pub face: FaceId,
    /// Source carrier (always `"sphere"` on the current engine).
    pub from: &'static str,
    /// Construction method; currently always `"interpolate33x17"`.
    pub method: &'static str,
    /// Fit grid resolution `(u, v)` the interpolation ran at.
    pub grid: (usize, usize),
    /// Measured sampled deviation in model units.
    pub max_residual: f64,
    /// Total check points compared (`nurbs_samples + analytic_samples`).
    pub check_points: usize,
}

/// Typed quality-aware result of a committed transform.
#[derive(Debug, Clone)]
pub struct TransformReport {
    /// Exact when no face was fitted; approximate otherwise.
    pub quality: TransformQuality,
    /// Signed determinant of the linear part. Volume scales by its absolute
    /// value; its sign records orientation reversal.
    pub determinant: f64,
    /// True when the linear part reverses orientation (`determinant < 0`).
    /// NURBS carriers (converted or pre-existing) are re-parameterized
    /// (u-reversed) to compensate with flags and coedges untouched;
    /// analytic and plane carriers are rebuilt or re-derived and need no
    /// compensation. Informational: committed results are outward-oriented
    /// either way.
    pub orientation_reversed: bool,
    /// True when the linear part is conformal (similarity): every analytic
    /// carrier is then preserved and no conversion runs.
    pub similarity: bool,
    /// Faces whose carrier changed family, in face-id order.
    pub face_changes: Vec<FaceCarrierChange>,
    /// Edges whose carrier changed family, in edge-id order.
    pub edge_changes: Vec<EdgeCarrierChange>,
    /// Sampled-fit evidence, in face-id order. Non-empty implies
    /// `quality == Approximate`.
    pub fitted_faces: Vec<FittedFaceEvidence>,
    /// Total NURBS control points minted by conversions and fits.
    pub control_points_total: usize,
}

impl TransformReport {
    /// True when no representation was degraded.
    #[must_use]
    pub const fn is_exact(&self) -> bool {
        matches!(self.quality, TransformQuality::Exact)
    }
}

/// Determinant of the linear (upper-left 3 × 3) part of an affine matrix.
///
/// Positive-volume solids scale by its absolute value; its sign decides the
/// NURBS orientation-flag carry. Computed directly rather than through
/// [`Mat4::determinant`] so the contract names the linear part explicitly.
#[must_use]
pub fn linear_determinant(matrix: &Mat4) -> f64 {
    let m = &matrix.0;
    let cofactor_00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    let cofactor_01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    let cofactor_02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    m[0][0] * cofactor_00 + m[0][1] * cofactor_01 + m[0][2] * cofactor_02
}

/// Angular gate (radians) for constant-latitude recognition of a sphere
/// patch boundary. `project_point` returns angles, so this is independent
/// of model scale; primitive latitude rims evaluate within float noise
/// while boolean trims wander orders of magnitude wider.
const SPHERE_LATITUDE_GATE: f64 = 1e-9;

/// Fit grid (u × v) for sampled sphere patches. Values are the historical
/// ones; changing them would change legacy geometry.
const SPHERE_FIT_GRID_U: usize = 33;
const SPHERE_FIT_GRID_V: usize = 17;

/// Evidence grids for [`fitted_patch_evidence`]: uniform samples of the
/// fitted NURBS domain versus a fine analytic grid of the true image.
const FIT_EVIDENCE_NURBS_U: usize = 25;
const FIT_EVIDENCE_NURBS_V: usize = 13;
const FIT_EVIDENCE_ANALYTIC_U: usize = 97;
const FIT_EVIDENCE_ANALYTIC_V: usize = 49;

/// Exact-class membership of one sphere face under an anisotropic map.
///
/// Decided read-only from boundary latitudes (vertices plus interior
/// samples of every non-chord edge, mapped back through `inverse`).
/// Faces outside the exact class keep the historical sampled fit (or
/// refuse under [`TransformPolicy::ExactOnly`]).
///
/// Two historical misroutings are closed here, not extended: latitude
/// bands used to pass the mean-latitude test and convert as hemispheres
/// (wrong pole-reaching patch); they now fail the spread gate and land in
/// `General`. Off-equator single-latitude loops (polar caps) are
/// deliberately NOT converted exactly even though the split is
/// surface-perfect (4e-16 on-sphere): the inserted-knot piece will not
/// mesh its trim — raw split pieces without any transform come out open
/// while natural-knot hemisphere splits mesh watertight — so caps keep
/// the fit / exact-only refusal until tessellation handles clamped
/// interior knots. See the B74 trail for the isolation probes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SpherePatch {
    /// No boundary edges: the whole sphere converts exactly.
    Full,
    /// Equatorial loop bounding the northern hemisphere.
    NorthHemisphere,
    /// Equatorial loop bounding the southern hemisphere.
    SouthHemisphere,
    /// Anything else (latitude bands, polar caps, general trims,
    /// ambiguous loops): sampled fit, never relabeled exact.
    General,
}

/// Per-face transform plan, computed read-only by preflight.
///
/// The WASM twin reads these to name refused faces on the exact-only path.
#[derive(Debug, Clone, Copy)]
pub struct FacePlan {
    /// Face being planned.
    pub face: FaceId,
    /// Source carrier tag.
    pub from: &'static str,
    /// How the face will be built.
    pub method: SurfaceMethod,
    /// Sphere patch class when `from == "sphere"` and the map is
    /// anisotropic; `None` otherwise.
    pub patch: Option<SpherePatch>,
}

/// Accumulates carrier changes, fit evidence, and output sizes.
///
/// Threaded through the shared engine so the legacy entry points, the
/// detailed twins, and the copy path record identically.
#[derive(Debug, Default)]
pub struct TransformRecorder {
    /// Whether the linear part reverses orientation: NURBS results are
    /// u-reversed to compensate (see [`orient_nurbs_for_map`]).
    orientation_reversing: bool,
    /// Whether the sampled sphere fit may run. `false` turns a `General`
    /// sphere patch into an exact-only refusal instead of a fit.
    allow_fit: bool,
    /// Faces whose carrier changed family.
    face_changes: Vec<FaceCarrierChange>,
    /// Edges whose carrier changed family.
    edge_changes: Vec<EdgeCarrierChange>,
    /// Sampled-fit evidence.
    fitted: Vec<FittedFaceEvidence>,
    /// Total NURBS control points minted.
    control_points_total: usize,
}

impl TransformRecorder {
    /// Recorder that discards nothing but reports to nobody: the legacy
    /// path's engine runs through it so fixes stay shared.
    pub(crate) fn new(orientation_reversing: bool, allow_fit: bool) -> Self {
        Self {
            orientation_reversing,
            allow_fit,
            face_changes: Vec::new(),
            edge_changes: Vec::new(),
            fitted: Vec::new(),
            control_points_total: 0,
        }
    }

    pub(crate) fn record_face(
        &mut self,
        face: FaceId,
        from: &'static str,
        to: &'static str,
        method: SurfaceMethod,
        control_points: usize,
    ) {
        if from != to {
            self.face_changes.push(FaceCarrierChange {
                face,
                from,
                to,
                method,
                control_points,
            });
        }
        self.control_points_total += control_points;
    }

    pub(crate) fn record_edge(&mut self, edge: EdgeId, from: &'static str, to: &'static str) {
        if from != to {
            self.edge_changes.push(EdgeCarrierChange { edge, from, to });
        }
    }

    pub(crate) fn record_fit(&mut self, evidence: FittedFaceEvidence) {
        self.fitted.push(evidence);
    }

    /// Assemble the report, sorting change lists by entity index so repeated
    /// runs agree regardless of arena iteration order.
    pub(crate) fn into_report(mut self, determinant: f64, similarity: bool) -> TransformReport {
        self.face_changes.sort_by_key(|change| change.face.index());
        self.edge_changes.sort_by_key(|change| change.edge.index());
        self.fitted.sort_by_key(|fit| fit.face.index());
        let quality = if self.fitted.is_empty() {
            TransformQuality::Exact
        } else {
            TransformQuality::Approximate
        };
        TransformReport {
            quality,
            determinant,
            orientation_reversed: self.orientation_reversing,
            similarity,
            face_changes: self.face_changes,
            edge_changes: self.edge_changes,
            fitted_faces: self.fitted,
            control_points_total: self.control_points_total,
        }
    }
}

/// Latitude of one boundary sample on the still-untransformed sphere.
fn sample_latitude(
    sph: &remus_math::surfaces::SphericalSurface,
    inverse: &Mat4,
    point: remus_math::vec::Point3,
) -> f64 {
    sph.project_point(inverse.mul_point(point)).1
}

/// Interior latitude samples of one boundary edge.
///
/// Returns an empty vector for chord (`Line`) edges — a chord midpoint lies
/// inside the sphere and its projection carries chord sag, not trim
/// evidence — and whenever the edge has no usable domain. Up to three
/// interior stations plus both endpoints for smooth trims.
fn edge_latitude_samples(
    topo: &Topology,
    edge: &remus_topology::edge::Edge,
    sph: &remus_math::surfaces::SphericalSurface,
    inverse: &Mat4,
) -> Vec<f64> {
    if matches!(edge.curve(), EdgeCurve::Line) {
        return Vec::new();
    }
    let Ok((a, b)) = edge.strict_domain() else {
        return Vec::new();
    };
    if a >= b || !a.is_finite() || !b.is_finite() {
        return Vec::new();
    }
    let (Ok(start), Ok(end)) = (
        topo.vertex(edge.start())
            .map(remus_topology::vertex::Vertex::point),
        topo.vertex(edge.end())
            .map(remus_topology::vertex::Vertex::point),
    ) else {
        return Vec::new();
    };
    let mut latitudes = Vec::with_capacity(3);
    for t in [
        f64::midpoint(a, b),
        f64::midpoint(a, f64::midpoint(a, b)),
        f64::midpoint(f64::midpoint(a, b), b),
    ] {
        let point = edge.curve().evaluate_with_endpoints(t, start, end);
        if point.x().is_finite() && point.y().is_finite() && point.z().is_finite() {
            latitudes.push(sample_latitude(sph, inverse, point));
        }
    }
    latitudes
}

/// Classify one sphere face's patch from its boundary.
///
/// See [`SpherePatch`]. The hemisphere branch keeps the historical
/// pole-vertex / equatorial-winding disambiguation; every other branch is
/// new. Latitude bands, which the historical code mistook for hemispheres
/// whenever their mean latitude sat near a pole reach, now land in
/// `General`: their boundary latitudes fail the spread gate.
fn classify_sphere_patch(
    topo: &Topology,
    face_id: FaceId,
    sph: &remus_math::surfaces::SphericalSurface,
    // The vertex phase may already have moved boundary points; this maps
    // them back into the still-untransformed sphere's frame. Pass identity
    // when vertices are untouched (preflight).
    inverse: &Mat4,
) -> Result<SpherePatch, crate::OperationsError> {
    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;
    if wire.edges().is_empty() {
        return Ok(SpherePatch::Full);
    }
    let mut latitudes = Vec::new();
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let point = topo.vertex(edge.start())?.point();
        latitudes.push(sample_latitude(sph, inverse, point));
        latitudes.extend(edge_latitude_samples(topo, edge, sph, inverse));
    }
    if latitudes.is_empty() {
        return Ok(SpherePatch::Full);
    }

    let (mut lat_min, mut lat_max) = (f64::INFINITY, f64::NEG_INFINITY);
    for latitude in &latitudes {
        lat_min = lat_min.min(*latitude);
        lat_max = lat_max.max(*latitude);
    }
    if lat_max - lat_min > SPHERE_LATITUDE_GATE {
        return Ok(SpherePatch::General);
    }
    let mean = f64::midpoint(lat_min, lat_max);
    if mean.abs() <= SPHERE_LATITUDE_GATE {
        return classify_equatorial_sphere_patch(topo, face_id, sph, inverse, mean);
    }

    // Off-equator loops — polar caps, bands, general trims — are all
    // `General`. (Caps have a surface-perfect exact split; it does not
    // ship because the inserted-knot piece will not mesh. See the enum
    // docs.)
    Ok(SpherePatch::General)
}

/// Degenerate pole point-edges on a face's outer wire.
///
/// Reports `(north, south)`: whether a closed edge (`start == end`) sits at
/// the actual north / south pole. A full equatorial rim is also closed;
/// its vertex must not decide the patch side.
/// the flag decides the patch side wherever the historical equatorial
/// rule already trusts it.
fn pole_point_edges(
    topo: &Topology,
    face_id: FaceId,
    sph: &remus_math::surfaces::SphericalSurface,
    inverse: &Mat4,
) -> Result<(bool, bool), crate::OperationsError> {
    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;
    let north_pole = sph.center() + sph.z_axis() * sph.radius();
    let south_pole = sph.center() - sph.z_axis() * sph.radius();
    let pole_tol = remus_math::tolerance::Tolerance::new()
        .linear
        .min(sph.radius() * 0.5);
    let (mut north, mut south) = (false, false);
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        if edge.start() == edge.end() {
            let point = inverse.mul_point(topo.vertex(edge.start())?.point());
            if (point - north_pole).length() < pole_tol {
                north = true;
            } else if (point - south_pole).length() < pole_tol {
                south = true;
            }
        }
    }
    Ok((north, south))
}

/// Sign of the outer wire's directed area in the sphere's (x, y) frame.
///
/// `Some(true)` runs counter-clockwise (north-side region), `Some(false)`
/// clockwise (south-side region), `None` when the loop is degenerate
/// (signed area within float-noise tolerance of zero). The hemisphere
/// branch decides on this sign; tiny polar circles whose area cannot
/// clear the tolerance refuse rather than guess.
fn equatorial_winding_sign(
    topo: &Topology,
    face_id: FaceId,
    sph: &remus_math::surfaces::SphericalSurface,
    inverse: &Mat4,
) -> Result<Option<bool>, crate::OperationsError> {
    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;
    let center = sph.center();
    let mut signed_area_twice = 0.0;
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let edge_start = topo.vertex(edge.start())?.point();
        let edge_end = topo.vertex(edge.end())?.point();
        let (t0, t1) = edge.domain_with_endpoints(edge_start, edge_end);
        let (t0, t1) = if oe.is_forward() { (t0, t1) } else { (t1, t0) };
        let samples = if matches!(edge.curve(), EdgeCurve::Line) {
            1
        } else {
            16
        };
        let mut prev = inverse.mul_point(
            edge.curve()
                .evaluate_with_endpoints(t0, edge_start, edge_end),
        );
        for k in 1..=samples {
            #[allow(clippy::cast_precision_loss)]
            let t = t0 + (t1 - t0) * k as f64 / samples as f64;
            let next = inverse.mul_point(
                edge.curve()
                    .evaluate_with_endpoints(t, edge_start, edge_end),
            );
            let a = prev - center;
            let b = next - center;
            signed_area_twice += sph.x_axis().dot(a) * sph.y_axis().dot(b)
                - sph.x_axis().dot(b) * sph.y_axis().dot(a);
            prev = next;
        }
    }
    let winding_tol = remus_math::tolerance::Tolerance::new().linear * sph.radius().powi(2);
    if signed_area_twice > winding_tol {
        Ok(Some(true))
    } else if signed_area_twice < -winding_tol {
        Ok(Some(false))
    } else {
        Ok(None)
    }
}

/// Disambiguate an equatorial sphere loop into hemispheres.
///
/// Historical pole-vertex / signed-area rule, unchanged: at an equatorial
/// trim, position alone cannot distinguish the north and south patches, so
/// a degenerate pole point-edge decides when present and the outer wire's
/// directed area in the sphere's own (x, y) frame decides otherwise (north
/// runs CCW, south runs CW). A single equatorial loop bounds exactly one
/// of the two hemispheres, so this branch is sound without a spread
/// escape.
fn classify_equatorial_sphere_patch(
    topo: &Topology,
    face_id: FaceId,
    sph: &remus_math::surfaces::SphericalSurface,
    inverse: &Mat4,
    boundary_v: f64,
) -> Result<SpherePatch, crate::OperationsError> {
    use std::f64::consts::FRAC_PI_2;

    let (has_pole_north, has_pole_south) = pole_point_edges(topo, face_id, sph, inverse)?;
    if has_pole_north {
        return Ok(SpherePatch::NorthHemisphere);
    }
    if has_pole_south {
        return Ok(SpherePatch::SouthHemisphere);
    }

    match equatorial_winding_sign(topo, face_id, sph, inverse)? {
        Some(true) => Ok(SpherePatch::NorthHemisphere),
        Some(false) => Ok(SpherePatch::SouthHemisphere),
        None => {
            let _ = (boundary_v, FRAC_PI_2);
            Err(crate::OperationsError::InvalidInput {
                reason: "cannot determine sphere patch across a degenerate equatorial boundary"
                    .to_string(),
            })
        }
    }
}

/// Read-only preflight plan for one face: how the engine will build it.
pub(crate) fn plan_face_surface(
    topo: &Topology,
    fid: FaceId,
    similarity: bool,
    inverse: &Mat4,
) -> Result<FacePlan, crate::OperationsError> {
    let face = topo.face(fid)?;
    let from = face.surface().type_tag();
    let (method, patch) = match face.surface() {
        FaceSurface::Plane { .. } | FaceSurface::Nurbs(_) => {
            (SurfaceMethod::AnalyticPreserved, None)
        }
        FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Torus(_)
        | FaceSurface::Sphere(_) => {
            if similarity {
                (SurfaceMethod::AnalyticPreserved, None)
            } else if let FaceSurface::Sphere(sph) = face.surface() {
                let sph_clone = sph.clone();
                match classify_sphere_patch(topo, fid, &sph_clone, inverse)? {
                    SpherePatch::General => (SurfaceMethod::SampledFit, Some(SpherePatch::General)),
                    patch => (SurfaceMethod::ExactRational, Some(patch)),
                }
            } else {
                (SurfaceMethod::ExactRational, None)
            }
        }
    };
    Ok(FacePlan {
        face: fid,
        from,
        method,
        patch,
    })
}

/// Refuse an exact-only request whose preflight needs sampled fitting.
///
/// Runs before any mutation, so the refusal is atomic by construction: no
/// topology changes, no handle churn. The error is the stable
/// [`crate::OperationsError::ExactOnlyUnattainable`] variant (WASM category
/// `quality_refused`, code `exact_only_unattainable`); the WASM twin
/// additionally names the fitted faces from the plans it already holds.
///
/// # Errors
///
/// Returns [`crate::OperationsError::ExactOnlyUnattainable`] when any plan
/// needs sampled fitting.
pub fn refuse_unless_exact(plans: &[FacePlan]) -> Result<(), crate::OperationsError> {
    if plans
        .iter()
        .any(|plan| matches!(plan.method, SurfaceMethod::SampledFit))
    {
        return Err(crate::OperationsError::ExactOnlyUnattainable);
    }
    Ok(())
}

/// Read-only preflight for one solid: face plans plus the pure edge check.
///
/// Re-runs the pure edge-curve transform on cloned curves, so structural
/// refusals (skewed circle/ellipse images, non-similarity open conics)
/// surface before any mutation. Faces are planned by
/// [`plan_face_surface`]; pass identity as `inverse` — vertices are
/// untouched at preflight time.
///
/// The WASM twin calls this directly to name refused faces.
///
/// # Errors
///
/// Returns an error when a referenced entity is missing or an edge image
/// is structurally unrepresentable (the same refusal the mutating path
/// would raise, before any mutation).
pub fn preflight_solid_transform(
    topo: &Topology,
    solid: SolidId,
    matrix: &Mat4,
) -> Result<Vec<FacePlan>, crate::OperationsError> {
    let (_, edge_ids, face_ids) = collect_solid_entities(topo, solid)?;
    for eid in &edge_ids {
        let edge = topo.edge(*eid)?;
        // Pure: clone, transform, discard. Propagates the refusal unchanged.
        let _ = transform_edge_curve_with_trim(edge.curve(), edge.trim(), matrix)?;
    }
    let similarity = is_uniform_scale(matrix);
    let identity = Mat4::identity();
    let mut plans: Vec<FacePlan> = face_ids
        .iter()
        .map(|fid| plan_face_surface(topo, *fid, similarity, &identity))
        .collect::<Result<_, _>>()?;
    plans.sort_by_key(|plan| plan.face.index());
    Ok(plans)
}

/// Reject a transform that collapses the model; accept every one that does not.
///
/// Validates the affine bottom row before testing the 3×3 linear part.
/// `Mat4::mul_point` ignores the bottom row, so accepting a projective matrix
/// here would apply a different transform from the one the caller supplied.
///
/// # Errors
///
/// Returns [`crate::OperationsError::InvalidInput`] when any entry is
/// non-finite, the matrix is not affine, a linear column is zero, or the
/// Hadamard ratio is at or below [`DEGENERATE_SHAPE_RATIO`].
pub(crate) fn reject_degenerate_transform(matrix: &Mat4) -> Result<(), crate::OperationsError> {
    let degenerate = |reason: &str| crate::OperationsError::InvalidInput {
        reason: format!("transform matrix is degenerate ({reason})"),
    };

    let m = &matrix.0;
    // Checked over the whole matrix, and before the band below: a NaN fails
    // every `>` comparison, so an unchecked NaN would slip through as affine.
    if m.iter().flatten().any(|value| !value.is_finite()) {
        return Err(degenerate("an entry is not finite"));
    }
    if m[3][0].abs() > AFFINE_ROW_WOBBLE
        || m[3][1].abs() > AFFINE_ROW_WOBBLE
        || m[3][2].abs() > AFFINE_ROW_WOBBLE
        || (m[3][3] - 1.0).abs() > AFFINE_ROW_WOBBLE
    {
        return Err(degenerate("the bottom row is not affine"));
    }
    // Normalize each column before taking the determinant, rather than
    // dividing the determinant by the product of the norms afterwards: the
    // product of three norms can overflow or underflow to 0/∞ for extreme
    // (but perfectly valid) matrices, and the quotient would then be NaN.
    let mut unit = [Vec3::new(0.0, 0.0, 0.0); 3];
    for (j, slot) in unit.iter_mut().enumerate() {
        let col = Vec3::new(m[0][j], m[1][j], m[2][j]);
        let Ok(n) = col.normalize() else {
            return Err(degenerate("a linear column is zero or non-finite"));
        };
        *slot = n;
    }

    // Scalar triple product of the unit columns = the Hadamard ratio, in
    // [0, 1] by construction. Reflections give a negative triple product and
    // a ratio of 1 — they are proper transforms and must keep passing.
    let ratio = unit[0].dot(unit[1].cross(unit[2])).abs();
    if ratio <= DEGENERATE_SHAPE_RATIO {
        return Err(degenerate(
            "it collapses the model onto a plane, line or point",
        ));
    }
    Ok(())
}

/// Apply an affine transform to a solid, modifying vertex positions and
/// face surface geometry in place.
///
/// The transform matrix must be non-degenerate — see
/// `reject_degenerate_transform`, which tests the matrix's *shape* and so
/// accepts a uniform scale of any size, in either direction.
/// All unique vertices reachable from the solid's shells are transformed,
/// NURBS edge curves and face surfaces have their control points updated,
/// and all planar face normals are updated using the inverse transpose.
///
/// Runs transacted: a mid-transform refusal (skewed circle image,
/// unsupported open conic, failed NURBS split) rolls the topology back to
/// its pre-call state instead of stranding a half-moved solid. Successful
/// geometry is identical to the untransacted engine.
///
/// This is the legacy entry point: anisotropic maps convert carriers
/// (cylinders, cones, tori and exact-class sphere patches to rational
/// NURBS; other sphere patches to a sampled fit) without disclosure. Use
/// [`transform_solid_detailed`] for the typed quality report.
///
/// # Errors
///
/// Returns an error if the matrix is degenerate or a referenced entity is missing.
pub fn transform_solid(
    topo: &mut Topology,
    solid: SolidId,
    matrix: &Mat4,
) -> Result<(), crate::OperationsError> {
    reject_degenerate_transform(matrix)?;
    // Validate every part of the matrix before changing live topology.
    let _ = matrix.inverse()?.transpose();
    let reversing = linear_determinant(matrix) < 0.0;
    let mut recorder = TransformRecorder::new(reversing, true);
    run_transacted(topo, |live| {
        execute_solid_transform(live, solid, matrix, &mut recorder)
    })
}

/// Quality-aware additive twin of [`transform_solid`].
///
/// Runs the same engine and commits the same geometry, additionally
/// returning a [`TransformReport`] that names every carrier-family change
/// and discloses sampled-fit output with its measured evidence. Under
/// [`TransformPolicy::ExactOnly`] a face outside the exact sphere class
/// refuses with [`crate::OperationsError::ExactOnlyUnattainable`] before
/// any mutation — an exact report never covers fitted output.
///
/// Like the legacy entry point this runs transacted: every failure rolls
/// back without exposing partial topology.
///
/// # Errors
///
/// Returns an error if the matrix is degenerate, a referenced entity is
/// missing, an edge or surface image is unrepresentable, or the exact-only
/// policy declines a sampled fit.
pub fn transform_solid_detailed(
    topo: &mut Topology,
    solid: SolidId,
    matrix: &Mat4,
    policy: TransformPolicy,
) -> Result<TransformReport, crate::OperationsError> {
    reject_degenerate_transform(matrix)?;
    // Validate every part of the matrix before changing live topology.
    let _ = matrix.inverse()?.transpose();
    if matches!(policy, TransformPolicy::ExactOnly) {
        let plans = preflight_solid_transform(topo, solid, matrix)?;
        refuse_unless_exact(&plans)?;
    }
    let determinant = linear_determinant(matrix);
    let reversing = determinant < 0.0;
    let similarity = is_uniform_scale(matrix);
    let mut recorder = TransformRecorder::new(reversing, true);
    run_transacted(topo, |live| {
        execute_solid_transform(live, solid, matrix, &mut recorder)
    })?;
    Ok(recorder.into_report(determinant, similarity))
}

/// Mutation phase shared by [`transform_solid`] and
/// [`transform_solid_detailed`]: vertices, then edges, then face surfaces.
fn execute_solid_transform(
    topo: &mut Topology,
    solid: SolidId,
    matrix: &Mat4,
    recorder: &mut TransformRecorder,
) -> Result<(), crate::OperationsError> {
    // Collect all unique vertex IDs, edge IDs, and face IDs in a read phase.
    let (vertex_ids, edge_ids, face_ids) = collect_solid_entities(topo, solid)?;
    let translation_certificates = translation_edge_certificates(topo, &edge_ids, matrix)?;

    // Mutate phase 1: transform each vertex.
    for vid in vertex_ids {
        let vertex = topo.vertex_mut(vid)?;
        let new_point = matrix.mul_point(vertex.point());
        vertex.set_point(new_point);
    }

    // Mutate phase 2: transform edge curves (NURBS, Circle, Ellipse).
    transform_edges_recorded(topo, &edge_ids, matrix, recorder)?;
    restore_translation_certificates(topo, translation_certificates)?;

    // Mutate phase 3: transform face surface geometry.
    // For plane normals, use the inverse transpose: n' = (M⁻¹)ᵀ · n
    let normal_matrix = matrix.inverse()?.transpose();
    for fid in face_ids {
        transform_face_surface_recorded(topo, fid, matrix, &normal_matrix, recorder)?;
    }

    Ok(())
}

// A translation preserves mathematical endpoint residuals, but independently
// rounding translated vertices and curve control points can increase the
// evaluated residual by a few ulps. Preserve an existing certificate only
// within a coordinate-scale floating-point budget. Invalid source edges and
// larger discrepancies are never repaired here; the strict I/O gate remains
// unchanged. The carried tolerance keeps the full coordinate-scale budget as
// headroom (not just the next ulp): re-evaluating the same edge on a
// different FP stack (wasm simd128 vs native, FMA vs non-FMA, different libm
// sin/cos) can shift the residual by a few ulps of the coordinates, which is
// many ulps of the residual itself. The budget is ~1e-8 of the tolerance, so
// this cannot mask a real geometric gap.
#[allow(clippy::float_cmp)] // Exact identity is required; near-identity may scale geometry.
pub(crate) fn translation_edge_certificates(
    topo: &Topology,
    edges: &HashSet<EdgeId>,
    matrix: &Mat4,
) -> Result<Vec<(EdgeId, f64, f64)>, crate::OperationsError> {
    let m = &matrix.0;
    if m[3] != [0.0, 0.0, 0.0, 1.0] || (m[0][3] == 0.0 && m[1][3] == 0.0 && m[2][3] == 0.0) {
        return Ok(Vec::new());
    }
    // Rigid motions (rotation/reflection + translation, no scale/shear)
    // preserve endpoint residuals mathematically; only independent f64
    // rounding of the mapped vertices and curve frames can move the measured
    // value, by a coordinate-scale few ulps. Uniform scales also qualify:
    // both the residual and the tolerance-carried gap scale together, and the
    // budget below scales with the coordinates.
    if !is_rigid_or_uniform_scale(matrix) {
        return Ok(Vec::new());
    }
    let mut certificates = Vec::new();
    for &id in edges {
        let edge = topo.edge(id)?;
        if matches!(edge.curve(), EdgeCurve::Line) {
            continue;
        }
        let Ok((a, b)) = edge.strict_domain() else {
            continue;
        };
        let start = topo.vertex(edge.start())?;
        let end = topo.vertex(edge.end())?;
        let tolerance = edge.effective_tolerance(start.tolerance().max(end.tolerance()));
        let p = start.point();
        let q = end.point();
        let first = (edge.curve().evaluate_with_endpoints(a, p, q) - p).length();
        let second = (edge.curve().evaluate_with_endpoints(b, p, q) - q).length();
        if !first.is_finite() || !second.is_finite() {
            continue;
        }
        let residual = first.max(second);
        if residual.is_finite() && residual <= tolerance {
            let scale = [
                p.x(),
                p.y(),
                p.z(),
                q.x(),
                q.y(),
                q.z(),
                m[0][3],
                m[1][3],
                m[2][3],
            ]
            .into_iter()
            .map(f64::abs)
            .fold(0.0, f64::max);
            // Large translations must not turn a coordinate-scale budget
            // into permission for a meaningful geometric gap.
            let budget = (64.0 * f64::EPSILON * scale).min(tolerance * 1e-8);
            certificates.push((id, tolerance, budget));
        }
    }
    Ok(certificates)
}

pub(crate) fn restore_translation_certificates(
    topo: &mut Topology,
    certificates: Vec<(EdgeId, f64, f64)>,
) -> Result<(), crate::OperationsError> {
    for (id, tolerance, budget) in certificates {
        let edge = topo.edge(id)?;
        let Ok((a, b)) = edge.strict_domain() else {
            continue;
        };
        let p = topo.vertex(edge.start())?.point();
        let q = topo.vertex(edge.end())?.point();
        let first = (edge.curve().evaluate_with_endpoints(a, p, q) - p).length();
        let second = (edge.curve().evaluate_with_endpoints(b, p, q) - q).length();
        if !first.is_finite() || !second.is_finite() {
            continue;
        }
        let residual = first.max(second);
        if residual.is_finite() && residual > tolerance {
            // A rigid motion preserves endpoint residuals mathematically, so a
            // post-transform excess within the coordinate-scale roundoff
            // budget is the transform's own rounding, not new geometry: carry
            // the certificate forward. Larger gaps are left alone — the
            // strict I/O gate still refuses them. Store the measured residual
            // plus the full budget (rounded up): the gate re-evaluates the
            // edge, potentially on a different FP stack than the transform
            // ran on, and that re-evaluation can drift by a few
            // coordinate-ulps. A single residual-ulp of headroom strands the
            // edge on wasm simd128 builds (#483); the budget is ~1e-8 of the
            // tolerance, so the slack cannot hide a real gap.
            if residual - tolerance <= budget {
                // Clamp to the arena's maximum entity tolerance so a
                // near-max edge does not mint an unserializable value; when
                // the residual itself exceeds the max the gate still refuses
                // it below.
                let carried = (residual + budget)
                    .next_up()
                    .min(DEFAULT_MAX_ENTITY_TOLERANCE);
                topo.edge_mut(id)?.set_tolerance(Some(carried))?;
            }
        }
    }
    Ok(())
}

/// Determine the v-range (latitude) of a sphere face from its boundary.
///
/// Projects boundary vertices onto the sphere to find their latitudes,
/// then uses a pole vertex or the equatorial wire winding to determine
/// which hemisphere the face covers.
fn sphere_face_v_range(
    topo: &Topology,
    face_id: FaceId,
    sph: &remus_math::surfaces::SphericalSurface,
    // The vertex phase has already moved boundary points; this maps them back
    // into the still-untransformed sphere's frame before any projection or
    // hemisphere heuristic. Pass identity when vertices are untouched.
    inverse: &Mat4,
) -> Result<(f64, f64), crate::OperationsError> {
    use std::f64::consts::FRAC_PI_2;

    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;
    let mut v_vals = Vec::new();

    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let pt = inverse.mul_point(topo.vertex(edge.start())?.point());
        let (_u, v) = sph.project_point(pt);
        v_vals.push(v);
    }

    if v_vals.is_empty() {
        // Full sphere with no boundary → full range
        return Ok((-FRAC_PI_2, FRAC_PI_2));
    }

    // All boundary vertices should be at roughly the same v (equator).
    // Determine hemisphere by checking whether face is above or below boundary.
    let boundary_v = v_vals.iter().copied().sum::<f64>() / v_vals.len() as f64;

    if boundary_v.abs() < 0.1 {
        return match classify_equatorial_sphere_patch(topo, face_id, sph, inverse, boundary_v)? {
            SpherePatch::NorthHemisphere => Ok((boundary_v, FRAC_PI_2)),
            SpherePatch::SouthHemisphere => Ok((-FRAC_PI_2, boundary_v)),
            SpherePatch::Full | SpherePatch::General => Err(crate::OperationsError::InvalidInput {
                reason: "cannot determine sphere patch across an equatorial boundary".into(),
            }),
        };
    }

    if boundary_v > 0.0 {
        Ok((boundary_v, FRAC_PI_2))
    } else {
        Ok((-FRAC_PI_2, boundary_v))
    }
}

/// Check whether a transform matrix has uniform scaling (all axis scale
/// factors are approximately equal). Non-uniform scaling distorts spheres
/// into ellipsoids, so analytic representations must be converted to NURBS.
/// Refuse a circle/ellipse image whose transformed axes are no longer
/// orthogonal. The transformed axes are conjugate diameters of the image
/// ellipse; only when they remain orthogonal are they its principal axes,
/// which is what `Circle3D::with_axes`/`Ellipse3D::with_axes` require (they
/// never validate orthogonality themselves). Recovering principal axes from
/// skewed conjugates (Rytz's construction) also re-parameterizes the curve
/// and every trim on it — until that is implemented, failing by name beats
/// silently wrong exact geometry.
pub(crate) fn ensure_orthogonal_conjugate_axes(
    u: Vec3,
    v: Vec3,
    kind: &str,
) -> Result<(), crate::OperationsError> {
    let u_dir = u.normalize()?;
    let v_dir = v.normalize()?;
    if u_dir.dot(v_dir).abs() > ANALYTIC_ROUNDOFF_REL {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!(
                "transform maps a {kind} edge to a skewed ellipse (non-orthogonal image \
                 axes); this exact frame cannot represent it — apply the transform to a \
                 B-spline-converted body instead"
            ),
        });
    }
    Ok(())
}

/// Compute the scaled radius of a circle perpendicular to `axis` after transform.
fn scaled_radius(matrix: &Mat4, axis: Vec3, radius: f64) -> f64 {
    // Pick a direction perpendicular to the axis
    let perp = if axis.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
            .cross(axis)
            .normalize()
            .unwrap_or(Vec3::new(1.0, 0.0, 0.0))
    } else {
        Vec3::new(0.0, 1.0, 0.0)
            .cross(axis)
            .normalize()
            .unwrap_or(Vec3::new(0.0, 1.0, 0.0))
    };
    // Transform the perpendicular direction and measure its length
    let origin = remus_math::vec::Point3::new(0.0, 0.0, 0.0);
    let end = remus_math::vec::Point3::new(perp.x() * radius, perp.y() * radius, perp.z() * radius);
    let t_origin = matrix.mul_point(origin);
    let t_end = matrix.mul_point(end);
    let diff = t_end - t_origin;
    diff.length()
}

/// Transform a single face's surface geometry.
///
/// The `normal_matrix` should be `matrix.inverse()?.transpose()`.
///
/// Legacy void wrapper: records into a throwaway recorder so the engine
/// stays shared with the quality-aware path.
pub(crate) fn transform_face_surface(
    topo: &mut Topology,
    fid: FaceId,
    matrix: &Mat4,
    normal_matrix: &Mat4,
) -> Result<(), crate::OperationsError> {
    let mut recorder = TransformRecorder::new(linear_determinant(matrix) < 0.0, true);
    transform_face_surface_recorded(topo, fid, matrix, normal_matrix, &mut recorder)
}

/// Recorded face-surface transform shared by the legacy entry points, the
/// detailed twins, and the copy path.
pub(crate) fn transform_face_surface_recorded(
    topo: &mut Topology,
    fid: FaceId,
    matrix: &Mat4,
    normal_matrix: &Mat4,
    recorder: &mut TransformRecorder,
) -> Result<(), crate::OperationsError> {
    let from = topo.face(fid)?.surface().type_tag();
    let face = topo.face(fid)?;
    match face.surface() {
        FaceSurface::Plane { normal, .. } => {
            let n = *normal;
            let transformed =
                normal_matrix.mul_point(remus_math::vec::Point3::new(n.x(), n.y(), n.z()));
            let origin = normal_matrix.mul_point(remus_math::vec::Point3::new(0.0, 0.0, 0.0));
            let raw = Vec3::new(
                transformed.x() - origin.x(),
                transformed.y() - origin.y(),
                transformed.z() - origin.z(),
            );
            let new_normal = raw.normalize()?;
            let wire = topo.wire(face.outer_wire())?;
            let first_oe =
                wire.edges()
                    .first()
                    .ok_or_else(|| crate::OperationsError::InvalidInput {
                        reason: "face has empty outer wire".into(),
                    })?;
            let edge = topo.edge(first_oe.edge())?;
            let ref_vid = if first_oe.is_forward() {
                edge.start()
            } else {
                edge.end()
            };
            let ref_point = topo.vertex(ref_vid)?.point();
            let new_d = new_normal.dot(Vec3::new(ref_point.x(), ref_point.y(), ref_point.z()));
            topo.face_mut(fid)?.set_surface(FaceSurface::Plane {
                normal: new_normal,
                d: new_d,
            });
            // Planes stay planes under every affine map (normals re-derived
            // as covectors): nothing to record, no flag to carry.
        }
        FaceSurface::Nurbs(s) => {
            let s_clone = s.clone();
            let new_control_points: Vec<Vec<_>> = s_clone
                .control_points()
                .iter()
                .map(|row| row.iter().map(|pt| matrix.mul_point(*pt)).collect())
                .collect();
            let new_surface = NurbsSurface::new(
                s_clone.degree_u(),
                s_clone.degree_v(),
                s_clone.knots_u().to_vec(),
                s_clone.knots_v().to_vec(),
                new_control_points,
                s_clone.weights().to_vec(),
            );
            let oriented = orient_nurbs_for_map(&new_surface?, recorder.orientation_reversing)?;
            topo.face_mut(fid)?
                .set_surface(FaceSurface::Nurbs(oriented));
        }
        FaceSurface::Cylinder(cyl) => {
            if is_uniform_scale(matrix) {
                let cyl_clone = cyl.clone();
                let new_origin = matrix.mul_point(cyl_clone.origin());
                let new_axis = transform_direction(matrix, cyl_clone.axis())?;
                let new_radius = scaled_radius(matrix, cyl_clone.axis(), cyl_clone.radius());
                let new_cyl = remus_math::surfaces::CylindricalSurface::new(
                    new_origin, new_axis, new_radius,
                )?;
                topo.face_mut(fid)?
                    .set_surface(FaceSurface::Cylinder(new_cyl));
            } else {
                // Same reasoning as the sibling arm above: an anisotropic
                // scale makes the cylinder elliptic, so convert to NURBS
                // rather than keep a circular surface with a wrong radius.
                // Vertices were already moved; probe with their inverse image.
                let inverse = matrix.inverse()?;
                let v_range = analytic_face_v_range(topo, fid, |pt| {
                    cyl.project_point(inverse.mul_point(pt)).1
                })?;
                // End the surface borrow before the fallible conversion.
                let cyl_clone = cyl.clone();
                let nurbs =
                    remus_heal::construct::convert_surface::cylinder_to_nurbs(&cyl_clone, v_range)
                        .map_err(|e| crate::OperationsError::InvalidInput {
                            reason: format!("cylinder_to_nurbs failed: {e}"),
                        })?;
                let transformed = transform_nurbs_surface(&nurbs, matrix)?;
                let oriented = orient_nurbs_for_map(&transformed, recorder.orientation_reversing)?;
                let count = nurbs_control_point_count(&oriented);
                topo.face_mut(fid)?
                    .set_surface(FaceSurface::Nurbs(oriented));
                recorder.record_face(fid, from, "nurbs", SurfaceMethod::ExactRational, count);
            }
        }
        FaceSurface::Cone(cone) => {
            if is_uniform_scale(matrix) {
                let cone_clone = cone.clone();
                let new_apex = matrix.mul_point(cone_clone.apex());
                let new_axis = transform_direction(matrix, cone_clone.axis())?;
                let new_cone = remus_math::surfaces::ConicalSurface::new(
                    new_apex,
                    new_axis,
                    cone_clone.half_angle(),
                )?;
                topo.face_mut(fid)?.set_surface(FaceSurface::Cone(new_cone));
            } else {
                let inverse = matrix.inverse()?;
                let v_range = analytic_face_v_range(topo, fid, |pt| {
                    cone.project_point(inverse.mul_point(pt)).1
                })?;
                let cone_clone = cone.clone();
                let nurbs =
                    remus_heal::construct::convert_surface::cone_to_nurbs(&cone_clone, v_range)
                        .map_err(|e| crate::OperationsError::InvalidInput {
                            reason: format!("cone_to_nurbs failed: {e}"),
                        })?;
                let transformed = transform_nurbs_surface(&nurbs, matrix)?;
                let oriented = orient_nurbs_for_map(&transformed, recorder.orientation_reversing)?;
                let count = nurbs_control_point_count(&oriented);
                topo.face_mut(fid)?
                    .set_surface(FaceSurface::Nurbs(oriented));
                recorder.record_face(fid, from, "nurbs", SurfaceMethod::ExactRational, count);
            }
        }
        FaceSurface::Sphere(sph) => {
            if is_uniform_scale(matrix) {
                let sph_clone = sph.clone();
                let new_center = matrix.mul_point(sph_clone.center());
                let m = &matrix.0;
                let sx = (m[0][0] * m[0][0] + m[1][0] * m[1][0] + m[2][0] * m[2][0]).sqrt();
                let new_sph = remus_math::surfaces::SphericalSurface::with_frame(
                    new_center,
                    sph_clone.radius() * sx,
                    transform_direction(matrix, sph_clone.z_axis())?,
                    transform_direction(matrix, sph_clone.x_axis())?,
                )?;
                topo.face_mut(fid)?
                    .set_surface(FaceSurface::Sphere(new_sph));
            } else {
                let inverse = matrix.inverse()?;
                let sph_clone = sph.clone();
                match classify_sphere_patch(topo, fid, &sph_clone, &inverse)? {
                    SpherePatch::General => {
                        if !recorder.allow_fit {
                            return Err(crate::OperationsError::ExactOnlyUnattainable);
                        }
                        // Historical fit range, unchanged: the face's
                        // v-extent of the sphere refit as NURBS. Boundary
                        // vertices were already moved; probe with their
                        // inverse image.
                        let (v_min, v_max) = sphere_face_v_range(topo, fid, &sph_clone, &inverse)?;
                        let nurbs = sphere_to_transformed_nurbs(&sph_clone, matrix, v_min, v_max)?;
                        let count = nurbs_control_point_count(&nurbs);
                        let (max_residual, check_points) = fitted_patch_evidence(
                            &nurbs,
                            &|u, v| matrix.mul_point(sph_clone.evaluate(u, v)),
                            v_min,
                            v_max,
                        );
                        let oriented =
                            orient_nurbs_for_map(&nurbs, recorder.orientation_reversing)?;
                        topo.face_mut(fid)?
                            .set_surface(FaceSurface::Nurbs(oriented));
                        recorder.record_face(fid, from, "nurbs", SurfaceMethod::SampledFit, count);
                        recorder.record_fit(FittedFaceEvidence {
                            face: fid,
                            from,
                            method: "interpolate33x17",
                            grid: (SPHERE_FIT_GRID_U, SPHERE_FIT_GRID_V),
                            max_residual,
                            check_points,
                        });
                    }
                    patch => {
                        let transformed = sphere_patch_to_nurbs(&sph_clone, patch, matrix)?;
                        let oriented =
                            orient_nurbs_for_map(&transformed, recorder.orientation_reversing)?;
                        let count = nurbs_control_point_count(&oriented);
                        topo.face_mut(fid)?
                            .set_surface(FaceSurface::Nurbs(oriented));
                        recorder.record_face(
                            fid,
                            from,
                            "nurbs",
                            SurfaceMethod::ExactRational,
                            count,
                        );
                    }
                }
            }
        }
        FaceSurface::Torus(tor) => {
            if is_uniform_scale(matrix) {
                let tor_clone = tor.clone();
                let new_center = matrix.mul_point(tor_clone.center());
                let m = &matrix.0;
                let sx = (m[0][0] * m[0][0] + m[1][0] * m[1][0] + m[2][0] * m[2][0]).sqrt();
                let new_tor = remus_math::surfaces::ToroidalSurface::with_axis_and_ref_dir(
                    new_center,
                    tor_clone.major_radius() * sx,
                    tor_clone.minor_radius() * sx,
                    transform_direction(matrix, tor_clone.z_axis())?,
                    transform_direction(matrix, tor_clone.x_axis())?,
                )?;
                topo.face_mut(fid)?.set_surface(FaceSurface::Torus(new_tor));
            } else {
                let tor_clone = tor.clone();
                let nurbs = remus_heal::construct::convert_surface::torus_to_nurbs(&tor_clone)
                    .map_err(|e| crate::OperationsError::InvalidInput {
                        reason: format!("torus_to_nurbs failed: {e}"),
                    })?;
                let transformed = transform_nurbs_surface(&nurbs, matrix)?;
                let oriented = orient_nurbs_for_map(&transformed, recorder.orientation_reversing)?;
                let count = nurbs_control_point_count(&oriented);
                topo.face_mut(fid)?
                    .set_surface(FaceSurface::Nurbs(oriented));
                recorder.record_face(fid, from, "nurbs", SurfaceMethod::ExactRational, count);
            }
        }
    }
    // Converted and reflected NURBS faces have a different UV chart. Keep
    // their exact 3D edges, but remove pcurves that use the old coordinates.
    let chart_replaced = matches!(topo.face(fid)?.surface(), FaceSurface::Nurbs(_))
        && (from != "nurbs" || recorder.orientation_reversing);
    if chart_replaced {
        let uses: Vec<_> = topo
            .pcurves_for_face(fid)
            .into_iter()
            .map(|(edge, forward, _)| (edge, forward))
            .collect();
        for (edge, forward) in uses {
            topo.remove_pcurve_oriented(edge, fid, forward)?;
        }
    }
    Ok(())
}

/// Carry orientation across an orientation-reversing map for NURBS carriers.
///
/// An affine map with a negative linear determinant flips the handedness of
/// a NURBS parameterization (`Su × Sv` points inward). The face's
/// `reversed` flag must NOT carry this instead: the shell validator reads
/// `forward != reversed` per shared edge, so toggling flags on converted
/// faces alone breaks mixed shells (B74: mirrored cylinder wall vs. caps).
/// Reversing the surface in u flips the handedness back with flags,
/// coedges, and trims untouched — the same point set, opposite travel.
///
/// Only NURBS faces need this. Analytic carriers are rebuilt right-handed
/// and plane normals are re-derived as covectors, both already correct.
fn orient_nurbs_for_map(
    surface: &NurbsSurface,
    reversing: bool,
) -> Result<NurbsSurface, crate::OperationsError> {
    if !reversing {
        return Ok(surface.clone());
    }
    // Mirror of `NurbsCurve::reversed` in u: control rows and the u-knot
    // vector mirror inside their own span, so the domain endpoints are
    // unchanged and `reversed.evaluate(u0 + u1 − u, v)` equals
    // `evaluate(u, v)`. Degree, multiplicities, and rationality survive.
    let (du0, du1) = surface.domain_u();
    let span = du0 + du1;
    let mut knots_u: Vec<f64> = surface.knots_u().iter().rev().map(|k| span - k).collect();
    if let (Some(first), Some(&original)) = (knots_u.first_mut(), surface.knots_u().first()) {
        *first = original;
    }
    if let (Some(last), Some(&original)) = (knots_u.last_mut(), surface.knots_u().last()) {
        *last = original;
    }
    let control_points: Vec<Vec<remus_math::vec::Point3>> =
        surface.control_points().iter().rev().cloned().collect();
    let weights: Vec<Vec<f64>> = surface.weights().iter().rev().cloned().collect();
    Ok(NurbsSurface::new(
        surface.degree_u(),
        surface.degree_v(),
        knots_u,
        surface.knots_v().to_vec(),
        control_points,
        weights,
    )?)
}

/// Control-point footprint of a NURBS surface: deterministic output-size
/// measure for conversion cost tracking.
fn nurbs_control_point_count(surface: &NurbsSurface) -> usize {
    surface.control_points().iter().map(Vec::len).sum()
}

/// Exact rational NURBS for one classified sphere patch, mapped by `matrix`.
///
/// The full-sphere converter is set-exact on the analytic sphere, and
/// knot-insertion splits keep the sub-domains, so 3D trims land on the
/// right set. The `(u, v)`-to-3D map is the converter's own rational
/// parameterization (documented on `sphere_to_nurbs`), not the analytic
/// trigonometric one: UV traces do not carry across, only point sets.
fn sphere_patch_to_nurbs(
    sph: &remus_math::surfaces::SphericalSurface,
    patch: SpherePatch,
    matrix: &Mat4,
) -> Result<NurbsSurface, crate::OperationsError> {
    let full = remus_heal::construct::convert_surface::sphere_to_nurbs(sph).map_err(|e| {
        crate::OperationsError::InvalidInput {
            reason: format!("sphere_to_nurbs failed: {e}"),
        }
    })?;
    let split_at = |v: f64| {
        remus_heal::upgrade::split_surface::split_surface_at_v(&full, v).map_err(|e| {
            crate::OperationsError::InvalidInput {
                reason: format!("splitting sphere NURBS failed: {e}"),
            }
        })
    };
    match patch {
        SpherePatch::Full => transform_nurbs_surface(&full, matrix),
        SpherePatch::NorthHemisphere => {
            let (_, north) = split_at(0.0)?;
            transform_nurbs_surface(&north, matrix)
        }
        SpherePatch::SouthHemisphere => {
            let (south, _) = split_at(0.0)?;
            transform_nurbs_surface(&south, matrix)
        }
        SpherePatch::General => Err(crate::OperationsError::ExactOnlyUnattainable),
    }
}

/// Sampled evidence for a fitted patch: symmetric sampled Hausdorff
/// estimate between uniform samples of the fitted NURBS domain and a fine
/// analytic grid of the true affine image.
///
/// Returns `(max_residual, check_points)`. The grids are finite, so this
/// is measured evidence, never a certified bound.
fn fitted_patch_evidence(
    nurbs: &NurbsSurface,
    analytic_point: &dyn Fn(f64, f64) -> remus_math::vec::Point3,
    v_min: f64,
    v_max: f64,
) -> (f64, usize) {
    use std::f64::consts::TAU;

    let (du0, du1) = nurbs.domain_u();
    let (dv0, dv1) = nurbs.domain_v();
    let mut nurbs_points = Vec::with_capacity(FIT_EVIDENCE_NURBS_U * FIT_EVIDENCE_NURBS_V);
    for iu in 0..FIT_EVIDENCE_NURBS_U {
        let s = du0 + (du1 - du0) * (iu as f64) / ((FIT_EVIDENCE_NURBS_U - 1) as f64);
        for iv in 0..FIT_EVIDENCE_NURBS_V {
            let t = dv0 + (dv1 - dv0) * (iv as f64) / ((FIT_EVIDENCE_NURBS_V - 1) as f64);
            nurbs_points.push(nurbs.evaluate(s, t));
        }
    }
    let mut analytic_points = Vec::with_capacity(FIT_EVIDENCE_ANALYTIC_U * FIT_EVIDENCE_ANALYTIC_V);
    for iu in 0..FIT_EVIDENCE_ANALYTIC_U {
        let u = TAU * (iu as f64) / ((FIT_EVIDENCE_ANALYTIC_U - 1) as f64);
        for iv in 0..FIT_EVIDENCE_ANALYTIC_V {
            let v = v_min + (v_max - v_min) * (iv as f64) / ((FIT_EVIDENCE_ANALYTIC_V - 1) as f64);
            analytic_points.push(analytic_point(u, v));
        }
    }
    let mut max_residual: f64 = 0.0;
    for point in &nurbs_points {
        let mut nearest = f64::INFINITY;
        for other in &analytic_points {
            nearest = nearest.min((*point - *other).length());
        }
        max_residual = max_residual.max(nearest);
    }
    for point in &analytic_points {
        let mut nearest = f64::INFINITY;
        for other in &nurbs_points {
            nearest = nearest.min((*point - *other).length());
        }
        max_residual = max_residual.max(nearest);
    }
    if !max_residual.is_finite() {
        max_residual = f64::INFINITY;
    }
    (max_residual, nurbs_points.len() + analytic_points.len())
}

/// Compute the v-parameter range for an analytic surface face.
///
/// Projects boundary vertices using `project_v` and returns (v_min, v_max).
fn analytic_face_v_range(
    topo: &Topology,
    face_id: FaceId,
    project_v: impl Fn(remus_math::vec::Point3) -> f64,
) -> Result<(f64, f64), crate::OperationsError> {
    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let pt = topo.vertex(edge.start())?.point();
        let v = project_v(pt);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }
    if v_min >= v_max {
        v_min = 0.0;
        v_max = 1.0;
    }
    Ok((v_min, v_max))
}

/// Transform a NURBS surface's control points by a matrix.
fn transform_nurbs_surface(
    surface: &NurbsSurface,
    matrix: &Mat4,
) -> Result<NurbsSurface, crate::OperationsError> {
    let new_cps: Vec<Vec<_>> = surface
        .control_points()
        .iter()
        .map(|row| row.iter().map(|pt| matrix.mul_point(*pt)).collect())
        .collect();
    Ok(NurbsSurface::new(
        surface.degree_u(),
        surface.degree_v(),
        surface.knots_u().to_vec(),
        surface.knots_v().to_vec(),
        new_cps,
        surface.weights().to_vec(),
    )?)
}

/// Whether a matrix preserves endpoint residuals up to f64 rounding:
/// a rigid motion (rotation/reflection + translation) or a rigid motion
/// with a uniform scale. Both keep the linear part conformal (MᵀM = s²I),
/// so the only drift is independent rounding of the mapped vertices and
/// curve frames. Anything else (shear, anisotropic scale) can move the
/// residual geometrically and must not mint certificates.
///
/// Separated from [`is_uniform_scale`] (which answers whether analytic
/// surfaces survive exactly) so each gate states its own contract.
fn is_rigid_or_uniform_scale(matrix: &Mat4) -> bool {
    is_uniform_scale(matrix)
}

/// Whether a matrix preserves every analytic carrier (similarity).
///
/// True when the linear part is conformal (`MᵀM = s²I` within
/// [`ANALYTIC_ROUNDOFF_REL`]): rotations, reflections, translations, and
/// uniform scales of any magnitude. Every other affine map converts at
/// least some carriers to NURBS. The engine routes on this predicate, and
/// the report echoes it, so carrier routing and disclosure cannot disagree.
#[must_use]
pub fn is_similarity(matrix: &Mat4) -> bool {
    is_uniform_scale(matrix)
}

fn is_uniform_scale(matrix: &Mat4) -> bool {
    let m = &matrix.0;
    // A map preserves analytic shapes exactly when its linear part is
    // conformal: MᵀM = s²I. Equal column norms alone are not enough — a
    // shear, or a single-axis scale at 45° to the columns, can keep the
    // norms equal while distorting circles into ellipses — and the old 1%
    // tolerance waved through deliberate small scales (a 1.009× stretch kept
    // a sphere "analytic" with a wrong radius). Check both column-norm
    // equality and mutual orthogonality at float-noise tolerance: composed
    // rotations carry ~1e-15 error, real anisotropy sits many orders above.
    let col = |j: usize| Vec3::new(m[0][j], m[1][j], m[2][j]);
    let (cx, cy, cz) = (col(0), col(1), col(2));
    let (nx, ny, nz) = (cx.length(), cy.length(), cz.length());
    if !nx.is_finite() || !ny.is_finite() || !nz.is_finite() {
        return false;
    }
    let norms_equal = equal_within_analytic_roundoff(nx, ny)
        && equal_within_analytic_roundoff(ny, nz)
        && equal_within_analytic_roundoff(nz, nx);
    let Ok(ux) = cx.normalize() else {
        return false;
    };
    let Ok(uy) = cy.normalize() else {
        return false;
    };
    let Ok(uz) = cz.normalize() else {
        return false;
    };
    let orthogonal = ux.dot(uy).abs() <= ANALYTIC_ROUNDOFF_REL
        && uy.dot(uz).abs() <= ANALYTIC_ROUNDOFF_REL
        && uz.dot(ux).abs() <= ANALYTIC_ROUNDOFF_REL;
    norms_equal && orthogonal
}

/// Convert a spherical patch to NURBS and apply a non-uniform transform.
///
/// Whole hemispheres use exact rational half-sphere patches. Other trims retain
/// their analytic parameter match through sampled interpolation.
fn sphere_to_transformed_nurbs(
    sph: &remus_math::surfaces::SphericalSurface,
    matrix: &Mat4,
    v_min: f64,
    v_max: f64,
) -> Result<NurbsSurface, crate::OperationsError> {
    use std::f64::consts::FRAC_PI_2;

    let is_north_hemisphere =
        v_min.abs() <= ANALYTIC_ROUNDOFF_REL && (v_max - FRAC_PI_2).abs() <= ANALYTIC_ROUNDOFF_REL;
    let is_south_hemisphere =
        (v_min + FRAC_PI_2).abs() <= ANALYTIC_ROUNDOFF_REL && v_max.abs() <= ANALYTIC_ROUNDOFF_REL;
    if is_north_hemisphere || is_south_hemisphere {
        let full = remus_heal::construct::convert_surface::sphere_to_nurbs(sph).map_err(|e| {
            crate::OperationsError::InvalidInput {
                reason: format!("sphere_to_nurbs failed: {e}"),
            }
        })?;
        let (south, north) = remus_heal::upgrade::split_surface::split_surface_at_v(&full, 0.0)
            .map_err(|e| crate::OperationsError::InvalidInput {
                reason: format!("splitting sphere NURBS at the equator failed: {e}"),
            })?;
        // Select on the classification, not on `sign(v_min)`: an equatorial
        // trim reaches here with `v_min` a rounding-noise value either side
        // of zero, which would hand a north face the southern patch.
        let patch = if is_south_hemisphere { &south } else { &north };
        return transform_nurbs_surface(patch, matrix);
    }

    sampled_transformed_nurbs(|u, v| sph.evaluate(u, v), matrix, v_min, v_max)
}

/// Sample an analytic surface over (u ∈ [0, τ], v ∈ [v_min, v_max]), map the
/// samples through `matrix`, and refit as a NURBS surface.
///
/// Used for trimmed sphere patches whose analytic latitude does not map to the
/// exact rational converter's polynomial parameter. Whole hemispheres and the
/// cylinder/cone/torus paths use exact rational conversions instead.
fn sampled_transformed_nurbs(
    evaluate: impl Fn(f64, f64) -> remus_math::vec::Point3,
    matrix: &Mat4,
    v_min: f64,
    v_max: f64,
) -> Result<NurbsSurface, crate::OperationsError> {
    use std::f64::consts::TAU;

    let n_u = SPHERE_FIT_GRID_U; // Angular samples (0 to 2π; endpoints coincide at the seam)
    let n_v = SPHERE_FIT_GRID_V;

    let mut rows: Vec<Vec<remus_math::vec::Point3>> = Vec::with_capacity(n_v);
    for iv in 0..n_v {
        let v = v_min + (v_max - v_min) * (iv as f64) / ((n_v - 1) as f64);
        let mut row = Vec::with_capacity(n_u);
        for iu in 0..n_u {
            let u = TAU * (iu as f64) / ((n_u - 1) as f64);
            row.push(matrix.mul_point(evaluate(u, v)));
        }
        rows.push(row);
    }

    let nurbs = remus_math::nurbs::surface_fitting::interpolate_surface(&rows, 3, 3)?;
    Ok(nurbs)
}

/// Transforms a direction vector by applying the matrix and subtracting the
/// translation component, then normalizing.
fn transform_direction(matrix: &Mat4, dir: Vec3) -> Result<Vec3, crate::OperationsError> {
    let origin = matrix.mul_point(remus_math::vec::Point3::new(0.0, 0.0, 0.0));
    let tip = matrix.mul_point(remus_math::vec::Point3::new(dir.x(), dir.y(), dir.z()));
    let raw = Vec3::new(
        tip.x() - origin.x(),
        tip.y() - origin.y(),
        tip.z() - origin.z(),
    );
    Ok(raw.normalize()?)
}

/// The transformed curve (None for Line) and its new explicit trim.
pub(crate) type TransformedEdgeCurve = (Option<EdgeCurve>, Option<(f64, f64)>);

/// Transform one edge curve and its explicit RFC 0002 trim by `matrix`.
///
/// Returns `(Some(new_curve), new_trim)`; `None` for `Line`, whose geometry
/// is defined by its vertices. The trim is retained where the map provably
/// preserves the curve's parameterization (NURBS control-point maps,
/// hyperbolas under a similarity, scaled circles/ellipses), exactly remapped
/// for handled re-parameterizations (parabola `t ↦ s·t`; Circle→Ellipse with
/// swapped principal axes `t ↦ t − π/2`), and dropped otherwise — the
/// endpoint-projection fallback then re-derives the domain on the new
/// parameterization.
pub(crate) fn transform_edge_curve_with_trim(
    curve: &EdgeCurve,
    trim: Option<(f64, f64)>,
    matrix: &Mat4,
) -> Result<TransformedEdgeCurve, crate::OperationsError> {
    // Translation must not participate in direction arithmetic: subtracting
    // translated points can make a rigid image falsely appear anisotropic.
    let transform_dir = |d: Vec3| -> Vec3 {
        let m = &matrix.0;
        Vec3::new(
            m[0][0].mul_add(d.x(), m[0][1].mul_add(d.y(), m[0][2] * d.z())),
            m[1][0].mul_add(d.x(), m[1][1].mul_add(d.y(), m[1][2] * d.z())),
            m[2][0].mul_add(d.x(), m[2][1].mul_add(d.y(), m[2][2] * d.z())),
        )
    };
    let (new_curve, new_trim) = match curve {
        EdgeCurve::Line => (None, None),
        // Exact under a similarity, typed refusal otherwise — see
        // `transform_open_conic`.
        c @ (EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_)) => {
            let (image, parameter_scale) = transform_open_conic(c, matrix)?;
            (
                Some(image),
                trim.map(|(t0, t1)| (t0 * parameter_scale, t1 * parameter_scale)),
            )
        }
        EdgeCurve::NurbsCurve(c) => {
            let new_control_points: Vec<_> = c
                .control_points()
                .iter()
                .map(|pt| matrix.mul_point(*pt))
                .collect();
            (
                Some(EdgeCurve::NurbsCurve(NurbsCurve::new(
                    c.degree(),
                    c.knots().to_vec(),
                    new_control_points,
                    c.weights().to_vec(),
                )?)),
                trim,
            )
        }
        EdgeCurve::Circle(c) => {
            let new_center = matrix.mul_point(c.center());
            let new_u = transform_dir(c.u_axis());
            let new_v = transform_dir(c.v_axis());
            let su = new_u.length();
            let sv = new_v.length();
            // The transformed axes are conjugate diameters of the image
            // ellipse, not its principal axes. When they stay orthogonal
            // they ARE principal and the arms below are exact; when they
            // do not (a shear, or an anisotropic scale oblique to the
            // circle plane), building a Circle/Ellipse from them silently
            // emits a skewed frame — refuse instead.
            ensure_orthogonal_conjugate_axes(new_u, new_v, "circular")?;
            let new_normal = new_u.cross(new_v).normalize()?;
            if equal_within_analytic_roundoff(su, sv) {
                (
                    Some(EdgeCurve::Circle(remus_math::curves::Circle3D::with_axes(
                        new_center,
                        new_normal,
                        c.radius() * su,
                        new_u.normalize()?,
                        new_v.normalize()?,
                    )?)),
                    trim,
                )
            } else {
                let (semi_major, semi_minor, u_dir, v_dir, mapped_trim) = if su >= sv {
                    (
                        c.radius() * su,
                        c.radius() * sv,
                        new_u.normalize()?,
                        new_v.normalize()?,
                        trim,
                    )
                } else {
                    (
                        c.radius() * sv,
                        c.radius() * su,
                        new_v.normalize()?,
                        -new_u.normalize()?,
                        trim.map(|(a, b)| {
                            (
                                a - std::f64::consts::FRAC_PI_2,
                                b - std::f64::consts::FRAC_PI_2,
                            )
                        }),
                    )
                };
                (
                    Some(EdgeCurve::Ellipse(
                        remus_math::curves::Ellipse3D::with_axes(
                            new_center, new_normal, semi_major, semi_minor, u_dir, v_dir,
                        )?,
                    )),
                    mapped_trim,
                )
            }
        }
        EdgeCurve::Ellipse(e) => {
            let new_center = matrix.mul_point(e.center());
            let new_u = transform_dir(e.u_axis());
            let new_v = transform_dir(e.v_axis());
            // Same conjugate-diameter reasoning as the Circle arm above.
            ensure_orthogonal_conjugate_axes(new_u, new_v, "elliptical")?;
            let new_normal = new_u.cross(new_v).normalize()?;
            let u_extent = e.semi_major() * new_u.length();
            let v_extent = e.semi_minor() * new_v.length();
            let (semi_major, semi_minor, u_dir, v_dir, mapped_trim) = if u_extent >= v_extent {
                (
                    u_extent,
                    v_extent,
                    new_u.normalize()?,
                    new_v.normalize()?,
                    trim,
                )
            } else {
                (
                    v_extent,
                    u_extent,
                    new_v.normalize()?,
                    -new_u.normalize()?,
                    trim.map(|(a, b)| {
                        (
                            a - std::f64::consts::FRAC_PI_2,
                            b - std::f64::consts::FRAC_PI_2,
                        )
                    }),
                )
            };
            (
                Some(EdgeCurve::Ellipse(
                    remus_math::curves::Ellipse3D::with_axes(
                        new_center, new_normal, semi_major, semi_minor, u_dir, v_dir,
                    )?,
                )),
                mapped_trim,
            )
        }
    };
    Ok((new_curve, new_trim))
}

/// Transform a set of edge curves in place.
///
/// Line edges need no update — their geometry is defined by vertices.
/// Legacy void wrapper over [`transform_edges_recorded`].
pub(crate) fn transform_edges(
    topo: &mut Topology,
    edge_ids: &HashSet<EdgeId>,
    matrix: &Mat4,
) -> Result<(), crate::OperationsError> {
    let mut recorder = TransformRecorder::new(false, true);
    transform_edges_recorded(topo, edge_ids, matrix, &mut recorder)
}

/// Recorded edge-curve transform shared by the legacy entry points, the
/// detailed twins, and the copy path.
pub(crate) fn transform_edges_recorded(
    topo: &mut Topology,
    edge_ids: &HashSet<EdgeId>,
    matrix: &Mat4,
    recorder: &mut TransformRecorder,
) -> Result<(), crate::OperationsError> {
    for &eid in edge_ids {
        let edge = topo.edge(eid)?;
        let from = edge.curve().type_tag();
        let (new_curve, new_trim) =
            transform_edge_curve_with_trim(edge.curve(), edge.trim(), matrix)?;
        let to = new_curve.as_ref().map_or(from, |curve| curve.type_tag());
        if let Some(curve) = new_curve {
            let edge = topo.edge_mut(eid)?;
            edge.set_curve(curve);
            edge.set_trim(new_trim);
        } else if topo.edge(eid)?.trim().is_some() {
            topo.edge_mut(eid)?.set_trim(None);
        }
        recorder.record_edge(eid, from, to);
    }
    Ok(())
}

/// Apply an affine transform to a wire, modifying vertex positions and
/// edge curve geometry in place.
///
/// Runs transacted like [`transform_solid`]: a refused edge image rolls
/// back instead of stranding moved vertices.
///
/// # Errors
///
/// Returns an error if the matrix is degenerate or a referenced entity is missing.
pub fn transform_wire(
    topo: &mut Topology,
    wire_id: WireId,
    matrix: &Mat4,
) -> Result<(), crate::OperationsError> {
    reject_degenerate_transform(matrix)?;
    let _ = matrix.inverse()?.transpose();
    run_transacted(topo, |live| {
        let (vertex_ids, edge_ids) = collect_wire_entities(live, wire_id)?;

        // Transform vertices.
        for vid in vertex_ids {
            let vertex = live.vertex_mut(vid)?;
            let new_point = matrix.mul_point(vertex.point());
            vertex.set_point(new_point);
        }

        // Transform edge curves.
        transform_edges(live, &edge_ids, matrix)?;

        Ok(())
    })
}

/// Apply an affine transform to a face, modifying vertex positions, edge
/// curve geometry, and the face surface in place.
///
/// Transforms all vertices/edges in the face's outer and inner wires, then
/// updates the face surface geometry (plane normal, NURBS CPs, etc.).
///
/// Runs transacted like [`transform_solid`].
///
/// # Errors
///
/// Returns an error if the matrix is degenerate or a referenced entity is missing.
#[allow(clippy::too_many_lines)]
pub fn transform_face(
    topo: &mut Topology,
    face_id: FaceId,
    matrix: &Mat4,
) -> Result<(), crate::OperationsError> {
    reject_degenerate_transform(matrix)?;
    // Validate every part of the matrix before changing live topology.
    let normal_matrix = matrix.inverse()?.transpose();
    run_transacted(topo, |live| {
        // Collect all vertices and edges from the face's wires.
        let (vertex_ids, edge_ids) = collect_face_entities(live, face_id)?;

        // Transform vertices.
        for vid in vertex_ids {
            let vertex = live.vertex_mut(vid)?;
            let new_point = matrix.mul_point(vertex.point());
            vertex.set_point(new_point);
        }

        // Transform edge curves.
        transform_edges(live, &edge_ids, matrix)?;

        // Transform face surface.
        transform_face_surface(live, face_id, matrix, &normal_matrix)?;

        Ok(())
    })
}

/// Traverses face → wires → edges → vertices and returns deduplicated sets.
fn collect_face_entities(
    topo: &Topology,
    face_id: FaceId,
) -> Result<(HashSet<VertexId>, HashSet<EdgeId>), crate::OperationsError> {
    let mut vertex_ids = HashSet::new();
    let mut edge_ids = HashSet::new();
    let face = topo.face(face_id)?;
    let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
        .chain(face.inner_wires().iter().copied())
        .collect();

    for wid in wire_ids {
        let wire = topo.wire(wid)?;
        for oe in wire.edges() {
            let eid = oe.edge();
            edge_ids.insert(eid);
            let edge = topo.edge(eid)?;
            vertex_ids.insert(edge.start());
            vertex_ids.insert(edge.end());
        }
    }

    Ok((vertex_ids, edge_ids))
}

/// Traverses wire → edges → vertices and returns deduplicated sets.
fn collect_wire_entities(
    topo: &Topology,
    wire_id: WireId,
) -> Result<(HashSet<VertexId>, HashSet<EdgeId>), crate::OperationsError> {
    let mut vertex_ids = HashSet::new();
    let mut edge_ids = HashSet::new();
    let wire = topo.wire(wire_id)?;
    for oe in wire.edges() {
        let eid = oe.edge();
        edge_ids.insert(eid);
        let edge = topo.edge(eid)?;
        vertex_ids.insert(edge.start());
        vertex_ids.insert(edge.end());
    }
    Ok((vertex_ids, edge_ids))
}

/// Traverses solid → shells → faces → wires → edges → vertices and
/// returns deduplicated sets of vertex IDs, edge IDs, and face IDs.
#[allow(clippy::type_complexity)]
fn collect_solid_entities(
    topo: &Topology,
    solid: SolidId,
) -> Result<(HashSet<VertexId>, HashSet<EdgeId>, HashSet<FaceId>), crate::OperationsError> {
    let mut vertex_ids = HashSet::new();
    let mut edge_ids = HashSet::new();
    let mut face_ids = HashSet::new();
    let solid_data = topo.solid(solid)?;
    let shell_ids: Vec<_> = std::iter::once(solid_data.outer_shell())
        .chain(solid_data.inner_shells().iter().copied())
        .collect();

    for shell_id in shell_ids {
        let shell = topo.shell(shell_id)?;
        let fids: Vec<_> = shell.faces().to_vec();

        for face_id in fids {
            face_ids.insert(face_id);
            let face = topo.face(face_id)?;
            let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
                .chain(face.inner_wires().iter().copied())
                .collect();

            for wire_id in wire_ids {
                let wire = topo.wire(wire_id)?;
                for oe in wire.edges() {
                    let eid = oe.edge();
                    edge_ids.insert(eid);
                    let edge = topo.edge(eid)?;
                    vertex_ids.insert(edge.start());
                    vertex_ids.insert(edge.end());
                }
            }
        }
    }

    Ok((vertex_ids, edge_ids, face_ids))
}

#[cfg(test)]
mod tests;

/// Transform an unbounded conic edge curve (`Hyperbola` / `Parabola`).
///
/// An affine map sends a parabola to a parabola and a hyperbola to a
/// hyperbola, but the image is only expressible in remus's canonical
/// `(centre/vertex, orthonormal axes, semi-axes/focal length)` form when
/// the map restricted to the conic's own plane is a *similarity* — a
/// uniform scale with a rotation and/or reflection. Under a shear or a
/// non-uniform scale the image is still a conic of the same type, but its
/// canonical axes are rotated by an amount this representation cannot
/// recover without a full re-fit.
///
/// So: exact when the in-plane map is a similarity, and a typed refusal
/// naming the variant otherwise. Silently keeping the untransformed
/// parameters, or approximating with the pre-image's axes, would move the
/// edge geometry away from its own vertices.
///
/// The similarity test is dimensionless — it compares axis image lengths
/// and their mutual dot product *relative to* the scale factor — so it
/// behaves identically at any model scale.
pub(crate) fn transform_open_conic(
    curve: &EdgeCurve,
    matrix: &Mat4,
) -> Result<(EdgeCurve, f64), crate::OperationsError> {
    use remus_math::curves::{Hyperbola3D, Parabola3D};
    use remus_math::vec::Point3;

    let origin = matrix.mul_point(Point3::new(0.0, 0.0, 0.0));
    let dir = |d: Vec3| -> Vec3 { matrix.mul_point(Point3::new(d.x(), d.y(), d.z())) - origin };

    // Uniform in-plane scale factor, or `None` if the map shears or scales
    // the two in-plane axes differently.
    let in_plane_scale = |a: Vec3, b: Vec3| -> Option<f64> {
        let (ia, ib) = (dir(a), dir(b));
        let (la, lb) = (ia.length(), ib.length());
        if !equal_within_analytic_roundoff(la, lb) {
            return None;
        }
        let (Ok(ua), Ok(ub)) = (ia.normalize(), ib.normalize()) else {
            return None;
        };
        if ua.dot(ub).abs() > ANALYTIC_ROUNDOFF_REL {
            return None;
        }
        Some(f64::midpoint(la, lb))
    };

    let refuse = |variant: &'static str| crate::OperationsError::Unsupported {
        operation: "transform",
        reason: format!(
            "{variant} edge under a non-similarity transform: the image is a \
             {variant} but its canonical axes cannot be recovered from this \
             representation"
        ),
    };

    match curve {
        EdgeCurve::Hyperbola(h) => {
            let s = in_plane_scale(h.u_axis(), h.v_axis()).ok_or_else(|| refuse("hyperbola"))?;
            Ok((
                EdgeCurve::Hyperbola(Hyperbola3D::with_axes(
                    matrix.mul_point(h.center()),
                    dir(h.u_axis()).cross(dir(h.v_axis())),
                    dir(h.u_axis()),
                    h.semi_major() * s,
                    h.semi_minor() * s,
                )?),
                1.0,
            ))
        }
        EdgeCurve::Parabola(p) => {
            let s = in_plane_scale(p.axis_dir(), p.u_axis()).ok_or_else(|| refuse("parabola"))?;
            Ok((
                EdgeCurve::Parabola(Parabola3D::with_axes(
                    matrix.mul_point(p.vertex()),
                    dir(p.axis_dir()),
                    dir(p.u_axis()),
                    p.focal_length() * s,
                )?),
                s,
            ))
        }
        EdgeCurve::Line
        | EdgeCurve::Circle(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::NurbsCurve(_) => Err(crate::OperationsError::Unsupported {
            operation: "transform",
            reason: format!(
                "transform_open_conic called with `{}`, which is not an \
                     unbounded conic",
                curve.type_tag()
            ),
        }),
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod translation_certificate_tests {
    use super::*;
    use remus_math::vec::Point3;
    use remus_topology::{edge::Edge, vertex::Vertex};

    #[test]
    fn translation_preserves_only_prevalidated_roundoff_certificates() {
        for (valid_source, corrupt_after) in [(true, false), (false, false), (true, true)] {
            let mut topo = Topology::new();
            let p = Point3::new(0.0, 0.0, 0.0);
            let q = Point3::new(10.0, 0.0, 0.0);
            let a = topo.add_vertex(Vertex::new(p, 1e-7));
            let b = topo.add_vertex(Vertex::new(q, 1e-7));
            let gap = 0.00004;
            let curve = NurbsCurve::new(
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![Point3::new(gap, 0.0, 0.0), q],
                vec![1.0, 1.0],
            )
            .unwrap();
            let tolerance = if valid_source { gap } else { gap / 2.0 };
            let mut edge =
                Edge::with_tolerance(a, b, EdgeCurve::NurbsCurve(curve), Some(tolerance));
            edge.set_trim(Some((0.0, 1.0)));
            let id = topo.add_edge(edge);
            let ids = HashSet::from([id]);
            let matrix = Mat4::translation(4.5, 0.0, 0.0);
            let certificates = translation_edge_certificates(&topo, &ids, &matrix).unwrap();
            assert_eq!(certificates.len(), usize::from(valid_source));
            topo.vertex_mut(a).unwrap().set_point(matrix.mul_point(p));
            topo.vertex_mut(b).unwrap().set_point(matrix.mul_point(q));
            transform_edges(&mut topo, &ids, &matrix).unwrap();
            if corrupt_after {
                let point = topo.vertex(a).unwrap().point();
                topo.vertex_mut(a)
                    .unwrap()
                    .set_point(Point3::new(point.x(), 0.001, point.z()));
            }
            restore_translation_certificates(&mut topo, certificates).unwrap();
            let edge = topo.edge(id).unwrap();
            let p = topo.vertex(a).unwrap().point();
            let q = topo.vertex(b).unwrap().point();
            let residual = (edge.curve().evaluate_with_endpoints(0.0, p, q) - p).length();
            assert!(residual > gap, "fixture must expose translation roundoff");
            let actual = edge.effective_tolerance(1e-7);
            if valid_source && !corrupt_after {
                assert!(residual <= actual);
                // The carried tolerance keeps the coordinate-scale budget as
                // slack for cross-platform re-evaluation (#483), so it sits
                // ~1e-13 above the gap here — budget-scale, not residual-ulp
                // scale, yet still tiny relative to the tolerance itself.
                assert!(
                    actual - gap < 1e-12,
                    "carried tolerance must stay budget-scale, got {}",
                    actual - gap
                );
            } else {
                assert_eq!(actual, tolerance);
                assert!(residual > actual);
            }
        }
    }
}
