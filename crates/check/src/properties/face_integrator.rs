//! Per-face Gauss quadrature integration for area, volume, CoM, and inertia.
//!
//! Provides numerical integration of geometric properties over individual
//! faces. Planar faces use polygon fan triangulation; parametric faces
//! (cylinder, cone, sphere, torus, NURBS) use tensor-product Gauss-Legendre
//! quadrature over the UV domain.

use brepkit_math::quadrature::gauss_legendre_points;
use brepkit_math::traits::ParametricSurface;
use brepkit_math::vec::{Point2, Point3, Vec3};
use brepkit_topology::Topology;
use brepkit_topology::edge::EdgeCurve;
use brepkit_topology::face::{FaceId, FaceSurface};

use crate::CheckError;

/// Contribution of a single face to global geometric properties.
#[derive(Debug, Clone)]
pub struct FaceContribution {
    /// Face area.
    pub area: f64,
    /// Volume contribution: (1/3) integral of P dot N dA.
    pub volume: f64,
    /// Volume-weighted x-moment: (1/2) integral of x^2 * n_x dA (divergence theorem).
    pub volume_moment_x: f64,
    /// Volume-weighted y-moment: (1/2) integral of y^2 * n_y dA (divergence theorem).
    pub volume_moment_y: f64,
    /// Volume-weighted z-moment: (1/2) integral of z^2 * n_z dA (divergence theorem).
    pub volume_moment_z: f64,
    /// Raw volume integral of `x²` about the global origin.
    pub volume_second_x: f64,
    /// Raw volume integral of `y²` about the global origin.
    pub volume_second_y: f64,
    /// Raw volume integral of `z²` about the global origin.
    pub volume_second_z: f64,
    /// Raw volume integral of `xy` about the global origin.
    pub volume_product_xy: f64,
    /// Raw volume integral of `xz` about the global origin.
    pub volume_product_xz: f64,
    /// Raw volume integral of `yz` about the global origin.
    pub volume_product_yz: f64,
    /// Area-weighted centroid x-component (for surface centroid, not solid CoM).
    pub centroid_x: f64,
    /// Area-weighted centroid y-component (for surface centroid, not solid CoM).
    pub centroid_y: f64,
    /// Area-weighted centroid z-component (for surface centroid, not solid CoM).
    pub centroid_z: f64,
}

/// Integrate a face's geometric contribution using Gauss quadrature.
///
/// For planar faces, evaluates via polygon fan triangulation. For
/// parametric surfaces (analytic and NURBS), evaluates the surface and its
/// partial derivatives on a Gauss-point grid over the UV domain derived
/// from the face's boundary vertices.
///
/// # Accuracy
///
/// * **Planar faces** bounded entirely by lines and circular arcs are exact:
///   `integrate_planar_face_exact` integrates the boundary in closed form by
///   Green's theorem, holes included. Any other edge type on any of the face's
///   wires drops the whole face to the chord-polygon fan path, which
///   under-counts a circular cap by its sagitta area.
/// * **Quadric faces** that span a full revolution, or whose boundary does not
///   trim the analytic domain, are integrated over that domain by composite
///   Gauss quadrature, converged to machine precision at the default order.
/// * **Quadric faces trimmed by a UV boundary polygon** are limited by the
///   chording of that polygon.
///
/// Inner wires ARE removed from a curved face, in the same UV domain the
/// quadrature runs over, and the quadrature is split on their outlines so each
/// subtracts to the accuracy of its own chording rather than to however many
/// abscissae happen to land inside it.
///
/// A hole that CLOSES in `u` is a patch, and rejects what it encloses. A hole
/// that WRAPS the periodic `u` axis of a cylinder or a cone encloses no patch —
/// it separates the wall into bands — so what it rejects is what an odd number
/// of such loops lie above; the two rims a cross-drilled bore opens in a wall
/// are one such pair. A wrapping hole that instead CAPS the wall, leaving the
/// outer wire at a single `v`, clips the `v` range rather than masking, as a
/// drilled tunnel's rim does on a sphere — see `full_revolution_hole_vs`. A
/// wrapping hole on a torus is still not subtracted: the tube is periodic in
/// BOTH parameters, so it has neither an "above" nor a far end to clip, and
/// callers that can meet one defer the whole solid rather than measure it here.
///
/// A face whose whole boundary is ONE closed edge has a single boundary
/// vertex, so it cannot bound its own domain out of the surface's analytic
/// one — which for a cylinder and a cone is unbounded in `v`, which made every
/// abscissa non-finite and the face's contribution exactly zero. The domain
/// comes from the boundary curves' own 3D extent instead; see
/// `face_boundary_v_extent`.
///
/// # Errors
///
/// Returns an error if topology entities are missing or the face has
/// insufficient geometry for integration.
pub fn integrate_face(
    topo: &Topology,
    face_id: FaceId,
    gauss_order: usize,
) -> Result<FaceContribution, CheckError> {
    let face = topo.face(face_id)?;
    let reversed = face.is_reversed();
    let sign = if reversed { -1.0 } else { 1.0 };

    match face.surface() {
        FaceSurface::Plane { normal, .. } => {
            let effective_normal = if reversed { -*normal } else { *normal };
            integrate_planar_face(topo, face_id, effective_normal)
        }
        FaceSurface::Cylinder(s) => {
            let full = (
                (0.0, std::f64::consts::TAU),
                face_boundary_v_extent(topo, face_id, s)?,
            );
            let (u_range, v_range) = face_uv_bounds(topo, face_id, s, true, false, full)?;
            let uv = build_face_uv(topo, face_id, |p| s.project_point(p), true, true)?;
            Ok(integrate_with_trimming(
                s,
                u_range,
                v_range,
                gauss_order,
                sign,
                &uv,
                PatchScale::ANGULAR,
            ))
        }
        FaceSurface::Cone(s) => {
            let full = (
                (0.0, std::f64::consts::TAU),
                face_boundary_v_extent(topo, face_id, s)?,
            );
            let (u_range, v_range) = face_uv_bounds(topo, face_id, s, true, false, full)?;
            let uv = build_face_uv(topo, face_id, |p| s.project_point(p), true, true)?;
            Ok(integrate_with_trimming(
                s,
                u_range,
                v_range,
                gauss_order,
                sign,
                &uv,
                PatchScale::ANGULAR,
            ))
        }
        FaceSurface::Sphere(s) => {
            let full = (
                (0.0, std::f64::consts::TAU),
                (-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2),
            );
            let (u_range, v_range) = face_uv_bounds(topo, face_id, s, true, false, full)?;
            let mut uv = build_face_uv(topo, face_id, |p| s.project_point(p), true, false)?;
            uv.hole_vs = full_revolution_hole_vs(topo, face_id, s);
            Ok(integrate_with_trimming(
                s,
                u_range,
                v_range,
                gauss_order,
                sign,
                &uv,
                PatchScale::ANGULAR,
            ))
        }
        FaceSurface::Torus(s) => {
            let full = ((0.0, std::f64::consts::TAU), (0.0, std::f64::consts::TAU));
            let (u_range, v_range) = face_uv_bounds(topo, face_id, s, true, true, full)?;
            let uv = build_face_uv(topo, face_id, |p| s.project_point(p), true, false)?;
            Ok(integrate_with_trimming(
                s,
                u_range,
                v_range,
                gauss_order,
                sign,
                &uv,
                PatchScale::ANGULAR,
            ))
        }
        FaceSurface::Nurbs(s) => {
            let full = (s.domain_u(), s.domain_v());
            let periodic_u = s.is_periodic_u();
            let periodic_v = s.is_periodic_v();
            let (u_range, v_range) =
                face_uv_bounds(topo, face_id, s, periodic_u, periodic_v, full)?;
            let uv = build_face_uv(topo, face_id, |p| s.project_point(p), periodic_u, false)?;
            Ok(integrate_with_trimming(
                s,
                u_range,
                v_range,
                gauss_order,
                sign,
                &uv,
                PatchScale {
                    u: knot_axis_patch_scale(s.knots_u(), s.domain_u()),
                    v: knot_axis_patch_scale(s.knots_v(), s.domain_v()),
                },
            ))
        }
    }
}

/// UV domain bounds as `((u_min, u_max), (v_min, v_max))`.
type UvBounds = ((f64, f64), (f64, f64));

/// Samples taken along each boundary edge when measuring a face's own extent.
const EXTENT_SAMPLES: usize = 8;

/// Winding of a `u` sequence within this much of a full turn counts as
/// wrapping the periodic axis.
const WRAP_EPS: f64 = 1e-3;

/// Shoelace area below which a UV loop encloses no patch — it has collapsed
/// onto a line or a point, or it wraps the periodic axis instead of closing.
const DEGENERATE_UV_AREA: f64 = 1e-12;

/// Samples a curved edge contributes when a face's boundary or one of its
/// holes is outlined in UV.
///
/// The quadrature is exact for the polylines these outlines are, so the whole
/// residual of a trimmed curved face is their chord error. That is worth more
/// samples than the `crate::util::CLOSED_CURVE_SAMPLES` a wire gets for merely
/// being walked: the chord error falls with the square of the step, so four
/// times the default buys a factor of sixteen. On a cross-drilled shaft it
/// takes the body from 0.06 % of its closed form to 0.005 %.
///
/// It applies to OPEN arcs as well as closed circles. An arc bows away from
/// the chord between its endpoints just as a circle does, and a face can be
/// bounded by nothing else: a rolling-ball corner patch is three quarter great
/// circles, and outlining it by its three vertices alone reads its area 25 %
/// low.
const TRIM_SAMPLES: usize = 128;

/// Maximum number of UV vertices a face may contribute to trim quadrature.
///
/// Integration repeatedly intersects every sampled loop at every `u` split,
/// so allowing attacker-controlled topology to grow both dimensions without a
/// bound makes property measurement an algorithmic-complexity denial of
/// service. This ceiling still accommodates detailed production trims while
/// placing a hard upper bound on the superlinear portion of the work.
const MAX_TRIM_POINTS: usize = 4096;

/// The `v` extent a face's own boundary curves cover on its surface.
///
/// A cylinder's and a cone's analytic domain is unbounded in `v`, so a face
/// whose projected boundary fails to bound a sub-region — every vertex landing
/// on one `(u, v)`, which is what a wall closed by a single closed edge does —
/// has no finite domain to fall back on, and integrating an infinite patch
/// makes every abscissa non-finite and the face's contribution NaN. Its
/// boundary curves themselves always have a finite extent, and it is the only
/// extent the face can cover, so that is the fallback. Every wire is sampled
/// along each edge's own span, not just at its vertices, because a single
/// closed edge has one vertex and an arc bows away from the chord between two.
///
/// A boundary that really does collapse to one `v` yields a zero-width range,
/// and the face then contributes nothing — which is what a face with no axial
/// extent should contribute.
fn face_boundary_v_extent<S: ParametricSurface>(
    topo: &Topology,
    face_id: FaceId,
    surface: &S,
) -> Result<(f64, f64), CheckError> {
    let face = topo.face(face_id)?;
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;

    for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        let wire = topo.wire(wid)?;
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            let start = topo.vertex(edge.start())?.point();
            let end = topo.vertex(edge.end())?.point();
            let (t0, t1) = edge.curve().domain_with_endpoints(start, end);
            for k in 0..=EXTENT_SAMPLES {
                #[allow(clippy::cast_precision_loss)]
                let f = k as f64 / EXTENT_SAMPLES as f64;
                let p = edge
                    .curve()
                    .evaluate_with_endpoints((t1 - t0).mul_add(f, t0), start, end);
                let (_, v) = surface.project_point(p);
                v_min = v_min.min(v);
                v_max = v_max.max(v);
            }
        }
    }

    if v_min.is_finite() && v_max.is_finite() {
        Ok((v_min, v_max))
    } else {
        Err(CheckError::IntegrationFailed(
            "face boundary has no finite extent on its surface".into(),
        ))
    }
}

/// A face boundary loop projected into its surface's UV domain.
///
/// `u` is stored unwrapped — consecutive samples differ by less than half a
/// period — so a loop straddling the seam stays contiguous instead of splitting
/// in two. The window it was unwrapped into is remembered, because a quadrature
/// abscissa has to be brought into the same branch before it can be tested
/// against the loop.
#[derive(Debug, Clone, Default)]
struct UvLoop {
    points: Vec<Point2>,
    u_center: f64,
}

impl UvLoop {
    /// Wrap a projected boundary into a loop, unwrapping `u` sequentially.
    fn new(mut points: Vec<Point2>, u_periodic: bool) -> Self {
        if u_periodic {
            for i in 1..points.len() {
                let u = unwrap_angle(points[i - 1].x(), points[i].x());
                points[i] = Point2::new(u, points[i].y());
            }
        }
        let u_min = points.iter().map(|p| p.x()).fold(f64::INFINITY, f64::min);
        let u_max = points
            .iter()
            .map(|p| p.x())
            .fold(f64::NEG_INFINITY, f64::max);
        let u_center = if points.is_empty() {
            0.0
        } else {
            f64::midpoint(u_min, u_max)
        };
        Self { points, u_center }
    }

    /// Absolute shoelace area. Near zero means the loop has collapsed onto a
    /// line or a point, or it wraps the periodic axis rather than closing.
    fn area(&self) -> f64 {
        let n = self.points.len();
        if n < 3 {
            return 0.0;
        }
        let mut a = 0.0;
        for i in 0..n {
            let p = self.points[i];
            let q = self.points[(i + 1) % n];
            a += p.x() * q.y() - q.x() * p.y();
        }
        (a * 0.5).abs()
    }

    /// Signed total of the shortest `u` steps around the loop: `±2π` when it
    /// wraps the periodic axis once, `~0` when it closes without wrapping.
    ///
    /// Taking each step by its shortest representative makes the total
    /// independent of how finely the loop was sampled.
    fn u_winding(&self) -> f64 {
        let tau = std::f64::consts::TAU;
        let n = self.points.len();
        (0..n)
            .map(|i| {
                let d = self.points[(i + 1) % n].x() - self.points[i].x();
                d - tau * ((d + std::f64::consts::PI) / tau).floor()
            })
            .sum()
    }

    /// `u` brought into the branch this loop's own coordinates live in.
    fn wrap_u(&self, u: f64, u_periodic: bool) -> f64 {
        if u_periodic {
            unwrap_angle(self.u_center, u)
        } else {
            u
        }
    }

    /// Whether `(u, v)` lies inside the patch this loop encloses.
    fn encloses(&self, u: f64, v: f64, u_periodic: bool) -> bool {
        use brepkit_math::predicates::point_in_polygon;
        point_in_polygon(Point2::new(self.wrap_u(u, u_periodic), v), &self.points)
    }

    /// Call `f` with the `v` of every point where the vertical line at `u`
    /// crosses this loop.
    ///
    /// `wraps` selects how the loop closes: a patch loop closes by a chord
    /// from its last sample back to its first, a period-wrapping loop by the
    /// step to its first sample one whole turn on. Each segment is taken
    /// half-open in `u` so a shared endpoint is reported once.
    fn for_each_v_crossing(&self, u: f64, u_periodic: bool, wraps: bool, mut f: impl FnMut(f64)) {
        let tau = std::f64::consts::TAU;
        let n = self.points.len();
        if n < 2 {
            return;
        }
        let uq = if wraps {
            let u0 = self.points[0].x();
            let d = u - u0;
            u0 + d - tau * (d / tau).floor()
        } else {
            self.wrap_u(u, u_periodic)
        };

        for i in 0..n {
            let a = self.points[i];
            let b = if i + 1 < n {
                self.points[i + 1]
            } else if wraps {
                Point2::new(self.points[0].x() + tau, self.points[0].y())
            } else {
                self.points[0]
            };
            let (lo, hi) = if a.x() <= b.x() {
                (a.x(), b.x())
            } else {
                (b.x(), a.x())
            };
            if uq >= lo && uq < hi {
                let t = (uq - a.x()) / (b.x() - a.x());
                f((b.y() - a.y()).mul_add(t, a.y()));
            }
        }
    }

    /// Reverse the point order so the loop's net `u` step is positive.
    fn oriented_along_u(mut self) -> Self {
        if self.u_winding() < 0.0 {
            self.points.reverse();
        }
        self
    }

    /// How many strands of a `u`-wrapping loop lie above `v` at abscissa `u`.
    ///
    /// A loop that wraps the period encloses no patch — it separates the wall
    /// into bands — so it is counted rather than tested for containment. There
    /// is no closing chord: the last sample joins the first one whole turn on,
    /// which is what makes the count the number of times the curve is above the
    /// abscissa rather than a polygon crossing number.
    ///
    /// The loop must already be [`Self::oriented_along_u`].
    fn strands_above(&self, u: f64, v: f64) -> usize {
        let mut count = 0;
        self.for_each_v_crossing(u, true, true, |vc| {
            if vc > v {
                count += 1;
            }
        });
        count
    }
}

/// A face's boundary and holes in its surface's UV domain.
#[derive(Debug, Clone, Default)]
struct FaceUv {
    /// The outer wire.
    boundary: UvLoop,
    /// Inner wires that enclose a patch: a hole in the ordinary sense.
    pockets: Vec<UvLoop>,
    /// Inner wires that wrap the periodic `u` axis of a surface whose `v` runs
    /// to infinity. Such a loop bounds a band, not a patch.
    bands: Vec<UvLoop>,
    /// Whether `u` is periodic on this surface.
    u_periodic: bool,
    /// `v` positions of full-revolution constant-`v` holes, which clip the
    /// integration range instead of masking abscissae (see
    /// [`full_revolution_hole_vs`]).
    hole_vs: Vec<f64>,
}

/// Project a face's wires into the surface's UV domain.
///
/// `v_unbounded` says the surface's `v` runs to infinity in both directions (a
/// cylinder or a cone). That is what makes the band test well posed: with no
/// far end to the wall, a sample that no band lies above is material, so
/// counting bands above it decides the sample. A sphere's `v` ends at a pole
/// and a torus's wraps, so a period-wrapping hole on those is left to
/// [`full_revolution_hole_vs`] or to the caller.
fn build_face_uv<F>(
    topo: &Topology,
    face_id: FaceId,
    project: F,
    u_periodic: bool,
    v_unbounded: bool,
) -> Result<FaceUv, CheckError>
where
    F: Fn(Point3) -> (f64, f64),
{
    let to_loop = |pts: &[Point3]| {
        UvLoop::new(
            pts.iter()
                .map(|&p| {
                    let (u, v) = project(p);
                    Point2::new(u, v)
                })
                .collect(),
            u_periodic,
        )
    };

    let face = topo.face(face_id)?;
    let mut trim_points = 0_usize;
    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        let wire = topo.wire(wire_id)?;
        for oriented_edge in wire.edges() {
            let edge = topo.edge(oriented_edge.edge())?;
            let samples = if matches!(edge.curve(), EdgeCurve::Line) {
                1
            } else {
                TRIM_SAMPLES
            };
            trim_points = trim_points.saturating_add(samples);
            if trim_points > MAX_TRIM_POINTS {
                return Err(CheckError::IntegrationFailed(format!(
                    "face trim exceeds the {MAX_TRIM_POINTS}-point integration budget"
                )));
            }
        }
    }
    let outer = crate::util::wire_polygon_curve_sampled(
        topo,
        face.outer_wire(),
        TRIM_SAMPLES,
        TRIM_SAMPLES,
    )?;
    let boundary = if outer.len() < 3 {
        UvLoop::default()
    } else {
        to_loop(&outer)
    };

    let mut pockets = Vec::new();
    let mut bands = Vec::new();
    for hole in
        crate::util::face_hole_polygons_curve_sampled(topo, face_id, TRIM_SAMPLES, TRIM_SAMPLES)?
    {
        let loop_ = to_loop(&hole);
        let wraps = u_periodic && loop_.u_winding().abs() >= std::f64::consts::TAU - WRAP_EPS;
        if wraps {
            if v_unbounded {
                bands.push(loop_.oriented_along_u());
            }
        } else if loop_.area() > DEGENERATE_UV_AREA {
            pockets.push(loop_);
        }
    }
    // A face is its outer wire minus its holes; keeping the face's own wires
    // is the only thing that keeps the two in step.
    debug_assert!(face.inner_wires().len() >= pockets.len() + bands.len());

    Ok(FaceUv {
        boundary,
        pockets,
        bands,
        u_periodic,
        hole_vs: Vec::new(),
    })
}

/// Where a quadrature abscissa must lie to belong to the face.
struct UvTrim<'a> {
    /// Boundary the abscissa must be inside. `None` when the projected
    /// boundary cannot trim — it wraps the full period, or collapses onto a
    /// seam or a pole — and the integration domain is the face.
    outer: Option<&'a UvLoop>,
    /// Holes enclosing a patch: the abscissa is out when inside one.
    pockets: &'a [UvLoop],
    /// Holes wrapping the periodic axis: the abscissa is out when an odd
    /// number of them lie above it.
    bands: &'a [UvLoop],
    /// Whether `u` is periodic on this surface.
    u_periodic: bool,
}

impl<'a> UvTrim<'a> {
    /// Keep the whole domain but remove `uv`'s holes.
    fn holes_of(uv: &'a FaceUv) -> Self {
        Self {
            outer: None,
            pockets: &uv.pockets,
            bands: &uv.bands,
            u_periodic: uv.u_periodic,
        }
    }

    /// Keep the whole domain and remove only the holes that enclose a patch.
    ///
    /// For a wall whose outer wire sits at ONE `v`, the wrapping holes are what
    /// cap it: they have already set the `v` range being integrated over (see
    /// [`full_revolution_hole_vs`]), and counting them again as bands would
    /// reject the very strip they bound.
    fn pockets_of(uv: &'a FaceUv) -> Self {
        Self {
            outer: None,
            pockets: &uv.pockets,
            bands: &[],
            u_periodic: uv.u_periodic,
        }
    }

    /// Trim to `uv`'s boundary and remove its holes.
    fn boundary_of(uv: &'a FaceUv) -> Self {
        Self {
            outer: Some(&uv.boundary),
            pockets: &uv.pockets,
            bands: &uv.bands,
            u_periodic: uv.u_periodic,
        }
    }

    fn accepts(&self, u: f64, v: f64) -> bool {
        if let Some(outer) = self.outer
            && !outer.encloses(u, v, self.u_periodic)
        {
            return false;
        }
        if self
            .pockets
            .iter()
            .any(|hole| hole.encloses(u, v, self.u_periodic))
        {
            return false;
        }
        let strands: usize = self.bands.iter().map(|b| b.strands_above(u, v)).sum();
        strands.is_multiple_of(2)
    }

    /// Whether any loop cuts the domain, so the quadrature must be split on
    /// its crossings rather than merely reject abscissae.
    const fn splits_domain(&self) -> bool {
        self.outer.is_some() || !self.pockets.is_empty() || !self.bands.is_empty()
    }

    /// The `u` values at which the accepted v-spans stop varying smoothly.
    ///
    /// A loop's UV outline is a polyline, so the `v` it cuts at moves affinely
    /// with `u` between two of its vertices but kinks at them, and a span
    /// appears or vanishes where a loop's `u` extremum passes. Splitting the
    /// v-quadrature is not enough on its own — the u-integrand still has those
    /// kinks — so every loop vertex becomes a quadrature interval boundary too,
    /// which makes the subtraction exact for the polyline the loop is.
    ///
    /// Returned sorted, spanning `u_range` inclusive.
    fn u_breaks(&self, u_range: (f64, f64)) -> Vec<f64> {
        let tau = std::f64::consts::TAU;
        let (lo, hi) = (u_range.0, u_range.1);
        let mut breaks = vec![lo, hi];
        let loops = self.outer.into_iter().chain(self.pockets).chain(self.bands);
        for l in loops {
            for p in &l.points {
                // A loop unwrapped about its own window may sit a whole period
                // away from the domain's; try both shifts and keep what lands
                // inside.
                let shifts: &[f64] = if self.u_periodic {
                    &[0.0, tau, -tau]
                } else {
                    &[0.0]
                };
                for &shift in shifts {
                    let u = p.x() + shift;
                    if u > lo && u < hi {
                        breaks.push(u);
                    }
                }
            }
        }
        breaks.sort_by(f64::total_cmp);
        breaks
    }

    /// The sub-intervals of `v_range` that belong to the face at abscissa `u`.
    ///
    /// Every loop is cut by the vertical line at `u`; between two consecutive
    /// cuts no loop crosses, so [`Self::accepts`] cannot change there and the
    /// integrand is smooth across the whole interval. Splitting the
    /// v-quadrature on those cuts is what makes a hole subtract to the accuracy
    /// of its own boundary chording, rather than to however many abscissae
    /// happen to land inside it — masking alone left a 1 mm² hole in a 300 mm²
    /// wall 8 % wrong at the default order.
    fn v_spans(&self, u: f64, v_range: (f64, f64)) -> Vec<(f64, f64)> {
        let (v0, v1) = if v_range.0 <= v_range.1 {
            v_range
        } else {
            (v_range.1, v_range.0)
        };

        let mut cuts: Vec<f64> = vec![v0, v1];
        if let Some(outer) = self.outer {
            outer.for_each_v_crossing(u, self.u_periodic, false, |vc| cuts.push(vc));
        }
        for hole in self.pockets {
            hole.for_each_v_crossing(u, self.u_periodic, false, |vc| cuts.push(vc));
        }
        for band in self.bands {
            band.for_each_v_crossing(u, self.u_periodic, true, |vc| cuts.push(vc));
        }
        cuts.retain(|c| c.is_finite() && *c >= v0 && *c <= v1);
        cuts.sort_by(f64::total_cmp);

        // Cuts closer together than this carry no width worth integrating, and
        // splitting on them would only manufacture empty spans.
        let eps = (v1 - v0).abs() * 1e-12;
        let mut spans: Vec<(f64, f64)> = Vec::new();
        for w in cuts.windows(2) {
            let (a, b) = (w[0], w[1]);
            if b - a <= eps || !self.accepts(u, f64::midpoint(a, b)) {
                continue;
            }
            match spans.last_mut() {
                Some(last) if (a - last.1).abs() <= eps => last.1 = b,
                _ => spans.push((a, b)),
            }
        }
        spans
    }
}

/// The v-positions of a face's full-revolution inner wires (holes) on a
/// surface periodic in u.
///
/// A boolean that drills a cylinder through a sphere leaves each spherical
/// band bounded by a latitude circle hole (the tunnel rim). Such a hole wraps
/// the full u-period and sits at a single v, so the band runs from its outer
/// latitude to the hole — not on to the pole. Collecting these lets the
/// integrator clip the band instead of over-integrating the polar cap the hole
/// removed. Each entry is the mean projected v of one full-revolution hole.
fn full_revolution_hole_vs<S: ParametricSurface>(
    topo: &Topology,
    face_id: FaceId,
    surface: &S,
) -> Vec<f64> {
    use std::f64::consts::TAU;
    let Ok(face) = topo.face(face_id) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for &wid in face.inner_wires() {
        let Ok(wire) = topo.wire(wid) else { continue };
        let mut us = Vec::new();
        let mut vs = Vec::new();
        for oe in wire.edges() {
            let Ok(edge) = topo.edge(oe.edge()) else {
                continue;
            };
            // Oriented traversal: the wire-ordered start vertex is the edge's
            // end when the oriented edge is reversed.
            let vid = if oe.is_forward() {
                edge.start()
            } else {
                edge.end()
            };
            let Ok(v) = topo.vertex(vid) else {
                continue;
            };
            let (u, vv) = surface.project_point(v.point());
            us.push(u);
            vs.push(vv);
        }
        if vs.is_empty() {
            continue;
        }
        let v_min = vs.iter().copied().fold(f64::INFINITY, f64::min);
        let v_max = vs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        // Constant-v latitude circle.
        if v_max - v_min > 1e-6 {
            continue;
        }
        // Full revolution in u: the unwrapped per-vertex deltas around the
        // CLOSED loop (including the closing step back to the first vertex) sum
        // to ≈ TAU. A single-edge closed circle has one vertex, so also accept
        // holes whose sole edge is a closed circle curve.
        let unwrapped_span = {
            let n = us.len();
            let mut acc = 0.0;
            for i in 0..n {
                let d = us[(i + 1) % n] - us[i];
                acc += d - TAU * ((d + std::f64::consts::PI) / TAU).floor();
            }
            acc.abs()
        };
        let single_closed_circle = wire.edges().len() == 1
            && wire.edges().first().is_some_and(|oe| {
                topo.edge(oe.edge())
                    .is_ok_and(|e| matches!(e.curve(), EdgeCurve::Circle(_)))
            });
        if unwrapped_span >= TAU - 1e-3 || single_closed_circle {
            out.push(0.5 * (v_min + v_max));
        }
    }
    out
}

/// Compute UV bounds for a parametric face by projecting boundary vertices
/// onto the surface and taking the min/max of the resulting parameters.
///
/// For surfaces with periodic u or v coordinates (cylinders, cones, spheres,
/// tori), sequentially unwraps the angular coordinates so that faces straddling
/// the 0/2pi seam produce correct ranges.
///
/// When all projected vertices coincide (e.g. a full-revolution face),
/// `full_domain` is returned instead.
///
/// Only the outer wire bounds the domain — a hole lies inside it by
/// definition, so it can never widen it. The holes are removed from the
/// integration itself, by [`UvTrim`].
///
/// `full_domain` must be finite on both axes. A cylinder's and a cone's
/// analytic domain is not, so those pass their face's own boundary extent
/// (see [`face_boundary_v_extent`]) rather than `±∞`.
fn face_uv_bounds<S: ParametricSurface>(
    topo: &Topology,
    face_id: FaceId,
    surface: &S,
    periodic_u: bool,
    periodic_v: bool,
    full_domain: UvBounds,
) -> Result<UvBounds, CheckError> {
    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;

    let mut uvs = Vec::new();
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let vid = oe.oriented_start(edge);
        let pt = topo.vertex(vid)?.point();
        uvs.push(surface.project_point(pt));
    }

    if uvs.is_empty() {
        return Err(CheckError::IntegrationFailed(
            "face wire has no edges".into(),
        ));
    }

    // Unwrap periodic coordinates sequentially so seam-straddling faces
    // produce a contiguous range instead of the full [0, 2pi).
    if periodic_u || periodic_v {
        for i in 1..uvs.len() {
            if periodic_u {
                uvs[i].0 = unwrap_angle(uvs[i - 1].0, uvs[i].0);
            }
            if periodic_v {
                uvs[i].1 = unwrap_angle(uvs[i - 1].1, uvs[i].1);
            }
        }
    }

    // Check for coincident vertices (all project to same point) — use full domain.
    let coincident = uvs.len() < 3 || {
        let ref_uv = uvs[0];
        uvs.iter()
            .all(|uv| (uv.0 - ref_uv.0).abs() < 1e-6 && (uv.1 - ref_uv.1).abs() < 1e-6)
    };
    if coincident {
        return Ok(full_domain);
    }

    let u_min = uvs.iter().map(|uv| uv.0).fold(f64::INFINITY, f64::min);
    let mut u_max = uvs.iter().map(|uv| uv.0).fold(f64::NEG_INFINITY, f64::max);
    let v_min = uvs.iter().map(|uv| uv.1).fold(f64::INFINITY, f64::min);
    let mut v_max = uvs.iter().map(|uv| uv.1).fold(f64::NEG_INFINITY, f64::max);

    // All boundary vertices on the seam of a periodic axis (e.g. a
    // full-revolution lateral face whose circles start/end at the seam)
    // collapse that axis's range to zero — the face actually spans the
    // full period.
    if periodic_u && u_max - u_min < 1e-9 {
        u_max = u_min + (full_domain.0.1 - full_domain.0.0);
    }
    if periodic_v && v_max - v_min < 1e-9 {
        v_max = v_min + (full_domain.1.1 - full_domain.1.0);
    }

    if u_min >= u_max || v_min >= v_max {
        // A degenerate projection (e.g. all boundary vertices on a sphere's
        // pole seam) does not mean an empty face — it means the boundary failed
        // to bound a sub-region, so the face spans the full analytic domain.
        return Ok(full_domain);
    }

    Ok(((u_min, u_max), (v_min, v_max)))
}

/// Unwrap a step in a periodic (angular) coordinate to avoid discontinuities.
///
/// Adjusts `next` so that `next - prev` lies in `(-pi, pi]`, keeping the
/// sequence monotonic through the 0/2pi seam.
fn unwrap_angle(prev: f64, next: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let diff = next - prev;
    prev + diff - tau * ((diff + std::f64::consts::PI) / tau).floor()
}

/// Integrate a planar face using polygon fan triangulation.
///
/// Inner wires (holes) are integrated the same way and subtracted from the
/// outer-wire contribution.
fn integrate_planar_face(
    topo: &Topology,
    face_id: FaceId,
    normal: Vec3,
) -> Result<FaceContribution, CheckError> {
    let normal = normal.normalize().map_err(|_| {
        CheckError::IntegrationFailed("planar face has a zero or non-finite normal".into())
    })?;
    // Faces whose wires consist only of line and circular-arc edges take an
    // exact Green's-theorem boundary-integral path — a chord polygon
    // undercounts a circular cap by the sagitta area (~0.2% at the default
    // discretization), far above the accuracy of the parametric quadrature
    // the curved faces get.
    if let Some(contrib) = integrate_planar_face_exact(topo, face_id, normal)? {
        return Ok(contrib);
    }
    let polygon = crate::util::face_polygon(topo, face_id)?;
    let mut contrib = integrate_planar_polygon(&polygon, normal);

    let face = topo.face(face_id)?;
    let inner: Vec<_> = face.inner_wires().to_vec();
    for wid in inner {
        let hole = crate::util::wire_polygon(topo, wid)?;
        let h = integrate_planar_polygon(&hole, normal);
        contrib.area -= h.area;
        contrib.volume -= h.volume;
        contrib.volume_moment_x -= h.volume_moment_x;
        contrib.volume_moment_y -= h.volume_moment_y;
        contrib.volume_moment_z -= h.volume_moment_z;
        contrib.volume_second_x -= h.volume_second_x;
        contrib.volume_second_y -= h.volume_second_y;
        contrib.volume_second_z -= h.volume_second_z;
        contrib.volume_product_xy -= h.volume_product_xy;
        contrib.volume_product_xz -= h.volume_product_xz;
        contrib.volume_product_yz -= h.volume_product_yz;
        contrib.centroid_x -= h.centroid_x;
        contrib.centroid_y -= h.centroid_y;
        contrib.centroid_z -= h.centroid_z;
    }

    Ok(contrib)
}

/// Integrate a planar polygon's contribution via fan triangulation.
fn integrate_planar_polygon(polygon: &[Point3], normal: Vec3) -> FaceContribution {
    if polygon.len() < 3 {
        return FaceContribution {
            area: 0.0,
            volume: 0.0,
            volume_moment_x: 0.0,
            volume_moment_y: 0.0,
            volume_moment_z: 0.0,
            volume_second_x: 0.0,
            volume_second_y: 0.0,
            volume_second_z: 0.0,
            volume_product_xy: 0.0,
            volume_product_xz: 0.0,
            volume_product_yz: 0.0,
            centroid_x: 0.0,
            centroid_y: 0.0,
            centroid_z: 0.0,
        };
    }

    // Fan triangulation from vertex 0 with signed triangle areas. For a
    // nonconvex polygon, triangles that cross a notch must cancel instead of
    // adding their absolute area.
    let mut area = 0.0;
    let mut vol = 0.0;
    let mut mx = 0.0;
    let mut my = 0.0;
    let mut mz = 0.0;
    let mut qxx = 0.0;
    let mut qyy = 0.0;
    let mut qzz = 0.0;
    let mut qxy = 0.0;
    let mut qxz = 0.0;
    let mut qyz = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;
    let mut cz = 0.0;

    for i in 1..polygon.len() - 1 {
        let (a, b, c) = (polygon[0], polygon[i], polygon[i + 1]);
        let ab = b - a;
        let ac = c - a;
        let cross = Vec3::new(
            ab.y() * ac.z() - ab.z() * ac.y(),
            ab.z() * ac.x() - ab.x() * ac.z(),
            ab.x() * ac.y() - ab.y() * ac.x(),
        );
        let tri_area = cross.dot(normal) * 0.5;
        let tri_sign = tri_area.signum();
        area += tri_area;

        // Volume contribution: (1/3) * centroid dot normal * area
        let centroid = Point3::new(
            (a.x() + b.x() + c.x()) / 3.0,
            (a.y() + b.y() + c.y()) / 3.0,
            (a.z() + b.z() + c.z()) / 3.0,
        );
        let pv = Vec3::new(centroid.x(), centroid.y(), centroid.z());
        vol += pv.dot(normal) * tri_area / 3.0;

        // Volume moments via divergence theorem: (1/2) integral of x^2 * n_x dA
        // For a planar triangle with constant normal, integral of x^2 over triangle
        // = (area/3) * (x_a^2 + x_b^2 + x_c^2 + x_a*x_b + x_a*x_c + x_b*x_c) / 2
        // Simplified: use (x_a^2 + x_b^2 + x_c^2 + x_a*x_b + x_a*x_c + x_b*x_c)/6
        let avg_x2 = (a.x() * a.x()
            + b.x() * b.x()
            + c.x() * c.x()
            + a.x() * b.x()
            + a.x() * c.x()
            + b.x() * c.x())
            / 6.0;
        let avg_y2 = (a.y() * a.y()
            + b.y() * b.y()
            + c.y() * c.y()
            + a.y() * b.y()
            + a.y() * c.y()
            + b.y() * c.y())
            / 6.0;
        let avg_z2 = (a.z() * a.z()
            + b.z() * b.z()
            + c.z() * c.z()
            + a.z() * b.z()
            + a.z() * c.z()
            + b.z() * c.z())
            / 6.0;
        mx += 0.5 * avg_x2 * normal.x() * tri_area;
        my += 0.5 * avg_y2 * normal.y() * tri_area;
        mz += 0.5 * avg_z2 * normal.z() * tri_area;

        // Raw second moments and products via the divergence theorem. The
        // four-point Hammer rule used here is exact for the cubic monomials.
        qxx += tri_sign * normal.x() * triangle_cubic_integral(a, b, c, |p| p.x().powi(3)) / 3.0;
        qyy += tri_sign * normal.y() * triangle_cubic_integral(a, b, c, |p| p.y().powi(3)) / 3.0;
        qzz += tri_sign * normal.z() * triangle_cubic_integral(a, b, c, |p| p.z().powi(3)) / 3.0;
        qxy += tri_sign * normal.x() * triangle_cubic_integral(a, b, c, |p| p.x().powi(2) * p.y())
            / 2.0;
        qxz += tri_sign * normal.x() * triangle_cubic_integral(a, b, c, |p| p.x().powi(2) * p.z())
            / 2.0;
        qyz += tri_sign * normal.y() * triangle_cubic_integral(a, b, c, |p| p.y().powi(2) * p.z())
            / 2.0;

        cx += centroid.x() * tri_area;
        cy += centroid.y() * tri_area;
        cz += centroid.z() * tri_area;
    }

    // A polygon wound clockwise about `normal` nets a negative signed area;
    // flip every accumulated quantity so callers keep the positive-area
    // contract while all first and second volume moments stay consistent.
    let flip = if area < 0.0 { -1.0 } else { 1.0 };
    FaceContribution {
        area: area * flip,
        volume: vol * flip,
        volume_moment_x: mx * flip,
        volume_moment_y: my * flip,
        volume_moment_z: mz * flip,
        volume_second_x: qxx * flip,
        volume_second_y: qyy * flip,
        volume_second_z: qzz * flip,
        volume_product_xy: qxy * flip,
        volume_product_xz: qxz * flip,
        volume_product_yz: qyz * flip,
        centroid_x: cx * flip,
        centroid_y: cy * flip,
        centroid_z: cz * flip,
    }
}

/// Integrate a cubic-or-lower scalar function over a triangle exactly using
/// the four-point Hammer rule.
fn triangle_cubic_integral(a: Point3, b: Point3, c: Point3, f: impl Fn(Point3) -> f64) -> f64 {
    let area = (b - a).cross(c - a).length() * 0.5;
    let barycentric = |wa: f64, wb: f64, wc: f64| {
        Point3::new(
            wa.mul_add(a.x(), wb.mul_add(b.x(), wc * c.x())),
            wa.mul_add(a.y(), wb.mul_add(b.y(), wc * c.y())),
            wa.mul_add(a.z(), wb.mul_add(b.z(), wc * c.z())),
        )
    };
    let centroid = barycentric(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);
    let value = (-27.0 / 48.0) * f(centroid)
        + (25.0 / 48.0)
            * (f(barycentric(0.6, 0.2, 0.2))
                + f(barycentric(0.2, 0.6, 0.2))
                + f(barycentric(0.2, 0.2, 0.6)));
    area * value
}

/// Monomial basis for degree-≤3 polynomials in the in-plane coordinates
/// `(s, t)`: `[1, s, t, s², st, t², s³, s²t, st², t³]`.
type Poly2 = [f64; 10];

/// Multiply a degree-≤2 polynomial by the linear form `l₀ + l₁s + l₂t`.
///
/// The caller must ensure `p` has no cubic terms (indices 6..10 zero);
/// products above degree 3 are not representable and are silently dropped.
fn poly2_mul_linear(p: &Poly2, l: [f64; 3]) -> Poly2 {
    // Shift tables: monomial index k multiplied by s (resp. t).
    const S_SHIFT: [usize; 6] = [1, 3, 4, 6, 7, 8];
    const T_SHIFT: [usize; 6] = [2, 4, 5, 7, 8, 9];
    let mut out = [0.0; 10];
    for k in 0..10 {
        out[k] += l[0] * p[k];
    }
    for k in 0..6 {
        out[S_SHIFT[k]] += l[1] * p[k];
        out[T_SHIFT[k]] += l[2] * p[k];
    }
    out
}

/// Dot a polynomial's coefficients with the region's monomial moments.
fn poly2_integrate(p: &Poly2, moments: &[f64; 10]) -> f64 {
    p.iter().zip(moments.iter()).map(|(c, m)| c * m).sum()
}

/// Region monomial moments `∫∫ sⁱtʲ ds dt` of one wire's enclosed planar
/// region via Green's theorem: `M_ij = ∮ s^{i+1}/(i+1) · tʲ dt`.
///
/// Returns `None` if the wire contains any edge that is not a line or a
/// circular arc (the exact path only handles those); the caller falls back
/// to chord-polygon integration. The result is winding-aligned so that the
/// area moment `M₀₀` is positive.
fn planar_wire_monomial_moments(
    topo: &Topology,
    wire_id: brepkit_topology::wire::WireId,
    origin: Point3,
    e1: Vec3,
    e2: Vec3,
) -> Result<Option<[f64; 10]>, CheckError> {
    let wire = topo.wire(wire_id)?;
    let mut moments = [0.0; 10];
    let mut prev_end: Option<brepkit_topology::vertex::VertexId> = None;

    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let start_vid = edge.start();
        let end_vid = edge.end();
        // Wires store edges in loop order, but per-edge orientation flags are
        // not guaranteed to chain head-to-tail; re-derive traversal direction
        // from vertex connectivity with the previous edge (same convention as
        // `util::wire_polygon`).
        let forward = match prev_end {
            Some(pe) if start_vid == pe && end_vid != pe => true,
            Some(pe) if end_vid == pe && start_vid != pe => false,
            _ => oe.is_forward(),
        };
        prev_end = Some(if forward { end_vid } else { start_vid });

        let start = topo.vertex(start_vid)?.point();
        let end = topo.vertex(end_vid)?.point();
        let dir_sign = if forward { 1.0 } else { -1.0 };

        match edge.curve() {
            EdgeCurve::Line => {
                // P(u) = start + (end - start)·u, u ∈ [0, 1].
                let d = end - start;
                accumulate_green_segment(
                    &mut moments,
                    (0.0, 1.0),
                    8,
                    dir_sign,
                    |u| (start + d * u, d),
                    origin,
                    e1,
                    e2,
                );
            }
            EdgeCurve::Circle(c) => {
                // Angular arc span from the edge's own endpoints; the
                // derivative magnitude is the radius.
                let (t0, t1) = edge.curve().domain_with_endpoints(start, end);
                let r = c.radius();
                // Split the span so each chunk is ≤ π/2; 16-point Gauss on
                // a ≤ π/2 trig span of frequency ≤ 5 is exact to machine
                // precision.
                let chunks =
                    (((t1 - t0).abs() / std::f64::consts::FRAC_PI_2).ceil() as usize).clamp(1, 8);
                let dt = (t1 - t0) / chunks as f64;
                for i in 0..chunks {
                    let a = dt.mul_add(i as f64, t0);
                    accumulate_green_segment(
                        &mut moments,
                        (a, a + dt),
                        16,
                        dir_sign,
                        |u| (c.evaluate(u), c.tangent(u) * r),
                        origin,
                        e1,
                        e2,
                    );
                }
            }
            EdgeCurve::Parabola(p) => {
                // A parabola is polynomial of degree 2 in its parameter, so
                // every moment integrand `s^{i+1}/(i+1) · t^j · t'(u)` is a
                // polynomial of degree ≤ 2·4 + 2·3 + 1 = 15. A 16-point
                // Gauss rule is exact through degree 31, so a single
                // segment integrates the arc exactly — no chunking, and no
                // dependence on the arc's extent or on model scale.
                let (t0, t1) = edge.curve().domain_with_endpoints(start, end);
                accumulate_green_segment(
                    &mut moments,
                    (t0, t1),
                    16,
                    dir_sign,
                    |u| (p.evaluate(u), p.tangent(u)),
                    origin,
                    e1,
                    e2,
                );
            }
            // Refused, not approximated. A hyperbola's integrand is
            // transcendental (cosh/sinh), so no fixed Gauss rule is exact
            // for it the way it is for circles and parabolas. Returning
            // `None` routes the whole face to the sampled fallback rather
            // than reporting a quadrature error as an exact result.
            EdgeCurve::Hyperbola(_) | EdgeCurve::Ellipse(_) | EdgeCurve::NurbsCurve(_) => {
                return Ok(None);
            }
        }
    }

    // Align winding so the enclosed area is positive.
    if moments[0] < 0.0 {
        for m in &mut moments {
            *m = -*m;
        }
    }
    Ok(Some(moments))
}

/// Accumulate one boundary segment's Green's-theorem contribution to the
/// region monomial moments with Gauss-Legendre quadrature.
///
/// `eval(u)` returns the curve point and its derivative `dP/du`; the
/// integrand for `M_ij` is `s^{i+1}/(i+1) · tʲ · t'(u)` with
/// `s = (P - origin)·e1`, `t = (P - origin)·e2`.
#[allow(clippy::too_many_arguments)]
fn accumulate_green_segment<F>(
    moments: &mut [f64; 10],
    range: (f64, f64),
    gauss_order: usize,
    dir_sign: f64,
    eval: F,
    origin: Point3,
    e1: Vec3,
    e2: Vec3,
) where
    F: Fn(f64) -> (Point3, Vec3),
{
    // Monomial exponents (i, j) matching the `Poly2` basis order.
    const EXPONENTS: [(i32, i32); 10] = [
        (0, 0),
        (1, 0),
        (0, 1),
        (2, 0),
        (1, 1),
        (0, 2),
        (3, 0),
        (2, 1),
        (1, 2),
        (0, 3),
    ];

    let scale = (range.1 - range.0) / 2.0;
    let mid = f64::midpoint(range.0, range.1);
    for gp in gauss_legendre_points(gauss_order) {
        let u = scale.mul_add(gp.x, mid);
        let (p, dp) = eval(u);
        let rel = p - origin;
        let s = rel.dot(e1);
        let t = rel.dot(e2);
        let dt_du = dp.dot(e2);
        let w = gp.w * scale * dir_sign * dt_du;
        for (k, &(i, j)) in EXPONENTS.iter().enumerate() {
            moments[k] += w * s.powi(i + 1) / f64::from(i + 1) * t.powi(j);
        }
    }
}

/// Newell normal of a wire's boundary, sampled densely enough that a wire
/// consisting of a single closed circle (one vertex) still determines its
/// plane. Returns `None` when the boundary is degenerate (collapsed to a
/// point or line) or contains an edge type the exact path does not handle.
///
/// The wire is walked in traversal order, exactly like
/// [`planar_wire_monomial_moments`] and [`crate::util::wire_polygon`]: each
/// edge emits the point the wire ARRIVES at (and, for an arc, the interior
/// samples that follow it), stopping one step short of the point it leaves
/// at, which the next edge supplies. Emitting `edge.start()` regardless of
/// traversal direction collapses any wire whose stored orientation flags
/// alternate — a four-line rectangle flagged `(fwd, rev, fwd, rev)` samples
/// as `A, B, B, A`, whose Newell normal is zero — and a zero normal here
/// rejects the exact path for the WHOLE face, holes included.
fn wire_newell_normal(
    topo: &Topology,
    wire_id: brepkit_topology::wire::WireId,
) -> Result<Option<Vec3>, CheckError> {
    /// Samples emitted per arc, enough that a wire of one closed circle
    /// (a single vertex) still spans its plane.
    const ARC_SAMPLES: usize = 4;

    let wire = topo.wire(wire_id)?;
    let mut pts: Vec<Point3> = Vec::new();
    let mut prev_end: Option<brepkit_topology::vertex::VertexId> = None;
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let start_vid = edge.start();
        let end_vid = edge.end();
        let forward = match prev_end {
            Some(pe) if start_vid == pe && end_vid != pe => true,
            Some(pe) if end_vid == pe && start_vid != pe => false,
            _ => oe.is_forward(),
        };
        prev_end = Some(if forward { end_vid } else { start_vid });

        let start = topo.vertex(start_vid)?.point();
        let end = topo.vertex(end_vid)?.point();
        match edge.curve() {
            EdgeCurve::Line => pts.push(if forward { start } else { end }),
            EdgeCurve::Circle(c) => {
                let (t0, t1) = edge.curve().domain_with_endpoints(start, end);
                let (from, to) = if forward { (t0, t1) } else { (t1, t0) };
                for k in 0..ARC_SAMPLES {
                    let f = k as f64 / ARC_SAMPLES as f64;
                    pts.push(c.evaluate((to - from).mul_add(f, from)));
                }
            }
            EdgeCurve::Parabola(p) => {
                let (t0, t1) = edge.curve().domain_with_endpoints(start, end);
                let (from, to) = if forward { (t0, t1) } else { (t1, t0) };
                for k in 0..ARC_SAMPLES {
                    let f = k as f64 / ARC_SAMPLES as f64;
                    pts.push(p.evaluate((to - from).mul_add(f, from)));
                }
            }
            // Matches the refusal in `planar_wire_monomial_moments`: the
            // exact path does not handle these edge types, so the normal it
            // would produce is never used.
            EdgeCurve::Hyperbola(_) | EdgeCurve::Ellipse(_) | EdgeCurve::NurbsCurve(_) => {
                return Ok(None);
            }
        }
    }
    if pts.len() < 3 {
        return Ok(None);
    }
    let (mut nx, mut ny, mut nz) = (0.0, 0.0, 0.0);
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        nx += (a.y() - b.y()) * (a.z() + b.z());
        ny += (a.z() - b.z()) * (a.x() + b.x());
        nz += (a.x() - b.x()) * (a.y() + b.y());
    }
    let n = Vec3::new(nx, ny, nz);
    if n.length() < 1e-12 {
        return Ok(None);
    }
    Ok(Some(n))
}

/// Exact planar-face integration via Green's-theorem boundary integrals.
///
/// Returns `Ok(None)` when any wire contains an edge type the exact path
/// does not handle (ellipse or NURBS), or when the boundary is too
/// degenerate to determine its plane; the caller then falls back to the
/// chord-polygon fan path for the whole face.
fn integrate_planar_face_exact(
    topo: &Topology,
    face_id: FaceId,
    normal: Vec3,
) -> Result<Option<FaceContribution>, CheckError> {
    let face = topo.face(face_id)?;
    let outer_wire = face.outer_wire();
    let inner: Vec<_> = face.inner_wires().to_vec();

    // In-plane frame anchored at the first boundary vertex. The frame's
    // normal is derived from the boundary geometry itself (Newell), NOT the
    // face's stored plane normal: a malformed face whose stored normal is
    // inconsistent with its boundary would otherwise project to a collapsed
    // (zero-area) region, where the chord-polygon fan path still measures
    // the true geometric area. The flux terms below keep using the passed
    // `normal`, exactly like the fan path.
    let wire = topo.wire(outer_wire)?;
    let Some(first_edge) = wire.edges().first() else {
        return Ok(None);
    };
    let origin = topo.vertex(topo.edge(first_edge.edge())?.start())?.point();
    let Some(boundary_normal) = wire_newell_normal(topo, outer_wire)? else {
        return Ok(None);
    };
    let Ok(frame) = brepkit_math::frame::Frame3::from_normal(origin, boundary_normal) else {
        return Ok(None);
    };
    let (e1, e2) = (frame.x, frame.y);

    let Some(mut moments) = planar_wire_monomial_moments(topo, outer_wire, origin, e1, e2)? else {
        return Ok(None);
    };
    for wid in inner {
        let Some(hole) = planar_wire_monomial_moments(topo, wid, origin, e1, e2)? else {
            return Ok(None);
        };
        for k in 0..10 {
            moments[k] -= hole[k];
        }
    }

    // Linear forms of the global coordinates in the in-plane basis:
    // x = origin.x + e1.x·s + e2.x·t, etc.
    let lx = [origin.x(), e1.x(), e2.x()];
    let ly = [origin.y(), e1.y(), e2.y()];
    let lz = [origin.z(), e1.z(), e2.z()];
    let lin = |l: [f64; 3]| -> Poly2 {
        let mut p = [0.0; 10];
        p[0] = l[0];
        p[1] = l[1];
        p[2] = l[2];
        p
    };
    let (px, py, pz) = (lin(lx), lin(ly), lin(lz));
    let x2 = poly2_mul_linear(&px, lx);
    let y2 = poly2_mul_linear(&py, ly);
    let z2 = poly2_mul_linear(&pz, lz);
    let ig = |p: &Poly2| poly2_integrate(p, &moments);

    let area = moments[0];
    let (ix, iy, iz) = (ig(&px), ig(&py), ig(&pz));
    Ok(Some(FaceContribution {
        area,
        volume: (normal.x() * ix + normal.y() * iy + normal.z() * iz) / 3.0,
        volume_moment_x: 0.5 * normal.x() * ig(&x2),
        volume_moment_y: 0.5 * normal.y() * ig(&y2),
        volume_moment_z: 0.5 * normal.z() * ig(&z2),
        volume_second_x: normal.x() * ig(&poly2_mul_linear(&x2, lx)) / 3.0,
        volume_second_y: normal.y() * ig(&poly2_mul_linear(&y2, ly)) / 3.0,
        volume_second_z: normal.z() * ig(&poly2_mul_linear(&z2, lz)) / 3.0,
        volume_product_xy: 0.5 * normal.x() * ig(&poly2_mul_linear(&x2, ly)),
        volume_product_xz: 0.5 * normal.x() * ig(&poly2_mul_linear(&x2, lz)),
        volume_product_yz: 0.5 * normal.y() * ig(&poly2_mul_linear(&y2, lz)),
        centroid_x: ix,
        centroid_y: iy,
        centroid_z: iz,
    }))
}

/// Running totals of a face's contribution over quadrature abscissae.
#[derive(Default)]
struct Accumulator {
    area: f64,
    vol: f64,
    mx: f64,
    my: f64,
    mz: f64,
    qxx: f64,
    qyy: f64,
    qzz: f64,
    qxy: f64,
    qxz: f64,
    qyz: f64,
    cx: f64,
    cy: f64,
    cz: f64,
}

impl Accumulator {
    /// Add one abscissa's contribution, weighted by `w` (which already carries
    /// the map from the reference interval to the patch).
    fn add<S: ParametricSurface>(&mut self, surface: &S, u: f64, v: f64, w: f64) {
        let p = surface.evaluate(u, v);
        let du = surface.partial_u(u, v);
        let dv = surface.partial_v(u, v);

        // Normal = du x dv (unnormalized, includes Jacobian)
        let n = Vec3::new(
            du.y() * dv.z() - du.z() * dv.y(),
            du.z() * dv.x() - du.x() * dv.z(),
            du.x() * dv.y() - du.y() * dv.x(),
        );
        let n_len = n.length();

        self.area += w * n_len;

        // Volume: (1/3) P dot N (unnormalized N includes Jacobian)
        let pv = Vec3::new(p.x(), p.y(), p.z());
        self.vol += w * pv.dot(n) / 3.0;

        // Volume moments via divergence theorem:
        // CoM_x = (1/2V) surface_integral(x^2 * n_x dA)
        // n already includes Jacobian, so n.x() = N_x * |J|
        self.mx += w * 0.5 * p.x() * p.x() * n.x();
        self.my += w * 0.5 * p.y() * p.y() * n.y();
        self.mz += w * 0.5 * p.z() * p.z() * n.z();

        self.qxx += w * p.x().powi(3) * n.x() / 3.0;
        self.qyy += w * p.y().powi(3) * n.y() / 3.0;
        self.qzz += w * p.z().powi(3) * n.z() / 3.0;
        self.qxy += w * 0.5 * p.x().powi(2) * p.y() * n.x();
        self.qxz += w * 0.5 * p.x().powi(2) * p.z() * n.x();
        self.qyz += w * 0.5 * p.y().powi(2) * p.z() * n.y();

        self.cx += w * p.x() * n_len;
        self.cy += w * p.y() * n_len;
        self.cz += w * p.z() * n_len;
    }

    fn finish(self, sign: f64) -> FaceContribution {
        FaceContribution {
            area: self.area,
            volume: self.vol * sign,
            volume_moment_x: self.mx * sign,
            volume_moment_y: self.my * sign,
            volume_moment_z: self.mz * sign,
            volume_second_x: self.qxx * sign,
            volume_second_y: self.qyy * sign,
            volume_second_z: self.qzz * sign,
            volume_product_xy: self.qxy * sign,
            volume_product_xz: self.qxz * sign,
            volume_product_yz: self.qyz * sign,
            centroid_x: self.cx,
            centroid_y: self.cy,
            centroid_z: self.cz,
        }
    }
}

/// Composite quadrature tiles a domain axis into patches no larger than ~PI/4
/// so one Gauss rule resolves curved and periodic integrands. A single patch
/// over a torus's full 2*PI period in both u and v under-resolves it (~0.5%
/// error); several patches per period converge to machine precision. The patch
/// count is capped so a long *linear* axis (e.g. a tall cylinder/cone whose v
/// is axial distance) cannot make integration cost scale with model size — its
/// integrand is low-degree, so a bounded number of patches stays exact. Angular
/// axes never exceed 2*PI (= 8 patches), well under the cap.
const MAX_PATCHES: usize = 16;

/// Patches per knot span on a NURBS axis.
///
/// A knot span is one polynomial (here rational) piece of the surface, so it is
/// the natural quadrature cell: the integrand is smooth inside one and only
/// finitely differentiable across the join. Subdividing each piece converges
/// cleanly — measured on the single-span ruled parabolic wall of
/// `conic_prism_closed_form`, relative area error against its closed form:
///
///   2 per span  8.5e-7    6 per span  7.4e-11
///   4 per span  2.2e-9    8 per span  2.1e-12
///
/// identical at 1x, 1000x and 0.001x in every row. Eight is taken: it is three
/// orders past what any consumer needs and, against `MAX_PATCHES`, costs no
/// more than the previous worst case — a surface of two or more knot spans
/// clamps to 16 either way, so the cost ceiling is unchanged.
const PATCHES_PER_KNOT_SPAN: usize = 8;

/// The parameter-space length of one quadrature patch, per axis.
///
/// This is what makes a patch count dimensionless. Dividing a span by an
/// ABSOLUTE constant is only meaningful when the parameter is an angle, because
/// radians are dimensionless. A NURBS knot vector carries whatever units its
/// control points were built in, so the same surface, uniformly scaled, would
/// otherwise be tiled into a different number of patches purely because of
/// model size — and return a different area with no error and no warning.
#[derive(Clone, Copy)]
struct PatchScale {
    u: f64,
    v: f64,
}

impl PatchScale {
    /// Both axes measured in radians.
    ///
    /// PI/4 keeps eight patches per full turn. A quadric's non-angular axis
    /// (a cylinder's or cone's axial `v`) is deliberately measured the same
    /// way: its integrands are low-degree polynomials in `v`, exact under one
    /// Gauss rule at any patch count, so the tiling there affects cost only and
    /// the historical density is kept.
    const ANGULAR: Self = Self {
        u: std::f64::consts::FRAC_PI_4,
        v: std::f64::consts::FRAC_PI_4,
    };
}

/// Patch scale for a NURBS axis: one knot span, subdivided.
///
/// Expressed as a fraction of the axis's own domain, so it carries the same
/// units the span does and their ratio is a pure number. A surface and its
/// uniform scaling get identical patch counts.
fn knot_axis_patch_scale(knots: &[f64], domain: (f64, f64)) -> f64 {
    let extent = (domain.1 - domain.0).abs();
    if !extent.is_finite() || extent <= 0.0 {
        return std::f64::consts::FRAC_PI_4;
    }
    // Distinct knot values strictly inside the domain split it into spans;
    // repeated knots (which only lower continuity) must not each count.
    let eps = extent * 1e-12;
    let mut spans = 1usize;
    let mut last = f64::NEG_INFINITY;
    for &k in knots {
        if k > domain.0 + eps && k < domain.1 - eps && (k - last).abs() > eps {
            spans += 1;
            last = k;
        }
    }
    extent / (spans * PATCHES_PER_KNOT_SPAN) as f64
}

/// Number of patches an axis of the given span is tiled into.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn patch_count(span: f64, scale: f64) -> usize {
    ((span.abs() / scale).ceil() as usize).clamp(1, MAX_PATCHES)
}

/// Integrate a parametric surface over a UV domain by Gauss quadrature,
/// keeping only what `trim` accepts.
///
/// A face no loop cuts — a full revolution with no holes, or a boundary that
/// collapsed onto a seam — is integrated over the domain box directly. Any
/// other face has its u-quadrature split at every loop vertex and its
/// v-quadrature split at the loop crossings of each u abscissa (see
/// [`UvTrim::u_breaks`] and [`UvTrim::v_spans`]), so every loop edge falls ON
/// a quadrature interval boundary. That is what makes the result exact for the
/// polylines the loops are, instead of leaving them to whichever abscissae
/// happen to land inside: masking alone left the two lobes of a cross-drilled
/// bore wall 0.16 % short of their closed form.
#[allow(clippy::cast_precision_loss)]
fn integrate_parametric<S: ParametricSurface>(
    surface: &S,
    u_range: (f64, f64),
    v_range: (f64, f64),
    gauss_order: usize,
    sign: f64,
    trim: &UvTrim<'_>,
    scale: PatchScale,
) -> FaceContribution {
    let gauss_pts = gauss_legendre_points(gauss_order);
    let nu = patch_count(u_range.1 - u_range.0, scale.u);
    let du_patch = (u_range.1 - u_range.0) / nu as f64;
    let u_scale = du_patch / 2.0;
    let mut acc = Accumulator::default();

    if trim.splits_domain() {
        let breaks = trim.u_breaks(u_range);
        let eps = (u_range.1 - u_range.0).abs() * 1e-12;
        for w in breaks.windows(2) {
            let (u0, u1) = (w[0], w[1]);
            if u1 - u0 <= eps {
                continue;
            }
            let nu = patch_count(u1 - u0, scale.u);
            let du_patch = (u1 - u0) / nu as f64;
            let u_scale = du_patch / 2.0;
            for iu in 0..nu {
                let u_mid = du_patch.mul_add(iu as f64, u0) + u_scale;
                for gpu in gauss_pts {
                    let u = u_scale.mul_add(gpu.x, u_mid);
                    for (a, b) in trim.v_spans(u, v_range) {
                        let nv = patch_count(b - a, scale.v);
                        let dv_patch = (b - a) / nv as f64;
                        let v_scale = dv_patch / 2.0;
                        for iv in 0..nv {
                            let v_mid = dv_patch.mul_add(iv as f64, a) + v_scale;
                            for gpv in gauss_pts {
                                let v = v_scale.mul_add(gpv.x, v_mid);
                                acc.add(surface, u, v, gpu.w * gpv.w * u_scale * v_scale);
                            }
                        }
                    }
                }
            }
        }
        return acc.finish(sign);
    }

    let nv = patch_count(v_range.1 - v_range.0, scale.v);
    let dv_patch = (v_range.1 - v_range.0) / nv as f64;
    let v_scale = dv_patch / 2.0;
    for iu in 0..nu {
        let u_mid = du_patch.mul_add(iu as f64, u_range.0) + u_scale;
        for iv in 0..nv {
            let v_mid = dv_patch.mul_add(iv as f64, v_range.0) + v_scale;
            for gpu in gauss_pts {
                let u = u_scale.mul_add(gpu.x, u_mid);
                for gpv in gauss_pts {
                    let v = v_scale.mul_add(gpv.x, v_mid);
                    if !trim.accepts(u, v) {
                        continue;
                    }
                    acc.add(surface, u, v, gpu.w * gpv.w * u_scale * v_scale);
                }
            }
        }
    }
    acc.finish(sign)
}

/// Choose the UV domain a face's quadrature runs over, and how its boundary
/// trims it.
///
/// The dense boundary polygon is the reliable signal for a face's true
/// parametric extent: `face_uv_bounds` samples only sparse edge endpoints and
/// under-spans full-revolution faces (a cone's lateral face reports a narrow
/// u-range though its boundary wraps the full 2pi). A face that wraps the
/// full period in u, or whose boundary collapses onto a seam or pole, cannot
/// be trimmed by a UV polygon — the apex/pole/seam folds the polygon and the
/// point-in-polygon test rejects valid interior samples. Those cases integrate
/// the analytic surface over its true domain instead, and only the face's
/// holes are removed.
fn integrate_with_trimming<S: ParametricSurface>(
    surface: &S,
    u_range: (f64, f64),
    v_range: (f64, f64),
    gauss_order: usize,
    sign: f64,
    uv: &FaceUv,
    scale: PatchScale,
) -> FaceContribution {
    let holes_only = UvTrim::holes_of(uv);
    if uv.boundary.points.len() < 3 {
        return integrate_parametric(
            surface,
            u_range,
            v_range,
            gauss_order,
            sign,
            &holes_only,
            scale,
        );
    }

    let u_min = uv
        .boundary
        .points
        .iter()
        .map(|p| p.x())
        .fold(f64::INFINITY, f64::min);
    let v_min = uv
        .boundary
        .points
        .iter()
        .map(|p| p.y())
        .fold(f64::INFINITY, f64::min);
    let v_max = uv
        .boundary
        .points
        .iter()
        .map(|p| p.y())
        .fold(f64::NEG_INFINITY, f64::max);

    // Winding number of the boundary around the periodic u-axis: ±TAU for a
    // face that wraps a full revolution, ~0 for a partially-trimmed face.
    let tau = std::f64::consts::TAU;
    let winding = uv.boundary.u_winding();
    let full_revolution = uv.u_periodic && winding.abs() >= tau - WRAP_EPS;
    let v_degenerate = (v_max - v_min) <= 1e-9;

    if full_revolution && v_degenerate {
        // Polar cap (e.g. a sphere hemisphere bounded only by one latitude
        // circle): the cap runs from that latitude to a pole. The winding sign
        // (CCW vs CW boundary) selects which pole — the boundary's interior
        // side — so the two hemispheres do not both integrate the whole sphere.
        let v_pole = if winding >= 0.0 { v_range.1 } else { v_range.0 };
        // A full-revolution hole at a latitude between the outer circle and the
        // pole (the drilled-tunnel rim) clips the cap into a band: integrate
        // only from the outer latitude to the hole, not on to the pole.
        let v_far = uv
            .hole_vs
            .iter()
            .copied()
            // Same side of v_min as the pole (strict same sign → positive
            // product), and not coincident with v_min.
            .filter(|&hv| (hv - v_min) * (v_pole - v_min) > 0.0 && (hv - v_min).abs() > 1e-9)
            .min_by(|a, b| (a - v_min).abs().total_cmp(&(b - v_min).abs()))
            .unwrap_or(v_pole);
        let v_dom = (v_min.min(v_far), v_min.max(v_far));
        integrate_parametric(
            surface,
            (u_min, u_min + tau),
            v_dom,
            gauss_order,
            sign,
            &UvTrim::pockets_of(uv),
            scale,
        )
    } else if full_revolution {
        // Full-revolution band (cone/cylinder): integrate the whole revolution
        // over the band's v-extent.
        integrate_parametric(
            surface,
            (u_min, u_min + tau),
            (v_min, v_max),
            gauss_order,
            sign,
            &holes_only,
            scale,
        )
    } else if uv.boundary.area() <= DEGENERATE_UV_AREA {
        // Collapsed polygon (e.g. a closed torus whose seam projects to a
        // point): trust the analytic full-domain range from `face_uv_bounds`.
        integrate_parametric(
            surface,
            u_range,
            v_range,
            gauss_order,
            sign,
            &holes_only,
            scale,
        )
    } else {
        integrate_parametric(
            surface,
            u_range,
            v_range,
            gauss_order,
            sign,
            &UvTrim::boundary_of(uv),
            scale,
        )
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::unwrap_used)]

    use super::*;
    use brepkit_math::vec::{Point3, Vec3};

    #[test]
    fn planar_fan_is_signed_on_nonconvex_polygons() {
        let poly = [
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(10.0, 0.0, 2.0),
            Point3::new(10.0, 5.0, 2.0),
            Point3::new(5.0, 5.0, 2.0),
            Point3::new(5.0, 10.0, 2.0),
            Point3::new(0.0, 10.0, 2.0),
        ];
        let up = Vec3::new(0.0, 0.0, 1.0);
        let contribution = integrate_planar_polygon(&poly, up);
        assert!((contribution.area - 75.0).abs() < 1e-9);
        assert!((contribution.volume - 50.0).abs() < 1e-9);

        let reversed: Vec<Point3> = poly.iter().rev().copied().collect();
        let reversed_contribution = integrate_planar_polygon(&reversed, up);
        assert!((reversed_contribution.area - 75.0).abs() < 1e-9);
        assert!((reversed_contribution.volume - 50.0).abs() < 1e-9);
    }
}
