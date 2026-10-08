//! Shared helper functions for PaveFiller phases.
//!
//! Extracted from phase_ee, phase_ef, and phase_ve to eliminate
//! duplicated vertex-lookup and pave-insertion logic.

use remus_math::aabb::Aabb3;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::FaceSurface;
use remus_topology::vertex::VertexId;

use crate::ds::{GfaArena, Pave};
use crate::error::AlgoError;

/// Clamped quarter-arc knots (`to_nurbs` u) and clamped linear knots (v) of
/// an exact-rational converted cylinder wall.
const WALL_KNOTS_U: [f64; 12] = [
    0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
];
const WALL_KNOTS_V: [f64; 4] = [0.0, 0.0, 1.0, 1.0];

/// Certify the same complete spline trace, including the exact reversed
/// representation produced by extrusion. Coefficients and parameter spans
/// must agree algebraically; co-endpoint lenses and partial spans do not match.
#[allow(clippy::float_cmp)]
#[cfg_attr(target_arch = "wasm32", inline(never))]
pub(super) fn identical_nurbs_span(
    a: &remus_math::nurbs::curve::NurbsCurve,
    span_a: (f64, f64),
    b: &remus_math::nurbs::curve::NurbsCurve,
    span_b: (f64, f64),
) -> bool {
    let same_span = |x: (f64, f64), y: (f64, f64)| x == y || x == (y.1, y.0);
    if a == b && same_span(span_a, span_b) {
        return true;
    }
    let mirrored = |curve: &remus_math::nurbs::curve::NurbsCurve, span: (f64, f64)| {
        let sum = curve.knots()[0] + curve.knots()[curve.knots().len() - 1];
        (sum - span.0, sum - span.1)
    };
    (a.reversed() == *b && same_span(mirrored(a, span_a), span_b))
        || (b.reversed() == *a && same_span(span_a, mirrored(b, span_b)))
}

/// Resolve the stored parameter authority for a topology edge.
///
/// PaveFiller must never reconstruct a curved edge's branch from its endpoint
/// positions: periodic seams and major/reversed spans are not recoverable from
/// those points alone. Lines retain their intrinsic endpoint-local `[0, 1]`
/// domain through [`Edge::strict_domain`].
///
/// Visible to `crate::builder` and `crate::classifier` alongside the other
/// helpers in this module (see the `redundant_pub_crate` note on `mod helpers`).
#[allow(clippy::redundant_pub_crate)]
#[cfg_attr(target_arch = "wasm32", inline(never))]
pub(crate) fn authoritative_edge_domain(
    edge: &Edge,
    edge_id: EdgeId,
    stage: &'static str,
) -> Result<(f64, f64), AlgoError> {
    edge.strict_domain().map_err(|error| {
        AlgoError::IntersectionFailed(format!(
            "{stage} edge {edge_id:?} lacks authoritative parameter range: {error}"
        ))
    })
}

/// Validate a complete edge set before a phase starts mutating its arena.
pub(super) fn validate_edge_domains(
    topo: &Topology,
    edges: &[EdgeId],
    stage: &'static str,
) -> Result<(), AlgoError> {
    for &edge_id in edges {
        let edge = topo.edge(edge_id)?;
        authoritative_edge_domain(edge, edge_id, stage)?;
    }
    Ok(())
}

/// Return the part of a vertex ball that widens an operation beyond its
/// global linear floor.
pub(super) fn vertex_tolerance_excess(
    topo: &Topology,
    vertex_id: VertexId,
    floor: f64,
) -> Result<f64, AlgoError> {
    let value = topo.vertex(vertex_id)?.tolerance();
    tolerance_excess(value, floor, "vertex")
}

/// Return the part of an edge tube that widens an operation beyond its
/// global linear floor.
pub(super) fn edge_tolerance_excess(
    topo: &Topology,
    edge_id: EdgeId,
    floor: f64,
) -> Result<f64, AlgoError> {
    let edge = topo.edge(edge_id)?;
    let start_tolerance = topo.vertex(edge.start())?.tolerance();
    let end_tolerance = topo.vertex(edge.end())?.tolerance();
    tolerance_excess(start_tolerance, floor, "vertex")?;
    tolerance_excess(end_tolerance, floor, "vertex")?;
    let vertex_tolerance = start_tolerance.max(end_tolerance);
    tolerance_excess(edge.effective_tolerance(vertex_tolerance), floor, "edge")
}

/// Add tolerance contributions and reject a non-finite acceptance band.
pub(super) fn tolerance_band(
    floor: f64,
    contributions: impl IntoIterator<Item = f64>,
) -> Result<f64, AlgoError> {
    let mut band = floor;
    for contribution in contributions {
        band += contribution;
    }
    if !band.is_finite() || band.is_sign_negative() {
        return Err(remus_topology::TopologyError::InvalidToleranceValue {
            entity: "predicate band",
            value: band,
        }
        .into());
    }
    Ok(band)
}

fn tolerance_excess(value: f64, floor: f64, entity: &'static str) -> Result<f64, AlgoError> {
    if !value.is_finite() || value.is_sign_negative() {
        return Err(remus_topology::TopologyError::InvalidToleranceValue { entity, value }.into());
    }
    Ok((value - floor).max(0.0))
}

/// Find a vertex near the given point among all pave block vertices.
///
/// Returns the resolved (same-domain canonical) vertex first encountered within
/// the operation floor or the candidate vertex's wider tolerance ball,
/// scanning pave blocks in `edge_pave_blocks` order
/// (ascending `EdgeId`, start-before-end). When the arena's spatial index is
/// available (built after Phase VV) the lookup is O(1) and returns the exact
/// same vertex; otherwise it falls back to the linear scan.
pub(super) fn find_nearby_pave_vertex(
    topo: &Topology,
    arena: &GfaArena,
    point: Point3,
    tol: Tolerance,
) -> Option<VertexId> {
    if let Some(index) = &arena.pave_vertex_index {
        return index.find_with_entry_radius(point);
    }
    for pbs in arena.edge_pave_blocks.values() {
        for &pb_id in pbs {
            if let Some(pb) = arena.pave_blocks.get(pb_id) {
                for vid in [pb.start.vertex, pb.end.vertex] {
                    crate::perf::bump_pave_vertex_probe();
                    let resolved = arena.resolve_vertex(vid);
                    if let Ok(v) = topo.vertex(resolved)
                        && (v.point() - point).length() <= v.tolerance().max(tol.linear)
                    {
                        return Some(resolved);
                    }
                }
            }
        }
    }
    None
}

/// Widened variant of [`find_nearby_pave_vertex`] for tangential contacts.
///
/// A grazing crossing's solved position is only accurate to
/// `sqrt(2 * r * residual)`, so the exact junction vertex can sit microns
/// outside the linear tolerance. This scans every pave-block endpoint within
/// `radius` and returns the nearest candidate that passes `accept` (the
/// caller checks genuine curve/surface incidence, which is what makes the
/// widened radius safe). If the accepted candidates span more than one
/// distinct position (beyond `tol_linear` of each other), the contact is
/// ambiguous — two different junctions inside the window — and `None` is
/// returned so the caller keeps the solved point rather than merging
/// distinct junctions. The spatial index uses cells at least as large as the
/// maximum widened radius, keeping this lookup bounded to a 3x3x3 stencil.
pub(super) fn find_nearby_pave_vertex_widened(
    arena: &GfaArena,
    point: Point3,
    radius: f64,
    tol_linear: f64,
    accept: impl Fn(Point3) -> bool,
) -> Option<VertexId> {
    arena
        .pave_vertex_index
        .as_ref()?
        .find_unambiguous_within(point, radius, tol_linear, accept)
}

/// Add a pave to the appropriate pave block of an edge.
///
/// Finds the pave block whose parameter range contains the pave's
/// parameter (with a small guard band) and adds the extra pave to it.
pub(super) fn add_pave_to_edge(arena: &mut GfaArena, edge_id: EdgeId, pave: Pave) {
    if let Some(pb_ids) = arena.edge_pave_blocks.get(&edge_id) {
        let pb_ids_copy: Vec<_> = pb_ids.clone();
        for pb_id in pb_ids_copy {
            if let Some(pb) = arena.pave_blocks.get_mut(pb_id)
                && pb.contains_parameter_interior(pave.parameter, 1e-10)
            {
                pb.add_extra_pave(pave);
            }
        }
    }
}

/// Recognize an exact-rational converted cylinder wall carried as NURBS.
///
/// Matches exactly what `CylindricalSurface::to_nurbs` (`math/src/surfaces.rs`)
/// emits: degree (2, 1), a 9×2 grid, clamped quarter-arc knots in u,
/// clamped linear knots in v, 1/√2 diagonal weights, coincident seam rows,
/// parallel ring axes, and one common radius. Returns the recovered
/// analytic cylinder so callers can measure against it exactly.
///
/// Structural gates only — sampled/freeform sheets (cone/sphere/torus
/// sampled grids, imported freeform) return `None` and keep their previous
/// handling. In particular the `(9, 2)` grid + `(2, 1)` degree + knot
/// fingerprint excludes every other `convert_to_bspline` emitter (bilinear
/// planes are (1,1) 2×2; cone/sphere/torus are 33×9 degree-1 sampled).
/// Callers must additionally bound the result to the face's trimmed region
/// (containment/extent); the carrier alone is unbounded in v.
#[allow(clippy::items_after_statements, clippy::redundant_pub_crate)]
pub(crate) fn rational_cylinder_wall(
    nurbs: &remus_math::nurbs::surface::NurbsSurface,
    tol: Tolerance,
) -> Option<remus_math::surfaces::CylindricalSurface> {
    use remus_math::vec::Vec3;
    if nurbs.degree_u() != 2 || nurbs.degree_v() != 1 {
        return None;
    }
    let cps = nurbs.control_points();
    if cps.len() != 9 || cps.iter().any(|row| row.len() != 2) {
        return None;
    }
    let ws = nurbs.weights();
    if ws.len() != 9 || ws.iter().any(|row| row.len() != 2) {
        return None;
    }
    if nurbs.knots_u() != WALL_KNOTS_U || nurbs.knots_v() != WALL_KNOTS_V {
        return None;
    }
    let w1 = std::f64::consts::FRAC_1_SQRT_2;
    for (i, row) in ws.iter().enumerate() {
        let expected = if i % 2 == 0 { 1.0 } else { w1 };
        if (row[0] - expected).abs() > 1e-12
            || (row[1] - expected).abs() > 1e-12
            || (row[0] - row[1]).abs() > 1e-12
        {
            return None;
        }
    }
    // Closed seam: first and last rows coincide (same gate `is_periodic_u`
    // uses, tightened to the caller's linear tolerance).
    if (cps[0][0] - cps[8][0]).length() > tol.linear
        || (cps[0][1] - cps[8][1]).length() > tol.linear
    {
        return None;
    }
    // All nine ring axes parallel: the v-direction column of every row.
    let mut axis_sum = Vec3::new(0.0, 0.0, 0.0);
    for row in cps {
        let v = row[1] - row[0];
        let len = v.length();
        if !len.is_finite() || len <= tol.linear {
            return None;
        }
        axis_sum += v * (1.0 / len);
    }
    let axis = axis_sum.normalize().ok()?;
    // Ring centre from the four cardinal bottom points; common radius.
    let bot = {
        let (mut sx, mut sy, mut sz) = (0.0, 0.0, 0.0);
        for i in [0, 2, 4, 6] {
            sx += cps[i][0].x();
            sy += cps[i][0].y();
            sz += cps[i][0].z();
        }
        Point3::new(sx / 4.0, sy / 4.0, sz / 4.0)
    };
    let mut radius = 0.0;
    for i in [0, 2, 4, 6] {
        radius += ((cps[i][0] - bot) - axis * axis.dot(cps[i][0] - bot)).length();
    }
    radius /= 4.0;
    if !radius.is_finite() || radius <= tol.linear {
        return None;
    }
    // Every cardinal bottom point sits on the recovered cylinder.
    for i in [0, 2, 4, 6] {
        let radial = (cps[i][0] - bot) - axis * axis.dot(cps[i][0] - bot);
        if (radial.length() - radius).abs() > tol.linear {
            return None;
        }
    }
    remus_math::surfaces::CylindricalSurface::new(bot, axis, radius).ok()
}

/// The plane a NURBS surface lies in when its whole control net is coplanar
/// within `tol.linear` — a rational surface never leaves the convex hull of
/// its net, so coplanar control points certify a planar surface exactly.
/// `None` for a genuinely curved net or for anything but a NURBS.
#[allow(clippy::redundant_pub_crate)]
pub(crate) fn planar_nurbs_as_plane(surface: &FaceSurface, tol: Tolerance) -> Option<FaceSurface> {
    let FaceSurface::Nurbs(nurbs) = surface else {
        return None;
    };
    let points: Vec<Point3> = nurbs
        .control_points()
        .iter()
        .flat_map(|row| row.iter().copied())
        .collect();
    let origin = *points.first()?;
    // A well-conditioned normal in two linear passes: the net point farthest
    // from the origin spans the first direction, and the point whose cross
    // product with it is largest spans the second.
    let far = points
        .iter()
        .copied()
        .max_by(|a, b| (*a - origin).length().total_cmp(&(*b - origin).length()))?;
    let axis = far - origin;
    let (len, normal) = points
        .iter()
        .map(|&p| {
            let n = axis.cross(p - origin);
            (n.length(), n)
        })
        .filter(|(l, _)| l.is_finite())
        .max_by(|a, b| a.0.total_cmp(&b.0))?;
    if len <= tol.linear * tol.linear {
        return None;
    }
    let normal = normal * (1.0 / len);
    let d = normal.dot(origin - Point3::new(0.0, 0.0, 0.0));
    let coplanar = points
        .iter()
        .all(|p| (normal.dot(*p - Point3::new(0.0, 0.0, 0.0)) - d).abs() <= tol.linear);
    coplanar.then_some(FaceSurface::Plane { normal, d })
}

/// A box containing every point `evaluate_with_endpoints` can return on this
/// edge: the whole carrier (a circle's or ellipse's full turn, a NURBS
/// curve's control polygon by the convex-hull property, a line's segment)
/// plus the stored endpoints, which may sit a vertex tolerance off the curve.
/// It holds up to the rounding of the evaluation itself, a few ulps of the
/// largest coordinate, which callers cover with [`coordinate_slack`].
///
/// Circle and ellipse extents come from the frame evaluation uses: coordinate
/// `e` of `a·cos(t)·u + b·sin(t)·v` never exceeds `hypot(a·u_e, b·v_e)`
/// (Cauchy-Schwarz), whatever the frame. A bound from `normal()` cancels to
/// zero when the normal is within ~1e-8 of a world axis, and fails outright
/// for the non-orthonormal frames `with_axes` and deserialization accept.
///
/// `None` for carriers without a cheap finite bound (parabola, hyperbola) and
/// for non-finite or degenerate data; such edges are never gated.
pub(super) fn conservative_curve_aabb(
    curve: &EdgeCurve,
    start: Point3,
    end: Point3,
) -> Option<Aabb3> {
    let (center, u, a, v, b) = match curve {
        EdgeCurve::Line => return finite_aabb([start, end]),
        EdgeCurve::Circle(circle) => (
            circle.center(),
            circle.u_axis(),
            circle.radius(),
            circle.v_axis(),
            circle.radius(),
        ),
        EdgeCurve::Ellipse(ellipse) => (
            ellipse.center(),
            ellipse.u_axis(),
            ellipse.semi_major(),
            ellipse.v_axis(),
            ellipse.semi_minor(),
        ),
        EdgeCurve::NurbsCurve(nurbs) => {
            if !nurbs.weights().iter().all(|w| w.is_finite() && *w > 0.0) {
                return None;
            }
            return finite_aabb(nurbs.control_points().iter().copied().chain([start, end]));
        }
        EdgeCurve::Parabola(_) | EdgeCurve::Hyperbola(_) => return None,
    };
    if !(a > 0.0 && b > 0.0) {
        return None;
    }
    let extent = |ue: f64, ve: f64| (a * ue).hypot(b * ve) * (1.0 + 8.0 * f64::EPSILON);
    let half = Vec3::new(
        extent(u.x(), v.x()),
        extent(u.y(), v.y()),
        extent(u.z(), v.z()),
    );
    finite_aabb([center - half, center + half, start, end])
}

/// The box of `points`, or `None` when any coordinate is not finite (a NaN
/// after the first point would otherwise be skipped by the min/max scan).
pub(super) fn finite_aabb(points: impl IntoIterator<Item = Point3>) -> Option<Aabb3> {
    let mut finite = true;
    let bbox = Aabb3::try_from_points(points.into_iter().inspect(|p| {
        finite &= p.x().is_finite() && p.y().is_finite() && p.z().is_finite();
    }))?;
    finite.then_some(bbox)
}

/// Allowance for testing a box of evaluated geometry against a fixed band:
/// `64 ε` times its largest absolute coordinate, well above the few ulps by
/// which curve or surface evaluation (and the box arithmetic) can stray past
/// the exact bound at that magnitude. Infinite for a non-finite box, so a
/// gate built on it never fires.
pub(super) fn coordinate_slack(bbox: Aabb3) -> f64 {
    let mut largest = 0.0_f64;
    for c in [bbox.min, bbox.max]
        .iter()
        .flat_map(|p| [p.x(), p.y(), p.z()])
    {
        if !c.is_finite() {
            return f64::INFINITY;
        }
        largest = largest.max(c.abs());
    }
    64.0 * f64::EPSILON * largest
}

/// Largest departure from orthonormality (`|e·e - 1|`, `|e·f|`) a carrier
/// frame may show and still be gated. Within it, every point the carrier
/// evaluates to lies within `3 * CARRIER_FRAME_TOL * radius` of the ideal
/// carrier, which [`CarrierGate::clearance`] deducts with room to spare.
const CARRIER_FRAME_TOL: f64 = 1e-12;

/// Rounding allowance of [`CarrierGate::clearance`] per unit of the
/// Euclidean scale `|anchor| + radius + |edge point|`. Evaluating the carrier
/// point `S(project(p))` and the curve point, their distance, and the
/// interval arithmetic here each err by a small multiple of `ε` of that
/// scale, about `50 ε` together in the worst case; `128 ε` more than
/// doubles it.
const CARRIER_GAP_ROUNDING: f64 = 128.0 * f64::EPSILON;

/// Where an edge can be: a superset of every point the edge-face scan
/// evaluates on it, in the form the carrier clearance bounds use.
#[derive(Debug, Clone, Copy)]
pub(super) enum EdgeReach {
    /// A line edge: its segment (`Line` evaluates `start + (end - start) * t`
    /// on exactly `t` in `[0, 1]`).
    Segment(Point3, Point3),
    /// A circle or ellipse: its whole carrier lies within `reach` of
    /// `center` (evaluation ignores the stored endpoints).
    Disc {
        /// Carrier centre.
        center: Point3,
        /// Bound on the distance of any carrier point from `center`.
        reach: f64,
    },
    /// A NURBS edge: its conservative box.
    Hull(Aabb3),
}

impl EdgeReach {
    /// `None` for parabolas and hyperbolas and for non-finite or degenerate
    /// data; such edges are never gated.
    pub(super) fn of(curve: &EdgeCurve, start: Point3, end: Point3) -> Option<Self> {
        // |a cos(t) u + b sin(t) v|^2 <= max(a^2 |u|^2, b^2 |v|^2) + a b |u.v|
        // for any frame, orthonormal or not.
        let disc = |center: Point3, u: Vec3, a: f64, v: Vec3, b: f64| {
            let widest = finite_max([a * a * u.length_squared(), b * b * v.length_squared()])?;
            let reach = (widest + a * b * u.dot(v).abs()).sqrt() * (1.0 + 8.0 * f64::EPSILON);
            (a > 0.0 && b > 0.0 && is_finite_point(center) && reach.is_finite())
                .then_some(Self::Disc { center, reach })
        };
        match curve {
            EdgeCurve::Line => (is_finite_point(start) && is_finite_point(end))
                .then_some(Self::Segment(start, end)),
            EdgeCurve::Circle(c) => {
                disc(c.center(), c.u_axis(), c.radius(), c.v_axis(), c.radius())
            }
            EdgeCurve::Ellipse(e) => disc(
                e.center(),
                e.u_axis(),
                e.semi_major(),
                e.v_axis(),
                e.semi_minor(),
            ),
            EdgeCurve::NurbsCurve(_) => conservative_curve_aabb(curve, start, end).map(Self::Hull),
            EdgeCurve::Parabola(_) | EdgeCurve::Hyperbola(_) => None,
        }
    }

    /// A bound on `|p|` over the reach, scaling the rounding allowance.
    fn scale(&self) -> f64 {
        match *self {
            Self::Segment(a, b) => magnitude(a).max(magnitude(b)),
            Self::Disc { center, reach } => magnitude(center) + reach,
            Self::Hull(bbox) => {
                let far = |lo: f64, hi: f64| lo.abs().max(hi.abs());
                let (lo, hi) = (bbox.min, bbox.max);
                Vec3::new(
                    far(lo.x(), hi.x()),
                    far(lo.y(), hi.y()),
                    far(lo.z(), hi.z()),
                )
                .length()
            }
        }
    }

    /// The interval of distances from the line through `origin` along unit
    /// `axis` over the reach. That distance is convex and 1-Lipschitz, so a
    /// segment peaks at an end, a hull at a corner, and nothing in a disc or
    /// hull is closer than its centre less its radius (the hull's being the
    /// farthest corner offset, seen across the axis).
    fn axis_distance(&self, origin: Point3, axis: Vec3) -> Option<(f64, f64)> {
        let across = |w: Vec3| w - axis * axis.dot(w);
        let rho = |p: Point3| across(p - origin).length();
        let (lo, hi) = match *self {
            Self::Segment(a, b) => {
                let q0 = across(a - origin);
                let d = across(b - a);
                let dd = d.dot(d);
                let lo = if dd > 0.0 {
                    (q0 + d * (-q0.dot(d) / dd).clamp(0.0, 1.0)).length()
                } else {
                    q0.length() - d.length()
                };
                (lo, finite_max([rho(a), rho(b)])?)
            }
            Self::Disc { center, reach } => {
                let c = rho(center);
                (c - reach, c + reach)
            }
            Self::Hull(bbox) => {
                let h = (bbox.max - bbox.min) * 0.5;
                let offsets = [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)]
                    .map(|(sy, sz)| across(Vec3::new(h.x(), sy * h.y(), sz * h.z())).length());
                let lo = rho(bbox.center()) - finite_max(offsets)?;
                (lo, finite_max(corners(bbox).map(rho))?)
            }
        };
        (lo.is_finite() && hi.is_finite()).then_some((lo, hi))
    }

    /// The interval of distances from `c` over the reach.
    fn point_distance(&self, c: Point3) -> Option<(f64, f64)> {
        let (lo, hi) = match *self {
            Self::Segment(a, b) => {
                let d = b - a;
                let dd = d.dot(d);
                let lo = if dd > 0.0 {
                    (a + d * ((c - a).dot(d) / dd).clamp(0.0, 1.0) - c).length()
                } else {
                    (a - c).length() - d.length()
                };
                (lo, finite_max([(a - c).length(), (b - c).length()])?)
            }
            Self::Disc { center, reach } => {
                let d = (center - c).length();
                (d - reach, d + reach)
            }
            Self::Hull(bbox) => {
                let (lo, hi) = (bbox.min, bbox.max);
                let nearest = Point3::new(
                    c.x().max(lo.x()).min(hi.x()),
                    c.y().max(lo.y()).min(hi.y()),
                    c.z().max(lo.z()).min(hi.z()),
                );
                let farthest = finite_max(corners(bbox).map(|p| (p - c).length()))?;
                ((nearest - c).length(), farthest)
            }
        };
        (lo.is_finite() && hi.is_finite()).then_some((lo, hi))
    }
}

/// A cylinder or sphere face carrier an edge-face pair can be skipped
/// against. The scan measures `|p - S(project(p))|` with `S` on the carrier,
/// which is never less, up to rounding, than the distance from `p` to the
/// infinite carrier.
#[derive(Debug, Clone, Copy)]
pub(super) enum CarrierGate {
    /// Points `radius` from the line through `origin` along unit `axis`.
    Cylinder {
        /// A point on the axis.
        origin: Point3,
        /// Unit axis direction.
        axis: Vec3,
        /// Wall radius.
        radius: f64,
    },
    /// Points `radius` from `center`.
    Sphere {
        /// Sphere centre.
        center: Point3,
        /// Sphere radius.
        radius: f64,
    },
}

impl CarrierGate {
    /// The gate for a face carrier, read from the fields evaluation uses.
    /// `None` for planes (crossed by a sign change, not by proximity),
    /// cones, tori and NURBS, and for a carrier whose radius is not finite
    /// and positive or whose frame is not orthonormal within
    /// [`CARRIER_FRAME_TOL`]: deserialized data never ran the constructors'
    /// checks.
    pub(super) fn of(surface: &FaceSurface) -> Option<Self> {
        match surface {
            FaceSurface::Cylinder(c) => {
                if !sound_carrier(c.origin(), c.radius(), [c.x_axis(), c.y_axis(), c.axis()]) {
                    return None;
                }
                Some(Self::Cylinder {
                    origin: c.origin(),
                    axis: c.axis().normalize().ok()?,
                    radius: c.radius(),
                })
            }
            FaceSurface::Sphere(s) => {
                sound_carrier(s.center(), s.radius(), [s.x_axis(), s.y_axis(), s.z_axis()])
                    .then_some(Self::Sphere {
                        center: s.center(),
                        radius: s.radius(),
                    })
            }
            FaceSurface::Plane { .. }
            | FaceSurface::Nurbs(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Torus(_) => None,
        }
    }

    /// A lower bound, net of rounding and frame error, on the distance the
    /// scan can measure from any point of `reach` to this carrier; `None`
    /// when it cannot be certified.
    pub(super) fn clearance(&self, reach: &EdgeReach) -> Option<f64> {
        let (anchor, radius, (lo, hi)) = match *self {
            Self::Cylinder {
                origin,
                axis,
                radius,
            } => (origin, radius, reach.axis_distance(origin, axis)?),
            Self::Sphere { center, radius } => (center, radius, reach.point_distance(center)?),
        };
        // Outside the carrier by `lo - radius`, or inside it by `radius - hi`.
        let gap = finite_max([lo - radius, radius - hi])?;
        let scale = magnitude(anchor) + radius + reach.scale();
        let clearance = gap - CARRIER_GAP_ROUNDING * scale - 4.0 * CARRIER_FRAME_TOL * radius;
        clearance.is_finite().then_some(clearance)
    }
}

/// Whether a carrier's radius and frame are fit to gate on.
fn sound_carrier(anchor: Point3, radius: f64, [x, y, z]: [Vec3; 3]) -> bool {
    let unit = |e: Vec3| (e.dot(e) - 1.0).abs() <= CARRIER_FRAME_TOL;
    let square = |e: Vec3, f: Vec3| e.dot(f).abs() <= CARRIER_FRAME_TOL;
    radius.is_finite()
        && radius > 0.0
        && is_finite_point(anchor)
        && unit(x)
        && unit(y)
        && unit(z)
        && square(x, y)
        && square(x, z)
        && square(y, z)
}

/// The largest of `values`, or `None` if any is not finite (`f64::max`
/// would silently drop a NaN).
fn finite_max<const N: usize>(values: [f64; N]) -> Option<f64> {
    let mut largest = f64::NEG_INFINITY;
    for value in values {
        if !value.is_finite() {
            return None;
        }
        if value > largest {
            largest = value;
        }
    }
    Some(largest)
}

fn is_finite_point(p: Point3) -> bool {
    p.x().is_finite() && p.y().is_finite() && p.z().is_finite()
}

fn magnitude(p: Point3) -> f64 {
    Vec3::new(p.x(), p.y(), p.z()).length()
}

fn corners(bbox: Aabb3) -> [Point3; 8] {
    let (lo, hi) = (bbox.min, bbox.max);
    [0_u8, 1, 2, 3, 4, 5, 6, 7].map(|i| {
        let pick = |bit: u8, a: f64, b: f64| if i & bit == 0 { a } else { b };
        Point3::new(
            pick(1, lo.x(), hi.x()),
            pick(2, lo.y(), hi.y()),
            pick(4, lo.z(), hi.z()),
        )
    })
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::float_cmp,
        clippy::panic
    )]

    use super::*;
    use remus_math::curves::{Circle3D, Ellipse3D, Parabola3D};
    use remus_math::nurbs::curve::NurbsCurve;

    const TAU: f64 = std::f64::consts::TAU;

    /// Every point the edge-face scan or the vertex-edge projection can
    /// evaluate on an edge must lie inside `conservative_curve_aabb` grown by
    /// its `coordinate_slack`, the box every caller actually tests. Checked on
    /// a dense parameter sweep, of which the scans' points are a subset, and
    /// on the stored endpoints themselves.
    fn assert_box_covers_edge(curve: &EdgeCurve, start: Point3, end: Point3, t0: f64, t1: f64) {
        let bbox = conservative_curve_aabb(curve, start, end).expect("gated curve kind");
        let bbox = bbox.expanded(coordinate_slack(bbox));
        for i in 0..=4096 {
            let t = t0 + (t1 - t0) * (f64::from(i) / 4096.0);
            let p = curve.evaluate_with_endpoints(t, start, end);
            assert!(
                bbox.contains_point(p),
                "sample at t={t} outside the gate box {bbox:?}: {p:?}"
            );
        }
        assert!(bbox.contains_point(start));
        assert!(bbox.contains_point(end));
    }

    /// The whole closed carrier, endpoints at its seam.
    fn assert_box_covers_full_turn(curve: &EdgeCurve) {
        let zero = Point3::new(0.0, 0.0, 0.0);
        let seam = curve.evaluate_with_endpoints(0.0, zero, zero);
        assert_box_covers_edge(curve, seam, seam, 0.0, TAU);
    }

    fn rational_cubic() -> NurbsCurve {
        NurbsCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 3.0, 0.0),
                Point3::new(2.0, -3.0, 1.0),
                Point3::new(3.0, 3.0, -1.0),
                Point3::new(4.0, 0.0, 0.0),
            ],
            vec![1.0, 2.0, 0.5, 2.0, 1.0],
        )
        .unwrap()
    }

    #[test]
    fn gate_box_covers_circle_arcs_across_the_seam_and_offset_endpoints() {
        let circle =
            Circle3D::new(Point3::new(3.0, -2.0, 1.0), Vec3::new(0.6, 0.0, 0.8), 2.5).unwrap();
        let curve = EdgeCurve::Circle(circle);
        let zero = Point3::new(0.0, 0.0, 0.0);
        // Endpoints a vertex tolerance off the true curve, as imported STEP
        // often stores them; ranges include the full turn, a seam crossing
        // and a reversed range.
        for (t0, t1) in [(0.0, TAU), (5.5, 7.0), (0.3, 2.9), (2.9, 0.3)] {
            let start = curve.evaluate_with_endpoints(t0, zero, zero);
            let end = curve.evaluate_with_endpoints(t1, zero, zero);
            let start = Point3::new(start.x() + 5e-7, start.y(), start.z() - 5e-7);
            let end = Point3::new(end.x(), end.y() + 5e-7, end.z());
            assert_box_covers_edge(&curve, start, end, t0, t1);
        }
    }

    #[test]
    fn gate_box_covers_ellipses_and_nurbs_curves() {
        let ellipse = Ellipse3D::new(
            Point3::new(-1.0, 4.0, 0.5),
            Vec3::new(0.0, 1.0, 0.0),
            3.0,
            1.5,
        )
        .unwrap();
        let curve = EdgeCurve::Ellipse(ellipse);
        let zero = Point3::new(0.0, 0.0, 0.0);
        let start = curve.evaluate_with_endpoints(0.2, zero, zero);
        let end = curve.evaluate_with_endpoints(4.0, zero, zero);
        assert_box_covers_edge(&curve, start, end, 0.2, 4.0);

        let curve = EdgeCurve::NurbsCurve(rational_cubic());
        let start = Point3::new(0.0, 0.0, 6e-7);
        let end = Point3::new(4.0, -6e-7, 0.0);
        assert_box_covers_edge(&curve, start, end, 0.0, 1.0);
    }

    #[test]
    fn gate_box_is_exact_for_lines_and_tight_for_planar_circles() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(1.0, 1.0, 1.0);
        let line = conservative_curve_aabb(&EdgeCurve::Line, a, b).unwrap();
        assert_eq!(line, Aabb3 { min: a, max: b });
        // A circle in the z = 7 plane must not be padded along z: that is
        // what lets it stay clear of a face carrier in the z = 10 plane.
        let flat =
            Circle3D::new(Point3::new(50.0, 8.0, 7.0), Vec3::new(0.0, 0.0, 1.0), 3.0).unwrap();
        let p = flat.evaluate(0.0);
        let bbox = conservative_curve_aabb(&EdgeCurve::Circle(flat), p, p).unwrap();
        assert!(bbox.max.z() - 7.0 < 1e-12 && 7.0 - bbox.min.z() < 1e-12);
        assert!(bbox.min.x() <= 47.0 && bbox.max.x() >= 53.0);
        assert!(bbox.min.x() > 47.0 - 1e-12 && bbox.max.x() < 53.0 + 1e-12);
        let parabola =
            Parabola3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        assert!(conservative_curve_aabb(&EdgeCurve::Parabola(parabola), a, b).is_none());
    }

    /// A normal within ~1e-8 of a world axis rounds that component to exactly
    /// 1, so a bound read from `normal()` (`r * sqrt(1 - n_z^2)`) collapses to
    /// zero while the circle still reaches `r * tilt` along it: 1e-6 at
    /// r = 100. The box read from the evaluation frame must cover it, also
    /// far from the origin.
    #[test]
    fn gate_box_covers_circles_whose_normal_is_nearly_a_world_axis() {
        for (tilt, radius) in [
            (1e-8, 100.0),
            (1e-8, 1000.0),
            (1e-7, 1000.0),
            (1e-9, 1000.0),
        ] {
            for offset in [0.0, 1e6, 1e13] {
                let normal = Vec3::new(tilt, 0.0, 1.0).normalize().unwrap();
                let center = Point3::new(3.0 + offset, -2.0, 1.0 - offset);
                let circle = Circle3D::new(center, normal, radius).unwrap();
                let curve = EdgeCurve::Circle(circle.clone());
                assert_box_covers_full_turn(&curve);
                // Both z extremes, where the collapsed bound failed.
                let top = circle.v_axis().z().atan2(circle.u_axis().z());
                let seam = circle.evaluate(top);
                let bbox = conservative_curve_aabb(&curve, seam, seam).unwrap();
                let bbox = bbox.expanded(coordinate_slack(bbox));
                assert!(bbox.contains_point(circle.evaluate(top + std::f64::consts::PI)));
            }
        }
    }

    /// `with_axes` (and deserialization) keep whatever frame they are given:
    /// a `u` not perpendicular to the normal (as wasm `makeCircleArc3d` builds
    /// from `start - center`), a `v = n x u` that is then not unit, a
    /// stretched `u`, or a non-orthogonal pair. Evaluation uses that frame,
    /// so the box must too.
    #[test]
    fn gate_box_covers_circles_and_ellipses_with_skewed_frames() {
        let center = Point3::new(1.0, 2.0, -3.0);
        let z = Vec3::new(0.0, 0.0, 1.0);
        let slanted = Vec3::new(1.0, 0.0, 1.0) * std::f64::consts::FRAC_1_SQRT_2;
        let frames = [
            (slanted, z.cross(slanted)),
            (Vec3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)),
            (Vec3::new(0.6, 0.8, 0.0), Vec3::new(0.8, 0.6, 0.0)),
        ];
        for (u, v) in frames {
            let circle = Circle3D::with_axes(center, z, 2.0, u, v).unwrap();
            assert_box_covers_full_turn(&EdgeCurve::Circle(circle));
            let ellipse = Ellipse3D::with_axes(center, z, 3.0, 1.0, u, v).unwrap();
            assert_box_covers_full_turn(&EdgeCurve::Ellipse(ellipse));
        }
        // The slanted circle rises r / sqrt(2) out of its nominal plane.
        let circle = Circle3D::with_axes(center, z, 2.0, slanted, z.cross(slanted)).unwrap();
        let p = circle.evaluate(0.0);
        let bbox = conservative_curve_aabb(&EdgeCurve::Circle(circle), p, p).unwrap();
        assert!(bbox.max.z() >= -3.0 + std::f64::consts::SQRT_2);
    }

    /// Exact extents: an ellipse with semi-axes 3 and 1 along x and y has the
    /// half-widths 3 and 1, scaled up by the documented `1 + 8 eps`; turned
    /// 45 degrees, each axis gets `hypot(3 / sqrt 2, 1 / sqrt 2) = sqrt 5`.
    #[test]
    fn gate_box_extent_is_the_frame_hypot_rounded_up() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let z = Vec3::new(0.0, 0.0, 1.0);
        let grow = 1.0 + 8.0 * f64::EPSILON;
        let boxed = |u: Vec3, v: Vec3| {
            let ellipse = Ellipse3D::with_axes(origin, z, 3.0, 1.0, u, v).unwrap();
            let p = ellipse.evaluate(0.0);
            conservative_curve_aabb(&EdgeCurve::Ellipse(ellipse), p, p).unwrap()
        };
        let bbox = boxed(Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
        assert_eq!(bbox.max, Point3::new(3.0 * grow, grow, 0.0));
        assert_eq!(bbox.min, Point3::new(-3.0 * grow, -grow, 0.0));
        let d = std::f64::consts::FRAC_1_SQRT_2;
        let bbox = boxed(Vec3::new(d, d, 0.0), Vec3::new(-d, d, 0.0));
        let expected = (3.0 * d).hypot(d) * grow;
        assert_eq!(bbox.max, Point3::new(expected, expected, 0.0));
        assert!((expected - 5.0_f64.sqrt()).abs() < 1e-14);
    }

    fn edited_circle(circle: &Circle3D, edit: impl Fn(&mut serde_json::Value)) -> Circle3D {
        let mut json = serde_json::to_value(circle).unwrap();
        edit(&mut json);
        serde_json::from_value(json).unwrap()
    }

    fn edited_nurbs(curve: &NurbsCurve, edit: impl Fn(&mut serde_json::Value)) -> NurbsCurve {
        let mut json = serde_json::to_value(curve).unwrap();
        edit(&mut json);
        serde_json::from_value(json).unwrap()
    }

    /// Data no validating constructor would build (it still arrives through
    /// deserialization, or through the radius check's NaN gap) gets no box,
    /// so no gate can prune on it.
    #[test]
    fn gate_box_declines_non_finite_and_degenerate_curves() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let nan = Point3::new(1.0, f64::NAN, 0.0);
        let inf = Point3::new(f64::INFINITY, 0.0, 0.0);
        assert!(conservative_curve_aabb(&EdgeCurve::Line, a, nan).is_none());
        assert!(conservative_curve_aabb(&EdgeCurve::Line, inf, a).is_none());

        let z = Vec3::new(0.0, 0.0, 1.0);
        let circle = Circle3D::new(a, z, 1.0).unwrap();
        let p = circle.evaluate(0.0);
        let boxed =
            |c: Circle3D, end: Point3| conservative_curve_aabb(&EdgeCurve::Circle(c), p, end);
        assert!(boxed(circle.clone(), p).is_some());
        assert!(boxed(circle.clone(), nan).is_none());
        assert!(boxed(Circle3D::new(a, z, f64::NAN).unwrap(), p).is_none());
        assert!(boxed(Circle3D::new(a, z, f64::MAX).unwrap(), p).is_none());
        assert!(boxed(edited_circle(&circle, |c| c["radius"] = (-1.0).into()), p).is_none());

        let nurbs = rational_cubic();
        let (s, e) = (Point3::new(0.0, 0.0, 0.0), Point3::new(4.0, 0.0, 0.0));
        let boxed = |n: NurbsCurve| conservative_curve_aabb(&EdgeCurve::NurbsCurve(n), s, e);
        assert!(boxed(nurbs.clone()).is_some());
        assert!(boxed(edited_nurbs(&nurbs, |n| n["weights"][2] = 0.0.into())).is_none());
        assert!(boxed(edited_nurbs(&nurbs, |n| n["weights"][2] = (-0.5).into())).is_none());
        // A NaN after the first point: a plain min/max scan skips it.
        assert!(finite_aabb([a, a, nan]).is_none());
        assert!(finite_aabb([a, inf]).is_none());
        let unit = finite_aabb([a, Point3::new(1.0, 2.0, 3.0)]).unwrap();
        assert_eq!(unit.max, Point3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn coordinate_slack_scales_with_the_largest_coordinate() {
        let boxed = |min: [f64; 3], max: [f64; 3]| Aabb3 {
            min: Point3::new(min[0], min[1], min[2]),
            max: Point3::new(max[0], max[1], max[2]),
        };
        let eps = f64::EPSILON;
        assert_eq!(
            coordinate_slack(boxed([-1.0, 0.0, 0.0], [0.0, 0.5, 0.0])),
            64.0 * eps
        );
        assert_eq!(
            coordinate_slack(boxed([0.0, -3e13, 0.0], [1.0, 2e13, 0.0])),
            64.0 * eps * 3e13
        );
        assert_eq!(coordinate_slack(boxed([0.0; 3], [0.0; 3])), 0.0);
        let poisoned = coordinate_slack(boxed([0.0, 0.0, f64::NAN], [1.0; 3]));
        assert_eq!(poisoned, f64::INFINITY);
    }

    fn pt(c: [f64; 3]) -> Point3 {
        Point3::new(c[0], c[1], c[2])
    }

    fn segment(a: [f64; 3], b: [f64; 3]) -> EdgeReach {
        EdgeReach::Segment(pt(a), pt(b))
    }

    fn hull(min: [f64; 3], max: [f64; 3]) -> EdgeReach {
        EdgeReach::Hull(Aabb3 {
            min: pt(min),
            max: pt(max),
        })
    }

    /// Closed-form distance intervals from the z axis: a segment through the
    /// axis (nearest 0, farthest its far end), one parallel to it (constant),
    /// a skew one (nearest its foot), a disc, and a hull whose points are
    /// `(x, 0, z)` with `x` in `[1, 2]`.
    #[test]
    fn axis_distance_intervals_match_closed_forms() {
        let o = pt([0.0; 3]);
        let z = Vec3::new(0.0, 0.0, 1.0);
        let through = segment([-3.0, 0.0, 1.0], [4.0, 0.0, 2.0]);
        assert_eq!(through.axis_distance(o, z), Some((0.0, 4.0)));
        let parallel = segment([2.0, 0.0, -5.0], [2.0, 0.0, 5.0]);
        assert_eq!(parallel.axis_distance(o, z), Some((2.0, 2.0)));
        let skew = segment([-3.0, 1.0, 0.0], [3.0, 1.0, 5.0]);
        assert_eq!(skew.axis_distance(o, z), Some((1.0, 10.0_f64.sqrt())));
        let near_end = segment([3.0, 4.0, 0.0], [6.0, 8.0, 1.0]);
        assert_eq!(near_end.axis_distance(o, z), Some((5.0, 10.0)));
        let disc = EdgeReach::Disc {
            center: pt([5.0, 0.0, 7.0]),
            reach: 1.0,
        };
        assert_eq!(disc.axis_distance(o, z), Some((4.0, 6.0)));
        let slab = hull([1.0, 0.0, 0.0], [2.0, 0.0, 10.0]);
        assert_eq!(slab.axis_distance(o, z), Some((1.0, 2.0)));
        // The hull's lower bound is its centre less the farthest corner
        // offset across the axis: 5 - hypot(3, 4) for a 6 x 8 x 2 box.
        let block = hull([2.0, -4.0, 0.0], [8.0, 4.0, 2.0]);
        assert_eq!(block.axis_distance(o, z), Some((0.0, 80.0_f64.sqrt())));
        // A segment along a tilted axis lies on it: both bounds vanish.
        let d = 3.0_f64.sqrt().recip();
        let tilted = Vec3::new(d, d, d);
        let on_axis = segment([1.0, 0.0, 0.0], [3.0, 2.0, 2.0]);
        let (lo, hi) = on_axis.axis_distance(pt([1.0, 0.0, 0.0]), tilted).unwrap();
        assert!(lo.abs() < 1e-15 && hi.abs() < 1e-15, "{lo} {hi}");
    }

    #[test]
    fn point_distance_intervals_match_closed_forms() {
        let o = pt([0.0; 3]);
        let chord = segment([-3.0, 4.0, 0.0], [3.0, 4.0, 0.0]);
        assert_eq!(chord.point_distance(o), Some((4.0, 5.0)));
        let past_end = segment([3.0, 4.0, 0.0], [6.0, 8.0, 0.0]);
        assert_eq!(past_end.point_distance(o), Some((5.0, 10.0)));
        let point = segment([3.0, 4.0, 0.0], [3.0, 4.0, 0.0]);
        assert_eq!(point.point_distance(o), Some((5.0, 5.0)));
        let disc = EdgeReach::Disc {
            center: pt([0.0, 0.0, 10.0]),
            reach: 2.0,
        };
        assert_eq!(disc.point_distance(o), Some((8.0, 12.0)));
        let rod = hull([3.0, 0.0, 0.0], [4.0, 0.0, 0.0]);
        assert_eq!(rod.point_distance(o), Some((3.0, 4.0)));
        let around = hull([-1.0, -2.0, -2.0], [1.0, 2.0, 2.0]);
        assert_eq!(around.point_distance(o), Some((0.0, 3.0)));
    }

    /// The clearance is the gap outside (`lo - r`) or inside (`r - hi`) the
    /// carrier, net of rounding and frame allowances (`~4e-12 r` at unit
    /// scale).
    /// Touching and straddling reaches have no clearance.
    #[test]
    fn carrier_clearance_is_the_outside_or_inside_gap() {
        let o = pt([0.0; 3]);
        let z = Vec3::new(0.0, 0.0, 1.0);
        let cylinder = |radius: f64| CarrierGate::Cylinder {
            origin: o,
            axis: z,
            radius,
        };
        let disc = |center: [f64; 3], reach: f64| EdgeReach::Disc {
            center: pt(center),
            reach,
        };
        let near = |value: Option<f64>, expected: f64| {
            let value = value.unwrap();
            assert!((value - expected).abs() < 1e-10, "{value} vs {expected}");
        };
        near(cylinder(2.0).clearance(&disc([5.0, 0.0, 0.0], 1.0)), 2.0);
        near(
            cylinder(10.0).clearance(&segment([-3.0, 0.0, 1.0], [3.0, 0.0, 1.0])),
            7.0,
        );
        // Tangent at exactly the radius, and a coaxial rim of the same radius
        // (a counterbore's wall carrying the other operand's rim).
        near(
            cylinder(2.0).clearance(&segment([-3.0, 2.0, 0.0], [3.0, 2.0, 0.0])),
            0.0,
        );
        near(cylinder(2.0).clearance(&disc([0.0, 0.0, 5.0], 2.0)), 0.0);
        near(cylinder(2.0).clearance(&disc([0.0, 0.0, 5.0], 1.0)), 1.0);
        near(
            cylinder(2.0).clearance(&segment([-3.0, 0.0, 0.0], [3.0, 0.0, 0.0])),
            -1.0,
        );
        let sphere = CarrierGate::Sphere {
            center: o,
            radius: 3.0,
        };
        near(
            sphere.clearance(&segment([5.0, -1.0, 0.0], [5.0, 1.0, 0.0])),
            2.0,
        );
        near(sphere.clearance(&disc([0.0, 0.0, 0.5], 1.0)), 1.5);
        near(
            sphere.clearance(&hull([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0])),
            3.0 - 3.0_f64.sqrt(),
        );
        near(
            sphere.clearance(&segment([-5.0, 0.0, 0.0], [5.0, 0.0, 0.0])),
            -2.0,
        );
        // Far out, the rounding allowance outweighs a 1 mm gap.
        let far = pt([1e13, -1e13, 1e13]);
        let far_cylinder = CarrierGate::Cylinder {
            origin: far,
            axis: z,
            radius: 2.0,
        };
        let far_disc = EdgeReach::Disc {
            center: Point3::new(far.x() + 4.001, far.y(), far.z()),
            reach: 2.0,
        };
        assert!(far_cylinder.clearance(&far_disc).unwrap() < 0.0);
    }

    /// The reach of a circle or ellipse holds for any frame `with_axes`
    /// keeps, and is tight for an orthonormal one.
    #[test]
    fn edge_reach_bounds_skewed_frames() {
        let center = pt([1.0, 2.0, -3.0]);
        let z = Vec3::new(0.0, 0.0, 1.0);
        let slanted = Vec3::new(1.0, 0.0, 1.0) * std::f64::consts::FRAC_1_SQRT_2;
        // (u, v, closed-form reach of a radius-2 circle on that frame)
        let frames = [
            (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 2.0),
            (slanted, z.cross(slanted), 2.0),
            (Vec3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 4.0),
            (
                Vec3::new(0.6, 0.8, 0.0),
                Vec3::new(0.8, 0.6, 0.0),
                2.0 * 1.4,
            ),
        ];
        let zero = pt([0.0; 3]);
        for (u, v, expected) in frames {
            let curve = EdgeCurve::Circle(Circle3D::with_axes(center, z, 2.0, u, v).unwrap());
            let Some(EdgeReach::Disc { reach, .. }) = EdgeReach::of(&curve, zero, zero) else {
                panic!("a circle reaches a disc");
            };
            assert!((reach - expected).abs() < 1e-12, "{reach} vs {expected}");
            for i in 0..=4096 {
                let p = curve.evaluate_with_endpoints(f64::from(i) * TAU / 4096.0, zero, zero);
                assert!((p - center).length() <= reach, "{p:?}");
            }
            let ellipse = Ellipse3D::with_axes(center, z, 3.0, 1.0, u, v).unwrap();
            let curve = EdgeCurve::Ellipse(ellipse);
            let Some(EdgeReach::Disc { reach, .. }) = EdgeReach::of(&curve, zero, zero) else {
                panic!("an ellipse reaches a disc");
            };
            for i in 0..=4096 {
                let p = curve.evaluate_with_endpoints(f64::from(i) * TAU / 4096.0, zero, zero);
                assert!((p - center).length() <= reach, "{p:?}");
            }
        }
        let nurbs = EdgeCurve::NurbsCurve(rational_cubic());
        assert!(matches!(
            EdgeReach::of(&nurbs, zero, zero),
            Some(EdgeReach::Hull(_))
        ));
        let parabola =
            Parabola3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        assert!(EdgeReach::of(&EdgeCurve::Parabola(parabola), zero, zero).is_none());
        let nan = pt([f64::NAN, 0.0, 0.0]);
        assert!(EdgeReach::of(&EdgeCurve::Line, zero, nan).is_none());
        let nan_radius = Circle3D::new(center, z, f64::NAN).unwrap();
        assert!(EdgeReach::of(&EdgeCurve::Circle(nan_radius), zero, zero).is_none());
    }

    /// Carriers that skipped the constructors' checks (deserialized, or
    /// through the radius check's NaN gap) are never gated: a negative
    /// radius would turn `lo - r` into `lo + |r|` and prune real crossings.
    #[test]
    fn carrier_gate_declines_malformed_carriers() {
        use remus_math::surfaces::{CylindricalSurface, SphericalSurface};

        let origin = pt([1.0, 2.0, 3.0]);
        let z = Vec3::new(0.0, 0.0, 1.0);
        let cylinder = CylindricalSurface::new(origin, Vec3::new(1.0, 1.0, 0.5), 2.0).unwrap();
        let sphere = SphericalSurface::with_axis(origin, 2.0, Vec3::new(0.3, 0.0, 1.0)).unwrap();
        assert!(CarrierGate::of(&FaceSurface::Cylinder(cylinder.clone())).is_some());
        assert!(CarrierGate::of(&FaceSurface::Sphere(sphere.clone())).is_some());

        let edited_cylinder = |edit: &dyn Fn(&mut serde_json::Value)| {
            let mut json = serde_json::to_value(&cylinder).unwrap();
            edit(&mut json);
            let c: CylindricalSurface = serde_json::from_value(json).unwrap();
            CarrierGate::of(&FaceSurface::Cylinder(c))
        };
        assert!(edited_cylinder(&|c| c["radius"] = (-2.0).into()).is_none());
        assert!(edited_cylinder(&|c| c["axis"] = serde_json::json!([0.0, 0.0, 2.0])).is_none());
        assert!(edited_cylinder(&|c| c["x_axis"] = serde_json::json!([1.0, 0.0, 0.1])).is_none());
        let mut json = serde_json::to_value(&sphere).unwrap();
        json["z_axis"] = serde_json::json!([0.0, 0.6, 0.6]);
        let skewed: SphericalSurface = serde_json::from_value(json).unwrap();
        assert!(CarrierGate::of(&FaceSurface::Sphere(skewed)).is_none());

        let nan_radius = CylindricalSurface::new(origin, z, f64::NAN).unwrap();
        assert!(CarrierGate::of(&FaceSurface::Cylinder(nan_radius)).is_none());
        let nan_origin = CylindricalSurface::new(pt([f64::NAN, 0.0, 0.0]), z, 1.0).unwrap();
        assert!(CarrierGate::of(&FaceSurface::Cylinder(nan_origin)).is_none());
        let nan_center = SphericalSurface::new(pt([0.0, f64::NAN, 0.0]), 1.0).unwrap();
        assert!(CarrierGate::of(&FaceSurface::Sphere(nan_center)).is_none());
        let plane = FaceSurface::Plane { normal: z, d: 1.0 };
        assert!(CarrierGate::of(&plane).is_none());
        let cone = remus_math::surfaces::ConicalSurface::new(origin, z, 0.5).unwrap();
        assert!(CarrierGate::of(&FaceSurface::Cone(cone)).is_none());
    }
}
