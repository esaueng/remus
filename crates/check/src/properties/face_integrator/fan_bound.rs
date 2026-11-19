//! How far a planar face's chord-polygon reading may be from the face's.
//!
//! [`super::integrate_planar_face_fan`] reads a planar face the exact
//! Green's-theorem reader declines as the polygon of its sampled boundary
//! ([`crate::util::wire_polygon`]): [`CLOSED_CURVE_SAMPLES`] chords for a
//! closed curved edge, [`OPEN_CURVE_SAMPLES`] for an open one, a line as its
//! own chord. Its reading does not depend on any quadrature order, so no
//! order comparison can tell how far it is off. This module bounds it from
//! the boundary's own geometry instead ([`PlanarFanBound`]).
//!
//! # The polygon bounded is the polygon integrated
//!
//! Each wire's sample sequence is rebuilt here with the sampler's own rules
//! (traversal direction, seam, parameters, orientation, the half-open
//! convention), recording for every sample the piece of curve that follows
//! it. The rebuilt points must equal the sampler's output bit for bit;
//! otherwise the bound is `+∞`.
//!
//! # The area bound
//!
//! The true region and the polygon differ by the slivers between each chord
//! and the piece of curve it cuts off, so `|A_true − A_fan| ≤ Σ |sliver|`
//! (holes and islands included, whatever their sign). Take the chord as the
//! `x` axis of its plane: the sliver's area is `|∫ y dx|` along the piece,
//! at most `g · ℓ`, where `g` bounds the piece's distance from the chord's
//! line and `ℓ` its arc length (which bounds `∫ |dx|`). Distances and
//! lengths are taken in 3D, never smaller than in the plane.
//!
//! * A conic piece of parameter length `Δ` with `|P'| ≤ M₁` and
//!   `|P''| ≤ M₂` on it: the curve's offset from the linear interpolant of
//!   its ends is `½ (t − a)(b − t)` times a weighted mean of `P''`, so
//!   `g ≤ Δ² M₂ / 8`, and `ℓ ≤ Δ M₁`. A circle of radius `r` has
//!   `M₁ = M₂ = r`; an ellipse `√(a² + b²)`; a hyperbola that times
//!   `cosh max|t|`; a parabola `1 + max|t| / 2f` and `1 / 2f`.
//! * A NURBS piece with positive weights, split at its knots and each span
//!   restricted to the piece (and halved) by blossoming, lies in the hull of
//!   its rational Bézier control points, so `g` is their largest distance
//!   from the chord's line. Rational de Casteljau subdivision cuts corners
//!   (each new point lies on a leg of the old polygon), so the control
//!   polygon's length bounds `ℓ`. A closed edge's chord that crosses the end
//!   of its domain adds the straight jump from the domain's end back to its
//!   start. Weights are divided by their largest before homogenising, and
//!   every control point carries the blossom's rounding bound
//!   `(8p + 16) ε κ M` (`κ` the weight ratio, `M` the local magnitude).
//! * Where an edge's last piece ends at a point `e` other than the next
//!   sample `q` the fan joins it to (an edge end within tolerance of the
//!   shared vertex), the fan's chord adds the triangle of its last sample,
//!   `e` and `q`, at most `|e − q| · |e − l| / 2`, and the true boundary's
//!   own gap at most `|e − q|²`.
//! * Every sample is within the curve evaluation's rounding of the curve:
//!   `(8p + 16) ε κ M` for a NURBS edge, `64 ε M` for a conic (`M` the
//!   absolute coordinates' L1 magnitude of its control points, or of its
//!   centre and samples), which moves the polygon's area by at most twice
//!   that times the edge's chord length. That term depends on where the
//!   face is, so it is kept as a coefficient
//!   ([`PlanarFanBound::rounding_length`]) and scaled by the face's own
//!   magnitude, its first vertex's plus `√3` times
//!   [`PlanarFanBound::reach`], where it is used
//!   ([`PlanarFanBound::area_error`]).
//!
//! The bound is `+∞` (no certificate) for a NURBS edge with a weight that is
//! not finite and positive or a weight ratio past [`MAX_WEIGHT_RATIO`], a
//! parameter outside its domain, a closed NURBS edge whose stored trim is
//! not its whole domain (the sampler reads the whole carrier), a closed
//! edge the sampler reads without a finite domain, and a rebuilt sequence
//! that differs from the sampler's.
//!
//! # Off the plane
//!
//! The fan integrates its samples where they are. With `h` the largest
//! distance of a sample from the face's stored plane and `Σ |A_k|` the
//! unsigned area of the fan's triangles, its volume term is off the face's
//! by at most `(|δ| · area_bound + h · Σ |A_k|) / 3`, `δ` the reference's
//! distance from the plane ([`PlanarFanBound::volume_bound`]).

use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::wire::WireId;

use crate::CheckError;
use crate::util::{CLOSED_CURVE_SAMPLES, OPEN_CURVE_SAMPLES};

const EPS: f64 = f64::EPSILON;

/// Relative rounding margin of [`PlanarFanBound::volume_bound`].
const MARGIN: f64 = 64.0 * EPS;

/// Largest weight ratio `w_max / w_min` a NURBS edge is bounded at. At
/// `1e15` the evaluation's rounding bound `κ ε` reaches a fifth of the
/// curve's own magnitude, so nothing smaller could be certified; below it
/// every weight divided by the largest is at least `1e-15`, so homogeneous
/// coordinates underflow only where they are below `1e-293`.
const MAX_WEIGHT_RATIO: f64 = 1e15;

/// How far a planar face's chord-polygon reading may be from the face's.
///
/// Every field depends on the face's shape alone, not on its position or
/// on a reference point, so a translated face's certificate equals this
/// one's to rounding, which [`Self::volume_bound`]'s margin covers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarFanBound {
    /// Bound on `|A_polygon − A_face|`; `+∞` where none is certified.
    pub area_bound: f64,
    /// The largest distance of a sample from the face's stored plane, times
    /// [`Self::unsigned_area`].
    pub off_plane: f64,
    /// `Σ |A_k|` over the fan's triangles, outer and inner wires.
    pub unsigned_area: f64,
    /// The diagonal of the box holding every sample.
    pub diameter: f64,
    /// Area the samples' evaluation rounding may move the polygon by, per
    /// unit of absolute coordinate magnitude: `Σ 2 ρ L` over the curved
    /// edges, `ρ` the evaluation's relative rounding bound and `L` the
    /// edge's chord length.
    pub rounding_length: f64,
    /// The largest distance from the face's first vertex of a sample, a
    /// NURBS control point or a conic's centre: with that vertex's
    /// magnitude it bounds the magnitude every evaluation works at.
    pub reach: f64,
    /// The largest distance of a sample from the face's stored plane.
    pub height: f64,
    /// `Σ (L/2 + m·D)` over the wires' polygons (`L` the perimeter, `m` the
    /// points, `D` the [`Self::diameter`]): a perturbation of every point by
    /// at most `d` moves the fan's triangles by `Σ |ΔA_k| ≤ d` times this.
    pub perturbation_extent: f64,
    /// How many triangles the fan sums.
    pub triangles: f64,
}

impl PlanarFanBound {
    /// Bound on `|A_polygon − A_face|` for the face whose first vertex has
    /// absolute coordinates of L1 magnitude `anchor_magnitude`.
    #[must_use]
    pub fn area_error(&self, anchor_magnitude: f64) -> f64 {
        let magnitude = 3.0_f64.sqrt().mul_add(self.reach, anchor_magnitude.abs());
        self.rounding_length.mul_add(magnitude, self.area_bound)
    }

    /// Bound on how far the fan's volume term is from the face's, about a
    /// reference `plane_offset` from the face's stored plane and
    /// `anchor_distance` from the face's first vertex, whose absolute
    /// coordinates have L1 magnitude `anchor_magnitude`; `+∞` where none is
    /// certified.
    ///
    /// The fan sums `Σ (n̂ · p̃_k) Ã_k`, with `p̃_k` its computed triangle
    /// centroids about the reference and `Ã_k` its computed triangle areas.
    /// Every sample is subtracted from the reference before anything else
    /// (`integrate_planar_polygon`), so each local point carries a rounding
    /// of at most `ε |p − R|`, and each centroid `2 ε |p − R|`: with
    /// `r = anchor_distance + diameter` bounding `|p − R|`, `d = 2 ε r`.
    /// Against the face's term `δ · A_face`, the error is at most
    ///
    /// * `|δ| · area_error`, the polygon against the face;
    /// * `height · Σ |A_k|`, the samples off the plane;
    /// * `(|δ| + height + d) · Σ |ΔA_k|`, where the computed areas move by
    ///   `Σ |ΔA_k| ≤ d · perturbation_extent + 4 ε (D + 2d)² · triangles`
    ///   (the perturbed points, then each cross product's rounding);
    /// * `d · Σ |A_k|`, the centroids' rounding,
    ///
    /// all over three. That is inflated by `MARGIN` (64ε), and
    /// `MARGIN · r · Σ |A_k|` covers the rounding of the fan's own sums and
    /// of a translated cache hit (whose fields equal a fresh reading's to
    /// rounding of the local geometry).
    #[must_use]
    pub fn volume_bound(
        &self,
        plane_offset: f64,
        anchor_distance: f64,
        anchor_magnitude: f64,
    ) -> f64 {
        let reach = anchor_distance.abs() + self.diameter;
        let d = 2.0 * EPS * reach;
        let spread = 2.0f64.mul_add(d, self.diameter);
        let moved = d.mul_add(
            self.perturbation_extent,
            4.0 * EPS * spread * spread * self.triangles,
        );
        let offset = plane_offset.abs();
        let terms = offset * self.area_error(anchor_magnitude)
            + self.off_plane
            + (offset + self.height + d) * moved
            + d * self.unsigned_area;
        let rounding = MARGIN * reach * self.unsigned_area;
        let bound = (terms / 3.0).mul_add(1.0 + MARGIN, rounding);
        if bound.is_nan() { f64::INFINITY } else { bound }
    }

    const UNBOUNDED: Self = Self {
        area_bound: f64::INFINITY,
        off_plane: f64::INFINITY,
        unsigned_area: f64::INFINITY,
        diameter: f64::INFINITY,
        rounding_length: f64::INFINITY,
        reach: f64::INFINITY,
        height: f64::INFINITY,
        perturbation_extent: f64::INFINITY,
        triangles: f64::INFINITY,
    };
}

/// [`PlanarFanBound`] for the planar face `face_id`, read as
/// [`super::integrate_planar_face_fan`] reads it.
///
/// # Errors
///
/// Topology lookups and edge domains, exactly as the fan's own sampler.
pub(super) fn fan_bound(topo: &Topology, face_id: FaceId) -> Result<PlanarFanBound, CheckError> {
    let face = topo.face(face_id)?;
    let plane = match face.surface() {
        FaceSurface::Plane { normal, d } => {
            let length = normal.length();
            normal.normalize().ok().map(|unit| (unit, *d / length))
        }
        FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_)
        | FaceSurface::Nurbs(_) => None,
    };
    let Some((normal, level)) = plane.filter(|(_, level)| level.is_finite()) else {
        return Ok(PlanarFanBound::UNBOUNDED);
    };
    let first = topo
        .wire(face.outer_wire())?
        .edges()
        .first()
        .map(remus_topology::OrientedEdge::edge);
    let Some(first) = first else {
        return Ok(PlanarFanBound::UNBOUNDED);
    };
    let anchor = topo.vertex(topo.edge(first)?.start())?.point();
    let (mut area_bound, mut rounding_length, mut reach) = (0.0, 0.0, 0.0_f64);
    let (mut height, mut unsigned_area) = (0.0_f64, 0.0);
    // Per polygon: its perimeter and point count, read after the diameter.
    let mut outlines: Vec<(f64, f64)> = Vec::new();
    let mut triangles = 0.0;
    let (mut lo, mut hi) = (
        Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
        Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        // The polygon the fan integrates, and its rebuilt sequence.
        let polygon = crate::util::wire_polygon(topo, wire)?;
        let read = wire_bound(topo, wire, anchor)?;
        if let Some(read) = read.filter(|read| same_points(&read.points, &polygon)) {
            area_bound += read.bound;
            rounding_length += read.rounding_length;
            reach = reach.max(read.reach);
        } else {
            area_bound = f64::INFINITY;
        }
        for &p in &polygon {
            height = height.max((normal.dot(p - Point3::new(0.0, 0.0, 0.0)) - level).abs());
            lo = Point3::new(lo.x().min(p.x()), lo.y().min(p.y()), lo.z().min(p.z()));
            hi = Point3::new(hi.x().max(p.x()), hi.y().max(p.y()), hi.z().max(p.z()));
        }
        // The fan's own triangles from its first point.
        if polygon.len() >= 3 {
            for i in 1..polygon.len() - 1 {
                let (a, b, c) = (polygon[0], polygon[i], polygon[i + 1]);
                unsigned_area += ((b - a).cross(c - a).dot(normal) * 0.5).abs();
            }
            #[allow(clippy::cast_precision_loss)]
            {
                triangles += (polygon.len() - 2) as f64;
            }
        }
        let perimeter: f64 = (0..polygon.len())
            .map(|i| (polygon[(i + 1) % polygon.len()] - polygon[i]).length())
            .sum();
        #[allow(clippy::cast_precision_loss)]
        outlines.push((perimeter, polygon.len() as f64));
    }
    let diameter = if lo.x() <= hi.x() {
        (hi - lo).length()
    } else {
        0.0
    };
    let bound = PlanarFanBound {
        area_bound,
        off_plane: height * unsigned_area,
        height,
        perturbation_extent: outlines
            .iter()
            .map(|&(perimeter, points)| points.mul_add(diameter, 0.5 * perimeter))
            .sum(),
        triangles,
        unsigned_area,
        diameter,
        rounding_length,
        reach,
    };
    let finite = [
        bound.area_bound,
        bound.off_plane,
        bound.unsigned_area,
        bound.diameter,
        bound.rounding_length,
        bound.reach,
        bound.height,
        bound.perturbation_extent,
        bound.triangles,
    ]
    .iter()
    .all(|v| v.is_finite());
    Ok(if finite {
        bound
    } else {
        PlanarFanBound::UNBOUNDED
    })
}

fn same_points(a: &[Point3], b: &[Point3]) -> bool {
    a.len() == b.len()
        && a.iter().zip(b).all(|(p, q)| {
            p.x().to_bits() == q.x().to_bits()
                && p.y().to_bits() == q.y().to_bits()
                && p.z().to_bits() == q.z().to_bits()
        })
}

/// One edge's share of the rebuilt sequence: the samples it emits, the curve
/// point its last piece ends at, the bound on its own chords' slivers, its
/// rounding coefficient, and the further points its evaluation works at.
struct EdgeSamples {
    points: Vec<Point3>,
    end: Point3,
    bound: f64,
    rounding_length: f64,
    magnitude_points: Vec<Point3>,
}

/// A wire's rebuilt sequence, its sliver bound (joins included), rounding
/// coefficient, and reach from the anchor.
struct WireRead {
    points: Vec<Point3>,
    bound: f64,
    rounding_length: f64,
    reach: f64,
}

/// The rebuilt sample sequence of `wire` (the sampler's
/// `wire_polygon_curve_sampled_except` with [`CLOSED_CURVE_SAMPLES`],
/// [`OPEN_CURVE_SAMPLES`] and no chord list) and its area bound, joins
/// included; `None` where an edge cannot be bounded.
fn wire_bound(
    topo: &Topology,
    wire: WireId,
    anchor: Point3,
) -> Result<Option<WireRead>, CheckError> {
    let mut edges: Vec<EdgeSamples> = Vec::new();
    let mut prev_end: Option<remus_topology::vertex::VertexId> = None;
    for oe in topo.wire(wire)?.edges() {
        let edge = topo.edge(oe.edge())?;
        let (start_vid, end_vid) = (edge.start(), edge.end());
        let forward = match prev_end {
            Some(pe) if start_vid == pe && end_vid != pe => true,
            Some(pe) if end_vid == pe && start_vid != pe => false,
            Some(_) | None if start_vid == end_vid => oe.is_forward(),
            Some(previous_end) => {
                let last = topo.vertex(previous_end)?.point();
                let sp = topo.vertex(start_vid)?.point();
                let ep = topo.vertex(end_vid)?.point();
                (sp - last).length_squared() <= (ep - last).length_squared()
            }
            None => oe.is_forward(),
        };
        let closed = start_vid == end_vid
            && matches!(
                edge.curve(),
                EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) | EdgeCurve::NurbsCurve(_)
            );
        let samples = if closed {
            prev_end = Some(start_vid);
            closed_edge(topo, edge, forward)?
        } else {
            let (from_vid, to_vid) = if forward {
                (start_vid, end_vid)
            } else {
                (end_vid, start_vid)
            };
            prev_end = Some(to_vid);
            open_edge(topo, edge, from_vid, to_vid)?
        };
        let Some(samples) = samples else {
            return Ok(None);
        };
        edges.push(samples);
    }
    let points: Vec<Point3> = edges
        .iter()
        .flat_map(|e| e.points.iter().copied())
        .collect();
    let (mut bound, mut rounding_length, mut reach) = (0.0, 0.0, 0.0_f64);
    for (i, samples) in edges.iter().enumerate() {
        bound += samples.bound;
        rounding_length += samples.rounding_length;
        for &q in samples.points.iter().chain(&samples.magnitude_points) {
            reach = reach.max((q - anchor).length());
        }
        // The chord from the edge's last sample to the next sample.
        let Some(&last) = samples.points.last() else {
            return Ok(None);
        };
        let next = edges[(i + 1) % edges.len()].points.first().copied();
        let Some(next) = next else {
            return Ok(None);
        };
        let gap = (samples.end - next).length();
        if gap > 0.0 {
            bound += gap.mul_add(0.5 * (samples.end - last).length(), gap * gap);
        }
    }
    Ok(bound.is_finite().then_some(WireRead {
        points,
        bound,
        rounding_length,
        reach,
    }))
}

/// The sampler's closed-edge branch.
fn closed_edge(
    topo: &Topology,
    edge: &Edge,
    forward: bool,
) -> Result<Option<EdgeSamples>, CheckError> {
    let n = CLOSED_CURVE_SAMPLES;
    let seam_pt = topo.vertex(edge.start())?.point();
    // The sampler's parameter offsets, in traversal order, end included.
    #[allow(clippy::cast_precision_loss)]
    let offsets = |period: f64| -> Vec<f64> {
        (0..=n)
            .map(|k| {
                let i = if forward { k } else { n - k };
                period * (i as f64) / (n as f64)
            })
            .collect()
    };
    let (points, bound, rho) = match edge.curve() {
        EdgeCurve::Circle(c) => {
            let (t0, t1) = edge
                .strict_domain()
                .map_err(crate::error::edge_domain_validation)?;
            let seam = c.project(seam_pt);
            if !faithful(seam, seam + (t1 - t0), n) {
                return Ok(None);
            }
            let points: Vec<Point3> = offsets(t1 - t0)
                .into_iter()
                .map(|dt| c.evaluate(seam + dt))
                .collect();
            (points, conic_bound(edge.curve(), t0, t1, n), CONIC_ROUNDING)
        }
        EdgeCurve::Ellipse(e) => {
            let (t0, t1) = edge
                .strict_domain()
                .map_err(crate::error::edge_domain_validation)?;
            let seam = e.project(seam_pt);
            if !faithful(seam, seam + (t1 - t0), n) {
                return Ok(None);
            }
            let points: Vec<Point3> = offsets(t1 - t0)
                .into_iter()
                .map(|dt| e.evaluate(seam + dt))
                .collect();
            (points, conic_bound(edge.curve(), t0, t1, n), CONIC_ROUNDING)
        }
        EdgeCurve::NurbsCurve(nc) => {
            let (u0, u1) = nc.domain();
            let span = u1 - u0;
            // The sampler reads the whole carrier from its seam, whatever
            // the stored trim: a trim short of the domain is not what it
            // integrates (a follow-up for the sampler), so it is not bounded.
            let whole = edge.trim().is_none_or(|(a, b)| {
                a.min(b).to_bits() == u0.to_bits() && a.max(b).to_bits() == u1.to_bits()
            });
            if !(whole && span.is_finite() && span > 0.0 && faithful(u0, u1, n)) {
                return Ok(None);
            }
            let seam = crate::util::nurbs_seam_parameter(nc, seam_pt, u0, u1);
            let param = |dt: f64| u0 + (seam - u0 + dt).rem_euclid(span);
            let offsets = offsets(span);
            let points: Vec<Point3> = offsets.iter().map(|&dt| nc.evaluate(param(dt))).collect();
            // Every piece between consecutive samples, in increasing `u`.
            #[allow(clippy::cast_precision_loss)]
            let at = |i: usize| param(span * (i as f64) / (n as f64));
            let jump = (nc.evaluate(u1) - nc.evaluate(u0)).length();
            let mut bound = 0.0;
            for j in 0..n {
                let (a, b) = (at(j), at(j + 1));
                let chord = (nc.evaluate(a), nc.evaluate(b));
                bound += if a <= b {
                    hull_bound(nc, &[(a, b)], chord, 0.0)
                } else {
                    hull_bound(nc, &[(a, u1), (u0, b)], chord, jump)
                };
            }
            (points, bound, nurbs_rounding(nc))
        }
        // `closed` admits only the three curves above.
        EdgeCurve::Line | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => return Ok(None),
    };
    let rounding_length = 2.0 * rho * chord_length(&points);
    let magnitude_points = magnitude_points(edge.curve());
    let mut points = points;
    let end = points.pop();
    Ok(end.map(|end| EdgeSamples {
        points,
        end,
        bound,
        rounding_length,
        magnitude_points,
    }))
}

/// The sampler's open-edge branch, traversed from `from_vid` to `to_vid`.
fn open_edge(
    topo: &Topology,
    edge: &Edge,
    from_vid: remus_topology::vertex::VertexId,
    to_vid: remus_topology::vertex::VertexId,
) -> Result<Option<EdgeSamples>, CheckError> {
    let curve = edge.curve();
    if matches!(curve, EdgeCurve::Line) {
        return Ok(Some(EdgeSamples {
            points: vec![topo.vertex(from_vid)?.point()],
            end: topo.vertex(to_vid)?.point(),
            bound: 0.0,
            rounding_length: 0.0,
            magnitude_points: Vec::new(),
        }));
    }
    let n = OPEN_CURVE_SAMPLES;
    let start_pt = topo.vertex(edge.start())?.point();
    let end_pt = topo.vertex(edge.end())?.point();
    let (t0, t1) = edge
        .strict_domain()
        .map_err(crate::error::edge_domain_validation)?;
    if !faithful(t0, t1, n) {
        return Ok(None);
    }
    let traversal_start = topo.vertex(from_vid)?.point();
    #[allow(clippy::cast_precision_loss)]
    let mut seq: Vec<Point3> = (0..=n)
        .map(|i| {
            let t = (t1 - t0).mul_add(i as f64 / n as f64, t0);
            curve.evaluate_with_endpoints(t, start_pt, end_pt)
        })
        .collect();
    if (seq[0] - traversal_start).length_squared() > (seq[n] - traversal_start).length_squared() {
        seq.reverse();
    }
    let (bound, rho) = match curve {
        EdgeCurve::NurbsCurve(nc) => {
            let mut bound = 0.0;
            #[allow(clippy::cast_precision_loss)]
            for i in 0..n {
                let a = (t1 - t0).mul_add(i as f64 / n as f64, t0);
                let b = (t1 - t0).mul_add((i + 1) as f64 / n as f64, t0);
                let chord = (nc.evaluate(a), nc.evaluate(b));
                bound += hull_bound(nc, &[(a.min(b), a.max(b))], chord, 0.0);
            }
            (bound, nurbs_rounding(nc))
        }
        EdgeCurve::Line
        | EdgeCurve::Circle(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => (conic_bound(curve, t0, t1, n), CONIC_ROUNDING),
    };
    let rounding_length = 2.0 * rho * chord_length(&seq);
    let end = seq.pop();
    Ok(end.map(|end| EdgeSamples {
        points: seq,
        end,
        bound,
        rounding_length,
        magnitude_points: magnitude_points(curve),
    }))
}

/// A conic evaluation's relative rounding bound: every computed point is
/// within `64 ε` of the L1 magnitude of its centre (vertex) plus its own.
const CONIC_ROUNDING: f64 = 64.0 * EPS;

/// A NURBS evaluation's relative rounding bound, `(8p + 16) ε κ` of the
/// control points' L1 magnitude (`κ` the weight ratio); `+∞` past
/// [`MAX_WEIGHT_RATIO`].
fn nurbs_rounding(nc: &NurbsCurve) -> f64 {
    let Some(ratio) = weight_ratio(nc.weights()) else {
        return f64::INFINITY;
    };
    #[allow(clippy::cast_precision_loss)]
    let levels = nc.degree() as f64;
    8.0f64.mul_add(levels, 16.0) * EPS * ratio
}

/// The points besides its samples whose magnitude a curve's evaluation
/// works at: a NURBS curve's control points, a conic's centre (vertex).
fn magnitude_points(curve: &EdgeCurve) -> Vec<Point3> {
    match curve {
        EdgeCurve::NurbsCurve(nc) => nc.control_points().to_vec(),
        EdgeCurve::Circle(c) => vec![c.center()],
        EdgeCurve::Ellipse(e) => vec![e.center()],
        EdgeCurve::Hyperbola(h) => vec![h.center()],
        EdgeCurve::Parabola(p) => vec![p.vertex()],
        EdgeCurve::Line => Vec::new(),
    }
}

fn chord_length(points: &[Point3]) -> f64 {
    points.windows(2).map(|w| (w[1] - w[0]).length()).sum()
}

/// `w_max / w_min`, when every weight is finite and positive and the ratio
/// is at most [`MAX_WEIGHT_RATIO`].
fn weight_ratio(weights: &[f64]) -> Option<f64> {
    let (mut lo, mut hi) = (f64::INFINITY, 0.0_f64);
    for &w in weights {
        if !(w.is_finite() && w > 0.0) {
            return None;
        }
        lo = lo.min(w);
        hi = hi.max(w);
    }
    let ratio = hi / lo;
    (ratio <= MAX_WEIGHT_RATIO).then_some(ratio)
}

/// Whether the sampler's `n` samples over `(t0, t1)` are faithful to the
/// parameter, so the pieces between them cover exactly the trim the bound
/// reads:
///
/// * the span `t1 − t0` is exact (`t0 + (t1 − t0) = t1`), and the first and
///   last samples are `t0` and `t1` themselves: `1 − (−2^53)` rounds to
///   `2^53`, and the last sample then lands on `0`, short of the trim;
/// * the samples increase strictly in the trim's direction, so no two
///   collapse onto one parameter;
/// * the rounding of a parameter near `max(|t0|, |t1|)` is at most `1e-9`
///   of one sample step.
fn faithful(t0: f64, t1: f64, n: usize) -> bool {
    let span = t1 - t0;
    let magnitude = t0.abs().max(t1.abs());
    #[allow(clippy::cast_precision_loss)]
    let step = span.abs() / n as f64;
    let ends = (t0 + span).to_bits() == t1.to_bits()
        && sample(t0, t1, 0, n).to_bits() == t0.to_bits()
        && sample(t0, t1, n, n).to_bits() == t1.to_bits();
    let monotone = (0..n).all(|i| {
        let (a, b) = (sample(t0, t1, i, n), sample(t0, t1, i + 1, n));
        if span > 0.0 { a < b } else { a > b }
    });
    span.is_finite() && span != 0.0 && ends && monotone && magnitude * EPS <= 1e-9 * step
}

/// The sampler's `i`-th of `n` parameters over `(t0, t1)`.
#[allow(clippy::cast_precision_loss)]
fn sample(t0: f64, t1: f64, i: usize, n: usize) -> f64 {
    (t1 - t0).mul_add(i as f64 / n as f64, t0)
}

/// `Σ g · ℓ` over the `n` chords of a conic over `(t0, t1)`, from closed-form
/// bounds on its first and second derivatives.
fn conic_bound(curve: &EdgeCurve, t0: f64, t1: f64, n: usize) -> f64 {
    let mut bound = 0.0;
    for i in 0..n {
        let (a, b) = (sample(t0, t1, i, n), sample(t0, t1, i + 1, n));
        let delta = (b - a).abs();
        let reach = a.abs().max(b.abs());
        let (m1, m2) = match curve {
            EdgeCurve::Circle(c) => (c.radius(), c.radius()),
            EdgeCurve::Ellipse(e) => {
                let m = e.semi_major().hypot(e.semi_minor());
                (m, m)
            }
            EdgeCurve::Hyperbola(h) => {
                let m = h.semi_major().hypot(h.semi_minor()) * reach.cosh();
                (m, m)
            }
            EdgeCurve::Parabola(p) => {
                let inverse = 1.0 / (2.0 * p.focal_length().abs());
                (reach.mul_add(inverse, 1.0), inverse)
            }
            // A line is its own chord; NURBS edges take the hull bound.
            EdgeCurve::Line => (0.0, 0.0),
            EdgeCurve::NurbsCurve(_) => (f64::INFINITY, f64::INFINITY),
        };
        bound += delta.powi(3) * m1 * m2 / 8.0;
    }
    if bound.is_nan() { f64::INFINITY } else { bound }
}

/// `g · ℓ` for the curve over `ranges` against the line through `chord`:
/// `g` the largest distance of a span's rational Bézier control point from
/// that line plus their rounding, `ℓ` the control polygons' total length
/// plus that rounding and `extra`.
fn hull_bound(nc: &NurbsCurve, ranges: &[(f64, f64)], chord: (Point3, Point3), extra: f64) -> f64 {
    let (origin, end) = chord;
    let direction = end - origin;
    let along = direction.length();
    let distance = |q: Vec3| {
        if along > 0.0 {
            q.cross(direction).length() / along
        } else {
            q.length()
        }
    };
    let (mut g, mut length) = (0.0_f64, extra);
    for &(lo, hi) in ranges {
        let Some((pieces, err)) = bezier_pieces(nc, lo, hi, origin) else {
            return f64::INFINITY;
        };
        for control in pieces {
            for pair in control.windows(2) {
                length += (pair[1] - pair[0]).length() + 2.0 * err;
            }
            for &q in &control {
                g = g.max(distance(q) + err);
            }
        }
    }
    let bound = g * length;
    if bound.is_finite() {
        bound
    } else {
        f64::INFINITY
    }
}

/// The rational Bézier control points, relative to `origin`, of every knot
/// span of `nc` restricted to `(lo, hi)` and halved, with a bound on their
/// rounding; `None` when a weight is not finite and positive, the weight
/// ratio passes [`MAX_WEIGHT_RATIO`], the curve's data is malformed, or
/// `(lo, hi)` leaves its domain.
fn bezier_pieces(
    nc: &NurbsCurve,
    lo: f64,
    hi: f64,
    origin: Point3,
) -> Option<(Vec<Vec<Vec3>>, f64)> {
    let p = nc.degree();
    let knots = nc.knots();
    let points = nc.control_points();
    let weights = nc.weights();
    let n = points.len();
    if p == 0 || n <= p || knots.len() != n + p + 1 || weights.len() != n {
        return None;
    }
    if !(lo >= knots[p] && hi <= knots[n] && lo <= hi) {
        return None;
    }
    let ratio = weight_ratio(weights)?;
    if lo >= hi {
        return Some((Vec::new(), 0.0));
    }
    // Weights divided by the largest: every one in `[1 / ratio, 1]`.
    let scale = weights.iter().copied().fold(0.0_f64, f64::max);
    let mut magnitude = 0.0_f64;
    let homogeneous: Vec<[f64; 4]> = points
        .iter()
        .zip(weights)
        .map(|(&point, &w)| {
            let d = point - origin;
            magnitude = magnitude.max(d.x().abs() + d.y().abs() + d.z().abs());
            let w = w / scale;
            [d.x() * w, d.y() * w, d.z() * w, w]
        })
        .collect();
    #[allow(clippy::cast_precision_loss)]
    let levels = p as f64;
    // `p` blossom levels of convex combinations, then the division by a
    // weight at least `1 / ratio`, on local coordinates of `magnitude`.
    let err = 8.0f64.mul_add(levels, 16.0) * EPS * ratio * magnitude;
    let mut k = (knots[p..=n].partition_point(|&u| u <= lo) + p).saturating_sub(1);
    k = k.clamp(p, n - 1);
    let mut pieces = Vec::new();
    while k < n && knots[k] < hi {
        let (a, b) = (knots[k].max(lo), knots[k + 1].min(hi));
        if b > a {
            let window = &homogeneous[k - p..=k];
            // Halved once: a quadratic piece's middle control point stands
            // off its chord twice the sagitta, its halves' at most the
            // sagitta, which brings `g · ℓ` from three times the sliver to
            // about one and a half.
            let m = f64::midpoint(a, b);
            for (a, b) in [(a, m), (m, b)] {
                let mut control = Vec::with_capacity(p + 1);
                for i in 0..=p {
                    let q = blossom(window, knots, k, p, a, b, i);
                    if !(q[3].is_finite() && q[3] > 0.0) {
                        return None;
                    }
                    control.push(Vec3::new(q[0] / q[3], q[1] / q[3], q[2] / q[3]));
                }
                pieces.push(control);
            }
        }
        k += 1;
    }
    Some((pieces, err))
}

/// The Bézier control point `i` of span `k` restricted to `(a, b)`: the
/// blossom at `a` repeated `p − i` times and `b` repeated `i` times, by de
/// Boor's triangle over the span's own control points `window`.
fn blossom(
    window: &[[f64; 4]],
    knots: &[f64],
    k: usize,
    p: usize,
    a: f64,
    b: f64,
    i: usize,
) -> [f64; 4] {
    let mut d = window.to_vec();
    for r in 1..=p {
        let t = if r <= p - i { a } else { b };
        for j in (r..=p).rev() {
            let g = k - p + j;
            let (lo, hi) = (knots[g], knots[g + p + 1 - r]);
            let alpha = (t - lo) / (hi - lo);
            for c in 0..4 {
                d[j][c] = (d[j][c] - d[j - 1][c]).mul_add(alpha, d[j - 1][c]);
            }
        }
    }
    d[p]
}
