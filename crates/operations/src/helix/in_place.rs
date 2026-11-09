//! Exact in-place helical sweep: a planar profile carried by a screw motion
//! about an axis line, starting where the profile sits.
//!
//! At turn angle `φ ∈ [0, Φ]` (`Φ = 2π · turns`, right-handed about the unit
//! axis `a` through `O`) every profile point `P = O + z·a + r` (`r ⟂ a`) moves
//! to
//!
//! ```text
//! M_φ(P) = O + (z + pitch · φ / 2π)·a + cos φ · r + sin φ · (a × r)
//! ```
//!
//! The radius is implied by where the profile sits; nothing is teleported.
//!
//! ## Construction
//!
//! Every profile boundary edge is converted to its **exact** NURBS form
//! (lines, circular and elliptic arcs, parabolic and hyperbolic arcs, NURBS
//! sub-spans). A closed edge is split into two exact halves so every side
//! face is a four-sided patch. Each edge's side face is the tensor product of
//! that NURBS with the rotation written as rational quadratic arc spans
//! (weights `[1, cos(α/2), 1]`, middle control point scaled radially by
//! `1 / cos(α/2)`, Piegl & Tiller's surface of revolution), with the axial
//! rise added per control row: `0`, `h/2` and `h` at a span's start, middle
//! and end. NURBS are affine invariant and the rotation part has the same
//! weights in every row, so each surface point is exactly `M_φ(C(u))` for the
//! angle `φ` the arc span reaches — the rotation is exact, the surfaces are
//! round, not faceted — **except** that the rise is not linear in that angle.
//! That axial offset is the construction's only approximation; see
//! [`rise_deviation_bound`] for its bound and [`segments_for`] for how the
//! span count keeps it below tolerance.
//!
//! One multi-span surface per profile edge covers every turn (double interior
//! knots at span joints, like [`super::make_helix_curve`]). Each profile
//! vertex's rail is the same construction applied to one point, so its
//! control points are the surfaces' boundary rows, bit for bit. The start cap
//! is the profile's own plane and edges; the end cap is the profile moved by
//! the full motion `M_Φ`, a rigid transform, with exactly transformed edges.
//!
//! ## Refusals
//!
//! A zero or non-finite axis, a zero or non-finite pitch, a non-positive,
//! non-finite or too-large turn count, a non-planar profile, a profile that
//! touches, straddles, encircles or wraps around the axis, a profile the
//! motion does not cross transversally (it would fold through itself), and
//! coils that would overlap (see [`check_coil_clearance`]).

use std::f64::consts::TAU;

use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::knot_ops::curve_split;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use super::MAX_HELIX_SEGMENTS;
use crate::OperationsError;

/// Target for the rise deviation, relative to `|pitch|`.
const RISE_RELATIVE_TOLERANCE: f64 = 1e-6;

/// Safety factor on the leading-order rise deviation. The exact maximum of
/// the deviation over a span of angle `α ≤ π/2`, divided by the leading-order
/// term `|h| α² / (72√3)`, rises monotonically from 1 (as `α → 0`) to 1.0659
/// at `α = π/2`; the factor covers that whole range (pinned by
/// `rise_deviation_bound_covers_the_exact_maximum`).
const RISE_BOUND_SAFETY: f64 = 1.07;

/// Samples per profile piece for the geometric refusal checks.
const CHECK_SAMPLES: usize = 32;

/// Relative scale for the geometric refusal margins.
const CHECK_RELATIVE_MARGIN: f64 = 1e-9;

/// Bound on the axial deviation `|z(t) − h · θ(t) / α|` of one rational
/// quadratic arc span of angle `α` (`0 < α ≤ π/2`) that rises `h`.
///
/// The arc's control rows rise `0`, `h/2`, `h` with weights `1`, `cos(α/2)`,
/// `1`. Writing the span parameter as `s = 2t − 1 ∈ [−1, 1]` and
/// `τ = tan(α/4)`, the arc's angle from the span middle is exactly
/// `θ − α/2 = 2·atan(τ s)` while the rational rise is
/// `z/h − 1/2 = s (1 + τ²) / (2 (1 + τ² s²))`. Their difference is odd in
/// `s`, vanishes at both ends and the middle, and expands as
/// `(α²/48)(s − s³) + O(α⁴)`, which peaks at `s = 1/√3`:
///
/// ```text
/// max |z − h θ/α| = |h| α² / (72√3) · (1 + O(α²))
/// ```
///
/// [`RISE_BOUND_SAFETY`] covers the `O(α²)` excess for every `α ≤ π/2`.
/// This is an **approximation bound**: the rise is the one quantity the
/// rational construction cannot reproduce exactly, and every surface and rail
/// point sits within this distance of the exact screw-motion point at the
/// same angle, displaced purely along the axis.
#[must_use]
fn rise_deviation_bound(alpha: f64, rise: f64) -> f64 {
    RISE_BOUND_SAFETY * rise.abs() * alpha * alpha / (72.0 * 3.0_f64.sqrt())
}

/// The axial-rise tolerance for a given pitch: `1e-6 · |pitch|`, but never
/// tighter than the kernel's linear tolerance.
///
/// **Deliberately looser than the linear tolerance** once `|pitch| > 0.1`:
/// at pitch 5 the side faces may sit up to 5 µm off the exact screw surface
/// (the [`rise_deviation_bound`], an axial offset only), not 0.1 µm. The
/// span count grows with the cube root of `|pitch| / tolerance`, so holding
/// 1e-7 would take 257 spans a turn at pitch 5 and about 700 at pitch 100
/// instead of 70. Every span is control rows in each side face, so that
/// multiplies the STEP size (a 3-turn spring is ~0.7 MB at 70 spans a turn)
/// and the cost of every intersection, projection and tessellation against
/// the faces, for an error far below any manufacturing or modelling
/// significance. The relative target also keeps the span count independent
/// of scale; the floor stops a tiny pitch from demanding precision below
/// what the kernel resolves (fewer spans a turn below `|pitch| = 0.1`).
/// Topology is unaffected: rails are the faces' own boundary rows and the
/// end cap is exact, so edges lie on their faces exactly.
fn rise_tolerance(pitch: f64) -> f64 {
    (RISE_RELATIVE_TOLERANCE * pitch.abs()).max(Tolerance::new().linear)
}

/// The smallest total span count whose rise deviation stays within
/// `tolerance`, with every span at most a quarter turn.
///
/// With `n` spans over `Φ = 2π·turns` rising `H = pitch·turns`, each span has
/// `α = Φ/n` and `h = H/n`, so [`rise_deviation_bound`] is
/// `C·|pitch|·turns·Φ²/n³`: the deviation falls with the cube of the span
/// count. Spans per turn are therefore `∛(4π²·C·|pitch| / tolerance)` with
/// `C` the bound's constant — 70 for the relative tolerance `1e-6`.
///
/// # Errors
///
/// [`OperationsError::InvalidInput`] when the count exceeds
/// [`MAX_HELIX_SEGMENTS`].
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
fn segments_for(pitch: f64, turns: f64, tolerance: f64) -> Result<usize, OperationsError> {
    let constant = RISE_BOUND_SAFETY / (72.0 * 3.0_f64.sqrt());
    let per_turn = (TAU * TAU * constant * pitch.abs() / tolerance).cbrt();
    let estimate = (turns * per_turn).ceil().max((turns * 4.0).ceil()).max(1.0);
    let too_many = || OperationsError::InvalidInput {
        reason: format!(
            "helical sweep needs too many spans: requested {estimate}, maximum is \
             {MAX_HELIX_SEGMENTS}"
        ),
    };
    if !estimate.is_finite() || estimate > MAX_HELIX_SEGMENTS as f64 {
        return Err(too_many());
    }
    let total = TAU * turns;
    let rise = pitch * turns;
    // The estimate is the closed form; step past any rounding at the
    // boundary so the returned count provably meets the bound.
    (estimate as usize..=MAX_HELIX_SEGMENTS)
        .find(|&n| rise_deviation_bound(total / n as f64, rise / n as f64) <= tolerance)
        .ok_or_else(too_many)
}

/// One control row of the screw direction.
#[derive(Debug, Clone, Copy)]
struct Row {
    cos: f64,
    sin: f64,
    /// Radial scale: 1 on span ends, `1 / cos(α/2)` on span middles.
    scale: f64,
    rise: f64,
    weight: f64,
}

/// The screw direction's control rows, knots and weights.
struct Screw {
    origin: Point3,
    axis: Vec3,
    rows: Vec<Row>,
    knots: Vec<f64>,
}

impl Screw {
    #[allow(clippy::cast_precision_loss)]
    fn new(origin: Point3, axis: Vec3, pitch: f64, turns: f64, segments: usize) -> Self {
        let total = TAU * turns;
        let height = pitch * turns;
        let n = segments as f64;
        let half = 0.5 * total / n;
        let weight = half.cos();
        let mut rows = Vec::with_capacity(2 * segments + 1);
        let row = |fraction: f64, scale: f64, weight: f64| {
            let (sin, cos) = (total * fraction).sin_cos();
            Row {
                cos,
                sin,
                scale,
                rise: height * fraction,
                weight,
            }
        };
        for k in 0..segments {
            let k = k as f64;
            rows.push(row(k / n, 1.0, 1.0));
            rows.push(row((k + 0.5) / n, 1.0 / weight, weight));
        }
        rows.push(row(1.0, 1.0, 1.0));

        let mut knots = Vec::with_capacity(2 * segments + 4);
        knots.extend([0.0; 3]);
        for k in 1..segments {
            let t = k as f64 / n;
            knots.extend([t, t]);
        }
        knots.extend([1.0; 3]);
        Self {
            origin,
            axis,
            rows,
            knots,
        }
    }

    /// The control point of `p`'s trajectory on `row`.
    fn point(&self, p: Point3, row: &Row) -> Point3 {
        let d = p - self.origin;
        let z = d.dot(self.axis);
        let radial = d - self.axis * z;
        let turned = radial * row.cos + self.axis.cross(radial) * row.sin;
        self.origin + self.axis * (z + row.rise) + turned * row.scale
    }

    fn last(&self) -> &Row {
        // `new` always pushes the final row.
        &self.rows[self.rows.len() - 1]
    }

    /// `p` moved by the full motion.
    fn end_point(&self, p: Point3) -> Point3 {
        self.point(p, self.last())
    }

    /// `v` turned by the full motion (a direction: no rise).
    fn end_vector(&self, v: Vec3) -> Vec3 {
        let row = self.last();
        let along = self.axis * v.dot(self.axis);
        let across = v - along;
        along + across * row.cos + self.axis.cross(across) * row.sin
    }

    fn weights(&self) -> Vec<f64> {
        self.rows.iter().map(|row| row.weight).collect()
    }

    /// The rail swept by one point.
    fn rail(&self, p: Point3) -> Result<NurbsCurve, OperationsError> {
        Ok(NurbsCurve::new(
            2,
            self.knots.clone(),
            self.rows.iter().map(|row| self.point(p, row)).collect(),
            self.weights(),
        )?)
    }

    /// The side surface swept by `curve`: `u` is the curve's own parameter,
    /// `v ∈ [0, 1]` runs along the motion.
    fn surface(&self, curve: &NurbsCurve) -> Result<NurbsSurface, OperationsError> {
        let row_weights = self.weights();
        let control_points: Vec<Vec<Point3>> = curve
            .control_points()
            .iter()
            .map(|&p| self.rows.iter().map(|row| self.point(p, row)).collect())
            .collect();
        let weights: Vec<Vec<f64>> = curve
            .weights()
            .iter()
            .map(|&w| row_weights.iter().map(|&rw| w * rw).collect())
            .collect();
        Ok(NurbsSurface::new(
            curve.degree(),
            2,
            curve.knots().to_vec(),
            self.knots.clone(),
            control_points,
            weights,
        )?)
    }

    /// `curve` moved rigidly by the full motion, keeping its parameterization.
    fn end_curve(&self, curve: &EdgeCurve) -> Result<EdgeCurve, OperationsError> {
        Ok(match curve {
            EdgeCurve::Line => EdgeCurve::Line,
            EdgeCurve::Circle(c) => EdgeCurve::Circle(Circle3D::with_axes(
                self.end_point(c.center()),
                self.end_vector(c.normal()),
                c.radius(),
                self.end_vector(c.u_axis()),
                self.end_vector(c.v_axis()),
            )?),
            EdgeCurve::Ellipse(e) => EdgeCurve::Ellipse(Ellipse3D::with_axes(
                self.end_point(e.center()),
                self.end_vector(e.normal()),
                e.semi_major(),
                e.semi_minor(),
                self.end_vector(e.u_axis()),
                self.end_vector(e.v_axis()),
            )?),
            EdgeCurve::Parabola(p) => EdgeCurve::Parabola(Parabola3D::with_axes(
                self.end_point(p.vertex()),
                self.end_vector(p.axis_dir()),
                self.end_vector(p.u_axis()),
                p.focal_length(),
            )?),
            EdgeCurve::Hyperbola(h) => EdgeCurve::Hyperbola(Hyperbola3D::with_axes(
                self.end_point(h.center()),
                self.end_vector(h.normal()),
                self.end_vector(h.u_axis()),
                h.semi_major(),
                h.semi_minor(),
            )?),
            EdgeCurve::NurbsCurve(n) => EdgeCurve::NurbsCurve(NurbsCurve::new(
                n.degree(),
                n.knots().to_vec(),
                n.control_points()
                    .iter()
                    .map(|&p| self.end_point(p))
                    .collect(),
                n.weights().to_vec(),
            )?),
        })
    }
}

/// One profile boundary piece: a source edge, or half of a closed one.
struct Piece {
    /// Start-cap edge geometry in the piece's natural direction.
    cap: EdgeCurve,
    trim: Option<(f64, f64)>,
    /// Exact NURBS form, natural direction, ends snapped to the vertices.
    nurbs: NurbsCurve,
    /// The loop traverses the piece in its natural direction.
    forward: bool,
}

/// One profile loop in traversal order.
struct Loop {
    pieces: Vec<Piece>,
    /// Traversal start point of each piece, and its vertex tolerance.
    points: Vec<(Point3, f64)>,
    outer: bool,
}

impl Loop {
    /// Indices of piece `i`'s natural start and end points.
    fn natural_ends(&self, i: usize) -> (usize, usize) {
        let next = (i + 1) % self.pieces.len();
        if self.pieces[i].forward {
            (i, next)
        } else {
            (next, i)
        }
    }

    /// Boundary samples in traversal order.
    #[allow(clippy::cast_precision_loss)]
    fn samples(&self) -> Vec<Point3> {
        let mut out = Vec::with_capacity(self.pieces.len() * CHECK_SAMPLES);
        for piece in &self.pieces {
            let (u0, u1) = piece.nurbs.domain();
            for j in 0..CHECK_SAMPLES {
                let f = j as f64 / CHECK_SAMPLES as f64;
                let f = if piece.forward { f } else { 1.0 - f };
                out.push(piece.nurbs.evaluate((u1 - u0).mul_add(f, u0)));
            }
        }
        out
    }
}

fn invalid(reason: impl Into<String>) -> OperationsError {
    OperationsError::InvalidInput {
        reason: reason.into(),
    }
}

/// Replace `curve`'s end control points by the vertex positions so rails and
/// surface boundaries share them exactly.
fn snap(
    curve: NurbsCurve,
    start: Point3,
    end: Point3,
    tolerance: f64,
) -> Result<NurbsCurve, OperationsError> {
    let mut points = curve.control_points().to_vec();
    let last = points.len() - 1;
    let miss = (points[0] - start)
        .length()
        .max((points[last] - end).length());
    if miss.is_nan() || miss > tolerance {
        return Err(invalid(format!(
            "helical sweep profile edge misses its vertices by {miss} (tolerance {tolerance})"
        )));
    }
    points[0] = start;
    points[last] = end;
    Ok(NurbsCurve::new(
        curve.degree(),
        curve.knots().to_vec(),
        points,
        curve.weights().to_vec(),
    )?)
}

/// The sub-span `[lo, hi]` of `curve`, exactly (knot insertion).
fn nurbs_sub_span(curve: &NurbsCurve, lo: f64, hi: f64) -> Result<NurbsCurve, OperationsError> {
    let (d0, d1) = curve.domain();
    let eps = 1e-12 * (d1 - d0).abs().max(1.0);
    let mut piece = curve.clone();
    if lo > d0 + eps {
        piece = curve_split(&piece, lo)?.1;
    }
    if hi < d1 - eps {
        piece = curve_split(&piece, hi)?.0;
    }
    Ok(piece)
}

/// Start-cap geometry, its trim, and the exact NURBS of one piece.
type ExactPiece = (EdgeCurve, Option<(f64, f64)>, NurbsCurve);

/// The exact NURBS of `curve` over `[a, b]` (possibly descending), traced
/// from `curve(a)` to `curve(b)`, plus the start-cap geometry and trim.
fn exact_piece(
    curve: &EdgeCurve,
    (a, b): (f64, f64),
    start: Point3,
    end: Point3,
) -> Result<ExactPiece, OperationsError> {
    let (lo, hi) = (a.min(b), a.max(b));
    let ascending = |n: NurbsCurve| if a <= b { n } else { n.reversed() };
    Ok(match curve {
        EdgeCurve::Line => (
            EdgeCurve::Line,
            None,
            NurbsCurve::new(
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![start, end],
                vec![1.0, 1.0],
            )?,
        ),
        EdgeCurve::Circle(c) => (
            curve.clone(),
            Some((a, b)),
            ascending(remus_geometry::convert::circle_to_nurbs(c, lo, hi)?),
        ),
        EdgeCurve::Ellipse(e) => (
            curve.clone(),
            Some((a, b)),
            ascending(remus_geometry::convert::ellipse_to_nurbs(e, lo, hi)?),
        ),
        EdgeCurve::Parabola(p) => (
            curve.clone(),
            Some((a, b)),
            ascending(remus_heal::construct::convert_curve::parabola_to_nurbs(
                p, lo, hi,
            )?),
        ),
        EdgeCurve::Hyperbola(h) => (
            curve.clone(),
            Some((a, b)),
            ascending(remus_heal::construct::convert_curve::hyperbola_to_nurbs(
                h, lo, hi,
            )?),
        ),
        EdgeCurve::NurbsCurve(n) => {
            let piece = ascending(nurbs_sub_span(n, lo, hi)?);
            let domain = piece.domain();
            (EdgeCurve::NurbsCurve(piece.clone()), Some(domain), piece)
        }
    })
}

/// A piece end: its point and vertex tolerance.
type End = (Point3, f64);

/// Read one profile wire into traversal-ordered pieces, splitting closed
/// edges into two exact halves.
fn read_loop(topo: &Topology, wire: WireId, outer: bool) -> Result<Loop, OperationsError> {
    let linear = Tolerance::new().linear;
    let mut pieces = Vec::new();
    let mut points = Vec::new();
    for oe in topo.wire(wire)?.edges() {
        let edge = topo.edge(oe.edge())?;
        let start_vertex = topo.vertex(edge.start())?;
        let end_vertex = topo.vertex(edge.end())?;
        let (start, end) = (start_vertex.point(), end_vertex.point());
        let (start_tol, end_tol) = (
            start_vertex.tolerance().max(linear),
            end_vertex.tolerance().max(linear),
        );
        let domain = match edge.strict_domain() {
            Ok(domain) => domain,
            Err(_) => crate::reconstruct_legacy_edge_domain(
                topo,
                oe.edge(),
                "helical sweep profile edge",
            )?,
        };
        let snap_tol = 10.0 * start_tol.max(end_tol);
        // Natural-direction pieces: range, then each end with its vertex
        // tolerance (a split point is a new vertex at the linear tolerance).
        let natural: Vec<((f64, f64), End, End)> = if edge.start() == edge.end() {
            let middle = f64::midpoint(domain.0, domain.1);
            let mid = (
                edge.curve().evaluate_with_endpoints(middle, start, end),
                linear,
            );
            vec![
                ((domain.0, middle), (start, start_tol), mid),
                ((middle, domain.1), mid, (end, end_tol)),
            ]
        } else {
            vec![(domain, (start, start_tol), (end, end_tol))]
        };
        let forward = oe.is_forward();
        let mut built = Vec::with_capacity(natural.len());
        for (range, a, b) in natural {
            let (cap, trim, nurbs) = exact_piece(edge.curve(), range, a.0, b.0)?;
            let nurbs = snap(nurbs, a.0, b.0, snap_tol)?;
            built.push((
                Piece {
                    cap,
                    trim,
                    nurbs,
                    forward,
                },
                a,
                b,
            ));
        }
        if !forward {
            built.reverse();
        }
        for (piece, a, b) in built {
            points.push(if forward { a } else { b });
            pieces.push(piece);
        }
    }
    if pieces.is_empty() {
        return Err(invalid("helical sweep profile wire has no edges"));
    }
    Ok(Loop {
        pieces,
        points,
        outer,
    })
}

/// Whether `samples` wind counter-clockwise about `normal`.
fn is_ccw(samples: &[Point3], normal: Vec3) -> bool {
    crate::winding::newell_normal(samples).dot(normal) >= 0.0
}

/// Refuse a profile that touches, straddles, encircles or wraps around the
/// axis, that the motion does not cross transversally, or whose coils would
/// overlap.
///
/// - **Touching**: any boundary sample within the margin of the axis.
/// - **Transversality**: the motion's velocity `V = a·pitch/2π + a × r`
///   must cross the profile plane with one sign everywhere. `V · n` is affine
///   on the plane, so its sign over the region is decided on the boundary.
///   For a profile in a plane containing the axis this is exactly the test
///   that the profile stays on one side of the axis.
/// - **Encircling / wrapping**: no loop may wind around the axis, and the
///   profile's angular extent about it must stay below a full turn.
/// - **Coil overlap** ([`check_coil_clearance`]).
fn check_profile(
    loops: &[Loop],
    origin: Point3,
    axis: Vec3,
    normal: Vec3,
    pitch: f64,
    turns: f64,
) -> Result<(), OperationsError> {
    let samples: Vec<Vec<Point3>> = loops.iter().map(Loop::samples).collect();
    let mut max_radius: f64 = 0.0;
    let mut min_radius = f64::INFINITY;
    for &p in samples.iter().flatten() {
        let d = p - origin;
        let r = (d - axis * d.dot(axis)).length();
        max_radius = max_radius.max(r);
        min_radius = min_radius.min(r);
    }
    let scale = max_radius.max(pitch.abs()).max(1.0);
    let margin = (CHECK_RELATIVE_MARGIN * scale).max(Tolerance::new().linear);
    if min_radius <= margin {
        return Err(invalid(format!(
            "helical sweep profile touches or crosses the axis (closest boundary point is \
             {min_radius} from it)"
        )));
    }

    let across = normal.cross(axis);
    let lift = pitch / TAU * axis.dot(normal);
    let speeds = samples
        .iter()
        .flatten()
        .map(|&p| lift + (p - origin).dot(across));
    let (low, high) = speeds.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), s| {
        (lo.min(s), hi.max(s))
    });
    if !(low > margin || high < -margin) {
        return Err(invalid(
            "helical sweep profile straddles the axis or is not crossed transversally by the \
             screw motion (the sweep would fold through itself)",
        ));
    }

    check_coil_clearance(&samples, origin, axis, pitch, turns, margin)
}

/// Refuse coils that would overlap.
///
/// The quantity `w = z − pitch · θ / 2π` (axial coordinate minus the rise
/// the motion attaches to the angle `θ` about the axis) is invariant under
/// the motion. Two profile points `P`, `Q` land on the same point of space
/// after different turn counts only when `w_P − w_Q` is a non-zero multiple
/// of `pitch` and the angle that requires is within the sweep, which needs
/// the sweep plus the profile's angular extent `Δθ` to reach a full turn. So
/// the coils are refused as overlapping when `Φ + Δθ ≥ 2π` and the profile's
/// `w` extent reaches `|pitch|`. For a profile in a plane through the axis
/// `w` is just the axial extent and `Δθ = 0`: more than one turn of a profile
/// at least `|pitch|` tall is refused. The check is conservative for profiles
/// that are not convex along the axis (two lobes that would interleave are
/// refused too).
fn check_coil_clearance(
    samples: &[Vec<Point3>],
    origin: Point3,
    axis: Vec3,
    pitch: f64,
    turns: f64,
    margin: f64,
) -> Result<(), OperationsError> {
    let seed = if axis.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let e1 = (seed - axis * seed.dot(axis)).normalize()?;
    let e2 = axis.cross(e1);
    let angle = |p: Point3| {
        let d = p - origin;
        d.dot(e2).atan2(d.dot(e1))
    };
    let wrap = |delta: f64| delta - TAU * (delta / TAU).round();

    // Unwrap every loop continuously; the outer loop fixes the branch.
    let mut all: Vec<(f64, f64)> = Vec::new(); // (z, θ)
    let mut range = (f64::INFINITY, f64::NEG_INFINITY);
    for (index, lp) in samples.iter().enumerate() {
        let Some(&first) = lp.first() else {
            continue;
        };
        let mut previous = angle(first);
        // A hole lies inside the outer loop, whose span is under a full
        // turn: its continuous branch is the one inside that span.
        let mut theta = if index == 0 {
            previous
        } else {
            range.0 + (previous - range.0).rem_euclid(TAU)
        };
        let mut total = 0.0;
        for (k, &p) in lp.iter().enumerate() {
            if k > 0 {
                let current = angle(p);
                let step = wrap(current - previous);
                theta += step;
                total += step;
                previous = current;
            }
            all.push(((p - origin).dot(axis), theta));
            if index == 0 {
                range = (range.0.min(theta), range.1.max(theta));
            }
        }
        total += wrap(angle(first) - previous);
        if total.abs() > std::f64::consts::PI {
            return Err(invalid(
                "helical sweep profile encircles the axis; sweep a profile beside the axis",
            ));
        }
    }
    let spread = range.1 - range.0;
    if spread >= TAU {
        return Err(invalid(
            "helical sweep profile wraps a full turn around the axis",
        ));
    }
    let (w_lo, w_hi) = all
        .iter()
        .map(|&(z, theta)| pitch.mul_add(-theta / TAU, z))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), w| {
            (lo.min(w), hi.max(w))
        });
    let extent = w_hi - w_lo;
    if TAU.mul_add(turns, spread) >= TAU && extent >= pitch.abs() - margin {
        return Err(invalid(format!(
            "helical sweep coils would overlap: the profile spans {extent} along the axis, at \
             least the pitch {}",
            pitch.abs()
        )));
    }
    Ok(())
}

/// Sweep `profile` by a screw motion about the axis line through
/// `axis_origin` along `axis_dir`, starting where the profile sits.
///
/// At turn angle `φ ∈ [0, 2π·turns]` (right-handed about `axis_dir`) every
/// profile point `P` moves to `O + a·(z_P + pitch·φ/2π) + Rot(φ)·r_P`, with
/// `z_P` and `r_P` its axial and radial parts about the axis. The coil radius
/// is wherever the profile sits; a positive `pitch` climbs along `axis_dir`
/// and a negative one against it.
///
/// Each profile edge sweeps into one rational NURBS face over every turn:
/// the edge's exact NURBS across, rational quadratic arc spans around. The
/// faces are exactly round about the axis; the one approximation is the
/// axial rise along each arc span, which is not linear in the angle the span
/// reaches. Over a span of angle `α` rising `h` it deviates by at most
/// `1.07·|h|·α²/(72√3)`, displacing points purely along the axis, and the
/// span count keeps that within `1e-6·|pitch|` (never tighter than the
/// kernel's linear tolerance): 70 spans per turn for `|pitch| ≥ 0.1`. This
/// is deliberately looser than the linear tolerance (5 µm at pitch 5): the
/// tighter target would take 257 spans a turn there, multiplying STEP size
/// and intersection cost for no modelling gain. The
/// start cap is the profile's own plane and edges; the end cap is the
/// profile moved rigidly by the full motion.
///
/// Coils are refused as overlapping when the sweep plus the profile's
/// angular extent about the axis reaches a full turn and the profile spans at
/// least `|pitch|` along the axis once the motion's rise is unwound (for a
/// profile in a plane through the axis: more than one turn of a profile at
/// least `|pitch|` tall).
///
/// # Errors
///
/// [`OperationsError::InvalidInput`] for a zero or non-finite axis or origin,
/// a zero or non-finite pitch, non-positive, non-finite or too many turns, a
/// non-planar profile, a profile that touches, straddles, encircles or wraps
/// around the axis or that the motion does not cross transversally, or coils
/// that would overlap. Construction errors propagate. Every error leaves the
/// topology unchanged.
pub fn helical_sweep_in_place(
    topo: &mut Topology,
    profile: FaceId,
    axis_origin: Point3,
    axis_dir: Vec3,
    pitch: f64,
    turns: f64,
) -> Result<SolidId, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        helical_sweep_in_place_impl(topo, profile, axis_origin, axis_dir, pitch, turns)
    })
}

#[allow(clippy::too_many_lines)]
fn helical_sweep_in_place_impl(
    topo: &mut Topology,
    profile: FaceId,
    axis_origin: Point3,
    axis_dir: Vec3,
    pitch: f64,
    turns: f64,
) -> Result<SolidId, OperationsError> {
    if ![axis_origin.x(), axis_origin.y(), axis_origin.z()]
        .iter()
        .all(|c| c.is_finite())
    {
        return Err(invalid("helical sweep axis origin must be finite"));
    }
    let length = axis_dir.length();
    if !length.is_finite() || length <= Tolerance::new().linear {
        return Err(invalid(format!(
            "helical sweep axis direction must be finite and non-zero, got length {length}"
        )));
    }
    let axis = axis_dir.normalize()?;
    if !pitch.is_finite() || pitch == 0.0 {
        return Err(invalid(format!(
            "helical sweep pitch must be finite and non-zero, got {pitch}"
        )));
    }
    if !turns.is_finite() || turns <= 0.0 {
        return Err(invalid(format!(
            "helical sweep turns must be finite and positive, got {turns}"
        )));
    }
    let segments = segments_for(pitch, turns, rise_tolerance(pitch))?;

    let face = topo.face(profile)?;
    let FaceSurface::Plane { normal, .. } = *face.surface() else {
        return Err(invalid(format!(
            "helical sweep in place needs a planar profile, got a {} face",
            face.surface().type_tag()
        )));
    };
    let normal = normal.normalize()?;
    let outer_wire = face.outer_wire();
    let inner_wires = face.inner_wires().to_vec();
    let mut loops = vec![read_loop(topo, outer_wire, true)?];
    for wire in inner_wires {
        loops.push(read_loop(topo, wire, false)?);
    }

    check_profile(&loops, axis_origin, axis, normal, pitch, turns)?;

    // Which way the motion leaves the profile plane: σ = sign(V · n), uniform
    // over the profile once `check_profile` passed.
    let sigma = {
        let (p, _) = loops[0].points[0];
        let speed =
            (pitch / TAU).mul_add(axis.dot(normal), (p - axis_origin).dot(normal.cross(axis)));
        if speed > 0.0 { 1.0 } else { -1.0 }
    };

    let screw = Screw::new(axis_origin, axis, pitch, turns, segments);
    let start_normal = -normal * sigma;
    let end_normal = screw.end_vector(normal) * sigma;

    let mut side_faces = Vec::new();
    let mut start_wires: Vec<WireId> = Vec::new();
    let mut end_wires: Vec<WireId> = Vec::new();
    let mut start_point = None;
    let mut end_point = None;

    for lp in &loops {
        let m = lp.pieces.len();
        let ccw = is_ccw(&lp.samples(), normal);
        let starts: Vec<VertexId> = lp
            .points
            .iter()
            .map(|&(p, tol)| topo.add_vertex(Vertex::new(p, tol)))
            .collect();
        let ends: Vec<VertexId> = lp
            .points
            .iter()
            .map(|&(p, tol)| topo.add_vertex(Vertex::new(screw.end_point(p), tol)))
            .collect();
        start_point.get_or_insert(lp.points[0].0);
        end_point.get_or_insert_with(|| screw.end_point(lp.points[0].0));

        let mut rails: Vec<EdgeId> = Vec::with_capacity(m);
        for (i, &(p, _)) in lp.points.iter().enumerate() {
            let rail = screw.rail(p)?;
            let domain = rail.domain();
            let mut edge = Edge::new(starts[i], ends[i], EdgeCurve::NurbsCurve(rail));
            edge.set_trim(Some(domain));
            rails.push(topo.add_edge(edge));
        }

        let mut start_edges = Vec::with_capacity(m);
        let mut end_edges = Vec::with_capacity(m);
        for (i, piece) in lp.pieces.iter().enumerate() {
            let (ns, ne) = lp.natural_ends(i);
            let mut start_edge = Edge::new(starts[ns], starts[ne], piece.cap.clone());
            start_edge.set_trim(piece.trim);
            let start_edge = topo.add_edge(start_edge);
            let mut end_edge = Edge::new(ends[ns], ends[ne], screw.end_curve(&piece.cap)?);
            end_edge.set_trim(piece.trim);
            let end_edge = topo.add_edge(end_edge);
            start_edges.push(start_edge);
            end_edges.push(end_edge);

            // Orient against the material: in the profile plane the material
            // lies left of the traversal about `normal` for a CCW outer loop
            // (and right of a CCW hole), so `T × n` scaled by that sign
            // points out of it.
            let surface = screw.surface(&piece.nurbs)?;
            let (u0, u1) = piece.nurbs.domain();
            let u_mid = f64::midpoint(u0, u1);
            let tangent = piece.nurbs.tangent(u_mid)?;
            let tangent = if piece.forward { tangent } else { -tangent };
            let side = if ccw == lp.outer { 1.0 } else { -1.0 };
            let outward = tangent.cross(normal) * side;
            let reversed = surface.normal(u_mid, 0.0)?.dot(outward) < 0.0;
            // The wire runs counter-clockwise about the surface's own normal
            // `∂u × ∂v` (bottom row, far rail, top row back, near rail
            // down); a reversed face flips both, so the wire is the same
            // either way.
            let edges = vec![
                OrientedEdge::new(start_edge, true),
                OrientedEdge::new(rails[ne], true),
                OrientedEdge::new(end_edge, false),
                OrientedEdge::new(rails[ns], false),
            ];
            let wire = topo.add_wire(Wire::new(edges, true)?);
            let mut face = Face::new(wire, vec![], FaceSurface::Nurbs(surface));
            if reversed {
                face.set_reversed(true);
            }
            side_faces.push(topo.add_face(face));
        }

        // Cap wires: an outer loop runs counter-clockwise about its cap's
        // outward normal, a hole clockwise.
        // `along_n`: the cap's outward normal points along `normal`.
        let cap_wire = |topo: &mut Topology,
                        edges: &[EdgeId],
                        along_n: bool|
         -> Result<WireId, OperationsError> {
            let oriented: Vec<OrientedEdge> = if (ccw == along_n) == lp.outer {
                edges
                    .iter()
                    .zip(&lp.pieces)
                    .map(|(&e, piece)| OrientedEdge::new(e, piece.forward))
                    .collect()
            } else {
                edges
                    .iter()
                    .zip(&lp.pieces)
                    .rev()
                    .map(|(&e, piece)| OrientedEdge::new(e, !piece.forward))
                    .collect()
            };
            Ok(topo.add_wire(Wire::new(oriented, true)?))
        };
        // The start cap faces −σ n; the end cap faces σ M_Φ(n), and the
        // rigid motion carries the loop's winding about n with it.
        start_wires.push(cap_wire(topo, &start_edges, sigma < 0.0)?);
        end_wires.push(cap_wire(topo, &end_edges, sigma > 0.0)?);
    }

    let (Some(start_point), Some(end_point)) = (start_point, end_point) else {
        return Err(invalid("helical sweep profile has no boundary"));
    };
    let start_cap = topo.add_face(Face::new(
        start_wires[0],
        start_wires[1..].to_vec(),
        FaceSurface::Plane {
            normal: start_normal,
            d: crate::dot_normal_point(start_normal, start_point),
        },
    ));
    let end_cap = topo.add_face(Face::new(
        end_wires[0],
        end_wires[1..].to_vec(),
        FaceSurface::Plane {
            normal: end_normal,
            d: crate::dot_normal_point(end_normal, end_point),
        },
    ));

    let mut faces = Vec::with_capacity(side_faces.len() + 2);
    faces.push(start_cap);
    faces.extend(side_faces);
    faces.push(end_cap);
    let shell = topo.add_shell(Shell::new(faces)?);
    Ok(topo.add_solid(Solid::new(shell, vec![])))
}

#[cfg(test)]
mod tests;
