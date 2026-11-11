//! Certify or refuse: the one gate a closed section of a planar face passes
//! before any span of it that ends on a hole is emitted.
//!
//! # The invariant
//!
//! The section (a circle or an ellipse) and every loop of the planar face
//! (its outline and its holes) are closed curves, so the section crosses each
//! loop an even number of times, entering and leaving alternately. A span of
//! the section between two split points is emitted only when all of this is
//! established for it:
//!
//! 1. every loop of the face the span comes within reach of has been read,
//!    exactly (lines, and circular arcs under a circular section);
//! 2. no crossing of any loop lies inside the span, and none of its ends is
//!    a crossing this module could not resolve;
//! 3. the span has a point farther than the tolerance band from every loop,
//!    where a second, independent evaluation of the side of each loop
//!    agrees with the side the span was followed on: inside the outline and
//!    outside every hole;
//! 4. its ends on a hole are farther than [`SectionTolerance::separate`]
//!    from every other vertex the face carries there, and the face
//!    splitter's own rule, run over the ends of the spans kept, puts a
//!    vertex on each hole edge at exactly those ends
//!    ([`SpanPlan::certify_hole_splits`]): the face splitter and shell
//!    assembly keep the vertex set phase FF decided on.
//!
//! A span that fails any of these while it would otherwise be kept is a
//! refusal by name ([`AlgoError::IntersectionFailed`]), never a default.
//! "Could not decide" is not "touch" and not "no crossing". A span that
//! ends on a hole and is certified off the face is dropped.
//!
//! # How a loop is read
//!
//! A *zone* belongs to one loop EDGE over its own extent, never to the
//! line or circle that carries it. It is an interval of the section's
//! parameter, found in closed form (the distance from a line, the position
//! along it, the squared distance from a circle's centre and the side of a
//! ray from that centre are all sinusoids of the parameter), and it is
//! bracketed both ways:
//!
//! - it holds every point of the section within the band of the edge: a
//!   straight edge's is the rectangle one band either side of the segment
//!   and one band past each end; an arc's is the annulus one band either
//!   side of its circle, between the rays through its ends turned outward
//!   by the angle a band subtends there;
//! - every point of it lies within [`ZONE_REACH`] bands of the edge (the
//!   corners of that rectangle, and of the annulus sector, are the
//!   farthest).
//!
//! Where the section runs within the band of an edge's carrier but past
//! the edge's ends, that stretch is in no zone of the edge: the next edge
//! of the loop takes over at the shared vertex, and where the loop turns
//! away the section is in a gap.
//!
//! The zones of one loop, merged, leave *gaps* between them. Along a gap
//! the section stays farther than the band from every edge of the loop, so
//! it does not meet the loop and lies on one side of it throughout: one
//! probe decides the gap.
//!
//! A zone whose two neighbouring gaps lie on different sides is a crossing.
//! Its vertex is the one exact root of the section with a loop edge found
//! in the zone (a root of the FINITE edge: on the segment, or within the
//! arc's own angle), or failing that the one loop vertex the section passes
//! within the band of there (a crossing through a corner). A zone with
//! neither is a crossing that cannot be placed: it is kept as a split point
//! and marked, and refuses any span kept beside it. That is what becomes of
//! a section that runs along a hole edge on, or nearly on, its own circle
//! and leaves it to different sides at the two ends of the overlap: the
//! change of side is certain (it is read off the two gaps, at the ends of
//! the overlap), where along the overlap to put the vertex is not.
//!
//! A zone whose neighbouring gaps lie on the same side is a touch. The
//! section stays within [`ZONE_REACH`] bands of the loop throughout it, so
//! it does not pass the loop by more than that there.
//!
//! The number of crossings of a loop is therefore even whenever every gap
//! could be decided, and consecutive crossings alternate; both are checked
//! again on the result ([`LoopCertificate::alternates`]).
//!
//! # A span that ends on no hole
//!
//! Such a span is decided by the collector as it was before the holes were
//! read (the planar face's outline and the partner face's boundary split
//! the section the way they always did), so a section whose hole crossings
//! all lie on spans the partner face does not reach is not refused for
//! them. But it is emitted only when it is also certified not to pass
//! through a hole ([`PlaneSectionCertificate::clear_of_holes`]). The
//! argument, hole by hole, with each premise enforced:
//!
//! 1. *Out of reach.* The hole's box (its vertices, widened by each curved
//!    edge's own reach) does not meet the section's exact box widened by
//!    the band. Then no point of the section lies in the hole or within the
//!    band of it. Such a hole is never read: a face with no hole, or with
//!    holes out of reach only, costs nothing and refuses nothing.
//! 2. *Read.* Every point of the span lies in a zone of the hole or in a
//!    gap. The span holds no crossing of the hole (every crossing is a
//!    split point, and a span ending on one is not this case; one that is
//!    not a split point refuses the span). Every gap the span meets must
//!    lie outside the hole, by the gap's own probe AND by a second
//!    evaluation at a point of the span itself with a different method
//!    (an even–odd ray count, [`ray_side`], against the winding number the
//!    gaps were decided with). Then each point of the span is outside the
//!    hole, or in a zone and so within [`ZONE_REACH`] bands of the hole's
//!    boundary: none lies deeper inside the hole than that. A gap inside
//!    the hole drops the span; gaps on both sides, an undecided gap, or
//!    a disagreeing second evaluation refuse it.
//! 3. *Not read* (an edge with no exact contact with the section). Every
//!    piece of every edge of the hole is shown farther than the band from
//!    the span — by its box against the span's exact box, or by a lower
//!    bound on its distance from the section — so the span does not meet
//!    the hole's boundary and lies wholly inside the hole or wholly outside
//!    it; the winding number of the boundary about the span's midpoint,
//!    summed over the same pieces (each subtends less than a half turn
//!    there), says which. A line or a conic edge is halved until its
//!    pieces are clear, a spline is cut at its knots' midpoints until the
//!    hulls of its control points are; a piece that cannot be shown clear
//!    (the hole comes within the band of the span, or runs along it for
//!    too long to follow) refuses the span.
//!
//! So the only way a span reaches the caller across a hole is a refusal.
//!
//! Two more premises are the collector's, and are held to here:
//!
//! - *Every crossing is a split point.* A span over a crossing that is not
//!   one lies partly on the face and partly off it, and no point of it
//!   speaks for the whole. A hole's crossings are handed to the collector
//!   by this module; the outline's are found by the collector its own way,
//!   and where the outline was read (a section that crosses a hole) each
//!   must have a split point within the separation distance. A span over
//!   one that has none is refused, kept or dropped.
//! - *A dropped span holds no material.* A span the collector drops
//!   because a point of it is off the planar face is dropped whole. That
//!   is sound between consecutive crossings; the crossings of a hole that
//!   was not read are not split points, so such a span must be shown clear
//!   of that hole or inside it, as in 3, or it is refused: dropped for a
//!   midpoint inside the hole, it would take the material either side of
//!   the hole with it.
//!
//! # One parameter
//!
//! A crossing is decided at a parameter `t` of the section, and that `t`
//! is what the collectors emit: the span ends there, and the edge made of
//! it is trimmed there. No stage re-derives it from the crossing's point.
//! The point may be a loop vertex within the band of the section (a
//! crossing through a corner), and the chart angle of a point off an
//! eccentric section is not the parameter of its foot: at ratio 1000 a
//! vertex 4e-8 off the curve re-projects 2e-5 along it, straight through
//! the hole. Three checks hold the stages to this
//! ([`PlaneSectionCertificate::plan`]):
//!
//! - a split point that stands for a crossing must carry the crossing's
//!   parameter: between the two parameters the section must stay within
//!   the band of the crossing's point, which is a point of the hole's
//!   boundary (so the stretch between them cannot run into the hole);
//! - the crossing's own point must re-project that close too (its chart
//!   angle, which is what a later stage reads off a vertex: the face
//!   splitter still takes a section end's parameter from its vertex when
//!   it splits one section at another's end). A corner too far off an
//!   eccentric section for that is not a vertex the section can end on,
//!   and is refused;
//! - both ends of a span that ends on a hole must lie within the band of
//!   the section's own point at their parameter.
//!
//! The band here is a LENGTH, at any eccentricity: a section ten thousand
//! times longer than wide is held to the same 1e-7 along itself, and what
//! it cannot meet (a corner 1e-9 off it re-projects 1e-5 along) it
//! refuses.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use remus_math::aabb::Aabb3;
use remus_math::compact_sort::CompactSort;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Point3, Vec2, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::wire::WireId;

use super::{
    Chord, LineContact, SectionConic, circle_arc_crossings, close_crossings_message,
    planar_loop_side, unresolved_chord_message,
};
use crate::builder::classify_2d::PlanarLoopEdge;
use crate::builder::plane_frame::PlaneFrame;
use crate::error::AlgoError;
use crate::section_tolerance::SectionTolerance;

/// How far from a loop edge a point of one of its zones can lie, in bands:
/// the corner of the rectangle a straight edge's zone is cut from (`√2`),
/// and of the annulus sector an arc's is (a little more, for the sector's
/// outer corner; an arc so small that its corner would lie farther than
/// this is not read, see [`arc_stretch`]).
pub(super) const ZONE_REACH: f64 = 1.5;

// ── Intervals of the section's parameter ────────────────────────────────

/// A closed interval of the section's parameter: `lo` in `[0, τ)`, `hi` in
/// `[lo, lo + τ]`.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Span {
    lo: f64,
    hi: f64,
}

impl Span {
    fn new(lo: f64, hi: f64) -> Self {
        let length = (hi - lo).clamp(0.0, TAU);
        let lo = lo.rem_euclid(TAU);
        Self {
            lo,
            hi: lo + length,
        }
    }

    fn length(self) -> f64 {
        self.hi - self.lo
    }

    /// The angle by which `t` lies outside the span; zero inside it.
    fn clearance(self, t: f64) -> f64 {
        let offset = (t - self.lo).rem_euclid(TAU);
        if offset <= self.length() {
            0.0
        } else {
            (offset - self.length()).min(TAU - offset)
        }
    }

    /// The parts of the open interval `(t0, t1)` (any real numbers, at most
    /// one turn apart) this span covers.
    fn overlaps(self, t0: f64, t1: f64) -> Vec<(f64, f64)> {
        let mut found = Vec::new();
        for turn in [-2.0, -1.0, 0.0, 1.0, 2.0] {
            let (lo, hi) = (
                t0.max(TAU.mul_add(turn, self.lo)),
                t1.min(TAU.mul_add(turn, self.hi)),
            );
            if hi > lo {
                found.push((lo, hi));
            }
        }
        found
    }
}

/// The parameters `t` with `lo ≤ amp · cos(t − phase) + offset ≤ hi`.
///
/// The end angles come from `amp ∓ value`, whichever is the smaller, so a
/// band at a tangency (`value ≈ ±amp`) keeps the accuracy of that
/// difference instead of losing half the digits to `acos` near ±1.
fn sinusoid_band(amp: f64, phase: f64, offset: f64, lo: f64, hi: f64) -> Vec<Span> {
    let whole = || vec![Span { lo: 0.0, hi: TAU }];
    if amp.is_nan() || amp <= 0.0 || !amp.is_finite() {
        return if offset >= lo && offset <= hi {
            whole()
        } else {
            Vec::new()
        };
    }
    let (below, above) = (lo - offset, hi - offset);
    if above < -amp || below > amp || below > above {
        return Vec::new();
    }
    let angle = |value: f64| {
        let (minus, plus) = (amp - value, amp + value);
        if minus <= plus {
            2.0 * (minus / (2.0 * amp)).clamp(0.0, 1.0).sqrt().asin()
        } else {
            2.0f64.mul_add(-(plus / (2.0 * amp)).clamp(0.0, 1.0).sqrt().asin(), PI)
        }
    };
    let inner = if above >= amp { 0.0 } else { angle(above) };
    let outer = if below <= -amp { PI } else { angle(below) };
    match (inner <= 0.0, outer >= PI) {
        (true, true) => whole(),
        (true, false) => vec![Span::new(phase - outer, phase + outer)],
        (false, true) => vec![Span::new(phase + inner, phase + TAU - inner)],
        (false, false) => vec![
            Span::new(phase + inner, phase + outer),
            Span::new(phase - outer, phase - inner),
        ],
    }
}

fn intersect(a: &[Span], b: &[Span]) -> Vec<Span> {
    let mut found = Vec::new();
    for x in a {
        for y in b {
            for turn in [-TAU, 0.0, TAU] {
                let (lo, hi) = (x.lo.max(y.lo + turn), x.hi.min(y.hi + turn));
                if hi > lo {
                    found.push(Span::new(lo, hi));
                }
            }
        }
    }
    found
}

/// A stretch of the section within the band of loop edge `edge`.
struct Stretch {
    span: Span,
    edge: usize,
}

/// The stretches merged into zones, in parameter order, each with the edges
/// it comes from; `None` when they cover the whole turn.
fn merge_stretches(mut stretches: Vec<Stretch>) -> Option<Vec<(Span, Vec<usize>)>> {
    if stretches.iter().any(|s| s.span.length() >= TAU) {
        return None;
    }
    stretches.compact_sort_by(|a, b| a.span.lo.total_cmp(&b.span.lo));
    let mut zones: Vec<(f64, f64, Vec<usize>)> = Vec::new();
    for stretch in stretches {
        match zones.last_mut() {
            Some((_, hi, edges)) if stretch.span.lo <= *hi => {
                *hi = hi.max(stretch.span.hi);
                if !edges.contains(&stretch.edge) {
                    edges.push(stretch.edge);
                }
            }
            _ => zones.push((stretch.span.lo, stretch.span.hi, vec![stretch.edge])),
        }
    }
    // The last zone may run through the turn into the first ones.
    while zones.len() >= 2 {
        let reach = zones[zones.len() - 1].1 - TAU;
        if reach < zones[0].0 {
            break;
        }
        let (_, hi, edges) = zones.remove(0);
        let last = zones.last_mut()?;
        last.1 = last.1.max(hi + TAU);
        for edge in edges {
            if !last.2.contains(&edge) {
                last.2.push(edge);
            }
        }
    }
    if zones.iter().any(|(lo, hi, _)| hi - lo >= TAU) {
        return None;
    }
    Some(
        zones
            .into_iter()
            .map(|(lo, hi, edges)| (Span { lo, hi }, edges))
            .collect(),
    )
}

// ── Which side of a loop a point lies on ────────────────────────────────

fn cross_2d(a: Vec2, b: Vec2) -> f64 {
    a.x().mul_add(b.y(), -(a.y() * b.x()))
}

fn dot_2d(a: Vec2, b: Vec2) -> f64 {
    a.x().mul_add(b.x(), a.y() * b.y())
}

/// Whether `p` lies inside the closed loop `outline`, or `None` when it lies
/// within `eps` of the loop itself.
///
/// The winding number, summed per edge: a segment adds the angle it
/// subtends, an arc its chord's angle plus a full turn when `p` lies between
/// the arc and its chord. One cross product decides both the chord angle's
/// branch and the side of the chord `p` is on, so the two cannot disagree
/// by rounding, and the test abstains only near the loop: unlike
/// [`crate::builder::classify_2d::point_in_planar_loop`], a point on or
/// beside an arc's circle away from the arc is decided.
fn loop_side(p: Point2, outline: &[PlanarLoopEdge], eps: f64) -> Option<bool> {
    if outline.is_empty() {
        return None;
    }
    let mut winding = 0.0;
    for &edge in outline {
        match edge {
            PlanarLoopEdge::Segment(a, b) => {
                let (da, db) = (a - p, b - p);
                let ab = b - a;
                let length_squared = dot_2d(ab, ab);
                let along = if length_squared > 0.0 {
                    (dot_2d(p - a, ab) / length_squared).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                if ((p - a) - ab * along).length() <= eps {
                    return None;
                }
                winding += cross_2d(da, db).atan2(dot_2d(da, db));
            }
            PlanarLoopEdge::Circle {
                center,
                radius,
                sense,
            } => {
                let offset = (p - center).length();
                if (offset - radius).abs() <= eps {
                    return None;
                }
                if offset < radius {
                    winding += TAU * sense;
                }
            }
            PlanarLoopEdge::Arc {
                center,
                radius,
                start,
                mid,
                end,
            } => {
                let (da, db) = (start - p, end - p);
                if da.length() <= eps || db.length() <= eps {
                    return None;
                }
                // `cross` is linear in `p` and vanishes on the chord's line:
                // its sign is the side of the chord `p` is on, and its value
                // at the arc's midpoint is the arc's side.
                let cross = cross_2d(da, db);
                let arc_side = cross_2d(start - mid, end - mid);
                let dot = dot_2d(da, db);
                if cross == 0.0 && dot < 0.0 {
                    // On the chord itself: the limit from the side away
                    // from the arc.
                    winding -= PI * arc_side.signum();
                } else {
                    winding += cross.atan2(dot);
                }
                if cross * arc_side > 0.0 {
                    // On the arc's side of its chord. Within `eps` of the
                    // circle there, `p` is on the ARC only where its
                    // direction from the centre lies within the arc's own
                    // turn. Past the arc's ends the circle bounds nothing,
                    // and a point just off it is decided like any other: a
                    // short arc's chord is all but tangent to its circle,
                    // so the circle outside the arc runs within a hair of
                    // the chord's line for a long way either side.
                    let offset = (p - center).length();
                    if (offset - radius).abs() <= eps
                        && arc_holds(center, start, arc_turn(center, start, mid, end), p)
                    {
                        return None;
                    }
                    if offset < radius {
                        winding -= TAU * arc_side.signum();
                    }
                }
            }
        }
    }
    Some((winding / TAU).round() != 0.0)
}

/// The turn of the arc `start → mid → end` about `center`: whether it runs
/// counter-clockwise, and the angle it sweeps.
fn arc_turn(center: Point2, start: Point2, mid: Point2, end: Point2) -> (bool, f64) {
    let from_start = |q: Point2| {
        cross_2d(start - center, q - center)
            .atan2(dot_2d(start - center, q - center))
            .rem_euclid(TAU)
    };
    let (to_mid, to_end) = (from_start(mid), from_start(end));
    if to_mid < to_end {
        (true, to_end)
    } else {
        (false, TAU - to_end)
    }
}

/// Whether the direction of `q` from `center` lies within the turn of the
/// arc that starts at `start` (see [`arc_turn`]).
fn arc_holds(center: Point2, start: Point2, (forward, sweep): (bool, f64), q: Point2) -> bool {
    let turned = cross_2d(start - center, q - center)
        .atan2(dot_2d(start - center, q - center))
        .rem_euclid(TAU);
    let turned = if forward {
        turned
    } else {
        (TAU - turned).rem_euclid(TAU)
    };
    turned <= sweep
}

/// The distance from `p` to one edge of a loop, over the edge's own extent.
fn loop_edge_distance(p: Point2, edge: PlanarLoopEdge) -> f64 {
    match edge {
        PlanarLoopEdge::Segment(a, b) => {
            let ab = b - a;
            let length_squared = dot_2d(ab, ab);
            let along = if length_squared > 0.0 {
                (dot_2d(p - a, ab) / length_squared).clamp(0.0, 1.0)
            } else {
                0.0
            };
            ((p - a) - ab * along).length()
        }
        PlanarLoopEdge::Circle { center, radius, .. } => ((p - center).length() - radius).abs(),
        PlanarLoopEdge::Arc {
            center,
            radius,
            start,
            mid,
            end,
        } => {
            if arc_holds(center, start, arc_turn(center, start, mid, end), p) {
                ((p - center).length() - radius).abs()
            } else {
                (p - start).length().min((p - end).length())
            }
        }
    }
}

/// Whether `p` lies inside the closed loop `outline`, by the even–odd rule:
/// the number of times a ray from `p` meets the loop. `None` when `p` lies
/// within `eps` of the loop, or when no ray tried passes clear of every
/// vertex and tangency.
///
/// The second evaluation of a span's side ([`loop_side`] is the first). It
/// shares the loop's edges with it and nothing else: no angle is summed, an
/// arc is met by solving its circle with the ray and keeping the roots
/// inside the arc's own turn, and its chord plays no part.
fn ray_side(p: Point2, outline: &[PlanarLoopEdge], eps: f64) -> Option<bool> {
    // Directions with no rational relation to an axis-aligned or diagonal
    // loop: one of them clears every vertex unless `p` is on the loop.
    const DIRECTIONS: [f64; 5] = [0.377, 1.291, 2.633, 3.907, 5.419];
    if outline.is_empty()
        || outline
            .iter()
            .any(|&edge| loop_edge_distance(p, edge) <= eps)
    {
        return None;
    }
    'direction: for angle in DIRECTIONS {
        let (sin, cos) = angle.sin_cos();
        let (along, across) = (Vec2::new(cos, sin), Vec2::new(-sin, cos));
        let mut hits = 0_usize;
        for &edge in outline {
            // Where the ray meets the edge's carrier, as distances along it.
            let (reaches, ends): ([Option<f64>; 2], [Point2; 2]) = match edge {
                PlanarLoopEdge::Segment(a, b) => {
                    let (ya, yb) = (dot_2d(a - p, across), dot_2d(b - p, across));
                    let (xa, xb) = (dot_2d(a - p, along), dot_2d(b - p, along));
                    let reach =
                        ((ya > 0.0) != (yb > 0.0)).then(|| (xb - xa).mul_add(ya / (ya - yb), xa));
                    ([reach, None], [a, b])
                }
                PlanarLoopEdge::Circle { center, radius, .. }
                | PlanarLoopEdge::Arc { center, radius, .. } => {
                    let (xc, yc) = (dot_2d(center - p, along), dot_2d(center - p, across));
                    if (yc.abs() - radius).abs() <= eps {
                        // The ray grazes the circle.
                        continue 'direction;
                    }
                    let ends = match edge {
                        PlanarLoopEdge::Arc { start, end, .. } => [start, end],
                        PlanarLoopEdge::Segment(..) | PlanarLoopEdge::Circle { .. } => {
                            [center, center]
                        }
                    };
                    if yc.abs() > radius {
                        ([None, None], ends)
                    } else {
                        let half = radius.mul_add(radius, -(yc * yc)).max(0.0).sqrt();
                        ([Some(xc - half), Some(xc + half)], ends)
                    }
                }
            };
            // A ray through an end of an edge would be counted by the
            // edge's neighbour too: take another.
            if !matches!(edge, PlanarLoopEdge::Circle { .. })
                && ends.iter().any(|&end| {
                    dot_2d(end - p, across).abs() <= eps && dot_2d(end - p, along) > 0.0
                })
            {
                continue 'direction;
            }
            for reach in reaches.into_iter().flatten() {
                if reach <= 0.0 {
                    continue;
                }
                if let PlanarLoopEdge::Arc {
                    center,
                    start,
                    mid,
                    end,
                    ..
                } = edge
                    && !arc_holds(
                        center,
                        start,
                        arc_turn(center, start, mid, end),
                        p + along * reach,
                    )
                {
                    continue;
                }
                hits += 1;
            }
        }
        return Some(!hits.is_multiple_of(2));
    }
    None
}

// ── One loop ────────────────────────────────────────────────────────────

/// A root of the section with a loop edge, or a loop vertex the section
/// passes within the band of.
#[derive(Clone, Copy)]
struct Root {
    t: f64,
    at: Point3,
    /// The circular edge the root lies on.
    arc: Option<EdgeId>,
    /// The loop edge the root lies on.
    edge: usize,
    /// The loop vertex the root sits at (within the band), if any.
    vertex: Option<usize>,
}

/// What a loop's edges give before its zones are decided.
struct LoopRead {
    outline: Vec<PlanarLoopEdge>,
    /// The traversal start of each edge.
    vertices: Vec<Point3>,
    stretches: Vec<Stretch>,
    roots: Vec<Root>,
    /// Per edge, the parameters at which the section comes nearest the
    /// edge's carrier (and farthest from it).
    tangencies: Vec<[f64; 2]>,
    /// The straight edges the section passes by more than the band.
    chords: Vec<(usize, EdgeId, Chord)>,
    /// Per edge, its ends in traversal order when it is straight.
    segments: Vec<Option<(Point3, Point3)>>,
}

/// A point where the section crosses a loop of the planar face.
#[derive(Clone, Debug)]
pub(super) struct Crossing {
    /// The section's parameter there, in `[0, τ)`.
    pub(super) t: f64,
    pub(super) at: Point3,
    /// The circular loop edge it lies on, `None` for a straight one or a
    /// vertex.
    pub(super) arc: Option<EdgeId>,
    /// Why a span kept beside this crossing must be refused, if it must.
    pub(super) taint: Option<String>,
    /// The loop edge it lies strictly inside, `None` at a vertex.
    edge: Option<usize>,
}

enum Contact {
    /// The section stays on one side of the loop, within the band of it.
    Touch(Point3),
    Crossing(Crossing),
    /// A neighbouring gap could not be decided.
    Undecided,
}

struct Zone {
    span: Span,
    contact: Contact,
}

/// A stretch of the section farther than the band from the loop, and the
/// side of the loop it lies on (`Some(true)` inside).
struct Gap {
    span: Span,
    side: Option<bool>,
}

/// One loop of the planar face against the section.
pub(super) struct LoopCertificate {
    wire: WireId,
    pub(super) outline: Vec<PlanarLoopEdge>,
    vertices: Vec<Point3>,
    /// Per edge, its ends in traversal order when it is straight.
    segments: Vec<Option<(Point3, Point3)>>,
    /// In parameter order.
    zones: Vec<Zone>,
    /// `gaps[i]` follows `zones[i]`; one whole-turn gap when there is no
    /// zone.
    gaps: Vec<Gap>,
    /// The section stays within the band of the loop all the way round.
    coincident: bool,
}

impl LoopCertificate {
    pub(super) fn crossings(&self) -> impl Iterator<Item = &Crossing> {
        self.zones.iter().filter_map(|zone| match &zone.contact {
            Contact::Crossing(crossing) => Some(crossing),
            Contact::Touch(_) | Contact::Undecided => None,
        })
    }

    fn crossings_mut(&mut self) -> impl Iterator<Item = &mut Crossing> {
        self.zones
            .iter_mut()
            .filter_map(|zone| match &mut zone.contact {
                Contact::Crossing(crossing) => Some(crossing),
                Contact::Touch(_) | Contact::Undecided => None,
            })
    }

    pub(super) fn touches(&self) -> impl Iterator<Item = Point3> + '_ {
        self.zones.iter().filter_map(|zone| match zone.contact {
            Contact::Touch(at) => Some(at),
            Contact::Crossing(_) | Contact::Undecided => None,
        })
    }

    /// Why the loop's contacts with the section could not all be decided,
    /// when a stretch of the section near it lies on a side that could not
    /// be read: such a loop has no crossing to show for it, and "could not
    /// decide" is not "no crossing".
    #[cfg(test)]
    pub(super) fn undecided(&self) -> Option<String> {
        self.zones
            .iter()
            .any(|zone| matches!(zone.contact, Contact::Undecided))
            .then(|| self.alternates().err())
            .flatten()
    }

    /// Whether the loop's crossings are even in number and alternate: going
    /// round the section, the side of the loop changes at every crossing
    /// and nowhere else. `Err` names the first place it does not.
    fn alternates(&self) -> Result<(), String> {
        let Some(last) = self.gaps.last() else {
            return Ok(());
        };
        let Some(mut side) = last.side else {
            return Err(format!(
                "the section's side of loop {:?} could not be decided all the way round",
                self.wire
            ));
        };
        let start = side;
        let mut crossings = 0_usize;
        for (zone, gap) in self.zones.iter().zip(&self.gaps) {
            if matches!(zone.contact, Contact::Crossing(_)) {
                side = !side;
                crossings += 1;
            }
            match gap.side {
                Some(after) if after == side => {}
                Some(_) | None => {
                    return Err(format!(
                        "the section's side of loop {:?} does not alternate at its crossings",
                        self.wire
                    ));
                }
            }
        }
        if side != start || !crossings.is_multiple_of(2) {
            return Err(format!(
                "the section crosses closed loop {:?} {crossings} time(s), an odd number",
                self.wire
            ));
        }
        Ok(())
    }
}

impl LoopCertificate {
    /// The zone or gap the parameter `t` falls in: `(index, true)` for
    /// zone `index`, `(index, false)` for the gap that follows it.
    fn place(&self, t: f64) -> (usize, bool) {
        if let Some(zone) = self
            .zones
            .iter()
            .position(|zone| zone.span.clearance(t) <= 0.0)
        {
            return (zone, true);
        }
        let gap = self
            .gaps
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.span.clearance(t).total_cmp(&b.1.span.clearance(t)))
            .map_or(0, |(index, _)| index);
        (gap, false)
    }

    /// The side of this loop the span `(t0, t1)` of the section lies on,
    /// from the split point `start` to the split point `end`, and the
    /// parts of the span farther than the band from the loop.
    ///
    /// The span is followed through the loop's zones and gaps by INDEX. An
    /// end that is one of the loop's crossings is found by its point, and
    /// the span starts in the gap after that crossing's zone (or ends in
    /// the gap before it), whatever the rounding of the zone's ends says
    /// about its parameter. Every gap it runs through must lie on one side,
    /// and no crossing may lie between them.
    fn side_along(
        &self,
        (t0, t1): (f64, f64),
        (start, end): (Point3, Point3),
        band: f64,
    ) -> Result<(bool, Vec<(f64, f64)>), String> {
        let wire = self.wire;
        if self.coincident {
            return Err(format!(
                "the section runs along loop {wire:?} all the way round"
            ));
        }
        let undecided = || {
            format!(
                "the side of loop {wire:?} the section lies on could not be decided along a \
                 span it would keep"
            )
        };
        let count = self.zones.len();
        if count == 0 {
            let side = self
                .gaps
                .first()
                .and_then(|gap| gap.side)
                .ok_or_else(undecided)?;
            return Ok((side, vec![(t0, t1)]));
        }
        let crossing_zone = |p: Point3| {
            self.zones.iter().position(
                |zone| matches!(&zone.contact, Contact::Crossing(c) if (c.at - p).length() <= band),
            )
        };
        // The zone each end lies in (if it lies in one), and the first and
        // last gaps the span runs through.
        let (start_zone, first) = crossing_zone(start).map_or_else(
            || {
                let (index, in_zone) = self.place(t0);
                (in_zone.then_some(index), index)
            },
            |zone| (Some(zone), zone),
        );
        let (end_zone, last) = crossing_zone(end).map_or_else(
            || {
                let (index, in_zone) = self.place(t1);
                if in_zone {
                    (Some(index), (index + count - 1) % count)
                } else {
                    (None, index)
                }
            },
            |zone| (Some(zone), (zone + count - 1) % count),
        );
        let mut gaps = (last + count - first) % count + 1;
        if gaps == count
            && let (Some(a), Some(b)) = (start_zone, end_zone)
            && a == b
        {
            // Both ends in one zone: the span runs through every gap, or
            // through none when it ends before that zone does.
            let span = self.zones[a].span;
            let offset = (t0 - span.lo).rem_euclid(TAU);
            let offset = if offset <= span.length() {
                offset
            } else if offset - span.length() < TAU - offset {
                span.length()
            } else {
                0.0
            };
            if t1 - t0 <= span.length() - offset {
                gaps = 0;
            }
        }
        if gaps == 0 {
            return Err(format!(
                "a span of the section stays within the tolerance of loop {wire:?} from end to \
                 end"
            ));
        }
        let mut side = None;
        let mut pieces = Vec::new();
        for step in 0..gaps {
            let index = (first + step) % count;
            if step > 0 {
                // The zone between the previous gap and this one.
                match &self.zones[index].contact {
                    Contact::Crossing(crossing) => {
                        return Err(format!(
                            "the section crosses loop {wire:?} at {:?}, inside a span it would \
                             keep whole",
                            crossing.at
                        ));
                    }
                    Contact::Undecided => return Err(undecided()),
                    Contact::Touch(_) => {}
                }
            }
            let gap = &self.gaps[index];
            match (gap.side, side) {
                (Some(here), Some(known)) if here != known => {
                    return Err(format!(
                        "the section changes sides of loop {wire:?} along a span with no \
                         crossing of it"
                    ));
                }
                (Some(here), _) => side = Some(here),
                (None, _) => return Err(undecided()),
            }
            pieces.extend(gap.span.overlaps(t0, t1));
        }
        side.map(|side| (side, pieces)).ok_or_else(undecided)
    }

    /// The side of this loop the span `(t0, t1)` of the section lies on,
    /// for a span that ends on none of the loop's crossings: `Some(true)`
    /// inside, `Some(false)` outside, `None` when the span stays within
    /// [`ZONE_REACH`] bands of the loop from end to end.
    ///
    /// Every point of the span lies in a zone of the loop (within that
    /// reach of it) or in a gap (on one side of it). The span may hold no
    /// crossing, every gap it meets must be decided and on one side, and
    /// that side must be read again off a point of the span itself, by
    /// [`ray_side`]: a different method from the one the gaps were decided
    /// with.
    fn uncrossed_side(
        &self,
        (t0, t1): (f64, f64),
        conic: &SectionConic,
        frame: &PlaneFrame,
        band: f64,
    ) -> Result<Option<bool>, String> {
        let wire = self.wire;
        if self.coincident {
            return Ok(None);
        }
        if let Some(crossing) = self
            .crossings()
            .find(|crossing| (crossing.t - t0).rem_euclid(TAU) <= t1 - t0)
        {
            return Err(format!(
                "the section crosses loop {wire:?} at {:?}, inside a span that does not end on \
                 it",
                crossing.at
            ));
        }
        let undecided = || {
            format!(
                "the side of loop {wire:?} the section lies on could not be decided along a \
                 span that does not end on it"
            )
        };
        let mut side = None;
        let mut longest: Option<(f64, f64)> = None;
        for gap in &self.gaps {
            let pieces = if self.zones.is_empty() {
                vec![(t0, t1)]
            } else {
                gap.span.overlaps(t0, t1)
            };
            if pieces.is_empty() {
                continue;
            }
            match (gap.side, side) {
                (Some(here), Some(known)) if here != known => {
                    return Err(format!(
                        "the section changes sides of loop {wire:?} along a span with no \
                         crossing of it"
                    ));
                }
                (Some(here), _) => side = Some(here),
                (None, _) => return Err(undecided()),
            }
            for piece in pieces {
                if longest.is_none_or(|(lo, hi)| piece.1 - piece.0 > hi - lo) {
                    longest = Some(piece);
                }
            }
        }
        let (Some(side), Some((lo, hi))) = (side, longest) else {
            return Ok(None);
        };
        let probe = frame.project(conic.point((hi - lo).mul_add(0.381_966, lo)));
        if ray_side(probe, &self.outline, 0.5 * band) != Some(side) {
            return Err(format!(
                "a probe of the section disagrees with the side of loop {wire:?} its span was \
                 followed on"
            ));
        }
        Ok(Some(side))
    }
}

/// The roots of the section with the straight edge `from → to` that lie on
/// the edge (to within the band past its ends): the section's parameter at
/// the root and the point, clamped to the edge.
///
/// The chord's half-length is `√(reach · (2h − reach))` over the support
/// distance `h` and the line's speed in the chart that maps the section to
/// the unit circle: `1 − (chart distance)²` written without cancellation,
/// so a shallow crossing keeps its accuracy.
///
/// The parameter is read off the root in that chart, where it is a point
/// of the unit circle to rounding, and not off the 3D point: the chart
/// angle of a point is its parameter only ON the section, and a point
/// rounded in model coordinates is off an eccentric one by enough to move
/// it along the curve (the rounding of a coordinate of 1e6, times a ratio
/// of 1000, is a tolerance band).
fn line_roots(conic: &SectionConic, from: Point3, to: Point3, band: f64) -> Vec<(f64, Point3)> {
    let [(u, a), (v, b)] = conic.axes;
    let normal = u.cross(v);
    let edge = to - from;
    let in_plane = edge - normal * normal.dot(edge);
    let length = in_plane.length();
    if length.is_nan() || length <= 0.0 {
        return Vec::new();
    }
    let dir = in_plane * (1.0 / length);
    let w = from - conic.center;
    let mut away = normal.cross(dir);
    let mut offset = w.dot(away);
    if offset < 0.0 {
        away = -away;
        offset = -offset;
    }
    let support = (a * away.dot(u)).hypot(b * away.dot(v));
    let reach = support - offset;
    if !reach.is_finite() || support <= 0.0 || reach < 0.0 {
        return Vec::new();
    }
    let (wx, wy) = (w.dot(u) / a, w.dot(v) / b);
    let (dx, dy) = (dir.dot(u) / a, dir.dot(v) / b);
    let chart_speed = dx.hypot(dy);
    let mid = -wx.mul_add(dx, wy * dy) / (chart_speed * chart_speed);
    let half = (reach * 2.0f64.mul_add(support, -reach).max(0.0)).sqrt() / (support * chart_speed);
    let ends = if half > 0.0 {
        vec![mid - half, mid + half]
    } else {
        vec![mid]
    };
    ends.into_iter()
        .filter(|s| *s >= -band && *s <= length + band)
        .map(|s| {
            let t = dy.mul_add(s, wy).atan2(dx.mul_add(s, wx));
            (t.rem_euclid(TAU), from + dir * s.clamp(0.0, length))
        })
        .collect()
}

/// Where the section lies within `band` of the straight edge `from → to`:
/// within the band of its line, between its ends moved out by the band.
fn line_stretch(
    conic: &SectionConic,
    from: Point3,
    to: Point3,
    band: f64,
) -> (Vec<Span>, [f64; 2]) {
    let [(u, a), (v, b)] = conic.axes;
    let normal = u.cross(v);
    let edge = to - from;
    let in_plane = edge - normal * normal.dot(edge);
    let length = in_plane.length();
    if length.is_nan() || length <= 0.0 {
        return (Vec::new(), [0.0, PI]);
    }
    let dir = in_plane * (1.0 / length);
    let across = normal.cross(dir);
    let w = conic.center - from;
    let (cu, cv) = (a * across.dot(u), b * across.dot(v));
    let phase = cv.atan2(cu);
    let slab = sinusoid_band(cu.hypot(cv), phase, w.dot(across), -band, band);
    let (du, dv) = (a * dir.dot(u), b * dir.dot(v));
    let along = sinusoid_band(du.hypot(dv), dv.atan2(du), w.dot(dir), -band, length + band);
    (intersect(&slab, &along), [phase, phase + PI])
}

/// Where a circular section lies within the band of a circular edge on the
/// circle `rim_center`, `rim_radius`: the whole circle for `arc = None`,
/// otherwise the arc about the unit direction `mid` (from the rim's
/// centre) reaching the angle `half` either side of it. `None` for an arc
/// too small to bracket (a radius of a dozen bands or less).
///
/// An arc's zone is the EDGE's, not its circle's: the annulus one band
/// either side of the circle, between the two rays from the centre
/// through the arc's ends, each turned outward by the angle a band
/// subtends at the annulus' inner radius. That holds every point within
/// the band of the arc (one within the band of an end lies in the annulus
/// and no farther round than that), and every point of it is within
/// [`ZONE_REACH`] bands of the arc.
///
/// Each ray bounds a half-plane through the centre, and the side of it a
/// point of the section lies on is a sinusoid of the parameter. An arc of
/// less than a half turn is the two half-planes' intersection; a longer
/// one is their union. (A chord of the arc moved by the band is not such a
/// bound: on a short arc, whose sagitta is under the band, it lets in the
/// carrier for the square root of the band either side.)
fn arc_stretch(
    conic: &SectionConic,
    rim_center: Point3,
    rim_radius: f64,
    arc: Option<(Vec3, f64)>,
    model: SectionTolerance,
) -> Option<(Vec<Span>, [f64; 2])> {
    let [(u, r), (v, _)] = conic.axes;
    let band = model.band;
    let dc = conic.center - rim_center;
    let (du, dv) = (dc.dot(u), dc.dot(v));
    // |S(t) − rim centre|² = |dc|² + r² + 2 r (dc·u cos t + dc·v sin t).
    let phase = dv.atan2(du);
    let inner = (rim_radius - band).max(0.0);
    let outer = rim_radius + band;
    let annulus = sinusoid_band(
        2.0 * r * du.hypot(dv),
        phase,
        du.mul_add(du, dv * dv) + r * r,
        inner * inner,
        outer * outer,
    );
    let tangencies = [phase, phase + PI];
    let Some((mid, half)) = arc else {
        return Some((annulus, tangencies));
    };
    // The sector's outer corner: one band out, and the band's angle past
    // the end at radius `rim_radius + band`.
    let corner = ((rim_radius + band) * rim_radius / ((rim_radius - band) * (rim_radius - band)))
        .mul_add(1.0, 1.0)
        .sqrt();
    if rim_radius <= band || corner.is_nan() || corner > ZONE_REACH {
        return None;
    }
    let reach = half + (band / (rim_radius - band)).asin();
    if reach >= PI {
        return Some((annulus, tangencies));
    }
    // (S(t) − rim centre) · n ≥ 0 for the inward normal `n` of each ray.
    let side = u.cross(v).cross(mid);
    let (sin, cos) = reach.sin_cos();
    let half_plane = |normal: Vec3| {
        let (nu, nv) = (normal.dot(u), normal.dot(v));
        sinusoid_band(
            r * nu.hypot(nv),
            nv.atan2(nu),
            dc.dot(normal),
            0.0,
            f64::INFINITY,
        )
    };
    let (upper, lower) = (
        half_plane(mid * sin - side * cos),
        half_plane(mid * sin + side * cos),
    );
    let sector = if reach <= FRAC_PI_2 {
        intersect(&upper, &lower)
    } else {
        upper.into_iter().chain(lower).collect()
    };
    Some((intersect(&annulus, &sector), tangencies))
}

/// Read one loop of a planar face against the section: `None` for a loop
/// with an edge that has no exact contact with it here (an ellipse, a
/// spline or an open conic; a circular edge under an elliptical section,
/// which is a quartic; an arc without a stored parameter range, or of a
/// radius too small for its zone to be bracketed).
fn read_loop(
    topo: &Topology,
    wire_id: WireId,
    section: &EdgeCurve,
    conic: &SectionConic,
    frame: &PlaneFrame,
    tol: Tolerance,
    model: SectionTolerance,
) -> Option<LoopRead> {
    let wire = topo.wire(wire_id).ok()?;
    let count = wire.edges().len();
    let mut read = LoopRead {
        outline: Vec::with_capacity(count),
        vertices: Vec::with_capacity(count),
        stretches: Vec::new(),
        roots: Vec::new(),
        tangencies: Vec::with_capacity(count),
        chords: Vec::new(),
        segments: Vec::with_capacity(count),
    };
    for (index, oe) in wire.edges().iter().enumerate() {
        let edge = topo.edge(oe.edge()).ok()?;
        let sp = topo.vertex(edge.start()).ok()?.point();
        let ep = topo.vertex(edge.end()).ok()?.point();
        let (from, to) = if oe.is_forward() { (sp, ep) } else { (ep, sp) };
        read.vertices.push(from);
        // The loop vertex a root at `p` sits at.
        let vertex_at = |p: Point3| {
            if (p - from).length() <= model.band {
                Some(index)
            } else if (p - to).length() <= model.band {
                Some((index + 1) % count)
            } else {
                None
            }
        };
        match (edge.curve(), section) {
            (EdgeCurve::Line, _) => {
                read.segments.push(Some((from, to)));
                read.outline.push(PlanarLoopEdge::Segment(
                    frame.project(from),
                    frame.project(to),
                ));
                let (spans, tangencies) = line_stretch(conic, from, to, model.band);
                read.tangencies.push(tangencies);
                read.stretches
                    .extend(spans.into_iter().map(|span| Stretch { span, edge: index }));
                for (t, root) in line_roots(conic, from, to, model.band) {
                    let vertex = vertex_at(root);
                    // A root within the band of an end of the edge is that
                    // vertex. Its parameter stays the root's own: the root
                    // is a point of the section, the vertex only near it,
                    // and the chart angle of a point off an eccentric
                    // section is not the parameter of its foot.
                    let at = match vertex {
                        Some(v) if v == index => from,
                        Some(_) => to,
                        None => root,
                    };
                    read.roots.push(Root {
                        t,
                        at,
                        arc: None,
                        edge: index,
                        vertex,
                    });
                }
                if let LineContact::Cross(chord) = conic.line_contact(sp, ep, tol) {
                    read.chords.push((index, oe.edge(), chord));
                }
            }
            (EdgeCurve::Circle(rim), EdgeCurve::Circle(circle)) => {
                read.segments.push(None);
                let (t0, t1) = super::super::helpers::authoritative_edge_domain(
                    edge,
                    oe.edge(),
                    "plane loop certificate",
                )
                .ok()?;
                let (a, b) = if oe.is_forward() { (t0, t1) } else { (t1, t0) };
                let at = |t: f64| {
                    edge.curve()
                        .evaluate_with_endpoints((b - a).mul_add(t, a), sp, ep)
                };
                read.outline.push(PlanarLoopEdge::project(
                    edge.curve(),
                    at,
                    edge.is_closed(),
                    frame,
                    tol.angular,
                )?);
                let arc = (!edge.is_closed()).then(|| {
                    let mid = rim.evaluate(0.5 * (t0 + t1)) - rim.center();
                    (mid * (1.0 / rim.radius()), 0.5 * (t1 - t0).abs())
                });
                let (spans, tangencies) =
                    arc_stretch(conic, rim.center(), rim.radius(), arc, model)?;
                read.tangencies.push(tangencies);
                read.stretches
                    .extend(spans.into_iter().map(|span| Stretch { span, edge: index }));
                // A root of the two circles is the edge's only within the
                // arc's own turn, the band past its ends at most: the rest
                // of the carrier bounds nothing.
                let on_edge = |p: Point3| {
                    let Some((_, half)) = arc else { return true };
                    let slack = model.band / rim.radius();
                    let from_mid = (rim.project(p) - 0.5 * (t0 + t1) + PI).rem_euclid(TAU) - PI;
                    from_mid.abs() <= half + slack
                };
                for (at, t) in circle_arc_crossings(edge, oe.edge(), sp, ep, rim, circle, tol)
                    .ok()?
                    .into_iter()
                    .filter(|(at, _)| on_edge(*at))
                {
                    let vertex = if edge.is_closed() {
                        ((at - from).length() <= model.band).then_some(index)
                    } else {
                        vertex_at(at)
                    };
                    read.roots.push(Root {
                        t: t.rem_euclid(TAU),
                        at,
                        arc: Some(oe.edge()),
                        edge: index,
                        vertex,
                    });
                }
            }
            // B24: exhaustive over the loop edge × section pair — a
            // circular edge under an elliptical section, and every ellipse,
            // NURBS or open-conic edge, has no exact contact here, so the
            // loop is not read. The section is a closed circle or ellipse
            // (the caller's own gate).
            (
                EdgeCurve::Circle(_),
                EdgeCurve::Line
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_),
            )
            | (
                EdgeCurve::Ellipse(_)
                | EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_),
                _,
            ) => return None,
        }
    }
    Some(read)
}

/// Decide a loop's zones: its crossings, its touches, and the reasons a
/// span kept beside one of its crossings must be refused.
#[allow(clippy::too_many_lines)]
fn certify_loop(
    read: LoopRead,
    wire: WireId,
    conic: &SectionConic,
    frame: &PlaneFrame,
    model: SectionTolerance,
    partner: Option<&FaceSurface>,
) -> LoopCertificate {
    let LoopRead {
        outline,
        vertices,
        stretches,
        mut roots,
        tangencies,
        chords,
        segments,
    } = read;
    let Some(merged) = merge_stretches(stretches) else {
        return LoopCertificate {
            wire,
            outline,
            vertices,
            segments,
            zones: Vec::new(),
            gaps: Vec::new(),
            coincident: true,
        };
    };
    // The side of each gap: one point decides it, and the first of a few
    // spread along it that the test does not abstain on is taken.
    let side_at = |t: f64| loop_side(frame.project(conic.point(t)), &outline, 0.5 * model.band);
    let side_of = |span: Span| {
        [0.5, 0.25, 0.75, 0.125, 0.875, 0.0625, 0.9375]
            .into_iter()
            .find_map(|f: f64| side_at(span.length().mul_add(f, span.lo)))
    };
    let count = merged.len();
    let gaps: Vec<Gap> = if count == 0 {
        let span = Span { lo: 0.0, hi: TAU };
        vec![Gap {
            span,
            side: side_of(span),
        }]
    } else {
        (0..count)
            .map(|i| {
                let lo = merged[i].0.hi;
                let next = merged[(i + 1) % count].0.lo;
                let hi = if next > lo { next } else { next + TAU };
                let span = Span { lo, hi };
                Gap {
                    span,
                    side: side_of(span),
                }
            })
            .collect()
    };

    let edge_roots = roots.clone();
    // The loop vertices the section passes within the band of are corner
    // candidates too, whether or not an edge's root landed on them.
    for (index, &vertex) in vertices.iter().enumerate() {
        let (distance, t) = conic.nearest(vertex);
        if distance <= model.band && !roots.iter().any(|root| root.vertex == Some(index)) {
            roots.push(Root {
                t: t.rem_euclid(TAU),
                at: vertex,
                arc: None,
                edge: index,
                vertex: Some(index),
            });
        }
    }

    // A root belongs to the zone nearest it along the section. It lies in
    // that zone but for the rounding of the zone's ends, which are sums of
    // angles: a few units in the last place of a parameter, times the
    // section's speed, is a length of up to a few bands on an eccentric
    // section far from the origin (the band scales with the same
    // coordinates). The weld distance bounds it, and two zones that close
    // hold crossings the separation rule refuses anyway.
    let zone_of: Vec<Option<usize>> = roots
        .iter()
        .map(|root| {
            merged
                .iter()
                .enumerate()
                .map(|(zone, (span, _))| (zone, span.clearance(root.t) * conic.speed(root.t)))
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .filter(|(_, clearance)| *clearance <= model.weld)
                .map(|(zone, _)| zone)
        })
        .collect();

    let mut zones = Vec::with_capacity(count);
    for (i, (span, edges)) in merged.into_iter().enumerate() {
        let before = gaps[(i + count - 1) % count].side;
        let after = gaps[i].side;
        let mut interior: Vec<Root> = Vec::new();
        let mut corners: Vec<Root> = Vec::new();
        for (root, _) in roots
            .iter()
            .zip(&zone_of)
            .filter(|(_, zone)| **zone == Some(i))
        {
            let list = if root.vertex.is_some() {
                &mut corners
            } else {
                &mut interior
            };
            if !list
                .iter()
                .any(|kept| (kept.at - root.at).length() <= model.band)
            {
                list.push(*root);
            }
        }
        let contact = match (before, after) {
            (Some(a), Some(b)) if a == b => {
                // The touch is reported at the corner the section passes,
                // or where it comes nearest the one edge it runs beside.
                let at = corners.first().map_or_else(
                    || {
                        let t = edges
                            .iter()
                            .flat_map(|&edge| tangencies[edge])
                            .find(|&t| span.clearance(t) <= 0.0)
                            .unwrap_or(0.5 * (span.lo + span.hi));
                        conic.point(t)
                    },
                    |corner| corner.at,
                );
                Contact::Touch(at)
            }
            (Some(_), Some(_)) => {
                let pick = match (interior.as_slice(), corners.as_slice()) {
                    ([root], _) | (_, [root]) => Some(*root),
                    _ => None,
                };
                Contact::Crossing(pick.map_or_else(
                    || {
                        let t = 0.5 * (span.lo + span.hi);
                        let at = conic.point(t);
                        Crossing {
                            t: t.rem_euclid(TAU),
                            at,
                            arc: None,
                            taint: Some(format!(
                                "a section changes sides of loop {wire:?} of a planar face near \
                                 {at:?}, where {} crossing(s) with its edges and {} of its \
                                 vertices lie within the tolerance of it; the contact cannot be \
                                 told from a tangency",
                                interior.len(),
                                corners.len()
                            )),
                            edge: None,
                        }
                    },
                    |root| Crossing {
                        t: root.t,
                        at: root.at,
                        arc: root.arc,
                        taint: None,
                        edge: root.vertex.is_none().then_some(root.edge),
                    },
                ))
            }
            _ => Contact::Undecided,
        };
        zones.push(Zone { span, contact });
    }

    let mut certificate = LoopCertificate {
        wire,
        outline,
        vertices,
        segments,
        zones,
        gaps,
        coincident: false,
    };
    certificate.taint_unresolved_chords(&chords, &edge_roots, model, partner);
    certificate.taint_close_crossings(model);
    if let Err(why) = certificate.alternates() {
        for crossing in certificate.crossings_mut() {
            crossing.taint.get_or_insert_with(|| why.clone());
        }
    }
    certificate
}

impl LoopCertificate {
    /// Mark the crossings of a straight edge whose chord with the section is
    /// not resolved: the edge stays within the tolerance of the surface the
    /// section was cut from along it ([`Chord::enters`]), or its crossings
    /// do not bound it ([`Chord::resolved_by`]).
    fn taint_unresolved_chords(
        &mut self,
        chords: &[(usize, EdgeId, Chord)],
        roots: &[Root],
        model: SectionTolerance,
        partner: Option<&FaceSurface>,
    ) {
        for &(index, edge_id, chord) in chords {
            let on_edge: Vec<Point3> = roots
                .iter()
                .filter(|root| root.edge == index && root.arc.is_none())
                .map(|root| root.at)
                .collect();
            // An edge inside the section for less than the separation
            // distance ends within it of a vertex: its contact is decided
            // with the neighbouring edge's.
            let why = if chord.on_edge() > model.separate
                && partner.is_some_and(|surface| !chord.enters(surface))
            {
                Some(unresolved_chord_message(edge_id, None, chord))
            } else if chord.resolved_by(&on_edge, model.separate) {
                None
            } else {
                Some(unresolved_chord_message(edge_id, Some(&on_edge), chord))
            };
            let Some(why) = why else { continue };
            for crossing in self.crossings_mut() {
                if crossing.edge == Some(index) {
                    crossing.taint.get_or_insert_with(|| why.clone());
                }
            }
        }
    }

    /// Mark the crossings of this loop closer than the separation distance
    /// to one another, or to a vertex of the loop they are not at.
    fn taint_close_crossings(&mut self, model: SectionTolerance) {
        let wire = self.wire;
        let points: Vec<Point3> = self.crossings().map(|crossing| crossing.at).collect();
        let vertices = self.vertices.clone();
        for (index, crossing) in self.crossings_mut().enumerate() {
            if crossing.taint.is_some() {
                continue;
            }
            let near_crossing = points
                .iter()
                .enumerate()
                .any(|(other, p)| other != index && (*p - crossing.at).length() <= model.separate);
            if near_crossing {
                crossing.taint = Some(close_crossings_message(
                    wire,
                    crossing.at,
                    model.separate,
                    "the section passes more than the tolerance across the hole between them",
                ));
                continue;
            }
            crossing.taint = near_vertex(&vertices, crossing.at, model).map(|distance| {
                close_crossings_message(
                    wire,
                    crossing.at,
                    model.separate,
                    &format!("one of them, {distance:e} away, is a vertex of the face"),
                )
            });
        }
    }
}

/// The distance from `at` to a vertex in `vertices` that it is near but not
/// at: farther than the band, no farther than the separation distance.
fn near_vertex(vertices: &[Point3], at: Point3, model: SectionTolerance) -> Option<f64> {
    vertices
        .iter()
        .map(|vertex| (*vertex - at).length())
        .find(|distance| *distance > model.band && *distance <= model.separate)
}

/// One hole of a planar face against `section`, on its own: `None` for a
/// hole that cannot be read or that the section runs along all the way
/// round.
#[cfg(test)]
pub(super) fn certify_hole(
    topo: &Topology,
    wire: WireId,
    section: &EdgeCurve,
    partner: Option<&FaceSurface>,
    tol: Tolerance,
) -> Option<LoopCertificate> {
    let conic = SectionConic::of_curve(section)?;
    let model = conic.tolerance(tol);
    let [(u, _), (v, _)] = conic.axes;
    let frame = PlaneFrame::from_normal_and_point(u.cross(v), conic.center);
    let read = read_loop(topo, wire, section, &conic, &frame, tol, model)?;
    let certificate = certify_loop(read, wire, &conic, &frame, model, partner);
    (!certificate.coincident).then_some(certificate)
}

// ── The planar face ─────────────────────────────────────────────────────

/// A closed section against every loop of the planar face it lies in.
pub(super) struct PlaneSectionCertificate {
    face: FaceId,
    conic: SectionConic,
    frame: PlaneFrame,
    model: SectionTolerance,
    /// The holes that could be read.
    holes: Vec<LoopCertificate>,
    /// The outline, when it could be read.
    outline: Option<LoopCertificate>,
    /// The holes that could not be read.
    unread_holes: Vec<UnreadHole>,
    /// The linear tolerance the face splitter splits a hole with.
    linear: f64,
}

impl PlaneSectionCertificate {
    /// Read every loop of `face` against `section`, a closed circle or
    /// ellipse lying in its plane; `partner` is the surface the section was
    /// cut from. `None` for any other section, or a face that cannot be
    /// read at all.
    pub(super) fn of_face(
        topo: &Topology,
        face: FaceId,
        section: &EdgeCurve,
        partner: Option<&FaceSurface>,
        tol: Tolerance,
    ) -> Option<Self> {
        let conic = SectionConic::of_curve(section)?;
        let model = conic.tolerance(tol);
        let [(u, _), (v, _)] = conic.axes;
        let frame = PlaneFrame::from_normal_and_point(u.cross(v), conic.center);
        let face_data = topo.face(face).ok()?;
        let mut holes = Vec::new();
        let mut unread_holes = Vec::new();
        let reach = conic.bounds().expanded(model.band);
        for &wire in face_data.inner_wires() {
            // A hole whose box the section's box does not meet has no
            // contact with it and holds no point of it: it is not read at
            // all, so a face with many openings pays for the ones the
            // section reaches.
            let bounds = loop_bounds(topo, wire);
            if bounds.is_some_and(|bounds| !bounds.intersects(reach)) {
                continue;
            }
            match read_loop(topo, wire, section, &conic, &frame, tol, model) {
                Some(read) => {
                    holes.push(certify_loop(read, wire, &conic, &frame, model, partner));
                }
                // A hole with no box is everywhere.
                None => unread_holes.push(UnreadHole {
                    wire,
                    bounds: bounds.unwrap_or(Aabb3 {
                        min: Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
                        max: Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
                    }),
                    edges: unread_edges(topo, wire),
                }),
            }
        }
        // The outline is only consulted for spans that end on a hole.
        let outline = holes
            .iter()
            .any(|hole| hole.crossings().next().is_some())
            .then(|| {
                let wire = face_data.outer_wire();
                read_loop(topo, wire, section, &conic, &frame, tol, model)
                    .map(|read| certify_loop(read, wire, &conic, &frame, model, None))
            })
            .flatten();
        Some(Self {
            face,
            conic,
            frame,
            model,
            holes,
            outline,
            unread_holes,
            linear: tol.linear,
        })
    }

    /// The crossings of the face's holes, hole by hole.
    pub(super) fn hole_crossings(&self) -> impl Iterator<Item = (&LoopCertificate, &Crossing)> {
        self.holes
            .iter()
            .flat_map(|hole| hole.crossings().map(move |crossing| (hole, crossing)))
    }

    /// Where the section touches a hole without crossing it.
    pub(super) fn hole_touches(&self) -> impl Iterator<Item = Point3> + '_ {
        self.holes.iter().flat_map(LoopCertificate::touches)
    }

    /// For `at`, one of the points [`Self::hole_touches`] gave: whether the
    /// stretch of the section that touches the hole there runs into one of
    /// `spans` (parameter ranges of the section). `None` for any other
    /// point.
    ///
    /// A touch is a stretch, not a point: the section stays within the band
    /// of the hole along it. It bears on a kept span only when that stretch
    /// reaches into the span, which the stretch itself decides exactly. A
    /// touch on the far side of a crossing (inside the hole, where the
    /// span was dropped) does not.
    pub(super) fn hole_touch_on(&self, at: Point3, spans: &[(f64, f64)]) -> Option<bool> {
        self.holes
            .iter()
            .flat_map(|hole| hole.zones.iter())
            .find(|zone| matches!(zone.contact, Contact::Touch(p) if (p - at).length() <= 0.0))
            .map(|zone| {
                spans
                    .iter()
                    .any(|&(t0, t1)| !zone.span.overlaps(t0, t1).is_empty())
            })
    }

    /// Whether `p`, a point of the face's plane, lies inside one of the
    /// holes that were read, farther than `band` from its boundary.
    pub(super) fn inside_a_hole(&self, p: Point3, band: f64) -> bool {
        let uv = self.frame.project(p);
        self.holes
            .iter()
            .any(|hole| planar_loop_side(uv, &hole.outline, band) == Some(true))
    }

    /// The gate for the spans between `splits`, the points the section is
    /// split at, sorted by parameter.
    pub(super) fn plan(&self, splits: &[(f64, Point3)]) -> SpanPlan<'_> {
        let model = self.model;
        // How far the section strays from the point `at` between the
        // parameters `from` and `to`, the short way round.
        let strays_from = |at: Point3, from: f64, to: f64| {
            const STEPS: u32 = 16;
            let turn = (to - from + PI).rem_euclid(TAU) - PI;
            (0..=STEPS)
                .map(|k| {
                    let t = turn.mul_add(f64::from(k) / f64::from(STEPS), from);
                    (self.conic.point(t) - at).length()
                })
                .fold(0.0, f64::max)
        };
        let mut ends: Vec<SpanEnd> = splits
            .iter()
            .map(|&(t, at)| {
                let crossing = self
                    .hole_crossings()
                    .find(|(_, crossing)| (crossing.at - at).length() <= model.band);
                // One parameter. The span ends at the split's parameter,
                // and the face splitter reads a section end's parameter off
                // its vertex: from the crossing's own parameter to either,
                // the section must stay within the band of the crossing's
                // point, which is a point of the hole's boundary. A quarter
                // band over is the rounding of the points compared.
                let taint = crossing.and_then(|(hole, crossing)| {
                    crossing.taint.clone().or_else(|| {
                        let split = strays_from(crossing.at, crossing.t, t);
                        let vertex =
                            strays_from(crossing.at, crossing.t, self.conic.angle(crossing.at));
                        (split.max(vertex) > END_SLACK * model.band).then(|| {
                            format!(
                                "a section crosses hole {:?} of planar face {:?} at {:?}; from \
                                 the crossing's parameter to the one it is split at the section \
                                 runs {split:e} from that point, and to the one the point \
                                 re-projects to {vertex:e}, farther than the tolerance {:e}: a \
                                 span ending there would run past the crossing",
                                hole.wire, self.face, crossing.at, model.band
                            )
                        })
                    })
                });
                SpanEnd {
                    at,
                    on_hole: crossing.is_some(),
                    taint,
                }
            })
            .collect();
        // A hole's crossing closer than the separation distance to another
        // split point of the section, or to a vertex of another loop it is
        // not at, cannot be kept apart from it downstream.
        let all_vertices: Vec<Point3> = self
            .holes
            .iter()
            .chain(self.outline.as_ref())
            .flat_map(|certificate| certificate.vertices.iter().copied())
            .collect();
        for index in 0..ends.len() {
            if !ends[index].on_hole || ends[index].taint.is_some() {
                continue;
            }
            let at = ends[index].at;
            let near_split = ends
                .iter()
                .enumerate()
                .any(|(other, end)| other != index && (end.at - at).length() <= model.separate);
            ends[index].taint = if near_split {
                Some(format!(
                    "a section crosses a hole of planar face {:?} at {at:?} closer than {:e} to \
                     another point it is split at, the distance at which two crossings are one; \
                     the contact cannot be told from a tangency",
                    self.face, model.separate
                ))
            } else {
                near_vertex(&all_vertices, at, model).map(|distance| {
                    format!(
                        "a section crosses a hole of planar face {:?} at {at:?}, {distance:e} \
                         from a vertex of the face: closer than {:e}, the distance at which two \
                         vertices are one, yet not at it",
                        self.face, model.separate
                    )
                })
            };
        }
        // A hole crossing that is not among the split points would leave a
        // span running across the hole.
        let mut strays: Vec<(f64, String)> = self
            .hole_crossings()
            .filter(|(_, crossing)| {
                !splits
                    .iter()
                    .any(|(_, at)| (*at - crossing.at).length() <= model.band)
            })
            .map(|(hole, crossing)| {
                (
                    crossing.t,
                    format!(
                        "a section crosses hole {:?} of planar face {:?} at {:?}, which is not \
                         among the points it is split at",
                        hole.wire, self.face, crossing.at
                    ),
                )
            })
            .collect();
        // So would a crossing of the outline, where it was read (a section
        // that crosses a hole): the span over it lies partly on the face and
        // partly off it, and no one point of it decides the whole. The
        // collector finds the outline's crossings its own way and takes two
        // within the separation distance for one, so that is how near a
        // split point must be.
        if let Some(outline) = &self.outline {
            strays.extend(
                outline
                    .crossings()
                    .filter(|crossing| {
                        !splits
                            .iter()
                            .any(|(_, at)| (*at - crossing.at).length() <= model.separate)
                    })
                    .map(|crossing| {
                        (
                            crossing.t,
                            format!(
                                "a section that crosses a hole of planar face {:?} crosses the \
                                 face's outline at {:?}, which is not among the points it is \
                                 split at",
                                self.face, crossing.at
                            ),
                        )
                    }),
            );
        }
        SpanPlan {
            certificate: self,
            ends,
            strays,
        }
    }

    /// Whether the span `(t0, t1)` of the section, from the split point
    /// `start` to the split point `end`, lies on the face: inside the
    /// outline and outside every hole. `Err` says why that cannot be
    /// certified.
    fn on_face(
        &self,
        (t0, t1): (f64, f64),
        (start, end): (Point3, Point3),
    ) -> Result<bool, String> {
        for hole in &self.unread_holes {
            if self.unread_side(hole, (t0, t1))? {
                return Ok(false);
            }
        }
        // Both ends within the band of the section's own point at their
        // parameter: an end that is no hole's (the outline's, the partner
        // face's seam) had its parameter read off its point.
        for (t, at) in [(t0, start), (t1, end)] {
            let off = (self.conic.point(t) - at).length();
            if off > END_SLACK * self.model.band {
                return Err(format!(
                    "a span of the section that ends on a hole of planar face {:?} ends at \
                     {at:?}, {off:e} from the section's point at its parameter: farther than \
                     the tolerance {:e}",
                    self.face, self.model.band
                ));
            }
        }
        let Some(outline) = &self.outline else {
            return Err(format!(
                "the outline of planar face {:?} has an edge with no exact contact with the \
                 section",
                self.face
            ));
        };
        let mut definite = vec![(t0, t1)];
        let mut sides = Vec::with_capacity(self.holes.len() + 1);
        for (certificate, is_hole) in
            std::iter::once((outline, false)).chain(self.holes.iter().map(|hole| (hole, true)))
        {
            let (side, pieces) = certificate
                .side_along((t0, t1), (start, end), self.model.band)
                .map_err(|why| format!("{why} (planar face {:?})", self.face))?;
            definite = definite
                .iter()
                .flat_map(|&(a, b)| {
                    pieces
                        .iter()
                        .map(move |&(c, d)| (a.max(c), b.min(d)))
                        .filter(|(lo, hi)| hi > lo)
                })
                .collect();
            sides.push((certificate, side, is_hole));
        }
        // One point farther than the band from every loop, tested against
        // each loop again: the span's side is read off a probe, not off the
        // bookkeeping above.
        let Some(&(lo, hi)) = definite
            .iter()
            .max_by(|a, b| (a.1 - a.0).total_cmp(&(b.1 - b.0)))
        else {
            return Err(format!(
                "a span of the section has no point farther than the tolerance from every loop \
                 of planar face {:?}",
                self.face
            ));
        };
        let probe = self
            .frame
            .project(self.conic.point((hi - lo).mul_add(0.381_966, lo)));
        let mut on_face = true;
        for (certificate, side, is_hole) in sides {
            if ray_side(probe, &certificate.outline, 0.5 * self.model.band) != Some(side) {
                return Err(format!(
                    "a probe of the section disagrees with the side of loop {:?} of planar face \
                     {:?} its span was certified on",
                    certificate.wire, self.face
                ));
            }
            on_face &= side != is_hole;
        }
        Ok(on_face)
    }
}

/// How far past the band two computations of one point may put it: the
/// rounding of the coordinates compared, a quarter of the band at most
/// (the band is at least sixteen times that rounding).
const END_SLACK: f64 = 1.25;

impl PlaneSectionCertificate {
    /// Whether the span `(t0, t1)` of the section, which ends on no hole,
    /// passes through none: `Ok(true)` when it is clear of every hole of
    /// the face (it may be emitted on the collector's verdict alone),
    /// `Ok(false)` when it lies inside one (it is not a span of the face).
    ///
    /// See "A span that ends on no hole" in the module documentation for
    /// the argument and its premises.
    ///
    /// # Errors
    ///
    /// The reason the span can be shown neither clear of a hole nor inside
    /// it.
    fn clear_of_holes(&self, (t0, t1): (f64, f64)) -> Result<bool, String> {
        for hole in &self.holes {
            let side = hole
                .uncrossed_side((t0, t1), &self.conic, &self.frame, self.model.band)
                .map_err(|why| format!("{why} (a hole of planar face {:?})", self.face))?;
            if side == Some(true) {
                return Ok(false);
            }
        }
        for hole in &self.unread_holes {
            if self.unread_side(hole, (t0, t1))? {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// The exact box of the span `(t0, t1)` of the section: its ends, and
    /// the points where it runs farthest along each axis.
    fn span_bounds(&self, t0: f64, t1: f64) -> Option<Aabb3> {
        let [(u, a), (v, b)] = self.conic.axes;
        let mut points = vec![self.conic.point(t0), self.conic.point(t1)];
        for (uk, vk) in [(u.x(), v.x()), (u.y(), v.y()), (u.z(), v.z())] {
            let extreme = (b * vk).atan2(a * uk);
            for at in [extreme, extreme + PI] {
                // The first copy of the extreme at or after `t0`.
                let t = ((t0 - at) / TAU).ceil().mul_add(TAU, at);
                if t <= t1 {
                    points.push(self.conic.point(t));
                }
            }
        }
        Aabb3::try_from_points(points.iter().copied())
    }

    /// A lower bound on the distance from `p`, a point of the section's
    /// plane, to the section.
    ///
    /// Outside, the section lies behind its tangent line square to the
    /// gradient at `p`: the support distance along that direction. Inside,
    /// the bound is the largest radius `r` (found by bisection) for which
    /// the implicit form `F = (x/a)² + (y/b)²` cannot reach 1 anywhere in
    /// the disc of radius `r` about `p`: `F(p) + r · L(r) ≤ 1`, with `L(r)`
    /// the largest slope of `F` on that disc. (The chart that maps the
    /// section to the unit circle gives a bound too, but one short by the
    /// ratio of the semi-axes at the ends of an eccentric section.)
    fn section_clearance(&self, p: Point3) -> f64 {
        let [(u, a), (v, b)] = self.conic.axes;
        let w = p - self.conic.center;
        let (x, y) = (w.dot(u), w.dot(v));
        let implicit = (x / a).mul_add(x / a, (y / b) * (y / b));
        if implicit <= 1.0 {
            let fits = |r: f64| {
                let slope = 2.0 * ((x.abs() + r) / (a * a)).hypot((y.abs() + r) / (b * b));
                r.mul_add(slope, implicit) <= 1.0
            };
            let (mut lo, mut hi) = (0.0, a.min(b));
            for _ in 0..48 {
                let mid = 0.5 * (lo + hi);
                if fits(mid) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            return lo;
        }
        let (gx, gy) = (x / (a * a), y / (b * b));
        let slope = gx.hypot(gy);
        let (nx, ny) = (gx / slope, gy / slope);
        nx.mul_add(x, ny * y) - (a * nx).hypot(b * ny)
    }

    /// A lower bound on the distance from the convex hull of `points`
    /// (points of the section's plane) to the section; not positive when
    /// none can be shown.
    ///
    /// The region the section bounds is convex. A hull of points inside it
    /// lies inside it, and no nearer its boundary than its nearest corner.
    /// A hull of points outside it is clear of it by as much as every
    /// corner lies beyond one tangent line of the section (the one square
    /// to the gradient at the corners' mean).
    fn hull_clearance(&self, points: &[Point3]) -> f64 {
        let [(u, a), (v, b)] = self.conic.axes;
        let flat: Vec<(f64, f64)> = points
            .iter()
            .map(|p| {
                let w = *p - self.conic.center;
                (w.dot(u), w.dot(v))
            })
            .collect();
        let implicit = |&(x, y): &(f64, f64)| (x / a).mul_add(x / a, (y / b) * (y / b));
        if flat.iter().all(|p| implicit(p) <= 1.0) {
            return points
                .iter()
                .map(|p| self.section_clearance(*p))
                .fold(f64::INFINITY, f64::min);
        }
        #[allow(clippy::cast_precision_loss)]
        let count = flat.len() as f64;
        let (mx, my) = flat.iter().fold((0.0, 0.0), |(sx, sy), &(x, y)| {
            (sx + x / count, sy + y / count)
        });
        let (gx, gy) = (mx / (a * a), my / (b * b));
        let slope = gx.hypot(gy);
        if slope.is_nan() || slope <= 0.0 {
            return 0.0;
        }
        let (nx, ny) = (gx / slope, gy / slope);
        let support = (a * nx).hypot(b * ny);
        flat.iter()
            .map(|&(x, y)| nx.mul_add(x, ny * y) - support)
            .fold(f64::INFINITY, f64::min)
    }

    /// For a span the collector drops because a point of it is off the
    /// planar face: `Err` when a hole that could not be read can be shown
    /// neither clear of the span nor to hold it.
    ///
    /// The crossings of such a hole are not split points, so a span may run
    /// into it and out again, and be dropped whole for a midpoint inside
    /// it, material and all. (A hole that was read cannot do that: its
    /// crossings are split points, and a span between them is on one side
    /// of it.)
    fn dropped_whole(&self, (t0, t1): (f64, f64)) -> Result<(), String> {
        for hole in &self.unread_holes {
            self.unread_side(hole, (t0, t1))?;
        }
        Ok(())
    }

    /// Whether the span `(t0, t1)` of the section lies inside `hole`, a
    /// hole that could not be read.
    ///
    /// Every piece of every edge of the hole must be shown farther than the
    /// band from the span: by its distance from the span's exact box, or by
    /// [`Self::section_clearance`], each less the piece's own reach from the
    /// point it is measured at. An edge is halved until its pieces are
    /// (within a budget). The span then does not meet the hole's boundary,
    /// and the boundary's winding number about the span's midpoint, summed
    /// over the same pieces (each lies in a disc or a box that does not
    /// hold the midpoint, so it subtends less than a half turn there),
    /// says which side the whole span is on.
    ///
    /// # Errors
    ///
    /// The refusal for a hole that cannot be shown clear of the span.
    fn unread_side(&self, hole: &UnreadHole, (t0, t1): (f64, f64)) -> Result<bool, String> {
        // Pieces looked at, over the whole hole. An edge two bands clear of
        // the span along a tangency takes a few thousand.
        const BUDGET: usize = 1 << 16;
        let band = self.model.band;
        let refuse = |why: &str| {
            format!(
                "hole {:?} of planar face {:?} has an edge with no exact contact with the \
                 section, and {why} a span it would keep",
                hole.wire, self.face
            )
        };
        let Some(span) = self.span_bounds(t0, t1) else {
            return Err(refuse("cannot be measured against"));
        };
        // A hole whose box the span's box does not reach neither touches
        // the span nor holds it.
        if !hole.bounds.intersects(span.expanded(band)) {
            return Ok(false);
        }
        let Some(edges) = &hole.edges else {
            return Err(refuse("lies within reach of"));
        };
        let probe = self.frame.project(self.conic.point(0.5 * (t0 + t1)));
        let turn = |from: Point3, to: Point3| {
            let (a, b) = (
                self.frame.project(from) - probe,
                self.frame.project(to) - probe,
            );
            cross_2d(a, b).atan2(dot_2d(a, b))
        };
        // The distance from `p` to the span's box.
        let off_box = |p: Point3| {
            let axis = |x: f64, lo: f64, hi: f64| (lo - x).max(x - hi).max(0.0);
            let (dx, dy, dz) = (
                axis(p.x(), span.min.x(), span.max.x()),
                axis(p.y(), span.min.y(), span.max.y()),
                axis(p.z(), span.min.z(), span.max.z()),
            );
            dx.hypot(dy).hypot(dz)
        };
        let mut turned = 0.0;
        let mut budget = BUDGET;
        for edge in edges {
            let (at, speed, range): (&dyn Fn(f64) -> Point3, f64, (f64, f64)) = match edge {
                UnreadEdge::Boxed { bounds, from, to } => {
                    if bounds.intersects(span.expanded(band)) {
                        return Err(refuse("lies within reach of"));
                    }
                    turned += turn(*from, *to);
                    continue;
                }
                UnreadEdge::Spline { curve, forward } => {
                    // A piece of a spline lies in the hull of its control
                    // points. That hull is clear of the span when its box
                    // is; when every control point is inside the section
                    // and clear of it (the distance to the boundary of a
                    // convex region is least at a corner of the hull); or
                    // when one tangent line of the section has them all
                    // beyond it. Otherwise the piece is cut in two.
                    let mut pieces = vec![curve.clone()];
                    let mut along = 0.0;
                    while let Some(piece) = pieces.pop() {
                        budget = budget
                            .checked_sub(1)
                            .ok_or_else(|| refuse("could not be shown clear of"))?;
                        let net = piece.control_points();
                        let (Some(&first), Some(&last)) = (net.first(), net.last()) else {
                            return Err(refuse("cannot be measured against"));
                        };
                        let Some(bounds) = Aabb3::try_from_points(net.iter().copied()) else {
                            return Err(refuse("cannot be measured against"));
                        };
                        let clear = !bounds.intersects(span.expanded(band))
                            || self.hull_clearance(net) > band;
                        if clear {
                            along += turn(first, last);
                            continue;
                        }
                        let reach = (bounds.max - bounds.min).length();
                        if reach <= 0.25 * band {
                            return Err(refuse("comes within the tolerance of"));
                        }
                        let (lo, hi) = piece.domain();
                        let (left, right) =
                            remus_math::nurbs::knot_ops::curve_split(&piece, 0.5 * (lo + hi))
                                .map_err(|_| refuse("could not be shown clear of"))?;
                        pieces.extend([left, right]);
                    }
                    turned += if *forward { along } else { -along };
                    continue;
                }
                UnreadEdge::Segment(from, to) => (
                    &|s: f64| *from + (*to - *from) * s,
                    (*to - *from).length(),
                    (0.0, 1.0),
                ),
                UnreadEdge::Conic {
                    center,
                    x,
                    y,
                    from,
                    to,
                    speed,
                } => (
                    &|s: f64| {
                        let (sin, cos) = s.sin_cos();
                        *center + *x * cos + *y * sin
                    },
                    *speed,
                    (*from, *to),
                ),
            };
            let mut pieces = vec![range];
            while let Some((s0, s1)) = pieces.pop() {
                budget = budget
                    .checked_sub(1)
                    .ok_or_else(|| refuse("could not be shown clear of"))?;
                let middle = at(0.5 * (s0 + s1));
                let reach = 0.5 * speed * (s1 - s0).abs();
                if off_box(middle).max(self.section_clearance(middle)) > reach + band {
                    turned += turn(at(s0), at(s1));
                } else if reach <= 0.25 * band {
                    return Err(refuse("comes within the tolerance of"));
                } else {
                    let half = 0.5 * (s0 + s1);
                    pieces.extend([(s0, half), (half, s1)]);
                }
            }
        }
        let turns = (turned / TAU).round();
        if (turned - turns * TAU).abs() > 0.1 {
            return Err(refuse("does not close round"));
        }
        Ok(turns != 0.0)
    }
}

/// A hole of the planar face that could not be read against the section.
struct UnreadHole {
    wire: WireId,
    /// A box that holds the hole.
    bounds: Aabb3,
    /// Its edges in traversal order; `None` when one of them has no bound
    /// at all.
    edges: Option<Vec<UnreadEdge>>,
}

/// One edge of an [`UnreadHole`], as what bounds it.
enum UnreadEdge {
    /// A straight edge, in traversal order.
    Segment(Point3, Point3),
    /// An arc of a circle or an ellipse: `center + x cos s + y sin s` for
    /// `s` from `from` to `to`, covering at most `speed` per unit of `s`.
    Conic {
        center: Point3,
        x: Vec3,
        y: Vec3,
        from: f64,
        to: f64,
        speed: f64,
    },
    /// A spline with positive weights, cut to the edge's own range;
    /// `forward` says the hole traverses it along its parameter.
    Spline {
        curve: remus_math::nurbs::curve::NurbsCurve,
        forward: bool,
    },
    /// Any other edge: a box that holds it, and its ends in traversal
    /// order.
    Boxed {
        bounds: Aabb3,
        from: Point3,
        to: Point3,
    },
}

/// The edges of a hole that could not be read, as [`UnreadEdge`]s; `None`
/// when one of them is unbounded or cannot be looked up.
fn unread_edges(topo: &Topology, wire_id: WireId) -> Option<Vec<UnreadEdge>> {
    let wire = topo.wire(wire_id).ok()?;
    let mut edges = Vec::with_capacity(wire.edges().len());
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge()).ok()?;
        let sp = topo.vertex(edge.start()).ok()?.point();
        let ep = topo.vertex(edge.end()).ok()?.point();
        let (from, to) = if oe.is_forward() { (sp, ep) } else { (ep, sp) };
        let boxed = |points: &mut dyn Iterator<Item = Point3>| {
            Aabb3::try_from_points(points.chain([from, to])).map(|bounds| UnreadEdge::Boxed {
                bounds,
                from,
                to,
            })
        };
        let conic = |center: Point3, x: Vec3, y: Vec3| {
            let speed = x.length().max(y.length());
            if let Ok((t0, t1)) = edge.strict_domain() {
                let (from, to) = if oe.is_forward() { (t0, t1) } else { (t1, t0) };
                Some(UnreadEdge::Conic {
                    center,
                    x,
                    y,
                    from,
                    to,
                    speed,
                })
            } else {
                // No stored range: the whole carrier bounds it.
                let reach = Vec3::new(speed, speed, speed);
                boxed(&mut [center + reach, center + (-reach)].into_iter())
            }
        };
        edges.push(match edge.curve() {
            EdgeCurve::Line => UnreadEdge::Segment(from, to),
            EdgeCurve::Circle(c) => {
                conic(c.center(), c.u_axis() * c.radius(), c.v_axis() * c.radius())?
            }
            EdgeCurve::Ellipse(e) => conic(
                e.center(),
                e.u_axis() * e.semi_major(),
                e.v_axis() * e.semi_minor(),
            )?,
            EdgeCurve::NurbsCurve(n) => {
                // The curve between the edge's own parameters, by exact
                // knot insertion; without a stored range, or with a weight
                // that is not positive (no hull), its control net's box.
                let piece = edge.strict_domain().ok().and_then(|(t0, t1)| {
                    if n.weights().iter().any(|w| !w.is_finite() || *w <= 0.0) {
                        return None;
                    }
                    let (lo, hi) = (t0.min(t1), t0.max(t1));
                    let (d0, d1) = n.domain();
                    let mut piece = n.clone();
                    if hi < d1 {
                        piece = remus_math::nurbs::knot_ops::curve_split(&piece, hi).ok()?.0;
                    }
                    if lo > d0 {
                        piece = remus_math::nurbs::knot_ops::curve_split(&piece, lo).ok()?.1;
                    }
                    Some(UnreadEdge::Spline {
                        curve: piece,
                        forward: oe.is_forward() == (t0 <= t1),
                    })
                });
                match piece {
                    Some(piece) => piece,
                    None => boxed(&mut n.control_points().iter().copied())?,
                }
            }
            // An unbounded conic has no box.
            EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => return None,
        });
    }
    Some(edges)
}

/// The bounding box of a loop: its vertices, widened by each curved edge's
/// own reach (the radius of a circle, the larger semi-axis of an ellipse,
/// the control net of a spline). `None` for a loop that cannot be read or
/// has an unbounded edge.
fn loop_bounds(topo: &Topology, wire_id: WireId) -> Option<Aabb3> {
    let wire = topo.wire(wire_id).ok()?;
    let mut corners = Vec::new();
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge()).ok()?;
        corners.extend([
            topo.vertex(edge.start()).ok()?.point(),
            topo.vertex(edge.end()).ok()?.point(),
        ]);
        let (center, reach) = match edge.curve() {
            EdgeCurve::Line => continue,
            EdgeCurve::Circle(c) => (c.center(), c.radius()),
            EdgeCurve::Ellipse(e) => (e.center(), e.semi_major()),
            EdgeCurve::NurbsCurve(n) => {
                corners.extend(n.control_points().iter().copied());
                continue;
            }
            // An unbounded conic has no box.
            EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => return None,
        };
        let reach = Vec3::new(reach, reach, reach);
        corners.extend([center + reach, center + (-reach)]);
    }
    Aabb3::try_from_points(corners.iter().copied())
}

// ── The gate ────────────────────────────────────────────────────────────

struct SpanEnd {
    at: Point3,
    /// The end is a crossing of one of the face's holes.
    on_hole: bool,
    taint: Option<String>,
}

/// The decision for every span between consecutive split points of a
/// section: [`SpanPlan::keep`] is the one place a span that ends on a hole
/// of the planar face is emitted or refused.
pub(super) struct SpanPlan<'a> {
    certificate: &'a PlaneSectionCertificate,
    ends: Vec<SpanEnd>,
    /// Crossings of a hole, or of the outline where it was read, that are
    /// not split points: the parameter and the refusal for a span running
    /// over one.
    strays: Vec<(f64, String)>,
}

/// The collector's own test of a span: one point of it against the partner
/// face and the planar face's extent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Elsewhere {
    /// On both faces by that test.
    On,
    /// Off the partner face, or off the pair by a test that does not say
    /// which face: dropped, as it always was.
    Off,
    /// On the partner face and off the planar one.
    OffThePlane,
}

impl Elsewhere {
    /// The verdict of a collector whose test does not tell the two faces
    /// apart.
    pub(super) const fn of(on_pair: bool) -> Self {
        if on_pair { Self::On } else { Self::Off }
    }
}

/// What the gate decided for one span of a section.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Kept {
    /// Not emitted.
    No,
    /// Emitted on the collector's own test, as it always was: the span
    /// ends on no hole, and it is certified to pass through none.
    AsBefore,
    /// Emitted because it is certified on the face: it ends on a hole.
    Certified,
}

impl SpanPlan<'_> {
    /// Whether span `index` — from split point `index` to the next, over
    /// the parameters `(t0, t1)` — is emitted.
    ///
    /// `elsewhere` is the collector's own test of the span: whether it lies
    /// on the partner face and within the planar face's extent. A span that
    /// fails it is dropped, as it always was — but one dropped for the
    /// planar face must not run through a hole whose crossings were not
    /// split points ([`PlaneSectionCertificate::dropped_whole`]): it would
    /// take the material either side of the hole with it. A span that
    /// passes it and has
    /// no end on a hole is the span the collector emitted before holes were
    /// read; it is kept when it is certified to pass through no hole
    /// ([`PlaneSectionCertificate::clear_of_holes`]: no work for a face
    /// without a hole in reach of the section), and dropped when it lies
    /// inside one. A span that passes it and ends on a hole is new, and is
    /// kept only when it is certified on the face.
    ///
    /// # Errors
    ///
    /// [`AlgoError::IntersectionFailed`] when a span that passes
    /// `elsewhere` ends on a hole and cannot be certified on the face or
    /// off it, or ends on none and cannot be certified clear of every hole
    /// or inside one; when a span runs over a crossing that is not a split
    /// point; or when a span dropped for the planar face reaches a hole
    /// that could not be read.
    pub(super) fn keep(
        &self,
        index: usize,
        (t0, t1): (f64, f64),
        elsewhere: Elsewhere,
    ) -> Result<Kept, AlgoError> {
        let refuse = |why: &str| AlgoError::IntersectionFailed(why.to_owned());
        // A span over a crossing that is not a split point lies partly on
        // the face and partly off it: neither the collector's test of one
        // of its points nor a certificate of one side holds for it, kept or
        // dropped.
        let stray = self
            .strays
            .iter()
            .find(|(t, _)| (t - t0).rem_euclid(TAU) < t1 - t0);
        if let Some((_, why)) = stray {
            return Err(refuse(why));
        }
        let certificate = self.certificate;
        match elsewhere {
            Elsewhere::On => {}
            Elsewhere::Off => return Ok(Kept::No),
            Elsewhere::OffThePlane => {
                return certificate
                    .dropped_whole((t0, t1))
                    .map(|()| Kept::No)
                    .map_err(|why| refuse(&why));
            }
        }
        let count = self.ends.len();
        if count == 0 || (certificate.holes.is_empty() && certificate.unread_holes.is_empty()) {
            return Ok(Kept::AsBefore);
        }
        let ends = [&self.ends[index % count], &self.ends[(index + 1) % count]];
        if ends.iter().all(|end| !end.on_hole) {
            return certificate
                .clear_of_holes((t0, t1))
                .map(|clear| if clear { Kept::AsBefore } else { Kept::No })
                .map_err(|why| refuse(&why));
        }
        if let Some(why) = ends.iter().find_map(|end| end.taint.as_deref()) {
            return Err(refuse(why));
        }
        certificate
            .on_face((t0, t1), (ends[0].at, ends[1].at))
            .map(|on_face| if on_face { Kept::Certified } else { Kept::No })
            .map_err(|why| refuse(&why))
    }

    /// The postcondition on the spans that were kept (`kept` holds their
    /// indices): the face splitter, splitting each hole at the ends of the
    /// sections that reach it, puts a vertex on every straight hole edge
    /// exactly where the certificate put a crossing a kept span ends at,
    /// and nowhere else.
    ///
    /// The splitter's rule is run here as it is run there
    /// ([`crate::section_tolerance::exact_line_splits`]), over the same
    /// points, so the planar face and the partner face of each section
    /// cannot come to carry different vertices on a hole.
    ///
    /// # Errors
    ///
    /// [`AlgoError::IntersectionFailed`] when the rule would weld two of
    /// the ends, split a hole edge where no crossing was certified, or
    /// leave a certified crossing without a vertex.
    pub(super) fn certify_hole_splits(&self, kept: &[usize]) -> Result<(), AlgoError> {
        let count = self.ends.len();
        if count == 0 {
            return Ok(());
        }
        let anchors: Vec<Point3> = kept
            .iter()
            .flat_map(|&index| {
                [
                    self.ends[index % count].at,
                    self.ends[(index + 1) % count].at,
                ]
            })
            .collect();
        let certificate = self.certificate;
        let band = certificate.model.band;
        let is_anchor = |p: Point3| anchors.iter().any(|a| (*a - p).length() <= band);
        for hole in &certificate.holes {
            for (edge, segment) in hole.segments.iter().enumerate() {
                let Some((from, to)) = *segment else { continue };
                let wanted: Vec<Point3> = hole
                    .crossings()
                    .filter(|crossing| crossing.edge == Some(edge) && is_anchor(crossing.at))
                    .map(|crossing| crossing.at)
                    .collect();
                let refuse = |why: String| {
                    AlgoError::IntersectionFailed(format!(
                        "the face splitter would {why} on an edge of hole {:?} of planar face \
                         {:?}, which the section's kept spans end on at {} of its crossings; \
                         the hole and the sections ending on it would disagree",
                        hole.wire,
                        certificate.face,
                        wanted.len()
                    ))
                };
                let splits = crate::section_tolerance::exact_line_splits(
                    from,
                    to,
                    &anchors,
                    certificate.linear,
                )
                .map_err(|close| {
                    refuse(format!(
                        "weld the vertices {:?} and {:?}, closer than {:e}",
                        close.first, close.second, close.weld
                    ))
                })?;
                let agree = splits.len() == wanted.len()
                    && splits
                        .iter()
                        .all(|(_, at)| wanted.iter().any(|p| (*p - *at).length() <= band));
                if !agree {
                    return Err(refuse(format!("put {} vertices", splits.len())));
                }
            }
        }
        Ok(())
    }

    /// The first reason a span kept beside one of the face's hole crossings
    /// would be refused, whichever span that is.
    #[cfg(test)]
    pub(super) fn first_refusal(&self) -> Option<AlgoError> {
        self.ends
            .iter()
            .find_map(|end| end.taint.clone())
            .or_else(|| self.strays.first().map(|(_, why)| why.clone()))
            .map(AlgoError::IntersectionFailed)
    }
}

#[cfg(test)]
mod tests;
