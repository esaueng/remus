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
//! Every point of the section within the band of a loop edge lies in a
//! *zone*: an interval of the section's parameter, found in closed form (the
//! distance from a line and the position along it are sinusoids of the
//! parameter; so is the squared distance from a circle's centre). The zones
//! of one loop, merged, leave *gaps* between them, and along a gap the
//! section stays farther than the band from the loop, so it lies on one
//! side of it throughout: one probe decides the gap.
//!
//! A zone whose two neighbouring gaps lie on different sides is a crossing.
//! Its vertex is the one exact root of the section with a loop edge found
//! in the zone, or failing that the one loop vertex the section passes
//! within the band of there (a crossing through a corner). A zone with
//! neither is a crossing that cannot be placed: it is kept as a split point
//! and marked, and refuses any span kept beside it.
//!
//! A zone whose neighbouring gaps lie on the same side is a touch. By
//! construction the section stays within the band of the loop throughout
//! it, so it does not cross the loop by more than the tolerance there.
//!
//! The number of crossings of a loop is therefore even whenever every gap
//! could be decided, and consecutive crossings alternate; both are checked
//! again on the result ([`LoopCertificate::alternates`]).
//!
//! # What is left to the collectors
//!
//! Spans with no end on a hole crossing are decided as they were before the
//! holes were read (the planar face's outline and the partner face's
//! boundary split the section the way they always did): a section whose
//! hole crossings all lie on spans the partner face does not reach is not
//! touched by this module. See [`SpanPlan::keep`].

use std::f64::consts::{PI, TAU};

use remus_math::aabb::Aabb3;
use remus_math::compact_sort::CompactSort;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Point3, Vec2};
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
/// [`crate::builder::classify_2d::point_in_planar_loop`], a point on an
/// arc's circle away from the arc is decided.
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
                    let offset = (p - center).length();
                    if (offset - radius).abs() <= eps {
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
}

/// The roots of the section with the straight edge `from → to` that lie on
/// the edge (to within the band past its ends): the distance along the
/// edge and the point.
///
/// The chord's half-length is `√(reach · (2h − reach))` over the support
/// distance `h` and the line's speed in the chart that maps the section to
/// the unit circle: `1 − (chart distance)²` written without cancellation,
/// so a shallow crossing keeps its accuracy.
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
        .map(|s| (s, from + dir * s.clamp(0.0, length)))
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

/// Where a circular section lies within `band` of the arc of `rim` centred
/// on angle `mid` (a direction from the rim's centre) and `half` wide either
/// side of it; `half ≥ π` is the whole rim.
fn arc_stretch(
    conic: &SectionConic,
    rim_center: Point3,
    rim_radius: f64,
    wedge: Option<(remus_math::vec::Vec3, f64)>,
    band: f64,
) -> (Vec<Span>, [f64; 2]) {
    let [(u, r), (v, _)] = conic.axes;
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
    let Some((mid, half)) = wedge else {
        return (annulus, tangencies);
    };
    if half >= PI {
        return (annulus, tangencies);
    }
    // A point of the annulus lies within the arc's angle of `mid` exactly
    // when its component along `mid` is at least `radius · cos(half)`; the
    // arc's ends are moved out by the band along the rim and across it.
    let threshold = rim_radius * (half + band / rim_radius).min(PI).cos() - band;
    let (mu, mv) = (r * mid.dot(u), r * mid.dot(v));
    let within = sinusoid_band(
        mu.hypot(mv),
        mv.atan2(mu),
        dc.dot(mid),
        threshold,
        f64::INFINITY,
    );
    (intersect(&annulus, &within), tangencies)
}

/// Read one loop of a planar face against the section: `None` for a loop
/// with an edge that has no exact contact with it here (an ellipse, a
/// spline or an open conic; a circular edge under an elliptical section,
/// which is a quartic; an arc without a stored parameter range).
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
                for (_, root) in line_roots(conic, from, to, model.band) {
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
                        t: conic.angle(root).rem_euclid(TAU),
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
                let wedge = (!edge.is_closed()).then(|| {
                    let mid = rim.evaluate(0.5 * (t0 + t1)) - rim.center();
                    (mid * (1.0 / rim.radius()), 0.5 * (t1 - t0).abs())
                });
                let (spans, tangencies) =
                    arc_stretch(conic, rim.center(), rim.radius(), wedge, model.band);
                read.tangencies.push(tangencies);
                read.stretches
                    .extend(spans.into_iter().map(|span| Stretch { span, edge: index }));
                for (at, _) in
                    circle_arc_crossings(edge, oe.edge(), sp, ep, rim, circle, tol).ok()?
                {
                    let vertex = if edge.is_closed() {
                        ((at - from).length() <= model.band).then_some(index)
                    } else {
                        vertex_at(at)
                    };
                    read.roots.push(Root {
                        t: conic.angle(at).rem_euclid(TAU),
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
    /// The holes that could not be read, with their bounding boxes.
    unread_holes: Vec<(WireId, Aabb3)>,
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
                None => unread_holes.push((
                    wire,
                    bounds.unwrap_or(Aabb3 {
                        min: Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
                        max: Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
                    }),
                )),
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
        let mut ends: Vec<SpanEnd> = splits
            .iter()
            .map(|&(_, at)| {
                let crossing = self
                    .hole_crossings()
                    .find(|(_, crossing)| (crossing.at - at).length() <= model.band);
                SpanEnd {
                    at,
                    on_hole: crossing.is_some(),
                    taint: crossing.and_then(|(_, crossing)| crossing.taint.clone()),
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
        let strays = self
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
        if let Some(wire) = self.unread_hole_near(t0, t1) {
            return Err(format!(
                "hole {wire:?} of planar face {:?} has an edge with no exact contact with the \
                 section, and lies within reach of a span it would keep",
                self.face
            ));
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
            if loop_side(probe, &certificate.outline, 0.5 * self.model.band) != Some(side) {
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

impl PlaneSectionCertificate {
    /// A hole that could not be read whose bounding box the span `(t0, t1)`
    /// of the section comes within the band of.
    ///
    /// The span's box is that of seventeen of its points, widened by the
    /// longest step between them: a conic arc lies within its chord's
    /// length of the chord.
    fn unread_hole_near(&self, t0: f64, t1: f64) -> Option<WireId> {
        const STEPS: u32 = 16;
        if self.unread_holes.is_empty() {
            return None;
        }
        let points: Vec<Point3> = (0..=STEPS)
            .map(|k| {
                self.conic
                    .point((t1 - t0).mul_add(f64::from(k) / f64::from(STEPS), t0))
            })
            .collect();
        let step = points
            .windows(2)
            .map(|pair| (pair[1] - pair[0]).length())
            .fold(0.0, f64::max);
        let span = Aabb3::try_from_points(points.iter().copied())?.expanded(step + self.model.band);
        self.unread_holes
            .iter()
            .find(|(_, hole)| hole.intersects(span))
            .map(|(wire, _)| *wire)
    }
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
        let reach = remus_math::vec::Vec3::new(reach, reach, reach);
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
    /// Hole crossings that are not split points: the parameter and the
    /// refusal for a span running over one.
    strays: Vec<(f64, String)>,
}

/// What the gate decided for one span of a section.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Kept {
    /// Not emitted.
    No,
    /// Emitted as it always was: the span ends on no hole, and the
    /// collector's own test is all that decides it.
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
    /// fails it is dropped, as it always was. A span that passes it and has
    /// no end on a hole is kept, as it always was: without the hole's
    /// crossings it is the span the collector emitted before holes were
    /// read. A span that passes it and ends on a hole is new, and is kept
    /// only when it is certified on the face.
    ///
    /// # Errors
    ///
    /// [`AlgoError::IntersectionFailed`] when a span that ends on a hole
    /// passes `elsewhere` but cannot be certified on the face or off it.
    pub(super) fn keep(
        &self,
        index: usize,
        (t0, t1): (f64, f64),
        elsewhere: bool,
    ) -> Result<Kept, AlgoError> {
        if !elsewhere {
            return Ok(Kept::No);
        }
        let count = self.ends.len();
        if count == 0 {
            return Ok(Kept::AsBefore);
        }
        let ends = [&self.ends[index % count], &self.ends[(index + 1) % count]];
        let stray = self
            .strays
            .iter()
            .find(|(t, _)| (t - t0).rem_euclid(TAU) < t1 - t0);
        if stray.is_none() && ends.iter().all(|end| !end.on_hole) {
            return Ok(Kept::AsBefore);
        }
        let refuse = |why: &str| AlgoError::IntersectionFailed(why.to_owned());
        if let Some((_, why)) = stray {
            return Err(refuse(why));
        }
        if let Some(why) = ends.iter().find_map(|end| end.taint.as_deref()) {
            return Err(refuse(why));
        }
        self.certificate
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
