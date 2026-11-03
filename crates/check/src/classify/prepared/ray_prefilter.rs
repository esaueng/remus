//! Knot-span boxes of NURBS faces and the ray prefilter they drive.
//!
//! The ray-NURBS solver behind a prepared crossing count scans a seed grid
//! and refines Newton seeds for every NURBS face whose support box the BVH
//! offers. A forward ray that stays clear of every knot-span box of the face
//! gets no hit from it, so [`PreparedSolid::count_ray_crossings`] skips the
//! face without running the solver.
//!
//! The scan must stay cheaper than the solver's 400-node seed scan it saves.
//! Past a few spans it tests runs of consecutive span boxes behind the box
//! holding each run: about `2 sqrt(n)` box tests rather than `n` for a ray
//! that meets few runs. A face with more than [`MAX_PREFILTER_SPANS`] spans,
//! or a single one, is left to the solver.
//!
//! On a face most rays hit, the scan only adds to the solver it cannot save.
//! Once a face has tested [`PAYOFF_WARMUP`] rays and skipped fewer than one
//! in [`PAYOFF_RATIO`], it tests only one ray in [`PAYOFF_RATIO`] and leaves
//! the others to the solver, until its skips catch up again. Which rays are
//! skipped then depends on the rays before them; every skip is exact either
//! way.

use std::cmp::Ordering;
use std::ops::Range;
use std::sync::atomic::AtomicU32;
use std::sync::atomic::Ordering::Relaxed;

use remus_math::aabb::Aabb3;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::face::FaceSurface;

use super::{PreparedFace, PreparedSolid};

/// Relative inflation of every span box and of the ray-box test.
const BOX_ROUNDING: f64 = 1.0e-12;

/// Most non-empty knot spans of a face the prefilter takes, the solver's
/// seed-grid size. A ray that meets many groups of a denser face could cost
/// more box tests than the seed scan the prefilter can save.
const MAX_PREFILTER_SPANS: usize = 400;

/// Faces with at most this many spans scan their span boxes without groups.
const UNGROUPED_SPANS: usize = 8;

/// Rays a face tests before its skip rate can thin the testing.
const PAYOFF_WARMUP: u32 = 64;

/// A face that skips fewer than one in this many of the rays it tests goes on
/// to test only one ray in this many: the solver runs it saves cost far more
/// than one scan each, so below this rate the scans cost about what the skips
/// save.
const PAYOFF_RATIO: u32 = 32;

fn point_norm(p: Point3) -> f64 {
    Vec3::new(p.x(), p.y(), p.z()).length()
}

/// Grow a box by its rounding allowance.
fn inflate(aabb: Aabb3) -> Aabb3 {
    let scale = [
        aabb.min.x(),
        aabb.min.y(),
        aabb.min.z(),
        aabb.max.x(),
        aabb.max.y(),
        aabb.max.z(),
    ]
    .iter()
    .fold(1.0_f64, |m, c| m.max(c.abs()));
    aabb.expanded(BOX_ROUNDING * scale)
}

/// Control-point boxes of a NURBS support's knot spans.
///
/// On the span `[u_k, u_k+1] x [v_l, v_l+1]` only the control points
/// `P[k-p..=k][l-q..=l]` have non-zero basis functions, and those are
/// non-negative and sum to one. With positive weights each surface point is
/// therefore a convex combination of exactly those points, so their box
/// contains the span's whole image (the local convex-hull property). Any
/// non-positive or non-finite weight refuses, and so does a ragged control
/// net. So does a knot vector that decreases or whose first or last span is
/// empty: evaluation clamps a parameter at the domain ends into those spans,
/// which must then be boxed.
pub(super) fn nurbs_span_boxes(surface: &NurbsSurface) -> Option<Vec<Aabb3>> {
    if !surface
        .weights()
        .iter()
        .flatten()
        .all(|&w| w.is_finite() && w > 0.0)
    {
        return None;
    }
    let (p, q) = (surface.degree_u(), surface.degree_v());
    let (knots_u, knots_v) = (surface.knots_u(), surface.knots_v());
    let points = surface.control_points();
    let (rows, cols) = (points.len(), points.first().map_or(0, Vec::len));
    if rows <= p
        || cols <= q
        || knots_u.len() != rows + p + 1
        || knots_v.len() != cols + q + 1
        || points.iter().any(|row| row.len() != cols)
    {
        return None;
    }
    let usable = |knots: &[f64], degree: usize, count: usize| {
        knots.windows(2).all(|pair| pair[0] <= pair[1])
            && knots[degree] < knots[degree + 1]
            && knots[count - 1] < knots[count]
    };
    if !usable(knots_u, p, rows) || !usable(knots_v, q, cols) {
        return None;
    }
    let mut boxes = Vec::new();
    for k in p..rows {
        if knots_u[k].partial_cmp(&knots_u[k + 1]) != Some(Ordering::Less) {
            continue;
        }
        for l in q..cols {
            if knots_v[l].partial_cmp(&knots_v[l + 1]) != Some(Ordering::Less) {
                continue;
            }
            let span = (k - p..=k).flat_map(|i| points[i][l - q..=l].iter().copied());
            let aabb = Aabb3::try_from_points(span)?;
            if ![aabb.min, aabb.max]
                .iter()
                .all(|c| c.x().is_finite() && c.y().is_finite() && c.z().is_finite())
            {
                return None;
            }
            boxes.push(inflate(aabb));
        }
    }
    (!boxes.is_empty()).then_some(boxes)
}

/// Whether the prefilter takes a NURBS support: one with between 2 and
/// [`MAX_PREFILTER_SPANS`] non-empty knot spans. The box of a single span is
/// the box of the whole control net, which is the face's bound in the BVH
/// (the surface samples lie inside it), so a ray the BVH offers practically
/// never misses it.
pub(super) fn takes(surface: &NurbsSurface) -> bool {
    let spans = |knots: &[f64]| knots.windows(2).filter(|pair| pair[0] < pair[1]).count();
    let count = spans(surface.knots_u()).saturating_mul(spans(surface.knots_v()));
    (2..=MAX_PREFILTER_SPANS).contains(&count)
}

/// A box the prefilter tests, with the norms of its corners.
#[derive(Debug, Clone, Copy)]
struct ScanBox {
    aabb: Aabb3,
    min_norm: f64,
    max_norm: f64,
}

impl ScanBox {
    fn new(aabb: Aabb3) -> Self {
        Self {
            aabb,
            min_norm: point_norm(aabb.min),
            max_norm: point_norm(aabb.max),
        }
    }

    /// Whether the forward ray `origin + t direction`, `t >= 0`, comes within
    /// `reach` of the box (slab test against the box grown by `reach` plus the
    /// rounding allowance `BOX_ROUNDING (1 + |origin| + |min| + |max|)`, the
    /// ray's part `1 + |origin|` given as `origin_scale`). A non-finite input
    /// never reads as a miss.
    fn meets(&self, origin: Point3, direction: Vec3, origin_scale: f64, reach: f64) -> bool {
        let scale = origin_scale + self.min_norm + self.max_norm;
        ray_meets_grown_box(origin, direction, &self.aabb, reach + BOX_ROUNDING * scale)
    }
}

/// The span boxes of one NURBS face as the prefilter scans them.
#[derive(Debug)]
pub(super) struct SpanBoxes {
    boxes: Vec<ScanBox>,
    /// Runs of consecutive `boxes`, each behind the box that holds it; empty
    /// when the boxes are scanned on their own.
    groups: Vec<(ScanBox, Range<usize>)>,
    /// Largest absolute control-point coordinate, which sets how far from a
    /// line the solver can report a hit.
    magnitude: f64,
    /// Rays tested and skipped so far, for the payoff test, and rays seen
    /// while it failed.
    tested: AtomicU32,
    skipped: AtomicU32,
    idle: AtomicU32,
}

impl Clone for SpanBoxes {
    fn clone(&self) -> Self {
        Self {
            boxes: self.boxes.clone(),
            groups: self.groups.clone(),
            magnitude: self.magnitude,
            tested: AtomicU32::new(self.tested.load(Relaxed)),
            skipped: AtomicU32::new(self.skipped.load(Relaxed)),
            idle: AtomicU32::new(self.idle.load(Relaxed)),
        }
    }
}

impl SpanBoxes {
    /// The prefilter's boxes for a NURBS support: `None` when it does not
    /// take the support ([`takes`]) or its span boxes cannot be certified
    /// ([`nurbs_span_boxes`]).
    ///
    /// A group's box holds every box of its run, so a ray that comes within
    /// reach of one of them comes within reach of the group's box, which its
    /// test then reports (the rounding allowance is the group box's own).
    pub(super) fn for_surface(surface: &NurbsSurface) -> Option<Self> {
        if !takes(surface) {
            return None;
        }
        let boxes: Vec<ScanBox> = nurbs_span_boxes(surface)?
            .into_iter()
            .map(ScanBox::new)
            .collect();
        let mut groups = Vec::new();
        if boxes.len() > UNGROUPED_SPANS {
            let size = boxes.len().isqrt();
            for start in (0..boxes.len()).step_by(size) {
                let run = start..(start + size).min(boxes.len());
                let held = boxes[run.clone()]
                    .iter()
                    .fold(boxes[start].aabb, |held, b| Aabb3 {
                        min: Point3::new(
                            held.min.x().min(b.aabb.min.x()),
                            held.min.y().min(b.aabb.min.y()),
                            held.min.z().min(b.aabb.min.z()),
                        ),
                        max: Point3::new(
                            held.max.x().max(b.aabb.max.x()),
                            held.max.y().max(b.aabb.max.y()),
                            held.max.z().max(b.aabb.max.z()),
                        ),
                    });
                groups.push((ScanBox::new(held), run));
            }
        }
        let magnitude = surface
            .control_points()
            .iter()
            .flatten()
            .flat_map(|p| p.0)
            .fold(0.0_f64, |m, c| m.max(c.abs()));
        Some(Self {
            boxes,
            groups,
            magnitude,
            tested: AtomicU32::new(0),
            skipped: AtomicU32::new(0),
            idle: AtomicU32::new(0),
        })
    }

    /// Whether the payoff test passes: through the warm-up, and after it
    /// while the face skips at least one ray in [`PAYOFF_RATIO`].
    fn paying(&self) -> bool {
        let tested = self.tested.load(Relaxed);
        tested < PAYOFF_WARMUP || self.skipped.load(Relaxed).saturating_mul(PAYOFF_RATIO) >= tested
    }

    /// Whether to test the next ray: every ray while the payoff test passes,
    /// one in [`PAYOFF_RATIO`] while it fails, so a face whose rays come to
    /// miss it can pass it again. Counts lost to a concurrent update only
    /// shift which rays are tested.
    fn due(&self) -> bool {
        if self.paying() {
            return true;
        }
        let idle = self.idle.load(Relaxed).wrapping_add(1);
        self.idle.store(idle, Relaxed);
        idle.is_multiple_of(PAYOFF_RATIO)
    }

    /// Whether the forward ray meets no span box, counted toward the payoff
    /// test.
    fn test(&self, origin: Point3, direction: Vec3) -> bool {
        let skip = !self.meet(origin, direction);
        self.tested
            .store(self.tested.load(Relaxed).wrapping_add(1), Relaxed);
        if skip {
            self.skipped
                .store(self.skipped.load(Relaxed).wrapping_add(1), Relaxed);
        }
        skip
    }

    /// Whether the forward ray comes within the solver's hit reach (see
    /// [`PreparedSolid::ray_misses_nurbs_face`]) of some span box.
    fn meet(&self, origin: Point3, direction: Vec3) -> bool {
        let reach = remus_math::nurbs::intersection::line_hit_reach(self.magnitude, origin);
        self.meet_within(origin, direction, reach)
    }

    /// Whether the forward ray comes within `reach` of some span box (see
    /// [`ScanBox::meets`]).
    fn meet_within(&self, origin: Point3, direction: Vec3, reach: f64) -> bool {
        let origin_scale = 1.0 + point_norm(origin);
        let meets = |b: &ScanBox| b.meets(origin, direction, origin_scale, reach);
        if self.groups.is_empty() {
            return self.boxes.iter().any(meets);
        }
        self.groups
            .iter()
            .any(|(group, run)| meets(group) && self.boxes[run.clone()].iter().any(meets))
    }
}

/// Slab test of the forward ray against `aabb` grown by `grow`.
///
/// The interval updates compare rather than call `f64::max` and `f64::min`:
/// both keep the bound when the candidate is NaN, as those do.
fn ray_meets_grown_box(origin: Point3, direction: Vec3, aabb: &Aabb3, grow: f64) -> bool {
    let (mut t0, mut t1) = (0.0_f64, f64::INFINITY);
    for k in 0..3 {
        let o = origin.0[k];
        let d = direction.0[k];
        let (lo, hi) = (aabb.min.0[k] - grow, aabb.max.0[k] + grow);
        if d == 0.0 {
            if o < lo || o > hi {
                return false;
            }
            continue;
        }
        let (mut a, mut b) = ((lo - o) / d, (hi - o) / d);
        if a > b {
            std::mem::swap(&mut a, &mut b);
        }
        if a > t0 {
            t0 = a;
        }
        if b < t1 {
            t1 = b;
        }
        if t0 > t1 {
            return false;
        }
    }
    true
}

impl PreparedSolid<'_> {
    /// Whether a forward ray provably meets no point of a NURBS face's
    /// support that the ray-NURBS solver could report.
    ///
    /// Every hit `intersect_line_nurbs_with_grid` reports is a surface
    /// evaluation at an in-domain parameter (its seeds and Newton iterates are
    /// clamped to the domain) lying within `line_hit_reach` of the line (the
    /// seed-grid pass's `1e-5`, plus the completion pass's round-off
    /// allowance, which grows with the control points' coordinates), and
    /// every such evaluation lies in the knot-span boxes (local convex hulls,
    /// see [`nurbs_span_boxes`]). A hit is kept only ahead of the origin,
    /// where the line's closest point to it lies on the forward ray. So a ray
    /// that stays farther than that reach from every span box, for `t >= 0`,
    /// gets no hits and contributes no crossing, and the solver need not run.
    /// Only faces with a cached trim qualify, and only for a direction the
    /// solver accepts: their crossing count could not have failed, so
    /// skipping one hides no error. A ray the payoff test leaves untested
    /// (see [`PAYOFF_RATIO`]) is not skipped.
    pub(super) fn ray_misses_nurbs_face(
        &self,
        prepared: &PreparedFace,
        origin: Point3,
        direction: Vec3,
    ) -> bool {
        if !prepared.prefilter {
            return false;
        }
        let boxes = prepared.span_boxes.get_or_init(|| {
            match self
                .topo
                .face(prepared.fid)
                .map(remus_topology::face::Face::surface)
            {
                Ok(FaceSurface::Nurbs(surface)) => SpanBoxes::for_surface(surface),
                Ok(
                    FaceSurface::Plane { .. }
                    | FaceSurface::Cylinder(_)
                    | FaceSurface::Cone(_)
                    | FaceSurface::Sphere(_)
                    | FaceSurface::Torus(_),
                )
                | Err(_) => None,
            }
        });
        let Some(boxes) = boxes.as_ref().filter(|boxes| boxes.due()) else {
            return false;
        };
        let usable = matches!(
            direction.dot(direction).partial_cmp(&1e-20),
            Some(Ordering::Greater | Ordering::Equal)
        );
        usable && boxes.test(origin, direction)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;
    use remus_math::surfaces::{CylindricalSurface, SphericalSurface, ToroidalSurface};

    fn ray_meets_box(origin: Point3, direction: Vec3, aabb: &Aabb3, reach: f64) -> bool {
        ScanBox::new(*aabb).meets(origin, direction, 1.0 + point_norm(origin), reach)
    }

    /// A bicubic dome over `[0, 20]^2` with `k x k` uniform spans.
    fn dome(k: usize) -> NurbsSurface {
        #![allow(clippy::cast_precision_loss)]
        let n = k + 3;
        let mut knots = vec![0.0; 4];
        knots.extend((1..k).map(|i| i as f64 / k as f64));
        knots.extend([1.0; 4]);
        let control = (0..n)
            .map(|i| {
                (0..n)
                    .map(|j| {
                        let (s, t) = (i as f64 / (n - 1) as f64, j as f64 / (n - 1) as f64);
                        let bump =
                            (std::f64::consts::PI * s).sin() * (std::f64::consts::PI * t).sin();
                        Point3::new(20.0 * s, 20.0 * t, 6.0 * bump)
                    })
                    .collect()
            })
            .collect();
        NurbsSurface::new(3, 3, knots.clone(), knots, control, vec![vec![1.0; n]; n]).unwrap()
    }

    #[test]
    fn faces_of_one_span_or_past_the_cap_are_left_to_the_solver() {
        let at_cap = SpanBoxes::for_surface(&dome(20)).unwrap();
        assert_eq!(at_cap.boxes.len(), MAX_PREFILTER_SPANS);
        assert!(SpanBoxes::for_surface(&dome(21)).is_none());
        assert!(SpanBoxes::for_surface(&dome(1)).is_none());
        let small = SpanBoxes::for_surface(&dome(2)).unwrap();
        assert!(small.boxes.len() <= UNGROUPED_SPANS && small.groups.is_empty());
    }

    /// One ray through the payoff test: `Some(skipped)` when it is tested.
    fn gate(boxes: &SpanBoxes, origin: Point3, direction: Vec3) -> Option<bool> {
        boxes.due().then(|| boxes.test(origin, direction))
    }

    /// A face whose rays all meet a span box fails the payoff test after the
    /// warm-up and then tests one ray in 32, misses included, until enough
    /// skips pass it again; one that skips one ray in 16 keeps passing.
    #[test]
    fn faces_test_fewer_rays_where_skips_do_not_pay() {
        let (below, up) = (Point3::new(10.0, 10.0, -5.0), Vec3::new(0.0, 0.0, 1.0));
        let (above, across) = (Point3::new(-5.0, 10.0, 30.0), Vec3::new(1.0, 0.0, 0.0));
        let boxes = SpanBoxes::for_surface(&dome(20)).unwrap();
        assert!(!boxes.meet(above, across));
        let tested = (0..PAYOFF_WARMUP + 10 * PAYOFF_RATIO)
            .filter(|_| gate(&boxes, below, up).is_some())
            .count();
        assert_eq!(tested, 74);
        assert!(!boxes.paying());
        // 3 skips in 77 tested rays pass it again.
        let (mut rays, mut skips) = (0, 0);
        while !boxes.paying() {
            rays += 1;
            skips += usize::from(gate(&boxes, above, across) == Some(true));
        }
        assert_eq!((rays, skips), (96, 3));
        assert_eq!(gate(&boxes, above, across), Some(true));

        let mixed = SpanBoxes::for_surface(&dome(20)).unwrap();
        for i in 0..1000 {
            let miss = i % 16 == 0;
            let (origin, direction) = if miss { (above, across) } else { (below, up) };
            assert_eq!(gate(&mixed, origin, direction), Some(miss));
        }
        assert!(mixed.paying());
    }

    /// The grouped scan meets a ray exactly when one of its span boxes does,
    /// and the groups cover every box once.
    #[test]
    fn grouped_scan_agrees_with_every_span_box() {
        #![allow(clippy::cast_precision_loss)]
        let mut state = 0x9e37_79b9_7f4a_7c15_u64;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 11) as f64 / (1_u64 << 53) as f64
        };
        let (mut met, mut missed) = (0, 0);
        for k in [3, 5, 9, 14, 20] {
            let boxes = SpanBoxes::for_surface(&dome(k)).unwrap();
            assert!(!boxes.groups.is_empty());
            let covered: Vec<usize> = boxes
                .groups
                .iter()
                .flat_map(|(_, run)| run.clone())
                .collect();
            assert_eq!(covered, (0..boxes.boxes.len()).collect::<Vec<_>>());
            for _ in 0..4000 {
                let origin = Point3::new(
                    next().mul_add(30.0, -5.0),
                    next().mul_add(30.0, -5.0),
                    next().mul_add(12.0, -3.0),
                );
                let direction = Vec3::new(next() - 0.5, next() - 0.5, next() - 0.5);
                let reach = [0.0, 1e-5, 0.5][(next() * 3.0) as usize % 3];
                let flat = boxes
                    .boxes
                    .iter()
                    .any(|b| b.meets(origin, direction, 1.0 + point_norm(origin), reach));
                assert_eq!(
                    boxes.meet_within(origin, direction, reach),
                    flat,
                    "k {k} from {origin:?} along {direction:?}"
                );
                if flat {
                    met += 1;
                } else {
                    missed += 1;
                }
            }
        }
        assert!(met > 2000 && missed > 2000, "{met} met, {missed} missed");
    }

    #[test]
    fn nurbs_span_boxes_contain_the_surface() {
        let axis = Vec3::new(0.2, 0.1, 1.0).normalize().unwrap();
        let surfaces = [
            CylindricalSurface::new(Point3::new(1.0, -2.0, 0.5), axis, 1.7)
                .unwrap()
                .to_nurbs(-1.0, 2.5)
                .unwrap(),
            SphericalSurface::new(Point3::new(0.5, 0.5, -1.0), 2.0)
                .unwrap()
                .to_nurbs()
                .unwrap(),
            ToroidalSurface::new(Point3::new(0.0, 1.0, 2.0), 3.0, 0.8)
                .unwrap()
                .to_nurbs()
                .unwrap(),
        ];
        for surface in &surfaces {
            let boxes = nurbs_span_boxes(surface).unwrap();
            let (u0, u1) = surface.domain_u();
            let (v0, v1) = surface.domain_v();
            for i in 0..=60 {
                for j in 0..=60 {
                    let u = u0 + (u1 - u0) * f64::from(i) / 60.0;
                    let v = v0 + (v1 - v0) * f64::from(j) / 60.0;
                    let point = surface.evaluate(u, v);
                    assert!(
                        boxes.iter().any(|b| b.contains_point(point)),
                        "({u}, {v}) -> {point:?} outside every span box"
                    );
                }
            }
        }
    }

    /// Forward hits of the solver, as the crossing count keeps them.
    fn forward_hits(surface: &NurbsSurface, origin: Point3, direction: Vec3) -> usize {
        remus_math::nurbs::intersection::intersect_line_nurbs(surface, origin, direction, 20)
            .unwrap()
            .iter()
            .filter(|hit| (hit.point - origin).dot(direction) / direction.dot(direction) > 1e-12)
            .count()
    }

    /// The solver can report a hit wider than its seed-grid tolerance: the
    /// completion pass accepts one within its round-off allowance, which the
    /// far end of this surface (z = 1e9) sets. The ray passes 3e-5 over the
    /// flat near span, clear of every span box by more than 1e-5, and the
    /// prefilter still keeps the face.
    #[test]
    fn prefilter_keeps_a_wide_completion_hit() {
        let surface = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 0.5, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
                vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
                vec![Point3::new(2.0, 0.0, 1e9), Point3::new(2.0, 1.0, 1e9)],
            ],
            vec![vec![1.0; 2]; 3],
        )
        .unwrap();
        let origin = Point3::new(0.5, -1.0, 3e-5);
        let direction = Vec3::new(0.0, 1.0, 0.0);
        assert_eq!(forward_hits(&surface, origin, direction), 1);
        let boxes = SpanBoxes::for_surface(&surface).unwrap();
        assert!(boxes.meet(origin, direction));
        assert!(!boxes.meet_within(origin, direction, 1e-5));
    }

    /// Deterministic low-discrepancy sample in [0, 1).
    fn halton(mut i: u32, base: u32) -> f64 {
        let (mut f, mut r) = (1.0_f64, 0.0_f64);
        while i > 0 {
            f /= f64::from(base);
            r += f * f64::from(i % base);
            i /= base;
        }
        r
    }

    /// No ray the solver gets a forward hit from is skipped: rational
    /// bicubic patches from 1e-3 to 1e9 units, rays aimed at and just past
    /// surface points (off by up to three times the solver's reach), from in
    /// front of and behind them.
    #[test]
    fn prefilter_never_skips_a_ray_the_solver_hits() {
        #![allow(clippy::cast_precision_loss)]
        let (mut hit, mut skipped) = (0, 0);
        let mut sample = 0_u32;
        let mut next = |base: u32| {
            sample += 1;
            halton(sample, base)
        };
        for scale in [1e-3, 1.0, 1e3, 1e6, 1e9] {
            let n = 6;
            let mut knots = vec![0.0; 4];
            knots.extend([0.3, 0.7]);
            knots.extend([1.0; 4]);
            let control: Vec<Vec<Point3>> = (0..n)
                .map(|i| {
                    (0..n)
                        .map(|j| {
                            let bump = next(2) - 0.5;
                            Point3::new(
                                scale * (i as f64 / 5.0 + 3.0),
                                scale * (j as f64 / 5.0 - 1.0),
                                scale * 0.4 * bump,
                            )
                        })
                        .collect()
                })
                .collect();
            let weights = (0..n)
                .map(|_| (0..n).map(|_| 0.5 + 1.5 * next(3)).collect())
                .collect();
            let surface = NurbsSurface::new(3, 3, knots.clone(), knots, control, weights).unwrap();
            let boxes = SpanBoxes::for_surface(&surface).unwrap();
            for _ in 0..120 {
                let target = surface.evaluate(next(5), next(7));
                let direction = Vec3::new(next(11) - 0.5, next(13) - 0.5, next(17) - 0.5);
                let Ok(unit) = direction.normalize() else {
                    continue;
                };
                let side = unit.cross(Vec3::new(0.3, -0.7, 0.2)).normalize().unwrap();
                let reach =
                    remus_math::nurbs::intersection::line_hit_reach(boxes.magnitude, target);
                let past = target + side * (3.0 * reach * next(19));
                let origin = past - unit * (scale * (3.0 * next(23) - 1.0));
                let hits = forward_hits(&surface, origin, unit);
                let meets = boxes.meet(origin, unit);
                assert!(hits == 0 || meets, "scale {scale}: {hits} hits skipped");
                hit += usize::from(hits > 0);
                skipped += usize::from(!meets);
            }
        }
        assert!(hit > 100 && skipped > 20, "{hit} hit, {skipped} skipped");
    }

    #[test]
    fn ray_box_test_keeps_every_ray_within_reach() {
        let aabb = Aabb3 {
            min: Point3::new(0.0, 0.0, 0.0),
            max: Point3::new(1.0, 1.0, 1.0),
        };
        let x = Vec3::new(1.0, 0.0, 0.0);
        // Through, grazing within reach, starting inside, and behind.
        assert!(ray_meets_box(Point3::new(-1.0, 0.5, 0.5), x, &aabb, 0.0));
        assert!(ray_meets_box(
            Point3::new(-1.0, 1.0 + 0.5e-5, 0.5),
            x,
            &aabb,
            1e-5
        ));
        assert!(ray_meets_box(Point3::new(0.5, 0.5, 0.5), -x, &aabb, 0.0));
        assert!(!ray_meets_box(Point3::new(2.0, 0.5, 0.5), x, &aabb, 1e-5));
        assert!(!ray_meets_box(
            Point3::new(-1.0, 1.0 + 2e-5, 0.5),
            x,
            &aabb,
            1e-5
        ));
        // A non-finite origin never reads as a miss.
        assert!(ray_meets_box(
            Point3::new(f64::NAN, 0.5, 0.5),
            x,
            &aabb,
            0.0
        ));
    }
}
