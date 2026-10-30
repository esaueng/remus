//! UV boundary polygon construction and containment tests for face trimming.
//!
//! Provides the core algorithms for determining whether a ray-surface hit
//! point falls within a face's trimming boundary, using UV-space projection
//! for analytic surfaces and 3D polygon containment for surfaces with
//! pole singularities (spheres).

use remus_math::compact_sort::CompactSort;
use std::sync::OnceLock;

use smallvec::SmallVec;

use remus_math::nurbs::intersection::LineSurfaceSeedGrid;
use remus_math::nurbs::projection::{SurfaceSeedGrid, project_point_to_surface_with_grid};
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::predicates::point_in_polygon;
use remus_math::traits::ParametricSurface;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

use crate::CheckError;
use crate::classify::ray_surface;
use crate::distance::plane_trim::PlanarTrim;
use crate::util::{face_hole_polygons, face_polygon, point_in_polygon_3d};

/// Minimum positive ray parameter to count as a forward hit.
const RAY_T_MIN: f64 = 1e-12;

/// Threshold for half-space sign test (negative side rejection).
const HALF_SPACE_EPS: f64 = 1e-10;

/// Threshold for coincident vertex detection (squared distance).
const COINCIDENT_SQ: f64 = 1e-12;

/// Unwrap a step in a periodic coordinate so the difference lies in
/// `[-period/2, period/2)`.
///
/// Given the previous unwrapped value `prev` and the next raw value `next`,
/// returns the next value adjusted so the step is continuous. `period` is the
/// coordinate's actual period: `2*PI` for the analytic surfaces' angular `u`,
/// the knot span for a NURBS direction that closes.
#[inline]
pub fn unwrap_periodic(prev: f64, next: f64, period: f64) -> f64 {
    let half = period * 0.5;
    let diff = next - prev;
    prev + diff - period * ((diff + half) / period).floor()
}

/// Build a UV boundary polygon from 3D face boundary vertices,
/// with proper unwrapping of periodic coordinates.
///
/// `u_period` / `v_period`: that direction's period, or `None` when the
/// direction does not close. The analytic surfaces are angular in u (`2*PI`)
/// and, for the torus alone, in v. A NURBS parameter is a knot value with no
/// inherent period: it gets `Some(knot span)` only when the control grid
/// actually closes in that direction, and `None` otherwise.
///
/// `u_pole`: the `v` at which the surface collapses to a point and `u` is
/// undefined (a cone's apex, `v = 0`), or `None`. See [`insert_pole_segments`].
fn build_uv_boundary<F>(
    verts: &[Point3],
    project: &F,
    u_period: Option<f64>,
    v_period: Option<f64>,
    u_pole: Option<f64>,
) -> Vec<(f64, f64)>
where
    F: Fn(Point3) -> (f64, f64),
{
    let mut uv: Vec<(f64, f64)> = verts.iter().map(|&p| project(p)).collect();

    if let Some(pole_v) = u_pole
        && let Some(with_poles) = insert_pole_segments(&uv, pole_v, u_period)
    {
        return with_poles;
    }

    for i in 1..uv.len() {
        if let Some(period) = u_period {
            uv[i].0 = unwrap_periodic(uv[i - 1].0, uv[i].0, period);
        }
        if let Some(period) = v_period {
            uv[i].1 = unwrap_periodic(uv[i - 1].1, uv[i].1, period);
        }
    }

    uv
}

/// `ConicalSurface::project_point` measures `v` along the generator from the
/// apex, so the apex — where `u` is undefined — sits at `v = 0`.
const CONE_APEX_V: f64 = 0.0;

/// Samples within this fraction of the loop's `v` extent of the pole are the
/// pole itself; relative so it holds at any model scale.
const POLE_REL_EPS: f64 = 1e-9;

/// Unwrap a loop that visits a surface pole, where `u` collapses.
///
/// A sample at the pole projects to an arbitrary `u` (`atan2(0, 0)`), so
/// unwrapping straight through it bends the loop toward that meaningless
/// angle. On `make_cone(3, 0, 3)`'s lateral face, whose wire is base circle,
/// seam up to the apex and seam back, the apex projected to `u = 0` while the
/// seam sits at `u = 3pi/2`: the "rectangle" became a triangle, and half the
/// wall at mid-height, `(0, 1.5, 1.5)` included, tested outside its own trim.
///
/// In `(u, v)` the pole is a segment along `v = pole_v`, not a point. Each pole
/// visit becomes two samples, one at the arriving neighbour's `u` and one at
/// the departing neighbour's. A loop that winds once around the axis (the full
/// cone) unwraps with a net period shift; the first pole segment absorbs it,
/// spanning the whole turn so the polygon closes on itself.
///
/// Returns `None` when no sample is at the pole (or every sample is), leaving
/// the caller's ordinary unwrapping in charge.
pub fn insert_pole_segments(
    uv: &[(f64, f64)],
    pole_v: f64,
    u_period: Option<f64>,
) -> Option<Vec<(f64, f64)>> {
    let extent = uv
        .iter()
        .map(|(_, v)| (v - pole_v).abs())
        .fold(0.0_f64, f64::max);
    let at_pole = |v: f64| (v - pole_v).abs() <= POLE_REL_EPS * extent;
    let start = uv.iter().position(|&(_, v)| !at_pole(v))?;
    if !uv.iter().any(|&(_, v)| at_pole(v)) {
        return None;
    }
    let unwrap = |prev: f64, next: f64| u_period.map_or(next, |p| unwrap_periodic(prev, next, p));

    // Start at a regular sample so every pole visit has an arriving `u`.
    let mut out: Vec<(f64, f64)> = Vec::with_capacity(uv.len() + 2);
    let mut first_departure: Option<usize> = None;
    let mut pending_departure = false;
    for k in 0..uv.len() {
        let (u, v) = uv[(start + k) % uv.len()];
        let prev_u = out.last().map_or(u, |&(pu, _)| pu);
        if at_pole(v) {
            if !pending_departure {
                out.push((prev_u, pole_v));
                pending_departure = true;
            }
            continue;
        }
        let u = unwrap(prev_u, u);
        if pending_departure {
            first_departure.get_or_insert(out.len());
            out.push((u, pole_v));
            pending_departure = false;
        }
        out.push((u, v));
    }
    if pending_departure {
        // The loop ends at the pole and departs toward its first sample.
        let prev_u = out.last().map_or(0.0, |&(pu, _)| pu);
        first_departure.get_or_insert(out.len());
        out.push((unwrap(prev_u, out[0].0), pole_v));
    }

    if let (Some(period), Some(from)) = (u_period, first_departure) {
        let closing = unwrap(out[out.len() - 1].0, out[0].0);
        let drift = ((closing - out[0].0) / period).round() * period;
        for sample in &mut out[from..] {
            sample.0 -= drift;
        }
    }
    Some(out)
}

/// Twice the signed area of a UV polygon (the shoelace sum).
pub fn uv_polygon_double_area(poly: &[(f64, f64)]) -> f64 {
    let n = poly.len();
    if n < 3 {
        return 0.0;
    }
    let mut acc = 0.0;
    let mut j = n - 1;
    for i in 0..n {
        acc += (poly[j].0 - poly[i].0) * (poly[j].1 + poly[i].1);
        j = i;
    }
    acc
}

/// True when a UV boundary encloses no region, so no containment test can use
/// it.
///
/// A face's boundary usually bounds a patch of its surface. Sometimes it merely
/// SPLITS it: a sphere hemisphere's entire boundary is the equator, which maps
/// to one constant `v` sweeping the whole `u` period. Measured on a converted
/// sphere, the trim polygon's v span is `0.000000` and its area is zero, so
/// every point-in-polygon test answers "outside" and the face contributes no
/// crossing for any ray.
///
/// Judged against the polygon's own extent so it holds at any parameter scale.
pub fn uv_boundary_is_degenerate(poly: &[(f64, f64)]) -> bool {
    if poly.len() < 3 {
        return true;
    }
    let (mut u_lo, mut u_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut v_lo, mut v_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for (u, v) in poly {
        u_lo = u_lo.min(*u);
        u_hi = u_hi.max(*u);
        v_lo = v_lo.min(*v);
        v_hi = v_hi.max(*v);
    }
    let scale = (u_hi - u_lo).max(v_hi - v_lo);
    if scale <= 0.0 {
        return true;
    }
    uv_polygon_double_area(poly).abs() <= 1e-9 * scale * scale
}

/// Test if a (u,v) point is inside the UV boundary polygon.
///
/// Adjusts the test point's u coordinate (and v when periodic) to lie within
/// the unwrapped polygon's coordinate range before testing.
fn point_in_uv_boundary(
    hit_u: f64,
    hit_v: f64,
    uv_boundary: &[(f64, f64)],
    u_period: Option<f64>,
    v_period: Option<f64>,
) -> bool {
    let u_min = uv_boundary
        .iter()
        .map(|(u, _)| *u)
        .fold(f64::INFINITY, f64::min);
    let u_max = uv_boundary
        .iter()
        .map(|(u, _)| *u)
        .fold(f64::NEG_INFINITY, f64::max);
    let u_center = (u_min + u_max) * 0.5;

    // Shift hit_u to be closest to the polygon's u center.
    let hu = u_period.map_or(hit_u, |p| unwrap_periodic(u_center, hit_u, p));

    // For surfaces that also close in v (torus), shift hit_v the same way.
    let hv = v_period.map_or(hit_v, |period| {
        let v_min = uv_boundary
            .iter()
            .map(|(_, v)| *v)
            .fold(f64::INFINITY, f64::min);
        let v_max = uv_boundary
            .iter()
            .map(|(_, v)| *v)
            .fold(f64::NEG_INFINITY, f64::max);
        let v_center = (v_min + v_max) * 0.5;
        unwrap_periodic(v_center, hit_v, period)
    });

    let poly: Vec<Point2> = uv_boundary
        .iter()
        .map(|(u, v)| Point2::new(*u, *v))
        .collect();
    let test = Point2::new(hu, hv);
    point_in_polygon(test, &poly)
}

/// Crossings closer together than this along a ray, in parameter units, are
/// one crossing seen twice: a sampled vertex both its segments reach, or the
/// two traversals of a doubled seam.
const CROSSING_GROUP_EPS: f64 = 1e-9;

/// The closing step of a loop from its last sample back to its first,
/// unwrapped on each periodic axis the same way `build_uv_boundary` unwraps
/// the steps between samples.
fn closing_point(
    uv_loop: &[(f64, f64)],
    u_period: Option<f64>,
    v_period: Option<f64>,
) -> Option<(f64, f64)> {
    let (&(lu, lv), &(fu, fv)) = (uv_loop.last()?, uv_loop.first()?);
    Some((
        u_period.map_or(fu, |p| unwrap_periodic(lu, fu, p)),
        v_period.map_or(fv, |p| unwrap_periodic(lv, fv, p)),
    ))
}

/// True when a closed UV loop winds around a periodic axis instead of
/// closing on itself.
///
/// Unwrapped sample by sample, a contractible loop comes back to where it
/// started; one that runs once around the torus tube (or the cylinder's
/// axis) comes back shifted by a whole period. Such a loop encloses no patch
/// of the plane — it separates the surface into bands — so no
/// point-in-polygon test can say which side belongs to the face.
fn uv_loop_wraps(uv_loop: &[(f64, f64)], u_period: Option<f64>, v_period: Option<f64>) -> bool {
    let Some(&(fu, fv)) = uv_loop.first() else {
        return false;
    };
    let Some((cu, cv)) = closing_point(uv_loop, u_period, v_period) else {
        return false;
    };
    let turns =
        |shift: f64, period: Option<f64>| period.is_some_and(|p| (shift / p).round().abs() >= 1.0);
    turns(cu - fu, u_period) || turns(cv - fv, v_period)
}

/// Decide whether `(hit_u, hit_v)` lies in the face bounded by `loops`, from
/// the ORIENTATION of the nearest boundary crossing rather than from parity.
///
/// Built for faces whose boundary loops wrap a periodic axis: the band a
/// fuse leaves on a torus between the two loops where a tool pierces the tube
/// (`regress_torus_pierce_band.rs`), or the band between a tool's cap oval
/// and its composite wall-and-cap loop. Each such loop, unwrapped, is an open
/// curve shifted by a period, so the polygon it makes is degenerate or
/// arbitrary, and parity cannot say which of the two bands it separates is
/// the face. Measured on the B45 fused band, every hit on the torus face was
/// rejected and 12 % of grid points near it classified wrongly.
///
/// The loops are read as segments on the periodic domain. A ray leaves the
/// hit along `+u` (once around `u` when `u` closes); at its nearest crossing
/// the boundary is traversed with the face on its LEFT in the surface's own
/// `(u, v)` frame, so a boundary running toward `+v` there has the hit on its
/// face side. That is the convention the face's wires already follow — the
/// torus band volume in `properties::face_integrator` selects the band the
/// same way — and it holds for reversed faces too, because a boolean flips a
/// face's normal flag, not its wires' traversal. When no loop crosses the
/// `u` ray (loops that wrap `u`, like a latitude band's rims), a ray along
/// `+v` decides instead, where a boundary running toward `-u` has the hit on
/// its face side.
///
/// Returns `None` only when neither ray meets any boundary, which cannot
/// happen once some loop wraps a periodic axis.
fn oriented_periodic_containment(
    loops: &[&[(f64, f64)]],
    hit_u: f64,
    hit_v: f64,
    u_period: Option<f64>,
    v_period: Option<f64>,
) -> Option<bool> {
    let periods = (u_period, v_period);
    // Ray along +u, then along +v when no loop crosses the first.
    nearest_oriented_crossing(loops, (hit_u, hit_v), periods, false)
        .or_else(|| nearest_oriented_crossing(loops, (hit_u, hit_v), periods, true))
        .map(|direction| direction > 0.0)
}

/// Net orientation of the nearest boundary crossing along a ray from `hit`.
///
/// The ray runs toward `+u`, or toward `+v` when `along_v`. Each crossing
/// contributes the sign of the segment's step across the ray (its `v` step for
/// the `u` ray; its `u` step, negated, for the `v` ray), so a positive result
/// always means the hit is on the face side. Crossings at one place along the
/// ray are summed first: the doubled traversal of a seam (up and back down at
/// the same `u`) cancels and the ray looks past it, and a sampled vertex
/// reached by both of its segments is not counted twice. The first group with
/// a non-zero sum decides; `None` when the ray meets no boundary at all.
fn nearest_oriented_crossing(
    loops: &[&[(f64, f64)]],
    hit: (f64, f64),
    periods: (Option<f64>, Option<f64>),
    along_v: bool,
) -> Option<f64> {
    // Work in (s, t): `s` runs along the ray, `t` across it.
    let st = |(u, v): (f64, f64)| if along_v { (v, u) } else { (u, v) };
    let (s0, t0) = st(hit);
    let (s_period, t_period) = if along_v {
        (periods.1, periods.0)
    } else {
        periods
    };
    // Transposing the axes mirrors the plane, which reverses orientation.
    let face_side = if along_v { -1.0 } else { 1.0 };

    let mut crossings: Vec<(f64, f64)> = Vec::new();
    for uv_loop in loops {
        let n = uv_loop.len();
        if n < 2 {
            continue;
        }
        let Some(close) = closing_point(uv_loop, periods.0, periods.1) else {
            continue;
        };
        for i in 0..n {
            let a = st(uv_loop[i]);
            let b = st(if i + 1 < n { uv_loop[i + 1] } else { close });
            let (lo, hi) = (a.1.min(b.1), a.1.max(b.1));
            if hi <= lo {
                continue;
            }
            // Every copy of the ray's line `t = t0 + k * period` the segment
            // spans, half-open so a vertex shared by two segments counts once.
            let mut lines: SmallVec<[f64; 2]> = SmallVec::new();
            match t_period {
                Some(p) => {
                    let first = ((lo - t0) / p).ceil().mul_add(p, t0);
                    lines.extend(
                        (0_u32..)
                            .map(|k| f64::from(k).mul_add(p, first))
                            .take_while(|&t| t < hi)
                            .filter(|&t| t >= lo),
                    );
                }
                None if (lo..hi).contains(&t0) => lines.push(t0),
                None => {}
            }
            for t in lines {
                let s = (t - a.1).mul_add((b.0 - a.0) / (b.1 - a.1), a.0);
                let distance = s_period.map_or(s - s0, |p| (s - s0).rem_euclid(p));
                if distance >= 0.0 {
                    crossings.push((distance, face_side * (b.1 - a.1).signum()));
                }
            }
        }
    }

    crossings.compact_sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut i = 0;
    while i < crossings.len() {
        let start = crossings[i].0;
        let mut sum = 0.0;
        while i < crossings.len() && crossings[i].0 - start <= CROSSING_GROUP_EPS {
            sum += crossings[i].1;
            i += 1;
        }
        if sum != 0.0 {
            return Some(sum);
        }
    }
    None
}

/// Compute the normal of a polygon via Newell's method.
///
/// Returns a unit-length normal, or `(0,0,1)` for degenerate polygons.
pub fn polygon_normal(verts: &[Point3]) -> Vec3 {
    crate::util::polygon_normal(verts)
}

/// Build the UV boundary of every hole from already-sampled 3D hole polygons.
///
/// Infallible: the topology lookups already happened when the caller built
/// its [`FaceTrimData`].
fn hole_uv_boundaries_from_cached<F>(
    holes_3d: &[Vec<Point3>],
    project: &F,
    u_period: Option<f64>,
    v_period: Option<f64>,
    u_pole: Option<f64>,
) -> Vec<Vec<(f64, f64)>>
where
    F: Fn(Point3) -> (f64, f64),
{
    holes_3d
        .iter()
        .map(|poly| build_uv_boundary(poly, project, u_period, v_period, u_pole))
        .collect()
}

/// Test a point against an outer trim polygon only (no hole sampling).
///
/// Full-surface polygons (fewer than 3 points, like a torus with seam edges
/// only) contain every point. This is the cheap first half of
/// [`crate::classify::trim_contains_point`]: callers sample holes only when
/// it passes.
#[must_use]
pub fn outer_contains_point(outer: &[Point3], point: Point3) -> bool {
    if outer.len() < 3 {
        return true;
    }
    let normal = polygon_normal(outer);
    point_in_polygon_3d(&point, outer, &normal)
}

/// True when a hit lands in one of the face's holes, where the trimmed face
/// has no material and therefore no crossing.
fn hit_in_hole_uv(
    holes: &[Vec<(f64, f64)>],
    hit_u: f64,
    hit_v: f64,
    u_period: Option<f64>,
    v_period: Option<f64>,
) -> bool {
    holes
        .iter()
        .any(|hole| point_in_uv_boundary(hit_u, hit_v, hole, u_period, v_period))
}

/// Per-face trim data reused across rays and query points (PERF-Q01).
///
/// Built once from the borrowed topology: the sampled outer-wire polygon plus
/// one sampled polygon per inner (hole) wire. Building it performs no surface
/// queries, so the same value feeds the plane, analytic-UV, sphere-cap,
/// 3D-polygon and NURBS crossing tests unchanged. The narrow-phase
/// `*_with_trim` variants below take `Some(trim)` for the prepared path and
/// `None` for the one-shot path, which builds exactly what the pre-refactor
/// code built, at exactly the point it built it.
#[derive(Debug, Clone)]
pub struct FaceTrimData {
    /// Sampled outer-wire polygon (`face_polygon`).
    pub outer: Vec<Point3>,
    /// One sampled polygon per inner wire (`face_hole_polygons`).
    pub holes: Vec<Vec<Point3>>,
}

impl FaceTrimData {
    /// Build the trim polygons for a face.
    ///
    /// # Errors
    ///
    /// Returns an error if any topology entity referenced by the face is missing.
    pub fn build(topo: &Topology, face_id: FaceId) -> Result<Self, CheckError> {
        crate::perf::bump_classify_trim_build();
        Ok(Self {
            outer: face_polygon(topo, face_id)?,
            holes: face_hole_polygons(topo, face_id)?,
        })
    }

    /// Sample the outer-wire polygon only.
    ///
    /// Call sites that test outer containment before paying for hole sampling
    /// (the pre-refactor lazy order of the plane crossing and boundary tests)
    /// start here and complete with [`FaceTrimData::build_holes`] only on a hit.
    ///
    /// # Errors
    ///
    /// Returns an error if any topology entity referenced by the face is missing.
    pub fn build_outer(topo: &Topology, face_id: FaceId) -> Result<Vec<Point3>, CheckError> {
        crate::perf::bump_classify_trim_build();
        face_polygon(topo, face_id)
    }

    /// Sample one polygon per inner (hole) wire.
    ///
    /// Completes an outer-only trim; see [`FaceTrimData::build_outer`].
    ///
    /// # Errors
    ///
    /// Returns an error if any topology entity referenced by the face is missing.
    pub fn build_holes(topo: &Topology, face_id: FaceId) -> Result<Vec<Vec<Point3>>, CheckError> {
        crate::perf::bump_classify_trim_build();
        face_hole_polygons(topo, face_id)
    }

    /// Assemble a trim from separately built parts (no extra counting: the
    /// parts already counted when sampled).
    #[must_use]
    pub fn from_parts(outer: Vec<Point3>, holes: Vec<Vec<Point3>>) -> Self {
        Self { outer, holes }
    }
}

/// Seed-grid resolution for ray-NURBS crossings (`ray_nurbs`'s `n_samples`).
const NURBS_RAY_SAMPLES: usize = 20;

/// Ray- and point-independent work for one face, computed at most once per
/// prepared face and then shared by every later query (O06).
///
/// Each value is a pure function of the face's support surface and its trim
/// polygons, so a cached value is bit-for-bit what the one-shot path
/// recomputes on every ray: for a NURBS face the line-intersection seed grid
/// behind `ray_nurbs` ([`LineSurfaceSeedGrid`]), the point-projection seed
/// grid behind `project_point_to_surface` ([`SurfaceSeedGrid`]) and the UV
/// images of the trim polygons; for a cylinder, cone or torus face the UV
/// images of its trim polygons. A cache belongs to exactly one face of one
/// frozen topology. `OnceLock` keeps the owning context `Send + Sync`.
#[derive(Debug, Clone, Default)]
pub struct FaceCache {
    line_grid: OnceLock<LineSurfaceSeedGrid>,
    projection_grid: OnceLock<SurfaceSeedGrid>,
    uv_trim: OnceLock<NurbsUvTrim>,
    analytic_uv_trim: OnceLock<AnalyticUvTrim>,
    spherical_trim: OnceLock<Option<GreatCircleTrim>>,
    /// The exact trim of a planar face with a curved edge.
    plane_trim: OnceLock<Option<PlanarTrim>>,
    /// Half-space supports of a sphere patch bounded by two circles.
    circle_patch: OnceLock<Option<Vec<(Point3, Vec3)>>>,
    /// Circle-arc trim of a sphere face the narrower sphere paths decline.
    sphere_arc_trim: OnceLock<Option<sphere_arcs::SphereArcTrim>>,
}

/// UV images of a cylinder, cone or torus face's trim polygons, as
/// [`count_analytic_crossings_with_trim`] builds them.
#[derive(Debug, Clone)]
struct AnalyticUvTrim {
    /// The outer loop in UV, or `None` for a full-surface face.
    uv_boundary: Option<Vec<(f64, f64)>>,
    /// Hole loops in UV.
    holes: Vec<Vec<(f64, f64)>>,
    /// Whether some loop wraps a period of a doubly periodic surface, so
    /// containment is decided by orientation rather than parity.
    wraps: bool,
}

impl AnalyticUvTrim {
    fn build<F>(
        trim_data: &FaceTrimData,
        project: &F,
        v_periodic: bool,
        u_pole: Option<f64>,
    ) -> Self
    where
        F: Fn(Point3) -> (f64, f64),
    {
        let verts = &trim_data.outer;

        // Detect degenerate boundary: a "full-surface" face whose wire has
        // fewer than 3 distinct vertices. Every positive-t root outside a hole
        // counts.
        let is_full_surface = verts.len() < 3 || {
            let ref_pt = verts[0];
            verts
                .iter()
                .all(|v| (*v - ref_pt).length_squared() < COINCIDENT_SQ)
        };
        // Every analytic surface here parameterizes u as an angle with period
        // 2pi; v is angular only for the torus.
        let u_period = Some(std::f64::consts::TAU);
        let v_period = v_periodic.then_some(std::f64::consts::TAU);
        let uv_boundary = (!is_full_surface)
            .then(|| build_uv_boundary(verts, project, u_period, v_period, u_pole));
        let holes =
            hole_uv_boundaries_from_cached(&trim_data.holes, project, u_period, v_period, u_pole);

        // A loop that wraps a period bounds no polygon: decide by orientation.
        // Only on the torus. Booleans bound cylinder and cone walls with
        // doubled seams, whose loops close in `(u, v)`, so parity stays exact
        // there; a seamless two-ring wall (outer rim, inner rim) still reads
        // as empty.
        let wraps = v_periodic
            && uv_boundary
                .iter()
                .chain(&holes)
                .any(|uv_loop| uv_loop_wraps(uv_loop, u_period, v_period));
        Self {
            uv_boundary,
            holes,
            wraps,
        }
    }
}

impl FaceCache {
    /// The projection seed grid for `surface` (the face's support surface).
    pub fn projection_grid(&self, surface: &NurbsSurface) -> &SurfaceSeedGrid {
        self.projection_grid
            .get_or_init(|| SurfaceSeedGrid::for_surface(surface))
    }

    fn line_grid(&self, surface: &NurbsSurface) -> &LineSurfaceSeedGrid {
        self.line_grid
            .get_or_init(|| LineSurfaceSeedGrid::for_surface(surface, NURBS_RAY_SAMPLES))
    }
}

/// `ParametricSurface::project_point` for a NURBS surface, optionally seeded
/// from a prebuilt grid: the same Newton start, so the same `(u, v)`, and the
/// same domain-midpoint fallback when Newton fails.
fn nurbs_project_uv(
    surface: &NurbsSurface,
    point: Point3,
    grid: Option<&SurfaceSeedGrid>,
) -> (f64, f64) {
    let Some(grid) = grid else {
        return surface.project_point(point);
    };
    if let Ok(proj) = project_point_to_surface_with_grid(surface, point, 1e-7, grid) {
        (proj.u, proj.v)
    } else {
        let (u0, u1) = surface.domain_u();
        let (v0, v1) = surface.domain_v();
        ((u0 + u1) * 0.5, (v0 + v1) * 0.5)
    }
}

/// UV images of a NURBS face's trim polygons, as [`count_nurbs_hits_with_trim`]
/// builds them.
#[derive(Debug, Clone)]
struct NurbsUvTrim {
    /// The surface's `u` period when it closes in `u` (its knot span).
    u_period: Option<f64>,
    /// The surface's `v` period when it closes in `v` (its knot span).
    v_period: Option<f64>,
    /// The outer loop in UV, or `None` for a full-surface face.
    uv_boundary: Option<Vec<(f64, f64)>>,
    /// Whether `uv_boundary` encloses no UV area.
    degenerate: bool,
    /// Hole loops in UV; built only for a non-degenerate boundary, exactly
    /// when the one-shot path builds them.
    holes: Vec<Vec<(f64, f64)>>,
}

impl NurbsUvTrim {
    fn build(surface: &NurbsSurface, trim: &FaceTrimData, grid: Option<&SurfaceSeedGrid>) -> Self {
        let verts = &trim.outer;
        let project = |p: Point3| -> (f64, f64) { nurbs_project_uv(surface, p, grid) };
        // A full-surface face counts every forward hit that does not land in
        // a hole. `count_analytic_crossings` tests this two ways -- too few
        // boundary points, OR every boundary point coincident -- and only the
        // first test was made here. A doubly-periodic face (a full torus) has
        // a boundary of exactly its seam endpoints: four vertices, all at one
        // place. That passes `len() >= 3`, so a degenerate polygon was built
        // and every hit on the face was then trimmed away against it.
        let is_full_surface = verts.len() < 3 || {
            let ref_pt = verts[0];
            verts
                .iter()
                .all(|v| (*v - ref_pt).length_squared() < COINCIDENT_SQ)
        };
        // A NURBS u/v are knot parameters, not angles. Unwrapping them by 2pi
        // is meaningless -- and actively wrong, since a b-spline domain
        // routinely spans more than pi (a converted box face spans 12.0), so
        // consecutive boundary vertices get shifted by a spurious 2pi. When
        // the surface really does close in a direction, the period is that
        // direction's knot span: without it the seam projects onto the wrong
        // branch and the trim polygon collapses, rejecting every hit on the
        // face.
        let u_period = surface
            .is_periodic_u()
            .then(|| surface.domain_u().1 - surface.domain_u().0);
        let v_period = surface
            .is_periodic_v()
            .then(|| surface.domain_v().1 - surface.domain_v().0);
        let uv_boundary = (!is_full_surface)
            .then(|| build_uv_boundary(verts, &project, u_period, v_period, None));
        let degenerate = uv_boundary
            .as_deref()
            .is_some_and(uv_boundary_is_degenerate);
        let holes = if degenerate {
            Vec::new()
        } else {
            hole_uv_boundaries_from_cached(&trim.holes, &project, u_period, v_period, None)
        };
        Self {
            u_period,
            v_period,
            uv_boundary,
            degenerate,
            holes,
        }
    }
}

/// True when a hit lands in one of the face's holes (3D polygon variant).
/// Whether `hit` lies in a hole. Nested inner wires alternate void and
/// material (an island inside a cavity opening), so a hit is in a hole when
/// an odd number of hole loops contain it.
fn hit_in_hole_3d(holes: &[Vec<Point3>], hit: Point3, normal: Vec3) -> bool {
    holes
        .iter()
        .filter(|hole| point_in_polygon_3d(&hit, hole, &normal))
        .count()
        % 2
        == 1
}

/// Count crossings for analytic (non-planar) faces using UV containment.
///
/// Given ray parameter roots (where the ray hits the infinite surface),
/// checks whether each hit point falls within the face's trimming boundary
/// by projecting to the surface's (u,v) parameter space.
///
/// If the face boundary is degenerate (all vertices coincide, as in a full
/// torus face with seam edges), every positive-t root is counted as a crossing.
///
/// # Errors
///
/// Returns an error if topology lookups fail.
fn count_analytic_crossings<F>(
    topo: &Topology,
    face_id: FaceId,
    hits: &[Point3],
    project: F,
    v_periodic: bool,
    u_pole: Option<f64>,
) -> Result<u32, CheckError>
where
    F: Fn(Point3) -> (f64, f64),
{
    count_analytic_crossings_with_trim(topo, face_id, None, None, hits, project, v_periodic, u_pole)
}

/// [`count_analytic_crossings`] with caller-supplied trim data and an optional
/// prepared cache for the trim's UV images.
///
/// `trim` carries the prepared polygons; `None` builds them on demand exactly
/// as the one-shot path always has (preserving its early-outs and errors).
#[allow(clippy::too_many_arguments)]
fn count_analytic_crossings_with_trim<F>(
    topo: &Topology,
    face_id: FaceId,
    trim: Option<&FaceTrimData>,
    cache: Option<&FaceCache>,
    hits: &[Point3],
    project: F,
    v_periodic: bool,
    u_pole: Option<f64>,
) -> Result<u32, CheckError>
where
    F: Fn(Point3) -> (f64, f64),
{
    if hits.is_empty() {
        return Ok(0);
    }

    let owned_trim;
    let trim_data = if let Some(cached) = trim {
        cached
    } else {
        owned_trim = FaceTrimData::build(topo, face_id)?;
        &owned_trim
    };
    let owned_uv;
    let uv = if let Some(cache) = cache {
        cache
            .analytic_uv_trim
            .get_or_init(|| AnalyticUvTrim::build(trim_data, &project, v_periodic, u_pole))
    } else {
        owned_uv = AnalyticUvTrim::build(trim_data, &project, v_periodic, u_pole);
        &owned_uv
    };
    let u_period = Some(std::f64::consts::TAU);
    let v_period = v_periodic.then_some(std::f64::consts::TAU);
    let uv_boundary = &uv.uv_boundary;
    let holes = &uv.holes;

    if uv.wraps {
        let loops: Vec<&[(f64, f64)]> =
            uv_boundary.iter().chain(holes).map(Vec::as_slice).collect();
        let mut crossings = 0u32;
        for &hit in hits {
            let (hit_u, hit_v) = project(hit);
            if oriented_periodic_containment(&loops, hit_u, hit_v, u_period, v_period) == Some(true)
            {
                crossings += 1;
            }
        }
        return Ok(crossings);
    }

    let mut crossings = 0u32;
    for &hit in hits {
        let (hit_u, hit_v) = project(hit);

        if let Some(boundary) = uv_boundary
            && !point_in_uv_boundary(hit_u, hit_v, boundary, u_period, v_period)
        {
            continue;
        }
        // The trimmed face carries no material inside its inner wires.
        if hit_in_hole_uv(holes, hit_u, hit_v, u_period, v_period) {
            continue;
        }
        crossings += 1;
    }

    Ok(crossings)
}

/// Count ray crossings of a spherical face whose outer boundary is planar,
/// trimming against the boundary PLANE rather than a polygon inscribed in it.
///
/// Returns `None` when the boundary is not planar, so the caller keeps the
/// polygon path for lunes and boolean-made spherical triangles.
///
/// # Why this exists
///
/// `face_polygon` samples a closed boundary edge at a fixed 32 points, so a
/// hemisphere's equator becomes a 32-gon INSCRIBED in the true circle. A ray
/// leaving the sphere within the scalloped band between chord and arc — angular
/// half-width ~pi/n, so 0.098 r at n = 32 — is inside neither hemisphere's
/// polygon: the north face rejects it on containment and the south face rejects
/// it on the half-space test. The crossing is counted by NO face, parity flips,
/// and an interior point is reported `Outside`. Measured on `make_sphere(1, s)`
/// at 0.9 r: 32.4% wrong at s = 8, 3.3% at s = 32, still 0.25% at s = 128, every
/// failure Inside -> Outside. The sphere CENTRE is always right, which is why
/// the single-point tests never caught it.
///
/// This is the sagitta gap the `numerical-robustness` guidance names, and the
/// same one `sphere_seam_plane_crossings` (`algo/src/pave_filler/phase_ff.rs`)
/// already fixes for section circles — its doc describes this defect verbatim.
///
/// # Why the cap side comes from the surface, not the winding
///
/// The obvious form of this fix reuses the sign already computed here from the
/// boundary polygon's Newell normal negated by `is_reversed`. That sign is
/// wrong on boolean-produced spherical faces, where it survives today only
/// because two complementary hemispheres tile the sphere and the errors cancel.
/// Break the symmetry — `cut(box, sphere)` leaves an annular face whose
/// complementary cap is not a face of the same solid — and a winding-derived
/// half-space counts a phantom cap, which is an O(1) error rather than the
/// bounded one it replaces.
///
/// So the side is taken from geometry the B-Rep states directly: the outward
/// surface normal at a boundary point (analytic, `(p - centre)/r` flipped by
/// `is_reversed`) crossed with the boundary's own traversal direction gives the
/// inward tangent, and its component along the plane normal says which cap the
/// face occupies.
fn count_sphere_cap_crossings(
    topo: &Topology,
    face_id: FaceId,
    hits: &[Point3],
    sph: &remus_math::surfaces::SphericalSurface,
) -> Result<Option<u32>, CheckError> {
    count_sphere_cap_crossings_with_trim(topo, face_id, None, hits, sph)
}

/// [`count_sphere_cap_crossings`] with caller-supplied trim data (`None`
/// builds it on demand, exactly as the one-shot path always has).
fn count_sphere_cap_crossings_with_trim(
    topo: &Topology,
    face_id: FaceId,
    trim: Option<&FaceTrimData>,
    hits: &[Point3],
    sph: &remus_math::surfaces::SphericalSurface,
) -> Result<Option<u32>, CheckError> {
    if hits.is_empty() {
        return Ok(Some(0));
    }

    let owned_trim;
    let trim_data = if let Some(cached) = trim {
        cached
    } else {
        owned_trim = FaceTrimData::build(topo, face_id)?;
        &owned_trim
    };
    let verts = &trim_data.outer;
    if verts.len() < 3 {
        return Ok(None);
    }

    let plane_normal = polygon_normal(verts);
    let n_len = plane_normal.length();
    if n_len < 1e-12 {
        return Ok(None); // degenerate or self-intersecting boundary
    }
    let plane_normal = plane_normal * (1.0 / n_len);
    let ref_pt = verts[0];

    // Planar to tolerance? A cap boundary is; a lune's is not.
    let radius = sph.radius();
    let planarity_tol = 1e-9_f64.mul_add(radius, 1e-9);
    if verts
        .iter()
        .any(|v| ((*v - ref_pt).dot(plane_normal)).abs() > planarity_tol)
    {
        return Ok(None);
    }

    // Which cap does the face occupy? Outward surface normal at a boundary
    // vertex, crossed with the direction the boundary is walked, points into
    // the face along the surface; its component along the plane normal is the
    // cap side. `face_polygon` returns the loop in traversal order.
    let mut inward_side = 0.0_f64;
    for i in 0..verts.len() {
        let v = verts[i];
        let next = verts[(i + 1) % verts.len()];
        let tangent = next - v;
        if tangent.length() < 1e-12 {
            continue;
        }
        let outward = v - sph.center();
        let r = outward.length();
        if r < 1e-12 {
            continue;
        }
        let outward = outward * (1.0 / r);
        // NOT negated by `is_reversed`. The traversal direction already carries
        // the face's orientation — `face_polygon` walks the outer wire as the
        // B-Rep stores it — so flipping the surface normal here as well applies
        // the same reversal twice. Measured: with the extra flip, the annular
        // face left by `cut(box, sphere)` where the sphere breaks the surface
        // (so its complementary cap is NOT a face of the same solid) gets 34.3%
        // of its carved region wrong; without it, 0.000%. The double flip is
        // invisible on a whole `make_sphere`, where two complementary
        // hemispheres tile the sphere and the errors cancel.
        let candidate = outward.cross(tangent).dot(plane_normal);
        if candidate.abs() > inward_side.abs() {
            inward_side = candidate;
        }
    }
    if inward_side.abs() < 1e-12 {
        return Ok(None);
    }
    // +1 when the face lies on the +plane_normal side, -1 otherwise.
    let cap_sign = if inward_side > 0.0 { 1.0 } else { -1.0 };

    let holes = &trim_data.holes;
    let mut crossings = 0u32;
    for &hit in hits {
        // Exact: the cap is every point of the sphere on this side of the plane.
        if (hit - ref_pt).dot(plane_normal) * cap_sign < -HALF_SPACE_EPS {
            continue;
        }
        if hit_in_hole_3d(holes, hit, plane_normal) {
            continue;
        }
        crossings += 1;
    }

    Ok(Some(crossings))
}

/// Outward arithmetic for the qualified great-circle trim predicate.
/// Unknown/overflowed arithmetic never decides a containment sign.
mod sphere_arcs;

mod spherical_trim_math {
    use super::{Point3, Vec3};

    #[derive(Debug, Clone, Copy)]
    pub(super) struct I {
        pub(super) lo: f64,
        pub(super) hi: f64,
    }

    impl I {
        pub(super) const fn point(value: f64) -> Self {
            Self {
                lo: value,
                hi: value,
            }
        }
        const fn unknown() -> Self {
            Self {
                lo: f64::NEG_INFINITY,
                hi: f64::INFINITY,
            }
        }
        pub(super) fn finite(self) -> bool {
            self.lo.is_finite() && self.hi.is_finite() && self.lo <= self.hi
        }
        pub(super) fn add(self, other: Self) -> Self {
            if !self.finite() || !other.finite() {
                return Self::unknown();
            }
            let result = Self {
                lo: (self.lo + other.lo).next_down(),
                hi: (self.hi + other.hi).next_up(),
            };
            if result.finite() {
                result
            } else {
                Self::unknown()
            }
        }
        pub(super) fn neg(self) -> Self {
            Self {
                lo: -self.hi,
                hi: -self.lo,
            }
        }
        pub(super) fn sub(self, other: Self) -> Self {
            self.add(other.neg())
        }
        pub(super) fn mul(self, other: Self) -> Self {
            if !self.finite() || !other.finite() {
                return Self::unknown();
            }
            let products = [
                self.lo * other.lo,
                self.lo * other.hi,
                self.hi * other.lo,
                self.hi * other.hi,
            ];
            if products.iter().any(|value| !value.is_finite()) {
                return Self::unknown();
            }
            Self {
                lo: products
                    .into_iter()
                    .fold(f64::INFINITY, f64::min)
                    .next_down(),
                hi: products
                    .into_iter()
                    .fold(f64::NEG_INFINITY, f64::max)
                    .next_up(),
            }
        }
        pub(super) fn div_positive(self, divisor: f64) -> Self {
            if !self.finite() || !divisor.is_finite() || divisor <= 0.0 {
                return Self::unknown();
            }
            Self {
                lo: (self.lo / divisor).next_down(),
                hi: (self.hi / divisor).next_up(),
            }
        }
        pub(super) fn abs(self) -> Self {
            if !self.finite() {
                return Self::unknown();
            }
            if self.lo <= 0.0 && self.hi >= 0.0 {
                Self {
                    lo: 0.0,
                    hi: self.lo.abs().max(self.hi.abs()),
                }
            } else {
                Self {
                    lo: self.lo.abs().min(self.hi.abs()),
                    hi: self.lo.abs().max(self.hi.abs()),
                }
            }
        }
    }
    pub(super) type V = [I; 3];
    pub(super) fn vector(v: Vec3) -> V {
        v.0.map(I::point)
    }
    pub(super) fn difference(a: Point3, b: Point3) -> V {
        std::array::from_fn(|j| I::point(a.0[j]).sub(I::point(b.0[j])))
    }
    pub(super) fn add(a: V, b: V) -> V {
        std::array::from_fn(|j| a[j].add(b[j]))
    }
    pub(super) fn sub(a: V, b: V) -> V {
        std::array::from_fn(|j| a[j].sub(b[j]))
    }
    pub(super) fn scale(a: V, b: I) -> V {
        a.map(|value| value.mul(b))
    }
    pub(super) fn dot(a: V, b: V) -> I {
        a[0].mul(b[0]).add(a[1].mul(b[1])).add(a[2].mul(b[2]))
    }
    pub(super) fn cross(a: V, b: V) -> V {
        [
            a[1].mul(b[2]).sub(a[2].mul(b[1])),
            a[2].mul(b[0]).sub(a[0].mul(b[2])),
            a[0].mul(b[1]).sub(a[1].mul(b[0])),
        ]
    }
    pub(super) fn norm1(v: V) -> I {
        v[0].abs().add(v[1].abs()).add(v[2].abs())
    }

    /// Reduce by an outward enclosure of π/2, then use degree39/38
    /// Taylor series on |x|<=1. The omitted terms are below2e-48;
    ///1e-30 encloses both remainders. Quadrant reduction avoids cancellation
    /// nearπ, including at large model scales with fixed absolute tolerances.
    pub(super) fn sin_cos(parameter: f64) -> Option<(I, I)> {
        use std::f64::consts::{FRAC_PI_2, PI};
        if !parameter.is_finite() || parameter.abs() > 64.0 * PI {
            return None;
        }
        let turns = (parameter / FRAC_PI_2).round();
        let x = I::point(parameter).sub(
            I {
                lo: FRAC_PI_2.next_down(),
                hi: FRAC_PI_2.next_up(),
            }
            .mul(I::point(turns)),
        );
        if !x.finite() || x.abs().hi > 1.0 {
            return None;
        }
        let square = x.mul(x);
        let (mut sine_term, mut cosine_term) = (x, I::point(1.0));
        let (mut sine, mut cosine) = (sine_term, cosine_term);
        for k in 1..20 {
            sine_term = sine_term
                .mul(square)
                .neg()
                .div_positive(f64::from((2 * k) * (2 * k + 1)));
            cosine_term = cosine_term
                .mul(square)
                .neg()
                .div_positive(f64::from((2 * k) * (2 * k - 1)));
            sine = sine.add(sine_term);
            cosine = cosine.add(cosine_term);
        }
        let remainder = I {
            lo: -1e-30,
            hi: 1e-30,
        };
        let sine = sine.add(remainder);
        let cosine = cosine.add(remainder);
        let quadrant = turns.rem_euclid(4.0);
        let result = if quadrant < 0.5 {
            (sine, cosine)
        } else if quadrant < 1.5 {
            (cosine, sine.neg())
        } else if quadrant < 2.5 {
            (sine.neg(), cosine.neg())
        } else {
            (cosine.neg(), sine)
        };
        (result.0.finite() && result.1.finite()).then_some(result)
    }
}

/// One full central-circle chain, or three support chains, with a simple boundary in
/// one closed hemisphere. Native major arcs are retained; intermediate
/// endpoints only partition exact great-circle arcs into spans shorter than π.
#[derive(Debug, Clone)]
struct GreatCircleTrim {
    vertices: Vec<spherical_trim_math::V>,
    hemisphere: spherical_trim_math::V,
    hemisphere_floor: f64,
    angular_band: f64,
    // Zero retains the exact-center path. Otherwise this bounds the whole
    // carrier deformation and all native-to-shared-anchor connectors, in
    // Sphere-radius units; it is reserved inside the existing physical band.
    deformation_band: f64,
    support_normals: Vec<spherical_trim_math::V>,
}

#[derive(Clone)]
struct GreatCircleChain {
    circle: remus_math::curves::Circle3D,
    start: remus_topology::vertex::VertexId,
    end: remus_topology::vertex::VertexId,
    from: f64,
    to: f64,
}

// These are identities of stored native carriers/parameters, not proximity
// tests. Coalescing different frames or offset periods would lose trim authority.
fn same_great_circle_frame(
    a: &remus_math::curves::Circle3D,
    b: &remus_math::curves::Circle3D,
) -> bool {
    a.center().0.map(f64::to_bits) == b.center().0.map(f64::to_bits)
        && a.radius().to_bits() == b.radius().to_bits()
        && a.u_axis().0.map(f64::to_bits) == b.u_axis().0.map(f64::to_bits)
        && a.v_axis().0.map(f64::to_bits) == b.v_axis().0.map(f64::to_bits)
}

impl GreatCircleTrim {
    #[allow(clippy::too_many_lines)]
    fn build(
        topo: &Topology,
        face_id: FaceId,
        sphere: &remus_math::surfaces::SphericalSurface,
    ) -> Result<Option<Self>, CheckError> {
        use spherical_trim_math::{
            I, V, add, cross, difference, dot, norm1, scale, sin_cos, vector,
        };
        let face = topo.face(face_id)?;
        if !face.inner_wires().is_empty() || !sphere.radius().is_finite() || sphere.radius() <= 0.0
        {
            return Ok(None);
        }
        let wire = topo.wire(face.outer_wire())?;
        if !wire.is_closed() {
            return Ok(None);
        }
        let mut chains: Vec<GreatCircleChain> = Vec::new();
        let mut has_support_error = false;
        let mut previous_end = None;
        let mut first_start = None;
        for oriented in wire.edges() {
            let edge = topo.edge(oriented.edge())?;
            let EdgeCurve::Circle(circle) = edge.curve() else {
                return Ok(None);
            };
            // Exact stored supports retain their established path. A slightly
            // offset actual Circle needs a whole-carrier deformation proof
            // below; proximity alone never authorizes its containment signs.
            if circle.center().0.map(f64::to_bits) != sphere.center().0.map(f64::to_bits)
                || circle.radius().to_bits() != sphere.radius().to_bits()
            {
                has_support_error = true;
                let center_error = norm1(difference(circle.center(), sphere.center()));
                let radius_error = I::point(circle.radius())
                    .sub(I::point(sphere.radius()))
                    .abs();
                if !center_error.finite()
                    || !radius_error.finite()
                    || center_error.hi > HALF_SPACE_EPS * 0.25
                    || radius_error.hi > HALF_SPACE_EPS * 0.25
                {
                    return Ok(None);
                }
            }
            let start = oriented.oriented_start(edge);
            let end = oriented.oriented_end(edge);
            if previous_end.is_some_and(|previous| previous != start) {
                return Ok(None);
            }
            first_start.get_or_insert(start);
            previous_end = Some(end);
            let (mut from, mut to) = edge
                .strict_domain()
                .map_err(crate::error::edge_domain_validation)?;
            if !oriented.is_forward() {
                std::mem::swap(&mut from, &mut to);
            }
            if let Some(last) = chains.last_mut()
                && same_great_circle_frame(&last.circle, circle)
                && last.to.to_bits() == from.to_bits()
                && (last.to - last.from).is_sign_positive() == (to - from).is_sign_positive()
            {
                last.end = end;
                last.to = to;
            } else {
                chains.push(GreatCircleChain {
                    circle: circle.clone(),
                    start,
                    end,
                    from,
                    to,
                });
            }
        }
        if first_start != previous_end {
            return Ok(None);
        }
        if chains.len() > 1 {
            let first = &chains[0];
            let last = &chains[chains.len() - 1];
            if same_great_circle_frame(&first.circle, &last.circle)
                && last.to.to_bits() == first.from.to_bits()
                && (last.to - last.from).is_sign_positive()
                    == (first.to - first.from).is_sign_positive()
            {
                let Some(last) = chains.pop() else {
                    return Ok(None);
                };
                chains[0].start = last.start;
                chains[0].from = last.from;
            }
        }
        let full_circle = chains.len() == 1
            && chains[0].start == chains[0].end
            && (chains[0].to - chains[0].from).abs().to_bits() == std::f64::consts::TAU.to_bits();
        if !full_circle && chains.len() != 3 {
            return Ok(None);
        }
        // Each pair of distinct central support planes intersects at the
        // common junction and its antipode. At least one member of every
        // pair is a minor arc, so that antipode is outside that member.
        let mut major_arcs = 0;
        let radius = sphere.radius();
        let angular_band = HALF_SPACE_EPS / radius;
        if !angular_band.is_finite() || angular_band <= 0.0 {
            return Ok(None);
        }
        let mut normals: Vec<V> = Vec::new();
        let mut vertices = Vec::new();
        let mut pending_end = None;
        let mut carrier_error = I::point(0.0);
        let mut connector_error = I::point(0.0);
        for chain in &chains {
            let span = chain.to - chain.from;
            if !span.is_finite()
                || span.abs() <= 1e-12
                || (!full_circle && span.abs() >= std::f64::consts::TAU.next_down())
            {
                return Ok(None);
            }
            if span.abs() >= std::f64::consts::PI.next_down() {
                major_arcs += 1;
            }
            let u = vector(chain.circle.u_axis());
            let v = vector(chain.circle.v_axis());
            let normal = cross(u, v);
            let gram = dot(u, u)
                .sub(I::point(1.0))
                .abs()
                .add(dot(v, v).sub(I::point(1.0)).abs())
                .add(dot(u, v).abs());
            if !gram.finite() || gram.hi > 1e-12 || gram.hi * radius > HALF_SPACE_EPS * 0.25 {
                return Ok(None);
            }
            if has_support_error {
                if !chain.circle.radius().is_finite() || chain.circle.radius() <= 0.0 {
                    return Ok(None);
                }
                // For w=u*cos(t)+v*sin(t), |w·w-1|<=gram and hence
                // ||w|-1|<=gram. This bounds the entire actual Circle against
                // its centered, radially normalized great-circle carrier.
                let error = norm1(difference(chain.circle.center(), sphere.center()))
                    .add(
                        I::point(chain.circle.radius())
                            .sub(I::point(radius))
                            .abs()
                            .mul(norm1(u).add(norm1(v))),
                    )
                    .add(gram.mul(I::point(radius)));
                if !error.finite() {
                    return Ok(None);
                }
                carrier_error.hi = carrier_error.hi.max(error.hi);
            }
            for existing in &normals {
                if dot(cross(normal, *existing), cross(normal, *existing)).lo <= 1e-20 {
                    return Ok(None);
                }
            }
            normals.push(scale(normal, I::point(span.signum())));
            // Four pieces per revolution suffice algebraically: a great-circle
            // subarc shorter than π has positive endpoint interpolation weights.
            let pieces = if span.abs() <= std::f64::consts::FRAC_PI_2 {
                1
            } else if span.abs() <= std::f64::consts::PI {
                2
            } else {
                4
            };
            for j in 0..=pieces {
                let parameter = if j == pieces {
                    chain.to
                } else {
                    (span / f64::from(pieces)).mul_add(f64::from(j), chain.from)
                };
                let Some((sine, cosine)) = sin_cos(parameter) else {
                    return Ok(None);
                };
                let native = add(scale(u, cosine), scale(v, sine));
                let point = if j == 0 || j == pieces {
                    let id = if j == 0 { chain.start } else { chain.end };
                    let anchor = difference(topo.vertex(id)?.point(), sphere.center())
                        .map(|value| value.div_positive(radius));
                    let endpoint_error = norm1(spherical_trim_math::sub(native, anchor));
                    if !endpoint_error.finite() || endpoint_error.hi > angular_band * 0.25 {
                        return Ok(None);
                    }
                    if has_support_error {
                        // The exact shared VertexId closes the actual loop.
                        // Both adjacent ideal endpoints connect to that same
                        // anchor; include those homotopies as well as the
                        // interior carrier deviation before using winding.
                        connector_error.hi = connector_error.hi.max(
                            endpoint_error
                                .mul(I::point(radius))
                                .add(gram.mul(I::point(radius)))
                                .hi,
                        );
                    }
                    std::array::from_fn(|k| I {
                        lo: anchor[k].lo.min(native[k].lo),
                        hi: anchor[k].hi.max(native[k].hi),
                    })
                } else {
                    native
                };
                if j == 0 {
                    let merged = pending_end.map_or(point, |previous: V| {
                        std::array::from_fn(|k| I {
                            lo: previous[k].lo.min(point[k].lo),
                            hi: previous[k].hi.max(point[k].hi),
                        })
                    });
                    vertices.push(merged);
                } else if j < pieces {
                    vertices.push(point);
                } else {
                    pending_end = Some(point);
                }
            }
        }
        if !full_circle && (major_arcs > 1 || chains.iter().any(|chain| chain.start == chain.end)) {
            return Ok(None);
        }
        if let Some(end) = pending_end {
            let first = &mut vertices[0];
            for k in 0..3 {
                first[k].lo = first[k].lo.min(end[k].lo);
                first[k].hi = first[k].hi.max(end[k].hi);
            }
        }
        let deformation_band = if has_support_error {
            // A native arc or its endpoint connector stays in the corresponding
            // ideal-point ball. Convexity covers the connector between the
            // actual native endpoint and shared anchor with the larger bound.
            let error = I {
                lo: 0.0,
                hi: carrier_error.hi.max(connector_error.hi),
            };
            if !error.finite() || error.div_positive(radius).hi > 0.25 {
                return Ok(None);
            }
            // Radial projection of a tube of radius error<r/4 onto the Sphere
            // moves directions by at most2*error/r. Include that distortion
            // before reserving the unchanged physical half-space budget.
            let projected_error = error.mul(I::point(2.0));
            if !projected_error.finite() || projected_error.hi > HALF_SPACE_EPS * 0.25 {
                return Ok(None);
            }
            projected_error.div_positive(radius).hi
        } else {
            0.0
        };
        // The chosen support is traversed with the face to its left. Every
        // other exact subarc stays in this hemisphere because its positive
        // endpoint coefficients sum to at most√2. The factor2 conservatively
        // bounds both endpoint uncertainty and the native/anchor connectors.
        for &hemisphere in &normals {
            let floor = vertices
                .iter()
                .map(|vertex| dot(*vertex, hemisphere).lo)
                .fold(0.0_f64, f64::min);
            let floor = (2.0 * floor).next_down();
            if floor >= -angular_band {
                return Ok(Some(Self {
                    vertices,
                    hemisphere,
                    hemisphere_floor: floor,
                    angular_band,
                    deformation_band,
                    support_normals: normals,
                }));
            }
        }
        Ok(None)
    }

    fn contains(
        &self,
        point: Point3,
        sphere: &remus_math::surfaces::SphericalSurface,
        tolerance: f64,
    ) -> Option<bool> {
        use spherical_trim_math::{I, V, add, cross, difference, dot, norm1, vector};
        let q = difference(point, sphere.center()).map(|value| value.div_positive(sphere.radius()));
        if q.iter().any(|value| !value.finite()) {
            return None;
        }
        let angular_band = if self.deformation_band > 0.0 {
            I::point(HALF_SPACE_EPS.min(tolerance))
                .div_positive(sphere.radius())
                .lo
        } else {
            self.angular_band.min(tolerance / sphere.radius())
        };
        if !angular_band.is_finite() || angular_band <= 0.0 {
            return None;
        }
        // A closed trim includes its exact great-circle arcs. This local band
        // is the existing spherical half-space band, not an edge weld tolerance.
        let boundary_band = if self.deformation_band > 0.0 {
            I::point(angular_band)
                .sub(I::point(self.deformation_band))
                .lo
        } else {
            angular_band
        };
        for (a, b) in self
            .vertices
            .iter()
            .copied()
            .zip(self.vertices.iter().copied().cycle().skip(1))
        {
            let normal = cross(a, b);
            let distance = dot(q, normal).abs();
            let normal_length = dot(normal, normal).lo.max(0.0).sqrt().next_down();
            let band = (boundary_band * normal_length).next_down();
            if boundary_band > 0.0
                && normal_length > 0.0
                && distance.finite()
                && distance.hi <= band
                && dot(q, add(a, b)).lo > 0.0
                && dot(cross(a, q), normal).lo >= -band
                && dot(cross(q, b), normal).lo >= -band
            {
                return Some(true);
            }
        }
        if self.deformation_band > 0.0 {
            // The actual offset loop and its centered ideal loop have the same
            // winding only away from every deformation tube. A strict plane
            // separation is conservative for the whole native arc and its
            // endpoint connectors; unresolved queries never use ideal winding.
            for normal in &self.support_normals {
                let separation = dot(q, *normal).abs();
                let uncertainty = I::point(self.deformation_band).mul(norm1(*normal));
                if !separation.finite() || !uncertainty.finite() || separation.lo <= uncertainty.hi
                {
                    return None;
                }
            }
        }
        let height = dot(q, self.hemisphere);
        let height_error = if self.deformation_band > 0.0 {
            I::point(self.deformation_band)
                .mul(norm1(self.hemisphere))
                .hi
        } else {
            0.0
        };
        let outside_height = if self.deformation_band > 0.0 {
            I::point(self.hemisphere_floor)
                .sub(I::point(angular_band))
                .sub(I::point(height_error))
                .lo
        } else {
            self.hemisphere_floor - angular_band
        };
        if height.hi < outside_height {
            return Some(false);
        }
        let inside_height = if self.deformation_band > 0.0 {
            I::point(-self.hemisphere_floor)
                .add(I::point(angular_band))
                .add(I::point(height_error))
                .hi
        } else {
            -self.hemisphere_floor + angular_band
        };
        if height.lo <= inside_height {
            return None;
        }
        // The antipode is outside the qualified hemisphere. Orthographic
        // projection into q's tangent plane therefore has winding+1 exactly
        // in the selected patch and0 outside. Each <π native subarc and its
        // projected endpoint connector have identical radial winding.
        let references = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(2.0, -3.0, 1.0),
            Vec3::new(-3.0, 1.0, 2.0),
        ];
        for reference in references {
            let r = vector(reference);
            let q2 = dot(q, q);
            let rq = dot(r, q);
            let y_axis = cross(q, r);
            let projected: Vec<(I, I)> = self
                .vertices
                .iter()
                .map(|a: &V| (q2.mul(dot(r, *a)).sub(rq.mul(dot(q, *a))), dot(y_axis, *a)))
                .collect();
            // No y=0 vertex or unresolved determinant is assigned an arbitrary
            // side. Change the algebraic reference axis; all choices represent
            // the same tangent plane and must give the same integer winding.
            if projected.iter().any(|(_, y)| y.lo <= 0.0 && y.hi >= 0.0) {
                continue;
            }
            let mut winding = 0;
            let mut resolved = true;
            for ((ax, ay), (bx, by)) in projected
                .iter()
                .copied()
                .zip(projected.iter().copied().cycle().skip(1))
            {
                if (ay.lo > 0.0) == (by.lo > 0.0) {
                    continue;
                }
                let determinant = ax.mul(by).sub(ay.mul(bx));
                if !determinant.finite() || (determinant.lo <= 0.0 && determinant.hi >= 0.0) {
                    resolved = false;
                    break;
                }
                if ay.hi < 0.0 && by.lo > 0.0 && determinant.lo > 0.0 {
                    winding += 1;
                }
                if ay.lo > 0.0 && by.hi < 0.0 && determinant.hi < 0.0 {
                    winding -= 1;
                }
            }
            if resolved {
                return if winding == 0 {
                    Some(false)
                } else if winding == 1 {
                    Some(true)
                } else {
                    None
                };
            }
        }
        None
    }
}

/// Qualified analytic spherical trim containment for ray and boundary queries.
/// `None` preserves the established path for unsupported or unresolved trims.
/// The boundary band never exceeds the caller's requested physical tolerance.
pub(super) fn qualified_sphere_trim_contains(
    topo: &Topology,
    face_id: FaceId,
    cache: Option<&FaceCache>,
    point: Point3,
    tolerance: f64,
) -> Result<Option<bool>, CheckError> {
    let FaceSurface::Sphere(sphere) = topo.face(face_id)?.surface() else {
        return Ok(None);
    };
    let owned;
    let trim = if let Some(cache) = cache {
        if cache.spherical_trim.get().is_none() {
            let _ = cache
                .spherical_trim
                .set(GreatCircleTrim::build(topo, face_id, sphere)?);
        }
        cache.spherical_trim.get().and_then(Option::as_ref)
    } else {
        owned = GreatCircleTrim::build(topo, face_id, sphere)?;
        owned.as_ref()
    };
    let Some(trim) = trim else {
        return Ok(None);
    };
    let result = trim.contains(point, sphere, tolerance);
    if trim.deformation_band > 0.0 && result.is_none() {
        // A qualified but unresolved perturbed support must not escape to the
        // sampled legacy trim and silently regain an unproved sign.
        return Err(CheckError::ClassificationFailed(
            "spherical trim support deformation is unresolved at query tolerance".into(),
        ));
    }
    Ok(result)
}

fn count_great_circle_trim_crossings(
    topo: &Topology,
    face_id: FaceId,
    cache: Option<&FaceCache>,
    hits: &[Point3],
) -> Result<Option<u32>, CheckError> {
    let mut count = 0;
    for &hit in hits {
        let Some(inside) =
            qualified_sphere_trim_contains(topo, face_id, cache, hit, HALF_SPACE_EPS)?
        else {
            return Ok(None);
        };
        count += u32::from(inside);
    }
    Ok(Some(count))
}

/// Count crossings for a sphere patch bounded entirely by exact circle arcs.
///
/// Each supporting circle plane contributes one exact half-space constraint.
/// The boundary traversal determines which side belongs to the face. This
/// handles the two-plane spherical patches produced by a transversal
/// sphere/sphere boolean without replacing their curved boundaries by an
/// inscribed polygon.
fn count_sphere_circle_patch_crossings(
    topo: &Topology,
    face_id: FaceId,
    cache: Option<&FaceCache>,
    hits: &[Point3],
    sphere: &remus_math::surfaces::SphericalSurface,
) -> Result<Option<u32>, CheckError> {
    let owned;
    let constraints = if let Some(cache) = cache {
        if let Some(constraints) = cache.circle_patch.get() {
            constraints
        } else {
            let constraints = sphere_circle_patch_constraints(topo, face_id, sphere)?;
            cache.circle_patch.get_or_init(|| constraints)
        }
    } else {
        owned = sphere_circle_patch_constraints(topo, face_id, sphere)?;
        &owned
    };
    let Some(constraints) = constraints else {
        return Ok(None);
    };
    let mut crossings = 0_u32;
    for &hit in hits {
        if constraints.iter().all(|(plane_point, half_normal)| {
            (hit - *plane_point).dot(*half_normal) >= -HALF_SPACE_EPS
        }) {
            crossings += 1;
        }
    }
    Ok(Some(crossings))
}

/// The two half-space supports of a sphere patch bounded by exactly two
/// circles, as [`count_sphere_circle_patch_crossings`] tests them; `None`
/// when the face is not such a patch.
fn sphere_circle_patch_constraints(
    topo: &Topology,
    face_id: FaceId,
    sphere: &remus_math::surfaces::SphericalSurface,
) -> Result<Option<Vec<(Point3, Vec3)>>, CheckError> {
    let face = topo.face(face_id)?;
    if !face.inner_wires().is_empty() {
        return Ok(None);
    }
    let wire = topo.wire(face.outer_wire())?;
    if wire.edges().len() < 2 {
        return Ok(None);
    }

    let mut constraints: Vec<(Point3, Vec3)> = Vec::with_capacity(wire.edges().len());
    for oriented in wire.edges() {
        let edge = topo.edge(oriented.edge())?;
        let EdgeCurve::Circle(circle) = edge.curve() else {
            return Ok(None);
        };
        let (t0, t1) = edge
            .strict_domain()
            .map_err(crate::error::edge_domain_validation)?;
        let start = topo.vertex(edge.start())?.point();
        let end = topo.vertex(edge.end())?.point();
        let mid_t = (t1 - t0).mul_add(0.5, t0);
        let midpoint = circle.evaluate(mid_t);
        let sphere_residual = ((midpoint - sphere.center()).length() - sphere.radius()).abs();
        let tolerance = edge.effective_tolerance(
            topo.vertex(edge.start())?
                .tolerance()
                .max(topo.vertex(edge.end())?.tolerance()),
        );
        if !sphere_residual.is_finite() || sphere_residual > tolerance.max(1e-9) {
            return Ok(None);
        }

        let mut tangent = edge.curve().tangent_with_endpoints(mid_t, start, end);
        if t1 < t0 {
            tangent = -tangent;
        }
        if !oriented.is_forward() {
            tangent = -tangent;
        }
        let Ok(outward) = (midpoint - sphere.center()).normalize() else {
            return Ok(None);
        };
        let plane_normal = circle.normal();
        let side = outward.cross(tangent).dot(plane_normal);
        if !side.is_finite() || side.abs() <= 1e-12 {
            return Ok(None);
        }
        let half_normal = plane_normal * side.signum();
        let mut duplicate = false;
        for (plane_point, existing_normal) in &constraints {
            let alignment = half_normal.dot(*existing_normal);
            if alignment.abs() > 1.0 - 1e-9
                && (circle.center() - *plane_point).dot(*existing_normal).abs()
                    <= tolerance.max(1e-9)
            {
                if alignment < 0.0 {
                    return Ok(None);
                }
                duplicate = true;
                break;
            }
        }
        if !duplicate {
            constraints.push((circle.center(), half_normal));
        }
    }

    // One support plane is the ordinary cap case handled above. Keep this
    // dispatch deliberately narrow: the Issue 2.2 patch is bounded by exactly
    // the primitive equator and the radical plane. Spherical polygons with
    // three or more supports may be concave; `sphere_arcs` decides those.
    if constraints.len() != 2 || constraints[0].1.cross(constraints[1].1).length() <= 1e-9 {
        return Ok(None);
    }

    Ok(Some(constraints))
}

/// Count crossings using 3D polygon containment (for faces with planar
/// boundaries, e.g. sphere hemispheres where UV projection has pole
/// singularities).
///
/// The polygon normal (from Newell's method) indicates which side of the
/// boundary plane the face extends into. A hit point must be on that side
/// AND project inside the boundary polygon.
///
/// # Errors
///
/// Returns an error if topology lookups fail.
/// Crossings of sphere hits decided by the face's circle-arc trim
/// ([`sphere_arcs`]); `None` when the face has another edge kind or some hit
/// stays undecided.
fn count_sphere_arc_trim_crossings(
    topo: &Topology,
    face_id: FaceId,
    cache: Option<&FaceCache>,
    hits: &[Point3],
    sphere: &remus_math::surfaces::SphericalSurface,
) -> Result<Option<u32>, CheckError> {
    if hits.is_empty() {
        return Ok(Some(0));
    }
    let owned;
    let trim = if let Some(cache) = cache {
        if let Some(trim) = cache.sphere_arc_trim.get() {
            trim
        } else {
            let trim = sphere_arcs::SphereArcTrim::build(topo, face_id, sphere)?;
            cache.sphere_arc_trim.get_or_init(|| trim)
        }
    } else {
        owned = sphere_arcs::SphereArcTrim::build(topo, face_id, sphere)?;
        &owned
    };
    let Some(trim) = trim else {
        return Ok(None);
    };
    let mut count = 0;
    for &hit in hits {
        let Some(inside) = trim.contains(hit) else {
            return Ok(None);
        };
        count += u32::from(inside);
    }
    Ok(Some(count))
}

fn count_3d_polygon_crossings(
    topo: &Topology,
    face_id: FaceId,
    hits: &[Point3],
) -> Result<u32, CheckError> {
    count_3d_polygon_crossings_with_trim(topo, face_id, None, hits)
}

/// [`count_3d_polygon_crossings`] with caller-supplied trim data (`None`
/// builds it on demand, exactly as the one-shot path always has).
fn count_3d_polygon_crossings_with_trim(
    topo: &Topology,
    face_id: FaceId,
    trim: Option<&FaceTrimData>,
    hits: &[Point3],
) -> Result<u32, CheckError> {
    if hits.is_empty() {
        return Ok(0);
    }

    let owned_trim;
    let trim_data = if let Some(cached) = trim {
        cached
    } else {
        owned_trim = FaceTrimData::build(topo, face_id)?;
        &owned_trim
    };
    let verts = &trim_data.outer;
    if verts.len() < 3 {
        return Ok(0);
    }
    let mut normal = polygon_normal(verts);
    // If the face is reversed, the surface normal is flipped.
    let face = topo.face(face_id)?;
    if face.is_reversed() {
        normal = -normal;
    }
    let ref_pt = verts[0];
    let holes = &trim_data.holes;

    let mut crossings = 0u32;
    for &hit in hits {
        // The hit must be on the face's side of the boundary plane.
        let side = (hit - ref_pt).dot(normal);
        if side < -HALF_SPACE_EPS {
            continue;
        }

        if point_in_polygon_3d(&hit, verts, &normal) && !hit_in_hole_3d(holes, hit, normal) {
            crossings += 1;
        }
    }

    Ok(crossings)
}

/// Count ray crossings for a single face, dispatching by surface type.
///
/// For plane faces, uses direct ray-plane + 3D polygon containment.
/// For analytic curved faces, uses ray-surface intersection + UV containment.
/// For sphere faces, uses 3D polygon containment (avoids UV pole singularity).
/// For NURBS faces, uses line-surface intersection.
///
/// # Errors
///
/// Returns an error if topology lookups or intersection computations fail.
#[allow(clippy::too_many_lines)]
pub fn count_face_ray_crossings(
    topo: &Topology,
    face_id: FaceId,
    origin: Point3,
    direction: Vec3,
) -> Result<u32, CheckError> {
    count_face_ray_crossings_with_trim(topo, face_id, None, origin, direction)
}

/// [`count_face_ray_crossings`] with caller-supplied trim data.
///
/// `trim` carries the prepared polygons; `None` builds them on demand exactly
/// as the one-shot path always has (preserving its early-outs and errors).
pub fn count_face_ray_crossings_with_trim(
    topo: &Topology,
    face_id: FaceId,
    trim: Option<&FaceTrimData>,
    origin: Point3,
    direction: Vec3,
) -> Result<u32, CheckError> {
    count_face_ray_crossings_cached(topo, face_id, trim, None, origin, direction)
}

/// [`count_face_ray_crossings_with_trim`] with an optional per-face cache
/// from a prepared context.
///
/// Every cached value equals what `None` recomputes, so the count is
/// identical.
///
/// # Errors
///
/// Returns an error if topology lookups or intersection computations fail.
pub fn count_face_ray_crossings_cached(
    topo: &Topology,
    face_id: FaceId,
    trim: Option<&FaceTrimData>,
    cache: Option<&FaceCache>,
    origin: Point3,
    direction: Vec3,
) -> Result<u32, CheckError> {
    let face = topo.face(face_id)?;
    match face.surface() {
        FaceSurface::Plane { normal, d } => ray_plane_crossings_with_trim(
            topo, face_id, trim, cache, origin, direction, *normal, *d,
        ),
        FaceSurface::Cylinder(cyl) => {
            let cyl = cyl.clone();
            let roots = ray_surface::ray_cylinder(origin, direction, &cyl);
            count_analytic_crossings_with_trim(
                topo,
                face_id,
                trim,
                cache,
                &ray_hit_points(origin, direction, &roots),
                |p| cyl.project_point(p),
                false,
                None,
            )
        }
        FaceSurface::Cone(cone) => {
            let cone = cone.clone();
            let roots = ray_surface::ray_cone(origin, direction, &cone);
            count_analytic_crossings_with_trim(
                topo,
                face_id,
                trim,
                cache,
                &ray_hit_points(origin, direction, &roots),
                |p| cone.project_point(p),
                false,
                Some(CONE_APEX_V),
            )
        }
        FaceSurface::Sphere(sph) => {
            // A spherical cap's boundary is planar, and the plane cuts the
            // sphere in a circle — so the exact trim is a half-space test
            // against that plane, with no polygon involved. Take it when the
            // boundary really is planar; fall back to the chorded polygon for a
            // lune or a boolean-made spherical triangle, whose boundary is not.
            let sph = sph.clone();
            let roots = ray_surface::ray_sphere(origin, direction, &sph);
            if let Some(count) = count_sphere_cap_crossings_with_trim(
                topo,
                face_id,
                trim,
                &ray_hit_points(origin, direction, &roots),
                &sph,
            )? {
                Ok(count)
            } else if let Some(count) = count_great_circle_trim_crossings(
                topo,
                face_id,
                cache,
                &ray_hit_points(origin, direction, &roots),
            )? {
                Ok(count)
            } else if let Some(count) = count_sphere_circle_patch_crossings(
                topo,
                face_id,
                cache,
                &ray_hit_points(origin, direction, &roots),
                &sph,
            )? {
                Ok(count)
            } else if let Some(count) = count_sphere_arc_trim_crossings(
                topo,
                face_id,
                cache,
                &ray_hit_points(origin, direction, &roots),
                &sph,
            )? {
                Ok(count)
            } else {
                count_3d_polygon_crossings_with_trim(
                    topo,
                    face_id,
                    trim,
                    &ray_hit_points(origin, direction, &roots),
                )
            }
        }
        FaceSurface::Torus(tor) => {
            let tor = tor.clone();
            let roots = ray_surface::ray_torus(origin, direction, &tor);
            count_analytic_crossings_with_trim(
                topo,
                face_id,
                trim,
                cache,
                &ray_hit_points(origin, direction, &roots),
                |p| tor.project_point(p),
                true,
                None,
            )
        }
        FaceSurface::Nurbs(surface) => {
            ray_crossings_nurbs_with_trim(topo, face_id, trim, cache, origin, direction, surface)
        }
    }
}

/// Whether a planar face has a boundary edge its trim polygon only
/// approximates: anything but a straight line.
fn planar_face_has_curved_edge(topo: &Topology, face_id: FaceId) -> Result<bool, CheckError> {
    let face = topo.face(face_id)?;
    for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oriented in topo.wire(wire)?.edges() {
            if !crate::distance::edge::is_linear_curve(topo.edge(oriented.edge())?.curve()) {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// The exact trim of a planar face with a curved edge, `None` for a
/// straight-edged face, whose polygon is already exact.
fn exact_plane_trim_for(
    topo: &Topology,
    face_id: FaceId,
    normal: Vec3,
) -> Result<Option<PlanarTrim>, CheckError> {
    if planar_face_has_curved_edge(topo, face_id)? {
        PlanarTrim::build(topo, face_id, normal).map(Some)
    } else {
        Ok(None)
    }
}

/// Exact trim membership of a plane hit on a face bounded by lines and
/// circles (RFC 0008 F1). The chorded polygon's inradius falls short of a
/// circular rim by its sagitta, a band no graze margin covers, so a ray
/// through that band would miss the cap while crossing the exact wall.
///
/// `None` when the face is straight-edged or the exact test declines (another
/// curve kind, or an event it cannot resolve); the caller then keeps the
/// polygon.
fn exact_plane_trim(
    topo: &Topology,
    face_id: FaceId,
    cache: Option<&FaceCache>,
    hit: Point3,
    normal: Vec3,
) -> Result<Option<bool>, CheckError> {
    let contains = |trim: &PlanarTrim| trim.contains(hit).ok();
    Ok(match cache {
        Some(cache) => {
            let trim = if let Some(trim) = cache.plane_trim.get() {
                trim
            } else {
                let trim = exact_plane_trim_for(topo, face_id, normal)?;
                cache.plane_trim.get_or_init(|| trim)
            };
            trim.as_ref().and_then(contains)
        }
        None => exact_plane_trim_for(topo, face_id, normal)?
            .as_ref()
            .and_then(contains),
    })
}

/// Ray-plane intersection with point-in-polygon boundary test.
///
/// Takes caller-supplied trim data. `None` rebuilds it on demand in the
/// pre-refactor lazy order: the outer polygon first, hole polygons only when
/// the hit survives outer containment. A face with circular edges is decided
/// exactly first ([`exact_plane_trim`]).
#[allow(clippy::too_many_arguments)]
fn ray_plane_crossings_with_trim(
    topo: &Topology,
    face_id: FaceId,
    trim: Option<&FaceTrimData>,
    cache: Option<&FaceCache>,
    origin: Point3,
    direction: Vec3,
    normal: Vec3,
    d: f64,
) -> Result<u32, CheckError> {
    let t = match ray_surface::ray_plane(origin, direction, normal, d) {
        Some(t) => t,
        None => return Ok(0),
    };

    let hit = origin + direction * t;
    if let Some(inside) = exact_plane_trim(topo, face_id, cache, hit, normal)? {
        return Ok(u32::from(inside));
    }
    if let Some(cached) = trim {
        return Ok(ray_plane_crossings_with_cached(hit, cached, normal));
    }
    // One-shot order: holes sample only on an outer hit, so rejected
    // candidates within a wide face AABB never pay for hole edges.
    let outer = FaceTrimData::build_outer(topo, face_id)?;
    if outer.len() < 3 {
        return Ok(0);
    }
    if !point_in_polygon_3d(&hit, &outer, &normal) {
        return Ok(0);
    }
    // A ray through a hole (bolt hole, absorbed hub circle) passes through
    // empty space, not material — the face contributes no crossing there.
    let holes = FaceTrimData::build_holes(topo, face_id)?;
    if hit_in_hole_3d(&holes, hit, normal) {
        return Ok(0);
    }
    Ok(1)
}

/// [`ray_plane_crossings_with_trim`] over already-prepared trim data.
fn ray_plane_crossings_with_cached(hit: Point3, trim: &FaceTrimData, normal: Vec3) -> u32 {
    let verts = &trim.outer;
    if verts.len() < 3 {
        return 0;
    }

    if !point_in_polygon_3d(&hit, verts, &normal) {
        return 0;
    }
    // A ray through a hole (bolt hole, absorbed hub circle) passes through
    // empty space, not material — the face contributes no crossing there.
    let holes = &trim.holes;
    if hit_in_hole_3d(holes, hit, normal) {
        return 0;
    }
    1
}

/// Count ray crossings for a NURBS face using ray-surface intersection.
///
/// Takes caller-supplied trim data (`None` builds it on demand, exactly as
/// the one-shot path always has).
fn ray_crossings_nurbs_with_trim(
    topo: &Topology,
    face_id: FaceId,
    trim: Option<&FaceTrimData>,
    cache: Option<&FaceCache>,
    origin: Point3,
    direction: Vec3,
    surface: &NurbsSurface,
) -> Result<u32, CheckError> {
    let hits = match cache {
        Some(cache) => {
            ray_surface::ray_nurbs_with_grid(origin, direction, surface, cache.line_grid(surface))?
        }
        None => ray_surface::ray_nurbs(origin, direction, surface, NURBS_RAY_SAMPLES)?,
    };
    if hits.is_empty() {
        return Ok(0);
    }

    let owned_trim;
    let trim_data = if let Some(cached) = trim {
        cached
    } else {
        owned_trim = FaceTrimData::build(topo, face_id)?;
        &owned_trim
    };
    let points: Vec<_> = hits
        .iter()
        .map(|&(t, u, v)| (origin + direction * t, u, v))
        .collect();
    // A near-tangent intersection is a valid math contact, but its parity
    // is unresolved at the root's residual floor. Abstain from this ray,
    // retaining all mathematical hits and letting another direction decide.
    for &(point, u, v) in &points {
        if nurbs_contact_parity_uncertain(surface, point, direction, u, v)
            && count_nurbs_hits_with_trim(
                topo,
                face_id,
                Some(trim_data),
                cache,
                surface,
                &[(point, u, v)],
            )? > 0
        {
            return Err(CheckError::ClassificationFailed(
                super::NURBS_RAY_GRAZE.into(),
            ));
        }
    }
    count_nurbs_hits_with_trim(topo, face_id, Some(trim_data), cache, surface, &points)
}

/// Whether the root's positional rounding can hide a stationary contact.
/// Scale by local curvature rather than a fixed angle: a shallow crossing
/// of a flat patch remains usable, while a tightly curved tangent root may
/// have a noticeable derivative merely because its parameters round.
fn nurbs_contact_parity_uncertain(
    surface: &NurbsSurface,
    point: Point3,
    direction: Vec3,
    u: f64,
    v: f64,
) -> bool {
    use remus_math::traits::ParametricSurface;
    let (su, sv) = (surface.partial_u(u, v), surface.partial_v(u, v));
    let normal = su.cross(sv);
    let norm = normal.length();
    let speed = direction.length();
    if norm == 0.0 || speed == 0.0 {
        return true;
    }
    let normal = normal * (1.0 / norm);
    let direction = direction * (1.0 / speed);
    let slope = normal.dot(direction).abs();
    if slope >= 1e-3 {
        return false;
    }
    let (e, f, g) = (su.dot(su), su.dot(sv), sv.dot(sv));
    let determinant = e * g - f * f;
    if determinant <= 0.0 || !determinant.is_finite() {
        return true;
    }
    let (a, b) = (su.dot(direction), sv.dot(direction));
    let (du, dv) = ((g * a - f * b) / determinant, (e * b - f * a) / determinant);
    let derivatives = surface.derivatives(u, v, 2);
    let acceleration = derivatives[2][0] * (du * du)
        + derivatives[1][1] * (2.0 * du * dv)
        + derivatives[0][2] * (dv * dv);
    let curvature = normal.dot(acceleration).abs();
    let magnitude = point.0.iter().fold(1.0_f64, |m, x| m.max(x.abs()));
    let rounding = 256.0 * f64::EPSILON;
    slope * slope <= 4.0 * rounding * magnitude * curvature + rounding * rounding
}

/// Certify a complete NURBS cap bounded by one planar periodic rim.
///
/// The rim is a full `u` turn on one `v` domain edge; positive rational
/// weights and a one-sided control net prove that the whole support patch
/// lies on the returned side of its rim plane. Its parameter domain is the
/// cap, so a ray hit needs the half-space test without an inscribed polygon.
fn full_nurbs_cap_inward_normal(
    surface: &remus_math::nurbs::surface::NurbsSurface,
    trim: &FaceTrimData,
    uv_boundary: &[(f64, f64)],
    u_period: Option<f64>,
    v_period: Option<f64>,
) -> Option<Vec3> {
    if !trim.holes.is_empty() || u_period.is_none() || v_period.is_some() {
        return None;
    }
    if !uv_loop_wraps(uv_boundary, u_period, v_period) {
        return None;
    }
    let (v0, v1) = surface.domain_v();
    let v_tol = (v1 - v0).abs() * 1e-9;
    if ![v0, v1]
        .iter()
        .any(|&edge| uv_boundary.iter().all(|&(_, v)| (v - edge).abs() <= v_tol))
    {
        return None;
    }
    if !surface
        .weights()
        .iter()
        .flatten()
        .all(|w| w.is_finite() && *w > 0.0)
    {
        return None;
    }
    let (ref_pt, rest) = trim.outer.split_first()?;
    let extent = rest
        .iter()
        .map(|point| (*point - *ref_pt).length())
        .fold(0.0_f64, f64::max);
    let tol = extent.max(1e-12) * 1e-9;
    let area = trim
        .outer
        .iter()
        .zip(trim.outer.iter().cycle().skip(1))
        .map(|(a, b)| (*a - *ref_pt).cross(*b - *ref_pt))
        .fold(Vec3::new(0.0, 0.0, 0.0), |sum, cross| sum + cross);
    if area.length() <= tol * extent {
        return None;
    }
    let normal = area.normalize().ok()?;
    if trim
        .outer
        .iter()
        .any(|point| ((*point - *ref_pt).dot(normal)).abs() > tol)
    {
        return None;
    }
    let mut positive = false;
    let mut negative = false;
    for point in surface.control_points().iter().flatten() {
        let side = (*point - *ref_pt).dot(normal);
        positive |= side > tol;
        negative |= side < -tol;
    }
    match (positive, negative) {
        (true, false) => Some(normal),
        (false, true) => Some(-normal),
        _ => None,
    }
}

/// UV-trimmed NURBS hit counting with caller-supplied trim data (`None`
/// builds it on demand, exactly as the one-shot path always has) and an
/// optional prepared cache for the trim's UV images.
fn count_nurbs_hits_with_trim(
    topo: &Topology,
    face_id: FaceId,
    trim: Option<&FaceTrimData>,
    cache: Option<&FaceCache>,
    surface: &NurbsSurface,
    hits: &[(Point3, f64, f64)],
) -> Result<u32, CheckError> {
    let owned_trim;
    let trim_data = if let Some(cached) = trim {
        cached
    } else {
        owned_trim = FaceTrimData::build(topo, face_id)?;
        &owned_trim
    };
    let owned_uv;
    let uv = if let Some(cache) = cache {
        cache.uv_trim.get_or_init(|| {
            NurbsUvTrim::build(surface, trim_data, Some(cache.projection_grid(surface)))
        })
    } else {
        owned_uv = NurbsUvTrim::build(surface, trim_data, None);
        &owned_uv
    };
    let (u_period, v_period) = (uv.u_period, uv.v_period);
    let uv_boundary = &uv.uv_boundary;

    // A boundary that encloses no UV area does not bound a patch -- it splits
    // the surface, and which half this face takes is carried by the WINDING of
    // its boundary, not by anything in UV. `count_3d_polygon_crossings` reads
    // exactly that (the two hemispheres of a sphere traverse their shared
    // equatorial wire in opposite senses), and it needs no surface type, so it
    // serves any NURBS face of this shape.
    //
    // It tests a half-space, so it assumes the degenerate boundary is planar in
    // 3D. That holds for the case this addresses -- an equator -- and matches
    // what the analytic sphere path already assumes. A non-planar zero-area
    // boundary would not be handled correctly here, though it is no worse off
    // than under the UV test, which credits it with no crossings at all.
    // A certified full NURBS cap uses its exact half-space before this fallback.
    if let Some(boundary) = uv_boundary
        && uv.degenerate
    {
        if let Some(inward) =
            full_nurbs_cap_inward_normal(surface, trim_data, boundary, u_period, v_period)
        {
            let ref_pt = trim_data.outer[0];
            let crossings = hits
                .iter()
                .filter(|(point, _, _)| (*point - ref_pt).dot(inward) >= -HALF_SPACE_EPS)
                .count();
            return Ok(u32::try_from(crossings).unwrap_or(u32::MAX));
        }
        let points: Vec<_> = hits.iter().map(|(point, _, _)| *point).collect();
        return count_3d_polygon_crossings_with_trim(topo, face_id, Some(trim_data), &points);
    }

    let holes = &uv.holes;

    let mut crossings = 0u32;
    for (_, hit_u, hit_v) in hits {
        if let Some(boundary) = uv_boundary
            && !point_in_uv_boundary(*hit_u, *hit_v, boundary, u_period, v_period)
        {
            continue;
        }
        if hit_in_hole_uv(holes, *hit_u, *hit_v, u_period, v_period) {
            continue;
        }
        crossings += 1;
    }

    Ok(crossings)
}

fn ray_hit_points(origin: Point3, direction: Vec3, roots: &[f64]) -> SmallVec<[Point3; 4]> {
    roots
        .iter()
        .filter(|&&t| t > RAY_T_MIN)
        .map(|&t| origin + direction * t)
        .collect()
}

/// Test whether a point on a face's support surface lies within its trims.
///
/// The caller must first project onto the support surface. Uses the same
/// sampled UV boundaries and spherical cap tests as ray classification.
///
/// # Errors
///
/// Returns an error if a boundary entity or curve domain is invalid.
pub fn surface_point_in_face(
    topo: &Topology,
    face_id: FaceId,
    point: Point3,
) -> Result<bool, CheckError> {
    let hits = [point];
    let count = match topo.face(face_id)?.surface() {
        FaceSurface::Plane { normal, .. } => {
            let trim = FaceTrimData::build(topo, face_id)?;
            return Ok(point_in_polygon_3d(&point, &trim.outer, normal)
                && !hit_in_hole_3d(&trim.holes, point, *normal));
        }
        FaceSurface::Cylinder(surface) => count_analytic_crossings(
            topo,
            face_id,
            &hits,
            |p| surface.project_point(p),
            false,
            None,
        )?,
        FaceSurface::Cone(surface) => count_analytic_crossings(
            topo,
            face_id,
            &hits,
            |p| surface.project_point(p),
            false,
            Some(CONE_APEX_V),
        )?,
        FaceSurface::Torus(surface) => count_analytic_crossings(
            topo,
            face_id,
            &hits,
            |p| surface.project_point(p),
            true,
            None,
        )?,
        FaceSurface::Sphere(surface) => {
            if let Some(count) = count_sphere_cap_crossings(topo, face_id, &hits, surface)? {
                count
            } else if let Some(count) =
                count_great_circle_trim_crossings(topo, face_id, None, &hits)?
            {
                count
            } else if let Some(count) =
                count_sphere_circle_patch_crossings(topo, face_id, None, &hits, surface)?
            {
                count
            } else if let Some(count) =
                count_sphere_arc_trim_crossings(topo, face_id, None, &hits, surface)?
            {
                count
            } else {
                count_3d_polygon_crossings(topo, face_id, &hits)?
            }
        }
        FaceSurface::Nurbs(surface) => {
            let (u, v) = surface.project_point(point);
            count_nurbs_hits_with_trim(topo, face_id, None, None, surface, &[(point, u, v)])?
        }
    };
    Ok(count > 0)
}

#[cfg(test)]
mod nurbs_bump_ray_tests;

#[cfg(test)]
mod spherical_triangle_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use super::*;
    use remus_math::curves::Circle3D;
    use remus_math::surfaces::SphericalSurface;
    use remus_topology::edge::Edge;
    use remus_topology::face::Face;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};
    use std::f64::consts::{FRAC_PI_2, PI, TAU};

    fn rotate(v: Vec3, placed: bool) -> Vec3 {
        if !placed {
            return v;
        }
        let axis = Vec3::new(1.0, 2.0, 3.0).normalize().unwrap();
        let (s, c) = 0.37_f64.sin_cos();
        v * c + axis.cross(v) * s + axis * (axis.dot(v) * (1.0 - c))
    }

    fn concave_upper_patch(scale: f64, center: Point3, placed: bool) -> (Topology, FaceId) {
        let mut topo = Topology::new();
        let a = topo.add_vertex(Vertex::new(
            center + rotate(Vec3::new(scale, 0.0, 0.0), placed),
            1e-7,
        ));
        let b = topo.add_vertex(Vertex::new(
            center + rotate(Vec3::new(0.0, scale, 0.0), placed),
            1e-7,
        ));
        let pole = topo.add_vertex(Vertex::new(
            center + rotate(Vec3::new(0.0, 0.0, scale), placed),
            1e-7,
        ));
        let specifications = [
            (
                b,
                a,
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
                FRAC_PI_2,
                TAU,
            ),
            (
                a,
                pole,
                Vec3::new(0.0, -1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                1.5 * PI,
                TAU,
            ),
            (
                pole,
                b,
                Vec3::new(-1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, -1.0),
                PI,
                1.5 * PI,
            ),
        ];
        let mut uses = Vec::new();
        for (start, end, normal, reference, t0, t1) in specifications {
            let curve = Circle3D::new_with_ref(
                center,
                rotate(normal, placed),
                scale,
                rotate(reference, placed),
            )
            .unwrap();
            let mut edge = Edge::with_tolerance(start, end, EdgeCurve::Circle(curve), Some(1e-7));
            edge.set_trim(Some((t0, t1)));
            uses.push(OrientedEdge::new(topo.add_edge(edge), true));
        }
        let wire = topo.add_wire(Wire::new(uses, true).unwrap());
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Sphere(SphericalSurface::new(center, scale).unwrap()),
        ));
        (topo, face)
    }

    #[test]
    fn concave_three_support_sphere_trim_has_cartesian_oracle() {
        for scale in [1.0, 1000.0] {
            for center in [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(3.25 * scale, -2.75 * scale, 4.5 * scale),
            ] {
                let placed = center.x().abs() > 0.0;
                let (topo, face) = concave_upper_patch(scale, center, placed);
                for x in [-0.8_f64, -0.3, 0.3, 0.8] {
                    for y in [-0.8_f64, -0.3, 0.3, 0.8] {
                        for z in [-0.8_f64, -0.3, 0.3, 0.8] {
                            let direction = Vec3::new(x, y, z).normalize().unwrap();
                            let point = center + rotate(direction, placed) * scale;
                            let expected = z > 0.0 && (x < 0.0 || y < 0.0);
                            assert_eq!(
                                surface_point_in_face(&topo, face, point).unwrap(),
                                expected,
                                "scale={scale} center={center:?} direction={direction:?}"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn cached_ray_trim_matches_independent_upper_patch_oracle() {
        for scale in [1.0, 1000.0] {
            let center = Point3::new(3.25 * scale, -2.75 * scale, 4.5 * scale);
            let (topo, face) = concave_upper_patch(scale, center, true);
            let cache = FaceCache::default();
            let trim = FaceTrimData::build(&topo, face).unwrap();
            for x in [-0.8, 0.3] {
                for y in [-0.3, 0.8] {
                    let q = rotate(Vec3::new(x, y, 0.6).normalize().unwrap(), true);
                    let origin = center + q * (2.0 * scale);
                    let expected = u32::from(x < 0.0 || y < 0.0);
                    assert_eq!(
                        count_face_ray_crossings(&topo, face, origin, -q).unwrap(),
                        expected
                    );
                    assert_eq!(
                        count_face_ray_crossings_cached(
                            &topo,
                            face,
                            Some(&trim),
                            Some(&cache),
                            origin,
                            -q
                        )
                        .unwrap(),
                        expected
                    );
                }
            }
            assert!(cache.spherical_trim.get().is_some_and(Option::is_some));
        }
    }

    #[test]
    fn stored_large_pose_circle_offset_keeps_certified_trim() {
        let center = Point3::new(10000.0, -7000.0, 3000.0);
        let (mut topo, face) = concave_upper_patch(1000.0, center, true);
        let edge_id = topo
            .wire(topo.face(face).unwrap().outer_wire())
            .unwrap()
            .edges()[2]
            .edge();
        let EdgeCurve::Circle(circle) = topo.edge(edge_id).unwrap().curve() else {
            panic!("circle fixture");
        };
        let shifted = Circle3D::new_with_ref(
            Point3::new(center.x().next_up(), center.y(), center.z()),
            circle.normal(),
            circle.radius(),
            circle.u_axis(),
        )
        .unwrap();
        let range = topo.edge(edge_id).unwrap().strict_domain().unwrap();
        let edge = topo.edge_mut(edge_id).unwrap();
        edge.set_curve(EdgeCurve::Circle(shifted));
        edge.set_trim(Some(range));
        let sphere = SphericalSurface::new(center, 1000.0).unwrap();
        assert!(
            GreatCircleTrim::build(&topo, face, &sphere)
                .unwrap()
                .is_some()
        );
        let cache = FaceCache::default();
        for x in [-0.8, 0.3] {
            for y in [-0.3, 0.8] {
                for z in [-0.6, 0.6] {
                    let q = rotate(Vec3::new(x, y, z).normalize().unwrap(), true);
                    assert_eq!(
                        qualified_sphere_trim_contains(
                            &topo,
                            face,
                            Some(&cache),
                            center + q * 1000.0,
                            HALF_SPACE_EPS
                        )
                        .unwrap(),
                        Some(z > 0.0 && (x < 0.0 || y < 0.0))
                    );
                }
            }
        }
        assert!(cache.spherical_trim.get().is_some_and(Option::is_some));
    }

    #[test]
    fn support_error_never_expands_finer_query_tolerance() {
        let center = Point3::new(0.0, 0.0, 0.0);
        let (mut topo, face) = concave_upper_patch(1.0, center, false);
        let edge_id = topo
            .wire(topo.face(face).unwrap().outer_wire())
            .unwrap()
            .edges()[2]
            .edge();
        let EdgeCurve::Circle(original) = topo.edge(edge_id).unwrap().curve() else {
            panic!("circle fixture");
        };
        let original = original.clone();
        let range = topo.edge(edge_id).unwrap().strict_domain().unwrap();
        for (offset, qualified) in [(1.818_989_403_545_856_5e-12, true), (1e-9, false)] {
            let curve = Circle3D::new_with_ref(
                Point3::new(offset, 0.0, 0.0),
                original.normal(),
                1.0,
                original.u_axis(),
            )
            .unwrap();
            let edge = topo.edge_mut(edge_id).unwrap();
            edge.set_curve(EdgeCurve::Circle(curve));
            edge.set_trim(Some(range));
            let sphere = SphericalSurface::new(center, 1.0).unwrap();
            assert_eq!(
                GreatCircleTrim::build(&topo, face, &sphere)
                    .unwrap()
                    .is_some(),
                qualified
            );
            if qualified {
                let point = Point3::new(0.0, 0.6, 0.8);
                assert_eq!(
                    qualified_sphere_trim_contains(&topo, face, None, point, HALF_SPACE_EPS)
                        .unwrap(),
                    Some(true)
                );
                assert!(matches!(
                    qualified_sphere_trim_contains(&topo, face, None, point, 1e-13),
                    Err(CheckError::ClassificationFailed(_))
                ));
                // A much finer caller tolerance still permits an independently
                // separated query; uncertainty does not blanket-reject a face.
                assert_eq!(
                    qualified_sphere_trim_contains(
                        &topo,
                        face,
                        None,
                        Point3::new(-0.6, -0.3, 0.55_f64.sqrt()),
                        1e-13
                    )
                    .unwrap(),
                    Some(true)
                );
            }
        }
    }

    #[test]
    fn tiny_sphere_support_error_and_malformed_dimensionless_frame_decline() {
        let center = Point3::new(0.0, 0.0, 0.0);
        let radius = 1e-15;
        let (mut topo, face) = concave_upper_patch(radius, center, false);
        let edge_id = topo
            .wire(topo.face(face).unwrap().outer_wire())
            .unwrap()
            .edges()[2]
            .edge();
        let original = topo.edge(edge_id).unwrap().clone();
        let EdgeCurve::Circle(circle) = original.curve() else {
            panic!("circle fixture");
        };
        let range = original.strict_domain().unwrap();
        let sphere = SphericalSurface::new(center, radius).unwrap();
        // This absolute displacement fits the existing physical band, but it
        // cannot qualify the radial-projection estimate at this tiny radius.
        let offset = Circle3D::new_with_ref(
            Point3::new(0.5 * radius, 0.0, 0.0),
            circle.normal(),
            radius,
            circle.u_axis(),
        )
        .unwrap();
        let edge = topo.edge_mut(edge_id).unwrap();
        edge.set_curve(EdgeCurve::Circle(offset));
        edge.set_trim(Some(range));
        assert!(
            GreatCircleTrim::build(&topo, face, &sphere)
                .unwrap()
                .is_none()
        );
        // Exact centers alone also cannot qualify a dimensionlessly malformed
        // stored frame merely because its physical Gram error is tiny.
        let malformed = Circle3D::with_axes(
            center,
            circle.normal(),
            radius,
            circle.u_axis() * 0.1,
            circle.v_axis() * 0.1,
        )
        .unwrap();
        let edge = topo.edge_mut(edge_id).unwrap();
        edge.set_curve(EdgeCurve::Circle(malformed));
        edge.set_trim(Some(range));
        assert!(
            GreatCircleTrim::build(&topo, face, &sphere)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn sphere_boundary_queries_preserve_requested_tolerance() {
        use remus_topology::{shell::Shell, solid::Solid};
        let (mut topo, upper) = concave_upper_patch(1.0, Point3::new(0.0, 0.0, 0.0), false);
        let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.0).unwrap();
        let circle = Circle3D::new_with_ref(
            sphere.center(),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let seam = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
        let mut edge = Edge::new(seam, seam, EdgeCurve::Circle(circle));
        edge.set_trim(Some((0.0, TAU)));
        let edge = topo.add_edge(edge);
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, false)], true).unwrap());
        let lower = topo.add_face(Face::new(wire, vec![], FaceSurface::Sphere(sphere)));
        // These two faces isolate spherical trim queries. The operation-level
        // regression supplies the three planar wound faces and a closed solid.
        let shell = topo.add_shell(Shell::new(vec![upper, lower]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        let prepared = crate::classify::PreparedSolid::prepare(&topo, solid).unwrap();
        let mut cache = crate::classify::cache::ClassificationCache::new();
        for (direction, tolerance, expected) in [
            (Vec3::new(1.0, 1.0, 1.0), 1e-7, false),
            (Vec3::new(-1.0, 1.0, 1.0), 1e-7, true),
            (Vec3::new(1.0, 1.0, -1.0), 1e-7, true),
            (Vec3::new(1e-11, 0.6, 0.8), 1e-13, false),
        ] {
            let q = direction.normalize().unwrap();
            let point = Point3::new(q.x(), q.y(), q.z());
            assert_eq!(
                crate::classify::is_point_on_boundary(&topo, solid, point, tolerance).unwrap(),
                expected
            );
            assert_eq!(
                prepared.is_point_on_boundary(point, tolerance).unwrap(),
                expected
            );
            let options = crate::classify::ClassifyOptions {
                tolerance,
                ..crate::classify::ClassifyOptions::default()
            };
            let expected = if expected {
                crate::classify::PointClassification::OnBoundary
            } else {
                crate::classify::PointClassification::Outside
            };
            assert_eq!(
                crate::classify::classify_point(&topo, solid, point, &options).unwrap(),
                expected
            );
            assert_eq!(prepared.classify_point(point, &options).unwrap(), expected);
            assert_eq!(
                cache.classify_point(&topo, solid, point, &options).unwrap(),
                expected
            );
        }
        assert_eq!(cache.stats().misses, 1);
        assert!(cache.stats().hits >= 3);
    }

    #[test]
    fn cyclic_wire_start_and_descending_native_domains_keep_major_arc() {
        let (mut topo, face) = concave_upper_patch(1.0, Point3::new(0.0, 0.0, 0.0), false);
        let wire = topo.face(face).unwrap().outer_wire();
        let old = topo.wire(wire).unwrap().edges().to_vec();
        let rim = topo.edge(old[0].edge()).unwrap().clone();
        let EdgeCurve::Circle(circle) = rim.curve() else {
            panic!("circle fixture");
        };
        let middle = topo.add_vertex(Vertex::new(circle.evaluate(PI), 1e-7));
        let mut head = rim.clone();
        head.set_end(middle);
        head.set_trim(Some((FRAC_PI_2, PI)));
        let mut tail = rim;
        tail.set_start(middle);
        tail.set_trim(Some((PI, TAU)));
        let head = topo.add_edge(head);
        let tail = topo.add_edge(tail);
        let shifted = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(tail, true),
                    old[1],
                    old[2],
                    OrientedEdge::new(head, true),
                ],
                true,
            )
            .unwrap(),
        );
        topo.set_face_boundary_wires(face, shifted, vec![]).unwrap();
        assert_eq!(
            qualified_sphere_trim_contains(
                &topo,
                face,
                None,
                Point3::new(-0.6, 0.0, 0.8),
                HALF_SPACE_EPS
            )
            .unwrap(),
            Some(true)
        );
        // Reverse each edge's own native range and vertex order, together with
        // its coedge, preserving the physical directed loop.
        let uses = topo.wire(shifted).unwrap().edges().to_vec();
        let mut reversed_uses = Vec::with_capacity(uses.len());
        for usage in &uses {
            let edge = topo.edge_mut(usage.edge()).unwrap();
            let (from, to) = edge.strict_domain().unwrap();
            let start = edge.start();
            let end = edge.end();
            edge.set_start(end);
            edge.set_end(start);
            edge.set_trim(Some((to, from)));
            reversed_uses.push(OrientedEdge::new(usage.edge(), false));
        }
        let reversed_wire = topo.add_wire(Wire::new(reversed_uses, true).unwrap());
        topo.set_face_boundary_wires(face, reversed_wire, vec![])
            .unwrap();
        assert_eq!(
            qualified_sphere_trim_contains(
                &topo,
                face,
                None,
                Point3::new(-0.6, 0.0, 0.8),
                HALF_SPACE_EPS
            )
            .unwrap(),
            Some(true)
        );
        assert_eq!(
            qualified_sphere_trim_contains(
                &topo,
                face,
                None,
                Point3::new(0.6, 0.0, -0.8),
                HALF_SPACE_EPS
            )
            .unwrap(),
            Some(false)
        );
    }

    #[test]
    fn unsupported_or_broken_spherical_boundaries_decline() {
        let center = Point3::new(0.0, 0.0, 0.0);
        let (mut topo, face) = concave_upper_patch(1.0, center, false);
        let edge = topo
            .wire(topo.face(face).unwrap().outer_wire())
            .unwrap()
            .edges()[0]
            .edge();
        let original = topo.edge(edge).unwrap().clone();
        topo.edge_mut(edge).unwrap().set_curve(EdgeCurve::Line);
        assert!(
            GreatCircleTrim::build(&topo, face, &SphericalSurface::new(center, 1.0).unwrap())
                .unwrap()
                .is_none()
        );
        *topo.edge_mut(edge).unwrap() = original.clone();
        let duplicate = topo.add_vertex(Vertex::new(
            topo.vertex(original.end()).unwrap().point(),
            1e-7,
        ));
        topo.edge_mut(edge).unwrap().set_end(duplicate);
        assert!(
            GreatCircleTrim::build(&topo, face, &SphericalSurface::new(center, 1.0).unwrap())
                .unwrap()
                .is_none()
        );
        *topo.edge_mut(edge).unwrap() = original;
        let curve = Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.01),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        topo.edge_mut(edge)
            .unwrap()
            .set_curve(EdgeCurve::Circle(curve));
        assert!(
            GreatCircleTrim::build(&topo, face, &SphericalSurface::new(center, 1.0).unwrap())
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn quadrant_interval_trigonometry_and_unknowns_are_conservative() {
        use spherical_trim_math::{I, sin_cos};
        for t in [-2.0 * TAU, -PI, -FRAC_PI_2, 0.0, FRAC_PI_2, PI, TAU, 13.0] {
            let (s, c) = sin_cos(t).unwrap();
            assert!(s.lo <= t.sin() && t.sin() <= s.hi);
            assert!(c.lo <= t.cos() && t.cos() <= c.hi);
        }
        assert!(sin_cos(f64::NAN).is_none());
        assert!(sin_cos(1000.0).is_none());
        assert!(!I::point(f64::NAN).add(I::point(1.0)).finite());
        assert!(!I { lo: 2.0, hi: 1.0 }.mul(I::point(1.0)).finite());
    }
}
