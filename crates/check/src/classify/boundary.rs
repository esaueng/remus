//! UV boundary polygon construction and containment tests for face trimming.
//!
//! Provides the core algorithms for determining whether a ray-surface hit
//! point falls within a face's trimming boundary, using UV-space projection
//! for analytic surfaces and 3D polygon containment for surfaces with
//! pole singularities (spheres).

use smallvec::SmallVec;

use remus_math::predicates::point_in_polygon;
use remus_math::traits::ParametricSurface;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

use crate::CheckError;
use crate::classify::ray_surface;
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
fn build_uv_boundary<F>(
    verts: &[Point3],
    project: &F,
    u_period: Option<f64>,
    v_period: Option<f64>,
) -> Vec<(f64, f64)>
where
    F: Fn(Point3) -> (f64, f64),
{
    let mut uv: Vec<(f64, f64)> = verts.iter().map(|&p| project(p)).collect();

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

    crossings.sort_by(|x, y| x.0.total_cmp(&y.0));
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
) -> Vec<Vec<(f64, f64)>>
where
    F: Fn(Point3) -> (f64, f64),
{
    holes_3d
        .iter()
        .map(|poly| build_uv_boundary(poly, project, u_period, v_period))
        .collect()
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
}

/// True when a hit lands in one of the face's holes (3D polygon variant).
fn hit_in_hole_3d(holes: &[Vec<Point3>], hit: Point3, normal: Vec3) -> bool {
    holes
        .iter()
        .any(|hole| point_in_polygon_3d(&hit, hole, &normal))
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
) -> Result<u32, CheckError>
where
    F: Fn(Point3) -> (f64, f64),
{
    count_analytic_crossings_with_trim(topo, face_id, None, hits, project, v_periodic)
}

/// [`count_analytic_crossings`] with caller-supplied trim data.
///
/// `trim` carries the prepared polygons; `None` builds them on demand exactly
/// as the one-shot path always has (preserving its early-outs and errors).
fn count_analytic_crossings_with_trim<F>(
    topo: &Topology,
    face_id: FaceId,
    trim: Option<&FaceTrimData>,
    hits: &[Point3],
    project: F,
    v_periodic: bool,
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
    let verts = &trim_data.outer;

    // Detect degenerate boundary: a "full-surface" face whose wire has fewer
    // than 3 distinct vertices. Every positive-t root outside a hole counts.
    let is_full_surface = verts.len() < 3 || {
        let ref_pt = verts[0];
        verts
            .iter()
            .all(|v| (*v - ref_pt).length_squared() < COINCIDENT_SQ)
    };
    // Every analytic surface here parameterizes u as an angle with period 2pi;
    // v is angular only for the torus.
    let u_period = Some(std::f64::consts::TAU);
    let v_period = v_periodic.then_some(std::f64::consts::TAU);
    let uv_boundary =
        (!is_full_surface).then(|| build_uv_boundary(verts, &project, u_period, v_period));
    let holes = hole_uv_boundaries_from_cached(&trim_data.holes, &project, u_period, v_period);

    // A loop that wraps a period bounds no polygon: decide by orientation.
    // Only on the torus. Booleans bound cylinder and cone walls with doubled
    // seams, whose loops close in `(u, v)`, so parity stays exact there; a
    // seamless two-ring wall (outer rim, inner rim) still reads as empty.
    if v_periodic
        && uv_boundary
            .iter()
            .chain(&holes)
            .any(|uv_loop| uv_loop_wraps(uv_loop, u_period, v_period))
    {
        let loops: Vec<&[(f64, f64)]> = uv_boundary
            .iter()
            .chain(&holes)
            .map(Vec::as_slice)
            .collect();
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

        if let Some(boundary) = &uv_boundary
            && !point_in_uv_boundary(hit_u, hit_v, boundary, u_period, v_period)
        {
            continue;
        }
        // The trimmed face carries no material inside its inner wires.
        if hit_in_hole_uv(&holes, hit_u, hit_v, u_period, v_period) {
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
    hits: &[Point3],
    sphere: &remus_math::surfaces::SphericalSurface,
) -> Result<Option<u32>, CheckError> {
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
    // three or more supports may be concave and need a winding-aware rule.
    if constraints.len() != 2 || constraints[0].1.cross(constraints[1].1).length() <= 1e-9 {
        return Ok(None);
    }

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
    let face = topo.face(face_id)?;
    match face.surface() {
        FaceSurface::Plane { normal, d } => {
            ray_plane_crossings_with_trim(topo, face_id, trim, origin, direction, *normal, *d)
        }
        FaceSurface::Cylinder(cyl) => {
            let cyl = cyl.clone();
            let roots = ray_surface::ray_cylinder(origin, direction, &cyl);
            count_analytic_crossings_with_trim(
                topo,
                face_id,
                trim,
                &ray_hit_points(origin, direction, &roots),
                |p| cyl.project_point(p),
                false,
            )
        }
        FaceSurface::Cone(cone) => {
            let cone = cone.clone();
            let roots = ray_surface::ray_cone(origin, direction, &cone);
            count_analytic_crossings_with_trim(
                topo,
                face_id,
                trim,
                &ray_hit_points(origin, direction, &roots),
                |p| cone.project_point(p),
                false,
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
            } else if let Some(count) = count_sphere_circle_patch_crossings(
                topo,
                face_id,
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
                &ray_hit_points(origin, direction, &roots),
                |p| tor.project_point(p),
                true,
            )
        }
        FaceSurface::Nurbs(surface) => {
            ray_crossings_nurbs_with_trim(topo, face_id, trim, origin, direction, surface)
        }
    }
}

/// Ray-plane intersection with point-in-polygon boundary test.
///
/// Takes caller-supplied trim data (`None` builds it on demand, exactly as
/// the one-shot path always has).
fn ray_plane_crossings_with_trim(
    topo: &Topology,
    face_id: FaceId,
    trim: Option<&FaceTrimData>,
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

    if !point_in_polygon_3d(&hit, verts, &normal) {
        return Ok(0);
    }
    // A ray through a hole (bolt hole, absorbed hub circle) passes through
    // empty space, not material — the face contributes no crossing there.
    let holes = &trim_data.holes;
    if hit_in_hole_3d(holes, hit, normal) {
        return Ok(0);
    }
    Ok(1)
}

/// Count ray crossings for a NURBS face using ray-surface intersection.
///
/// Takes caller-supplied trim data (`None` builds it on demand, exactly as
/// the one-shot path always has).
fn ray_crossings_nurbs_with_trim(
    topo: &Topology,
    face_id: FaceId,
    trim: Option<&FaceTrimData>,
    origin: Point3,
    direction: Vec3,
    surface: &remus_math::nurbs::surface::NurbsSurface,
) -> Result<u32, CheckError> {
    let hits = ray_surface::ray_nurbs(origin, direction, surface, 20)?;
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
    count_nurbs_hits_with_trim(topo, face_id, Some(trim_data), surface, &points)
}

/// UV-trimmed NURBS hit counting with caller-supplied trim data (`None`
/// builds it on demand, exactly as the one-shot path always has).
fn count_nurbs_hits_with_trim(
    topo: &Topology,
    face_id: FaceId,
    trim: Option<&FaceTrimData>,
    surface: &remus_math::nurbs::surface::NurbsSurface,
    hits: &[(Point3, f64, f64)],
) -> Result<u32, CheckError> {
    let owned_trim;
    let trim_data = if let Some(cached) = trim {
        cached
    } else {
        owned_trim = FaceTrimData::build(topo, face_id)?;
        &owned_trim
    };
    let verts = &trim_data.outer;
    let project = |p: Point3| -> (f64, f64) { surface.project_point(p) };
    // A full-surface face counts every forward hit that does not land in a
    // hole. `count_analytic_crossings` tests this two ways -- too few boundary
    // points, OR every boundary point coincident -- and only the first test was
    // made here. A doubly-periodic face (a full torus) has a boundary of
    // exactly its seam endpoints: four vertices, all at one place. That passes
    // `len() >= 3`, so a degenerate polygon was built and every hit on the face
    // was then trimmed away against it.
    let is_full_surface = verts.len() < 3 || {
        let ref_pt = verts[0];
        verts
            .iter()
            .all(|v| (*v - ref_pt).length_squared() < COINCIDENT_SQ)
    };
    // A NURBS u/v are knot parameters, not angles. Unwrapping them by 2pi is
    // meaningless -- and actively wrong, since a b-spline domain routinely
    // spans more than pi (a converted box face spans 12.0), so consecutive
    // boundary vertices get shifted by a spurious 2pi. When the surface really
    // does close in a direction, the period is that direction's knot span:
    // without it the seam projects onto the wrong branch and the trim polygon
    // collapses, rejecting every hit on the face.
    let u_period = surface
        .is_periodic_u()
        .then(|| surface.domain_u().1 - surface.domain_u().0);
    let v_period = surface
        .is_periodic_v()
        .then(|| surface.domain_v().1 - surface.domain_v().0);
    let uv_boundary =
        (!is_full_surface).then(|| build_uv_boundary(verts, &project, u_period, v_period));

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
    if let Some(boundary) = &uv_boundary
        && uv_boundary_is_degenerate(boundary)
    {
        let points: Vec<_> = hits.iter().map(|(point, _, _)| *point).collect();
        return count_3d_polygon_crossings_with_trim(topo, face_id, Some(trim_data), &points);
    }

    let holes = hole_uv_boundaries_from_cached(&trim_data.holes, &project, u_period, v_period);

    let mut crossings = 0u32;
    for (_, hit_u, hit_v) in hits {
        if let Some(boundary) = &uv_boundary
            && !point_in_uv_boundary(*hit_u, *hit_v, boundary, u_period, v_period)
        {
            continue;
        }
        if hit_in_hole_uv(&holes, *hit_u, *hit_v, u_period, v_period) {
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
        FaceSurface::Cylinder(surface) => {
            count_analytic_crossings(topo, face_id, &hits, |p| surface.project_point(p), false)?
        }
        FaceSurface::Cone(surface) => {
            count_analytic_crossings(topo, face_id, &hits, |p| surface.project_point(p), false)?
        }
        FaceSurface::Torus(surface) => {
            count_analytic_crossings(topo, face_id, &hits, |p| surface.project_point(p), true)?
        }
        FaceSurface::Sphere(surface) => {
            if let Some(count) = count_sphere_cap_crossings(topo, face_id, &hits, surface)? {
                count
            } else if let Some(count) =
                count_sphere_circle_patch_crossings(topo, face_id, &hits, surface)?
            {
                count
            } else {
                count_3d_polygon_crossings(topo, face_id, &hits)?
            }
        }
        FaceSurface::Nurbs(surface) => {
            let (u, v) = surface.project_point(point);
            count_nurbs_hits_with_trim(topo, face_id, None, surface, &[(point, u, v)])?
        }
    };
    Ok(count > 0)
}
