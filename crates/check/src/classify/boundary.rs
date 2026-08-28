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
fn unwrap_periodic(prev: f64, next: f64, period: f64) -> f64 {
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

/// Compute the normal of a polygon via Newell's method.
///
/// Returns a unit-length normal, or `(0,0,1)` for degenerate polygons.
pub fn polygon_normal(verts: &[Point3]) -> Vec3 {
    crate::util::polygon_normal(verts)
}

/// Build the UV boundary of every hole (inner wire) of a face.
fn hole_uv_boundaries<F>(
    topo: &Topology,
    face_id: FaceId,
    project: &F,
    u_period: Option<f64>,
    v_period: Option<f64>,
) -> Result<Vec<Vec<(f64, f64)>>, CheckError>
where
    F: Fn(Point3) -> (f64, f64),
{
    Ok(face_hole_polygons(topo, face_id)?
        .iter()
        .map(|poly| build_uv_boundary(poly, project, u_period, v_period))
        .collect())
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
    origin: Point3,
    direction: Vec3,
    roots: &SmallVec<[f64; 4]>,
    project: F,
    v_periodic: bool,
) -> Result<u32, CheckError>
where
    F: Fn(Point3) -> (f64, f64),
{
    if roots.is_empty() {
        return Ok(0);
    }

    let verts = face_polygon(topo, face_id)?;

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
        (!is_full_surface).then(|| build_uv_boundary(&verts, &project, u_period, v_period));
    let holes = hole_uv_boundaries(topo, face_id, &project, u_period, v_period)?;

    let mut crossings = 0u32;
    for &t in roots {
        if t <= RAY_T_MIN {
            continue;
        }
        let hit = origin + direction * t;
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
    origin: Point3,
    direction: Vec3,
    roots: &SmallVec<[f64; 4]>,
) -> Result<u32, CheckError> {
    if roots.is_empty() {
        return Ok(0);
    }

    let verts = face_polygon(topo, face_id)?;
    if verts.len() < 3 {
        return Ok(0);
    }
    let mut normal = polygon_normal(&verts);
    // If the face is reversed, the surface normal is flipped.
    let face = topo.face(face_id)?;
    if face.is_reversed() {
        normal = -normal;
    }
    let ref_pt = verts[0];
    let holes = face_hole_polygons(topo, face_id)?;

    let mut crossings = 0u32;
    for &t in roots {
        if t <= RAY_T_MIN {
            continue;
        }
        let hit = origin + direction * t;

        // The hit must be on the face's side of the boundary plane.
        let side = (hit - ref_pt).dot(normal);
        if side < -HALF_SPACE_EPS {
            continue;
        }

        if point_in_polygon_3d(&hit, &verts, &normal) && !hit_in_hole_3d(&holes, hit, normal) {
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
    let face = topo.face(face_id)?;
    match face.surface() {
        FaceSurface::Plane { normal, d } => {
            ray_plane_crossings(topo, face_id, origin, direction, *normal, *d)
        }
        FaceSurface::Cylinder(cyl) => {
            let cyl = cyl.clone();
            let roots = ray_surface::ray_cylinder(origin, direction, &cyl);
            count_analytic_crossings(
                topo,
                face_id,
                origin,
                direction,
                &roots,
                |p| cyl.project_point(p),
                false,
            )
        }
        FaceSurface::Cone(cone) => {
            let cone = cone.clone();
            let roots = ray_surface::ray_cone(origin, direction, &cone);
            count_analytic_crossings(
                topo,
                face_id,
                origin,
                direction,
                &roots,
                |p| cone.project_point(p),
                false,
            )
        }
        FaceSurface::Sphere(sph) => {
            // Sphere boundaries are planar (equator, small circles), so
            // point_in_polygon_3d works. UV projection fails at poles.
            let sph = sph.clone();
            let roots = ray_surface::ray_sphere(origin, direction, &sph);
            count_3d_polygon_crossings(topo, face_id, origin, direction, &roots)
        }
        FaceSurface::Torus(tor) => {
            let tor = tor.clone();
            let roots = ray_surface::ray_torus(origin, direction, &tor);
            count_analytic_crossings(
                topo,
                face_id,
                origin,
                direction,
                &roots,
                |p| tor.project_point(p),
                true,
            )
        }
        FaceSurface::Nurbs(surface) => {
            ray_crossings_nurbs(topo, face_id, origin, direction, surface)
        }
    }
}

/// Ray-plane intersection with point-in-polygon boundary test.
fn ray_plane_crossings(
    topo: &Topology,
    face_id: FaceId,
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
    let verts = face_polygon(topo, face_id)?;
    if verts.len() < 3 {
        return Ok(0);
    }

    if !point_in_polygon_3d(&hit, &verts, &normal) {
        return Ok(0);
    }
    // A ray through a hole (bolt hole, absorbed hub circle) passes through
    // empty space, not material — the face contributes no crossing there.
    let holes = face_hole_polygons(topo, face_id)?;
    if hit_in_hole_3d(&holes, hit, normal) {
        return Ok(0);
    }
    Ok(1)
}

/// Count ray crossings for a NURBS face using ray-surface intersection.
fn ray_crossings_nurbs(
    topo: &Topology,
    face_id: FaceId,
    origin: Point3,
    direction: Vec3,
    surface: &remus_math::nurbs::surface::NurbsSurface,
) -> Result<u32, CheckError> {
    let hits = ray_surface::ray_nurbs(origin, direction, surface, 20)?;
    if hits.is_empty() {
        return Ok(0);
    }

    let verts = face_polygon(topo, face_id)?;
    let project = |p: Point3| -> (f64, f64) { surface.project_point(p) };
    // A full-surface face (fewer than 3 boundary points) counts every forward
    // hit that does not land in a hole.
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
        (verts.len() >= 3).then(|| build_uv_boundary(&verts, &project, u_period, v_period));
    let holes = hole_uv_boundaries(topo, face_id, &project, u_period, v_period)?;

    let mut crossings = 0u32;
    for (_, hit_u, hit_v) in &hits {
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
