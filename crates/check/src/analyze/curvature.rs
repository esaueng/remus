//! Surface curvature interrogation.
//!
//! Pointwise principal curvatures and face-wide minimum radius of curvature,
//! for all six surface types the kernel stores on faces.
//!
//! # Sign convention
//!
//! Inherited from [`remus_math::curvature`]: curvatures are positive for
//! convex-outward (the surface bends away from the reference normal, as on a
//! ball's exterior) and negative for concave (bowl interior, torus inner
//! equator). The reference normal is the face's **effective outward normal**:
//! the surface's natural `normal(u, v)` — which points outward from the solid
//! for every primitive — negated when the face is `reversed` (the standard
//! state for faces emitted by booleans). Flipping that normal flips `k1`,
//! `k2`, and `mean`; `gaussian` and the tangent `directions` are
//! orientation-independent.
//!
//! Principal curvatures are sorted `k1 >= k2` after the orientation is
//! applied, so a reversed cylinder reports `k1 = 0, k2 = −1/r`.

use remus_math::curvature::{
    cone_principal_curvatures, cylinder_principal_curvatures, sphere_principal_curvatures,
    torus_principal_curvatures,
};
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::predicates::point_in_polygon;
use remus_math::surfaces::ToroidalSurface;
use remus_math::traits::ParametricSurface;
use remus_math::vec::{Point2, Vec3};
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};

use crate::CheckError;
use crate::util::{face_hole_polygons, face_polygon};

/// Grid resolution per axis of the coarse NURBS minimum-radius sweep.
const NURBS_GRID: usize = 16;
/// Grid resolution per axis of the refinement pass around the coarse best point.
const NURBS_REFINE: usize = 8;

/// Principal curvatures at a point of a face's surface.
///
/// See the [module documentation](self) for the sign convention and the
/// `k1 >= k2` ordering.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurvatureReport {
    /// Largest principal curvature (convex-outward positive).
    pub k1: f64,
    /// Smallest principal curvature.
    pub k2: f64,
    /// Gaussian curvature `K = k1·k2` (orientation-independent).
    pub gaussian: f64,
    /// Mean curvature `H = (k1 + k2)/2` (flips with face orientation).
    pub mean: f64,
    /// Unit tangent principal directions `(d1, d2)` matching `(k1, k2)`.
    ///
    /// `None` at umbilic points — sphere and plane everywhere, torus
    /// nowhere, NURBS wherever the two curvatures coincide within
    /// representation noise — where every tangent direction is principal
    /// and reporting two would fabricate information.
    pub directions: Option<(Vec3, Vec3)>,
}

/// Principal curvatures of a face's surface at surface parameters `(u, v)`.
///
/// `(u, v)` are parameters of the underlying surface (not restricted to the
/// face's trimmed region); the face supplies the surface and its
/// orientation. For a plane face the parameters are ignored and the report
/// is identically zero.
///
/// # Errors
///
/// Returns [`CheckError::CurvatureFailed`] when curvature is undefined at
/// `(u, v)`: a cone at or below its apex, a torus parallel that degenerates
/// (`R + r·cos v <= 0`, self-intersecting configurations only), a
/// non-finite parameter, or a NURBS point where the parametrization
/// collapses (e.g. a sphere pole).
pub fn surface_curvature(
    topo: &Topology,
    face_id: FaceId,
    u: f64,
    v: f64,
) -> Result<CurvatureReport, CheckError> {
    if !u.is_finite() || !v.is_finite() {
        return Err(CheckError::CurvatureFailed(format!(
            "non-finite surface parameters ({u}, {v})"
        )));
    }
    let face = topo.face(face_id)?;
    let sign = if face.is_reversed() { -1.0 } else { 1.0 };
    match face.surface() {
        FaceSurface::Plane { .. } => Ok(CurvatureReport {
            k1: 0.0,
            k2: 0.0,
            gaussian: 0.0,
            mean: 0.0,
            directions: None,
        }),
        FaceSurface::Cylinder(cyl) => {
            // k1 = 1/r along the circumferential (u) direction, k2 = 0 axial.
            let p = cylinder_principal_curvatures(cyl.radius());
            let dir_u = unit_partial(face.surface(), u, v)?;
            let dir_v = cyl.axis();
            Ok(oriented_report(sign, (p.k1, dir_u), (p.k2, dir_v)))
        }
        FaceSurface::Cone(cone) => {
            if v <= 0.0 {
                return Err(CheckError::CurvatureFailed(format!(
                    "cone curvature undefined at slant distance v = {v} (apex at v = 0)"
                )));
            }
            // k1 = tan(α)/v along the circumferential (u) direction, k2 = 0
            // along the ruling.
            let p = cone_principal_curvatures(cone.half_angle(), v)?;
            let dir_u = unit_partial(face.surface(), u, v)?;
            let dir_v = unit_partial_v(face.surface(), u, v)?;
            Ok(oriented_report(sign, (p.k1, dir_u), (p.k2, dir_v)))
        }
        FaceSurface::Sphere(sph) => {
            let p = sphere_principal_curvatures(sph.radius());
            Ok(oriented_umbilic(sign, p.k1))
        }
        FaceSurface::Torus(tor) => {
            let parallel = tor.major_radius() + tor.minor_radius() * v.cos();
            if parallel <= 0.0 {
                return Err(CheckError::CurvatureFailed(format!(
                    "torus parallel degenerates at v = {v} (R + r·cos v = {parallel})"
                )));
            }
            // k_ring = cos v/(R + r·cos v) along the ring (u) direction,
            // k_tube = 1/r along the meridian (v) direction.
            let p = torus_principal_curvatures(tor.major_radius(), tor.minor_radius(), v)?;
            let dir_u = unit_partial(face.surface(), u, v)?;
            let dir_v = unit_partial_v(face.surface(), u, v)?;
            Ok(oriented_report(sign, (p.k1, dir_u), (p.k2, dir_v)))
        }
        FaceSurface::Nurbs(nurbs) => {
            let c = remus_math::nurbs::curvature::surface_curvature(nurbs, u, v)?;
            match c.directions {
                Some((d1, d2)) => Ok(oriented_report(sign, (c.k1, d1), (c.k2, d2))),
                None => Ok(oriented_umbilic(sign, c.k1)),
            }
        }
    }
}

/// Minimum radius of curvature over a face, i.e. `1 / max(|k1|, |k2|)` taken
/// across the face's (trimmed) domain. Orientation-independent.
///
/// - Plane: [`f64::INFINITY`] (no curvature anywhere).
/// - Sphere: the radius, exactly.
/// - Cylinder: the radius, exactly.
/// - Cone: exact from the face's slant-distance extent, `v_min/tan α`; `0.0`
///   when the face reaches its apex (where curvature is unbounded).
/// - Torus: exact from the face's cross-section-angle extent, including the
///   analytically known curvature extrema (outer equator, inner equator,
///   top/bottom circles, and degenerate parallels of spindle-like tori).
/// - NURBS: approximate — a coarse grid over the face's UV domain with a
///   refinement pass around the strongest sample.
///
/// The extent of an analytic face comes from projecting its outer-wire
/// boundary samples into surface parameters. For primitive-style boundaries
/// (circles at constant parameter, rulings, seams) those projections are
/// exact; for exotic trims whose parameter extremes fall between boundary
/// samples the result can only overstate the curvature, i.e. understate the
/// radius (the conservative direction).
///
/// # Errors
///
/// Returns [`CheckError::CurvatureFailed`] if the boundary cannot be
/// projected onto the surface or no curvature sample can be evaluated.
pub fn min_radius_of_curvature(topo: &Topology, face_id: FaceId) -> Result<f64, CheckError> {
    let face = topo.face(face_id)?;
    match face.surface() {
        FaceSurface::Plane { .. } => Ok(f64::INFINITY),
        FaceSurface::Sphere(sph) => Ok(sph.radius()),
        FaceSurface::Cylinder(cyl) => Ok(cyl.radius()),
        FaceSurface::Cone(cone) => {
            let v_lo = boundary_v_range(topo, face_id, None)?.map_or(0.0, |(lo, _)| lo);
            if v_lo <= 0.0 {
                // The face reaches the apex, where curvature is unbounded.
                return Ok(0.0);
            }
            Ok(v_lo / cone.half_angle().tan())
        }
        FaceSurface::Torus(tor) => {
            let band = boundary_v_range(topo, face_id, Some(std::f64::consts::TAU))?;
            Ok(torus_min_radius(tor, band))
        }
        FaceSurface::Nurbs(nurbs) => nurbs_min_radius(topo, face_id, nurbs),
    }
}

/// Assemble a report from two oriented `(curvature, direction)` candidates,
/// sorted `k1 >= k2` with their directions.
fn oriented_report(sign: f64, a: (f64, Vec3), b: (f64, Vec3)) -> CurvatureReport {
    let mut pair = [(a.0 * sign, a.1), (b.0 * sign, b.1)];
    pair.sort_by(|x, y| y.0.total_cmp(&x.0));
    CurvatureReport {
        k1: pair[0].0,
        k2: pair[1].0,
        gaussian: pair[0].0 * pair[1].0,
        mean: (pair[0].0 + pair[1].0) * 0.5,
        directions: Some((pair[0].1, pair[1].1)),
    }
}

/// Assemble a report at an umbilic point (equal curvatures, no directions).
fn oriented_umbilic(sign: f64, k: f64) -> CurvatureReport {
    let k = k * sign;
    CurvatureReport {
        k1: k,
        k2: k,
        gaussian: k * k,
        mean: k,
        directions: None,
    }
}

/// Normalized `∂S/∂u` of a face's surface.
fn unit_partial(surface: &FaceSurface, u: f64, v: f64) -> Result<Vec3, CheckError> {
    surface
        .partial_u(u, v)
        .ok_or_else(|| {
            CheckError::CurvatureFailed("surface has no u parameterization here".into())
        })?
        .normalize()
        .map_err(CheckError::from)
}

/// Normalized `∂S/∂v` of a face's surface.
fn unit_partial_v(surface: &FaceSurface, u: f64, v: f64) -> Result<Vec3, CheckError> {
    surface
        .partial_v(u, v)
        .ok_or_else(|| {
            CheckError::CurvatureFailed("surface has no v parameterization here".into())
        })?
        .normalize()
        .map_err(CheckError::from)
}

/// The `[v_min, v_max]` extent of a face's outer-wire boundary in the
/// surface's `v` parameter, from projecting boundary samples.
///
/// `v_period` unwraps the projection sequentially so a band that crosses the
/// parameter seam stays contiguous. Returns `None` for a degenerate boundary
/// (a full-surface face whose wire collapses to a point) — callers fall back
/// to the surface's natural domain.
fn boundary_v_range(
    topo: &Topology,
    face_id: FaceId,
    v_period: Option<f64>,
) -> Result<Option<(f64, f64)>, CheckError> {
    let verts = face_polygon(topo, face_id)?;
    let degenerate = verts.len() < 3 || {
        let first = verts[0];
        verts.iter().all(|p| (*p - first).length_squared() < 1e-12)
    };
    if degenerate {
        return Ok(None);
    }
    let surface = topo.face(face_id)?.surface();
    let mut v_lo = f64::INFINITY;
    let mut v_hi = f64::NEG_INFINITY;
    let mut prev: Option<f64> = None;
    for &p in &verts {
        let Some((_, raw)) = surface.project_point(p) else {
            return Err(CheckError::CurvatureFailed(
                "face boundary point does not project onto the surface".into(),
            ));
        };
        let v = match (v_period, prev) {
            (Some(period), Some(prev_v)) => unwrap_periodic(prev_v, raw, period),
            _ => raw,
        };
        prev = Some(v);
        v_lo = v_lo.min(v);
        v_hi = v_hi.max(v);
    }
    Ok(Some((v_lo, v_hi)))
}

/// Unwrap one step of a periodic coordinate so the difference from `prev`
/// lies in `[−period/2, period/2)`.
fn unwrap_periodic(prev: f64, next: f64, period: f64) -> f64 {
    let half = period * 0.5;
    let diff = next - prev;
    prev + diff - period * ((diff + half) / period).floor()
}

/// Exact minimum radius of curvature of a torus restricted to a
/// cross-section-angle band `[v_lo, v_hi]` (the full `[0, 2π]` circle when
/// the band is unknown).
///
/// The tube curvature `1/r` is constant; the ring curvature
/// `cos v/(R + r·cos v)` has its extrema at the outer equator (`v = 0`),
/// inner equator (`v = π`) and the top/bottom circles (`v = ±π/2`, value 0),
/// and blows up at degenerate parallels `R + r·cos v = 0` (reachable only
/// when `r >= R`). All candidates inside the band are evaluated in closed
/// form.
fn torus_min_radius(tor: &ToroidalSurface, band: Option<(f64, f64)>) -> f64 {
    let tau = std::f64::consts::TAU;
    let pi = std::f64::consts::PI;
    let (big, minor) = (tor.major_radius(), tor.minor_radius());
    let (v_lo, v_hi) = band.map_or((0.0, tau), |(lo, hi)| (lo.min(hi), lo.max(hi)));

    let mut candidates = vec![v_lo, v_hi];
    let push = |c: f64, candidates: &mut Vec<f64>| {
        for shift in [-tau, 0.0, tau] {
            let v = c + shift;
            if v >= v_lo && v <= v_hi {
                candidates.push(v);
            }
        }
    };
    // Ring-curvature extrema.
    push(0.0, &mut candidates);
    push(pi, &mut candidates);
    // Degenerate parallels (spindle/horn configurations only).
    if minor >= big {
        let v_star = (-big / minor).acos();
        push(v_star, &mut candidates);
        push(tau - v_star, &mut candidates);
    }

    let mut max_k = 1.0 / minor;
    for &v in &candidates {
        let denom = big + minor * v.cos();
        if denom <= 0.0 {
            // A degenerate parallel lies inside the face: curvature unbounded.
            return 0.0;
        }
        max_k = max_k.max((v.cos() / denom).abs());
    }
    1.0 / max_k
}

/// Approximate minimum radius of curvature of a NURBS face: a coarse grid
/// over the face's UV domain (boundary-polygon containment when the boundary
/// encloses a region, the full surface domain when it degenerates) plus a
/// refinement pass around the strongest sample.
fn nurbs_min_radius(
    topo: &Topology,
    face_id: FaceId,
    surface: &NurbsSurface,
) -> Result<f64, CheckError> {
    let (u0, u1) = surface.domain_u();
    let (v0, v1) = surface.domain_v();
    let verts = face_polygon(topo, face_id)?;
    let holes = face_hole_polygons(topo, face_id)?;
    let u_period = surface.is_periodic_u().then_some(u1 - u0);
    let v_period = surface.is_periodic_v().then_some(v1 - v0);

    let project_all = |points: &[remus_math::vec::Point3]| -> Vec<(f64, f64)> {
        let mut out = Vec::with_capacity(points.len());
        let mut prev: Option<(f64, f64)> = None;
        for &p in points {
            let (mut pu, mut pv) = ParametricSurface::project_point(surface, p);
            if let (Some(period), Some((prev_u, _))) = (u_period, prev) {
                pu = unwrap_periodic(prev_u, pu, period);
            }
            if let (Some(period), Some((_, prev_v))) = (v_period, prev) {
                pv = unwrap_periodic(prev_v, pv, period);
            }
            prev = Some((pu, pv));
            out.push((pu, pv));
        }
        out
    };

    let outer = project_all(&verts);
    let hole_polys: Vec<Vec<(f64, f64)>> = holes.iter().map(|h| project_all(h)).collect();

    let degenerate = outer.len() < 3 || {
        let (u_ref, v_ref) = outer[0];
        outer
            .iter()
            .all(|(u, v)| (u - u_ref).hypot(v - v_ref) < 1e-9)
    };

    let to_point2 = |poly: &[(f64, f64)]| -> Vec<Point2> {
        poly.iter().map(|(u, v)| Point2::new(*u, *v)).collect()
    };

    // (max |k|, u, v) over accepted samples.
    let mut best: Option<(f64, f64, f64)> = None;
    let consider = |u: f64, v: f64, inside: bool, best: &mut Option<(f64, f64, f64)>| {
        if !inside {
            return;
        }
        if let Ok(c) = remus_math::nurbs::curvature::surface_curvature(surface, u, v) {
            let k = c.k1.abs().max(c.k2.abs());
            if best.is_none_or(|(bk, _, _)| k > bk) {
                *best = Some((k, u, v));
            }
        }
    };

    let (dom_u, dom_v) = if degenerate {
        ((u0, u1), (v0, v1))
    } else {
        let u_lo = outer.iter().fold(f64::INFINITY, |a, (u, _)| a.min(*u));
        let u_hi = outer.iter().fold(f64::NEG_INFINITY, |a, (u, _)| a.max(*u));
        let v_lo = outer.iter().fold(f64::INFINITY, |a, (_, v)| a.min(*v));
        let v_hi = outer.iter().fold(f64::NEG_INFINITY, |a, (_, v)| a.max(*v));
        ((u_lo, u_hi), (v_lo, v_hi))
    };
    let outer_poly = to_point2(&outer);
    let hole_polys2: Vec<Vec<Point2>> = hole_polys.iter().map(|h| to_point2(h)).collect();

    let sweep = |u_range: (f64, f64),
                 v_range: (f64, f64),
                 steps: usize,
                 best: &mut Option<(f64, f64, f64)>| {
        for i in 0..=steps {
            let du = u_range.0 + (u_range.1 - u_range.0) * i as f64 / steps as f64;
            for j in 0..=steps {
                let dv = v_range.0 + (v_range.1 - v_range.0) * j as f64 / steps as f64;
                let inside = degenerate
                    || (point_in_polygon(Point2::new(du, dv), &outer_poly)
                        && !hole_polys2
                            .iter()
                            .any(|h| point_in_polygon(Point2::new(du, dv), h)));
                consider(du, dv, inside, best);
            }
        }
    };

    sweep(dom_u, dom_v, NURBS_GRID, &mut best);
    if !degenerate {
        for &(u, v) in &outer {
            consider(u, v, true, &mut best);
        }
    }
    // Refine around the strongest coarse sample.
    if let Some((_, bu, bv)) = best {
        let step_u = (dom_u.1 - dom_u.0) / NURBS_GRID as f64;
        let step_v = (dom_v.1 - dom_v.0) / NURBS_GRID as f64;
        let u_range = ((bu - step_u).max(dom_u.0), (bu + step_u).min(dom_u.1));
        let v_range = ((bv - step_v).max(dom_v.0), (bv + step_v).min(dom_v.1));
        sweep(u_range, v_range, NURBS_REFINE, &mut best);
    }

    let Some((max_k, _, _)) = best else {
        return Err(CheckError::CurvatureFailed(
            "no valid curvature sample on the NURBS face".into(),
        ));
    };
    if max_k == 0.0 {
        return Ok(f64::INFINITY);
    }
    Ok(1.0 / max_k)
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::float_cmp
    )]

    use remus_math::curves::Circle3D;
    use remus_math::surfaces::{
        ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface,
    };
    use remus_math::vec::Point3;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::topology::Topology;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    use super::{min_radius_of_curvature, surface_curvature};

    const TOL: f64 = 1e-7;
    const Z_AXIS: remus_math::vec::Vec3 = remus_math::vec::Vec3::new(0.0, 0.0, 1.0);

    /// Full lateral cylinder face: two rim circles joined by a seam ruling.
    fn cylinder_face(
        topo: &mut Topology,
        radius: f64,
        height: f64,
    ) -> remus_topology::face::FaceId {
        let cyl = CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Z_AXIS, radius).unwrap();
        let v_bot = topo.add_vertex(Vertex::new(Point3::new(radius, 0.0, 0.0), TOL));
        let v_top = topo.add_vertex(Vertex::new(Point3::new(radius, 0.0, height), TOL));
        let bot = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Z_AXIS, radius).unwrap();
        let top = Circle3D::new(Point3::new(0.0, 0.0, height), Z_AXIS, radius).unwrap();
        let t_bot = bot.project(Point3::new(radius, 0.0, 0.0));
        let mut e_bot = Edge::new(v_bot, v_bot, EdgeCurve::Circle(bot));
        e_bot.set_trim(Some((t_bot, t_bot + std::f64::consts::TAU)));
        let t_top = top.project(Point3::new(radius, 0.0, height));
        let mut e_top = Edge::new(v_top, v_top, EdgeCurve::Circle(top));
        e_top.set_trim(Some((t_top, t_top + std::f64::consts::TAU)));
        let e_seam = topo.add_edge(Edge::new(v_bot, v_top, EdgeCurve::Line));
        let e_bot = topo.add_edge(e_bot);
        let e_top = topo.add_edge(e_top);
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e_bot, true),
                    OrientedEdge::new(e_seam, true),
                    OrientedEdge::new(e_top, false),
                    OrientedEdge::new(e_seam, false),
                ],
                true,
            )
            .unwrap(),
        );
        topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(cyl)))
    }

    /// Full lateral cone frustum face between slant distances `v_lo`/`v_hi`.
    fn cone_face(
        topo: &mut Topology,
        half_angle: f64,
        v_lo: f64,
        v_hi: f64,
    ) -> remus_topology::face::FaceId {
        let cone = ConicalSurface::new(Point3::new(0.0, 0.0, 0.0), Z_AXIS, half_angle).unwrap();
        let (sin_a, cos_a) = half_angle.sin_cos();
        let (r_lo, z_lo) = (v_lo * cos_a, v_lo * sin_a);
        let (r_hi, z_hi) = (v_hi * cos_a, v_hi * sin_a);
        let v_a = topo.add_vertex(Vertex::new(Point3::new(r_lo, 0.0, z_lo), TOL));
        let v_b = topo.add_vertex(Vertex::new(Point3::new(r_hi, 0.0, z_hi), TOL));
        let circle_lo = Circle3D::new(Point3::new(0.0, 0.0, z_lo), Z_AXIS, r_lo).unwrap();
        let circle_hi = Circle3D::new(Point3::new(0.0, 0.0, z_hi), Z_AXIS, r_hi).unwrap();
        let t_lo = circle_lo.project(Point3::new(r_lo, 0.0, z_lo));
        let mut e_lo = Edge::new(v_a, v_a, EdgeCurve::Circle(circle_lo));
        e_lo.set_trim(Some((t_lo, t_lo + std::f64::consts::TAU)));
        let t_hi = circle_hi.project(Point3::new(r_hi, 0.0, z_hi));
        let mut e_hi = Edge::new(v_b, v_b, EdgeCurve::Circle(circle_hi));
        e_hi.set_trim(Some((t_hi, t_hi + std::f64::consts::TAU)));
        let e_seam = topo.add_edge(Edge::new(v_a, v_b, EdgeCurve::Line));
        let e_lo = topo.add_edge(e_lo);
        let e_hi = topo.add_edge(e_hi);
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e_lo, true),
                    OrientedEdge::new(e_seam, true),
                    OrientedEdge::new(e_hi, false),
                    OrientedEdge::new(e_seam, false),
                ],
                true,
            )
            .unwrap(),
        );
        topo.add_face(Face::new(wire, vec![], FaceSurface::Cone(cone)))
    }

    /// Full torus face bounded by the u = 0 meridian circle.
    fn torus_face(topo: &mut Topology, major: f64, minor: f64) -> remus_topology::face::FaceId {
        let tor = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), major, minor).unwrap();
        let v = topo.add_vertex(Vertex::new(Point3::new(major + minor, 0.0, 0.0), TOL));
        let meridian = Circle3D::new(
            Point3::new(major, 0.0, 0.0),
            remus_math::vec::Vec3::new(0.0, 1.0, 0.0),
            minor,
        )
        .unwrap();
        let t0 = meridian.project(Point3::new(major + minor, 0.0, 0.0));
        let mut e = Edge::new(v, v, EdgeCurve::Circle(meridian));
        e.set_trim(Some((t0, t0 + std::f64::consts::TAU)));
        let e = topo.add_edge(e);
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap());
        topo.add_face(Face::new(wire, vec![], FaceSurface::Torus(tor)))
    }

    /// Full sphere face bounded by the u = 0 meridian circle.
    fn sphere_face(topo: &mut Topology, radius: f64) -> remus_topology::face::FaceId {
        let sph = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), radius).unwrap();
        let v = topo.add_vertex(Vertex::new(Point3::new(radius, 0.0, 0.0), TOL));
        let meridian = Circle3D::new(
            Point3::new(0.0, 0.0, 0.0),
            remus_math::vec::Vec3::new(0.0, 1.0, 0.0),
            radius,
        )
        .unwrap();
        let t0 = meridian.project(Point3::new(radius, 0.0, 0.0));
        let mut e = Edge::new(v, v, EdgeCurve::Circle(meridian));
        e.set_trim(Some((t0, t0 + std::f64::consts::TAU)));
        let e = topo.add_edge(e);
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap());
        topo.add_face(Face::new(wire, vec![], FaceSurface::Sphere(sph)))
    }

    /// Exact rational NURBS sphere (degree (2, 2) surface of revolution).
    ///
    /// This is the differential oracle for the NURBS curvature path. The
    /// sampled conversion in `remus-geometry` (`convert::sphere_to_nurbs`)
    /// emits a degree-1 faceted approximation whose pointwise curvature is
    /// ~0 — no oracle against the analytic sphere is possible through it,
    /// so the test builds the geometrically exact rational sphere instead.
    fn exact_sphere_nurbs(radius: f64) -> remus_math::nurbs::surface::NurbsSurface {
        let s2 = std::f64::consts::FRAC_1_SQRT_2;
        let profile: [(f64, f64, f64); 5] = [
            (0.0, -radius, 1.0),
            (radius, -radius, s2),
            (radius, 0.0, 1.0),
            (radius, radius, s2),
            (0.0, radius, 1.0),
        ];
        let rev_w = [1.0, s2, 1.0, s2, 1.0, s2, 1.0, s2, 1.0];
        let mut cps = Vec::with_capacity(9);
        let mut weights = Vec::with_capacity(9);
        for (i, w) in rev_w.iter().enumerate() {
            let phi = i as f64 * std::f64::consts::FRAC_PI_4;
            let (sin_phi, cos_phi) = phi.sin_cos();
            let rho = if i % 2 == 0 {
                1.0
            } else {
                std::f64::consts::SQRT_2
            };
            let mut row = Vec::with_capacity(5);
            let mut w_row = Vec::with_capacity(5);
            for &(px, pz, wp) in &profile {
                row.push(Point3::new(px * rho * cos_phi, px * rho * sin_phi, pz));
                w_row.push(wp * w);
            }
            cps.push(row);
            weights.push(w_row);
        }
        let knots_u = vec![
            0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
        ];
        let knots_v = vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0];
        remus_math::nurbs::surface::NurbsSurface::new(2, 2, knots_u, knots_v, cps, weights).unwrap()
    }

    /// NURBS sphere face bounded by the seam meridian.
    fn nurbs_sphere_face(topo: &mut Topology, radius: f64) -> remus_topology::face::FaceId {
        let nurbs = exact_sphere_nurbs(radius);
        let v = topo.add_vertex(Vertex::new(Point3::new(radius, 0.0, 0.0), TOL));
        let meridian = Circle3D::new(
            Point3::new(0.0, 0.0, 0.0),
            remus_math::vec::Vec3::new(0.0, 1.0, 0.0),
            radius,
        )
        .unwrap();
        let t0 = meridian.project(Point3::new(radius, 0.0, 0.0));
        let mut e = Edge::new(v, v, EdgeCurve::Circle(meridian));
        e.set_trim(Some((t0, t0 + std::f64::consts::TAU)));
        let e = topo.add_edge(e);
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap());
        topo.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(nurbs)))
    }

    fn assert_close(actual: f64, expected: f64, tol: f64, what: &str) {
        assert!(
            (actual - expected).abs() <= tol * expected.abs().max(1.0),
            "{what}: {actual} vs {expected}"
        );
    }

    #[test]
    fn plane_has_zero_curvature_and_infinite_radius() {
        let mut topo = Topology::new();
        let face = remus_topology::test_utils::make_unit_square_face(&mut topo);
        let r = surface_curvature(&topo, face, 0.3, 0.7).unwrap();
        assert_close(r.k1, 0.0, 1e-15, "plane k1");
        assert_close(r.k2, 0.0, 1e-15, "plane k2");
        assert_close(r.gaussian, 0.0, 1e-15, "plane K");
        assert_close(r.mean, 0.0, 1e-15, "plane H");
        assert!(r.directions.is_none(), "plane is umbilic everywhere");
        assert!(min_radius_of_curvature(&topo, face).unwrap().is_infinite());
    }

    #[test]
    fn cylinder_curvature_and_directions_match_closed_form() {
        let mut topo = Topology::new();
        let (radius, height) = (2.0_f64, 5.0_f64);
        let face = cylinder_face(&mut topo, radius, height);
        let r = surface_curvature(&topo, face, 1.0, 0.7).unwrap();
        assert_close(r.k1, 1.0 / radius, 1e-12, "cylinder k1");
        assert_close(r.k2, 0.0, 1e-12, "cylinder k2");
        assert_close(r.gaussian, 0.0, 1e-12, "cylinder K");
        assert_close(r.mean, 1.0 / (2.0 * radius), 1e-12, "cylinder H");
        let (d1, d2) = r.directions.expect("cylinder is not umbilic");
        // k1 is circumferential (perpendicular to the axis), k2 axial.
        assert!(d1.dot(Z_AXIS).abs() < 1e-12);
        assert!((d2.dot(Z_AXIS) - 1.0).abs() < 1e-12);
        assert_close(
            min_radius_of_curvature(&topo, face).unwrap(),
            radius,
            1e-12,
            "cylinder min radius",
        );
    }

    #[test]
    fn reversed_cylinder_flips_signed_curvature() {
        let mut topo = Topology::new();
        let radius = 2.0_f64;
        // Same face, orientation reversed (boolean-emitted state).
        let cyl = CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Z_AXIS, radius).unwrap();
        let v_bot = topo.add_vertex(Vertex::new(Point3::new(radius, 0.0, 0.0), TOL));
        let v_top = topo.add_vertex(Vertex::new(Point3::new(radius, 0.0, 3.0), TOL));
        let bot = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Z_AXIS, radius).unwrap();
        let top = Circle3D::new(Point3::new(0.0, 0.0, 3.0), Z_AXIS, radius).unwrap();
        let t0 = bot.project(Point3::new(radius, 0.0, 0.0));
        let mut e_bot = Edge::new(v_bot, v_bot, EdgeCurve::Circle(bot));
        e_bot.set_trim(Some((t0, t0 + std::f64::consts::TAU)));
        let t1 = top.project(Point3::new(radius, 0.0, 3.0));
        let mut e_top = Edge::new(v_top, v_top, EdgeCurve::Circle(top));
        e_top.set_trim(Some((t1, t1 + std::f64::consts::TAU)));
        let e_seam = topo.add_edge(Edge::new(v_bot, v_top, EdgeCurve::Line));
        let e_bot = topo.add_edge(e_bot);
        let e_top = topo.add_edge(e_top);
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e_bot, true),
                    OrientedEdge::new(e_seam, true),
                    OrientedEdge::new(e_top, false),
                    OrientedEdge::new(e_seam, false),
                ],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new_reversed(wire, vec![], FaceSurface::Cylinder(cyl)));

        let r = surface_curvature(&topo, face, 1.0, 0.7).unwrap();
        assert_close(r.mean, -1.0 / (2.0 * radius), 1e-12, "reversed H flips");
        assert_close(r.gaussian, 0.0, 1e-12, "reversed K unchanged");
        assert_close(r.k2, -1.0 / radius, 1e-12, "reversed k2");
        // min radius is orientation-independent.
        assert_close(
            min_radius_of_curvature(&topo, face).unwrap(),
            radius,
            1e-12,
            "reversed min radius",
        );
    }

    #[test]
    fn cone_curvature_matches_derived_form() {
        let mut topo = Topology::new();
        let alpha = std::f64::consts::FRAC_PI_6;
        let (v_lo, v_hi) = (2.0_f64, 5.0_f64);
        let face = cone_face(&mut topo, alpha, v_lo, v_hi);

        // k = tan(α)/v at several slant distances (closed form, machine-precision).
        let u_ang = 0.9_f64;
        let FaceSurface::Cone(cone) = topo.face(face).unwrap().surface() else {
            panic!("expected a cone face");
        };
        for &v in &[2.5_f64, 3.0, 4.7] {
            let r = surface_curvature(&topo, face, u_ang, v).unwrap();
            assert_close(r.k1, alpha.tan() / v, 1e-12, "cone k1");
            assert_close(r.k2, 0.0, 1e-12, "cone k2 (ruling)");
            let (d1, d2) = r.directions.expect("cone is not umbilic");
            // k1 circumferential, k2 along the ruling through the actual
            // query point (the ruling passes through the apex). The
            // parametric frame may be rotated in world axes, so derive the
            // ruling from the evaluated point, not from the parameter.
            let point = cone.evaluate(u_ang, v);
            let ruling = (point - cone.apex()).normalize().unwrap();
            assert!(d1.dot(ruling).abs() < 1e-12, "k1 dir ⊥ ruling");
            assert!((d2.dot(ruling) - 1.0).abs() < 1e-12, "k2 dir = ruling");
        }
        // min radius exact from the v extent: v_lo / tan α.
        assert_close(
            min_radius_of_curvature(&topo, face).unwrap(),
            v_lo / alpha.tan(),
            1e-9,
            "cone min radius",
        );
    }

    #[test]
    fn cone_curvature_undefined_at_apex() {
        let mut topo = Topology::new();
        let face = cone_face(&mut topo, std::f64::consts::FRAC_PI_6, 2.0, 5.0);
        let err = surface_curvature(&topo, face, 0.0, 0.0).unwrap_err();
        assert!(err.to_string().contains("apex"), "unexpected error: {err}");
    }

    #[test]
    fn sphere_is_umbilic_with_exact_curvature() {
        let mut topo = Topology::new();
        let radius = 2.5_f64;
        let face = sphere_face(&mut topo, radius);
        for &(u, v) in &[(1.0_f64, 0.3_f64), (2.0, -0.8), (4.5, 1.2)] {
            let r = surface_curvature(&topo, face, u, v).unwrap();
            assert_close(r.k1, 1.0 / radius, 1e-12, "sphere k1");
            assert_close(r.k2, 1.0 / radius, 1e-12, "sphere k2");
            assert_close(r.gaussian, 1.0 / (radius * radius), 1e-12, "sphere K");
            assert_close(r.mean, 1.0 / radius, 1e-12, "sphere H");
            assert!(r.directions.is_none(), "sphere is umbilic");
        }
        assert_close(
            min_radius_of_curvature(&topo, face).unwrap(),
            radius,
            1e-12,
            "sphere min radius",
        );
    }

    #[test]
    fn torus_special_parallels_match_closed_forms() {
        let mut topo = Topology::new();
        let (major, minor) = (4.0_f64, 1.0_f64);
        let face = torus_face(&mut topo, major, minor);
        let pi = std::f64::consts::PI;

        // Outer equator: both convex, tube curvature dominates.
        let r = surface_curvature(&topo, face, 0.0, 0.0).unwrap();
        assert_close(r.k1, 1.0 / minor, 1e-12, "torus outer k1 (tube)");
        assert_close(r.k2, 1.0 / (major + minor), 1e-12, "torus outer k2 (ring)");
        // Inner equator: ring direction concave (saddle, K < 0).
        let r = surface_curvature(&topo, face, 0.0, pi).unwrap();
        assert_close(r.k1, 1.0 / minor, 1e-12, "torus inner k1 (tube)");
        assert_close(r.k2, -1.0 / (major - minor), 1e-12, "torus inner k2 (ring)");
        assert!(r.gaussian < 0.0, "inner equator is saddle");
        // Top circle: ring curvature vanishes.
        let r = surface_curvature(&topo, face, 0.0, pi * 0.5).unwrap();
        assert_close(r.k1, 1.0 / minor, 1e-12, "torus top k1 (tube)");
        assert_close(r.k2, 0.0, 1e-12, "torus top k2 (ring)");
        // Bottom circle: same by symmetry.
        let r = surface_curvature(&topo, face, 0.0, -pi * 0.5).unwrap();
        assert_close(r.k1, 1.0 / minor, 1e-12, "torus bottom k1 (tube)");
        assert_close(r.k2, 0.0, 1e-12, "torus bottom k2 (ring)");
        // All four parallels are non-umbilic: directions reported.
        assert!(r.directions.is_some());

        // Full face: max |k| = 1/r (tube) since 1/(R−r) = 1/3 < 1.
        assert_close(
            min_radius_of_curvature(&topo, face).unwrap(),
            minor,
            1e-12,
            "torus min radius",
        );
    }

    #[test]
    fn torus_inner_equator_can_dominate_min_radius() {
        // R = 1.5, r = 1: inner-equator ring curvature 1/(R−r) = 2 > 1/r.
        let mut topo = Topology::new();
        let face = torus_face(&mut topo, 1.5, 1.0);
        assert_close(
            min_radius_of_curvature(&topo, face).unwrap(),
            0.5,
            1e-12,
            "tight torus min radius",
        );
    }

    #[test]
    fn nurbs_sphere_matches_analytic_within_1e_9() {
        let mut topo = Topology::new();
        let radius = 2.5_f64;
        let face = nurbs_sphere_face(&mut topo, radius);
        for &(u, v) in &[(0.1_f64, 0.4_f64), (0.35, 0.15), (0.6, 0.75), (0.9, 0.5)] {
            let r = surface_curvature(&topo, face, u, v).unwrap();
            assert!(
                (r.k1 - 1.0 / radius).abs() < 1e-9,
                "NURBS sphere k1 at ({u},{v}): {} vs {}",
                r.k1,
                1.0 / radius
            );
            assert!(
                (r.k2 - 1.0 / radius).abs() < 1e-9,
                "NURBS sphere k2 at ({u},{v}): {} vs {}",
                r.k2,
                1.0 / radius
            );
            assert!(
                r.directions.is_none(),
                "NURBS sphere must be umbilic at ({u},{v})"
            );
        }
    }

    #[test]
    fn nurbs_sphere_degenerate_at_poles() {
        let mut topo = Topology::new();
        let face = nurbs_sphere_face(&mut topo, 2.0);
        // v = 0 and v = 1 are the poles: the parametrization collapses and
        // the fundamental forms are singular (surfaced as the math error).
        assert!(matches!(
            surface_curvature(&topo, face, 0.3, 0.0),
            Err(crate::CheckError::Math(
                remus_math::MathError::SingularMatrix
            ))
        ));
        assert!(surface_curvature(&topo, face, 0.3, 1.0).is_err());
    }

    #[test]
    fn nurbs_sphere_min_radius_approximates_radius() {
        let mut topo = Topology::new();
        let radius = 2.0_f64;
        let face = nurbs_sphere_face(&mut topo, radius);
        assert_close(
            min_radius_of_curvature(&topo, face).unwrap(),
            radius,
            1e-6,
            "NURBS sphere min radius",
        );
    }

    #[test]
    fn spindle_torus_point_query_errors_on_degenerate_parallel() {
        let mut topo = Topology::new();
        // R = 0.5, r = 1: R + r·cos π = −0.5 < 0.
        let face = torus_face(&mut topo, 0.5, 1.0);
        let err = surface_curvature(&topo, face, 0.0, std::f64::consts::PI).unwrap_err();
        assert!(
            err.to_string().contains("degenerates"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn non_finite_parameters_are_rejected() {
        let mut topo = Topology::new();
        let face = cylinder_face(&mut topo, 2.0, 5.0);
        assert!(surface_curvature(&topo, face, f64::NAN, 0.0).is_err());
        assert!(surface_curvature(&topo, face, 0.0, f64::INFINITY).is_err());
    }
}
