//! Per-face Gauss quadrature integration for area, volume, CoM, and inertia.
//!
//! Provides numerical integration of geometric properties over individual
//! faces. Planar faces use polygon fan triangulation; parametric faces
//! (cylinder, cone, sphere, torus, NURBS) use tensor-product Gauss-Legendre
//! quadrature over the UV domain.

use brepkit_math::quadrature::gauss_legendre_points;
use brepkit_math::traits::ParametricSurface;
use brepkit_math::vec::{Point3, Vec3};
use brepkit_topology::Topology;
use brepkit_topology::edge::EdgeCurve;
use brepkit_topology::face::{FaceId, FaceSurface};

use crate::CheckError;

/// Contribution of a single face to global geometric properties.
#[derive(Debug, Clone)]
pub struct FaceContribution {
    /// Face area.
    pub area: f64,
    /// Volume contribution: (1/3) integral of P dot N dA.
    pub volume: f64,
    /// Volume-weighted x-moment: (1/2) integral of x^2 * n_x dA (divergence theorem).
    pub volume_moment_x: f64,
    /// Volume-weighted y-moment: (1/2) integral of y^2 * n_y dA (divergence theorem).
    pub volume_moment_y: f64,
    /// Volume-weighted z-moment: (1/2) integral of z^2 * n_z dA (divergence theorem).
    pub volume_moment_z: f64,
    /// Raw volume integral of `x²` about the global origin.
    pub volume_second_x: f64,
    /// Raw volume integral of `y²` about the global origin.
    pub volume_second_y: f64,
    /// Raw volume integral of `z²` about the global origin.
    pub volume_second_z: f64,
    /// Raw volume integral of `xy` about the global origin.
    pub volume_product_xy: f64,
    /// Raw volume integral of `xz` about the global origin.
    pub volume_product_xz: f64,
    /// Raw volume integral of `yz` about the global origin.
    pub volume_product_yz: f64,
    /// Area-weighted centroid x-component (for surface centroid, not solid CoM).
    pub centroid_x: f64,
    /// Area-weighted centroid y-component (for surface centroid, not solid CoM).
    pub centroid_y: f64,
    /// Area-weighted centroid z-component (for surface centroid, not solid CoM).
    pub centroid_z: f64,
}

/// Integrate a face's geometric contribution using Gauss quadrature.
///
/// For planar faces, evaluates via polygon fan triangulation. For
/// parametric surfaces (analytic and NURBS), evaluates the surface and its
/// partial derivatives on a Gauss-point grid over the UV domain derived
/// from the face's boundary vertices.
///
/// # Errors
///
/// Returns an error if topology entities are missing or the face has
/// insufficient geometry for integration.
#[allow(clippy::too_many_lines)]
pub fn integrate_face(
    topo: &Topology,
    face_id: FaceId,
    gauss_order: usize,
) -> Result<FaceContribution, CheckError> {
    let face = topo.face(face_id)?;
    let reversed = face.is_reversed();
    let sign = if reversed { -1.0 } else { 1.0 };

    match face.surface() {
        FaceSurface::Plane { normal, .. } => {
            let effective_normal = if reversed { -*normal } else { *normal };
            integrate_planar_face(topo, face_id, effective_normal)
        }
        FaceSurface::Cylinder(s) => {
            let full = (
                (0.0, std::f64::consts::TAU),
                (f64::NEG_INFINITY, f64::INFINITY),
            );
            let (u_range, v_range) = face_uv_bounds(topo, face_id, s, true, false, full)?;
            let uv_boundary = build_face_uv_boundary(topo, face_id, |p| s.project_point(p), true)?;
            Ok(integrate_with_trimming(
                s,
                u_range,
                v_range,
                gauss_order,
                sign,
                &uv_boundary,
                true,
                &[],
            ))
        }
        FaceSurface::Cone(s) => {
            let full = (
                (0.0, std::f64::consts::TAU),
                (f64::NEG_INFINITY, f64::INFINITY),
            );
            let (u_range, v_range) = face_uv_bounds(topo, face_id, s, true, false, full)?;
            let uv_boundary = build_face_uv_boundary(topo, face_id, |p| s.project_point(p), true)?;
            Ok(integrate_with_trimming(
                s,
                u_range,
                v_range,
                gauss_order,
                sign,
                &uv_boundary,
                true,
                &[],
            ))
        }
        FaceSurface::Sphere(s) => {
            let full = (
                (0.0, std::f64::consts::TAU),
                (-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2),
            );
            let (u_range, v_range) = face_uv_bounds(topo, face_id, s, true, false, full)?;
            let uv_boundary = build_face_uv_boundary(topo, face_id, |p| s.project_point(p), true)?;
            let hole_vs = full_revolution_hole_vs(topo, face_id, s);
            Ok(integrate_with_trimming(
                s,
                u_range,
                v_range,
                gauss_order,
                sign,
                &uv_boundary,
                true,
                &hole_vs,
            ))
        }
        FaceSurface::Torus(s) => {
            let full = ((0.0, std::f64::consts::TAU), (0.0, std::f64::consts::TAU));
            let (u_range, v_range) = face_uv_bounds(topo, face_id, s, true, true, full)?;
            let uv_boundary = build_face_uv_boundary(topo, face_id, |p| s.project_point(p), true)?;
            Ok(integrate_with_trimming(
                s,
                u_range,
                v_range,
                gauss_order,
                sign,
                &uv_boundary,
                true,
                &[],
            ))
        }
        FaceSurface::Nurbs(s) => {
            let full = (s.domain_u(), s.domain_v());
            let periodic_u = s.is_periodic_u();
            let periodic_v = s.is_periodic_v();
            let (u_range, v_range) =
                face_uv_bounds(topo, face_id, s, periodic_u, periodic_v, full)?;
            let uv_boundary =
                build_face_uv_boundary(topo, face_id, |p| s.project_point(p), periodic_u)?;
            Ok(integrate_with_trimming(
                s,
                u_range,
                v_range,
                gauss_order,
                sign,
                &uv_boundary,
                periodic_u,
                &[],
            ))
        }
    }
}

/// UV domain bounds as `((u_min, u_max), (v_min, v_max))`.
type UvBounds = ((f64, f64), (f64, f64));

/// The v-positions of a face's full-revolution inner wires (holes) on a
/// surface periodic in u.
///
/// A boolean that drills a cylinder through a sphere leaves each spherical
/// band bounded by a latitude circle hole (the tunnel rim). Such a hole wraps
/// the full u-period and sits at a single v, so the band runs from its outer
/// latitude to the hole — not on to the pole. Collecting these lets the
/// integrator clip the band instead of over-integrating the polar cap the hole
/// removed. Each entry is the mean projected v of one full-revolution hole.
fn full_revolution_hole_vs<S: ParametricSurface>(
    topo: &Topology,
    face_id: FaceId,
    surface: &S,
) -> Vec<f64> {
    use std::f64::consts::TAU;
    let Ok(face) = topo.face(face_id) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for &wid in face.inner_wires() {
        let Ok(wire) = topo.wire(wid) else { continue };
        let mut us = Vec::new();
        let mut vs = Vec::new();
        for oe in wire.edges() {
            let Ok(edge) = topo.edge(oe.edge()) else {
                continue;
            };
            // Oriented traversal: the wire-ordered start vertex is the edge's
            // end when the oriented edge is reversed.
            let vid = if oe.is_forward() {
                edge.start()
            } else {
                edge.end()
            };
            let Ok(v) = topo.vertex(vid) else {
                continue;
            };
            let (u, vv) = surface.project_point(v.point());
            us.push(u);
            vs.push(vv);
        }
        if vs.is_empty() {
            continue;
        }
        let v_min = vs.iter().copied().fold(f64::INFINITY, f64::min);
        let v_max = vs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        // Constant-v latitude circle.
        if v_max - v_min > 1e-6 {
            continue;
        }
        // Full revolution in u: the unwrapped per-vertex deltas around the
        // CLOSED loop (including the closing step back to the first vertex) sum
        // to ≈ TAU. A single-edge closed circle has one vertex, so also accept
        // holes whose sole edge is a closed circle curve.
        let unwrapped_span = {
            let n = us.len();
            let mut acc = 0.0;
            for i in 0..n {
                let d = us[(i + 1) % n] - us[i];
                acc += d - TAU * ((d + std::f64::consts::PI) / TAU).floor();
            }
            acc.abs()
        };
        let single_closed_circle = wire.edges().len() == 1
            && wire.edges().first().is_some_and(|oe| {
                topo.edge(oe.edge())
                    .is_ok_and(|e| matches!(e.curve(), EdgeCurve::Circle(_)))
            });
        if unwrapped_span >= TAU - 1e-3 || single_closed_circle {
            out.push(0.5 * (v_min + v_max));
        }
    }
    out
}

/// Compute UV bounds for a parametric face by projecting boundary vertices
/// onto the surface and taking the min/max of the resulting parameters.
///
/// For surfaces with periodic u or v coordinates (cylinders, cones, spheres,
/// tori), sequentially unwraps the angular coordinates so that faces straddling
/// the 0/2pi seam produce correct ranges.
///
/// When all projected vertices coincide (e.g. a full-revolution face),
/// `full_domain` is returned instead.
///
/// **Limitation:** Only the outer wire is used for UV bounds. Inner wires
/// (holes) are handled during Gauss integration by the UV containment check
/// in `integrate_parametric_trimmed`, but the current containment only tests
/// against the outer boundary. Faces with holes will over-integrate the hole
/// region. A proper fix requires multi-polygon UV containment (outer minus
/// holes).
fn face_uv_bounds<S: ParametricSurface>(
    topo: &Topology,
    face_id: FaceId,
    surface: &S,
    periodic_u: bool,
    periodic_v: bool,
    full_domain: UvBounds,
) -> Result<UvBounds, CheckError> {
    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;

    let mut uvs = Vec::new();
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let vid = oe.oriented_start(edge);
        let pt = topo.vertex(vid)?.point();
        uvs.push(surface.project_point(pt));
    }

    if uvs.is_empty() {
        return Err(CheckError::IntegrationFailed(
            "face wire has no edges".into(),
        ));
    }

    // Unwrap periodic coordinates sequentially so seam-straddling faces
    // produce a contiguous range instead of the full [0, 2pi).
    if periodic_u || periodic_v {
        for i in 1..uvs.len() {
            if periodic_u {
                uvs[i].0 = unwrap_angle(uvs[i - 1].0, uvs[i].0);
            }
            if periodic_v {
                uvs[i].1 = unwrap_angle(uvs[i - 1].1, uvs[i].1);
            }
        }
    }

    // Check for coincident vertices (all project to same point) — use full domain.
    let coincident = uvs.len() < 3 || {
        let ref_uv = uvs[0];
        uvs.iter()
            .all(|uv| (uv.0 - ref_uv.0).abs() < 1e-6 && (uv.1 - ref_uv.1).abs() < 1e-6)
    };
    if coincident {
        return Ok(full_domain);
    }

    let u_min = uvs.iter().map(|uv| uv.0).fold(f64::INFINITY, f64::min);
    let mut u_max = uvs.iter().map(|uv| uv.0).fold(f64::NEG_INFINITY, f64::max);
    let v_min = uvs.iter().map(|uv| uv.1).fold(f64::INFINITY, f64::min);
    let mut v_max = uvs.iter().map(|uv| uv.1).fold(f64::NEG_INFINITY, f64::max);

    // All boundary vertices on the seam of a periodic axis (e.g. a
    // full-revolution lateral face whose circles start/end at the seam)
    // collapse that axis's range to zero — the face actually spans the
    // full period.
    if periodic_u && u_max - u_min < 1e-9 {
        u_max = u_min + (full_domain.0.1 - full_domain.0.0);
    }
    if periodic_v && v_max - v_min < 1e-9 {
        v_max = v_min + (full_domain.1.1 - full_domain.1.0);
    }

    if u_min >= u_max || v_min >= v_max {
        // A degenerate projection (e.g. all boundary vertices on a sphere's
        // pole seam) does not mean an empty face — it means the boundary failed
        // to bound a sub-region, so the face spans the full analytic domain.
        return Ok(full_domain);
    }

    Ok(((u_min, u_max), (v_min, v_max)))
}

/// Unwrap a step in a periodic (angular) coordinate to avoid discontinuities.
///
/// Adjusts `next` so that `next - prev` lies in `(-pi, pi]`, keeping the
/// sequence monotonic through the 0/2pi seam.
fn unwrap_angle(prev: f64, next: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let diff = next - prev;
    prev + diff - tau * ((diff + std::f64::consts::PI) / tau).floor()
}

/// Integrate a planar face using polygon fan triangulation.
///
/// Inner wires (holes) are integrated the same way and subtracted from the
/// outer-wire contribution.
fn integrate_planar_face(
    topo: &Topology,
    face_id: FaceId,
    normal: Vec3,
) -> Result<FaceContribution, CheckError> {
    // Faces whose wires consist only of line and circular-arc edges take an
    // exact Green's-theorem boundary-integral path — a chord polygon
    // undercounts a circular cap by the sagitta area (~0.2% at the default
    // discretization), far above the accuracy of the parametric quadrature
    // the curved faces get.
    if let Some(contrib) = integrate_planar_face_exact(topo, face_id, normal)? {
        return Ok(contrib);
    }
    let polygon = crate::util::face_polygon(topo, face_id)?;
    let mut contrib = integrate_planar_polygon(&polygon, normal);

    let face = topo.face(face_id)?;
    let inner: Vec<_> = face.inner_wires().to_vec();
    for wid in inner {
        let hole = crate::util::wire_polygon(topo, wid)?;
        let h = integrate_planar_polygon(&hole, normal);
        contrib.area -= h.area;
        contrib.volume -= h.volume;
        contrib.volume_moment_x -= h.volume_moment_x;
        contrib.volume_moment_y -= h.volume_moment_y;
        contrib.volume_moment_z -= h.volume_moment_z;
        contrib.volume_second_x -= h.volume_second_x;
        contrib.volume_second_y -= h.volume_second_y;
        contrib.volume_second_z -= h.volume_second_z;
        contrib.volume_product_xy -= h.volume_product_xy;
        contrib.volume_product_xz -= h.volume_product_xz;
        contrib.volume_product_yz -= h.volume_product_yz;
        contrib.centroid_x -= h.centroid_x;
        contrib.centroid_y -= h.centroid_y;
        contrib.centroid_z -= h.centroid_z;
    }

    Ok(contrib)
}

/// Integrate a planar polygon's contribution via fan triangulation.
fn integrate_planar_polygon(polygon: &[Point3], normal: Vec3) -> FaceContribution {
    if polygon.len() < 3 {
        return FaceContribution {
            area: 0.0,
            volume: 0.0,
            volume_moment_x: 0.0,
            volume_moment_y: 0.0,
            volume_moment_z: 0.0,
            volume_second_x: 0.0,
            volume_second_y: 0.0,
            volume_second_z: 0.0,
            volume_product_xy: 0.0,
            volume_product_xz: 0.0,
            volume_product_yz: 0.0,
            centroid_x: 0.0,
            centroid_y: 0.0,
            centroid_z: 0.0,
        };
    }

    // Fan triangulation from vertex 0
    let mut area = 0.0;
    let mut vol = 0.0;
    let mut mx = 0.0;
    let mut my = 0.0;
    let mut mz = 0.0;
    let mut qxx = 0.0;
    let mut qyy = 0.0;
    let mut qzz = 0.0;
    let mut qxy = 0.0;
    let mut qxz = 0.0;
    let mut qyz = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;
    let mut cz = 0.0;

    for i in 1..polygon.len() - 1 {
        let (a, b, c) = (polygon[0], polygon[i], polygon[i + 1]);
        let ab = b - a;
        let ac = c - a;
        let cross = Vec3::new(
            ab.y() * ac.z() - ab.z() * ac.y(),
            ab.z() * ac.x() - ab.x() * ac.z(),
            ab.x() * ac.y() - ab.y() * ac.x(),
        );
        let tri_area = cross.length() * 0.5;
        area += tri_area;

        // Volume contribution: (1/3) * centroid dot normal * area
        let centroid = Point3::new(
            (a.x() + b.x() + c.x()) / 3.0,
            (a.y() + b.y() + c.y()) / 3.0,
            (a.z() + b.z() + c.z()) / 3.0,
        );
        let pv = Vec3::new(centroid.x(), centroid.y(), centroid.z());
        vol += pv.dot(normal) * tri_area / 3.0;

        // Volume moments via divergence theorem: (1/2) integral of x^2 * n_x dA
        // For a planar triangle with constant normal, integral of x^2 over triangle
        // = (area/3) * (x_a^2 + x_b^2 + x_c^2 + x_a*x_b + x_a*x_c + x_b*x_c) / 2
        // Simplified: use (x_a^2 + x_b^2 + x_c^2 + x_a*x_b + x_a*x_c + x_b*x_c)/6
        let avg_x2 = (a.x() * a.x()
            + b.x() * b.x()
            + c.x() * c.x()
            + a.x() * b.x()
            + a.x() * c.x()
            + b.x() * c.x())
            / 6.0;
        let avg_y2 = (a.y() * a.y()
            + b.y() * b.y()
            + c.y() * c.y()
            + a.y() * b.y()
            + a.y() * c.y()
            + b.y() * c.y())
            / 6.0;
        let avg_z2 = (a.z() * a.z()
            + b.z() * b.z()
            + c.z() * c.z()
            + a.z() * b.z()
            + a.z() * c.z()
            + b.z() * c.z())
            / 6.0;
        mx += 0.5 * avg_x2 * normal.x() * tri_area;
        my += 0.5 * avg_y2 * normal.y() * tri_area;
        mz += 0.5 * avg_z2 * normal.z() * tri_area;

        // Raw second moments and products via the divergence theorem. The
        // four-point Hammer rule used here is exact for the cubic monomials.
        qxx += normal.x() * triangle_cubic_integral(a, b, c, |p| p.x().powi(3)) / 3.0;
        qyy += normal.y() * triangle_cubic_integral(a, b, c, |p| p.y().powi(3)) / 3.0;
        qzz += normal.z() * triangle_cubic_integral(a, b, c, |p| p.z().powi(3)) / 3.0;
        qxy += normal.x() * triangle_cubic_integral(a, b, c, |p| p.x().powi(2) * p.y()) / 2.0;
        qxz += normal.x() * triangle_cubic_integral(a, b, c, |p| p.x().powi(2) * p.z()) / 2.0;
        qyz += normal.y() * triangle_cubic_integral(a, b, c, |p| p.y().powi(2) * p.z()) / 2.0;

        cx += centroid.x() * tri_area;
        cy += centroid.y() * tri_area;
        cz += centroid.z() * tri_area;
    }

    FaceContribution {
        area,
        volume: vol,
        volume_moment_x: mx,
        volume_moment_y: my,
        volume_moment_z: mz,
        volume_second_x: qxx,
        volume_second_y: qyy,
        volume_second_z: qzz,
        volume_product_xy: qxy,
        volume_product_xz: qxz,
        volume_product_yz: qyz,
        centroid_x: cx,
        centroid_y: cy,
        centroid_z: cz,
    }
}

/// Integrate a cubic-or-lower scalar function over a triangle exactly using
/// the four-point Hammer rule.
fn triangle_cubic_integral(a: Point3, b: Point3, c: Point3, f: impl Fn(Point3) -> f64) -> f64 {
    let area = (b - a).cross(c - a).length() * 0.5;
    let barycentric = |wa: f64, wb: f64, wc: f64| {
        Point3::new(
            wa.mul_add(a.x(), wb.mul_add(b.x(), wc * c.x())),
            wa.mul_add(a.y(), wb.mul_add(b.y(), wc * c.y())),
            wa.mul_add(a.z(), wb.mul_add(b.z(), wc * c.z())),
        )
    };
    let centroid = barycentric(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);
    let value = (-27.0 / 48.0) * f(centroid)
        + (25.0 / 48.0)
            * (f(barycentric(0.6, 0.2, 0.2))
                + f(barycentric(0.2, 0.6, 0.2))
                + f(barycentric(0.2, 0.2, 0.6)));
    area * value
}

/// Monomial basis for degree-≤3 polynomials in the in-plane coordinates
/// `(s, t)`: `[1, s, t, s², st, t², s³, s²t, st², t³]`.
type Poly2 = [f64; 10];

/// Multiply a degree-≤2 polynomial by the linear form `l₀ + l₁s + l₂t`.
///
/// The caller must ensure `p` has no cubic terms (indices 6..10 zero);
/// products above degree 3 are not representable and are silently dropped.
fn poly2_mul_linear(p: &Poly2, l: [f64; 3]) -> Poly2 {
    // Shift tables: monomial index k multiplied by s (resp. t).
    const S_SHIFT: [usize; 6] = [1, 3, 4, 6, 7, 8];
    const T_SHIFT: [usize; 6] = [2, 4, 5, 7, 8, 9];
    let mut out = [0.0; 10];
    for k in 0..10 {
        out[k] += l[0] * p[k];
    }
    for k in 0..6 {
        out[S_SHIFT[k]] += l[1] * p[k];
        out[T_SHIFT[k]] += l[2] * p[k];
    }
    out
}

/// Dot a polynomial's coefficients with the region's monomial moments.
fn poly2_integrate(p: &Poly2, moments: &[f64; 10]) -> f64 {
    p.iter().zip(moments.iter()).map(|(c, m)| c * m).sum()
}

/// Region monomial moments `∫∫ sⁱtʲ ds dt` of one wire's enclosed planar
/// region via Green's theorem: `M_ij = ∮ s^{i+1}/(i+1) · tʲ dt`.
///
/// Returns `None` if the wire contains any edge that is not a line or a
/// circular arc (the exact path only handles those); the caller falls back
/// to chord-polygon integration. The result is winding-aligned so that the
/// area moment `M₀₀` is positive.
fn planar_wire_monomial_moments(
    topo: &Topology,
    wire_id: brepkit_topology::wire::WireId,
    origin: Point3,
    e1: Vec3,
    e2: Vec3,
) -> Result<Option<[f64; 10]>, CheckError> {
    let wire = topo.wire(wire_id)?;
    let mut moments = [0.0; 10];
    let mut prev_end: Option<brepkit_topology::vertex::VertexId> = None;

    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let start_vid = edge.start();
        let end_vid = edge.end();
        // Wires store edges in loop order, but per-edge orientation flags are
        // not guaranteed to chain head-to-tail; re-derive traversal direction
        // from vertex connectivity with the previous edge (same convention as
        // `util::wire_polygon`).
        let forward = match prev_end {
            Some(pe) if start_vid == pe && end_vid != pe => true,
            Some(pe) if end_vid == pe && start_vid != pe => false,
            _ => oe.is_forward(),
        };
        prev_end = Some(if forward { end_vid } else { start_vid });

        let start = topo.vertex(start_vid)?.point();
        let end = topo.vertex(end_vid)?.point();
        let dir_sign = if forward { 1.0 } else { -1.0 };

        match edge.curve() {
            EdgeCurve::Line => {
                // P(u) = start + (end - start)·u, u ∈ [0, 1].
                let d = end - start;
                accumulate_green_segment(
                    &mut moments,
                    (0.0, 1.0),
                    8,
                    dir_sign,
                    |u| (start + d * u, d),
                    origin,
                    e1,
                    e2,
                );
            }
            EdgeCurve::Circle(c) => {
                // Angular arc span from the edge's own endpoints; the
                // derivative magnitude is the radius.
                let (t0, t1) = edge.curve().domain_with_endpoints(start, end);
                let r = c.radius();
                // Split the span so each chunk is ≤ π/2; 16-point Gauss on
                // a ≤ π/2 trig span of frequency ≤ 5 is exact to machine
                // precision.
                let chunks =
                    (((t1 - t0).abs() / std::f64::consts::FRAC_PI_2).ceil() as usize).clamp(1, 8);
                let dt = (t1 - t0) / chunks as f64;
                for i in 0..chunks {
                    let a = dt.mul_add(i as f64, t0);
                    accumulate_green_segment(
                        &mut moments,
                        (a, a + dt),
                        16,
                        dir_sign,
                        |u| (c.evaluate(u), c.tangent(u) * r),
                        origin,
                        e1,
                        e2,
                    );
                }
            }
            EdgeCurve::Ellipse(_) | EdgeCurve::NurbsCurve(_) => return Ok(None),
        }
    }

    // Align winding so the enclosed area is positive.
    if moments[0] < 0.0 {
        for m in &mut moments {
            *m = -*m;
        }
    }
    Ok(Some(moments))
}

/// Accumulate one boundary segment's Green's-theorem contribution to the
/// region monomial moments with Gauss-Legendre quadrature.
///
/// `eval(u)` returns the curve point and its derivative `dP/du`; the
/// integrand for `M_ij` is `s^{i+1}/(i+1) · tʲ · t'(u)` with
/// `s = (P - origin)·e1`, `t = (P - origin)·e2`.
#[allow(clippy::too_many_arguments)]
fn accumulate_green_segment<F>(
    moments: &mut [f64; 10],
    range: (f64, f64),
    gauss_order: usize,
    dir_sign: f64,
    eval: F,
    origin: Point3,
    e1: Vec3,
    e2: Vec3,
) where
    F: Fn(f64) -> (Point3, Vec3),
{
    // Monomial exponents (i, j) matching the `Poly2` basis order.
    const EXPONENTS: [(i32, i32); 10] = [
        (0, 0),
        (1, 0),
        (0, 1),
        (2, 0),
        (1, 1),
        (0, 2),
        (3, 0),
        (2, 1),
        (1, 2),
        (0, 3),
    ];

    let scale = (range.1 - range.0) / 2.0;
    let mid = f64::midpoint(range.0, range.1);
    for gp in gauss_legendre_points(gauss_order) {
        let u = scale.mul_add(gp.x, mid);
        let (p, dp) = eval(u);
        let rel = p - origin;
        let s = rel.dot(e1);
        let t = rel.dot(e2);
        let dt_du = dp.dot(e2);
        let w = gp.w * scale * dir_sign * dt_du;
        for (k, &(i, j)) in EXPONENTS.iter().enumerate() {
            moments[k] += w * s.powi(i + 1) / f64::from(i + 1) * t.powi(j);
        }
    }
}

/// Newell normal of a wire's boundary, sampled densely enough that a wire
/// consisting of a single closed circle (one vertex) still determines its
/// plane. Returns `None` when the boundary is degenerate (collapsed to a
/// point or line) or contains an edge type the exact path does not handle.
fn wire_newell_normal(
    topo: &Topology,
    wire_id: brepkit_topology::wire::WireId,
) -> Result<Option<Vec3>, CheckError> {
    let wire = topo.wire(wire_id)?;
    let mut pts: Vec<Point3> = Vec::new();
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let start = topo.vertex(edge.start())?.point();
        let end = topo.vertex(edge.end())?.point();
        match edge.curve() {
            EdgeCurve::Line => pts.push(start),
            EdgeCurve::Circle(c) => {
                let (t0, t1) = edge.curve().domain_with_endpoints(start, end);
                for k in 0..4 {
                    pts.push(c.evaluate(t0 + (t1 - t0) * f64::from(k) / 4.0));
                }
            }
            EdgeCurve::Ellipse(_) | EdgeCurve::NurbsCurve(_) => return Ok(None),
        }
    }
    if pts.len() < 3 {
        return Ok(None);
    }
    let (mut nx, mut ny, mut nz) = (0.0, 0.0, 0.0);
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        nx += (a.y() - b.y()) * (a.z() + b.z());
        ny += (a.z() - b.z()) * (a.x() + b.x());
        nz += (a.x() - b.x()) * (a.y() + b.y());
    }
    let n = Vec3::new(nx, ny, nz);
    if n.length() < 1e-12 {
        return Ok(None);
    }
    Ok(Some(n))
}

/// Exact planar-face integration via Green's-theorem boundary integrals.
///
/// Returns `Ok(None)` when any wire contains an edge type the exact path
/// does not handle (ellipse or NURBS), or when the boundary is too
/// degenerate to determine its plane; the caller then falls back to the
/// chord-polygon fan path for the whole face.
fn integrate_planar_face_exact(
    topo: &Topology,
    face_id: FaceId,
    normal: Vec3,
) -> Result<Option<FaceContribution>, CheckError> {
    let face = topo.face(face_id)?;
    let outer_wire = face.outer_wire();
    let inner: Vec<_> = face.inner_wires().to_vec();

    // In-plane frame anchored at the first boundary vertex. The frame's
    // normal is derived from the boundary geometry itself (Newell), NOT the
    // face's stored plane normal: a malformed face whose stored normal is
    // inconsistent with its boundary would otherwise project to a collapsed
    // (zero-area) region, where the chord-polygon fan path still measures
    // the true geometric area. The flux terms below keep using the passed
    // `normal`, exactly like the fan path.
    let wire = topo.wire(outer_wire)?;
    let Some(first_edge) = wire.edges().first() else {
        return Ok(None);
    };
    let origin = topo.vertex(topo.edge(first_edge.edge())?.start())?.point();
    let Some(boundary_normal) = wire_newell_normal(topo, outer_wire)? else {
        return Ok(None);
    };
    let Ok(frame) = brepkit_math::frame::Frame3::from_normal(origin, boundary_normal) else {
        return Ok(None);
    };
    let (e1, e2) = (frame.x, frame.y);

    let Some(mut moments) = planar_wire_monomial_moments(topo, outer_wire, origin, e1, e2)? else {
        return Ok(None);
    };
    for wid in inner {
        let Some(hole) = planar_wire_monomial_moments(topo, wid, origin, e1, e2)? else {
            return Ok(None);
        };
        for k in 0..10 {
            moments[k] -= hole[k];
        }
    }

    // Linear forms of the global coordinates in the in-plane basis:
    // x = origin.x + e1.x·s + e2.x·t, etc.
    let lx = [origin.x(), e1.x(), e2.x()];
    let ly = [origin.y(), e1.y(), e2.y()];
    let lz = [origin.z(), e1.z(), e2.z()];
    let lin = |l: [f64; 3]| -> Poly2 {
        let mut p = [0.0; 10];
        p[0] = l[0];
        p[1] = l[1];
        p[2] = l[2];
        p
    };
    let (px, py, pz) = (lin(lx), lin(ly), lin(lz));
    let x2 = poly2_mul_linear(&px, lx);
    let y2 = poly2_mul_linear(&py, ly);
    let z2 = poly2_mul_linear(&pz, lz);
    let ig = |p: &Poly2| poly2_integrate(p, &moments);

    let area = moments[0];
    let (ix, iy, iz) = (ig(&px), ig(&py), ig(&pz));
    Ok(Some(FaceContribution {
        area,
        volume: (normal.x() * ix + normal.y() * iy + normal.z() * iz) / 3.0,
        volume_moment_x: 0.5 * normal.x() * ig(&x2),
        volume_moment_y: 0.5 * normal.y() * ig(&y2),
        volume_moment_z: 0.5 * normal.z() * ig(&z2),
        volume_second_x: normal.x() * ig(&poly2_mul_linear(&x2, lx)) / 3.0,
        volume_second_y: normal.y() * ig(&poly2_mul_linear(&y2, ly)) / 3.0,
        volume_second_z: normal.z() * ig(&poly2_mul_linear(&z2, lz)) / 3.0,
        volume_product_xy: 0.5 * normal.x() * ig(&poly2_mul_linear(&x2, ly)),
        volume_product_xz: 0.5 * normal.x() * ig(&poly2_mul_linear(&x2, lz)),
        volume_product_yz: 0.5 * normal.y() * ig(&poly2_mul_linear(&y2, lz)),
        centroid_x: ix,
        centroid_y: iy,
        centroid_z: iz,
    }))
}

/// Integrate a parametric surface using Gauss quadrature over the UV domain.
#[allow(clippy::cast_precision_loss)]
fn integrate_parametric<S: ParametricSurface>(
    surface: &S,
    u_range: (f64, f64),
    v_range: (f64, f64),
    gauss_order: usize,
    sign: f64,
) -> FaceContribution {
    // Composite quadrature: tile the domain into patches no larger than ~PI/4
    // so one Gauss rule resolves curved and periodic integrands. A single patch
    // over a torus's full 2*PI period in both u and v under-resolves it (~0.5%
    // error); several patches per period converge to machine precision. The
    // patch count is capped so a long *linear* axis (e.g. a tall cylinder/cone
    // whose v is axial distance) cannot make integration cost scale with model
    // size — its integrand is low-degree, so a bounded number of patches stays
    // exact. Angular axes never exceed 2*PI (= 8 patches), well under the cap.
    const MAX_PATCHES: usize = 16;

    let gauss_pts = gauss_legendre_points(gauss_order);
    let patch = std::f64::consts::FRAC_PI_4;
    let nu = (((u_range.1 - u_range.0).abs() / patch).ceil() as usize).clamp(1, MAX_PATCHES);
    let nv = (((v_range.1 - v_range.0).abs() / patch).ceil() as usize).clamp(1, MAX_PATCHES);
    let du_patch = (u_range.1 - u_range.0) / nu as f64;
    let dv_patch = (v_range.1 - v_range.0) / nv as f64;
    let u_scale = du_patch / 2.0;
    let v_scale = dv_patch / 2.0;

    let mut area = 0.0;
    let mut vol = 0.0;
    let mut mx = 0.0;
    let mut my = 0.0;
    let mut mz = 0.0;
    let mut qxx = 0.0;
    let mut qyy = 0.0;
    let mut qzz = 0.0;
    let mut qxy = 0.0;
    let mut qxz = 0.0;
    let mut qyz = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;
    let mut cz = 0.0;

    for iu in 0..nu {
        let u_mid = du_patch.mul_add(iu as f64, u_range.0) + u_scale;
        for iv in 0..nv {
            let v_mid = dv_patch.mul_add(iv as f64, v_range.0) + v_scale;
            for gpu in gauss_pts {
                let u = u_scale.mul_add(gpu.x, u_mid);
                for gpv in gauss_pts {
                    let v = v_scale.mul_add(gpv.x, v_mid);
                    let w = gpu.w * gpv.w * u_scale * v_scale;

                    let p = surface.evaluate(u, v);
                    let du = surface.partial_u(u, v);
                    let dv = surface.partial_v(u, v);

                    // Normal = du x dv (unnormalized, includes Jacobian)
                    let n = Vec3::new(
                        du.y() * dv.z() - du.z() * dv.y(),
                        du.z() * dv.x() - du.x() * dv.z(),
                        du.x() * dv.y() - du.y() * dv.x(),
                    );
                    let n_len = n.length();

                    area += w * n_len;

                    // Volume: (1/3) P dot N (unnormalized N includes Jacobian)
                    let pv = Vec3::new(p.x(), p.y(), p.z());
                    vol += w * pv.dot(n) / 3.0;

                    // Volume moments via divergence theorem:
                    // CoM_x = (1/2V) surface_integral(x^2 * n_x dA)
                    // n already includes Jacobian, so n.x() = N_x * |J|
                    mx += w * 0.5 * p.x() * p.x() * n.x();
                    my += w * 0.5 * p.y() * p.y() * n.y();
                    mz += w * 0.5 * p.z() * p.z() * n.z();

                    qxx += w * p.x().powi(3) * n.x() / 3.0;
                    qyy += w * p.y().powi(3) * n.y() / 3.0;
                    qzz += w * p.z().powi(3) * n.z() / 3.0;
                    qxy += w * 0.5 * p.x().powi(2) * p.y() * n.x();
                    qxz += w * 0.5 * p.x().powi(2) * p.z() * n.x();
                    qyz += w * 0.5 * p.y().powi(2) * p.z() * n.y();

                    cx += w * p.x() * n_len;
                    cy += w * p.y() * n_len;
                    cz += w * p.z() * n_len;
                }
            }
        }
    }

    FaceContribution {
        area,
        volume: vol * sign,
        volume_moment_x: mx * sign,
        volume_moment_y: my * sign,
        volume_moment_z: mz * sign,
        volume_second_x: qxx * sign,
        volume_second_y: qyy * sign,
        volume_second_z: qzz * sign,
        volume_product_xy: qxy * sign,
        volume_product_xz: qxz * sign,
        volume_product_yz: qyz * sign,
        centroid_x: cx,
        centroid_y: cy,
        centroid_z: cz,
    }
}

/// Absolute shoelace area of a UV polygon. Near-zero means the boundary has
/// collapsed onto a line or point (a degenerate seam/pole projection).
fn polygon_area(poly: &[(f64, f64)]) -> f64 {
    let n = poly.len();
    if n < 3 {
        return 0.0;
    }
    let mut a = 0.0;
    for i in 0..n {
        let (x0, y0) = poly[i];
        let (x1, y1) = poly[(i + 1) % n];
        a += x0 * y1 - x1 * y0;
    }
    (a * 0.5).abs()
}

/// Dispatch to trimmed or untrimmed parametric integration based on whether
/// a UV boundary polygon is available.
#[allow(clippy::too_many_arguments)]
fn integrate_with_trimming<S: ParametricSurface>(
    surface: &S,
    u_range: (f64, f64),
    v_range: (f64, f64),
    gauss_order: usize,
    sign: f64,
    uv_boundary: &[(f64, f64)],
    u_periodic: bool,
    hole_vs: &[f64],
) -> FaceContribution {
    if uv_boundary.len() < 3 {
        return integrate_parametric(surface, u_range, v_range, gauss_order, sign);
    }

    // The dense boundary polygon is the reliable signal for a face's true
    // parametric extent: `face_uv_bounds` samples only sparse edge endpoints and
    // under-spans full-revolution faces (a cone's lateral face reports a narrow
    // u-range though its boundary wraps the full 2pi). A face that wraps the
    // full period in u, or whose boundary collapses onto a seam or pole, cannot
    // be trimmed by a UV polygon — the apex/pole/seam folds the polygon and the
    // point-in-polygon test rejects valid interior samples. Integrate the
    // analytic surface untrimmed over its true domain in those cases.
    let u_min = uv_boundary
        .iter()
        .map(|p| p.0)
        .fold(f64::INFINITY, f64::min);
    let v_min = uv_boundary
        .iter()
        .map(|p| p.1)
        .fold(f64::INFINITY, f64::min);
    let v_max = uv_boundary
        .iter()
        .map(|p| p.1)
        .fold(f64::NEG_INFINITY, f64::max);

    // Winding number of the boundary around the periodic u-axis: ±TAU for a
    // face that wraps a full revolution, ~0 for a partially-trimmed face.
    // Computed from shortest signed steps so it is independent of the
    // boundary's discretization (segment count).
    let tau = std::f64::consts::TAU;
    let winding: f64 = (0..uv_boundary.len())
        .map(|i| {
            let d = uv_boundary[(i + 1) % uv_boundary.len()].0 - uv_boundary[i].0;
            d - tau * ((d + std::f64::consts::PI) / tau).floor()
        })
        .sum();
    let full_revolution = u_periodic && winding.abs() >= tau - 1e-3;
    let v_degenerate = (v_max - v_min) <= 1e-9;

    if full_revolution && v_degenerate {
        // Polar cap (e.g. a sphere hemisphere bounded only by one latitude
        // circle): the cap runs from that latitude to a pole. The winding sign
        // (CCW vs CW boundary) selects which pole — the boundary's interior
        // side — so the two hemispheres do not both integrate the whole sphere.
        let v_pole = if winding >= 0.0 { v_range.1 } else { v_range.0 };
        // A full-revolution hole at a latitude between the outer circle and the
        // pole (the drilled-tunnel rim) clips the cap into a band: integrate
        // only from the outer latitude to the hole, not on to the pole.
        let v_far = hole_vs
            .iter()
            .copied()
            // Same side of v_min as the pole (strict same sign → positive
            // product), and not coincident with v_min.
            .filter(|&hv| (hv - v_min) * (v_pole - v_min) > 0.0 && (hv - v_min).abs() > 1e-9)
            .min_by(|a, b| (a - v_min).abs().total_cmp(&(b - v_min).abs()))
            .unwrap_or(v_pole);
        let v_dom = (v_min.min(v_far), v_min.max(v_far));
        integrate_parametric(surface, (u_min, u_min + tau), v_dom, gauss_order, sign)
    } else if full_revolution {
        // Full-revolution band (cone/cylinder): integrate the whole revolution
        // over the band's v-extent.
        integrate_parametric(
            surface,
            (u_min, u_min + tau),
            (v_min, v_max),
            gauss_order,
            sign,
        )
    } else if polygon_area(uv_boundary) <= 1e-12 {
        // Collapsed polygon (e.g. a closed torus whose seam projects to a
        // point): trust the analytic full-domain range from `face_uv_bounds`.
        integrate_parametric(surface, u_range, v_range, gauss_order, sign)
    } else {
        integrate_parametric_trimmed(
            surface,
            u_range,
            v_range,
            gauss_order,
            sign,
            uv_boundary,
            u_periodic,
        )
    }
}

/// Integrate a parametric surface with UV boundary trimming.
///
/// At each Gauss point, checks if the (u,v) coordinate falls inside the
/// face's UV boundary polygon. Points outside are skipped (zero contribution).
#[allow(clippy::cast_precision_loss, clippy::too_many_lines)]
fn integrate_parametric_trimmed<S: ParametricSurface>(
    surface: &S,
    u_range: (f64, f64),
    v_range: (f64, f64),
    gauss_order: usize,
    sign: f64,
    uv_boundary: &[(f64, f64)],
    u_periodic: bool,
) -> FaceContribution {
    use brepkit_math::predicates::point_in_polygon;
    use brepkit_math::vec::Point2;

    // Composite quadrature over patches no larger than ~PI/4, mirroring
    // `integrate_parametric`: one Gauss rule over a full 2*PI period badly
    // under-resolves trigonometric moment integrands (a cylinder wall's
    // second moments were ~5% off with a single order-5 grid).
    const MAX_PATCHES: usize = 16;

    let gauss_pts = gauss_legendre_points(gauss_order);
    let patch = std::f64::consts::FRAC_PI_4;
    let nu = (((u_range.1 - u_range.0).abs() / patch).ceil() as usize).clamp(1, MAX_PATCHES);
    let nv = (((v_range.1 - v_range.0).abs() / patch).ceil() as usize).clamp(1, MAX_PATCHES);
    let du_patch = (u_range.1 - u_range.0) / nu as f64;
    let dv_patch = (v_range.1 - v_range.0) / nv as f64;
    let u_scale = du_patch / 2.0;
    let v_scale = dv_patch / 2.0;

    let uv_poly: Vec<Point2> = uv_boundary
        .iter()
        .map(|(u, v)| Point2::new(*u, *v))
        .collect();

    let u_bcenter = if u_periodic {
        let bmin = uv_boundary
            .iter()
            .map(|(bu, _)| *bu)
            .fold(f64::INFINITY, f64::min);
        let bmax = uv_boundary
            .iter()
            .map(|(bu, _)| *bu)
            .fold(f64::NEG_INFINITY, f64::max);
        (bmin + bmax) * 0.5
    } else {
        0.0
    };

    let mut area = 0.0;
    let mut vol = 0.0;
    let mut mx = 0.0;
    let mut my = 0.0;
    let mut mz = 0.0;
    let mut qxx = 0.0;
    let mut qyy = 0.0;
    let mut qzz = 0.0;
    let mut qxy = 0.0;
    let mut qxz = 0.0;
    let mut qyz = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;
    let mut cz = 0.0;

    for iu in 0..nu {
        let u_mid = du_patch.mul_add(iu as f64, u_range.0) + u_scale;
        for iv in 0..nv {
            let v_mid = dv_patch.mul_add(iv as f64, v_range.0) + v_scale;
            for gpu in gauss_pts {
                let u = u_scale.mul_add(gpu.x, u_mid);
                for gpv in gauss_pts {
                    let v = v_scale.mul_add(gpv.x, v_mid);

                    let test_u = if u_periodic {
                        let tau = std::f64::consts::TAU;
                        let diff = u - u_bcenter;
                        u_bcenter + diff - tau * ((diff + std::f64::consts::PI) / tau).floor()
                    } else {
                        u
                    };

                    if !point_in_polygon(Point2::new(test_u, v), &uv_poly) {
                        continue;
                    }

                    let w = gpu.w * gpv.w * u_scale * v_scale;
                    let p = surface.evaluate(u, v);
                    let du = surface.partial_u(u, v);
                    let dv = surface.partial_v(u, v);
                    let n = Vec3::new(
                        du.y() * dv.z() - du.z() * dv.y(),
                        du.z() * dv.x() - du.x() * dv.z(),
                        du.x() * dv.y() - du.y() * dv.x(),
                    );
                    let n_len = n.length();

                    area += w * n_len;

                    let pv = Vec3::new(p.x(), p.y(), p.z());
                    vol += w * pv.dot(n) / 3.0;

                    mx += w * 0.5 * p.x() * p.x() * n.x();
                    my += w * 0.5 * p.y() * p.y() * n.y();
                    mz += w * 0.5 * p.z() * p.z() * n.z();

                    qxx += w * p.x().powi(3) * n.x() / 3.0;
                    qyy += w * p.y().powi(3) * n.y() / 3.0;
                    qzz += w * p.z().powi(3) * n.z() / 3.0;
                    qxy += w * 0.5 * p.x().powi(2) * p.y() * n.x();
                    qxz += w * 0.5 * p.x().powi(2) * p.z() * n.x();
                    qyz += w * 0.5 * p.y().powi(2) * p.z() * n.y();

                    cx += w * p.x() * n_len;
                    cy += w * p.y() * n_len;
                    cz += w * p.z() * n_len;
                }
            }
        }
    }

    FaceContribution {
        area,
        volume: vol * sign,
        volume_moment_x: mx * sign,
        volume_moment_y: my * sign,
        volume_moment_z: mz * sign,
        volume_second_x: qxx * sign,
        volume_second_y: qyy * sign,
        volume_second_z: qzz * sign,
        volume_product_xy: qxy * sign,
        volume_product_xz: qxz * sign,
        volume_product_yz: qyz * sign,
        centroid_x: cx,
        centroid_y: cy,
        centroid_z: cz,
    }
}

/// Build a UV boundary polygon from a face's outer wire.
///
/// Projects each boundary vertex onto the surface to obtain (u, v) coordinates,
/// then unwraps periodic u-coordinates to avoid seam discontinuities.
fn build_face_uv_boundary<F>(
    topo: &Topology,
    face_id: FaceId,
    project: F,
    u_periodic: bool,
) -> Result<Vec<(f64, f64)>, CheckError>
where
    F: Fn(Point3) -> (f64, f64),
{
    let polygon = crate::util::face_polygon(topo, face_id)?;
    if polygon.len() < 3 {
        return Ok(vec![]);
    }

    let mut uv: Vec<(f64, f64)> = polygon.iter().map(|&p| project(p)).collect();

    for i in 1..uv.len() {
        if u_periodic {
            uv[i].0 = unwrap_angle(uv[i - 1].0, uv[i].0);
        }
    }

    Ok(uv)
}
