// Walking engine infrastructure — used progressively as more blend paths are wired up.
#![allow(dead_code)]
//! Spherical triangle corner patches for vertex blends.
//!
//! At vertices where 3+ fillet stripes meet, a gap appears that needs
//! filling with a smooth surface patch. This module builds exact rational
//! NURBS patches bounded by great-circle arcs on the rolling-ball sphere,
//! producing watertight corners with no overlap by construction.

use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_topology::face::FaceSurface;
use remus_topology::vertex::VertexId;

use crate::BlendError;

/// Tolerance for geometric checks.
const TOL: f64 = 1e-6;

/// Input data for building a spherical corner patch at a vertex.
pub struct VertexContactData {
    /// Position of the vertex in 3D space.
    pub vertex_pos: Point3,
    /// Contact points from each stripe endpoint at this vertex.
    pub contact_points: Vec<Point3>,
    /// Inward-facing normals of the faces meeting at the vertex.
    pub face_normals: Vec<Vec3>,
    /// Fillet radius (same for all stripes at this vertex).
    pub radius: f64,
    /// True if the vertex is convex (material on inside).
    pub is_convex: bool,
    /// Vertex ID for error reporting.
    pub vertex_id: VertexId,
    /// The exact corner ball `(center, radius)` when the caller solved it
    /// from face tangency (see `corner::tangent_corner_ball`). When absent,
    /// the legacy normal-sum heuristic estimates it from the contacts — that
    /// estimate is only correct for mutually orthogonal faces with set-back
    /// contacts, and produces a √2·R ball from unset-back ones.
    pub ball: Option<(Point3, f64)>,
}

/// Result of building a spherical corner patch.
pub struct SphericalCornerResult {
    /// The NURBS surface patch filling the corner gap.
    pub surface: FaceSurface,
    /// The 3 boundary arcs (great-circle arcs on the sphere).
    pub boundary_curves: Vec<NurbsCurve>,
}

/// Compute the sphere center and actual sphere radius from vertex
/// position, face normals, and the fillet radius.
///
/// The center is offset from the vertex by `R * sum(face_normals)` —
/// each face contributes an independent offset of R along its inward
/// normal.  The actual sphere radius (distance from center to contact
/// points) may differ from the fillet radius when the faces are not
/// mutually orthogonal.
///
/// Returns `(center, sphere_radius)`.
fn compute_sphere_center(data: &VertexContactData) -> Result<(Point3, f64), BlendError> {
    if let Some((center, radius)) = data.ball {
        if radius < TOL {
            return Err(BlendError::CornerFailure {
                vertex: data.vertex_id,
            });
        }
        return Ok((center, radius));
    }

    let mut normal_sum = Vec3::new(0.0, 0.0, 0.0);
    for n in &data.face_normals {
        normal_sum += *n;
    }
    let len = normal_sum.length();
    if len < TOL {
        return Err(BlendError::CornerFailure {
            vertex: data.vertex_id,
        });
    }

    // Each face pushes the sphere center by R along its normal.
    let offset = normal_sum * data.radius;

    let center = if data.is_convex {
        data.vertex_pos + offset
    } else {
        data.vertex_pos - offset
    };

    if data.contact_points.is_empty() {
        return Err(BlendError::CornerFailure {
            vertex: data.vertex_id,
        });
    }
    let sphere_radius = (data.contact_points[0] - center).length();
    if sphere_radius < TOL {
        return Err(BlendError::CornerFailure {
            vertex: data.vertex_id,
        });
    }

    // Validate: all contact points should be at the same distance from center.
    for cp in &data.contact_points {
        let dist = (*cp - center).length();
        let err = (dist - sphere_radius).abs();
        if err > TOL * 100.0 {
            return Err(BlendError::CornerFailure {
                vertex: data.vertex_id,
            });
        }
    }

    Ok((center, sphere_radius))
}

/// Build a rational quadratic NURBS curve representing a great-circle
/// arc on the sphere from `q_start` to `q_end` centered at `center`.
fn build_great_circle_arc(
    center: Point3,
    radius: f64,
    q_start: Point3,
    q_end: Point3,
    vertex_id: VertexId,
) -> Result<NurbsCurve, BlendError> {
    let dir_i = (q_start - center) * (1.0 / radius);
    let dir_j = (q_end - center) * (1.0 / radius);

    let bisector_raw = dir_i + dir_j;
    let bisector_len = bisector_raw.length();
    if bisector_len < TOL {
        return Err(BlendError::CornerFailure { vertex: vertex_id });
    }
    let bisector = bisector_raw * (1.0 / bisector_len);

    let cos_half = dir_i.dot(bisector);
    if cos_half.abs() < TOL {
        return Err(BlendError::CornerFailure { vertex: vertex_id });
    }
    // Tangent intersection point (the middle control point in 3D).
    let mid_cp = center + bisector * (radius / cos_half);

    let control_points = vec![q_start, mid_cp, q_end];
    let weights = vec![1.0, cos_half, 1.0];
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];

    Ok(NurbsCurve::new(2, knots, control_points, weights)?)
}

/// Build a spherical triangle corner patch for a standard 3-edge vertex.
///
/// The result is an exact analytic sphere face on the corner ball, bounded
/// by three great-circle arcs connecting the contact points. (An earlier
/// revision approximated the patch with a degree-(2,2) rational NURBS whose
/// heuristic apex sagged several percent of R mid-patch; the analytic
/// surface removed both that error and the degenerate-corner fold.)
///
/// # Errors
///
/// Returns `BlendError::CornerFailure` if the geometry is degenerate
/// (e.g. coplanar face normals, contact points not on the sphere).
pub fn build_spherical_corner(
    data: &VertexContactData,
) -> Result<SphericalCornerResult, BlendError> {
    if data.contact_points.len() < 3 {
        return Err(BlendError::CornerFailure {
            vertex: data.vertex_id,
        });
    }

    let (center, r) = compute_sphere_center(data)?;

    build_triangle_on_sphere(
        center,
        r,
        data.contact_points[0],
        data.contact_points[1],
        data.contact_points[2],
        data.vertex_id,
    )
}

/// Build spherical corner patches for a vertex with N > 3 edges.
///
/// Uses centroid-fan triangulation: computes the centroid of the contact
/// points projected onto the sphere, then builds N spherical triangles
/// each spanning (centroid, Q_i, Q_{i+1}).
///
/// # Errors
///
/// Returns `BlendError::CornerFailure` if any triangle patch fails to build.
pub fn build_n_edge_corner(
    data: &VertexContactData,
) -> Result<Vec<SphericalCornerResult>, BlendError> {
    let n = data.contact_points.len();
    if n < 3 {
        return Err(BlendError::CornerFailure {
            vertex: data.vertex_id,
        });
    }

    let (center, r) = compute_sphere_center(data)?;

    // Centroid of contact points, projected onto the sphere.
    let mut centroid_raw = Vec3::new(0.0, 0.0, 0.0);
    for p in &data.contact_points {
        centroid_raw += *p - center;
    }
    centroid_raw = centroid_raw * (1.0 / n as f64);

    let centroid_len = centroid_raw.length();
    if centroid_len < TOL {
        return Err(BlendError::CornerFailure {
            vertex: data.vertex_id,
        });
    }
    let centroid = center + centroid_raw * (r / centroid_len);

    let mut results = Vec::with_capacity(n);

    for i in 0..n {
        let j = (i + 1) % n;
        let qi = data.contact_points[i];
        let qj = data.contact_points[j];

        let result = build_triangle_on_sphere(center, r, qi, qj, centroid, data.vertex_id)?;
        results.push(result);
    }

    Ok(results)
}

/// Build a single spherical triangle patch given three points already on the sphere.
///
/// The surface is the exact analytic sphere, oriented so that both chart
/// singularities stay away from the patch: the poles go 90° off the patch
/// centroid direction and the `u`-seam to its antipode, keeping UV
/// projection of the patch continuous for anything smaller than a
/// hemisphere.
fn build_triangle_on_sphere(
    center: Point3,
    radius: f64,
    q1: Point3,
    q2: Point3,
    q3: Point3,
    vertex_id: VertexId,
) -> Result<SphericalCornerResult, BlendError> {
    let r = radius;

    let dir1 = (q1 - center) * (1.0 / r);
    let dir2 = (q2 - center) * (1.0 / r);
    let dir3 = (q3 - center) * (1.0 / r);

    let mean_raw = dir1 + dir2 + dir3;
    if mean_raw.length() < TOL {
        return Err(BlendError::CornerFailure { vertex: vertex_id });
    }
    let mean_dir = mean_raw
        .normalize()
        .map_err(|_| BlendError::CornerFailure { vertex: vertex_id })?;
    let polar = remus_math::frame::Frame3::from_normal(center, mean_dir)
        .map_err(|_| BlendError::CornerFailure { vertex: vertex_id })?
        .x;
    let sphere = remus_math::surfaces::SphericalSurface::with_frame(center, r, polar, -mean_dir)
        .map_err(|_| BlendError::CornerFailure { vertex: vertex_id })?;

    let arc_q1q2 = build_great_circle_arc(center, r, q1, q2, vertex_id)?;
    let arc_q2q3 = build_great_circle_arc(center, r, q2, q3, vertex_id)?;
    let arc_q3q1 = build_great_circle_arc(center, r, q3, q1, vertex_id)?;

    Ok(SphericalCornerResult {
        surface: FaceSurface::Sphere(sphere),
        boundary_curves: vec![arc_q1q2, arc_q2q3, arc_q3q1],
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use super::*;
    use remus_topology::topology::Topology;
    use remus_topology::vertex::Vertex;

    /// Helper: create a dummy `VertexId` via a real topology arena.
    fn make_vertex_id() -> (Topology, VertexId) {
        let mut topo = Topology::new();
        let vid = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        (topo, vid)
    }

    /// Unit cube corner at origin with fillet radius 0.2.
    /// Face normals point inward (+X, +Y, +Z), vertex is convex.
    fn unit_cube_corner_data(vertex_id: VertexId) -> VertexContactData {
        let r = 0.2;
        let origin = Point3::new(0.0, 0.0, 0.0);
        let nx = Vec3::new(1.0, 0.0, 0.0);
        let ny = Vec3::new(0.0, 1.0, 0.0);
        let nz = Vec3::new(0.0, 0.0, 1.0);

        // Sphere center = origin + r * normalize(nx+ny+nz)
        let normal_sum = nx + ny + nz;
        let normal_dir = normal_sum * (1.0 / normal_sum.length());
        let center = origin + normal_dir * r;

        // Contact points: where the sphere touches each face plane.
        // Contact on face with normal n_i is C - r * n_i
        // (point on sphere closest to the face plane through the vertex).
        let q1 = center - nx * r; // on YZ face
        let q2 = center - ny * r; // on XZ face
        let q3 = center - nz * r; // on XY face

        VertexContactData {
            vertex_pos: origin,
            contact_points: vec![q1, q2, q3],
            face_normals: vec![nx, ny, nz],
            radius: r,
            is_convex: true,
            vertex_id,
            ball: Option::None,
        }
    }

    #[test]
    fn test_sphere_center_convex() {
        let (_topo, vid) = make_vertex_id();
        let data = unit_cube_corner_data(vid);
        let (center, sphere_r) = compute_sphere_center(&data).expect("should compute center");

        let r = data.radius;
        // Center = vertex + R * sum(normals) = (0,0,0) + 0.2*(1,1,1) = (0.2,0.2,0.2)
        let expected = data.vertex_pos + Vec3::new(1.0, 1.0, 1.0) * r;

        let err = (center - expected).length();
        assert!(err < 1e-10, "center offset: {err}");

        // All contact points at distance sphere_r from center.
        for (i, cp) in data.contact_points.iter().enumerate() {
            let dist = (*cp - center).length();
            let diff = (dist - sphere_r).abs();
            assert!(diff < 1e-10, "contact point {i} distance error: {diff}");
        }
    }

    #[test]
    fn test_spherical_triangle_surface_is_exact_sphere() {
        let (_topo, vid) = make_vertex_id();
        let data = unit_cube_corner_data(vid);
        let (center, r) = compute_sphere_center(&data).expect("should compute center");

        let result = build_spherical_corner(&data).expect("should build corner");

        // The patch is the analytic corner ball itself — no approximation.
        // (The former degree-(2,2) rational patch needed a 15%-of-R sampling
        // allowance here; that tolerance codified the corner-blob defect.)
        let FaceSurface::Sphere(sphere) = &result.surface else {
            panic!("expected analytic Sphere surface");
        };
        assert!((sphere.radius() - r).abs() < 1e-12, "radius mismatch");
        assert!(
            (sphere.center() - center).length() < 1e-12,
            "center mismatch"
        );

        // The chart's singularities must stay off the patch: every contact
        // direction keeps a healthy margin from both poles and the u-seam.
        for (i, cp) in data.contact_points.iter().enumerate() {
            let (u, v) = sphere.project_point(*cp);
            assert!(
                v.abs() < std::f64::consts::FRAC_PI_2 - 0.3,
                "contact {i} too close to a pole (v={v})"
            );
            assert!(
                u > 0.3 && u < std::f64::consts::TAU - 0.3,
                "contact {i} too close to the u-seam (u={u})"
            );
        }
    }

    #[test]
    fn test_explicit_ball_overrides_heuristic() {
        // With an exact ball supplied, the solver must use it verbatim
        // instead of re-deriving a (wrong) ball from the contacts. Feed the
        // unset-back contacts of a unit-cube corner — the configuration that
        // historically inflated the ball to √2·R — together with the true
        // tangent ball.
        let (_topo, vid) = make_vertex_id();
        let r = 0.2;
        let center = Point3::new(r, r, r);
        let data = VertexContactData {
            vertex_pos: Point3::new(0.0, 0.0, 0.0),
            contact_points: vec![
                Point3::new(0.0, r, r),
                Point3::new(r, 0.0, r),
                Point3::new(r, r, 0.0),
            ],
            face_normals: vec![
                Vec3::new(-1.0, 0.0, 0.0),
                Vec3::new(0.0, -1.0, 0.0),
                Vec3::new(0.0, 0.0, -1.0),
            ],
            radius: r,
            is_convex: true,
            vertex_id: vid,
            ball: Some((center, r)),
        };

        let result = build_spherical_corner(&data).expect("should build corner");
        let FaceSurface::Sphere(sphere) = &result.surface else {
            panic!("expected analytic Sphere surface");
        };
        assert!(
            (sphere.radius() - r).abs() < 1e-12,
            "ball radius must be the fillet radius, not √2·R: got {}",
            sphere.radius()
        );
        assert!((sphere.center() - center).length() < 1e-12);
    }

    #[test]
    fn test_boundary_arc_is_circular() {
        let (_topo, vid) = make_vertex_id();
        let data = unit_cube_corner_data(vid);
        let (center, r) = compute_sphere_center(&data).expect("should compute center");

        let result = build_spherical_corner(&data).expect("should build corner");

        // Each boundary arc should have all sampled points at distance R from center.
        for (arc_idx, arc) in result.boundary_curves.iter().enumerate() {
            let n_samples = 20;
            for i in 0..=n_samples {
                let t = i as f64 / n_samples as f64;
                let pt = arc.evaluate(t);
                let dist = (pt - center).length();
                let err = (dist - r).abs();
                assert!(
                    err < 1e-10,
                    "arc {arc_idx} at t={t}: dist error {err} (dist={dist}, r={r})"
                );
            }
        }
    }
}
