//! Affine transforms applied to topological shapes.

use std::collections::HashSet;

use brepkit_math::mat::Mat4;
use brepkit_math::nurbs::curve::NurbsCurve;
use brepkit_math::nurbs::surface::NurbsSurface;
use brepkit_math::vec::Vec3;
use brepkit_topology::Topology;
use brepkit_topology::edge::{EdgeCurve, EdgeId};
use brepkit_topology::face::{FaceId, FaceSurface};
use brepkit_topology::solid::SolidId;
use brepkit_topology::vertex::VertexId;
use brepkit_topology::wire::WireId;

/// Dimensionless floor below which a transform is treated as collapsing the
/// model rather than moving it.
///
/// The quantity compared against it is the linear part's **Hadamard ratio**,
/// `|det| / (‖c₀‖·‖c₁‖·‖c₂‖)` over the three columns — equivalently the
/// determinant of the same matrix with every column normalized to unit
/// length. It is the volume of the unit cube's image divided by the volume
/// those columns would span if they were mutually orthogonal, so it is `1`
/// for any similarity (rotation, uniform scale, reflection), `0` for a matrix
/// that flattens space onto a plane, line or point, and — being a quotient of
/// two volumes — dimensionless. It is therefore identical for a model in
/// metres and the same model in nanometres, which is the whole point: a
/// transform is degenerate because of its *shape*, never because of its size.
///
/// `1e-12` sits far below any transform a user means (every non-degenerate
/// matrix in the suite below, uniform or not, measures between 0.14 and 1.0)
/// and far above the `f64::EPSILON`-relative algebraic test inside
/// `Mat4::inverse`, which remains the backstop. This guard exists to name the
/// failure before the inverse-transpose normal update starts amplifying
/// round-off.
const DEGENERATE_SHAPE_RATIO: f64 = 1e-12;

/// Half-width of the ulp-scale band around `[0, 0, 0, 1]` inside which a
/// bottom row still counts as affine.
///
/// This is deliberately a tolerance and not an exact comparison. `Mat4::inverse`
/// is an adjugate inversion, so inverting a perfectly rigid frame legitimately
/// returns a bottom row like `[0, 0, -0.0, 1.0000000000000002]`: the `w` entry
/// is a sum of 2×2 minors divided by the determinant, and that division does
/// not have to land on exactly `1.0`. Rejecting those rows would refuse every
/// caller that feeds a `Mat4::inverse()` result back into a transform.
///
/// A genuinely projective row is nowhere near this band — perspective entries
/// are on the order of the reciprocal of the model's size, some twelve or more
/// orders of magnitude above `8·f64::EPSILON`.
const AFFINE_ROW_WOBBLE: f64 = 8.0 * f64::EPSILON;

/// Reject a transform that collapses the model; accept every one that does not.
///
/// Validates the affine bottom row before testing the 3×3 linear part.
/// `Mat4::mul_point` ignores the bottom row, so accepting a projective matrix
/// here would apply a different transform from the one the caller supplied.
///
/// # Errors
///
/// Returns [`crate::OperationsError::InvalidInput`] when any entry is
/// non-finite, the matrix is not affine, a linear column is zero, or the
/// Hadamard ratio is at or below [`DEGENERATE_SHAPE_RATIO`].
pub(crate) fn reject_degenerate_transform(matrix: &Mat4) -> Result<(), crate::OperationsError> {
    let degenerate = |reason: &str| crate::OperationsError::InvalidInput {
        reason: format!("transform matrix is degenerate ({reason})"),
    };

    let m = &matrix.0;
    // Checked over the whole matrix, and before the band below: a NaN fails
    // every `>` comparison, so an unchecked NaN would slip through as affine.
    if m.iter().flatten().any(|value| !value.is_finite()) {
        return Err(degenerate("an entry is not finite"));
    }
    if m[3][0].abs() > AFFINE_ROW_WOBBLE
        || m[3][1].abs() > AFFINE_ROW_WOBBLE
        || m[3][2].abs() > AFFINE_ROW_WOBBLE
        || (m[3][3] - 1.0).abs() > AFFINE_ROW_WOBBLE
    {
        return Err(degenerate("the bottom row is not affine"));
    }
    // Normalize each column before taking the determinant, rather than
    // dividing the determinant by the product of the norms afterwards: the
    // product of three norms can overflow or underflow to 0/∞ for extreme
    // (but perfectly valid) matrices, and the quotient would then be NaN.
    let mut unit = [Vec3::new(0.0, 0.0, 0.0); 3];
    for (j, slot) in unit.iter_mut().enumerate() {
        let col = Vec3::new(m[0][j], m[1][j], m[2][j]);
        let Ok(n) = col.normalize() else {
            return Err(degenerate("a linear column is zero or non-finite"));
        };
        *slot = n;
    }

    // Scalar triple product of the unit columns = the Hadamard ratio, in
    // [0, 1] by construction. Reflections give a negative triple product and
    // a ratio of 1 — they are proper transforms and must keep passing.
    let ratio = unit[0].dot(unit[1].cross(unit[2])).abs();
    if ratio <= DEGENERATE_SHAPE_RATIO {
        return Err(degenerate(
            "it collapses the model onto a plane, line or point",
        ));
    }
    Ok(())
}

/// Apply an affine transform to a solid, modifying vertex positions and
/// face surface geometry in place.
///
/// The transform matrix must be non-degenerate — see
/// `reject_degenerate_transform`, which tests the matrix's *shape* and so
/// accepts a uniform scale of any size, in either direction.
/// All unique vertices reachable from the solid's shells are transformed,
/// NURBS edge curves and face surfaces have their control points updated,
/// and all planar face normals are updated using the inverse transpose.
///
/// # Errors
///
/// Returns an error if the matrix is degenerate or a referenced entity is missing.
#[allow(clippy::too_many_lines)]
pub fn transform_solid(
    topo: &mut Topology,
    solid: SolidId,
    matrix: &Mat4,
) -> Result<(), crate::OperationsError> {
    reject_degenerate_transform(matrix)?;
    // Validate every part of the matrix before changing live topology.
    let normal_matrix = matrix.inverse()?.transpose();

    // Collect all unique vertex IDs, edge IDs, and face IDs in a read phase.
    let (vertex_ids, edge_ids, face_ids) = collect_solid_entities(topo, solid)?;

    // Mutate phase 1: transform each vertex.
    for vid in vertex_ids {
        let vertex = topo.vertex_mut(vid)?;
        let new_point = matrix.mul_point(vertex.point());
        vertex.set_point(new_point);
    }

    // Mutate phase 2: transform edge curves (NURBS, Circle, Ellipse).
    transform_edges(topo, &edge_ids, matrix)?;

    // Mutate phase 3: transform face surface geometry.
    // For plane normals, use the inverse transpose: n' = (M⁻¹)ᵀ · n
    for fid in face_ids {
        let face = topo.face(fid)?;
        match face.surface() {
            FaceSurface::Plane { normal, .. } => {
                let n = *normal;
                // Transform the normal via the inverse transpose (treating it as
                // a direction, so we use mul_point on a point at (nx, ny, nz)
                // and subtract the translation component).
                let transformed =
                    normal_matrix.mul_point(brepkit_math::vec::Point3::new(n.x(), n.y(), n.z()));
                // Extract direction only (ignore any translation component from
                // the inverse transpose by subtracting the origin transform).
                let origin = normal_matrix.mul_point(brepkit_math::vec::Point3::new(0.0, 0.0, 0.0));
                let raw = Vec3::new(
                    transformed.x() - origin.x(),
                    transformed.y() - origin.y(),
                    transformed.z() - origin.z(),
                );
                let new_normal = raw.normalize()?;

                // Recompute d from a transformed vertex on this face. We use
                // the first vertex of the outer wire.
                let wire = topo.wire(face.outer_wire())?;
                let first_oe = &wire.edges()[0];
                let edge = topo.edge(first_oe.edge())?;
                let ref_vid = if first_oe.is_forward() {
                    edge.start()
                } else {
                    edge.end()
                };
                let ref_point = topo.vertex(ref_vid)?.point();
                let new_d = new_normal.dot(Vec3::new(ref_point.x(), ref_point.y(), ref_point.z()));

                let face_mut = topo.face_mut(fid)?;
                face_mut.set_surface(FaceSurface::Plane {
                    normal: new_normal,
                    d: new_d,
                });
            }
            FaceSurface::Nurbs(s) => {
                let new_control_points: Vec<Vec<_>> = s
                    .control_points()
                    .iter()
                    .map(|row| row.iter().map(|pt| matrix.mul_point(*pt)).collect())
                    .collect();
                let new_surface = NurbsSurface::new(
                    s.degree_u(),
                    s.degree_v(),
                    s.knots_u().to_vec(),
                    s.knots_v().to_vec(),
                    new_control_points,
                    s.weights().to_vec(),
                );
                topo.face_mut(fid)?
                    .set_surface(FaceSurface::Nurbs(new_surface?));
            }
            FaceSurface::Cylinder(cyl) => {
                let new_origin = matrix.mul_point(cyl.origin());
                let new_axis = transform_direction(matrix, cyl.axis())?;
                // Scale radius: measure how the matrix scales a direction perpendicular to axis
                let new_radius = scaled_radius(matrix, cyl.axis(), cyl.radius());
                let new_cyl = brepkit_math::surfaces::CylindricalSurface::new(
                    new_origin, new_axis, new_radius,
                )?;
                topo.face_mut(fid)?
                    .set_surface(FaceSurface::Cylinder(new_cyl));
            }
            FaceSurface::Cone(cone) => {
                if is_uniform_scale(matrix) {
                    let new_apex = matrix.mul_point(cone.apex());
                    let new_axis = transform_direction(matrix, cone.axis())?;
                    let new_cone = brepkit_math::surfaces::ConicalSurface::new(
                        new_apex,
                        new_axis,
                        cone.half_angle(),
                    )?;
                    topo.face_mut(fid)?.set_surface(FaceSurface::Cone(new_cone));
                } else {
                    let v_range = analytic_face_v_range(topo, fid, |pt| cone.project_point(pt).1)?;
                    let cone_clone = cone.clone();
                    // Use heal's exact rational cone converter (geometry's
                    // delegates to math's sampled approximation; heal's is
                    // geometrically exact). v_range is the cone-generator
                    // distance from apex.
                    let nurbs = brepkit_heal::construct::convert_surface::cone_to_nurbs(
                        &cone_clone,
                        v_range,
                    )
                    .map_err(|e| crate::OperationsError::InvalidInput {
                        reason: format!("cone_to_nurbs failed: {e}"),
                    })?;
                    let transformed = transform_nurbs_surface(&nurbs, matrix)?;
                    topo.face_mut(fid)?
                        .set_surface(FaceSurface::Nurbs(transformed));
                }
            }
            FaceSurface::Sphere(sph) => {
                if is_uniform_scale(matrix) {
                    let new_center = matrix.mul_point(sph.center());
                    // Extract uniform scale factor from column magnitudes
                    let m = &matrix.0;
                    let sx = (m[0][0] * m[0][0] + m[1][0] * m[1][0] + m[2][0] * m[2][0]).sqrt();
                    let new_sph = brepkit_math::surfaces::SphericalSurface::new(
                        new_center,
                        sph.radius() * sx,
                    )?;
                    topo.face_mut(fid)?
                        .set_surface(FaceSurface::Sphere(new_sph));
                } else {
                    // Non-uniform scale: sample the face's v-range of the
                    // sphere and refit as NURBS.
                    let (v_min, v_max) = sphere_face_v_range(topo, fid, sph)?;
                    let sph_clone = sph.clone();
                    let nurbs = sphere_to_transformed_nurbs(&sph_clone, matrix, v_min, v_max)?;
                    topo.face_mut(fid)?.set_surface(FaceSurface::Nurbs(nurbs));
                }
            }
            FaceSurface::Torus(tor) => {
                if is_uniform_scale(matrix) {
                    let new_center = matrix.mul_point(tor.center());
                    let m = &matrix.0;
                    let sx = (m[0][0] * m[0][0] + m[1][0] * m[1][0] + m[2][0] * m[2][0]).sqrt();
                    let new_tor = brepkit_math::surfaces::ToroidalSurface::new(
                        new_center,
                        tor.major_radius() * sx,
                        tor.minor_radius() * sx,
                    )?;
                    topo.face_mut(fid)?.set_surface(FaceSurface::Torus(new_tor));
                } else {
                    let tor_clone = tor.clone();
                    // Use heal's exact rational torus converter (geometry's
                    // delegates to math's sampled approximation; heal's is
                    // geometrically exact 9×9 tensor product).
                    let nurbs =
                        brepkit_heal::construct::convert_surface::torus_to_nurbs(&tor_clone)
                            .map_err(|e| crate::OperationsError::InvalidInput {
                                reason: format!("torus_to_nurbs failed: {e}"),
                            })?;
                    let transformed = transform_nurbs_surface(&nurbs, matrix)?;
                    topo.face_mut(fid)?
                        .set_surface(FaceSurface::Nurbs(transformed));
                }
            }
        }
    }

    Ok(())
}

/// Determine the v-range (latitude) of a sphere face from its boundary.
///
/// Projects boundary vertices onto the sphere to find their latitudes,
/// then uses the sign of the average vertex Z offset from center to
/// determine which hemisphere the face covers.
fn sphere_face_v_range(
    topo: &Topology,
    face_id: FaceId,
    sph: &brepkit_math::surfaces::SphericalSurface,
) -> Result<(f64, f64), crate::OperationsError> {
    use std::f64::consts::FRAC_PI_2;

    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;
    let mut v_vals = Vec::new();

    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let pt = topo.vertex(edge.start())?.point();
        let (_u, v) = sph.project_point(pt);
        v_vals.push(v);
    }

    if v_vals.is_empty() {
        // Full sphere with no boundary → full range
        return Ok((-FRAC_PI_2, FRAC_PI_2));
    }

    // All boundary vertices should be at roughly the same v (equator).
    // Determine hemisphere by checking whether face is above or below boundary.
    let boundary_v = v_vals.iter().copied().sum::<f64>() / v_vals.len() as f64;

    // Check which side: sample a face interior point. A simpler heuristic:
    // if any inner wire exists, check it. Otherwise, examine the face's
    // Newell normal direction relative to the sphere center.
    //
    // For brepkit's make_sphere: south hemisphere has normals pointing
    // away from center with v ∈ [-π/2, boundary_v], north hemisphere
    // v ∈ [boundary_v, π/2].
    //
    // Use a heuristic: compute the average Z of boundary relative to center
    // and compare with the face's position hints.
    let center = sph.center();
    let avg_boundary_z: f64 = {
        let mut sum = 0.0;
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            let pt = topo.vertex(edge.start())?.point();
            sum += pt.z() - center.z();
        }
        sum / wire.edges().len() as f64
    };

    // If the boundary is near the equator (avg_z ≈ 0), we need another way.
    // Try to detect hemisphere by checking if the face has a pole vertex
    // (a degenerate edge with a pole at v = ±π/2).
    // Simpler approach: this is called before the transform, and make_sphere
    // creates two faces. Just check if boundary_v ≈ 0 and pick hemispheres.
    if boundary_v.abs() < 0.1 {
        // Near equator: use face ordering. Check if this face has vertices
        // near the north pole (z > center.z) or south pole (z < center.z).
        // If avg_boundary_z is near 0, look for a degenerate pole vertex.
        let mut has_pole_north = false;
        let mut has_pole_south = false;
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            if edge.start() == edge.end() {
                let pt = topo.vertex(edge.start())?.point();
                let dz = pt.z() - center.z();
                if dz > 0.0 {
                    has_pole_north = true;
                } else {
                    has_pole_south = true;
                }
            }
        }
        if has_pole_north {
            return Ok((boundary_v, FRAC_PI_2));
        }
        if has_pole_south {
            return Ok((-FRAC_PI_2, boundary_v));
        }
        // Default: use the winding direction. If first edge goes "forward" in
        // parameter space, it's the north hemisphere.
        // Fallback: just check avg Z of all edge midpoints would require
        // curve evaluation. Use a simpler heuristic based on face ordering.
        // The first face in make_sphere is south, second is north.
        // This is fragile, but works for this specific case.
        if avg_boundary_z >= 0.0 {
            return Ok((boundary_v, FRAC_PI_2));
        }
        return Ok((-FRAC_PI_2, boundary_v));
    }

    if boundary_v > 0.0 {
        Ok((boundary_v, FRAC_PI_2))
    } else {
        Ok((-FRAC_PI_2, boundary_v))
    }
}

/// Check whether a transform matrix has uniform scaling (all axis scale
/// factors are approximately equal). Non-uniform scaling distorts spheres
/// into ellipsoids, so analytic representations must be converted to NURBS.
/// Compute the scaled radius of a circle perpendicular to `axis` after transform.
fn scaled_radius(matrix: &Mat4, axis: Vec3, radius: f64) -> f64 {
    // Pick a direction perpendicular to the axis
    let perp = if axis.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
            .cross(axis)
            .normalize()
            .unwrap_or(Vec3::new(1.0, 0.0, 0.0))
    } else {
        Vec3::new(0.0, 1.0, 0.0)
            .cross(axis)
            .normalize()
            .unwrap_or(Vec3::new(0.0, 1.0, 0.0))
    };
    // Transform the perpendicular direction and measure its length
    let origin = brepkit_math::vec::Point3::new(0.0, 0.0, 0.0);
    let end =
        brepkit_math::vec::Point3::new(perp.x() * radius, perp.y() * radius, perp.z() * radius);
    let t_origin = matrix.mul_point(origin);
    let t_end = matrix.mul_point(end);
    let diff = t_end - t_origin;
    diff.length()
}

/// Transform a single face's surface geometry.
///
/// The `normal_matrix` should be `matrix.inverse()?.transpose()`.
#[allow(clippy::too_many_lines)]
fn transform_face_surface(
    topo: &mut Topology,
    fid: FaceId,
    matrix: &Mat4,
    normal_matrix: &Mat4,
) -> Result<(), crate::OperationsError> {
    let face = topo.face(fid)?;
    match face.surface() {
        FaceSurface::Plane { normal, .. } => {
            let n = *normal;
            let transformed =
                normal_matrix.mul_point(brepkit_math::vec::Point3::new(n.x(), n.y(), n.z()));
            let origin = normal_matrix.mul_point(brepkit_math::vec::Point3::new(0.0, 0.0, 0.0));
            let raw = Vec3::new(
                transformed.x() - origin.x(),
                transformed.y() - origin.y(),
                transformed.z() - origin.z(),
            );
            let new_normal = raw.normalize()?;
            let wire = topo.wire(face.outer_wire())?;
            let first_oe =
                wire.edges()
                    .first()
                    .ok_or_else(|| crate::OperationsError::InvalidInput {
                        reason: "face has empty outer wire".into(),
                    })?;
            let edge = topo.edge(first_oe.edge())?;
            let ref_vid = if first_oe.is_forward() {
                edge.start()
            } else {
                edge.end()
            };
            let ref_point = topo.vertex(ref_vid)?.point();
            let new_d = new_normal.dot(Vec3::new(ref_point.x(), ref_point.y(), ref_point.z()));
            topo.face_mut(fid)?.set_surface(FaceSurface::Plane {
                normal: new_normal,
                d: new_d,
            });
        }
        FaceSurface::Nurbs(s) => {
            let new_control_points: Vec<Vec<_>> = s
                .control_points()
                .iter()
                .map(|row| row.iter().map(|pt| matrix.mul_point(*pt)).collect())
                .collect();
            let new_surface = NurbsSurface::new(
                s.degree_u(),
                s.degree_v(),
                s.knots_u().to_vec(),
                s.knots_v().to_vec(),
                new_control_points,
                s.weights().to_vec(),
            );
            topo.face_mut(fid)?
                .set_surface(FaceSurface::Nurbs(new_surface?));
        }
        FaceSurface::Cylinder(cyl) => {
            let new_origin = matrix.mul_point(cyl.origin());
            let new_axis = transform_direction(matrix, cyl.axis())?;
            let new_radius = scaled_radius(matrix, cyl.axis(), cyl.radius());
            let new_cyl =
                brepkit_math::surfaces::CylindricalSurface::new(new_origin, new_axis, new_radius)?;
            topo.face_mut(fid)?
                .set_surface(FaceSurface::Cylinder(new_cyl));
        }
        FaceSurface::Cone(cone) => {
            if is_uniform_scale(matrix) {
                let new_apex = matrix.mul_point(cone.apex());
                let new_axis = transform_direction(matrix, cone.axis())?;
                let new_cone = brepkit_math::surfaces::ConicalSurface::new(
                    new_apex,
                    new_axis,
                    cone.half_angle(),
                )?;
                topo.face_mut(fid)?.set_surface(FaceSurface::Cone(new_cone));
            } else {
                let v_range = analytic_face_v_range(topo, fid, |pt| cone.project_point(pt).1)?;
                let cone_clone = cone.clone();
                let nurbs =
                    brepkit_heal::construct::convert_surface::cone_to_nurbs(&cone_clone, v_range)
                        .map_err(|e| crate::OperationsError::InvalidInput {
                        reason: format!("cone_to_nurbs failed: {e}"),
                    })?;
                let transformed = transform_nurbs_surface(&nurbs, matrix)?;
                topo.face_mut(fid)?
                    .set_surface(FaceSurface::Nurbs(transformed));
            }
        }
        FaceSurface::Sphere(sph) => {
            if is_uniform_scale(matrix) {
                let new_center = matrix.mul_point(sph.center());
                let m = &matrix.0;
                let sx = (m[0][0] * m[0][0] + m[1][0] * m[1][0] + m[2][0] * m[2][0]).sqrt();
                let new_sph =
                    brepkit_math::surfaces::SphericalSurface::new(new_center, sph.radius() * sx)?;
                topo.face_mut(fid)?
                    .set_surface(FaceSurface::Sphere(new_sph));
            } else {
                let (v_min, v_max) = sphere_face_v_range(topo, fid, sph)?;
                let sph_clone = sph.clone();
                let nurbs = sphere_to_transformed_nurbs(&sph_clone, matrix, v_min, v_max)?;
                topo.face_mut(fid)?.set_surface(FaceSurface::Nurbs(nurbs));
            }
        }
        FaceSurface::Torus(tor) => {
            if is_uniform_scale(matrix) {
                let new_center = matrix.mul_point(tor.center());
                let m = &matrix.0;
                let sx = (m[0][0] * m[0][0] + m[1][0] * m[1][0] + m[2][0] * m[2][0]).sqrt();
                let new_tor = brepkit_math::surfaces::ToroidalSurface::new(
                    new_center,
                    tor.major_radius() * sx,
                    tor.minor_radius() * sx,
                )?;
                topo.face_mut(fid)?.set_surface(FaceSurface::Torus(new_tor));
            } else {
                let tor_clone = tor.clone();
                let nurbs = brepkit_heal::construct::convert_surface::torus_to_nurbs(&tor_clone)
                    .map_err(|e| crate::OperationsError::InvalidInput {
                        reason: format!("torus_to_nurbs failed: {e}"),
                    })?;
                let transformed = transform_nurbs_surface(&nurbs, matrix)?;
                topo.face_mut(fid)?
                    .set_surface(FaceSurface::Nurbs(transformed));
            }
        }
    }
    Ok(())
}

/// Compute the v-parameter range for an analytic surface face.
///
/// Projects boundary vertices using `project_v` and returns (v_min, v_max).
fn analytic_face_v_range(
    topo: &Topology,
    face_id: FaceId,
    project_v: impl Fn(brepkit_math::vec::Point3) -> f64,
) -> Result<(f64, f64), crate::OperationsError> {
    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let pt = topo.vertex(edge.start())?.point();
        let v = project_v(pt);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }
    if v_min >= v_max {
        v_min = 0.0;
        v_max = 1.0;
    }
    Ok((v_min, v_max))
}

/// Transform a NURBS surface's control points by a matrix.
fn transform_nurbs_surface(
    surface: &NurbsSurface,
    matrix: &Mat4,
) -> Result<NurbsSurface, crate::OperationsError> {
    let new_cps: Vec<Vec<_>> = surface
        .control_points()
        .iter()
        .map(|row| row.iter().map(|pt| matrix.mul_point(*pt)).collect())
        .collect();
    Ok(NurbsSurface::new(
        surface.degree_u(),
        surface.degree_v(),
        surface.knots_u().to_vec(),
        surface.knots_v().to_vec(),
        new_cps,
        surface.weights().to_vec(),
    )?)
}

fn is_uniform_scale(matrix: &Mat4) -> bool {
    let m = &matrix.0;
    // Column vector magnitudes of the upper-left 3×3
    let sx = (m[0][0] * m[0][0] + m[1][0] * m[1][0] + m[2][0] * m[2][0]).sqrt();
    let sy = (m[0][1] * m[0][1] + m[1][1] * m[1][1] + m[2][1] * m[2][1]).sqrt();
    let sz = (m[0][2] * m[0][2] + m[1][2] * m[1][2] + m[2][2] * m[2][2]).sqrt();
    let avg = (sx + sy + sz) / 3.0;
    let rel = 0.01; // 1% tolerance
    (sx - avg).abs() < avg * rel && (sy - avg).abs() < avg * rel && (sz - avg).abs() < avg * rel
}

/// Sample a spherical surface over a given v-range, transform the points
/// with a matrix, and refit as a NURBS surface. This preserves the correct
/// geometry when a non-uniform scale is applied (sphere → ellipsoid).
#[allow(clippy::cast_precision_loss)]
fn sphere_to_transformed_nurbs(
    sph: &brepkit_math::surfaces::SphericalSurface,
    matrix: &Mat4,
    v_min: f64,
    v_max: f64,
) -> Result<NurbsSurface, crate::OperationsError> {
    use std::f64::consts::TAU;

    let n_u = 33; // Longitude samples (0 to 2π)
    let n_v = 17; // Latitude samples

    let mut rows: Vec<Vec<brepkit_math::vec::Point3>> = Vec::with_capacity(n_v);
    for iv in 0..n_v {
        let v = v_min + (v_max - v_min) * (iv as f64) / ((n_v - 1) as f64);
        let mut row = Vec::with_capacity(n_u);
        for iu in 0..n_u {
            let u = TAU * (iu as f64) / ((n_u - 1) as f64);
            let pt = sph.evaluate(u, v);
            row.push(matrix.mul_point(pt));
        }
        rows.push(row);
    }

    let nurbs = brepkit_math::nurbs::surface_fitting::interpolate_surface(&rows, 3, 3)?;
    Ok(nurbs)
}

/// Transforms a direction vector by applying the matrix and subtracting the
/// translation component, then normalizing.
fn transform_direction(matrix: &Mat4, dir: Vec3) -> Result<Vec3, crate::OperationsError> {
    let origin = matrix.mul_point(brepkit_math::vec::Point3::new(0.0, 0.0, 0.0));
    let tip = matrix.mul_point(brepkit_math::vec::Point3::new(dir.x(), dir.y(), dir.z()));
    let raw = Vec3::new(
        tip.x() - origin.x(),
        tip.y() - origin.y(),
        tip.z() - origin.z(),
    );
    Ok(raw.normalize()?)
}

/// Transform a set of edge curves in place.
///
/// Line edges need no update — their geometry is defined by vertices.
#[allow(clippy::too_many_lines)]
fn transform_edges(
    topo: &mut Topology,
    edge_ids: &HashSet<EdgeId>,
    matrix: &Mat4,
) -> Result<(), crate::OperationsError> {
    let origin = matrix.mul_point(brepkit_math::vec::Point3::new(0.0, 0.0, 0.0));
    let transform_dir = |d: Vec3| -> Vec3 {
        matrix.mul_point(brepkit_math::vec::Point3::new(d.x(), d.y(), d.z())) - origin
    };
    for &eid in edge_ids {
        let edge = topo.edge(eid)?;
        let new_curve = match edge.curve() {
            EdgeCurve::Line => None,
            // Exact under a similarity, typed refusal otherwise — see
            // `transform_open_conic`.
            c @ (EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_)) => {
                Some(transform_open_conic(c, matrix)?)
            }
            EdgeCurve::NurbsCurve(c) => {
                let new_control_points: Vec<_> = c
                    .control_points()
                    .iter()
                    .map(|pt| matrix.mul_point(*pt))
                    .collect();
                Some(EdgeCurve::NurbsCurve(NurbsCurve::new(
                    c.degree(),
                    c.knots().to_vec(),
                    new_control_points,
                    c.weights().to_vec(),
                )?))
            }
            EdgeCurve::Circle(c) => {
                let new_center = matrix.mul_point(c.center());
                let new_u = transform_dir(c.u_axis());
                let new_v = transform_dir(c.v_axis());
                let su = new_u.length();
                let sv = new_v.length();
                let new_normal = new_u.cross(new_v).normalize()?;
                if (su - sv).abs() < 1e-12 * su.max(sv).max(1.0) {
                    Some(EdgeCurve::Circle(
                        brepkit_math::curves::Circle3D::with_axes(
                            new_center,
                            new_normal,
                            c.radius() * su,
                            new_u.normalize()?,
                            new_v.normalize()?,
                        )?,
                    ))
                } else {
                    let (semi_major, semi_minor, u_dir, v_dir) = if su >= sv {
                        (
                            c.radius() * su,
                            c.radius() * sv,
                            new_u.normalize()?,
                            new_v.normalize()?,
                        )
                    } else {
                        (
                            c.radius() * sv,
                            c.radius() * su,
                            new_v.normalize()?,
                            new_u.normalize()?,
                        )
                    };
                    Some(EdgeCurve::Ellipse(
                        brepkit_math::curves::Ellipse3D::with_axes(
                            new_center, new_normal, semi_major, semi_minor, u_dir, v_dir,
                        )?,
                    ))
                }
            }
            EdgeCurve::Ellipse(e) => {
                let new_center = matrix.mul_point(e.center());
                let new_u = transform_dir(e.u_axis());
                let new_v = transform_dir(e.v_axis());
                let new_normal = new_u.cross(new_v).normalize()?;
                Some(EdgeCurve::Ellipse(
                    brepkit_math::curves::Ellipse3D::with_axes(
                        new_center,
                        new_normal,
                        e.semi_major() * new_u.length(),
                        e.semi_minor() * new_v.length(),
                        new_u.normalize()?,
                        new_v.normalize()?,
                    )?,
                ))
            }
        };
        if let Some(curve) = new_curve {
            topo.edge_mut(eid)?.set_curve(curve);
        }
    }
    Ok(())
}

/// Apply an affine transform to a wire, modifying vertex positions and
/// edge curve geometry in place.
///
/// # Errors
///
/// Returns an error if the matrix is degenerate or a referenced entity is missing.
pub fn transform_wire(
    topo: &mut Topology,
    wire_id: WireId,
    matrix: &Mat4,
) -> Result<(), crate::OperationsError> {
    reject_degenerate_transform(matrix)?;

    let (vertex_ids, edge_ids) = collect_wire_entities(topo, wire_id)?;

    // Transform vertices.
    for vid in vertex_ids {
        let vertex = topo.vertex_mut(vid)?;
        let new_point = matrix.mul_point(vertex.point());
        vertex.set_point(new_point);
    }

    // Transform edge curves.
    transform_edges(topo, &edge_ids, matrix)?;

    Ok(())
}

/// Apply an affine transform to a face, modifying vertex positions, edge
/// curve geometry, and the face surface in place.
///
/// Transforms all vertices/edges in the face's outer and inner wires, then
/// updates the face surface geometry (plane normal, NURBS CPs, etc.).
///
/// # Errors
///
/// Returns an error if the matrix is degenerate or a referenced entity is missing.
#[allow(clippy::too_many_lines)]
pub fn transform_face(
    topo: &mut Topology,
    face_id: FaceId,
    matrix: &Mat4,
) -> Result<(), crate::OperationsError> {
    reject_degenerate_transform(matrix)?;
    // Validate every part of the matrix before changing live topology.
    let normal_matrix = matrix.inverse()?.transpose();

    // Collect all vertices and edges from the face's wires.
    let (vertex_ids, edge_ids) = collect_face_entities(topo, face_id)?;

    // Transform vertices.
    for vid in vertex_ids {
        let vertex = topo.vertex_mut(vid)?;
        let new_point = matrix.mul_point(vertex.point());
        vertex.set_point(new_point);
    }

    // Transform edge curves.
    transform_edges(topo, &edge_ids, matrix)?;

    // Transform face surface.
    transform_face_surface(topo, face_id, matrix, &normal_matrix)?;

    Ok(())
}

/// Traverses face → wires → edges → vertices and returns deduplicated sets.
fn collect_face_entities(
    topo: &Topology,
    face_id: FaceId,
) -> Result<(HashSet<VertexId>, HashSet<EdgeId>), crate::OperationsError> {
    let mut vertex_ids = HashSet::new();
    let mut edge_ids = HashSet::new();
    let face = topo.face(face_id)?;
    let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
        .chain(face.inner_wires().iter().copied())
        .collect();

    for wid in wire_ids {
        let wire = topo.wire(wid)?;
        for oe in wire.edges() {
            let eid = oe.edge();
            edge_ids.insert(eid);
            let edge = topo.edge(eid)?;
            vertex_ids.insert(edge.start());
            vertex_ids.insert(edge.end());
        }
    }

    Ok((vertex_ids, edge_ids))
}

/// Traverses wire → edges → vertices and returns deduplicated sets.
fn collect_wire_entities(
    topo: &Topology,
    wire_id: WireId,
) -> Result<(HashSet<VertexId>, HashSet<EdgeId>), crate::OperationsError> {
    let mut vertex_ids = HashSet::new();
    let mut edge_ids = HashSet::new();
    let wire = topo.wire(wire_id)?;
    for oe in wire.edges() {
        let eid = oe.edge();
        edge_ids.insert(eid);
        let edge = topo.edge(eid)?;
        vertex_ids.insert(edge.start());
        vertex_ids.insert(edge.end());
    }
    Ok((vertex_ids, edge_ids))
}

/// Traverses solid → shells → faces → wires → edges → vertices and
/// returns deduplicated sets of vertex IDs, edge IDs, and face IDs.
#[allow(clippy::type_complexity)]
fn collect_solid_entities(
    topo: &Topology,
    solid: SolidId,
) -> Result<(HashSet<VertexId>, HashSet<EdgeId>, HashSet<FaceId>), crate::OperationsError> {
    let mut vertex_ids = HashSet::new();
    let mut edge_ids = HashSet::new();
    let mut face_ids = HashSet::new();
    let solid_data = topo.solid(solid)?;
    let shell_ids: Vec<_> = std::iter::once(solid_data.outer_shell())
        .chain(solid_data.inner_shells().iter().copied())
        .collect();

    for shell_id in shell_ids {
        let shell = topo.shell(shell_id)?;
        let fids: Vec<_> = shell.faces().to_vec();

        for face_id in fids {
            face_ids.insert(face_id);
            let face = topo.face(face_id)?;
            let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
                .chain(face.inner_wires().iter().copied())
                .collect();

            for wire_id in wire_ids {
                let wire = topo.wire(wire_id)?;
                for oe in wire.edges() {
                    let eid = oe.edge();
                    edge_ids.insert(eid);
                    let edge = topo.edge(eid)?;
                    vertex_ids.insert(edge.start());
                    vertex_ids.insert(edge.end());
                }
            }
        }
    }

    Ok((vertex_ids, edge_ids, face_ids))
}

#[cfg(test)]
mod tests;

/// Transform an unbounded conic edge curve (`Hyperbola` / `Parabola`).
///
/// An affine map sends a parabola to a parabola and a hyperbola to a
/// hyperbola, but the image is only expressible in brepkit's canonical
/// `(centre/vertex, orthonormal axes, semi-axes/focal length)` form when
/// the map restricted to the conic's own plane is a *similarity* — a
/// uniform scale with a rotation and/or reflection. Under a shear or a
/// non-uniform scale the image is still a conic of the same type, but its
/// canonical axes are rotated by an amount this representation cannot
/// recover without a full re-fit.
///
/// So: exact when the in-plane map is a similarity, and a typed refusal
/// naming the variant otherwise. Silently keeping the untransformed
/// parameters, or approximating with the pre-image's axes, would move the
/// edge geometry away from its own vertices.
///
/// The similarity test is dimensionless — it compares axis image lengths
/// and their mutual dot product *relative to* the scale factor — so it
/// behaves identically at any model scale.
pub(crate) fn transform_open_conic(
    curve: &EdgeCurve,
    matrix: &Mat4,
) -> Result<EdgeCurve, crate::OperationsError> {
    use brepkit_math::curves::{Hyperbola3D, Parabola3D};
    use brepkit_math::vec::Point3;

    /// Relative band for "same length" and "still orthogonal". Dimensionless:
    /// both quantities are normalized by the scale factor before comparison.
    const SIMILARITY_EPS: f64 = 1e-12;

    let origin = matrix.mul_point(Point3::new(0.0, 0.0, 0.0));
    let dir = |d: Vec3| -> Vec3 { matrix.mul_point(Point3::new(d.x(), d.y(), d.z())) - origin };

    // Uniform in-plane scale factor, or `None` if the map shears or scales
    // the two in-plane axes differently.
    let in_plane_scale = |a: Vec3, b: Vec3| -> Option<f64> {
        let (ia, ib) = (dir(a), dir(b));
        let (la, lb) = (ia.length(), ib.length());
        let s = la.max(lb);
        if s <= 0.0 || (la - lb).abs() > SIMILARITY_EPS * s {
            return None;
        }
        if ia.dot(ib).abs() > SIMILARITY_EPS * la * lb {
            return None;
        }
        Some(f64::midpoint(la, lb))
    };

    let refuse = |variant: &'static str| crate::OperationsError::Unsupported {
        operation: "transform",
        reason: format!(
            "{variant} edge under a non-similarity transform: the image is a \
             {variant} but its canonical axes cannot be recovered from this \
             representation"
        ),
    };

    match curve {
        EdgeCurve::Hyperbola(h) => {
            let s = in_plane_scale(h.u_axis(), h.v_axis()).ok_or_else(|| refuse("hyperbola"))?;
            Ok(EdgeCurve::Hyperbola(Hyperbola3D::with_axes(
                matrix.mul_point(h.center()),
                dir(h.u_axis()).cross(dir(h.v_axis())),
                dir(h.u_axis()),
                h.semi_major() * s,
                h.semi_minor() * s,
            )?))
        }
        EdgeCurve::Parabola(p) => {
            let s = in_plane_scale(p.axis_dir(), p.u_axis()).ok_or_else(|| refuse("parabola"))?;
            Ok(EdgeCurve::Parabola(Parabola3D::with_axes(
                matrix.mul_point(p.vertex()),
                dir(p.axis_dir()),
                dir(p.u_axis()),
                p.focal_length() * s,
            )?))
        }
        EdgeCurve::Line
        | EdgeCurve::Circle(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::NurbsCurve(_) => Err(crate::OperationsError::Unsupported {
            operation: "transform",
            reason: format!(
                "transform_open_conic called with `{}`, which is not an \
                     unbounded conic",
                curve.type_tag()
            ),
        }),
    }
}
