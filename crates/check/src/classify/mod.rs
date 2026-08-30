//! Point-in-solid classification (ray casting + winding numbers).
//!
//! The primary entry point is [`classify_point`], which uses analytic ray
//! casting with UV boundary containment to determine whether a 3D point
//! lies inside, outside, or on the boundary of a B-Rep solid.

pub(crate) mod boundary;
pub(crate) mod ray_surface;
pub(crate) mod winding;

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use crate::CheckError;

/// Result of classifying a point relative to a solid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointClassification {
    /// The point is inside the solid.
    Inside,
    /// The point is outside the solid.
    Outside,
    /// The point is on the boundary (within tolerance).
    OnBoundary,
}

/// Options controlling the classification algorithm.
#[derive(Debug, Clone)]
pub struct ClassifyOptions {
    /// Distance threshold for "on boundary" detection.
    pub tolerance: f64,
    /// Maximum recovery attempts when ray hits face boundary.
    pub max_recovery_attempts: usize,
}

impl Default for ClassifyOptions {
    fn default() -> Self {
        Self {
            tolerance: 1e-6,
            max_recovery_attempts: 10,
        }
    }
}

/// Classify a point relative to a solid using analytic ray casting.
///
/// Uses three irrational ray directions for majority-vote consensus.
/// If the first two agree, the third is skipped. If all three disagree
/// (very rare — indicates grazing rays), perturbed recovery directions
/// are tried.
///
/// # Errors
///
/// Returns an error if the solid or its faces contain invalid topology references.
#[allow(clippy::cast_precision_loss, clippy::too_many_lines)]
pub fn classify_point(
    topo: &Topology,
    solid: SolidId,
    point: Point3,
    options: &ClassifyOptions,
) -> Result<PointClassification, CheckError> {
    let faces = remus_topology::explorer::solid_faces(topo, solid)?;

    if is_on_boundary(topo, &faces, point, options.tolerance)? {
        return Ok(PointClassification::OnBoundary);
    }

    // Three irrational ray directions for majority-vote consensus.
    // If the first two agree, the third breaks no tie and we exit early.
    let base_dirs = [
        Vec3::new(
            0.573_576_436_351_046,
            0.740_535_693_464_567_5,
            0.350_889_803_483_932_2,
        ),
        Vec3::new(
            -0.350_889_803_483_932_2,
            0.573_576_436_351_046,
            0.740_535_693_464_567_5,
        ),
        Vec3::new(
            0.740_535_693_464_567_5,
            -0.350_889_803_483_932_2,
            0.573_576_436_351_046,
        ),
    ];

    let mut inside_votes = 0u32;
    let mut outside_votes = 0u32;

    for &dir in &base_dirs {
        let crossings = count_ray_crossings(topo, &faces, point, dir)?;
        if crossings % 2 == 1 {
            inside_votes += 1;
        } else {
            outside_votes += 1;
        }
        // Early exit: if 2 rays agree, that's the answer.
        if inside_votes >= 2 {
            return Ok(PointClassification::Inside);
        }
        if outside_votes >= 2 {
            return Ok(PointClassification::Outside);
        }
    }

    // All three disagreed (very rare). Try perturbed directions as recovery.
    for attempt in 0..options.max_recovery_attempts {
        // Generate a pseudo-random direction from attempt index using golden ratio.
        let seed = (attempt as f64 + 1.0) * 0.618_033_988_749_895;
        let theta = seed * std::f64::consts::TAU;
        let phi = (seed * std::f64::consts::E).fract() * std::f64::consts::PI;
        let dir = Vec3::new(phi.sin() * theta.cos(), phi.sin() * theta.sin(), phi.cos());

        let crossings = count_ray_crossings(topo, &faces, point, dir)?;
        if crossings % 2 == 1 {
            inside_votes += 1;
        } else {
            outside_votes += 1;
        }
        let remaining = options.max_recovery_attempts as u32 - attempt as u32;
        if inside_votes > outside_votes + remaining {
            return Ok(PointClassification::Inside);
        }
        if outside_votes > inside_votes + remaining {
            return Ok(PointClassification::Outside);
        }
    }

    // Majority vote from all attempts.
    if inside_votes > outside_votes {
        Ok(PointClassification::Inside)
    } else {
        Ok(PointClassification::Outside)
    }
}

/// Checks if a point is within `tolerance` of any face boundary.
///
/// Uses analytic point-to-surface distance for all surface types, then
/// verifies the projection falls within the face polygon.
fn is_on_boundary(
    topo: &Topology,
    faces: &[FaceId],
    point: Point3,
    tolerance: f64,
) -> Result<bool, CheckError> {
    for &fid in faces {
        let face = topo.face(fid)?;
        let dist = match face.surface() {
            FaceSurface::Plane { normal, d } => {
                let pv = Vec3::new(point.x(), point.y(), point.z());
                (normal.dot(pv) - d).abs()
            }
            FaceSurface::Cylinder(cyl) => {
                let (u, v) = cyl.project_point(point);
                let on_surface = cyl.evaluate(u, v);
                (point - on_surface).length()
            }
            FaceSurface::Cone(cone) => {
                let (u, v) = cone.project_point(point);
                let on_surface = cone.evaluate(u, v);
                (point - on_surface).length()
            }
            FaceSurface::Sphere(sph) => {
                let (u, v) = sph.project_point(point);
                let on_surface = sph.evaluate(u, v);
                (point - on_surface).length()
            }
            FaceSurface::Torus(tor) => {
                let (u, v) = tor.project_point(point);
                let on_surface = tor.evaluate(u, v);
                (point - on_surface).length()
            }
            FaceSurface::Nurbs(nurbs) => {
                match remus_math::nurbs::projection::project_point_to_surface(
                    nurbs, point, tolerance,
                ) {
                    Ok(proj) => proj.distance,
                    Err(_) => f64::INFINITY,
                }
            }
        };
        if dist < tolerance {
            let polygon = crate::util::face_polygon(topo, fid)?;
            if polygon.len() >= 3 {
                let normal = boundary::polygon_normal(&polygon);
                if crate::util::point_in_polygon_3d(&point, &polygon, &normal) {
                    // A point in one of the face's holes lies in open space,
                    // not on the trimmed face.
                    let in_hole = crate::util::face_hole_polygons(topo, fid)?
                        .iter()
                        .any(|hole| crate::util::point_in_polygon_3d(&point, hole, &normal));
                    if !in_hole {
                        return Ok(true);
                    }
                }
            } else {
                // Full-surface face (like torus with seam edges only).
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// Test whether `point` lies within `tolerance` of the solid's boundary.
///
/// Exposed so higher layers can reuse the same boundary test [`classify_point`]
/// applies, instead of duplicating the per-surface distance logic. In
/// particular `remus-operations` needs it for its tessellation-based winding
/// classifier, which cannot live in this crate (tessellation is L3).
///
/// # Errors
///
/// Returns an error if the solid or its faces contain invalid topology references.
pub fn is_point_on_boundary(
    topo: &Topology,
    solid: SolidId,
    point: Point3,
    tolerance: f64,
) -> Result<bool, CheckError> {
    let faces = remus_topology::explorer::solid_faces(topo, solid)?;
    is_on_boundary(topo, &faces, point, tolerance)
}

/// Classify a point relative to a solid using generalized winding numbers.
///
/// Sums the signed solid angles of each face's boundary-loop fan
/// triangulation, with holes subtracted.
///
/// # Accuracy
///
/// **Exact only for planar faces.** The fan is built from a face's *boundary
/// loop*, which equals the face itself only when the face is planar. For a
/// cylindrical face the loop is `bottom rim -> seam -> top rim -> seam`, whose
/// fan is not the tube, so the solid-angle sum is wrong for curved geometry —
/// interior points of a cylinder can read `Outside`. Subtracting holes (which
/// this does) does not change that.
///
/// For curved solids use [`classify_point`] (analytic ray casting, exact for
/// every supported surface type), or
/// `remus_operations::classify::classify_point_winding`, which sums solid
/// angles over a real watertight tessellation.
///
/// # Errors
///
/// Returns an error if the solid or its faces contain invalid topology references.
pub fn classify_point_winding(
    topo: &Topology,
    solid: SolidId,
    point: Point3,
    options: &ClassifyOptions,
) -> Result<PointClassification, CheckError> {
    let faces = remus_topology::explorer::solid_faces(topo, solid)?;
    if is_on_boundary(topo, &faces, point, options.tolerance)? {
        return Ok(PointClassification::OnBoundary);
    }

    let w = winding::winding_number(topo, solid, point)?;
    if w > 0.5 {
        Ok(PointClassification::Inside)
    } else {
        Ok(PointClassification::Outside)
    }
}

/// Robust classification combining winding numbers and ray casting.
///
/// Uses winding numbers first, falling back to ray casting when the
/// winding number is ambiguous (between 0.4 and 0.6).
///
/// # Accuracy
///
/// Inherits the curved-face limitation of [`classify_point_winding`]: a
/// confidently-wrong winding number (outside the 0.4..0.6 band) is returned
/// without ever consulting the ray caster. On curved solids prefer
/// [`classify_point`].
///
/// # Errors
///
/// Returns an error if the solid or its faces contain invalid topology references.
pub fn classify_point_robust(
    topo: &Topology,
    solid: SolidId,
    point: Point3,
    options: &ClassifyOptions,
) -> Result<PointClassification, CheckError> {
    let faces = remus_topology::explorer::solid_faces(topo, solid)?;
    if is_on_boundary(topo, &faces, point, options.tolerance)? {
        return Ok(PointClassification::OnBoundary);
    }

    let w = winding::winding_number(topo, solid, point)?;
    if w > 0.6 {
        return Ok(PointClassification::Inside);
    }
    if w < 0.4 {
        return Ok(PointClassification::Outside);
    }
    classify_point(topo, solid, point, options)
}

/// Count total ray crossings across all faces of a shell.
///
/// Builds a BVH over face AABBs to skip faces whose bounding box
/// the ray does not intersect.
fn count_ray_crossings(
    topo: &Topology,
    faces: &[FaceId],
    origin: Point3,
    direction: Vec3,
) -> Result<u32, CheckError> {
    use remus_math::bvh::Bvh;

    let face_aabbs: Vec<(usize, remus_math::aabb::Aabb3)> = faces
        .iter()
        .enumerate()
        .filter_map(|(i, &fid)| crate::util::face_aabb(topo, fid).ok().map(|aabb| (i, aabb)))
        .collect();
    let bvh = Bvh::build(&face_aabbs);

    // query_ray returns the primitive IDs (the `i` values), which are
    // indices into the original `faces` slice.
    let candidates = bvh.query_ray(origin, direction);

    let mut crossings = 0u32;
    for face_idx in candidates {
        crossings += boundary::count_face_ray_crossings(topo, faces[face_idx], origin, direction)?;
    }
    Ok(crossings)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::winding;
    use super::*;
    use remus_topology::face::FaceSurface;
    use remus_topology::solid::{Solid, SolidId};
    use remus_topology::test_utils::make_unit_cube_manifold;

    fn make_hollow_unit_cube(topo: &mut Topology) -> SolidId {
        let outer = make_unit_cube_manifold(topo);
        let inner = make_unit_cube_manifold(topo);
        let inner_shell = topo.solid(inner).unwrap().outer_shell();

        for vertex_id in remus_topology::explorer::solid_vertices(topo, inner).unwrap() {
            let point = topo.vertex(vertex_id).unwrap().point();
            topo.vertex_mut(vertex_id).unwrap().set_point(Point3::new(
                0.25 + 0.5 * point.x(),
                0.25 + 0.5 * point.y(),
                0.25 + 0.5 * point.z(),
            ));
        }

        let inner_faces = topo.shell(inner_shell).unwrap().faces().to_vec();
        for face_id in inner_faces {
            let (normal, wire_id) = match topo.face(face_id).unwrap().surface() {
                FaceSurface::Plane { normal, .. } => {
                    (*normal, topo.face(face_id).unwrap().outer_wire())
                }
                _ => unreachable!("test cube faces are planar"),
            };
            let wire = topo.wire(wire_id).unwrap();
            let oriented_edge = wire.edges()[0];
            let edge = topo.edge(oriented_edge.edge()).unwrap();
            let point = topo
                .vertex(oriented_edge.oriented_start(edge))
                .unwrap()
                .point();
            let d = normal.dot(Vec3::new(point.x(), point.y(), point.z()));
            let face = topo.face_mut(face_id).unwrap();
            face.set_surface(FaceSurface::Plane { normal, d });
            face.set_reversed(true);
        }

        let outer_shell = topo.solid(outer).unwrap().outer_shell();
        topo.add_solid(Solid::new(outer_shell, vec![inner_shell]))
    }

    #[test]
    fn point_inside_box() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let center = Point3::new(0.5, 0.5, 0.5);
        let opts = ClassifyOptions::default();

        let result = classify_point(&topo, solid, center, &opts).unwrap();
        assert_eq!(result, PointClassification::Inside);
    }

    #[test]
    fn hollow_solid_classifiers_subtract_inner_shell() {
        let mut topo = Topology::new();
        let solid = make_hollow_unit_cube(&mut topo);
        let options = ClassifyOptions::default();
        let cavity = Point3::new(0.5, 0.5, 0.5);
        let material = Point3::new(0.1, 0.1, 0.1);

        for classify in [
            classify_point,
            classify_point_winding,
            classify_point_robust,
        ] {
            assert_eq!(
                classify(&topo, solid, cavity, &options).unwrap(),
                PointClassification::Outside
            );
            assert_eq!(
                classify(&topo, solid, material, &options).unwrap(),
                PointClassification::Inside
            );
        }

        let properties_options = crate::properties::PropertiesOptions::default();
        let volume = crate::properties::solid_volume(&topo, solid, &properties_options).unwrap();
        let area = crate::properties::solid_area(&topo, solid, &properties_options).unwrap();
        let center = crate::properties::center_of_mass(&topo, solid, &properties_options).unwrap();
        assert!((volume - 0.875).abs() < 1e-12, "volume={volume}");
        assert!((area - 7.5).abs() < 1e-12, "area={area}");
        assert!((center - Point3::new(0.5, 0.5, 0.5)).length() < 1e-12);
    }

    #[test]
    fn point_outside_box() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let far = Point3::new(5.0, 5.0, 5.0);
        let opts = ClassifyOptions::default();

        let result = classify_point(&topo, solid, far, &opts).unwrap();
        assert_eq!(result, PointClassification::Outside);
    }

    #[test]
    fn point_on_boundary_box() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        // Center of the top face (z=1).
        let on_face = Point3::new(0.5, 0.5, 1.0);
        let opts = ClassifyOptions::default();

        let result = classify_point(&topo, solid, on_face, &opts).unwrap();
        assert_eq!(result, PointClassification::OnBoundary);
    }

    #[test]
    fn point_near_edge_outside() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        // Just outside the box along the x-axis.
        let outside = Point3::new(1.001, 0.5, 0.5);
        let opts = ClassifyOptions::default();

        let result = classify_point(&topo, solid, outside, &opts).unwrap();
        assert_eq!(result, PointClassification::Outside);
    }

    #[test]
    fn point_at_corner_boundary() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        // Very close to a vertex of the box.
        let near_corner = Point3::new(0.0, 0.0, 0.0);
        let opts = ClassifyOptions::default();

        let result = classify_point(&topo, solid, near_corner, &opts).unwrap();
        assert_eq!(result, PointClassification::OnBoundary);
    }

    #[test]
    fn winding_inside_box() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let center = Point3::new(0.5, 0.5, 0.5);

        let w = winding::winding_number(&topo, solid, center).unwrap();
        assert!(
            w > 0.5,
            "winding number for interior point should be > 0.5, got {w}"
        );
    }

    #[test]
    fn winding_outside_box() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let far = Point3::new(5.0, 5.0, 5.0);

        let w = winding::winding_number(&topo, solid, far).unwrap();
        assert!(
            w < 0.5,
            "winding number for exterior point should be < 0.5, got {w}"
        );
    }

    #[test]
    fn classify_winding_matches_ray() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let center = Point3::new(0.5, 0.5, 0.5);
        let opts = ClassifyOptions::default();

        let ray_result = classify_point(&topo, solid, center, &opts).unwrap();
        let winding_result = classify_point_winding(&topo, solid, center, &opts).unwrap();
        assert_eq!(ray_result, winding_result);
    }

    #[test]
    fn point_negative_quadrant_outside() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let neg = Point3::new(-1.0, -1.0, -1.0);
        let opts = ClassifyOptions::default();

        let result = classify_point(&topo, solid, neg, &opts).unwrap();
        assert_eq!(result, PointClassification::Outside);
    }

    /// Add a planar face whose plane normal is taken from the loop's own
    /// winding, so every wire orientation stays consistent with the outward
    /// normal it implies.
    fn add_planar_face(
        topo: &mut Topology,
        outer: &[Point3],
        holes: &[Vec<Point3>],
        tol: f64,
    ) -> remus_topology::face::FaceId {
        use remus_topology::builder::make_polygon_wire;
        use remus_topology::face::Face;

        let normal = crate::util::polygon_normal(outer);
        let p0 = outer[0];
        let d = normal.dot(Vec3::new(p0.x(), p0.y(), p0.z()));
        let outer_wire = make_polygon_wire(topo, outer, tol).unwrap();
        let inner_wires = holes
            .iter()
            .map(|h| make_polygon_wire(topo, h, tol).unwrap())
            .collect();
        topo.add_face(Face::new(
            outer_wire,
            inner_wires,
            FaceSurface::Plane { normal, d },
        ))
    }

    /// Build a square slab (x,y in `[0,4]`, z in `[0,1]`) pierced by a square
    /// through-hole (x,y in `[1,3]`).
    ///
    /// The z=0 and z=1 caps each carry the hole as an inner wire — the shape
    /// a boolean cut leaves behind, and the one a classifier that reads only
    /// outer wires gets wrong. The hole is wide relative to the slab so the
    /// classifier's fixed ray directions actually travel up it instead of
    /// leaving through a side wall first.
    fn make_slab_with_square_hole(topo: &mut Topology) -> SolidId {
        use remus_topology::shell::Shell;

        const TOL: f64 = 1e-7;
        const Z0: f64 = 0.0;
        const Z1: f64 = 1.0;
        let (lo, hi) = (1.0_f64, 3.0_f64);

        // Square ring, counter-clockwise seen from +z.
        let ccw = |z: f64, a: f64, b: f64| {
            vec![
                Point3::new(a, a, z),
                Point3::new(b, a, z),
                Point3::new(b, b, z),
                Point3::new(a, b, z),
            ]
        };
        let cw = |z: f64, a: f64, b: f64| {
            let mut r = ccw(z, a, b);
            r.reverse();
            r
        };

        let mut faces = Vec::new();
        // Bottom cap: outer clockwise (normal -z), hole wound the other way.
        faces.push(add_planar_face(
            topo,
            &cw(Z0, 0.0, 4.0),
            &[ccw(Z0, lo, hi)],
            TOL,
        ));
        // Top cap: outer counter-clockwise (normal +z), hole reversed.
        faces.push(add_planar_face(
            topo,
            &ccw(Z1, 0.0, 4.0),
            &[cw(Z1, lo, hi)],
            TOL,
        ));

        // Walls. Traversing the base counter-clockwise then rising gives a
        // Newell normal of (dy, -dx) — outward for the slab rim. The hole
        // ring is walked clockwise so its walls face into the hole, i.e.
        // away from material in both cases.
        for ring in [ccw(Z0, 0.0, 4.0), cw(Z0, lo, hi)] {
            for i in 0..4 {
                let a = ring[i];
                let b = ring[(i + 1) % 4];
                faces.push(add_planar_face(
                    topo,
                    &[
                        Point3::new(a.x(), a.y(), Z0),
                        Point3::new(b.x(), b.y(), Z0),
                        Point3::new(b.x(), b.y(), Z1),
                        Point3::new(a.x(), a.y(), Z1),
                    ],
                    &[],
                    TOL,
                ));
            }
        }

        let shell = topo.add_shell(Shell::new(faces).unwrap());
        topo.add_solid(Solid::new(shell, vec![]))
    }

    /// The fixture itself must be a well-formed solid: every wall and cap
    /// normal points away from material, and the volume is the slab minus
    /// the through-hole.
    #[test]
    fn slab_with_square_hole_fixture_is_well_formed() {
        let mut topo = Topology::new();
        let solid = make_slab_with_square_hole(&mut topo);
        let opts = crate::properties::PropertiesOptions::default();
        let volume = crate::properties::solid_volume(&topo, solid, &opts).unwrap();
        assert!(
            (volume - 12.0).abs() < 1e-9,
            "4x4x1 slab minus a 2x2x1 hole should be 12, got {volume}"
        );
    }

    /// Regression: a ray passing through a face's hole must not be counted as
    /// a crossing. `face_polygon` returns only the outer wire, so a holed cap
    /// reported a crossing for rays travelling up the through-hole, flipping
    /// the parity and classifying exterior points as Inside.
    #[test]
    fn holed_face_does_not_count_crossings_through_the_hole() {
        let mut topo = Topology::new();
        let solid = make_slab_with_square_hole(&mut topo);
        let opts = ClassifyOptions::default();

        // Just below the bottom cap, under the hole: every ray crosses the
        // z=0 plane inside the hole, which is open space, not material.
        // Likewise just under the top cap, and mid-hole.
        let outside = [
            Point3::new(2.0, 2.0, -0.05),
            Point3::new(1.6, 2.3, -0.05),
            Point3::new(2.4, 1.7, -0.02),
            Point3::new(2.0, 2.0, 0.95),
            Point3::new(2.0, 2.0, 0.5),
            Point3::new(2.0, 2.0, 1.05),
        ];
        for p in outside {
            assert_eq!(
                classify_point(&topo, solid, p, &opts).unwrap(),
                PointClassification::Outside,
                "probe {p:?}"
            );
        }

        // Material away from the hole must still read Inside.
        let inside = [
            Point3::new(0.5, 0.5, 0.5),
            Point3::new(3.5, 2.0, 0.5),
            Point3::new(2.0, 0.5, 0.5),
            Point3::new(0.5, 3.5, 0.25),
            Point3::new(3.5, 3.5, 0.75),
        ];
        for p in inside {
            assert_eq!(
                classify_point(&topo, solid, p, &opts).unwrap(),
                PointClassification::Inside,
                "probe {p:?}"
            );
        }
    }

    /// A point sitting in the open space of a through-hole is not "on" the
    /// cap face whose plane it happens to share.
    #[test]
    fn point_in_hole_is_not_on_boundary() {
        let mut topo = Topology::new();
        let solid = make_slab_with_square_hole(&mut topo);
        let opts = ClassifyOptions::default();

        assert_eq!(
            classify_point(&topo, solid, Point3::new(2.0, 2.0, 0.0), &opts).unwrap(),
            PointClassification::Outside,
            "hole centre on the z=0 cap plane is open space"
        );
        // The cap itself, away from the hole, is still boundary.
        assert_eq!(
            classify_point(&topo, solid, Point3::new(0.5, 0.5, 0.0), &opts).unwrap(),
            PointClassification::OnBoundary
        );
    }

    /// The winding classifier subtracts holes too.
    #[test]
    fn winding_number_subtracts_face_holes() {
        let mut topo = Topology::new();
        let solid = make_slab_with_square_hole(&mut topo);

        let in_hole = winding::winding_number(&topo, solid, Point3::new(2.0, 2.0, 0.5)).unwrap();
        assert!(
            in_hole < 0.5,
            "through-hole interior should be out: {in_hole}"
        );
        let in_material =
            winding::winding_number(&topo, solid, Point3::new(0.5, 0.5, 0.5)).unwrap();
        assert!(
            in_material > 0.5,
            "slab material should be in: {in_material}"
        );
    }

    /// Build the 3-face solid a partial-turn circle revolve produces: one
    /// trimmed torus band (u in `[0, angle]`, full tube wrap; wire = two
    /// closed rim circles + a doubled seam arc, only 2 distinct vertices)
    /// plus two planar disc caps each bounded by a single closed circle.
    fn make_partial_torus_band(topo: &mut Topology, big_r: f64, rho: f64, angle: f64) -> SolidId {
        use remus_math::curves::Circle3D;
        use remus_math::surfaces::ToroidalSurface;
        use remus_topology::edge::{Edge, EdgeCurve};
        use remus_topology::face::{Face, FaceSurface};
        use remus_topology::shell::Shell;
        use remus_topology::solid::Solid;
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};

        let (sin_a, cos_a) = angle.sin_cos();
        let v1 = topo.add_vertex(Vertex::new(Point3::new(big_r, 0.0, -rho), 1e-7));
        let v2 = topo.add_vertex(Vertex::new(
            Point3::new(big_r * cos_a, big_r * sin_a, -rho),
            1e-7,
        ));

        let rim1 =
            Circle3D::new(Point3::new(big_r, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), rho).unwrap();
        let rim2 = Circle3D::new(
            Point3::new(big_r * cos_a, big_r * sin_a, 0.0),
            Vec3::new(-sin_a, cos_a, 0.0),
            rho,
        )
        .unwrap();
        let seam =
            Circle3D::new(Point3::new(0.0, 0.0, -rho), Vec3::new(0.0, 0.0, 1.0), big_r).unwrap();

        let mut rim1_edge = Edge::new(v1, v1, EdgeCurve::Circle(rim1.clone()));
        let rim1_anchor = rim1.project(topo.vertex(v1).unwrap().point());
        rim1_edge.set_trim(Some((rim1_anchor, rim1_anchor + std::f64::consts::TAU)));
        let e_rim1 = topo.add_edge(rim1_edge);

        let mut rim2_edge = Edge::new(v2, v2, EdgeCurve::Circle(rim2.clone()));
        let rim2_anchor = rim2.project(topo.vertex(v2).unwrap().point());
        rim2_edge.set_trim(Some((rim2_anchor, rim2_anchor + std::f64::consts::TAU)));
        let e_rim2 = topo.add_edge(rim2_edge);

        let mut seam_edge = Edge::new(v1, v2, EdgeCurve::Circle(seam.clone()));
        let seam_start = seam.project(topo.vertex(v1).unwrap().point());
        let canonical_seam_end = seam.project(topo.vertex(v2).unwrap().point());
        let seam_end = if canonical_seam_end <= seam_start {
            canonical_seam_end + std::f64::consts::TAU
        } else {
            canonical_seam_end
        };
        seam_edge.set_trim(Some((seam_start, seam_end)));
        let e_seam = topo.add_edge(seam_edge);

        let band_wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e_rim1, true),
                    OrientedEdge::new(e_seam, true),
                    OrientedEdge::new(e_rim2, false),
                    OrientedEdge::new(e_seam, false),
                ],
                true,
            )
            .unwrap(),
        );
        let torus = ToroidalSurface::with_axis(
            Point3::new(0.0, 0.0, 0.0),
            big_r,
            rho,
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();
        let band = topo.add_face(Face::new(band_wire, vec![], FaceSurface::Torus(torus)));

        let cap1_wire =
            topo.add_wire(Wire::new(vec![OrientedEdge::new(e_rim1, false)], true).unwrap());
        let cap1 = topo.add_face(Face::new(
            cap1_wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
        ));
        let cap2_wire =
            topo.add_wire(Wire::new(vec![OrientedEdge::new(e_rim2, true)], true).unwrap());
        let cap2 = topo.add_face(Face::new(
            cap2_wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(-sin_a, cos_a, 0.0),
                d: 0.0,
            },
        ));

        let shell = topo.add_shell(Shell::new(vec![band, cap1, cap2]).unwrap());
        topo.add_solid(Solid::new(shell, vec![]))
    }

    /// Regression: interior points of a partial-turn torus band read Outside.
    /// Two stacked roots: the local Ferrari ray-torus quartic missed real
    /// roots and emitted off-surface spurious ones, and `face_aabb` collapsed
    /// each cap disc (single closed-circle wire, one vertex) to a point AABB,
    /// so the BVH prefilter never offered the caps and their crossings were
    /// dropped from the parity count.
    #[test]
    fn partial_torus_band_interior_points() {
        let (big_r, rho, angle) = (6.0_f64, 2.0_f64, 2.0 * std::f64::consts::PI / 3.0);
        let mut topo = Topology::new();
        let solid = make_partial_torus_band(&mut topo, big_r, rho, angle);
        let opts = ClassifyOptions::default();

        let mid = angle / 2.0;
        let inside = [
            Point3::new(big_r * mid.cos(), big_r * mid.sin(), 0.0),
            Point3::new(big_r * mid.cos(), big_r * mid.sin(), 1.0),
            Point3::new(big_r * mid.cos(), big_r * mid.sin(), -1.0),
            Point3::new(big_r * 0.05f64.cos(), big_r * 0.05f64.sin(), 0.0),
            Point3::new(
                big_r * (angle - 0.05).cos(),
                big_r * (angle - 0.05).sin(),
                0.0,
            ),
            Point3::new((big_r - 1.5) * mid.cos(), (big_r - 1.5) * mid.sin(), 0.0),
            Point3::new((big_r + 1.5) * mid.cos(), (big_r + 1.5) * mid.sin(), 0.0),
        ];
        for p in inside {
            let result = classify_point(&topo, solid, p, &opts).unwrap();
            assert_eq!(result, PointClassification::Inside, "probe {p:?}");
        }

        let outside = [
            Point3::new(big_r * mid.cos(), big_r * mid.sin(), 2.5),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(-big_r, 0.0, 0.0),
            Point3::new(
                big_r * (angle + 0.1).cos(),
                big_r * (angle + 0.1).sin(),
                0.0,
            ),
            Point3::new(big_r * (-0.1f64).cos(), big_r * (-0.1f64).sin(), 0.0),
        ];
        for p in outside {
            let result = classify_point(&topo, solid, p, &opts).unwrap();
            assert_eq!(result, PointClassification::Outside, "probe {p:?}");
        }
    }

    /// A cap disc bounded by a single closed circle edge must get a full-disc
    /// AABB, not a point box at its lone seam vertex (the collapsed box
    /// starved the classifier's BVH prefilter).
    #[test]
    fn face_aabb_covers_closed_circle_boundary() {
        let (big_r, rho, angle) = (6.0_f64, 2.0_f64, 2.0 * std::f64::consts::PI / 3.0);
        let mut topo = Topology::new();
        let solid = make_partial_torus_band(&mut topo, big_r, rho, angle);
        let shell = topo
            .shell(topo.solid(solid).unwrap().outer_shell())
            .unwrap();

        // Face index 1 is the y=0 cap: disc center (6,0,0) radius 2 in the
        // xz-plane, so the AABB must span x in [4,8] and z in [-2,2].
        let cap = shell.faces()[1];
        let aabb = crate::util::face_aabb(&topo, cap).unwrap();
        assert!(
            aabb.min.x() < 4.0 + 1e-9 && aabb.max.x() > 8.0 - 1e-9,
            "cap AABB x-span collapsed: {aabb:?}"
        );
        assert!(
            aabb.min.z() < -2.0 + 1e-9 && aabb.max.z() > 2.0 - 1e-9,
            "cap AABB z-span collapsed: {aabb:?}"
        );
    }
}
