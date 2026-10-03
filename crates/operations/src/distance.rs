//! Distance measurement between shapes.
//!
//! Computes minimum distance between solids and point-to-solid distance.
//! Supports planar, NURBS, and analytic (cylinder, cone, sphere, torus) faces
//! with conservative branch-and-bound acceleration.
//! Point queries involving NURBS surfaces or trims are local numerical
//! projection estimates.
//! Solid-pair minima require supported global certificates; unsupported
//! curved pairs return a typed error.

#![allow(
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::suboptimal_flops,
    clippy::needless_range_loop,
    clippy::cast_precision_loss,
    clippy::doc_markdown,
    clippy::module_name_repetitions,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::manual_let_else,
    clippy::needless_pass_by_value,
    clippy::imprecise_flops
)]

use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

/// Result of a distance computation.
#[derive(Debug, Clone)]
pub struct DistanceResult {
    /// The minimum distance found.
    pub distance: f64,
    /// The closest point on the first shape.
    pub point_a: Point3,
    /// The closest point on the second shape.
    pub point_b: Point3,
}

/// Compute the minimum distance from a point to a solid.
///
/// Uses the shared best-first traversal over certified conservative face
/// bounds. NURBS surfaces and trims retain local numerical projection estimates,
/// including NURBS trims sewn onto analytic faces.
///
/// # Errors
///
/// Returns an error if the solid is invalid.
#[allow(clippy::too_many_lines)]
pub fn point_to_solid_distance(
    topo: &Topology,
    point: Point3,
    solid_id: SolidId,
) -> Result<DistanceResult, crate::OperationsError> {
    let (result, stats) = remus_check::distance::point_to_solid_with_stats(topo, point, solid_id)?;
    for _ in 0..stats.faces_evaluated {
        remus_algo::perf::bump_distance_face_probe();
    }
    Ok(DistanceResult {
        distance: result.distance,
        point_a: result.point_a,
        point_b: result.point_b,
    })
}

/// Query several points against one frozen boundary preparation.
///
/// Output order and the all-or-nothing error contract match individual calls;
/// preparation and scratch storage are shared across the batch.
///
/// # Errors
/// Returns an error for invalid topology, failed projection, or unsupported
/// extrema that conservative bounds cannot exclude from the final minimum.
pub fn point_to_solid_batch(
    topo: &Topology,
    points: &[Point3],
    solid: SolidId,
) -> Result<Vec<DistanceResult>, crate::OperationsError> {
    let prepared = remus_check::distance::PreparedDistanceSolid::prepare(topo, solid)?;
    let mut scratch = remus_check::distance::DistanceScratch::new();
    let mut results = Vec::with_capacity(points.len());
    for &point in points {
        let (result, stats) = prepared.query_with_stats(point, &mut scratch)?;
        for _ in 0..stats.faces_evaluated {
            remus_algo::perf::bump_distance_face_probe();
        }
        results.push(DistanceResult {
            distance: result.distance,
            point_a: result.point_a,
            point_b: result.point_b,
        });
    }
    Ok(results)
}

/// Compute the minimum distance between two solid boundaries.
///
/// Complete native sphere pairs and straight-edged planar boundaries (including
/// certified affine NURBS planes) have global minimum certificates. Unsupported
/// curved pairs return a typed error instead of a sampled/chord approximation.
/// Nested bodies measure the separation of their boundaries.
///
/// # Errors
/// Returns an error for missing topology or an unsupported minimum certificate.
pub fn solid_to_solid_distance(
    topo: &Topology,
    solid_a: SolidId,
    solid_b: SolidId,
) -> Result<DistanceResult, crate::OperationsError> {
    let result = remus_check::distance::solid_to_solid_with_face_probe(
        topo,
        solid_a,
        solid_b,
        remus_algo::perf::bump_distance_face_probe,
    )?;
    Ok(DistanceResult {
        distance: result.distance,
        point_a: result.point_a,
        point_b: result.point_b,
    })
}

/// Compute the minimum distance from a point to a face.
/// NURBS surfaces and trims retain local numerical estimates, not a global
/// minimum certificate.
///
/// # Errors
///
/// Returns an error if the face lookup fails.
pub fn point_to_face(
    topo: &Topology,
    point: Point3,
    face_id: FaceId,
) -> Result<DistanceResult, crate::OperationsError> {
    let Some((distance, closest)) = point_to_face_distance(topo, point, face_id, Tolerance::new())?
    else {
        return Err(crate::OperationsError::Check(
            remus_check::CheckError::DistanceFailed(
                "no point-to-face distance witness was found".into(),
            ),
        ));
    };
    Ok(DistanceResult {
        distance,
        point_a: point,
        point_b: closest,
    })
}

/// Compute the minimum distance to an authoritative line or circle interval.
/// Other carriers are refused until a global curve extremum solver is available.
///
/// # Errors
/// Returns an error for missing topology, invalid trims or unsupported carriers.
pub fn point_to_edge(
    topo: &Topology,
    point: Point3,
    edge_id: remus_topology::edge::EdgeId,
) -> Result<DistanceResult, crate::OperationsError> {
    let (distance, closest) = remus_check::distance::point_to_edge(topo, point, edge_id)?;
    Ok(DistanceResult {
        distance,
        point_a: point,
        point_b: closest,
    })
}

/// Shared trim-aware narrow phase. NURBS surfaces and trims retain the documented
/// local numerical estimates; they do not certify a solid-to-solid minimum.
pub(crate) fn point_to_face_distance(
    topo: &Topology,
    point: Point3,
    face_id: FaceId,
    tol: Tolerance,
) -> Result<Option<(f64, Point3)>, crate::OperationsError> {
    Ok(remus_check::distance::point_to_face_with_options(
        topo,
        point,
        face_id,
        remus_check::distance::DistanceOptions {
            projection_tolerance: tol.linear,
        },
    )?)
}

/// Point-in-polygon test for 3D (projecting to 2D).
pub(crate) fn point_in_polygon_3d(point: &Point3, polygon: &[Point3], normal: &Vec3) -> bool {
    use remus_math::predicates::point_in_polygon;
    use remus_math::vec::Point2;

    let ax = normal.x().abs();
    let ay = normal.y().abs();
    let az = normal.z().abs();

    let (proj_pt, proj_poly): (Point2, Vec<Point2>) = if az >= ax && az >= ay {
        (
            Point2::new(point.x(), point.y()),
            polygon.iter().map(|p| Point2::new(p.x(), p.y())).collect(),
        )
    } else if ay >= ax {
        (
            Point2::new(point.x(), point.z()),
            polygon.iter().map(|p| Point2::new(p.x(), p.z())).collect(),
        )
    } else {
        (
            Point2::new(point.y(), point.z()),
            polygon.iter().map(|p| Point2::new(p.y(), p.z())).collect(),
        )
    };

    point_in_polygon(proj_pt, &proj_poly)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::tolerance::Tolerance;
    use remus_math::vec::Point3;
    use remus_topology::Topology;
    use remus_topology::face::FaceSurface;
    use remus_topology::test_utils::make_unit_cube_manifold_at;

    use super::*;

    #[test]
    fn empty_operations_batch_checks_inner_references_after_unknown_outer_bound() {
        use remus_topology::wire::{OrientedEdge, Wire};
        let mut topo = Topology::new();
        let solid = crate::primitives::make_cylinder(&mut topo, 1.0, 1.0).unwrap();
        let face = remus_topology::explorer::solid_faces(&topo, solid)
            .unwrap()
            .into_iter()
            .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
            .unwrap();
        let outer = topo.face(face).unwrap().outer_wire();
        let edge = topo.wire(outer).unwrap().edges()[0].edge();
        // This produces an early unknown bound before the missing inner wire.
        topo.edge_mut(edge)
            .unwrap()
            .set_trim(Some((0.0, std::f64::consts::TAU + 0.5)));
        let mut other = Topology::new();
        let missing = (0..100)
            .map(|_| other.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap()))
            .last()
            .unwrap();
        let surface = topo.face(face).unwrap().surface().clone();
        *topo.face_mut(face).unwrap() =
            remus_topology::face::Face::new(outer, vec![missing], surface);
        assert!(matches!(
            point_to_solid_batch(&topo, &[], solid),
            Err(crate::OperationsError::Check(
                remus_check::CheckError::Topology(_)
            ))
        ));
        assert!(matches!(
            point_to_solid_batch(&topo, &[Point3::new(0.0, 0.0, 3.0)], solid),
            Err(crate::OperationsError::Check(
                remus_check::CheckError::Topology(_)
            ))
        ));
    }

    #[test]
    #[allow(clippy::too_many_lines, clippy::float_cmp)] // Exact witness parity across public entrypoints.
    fn bounded_narrow_failure_needs_a_strict_final_witness_gap_in_every_path() {
        use remus_math::curves::Ellipse3D;
        use remus_topology::builder::{make_face_from_wire, make_polygon_wire};
        use remus_topology::edge::{Edge, EdgeCurve};
        use remus_topology::face::Face;
        use remus_topology::shell::Shell;
        use remus_topology::solid::Solid;
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};
        for far in [true, false] {
            for reverse in [false, true] {
                let mut topo = Topology::new();
                let near_wire = make_polygon_wire(
                    &mut topo,
                    &[
                        Point3::new(-1.0, -1.0, 0.0),
                        Point3::new(1.0, -1.0, 0.0),
                        Point3::new(1.0, 1.0, 0.0),
                        Point3::new(-1.0, 1.0, 0.0),
                    ],
                    1e-7,
                )
                .unwrap();
                let near = make_face_from_wire(&mut topo, near_wire).unwrap();
                let x = if far { 100.0 } else { 0.0 };
                let ellipse =
                    Ellipse3D::new(Point3::new(x, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0, 1.0)
                        .unwrap();
                let vertex = topo.add_vertex(Vertex::new(ellipse.evaluate(0.0), 1e-7));
                let mut edge = Edge::new(vertex, vertex, EdgeCurve::Ellipse(ellipse));
                edge.set_trim(Some((0.0, std::f64::consts::TAU)));
                let edge = topo.add_edge(edge);
                let wire =
                    topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
                let unsupported = topo.add_face(Face::new(
                    wire,
                    vec![],
                    FaceSurface::Plane {
                        normal: Vec3::new(0.0, 0.0, 1.0),
                        d: 0.0,
                    },
                ));
                let faces = if reverse {
                    vec![unsupported, near]
                } else {
                    vec![near, unsupported]
                };
                let shell = topo.add_shell(Shell::new(faces).unwrap());
                let solid = topo.add_solid(Solid::new(shell, vec![]));
                let point = Point3::new(0.0, 0.0, 1.0);
                let prepared =
                    remus_check::distance::PreparedDistanceSolid::prepare(&topo, solid).unwrap();
                let mut scratch = remus_check::distance::DistanceScratch::new();
                let fast = remus_check::distance::point_to_solid_with_stats(&topo, point, solid);
                let slow = remus_check::distance::point_to_solid_exhaustive(&topo, point, solid);
                let cached = prepared.query_with_stats(point, &mut scratch);
                let cached_slow = prepared.query_exhaustive_with_stats(point, &mut scratch);
                let batch = prepared.batch(&[point], &mut scratch);
                let checked_batch =
                    remus_check::distance::point_to_solid_batch(&topo, &[point], solid);
                let operation = point_to_solid_distance(&topo, point, solid);
                let operation_batch = point_to_solid_batch(&topo, &[point], solid);
                if far {
                    let fast = fast.unwrap();
                    let slow = slow.unwrap();
                    assert_eq!(fast.0.distance, 1.0);
                    assert_eq!(fast.0.point_b, Point3::new(0.0, 0.0, 0.0));
                    assert_eq!(fast.1.faces_skipped_by_bound, 1);
                    assert_eq!(slow.1.faces_evaluated, 2);
                    assert_eq!(slow.1.narrow_phase_failures, 1);
                    assert_eq!(slow.0.distance, fast.0.distance);
                    assert_eq!(cached.unwrap().0.distance, 1.0);
                    assert_eq!(cached_slow.unwrap().0.distance, 1.0);
                    assert_eq!(batch.unwrap()[0].distance, 1.0);
                    assert_eq!(checked_batch.unwrap()[0].distance, 1.0);
                    assert_eq!(operation.unwrap().distance, 1.0);
                    assert_eq!(operation_batch.unwrap()[0].distance, 1.0);
                } else {
                    for error in [
                        fast.unwrap_err(),
                        slow.unwrap_err(),
                        cached.unwrap_err(),
                        cached_slow.unwrap_err(),
                        batch.unwrap_err(),
                        checked_batch.unwrap_err(),
                    ] {
                        assert!(matches!(error, remus_check::CheckError::DistanceFailed(_)));
                    }
                    assert!(matches!(
                        operation,
                        Err(crate::OperationsError::Check(
                            remus_check::CheckError::DistanceFailed(_)
                        ))
                    ));
                    assert!(matches!(
                        operation_batch,
                        Err(crate::OperationsError::Check(
                            remus_check::CheckError::DistanceFailed(_)
                        ))
                    ));
                }
            }
        }
    }

    #[test]
    fn far_unknown_carrier_failure_cannot_be_discharged() {
        use remus_math::curves::Ellipse3D;
        use remus_topology::builder::{make_face_from_wire, make_polygon_wire};
        use remus_topology::edge::{Edge, EdgeCurve};
        use remus_topology::face::Face;
        use remus_topology::shell::Shell;
        use remus_topology::solid::Solid;
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};
        let mut topo = Topology::new();
        let near_wire = make_polygon_wire(
            &mut topo,
            &[
                Point3::new(-1.0, -1.0, 0.0),
                Point3::new(1.0, -1.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(-1.0, 1.0, 0.0),
            ],
            1e-7,
        )
        .unwrap();
        let near = make_face_from_wire(&mut topo, near_wire).unwrap();
        // Exact tilted-plane intersections with the same radius-one cylinder.
        // Each ellipse has radial coordinates (-cos(t), -sin(t)), while its
        // height is center_z + sqrt(3)*cos(t).
        let normal = Vec3::new(3.0_f64.sqrt() * 0.5, 0.0, 0.5);
        let reference = Vec3::new(-0.5, 0.0, 3.0_f64.sqrt() * 0.5);
        let mut rims = Vec::new();
        let mut vertices = Vec::new();
        for height in [0.0, 4.0] {
            let ellipse = Ellipse3D::new_with_ref(
                Point3::new(100.0, 0.0, height),
                normal,
                2.0,
                1.0,
                reference,
            )
            .unwrap();
            let vertex = topo.add_vertex(Vertex::new(ellipse.evaluate(0.0), 1e-7));
            vertices.push(vertex);
            let mut edge = Edge::new(vertex, vertex, EdgeCurve::Ellipse(ellipse));
            edge.set_trim(Some((0.0, std::f64::consts::TAU)));
            rims.push(topo.add_edge(edge));
        }
        let seam = topo.add_edge(Edge::new(vertices[0], vertices[1], EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(rims[0], true),
                    OrientedEdge::new(seam, true),
                    OrientedEdge::new(rims[1], false),
                    OrientedEdge::new(seam, false),
                ],
                true,
            )
            .unwrap(),
        );
        let carrier = remus_math::surfaces::CylindricalSurface::new(
            Point3::new(100.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
        )
        .unwrap();
        let far = topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(carrier)));
        assert!(
            !remus_check::distance::face_bounds::face_bound(&topo, far)
                .unwrap()
                .prunable
        );
        let shell = topo.add_shell(Shell::new(vec![near, far]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        let point = Point3::new(0.0, 0.0, 1.0);
        let prepared = remus_check::distance::PreparedDistanceSolid::prepare(&topo, solid).unwrap();
        let mut scratch = remus_check::distance::DistanceScratch::new();
        for error in [
            remus_check::distance::point_to_solid(&topo, point, solid).unwrap_err(),
            remus_check::distance::point_to_solid_exhaustive(&topo, point, solid).unwrap_err(),
            prepared.query(point, &mut scratch).unwrap_err(),
            prepared
                .query_exhaustive_with_stats(point, &mut scratch)
                .unwrap_err(),
            prepared.batch(&[point], &mut scratch).unwrap_err(),
        ] {
            assert!(matches!(error, remus_check::CheckError::DistanceFailed(_)));
        }
        assert!(point_to_solid_distance(&topo, point, solid).is_err());
        assert!(point_to_solid_batch(&topo, &[point], solid).is_err());
    }

    #[test]
    fn complete_sphere_pairs_have_global_face_interior_extrema() {
        use remus_math::mat::Mat4;
        // Closed-form boundary gaps: separated, tangent, intersecting,
        // eccentric containment and concentric containment. The oblique
        // separation's minimizer is neither a primitive vertex nor an edge.
        for (ra, rb, delta, expected) in [
            (1.0, 1.0, Vec3::new(0.0, 0.0, 3.0), 1.0),
            (2.0, 3.0, Vec3::new(3.0, 4.0, 12.0), 8.0),
            (1.0, 1.0, Vec3::new(0.0, 0.0, 2.0), 0.0),
            (2.0, 1.0, Vec3::new(0.0, 0.0, 2.0), 0.0),
            (4.0, 1.0, Vec3::new(0.0, 0.0, 1.0), 2.0),
            (1.0, 4.0, Vec3::new(0.0, 0.0, 1.0), 2.0),
            (4.0, 1.0, Vec3::new(0.0, 0.0, 0.0), 3.0),
        ] {
            let mut topo = Topology::new();
            let a = crate::primitives::make_sphere(&mut topo, ra, 8).unwrap();
            let b = crate::primitives::make_sphere(&mut topo, rb, 12).unwrap();
            let center_a = Point3::new(2.0, -3.0, 5.0);
            let center_b = center_a + delta;
            crate::transform::transform_solid(&mut topo, a, &Mat4::translation(2.0, -3.0, 5.0))
                .unwrap();
            crate::transform::transform_solid(
                &mut topo,
                b,
                &Mat4::translation(center_b.x(), center_b.y(), center_b.z()),
            )
            .unwrap();
            let snapshot = format!("{topo:?}");
            let result = solid_to_solid_distance(&topo, a, b).unwrap();
            let checked = remus_check::distance::solid_to_solid(&topo, a, b).unwrap();
            let reverse = solid_to_solid_distance(&topo, b, a).unwrap();
            assert!((result.distance - expected).abs() < 1e-10, "{result:?}");
            assert!((reverse.distance - expected).abs() < 1e-10);
            assert!((checked.distance - expected).abs() < 1e-10);
            assert!(((result.point_a - center_a).length() - ra).abs() < 1e-10);
            assert!(((result.point_b - center_b).length() - rb).abs() < 1e-10);
            assert!(((result.point_a - result.point_b).length() - result.distance).abs() < 1e-10);
            assert_eq!(format!("{topo:?}"), snapshot);
        }
    }

    #[test]
    fn cropped_or_unsupported_curved_pairs_refuse_a_global_minimum() {
        let mut topo = Topology::new();
        let a = crate::primitives::make_sphere(&mut topo, 1.0, 8).unwrap();
        let b = crate::primitives::make_sphere(&mut topo, 1.0, 8).unwrap();
        let cylinder = crate::primitives::make_cylinder(&mut topo, 1.0, 1.0).unwrap();
        assert!(matches!(
            remus_check::distance::solid_to_solid(&topo, a, cylinder),
            Err(remus_check::CheckError::DistanceFailed(_))
        ));
        assert!(matches!(
            solid_to_solid_distance(&topo, a, cylinder),
            Err(crate::OperationsError::Check(
                remus_check::CheckError::DistanceFailed(_)
            ))
        ));
        let shell = topo.solid(b).unwrap().outer_shell();
        let face = topo.shell(shell).unwrap().faces()[0];
        *topo.shell_mut(shell).unwrap() = remus_topology::shell::Shell::new(vec![face]).unwrap();
        assert!(matches!(
            solid_to_solid_distance(&topo, a, b),
            Err(crate::OperationsError::Check(
                remus_check::CheckError::DistanceFailed(_)
            ))
        ));
    }

    #[test]
    fn overflowing_line_or_planar_distance_refuses_a_nonfinite_minimum() {
        use remus_topology::edge::{Edge, EdgeCurve};
        use remus_topology::vertex::Vertex;
        let mut topo = Topology::new();
        let a = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let b = topo.add_vertex(Vertex::new(Point3::new(1e200, 0.0, 0.0), 1e-7));
        let line = topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
        assert!(matches!(
            point_to_edge(&topo, Point3::new(1.0, 1.0, 0.0), line),
            Err(crate::OperationsError::Check(
                remus_check::CheckError::DistanceFailed(_)
            ))
        ));
        let left = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let right = crate::primitives::make_box(&mut topo, 1e200, 1.0, 1.0).unwrap();
        assert!(matches!(
            solid_to_solid_distance(&topo, left, right),
            Err(crate::OperationsError::Check(
                remus_check::CheckError::DistanceFailed(_)
            ))
        ));
    }

    #[test]
    fn large_finite_sphere_gap_has_finite_carrier_witnesses() {
        let mut topo = Topology::new();
        let radius = 1e154;
        let a = crate::primitives::make_sphere(&mut topo, radius, 8).unwrap();
        let b = crate::primitives::make_sphere(&mut topo, radius, 8).unwrap();
        let offset = Vec3::new(0.0, 0.0, 3e154);
        for vertex in remus_topology::explorer::solid_vertices(&topo, b).unwrap() {
            let point = topo.vertex(vertex).unwrap().point();
            topo.vertex_mut(vertex).unwrap().set_point(point + offset);
        }
        for face in remus_topology::explorer::solid_faces(&topo, b).unwrap() {
            topo.face_mut(face)
                .unwrap()
                .set_surface(FaceSurface::Sphere(
                    remus_math::surfaces::SphericalSurface::new(
                        Point3::new(0.0, 0.0, 3e154),
                        radius,
                    )
                    .unwrap(),
                ));
        }
        let result = solid_to_solid_distance(&topo, a, b).unwrap();
        assert!(result.distance.is_finite());
        assert!((result.distance / radius - 1.0).abs() < 1e-12, "{result:?}");
        let length = |point: Point3, center: Point3| {
            let offset = point - center;
            offset.x().hypot(offset.y()).hypot(offset.z())
        };
        assert!((length(result.point_a, Point3::new(0.0, 0.0, 0.0)) / radius - 1.0).abs() < 1e-12);
        assert!(
            (length(result.point_b, Point3::new(0.0, 0.0, 3e154)) / radius - 1.0).abs() < 1e-12
        );
    }

    #[test]
    fn native_sphere_point_witness_is_on_the_surface_not_an_equator_chord() {
        let mut topo = Topology::new();
        let sphere = crate::primitives::make_sphere(&mut topo, 1.0, 8).unwrap();
        for point in [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.2, 0.2, 0.1),
            Point3::new(0.2, 0.2, -0.1),
        ] {
            let result = remus_check::distance::point_to_solid(&topo, point, sphere).unwrap();
            let operation = point_to_solid_distance(&topo, point, sphere).unwrap();
            assert!(
                (result.distance - (1.0 - (point - Point3::new(0.0, 0.0, 0.0)).length())).abs()
                    < 1e-10
            );
            assert!(((result.point_b - Point3::new(0.0, 0.0, 0.0)).length() - 1.0).abs() < 1e-10);
            assert!((operation.distance - result.distance).abs() < 1e-10);
        }
    }

    #[test]
    fn holed_plate_distance_witness_is_on_the_rim_in_all_paths() {
        use remus_math::mat::Mat4;
        let mut topo = Topology::new();
        let plate = crate::primitives::make_box(&mut topo, 10.0, 10.0, 2.0).unwrap();
        let tool = crate::primitives::make_box(&mut topo, 2.0, 2.0, 4.0).unwrap();
        crate::transform::transform_solid(&mut topo, tool, &Mat4::translation(4.0, 4.0, -1.0))
            .unwrap();
        let solid = crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Cut, plate, tool)
            .unwrap();
        let points = [
            Point3::new(5.0, 5.0, 3.0),
            Point3::new(5.0, 5.0, 1.0),
            Point3::new(2.0, 2.0, 3.0),
        ];
        let expected = [2.0_f64.sqrt(), 1.0, 1.0];
        let prepared = remus_check::distance::PreparedDistanceSolid::prepare(&topo, solid).unwrap();
        let mut scratch = remus_check::distance::DistanceScratch::new();
        let batch = remus_check::distance::point_to_solid_batch(&topo, &points, solid).unwrap();
        let operation_batch = point_to_solid_batch(&topo, &points, solid).unwrap();
        for (i, point) in points.into_iter().enumerate() {
            let checked = remus_check::distance::point_to_solid(&topo, point, solid).unwrap();
            let exhaustive = remus_check::distance::point_to_solid_exhaustive(&topo, point, solid)
                .unwrap()
                .0;
            let cached = prepared.query(point, &mut scratch).unwrap();
            let operation = point_to_solid_distance(&topo, point, solid).unwrap();
            for result in [&checked, &exhaustive, &cached, &batch[i]] {
                assert!((result.distance - expected[i]).abs() < 1e-10, "{result:?}");
                assert!(((point - result.point_b).length() - result.distance).abs() < 1e-10);
                // No witness may float in the centre of the through hole.
                assert!(
                    !(result.point_b.x() > 4.0
                        && result.point_b.x() < 6.0
                        && result.point_b.y() > 4.0
                        && result.point_b.y() < 6.0)
                );
            }
            assert!((operation.distance - expected[i]).abs() < 1e-10);
            assert!((operation_batch[i].distance - expected[i]).abs() < 1e-10);
        }
    }

    #[test]
    fn clockwise_circular_hole_distance_keeps_witnesses_on_the_annular_boundary() {
        use remus_topology::face::Face;
        use remus_topology::wire::{OrientedEdge, Wire};
        let mut topo = Topology::new();
        let mut rim = |radius, normal_z| {
            let edge = remus_topology::builder::make_circle_edge_with_ref(
                &mut topo,
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, normal_z),
                radius,
                Vec3::new(0.0, 1.0, 0.0),
                1e-7,
            )
            .unwrap();
            topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap())
        };
        let outer = rim(3.0, 1.0);
        let hole = rim(1.0, -1.0);
        let face = topo.add_face(Face::new(
            outer,
            vec![hole],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let solid =
            crate::extrude::extrude(&mut topo, face, Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        assert!(
            crate::validate::validate_solid(&topo, solid)
                .unwrap()
                .is_valid()
        );
        let snapshot = format!("{topo:?}");
        let queries = [
            (Point3::new(0.0, 0.0, -1.0), 2.0_f64.sqrt(), 1.0, 0.0),
            (Point3::new(0.0, 0.0, 3.0), 2.0_f64.sqrt(), 1.0, 2.0),
            (Point3::new(2.0, 0.0, -1.0), 1.0, 2.0, 0.0),
            (Point3::new(2.0, 0.0, 3.0), 1.0, 2.0, 2.0),
        ];
        let points: Vec<_> = queries.iter().map(|entry| entry.0).collect();
        let batch = point_to_solid_batch(&topo, &points, solid).unwrap();
        let prepared = remus_check::distance::PreparedDistanceSolid::prepare(&topo, solid).unwrap();
        let mut scratch = remus_check::distance::DistanceScratch::new();
        for (i, (point, expected, radius, z)) in queries.into_iter().enumerate() {
            let direct = point_to_solid_distance(&topo, point, solid).unwrap();
            let checked = remus_check::distance::point_to_solid(&topo, point, solid).unwrap();
            let exhaustive = remus_check::distance::point_to_solid_exhaustive(&topo, point, solid)
                .unwrap()
                .0;
            let cached = prepared.query(point, &mut scratch).unwrap();
            let cached_exhaustive = prepared
                .query_exhaustive_with_stats(point, &mut scratch)
                .unwrap()
                .0;
            for (distance, witness) in [
                (direct.distance, direct.point_b),
                (batch[i].distance, batch[i].point_b),
                (checked.distance, checked.point_b),
                (exhaustive.distance, exhaustive.point_b),
                (cached.distance, cached.point_b),
                (cached_exhaustive.distance, cached_exhaustive.point_b),
            ] {
                assert!(
                    (distance - expected).abs() < 1e-10,
                    "distance={distance}, witness={witness:?}"
                );
                assert!((witness.x().hypot(witness.y()) - radius).abs() < 1e-10);
                assert!((witness.z() - z).abs() < 1e-10);
                assert!(((point - witness).length() - distance).abs() < 1e-10);
            }
            if point.z() < 0.0 {
                let face_result = point_to_face(&topo, point, face).unwrap();
                assert!(
                    (face_result.distance - expected).abs() < 1e-10,
                    "{face_result:?}"
                );
                assert!(
                    (face_result.point_b.x().hypot(face_result.point_b.y()) - radius).abs() < 1e-10
                );
                assert!(face_result.point_b.z().abs() < 1e-10);
            }
        }
        assert_eq!(format!("{topo:?}"), snapshot);
    }

    #[test]
    fn cylinder_lateral_distance_uses_the_closed_curved_rim() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_cylinder(&mut topo, 1.0, 1.0).unwrap();
        let face = remus_topology::explorer::solid_faces(&topo, solid)
            .unwrap()
            .into_iter()
            .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
            .unwrap();
        let point = Point3::new(-2.0, 0.0, 2.0);
        let checked = remus_check::distance::point_to_face(&topo, point, face)
            .unwrap()
            .unwrap();
        let operation = point_to_face(&topo, point, face).unwrap();
        assert!((checked.0 - 2.0_f64.sqrt()).abs() < 1e-10);
        assert!((checked.1 - Point3::new(-1.0, 0.0, 1.0)).length() < 1e-10);
        assert!((operation.point_b - checked.1).length() < 1e-10);
        assert!((operation.distance - checked.0).abs() < 1e-10);
    }

    #[test]
    fn crossing_planar_faces_have_a_shared_boundary_witness() {
        use remus_math::mat::Mat4;
        let mut topo = Topology::new();
        let a = crate::primitives::make_box(&mut topo, 6.0, 1.0, 1.0).unwrap();
        let b = crate::primitives::make_box(&mut topo, 1.0, 6.0, 1.0).unwrap();
        crate::transform::transform_solid(&mut topo, a, &Mat4::translation(-3.0, -0.5, -0.5))
            .unwrap();
        crate::transform::transform_solid(&mut topo, b, &Mat4::translation(-0.5, -3.0, -0.1))
            .unwrap();
        // No vertex lies in the other body's boundary, and the crossing
        // horizontal edges have different heights. Edge-face interiors meet.
        let result = solid_to_solid_distance(&topo, a, b).unwrap();
        assert!(result.distance < 1e-10);
        for solid in [a, b] {
            let witness =
                remus_check::distance::point_to_solid(&topo, result.point_a, solid).unwrap();
            assert!(witness.distance < 1e-10, "{witness:?}");
        }
    }

    #[test]
    fn planar_solid_distance_includes_cavity_boundaries() {
        use remus_math::mat::Mat4;
        let mut topo = Topology::new();
        let outside = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let cavity = crate::primitives::make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
        crate::transform::transform_solid(&mut topo, cavity, &Mat4::translation(3.0, 3.0, 3.0))
            .unwrap();
        let hollow =
            crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Cut, outside, cavity)
                .unwrap();
        let inner = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        crate::transform::transform_solid(&mut topo, inner, &Mat4::translation(4.5, 4.5, 4.5))
            .unwrap();
        let result = solid_to_solid_distance(&topo, hollow, inner).unwrap();
        assert!((result.distance - 1.5).abs() < 1e-10, "{result:?}");
        assert!(
            remus_check::distance::point_to_solid(&topo, result.point_a, hollow)
                .unwrap()
                .distance
                < 1e-10
        );
        assert!(
            remus_check::distance::point_to_solid(&topo, result.point_b, inner)
                .unwrap()
                .distance
                < 1e-10
        );
    }

    #[test]
    fn circular_edge_distance_uses_trim_authority_and_both_endpoints() {
        use remus_math::curves::Circle3D;
        use remus_topology::edge::{Edge, EdgeCurve};
        use remus_topology::vertex::Vertex;
        let mut topo = Topology::new();
        let circle = Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let (lo, hi) = (1.5 * std::f64::consts::PI, 2.5 * std::f64::consts::PI);
        let a = topo.add_vertex(Vertex::new(circle.evaluate(lo), 1e-7));
        let b = topo.add_vertex(Vertex::new(circle.evaluate(hi), 1e-7));
        for (start, end, trim) in [(a, b, (lo, hi)), (b, a, (hi, lo))] {
            let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle.clone()));
            edge.set_trim(Some(trim));
            let edge = topo.add_edge(edge);
            let result = point_to_edge(&topo, Point3::new(-2.0, 0.0, 0.0), edge).unwrap();
            assert!(
                (result.distance - 5.0_f64.sqrt()).abs() < 1e-10,
                "{result:?}"
            );
            assert!(result.point_b.x().abs() < 1e-10);
            assert!((result.point_b.y().abs() - 1.0).abs() < 1e-10);
        }
        let seam = topo.add_vertex(Vertex::new(circle.evaluate(1.2), 1e-7));
        let mut closed = Edge::new(seam, seam, EdgeCurve::Circle(circle.clone()));
        closed.set_trim(Some((1.2, 1.2 + std::f64::consts::TAU)));
        let closed = topo.add_edge(closed);
        let result = point_to_edge(&topo, Point3::new(-2.0, 0.0, 0.0), closed).unwrap();
        assert!((result.distance - 1.0).abs() < 1e-10);
        assert!((result.point_b - Point3::new(-1.0, 0.0, 0.0)).length() < 1e-10);
        let missing_trim = topo.add_edge(Edge::new(seam, seam, EdgeCurve::Circle(circle)));
        assert!(point_to_edge(&topo, Point3::new(-2.0, 0.0, 0.0), missing_trim).is_err());
    }

    #[test]
    #[allow(clippy::too_many_lines)]
    fn analytic_face_nurbs_trim_keeps_local_curve_and_sewn_vertex_witnesses() {
        use remus_math::curves::Circle3D;
        use remus_math::surfaces::CylindricalSurface;
        use remus_topology::edge::{Edge, EdgeCurve};
        use remus_topology::face::{Face, FaceSurface};
        use remus_topology::shell::Shell;
        use remus_topology::solid::Solid;
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};

        let mut topo = Topology::new();
        let axis = Vec3::new(0.0, 0.0, 1.0);
        // The sewn trim is 2e-5 from its carrier, as in imported STEP blends.
        let bottom = Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            axis,
            1.000_02,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let curve =
            remus_geometry::convert::circle_to_nurbs(&bottom, 0.0, std::f64::consts::TAU).unwrap();
        let trim = curve.domain();
        let on_curve = curve.evaluate(trim.0 + 0.371 * (trim.1 - trim.0));
        let sewn_vertex = curve.evaluate(trim.0) + axis * 1e-5;
        let low = topo.add_vertex(Vertex::new(sewn_vertex, 3e-5));
        let top = Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 1.0),
            axis,
            1.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let high = topo.add_vertex(Vertex::new(top.evaluate(0.0), 3e-5));
        let mut lower = Edge::new(low, low, EdgeCurve::NurbsCurve(curve));
        lower.set_trim(Some(trim));
        let lower = topo.add_edge(lower);
        let mut upper = Edge::new(high, high, EdgeCurve::Circle(top));
        upper.set_trim(Some((0.0, std::f64::consts::TAU)));
        let upper = topo.add_edge(upper);
        let seam = topo.add_edge(Edge::new(low, high, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(lower, true),
                    OrientedEdge::new(seam, true),
                    OrientedEdge::new(upper, false),
                    OrientedEdge::new(seam, false),
                ],
                true,
            )
            .unwrap(),
        );
        let carrier = CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), axis, 1.0).unwrap();
        let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(carrier)));
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        let points = [on_curve, sewn_vertex];
        let prepared = remus_check::distance::PreparedDistanceSolid::prepare(&topo, solid).unwrap();
        let mut scratch = remus_check::distance::DistanceScratch::new();
        let batch = point_to_solid_batch(&topo, &points, solid).unwrap();
        for (i, point) in points.into_iter().enumerate() {
            let direct = point_to_solid_distance(&topo, point, solid).unwrap();
            let checked = remus_check::distance::point_to_solid(&topo, point, solid).unwrap();
            let exhaustive = remus_check::distance::point_to_solid_exhaustive(&topo, point, solid)
                .unwrap()
                .0;
            let cached = prepared.query(point, &mut scratch).unwrap();
            for result in [&checked, &exhaustive, &cached] {
                assert!(result.distance < 1e-10, "{result:?}");
                assert!((result.point_b - point).length() < 1e-10, "{result:?}");
            }
            for result in [
                &direct,
                &batch[i],
                &point_to_face(&topo, point, face).unwrap(),
            ] {
                assert!(result.distance < 1e-10, "{result:?}");
                assert!((result.point_b - point).length() < 1e-10, "{result:?}");
            }
        }
        // Local point estimates do not authorize an exact edge or solid-pair minimum.
        assert!(matches!(
            remus_check::distance::point_to_edge(&topo, on_curve, lower),
            Err(remus_check::CheckError::DistanceFailed(_))
        ));
        let other = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        assert!(matches!(
            solid_to_solid_distance(&topo, solid, other),
            Err(crate::OperationsError::Check(
                remus_check::CheckError::DistanceFailed(_)
            ))
        ));
    }

    /// A hollow solid's cavity wall is boundary. Walking only `outer_shell()`
    /// answers this with the distance to the OUTER wall, which is both wrong and
    /// larger — the failure mode CLAUDE.md's "Walking faces in a solid" warns
    /// about, and it was live here.
    #[test]
    fn distance_sees_the_cavity_wall_of_a_hollow_solid() {
        let mut topo = Topology::new();
        let block = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        // A fully-interior tool leaves a void: the result carries an inner shell
        // spanning 3.0 ..= 7.0 in every axis.
        let void = crate::primitives::make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
        crate::transform::transform_solid(
            &mut topo,
            void,
            &remus_math::mat::Mat4::translation(3.0, 3.0, 3.0),
        )
        .unwrap();
        let hollow =
            crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Cut, block, void)
                .unwrap();
        assert!(
            !topo.solid(hollow).unwrap().inner_shells().is_empty(),
            "test needs a solid that actually has a cavity shell"
        );

        // From the cavity centre the nearest boundary is the cavity wall at 2.0.
        // Outer-shell-only walking reports 5.0 (the outer wall).
        let result = point_to_solid_distance(&topo, Point3::new(5.0, 5.0, 5.0), hollow).unwrap();
        assert!(
            (result.distance - 2.0).abs() < 1e-6,
            "expected the cavity wall at 2.0, got {} (5.0 means only the outer shell was walked)",
            result.distance
        );
    }

    #[test]
    fn point_inside_cube_distance_is_half() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);

        // Point at center of cube — closest face is 0.5 away.
        let result = point_to_solid_distance(&topo, Point3::new(0.5, 0.5, 0.5), cube).unwrap();
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(result.distance, 0.5),
            "center-to-face distance should be ~0.5, got {}",
            result.distance
        );
    }

    #[test]
    fn point_outside_cube_distance() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);

        // Point above the cube.
        let result = point_to_solid_distance(&topo, Point3::new(0.5, 0.5, 3.0), cube).unwrap();
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(result.distance, 2.0),
            "point 2 above cube top should be distance ~2.0, got {}",
            result.distance
        );
    }

    #[test]
    fn disjoint_cubes_distance() {
        let mut topo = Topology::new();
        let a = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);
        let b = make_unit_cube_manifold_at(&mut topo, 5.0, 0.0, 0.0);

        let result = solid_to_solid_distance(&topo, a, b).unwrap();
        let tol = Tolerance::loose();
        // Cubes are [0,1] and [5,6], gap is 4.0.
        assert!(
            tol.approx_eq(result.distance, 4.0),
            "disjoint cubes should be ~4.0 apart, got {}",
            result.distance
        );
    }

    #[test]
    fn adjacent_cubes_distance_is_zero() {
        let mut topo = Topology::new();
        let a = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);
        let b = make_unit_cube_manifold_at(&mut topo, 1.0, 0.0, 0.0);

        let result = solid_to_solid_distance(&topo, a, b).unwrap();
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(result.distance, 0.0),
            "touching cubes should have distance ~0, got {}",
            result.distance
        );
    }

    #[test]
    fn same_solid_distance_is_zero() {
        let mut topo = Topology::new();
        let a = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);

        let result = solid_to_solid_distance(&topo, a, a).unwrap();
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(result.distance, 0.0),
            "distance to self should be 0, got {}",
            result.distance
        );
    }

    #[test]
    fn point_to_sphere_distance() {
        let sphere =
            remus_math::surfaces::SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 5.0).unwrap();
        let projection =
            remus_geometry::extrema::point_to_sphere(Point3::new(10.0, 0.0, 0.0), &sphere);
        let (dist, closest) = (projection.distance, projection.point);
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(dist, 5.0),
            "distance to sphere should be ~5.0, got {dist}"
        );
        assert!(
            tol.approx_eq(closest.x(), 5.0),
            "closest x should be ~5.0, got {}",
            closest.x()
        );
    }

    #[test]
    fn point_to_cylinder_distance() {
        let cyl = remus_math::surfaces::CylindricalSurface::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            3.0,
        )
        .unwrap();
        let dist =
            remus_geometry::extrema::point_to_cylinder(Point3::new(5.0, 0.0, 1.0), &cyl).distance;
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(dist, 2.0),
            "distance to cylinder should be ~2.0, got {dist}"
        );
    }

    #[test]
    fn segment_to_segment_parallel() {
        let (dist, _, _) = remus_geometry::extrema::segment_segment_distance(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 3.0, 0.0),
            Point3::new(1.0, 3.0, 0.0),
        );
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(dist, 3.0),
            "parallel segments 3 apart should have distance ~3.0, got {dist}"
        );
    }

    #[test]
    fn segment_to_segment_crossing() {
        let (dist, _, _) = remus_geometry::extrema::segment_segment_distance(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, -1.0),
            Point3::new(0.5, 0.0, 1.0),
        );
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(dist, 0.0),
            "crossing segments should have distance ~0, got {dist}"
        );
    }
}
