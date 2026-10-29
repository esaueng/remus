//! Closed-form maximum oracles, independent of the candidate enumeration.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

use super::{
    MaximumDistanceError, solid_to_solid_maximum_distance as maximum,
    solid_to_solid_maximum_distance_with_context as with_context,
};
use remus_math::context::{CancellationToken, OperationContext, WorkBudgets};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_topology::{
    Topology,
    solid::{Solid, SolidId},
    vertex::Vertex,
};

fn box_at(topo: &mut Topology, dimensions: [f64; 3], offset: [f64; 3]) -> SolidId {
    let solid =
        crate::primitives::make_box(topo, dimensions[0], dimensions[1], dimensions[2]).unwrap();
    crate::transform::transform_solid(
        topo,
        solid,
        &Mat4::translation(offset[0], offset[1], offset[2]),
    )
    .unwrap();
    solid
}
fn sphere_at(topo: &mut Topology, radius: f64, offset: [f64; 3]) -> SolidId {
    let solid = crate::primitives::make_sphere(topo, radius, 16).unwrap();
    crate::transform::transform_solid(
        topo,
        solid,
        &Mat4::translation(offset[0], offset[1], offset[2]),
    )
    .unwrap();
    solid
}

fn affine_faces(topo: &mut Topology, solid: SolidId, every_face: bool) {
    use remus_geometry::convert::certify_affine_nurbs_plane;
    use remus_math::curves2d::{Curve2D, Line2D};
    use remus_math::nurbs::surface::NurbsSurface;
    use remus_math::vec::{Point2, Vec2};
    use remus_topology::{face::FaceSurface, pcurve::PCurve};

    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    for face in faces
        .into_iter()
        .take(if every_face { usize::MAX } else { 1 })
    {
        let FaceSurface::Plane { normal, .. } = *topo.face(face).unwrap().surface() else {
            unreachable!("axis-aligned source plane required");
        };
        let uses: Vec<_> = topo
            .loops_of_face(face)
            .unwrap()
            .iter()
            .flat_map(|&l| topo.face_loop(l).unwrap().coedges().to_vec())
            .collect();
        let points: Vec<_> = uses
            .iter()
            .flat_map(|&c| {
                let e = topo.edge(topo.coedge(c).unwrap().edge()).unwrap();
                [
                    topo.vertex(e.start()).unwrap().point(),
                    topo.vertex(e.end()).unwrap().point(),
                ]
            })
            .collect();
        let axis = (0..3).find(|&i| normal.0[i] != 0.0).unwrap();
        let (u, v) = if normal.0[axis] > 0.0 {
            ((axis + 1) % 3, (axis + 2) % 3)
        } else {
            ((axis + 2) % 3, (axis + 1) % 3)
        };
        let low = std::array::from_fn::<_, 3, _>(|i| {
            points.iter().map(|p| p.0[i]).fold(f64::INFINITY, f64::min)
        });
        let high = std::array::from_fn::<_, 3, _>(|i| {
            points
                .iter()
                .map(|p| p.0[i])
                .fold(f64::NEG_INFINITY, f64::max)
        });
        let corner = |upper_u, upper_v| {
            let mut p = low;
            if upper_u {
                p[u] = high[u];
            }
            if upper_v {
                p[v] = high[v];
            }
            Point3::new(p[0], p[1], p[2])
        };
        let surface = NurbsSurface::new(
            1,
            1,
            vec![2.0, 2.0, 8.0, 8.0],
            vec![-3.0, -3.0, 3.0, 3.0],
            vec![
                vec![corner(false, false), corner(false, true)],
                vec![corner(true, false), corner(true, true)],
            ],
            vec![vec![3.0; 2]; 2],
        )
        .unwrap();
        let cert = certify_affine_nurbs_plane(&surface, 1e-7).unwrap();
        topo.face_mut(face)
            .unwrap()
            .set_surface(FaceSurface::Nurbs(surface));
        for coedge in uses {
            let usage = topo.coedge(coedge).unwrap();
            let edge = topo.edge(usage.edge()).unwrap();
            let a = cert
                .parameters(topo.vertex(edge.start()).unwrap().point(), 1e-7)
                .unwrap();
            let b = cert
                .parameters(topo.vertex(edge.end()).unwrap().point(), 1e-7)
                .unwrap();
            let delta = Vec2::new(b.0 - a.0, b.1 - a.1);
            let span = delta.length();
            let range = if usage.is_forward() {
                (0.0, span)
            } else {
                (span, 0.0)
            };
            let curve = Curve2D::Line(Line2D::new(Point2::new(a.0, a.1), delta).unwrap());
            topo.set_coedge_pcurve(coedge, PCurve::new(curve, range.0, range.1))
                .unwrap();
        }
    }
}

#[test]
fn maximum_affine_nurbs_boxes_keep_global_witnesses_and_read_only_state() {
    for scale in [1e-3, 1.0, 1e3] {
        for placed in [false, true] {
            for all in [false, true] {
                let mut topo = Topology::new();
                let a = box_at(&mut topo, [2.0 * scale, 3.0 * scale, 4.0 * scale], [0.0; 3]);
                let b = box_at(
                    &mut topo,
                    [5.0 * scale, 2.0 * scale, scale],
                    [8.0 * scale, -4.0 * scale, 2.0 * scale],
                );
                affine_faces(&mut topo, a, all);
                affine_faces(&mut topo, b, true);
                let pose = if placed {
                    Mat4([
                        [0.0, 0.0, 1.0, 17.0 * scale],
                        [1.0, 0.0, 0.0, -11.0 * scale],
                        [0.0, 1.0, 0.0, 9.0 * scale],
                        [0.0, 0.0, 0.0, 1.0],
                    ])
                } else {
                    Mat4::identity()
                };
                for solid in [a, b] {
                    crate::transform::transform_solid(&mut topo, solid, &pose).unwrap();
                }
                let before = remus_io::arena_io::serialize_solids(&topo, &[a, b]).unwrap();
                let tick = topo.mutation_ticks();
                let journal = topo.journal().len();
                let r = maximum(&topo, a, b).unwrap();
                separation(&r, 227.0_f64.sqrt() * scale, scale);
                point_close(
                    r.point_a,
                    pose.mul_point(Point3::new(0.0, 3.0 * scale, 0.0)),
                    scale,
                );
                point_close(
                    r.point_b,
                    pose.mul_point(Point3::new(13.0 * scale, -4.0 * scale, 3.0 * scale)),
                    scale,
                );
                let rev = maximum(&topo, b, a).unwrap();
                point_close(rev.point_a, r.point_b, scale);
                point_close(rev.point_b, r.point_a, scale);
                separation(
                    &maximum(&topo, a, a).unwrap(),
                    29.0_f64.sqrt() * scale,
                    scale,
                );
                assert_eq!(
                    remus_io::arena_io::serialize_solids(&topo, &[a, b]).unwrap(),
                    before
                );
                assert_eq!(topo.mutation_ticks(), tick);
                assert_eq!(topo.journal().len(), journal);
                let mut context = OperationContext::new();
                context.budgets.vertex_pairs = 0;
                assert!(matches!(
                    with_context(&topo, a, b, &context),
                    Err(MaximumDistanceError::ResourceLimit { .. })
                ));
                let token = CancellationToken::new();
                token.cancel();
                let context = context.with_cancellation(token);
                assert!(matches!(
                    with_context(&topo, a, b, &context),
                    Err(MaximumDistanceError::Cancelled)
                ));
            }
        }
    }
}

#[test]
fn maximum_affine_nurbs_mixes_with_complete_native_spheres() {
    for scale in [1e-3, 1.0, 1e3] {
        let mut topo = Topology::new();
        let a = box_at(
            &mut topo,
            [2.0 * scale, 3.0 * scale, 4.0 * scale],
            [4.0 * scale, -3.0 * scale, scale],
        );
        let b = sphere_at(&mut topo, 2.0 * scale, [0.0; 3]);
        affine_faces(&mut topo, a, true);
        let r = maximum(&topo, a, b).unwrap();
        separation(&r, (70.0_f64.sqrt() + 2.0) * scale, scale);
        point_close(
            r.point_a,
            Point3::new(6.0 * scale, -3.0 * scale, 5.0 * scale),
            scale,
        );
        let rev = maximum(&topo, b, a).unwrap();
        point_close(rev.point_b, r.point_a, scale);
        point_close(rev.point_a, r.point_b, scale);
    }
}

#[test]
fn maximum_affine_nurbs_requires_exact_carriers_and_complete_curve_use_proofs() {
    use remus_math::curves2d::{Curve2D, Line2D, NurbsCurve2D};
    use remus_math::nurbs::surface::NurbsSurface;
    use remus_math::vec::{Point2, Vec2};
    use remus_topology::{face::FaceSurface, pcurve::PCurve};
    for fault in 0..5 {
        let mut topo = Topology::new();
        let a = box_at(&mut topo, [2.0, 3.0, 4.0], [0.0; 3]);
        affine_faces(&mut topo, a, true);
        let f = remus_topology::explorer::solid_faces(&topo, a).unwrap()[0];
        let c = topo
            .face_loop(topo.loops_of_face(f).unwrap()[0])
            .unwrap()
            .coedges()[0];
        match fault {
            0 | 1 => {
                let FaceSurface::Nurbs(s) = topo.face(f).unwrap().surface() else {
                    unreachable!("fixture uses an affine NURBS carrier")
                };
                let mut cp = s.control_points().to_vec();
                let mut weights = s.weights().to_vec();
                if fault == 0 {
                    cp[1][1].0[2] += 1e-9;
                } else {
                    weights[1][1] = 6.0;
                }
                let replacement = NurbsSurface::new(
                    1,
                    1,
                    s.knots_u().to_vec(),
                    s.knots_v().to_vec(),
                    cp,
                    weights,
                )
                .unwrap();
                topo.face_mut(f)
                    .unwrap()
                    .set_surface(FaceSurface::Nurbs(replacement));
            }
            2 => {
                topo.remove_coedge_pcurve(c).unwrap();
            }
            3 => {
                topo.set_coedge_pcurve(
                    c,
                    PCurve::new(
                        Curve2D::Line(
                            Line2D::new(Point2::new(100.0, 100.0), Vec2::new(1.0, 0.0)).unwrap(),
                        ),
                        0.0,
                        1.0,
                    ),
                )
                .unwrap();
            }
            4 => {
                let pc = topo.coedge_pcurve(c).unwrap().unwrap();
                let a = pc.evaluate(pc.t_start());
                let b = pc.evaluate(pc.t_end());
                let midpoint =
                    Point2::new((a.x() + b.x()) * 0.5 + 0.1, (a.y() + b.y()) * 0.5 + 0.1);
                let curve = NurbsCurve2D::new(
                    2,
                    vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                    vec![a, midpoint, b],
                    vec![1.0; 3],
                )
                .unwrap();
                assert!((curve.evaluate(0.0) - a).length() <= 1e-14);
                assert!((curve.evaluate(1.0) - b).length() <= 1e-14);
                topo.set_coedge_pcurve(c, PCurve::new(Curve2D::Nurbs(curve), 0.0, 1.0))
                    .unwrap();
            }
            _ => unreachable!(),
        }
        let before = remus_io::arena_io::serialize_solid(&topo, a).unwrap();
        let tick = topo.mutation_ticks();
        assert!(
            matches!(
                maximum(&topo, a, a),
                Err(MaximumDistanceError::Unsupported(_))
            ),
            "fault {fault}"
        );
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, a).unwrap(),
            before
        );
        assert_eq!(topo.mutation_ticks(), tick);
    }
}

#[test]
fn maximum_affine_nurbs_unsafe_chart_roundoff_is_unresolved_and_read_only() {
    use remus_math::curves2d::{Curve2D, Line2D};
    use remus_math::nurbs::surface::NurbsSurface;
    use remus_math::vec::Point2;
    use remus_topology::{face::FaceSurface, pcurve::PCurve};

    let mut topo = Topology::new();
    let body = box_at(&mut topo, [2.0, 3.0, 4.0], [0.0; 3]);
    affine_faces(&mut topo, body, true);
    for face in remus_topology::explorer::solid_faces(&topo, body).unwrap() {
        let FaceSurface::Nurbs(source) = topo.face(face).unwrap().surface() else {
            unreachable!("fixture uses affine NURBS faces")
        };
        let surface = NurbsSurface::new(
            1,
            1,
            vec![1e12, 1e12, 1e12 + 6.0, 1e12 + 6.0],
            source.knots_v().to_vec(),
            source.control_points().to_vec(),
            source.weights().to_vec(),
        )
        .unwrap();
        assert!(remus_geometry::convert::certify_affine_nurbs_plane(&surface, 1e-7).is_some());
        topo.face_mut(face)
            .unwrap()
            .set_surface(FaceSurface::Nurbs(surface));
        let uses = topo
            .face_loop(topo.loops_of_face(face).unwrap()[0])
            .unwrap()
            .coedges()
            .to_vec();
        for coedge in uses {
            let pcurve = topo.coedge_pcurve(coedge).unwrap().unwrap();
            let Curve2D::Line(line) = pcurve.curve() else {
                unreachable!("fixture uses Line2D pcurves")
            };
            let shifted = PCurve::new(
                Curve2D::Line(
                    Line2D::new(
                        Point2::new(line.origin().x() + (1e12 - 2.0), line.origin().y()),
                        line.direction(),
                    )
                    .unwrap(),
                ),
                pcurve.t_start(),
                pcurve.t_end(),
            );
            topo.set_coedge_pcurve(coedge, shifted).unwrap();
            let usage = topo.coedge(coedge).unwrap();
            assert!(matches!(
                remus_topology::validation::check_same_parameter_strict(
                    &topo,
                    usage.edge(),
                    face,
                    usage.is_forward(),
                    0,
                ),
                Err(remus_topology::validation::CurveUseValidationError::SameParameterProofUnavailable { .. })
            ));
        }
    }
    let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
    let tick = topo.mutation_ticks();
    let error = maximum(&topo, body, body).unwrap_err();
    assert!(
        matches!(error, MaximumDistanceError::Unresolved(_)),
        "{error:?}"
    );
    assert_eq!(error.kernel_code(), "maximum-distance:unresolved");
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
        before
    );
    assert_eq!(topo.mutation_ticks(), tick);
}

#[test]
fn maximum_affine_nurbs_oblique_exact_nets_follow_scaled_orthogonal_metric() {
    for scale in [1.0 / 1024.0, 1.0, 1024.0] {
        let mut topo = Topology::new();
        let a = box_at(&mut topo, [2.0 * scale, 3.0 * scale, 4.0 * scale], [0.0; 3]);
        let b = box_at(
            &mut topo,
            [5.0 * scale, 2.0 * scale, scale],
            [8.0 * scale, -4.0 * scale, 2.0 * scale],
        );
        affine_faces(&mut topo, a, true);
        affine_faces(&mut topo, b, true);
        // Integer dyadic data stays exactly affine. Orthogonal columns have
        // squared lengths2,2,4, so the independent farthest-axis oracle is
        // 2*13^2 +2*7^2 +4*3^2 =472, even though no world axis is a box edge.
        let pose = Mat4([
            [1.0, 1.0, 0.0, 0.0],
            [1.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, -2.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        for solid in [a, b] {
            crate::transform::transform_solid(&mut topo, solid, &pose).unwrap();
        }
        let r = maximum(&topo, a, b).unwrap();
        separation(&r, 472.0_f64.sqrt() * scale, scale);
        point_close(
            r.point_a,
            pose.mul_point(Point3::new(0.0, 3.0 * scale, 0.0)),
            scale,
        );
        point_close(
            r.point_b,
            pose.mul_point(Point3::new(13.0 * scale, -4.0 * scale, 3.0 * scale)),
            scale,
        );
    }
}

#[test]
fn maximum_affine_nurbs_holes_and_cavity_shells_keep_full_authority() {
    for cavity in [false, true] {
        let mut topo = Topology::new();
        let outer = box_at(&mut topo, [8.0, 6.0, 4.0], [0.0; 3]);
        let tool = if cavity {
            box_at(&mut topo, [2.0, 2.0, 2.0], [3.0, 2.0, 1.0])
        } else {
            box_at(&mut topo, [2.0, 2.0, 6.0], [3.0, 2.0, -1.0])
        };
        let a = crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Cut, outer, tool)
            .unwrap();
        assert_eq!(
            topo.solid(a).unwrap().inner_shells().len(),
            usize::from(cavity)
        );
        affine_faces(&mut topo, a, true);
        assert!(
            crate::validate::validate_solid(&topo, a)
                .unwrap()
                .issues
                .iter()
                .all(|issue| issue.severity != crate::validate::Severity::Error)
        );
        let before = remus_io::arena_io::serialize_solid(&topo, a).unwrap();
        separation(&maximum(&topo, a, a).unwrap(), 116.0_f64.sqrt(), 1.0);
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, a).unwrap(),
            before
        );
        // Corrupt a use in a cavity when present: outer-hull dominance cannot
        // hide unsupported inner boundaries during preparation.
        let shell = if cavity {
            topo.solid(a).unwrap().inner_shells()[0]
        } else {
            topo.solid(a).unwrap().outer_shell()
        };
        let f = topo.shell(shell).unwrap().faces()[0];
        let coedge = topo
            .face_loop(topo.loops_of_face(f).unwrap()[0])
            .unwrap()
            .coedges()[0];
        topo.remove_coedge_pcurve(coedge).unwrap();
        assert!(matches!(
            maximum(&topo, a, a),
            Err(MaximumDistanceError::Unsupported(_))
        ));
    }
}
fn close(actual: f64, expected: f64, scale: f64) {
    assert!(
        (actual - expected).abs() <= 1e-10 * scale,
        "maximum-distance witness disagrees with the analytic oracle"
    );
}
fn point_close(actual: Point3, expected: Point3, scale: f64) {
    close((actual - expected).length(), 0.0, scale);
}
fn separation(result: &super::MaximumDistanceResult, expected: f64, scale: f64) {
    close(result.distance, expected, scale);
    close((result.point_b - result.point_a).length(), expected, scale);
}

#[test]
fn maximum_boxes_follow_farthest_component_oracle_at_scales_and_rigid_poses() {
    for scale in [1e-3, 1.0, 1e3] {
        for placed in [false, true] {
            let mut topo = Topology::new();
            let a = box_at(&mut topo, [2.0 * scale, 3.0 * scale, 4.0 * scale], [0.0; 3]);
            let b = box_at(
                &mut topo,
                [5.0 * scale, 2.0 * scale, scale],
                [8.0 * scale, -4.0 * scale, 2.0 * scale],
            );
            let pose = if placed {
                Mat4::translation(17.0 * scale, -11.0 * scale, 9.0 * scale)
                    * Mat4::rotation_z(0.37)
                    * Mat4::rotation_y(0.41)
            } else {
                Mat4::identity()
            };
            for body in [a, b] {
                crate::transform::transform_solid(&mut topo, body, &pose).unwrap();
            }
            let tick = topo.mutation_ticks();
            let journal = topo.journal().len();
            let result = maximum(&topo, a, b).unwrap();
            // Per-axis farthest separations are13,7,3, not a nearest-point oracle.
            separation(&result, 227.0_f64.sqrt() * scale, scale);
            point_close(
                result.point_a,
                pose.mul_point(Point3::new(0.0, 3.0 * scale, 0.0)),
                scale,
            );
            point_close(
                result.point_b,
                pose.mul_point(Point3::new(13.0 * scale, -4.0 * scale, 3.0 * scale)),
                scale,
            );
            let reverse = maximum(&topo, b, a).unwrap();
            point_close(reverse.point_a, result.point_b, scale);
            point_close(reverse.point_b, result.point_a, scale);
            separation(
                &maximum(&topo, a, a).unwrap(),
                29.0_f64.sqrt() * scale,
                scale,
            );
            assert_eq!(topo.mutation_ticks(), tick);
            assert_eq!(topo.journal().len(), journal);
        }
    }
}

#[test]
fn maximum_complete_spheres_cover_separation_overlap_containment_and_coincidence() {
    for scale in [1e-3, 1.0, 1e3] {
        for offset in [[8.0, 6.0, 0.0], [4.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0; 3]] {
            let mut topo = Topology::new();
            let a = sphere_at(
                &mut topo,
                2.0 * scale,
                [17.0 * scale, -11.0 * scale, 9.0 * scale],
            );
            let center = Point3::new(
                (17.0 + offset[0]) * scale,
                (-11.0 + offset[1]) * scale,
                (9.0 + offset[2]) * scale,
            );
            let b = sphere_at(&mut topo, 3.0 * scale, center.0);
            let result = maximum(&topo, a, b).unwrap();
            let gap = offset[0].hypot(offset[1]).hypot(offset[2]) * scale;
            separation(&result, gap + 5.0 * scale, scale);
            close(
                (result.point_a - Point3::new(17.0 * scale, -11.0 * scale, 9.0 * scale)).length(),
                2.0 * scale,
                scale,
            );
            close((result.point_b - center).length(), 3.0 * scale, scale);
            separation(&maximum(&topo, a, a).unwrap(), 4.0 * scale, scale);
        }
    }
}

#[test]
fn maximum_rigid_sphere_pose_keeps_equator_and_surface_frames_consistent() {
    for scale in [1e-3, 1.0, 1e3] {
        // Exact numeric matrix used by the public JavaScript pose witness.
        let pose = Mat4([
            [
                0.855_056_822_336_627_7,
                -0.361_615_431_964_962,
                0.371_634_376_693_522_2,
                17.0 * scale,
            ],
            [
                0.331_645_042_506_888_03,
                0.932_327_345_606_034_5,
                0.144_143_284_324_350_3,
                -11.0 * scale,
            ],
            [
                -0.398_609_327_984_422_9,
                0.0,
                0.917_120_822_816_605_1,
                9.0 * scale,
            ],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        for offset in [[8.0, 6.0, 0.0], [4.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0; 3]] {
            let mut topo = Topology::new();
            let a = sphere_at(&mut topo, 2.0 * scale, [0.0; 3]);
            let b = sphere_at(&mut topo, 3.0 * scale, offset.map(|x| x * scale));
            for solid in [a, b] {
                crate::transform::transform_solid(&mut topo, solid, &pose).unwrap();
            }
            if offset == [8.0, 6.0, 0.0] {
                let adjacency = remus_topology::adjacency::AdjacencyIndex::build(&topo, a).unwrap();
                assert!(adjacency.boundary_edges().is_empty());
                assert!(adjacency.non_manifold_edges().is_empty());
                let faces = remus_topology::explorer::solid_faces(&topo, a).unwrap();
                assert_eq!(faces.len(), 2);
                assert!(faces.iter().all(|&f| matches!(
                    topo.face(f).unwrap().surface(),
                    remus_topology::face::FaceSurface::Sphere(_)
                )));
                let mesh = crate::tessellate::tessellate_solid(&topo, a, 0.02 * scale).unwrap();
                assert_eq!(crate::tessellate::boundary_edge_count(&mesh), 0);
                assert_eq!(crate::tessellate::non_manifold_edge_count(&mesh), 0);
                let expected = (4.0 / 3.0) * std::f64::consts::PI * (2.0 * scale).powi(3);
                for deflection in [1e-5 * scale, 5e-6 * scale] {
                    let volume = crate::measure::solid_volume(&topo, a, deflection).unwrap();
                    assert!((volume - expected).abs() / expected <= 1e-9);
                }
                let options = remus_check::classify::ClassifyOptions::default();
                assert_eq!(
                    remus_check::classify::classify_point(
                        &topo,
                        a,
                        pose.mul_point(Point3::new(0.0, 0.0, 0.0)),
                        &options
                    )
                    .unwrap(),
                    remus_check::classify::PointClassification::Inside
                );
                assert_eq!(
                    remus_check::classify::classify_point(
                        &topo,
                        a,
                        pose.mul_point(Point3::new(4.0 * scale, 0.0, 0.0)),
                        &options
                    )
                    .unwrap(),
                    remus_check::classify::PointClassification::Outside
                );
            }
            separation(&maximum(&topo, a, a).unwrap(), 4.0 * scale, scale);
            separation(&maximum(&topo, b, b).unwrap(), 6.0 * scale, scale);
            separation(
                &maximum(&topo, a, b).unwrap(),
                (offset[0].hypot(offset[1]).hypot(offset[2]) + 5.0) * scale,
                scale,
            );
        }
    }
}

#[test]
fn maximum_mixed_sphere_box_has_analytic_corner_and_radial_witness() {
    for scale in [1e-3, 1.0, 1e3] {
        let mut topo = Topology::new();
        let sphere = sphere_at(&mut topo, 2.0 * scale, [0.0; 3]);
        let polygon = box_at(
            &mut topo,
            [2.0 * scale, 3.0 * scale, 4.0 * scale],
            [4.0 * scale, -3.0 * scale, scale],
        );
        let result = maximum(&topo, sphere, polygon).unwrap();
        separation(&result, (70.0_f64.sqrt() + 2.0) * scale, scale);
        point_close(
            result.point_b,
            Point3::new(6.0 * scale, -3.0 * scale, 5.0 * scale),
            scale,
        );
        close(
            (result.point_a - Point3::new(0.0, 0.0, 0.0)).length(),
            2.0 * scale,
            scale,
        );
        let reverse = maximum(&topo, polygon, sphere).unwrap();
        point_close(reverse.point_a, result.point_b, scale);
        point_close(reverse.point_b, result.point_a, scale);
    }
}

#[test]
fn maximum_respects_exact_comparison_budget_and_cancellation_without_mutation() {
    let mut topo = Topology::new();
    let a = box_at(&mut topo, [2.0, 3.0, 4.0], [0.0; 3]);
    let before = (topo.mutation_ticks(), topo.journal().len());
    let context = OperationContext::new().with_budgets(WorkBudgets::new().with_vertex_pairs(64));
    separation(
        &with_context(&topo, a, a, &context).unwrap(),
        29.0_f64.sqrt(),
        1.0,
    );
    let context = context.with_budgets(WorkBudgets::new().with_vertex_pairs(63));
    assert!(matches!(
        with_context(&topo, a, a, &context),
        Err(MaximumDistanceError::ResourceLimit {
            required: 64,
            limit: 63,
            ..
        })
    ));
    let token = CancellationToken::new();
    token.cancel();
    assert!(matches!(
        with_context(
            &topo,
            a,
            a,
            &OperationContext::new().with_cancellation(token)
        ),
        Err(MaximumDistanceError::Cancelled)
    ));
    assert_eq!((topo.mutation_ticks(), topo.journal().len()), before);
}

#[test]
fn maximum_checks_cavity_carriers_and_ignores_unrelated_arena_geometry() {
    let mut topo = Topology::new();
    let outer = box_at(&mut topo, [10.0; 3], [0.0; 3]);
    let inner = box_at(&mut topo, [2.0; 3], [4.0; 3]);
    let hollow = topo.add_solid(Solid::new(
        topo.solid(outer).unwrap().outer_shell(),
        vec![topo.solid(inner).unwrap().outer_shell()],
    ));
    let unrelated = box_at(&mut topo, [1.0; 3], [30.0; 3]);
    let vertex = remus_topology::explorer::solid_vertices(&topo, unrelated).unwrap()[0];
    *topo.vertex_mut(vertex).unwrap() = Vertex::new(Point3::new(f64::NAN, 0.0, 0.0), 1e-7);
    separation(
        &maximum(&topo, hollow, hollow).unwrap(),
        300.0_f64.sqrt(),
        1.0,
    );
    let cavity = sphere_at(&mut topo, 1.0, [5.0; 3]);
    let curved = topo.add_solid(Solid::new(
        topo.solid(outer).unwrap().outer_shell(),
        vec![topo.solid(cavity).unwrap().outer_shell()],
    ));
    assert!(matches!(
        maximum(&topo, curved, outer),
        Err(MaximumDistanceError::Unsupported(_))
    ));
    assert!(matches!(
        maximum(&topo, unrelated, outer),
        Err(MaximumDistanceError::InvalidGeometry(_))
    ));
}

#[test]
#[allow(deprecated)] // Deliberately poison the legacy wire view to test authority.
fn maximum_refuses_stale_authority_partial_spheres_and_nonfinite_planes() {
    let mut topo = Topology::new();
    let a = box_at(&mut topo, [2.0; 3], [0.0; 3]);
    let faces = remus_topology::explorer::solid_faces(&topo, a).unwrap();
    let other = topo.face(faces[1]).unwrap().outer_wire();
    topo.face_mut(faces[0]).unwrap().set_outer_wire(other);
    assert!(matches!(
        maximum(&topo, a, a),
        Err(MaximumDistanceError::Topology(_))
    ));
    let sphere = sphere_at(&mut topo, 2.0, [0.0; 3]);
    let face = remus_topology::explorer::solid_faces(&topo, sphere).unwrap()[0];
    let partial_shell = topo.add_shell(remus_topology::shell::Shell::new(vec![face]).unwrap());
    let partial = topo.add_solid(Solid::new(partial_shell, vec![]));
    assert!(matches!(
        maximum(&topo, partial, sphere),
        Err(MaximumDistanceError::Unsupported(_))
    ));
    let valid = box_at(&mut topo, [2.0; 3], [0.0; 3]);
    let face = remus_topology::explorer::solid_faces(&topo, valid).unwrap()[0];
    topo.face_mut(face)
        .unwrap()
        .set_surface(remus_topology::face::FaceSurface::Plane {
            normal: remus_math::vec::Vec3::new(f64::NAN, 0.0, 1.0),
            d: 0.0,
        });
    assert!(matches!(
        maximum(&topo, valid, valid),
        Err(MaximumDistanceError::InvalidGeometry(_))
    ));
}

#[test]
fn maximum_preparation_bounds_sphere_recognition_before_native_helpers() {
    let mut topo = Topology::new();
    let sphere = crate::primitives::make_sphere(&mut topo, 2.0, 4097).unwrap();
    assert!(matches!(
        maximum(&topo, sphere, sphere),
        Err(MaximumDistanceError::ResourceLimit {
            resource: "face boundary uses",
            required: 4097,
            limit: 4096
        })
    ));
}

#[test]
fn maximum_rejects_poisoned_sphere_data_and_unresolved_translated_carriers() {
    use remus_math::surfaces::SphericalSurface;
    use remus_topology::face::FaceSurface;
    for (center, radius) in [
        (Point3::new(f64::NAN, 0.0, 0.0), 2.0),
        (Point3::new(0.0, 0.0, 0.0), f64::NAN),
        (Point3::new(0.0, 0.0, 0.0), f64::INFINITY),
    ] {
        let mut topo = Topology::new();
        let sphere = sphere_at(&mut topo, 2.0, [0.0; 3]);
        for face in remus_topology::explorer::solid_faces(&topo, sphere).unwrap() {
            topo.face_mut(face)
                .unwrap()
                .set_surface(FaceSurface::Sphere(
                    SphericalSurface::new(center, radius).unwrap(),
                ));
        }
        assert!(matches!(
            maximum(&topo, sphere, sphere),
            Err(MaximumDistanceError::InvalidGeometry(_))
        ));
    }
    // Build exact YZ equators without asking transform_solid to normalize a
    // translated polar-axis point that is itself unrepresentable. These valid
    // rings retain radius1; only the required X-directed maximum witnesses
    // collapse at the large translated centers.
    let shifted_sphere = |topo: &mut Topology, center: Point3| {
        use remus_math::{curves::Circle3D, vec::Vec3};
        use remus_topology::edge::EdgeCurve;
        let body = crate::primitives::make_sphere(topo, 1.0, 16).unwrap();
        for vertex in remus_topology::explorer::solid_vertices(topo, body).unwrap() {
            let old = topo.vertex(vertex).unwrap().point();
            *topo.vertex_mut(vertex).unwrap() =
                Vertex::new(Point3::new(center.x(), old.y(), -old.x()), 1e-7);
        }
        let equator = Circle3D::new_with_ref(
            center,
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
            Vec3::new(0.0, 0.0, -1.0),
        )
        .unwrap();
        for edge_id in remus_topology::explorer::solid_edges(topo, body).unwrap() {
            let edge = topo.edge_mut(edge_id).unwrap();
            let trim = edge.trim();
            edge.set_curve(EdgeCurve::Circle(equator.clone()));
            edge.set_trim(trim);
        }
        for face in remus_topology::explorer::solid_faces(topo, body).unwrap() {
            topo.face_mut(face)
                .unwrap()
                .set_surface(FaceSurface::Sphere(
                    SphericalSurface::with_frame(
                        center,
                        1.0,
                        Vec3::new(1.0, 0.0, 0.0),
                        Vec3::new(0.0, 0.0, -1.0),
                    )
                    .unwrap(),
                ));
        }
        body
    };
    let mut topo = Topology::new();
    let a = shifted_sphere(&mut topo, Point3::new(1e16, 0.0, 0.0));
    let b = shifted_sphere(&mut topo, Point3::new(1e16 + 8.0, 0.0, 0.0));
    separation(&maximum(&topo, a, a).unwrap(), 2.0, 1.0);
    assert!(matches!(
        maximum(&topo, a, b),
        Err(MaximumDistanceError::Unresolved(_))
    ));
}

#[test]
fn maximum_mixed_and_sphere_comparison_budgets_charge_actual_candidates() {
    let mut topo = Topology::new();
    let sphere = sphere_at(&mut topo, 2.0, [0.0; 3]);
    let polygon = box_at(&mut topo, [2.0, 3.0, 4.0], [4.0, -3.0, 1.0]);
    let context = OperationContext::new().with_budgets(WorkBudgets::new().with_vertex_pairs(8));
    assert!(with_context(&topo, sphere, polygon, &context).is_ok());
    let context = context.with_budgets(WorkBudgets::new().with_vertex_pairs(7));
    assert!(matches!(
        with_context(&topo, sphere, polygon, &context),
        Err(MaximumDistanceError::ResourceLimit {
            required: 8,
            limit: 7,
            ..
        })
    ));
    let context = context.with_budgets(WorkBudgets::new().with_vertex_pairs(1));
    assert!(with_context(&topo, sphere, sphere, &context).is_ok());
    let context = context.with_budgets(WorkBudgets::new().with_vertex_pairs(0));
    assert!(matches!(
        with_context(&topo, sphere, sphere, &context),
        Err(MaximumDistanceError::ResourceLimit {
            required: 1,
            limit: 0,
            ..
        })
    ));
    for tolerance in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let mut context = OperationContext::new();
        context.tolerance.linear = tolerance;
        assert!(matches!(
            with_context(&topo, sphere, sphere, &context),
            Err(MaximumDistanceError::InvalidGeometry(_))
        ));
    }
}

#[test]
fn maximum_valid_through_hole_keeps_outer_corner_diameter() {
    for scale in [1e-3, 1.0, 1e3] {
        let mut topo = Topology::new();
        let outer = box_at(&mut topo, [8.0 * scale, 6.0 * scale, 4.0 * scale], [0.0; 3]);
        let tool = box_at(
            &mut topo,
            [2.0 * scale, 2.0 * scale, 6.0 * scale],
            [3.0 * scale, 2.0 * scale, -scale],
        );
        let hollow =
            crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Cut, outer, tool)
                .unwrap();
        assert!(
            crate::validate::validate_solid(&topo, hollow)
                .unwrap()
                .is_valid()
        );
        assert!(
            remus_topology::explorer::solid_faces(&topo, hollow)
                .unwrap()
                .into_iter()
                .any(|f| !topo.face(f).unwrap().inner_wires().is_empty())
        );
        let before = (topo.mutation_ticks(), topo.journal().len());
        separation(
            &maximum(&topo, hollow, hollow).unwrap(),
            116.0_f64.sqrt() * scale,
            scale,
        );
        assert_eq!((topo.mutation_ticks(), topo.journal().len()), before);
    }
}
