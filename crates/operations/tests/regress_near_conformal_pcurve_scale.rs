//! A retained planar circle and its pcurve must use the same radius even
//! inside the transform's analytic roundoff band.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI};

use remus_math::curves::Circle3D;
use remus_math::curves2d::{Circle2D, Curve2D};
use remus_math::frame::Frame3;
use remus_math::mat::Mat4;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_operations::copy::copy_and_transform_solid;
use remus_operations::transform::{is_similarity, transform_solid};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceSurface};
use remus_topology::pcurve::PCurve;
use remus_topology::shell::Shell;
use remus_topology::solid::Solid;
use remus_topology::validation::{CurveUseValidationError, check_same_parameter_strict};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

#[test]
#[allow(clippy::too_many_lines)]
fn near_conformal_planar_circle_uses_its_retained_edge_radius() {
    let tolerance = Tolerance::new().linear;
    let stretch = 1.0 + 3e-15;
    for (z_scale, globally_similar) in [(1.0, true), (2.0, false)] {
        for radius in [1e9, 1.0] {
            for quarter_turn in [false, true] {
                for forward in [false, true] {
                    for reflect in [false, true] {
                        let mut source = Topology::new();
                        let normal = Vec3::new(0.0, 0.0, 1.0);
                        let frame =
                            Frame3::from_normal(Point3::new(0.0, 0.0, 0.0), normal).unwrap();
                        let (u, v, phase, edge_scale) = if quarter_turn {
                            (frame.y, -frame.x, FRAC_PI_2, 1.0)
                        } else {
                            (frame.x, frame.y, 0.0, stretch)
                        };
                        let circle =
                            Circle3D::with_axes(Point3::new(0.0, 0.0, 0.0), normal, radius, u, v)
                                .unwrap();
                        let vertex =
                            source.add_vertex(Vertex::new(circle.evaluate(0.0), tolerance));
                        let end_vertex =
                            source.add_vertex(Vertex::new(circle.evaluate(PI), tolerance));
                        let mut edge = Edge::new(vertex, end_vertex, EdgeCurve::Circle(circle));
                        edge.set_trim(Some((0.0, PI)));
                        let edge = source.add_edge(edge);
                        let diameter =
                            source.add_edge(Edge::new(end_vertex, vertex, EdgeCurve::Line));
                        let wire = source.add_wire(
                            Wire::new(
                                vec![
                                    OrientedEdge::new(edge, forward),
                                    OrientedEdge::new(diameter, forward),
                                ],
                                true,
                            )
                            .unwrap(),
                        );
                        let face = source.add_face(Face::new(
                            wire,
                            vec![],
                            FaceSurface::Plane { normal, d: 0.0 },
                        ));
                        let (start, end) = if forward {
                            (phase, phase + PI)
                        } else {
                            (phase + PI, phase)
                        };
                        source
                            .set_pcurve_oriented(
                                edge,
                                face,
                                forward,
                                PCurve::new(
                                    Curve2D::Circle(
                                        Circle2D::new(Point2::new(0.0, 0.0), radius).unwrap(),
                                    ),
                                    start,
                                    end,
                                ),
                            )
                            .unwrap();
                        if radius > 1.0 {
                            let proof =
                                check_same_parameter_strict(&source, edge, face, forward, 16);
                            assert!(
                                matches!(
                                    proof,
                                    Err(
                                        CurveUseValidationError::SameParameterProofUnavailable { .. }
                                    )
                                ),
                                "source proof: {proof:?}"
                            );
                        }
                        let source_face = face;
                        let source_edge = edge;
                        let shell = source.add_shell(Shell::new(vec![face]).unwrap());
                        // Only the planar chart is relevant; this sheet fixture
                        // exercises the solid transform and copy traversal paths.
                        let solid = source.add_solid(Solid::new(shell, vec![]));
                        let x_scale = if reflect { -1.0 } else { 1.0 };
                        let matrix = Mat4::scale(x_scale, stretch, z_scale);
                        assert!(
                            is_similarity(&matrix) == globally_similar,
                            "out-of-plane scaling must exercise the locally retained-circle path"
                        );
                        for copy in [false, true] {
                            let mut topo = source.clone();
                            let result = if copy {
                                copy_and_transform_solid(&mut topo, solid, &matrix).unwrap()
                            } else {
                                transform_solid(&mut topo, solid, &matrix).unwrap();
                                solid
                            };
                            let face = solid_faces(&topo, result).unwrap()[0];
                            let loop_id = topo.face(face).unwrap().boundary_loops()[0];
                            let coedge = topo.face_loop(loop_id).unwrap().coedges()[0];
                            let data = topo.coedge(coedge).unwrap();
                            let EdgeCurve::Circle(edge) = topo.edge(data.edge()).unwrap().curve()
                            else {
                                panic!("near-conformal transform must keep the 3D circle");
                            };
                            assert_eq!(edge.radius().to_bits(), (radius * edge_scale).to_bits());
                            let pcurve =
                                data.pcurve().expect("planar circular pcurve must survive");
                            let Curve2D::Circle(circle) = pcurve.curve() else {
                                panic!("planar circular pcurve must retain its family");
                            };
                            let discrepancy = (circle.radius() - edge.radius()).abs();
                            assert!(
                                discrepancy <= tolerance,
                                "copy={copy}, quarter_turn={quarter_turn}, forward={forward}, reflect={reflect}: radius mismatch {discrepancy:e}"
                            );
                            assert_eq!(circle.radius().to_bits(), edge.radius().to_bits());
                            let strict = check_same_parameter_strict(
                                &topo,
                                data.edge(),
                                face,
                                data.is_forward(),
                                16,
                            );
                            if radius > 1.0 {
                                // The strict proof's existing arithmetic limit at
                                // this scale still refuses; this fix cannot waive it.
                                assert!(matches!(
                                    strict,
                                    Err(
                                        CurveUseValidationError::SameParameterProofUnavailable { .. }
                                    )
                                ));
                            } else {
                                assert!(strict.unwrap().unwrap().max_deviation <= tolerance);
                            }
                            if copy {
                                assert_eq!(topo.num_pcurves(), 2);
                                let original = topo
                                    .pcurve_oriented(source_edge, source_face, forward)
                                    .unwrap();
                                let Curve2D::Circle(original_circle) = original.curve() else {
                                    panic!("source circle changed");
                                };
                                assert_eq!(original_circle.radius().to_bits(), radius.to_bits());
                                assert_eq!(original.t_start().to_bits(), start.to_bits());
                            }
                        }
                    }
                }
            }
        }
    }
}
