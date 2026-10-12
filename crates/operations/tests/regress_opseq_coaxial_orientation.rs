//! Operation-sequence F1/F2: shared closed rims must keep their traversal
//! when an unsplit cap is rebuilt with a CommonBlock edge.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::f64::consts::PI;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::make_cylinder;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;

#[test]
#[allow(clippy::too_many_lines)]
fn coaxial_boolean_orientation_in_both_orders_and_placements() {
    // Minimized witness and both original campaign seeds: wide/short A,
    // narrow/tall B. Their overlap is a cylinder of B's radius and A's height.
    for (ra, ha, rb, hb) in [
        (1.5, 1.0, 1.0, 1.5),
        (2.5, 4.0, 2.0, 5.0),
        (5.0, 2.5, 2.0, 5.0),
    ] {
        for scale in [1.0, 1e3] {
            for placed in [false, true] {
                let placement = if placed {
                    Mat4::translation(10.0 * scale, -7.0 * scale, 3.0 * scale)
                        * Mat4::rotation_x(0.37)
                        * Mat4::rotation_z(0.23)
                } else {
                    Mat4::identity()
                };
                for swapped in [false, true] {
                    for op in [BooleanOp::Intersect, BooleanOp::Fuse, BooleanOp::Cut] {
                        let label = format!(
                            "{op:?}, swapped={swapped}, scale={scale}, placed={placed}, radii={ra}/{rb}"
                        );
                        let mut topo = Topology::new();
                        let a = make_cylinder(&mut topo, ra * scale, ha * scale).unwrap();
                        let b = make_cylinder(&mut topo, rb * scale, hb * scale).unwrap();
                        if placed {
                            transform_solid(&mut topo, a, &placement).unwrap();
                            transform_solid(&mut topo, b, &placement).unwrap();
                        }
                        let (lhs, rhs) = if swapped { (b, a) } else { (a, b) };
                        let context =
                            OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
                        let result = boolean_with_context(&mut topo, op, lhs, rhs, &context)
                            .unwrap_or_else(|e| panic!("{label}: {e:?}"));
                        assert!(matches!(result.quality, BooleanQuality::Exact), "{label}");
                        let solid = result.solid;
                        let report =
                            remus_operations::validate::validate_solid(&topo, solid).unwrap();
                        assert!(report.is_valid(), "{label}: {:?}", report.issues);
                        let report = remus_check::validate::validate_solid(
                            &topo,
                            solid,
                            &remus_check::validate::ValidateOptions::default(),
                        )
                        .unwrap();
                        assert!(
                            !report
                                .issues
                                .iter()
                                .any(|i| i.severity == remus_check::validate::Severity::Error),
                            "{label}: {:?}",
                            report.issues
                        );

                        let faces = solid_faces(&topo, solid).unwrap();
                        assert!(faces.len() <= 7, "{label}: faceted fallback");
                        let mut uses = BTreeMap::new();
                        let mut curved = false;
                        for f in faces {
                            let face = topo.face(f).unwrap();
                            assert!(
                                matches!(
                                    face.surface(),
                                    FaceSurface::Plane { .. } | FaceSurface::Cylinder(_)
                                ),
                                "{label}"
                            );
                            curved |= matches!(face.surface(), FaceSurface::Cylinder(_));
                            for w in std::iter::once(face.outer_wire())
                                .chain(face.inner_wires().iter().copied())
                            {
                                for oe in topo.wire(w).unwrap().edges() {
                                    uses.entry(oe.edge())
                                        .or_insert_with(Vec::new)
                                        .push(oe.is_forward() != face.is_reversed());
                                }
                            }
                        }
                        assert!(curved, "{label}: analytic cylinder required");
                        for directions in uses.values() {
                            assert_eq!(directions.len(), 2, "{label}: open or non-manifold edge");
                            assert_ne!(
                                directions[0], directions[1],
                                "{label}: same-sense shared edge"
                            );
                        }

                        let va = PI * ra * ra * ha;
                        let vb = PI * rb * rb * hb;
                        let overlap = PI * rb * rb * ha;
                        let expected = match op {
                            BooleanOp::Intersect => overlap,
                            BooleanOp::Fuse => va + vb - overlap,
                            BooleanOp::Cut => (if swapped { vb } else { va }) - overlap,
                        } * scale.powi(3);
                        let gauss = mass_properties(&topo, solid).unwrap().mass;
                        let measured = solid_volume(&topo, solid, 0.01 * scale).unwrap();
                        for volume in [gauss, measured] {
                            assert!(
                                (volume - expected).abs() <= 1e-9 * expected,
                                "{label}: volume={volume}, expected={expected}"
                            );
                        }
                        for deflection in [0.1, 0.01] {
                            let mesh = tessellate_solid(&topo, solid, deflection * scale).unwrap();
                            assert!(!mesh.indices.is_empty(), "{label}: empty mesh");
                            assert_eq!(boundary_edge_count(&mesh), 0, "{label}: open mesh");
                            assert_eq!(
                                non_manifold_edge_count(&mesh),
                                0,
                                "{label}: non-manifold mesh"
                            );
                        }
                        // Common material, A-only annulus, B-only extension,
                        // and exterior: evaluate set membership independently.
                        for (p, in_a, in_b) in [
                            ([0.25 * rb, 0.1 * rb, 0.5 * ha], true, true),
                            ([0.5 * (ra + rb), 0.0, 0.5 * ha], true, false),
                            ([0.25 * rb, 0.1 * rb, 0.5 * (ha + hb)], false, true),
                            ([1.5 * ra, 0.0, 0.5 * ha], false, false),
                        ] {
                            let (l, r) = if swapped { (in_b, in_a) } else { (in_a, in_b) };
                            let inside = match op {
                                BooleanOp::Intersect => l && r,
                                BooleanOp::Fuse => l || r,
                                BooleanOp::Cut => l && !r,
                            };
                            let point = placement.mul_point(Point3::new(
                                p[0] * scale,
                                p[1] * scale,
                                p[2] * scale,
                            ));
                            let class =
                                classify_point(&topo, solid, point, &ClassifyOptions::default())
                                    .unwrap();
                            assert_eq!(
                                class,
                                if inside {
                                    PointClassification::Inside
                                } else {
                                    PointClassification::Outside
                                },
                                "{label}: probe {p:?}"
                            );
                        }
                    }
                }
            }
        }
    }
}
