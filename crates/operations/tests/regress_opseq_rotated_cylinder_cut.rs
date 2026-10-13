//! OpSeq F3: a tilted finite cylinder cutter must retain all stock material
//! outside the cutter, including the connected crescent beside its wall.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::prepared::PreparedSolid;
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

#[test]
#[allow(clippy::too_many_lines)]
fn rotated_cylinder_cut_retains_material_and_closed_boundary() {
    for scale in [1.0, 1e3] {
        for placed in [false, true] {
            let mut topo = Topology::new();
            let tool = make_cylinder(&mut topo, 5.5 * scale, 1.5 * scale).unwrap();
            let stock = make_cylinder(&mut topo, 2.0 * scale, 3.5 * scale).unwrap();
            let placement = if placed {
                Mat4::translation(10.0 * scale, -7.0 * scale, 3.0 * scale)
                    * Mat4::rotation_x(0.37)
                    * Mat4::rotation_z(0.23)
            } else {
                Mat4::identity()
            };
            let tool_transform = placement
                * Mat4::translation(4.0 * scale, -2.0 * scale, -0.5 * scale)
                * Mat4::rotation_x(std::f64::consts::PI / 6.0);
            transform_solid(&mut topo, tool, &tool_transform).unwrap();
            transform_solid(&mut topo, stock, &placement).unwrap();
            let before = remus_io::arena_io::serialize_solids(&topo, &[stock, tool]).unwrap();
            let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
            let result = boolean_with_context(&mut topo, BooleanOp::Cut, stock, tool, &context)
                .unwrap_or_else(|error| panic!("scale={scale}, placed={placed}: {error:?}"));
            assert!(matches!(result.quality, BooleanQuality::Exact));
            let options = ClassifyOptions::default();
            // These memberships follow directly from the two cylinders' equations.
            let kept = placement.mul_point(Point3::new(-1.5 * scale, 0.0, scale));
            assert_eq!(
                classify_point(&topo, stock, kept, &options).unwrap(),
                PointClassification::Inside
            );
            assert_eq!(
                classify_point(&topo, tool, kept, &options).unwrap(),
                PointClassification::Outside
            );
            assert_eq!(
                classify_point(&topo, result.solid, kept, &options).unwrap(),
                PointClassification::Inside
            );
            let removed = placement.mul_point(Point3::new(0.5 * scale, -0.5 * scale, 1.5 * scale));
            assert_eq!(
                classify_point(&topo, stock, removed, &options).unwrap(),
                PointClassification::Inside
            );
            assert_eq!(
                classify_point(&topo, tool, removed, &options).unwrap(),
                PointClassification::Inside
            );
            assert_eq!(
                classify_point(&topo, result.solid, removed, &options).unwrap(),
                PointClassification::Outside
            );

            // An independent grid also crosses the stock's original parameter seam.
            // A correct boundary integral alone misses a folded UV outline that loses
            // some points and adds others on either side of that seam.
            let prepared = PreparedSolid::prepare(&topo, result.solid).unwrap();
            for x in [-1.5_f64, -0.5, 0.5, 1.5] {
                for y in [-1.5_f64, -0.5, 0.5, 1.5] {
                    if x * x + y * y >= 4.0 {
                        continue;
                    }
                    for z in [0.25_f64, 0.75, 1.25, 1.75, 2.25, 2.75, 3.25] {
                        let tool_y = (y + 2.0) * 3.0_f64.sqrt() / 2.0 + f64::midpoint(z, 0.5);
                        let tool_z = -(y + 2.0) / 2.0 + (z + 0.5) * 3.0_f64.sqrt() / 2.0;
                        let radial_squared = (x - 4.0).powi(2) + tool_y.powi(2);
                        if (radial_squared - 30.25).abs() < 0.02
                            || tool_z.abs() < 0.02
                            || (tool_z - 1.5).abs() < 0.02
                        {
                            continue;
                        }
                        let expected = if radial_squared < 30.25 && (0.0..1.5).contains(&tool_z) {
                            PointClassification::Outside
                        } else {
                            PointClassification::Inside
                        };
                        let point =
                            placement.mul_point(Point3::new(x * scale, y * scale, z * scale));
                        assert_eq!(
                            prepared.classify_point(point, &options).unwrap(),
                            expected,
                            "scale={scale}, placed={placed}, local=({x}, {y}, {z})"
                        );
                    }
                }
            }

            assert!(
                remus_operations::validate::validate_solid(&topo, result.solid)
                    .unwrap()
                    .is_valid()
            );
            let report = remus_check::validate::validate_solid(
                &topo,
                result.solid,
                &remus_check::validate::ValidateOptions::default(),
            )
            .unwrap();
            assert!(
                !report
                    .issues
                    .iter()
                    .any(|issue| issue.severity == remus_check::validate::Severity::Error),
                "scale={scale}, placed={placed}: {:?}",
                report.issues
            );
            let mut uses = std::collections::BTreeMap::new();
            for f in remus_topology::explorer::solid_faces(&topo, result.solid).unwrap() {
                let face = topo.face(f).unwrap();
                for w in
                    std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
                {
                    for edge in topo.wire(w).unwrap().edges() {
                        uses.entry(edge.edge())
                            .or_insert_with(Vec::new)
                            .push(edge.is_forward() != face.is_reversed());
                    }
                }
            }
            for directions in uses.values() {
                assert_eq!(directions.len(), 2, "open/non-manifold edge");
                assert_ne!(directions[0], directions[1], "same-sense shared edge");
            }
            // Independent reference: integrate the exact affine z-interval overlap
            // at each (x,y) in the stock disc. Refining the one-dimensional outer
            // quadrature from 1e-9 to 1e-11 changes this value by less than 6e-12.
            // See scripts/opseq-f3-volume-oracle.py for the complete derivation.
            let expected = 30.240_105_421_226_68 * scale.powi(3);
            for volume in [
                solid_volume(&topo, result.solid, 0.0001 * scale).unwrap(),
                mass_properties(&topo, result.solid).unwrap().mass,
            ] {
                assert!(
                    (volume - expected).abs() < 1e-4 * scale.powi(3),
                    "volume {volume} vs independent {expected}"
                );
            }
            for deflection in [0.1, 0.01] {
                let mesh = tessellate_solid(&topo, result.solid, deflection * scale).unwrap();
                assert!(!mesh.indices.is_empty());
                assert_eq!(boundary_edge_count(&mesh), 0);
                assert_eq!(non_manifold_edge_count(&mesh), 0);
            }
            assert_eq!(
                remus_io::arena_io::serialize_solids(&topo, &[stock, tool]).unwrap(),
                before
            );
        }
    }
}
