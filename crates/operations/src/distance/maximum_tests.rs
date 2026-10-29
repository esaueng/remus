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
fn close(actual: f64, expected: f64, scale: f64) {
    assert!(
        (actual - expected).abs() <= 1e-10 * scale,
        "{actual} != {expected}"
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
