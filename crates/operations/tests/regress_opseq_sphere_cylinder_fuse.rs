//! F10: a cylinder standing on a sphere's equator retains the lower hemisphere.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::f64::consts::PI;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::context::{CancellationToken, FallbackPolicy, OperationContext, WorkBudgets};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::{mass_properties, solid_surface_area, solid_volume};
use remus_operations::primitives::{make_cylinder, make_sphere};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid_grouped_with_tolerance,
    tessellate_solid_with_tolerance,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;

fn topology_counts(t: &Topology) -> [usize; 8] {
    [
        t.num_vertices(),
        t.num_edges(),
        t.num_wires(),
        t.num_faces(),
        t.num_shells(),
        t.num_solids(),
        t.num_loops(),
        t.num_coedges(),
    ]
}

#[allow(clippy::too_many_lines, clippy::float_cmp)]
fn check_fixture(
    scale: f64,
    placed: bool,
    segments: usize,
    height: f64,
    swapped: bool,
    fine_mesh: bool,
) {
    let label =
        format!("scale={scale} placed={placed} segments={segments} h={height} swap={swapped}");
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, scale, segments).unwrap();
    let cylinder = make_cylinder(&mut topo, scale, height * scale).unwrap();
    let placement = if placed {
        Mat4::translation(10.0 * scale, -7.0 * scale, 3.0 * scale)
            * Mat4::rotation_x(0.37)
            * Mat4::rotation_z(0.23)
    } else {
        Mat4::identity()
    };
    if placed {
        for solid in [sphere, cylinder] {
            transform_solid(&mut topo, solid, &placement).unwrap();
        }
    }
    let original = remus_io::arena_io::serialize_solids(&topo, &[sphere, cylinder]).unwrap();
    let before_counts = topology_counts(&topo);
    let (lhs, rhs) = if swapped {
        (sphere, cylinder)
    } else {
        (cylinder, sphere)
    };
    let result = boolean_with_context(
        &mut topo,
        BooleanOp::Fuse,
        lhs,
        rhs,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    );
    // Exactly these two placed large-scale cells already refuse on immutable
    // base 0551: incomplete section loops produce a rejected two-face result.
    if scale == 1000.0 && placed {
        let error = result.unwrap_err();
        assert!(
            matches!(
                error,
                remus_operations::OperationsError::ExactOnlyUnattainable
            ),
            "{label}: {error:?}"
        );
        assert_eq!(
            topology_counts(&topo),
            before_counts,
            "{label}: refusal rollback"
        );
        assert_eq!(
            remus_io::arena_io::serialize_solids(&topo, &[sphere, cylinder]).unwrap(),
            original
        );
        return;
    }
    let result = result.unwrap_or_else(|e| panic!("{label}: {e:?}"));
    assert!(matches!(result.quality, BooleanQuality::Exact), "{label}");
    let solid = result.solid;
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[sphere, cylinder]).unwrap(),
        original
    );
    let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
    assert_eq!(faces.len(), 3, "{label}: hemisphere, cylinder, top disc");
    let mut counts = [0; 3];
    let mut uses = BTreeMap::new();
    for &fid in &faces {
        let face = topo.face(fid).unwrap();
        match face.surface() {
            FaceSurface::Plane { .. } => counts[0] += 1,
            FaceSurface::Cylinder(_) => counts[1] += 1,
            FaceSurface::Sphere(_) => counts[2] += 1,
            other => panic!("{label}: unexpected {} carrier", other.type_tag()),
        }
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for edge in topo.wire(wid).unwrap().edges() {
                uses.entry(edge.edge())
                    .or_insert_with(Vec::new)
                    .push(edge.is_forward() != face.is_reversed());
            }
        }
    }
    assert_eq!(counts, [1, 1, 1], "{label}");
    for (edge, directions) in uses {
        assert_eq!(directions.len(), 2, "{label}: incidence {edge:?}");
        assert_ne!(directions[0], directions[1], "{label}: same-sense {edge:?}");
    }
    let validation = remus_operations::validate::validate_solid(&topo, solid).unwrap();
    assert!(validation.is_valid(), "{label}: {:?}", validation.issues);
    let validation = remus_check::validate::validate_solid(
        &topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(
        !validation
            .issues
            .iter()
            .any(|i| i.severity == remus_check::validate::Severity::Error),
        "{label}: {:?}",
        validation.issues
    );

    let inverse = placement.inverse().unwrap();
    let deflections: &[f64] = if fine_mesh {
        &[0.00002, 0.00001]
    } else {
        &[0.1, 0.01]
    };
    for &deflection in deflections {
        if fine_mesh {
            // The ungrouped route does not retain face attribution. A correct
            // coincidence decision must not depend on that optional metadata.
            let plain =
                tessellate_solid_with_tolerance(&topo, solid, deflection * scale, 0.35).unwrap();
            assert_eq!(boundary_edge_count(&plain), 0, "{label}: plain fine mesh");
            assert_eq!(
                non_manifold_edge_count(&plain),
                0,
                "{label}: plain incidence"
            );
        }
        let (mesh, offsets) =
            tessellate_solid_grouped_with_tolerance(&topo, solid, deflection * scale, 0.35)
                .unwrap();
        assert_eq!(
            boundary_edge_count(&mesh),
            0,
            "{label}: raw mesh boundary at {deflection}"
        );
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "{label}: raw mesh non-manifold at {deflection}"
        );
        let mut mesh_uses = BTreeMap::new();
        for (index, &fid) in faces.iter().enumerate() {
            assert!(offsets[index + 1] > offsets[index], "{label}: empty face");
            let spherical = matches!(topo.face(fid).unwrap().surface(), FaceSurface::Sphere(_));
            for tri in
                mesh.indices[offsets[index] as usize..offsets[index + 1] as usize].chunks_exact(3)
            {
                let [a, b, c] = [tri[0], tri[1], tri[2]].map(|i| {
                    (inverse.mul_point(mesh.positions[i as usize]) - Point3::new(0.0, 0.0, 0.0))
                        * scale.recip()
                });
                let normal = (b - a).cross(c - a);
                assert!(normal.length() > 1e-15, "{label}: degenerate triangle");
                if spherical {
                    assert!(
                        normal.dot((a + b + c) * (1.0 / 3.0)) > 0.0,
                        "{label}: inward sphere triangle"
                    );
                    for p in [a, b, c] {
                        assert!(p.z() <= 1e-6, "{label}: upper-hemisphere exterior triangle");
                        assert!(
                            (p.length() - 1.0).abs() < 1e-5,
                            "{label}: off-sphere vertex"
                        );
                    }
                }
                for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                    mesh_uses
                        .entry((a.min(b), a.max(b)))
                        .or_insert_with(Vec::new)
                        .push(a < b);
                }
            }
        }
        for (edge, directions) in mesh_uses {
            assert_eq!(directions.len(), 2, "{label}: raw mesh incidence {edge:?}");
            assert_ne!(
                directions[0], directions[1],
                "{label}: raw mesh winding {edge:?}"
            );
        }
    }

    // Cartesian sphere/cylinder inequalities, independently of face sampling.
    let options = ClassifyOptions::default();
    let mut coverage = [0_usize; 4];
    for x in [-1.2_f64, -0.8, -0.4, 0.2, 0.6, 1.2] {
        for y in [-0.8_f64, -0.3, 0.2, 0.7] {
            for z in [-1.2_f64, -0.8, -0.3, 0.3, 0.7, 1.2, 1.7] {
                let radial = x.hypot(y);
                let radius = radial.hypot(z);
                if (radius - 1.0).abs() < 0.06
                    || (radial - 1.0).abs() < 0.06
                    || (z - height).abs() < 0.06
                {
                    continue;
                }
                let ball = radius < 1.0;
                let tube = radial < 1.0 && z > 0.0 && z < height;
                coverage[2 * usize::from(ball) + usize::from(tube)] += 1;
                let point = placement.mul_point(Point3::new(x * scale, y * scale, z * scale));
                for (name, id, inside) in [
                    ("sphere", sphere, ball),
                    ("cylinder", cylinder, tube),
                    ("union", solid, ball || tube),
                ] {
                    assert_eq!(
                        classify_point(&topo, id, point, &options).unwrap(),
                        if inside {
                            PointClassification::Inside
                        } else {
                            PointClassification::Outside
                        },
                        "{label}: {name} at {x},{y},{z}"
                    );
                }
            }
        }
    }
    assert!(
        coverage.into_iter().all(|n| n > 0),
        "{label}: all membership cells"
    );
    let expected_volume = PI * (height + 2.0 / 3.0);
    let expected_area = PI * (2.0 * height + 3.0);
    // The 32-segment fixture's fragmented cylinder rims integrate over their
    // exact band rectangle instead of the 4096-point trim budget.
    let gauss = mass_properties(&topo, solid).unwrap().mass / scale.powi(3);
    assert!(
        (gauss - expected_volume).abs() < 1e-9 * expected_volume,
        "{label}: Gauss {gauss}"
    );
    for d in [0.0001, 0.00005] {
        let volume = solid_volume(&topo, solid, d * scale).unwrap() / scale.powi(3);
        assert!(
            (volume - expected_volume).abs() < 1e-4 * expected_volume,
            "{label}: volume {volume}"
        );
        let area = solid_surface_area(&topo, solid, d * scale).unwrap() / scale.powi(2);
        assert!(
            (area - expected_area).abs() < 1e-4 * expected_area,
            "{label}: area {area}"
        );
    }
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[sphere, cylinder]).unwrap(),
        original
    );
}

#[test]
fn hemisphere_union_retains_material_orientation_and_quantities() {
    for scale in [1.0, 0.001, 1000.0] {
        for placed in [false, true] {
            for swapped in [false, true] {
                check_fixture(scale, placed, 16, 1.0, swapped, false);
            }
        }
    }
    for segments in [4, 8, 32] {
        check_fixture(1.0, false, segments, 1.0, false, false);
    }
    check_fixture(1.0, false, 16, 1.25, false, false);
}

/// Separate retained counterexample: the pre-dedup mesh is closed, but four
/// position-quantized candidate groups cancel eight distinct polar triangles.
/// All six vertex identities in each pair differ. A shared-edge exclusion does
/// not apply; changing the near-coincidence policy needs its own qualification.
#[test]
fn hemisphere_union_tiny_placed_fine_mesh_polar_dedup() {
    check_fixture(0.001, true, 16, 1.0, false, true);
}

#[test]
fn hemisphere_union_refusals_preserve_operands_and_topology() {
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, 1.0, 16).unwrap();
    let cylinder = make_cylinder(&mut topo, 1.0, 1.0).unwrap();
    let before = remus_io::arena_io::serialize_solids(&topo, &[sphere, cylinder]).unwrap();
    let before_counts = topology_counts(&topo);
    let token = CancellationToken::new();
    token.cancel();
    let contexts = [
        (OperationContext::new().with_cancellation(token), true),
        (
            OperationContext::new().with_budgets(WorkBudgets::new().with_vertex_pairs(0)),
            false,
        ),
    ];
    for (context, cancelled) in contexts {
        let error = boolean_with_context(
            &mut topo,
            BooleanOp::Fuse,
            cylinder,
            sphere,
            &context.with_fallback(FallbackPolicy::ExactOnly),
        )
        .unwrap_err();
        if cancelled {
            assert!(
                matches!(
                    error,
                    remus_operations::OperationsError::Math(remus_math::MathError::Cancelled)
                ),
                "{error:?}"
            );
        } else {
            assert!(
                matches!(
                    error,
                    remus_operations::OperationsError::Algo(
                        remus_algo::error::AlgoError::ResourceLimitExceeded {
                            resource: "GFA vertex pairs",
                            limit: 0,
                            actual: _
                        }
                    )
                ),
                "{error:?}"
            );
        }
        assert_eq!(
            topology_counts(&topo),
            before_counts,
            "failed operation must roll back allocations"
        );
        assert_eq!(
            remus_io::arena_io::serialize_solids(&topo, &[sphere, cylinder]).unwrap(),
            before,
            "failed operation must preserve original geometry and boundaries"
        );
    }
}
