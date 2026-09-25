//! Exact ruling cuts preserve genuinely nonplanar bilinear NURBS caps.
//!
//! At scale 1e-3 translated by (13,-7,5), the legacy `solid_volume` path
//! suffers origin cancellation: 6.0000096757e-8 versus 6e-8. Gauss and the
//! independently recentered closed mesh agree within 4e-12 relative. The
//! geometry matrix retains that placement; the separate kernel-volume test
//! qualifies scale-relative translations without claiming that limitation fixed.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_geometry::convert::recognize_surface::{RecognizedSurface, recognize_surface};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_topology::{
    Topology,
    builder::make_polygon_wire,
    face::{Face, FaceSurface},
    solid::SolidId,
};

fn build_pair(scale: f64, along_x: bool, offset: Vec3) -> (Topology, SolidId, SolidId) {
    let mut topo = Topology::new();
    let corners = [(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)]
        .map(|(x, y)| Point3::new(x * scale, y * scale, 0.1 * x * y * scale));
    let carrier = remus_operations::nonplanar_ring_surface(&corners).unwrap();
    assert!(matches!(
        recognize_surface(&carrier, 1e-7),
        RecognizedSurface::NotRecognized
    ));
    let points = carrier.control_points();
    assert!(((points[1][1] - points[1][0]) - (points[0][1] - points[0][0])).length() > scale);
    let wire = make_polygon_wire(&mut topo, &corners, 1e-7).unwrap();
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(carrier)));
    let path = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 6.0 * scale),
        ],
        vec![1.0, 1.0],
    )
    .unwrap();
    let stock = remus_operations::sweep::sweep(&mut topo, face, &path).unwrap();
    let (width, depth, x, y) = if along_x {
        (4.0, 6.0, 0.5, -3.0)
    } else {
        (6.0, 4.0, -3.0, 0.5)
    };
    let cutter = remus_operations::primitives::make_box(
        &mut topo,
        width * scale,
        depth * scale,
        9.0 * scale,
    )
    .unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        cutter,
        &Mat4::translation(x * scale, y * scale, -scale),
    )
    .unwrap();
    let motion = Mat4::translation(offset.x(), offset.y(), offset.z());
    for solid in [stock, cutter] {
        remus_operations::transform::transform_solid(&mut topo, solid, &motion).unwrap();
    }
    (topo, stock, cutter)
}

fn assert_valid(topo: &Topology, solid: SolidId, label: &str) {
    let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(
        report.is_valid(),
        "{label}: operations validation {report:?}"
    );
    let report = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    let errors: Vec<_> = report
        .issues
        .iter()
        .filter(|issue| issue.severity == remus_check::validate::Severity::Error)
        .collect();
    assert!(errors.is_empty(), "{label}: check validation {errors:?}");
}

fn assert_result(
    topo: &Topology,
    solid: SolidId,
    scale: f64,
    along_x: bool,
    motion: Mat4,
    op: BooleanOp,
    label: &str,
) -> f64 {
    assert_valid(topo, solid, label);
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    assert!(faces.len() <= 20, "{label}: {} faces", faces.len());
    let mut caps = 0;
    let mut edges = std::collections::BTreeMap::new();
    for fid in faces {
        let face = topo.face(fid).unwrap();
        if let FaceSurface::Nurbs(surface) = face.surface() {
            assert!(matches!(
                recognize_surface(surface, 1e-7),
                RecognizedSurface::NotRecognized
            ));
            caps += 1;
        } else {
            assert!(matches!(face.surface(), FaceSurface::Plane { .. }));
        }
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for edge in topo.wire(wid).unwrap().edges() {
                *edges.entry(edge.edge()).or_insert(0) += 1;
            }
        }
    }
    assert_eq!(caps, 2, "{label}: retained NURBS caps");
    assert!(
        edges.values().all(|count| *count == 2),
        "{label}: edge uses {edges:?}"
    );
    for deflection in [0.1, 0.01, 1e-4] {
        let mesh = remus_operations::tessellate::tessellate_solid(topo, solid, deflection * scale)
            .unwrap();
        assert_eq!(
            remus_operations::tessellate::boundary_edge_count(&mesh),
            0,
            "{label}: mesh boundary at {deflection}"
        );
        assert_eq!(
            remus_operations::tessellate::non_manifold_edge_count(&mesh),
            0,
            "{label}: mesh nonmanifold at {deflection}"
        );
        let expected = if op == BooleanOp::Cut { 60.0 } else { 36.0 } * scale.powi(3);
        let origin = mesh.positions[0];
        let mesh_volume = mesh
            .indices
            .chunks_exact(3)
            .map(|triangle| {
                let a = mesh.positions[triangle[0] as usize] - origin;
                let b = mesh.positions[triangle[1] as usize] - origin;
                let c = mesh.positions[triangle[2] as usize] - origin;
                a.dot(b.cross(c)) / 6.0
            })
            .sum::<f64>()
            .abs();
        assert!(
            (mesh_volume - expected).abs() / expected <= 1e-6,
            "{label}: recentered mesh {mesh_volume}, expected {expected}"
        );
    }
    for (x, y, z, stock_side) in [
        (-1.0, 0.3, 3.0, true),
        (1.5, 0.3, 3.0, false),
        (-1.0, 0.3, -1.0, true),
        (-1.0, 0.3, 7.0, true),
    ] {
        let (x, y) = if along_x { (x, y) } else { (y, x) };
        let inside = (0.0..6.0).contains(&z)
            && match op {
                BooleanOp::Cut => stock_side,
                BooleanOp::Intersect => !stock_side,
                BooleanOp::Fuse => unreachable!(),
            };
        let point = motion.mul_point(Point3::new(x * scale, y * scale, z * scale));
        let expected = if inside {
            PointClassification::Inside
        } else {
            PointClassification::Outside
        };
        assert_eq!(
            classify_point(topo, solid, point, &ClassifyOptions::default()).unwrap(),
            expected,
            "{label}: material at {point:?}"
        );
    }
    let expected = if op == BooleanOp::Cut { 60.0 } else { 36.0 } * scale.powi(3);
    let volume = remus_operations::measure::mass_properties(topo, solid)
        .unwrap()
        .mass;
    assert!(
        (volume - expected).abs() / expected <= 1e-6,
        "{label}: Gauss {volume}, expected {expected}"
    );
    volume
}

#[test]
fn bilinear_saddle_cap_cut_along_ruling() {
    for scale in [1e-3, 1.0, 1e3] {
        for along_x in [true, false] {
            let mut reference: Option<f64> = None;
            for offset in [Vec3::new(0.0, 0.0, 0.0), Vec3::new(13.0, -7.0, 5.0)] {
                let mut volumes = Vec::new();
                for op in [BooleanOp::Cut, BooleanOp::Intersect] {
                    let label = format!("{op:?} scale={scale} along_x={along_x} offset={offset:?}");
                    let (mut topo, stock, cutter) = build_pair(scale, along_x, offset);
                    assert_valid(&topo, stock, &label);
                    let outcome = boolean_with_context(
                        &mut topo,
                        op,
                        stock,
                        cutter,
                        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
                    )
                    .unwrap_or_else(|error| panic!("{label}: {error:?}"));
                    assert!(matches!(outcome.quality, BooleanQuality::Exact));
                    volumes.push(assert_result(
                        &topo,
                        outcome.solid,
                        scale,
                        along_x,
                        Mat4::translation(offset.x(), offset.y(), offset.z()),
                        op,
                        &label,
                    ));
                }
                let whole = 96.0 * scale.powi(3);
                assert!(
                    (volumes.iter().sum::<f64>() - whole).abs() / whole <= 1e-6,
                    "cut complement {volumes:?}"
                );
                if let Some(previous) = reference {
                    assert!(
                        (volumes[0] - previous).abs() / whole <= 1e-6,
                        "translation changed volume"
                    );
                }
                reference = Some(volumes[0]);
            }
        }
    }
}

#[test]
fn bilinear_ruling_kernel_volume_with_scale_relative_translations() {
    for scale in [1e-3, 1.0, 1e3] {
        for offset in [Vec3::new(0.0, 0.0, 0.0), Vec3::new(13.0, -7.0, 5.0) * scale] {
            for op in [BooleanOp::Cut, BooleanOp::Intersect] {
                let (mut topo, stock, cutter) = build_pair(scale, true, offset);
                let result = boolean_with_context(
                    &mut topo,
                    op,
                    stock,
                    cutter,
                    &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
                )
                .unwrap();
                assert!(matches!(result.quality, BooleanQuality::Exact));
                let expected = if op == BooleanOp::Cut { 60.0 } else { 36.0 } * scale.powi(3);
                for deflection in [2e-4, 1e-4] {
                    let measured = remus_operations::measure::solid_volume(
                        &topo,
                        result.solid,
                        deflection * scale,
                    )
                    .unwrap();
                    assert!(
                        (measured - expected).abs() / expected <= 1e-6,
                        "{op:?} scale={scale} offset={offset:?}: measured {measured}, expected {expected}"
                    );
                }
            }
        }
    }
}

#[test]
fn bilinear_ruling_cut_and_intersection_survive_rigid_rotation() {
    let rotation = Mat4::rotation_y(0.37) * Mat4::rotation_z(0.21);
    for op in [BooleanOp::Cut, BooleanOp::Intersect] {
        let (mut topo, stock, cutter) = build_pair(1.0, true, Vec3::new(0.0, 0.0, 0.0));
        for solid in [stock, cutter] {
            remus_operations::transform::transform_solid(&mut topo, solid, &rotation).unwrap();
        }
        assert_valid(&topo, stock, "rotated input");
        let result = boolean_with_context(
            &mut topo,
            op,
            stock,
            cutter,
            &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
        )
        .unwrap();
        assert!(matches!(result.quality, BooleanQuality::Exact));
        assert_result(&topo, result.solid, 1.0, true, rotation, op, "rotated");
    }
}
