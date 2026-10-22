//! F9/F14/F15: signed offset rims must preserve the material side and apex.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::f64::consts::PI;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_io::arena_io::serialize_solid;
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::measure::solid_volume;
use remus_operations::offset_v2::offset_solid_v2;
use remus_operations::primitives::{make_cone, make_cylinder};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid_grouped_with_tolerance,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;

/// Intersect the independently offset profile line with the displaced caps.
/// A pointed cone has no cap at the apex; its tip follows the offset line.
fn profile(r0: f64, r1: f64, h: f64, d: f64) -> (f64, f64, f64, f64) {
    let slope = (r1 - r0) / h;
    let radial_shift = d * slope.hypot(1.0);
    let mut lo = -d;
    let mut hi = h + d;
    if r0 == 0.0 {
        lo = -radial_shift / slope;
    }
    if r1 == 0.0 {
        hi = h - radial_shift / slope;
    }
    let radius = |z: f64| r0 + slope * z + radial_shift;
    (lo, hi, radius(lo).max(0.0), radius(hi).max(0.0))
}

#[test]
#[allow(clippy::too_many_lines)]
fn offset_revolution_walls_have_closed_outward_meshes_and_correct_material() {
    // Literal F15/F14/F9, the apex-up twin, and a taller neighboring cylinder.
    for (cylinder, r0, r1, h) in [
        (true, 1.0, 1.0, 1.0),
        (false, 4.0, 2.0, 3.0),
        (false, 0.0, 1.0, 1.0),
        (false, 1.0, 0.0, 1.0),
        (true, 0.7, 0.7, 3.0),
    ] {
        for d in [-0.25, 0.25] {
            let (lo, hi, ra, rb) = profile(r0, r1, h, d);
            let volume = PI * (hi - lo) * (ra * ra + ra * rb + rb * rb) / 3.0;
            for scale in [1e-3, 1.0, 1e3] {
                for placed in [false, true] {
                    let label =
                        format!("radii={r0}/{r1} h={h} d={d} scale={scale} placed={placed}");
                    let matrix = if placed {
                        Mat4::translation(11.0 * scale, -7.0 * scale, 5.0 * scale)
                            * Mat4::rotation_z(0.7)
                            * Mat4::rotation_x(0.3)
                    } else {
                        Mat4::identity()
                    };
                    let inverse = matrix.inverse().unwrap();
                    let mut topo = Topology::new();
                    let source = if cylinder {
                        make_cylinder(&mut topo, r0 * scale, h * scale).unwrap()
                    } else {
                        make_cone(&mut topo, r0 * scale, r1 * scale, h * scale).unwrap()
                    };
                    transform_solid(&mut topo, source, &matrix).unwrap();
                    let original = serialize_solid(&topo, source).unwrap();
                    let result = offset_solid_v2(&mut topo, source, d * scale)
                        .unwrap_or_else(|error| panic!("{label}: {error}"));
                    assert_eq!(
                        serialize_solid(&topo, source).unwrap(),
                        original,
                        "{label}: mutated source"
                    );
                    let report = remus_operations::validate::validate_solid(&topo, result).unwrap();
                    assert!(report.is_valid(), "{label}: {:?}", report.issues);
                    let faces = solid_faces(&topo, result).unwrap();
                    assert_eq!(
                        faces.len(),
                        if r0 == 0.0 || r1 == 0.0 { 2 } else { 3 },
                        "{label}"
                    );
                    let mut uses = BTreeMap::new();
                    for &fid in &faces {
                        let face = topo.face(fid).unwrap();
                        assert!(!face.is_reversed(), "{label}: inverted face");
                        assert!(
                            matches!(
                                face.surface(),
                                FaceSurface::Plane { .. }
                                    | FaceSurface::Cone(_)
                                    | FaceSurface::Cylinder(_)
                            ),
                            "{label}: faceted result"
                        );
                        for oe in topo.wire(face.outer_wire()).unwrap().edges() {
                            uses.entry(oe.edge())
                                .or_insert_with(Vec::new)
                                .push(oe.is_forward());
                        }
                    }
                    for directions in uses.values() {
                        assert_eq!(directions.len(), 2, "{label}: B-Rep boundary");
                        assert_ne!(directions[0], directions[1], "{label}: same-sense B-Rep");
                    }
                    for deflection in [1e-2, 1e-3] {
                        let (mesh, groups) = tessellate_solid_grouped_with_tolerance(
                            &topo,
                            result,
                            deflection * scale,
                            0.35,
                        )
                        .unwrap();
                        assert_eq!(boundary_edge_count(&mesh), 0, "{label}: raw mesh boundary");
                        assert_eq!(
                            non_manifold_edge_count(&mesh),
                            0,
                            "{label}: non-manifold mesh"
                        );
                        let local: Vec<_> = mesh
                            .positions
                            .iter()
                            .map(|p| {
                                let q = inverse.mul_point(*p);
                                Vec3::new(q.x() / scale, q.y() / scale, q.z() / scale)
                            })
                            .collect();
                        let origin = local[0];
                        let mut signed_volume = 0.0;
                        for (index, pair) in groups.windows(2).enumerate() {
                            assert!(pair[1] > pair[0], "{label}: empty face");
                            let planar = matches!(
                                topo.face(faces[index]).unwrap().surface(),
                                FaceSurface::Plane { .. }
                            );
                            for tri in
                                mesh.indices[pair[0] as usize..pair[1] as usize].chunks_exact(3)
                            {
                                let [a, b, c] = [0, 1, 2].map(|i| local[tri[i] as usize]);
                                let center = (a + b + c) * (1.0 / 3.0);
                                let outward = if planar {
                                    Vec3::new(
                                        0.0,
                                        0.0,
                                        if center.z() < f64::midpoint(lo, hi) {
                                            -1.0
                                        } else {
                                            1.0
                                        },
                                    )
                                } else {
                                    let radius = center.x().hypot(center.y());
                                    Vec3::new(
                                        center.x() / radius,
                                        center.y() / radius,
                                        -(r1 - r0) / h,
                                    )
                                };
                                assert!(
                                    (b - a).cross(c - a).dot(outward) > 0.0,
                                    "{label}: inward triangle"
                                );
                                signed_volume +=
                                    (a - origin).dot((b - origin).cross(c - origin)) / 6.0;
                            }
                        }
                        assert!(
                            (signed_volume - volume).abs() < 0.04 * volume,
                            "{label}: mesh volume {signed_volume} != {volume}"
                        );
                        let measured = solid_volume(&topo, result, deflection * scale).unwrap()
                            / scale.powi(3);
                        assert!(
                            (measured - volume).abs() < 1e-6 * volume,
                            "{label}: volume {measured} != {volume}"
                        );
                    }
                    let options = ClassifyOptions {
                        tolerance: 1e-7 * scale,
                        ..ClassifyOptions::default()
                    };
                    for fraction in [0.2, 0.5, 0.8] {
                        let z = lo + fraction * (hi - lo);
                        let radius = ra + fraction * (rb - ra);
                        for factor in [0.25, 1.3] {
                            let point = matrix.mul_point(Point3::new(
                                factor * radius * scale,
                                0.13 * radius * scale,
                                z * scale,
                            ));
                            let expected = if factor < 1.0 {
                                PointClassification::Inside
                            } else {
                                PointClassification::Outside
                            };
                            assert_eq!(
                                classify_point(&topo, result, point, &options).unwrap(),
                                expected,
                                "{label}: radial probe {fraction}/{factor}"
                            );
                        }
                    }
                    for z in [lo - 0.1 * (hi - lo), hi + 0.1 * (hi - lo)] {
                        let point =
                            matrix.mul_point(Point3::new(0.01 * scale, 0.02 * scale, z * scale));
                        assert_eq!(
                            classify_point(&topo, result, point, &options).unwrap(),
                            PointClassification::Outside,
                            "{label}: axial probe"
                        );
                    }
                }
            }
        }
    }
}
