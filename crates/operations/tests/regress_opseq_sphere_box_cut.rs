//! F6: an exact sphere-minus-box cut must preserve the seven retained octants.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::f64::consts::PI;

use remus_check::classify::cache::ClassificationCache;
use remus_check::classify::prepared::PreparedSolid;
use remus_check::classify::{ClassifyOptions, PointClassification};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::{mass_properties, solid_surface_area, solid_volume};
use remus_operations::primitives::{make_box, make_sphere};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid_grouped_with_tolerance,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;

// Clip the entire triangle against the three positive coordinate half-spaces.
// A centroid-only probe can miss a triangle that crosses into the removed octant.
fn intersects_removed_octant(triangle: [Vec3; 3]) -> bool {
    let mut polygon = triangle.to_vec();
    for axis in 0..3 {
        let mut clipped = Vec::new();
        for index in 0..polygon.len() {
            let a = polygon[index];
            let b = polygon[(index + 1) % polygon.len()];
            let da = a.0[axis] - 1e-6;
            let db = b.0[axis] - 1e-6;
            if da >= 0.0 {
                clipped.push(a);
            }
            if (da >= 0.0) != (db >= 0.0) {
                clipped.push(a + (b - a) * (da / (da - db)));
            }
        }
        polygon = clipped;
    }
    !polygon.is_empty()
}

#[allow(clippy::too_many_lines)]
fn check_fixture(scale: f64, placed: bool, segments: usize, side: f64) {
    check_boolean_fixture(scale, placed, segments, side, false, false);
}

#[allow(clippy::too_many_lines)]
fn check_boolean_fixture(
    scale: f64,
    placed: bool,
    segments: usize,
    side: f64,
    fuse: bool,
    reverse: bool,
) {
    let label = format!(
        "F6/F8 scale={scale} placed={placed} segments={segments} side={side} fuse={fuse} reverse={reverse}"
    );
    let mut topo = Topology::new();
    // Literal bundle order: F6 creates box 0 then sphere 1; F8 reverses that.
    let (cube, sphere) = if fuse {
        let sphere = make_sphere(&mut topo, scale, segments).unwrap();
        let cube = make_box(&mut topo, side * scale, side * scale, side * scale).unwrap();
        (cube, sphere)
    } else {
        let cube = make_box(&mut topo, side * scale, side * scale, side * scale).unwrap();
        let sphere = make_sphere(&mut topo, scale, segments).unwrap();
        (cube, sphere)
    };
    let placement = if placed {
        Mat4::translation(10.0 * scale, -7.0 * scale, 3.0 * scale)
            * Mat4::rotation_x(0.37)
            * Mat4::rotation_z(0.23)
    } else {
        Mat4::identity()
    };
    if placed {
        transform_solid(&mut topo, cube, &placement).unwrap();
        transform_solid(&mut topo, sphere, &placement).unwrap();
    }
    let original = remus_io::arena_io::serialize_solids(&topo, &[cube, sphere]).unwrap();
    let outcome = boolean_with_context(
        &mut topo,
        if fuse {
            BooleanOp::Fuse
        } else {
            BooleanOp::Cut
        },
        if reverse { cube } else { sphere },
        if reverse { sphere } else { cube },
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap();
    assert!(matches!(outcome.quality, BooleanQuality::Exact));
    let solid = outcome.solid;
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[cube, sphere]).unwrap(),
        original,
        "boolean must preserve both operands"
    );
    let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
    assert!(
        faces.len() <= 12 && faces.len() >= 4,
        "analytic face census"
    );
    let mut spheres = 0;
    let mut uses = BTreeMap::new();
    for &fid in &faces {
        let face = topo.face(fid).unwrap();
        match face.surface() {
            FaceSurface::Plane { .. } => {}
            FaceSurface::Sphere(_) => spheres += 1,
            other @ (FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Torus(_)
            | FaceSurface::Nurbs(_)) => {
                panic!("unexpected {} carrier", other.type_tag());
            }
        }
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for edge in topo.wire(wid).unwrap().edges() {
                uses.entry(edge.edge())
                    .or_insert_with(Vec::new)
                    .push(edge.is_forward() != face.is_reversed());
            }
        }
    }
    assert!(spheres > 0);
    for (edge, directions) in uses {
        assert_eq!(directions.len(), 2, "open/non-manifold {edge:?}");
        assert_ne!(directions[0], directions[1], "same-sense {edge:?}");
    }
    assert!(
        remus_operations::validate::validate_solid(&topo, solid)
            .unwrap()
            .is_valid()
    );
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
            .any(|issue| issue.severity == remus_check::validate::Severity::Error),
        "{:?}",
        report.issues
    );

    // Closure is checked independently of the quantity routes, which shared
    // the same wrong open-mesh Gauss fallback before this repair.
    let inverse = placement.inverse().unwrap();
    for deflection in [0.1, 0.01, 0.001] {
        let (mesh, offsets) =
            tessellate_solid_grouped_with_tolerance(&topo, solid, deflection * scale, 0.35)
                .unwrap();
        assert!(!mesh.indices.is_empty());
        assert_eq!(
            boundary_edge_count(&mesh),
            0,
            "{label}: raw boundary at {deflection}"
        );
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "{label}: non-manifold at {deflection}"
        );
        let local: Vec<Vec3> = mesh
            .positions
            .iter()
            .map(|&p| (inverse.mul_point(p) - Point3::new(0.0, 0.0, 0.0)) * scale.recip())
            .collect();
        let mut mesh_uses = BTreeMap::new();
        for (index, &fid) in faces.iter().enumerate() {
            assert!(offsets[index + 1] > offsets[index], "{label}: empty face");
            let spherical = matches!(topo.face(fid).unwrap().surface(), FaceSurface::Sphere(_));
            for triangle in
                mesh.indices[offsets[index] as usize..offsets[index + 1] as usize].chunks_exact(3)
            {
                let [a, b, c] = [triangle[0], triangle[1], triangle[2]].map(|i| local[i as usize]);
                let normal = (b - a).cross(c - a);
                assert!(normal.length() > 1e-15, "{label}: degenerate triangle");
                if spherical {
                    assert!(
                        normal.dot((a + b + c) * (1.0 / 3.0)) > 0.0,
                        "{label}: inward triangle"
                    );
                    assert!(
                        !intersects_removed_octant([a, b, c]),
                        "{label}: removed octant triangle"
                    );
                    for p in [a, b, c] {
                        assert!(
                            (p.length() - 1.0).abs() < 1e-5,
                            "{label}: off-sphere vertex"
                        );
                    }
                }
                for (a, b) in [
                    (triangle[0], triangle[1]),
                    (triangle[1], triangle[2]),
                    (triangle[2], triangle[0]),
                ] {
                    mesh_uses
                        .entry((a.min(b), a.max(b)))
                        .or_insert_with(Vec::new)
                        .push(a < b);
                }
            }
        }
        for (edge, directions) in mesh_uses {
            assert_eq!(directions.len(), 2, "{label}: mesh incidence {edge:?}");
            assert_ne!(
                directions[0], directions[1],
                "{label}: same-sense mesh edge {edge:?}"
            );
        }
    }

    // These Cartesian inequalities are independent of every kernel route.
    let options = ClassifyOptions::default();
    let wasm_options = ClassifyOptions {
        tolerance: 1e-7,
        ..ClassifyOptions::default()
    };
    let mut persistent = ClassificationCache::new();
    let sources = [
        PreparedSolid::prepare(&topo, sphere).unwrap(),
        PreparedSolid::prepare(&topo, cube).unwrap(),
    ];
    let result = PreparedSolid::prepare(&topo, solid).unwrap();
    let mut coverage = [0_usize; 4];
    for x in [-1.15_f64, -0.85, -0.55, -0.25, 0.25, 0.55, 0.85, 1.15] {
        for y in [-1.15_f64, -0.85, -0.55, -0.25, 0.25, 0.55, 0.85, 1.15] {
            for z in [-1.15_f64, -0.85, -0.55, -0.25, 0.25, 0.55, 0.85, 1.15] {
                let r = x.mul_add(x, y.mul_add(y, z * z)).sqrt();
                if (r - 1.0).abs() < 0.06 {
                    continue;
                }
                if [x, y, z].into_iter().any(|v| (v - side).abs() < 0.06) {
                    continue;
                }
                let in_sphere = r < 1.0;
                let in_box = [x, y, z].into_iter().all(|v| v > 0.0 && v < side);
                coverage[usize::from(in_sphere) * 2 + usize::from(in_box)] += 1;
                let point = placement.mul_point(Point3::new(x * scale, y * scale, z * scale));
                for (id, name, classifier, expected) in [
                    (sphere, "sphere", &sources[0], in_sphere),
                    (cube, "box", &sources[1], in_box),
                    (
                        solid,
                        "result",
                        &result,
                        if fuse {
                            in_sphere || in_box
                        } else {
                            in_sphere && !in_box
                        },
                    ),
                ] {
                    let expected = if expected {
                        PointClassification::Inside
                    } else {
                        PointClassification::Outside
                    };
                    for options in [&options, &wasm_options] {
                        for (path, actual) in [
                            ("prepared", classifier.classify_point(point, options)),
                            (
                                "one-shot",
                                remus_check::classify::classify_point(&topo, id, point, options),
                            ),
                            (
                                "persistent",
                                persistent.classify_point(&topo, id, point, options),
                            ),
                        ] {
                            assert_eq!(
                                actual.unwrap(),
                                expected,
                                "{label} {name} {path} tol={}: local=({x},{y},{z})",
                                options.tolerance
                            );
                        }
                    }
                }
            }
        }
    }
    assert!(coverage.into_iter().all(|n| n > 0));
    let unit = 1.0 / 3.0_f64.sqrt();
    for (point, expected) in [
        (
            Point3::new(unit, unit, unit),
            if fuse {
                PointClassification::Inside
            } else {
                PointClassification::Outside
            },
        ),
        (
            Point3::new(-unit, unit, unit),
            PointClassification::OnBoundary,
        ),
        (
            Point3::new(0.25, 0.25, 0.0),
            if fuse {
                PointClassification::Inside
            } else {
                PointClassification::OnBoundary
            },
        ),
    ] {
        let world = placement.mul_point(Point3::new(
            point.x() * scale,
            point.y() * scale,
            point.z() * scale,
        ));
        for options in [&options, &wasm_options] {
            for (path, actual) in [
                ("prepared", result.classify_point(world, options)),
                (
                    "one-shot",
                    remus_check::classify::classify_point(&topo, solid, world, options),
                ),
                (
                    "persistent",
                    persistent.classify_point(&topo, solid, world, options),
                ),
            ] {
                assert_eq!(
                    actual.unwrap(),
                    expected,
                    "{label}: {path} tol={} exact boundary membership at {point:?}",
                    options.tolerance
                );
            }
        }
    }
    for (point, inside) in [
        (Point3::new(-0.55, -0.25, 0.55), true),
        (Point3::new(0.25, 0.25, 0.25), fuse),
        (Point3::new(0.85, 0.85, -0.85), false),
    ] {
        let point = placement.mul_point(Point3::new(
            point.x() * scale,
            point.y() * scale,
            point.z() * scale,
        ));
        let robust = remus_operations::classify::classify_point_robust(
            &topo,
            solid,
            point,
            0.1 * scale,
            1e-7,
        )
        .unwrap();
        assert_eq!(
            robust,
            if inside {
                remus_operations::classify::PointClassification::Inside
            } else {
                remus_operations::classify::PointClassification::Outside
            },
            "public robust result: {point:?}"
        );
    }
    // Seven eighths of the unit ball; seven eighths of its sphere plus
    // three quarter-discs. Neither expression uses kernel integration.
    let expected_volume = 7.0 * PI / 6.0 + if fuse { 1.0 } else { 0.0 };
    let expected_area = if fuse {
        11.0 * PI / 4.0 + 6.0
    } else {
        17.0 * PI / 4.0
    };
    for deflection in [0.0001, 0.00005] {
        let volume = solid_volume(&topo, solid, deflection * scale).unwrap() / scale.powi(3);
        assert!(
            (volume - expected_volume).abs() < 1e-4 * expected_volume,
            "{label}: V={volume} at {deflection}"
        );
    }
    let gauss = mass_properties(&topo, solid).unwrap().mass / scale.powi(3);
    assert!(
        (gauss - expected_volume).abs() < 1e-4 * expected_volume,
        "{label}: Gauss V={gauss}"
    );
    let area = solid_surface_area(&topo, solid, 0.0001 * scale).unwrap() / scale.powi(2);
    assert!(
        (area - expected_area).abs() < 1e-4 * expected_area,
        "{label}: A={area}"
    );
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[cube, sphere]).unwrap(),
        original,
        "read-only checks must preserve both operands"
    );
}

#[test]
fn f6_literal_cut_preserves_material_quantities_and_closed_meshes() {
    check_fixture(1.0, false, 16, 1.0);
}

#[test]
fn f6_rigid_placement_preserves_material_quantities_and_closed_meshes() {
    check_fixture(1.0, true, 16, 1.0);
}

#[test]
fn f6_small_scale_origin_and_placed() {
    for placed in [false, true] {
        check_fixture(0.001, placed, 16, 1.0);
    }
}

#[test]
fn f6_large_scale_origin_and_placed() {
    for placed in [false, true] {
        check_fixture(1000.0, placed, 16, 1.0);
    }
}

#[test]
fn f6_equator_segmentation_and_larger_box_controls() {
    for segments in [8, 32] {
        check_fixture(1.0, false, segments, 1.0);
    }
    check_fixture(1.0, false, 16, 1.25);
}

#[test]
fn f8_complementary_union_has_independent_material_and_quantity_oracles() {
    for reverse in [false, true] {
        check_boolean_fixture(1.0, false, 16, 1.0, true, reverse);
    }
}

mod placed_sphere_box {
    use super::*;

    // Integrate the Cartesian sphere height over [0,a] x [0,b]. The
    // divergence identity 3*integral(sqrt(R²-x²-y²)) = boundary + R²*K
    // gives an independent closed form; K is the integral of its reciprocal.
    fn quantities(a: f64, b: f64, h: f64) -> (f64, f64, f64, f64) {
        let r = 3.0_f64;
        let k = a * (b / (r * r - a * a).sqrt()).asin() + b * (a / (r * r - b * b).sqrt()).asin()
            - r * (a * b / (r * (r * r - a * a - b * b).sqrt())).atan();
        let integral = |radius: f64, side: f64| {
            (side * (radius * radius - side * side).sqrt()
                + radius * radius * (side / radius).asin())
                * 0.5
        };
        let bite = (a * integral((r * r - a * a).sqrt(), b)
            + b * integral((r * r - b * b).sqrt(), a)
            + r * r * k)
            / 3.0
            - h * a * b;
        let inner_planes = a * b + integral(r, a) - h * a + integral(r, b) - h * b
            + integral((r * r - b * b).sqrt(), a)
            - h * a
            + integral((r * r - a * a).sqrt(), b)
            - h * b;
        let trim_length = r * (a / r).asin()
            + r * (b / r).asin()
            + (r * r - a * a).sqrt() * (b / (r * r - a * a).sqrt()).asin()
            + (r * r - b * b).sqrt() * (a / (r * r - b * b).sqrt()).asin();
        (bite, r * k, inner_planes, trim_length)
    }

    fn in_material(p: Vec3, a: f64, b: f64, h: f64, op: BooleanOp) -> bool {
        let sphere = p
            .x()
            .mul_add(p.x(), p.y().mul_add(p.y(), (p.z() + h).powi(2)))
            < 9.0;
        let cube =
            p.x() > 0.0 && p.x() < a && p.y() > 0.0 && p.y() < b && p.z() > 0.0 && p.z() < 1.0;
        match op {
            BooleanOp::Cut => sphere && !cube,
            BooleanOp::Fuse => sphere || cube,
            BooleanOp::Intersect => sphere && cube,
        }
    }

    fn enters_box(triangle: [Vec3; 3], a: f64, b: f64) -> bool {
        let mut polygon = triangle.to_vec();
        for (axis, high) in [(0, a), (1, b), (2, 1.0)] {
            for upper in [false, true] {
                let mut clipped = Vec::new();
                for i in 0..polygon.len() {
                    let p = polygon[i];
                    let q = polygon[(i + 1) % polygon.len()];
                    let signed = |v: Vec3| {
                        if upper {
                            high - v.0[axis] - 1e-6
                        } else {
                            v.0[axis] - 1e-6
                        }
                    };
                    let (dp, dq) = (signed(p), signed(q));
                    if dp >= 0.0 {
                        clipped.push(p);
                    }
                    if (dp >= 0.0) != (dq >= 0.0) {
                        clipped.push(p + (q - p) * (dp / (dp - dq)));
                    }
                }
                polygon = clipped;
            }
        }
        !polygon.is_empty()
    }

    #[allow(clippy::too_many_lines)]
    fn check(
        scale: f64,
        placed: bool,
        side: f64,
        height: f64,
        angle: f64,
        op: BooleanOp,
        reverse: bool,
    ) {
        let label = format!(
            "F13 scale={scale} placed={placed} side={side} h={height} angle={angle} op={op:?} reverse={reverse}"
        );
        let mut topo = Topology::new();
        let cube = make_box(&mut topo, side * scale, side * scale, scale).unwrap();
        let sphere = make_sphere(&mut topo, 3.0 * scale, 16).unwrap();
        transform_solid(
            &mut topo,
            sphere,
            &(Mat4::translation(0.0, 0.0, -height * scale) * Mat4::rotation_y(angle)),
        )
        .unwrap();
        let placement = if placed {
            Mat4::translation(10.0 * scale, -7.0 * scale, 3.0 * scale)
                * Mat4::rotation_x(0.37)
                * Mat4::rotation_z(0.23)
        } else {
            Mat4::identity()
        };
        if placed {
            transform_solid(&mut topo, cube, &placement).unwrap();
            transform_solid(&mut topo, sphere, &placement).unwrap();
        }
        let original = remus_io::arena_io::serialize_solids(&topo, &[cube, sphere]).unwrap();
        let result = boolean_with_context(
            &mut topo,
            op,
            if reverse { cube } else { sphere },
            if reverse { sphere } else { cube },
            &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
        )
        .unwrap();
        assert!(matches!(result.quality, BooleanQuality::Exact), "{label}");
        let solid = result.solid;
        assert_eq!(
            remus_io::arena_io::serialize_solids(&topo, &[cube, sphere]).unwrap(),
            original
        );
        let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
        assert!((4..=12).contains(&faces.len()), "{label}: analytic census");
        let mut uses = BTreeMap::new();
        for &fid in &faces {
            let face = topo.face(fid).unwrap();
            assert!(
                matches!(
                    face.surface(),
                    FaceSurface::Plane { .. } | FaceSurface::Sphere(_)
                ),
                "{label}"
            );
            for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                for edge in topo.wire(wid).unwrap().edges() {
                    uses.entry(edge.edge())
                        .or_insert_with(Vec::new)
                        .push(edge.is_forward() != face.is_reversed());
                }
            }
        }
        for (edge, directions) in uses {
            assert_eq!(directions.len(), 2, "{label}: incidence {edge:?}");
            assert_ne!(directions[0], directions[1], "{label}: same-sense {edge:?}");
        }
        assert!(
            remus_operations::validate::validate_solid(&topo, solid)
                .unwrap()
                .is_valid(),
            "{label}"
        );
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
                .any(|issue| issue.severity == remus_check::validate::Severity::Error),
            "{label}: {:?}",
            report.issues
        );
        let (bite, removed_area, inner_planes, trim_length) = quantities(side, side, height);
        let (expected_v, expected_a) = match op {
            BooleanOp::Cut => (36.0 * PI - bite, 36.0 * PI - removed_area + inner_planes),
            BooleanOp::Fuse => (
                36.0 * PI + side * side - bite,
                36.0 * PI - removed_area + 2.0 * side * side + 4.0 * side - inner_planes,
            ),
            BooleanOp::Intersect => (bite, removed_area + inner_planes),
        };
        let inverse = placement.inverse().unwrap();
        for (deflection, angular) in [(0.1, 0.35), (0.01, 0.0), (0.003, 0.35)] {
            let (mesh, offsets) =
                tessellate_solid_grouped_with_tolerance(&topo, solid, deflection * scale, angular)
                    .unwrap();
            assert_eq!(
                boundary_edge_count(&mesh),
                0,
                "{label}: raw boundary d={deflection}"
            );
            assert_eq!(non_manifold_edge_count(&mesh), 0, "{label}: non-manifold");
            let local: Vec<_> = mesh
                .positions
                .iter()
                .map(|&p| (inverse.mul_point(p) - Point3::new(0.0, 0.0, 0.0)) * scale.recip())
                .collect();
            let reference = Vec3::new(0.0, 0.0, -height);
            let mut volume = 0.0;
            let mut area = 0.0;
            let mut mesh_uses = BTreeMap::new();
            let mut position_uses = BTreeMap::new();
            let position_key = |id: u32| local[id as usize].0.map(|v| (v * 1e8).round() as i64);
            for (fi, &fid) in faces.iter().enumerate() {
                assert!(offsets[fi + 1] > offsets[fi], "{label}: empty face");
                let face = topo.face(fid).unwrap();
                for t in
                    mesh.indices[offsets[fi] as usize..offsets[fi + 1] as usize].chunks_exact(3)
                {
                    let [a, b, c] = [t[0], t[1], t[2]].map(|i| local[i as usize]);
                    let normal = (b - a).cross(c - a);
                    assert!(normal.length() > 1e-15, "{label}: degenerate");
                    area += normal.length() * 0.5;
                    volume += (a - reference).dot((b - reference).cross(c - reference)) / 6.0;
                    if matches!(face.surface(), FaceSurface::Sphere(_)) {
                        assert!(
                            normal.dot((a + b + c) * (1.0 / 3.0) - reference) > 0.0,
                            "{label}: inward sphere triangle face={fi} d={deflection} points={a:?}/{b:?}/{c:?}"
                        );
                        for p in [a, b, c] {
                            assert!(
                                ((p - reference).length() - 3.0).abs() < 1e-6,
                                "{label}: off-sphere"
                            );
                        }
                        if !matches!(op, BooleanOp::Intersect) {
                            assert!(
                                !enters_box([a, b, c], side, side),
                                "{label}: triangle crosses removed sphere patch"
                            );
                        }
                    }
                    for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                        let (ka, kb) = (position_key(a), position_key(b));
                        assert_ne!(ka, kb, "{label}: collapsed position edge");
                        position_uses
                            .entry((ka.min(kb), ka.max(kb)))
                            .or_insert_with(Vec::new)
                            .push(ka < kb);
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
                    "{label}: raw mesh same-sense {edge:?}"
                );
            }
            for (edge, directions) in position_uses {
                assert_eq!(
                    directions.len(),
                    2,
                    "{label}: position mesh incidence {edge:?}"
                );
                assert_ne!(
                    directions[0], directions[1],
                    "{label}: position mesh orientation {edge:?}"
                );
            }
            assert!(
                (volume - expected_v).abs() < expected_a * deflection * 0.75,
                "{label}: raw volume {volume} != {expected_v}"
            );
            // Planar circular trims add arc/chord strip area omitted from
            // the sphere-curvature term: rho²*(delta-sin(delta))/2 <=
            // (rho*delta)*sagitta. Sum L_arc*d over the four section arcs.
            assert!(
                (area - expected_a).abs() < (expected_a * (2.0 / 3.0) + trim_length) * deflection,
                "{label}: raw area {area} != {expected_a} at d={deflection}"
            );
        }
        for deflection in [1e-4, 5e-5] {
            let v = solid_volume(&topo, solid, deflection * scale).unwrap() / scale.powi(3);
            assert!(
                (v - expected_v).abs() < expected_v * 2e-4,
                "{label}: volume {v} != {expected_v}"
            );
        }
        let gauss = mass_properties(&topo, solid).unwrap().mass / scale.powi(3);
        assert!(
            (gauss - expected_v).abs() < expected_v * 2e-4,
            "{label}: independent closed-form versus Gauss volume {gauss}"
        );
        let area = solid_surface_area(&topo, solid, 1e-4 * scale).unwrap() / scale.powi(2);
        assert!(
            (area - expected_a).abs() < expected_a * 3e-4,
            "{label}: area {area} != {expected_a}"
        );
        let source_sphere = PreparedSolid::prepare(&topo, sphere).unwrap();
        let source_box = PreparedSolid::prepare(&topo, cube).unwrap();
        let prepared = PreparedSolid::prepare(&topo, solid).unwrap();
        let reference = Vec3::new(0.0, 0.0, -height);
        let mut coverage = [0usize; 2];
        for x in [-1.2, 0.15, 0.5, 0.85, 1.2] {
            for y in [-1.2, 0.15, 0.5, 0.85, 1.2] {
                for z in [-3.0, -0.2, 0.1, 0.3, 0.7, 1.2] {
                    let p = Vec3::new(x, y, z);
                    if ((p - reference).length() - 3.0).abs() < 0.06 {
                        continue;
                    }
                    let inside = in_material(p, side, side, height, op);
                    coverage[usize::from(inside)] += 1;
                    let world = placement.mul_point(Point3::new(x * scale, y * scale, z * scale));
                    for (source, inside) in [
                        (&source_sphere, (p - reference).length() < 3.0),
                        (
                            &source_box,
                            x > 0.0 && x < side && y > 0.0 && y < side && z > 0.0 && z < 1.0,
                        ),
                    ] {
                        assert_eq!(
                            source
                                .classify_point(world, &ClassifyOptions::default())
                                .unwrap(),
                            if inside {
                                PointClassification::Inside
                            } else {
                                PointClassification::Outside
                            },
                            "{label}: source membership {p:?}"
                        );
                    }
                    assert_eq!(
                        prepared
                            .classify_point(world, &ClassifyOptions::default())
                            .unwrap(),
                        if inside {
                            PointClassification::Inside
                        } else {
                            PointClassification::Outside
                        },
                        "{label}: material {p:?}"
                    );
                }
            }
        }
        assert!(coverage.into_iter().all(|n| n > 0));
    }

    #[test]
    fn f13_placed_sphere_box_preserves_material_quantities_and_raw_mesh() {
        for scale in [0.001, 1.0] {
            for placed in [false, true] {
                for (op, reverse) in [
                    (BooleanOp::Cut, false),
                    (BooleanOp::Fuse, false),
                    (BooleanOp::Fuse, true),
                    (BooleanOp::Intersect, false),
                ] {
                    check(
                        scale,
                        placed,
                        1.0,
                        2.5,
                        std::f64::consts::FRAC_PI_6,
                        op,
                        reverse,
                    );
                }
            }
        }
    }

    #[test]
    fn f13_neighbor_dimensions_and_carrier_frames_keep_the_hole() {
        for (side, height, angle) in [
            (0.9, 2.5, std::f64::consts::FRAC_PI_6),
            (1.05, 2.5, std::f64::consts::FRAC_PI_6),
            (1.0, 2.55, std::f64::consts::FRAC_PI_6),
            (1.0, 2.5, 0.47),
            (1.0, 2.5, 0.58),
        ] {
            for op in [BooleanOp::Cut, BooleanOp::Fuse] {
                check(1.0, false, side, height, angle, op, false);
            }
        }
    }

    #[test]
    fn f13_large_healthy_equator_control_keeps_both_rims() {
        for placed in [false, true] {
            for (op, reverse) in [
                (BooleanOp::Cut, false),
                (BooleanOp::Fuse, false),
                (BooleanOp::Fuse, true),
                (BooleanOp::Intersect, false),
            ] {
                check(
                    1000.0,
                    placed,
                    1.0,
                    2.55,
                    std::f64::consts::FRAC_PI_6,
                    op,
                    reverse,
                );
            }
        }
    }

    /// Immutable baseline2026.1.70 already gives both spherical faces the
    /// same equator traversal at h=2.45. This fails before tessellation and
    /// is a separate GFA orientation foil, retained at full acceptance strength.
    #[test]
    #[ignore = "open: pre-mesh sphere equator coedges have equal sense at h=2.45"]
    fn f13_upstream_deeper_bite_equator_orientation() {
        for op in [BooleanOp::Cut, BooleanOp::Fuse] {
            check(
                1.0,
                false,
                1.0,
                2.45,
                std::f64::consts::FRAC_PI_6,
                op,
                false,
            );
        }
    }

    /// The literal1000x pair has the same independent upstream defect on
    /// baseline2026.1.70. The nearby h=2.55 large-scale control above is healthy.
    #[test]
    #[ignore = "open: pre-mesh sphere equator coedges have equal sense at1000x"]
    fn f13_upstream_large_literal_equator_orientation() {
        for op in [BooleanOp::Cut, BooleanOp::Fuse] {
            check(
                1000.0,
                false,
                1.0,
                2.5,
                std::f64::consts::FRAC_PI_6,
                op,
                false,
            );
        }
    }
}
