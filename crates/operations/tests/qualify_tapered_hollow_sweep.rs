//! P-Class 7.1: one scale law must move outer and inner profile boundaries.
//!
//! For a straight path and a linear scale law, the exact solid is the
//! difference of two square frusta. Its section and material oracles do not
//! depend on sweep assembly, tessellation, or a second property wrapper.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_operations::sweep::{SweepContactMode, SweepOptions, sweep_with_options};
use remus_topology::Topology;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;

fn profile(topo: &mut Topology, scale: f64, reverse_hole: bool) -> FaceId {
    profile_family(topo, scale, reverse_hole, true)
}

fn profile_family(topo: &mut Topology, scale: f64, reverse_hole: bool, hollow: bool) -> FaceId {
    let square = |half: f64| {
        vec![
            Point3::new(-half, -half, 0.0),
            Point3::new(half, -half, 0.0),
            Point3::new(half, half, 0.0),
            Point3::new(-half, half, 0.0),
        ]
    };
    let outer =
        remus_topology::builder::make_polygon_wire(topo, &square(2.0 * scale), 1e-7).unwrap();
    let mut hole = square(scale);
    if reverse_hole {
        hole.reverse();
    }
    let hole = remus_topology::builder::make_polygon_wire(topo, &hole, 1e-7).unwrap();
    topo.add_face(Face::new(
        outer,
        if hollow { vec![hole] } else { vec![] },
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ))
}

fn path(scale: f64, origin: Point3) -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![origin, origin + Vec3::new(0.0, 0.0, 6.0 * scale)],
        vec![1.0, 1.0],
    )
    .unwrap()
}

fn assert_closed_mesh(topo: &Topology, solid: SolidId, scale: f64) {
    let mesh = remus_operations::tessellate::tessellate_solid(topo, solid, scale * 0.01).unwrap();
    assert!(!mesh.indices.is_empty());
    assert!(remus_operations::tessellate::is_watertight(&mesh));
    assert_eq!(
        remus_operations::tessellate::non_manifold_edge_count(&mesh),
        0
    );
    let mut canonical = BTreeMap::new();
    let mut remap = Vec::new();
    for p in &mesh.positions {
        #[allow(clippy::cast_possible_truncation)]
        let key = (
            (p.x() / scale * 1e7).round() as i64,
            (p.y() / scale * 1e7).round() as i64,
            (p.z() / scale * 1e7).round() as i64,
        );
        let next = canonical.len();
        remap.push(*canonical.entry(key).or_insert(next));
    }
    let mut incidences = BTreeMap::<_, usize>::new();
    for triangle in mesh.indices.chunks_exact(3) {
        let ids = [
            remap[triangle[0] as usize],
            remap[triangle[1] as usize],
            remap[triangle[2] as usize],
        ];
        for (a, b) in [(ids[0], ids[1]), (ids[1], ids[2]), (ids[2], ids[0])] {
            *incidences.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    assert!(incidences.values().all(|count| *count == 2));
}

#[test]
fn tapered_hollow_sweep_matches_frustum_sections_material_and_volume() {
    for scale in [1.0, 1e-3, 1e3] {
        for reverse_hole in [false, true] {
            for (start, middle, end) in [
                (1.0, 1.5, 2.0),
                (2.0, 1.25, 0.5),
                (1.0, 2.0, 1.0),
                // A dimensionless scale change smaller than model tolerance
                // must still affect both boundaries.
                (1.0 + 5e-8, 1.0 + 5e-8, 1.0 + 5e-8),
            ] {
                let mut topo = Topology::new();
                let input = profile(&mut topo, scale, reverse_hole);
                let origin = Point3::new(3.0 * scale, -5.0 * scale, 7.0 * scale);
                let law = move |t: f64| {
                    if t <= 0.5 {
                        start + (middle - start) * (2.0 * t)
                    } else {
                        middle + (end - middle) * (2.0 * t - 1.0)
                    }
                };
                let options = SweepOptions {
                    contact_mode: SweepContactMode::Fixed,
                    scale_law: Some(Box::new(law)),
                    segments: 4,
                    ..SweepOptions::default()
                };
                let solid =
                    sweep_with_options(&mut topo, input, &path(scale, origin), &options).unwrap();
                // Integrate A(t) = 12*s(t)^2 separately over both linear pieces.
                let expected = 12.0
                    * 3.0
                    * (start * start
                        + start * middle
                        + 2.0 * middle * middle
                        + middle * end
                        + end * end)
                    / 3.0;
                let volume = remus_operations::measure::solid_volume(&topo, solid, scale * 0.01)
                    .unwrap()
                    / scale.powi(3);
                assert!(
                    (volume - expected).abs() < expected * 1e-8,
                    "scale={scale}, start={start}, end={end}: {volume} vs {expected}"
                );
                let report = remus_operations::validate::validate_solid(&topo, solid).unwrap();
                assert!(report.is_valid(), "{:?}", report.issues);
                let vertices = remus_topology::explorer::solid_vertices(&topo, solid).unwrap();
                for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
                    let radius = scale * law(t);
                    let mut inner = 0;
                    let mut outer = 0;
                    for vertex in &vertices {
                        let point = topo.vertex(*vertex).unwrap().point() - origin;
                        if (point.z() - 6.0 * scale * t).abs() > scale * 1e-8 {
                            continue;
                        }
                        if (point.x().abs() - radius).abs() < scale * 1e-8
                            && (point.y().abs() - radius).abs() < scale * 1e-8
                        {
                            inner += 1;
                        }
                        if (point.x().abs() - 2.0 * radius).abs() < scale * 1e-8
                            && (point.y().abs() - 2.0 * radius).abs() < scale * 1e-8
                        {
                            outer += 1;
                        }
                    }
                    assert_eq!(
                        (inner, outer),
                        (4, 4),
                        "wrong section at t={t}, scale={scale}"
                    );
                }
                for t in [0.125, 0.375, 0.625, 0.875] {
                    let radius = scale * law(t);
                    for (x, expected) in [(0.5, false), (1.5, true), (2.5, false)] {
                        let p = origin + Vec3::new(x * radius, 0.0, 6.0 * scale * t);
                        let state = remus_check::classify::classify_point(
                            &topo,
                            solid,
                            p,
                            &remus_check::classify::ClassifyOptions::default(),
                        )
                        .unwrap();
                        assert_eq!(
                            state == remus_check::classify::PointClassification::Inside,
                            expected,
                            "scale={scale}, t={t}, x={x}: {state:?}"
                        );
                    }
                }
                assert_closed_mesh(&topo, solid, scale);
            }
        }
    }
}

#[test]
fn tapered_hollow_sweep_invalid_actual_ring_scale_rolls_back() {
    let mut topo = Topology::new();
    let input = profile(&mut topo, 1.0, true);
    let before = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.faces().len(),
        topo.num_shells(),
        topo.num_solids(),
    );
    let history = topo.journal().snapshot();
    let options = SweepOptions {
        // k/64 preflight probes miss 1/3; the actual ring evaluation must refuse.
        scale_law: Some(Box::new(|t| {
            if (t - 1.0 / 3.0).abs() < 1e-12 {
                0.0
            } else {
                1.0
            }
        })),
        segments: 3,
        ..SweepOptions::default()
    };
    let error = sweep_with_options(
        &mut topo,
        input,
        &path(1.0, Point3::new(0.0, 0.0, 0.0)),
        &options,
    )
    .unwrap_err();
    assert!(matches!(
        error,
        remus_operations::OperationsError::InvalidInput { .. }
    ));
    assert_eq!(
        before,
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.faces().len(),
            topo.num_shells(),
            topo.num_solids()
        )
    );
    assert_eq!(topo.journal().snapshot(), history);
}

#[test]
fn tapered_hollow_sweep_scales_an_eccentric_hole_about_the_outer_profile_center() {
    let mut topo = Topology::new();
    let outer = remus_topology::builder::make_polygon_wire(
        &mut topo,
        &[
            Point3::new(-3.0, -2.0, 0.0),
            Point3::new(3.0, -2.0, 0.0),
            Point3::new(3.0, 2.0, 0.0),
            Point3::new(-3.0, 2.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let hole = remus_topology::builder::make_polygon_wire(
        &mut topo,
        &[
            Point3::new(0.2, -0.4, 0.0),
            Point3::new(0.2, 0.4, 0.0),
            Point3::new(1.2, 0.4, 0.0),
            Point3::new(1.2, -0.4, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let input = topo.add_face(Face::new(
        outer,
        vec![hole],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    let options = SweepOptions {
        contact_mode: SweepContactMode::Fixed,
        scale_law: Some(Box::new(|t| 1.0 + t)),
        segments: 4,
        ..SweepOptions::default()
    };
    let solid = sweep_with_options(
        &mut topo,
        input,
        &path(1.0, Point3::new(0.0, 0.0, 0.0)),
        &options,
    )
    .unwrap();
    // Outer area 24 minus hole area 0.8, times integral_0^6 (1+z/6)^2 dz = 14.
    let volume = remus_operations::measure::solid_volume(&topo, solid, 0.01).unwrap();
    assert!((volume - 23.2 * 14.0).abs() < 1e-8);
    // At z=4.5 the correct void is x=[0.35,2.1], y=[-0.7,0.7].
    // Scaling about the hole's own center instead gives x=[-0.175,1.575];
    // x=1.85 and x=-0.1 independently distinguish those two constructions.
    for (x, inside) in [
        (1.0, false),
        (1.85, false),
        (-0.1, true),
        (-1.0, true),
        (3.0, true),
        (6.0, false),
    ] {
        let state = remus_check::classify::classify_point(
            &topo,
            solid,
            Point3::new(x, 0.0, 4.5),
            &remus_check::classify::ClassifyOptions::default(),
        )
        .unwrap();
        assert_eq!(
            state == remus_check::classify::PointClassification::Inside,
            inside
        );
    }
    assert!(
        remus_operations::validate::validate_solid(&topo, solid)
            .unwrap()
            .is_valid()
    );
    assert_closed_mesh(&topo, solid, 1.0);
}

/// The same physical frusta must survive a change of native parameter labels.
/// High-anchor cases deliberately request more rings than representable native
/// coordinates, so a mere q -> native-u remapping cannot satisfy these oracles.
#[test]
fn native_domains_preserve_physical_sections_hollow_material_and_meshes() {
    for (a, b, segments) in [
        (0.0, 1.0, 4),
        (10.0, 20.0, 4),
        (-20.0, -10.0, 4),
        (-2.0, 3.0, 4),
        (0.0, 2.0, 4),
        (-1.0, 1.0, 4),
        (1e16, 1e16 + 16.0, 16),
        (1e16, 1e16 + 64.0, 64),
        (0.0, 1e308, 16),
    ] {
        for scale in [1e-3, 1.0, 1e3] {
            for placed in [false, true] {
                for hollow in [false, true] {
                    for contact_mode in [
                        SweepContactMode::Fixed,
                        SweepContactMode::RotationMinimizing,
                        SweepContactMode::ConstantNormal(Vec3::new(0.0, 1.0, 0.0)),
                    ] {
                        let mut topo = Topology::new();
                        let input = profile_family(&mut topo, scale, true, hollow);
                        let origin = if placed {
                            Point3::new(3.0 * scale, -5.0 * scale, 7.0 * scale)
                        } else {
                            Point3::new(0.0, 0.0, 0.0)
                        };
                        let curve = NurbsCurve::new(
                            1,
                            vec![a, a, b, b],
                            vec![origin, origin + Vec3::new(0.0, 0.0, 6.0 * scale)],
                            vec![1.0, 1.0],
                        )
                        .unwrap();
                        let original = curve.clone();
                        let options = SweepOptions {
                            contact_mode,
                            segments,
                            scale_law: Some(Box::new(|q| {
                                assert!((0.0..=1.0).contains(&q));
                                1.0 + q
                            })),
                            ..SweepOptions::default()
                        };
                        let solid = sweep_with_options(&mut topo, input, &curve, &options)
                            .unwrap_or_else(|error| panic!("domain [{a},{b}], scale={scale}, placed={placed}, hollow={hollow}: {error}"));
                        assert_eq!(curve, original);
                        let expected = if hollow { 168.0 } else { 224.0 };
                        let volume =
                            remus_operations::measure::solid_volume(&topo, solid, scale * 0.01)
                                .unwrap()
                                / scale.powi(3);
                        assert!(
                            (volume - expected).abs() < expected * 1e-8,
                            "{volume} vs {expected}"
                        );
                        let report =
                            remus_operations::validate::validate_solid(&topo, solid).unwrap();
                        assert!(report.is_valid(), "{report:?}");
                        let vertices =
                            remus_topology::explorer::solid_vertices(&topo, solid).unwrap();
                        for q in [0.0, 0.25, 0.5, 0.75, 1.0] {
                            let radius = scale * (1.0 + q);
                            let mut inner = 0;
                            let mut outer = 0;
                            for vertex in &vertices {
                                let point = topo.vertex(*vertex).unwrap().point() - origin;
                                if (point.z() - 6.0 * scale * q).abs() > scale * 1e-8 {
                                    continue;
                                }
                                if (point.x().abs() - radius).abs() < scale * 1e-8
                                    && (point.y().abs() - radius).abs() < scale * 1e-8
                                {
                                    inner += 1;
                                }
                                if (point.x().abs() - 2.0 * radius).abs() < scale * 1e-8
                                    && (point.y().abs() - 2.0 * radius).abs() < scale * 1e-8
                                {
                                    outer += 1;
                                }
                            }
                            assert_eq!(outer, 4, "outer section at q={q}, domain [{a},{b}]");
                            assert_eq!(inner, if hollow { 4 } else { 0 }, "inner section at q={q}");
                        }
                        for q in [0.125, 0.375, 0.625, 0.875] {
                            let r = scale * (1.0 + q);
                            for (x, inside) in [(0.5, !hollow), (1.5, true), (2.5, false)] {
                                let state = remus_check::classify::classify_point(
                                    &topo,
                                    solid,
                                    origin + Vec3::new(x * r, 0.0, 6.0 * scale * q),
                                    &remus_check::classify::ClassifyOptions::default(),
                                )
                                .unwrap();
                                assert_eq!(
                                    state == remus_check::classify::PointClassification::Inside,
                                    inside
                                );
                            }
                        }
                        assert_closed_mesh(&topo, solid, scale);
                    }
                }
            }
        }
    }
}
