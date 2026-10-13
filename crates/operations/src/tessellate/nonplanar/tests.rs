//! Exact predicate, rounding and complete-mesh oracles for boundary filtering.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::cell::Cell;

use remus_math::vec::Point2;

use super::PreparedBoundarySegments;

thread_local! {
    static REFERENCE: Cell<bool> = const { Cell::new(false) };
    static DISTANCE_TESTS: Cell<usize> = const { Cell::new(0) };
}

pub(super) fn use_reference() -> bool {
    REFERENCE.with(Cell::get)
}

pub(super) fn record_distance_test() {
    DISTANCE_TESTS.with(|count| count.set(count.get() + 1));
}

fn reset_count() {
    DISTANCE_TESTS.with(|count| count.set(0));
}

fn distance_tests() -> usize {
    DISTANCE_TESTS.with(Cell::get)
}

fn with_reference<T>(run: impl FnOnce() -> T) -> T {
    struct Reset(bool);
    impl Drop for Reset {
        fn drop(&mut self) {
            REFERENCE.with(|flag| flag.set(self.0));
        }
    }
    let old = REFERENCE.with(|flag| flag.replace(true));
    let _reset = Reset(old);
    run()
}

fn reference_segment_contact(
    start: (f64, f64),
    end: (f64, f64),
    point: Point2,
    clearance: f64,
) -> bool {
    let a = Point2::new(start.0, start.1);
    let b = Point2::new(end.0, end.1);
    let edge = b - a;
    let length_sq = edge.dot(edge);
    let fraction = if length_sq > 1e-30 {
        ((point - a).dot(edge) / length_sq).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let nearest = a + edge * fraction;
    (point - nearest).length() <= clearance
}

/// Independent copy of the original nested-loop predicate, with original
/// endpoints and the same short-circuit order. No prepared data is consulted.
pub(super) fn reference_is_on_boundary(
    outer: &[(f64, f64)],
    holes: &[Vec<(f64, f64)>],
    point: Point2,
    clearance: f64,
) -> bool {
    std::iter::once(outer)
        .chain(holes.iter().map(Vec::as_slice))
        .any(|wire| {
            (0..wire.len()).any(|index| {
                record_distance_test();
                reference_segment_contact(
                    wire[index],
                    wire[(index + 1) % wire.len()],
                    point,
                    clearance,
                )
            })
        })
}

#[test]
fn clearance_preserves_holes_empty_duplicate_and_unwrapped_seam_loops() {
    let outer = [(6.1, -2.), (6.5, -2.), (6.5, 2.), (6.1, 2.)];
    let holes = [
        vec![],
        vec![(6.2, -0.5), (6.4, -0.5), (6.4, 0.5), (6.2, 0.5)],
        vec![(6.3, 1.), (6.3, 1.), (6.3, 1.)],
    ];
    for shift in [-std::f64::consts::TAU, 0., std::f64::consts::TAU] {
        let outer: Vec<_> = outer.iter().map(|&(u, v)| (u + shift, v)).collect();
        let holes: Vec<Vec<_>> = holes
            .iter()
            .map(|wire| wire.iter().map(|&(u, v)| (u + shift, v)).collect())
            .collect();
        let prepared = PreparedBoundarySegments::new(&outer, &holes, 1e-10);
        for (u, v, expected) in [
            (6.1, 0., true),
            (6.3, 2., true),
            (6.2, 0., true),
            (6.3, 0., false),
            (6.3, 1., true),
            (6.0, 0., false),
            (6.5, 2., true),
        ] {
            let point = Point2::new(u + shift, v);
            assert_eq!(
                reference_is_on_boundary(&outer, &holes, point, 1e-10),
                expected
            );
            assert_eq!(prepared.is_on_boundary(point), expected);
        }
    }
    assert!(!PreparedBoundarySegments::new(&[], &[], 1e-10).is_on_boundary(Point2::new(0., 0.)));
}

#[test]
fn computed_nearest_endpoint_has_authority_over_ideal_segment_bounds() {
    // In binary64, b-a loses b here, and a + (b-a) is 0 rather than 1.
    // The old fraction=1 predicate therefore accepts (0,0). An AABB built
    // only from the ideal endpoints would incorrectly reject it.
    let outer = [(1e16, 0.), (1., 0.)];
    let point = Point2::new(0., 0.);
    let prepared = PreparedBoundarySegments::new(&outer, &[], 1e-10);
    assert_eq!(
        (prepared.segments[0].start + prepared.segments[0].edge)
            .x()
            .to_bits(),
        0_f64.to_bits()
    );
    assert!(reference_is_on_boundary(&outer, &[], point, 1e-10));
    assert!(prepared.is_on_boundary(point));
}

#[test]
fn clearance_matches_reference_at_ulps_across_scales_and_degeneracy() {
    let mut state = 0x0123_4567_89ab_cdef_u64;
    let mut next = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    for index in 0..8_000 {
        #[allow(clippy::cast_possible_wrap, clippy::cast_possible_truncation)]
        let exponent = (next() % 1000) as i32 - 500;
        let scale = 2_f64.powi(exponent);
        #[allow(clippy::cast_precision_loss)]
        let mut coordinate = || ((next() >> 11) as f64 / (1_u64 << 53) as f64 * 2. - 1.) * scale;
        let a = (coordinate(), coordinate());
        let b = if index % 11 == 0 {
            a
        } else {
            (coordinate(), coordinate())
        };
        let outer = [a, b];
        let clearance = if index % 3 == 0 { 1e-10 } else { scale * 1e-10 };
        let prepared = PreparedBoundarySegments::new(&outer, &[], clearance);
        for fraction in [0., 0.5, 1.] {
            let center = Point2::new(a.0 + (b.0 - a.0) * fraction, a.1 + (b.1 - a.1) * fraction);
            for axis in 0..2 {
                for offset in [-2., -1., 0., 1., 2.] {
                    let mut p = [center.x(), center.y()];
                    p[axis] += clearance * offset;
                    for ulps in -2_i32..=2 {
                        let mut q = p;
                        for _ in 0..ulps.unsigned_abs() {
                            q[axis] = if ulps < 0 {
                                q[axis].next_down()
                            } else {
                                q[axis].next_up()
                            };
                        }
                        let point = Point2::new(q[0], q[1]);
                        // Check each directed segment independently too: the
                        // reverse closing segment must not mask a false reject.
                        for (index, segment) in prepared.segments.iter().enumerate() {
                            if reference_segment_contact(
                                outer[index],
                                outer[(index + 1) % 2],
                                point,
                                clearance,
                            ) && let Some((lower, upper)) = segment.bounds
                            {
                                assert!(
                                    point.x() >= lower[0]
                                        && point.x() <= upper[0]
                                        && point.y() >= lower[1]
                                        && point.y() <= upper[1]
                                );
                            }
                        }
                        assert_eq!(
                            prepared.is_on_boundary(point),
                            reference_is_on_boundary(&outer, &[], point, clearance),
                            "a={a:?}, b={b:?}, query={q:?}, clearance={clearance:e}"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn unknown_ranges_and_nonfinite_queries_retain_the_narrow_scan() {
    for outer in [
        [(f64::MAX, 0.), (-f64::MAX, 0.)],
        [(f64::NAN, 0.), (0., 0.)],
        [(f64::INFINITY, 0.), (0., 0.)],
    ] {
        let prepared = PreparedBoundarySegments::new(&outer, &[], 1e-10);
        assert!(
            prepared
                .segments
                .iter()
                .all(|segment| segment.bounds.is_none())
        );
        for point in [
            Point2::new(0., 0.),
            Point2::new(f64::NAN, 0.),
            Point2::new(f64::INFINITY, 0.),
        ] {
            reset_count();
            let fast = prepared.is_on_boundary(point);
            assert!(distance_tests() > 0);
            assert_eq!(fast, reference_is_on_boundary(&outer, &[], point, 1e-10));
        }
    }
    for clearance in [0., f64::MIN_POSITIVE, f64::NAN, f64::INFINITY, f64::MAX] {
        let outer = [(0., 0.), (1., 1.)];
        let prepared = PreparedBoundarySegments::new(&outer, &[], clearance);
        assert!(
            prepared
                .segments
                .iter()
                .all(|segment| segment.bounds.is_none())
        );
        for point in [
            Point2::new(0., 0.),
            Point2::new(0.5, 0.5),
            Point2::new(9., 9.),
        ] {
            assert_eq!(
                prepared.is_on_boundary(point),
                reference_is_on_boundary(&outer, &[], point, clearance)
            );
        }
    }
}

#[test]
fn interior_queries_avoid_all_exact_segment_distances() {
    let mut outer = Vec::new();
    for side in 0..4 {
        for step in 0..128 {
            let t = f64::from(step) / 128.;
            outer.push(match side {
                0 => (-1. + 2. * t, -1.),
                1 => (1., -1. + 2. * t),
                2 => (1. - 2. * t, 1.),
                _ => (-1., 1. - 2. * t),
            });
        }
    }
    let prepared = PreparedBoundarySegments::new(&outer, &[], 1e-10);
    reset_count();
    for x in 0..32 {
        for y in 0..32 {
            assert!(!prepared.is_on_boundary(Point2::new(
                f64::from(x) / 64. - 0.25,
                f64::from(y) / 64. - 0.25
            )));
        }
    }
    assert_eq!(
        distance_tests(),
        0,
        "broad-phase rejection must avoid every narrow distance"
    );
    reset_count();
    for x in 0..32 {
        for y in 0..32 {
            assert!(!reference_is_on_boundary(
                &outer,
                &[],
                Point2::new(f64::from(x) / 64. - 0.25, f64::from(y) / 64. - 0.25),
                1e-10
            ));
        }
    }
    assert_eq!(distance_tests(), 512 * 32 * 32);
}

fn assert_mesh_bits(a: &super::super::TriangleMesh, b: &super::super::TriangleMesh, context: &str) {
    assert_eq!(a.indices, b.indices, "{context}: indices");
    let positions = |mesh: &super::super::TriangleMesh| {
        mesh.positions
            .iter()
            .map(|p| p.0.map(f64::to_bits))
            .collect::<Vec<_>>()
    };
    let normals = |mesh: &super::super::TriangleMesh| {
        mesh.normals
            .iter()
            .map(|p| p.0.map(f64::to_bits))
            .collect::<Vec<_>>()
    };
    assert_eq!(positions(a), positions(b), "{context}: positions");
    assert_eq!(normals(a), normals(b), "{context}: normals");
}

fn assert_pipeline_bits(
    topo: &remus_topology::Topology,
    solid: remus_topology::solid::SolidId,
    deflection: f64,
    context: &str,
) {
    use super::super::{
        clear_face_mesh_cache, tessellate_solid_for_measurement,
        tessellate_solid_grouped_with_tolerance,
    };
    clear_face_mesh_cache();
    let reference = with_reference(|| {
        tessellate_solid_grouped_with_tolerance(topo, solid, deflection, 0.35).unwrap()
    });
    clear_face_mesh_cache();
    let fast = tessellate_solid_grouped_with_tolerance(topo, solid, deflection, 0.35).unwrap();
    assert_eq!(reference.1, fast.1, "{context}: grouped offsets");
    assert_mesh_bits(&reference.0, &fast.0, context);
    clear_face_mesh_cache();
    let reference_quantity =
        with_reference(|| tessellate_solid_for_measurement(topo, solid, deflection).unwrap());
    clear_face_mesh_cache();
    let fast_quantity = tessellate_solid_for_measurement(topo, solid, deflection).unwrap();
    assert_mesh_bits(&reference_quantity, &fast_quantity, context);
    // Quantity replay and subsequent display replay must preserve each mode's
    // normal ownership, as well as geometry and face attribution.
    let warm_quantity = tessellate_solid_for_measurement(topo, solid, deflection).unwrap();
    assert_mesh_bits(&fast_quantity, &warm_quantity, context);
    let warm_display =
        tessellate_solid_grouped_with_tolerance(topo, solid, deflection, 0.35).unwrap();
    assert_eq!(reference.1, warm_display.1);
    assert_mesh_bits(&reference.0, &warm_display.0, context);
}

#[test]
fn boundary_filter_preserves_all_carriers_holes_cavities_and_periodic_seams() {
    use super::super::{disable_face_mesh_cache, enable_face_mesh_cache};
    use remus_topology::Topology;
    struct Reset;
    impl Drop for Reset {
        fn drop(&mut self) {
            disable_face_mesh_cache();
        }
    }
    enable_face_mesh_cache();
    let _reset = Reset;
    let mut topo = Topology::new();
    let mut bodies = super::super::tests::mesh_passes::bodies(&mut topo);
    let converted = crate::primitives::make_cylinder(&mut topo, 1., 2.).unwrap();
    crate::heal::convert_to_bspline(&mut topo, converted).unwrap();
    bodies.push(("converted periodic cylinder", converted));
    for (label, solid) in bodies {
        for deflection in [0.1, 0.02] {
            assert_pipeline_bits(&topo, solid, deflection, label);
        }
    }
    let (topo, drilled) = super::super::tests::shaft_drilled_with(1.5);
    for deflection in [0.05, 0.005] {
        assert_pipeline_bits(&topo, drilled, deflection, "bore crossing periodic seam");
    }
}

#[test]
fn fresh_edited_holder_mesh_and_quantity_bits_match_with_fewer_distances() {
    use super::super::{
        clear_face_mesh_cache, disable_face_mesh_cache, tessellate_solid_for_measurement,
    };
    use remus_topology::Topology;
    disable_face_mesh_cache();
    let mut source = Topology::new();
    let solid = remus_io::step::reader::read_step(
        include_str!("../../../../io/tests/data/shapr3d_hammer_holder.step"),
        &mut source,
    )
    .unwrap()[0];
    let faces = remus_topology::explorer::solid_faces(&source, solid).unwrap();
    assert_eq!(faces.len(), 160);
    let face = faces
        .into_iter()
        .find(|&face| {
            source
                .face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.x() > 0.99)
                && (crate::measure::face_area(&source, face, 0.05).unwrap() - 1_045.930_731_242_697)
                    .abs()
                    < 1e-4
        })
        .expect("holder +X width face");
    for distance in [5., 5.06] {
        let mut topo = source.clone();
        let moved = crate::push_pull::move_faces(&mut topo, solid, &[face], distance).unwrap();
        let bounds = crate::measure::solid_bounding_box(&topo, moved).unwrap();
        let deflection = 0.08_f64.min(((bounds.max - bounds.min).length() * 5e-5).max(1e-9));
        reset_count();
        let reference =
            with_reference(|| tessellate_solid_for_measurement(&topo, moved, deflection).unwrap());
        let original_distances = distance_tests();
        reset_count();
        let fast = tessellate_solid_for_measurement(&topo, moved, deflection).unwrap();
        let filtered_distances = distance_tests();
        assert_mesh_bits(&reference, &fast, "fresh holder quantity mesh");
        assert!(original_distances > 9_000_000);
        assert!(
            filtered_distances < original_distances / 20,
            "{filtered_distances} of {original_distances} distances remain"
        );
        crate::measure::clear_thread_volume_memo();
        clear_face_mesh_cache();
        let reference_volume =
            with_reference(|| crate::measure::solid_volume(&topo, moved, 0.08).unwrap());
        crate::measure::clear_thread_volume_memo();
        clear_face_mesh_cache();
        let fast_volume = crate::measure::solid_volume(&topo, moved, 0.08).unwrap();
        assert_eq!(
            reference_volume.to_bits(),
            fast_volume.to_bits(),
            "distance {distance}: fresh quantity bits"
        );
    }
}
