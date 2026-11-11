//! The face-integral cache against the uncached integrator on faces with
//! nested inner loops, primed in both orders.
//!
//! The cache answers a face from an entry of identical content bit for bit,
//! and a rigid translation of an entry's face from that entry re-expressed
//! about the new reference, equal to integrating afresh up to rounding (see
//! [`crate::properties::face_cache`]). On review head c02ec634 the
//! translated reading of a nested face returned the other face's
//! subtract-every-hole value: 4666.67 for 5999.99.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::TAU;

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use super::{InnerLoopNesting, inner_loop_nesting, with_nesting_work_limit};
use crate::CheckError;
use crate::properties::face_cache::{FaceIntegralCache, FaceVolumeTerm};
use crate::properties::face_integrator::{FaceContribution, integrate_face_fixed_about};

const UP: Vec3 = Vec3::new(0.0, 0.0, 1.0);
const ORDER: usize = 5;

/// The review's square island half-width 9.999999 in a hole half-width 10,
/// in a square of half-width 30, centred at `(offset, offset, 5)`.
fn squares(offset: f64) -> (Topology, FaceId) {
    let mut t = Topology::new();
    let mut square = |r: f64| {
        make_polygon_wire(
            &mut t,
            &[
                Point3::new(offset - r, offset - r, 5.0),
                Point3::new(offset + r, offset - r, 5.0),
                Point3::new(offset + r, offset + r, 5.0),
                Point3::new(offset - r, offset + r, 5.0),
            ],
            1e-10,
        )
        .unwrap()
    };
    let outer = square(30.0);
    let hole = square(10.0);
    let island = square(9.999_999);
    let f = t.add_face(Face::new(
        outer,
        vec![hole, island],
        FaceSurface::Plane { normal: UP, d: 5.0 },
    ));
    (t, f)
}

fn cold(t: &Topology, f: FaceId, reference: Point3) -> FaceContribution {
    integrate_face_fixed_about(t, f, ORDER, reference).unwrap()
}

/// A translated reading against a fresh one, to the rounding the cache
/// documents for re-expressed readings: the translated copy's own input is
/// rounded at `shift`, which moves its boundary (perimeter under 400) by
/// `64 ε (shift + 100)`, and re-expressing a volume regroups a sum.
fn assert_within_rounding(warm: &FaceVolumeTerm, cold: &FaceContribution, shift: f64, what: &str) {
    let moved = 400.0 * 64.0 * f64::EPSILON * (shift + 100.0);
    let area = moved + 1e-12 * cold.area.abs();
    let volume = 2.0f64.mul_add(moved, 1e-9 * cold.volume.abs().max(cold.area.abs()));
    assert!(
        (warm.volume - cold.volume).abs() <= volume,
        "{what}: volume {} against {}",
        warm.volume,
        cold.volume
    );
    assert!(
        (warm.area - cold.area).abs() <= area,
        "{what}: area {} against {}",
        warm.area,
        cold.area
    );
}

/// An identical-content reading against a fresh one: bit for bit.
fn assert_bits(warm: &FaceVolumeTerm, cold: &FaceContribution, what: &str) {
    assert_eq!(
        warm.volume.to_bits(),
        cold.volume.to_bits(),
        "{what}: volume"
    );
    assert_eq!(warm.area.to_bits(), cold.area.to_bits(), "{what}: area");
}

/// Review 2, primed both ways: a warm reading of either face equals its
/// cold reading.
#[test]
fn review_squares_read_the_same_warm_and_cold_in_both_orders() {
    let (near, near_face) = squares(0.0);
    let (far, far_face) = squares(1e6);
    let reference = Point3::new(0.0, 0.0, 0.0);
    let cold_near = cold(&near, near_face, reference);
    let cold_far = cold(&far, far_face, reference);
    // The uncached readings are the nested answer at either position.
    assert!((cold_near.area - 3_599.999_920_000_001).abs() < 1e-6);
    assert!((cold_far.area - 3_599.999_920_000_001).abs() < 1e-6);

    for prime_far_first in [true, false] {
        let mut cache = FaceIntegralCache::new();
        let order: [(&Topology, FaceId, &FaceContribution); 2] = if prime_far_first {
            [(&far, far_face, &cold_far), (&near, near_face, &cold_near)]
        } else {
            [(&near, near_face, &cold_near), (&far, far_face, &cold_far)]
        };
        let primed = cache
            .integrate_face_volume_about(order[0].0, order[0].1, ORDER, reference)
            .unwrap();
        assert_bits(&primed, order[0].2, "the miss that primed the cache");
        let warm = cache
            .integrate_face_volume_about(order[1].0, order[1].1, ORDER, reference)
            .unwrap();
        assert_eq!(cache.stats().translated, 1, "{:?}", cache.stats());
        assert_within_rounding(&warm, order[1].2, 1e6, "translated reading");
        // The same content again: identical, bit for bit.
        let again = cache
            .integrate_face_volume_about(order[0].0, order[0].1, ORDER, reference)
            .unwrap();
        assert_bits(&again, order[0].2, "identical reading");
    }
}

/// A translated face whose nesting reads differently is never answered from
/// the other face's entry, in either priming order.
#[test]
fn a_translated_hit_needs_the_same_nesting() {
    let (near, near_face) = squares(0.0);
    let (far, far_face) = squares(1e6);
    let reference = Point3::new(0.0, 0.0, 0.0);
    // Starve the far face's nesting decision: it reads undecided, so the
    // uncached integration refuses it.
    let starved = |t: &Topology, f: FaceId| {
        with_nesting_work_limit(1, || {
            assert!(matches!(
                inner_loop_nesting(t, f),
                InnerLoopNesting::Undecided(_)
            ));
            integrate_face_fixed_about(t, f, ORDER, reference)
        })
    };
    assert!(matches!(
        starved(&far, far_face),
        Err(CheckError::UndecidedLoopNesting(_))
    ));

    // Primed with the nested near face, the starved far face still refuses.
    let mut cache = FaceIntegralCache::new();
    cache
        .integrate_face_volume_about(&near, near_face, ORDER, reference)
        .unwrap();
    let warm = with_nesting_work_limit(1, || {
        cache.integrate_face_volume_about(&far, far_face, ORDER, reference)
    });
    assert!(
        matches!(warm, Err(CheckError::UndecidedLoopNesting(_))),
        "{warm:?}"
    );
    assert_eq!(cache.stats().translated, 0);

    // A refusal is never cached, so the reverse order integrates afresh.
    let mut cache = FaceIntegralCache::new();
    let refused = with_nesting_work_limit(1, || {
        cache.integrate_face_volume_about(&far, far_face, ORDER, reference)
    });
    assert!(refused.is_err());
    let fresh = cache
        .integrate_face_volume_about(&near, near_face, ORDER, reference)
        .unwrap();
    assert_bits(
        &fresh,
        &cold(&near, near_face, reference),
        "after a refusal",
    );
    assert_eq!(cache.stats().insertions, 1);
}

fn circle(t: &mut Topology, c: Point3, r: f64, arcs: usize) -> WireId {
    let curve = Circle3D::new_with_ref(c, UP, r, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let ec = EdgeCurve::Circle(curve.clone());
    arcs_wire(t, arcs, &|u| curve.evaluate(u), &ec)
}

fn ellipse(t: &mut Topology, c: Point3, a: f64, b: f64, arcs: usize) -> WireId {
    let curve = Ellipse3D::new_with_ref(c, UP, a, b, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let ec = EdgeCurve::Ellipse(curve.clone());
    arcs_wire(t, arcs, &|u| curve.evaluate(u), &ec)
}

#[allow(clippy::cast_precision_loss)]
fn arcs_wire(t: &mut Topology, arcs: usize, at: &dyn Fn(f64) -> Point3, ec: &EdgeCurve) -> WireId {
    let angle = |i: usize| TAU * i as f64 / arcs as f64;
    let vertices: Vec<_> = (0..arcs)
        .map(|i| t.add_vertex(Vertex::new(at(angle(i)), 1e-9)))
        .collect();
    let edges = (0..arcs)
        .map(|i| {
            let mut edge = Edge::new(vertices[i], vertices[(i + 1) % arcs], ec.clone());
            edge.set_trim(Some((angle(i), angle(i + 1))));
            OrientedEdge::new(t.add_edge(edge), true)
        })
        .collect();
    t.add_wire(Wire::new(edges, true).unwrap())
}

/// A seeded family of nested faces, each measured cold and warm (primed
/// with a translated copy, and the reverse): every warm reading equals the
/// cold one to rounding, and every translated reading reused a face whose
/// nesting was the same.
#[test]
fn translated_nested_family_reads_warm_as_cold() {
    let mut seed = 0x2545_f491_4f6c_dd1d_u64;
    let mut next = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        #[allow(clippy::cast_precision_loss)]
        let unit = (seed >> 11) as f64 / (1u64 << 53) as f64;
        unit
    };
    let mut translated_hits = 0;
    for case in 0..16 {
        let gap = 10f64.powf(-1.0 - 6.0 * next());
        let shift = 10f64.powf(2.0 + 5.0 * next());
        let arcs = 1 + case % 4;
        let build = |offset: f64| {
            let mut t = Topology::new();
            let c = Point3::new(offset + 1.25, -offset, 5.0);
            let hole = circle(&mut t, c, 10.0, arcs);
            let island = ellipse(&mut t, c, 10.0 - gap, 8.0, 1 + case % 3);
            let pocket = circle(&mut t, c, 3.0, 2);
            let outer = make_polygon_wire(
                &mut t,
                &[
                    Point3::new(c.x() - 30.0, c.y() - 30.0, 5.0),
                    Point3::new(c.x() + 30.0, c.y() - 30.0, 5.0),
                    Point3::new(c.x() + 30.0, c.y() + 30.0, 5.0),
                    Point3::new(c.x() - 30.0, c.y() + 30.0, 5.0),
                ],
                1e-9,
            )
            .unwrap();
            let f = t.add_face(Face::new(
                outer,
                vec![pocket, hole, island],
                FaceSurface::Plane { normal: UP, d: 5.0 },
            ));
            (t, f)
        };
        let (a, fa) = build(0.0);
        let (b, fb) = build(shift);
        assert_eq!(
            inner_loop_nesting(&a, fa),
            InnerLoopNesting::Nested(vec![2, 0, 1])
        );
        assert_eq!(
            inner_loop_nesting(&b, fb),
            InnerLoopNesting::Nested(vec![2, 0, 1])
        );
        let reference = Point3::new(0.5 * shift, -0.25 * shift, 0.0);
        let (cold_a, cold_b) = (cold(&a, fa, reference), cold(&b, fb, reference));
        // The nested area: the island and its pocket.
        let want = 3600.0 - std::f64::consts::PI * (100.0 - (10.0 - gap) * 8.0 + 9.0);
        assert!((cold_a.area - want).abs() <= 1e-9 * want, "case {case}");
        for (first, second) in [((&a, fa), (&b, fb, &cold_b)), ((&b, fb), (&a, fa, &cold_a))] {
            let mut cache = FaceIntegralCache::new();
            cache
                .integrate_face_volume_about(first.0, first.1, ORDER, reference)
                .unwrap();
            let warm = cache
                .integrate_face_volume_about(second.0, second.1, ORDER, reference)
                .unwrap();
            translated_hits += cache.stats().translated;
            assert_within_rounding(&warm, second.2, shift, "seeded family");
        }
    }
    // The family exercises the translated readings, not just misses.
    assert!(translated_hits > 0);
}
