//! The #1879 review's scenarios: inputs where the fan's chord bound must
//! still cover the fan's error, or decline.
//!
//! Every face here is forced to the fan by a loop `1e-8` wide at its first
//! corner (see `fan_bound_tests`), off the origin.

#![allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]

use std::f64::consts::FRAC_1_SQRT_2;

use super::*;
use remus_math::nurbs::curve::NurbsCurve;
use remus_topology::edge::Edge;
use remus_topology::face::Face;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// One boundary edge: the position of the vertex it leaves, its curve and
/// its trim. Each edge ends at the next entry's vertex, the last at the
/// first's.
struct Leg {
    from: Point3,
    curve: EdgeCurve,
    trim: Option<(f64, f64)>,
}

fn line(from: Point3) -> Leg {
    Leg {
        from,
        curve: EdgeCurve::Line,
        trim: None,
    }
}

fn nurbs(from: Point3, curve: NurbsCurve, trim: (f64, f64)) -> Leg {
    Leg {
        from,
        curve: EdgeCurve::NurbsCurve(curve),
        trim: Some(trim),
    }
}

/// The loop `1e-8` wide that leaves from and returns to `at`, closing the
/// wire into the first leg's vertex: it forces the fan.
fn lump(at: Point3) -> Leg {
    let off = Point3::new(at.x() + 1e-8, at.y() + 1e-8, at.z());
    nurbs(
        at,
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![at, off, at],
            vec![1.0, FRAC_1_SQRT_2, 1.0],
        )
        .unwrap(),
        (0.0, 1.0),
    )
}

fn wire_of(topo: &mut Topology, legs: Vec<Leg>) -> remus_topology::wire::WireId {
    let vertices: Vec<_> = legs
        .iter()
        .map(|leg| topo.add_vertex(Vertex::new(leg.from, 1e-7)))
        .collect();
    let n = legs.len();
    let edges = legs
        .into_iter()
        .enumerate()
        .map(|(i, leg)| {
            let mut edge = Edge::new(vertices[i], vertices[(i + 1) % n], leg.curve);
            edge.set_trim(leg.trim);
            OrientedEdge::new(topo.add_edge(edge), true)
        })
        .collect();
    topo.add_wire(Wire::new(edges, true).unwrap())
}

fn planar_face(
    topo: &mut Topology,
    outer: Vec<Leg>,
    holes: Vec<remus_topology::wire::WireId>,
    z: f64,
) -> FaceId {
    let wire = wire_of(topo, outer);
    topo.add_face(Face::new(
        wire,
        holes,
        FaceSurface::Plane { normal: Z, d: z },
    ))
}

/// The unit square at `c` (counter-clockwise from `c`), its sides given by
/// `sides` (one per corner, leaving it), closed by the fan-forcing loop.
fn square(c: Point3, s: f64, sides: [Option<(NurbsCurve, (f64, f64))>; 4]) -> Vec<Leg> {
    let corners = [
        c,
        Point3::new(c.x() + s, c.y(), c.z()),
        Point3::new(c.x() + s, c.y() + s, c.z()),
        Point3::new(c.x(), c.y() + s, c.z()),
    ];
    let mut legs: Vec<Leg> = corners
        .iter()
        .zip(sides)
        .map(|(&p, side)| match side {
            Some((curve, trim)) => nurbs(p, curve, trim),
            None => line(p),
        })
        .collect();
    legs.push(lump(c));
    legs
}

fn reading(topo: &Topology, face: FaceId, reference: Point3) -> FluxReading {
    assert!(
        integrate_planar_face_exact(topo, face, Z, reference)
            .unwrap()
            .is_none(),
        "the loop forces the fan"
    );
    integrate_face_fixed_flux_with_boundary_about(topo, face, 5, reference, true).unwrap()
}

fn certificate(reading: &FluxReading) -> PlanarFanBound {
    reading
        .planar_fan
        .expect("a fan reading carries its certificate")
}

/// Review finding 1: a closed NURBS hole whose carrier laps a square twice,
/// trimmed to its first lap. The fan samples the whole carrier and removes
/// the square twice; the bound must decline it.
#[test]
fn a_closed_nurbs_edge_trimmed_short_of_its_domain_declines() {
    let c = Point3::new(5.0, 3.0, 1.0);
    let mut topo = Topology::new();
    let h = Point3::new(c.x() + 0.75, c.y() + 0.75, c.z());
    let corners = [
        h,
        Point3::new(h.x() + 0.5, h.y(), h.z()),
        Point3::new(h.x() + 0.5, h.y() + 0.5, h.z()),
        Point3::new(h.x(), h.y() + 0.5, h.z()),
    ];
    let mut points = Vec::new();
    for _ in 0..2 {
        points.extend_from_slice(&corners);
    }
    points.push(h);
    #[allow(clippy::cast_precision_loss)]
    let knots: Vec<f64> = std::iter::once(0.0)
        .chain((0..=8).map(|i| f64::from(i) / 8.0))
        .chain(std::iter::once(1.0))
        .collect();
    let lapped = NurbsCurve::new(1, knots, points, vec![1.0; 9]).unwrap();
    let seam = topo.add_vertex(Vertex::new(h, 1e-7));
    let mut edge = Edge::new(seam, seam, EdgeCurve::NurbsCurve(lapped));
    edge.set_trim(Some((0.0, 0.5)));
    let e = topo.add_edge(edge);
    let hole = topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap());
    let face = planar_face(
        &mut topo,
        square(c, 2.0, [None, None, None, None]),
        vec![hole],
        1.0,
    );
    let read = reading(&topo, face, Point3::new(0.0, 0.0, 0.0));
    let error = (read.contribution.area - (4.0 - 0.25)).abs();
    let bound = certificate(&read).area_bound;
    assert!(
        error <= bound,
        "the fan is {error:.3e} off, bound {bound:.3e}"
    );
    assert!(bound.is_infinite(), "a trimmed closed NURBS edge declines");
}

/// Review finding 2: a degree-one detour through `(0.01, 0.125)` and back
/// at `x = 0.02`, before the first fan sample, with every weight the
/// smallest positive `f64`. Unscaled, its homogeneous `y` rounds to zero.
#[test]
fn a_detour_with_subnormal_weights_is_charged() {
    let c = Point3::new(5.0, 3.0, 1.0);
    let at = |x: f64, y: f64| Point3::new(c.x() + x, c.y() + y, c.z());
    let detour = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 0.01, 0.02, 1.0, 1.0],
        vec![at(0.0, 0.0), at(0.01, 0.125), at(0.02, 0.0), at(1.0, 0.0)],
        vec![f64::from_bits(1); 4],
    )
    .unwrap();
    let mut topo = Topology::new();
    let face = planar_face(
        &mut topo,
        square(c, 1.0, [Some((detour, (0.0, 1.0))), None, None, None]),
        vec![],
        1.0,
    );
    let read = reading(&topo, face, Point3::new(0.0, 0.0, 0.0));
    // The detour bites a triangle of area ½ · 0.02 · 0.125 out of the square.
    let error = (read.contribution.area - (1.0 - 0.00125)).abs();
    let bound = certificate(&read).area_error(c.x() + c.y() + c.z());
    assert!(
        error <= bound,
        "the fan is {error:.3e} off, bound {bound:.3e}"
    );
}

/// Review finding 3: a square on `z = 0` whose top side bows `1e-5` out of
/// the plane (a straight line in plan). About a reference on the plane the
/// true term is zero, but the fan integrates its samples where they are.
#[test]
fn samples_off_the_plane_are_charged() {
    let c = Point3::new(5.0, 3.0, 0.0);
    let top = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(c.x() + 1.0, c.y() + 1.0, 0.0),
            Point3::new(c.x() + 0.5, c.y() + 1.0, 1e-5),
            Point3::new(c.x(), c.y() + 1.0, 0.0),
        ],
        vec![1.0; 3],
    )
    .unwrap();
    let mut topo = Topology::new();
    let face = planar_face(
        &mut topo,
        square(c, 1.0, [None, None, Some((top, (0.0, 1.0))), None]),
        vec![],
        0.0,
    );
    let reference = Point3::new(c.x() + 0.5, c.y() + 0.5, 0.0);
    let read = reading(&topo, face, reference);
    let error = read.contribution.volume.abs();
    let anchor = c - reference;
    let magnitude = c.x().abs() + c.y().abs() + c.z().abs();
    let bound = certificate(&read).volume_bound(0.0, anchor.length(), magnitude);
    assert!(
        error > 0.0 && error <= bound,
        "the fan term is {error:.3e}, bound {bound:.3e}"
    );
}

/// Review finding 4: a straight NURBS top side that ends at `(0, 1)` while
/// the vertex it shares with the next side sits `ε` above, within its
/// tolerance. The fan's chord from the top side's last sample to that
/// vertex adds a triangle of `ε / 64`.
#[test]
fn a_gap_between_an_edge_end_and_its_vertex_is_charged() {
    let c = Point3::new(5.0, 3.0, 1.0);
    let eps = 1e-7;
    let top = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(c.x() + 1.0, c.y() + 1.0, c.z()),
            Point3::new(c.x(), c.y() + 1.0, c.z()),
        ],
        vec![1.0; 2],
    )
    .unwrap();
    let mut legs = square(c, 1.0, [None, None, Some((top, (0.0, 1.0))), None]);
    legs[3].from = Point3::new(c.x(), c.y() + 1.0 + eps, c.z());
    let mut topo = Topology::new();
    let face = planar_face(&mut topo, legs, vec![], 1.0);
    let read = reading(&topo, face, Point3::new(0.0, 0.0, 0.0));
    let error = (read.contribution.area - 1.0).abs();
    let bound = certificate(&read).area_bound;
    assert!(
        error > 1e-10 && error <= bound,
        "the fan is {error:.3e} off, bound {bound:.3e}"
    );
}

/// Re-check finding 1: the fan subtracts the reference before it forms its
/// triangles, so about a reference `2^54` away every `x` of a unit square
/// rounds alike and the fan reads zero, where the face's term is a third of
/// its area times `1e6`.
#[test]
fn a_far_reference_charges_the_fans_coordinate_rounding() {
    // Off the origin in plan, so the loop forces the fan; on `z = 0`.
    let c = Point3::new(5.0, 3.0, 0.0);
    let mut topo = Topology::new();
    let face = planar_face(
        &mut topo,
        square(c, 1.0, [None, None, None, None]),
        vec![],
        0.0,
    );
    let reference = Point3::new(2.0_f64.powi(54), 0.5, -1e6);
    let read = reading(&topo, face, reference);
    let exact = 1e6 / 3.0;
    let error = (read.contribution.volume - exact).abs();
    let bound = certificate(&read).volume_bound(1e6, (c - reference).length(), 8.0);
    assert!(
        error <= bound,
        "the fan term is {:.6e} against {exact:.6e}, bound {bound:.3e}",
        read.contribution.volume
    );
}

/// Re-check finding 2: a face translated by `2^20` and widened by `2^-28`
/// still matches an anchored cache entry, but its fan reading is another
/// one. A fan-read face is served only from an identical entry about the
/// same reference, so the cached reading is the uncached one bit for bit.
#[test]
fn a_fan_read_face_is_never_served_from_a_translated_entry() {
    use crate::properties::face_cache::FaceIntegralCache;

    let c = Point3::new(5.0, 3.0, 1.0);
    let shift = Vec3::new(2.0_f64.powi(20), 0.0, 0.0);
    let mut topo = Topology::new();
    let source = planar_face(
        &mut topo,
        square(c, 1.0, [None, None, None, None]),
        vec![],
        1.0,
    );
    let copy = planar_face(
        &mut topo,
        square(c + shift, 1.0 + 2.0_f64.powi(-28), [None, None, None, None]),
        vec![],
        1.0,
    );
    let reference = Point3::new(1.0, -2.0, 3.0);
    let mut cache = FaceIntegralCache::new();
    cache
        .integrate_face_volume_about(&topo, source, 5, reference)
        .unwrap();
    let warm = cache
        .integrate_face_volume_about(&topo, copy, 5, reference)
        .unwrap();
    let cold = FaceIntegralCache::disabled()
        .integrate_face_volume_about(&topo, copy, 5, reference)
        .unwrap();
    assert!(cold.planar_fan.is_some(), "the copy is read by the fan");
    assert_eq!(warm.volume.to_bits(), cold.volume.to_bits());
    assert_eq!(warm.area.to_bits(), cold.area.to_bits());
    assert_eq!(warm.planar_fan, cold.planar_fan);
    let stats = cache.stats();
    assert_eq!((stats.translated, stats.rereferenced), (0, 0));
    // About another reference, the source itself is integrated afresh too.
    let again = cache
        .integrate_face_volume_about(&topo, source, 5, Point3::new(-7.0, 0.5, 2.0))
        .unwrap();
    let fresh = FaceIntegralCache::disabled()
        .integrate_face_volume_about(&topo, source, 5, Point3::new(-7.0, 0.5, 2.0))
        .unwrap();
    assert_eq!(again.volume.to_bits(), fresh.volume.to_bits());
    assert_eq!(cache.stats().rereferenced, 0);
}

/// Re-check finding 3: an open degree-one NURBS edge trimmed to
/// `[−2^53, 1]`. `1 − (−2^53)` rounds to `2^53`, so the fan's last
/// parameter is `0`, not `1`: the samples never reach the detour on
/// `[0, 1]`. A trim whose parameters cannot be sampled faithfully declines.
#[test]
fn a_trim_too_wide_to_sample_faithfully_declines() {
    let c = Point3::new(5.0, 3.0, 1.0);
    let at = |x: f64, y: f64| Point3::new(c.x() + x, c.y() + y, c.z());
    let low = -(2.0_f64.powi(53));
    let detour = NurbsCurve::new(
        1,
        vec![low, low, 0.0, 0.5, 1.0, 1.0],
        vec![
            at(0.0, 0.0),
            at(0.5, 0.0),
            at(0.75, 0.000_488),
            at(1.0, 0.0),
        ],
        vec![1.0; 4],
    )
    .unwrap();
    let mut topo = Topology::new();
    let face = planar_face(
        &mut topo,
        square(c, 1.0, [Some((detour, (low, 1.0))), None, None, None]),
        vec![],
        1.0,
    );
    let read = reading(&topo, face, Point3::new(0.0, 0.0, 0.0));
    // The detour bites ½ · 0.5 · 0.000488 out of the square.
    let error = (read.contribution.area - (1.0 - 1.22e-4)).abs();
    let bound = certificate(&read).area_bound;
    assert!(
        error <= bound,
        "the fan is {error:.3e} off, bound {bound:.3e}"
    );
    assert!(bound.is_infinite(), "an unfaithfully sampled trim declines");
}
