//! Bit-identity oracle for the NURBS edge sizing measures
//! (`measure_max_chord_deviation`, `measure_max_segment_turn`): the versions
//! that evaluated both ends of every segment, kept verbatim, against the
//! ones that carry each segment's end over to the next. Compared on every
//! NURBS edge of a thread, an in-place helical sweep, a helical gear and a
//! cross-drilled shaft at the segment counts the sampler uses, with the
//! evaluation counts pinned.

#![allow(clippy::unwrap_used, clippy::cast_precision_loss)]

use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;

use super::{
    max_chord_deviation_by, max_segment_turn_by, measure_max_chord_deviation,
    measure_max_segment_turn,
};
use crate::tessellate::swept_fixtures as fixtures;

/// `measure_max_chord_deviation` before it carried segment ends over.
fn legacy_max_chord_deviation(nurbs: &NurbsCurve, u0: f64, u1: f64, n: usize) -> f64 {
    let mut max_dev: f64 = 0.0;
    for i in 0..n {
        let t0 = u0 + (u1 - u0) * (i as f64) / (n as f64);
        let t1 = u0 + (u1 - u0) * ((i + 1) as f64) / (n as f64);
        let p0 = nurbs.evaluate(t0);
        let p1 = nurbs.evaluate(t1);
        let mid_chord = Point3::new(
            (p0.x() + p1.x()) * 0.5,
            (p0.y() + p1.y()) * 0.5,
            (p0.z() + p1.z()) * 0.5,
        );
        let mid_curve = nurbs.evaluate((t0 + t1) * 0.5);
        let dev = (mid_curve - mid_chord).length();
        max_dev = max_dev.max(dev);
    }
    max_dev
}

/// `measure_max_segment_turn` before it carried segment ends over.
fn legacy_max_segment_turn(nurbs: &NurbsCurve, u0: f64, u1: f64, n: usize) -> f64 {
    let mut max_turn: f64 = 0.0;
    for i in 0..n {
        let t0 = u0 + (u1 - u0) * (i as f64) / (n as f64);
        let t1 = u0 + (u1 - u0) * ((i + 1) as f64) / (n as f64);
        if let (Ok(a), Ok(b)) = (nurbs.tangent(t0), nurbs.tangent(t1)) {
            let dot = a.dot(b).clamp(-1.0, 1.0);
            max_turn = max_turn.max(dot.acos());
        }
    }
    max_turn
}

/// Every NURBS edge curve of the fixtures, with its edge's own domain.
fn nurbs_edges() -> Vec<(NurbsCurve, (f64, f64))> {
    let mut topo = Topology::new();
    let solids = [
        fixtures::external_thread(&mut topo, 2.0),
        fixtures::coil(&mut topo, 3.0),
        fixtures::helical_gear(&mut topo),
        fixtures::cross_drilled_shaft(&mut topo),
    ];
    let mut edges = Vec::new();
    for solid in solids {
        for edge in remus_topology::explorer::solid_edges(&topo, solid).unwrap() {
            let edge = topo.edge(edge).unwrap();
            if let EdgeCurve::NurbsCurve(curve) = edge.curve() {
                let domain = crate::authoritative_edge_domain(edge, "oracle").unwrap();
                edges.push((curve.clone(), domain));
            }
        }
    }
    edges
}

#[test]
fn carried_ends_measure_the_same_bits() {
    let edges = nurbs_edges();
    assert!(edges.len() > 100, "{} NURBS edges", edges.len());
    let (mut bent, mut turned) = (0, 0);
    for (curve, (u0, u1)) in &edges {
        // The sampler's coarse count for this edge, its clamp ends and the
        // degenerate counts.
        let spans = curve
            .control_points()
            .len()
            .saturating_sub(curve.degree())
            .max(1);
        for n in [0, 1, 2, 7, 8, 128, (spans * 4).clamp(8, 128)] {
            // Both directions: a section edge can run its curve backwards.
            for (a, b) in [(*u0, *u1), (*u1, *u0)] {
                let deviation = measure_max_chord_deviation(curve, a, b, n);
                let turn = measure_max_segment_turn(curve, a, b, n);
                assert_eq!(
                    deviation.to_bits(),
                    legacy_max_chord_deviation(curve, a, b, n).to_bits(),
                    "deviation over [{a}, {b}] in {n}"
                );
                assert_eq!(
                    turn.to_bits(),
                    legacy_max_segment_turn(curve, a, b, n).to_bits(),
                    "turn over [{a}, {b}] in {n}"
                );
                bent += usize::from(deviation > 1e-6);
                turned += usize::from(turn > 1e-3);
            }
        }
    }
    // The corpus bends: the measures are not comparing zeros.
    assert!(bent > 100 && turned > 100, "{bent} bent, {turned} turned");
}

#[test]
fn each_parameter_is_evaluated_once() {
    let (curve, (u0, u1)) = nurbs_edges()
        .into_iter()
        .max_by_key(|(curve, _)| curve.control_points().len())
        .unwrap();
    for n in [1_usize, 8, 37, 128] {
        // Two ends and a midpoint per segment took 3n evaluations.
        let mut evaluations = 0;
        let deviation = max_chord_deviation_by(
            |t| {
                evaluations += 1;
                curve.evaluate(t)
            },
            u0,
            u1,
            n,
        );
        assert_eq!(evaluations, 2 * n + 1);
        assert_eq!(
            deviation.to_bits(),
            legacy_max_chord_deviation(&curve, u0, u1, n).to_bits()
        );
        // Two ends per segment took 2n tangents.
        let mut tangents = 0;
        let turn = max_segment_turn_by(
            |t| {
                tangents += 1;
                curve.tangent(t).ok()
            },
            u0,
            u1,
            n,
        );
        assert_eq!(tangents, n + 1);
        assert_eq!(
            turn.to_bits(),
            legacy_max_segment_turn(&curve, u0, u1, n).to_bits()
        );
    }
    let never = |_: f64| -> Point3 { unreachable!("no segment, no evaluation") };
    assert_eq!(max_chord_deviation_by(never, u0, u1, 0).to_bits(), 0);
    assert_eq!(max_segment_turn_by(|_| None, u0, u1, 0).to_bits(), 0);
}
