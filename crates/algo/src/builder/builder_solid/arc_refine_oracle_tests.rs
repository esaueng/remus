//! Differential oracle for the knot-span box prefilter of the NURBS arc
//! refinement in `split_arc_edges_at_collinear_vertices`: every vertex the
//! prefilter rejects is one the unchanged sampled search rejects too, so the
//! accepted cuts are the same with and without it.
//!
//! The corpus is random clamped NURBS curves (degrees 1 to 5, rational and
//! polynomial, repeated interior knots, three coordinate scales) with forward,
//! partial and reversed trims, probed at points on the curve, at offsets that
//! straddle the snap distance and the prefilter's `2 snap` threshold, at the
//! control points and anywhere in the curve's box.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]

use remus_math::nurbs::curve::NurbsCurve;

use super::{EdgeCurve, Point3, Vec3, box_distance, nurbs_cut_at, nurbs_span_boxes};

/// The refinement's snap distance (ten merge tolerances).
const SNAP: f64 = 1e-6;

/// Xorshift64*, deterministic across platforms.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1_u64 << 53) as f64
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        (hi - lo).mul_add(self.next(), lo)
    }

    fn below(&mut self, n: usize) -> usize {
        ((self.next() * n as f64) as usize).min(n - 1)
    }

    fn unit(&mut self) -> Vec3 {
        loop {
            let v = Vec3::new(
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
            );
            if let Ok(u) = v.normalize() {
                return u;
            }
        }
    }
}

fn random_curve(rng: &mut Rng, scale: f64) -> NurbsCurve {
    let degree = 1 + rng.below(5);
    let n = degree + 1 + rng.below(9);
    let (a, b) = {
        let a = rng.range(-3.0, 3.0);
        (a, a + rng.range(0.1, 10.0))
    };
    let mut interior: Vec<f64> = (0..n - degree - 1).map(|_| rng.range(a, b)).collect();
    // Repeat some interior knots, up to the degree.
    for k in 0..interior.len() {
        if k > 0 && rng.next() < 0.25 {
            interior[k] = interior[k - 1];
        }
    }
    interior.sort_by(f64::total_cmp);
    let mut knots = vec![a; degree + 1];
    knots.extend(interior);
    knots.extend(vec![b; degree + 1]);
    let mut point = Point3::new(
        rng.range(-50.0, 50.0) * scale,
        rng.range(-50.0, 50.0) * scale,
        rng.range(-50.0, 50.0) * scale,
    );
    let points: Vec<Point3> = (0..n)
        .map(|_| {
            point = point + rng.unit() * (rng.range(0.1, 5.0) * scale);
            point
        })
        .collect();
    let rational = rng.next() < 0.6;
    let weights: Vec<f64> = (0..n)
        .map(|_| if rational { rng.range(0.2, 5.0) } else { 1.0 })
        .collect();
    NurbsCurve::new(degree, knots, points, weights).unwrap()
}

#[test]
fn span_box_prefilter_rejects_only_vertices_the_search_rejects() {
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    let (mut probes, mut rejected, mut accepted, mut accepted_near_threshold) = (0, 0, 0, 0);
    for scale in [1e-3, 1.0, 1e3] {
        for _ in 0..150 {
            let nurbs = random_curve(&mut rng, scale);
            let boxes = nurbs_span_boxes(&nurbs).expect("a clamped positive-weight curve");
            let (a, b) = nurbs.domain();
            let curve = EdgeCurve::NurbsCurve(nurbs.clone());
            for (t0, t1) in [
                (a, b),
                ((b - a).mul_add(0.2, a), (b - a).mul_add(0.7, a)),
                (b, a),
            ] {
                let span = t1 - t0;
                let (sp, ep) = (nurbs.evaluate(t0), nurbs.evaluate(t1));
                let samples: Vec<Point3> = (0..=32_usize)
                    .map(|k| {
                        curve.evaluate_with_endpoints(span.mul_add(k as f64 / 32.0, t0), sp, ep)
                    })
                    .collect();
                let mut candidates: Vec<Point3> = nurbs.control_points().to_vec();
                for _ in 0..40 {
                    let on = nurbs.evaluate(rng.range(a, b));
                    let offset = [0.0, 0.3, 0.9, 1.0, 1.1, 1.9, 2.0, 2.1, 3.0, 10.0, 1e3, 1e6]
                        [rng.below(12)]
                        * SNAP;
                    candidates.push(on + rng.unit() * offset);
                }
                let (lo, hi) = boxes.iter().fold((boxes[0].0, boxes[0].1), |(lo, hi), b| {
                    (
                        Point3::new(
                            lo.x().min(b.0.x()),
                            lo.y().min(b.0.y()),
                            lo.z().min(b.0.z()),
                        ),
                        Point3::new(
                            hi.x().max(b.1.x()),
                            hi.y().max(b.1.y()),
                            hi.z().max(b.1.z()),
                        ),
                    )
                });
                for _ in 0..40 {
                    candidates.push(Point3::new(
                        rng.range(lo.x(), hi.x()),
                        rng.range(lo.y(), hi.y()),
                        rng.range(lo.z(), hi.z()),
                    ));
                }
                for p in candidates {
                    probes += 1;
                    let reference = nurbs_cut_at(&curve, (sp, ep), (t0, span), &samples, p, SNAP);
                    let skip = boxes.iter().all(|aabb| box_distance(p, aabb) > 2.0 * SNAP);
                    if skip {
                        rejected += 1;
                        assert_eq!(
                            reference, None,
                            "prefilter rejected a cut: {nurbs:?} trim ({t0}, {t1}) at {p:?}"
                        );
                    }
                    if reference.is_some() {
                        accepted += 1;
                        let nearest = boxes
                            .iter()
                            .map(|aabb| box_distance(p, aabb))
                            .fold(f64::INFINITY, f64::min);
                        accepted_near_threshold += usize::from(nearest > 0.5 * SNAP);
                    }
                }
            }
        }
    }
    // Both outcomes occur often, including accepted cuts outside every box.
    assert!(
        rejected * 6 > probes && accepted > 1000 && accepted_near_threshold > 0,
        "{rejected} rejected, {accepted} accepted ({accepted_near_threshold} near) of {probes}"
    );
}

#[test]
fn span_boxes_refuse_curves_they_cannot_bound() {
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 2.0, 0.0),
        Point3::new(3.0, -1.0, 1.0),
        Point3::new(4.0, 0.0, 0.0),
    ];
    let clamped = vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0];
    let curve = |weights: Vec<f64>| NurbsCurve::new(2, clamped.clone(), points.clone(), weights);
    assert!(nurbs_span_boxes(&curve(vec![1.0, 0.5, 2.0, 1.0]).unwrap()).is_some());
    for weights in [
        vec![1.0, 0.0, 2.0, 1.0],
        vec![1.0, -0.5, 2.0, 1.0],
        vec![1.0, f64::INFINITY, 2.0, 1.0],
    ] {
        if let Ok(c) = curve(weights) {
            assert!(nurbs_span_boxes(&c).is_none(), "{c:?}");
        }
    }
    // Empty first or last span: evaluation clamps the domain ends into it.
    for knots in [
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
    ] {
        if let Ok(c) = NurbsCurve::new(2, knots, points.clone(), vec![1.0; 4]) {
            assert!(nurbs_span_boxes(&c).is_none(), "{c:?}");
        }
    }
    let mut far = points.clone();
    far[1] = Point3::new(f64::NAN, 0.0, 0.0);
    if let Ok(c) = NurbsCurve::new(2, clamped, far, vec![1.0; 4]) {
        assert!(nurbs_span_boxes(&c).is_none(), "{c:?}");
    }
}
