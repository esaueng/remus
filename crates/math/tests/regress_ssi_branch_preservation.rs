//! Independent quadratic/plane sections expose parameter-vs-model dedup units.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
use remus_math::nurbs::intersection::{IntersectionCurve, intersect_nurbs_nurbs};
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;

fn pair(scale: f64) -> (NurbsSurface, NurbsSurface) {
    // Bernstein coefficients of (u - 1/4)(u - 3/4) on [0,1].
    let controls = [0.0, 0.5, 1.0]
        .into_iter()
        .zip([0.1875, -0.3125, 0.1875])
        .map(|(u, z)| {
            [0.0, 1.0]
                .into_iter()
                .map(|v| Point3::new(u * scale, v * scale, z * scale))
                .collect()
        })
        .collect();
    let surface = NurbsSurface::new(
        2,
        1,
        vec![0., 0., 0., 1., 1., 1.],
        vec![0., 0., 1., 1.],
        controls,
        vec![vec![1.; 2]; 3],
    )
    .unwrap();
    let controls = [0., 1.]
        .into_iter()
        .map(|u| {
            [0., 1.]
                .into_iter()
                .map(|v| Point3::new(u * scale, v * scale, 0.))
                .collect()
        })
        .collect();
    let plane = NurbsSurface::new(
        1,
        1,
        vec![0., 0., 1., 1.],
        vec![0., 0., 1., 1.],
        controls,
        vec![vec![1.; 2]; 2],
    )
    .unwrap();
    (surface, plane)
}

fn assert_two_lines(curves: &[IntersectionCurve], scale: f64, placed: bool) {
    assert_eq!(curves.len(), 2, "scale {scale}: component count");
    let mut represented = [false; 2];
    for section in curves {
        let first = local(section.points.first().unwrap().point, placed);
        let branch = usize::from(first.x() > scale * 0.5);
        assert!(!represented[branch], "scale {scale}: duplicate component");
        represented[branch] = true;
        let expected_x = [0.25, 0.75][branch] * scale;
        let (mut min_y, mut max_y) = (f64::INFINITY, f64::NEG_INFINITY);
        for sample in &section.points {
            let p = local(sample.point, placed);
            assert!(
                (p.x() - expected_x).abs() < 2e-6,
                "scale {scale}: wrong line"
            );
            assert!(p.z().abs() < 2e-6, "scale {scale}: off section plane");
            min_y = min_y.min(p.y());
            max_y = max_y.max(p.y());
        }
        // The existing marcher keeps a 0.1% boundary margin. Pin interior
        // coverage, not just seed presence; boundary endpoint recovery is separate.
        assert!(max_y - min_y > 0.98 * scale, "scale {scale}: partial line");
        // Positive rational weights and a control hull in the same exact line
        // bound the whole fitted curve, beyond sampled-point agreement.
        assert!(
            section
                .curve
                .weights()
                .iter()
                .all(|&w| w > 0.0 && w.is_finite())
        );
        for p in section.curve.control_points() {
            let p = local(*p, placed);
            assert!(
                (p.x() - expected_x).abs() < 2e-6,
                "scale {scale}: fit left line"
            );
            assert!(p.z().abs() < 2e-6, "scale {scale}: fit left plane");
            assert!(
                p.y() >= -2e-6 && p.y() <= scale + 2e-6,
                "scale {scale}: fit left bounded section"
            );
        }
    }
    assert!(represented.into_iter().all(|present| present));
}

#[test]
fn separate_quadratic_plane_sections_survive_model_scaling() {
    for scale in [1e-3, 1e-2, 0.1, 1., 1e3] {
        let (surface, plane) = pair(scale);
        let curves = intersect_nurbs_nurbs(&surface, &plane, 10, 0.02).unwrap();
        assert_two_lines(&curves, scale, false);
    }
}

fn local(p: Point3, placed: bool) -> Point3 {
    if !placed {
        return p;
    }
    let (x, y, z) = (p.x() - 37.0, p.y() + 91.0, p.z() - 13.0);
    Point3::new(0.6 * x + 0.8 * y, z, 0.8 * x - 0.6 * y)
}

fn placed(p: Point3) -> Point3 {
    Point3::new(
        37.0 + 0.6 * p.x() + 0.8 * p.z(),
        -91.0 + 0.8 * p.x() - 0.6 * p.z(),
        13.0 + p.y(),
    )
}

fn variant(s: &NurbsSurface, chart: usize, placement: bool) -> NurbsSurface {
    let mut cps = s.control_points().to_vec();
    let mut weights = s.weights().to_vec();
    let (mut du, mut dv) = (s.degree_u(), s.degree_v());
    let (mut ku, mut kv) = (s.knots_u().to_vec(), s.knots_v().to_vec());
    match chart {
        0 => {}
        1 => {
            for k in &mut ku {
                *k = 4.0 + *k * 0.125;
            }
            for k in &mut kv {
                *k = 10.0 + *k * 0.125;
            }
        }
        2 => {
            cps.reverse();
            weights.reverse();
            ku = ku.into_iter().rev().map(|k| 1.0 - k).collect();
        }
        3 => {
            cps = (0..cps[0].len())
                .map(|j| cps.iter().map(|r| r[j]).collect())
                .collect();
            weights = (0..weights[0].len())
                .map(|j| weights.iter().map(|r| r[j]).collect())
                .collect();
            std::mem::swap(&mut du, &mut dv);
            std::mem::swap(&mut ku, &mut kv);
        }
        _ => unreachable!(),
    }
    if placement {
        for row in &mut cps {
            for p in row {
                *p = placed(*p);
            }
        }
    }
    NurbsSurface::new(du, dv, ku, kv, cps, weights).unwrap()
}

#[test]
fn separate_sections_keep_identity_under_charts_placement_and_order() {
    for scale in [1e-3, 1e-2, 0.1, 1.0, 1e3] {
        let (surface, plane) = pair(scale);
        for chart in 0..4 {
            for placement in [false, true] {
                let a = variant(&surface, chart, placement);
                let b = variant(&plane, chart, placement);
                for swap in [false, true] {
                    let (first, second) = if swap { (&b, &a) } else { (&a, &b) };
                    for samples in [5, 20] {
                        for step_fraction in [0.002, 0.02] {
                            let step = step_fraction * if chart == 1 { 0.125 } else { 1.0 };
                            let curves =
                                intersect_nurbs_nurbs(first, second, samples, step).unwrap();
                            assert_eq!(
                                curves.len(),
                                2,
                                "scale={scale} chart={chart} placed={placement} swap={swap} samples={samples} step={step}"
                            );
                            assert_two_lines(&curves, scale, placement);
                            for curve in &curves {
                                for sample in &curve.points {
                                    assert!(
                                        (first.evaluate(sample.param1.0, sample.param1.1)
                                            - sample.point)
                                            .length()
                                            < 2e-6
                                    );
                                    assert!(
                                        (second.evaluate(sample.param2.0, sample.param2.1)
                                            - sample.point)
                                            .length()
                                            < 2e-6
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

// Dense tracing exercises repeated seeds across model scales, including the
// small-model straight sections that previously closed near their seed.
#[test]
fn dense_seeds_do_not_duplicate_the_two_sections() {
    for scale in [1e-3, 0.01, 1.0, 1e3] {
        let (a, b) = pair(scale);
        for samples in [5, 20] {
            let curves = intersect_nurbs_nurbs(&a, &b, samples, 0.002).unwrap();
            assert_two_lines(&curves, scale, false);
        }
    }
}
