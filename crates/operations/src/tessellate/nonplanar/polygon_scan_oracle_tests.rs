//! Differential oracle for the boundary scans of the CDT mesher's interior
//! grid: [`point_in_polygon_2d`] and [`classify_dense_candidate`] as they
//! indexed each segment's end by `(i + 1) % n`, kept verbatim, against the
//! versions walking the same segments as pairs. Containment and clearance
//! must agree on every probe of polygons with collinear, duplicate,
//! horizontal and degenerate edges, both orientations, and points on and
//! around their vertices and edges.

#![allow(clippy::cast_precision_loss, clippy::float_cmp)]

use remus_math::vec::Point2;

use super::{classify_dense_candidate, point_in_polygon_2d};

fn legacy_point_in_polygon_2d(polygon: &[(f64, f64)], pt: remus_math::vec::Point2) -> bool {
    let n = polygon.len();
    let mut winding = 0i32;
    for i in 0..n {
        let j = (i + 1) % n;
        let yi = polygon[i].1;
        let yj = polygon[j].1;
        if yi <= pt.y() {
            if yj > pt.y() {
                let cross = (polygon[j].0 - polygon[i].0) * (pt.y() - yi)
                    - (pt.x() - polygon[i].0) * (yj - yi);
                if cross > 0.0 {
                    winding += 1;
                }
            }
        } else if yj <= pt.y() {
            let cross =
                (polygon[j].0 - polygon[i].0) * (pt.y() - yi) - (pt.x() - polygon[i].0) * (yj - yi);
            if cross < 0.0 {
                winding -= 1;
            }
        }
    }
    winding != 0
}

fn legacy_classify_dense_candidate(
    boundary_uv: &[(f64, f64)],
    boundary_cdt: &[remus_math::vec::Point2],
    u: f64,
    v: f64,
    point: remus_math::vec::Point2,
    clearance: f64,
) -> (bool, bool) {
    // `boundary_cdt` is mapped from `boundary_uv` one-to-one at the call
    // site, so both index ranges coincide.
    let mut winding = 0i32;
    let mut clear = true;
    for i in 0..boundary_uv.len() {
        let j = (i + 1) % boundary_uv.len();
        let yi = boundary_uv[i].1;
        let yj = boundary_uv[j].1;
        if yi <= v {
            if yj > v {
                let cross = (boundary_uv[j].0 - boundary_uv[i].0) * (v - yi)
                    - (u - boundary_uv[i].0) * (yj - yi);
                if cross > 0.0 {
                    winding += 1;
                }
            }
        } else if yj <= v {
            let cross = (boundary_uv[j].0 - boundary_uv[i].0) * (v - yi)
                - (u - boundary_uv[i].0) * (yj - yi);
            if cross < 0.0 {
                winding -= 1;
            }
        }
        let a = boundary_cdt[i];
        let b = boundary_cdt[j];
        let ab = b - a;
        let length_squared = ab.dot(ab);
        let t = if length_squared > 1.0e-30 {
            ((point - a).dot(ab) / length_squared).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let foot = remus_math::vec::Point2::new(a.x() + ab.x() * t, a.y() + ab.y() * t);
        clear = (point - foot).length() > clearance && clear;
    }
    (winding != 0, clear)
}

struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

fn polygons() -> Vec<Vec<(f64, f64)>> {
    let mut out = vec![
        Vec::new(),
        vec![(0.0, 0.0)],
        vec![(0.0, 0.0), (1.0, 1.0)],
        vec![(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)],
        vec![(0.0, 1.0), (2.0, 1.0), (2.0, 0.0), (0.0, 0.0)],
        // A notch with collinear, duplicate and horizontal runs.
        vec![
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
            (2.0, 1.0),
            (1.5, 1.0),
            (1.5, 0.5),
            (0.5, 0.5),
            (0.5, 1.0),
            (0.0, 1.0),
            (0.0, 0.5),
        ],
        // A bow tie, whose crossing makes winding signs cancel.
        vec![(0.0, 0.0), (1.0, 1.0), (1.0, 0.0), (0.0, 1.0)],
    ];
    let mut rng = Lcg(17);
    for n in [5, 12, 40, 97] {
        // A star-shaped polygon with jittered radii, in both orientations.
        let star: Vec<(f64, f64)> = (0..n)
            .map(|k| {
                let t = std::f64::consts::TAU * k as f64 / n as f64;
                let r = 0.5 + rng.next();
                (r * t.cos() + 3.0, r * t.sin() - 1.0)
            })
            .collect();
        let mut reversed = star.clone();
        reversed.reverse();
        out.push(star);
        out.push(reversed);
    }
    out
}

fn probes(polygon: &[(f64, f64)], rng: &mut Lcg) -> Vec<(f64, f64)> {
    let mut out: Vec<(f64, f64)> = polygon.to_vec();
    for pair in polygon.windows(2) {
        let ((x0, y0), (x1, y1)) = (pair[0], pair[1]);
        out.push((f64::midpoint(x0, x1), f64::midpoint(y0, y1)));
        out.push((f64::midpoint(x0, x1), y0));
    }
    let (lo_x, hi_x, lo_y, hi_y) = polygon.iter().fold(
        (-1.0_f64, 1.0_f64, -1.0_f64, 1.0_f64),
        |(lo_x, hi_x, lo_y, hi_y), &(x, y)| (lo_x.min(x), hi_x.max(x), lo_y.min(y), hi_y.max(y)),
    );
    for _ in 0..400 {
        out.push((
            (hi_x - lo_x + 0.5).mul_add(rng.next(), lo_x - 0.25),
            (hi_y - lo_y + 0.5).mul_add(rng.next(), lo_y - 0.25),
        ));
    }
    out
}

#[test]
fn segment_pairs_scan_like_the_indexed_loops() {
    let mut rng = Lcg(5);
    let mut compared = 0;
    for polygon in polygons() {
        for scale in [1.0, std::f64::consts::TAU / 6.0] {
            let boundary_cdt: Vec<Point2> = polygon
                .iter()
                .map(|&(u, v)| Point2::new(u, v * scale))
                .collect();
            for (u, v) in probes(&polygon, &mut rng) {
                let point = Point2::new(u, v * scale);
                assert_eq!(
                    point_in_polygon_2d(&polygon, Point2::new(u, v)),
                    legacy_point_in_polygon_2d(&polygon, Point2::new(u, v)),
                    "containment at ({u}, {v}) in {polygon:?}"
                );
                for clearance in [0.0, 0.01, 0.2] {
                    assert_eq!(
                        classify_dense_candidate(&polygon, &boundary_cdt, u, v, point, clearance),
                        legacy_classify_dense_candidate(
                            &polygon,
                            &boundary_cdt,
                            u,
                            v,
                            point,
                            clearance
                        ),
                        "classification at ({u}, {v}) in {polygon:?}"
                    );
                }
                compared += 1;
            }
        }
    }
    assert!(compared > 10_000, "{compared} probes");
}
