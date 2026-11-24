//! Triangulate a trimmed cylinder between two angularly monotone rim chains.
use remus_math::{surfaces::CylindricalSurface, vec::Point3};
use remus_topology::edge::EdgeId;

/// March around the cylinder in increasing angle. Consecutive cross-strip
/// edges then bound one triangle at a time. Reject reversed cells, wide
/// chords and off-carrier vertices before publishing any indices.
pub(super) fn triangles(
    cylinder: &CylindricalSurface,
    uv: &[(f64, f64)],
    samples: &[(Point3, u32, EdgeId, bool)],
    deflection: f64,
    angular_tol: f64,
) -> Option<Vec<u32>> {
    let n = uv.len();
    if n < 4 || n != samples.len() {
        return None;
    }
    let start = (0..n).min_by(|&a, &b| uv[a].0.total_cmp(&uv[b].0))?;
    let end = (0..n).max_by(|&a, &b| uv[a].0.total_cmp(&uv[b].0))?;
    let sweep = uv[end].0 - uv[start].0;
    if !sweep.is_finite() || sweep >= std::f64::consts::TAU {
        return None;
    }
    let residual = uv
        .iter()
        .zip(samples)
        .map(|(&(u, v), s)| (cylinder.evaluate(u, v) - s.0).length())
        .try_fold(0.0_f64, |acc, d| d.is_finite().then(|| acc.max(d)))?;
    let mut result = Vec::with_capacity((n - 2) * 3);
    let (mut left, mut right) = ((start + 1) % n, (start + n - 1) % n);
    if left == end || right == end {
        return None;
    }
    let mut orientation = None;
    for index in 0..n - 2 {
        let next_l = (left + 1) % n;
        let next_r = (right + n - 1) % n;
        if uv[next_l].0 <= uv[left].0 || uv[next_r].0 <= uv[right].0 {
            return None;
        }
        let tri = if index == 0 {
            [start, left, right]
        } else if next_l == end && next_r == end {
            [left, end, right]
        } else if next_r == end || (next_l != end && uv[next_l].0 <= uv[next_r].0) {
            let tri = [left, next_l, right];
            left = next_l;
            tri
        } else {
            let tri = [left, next_r, right];
            right = next_r;
            tri
        };
        let [i, j, k] = tri;
        let [(u0, v0), (u1, v1), (u2, v2)] = tri.map(|i| uv[i]);
        let span = u0.max(u1).max(u2) - u0.min(u1).min(u2);
        // Every convex combination of these three cylinder points lies
        // between the circle and its outermost chord in this angular wedge.
        let sag = 2.0 * cylinder.radius() * (span / 4.0).sin().powi(2);
        if !(span < std::f64::consts::PI && span <= angular_tol && sag + residual <= deflection) {
            return None;
        }
        let cross = (u1 - u0) * (v2 - v0) - (u2 - u0) * (v1 - v0);
        if !cross.is_finite()
            || cross == 0.0
            || *orientation.get_or_insert(cross > 0.0) != (cross > 0.0)
        {
            return None;
        }
        let [a, b, c] = tri.map(|i| samples[i].0);
        let normal = (b - a).cross(c - a);
        let oriented = if cross > 0.0 { normal } else { -normal };
        // The angular tolerance bounds the carrier's turn across a segment
        // (the span above). A sliver at a shared trim must still face out.
        if [(u0, v0), (u1, v1), (u2, v2)].into_iter().any(|(u, v)| {
            let outward = oriented.dot(cylinder.normal(u, v));
            !outward.is_finite() || outward <= 0.0
        }) {
            return None;
        }
        result.extend((if cross > 0.0 { [i, j, k] } else { [i, k, j] }).map(|i| samples[i].1));
    }
    (result.len() == (n - 2) * 3).then_some(result)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use remus_math::vec::Vec3;
    use remus_topology::{Topology, builder::make_line_edge};
    use std::collections::BTreeMap;

    const RIM: [(f64, f64); 7] = [
        (0.0, 0.0),
        (0.05, -0.1),
        (0.10, -0.06),
        (0.18, -0.02),
        (0.24, 0.10),
        (0.16, 0.19),
        (0.06, 0.14),
    ];

    fn samples(
        cylinder: &CylindricalSurface,
        uv: &[(f64, f64)],
    ) -> Vec<(Point3, u32, EdgeId, bool)> {
        let edge = make_line_edge(
            &mut Topology::new(),
            Point3::new(0., 0., 0.),
            Point3::new(1., 0., 0.),
            1e-7,
        )
        .unwrap();
        uv.iter()
            .enumerate()
            .map(|(i, &(u, v))| (cylinder.evaluate(u, v), i as u32, edge, false))
            .collect()
    }

    #[test]
    fn uses_each_rim_segment_once_and_shared_diagonals_twice() {
        let cylinder =
            CylindricalSurface::new(Point3::new(0., 0., 0.), Vec3::new(0., 0., 1.), 2.).unwrap();
        for reverse in [false, true] {
            let mut uv = RIM;
            if reverse {
                uv.reverse();
            }
            uv.rotate_left(2);
            let samples = samples(&cylinder, &uv);
            let indices = triangles(&cylinder, &uv, &samples, 0.01, 0.2).unwrap();
            assert_eq!(indices.len(), 3 * (uv.len() - 2));
            let mut counts = BTreeMap::new();
            for triangle in indices.chunks_exact(3) {
                for i in 0..3 {
                    let (a, b) = (triangle[i], triangle[(i + 1) % 3]);
                    *counts.entry((a.min(b), a.max(b))).or_insert(0) += 1;
                }
            }
            for i in 0..uv.len() as u32 {
                let j = (i + 1) % uv.len() as u32;
                assert_eq!(counts.remove(&(i.min(j), i.max(j))), Some(1));
            }
            assert!(counts.values().all(|&n| n == 2));
        }
    }

    #[test]
    fn rejects_over_tolerance_chords_off_surface_samples_and_folded_cells() {
        let cylinder =
            CylindricalSurface::new(Point3::new(0., 0., 0.), Vec3::new(0., 0., 1.), 2.).unwrap();
        let mut rim = samples(&cylinder, &RIM);
        assert!(triangles(&cylinder, &RIM, &rim, 0.00001, 0.2).is_none());
        assert!(triangles(&cylinder, &RIM, &rim, 0.01, 0.01).is_none());
        rim[2].0 = rim[2].0 + Vec3::new(0., 0., 0.02);
        assert!(triangles(&cylinder, &RIM, &rim, 0.01, 0.2).is_none());
        let mut folded = RIM;
        folded[2].1 = 0.5;
        assert!(triangles(&cylinder, &folded, &samples(&cylinder, &folded), 0.01, 0.2).is_none());
    }
}
