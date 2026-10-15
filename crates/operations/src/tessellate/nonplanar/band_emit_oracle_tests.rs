//! The band emitter's per-vertex outward reference returns what projecting
//! every triangle corner did: the pre-cache emitter, kept verbatim, is the
//! oracle on random triangle soups over a sphere, including a pool truncated
//! and refilled with other points under the same indices.
#![allow(clippy::unwrap_used, clippy::cast_precision_loss)]

use remus_math::surfaces::SphericalSurface;
use remus_math::vec::{Point3, Vec3};

use super::{TriangleMesh, make_band_emit};

/// `make_band_emit` before the per-vertex cache, verbatim.
fn reference_band_emit<'a>(
    project: &'a dyn Fn(Point3) -> (f64, f64),
    surf_normal: &'a dyn Fn(f64, f64) -> Vec3,
) -> impl Fn(&mut TriangleMesh, u32, u32, u32) + 'a {
    move |merged: &mut TriangleMesh, a: u32, b: u32, c: u32| {
        if a == b || b == c || a == c {
            return;
        }
        let (pa, pb, pc) = (
            merged.positions[a as usize],
            merged.positions[b as usize],
            merged.positions[c as usize],
        );
        let geo = (pb - pa).cross(pc - pa);
        if geo.length() < 1e-20 {
            return;
        }
        let n_at = |p: Point3| -> Vec3 {
            let (u, v) = project(p);
            surf_normal(u, v)
        };
        let outward = n_at(pa) + n_at(pb) + n_at(pc);
        let mut tri = [a, b, c];
        if geo.dot(outward) < 0.0 {
            tri.swap(1, 2);
        }
        merged.indices.extend_from_slice(&tri);
    }
}

#[test]
fn cached_band_emit_matches_the_projecting_emitter() {
    let sphere = SphericalSurface::new(Point3::new(0.3, -1.2, 2.0), 7.0).unwrap();
    let project = |p| sphere.project_point(p);
    let normal = |u, v| sphere.normal(u, v);
    let mut state = 0x9e37_79b9_7f4a_7c15_u64;
    let mut next = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    let point = |next: &mut dyn FnMut() -> u64| {
        let u = (next() % 10_000) as f64 / 10_000.0 * std::f64::consts::TAU;
        let v = ((next() % 10_000) as f64 / 10_000.0 - 0.5) * std::f64::consts::PI;
        sphere.evaluate(u, v)
    };
    let cached = make_band_emit(&project, &normal);
    let mut cached_mesh = TriangleMesh::default();
    let mut reference_mesh = TriangleMesh::default();
    for round in 0..4 {
        // Truncate and refill: the same indices now name other points.
        let keep = cached_mesh.positions.len() / 2;
        cached_mesh.positions.truncate(keep);
        reference_mesh.positions.truncate(keep);
        for _ in 0..64 {
            let p = point(&mut next);
            cached_mesh.positions.push(p);
            reference_mesh.positions.push(p);
        }
        let reference = reference_band_emit(&project, &normal);
        let pool = cached_mesh.positions.len() as u64;
        for _ in 0..2_000 {
            let [a, b, c] = [next() % pool, next() % pool, next() % pool].map(|i| i as u32);
            cached(&mut cached_mesh, a, b, c);
            reference(&mut reference_mesh, a, b, c);
        }
        assert_eq!(
            cached_mesh.indices, reference_mesh.indices,
            "round {round}: emitted triangles differ"
        );
    }
    assert!(cached_mesh.indices.len() > 3 * 4_000);
}
