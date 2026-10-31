//! Pure-topology test assertions: Euler characteristic, manifoldness, point
//! distance and boolean volume conservation.
//!
//! Compiled for this crate's own tests and, behind the `test-utils` feature,
//! for downstream tests. Helpers that need an operation (measure, fill, fillet)
//! live with the modules that provide it (RFC 0007 stage 0b).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::explorer;
use remus_topology::solid::SolidId;

/// Assert that two points are within `abs_tol` of each other.
///
/// # Panics
///
/// Panics if the Euclidean distance exceeds `abs_tol`.
pub fn assert_point_near(actual: Point3, expected: Point3, abs_tol: f64) {
    let dx = actual.x() - expected.x();
    let dy = actual.y() - expected.y();
    let dz = actual.z() - expected.z();
    let dist = (dx * dx + dy * dy + dz * dz).sqrt();
    assert!(
        dist < abs_tol,
        "point mismatch: got ({:.6}, {:.6}, {:.6}), \
         expected ({:.6}, {:.6}, {:.6}), distance={dist:.2e}",
        actual.x(),
        actual.y(),
        actual.z(),
        expected.x(),
        expected.y(),
        expected.z(),
    );
}

/// Compute the Euler characteristic V - E + F for a solid.
///
/// For a closed orientable surface of genus g:
/// - genus 0 (sphere-like): χ = 2
/// - genus 1 (torus-like): χ = 0
/// - genus 2 (double torus): χ = -2
///
/// # Panics
///
/// Panics if topology lookups fail.
#[allow(clippy::cast_possible_wrap)]
#[must_use]
pub fn euler_characteristic(topo: &Topology, solid: SolidId) -> i64 {
    let (f, e, v) = explorer::solid_entity_counts(topo, solid).unwrap();
    (v as i64) - (e as i64) + (f as i64)
}

/// Assert that a solid has Euler characteristic 2 (genus-0, sphere-like).
///
/// This is the expected value for any simply-connected closed solid
/// (box, cylinder, cone, sphere, any convex solid, boolean result
/// of convex inputs without holes).
///
/// # Panics
///
/// Panics if the Euler characteristic is not 2.
pub fn assert_euler_genus0(topo: &Topology, solid: SolidId) {
    let chi = euler_characteristic(topo, solid);
    assert_eq!(
        chi, 2,
        "expected Euler characteristic V-E+F = 2 (genus-0), got {chi}"
    );
}

/// Assert that a solid's shell is manifold (every edge shared by exactly 2 faces).
///
/// # Panics
///
/// Panics if the solid is not manifold or if topology lookups fail.
pub fn assert_manifold(topo: &Topology, solid: SolidId) {
    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();
    remus_topology::validation::validate_shell_manifold(sh, topo)
        .expect("solid should be manifold");
}

/// Assert the inclusion-exclusion principle: V(A) + V(B) = V(A∪B) + V(A∩B).
///
/// This is a fundamental conservation law for boolean operations.
///
/// # Panics
///
/// Panics if the identity is violated beyond `rel_tol`.
pub fn assert_volume_conservation(
    vol_a: f64,
    vol_b: f64,
    vol_fused: f64,
    vol_intersected: f64,
    rel_tol: f64,
) {
    let lhs = vol_a + vol_b;
    let rhs = vol_fused + vol_intersected;
    let rel_error = if lhs.abs() < 1e-15 {
        rhs.abs()
    } else {
        (lhs - rhs).abs() / lhs.abs()
    };
    assert!(
        rel_error < rel_tol,
        "volume conservation violated: V(A)+V(B) = {lhs:.6}, \
         V(A∪B)+V(A∩B) = {rhs:.6} (error: {:.4}%, tolerance: {:.4}%)\n\
         V(A)={vol_a:.6}, V(B)={vol_b:.6}, V(A∪B)={vol_fused:.6}, V(A∩B)={vol_intersected:.6}",
        rel_error * 100.0,
        rel_tol * 100.0,
    );
}
