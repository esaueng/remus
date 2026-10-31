//! Test assertions that measure volumes and areas (RFC 0007 stage 0b split
//! of the old `test_helpers`).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, dead_code)]

use remus_topology::Topology;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

/// Assert that a solid's volume is within `rel_tol` of `expected`.
///
/// When `expected` is near zero (< 1e-15), `rel_tol` is treated as an
/// absolute tolerance instead (i.e., asserts `|volume| < rel_tol`).
///
/// # Panics
///
/// Panics if the relative error exceeds `rel_tol` or if volume
/// computation fails.
pub fn assert_volume_near(topo: &Topology, solid: SolidId, expected: f64, rel_tol: f64) {
    let vol = crate::measure::solid_volume(topo, solid, 0.05).unwrap();
    let rel_error = if expected.abs() < 1e-15 {
        vol.abs()
    } else {
        (vol - expected).abs() / expected.abs()
    };
    assert!(
        rel_error < rel_tol,
        "volume mismatch: got {vol:.6}, expected {expected:.6} \
         (error: {:.4}%, tolerance: {:.4}%)",
        rel_error * 100.0,
        rel_tol * 100.0,
    );
}

/// Assert that a face's area is within `rel_tol` of `expected`.
///
/// # Panics
///
/// Panics if the relative error exceeds `rel_tol` or if area
/// computation fails.
pub fn assert_area_near(topo: &Topology, face: FaceId, expected: f64, rel_tol: f64) {
    let area = crate::measure::face_area(topo, face, 0.1).unwrap();
    let rel_error = if expected.abs() < 1e-15 {
        area.abs()
    } else {
        (area - expected).abs() / expected.abs()
    };
    assert!(
        rel_error < rel_tol,
        "area mismatch: got {area:.6}, expected {expected:.6} \
         (error: {:.4}%, tolerance: {:.4}%)",
        rel_error * 100.0,
        rel_tol * 100.0,
    );
}

/// Assert that a CW-wound profile produces a solid with the expected volume.
///
/// Creates a CW unit square face, passes it to `build_solid`, and asserts
/// the resulting volume is within `rel_tol` of `expected_vol`.
///
/// # Panics
///
/// Panics if the volume deviates beyond `rel_tol` or if any operation fails.
pub fn assert_cw_profile_produces_valid_solid<F>(build_solid: F, expected_vol: f64, rel_tol: f64)
where
    F: Fn(&mut Topology, FaceId) -> SolidId,
{
    let mut topo = Topology::new();
    let face = remus_topology::test_utils::make_cw_unit_square_face(&mut topo);
    let solid = build_solid(&mut topo, face);
    assert_volume_near(&topo, solid, expected_vol, rel_tol);
}
