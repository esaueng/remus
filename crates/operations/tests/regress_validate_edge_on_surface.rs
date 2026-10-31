//! Validation reports an edge that leaves the surface of a face using it.
//!
//! `openzcad_e_nurbs_fillet_plate.step` bounds its parabolic corner bands
//! with straight chord edges that leave the bands by 1.06 at mid-span. With
//! no pcurve on those uses for SameParameter to compare, and both end
//! vertices on the band, both validators reported the solid clean.
//!
//! Here a box's top face is swapped for a flat bilinear NURBS patch lifted
//! by `h`: its four boundary edges stay at `z = 10`, exactly `h` off the
//! face. The allowance is the suite's `1e-4`; past
//! `EDGE_OFF_SURFACE_ERROR_FACTOR` (100) allowances the use is an error.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::validate::{CheckId, Severity, ValidateOptions};
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;
use remus_operations::primitives::make_box;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

/// A 20 x 20 x 10 box whose top face is the plane `z = 10 + lift`, as a
/// degree-(1, 1) NURBS patch.
fn lifted_top_box(lift: f64) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
    let top = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.z() > 0.999)
        })
        .unwrap();
    let z = 10.0 + lift;
    let control = vec![
        vec![Point3::new(0.0, 0.0, z), Point3::new(0.0, 20.0, z)],
        vec![Point3::new(20.0, 0.0, z), Point3::new(20.0, 20.0, z)],
    ];
    let knots = vec![0.0, 0.0, 1.0, 1.0];
    let patch =
        NurbsSurface::new(1, 1, knots.clone(), knots, control, vec![vec![1.0; 2]; 2]).unwrap();
    topo.face_mut(top)
        .unwrap()
        .set_surface(FaceSurface::Nurbs(patch));
    (topo, solid)
}

/// Severities of the check suite's edge-on-surface findings.
fn check_findings(topo: &Topology, solid: SolidId) -> Vec<Severity> {
    remus_check::validate::validate_solid(topo, solid, &ValidateOptions::default())
        .unwrap()
        .issues
        .iter()
        .filter(|issue| issue.check == CheckId::EdgeOnSurface)
        .map(|issue| issue.severity)
        .collect()
}

/// Severities of the operations validator's edge-on-surface findings.
fn operations_findings(
    topo: &Topology,
    solid: SolidId,
) -> Vec<remus_operations::validate::Severity> {
    remus_operations::validate::validate_solid(topo, solid)
        .unwrap()
        .issues
        .iter()
        .filter(|issue| issue.description.contains("edge leaves face"))
        .map(|issue| issue.severity)
        .collect()
}

#[test]
fn edges_on_their_faces_are_clean() {
    for lift in [0.0, 5e-5] {
        let (topo, solid) = lifted_top_box(lift);
        assert!(check_findings(&topo, solid).is_empty(), "lift {lift}");
        assert!(operations_findings(&topo, solid).is_empty(), "lift {lift}");
    }
}

#[test]
fn small_departures_warn() {
    for lift in [2e-4, 5e-3] {
        let (topo, solid) = lifted_top_box(lift);
        assert_eq!(
            check_findings(&topo, solid),
            vec![Severity::Warning; 4],
            "lift {lift}"
        );
        assert_eq!(
            operations_findings(&topo, solid),
            vec![remus_operations::validate::Severity::Warning; 4],
            "lift {lift}"
        );
        assert!(
            remus_operations::validate::validate_solid(&topo, solid)
                .unwrap()
                .is_valid()
        );
    }
}

#[test]
fn far_departures_are_errors() {
    for lift in [2e-2, 1.0] {
        let (topo, solid) = lifted_top_box(lift);
        assert_eq!(
            check_findings(&topo, solid),
            vec![Severity::Error; 4],
            "lift {lift}"
        );
        assert_eq!(
            operations_findings(&topo, solid),
            vec![remus_operations::validate::Severity::Error; 4],
            "lift {lift}"
        );
        assert!(
            !remus_operations::validate::validate_solid(&topo, solid)
                .unwrap()
                .is_valid()
        );
        let report =
            remus_check::validate::validate_solid(&topo, solid, &ValidateOptions::default())
                .unwrap();
        let deviation = report
            .issues
            .iter()
            .filter(|issue| issue.check == CheckId::EdgeOnSurface)
            .filter_map(|issue| issue.deviation)
            .fold(0.0, f64::max);
        assert!(
            (deviation - lift).abs() <= 1e-9 * (1.0 + lift),
            "lift {lift}: {deviation}"
        );
    }
}
