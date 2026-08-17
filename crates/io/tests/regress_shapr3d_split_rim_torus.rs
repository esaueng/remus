//! Regression for HOOPS Exchange's split-rim periodic-face encoding.

#![allow(clippy::expect_used)]

use remus_io::step::reader::read_step;
use remus_math::tolerance::Tolerance;
use remus_operations::heal::merge_split_rim_arcs;
use remus_operations::measure::solid_volume;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid_with_tolerance,
};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;

const FOOT: &str = include_str!("data/shapr3d_walking_stick_foot.step");

#[test]
fn shapr3d_split_rim_tori_import_closed_at_every_quality() {
    let mut topo = Topology::new();
    let solids = read_step(FOOT, &mut topo).expect("import Shapr3D STEP");
    assert_eq!(solids.len(), 1, "fixture must contain one solid");
    let solid = solids[0];

    let report = validate_solid(&topo, solid).expect("validate imported solid");
    assert!(
        report.issues.is_empty(),
        "fixture should import without validation issues: {:?}",
        report.issues
    );

    let volume = solid_volume(&topo, solid, 0.05).expect("measure imported solid");
    let expected_volume = 32_364.901_7;
    assert!(
        (volume - expected_volume).abs() <= expected_volume * 0.001,
        "volume {volume} differs from measured reference {expected_volume}"
    );

    let faces = solid_faces(&topo, solid).expect("enumerate imported faces");
    assert_eq!(faces.len(), 11, "fixture face census changed");
    let torus_count = faces
        .iter()
        .filter(|&&face_id| {
            matches!(
                topo.face(face_id).expect("face").surface(),
                FaceSurface::Torus(_)
            )
        })
        .count();
    assert_eq!(torus_count, 3, "fixture torus census changed");

    // HOOPS gives the two faces sharing several of these rims different
    // periodic-seam attachment vertices. One closed EdgeId cannot represent
    // both attachments without opening a wire, so the conservative importer
    // preserves those cycles; a repeat pass must nevertheless be idempotent.
    assert_eq!(
        merge_split_rim_arcs(&mut topo, solid, Tolerance::new())
            .expect("repeat conservative rim canonicalization"),
        0,
        "STEP import should exhaust every safely mergeable split rim"
    );

    for (deflection, angular_degrees) in [(0.01_f64, 5.0_f64), (0.03, 8.0), (0.5, 15.0)] {
        let mesh =
            tessellate_solid_with_tolerance(&topo, solid, deflection, angular_degrees.to_radians())
                .expect("tessellate imported solid");
        assert_eq!(
            boundary_edge_count(&mesh),
            0,
            "open mesh at deflection {deflection}"
        );
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "non-manifold mesh at deflection {deflection}"
        );
    }
}
