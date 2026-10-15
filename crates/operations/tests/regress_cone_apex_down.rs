//! PRIMITIVE (kernel review 2026-10-14) — `make_cone(0, r, h)` built its
//! lateral face against its base cap.
//!
//! With the apex down, the base circle sits at `z = h` and the cap uses it
//! forward (counter-clockwise about +z, outward normal +z). The lateral wire
//! used the same circle forward too, so the one edge the two faces share ran
//! the same way in both: `validate_solid` reported "1 shared edges have
//! inconsistent face orientations", and the lateral face wound against its
//! cone normal. Apex-up cones and frustums were unaffected. A fuse consuming
//! such a cone carried the defect into its result (op_seq_81 finding F11).
//!
//! The lateral face now uses the base circle opposite to the cap, which makes
//! the apex-down cone the exact half-turn image of the apex-up one.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::f64::consts::PI;

use remus_check::validate::{CheckId, EntityRef, ValidateOptions, validate_solid};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_cone, make_cylinder};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

fn assert_clean(topo: &Topology, solid: SolidId, label: &str) {
    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    assert!(report.issues.is_empty(), "{label}: {:?}", report.issues);
}

/// Signed volume of a triangle mesh: positive only when its triangles face
/// outward.
fn signed_mesh_volume(topo: &Topology, solid: SolidId, deflection: f64) -> f64 {
    let mesh = tessellate_solid(topo, solid, deflection).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0, "mesh is open");
    assert_eq!(non_manifold_edge_count(&mesh), 0, "mesh is non-manifold");
    mesh.indices
        .chunks_exact(3)
        .map(|tri| {
            let [a, b, c] = [0, 1, 2].map(|k| {
                let p = mesh.positions[tri[k] as usize];
                remus_math::vec::Vec3::new(p.x(), p.y(), p.z())
            });
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

#[test]
fn apex_down_cone_is_valid_with_closed_form_volume() {
    for scale in [1e-3, 1.0, 1e3] {
        for (r, h) in [(1.0, 1.0), (3.0, 1.0), (0.5, 4.0)] {
            let (r, h) = (r * scale, h * scale);
            let label = format!("make_cone(0, {r}, {h})");
            let mut topo = Topology::new();
            let cone = make_cone(&mut topo, 0.0, r, h).unwrap();
            assert_clean(&topo, cone, &label);

            let exact = PI * r * r * h / 3.0;
            let volume = solid_volume(&topo, cone, 1e-3 * r).unwrap();
            assert!(
                (volume - exact).abs() <= exact * 1e-12,
                "{label}: volume {volume}, expected {exact}"
            );
            // Chord-inscribed, so a little under the exact volume, but with
            // the sign of outward-facing triangles.
            let meshed = signed_mesh_volume(&topo, cone, 1e-3 * r);
            assert!(
                meshed > 0.99 * exact && meshed <= exact * (1.0 + 1e-12),
                "{label}: signed mesh volume {meshed}, expected about {exact}"
            );

            let probes = [
                (Point3::new(0.0, 0.0, 0.75 * h), PointClassification::Inside),
                (
                    Point3::new(0.1 * r, 0.0, 0.25 * h),
                    PointClassification::Inside,
                ),
                (
                    Point3::new(0.5 * r, 0.0, 0.25 * h),
                    PointClassification::Outside,
                ),
                (
                    Point3::new(0.0, 0.0, -0.1 * h),
                    PointClassification::Outside,
                ),
                (Point3::new(0.0, 0.0, 1.1 * h), PointClassification::Outside),
            ];
            for (point, expected) in probes {
                assert_eq!(
                    classify_point(&topo, cone, point, 1e-3 * r, 1e-9 * scale).unwrap(),
                    expected,
                    "{label}: probe {point:?}"
                );
            }
        }
    }
}

/// The apex-down cone is the apex-up cone turned half a turn about x: the
/// two must agree on validity and volume after the turn.
#[test]
fn apex_down_cone_matches_the_turned_apex_up_cone() {
    let mut topo = Topology::new();
    let down = make_cone(&mut topo, 0.0, 2.0, 3.0).unwrap();
    let up = make_cone(&mut topo, 2.0, 0.0, 3.0).unwrap();
    transform_solid(
        &mut topo,
        up,
        &(Mat4::translation(0.0, 0.0, 3.0) * Mat4::rotation_x(PI)),
    )
    .unwrap();
    assert_clean(&topo, up, "turned apex-up cone");
    assert_clean(&topo, down, "apex-down cone");
    let (a, b) = (
        signed_mesh_volume(&topo, down, 1e-3),
        signed_mesh_volume(&topo, up, 1e-3),
    );
    assert!((a - b).abs() <= 1e-9 * a, "{a} vs {b}");
}

/// op_seq_81 F11, without its mirror: a fuse consuming the apex-down cone
/// carried the misoriented base edge into its result.
#[test]
fn apex_down_cone_fuse_is_exact_and_valid() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 1.0, 1.0).unwrap();
    transform_solid(&mut topo, cylinder, &Mat4::translation(1.0, 0.0, -3.0)).unwrap();
    let cone = make_cone(&mut topo, 0.0, 1.0, 1.0).unwrap();
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);

    let outcome =
        boolean_with_context(&mut topo, BooleanOp::Fuse, cylinder, cone, &context).unwrap();

    assert_eq!(outcome.quality, BooleanQuality::Exact);
    let result = outcome.solid;
    // The two lumps share one shell, which `ShellConnected` and the Euler
    // count report by design, and the wrapped cylinder wall's Newell polygon
    // is degenerate. Every other face must wind with its surface and every
    // shared edge must be used both ways.
    let report = validate_solid(&topo, result, &ValidateOptions::default()).unwrap();
    let orientation: Vec<_> = report
        .issues
        .iter()
        .filter(|issue| match issue.check {
            CheckId::ShellOrientationConsistent => true,
            CheckId::FaceOrientationConsistency => !matches!(
                issue.entity,
                EntityRef::Face(face)
                    if matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_))
            ),
            _ => false,
        })
        .collect();
    assert!(orientation.is_empty(), "{orientation:?}");
    // `solid_volume` meshes this two-lump shell; the Gauss route is exact.
    let exact = PI + PI / 3.0;
    let volume = remus_operations::measure::mass_properties(&topo, result)
        .unwrap()
        .mass;
    assert!(
        (volume - exact).abs() <= exact * 1e-9,
        "volume {volume}, expected {exact}"
    );
    for (point, expected) in [
        (Point3::new(0.0, 0.0, 0.75), PointClassification::Inside),
        (Point3::new(0.5, 0.0, 0.25), PointClassification::Outside),
        (Point3::new(1.0, 0.0, -2.5), PointClassification::Inside),
        (Point3::new(0.0, 0.0, -1.0), PointClassification::Outside),
    ] {
        assert_eq!(
            classify_point(&topo, result, point, 1e-3, 1e-9).unwrap(),
            expected,
            "probe {point:?}"
        );
    }
}
