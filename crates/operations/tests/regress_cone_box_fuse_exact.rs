//! The census cone/box fuse must remain an exact closed analytic body.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::{
    context::{FallbackPolicy, OperationContext},
    mat::Mat4,
};
use remus_operations::{
    boolean::{BooleanOp, BooleanQuality, boolean_with_context},
    primitives::{make_box, make_cone},
    transform::transform_solid,
};
use remus_topology::Topology;

#[test]
fn cone_box_fuse_retains_exact_census_body() {
    let _ = env_logger::try_init();
    let mut topo = Topology::new();
    let cone = make_cone(&mut topo, 6.0, 2.0, 12.0).unwrap();
    let tool = make_box(&mut topo, 8.0, 8.0, 8.0).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(-4.0, -4.0, 6.0)).unwrap();
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let result = boolean_with_context(&mut topo, BooleanOp::Fuse, cone, tool, &context).unwrap();
    assert_eq!(result.quality, BooleanQuality::Exact);
    assert!(
        remus_operations::validate::validate_solid(&topo, result.solid)
            .unwrap()
            .is_valid()
    );
    let expected = 512.0 + 152.0 * std::f64::consts::PI;
    let volume = remus_operations::measure::mass_properties(&topo, result.solid)
        .unwrap()
        .mass;
    assert!(
        (volume - expected).abs() <= expected * 1e-8,
        "volume {volume} versus {expected}"
    );
    for face in remus_topology::explorer::solid_faces(&topo, result.solid).unwrap() {
        assert!(matches!(
            topo.face(face).unwrap().surface(),
            remus_topology::face::FaceSurface::Plane { .. }
                | remus_topology::face::FaceSurface::Cone(_)
        ));
    }
    // The deflection-based public volume path may chord the conical rim.
    let sampled = remus_operations::measure::solid_volume(&topo, result.solid, 0.005).unwrap();
    assert!((sampled - expected).abs() <= expected * 1e-3);
    for deflection in [0.005, 0.02] {
        let mesh = remus_operations::tessellate::tessellate_solid(&topo, result.solid, deflection)
            .unwrap();
        let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
        assert!(
            quality.is_watertight(),
            "deflection {deflection}: {quality:?}"
        );
    }
}
