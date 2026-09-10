//! Exact opening partition regression for the imported hammer holder.
#![allow(clippy::expect_used)]
use remus_check::validate::{ValidateOptions, validate_solid};
use remus_io::step::reader::read_step;
use remus_math::{
    context::{FallbackPolicy, OperationContext},
    mat::Mat4,
};
use remus_operations::{
    boolean::{BooleanOp, BooleanQuality, boolean_with_context},
    primitives::make_box,
    transform::transform_solid,
};
use remus_topology::{Topology, face::FaceSurface, solid::SolidId};

fn assert_mounting_bores(topo: &Topology, solid: SolidId) {
    let mut centers = Vec::new();
    for fid in remus_topology::explorer::solid_faces(topo, solid).expect("faces") {
        let face = topo.face(fid).expect("face");
        if let FaceSurface::Cylinder(cylinder) = face.surface()
            && (cylinder.radius() - 2.5).abs() < 1e-7
        {
            assert!((cylinder.origin().y() - 49.5).abs() < 1e-7);
            assert!((cylinder.axis().z() + 1.0).abs() < 1e-7);
            centers.push(cylinder.origin().x());
            let heights: Vec<_> = remus_topology::explorer::face_vertices(topo, fid)
                .expect("vertices")
                .into_iter()
                .map(|v| topo.vertex(v).expect("vertex").point().z())
                .collect();
            assert!((heights.iter().copied().fold(f64::INFINITY, f64::min) - 4.5).abs() < 1e-7);
            assert!(
                (heights.iter().copied().fold(f64::NEG_INFINITY, f64::max) - 10.5).abs() < 1e-7
            );
        }
    }
    centers.sort_by(f64::total_cmp);
    assert_eq!(centers.len(), 2);
    assert!((centers[0] + 9.0).abs() < 1e-7);
    assert!((centers[1] - 31.0).abs() < 1e-7);
}

fn assert_valid_mesh(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid, &ValidateOptions::default()).expect("validate");
    assert!(report.is_valid(), "{:?}", report.issues);
    let mesh =
        remus_operations::tessellate::tessellate_solid_with_tolerance(topo, solid, 0.05, 0.1)
            .expect("mesh");
    assert!(remus_operations::tessellate::is_watertight(&mesh));
    let welded = remus_operations::tessellate::welded_mesh_quality(&mesh);
    assert!(welded.is_watertight(), "{welded:?}");
    assert_mounting_bores(topo, solid);
}

#[test]
fn hammer_opening_partition_is_strictly_valid() {
    let mut topo = Topology::new();
    let source =
        read_step(include_str!("data/shapr3d_hammer_holder.step"), &mut topo).expect("import")[0];
    let original = remus_io::arena_io::serialize_solid(&topo, source).expect("source before");
    let mask = make_box(&mut topo, 29.0, 53.0, 70.0).expect("mask");
    transform_solid(&mut topo, mask, &Mat4::translation(-18.0, -10.0, 0.0)).expect("place");
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let result =
        boolean_with_context(&mut topo, BooleanOp::Cut, source, mask, &context).expect("partition");
    assert_eq!(result.quality, BooleanQuality::Exact);
    assert_valid_mesh(&topo, result.solid);
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, source).expect("source after"),
        original
    );
    let exported = remus_io::step::writer::write_step(&topo, &[result.solid]).expect("export");
    let mut restored = Topology::new();
    let imported = read_step(&exported, &mut restored).expect("reimport");
    assert_eq!(imported.len(), 1);
    assert_valid_mesh(&restored, imported[0]);
    let volume =
        remus_operations::measure::solid_volume(&topo, result.solid, 0.01).expect("volume");
    let restored_volume = remus_operations::measure::solid_volume(&restored, imported[0], 0.01)
        .expect("restored volume");
    assert!(volume > 0.0);
    assert!(
        (restored_volume - volume).abs() < 1e-6 * volume,
        "{volume} -> {restored_volume}"
    );
}
