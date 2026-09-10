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

#[test]
fn hammer_trim_curve_distance_is_not_shadowed_by_carrier_projection() {
    let mut topo = Topology::new();
    let source =
        read_step(include_str!("data/shapr3d_hammer_holder.step"), &mut topo).expect("import")[0];
    // This point lies on the imported NURBS trimming curve shared by a
    // cylinder and a freeform blend. Its cylinder projection is 2.45e-5 mm
    // away, but the trim itself passes through the point to roundoff.
    let point = remus_math::vec::Point3::new(
        -12.803_495_148_159_948,
        28.543_321_496_372_553,
        16.992_178_819_125_375,
    );
    let distance = remus_operations::distance::point_to_solid_distance(&topo, point, source)
        .expect("distance");
    assert!(distance.distance < 1e-7, "{distance:?}");
}

#[test]
fn hammer_topological_vertex_is_part_of_the_distance_boundary() {
    let mut topo = Topology::new();
    let source =
        read_step(include_str!("data/shapr3d_hammer_holder.step"), &mut topo).expect("import")[0];
    // Shared by four faces; the nearest carrier/curve is about 2e-5 mm
    // away from this stored vertex. Querying the vertex must return zero.
    let point = remus_math::vec::Point3::new(-12.940_962_466, 34.499_957_672, 11.681_806_638);
    let distance = remus_operations::distance::point_to_solid_distance(&topo, point, source)
        .expect("distance");
    assert!(distance.distance < 1e-10, "{distance:?}");
}

#[test]
fn hammer_intersection_preserves_the_closed_lettering_loops() {
    let mut topo = Topology::new();
    let source =
        read_step(include_str!("data/shapr3d_hammer_holder.step"), &mut topo).expect("import")[0];
    let original = remus_io::arena_io::serialize_solid(&topo, source).expect("source before");
    let mask = make_box(&mut topo, 29.0, 53.0, 70.0).expect("mask");
    transform_solid(&mut topo, mask, &Mat4::translation(-18.0, -10.0, 0.0)).expect("place");
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let result = boolean_with_context(&mut topo, BooleanOp::Intersect, source, mask, &context)
        .expect("intersection");
    assert_eq!(result.quality, BooleanQuality::Exact);
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, result.solid)
            .expect("faces")
            .len(),
        101
    );
    assert!(
        validate_solid(&topo, result.solid, &ValidateOptions::default())
            .expect("validate")
            .is_valid()
    );
    let mesh = remus_operations::tessellate::tessellate_solid_with_tolerance(
        &topo,
        result.solid,
        0.05,
        0.1,
    )
    .expect("mesh");
    let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
    assert!(quality.is_watertight(), "{quality:?}");
    assert!(
        mesh.positions
            .iter()
            .all(|p| p.x() >= -18.000_001 && p.x() <= 11.000_001 && p.y() <= 43.000_001)
    );
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, source).expect("source after"),
        original
    );
    let step = remus_io::step::writer::write_step(&topo, &[result.solid]).expect("export");
    let mut restored = Topology::new();
    let solids = read_step(&step, &mut restored).expect("reimport");
    assert_eq!(solids.len(), 1);
    assert!(
        validate_solid(&restored, solids[0], &ValidateOptions::default())
            .expect("restored validation")
            .is_valid()
    );
    let mesh = remus_operations::tessellate::tessellate_solid_with_tolerance(
        &restored, solids[0], 0.05, 0.1,
    )
    .expect("restored mesh");
    assert!(remus_operations::tessellate::welded_mesh_quality(&mesh).is_watertight());
}

/// Qualify the shifted intersection and its reassembly with the outside partition.
#[test]
fn hammer_shifted_intersection_and_left_fuse_are_strictly_valid() {
    let mut topo = Topology::new();
    let source =
        read_step(include_str!("data/shapr3d_hammer_holder.step"), &mut topo).expect("import")[0];
    let original = remus_io::arena_io::serialize_solid(&topo, source).expect("source");
    let mask = make_box(&mut topo, 29.0, 53.0, 70.0).expect("mask");
    transform_solid(&mut topo, mask, &Mat4::translation(-18.0, -10.0, 0.0)).expect("place");
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let inside = boolean_with_context(&mut topo, BooleanOp::Intersect, source, mask, &context)
        .expect("first intersection")
        .solid;
    let shifted = remus_operations::copy::copy_solid(&mut topo, source).expect("copy");
    transform_solid(&mut topo, shifted, &Mat4::translation(-2.0, 0.0, 0.0)).expect("shift");
    let result = boolean_with_context(&mut topo, BooleanOp::Intersect, inside, shifted, &context)
        .expect("shifted intersection");
    assert_eq!(result.quality, BooleanQuality::Exact);
    let candidate = result.solid;
    let report =
        validate_solid(&topo, candidate, &ValidateOptions::default()).expect("validate candidate");
    assert!(report.is_valid(), "{:?}", report.issues);
    let mesh =
        remus_operations::tessellate::tessellate_solid_with_tolerance(&topo, candidate, 0.05, 0.1)
            .expect("candidate mesh");
    assert!(remus_operations::tessellate::welded_mesh_quality(&mesh).is_watertight());
    let step = remus_io::step::writer::write_step(&topo, &[candidate]).expect("candidate export");
    let mut restored = Topology::new();
    let solids = read_step(&step, &mut restored).expect("candidate reimport");
    assert_eq!(solids.len(), 1);
    let report = validate_solid(&restored, solids[0], &ValidateOptions::default())
        .expect("round trip validation");
    assert!(report.is_valid(), "{:?}", report.issues);

    let mesh = remus_operations::tessellate::tessellate_solid_with_tolerance(
        &restored, solids[0], 0.05, 0.1,
    )
    .expect("restored mesh");
    assert!(remus_operations::tessellate::welded_mesh_quality(&mesh).is_watertight());
    let volume =
        remus_operations::measure::solid_volume(&topo, candidate, 0.01).expect("candidate volume");
    let restored_volume = remus_operations::measure::solid_volume(&restored, solids[0], 0.01)
        .expect("restored volume");
    let inside_volume =
        remus_operations::measure::solid_volume(&topo, inside, 0.01).expect("partition volume");
    assert!(volume > 0.0 && volume < inside_volume);
    assert!((restored_volume - volume).abs() < volume * 1e-6);

    let mut uses = std::collections::BTreeMap::new();
    for fid in remus_topology::explorer::solid_faces(&topo, candidate).expect("faces") {
        let face = topo.face(fid).expect("face");
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).expect("wire").edges() {
                *uses.entry(oe.edge()).or_insert(0usize) += 1;
            }
        }
    }
    let mut bottom_edges = 0;
    let mut round_edges = 0;
    let mut lettering_edges = 0;
    let mut slope_edges = 0;
    for (eid, count) in uses {
        let edge = topo.edge(eid).expect("edge");
        let a = topo.vertex(edge.start()).expect("start").point();
        let b = topo.vertex(edge.end()).expect("end").point();
        assert_eq!(count, 2, "unpaired candidate edge: {a:?} -> {b:?}");
        let slope = |p: remus_math::vec::Point3| {
            p.x() >= -18.000_001
                && p.x() <= -16.999_999
                && p.y() >= 30.378_678
                && p.y() <= 36.621_322
                && (p.z() - p.y() - 19.0).abs() < 1e-7
        };
        if slope(a) && slope(b) {
            slope_edges += 1;
            assert_eq!(count, 2, "unpaired sloped edge: {a:?} -> {b:?}");
        }
        let upper_round = |p: remus_math::vec::Point3| {
            p.x() >= -17.000_001
                && p.x() <= -13.999_999
                && p.y() <= 21.500_001
                && p.z() >= 47.499_999
        };
        if upper_round(a) && upper_round(b) {
            round_edges += 1;
            assert_eq!(count, 2, "unpaired upper-round edge: {a:?} -> {b:?}");
        }
        let lettering = |p: remus_math::vec::Point3| {
            p.x() >= -14.000_001
                && p.x() <= -13.599_999
                && p.y() >= 17.0
                && p.y() <= 20.0
                && p.z() >= 17.0
                && p.z() <= 19.0
        };
        if lettering(a) && lettering(b) {
            lettering_edges += 1;
            assert_eq!(count, 2, "unpaired lettering edge: {a:?} -> {b:?}");
        }
        if (a.z() - 4.5).abs() < 1e-7 && (b.z() - 4.5).abs() < 1e-7 {
            bottom_edges += 1;
            assert_eq!(count, 2, "unpaired bottom edge: {a:?} -> {b:?}");
        }
    }
    assert!(
        bottom_edges > 0,
        "candidate must retain its bottom boundary"
    );
    assert!(
        slope_edges >= 4,
        "candidate must retain its sloped boundary"
    );
    assert!(round_edges >= 4, "candidate must retain its upper round");
    assert!(lettering_edges > 0, "candidate must retain its lettering");
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, source).expect("source after"),
        original
    );
    // Reattach the edited left partition without introducing off-patch sphere
    // circles or treating a subdivided straight junction as a crossing.
    let outside = boolean_with_context(&mut topo, BooleanOp::Cut, source, mask, &context)
        .expect("outside partition")
        .solid;
    let fused = boolean_with_context(&mut topo, BooleanOp::Fuse, outside, candidate, &context)
        .expect("left reassembly");
    assert_eq!(fused.quality, BooleanQuality::Exact);
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, fused.solid)
            .expect("fused faces")
            .len(),
        177
    );
    assert_valid_mesh(&topo, fused.solid);
    let fused_step =
        remus_io::step::writer::write_step(&topo, &[fused.solid]).expect("fused export");
    let mut fused_topo = Topology::new();
    let fused_import = read_step(&fused_step, &mut fused_topo).expect("fused reimport");
    assert_eq!(fused_import.len(), 1);
    assert_valid_mesh(&fused_topo, fused_import[0]);
    let fused_volume =
        remus_operations::measure::solid_volume(&topo, fused.solid, 0.01).expect("fused volume");
    let outside_volume =
        remus_operations::measure::solid_volume(&topo, outside, 0.01).expect("outside volume");
    let round_trip_volume =
        remus_operations::measure::solid_volume(&fused_topo, fused_import[0], 0.01)
            .expect("fused restored volume");
    assert!(
        (fused_volume - outside_volume - volume).abs() < fused_volume * 1e-5,
        "partition volumes: {outside_volume} + {volume} -> {fused_volume}"
    );
    assert!((round_trip_volume - fused_volume).abs() < fused_volume * 1e-6);
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, source).expect("source after fuse"),
        original
    );
}

#[test]
fn hammer_rear_torus_points_are_outside_the_shifted_holder() {
    use remus_algo::{
        FaceClass,
        classifier::{RayCastGeoms, classify_ray_cast_cached},
    };
    use remus_math::vec::Point3;
    let mut topo = Topology::new();
    let source =
        read_step(include_str!("data/shapr3d_hammer_holder.step"), &mut topo).expect("import")[0];
    transform_solid(&mut topo, source, &Mat4::translation(-2.0, 0.0, 0.0)).expect("shift");
    let geoms = RayCastGeoms::new(&topo, source).expect("classifier");
    // Points on the original rear round lie in the opening of the translated
    // holder. Flat polygons substituted for its nonrectangular torus trim
    // incorrectly count ray crossings here, retaining the original round.
    for point in [
        Point3::new(
            -10.369_114_578_163_55,
            41.327_440_176_699_69,
            12.474_994_431_357_405,
        ),
        Point3::new(
            -12.236_284_624_550_887,
            40.026_861_291_718_77,
            12.474_994_431_357_405,
        ),
        Point3::new(
            -10.498_752_769_552_034,
            38.192_908_739_274_17,
            10.217_747_987_641_948,
        ),
    ] {
        assert_eq!(
            classify_ray_cast_cached(&geoms, point).expect("classify"),
            FaceClass::Outside,
            "{point:?}"
        );
    }
}
