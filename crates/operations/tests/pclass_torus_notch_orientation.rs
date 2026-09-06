//! Complementary torus notch bands must retain their oriented material regions.
#![allow(clippy::unwrap_used, clippy::panic)]
use remus_math::{
    context::{FallbackPolicy, OperationContext},
    mat::Mat4,
};
use remus_operations::{
    boolean::{BooleanOp, boolean_with_context},
    primitives::{make_box, make_torus},
    transform::transform_solid,
};
use remus_topology::Topology;

fn intersection_volume() -> f64 {
    // Integrate the annular horizontal section clipped by x >= 6, |y| <= 4.
    let disk = |r: f64| {
        let y = 4.0_f64.min((r * r - 36.0).max(0.0).sqrt());
        y * (r * r - y * y).sqrt() + r * r * (y / r).asin() - 12.0 * y
    };
    let area = |angle: f64| {
        let tube = 3.0 * angle.cos();
        (disk(10.0 + tube) - disk(10.0 - tube)) * tube
    };
    let n = 80_000;
    let h = std::f64::consts::PI / f64::from(n);
    let low = -std::f64::consts::FRAC_PI_2;
    h / 3.0
        * (area(low)
            + area(-low)
            + (1..n)
                .map(|i| (if i % 2 == 0 { 2.0 } else { 4.0 }) * area(low + f64::from(i) * h))
                .sum::<f64>())
}

fn qualify(op: BooleanOp, scale: f64, placed: bool) {
    let _ = env_logger::try_init();
    let overlap = intersection_volume();
    let box_volume_expected = 512.0;
    let torus_volume_expected = 180.0 * std::f64::consts::PI.powi(2);
    let expected = match op {
        BooleanOp::Fuse => box_volume_expected + torus_volume_expected - overlap,
        BooleanOp::Cut => torus_volume_expected - overlap,
        BooleanOp::Intersect => overlap,
    };
    let mut topo = Topology::new();
    let box_operand = make_box(&mut topo, 8.0 * scale, 8.0 * scale, 8.0 * scale).unwrap();
    let torus_operand = make_torus(&mut topo, 10.0 * scale, 3.0 * scale, 32).unwrap();
    transform_solid(
        &mut topo,
        box_operand,
        &Mat4::translation(6.0 * scale, -4.0 * scale, -4.0 * scale),
    )
    .unwrap();
    if placed {
        let transform =
            Mat4::translation(17.0 * scale, -23.0 * scale, 31.0 * scale) * Mat4::rotation_y(0.37);
        for solid in [box_operand, torus_operand] {
            transform_solid(&mut topo, solid, &transform).unwrap();
        }
    }
    let ctx = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let result = boolean_with_context(&mut topo, op, torus_operand, box_operand, &ctx)
        .unwrap_or_else(|e| panic!("{op:?} scale={scale} placed={placed}: {e}"));
    let report = remus_operations::validate::validate_solid(&topo, result.solid).unwrap();
    assert!(
        report.is_valid(),
        "{op:?} scale={scale} placed={placed}: {report:?}"
    );
    let mut tori = 0;
    let mut planes = 0;
    for fid in remus_topology::explorer::solid_faces(&topo, result.solid).unwrap() {
        let face = topo.face(fid).unwrap();
        match face.surface() {
            remus_topology::face::FaceSurface::Torus(_) => tori += 1,
            remus_topology::face::FaceSurface::Plane { .. } => planes += 1,
            other => panic!("{op:?}: unexpected {} carrier", other.type_tag()),
        }
        let mut error = 0.0_f64;
        for eid in remus_topology::explorer::face_edges(&topo, fid).unwrap() {
            let e = topo.edge(eid).unwrap();
            if !matches!(e.curve(), remus_topology::edge::EdgeCurve::NurbsCurve(_)) {
                continue;
            }
            let (a, b) = e.strict_domain().unwrap();
            for i in 0..=128 {
                let p = e.curve().evaluate_with_endpoints(
                    a + (b - a) * f64::from(i) / 128.0,
                    topo.vertex(e.start()).unwrap().point(),
                    topo.vertex(e.end()).unwrap().point(),
                );
                let residual = match face.surface() {
                    remus_topology::face::FaceSurface::Plane { normal, d } => {
                        (normal.dot(p - remus_math::vec::Point3::new(0.0, 0.0, 0.0)) - d).abs()
                    }
                    surface => {
                        let (u, v) = surface.project_point(p).unwrap();
                        (p - surface.evaluate(u, v).unwrap()).length()
                    }
                };
                error = error.max(residual);
            }
        }
        assert!(
            error <= ctx.tolerance.linear,
            "{op:?} scale={scale} placed={placed}: {} seam residual {error}",
            face.surface().type_tag()
        );
    }
    assert!(tori > 0 && planes > 0, "{op:?}: both carriers must survive");
    let target = expected * scale.powi(3);
    let volume =
        remus_operations::measure::solid_volume(&topo, result.solid, 0.01 * scale).unwrap();

    let mesh =
        remus_operations::tessellate::tessellate_solid(&topo, result.solid, 0.005 * scale).unwrap();
    let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
    assert!(
        quality.is_watertight(),
        "{op:?} scale={scale} placed={placed}: {quality:?}"
    );
    let origin = mesh.positions[0];
    let mesh_volume: f64 = mesh
        .indices
        .chunks_exact(3)
        .map(|tri| {
            let a = mesh.positions[tri[0] as usize] - origin;
            let b = mesh.positions[tri[1] as usize] - origin;
            let c = mesh.positions[tri[2] as usize] - origin;
            a.dot(b.cross(c)) / 6.0
        })
        .sum();
    assert!(
        (volume - target).abs() / target < 0.001,
        "{op:?} scale={scale} placed={placed}: {volume} vs {target}; mesh {mesh_volume}; signed {:?}",
        remus_topology::explorer::solid_faces(&topo, result.solid)
            .unwrap()
            .iter()
            .map(
                |&f| remus_check::properties::face_integrator::integrate_face(&topo, f, 8)
                    .unwrap()
                    .volume
            )
            .sum::<f64>()
    );
    assert!(
        (mesh_volume - target).abs() / target < 0.01,
        "{op:?} scale={scale} placed={placed}: mesh {mesh_volume} vs {target}"
    );
}

#[test]
fn torus_notch_cut_small_origin() {
    qualify(BooleanOp::Cut, 0.1, false);
}

#[test]
fn torus_notch_cut_small_placed() {
    qualify(BooleanOp::Cut, 0.1, true);
}

#[test]
fn torus_notch_cut_unit_origin() {
    qualify(BooleanOp::Cut, 1.0, false);
}

#[test]
fn torus_notch_cut_unit_placed() {
    qualify(BooleanOp::Cut, 1.0, true);
}

#[test]
fn torus_notch_cut_large_origin() {
    qualify(BooleanOp::Cut, 10.0, false);
}

#[test]
fn torus_notch_cut_large_placed() {
    qualify(BooleanOp::Cut, 10.0, true);
}

#[test]
fn torus_notch_fuse_small_origin() {
    qualify(BooleanOp::Fuse, 0.1, false);
}

#[test]
fn torus_notch_fuse_small_placed() {
    qualify(BooleanOp::Fuse, 0.1, true);
}

#[test]
fn torus_notch_fuse_unit_origin() {
    qualify(BooleanOp::Fuse, 1.0, false);
}

#[test]
fn torus_notch_fuse_unit_placed() {
    qualify(BooleanOp::Fuse, 1.0, true);
}

#[test]
fn torus_notch_fuse_large_origin() {
    qualify(BooleanOp::Fuse, 10.0, false);
}

#[test]
fn torus_notch_fuse_large_placed() {
    qualify(BooleanOp::Fuse, 10.0, true);
}

#[test]
fn torus_notch_intersect_small_origin() {
    qualify(BooleanOp::Intersect, 0.1, false);
}

#[test]
fn torus_notch_intersect_small_placed() {
    qualify(BooleanOp::Intersect, 0.1, true);
}

#[test]
fn torus_notch_intersect_unit_origin() {
    qualify(BooleanOp::Intersect, 1.0, false);
}

#[test]
fn torus_notch_intersect_unit_placed() {
    qualify(BooleanOp::Intersect, 1.0, true);
}

#[test]
fn torus_notch_intersect_large_origin() {
    qualify(BooleanOp::Intersect, 10.0, false);
}

#[test]
fn torus_notch_intersect_large_placed() {
    qualify(BooleanOp::Intersect, 10.0, true);
}
