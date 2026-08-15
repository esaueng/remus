//! Regression: shrinking a widened mounting bore in the OpenZCAD bracket.
//!
//! The bracket has no simple analytic solid classifier. Its AABB strictly
//! encloses the annular sleeve used to shrink the bore from r=4.8 to r=3.8,
//! but the sleeve occupies the bore rather than existing material. The old
//! no-classifier containment shortcut therefore copied the bracket unchanged
//! and `resize_cylindrical_face` correctly rejected the no-op by volume.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;
use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_operations::blend_ops::fillet_v2;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::compound_ops::fuse_all;
use remus_operations::heal::unify_faces;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::push_pull::resize_cylindrical_face;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::compound::Compound;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const W: f64 = 80.0;
const D: f64 = 40.0;
const PLATE_T: f64 = 8.0;
const WALL_H: f64 = 32.0;
const MOUNT_X: f64 = 16.0;
const MOUNT_Y: f64 = 20.0;
const DEFLECTION: f64 = 0.05;

fn rot_x90_translate(tx: f64, ty: f64, tz: f64) -> Mat4 {
    Mat4::translation(tx, ty, tz) * Mat4::rotation_x(std::f64::consts::FRAC_PI_2)
}

fn fuse_uniform(topo: &mut Topology, solids: Vec<SolidId>) -> SolidId {
    let compound = topo.add_compound(Compound::new(solids));
    let fused = fuse_all(topo, compound).expect("fuse bracket operands");
    unify_faces(topo, fused).expect("unify fused faces");
    fused
}

fn cut_uniform(topo: &mut Topology, mut target: SolidId, tools: &[SolidId]) -> SolidId {
    for &tool in tools {
        target = boolean(topo, BooleanOp::Cut, target, tool).expect("cut bracket feature");
    }
    unify_faces(topo, target).expect("unify cut faces");
    target
}

fn build_filleted_bracket(topo: &mut Topology) -> SolidId {
    let base = make_box(topo, W, D, PLATE_T).expect("base plate");
    let wall = make_box(topo, W, PLATE_T, WALL_H).expect("wall plate");
    transform_solid(
        topo,
        wall,
        &Mat4::translation(0.0, D - PLATE_T, PLATE_T - 0.5),
    )
    .expect("seat wall");
    let blank = fuse_uniform(topo, vec![base, wall]);

    let boss = make_cylinder(topo, 10.0, 12.0).expect("boss");
    transform_solid(
        topo,
        boss,
        &rot_x90_translate(W / 2.0, D - PLATE_T + 2.0, PLATE_T + WALL_H / 2.0),
    )
    .expect("place boss");
    let bossed = fuse_uniform(topo, vec![blank, boss]);

    let bore = make_cylinder(topo, 4.0, 48.0).expect("boss bore");
    transform_solid(
        topo,
        bore,
        &rot_x90_translate(W / 2.0, D + 8.0, PLATE_T + WALL_H / 2.0),
    )
    .expect("place boss bore");
    let bored = cut_uniform(topo, bossed, &[bore]);

    let mount_a = make_cylinder(topo, 3.0, 12.0).expect("mount A");
    transform_solid(topo, mount_a, &Mat4::translation(MOUNT_X, MOUNT_Y, -2.0))
        .expect("place mount A");
    let mount_b = make_cylinder(topo, 3.0, 12.0).expect("mount B");
    transform_solid(
        topo,
        mount_b,
        &Mat4::translation(W - MOUNT_X, MOUNT_Y, -2.0),
    )
    .expect("place mount B");
    let drilled = cut_uniform(topo, bored, &[mount_a, mount_b]);

    let edges = base_corner_edges(topo, drilled);
    assert_eq!(edges.len(), 4, "expected four outside base corners");
    let result = fillet_v2(topo, drilled, &edges, 3.0).expect("fillet bracket corners");
    assert!(!result.is_partial, "all four bracket fillets must succeed");
    result.solid
}

fn base_corner_edges(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    let mut picked = Vec::new();
    for eid in remus_topology::explorer::solid_edges(topo, solid).expect("bracket edges") {
        let edge = topo.edge(eid).expect("edge");
        let a = topo.vertex(edge.start()).expect("edge start").point();
        let b = topo.vertex(edge.end()).expect("edge end").point();
        let at_corner = |p: remus_math::vec::Point3| {
            (p.x().abs() < 0.1 || (p.x() - W).abs() < 0.1)
                && (p.y().abs() < 0.1 || (p.y() - D).abs() < 0.1)
                && (-0.1..=8.1).contains(&p.z())
        };
        if at_corner(a)
            && at_corner(b)
            && (a.x() - b.x()).abs() <= 1.5
            && (a.y() - b.y()).abs() <= 1.5
            && (a.z() - b.z()).abs() >= 4.0
        {
            picked.push(eid);
        }
    }
    picked
}

fn mounting_wall(topo: &Topology, solid: SolidId, x: f64, radius: f64) -> FaceId {
    let matches: Vec<_> = solid_faces(topo, solid)
        .expect("solid faces")
        .into_iter()
        .filter(|&fid| {
            let face = topo.face(fid).expect("face");
            let FaceSurface::Cylinder(cyl) = face.surface() else {
                return false;
            };
            face.is_reversed()
                && (cyl.radius() - radius).abs() < 1e-8
                && cyl.axis().z().abs() > 1.0 - 1e-10
                && (cyl.origin().x() - x).abs() < 1e-8
                && (cyl.origin().y() - MOUNT_Y).abs() < 1e-8
        })
        .collect();
    assert_eq!(matches.len(), 1, "expected one r={radius} wall at x={x}");
    matches[0]
}

fn assert_closed(topo: &Topology, solid: SolidId) {
    let shell = topo
        .shell(topo.solid(solid).expect("solid").outer_shell())
        .expect("outer shell");
    validate_shell_closed(shell, topo).expect("result shell must be closed");
    let orientation = remus_check::validate::shell::check_shell_orientation(
        topo,
        topo.solid(solid).expect("solid").outer_shell(),
    )
    .expect("check shell orientation");
    assert!(
        orientation.is_empty(),
        "result shell orientation must be consistent: {orientation:?}"
    );

    let mut uses: HashMap<usize, usize> = HashMap::new();
    for fid in solid_faces(topo, solid).expect("solid faces") {
        let face = topo.face(fid).expect("face");
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for edge in topo.wire(wid).expect("wire").edges() {
                *uses.entry(edge.edge().index()).or_insert(0) += 1;
            }
        }
    }
    assert!(
        uses.values().all(|&count| count == 2),
        "every B-rep edge must have exactly two face uses: {uses:?}"
    );
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    solid_volume(topo, solid, DEFLECTION).expect("solid volume")
}

fn assert_volume(actual: f64, expected: f64) {
    let tolerance = expected.abs().max(1.0) * 1e-9;
    assert!(
        (actual - expected).abs() <= tolerance,
        "volume {actual:.15} differs from expected {expected:.15} by more than {tolerance}"
    );
}

#[test]
fn mounting_bore_widen_then_shrink_is_exact_and_preserves_bracket() {
    let mut topo = Topology::new();
    let bracket = build_filleted_bracket(&mut topo);
    assert_volume(volume(&topo, bracket), 47_360.940_056_943_74);

    let mount = mounting_wall(&topo, bracket, MOUNT_X, 3.0);
    let wide = resize_cylindrical_face(&mut topo, bracket, mount, 4.8)
        .expect("widen mounting bore from r3 to r4.8");
    let wide_volume = volume(&topo, wide);
    assert_volume(wide_volume, 47_008.076_370_092_516);
    assert_closed(&topo, wide);

    let wide_mount = mounting_wall(&topo, wide, MOUNT_X, 4.8);
    let narrow = resize_cylindrical_face(&mut topo, wide, wide_mount, 3.8)
        .expect("shrink mounting bore from r4.8 to r3.8");
    let expected = wide_volume + PI * (4.8_f64.powi(2) - 3.8_f64.powi(2)) * PLATE_T;
    assert_volume(volume(&topo, narrow), expected);
    assert_closed(&topo, narrow);

    mounting_wall(&topo, narrow, MOUNT_X, 3.8);
    mounting_wall(&topo, narrow, W - MOUNT_X, 3.0);
    assert!(
        solid_faces(&topo, narrow)
            .expect("solid faces")
            .iter()
            .all(|&fid| match topo.face(fid).expect("face").surface() {
                FaceSurface::Cylinder(cyl) => {
                    (cyl.radius() - 4.8).abs() > 1e-8
                        || (cyl.origin().x() - MOUNT_X).abs() > 1e-8
                        || (cyl.origin().y() - MOUNT_Y).abs() > 1e-8
                }
                _ => true,
            }),
        "the old r4.8 mounting wall must be gone"
    );

    let mut radii: Vec<f64> = solid_faces(&topo, narrow)
        .expect("solid faces")
        .into_iter()
        .filter_map(|fid| match topo.face(fid).expect("face").surface() {
            FaceSurface::Cylinder(cyl) => Some(cyl.radius()),
            _ => None,
        })
        .collect();
    radii.sort_by(f64::total_cmp);
    let expected_radii = [3.0, 3.0, 3.0, 3.0, 3.0, 3.8, 4.0, 10.0];
    assert_eq!(radii.len(), expected_radii.len());
    for (actual, expected) in radii.iter().zip(expected_radii) {
        assert!((actual - expected).abs() < 1e-8, "radii {radii:?}");
    }
}
