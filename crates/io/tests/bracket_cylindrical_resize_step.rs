//! STEP regression for the complete OpenZCAD mounting-bracket bore resize.

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
const T: f64 = 8.0;
const MOUNT_X: f64 = 16.0;
const MOUNT_Y: f64 = 20.0;
const DEFLECTION: f64 = 0.05;

fn rot_x90_translate(tx: f64, ty: f64, tz: f64) -> Mat4 {
    Mat4::translation(tx, ty, tz) * Mat4::rotation_x(std::f64::consts::FRAC_PI_2)
}

fn fuse_uniform(topo: &mut Topology, solids: Vec<SolidId>) -> SolidId {
    let compound = topo.add_compound(Compound::new(solids));
    let solid = fuse_all(topo, compound).expect("fuse bracket operands");
    unify_faces(topo, solid).expect("unify fuse");
    solid
}

fn cut_uniform(topo: &mut Topology, mut target: SolidId, tools: &[SolidId]) -> SolidId {
    for &tool in tools {
        target = boolean(topo, BooleanOp::Cut, target, tool).expect("cut bracket feature");
    }
    unify_faces(topo, target).expect("unify cut");
    target
}

fn corner_edges(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    remus_topology::explorer::solid_edges(topo, solid)
        .expect("solid edges")
        .into_iter()
        .filter(|&eid| {
            let edge = topo.edge(eid).expect("edge");
            let a = topo.vertex(edge.start()).expect("start").point();
            let b = topo.vertex(edge.end()).expect("end").point();
            let at_corner = |p: remus_math::vec::Point3| {
                (p.x().abs() < 0.1 || (p.x() - W).abs() < 0.1)
                    && (p.y().abs() < 0.1 || (p.y() - D).abs() < 0.1)
                    && (-0.1..=8.1).contains(&p.z())
            };
            at_corner(a)
                && at_corner(b)
                && (a.x() - b.x()).abs() <= 1.5
                && (a.y() - b.y()).abs() <= 1.5
                && (a.z() - b.z()).abs() >= 4.0
        })
        .collect()
}

fn mounting_wall(topo: &Topology, solid: SolidId, radius: f64) -> FaceId {
    let faces: Vec<_> = solid_faces(topo, solid)
        .expect("solid faces")
        .into_iter()
        .filter(|&fid| {
            let face = topo.face(fid).expect("face");
            let FaceSurface::Cylinder(cyl) = face.surface() else {
                return false;
            };
            face.is_reversed()
                && (cyl.radius() - radius).abs() < 1e-8
                && (cyl.origin().x() - MOUNT_X).abs() < 1e-8
                && (cyl.origin().y() - MOUNT_Y).abs() < 1e-8
                && cyl.axis().z().abs() > 1.0 - 1e-10
        })
        .collect();
    assert_eq!(faces.len(), 1, "one r={radius} mounting wall");
    faces[0]
}

fn build_resized_bracket() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let base = make_box(&mut topo, W, D, T).expect("base");
    let wall = make_box(&mut topo, W, T, 32.0).expect("wall");
    transform_solid(&mut topo, wall, &Mat4::translation(0.0, 32.0, 7.5)).expect("seat wall");
    let blank = fuse_uniform(&mut topo, vec![base, wall]);

    let boss = make_cylinder(&mut topo, 10.0, 12.0).expect("boss");
    transform_solid(&mut topo, boss, &rot_x90_translate(40.0, 34.0, 24.0)).expect("place boss");
    let bossed = fuse_uniform(&mut topo, vec![blank, boss]);

    let bore = make_cylinder(&mut topo, 4.0, 48.0).expect("boss bore");
    transform_solid(&mut topo, bore, &rot_x90_translate(40.0, 48.0, 24.0))
        .expect("place boss bore");
    let bored = cut_uniform(&mut topo, bossed, &[bore]);

    let left = make_cylinder(&mut topo, 3.0, 12.0).expect("left mount");
    transform_solid(&mut topo, left, &Mat4::translation(MOUNT_X, MOUNT_Y, -2.0))
        .expect("place left mount");
    let right = make_cylinder(&mut topo, 3.0, 12.0).expect("right mount");
    transform_solid(
        &mut topo,
        right,
        &Mat4::translation(W - MOUNT_X, MOUNT_Y, -2.0),
    )
    .expect("place right mount");
    let drilled = cut_uniform(&mut topo, bored, &[left, right]);

    let edges = corner_edges(&topo, drilled);
    assert_eq!(edges.len(), 4);
    let filleted = fillet_v2(&mut topo, drilled, &edges, 3.0)
        .expect("fillet corners")
        .solid;
    let wall = mounting_wall(&topo, filleted, 3.0);
    let wide = resize_cylindrical_face(&mut topo, filleted, wall, 4.8).expect("widen mount");
    let wide_wall = mounting_wall(&topo, wide, 4.8);
    let narrow = resize_cylindrical_face(&mut topo, wide, wide_wall, 3.8).expect("shrink mount");
    (topo, narrow)
}

fn assert_closed(topo: &Topology, solid: SolidId) {
    let shell = topo
        .shell(topo.solid(solid).expect("solid").outer_shell())
        .expect("shell");
    validate_shell_closed(shell, topo).expect("closed shell");

    let mut uses: HashMap<usize, usize> = HashMap::new();
    for fid in solid_faces(topo, solid).expect("faces") {
        let face = topo.face(fid).expect("face");
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for edge in topo.wire(wid).expect("wire").edges() {
                *uses.entry(edge.edge().index()).or_insert(0) += 1;
            }
        }
    }
    assert!(uses.values().all(|&count| count == 2), "edge uses {uses:?}");
}

fn assert_resized_geometry(topo: &Topology, solid: SolidId, expected_volume: f64) {
    assert_closed(topo, solid);
    let actual = solid_volume(topo, solid, DEFLECTION).expect("volume");
    assert!(
        (actual - expected_volume).abs() <= expected_volume.abs().max(1.0) * 1e-9,
        "volume {actual} != {expected_volume}"
    );

    let mut matching = 0;
    let mut old = 0;
    for fid in solid_faces(topo, solid).expect("faces") {
        if let FaceSurface::Cylinder(cyl) = topo.face(fid).expect("face").surface()
            && (cyl.origin().x() - MOUNT_X).abs() < 1e-8
            && (cyl.origin().y() - MOUNT_Y).abs() < 1e-8
        {
            matching += usize::from((cyl.radius() - 3.8).abs() < 1e-8);
            old += usize::from((cyl.radius() - 4.8).abs() < 1e-8);
        }
    }
    assert_eq!(matching, 1, "one exact r3.8 mounting cylinder");
    assert_eq!(old, 0, "the r4.8 wall must not survive");
}

#[test]
fn resized_mounting_bracket_step_round_trip_is_exact_and_deterministic() {
    let expected = 47_008.076_370_092_516 + PI * (4.8_f64.powi(2) - 3.8_f64.powi(2)) * T;
    let (topo_a, solid_a) = build_resized_bracket();
    let (topo_b, solid_b) = build_resized_bracket();
    assert_resized_geometry(&topo_a, solid_a, expected);
    assert_resized_geometry(&topo_b, solid_b, expected);

    let step_a = remus_io::step::writer::write_step(&topo_a, &[solid_a]).expect("write STEP A");
    let step_b = remus_io::step::writer::write_step(&topo_b, &[solid_b]).expect("write STEP B");
    assert_eq!(
        step_a, step_b,
        "repeated rebuilds must export byte-identical STEP"
    );
    assert_eq!(
        step_a.matches("CYLINDRICAL_SURFACE").count(),
        8,
        "all eight analytic cylindrical faces must export exactly"
    );

    let mut imported = Topology::new();
    let solids = remus_io::step::reader::read_step(&step_a, &mut imported).expect("re-import STEP");
    assert_eq!(solids.len(), 1);
    assert_resized_geometry(&imported, solids[0], expected);
}
