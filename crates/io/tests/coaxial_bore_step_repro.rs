//! A boolean that replaces an existing coaxial cylindrical face must still
//! export a closed shell.
//!
//! Re-boring a hole (cut a block with r=3, then cut it again coaxially with
//! r=5) used to leave the OLD r=3 rim behind as an inner wire on both cap
//! faces while no r=3 cylindrical face remained. Each stale rim was then used
//! by exactly one face, so the exported shell was not closed.
//!
//! The defect was invisible to the usual gates: the old disc is contained in
//! the new one, so it subtracts no area and both the volume and the relaxed
//! validation still passed. Only the free-edge count exposes it, which is what
//! these tests assert alongside the STEP round-trip.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::collections::HashMap;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::push_pull::{push_pull_face, resize_cylindrical_face};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const DEFLECTION: f64 = 0.01;

fn cylinder_at(topo: &mut Topology, r: f64, h: f64, x: f64, y: f64, z: f64) -> SolidId {
    let c = make_cylinder(topo, r, h).unwrap();
    transform_solid(topo, c, &Mat4::translation(x, y, z)).unwrap();
    c
}

/// A 40x40x10 block with an r=3 through-bore at (20, 20).
fn drilled_block(topo: &mut Topology) -> SolidId {
    let block = make_box(topo, 40.0, 40.0, 10.0).unwrap();
    let drill = cylinder_at(topo, 3.0, 10.0, 20.0, 20.0, 0.0);
    boolean(topo, BooleanOp::Cut, block, drill).unwrap()
}

/// Every edge of a closed shell is used by exactly two faces.
fn assert_watertight(topo: &Topology, solid: SolidId, label: &str) {
    let mut counts: HashMap<usize, usize> = HashMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                *counts.entry(oe.edge().index()).or_insert(0) += 1;
            }
        }
    }
    let free: Vec<_> = counts.iter().filter(|&(_, &c)| c != 2).collect();
    assert!(
        free.is_empty(),
        "{label}: edges not shared by exactly 2 faces: {free:?}"
    );
}

fn face_count(topo: &Topology, solid: SolidId, tag: &str) -> usize {
    solid_faces(topo, solid)
        .unwrap()
        .iter()
        .filter(|&&f| topo.face(f).unwrap().surface().type_tag() == tag)
        .count()
}

fn only_cylinder(topo: &Topology, solid: SolidId) -> FaceId {
    let cyls: Vec<_> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .collect();
    assert_eq!(cyls.len(), 1, "expected exactly one cylindrical face");
    cyls[0]
}

fn top_face(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| {
            topo.face(f)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.z() > 0.99)
        })
        .expect("no +Z face")
}

/// Export, re-import, and require the same watertight solid of equal volume.
fn assert_step_round_trip(topo: &Topology, solid: SolidId, label: &str) {
    assert_watertight(topo, solid, &format!("{label} (before export)"));

    let text = remus_io::step::writer::write_step(topo, &[solid]).unwrap();
    let before = solid_volume(topo, solid, DEFLECTION).unwrap();

    let mut rt = Topology::new();
    let solids = remus_io::step::reader::read_step(&text, &mut rt).unwrap();
    assert_eq!(solids.len(), 1, "{label}: expected one solid after import");

    assert_watertight(&rt, solids[0], &format!("{label} (after import)"));
    let after = solid_volume(&rt, solids[0], DEFLECTION).unwrap();
    assert!(
        (after - before).abs() < 1e-3 * before.abs().max(1.0),
        "{label}: volume changed across STEP round-trip: {before} -> {after}"
    );
}

#[test]
fn drilled_block_step_round_trips() {
    let mut topo = Topology::new();
    let drilled = drilled_block(&mut topo);
    assert_step_round_trip(&topo, drilled, "drilled r=3");
}

/// The original repro: a second, coaxial cut that must replace the r=3 wall.
#[test]
fn recut_coaxial_bore_step_round_trips() {
    let mut topo = Topology::new();
    let drilled = drilled_block(&mut topo);
    let widen = cylinder_at(&mut topo, 5.0, 10.0, 20.0, 20.0, 0.0);
    let widened = boolean(&mut topo, BooleanOp::Cut, drilled, widen).unwrap();

    let v = solid_volume(&topo, widened, DEFLECTION).unwrap();
    assert!(
        (v - 15_214.6).abs() < 5.0,
        "expected re-bored volume ~15214.6, got {v}"
    );
    // Exactly one bore wall, and no leftover rim from the r=3 one.
    assert_eq!(face_count(&topo, widened, "cylinder"), 1);
    assert_eq!(face_count(&topo, widened, "plane"), 6);
    assert_step_round_trip(&topo, widened, "re-bored r=5");
}

/// The same replacement reached through a tool that overshoots both caps.
#[test]
fn recut_coaxial_bore_with_overshooting_tool_step_round_trips() {
    let mut topo = Topology::new();
    let drilled = drilled_block(&mut topo);
    let widen = cylinder_at(&mut topo, 5.0, 14.0, 20.0, 20.0, -2.0);
    let widened = boolean(&mut topo, BooleanOp::Cut, drilled, widen).unwrap();

    assert_eq!(face_count(&topo, widened, "cylinder"), 1);
    assert_step_round_trip(&topo, widened, "re-bored r=5 (overshooting tool)");
}

#[test]
fn push_pull_result_step_round_trips() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let top = top_face(&topo, block);
    let pulled = push_pull_face(&mut topo, block, top, 5.0).unwrap();
    assert_step_round_trip(&topo, pulled, "pulled box face");

    let top2 = top_face(&topo, pulled);
    let pushed = push_pull_face(&mut topo, pulled, top2, -4.0).unwrap();
    assert_step_round_trip(&topo, pushed, "pushed box face");
}

#[test]
fn widened_bore_via_resize_step_round_trips() {
    let mut topo = Topology::new();
    let drilled = drilled_block(&mut topo);
    let bore = only_cylinder(&topo, drilled);
    let out = resize_cylindrical_face(&mut topo, drilled, bore, 5.0).unwrap();
    assert_step_round_trip(&topo, out, "resize bore 3 -> 5");
}

#[test]
fn shrunk_bore_via_resize_step_round_trips() {
    let mut topo = Topology::new();
    let drilled = drilled_block(&mut topo);
    let bore = only_cylinder(&topo, drilled);
    let out = resize_cylindrical_face(&mut topo, drilled, bore, 2.0).unwrap();
    assert_step_round_trip(&topo, out, "resize bore 3 -> 2");
}

#[test]
fn resized_boss_step_round_trips() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 40.0, 40.0, 10.0).unwrap();
    let boss = cylinder_at(&mut topo, 5.0, 10.0, 20.0, 20.0, 10.0);
    let bossed = boolean(&mut topo, BooleanOp::Fuse, block, boss).unwrap();

    let wall = only_cylinder(&topo, bossed);
    let grown = resize_cylindrical_face(&mut topo, bossed, wall, 8.0).unwrap();
    assert_step_round_trip(&topo, grown, "resize boss 5 -> 8");

    let wall2 = only_cylinder(&topo, grown);
    let shrunk = resize_cylindrical_face(&mut topo, grown, wall2, 4.0).unwrap();
    assert_step_round_trip(&topo, shrunk, "resize boss 8 -> 4");
}
