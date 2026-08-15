//! Regression test for the OpenZCAD demo bracket corner fillet (#1240-class).
//!
//! Builds the demo L-bracket (base plate + wall + boss, minus a bore and two
//! mount holes) and fillets the four vertical corner edges of the base plate
//! with radius 3 via `fillet_v2`. The v1 rolling-ball rebuild emits an open
//! shell on this topology (L-shaped side faces, coplanar slivers from the
//! 0.5 mm wall seat, holed caps), so `fillet_v2` must fall back to the
//! walking builder's stitched planar assembly and produce a closed solid.
//!
//! The two rear corners terminate at z = 7.5 against coplanar continuation
//! faces (no perpendicular cap), exercising the corner-patch stitch; the two
//! front corners terminate against real caps, exercising the cap trim.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const W: f64 = 80.0; // width
const D: f64 = 40.0; // depth
const PT: f64 = 8.0; // plate_t
const WH: f64 = 32.0; // wall_h
const BOSS_R: f64 = 10.0;
const HOLE_R: f64 = 4.0;
const MOUNT_R: f64 = 3.0;
const MOUNT_INSET: f64 = 16.0;
const FILLET_R: f64 = 3.0;

/// ZYX-Euler rotate-about-x-90° + translate, matching OpenZCAD's
/// `transformMatrix`: (x, y, z) → (x, -z, y) + t.
fn rot_x90_translate(tx: f64, ty: f64, tz: f64) -> Mat4 {
    Mat4::translation(tx, ty, tz) * Mat4::rotation_x(std::f64::consts::FRAC_PI_2)
}

fn build_bracket(topo: &mut Topology) -> SolidId {
    // Rev A: L blank.
    let base = make_box(topo, W, D, PT).expect("base plate");
    let wall = make_box(topo, W, PT, WH).expect("wall plate");
    transform_solid(topo, wall, &Mat4::translation(0.0, D - PT, PT - 0.5)).expect("seat wall");
    let l_blank = boolean(topo, BooleanOp::Fuse, base, wall).expect("union L bracket");

    // Rev B: boss + holes.
    let boss = make_cylinder(topo, BOSS_R, PT + 4.0).expect("boss");
    transform_solid(
        topo,
        boss,
        &rot_x90_translate(W / 2.0, D - PT + 2.0, PT + WH / 2.0),
    )
    .expect("place boss");
    let with_boss = boolean(topo, BooleanOp::Fuse, l_blank, boss).expect("union boss");

    let bore = make_cylinder(topo, HOLE_R, WH + 16.0).expect("bore");
    transform_solid(
        topo,
        bore,
        &rot_x90_translate(W / 2.0, D + 8.0, PT + WH / 2.0),
    )
    .expect("aim bore");
    let bored = boolean(topo, BooleanOp::Cut, with_boss, bore).expect("boss bore");

    let mount_a = make_cylinder(topo, MOUNT_R, PT + 4.0).expect("mount hole L");
    transform_solid(
        topo,
        mount_a,
        &Mat4::translation(MOUNT_INSET, D / 2.0, -2.0),
    )
    .expect("place mount L");
    let cut_a = boolean(topo, BooleanOp::Cut, bored, mount_a).expect("cut mount L");

    let mount_b = make_cylinder(topo, MOUNT_R, PT + 4.0).expect("mount hole R");
    transform_solid(
        topo,
        mount_b,
        &Mat4::translation(W - MOUNT_INSET, D / 2.0, -2.0),
    )
    .expect("place mount R");
    boolean(topo, BooleanOp::Cut, cut_a, mount_b).expect("cut mount R")
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 0.1
}

/// The four z-spanning edges at the (x, y) corners of the base plate,
/// mirroring the demo's fingerprint pick (all points in the corner column,
/// z within [-0.1, 8.1], z-span >= 4, xy-span <= 1.5).
fn pick_corner_edges(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    let mut picked = Vec::new();
    for eid in remus_topology::explorer::solid_edges(topo, solid).expect("edges") {
        let edge = topo.edge(eid).expect("edge");
        let a = topo.vertex(edge.start()).expect("v").point();
        let b = topo.vertex(edge.end()).expect("v").point();
        let corner =
            |px: f64, py: f64| (near(px, 0.0) || near(px, W)) && (near(py, 0.0) || near(py, D));
        let in_z = |pz: f64| (-0.1..=8.1).contains(&pz);
        if corner(a.x(), a.y())
            && corner(b.x(), b.y())
            && in_z(a.z())
            && in_z(b.z())
            && (a.x() - b.x()).abs() <= 1.5
            && (a.y() - b.y()).abs() <= 1.5
            && (a.z() - b.z()).abs() >= 4.0
        {
            picked.push(eid);
        }
    }
    picked
}

fn counts(topo: &Topology, solid: SolidId) -> (usize, usize, usize) {
    remus_topology::explorer::solid_entity_counts(topo, solid).expect("counts")
}

/// Mesh edges with incidence != 2, after welding coincident vertices by
/// quantized position. Mirrors the OpenZCAD parity harness's `meshEdgeUse`.
fn mesh_boundary_edges(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
    angle: f64,
    weld: f64,
) -> (usize, usize) {
    use std::collections::HashMap;
    // The OpenZCAD adapter consumes the GROUPED entry point (per-face
    // triangle ranges), so probe that one.
    let mesh = remus_operations::tessellate::tessellate_solid_grouped_with_tolerance(
        topo, solid, deflection, angle,
    )
    .expect("tessellate")
    .0;
    let q = 1.0 / weld;
    let mut canon: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut remap = vec![0u32; mesh.positions.len()];
    for (i, p) in mesh.positions.iter().enumerate() {
        #[allow(clippy::cast_possible_truncation)]
        let key = (
            (p.x() * q).round() as i64,
            (p.y() * q).round() as i64,
            (p.z() * q).round() as i64,
        );
        #[allow(clippy::cast_possible_truncation)]
        let next = canon.len() as u32;
        remap[i] = *canon.entry(key).or_insert(next);
    }
    let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let v = [
            remap[tri[0] as usize],
            remap[tri[1] as usize],
            remap[tri[2] as usize],
        ];
        for &(a, b) in &[(v[0], v[1]), (v[1], v[2]), (v[2], v[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    (edges.len(), edges.values().filter(|&&c| c != 2).count())
}

/// All four base-plate corner edges filleted at once (the demo's Rev C).
#[test]
fn bracket_corner_fillet_v2_all_corners() {
    let mut topo = Topology::new();
    let bracket = build_bracket(&mut topo);
    let edges = pick_corner_edges(&topo, bracket);
    assert_eq!(edges.len(), 4, "expected 4 corner edges");
    let before = counts(&topo, bracket);
    let vol_before = solid_volume(&topo, bracket, 0.1).expect("volume before");

    let result = remus_operations::blend_ops::fillet_v2(&mut topo, bracket, &edges, FILLET_R)
        .expect("fillet_v2 on bracket corners");
    assert!(!result.is_partial, "all 4 corners must succeed");
    assert_eq!(result.succeeded.len(), 4);

    let out = result.solid;
    let after = counts(&topo, out);
    assert_ne!(before, after, "fillet was a silent no-op");

    let shell_id = topo.solid(out).expect("solid").outer_shell();
    validate_shell_closed(topo.shell(shell_id).expect("shell"), &topo)
        .expect("filleted bracket shell must be closed");

    // Two full-height corners remove (r² − πr²/4)·8 ≈ 15.45 each; the two
    // rear corners stop at the wall seat (h = 7.5) and remove ≈ 14.49 each.
    let vol_after = solid_volume(&topo, out, 0.1).expect("volume after");
    let removed = vol_before - vol_after;
    assert!(
        (50.0..=70.0).contains(&removed),
        "expected ≈60 mm³ removed by 4 corner fillets, got {removed:.3}"
    );

    // The mesh must be watertight too: the OpenZCAD viewer and STL export
    // consume the tessellation, not the B-Rep. Checked at both a fine
    // tolerance and the consumer's coarser one (deflection 0.08, angle 0.35,
    // welded at 1e-4 — the OpenZCAD kernel-adapter settings).
    for &(deflection, angle, weld) in &[(0.01, 0.1, 1e-6), (0.08, 0.35, 1e-4)] {
        let (edges, boundary) = mesh_boundary_edges(&topo, out, deflection, angle, weld);
        assert_eq!(
            boundary, 0,
            "filleted bracket mesh not watertight at deflection {deflection}: \
             {boundary}/{edges} edges with incidence != 2"
        );
    }
}

/// Each corner individually must also succeed (front corners end on real
/// caps, rear corners end against coplanar slivers needing corner patches).
#[test]
fn bracket_corner_fillet_v2_each_corner() {
    for idx in 0..4 {
        let mut topo = Topology::new();
        let bracket = build_bracket(&mut topo);
        let edges = pick_corner_edges(&topo, bracket);
        assert_eq!(edges.len(), 4);
        let eid = edges[idx];
        let vol_before = solid_volume(&topo, bracket, 0.1).expect("volume before");

        let result = remus_operations::blend_ops::fillet_v2(&mut topo, bracket, &[eid], FILLET_R)
            .unwrap_or_else(|e| panic!("fillet_v2 on corner {idx} failed: {e}"));
        let vol_after = solid_volume(&topo, result.solid, 0.1).expect("volume after");
        let removed = vol_before - vol_after;
        assert!(
            (10.0..=20.0).contains(&removed),
            "corner {idx}: expected ≈15 mm³ removed, got {removed:.3}"
        );

        let shell_id = topo.solid(result.solid).expect("solid").outer_shell();
        validate_shell_closed(topo.shell(shell_id).expect("shell"), &topo)
            .unwrap_or_else(|e| panic!("corner {idx}: shell not closed: {e:?}"));
    }
}
