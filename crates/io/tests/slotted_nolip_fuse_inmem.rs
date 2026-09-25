//! Fusing the four-socket assembly onto a 2x2 slotted no-lip bin body must
//! stay analytic. FIXED by the within-rank cross-shell gate in
//! detect_same_domain: the fuse is watertight, keeps the full analytic mix,
//! and its volume equals the operand sum exactly. Historically it aborted
//! with "open hole shell with 45 faces", dropped to the mesh fallback, and
//! the fallback's open output carried 107-109 boundary edges into the
//! export (the `2x2 slotted no lip` export-integrity failure).
//!
//! Both operands are clean: the body (F=56, 8 cylinders, watertight) and the
//! socket assembly (F=136, 32 cones + 32 cylinders, watertight). Every other
//! boolean in the export chain replays clean and analytic; this fuse is the
//! sole leak producer. The failure reproduces identically on kernels from
//! before and after the 2026-08-04/05 engine work, so the trigger is the
//! tool's generator changes (the #3223-#3227 era) reshaping this
//! configuration, not an engine regression.
//!
//! Operands captured 2026-08-05 via the kernel-test boolean monkey-patch on
//! the failing export scenario (call 009 of 10).
//!
//! BK_OPEN_SHELL characterization: the aborting 45-face shell has signed
//! volume -51259 and is built from the BODY's own faces (src 10-13, the
//! outer walls and corner cylinders) — the fuse classifies a body-sized
//! chunk as a hole shell, the "no outer shell / misgrouped interior" family
//! rather than a small-fragment drop.
//!
//! ROOT MEASURED (BK_TRACE + BK_SD + BK_OPEN_SHELL): every free edge of the
//! aborting shell lies at z=21 — the top rim of the body's cavity-wall band.
//! All 191 classified sub-faces were Outside (nothing dropped by
//! classification); the missing face was the body's cavity CEILING Id(31)
//! (32-edge plane at z=21), which detect_same_domain declared a WITHIN-RANK
//! DUPLICATE of the body's exterior top disc Id(9) (8-edge plane, same z=21
//! plane) and dropped before classification. The two faces are coplanar
//! because the no-lip bin has a zero-thickness roof (the cavity ceiling and
//! the exterior top coincide), and the rank-agnostic geometric-containment
//! pass unioned them; the within-rank emission then treated group
//! membership as the #696 residue signature. Extent (edge-set equality)
//! and outward orientation were both tried as discriminants and REFUTED by
//! measurement: the honeycomb's true residue caps and this load-bearing
//! pair produce identical signatures on both (same_outward=Some(true),
//! coextensive=false). The discriminant that holds is SHELL MEMBERSHIP:
//! Id(9) lives in the body's outer shell and Id(31) in its inner (void)
//! shell — residue accumulates within one shell, while a cross-shell
//! coincidence is structural. detect_same_domain now takes the operands'
//! face-to-shell map and skips within-rank dedup for cross-shell pairs.
//!
//! EXPORT-LEVEL NOTE: on the conflict re-cast kernel (the O-shape fix) the
//! export test already passed because the chain's upstream booleans
//! classified differently and stopped feeding this operand pair into the
//! final fuse; the captured operands still reproduced the abort until the
//! coextensivity gate closed the root itself.
//!
//! ORIENTATION REPAIR (B59, 2026-09-25): the body as captured was not a
//! consistently oriented solid. Its void ceiling Id(31) (the z=21 plane of
//! the inner shell) faced +z, into the roof material, instead of -z into the
//! void; every other face agreed with its neighbours. The B-Rep still read
//! closed (each edge used twice), but the mesh traversed the ceiling's 48
//! rim half-edges in the same direction as the cavity walls
//! (`boundary_edge_count` 48 at 0.05, 64 at 0.01), and the signed volume
//! depended on the reference point: 106091.8 about the origin, 49075.1
//! about the bbox centre. The true volume is 13990.87 (closed form below).
//! Face-by-face attribution showed every face's triangles following that
//! face's declared outward normal, so the tessellator was faithful to bad
//! data. The capture's producer, a 2026-08-05 upstream boolean inside the
//! retired gridfinity tool chain, cannot be replayed here. The fixture was
//! repaired in place: Id(31) now has `reversed = true`, and the four
//! reversed cavity-corner cylinders Id(11/19/21/29), whose wires used the
//! legacy material-direction convention, were rewound to the kernel's
//! `forward XOR reversed` convention. The body now passes strict
//! `validate_solid`. The same-domain gate above still carries the fuse:
//! with the cross-shell check disabled, the repaired operands abort with
//! the same "open hole shell with 45 faces".

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use remus_io::arena_io::deserialize_solid;
use remus_math::mat::Mat4;
use remus_operations::measure::{oriented_solid_volume, solid_volume};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_faces;

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
}

fn load(name: &str, topo: &mut Topology) -> remus_topology::solid::SolidId {
    deserialize_solid(&std::fs::read(fixture(name)).unwrap(), topo).unwrap()
}

fn health(topo: &Topology, sid: remus_topology::solid::SolidId) -> (usize, usize, usize) {
    let faces = solid_faces(topo, sid).unwrap();
    let mut uses: HashMap<EdgeId, usize> = HashMap::new();
    let mut curved = 0;
    for &fid in &faces {
        let face = topo.face(fid).unwrap();
        if face.surface().type_tag() != "plane" {
            curved += 1;
        }
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                *uses.entry(oe.edge()).or_default() += 1;
            }
        }
    }
    let free = uses.values().filter(|&&c| c == 1).count();
    let over = uses.values().filter(|&&c| c > 2).count();
    (free, over, curved)
}

#[test]
fn slotted_operands_are_clean() {
    let mut topo = Topology::new();
    for name in ["slotted_nolip_body.bin", "slotted_socket_assembly.bin"] {
        let sid = load(name, &mut topo);
        let (free, over, curved) = health(&topo, sid);
        assert_eq!((free, over), (0, 0), "{name} must be closed and manifold");
        assert!(curved > 0, "{name} must keep analytic curved faces");
        // Edge counts alone passed the mis-oriented capture; strict
        // validation checks that each shared edge is used once per direction.
        let issues = validate_solid(&topo, sid).unwrap().issues;
        assert!(
            issues.is_empty(),
            "{name} must validate strictly: {issues:?}"
        );
        for deflection in [0.05, 0.01] {
            assert_mesh_closed(&topo, sid, deflection, name);
        }
    }
}

/// Every mesh half-edge must have a reversed twin. A face meshed against its
/// neighbours' winding leaves its rim half-edges unpaired even though every
/// edge is still shared by exactly two triangles.
fn assert_mesh_closed(
    topo: &Topology,
    sid: remus_topology::solid::SolidId,
    deflection: f64,
    what: &str,
) {
    let mesh = tessellate_solid(topo, sid, deflection).unwrap();
    assert_eq!(
        (boundary_edge_count(&mesh), non_manifold_edge_count(&mesh)),
        (0, 0),
        "{what} mesh at {deflection} must pair every half-edge with a reversed twin"
    );
}

/// Closed-form volume of the slotted no-lip body, from its own dimensions:
/// an 83.5 mm rounded square (corner r 3.75) over z 5..21, less the 81.1 mm
/// rounded-square void (corner r 2.55) over z 6.2..21, less six 0.6 x 2.1
/// wall slots over the void's 14.8 mm height.
fn body_closed_form_volume() -> f64 {
    let corner_loss = 4.0 - std::f64::consts::PI;
    let outer = 3.75f64.mul_add(-3.75 * corner_loss, 83.5 * 83.5) * 16.0;
    let void = 2.55f64.mul_add(-2.55 * corner_loss, 81.1 * 81.1) * 14.8;
    let slots = 6.0 * 0.6 * 2.1 * 14.8;
    outer - void - slots
}

#[test]
fn slotted_body_volume_matches_closed_form_off_origin() {
    let mut topo = Topology::new();
    let body = load("slotted_nolip_body.bin", &mut topo);
    let expected = body_closed_form_volume();

    let exact = solid_volume(&topo, body, 0.05).unwrap();
    assert!(
        (exact - expected).abs() < 0.01,
        "body volume {exact:.4} must match the closed form {expected:.4}"
    );
    // The chordal mesh sits a few mm^3 below the exact value at 0.05.
    let mesh_in_place = oriented_solid_volume(&topo, body, 0.05).unwrap();
    assert!(
        (mesh_in_place - expected).abs() < 5e-4 * expected,
        "mesh volume {mesh_in_place:.4} must be within 0.05% of {expected:.4}"
    );

    // A mis-wound face in a plane through the origin contributes nothing to
    // an origin-referenced sum; off the origin it does.
    transform_solid(&mut topo, body, &Mat4::translation(120.0, -80.0, 45.0));
    let mesh_moved = oriented_solid_volume(&topo, body, 0.05).unwrap();
    assert!(
        (mesh_moved - mesh_in_place).abs() < 1e-6 * expected,
        "translated body volume {mesh_moved:.6} must equal the in-place {mesh_in_place:.6}"
    );
    assert_mesh_closed(&topo, body, 0.05, "translated body");
}

#[test]
fn slotted_nolip_socket_fuse_is_analytic_watertight() {
    let mut topo = Topology::new();
    let body = load("slotted_nolip_body.bin", &mut topo);
    let sockets = load("slotted_socket_assembly.bin", &mut topo);
    let vol_body = oriented_solid_volume(&topo, body, 0.05).unwrap();
    let vol_sockets = oriented_solid_volume(&topo, sockets, 0.05).unwrap();

    let result =
        remus_algo::gfa::boolean(&mut topo, remus_algo::bop::BooleanOp::Fuse, body, sockets)
            .expect("analytic fuse should not abort");

    let (free, over, curved) = health(&topo, result);
    assert!(curved > 0, "all-planar output is the mesh-fallback tell");
    assert_eq!(over, 0, "fuse must stay manifold, got {over} over-shared");
    assert_eq!(free, 0, "fuse must be closed, got {free} free edges");

    let issues = validate_solid(&topo, result).unwrap().issues;
    assert!(issues.is_empty(), "fuse must validate strictly: {issues:?}");
    assert_mesh_closed(&topo, result, 0.05, "fuse");

    // The socket assembly attaches below the body without overlapping it, so
    // the fuse volume must equal the operand sum (measured 43133.580 =
    // 13987.856 + 29145.724).
    let vol = oriented_solid_volume(&topo, result, 0.05).unwrap();
    assert!(
        (vol - (vol_body + vol_sockets)).abs() < 0.01,
        "fuse volume {vol:.3} must equal the operand sum {:.3}",
        vol_body + vol_sockets
    );
}

#[test]
fn slotted_nolip_socket_fuse_volume_holds_off_origin() {
    let mut topo = Topology::new();
    let body = load("slotted_nolip_body.bin", &mut topo);
    let sockets = load("slotted_socket_assembly.bin", &mut topo);
    let in_place_sum = oriented_solid_volume(&topo, body, 0.05).unwrap()
        + oriented_solid_volume(&topo, sockets, 0.05).unwrap();

    // Move the whole assembly rigidly so no face lies in a plane through
    // the origin.
    let offset = Mat4::translation(120.0, -80.0, 45.0);
    transform_solid(&mut topo, body, &offset);
    transform_solid(&mut topo, sockets, &offset);
    let sum = oriented_solid_volume(&topo, body, 0.05).unwrap()
        + oriented_solid_volume(&topo, sockets, 0.05).unwrap();
    assert!(
        (sum - in_place_sum).abs() < 1e-6 * in_place_sum,
        "translated operand sum {sum:.6} must equal the in-place {in_place_sum:.6}"
    );

    let result =
        remus_algo::gfa::boolean(&mut topo, remus_algo::bop::BooleanOp::Fuse, body, sockets)
            .expect("translated analytic fuse should not abort");
    let (free, over, curved) = health(&topo, result);
    assert_eq!(
        (free, over),
        (0, 0),
        "translated fuse must be closed and manifold"
    );
    assert!(curved > 0, "all-planar output is the mesh-fallback tell");
    assert_mesh_closed(&topo, result, 0.05, "translated fuse");

    let vol = oriented_solid_volume(&topo, result, 0.05).unwrap();
    assert!(
        (vol - sum).abs() < 0.01,
        "translated fuse volume {vol:.3} must equal the operand sum {sum:.3}"
    );
}

fn transform_solid(topo: &mut Topology, sid: remus_topology::solid::SolidId, m: &Mat4) {
    remus_operations::transform::transform_solid(topo, sid, m).unwrap();
}
