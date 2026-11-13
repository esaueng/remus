//! B7 ready-repros: face–face section curves never receive interior paves.
//!
//! Both cases are a three-step primitive chain on a 10 × 10 × 4 plate: a
//! vertical bore (r 2, axis (5, 5)), then a horizontal tunnel (r 1.2, axis
//! y = 6, z = 2) whose wall meets the bore wall in a MARCHED (NURBS) loop,
//! then a third cutter. The third step's operand therefore carries marched
//! edges on curved faces, which is the condition the cross-face desync family
//! needs.
//!
//! Root (traced with an env-gated `eprintln!` after `make_split_edges`, kept
//! out of the tree): every face–face curve gets exactly ONE pave block from
//! its two endpoints in `phase_ff`, and `make_blocks` only splits operand-edge
//! pave blocks. Nothing puts an EF pave, or an existing face vertex, that lies
//! inside a section's span onto that section's pave block, and nothing checks
//! that each piece of a section lies on BOTH faces. Splitting and keep/drop
//! decisions are instead taken per face in the builder, independently, so the
//! two faces sharing a section disagree.
//!
//! - `coaxial_wider_half_tunnel`: the third cutter is a coaxial r 1.5 tunnel
//!   ending at x = 6. EF correctly splits the old marched loop edges where
//!   they cross the cutter's end cap (two leaves each), but the bore-wall ×
//!   end-cap section (the bore generator at x = 6) keeps `extra_paves = 0`:
//!   it is never broken where it enters and leaves the bore wall's existing
//!   hole, the in-hole middle is never dropped, and the old loop survives
//!   unsplit with one use per edge.
//! - `oblique_tunnel_crossing`: the third cutter is an oblique r 0.8 tunnel at
//!   30°, z = 2.3. Its section with the bore wall lies entirely inside the
//!   bore wall's existing tunnel hole (the analytic `FaceExtent` used by
//!   `restrict_curves_to_faces` ignores inner wires), so the whole marched
//!   loop is emitted on the bore face with nothing on the other side.
//!
//! Oracles are independent of the kernel: edge use per `EdgeId`,
//! position-quantized edge pairing, and volume from a column integration of
//! the primitives' analytic membership (h = 0.004, 0.002 and 0.001, then
//! Richardson-extrapolated; the same integration reproduces the kernel's
//! clean step-2 volume to about 1e-5 relative). Assertions target the raw GFA
//! (`remus_algo::gfa::boolean`), the layer B7 owns.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_possible_truncation
)]

use std::collections::{BTreeMap, HashMap};
use std::f64::consts::{FRAC_PI_2, FRAC_PI_6};

use remus_algo::bop::BooleanOp;
use remus_check::properties::PropertiesOptions;
use remus_math::mat::Mat4;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

type Q = (i64, i64, i64);

fn placed(topo: &mut Topology, solid: SolidId, m: &Mat4) -> SolidId {
    transform_solid(topo, solid, m).unwrap();
    solid
}

/// Plate minus the vertical bore minus the horizontal tunnel, then minus
/// `third`, all through the raw GFA.
fn chain(topo: &mut Topology, third: SolidId) -> SolidId {
    let plate = make_box(topo, 10.0, 10.0, 4.0).unwrap();
    let bore = make_cylinder(topo, 2.0, 6.0).unwrap();
    let bore = placed(topo, bore, &Mat4::translation(5.0, 5.0, -1.0));
    let tunnel = make_cylinder(topo, 1.2, 12.0).unwrap();
    let tunnel = placed(
        topo,
        tunnel,
        &(Mat4::translation(-1.0, 6.0, 2.0) * Mat4::rotation_y(FRAC_PI_2)),
    );
    let s = remus_algo::gfa::boolean(topo, BooleanOp::Cut, plate, bore).unwrap();
    let s = remus_algo::gfa::boolean(topo, BooleanOp::Cut, s, tunnel).unwrap();
    remus_algo::gfa::boolean(topo, BooleanOp::Cut, s, third).expect("third cut builds")
}

/// The desired result: every edge used exactly twice, by id and by quantized
/// end positions, and the volume within 2e-4 of the independent reference.
/// All three oracles are evaluated before the single assertion, so a failure
/// reports every reading.
fn assert_desired(topo: &Topology, solid: SolidId, expected_volume: f64, tag: &str) {
    let mut by_id: BTreeMap<usize, usize> = BTreeMap::new();
    let mut by_pos: HashMap<(Q, Q), usize> = HashMap::new();
    let q = |v: f64| (v * 1.0e5).round() as i64;
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                *by_id.entry(oe.edge().index()).or_default() += 1;
                let e = topo.edge(oe.edge()).unwrap();
                let a = topo.vertex(e.start()).unwrap().point();
                let b = topo.vertex(e.end()).unwrap().point();
                let ka = (q(a.x()), q(a.y()), q(a.z()));
                let kb = (q(b.x()), q(b.y()), q(b.z()));
                *by_pos
                    .entry(if ka <= kb { (ka, kb) } else { (kb, ka) })
                    .or_default() += 1;
            }
        }
    }
    let bad_id = by_id.values().filter(|&&c| c != 2).count();
    let bad_pos = by_pos.values().filter(|&&c| c != 2).count();
    // An open shell can make the face integration fail; record that as a
    // non-finite reading rather than a setup panic.
    let volume = remus_check::properties::solid_volume(topo, solid, &PropertiesOptions::default())
        .unwrap_or(f64::NAN);
    let rel = (volume - expected_volume).abs() / expected_volume;
    assert!(
        bad_id == 0 && bad_pos == 0 && rel < 2e-4,
        "{tag}: edges not used twice = {bad_id}, unpaired position-quantized edges = \
         {bad_pos}, volume {volume:.6} vs reference {expected_volume} (rel {rel:.2e}); \
         want 0, 0, rel < 2e-4"
    );
}

/// Raw GFA today: 4 edges used once, volume 319.757 (above the 318.49 of the
/// step-2 body it cut). Oracle 308.387.
#[test]
#[ignore = "open: section through an operand hole never gets the EF paves that split it (B7)"]
fn coaxial_wider_half_tunnel() {
    let mut topo = Topology::new();
    let third = make_cylinder(&mut topo, 1.5, 7.0).unwrap();
    let third = placed(
        &mut topo,
        third,
        &(Mat4::translation(-1.0, 6.0, 2.0) * Mat4::rotation_y(FRAC_PI_2)),
    );
    let result = chain(&mut topo, third);
    assert_desired(&topo, result, 308.387, "coaxial wider half tunnel");
}

/// Raw GFA today: 7 edges not used twice (a closed marched loop used once on
/// the bore wall, the old loop used three times), volume 294.378. Oracle
/// 307.568.
#[test]
#[ignore = "open: marched section inside a curved face's hole is kept on that face (B7)"]
fn oblique_tunnel_crossing() {
    let mut topo = Topology::new();
    let third = make_cylinder(&mut topo, 0.8, 16.0).unwrap();
    let third = placed(
        &mut topo,
        third,
        &(Mat4::translation(5.0, 5.0, 2.3)
            * Mat4::rotation_z(FRAC_PI_6)
            * Mat4::translation(-8.0, 0.0, 0.0)
            * Mat4::rotation_y(FRAC_PI_2)),
    );
    let result = chain(&mut topo, third);
    assert_desired(&topo, result, 307.568, "oblique tunnel crossing");
}
