//! B7 ready-repros: face–face section curves never receive interior paves.
//!
//! Both cases are a three-step primitive chain on a 10 × 10 × 4 plate: a
//! vertical bore (r 2, axis (5, 5)), then a horizontal tunnel (r 1.2, axis
//! y = 6, z = 2) whose wall meets the bore wall in a MARCHED (NURBS) loop,
//! then a third cutter. The third step's operand therefore carries marched
//! edges on curved faces and holes in curved faces, which is the condition the
//! cross-face desync family needs.
//!
//! Root (traced with an env-gated `eprintln!` of the GFA arena after
//! `make_split_edges`, kept out of the tree): every face–face curve gets
//! exactly ONE pave block from its two endpoints in `phase_ff`, and
//! `make_blocks` only splits operand-edge pave blocks. Nothing puts an EF
//! pave, or an existing face vertex, that lies inside a section's span onto
//! that section's pave block, and nothing checks that each piece of a section
//! lies on BOTH faces. Splitting and keep/drop decisions are instead taken per
//! face in the builder, independently, so the two faces sharing a section
//! disagree.
//!
//! What these repros do and do not certify: B7 is named for pave-block
//! attachment of MARCHED sections, but the coaxial case's failing sections
//! are a Line and a Circle (analytic; the Line reaches the builder through
//! the per-pave-block path in `fill_images_faces.rs`, the Circle as one whole
//! curve), and the oblique case's in-hole loop survives because the
//! analytic-face extent used by `restrict_curves_to_faces`
//! (`FaceExtent::Analytic` in `phase_ff.rs`) has no hole field, a restriction
//! gap rather than a pave gap. Only the oblique exit (marched sections C6/C7
//! below) is a marched section ending on unshared vertices. A fix that
//! attaches paves to marched curves alone will not turn these green; they
//! certify the general section-pave pass the roadmap row's fix site names.
//!
//! - `coaxial_wider_half_tunnel`: the third cutter is a coaxial r 1.5 tunnel
//!   ending at x = 6. EF correctly splits the old bore × tunnel loop where it
//!   crosses the cutter's end-cap plane, at (6, 6.732, 1.049) and
//!   (6, 6.732, 2.951), but neither section through those points takes them:
//!   - the bore wall × end cap Line (the bore generator at x = 6, y = 6.732)
//!     spans the bore face's whole height z 0..4, with no pave at the end-cap
//!     disc rim (z 0.691 and 3.309, where the marched bore × cutter-wall
//!     section ends) or where it enters and leaves the bore wall's tunnel
//!     hole;
//!   - the old tunnel wall × end cap Circle (r 1.2 at x = 6) stays one closed
//!     block, never broken where it enters the bore.
//!
//!   The result has no r 1.2 tunnel-wall face and no annular end-cap face at
//!   all: the tunnel wall's x 6..10 piece is lost, so its far rim (the
//!   r 1.2 circle on the x = 10 plate face) and the old loop (left unsplit
//!   on the bore wall) are each used once: 4 edges.
//! - `oblique_tunnel_crossing`: the third cutter is an oblique r 0.8 tunnel at
//!   30°, z = 2.3, through the bore axis, so it meets the bore wall in two
//!   closed marched loops. The entry loop (near (3.0, 4.8)) lies in solid
//!   bore wall and is built correctly; the exit loop (near (7.0, 5.2)) lies
//!   entirely inside the bore wall's existing tunnel hole and is kept on the
//!   bore face anyway (the hole-blind `FaceExtent::Analytic`), used once. At
//!   the plate's x = 10 face the cutter's elliptical rim crosses the tunnel
//!   opening at (10, 7.034, 2.608) and (10, 7.180, 1.784). The tunnel's
//!   x = 10 rim circle gets no EF pave there, and the plate × cutter ellipse
//!   arcs and the tunnel × cutter marched sections (C6/C7 in the arena) that
//!   meet at each point end on DIFFERENT vertices at the same position. The
//!   plate face splits its hole correctly; the tunnel wall instead keeps the
//!   arc of its rim that lies inside the cutter (used once), and C6/C7 are
//!   used once each, by cutter faces only. Why the tunnel wall declines
//!   C6/C7 is not traced past the missing paves. The old bore × tunnel loop
//!   is used three times (once by the bore wall, and as both an outer and an
//!   inner wire of the tunnel wall); that duplication is not diagnosed.
//!   Together: 7 edges.
//!
//! Oracles are independent of the kernel: edge use per `EdgeId`;
//! position-keyed edge pairing (quantized endpoints plus the arc-length
//! midpoint, so a closed curve split into two arcs between the same two
//! vertices pairs arc by arc); point classification probes; and volume. The
//! repro volumes come from a column integration of the primitives' analytic
//! membership (h = 0.004, 0.002 and 0.001, then Richardson-extrapolated); the
//! controls' from closed-form 1-D integrals. The two controls pass today and
//! pin the oracles to valid topology, including two-arc split loops.
//! Assertions target the raw GFA (`remus_algo::gfa::boolean`), the layer B7
//! owns.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss
)]

use std::collections::{BTreeMap, HashMap};
use std::f64::consts::{FRAC_PI_2, FRAC_PI_6, PI};

use remus_algo::bop::BooleanOp;
use remus_check::properties::PropertiesOptions;
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

type Q = (i64, i64, i64);

/// Two edge uses whose endpoint keys match pair only when their arc-length
/// midpoints are this close: far below any feature here, far above the
/// sampling error of the midpoint.
const MIDPOINT_TOL: f64 = 1.0e-3;

fn placed(topo: &mut Topology, solid: SolidId, m: &Mat4) -> SolidId {
    transform_solid(topo, solid, m).unwrap();
    solid
}

/// Plate minus the vertical bore minus the horizontal tunnel, through the raw
/// GFA: the clean step-2 body.
fn plate_bore_tunnel(topo: &mut Topology) -> SolidId {
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
    remus_algo::gfa::boolean(topo, BooleanOp::Cut, s, tunnel).unwrap()
}

/// The step-2 body minus `third`.
fn chain(topo: &mut Topology, third: SolidId) -> SolidId {
    let s = plate_bore_tunnel(topo);
    remus_algo::gfa::boolean(topo, BooleanOp::Cut, s, third).expect("third cut builds")
}

/// The oblique r 0.8 cutter: axis through (5, 5, 2.3) at 30° to x, length 16.
fn oblique_cutter(topo: &mut Topology) -> SolidId {
    let third = make_cylinder(topo, 0.8, 16.0).unwrap();
    placed(
        topo,
        third,
        &(Mat4::translation(5.0, 5.0, 2.3)
            * Mat4::rotation_z(FRAC_PI_6)
            * Mat4::translation(-8.0, 0.0, 0.0)
            * Mat4::rotation_y(FRAC_PI_2)),
    )
}

/// The point halfway along `edge` by arc length (64-chord polyline).
fn arc_length_midpoint(topo: &Topology, edge: EdgeId) -> Point3 {
    const N: usize = 64;
    let e = topo.edge(edge).unwrap();
    let a = topo.vertex(e.start()).unwrap().point();
    let b = topo.vertex(e.end()).unwrap().point();
    // Stored parameter authority first; edges without it fall back to the
    // named endpoint-reconstruction adapter (RFC 0002).
    let (t0, t1) = e
        .strict_domain()
        .unwrap_or_else(|_| e.curve().reconstruct_domain_from_endpoints(a, b));
    let pts: Vec<Point3> = (0..=N)
        .map(|i| {
            e.curve()
                .evaluate_with_endpoints(t0 + (t1 - t0) * i as f64 / N as f64, a, b)
        })
        .collect();
    let lens: Vec<f64> = pts.windows(2).map(|w| (w[1] - w[0]).length()).collect();
    let mut remaining = lens.iter().sum::<f64>() / 2.0;
    for (w, len) in pts.windows(2).zip(&lens) {
        if remaining <= *len && *len > 0.0 {
            let s = remaining / len;
            return w[0] + (w[1] - w[0]) * s;
        }
        remaining -= len;
    }
    pts[N]
}

/// Edge-use readings of a solid: edges not used exactly twice (by id),
/// position-keyed edge classes not used exactly twice, and the number of
/// endpoint keys holding more than one position class (closed curves split
/// into arcs between the same two vertices).
fn edge_readings(topo: &Topology, solid: SolidId) -> (usize, usize, usize) {
    let mut by_id: BTreeMap<usize, usize> = BTreeMap::new();
    let mut by_pos: HashMap<(Q, Q), Vec<(Point3, usize)>> = HashMap::new();
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
                let mid = arc_length_midpoint(topo, oe.edge());
                let classes = by_pos
                    .entry(if ka <= kb { (ka, kb) } else { (kb, ka) })
                    .or_default();
                match classes
                    .iter_mut()
                    .find(|(m, _)| (*m - mid).length() < MIDPOINT_TOL)
                {
                    Some((_, uses)) => *uses += 1,
                    None => classes.push((mid, 1)),
                }
            }
        }
    }
    let bad_id = by_id.values().filter(|&&c| c != 2).count();
    let bad_pos = by_pos
        .values()
        .flatten()
        .filter(|(_, uses)| *uses != 2)
        .count();
    let split_keys = by_pos.values().filter(|classes| classes.len() > 1).count();
    (bad_id, bad_pos, split_keys)
}

fn volume_of(topo: &Topology, solid: SolidId) -> f64 {
    // An open shell can make the face integration fail; record that as a
    // non-finite reading rather than a setup panic.
    remus_check::properties::solid_volume(topo, solid, &PropertiesOptions::default())
        .unwrap_or(f64::NAN)
}

/// The desired result: every edge used exactly twice, by id and by position,
/// every probe classified as expected, and the volume within 2e-4 of the
/// independent reference. Every oracle is evaluated before the single
/// assertion, so a failure reports every reading.
fn assert_desired(
    topo: &Topology,
    solid: SolidId,
    expected_volume: f64,
    probes: &[([f64; 3], PointClassification)],
    tag: &str,
) {
    let (bad_id, bad_pos, _) = edge_readings(topo, solid);
    let wrong_probes: Vec<String> = probes
        .iter()
        .filter_map(|&([x, y, z], want)| {
            let got = classify_point(topo, solid, Point3::new(x, y, z), 0.01, 1.0e-6);
            (got.as_ref().ok() != Some(&want))
                .then(|| format!("({x}, {y}, {z}): {got:?}, want {want:?}"))
        })
        .collect();
    let volume = volume_of(topo, solid);
    let rel = (volume - expected_volume).abs() / expected_volume;
    assert!(
        bad_id == 0 && bad_pos == 0 && wrong_probes.is_empty() && rel < 2e-4,
        "{tag}: edges not used twice = {bad_id}, unpaired position-keyed edges = \
         {bad_pos}, wrong probes = {wrong_probes:?}, volume {volume:.6} vs reference \
         {expected_volume} (rel {rel:.2e}); want 0, 0, [], rel < 2e-4"
    );
}

/// Composite Simpson rule on `[a, b]` with `n` (even) intervals.
fn simpson(f: impl Fn(f64) -> f64, a: f64, b: f64, n: usize) -> f64 {
    let h = (b - a) / n as f64;
    let inner: f64 = (1..n)
        .map(|i| f(a + h * i as f64) * if i % 2 == 1 { 4.0 } else { 2.0 })
        .sum();
    (f(a) + f(b) + inner) * h / 3.0
}

/// Control: the clean step-2 body passes every oracle. Reference volume:
/// plate − bore − tunnel + (bore ∩ tunnel), the overlap integrated over y as
/// the product of the bore's x chord and the tunnel's z chord (both spans
/// cross the plate whole), with `y = 4.8 + 1.1 (1 − cos φ)` smoothing the
/// square-root ends.
#[test]
fn control_step_two_body_reads_clean() {
    let overlap = simpson(
        |phi| {
            let y = 1.1f64.mul_add(1.0 - phi.cos(), 4.8);
            let bore_chord = 2.0 * (4.0 - (y - 5.0).powi(2)).max(0.0).sqrt();
            let tunnel_chord = 2.0 * (1.44 - (y - 6.0).powi(2)).max(0.0).sqrt();
            bore_chord * tunnel_chord * 1.1 * phi.sin()
        },
        0.0,
        PI,
        4000,
    );
    let reference = 400.0 - 16.0 * PI - 14.4 * PI + overlap;
    let mut topo = Topology::new();
    let body = plate_bore_tunnel(&mut topo);
    assert_desired(
        &topo,
        body,
        reference,
        &[
            ([8.0, 6.0, 3.3], PointClassification::Inside),
            ([8.0, 6.0, 2.0], PointClassification::Outside),
            ([5.0, 5.0, 1.0], PointClassification::Outside),
        ],
        "step-2 control",
    );
}

/// Control: plate − bore − oblique cutter (no tunnel) holds closed curves
/// split into two arcs between the same two vertices (the cutter's rim on the
/// x = 0 face, its loops on the bore wall), and the position key pairs them.
/// Reference volume: plate − bore − cutter-in-plate + (bore ∩ cutter); the
/// cutter crosses the plate whole, so its part in the plate is
/// π r² · 10 / cos 30°, and each horizontal slice of the overlap is a disc
/// cut by a centred strip of half-width `w`, `2 (w √(4 − w²) + 4 asin(w / 2))`.
#[test]
fn control_split_loops_pair_by_position() {
    let overlap = simpson(
        |theta| {
            let w = 0.8 * theta.cos();
            2.0 * w.mul_add((4.0 - w * w).sqrt(), 4.0 * (w / 2.0).asin()) * w
        },
        -FRAC_PI_2,
        FRAC_PI_2,
        2000,
    );
    let reference = 400.0 - 16.0 * PI - 0.64 * PI * 10.0 / FRAC_PI_6.cos() + overlap;
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 10.0, 10.0, 4.0).unwrap();
    let bore = make_cylinder(&mut topo, 2.0, 6.0).unwrap();
    let bore = placed(&mut topo, bore, &Mat4::translation(5.0, 5.0, -1.0));
    let cutter = oblique_cutter(&mut topo);
    let s = remus_algo::gfa::boolean(&mut topo, BooleanOp::Cut, plate, bore).unwrap();
    let body = remus_algo::gfa::boolean(&mut topo, BooleanOp::Cut, s, cutter).unwrap();
    let (_, _, split_keys) = edge_readings(&topo, body);
    assert!(
        split_keys > 0,
        "control lost its two-arc split loops (split endpoint keys = 0)"
    );
    assert_desired(
        &topo,
        body,
        reference,
        &[
            ([1.969, 3.25, 2.3], PointClassification::Outside),
            ([2.0, 8.0, 1.0], PointClassification::Inside),
        ],
        "split-loop control",
    );
}

/// Raw GFA today: 4 edges used once (by id and by position), volume 319.757
/// (above the 318.49 of the step-2 body it cut). Oracle 308.387.
#[test]
#[ignore = "open: sections through an operand hole never get the EF paves that split them (B7)"]
fn coaxial_wider_half_tunnel() {
    let mut topo = Topology::new();
    let third = make_cylinder(&mut topo, 1.5, 7.0).unwrap();
    let third = placed(
        &mut topo,
        third,
        &(Mat4::translation(-1.0, 6.0, 2.0) * Mat4::rotation_y(FRAC_PI_2)),
    );
    let result = chain(&mut topo, third);
    assert_desired(
        &topo,
        result,
        308.387,
        &[
            // Inside the r 1.5 cutter, outside the r 1.2 tunnel.
            ([2.0, 6.0, 3.3], PointClassification::Outside),
            // Past the cutter's end: plate material over the tunnel.
            ([8.0, 6.0, 3.3], PointClassification::Inside),
        ],
        "coaxial wider half tunnel",
    );
}

/// Raw GFA today: 7 edges not used twice (by id and by position): the exit
/// loop used once on the bore wall, the old loop used three times, and the
/// rim arc plus two marched sections at the x = 10 exit used once each.
/// Volume 294.378. Oracle 307.568.
#[test]
#[ignore = "open: in-hole section kept and exit sections unshared on curved faces (B7)"]
fn oblique_tunnel_crossing() {
    let mut topo = Topology::new();
    let third = oblique_cutter(&mut topo);
    let result = chain(&mut topo, third);
    assert_desired(
        &topo,
        result,
        307.568,
        &[
            // On the cutter's axis, clear of the bore and the tunnel.
            ([1.969, 3.25, 2.3], PointClassification::Outside),
            ([2.0, 8.0, 1.0], PointClassification::Inside),
        ],
        "oblique tunnel crossing",
    );
}
