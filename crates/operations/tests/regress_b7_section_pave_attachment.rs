//! B7 regressions: a third cut through a body whose curved faces an earlier
//! cut has already trimmed.
//!
//! Both cases are a three-step primitive chain on a 10 × 10 × 4 plate: a
//! vertical bore (r 2, axis (5, 5)), then a horizontal tunnel (r 1.2, axis
//! y = 6, z = 2) whose wall meets the bore wall in a MARCHED (NURBS) loop,
//! then a third cutter. After the second cut the bore wall carries that loop
//! as a HOLE straddling its seam, and the tunnel wall carries it as a BAY in
//! its outer wire. Neither is a chart rectangle any more, and three stages
//! of the third cut assumed one:
//!
//! - **Section curves had no paves.** Every face–face curve got one
//!   end-to-end pave block in `phase_ff`, `make_blocks` split only
//!   operand-edge blocks, and the analytic extent `phase_ff` restricts curved
//!   sections with is a height band plus an angular gap. A section entering
//!   the hole or the bay reached both faces whole and each decided alone
//!   what to keep. `pave_filler::section_paves` now puts the trimming edges'
//!   pave vertices on the curve, splits its block, and drops the pieces off
//!   the face for both faces at once.
//! - **The classifier did not model the operand.** A cylinder wall with a
//!   non-band hole fell to the planar polygon fallback and a bayed wall read
//!   as a full band, so the whole bore void classified as INSIDE the step-2
//!   body (2,278 of 25,306 points of a quarter-unit grid) and void pieces of
//!   the cutter wall were kept. `classifier::ray_cast` now reads such a wall
//!   through its chart outlines.
//! - **No splitter joined a section to a hole or a bay.** The established
//!   paths route such a wall to the internal-loops emitter, which keeps
//!   closed loops only, and the certified chart partition admitted chart
//!   rectangles only. `face_splitter::trimmed_cylinder_chart` admits the
//!   holed and the bayed wall to the same certified arrangement.
//!
//! - `coaxial_wider_half_tunnel`: the third cutter is a coaxial r 1.5 tunnel
//!   ending at x = 6. Its end cap meets the bore wall in a ruling that runs
//!   through the hole, and the old tunnel wall in a circle that runs through
//!   the bay.
//! - `oblique_tunnel_crossing`: the third cutter is an oblique r 0.8 tunnel at
//!   30°, z = 2.3. One of its two loops on the bore wall lies wholly inside
//!   the hole, and one of its loops on the old tunnel wall's carrier lies
//!   wholly inside the bay.
//!
//! Oracles are independent of the kernel under test: edge use per `EdgeId`;
//! position-keyed edge pairing (quantized endpoints plus the arc-length
//! midpoint, so a closed curve split into two arcs between the same two
//! vertices pairs arc by arc); point classification probes, through both the
//! operations classifier and the check crate's ray cast, at points the
//! primitives decide; a grid classification of the step-2 operand with the
//! engine's own ray-cast classifier; and volume. The repro volumes come from
//! a column integration of the primitives' analytic membership (h = 0.004,
//! 0.002 and 0.001, then Richardson-extrapolated); the controls' from
//! closed-form 1-D integrals. The two controls pin the oracles to valid
//! topology, including two-arc split loops. Assertions target the raw GFA
//! (`remus_algo::gfa::boolean`), the layer B7 owns.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss
)]

use std::collections::{BTreeMap, HashMap};
use std::f64::consts::{FRAC_PI_2, FRAC_PI_6, PI};

use remus_algo::bop::BooleanOp;
use remus_check::classify::{ClassifyOptions, PointClassification as CheckClass};
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

/// Preserve the review's measured volumes and watertight display meshes.
fn assert_review_measurements(topo: &Topology, solid: SolidId, volume: f64, tag: &str) {
    use remus_operations::tessellate::{edge_census, is_watertight, tessellate_solid};

    let measured = volume_of(topo, solid);
    assert!((measured - volume).abs() < 1e-6, "{tag}: {measured:.9}");
    for deflection in [0.05, 0.01] {
        let mesh = tessellate_solid(topo, solid, deflection).unwrap();
        let census = edge_census(&mesh);
        assert!(is_watertight(&mesh), "{tag}: {deflection}, {census:?}");
    }
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

fn in_bore(x: f64, y: f64) -> bool {
    (x - 5.0).powi(2) + (y - 5.0).powi(2) < 4.0
}

fn in_tunnel(y: f64, z: f64) -> bool {
    (y - 6.0).powi(2) + (z - 2.0).powi(2) < 1.44
}

/// Ray-cast classification of `solid` (the check crate's classifier) at
/// points whose membership the primitives decide: `(point, expected inside)`.
fn assert_material(
    topo: &Topology,
    solid: SolidId,
    material: &[((f64, f64, f64), bool)],
    tag: &str,
) {
    let mut wrong = Vec::new();
    for &((x, y, z), inside) in material {
        let got = remus_check::classify::classify_point(
            topo,
            solid,
            Point3::new(x, y, z),
            &ClassifyOptions::default(),
        )
        .unwrap();
        let want = if inside {
            CheckClass::Inside
        } else {
            CheckClass::Outside
        };
        if got != want {
            wrong.push(((x, y, z), want, got));
        }
    }
    assert!(wrong.is_empty(), "{tag}: misclassified probes {wrong:?}");
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

/// The engine's own classifier must read the step-2 operand as its
/// primitives do: every later cut classifies the other operand's pieces
/// against it. Before the chart-outline wall model the bore void read as
/// material.
#[test]
fn operand_with_a_holed_and_a_bayed_wall_classifies_like_its_primitives() {
    let mut topo = Topology::new();
    let solid = plate_bore_tunnel(&mut topo);
    let geoms = remus_algo::classifier::RayCastGeoms::new(&topo, solid).unwrap();
    let mut wrong = Vec::new();
    let mut total = 0_usize;
    for i in 0..20 {
        for j in 0..20 {
            for k in 0..8 {
                let (x, y, z) = (
                    0.137 + 0.5 * f64::from(i),
                    0.113 + 0.5 * f64::from(j),
                    0.129 + 0.5 * f64::from(k),
                );
                // Stay clear of the two curved walls; the planar ones are
                // at least 0.1 away on this grid.
                let d_bore = ((x - 5.0).powi(2) + (y - 5.0).powi(2)).sqrt() - 2.0;
                let d_tunnel = ((y - 6.0).powi(2) + (z - 2.0).powi(2)).sqrt() - 1.2;
                if d_bore.abs() < 0.02 || d_tunnel.abs() < 0.02 {
                    continue;
                }
                let material = !in_bore(x, y) && !in_tunnel(y, z);
                let got = remus_algo::classifier::classify_point_cached(
                    &topo,
                    solid,
                    Some(&geoms),
                    Point3::new(x, y, z),
                );
                total += 1;
                if matches!(got, Ok(remus_algo::FaceClass::Inside)) != material {
                    wrong.push(((x, y, z), material, got));
                }
            }
        }
    }
    assert!(total > 3000, "grid too sparse: {total}");
    assert!(
        wrong.is_empty(),
        "{} of {total} grid points misclassified, first {:?}",
        wrong.len(),
        wrong.first()
    );
}

/// Before the fix: 4 edges used once (by id and by position), volume 319.757
/// (above the 318.49 of the step-2 body it cut). Oracle 308.387.
#[test]
fn coaxial_wider_half_tunnel() {
    let mut topo = Topology::new();
    let third = make_cylinder(&mut topo, 1.5, 7.0).unwrap();
    let third = placed(
        &mut topo,
        third,
        &(Mat4::translation(-1.0, 6.0, 2.0) * Mat4::rotation_y(FRAC_PI_2)),
    );
    let result = chain(&mut topo, third);
    assert_review_measurements(&topo, result, 308.385_322, "coaxial wider half tunnel");
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
    assert_material(
        &topo,
        result,
        &[
            // In the wider cutter (1.35 from its axis) but outside the old
            // tunnel (r 1.2), before its end at x = 6: carved now.
            ((2.0, 6.0, 3.35), false),
            // The same ring past the cutter's end: still material.
            ((8.0, 6.0, 3.35), true),
            // Either side of the end cap, beside the bore (y = 7.3).
            ((5.9, 7.3, 2.0), false),
            ((6.1, 7.3, 2.0), true),
            // The bore and the old tunnel are still void, the plate solid.
            ((5.0, 4.0, 1.0), false),
            ((8.5, 6.0, 2.0), false),
            ((1.0, 1.0, 2.0), true),
        ],
        "coaxial wider half tunnel",
    );
}

/// Before the fix: 7 edges not used twice (by id and by position): the exit
/// loop used once on the bore wall, the old loop used three times, and the
/// rim arc plus two marched sections at the x = 10 exit used once each.
/// Volume 294.378. Oracle 307.568.
#[test]
fn oblique_tunnel_crossing() {
    let mut topo = Topology::new();
    let third = oblique_cutter(&mut topo);
    let result = chain(&mut topo, third);
    assert_review_measurements(&topo, result, 307.564_244, "oblique tunnel crossing");
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
    // The cutter's axis is (5, 5, 2.3) + s·(cos 30°, sin 30°, 0).
    let axis = |s: f64, dz: f64| {
        (
            5.0 + s * FRAC_PI_6.cos(),
            5.0 + s * FRAC_PI_6.sin(),
            2.3 + dz,
        )
    };
    assert_material(
        &topo,
        result,
        &[
            // On the axis before the bore, and 1.0 above it (r 0.8 cutter).
            (axis(-3.0, 0.0), false),
            (axis(-3.0, 1.0), true),
            // On the axis past the old tunnel (0.4 outside its wall there),
            // and 1.0 above it.
            (axis(4.5, 0.0), false),
            (axis(4.5, 1.0), true),
            // The bore and the old tunnel are still void, the plate solid.
            ((5.0, 4.0, 1.0), false),
            ((1.0, 6.0, 2.0), false),
            ((1.0, 1.0, 2.0), true),
        ],
        "oblique tunnel crossing",
    );
}
