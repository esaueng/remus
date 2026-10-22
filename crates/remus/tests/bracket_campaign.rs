//! Filleted mechanical bracket campaign: one shared target for fillet work.
//!
//! The part (mm, z up): a 74 × 60 × 4 base plate with a U-slot cut into its
//! back edge, two 18 × 10 × 40 tabs fused onto that edge, a groove across the
//! tab backs, a counterbored hole, two countersunk holes, a cross hole through
//! each tab, and seven fillet passes F1–F7 picked by edge-endpoint predicates.
//!
//! Every step that passes on `main` is asserted with an oracle: exact boolean
//! quality and no NURBS faces, `validate` valid, a watertight welded mesh, and
//! a volume change against a closed form. Every step that fails is an
//! `#[ignore]`d ready-repro asserting the DESIRED result, so a fix shows up as
//! an ignored test that starts passing:
//!
//! ```text
//! cargo test --profile ci-test -p remus --test bracket_campaign -- --ignored
//! ```
//!
//! `current_refusals_are_typed_and_atomic` pins today's refusal codes. When a
//! fix lands, drop its row there and un-ignore its ready-repro in the same PR.
//!
//! Plate profile: the plate is extruded from the slotted profile directly.
//! Fusing the tabs onto a plain plate and then cutting the slot falls back to
//! a mesh on `main` (fix: esaueng/remus#1237), so that order is not used here.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, FRAC_PI_6, PI};

use remus::prelude::*;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;

/// Endpoint-coordinate match tolerance for edge selection (mm).
const SELECT_TOL: f64 = 1e-6;
/// Volume match for planar and cylindrical results (mm³).
const PLANAR_VOLUME_TOL: f64 = 1e-6;
/// Volume match once sphere or torus blend faces enter the integral (mm³).
/// The F5 corner spheres integrate ~0.01 mm³ below their closed form.
const CURVED_VOLUME_TOL: f64 = 0.05;
const MEASURE_DEFLECTION: f64 = 0.01;
const MESH_DEFLECTION: f64 = 0.1;

const PLATE_T: f64 = 4.0;
const TAB_TOP: f64 = 40.0;
const PLATE_VOLUME: f64 = 74.0 * 60.0 * PLATE_T - 30.0 * 8.0 * PLATE_T;
/// Each tab adds 18 × 10 above the plate top.
const TABS_VOLUME: f64 = PLATE_VOLUME + 2.0 * 18.0 * 10.0 * (TAB_TOP - PLATE_T);

/// Cross-section the fillet of a right-angled straight edge removes (convex)
/// or adds (concave): the square minus the quarter disc.
fn spandrel(r: f64) -> f64 {
    r * r * (1.0 - FRAC_PI_4)
}

// ── Construction ─────────────────────────────────────────────────────────

fn placed_box(m: &mut Model, lo: [f64; 3], hi: [f64; 3]) -> SolidId {
    let s = m
        .make_box(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])
        .unwrap();
    m.transform(s, &Mat4::translation(lo[0], lo[1], lo[2]))
        .unwrap();
    s
}

fn placed(m: &mut Model, tool: SolidId, matrix: &Mat4) -> SolidId {
    m.transform(tool, matrix).unwrap();
    tool
}

/// The slotted base plate, extruded from its exact profile.
fn plate(m: &mut Model) -> SolidId {
    let profile: Vec<Point3> = [
        (0.0, 0.0),
        (22.0, 0.0),
        (22.0, 8.0),
        (52.0, 8.0),
        (52.0, 0.0),
        (74.0, 0.0),
        (74.0, 60.0),
        (0.0, 60.0),
    ]
    .into_iter()
    .map(|(x, y)| Point3::new(x, y, 0.0))
    .collect();
    let face = m.make_planar_face(&profile).unwrap();
    m.extrude(face, Vec3::new(0.0, 0.0, 1.0), PLATE_T).unwrap()
}

/// Plate plus both tabs (the unfilleted, uncut body every pass starts from).
fn tabbed(m: &mut Model) -> SolidId {
    let mut body = plate(m);
    for x0 in [0.0, 56.0] {
        let tab = placed_box(m, [x0, 0.0, 0.0], [x0 + 18.0, 10.0, TAB_TOP]);
        body = exact_boolean(m, BooleanOp::Fuse, body, tab, "tab fuse");
    }
    body
}

/// One cut feature: its tools and the closed-form volume it removes.
struct Feature {
    name: &'static str,
    tools: fn(&mut Model) -> Vec<SolidId>,
    removed: f64,
}

fn groove_tools(m: &mut Model) -> Vec<SolidId> {
    vec![placed_box(m, [-1.0, 8.5, 26.0], [75.0, 11.0, 29.0])]
}

fn counterbored_hole_tools(m: &mut Model) -> Vec<SolidId> {
    let bore = m.make_cylinder(3.5, 6.0).unwrap();
    let bore = placed(m, bore, &Mat4::translation(20.0, 40.0, -1.0));
    let counterbore = m.make_cylinder(6.0, 2.5).unwrap();
    let counterbore = placed(m, counterbore, &Mat4::translation(20.0, 40.0, 2.5));
    vec![bore, counterbore]
}

fn countersunk_hole_tools(m: &mut Model) -> Vec<SolidId> {
    let mut tools = Vec::new();
    for (x, y) in [(32.0, 18.0), (45.0, 13.0)] {
        let hole = m.make_cylinder(1.6, 6.0).unwrap();
        tools.push(placed(m, hole, &Mat4::translation(x, y, -1.0)));
        let sink = m.make_cone(1.6, 3.6, 2.0).unwrap();
        tools.push(placed(m, sink, &Mat4::translation(x, y, 2.4)));
    }
    tools
}

fn cross_hole_tools(m: &mut Model) -> Vec<SolidId> {
    [9.0, 65.0]
        .into_iter()
        .map(|x| {
            let pin = m.make_cylinder(1.5, 12.0).unwrap();
            // rotation_x(-π/2) takes the cylinder's +z axis to +y.
            placed(
                m,
                pin,
                &(Mat4::translation(x, -1.0, 35.0) * Mat4::rotation_x(-FRAC_PI_2)),
            )
        })
        .collect()
}

fn features() -> [Feature; 4] {
    // Countersink: the cone grows 1 mm in radius per mm of height, so inside
    // the plate (z 2.4..4) it is a frustum from r 1.6 to r 3.2, less the
    // r 1.6 hole it overlaps.
    let sink_h: f64 = PLATE_T - 2.4;
    let sink = PI * sink_h / 3.0 * (3.2_f64.powi(2) + 3.2 * 1.6 + 1.6_f64.powi(2))
        - PI * 1.6_f64.powi(2) * sink_h;
    [
        Feature {
            name: "groove",
            tools: groove_tools,
            removed: 2.0 * 18.0 * (10.0 - 8.5) * 3.0,
        },
        Feature {
            name: "counterbored hole",
            tools: counterbored_hole_tools,
            removed: PI * 3.5_f64.powi(2) * PLATE_T
                + PI * (6.0_f64.powi(2) - 3.5_f64.powi(2)) * (PLATE_T - 2.5),
        },
        Feature {
            name: "countersunk holes",
            tools: countersunk_hole_tools,
            removed: 2.0 * (PI * 1.6_f64.powi(2) * PLATE_T + sink),
        },
        Feature {
            name: "cross holes",
            tools: cross_hole_tools,
            removed: 2.0 * PI * 1.5_f64.powi(2) * 10.0,
        },
    ]
}

fn exact_boolean(m: &mut Model, op: BooleanOp, a: SolidId, b: SolidId, label: &str) -> SolidId {
    let outcome = m.boolean(op, a, b).unwrap();
    assert_eq!(
        outcome.quality,
        BooleanQuality::Exact,
        "{label}: boolean fell back to a mesh"
    );
    outcome.solid
}

/// Cuts every feature, asserting each cut's soundness and closed-form volume.
fn cut_features(m: &mut Model, mut body: SolidId, only: &[&str]) -> SolidId {
    for feature in features() {
        if !only.is_empty() && !only.contains(&feature.name) {
            continue;
        }
        let before = volume(m, body);
        for tool in (feature.tools)(m) {
            body = exact_boolean(m, BooleanOp::Cut, body, tool, feature.name);
        }
        let after = assert_sound(m, body, feature.name);
        assert_volume(
            before - after,
            feature.removed,
            PLANAR_VOLUME_TOL,
            feature.name,
        );
    }
    body
}

// ── Fillet passes ────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pass {
    /// Base front vertical corners, r6.
    F1,
    /// U-slot inner (concave) vertical corners, r2.
    F2,
    /// Tab outer vertical edges, r2.
    F3,
    /// Concave tab/base edges, r3.
    F4,
    /// Tab top loops, r2.
    F5,
    /// Base top perimeter on the front and sides, r1.
    F6,
    /// Groove lip edges, r0.5.
    F7,
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < SELECT_TOL
}

fn near_any(a: f64, values: &[f64]) -> bool {
    values.iter().any(|&v| near(a, v))
}

impl Pass {
    const fn radius(self) -> f64 {
        match self {
            Self::F1 => 6.0,
            Self::F2 | Self::F3 | Self::F5 => 2.0,
            Self::F4 => 3.0,
            Self::F6 => 1.0,
            Self::F7 => 0.5,
        }
    }

    fn selects(self, a: Point3, b: Point3) -> bool {
        let vertical = near(a.x(), b.x()) && near(a.y(), b.y());
        match self {
            Self::F1 => vertical && near(a.y(), 60.0) && near_any(a.x(), &[0.0, 74.0]),
            Self::F2 => vertical && near(a.y(), 8.0) && near_any(a.x(), &[22.0, 52.0]),
            Self::F3 => {
                vertical
                    && near_any(a.x(), &[18.0, 56.0])
                    && near_any(a.y(), &[0.0, 10.0])
                    && a.z().min(b.z()) > PLATE_T - SELECT_TOL
            }
            Self::F4 => {
                near(a.z(), PLATE_T)
                    && near(b.z(), PLATE_T)
                    && near(a.y(), 10.0)
                    && near(b.y(), 10.0)
            }
            Self::F5 => near(a.z(), TAB_TOP) && near(b.z(), TAB_TOP),
            Self::F6 => {
                // Open edges on the outer band of the top face; the band test
                // keeps the counterbore and countersink rims out.
                let on_band = |p: Point3| p.x() < 6.5 || p.x() > 67.5 || p.y() > 53.5;
                near(a.z(), PLATE_T)
                    && near(b.z(), PLATE_T)
                    && a.y() > 13.0
                    && b.y() > 13.0
                    && (a - b).length() > SELECT_TOL
                    && on_band(a)
                    && on_band(b)
            }
            Self::F7 => {
                near(a.y(), 10.0)
                    && near(b.y(), 10.0)
                    && ((near(a.z(), 26.0) && near(b.z(), 26.0))
                        || (near(a.z(), 29.0) && near(b.z(), 29.0)))
            }
        }
    }
}

fn select(m: &Model, body: SolidId, pass: Pass) -> Vec<EdgeId> {
    let topo = m.topology();
    m.solid_edges(body)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            pass.selects(a, b)
        })
        .collect()
}

fn try_fillet(
    m: &mut Model,
    body: SolidId,
    pass: Pass,
    expected_edges: usize,
) -> Result<BlendResult, OperationsError> {
    let edges = select(m, body, pass);
    assert_eq!(
        edges.len(),
        expected_edges,
        "{pass:?}: edge selection drifted"
    );
    m.fillet(body, &edges, pass.radius())
}

/// Runs a pass that must succeed and checks it removed `removed` mm³
/// (negative when a concave fillet adds material).
fn fillet(
    m: &mut Model,
    body: SolidId,
    pass: Pass,
    expected_edges: usize,
    removed: f64,
    tolerance: f64,
) -> SolidId {
    let before = volume(m, body);
    let blend = try_fillet(m, body, pass, expected_edges)
        .unwrap_or_else(|e| panic!("{pass:?}: fillet refused: {e}"));
    assert!(!blend.is_partial, "{pass:?}: partial result");
    assert!(blend.failed.is_empty(), "{pass:?}: {:?}", blend.failed);
    assert_eq!(blend.succeeded.len(), expected_edges, "{pass:?}");
    let label = format!("{pass:?}");
    let after = assert_sound(m, blend.solid, &label);
    assert_volume(before - after, removed, tolerance, &label);
    blend.solid
}

// ── Oracles ──────────────────────────────────────────────────────────────

fn volume(m: &Model, body: SolidId) -> f64 {
    m.volume(body, MEASURE_DEFLECTION).unwrap()
}

fn assert_volume(actual: f64, expected: f64, tolerance: f64, label: &str) {
    assert!(
        (actual - expected).abs() < tolerance,
        "{label}: volume change {actual} mm³, closed form {expected} mm³"
    );
}

/// Valid, exact (analytic faces only), watertight. Returns the volume.
fn assert_sound(m: &Model, body: SolidId, label: &str) -> f64 {
    let report = m.validate(body).unwrap();
    assert!(report.is_valid(), "{label}: invalid: {:?}", report.issues);
    let faces = solid_faces(m.topology(), body).unwrap();
    let nurbs = faces
        .iter()
        .filter(|&&f| {
            matches!(
                m.topology().face(f).unwrap().surface(),
                FaceSurface::Nurbs(_)
            )
        })
        .count();
    assert_eq!(
        nurbs, 0,
        "{label}: {nurbs} NURBS faces, expected exact analytic"
    );
    let quality = welded_mesh_quality(&m.tessellate(body, MESH_DEFLECTION).unwrap());
    assert!(quality.is_watertight(), "{label}: mesh {quality:?}");
    volume(m, body)
}

// ── Construction steps (all pass) ────────────────────────────────────────

#[test]
fn plate_extrudes_exact() {
    let mut m = Model::new();
    let body = plate(&mut m);
    let v = assert_sound(&m, body, "plate");
    assert_volume(v, PLATE_VOLUME, PLANAR_VOLUME_TOL, "plate");
    // 8 side walls + 2 caps: the slot is part of the profile, not a cut.
    assert_eq!(solid_faces(m.topology(), body).unwrap().len(), 10);
}

#[test]
fn tabs_fuse_exact() {
    let mut m = Model::new();
    let body = tabbed(&mut m);
    let v = assert_sound(&m, body, "tabs");
    assert_volume(v, TABS_VOLUME, PLANAR_VOLUME_TOL, "tabs");
}

#[test]
fn feature_cuts_stay_exact() {
    let mut m = Model::new();
    let body = tabbed(&mut m);
    let body = cut_features(&mut m, body, &[]);
    let total: f64 = features().iter().map(|f| f.removed).sum();
    assert_volume(volume(&m, body), TABS_VOLUME - total, 1e-5, "all cuts");
}

// ── Fillet passes alone on the uncut body ────────────────────────────────

#[test]
fn f1_front_corners_alone() {
    let mut m = Model::new();
    let body = tabbed(&mut m);
    fillet(
        &mut m,
        body,
        Pass::F1,
        2,
        2.0 * PLATE_T * spandrel(6.0),
        1e-5,
    );
}

#[test]
fn f2_slot_corners_alone() {
    let mut m = Model::new();
    let body = tabbed(&mut m);
    // Concave: material is added.
    fillet(
        &mut m,
        body,
        Pass::F2,
        2,
        -2.0 * PLATE_T * spandrel(2.0),
        1e-5,
    );
}

#[test]
fn f3_tab_vertical_edges_alone() {
    let mut m = Model::new();
    let body = tabbed(&mut m);
    let removed = 4.0 * (TAB_TOP - PLATE_T) * spandrel(2.0);
    fillet(&mut m, body, Pass::F3, 4, removed, 1e-5);
}

/// Each tab-top corner is the rolling-ball two-edge corner (ball octant plus
/// a planar ledge closing it onto the unfilleted vertical edge), so the
/// corner cube loses r³(1 − π/6) and each edge's straight run is L − 2r.
fn f5_removed() -> f64 {
    let r = Pass::F5.radius();
    let edge_lengths = 2.0 * 2.0 * (18.0 + 10.0);
    (edge_lengths - 16.0 * r) * spandrel(r) + 8.0 * r.powi(3) * (1.0 - FRAC_PI_6)
}

#[test]
fn f5_tab_top_loops_alone() {
    let mut m = Model::new();
    let body = tabbed(&mut m);
    fillet(&mut m, body, Pass::F5, 8, f5_removed(), CURVED_VOLUME_TOL);
}

#[test]
fn f6_front_top_edge_alone() {
    let mut m = Model::new();
    let body = tabbed(&mut m);
    // Uncut and unfilleted, only the 74 mm front edge has both ends at y > 13.
    fillet(&mut m, body, Pass::F6, 1, 74.0 * spandrel(1.0), 1e-5);
}

#[test]
#[ignore = "open: F4 concave tab/base fillet alone fails postcondition: 16 free edges, 1 inconsistent orientation"]
fn f4_concave_tab_base_edges_alone() {
    let mut m = Model::new();
    let body = tabbed(&mut m);
    // Concave, 18 mm per tab, ends flush with the tab sides.
    fillet(&mut m, body, Pass::F4, 2, -2.0 * 18.0 * spandrel(3.0), 1e-3);
}

// ── Fillet passes in sequence ────────────────────────────────────────────

/// F1 then F2, both exact. Every sequence below starts here.
fn f1_f2(m: &mut Model) -> SolidId {
    let body = tabbed(m);
    let body = fillet(m, body, Pass::F1, 2, 2.0 * PLATE_T * spandrel(6.0), 1e-5);
    fillet(m, body, Pass::F2, 2, -2.0 * PLATE_T * spandrel(2.0), 1e-5)
}

/// F1, F2, then F5 (F3 and F4 currently refuse, so they are skipped).
fn f1_f2_f5(m: &mut Model) -> SolidId {
    let body = f1_f2(m);
    fillet(m, body, Pass::F5, 8, f5_removed(), CURVED_VOLUME_TOL)
}

#[test]
fn sequence_f1_f2_f5() {
    let mut m = Model::new();
    f1_f2_f5(&mut m);
}

#[test]
fn feature_cuts_after_f1_f2_f5_stay_exact() {
    let mut m = Model::new();
    let body = f1_f2_f5(&mut m);
    cut_features(&mut m, body, &[]);
}

#[test]
#[ignore = "open: F3 after F1+F2 fails postcondition: 11 free edges, 1 inconsistent orientation (passes alone)"]
fn sequence_f3_after_f1_f2() {
    let mut m = Model::new();
    let body = f1_f2(&mut m);
    let removed = 4.0 * (TAB_TOP - PLATE_T) * spandrel(2.0);
    fillet(&mut m, body, Pass::F3, 4, removed, 1e-3);
}

#[test]
#[ignore = "open: F4 after F1+F2 refuses with blend TrimmingFailure"]
fn sequence_f4_after_f1_f2() {
    let mut m = Model::new();
    let body = f1_f2(&mut m);
    fillet(&mut m, body, Pass::F4, 2, -2.0 * 18.0 * spandrel(3.0), 1e-3);
}

/// F6 after F1 runs the 62 mm straight front edge plus the two F1 quarter
/// arcs at z = 4. Pappus over each arc: the spandrel's centroid sits
/// (10 − 3π)/(3(4 − π))·r from the corner, inside the R 6 arc.
fn f6_after_f1_removed() -> f64 {
    let r = Pass::F6.radius();
    let big_r = Pass::F1.radius();
    let centroid = (10.0 - 3.0 * PI) / (3.0 * (4.0 - PI)) * r;
    spandrel(r) * (74.0 - 2.0 * big_r + 2.0 * FRAC_PI_2 * (big_r - centroid))
}

#[test]
#[ignore = "open: F6 across the F1 tangent seams refuses with blend UnsupportedSeamCrossing"]
fn sequence_f6_after_f1_f2_f5() {
    let mut m = Model::new();
    let body = f1_f2_f5(&mut m);
    fillet(
        &mut m,
        body,
        Pass::F6,
        3,
        f6_after_f1_removed(),
        CURVED_VOLUME_TOL,
    );
}

#[test]
#[ignore = "open: F7 groove lip fillet on the filleted, holed body fails postcondition: 4 inconsistent orientations"]
fn f7_groove_lips_on_filleted_holed_body() {
    let mut m = Model::new();
    let body = f1_f2_f5(&mut m);
    let body = cut_features(&mut m, body, &[]);
    fillet(&mut m, body, Pass::F7, 4, 4.0 * 18.0 * spandrel(0.5), 1e-3);
}

// ── Fillet passes after the cuts ─────────────────────────────────────────

#[test]
fn f1_f2_f6_after_feature_cuts() {
    let mut m = Model::new();
    let body = tabbed(&mut m);
    let body = cut_features(&mut m, body, &[]);
    // F6 branches off the cut body; F1 and F2 then chain on the same input.
    fillet(&mut m, body, Pass::F6, 1, 74.0 * spandrel(1.0), 1e-5);
    let body = fillet(
        &mut m,
        body,
        Pass::F1,
        2,
        2.0 * PLATE_T * spandrel(6.0),
        1e-5,
    );
    fillet(
        &mut m,
        body,
        Pass::F2,
        2,
        -2.0 * PLATE_T * spandrel(2.0),
        1e-5,
    );
}

/// The groove leaves 1.5 mm of tab wall at y 8.5..10, so an r 2 fillet on
/// the tab's back vertical edges has nowhere to land at the groove: the
/// cliff refusal is the correct verdict, not a gap.
#[test]
fn f3_after_groove_refuses_with_cliff() {
    let mut m = Model::new();
    let body = tabbed(&mut m);
    let body = cut_features(&mut m, body, &["groove"]);
    let before = volume(&m, body);
    // The groove splits each y = 10 edge in two: 2 front + 4 back edges.
    let err = try_fillet(&mut m, body, Pass::F3, 6)
        .err()
        .expect("cliff refusal");
    assert!(
        matches!(
            err,
            OperationsError::Blend(BlendError::CliffEncountered { available_radius, .. })
                if (available_radius - 1.5).abs() < 1e-9
        ),
        "{err:?}"
    );
    assert_volume(volume(&m, body), before, 1e-9, "refusal is atomic");
}

#[test]
#[ignore = "open: the tab-back groove alone turns F5's tab-top corners into an unsupported 2-stripe vertex blend"]
fn f5_tab_top_loops_after_groove() {
    let mut m = Model::new();
    let body = tabbed(&mut m);
    let body = cut_features(&mut m, body, &["groove"]);
    fillet(&mut m, body, Pass::F5, 8, f5_removed(), CURVED_VOLUME_TOL);
}

#[test]
#[ignore = "open: any groove lip fillet (even one edge) fails postcondition: 4 inconsistent orientations"]
fn f7_groove_lips_after_groove() {
    let mut m = Model::new();
    let body = tabbed(&mut m);
    let body = cut_features(&mut m, body, &["groove"]);
    fillet(&mut m, body, Pass::F7, 4, 4.0 * 18.0 * spandrel(0.5), 1e-3);
}

// ── Every edge at one radius ─────────────────────────────────────────────

/// Desired: one call fillets every edge of the uncut body. No closed form
/// covers all the corner patches, so the volume change is bounded by
/// ±Σ L·r² (each edge's spandrel is under r² per unit length).
fn fillet_every_edge(radius: f64) {
    let mut m = Model::new();
    let body = tabbed(&mut m);
    let edges = m.solid_edges(body).unwrap();
    let total_length: f64 = edges
        .iter()
        .map(|&e| edge_length(m.topology(), e).unwrap())
        .sum();
    let blend = m
        .fillet(body, &edges, radius)
        .unwrap_or_else(|e| panic!("r {radius}: {e}"));
    assert!(!blend.is_partial && blend.failed.is_empty());
    let after = assert_sound(&m, blend.solid, &format!("all edges r {radius}"));
    let change = TABS_VOLUME - after;
    assert!(
        change.abs() < total_length * radius * radius,
        "r {radius}: volume change {change} mm³ is implausible"
    );
}

#[test]
#[ignore = "open: filleting every edge at r0.5 refuses: unsupported vertex blend, 2 stripes meet"]
fn every_edge_r0_5() {
    fillet_every_edge(0.5);
}

#[test]
#[ignore = "open: filleting every edge at r1 refuses: unsupported vertex blend, 2 stripes meet"]
fn every_edge_r1() {
    fillet_every_edge(1.0);
}

#[test]
#[ignore = "open: filleting every edge at r2 refuses: unsupported vertex blend, 2 stripes meet"]
fn every_edge_r2() {
    fillet_every_edge(2.0);
}

// ── Today's refusals, pinned ─────────────────────────────────────────────

fn postcondition_reason(err: &OperationsError) -> &str {
    match err {
        OperationsError::InvalidInput { reason } => reason,
        other => panic!("expected a fillet postcondition refusal, got {other:?}"),
    }
}

/// Every failing step refuses with a typed error and leaves its input body
/// untouched. When a fix lands, delete its row here and remove the
/// `#[ignore]` on its ready-repro above.
#[test]
fn current_refusals_are_typed_and_atomic() {
    let mut m = Model::new();
    let tabs = tabbed(&mut m);
    let seq = f1_f2(&mut m);
    let seq_f5 = fillet(&mut m, seq, Pass::F5, 8, f5_removed(), CURVED_VOLUME_TOL);
    let grooved = cut_features(&mut m, tabs, &["groove"]);

    let cases: [(&str, SolidId, Pass, usize); 6] = [
        ("F4 alone", tabs, Pass::F4, 2),
        ("F3 after F1+F2", seq, Pass::F3, 4),
        ("F4 after F1+F2", seq, Pass::F4, 2),
        ("F6 after F1+F2+F5", seq_f5, Pass::F6, 3),
        ("F5 after groove", grooved, Pass::F5, 8),
        ("F7 after groove", grooved, Pass::F7, 4),
    ];
    for (label, body, pass, n) in cases {
        let before = volume(&m, body);
        let err = try_fillet(&mut m, body, pass, n).err().expect(label);
        match label {
            "F4 alone" | "F3 after F1+F2" => {
                let reason = postcondition_reason(&err);
                assert!(
                    reason.contains("free (boundary) edges"),
                    "{label}: {reason}"
                );
                assert!(
                    reason.contains("inconsistent face orientations"),
                    "{label}: {reason}"
                );
            }
            "F4 after F1+F2" => assert!(
                matches!(
                    err,
                    OperationsError::Blend(BlendError::TrimmingFailure { .. })
                ),
                "{label}: {err:?}"
            ),
            "F6 after F1+F2+F5" => assert!(
                matches!(
                    err,
                    OperationsError::Blend(BlendError::UnsupportedSeamCrossing { .. })
                ),
                "{label}: {err:?}"
            ),
            "F5 after groove" => assert!(
                matches!(
                    err,
                    OperationsError::Blend(BlendError::UnsupportedVertexBlend { stripes: 2, .. })
                ),
                "{label}: {err:?}"
            ),
            "F7 after groove" => {
                let reason = postcondition_reason(&err);
                assert!(
                    reason.contains("4 shared edges have inconsistent face orientations"),
                    "{label}: {reason}"
                );
            }
            _ => unreachable!(),
        }
        assert!(
            m.validate(body).unwrap().is_valid(),
            "{label}: input damaged"
        );
        assert_volume(volume(&m, body), before, 1e-9, label);
    }

    for radius in [0.5, 1.0, 2.0] {
        let edges = m.solid_edges(tabs).unwrap();
        let err = m.fillet(tabs, &edges, radius).err().expect("every edge");
        assert!(
            matches!(
                err,
                OperationsError::Blend(BlendError::UnsupportedVertexBlend { stripes: 2, .. })
            ),
            "every edge r {radius}: {err:?}"
        );
    }
}
