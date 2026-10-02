//! B79 — a closed section loop that crosses a planar face's existing hole.
//!
//! Ready-repro for the all-planar family of the 2026-10-02 gauntlet census
//! (`docs/kernel-maturity/gauntlet-approx-census-2026-10.md`, candidate 1:
//! `mambo-basic-b35`, `mambo-basic-b36`, `mambo-basic-b37`). Every one of
//! those MAMBO models is a plate whose top face carries the footprint of a
//! cross of gusset ribs as an inner wire; the gauntlet's centred probe box
//! cuts a square through the plate that crosses every rib arm and touches no
//! outer edge of the plate. All three operands import closed (0 free / 0
//! over-used edges) and all-planar, yet the probe cut leaves the exact path:
//! b35 and b37 come back from the GFA with 8 / 4 and 16 / 8 free /
//! non-manifold edges and are rejected at the gate, b36 errors in assembly
//! ("open growth shell with 26 faces would be dropped"), and all three are
//! answered by the disclosed mesh fallback.
//!
//! The same failure reproduces from boxes alone. A 10×1×10 plate fused with
//! an 8×1×1 bar rib (an exact fuse: the plate top keeps the bar's footprint as
//! an inner wire) and cut by a 4×4×4 box through its middle:
//!
//! * `plate_internal_line_loops` (`algo/src/builder/face_splitter/mod.rs`)
//!   accepts the tool's square footprint on the plate top as a closed loop
//!   strictly inside the face. It tests the loop against the OUTER boundary
//!   polygon only, never against the face's original inner wires.
//! * `split_face_with_internal_loops` then carves the square as an
//!   independent hole and keeps the bar footprint as a second hole, split at
//!   the four crossing points but still running through the square. The two
//!   holes overlap, so the bar-hole segments inside the square are free and
//!   the square's edges across the bar mouth are used three times
//!   (raw GFA: 20 faces, 2 free, 2 non-manifold; b35: 26 faces, 8 / 4).
//! * Cut, reversed cut, intersect and fuse of the same pair all refuse under
//!   `ExactOnly`, as do a blind tool, a pocket (cut-made hole), and the
//!   cross-rib plate that is the census shape.
//!
//! The locus was confirmed by experiment and the experiment reverted (no
//! kernel change ships with this file): an env-gated scratch guard that sent
//! such loops to the generic wire-builder route whenever the loop's UV box
//! overlapped an inner wire's made every configuration below exact, with the
//! closed-form volumes, and flipped the three gauntlet cells to exact with
//! 0 free / 0 over-used edges. A bounding-box overlap is NOT the fix (it would
//! also reroute disjoint nested footprints); the fix needs an exact
//! loop-versus-hole crossing test, then the full splitter foil set.
//!
//! Controls that already pass on `main` (not ignored): the same square
//! running out past the plate edge (the section reaches the outer wire, so the
//! generic route takes it) and a tool that lies inside the bar footprint
//! (no section on the plate top at all).
//!
//! Oracles: `ExactOnly` quality, strict validation on both validators, an
//! all-plane census, edge-use counts, the exact Gauss volume against a
//! closed form built from the box dimensions (inclusion–exclusion), watertight
//! and manifold meshes, and ray-cast material probes. The cut configurations
//! also run at scale 1e3 and under a rigid off-origin placement.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::mass_properties;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

/// Plate `[-5, 5] × [0, 1] × [-5, 5]`.
const PLATE: ([f64; 3], [f64; 3]) = ([-5.0, 0.0, -5.0], [5.0, 1.0, 5.0]);
/// Bar rib on the plate top, footprint strictly inside the plate top.
const BAR: ([f64; 3], [f64; 3]) = ([-4.0, 1.0, -0.5], [4.0, 2.0, 0.5]);
/// Second bar of the cross rib (the census shape).
const CROSS_BAR: ([f64; 3], [f64; 3]) = ([-0.5, 1.0, -4.0], [0.5, 2.0, 4.0]);
/// Pocket slot cut down into the plate top (a cut-made inner wire).
const POCKET: ([f64; 3], [f64; 3]) = ([-4.0, 0.5, -0.5], [4.0, 1.5, 0.5]);
/// Through tool: its footprint on the plate top is a closed square strictly
/// inside the plate that crosses the bar footprint four times.
const THROUGH: ([f64; 3], [f64; 3]) = ([-2.0, -1.0, -2.0], [2.0, 3.0, 2.0]);
/// Blind tool: top inside the rib, bottom through the plate.
const BLIND: ([f64; 3], [f64; 3]) = ([-2.0, -1.0, -2.0], [2.0, 1.5, 2.0]);
/// Pocket tool: bottom inside the plate, top above the rib.
const POCKETING: ([f64; 3], [f64; 3]) = ([-2.0, 0.5, -2.0], [2.0, 3.0, 2.0]);
/// Control: the through tool run out past the plate's x = 5 edge.
const PAST_EDGE: ([f64; 3], [f64; 3]) = ([-2.0, -1.0, -2.0], [6.0, 3.0, 2.0]);
/// Control: a tool inside the bar footprint (no section on the plate top).
const IN_FOOTPRINT: ([f64; 3], [f64; 3]) = ([-2.0, -1.0, -0.25], [2.0, 3.0, 0.25]);

/// Rigid placements the cut acceptance runs under: (scale, translation).
///
/// Not (37, −91, 13): there the plate ∪ bar fuse and the plate − pocket cut
/// that BUILD the operands already refuse exact-only on `main` (a y-offset of
/// −91 alone reproduces it; −10, +100 and every other axis are exact). That
/// placement-variance defect is upstream of B79 and is recorded on the B79
/// row as an adjacent finding, so it cannot fail this repro.
const PLACEMENTS: [(f64, [f64; 3]); 3] = [
    (1.0, [0.0, 0.0, 0.0]),
    (1.0, [10.0, -10.0, 10.0]),
    (1e3, [0.0, 0.0, 0.0]),
];

fn vol(b: ([f64; 3], [f64; 3])) -> f64 {
    (b.1[0] - b.0[0]) * (b.1[1] - b.0[1]) * (b.1[2] - b.0[2])
}

/// Volume of the intersection of two axis-aligned boxes.
fn overlap(a: ([f64; 3], [f64; 3]), b: ([f64; 3], [f64; 3])) -> f64 {
    (0..3)
        .map(|k| (a.1[k].min(b.1[k]) - a.0[k].max(b.0[k])).max(0.0))
        .product()
}

/// Volume of the intersection of three axis-aligned boxes.
fn overlap3(a: ([f64; 3], [f64; 3]), b: ([f64; 3], [f64; 3]), c: ([f64; 3], [f64; 3])) -> f64 {
    (0..3)
        .map(|k| (a.1[k].min(b.1[k]).min(c.1[k]) - a.0[k].max(b.0[k]).max(c.0[k])).max(0.0))
        .product()
}

struct Frame {
    scale: f64,
    offset: [f64; 3],
}

impl Frame {
    fn point(&self, p: [f64; 3]) -> Point3 {
        Point3::new(
            p[0].mul_add(self.scale, self.offset[0]),
            p[1].mul_add(self.scale, self.offset[1]),
            p[2].mul_add(self.scale, self.offset[2]),
        )
    }

    fn make(&self, topo: &mut Topology, b: ([f64; 3], [f64; 3])) -> SolidId {
        let s = self.scale;
        let solid = make_box(
            topo,
            (b.1[0] - b.0[0]) * s,
            (b.1[1] - b.0[1]) * s,
            (b.1[2] - b.0[2]) * s,
        )
        .expect("box");
        let o = self.point(b.0);
        transform_solid(topo, solid, &Mat4::translation(o.x(), o.y(), o.z())).expect("place box");
        solid
    }
}

fn exact(topo: &mut Topology, op: BooleanOp, a: SolidId, b: SolidId, what: &str) -> SolidId {
    let outcome = boolean_with_context(
        topo,
        op,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap_or_else(|e| panic!("{what}: exact-only {op:?} refused: {e}"));
    assert!(
        matches!(outcome.quality, BooleanQuality::Exact),
        "{what}: non-exact quality {:?}",
        outcome.quality
    );
    outcome.solid
}

/// Plate with the bar rib fused on top. Exact on `main`: the bar footprint
/// lands on the unholed plate top as an internal loop.
fn plate_with_bar(topo: &mut Topology, f: &Frame) -> SolidId {
    let plate = f.make(topo, PLATE);
    let bar = f.make(topo, BAR);
    exact(topo, BooleanOp::Fuse, plate, bar, "plate ∪ bar")
}

/// Plate with the cross rib fused on top. The two bars are fused to each
/// other first so the plate top receives ONE internal loop (no prior hole).
fn plate_with_cross(topo: &mut Topology, f: &Frame) -> SolidId {
    let plate = f.make(topo, PLATE);
    let bar = f.make(topo, BAR);
    let cross_bar = f.make(topo, CROSS_BAR);
    let cross = exact(topo, BooleanOp::Fuse, bar, cross_bar, "bar ∪ cross bar");
    exact(topo, BooleanOp::Fuse, plate, cross, "plate ∪ cross")
}

/// Plate with the slot pocket cut into its top.
fn plate_with_pocket(topo: &mut Topology, f: &Frame) -> SolidId {
    let plate = f.make(topo, PLATE);
    let pocket = f.make(topo, POCKET);
    exact(topo, BooleanOp::Cut, plate, pocket, "plate − pocket")
}

fn edge_uses(topo: &Topology, s: SolidId) -> (usize, usize) {
    let mut uses: HashMap<EdgeId, usize> = HashMap::new();
    for fid in solid_faces(topo, s).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                *uses.entry(oe.edge()).or_default() += 1;
            }
        }
    }
    (
        uses.values().filter(|&&n| n == 1).count(),
        uses.values().filter(|&&n| n > 2).count(),
    )
}

fn assert_strict_valid(topo: &Topology, s: SolidId, what: &str) {
    let strict = remus_operations::validate::validate_solid(topo, s)
        .unwrap_or_else(|e| panic!("{what}: validator error: {e:?}"));
    assert!(
        strict.is_valid(),
        "{what}: ops validator issues {:?}",
        strict.issues
    );
    let mut opts = remus_check::validate::ValidateOptions::default();
    opts.disabled_checks
        .insert(remus_check::validate::CheckId::ShellConnected);
    let rep = remus_check::validate::validate_solid(topo, s, &opts).unwrap();
    let errors: Vec<_> = rep
        .issues
        .iter()
        .filter(|i| i.severity == remus_check::validate::Severity::Error)
        .collect();
    assert!(errors.is_empty(), "{what}: check-crate errors {errors:?}");
}

/// The full acceptance bar for one exact result.
fn assert_result(
    topo: &Topology,
    s: SolidId,
    f: &Frame,
    expected_unit_volume: f64,
    inside: &[[f64; 3]],
    outside: &[[f64; 3]],
    what: &str,
) {
    assert_strict_valid(topo, s, what);

    let faces = solid_faces(topo, s).unwrap();
    for &fid in &faces {
        assert!(
            matches!(topo.face(fid).unwrap().surface(), FaceSurface::Plane { .. }),
            "{what}: face {fid:?} is not a plane"
        );
    }
    let (free, over) = edge_uses(topo, s);
    assert_eq!((free, over), (0, 0), "{what}: free / over-used edges");

    let expected = expected_unit_volume * f.scale.powi(3);
    let gauss = mass_properties(topo, s).unwrap().mass;
    assert!(
        (gauss - expected).abs() <= 1e-9 * expected,
        "{what}: Gauss volume {gauss:.12} vs closed form {expected:.12}"
    );

    for d in [0.1, 0.01] {
        let mesh = tessellate_solid(topo, s, d * f.scale).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "{what}: open mesh at d={d}");
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "{what}: non-manifold mesh at d={d}"
        );
    }

    let opts = ClassifyOptions::default();
    for &p in inside {
        let c = classify_point(topo, s, f.point(p), &opts).unwrap();
        assert_eq!(
            c,
            PointClassification::Inside,
            "{what}: probe {p:?} must be material"
        );
    }
    for &p in outside {
        let c = classify_point(topo, s, f.point(p), &opts).unwrap();
        assert_eq!(
            c,
            PointClassification::Outside,
            "{what}: probe {p:?} must be empty"
        );
    }
}

/// Material that must survive every cut below: the plate outside the tool's
/// square, and both bar stubs beyond it.
const KEPT: [[f64; 3]; 4] = [
    [4.0, 0.5, 4.0],
    [-4.0, 0.25, 3.0],
    [3.0, 1.5, 0.0],
    [-3.0, 1.5, 0.0],
];
/// Air beside the bar and above everything.
const AIR: [[f64; 3]; 2] = [[3.0, 1.5, 3.0], [0.0, 2.5, 0.0]];

fn with_air(extra: &[[f64; 3]]) -> Vec<[f64; 3]> {
    AIR.iter().chain(extra).copied().collect()
}

#[test]
#[ignore = "open: B79 — a closed section loop crossing a planar face's existing hole is carved as an independent hole (plane_internal_line_loops ignores original inner wires); exact cut refuses"]
fn b79_plate_bar_through_cut_is_exact() {
    for (scale, offset) in PLACEMENTS {
        let f = Frame { scale, offset };
        let what = format!("plate∪bar − through box (scale {scale}, offset {offset:?})");
        let mut topo = Topology::new();
        let a = plate_with_bar(&mut topo, &f);
        let tool = f.make(&mut topo, THROUGH);
        let r = exact(&mut topo, BooleanOp::Cut, a, tool, &what);
        let expected = vol(PLATE) + vol(BAR) - overlap(PLATE, THROUGH) - overlap(BAR, THROUGH);
        assert_result(
            &topo,
            r,
            &f,
            expected,
            &KEPT,
            &with_air(&[[0.0, 0.5, 0.0], [0.0, 1.5, 0.0], [1.0, 0.5, 1.5]]),
            &what,
        );
    }
}

#[test]
#[ignore = "open: B79 — a closed section loop crossing a planar face's existing hole is carved as an independent hole (plane_internal_line_loops ignores original inner wires); exact cut refuses"]
fn b79_plate_cross_through_cut_is_exact() {
    // The census shape: the square crosses all four arms of the cross.
    for (scale, offset) in PLACEMENTS {
        let f = Frame { scale, offset };
        let what = format!("plate∪cross − through box (scale {scale}, offset {offset:?})");
        let mut topo = Topology::new();
        let a = plate_with_cross(&mut topo, &f);
        let tool = f.make(&mut topo, THROUGH);
        let r = exact(&mut topo, BooleanOp::Cut, a, tool, &what);
        let operand = vol(PLATE) + vol(BAR) + vol(CROSS_BAR) - overlap(BAR, CROSS_BAR);
        let removed = overlap(PLATE, THROUGH) + overlap(BAR, THROUGH) + overlap(CROSS_BAR, THROUGH)
            - overlap3(BAR, CROSS_BAR, THROUGH);
        assert_result(
            &topo,
            r,
            &f,
            operand - removed,
            &[KEPT[0], KEPT[2], KEPT[3], [0.0, 1.5, 3.0], [0.0, 1.5, -3.0]],
            &with_air(&[[0.0, 0.5, 0.0], [0.0, 1.5, 0.0], [1.0, 1.5, 1.0]]),
            &what,
        );
    }
}

#[test]
#[ignore = "open: B79 — a closed section loop crossing a planar face's existing hole is carved as an independent hole (plane_internal_line_loops ignores original inner wires); exact cut refuses"]
fn b79_plate_pocket_through_cut_is_exact() {
    // The hole on the plate top comes from a cut (a slot pocket), not a fuse.
    for (scale, offset) in PLACEMENTS {
        let f = Frame { scale, offset };
        let what = format!("plate−pocket − through box (scale {scale}, offset {offset:?})");
        let mut topo = Topology::new();
        let a = plate_with_pocket(&mut topo, &f);
        let tool = f.make(&mut topo, THROUGH);
        let r = exact(&mut topo, BooleanOp::Cut, a, tool, &what);
        let operand = vol(PLATE) - overlap(PLATE, POCKET);
        let removed = overlap(PLATE, THROUGH) - overlap3(PLATE, POCKET, THROUGH);
        assert_result(
            &topo,
            r,
            &f,
            operand - removed,
            &[KEPT[0], KEPT[1], [3.0, 0.25, 0.0], [-3.0, 0.25, 0.0]],
            &[
                [3.0, 0.75, 0.0],
                [-3.0, 0.75, 0.0],
                [0.0, 0.5, 0.0],
                [1.0, 0.25, 1.5],
            ],
            &what,
        );
    }
}

#[test]
#[ignore = "open: B79 — a closed section loop crossing a planar face's existing hole is carved as an independent hole (plane_internal_line_loops ignores original inner wires); blind and pocketing cuts refuse"]
fn b79_plate_bar_blind_and_pocketing_cuts_are_exact() {
    let f = Frame {
        scale: 1.0,
        offset: [0.0, 0.0, 0.0],
    };
    // Blind: the tool's top stops inside the rib, so a rib skin stays over
    // the square.
    {
        let mut topo = Topology::new();
        let a = plate_with_bar(&mut topo, &f);
        let tool = f.make(&mut topo, BLIND);
        let r = exact(&mut topo, BooleanOp::Cut, a, tool, "blind cut");
        let expected = vol(PLATE) + vol(BAR) - overlap(PLATE, BLIND) - overlap(BAR, BLIND);
        assert_result(
            &topo,
            r,
            &f,
            expected,
            &[KEPT[0], KEPT[2], KEPT[3], [0.0, 1.75, 0.0]],
            &with_air(&[[0.0, 0.5, 0.0], [0.0, 1.25, 0.0]]),
            "blind cut",
        );
    }
    // Pocketing: the tool's bottom stops inside the plate, so a floor stays.
    {
        let mut topo = Topology::new();
        let a = plate_with_bar(&mut topo, &f);
        let tool = f.make(&mut topo, POCKETING);
        let r = exact(&mut topo, BooleanOp::Cut, a, tool, "pocketing cut");
        let expected = vol(PLATE) + vol(BAR) - overlap(PLATE, POCKETING) - overlap(BAR, POCKETING);
        assert_result(
            &topo,
            r,
            &f,
            expected,
            &[KEPT[0], KEPT[2], KEPT[3], [0.0, 0.25, 0.0]],
            &with_air(&[[0.0, 0.75, 0.0], [0.0, 1.5, 0.0]]),
            "pocketing cut",
        );
    }
}

#[test]
#[ignore = "open: B79 — a closed section loop crossing a planar face's existing hole is carved as an independent hole (plane_internal_line_loops ignores original inner wires); every operation on the pair refuses"]
fn b79_plate_bar_and_box_all_operations_are_exact() {
    let f = Frame {
        scale: 1.0,
        offset: [0.0, 0.0, 0.0],
    };
    let common = overlap(PLATE, THROUGH) + overlap(BAR, THROUGH);
    let a_vol = vol(PLATE) + vol(BAR);
    // Reversed cut: the box minus the plate and rib inside it.
    {
        let mut topo = Topology::new();
        let a = plate_with_bar(&mut topo, &f);
        let tool = f.make(&mut topo, THROUGH);
        let r = exact(&mut topo, BooleanOp::Cut, tool, a, "box − plate∪bar");
        assert_result(
            &topo,
            r,
            &f,
            vol(THROUGH) - common,
            &[[0.0, -0.5, 0.0], [0.0, 2.5, 0.0], [1.0, 1.5, 1.5]],
            &[[0.0, 0.5, 0.0], [0.0, 1.5, 0.0], [4.0, 0.5, 4.0]],
            "reversed cut",
        );
    }
    {
        let mut topo = Topology::new();
        let a = plate_with_bar(&mut topo, &f);
        let tool = f.make(&mut topo, THROUGH);
        let r = exact(&mut topo, BooleanOp::Intersect, a, tool, "plate∪bar ∩ box");
        assert_result(
            &topo,
            r,
            &f,
            common,
            &[[0.0, 0.5, 0.0], [0.0, 1.5, 0.0], [1.5, 0.5, 1.5]],
            &[[1.0, 1.5, 1.5], [3.0, 1.5, 0.0], [0.0, -0.5, 0.0]],
            "intersect",
        );
    }
    {
        let mut topo = Topology::new();
        let a = plate_with_bar(&mut topo, &f);
        let tool = f.make(&mut topo, THROUGH);
        let r = exact(&mut topo, BooleanOp::Fuse, a, tool, "plate∪bar ∪ box");
        assert_result(
            &topo,
            r,
            &f,
            a_vol + vol(THROUGH) - common,
            &[[0.0, -0.5, 0.0], [0.0, 2.5, 0.0], KEPT[0], KEPT[2]],
            &[[3.0, 1.5, 3.0], [0.0, 3.5, 0.0], [3.0, -0.5, 3.0]],
            "fuse",
        );
    }
}

/// Controls on `main`: the same pair takes the exact path when the square
/// reaches the plate's outer edge or misses the plate top entirely. A fix for
/// B79 must keep both.
#[test]
fn b79_controls_outer_reaching_and_in_footprint_cuts_stay_exact() {
    let f = Frame {
        scale: 1.0,
        offset: [0.0, 0.0, 0.0],
    };
    {
        let mut topo = Topology::new();
        let a = plate_with_bar(&mut topo, &f);
        let tool = f.make(&mut topo, PAST_EDGE);
        let r = exact(&mut topo, BooleanOp::Cut, a, tool, "past-edge cut");
        let expected = vol(PLATE) + vol(BAR) - overlap(PLATE, PAST_EDGE) - overlap(BAR, PAST_EDGE);
        assert_result(
            &topo,
            r,
            &f,
            expected,
            &[[-4.0, 0.5, 4.0], [-3.0, 1.5, 0.0], [3.0, 0.5, 4.0]],
            &[
                [3.0, 0.5, 0.0],
                [3.0, 1.5, 0.0],
                [0.0, 1.5, 0.0],
                [-3.0, 1.5, 3.0],
            ],
            "past-edge cut",
        );
    }
    {
        let mut topo = Topology::new();
        let a = plate_with_bar(&mut topo, &f);
        let tool = f.make(&mut topo, IN_FOOTPRINT);
        let r = exact(&mut topo, BooleanOp::Cut, a, tool, "in-footprint cut");
        let expected =
            vol(PLATE) + vol(BAR) - overlap(PLATE, IN_FOOTPRINT) - overlap(BAR, IN_FOOTPRINT);
        assert_result(
            &topo,
            r,
            &f,
            expected,
            &[KEPT[0], [0.0, 1.5, 0.4], [0.0, 0.5, 0.4], KEPT[2]],
            &[[0.0, 0.5, 0.0], [0.0, 1.5, 0.0], [3.0, 1.5, 3.0]],
            "in-footprint cut",
        );
    }
}
