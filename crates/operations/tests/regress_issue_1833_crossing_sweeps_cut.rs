//! Bridge B283 / issue #1833: `cut` returned `Ok` with an open body and a
//! corrupted hole boundary for two crossing imported sweeps.
//!
//! Both operands are imported swept-arc tubes (`issue_1242_swept_tube_target.step`
//! and `issue_1833_crossing_sweep_tool.step`), each a torus bounded by its two
//! planar end caps and an inner-equator seam circle. Their torus×torus
//! intersection loop crosses the tool's seam circle twice, so the periodic
//! chart splitter splits the tool's interior cavity patch along that seam into
//! a 3-edge face and a 2-edge lens bounded by `[seam circle, section NURBS]`.
//!
//! Previously, `merge_duplicate_edges` phase 2 guarded only `Line × Conic` and
//! `Line × NURBS` wire-collapsing pairs and omitted `Conic × NURBS`, so it
//! merged the NURBS section arc into the tool's seam circle. That collapsed the
//! 2-edge tool face into a spur and replaced the target hole's NURBS edge with
//! the tool's seam circle floating ~1.22 mm off the target torus — producing a
//! topologically 2-manifold 4-face shell whose mesh was open and whose cut
//! retained material inside the tool, while `Intersect` refused with
//! `ExactOnlyUnattainable`.
//!
//! With `Conic × NURBS` lens preservation, `Cut`, `Intersect`, and `Fuse` are
//! all exact analytic B-Reps with watertight meshes, correct material
//! classification, and volumes matching the independent reference.
//!
//! Volumes are read two ways. The enclosed volume of the display mesh,
//! extrapolated to zero deflection from two fine meshes (the chord deficit is
//! linear in the deflection), matches the reference to 5e-3. `solid_volume`'s
//! fixed-rule reading of torus faces trimmed by fitted section curves runs low
//! by up to ~1.6e-4 relative (a measurement limit of those faces, also seen on
//! natively revolved operands), so it is held to a 3e-4 relative band.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::boolean::{
    BooleanOp, BooleanOptions, boolean, boolean_with_entity_evolution, boolean_with_options,
};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::journal_ops::boolean_journaled;
use remus_operations::measure::solid_volume;
use remus_operations::tessellate::tessellate_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const TARGET: &str = include_str!("data/issue_1242_swept_tube_target.step");
const TOOL: &str = include_str!("data/issue_1833_crossing_sweep_tool.step");

/// Strictly inside both operands (>0.92 mm clearance from either boundary).
const WITNESS: [f64; 3] = [5.842_755_04, 1.473_700_56, 5.917_635_25];
/// Inside the target's left leg, well clear of the crossing tool.
const TARGET_ONLY: [f64; 3] = [2.0, 1.0, 4.14];
/// Inside the crossing tool's front leg, well clear of the target.
const TOOL_ONLY: [f64; 3] = [6.0, -4.0, 3.8];
/// Outside both operands.
const OUTSIDE: [f64; 3] = [0.0, 10.0, 10.0];

/// Independent reference volumes (mm^3) from the crossing-sweeps fixture set.
const TARGET_VOLUME: f64 = 219.538_848;
const TOOL_VOLUME: f64 = 269.549_114;
const CUT_VOLUME: f64 = 202.862_135;
const REVERSE_CUT_VOLUME: f64 = TOOL_VOLUME - 16.676_733;
const COMMON_VOLUME: f64 = 16.676_733;
const FUSE_VOLUME: f64 = TARGET_VOLUME + TOOL_VOLUME - COMMON_VOLUME;

fn import(topo: &mut Topology, text: &str) -> SolidId {
    let solids = remus_io::step::reader::read_step(text, topo).unwrap();
    assert_eq!(solids.len(), 1);
    solids[0]
}

fn operands() -> (Topology, SolidId, SolidId) {
    let mut topo = Topology::new();
    let target = import(&mut topo, TARGET);
    let tool = import(&mut topo, TOOL);
    (topo, target, tool)
}

fn point(p: [f64; 3]) -> Point3 {
    Point3::new(p[0], p[1], p[2])
}

fn class(topo: &Topology, solid: SolidId, p: [f64; 3]) -> PointClassification {
    classify_point(topo, solid, point(p), 0.01, 1e-7).unwrap()
}

/// An exact analytic B-Rep: planes and tori only, every edge used by exactly
/// two face uses.
fn assert_exact_brep(topo: &Topology, solid: SolidId, expected_faces: usize, what: &str) {
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    assert_eq!(
        faces.len(),
        expected_faces,
        "{what}: expected {expected_faces} faces, got {}",
        faces.len()
    );
    let mut tori = 0usize;
    let mut uses: BTreeMap<usize, usize> = BTreeMap::new();
    for &face in &faces {
        let face = topo.face(face).unwrap();
        let tag = face.surface().type_tag();
        assert!(
            matches!(tag, "plane" | "torus"),
            "{what}: unexpected {tag} face"
        );
        tori += usize::from(tag == "torus");
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wire).unwrap().edges() {
                *uses.entry(oe.edge().index()).or_default() += 1;
            }
        }
    }
    assert!(tori >= 2, "{what}: expected torus faces, got {tori}");
    for (edge, count) in uses {
        assert_eq!(count, 2, "{what}: edge {edge} used {count} times");
    }
}

/// Every display mesh edge must be shared by exactly two triangles (watertight,
/// manifold mesh) with positive signed volume.
fn assert_watertight_mesh(topo: &Topology, solid: SolidId, deflection: f64, what: &str) {
    let mesh = tessellate_solid(topo, solid, deflection)
        .unwrap_or_else(|e| panic!("{what}: tessellate({deflection}) failed: {e:?}"));
    assert!(
        !mesh.indices.is_empty(),
        "{what}: empty mesh at deflection {deflection}"
    );
    let mut edge_counts: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_counts.entry(key).or_default() += 1;
        }
    }
    let boundary = edge_counts.values().filter(|&&c| c == 1).count();
    let non_manifold = edge_counts.values().filter(|&&c| c > 2).count();
    assert_eq!(
        (boundary, non_manifold),
        (0, 0),
        "{what}: mesh at deflection {deflection} has {boundary} boundary and {non_manifold} non-manifold edges"
    );
}

/// Signed volume enclosed by the display mesh at `deflection`.
fn mesh_volume(topo: &Topology, solid: SolidId, deflection: f64) -> f64 {
    let mesh = tessellate_solid(topo, solid, deflection).unwrap();
    mesh.indices
        .chunks_exact(3)
        .map(|t| {
            let a = mesh.positions[t[0] as usize];
            let b = mesh.positions[t[1] as usize];
            let c = mesh.positions[t[2] as usize];
            let (a, b, c) = (
                Vec3::new(a.x(), a.y(), a.z()),
                Vec3::new(b.x(), b.y(), b.z()),
                Vec3::new(c.x(), c.y(), c.z()),
            );
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

/// Mesh volume extrapolated to zero deflection from two fine meshes.
fn converged_volume(topo: &Topology, solid: SolidId) -> f64 {
    let (d1, d2) = (0.002, 0.0005);
    let (v1, v2) = (mesh_volume(topo, solid, d1), mesh_volume(topo, solid, d2));
    v2 + (v2 - v1) * d2 / (d1 - d2)
}

/// The result's volume matches `reference`: the converged mesh volume to
/// 5e-3, and `solid_volume` within its 3e-4 relative band.
fn assert_volume(topo: &Topology, solid: SolidId, reference: f64, what: &str) -> f64 {
    let converged = converged_volume(topo, solid);
    let measured = solid_volume(topo, solid, 0.002).unwrap();
    assert!(
        (converged - reference).abs() < 5e-3,
        "{what}: converged mesh volume {converged}, reference {reference}"
    );
    assert!(
        (measured - reference).abs() < 3e-4 * reference,
        "{what}: solid_volume {measured}, reference {reference}"
    );
    converged
}

/// Imported planar caps carry pcurves whose SameParameter proof is unavailable;
/// no other validation error may appear.
fn assert_valid_but_for_inherited_caps(topo: &Topology, solid: SolidId, what: &str) {
    let report = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    for issue in &report.issues {
        if issue.severity == remus_check::validate::Severity::Error {
            assert!(
                matches!(
                    issue.check,
                    remus_check::validate::CheckId::EdgeSameParameter
                ) && issue.description.contains("plane surface / circle edge"),
                "{what}: {issue:?}"
            );
        }
    }
}

fn build_revolved_tube(
    topo: &mut Topology,
    axis_point: Point3,
    axis_dir: Vec3,
    start: Point3,
    tube_radius: f64,
    sweep_rad: f64,
) -> SolidId {
    use remus_math::curves::Circle3D;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    let radial = start - axis_point;
    let tangent = axis_dir.cross(radial).normalize().unwrap();
    let circle = Circle3D::new(start, tangent, tube_radius).unwrap();
    let vertex = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let mut edge = Edge::new(vertex, vertex, EdgeCurve::Circle(circle));
    edge.set_trim(Some((0.0, std::f64::consts::TAU)));
    let edge = topo.add_edge(edge);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let d = tangent.dot(Vec3::new(start.x(), start.y(), start.z()));
    let profile = topo.add_face(Face::new(
        wire,
        Vec::new(),
        FaceSurface::Plane { normal: tangent, d },
    ));
    remus_operations::revolve::revolve(topo, profile, axis_point, axis_dir, sweep_rad).unwrap()
}

/// The same crossing sweep pair built by Remus's own revolve.
fn native_operands() -> (Topology, SolidId, SolidId) {
    let mut topo = Topology::new();
    let scale = 0.82;
    let target = build_revolved_tube(
        &mut topo,
        Point3::new(6.0, 1.0, (-53.0 / 22.0) * scale + 0.5),
        Vec3::new(0.0, 1.0, 0.0),
        Point3::new(-8.0 * scale + 6.0, 1.0, -5.0 * scale + 0.5),
        2.0 * scale,
        215.890_506_459_585_6_f64.to_radians(),
    );
    // Arc (6, -7, 1) -> (6, 1, 7) -> (6, 9, 1): centre (6, 1, -4/3), R = 25/3.
    let tool = build_revolved_tube(
        &mut topo,
        Point3::new(6.0, 1.0, -4.0 / 3.0),
        Vec3::new(-1.0, 0.0, 0.0),
        Point3::new(6.0, -7.0, 1.0),
        2.0,
        2.574_004_435_173_188,
    );
    (topo, target, tool)
}

#[test]
fn fixture_operands_hold_the_witnesses() {
    let (topo, target, tool) = operands();
    let vt = solid_volume(&topo, target, 0.005).unwrap();
    assert!((vt - TARGET_VOLUME).abs() < 0.02, "target vol {vt}");
    let vb = solid_volume(&topo, tool, 0.005).unwrap();
    assert!((vb - TOOL_VOLUME).abs() < 0.02, "tool vol {vb}");

    assert_eq!(class(&topo, target, WITNESS), PointClassification::Inside);
    assert_eq!(class(&topo, tool, WITNESS), PointClassification::Inside);
    assert_eq!(
        class(&topo, target, TARGET_ONLY),
        PointClassification::Inside
    );
    assert_eq!(
        class(&topo, tool, TARGET_ONLY),
        PointClassification::Outside
    );
    assert_eq!(
        class(&topo, target, TOOL_ONLY),
        PointClassification::Outside
    );
    assert_eq!(class(&topo, tool, TOOL_ONLY), PointClassification::Inside);
    assert_eq!(class(&topo, target, OUTSIDE), PointClassification::Outside);
    assert_eq!(class(&topo, tool, OUTSIDE), PointClassification::Outside);
}

type Entry = fn(&mut Topology, SolidId, SolidId) -> Result<SolidId, OperationsError>;

/// Cut entry points with the face count each returns: `boolean_with_options`
/// unifies same-domain faces by default, merging the two pieces of the tool's
/// torus that the section splits along its seam.
const CUT_ENTRIES: [(&str, Entry, usize); 4] = [
    ("boolean", |t, a, b| boolean(t, BooleanOp::Cut, a, b), 5),
    (
        "boolean_with_options",
        |t, a, b| boolean_with_options(t, BooleanOp::Cut, a, b, BooleanOptions::default()),
        4,
    ),
    (
        "entity evolution",
        |t, a, b| boolean_with_entity_evolution(t, BooleanOp::Cut, a, b).map(|(s, _)| s),
        5,
    ),
    (
        "journaled",
        |t, a, b| boolean_journaled(t, remus_algo::bop::BooleanOp::Cut, a, b).map(|r| r.solid),
        5,
    ),
];

#[test]
fn cut_is_exact_watertight_and_matches_reference_through_every_entry_point() {
    for (name, entry, faces) in CUT_ENTRIES {
        let (mut topo, target, tool) = operands();
        let drops = remus_algo::gfa::SectionDropScope::enter();
        let result =
            entry(&mut topo, target, tool).unwrap_or_else(|e| panic!("{name}: cut refused: {e:?}"));
        assert_eq!(drops.dropped(), 0, "{name}: dropped section chain");
        assert_eq!(drops.uncertified_charts(), 0, "{name}: uncertified chart");
        assert_eq!(drops.uncertified_rescues(), 0, "{name}: uncertified rescue");

        assert_exact_brep(&topo, result, faces, name);
        assert_valid_but_for_inherited_caps(&topo, result, name);
        for d in [0.1, 0.01, 0.002] {
            assert_watertight_mesh(&topo, result, d, name);
        }

        assert_volume(&topo, result, CUT_VOLUME, name);

        assert_eq!(
            class(&topo, result, WITNESS),
            PointClassification::Outside,
            "{name}: cut kept material inside both operands"
        );
        assert_eq!(
            class(&topo, result, TARGET_ONLY),
            PointClassification::Inside,
            "{name}: cut lost target-only material"
        );
        assert_eq!(
            class(&topo, result, TOOL_ONLY),
            PointClassification::Outside,
            "{name}: cut included tool-only material"
        );
        assert_eq!(
            class(&topo, result, OUTSIDE),
            PointClassification::Outside,
            "{name}: exterior classified inside"
        );
        // Source operands stay intact.
        assert_eq!(class(&topo, target, WITNESS), PointClassification::Inside);
        assert_eq!(class(&topo, tool, WITNESS), PointClassification::Inside);
    }
}

#[test]
fn reverse_cut_intersect_and_fuse_are_exact_in_both_orders() {
    // Reverse cut: tool - target
    {
        let (mut topo, target, tool) = operands();
        let drops = remus_algo::gfa::SectionDropScope::enter();
        let result = boolean(&mut topo, BooleanOp::Cut, tool, target)
            .unwrap_or_else(|e| panic!("reverse cut refused: {e:?}"));
        assert_eq!(drops.dropped(), 0, "reverse cut dropped section chain");
        assert_valid_but_for_inherited_caps(&topo, result, "reverse cut");
        for d in [0.1, 0.01, 0.002] {
            assert_watertight_mesh(&topo, result, d, "reverse cut");
        }
        assert_volume(&topo, result, REVERSE_CUT_VOLUME, "reverse cut");
        assert_eq!(class(&topo, result, WITNESS), PointClassification::Outside);
        assert_eq!(class(&topo, result, TOOL_ONLY), PointClassification::Inside);
        assert_eq!(
            class(&topo, result, TARGET_ONLY),
            PointClassification::Outside
        );
    }

    // Intersect and Fuse in both orders
    for (op, expected_faces, reference) in [
        (BooleanOp::Intersect, 3, COMMON_VOLUME),
        (BooleanOp::Fuse, 6, FUSE_VOLUME),
    ] {
        for swap in [false, true] {
            let what = format!("{op:?} swap={swap}");
            let (mut topo, target, tool) = operands();
            let (a, b) = if swap { (tool, target) } else { (target, tool) };
            let drops = remus_algo::gfa::SectionDropScope::enter();
            let result =
                boolean(&mut topo, op, a, b).unwrap_or_else(|e| panic!("{what}: refused: {e:?}"));
            assert_eq!(drops.dropped(), 0, "{what}: dropped section chain");
            assert_exact_brep(&topo, result, expected_faces, &what);
            assert_valid_but_for_inherited_caps(&topo, result, &what);
            for d in [0.1, 0.01, 0.002] {
                assert_watertight_mesh(&topo, result, d, &what);
            }
            assert_volume(&topo, result, reference, &what);
            assert_eq!(
                class(&topo, result, WITNESS),
                PointClassification::Inside,
                "{what}: lost shared witness"
            );
            let expect_leg = if op == BooleanOp::Fuse {
                PointClassification::Inside
            } else {
                PointClassification::Outside
            };
            assert_eq!(
                class(&topo, result, TARGET_ONLY),
                expect_leg,
                "{what}: target leg"
            );
            assert_eq!(
                class(&topo, result, TOOL_ONLY),
                expect_leg,
                "{what}: tool leg"
            );
            assert_eq!(
                class(&topo, result, OUTSIDE),
                PointClassification::Outside,
                "{what}: outside"
            );
        }
    }

    // Inclusion-exclusion on one arena.
    let (mut topo, target, tool) = operands();
    let cut = boolean(&mut topo, BooleanOp::Cut, target, tool).unwrap();
    let common = boolean(&mut topo, BooleanOp::Intersect, target, tool).unwrap();
    let fuse = boolean(&mut topo, BooleanOp::Fuse, target, tool).unwrap();
    let v_cut = converged_volume(&topo, cut);
    let v_common = converged_volume(&topo, common);
    let v_fuse = converged_volume(&topo, fuse);
    assert!(
        (v_cut + v_common - TARGET_VOLUME).abs() < 5e-3,
        "cut ({v_cut}) + common ({v_common}) != target ({TARGET_VOLUME})"
    );
    assert!(
        (v_fuse + v_common - (TARGET_VOLUME + TOOL_VOLUME)).abs() < 5e-3,
        "fuse ({v_fuse}) + common ({v_common}) != target + tool"
    );
}

#[test]
fn native_revolved_crossing_sweeps_are_exact_and_strict_valid() {
    for (op, reference) in [
        (BooleanOp::Cut, CUT_VOLUME),
        (BooleanOp::Intersect, COMMON_VOLUME),
        (BooleanOp::Fuse, FUSE_VOLUME),
    ] {
        let what = format!("native {op:?}");
        let (mut topo, target, tool) = native_operands();
        let vt = solid_volume(&topo, target, 0.005).unwrap();
        let vb = solid_volume(&topo, tool, 0.005).unwrap();
        assert!((vt - TARGET_VOLUME).abs() < 0.02, "{what}: target {vt}");
        assert!((vb - TOOL_VOLUME).abs() < 0.02, "{what}: tool {vb}");

        let drops = remus_algo::gfa::SectionDropScope::enter();
        let result = boolean(&mut topo, op, target, tool)
            .unwrap_or_else(|e| panic!("{what}: refused: {e:?}"));
        assert_eq!(drops.dropped(), 0, "{what}: dropped section chain");
        assert_watertight_mesh(&topo, result, 0.01, &what);
        assert_volume(&topo, result, reference, &what);
        let inside = class(&topo, result, WITNESS) == PointClassification::Inside;
        assert_eq!(inside, op != BooleanOp::Cut, "{what}: witness");
        let report = remus_check::validate::validate_solid(
            &topo,
            result,
            &remus_check::validate::ValidateOptions::default(),
        )
        .unwrap();
        assert!(report.is_valid(), "{what}: {report:?}");
    }
}
