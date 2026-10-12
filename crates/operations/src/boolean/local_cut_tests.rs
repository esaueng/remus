//! Oracle tests for the local drilled-hole cut (through and blind holes).
//!
//! Every qualified case is cut twice from the same topology — once through
//! the local cell and once with it disabled, so the general GFA builds the
//! result — and the two results must agree on entity counts, surface kinds,
//! hole loops, volume, validity and a watertight mesh. Refusal cases must
//! leave the cell unused.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::f64::consts::{FRAC_PI_2, PI};

use remus_math::mat::Mat4;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

use super::local_cut::{BUILT, DISABLED};
use super::{BooleanOp, boolean};
use crate::primitives::{make_box, make_cylinder};
use crate::transform::transform_solid;

#[derive(Debug, PartialEq)]
struct Shape {
    faces: usize,
    edges: usize,
    vertices: usize,
    /// Sorted `(surface kind, reversed, inner wire count)` per face.
    face_kinds: Vec<(String, bool, usize)>,
}

fn shape(topo: &Topology, solid: SolidId) -> Shape {
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    let mut face_kinds: Vec<_> = faces
        .iter()
        .map(|&face| {
            let face = topo.face(face).unwrap();
            (
                face.surface().type_tag().to_string(),
                face.is_reversed(),
                face.inner_wires().len(),
            )
        })
        .collect();
    face_kinds.sort();
    Shape {
        faces: faces.len(),
        edges: remus_topology::explorer::solid_edges(topo, solid)
            .unwrap()
            .len(),
        vertices: remus_topology::explorer::solid_vertices(topo, solid)
            .unwrap()
            .len(),
        face_kinds,
    }
}

/// The solid's attributes, then every face in shell order with its
/// attributes, surface, orientation and wires, every wire use with its
/// edge's id, curve, trim and tolerance, and every vertex with its id, point
/// and tolerance — `Debug` prints each `f64` exactly, so equal dumps mean
/// bit-identical results, entity numbering included.
fn dump(topo: &Topology, solid: SolidId) -> String {
    use std::fmt::Write;
    let mut out = format!("solid {:?}\n", topo.attributes().solid(solid));
    for face_id in remus_topology::explorer::solid_faces(topo, solid).unwrap() {
        let face = topo.face(face_id).unwrap();
        writeln!(
            out,
            "face {} reversed={} {:?} {:?}",
            face_id.index(),
            face.is_reversed(),
            face.surface(),
            topo.attributes().face(face_id)
        )
        .unwrap();
        for &loop_id in topo.loops_of_face(face_id).unwrap() {
            for &coedge_id in topo.face_loop(loop_id).unwrap().coedges() {
                let coedge = topo.coedge(coedge_id).unwrap();
                writeln!(
                    out,
                    "  coedge edge={} forward={} winding={:?} pcurve={:?}",
                    coedge.edge().index(),
                    coedge.is_forward(),
                    coedge.periodic_winding(),
                    coedge.pcurve()
                )
                .unwrap();
            }
        }
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            let wire = topo.wire(wire_id).unwrap();
            writeln!(
                out,
                "  wire {} closed={}",
                wire_id.index(),
                wire.is_closed()
            )
            .unwrap();
            for oriented in wire.edges() {
                let edge = topo.edge(oriented.edge()).unwrap();
                let vertex = |id: remus_topology::vertex::VertexId| {
                    let v = topo.vertex(id).unwrap();
                    format!("v{} {:?} tol={:?}", id.index(), v.point(), v.tolerance())
                };
                writeln!(
                    out,
                    "    edge {} forward={} {:?} trim={:?} tol={:?} [{} -> {}]",
                    oriented.edge().index(),
                    oriented.is_forward(),
                    edge.curve(),
                    edge.trim(),
                    edge.tolerance(),
                    vertex(edge.start()),
                    vertex(edge.end())
                )
                .unwrap();
            }
        }
    }
    out
}

/// [`dump`] with every float dropped: entity ids, orientations, curve and
/// surface kinds only.
fn structure(topo: &Topology, solid: SolidId) -> String {
    dump(topo, solid)
        .lines()
        .map(|line| {
            line.split_whitespace()
                .filter(|token| !(token.contains('.') || token.contains("e-")))
                .collect::<Vec<_>>()
                .join(" ")
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// Largest distance between same-id vertices of two results.
fn vertex_drift(a: (&Topology, SolidId), b: (&Topology, SolidId)) -> f64 {
    let vertices = remus_topology::explorer::solid_vertices(a.0, a.1).unwrap();
    let others = remus_topology::explorer::solid_vertices(b.0, b.1).unwrap();
    assert_eq!(vertices, others, "vertex ids differ");
    vertices
        .iter()
        .map(|&v| (a.0.vertex(v).unwrap().point() - b.0.vertex(v).unwrap().point()).length())
        .fold(0.0, f64::max)
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    crate::measure::solid_volume(topo, solid, 0.01).unwrap()
}

fn assert_sound(topo: &Topology, solid: SolidId, label: &str) {
    let report = crate::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: invalid result {report:?}");
    let mesh = crate::tessellate::tessellate_solid(topo, solid, 0.05).unwrap();
    assert_eq!(
        crate::tessellate::boundary_edge_count(&mesh),
        0,
        "{label}: open mesh"
    );
}

/// Cut `tool` from `blank` locally and through the GFA; returns both.
fn cut_both(
    topo: &Topology,
    blank: SolidId,
    tool: SolidId,
) -> ((Topology, SolidId), (Topology, SolidId)) {
    let mut local = topo.clone();
    let built = BUILT.with(std::cell::Cell::get);
    let local_result = boolean(&mut local, BooleanOp::Cut, blank, tool).unwrap();
    assert_eq!(
        BUILT.with(std::cell::Cell::get),
        built + 1,
        "the local cell declined a qualified cut"
    );
    let mut general = topo.clone();
    DISABLED.with(|disabled| disabled.set(true));
    let general_result = boolean(&mut general, BooleanOp::Cut, blank, tool);
    DISABLED.with(|disabled| disabled.set(false));
    ((local, local_result), (general, general_result.unwrap()))
}

fn assert_matches_gfa(
    topo: &Topology,
    blank: SolidId,
    tool: SolidId,
    label: &str,
) -> (Topology, SolidId) {
    let ((local, local_result), (general, general_result)) = cut_both(topo, blank, tool);
    assert_eq!(
        shape(&local, local_result),
        shape(&general, general_result),
        "{label}: topology differs from the GFA"
    );
    assert_eq!(
        dump(&local, local_result),
        dump(&general, general_result),
        "{label}: result is not bit-identical to the GFA's"
    );
    let (v_local, v_general) = (
        volume(&local, local_result),
        volume(&general, general_result),
    );
    assert!(
        (v_local - v_general).abs() <= 1e-9 * v_general.abs().max(1.0),
        "{label}: volume {v_local} vs GFA {v_general}"
    );
    assert_sound(&local, local_result, label);
    (local, local_result)
}

fn placed(topo: &mut Topology, solid: SolidId, matrix: &Mat4) -> SolidId {
    transform_solid(topo, solid, matrix).unwrap();
    solid
}

fn plate(topo: &mut Topology) -> SolidId {
    make_box(topo, 40.0, 40.0, 10.0).unwrap()
}

fn drill(topo: &mut Topology, radius: f64, height: f64, matrix: &Mat4) -> SolidId {
    let tool = make_cylinder(topo, radius, height).unwrap();
    placed(topo, tool, matrix)
}

#[test]
fn vertical_through_hole_matches_gfa() {
    let mut topo = Topology::new();
    let blank = plate(&mut topo);
    let tool = drill(&mut topo, 2.0, 20.0, &Mat4::translation(10.0, 10.0, -5.0));
    let (local, result) = assert_matches_gfa(&topo, blank, tool, "vertical");
    let expected = 40.0 * 40.0 * 10.0 - PI * 4.0 * 10.0;
    let measured = volume(&local, result);
    assert!((measured - expected).abs() < 1e-6 * expected, "{measured}");
}

#[test]
fn downward_tool_matches_gfa() {
    let mut topo = Topology::new();
    let blank = plate(&mut topo);
    // Axis −z: flipped about x, then lifted so it spans z ∈ [−5, 15].
    let tool = drill(
        &mut topo,
        2.0,
        20.0,
        &(Mat4::translation(25.0, 12.0, 15.0) * Mat4::rotation_x(PI)),
    );
    assert_matches_gfa(&topo, blank, tool, "downward");
}

#[test]
fn horizontal_hole_through_side_faces_matches_gfa() {
    let mut topo = Topology::new();
    let blank = plate(&mut topo);
    // Axis +x through the x = 0 and x = 40 walls at mid-thickness.
    let tool = drill(
        &mut topo,
        2.0,
        60.0,
        &(Mat4::translation(-10.0, 20.0, 5.0) * Mat4::rotation_y(FRAC_PI_2)),
    );
    assert_matches_gfa(&topo, blank, tool, "horizontal");
}

#[test]
fn rotated_plate_and_tool_match_gfa() {
    let mut topo = Topology::new();
    let frame = Mat4::rotation_z(0.3) * Mat4::rotation_x(0.2);
    let blank = plate(&mut topo);
    let blank = placed(&mut topo, blank, &frame);
    let tool = drill(
        &mut topo,
        3.0,
        20.0,
        &(frame * Mat4::translation(20.0, 20.0, -5.0)),
    );
    // A rotation can leave the plate normal and the tool axis a few ulps
    // apart; the cell then declines rather than guess. Either way the result
    // must equal the GFA's.
    let mut local = topo.clone();
    let before = BUILT.with(std::cell::Cell::get);
    let local_result = boolean(&mut local, BooleanOp::Cut, blank, tool).unwrap();
    let mut general = topo.clone();
    DISABLED.with(|disabled| disabled.set(true));
    let general_result = boolean(&mut general, BooleanOp::Cut, blank, tool).unwrap();
    DISABLED.with(|disabled| disabled.set(false));
    if BUILT.with(std::cell::Cell::get) > before {
        // Off the coordinate axes the GFA reaches the rim frames and seam
        // vertices through different arithmetic, so the two results agree
        // entity for entity but differ by ulps in those coordinates.
        assert_eq!(shape(&local, local_result), shape(&general, general_result));
        assert_eq!(
            structure(&local, local_result),
            structure(&general, general_result)
        );
        let drift = vertex_drift((&local, local_result), (&general, general_result));
        assert!(drift < 1e-9, "vertex drift {drift}");
        assert_sound(&local, local_result, "rotated");
    }
    let (v_local, v_general) = (
        volume(&local, local_result),
        volume(&general, general_result),
    );
    assert!((v_local - v_general).abs() <= 1e-9 * v_general);
}

#[test]
fn drilled_grid_matches_gfa_hole_by_hole() {
    let mut local = Topology::new();
    let mut local_blank = make_box(&mut local, 50.0, 50.0, 10.0).unwrap();
    let mut general = local.clone();
    let mut general_blank = local_blank;
    for row in 0..4 {
        for col in 0..4 {
            let at = Mat4::translation(
                10.0 + f64::from(col) * 10.0,
                10.0 + f64::from(row) * 10.0,
                -5.0,
            );
            let local_tool = drill(&mut local, 3.0, 20.0, &at);
            let before = BUILT.with(std::cell::Cell::get);
            local_blank = boolean(&mut local, BooleanOp::Cut, local_blank, local_tool).unwrap();
            assert_eq!(
                BUILT.with(std::cell::Cell::get),
                before + 1,
                "hole {row},{col}"
            );

            let general_tool = drill(&mut general, 3.0, 20.0, &at);
            DISABLED.with(|disabled| disabled.set(true));
            general_blank =
                boolean(&mut general, BooleanOp::Cut, general_blank, general_tool).unwrap();
            DISABLED.with(|disabled| disabled.set(false));
            assert_eq!(
                shape(&local, local_blank),
                shape(&general, general_blank),
                "hole {row},{col}"
            );
            assert_eq!(
                dump(&local, local_blank),
                dump(&general, general_blank),
                "hole {row},{col}: not bit-identical"
            );
        }
    }
    let expected = 50.0 * 50.0 * 10.0 - 16.0 * PI * 9.0 * 10.0;
    let measured = volume(&local, local_blank);
    assert!((measured - expected).abs() < 1e-6 * expected, "{measured}");
    assert!((measured - volume(&general, general_blank)).abs() < 1e-9 * expected);
    assert_sound(&local, local_blank, "grid");
}

#[test]
fn hole_beside_a_rectangular_opening_matches_gfa() {
    let mut topo = Topology::new();
    let blank = plate(&mut topo);
    let window = make_box(&mut topo, 10.0, 10.0, 20.0).unwrap();
    let window = placed(&mut topo, window, &Mat4::translation(15.0, 15.0, -5.0));
    let blank = boolean(&mut topo, BooleanOp::Cut, blank, window).unwrap();
    // Circle 1 mm clear of the opening's straight edges.
    let tool = drill(&mut topo, 2.0, 20.0, &Mat4::translation(28.0, 20.0, -5.0));
    assert_matches_gfa(&topo, blank, tool, "beside opening");
}

/// A cut the cell must leave to the GFA (whatever the GFA then returns —
/// overlapping holes, for one, are its typed refusal).
fn assert_declined(topo: &Topology, blank: SolidId, tool: SolidId, label: &str) {
    let mut copy = topo.clone();
    let before = BUILT.with(std::cell::Cell::get);
    let _ = boolean(&mut copy, BooleanOp::Cut, blank, tool);
    assert_eq!(
        BUILT.with(std::cell::Cell::get),
        before,
        "{label}: the local cell accepted it"
    );
}

#[test]
fn cell_declines_unqualified_cuts() {
    let mut topo = Topology::new();
    let blank = plate(&mut topo);
    let first = drill(&mut topo, 2.0, 20.0, &Mat4::translation(10.0, 10.0, -5.0));
    let drilled = boolean(&mut topo, BooleanOp::Cut, blank, first).unwrap();

    let cases: Vec<(&str, SolidId, SolidId)> = vec![
        (
            "overlaps an existing hole",
            drilled,
            drill(&mut topo, 2.0, 20.0, &Mat4::translation(13.0, 10.0, -5.0)),
        ),
        (
            "blind floor within the clearance of the bottom face",
            blank,
            drill(&mut topo, 2.0, 20.0, &Mat4::translation(20.0, 20.0, 1e-5)),
        ),
        (
            "blind hole overlapping an existing hole",
            drilled,
            drill(&mut topo, 1.0, 20.0, &Mat4::translation(10.0, 12.5, 5.0)),
        ),
        (
            "blind hole drilled sideways out of a hole's wall",
            drilled,
            drill(
                &mut topo,
                0.5,
                10.0,
                &(Mat4::translation(10.0, 10.0, 5.0) * Mat4::rotation_x(-FRAC_PI_2)),
            ),
        ),
        (
            "blind hole entering a side wall near a corner",
            blank,
            drill(
                &mut topo,
                2.0,
                10.0,
                &(Mat4::translation(-5.0, 1.0, 5.0) * Mat4::rotation_y(FRAC_PI_2)),
            ),
        ),
        (
            "base coplanar with the bottom face",
            blank,
            drill(&mut topo, 2.0, 20.0, &Mat4::translation(20.0, 20.0, 0.0)),
        ),
        (
            "tilted tool",
            blank,
            drill(
                &mut topo,
                2.0,
                20.0,
                &(Mat4::translation(20.0, 20.0, -5.0) * Mat4::rotation_x(0.1)),
            ),
        ),
        (
            "breaks through a side wall",
            blank,
            drill(&mut topo, 2.0, 20.0, &Mat4::translation(1.0, 20.0, -5.0)),
        ),
        (
            "wider than the plate",
            blank,
            drill(&mut topo, 30.0, 20.0, &Mat4::translation(20.0, 20.0, -5.0)),
        ),
    ];
    for (label, blank, tool) in cases {
        assert_declined(&topo, blank, tool, label);
    }

    // Two stacked plates in one body: the tool pierces four faces.
    let lower = plate(&mut topo);
    let upper = plate(&mut topo);
    let upper = placed(&mut topo, upper, &Mat4::translation(0.0, 0.0, 20.0));
    let stack = boolean(&mut topo, BooleanOp::Fuse, lower, upper).unwrap();
    let long = drill(&mut topo, 2.0, 50.0, &Mat4::translation(20.0, 20.0, -5.0));
    assert_declined(&topo, stack, long, "two stacked plates");

    // A cylindrical blank cut by a box tool is not a drilling at all.
    let rod = make_cylinder(&mut topo, 5.0, 30.0).unwrap();
    let block = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    assert_declined(&topo, rod, block, "box tool");
}

#[test]
fn blind_holes_match_gfa() {
    let blind = 40.0 * 40.0 * 10.0 - PI * 4.0 * 5.0;
    let cases = [
        // Tool up from inside the plate: the floor is the tool's base cap.
        (
            "into the top face",
            Mat4::translation(20.0, 20.0, 5.0),
            20.0,
        ),
        // Tool up from below: the floor is the tool's top cap.
        (
            "into the bottom face",
            Mat4::translation(20.0, 20.0, -15.0),
            20.0,
        ),
        // Axis −z from above, the floor is the far cap at z = 5.
        (
            "downward into the top face",
            Mat4::translation(25.0, 12.0, 15.0) * Mat4::rotation_x(PI),
            10.0,
        ),
        // Axis +x into the x = 0 wall, 5 deep.
        (
            "into a side wall",
            Mat4::translation(-10.0, 20.0, 5.0) * Mat4::rotation_y(FRAC_PI_2),
            15.0,
        ),
    ];
    for (label, at, height) in cases {
        let mut topo = Topology::new();
        let blank = plate(&mut topo);
        let tool = drill(&mut topo, 2.0, height, &at);
        let (local, result) = assert_matches_gfa(&topo, blank, tool, label);
        let measured = volume(&local, result);
        assert!(
            (measured - blind).abs() < 1e-6 * blind,
            "{label}: {measured}"
        );
    }
}

/// Cut `holes` (radius, height, placement) one after another, locally and
/// through the GFA, requiring every cut to build locally and the two
/// results to stay bit-identical after each one.
fn assert_sequence_matches_gfa(
    topo: &Topology,
    blank: SolidId,
    holes: &[(f64, f64, Mat4)],
    label: &str,
) -> (Topology, SolidId) {
    let mut local = topo.clone();
    let mut general = topo.clone();
    let (mut local_blank, mut general_blank) = (blank, blank);
    for (index, (radius, height, at)) in holes.iter().enumerate() {
        let local_tool = drill(&mut local, *radius, *height, at);
        let before = BUILT.with(std::cell::Cell::get);
        local_blank = boolean(&mut local, BooleanOp::Cut, local_blank, local_tool).unwrap();
        assert_eq!(
            BUILT.with(std::cell::Cell::get),
            before + 1,
            "{label}: hole {index} declined"
        );
        let general_tool = drill(&mut general, *radius, *height, at);
        DISABLED.with(|disabled| disabled.set(true));
        general_blank = boolean(&mut general, BooleanOp::Cut, general_blank, general_tool).unwrap();
        DISABLED.with(|disabled| disabled.set(false));
        assert_eq!(
            dump(&local, local_blank),
            dump(&general, general_blank),
            "{label}: hole {index} not bit-identical"
        );
    }
    assert_sound(&local, local_blank, label);
    (local, local_blank)
}

#[test]
fn mixed_blind_and_through_grid_matches_gfa_hole_by_hole() {
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, 50.0, 50.0, 10.0).unwrap();
    let mut holes = Vec::new();
    let mut expected = 50.0 * 50.0 * 10.0;
    for row in 0..4 {
        for col in 0..4 {
            let (x, y) = (10.0 + f64::from(col) * 10.0, 10.0 + f64::from(row) * 10.0);
            // Through, blind from the top 4 deep, blind from the bottom
            // 3 deep, downward blind 6 deep.
            let (at, height, depth) = match (row + col) % 4 {
                0 => (Mat4::translation(x, y, -5.0), 20.0, 10.0),
                1 => (Mat4::translation(x, y, 6.0), 20.0, 4.0),
                2 => (Mat4::translation(x, y, -17.0), 20.0, 3.0),
                _ => (
                    Mat4::translation(x, y, 14.0) * Mat4::rotation_x(PI),
                    10.0,
                    6.0,
                ),
            };
            holes.push((3.0, height, at));
            expected -= PI * 9.0 * depth;
        }
    }
    let (local, result) = assert_sequence_matches_gfa(&topo, blank, &holes, "mixed grid");
    let measured = volume(&local, result);
    assert!((measured - expected).abs() < 1e-6 * expected, "{measured}");
}

#[test]
fn blind_magnet_holes_match_gfa_hole_by_hole() {
    // The gridfinity baseplate bench: Ø6 magnet pockets whose tool tops out
    // at z = 4.5 inside a 4.65-thick unit, leaving a 0.15 floor.
    let mut topo = Topology::new();
    let unit = make_box(&mut topo, 42.0, 42.0, 4.65).unwrap();
    let holes: Vec<_> = [(4.0, 4.0), (38.0, 4.0), (4.0, 38.0), (38.0, 38.0)]
        .into_iter()
        .map(|(x, y)| (3.0, 5.0, Mat4::translation(x, y, -0.5)))
        .collect();
    let (local, result) = assert_sequence_matches_gfa(&topo, unit, &holes, "magnet pockets");
    let expected = 42.0 * 42.0 * 4.65 - 4.0 * PI * 9.0 * 4.5;
    let measured = volume(&local, result);
    assert!((measured - expected).abs() < 1e-6 * expected, "{measured}");
}

#[test]
fn rotated_blind_hole_matches_gfa() {
    let mut topo = Topology::new();
    // A small tilt: steeper, and the tilted bottom face's axis-aligned box
    // reaches the tool's, so the cell (correctly) declines.
    let frame = Mat4::rotation_z(0.3) * Mat4::rotation_x(0.05);
    let blank = plate(&mut topo);
    let blank = placed(&mut topo, blank, &frame);
    let tool = drill(
        &mut topo,
        3.0,
        20.0,
        &(frame * Mat4::translation(20.0, 20.0, 4.0)),
    );
    let ((local, local_result), (general, general_result)) = cut_both(&topo, blank, tool);
    // As for through holes off the axes: the same entities, with the entry
    // rim frame and seam vertex a few ulps apart.
    assert_eq!(
        structure(&local, local_result),
        structure(&general, general_result)
    );
    let drift = vertex_drift((&local, local_result), (&general, general_result));
    assert!(drift < 1e-9, "vertex drift {drift}");
    assert_sound(&local, local_result, "rotated blind");
    let expected = 40.0 * 40.0 * 10.0 - PI * 9.0 * 6.0;
    for (topo, solid) in [(&local, local_result), (&general, general_result)] {
        let measured = volume(topo, solid);
        assert!((measured - expected).abs() < 1e-6 * expected, "{measured}");
    }

    let steep = Mat4::rotation_z(0.3) * Mat4::rotation_x(0.2);
    let blank = plate(&mut topo);
    let blank = placed(&mut topo, blank, &steep);
    let tool = drill(
        &mut topo,
        3.0,
        20.0,
        &(steep * Mat4::translation(20.0, 20.0, 4.0)),
    );
    assert_declined(&topo, blank, tool, "steeply tilted blind hole");
}

#[test]
fn local_cuts_drop_blank_attributes_like_the_gfa() {
    // GFA results carry no solid or face attributes; a local cut must not
    // keep them either, or the result would depend on which path ran.
    let mut topo = Topology::new();
    let blank = plate(&mut topo);
    let attributes = remus_topology::attributes::EntityAttributes {
        name: Some("plate".into()),
        color: None,
    };
    for face in remus_topology::explorer::solid_faces(&topo, blank).unwrap() {
        topo.set_face_attributes(face, attributes.clone()).unwrap();
    }
    topo.set_solid_attributes(blank, attributes).unwrap();
    let through = drill(&mut topo, 2.0, 20.0, &Mat4::translation(10.0, 10.0, -5.0));
    let blind = drill(&mut topo, 2.0, 20.0, &Mat4::translation(30.0, 30.0, 5.0));
    for (label, tool) in [("through", through), ("blind", blind)] {
        let (local, result) = assert_matches_gfa(&topo, blank, tool, label);
        assert!(local.attributes().solid(result).is_none(), "{label}");
    }
}

#[test]
fn ruled_face_bound_encloses_every_meshed_point() {
    let mut topo = Topology::new();
    let blank = plate(&mut topo);
    let tool = drill(&mut topo, 2.0, 20.0, &Mat4::translation(10.0, 10.0, -5.0));
    let drilled = boolean(&mut topo, BooleanOp::Cut, blank, tool).unwrap();
    let rod = make_cylinder(&mut topo, 3.0, 7.0).unwrap();
    let rod = placed(
        &mut topo,
        rod,
        &(Mat4::translation(5.0, -2.0, 1.0) * Mat4::rotation_x(0.7)),
    );
    let cone = crate::primitives::make_cone(&mut topo, 4.0, 0.0, 6.0).unwrap();
    let frustum = crate::primitives::make_cone(&mut topo, 4.0, 1.5, 6.0).unwrap();
    let mut checked = 0;
    for solid in [drilled, rod, cone, frustum] {
        for face in remus_topology::explorer::solid_faces(&topo, solid).unwrap() {
            let Some(bounds) = super::local_cut::ruled_face_bound(&topo, face).unwrap() else {
                continue;
            };
            let grown = bounds.expanded(1e-9);
            let mesh = crate::tessellate::tessellate(&topo, face, 0.01).unwrap();
            for point in &mesh.positions {
                assert!(
                    grown.contains_point(*point),
                    "face {} point {point:?} outside {bounds:?}",
                    face.index()
                );
            }
            checked += 1;
        }
    }
    assert!(checked >= 12, "checked only {checked} faces");
}
