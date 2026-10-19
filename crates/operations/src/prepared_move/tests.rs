#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::fmt::Write as _;

use remus_math::mat::Mat4;
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::face::FaceId;
use remus_topology::journal::{EntityKey, EntityKind};
use remus_topology::solid::SolidId;

use super::PreparedFaceMove;
use crate::boolean::{BooleanOp, boolean};
use crate::evolution::EvolutionMap;
use crate::{blend_ops, primitives, transform};

fn translate(topo: &mut Topology, solid: SolidId, x: f64, y: f64, z: f64) {
    transform::transform_solid(topo, solid, &Mat4::translation(x, y, z)).unwrap();
}

/// `solid`'s content with every handle replaced by its first-visit ordinal:
/// equal for two solids exactly when they hold the same geometry, bit for
/// bit, in the same structure.
fn canonical(topo: &Topology, solid: SolidId) -> String {
    let mut out = String::new();
    let mut edges = BTreeMap::new();
    let mut vertices = BTreeMap::new();
    let ordinal = |map: &mut BTreeMap<usize, usize>, index: usize| {
        let next = map.len();
        *map.entry(index).or_insert(next)
    };
    for face_id in solid_faces(topo, solid).unwrap() {
        let face = topo.face(face_id).unwrap();
        writeln!(
            out,
            "face {:?} reversed {}",
            face.surface(),
            face.is_reversed()
        )
        .unwrap();
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            let wire = topo.wire(wire_id).unwrap();
            writeln!(out, " wire closed {}", wire.is_closed()).unwrap();
            for oriented in wire.edges() {
                let edge = topo.edge(oriented.edge()).unwrap();
                let fresh = !edges.contains_key(&oriented.edge().index());
                let e = ordinal(&mut edges, oriented.edge().index());
                let pcurve = topo
                    .pcurve_oriented(oriented.edge(), face_id, oriented.is_forward())
                    .map(|p| format!("{:?} {:?} {:?}", p.curve(), p.t_start(), p.t_end()));
                writeln!(
                    out,
                    "  edge {e} forward {} pcurve {pcurve:?}",
                    oriented.is_forward()
                )
                .unwrap();
                if fresh {
                    let start = ordinal(&mut vertices, edge.start().index());
                    let end = ordinal(&mut vertices, edge.end().index());
                    writeln!(
                        out,
                        "   {:?} trim {:?} tol {:?} from {start} to {end}",
                        edge.curve(),
                        edge.trim(),
                        edge.tolerance()
                    )
                    .unwrap();
                }
            }
        }
    }
    for vertex_id in solid_vertices(topo, solid).unwrap() {
        let vertex = topo.vertex(vertex_id).unwrap();
        writeln!(
            out,
            "vertex {} {:?} {:?}",
            ordinal(&mut vertices, vertex_id.index()),
            vertex.point(),
            vertex.tolerance()
        )
        .unwrap();
    }
    out
}

/// `solid`'s faces as an order-free set of geometric signatures (surface
/// kind, orientation, area and boundary vertices to 1e-9), for results whose
/// construction order is not deterministic: the fillet engine the
/// remove/rebuild path calls builds corner patches in hash-map order, so two
/// unprepared runs of the same move already differ in face order and in the
/// last bits of a corner sphere's frame.
fn signature(topo: &Topology, solid: SolidId) -> String {
    let mut faces: Vec<String> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(|face| face_signature(topo, face))
        .collect();
    faces.sort_unstable();
    format!(
        "{:?}\n{}",
        remus_topology::explorer::solid_entity_counts(topo, solid).unwrap(),
        faces.join("\n")
    )
}

fn face_signature(topo: &Topology, face: FaceId) -> String {
    let round = |x: f64| (x * 1e9).round() / 1e9;
    let mut points: Vec<String> = remus_topology::explorer::face_vertices(topo, face)
        .unwrap()
        .into_iter()
        .map(|v| {
            let p = topo.vertex(v).unwrap().point();
            format!("({}, {}, {})", round(p.x()), round(p.y()), round(p.z()))
        })
        .collect();
    points.sort_unstable();
    let data = topo.face(face).unwrap();
    format!(
        "{} reversed {} area {} at {points:?}",
        data.surface().type_tag(),
        data.is_reversed(),
        round(crate::measure::face_area(topo, face, 0.01).unwrap())
    )
}

/// [`history`] by face signature, for the same results [`signature`] serves.
fn signature_history(topo: &Topology, map: &EvolutionMap) -> String {
    let face = |index: usize| face_signature(topo, topo.face_id_from_index(index).unwrap());
    let mut modified: Vec<_> = map
        .modified
        .iter()
        .map(|(input, outputs)| (*input, outputs.iter().map(|&o| face(o)).collect::<Vec<_>>()))
        .collect();
    modified.sort_unstable();
    format!(
        "{:?} modified {modified:?} unresolved {}",
        map.origin,
        map.unresolved.len()
    )
}

/// A move's face history and boundary correspondence, sources by handle
/// (shared by both runs), results by their ordinal in the result solid.
fn history(
    topo: &Topology,
    solid: SolidId,
    map: &EvolutionMap,
    boundary: &[(EntityKey, EntityKey)],
) -> String {
    let faces: BTreeMap<usize, usize> = solid_faces(topo, solid)
        .unwrap()
        .iter()
        .enumerate()
        .map(|(i, f)| (f.index(), i))
        .collect();
    let edges: BTreeMap<usize, usize> = solid_edges(topo, solid)
        .unwrap()
        .iter()
        .enumerate()
        .map(|(i, e)| (e.index(), i))
        .collect();
    let vertices: BTreeMap<usize, usize> = solid_vertices(topo, solid)
        .unwrap()
        .iter()
        .enumerate()
        .map(|(i, v)| (v.index(), i))
        .collect();
    let mut modified: Vec<_> = map
        .modified
        .iter()
        .map(|(input, outputs)| (*input, outputs.iter().map(|o| faces[o]).collect::<Vec<_>>()))
        .collect();
    modified.sort_unstable();
    let mut generated: Vec<_> = map
        .generated
        .iter()
        .map(|(input, outputs)| (*input, outputs.iter().map(|o| faces[o]).collect::<Vec<_>>()))
        .collect();
    generated.sort_unstable();
    let mut deleted: Vec<_> = map.deleted.iter().copied().collect();
    deleted.sort_unstable();
    let unresolved: Vec<_> = map
        .unresolved
        .iter()
        .map(|(output, inputs)| (faces[output], inputs.clone()))
        .collect();
    let mut pairs: Vec<String> = boundary
        .iter()
        .map(|(source, target)| {
            let ordinal = match target.kind {
                EntityKind::Edge => edges.get(&target.index),
                EntityKind::Vertex => vertices.get(&target.index),
                EntityKind::Face => faces.get(&target.index),
            };
            format!("{source:?} -> {:?} {ordinal:?}", target.kind)
        })
        .collect();
    pairs.sort_unstable();
    format!(
        "{:?} modified {modified:?} generated {generated:?} deleted {deleted:?} \
         unresolved {unresolved:?} boundary {pairs:?}",
        map.origin
    )
}

/// One move's observable outcome: the result's content and history, or the
/// refusal text, and whether the source was left intact.
fn outcome(
    topo: &Topology,
    source: SolidId,
    before: &str,
    result: Result<crate::push_pull::DirectEditEvolution, crate::OperationsError>,
    exact: bool,
) -> String {
    assert_eq!(canonical(topo, source), before, "the source is left intact");
    match result {
        Ok((moved, boundary)) if exact => format!(
            "ok\n{}\n{}",
            canonical(topo, moved.solid),
            history(topo, moved.solid, &moved.evolution, &boundary)
        ),
        Ok((moved, boundary)) => format!(
            "ok\n{}\n{}\nboundary pairs {}",
            signature(topo, moved.solid),
            signature_history(topo, &moved.evolution),
            boundary.len()
        ),
        Err(error) => format!("refused: {error}"),
    }
}

/// Moves `faces` of `solid` to each distance in turn, restoring the source
/// between moves as a drag does, unprepared in one copy of the document and
/// prepared in another, and asserts every outcome is the same: bit for bit
/// when `exact`, else by [`signature`].
fn assert_prepared_matches(
    topo: &Topology,
    solid: SolidId,
    faces: &[FaceId],
    distances: &[f64],
    exact: bool,
) -> Vec<String> {
    let checkpoint = topo.clone();
    let before = canonical(topo, solid);
    let mut plain = topo.clone();
    let mut prepared_topo = topo.clone();
    let mut prepared = PreparedFaceMove::prepare(&prepared_topo, solid, faces).unwrap();
    let mut outcomes = Vec::new();
    for &distance in distances {
        plain.restore_preserving_handle_slots(&checkpoint);
        prepared_topo.restore_preserving_handle_slots(&checkpoint);
        let moved =
            crate::push_pull::move_faces_with_entity_evolution(&mut plain, solid, faces, distance);
        let expected = outcome(&plain, solid, &before, moved, exact);
        assert!(prepared.is_current(&prepared_topo));
        let moved = prepared.edit(&mut prepared_topo, distance);
        let actual = outcome(&prepared_topo, solid, &before, moved, exact);
        assert_same(&actual, &expected, &format!("distance {distance}"));
        outcomes.push(expected);
    }
    // Moves without a restore in between start from the same source too.
    for &distance in distances.iter().rev().take(3) {
        let moved =
            crate::push_pull::move_faces_with_entity_evolution(&mut plain, solid, faces, distance);
        let expected = outcome(&plain, solid, &before, moved, exact);
        let moved = prepared.edit(&mut prepared_topo, distance);
        let actual = outcome(&prepared_topo, solid, &before, moved, exact);
        assert_same(
            &actual,
            &expected,
            &format!("distance {distance} without restore"),
        );
    }
    outcomes
}

/// `assert_eq!` that names the first line where two outcomes part.
fn assert_same(actual: &str, expected: &str, what: &str) {
    if actual == expected {
        return;
    }
    let (a, e): (Vec<_>, Vec<_>) = (actual.lines().collect(), expected.lines().collect());
    let line = a
        .iter()
        .zip(&e)
        .position(|(x, y)| x != y)
        .unwrap_or_else(|| a.len().min(e.len()));
    panic!(
        "{what}: prepared and unprepared outcomes differ at line {line} of {}/{}:\n  prepared:   {:?}\n  unprepared: {:?}",
        a.len(),
        e.len(),
        a.get(line),
        e.get(line)
    );
}

fn top_face(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            topo.face(f)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.z() > 0.999)
        })
        .max_by(|&a, &b| {
            let area = |f| crate::measure::face_area(topo, f, 0.1).unwrap();
            area(a).total_cmp(&area(b))
        })
        .unwrap()
}

fn filleted_box() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let body = primitives::make_box(&mut topo, 40.0, 30.0, 20.0).unwrap();
    let edges = solid_edges(&topo, body).unwrap();
    let body = blend_ops::fillet_cascade(&mut topo, body, &edges, 3.0)
        .unwrap()
        .solid;
    (topo, body)
}

fn pocketed_block() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let mut body = primitives::make_box(&mut topo, 80.0, 50.0, 20.0).unwrap();
    for i in 0..3 {
        let pocket = primitives::make_box(&mut topo, 12.0, 10.0, 8.0).unwrap();
        translate(&mut topo, pocket, 8.0 + 20.0 * f64::from(i), 8.0, 12.0);
        body = boolean(&mut topo, BooleanOp::Cut, body, pocket).unwrap();
    }
    let hole = primitives::make_cylinder(&mut topo, 4.0, 40.0).unwrap();
    translate(&mut topo, hole, 30.0, 36.0, -10.0);
    body = boolean(&mut topo, BooleanOp::Cut, body, hole).unwrap();
    (topo, body)
}

#[test]
fn blend_remove_rebuild_moves_match_the_unprepared_move() {
    let (topo, body) = filleted_box();
    let top = top_face(&topo, body);
    let distances = [
        2.5,
        -1.0,
        0.75,
        -3.25,
        6.0,
        -0.125,
        1e-3,
        -1e-3,
        0.0,
        f64::NAN,
        -16.5,
        -40.0,
        4.0,
    ];
    let outcomes = assert_prepared_matches(&topo, body, &[top], &distances, false);
    assert!(outcomes.iter().filter(|o| o.starts_with("ok")).count() >= 8);
    assert!(outcomes.iter().any(|o| o.starts_with("refused")));
}

#[test]
fn planar_moves_match_the_unprepared_move() {
    let (topo, body) = pocketed_block();
    let top = top_face(&topo, body);
    let distances = [
        1.0,
        -0.5,
        3.75,
        -2.0,
        0.0,
        f64::INFINITY,
        -7.5,
        -8.0,
        -12.0,
        0.25,
        12.0,
    ];
    let outcomes = assert_prepared_matches(&topo, body, &[top], &distances, true);
    assert!(outcomes.iter().filter(|o| o.starts_with("ok")).count() >= 5);
    assert!(outcomes.iter().any(|o| o.starts_with("refused")));
}

#[test]
fn bore_and_invalid_selections_keep_the_unprepared_move() {
    let (topo, body) = pocketed_block();
    let bore = solid_faces(&topo, body)
        .unwrap()
        .into_iter()
        .find(|&f| topo.face(f).unwrap().surface().type_tag() == "cylinder")
        .unwrap();
    let top = top_face(&topo, body);
    for faces in [vec![bore], vec![], vec![top, top], vec![top, bore]] {
        let prepared = PreparedFaceMove::prepare(&topo, body, &faces).unwrap();
        assert_eq!(
            prepared.planar.is_some(),
            faces.first() == Some(&top),
            "{faces:?}"
        );
        assert_prepared_matches(&topo, body, &faces, &[0.5, -0.5, 2.0], true);
    }
}

#[test]
fn journaled_moves_match_and_a_changed_source_is_refused() {
    let (mut topo, body) = filleted_box();
    let top = top_face(&topo, body);
    let checkpoint = topo.clone();
    let mut plain = topo.clone();
    let mut prepared = PreparedFaceMove::prepare(&topo, body, &[top]).unwrap();
    for distance in [1.5, -2.0, 0.0, 3.0] {
        topo.restore_preserving_handle_slots(&checkpoint);
        plain.restore_preserving_handle_slots(&checkpoint);
        let expected = crate::journal_ops::move_faces_journaled(&mut plain, body, &[top], distance)
            .map(|op| {
                let entry = plain.journal().entries().last().unwrap();
                (
                    signature(&plain, op.solid),
                    signature_history(&plain, &op.map),
                    format!("{:?}", entry.kind()),
                    plain.journal().entries().len(),
                )
            })
            .map_err(|error| error.to_string());
        let actual = prepared
            .move_faces_journaled(&mut topo, distance)
            .map(|op| {
                let entry = topo.journal().entries().last().unwrap();
                (
                    signature(&topo, op.solid),
                    signature_history(&topo, &op.map),
                    format!("{:?}", entry.kind()),
                    topo.journal().entries().len(),
                )
            })
            .map_err(|error| error.to_string());
        assert_eq!(actual, expected, "distance {distance}");
    }

    // An edit of the source after preparation makes the preparation stale,
    // and restoring the prepared state makes it current again.
    topo.restore_preserving_handle_slots(&checkpoint);
    translate(&mut topo, body, 0.0, 0.0, 1.0);
    assert!(!prepared.is_current(&topo));
    let stale = prepared.move_faces_journaled(&mut topo, 1.0).unwrap_err();
    assert!(stale.to_string().contains("stale"), "{stale}");
    let after_refusal = canonical(&topo, body);
    assert!(prepared.move_faces(&mut topo, 1.0).is_err());
    assert_eq!(canonical(&topo, body), after_refusal);
    topo.restore_preserving_handle_slots(&checkpoint);
    assert!(prepared.is_current(&topo));
    assert!(prepared.move_faces(&mut topo, 1.0).is_ok());
}
