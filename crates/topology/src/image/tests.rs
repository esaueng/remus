#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::vec::{Point2, Point3, Vec2, Vec3};

use super::*;
use crate::attributes::ColorRgb;
use crate::edge::EdgeId;
use crate::face::FaceId;
use crate::journal::{EventDraft, EvolutionDraft};
use crate::shell::Shell;
use crate::solid::{Solid, SolidId};
use crate::wire::OrientedEdge;

const TOL: f64 = 1e-7;

fn square_face(topo: &mut Topology, z: f64) -> (FaceId, Vec<EdgeId>) {
    let corners = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
    let vertices: Vec<_> = corners
        .iter()
        .map(|&(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, z), TOL)))
        .collect();
    let edges: Vec<_> = (0..4)
        .map(|i| {
            topo.add_edge(Edge::new(
                vertices[i],
                vertices[(i + 1) % 4],
                EdgeCurve::Line,
            ))
        })
        .collect();
    let wire = Wire::new(
        edges.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
        true,
    )
    .unwrap();
    let wire = topo.add_wire(wire);
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: z,
        },
    ));
    (face, edges)
}

fn plate(topo: &mut Topology, z: f64) -> (SolidId, FaceId) {
    let (face, edges) = square_face(topo, z);
    let line = Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap();
    topo.set_pcurve(edges[0], face, PCurve::new(Curve2D::Line(line), 0.0, 1.0))
        .unwrap();
    let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
    (topo.add_solid(Solid::new(shell, vec![])), face)
}

/// A session with history: journaled entries, attributes, pcurves, a
/// checkpoint restore that leaves tombstones, and an unjournaled mutation.
fn session() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let (first, face) = plate(&mut topo, 0.0);
    let pending = topo.journal_begin("make_plate");
    let mut draft = EvolutionDraft::construction();
    draft.push(
        EntityKey::face(face.index()),
        EventDraft::Generated { sources: vec![] },
    );
    topo.journal_record_evolution(pending, draft).unwrap();
    topo.set_face_attributes(
        face,
        EntityAttributes {
            name: Some("top".into()),
            color: Some(ColorRgb::new(0.25, 0.5, 1.0).unwrap()),
        },
    )
    .unwrap();
    topo.set_solid_attributes(
        first,
        EntityAttributes {
            name: Some("plate".into()),
            color: None,
        },
    )
    .unwrap();

    let checkpoint = topo.clone();
    let _ = plate(&mut topo, 5.0);
    topo.restore_preserving_handle_slots(&checkpoint);
    let _ = plate(&mut topo, 7.0);
    let pending = topo.journal_begin("barrier");
    topo.journal_record_barrier(pending, vec![EntityKey::face(face.index())]);
    // An unjournaled mutation: the next `journal_begin` must see the gap.
    topo.add_vertex(Vertex::new(Point3::new(9.0, 9.0, 9.0), TOL));
    (topo, first)
}

fn rebuilt(topo: &Topology) -> Topology {
    Topology::from_image(topo.to_image()).unwrap()
}

#[test]
fn round_trip_is_exact_and_allocates_the_same_next_handles() {
    let (topo, solid) = session();
    let copy = rebuilt(&topo);
    assert_eq!(
        format!("{:?}", copy.to_image()),
        format!("{:?}", topo.to_image())
    );
    assert_eq!(copy.mutation_ticks(), topo.mutation_ticks());
    assert_eq!(copy.journal().snapshot(), topo.journal().snapshot());
    assert!(copy.solid(solid).is_ok());

    // Tombstones stay stale and the next allocation in every arena matches.
    let image = topo.to_image();
    let stale = image.vertices.live.iter().position(|live| !live).unwrap();
    assert_eq!(copy.vertices().slot_len(), topo.vertices().slot_len());
    assert!(topo.vertex_id_from_index(stale).is_none());
    assert!(copy.vertex_id_from_index(stale).is_none());
    let (mut a, mut b) = (topo, copy);
    let (sa, fa) = plate(&mut a, 2.0);
    let (sb, fb) = plate(&mut b, 2.0);
    assert_eq!(sa, sb);
    assert_eq!(fa, fb);

    // Journal continuity: both see the same unjournaled gap.
    let pa = a.journal_begin("next");
    let pb = b.journal_begin("next");
    a.journal_record_barrier(pa, vec![]);
    b.journal_record_barrier(pb, vec![]);
    assert_eq!(a.journal().snapshot(), b.journal().snapshot());
    assert_eq!(format!("{:?}", a.to_image()), format!("{:?}", b.to_image()));
}

#[test]
fn a_rebuilt_topology_has_a_fresh_cache_lineage() {
    let (topo, _) = session();
    let copy = rebuilt(&topo);
    assert_ne!(copy.cache_lineage(), topo.cache_lineage());
    assert_ne!(copy.cache_lineage(), 0);
    assert!(!copy.is_cache_poisoned());
}

fn refused(image: TopologyImage, needle: &str) {
    let error = Topology::from_image(image).unwrap_err();
    assert!(
        error.reason.contains(needle),
        "expected {needle:?} in {error}"
    );
}

#[test]
fn out_of_range_and_dangling_references_are_refused() {
    let (topo, _) = session();
    let mut image = topo.to_image();
    image.edges.items[0].start = 10_000;
    refused(image, "beyond the arena");

    let mut image = topo.to_image();
    let retired = image.vertices.live.iter().position(|live| !live).unwrap();
    image.edges.items[0].end = retired;
    refused(image, "references retired vertex");

    let mut image = topo.to_image();
    image.loops.live[0] = false;
    refused(image, "references retired loop");

    let mut image = topo.to_image();
    image.vertices.live.pop();
    refused(image, "liveness flags");
}

#[test]
fn ownership_and_index_inconsistencies_are_refused() {
    let (topo, _) = session();
    let mut image = topo.to_image();
    let other = image.faces.live.iter().rposition(|live| *live).unwrap();
    image.loops.items[0].face = other;
    refused(image, "is not listed by its face");

    let mut image = topo.to_image();
    let coedge = image.loops.items[0].coedges[0];
    image.loops.items[0].coedges.push(coedge);
    refused(image, "listed by more than one live loop");

    let mut image = topo.to_image();
    image.pcurve_uses.reverse();
    refused(image, "strictly increasing");

    let mut image = topo.to_image();
    image.pcurve_uses[0].forward = !image.pcurve_uses[0].forward;
    refused(image, "another edge use");

    let mut image = topo.to_image();
    image.journal_ticks.pop();
    refused(image, "tick counts");

    let mut image = topo.to_image();
    image.journal.index[0].1.index = 1_000_000;
    refused(image, "journal ordinal");

    let mut image = topo.to_image();
    image.face_attributes[0].1 = EntityAttributes::default();
    refused(image, "is empty");

    let mut image = topo.to_image();
    image.wires.items[0].edges.clear();
    refused(image, "has no edges");
}
