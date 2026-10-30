//! A boolean's curved seams name the face pair that made them.
//!
//! Fusing a round boss into a block makes three kinds of seam on the boss: a
//! straight line where it meets each wall, an arc where it meets the block's
//! top, and, when the block's top edge is rounded, a free-form curve where it
//! meets that round. The face splitter builds the curved ones from the whole
//! intersection curve, so they used to come back `Unresolved` while the lines
//! were `Generated`, and a consumer could name only the straight part of one
//! continuous seam.
//!
//! Every expected pair below is a fact about the construction: the seam lies
//! where the boss wall meets that one block face.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::compact_sort::CompactSort;
use remus_math::vec::Point3;
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, EdgeEvent, boolean_with_entity_evolution};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const BOSS_RADIUS: f64 = 6.0;
const ROUND_RADIUS: f64 = 2.8;
const BLOCK_HEIGHT: f64 = 24.0;

fn edge_between(topo: &Topology, solid: SolidId, a: Point3, b: Point3) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            ((start - a).length() < 1e-6 && (end - b).length() < 1e-6)
                || ((start - b).length() < 1e-6 && (end - a).length() < 1e-6)
        })
        .unwrap_or_else(|| panic!("edge {a:?} -- {b:?}"))
}

fn face_where(topo: &Topology, solid: SolidId, test: impl Fn(&FaceSurface) -> bool) -> FaceId {
    let matches: Vec<FaceId> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| test(topo.face(face).unwrap().surface()))
        .collect();
    assert_eq!(matches.len(), 1, "expected exactly one matching face");
    matches[0]
}

fn plane_at(surface: &FaceSurface, normal: [f64; 3], offset: f64) -> bool {
    match surface {
        FaceSurface::Plane { normal: n, d } => {
            (n.x() - normal[0]).abs() < 1e-9
                && (n.y() - normal[1]).abs() < 1e-9
                && (n.z() - normal[2]).abs() < 1e-9
                && (d - offset).abs() < 1e-9
        }
        _ => false,
    }
}

fn cylinder_of(surface: &FaceSurface, radius: f64) -> bool {
    matches!(surface, FaceSurface::Cylinder(c) if (c.radius() - radius).abs() < 1e-9)
}

/// The face pair a `Generated` seam names, order-free, for every result edge
/// whose ends both lie on the boss wall, from its base to the block's top.
fn boss_seams(
    topo: &Topology,
    result: SolidId,
    events: &[(usize, EdgeEvent)],
) -> Vec<(String, Option<[usize; 2]>)> {
    let mut seams = Vec::new();
    for edge_id in solid_edges(topo, result).unwrap() {
        let edge = topo.edge(edge_id).unwrap();
        let ends = [edge.start(), edge.end()].map(|v| topo.vertex(v).unwrap().point());
        let on_wall = |p: Point3| {
            (p.x().hypot(p.y()) - BOSS_RADIUS).abs() < 1e-6 && p.z() <= BLOCK_HEIGHT + 1e-6
        };
        // The base circle's arcs lie on the boss wall too; a seam climbs it.
        if !ends.iter().all(|&p| on_wall(p)) || ends.iter().all(|p| p.z().abs() < 1e-6) {
            continue;
        }
        let event = events
            .iter()
            .find(|(index, _)| *index == edge_id.index())
            .map(|(_, event)| event)
            .unwrap_or_else(|| panic!("edge {} has no event", edge_id.index()));
        let pair = match event {
            EdgeEvent::Generated {
                face_a: Some(a),
                face_b: Some(b),
            } => {
                let mut pair = [*a, *b];
                pair.compact_sort_unstable();
                Some(pair)
            }
            _ => None,
        };
        let kind = match edge.curve() {
            remus_topology::edge::EdgeCurve::Line => "line",
            remus_topology::edge::EdgeCurve::Circle(_) => "arc",
            remus_topology::edge::EdgeCurve::NurbsCurve(_) => "free-form",
            _ => "other",
        };
        seams.push((kind.to_owned(), pair));
    }
    seams.compact_sort_unstable();
    seams
}

fn sorted(a: FaceId, b: FaceId) -> [usize; 2] {
    let mut pair = [a.index(), b.index()];
    pair.compact_sort_unstable();
    pair
}

#[test]
fn boss_on_block_top_arc_names_its_face_pair() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 30.0, 18.0, BLOCK_HEIGHT).unwrap();
    let boss = make_cylinder(&mut topo, BOSS_RADIUS, 28.0).unwrap();
    let wall = face_where(&topo, boss, |s| cylinder_of(s, BOSS_RADIUS));
    let front = face_where(&topo, block, |s| plane_at(s, [0.0, -1.0, 0.0], 0.0));
    let side = face_where(&topo, block, |s| plane_at(s, [-1.0, 0.0, 0.0], 0.0));
    let top = face_where(&topo, block, |s| plane_at(s, [0.0, 0.0, 1.0], BLOCK_HEIGHT));

    let (result, evolution) =
        boolean_with_entity_evolution(&mut topo, BooleanOp::Fuse, boss, block).unwrap();

    assert_eq!(
        boss_seams(&topo, result, &evolution.edges),
        vec![
            ("arc".to_owned(), Some(sorted(wall, top))),
            ("line".to_owned(), Some(sorted(wall, side))),
            ("line".to_owned(), Some(sorted(wall, front))),
        ]
        .tap_sorted(),
    );
    assert!(
        !evolution
            .edges
            .iter()
            .any(|(_, event)| matches!(event, EdgeEvent::Unresolved)),
        "no result edge of this fuse is unresolved"
    );
}

#[test]
fn seam_over_an_earlier_round_names_every_piece() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 30.0, 18.0, BLOCK_HEIGHT).unwrap();
    let top_front = edge_between(
        &topo,
        block,
        Point3::new(0.0, 0.0, BLOCK_HEIGHT),
        Point3::new(30.0, 0.0, BLOCK_HEIGHT),
    );
    let block = fillet_cascade(&mut topo, block, &[top_front], ROUND_RADIUS)
        .unwrap()
        .solid;
    let boss = make_cylinder(&mut topo, BOSS_RADIUS, 28.0).unwrap();
    let wall = face_where(&topo, boss, |s| cylinder_of(s, BOSS_RADIUS));
    let round = face_where(&topo, block, |s| cylinder_of(s, ROUND_RADIUS));
    let front = face_where(&topo, block, |s| plane_at(s, [0.0, -1.0, 0.0], 0.0));
    let side = face_where(&topo, block, |s| plane_at(s, [-1.0, 0.0, 0.0], 0.0));
    let top = face_where(&topo, block, |s| plane_at(s, [0.0, 0.0, 1.0], BLOCK_HEIGHT));

    let (result, evolution) =
        boolean_with_entity_evolution(&mut topo, BooleanOp::Fuse, boss, block).unwrap();

    assert_eq!(
        boss_seams(&topo, result, &evolution.edges),
        vec![
            ("arc".to_owned(), Some(sorted(wall, top))),
            ("free-form".to_owned(), Some(sorted(wall, round))),
            ("line".to_owned(), Some(sorted(wall, side))),
            ("line".to_owned(), Some(sorted(wall, front))),
        ]
        .tap_sorted(),
    );
}

trait TapSorted {
    fn tap_sorted(self) -> Self;
}

impl<T: Ord> TapSorted for Vec<T> {
    fn tap_sorted(mut self) -> Self {
        self.compact_sort_unstable();
        self
    }
}
