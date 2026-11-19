#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::curves::Circle3D;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use super::{Chord, crossing_splits, face_chords, refine_planar_crossings};
use crate::tessellate::boundary_plan::{SampleChains, SampleParams};

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

fn p(x: f64, y: f64) -> Point3 {
    Point3::new(x, y, 0.0)
}

/// A planar fixture: edges with their sample chains, and faces over them.
#[derive(Default)]
struct Fixture {
    topo: Topology,
    points: SampleChains,
    params: SampleParams,
}

impl Fixture {
    fn line(&mut self, a: Point3, b: Point3) -> EdgeId {
        let va = self.topo.add_vertex(Vertex::new(a, 1e-7));
        let vb = self.topo.add_vertex(Vertex::new(b, 1e-7));
        let edge = self.topo.add_edge(Edge::new(va, vb, EdgeCurve::Line));
        self.points.insert(edge.index(), vec![a, b]);
        self.params
            .insert(edge.index(), vec![Some(0.0), Some((b - a).length())]);
        edge
    }

    /// The arc of `circle` from angle `t0` to `t1`, sampled at its ends.
    fn arc(&mut self, circle: &Circle3D, t0: f64, t1: f64) -> EdgeId {
        let (a, b) = (circle.evaluate(t0), circle.evaluate(t1));
        let va = self.topo.add_vertex(Vertex::new(a, 1e-7));
        let vb = self.topo.add_vertex(Vertex::new(b, 1e-7));
        let edge = self
            .topo
            .add_edge(Edge::new(va, vb, EdgeCurve::Circle(circle.clone())));
        self.points.insert(edge.index(), vec![a, b]);
        self.params.insert(edge.index(), vec![Some(t0), Some(t1)]);
        edge
    }

    fn face(&mut self, edges: &[(EdgeId, bool)]) -> FaceId {
        let oriented = edges
            .iter()
            .map(|&(edge, forward)| OrientedEdge::new(edge, forward))
            .collect();
        let wire = self.topo.add_wire(Wire::new(oriented, true).unwrap());
        self.topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane { normal: Z, d: 0.0 },
        ))
    }

    /// The circle an arc edge lies on, and the edge's index.
    fn circle_of(&self, edge: EdgeId) -> (Circle3D, usize) {
        let EdgeCurve::Circle(circle) = self.topo.edge(edge).unwrap().curve() else {
            panic!("not an arc");
        };
        (circle.clone(), edge.index())
    }

    /// The edge indices of `face`'s outer wire.
    fn edges_of(&self, face: FaceId) -> Vec<usize> {
        let wire = self.topo.face(face).unwrap().outer_wire();
        self.topo
            .wire(wire)
            .unwrap()
            .edges()
            .iter()
            .map(|oriented| oriented.edge().index())
            .collect()
    }

    /// Properly crossing chord pairs of `face`'s sampled outline, straight
    /// pairs included.
    fn crossings(&self, face: FaceId) -> usize {
        let chords = face_chords(&self.topo, face, &self.points, &self.params).unwrap();
        let mut count = 0;
        for i in 0..chords.len() {
            for j in i + 1..chords.len() {
                count += usize::from(super::properly_cross(&chords[i], &chords[j]));
            }
        }
        count
    }
}

/// A planar face over one closed curved edge whose chain zigzags across
/// `x ∈ [0, 100]` while climbing in `y`: `count` chords that all overlap in
/// `x`, each disjoint in `y` from all but its neighbours, and none crossing.
fn zigzag_cap(fx: &mut Fixture, count: usize) -> FaceId {
    let circle = Circle3D::new_with_ref(p(0.0, 0.0), Z, 1.0, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let vertex = fx.topo.add_vertex(Vertex::new(p(0.0, 0.0), 1e-7));
    let edge = fx
        .topo
        .add_edge(Edge::new(vertex, vertex, EdgeCurve::Circle(circle)));
    #[allow(clippy::cast_precision_loss)]
    let points: Vec<Point3> = (0..=count)
        .map(|k| p(if k % 2 == 0 { 0.0 } else { 100.0 }, 0.01 * k as f64))
        .collect();
    #[allow(clippy::cast_precision_loss)]
    let params = (0..=count).map(|k| Some(k as f64)).collect();
    fx.points.insert(edge.index(), points);
    fx.params.insert(edge.index(), params);
    fx.face(&[(edge, true)])
}

/// Review finding 3 of #1868: a face whose outline was simple when first
/// read must be read again once a neighbour's split changes an edge they
/// share. Face A's straight trim crosses the shared arc's chord and splits
/// it at the arc's middle (0, 0.08); face B's straight side y = 0.02 cleared
/// the original chord but crosses both new halves.
#[test]
fn a_neighbour_split_rechecks_the_faces_sharing_the_edge() {
    let mut fx = Fixture::default();
    let (face_a, face_b, arc) = neighbour_pair(&mut fx);
    assert_eq!(fx.crossings(face_a), 1);
    assert_eq!(fx.crossings(face_b), 0);
    let (circle, _) = fx.circle_of(arc);

    let Fixture {
        topo,
        mut points,
        mut params,
    } = fx;
    let inserted =
        refine_planar_crossings(&topo, &[face_b, face_a], &mut points, &mut params).unwrap();
    assert!(inserted > 0);
    let fx = Fixture {
        topo,
        points,
        params,
    };
    assert_eq!(fx.crossings(face_a), 0, "face A");
    assert_eq!(fx.crossings(face_b), 0, "face B");
    // Every inserted sample lies on the arc.
    for point in &fx.points[&arc.index()] {
        assert!(((*point - circle.center()).length() - circle.radius()).abs() < 1e-12);
    }
}

/// Re-check of #1868 (finding 2): a face that exhausts its work bounds
/// reverts only the faces it shares split edges with. Here the neighbour
/// pair above clears with one split, and an unrelated cap whose 12 000
/// curved chords all overlap in `x` (disjoint in `y`) spends the crossing
/// search's visit bound; the pair still comes out simple, the cap keeps its
/// chain as sampled.
#[test]
fn a_capped_face_reverts_only_its_own_group() {
    let mut fx = Fixture::default();
    let (face_a, face_b, _) = neighbour_pair(&mut fx);
    let cap = zigzag_cap(&mut fx, 12_000);
    let cap_chain = fx.points[&fx.edges_of(cap)[0]].clone();
    let Fixture {
        topo,
        mut points,
        mut params,
    } = fx;
    let inserted =
        refine_planar_crossings(&topo, &[cap, face_b, face_a], &mut points, &mut params).unwrap();
    assert!(inserted > 0);
    let fx = Fixture {
        topo,
        points,
        params,
    };
    assert_eq!(fx.crossings(face_a), 0, "face A");
    assert_eq!(fx.crossings(face_b), 0, "face B");
    assert_eq!(fx.points[&fx.edges_of(cap)[0]], cap_chain, "the cap");
}

/// Re-check of #1868 (finding 3): the visit bound counts every active-list
/// entry the search reads, the scan that prunes the list and the scan of
/// the survivors. 10 000 curved chords that all overlap in `x` read
/// 99 990 000 entries; the old count booked half of them (49 995 000, under
/// the 64M bound) and searched on.
#[test]
fn the_visit_bound_counts_both_scans() {
    let chords: Vec<Chord> = (0..10_000)
        .map(|k| {
            #[allow(clippy::cast_precision_loss)]
            let y = 0.01 * k as f64;
            let (a, b) = if k % 2 == 0 {
                (0.0, 100.0)
            } else {
                (100.0, 0.0)
            };
            chord((a, y), (b, y + 0.01), true, 1)
        })
        .collect();
    assert!(crossing_splits(&chords).is_none());
}

fn neighbour_pair(fx: &mut Fixture) -> (FaceId, FaceId, EdgeId) {
    // The circle through (±1, 0) and (0, 0.08).
    let radius = (1.0 + 0.08 * 0.08) / (2.0 * 0.08);
    let centre = p(0.0, 0.08 - radius);
    let circle = Circle3D::new_with_ref(centre, Z, radius, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let t = (-centre.y()).atan2(1.0);
    // The shared arc, from (-1, 0) over the top to (1, 0).
    let arc = fx.arc(&circle, std::f64::consts::PI - t, t);
    assert!((fx.points[&arc.index()][0] - p(-1.0, 0.0)).length() < 1e-12);
    let (start, end) = (fx.points[&arc.index()][0], fx.points[&arc.index()][1]);
    let a1 = fx.line(end, p(0.0, 0.05));
    let a2 = fx.line(p(0.0, 0.05), p(0.0, -0.05));
    let a3 = fx.line(p(0.0, -0.05), start);
    let face_a = fx.face(&[(arc, true), (a1, true), (a2, true), (a3, true)]);
    let b1 = fx.line(start, p(-0.8, 0.02));
    let b2 = fx.line(p(-0.8, 0.02), p(0.8, 0.02));
    let b3 = fx.line(p(0.8, 0.02), end);
    // B runs the shared arc the other way round. It is listed first, so a
    // single pass reads it before A splits the arc.
    let face_b = fx.face(&[(arc, false), (b1, true), (b2, true), (b3, true)]);
    (face_a, face_b, arc)
}

/// When the crossings cannot be cleared within the bounds, the chains are
/// left exactly as they were sampled: every face keeps its old outline.
#[test]
fn an_unclearable_crossing_leaves_the_chains_as_sampled() {
    // A line that crosses the true arc, not just its chord: no bisection of
    // the arc ever clears it.
    let circle = Circle3D::new_with_ref(p(0.0, -1.0), Z, 2.0, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let mut fx = Fixture::default();
    let t = (1.0f64).atan2(3f64.sqrt());
    let arc = fx.arc(&circle, std::f64::consts::PI - t, t);
    let (start, end) = (fx.points[&arc.index()][0], fx.points[&arc.index()][1]);
    let a1 = fx.line(end, p(3.0, 3.0));
    let a2 = fx.line(p(3.0, 3.0), p(0.3, 3.0));
    let a3 = fx.line(p(0.3, 3.0), p(0.3, -0.5));
    let a4 = fx.line(p(0.3, -0.5), start);
    let face = fx.face(&[(arc, true), (a1, true), (a2, true), (a3, true), (a4, true)]);
    assert_eq!(fx.crossings(face), 1);
    let before = (fx.points.clone(), fx.params.clone());
    let Fixture {
        topo,
        mut points,
        mut params,
    } = fx;
    let inserted = refine_planar_crossings(&topo, &[face], &mut points, &mut params).unwrap();
    assert_eq!(inserted, 0);
    assert_eq!(points, before.0);
    assert_eq!(params, before.1);
}

fn chord(a: (f64, f64), b: (f64, f64), curved: bool, edge: usize) -> Chord {
    Chord {
        a: Point2::new(a.0, a.1),
        b: Point2::new(b.0, b.1),
        edge,
        at: 0,
        curved,
    }
}

/// Review finding 4 of #1868: straight chords that overlap in x (1 500
/// parallel slots, 3 000 long sides) must not spend the pair budget before
/// a curved chord's crossing is found.
#[test]
fn straight_pairs_do_not_spend_the_crossing_budget() {
    let mut chords = Vec::new();
    for slot in 0..1500 {
        #[allow(clippy::cast_precision_loss)]
        let y = 10.0 + slot as f64;
        chords.push(chord((0.0, y), (100.0, y), false, 1));
        chords.push(chord((0.0, y + 0.5), (100.0, y + 0.5), false, 1));
    }
    // A crescent tip: a curved chord crossed by a straight one.
    chords.push(chord((40.0, 0.0), (60.0, 0.0), true, 2));
    chords.push(chord((50.0, -1.0), (50.0, 1.0), false, 3));
    let split = crossing_splits(&chords).expect("within the work bounds");
    assert_eq!(split, vec![3000]);
}
