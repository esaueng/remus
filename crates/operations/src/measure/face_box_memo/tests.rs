//! The memo's answers against the unmemoized box, bit for bit, over analytic,
//! filleted, drilled and NURBS faces, copies and edits.

#![allow(clippy::unwrap_used, clippy::panic, clippy::cast_precision_loss)]

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::make_nurbs_face;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use super::*;
use crate::boolean::{BooleanOp, boolean};
use crate::measure::face_set_bounding_box;

struct ResetOnDrop;

impl Drop for ResetOnDrop {
    fn drop(&mut self) {
        disable_thread_face_box_memo();
    }
}

fn fresh_memo() -> ResetOnDrop {
    disable_thread_face_box_memo();
    enable_thread_face_box_memo();
    ResetOnDrop
}

fn bits(aabb: Aabb3) -> [u64; 6] {
    [
        aabb.min.x().to_bits(),
        aabb.min.y().to_bits(),
        aabb.min.z().to_bits(),
        aabb.max.x().to_bits(),
        aabb.max.y().to_bits(),
        aabb.max.z().to_bits(),
    ]
}

/// The memoized box of `face` against the plain one, bit for bit.
fn assert_box_bits(topo: &Topology, face: FaceId) {
    let expected = face_set_bounding_box(topo, &[face]).unwrap();
    let memoized = face_bounding_box(topo, face).unwrap();
    assert_eq!(bits(memoized), bits(expected), "face {}", face.index());
}

fn translate(topo: &mut Topology, solid: SolidId, x: f64, y: f64, z: f64) {
    crate::transform::transform_solid(topo, solid, &remus_math::mat::Mat4::translation(x, y, z))
        .unwrap();
}

/// Analytic primitives, a filleted box (cylinder, torus and sphere blends),
/// a drilled plate (holed planes, partial cylinders) and a rotated cone.
fn corpus() -> (Topology, Vec<SolidId>) {
    let mut topo = Topology::new();
    let mut solids = Vec::new();
    solids.push(crate::primitives::make_box(&mut topo, 3.0, 2.0, 1.0).unwrap());
    solids.push(crate::primitives::make_cylinder(&mut topo, 1.5, 4.0).unwrap());
    let cone = crate::primitives::make_cone(&mut topo, 2.0, 0.5, 3.0).unwrap();
    crate::transform::transform_solid(&mut topo, cone, &remus_math::mat::Mat4::rotation_x(0.7))
        .unwrap();
    solids.push(cone);
    solids.push(crate::primitives::make_sphere(&mut topo, 2.5, 16).unwrap());
    solids.push(crate::primitives::make_torus(&mut topo, 4.0, 1.0, 16).unwrap());
    let sharp = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, sharp).unwrap();
    solids.push(
        crate::blend_ops::fillet_v2(&mut topo, sharp, &edges, 1.0)
            .unwrap()
            .solid,
    );
    let mut plate = crate::primitives::make_box(&mut topo, 30.0, 20.0, 5.0).unwrap();
    for (x, y) in [(8.0, 8.0), (20.0, 12.0)] {
        let tool = crate::primitives::make_cylinder(&mut topo, 3.0, 20.0).unwrap();
        translate(&mut topo, tool, x, y, -5.0);
        plate = boolean(&mut topo, BooleanOp::Cut, plate, tool).unwrap();
    }
    solids.push(plate);
    (topo, solids)
}

fn nurbs_patch(lift: f64) -> NurbsSurface {
    let points: Vec<Vec<Point3>> = (0..4)
        .map(|i| {
            (0..3)
                .map(|j| {
                    let z = if (i == 1 || i == 2) && j == 1 {
                        lift
                    } else {
                        0.0
                    };
                    Point3::new(i as f64, j as f64, z)
                })
                .collect()
        })
        .collect();
    let mut weights = vec![vec![1.0; 3]; 4];
    weights[1][1] = 1.125;
    NurbsSurface::new(
        2,
        2,
        vec![0.0, 0.0, 0.0, 0.4, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        points,
        weights,
    )
    .unwrap()
}

#[test]
fn hits_and_misses_return_the_unmemoized_box_bits() {
    let _reset = fresh_memo();
    let (topo, solids) = corpus();
    let mut faces = Vec::new();
    for &solid in &solids {
        faces.extend(solid_faces(&topo, solid).unwrap());
    }
    let mut nurbs_topo = Topology::new();
    let nurbs = make_nurbs_face(&mut nurbs_topo, nurbs_patch(0.75), 1e-7).unwrap();

    for _ in 0..2 {
        for &face in &faces {
            assert_box_bits(&topo, face);
        }
        assert_box_bits(&nurbs_topo, nurbs);
    }
    let stats = thread_face_box_memo_stats().unwrap();
    assert!(faces.len() > 40, "corpus has {} faces", faces.len());
    // Faces of one primitive can share content (a box's opposite sides do
    // not; a sphere's segments can), so the first pass may hit already.
    assert_eq!(stats.hits + stats.misses, 2 * (faces.len() as u64 + 1));
    assert!(stats.hits > faces.len() as u64);
    assert_eq!(stats.len as u64, stats.misses);
}

#[test]
fn copies_hit_and_edits_miss() {
    let _reset = fresh_memo();
    let (mut topo, solids) = corpus();
    let source = solids[5];
    // Off the origin, so no axis holds both signed zeros and the copy's
    // handle order cannot matter.
    translate(&mut topo, source, 0.5, 0.5, 0.5);
    for face in solid_faces(&topo, source).unwrap() {
        assert_box_bits(&topo, face);
    }
    let first = thread_face_box_memo_stats().unwrap();

    // A copy has new handles and the same content: every face hits.
    let copy = crate::copy::copy_solid(&mut topo, source).unwrap();
    let copied = solid_faces(&topo, copy).unwrap();
    for &face in &copied {
        assert_box_bits(&topo, face);
    }
    let after_copy = thread_face_box_memo_stats().unwrap();
    assert_eq!(after_copy.misses, first.misses);
    assert_eq!(after_copy.hits, first.hits + copied.len() as u64);

    // Moving the copy changes every face's content: every face misses and
    // still reads its own box.
    translate(&mut topo, copy, 0.25, -0.5, 1.0);
    for &face in &copied {
        assert_box_bits(&topo, face);
    }
    let after_move = thread_face_box_memo_stats().unwrap();
    assert_eq!(after_move.hits, after_copy.hits);
    assert_eq!(after_move.misses, after_copy.misses + copied.len() as u64);

    // An edit of one surface field misses too.
    let mut nurbs_topo = Topology::new();
    let face = make_nurbs_face(&mut nurbs_topo, nurbs_patch(0.75), 1e-7).unwrap();
    assert_box_bits(&nurbs_topo, face);
    nurbs_topo
        .face_mut(face)
        .unwrap()
        .set_surface(FaceSurface::Nurbs(nurbs_patch(1.5)));
    let before = thread_face_box_memo_stats().unwrap();
    assert_box_bits(&nurbs_topo, face);
    assert_eq!(
        thread_face_box_memo_stats().unwrap().misses,
        before.misses + 1
    );
}

/// A quadrilateral whose two lowest-x vertices are `-0.0` and `+0.0`, joined
/// only through vertices off that line (an edge between them would sample
/// a zero of its own): the box keeps the first of the two zeros in ascending
/// handle order, so the same boundary with those vertices allocated the
/// other way round boxes to the other zero and must not share an entry.
fn signed_zero_quad(negative_first: bool) -> (Topology, FaceId) {
    let mut topo = Topology::new();
    let negative = Point3::new(-0.0, 0.0, 0.0);
    let positive = Point3::new(0.0, 2.0, 0.0);
    let (a, b) = if negative_first {
        let a = topo.add_vertex(Vertex::new(negative, 1e-7));
        (a, topo.add_vertex(Vertex::new(positive, 1e-7)))
    } else {
        let b = topo.add_vertex(Vertex::new(positive, 1e-7));
        (topo.add_vertex(Vertex::new(negative, 1e-7)), b)
    };
    let c = topo.add_vertex(Vertex::new(Point3::new(1.0, 1.0, 0.0), 1e-7));
    let d = topo.add_vertex(Vertex::new(Point3::new(2.0, 1.0, 0.0), 1e-7));
    let mut uses = Vec::new();
    for (from, to) in [(a, c), (c, b), (b, d), (d, a)] {
        uses.push(OrientedEdge::new(
            topo.add_edge(Edge::new(from, to, EdgeCurve::Line)),
            true,
        ));
    }
    let wire = topo.add_wire(Wire::new(uses, true).unwrap());
    let face = topo.add_face(Face::new(
        wire,
        Vec::new(),
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    (topo, face)
}

#[test]
fn handle_order_of_signed_zeros_is_part_of_the_key() {
    let _reset = fresh_memo();
    let (negative_first, a) = signed_zero_quad(true);
    let (positive_first, b) = signed_zero_quad(false);
    let boxed = |topo: &Topology, face| face_set_bounding_box(topo, &[face]).unwrap();
    assert_ne!(
        bits(boxed(&negative_first, a)),
        bits(boxed(&positive_first, b)),
        "the fixture exercises the order dependence"
    );
    assert_box_bits(&negative_first, a);
    assert_box_bits(&positive_first, b);
    assert_eq!(thread_face_box_memo_stats().unwrap().misses, 2);
}

#[test]
fn bounds_disable_and_evict() {
    let _reset = fresh_memo();
    let (topo, solids) = corpus();
    let faces = solid_faces(&topo, solids[6]).unwrap();
    set_thread_face_box_memo_limits(3, DEFAULT_FACE_BOX_MEMO_BYTE_BUDGET);
    for &face in &faces {
        assert_box_bits(&topo, face);
    }
    let stats = thread_face_box_memo_stats().unwrap();
    assert_eq!(stats.len, 3);
    assert_eq!(stats.evictions, stats.misses - 3);
    // The three newest still hit after the evictions.
    for &face in &faces[faces.len() - 3..] {
        assert_box_bits(&topo, face);
    }
    assert_eq!(thread_face_box_memo_stats().unwrap().hits, stats.hits + 3);

    set_thread_face_box_memo_limits(0, DEFAULT_FACE_BOX_MEMO_BYTE_BUDGET);
    let off = thread_face_box_memo_stats().unwrap();
    assert_eq!(off.len, 0);
    assert_box_bits(&topo, faces[0]);
    assert_eq!(thread_face_box_memo_stats().unwrap(), off);

    clear_thread_face_box_memo();
    disable_thread_face_box_memo();
    assert!(thread_face_box_memo_stats().is_none());
    assert_box_bits(&topo, faces[0]);
}

/// A disc bounded by one circle edge whose range runs one ULP past a full
/// turn, as an anchored full turn can. With one vertex the edge is closed and
/// the preflight accepts the turn; with two distinct vertices at the same
/// point it is open, its range overruns a turn and the preflight refuses
/// it. Every other key word is the same for both.
fn full_turn_disc(closed: bool) -> (Topology, FaceId) {
    use remus_math::curves::Circle3D;

    let mut topo = Topology::new();
    let circle = Circle3D::new(Point3::new(3.0, 2.0, 1.0), Vec3::new(0.0, 0.0, 1.0), 1.5).unwrap();
    let range = (0.0, std::f64::consts::TAU.next_up());
    let point = circle.evaluate(range.0);
    let start = topo.add_vertex(Vertex::new(point, 1e-7));
    let end = if closed {
        start
    } else {
        topo.add_vertex(Vertex::new(point, 1e-7))
    };
    let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle));
    edge.set_trim(Some(range));
    let edge = topo.add_edge(edge);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let face = topo.add_face(Face::new(
        wire,
        Vec::new(),
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 1.0,
        },
    ));
    (topo, face)
}

#[test]
fn closed_edge_flag_is_part_of_the_key() {
    let _reset = fresh_memo();
    let (closed_topo, closed) = full_turn_disc(true);
    let (open_topo, open) = full_turn_disc(false);
    assert!(face_set_bounding_box(&closed_topo, &[closed]).is_ok());
    let refusal = face_set_bounding_box(&open_topo, &[open])
        .unwrap_err()
        .to_string();
    assert!(
        refusal.contains("requires an authoritative edge domain"),
        "{refusal}"
    );
    // The closed disc's box is stored first; the open disc must not read it.
    assert_box_bits(&closed_topo, closed);
    assert_eq!(
        face_bounding_box(&open_topo, open).unwrap_err().to_string(),
        refusal
    );
    let stats = thread_face_box_memo_stats().unwrap();
    assert_eq!((stats.hits, stats.misses, stats.len), (0, 2, 1));
}

/// Pins every geometry type's size, so a new field fails here until the key
/// reads it.
#[test]
fn key_reads_every_geometry_field() {
    use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
    use remus_math::nurbs::curve::NurbsCurve;
    use remus_math::surfaces::{
        ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface,
    };
    use std::mem::size_of;
    const F: usize = size_of::<f64>();
    const LOCK_F64: usize = size_of::<std::sync::OnceLock<f64>>();
    const LOCK_BOOL: usize = size_of::<std::sync::OnceLock<bool>>();
    const LOCK_GRID: usize = size_of::<
        std::sync::OnceLock<std::sync::Arc<remus_math::nurbs::projection::SurfaceSeedGrid>>,
    >();
    assert_eq!(size_of::<CylindricalSurface>(), 13 * F);
    assert_eq!(size_of::<ConicalSurface>(), 13 * F);
    assert_eq!(size_of::<SphericalSurface>(), 13 * F);
    assert_eq!(size_of::<ToroidalSurface>(), 14 * F);
    assert_eq!(size_of::<Circle3D>(), 13 * F);
    assert_eq!(size_of::<Ellipse3D>(), 14 * F);
    assert_eq!(size_of::<Hyperbola3D>(), 14 * F);
    assert_eq!(size_of::<Parabola3D>(), 10 * F);
    // Degrees, knots, control points and weights, plus caches derived from
    // them.
    assert_eq!(
        size_of::<NurbsSurface>(),
        2 * F + 4 * size_of::<std::sync::Arc<Vec<f64>>>() + LOCK_F64 + LOCK_BOOL + LOCK_GRID
    );
    assert_eq!(
        size_of::<NurbsCurve>(),
        F + 3 * size_of::<std::sync::Arc<Vec<f64>>>() + LOCK_F64
    );
    // Start, end, curve, tolerance, trim.
    assert_eq!(size_of::<Edge>(), 176);
}
