//! A round boss's own periodic seam keeps its lineage through a boolean.
//!
//! A cylinder lateral cut by a ring (a box top, a sphere) is re-emitted as
//! stacked bands, and the band emitters rebuild the cylinder's seam Line
//! between their levels. They used to rebuild it with no construction source,
//! so the kept seam piece came back `Unresolved` from the entity-evolution
//! booleans: the one edge a consumer could not name, which made it drop the
//! whole result's lineage (OpenZCAD on Remus 2026.1.122: a fillet on the boss
//! seam lost its edge when an upstream dimension changed).
//!
//! Every expectation below is a construction fact, not a geometric guess:
//! the seam piece is part of the cylinder's seam edge, and each ring piece
//! lies where the cylinder wall meets one face of the other operand. Each
//! family runs in both operand orders, at two placements (a pure translation
//! and the rotated frame OpenZCAD builds for a +Z axis, which moves the seam
//! meridian), and over a 1e-3/1/1e3 scale sweep. The evolution result is also
//! held to the plain boolean's body: same face and edge counts, surface
//! types and volume.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::compact_sort::CompactSort;
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, EdgeEvent, boolean, boolean_with_entity_evolution};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder, make_sphere};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];
/// The sphere families stop at 10x: from 100x up the exact sphere ×
/// cylinder section fails to converge in the pave filler, before the face
/// splitter runs, and the plain boolean refuses the same bodies
/// (`ExactOnlyUnattainable`). That is a section-solver limit, not a lineage
/// one, so it is outside what this file pins.
const SPHERE_SCALES: [f64; 3] = [1e-3, 1.0, 10.0];
const SPHERE_SEGMENTS: usize = 32;

#[derive(Clone, Copy, Debug)]
enum Family {
    /// Box 60×60×20 fused with a r6 h30 cylinder standing on (30, 30, 10).
    BoxBoss,
    /// Sphere r73.5 at (25.5, −1.5, 0) fused with a r6 h64.1 cylinder from
    /// (0, 0, 58): the boss leaves the sphere off its pole.
    SphereBoss,
    /// The same sphere minus the same cylinder: a blind bore.
    SphereBore,
}

#[derive(Clone, Copy, Debug)]
enum Placement {
    /// The cylinder is only translated.
    Translated,
    /// The cylinder is placed by OpenZCAD's +Z coordinate frame, a −90° turn
    /// about Z before the translation.
    Framed,
}

impl Family {
    const fn op(self) -> BooleanOp {
        match self {
            Self::BoxBoss | Self::SphereBoss => BooleanOp::Fuse,
            Self::SphereBore => BooleanOp::Cut,
        }
    }
}

/// The two operands, fresh in their own arena, plus what the expectations
/// need: the cylinder solid, the other operand, and the boss axis.
struct Operands {
    topo: Topology,
    cylinder: SolidId,
    other: SolidId,
    axis_xy: (f64, f64),
    radius: f64,
}

fn build(family: Family, placement: Placement, s: f64) -> Operands {
    let mut topo = Topology::new();
    let (other, radius, height, base) = match family {
        Family::BoxBoss => (
            make_box(&mut topo, 60.0 * s, 60.0 * s, 20.0 * s).unwrap(),
            6.0 * s,
            30.0 * s,
            [30.0 * s, 30.0 * s, 10.0 * s],
        ),
        Family::SphereBoss | Family::SphereBore => {
            let sphere = make_sphere(&mut topo, 73.5 * s, SPHERE_SEGMENTS).unwrap();
            transform_solid(
                &mut topo,
                sphere,
                &Mat4::translation(25.5 * s, -1.5 * s, 0.0),
            )
            .unwrap();
            (sphere, 6.0 * s, 64.1 * s, [0.0, 0.0, 58.0 * s])
        }
    };
    let cylinder = make_cylinder(&mut topo, radius, height).unwrap();
    let shift = Mat4::translation(base[0], base[1], base[2]);
    let matrix = match placement {
        Placement::Translated => shift,
        Placement::Framed => shift * Mat4::rotation_z(-std::f64::consts::FRAC_PI_2),
    };
    transform_solid(&mut topo, cylinder, &matrix).unwrap();
    Operands {
        topo,
        cylinder,
        other,
        axis_xy: (base[0], base[1]),
        radius,
    }
}

fn surface_tag(surface: &FaceSurface) -> &'static str {
    match surface {
        FaceSurface::Plane { .. } => "plane",
        FaceSurface::Cylinder(_) => "cylinder",
        FaceSurface::Cone(_) => "cone",
        FaceSurface::Sphere(_) => "sphere",
        FaceSurface::Torus(_) => "torus",
        FaceSurface::Nurbs(_) => "nurbs",
    }
}

/// Face count, edge count, sorted surface tags and volume of a result body.
fn body(topo: &Topology, solid: SolidId) -> (usize, usize, Vec<&'static str>, f64) {
    let faces = solid_faces(topo, solid).unwrap();
    let mut tags: Vec<&'static str> = faces
        .iter()
        .map(|&f| surface_tag(topo.face(f).unwrap().surface()))
        .collect();
    tags.compact_sort_unstable();
    let edges = solid_edges(topo, solid).unwrap().len();
    (
        faces.len(),
        edges,
        tags,
        solid_volume(topo, solid, 0.01).unwrap(),
    )
}

fn check(family: Family, placement: Placement, s: f64, cylinder_first: bool) {
    let label = format!("{family:?} {placement:?} scale {s} cylinder_first {cylinder_first}");
    let Operands {
        mut topo,
        cylinder,
        other,
        axis_xy,
        radius,
    } = build(family, placement, s);
    let tol = 1e-6 * s;

    // Input identities: the cylinder wall, the cylinder's seam Line, and the
    // other operand's faces (the only faces a ring piece may pair the wall with).
    let cylinder_faces = solid_faces(&topo, cylinder).unwrap();
    let walls: Vec<usize> = cylinder_faces
        .iter()
        .filter(|&&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .map(|f| f.index())
        .collect();
    let [wall] = walls.as_slice() else {
        panic!("{label}: one cylinder wall, got {walls:?}");
    };
    let seams: Vec<usize> = solid_edges(&topo, cylinder)
        .unwrap()
        .into_iter()
        .filter(|&e| matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Line))
        .map(remus_topology::arena::Id::index)
        .collect();
    let [seam] = seams.as_slice() else {
        panic!("{label}: one cylinder seam line, got {seams:?}");
    };
    let other_faces: Vec<usize> = solid_faces(&topo, other)
        .unwrap()
        .into_iter()
        .map(remus_topology::arena::Id::index)
        .collect();

    let (a, b) = if cylinder_first {
        (cylinder, other)
    } else {
        (other, cylinder)
    };
    let mut plain_topo = topo.clone();
    let (result, evolution) = boolean_with_entity_evolution(&mut topo, family.op(), a, b)
        .unwrap_or_else(|e| {
            panic!("{label}: evolution boolean failed: {e}");
        });

    // Total: one event per result edge, none unresolved.
    let result_edges = solid_edges(&topo, result).unwrap();
    assert_eq!(
        evolution.edges.len(),
        result_edges.len(),
        "{label}: one event per result edge"
    );
    let unresolved: Vec<usize> = evolution
        .edges
        .iter()
        .filter(|(_, event)| matches!(event, EdgeEvent::Unresolved))
        .map(|(edge, _)| *edge)
        .collect();
    assert!(
        unresolved.is_empty(),
        "{label}: unresolved edges {unresolved:?}"
    );

    let on_wall = |p: Point3| ((p.x() - axis_xy.0).hypot(p.y() - axis_xy.1) - radius).abs() < tol;
    let mut ring_pieces = 0;
    let mut seam_pieces = 0;
    for edge_id in result_edges {
        let edge = topo.edge(edge_id).unwrap();
        let ends = [edge.start(), edge.end()].map(|v| topo.vertex(v).unwrap().point());
        let event = evolution
            .edges
            .iter()
            .find(|(index, _)| *index == edge_id.index())
            .map(|(_, event)| event.clone())
            .unwrap();
        match edge.curve() {
            // A Line on the wall is a piece of the cylinder's seam.
            EdgeCurve::Line if ends.iter().all(|&p| on_wall(p)) => {
                seam_pieces += 1;
                assert!(
                    matches!(event, EdgeEvent::Modified(from) | EdgeEvent::Preserved(from) if from == *seam),
                    "{label}: seam piece {ends:?} must descend from seam edge {seam}, got {event:?}"
                );
            }
            // The rims are the cylinder's own circles; every other curved edge
            // on the wall is a ring piece the boolean generated.
            EdgeCurve::Line => {}
            _ if matches!(event, EdgeEvent::Generated { .. }) => {
                ring_pieces += 1;
                let EdgeEvent::Generated {
                    face_a: Some(fa),
                    face_b: Some(fb),
                } = event
                else {
                    panic!("{label}: ring piece {ends:?} must name both faces, got {event:?}");
                };
                let (w, o) = if fa == *wall { (fa, fb) } else { (fb, fa) };
                assert_eq!(
                    w, *wall,
                    "{label}: ring piece {ends:?} pairs the wall: {event:?}"
                );
                assert!(
                    other_faces.contains(&o),
                    "{label}: ring piece {ends:?} pairs the wall with the other operand: {event:?}"
                );
                assert!(
                    ends.iter().all(|&p| on_wall(p)),
                    "{label}: generated edge {ends:?} lies on the wall"
                );
            }
            _ => {}
        }
    }
    let expected_ring = match family {
        Family::BoxBoss => 1,
        Family::SphereBoss | Family::SphereBore => 4,
    };
    assert_eq!(ring_pieces, expected_ring, "{label}: ring pieces");
    assert_eq!(seam_pieces, 1, "{label}: one kept seam piece");

    // The evolution entry point is a separate boolean implementation; its
    // body must be the plain boolean's.
    let plain = boolean(&mut plain_topo, family.op(), a, b).unwrap();
    let (ef, ee, et, ev) = body(&topo, result);
    let (pf, pe, pt, pv) = body(&plain_topo, plain);
    assert_eq!(
        (ef, ee, &et),
        (pf, pe, &pt),
        "{label}: same body as the plain boolean"
    );
    assert!(
        (ev - pv).abs() <= 1e-6 * pv.abs(),
        "{label}: volume {ev} vs plain {pv}"
    );
}

fn sweep(family: Family, orders: &[bool]) {
    let scales = match family {
        Family::BoxBoss => SCALES,
        Family::SphereBoss | Family::SphereBore => SPHERE_SCALES,
    };
    for placement in [Placement::Translated, Placement::Framed] {
        for s in scales {
            for &cylinder_first in orders {
                check(family, placement, s, cylinder_first);
            }
        }
    }
}

#[test]
fn box_boss_seam_piece_and_ring_keep_their_lineage() {
    sweep(Family::BoxBoss, &[false, true]);
}

#[test]
fn sphere_boss_seam_piece_and_ring_keep_their_lineage() {
    sweep(Family::SphereBoss, &[false, true]);
}

#[test]
fn sphere_bore_seam_piece_and_ring_keep_their_lineage() {
    // A cut is not symmetric: the cylinder-minus-sphere body is a different
    // part, so only the bore's order is the consumer case.
    sweep(Family::SphereBore, &[false]);
}
