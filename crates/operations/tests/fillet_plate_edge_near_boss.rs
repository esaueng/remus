//! Rounding a plate's outline where it passes close to a boss's foot.
//!
//! A 7 mm plate carries a 7.75 radius hub fused onto its top face. Next to
//! the hub the plate's outline is a tangent S-curve: a concave R4 arc between
//! two convex R4 arcs, the concave arc clearing the hub's base circle by only
//! 0.137. A round of radius `r` on the top outline puts its contact line `r`
//! in from the edge, so for any `r` above the clearance that line runs under
//! the hub's foot. The rolling-ball rebuild used to emit it anyway: the top
//! face's outer loop crossed its inner loop (the hub's base circle), leaving
//! two open lens-shaped slivers, while the closed-shell, Euler and validation
//! checks all passed.
//!
//! The blend-into-hole guard now measures open curved blended edges as well
//! as straight ones and refuses with a typed `CliffEncountered` that names
//! the clearance; the validator reports crossed face loops. Below the
//! clearance the round still builds watertight and validates clean. The
//! straight-wall version of the plate, which the guard already covered,
//! keeps its behavior.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::{FRAC_PI_2, FRAC_PI_3};

use remus_math::curves::Circle3D;
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::blend_ops::{
    blend_failure_code, chamfer_v2_tangent_chain, fillet_cascade, fillet_v2,
};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::extrude::extrude;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::tessellate::{boundary_edge_count, tessellate_solid};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::builder::make_planar_face_from_wire;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const HUB: f64 = 7.75;
const CLEARANCE: f64 = 0.137;
const PLATE: f64 = 7.0;
const ARC: f64 = 4.0;
/// Radii that reach under the hub's foot.
const TOO_LARGE: [f64; 3] = [0.5, 0.3, 0.2];
/// A radius below the clearance.
const CLEARS: f64 = 0.1;

fn hub_on(topo: &mut Topology, plate: SolidId) -> SolidId {
    let hub = make_cylinder(topo, HUB, 10.0).unwrap();
    transform_solid(topo, hub, &Mat4::translation(0.0, 0.0, PLATE)).unwrap();
    boolean(topo, BooleanOp::Fuse, plate, hub).unwrap()
}

/// An outline segment: its start and, for an arc, (centre, counter-clockwise,
/// sweep).
type Segment = (Point3, Option<(Point3, bool, f64)>);

/// The plate: a rounded rectangle whose hub-side wall runs
/// line, convex arc, concave arc, convex arc, line, every joint tangent.
fn s_curve_body(topo: &mut Topology) -> SolidId {
    let p = |x: f64, y: f64| Point3::new(x, y, 0.0);
    // The concave arc's centre is `HUB + CLEARANCE + ARC` out, so its nearest
    // point clears the hub circle by `CLEARANCE`; each convex arc's centre is
    // `2 ARC` from it and `ARC` in from the straight wall.
    let wall = HUB + CLEARANCE + ARC;
    let convex_y = 48.0_f64.sqrt();
    let joint_y = 2.0 * 3.0_f64.sqrt();
    let (left, half, corner) = (-20.0, 20.0, 5.0);
    let segments: [Segment; 12] = [
        (p(wall, -half + corner), None),
        (
            p(wall, -convex_y),
            Some((p(wall - ARC, -convex_y), true, FRAC_PI_3)),
        ),
        (
            p(wall - 2.0, -joint_y),
            Some((p(wall, 0.0), false, 2.0 * FRAC_PI_3)),
        ),
        (
            p(wall - 2.0, joint_y),
            Some((p(wall - ARC, convex_y), true, FRAC_PI_3)),
        ),
        (p(wall, convex_y), None),
        (
            p(wall, half - corner),
            Some((p(wall - corner, half - corner), true, FRAC_PI_2)),
        ),
        (p(wall - corner, half), None),
        (
            p(left + corner, half),
            Some((p(left + corner, half - corner), true, FRAC_PI_2)),
        ),
        (p(left, half - corner), None),
        (
            p(left, -half + corner),
            Some((p(left + corner, -half + corner), true, FRAC_PI_2)),
        ),
        (p(left + corner, -half), None),
        (
            p(wall - corner, -half),
            Some((p(wall - corner, -half + corner), true, FRAC_PI_2)),
        ),
    ];
    let vertices: Vec<_> = segments
        .iter()
        .map(|(start, _)| topo.add_vertex(Vertex::new(*start, 1e-7)))
        .collect();
    let n = segments.len();
    let mut oriented = Vec::with_capacity(n);
    for (i, (start, arc)) in segments.iter().enumerate() {
        let (v0, v1) = (vertices[i], vertices[(i + 1) % n]);
        let edge = match arc {
            None => Edge::new(v0, v1, EdgeCurve::Line),
            Some((centre, ccw, sweep)) => {
                let normal = Vec3::new(0.0, 0.0, if *ccw { 1.0 } else { -1.0 });
                let offset = *start - *centre;
                let circle =
                    Circle3D::new_with_ref(*centre, normal, offset.length(), offset).unwrap();
                let mut edge = Edge::new(v0, v1, EdgeCurve::Circle(circle));
                edge.set_trim(Some((0.0, *sweep)));
                edge
            }
        };
        oriented.push(OrientedEdge::new(topo.add_edge(edge), true));
    }
    let wire = topo.add_wire(Wire::new(oriented, true).unwrap());
    let face = make_planar_face_from_wire(topo, wire).unwrap();
    let plate = extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), PLATE).unwrap();
    hub_on(topo, plate)
}

/// The top edge of the concave arc, the one nearest the hub.
fn concave_top_edge(topo: &Topology, solid: SolidId) -> EdgeId {
    let centre = HUB + CLEARANCE + ARC;
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&e| {
            let EdgeCurve::Circle(circle) = topo.edge(e).unwrap().curve() else {
                return false;
            };
            let c = circle.center();
            (c.x() - centre).abs() < 1e-9 && c.y().abs() < 1e-9 && (c.z() - PLATE).abs() < 1e-9
        })
        .expect("concave arc top edge")
}

/// Every edge of the plate's top outline: the tangent chain through the
/// concave arc.
fn top_outline(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    let seed = concave_top_edge(topo, solid);
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(|f| topo.face(f).unwrap())
        .find(|face| {
            matches!(face.surface(), FaceSurface::Plane { .. })
                && !face.inner_wires().is_empty()
                && topo
                    .wire(face.outer_wire())
                    .unwrap()
                    .edges()
                    .iter()
                    .any(|oe| oe.edge() == seed)
        })
        .map(|face| {
            topo.wire(face.outer_wire())
                .unwrap()
                .edges()
                .iter()
                .map(remus_topology::wire::OrientedEdge::edge)
                .collect()
        })
        .expect("plate top face")
}

/// The plate as a box whose wall is straight, `CLEARANCE` outside the hub.
fn straight_wall_body(topo: &mut Topology) -> SolidId {
    let wall = HUB + CLEARANCE;
    let plate = make_box(topo, 40.0, 40.0, PLATE).unwrap();
    transform_solid(topo, plate, &Mat4::translation(wall - 40.0, -20.0, 0.0)).unwrap();
    hub_on(topo, plate)
}

fn straight_top_edge(topo: &Topology, solid: SolidId) -> EdgeId {
    let wall = HUB + CLEARANCE;
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&e| {
            let edge = topo.edge(e).unwrap();
            let on = |v| {
                let p: Point3 = topo.vertex(v).unwrap().point();
                (p.x() - wall).abs() < 1e-9 && (p.z() - PLATE).abs() < 1e-9
            };
            on(edge.start()) && on(edge.end())
        })
        .expect("straight hub-side top edge")
}

/// A refusal that names the clearance, with the input left as it was.
fn assert_cliff(
    topo: &Topology,
    solid: SolidId,
    before: (usize, usize, usize),
    result: Result<impl std::fmt::Debug, OperationsError>,
    label: &str,
) {
    let error = result.expect_err(label);
    assert_eq!(
        blend_failure_code(&error),
        "cliff-encountered",
        "{label}: {error}"
    );
    let OperationsError::Blend(remus_blend::BlendError::CliffEncountered {
        available_radius, ..
    }) = error
    else {
        panic!("{label}: {error}");
    };
    assert!(
        (available_radius - CLEARANCE).abs() < 1e-3,
        "{label}: available radius {available_radius}"
    );
    assert_eq!(
        solid_entity_counts(topo, solid).unwrap(),
        before,
        "{label}: input changed"
    );
}

/// A built round: topology changed, closed, validator clean, mesh watertight.
fn assert_sound(topo: &Topology, input: (usize, usize, usize), solid: SolidId, label: &str) {
    assert_ne!(
        solid_entity_counts(topo, solid).unwrap(),
        input,
        "{label}: no-op"
    );
    let shell = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    assert!(
        validate_shell_closed(shell, topo).is_ok(),
        "{label}: open shell"
    );
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {:?}", report.issues);
    let strict = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(
        !strict
            .issues
            .iter()
            .any(|i| i.check == remus_check::validate::CheckId::FaceLoopContainment),
        "{label}: {:?}",
        strict.issues
    );
    let mesh = tessellate_solid(topo, solid, 0.01).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0, "{label}: open mesh");
}

#[test]
fn round_of_the_whole_outline_past_the_hub_is_refused() {
    for radius in TOO_LARGE {
        let mut topo = Topology::new();
        let solid = s_curve_body(&mut topo);
        let edges = top_outline(&topo, solid);
        let before = solid_entity_counts(&topo, solid).unwrap();
        let result = fillet_cascade(&mut topo, solid, &edges, radius);
        assert_cliff(
            &topo,
            solid,
            before,
            result.map(|r| r.solid),
            &format!("r={radius}"),
        );
    }
}

#[test]
fn round_of_the_arc_alone_past_the_hub_is_refused() {
    for radius in TOO_LARGE {
        let mut topo = Topology::new();
        let solid = s_curve_body(&mut topo);
        let edge = concave_top_edge(&topo, solid);
        let before = solid_entity_counts(&topo, solid).unwrap();
        let result = fillet_cascade(&mut topo, solid, &[edge], radius);
        assert_cliff(
            &topo,
            solid,
            before,
            result.map(|r| r.solid),
            &format!("r={radius}"),
        );
        let result = fillet_v2(&mut topo, solid, &[edge], radius);
        assert_cliff(
            &topo,
            solid,
            before,
            result.map(|r| r.solid),
            &format!("v2 r={radius}"),
        );
    }
}

#[test]
fn round_below_the_clearance_builds_sound() {
    let mut topo = Topology::new();
    let solid = s_curve_body(&mut topo);
    let edges = top_outline(&topo, solid);
    let before = solid_entity_counts(&topo, solid).unwrap();
    let result = fillet_cascade(&mut topo, solid, &edges, CLEARS).unwrap();
    assert_sound(&topo, before, result.solid, "outline r=0.1");

    let mut topo = Topology::new();
    let solid = s_curve_body(&mut topo);
    let edge = concave_top_edge(&topo, solid);
    let result = fillet_cascade(&mut topo, solid, &[edge], CLEARS).unwrap();
    assert_sound(&topo, before, result.solid, "arc r=0.1");
}

/// A chamfer of the same outline never ships crossed loops: it is refused or
/// it is sound.
#[test]
fn bevel_of_the_outline_past_the_hub_is_not_shipped_broken() {
    for distance in [0.5, CLEARS] {
        let mut topo = Topology::new();
        let solid = s_curve_body(&mut topo);
        let edge = concave_top_edge(&topo, solid);
        let before = solid_entity_counts(&topo, solid).unwrap();
        match chamfer_v2_tangent_chain(&mut topo, solid, &[edge], distance, distance) {
            Ok(result) => assert_sound(&topo, before, result.solid, &format!("d={distance}")),
            Err(error) => {
                assert!(distance > CLEARANCE, "d={distance} refused: {error}");
                assert_eq!(solid_entity_counts(&topo, solid).unwrap(), before);
            }
        }
    }
}

#[test]
fn straight_wall_keeps_its_refusal_and_its_round() {
    for radius in TOO_LARGE {
        let mut topo = Topology::new();
        let solid = straight_wall_body(&mut topo);
        let edge = straight_top_edge(&topo, solid);
        let before = solid_entity_counts(&topo, solid).unwrap();
        let result = fillet_cascade(&mut topo, solid, &[edge], radius);
        assert_cliff(
            &topo,
            solid,
            before,
            result.map(|r| r.solid),
            &format!("r={radius}"),
        );
    }
    let mut topo = Topology::new();
    let solid = straight_wall_body(&mut topo);
    let edge = straight_top_edge(&topo, solid);
    let before = solid_entity_counts(&topo, solid).unwrap();
    let result = fillet_cascade(&mut topo, solid, &[edge], CLEARS).unwrap();
    assert_sound(&topo, before, result.solid, "straight r=0.1");
}

/// A quarter of an R100 disc with a pin hole of radius 0.1 whose rim comes
/// exactly 1 from the disc's arc. Measured on samples, the clearance reads
/// wrong both ways: the long arc's chords (at the sample cap) sit up to 5e-4
/// inside it, so a radius that fits could be refused, and the pin's eight
/// rim samples miss its nearest point, so 1.0001 slipped past the guard to
/// the validator. The arc is measured exactly and the pin's rim re-sampled
/// around its nearest point: 0.9999 is not refused at the cliff, and 1.0001
/// is, naming the clearance.
#[test]
fn long_arc_guard_measures_the_arc_not_its_chords() {
    const DISC: f64 = 100.0;
    const PIN: f64 = 0.1;
    let build = |topo: &mut Topology| {
        let disc = make_cylinder(topo, DISC, PLATE).unwrap();
        let quadrant = make_box(topo, DISC + 10.0, DISC + 10.0, PLATE + 2.0).unwrap();
        transform_solid(topo, quadrant, &Mat4::translation(0.0, 0.0, -1.0)).unwrap();
        let plate = boolean(topo, BooleanOp::Intersect, disc, quadrant).unwrap();
        // Off the arc's middle, so no sample of it need sit at the nearest
        // point.
        let angle = 0.3_f64;
        let at = DISC - 1.0 - PIN;
        let pin = make_cylinder(topo, PIN, PLATE + 2.0).unwrap();
        transform_solid(
            topo,
            pin,
            &Mat4::translation(at * angle.cos(), at * angle.sin(), -1.0),
        )
        .unwrap();
        boolean(topo, BooleanOp::Cut, plate, pin).unwrap()
    };
    let rim = |topo: &Topology, solid: SolidId| {
        solid_edges(topo, solid)
            .unwrap()
            .into_iter()
            .find(|&e| {
                let edge = topo.edge(e).unwrap();
                let EdgeCurve::Circle(circle) = edge.curve() else {
                    return false;
                };
                (circle.radius() - DISC).abs() < 1e-9 && (circle.center().z() - PLATE).abs() < 1e-9
            })
            .expect("top arc of the disc")
    };

    let mut topo = Topology::new();
    let solid = build(&mut topo);
    let edge = rim(&topo, solid);
    if let Err(error) = fillet_v2(&mut topo, solid, &[edge], 0.9999) {
        assert_ne!(blend_failure_code(&error), "cliff-encountered", "{error}");
    }

    let mut topo = Topology::new();
    let solid = build(&mut topo);
    let edge = rim(&topo, solid);
    let before = solid_entity_counts(&topo, solid).unwrap();
    let error = fillet_v2(&mut topo, solid, &[edge], 1.0001)
        .map(|result| result.solid)
        .expect_err("past the pin");
    let OperationsError::Blend(remus_blend::BlendError::CliffEncountered {
        available_radius, ..
    }) = error
    else {
        panic!("{error}");
    };
    // The pin's rim is sampled, so the reading may sit a hair above 1.
    assert!(
        (0.0..1e-4).contains(&(available_radius - 1.0 + 1e-9)),
        "available radius {available_radius}"
    );
    assert_eq!(solid_entity_counts(&topo, solid).unwrap(), before);
}
