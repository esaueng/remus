//! Fillets and chamfers on the circular edges of a countersink and of a
//! frustum boss standing on a plate.
//!
//! A countersink's entry rim is a convex plane–cone edge whose plate reaches
//! past the rim, the mirror of a frustum cap. The analytic plane–cone arms
//! only modelled the cap, so the fillet fell to the walker and refused with
//! `trimming failure`, and the chamfer refused outright. The countersink's
//! floor rim on its bore (a coaxial cone–cylinder edge) had no chamfer arm at
//! all. The concave twin, a frustum boss's foot on its plate, has the same
//! plate-past-the-rim geometry.
//!
//! Each result is held to an independent meridian oracle: the cross-section
//! the blend removes (or adds) is integrated with Green's theorem in the
//! `(ρ, z)` half-plane and revolved by Pappus, with no kernel code involved.
//! The bands must be the exact torus or cone of revolution, and the body
//! must stay strictly valid and closed.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::{PI, TAU};

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::{chamfer_v2, fillet_cascade};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::copy::copy_and_transform_solid;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cone};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const TOL: f64 = 1e-7;
/// Plate thickness of the countersunk block.
const TOP: f64 = 15.0;
/// Countersink: Ø12 × 90° over a Ø8 bore, centred at (20, 15).
const SINK_RADIUS: f64 = 6.0;
const BORE_RADIUS: f64 = 4.0;
const SINK_DEPTH: f64 = SINK_RADIUS - BORE_RADIUS;
/// Frustum boss on a 5 mm plate: r8 at its foot, r4 at its top, 10 tall.
const PLATE: f64 = 5.0;
const BOSS: (f64, f64, f64) = (8.0, 4.0, 10.0);

/// OpenZCAD's countersink: one revolved radial section, cut through the
/// block's top face.
fn countersunk_block(topo: &mut Topology) -> SolidId {
    let block = make_box(topo, 40.0, 30.0, TOP).unwrap();
    let (entry, exit) = (0.2, 0.2);
    let total = entry + TOP + exit;
    // 90° included angle: the sink's radius drops one per unit of depth.
    let section: Vec<Point3> = [
        (0.0, 0.0),
        (SINK_RADIUS + entry, 0.0),
        (BORE_RADIUS, entry + SINK_DEPTH),
        (BORE_RADIUS, total),
        (0.0, total),
    ]
    .iter()
    .map(|&(r, a)| Point3::new(r, 0.0, a))
    .collect();
    let wire = remus_topology::builder::make_polygon_wire(topo, &section, TOL).unwrap();
    let face = remus_topology::builder::make_planar_face_from_wire(topo, wire).unwrap();
    let tool = remus_operations::revolve::revolve(
        topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        TAU,
    )
    .unwrap();
    // Tool axis pointing down into the block from just above its top.
    let frame = Mat4([
        [0.0, 1.0, 0.0, 20.0],
        [1.0, 0.0, 0.0, 15.0],
        [0.0, 0.0, -1.0, TOP + entry],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    let tool = copy_and_transform_solid(topo, tool, &frame).unwrap();
    let cut = boolean(topo, BooleanOp::Cut, block, tool).unwrap();
    remus_operations::heal::unify_faces_checked(topo, cut).unwrap();
    cut
}

fn boss_on_plate(topo: &mut Topology) -> SolidId {
    let plate = make_box(topo, 40.0, 40.0, PLATE).unwrap();
    let cone = make_cone(topo, BOSS.0, BOSS.1, BOSS.2).unwrap();
    let cone = copy_and_transform_solid(topo, cone, &Mat4::translation(20.0, 20.0, PLATE)).unwrap();
    boolean(topo, BooleanOp::Fuse, plate, cone).unwrap()
}

/// The circular edge of radius `radius` at height `z`.
fn circle_edge(topo: &Topology, solid: SolidId, radius: f64, z: f64) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&edge| {
            matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Circle(c)
                if (c.radius() - radius).abs() < 1e-6 && (c.center().z() - z).abs() < 1e-6)
        })
        .unwrap_or_else(|| panic!("no circle r{radius} at z{z}"))
}

/// `2π ∬ ρ dA` of the closed meridian polygon `(ρ, z)`: Green's theorem
/// (`∬ ρ dρ dz = ∮ ρ²/2 dz`), revolved by Pappus.
fn revolved_volume(polygon: &[(f64, f64)]) -> f64 {
    let mut moment = 0.0;
    for (i, &(r0, z0)) in polygon.iter().enumerate() {
        let (r1, z1) = polygon[(i + 1) % polygon.len()];
        // Exact for a straight segment: ∫ ρ²/2 dz with ρ linear in z.
        moment += (z1 - z0) * (r0 * r0 + r0 * r1 + r1 * r1) / 6.0;
    }
    TAU * moment.abs()
}

/// The corner a rolling ball of `radius` centred at `centre` cuts off a rim:
/// rim → first contact, the short arc to the second contact, back to the rim.
fn ball_corner(rim: (f64, f64), centre: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let angle = |p: (f64, f64)| (p.1 - centre.1).atan2(p.0 - centre.0);
    let start = angle(a);
    let sweep = (angle(b) - start + PI).rem_euclid(TAU) - PI;
    let radius = (a.0 - centre.0).hypot(a.1 - centre.1);
    let mut polygon = vec![rim, a];
    let samples = 20_000;
    for k in 1..samples {
        let t = start + sweep * f64::from(k) / f64::from(samples);
        polygon.push((
            radius.mul_add(t.cos(), centre.0),
            radius.mul_add(t.sin(), centre.1),
        ));
    }
    polygon.push(b);
    revolved_volume(&polygon)
}

/// Run `blend`, check the result is a valid closed solid whose new face is
/// the expected exact surface, and return its signed volume change.
fn blended_volume_change(
    topo: &mut Topology,
    solid: SolidId,
    blend: impl FnOnce(&mut Topology) -> SolidId,
    band_is_expected: impl Fn(&FaceSurface) -> bool,
) -> f64 {
    let before = solid_volume(topo, solid, 1e-3).unwrap();
    let source_faces = solid_faces(topo, solid).unwrap();
    let result = blend(topo);
    let report = validate_solid(topo, result).unwrap();
    assert!(report.is_valid(), "invalid blend result: {report:?}");
    let shell = topo.solid(result).unwrap().outer_shell();
    validate_shell_closed(topo.shell(shell).unwrap(), topo).unwrap();
    let bands: Vec<_> = solid_faces(topo, result)
        .unwrap()
        .into_iter()
        .filter(|face| !source_faces.contains(face))
        .filter(|&face| band_is_expected(topo.face(face).unwrap().surface()))
        .collect();
    assert_eq!(bands.len(), 1, "expected one exact band");
    solid_volume(topo, result, 1e-3).unwrap() - before
}

fn is_torus(radius: f64, minor: f64) -> impl Fn(&FaceSurface) -> bool {
    move |surface| {
        matches!(surface, FaceSurface::Torus(t)
            if (t.major_radius() - radius).abs() < 1e-9 && (t.minor_radius() - minor).abs() < 1e-12)
    }
}

/// The cone of revolution through two meridian contacts `(ρ, z)`, judged by
/// its half-angle from the radial plane (the walls it replaced are cones too).
fn is_cone_through(a: (f64, f64), b: (f64, f64)) -> impl Fn(&FaceSurface) -> bool {
    let half_angle = (b.1 - a.1).abs().atan2((b.0 - a.0).abs());
    move |surface| matches!(surface, FaceSurface::Cone(c) if (c.half_angle() - half_angle).abs() < 1e-9)
}

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= 1e-6 * expected.abs().max(1.0),
        "volume change {actual} differs from the meridian oracle {expected}"
    );
}

#[test]
fn countersink_entry_rim_fillet_is_the_exact_torus() {
    for radius in [0.5_f64, 1.0, 1.5] {
        let mut topo = Topology::new();
        let solid = countersunk_block(&mut topo);
        let rim = circle_edge(&topo, solid, SINK_RADIUS, TOP);
        // 45° wall: the ball sits r·tan(22.5°) outside the rim, r below the top.
        let centre = (radius.mul_add((PI / 8.0).tan(), SINK_RADIUS), TOP - radius);
        let wall_foot = (
            centre.0 - radius * (PI / 4.0).sin(),
            radius.mul_add((PI / 4.0).cos(), centre.1),
        );
        let removed = ball_corner((SINK_RADIUS, TOP), centre, (centre.0, TOP), wall_foot);
        let change = blended_volume_change(
            &mut topo,
            solid,
            |topo| fillet_cascade(topo, solid, &[rim], radius).unwrap().solid,
            is_torus(centre.0, radius),
        );
        assert_close(change, -removed);
    }
}

#[test]
fn countersink_entry_rim_chamfer_is_the_exact_cone() {
    for distance in [0.5_f64, 1.0, 1.5] {
        let mut topo = Topology::new();
        let solid = countersunk_block(&mut topo);
        let rim = circle_edge(&topo, solid, SINK_RADIUS, TOP);
        let step = distance * (PI / 4.0).sin();
        let (plate, wall) = (
            (SINK_RADIUS + distance, TOP),
            (SINK_RADIUS - step, TOP - step),
        );
        let removed = revolved_volume(&[(SINK_RADIUS, TOP), plate, wall]);
        let change = blended_volume_change(
            &mut topo,
            solid,
            |topo| {
                let result = chamfer_v2(topo, solid, &[rim], distance, distance).unwrap();
                assert!(!result.is_partial);
                result.solid
            },
            is_cone_through(plate, wall),
        );
        assert_close(change, -removed);
    }
}

#[test]
fn countersink_floor_rim_chamfer_is_the_exact_cone() {
    for distance in [0.5_f64, 1.0] {
        let mut topo = Topology::new();
        let solid = countersunk_block(&mut topo);
        let floor = TOP - SINK_DEPTH;
        let rim = circle_edge(&topo, solid, BORE_RADIUS, floor);
        let step = distance * (PI / 4.0).sin();
        let (bore, sink) = (
            (BORE_RADIUS, floor - distance),
            (BORE_RADIUS + step, floor + step),
        );
        let removed = revolved_volume(&[(BORE_RADIUS, floor), bore, sink]);
        let change = blended_volume_change(
            &mut topo,
            solid,
            |topo| {
                let result = chamfer_v2(topo, solid, &[rim], distance, distance).unwrap();
                assert!(!result.is_partial);
                result.solid
            },
            is_cone_through(bore, sink),
        );
        assert_close(change, -removed);
    }
}

/// The boss wall's unit generator up from the foot, and its outward normal.
fn boss_wall() -> ((f64, f64), (f64, f64)) {
    let (dr, dz) = (BOSS.1 - BOSS.0, BOSS.2);
    let length = dr.hypot(dz);
    ((dr / length, dz / length), (dz / length, -dr / length))
}

#[test]
fn boss_foot_fillet_adds_the_exact_torus() {
    let radius = 1.0_f64;
    let mut topo = Topology::new();
    let solid = boss_on_plate(&mut topo);
    let foot = circle_edge(&topo, solid, BOSS.0, PLATE);
    let (_, normal) = boss_wall();
    // In the void: one radius above the plate and one off the wall.
    let rho = BOSS.0 + radius.mul_add(-normal.1, radius) / normal.0;
    let centre = (rho, PLATE + radius);
    let wall_foot = (rho - radius * normal.0, centre.1 - radius * normal.1);
    let added = ball_corner((BOSS.0, PLATE), centre, (rho, PLATE), wall_foot);
    let change = blended_volume_change(
        &mut topo,
        solid,
        |topo| fillet_cascade(topo, solid, &[foot], radius).unwrap().solid,
        is_torus(rho, radius),
    );
    assert_close(change, added);
}

#[test]
fn boss_foot_chamfer_adds_the_exact_cone() {
    let distance = 1.0_f64;
    let mut topo = Topology::new();
    let solid = boss_on_plate(&mut topo);
    let foot = circle_edge(&topo, solid, BOSS.0, PLATE);
    let (up, _) = boss_wall();
    let (plate, wall) = (
        (BOSS.0 + distance, PLATE),
        (
            distance.mul_add(up.0, BOSS.0),
            distance.mul_add(up.1, PLATE),
        ),
    );
    let added = revolved_volume(&[(BOSS.0, PLATE), plate, wall]);
    let change = blended_volume_change(
        &mut topo,
        solid,
        |topo| {
            let result = chamfer_v2(topo, solid, &[foot], distance, distance).unwrap();
            assert!(!result.is_partial);
            result.solid
        },
        is_cone_through(plate, wall),
    );
    assert_close(change, added);
}
