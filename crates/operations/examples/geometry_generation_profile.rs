//! Geometry-generation probe: for each generator (primitives, extrude,
//! revolve, loft, sweep, pipe, helix, gears, threads) it times, on a fresh
//! `Topology` per run:
//!
//! - setup (profile wires/faces, blanks the generator consumes),
//! - construction (the generator call only),
//! - display tessellation (`tessellate_solid` at the printed deflection),
//! - `solid_volume` at the same deflection,
//!
//! and records the face and triangle counts. Each phase reports the median
//! over the runs (5 by default; 3 when one run takes over 1 s, 1 over 10 s).
//! Parameters come from each module's own tests.
//!
//! The twisted and tapered extrusions (`extrude::twist`, `extrude::taper`)
//! are crate-private and are reached only through the helical, herringbone
//! and bevel gears and the threads.
//!
//! ```sh
//! cargo run --profile profiling -p remus-operations --example geometry_generation_profile
//! cargo run --profile profiling -p remus-operations --example geometry_generation_profile -- spur --runs 9
//! ```
#![allow(
    clippy::print_stdout,
    clippy::print_stderr,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::too_many_lines,
    missing_docs
)]

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI, TAU};
use std::fmt::Display;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::time::{Duration, Instant};

use remus_math::compact_sort::CompactSort;
use remus_math::curves::Circle3D;
use remus_math::mat::Mat4;
use remus_math::nurbs::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::gear::{
    BevelGearParams, CylinderGearOptions, EnvelopingWheelOptions, EnvelopingWormParams, GearKind,
    GearParams, GearPlacement, Hand, ThroatedWheelOptions, WormParams, cut_gear_teeth,
    make_bevel_gear, make_enveloping_worm, make_enveloping_worm_wheel, make_gear,
    make_throated_worm_wheel, make_worm, worm_wheel,
};
use remus_operations::thread::{
    CylinderThreadOptions, ThreadEnd, ThreadForm, ThreadHand, ThreadKind, cut_thread,
};
use remus_operations::transform::transform_solid;
use remus_operations::{
    extrude, helix, loft, measure, pipe, primitives, revolve, sweep, tessellate,
};
use remus_topology::Topology;
use remus_topology::builder::{make_circle_edge, make_polygon_wire};
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

const TOL: f64 = 1e-7;
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

type Built = Result<(SolidId, Duration), String>;

struct Scenario {
    name: &'static str,
    deflection: f64,
    build: fn(&mut Topology) -> Built,
}

/// Time the generator call alone.
fn timed<T, E: Display>(f: impl FnOnce() -> Result<T, E>) -> Result<(T, Duration), String> {
    let start = Instant::now();
    let value = f().map_err(|e| e.to_string())?;
    Ok((value, start.elapsed()))
}

fn s<E: Display>(e: E) -> String {
    e.to_string()
}

// ── profile helpers (setup) ─────────────────────────────────────────────

fn plane_face(
    topo: &mut Topology,
    outer: WireId,
    holes: Vec<WireId>,
    normal: Vec3,
    at: Point3,
) -> FaceId {
    let d = normal.x() * at.x() + normal.y() * at.y() + normal.z() * at.z();
    topo.add_face(Face::new(outer, holes, FaceSurface::Plane { normal, d }))
}

fn circle_wire(
    topo: &mut Topology,
    center: Point3,
    normal: Vec3,
    radius: f64,
    forward: bool,
) -> Result<WireId, String> {
    let edge = make_circle_edge(topo, center, normal, radius, TOL).map_err(s)?;
    Ok(topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, forward)], true).map_err(s)?))
}

fn circle_face(
    topo: &mut Topology,
    center: Point3,
    normal: Vec3,
    radius: f64,
) -> Result<FaceId, String> {
    let wire = circle_wire(topo, center, normal, radius, true)?;
    let n = normal.normalize().map_err(s)?;
    Ok(plane_face(topo, wire, vec![], n, center))
}

fn polygon_face(topo: &mut Topology, points: &[Point3]) -> Result<FaceId, String> {
    let wire = make_polygon_wire(topo, points, TOL).map_err(s)?;
    // Newell normal of the polygon.
    let mut n = Vec3::new(0.0, 0.0, 0.0);
    for (i, a) in points.iter().enumerate() {
        let b = points[(i + 1) % points.len()];
        n += Vec3::new(
            (a.y() - b.y()) * (a.z() + b.z()),
            (a.z() - b.z()) * (a.x() + b.x()),
            (a.x() - b.x()) * (a.y() + b.y()),
        );
    }
    let n = n.normalize().map_err(s)?;
    Ok(plane_face(topo, wire, vec![], n, points[0]))
}

fn square_at(topo: &mut Topology, size: f64, z: f64) -> Result<FaceId, String> {
    let h = size / 2.0;
    polygon_face(
        topo,
        &[
            Point3::new(-h, -h, z),
            Point3::new(h, -h, z),
            Point3::new(h, h, z),
            Point3::new(-h, h, z),
        ],
    )
}

fn xz_polygon(topo: &mut Topology, xz: &[(f64, f64)]) -> Result<FaceId, String> {
    let points: Vec<Point3> = xz.iter().map(|&(x, z)| Point3::new(x, 0.0, z)).collect();
    polygon_face(topo, &points)
}

fn line_path(to: Point3) -> Result<NurbsCurve, String> {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(0.0, 0.0, 0.0), to],
        vec![1.0, 1.0],
    )
    .map_err(s)
}

/// Quarter circle of `radius` in the XZ plane, starting at the origin.
fn quarter_arc_path(radius: f64) -> Result<NurbsCurve, String> {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(radius, 0.0, 0.0),
            Point3::new(radius, 0.0, radius),
        ],
        vec![1.0, FRAC_1_SQRT_2, 1.0],
    )
    .map_err(s)
}

/// A cubic S-shaped spline leaving the origin along +Z.
fn spline_path() -> Result<NurbsCurve, String> {
    NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(10.0, 5.0, 15.0),
            Point3::new(15.0, -5.0, 25.0),
            Point3::new(20.0, 0.0, 35.0),
        ],
        vec![1.0; 5],
    )
    .map_err(s)
}

fn cylinder_face_of(topo: &Topology, solid: SolidId, radius: f64) -> Result<FaceId, String> {
    solid_faces(topo, solid)
        .map_err(s)?
        .into_iter()
        .find(|&f| matches!(topo.face(f).map(remus_topology::face::Face::surface), Ok(FaceSurface::Cylinder(c)) if (c.radius() - radius).abs() < 1e-9))
        .ok_or_else(|| format!("no cylinder face of radius {radius}"))
}

fn moved_cylinder(
    topo: &mut Topology,
    radius: f64,
    height: f64,
    z: f64,
) -> Result<SolidId, String> {
    let solid = primitives::make_cylinder(topo, radius, height).map_err(s)?;
    transform_solid(topo, solid, &Mat4::translation(0.0, 0.0, z)).map_err(s)?;
    Ok(solid)
}

// ── scenarios ───────────────────────────────────────────────────────────

fn prim_box(t: &mut Topology) -> Built {
    timed(|| primitives::make_box(t, 20.0, 20.0, 20.0))
}
fn prim_cylinder(t: &mut Topology) -> Built {
    timed(|| primitives::make_cylinder(t, 10.0, 20.0))
}
fn prim_cone(t: &mut Topology) -> Built {
    timed(|| primitives::make_cone(t, 10.0, 5.0, 20.0))
}
fn prim_sphere(t: &mut Topology) -> Built {
    timed(|| primitives::make_sphere(t, 10.0, 32))
}
fn prim_torus(t: &mut Topology) -> Built {
    timed(|| primitives::make_torus(t, 15.0, 4.0, 32))
}

fn extrude_rect(t: &mut Topology) -> Built {
    let face = square_at(t, 20.0, 0.0)?;
    timed(|| extrude::extrude(t, face, Z, 10.0))
}
fn extrude_circle(t: &mut Topology) -> Built {
    let face = circle_face(t, Point3::new(0.0, 0.0, 0.0), Z, 10.0)?;
    timed(|| extrude::extrude(t, face, Z, 20.0))
}
fn extrude_rect_hole(t: &mut Topology) -> Built {
    // extrude/tests.rs: extrude_rect_with_circular_hole_uses_exact_cylinder.
    let outer = make_polygon_wire(
        t,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(20.0, 0.0, 0.0),
            Point3::new(20.0, 20.0, 0.0),
            Point3::new(0.0, 20.0, 0.0),
        ],
        TOL,
    )
    .map_err(s)?;
    let hole = circle_wire(t, Point3::new(10.0, 10.0, 0.0), Z, 3.0, false)?;
    let face = plane_face(t, outer, vec![hole], Z, Point3::new(0.0, 0.0, 0.0));
    timed(|| extrude::extrude(t, face, Z, 10.0))
}
fn extrude_spline(t: &mut Topology) -> Built {
    // extrude/profile_compatibility_tests.rs family 4: a 40×40 square whose
    // sides are bulged quadratic NURBS.
    let p = |x: f64, y: f64| Point3::new(x, y, 0.0);
    let w = make_polygon_wire(
        t,
        &[p(-20., -20.), p(20., -20.), p(20., 20.), p(-20., 20.)],
        TOL,
    )
    .map_err(s)?;
    for oe in t.wire(w).map_err(s)?.edges().to_vec() {
        let e = t.edge(oe.edge()).map_err(s)?;
        let a = t.vertex(e.start()).map_err(s)?.point();
        let b = t.vertex(e.end()).map_err(s)?.point();
        let delta = b - a;
        let mid = a + delta * 0.5 + Vec3::new(delta.y(), -delta.x(), 0.) * 0.05;
        let curve = NurbsCurve::new(
            2,
            vec![0., 0., 0., 1., 1., 1.],
            vec![a, mid, b],
            vec![1.; 3],
        )
        .map_err(s)?;
        t.edge_mut(oe.edge())
            .map_err(s)?
            .set_curve(EdgeCurve::NurbsCurve(curve));
    }
    let face = plane_face(t, w, vec![], Z, p(0.0, 0.0));
    timed(|| extrude::extrude(t, face, Z, 10.0))
}

fn revolve_rect_full(t: &mut Topology) -> Built {
    let face = xz_polygon(t, &[(5.0, 0.0), (8.0, 0.0), (8.0, 4.0), (5.0, 4.0)])?;
    timed(|| revolve::revolve(t, face, Point3::new(0.0, 0.0, 0.0), Z, TAU))
}
fn revolve_rect_partial(t: &mut Topology) -> Built {
    let face = xz_polygon(t, &[(5.0, 0.0), (8.0, 0.0), (8.0, 4.0), (5.0, 4.0)])?;
    timed(|| revolve::revolve(t, face, Point3::new(0.0, 0.0, 0.0), Z, FRAC_PI_2))
}
fn revolve_arc_profile(t: &mut Topology) -> Built {
    // revolve.rs: revolve_arc_profile_edge_is_torus_band (half disc).
    let (d, rho) = (10.0_f64, 3.0_f64);
    let circ = Circle3D::new(Point3::new(d, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), rho).map_err(s)?;
    let v_bot = t.add_vertex(Vertex::new(circ.evaluate(-FRAC_PI_2), TOL));
    let v_top = t.add_vertex(Vertex::new(circ.evaluate(FRAC_PI_2), TOL));
    let e_arc = t.add_edge(Edge::new(v_bot, v_top, EdgeCurve::Circle(circ)));
    let e_dia = t.add_edge(Edge::new(v_top, v_bot, EdgeCurve::Line));
    let wire = Wire::new(
        vec![
            OrientedEdge::new(e_arc, true),
            OrientedEdge::new(e_dia, true),
        ],
        true,
    )
    .map_err(s)?;
    let wid = t.add_wire(wire);
    let face = plane_face(
        t,
        wid,
        vec![],
        Vec3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 0.0),
    );
    timed(|| revolve::revolve(t, face, Point3::new(0.0, 0.0, 0.0), Z, TAU))
}
fn revolve_circle_partial(t: &mut Topology) -> Built {
    // revolve.rs: revolve_circle_partial_turn_is_trimmed_torus.
    let face = circle_face(t, Point3::new(6.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 2.0)?;
    timed(|| revolve::revolve(t, face, Point3::new(0.0, 0.0, 0.0), Z, 2.0 * PI / 3.0))
}

fn loft_coaxial_circles(t: &mut Topology) -> Built {
    let a = circle_face(t, Point3::new(0.0, 0.0, 0.0), Z, 10.0)?;
    let b = circle_face(t, Point3::new(0.0, 0.0, 20.0), Z, 5.0)?;
    timed(|| loft::loft(t, &[a, b]))
}
fn loft_offset_circles(t: &mut Topology) -> Built {
    // loft/tests.rs: loft_non_coaxial_circles_make_exact_oblique_band.
    let a = circle_face(t, Point3::new(0.0, 0.0, 0.0), Z, 10.0)?;
    let b = circle_face(t, Point3::new(8.0, 0.0, 20.0), Z, 5.0)?;
    timed(|| loft::loft(t, &[a, b]))
}
fn loft_three_squares(t: &mut Topology) -> Built {
    // loft/tests.rs: loft_three_profiles, scaled ×10.
    let a = square_at(t, 20.0, 0.0)?;
    let b = square_at(t, 10.0, 15.0)?;
    let c = square_at(t, 20.0, 30.0)?;
    timed(|| loft::loft(t, &[a, b, c]))
}
fn loft_smooth_four_squares(t: &mut Topology) -> Built {
    // loft/tests.rs: loft_smooth_four_profiles, scaled ×10.
    let a = square_at(t, 20.0, 0.0)?;
    let b = square_at(t, 15.0, 10.0)?;
    let c = square_at(t, 10.0, 20.0)?;
    let d = square_at(t, 15.0, 30.0)?;
    timed(|| loft::loft_smooth(t, &[a, b, c, d]))
}

fn sweep_square_line(t: &mut Topology) -> Built {
    let face = square_at(t, 5.0, 0.0)?;
    let path = line_path(Point3::new(0.0, 0.0, 30.0))?;
    timed(|| sweep::sweep(t, face, &path))
}
fn sweep_square_arc(t: &mut Topology) -> Built {
    let face = square_at(t, 5.0, 0.0)?;
    let path = quarter_arc_path(20.0)?;
    timed(|| sweep::sweep(t, face, &path))
}
fn sweep_circle_spline(t: &mut Topology) -> Built {
    let face = circle_face(t, Point3::new(0.0, 0.0, 0.0), Z, 2.0)?;
    let path = spline_path()?;
    timed(|| sweep::sweep(t, face, &path))
}
fn sweep_smooth_square_arc(t: &mut Topology) -> Built {
    let face = square_at(t, 5.0, 0.0)?;
    let path = quarter_arc_path(20.0)?;
    timed(|| sweep::sweep_smooth(t, face, &path))
}
fn sweep_smooth_square_spline(t: &mut Topology) -> Built {
    let face = square_at(t, 5.0, 0.0)?;
    let path = spline_path()?;
    timed(|| sweep::sweep_smooth(t, face, &path))
}
fn pipe_square_spline(t: &mut Topology) -> Built {
    let face = square_at(t, 5.0, 0.0)?;
    let path = spline_path()?;
    timed(|| pipe::pipe(t, face, &path, None))
}
fn pipe_guided(t: &mut Topology) -> Built {
    // pipe.rs: pipe_with_expanding_guide, scaled ×10.
    let face = square_at(t, 10.0, 0.0)?;
    let path = line_path(Point3::new(0.0, 0.0, 20.0))?;
    let guide = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(10.0, 0.0, 0.0), Point3::new(20.0, 0.0, 20.0)],
        vec![1.0, 1.0],
    )
    .map_err(s)?;
    timed(|| pipe::pipe(t, face, &path, Some(&guide)))
}

fn helix_legacy(t: &mut Topology, turns: f64) -> Built {
    let face = circle_face(t, Point3::new(0.0, 0.0, 0.0), Z, 1.5)?;
    timed(|| helix::helical_sweep(t, face, Point3::new(0.0, 0.0, 0.0), Z, 12.0, 5.0, turns, 16))
}
fn helix_legacy_5(t: &mut Topology) -> Built {
    helix_legacy(t, 5.0)
}
fn helix_legacy_20(t: &mut Topology) -> Built {
    helix_legacy(t, 20.0)
}
fn helix_in_place(t: &mut Topology, turns: f64) -> Built {
    // helix/in_place/tests.rs: spring (r 1.5 wire at 12 from Z, pitch 5).
    let face = circle_face(
        t,
        Point3::new(12.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        1.5,
    )?;
    timed(|| helix::helical_sweep_in_place(t, face, Point3::new(0.0, 0.0, 0.0), Z, 5.0, turns))
}
fn helix_in_place_5(t: &mut Topology) -> Built {
    helix_in_place(t, 5.0)
}
fn helix_in_place_20(t: &mut Topology) -> Built {
    helix_in_place(t, 20.0)
}
fn helix_in_place_rect(t: &mut Topology) -> Built {
    // helix/in_place/tests.rs: rectangle_beside_the_axis (pitch 2, 2 turns).
    let face = xz_polygon(t, &[(4.0, 0.0), (6.0, 0.0), (6.0, 1.5), (4.0, 1.5)])?;
    timed(|| helix::helical_sweep_in_place(t, face, Point3::new(0.0, 0.0, 0.0), Z, 2.0, 2.0))
}

fn gear(t: &mut Topology, params: &GearParams) -> Built {
    timed(|| make_gear(t, params, &GearPlacement::default())).map(|(o, d)| (o.solid, d))
}
fn gear_spur20(t: &mut Topology) -> Built {
    gear(t, &GearParams::standard(GearKind::External, 2.0, 20, 5.0))
}
fn gear_spur60(t: &mut Topology) -> Built {
    gear(t, &GearParams::standard(GearKind::External, 2.0, 60, 5.0))
}
fn gear_helical(t: &mut Topology) -> Built {
    // gear/tests/helical.rs: helical(20, 20°, 10, Right).
    gear(
        t,
        &GearParams {
            helix_angle: 20.0_f64.to_radians(),
            hand: Hand::Right,
            ..GearParams::standard(GearKind::External, 2.0, 20, 10.0)
        },
    )
}
fn gear_herringbone(t: &mut Topology) -> Built {
    // gear/tests/herringbone.rs: herringbone(20, 30°, 20, Right).
    gear(
        t,
        &GearParams {
            helix_angle: 30.0_f64.to_radians(),
            hand: Hand::Right,
            herringbone: true,
            ..GearParams::standard(GearKind::External, 2.0, 20, 20.0)
        },
    )
}
fn gear_internal(t: &mut Topology) -> Built {
    let mut params = GearParams::standard(GearKind::Internal, 1.5, 48, 6.0);
    params.rim_diameter = Some(90.0);
    gear(t, &params)
}
fn gear_rack(t: &mut Topology) -> Built {
    gear(t, &GearParams::standard(GearKind::Rack, 2.0, 7, 4.0))
}
fn gear_bevel(t: &mut Topology) -> Built {
    let params = BevelGearParams::standard(2.0, 20, 40, 8.0);
    timed(|| make_bevel_gear(t, &params, &GearPlacement::default())).map(|(o, d)| (o.solid, d))
}
fn worm_params() -> WormParams {
    WormParams::standard(2.0, 1, 10.0, 20.0)
}
fn gear_worm(t: &mut Topology) -> Built {
    let params = worm_params();
    timed(|| make_worm(t, &params, &GearPlacement::default())).map(|(o, d)| (o.solid, d))
}
fn gear_worm_wheel(t: &mut Topology) -> Built {
    let wheel = worm_wheel(&worm_params(), 30, 12.0).map_err(s)?;
    gear(t, &wheel.params)
}
fn gear_throated_wheel(t: &mut Topology) -> Built {
    let worm = worm_params();
    timed(|| {
        make_throated_worm_wheel(
            t,
            &worm,
            30,
            14.0,
            &ThroatedWheelOptions::default(),
            &GearPlacement::default(),
        )
    })
    .map(|(o, d)| (o.gear.solid, d))
}
fn hindley_params() -> EnvelopingWormParams {
    EnvelopingWormParams::standard(2.0, 1, 30, 10.0, 24.0)
}
fn gear_enveloping_worm(t: &mut Topology) -> Built {
    let params = hindley_params();
    timed(|| make_enveloping_worm(t, &params, &GearPlacement::default())).map(|(o, d)| (o.solid, d))
}
fn gear_enveloping_wheel(t: &mut Topology) -> Built {
    let worm = hindley_params();
    timed(|| {
        make_enveloping_worm_wheel(
            t,
            &worm,
            &EnvelopingWheelOptions::new(8.0),
            &GearPlacement::default(),
        )
    })
    .map(|(o, d)| (o.solid, d))
}
fn gear_cut_teeth(t: &mut Topology) -> Built {
    // gear/cylinder/tests.rs: keep-diameter spur(20, 6) on an r 11 boss.
    let blank = primitives::make_cylinder(t, 11.0, 6.0).map_err(s)?;
    let face = cylinder_face_of(t, blank, 11.0)?;
    let params = GearParams::standard(GearKind::External, 1.0, 20, 6.0);
    timed(|| cut_gear_teeth(t, blank, face, &params, &CylinderGearOptions::default()))
        .map(|(o, d)| (o.solid, d))
}

fn thread_external(t: &mut Topology) -> Built {
    // thread/tests.rs: ISO M6×1 on an r 3 boss, 10 tall (10 turns).
    let blank = primitives::make_cylinder(t, 3.0, 10.0).map_err(s)?;
    let face = cylinder_face_of(t, blank, 3.0)?;
    let form = ThreadForm::iso_metric(1.0, ThreadKind::External);
    timed(|| cut_thread(t, blank, face, &form, &CylinderThreadOptions::default()))
        .map(|(o, d)| (o.solid, d))
}
fn thread_internal(t: &mut Topology) -> Built {
    // thread/tests.rs: ISO M6×1 internal in a bored r 6 tube, 6 tall.
    let form = ThreadForm::iso_metric(1.0, ThreadKind::Internal);
    let minor = 3.0 - form.depth;
    let outer = primitives::make_cylinder(t, 6.0, 6.0).map_err(s)?;
    let hole = moved_cylinder(t, minor, 8.0, -1.0)?;
    let body = boolean(t, BooleanOp::Cut, outer, hole).map_err(s)?;
    let face = cylinder_face_of(t, body, minor)?;
    timed(|| cut_thread(t, body, face, &form, &CylinderThreadOptions::default()))
        .map(|(o, d)| (o.solid, d))
}
fn thread_two_start(t: &mut Topology) -> Built {
    // thread/tests.rs: turned_down_left_hand_two_start_band_keeps_collars.
    let blank = primitives::make_cylinder(t, 5.0, 12.0).map_err(s)?;
    let face = cylinder_face_of(t, blank, 5.0)?;
    let form = ThreadForm {
        starts: 2,
        hand: ThreadHand::Left,
        ..ThreadForm::iso_metric(1.5, ThreadKind::External)
    };
    let options = CylinderThreadOptions {
        major_diameter: Some(9.6),
        offset: 2.0,
        length: Some(7.0),
        ..CylinderThreadOptions::default()
    };
    timed(|| cut_thread(t, blank, face, &form, &options)).map(|(o, d)| (o.solid, d))
}
fn thread_chamfered(t: &mut Topology) -> Built {
    // thread/tests.rs: chamfered_external_ends_taper_into_the_cone.
    let blank = primitives::make_cylinder(t, 3.0, 8.0).map_err(s)?;
    let face = cylinder_face_of(t, blank, 3.0)?;
    let form = ThreadForm::iso_metric(1.0, ThreadKind::External);
    let chamfer = ThreadEnd::Chamfer { angle: PI / 4.0 };
    let options = CylinderThreadOptions {
        start: chamfer,
        end: chamfer,
        ..CylinderThreadOptions::default()
    };
    timed(|| cut_thread(t, blank, face, &form, &options)).map(|(o, d)| (o.solid, d))
}

fn scenarios() -> Vec<Scenario> {
    macro_rules! sc {
        ($name:expr, $defl:expr, $f:expr) => {
            Scenario {
                name: $name,
                deflection: $defl,
                build: $f,
            }
        };
    }
    vec![
        sc!("prim/box 20", 0.05, prim_box),
        sc!("prim/cylinder r10 h20", 0.05, prim_cylinder),
        sc!("prim/cone 10-5 h20", 0.05, prim_cone),
        sc!("prim/sphere r10", 0.05, prim_sphere),
        sc!("prim/torus 15x4", 0.05, prim_torus),
        sc!("extrude/rect", 0.05, extrude_rect),
        sc!("extrude/circle", 0.05, extrude_circle),
        sc!("extrude/rect+hole", 0.05, extrude_rect_hole),
        sc!("extrude/spline square", 0.05, extrude_spline),
        sc!("revolve/rect 360", 0.05, revolve_rect_full),
        sc!("revolve/rect 90", 0.05, revolve_rect_partial),
        sc!("revolve/half-disc 360", 0.05, revolve_arc_profile),
        sc!("revolve/circle 120", 0.05, revolve_circle_partial),
        sc!("loft/coaxial circles", 0.05, loft_coaxial_circles),
        sc!("loft/offset circles", 0.05, loft_offset_circles),
        sc!("loft/3 squares", 0.05, loft_three_squares),
        sc!("loft_smooth/4 squares", 0.05, loft_smooth_four_squares),
        sc!("sweep/square line", 0.05, sweep_square_line),
        sc!("sweep/square arc", 0.05, sweep_square_arc),
        sc!("sweep/circle spline", 0.05, sweep_circle_spline),
        sc!("sweep_smooth/square arc", 0.05, sweep_smooth_square_arc),
        sc!(
            "sweep_smooth/square spline",
            0.05,
            sweep_smooth_square_spline
        ),
        sc!("pipe/square spline", 0.05, pipe_square_spline),
        sc!("pipe/square guided", 0.05, pipe_guided),
        sc!("helix/legacy circle 5t", 0.05, helix_legacy_5),
        sc!("helix/legacy circle 20t", 0.05, helix_legacy_20),
        sc!("helix/in-place circle 5t", 0.05, helix_in_place_5),
        sc!("helix/in-place circle 20t", 0.05, helix_in_place_20),
        sc!("helix/in-place rect 2t", 0.05, helix_in_place_rect),
        sc!("gear/spur z20 m2", 0.05, gear_spur20),
        sc!("gear/spur z60 m2", 0.1, gear_spur60),
        sc!("gear/helical z20 20deg", 0.05, gear_helical),
        sc!("gear/herringbone z20 30deg", 0.05, gear_herringbone),
        sc!("gear/internal z48 m1.5", 0.1, gear_internal),
        sc!("gear/rack z7", 0.05, gear_rack),
        sc!("gear/bevel z20:40", 0.05, gear_bevel),
        sc!("gear/worm 1-start", 0.05, gear_worm),
        sc!("gear/worm wheel plain z30", 0.05, gear_worm_wheel),
        sc!("gear/worm wheel throated z30", 0.05, gear_throated_wheel),
        sc!("gear/enveloping worm", 0.05, gear_enveloping_worm),
        sc!("gear/enveloping wheel", 0.05, gear_enveloping_wheel),
        sc!("gear/cut teeth on boss", 0.05, gear_cut_teeth),
        sc!("thread/M6 external 10t", 0.01, thread_external),
        sc!("thread/M6 internal 6t", 0.01, thread_internal),
        sc!("thread/2-start LH band", 0.01, thread_two_start),
        sc!("thread/M6 chamfered", 0.01, thread_chamfered),
    ]
}

// ── measurement ─────────────────────────────────────────────────────────

struct Sample {
    setup: Duration,
    construct: Duration,
    tess: Duration,
    volume: Duration,
    faces: usize,
    tris: usize,
    vol: f64,
}

fn run_once(sc: &Scenario) -> Result<Sample, String> {
    let mut topo = Topology::new();
    let total = Instant::now();
    let (solid, construct) = (sc.build)(&mut topo)?;
    let setup = total.elapsed().saturating_sub(construct);
    let faces = solid_faces(&topo, solid)
        .map_err(|e| format!("faces: {e}"))?
        .len();
    let t0 = Instant::now();
    let mesh = tessellate::tessellate_solid(&topo, solid, sc.deflection)
        .map_err(|e| format!("tessellate: {e}"))?;
    let tess = t0.elapsed();
    let t0 = Instant::now();
    let vol =
        measure::solid_volume(&topo, solid, sc.deflection).map_err(|e| format!("volume: {e}"))?;
    let volume = t0.elapsed();
    Ok(Sample {
        setup,
        construct,
        tess,
        volume,
        faces,
        tris: mesh.indices.len() / 3,
        vol,
    })
}

fn run_guarded(sc: &Scenario) -> Result<Sample, String> {
    catch_unwind(AssertUnwindSafe(|| run_once(sc))).unwrap_or_else(|payload| {
        let msg = payload
            .downcast_ref::<String>()
            .cloned()
            .or_else(|| payload.downcast_ref::<&str>().map(|m| (*m).to_owned()))
            .unwrap_or_else(|| "unknown".to_owned());
        Err(format!("panic: {msg}"))
    })
}

fn median(mut v: Vec<Duration>) -> Duration {
    v.compact_sort_unstable();
    v[v.len() / 2]
}

fn ms(d: Duration) -> String {
    // Three significant digits.
    let v = d.as_secs_f64() * 1e3;
    if v <= 0.0 {
        return "0".to_owned();
    }
    let digits = v.log10().floor() as i32;
    let scale = 10f64.powi(digits - 2);
    let rounded = (v / scale).round() * scale;
    let decimals = usize::try_from(2 - digits).unwrap_or(0);
    format!("{rounded:.decimals$}")
}

struct Row {
    name: &'static str,
    deflection: f64,
    runs: usize,
    result: Result<Sample, String>,
}

fn print_table(title: &str, rows: &[&Row]) {
    println!("\n== {title} ==");
    println!(
        "{:<32} {:>6} {:>4} {:>10} {:>10} {:>10} {:>10} {:>10} {:>6} {:>8} {:>12}",
        "scenario",
        "defl",
        "runs",
        "setup ms",
        "build ms",
        "tess ms",
        "volume ms",
        "b+t ms",
        "faces",
        "tris",
        "volume"
    );
    for row in rows {
        match &row.result {
            Ok(m) => println!(
                "{:<32} {:>6} {:>4} {:>10} {:>10} {:>10} {:>10} {:>10} {:>6} {:>8} {:>12.4}",
                row.name,
                row.deflection,
                row.runs,
                ms(m.setup),
                ms(m.construct),
                ms(m.tess),
                ms(m.volume),
                ms(m.construct + m.tess),
                m.faces,
                m.tris,
                m.vol
            ),
            Err(e) => println!(
                "{:<32} {:>6} {:>4} ERROR: {e}",
                row.name, row.deflection, row.runs
            ),
        }
    }
}

fn main() {
    let mut filter: Option<String> = None;
    let mut default_runs = 5_usize;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--runs" {
            default_runs = args
                .next()
                .and_then(|n| n.parse().ok())
                .unwrap_or(default_runs)
                .max(1);
        } else if filter.is_none() {
            filter = Some(arg);
        }
    }

    let mut rows = Vec::new();
    for sc in scenarios() {
        if filter.as_deref().is_some_and(|f| !sc.name.contains(f)) {
            continue;
        }
        eprint!("{:<32} ", sc.name);
        let first = run_guarded(&sc);
        let row = match first {
            Err(e) => {
                eprintln!("error");
                Row {
                    name: sc.name,
                    deflection: sc.deflection,
                    runs: 1,
                    result: Err(e),
                }
            }
            Ok(first) => {
                let one = first.setup + first.construct + first.tess + first.volume;
                let runs = if one > Duration::from_secs(10) {
                    1
                } else if one > Duration::from_secs(1) {
                    default_runs.min(3)
                } else {
                    default_runs
                };
                let mut samples = vec![first];
                let mut error = None;
                for _ in 1..runs {
                    match run_guarded(&sc) {
                        Ok(sample) => samples.push(sample),
                        Err(e) => {
                            error = Some(e);
                            break;
                        }
                    }
                }
                eprintln!("{} run(s)", samples.len());
                let pick = |f: fn(&Sample) -> Duration| median(samples.iter().map(f).collect());
                let result = match error {
                    Some(e) => Err(format!("later run failed: {e}")),
                    None => Ok(Sample {
                        setup: pick(|x| x.setup),
                        construct: pick(|x| x.construct),
                        tess: pick(|x| x.tess),
                        volume: pick(|x| x.volume),
                        faces: samples[0].faces,
                        tris: samples[0].tris,
                        vol: samples[0].vol,
                    }),
                };
                Row {
                    name: sc.name,
                    deflection: sc.deflection,
                    runs: samples.len(),
                    result,
                }
            }
        };
        rows.push(row);
    }

    // Errors sort first so they are not missed.
    let key = |row: &Row, f: fn(&Sample) -> Duration| row.result.as_ref().map_or(Duration::MAX, f);
    let mut by_build: Vec<&Row> = rows.iter().collect();
    by_build.compact_sort_by(|a, b| key(b, |m| m.construct).cmp(&key(a, |m| m.construct)));
    print_table("sorted by construction (median ms)", &by_build);
    let mut by_tess: Vec<&Row> = rows.iter().collect();
    by_tess.compact_sort_by(|a, b| key(b, |m| m.tess).cmp(&key(a, |m| m.tess)));
    print_table("sorted by tessellation (median ms)", &by_tess);
}
