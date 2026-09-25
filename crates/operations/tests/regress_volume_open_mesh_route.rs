//! B54: OpenZCAD's growing-holder recipe measures every width on one route.
//!
//! Native port of `test/growing-holder-recipe.test.ts` (OpenZCAD, 2026-09-24):
//! a 60 × 32 × 20 U-bracket (8 mm floor and arms) with a Ø5 through hole in
//! each arm under a Ø9 × 90° countersink, and a 0.4 mm boss on one arm's
//! inner face, exported to STEP in one of three placements and re-imported.
//! The recipe keeps each end as the intersection of an import with a box mask
//! beyond the cuts at 12 and 48, moves the ends apart symmetrically about 30,
//! rebuilds the 8 × 20 floor section between them as an extrusion, and fuses
//! the three. Growing the opening from 44 changes the volume by exactly the
//! floor section times the change: 160 mm² × (width − 44).
//!
//! Two defects made width 10 read −5411.3547 (x) / −5411.3544 (y) instead of
//! −5440 on kernel `bdb44304`:
//!
//! 1. Tessellation: the floor cap and the arm's inner face are both planes
//!    with holes, so both are triangulated up front as planar CDT jobs. At
//!    width 10 the cap's constraint recovery Steiner-split its edge with the
//!    arm face at the midpoint; the splice that shares such a point only
//!    reached faces tessellated after the jobs, so the arm face still spanned
//!    the edge in one triangle — three open mesh edges at deflections 0.01 and
//!    0.001 (x and y placements; the z placement never split).
//! 2. Measurement: the countersink cones are trimmed by the arms' side faces
//!    (hyperbolas stored as NURBS), so `solid_volume` sends the body to its
//!    closed whole-solid mesh (B32). The open mesh silently fell through to
//!    the analytic bounding rectangle the body was routed away from, 28.9 mm³
//!    heavy, while the neighbouring widths measured on the closed mesh.
//!
//! Oracles are independent of both fixed routes: the closed-form volume
//! (prism, bores, countersink frusta clipped to the 8 mm arms, boss) and the
//! exact width-to-width difference.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::step::reader::read_step;
use remus_io::step::writer::write_step;
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::copy::copy_and_transform_solid;
use remus_operations::measure::{mass_properties, solid_bounding_box, solid_volume};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const TOL: f64 = 1e-7;
/// The recipe's source opening between the arms' inner faces.
const OPENING: f64 = 44.0;
/// Cut planes along the opening axis; the section between them is rebuilt.
const CUTS: [f64; 2] = [12.0, 48.0];
/// The rebuilt floor section, 8 × 20 mm.
const FLOOR_AREA: f64 = 8.0 * 20.0;
/// The widths the OpenZCAD test walks, in its order.
const WIDTHS: [f64; 4] = [OPENING, 60.0, 30.0, 10.0];

fn translated(topo: &mut Topology, s: SolidId, x: f64, y: f64, z: f64) -> SolidId {
    copy_and_transform_solid(topo, s, &Mat4::translation(x, y, z)).expect("translate")
}

/// OpenZCAD's `drillHole` with `style: 'countersink'`: one revolved radial
/// section placed in the adapter's cylinder frame, cut exactly, then unified
/// under the strict gate.
fn countersink(topo: &mut Topology, holder: SolidId, x: f64) -> SolidId {
    let (radius, depth, entry, exit) = (2.5_f64, 20.0_f64, 0.2_f64, 0.2_f64);
    let (sink_radius, sink_angle) = (4.5_f64, std::f64::consts::FRAC_PI_2);
    let half_tangent = (sink_angle / 2.0).tan();
    let sink_depth = (sink_radius - radius) / half_tangent;
    let total = entry + depth + exit;
    let section: Vec<Point3> = [
        (0.0, 0.0),
        (entry.mul_add(half_tangent, sink_radius), 0.0),
        (radius, entry + sink_depth),
        (radius, total),
        (0.0, total),
    ]
    .iter()
    .map(|&(r, a)| Point3::new(r, 0.0, a))
    .collect();
    let wire = remus_topology::builder::make_polygon_wire(topo, &section, TOL).expect("wire");
    let face = remus_topology::builder::make_planar_face_from_wire(topo, wire).expect("face");
    let local = remus_operations::revolve::revolve(
        topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::TAU,
    )
    .expect("revolve");
    // `coordinateFrameMatrix(origin = (x, 19, 20.2), zAxis = (0, 0, -1))`.
    let frame = Mat4([
        [0.0, 1.0, 0.0, x],
        [1.0, 0.0, 0.0, 19.0],
        [0.0, 0.0, -1.0, 20.0 + entry],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    let tool = copy_and_transform_solid(topo, local, &frame).expect("place tool");
    let cut = boolean(topo, BooleanOp::Cut, holder, tool).expect("drill");
    let report = remus_operations::heal::unify_faces_checked(topo, cut).expect("unify");
    assert_eq!(
        report.result_errors, 0,
        "drilled holder must stay strict-valid"
    );
    cut
}

/// `syntheticHolderSolid` from OpenZCAD's `test/support/synthetic-holder.ts`.
fn synthetic_holder(topo: &mut Topology) -> SolidId {
    let side: Vec<Point3> = [
        (0.0, 0.0),
        (60.0, 0.0),
        (60.0, 32.0),
        (52.0, 32.0),
        (52.0, 8.0),
        (8.0, 8.0),
        (8.0, 32.0),
        (0.0, 32.0),
    ]
    .iter()
    .map(|&(x, y)| Point3::new(x, y, 0.0))
    .collect();
    let face = remus_topology::builder::make_planar_face(topo, &side, TOL).expect("profile");
    let mut holder = remus_operations::extrude::extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), 20.0)
        .expect("extrude");
    for x in [4.0, 56.0] {
        holder = countersink(topo, holder, x);
    }
    let boss = remus_operations::primitives::make_box(topo, 0.4, 6.0, 4.0).expect("boss");
    let emboss = translated(topo, boss, 8.0, 14.0, 7.0);
    boolean(topo, BooleanOp::Fuse, holder, emboss).expect("emboss")
}

/// One of the test's placements: the opening axis, the row-major placement
/// matrix, and the measured source envelope.
struct Placement {
    axis: usize,
    matrix: Mat4,
    envelope: ([f64; 3], [f64; 3]),
}

fn placement(axis: usize) -> Placement {
    match axis {
        0 => Placement {
            axis,
            matrix: Mat4::identity(),
            envelope: ([-0.5, 0.0, 0.0], [60.5, 32.0, 20.0]),
        },
        // Rotate +90° about z: (x, y, z) → (−y, x, z).
        1 => Placement {
            axis,
            matrix: Mat4([
                [0.0, -1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
            envelope: ([-32.0, -0.5, 0.0], [0.0, 60.5, 20.0]),
        },
        // Rotate −90° about y: (x, y, z) → (−z, y, x).
        _ => Placement {
            axis,
            matrix: Mat4([
                [0.0, 0.0, -1.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
            envelope: ([-20.0, 0.0, -0.5], [0.0, 32.0, 60.5]),
        },
    }
}

/// The placed holder as the STEP text OpenZCAD imports.
fn source_step(p: &Placement) -> String {
    let mut topo = Topology::new();
    let holder = synthetic_holder(&mut topo);
    let placed = copy_and_transform_solid(&mut topo, holder, &p.matrix).expect("place");
    write_step(&topo, &[placed]).expect("export")
}

/// `growingHolderPlan`'s box mask keeping one end: the envelope grown by
/// `1 + 5 %` of its largest extent, cut at the recipe plane, built as a box
/// primitive moved to its min corner.
fn mask(topo: &mut Topology, p: &Placement, negative: bool) -> SolidId {
    let tidy = |v: f64| (v * 1e9).round() / 1e9;
    let (min, max) = p.envelope;
    let extent = (0..3).map(|i| max[i] - min[i]).fold(0.0, f64::max);
    let margin = 0.05f64.mul_add(extent, 1.0);
    let mut lo = [0.0; 3];
    let mut hi = [0.0; 3];
    for i in 0..3 {
        lo[i] = tidy(min[i] - margin);
        hi[i] = tidy(max[i] + margin);
    }
    if negative {
        hi[p.axis] = CUTS[0];
    } else {
        lo[p.axis] = CUTS[1];
    }
    let b = remus_operations::primitives::make_box(
        topo,
        tidy(hi[0] - lo[0]),
        tidy(hi[1] - lo[1]),
        tidy(hi[2] - lo[2]),
    )
    .expect("mask box");
    translated(topo, b, lo[0], lo[1], lo[2])
}

/// The width bridge: the recipe's section sketched on the plane across the
/// axis at the moved negative cut, extruded along the axis.
fn bridge(topo: &mut Topology, p: &Placement, width: f64) -> SolidId {
    let offset = CUTS[0] + (OPENING - width) / 2.0;
    let length = CUTS[1] - CUTS[0] + width - OPENING;
    let corners: Vec<Point3> = match p.axis {
        // YZ sketch plane (u = y, v = z), rect(0, 0, 8, 20).
        0 => [(0.0, 0.0), (8.0, 0.0), (8.0, 20.0), (0.0, 20.0)]
            .iter()
            .map(|&(u, v)| Point3::new(offset, u, v))
            .collect(),
        // XZ sketch plane (u = x, v = −z), rect(−8, −20, 0, 0).
        1 => [(-8.0, -20.0), (0.0, -20.0), (0.0, 0.0), (-8.0, 0.0)]
            .iter()
            .map(|&(u, v)| Point3::new(u, offset, -v))
            .collect(),
        // XY sketch plane (u = x, v = y), rect(−20, 0, 0, 8).
        _ => [(-20.0, 0.0), (0.0, 0.0), (0.0, 8.0), (-20.0, 8.0)]
            .iter()
            .map(|&(u, v)| Point3::new(u, v, offset))
            .collect(),
    };
    let face = remus_topology::builder::make_planar_face(topo, &corners, TOL).expect("section");
    let mut d = [0.0; 3];
    d[p.axis] = 1.0;
    remus_operations::extrude::extrude(topo, face, Vec3::new(d[0], d[1], d[2]), length)
        .expect("bridge")
}

/// The recipe at one width, as OpenZCAD's build loop runs it: each end from
/// its own import intersected with its mask and unified, moved; the bridge;
/// one `fuseAll`; then the union gate — unify a copy and keep it only when it
/// is strict-valid and its display mesh is closed.
fn build(step: &str, p: &Placement, width: f64) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let piece = |topo: &mut Topology, negative: bool| {
        let source = read_step(step, topo).expect("import")[0];
        let m = mask(topo, p, negative);
        let carved = boolean(topo, BooleanOp::Intersect, source, m).expect("carve");
        remus_operations::heal::unify_faces(topo, carved).expect("unify piece");
        let shift = if negative {
            OPENING - width
        } else {
            width - OPENING
        } / 2.0;
        let mut d = [0.0; 3];
        d[p.axis] = shift;
        translated(topo, carved, d[0], d[1], d[2])
    };
    let negative = piece(&mut topo, true);
    let positive = piece(&mut topo, false);
    let section = bridge(&mut topo, p, width);
    let raw =
        remus_operations::compound_ops::fuse_solids(&mut topo, &[negative, section, positive])
            .expect("union");
    let candidate = copy_and_transform_solid(&mut topo, raw, &Mat4::identity()).expect("copy");
    let report = remus_operations::heal::unify_faces_checked(&mut topo, candidate).expect("unify");
    let closed = tessellate_solid(&topo, candidate, 0.05)
        .is_ok_and(|mesh| boundary_edge_count(&mesh) == 0 && non_manifold_edge_count(&mesh) == 0);
    let solid = if report.result_errors == 0 && closed {
        candidate
    } else {
        raw
    };
    (topo, solid)
}

/// Closed-form volume at `width`: the U prism, minus two Ø5 bores below the
/// countersinks, minus two countersink frusta (r = 2.5 → 4.5 over 2) clipped
/// to the 8 mm arm (|x − 4| ≤ 4) once their radius passes 4, plus the boss,
/// plus the floor section's growth.
fn closed_form(width: f64) -> f64 {
    let a = 4.0_f64;
    // Antiderivative of the clipped disc area 2r²·asin(a/r) + 2a·√(r² − a²).
    let clipped = |r: f64| {
        let s = r.mul_add(r, -a * a).sqrt();
        let l = (r + s).ln();
        (2.0 * r.powi(3) / 3.0).mul_add(
            (a / r).asin(),
            (2.0 * a / 3.0) * (r / 2.0).mul_add(s, a * a / 2.0 * l),
        ) + a * r.mul_add(s, -a * a * l)
    };
    // The adapter's half-angle tangent, tan(π/4) in floating point.
    let half_tangent = (std::f64::consts::FRAC_PI_2 / 2.0).tan();
    let sink = (std::f64::consts::PI * (a.powi(3) - 2.5_f64.powi(3)) / 3.0 + clipped(4.5)
        - clipped(a))
        * half_tangent;
    let bore = std::f64::consts::PI * 2.5 * 2.5 * (20.0 - 2.0 / half_tangent);
    let prism = (60.0 * 32.0 - 44.0 * 24.0) * 20.0;
    let boss = 0.4 * 6.0 * 4.0;
    FLOOR_AREA.mul_add(width - OPENING, prism - 2.0 * (bore + sink) + boss)
}

fn assert_watertight(topo: &Topology, solid: SolidId, deflection: f64, what: &str) {
    let mesh = tessellate_solid(topo, solid, deflection).expect("tessellate");
    assert_eq!(
        boundary_edge_count(&mesh),
        0,
        "{what}: open mesh at deflection {deflection}"
    );
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "{what}: non-manifold mesh at deflection {deflection}"
    );
}

/// The countersunk-bracket census: 21 faces, the two Ø5 bores centred 4 mm
/// outside the moved inner faces, and both countersink cones.
fn assert_census(topo: &Topology, solid: SolidId, p: &Placement, width: f64, what: &str) {
    let faces = remus_topology::explorer::solid_faces(topo, solid).expect("faces");
    assert_eq!(faces.len(), 21, "{what}: face count (mesh fallback?)");
    let mut bores = Vec::new();
    let mut cones = 0;
    for fid in faces {
        match topo.face(fid).expect("face").surface() {
            FaceSurface::Cylinder(c) if (c.radius() - 2.5).abs() < 1e-7 => {
                let o = c.origin();
                bores.push([o.x(), o.y(), o.z()][p.axis]);
            }
            FaceSurface::Cone(_) => cones += 1,
            _ => {}
        }
    }
    bores.sort_by(f64::total_cmp);
    assert_eq!(bores.len(), 2, "{what}: bores {bores:?}");
    assert_eq!(cones, 2, "{what}: countersink cones");
    assert!(
        (bores[0] - (30.0 - width / 2.0 - 4.0)).abs() < 1e-6,
        "{what}: {bores:?}"
    );
    assert!(
        (bores[1] - (30.0 + width / 2.0 + 4.0)).abs() < 1e-6,
        "{what}: {bores:?}"
    );
}

fn growing_holder_measures_every_width_on_one_route(axis: usize) {
    let p = placement(axis);
    let step = source_step(&p);
    let mut baseline = None;
    for width in WIDTHS {
        let what = format!("axis {axis}, width {width}");
        let (topo, solid) = build(&step, &p, width);
        let report = remus_operations::validate::validate_solid(&topo, solid).expect("validate");
        assert!(report.is_valid(), "{what}: {:?}", report.issues);
        assert_census(&topo, solid, &p, width, &what);

        let exact = closed_form(width);
        // Gauss's residual is the trim outlines' chord error on the two
        // cones: 0.0086 mm³ here, under 1e-6 relative.
        let gauss = mass_properties(&topo, solid).expect("mass").mass.abs();
        assert!(
            (gauss - exact).abs() <= 1e-5 * exact,
            "{what}: Gauss {gauss} vs closed form {exact}"
        );

        // Closed at the display deflection the OpenZCAD test checks, at every
        // deflection it measures with, and at the clamped deflection
        // `solid_volume` actually tessellates at.
        let aabb = solid_bounding_box(&topo, solid).expect("bbox");
        let clamped = (aabb.max - aabb.min).length() * 5e-5;
        for deflection in [0.1, 0.08, 0.05, 0.01, clamped, 0.001] {
            assert_watertight(&topo, solid, deflection, &what);
        }

        // Every deflection measures within the mesh's chord budget of the
        // closed form (measured ≤ 6e-5 relative): the 28.9 mm³ rectangle
        // error is 2.6e-3 at width 10.
        for deflection in [0.1, 0.08, 0.01, 0.001] {
            let volume = solid_volume(&topo, solid, deflection).expect("volume");
            assert!(
                (volume - exact).abs() <= 1e-4 * exact,
                "{what}: volume {volume} at {deflection} vs closed form {exact}"
            );
        }

        // The OpenZCAD assertion: at 0.001 the width-to-width change is the
        // floor section's growth to three decimals.
        let fine = solid_volume(&topo, solid, 0.001).expect("volume");
        let base = *baseline.get_or_insert(fine);
        let expected = FLOOR_AREA * (width - OPENING);
        assert!(
            (fine - base - expected).abs() <= 1e-3,
            "{what}: change {} vs {expected}",
            fine - base
        );

        // Rigid translation moves neither the mesh route nor the result.
        let mut moved = topo.clone();
        remus_operations::transform::transform_solid(
            &mut moved,
            solid,
            &Mat4::translation(13.0, -7.0, 5.0),
        )
        .expect("translate");
        assert_watertight(&moved, solid, 0.001, &what);
        let shifted = solid_volume(&moved, solid, 0.001).expect("volume");
        assert!(
            (shifted - fine).abs() <= 1e-6 * exact,
            "{what}: volume moved {fine} -> {shifted} under translation"
        );
    }
}

#[test]
fn growing_holder_x_measures_every_width_on_one_route() {
    growing_holder_measures_every_width_on_one_route(0);
}

#[test]
fn growing_holder_y_measures_every_width_on_one_route() {
    growing_holder_measures_every_width_on_one_route(1);
}

#[test]
fn growing_holder_z_measures_every_width_on_one_route() {
    growing_holder_measures_every_width_on_one_route(2);
}
