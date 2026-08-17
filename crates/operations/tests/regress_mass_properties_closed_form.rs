//! `mass_properties` must report the body, not the decomposition.
//!
//! It sums `remus_check::properties::face_integrator::integrate_face` over a
//! solid's faces. That integrator measures a line-and-arc-bounded planar face
//! in closed form (Green's theorem on the boundary) and falls back to a chord
//! polygon otherwise — but it decided which by sampling the boundary's plane
//! with `edge.start()` for every line edge, ignoring the direction the wire
//! traverses it. A loop whose stored flags alternate, which is what a boolean
//! leaves behind, sampled to a collapsed sequence with a zero Newell normal,
//! and the closed form was then rejected for the WHOLE face, holes included.
//!
//! On the OpenZCAD demo bracket that hit the y = 32 wall face: its r = 10 boss
//! bore was subtracted as the 32-gon inscribed in it, keeping 2.0147 mm² of
//! bore as material. The face contributed −23550.459 mm³ instead of
//! −23528.968, and the face-unified body read 47339.449 against a true
//! 47360.940 — 21.49 mm³, 0.045 % light. The same design left un-unified read
//! 47467.358 against its own true 47484.551 — 17.19 mm³, 0.036 % light — so
//! the two decompositions of one design also disagreed with each other.
//!
//! Every expected value here is a closed form composed from the same dimension
//! constants the model is built from, via signed raw moments about the global
//! origin. Nothing is a recorded measurement.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::solid::SolidId;

// ---------------------------------------------------------------------------
// Closed forms: signed raw moments about the global origin
// ---------------------------------------------------------------------------

/// Signed volume moments of a region about the GLOBAL origin.
///
/// Composing a body is then addition, and removing a feature is adding it with
/// `sign = -1`. `finish` converts to the same `(volume, centre, inertia)`
/// convention [`remus_check::properties::GProps`] uses, including its habit
/// of storing the products of inertia as `∫xy dV` (positive) and letting
/// `matrix_of_inertia` negate them.
#[derive(Clone, Copy, Default)]
struct Moments {
    v: f64,
    sx: f64,
    sy: f64,
    sz: f64,
    qxx: f64,
    qyy: f64,
    qzz: f64,
    qxy: f64,
    qxz: f64,
    qyz: f64,
}

impl Moments {
    fn sum(parts: &[Self]) -> Self {
        let mut t = Self::default();
        for p in parts {
            t.v += p.v;
            t.sx += p.sx;
            t.sy += p.sy;
            t.sz += p.sz;
            t.qxx += p.qxx;
            t.qyy += p.qyy;
            t.qzz += p.qzz;
            t.qxy += p.qxy;
            t.qxz += p.qxz;
            t.qyz += p.qyz;
        }
        t
    }

    fn finish(self) -> (f64, Point3, [f64; 6]) {
        let (cx, cy, cz) = (self.sx / self.v, self.sy / self.v, self.sz / self.v);
        let xx = self.qxx - self.v * cx * cx;
        let yy = self.qyy - self.v * cy * cy;
        let zz = self.qzz - self.v * cz * cz;
        (
            self.v,
            Point3::new(cx, cy, cz),
            [
                yy + zz,
                xx + zz,
                xx + yy,
                self.qxy - self.v * cx * cy,
                self.qxz - self.v * cx * cz,
                self.qyz - self.v * cy * cz,
            ],
        )
    }
}

/// Moments of an axis-aligned cuboid `(dx, dy, dz)` centred at `c`.
///
/// About its own centroid a cuboid has `∫(x−cx)² dV = V·dx²/12` and no
/// products, so the global second moments are the parallel-axis shift of that.
fn cuboid(dx: f64, dy: f64, dz: f64, c: Point3, sign: f64) -> Moments {
    let v = sign * dx * dy * dz;
    second_moments(v, c, [dx * dx / 12.0, dy * dy / 12.0, dz * dz / 12.0])
}

/// Which global axis a cylinder's axis runs along.
#[derive(Clone, Copy)]
enum Axis {
    Y,
    Z,
}

/// Moments of a right circular cylinder of radius `r` and length `h` centred
/// at `c` with its axis along `axis`.
///
/// About its own centroid: `r²/4` across the axis, `h²/12` along it.
fn cylinder(r: f64, h: f64, c: Point3, axis: Axis, sign: f64) -> Moments {
    let v = sign * PI * r * r * h;
    let (across, along) = (r * r / 4.0, h * h / 12.0);
    let own = match axis {
        Axis::Y => [across, along, across],
        Axis::Z => [across, across, along],
    };
    second_moments(v, c, own)
}

/// Assemble global moments from a signed volume, a centroid, and the region's
/// second moments about its own centroid (products assumed zero — true for
/// every primitive used here).
fn second_moments(v: f64, c: Point3, own: [f64; 3]) -> Moments {
    Moments {
        v,
        sx: v * c.x(),
        sy: v * c.y(),
        sz: v * c.z(),
        qxx: v * (c.x() * c.x() + own[0]),
        qyy: v * (c.y() * c.y() + own[1]),
        qzz: v * (c.z() * c.z() + own[2]),
        qxy: v * c.x() * c.y(),
        qxz: v * c.x() * c.z(),
        qyz: v * c.y() * c.z(),
    }
}

/// Moments of the material a radius-`r` fillet removes from a vertical corner
/// at `corner = (x0, y0)` running `z ∈ (z0, z1)`, entered with `sign = -1`.
///
/// `inward = (ux, uy)` is the pair of signs, each ±1, pointing from the corner
/// into the body.
///
/// The cross-section is the square `[0, r]²` at the corner less the quarter
/// disc of radius `r` centred at its far corner — the sliver between a right
/// angle and the arc that replaces it. In local coordinates `x', y' ≥ 0`
/// measured inward from the corner (so `x = x0 + ux·x'`, `y = y0 + uy·y'`),
/// integrating the square and subtracting the quarter disc gives
///
/// ```text
/// ∫dA    = r²(1 − π/4)
/// ∫x' dA = r³(5/6 − π/4)
/// ∫x'²dA = r⁴(1 − 5π/16)
/// ∫x'y'dA= r⁴(19/24 − π/4)
/// ```
///
/// with `∫y' dA = ∫x' dA` and `∫y'² dA = ∫x'² dA` by the diagonal symmetry.
fn corner_blend_prism(
    r: f64,
    corner: (f64, f64),
    inward: (f64, f64),
    z: (f64, f64),
    sign: f64,
) -> Moments {
    let (x0, y0) = corner;
    let (ux, uy) = inward;
    let (z0, z1) = z;
    let a = r * r * (1.0 - PI / 4.0);
    let s = r.powi(3) * (5.0 / 6.0 - PI / 4.0);
    let i2 = r.powi(4) * (1.0 - 5.0 * PI / 16.0);
    let ic = r.powi(4) * (19.0 / 24.0 - PI / 4.0);

    let ax = a * x0 + ux * s;
    let ay = a * y0 + uy * s;
    let axx = a * x0 * x0 + 2.0 * ux * x0 * s + i2;
    let ayy = a * y0 * y0 + 2.0 * uy * y0 * s + i2;
    let axy = a * x0 * y0 + ux * y0 * s + uy * x0 * s + ux * uy * ic;

    let h = z1 - z0;
    let zc = f64::midpoint(z0, z1);
    Moments {
        v: sign * a * h,
        sx: sign * ax * h,
        sy: sign * ay * h,
        sz: sign * a * h * zc,
        qxx: sign * axx * h,
        qyy: sign * ayy * h,
        qzz: sign * a * h * (zc * zc + h * h / 12.0),
        qxy: sign * axy * h,
        qxz: sign * ax * h * zc,
        qyz: sign * ay * h * zc,
    }
}

// ---------------------------------------------------------------------------
// Assertions
// ---------------------------------------------------------------------------

/// Relative tolerance for an all-analytic body. The planar faces are integrated
/// in closed form and the quadric ones by Gauss quadrature that is converged to
/// machine precision at these orders, so anything above ~1e-12 is a real error,
/// not quadrature noise.
const REL: f64 = 1e-10;

fn assert_rel(actual: f64, expected: f64, scale: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= REL * scale.abs(),
        "{what}: expected the closed form {expected:.9}, got {actual:.9} \
         ({:+.9}, {:+.6} % of {scale:.6})",
        actual - expected,
        100.0 * (actual - expected) / scale
    );
}

/// Assert volume, centre of mass, and the full inertia tensor against a
/// closed-form composition.
fn assert_properties(topo: &Topology, solid: SolidId, expected: &Moments, what: &str) {
    let (v, c, i) = expected.finish();
    let props = mass_properties(topo, solid).unwrap();

    assert_rel(props.mass, v, v, &format!("{what} volume"));

    // The centre is scaled by the body's own extent so the tolerance means the
    // same thing on every axis.
    let extent = i[2].max(i[0]).max(i[1]) / v;
    let extent = extent.sqrt().max(1.0);
    assert_rel(props.center.x(), c.x(), extent, &format!("{what} CoM x"));
    assert_rel(props.center.y(), c.y(), extent, &format!("{what} CoM y"));
    assert_rel(props.center.z(), c.z(), extent, &format!("{what} CoM z"));

    // Products of inertia can be legitimately zero, so scale every component by
    // the largest diagonal moment rather than by itself.
    let iscale = i[0].abs().max(i[1].abs()).max(i[2].abs());
    for (k, name) in ["Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"]
        .iter()
        .enumerate()
    {
        assert_rel(props.inertia[k], i[k], iscale, &format!("{what} {name}"));
    }
}

/// A measurement is a property of the body, not of the quadrature order the
/// caller configured, nor of the preview quality asked of the other integrator.
///
/// `mass_properties` exposes no deflection knob — it integrates exact face
/// geometry — so the analogue is the Gauss order, swept here across the whole
/// useful range. The deflection-independence statement is made against
/// `solid_volume`, which does take one: the two independent integrators must
/// land on the same number at every deflection.
fn assert_measurement_is_not_a_setting(topo: &Topology, solid: SolidId, what: &str) {
    let reference = mass_properties(topo, solid).unwrap().mass;

    for order in [4_usize, 5, 6, 8, 10, 12, 16] {
        let options = remus_check::properties::PropertiesOptions {
            gauss_order: order,
            ..Default::default()
        };
        let v = remus_check::properties::solid_volume(topo, solid, &options).unwrap();
        assert!(
            (v - reference).abs() <= 1e-9 * reference.abs(),
            "{what}: mass_properties' volume depends on Gauss order — \
             {reference} vs {v} at order {order}"
        );
    }

    for deflection in [1.0, 0.5, 0.1, 0.01, 1e-4, 1e-6] {
        let v = solid_volume(topo, solid, deflection).unwrap();
        assert!(
            (v - reference).abs() <= 1e-9 * reference.abs(),
            "{what}: mass_properties disagrees with solid_volume at deflection \
             {deflection} — {reference} vs {v}"
        );
    }
}

// ---------------------------------------------------------------------------
// The OpenZCAD demo bracket (the reported body)
// ---------------------------------------------------------------------------

const W: f64 = 80.0; // width
const D: f64 = 40.0; // depth
const PT: f64 = 8.0; // plate thickness
const WH: f64 = 32.0; // wall height
const SEAT: f64 = 0.5; // how far the wall is seated into the base plate
const BOSS_R: f64 = 10.0;
const BOSS_H: f64 = PT + 4.0;
const HOLE_R: f64 = 4.0;
const MOUNT_R: f64 = 3.0;
const MOUNT_INSET: f64 = 16.0;
const FILLET_R: f64 = 3.0;

/// OpenZCAD's `transformMatrix` for a 90°-about-x placement: (x, y, z) → (x, −z, y) + t.
fn rot_x90_translate(tx: f64, ty: f64, tz: f64) -> Mat4 {
    Mat4::translation(tx, ty, tz) * Mat4::rotation_x(std::f64::consts::FRAC_PI_2)
}

/// Whether to merge coplanar faces after every boolean, as the OpenZCAD
/// adapter does. `unify_faces` does not change the body, only how many faces
/// it is carved into, so a measurement must not notice.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Unify {
    Yes,
    No,
}

/// The demo's Rev B body: base plate + seated wall + boss, less a bore and two
/// mount holes.
fn build_bracket_rev_b_with(topo: &mut Topology, unify_faces: Unify) -> SolidId {
    let unify = |topo: &mut Topology, s: SolidId| {
        if unify_faces == Unify::Yes {
            remus_operations::heal::unify_faces(topo, s).unwrap();
        }
        s
    };

    let base = make_box(topo, W, D, PT).unwrap();
    let wall = make_box(topo, W, PT, WH).unwrap();
    transform_solid(topo, wall, &Mat4::translation(0.0, D - PT, PT - SEAT)).unwrap();
    let fused = boolean(topo, BooleanOp::Fuse, base, wall).unwrap();
    let l_blank = unify(topo, fused);

    let boss = make_cylinder(topo, BOSS_R, BOSS_H).unwrap();
    transform_solid(
        topo,
        boss,
        &rot_x90_translate(W / 2.0, D - PT + 2.0, PT + WH / 2.0),
    )
    .unwrap();
    let fused = boolean(topo, BooleanOp::Fuse, l_blank, boss).unwrap();
    let with_boss = unify(topo, fused);

    let bore = make_cylinder(topo, HOLE_R, WH + 16.0).unwrap();
    transform_solid(
        topo,
        bore,
        &rot_x90_translate(W / 2.0, D + 8.0, PT + WH / 2.0),
    )
    .unwrap();
    let cut = boolean(topo, BooleanOp::Cut, with_boss, bore).unwrap();
    let bored = unify(topo, cut);

    let mount_a = make_cylinder(topo, MOUNT_R, PT + 4.0).unwrap();
    transform_solid(
        topo,
        mount_a,
        &Mat4::translation(MOUNT_INSET, D / 2.0, -2.0),
    )
    .unwrap();
    let cut = boolean(topo, BooleanOp::Cut, bored, mount_a).unwrap();
    let one_hole = unify(topo, cut);

    let mount_b = make_cylinder(topo, MOUNT_R, PT + 4.0).unwrap();
    transform_solid(
        topo,
        mount_b,
        &Mat4::translation(W - MOUNT_INSET, D / 2.0, -2.0),
    )
    .unwrap();
    let cut = boolean(topo, BooleanOp::Cut, one_hole, mount_b).unwrap();
    unify(topo, cut)
}

/// The face-unified Rev B body, as the OpenZCAD adapter builds it.
fn build_bracket_rev_b(topo: &mut Topology) -> SolidId {
    build_bracket_rev_b_with(topo, Unify::Yes)
}

/// Rev B as the primitives that make it: base plate ∪ wall (their 0.5 mm seat
/// counted once), plus the part of the boss standing proud of the wall, less
/// the bore and the two mount holes.
///
/// The bore runs from the boss's free face at y = 22 out through the back at
/// y = 40 as one continuous 18 mm hole; the boss's buried half (y ∈ [32, 34])
/// is inside the wall and is therefore not a separate term.
fn bracket_rev_b_moments() -> Vec<Moments> {
    let wall_front = D - PT; //             y = 32
    let boss_front = D - PT + 2.0 - BOSS_H; // y = 22
    let boss_proud = wall_front - boss_front; // 10 mm of boss stands out
    let bore_len = D - boss_front; //          18 mm of bore
    vec![
        cuboid(W, D, PT, Point3::new(W / 2.0, D / 2.0, PT / 2.0), 1.0),
        cuboid(
            W,
            PT,
            WH,
            Point3::new(W / 2.0, D - PT / 2.0, PT - SEAT + WH / 2.0),
            1.0,
        ),
        // The seat: the slab where plate and wall overlap, counted once.
        cuboid(
            W,
            PT,
            SEAT,
            Point3::new(W / 2.0, D - PT / 2.0, PT - SEAT / 2.0),
            -1.0,
        ),
        cylinder(
            BOSS_R,
            boss_proud,
            Point3::new(W / 2.0, boss_front + boss_proud / 2.0, PT + WH / 2.0),
            Axis::Y,
            1.0,
        ),
        cylinder(
            HOLE_R,
            bore_len,
            Point3::new(W / 2.0, boss_front + bore_len / 2.0, PT + WH / 2.0),
            Axis::Y,
            -1.0,
        ),
        cylinder(
            MOUNT_R,
            PT,
            Point3::new(MOUNT_INSET, D / 2.0, PT / 2.0),
            Axis::Z,
            -1.0,
        ),
        cylinder(
            MOUNT_R,
            PT,
            Point3::new(W - MOUNT_INSET, D / 2.0, PT / 2.0),
            Axis::Z,
            -1.0,
        ),
    ]
}

/// Rev C: Rev B with the four vertical corner edges of the base plate filleted.
/// Each blend runs the full height of the corner it rounds — the two front ones
/// are 8 mm of base plate, the two rear ones continue up the wall to z = 39.5.
fn bracket_rev_c_moments() -> Vec<Moments> {
    let rear_top = PT + WH - SEAT;
    let mut parts = bracket_rev_b_moments();
    // Front pair: base plate only. Rear pair: on up the wall.
    for (corner, inward, top) in [
        ((0.0, 0.0), (1.0, 1.0), PT),
        ((W, 0.0), (-1.0, 1.0), PT),
        ((0.0, D), (1.0, -1.0), rear_top),
        ((W, D), (-1.0, -1.0), rear_top),
    ] {
        parts.push(corner_blend_prism(
            FILLET_R,
            corner,
            inward,
            (0.0, top),
            -1.0,
        ));
    }
    parts
}

/// The demo's Rev C pick: the four vertical corner edges of the base plate.
fn pick_corner_edges(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    let near = |a: f64, b: f64| (a - b).abs() < 0.1;
    remus_topology::explorer::solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&eid| {
            let e = topo.edge(eid).unwrap();
            let a = topo.vertex(e.start()).unwrap().point();
            let b = topo.vertex(e.end()).unwrap().point();
            let corner =
                |x: f64, y: f64| (near(x, 0.0) || near(x, W)) && (near(y, 0.0) || near(y, D));
            corner(a.x(), a.y())
                && corner(b.x(), b.y())
                && (-0.1..=8.1).contains(&a.z())
                && (-0.1..=8.1).contains(&b.z())
                && (a.x() - b.x()).abs() <= 1.5
                && (a.y() - b.y()).abs() <= 1.5
                && (a.z() - b.z()).abs() >= 4.0
        })
        .collect()
}

#[test]
fn demo_bracket_rev_b_mass_properties_match_closed_form() {
    let mut topo = Topology::new();
    let body = build_bracket_rev_b(&mut topo);
    let expected = Moments::sum(&bracket_rev_b_moments());

    assert_properties(&topo, body, &expected, "bracket Rev B");
    assert_measurement_is_not_a_setting(&topo, body, "bracket Rev B");
}

#[test]
fn demo_bracket_rev_c_mass_properties_match_closed_form() {
    let mut topo = Topology::new();
    let body = build_bracket_rev_b(&mut topo);
    let edges = pick_corner_edges(&topo, body);
    assert_eq!(
        edges.len(),
        4,
        "the demo picks four base-plate corner edges"
    );

    let filleted = remus_operations::blend_ops::fillet_v2(&mut topo, body, &edges, FILLET_R)
        .expect("bracket corner fillet")
        .solid;
    let shell = topo.solid(filleted).unwrap().outer_shell();
    remus_topology::validation::validate_shell_closed(topo.shell(shell).unwrap(), &topo)
        .expect("filleted bracket must be watertight");

    let expected = Moments::sum(&bracket_rev_c_moments());
    assert_properties(&topo, filleted, &expected, "bracket Rev C");
    assert_measurement_is_not_a_setting(&topo, filleted, "bracket Rev C");

    // The biased path read 47339.449389 here — 21.49 mm³ heavy, because the
    // y = 32 wall face charged its r = 10 bore as an inscribed 32-gon. Nail
    // that magnitude down so a re-record cannot quietly restore it.
    let v = mass_properties(&topo, filleted).unwrap().mass;
    assert!(
        (v - 47_339.449_389).abs() > 1.0,
        "Rev C volume regressed to the chorded-hole figure: {v}"
    );
}

#[test]
fn demo_bracket_measures_the_same_however_the_model_was_decomposed() {
    // `unify_faces` merges coplanar faces; it does not move a single surface.
    // The unified body carries 13 faces and the raw one 19, and the volume,
    // centre of mass and inertia tensor of the two must be indistinguishable.
    // The biased path read 47522.934 unified against 47527.232 raw — 4.3 mm³
    // apart, with neither equal to the closed form — because how much of the
    // y = 32 wall ended up on a single face with a full-circle hole depended
    // on whether the coplanar merge had run.
    let mut unified_topo = Topology::new();
    let unified = build_bracket_rev_b_with(&mut unified_topo, Unify::Yes);
    let mut raw_topo = Topology::new();
    let raw = build_bracket_rev_b_with(&mut raw_topo, Unify::No);

    let a = mass_properties(&unified_topo, unified).unwrap();
    let b = mass_properties(&raw_topo, raw).unwrap();

    assert!(
        (a.mass - b.mass).abs() <= 1e-9 * a.mass,
        "volume depends on the face decomposition: unified {} vs raw {}",
        a.mass,
        b.mass
    );
    for (axis, x, y) in [
        ("x", a.center.x(), b.center.x()),
        ("y", a.center.y(), b.center.y()),
        ("z", a.center.z(), b.center.z()),
    ] {
        assert!(
            (x - y).abs() <= 1e-9 * W,
            "CoM {axis} depends on the face decomposition: unified {x} vs raw {y}"
        );
    }
    let scale = a.inertia[0]
        .abs()
        .max(a.inertia[1].abs())
        .max(a.inertia[2].abs());
    for k in 0..6 {
        assert!(
            (a.inertia[k] - b.inertia[k]).abs() <= 1e-9 * scale,
            "inertia[{k}] depends on the face decomposition: unified {} vs raw {}",
            a.inertia[k],
            b.inertia[k]
        );
    }

    // And both are the closed form, not merely equal to each other.
    let expected = Moments::sum(&bracket_rev_b_moments());
    assert_properties(&unified_topo, unified, &expected, "bracket Rev B unified");
    assert_properties(&raw_topo, raw, &expected, "bracket Rev B raw");
    assert_measurement_is_not_a_setting(&raw_topo, raw, "bracket Rev B raw");
}

#[test]
fn demo_bracket_centre_of_mass_respects_the_body_symmetry() {
    // Both revisions are mirror-symmetric about x = W/2: every feature is
    // either centred there or paired across it. No integration error that is
    // charged per-face can respect that by accident — the biased path put the
    // centre at x = 40.0181, 18 µm off a plane of symmetry.
    for (label, build_rev_c) in [("Rev B", false), ("Rev C", true)] {
        let mut topo = Topology::new();
        let mut body = build_bracket_rev_b(&mut topo);
        if build_rev_c {
            let edges = pick_corner_edges(&topo, body);
            body = remus_operations::blend_ops::fillet_v2(&mut topo, body, &edges, FILLET_R)
                .expect("bracket corner fillet")
                .solid;
        }
        let props = mass_properties(&topo, body).unwrap();
        assert!(
            (props.center.x() - W / 2.0).abs() < 1e-9,
            "{label}: centre of mass x = {} is off the plane of symmetry x = {}",
            props.center.x(),
            W / 2.0
        );
        // Symmetry also forces the two products of inertia that involve x.
        let scale = props.inertia[0].abs().max(props.inertia[2].abs());
        assert!(
            props.inertia[3].abs() < 1e-9 * scale && props.inertia[4].abs() < 1e-9 * scale,
            "{label}: symmetry forces Ixy = Ixz = 0, got {} and {}",
            props.inertia[3],
            props.inertia[4]
        );
    }
}

// ---------------------------------------------------------------------------
// Shapes whose answers are known analytically
// ---------------------------------------------------------------------------

#[test]
fn box_mass_properties_match_closed_form() {
    let (dx, dy, dz) = (7.0, 11.0, 3.0);
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, dx, dy, dz).unwrap();
    let expected = cuboid(dx, dy, dz, Point3::new(dx / 2.0, dy / 2.0, dz / 2.0), 1.0);
    assert_properties(&topo, solid, &expected, "box");
    assert_measurement_is_not_a_setting(&topo, solid, "box");
}

#[test]
fn cylinder_mass_properties_match_closed_form() {
    let (r, h) = (3.0, 10.0);
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, r, h).unwrap();
    let expected = cylinder(r, h, Point3::new(0.0, 0.0, h / 2.0), Axis::Z, 1.0);
    assert_properties(&topo, solid, &expected, "cylinder");
    assert_measurement_is_not_a_setting(&topo, solid, "cylinder");
}

#[test]
fn box_with_concentric_bore_mass_properties_match_closed_form() {
    // The configuration the bracket's wall face is an instance of: two planar
    // faces each carrying a full-circle hole, plus a reversed cylinder wall.
    // The bore is concentric, so box and cylinder share a centroid and the
    // tensor is a straight subtraction.
    let (dx, dy, dz, r) = (24.0, 16.0, 9.0, 5.0);
    let centre = Point3::new(dx / 2.0, dy / 2.0, dz / 2.0);

    let mut topo = Topology::new();
    let block = make_box(&mut topo, dx, dy, dz).unwrap();
    let drill = make_cylinder(&mut topo, r, dz + 20.0).unwrap();
    transform_solid(
        &mut topo,
        drill,
        &Mat4::translation(dx / 2.0, dy / 2.0, -10.0),
    )
    .unwrap();
    let bored = boolean(&mut topo, BooleanOp::Cut, block, drill).unwrap();
    remus_operations::heal::unify_faces(&mut topo, bored).unwrap();

    let expected = Moments::sum(&[
        cuboid(dx, dy, dz, centre, 1.0),
        cylinder(r, dz, centre, Axis::Z, -1.0),
    ]);
    assert_properties(&topo, bored, &expected, "box with concentric bore");
    assert_measurement_is_not_a_setting(&topo, bored, "box with concentric bore");

    // Spelled out, independent of the composition machinery above.
    let props = mass_properties(&topo, bored).unwrap();
    let vb = dx * dy * dz;
    let vc = PI * r * r * dz;
    assert_rel(props.mass, vb - vc, vb, "box with bore volume");
    let izz = vb / 12.0 * (dx * dx + dy * dy) - vc * r * r / 2.0;
    assert_rel(props.inertia[2], izz, izz, "box with bore Izz");
}

#[test]
fn box_with_offset_bore_mass_properties_match_closed_form() {
    // Off-centre, so the centre of mass moves and the parallel-axis terms are
    // actually exercised rather than cancelling.
    let (dx, dy, dz, r) = (24.0, 16.0, 9.0, 4.0);
    let (bx, by) = (7.0, 6.0);

    let mut topo = Topology::new();
    let block = make_box(&mut topo, dx, dy, dz).unwrap();
    let drill = make_cylinder(&mut topo, r, dz + 20.0).unwrap();
    transform_solid(&mut topo, drill, &Mat4::translation(bx, by, -10.0)).unwrap();
    let bored = boolean(&mut topo, BooleanOp::Cut, block, drill).unwrap();
    remus_operations::heal::unify_faces(&mut topo, bored).unwrap();

    let expected = Moments::sum(&[
        cuboid(dx, dy, dz, Point3::new(dx / 2.0, dy / 2.0, dz / 2.0), 1.0),
        cylinder(r, dz, Point3::new(bx, by, dz / 2.0), Axis::Z, -1.0),
    ]);
    assert_properties(&topo, bored, &expected, "box with offset bore");
    assert_measurement_is_not_a_setting(&topo, bored, "box with offset bore");
}
