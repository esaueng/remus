//! `solid_volume` must not depend on how a solid was decomposed.
//!
//! A body with holed faces is measured by
//! `volume_from_direct_face_tessellation`, which integrates quadric faces
//! analytically. It used to integrate the PLANAR faces from their own
//! tessellation instead, and that tessellation dropped the vertex where a wire
//! ARRIVES at a reversed circular arc: the boundary polygon ran from the
//! previous edge's last sample straight into the arc's interior. Where the
//! previous edge was long, that chord sliced a large triangle off the face, and
//! the divergence sum lost `(1/3)·|d|·(sliced area)` with nothing to cancel it.
//!
//! Two decompositions of the same solid therefore measured differently: the
//! OpenZCAD demo bracket read 47348.195 mm³ once its rear corner blends put an
//! arc on the z = 39.5 wall top, against a true 47360.940 mm³ — 0.027 % light —
//! and a plate whose corner is scalloped by a single cut read 0.136 % light.
//!
//! Ground truth in each case is a closed form for the nominal geometry, built
//! from the same dimension constants the model is, and cross-checked against a
//! whole-solid mesh convergence sweep. Nothing here is a recorded measurement.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::Vec3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

/// Divergence-theorem volume of a triangle mesh, plus its count of edges that
/// are not shared by exactly two triangles (0 ⇒ closed).
fn mesh_volume(topo: &Topology, solid: SolidId, deflection: f64) -> (f64, usize) {
    use std::collections::HashMap;

    let mesh = remus_operations::tessellate::tessellate_solid(topo, solid, deflection).unwrap();
    let mut incidence: HashMap<(u32, u32), usize> = HashMap::new();
    let mut total = 0.0;
    for tri in mesh.indices.chunks_exact(3) {
        for &(i, j) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            *incidence
                .entry(if i < j { (i, j) } else { (j, i) })
                .or_insert(0) += 1;
        }
        let p = |k: usize| {
            let v = mesh.positions[tri[k] as usize];
            Vec3::new(v.x(), v.y(), v.z())
        };
        total += p(0).dot(p(1).cross(p(2)));
    }
    (
        (total / 6.0).abs(),
        incidence.values().filter(|&&c| c != 2).count(),
    )
}

/// `(planes, cylinders, nurbs, faces)` of a solid's outer shell.
fn census(topo: &Topology, solid: SolidId) -> (usize, usize, usize, usize) {
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    let (mut planes, mut cylinders, mut nurbs) = (0, 0, 0);
    for &fid in &faces {
        match topo.face(fid).unwrap().surface() {
            FaceSurface::Plane { .. } => planes += 1,
            FaceSurface::Cylinder(_) => cylinders += 1,
            FaceSurface::Nurbs(_) => nurbs += 1,
            _ => {}
        }
    }
    (planes, cylinders, nurbs, faces.len())
}

/// A measurement is a property of the body, not of the preview quality the
/// caller happened to ask for: `solid_volume` must return the SAME number at
/// every deflection for an all-analytic body. This is what a re-recorded
/// baseline cannot satisfy — it pins one number at one deflection.
fn assert_deflection_independent(topo: &Topology, solid: SolidId, what: &str) -> f64 {
    let reference = solid_volume(topo, solid, 1.0).unwrap();
    for deflection in [0.5, 0.1, 0.01, 1e-4, 1e-6] {
        let v = solid_volume(topo, solid, deflection).unwrap();
        assert!(
            (v - reference).abs() <= 1e-9 * reference.abs(),
            "{what}: solid_volume depends on deflection — {reference} at 1.0 vs {v} at {deflection}"
        );
    }
    reference
}

fn assert_matches(actual: f64, expected: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= 1e-9 * expected.abs(),
        "{what}: expected the closed form {expected:.9}, got {actual:.9} \
         ({:+.6} mm³, {:+.5} %)",
        actual - expected,
        100.0 * (actual - expected) / expected
    );
}

// ---------------------------------------------------------------------------
// The OpenZCAD demo bracket (the reported case)
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

/// The demo's Rev B body: base plate + seated wall + boss, less a bore and two
/// mount holes, face-unified after every boolean exactly as the OpenZCAD
/// adapter does.
fn build_bracket_rev_b(topo: &mut Topology) -> SolidId {
    let unify = |topo: &mut Topology, s: SolidId| {
        remus_operations::heal::unify_faces(topo, s).unwrap();
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

/// Rev B closed form: base ∪ wall (their 0.5 mm seat counted once), plus the
/// part of the boss that stands proud of the wall, less the bore and the two
/// mount holes. 45760 + 568·π mm³.
fn bracket_rev_b_closed_form() -> f64 {
    let wall_front = D - PT; //                    y = 32, the wall's front face
    let boss_far = D - PT + 2.0; //                y = 34, the boss's buried face
    let boss_front = boss_far - BOSS_H; //         y = 22, the boss's free face

    let l_blank = W * D * PT + W * PT * WH - W * PT * SEAT;
    let boss_proud = PI * BOSS_R * BOSS_R * (wall_front - boss_front);
    // The bore runs through the boss and the wall as one continuous 18 mm hole
    // (they overlap in y ∈ [32, 34]) and exits the back face at y = D.
    let bore = PI * HOLE_R * HOLE_R * (D - boss_front);
    let mounts = 2.0 * PI * MOUNT_R * MOUNT_R * PT;
    l_blank + boss_proud - bore - mounts
}

/// The demo's Rev C pick: the four vertical corner edges of the base plate
/// (all points in a corner column, z within [−0.1, 8.1], z-span ≥ 4).
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

/// Material a radius-`r` fillet takes off a right-angle corner of height `h`.
fn corner_blend_volume(r: f64, h: f64) -> f64 {
    (r * r - PI * r * r / 4.0) * h
}

#[test]
fn demo_bracket_rev_b_volume_matches_closed_form() {
    let mut topo = Topology::new();
    let body = build_bracket_rev_b(&mut topo);

    // The Rev B face set: 9 planes (L-shaped sides, holed caps) + 4 quadric
    // walls (boss, bore, two mount holes). Fully analytic, holed ⇒ measured by
    // the per-face summation path.
    assert_eq!(census(&topo, body), (9, 4, 0, 13), "Rev B face census");

    let v = assert_deflection_independent(&topo, body, "bracket Rev B");
    assert_matches(v, bracket_rev_b_closed_form(), "bracket Rev B");
}

#[test]
fn demo_bracket_rev_c_volume_matches_closed_form() {
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

    // The reported fast-path face set: 9 planes, 8 cylinders, no b-splines.
    assert_eq!(census(&topo, filleted), (9, 8, 0, 17), "Rev C face census");

    // Each blend runs the full height of the corner it rounds: the two front
    // corners are 8 mm of base plate; the two rear ones continue up the wall
    // to its top face at z = plate_t + wall_h − seat = 39.5.
    let expected = bracket_rev_b_closed_form()
        - 2.0 * corner_blend_volume(FILLET_R, PT)
        - 2.0 * corner_blend_volume(FILLET_R, PT + WH - SEAT);

    let v = assert_deflection_independent(&topo, filleted, "bracket Rev C");
    assert_matches(v, expected, "bracket Rev C");

    // The old per-face path read 47348.195 here — 12.7 mm³ (0.027 %) light,
    // because the rear blends put an r = 3 arc on the z = 39.5 wall top whose
    // arrival vertex the planar tessellation dropped. Nail that magnitude down
    // so a re-record cannot quietly restore it.
    assert!(
        (v - 47_348.194_726).abs() > 1.0,
        "Rev C volume regressed to the decomposition-dependent figure: {v}"
    );
}

// ---------------------------------------------------------------------------
// A minimal reproduction with no blend engine in the loop
// ---------------------------------------------------------------------------

/// A tall plate with one vertical corner scalloped away by a cylinder and a
/// bore through the middle. Two cylinders, six planes, holed end caps — the
/// same measurement configuration as the bracket's Rev C body, reachable with
/// booleans alone.
fn build_scalloped_plate(
    topo: &mut Topology,
    w: f64,
    d: f64,
    h: f64,
    r: f64,
    bore_r: f64,
) -> SolidId {
    let plate = make_box(topo, w, d, h).unwrap();
    let scallop = make_cylinder(topo, r, h + 20.0).unwrap();
    transform_solid(topo, scallop, &Mat4::translation(0.0, d, -10.0)).unwrap();
    let rounded = boolean(topo, BooleanOp::Cut, plate, scallop).unwrap();
    let drill = make_cylinder(topo, bore_r, h + 20.0).unwrap();
    transform_solid(topo, drill, &Mat4::translation(w / 2.0, d / 2.0, -10.0)).unwrap();
    let out = boolean(topo, BooleanOp::Cut, rounded, drill).unwrap();
    remus_operations::heal::unify_faces(topo, out).unwrap();
    out
}

#[test]
fn scalloped_plate_volume_matches_closed_form_and_dense_mesh() {
    let (w, d, h, r, bore_r) = (80.0, 40.0, 39.5, 3.0, 4.0);
    let mut topo = Topology::new();
    let body = build_scalloped_plate(&mut topo, w, d, h, r, bore_r);
    assert_eq!(census(&topo, body), (6, 2, 0, 8), "scalloped plate census");

    // Box, less the quarter of the corner cylinder that was inside it, less the
    // bore.
    let expected = w * d * h - PI * r * r / 4.0 * h - PI * bore_r * bore_r * h;

    let v = assert_deflection_independent(&topo, body, "scalloped plate");
    assert_matches(v, expected, "scalloped plate");

    // The old path read 123966.292 at any preview deflection and 124110.705 at
    // 1e-4 — 0.136 % and 0.020 % light. Both are far outside the closed form.
    assert!(
        (v - 123_966.292_205).abs() > 1.0 && (v - 124_110.704_844).abs() > 1.0,
        "scalloped plate volume regressed to a chord-limited figure: {v}"
    );

    // Independent check: the whole-solid mesh is a closed surface whose
    // divergence-theorem volume must converge on the same number from above
    // (both curved features here are concave, so an inscribed mesh removes too
    // little). Three deflections, each closer than the last.
    let mut previous = f64::INFINITY;
    for deflection in [0.02, 0.005, 0.001] {
        let (mesh_v, open) = mesh_volume(&topo, body, deflection);
        assert_eq!(open, 0, "mesh at deflection {deflection} is not closed");
        assert!(
            mesh_v > expected,
            "inscribed mesh at {deflection} should over-count the concave features: \
             {mesh_v} vs {expected}"
        );
        assert!(
            mesh_v < previous,
            "mesh volume must tighten with deflection: {mesh_v} at {deflection} vs {previous}"
        );
        previous = mesh_v;
    }
    assert!(
        previous - expected < 1e-4 * expected,
        "dense mesh should land within 0.01 % of the closed form, got {previous} vs {expected}"
    );
}

// ---------------------------------------------------------------------------
// The tessellation defect underneath, pinned on its own
// ---------------------------------------------------------------------------

#[test]
fn planar_face_boundary_keeps_the_vertex_where_a_reversed_arc_starts() {
    // A plate with one corner scalloped: its top face is a long rectangle whose
    // boundary runs 77 mm along y = depth and then turns into an r = 3 arc.
    let (w, d, h, r) = (80.0, 40.0, 10.0, 3.0);
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, w, d, h).unwrap();
    let scallop = make_cylinder(&mut topo, r, h + 20.0).unwrap();
    transform_solid(&mut topo, scallop, &Mat4::translation(0.0, d, -10.0)).unwrap();
    let body = boolean(&mut topo, BooleanOp::Cut, plate, scallop).unwrap();

    let top = remus_topology::explorer::solid_faces(&topo, body)
        .unwrap()
        .into_iter()
        .find(|&fid| {
            matches!(
                topo.face(fid).unwrap().surface(),
                FaceSurface::Plane { normal, d } if normal.z() > 0.9 && (*d - h).abs() < 1e-9
            )
        })
        .expect("top face");

    let exact_area = w * d - PI * r * r / 4.0;
    // Chording an arc moves the boundary by at most the deflection, so the
    // polygon's area may differ from the true one by at most ~(2/3)·L·δ for arc
    // length L — 0.094 mm² here (the scallop is concave, so the polygon keeps a
    // little too much). The dropped-arrival-vertex bug lost ~2.9 mm², two
    // orders of magnitude more, because the chord cut back across the 77 mm
    // straight run that precedes the arc.
    let deflection = 0.02;
    let bound = 2.0 / 3.0 * (r * PI / 2.0) * deflection * 1.5;
    let mesh = remus_operations::tessellate::tessellate(&topo, top, deflection).unwrap();
    let mut area = 0.0;
    for tri in mesh.indices.chunks_exact(3) {
        let p = |k: usize| {
            let v = mesh.positions[tri[k] as usize];
            Vec3::new(v.x(), v.y(), v.z())
        };
        area += (p(1) - p(0)).cross(p(2) - p(0)).length() / 2.0;
    }
    assert!(
        (exact_area - area).abs() <= bound,
        "top face tessellation is off by {:.5} mm² on a {exact_area:.5} mm² face at deflection \
         {deflection}; chording an r = {r} quarter arc may move it by at most {bound:.5}",
        exact_area - area
    );

    // And the arc's arrival vertex must actually be on the boundary.
    let arrival = remus_math::vec::Point3::new(r, d, h);
    assert!(
        mesh.positions
            .iter()
            .any(|p| (*p - arrival).length() < 1e-9),
        "the vertex where the straight run meets the arc is missing from the boundary"
    );
}
