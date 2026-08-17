//! Regression: `shell` must not fill in the holes of the body it hollows, and
//! must not hand back a shell that is still open.
//!
//! `shell` rebuilt every face of the solid from a list of outer-wire vertex
//! positions and handed the assembler `inner_wires: vec![]` for all ten of
//! them — the five it copies to make the outer skin, and the five it offsets
//! to make the inner one. Any face carrying an inner wire — a through bore's
//! mouth, a pocket opening — therefore came back solid on both skins. The
//! bore's own wall survived with its rims referenced by nothing, so the free
//! rims were swept up by the rim-closing pass and stitched into ONE planar
//! face spanning the whole opening with every other loop as a hole: a lid
//! lying across a body it did not belong to. Hollowing a bored plate with no
//! openings at all came back with a spurious lid, an invalid Euler
//! characteristic, and a volume that was neither the shell's nor the plate's.
//!
//! The same rebuild hardcoded `reversed: false` on the copied outer faces, so
//! a bore's wall — which a boolean cut always leaves reversed — came back
//! facing into the metal and faceted from a cylinder into a 64-gon; and it
//! offset every cylinder and sphere to `radius - thickness` regardless of
//! which way the face pointed, so a bore was made NARROWER by hollowing when
//! taking material out around it must make it wider.
//!
//! Independently of holes, the polygon a full-circle wall was described by
//! never closed either of its rim circles — it published 31 of each rim's 32
//! chords and two diagonals across the seam — so the caps that meet those
//! rims had nothing to share them with. A plain cylinder hollowed into a cup
//! came back 20% under its analytic volume.
//!
//! What must hold now:
//!   * the outer skin is the input's own surface: same faces, same
//!     orientation, same curved edges, same holes;
//!   * the inner skin carries each face's holes through the same miter offset
//!     that places the face's own corners, so a bore's mouth meets the rim of
//!     the offset bore wall;
//!   * a bore widens by the wall thickness and keeps facing its own axis;
//!   * every returned body is closed — every edge shared by exactly two face
//!     uses, and its triangle mesh watertight — and encloses the volume the
//!     closed form says it should.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;
use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_cylinder, make_sphere};
use remus_operations::shell_op::shell;
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_operations::transform::transform_solid;
use remus_operations::{OperationsError, measure};
use remus_topology::Topology;
use remus_topology::explorer::{edge_to_face_map, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const W: f64 = 80.0;
const D: f64 = 60.0;
const T: f64 = 6.0;
const BORE_R: f64 = 4.0;
const WALL: f64 = 1.0;

/// Deflection for volume comparisons. Fine enough that a bore's faceting is
/// identical between input and output, so a filled hole cannot hide in it.
const DEFLECTION: f64 = 0.01;

/// A `W` x `D` x `T` plate with `holes` drilled straight through in Z.
fn plate(topo: &mut Topology, holes: &[(f64, f64)]) -> SolidId {
    let mut body = make_box(topo, W, D, T).expect("plate blank");
    for &(x, y) in holes {
        let drill = make_cylinder(topo, BORE_R, T + 4.0).expect("drill");
        transform_solid(topo, drill, &Mat4::translation(x, y, -2.0)).expect("place drill");
        body = boolean(topo, BooleanOp::Cut, body, drill).expect("drill bore");
    }
    body
}

/// What a `W` x `D` x `T` plate with `n` bores of `BORE_R` encloses once
/// hollowed to `WALL`, with `open` faces removed from the top.
///
/// The hollow is the set of points more than one wall thickness inside the
/// body: the plate shrunk by `WALL` on every closed side, with each bore
/// GROWN by `WALL`. Every bore here is far enough from the plate's edges and
/// from its neighbours that these do not interact.
fn hollow_plate_volume(n: f64, open_top: bool) -> f64 {
    let solid = W * D * T - n * PI * BORE_R * BORE_R * T;
    let depth = if open_top { T - WALL } else { T - 2.0 * WALL };
    let bore_out = BORE_R + WALL;
    let hollow = (W - 2.0 * WALL) * (D - 2.0 * WALL) * depth - n * PI * bore_out * bore_out * depth;
    solid - hollow
}

fn faces(topo: &Topology, solid: SolidId) -> Vec<FaceId> {
    solid_faces(topo, solid).unwrap()
}

/// Total inner-wire count across the solid.
fn hole_count(topo: &Topology, solid: SolidId) -> usize {
    faces(topo, solid)
        .iter()
        .map(|&f| topo.face(f).unwrap().inner_wires().len())
        .sum()
}

/// Panics unless every edge of `solid` is used by exactly two face uses.
fn assert_closed(topo: &Topology, solid: SolidId, what: &str) {
    let uses = edge_to_face_map(topo, solid).unwrap();
    let free = uses.values().filter(|f| f.len() < 2).count();
    let non_manifold = uses.values().filter(|f| f.len() > 2).count();
    assert_eq!(
        (free, non_manifold),
        (0, 0),
        "{what} is not a closed shell: {free} free edges, {non_manifold} non-manifold edges"
    );
}

/// The solid's triangle mesh, as `(signed volume, edges with incidence != 2)`
/// after welding coincident vertices.
///
/// An independent reading of the same body: it goes through the tessellator
/// rather than `solid_volume`'s analytic per-face integrals, and it is the
/// form a slicer or an STL consumer sees.
fn mesh(topo: &Topology, solid: SolidId) -> (f64, usize) {
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01, 0.1).unwrap();
    let mut volume = 0.0;
    for tri in mesh.indices.chunks_exact(3) {
        let (a, b, c) = (
            mesh.positions[tri[0] as usize],
            mesh.positions[tri[1] as usize],
            mesh.positions[tri[2] as usize],
        );
        volume += (a.x() * (b.y() * c.z() - b.z() * c.y())
            - a.y() * (b.x() * c.z() - b.z() * c.x())
            + a.z() * (b.x() * c.y() - b.y() * c.x()))
            / 6.0;
    }

    let mut canonical: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut remap = vec![0_u32; mesh.positions.len()];
    for (i, p) in mesh.positions.iter().enumerate() {
        let key = (
            (p.x() * 1e6).round() as i64,
            (p.y() * 1e6).round() as i64,
            (p.z() * 1e6).round() as i64,
        );
        let next = u32::try_from(canonical.len()).unwrap();
        remap[i] = *canonical.entry(key).or_insert(next);
    }
    let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let v = [
            remap[tri[0] as usize],
            remap[tri[1] as usize],
            remap[tri[2] as usize],
        ];
        for &(a, b) in &[(v[0], v[1]), (v[1], v[2]), (v[2], v[0])] {
            *edges
                .entry(if a < b { (a, b) } else { (b, a) })
                .or_insert(0) += 1;
        }
    }
    (volume, edges.values().filter(|&&c| c != 2).count())
}

fn assert_watertight_mesh(topo: &Topology, solid: SolidId, what: &str) -> f64 {
    let (volume, open_edges) = mesh(topo, solid);
    assert_eq!(open_edges, 0, "{what}: mesh is not watertight");
    volume
}

/// Every cylindrical face of the solid, as `(radius, reversed)`.
fn cylinders(topo: &Topology, solid: SolidId) -> Vec<(f64, bool)> {
    let mut found: Vec<(f64, bool)> = faces(topo, solid)
        .iter()
        .filter_map(|&f| {
            let face = topo.face(f).unwrap();
            match face.surface() {
                FaceSurface::Cylinder(c) => Some((c.radius(), face.is_reversed())),
                _ => None,
            }
        })
        .collect();
    found.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    found
}

fn face_with_normal(topo: &Topology, solid: SolidId, n: Vec3) -> FaceId {
    let mut best: Option<(f64, FaceId)> = None;
    for f in faces(topo, solid) {
        let face = topo.face(f).unwrap();
        if !face
            .effective_plane_normal()
            .is_some_and(|e| (e - n).length() < 1e-9)
        {
            continue;
        }
        let wire = topo.wire(face.outer_wire()).unwrap();
        let edge = topo.edge(wire.edges()[0].edge()).unwrap();
        let p: Point3 = topo.vertex(edge.start()).unwrap().point();
        let reach = n.dot(Vec3::new(p.x(), p.y(), p.z()));
        if best.is_none_or(|(r, _)| reach > r) {
            best = Some((reach, f));
        }
    }
    best.expect("no face with that outward normal").1
}

// ── The bore survives ────────────────────────────────────────────────────

#[test]
fn hollowing_a_bored_plate_keeps_the_bore() {
    let mut topo = Topology::new();
    let body = plate(&mut topo, &[(40.0, 30.0)]);
    assert_eq!(hole_count(&topo, body), 2, "the plate's two bore mouths");

    let hollow = shell(&mut topo, body, WALL, &[]).expect("hollow the plate");

    // Both skins keep both mouths: 2 on the outer caps, 2 on the inner ones.
    // This used to be 1 — and that one was the spurious lid's.
    assert_eq!(
        hole_count(&topo, hollow),
        4,
        "the bore's mouth must survive on both the outer and the inner skin"
    );
    assert_closed(&topo, hollow, "the hollowed plate");
    assert_watertight_mesh(&topo, hollow, "the hollowed plate");

    let expected = hollow_plate_volume(1.0, false);
    let got = measure::solid_volume(&topo, hollow, DEFLECTION).unwrap();
    assert!(
        (got - expected).abs() < 1e-6 * expected,
        "hollowed bored plate encloses {got}, closed form is {expected}"
    );
}

#[test]
fn hollowing_a_four_bore_plate_keeps_all_four() {
    let mut topo = Topology::new();
    let body = plate(
        &mut topo,
        &[(20.0, 15.0), (60.0, 15.0), (20.0, 45.0), (60.0, 45.0)],
    );
    assert_eq!(hole_count(&topo, body), 8);

    let hollow = shell(&mut topo, body, WALL, &[]).expect("hollow the plate");

    assert_eq!(
        hole_count(&topo, hollow),
        16,
        "four bores, two mouths, two skins"
    );
    assert_closed(&topo, hollow, "the hollowed four-bore plate");
    assert_watertight_mesh(&topo, hollow, "the hollowed four-bore plate");

    let expected = hollow_plate_volume(4.0, false);
    let got = measure::solid_volume(&topo, hollow, DEFLECTION).unwrap();
    assert!(
        (got - expected).abs() < 1e-6 * expected,
        "hollowed four-bore plate encloses {got}, closed form is {expected}"
    );
}

#[test]
fn a_bore_widens_by_the_wall_thickness_and_keeps_facing_its_axis() {
    let mut topo = Topology::new();
    let body = plate(&mut topo, &[(40.0, 30.0)]);
    assert_eq!(
        cylinders(&topo, body),
        vec![(BORE_R, true)],
        "a drilled bore's wall faces its own axis, so the face is reversed"
    );

    let hollow = shell(&mut topo, body, WALL, &[]).expect("hollow the plate");

    // Taking metal out AROUND a bore makes the bore wider, not narrower, and
    // the new wall faces the metal it now bounds — the opposite way from the
    // wall it was offset from. Both used to come out backwards.
    assert_eq!(
        cylinders(&topo, hollow),
        vec![(BORE_R, true), (BORE_R + WALL, false)],
        "the offset bore wall must sit one wall thickness OUT from the bore"
    );
}

// ── Openings ─────────────────────────────────────────────────────────────

#[test]
fn hollowing_a_bored_plate_with_its_top_open_rims_the_bore_separately() {
    let mut topo = Topology::new();
    let body = plate(&mut topo, &[(40.0, 30.0)]);
    let top = face_with_normal(&topo, body, Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(topo.face(top).unwrap().inner_wires().len(), 1);

    let hollow = shell(&mut topo, body, WALL, &[top]).expect("hollow with an open top");

    // The opening's rim is TWO annuli, not one face with three holes in it:
    // the wall's own rim, and the rim of the bore that passes through it.
    // Six outer faces, six inner, two rims.
    assert_eq!(faces(&topo, hollow).len(), 14);
    assert_closed(&topo, hollow, "the open-topped hollow plate");
    assert_watertight_mesh(&topo, hollow, "the open-topped hollow plate");

    let expected = hollow_plate_volume(1.0, true);
    let got = measure::solid_volume(&topo, hollow, DEFLECTION).unwrap();
    assert!(
        (got - expected).abs() < 1e-6 * expected,
        "open-topped hollow plate encloses {got}, closed form is {expected}"
    );
}

#[test]
fn hollowing_a_bored_plate_with_a_side_open_keeps_the_bore() {
    let mut topo = Topology::new();
    let body = plate(&mut topo, &[(40.0, 30.0)]);
    let side = face_with_normal(&topo, body, Vec3::new(1.0, 0.0, 0.0));

    let hollow = shell(&mut topo, body, WALL, &[side]).expect("hollow with an open side");

    assert_closed(&topo, hollow, "the side-opened hollow plate");
    assert_watertight_mesh(&topo, hollow, "the side-opened hollow plate");
    assert_eq!(
        cylinders(&topo, hollow),
        vec![(BORE_R, true), (BORE_R + WALL, false)]
    );

    // The hollow now runs to x = W: solid minus a (W - WALL) x (D - 2 WALL) x
    // (T - 2 WALL) box with the grown bore taken out of it.
    let solid = W * D * T - PI * BORE_R * BORE_R * T;
    let depth = T - 2.0 * WALL;
    let bore_out = BORE_R + WALL;
    let hollow_v = (W - WALL) * (D - 2.0 * WALL) * depth - PI * bore_out * bore_out * depth;
    let expected = solid - hollow_v;
    let got = measure::solid_volume(&topo, hollow, DEFLECTION).unwrap();
    assert!(
        (got - expected).abs() < 1e-6 * expected,
        "side-opened hollow plate encloses {got}, closed form is {expected}"
    );
}

#[test]
fn opening_a_face_that_is_not_planar_is_refused_by_name() {
    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, 10.0, 16.0).unwrap();
    let wall = faces(&topo, cyl)
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();

    // There is no plane to rim the exposed wall in, so this is refused rather
    // than closed with a guessed one.
    match shell(&mut topo, cyl, 1.2, &[wall]) {
        Err(OperationsError::Unsupported { operation, reason }) => {
            assert_eq!(operation, "shell");
            assert!(reason.contains("not planar"), "unexpected reason: {reason}");
        }
        other => panic!("expected a typed refusal, got {other:?}"),
    }
}

// ── Other holed faces ────────────────────────────────────────────────────

#[test]
fn hollowing_a_pocketed_block_does_not_fill_the_pocket() {
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, 40.0, 40.0, 10.0).unwrap();
    let cutter = make_box(&mut topo, 12.0, 12.0, 4.0).unwrap();
    transform_solid(&mut topo, cutter, &Mat4::translation(14.0, 14.0, 7.0)).unwrap();
    let pocketed = boolean(&mut topo, BooleanOp::Cut, blank, cutter).unwrap();
    assert_eq!(hole_count(&topo, pocketed), 1, "the pocket's mouth");

    let hollow = shell(&mut topo, pocketed, 1.0, &[]).expect("hollow the block");

    assert_eq!(hole_count(&topo, hollow), 2, "one mouth per skin");
    assert_closed(&topo, hollow, "the hollowed pocketed block");
    assert_watertight_mesh(&topo, hollow, "the hollowed pocketed block");

    // 40 x 40 x 10 less a 12 x 12 x 3 pocket, hollowed to 1: the cavity is the
    // 38 x 38 x 8 core less the pocket grown by 1 on each side, 14 x 14 x 3.
    let expected = (16_000.0 - 432.0) - (38.0 * 38.0 * 8.0 - 14.0 * 14.0 * 3.0);
    let got = measure::solid_volume(&topo, hollow, DEFLECTION).unwrap();
    assert!(
        (got - expected).abs() < 1e-9 * expected,
        "hollowed pocketed block encloses {got}, closed form is {expected}"
    );
}

#[test]
fn hollowing_a_drilled_sphere_keeps_the_drilling() {
    let mut topo = Topology::new();
    let ball = make_sphere(&mut topo, 10.0, 32).unwrap();
    let drill = make_cylinder(&mut topo, 3.0, 40.0).unwrap();
    transform_solid(&mut topo, drill, &Mat4::translation(0.0, 0.0, -20.0)).unwrap();
    let drilled = boolean(&mut topo, BooleanOp::Cut, ball, drill).unwrap();
    assert_eq!(hole_count(&topo, drilled), 2, "one mouth per hemisphere");

    let hollow = shell(&mut topo, drilled, 1.0, &[]).expect("hollow the ball");

    assert_eq!(
        hole_count(&topo, hollow),
        4,
        "one mouth per hemisphere skin"
    );
    assert_closed(&topo, hollow, "the hollowed drilled sphere");
    assert_eq!(
        cylinders(&topo, hollow),
        vec![(3.0, true), (4.0, false)],
        "the drilling widens by the wall thickness"
    );

    // A napkin ring: a sphere of radius R with a coaxial hole of radius a
    // encloses 4/3 pi (R^2 - a^2)^(3/2), whatever R and a are. The hollow is
    // the ring of the shrunk sphere and the grown hole.
    //
    // Read off the triangle mesh rather than `solid_volume`: the analytic
    // per-face path integrates a sphere face over the parameter box its outer
    // wire spans and takes no account of the wire inside it, so it reads this
    // body as if the drilling's own wall enclosed nothing. That is a defect in
    // `measure`, not here — the mesh, which is what a slicer sees, is
    // watertight and agrees with the closed form.
    let ring = |r: f64, a: f64| 4.0 / 3.0 * PI * (r * r - a * a).powf(1.5);
    let expected = ring(10.0, 3.0) - ring(9.0, 4.0);
    let got = assert_watertight_mesh(&topo, hollow, "the hollowed drilled sphere");
    assert!(
        (got - expected).abs() < 1e-3 * expected,
        "hollowed drilled sphere meshes to {got}, closed form is {expected}"
    );
}

// ── The plain bodies still work ──────────────────────────────────────────

#[test]
fn hollowing_a_plain_cylinder_gives_the_analytic_cup() {
    let (r, h, wall) = (10.0, 16.0, 1.2);
    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, r, h).unwrap();
    let top = face_with_normal(&topo, cyl, Vec3::new(0.0, 0.0, 1.0));

    let cup = shell(&mut topo, cyl, wall, &[top]).expect("hollow the cylinder");

    assert_closed(&topo, cup, "the cup");
    assert_watertight_mesh(&topo, cup, "the cup");

    // This read 1133.39 against the analytic 1425.93 — a fifth of the cup
    // missing — because the wall's polygon closed neither of its rim circles,
    // so the bottom cap could share neither and the rim was traced across the
    // body instead of around the opening.
    let expected = PI * (r * r * h - (r - wall).powi(2) * (h - wall));
    let got = measure::solid_volume(&topo, cup, DEFLECTION).unwrap();
    assert!(
        (got - expected).abs() < 1e-6 * expected,
        "the cup encloses {got}, closed form is {expected}"
    );
}

#[test]
fn hollowing_a_plain_box_is_unchanged() {
    let mut topo = Topology::new();
    let body = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let top = face_with_normal(&topo, body, Vec3::new(0.0, 0.0, 1.0));

    let open = shell(&mut topo, body, 1.0, &[top]).expect("hollow the box");
    assert_closed(&topo, open, "the open-topped box");
    let expected = 1000.0 - 8.0 * 8.0 * 9.0;
    let got = measure::solid_volume(&topo, open, DEFLECTION).unwrap();
    assert!((got - expected).abs() < 1e-9, "got {got}, want {expected}");

    let mut topo = Topology::new();
    let body = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let closed = shell(&mut topo, body, 1.0, &[]).expect("hollow the box");
    assert_closed(&topo, closed, "the closed box");
    let expected = 1000.0 - 8.0 * 8.0 * 8.0;
    let got = measure::solid_volume(&topo, closed, DEFLECTION).unwrap();
    assert!((got - expected).abs() < 1e-9, "got {got}, want {expected}");
}
