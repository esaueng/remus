//! Sequential fillets on a tabbed bracket, one feature per pass, through
//! `Model::fillet`.
//!
//! The body: the profile (0,0) (22,0) (22,8) (52,8) (52,0) (74,0) (74,60)
//! (0,60) extruded 4 in +z, then two 18 × 10 × 40 tabs fused at x ∈ [0,18]
//! and x ∈ [56,74], y ∈ [0,10]. The passes a user makes:
//!
//! | Pass | Edges | Radius |
//! |---|---|---|
//! | F1 | front vertical corners (x ∈ {0,74}, y = 60) | 6 |
//! | F2 | slot corners (x ∈ {22,52}, y = 8) | 2 |
//! | F3 | tab vertical edges (x ∈ {18,56}, z ≥ 4) | 2 |
//! | F4 | concave tab/base edges (z = 4, y = 10) | 3 |
//! | F5 | tab top loops (z = 40) | 2 |
//! | F6 | base top front edge plus the two F1 arcs (z = 4) | 1 |
//!
//! Each pass worked on the fresh body, but three failed after earlier passes:
//!
//! - **F3** after F1 or F2: the rolling-ball rebuild of an earlier pass merges
//!   coplanar faces, so the tab's front face and the base's front face become
//!   one L-shaped face. The F3 spine then ends at a reflex corner of a support
//!   face, and the cap the rolling ball assumed (a face holding both end
//!   contacts) does not exist.
//! - **F6**: the selection's tangent chain crosses the F1 arcs (the walking
//!   engine refuses seam crossings by design) and its side edges run into the
//!   tabs: the same capless straight end.
//! - **F4** after F1/F2 failed the same way.
//!
//! The rolling ball now closes such a straight runout with a planar section
//! facet. It does so only as the cascade's last attempt, after the walking
//! engine has refused the selection, so a selection the walking engine
//! already handled keeps its result. Two pre-existing defects surfaced on the way and are pinned here
//! too: a later rolling-ball pass flattened an earlier pass's sphere corner
//! patches to chords, and the solid mesher dropped a triangle at the pole of
//! a NURBS band whose collapsed row is the last one.
//!
//! F4 after F3 stays refused: G1 propagation carries the concave chain around
//! the F3 cylinders onto an arc that ends tangent to the front face, where the
//! band overflows the body for any radius. That needs blend rollover; see the
//! ignored ready-repro at the end.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::FRAC_PI_4;

use remus::prelude::*;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;

/// Cross-section area a fillet of radius `r` removes or adds at a right-angle
/// edge: the square corner minus the quarter disc.
fn spandrel(r: f64) -> f64 {
    (1.0 - FRAC_PI_4) * r * r
}

const BODY: f64 = 74.0 * 60.0 * 4.0 - 30.0 * 8.0 * 4.0 + 2.0 * 18.0 * 10.0 * 36.0;

fn bracket(model: &mut Model) -> SolidId {
    let profile: Vec<Point3> = [
        (0.0, 0.0),
        (22.0, 0.0),
        (22.0, 8.0),
        (52.0, 8.0),
        (52.0, 0.0),
        (74.0, 0.0),
        (74.0, 60.0),
        (0.0, 60.0),
    ]
    .iter()
    .map(|&(x, y)| Point3::new(x, y, 0.0))
    .collect();
    let face = model.make_planar_face(&profile).unwrap();
    let base = model.extrude(face, Vec3::new(0.0, 0.0, 1.0), 4.0).unwrap();
    let left = model.make_box(18.0, 10.0, 40.0).unwrap();
    let right = model.make_box(18.0, 10.0, 40.0).unwrap();
    model
        .transform(right, &Mat4::translation(56.0, 0.0, 0.0))
        .unwrap();
    let with_left = model.fuse(base, left).unwrap().solid;
    model.fuse(with_left, right).unwrap().solid
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

/// Edges of `solid` whose endpoints and curve satisfy `pick`.
fn edges_where(
    model: &Model,
    solid: SolidId,
    pick: impl Fn(Point3, Point3, &EdgeCurve) -> bool,
) -> Vec<EdgeId> {
    let topo = model.topology();
    model
        .solid_edges(solid)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let edge = topo.edge(id).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            pick(a, b, edge.curve())
        })
        .collect()
}

fn f1_edges(model: &Model, solid: SolidId) -> Vec<EdgeId> {
    edges_where(model, solid, |a, b, _| {
        near(a.y(), 60.0)
            && near(b.y(), 60.0)
            && near(a.x(), b.x())
            && (near(a.x(), 0.0) || near(a.x(), 74.0))
    })
}

fn f2_edges(model: &Model, solid: SolidId) -> Vec<EdgeId> {
    edges_where(model, solid, |a, b, _| {
        near(a.y(), 8.0)
            && near(b.y(), 8.0)
            && near(a.x(), b.x())
            && (near(a.x(), 22.0) || near(a.x(), 52.0))
    })
}

fn f3_edges(model: &Model, solid: SolidId) -> Vec<EdgeId> {
    edges_where(model, solid, |a, b, _| {
        near(a.x(), b.x())
            && near(a.y(), b.y())
            && (near(a.x(), 18.0) || near(a.x(), 56.0))
            && (near(a.y(), 0.0) || near(a.y(), 10.0))
            && a.z().min(b.z()) > 4.0 - 1e-6
    })
}

fn f4_edges(model: &Model, solid: SolidId) -> Vec<EdgeId> {
    edges_where(model, solid, |a, b, _| {
        near(a.z(), 4.0) && near(b.z(), 4.0) && near(a.y(), 10.0) && near(b.y(), 10.0)
    })
}

/// Signed volume of a fine tessellation: an oracle independent of the exact
/// and Gauss volume paths.
fn mesh_volume(mesh: &TriangleMesh) -> f64 {
    mesh.indices
        .chunks(3)
        .map(|t| {
            let [a, b, c] = [t[0], t[1], t[2]].map(|i| {
                let p = mesh.positions[i as usize];
                Vec3::new(p.x(), p.y(), p.z())
            });
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

/// Valid, closed, and watertight as a mesh. Returns the mesh volume.
fn assert_sound(model: &Model, solid: SolidId, pass: &str) -> f64 {
    let report = model.validate(solid).unwrap();
    assert!(
        report.is_valid(),
        "{pass}: invalid result: {:?}",
        report.issues
    );
    let topo = model.topology();
    let shell = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    remus_topology::validation::validate_shell_closed(shell, topo)
        .unwrap_or_else(|e| panic!("{pass}: open shell: {e}"));
    let mesh = model.tessellate_with_tolerance(solid, 0.002, 0.05).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0, "{pass}: mesh has open edges");
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "{pass}: mesh is non-manifold"
    );
    mesh_volume(&mesh)
}

/// Every straight edge between a cylinder and a plane is a tangent seam: the
/// two outward normals agree at its midpoint.
fn assert_cylinder_walls_g1(model: &Model, solid: SolidId, pass: &str) {
    let topo = model.topology();
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    let outward = |face: FaceId, p: Point3| -> Option<Vec3> {
        let data = topo.face(face).ok()?;
        let normal = match data.surface() {
            FaceSurface::Plane { normal, .. } => *normal,
            FaceSurface::Cylinder(cylinder) => {
                let (u, v) = cylinder.project_point(p);
                cylinder.normal(u, v)
            }
            _ => return None,
        };
        Some(if data.is_reversed() { -normal } else { normal })
    };
    let mut seams = 0;
    for &cylinder in &faces {
        if !matches!(
            topo.face(cylinder).unwrap().surface(),
            FaceSurface::Cylinder(_)
        ) {
            continue;
        }
        let wire = topo
            .wire(topo.face(cylinder).unwrap().outer_wire())
            .unwrap();
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge()).unwrap();
            if !matches!(edge.curve(), EdgeCurve::Line) {
                continue;
            }
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            let mid = Point3::new(
                f64::midpoint(a.x(), b.x()),
                f64::midpoint(a.y(), b.y()),
                f64::midpoint(a.z(), b.z()),
            );
            for &plane in &faces {
                if plane == cylinder
                    || !matches!(
                        topo.face(plane).unwrap().surface(),
                        FaceSurface::Plane { .. }
                    )
                {
                    continue;
                }
                let shares = topo
                    .wire(topo.face(plane).unwrap().outer_wire())
                    .unwrap()
                    .edges()
                    .iter()
                    .any(|other| other.edge() == oe.edge());
                if !shares {
                    continue;
                }
                let (Some(nc), Some(np)) = (outward(cylinder, mid), outward(plane, mid)) else {
                    continue;
                };
                assert!(
                    nc.dot(np) > 1.0 - 1e-9,
                    "{pass}: cylinder/plane seam at {mid:?} is not G1 ({nc:?} vs {np:?})"
                );
                seams += 1;
            }
        }
    }
    assert!(seams > 0, "{pass}: no cylinder/plane seams were checked");
}

/// Runs one pass and checks it against the expected volume change.
fn pass(
    model: &mut Model,
    solid: SolidId,
    name: &str,
    edges: &[EdgeId],
    radius: f64,
    expected_change: f64,
) -> SolidId {
    assert!(!edges.is_empty(), "{name}: no edges picked");
    let before = model.volume(solid, 0.01).unwrap();
    let result = model
        .fillet(solid, edges, radius)
        .unwrap_or_else(|e| panic!("{name}: {e}"));
    assert!(
        result.failed.is_empty() && !result.is_partial,
        "{name}: partial blend"
    );
    let after = model.volume(result.solid, 0.01).unwrap();
    assert!(
        (after - before - expected_change).abs() < 1e-6 * BODY,
        "{name}: volume changed by {}, expected {expected_change}",
        after - before
    );
    let meshed = assert_sound(model, result.solid, name);
    assert!(
        (meshed - after).abs() < 0.5,
        "{name}: mesh volume {meshed} disagrees with {after}"
    );
    result.solid
}

#[test]
fn tab_verticals_after_corner_and_slot_fillets() {
    let mut model = Model::new();
    let s0 = bracket(&mut model);
    assert!((model.volume(s0, 0.01).unwrap() - BODY).abs() < 1e-6);

    let edges = f1_edges(&model, s0);
    let s1 = pass(
        &mut model,
        s0,
        "F1",
        &edges,
        6.0,
        -2.0 * spandrel(6.0) * 4.0,
    );
    let edges = f2_edges(&model, s1);
    let s2 = pass(&mut model, s1, "F2", &edges, 2.0, 2.0 * spandrel(2.0) * 4.0);
    // F3 ends on the base top at z = 4, beside the merged L-shaped front face.
    let edges = f3_edges(&model, s2);
    assert_eq!(edges.len(), 4);
    let s3 = pass(
        &mut model,
        s2,
        "F3",
        &edges,
        2.0,
        -4.0 * spandrel(2.0) * 36.0,
    );
    assert_cylinder_walls_g1(&model, s3, "F3");

    // Straight edges between planes stay exact cylinders.
    let topo = model.topology();
    for face in remus_topology::explorer::solid_faces(topo, s3).unwrap() {
        assert!(
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Plane { .. } | FaceSurface::Cylinder(_)
            ),
            "F3 result carries a non-analytic face"
        );
    }
}

#[test]
fn concave_tab_base_after_corner_and_slot_fillets() {
    let mut model = Model::new();
    let s0 = bracket(&mut model);
    let edges = f1_edges(&model, s0);
    let s1 = pass(
        &mut model,
        s0,
        "F1",
        &edges,
        6.0,
        -2.0 * spandrel(6.0) * 4.0,
    );
    let edges = f2_edges(&model, s1);
    let s2 = pass(&mut model, s1, "F2", &edges, 2.0, 2.0 * spandrel(2.0) * 4.0);
    // Concave: both ends meet coplanar continuations (the side face at x = 0
    // and the tab side at x = 18), so both close with section facets.
    let edges = f4_edges(&model, s2);
    assert_eq!(edges.len(), 2);
    let s4 = pass(
        &mut model,
        s2,
        "F4",
        &edges,
        3.0,
        2.0 * spandrel(3.0) * 18.0,
    );
    assert_cylinder_walls_g1(&model, s4, "F4");
}

#[test]
fn concave_tab_base_after_tab_verticals_is_refused_whole() {
    // After F3 the concave chain is G1-propagated around the F3 cylinders and
    // ends on an arc tangent to the front face, where the band would overflow
    // the body at any radius. Until rollover exists that is a typed refusal
    // that leaves the body untouched.
    let mut model = Model::new();
    let s0 = bracket(&mut model);
    let edges = f3_edges(&model, s0);
    let s3 = pass(
        &mut model,
        s0,
        "F3",
        &edges,
        2.0,
        -4.0 * spandrel(2.0) * 36.0,
    );
    let before = remus_topology::explorer::solid_entity_counts(model.topology(), s3).unwrap();
    let volume = model.volume(s3, 0.01).unwrap();

    let edges = f4_edges(&model, s3);
    let Err(error) = model.fillet(s3, &edges, 3.0) else {
        panic!("F4 after F3 should be refused until rollover exists");
    };
    assert!(
        matches!(
            error,
            OperationsError::Blend(BlendError::CliffEncountered { .. })
        ),
        "unexpected refusal: {error}"
    );
    let after = remus_topology::explorer::solid_entity_counts(model.topology(), s3).unwrap();
    assert_eq!(before, after);
    assert!((model.volume(s3, 0.01).unwrap() - volume).abs() < 1e-9);
}

#[test]
#[ignore = "open: F4 after F3 needs blend rollover — the G1 chain ends on an arc tangent \
            to the front face and the band overflows it (roadmap 5.8 / B4 curved trimming)"]
fn concave_tab_base_after_tab_verticals_rolls_over_front_face() {
    let mut model = Model::new();
    let s0 = bracket(&mut model);
    let edges = f3_edges(&model, s0);
    let s3 = model.fillet(s0, &edges, 2.0).unwrap().solid;
    let v3 = model.volume(s3, 0.01).unwrap();

    let edges = f4_edges(&model, s3);
    let result = model.fillet(s3, &edges, 3.0).unwrap();
    assert!(result.failed.is_empty() && !result.is_partial);
    let meshed = assert_sound(&model, result.solid, "F4 after F3");
    // The straight runs alone (2 × 16 of the r = 3 section) are a lower bound;
    // the band around each tab footprint, trimmed by the front plane, at most
    // doubles it.
    let lower = 2.0 * spandrel(3.0) * 16.0;
    let added = meshed - v3;
    assert!(
        added > lower && added < 2.0 * lower + 2.0 * spandrel(3.0) * 18.0,
        "added {added}"
    );
}
