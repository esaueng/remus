//! Rounding an outer vertical edge of a shelled, drafted box (#1740).
//!
//! A 40 × 40 × 20 box, its four sides drafted 2°, shelled 2 mm with the top
//! open. Each outer vertical edge leans, so its stripe runs out on the base
//! and on the rim obliquely and both ends are ellipses. Two things went wrong:
//!
//! 1. At R1 and R0.1, three of the four mirror-symmetric corners were refused.
//!    The rolling ball built the right solid, but the volume guard measured
//!    its cylinder wall with a fallback that saw only the wall's two ruling
//!    angles and integrated a full turn, so it read the fillet as removing
//!    ~226·r more than it does. Which corners failed depended on rounding in
//!    those angles.
//! 2. At R5 every corner came out as a NURBS stripe. The edge's convexity
//!    probe stepped one radius-capped 2 mm off the edge, onto the inner wall
//!    of the 2 mm shell, so the edge never got its exact two-plane section.
//!
//! Every corner must round at R0.1, R1 and R5 to a valid solid with one exact
//! cylindrical blend face and the closed-form volume.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::disallowed_methods,
    missing_docs
)]

use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::draft::draft;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::shell_op::shell;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{face_edges, solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const RADII: [f64; 3] = [0.1, 1.0, 5.0];
const DRAFT_DEGREES: f64 = 2.0;

/// The planar face with outward normal `n` through `p`.
fn plane_face(topo: &Topology, solid: SolidId, n: Vec3, p: Point3) -> FaceId {
    let hits: Vec<_> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            let face = topo.face(f).unwrap();
            let FaceSurface::Plane { normal, d } = face.surface() else {
                return false;
            };
            let outward = if face.is_reversed() {
                -*normal
            } else {
                *normal
            };
            (outward - n).length() < 1e-9
                && (normal.dot(p - Point3::new(0.0, 0.0, 0.0)) - d).abs() < 1e-9
        })
        .collect();
    assert_eq!(
        hits.len(),
        1,
        "one plane face with normal {n:?} through {p:?}"
    );
    hits[0]
}

/// The shelled box, its sides drafted `DRAFT_DEGREES` when `drafted`.
fn shelled_box(topo: &mut Topology, drafted: bool) -> SolidId {
    let mut body = make_box(topo, 40.0, 40.0, 20.0).unwrap();
    if drafted {
        let sides = [
            (Vec3::new(1.0, 0.0, 0.0), Point3::new(40.0, 20.0, 10.0)),
            (Vec3::new(-1.0, 0.0, 0.0), Point3::new(0.0, 20.0, 10.0)),
            (Vec3::new(0.0, 1.0, 0.0), Point3::new(20.0, 40.0, 10.0)),
            (Vec3::new(0.0, -1.0, 0.0), Point3::new(20.0, 0.0, 10.0)),
        ]
        .map(|(n, p)| plane_face(topo, body, n, p));
        body = draft(
            topo,
            body,
            &sides,
            Vec3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 0.0),
            DRAFT_DEGREES.to_radians(),
        )
        .unwrap();
    }
    let top = plane_face(
        topo,
        body,
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(20.0, 20.0, 20.0),
    );
    let shelled = shell(topo, body, 2.0, &[top]).unwrap();
    assert!(
        validate_solid(topo, shelled).unwrap().is_valid(),
        "valid input"
    );
    shelled
}

/// The outer vertical edge standing on base corner `(x, y)`, with its two
/// support faces.
fn outer_vertical(topo: &Topology, solid: SolidId, x: f64, y: f64) -> (EdgeId, [FaceId; 2]) {
    let edges: Vec<_> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            let ends = [edge.start(), edge.end()].map(|v| topo.vertex(v).unwrap().point());
            ends.iter()
                .any(|p| (*p - Point3::new(x, y, 0.0)).length() < 1e-9)
                && (ends[0].z() - ends[1].z()).abs() > 10.0
        })
        .collect();
    assert_eq!(edges.len(), 1, "one outer vertical edge at ({x}, {y})");
    let faces: Vec<_> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| face_edges(topo, f).unwrap().contains(&edges[0]))
        .collect();
    assert_eq!(faces.len(), 2, "two support faces");
    (edges[0], [faces[0], faces[1]])
}

fn outward_normal(topo: &Topology, face: FaceId) -> Vec3 {
    let face = topo.face(face).unwrap();
    let FaceSurface::Plane { normal, .. } = face.surface() else {
        panic!("planar support");
    };
    if face.is_reversed() {
        -*normal
    } else {
        *normal
    }
}

/// Round one corner and check the result against closed forms. Returns the
/// result's volume.
fn round_corner(drafted: bool, x: f64, y: f64, radius: f64) -> f64 {
    let what = format!(
        "{} corner ({x}, {y}) R{radius}",
        if drafted { "drafted" } else { "undrafted" }
    );
    let mut topo = Topology::new();
    let input = shelled_box(&mut topo, drafted);
    let (edge, supports) = outer_vertical(&topo, input, x, y);
    let span = {
        let e = topo.edge(edge).unwrap();
        topo.vertex(e.end()).unwrap().point() - topo.vertex(e.start()).unwrap().point()
    };
    let (length, axis) = (span.length(), span.normalize().unwrap());
    let normals = supports.map(|f| outward_normal(&topo, f));
    let before = solid_volume(&topo, input, 1e-3).unwrap();
    let counts_before = solid_entity_counts(&topo, input).unwrap();

    let result =
        fillet_cascade(&mut topo, input, &[edge], radius).unwrap_or_else(|e| panic!("{what}: {e}"));
    let solid = result.solid;
    assert!(!result.is_partial, "{what}: partial");
    assert_eq!(result.succeeded, vec![edge], "{what}: edge blended");

    let report = validate_solid(&topo, solid).unwrap();
    assert!(
        report.is_valid(),
        "{what}: invalid result {:?}",
        report.issues
    );
    let counts_after = solid_entity_counts(&topo, solid).unwrap();
    assert_eq!(
        counts_after.0,
        counts_before.0 + 1,
        "{what}: one blend face added ({counts_before:?} -> {counts_after:?})"
    );

    // Exactly one new face, an exact cylinder of the blend radius about the
    // edge's direction; nothing approximate.
    let faces = solid_faces(&topo, solid).unwrap();
    assert!(
        faces
            .iter()
            .all(|&f| !matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_))),
        "{what}: NURBS face in an exact two-plane blend"
    );
    let cylinders: Vec<_> = faces
        .iter()
        .filter_map(|&f| match topo.face(f).unwrap().surface() {
            FaceSurface::Cylinder(c) => Some(c.clone()),
            _ => None,
        })
        .collect();
    assert_eq!(cylinders.len(), 1, "{what}: one cylindrical blend face");
    assert!(
        (cylinders[0].radius() - radius).abs() < 1e-9,
        "{what}: blend radius {}",
        cylinders[0].radius()
    );
    assert!(
        cylinders[0].axis().cross(axis).length() < 1e-9,
        "{what}: blend axis {:?} not along the edge {axis:?}",
        cylinders[0].axis()
    );

    // The rounded-off material is a prism of the blend's cross-section along
    // the edge, cut by the horizontal base and rim. By Cavalieri its volume
    // is the cross-section area times the edge length. With `phi` the angle
    // between the outward normals, that area is r²(tan(phi/2) − phi/2).
    let phi = normals[0].dot(normals[1]).clamp(-1.0, 1.0).acos();
    let removed = radius * radius * ((0.5 * phi).tan() - 0.5 * phi) * length;
    let after = solid_volume(&topo, solid, 1e-3).unwrap();
    // The volume measurement chords the base's and rim's ellipse ends; that
    // error grows with the arc, so with r. The refused corners read ~226·r.
    let budget = 5e-3 * radius;
    assert!(
        ((before - after) - removed).abs() <= budget,
        "{what}: removed {} vs closed form {removed} (budget {budget})",
        before - after
    );
    after
}

fn check_all_corners(drafted: bool) {
    let corners = [(0.0, 0.0), (40.0, 0.0), (0.0, 40.0), (40.0, 40.0)];
    for radius in RADII {
        let volumes = corners.map(|(x, y)| round_corner(drafted, x, y, radius));
        // The four corners are mirror images: one volume.
        for v in volumes {
            assert!(
                (v - volumes[0]).abs() <= 1e-9 * volumes[0],
                "R{radius}: corner volumes differ {volumes:?}"
            );
        }
    }
}

#[test]
fn drafted_shell_corner_rounds_exact_at_every_corner_and_radius() {
    check_all_corners(true);
}

#[test]
fn undrafted_shell_corner_control() {
    check_all_corners(false);
}
