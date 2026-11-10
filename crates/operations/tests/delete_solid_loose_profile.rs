//! Deleting a solid built from a loose profile face keeps the profile usable.
//!
//! Constructions build their caps from the profile's own edges (an
//! extrusion's bottom cap, a partial revolution's start cap), so the solid
//! and the loose profile share edges and vertices. `delete_solid` once
//! retired those, leaving the profile's wire, loops and coedges pointing at
//! dead edges: building from the profile again failed with "edge not found".
//! It must now keep every entity the live profile still uses, so the same
//! construction rebuilds with the same volume.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::extrude::extrude;
use remus_operations::loft::loft;
use remus_operations::measure::solid_volume;
use remus_operations::pipe::pipe;
use remus_operations::revolve::revolve;
use remus_operations::sweep::sweep;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;
use remus_topology::validation::{validate_boundary_authority, validate_face_loops};

const TOL: f64 = 1e-7;

fn polygon_face(topo: &mut Topology, points: &[Point3]) -> FaceId {
    let wire = remus_topology::builder::make_polygon_wire(topo, points, TOL).unwrap();
    remus_topology::builder::make_face_from_wire(topo, wire).unwrap()
}

/// A 2 x 3 rectangle in the XZ plane, 2 to 4 from the Z axis.
fn xz_rectangle(topo: &mut Topology) -> FaceId {
    polygon_face(
        topo,
        &[
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 3.0),
            Point3::new(2.0, 0.0, 3.0),
        ],
    )
}

/// A 2 x 1 rectangle in the plane `z = height`.
fn xy_rectangle(topo: &mut Topology, height: f64) -> FaceId {
    polygon_face(
        topo,
        &[
            Point3::new(0.0, 0.0, height),
            Point3::new(2.0, 0.0, height),
            Point3::new(2.0, 1.0, height),
            Point3::new(0.0, 1.0, height),
        ],
    )
}

fn straight_path(length: f64) -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(1.0, 0.5, 0.0), Point3::new(1.0, 0.5, length)],
        vec![1.0, 1.0],
    )
    .unwrap()
}

/// Builds from `profiles`, deletes the result, checks every profile is
/// intact, builds again and returns both volumes.
fn rebuild_after_delete(
    topo: &mut Topology,
    profiles: &[FaceId],
    build: impl Fn(&mut Topology) -> Result<SolidId, OperationsError>,
) -> (f64, f64) {
    let first = build(topo).unwrap();
    let first_faces = solid_faces(topo, first).unwrap();
    for profile in profiles {
        assert!(
            !first_faces.contains(profile),
            "the construction holds the profile face itself"
        );
    }
    let first_volume = solid_volume(topo, first, 1e-3).unwrap();
    let profile_loops: Vec<_> = profiles
        .iter()
        .map(|&face| topo.loops_of_face(face).unwrap().to_vec())
        .collect();

    topo.delete_solid(first).unwrap();

    assert!(topo.solid(first).is_err());
    for (&profile, loops) in profiles.iter().zip(&profile_loops) {
        validate_face_loops(topo, profile).unwrap();
        assert_eq!(topo.loops_of_face(profile).unwrap(), loops.as_slice());
        let wire = topo.wire(topo.face(profile).unwrap().outer_wire()).unwrap();
        for oriented in wire.edges() {
            let edge = topo.edge(oriented.edge()).unwrap();
            topo.vertex(edge.start()).unwrap();
            topo.vertex(edge.end()).unwrap();
        }
    }
    validate_boundary_authority(topo).unwrap();
    assert!(topo.is_restore_fixed_point());

    let second = build(topo).expect("the profile builds again after the delete");
    let second_volume = solid_volume(topo, second, 1e-3).unwrap();
    (first_volume, second_volume)
}

fn assert_close(actual: f64, expected: f64, context: &str) {
    assert!(
        ((actual - expected) / expected).abs() < 1e-6,
        "{context}: {actual} vs {expected}"
    );
}

#[test]
fn extrude_delete_extrude_reuses_the_profile() {
    let mut topo = Topology::new();
    let face = remus_topology::builder::make_rectangle_face(&mut topo, 10.0, 5.0, TOL).unwrap();
    let (first, second) = rebuild_after_delete(&mut topo, &[face], |topo| {
        extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), 3.0)
    });
    assert_close(first, 150.0, "first extrusion");
    assert_close(second, 150.0, "second extrusion");
}

#[test]
fn revolve_delete_revolve_reuses_the_profile() {
    for (angle, label) in [(PI / 2.0, "quarter turn"), (2.0 * PI, "full turn")] {
        let mut topo = Topology::new();
        let face = xz_rectangle(&mut topo);
        let (first, second) = rebuild_after_delete(&mut topo, &[face], |topo| {
            revolve(
                topo,
                face,
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                angle,
            )
        });
        // Pappus: the annular sector of radii 2 and 4, height 3.
        let expected = 0.5 * angle * (16.0 - 4.0) * 3.0;
        assert_close(first, expected, label);
        assert_close(second, expected, label);
    }
}

#[test]
fn sweep_delete_sweep_reuses_the_profile() {
    let mut topo = Topology::new();
    let face = xy_rectangle(&mut topo, 0.0);
    let path = straight_path(4.0);
    let (first, second) = rebuild_after_delete(&mut topo, &[face], |topo| sweep(topo, face, &path));
    assert_close(first, 8.0, "first sweep");
    assert_close(second, 8.0, "second sweep");
}

#[test]
fn pipe_delete_pipe_reuses_the_profile() {
    let mut topo = Topology::new();
    let face = xy_rectangle(&mut topo, 0.0);
    let path = straight_path(4.0);
    let (first, second) =
        rebuild_after_delete(&mut topo, &[face], |topo| pipe(topo, face, &path, None));
    assert_close(first, 8.0, "first pipe");
    assert_close(second, 8.0, "second pipe");
}

#[test]
fn loft_delete_loft_reuses_both_profiles() {
    let mut topo = Topology::new();
    let bottom = xy_rectangle(&mut topo, 0.0);
    let top = xy_rectangle(&mut topo, 5.0);
    let (first, second) =
        rebuild_after_delete(&mut topo, &[bottom, top], |topo| loft(topo, &[bottom, top]));
    assert_close(first, 10.0, "first loft");
    assert_close(second, 10.0, "second loft");
}
