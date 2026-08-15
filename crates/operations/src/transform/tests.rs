#![allow(clippy::unwrap_used, clippy::panic)]

use brepkit_math::mat::Mat4;
use brepkit_math::tolerance::Tolerance;
use brepkit_topology::Topology;
use brepkit_topology::face::FaceSurface;
use brepkit_topology::test_utils::make_unit_cube_non_manifold;
use std::ops::Mul;

use super::*;

#[test]
fn translate_cube() {
    let mut topo = Topology::new();
    let solid = make_unit_cube_non_manifold(&mut topo);
    let matrix = Mat4::translation(1.0, 0.0, 0.0);

    transform_solid(&mut topo, solid, &matrix).unwrap();

    // All vertices should have x shifted by 1.0.
    let tol = Tolerance::new();
    for (_id, v) in topo.vertices().iter() {
        let x = v.point().x();
        assert!(
            tol.approx_eq(x, 1.0) || tol.approx_eq(x, 2.0),
            "unexpected x = {x}"
        );
    }
}

#[test]
fn identity_transform_no_change() {
    let mut topo = Topology::new();
    let solid = make_unit_cube_non_manifold(&mut topo);

    let before: Vec<_> = topo.vertices().iter().map(|(_, v)| v.point()).collect();

    transform_solid(&mut topo, solid, &Mat4::identity()).unwrap();

    let tol = Tolerance::new();
    for (i, (_, v)) in topo.vertices().iter().enumerate() {
        assert!(tol.approx_eq(v.point().x(), before[i].x()));
        assert!(tol.approx_eq(v.point().y(), before[i].y()));
        assert!(tol.approx_eq(v.point().z(), before[i].z()));
    }
}

#[test]
fn degenerate_matrix_error() {
    let mut topo = Topology::new();
    let solid = make_unit_cube_non_manifold(&mut topo);
    let matrix = Mat4::scale(0.0, 1.0, 1.0);

    let result = transform_solid(&mut topo, solid, &matrix);
    assert!(result.is_err());
}

/// Rotating a cube 90 degrees around the Z axis should update face normals.
#[test]
fn rotation_updates_face_normals() {
    let mut topo = Topology::new();
    let solid = make_unit_cube_non_manifold(&mut topo);

    // 90-degree rotation around Z: +X face normal → +Y, -X → -Y, etc.
    let matrix = Mat4::rotation_z(std::f64::consts::FRAC_PI_2);
    transform_solid(&mut topo, solid, &matrix).unwrap();

    let tol = Tolerance::loose();
    let solid_data = topo.solid(solid).unwrap();
    let shell = topo.shell(solid_data.outer_shell()).unwrap();

    // Collect all plane normals.
    let mut normals: Vec<Vec3> = Vec::new();
    for &fid in shell.faces() {
        let f = topo.face(fid).unwrap();
        if let FaceSurface::Plane { normal, .. } = f.surface() {
            normals.push(*normal);
        }
    }

    // Original cube had normals along ±X, ±Y, ±Z.
    // After 90° Z-rotation: ±X → ±Y, ±Y → ∓X, ±Z unchanged.
    // So we should still have 6 normals, each approximately axis-aligned.
    assert_eq!(normals.len(), 6);

    // Check that we still have a +Z and -Z normal (unchanged by Z rotation).
    let has_pos_z = normals
        .iter()
        .any(|n| tol.approx_eq(n.z(), 1.0) && tol.approx_eq(n.x(), 0.0));
    let has_neg_z = normals
        .iter()
        .any(|n| tol.approx_eq(n.z(), -1.0) && tol.approx_eq(n.x(), 0.0));
    assert!(has_pos_z, "should have +Z normal after Z rotation");
    assert!(has_neg_z, "should have -Z normal after Z rotation");
}

/// Build a minimal solid containing a single face with the given surface.
///
/// The wire is a unit square in XY; only the face surface type varies.
fn make_single_face_solid(
    topo: &mut Topology,
    surface: FaceSurface,
) -> brepkit_topology::solid::SolidId {
    use brepkit_math::vec::Point3;
    use brepkit_topology::edge::{Edge, EdgeCurve};
    use brepkit_topology::face::Face;
    use brepkit_topology::shell::Shell;
    use brepkit_topology::solid::Solid;
    use brepkit_topology::vertex::Vertex;
    use brepkit_topology::wire::{OrientedEdge, Wire};

    let tol = 1e-7;
    let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), tol));
    let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), tol));
    let v2 = topo.add_vertex(Vertex::new(Point3::new(1.0, 1.0, 0.0), tol));
    let v3 = topo.add_vertex(Vertex::new(Point3::new(0.0, 1.0, 0.0), tol));

    let e0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
    let e1 = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
    let e2 = topo.add_edge(Edge::new(v2, v3, EdgeCurve::Line));
    let e3 = topo.add_edge(Edge::new(v3, v0, EdgeCurve::Line));

    let wire = Wire::new(
        vec![
            OrientedEdge::new(e0, true),
            OrientedEdge::new(e1, true),
            OrientedEdge::new(e2, true),
            OrientedEdge::new(e3, true),
        ],
        true,
    )
    .unwrap();
    let wid = topo.add_wire(wire);
    let fid = topo.add_face(Face::new(wid, vec![], surface));
    let shell = Shell::new(vec![fid]).unwrap();
    let shell_id = topo.add_shell(shell);
    topo.add_solid(Solid::new(shell_id, vec![]))
}

#[test]
fn translate_cylinder_face_updates_origin() {
    use brepkit_math::surfaces::CylindricalSurface;
    use brepkit_math::vec::Point3;

    let mut topo = Topology::new();
    let cyl =
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
    let solid = make_single_face_solid(&mut topo, FaceSurface::Cylinder(cyl));

    let matrix = Mat4::translation(5.0, 3.0, 1.0);
    transform_solid(&mut topo, solid, &matrix).unwrap();

    // Find the (now-transformed) cylinder face.
    let tol = Tolerance::new();
    let solid_data = topo.solid(solid).unwrap();
    let shell = topo.shell(solid_data.outer_shell()).unwrap();
    let mut found = false;
    for &fid in shell.faces() {
        if let FaceSurface::Cylinder(c) = topo.face(fid).unwrap().surface() {
            assert!(
                tol.approx_eq(c.origin().x(), 5.0),
                "cylinder origin x should be 5.0, got {}",
                c.origin().x()
            );
            assert!(
                tol.approx_eq(c.origin().y(), 3.0),
                "cylinder origin y should be 3.0, got {}",
                c.origin().y()
            );
            assert!(
                tol.approx_eq(c.origin().z(), 1.0),
                "cylinder origin z should be 1.0, got {}",
                c.origin().z()
            );
            // The axis (0,0,1) should be unchanged by a pure translation.
            assert!(
                tol.approx_eq(c.axis().z(), 1.0),
                "cylinder axis z should still be 1.0"
            );
            assert!(
                tol.approx_eq(c.radius(), 2.0),
                "cylinder radius should be unchanged"
            );
            found = true;
        }
    }
    assert!(found, "cylinder face not found after transform");
}

#[test]
fn rotate_cylinder_face_updates_axis() {
    use brepkit_math::surfaces::CylindricalSurface;
    use brepkit_math::vec::Point3;

    let mut topo = Topology::new();
    // Cylinder with axis along +Z.
    let cyl =
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let solid = make_single_face_solid(&mut topo, FaceSurface::Cylinder(cyl));

    // 90° rotation around Y: Z-axis → X-axis
    let matrix = Mat4::rotation_y(std::f64::consts::FRAC_PI_2);
    transform_solid(&mut topo, solid, &matrix).unwrap();

    let tol = Tolerance::loose();
    let solid_data = topo.solid(solid).unwrap();
    let shell = topo.shell(solid_data.outer_shell()).unwrap();
    let mut found = false;
    for &fid in shell.faces() {
        if let FaceSurface::Cylinder(c) = topo.face(fid).unwrap().surface() {
            // After 90° Y rotation, original Z-axis should point along +X.
            assert!(
                tol.approx_eq(c.axis().x().abs(), 1.0),
                "cylinder axis should be along X after Y rotation, got {:?}",
                c.axis()
            );
            found = true;
        }
    }
    assert!(found, "cylinder face not found after rotation");
}

#[test]
fn translate_cone_face_updates_apex() {
    use brepkit_math::surfaces::ConicalSurface;
    use brepkit_math::vec::Point3;

    let mut topo = Topology::new();
    let cone = ConicalSurface::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::FRAC_PI_4,
    )
    .unwrap();
    let solid = make_single_face_solid(&mut topo, FaceSurface::Cone(cone));

    let matrix = Mat4::translation(2.0, 4.0, 6.0);
    transform_solid(&mut topo, solid, &matrix).unwrap();

    let tol = Tolerance::new();
    let solid_data = topo.solid(solid).unwrap();
    let shell = topo.shell(solid_data.outer_shell()).unwrap();
    let mut found = false;
    for &fid in shell.faces() {
        if let FaceSurface::Cone(c) = topo.face(fid).unwrap().surface() {
            assert!(
                tol.approx_eq(c.apex().x(), 2.0),
                "cone apex x should be 2.0, got {}",
                c.apex().x()
            );
            assert!(
                tol.approx_eq(c.apex().y(), 4.0),
                "cone apex y should be 4.0"
            );
            assert!(
                tol.approx_eq(c.apex().z(), 6.0),
                "cone apex z should be 6.0"
            );
            // Axis should be unchanged by a translation.
            assert!(
                tol.approx_eq(c.axis().z(), 1.0),
                "cone axis z should still be 1.0"
            );
            found = true;
        }
    }
    assert!(found, "cone face not found after transform");
}

#[test]
fn translate_sphere_face_updates_center() {
    use brepkit_math::surfaces::SphericalSurface;
    use brepkit_math::vec::Point3;

    let mut topo = Topology::new();
    let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 3.0).unwrap();
    let solid = make_single_face_solid(&mut topo, FaceSurface::Sphere(sphere));

    let matrix = Mat4::translation(-1.0, 2.0, 5.0);
    transform_solid(&mut topo, solid, &matrix).unwrap();

    let tol = Tolerance::new();
    let solid_data = topo.solid(solid).unwrap();
    let shell = topo.shell(solid_data.outer_shell()).unwrap();
    let mut found = false;
    for &fid in shell.faces() {
        if let FaceSurface::Sphere(s) = topo.face(fid).unwrap().surface() {
            assert!(
                tol.approx_eq(s.center().x(), -1.0),
                "sphere center x should be -1.0"
            );
            assert!(
                tol.approx_eq(s.center().y(), 2.0),
                "sphere center y should be 2.0"
            );
            assert!(
                tol.approx_eq(s.center().z(), 5.0),
                "sphere center z should be 5.0"
            );
            assert!(
                tol.approx_eq(s.radius(), 3.0),
                "sphere radius should be unchanged"
            );
            found = true;
        }
    }
    assert!(found, "sphere face not found after transform");
}

#[test]
fn translate_torus_face_updates_center() {
    use brepkit_math::surfaces::ToroidalSurface;
    use brepkit_math::vec::Point3;

    let mut topo = Topology::new();
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 5.0, 1.5).unwrap();
    let solid = make_single_face_solid(&mut topo, FaceSurface::Torus(torus));

    let matrix = Mat4::translation(10.0, -3.0, 0.5);
    transform_solid(&mut topo, solid, &matrix).unwrap();

    let tol = Tolerance::new();
    let solid_data = topo.solid(solid).unwrap();
    let shell = topo.shell(solid_data.outer_shell()).unwrap();
    let mut found = false;
    for &fid in shell.faces() {
        if let FaceSurface::Torus(t) = topo.face(fid).unwrap().surface() {
            assert!(
                tol.approx_eq(t.center().x(), 10.0),
                "torus center x should be 10.0"
            );
            assert!(
                tol.approx_eq(t.center().y(), -3.0),
                "torus center y should be -3.0"
            );
            assert!(
                tol.approx_eq(t.center().z(), 0.5),
                "torus center z should be 0.5"
            );
            assert!(
                tol.approx_eq(t.major_radius(), 5.0),
                "torus major radius should be unchanged"
            );
            assert!(
                tol.approx_eq(t.minor_radius(), 1.5),
                "torus minor radius should be unchanged"
            );
            found = true;
        }
    }
    assert!(found, "torus face not found after transform");
}

#[test]
fn transform_direction_zero_vector_is_error() {
    // A zero direction vector cannot be normalized and must return an error.
    // This exercises the normalize() error branch in transform_direction.
    let result = super::transform_direction(&Mat4::identity(), Vec3::new(0.0, 0.0, 0.0));
    assert!(
        result.is_err(),
        "transform_direction with zero vector should return an error"
    );
}

#[test]
fn transform_direction_unit_z_identity_unchanged() {
    // Identity matrix should leave a unit direction unchanged.
    let dir = Vec3::new(0.0, 0.0, 1.0);
    let result = super::transform_direction(&Mat4::identity(), dir).unwrap();
    let tol = Tolerance::new();
    assert!(tol.approx_eq(result.z(), 1.0), "z should remain 1.0");
    assert!(tol.approx_eq(result.x(), 0.0), "x should remain 0.0");
    assert!(tol.approx_eq(result.y(), 0.0), "y should remain 0.0");
}

/// Revolving a face produces NURBS surfaces; translating the result
/// should move both vertices and NURBS control points.
#[test]
fn transform_nurbs_solid() {
    use brepkit_math::vec::Point3;
    use brepkit_topology::edge::{Edge, EdgeCurve};
    use brepkit_topology::face::Face;
    use brepkit_topology::vertex::Vertex;
    use brepkit_topology::wire::{OrientedEdge, Wire};

    let mut topo = Topology::new();

    // Build a NURBS-faced solid by lofting two offset squares: a smooth loft
    // produces genuine NURBS side surfaces (revolve of a polygonal profile is now
    // recognised as analytic cone/cylinder/plane bands, so it no longer yields
    // NURBS walls — see the revolve analytic-surface recognition).
    let square = |topo: &mut Topology, half: f64, z: f64| -> FaceId {
        let tol_val = 1e-10;
        let a = topo.add_vertex(Vertex::new(Point3::new(-half, -half, z), tol_val));
        let b = topo.add_vertex(Vertex::new(Point3::new(half, -half, z), tol_val));
        let c = topo.add_vertex(Vertex::new(Point3::new(half, half, z), tol_val));
        let d = topo.add_vertex(Vertex::new(Point3::new(-half, half, z), tol_val));
        let e0 = topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
        let e1 = topo.add_edge(Edge::new(b, c, EdgeCurve::Line));
        let e2 = topo.add_edge(Edge::new(c, d, EdgeCurve::Line));
        let e3 = topo.add_edge(Edge::new(d, a, EdgeCurve::Line));
        let wire = Wire::new(
            vec![
                OrientedEdge::new(e0, true),
                OrientedEdge::new(e1, true),
                OrientedEdge::new(e2, true),
                OrientedEdge::new(e3, true),
            ],
            true,
        )
        .unwrap();
        let wid = topo.add_wire(wire);
        topo.add_face(Face::new(
            wid,
            vec![],
            FaceSurface::Plane {
                normal: brepkit_math::vec::Vec3::new(0.0, 0.0, 1.0),
                d: z,
            },
        ))
    };
    let p0 = square(&mut topo, 3.0, 0.0);
    let p1 = square(&mut topo, 2.0, 2.0);
    let p2 = square(&mut topo, 3.0, 4.0);
    let solid = crate::loft::loft_smooth(&mut topo, &[p0, p1, p2]).unwrap();

    // Record a NURBS surface control point before the transform.
    let solid_data = topo.solid(solid).unwrap();
    let shell = topo.shell(solid_data.outer_shell()).unwrap();
    let mut original_nurbs_cp = None;
    for &fid in shell.faces() {
        let f = topo.face(fid).unwrap();
        if let FaceSurface::Nurbs(s) = f.surface() {
            original_nurbs_cp = Some(s.control_points()[0][0]);
            break;
        }
    }
    let original_cp = original_nurbs_cp.unwrap();

    // Translate by (10, 0, 0).
    let matrix = Mat4::translation(10.0, 0.0, 0.0);
    transform_solid(&mut topo, solid, &matrix).unwrap();

    // Verify NURBS control points have shifted.
    let tol = Tolerance::new();
    let solid_data = topo.solid(solid).unwrap();
    let shell = topo.shell(solid_data.outer_shell()).unwrap();
    let mut found = false;
    for &fid in shell.faces() {
        let f = topo.face(fid).unwrap();
        if let FaceSurface::Nurbs(s) = f.surface() {
            let cp = s.control_points()[0][0];
            assert!(
                tol.approx_eq(cp.x(), original_cp.x() + 10.0),
                "NURBS control point x should shift by 10, got {} (was {})",
                cp.x(),
                original_cp.x()
            );
            assert!(
                tol.approx_eq(cp.y(), original_cp.y()),
                "NURBS control point y should be unchanged"
            );
            assert!(
                tol.approx_eq(cp.z(), original_cp.z()),
                "NURBS control point z should be unchanged"
            );
            found = true;
            break;
        }
    }
    assert!(found, "should still have NURBS faces after transform");
}

#[test]
fn translate_wire() {
    use brepkit_math::vec::Point3;
    use brepkit_topology::builder::make_polygon_wire;

    let mut topo = Topology::new();
    let wire = make_polygon_wire(
        &mut topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();

    transform_wire(&mut topo, wire, &Mat4::translation(5.0, 0.0, 0.0)).unwrap();

    // All vertices should have x shifted by 5 (original x values were 0, 1, 1).
    let tol = Tolerance::new();
    let w = topo.wire(wire).unwrap();
    for oe in w.edges() {
        let edge = topo.edge(oe.edge()).unwrap();
        let x = topo.vertex(edge.start()).unwrap().point().x();
        assert!(
            tol.approx_eq(x, 5.0) || tol.approx_eq(x, 6.0),
            "vertex x should be 5.0 or 6.0 after translation, got {x}"
        );
    }
}

#[test]
fn degenerate_matrix_errors_for_wire() {
    use brepkit_math::vec::Point3;
    use brepkit_topology::builder::make_polygon_wire;

    let mut topo = Topology::new();
    let wire = make_polygon_wire(
        &mut topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();

    let result = transform_wire(&mut topo, wire, &Mat4::scale(0.0, 1.0, 1.0));
    assert!(result.is_err());
}

#[test]
fn translate_wire_with_circle_edge() {
    use brepkit_math::curves::Circle3D;
    use brepkit_math::vec::{Point3, Vec3};
    use brepkit_topology::edge::{Edge, EdgeCurve};
    use brepkit_topology::vertex::Vertex;
    use brepkit_topology::wire::{OrientedEdge, Wire};

    let mut topo = Topology::new();
    let v = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
    let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let edge = topo.add_edge(Edge::new(v, v, EdgeCurve::Circle(circle)));
    let wire = Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap();
    let wid = topo.add_wire(wire);

    transform_wire(&mut topo, wid, &Mat4::translation(5.0, 0.0, 0.0)).unwrap();

    // Vertex should be shifted.
    let tol = Tolerance::new();
    let pos = topo.vertex(v).unwrap().point();
    assert!(
        tol.approx_eq(pos.x(), 6.0),
        "vertex should be at x=6 after +5 translation, got {}",
        pos.x()
    );

    // Circle center should also be shifted.
    let w = topo.wire(wid).unwrap();
    let e = topo.edge(w.edges()[0].edge()).unwrap();
    assert!(
        matches!(e.curve(), EdgeCurve::Circle(_)),
        "expected Circle edge after transform"
    );
    if let EdgeCurve::Circle(c) = e.curve() {
        assert!(
            tol.approx_eq(c.center().x(), 5.0),
            "circle center should be at x=5, got {}",
            c.center().x()
        );
    }
}

// ── Degeneracy is a matter of shape, not of size ────────────────────────
//
// The guard used to compare the matrix DETERMINANT against
// `Tolerance.linear` (1e-7). A determinant is a volume ratio and that
// tolerance is a length, so for a uniform scale the test collapsed to
// `s³ <= 1e-7` and every `s <= 0.0046415888` was called degenerate — a
// millimetres-to-metres conversion among them. It failed CLOSED: a valid
// transform was refused outright.
//
// The cases below pin both halves: everything that merely resizes or
// reflects the model must be accepted at any magnitude, and everything that
// actually flattens it must still be refused.

/// A 2x3x5 box: volume 30 by hand, before anything in the kernel is asked.
const BOX_DIMS: (f64, f64, f64) = (2.0, 3.0, 5.0);
const BOX_VOLUME: f64 = 30.0;

/// Every corner of that box, written out rather than read back.
const BOX_CORNERS: [(f64, f64, f64); 8] = [
    (0.0, 0.0, 0.0),
    (2.0, 0.0, 0.0),
    (2.0, 3.0, 0.0),
    (0.0, 3.0, 0.0),
    (0.0, 0.0, 5.0),
    (2.0, 0.0, 5.0),
    (2.0, 3.0, 5.0),
    (0.0, 3.0, 5.0),
];

/// Apply `matrix` to a fresh box and check everything that must survive it:
/// each corner lands where the matrix says it does, the shell is still a
/// closed 2-manifold with no free or non-manifold edges, and the volume is
/// `|det| * 30` — the closed form, not another route through the kernel.
fn transformed_box_is_sound(matrix: &Mat4, what: &str) {
    use brepkit_math::vec::Point3;
    use brepkit_topology::adjacency::AdjacencyIndex;

    let (dx, dy, dz) = BOX_DIMS;
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, dx, dy, dz).unwrap();

    transform_solid(&mut topo, solid, matrix)
        .unwrap_or_else(|e| panic!("{what}: refused a non-degenerate transform: {e}"));

    // 1. Vertices. Hand-applied matrix vs the kernel's, corner by corner.
    let mut placed = vec![false; BOX_CORNERS.len()];
    let far = matrix.mul_point(Point3::new(dx, dy, dz));
    let origin = matrix.mul_point(Point3::new(0.0, 0.0, 0.0));
    let extent = (far - origin).length().max(f64::MIN_POSITIVE);
    for (_, v) in topo.vertices().iter() {
        let p = v.point();
        let hit = BOX_CORNERS.iter().position(|&(x, y, z)| {
            let want = matrix.mul_point(Point3::new(x, y, z));
            (p - want).length() <= 1e-12 * extent
        });
        let Some(i) = hit else {
            panic!("{what}: vertex {p:?} is not the image of any box corner");
        };
        assert!(!placed[i], "{what}: two vertices landed on corner {i}");
        placed[i] = true;
    }
    assert!(placed.iter().all(|&b| b), "{what}: a corner went missing");

    // 2. Topology. Resizing a body cannot change what it is.
    let adj = AdjacencyIndex::build(&topo, solid).unwrap();
    assert!(adj.is_manifold(), "{what}: shell is no longer 2-manifold");
    assert!(
        adj.boundary_edges().is_empty(),
        "{what}: {} free edge(s) after the transform",
        adj.boundary_edges().len()
    );
    assert!(
        adj.non_manifold_edges().is_empty(),
        "{what}: {} non-manifold edge(s) after the transform",
        adj.non_manifold_edges().len()
    );

    // 3. Volume. An affine map multiplies every volume by |det|.
    let want = BOX_VOLUME * matrix.determinant().abs();
    let diag = extent;
    let got = crate::measure::solid_volume(&topo, solid, diag * 1e-4).unwrap();
    let rel = (got.abs() - want).abs() / want;
    assert!(
        rel <= 1e-9,
        "{what}: volume {got:e} vs the closed form {want:e} (relative {rel:.3e})",
    );
}

#[test]
fn uniform_scales_are_accepted_across_every_decade() {
    // The old band's edge sat at s = 1e-7^(1/3) = 0.0046415888: 0.00465 was
    // the last accepted scale and 0.00464 the first refused. The sweep pins
    // both sides of it and then runs six more decades past, so the edge is
    // recorded rather than described. 1e-3 is the millimetres-to-metres case
    // the old band swallowed whole.
    for s in [
        1e3, 1e1, 1.0, 1e-1, 1e-2, 4.7e-3, 4.65e-3, 4.64e-3, 4.6e-3, 1e-3, 1e-4, 1e-6, 1e-9,
    ] {
        transformed_box_is_sound(&Mat4::scale(s, s, s), &format!("uniform {s:e}"));
    }
}

#[test]
fn anisotropic_scales_are_accepted_however_small_the_determinant() {
    // A determinant is a product, so an anisotropic scale reaches the old
    // band from a different direction: (1e-4, 1e-2, 1e-2) has |det| = 1e-8
    // and was refused even though nothing about it is degenerate. The
    // Hadamard ratio of every one of these is exactly 1.
    for (sx, sy, sz) in [
        (1e-4, 1e-2, 1e-2),
        (1e-6, 1.0, 1.0),
        (1e3, 1e-3, 1.0),
        (1e-5, 1e-5, 1e3),
        (2.0, 3.0, 5.0),
    ] {
        transformed_box_is_sound(
            &Mat4::scale(sx, sy, sz),
            &format!("anisotropic ({sx:e}, {sy:e}, {sz:e})"),
        );
    }
}

#[test]
fn reflections_are_accepted_at_every_scale_and_stay_accepted() {
    // Negative determinant. These were accepted before at 1x and must still
    // be — the fix must not change how orientation-reversing maps are
    // classified, only how small ones are.
    for (sx, sy, sz) in [
        (-1.0, 1.0, 1.0),
        (1.0, -1.0, 1.0),
        (-1.0, -1.0, -1.0),
        (-1e-3, 1e-3, 1e-3),
        (-1e-9, -1e-9, 1e-9),
    ] {
        transformed_box_is_sound(
            &Mat4::scale(sx, sy, sz),
            &format!("reflection ({sx:e}, {sy:e}, {sz:e})"),
        );
    }
}

#[test]
fn a_rotated_and_sheared_transform_is_still_accepted() {
    // Neither a pure scale nor axis-aligned: rotate, shear, scale down past
    // the old band, and translate. The shear's Hadamard ratio is 0.14 — well
    // clear of the floor, and the point is that the floor does not care what
    // the uniform factor in front of it is.
    let shear = Mat4([
        [1.0, 0.7, 0.3, 0.0],
        [0.0, 1.0, 0.9, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    // The translation scales with the body. It has to: `solid_volume`
    // integrates P·n over the faces in absolute coordinates, so parking a
    // 1e-4-sized body 7 units from the origin costs (7/6e-4)³ ~ 1e12 of the
    // 1e-16 relative precision to cancellation and the volume comes back
    // 5.6e-5 wrong — an offset-to-size conditioning limit of the divergence
    // integral, not anything about the transform. Scaling the placement with
    // the body keeps the configuration geometrically similar, which is what
    // this test is actually about; the residual is then ~1e-16 at every k.
    for k in [1.0f64, 1e-2, 1e-4, 1e-6, 1e-9] {
        let m = Mat4::translation(4.0 * k, -2.0 * k, 7.0 * k)
            .mul(Mat4::rotation_z(0.7))
            .mul(Mat4::rotation_x(-0.4))
            .mul(shear)
            .mul(Mat4::scale(k, k, k));
        transformed_box_is_sound(&m, &format!("rotate+shear+{k:e}"));
    }
}

#[test]
fn genuinely_degenerate_matrices_are_still_refused() {
    // Losing the real degeneracy check while removing the false one would be
    // the bad outcome. Each of these collapses space onto a plane, a line or
    // a point, at a range of magnitudes so that no absolute threshold could
    // be doing the work.
    let flatten_to_plane = |k: f64| Mat4::scale(k, k, 0.0);
    let flatten_to_line = |k: f64| Mat4::scale(k, 0.0, 0.0);
    // Rank 2: the third column is the sum of the first two, so the image is
    // a plane even though no single column is zero and every entry is O(1).
    let rank_two = Mat4([
        [1.0, 0.0, 1.0, 0.0],
        [0.0, 1.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    // Rank 2 to within 1e-14 — degenerate in shape, not in size, and the
    // magnitude is deliberately huge so that a determinant test would call
    // it healthy (|det| = 1e-2).
    let nearly_rank_two = Mat4([
        [1e6, 0.0, 1e6, 0.0],
        [0.0, 1e6, 1e6, 0.0],
        [0.0, 0.0, 1e-8, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);

    let cases: [(&str, Mat4); 8] = [
        ("flatten to plane 1x", flatten_to_plane(1.0)),
        ("flatten to plane 1000x", flatten_to_plane(1e3)),
        ("flatten to plane 0.001x", flatten_to_plane(1e-3)),
        ("flatten to line", flatten_to_line(1.0)),
        ("collapse to point", Mat4::scale(0.0, 0.0, 0.0)),
        ("rank two", rank_two),
        ("nearly rank two, large", nearly_rank_two),
        ("non-finite column", Mat4::scale(f64::NAN, 1.0, 1.0)),
    ];

    for (what, m) in cases {
        let mut topo = Topology::new();
        let (dx, dy, dz) = BOX_DIMS;
        let solid = crate::primitives::make_box(&mut topo, dx, dy, dz).unwrap();
        assert!(
            transform_solid(&mut topo, solid, &m).is_err(),
            "{what}: a matrix that collapses the model must be refused",
        );
        // The same verdict from every entry point that takes a matrix.
        let mut t2 = Topology::new();
        let s2 = crate::primitives::make_box(&mut t2, dx, dy, dz).unwrap();
        assert!(
            crate::copy::copy_and_transform_solid(&mut t2, s2, &m).is_err(),
            "{what}: copy_and_transform_solid must refuse it too",
        );
    }
}

#[test]
fn singular_non_affine_transforms_are_refused_without_mutation() {
    let matrix = Mat4([
        [1.0, 0.0, 0.0, 10.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 10.0],
        [0.0, 0.0, 0.0, 0.0],
    ]);

    let mut topo = Topology::new();
    let solid = make_single_face_solid(
        &mut topo,
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    );
    let before: Vec<_> = topo.vertices().iter().map(|(_, v)| v.point()).collect();

    assert!(transform_solid(&mut topo, solid, &matrix).is_err());
    let after: Vec<_> = topo.vertices().iter().map(|(_, v)| v.point()).collect();
    assert_eq!(after, before, "a rejected transform must be atomic");

    let face = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap()
        .faces()[0];
    assert!(transform_face(&mut topo, face, &matrix).is_err());
    let after_face: Vec<_> = topo.vertices().iter().map(|(_, v)| v.point()).collect();
    assert_eq!(
        after_face, before,
        "a rejected face transform must be atomic"
    );
}

#[test]
fn non_finite_and_projective_matrices_are_rejected() {
    for (what, matrix) in [
        (
            "NaN in the bottom row",
            Mat4([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [f64::NAN, 0.0, 0.0, 1.0],
            ]),
        ),
        (
            // Not reached by the linear-column check: the translation column
            // is not one of the three columns that get normalized.
            "NaN in the translation column",
            Mat4([
                [1.0, 0.0, 0.0, f64::NAN],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
        ),
        (
            "infinity in the linear part",
            Mat4([
                [f64::INFINITY, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
        ),
        (
            // A real perspective divide, not ulp noise: this is the row shape
            // the affine band exists to exclude.
            "perspective term in the bottom row",
            Mat4([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [1e-3, 0.0, 0.0, 1.0],
            ]),
        ),
        (
            "a homogeneous scale in w",
            Mat4([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 2.0],
            ]),
        ),
    ] {
        assert!(
            reject_degenerate_transform(&matrix).is_err(),
            "{what}: must be refused",
        );
    }
}

/// The regression behind the affine band: `Mat4::inverse` is an adjugate
/// inversion, so round-tripping a rigid frame through it yields a bottom row
/// that is *near* `[0, 0, 0, 1]` rather than exactly it. Every such matrix is
/// a legitimate transform and must keep being accepted.
#[test]
fn inverses_of_rigid_frames_are_accepted() {
    // The shape `push_pull::frame_matrix` produces before it normalizes an
    // operand into the canonical +Z frame.
    let frames = [
        rigid_frame(Vec3::new(0.3, 0.5, 0.81), Vec3::new(11.0, -4.0, 7.5)),
        rigid_frame(Vec3::new(-0.2, 0.97, 0.14), Vec3::new(-1e3, 250.0, 0.0)),
        rigid_frame(Vec3::new(1.0, 1.0, 1.0), Vec3::new(0.001, 0.002, -0.003)),
        rigid_frame(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 0.0)),
    ];

    for (i, frame) in frames.iter().enumerate() {
        assert!(
            reject_degenerate_transform(frame).is_ok(),
            "frame {i}: a rigid frame must be accepted",
        );

        let inverse = frame.inverse().unwrap();
        assert!(
            reject_degenerate_transform(&inverse).is_ok(),
            "frame {i}: Mat4::inverse of a rigid frame must be accepted, \
             bottom row was {:?}",
            inverse.0[3],
        );

        // And it must be usable, not merely pass the guard.
        let mut topo = Topology::new();
        let solid = crate::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
        transform_solid(&mut topo, solid, frame).unwrap();
        transform_solid(&mut topo, solid, &inverse).unwrap();
    }
}

/// An orthonormal frame rotating `+Z` onto `axis`, then translating.
fn rigid_frame(axis: Vec3, translation: Vec3) -> Mat4 {
    let z = axis.normalize().unwrap();
    // Any seed not parallel to `z` gives an orthonormal completion.
    let seed = if z.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let x = (seed - z * seed.dot(z)).normalize().unwrap();
    let y = z.cross(x);
    Mat4([
        [x.x(), y.x(), z.x(), translation.x()],
        [x.y(), y.y(), z.y(), translation.y()],
        [x.z(), y.z(), z.z(), translation.z()],
        [0.0, 0.0, 0.0, 1.0],
    ])
}

#[test]
fn the_degeneracy_verdict_does_not_move_with_the_units() {
    // The guard's whole contract in one assertion: multiplying a matrix by a
    // uniform scale changes |det| by s³ but never changes whether the
    // transform is degenerate.
    let proper = Mat4([
        [1.0, 0.2, 0.0, 0.0],
        [0.0, 1.0, 0.4, 0.0],
        [0.3, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    let flat = Mat4([
        [1.0, 0.2, 1.2, 0.0],
        [0.0, 1.0, 1.0, 0.0],
        [0.3, 0.0, 0.3, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    for k in [1e6, 1e3, 1.0, 1e-3, 1e-6, 1e-9] {
        let s = Mat4::scale(k, k, k);
        assert!(
            reject_degenerate_transform(&proper.mul(s)).is_ok(),
            "a proper transform became degenerate at {k:e}",
        );
        assert!(
            reject_degenerate_transform(&flat.mul(s)).is_err(),
            "a rank-deficient transform became proper at {k:e}",
        );
    }
}
