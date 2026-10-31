#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stderr,
    deprecated
)]

use std::collections::{HashMap, HashSet};

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::test_utils::make_unit_cube_manifold;
use remus_topology::validation::validate_shell_manifold;

use crate::test_utils::assert_euler_genus0;

use super::*;

fn solid_edge_ids(topo: &Topology, solid_id: SolidId) -> Vec<EdgeId> {
    let solid = topo.solid(solid_id).expect("test solid");
    let shell = topo.shell(solid.outer_shell()).expect("test shell");
    let mut seen = HashSet::new();
    let mut edges = Vec::new();
    for &fid in shell.faces() {
        let face = topo.face(fid).expect("test face");
        let wire = topo.wire(face.outer_wire()).expect("test wire");
        for oe in wire.edges() {
            if seen.insert(oe.edge().index()) {
                edges.push(oe.edge());
            }
        }
    }
    edges
}

#[test]
fn fillet_single_edge() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);

    let edges = solid_edge_ids(&topo, cube);
    let target = edges[0];

    let result = fillet(&mut topo, cube, &[target], 0.1).expect("fillet should succeed");

    let s = topo.solid(result).expect("result solid");
    let sh = topo.shell(s.outer_shell()).expect("shell");

    // 6 original + 1 fillet = 7 faces
    assert_eq!(
        sh.faces().len(),
        7,
        "expected 7 faces after single-edge fillet"
    );
}

#[test]
fn fillet_single_edge_euler() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    let edges = solid_edge_ids(&topo, cube);
    let result = fillet(&mut topo, cube, &[edges[0]], 0.1).expect("fillet should succeed");
    assert_euler_genus0(&topo, result);
}

#[test]
fn fillet_result_is_manifold() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);

    let edges = solid_edge_ids(&topo, cube);
    let result = fillet(&mut topo, cube, &[edges[0]], 0.1).expect("fillet should succeed");

    let s = topo.solid(result).expect("result solid");
    let sh = topo.shell(s.outer_shell()).expect("shell");
    validate_shell_manifold(sh, &topo).expect("fillet result should be manifold");
}

#[test]
fn fillet_zero_radius_error() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    let edges = solid_edge_ids(&topo, cube);
    assert!(fillet(&mut topo, cube, &[edges[0]], 0.0).is_err());
}

#[test]
fn fillet_negative_radius_error() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    let edges = solid_edge_ids(&topo, cube);
    assert!(fillet(&mut topo, cube, &[edges[0]], -0.1).is_err());
}

#[test]
fn fillet_no_edges_error() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    assert!(fillet(&mut topo, cube, &[], 0.1).is_err());
}

#[test]
fn radius_law_constant() {
    let law = FilletRadiusLaw::Constant(0.5);
    assert!((law.evaluate(0.0) - 0.5).abs() < 1e-10);
    assert!((law.evaluate(0.5) - 0.5).abs() < 1e-10);
    assert!((law.evaluate(1.0) - 0.5).abs() < 1e-10);
}

#[test]
fn radius_law_linear() {
    let law = FilletRadiusLaw::Linear {
        start: 0.1,
        end: 0.5,
    };
    assert!((law.evaluate(0.0) - 0.1).abs() < 1e-10);
    assert!((law.evaluate(0.5) - 0.3).abs() < 1e-10);
    assert!((law.evaluate(1.0) - 0.5).abs() < 1e-10);
}

#[test]
fn radius_law_scurve() {
    let law = FilletRadiusLaw::SCurve {
        start: 0.1,
        end: 0.5,
    };
    // S-curve should match endpoints
    assert!((law.evaluate(0.0) - 0.1).abs() < 1e-10);
    assert!((law.evaluate(1.0) - 0.5).abs() < 1e-10);
    // Midpoint should be between start and end
    let mid = law.evaluate(0.5);
    assert!(mid > 0.1 && mid < 0.5);
}

#[test]
fn fillet_variable_constant_law() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);

    let edges = solid_edge_ids(&topo, cube);
    let laws = vec![(edges[0], FilletRadiusLaw::Constant(0.1))];

    let result = fillet_variable(&mut topo, cube, &laws).expect("variable fillet should work");

    let s = topo.solid(result).expect("result solid");
    let sh = topo.shell(s.outer_shell()).expect("shell");
    assert_eq!(sh.faces().len(), 7, "should have 7 faces after fillet");
}

#[test]
fn fillet_variable_linear_law() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);

    let edges = solid_edge_ids(&topo, cube);
    let laws = vec![(
        edges[0],
        FilletRadiusLaw::Linear {
            start: 0.05,
            end: 0.15,
        },
    )];

    let result = fillet_variable(&mut topo, cube, &laws).expect("variable fillet should work");

    let vol = crate::measure::solid_volume(&topo, result, 0.1).unwrap();
    assert!(vol > 0.5, "filleted cube should have volume, got {vol}");
}

#[test]
fn fillet_variable_removes_material_linear_law() {
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edge_ids(&topo, solid);
    let laws = vec![(
        edges[0],
        FilletRadiusLaw::Linear {
            start: 0.5,
            end: 1.5,
        },
    )];
    let result = fillet_variable(&mut topo, solid, &laws).expect("variable fillet");
    let vol = crate::measure::solid_volume(&topo, result, 0.05).unwrap();
    assert!(vol < 1000.0, "fillet must remove material, got {vol}");
    assert!(
        vol > 900.0,
        "single-edge fillet removes only a sliver, got {vol}"
    );

    let s = topo.solid(result).expect("result solid");
    let sh = topo.shell(s.outer_shell()).expect("shell");
    validate_shell_manifold(sh, &topo).expect("variable fillet result should be manifold");
    assert_euler_genus0(&topo, result);
}

#[test]
fn fillet_has_positive_volume() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);

    let edges = solid_edge_ids(&topo, cube);
    let result = fillet(&mut topo, cube, &[edges[0]], 0.1).expect("fillet should succeed");

    let vol = crate::measure::solid_volume(&topo, result, 0.1).unwrap();
    assert!(
        vol > 0.5,
        "filleted cube should have significant volume, got {vol}"
    );
}

#[test]
fn rolling_ball_fillet_single_edge() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);

    let edges = solid_edge_ids(&topo, cube);
    let result = fillet_rolling_ball(&mut topo, cube, &[edges[0]], 0.1)
        .expect("rolling-ball fillet should succeed");

    let s = topo.solid(result).expect("result solid");
    let sh = topo.shell(s.outer_shell()).expect("shell");

    // 6 original faces + 1 NURBS fillet = 7 faces
    assert_eq!(
        sh.faces().len(),
        7,
        "expected 7 faces after single-edge rolling-ball fillet"
    );

    // Rolling-ball fillet on a box should still be genus-0 (χ=2).
    assert_euler_genus0(&topo, result);
}

/// A constant-radius blend along a straight edge between two planes IS a right
/// circular cylinder, so the engine emits one rather than a NURBS
/// approximation of one. `tests/regress_fillet_analytic_cylinder.rs` records
/// what rides on that exactness.
#[test]
fn rolling_ball_fillet_has_analytic_cylinder_face() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);

    let edges = solid_edge_ids(&topo, cube);
    let result = fillet_rolling_ball(&mut topo, cube, &[edges[0]], 0.1)
        .expect("rolling-ball fillet should succeed");

    let s = topo.solid(result).expect("result solid");
    let sh = topo.shell(s.outer_shell()).expect("shell");

    let blend = sh
        .faces()
        .iter()
        .find(|&&fid| {
            matches!(
                topo.face(fid).expect("face").surface(),
                FaceSurface::Cylinder(_)
            )
        })
        .expect("rolling-ball fillet should produce a cylindrical blend face");
    let FaceSurface::Cylinder(cyl) = topo.face(*blend).expect("face").surface() else {
        unreachable!("filtered to cylinders")
    };
    assert!(
        (cyl.radius() - 0.1).abs() < 1e-12,
        "the blend cylinder carries the requested radius, got {}",
        cyl.radius()
    );
    assert!(
        !sh.faces().iter().any(|&fid| matches!(
            topo.face(fid).expect("face").surface(),
            FaceSurface::Nurbs(_)
        )),
        "no face of a plane-to-plane fillet should be a b-spline"
    );
}

/// A cylinder's closed circular rim collapses its blend strip to zero area, and
/// `try_fillet` relies on the rolling-ball engine ERRORING to fall through to
/// the walking engine. The degeneracy guard now clears healthy faces on a cheap
/// boundary-polygon area first, so this pins that the shortcut cannot turn that
/// refusal into an acceptance and silently ship the corrupt solid.
///
/// Ported from upstream andymai/remus#1248, which asserts the guard's own
/// "degenerate face" `InvalidInput`. This fork refuses the same input EARLIER
/// and more specifically — `Blend(EdgesNotBlended)` from edge selection, before
/// any face is measured — so the assertion is on the refusal that matters to
/// the dispatcher rather than on which stage produced it.
#[test]
fn rolling_ball_rejects_closed_rim_so_dispatcher_falls_through() {
    let mut topo = Topology::new();
    let cyl = crate::primitives::make_cylinder(&mut topo, 10.0, 20.0).expect("cylinder");
    let edges = solid_edge_ids(&topo, cyl);

    let err = fillet_rolling_ball(&mut topo, cyl, &edges, 0.5)
        .expect_err("rolling-ball must reject a closed circular rim");
    assert!(
        matches!(
            err,
            crate::OperationsError::Blend(_) | crate::OperationsError::InvalidInput { .. }
        ),
        "the refusal must be one `try_fillet` falls through on, got: {err:?}"
    );
}

#[test]
fn rolling_ball_fillet_surface_is_circular_arc() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);

    let edges = solid_edge_ids(&topo, cube);
    let result = fillet_rolling_ball(&mut topo, cube, &[edges[0]], 0.2)
        .expect("rolling-ball fillet should succeed");

    let s = topo.solid(result).expect("result solid");
    let sh = topo.shell(s.outer_shell()).expect("shell");

    // Find the NURBS fillet face and verify it's a proper circular arc.
    for &fid in sh.faces() {
        let face = topo.face(fid).expect("face");
        if let FaceSurface::Nurbs(surface) = face.surface() {
            // The surface should be degree (2, 1) — circular arc × linear.
            assert_eq!(
                surface.degree_u(),
                2,
                "u (arc) direction should be degree 2"
            );
            assert_eq!(
                surface.degree_v(),
                1,
                "v (extrusion) direction should be degree 1"
            );

            // Evaluate at the midpoint (u=0.5, v=0.5) and check that
            // the point is at distance R from both adjacent faces.
            let mid_pt = surface.evaluate(0.5, 0.5);

            // For a unit cube, the fillet point should be inside the cube
            // (all coordinates between -0.1 and 1.1 for radius 0.2).
            assert!(
                mid_pt.x() > -0.5 && mid_pt.x() < 1.5,
                "fillet midpoint x should be near cube: {mid_pt:?}"
            );
        }
    }
}

#[test]
fn rolling_ball_fillet_positive_volume() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);

    let edges = solid_edge_ids(&topo, cube);
    let result =
        fillet_rolling_ball(&mut topo, cube, &[edges[0]], 0.1).expect("fillet should succeed");

    let vol = crate::measure::solid_volume(&topo, result, 0.1).unwrap();
    assert!(
        vol > 0.5,
        "filleted cube should have significant volume, got {vol}"
    );
}

#[test]
fn rolling_ball_fillet_multiple_edges() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);

    let edges = solid_edge_ids(&topo, cube);
    // edges[0] and edges[1] share a corner vertex.
    let result = fillet_rolling_ball(&mut topo, cube, &[edges[0], edges[1]], 0.1)
        .expect("multi-edge rolling-ball fillet should succeed");

    let s = topo.solid(result).expect("result solid");
    let sh = topo.shell(s.outer_shell()).expect("shell");

    // 6 trimmed planar + 2 fillet strips + the corner, which is TWO faces:
    // the octant of the corner ball, plus the planar ledge the third (still
    // sharp) edge runs out onto one radius below the shared face. Those two
    // used to be approximated by a single blended patch; splitting them is what
    // makes the corner exact. = 10 faces.
    assert_eq!(
        sh.faces().len(),
        10,
        "expected 10 faces after two-edge shared-corner rolling-ball fillet"
    );
    let spheres = sh
        .faces()
        .iter()
        .filter(|&&fid| matches!(topo.face(fid).unwrap().surface(), FaceSurface::Sphere(_)))
        .count();
    assert_eq!(
        spheres, 1,
        "the corner must be an exact ball octant, not an approximating patch"
    );
    validate_shell_manifold(sh, &topo).expect("two-edge fillet should be manifold");
}

/// Regression for #841: filleting two edges that share a corner vertex in a
/// single rolling-ball pass must produce a watertight (closed, euler-2) solid.
/// Previously the corner gap was left open (4 free edges, euler=1) because the
/// strips were set back but no corner patch closed the junction and the side
/// faces collapsed the unfilleted edge onto the far corner.
#[test]
fn rolling_ball_two_edges_shared_corner_watertight() {
    // Collect every pair of box edges that share a vertex, then fillet a sample.
    let mut topo = Topology::new();
    let cube = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let all_edges = solid_edge_ids(&topo, cube);

    // Find pairs of edges that share a vertex (a corner).
    let mut shared_pairs: Vec<(usize, usize)> = Vec::new();
    for i in 0..all_edges.len() {
        for j in (i + 1)..all_edges.len() {
            let ei = topo.edge(all_edges[i]).unwrap();
            let ej = topo.edge(all_edges[j]).unwrap();
            let shares = ei.start() == ej.start()
                || ei.start() == ej.end()
                || ei.end() == ej.start()
                || ei.end() == ej.end();
            if shares {
                shared_pairs.push((i, j));
            }
        }
    }
    assert!(
        !shared_pairs.is_empty(),
        "box should have corner-sharing edge pairs"
    );

    // Verify the first several corner-sharing pairs (iteration order); capped to
    // keep the test fast while still exercising multiple corners.
    for &(i, j) in shared_pairs.iter().take(6) {
        let mut t = Topology::new();
        let box_id = crate::primitives::make_box(&mut t, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edge_ids(&t, box_id);
        let result = fillet_rolling_ball(&mut t, box_id, &[edges[i], edges[j]], 1.0)
            .expect("fillet of shared-corner edges should succeed");

        let s = t.solid(result).expect("result solid");
        let sh = t.shell(s.outer_shell()).expect("shell");
        assert!(
            remus_topology::validation::validate_shell_closed(sh, &t).is_ok(),
            "shared-corner pair {i},{j} should be watertight"
        );
        assert!(
            validate_shell_manifold(sh, &t).is_ok(),
            "shared-corner pair {i},{j} should be manifold"
        );
        assert_euler_genus0(&t, result);

        let vol = crate::measure::solid_volume(&t, result, 0.01).unwrap();
        assert!(
            (985.0..1000.0).contains(&vol),
            "shared-corner pair {i},{j}: volume {vol} outside sane range"
        );
    }
}

/// A box edge pair that does NOT share a corner must remain watertight too
/// (the corner patch must not fire spuriously).
#[test]
fn rolling_ball_two_edges_no_shared_corner_watertight() {
    let mut topo = Topology::new();
    let cube = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edge_ids(&topo, cube);

    // Find a pair with no shared vertex.
    let mut pair = None;
    'outer: for i in 0..edges.len() {
        for j in (i + 1)..edges.len() {
            let ei = topo.edge(edges[i]).unwrap();
            let ej = topo.edge(edges[j]).unwrap();
            let shares = ei.start() == ej.start()
                || ei.start() == ej.end()
                || ei.end() == ej.start()
                || ei.end() == ej.end();
            if !shares {
                pair = Some((i, j));
                break 'outer;
            }
        }
    }
    let (i, j) = pair.expect("box should have a non-corner-sharing edge pair");

    let result = fillet_rolling_ball(&mut topo, cube, &[edges[i], edges[j]], 1.0)
        .expect("non-shared-corner fillet should succeed");
    let s = topo.solid(result).expect("result solid");
    let sh = topo.shell(s.outer_shell()).expect("shell");
    // 6 trimmed planar + 2 fillet strips = 8 faces, no corner patch.
    assert_eq!(sh.faces().len(), 8, "non-shared pair should add no patch");
    remus_topology::validation::validate_shell_closed(sh, &topo)
        .expect("non-shared-corner fillet should be watertight");
    assert_euler_genus0(&topo, result);
}

#[test]
fn rolling_ball_fillet_error_cases() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    let edges = solid_edge_ids(&topo, cube);

    assert!(fillet_rolling_ball(&mut topo, cube, &[edges[0]], 0.0).is_err());
    assert!(fillet_rolling_ball(&mut topo, cube, &[edges[0]], -0.1).is_err());
    assert!(fillet_rolling_ball(&mut topo, cube, &[], 0.1).is_err());
}

#[test]
fn vertex_blend_all_edges_box() {
    // Fillet all 12 edges of a unit cube → 8 vertex blend patches should
    // close the corners, giving a watertight mesh.
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    let edges = solid_edge_ids(&topo, cube);
    assert_eq!(edges.len(), 12, "unit cube should have 12 edges");

    let result =
        fillet_rolling_ball(&mut topo, cube, &edges, 0.1).expect("all-edges fillet should succeed");

    let s = topo.solid(result).expect("result solid");
    let sh = topo.shell(s.outer_shell()).expect("shell");

    // 6 trimmed planar faces + 12 NURBS fillet strips + 8 vertex blend triangles = 26
    assert_eq!(
        sh.faces().len(),
        26,
        "expected 26 faces (6 planar + 12 fillet + 8 blend)"
    );
}

#[test]
fn vertex_blend_all_edges_box_volume() {
    // Fillet all 12 edges of a 10×10×10 box at r=1.0. The result must be a
    // closed, manifold, genus-0 solid (26 faces) whose volume reflects only
    // the fillet-arc and corner-blend material removal — not gross corner
    // excision. Analytic expectation ≈ 975.6:
    //   1000 − 12·(1−π/4)·1²·8 (edge slivers ≈ 20.6)
    //        − 8·(1−π/6)·1³    (corner blends ≈ 3.8).
    let mut topo = Topology::new();
    let cube = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edge_ids(&topo, cube);
    assert_eq!(edges.len(), 12, "box should have 12 edges");

    let result =
        fillet_rolling_ball(&mut topo, cube, &edges, 1.0).expect("all-edges fillet should succeed");

    let s = topo.solid(result).expect("result solid");
    let sh = topo.shell(s.outer_shell()).expect("shell");
    validate_shell_manifold(sh, &topo).expect("filleted box should be manifold");
    assert_euler_genus0(&topo, result);

    assert_eq!(
        sh.faces().len(),
        26,
        "expected 26 faces (6 planar + 12 fillet + 8 blend)"
    );

    let vol = crate::measure::solid_volume(&topo, result, 0.01).unwrap();
    assert!(
        vol > 974.0 && vol < 978.0,
        "filleted box volume should be ≈975.6 (in 974..978), got {vol}"
    );
}

#[test]
fn vertex_blend_tessellates_successfully() {
    // Verify the fully-filleted box can be tessellated without error.
    // Watertight stitching at NURBS-to-planar seams is a tessellation-level
    // concern tracked separately.
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    let edges = solid_edge_ids(&topo, cube);

    let result =
        fillet_rolling_ball(&mut topo, cube, &edges, 0.1).expect("all-edges fillet should succeed");

    let mesh = crate::tessellate::tessellate_solid(&topo, result, 0.05).unwrap();
    // Should produce a non-trivial mesh.
    assert!(mesh.positions.len() > 20, "should have many vertices");
    assert!(mesh.indices.len() > 60, "should have many triangles");
}

#[test]
fn vertex_blend_positive_volume() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    let edges = solid_edge_ids(&topo, cube);

    let result =
        fillet_rolling_ball(&mut topo, cube, &edges, 0.1).expect("all-edges fillet should succeed");

    let vol = crate::measure::solid_volume(&topo, result, 0.005).unwrap();
    // Analytic rounded-cube volume for s=1, r=0.1 (Minkowski sum of the inner
    // (s−2r)³ box with a ball of radius r):
    //   (0.8)³ + 6·(0.8)²·0.1 + 12·0.8·(π/4)·0.1² + (4/3)π·0.1³ ≈ 0.9756.
    assert!(
        vol > 0.970 && vol < 0.980,
        "filleted unit-cube volume should be ≈0.9756 (in 0.970..0.980), got {vol}"
    );
}

#[test]
fn vertex_blend_box_primitive() {
    // Test with make_box (2×3×4) to verify non-unit dimensions work.
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let edges = solid_edge_ids(&topo, solid);
    assert_eq!(edges.len(), 12);

    let result = fillet_rolling_ball(&mut topo, solid, &edges, 0.2)
        .expect("box primitive all-edges fillet should succeed");

    let s = topo.solid(result).expect("result solid");
    let sh = topo.shell(s.outer_shell()).expect("shell");
    assert_eq!(sh.faces().len(), 26);
}

#[test]
fn vertex_blend_three_edges_at_corner() {
    // Fillet just the 3 edges meeting at one corner vertex to test minimal
    // vertex blend (produces one blend triangle).
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    let all_edges = solid_edge_ids(&topo, cube);

    // Find 3 edges sharing a common vertex.
    let mut vertex_to_edges: HashMap<usize, Vec<EdgeId>> = HashMap::new();
    for &eid in &all_edges {
        let e = topo.edge(eid).unwrap();
        vertex_to_edges
            .entry(e.start().index())
            .or_default()
            .push(eid);
        vertex_to_edges
            .entry(e.end().index())
            .or_default()
            .push(eid);
    }

    let (&_vi, corner_edges) = vertex_to_edges
        .iter()
        .find(|(_, edges)| edges.len() >= 3)
        .expect("box should have vertices with 3 edges");

    let targets: Vec<EdgeId> = corner_edges.iter().take(3).copied().collect();

    let result = fillet_rolling_ball(&mut topo, cube, &targets, 0.1)
        .expect("3-edge corner fillet should succeed");

    let s = topo.solid(result).expect("result solid");
    let sh = topo.shell(s.outer_shell()).expect("shell");

    // 6 original faces + 3 NURBS fillets + at least 1 vertex blend triangle
    assert!(
        sh.faces().len() >= 10,
        "expected at least 10 faces (6 + 3 + 1 blend), got {}",
        sh.faces().len()
    );
}

#[test]
fn vertex_blend_is_exact_sphere() {
    // Fillet all 12 edges of a unit cube. Every vertex blend must be an
    // analytic sphere face on the correct corner ball: center at the cube
    // corner offset inward by R along each face normal, radius exactly R.
    // (The former degree-(2,2) rational approximation sagged up to ~5% of R
    // mid-patch and folded at its degenerate corner, which rendered as a
    // pinched blob on every filleted corner.)
    let r = 0.1_f64;
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    let edges = solid_edge_ids(&topo, cube);

    let result =
        fillet_rolling_ball(&mut topo, cube, &edges, r).expect("all-edges fillet should succeed");

    let solid = topo.solid(result).unwrap();
    let shell = topo.shell(solid.outer_shell()).unwrap();

    let mut blend_face_count = 0;

    for &fid in shell.faces() {
        let face = topo.face(fid).unwrap();
        let FaceSurface::Sphere(sphere) = face.surface() else {
            continue;
        };
        blend_face_count += 1;

        assert!(
            (sphere.radius() - r).abs() < 1e-9,
            "corner ball radius should equal the fillet radius, got {}",
            sphere.radius()
        );

        // Expected center: nearest cube corner offset inward by R per axis.
        let c = sphere.center();
        let corner = Point3::new(
            if c.x() > 0.5 { 1.0 } else { 0.0 },
            if c.y() > 0.5 { 1.0 } else { 0.0 },
            if c.z() > 0.5 { 1.0 } else { 0.0 },
        );
        let expected = Point3::new(
            corner.x() + if corner.x() > 0.5 { -r } else { r },
            corner.y() + if corner.y() > 0.5 { -r } else { r },
            corner.z() + if corner.z() > 0.5 { -r } else { r },
        );
        assert!(
            (c - expected).length() < 1e-9,
            "corner ball center ({:.4},{:.4},{:.4}) should be at ({:.4},{:.4},{:.4})",
            c.x(),
            c.y(),
            c.z(),
            expected.x(),
            expected.y(),
            expected.z(),
        );

        // Every boundary vertex lies on the ball.
        let wire = topo.wire(face.outer_wire()).unwrap();
        for oe in wire.edges() {
            let v = topo.vertex(topo.edge(oe.edge()).unwrap().start()).unwrap();
            let dist = (v.point() - c).length();
            assert!(
                (dist - r).abs() < 1e-9,
                "corner cap boundary vertex should sit on the ball, dist {dist}"
            );
        }
    }

    assert_eq!(
        blend_face_count, 8,
        "expected 8 spherical vertex blend faces, found {blend_face_count}"
    );
}

#[test]
fn vertex_blend_cap_mesh_on_sphere_and_outward() {
    // Tessellate the fully filleted cube and check the corner caps at mesh
    // level: every triangle vertex of a sphere-surface face lies on the
    // corner ball (exact surface evaluation, not an approximation) and every
    // triangle faces outward — the folded-over slivers of the old rational
    // patch showed up here as normals pointing into the body.
    let r = 0.1_f64;
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    let edges = solid_edge_ids(&topo, cube);

    let result =
        fillet_rolling_ball(&mut topo, cube, &edges, r).expect("all-edges fillet should succeed");

    let (mesh, face_offsets) =
        crate::tessellate::tessellate_solid_grouped_with_tolerance(&topo, result, 0.002, 0.06)
            .expect("grouped tessellation should succeed");
    let face_ids = remus_topology::explorer::solid_faces(&topo, result).unwrap();
    assert_eq!(face_ids.len() + 1, face_offsets.len());

    let mut checked_faces = 0;
    for (i, &fid) in face_ids.iter().enumerate() {
        let face = topo.face(fid).unwrap();
        let FaceSurface::Sphere(sphere) = face.surface() else {
            continue;
        };
        checked_faces += 1;
        let center = sphere.center();
        let radius = sphere.radius();

        let start = face_offsets[i] as usize;
        let end = face_offsets[i + 1] as usize;
        assert!(end > start, "corner cap should produce triangles");
        for t in (start..end).step_by(3) {
            let pa = mesh.positions[mesh.indices[t] as usize];
            let pb = mesh.positions[mesh.indices[t + 1] as usize];
            let pc = mesh.positions[mesh.indices[t + 2] as usize];
            for p in [pa, pb, pc] {
                let dist = (p - center).length();
                assert!(
                    (dist - radius).abs() < 1e-6,
                    "corner cap mesh vertex off the ball by {}",
                    (dist - radius).abs()
                );
            }
            // Outward check: the geometric triangle normal must not point
            // toward the ball center (convex corner).
            let n = (pb - pa).cross(pc - pa);
            let mid = Point3::new(
                (pa.x() + pb.x() + pc.x()) / 3.0,
                (pa.y() + pb.y() + pc.y()) / 3.0,
                (pa.z() + pb.z() + pc.z()) / 3.0,
            );
            let radial = mid - center;
            assert!(
                n.dot(radial) > 0.0,
                "corner cap triangle folded over (normal points into the body)"
            );
        }
    }
    assert_eq!(checked_faces, 8, "expected 8 spherical corner caps");
}

#[test]
fn vertex_blend_cap_single_face_tessellation_trims_to_wire() {
    // The single-face tessellation path (`tessellate_face`) must trim the
    // spherical cap to its three boundary arcs rather than sweeping the UV
    // bounding-box rectangle: the rectangle over-covers a corner cap by tens
    // of percent of its area. The octant cap of a box corner has exact area
    // (4πR²)/8; the chordal mesh approaches it from below.
    let r = 0.1_f64;
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    let edges = solid_edge_ids(&topo, cube);

    let result =
        fillet_rolling_ball(&mut topo, cube, &edges, r).expect("all-edges fillet should succeed");

    let solid = topo.solid(result).unwrap();
    let shell = topo.shell(solid.outer_shell()).unwrap();
    let cap = shell
        .faces()
        .iter()
        .copied()
        .find(|&fid| {
            matches!(
                topo.face(fid).map(remus_topology::face::Face::surface),
                Ok(FaceSurface::Sphere(_))
            )
        })
        .expect("filleted cube should have a spherical corner cap");

    let mesh = crate::tessellate::tessellate(&topo, cap, 0.001)
        .expect("corner cap should tessellate standalone");
    let mut area = 0.0_f64;
    for t in (0..mesh.indices.len()).step_by(3) {
        let pa = mesh.positions[mesh.indices[t] as usize];
        let pb = mesh.positions[mesh.indices[t + 1] as usize];
        let pc = mesh.positions[mesh.indices[t + 2] as usize];
        area += (pb - pa).cross(pc - pa).length() * 0.5;
    }
    let octant = 4.0 * std::f64::consts::PI * r * r / 8.0;
    assert!(
        (area - octant).abs() < octant * 0.05,
        "corner cap standalone mesh area {area:.6} should be ≈ octant {octant:.6}"
    );
}

#[test]
fn vertex_blend_sphere_stays_inside_solid() {
    // The corner ball is tangent to the three face planes from inside, so the
    // whole cap — now exactly on the ball — must stay within the original
    // bounds. (The rational-patch predecessor needed an R/2 overshoot
    // allowance here.)
    let r = 0.1_f64;
    let margin = 1e-9;
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    let edges = solid_edge_ids(&topo, cube);

    let result =
        fillet_rolling_ball(&mut topo, cube, &edges, r).expect("all-edges fillet should succeed");

    let solid = topo.solid(result).unwrap();
    let shell = topo.shell(solid.outer_shell()).unwrap();

    let mut cap_count = 0;
    for &fid in shell.faces() {
        let face = topo.face(fid).unwrap();
        let FaceSurface::Sphere(sphere) = face.surface() else {
            continue;
        };
        cap_count += 1;
        let c = sphere.center();
        let rr = sphere.radius();
        for (lo, hi, v) in [(0.0, 1.0, c.x()), (0.0, 1.0, c.y()), (0.0, 1.0, c.z())] {
            assert!(
                v - rr >= lo - margin && v + rr <= hi + margin,
                "corner ball (center coord {v}, radius {rr}) protrudes past [{lo},{hi}]"
            );
        }
    }
    assert_eq!(cap_count, 8, "expected 8 spherical corner caps");
}

/// Fillet on a boolean result: fuse(box, cylinder) → fillet should work
/// on edges shared between two planar faces.
#[test]
fn fillet_on_boolean_result() {
    let mut topo = Topology::new();
    let base = crate::primitives::make_box(&mut topo, 80.0, 60.0, 10.0).unwrap();
    let boss = crate::primitives::make_cylinder(&mut topo, 15.0, 30.0).unwrap();
    let mat = remus_math::mat::Mat4::translation(40.0, 30.0, 10.0);
    crate::transform::transform_solid(&mut topo, boss, &mat).unwrap();

    let fused =
        crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Fuse, base, boss).unwrap();

    let solid = topo.solid(fused).unwrap();
    let shell = topo.shell(solid.outer_shell()).unwrap();

    // Build edge-to-face map from all wires (outer + inner).
    let mut edge_to_face_ids: HashMap<usize, Vec<FaceId>> = HashMap::new();
    for &fid in shell.faces() {
        let face = topo.face(fid).unwrap();
        let wire = topo.wire(face.outer_wire()).unwrap();
        for oe in wire.edges() {
            edge_to_face_ids
                .entry(oe.edge().index())
                .or_default()
                .push(fid);
        }
        for &iwid in face.inner_wires() {
            let iw = topo.wire(iwid).unwrap();
            for oe in iw.edges() {
                edge_to_face_ids
                    .entry(oe.edge().index())
                    .or_default()
                    .push(fid);
            }
        }
    }

    // Allow a small number of seam edges from cylindrical band discretization.
    let bad_count = edge_to_face_ids.values().filter(|f| f.len() != 2).count();
    assert!(
        bad_count <= 4,
        "too many non-manifold edges: {bad_count} (expected <= 4 seam edges)",
    );

    // Fillet only manifold edges where BOTH adjacent faces are planar.
    let is_planar = |fid: FaceId| -> bool {
        matches!(topo.face(fid).unwrap().surface(), FaceSurface::Plane { .. })
    };
    let mut planar_edges = Vec::new();
    for (&eidx, face_ids) in &edge_to_face_ids {
        if face_ids.len() == 2 && is_planar(face_ids[0]) && is_planar(face_ids[1]) {
            let face = topo.face(face_ids[0]).unwrap();
            let wire = topo.wire(face.outer_wire()).unwrap();
            for oe in wire.edges() {
                if oe.edge().index() == eidx {
                    planar_edges.push(oe.edge());
                    break;
                }
            }
        }
    }
    planar_edges.sort_unstable_by_key(|e| e.index());
    planar_edges.dedup_by_key(|e| e.index());

    assert!(
        !planar_edges.is_empty(),
        "should have planar-planar edges to fillet"
    );

    // The deprecated flat-bevel engine cannot rebuild this boolean-result
    // topology: historically it returned `Ok` with a shell in two connected
    // components and 90 free edges, and this test accepted it because it only
    // asserted `is_ok()`. The fail-closed contract is: `Ok` requires a result
    // that validates cleanly against the input, anything else must be a typed
    // refusal. Today it refuses; pin that, and prove the production engine
    // covers the scenario instead.
    let result = fillet(&mut topo, fused, &planar_edges, 1.0);
    match result {
        Ok(s) => {
            let report = remus_check::validate::validate_solid(
                &topo,
                s,
                &remus_check::validate::ValidateOptions::default(),
            )
            .unwrap();
            assert!(
                report.is_valid(),
                "an accepted flat-bevel fillet must be valid: {:#?}",
                report.issues
            );
        }
        Err(error) => {
            assert!(
                !crate::blend_ops::blend_failure_code(&error).is_empty(),
                "the refusal must carry a machine-readable code, got {error}"
            );
        }
    }

    // The same selection through the production engine succeeds, is valid,
    // and removes the rounded slivers — (1−π/4)·r² per unit length per edge.
    let expected_removed: f64 = planar_edges
        .iter()
        .map(|&e| {
            let edge = topo.edge(e).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            (b - a).length()
        })
        .sum::<f64>()
        * (1.0 - std::f64::consts::FRAC_PI_4);
    let fused_volume = crate::measure::solid_volume(&topo, fused, 0.05).unwrap();
    let v2 = crate::blend_ops::fillet_v2(&mut topo, fused, &planar_edges, 1.0)
        .expect("the walking engine fillets a boolean result's planar edges");
    let report = remus_check::validate::validate_solid(
        &topo,
        v2.solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(
        report.is_valid(),
        "v2 result must be valid: {:#?}",
        report.issues
    );
    let volume = crate::measure::solid_volume(&topo, v2.solid, 0.05).unwrap();
    assert!(
        (volume - (fused_volume - expected_removed)).abs() < expected_removed * 0.3,
        "expected ≈{expected_removed:.2} mm³ removed from {fused_volume:.2}, got {volume:.2}"
    );
}

#[test]
fn fillet_support_cliff_rejected() {
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let edges = solid_edge_ids(&topo, solid);
    // Unit cube has edge length 2.0 — a radius of 3.0 exceeds adjacent edges.
    let result = fillet_rolling_ball(&mut topo, solid, &edges[..1], 3.0);
    assert!(result.is_err(), "should reject radius exceeding face size");
    assert!(
        matches!(
            &result,
            Err(crate::OperationsError::Blend(
                remus_blend::BlendError::CliffEncountered { .. }
            ))
        ),
        "oversized radius should have a typed cliff refusal"
    );
    if let Err(crate::OperationsError::Blend(remus_blend::BlendError::CliffEncountered {
        edge,
        face,
        requested_radius,
        available_radius,
    })) = result
    {
        assert_eq!(edge, edges[0]);
        assert!(
            remus_topology::explorer::solid_faces(&topo, solid)
                .unwrap()
                .contains(&face)
        );
        assert!((requested_radius - 3.0).abs() < 1e-12);
        assert!((available_radius - 2.0).abs() < 1e-12);
    }
}

#[test]
fn fillet_radius_exceeds_cylinder_curvature_rejected() {
    // A cylinder of radius 1.0 cannot be filleted with radius >= 1.0:
    // the offset surface would degenerate to a line.
    let mut topo = Topology::new();
    let solid = crate::primitives::make_cylinder(&mut topo, 1.0, 4.0).unwrap();
    let plane_cyl_edge = {
        let s = topo.solid(solid).unwrap();
        let sh = topo.shell(s.outer_shell()).unwrap();
        let mut edge_faces: HashMap<usize, Vec<FaceId>> = HashMap::new();
        for &fid in sh.faces() {
            let wire = topo.wire(topo.face(fid).unwrap().outer_wire()).unwrap();
            for oe in wire.edges() {
                edge_faces.entry(oe.edge().index()).or_default().push(fid);
            }
        }
        let mut found = None;
        'outer: for (&eidx, fids) in &edge_faces {
            if fids.len() == 2 {
                let s1 = topo.face(fids[0]).unwrap().surface().clone();
                let s2 = topo.face(fids[1]).unwrap().surface().clone();
                let has_plane = matches!(s1, FaceSurface::Plane { .. })
                    || matches!(s2, FaceSurface::Plane { .. });
                let has_cyl = matches!(s1, FaceSurface::Cylinder(_))
                    || matches!(s2, FaceSurface::Cylinder(_));
                if has_plane && has_cyl {
                    for &fid in sh.faces() {
                        let wire = topo.wire(topo.face(fid).unwrap().outer_wire()).unwrap();
                        for oe in wire.edges() {
                            if oe.edge().index() == eidx {
                                found = Some(oe.edge());
                                break 'outer;
                            }
                        }
                    }
                }
            }
        }
        found.expect("cylinder must have a plane-cylinder edge")
    };

    // radius == cylinder radius → curvature radius exactly met → reject.
    let result = fillet_rolling_ball(&mut topo, solid, &[plane_cyl_edge], 1.0);
    assert!(
        result.is_err(),
        "radius == cylinder radius should be rejected"
    );
    let msg = format!("{}", result.unwrap_err());
    assert!(
        msg.contains("curvature"),
        "error should mention curvature: {msg}"
    );

    // radius > cylinder radius → also rejected.
    let result2 = fillet_rolling_ball(&mut topo, solid, &[plane_cyl_edge], 1.5);
    assert!(
        result2.is_err(),
        "radius > cylinder radius should be rejected"
    );

    // radius < cylinder radius → passes curvature check (may succeed or fail
    // for other fillet reasons, but must not fail on curvature).
    let result3 = fillet_rolling_ball(&mut topo, solid, &[plane_cyl_edge], 0.3);
    if let Err(ref e) = result3 {
        let msg = format!("{e}");
        assert!(
            !msg.contains("curvature"),
            "small radius should not fail curvature check: {msg}"
        );
    }
}

#[test]
fn fillet_radius_just_fits() {
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    let edges = solid_edge_ids(&topo, solid);
    // Edge length is 4.0 — a radius of 1.0 should fit comfortably.
    let result = fillet_rolling_ball(&mut topo, solid, &edges[..1], 1.0);
    assert!(
        result.is_ok(),
        "small radius should succeed: {:?}",
        result.err()
    );
}

#[test]
fn fillet_plane_cylinder_edge() {
    // A cylinder's rim edges are CLOSED circles (start vertex == end vertex).
    // The rolling-ball engine is built around polygonal wires with distinct
    // corner vertices; a closed circular edge collapses its blend strip and the
    // adjacent disc cap to zero-area faces (the result is manifold but
    // geometrically degenerate — it silently drops the caps and shrinks the
    // wall, removing ~35% of the volume; see gh #967). The engine must reject
    // this rather than return the broken solid, so the dispatcher
    // (`wasm::try_fillet`) falls through to the walking blend engine
    // (`blend_ops::fillet_v2`), which rounds a closed rim correctly.
    let mut topo = Topology::new();
    let solid = crate::primitives::make_cylinder(&mut topo, 2.0, 4.0).unwrap();

    // Find edges that border both a planar face and a cylindrical face.
    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();
    let mut plane_cyl_edges: Vec<EdgeId> = Vec::new();
    let mut edge_faces: HashMap<usize, Vec<FaceId>> = HashMap::new();

    for &fid in sh.faces() {
        let face = topo.face(fid).unwrap();
        let wire = topo.wire(face.outer_wire()).unwrap();
        for oe in wire.edges() {
            edge_faces.entry(oe.edge().index()).or_default().push(fid);
        }
    }

    for (&eidx, fids) in &edge_faces {
        if fids.len() == 2 {
            let s1 = topo.face(fids[0]).unwrap().surface().clone();
            let s2 = topo.face(fids[1]).unwrap().surface().clone();
            let has_plane =
                matches!(s1, FaceSurface::Plane { .. }) || matches!(s2, FaceSurface::Plane { .. });
            let has_cyl =
                matches!(s1, FaceSurface::Cylinder(_)) || matches!(s2, FaceSurface::Cylinder(_));
            if has_plane && has_cyl {
                // Recover the EdgeId from eidx — walk the shell to find it.
                for &fid in sh.faces() {
                    let face = topo.face(fid).unwrap();
                    let wire = topo.wire(face.outer_wire()).unwrap();
                    for oe in wire.edges() {
                        if oe.edge().index() == eidx {
                            plane_cyl_edges.push(oe.edge());
                        }
                    }
                }
            }
        }
    }

    plane_cyl_edges.sort_unstable_by_key(|edge| edge.index());
    plane_cyl_edges.dedup_by_key(|edge| edge.index());

    assert_eq!(
        plane_cyl_edges.len(),
        2,
        "cylinder should have two plane-cylinder rim edges"
    );

    // Filleting the closed rim circle must be REJECTED (not return a degenerate
    // solid). The rejection lets the dispatcher fall through to the walking
    // engine, which rounds the rim into an exact quarter-torus (gh #967).
    for edge in plane_cyl_edges {
        let result = fillet_rolling_ball(&mut topo, solid, &[edge], 0.3);
        assert!(
            result.is_err(),
            "rolling-ball fillet of closed cylinder-rim edge {edge:?} must be rejected as \
             degenerate, not return a silently-broken solid"
        );
        let msg = format!("{}", result.err().unwrap());
        assert!(
            msg.contains("degenerate"),
            "expected a degenerate-face rejection for {edge:?}, got: {msg}"
        );
    }
}

#[test]
fn g1_propagate_box_no_expansion() {
    // On a box every edge meets its neighbors at 90°.
    // Seeding one edge should yield a set of size exactly 1 — no expansion.
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let edges = solid_edge_ids(&topo, solid);
    let seed = &edges[..1];

    // expand_g1_chain is private; exercise it via the public wrapper.
    // We expect the wrapper to succeed and the fillet result to be valid
    // (same as seeding with the single edge directly).
    let result = fillet_rolling_ball_propagate_g1(&mut topo, solid, seed, 0.1);
    assert!(
        result.is_ok(),
        "propagate_g1 on a box edge should succeed: {:?}",
        result.err()
    );
    let result_solid = result.unwrap();
    let vol = crate::measure::solid_volume(&topo, result_solid, 0.01).unwrap();
    assert!(
        vol > 0.0 && vol < 1.0,
        "filleted box should have smaller volume than original: {vol}"
    );
}

#[test]
fn g1_propagate_collinear_long_box() {
    // Build a long box (4×1×1) and seed one of the long top edges.
    // A box's long edges are each a single edge, so propagation still
    // yields size 1 — but the wrapper should succeed and produce a valid solid.
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, 4.0, 1.0, 1.0).unwrap();

    // Pick the first edge that is parallel to the X axis (length ≈ 4.0).
    let edges = solid_edge_ids(&topo, solid);
    let long_edge = edges
        .iter()
        .find(|&&eid| {
            let e = topo.edge(eid).unwrap();
            let p0 = topo.vertex(e.start()).unwrap().point();
            let p1 = topo.vertex(e.end()).unwrap().point();
            let len = (p1 - p0).length();
            len > 3.5
        })
        .copied();
    let seed_edge = long_edge.expect("could not find a long edge on a 4×1×1 box");

    let result = fillet_rolling_ball_propagate_g1(&mut topo, solid, &[seed_edge], 0.1);
    assert!(
        result.is_ok(),
        "propagate_g1 on long-box edge should succeed: {:?}",
        result.err()
    );
    let result_solid = result.unwrap();
    let vol = crate::measure::solid_volume(&topo, result_solid, 0.01).unwrap();
    assert!(
        vol > 0.0 && vol < 4.0,
        "filleted long box volume should be positive and less than original: {vol}"
    );
}

#[test]
fn adjacent_fillet_overlap_all_edges_rejected() {
    // A 1×1×1 box with all 12 edges filleted at R=0.5 must fail:
    // at each 90° corner the setback from each end is R/tan(45°) = 0.5,
    // and 0.5 + 0.5 = 1.0 = edge length → strips exactly touch → reject.
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let edges = solid_edge_ids(&topo, solid);
    let result = fillet_rolling_ball(&mut topo, solid, &edges, 0.5);
    assert!(
        result.is_err(),
        "all-edge fillet with R=0.5 on unit box should be rejected"
    );
    let msg = format!("{}", result.unwrap_err());
    assert!(
        msg.contains("adjacent fillet strips overlap"),
        "error should mention overlap: {msg}"
    );
}

#[test]
fn adjacent_fillet_overlap_fits_with_small_radius() {
    // The same box with R=0.4 must succeed: 0.4+0.4=0.8 < 1.0.
    // (We don't check the full solid validity here — that's covered by
    // vertex_blend_all_edges_box.  Just verify Phase 2d does not block it.)
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let edges = solid_edge_ids(&topo, solid);
    let result = fillet_rolling_ball(&mut topo, solid, &edges, 0.4);
    assert!(
        result.is_ok(),
        "all-edge fillet with R=0.4 on unit box should be accepted by Phase 2d: {:?}",
        result.err()
    );
}

#[test]
fn adjacent_fillet_single_edge_no_phase2d_rejection() {
    // Filleting one edge of a box has no adjacent target edges — Phase 2d
    // never fires even for R close to the face size.
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let edges = solid_edge_ids(&topo, solid);
    // Phase 2b caps R at the adjacent edge length (1.0). R=0.4 is well below that.
    let result = fillet_rolling_ball(&mut topo, solid, &edges[..1], 0.4);
    assert!(
        result.is_ok(),
        "single-edge fillet with R=0.4 should succeed: {:?}",
        result.err()
    );
}

#[test]
fn face_surface_normal_at_nurbs_via_projection() {
    // Directly test the NURBS branch of face_surface_normal_at.
    // Build a flat bilinear NURBS patch in the XY plane.  The outward
    // normal is (0, 0, 1) everywhere; point projection must return a
    // valid (u, v) that yields the correct normal.
    let srf = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        ],
        vec![vec![1.0, 1.0], vec![1.0, 1.0]],
    )
    .expect("bilinear XY patch");

    let surface = FaceSurface::Nurbs(srf);

    // Test at a non-central point to ensure projection (not midpoint) is used.
    let n = face_surface_normal_at(&surface, Point3::new(0.2, 0.8, 0.0));
    let n = n.expect("NURBS normal should be Some for a surface point");

    // Flat XY patch has normal along ±Z.
    assert!(
        n.z().abs() > 0.9,
        "flat XY patch normal must be along Z, got: {n:?}"
    );
    // Result must be approximately unit length.
    assert!(
        (n.length() - 1.0).abs() < 0.01,
        "NURBS normal must be unit length, got: {}",
        n.length()
    );
}

#[test]
fn fillet_rolling_ball_second_pass_on_blended_solid() {
    // After a rolling-ball fillet the result solid carries a blend face whose
    // surface is not one of the box's planes. A second fillet on a different
    // manifold edge must succeed: this verifies that a `face_surfaces` map
    // holding non-planar entries does not crash `fillet_rolling_ball`.
    let mut topo = Topology::new();
    let solid = make_unit_cube_manifold(&mut topo);

    let edges1 = solid_edge_ids(&topo, solid);
    let result1 = fillet_rolling_ball(&mut topo, solid, &[edges1[0]], 0.1)
        .expect("first rolling-ball fillet should succeed");

    // Confirm the blend face was created, as the analytic cylinder it is.
    let has_blend = {
        let s = topo.solid(result1).unwrap();
        let sh = topo.shell(s.outer_shell()).unwrap();
        sh.faces()
            .iter()
            .any(|&fid| matches!(topo.face(fid).unwrap().surface(), FaceSurface::Cylinder(_)))
    };
    assert!(has_blend, "first fillet must produce a blend face");

    // Second fillet on a different edge — one with a real dihedral to round.
    //
    // This used to pick `edges2[1]`, which is one of the first blend's G1
    // CONTACT lines: the engine cannot round a tangent joint, skipped it, and
    // returned a rebuilt solid carrying no second blend at all. The test passed
    // on a fillet that had not happened. Pick an edge the selection filter
    // agrees is blendable, and check the blend is actually there.
    let edges2 = solid_edge_ids(&topo, result1);
    let filletable =
        crate::query::filter_filletable_edges(&topo, result1, &edges2).expect("filletable edges");
    let target = *filletable
        .first()
        .expect("the once-filleted cube still has blendable edges");
    let faces_before = {
        let s = topo.solid(result1).unwrap();
        topo.shell(s.outer_shell()).unwrap().faces().len()
    };
    // Fail-closed contract: the engine must either return a result that is
    // fully valid against its input or refuse with a typed error. On this
    // target the walking builder also refuses (`TrimmingFailure`), and the
    // rolling-ball's historical "success" carried 6 orientation-inconsistent
    // shared edges — closed and manifold, but non-orientable. It is a typed
    // refusal now; if the engine is ever repaired, the success branch's
    // oracles must all hold.
    let result2 = fillet_rolling_ball(&mut topo, result1, &[target], 0.05);
    match result2 {
        Ok(result2) => {
            let report = remus_check::validate::validate_solid(
                &topo,
                result2,
                &remus_check::validate::ValidateOptions::default(),
            )
            .unwrap();
            assert!(
                report.is_valid(),
                "an accepted second fillet must be valid: {:#?}",
                report.issues
            );
            let vol = crate::measure::solid_volume(&topo, result2, 0.1).unwrap();
            assert!(
                vol > 0.5,
                "doubly-filleted solid must have positive volume, got {vol}"
            );
            let faces_after = {
                let s = topo.solid(result2).unwrap();
                topo.shell(s.outer_shell()).unwrap().faces().len()
            };
            assert!(
                faces_after > faces_before,
                "the second fillet must add a blend face ({faces_before} -> {faces_after})"
            );
        }
        Err(error) => {
            assert!(
                !crate::blend_ops::blend_failure_code(&error).is_empty(),
                "the refusal must carry a machine-readable code, got {error}"
            );
            // A refused second pass must leave its input exactly as it was.
            let report = remus_check::validate::validate_solid(
                &topo,
                result1,
                &remus_check::validate::ValidateOptions::default(),
            )
            .unwrap();
            assert!(
                report.is_valid(),
                "the input must still validate after a refused second fillet: {:#?}",
                report.issues
            );
        }
    }

    // And an edge the engine CANNOT round is now refused by name rather than
    // quietly left out of an otherwise successful rebuild.
    let tangent = edges2
        .iter()
        .copied()
        .find(|e| !filletable.contains(e))
        .expect("the first blend leaves two tangent contact lines");
    let refused = fillet_rolling_ball(&mut topo, result1, &[tangent], 0.05);
    assert!(
        matches!(
            refused,
            Err(crate::OperationsError::Blend(
                remus_blend::BlendError::EdgesNotBlended { .. }
            ))
        ),
        "a tangent contact line must be refused by name, got: {:?}",
        refused.map(|_| "Ok")
    );
}

#[test]
fn fillet_on_fillet_box() {
    // Fillet all 12 edges of a box, then fillet the resulting NURBS edges.
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let edges = solid_edge_ids(&topo, solid);

    let result1 = fillet_rolling_ball(&mut topo, solid, &edges, 0.1).unwrap();
    let vol1 = crate::measure::solid_volume(&topo, result1, 0.01).unwrap();
    assert!(vol1 > 0.0, "first fillet should produce positive volume");

    let edges2 = solid_edge_ids(&topo, result1);
    assert!(
        !edges2.is_empty(),
        "filleted solid must have edges for second fillet attempt"
    );

    // Try to fillet one of the new NURBS-NURBS edges with smaller radius.
    // This should not panic or error — it's the #39 test.
    let result2 = fillet_rolling_ball(&mut topo, result1, &edges2[..1], 0.05);
    match result2 {
        Ok(solid2) => {
            let vol2 = crate::measure::solid_volume(&topo, solid2, 0.01).unwrap();
            assert!(vol2 > 0.0, "second fillet should produce positive volume");
        }
        Err(e) => {
            // Graceful failure is acceptable — log the error for diagnostics
            eprintln!("second fillet failed gracefully: {e}");
        }
    }
}

#[test]
fn adjacent_fillet_overlap_curved_face_detected() {
    // Two small fillets on a cylinder face that would overlap.
    let mut topo = Topology::new();
    let cyl = crate::primitives::make_cylinder(&mut topo, 1.0, 2.0).unwrap();

    // Get edges - the cylinder has circle edges at top and bottom
    let edges = solid_edge_ids(&topo, cyl);

    // Try to fillet with a very large radius that should trigger overlap
    // on the curved cylinder face
    let result = fillet_rolling_ball(&mut topo, cyl, &edges, 0.9);
    // Should either succeed (if no overlap) or return an error (if overlap detected)
    // The key is: it should NOT panic
    match result {
        Ok(solid) => {
            let vol = crate::measure::solid_volume(&topo, solid, 0.01).unwrap();
            assert!(vol > 0.0);
        }
        Err(e) => {
            // A large radius on the closed cylinder rim is expected to fail.
            // Acceptable reasons: overlap / curvature bound, or the degenerate
            // closed-circular-edge rejection (the rim collapses the blend strip;
            // see gh #967). The key is that it must NOT panic.
            let msg = format!("{e}");
            assert!(
                msg.contains("overlap")
                    || msg.contains("curvature")
                    || msg.contains("exceeds")
                    || msg.contains("degenerate")
                    || msg.contains("not blended"),
                "expected overlap/curvature/degenerate/not-blended error, got: {msg}"
            );
        }
    }
}

#[test]
fn g1_chain_no_expansion_for_box() {
    // Box edges meet at 90 degrees — no G1 chains should be detected.
    // Filleting a single edge should succeed without expanding.
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let edges = solid_edge_ids(&topo, solid);

    // Fillet a single edge with a small radius.
    let result = fillet_rolling_ball(&mut topo, solid, &edges[..1], 0.1);
    assert!(
        result.is_ok(),
        "single-edge fillet on box should succeed: {:?}",
        result.err()
    );

    let result_solid = result.unwrap();
    let vol = crate::measure::solid_volume(&topo, result_solid, 0.01).unwrap();
    // Original box volume: 8.0; fillet removes a small amount.
    assert!(
        vol > 7.0 && vol < 8.0,
        "filleted box volume should be slightly less than 8.0, got {vol}"
    );
}

#[test]
fn g1_chain_integrated_matches_explicit_wrapper() {
    // Since fillet_rolling_ball now does G1 expansion internally,
    // calling it directly on a seed should give the same result as
    // the explicit propagate_g1 wrapper.
    let mut topo1 = Topology::new();
    let solid1 = crate::primitives::make_box(&mut topo1, 1.0, 1.0, 1.0).unwrap();
    let edges1 = solid_edge_ids(&topo1, solid1);
    let result1 = fillet_rolling_ball(&mut topo1, solid1, &edges1[..1], 0.1);
    assert!(result1.is_ok(), "direct call should succeed");
    let vol1 = crate::measure::solid_volume(&topo1, result1.unwrap(), 0.01).unwrap();

    let mut topo2 = Topology::new();
    let solid2 = crate::primitives::make_box(&mut topo2, 1.0, 1.0, 1.0).unwrap();
    let edges2 = solid_edge_ids(&topo2, solid2);
    let result2 = fillet_rolling_ball_propagate_g1(&mut topo2, solid2, &edges2[..1], 0.1);
    assert!(result2.is_ok(), "wrapper call should succeed");
    let vol2 = crate::measure::solid_volume(&topo2, result2.unwrap(), 0.01).unwrap();

    // Both should produce the same volume (within tolerance).
    assert!(
        (vol1 - vol2).abs() < 0.01,
        "volumes should match: direct={vol1}, wrapper={vol2}"
    );
}

/// Dihedral angle (degrees) between the two faces of an edge, sampled at the
/// edge midpoint, using effective (reversal-adjusted) outward normals.
fn dihedral_deg(topo: &Topology, e: EdgeId, fs: &[FaceId]) -> f64 {
    let ed = topo.edge(e).unwrap();
    let a = topo.vertex(ed.start()).unwrap().point();
    let b = topo.vertex(ed.end()).unwrap().point();
    let mid = a + (b - a) * 0.5;
    let nrm = |fid: FaceId| {
        let face = topo.face(fid).unwrap();
        let n = match face.surface() {
            FaceSurface::Plane { normal, .. } => *normal,
            other => {
                let (u, v) = other.project_point(mid).unwrap_or((0.0, 0.0));
                other.normal(u, v)
            }
        };
        let n = if face.is_reversed() { -n } else { n };
        n.normalize().unwrap()
    };
    nrm(fs[0])
        .dot(nrm(fs[1]))
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees()
}

/// #834: round an edge whose neighbour is a previous fillet's blend face.
///
/// After a single-edge rolling-ball fillet, the arc where the blend band
/// meets the top face is tangent-continuous with the two box edges beside
/// it: the band is tangent to both walls. A seed on that arc therefore
/// expands across the tangent seams onto both box edges, and the second
/// fillet rounds the whole ridgeline into a valid, watertight manifold —
/// convex, so it removes material.
#[test]
fn fillet_edge_adjacent_to_blend_is_watertight() {
    use remus_topology::validation::validate_shell_closed;

    let mut topo = Topology::new();
    let cube = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edge_ids(&topo, cube);

    // First fillet — one box edge → a watertight solid with a blend face.
    let first = fillet_rolling_ball(&mut topo, cube, &[edges[0]], 1.0).unwrap();
    {
        let sh = topo
            .shell(topo.solid(first).unwrap().outer_shell())
            .unwrap();
        validate_shell_closed(sh, &topo).expect("first fillet should be watertight");
    }
    let vol1 = crate::measure::solid_volume(&topo, first, 0.05).unwrap();

    // Collect blend faces and edge→faces adjacency. The box has no curved face
    // of its own, so every cylinder in the result came from the blend.
    let blend: HashSet<usize> = {
        let sh = topo
            .shell(topo.solid(first).unwrap().outer_shell())
            .unwrap();
        sh.faces()
            .iter()
            .filter(|&&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
            .map(|f| f.index())
            .collect()
    };
    assert!(!blend.is_empty(), "first fillet must create a blend face");

    let mut ef: HashMap<usize, Vec<FaceId>> = HashMap::new();
    {
        let sh = topo
            .shell(topo.solid(first).unwrap().outer_shell())
            .unwrap();
        for &fid in sh.faces() {
            for oe in topo
                .wire(topo.face(fid).unwrap().outer_wire())
                .unwrap()
                .edges()
            {
                ef.entry(oe.edge().index()).or_default().push(fid);
            }
        }
    }

    // A non-tangent edge bordering the blend face (the tangent contact lines
    // are G1/degenerate and are not fillettable).
    let target = solid_edge_ids(&topo, first)
        .into_iter()
        .find(|&e| {
            ef.get(&e.index()).is_some_and(|fs| {
                fs.len() == 2
                    && fs.iter().any(|f| blend.contains(&f.index()))
                    && dihedral_deg(&topo, e, fs) > 5.0
            })
        })
        .expect("a non-tangent edge bordering the blend face");

    // The seed runs across both tangent seams: arc plus the two box edges.
    let chain = remus_blend::g1_chain::expand_g1_chain(
        &topo,
        first,
        &[target],
        remus_math::tolerance::Tolerance::new(),
    )
    .unwrap();
    assert_eq!(
        chain.len(),
        3,
        "arc seed must expand onto both box edges: {chain:?}"
    );

    let result = fillet_rolling_ball(&mut topo, first, &[target], 0.5)
        .expect("second fillet on a blend-adjacent edge must succeed");
    let report = remus_check::validate::validate_solid(
        &topo,
        result,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(
        report.is_valid(),
        "the blend-adjacent fillet must be fully valid: {:#?}",
        report.issues
    );
    let sh = topo
        .shell(topo.solid(result).unwrap().outer_shell())
        .unwrap();
    validate_shell_manifold(sh, &topo).expect("second fillet must be manifold");
    validate_shell_closed(sh, &topo)
        .expect("second fillet on a blend-adjacent edge must be watertight");

    // Two 9-long box edges and the corner arc, all convex: material leaves.
    let vol2 = crate::measure::solid_volume(&topo, result, 0.05).unwrap();
    let straight = (1.0 - std::f64::consts::FRAC_PI_4) * 0.25 * 18.0;
    assert!(
        vol2 < vol1 - straight && vol2 > vol1 - straight - 0.5,
        "second fillet must remove its straight runs' material: first={vol1}, second={vol2}"
    );
}

/// Endpoints of edge `e`.
fn edge_endpoints(topo: &Topology, e: EdgeId) -> (Point3, Point3) {
    let ed = topo.edge(e).unwrap();
    (
        topo.vertex(ed.start()).unwrap().point(),
        topo.vertex(ed.end()).unwrap().point(),
    )
}

/// The vertical box edge at `x = dx, y = 0`.
fn box_vertical_edge(topo: &Topology, solid: SolidId, dx: f64) -> EdgeId {
    let near = |a: f64, b: f64| (a - b).abs() < 1e-9;
    solid_edge_ids(topo, solid)
        .into_iter()
        .find(|&e| {
            let (a, b) = edge_endpoints(topo, e);
            near(a.x(), dx) && near(b.x(), dx) && near(a.y(), 0.0) && near(b.y(), 0.0)
        })
        .expect("vertical edge at x=dx, y=0")
}

/// The straight top box edge at `x = dx, z = dz`.
fn box_top_edge(topo: &Topology, solid: SolidId, dx: f64, dz: f64) -> EdgeId {
    let near = |a: f64, b: f64| (a - b).abs() < 1e-9;
    solid_edge_ids(topo, solid)
        .into_iter()
        .find(|&e| {
            let (a, b) = edge_endpoints(topo, e);
            matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Line)
                && near(a.x(), dx)
                && near(b.x(), dx)
                && near(a.z(), dz)
                && near(b.z(), dz)
        })
        .expect("top edge at x=dx, z=dz")
}

/// Closed-form volume of the box with a vertical edge filleted at `r1` and,
/// afterwards, the top edge beside it filleted at `r2` through the
/// production cascade: the top fillet propagates along its tangent ridgeline
/// across the earlier band's arc onto the far top edge. Returns the measured
/// result and its solid.
fn sequential_top_fillet(
    topo: &mut Topology,
    (dx, dy, dz): (f64, f64, f64),
    r1: f64,
    r2: f64,
) -> (SolidId, SolidId) {
    let solid = crate::primitives::make_box(topo, dx, dy, dz).unwrap();
    let vertical = box_vertical_edge(topo, solid, dx);
    let first = crate::blend_ops::fillet_cascade(topo, solid, &[vertical], r1)
        .expect("first fillet")
        .solid;
    let top = box_top_edge(topo, first, dx, dz);

    // Propagation across the band: line → arc → line.
    let chain = remus_blend::g1_chain::expand_g1_chain(
        topo,
        first,
        &[top],
        remus_math::tolerance::Tolerance::new(),
    )
    .unwrap();
    assert_eq!(
        chain.len(),
        3,
        "top seed must run across the band's arc: {chain:?}"
    );
    let arcs = chain
        .iter()
        .filter(|&&e| matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Circle(_)))
        .count();
    assert_eq!(arcs, 1, "the chain carries exactly the band's arc");

    let second = crate::blend_ops::fillet_cascade(topo, first, &[top], r2)
        .expect("the top edge must fillet after the vertical one")
        .solid;
    (first, second)
}

fn assert_fully_valid(topo: &Topology, solid: SolidId, what: &str) {
    let report = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(
        report.is_valid(),
        "{what} must be fully valid: {:#?}",
        report.issues
    );
    let sh = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    let closed = remus_topology::validation::validate_shell_closed(sh, topo);
    assert!(
        closed.is_ok(),
        "{what} must be watertight: {:?}",
        closed.err()
    );
}

/// A fillet on a box edge that ends on an earlier fillet of the same
/// radius (the interactive default: fillet one edge, then the next).
///
/// The rolling ball's centre runs along the earlier band's axis, so the
/// corner is a sphere octant: the same shape as blending all three corner
/// edges at once, which the engine already emits as an exact sphere.
#[test]
fn sequential_fillet_equal_radius_rounds_corner_as_sphere() {
    let dims = (9.5, 12.5, 8.5);
    let (dx, dy, dz) = dims;
    let mut topo = Topology::new();
    let (first, second) = sequential_top_fillet(&mut topo, dims, 1.0, 1.0);
    assert_fully_valid(&topo, second, "sequential equal-radius fillet");

    // Same topology as the three-edge corner blend: 3 cylinders + 1 cap.
    let counts = remus_topology::explorer::solid_entity_counts(&topo, second).unwrap();
    assert_eq!(counts, (10, 21, 13), "faces, edges, vertices");

    // Closed form: three quarter-cylinder runs plus a sphere-octant corner.
    let straight = (1.0 - std::f64::consts::FRAC_PI_4) * ((dz - 1.0) + (dy - 1.0) + (dx - 1.0));
    let corner = 1.0 - std::f64::consts::FRAC_PI_6;
    let expected = dx * dy * dz - straight - corner;
    let vol = crate::measure::solid_volume(&topo, second, 0.01).unwrap();
    assert!(
        (vol - expected).abs() < 0.05,
        "sequential fillet volume {vol} should match closed form {expected}"
    );
    let vol_first = crate::measure::solid_volume(&topo, first, 0.01).unwrap();
    assert!(vol < vol_first, "a convex fillet removes material");

    // The corner cap is the ball itself, an exact sphere centred on the
    // earlier band's axis one radius down, bounded by three great circles.
    let centre = Point3::new(dx - 1.0, 1.0, dz - 1.0);
    let caps = exact_sphere_caps(&topo, second);
    assert_eq!(caps.len(), 1, "one exact sphere corner");
    let (cap, sphere) = &caps[0];
    assert!((sphere.center() - centre).length() < 1e-9);
    assert!((sphere.radius() - 1.0).abs() < 1e-12);
    assert_great_circle_boundary(&topo, *cap, sphere);

    // The same body as blending all three corner edges at once.
    let mut direct_topo = Topology::new();
    let solid = crate::primitives::make_box(&mut direct_topo, dx, dy, dz).unwrap();
    let edges = [
        box_vertical_edge(&direct_topo, solid, dx),
        box_top_edge(&direct_topo, solid, dx, dz),
        find_line_edge(
            &direct_topo,
            solid,
            Point3::new(0.0, 0.0, dz),
            Point3::new(dx, 0.0, dz),
        ),
    ];
    let direct = crate::blend_ops::fillet_cascade(&mut direct_topo, solid, &edges, 1.0)
        .unwrap()
        .solid;
    assert_eq!(
        counts,
        solid_entity_counts(&direct_topo, direct).unwrap(),
        "same topology as the three-edge corner blend"
    );
    assert_eq!(
        surface_census(&topo, second),
        surface_census(&direct_topo, direct)
    );
    let direct_vol = crate::measure::solid_volume(&direct_topo, direct, 0.01).unwrap();
    assert!(
        (vol - direct_vol).abs() < 1e-3,
        "sequential {vol} should match all-at-once {direct_vol}"
    );
}

/// Every sphere face of `solid`, after asserting no face of it is NURBS.
fn exact_sphere_caps(
    topo: &Topology,
    solid: SolidId,
) -> Vec<(FaceId, remus_math::surfaces::SphericalSurface)> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|f| match topo.face(f).unwrap().surface() {
            FaceSurface::Nurbs(_) => panic!("face {} is an approximate NURBS patch", f.index()),
            FaceSurface::Sphere(sphere) => Some((f, sphere.clone())),
            _ => None,
        })
        .collect()
}

/// `cap` is a spherical triangle: three circle edges, each a great circle of
/// `sphere`.
fn assert_great_circle_boundary(
    topo: &Topology,
    cap: FaceId,
    sphere: &remus_math::surfaces::SphericalSurface,
) {
    let wire = topo.wire(topo.face(cap).unwrap().outer_wire()).unwrap();
    assert_eq!(wire.edges().len(), 3, "a three-sided sphere patch");
    for oe in wire.edges() {
        let EdgeCurve::Circle(circle) = topo.edge(oe.edge()).unwrap().curve() else {
            panic!("sphere patch edge must be a circle");
        };
        assert!((circle.center() - sphere.center()).length() < 1e-9);
        assert!((circle.radius() - sphere.radius()).abs() < 1e-9);
    }
}

/// The straight edge of `solid` between `a` and `b`, either way round.
fn find_line_edge(topo: &Topology, solid: SolidId, a: Point3, b: Point3) -> EdgeId {
    solid_edge_ids(topo, solid)
        .into_iter()
        .find(|&e| {
            let (p, q) = edge_endpoints(topo, e);
            matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Line)
                && (((p - a).length() < 1e-6 && (q - b).length() < 1e-6)
                    || ((p - b).length() < 1e-6 && (q - a).length() < 1e-6))
        })
        .unwrap_or_else(|| panic!("no straight edge {a:?} -> {b:?}"))
}

/// An L-step blank, `w` wide: a step `d1` deep and `h1` tall in front of a
/// wall reaching back to `d2` and up to `h2`, fused from two boxes.
fn l_step_blank(topo: &mut Topology, (w, d1, d2, h1, h2): (f64, f64, f64, f64, f64)) -> SolidId {
    let step = crate::primitives::make_box(topo, w, d1 + 1.0, h1).unwrap();
    let wall = crate::primitives::make_box(topo, w, d2 - d1, h2).unwrap();
    crate::transform::transform_solid(
        topo,
        wall,
        &remus_math::mat::Mat4::translation(0.0, d1, 0.0),
    )
    .unwrap();
    crate::boolean::boolean(topo, crate::boolean::BooleanOp::Fuse, step, wall).unwrap()
}

/// The audit-plate fillet order on an L-step: R1 on the concave step/wall
/// edge, R1 on both of the step's side top edges (which run on over the
/// concave band and up the wall's vertical edges), then R1 on the step's
/// front top edge. That last chain runs over the side bands' front arcs and
/// down the front vertical edges, and its two arc stretches have no torus:
/// they are sphere triangles, the same corners blending the front edges
/// together with the side top edges gives.
#[test]
fn sequential_l_step_front_fillet_rounds_band_ends_as_spheres() {
    let dims = (12.0, 8.0, 11.0, 4.0, 9.0);
    let (w, d1, _, h1, _) = dims;
    let p = Point3::new;
    let mut topo = Topology::new();
    let blank = l_step_blank(&mut topo, dims);
    let concave = find_line_edge(&topo, blank, p(0.0, d1, h1), p(w, d1, h1));
    let stepped = crate::blend_ops::fillet_cascade(&mut topo, blank, &[concave], 1.0)
        .unwrap()
        .solid;
    let side_edges = |topo: &Topology| {
        [
            find_line_edge(topo, stepped, p(0.0, 0.0, h1), p(0.0, d1 - 1.0, h1)),
            find_line_edge(topo, stepped, p(w, 0.0, h1), p(w, d1 - 1.0, h1)),
        ]
    };
    let sides = side_edges(&topo);
    let banded = crate::blend_ops::fillet_cascade(&mut topo, stepped, &sides, 1.0)
        .unwrap()
        .solid;
    let front = find_line_edge(&topo, banded, p(1.0, 0.0, h1), p(w - 1.0, 0.0, h1));
    let result = crate::blend_ops::fillet_cascade(&mut topo, banded, &[front], 1.0)
        .unwrap()
        .solid;
    assert_fully_valid(&topo, result, "L-step front fillet after the side bands");

    let caps = exact_sphere_caps(&topo, result);
    assert_eq!(caps.len(), 2, "one exact sphere corner per side band");
    for (cap, sphere) in &caps {
        let c = sphere.center();
        assert!(
            ((c.x() - 1.0).abs() < 1e-9 || (c.x() - (w - 1.0)).abs() < 1e-9)
                && (c.y() - 1.0).abs() < 1e-9
                && (c.z() - (h1 - 1.0)).abs() < 1e-9,
            "ball centre {c:?} must sit on a side band's axis"
        );
        assert!((sphere.radius() - 1.0).abs() < 1e-12);
        assert_great_circle_boundary(&topo, *cap, sphere);
    }

    // Closed form: the concave band adds its section along the full width;
    // each side band removes its section along the step's top and the wall's
    // vertical edge, plus a torus quarter turn about the concave band; the
    // front chain removes its section along the front edge and both front
    // verticals, plus π/12 more than the side bands already took at each
    // sphere corner.
    let (w, d1, _, h1, h2) = dims;
    let (section, inset) = quarter_fillet_section(1.0);
    let side = section * ((d1 - 1.0) + (h2 - (h1 + 1.0)))
        + section * std::f64::consts::FRAC_PI_2 * (1.0 + inset);
    let front_chain = section * ((w - 2.0) + 2.0 * (h1 - 1.0)) + std::f64::consts::FRAC_PI_6;
    let expected = 708.0 + section * w - 2.0 * side - front_chain;
    let vol = crate::measure::solid_volume(&topo, result, 0.01).unwrap();
    assert!(
        (vol - expected).abs() < 1e-3,
        "volume {vol} should match closed form {expected}"
    );

    // The same body as filleting the side and front edges together, the
    // front verticals the sequential chain reaches included.
    let mut direct_topo = Topology::new();
    let blank = l_step_blank(&mut direct_topo, dims);
    let concave = find_line_edge(&direct_topo, blank, p(0.0, d1, h1), p(w, d1, h1));
    let stepped = crate::blend_ops::fillet_cascade(&mut direct_topo, blank, &[concave], 1.0)
        .unwrap()
        .solid;
    let [left, right] = side_edges(&direct_topo);
    let edges = [
        left,
        right,
        find_line_edge(&direct_topo, stepped, p(0.0, 0.0, h1), p(w, 0.0, h1)),
        find_line_edge(&direct_topo, stepped, p(0.0, 0.0, 0.0), p(0.0, 0.0, h1)),
        find_line_edge(&direct_topo, stepped, p(w, 0.0, 0.0), p(w, 0.0, h1)),
    ];
    let direct = crate::blend_ops::fillet_cascade(&mut direct_topo, stepped, &edges, 1.0)
        .unwrap()
        .solid;
    assert_eq!(
        solid_entity_counts(&topo, result).unwrap(),
        solid_entity_counts(&direct_topo, direct).unwrap(),
        "same topology as filleting the remaining edges at once"
    );
    assert_eq!(
        surface_census(&topo, result),
        surface_census(&direct_topo, direct)
    );
    let direct_vol = crate::measure::solid_volume(&direct_topo, direct, 0.01).unwrap();
    assert!(
        (vol - direct_vol).abs() < 1e-3,
        "sequential {vol} should match all-at-once {direct_vol}"
    );
}

/// The audit plate: an L-step part with through holes whose step side top
/// edges, concave step/wall edge and wall vertical edges already carry R1
/// fillets. Returns the body and the straight top edge, 54.5 long, at
/// height `z` on the front plane `y`.
fn audit_plate(topo: &mut Topology, y: f64, z: f64) -> (SolidId, EdgeId) {
    let solid = remus_io::arena_io::deserialize_solid(
        include_bytes!("../../tests/data/audit_plate_body.arena.json"),
        topo,
    )
    .unwrap();
    let edge = find_line_edge(topo, solid, Point3::new(1.0, y, z), Point3::new(55.5, y, z));
    (solid, edge)
}

/// The audit plate's step front top edge and upper front top edge at the
/// earlier bands' radius. Each chain runs over two band arcs whose torus
/// would have no ring; both corners are exact sphere triangles, so the
/// result is all-exact, and the removed material matches the closed form.
#[test]
fn audit_plate_equal_radius_fillets_are_exact() {
    // (front plane, height, straight chain length): the step's front edge
    // runs on down both front verticals, the upper one back along both top
    // side edges.
    for (y, z, straight) in [
        (-4.5, 11.0, 54.5 + 2.0 * 10.0),
        (21.5, 37.0, 54.5 + 2.0 * 28.5),
    ] {
        let mut topo = Topology::new();
        let (input, edge) = audit_plate(&mut topo, y, z);
        let before = crate::measure::solid_volume(&topo, input, 0.01).unwrap();
        assert!((before - 44_820.27).abs() < 0.01, "fixture volume {before}");
        assert_fully_valid(&topo, input, "audit plate fixture");

        let result = crate::blend_ops::fillet_cascade(&mut topo, input, &[edge], 1.0)
            .unwrap()
            .solid;
        assert_fully_valid(&topo, result, "audit plate R1 fillet");
        let caps = exact_sphere_caps(&topo, result);
        assert_eq!(caps.len(), 2, "edge at y={y}: two exact sphere corners");
        for (cap, sphere) in &caps {
            assert!((sphere.radius() - 1.0).abs() < 1e-12);
            assert_great_circle_boundary(&topo, *cap, sphere);
        }
        assert_eq!(solid_entity_counts(&topo, result).unwrap(), (29, 71, 44));

        let (section, _) = quarter_fillet_section(1.0);
        let removed = section * straight + std::f64::consts::FRAC_PI_6;
        let vol = crate::measure::solid_volume(&topo, result, 0.01).unwrap();
        assert!(
            (before - vol - removed).abs() < 2e-3,
            "edge at y={y}: removed {} should match closed form {removed}",
            before - vol
        );

        // Deterministic: a fresh run serializes to the same bytes.
        let mut again = Topology::new();
        let (input, edge) = audit_plate(&mut again, y, z);
        let repeat = crate::blend_ops::fillet_cascade(&mut again, input, &[edge], 1.0)
            .unwrap()
            .solid;
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, result).unwrap(),
            remus_io::arena_io::serialize_solid(&again, repeat).unwrap()
        );
    }
}

/// Between half the band radius and the band radius the arc stretch needs a
/// spindle torus, so the engine lofts it. The audit reported such results
/// removing less material than R0.49 does; that was the volume reading, not
/// the shape. With a NURBS face present `solid_volume` measures the closed
/// whole-solid mesh, and this plate's through holes bias that mesh by about
/// 2000 × deflection mm³. Measured at one deflection on both bodies, the
/// removal is the closed form.
#[test]
fn audit_plate_spindle_range_fillet_removes_closed_form_material() {
    let r = 0.8;
    let mut topo = Topology::new();
    let (input, edge) = audit_plate(&mut topo, -4.5, 11.0);
    let result = crate::blend_ops::fillet_cascade(&mut topo, input, &[edge], r)
        .unwrap()
        .solid;
    assert_fully_valid(&topo, result, "audit plate R0.8 fillet");
    let mesh_volume = |solid: SolidId| {
        let mesh = crate::tessellate::tessellate_solid(&topo, solid, 0.001).unwrap();
        mesh.indices
            .chunks(3)
            .map(|t| {
                let [a, b, c] = [0, 1, 2].map(|i| {
                    let p = mesh.positions[t[i] as usize];
                    Vec3::new(p.x(), p.y(), p.z())
                });
                a.dot(b.cross(c)) / 6.0
            })
            .sum::<f64>()
    };
    let removed = mesh_volume(input) - mesh_volume(result);

    // Straight runs along the front edge and both verticals, plus the
    // section swept a quarter turn about each side band's axis.
    let (section, inset) = quarter_fillet_section(r);
    let expected =
        section * (54.5 + 2.0 * 10.0) + 2.0 * section * std::f64::consts::FRAC_PI_2 * (1.0 - inset);
    assert!(
        (removed - expected).abs() < 0.05,
        "R{r} removes {removed}, closed form {expected}"
    );
}

/// The removed area and its centroid's inset from the corner, per unit length,
/// of a radius-`r` fillet on a right-angle edge.
fn quarter_fillet_section(r: f64) -> (f64, f64) {
    let section = (1.0 - std::f64::consts::FRAC_PI_4) * r * r;
    let square = r * r * r / 2.0;
    let quarter_disc =
        std::f64::consts::FRAC_PI_4 * r * r * (r - 4.0 * r / (3.0 * std::f64::consts::PI));
    (section, (square - quarter_disc) / section)
}

/// The same sequence with a larger second radius. No rolling ball of the
/// larger radius can follow the earlier band's arc, so the earlier band is
/// rebuilt after the new fillet: the result is the one filleting the larger
/// edge first gives, whose closed form is the smaller-radius corner below with
/// the roles of the two edges swapped.
#[test]
fn sequential_fillet_larger_radius_rebuilds_earlier_band() {
    let dims = (9.5, 12.5, 8.5);
    let (dx, dy, dz) = dims;
    let (small, big) = (1.0, 2.0);
    let mut topo = Topology::new();
    let (_, healed) = sequential_top_fillet(&mut topo, dims, small, big);
    assert_fully_valid(&topo, healed, "larger fillet after a smaller one");

    // Largest first: the top edge at `big`, then the vertical edge at `small`,
    // which runs across the top band's arc onto the front top edge.
    let mut direct_topo = Topology::new();
    let solid = crate::primitives::make_box(&mut direct_topo, dx, dy, dz).unwrap();
    let top = box_top_edge(&direct_topo, solid, dx, dz);
    let first = crate::blend_ops::fillet_cascade(&mut direct_topo, solid, &[top], big)
        .unwrap()
        .solid;
    let vertical = box_vertical_edge(&direct_topo, first, dx);
    let direct = crate::blend_ops::fillet_cascade(&mut direct_topo, first, &[vertical], small)
        .unwrap()
        .solid;

    let counts = remus_topology::explorer::solid_entity_counts(&topo, healed).unwrap();
    assert_eq!(
        counts,
        remus_topology::explorer::solid_entity_counts(&direct_topo, direct).unwrap(),
        "same topology as filleting the larger edge first"
    );
    let census = |topo: &Topology, solid: SolidId| {
        let mut tags: Vec<_> = remus_topology::explorer::solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .map(|f| topo.face(f).unwrap().surface().type_tag())
            .collect();
        tags.sort_unstable();
        tags
    };
    assert_eq!(census(&topo, healed), census(&direct_topo, direct));

    let (big_section, _) = quarter_fillet_section(big);
    let (section, inset) = quarter_fillet_section(small);
    let corner = section * std::f64::consts::FRAC_PI_2 * (big - inset);
    let expected = dx * dy * dz - big_section * dy - section * ((dz - big) + (dx - big)) - corner;
    let vol = crate::measure::solid_volume(&topo, healed, 0.01).unwrap();
    assert!(
        (vol - expected).abs() < 0.05,
        "healed volume {vol} should match closed form {expected}"
    );
    let direct_vol = crate::measure::solid_volume(&direct_topo, direct, 0.01).unwrap();
    assert!(
        (vol - direct_vol).abs() < 0.01,
        "healed volume {vol} should match the larger-first volume {direct_vol}"
    );
}

/// A shortened edge between two earlier, different, smaller fillets (top at
/// 1, bottom at 1.7) takes a 3 mm fillet: both bands are rebuilt after it,
/// largest first.
#[test]
fn fillet_between_two_smaller_bands_rebuilds_both() {
    let (dx, dy, dz) = (40.0, 20.0, 8.0);
    let near = |a: f64, b: f64| (a - b).abs() < 1e-9;
    let front_line = |topo: &Topology, solid: SolidId, z: f64| {
        solid_edge_ids(topo, solid)
            .into_iter()
            .find(|&e| {
                let (a, b) = edge_endpoints(topo, e);
                matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Line)
                    && near(a.y(), 0.0)
                    && near(b.y(), 0.0)
                    && near(a.z(), z)
                    && near(b.z(), z)
            })
            .expect("front box edge")
    };
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, dx, dy, dz).unwrap();
    let top = front_line(&topo, solid, dz);
    let solid = crate::blend_ops::fillet_cascade(&mut topo, solid, &[top], 1.0)
        .unwrap()
        .solid;
    let bottom = front_line(&topo, solid, 0.0);
    let input = crate::blend_ops::fillet_cascade(&mut topo, solid, &[bottom], 1.7)
        .unwrap()
        .solid;
    let vertical = box_vertical_edge(&topo, input, dx);
    let input_vol = crate::measure::solid_volume(&topo, input, 0.01).unwrap();

    let healed = crate::blend_ops::fillet_cascade(&mut topo, input, &[vertical], 3.0)
        .expect("a larger fillet between smaller bands must build")
        .solid;
    assert_fully_valid(&topo, healed, "fillet between two smaller bands");
    let vol = crate::measure::solid_volume(&topo, healed, 0.01).unwrap();
    assert!(vol < input_vol, "a convex fillet removes material");

    // Largest first reaches the same body.
    let mut direct_topo = Topology::new();
    let solid = crate::primitives::make_box(&mut direct_topo, dx, dy, dz).unwrap();
    let vertical = box_vertical_edge(&direct_topo, solid, dx);
    let mut solid = crate::blend_ops::fillet_cascade(&mut direct_topo, solid, &[vertical], 3.0)
        .unwrap()
        .solid;
    for (z, radius) in [(0.0, 1.7), (dz, 1.0)] {
        let edge = front_line(&direct_topo, solid, z);
        solid = crate::blend_ops::fillet_cascade(&mut direct_topo, solid, &[edge], radius)
            .unwrap()
            .solid;
    }
    assert_eq!(
        remus_topology::explorer::solid_entity_counts(&topo, healed).unwrap(),
        remus_topology::explorer::solid_entity_counts(&direct_topo, solid).unwrap()
    );
    let direct_vol = crate::measure::solid_volume(&direct_topo, solid, 0.01).unwrap();
    assert!(
        (vol - direct_vol).abs() < 0.01,
        "healed volume {vol} should match the larger-first volume {direct_vol}"
    );
}

/// The concave L-bracket (XY `[(0,0),(40,0),(40,8),(8,8),(8,50),(0,50)]`,
/// 20 mm along +Z) shared with `regress_notch_torus_whole_edge`.
fn l_bracket(topo: &mut Topology) -> SolidId {
    let profile = remus_topology::builder::make_polygon_wire(
        topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(40.0, 0.0, 0.0),
            Point3::new(40.0, 8.0, 0.0),
            Point3::new(8.0, 8.0, 0.0),
            Point3::new(8.0, 50.0, 0.0),
            Point3::new(0.0, 50.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let face = topo.add_face(remus_topology::face::Face::new(
        profile,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    crate::extrude::extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), 20.0).unwrap()
}

/// The straight edges of `solid` with both endpoints satisfying `at`.
fn lines_where(topo: &Topology, solid: SolidId, at: impl Fn(Point3) -> bool) -> Vec<EdgeId> {
    solid_edge_ids(topo, solid)
        .into_iter()
        .filter(|&e| {
            let (a, b) = edge_endpoints(topo, e);
            matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Line) && at(a) && at(b)
        })
        .collect()
}

/// The concave wall/base edge at the bracket's top notch, `(8, 8, z)`.
fn notch_concave_edge(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    let near = |a: f64, b: f64| (a - b).abs() < 1e-9;
    lines_where(topo, solid, |p| near(p.x(), 8.0) && near(p.y(), 8.0))
}

/// The two convex end-face edges meeting at the top notch: the base top
/// (`y = 8`) and the wall (`x = 8`), both on `z = 20`.
fn notch_end_edges(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    let near = |a: f64, b: f64| (a - b).abs() < 1e-9;
    let mut edges = lines_where(topo, solid, |p| {
        near(p.z(), 20.0) && near(p.y(), 8.0) && p.x() > 8.0 - 1e-9
    });
    edges.extend(lines_where(topo, solid, |p| {
        near(p.z(), 20.0) && near(p.x(), 8.0) && p.y() > 8.0 - 1e-9
    }));
    edges
}

/// Closed-form volume of the bracket with its top notch's concave edge at
/// `concave` and both end-face edges at `convex`, the corner an exact torus
/// about the concave band's axis (ring `concave + convex`, tube `convex`).
fn notch_volume(concave: f64, convex: f64) -> f64 {
    let (concave_section, _) = quarter_fillet_section(concave);
    let (section, inset) = quarter_fillet_section(convex);
    // The concave band adds material along the whole 20 mm edge; each end
    // stripe removes it from the band's end (`concave` from the notch) to the
    // far face; the torus sweeps the convex section a quarter turn at the
    // section centroid's distance from the axis (Pappus).
    let corner = section * std::f64::consts::FRAC_PI_2 * (concave + inset);
    13_120.0 + concave_section * 20.0 - section * ((32.0 - concave) + (42.0 - concave)) - corner
}

/// The faces of `solid` by surface type, sorted.
fn surface_census(topo: &Topology, solid: SolidId) -> Vec<&'static str> {
    let mut tags: Vec<_> = remus_topology::explorer::solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(|f| topo.face(f).unwrap().surface().type_tag())
        .collect();
    tags.sort_unstable();
    tags
}

/// A mixed-radius notch corner, concave edge first: the convex end-face
/// stripes run across the concave band's end arc as one exact torus about the
/// band's axis, for a convex radius below, at and above the concave one. The
/// rolling ball rides the band's outside there, so the band's radius does not
/// bound it.
#[test]
fn notch_concave_then_convex_rounds_corner_as_exact_torus() {
    for (concave, convex) in [(3.0, 1.0), (1.0, 1.0), (1.0, 3.0)] {
        let what = format!("notch concave {concave} then convex {convex}");
        let mut topo = Topology::new();
        let bracket = l_bracket(&mut topo);
        let edges = notch_concave_edge(&topo, bracket);
        let first = crate::blend_ops::fillet_cascade(&mut topo, bracket, &edges, concave)
            .unwrap()
            .solid;
        let edges = notch_end_edges(&topo, first);
        let second = crate::blend_ops::fillet_cascade(&mut topo, first, &edges, convex)
            .unwrap_or_else(|e| panic!("{what}: {e}"))
            .solid;
        assert_fully_valid(&topo, second, &what);
        assert_eq!(
            surface_census(&topo, second),
            [
                "cylinder", "cylinder", "cylinder", "plane", "plane", "plane", "plane", "plane",
                "plane", "plane", "plane", "torus"
            ],
            "{what}: every face exact"
        );
        // No chord anywhere: the torus's four seams are circles.
        let adjacency = topo.build_adjacency(second).unwrap();
        for edge in remus_topology::explorer::solid_edges(&topo, second).unwrap() {
            let on_torus = adjacency
                .faces_for_edge(edge)
                .iter()
                .any(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)));
            if on_torus {
                assert!(
                    matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Circle(_)),
                    "{what}: torus seam {edge:?} must be a circle"
                );
            }
        }
        let vol = crate::measure::solid_volume(&topo, second, 0.01).unwrap();
        let expected = notch_volume(concave, convex);
        assert!(
            (vol - expected).abs() < 0.01,
            "{what}: volume {vol} should match closed form {expected}"
        );
    }
}

/// The other order. The convex end-face edges alone mitre over the reflex
/// corner: each stripe runs past the vertex to the bisecting plane, where
/// the two cylinders meet in one exact ellipse. Filleting the concave edge
/// afterwards takes that mitred pair off, rounds the concave edge, and puts
/// the pair back: the same body the concave-first order gives.
#[test]
fn notch_convex_first_mitres_then_matches_concave_first() {
    for (concave, convex) in [(3.0, 1.0), (1.0, 1.0), (1.0, 3.0)] {
        let what = format!("notch convex {convex} then concave {concave}");
        let mut topo = Topology::new();
        let bracket = l_bracket(&mut topo);
        let edges = notch_end_edges(&topo, bracket);
        let mitred = crate::blend_ops::fillet_cascade(&mut topo, bracket, &edges, convex)
            .unwrap_or_else(|e| panic!("{what}, mitre: {e}"))
            .solid;
        assert_fully_valid(&topo, mitred, &format!("{what}, mitre"));
        assert_eq!(
            remus_topology::explorer::solid_entity_counts(&topo, mitred).unwrap(),
            (10, 23, 15)
        );
        let ellipses = remus_topology::explorer::solid_edges(&topo, mitred)
            .unwrap()
            .into_iter()
            .filter(|&e| matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Ellipse(_)))
            .count();
        assert_eq!(ellipses, 1, "{what}: the stripes meet in one ellipse");
        // Each stripe's removed section runs from the mitre plane `x = y` to
        // the far face, so its mean length is measured at the section centroid.
        let (section, inset) = quarter_fillet_section(convex);
        let expected = 13_120.0 - section * (74.0 + 2.0 * inset);
        let vol = crate::measure::solid_volume(&topo, mitred, 0.01).unwrap();
        assert!(
            (vol - expected).abs() < 0.01,
            "{what}: mitre volume {vol} should match closed form {expected}"
        );

        let edges = notch_concave_edge(&topo, mitred);
        let healed = crate::blend_ops::fillet_cascade(&mut topo, mitred, &edges, concave)
            .unwrap_or_else(|e| panic!("{what}: {e}"))
            .solid;
        assert_fully_valid(&topo, healed, &what);
        assert_eq!(
            remus_topology::explorer::solid_entity_counts(&topo, healed).unwrap(),
            (12, 28, 18)
        );
        assert!(surface_census(&topo, healed).contains(&"torus"));
        let vol = crate::measure::solid_volume(&topo, healed, 0.01).unwrap();
        let expected = notch_volume(concave, convex);
        assert!(
            (vol - expected).abs() < 0.01,
            "{what}: volume {vol} should match the concave-first closed form {expected}"
        );
    }
}

/// A 20 mm box with an enclosed cavity box at `min .. min + size`, or a plain
/// box when `cavity` is `None`.
fn box_with_cavity(topo: &mut Topology, cavity: Option<(Point3, Vec3)>) -> SolidId {
    let outer = crate::primitives::make_box(topo, 20.0, 20.0, 20.0).unwrap();
    let Some((min, size)) = cavity else {
        return outer;
    };
    let inner = crate::primitives::make_box(topo, size.x(), size.y(), size.z()).unwrap();
    crate::transform::transform_solid(
        topo,
        inner,
        &remus_math::mat::Mat4::translation(min.x(), min.y(), min.z()),
    )
    .unwrap();
    let hollow =
        crate::boolean::boolean(topo, crate::boolean::BooleanOp::Cut, outer, inner).unwrap();
    assert_eq!(topo.solid(hollow).unwrap().inner_shells().len(), 1);
    hollow
}

/// Fillet the box's vertical edge at `(20, 0)` at `first`, then its top edge
/// at `x = 20, z = 20` at `second`.
fn vertical_then_top(
    topo: &mut Topology,
    solid: SolidId,
    first: f64,
    second: f64,
) -> Result<SolidId, crate::OperationsError> {
    let vertical = box_vertical_edge(topo, solid, 20.0);
    let first = crate::blend_ops::fillet_cascade(topo, solid, &[vertical], first)?.solid;
    let top = box_top_edge(topo, first, 20.0, 20.0);
    Ok(crate::blend_ops::fillet_cascade(topo, first, &[top], second)?.solid)
}

/// A hollow body takes the same sequential fillets as a solid one: the
/// cascade fillets the outer shell on its own and carries the cavity shell
/// through untouched. The result is the solid body's, less the cavity.
#[test]
fn sequential_fillets_on_a_hollow_body_carry_the_cavity() {
    let cavity = (Point3::new(5.0, 5.0, 5.0), Vec3::new(10.0, 10.0, 10.0));
    for (first, second) in [(1.0, 1.0), (1.0, 0.5), (1.0, 2.0)] {
        let what = format!("hollow box, vertical {first} then top {second}");
        let mut solid_topo = Topology::new();
        let plain = box_with_cavity(&mut solid_topo, None);
        let plain = vertical_then_top(&mut solid_topo, plain, first, second).unwrap();

        let mut topo = Topology::new();
        let hollow = box_with_cavity(&mut topo, Some(cavity));
        let cavity_faces: Vec<FaceId> = topo
            .shell(topo.solid(hollow).unwrap().inner_shells()[0])
            .unwrap()
            .faces()
            .to_vec();
        let result = vertical_then_top(&mut topo, hollow, first, second)
            .unwrap_or_else(|e| panic!("{what}: {e}"));
        assert_fully_valid(&topo, result, &what);

        let inner = topo.solid(result).unwrap().inner_shells().to_vec();
        assert_eq!(inner.len(), 1, "{what}: one cavity shell");
        assert_eq!(
            topo.shell(inner[0]).unwrap().faces(),
            cavity_faces.as_slice(),
            "{what}: the cavity is carried through untouched"
        );
        let (f, e, v) = remus_topology::explorer::solid_entity_counts(&solid_topo, plain).unwrap();
        assert_eq!(
            remus_topology::explorer::solid_entity_counts(&topo, result).unwrap(),
            (f + 6, e + 12, v + 8),
            "{what}: the solid body's topology plus the cavity box"
        );
        let expected = crate::measure::solid_volume(&solid_topo, plain, 0.01).unwrap() - 1000.0;
        let vol = crate::measure::solid_volume(&topo, result, 0.01).unwrap();
        assert!(
            (vol - expected).abs() < 0.01,
            "{what}: volume {vol} should be the solid body's less the cavity, {expected}"
        );
    }
}

/// A fillet whose band would cut into the cavity is still refused, and the
/// input is left as it was: the cavity sits under the top edge, inside the
/// material a 2 mm fillet there removes.
#[test]
fn hollow_body_fillet_reaching_the_cavity_is_refused() {
    let mut topo = Topology::new();
    let hollow = box_with_cavity(
        &mut topo,
        Some((Point3::new(18.5, 5.0, 18.5), Vec3::new(1.0, 10.0, 1.0))),
    );
    let vertical = box_vertical_edge(&topo, hollow, 20.0);
    let first = crate::blend_ops::fillet_cascade(&mut topo, hollow, &[vertical], 1.0)
        .unwrap()
        .solid;
    let before = remus_topology::explorer::solid_entity_counts(&topo, first).unwrap();
    let top = box_top_edge(&topo, first, 20.0, 20.0);
    assert!(
        crate::blend_ops::fillet_cascade(&mut topo, first, &[top], 2.0).is_err(),
        "a band through the cavity must be refused"
    );
    assert_eq!(
        remus_topology::explorer::solid_entity_counts(&topo, first).unwrap(),
        before
    );
    assert_eq!(topo.solid(first).unwrap().inner_shells().len(), 1);
}

/// The same sequence with a smaller second radius: the corner is a torus
/// band around the earlier fillet's axis, whose material follows Pappus.
#[test]
fn sequential_fillet_smaller_radius_rounds_corner_as_torus() {
    let dims = (9.5, 12.5, 8.5);
    let (dx, dy, _) = dims;
    let mut topo = Topology::new();
    let (first, second) = sequential_top_fillet(&mut topo, dims, 1.0, 0.5);
    assert_fully_valid(&topo, second, "sequential smaller-radius fillet");

    let counts = remus_topology::explorer::solid_entity_counts(&topo, second).unwrap();
    assert_eq!(counts, (10, 22, 14), "faces, edges, vertices");

    // Closed form. The second fillet's cross-section removes
    // A = (1−π/4)·r² per unit length along its two straight runs, and along
    // the quarter-turn arc its centroid (0.2234·r inside the corner) travels
    // a quarter circle of radius R − 0.2234·r about the band's axis.
    let (big, r) = (1.0, 0.5);
    let section = (1.0 - std::f64::consts::FRAC_PI_4) * r * r;
    let centroid_inset = {
        let square = r * r * r / 2.0;
        let quarter_disc =
            std::f64::consts::FRAC_PI_4 * r * r * (r - 4.0 * r / (3.0 * std::f64::consts::PI));
        (square - quarter_disc) / section
    };
    let corner = section * std::f64::consts::FRAC_PI_2 * (big - centroid_inset);
    let vol_first = crate::measure::solid_volume(&topo, first, 0.01).unwrap();
    let expected = vol_first - section * ((dy - big) + (dx - big)) - corner;
    let vol = crate::measure::solid_volume(&topo, second, 0.01).unwrap();
    assert!(
        (vol - expected).abs() < 0.05,
        "sequential fillet volume {vol} should match closed form {expected}"
    );
}

/// A seam between two faces of one plane is a split, not a transition: the
/// L-blank a fuse leaves behind has its wall corner edge collinear with the
/// base-plate corner edge, on coplanar side faces. Selecting the base
/// segment must fillet the base segment only, as it always has.
#[test]
fn g1_chain_does_not_cross_a_coplanar_split() {
    use remus_math::mat::Mat4;

    let mut topo = Topology::new();
    let base = crate::primitives::make_box(&mut topo, 80.0, 40.0, 8.0).unwrap();
    let wall = crate::primitives::make_box(&mut topo, 80.0, 8.0, 32.0).unwrap();
    crate::transform::transform_solid(&mut topo, wall, &Mat4::translation(0.0, 32.0, 7.5)).unwrap();
    let blank =
        crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Fuse, base, wall).unwrap();

    let near = |a: f64, b: f64| (a - b).abs() < 1e-6;
    let base_corner = solid_edge_ids(&topo, blank)
        .into_iter()
        .find(|&e| {
            let (a, b) = edge_endpoints(&topo, e);
            near(a.x(), 0.0)
                && near(b.x(), 0.0)
                && near(a.y(), 40.0)
                && near(b.y(), 40.0)
                && a.z().max(b.z()) < 8.5
                && (a.z() - b.z()).abs() > 4.0
        })
        .expect("base-plate corner edge at x=0, y=40");
    let chain = remus_blend::g1_chain::expand_g1_chain(
        &topo,
        blank,
        &[base_corner],
        remus_math::tolerance::Tolerance::new(),
    )
    .unwrap();
    assert_eq!(
        chain,
        vec![base_corner],
        "the collinear wall edge must not join"
    );
}

/// A full-turn rim of the blend's own radius is a dome, not a fillet: the
/// plane it borders would shrink to a point. The curvature guard keeps
/// refusing it while the partial arc above is allowed.
#[test]
fn equal_radius_full_rim_is_still_refused() {
    let mut topo = Topology::new();
    let solid = crate::primitives::make_cylinder(&mut topo, 1.0, 4.0).unwrap();
    let rim = solid_edge_ids(&topo, solid)
        .into_iter()
        .find(|&e| topo.edge(e).unwrap().is_closed())
        .expect("a closed rim edge");
    let err = fillet_rolling_ball(&mut topo, solid, &[rim], 1.0).unwrap_err();
    assert!(format!("{err}").contains("curvature"), "{err}");
}

fn positional_cube_specs(topo: &Topology, solid: SolidId) -> Vec<FaceSpec> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(|id| {
            let face = topo.face(id).unwrap();
            let FaceSurface::Plane { normal, d } = face.surface() else {
                panic!("cube must be planar");
            };
            FaceSpec::Planar {
                vertices: crate::boolean::face_polygon(topo, id).unwrap(),
                normal: *normal,
                d: *d,
                inner_wires: vec![],
            }
        })
        .collect()
}

#[test]
fn explicit_stripe_boundaries_preserve_native_trim_direction_on_both_faces() {
    use crate::boolean::{BoundaryCurveOverride, assemble_solid_mixed_with_history_and_curves};
    use remus_math::nurbs::curve::NurbsCurve;

    for reversed in [false, true] {
        let mut topo = Topology::new();
        let input = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let specs = positional_cube_specs(&topo, input);
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(1.0, 0.0, 0.0);
        let curve =
            NurbsCurve::new(1, vec![2.0, 2.0, 5.0, 5.0], vec![a, b], vec![1.0, 2.0]).unwrap();
        let override_ = BoundaryCurveOverride {
            start: if reversed { b } else { a },
            end: if reversed { a } else { b },
            curve: EdgeCurve::NurbsCurve(curve),
            trim: if reversed { (5.0, 2.0) } else { (2.0, 5.0) },
        };
        let expected_trim = override_.trim;
        let result = assemble_solid_mixed_with_history_and_curves(
            &mut topo,
            &specs,
            &[override_],
            Tolerance::new(),
        )
        .unwrap()
        .solid;
        let edge = solid_edges(&topo, result)
            .unwrap()
            .into_iter()
            .find(|&edge| matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::NurbsCurve(_)))
            .unwrap();
        let data = topo.edge(edge).unwrap();
        assert_eq!(data.strict_domain().unwrap(), expected_trim);
        assert_eq!(
            topo.vertex(data.start()).unwrap().point(),
            if reversed { b } else { a }
        );
        let faces = topo
            .build_adjacency(result)
            .unwrap()
            .faces_for_edge(edge)
            .to_vec();
        assert_eq!(faces.len(), 2);
        let directions: Vec<_> = faces
            .iter()
            .map(|&id| {
                let face = topo.face(id).unwrap();
                topo.wire(face.outer_wire())
                    .unwrap()
                    .edges()
                    .iter()
                    .find(|use_| use_.edge() == edge)
                    .unwrap()
                    .is_forward()
                    ^ face.is_reversed()
            })
            .collect();
        assert_ne!(directions[0], directions[1]);
        assert!(
            crate::validate::validate_solid(&topo, result)
                .unwrap()
                .is_valid()
        );
        assert!((crate::measure::solid_volume(&topo, result, 0.001).unwrap() - 1.0).abs() < 1e-9);
    }
}

#[test]
fn malformed_stripe_boundary_override_refuses_and_rolls_back() {
    use crate::boolean::{BoundaryCurveOverride, assemble_solid_mixed_with_history_and_curves};
    use remus_math::nurbs::curve::NurbsCurve;

    let a = Point3::new(0.0, 0.0, 0.0);
    let b = Point3::new(1.0, 0.0, 0.0);
    for (start, end, trim) in [
        (a, b, (f64::NAN, 1.0)),
        (a, b, (0.0, 0.0)),
        (a, b, (0.0, 0.5)),
        (a, b, (-1.0, 2.0)),
        (Point3::new(f64::NAN, 0.0, 0.0), b, (0.0, 1.0)),
        (a, Point3::new(f64::INFINITY, 0.0, 0.0), (0.0, 1.0)),
    ] {
        let mut topo = Topology::new();
        let input = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let specs = positional_cube_specs(&topo, input);
        let before = remus_io::arena_io::serialize_solids(&topo, &[input]).unwrap();
        let live_before = (topo.num_vertices(), topo.num_edges(), topo.num_faces());
        let curve =
            NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![a, b], vec![1.0, 1.0]).unwrap();
        let result = remus_topology::transaction::run_transacted(&mut topo, |topo| {
            assemble_solid_mixed_with_history_and_curves(
                topo,
                &specs,
                &[BoundaryCurveOverride {
                    start,
                    end,
                    curve: EdgeCurve::NurbsCurve(curve),
                    trim,
                }],
                Tolerance::new(),
            )
        });
        assert!(matches!(
            result,
            Err(crate::OperationsError::Unsupported { .. })
        ));
        assert_eq!(
            remus_io::arena_io::serialize_solids(&topo, &[input]).unwrap(),
            before
        );
        assert_eq!(
            (topo.num_vertices(), topo.num_edges(), topo.num_faces()),
            live_before
        );
    }
}

#[test]
fn positional_assembly_without_stripe_overrides_keeps_lines_and_cube_quantities() {
    let mut topo = Topology::new();
    let input = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let specs = positional_cube_specs(&topo, input);
    let result = crate::boolean::assemble_solid_mixed(&mut topo, &specs, Tolerance::new()).unwrap();
    for edge in solid_edges(&topo, result).unwrap() {
        assert!(matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Line));
    }
    assert_eq!(solid_entity_counts(&topo, result).unwrap(), (6, 12, 8));
    assert!((crate::measure::solid_volume(&topo, result, 0.001).unwrap() - 1.0).abs() < 1e-9);
}

#[test]
fn altered_native_boundary_pcurves_refuse_on_both_faces_and_roll_back() {
    use remus_math::nurbs::curve::NurbsCurve;
    let mut topo = Topology::new();
    let input = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let source_edge = solid_edges(&topo, input)
        .unwrap()
        .into_iter()
        .find(|&id| {
            let e = topo.edge(id).unwrap();
            let (a, b) = (
                topo.vertex(e.start()).unwrap().point(),
                topo.vertex(e.end()).unwrap().point(),
            );
            a.x() == 0.0 && a.y() == 0.0 && b.x() == 0.0 && b.y() == 0.0
        })
        .unwrap();
    let result = fillet_variable(
        &mut topo,
        input,
        &[(source_edge, FilletRadiusLaw::Constant(9.0))],
    )
    .unwrap();
    let boundary = solid_edges(&topo,result).unwrap().into_iter().find(|&id| {
        matches!(topo.edge(id).unwrap().curve(),EdgeCurve::NurbsCurve(c) if c.degree()==2)
    }).unwrap();
    let incident = topo
        .build_adjacency(result)
        .unwrap()
        .faces_for_edge(boundary)
        .to_vec();
    assert_eq!(incident.len(), 2);
    let before = remus_io::arena_io::serialize_solids(&topo, &[input, result]).unwrap();
    let error: Result<(), crate::OperationsError> =
        remus_topology::transaction::run_transacted(&mut topo, |topo| {
            let edge = topo.edge(boundary).unwrap();
            let trim = edge.strict_domain().unwrap();
            let EdgeCurve::NurbsCurve(c) = edge.curve() else {
                panic!("native boundary");
            };
            let mut controls = c.control_points().to_vec();
            controls[1] = controls[1] + Vec3::new(0.0, 0.0, 0.1);
            let bowed = NurbsCurve::new(
                c.degree(),
                c.knots().to_vec(),
                controls,
                c.weights().to_vec(),
            )
            .unwrap();
            let edge = topo.edge_mut(boundary).unwrap();
            edge.set_curve(EdgeCurve::NurbsCurve(bowed));
            edge.set_trim(Some(trim));
            assert!(
                !crate::validate::validate_solid(topo, result)
                    .unwrap()
                    .is_valid(),
                "the ordinary validator must detect the stored pcurve mismatch"
            );
            for &face in &incident {
                let wire = topo.face(face).unwrap().outer_wire();
                let use_ = topo
                    .wire(wire)
                    .unwrap()
                    .edges()
                    .iter()
                    .find(|u| u.edge() == boundary)
                    .unwrap();
                assert!(
                    remus_topology::validation::validate_same_parameter_strict(
                        topo,
                        boundary,
                        face,
                        use_.is_forward(),
                        1e-7,
                        0
                    )
                    .is_err(),
                    "both incident faces must reject a changed interior"
                );
            }
            variable_pcurves::register(topo, result, Tolerance::new())
        });
    assert!(matches!(
        error,
        Err(crate::OperationsError::Unsupported { .. })
    ));
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[input, result]).unwrap(),
        before
    );
    remus_topology::validation::validate_solid_pcurve_contracts(&topo, result, 1e-7, 0).unwrap();
}

#[test]
fn ordinary_validator_keeps_valid_unproved_native_pcurve_families() {
    use crate::boolean::{BoundaryCurveOverride, assemble_solid_mixed_with_history_and_curves};
    use remus_math::curves2d::{Curve2D, Line2D};
    use remus_math::nurbs::curve::NurbsCurve;
    use remus_math::vec::{Point2, Vec2};
    use remus_topology::pcurve::PCurve;
    let mut topo = Topology::new();
    let input = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let specs = positional_cube_specs(&topo, input);
    let (a, b) = (Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
    let curve = NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![a, b], vec![1.0, 1.0]).unwrap();
    let solid = assemble_solid_mixed_with_history_and_curves(
        &mut topo,
        &specs,
        &[BoundaryCurveOverride {
            start: a,
            end: b,
            curve: EdgeCurve::NurbsCurve(curve),
            trim: (0.0, 1.0),
        }],
        Tolerance::new(),
    )
    .unwrap()
    .solid;
    let edge = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&id| matches!(topo.edge(id).unwrap().curve(), EdgeCurve::NurbsCurve(_)))
        .unwrap();
    let faces = topo
        .build_adjacency(solid)
        .unwrap()
        .faces_for_edge(edge)
        .to_vec();
    for face in faces {
        let wire = topo.face(face).unwrap().outer_wire();
        let forward = topo
            .wire(wire)
            .unwrap()
            .edges()
            .iter()
            .find(|u| u.edge() == edge)
            .unwrap()
            .is_forward();
        // Both incident planes use canonical chart v=-x. This exact linear
        // use is geometrically valid, but Plane/Line-pcurve/NURBS-edge has
        // no whole-interval proof family yet and must not be promoted.
        let pc = Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(0.0, -1.0)).unwrap());
        let (p0, p1) = if forward { (0.0, 1.0) } else { (1.0, 0.0) };
        topo.set_pcurve_oriented(edge, face, forward, PCurve::new(pc, p0, p1))
            .unwrap();
        assert!(matches!(remus_topology::validation::check_same_parameter_strict(&topo,edge,face,forward,0),Err(remus_topology::validation::CurveUseValidationError::SameParameterProofUnavailable {..})));
    }
    assert!(
        crate::validate::validate_solid(&topo, solid)
            .unwrap()
            .is_valid()
    );
    assert!((crate::measure::solid_volume(&topo, solid, 0.001).unwrap() - 1.0).abs() < 1e-9);
}

#[test]
fn ordinary_validator_resolves_native_pcurve_entity_tolerance_boundary() {
    use crate::boolean::{BoundaryCurveOverride, assemble_solid_mixed_with_history_and_curves};
    use remus_math::curves2d::{Curve2D, NurbsCurve2D};
    use remus_math::nurbs::curve::NurbsCurve;
    use remus_math::vec::Point2;
    use remus_topology::pcurve::PCurve;
    for tolerance in [1e-7, 1e-4] {
        for (factor, valid) in [(1.5, true), (3.0, false)] {
            let mut topo = Topology::new();
            let input = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
            let specs = positional_cube_specs(&topo, input);
            let (a, b) = (Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
            let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
            let curve = NurbsCurve::new(
                2,
                knots.clone(),
                vec![a, Point3::new(0.5, factor * tolerance, 0.0), b],
                vec![1.0; 3],
            )
            .unwrap();
            let solid = assemble_solid_mixed_with_history_and_curves(
                &mut topo,
                &specs,
                &[BoundaryCurveOverride {
                    start: a,
                    end: b,
                    curve: EdgeCurve::NurbsCurve(curve),
                    trim: (0.0, 1.0),
                }],
                Tolerance::new(),
            )
            .unwrap()
            .solid;
            let edge = solid_edges(&topo, solid)
                .unwrap()
                .into_iter()
                .find(|&id| matches!(topo.edge(id).unwrap().curve(), EdgeCurve::NurbsCurve(_)))
                .unwrap();
            topo.edge_mut(edge)
                .unwrap()
                .set_tolerance(Some(tolerance))
                .unwrap();
            let face=topo.build_adjacency(solid).unwrap().faces_for_edge(edge).iter().copied().find(|&id|matches!(topo.face(id).unwrap().surface(),FaceSurface::Plane {normal,..} if normal.z().abs()>0.9)).unwrap();
            let wire = topo.face(face).unwrap().outer_wire();
            let forward = topo
                .wire(wire)
                .unwrap()
                .edges()
                .iter()
                .find(|u| u.edge() == edge)
                .unwrap()
                .is_forward();
            let pc = NurbsCurve2D::new(
                2,
                knots,
                vec![
                    Point2::new(0.0, 0.0),
                    Point2::new(0.0, -0.5),
                    Point2::new(0.0, -1.0),
                ],
                vec![1.0; 3],
            )
            .unwrap();
            let (p0, p1) = if forward { (0.0, 1.0) } else { (1.0, 0.0) };
            topo.set_pcurve_oriented(edge, face, forward, PCurve::new(Curve2D::Nurbs(pc), p0, p1))
                .unwrap();
            assert_eq!(
                remus_topology::validation::validate_same_parameter_strict(
                    &topo, edge, face, forward, 1e-7, 0
                )
                .is_ok(),
                valid
            );
            assert_eq!(
                crate::validate::validate_solid(&topo, solid)
                    .unwrap()
                    .is_valid(),
                valid,
                "true quadratic maximum is half its control-point amplitude"
            );
        }
    }
}

#[test]
fn variable_boundary_outside_precision_proof_refuses_typed_and_rolls_back() {
    let mut topo = Topology::new();
    // The geometry fits, but its arithmetic bound exceeds the unchanged
    // absolute tolerance. The emitter must refuse an unproved boundary.
    let input = crate::primitives::make_box(&mut topo, 1e7, 1e7, 1e7).unwrap();
    let edge = solid_edges(&topo, input)
        .unwrap()
        .into_iter()
        .find(|&id| {
            let edge = topo.edge(id).unwrap();
            [edge.start(), edge.end()].into_iter().all(|vertex| {
                let point = topo.vertex(vertex).unwrap().point();
                point.x() == 0.0 && point.y() == 0.0
            })
        })
        .unwrap();
    let before = remus_io::arena_io::serialize_solids(&topo, &[input]).unwrap();
    let counts = |topo: &Topology| {
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
        )
    };
    let live_before = counts(&topo);
    let error =
        fillet_variable(&mut topo, input, &[(edge, FilletRadiusLaw::Constant(9e6))]).unwrap_err();
    assert!(matches!(
        &error,
        crate::OperationsError::Unsupported {
            operation: "fillet_variable",
            ..
        }
    ));
    assert_eq!(
        crate::blend_ops::blend_failure_code(&error),
        "unsupported-variable-boundary"
    );
    assert_eq!(counts(&topo), live_before);
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[input]).unwrap(),
        before
    );
}
