#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stderr,
    deprecated
)]

use std::collections::{HashMap, HashSet};

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::test_utils::make_unit_cube_manifold;
use remus_topology::validation::validate_shell_manifold;

use crate::test_helpers::assert_euler_genus0;

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
/// A single-edge rolling-ball fillet yields a watertight solid with a blend
/// face. Filleting a (non-tangent) edge bordering that blend face must itself
/// produce a valid, watertight manifold — the blend's accessible
/// non-degenerate edges are concave end-caps, so the fillet fills the seam.
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

    // Second fillet on that blend-adjacent edge.
    //
    // Fail-closed contract: the historical "success" here passed the manifold
    // and closed-shell checks below while carrying 6 orientation-inconsistent
    // shared edges — a non-orientable patch that only the full validation
    // baseline catches. The engine now refuses it with a typed error. If the
    // engine is ever repaired, every oracle in the success branch must hold.
    let result = fillet_rolling_ball(&mut topo, first, &[target], 0.5);
    match result {
        Ok(result) => {
            let report = remus_check::validate::validate_solid(
                &topo,
                result,
                &remus_check::validate::ValidateOptions::default(),
            )
            .unwrap();
            assert!(
                report.is_valid(),
                "an accepted blend-adjacent fillet must be fully valid: {:#?}",
                report.issues
            );
            let sh = topo
                .shell(topo.solid(result).unwrap().outer_shell())
                .unwrap();
            validate_shell_manifold(sh, &topo).expect("second fillet must be manifold");
            validate_shell_closed(sh, &topo)
                .expect("second fillet on a blend-adjacent edge must be watertight");

            let vol2 = crate::measure::solid_volume(&topo, result, 0.05).unwrap();
            // Concave end-cap edge → the fillet fills; volume stays sane
            // (between the first fillet and the original box).
            assert!(
                vol2 > vol1 - 1e-6 && vol2 <= 1000.0 + 1e-6,
                "filled fillet volume out of range: first={vol1}, second={vol2}"
            );
        }
        Err(error) => {
            assert!(
                !crate::blend_ops::blend_failure_code(&error).is_empty(),
                "the refusal must carry a machine-readable code, got {error}"
            );
            // The refused second fillet must leave the once-filleted box
            // exactly as it was: still closed, still the same volume.
            let sh = topo
                .shell(topo.solid(first).unwrap().outer_shell())
                .unwrap();
            validate_shell_closed(sh, &topo)
                .expect("input must still be watertight after a refused second fillet");
            let vol_after = crate::measure::solid_volume(&topo, first, 0.05).unwrap();
            assert!(
                (vol_after - vol1).abs() < 1e-9,
                "input volume changed across a refused fillet: {vol1} -> {vol_after}"
            );
        }
    }
}
