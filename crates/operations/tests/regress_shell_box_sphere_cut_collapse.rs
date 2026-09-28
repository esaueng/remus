//! Shell collapse refusal for a box cut by a sphere (mixed planar/quadric lump).
//!
//! Fuzz Smoke run 36309258004 (`modifier_ops`, crash-c74759d0, 2026-09-27):
//! a 1×1.5×1.5 box cut by an r=1.5 sphere (13 segments) placed at
//! (1.5, 2.5, −0.5) after a 45° rotation about Y, then hollowed at thickness
//! 0.6 with no open faces, tessellated open (11 boundary edges at the harness
//! deflection) while passing coarsely (0 at 0.1) and both volume routes.
//!
//! Root cause (shell topology, not tessellation): the thin planar walls cross
//! (1.0 − 2·0.6 < 0, inner x at 0.6 vs 0.4) exactly like the standalone box,
//! but the all-planar fold gate let the mixed plane-plus-sphere lump through
//! and the inverted cavity shipped through the ordinary gate. Per-face
//! attribution (grouped tessellation) puts all 11 directed boundary edges on
//! the inner cavity (planes 26/27/29/30 and spheres 28/31); the outer skin is
//! clean. Inner sphere vertices sit ~0.096 off their offset carrier (linear
//! miter vs radial offset) at this thickness, but the watertightness failure
//! is the planar inversion: the same miter at 0.4 (0.048 off) still meshes
//! closed.
//!
//! Fix (owning layer `shell_op.rs` only): hole-free planar faces are collected
//! from every lump for closed hollows, so the mixed lump's planar subset
//! reaches the fold remover and fails closed as unqualified instead of
//! shipping. Open shells keep the established all-planar gate (the rim joins
//! outer and inner, so the remover must not see open cups). No weld-tolerance
//! or coincident-triangle change; B71's wrong-side-blend refusal is untouched.
//!
//! Bounded refusal (not capability closure): thickness past half the thin
//! planar separation (here 0.5) on this mixed lump refuses like the plain box
//! does. Thin walls (≤0.4) still hollow exactly; the linear-miter sphere
//! deviation at those walls (0.0009–0.048) and the open-thick (0.6, one face
//! opened) indexed leak hidden by the 1µm weld remain documented residuals.
//!
//! Fails before the fix (shell Ok with 11 boundary edges at the harness
//! deflection) and passes after (typed `Unsupported` with rollback).
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stderr
)]

use std::f64::consts::FRAC_PI_4;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{mass_properties, solid_bounding_box, solid_volume};
use remus_operations::primitives::{make_box, make_sphere};
use remus_operations::shell_op::shell;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid, welded_mesh_quality,
};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid_relaxed;
use remus_topology::Topology;
use remus_topology::explorer;
use remus_topology::solid::SolidId;

fn base_body(topo: &mut Topology) -> SolidId {
    let stock = make_box(topo, 1.0, 1.5, 1.5).unwrap();
    let tool = make_sphere(topo, 1.5, 13).unwrap();
    let place = Mat4::translation(1.5, 2.5, -0.5) * Mat4::rotation_y(FRAC_PI_4);
    transform_solid(topo, tool, &place).unwrap();
    boolean(topo, BooleanOp::Cut, stock, tool).unwrap()
}

fn harness_deflection(topo: &Topology, solid: SolidId) -> f64 {
    let aabb = solid_bounding_box(topo, solid).unwrap();
    ((aabb.max - aabb.min).length() * 4e-5).max(1e-7) * 4.0
}

fn same_sense_pairs(topo: &Topology, solid: SolidId) -> usize {
    use std::collections::HashMap;
    let faces = explorer::solid_faces(topo, solid).unwrap();
    let mut uses: HashMap<remus_topology::edge::EdgeId, Vec<bool>> = HashMap::new();
    for &fid in &faces {
        let face = topo.face(fid).unwrap();
        let rev = face.is_reversed();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                uses.entry(oe.edge())
                    .or_default()
                    .push(oe.is_forward() != rev);
            }
        }
    }
    uses.values()
        .filter(|u| u.len() == 2 && u[0] == u[1])
        .count()
}

/// The exact fuzz input body, before any shell: valid, watertight, and
/// measured consistently by both volume routes. Recorded separately from the
/// shell result per the assignment.
#[test]
fn base_body_is_valid_watertight_and_measured() {
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let faces = explorer::solid_faces(&topo, body).unwrap();
    assert_eq!(faces.len(), 8, "6 planes + 2 sphere patches");
    let mut planes = 0;
    let mut spheres = 0;
    for fid in &faces {
        match topo.face(*fid).unwrap().surface().type_tag() {
            "plane" => planes += 1,
            "sphere" => spheres += 1,
            other => panic!("unexpected carrier {other}"),
        }
    }
    assert_eq!((planes, spheres), (6, 2));
    let map = explorer::edge_to_face_map(&topo, body).unwrap();
    assert_eq!(map.values().filter(|v| v.len() == 1).count(), 0);
    assert_eq!(map.values().filter(|v| v.len() > 2).count(), 0);
    let report = validate_solid_relaxed(&topo, body).unwrap();
    assert!(report.is_valid(), "base must be relaxed-valid");
    let aabb = solid_bounding_box(&topo, body).unwrap();
    let diag = (aabb.max - aabb.min).length();
    let defl = (diag * 4e-5).max(1e-7) * 4.0;
    let mesh = tessellate_solid(&topo, body, defl).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0);
    assert_eq!(non_manifold_edge_count(&mesh), 0);
    let v = solid_volume(&topo, body, (diag * 4e-5).max(1e-7)).unwrap();
    let m = mass_properties(&topo, body).unwrap().mass;
    eprintln!("base solid_volume={v:.9} mass_properties={m:.9}");
    assert!((v - 2.230_27).abs() / 2.230_27 < 1e-3);
    assert!((v - m).abs() / v.max(m) < 1e-3);
}

/// The minimized crash: closed shell at 0.6 must refuse (fail-closed) with
/// rollback, never return the 11-boundary-edge solid.
#[test]
fn closed_shell_at_0_6_refuses_with_rollback() {
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
    let err = shell(&mut topo, body, 0.6, &[]).expect_err("collapsed thin wall must refuse");
    assert!(
        matches!(err, remus_operations::OperationsError::Unsupported { .. }),
        "bounded typed refusal expected, got {err}"
    );
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
        before,
        "failed shell must roll back"
    );
}

/// The degenerate half-thickness (1.0 − 2·0.5 == 0) also refuses.
#[test]
fn closed_shell_at_0_5_refuses_with_rollback() {
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
    let err = shell(&mut topo, body, 0.5, &[]).expect_err("degenerate wall must refuse");
    assert!(
        matches!(err, remus_operations::OperationsError::Unsupported { .. }),
        "typed refusal expected, got {err}"
    );
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
        before
    );
}

fn assert_thin_wall_ok(thickness: f64) {
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let v_before = solid_volume(&topo, body, 1e-4)
        .unwrap_or_else(|_| mass_properties(&topo, body).unwrap().mass);
    let hollow = shell(&mut topo, body, thickness, &[]).unwrap_or_else(|e| {
        panic!("thin wall {thickness} must hollow, got refusal {e}");
    });
    // B-Rep: relaxed-valid (hollow is two components in one shell), no free
    // or non-manifold edge uses, no same-sense pairs.
    let report = validate_solid_relaxed(&topo, hollow).unwrap();
    assert!(report.is_valid(), "thin {thickness} relaxed-valid");
    let map = explorer::edge_to_face_map(&topo, hollow).unwrap();
    assert_eq!(map.values().filter(|v| v.len() == 1).count(), 0);
    assert_eq!(map.values().filter(|v| v.len() > 2).count(), 0);
    assert_eq!(same_sense_pairs(&topo, hollow), 0);
    // Mesh: indexed and 1µm-welded closure at three physical deflections.
    let harness = harness_deflection(&topo, hollow);
    for d in [0.1, 0.01, harness] {
        let mesh = tessellate_solid(&topo, hollow, d).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "thin {thickness} d={d}");
        assert_eq!(non_manifold_edge_count(&mesh), 0);
        let wq = welded_mesh_quality(&mesh);
        assert_eq!(wq.boundary_edges, 0);
        assert_eq!(wq.non_manifold_edges, 0);
        assert!(wq.triangle_count > 0);
    }
    // Volume: hollowing shrinks, both routes agree (independent oracles share
    // the face integrator, so this is a co-signature paired with the mesh and
    // classification checks, not a standalone proof).
    let aabb = solid_bounding_box(&topo, hollow).unwrap();
    let diag = (aabb.max - aabb.min).length();
    let v = solid_volume(&topo, hollow, (diag * 4e-5).max(1e-7)).unwrap();
    let m = mass_properties(&topo, hollow).unwrap().mass;
    eprintln!("thin {thickness}: solid_volume={v:.9} mass={m:.9} before={v_before:.9}");
    assert!(v < v_before, "hollowing must not invent material");
    assert!((v - m).abs() / v.max(m) < 1e-2);
    // Classification spot checks: cavity Outside, wall Inside. The wall probe
    // sits 0.02 from the outer corner so it stays in material for every thin
    // wall in this family (0.05–0.4); 0.05 itself lies on the 0.05 cavity wall.
    let outside = remus_math::vec::Point3::new(0.5, 0.75, 0.75);
    let wall = remus_math::vec::Point3::new(0.02, 0.02, 0.02);
    assert_eq!(
        remus_check::classify::classify_point(
            &topo,
            hollow,
            outside,
            &remus_check::classify::ClassifyOptions::default()
        )
        .unwrap(),
        remus_check::classify::PointClassification::Outside
    );
    assert_eq!(
        remus_check::classify::classify_point(
            &topo,
            hollow,
            wall,
            &remus_check::classify::ClassifyOptions::default()
        )
        .unwrap(),
        remus_check::classify::PointClassification::Inside
    );
}

#[test]
fn thin_walls_hollow_watertight() {
    for t in [0.05, 0.1, 0.2, 0.3, 0.4] {
        assert_thin_wall_ok(t);
    }
}

/// One relevant opening (top cap removed) at a thin wall stays exact and
/// closed; the rim is a single annulus.
#[test]
fn open_top_at_thin_wall_hollows_watertight() {
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let faces = explorer::solid_faces(&topo, body).unwrap();
    let open: Vec<_> = faces
        .iter()
        .filter(|fid| {
            let f = topo.face(**fid).unwrap();
            matches!(f.surface(), remus_topology::face::FaceSurface::Plane { normal, d } if (*normal - remus_math::vec::Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9 && (*d - 1.5).abs() < 1e-9)
        })
        .copied()
        .collect();
    assert_eq!(open.len(), 1);
    let hollow = shell(&mut topo, body, 0.1, &open).unwrap();
    let harness = harness_deflection(&topo, hollow);
    for d in [0.1, 0.01, harness] {
        let mesh = tessellate_solid(&topo, hollow, d).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "open thin d={d}");
        assert_eq!(non_manifold_edge_count(&mesh), 0);
        assert_eq!(welded_mesh_quality(&mesh).boundary_edges, 0);
    }
    assert!(validate_solid_relaxed(&topo, hollow).unwrap().is_valid());
}

/// Scale: the same thin wall at 1000× carries census, s³ volume, and
/// watertightness.
#[test]
fn scaled_thin_wall_matches() {
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    transform_solid(&mut topo, body, &Mat4::scale(1000.0, 1000.0, 1000.0)).unwrap();
    let hollow = shell(&mut topo, body, 0.1 * 1000.0, &[]).unwrap();
    let harness = harness_deflection(&topo, hollow);
    let mesh = tessellate_solid(&topo, hollow, harness).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0);
    assert_eq!(non_manifold_edge_count(&mesh), 0);
    let aabb = solid_bounding_box(&topo, hollow).unwrap();
    let diag = (aabb.max - aabb.min).length();
    let v = solid_volume(&topo, hollow, (diag * 4e-5).max(1e-7)).unwrap();
    let m = mass_properties(&topo, hollow).unwrap().mass;
    assert!((v - m).abs() / v.max(m) < 1e-2);
    // s³ of the unit thin-wall volume (0.887_121 at 0.1, measured above).
    assert!((v / 1e9 - 0.887_121).abs() / 0.887_121 < 2e-2);
}

/// Rigid placement: translating the base body preserves the thin-wall
/// success and the thick-wall refusal with translation-invariant volume.
#[test]
fn rigid_placement_preserves_outcome() {
    let place = Mat4::translation(10.0, -20.0, 30.0);
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    transform_solid(&mut topo, body, &place).unwrap();
    let hollow = shell(&mut topo, body, 0.1, &[]).unwrap();
    let harness = harness_deflection(&topo, hollow);
    let mesh = tessellate_solid(&topo, hollow, harness).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0);
    let aabb = solid_bounding_box(&topo, hollow).unwrap();
    let diag = (aabb.max - aabb.min).length();
    let v = solid_volume(&topo, hollow, (diag * 4e-5).max(1e-7)).unwrap();
    assert!((v - 0.887_121).abs() / 0.887_121 < 2e-2);

    let mut topo2 = Topology::new();
    let body2 = base_body(&mut topo2);
    transform_solid(&mut topo2, body2, &place).unwrap();
    let before = remus_io::arena_io::serialize_solid(&topo2, body2).unwrap();
    let err = shell(&mut topo2, body2, 0.6, &[]).expect_err("placed thick must refuse");
    assert!(matches!(
        err,
        remus_operations::OperationsError::Unsupported { .. }
    ));
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo2, body2).unwrap(),
        before
    );
}
