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
//! Bounded exact domain, revised by the shell-correctness campaign (tolerance
//! contract, not wall fraction): the oblique plane–sphere cut carries an
//! off-carrier inner sphere at EVERY thickness (0.0009 at 0.05 through 0.096
//! at 0.6, all orders of magnitude beyond `Tolerance::linear` 1e-7), so no
//! wall hollows exactly — thin through thick refuse typed `Unsupported` with
//! rollback, naming the face, carrier, miss, and the missing exact
//! offset-surface intersection primitive (`intersect_plane_sphere` with shared
//! edge topology). PROMINENT NARROWING: thin walls (0.05–0.4) previously
//! hollowed `Exact` with indexed+welded closure despite 1.8–11.9% carrier miss;
//! they were mislabelled and now refuse like the thick collapse. The base body
//! stays valid/watertight/measured (pinned separately); the defect is in shell
//! construction, not the boolean. Open-thick (0.6, one face opened) previously
//! leaked 11 indexed edges hidden by the weld (0 welded); it now refuses via
//! the same carrier gate without breaking valid planar cups.
//!
//! Fails before the B76 fix (shell Ok with 11 boundary edges at the harness
//! deflection) and before the campaign (thin Ok labelled `Exact` despite
//! off-carrier); passes after (typed `Unsupported` with rollback at every
//! thickness).
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
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
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

fn assert_thin_wall_refuses(thickness: f64) {
    // Campaign narrowing: every wall on this oblique mixed lump refuses —
    // the inner sphere misses its carrier by 0.0009 at 0.05 through 0.048 at
    // 0.4 (1.8–11.9% of the wall, all indexed+welded shut pre-campaign yet
    // thousands of tolerance lengths off). Grouped attribution is the refusal
    // itself: it names the off-carrier sphere face, carrier, miss, and bound.
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
    let err = shell(&mut topo, body, thickness, &[]).expect_err(&format!(
        "thin wall {thickness} must refuse as off-carrier, not ship Exact"
    ));
    assert!(
        matches!(err, remus_operations::OperationsError::Unsupported { .. }),
        "thin {thickness}: typed refusal expected, got {err}"
    );
    let msg = format!("{err}");
    assert!(
        msg.contains("misses its carrier") && msg.contains("sphere"),
        "thin {thickness}: off-carrier sphere reason expected, got: {msg}"
    );
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
        before,
        "thin {thickness}: refusal must roll back"
    );
}

#[test]
fn thin_walls_refuse_as_off_carrier() {
    for t in [0.05, 0.1, 0.2, 0.3, 0.4] {
        assert_thin_wall_refuses(t);
    }
}

/// The previously pinned open-top thin hollow (single annulus rim) also
/// refuses: its inner sphere carries the same off-carrier boundary as the
/// closed thin walls. Valid planar cups (box open-top) still hollow — pinned
/// in the bounded-domain regression, not here.
#[test]
fn open_top_at_thin_wall_refuses_as_off_carrier() {
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
    let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
    let err = shell(&mut topo, body, 0.1, &open).expect_err("open thin must refuse");
    assert!(matches!(
        err,
        remus_operations::OperationsError::Unsupported { .. }
    ));
    assert!(format!("{err}").contains("misses its carrier"));
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
        before
    );
}

/// Scale: the same thin wall at 1000× refuses (carrier miss scales with the
/// wall, tolerance bound with the extent — the ratio never closes).
#[test]
fn scaled_thin_wall_refuses() {
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    transform_solid(&mut topo, body, &Mat4::scale(1000.0, 1000.0, 1000.0)).unwrap();
    let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
    let err = shell(&mut topo, body, 0.1 * 1000.0, &[]).expect_err("scaled thin must refuse");
    assert!(matches!(
        err,
        remus_operations::OperationsError::Unsupported { .. }
    ));
    assert!(format!("{err}").contains("misses its carrier"));
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
        before
    );
}

/// Rigid placement: translating the base body preserves the refusal at thin
/// and thick alike, with rollback.
#[test]
fn rigid_placement_preserves_outcome() {
    let place = Mat4::translation(10.0, -20.0, 30.0);
    for t in [0.1, 0.6] {
        let mut topo = Topology::new();
        let body = base_body(&mut topo);
        transform_solid(&mut topo, body, &place).unwrap();
        let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
        let err = shell(&mut topo, body, t, &[]).expect_err(&format!("placed {t} must refuse"));
        assert!(matches!(
            err,
            remus_operations::OperationsError::Unsupported { .. }
        ));
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
            before
        );
    }
}
