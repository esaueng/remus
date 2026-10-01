//! Shell refusal for a box fused with an oblique cylinder boss (mixed
//! planar/cylindrical holed lump).
//!
//! Fuzz Smoke run 36510764679 (`modifier_ops`,
//! crash-26090c907bfd89e210bb45b6721562302ab8bbb2): a 1x1.5x1.5 box fused
//! with an r=1.5 h=3.5 cylinder placed at (-1.5, 2.5, -0.5) after a 45-degree
//! rotation about Y (mode Fuse), then hollowed closed at thickness 0.6,
//! tessellated open (13 directed boundary edges at the harness deflection
//! 0.0012211470017978998, 10 at 0.1/0.01) while the base body is fully valid.
//!
//! Root cause (shell construction, not tessellation, not the base boolean):
//! the single lump carries 8 planes plus one holed cylinder lateral (the fuse
//! seam's inner wire, 4 ellipses + 1 line). The thin box walls cross
//! (1.0 - 2*0.6 < 0, inner x at 0.6 vs 0.4) exactly like the standalone box
//! and the B76 cut, but the fuse seam keeps the lump holed so the
//! hole-free-lump fold gate excludes it. Independently, the linear-miter
//! inner cylinder misses its offset carrier (r-t) by 0.72 at 0.6 (119% of the
//! wall; 0.074 at 0.2, 37%), so even where the planes do not cross (0.2-0.4)
//! the hollow meshes open while passing the relaxed validator with 0 free
//! B-Rep edges. An analytic carrier with an off-carrier boundary is not
//! exact.
//!
//! Fix (owning layer `shell_op.rs` only): every inner analytic-curved face
//! (cylinder, cone, sphere, torus) is checked against its offset carrier
//! before assembly and refused as unqualified when the linear-miter miss
//! exceeds the kernel tolerance contract (`Tolerance::linear` 1e-7,
//! scale-relativized), never a fraction of the wall. An analytic carrier with
//! an off-carrier boundary is not exact at any percentage: this witness misses
//! by 0.0043 at 0.05 (8.6%) through 0.72 at 0.6 (119%), all thousands of
//! tolerance lengths off, while B76 misses by 0.0009–0.096. Planes are exempt
//! (the miter is their exact intersection) and NURBS is exempt (already
//! disclosed `Approximate`). Coaxial sphere–cylinder bore rims are repaired
//! exactly via `exact_sphere_cylinder`, preserving drilled-sphere hollows. No
//! weld, epsilon, or tessellation change; the hole-free fold gate is untouched
//! (broadening it to holed lumps was tried and reverted: it blanket-refuses
//! valid hollows); B71 intact.
//!
//! Bounded exact domain, revised by the shell-correctness campaign:
//! this oblique holed mixed lump lies outside at EVERY thickness — thin
//! (0.05–0.16) previously hollowed `Exact` with indexed+welded closure despite
//! 8.6–29.3% carrier miss; it was mislabelled and now refuses like the thick
//! collapse. PROMINENT NARROWING: closed walls ≤0.16 no longer hollow; ≥0.18
//! still refuse. The base body stays valid/watertight/measured (pinned
//! separately); the defect is shell construction, not the boolean. Open-thin
//! box faces other than −Y previously leaked 3–5 indexed edges hidden by the
//! weld (0 welded) while −Y stayed shut; all six openings now refuse via the
//! same carrier gate without breaking valid planar/cylinder cups (pinned in
//! the bounded-domain regression). B76 open-thick (11/0) likewise refuses.
//!
//! Fails before PR #881 (shell Ok with 13 boundary edges at the harness
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
use remus_operations::primitives::{make_box, make_cylinder};
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
    let tool = make_cylinder(topo, 1.5, 3.5).unwrap();
    let place = Mat4::translation(-1.5, 2.5, -0.5) * Mat4::rotation_y(FRAC_PI_4);
    transform_solid(topo, tool, &place).unwrap();
    boolean(topo, BooleanOp::Fuse, stock, tool).unwrap()
}

/// The exact fuzz input body, before any shell: valid, watertight, and
/// measured consistently by both volume routes. Recorded separately from the
/// shell result per the assignment.
#[test]
fn base_body_is_valid_watertight_and_measured() {
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let faces = explorer::solid_faces(&topo, body).unwrap();
    assert_eq!(faces.len(), 9, "8 planes + 1 holed cylinder lateral");
    let mut planes = 0;
    let mut cylinders = 0;
    let mut holed = 0;
    for fid in &faces {
        let face = topo.face(*fid).unwrap();
        match face.surface().type_tag() {
            "plane" => planes += 1,
            "cylinder" => cylinders += 1,
            other => panic!("unexpected carrier {other}"),
        }
        if !face.inner_wires().is_empty() {
            holed += 1;
        }
    }
    assert_eq!((planes, cylinders), (8, 1));
    assert_eq!(holed, 1, "the fuse seam trims one cylinder lateral");
    // Geometric boundaries: every vertex on its carrier (planes by incidence,
    // curved carriers by projection).
    for fid in &faces {
        let face = topo.face(*fid).unwrap();
        if let remus_topology::face::FaceSurface::Plane { normal, d } = face.surface() {
            for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                for oe in topo.wire(wid).unwrap().edges() {
                    let edge = topo.edge(oe.edge()).unwrap();
                    for vid in [edge.start(), edge.end()] {
                        let p = topo.vertex(vid).unwrap().point();
                        let dev =
                            (normal.dot(remus_math::vec::Vec3::new(p.x(), p.y(), p.z())) - d).abs();
                        assert!(dev < 1e-9, "base vertex off plane on face {}", fid.index());
                    }
                }
            }
            continue;
        }
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                let edge = topo.edge(oe.edge()).unwrap();
                for vid in [edge.start(), edge.end()] {
                    let p = topo.vertex(vid).unwrap().point();
                    let (u, v) = face.surface().project_point(p).unwrap_or((0.0, 0.0));
                    let q = face.surface().evaluate(u, v).unwrap();
                    assert!(
                        (p - q).length() < 1e-9,
                        "base vertex off carrier on face {}",
                        fid.index()
                    );
                }
            }
        }
    }
    let map = explorer::edge_to_face_map(&topo, body).unwrap();
    assert_eq!(map.values().filter(|v| v.len() == 1).count(), 0);
    assert_eq!(map.values().filter(|v| v.len() > 2).count(), 0);
    let report = validate_solid_relaxed(&topo, body).unwrap();
    assert!(report.is_valid(), "base must be relaxed-valid");
    let aabb = solid_bounding_box(&topo, body).unwrap();
    let diag = (aabb.max - aabb.min).length();
    let defl = (diag * 4e-5).max(1e-7) * 4.0;
    // Indexed and welded closure at coarse, fine, and harness deflections.
    for d in [0.1, 0.01, defl] {
        let mesh = tessellate_solid(&topo, body, d).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "base d={d}");
        assert_eq!(non_manifold_edge_count(&mesh), 0);
        assert_eq!(welded_mesh_quality(&mesh).boundary_edges, 0);
    }
    let v = solid_volume(&topo, body, (diag * 4e-5).max(1e-7)).unwrap();
    let m = mass_properties(&topo, body).unwrap().mass;
    eprintln!("base solid_volume={v:.9} mass_properties={m:.9}");
    assert!((v - 26.464_032_777).abs() / 26.464_032_777 < 1e-3);
    assert!((v - m).abs() / v.max(m) < 1e-3);
    // Material probes on the base: box interior Inside, far field Outside.
    let inside = remus_math::vec::Point3::new(0.5, 0.75, 0.75);
    let outside = remus_math::vec::Point3::new(10.0, 10.0, 10.0);
    assert_eq!(
        remus_check::classify::classify_point(
            &topo,
            body,
            inside,
            &remus_check::classify::ClassifyOptions::default()
        )
        .unwrap(),
        remus_check::classify::PointClassification::Inside
    );
    assert_eq!(
        remus_check::classify::classify_point(
            &topo,
            body,
            outside,
            &remus_check::classify::ClassifyOptions::default()
        )
        .unwrap(),
        remus_check::classify::PointClassification::Outside
    );
}

/// The minimized crash: closed shell at 0.6 must refuse (fail-closed) with
/// rollback, never return the 13-boundary-edge solid.
#[test]
fn closed_shell_at_0_6_refuses_with_rollback() {
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
    let err = shell(&mut topo, body, 0.6, &[]).expect_err("collapsed/off-carrier wall must refuse");
    assert!(
        matches!(err, remus_operations::OperationsError::Unsupported { .. }),
        "bounded typed refusal expected, got {err}"
    );
    let msg = format!("{err}");
    assert!(
        msg.contains("misses its carrier") || msg.contains("self-intersection"),
        "refusal must name the introducing stage, got: {msg}"
    );
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
        before,
        "failed shell must roll back"
    );
}

/// The 0.5 wall also refuses (typed, with rollback). Before the curved gate
/// it surfaced as a non-manifold assembly; now the introducing stage fails
/// first with the off-carrier reason.
#[test]
fn closed_shell_at_0_5_refuses_with_rollback() {
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
    let err = shell(&mut topo, body, 0.5, &[]).expect_err("0.5 wall must refuse");
    assert!(
        matches!(err, remus_operations::OperationsError::Unsupported { .. }),
        "typed refusal expected, got {err}"
    );
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
        before
    );
}

/// Walls that meshed open before the gate (0.18-0.40) now refuse instead of
/// shipping an open hollow.
#[test]
fn closed_shell_past_the_carrier_bound_refuses() {
    for t in [0.18, 0.20, 0.30, 0.40] {
        let mut topo = Topology::new();
        let body = base_body(&mut topo);
        let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
        let err = shell(&mut topo, body, t, &[]).expect_err(&format!("wall {t} must refuse"));
        assert!(
            matches!(err, remus_operations::OperationsError::Unsupported { .. }),
            "wall {t}: typed refusal expected, got {err}"
        );
        assert!(
            format!("{err}").contains("misses its carrier"),
            "wall {t}: off-carrier reason expected, got {err}"
        );
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
            before,
            "wall {t}: rollback"
        );
    }
}

fn assert_thin_wall_refuses(thickness: f64) {
    // Campaign narrowing: every closed wall on this oblique holed lump
    // refuses — the inner cylinder misses by 0.0043 at 0.05 (8.6%) through
    // 0.047 at 0.16 (29.3%), all thousands of tolerance lengths off despite
    // indexed+welded closure pre-campaign. The refusal names the cylinder
    // face, carrier, miss, and bound (grouped attribution).
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
        msg.contains("misses its carrier") && msg.contains("cylinder"),
        "thin {thickness}: off-carrier cylinder reason expected, got: {msg}"
    );
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
        before,
        "thin {thickness}: refusal must roll back"
    );
}

#[test]
fn thin_walls_refuse_as_off_carrier() {
    for t in [0.05, 0.08, 0.10, 0.12, 0.14, 0.16] {
        assert_thin_wall_refuses(t);
    }
}

/// Every planar opening at a thin wall now refuses via the same carrier gate:
/// pre-campaign five of six box sides leaked 3–5 indexed edges hidden by the
/// weld (0 welded) while −Y stayed shut indexed. No indexed-leak body ships;
/// valid planar/cylinder cups still hollow — pinned in the bounded-domain
/// regression, not here.
#[test]
fn open_thin_refuses_on_every_box_side() {
    let sides = [
        ((1.0, 0.0, 0.0), 1.0),
        ((-1.0, 0.0, 0.0), 0.0),
        ((0.0, 1.0, 0.0), 1.5),
        ((0.0, -1.0, 0.0), 0.0),
        ((0.0, 0.0, 1.0), 1.5),
        ((0.0, 0.0, -1.0), 0.0),
    ];
    for (normal, d) in sides {
        let mut topo = Topology::new();
        let body = base_body(&mut topo);
        let faces = explorer::solid_faces(&topo, body).unwrap();
        let open: Vec<_> = faces
            .iter()
            .filter(|fid| {
                let f = topo.face(**fid).unwrap();
                matches!(f.surface(), remus_topology::face::FaceSurface::Plane { normal: n, d: dd } if (n.x()-normal.0).abs()<1e-9 && (n.y()-normal.1).abs()<1e-9 && (n.z()-normal.2).abs()<1e-9 && (*dd-d).abs()<1e-9)
            })
            .copied()
            .collect();
        assert_eq!(open.len(), 1, "side {normal:?} must exist");
        let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
        let err = shell(&mut topo, body, 0.1, &open)
            .expect_err(&format!("open thin side {normal:?} must refuse"));
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
}

/// Opening the cylinder lateral is not a planar opening and must refuse.
#[test]
fn open_cylinder_lateral_refuses() {
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let faces = explorer::solid_faces(&topo, body).unwrap();
    let cyl: Vec<_> = faces
        .iter()
        .filter(|fid| {
            matches!(
                topo.face(**fid).unwrap().surface(),
                remus_topology::face::FaceSurface::Cylinder(_)
            )
        })
        .copied()
        .collect();
    assert_eq!(cyl.len(), 1);
    let err = shell(&mut topo, body, 0.1, &cyl).expect_err("non-planar opening must refuse");
    assert!(matches!(
        err,
        remus_operations::OperationsError::Unsupported { .. }
    ));
}

/// Any planar opening at the collapsed/off-carrier thickness refuses with
/// rollback (the open-thick regime ships open without the gate).
#[test]
fn open_at_thick_refuses_with_rollback() {
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let faces = explorer::solid_faces(&topo, body).unwrap();
    let open: Vec<_> = faces
        .iter()
        .filter(|fid| {
            let f = topo.face(**fid).unwrap();
            matches!(f.surface(), remus_topology::face::FaceSurface::Plane { normal, d } if (*normal - remus_math::vec::Vec3::new(0.0, -1.0, 0.0)).length() < 1e-9 && d.abs() < 1e-9)
        })
        .copied()
        .collect();
    assert_eq!(open.len(), 1);
    let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
    let err = shell(&mut topo, body, 0.6, &open).expect_err("open thick must refuse");
    assert!(matches!(
        err,
        remus_operations::OperationsError::Unsupported { .. }
    ));
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
        before
    );
}

/// Scale: the same thin wall at 1000x refuses (carrier miss scales with the
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
/// and thick alike, with rollback. The pre-campaign translation-oracle
/// (bit-identical mesh volumes, B59-class `solid_volume` drift) is retired
/// with the thin success it measured: there is no thin hollow whose mesh
/// volume to compare. The origin-dependence handoff to the measurement owner
/// stands on the B76 row and the placed-positive pins in the bounded-domain
/// regression.
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
        assert!(format!("{err}").contains("misses its carrier"));
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
            before
        );
    }
}

/// State: refused shells leave the original handle live and publish nothing —
/// thick then thin both refuse, and the failed attempts poison neither the
/// handle nor the journal.
#[test]
fn refused_thick_then_thin_both_refuse_without_poisoning_the_handle() {
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
    for t in [0.6, 0.1] {
        let err = shell(&mut topo, body, t, &[]).expect_err(&format!("{t} must refuse"));
        assert!(matches!(
            err,
            remus_operations::OperationsError::Unsupported { .. }
        ));
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
            before,
            "{t}: refusal must roll back"
        );
    }
}

/// State: attributes on the base survive refused shells (thin and thick).
#[test]
fn attributes_survive_refusals() {
    use remus_topology::attributes::EntityAttributes;
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    topo.set_solid_attributes(
        body,
        EntityAttributes {
            name: Some("boss-body".to_string()),
            color: None,
        },
    )
    .unwrap();
    let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
    for t in [0.6, 0.1] {
        let _ = shell(&mut topo, body, t, &[]).expect_err("must refuse");
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
            before
        );
        assert_eq!(
            topo.attributes().solid(body).and_then(|a| a.name.clone()),
            Some("boss-body".to_string()),
            "refusal must not drop base attributes"
        );
    }
}

/// History: refused shells (thin and thick) publish no journal entries.
#[test]
fn journaled_refusals_publish_nothing() {
    use remus_operations::journal_ops::shell_journaled;
    for t in [0.1, 0.6] {
        let mut topo = Topology::new();
        let body = base_body(&mut topo);
        let before = topo.journal().entries().len();
        let err = shell_journaled(&mut topo, body, t, &[]).expect_err("must refuse");
        assert!(matches!(
            err,
            remus_operations::OperationsError::Unsupported { .. }
        ));
        assert_eq!(
            topo.journal().entries().len(),
            before,
            "refused shell must publish no journal entry"
        );
    }
}

/// Quality: the exact-only outcome twin refuses the thin hollow (no `Exact`
/// over an off-carrier wall); the explicit NURBS approximation policy is
/// preserved elsewhere (`shell_nurbs_policy_refuses_exact_and_discloses_approximation`).
#[test]
fn outcome_quality_refuses_the_thin_hollow() {
    use remus_operations::shell_op::shell_outcome_with_evolution;
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
    let err = shell_outcome_with_evolution(&mut topo, body, 0.1, &[], None)
        .expect_err("thin must refuse exact-only");
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
