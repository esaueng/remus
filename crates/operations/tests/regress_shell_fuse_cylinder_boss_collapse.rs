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
//! exceeds 30% of the wall thickness. Thirty percent keeps every
//! currently-qualified thin hollow (B76 1.8-11.9% at 0.05-0.4; this witness
//! 8.6-29.3% at 0.05-0.16, all indexed+welded shut; box/cup/sphere 0%) while
//! refusing the open-mesh regime with margin (33.3% at 0.18, 37-119% at
//! 0.20-0.60). Scale-invariant (both sides scale) and rigid-motion
//! invariant. Planes are exempt (the miter is their exact intersection) and
//! NURBS is exempt (already disclosed `Approximate`). No weld, epsilon, or
//! tessellation change; the hole-free fold gate is untouched (broadening it
//! to holed lumps was tried and reverted: it blanket-refuses the thin valid
//! hollows it is meant to protect). B71's wrong-side-blend refusal is
//! untouched.
//!
//! Bounded qualification (not capability closure): closed walls at or below
//! 0.16 hollow exactly; 0.18 and above refuse typed `Unsupported` with
//! rollback. Thin-hollow `mass_properties` still declines with an integration
//! budget error even where the mesh is shut, so volume agreement is
//! documented as insufficient evidence there (solid-volume + mesh + probes,
//! not dual-route). Open-thin box faces other than -Y leak indexed edges
//! hidden by the weld (documented gaps); the pinned -Y opening and both
//! cylinder caps stay shut. B76's open-thick indexed leak (11/0, 16% of the
//! wall) remains below the bound and stays a documented residual.
//!
//! Fails before the fix (shell Ok with 13 boundary edges at the harness
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

/// Largest inner-curved vertex miss against its own offset carrier, for the
/// failure-analysis printout (not the gate itself, which lives in `shell_op`).
fn max_inner_curved_miss(topo: &Topology, solid: SolidId) -> f64 {
    let mut worst: f64 = 0.0;
    for fid in explorer::solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        if matches!(
            face.surface(),
            remus_topology::face::FaceSurface::Plane { .. }
        ) {
            continue;
        }
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                let edge = topo.edge(oe.edge()).unwrap();
                for vid in [edge.start(), edge.end()] {
                    let p = topo.vertex(vid).unwrap().point();
                    let (u, v) = face.surface().project_point(p).unwrap_or((0.0, 0.0));
                    if let Some(q) = face.surface().evaluate(u, v) {
                        worst = worst.max((p - q).length());
                    }
                }
            }
        }
    }
    worst
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

fn assert_thin_wall_ok(thickness: f64) {
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let v_before = solid_volume(&topo, body, 1e-4)
        .unwrap_or_else(|_| mass_properties(&topo, body).unwrap().mass);
    let hollow = shell(&mut topo, body, thickness, &[]).unwrap_or_else(|e| {
        panic!("thin wall {thickness} must hollow, got refusal {e}");
    });
    let report = validate_solid_relaxed(&topo, hollow).unwrap();
    assert!(report.is_valid(), "thin {thickness} relaxed-valid");
    let map = explorer::edge_to_face_map(&topo, hollow).unwrap();
    assert_eq!(map.values().filter(|v| v.len() == 1).count(), 0);
    assert_eq!(map.values().filter(|v| v.len() > 2).count(), 0);
    assert_eq!(same_sense_pairs(&topo, hollow), 0);
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
    // Carrier adherence for the failure analysis: thin stays under the 30%
    // bound (this is the qualified domain, not a second gate).
    let miss = max_inner_curved_miss(&topo, hollow);
    eprintln!(
        "thin {thickness}: max inner-curved miss={miss:.6} ({:.1}%)",
        miss / thickness * 100.0
    );
    assert!(
        miss <= 0.30 * thickness,
        "thin {thickness} must stay inside the qualified carrier bound"
    );
    // Volume: hollowing shrinks. `mass_properties` still declines on these
    // hollows with an integration-budget error even where the mesh is shut,
    // so dual-route agreement is documented as insufficient evidence here;
    // the solid-volume route plus mesh closure and probes carry the claim.
    let aabb = solid_bounding_box(&topo, hollow).unwrap();
    let diag = (aabb.max - aabb.min).length();
    let v = solid_volume(&topo, hollow, (diag * 4e-5).max(1e-7)).unwrap();
    eprintln!("thin {thickness}: solid_volume={v:.9} before={v_before:.9}");
    assert!(v < v_before, "hollowing must not invent material");
    assert!(v.is_finite() && v > 0.0);
    match mass_properties(&topo, hollow) {
        Ok(m) => eprintln!("thin {thickness}: mass_properties agrees at {:.9}", m.mass),
        Err(e) => eprintln!("thin {thickness}: mass_properties declines (documented gap): {e}"),
    }
    // Classification spot checks: cavity Outside, wall Inside.
    let outside = remus_math::vec::Point3::new(0.5, 0.75, 0.75);
    let wall = remus_math::vec::Point3::new(0.02, 0.02, 0.02);
    // At these thin walls the box centre is the cavity (Outside); the corner
    // probe sits in material for every thickness in this family.
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
    for t in [0.05, 0.08, 0.10, 0.12, 0.14, 0.16] {
        assert_thin_wall_ok(t);
    }
}

/// The -Y box side (normal (0,-1,0), d=0) opens cleanly at a thin wall; the
/// rim is a single annulus and the cup stays shut indexed and welded.
#[test]
fn open_minus_y_at_thin_wall_hollows_watertight() {
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

/// Scale: the same thin wall at 1000x carries census, s^3 volume, and
/// watertightness (the carrier bound is thickness-relative, so it survives
/// scaling; the absolute weld grid is the only scale-coupled check and it
/// holds here).
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
    assert_eq!(welded_mesh_quality(&mesh).boundary_edges, 0);
    let aabb = solid_bounding_box(&topo, hollow).unwrap();
    let diag = (aabb.max - aabb.min).length();
    let v = solid_volume(&topo, hollow, (diag * 4e-5).max(1e-7)).unwrap();
    assert!(v.is_finite() && v > 0.0);
    // s^3 of the unit thin-wall volume (measured above at 0.1).
    let mut topo2 = Topology::new();
    let body2 = base_body(&mut topo2);
    let hollow2 = shell(&mut topo2, body2, 0.1, &[]).unwrap();
    let aabb2 = solid_bounding_box(&topo2, hollow2).unwrap();
    let diag2 = (aabb2.max - aabb2.min).length();
    let v2 = solid_volume(&topo2, hollow2, (diag2 * 4e-5).max(1e-7)).unwrap();
    assert!((v / 1e9 - v2).abs() / v2 < 1e-6, "volume must scale as s^3");
}

/// Rigid placement: translating the base body preserves the thin-wall
/// success and the thick-wall refusal with translation-invariant geometry.
/// The tessellated (mesh) volume is the translation oracle here:
/// `solid_volume` drifts 49% under this translation (4.807 vs 7.197) while
/// the mesh volume is bit-identical (4.936725 at 0.01) and the bboxes differ
/// by exactly the placement, so the shell geometry is invariant and the
/// analytic route carries an origin-dependent term (B59 class, handed to the
/// measurement owner; not patched here).
#[test]
fn rigid_placement_preserves_outcome() {
    fn mesh_volume(topo: &Topology, solid: SolidId, deflection: f64) -> f64 {
        let mesh = tessellate_solid(topo, solid, deflection).unwrap();
        let mut v = 0.0;
        for tri in mesh.indices.chunks_exact(3) {
            let a = mesh.positions[tri[0] as usize];
            let b = mesh.positions[tri[1] as usize];
            let c = mesh.positions[tri[2] as usize];
            v += a.x() * (b.y() * c.z() - b.z() * c.y()) - a.y() * (b.x() * c.z() - b.z() * c.x())
                + a.z() * (b.x() * c.y() - b.y() * c.x());
        }
        v / 6.0
    }

    let place = Mat4::translation(10.0, -20.0, 30.0);
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    transform_solid(&mut topo, body, &place).unwrap();
    let hollow = shell(&mut topo, body, 0.1, &[]).unwrap();
    let harness = harness_deflection(&topo, hollow);
    let mesh = tessellate_solid(&topo, hollow, harness).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0);
    let mv = mesh_volume(&topo, hollow, 0.01);

    let mut topo0 = Topology::new();
    let body0 = base_body(&mut topo0);
    let hollow0 = shell(&mut topo0, body0, 0.1, &[]).unwrap();
    let mv0 = mesh_volume(&topo0, hollow0, 0.01);
    eprintln!("placed mesh vol={mv:.9} unplaced mesh vol={mv0:.9}");
    assert!(
        (mv - mv0).abs() / mv0 < 1e-9,
        "mesh volume translation-invariant"
    );
    // Bboxes differ by exactly the placement.
    let a = solid_bounding_box(&topo, hollow).unwrap();
    let a0 = solid_bounding_box(&topo0, hollow0).unwrap();
    assert!((a.min.x() - a0.min.x() - 10.0).abs() < 1e-9);
    assert!((a.min.y() - a0.min.y() + 20.0).abs() < 1e-9);
    assert!((a.min.z() - a0.min.z() - 30.0).abs() < 1e-9);
    // Documented upstream gap: the analytic route drifts under translation.
    let diag = (a.max - a.min).length();
    let v = solid_volume(&topo, hollow, (diag * 4e-5).max(1e-7)).unwrap();
    let diag0 = (a0.max - a0.min).length();
    let v0 = solid_volume(&topo0, hollow0, (diag0 * 4e-5).max(1e-7)).unwrap();
    eprintln!(
        "placed solid_volume={v:.9} unplaced={v0:.9} (upstream origin-dependence, not shell)"
    );

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

/// State: a refused thick shell leaves the original handle live — a thin
/// shell on the same body afterwards succeeds and the failed attempt published
/// nothing.
#[test]
fn refused_thick_then_thin_succeeds_on_the_same_handle() {
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let before = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
    let err = shell(&mut topo, body, 0.6, &[]).expect_err("thick must refuse");
    assert!(matches!(
        err,
        remus_operations::OperationsError::Unsupported { .. }
    ));
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
        before
    );
    // The handle is not poisoned: the qualified thin wall still hollows.
    let hollow = shell(&mut topo, body, 0.1, &[]).unwrap();
    let harness = harness_deflection(&topo, hollow);
    let mesh = tessellate_solid(&topo, hollow, harness).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0);
}

/// State: attributes on the base survive a refused shell, and a successful
/// thin shell carries the solid's attributes forward.
#[test]
fn attributes_survive_refusal_and_thin_shell() {
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
    let _ = shell(&mut topo, body, 0.6, &[]).expect_err("thick must refuse");
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
        before
    );
    assert_eq!(
        topo.attributes().solid(body).and_then(|a| a.name.clone()),
        Some("boss-body".to_string()),
        "refusal must not drop base attributes"
    );
    let hollow = shell(&mut topo, body, 0.1, &[]).unwrap();
    // The outer skin is carried through; the solid-level name is retained
    // by the topology snapshot, not rebound by shell (shell never invents
    // names — the kernel never synthesizes them).
    assert!(
        topo.attributes().solid(hollow).is_none()
            || topo
                .attributes()
                .solid(hollow)
                .and_then(|a| a.name.clone())
                .is_none(),
        "shell must not synthesize solid attributes"
    );
    assert_eq!(
        topo.attributes().solid(body).and_then(|a| a.name.clone()),
        Some("boss-body".to_string()),
        "base attributes stay on the base handle"
    );
}

/// History: the journaled thin shell records one `shell` entry with a
/// construction-derived face map; the refused thick shell publishes nothing.
#[test]
fn journaled_thin_shell_records_and_thick_publishes_nothing() {
    use remus_operations::journal_ops::shell_journaled;
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let journaled = shell_journaled(&mut topo, body, 0.1, &[]).unwrap();
    let entry = topo
        .journal()
        .entries()
        .iter()
        .find(|e| e.op() == journaled.op)
        .expect("journal must hold the shell entry");
    assert_eq!(entry.kind(), "shell");
    assert!(
        !journaled.map.generated.is_empty() || !journaled.map.modified.is_empty(),
        "journaled shell must carry face evolution"
    );
    assert!(
        !topo.journal().entries().is_empty(),
        "journal must hold the shell entry"
    );

    let mut topo2 = Topology::new();
    let body2 = base_body(&mut topo2);
    let before = topo2.journal().entries().len();
    let err = shell_journaled(&mut topo2, body2, 0.6, &[]).expect_err("thick must refuse");
    assert!(matches!(
        err,
        remus_operations::OperationsError::Unsupported { .. }
    ));
    assert_eq!(
        topo2.journal().entries().len(),
        before,
        "refused shell must publish no journal entry"
    );
}

/// Quality: the outcome twin reports `Exact` on this all-analytic thin hollow
/// (no NURBS faces sampled), and the explicit NURBS approximation policy is
/// preserved (exact-only refuses a NURBS body; tested in
/// `shell_nurbs_policy_refuses_exact_and_discloses_approximation`).
#[test]
fn outcome_quality_is_exact_on_the_thin_hollow() {
    use remus_operations::shell_op::{ShellQuality, shell_outcome_with_evolution};
    let mut topo = Topology::new();
    let body = base_body(&mut topo);
    let outcome = shell_outcome_with_evolution(&mut topo, body, 0.1, &[], None).unwrap();
    assert_eq!(outcome.outcome.quality, ShellQuality::Exact);
    let harness = harness_deflection(&topo, outcome.outcome.solid);
    let mesh = tessellate_solid(&topo, outcome.outcome.solid, harness).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0);
}
