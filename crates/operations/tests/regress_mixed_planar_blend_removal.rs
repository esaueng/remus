//! Plane-to-plane fillet removal on bodies that also carry curved geometry.
//!
//! Before this regression existed, `heal_by_extending` required every kept
//! face to be planar and refused with "extending the shell to close the gap
//! is only implemented for planar faces" as soon as the solid contained an
//! unrelated cylinder, sphere, torus, or cone — even when the two blend
//! supports themselves were planar. Simply deleting that guard would have
//! faceted the curved faces, because the old rebuild re-emitted every kept
//! face as a planar polygon.
//!
//! The heal now rebuilds only the wound-adjacent faces and carries every
//! other face verbatim through `FaceSpec::Existing` (exact surface, curves,
//! orientation, holes). Wound-adjacent faces without deleted holes keep
//! their holes verbatim too; only their outer wire is re-trimmed. A curved
//! wound neighbor, or a wound edge reaching an inner wire, is still an
//! explicit typed refusal.
//!
//! Every test asserts exactness (validated shell, exact signed volume change,
//! surviving analytic geometry) or a typed refusal with rollback. Tessellation
//! success is never used as proof.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_operations::blend_ops::fillet_v2;
use remus_operations::boolean::{BooleanOp, boolean, face_polygon};
use remus_operations::defeature::defeature;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::resize_blend::resize_blend;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

/// Filleted box crossed by an unrelated through-bore.
///
/// The vertical edge at (40, 40) carries an R3 plane-plane fillet; a separate
/// R3 bore through (10, 10) supplies an unrelated cylindrical kept face and
/// hole rims on both fillet end faces. The old code refused this body because
/// the bore wall is a cylinder; the exact heal must remove only the fillet.
fn mixed_filleted_body(topo: &mut Topology) -> (SolidId, FaceId, f64, f64) {
    let sharp = make_box(topo, 40.0, 40.0, 10.0).unwrap();
    let edge = remus_topology::explorer::solid_edges(topo, sharp)
        .unwrap()
        .into_iter()
        .find(|edge_id| {
            let edge = topo.edge(*edge_id).unwrap();
            [edge.start(), edge.end()].iter().all(|vertex| {
                let point = topo.vertex(*vertex).unwrap().point();
                (point.x() - 40.0).abs() < 1e-9 && (point.y() - 40.0).abs() < 1e-9
            })
        })
        .unwrap();
    let filleted = fillet_v2(topo, sharp, &[edge], 3.0).unwrap().solid;
    let drill = make_cylinder(topo, 3.0, 20.0).unwrap();
    remus_operations::transform::transform_solid(topo, drill, &Mat4::translation(10.0, 10.0, -5.0))
        .unwrap();
    let mixed = boolean(topo, BooleanOp::Cut, filleted, drill).unwrap();
    let volume = remus_operations::measure::solid_volume(topo, mixed, 0.02).unwrap();
    // Select the fillet band geometrically: the R3 cylinder near the (40,40)
    // corner. The R3 bore wall near (10, 10) is excluded by position, so no
    // arena handle is hard-coded.
    let band = remus_topology::explorer::solid_faces(topo, mixed)
        .unwrap()
        .into_iter()
        .find(|face| {
            if !matches!(
                topo.face(*face).unwrap().surface(),
                FaceSurface::Cylinder(cylinder) if (cylinder.radius() - 3.0).abs() < 1e-6
            ) {
                return false;
            }
            let polygon = face_polygon(topo, *face).unwrap_or_default();
            if polygon.is_empty() {
                return false;
            }
            #[allow(clippy::cast_precision_loss)]
            let n = polygon.len() as f64;
            let (cx, cy) = polygon.iter().fold((0.0, 0.0), |(x, y), point| {
                (x + point.x() / n, y + point.y() / n)
            });
            // Fillet band hugs the (40, 40) corner; the bore wall is at (10, 10).
            cx > 30.0 && cy > 30.0
        })
        .expect("one R3 plane-plane fillet band");
    (
        mixed,
        band,
        volume,
        10.0 * 9.0 * (1.0 - std::f64::consts::PI / 4.0),
    )
}

fn assert_valid(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(
        report.is_valid(),
        "healed solid must validate, got: {:?}",
        report
            .issues
            .iter()
            .map(|issue| issue.description.clone())
            .collect::<Vec<_>>()
    );
}

/// The sharp edge recovered at the (40, 40) corner: a line of length 10
/// shared by two planes.
fn sharp_edge_at_corner(topo: &Topology, solid: SolidId) -> bool {
    let adjacency = topo.build_adjacency(solid).unwrap();
    remus_topology::explorer::solid_edges(topo, solid)
        .unwrap()
        .iter()
        .any(|edge_id| {
            if !matches!(topo.edge(*edge_id).unwrap().curve(), EdgeCurve::Line) {
                return false;
            }
            let adjacent = adjacency.faces_for_edge(*edge_id);
            if adjacent.len() != 2
                || !adjacent.iter().all(|face| {
                    topo.face(*face)
                        .is_ok_and(|face| face.surface().is_planar())
                })
            {
                return false;
            }
            let edge = topo.edge(*edge_id).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            ((start.x() - 40.0).abs() < 1e-6 && (end.x() - 40.0).abs() < 1e-6
                || (start.y() - 40.0).abs() < 1e-6 && (end.y() - 40.0).abs() < 1e-6)
                && (end - start).length() - 10.0 < 1e-6
        })
}

#[test]
fn mixed_body_planar_fillet_removal_is_exact() {
    let mut topo = Topology::new();
    let (mixed, band, volume_before, expected_gain) = mixed_filleted_body(&mut topo);
    let faces_before = remus_topology::explorer::solid_faces(&topo, mixed)
        .unwrap()
        .len();

    let healed = defeature(&mut topo, mixed, &[band]).unwrap();
    assert_valid(&topo, healed);
    let volume_after = remus_operations::measure::solid_volume(&topo, healed, 0.02).unwrap();
    // Signed volume change is exactly the removed fillet prism, not a broad
    // plausibility band: L * r^2 * (1 - pi/4) for a right-angle round.
    assert!(
        (volume_after - volume_before - expected_gain).abs() < 1e-6,
        "volume gain {} differs from fillet prism {expected_gain}",
        volume_after - volume_before
    );
    // One face fewer (the band), nothing else created or absorbed.
    let faces_after = remus_topology::explorer::solid_faces(&topo, healed).unwrap();
    assert_eq!(faces_after.len(), faces_before - 1);
    // The bore wall survives as the only cylinder, with its exact radius.
    let cylinders: Vec<FaceId> = faces_after
        .iter()
        .filter(|face| {
            matches!(
                topo.face(**face).unwrap().surface(),
                FaceSurface::Cylinder(_)
            )
        })
        .copied()
        .collect();
    assert_eq!(
        cylinders.len(),
        1,
        "bore wall survives, fillet band is gone"
    );
    let FaceSurface::Cylinder(wall) = topo.face(cylinders[0]).unwrap().surface() else {
        unreachable!("filtered to the one cylinder");
    };
    assert!((wall.radius() - 3.0).abs() < 1e-9);
    // Bore pose is pinned, not just its radius: axis parallel to Z through
    // (10, 10). A rigidly displaced bore wall must not pass.
    let axis = wall.axis().normalize().unwrap();
    assert!(
        axis.x().abs() < 1e-12 && axis.y().abs() < 1e-12 && (axis.z().abs() - 1.0).abs() < 1e-12,
        "bore axis stays parallel to Z, got ({:.3e}, {:.3e}, {:.3e})",
        axis.x(),
        axis.y(),
        axis.z()
    );
    assert!(
        (wall.origin().x() - 10.0).abs() < 1e-9 && (wall.origin().y() - 10.0).abs() < 1e-9,
        "bore axis stays through (10, 10), got ({:.6}, {:.6})",
        wall.origin().x(),
        wall.origin().y()
    );
    // Both bore rims survive as inner wires on the healed end faces.
    let holes: usize = faces_after
        .iter()
        .map(|face| topo.face(*face).unwrap().inner_wires().len())
        .sum();
    assert_eq!(holes, 2, "unrelated bore rims survive the heal");
    // The sharp support intersection is restored at the filleted corner.
    assert!(
        sharp_edge_at_corner(&topo, healed),
        "recovered sharp edge at (40, 40) of length 10"
    );
    // The input solid is untouched by the successful heal.
    assert!(validate_solid(&topo, mixed).unwrap().is_valid());
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, mixed)
            .unwrap()
            .len(),
        faces_before
    );
}

#[test]
fn mixed_body_resize_blend_removal_reports_exact_evolution() {
    let mut topo = Topology::new();
    let (mixed, band, volume_before, expected_gain) = mixed_filleted_body(&mut topo);

    let result = resize_blend(&mut topo, mixed, band, 3.0, 0.0).unwrap();
    assert_valid(&topo, result.solid);
    let volume_after = remus_operations::measure::solid_volume(&topo, result.solid, 0.02).unwrap();
    assert!((volume_after - volume_before - expected_gain).abs() < 1e-6);
    // Face evolution: the band is deleted, every other face is modified
    // (wound neighbors re-trimmed, preserved faces carried through).
    assert!(result.evolution.deleted.contains(&band.index()));
    let surviving = remus_topology::explorer::solid_faces(&topo, mixed)
        .unwrap()
        .into_iter()
        .filter(|face| *face != band)
        .count();
    assert_eq!(result.evolution.modified.len(), surviving);
    // No surviving face changes its carrier: planes keep normal and offset,
    // cylinders keep radius, axis, and origin. This pins "preserve unrelated
    // geometry exactly" for every analytic type through the same verbatim
    // path that would carry the towel-rack spheres, tori, and cones.
    for (&source_index, targets) in &result.evolution.modified {
        let [target_index] = targets.as_slice() else {
            panic!("one-to-one face evolution, got {targets:?}");
        };
        let source = topo.face_id_from_index(source_index).unwrap();
        let target = topo.face_id_from_index(*target_index).unwrap();
        assert_same_surface(&topo, source, target);
    }
    // STEP export/reimport preserves the exact invariants, not just a mesh.
    let step = remus_io::step::write_step(&topo, &[result.solid]).unwrap();
    let mut reread = Topology::new();
    let solid = remus_io::step::read_step(&step, &mut reread).unwrap()[0];
    assert_valid(&reread, solid);
    let reread_volume = remus_operations::measure::solid_volume(&reread, solid, 0.02).unwrap();
    assert!((reread_volume - volume_after).abs() < 1e-6 * volume_after.max(1.0));
    let reread_cylinders = remus_topology::explorer::solid_faces(&reread, solid)
        .unwrap()
        .iter()
        .filter(|face| !reread.face(**face).unwrap().surface().is_planar())
        .count();
    assert_eq!(
        reread_cylinders, 1,
        "bore wall survives the STEP round-trip"
    );
}

/// Assert two faces share the same analytic carrier parameters.
///
/// Supports grow within their own planes, so even re-trimmed faces keep
/// their surface; preserved faces are verbatim clones. Any silent carrier
/// swap (cylinder faceted to NURBS, plane tilted, radius changed) fails here.
fn assert_same_surface(topo: &Topology, source: FaceId, target: FaceId) {
    match (
        topo.face(source).unwrap().surface(),
        topo.face(target).unwrap().surface(),
    ) {
        (
            FaceSurface::Plane {
                normal: source_normal,
                d: source_d,
            },
            FaceSurface::Plane {
                normal: target_normal,
                d: target_d,
            },
        ) => {
            assert!(
                (source_normal.x() - target_normal.x()).abs() < 1e-12
                    && (source_normal.y() - target_normal.y()).abs() < 1e-12
                    && (source_normal.z() - target_normal.z()).abs() < 1e-12
                    && (source_d - target_d).abs() < 1e-9,
                "plane carrier moved for face {}",
                source.index()
            );
        }
        (FaceSurface::Cylinder(source_cyl), FaceSurface::Cylinder(target_cyl)) => {
            assert!(
                (source_cyl.radius() - target_cyl.radius()).abs() < 1e-12
                    && (source_cyl.axis() - target_cyl.axis()).length() < 1e-12
                    && (source_cyl.origin() - target_cyl.origin()).length() < 1e-9,
                "cylinder carrier moved for face {}",
                source.index()
            );
        }
        (source_surface, target_surface) => {
            assert_eq!(
                source_surface.type_tag(),
                target_surface.type_tag(),
                "surface type changed for face {}",
                source.index()
            );
        }
    }
}

#[test]
fn mixed_body_witness_mismatch_rolls_back_without_mutation() {
    let mut topo = Topology::new();
    let (mixed, band, volume_before, _) = mixed_filleted_body(&mut topo);
    let faces_before = remus_topology::explorer::solid_faces(&topo, mixed)
        .unwrap()
        .len();

    let error = resize_blend(&mut topo, mixed, band, 2.0, 0.0).unwrap_err();
    assert!(
        format!("{error}").contains("expected radius"),
        "witness mismatch must name the radius, got {error:?}"
    );
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, mixed)
            .unwrap()
            .len(),
        faces_before,
        "refusal must not mutate the input"
    );
    assert!(
        (remus_operations::measure::solid_volume(&topo, mixed, 0.02).unwrap() - volume_before)
            .abs()
            < 1e-12
    );
}

#[test]
fn curved_wound_neighbor_refuses_without_mutation() {
    // A top face whose wound reaches the cylindrical boss wall: the wound
    // neighbor is curved, so extension has no exact construction. Unrelated
    // curved faces are preserved, but a curved face ON the wound is refused.
    let mut topo = Topology::new();
    let base = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
    let boss = make_cylinder(&mut topo, 3.0, 4.0).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        boss,
        &Mat4::translation(10.0, 10.0, 10.0),
    )
    .unwrap();
    let fused = boolean(&mut topo, BooleanOp::Fuse, base, boss).unwrap();
    let top = remus_topology::explorer::solid_faces(&topo, fused)
        .unwrap()
        .into_iter()
        .find(|face| {
            face_polygon(&topo, *face).is_ok_and(|polygon| {
                #[allow(clippy::cast_precision_loss)]
                let n = polygon.len() as f64;
                let centroid_z = polygon.iter().map(|point| point.z()).sum::<f64>() / n;
                (centroid_z - 10.0).abs() < 0.5 && topo.face(*face).unwrap().surface().is_planar()
            })
        })
        .unwrap();
    let faces_before = remus_topology::explorer::solid_faces(&topo, fused)
        .unwrap()
        .len();
    let volume_before = remus_operations::measure::solid_volume(&topo, fused, 0.02).unwrap();

    let error = defeature(&mut topo, fused, &[top]).unwrap_err();
    assert!(
        matches!(error, remus_operations::OperationsError::Unsupported { .. }),
        "curved wound neighbor must be a typed refusal, got {error:?}"
    );
    assert!(
        format!("{error}").contains("wound-neighbor"),
        "refusal must name the wound neighbor, got {error:?}"
    );
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, fused)
            .unwrap()
            .len(),
        faces_before,
        "refusal must not mutate the input"
    );
    assert!(
        (remus_operations::measure::solid_volume(&topo, fused, 0.02).unwrap() - volume_before)
            .abs()
            < 1e-12
    );
    assert!(validate_solid(&topo, fused).unwrap().is_valid());
}
