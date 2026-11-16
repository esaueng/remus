//! Deleting a chamfer band that meets a curved face must restore the sharp rim.
//!
//! A chamfer on a hole or boss rim is a closed cone between a plane and a
//! cylinder. `defeature` extends only planar wound neighbours, so it refused
//! these with "wound-neighbor face is a cylinder surface", even though the
//! same supports carrying a rolling-ball blend instead heal exactly. A CAD
//! user picking that cone on an imported part and pressing Delete got a
//! refusal. Each case here builds the unchamfered solid directly and asserts
//! the heal reproduces it: validated shell, no cone left, the same faces and
//! the same volume, measured the same way, with complete exact history.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_operations::blend_ops::chamfer_v2;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::defeature::{defeature, defeature_with_evolution};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use std::collections::BTreeSet;

fn translate(topo: &mut Topology, solid: SolidId, x: f64, y: f64, z: f64) {
    transform_solid(topo, solid, &Mat4::translation(x, y, z)).unwrap();
}

/// The closed circular edge of `radius` lying in the plane `z`.
fn rim(topo: &Topology, solid: SolidId, radius: f64, z: f64) -> EdgeId {
    let rims: Vec<EdgeId> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| match topo.edge(edge).unwrap().curve() {
            EdgeCurve::Circle(circle) => {
                (circle.radius() - radius).abs() < 1e-9 && (circle.center().z() - z).abs() < 1e-9
            }
            _ => false,
        })
        .collect();
    assert_eq!(rims.len(), 1, "one rim of radius {radius} at z = {z}");
    rims[0]
}

fn cones(topo: &Topology, solid: SolidId) -> Vec<FaceId> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cone(_)))
        .collect()
}

/// `healed` is the exact solid `expected` is: valid, the same face count and
/// surface kinds, and the same volume measured the same way.
fn assert_restores(topo: &Topology, healed: SolidId, expected: SolidId) {
    let report = validate_solid(topo, healed).unwrap();
    assert!(
        report.is_valid(),
        "healed solid must validate: {:?}",
        report
            .issues
            .iter()
            .map(|issue| issue.description.clone())
            .collect::<Vec<_>>()
    );
    assert!(cones(topo, healed).is_empty(), "the chamfer band is gone");
    let kinds = |solid: SolidId| {
        let mut kinds: Vec<&'static str> = solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .map(|face| topo.face(face).unwrap().surface().type_tag())
            .collect();
        kinds.sort_unstable();
        kinds
    };
    assert_eq!(kinds(healed), kinds(expected));
    let healed_volume = solid_volume(topo, healed, 0.01).unwrap();
    let expected_volume = solid_volume(topo, expected, 0.01).unwrap();
    assert!(
        (healed_volume - expected_volume).abs() <= expected_volume * 1e-9,
        "healed volume {healed_volume} differs from the unchamfered {expected_volume}"
    );
}

/// The band is deleted and every other face is modified into exactly one
/// result face, with nothing generated or unresolved.
fn assert_history(topo: &mut Topology, chamfered: SolidId, band: FaceId) {
    let sources: BTreeSet<usize> = solid_faces(topo, chamfered)
        .unwrap()
        .iter()
        .map(|face| face.index())
        .collect();
    let (result, history) = defeature_with_evolution(topo, chamfered, &[band]).unwrap();
    assert!(history.origin.is_exact() && history.is_complete());
    assert!(history.generated.is_empty());
    assert_eq!(
        history.deleted.iter().copied().collect::<BTreeSet<_>>(),
        BTreeSet::from([band.index()])
    );
    let mut kept = sources;
    kept.remove(&band.index());
    assert_eq!(
        history.modified.keys().copied().collect::<BTreeSet<_>>(),
        kept
    );
    assert!(history.modified.values().all(|outputs| outputs.len() == 1));
    assert_eq!(
        history
            .modified
            .values()
            .flatten()
            .copied()
            .collect::<BTreeSet<_>>(),
        solid_faces(topo, result)
            .unwrap()
            .iter()
            .map(|face| face.index())
            .collect()
    );
}

fn holed_plate(topo: &mut Topology) -> SolidId {
    let plate = make_box(topo, 30.0, 30.0, 10.0).unwrap();
    let bore = make_cylinder(topo, 4.0, 30.0).unwrap();
    translate(topo, bore, 15.0, 15.0, -10.0);
    boolean(topo, BooleanOp::Cut, plate, bore).unwrap()
}

fn bossed_plate(topo: &mut Topology) -> SolidId {
    let plate = make_box(topo, 30.0, 30.0, 10.0).unwrap();
    let boss = make_cylinder(topo, 5.0, 10.0).unwrap();
    translate(topo, boss, 15.0, 15.0, 10.0);
    boolean(topo, BooleanOp::Fuse, plate, boss).unwrap()
}

#[test]
fn chamfered_hole_rim_heals_to_the_plain_bore() {
    let mut topo = Topology::new();
    let plain = holed_plate(&mut topo);
    let holed = holed_plate(&mut topo);
    let edge = rim(&topo, holed, 4.0, 10.0);
    let chamfered = chamfer_v2(&mut topo, holed, &[edge], 1.0, 1.0)
        .unwrap()
        .solid;
    let band = cones(&topo, chamfered);
    assert_eq!(band.len(), 1, "one conical chamfer band");

    let healed = defeature(&mut topo, chamfered, &band).unwrap();
    assert_restores(&topo, healed, plain);
    assert_history(&mut topo, chamfered, band[0]);
    // The input survives the heal untouched.
    assert_eq!(cones(&topo, chamfered), band);
    assert!(validate_solid(&topo, chamfered).unwrap().is_valid());
}

#[test]
fn chamfered_boss_rim_heals_to_the_plain_boss() {
    let mut topo = Topology::new();
    let plain = bossed_plate(&mut topo);
    let bossed = bossed_plate(&mut topo);
    let edge = rim(&topo, bossed, 5.0, 20.0);
    let chamfered = chamfer_v2(&mut topo, bossed, &[edge], 1.0, 1.0)
        .unwrap()
        .solid;
    let band = cones(&topo, chamfered);
    assert_eq!(band.len(), 1, "one conical chamfer band");

    let healed = defeature(&mut topo, chamfered, &band).unwrap();
    assert_restores(&topo, healed, plain);
    assert_history(&mut topo, chamfered, band[0]);
}

#[test]
fn a_chamfer_band_selected_with_its_support_keeps_the_old_logic() {
    // Only the band alone is a chamfer-band removal. With the bore wall in the
    // selection too the request is a different one, and the established
    // defeature logic answers it (exactly, or with a typed refusal) without
    // this path guessing at it.
    let mut topo = Topology::new();
    let holed = holed_plate(&mut topo);
    let edge = rim(&topo, holed, 4.0, 10.0);
    let chamfered = chamfer_v2(&mut topo, holed, &[edge], 1.0, 1.0)
        .unwrap()
        .solid;
    let band = cones(&topo, chamfered)[0];
    let wall = solid_faces(&topo, chamfered)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    match defeature(&mut topo, chamfered, &[band, wall]) {
        Ok(healed) => {
            assert!(validate_solid(&topo, healed).unwrap().is_valid());
            assert!(cones(&topo, healed).is_empty());
        }
        Err(error) => assert!(!error.to_string().is_empty(), "a refusal names its reason"),
    }
    assert!(validate_solid(&topo, chamfered).unwrap().is_valid());
}
