//! Filling a through-hole is `body ∪ cylinder(bore)`, and it must stay exact.
//!
//! The plug's wall is coincident with the bore wall over its whole area, so the
//! fuse collapses a handle: the result has lower genus than the target. GFA used
//! to decline this on plate-like bodies and fall back to a co-refined mesh —
//! ~100–180 planar faces and ~1e-4 relative volume error, small enough that a
//! volume gate would not catch it and large enough to be wrong. Measured on a
//! 30x30x10 plate it degenerated at bore radii 2, 3, 5, 6 and 8 and survived
//! only at 4.
//!
//! The cause was the interior sample taken for the plug's cap faces. A disc
//! bounded by a single circular edge has one boundary vertex, so the sampler's
//! polygon degenerated and it fell back to a point one inward offset from the
//! rim — sitting on the bore wall the plug is coincident with, where the ray
//! cast that classifies the face is a coin flip. The two caps then classified
//! differently, one was dropped, and the open shell sent the fuse to mesh.
//!
//! These tests pin the exact result across the whole radius matrix. They fail
//! loudly if the mesh fallback is ever reached again: it triples the face count
//! and replaces every analytic surface with planar facets.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// Plate dimensions used across the matrix.
const PLATE: (f64, f64, f64) = (30.0, 30.0, 10.0);

struct PlugCase {
    topo: Topology,
    /// The plate with the bore still open.
    holed: SolidId,
    /// Faces of `holed`, the count the adapter's own gate compares against.
    faces_before: usize,
    /// The bore filled by fusing a plug of the same radius.
    result: SolidId,
}

/// Bore a through-hole of `radius` through the plate, then fill it by fusing a
/// plug cylinder of exactly the same radius spanning the plate's thickness.
fn fill_through_hole(radius: f64) -> PlugCase {
    let (dx, dy, dz) = PLATE;
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, dx, dy, dz).unwrap();

    // The bore overhangs the plate on both sides so the cut is a clean
    // through-hole rather than a coincident-cap configuration.
    let bore = make_cylinder(&mut topo, radius, dz * 3.0).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        bore,
        &Mat4::translation(dx / 2.0, dy / 2.0, -dz),
    )
    .unwrap();
    let holed = boolean(&mut topo, BooleanOp::Cut, plate, bore).unwrap();
    let faces_before = remus_topology::explorer::solid_faces(&topo, holed)
        .unwrap()
        .len();

    let plug = make_cylinder(&mut topo, radius, dz).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        plug,
        &Mat4::translation(dx / 2.0, dy / 2.0, 0.0),
    )
    .unwrap();
    let filled = boolean(&mut topo, BooleanOp::Fuse, holed, plug).unwrap();

    PlugCase {
        topo,
        holed,
        faces_before,
        result: filled,
    }
}

/// Every bore radius that used to degenerate, plus the one that did not.
const RADII: [f64; 6] = [2.0, 3.0, 4.0, 5.0, 6.0, 8.0];

#[test]
fn filling_the_bore_restores_the_plain_plate_at_every_radius() {
    let (dx, dy, dz) = PLATE;
    let solid_volume = dx * dy * dz;

    for radius in RADII {
        let case = fill_through_hole(radius);
        let PlugCase {
            topo,
            result: filled,
            faces_before,
            ..
        } = &case;

        let report = validate_solid(topo, *filled).unwrap();
        assert!(
            report.is_valid(),
            "r={radius}: filled plate must validate, got {:?}",
            report
                .issues
                .iter()
                .map(|i| i.description.clone())
                .collect::<Vec<_>>()
        );

        // Exact, not merely close. The mesh fallback's ~1e-4 relative error
        // would sail through a loose gate; a bound at float noise cannot.
        let volume = remus_operations::measure::solid_volume(topo, *filled, 0.02).unwrap();
        let error = (volume - solid_volume).abs() / solid_volume;
        assert!(
            error < 1e-12,
            "r={radius}: volume {volume} is not exactly {solid_volume} (relative {error:.3e})"
        );

        // An exact fuse yields the plate's six planes plus the plug's two caps
        // (the caps are coplanar with the plate faces they fill, and merging
        // them is `unify_faces`' job — see the next test). The mesh fallback
        // returned 93-177 faces here against this 7-face input, so a bound
        // just above the exact answer separates the two outcomes decisively.
        let faces = remus_topology::explorer::solid_faces(topo, *filled)
            .unwrap()
            .len();
        assert!(
            faces <= 8,
            "r={radius}: filling the bore left {faces} faces (input had \
             {faces_before}) — the mesh fallback was reached"
        );

        // Nothing curved survives: the bore wall is gone and no facetted
        // stand-in replaced it.
        for face in remus_topology::explorer::solid_faces(topo, *filled).unwrap() {
            assert!(
                topo.face(face).unwrap().surface().is_planar(),
                "r={radius}: filled plate should be all planar, found {}",
                topo.face(face).unwrap().surface().type_tag()
            );
        }
    }
}

#[test]
fn the_filled_plate_unifies_back_to_six_faces() {
    // This is what the adapter does after the fuse: `unifyFaces` merges each
    // plug cap into the plate face whose rim it fills. A plain box is the only
    // correct answer, and it is only reachable from an exact fuse — the mesh
    // fallback's facets never unify back down.
    for radius in RADII {
        let mut case = fill_through_hole(radius);
        for _ in 0..3 {
            if remus_operations::heal::unify_faces(&mut case.topo, case.result).unwrap() == 0 {
                break;
            }
        }
        let faces = remus_topology::explorer::solid_faces(&case.topo, case.result)
            .unwrap()
            .len();
        assert_eq!(faces, 6, "r={radius}: filled plate should unify to a box");
        // The adapter's own degenerate-fill gate: filling a hole must leave
        // fewer faces than it started with.
        assert!(faces < case.faces_before);
        assert!(validate_solid(&case.topo, case.result).unwrap().is_valid());
    }
}

#[test]
fn filling_the_bore_leaves_the_holed_plate_intact() {
    let radius = 5.0;
    let case = fill_through_hole(radius);
    let report = validate_solid(&case.topo, case.holed).unwrap();
    assert!(report.is_valid(), "the input must survive the fuse");
    assert_eq!(
        remus_topology::explorer::solid_faces(&case.topo, case.holed)
            .unwrap()
            .len(),
        case.faces_before
    );
}

#[test]
fn an_off_centre_bore_fills_exactly_too() {
    // The centred bore is symmetric about both plate axes, which could mask a
    // fix that only works when the plug sits on the ray-cast axes.
    let (dx, dy, dz) = PLATE;
    let radius = 3.5;
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, dx, dy, dz).unwrap();
    let bore = make_cylinder(&mut topo, radius, dz * 3.0).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        bore,
        &Mat4::translation(9.0, 21.0, -dz),
    )
    .unwrap();
    let holed = boolean(&mut topo, BooleanOp::Cut, plate, bore).unwrap();

    let plug = make_cylinder(&mut topo, radius, dz).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        plug,
        &Mat4::translation(9.0, 21.0, 0.0),
    )
    .unwrap();
    let filled = boolean(&mut topo, BooleanOp::Fuse, holed, plug).unwrap();

    assert!(validate_solid(&topo, filled).unwrap().is_valid());
    let volume = remus_operations::measure::solid_volume(&topo, filled, 0.02).unwrap();
    let expected = dx * dy * dz;
    assert!(
        (volume - expected).abs() / expected < 1e-12,
        "off-centre fill volume {volume} is not exactly {expected}"
    );
    for face in remus_topology::explorer::solid_faces(&topo, filled).unwrap() {
        assert!(topo.face(face).unwrap().surface().is_planar());
    }
}
