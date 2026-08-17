//! Face provenance must not depend on the unit a body is modelled in.
//!
//! The same body at 1x, 1000x and 0.001x is the same body: the map from input
//! faces to output faces is identical up to the scale factor. A provenance
//! matcher with an absolute distance budget instead answers differently at
//! every scale — and a wrong answer moves a user's saved face selection onto a
//! different face rather than dropping it.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::Point3;
use remus_operations::boolean::{self, BooleanOp, collect_face_signatures};
use remus_operations::evolution::{EvolutionMap, build_evolution_by_geometry};
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

/// Two overlapping boxes fused, everything scaled by `s`.
///
/// A fresh arena each time, so face indices are identical across scales and
/// the maps can be compared entry for entry.
fn fused_pair_evolution(s: f64) -> EvolutionMap {
    let mut topo = Topology::new();
    let a = make_box(&mut topo, 10.0 * s, 10.0 * s, 10.0 * s).unwrap();
    let b = make_box(&mut topo, 10.0 * s, 10.0 * s, 10.0 * s).unwrap();
    let shift = remus_math::mat::Mat4::translation(6.0 * s, 0.0, 0.0);
    transform_solid(&mut topo, b, &shift).unwrap();

    let mut inputs = collect_face_signatures(&topo, a).unwrap();
    inputs.extend(collect_face_signatures(&topo, b).unwrap());

    let result = boolean::boolean(&mut topo, BooleanOp::Fuse, a, b).unwrap();
    let outputs = collect_face_signatures(&topo, result).unwrap();

    build_evolution_by_geometry(&inputs, &outputs)
}

fn normalized(evo: &EvolutionMap) -> String {
    let mut modified: Vec<(usize, Vec<usize>)> = evo
        .modified
        .iter()
        .map(|(k, v)| {
            let mut v = v.clone();
            v.sort_unstable();
            (*k, v)
        })
        .collect();
    modified.sort();
    let mut generated: Vec<(usize, Vec<usize>)> = evo
        .generated
        .iter()
        .map(|(k, v)| {
            let mut v = v.clone();
            v.sort_unstable();
            (*k, v)
        })
        .collect();
    generated.sort();
    let mut deleted: Vec<usize> = evo.deleted.iter().copied().collect();
    deleted.sort_unstable();
    format!("modified={modified:?} generated={generated:?} deleted={deleted:?}")
}

#[test]
fn fuse_provenance_is_the_same_at_every_scale() {
    let at_1 = fused_pair_evolution(1.0);
    let at_1000 = fused_pair_evolution(1000.0);
    let at_milli = fused_pair_evolution(0.001);

    assert_eq!(
        normalized(&at_1),
        normalized(&at_1000),
        "1x vs 1000x\n  1x:    {}\n  1000x: {}",
        normalized(&at_1),
        normalized(&at_1000)
    );
    assert_eq!(
        normalized(&at_1),
        normalized(&at_milli),
        "1x vs 0.001x\n  1x:     {}\n  0.001x: {}",
        normalized(&at_1),
        normalized(&at_milli)
    );
}

/// Which output face a fused pair's input face must map to is not a matter of
/// opinion, so this asserts the whole map against the construction rather than
/// against recorded indices.
///
/// Two 10-cubes, the second offset 6 along +X. The union is a 16x10x10 block.
/// Each body's four side walls are coplanar with the other's and merge pairwise
/// into one wall of the block; the two outer end caps survive alone; and the
/// two walls facing each other across the overlap are interior to the union and
/// are gone.
#[test]
fn fuse_of_two_boxes_maps_what_the_construction_says() {
    use remus_math::vec::Vec3;
    use remus_topology::explorer::solid_faces;
    use remus_topology::face::{FaceId, FaceSurface};

    let mut topo = Topology::new();
    let a = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let b = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    transform_solid(
        &mut topo,
        b,
        &remus_math::mat::Mat4::translation(6.0, 0.0, 0.0),
    )
    .unwrap();

    let mut inputs = collect_face_signatures(&topo, a).unwrap();
    inputs.extend(collect_face_signatures(&topo, b).unwrap());
    let result = boolean::boolean(&mut topo, BooleanOp::Fuse, a, b).unwrap();
    let evo =
        build_evolution_by_geometry(&inputs, &collect_face_signatures(&topo, result).unwrap());

    // Name each face by the plane it lies in, so the assertions can be written
    // in geometry rather than in arena indices.
    let plane_of = |topo: &Topology, f: FaceId| -> (Vec3, f64) {
        match topo.face(f).unwrap().surface() {
            FaceSurface::Plane { normal, d } => (*normal, *d),
            other => panic!("box faces are planar, got {}", other.type_tag()),
        }
    };
    let find = |topo: &Topology, solid, want_n: Vec3, want_d: f64| -> usize {
        solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .find(|&f| {
                let (n, d) = plane_of(topo, f);
                n.dot(want_n) > 0.999 && (d - want_d).abs() < 1e-9
            })
            .unwrap_or_else(|| panic!("no face with normal {want_n:?} at d={want_d}"))
            .index()
    };

    let px = Vec3::new(1.0, 0.0, 0.0);
    let py = Vec3::new(0.0, 1.0, 0.0);
    let pz = Vec3::new(0.0, 0.0, 1.0);

    // The four walls parallel to X: each pair merges into one output wall.
    for (n, d) in [(py, 10.0), (-py, 0.0), (pz, 10.0), (-pz, 0.0)] {
        let from_a = find(&topo, a, n, d);
        let from_b = find(&topo, b, n, d);
        let out = find(&topo, result, n, d);
        assert_eq!(
            evo.modified.get(&from_a),
            Some(&vec![out]),
            "A's wall at {n:?} d={d} merges into the block's"
        );
        assert_eq!(
            evo.modified.get(&from_b),
            Some(&vec![out]),
            "B's wall at {n:?} d={d} merges into the same one"
        );
    }

    // The two outer end caps survive alone.
    let a_minus_x = find(&topo, a, -px, 0.0);
    let b_plus_x = find(&topo, b, px, 16.0);
    assert_eq!(
        evo.modified.get(&a_minus_x),
        Some(&vec![find(&topo, result, -px, 0.0)])
    );
    assert_eq!(
        evo.modified.get(&b_plus_x),
        Some(&vec![find(&topo, result, px, 16.0)])
    );

    // The two walls inside the union are gone, and must be reported gone —
    // this is the pair the absolute distance budget silently rebound to the
    // block's outer end caps at 0.001x.
    let a_plus_x = find(&topo, a, px, 10.0);
    let b_minus_x = find(&topo, b, -px, -6.0);
    assert!(
        evo.deleted.contains(&a_plus_x),
        "A's +X wall is interior to the union: {:?}",
        evo.deleted
    );
    assert!(
        evo.deleted.contains(&b_minus_x),
        "B's -X wall is interior to the union: {:?}",
        evo.deleted
    );
    assert!(
        !evo.modified.contains_key(&a_plus_x) && !evo.modified.contains_key(&b_minus_x),
        "a consumed wall must not also be reported as surviving"
    );

    assert!(
        evo.generated.is_empty(),
        "a fuse of two boxes invents no face"
    );
    assert!(evo.is_complete(), "unresolved: {:?}", evo.unresolved);
}

/// A synthetic pair of unit cubes' worth of face signatures, scaled.
///
/// Removes the boolean engine from the picture entirely: only the matcher is
/// under test, and the correct answer is a 1:1 correspondence.
fn box_signatures(s: f64, base: usize) -> Vec<(usize, remus_math::vec::Vec3, Point3)> {
    use remus_math::vec::Vec3;
    let h = 5.0 * s;
    vec![
        (base, Vec3::new(0.0, 0.0, -1.0), Point3::new(h, h, 0.0)),
        (
            base + 1,
            Vec3::new(0.0, 0.0, 1.0),
            Point3::new(h, h, 10.0 * s),
        ),
        (base + 2, Vec3::new(0.0, -1.0, 0.0), Point3::new(h, 0.0, h)),
        (
            base + 3,
            Vec3::new(0.0, 1.0, 0.0),
            Point3::new(h, 10.0 * s, h),
        ),
        (base + 4, Vec3::new(-1.0, 0.0, 0.0), Point3::new(0.0, h, h)),
        (
            base + 5,
            Vec3::new(1.0, 0.0, 0.0),
            Point3::new(10.0 * s, h, h),
        ),
    ]
}

#[test]
fn matcher_answer_does_not_move_with_scale() {
    for &s in &[1.0_f64, 1000.0, 0.001] {
        let inputs = box_signatures(s, 0);
        let outputs = box_signatures(s, 100);
        let evo = build_evolution_by_geometry(&inputs, &outputs);
        for i in 0..6 {
            assert_eq!(
                evo.modified.get(&i),
                Some(&vec![100 + i]),
                "scale {s}: face {i} must map to exactly one output, got {:?}",
                evo.modified.get(&i)
            );
        }
        assert!(evo.deleted.is_empty(), "scale {s}: nothing was deleted");
        assert!(evo.generated.is_empty(), "scale {s}: nothing was generated");
    }
}
