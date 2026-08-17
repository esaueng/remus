//! Provenance an operation can state exactly must not be guessed at.
//!
//! A blend engine that trims its base faces knows which output face carries
//! each input face and which two base faces every blend band was built between.
//! A pattern instance is a copy, so its faces correspond to the source body's
//! faces by construction — and geometric matching could not tell one congruent
//! instance from another even in principle.
//!
//! Each assertion below is a fact about the construction, not a recording of
//! what the code happened to output.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashSet;

use remus_math::vec::Vec3;
use remus_operations::blend_ops;
use remus_operations::evolution::{EvolutionMap, EvolutionOrigin};
use remus_operations::pattern;
use remus_operations::primitives;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

const R: f64 = 45.0;
const H: f64 = 10.0;

/// Distinct edges of a cylinder in discovery order: `[bottom rim, seam, top rim]`.
fn cylinder_edges(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    let mut seen = Vec::new();
    for fid in solid_faces(topo, s).unwrap() {
        let f = topo.face(fid).unwrap();
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                if !seen.contains(&oe.edge()) {
                    seen.push(oe.edge());
                }
            }
        }
    }
    seen
}

fn face_indices(topo: &Topology, s: SolidId) -> HashSet<usize> {
    solid_faces(topo, s)
        .unwrap()
        .into_iter()
        .map(remus_topology::arena::Id::index)
        .collect()
}

/// Every face of the result must appear somewhere in the map. Silence about a
/// face is indistinguishable from "not in the result", which is the failure
/// mode this whole exercise exists to remove.
fn assert_accounts_for_every_result_face(evo: &EvolutionMap, result_faces: &HashSet<usize>) {
    let mut named: HashSet<usize> = HashSet::new();
    for outs in evo.modified.values().chain(evo.generated.values()) {
        named.extend(outs.iter().copied());
    }
    named.extend(evo.unresolved.keys().copied());
    let missing: Vec<usize> = result_faces.difference(&named).copied().collect();
    assert!(
        missing.is_empty(),
        "result faces absent from the evolution map: {missing:?}"
    );
}

/// A cylinder rim fillet goes to the walking builder's annular assembler, which
/// keeps a real record. The band is between the cap and the wall — the two
/// faces the rim edge separates — and that is not a matter of opinion.
#[test]
fn cylinder_rim_fillet_reports_construction_derived_provenance() {
    let mut topo = Topology::new();
    let cyl = primitives::make_cylinder(&mut topo, R, H).unwrap();
    let edges = cylinder_edges(&topo, cyl);
    let top = edges[2];

    let before: HashSet<usize> = face_indices(&topo, cyl);
    let (result, evo) =
        blend_ops::fillet_with_evolution(&mut topo, cyl, &[top], 2.0).expect("cylinder rim fillet");

    assert_eq!(
        evo.origin,
        EvolutionOrigin::Construction,
        "the annular assembler records its own provenance; nothing here should be inferred"
    );

    let after = face_indices(&topo, result.solid);
    assert_accounts_for_every_result_face(&evo, &after);

    // The input had three faces; every one of them must still be named.
    for fid in &before {
        assert!(
            evo.modified.contains_key(fid),
            "input face {fid} vanished from the map"
        );
    }

    // Exactly one band was created, from exactly two base faces, and both of
    // those are input faces of the cylinder.
    let created: Vec<usize> = evo
        .generated
        .values()
        .flatten()
        .copied()
        .collect::<HashSet<_>>()
        .into_iter()
        .collect();
    assert_eq!(created.len(), 1, "one rim, one band: {created:?}");
    let band = created[0];
    let mut sources: Vec<usize> = evo
        .generated
        .iter()
        .filter(|(_, outs)| outs.contains(&band))
        .map(|(src, _)| *src)
        .collect();
    sources.sort_unstable();
    assert_eq!(
        sources.len(),
        2,
        "a blend band is built between two base faces, got {sources:?}"
    );
    for src in &sources {
        assert!(
            before.contains(src),
            "band attributed to {src}, which is not an input face"
        );
    }
}

/// The same for a chamfer: the cone band descends from the cap and the wall.
#[test]
fn cylinder_rim_chamfer_reports_construction_derived_provenance() {
    let mut topo = Topology::new();
    let cyl = primitives::make_cylinder(&mut topo, R, H).unwrap();
    let edges = cylinder_edges(&topo, cyl);
    let top = edges[2];

    let before: HashSet<usize> = face_indices(&topo, cyl);
    let (result, evo) = blend_ops::chamfer_with_evolution(&mut topo, cyl, &[top], 2.0, 2.0)
        .expect("cylinder rim chamfer");

    assert_eq!(evo.origin, EvolutionOrigin::Construction);
    assert_accounts_for_every_result_face(&evo, &face_indices(&topo, result.solid));
    for fid in &before {
        assert!(
            evo.modified.contains_key(fid),
            "input face {fid} vanished from the map"
        );
    }
    let created: HashSet<usize> = evo.generated.values().flatten().copied().collect();
    assert_eq!(created.len(), 1, "one rim, one band: {created:?}");
}

/// Every face of every pattern instance descends from the source face it was
/// copied from. The check is geometric on purpose — it verifies the recorded
/// correspondence against the actual translation, rather than against itself.
#[test]
fn pattern_instance_faces_descend_from_the_source_face_they_copy() {
    let mut topo = Topology::new();
    let src = primitives::make_box(&mut topo, 10.0, 4.0, 4.0).unwrap();
    let source_faces: Vec<remus_topology::face::FaceId> = solid_faces(&topo, src).unwrap();

    let spacing = 25.0;
    let dir = Vec3::new(1.0, 0.0, 0.0);
    let (compound, evo) =
        pattern::linear_pattern_with_evolution(&mut topo, src, dir, spacing, 3).unwrap();

    assert_eq!(evo.origin, EvolutionOrigin::Construction);
    assert!(evo.is_complete(), "a copy leaves nothing to resolve");
    assert!(evo.deleted.is_empty(), "a pattern deletes nothing");

    // Index -> FaceId for every face in the pattern, so the recorded indices
    // can be resolved back to real geometry.
    let mut by_index: std::collections::HashMap<usize, remus_topology::face::FaceId> =
        std::collections::HashMap::new();
    for &inst in topo.compound(compound).unwrap().solids() {
        for fid in solid_faces(&topo, inst).unwrap() {
            by_index.insert(fid.index(), fid);
        }
    }

    // The source body is instance 0 and is reused as-is.
    for f in &source_faces {
        assert_eq!(
            evo.modified.get(&f.index()),
            Some(&vec![f.index()]),
            "source face {} is its own first instance",
            f.index()
        );
    }

    for f in &source_faces {
        let instances = evo
            .generated
            .get(&f.index())
            .unwrap_or_else(|| panic!("source face {} generated no instance faces", f.index()));
        assert_eq!(
            instances.len(),
            2,
            "count 3 means the source plus two copies"
        );

        // The recorded instance faces must actually BE the source face moved by
        // one and two spacings — checked against the geometry, not the record.
        let (src_normal, src_d) = plane_of(&topo, *f);
        let mut offsets: Vec<i64> = Vec::new();
        for &g in instances {
            let gid = by_index[&g];
            let (n, d) = plane_of(&topo, gid);
            assert!(
                n.dot(src_normal) > 0.999,
                "instance face {g} is not parallel to its source {}",
                f.index()
            );
            #[allow(clippy::cast_possible_truncation)]
            offsets.push(((d - src_d) / (spacing * src_normal.dot(dir))).round() as i64);
        }
        offsets.sort_unstable();
        // A face whose normal runs along the pattern direction sits one and two
        // spacings further out; one perpendicular to it does not move at all,
        // and its plane offset is unchanged.
        let along = src_normal.dot(dir).abs() > 0.5;
        if along {
            assert_eq!(offsets, vec![1, 2], "source face {}", f.index());
        } else {
            assert!(
                instances.iter().all(|&g| {
                    let (_, d) = plane_of(&topo, by_index[&g]);
                    (d - src_d).abs() < 1e-9
                }),
                "source face {} is perpendicular to the pattern and must not move",
                f.index()
            );
        }
    }
}

/// A planar face's plane as `(normal, signed distance from the origin)`.
fn plane_of(topo: &Topology, face: remus_topology::face::FaceId) -> (Vec3, f64) {
    match topo.face(face).unwrap().surface() {
        remus_topology::face::FaceSurface::Plane { normal, d } => (*normal, *d),
        other => panic!("expected a planar box face, got {}", other.type_tag()),
    }
}

/// A box-edge fillet takes the planar rolling-ball rebuild. Its face-spec
/// history is carried through assembly and same-surface unification, so the map
/// is construction-derived just like the walking-builder path.
///
/// The blend face is the interesting one, and it has had two wrong answers.
///
/// It did not exist in the input, and its sampled normal sits between the two
/// faces the rounded edge separated — equidistant from both, on neither's
/// plane. The original near-tie rule read that tie as agreement and recorded
/// the blend as a **modified** version of the top face and of the side face at
/// once, so a selection stored against either silently acquired a face the user
/// never picked. That was right to remove.
///
/// What replaced it read the tie as ambiguity and refused the face outright.
/// But a tie between two faces the output is parallel to *neither* of is not an
/// ambiguity: an output at 45° to both cannot be either of them re-trimmed,
/// which leaves only one thing it can be — new geometry built between them.
/// That is `generated`, and it is what the walking builder's own construction
/// record says for the same band on a cylinder rim. Two engines behind one
/// operation are not entitled to different answers about what a blend face
/// descends from.
///
/// The property that made the first answer dangerous is still enforced below:
/// the blend appears in no `modified` entry, so no stored selection moves.
#[test]
fn a_rebuilt_fillet_attributes_its_blend_face_to_both_base_faces() {
    let mut topo = Topology::new();
    let cube = primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = remus_topology::explorer::solid_edges(&topo, cube).unwrap();

    let before = face_indices(&topo, cube);
    let (result, evo) = blend_ops::fillet_with_evolution(&mut topo, cube, &[edges[0]], 1.0)
        .expect("box edge fillet");

    assert_eq!(
        evo.origin,
        EvolutionOrigin::Construction,
        "the rolling-ball builder must carry its assembly history through unification"
    );

    let after = face_indices(&topo, result.solid);
    assert_eq!(
        after.len(),
        before.len() + 1,
        "one rounded edge adds exactly one blend face"
    );
    assert_accounts_for_every_result_face(&evo, &after);

    // Each of the six original faces survives into exactly one output face,
    // and no two of them claim the same one.
    let mut claimed: Vec<usize> = Vec::new();
    for fid in &before {
        let outs = evo
            .modified
            .get(fid)
            .unwrap_or_else(|| panic!("input face {fid} lost"));
        assert_eq!(outs.len(), 1, "input face {fid} -> {outs:?}");
        claimed.push(outs[0]);
    }
    claimed.sort_unstable();
    claimed.dedup();
    assert_eq!(claimed.len(), 6, "six faces in, six distinct faces out");

    // The seventh face is the blend, and it is generated from exactly the two
    // faces the rounded edge ran between.
    let generated: HashSet<usize> = evo.generated.values().flatten().copied().collect();
    assert_eq!(
        generated.len(),
        1,
        "one rounded edge, one blend face: {:?}",
        evo.generated
    );
    let blend = *generated.iter().next().unwrap();
    assert!(
        !before.contains(&blend) && after.contains(&blend),
        "the blend is a face of the result that was not a face of the input"
    );

    let mut sources: Vec<usize> = evo
        .generated
        .iter()
        .filter(|(_, outs)| outs.contains(&blend))
        .map(|(src, _)| *src)
        .collect();
    sources.sort_unstable();
    assert_eq!(
        sources.len(),
        2,
        "a band is built between the two faces its edge separated, got {sources:?}"
    );
    for src in &sources {
        assert!(
            before.contains(src),
            "band attributed to {src}, which is not an input face"
        );
    }

    // The hazard the old near-tie rule created: the blend must never be handed
    // out as a surviving version of a face the user could have selected.
    assert!(
        !claimed.contains(&blend),
        "the blend face must not also be claimed as a surviving input face"
    );
    for (src, outs) in &evo.modified {
        assert!(
            !outs.contains(&blend),
            "input face {src} claims the blend as a modified version of itself"
        );
    }

    assert!(
        evo.deleted.is_empty(),
        "rounding an edge removes no face: {:?}",
        evo.deleted
    );
    assert!(
        evo.is_complete(),
        "every face of the result is placed: {:?}",
        evo.unresolved
    );
}
