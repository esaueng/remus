//! Every face of a result must be claimed by exactly one lineage record.
//!
//! An [`EvolutionMap`] is the only thing standing between a stored face
//! selection and the geometry it was stored against. A result face no record
//! mentions is indistinguishable, to a consumer, from a face that is not in the
//! result at all — so the consumer either drops a live reference or keeps a dead
//! one, and nothing in the kernel reports a problem either way.
//!
//! The invariant these tests enforce, for every operation that produces a
//! history:
//!
//! ```text
//! modified-results ∪ generated-results  ==  faces(result)
//! modified-sources ∪ deleted            ==  faces(sources)
//! ```
//!
//! # Why this is a set equality and not a count
//!
//! The defect that prompted this file changed no face count. A box-edge fillet
//! produced seven faces before and after; all that moved was which record
//! claimed the cylindrical band. A count-based assertion — "six planes in,
//! seven faces out, one more than we started with" — passes on a map that
//! attributes the band to nothing at all. Only comparing the *sets* sees it.
//!
//! # Why `unresolved` does not count as a claim
//!
//! [`EvolutionMap::unresolved`] is an admission that the operation could not
//! establish an origin, and a consumer must fail closed on it. It is a
//! legitimate answer where an origin genuinely cannot be established, and these
//! tests assert it is empty because for these operations it *can* be: a blend
//! band's base faces and a boolean's face splits are facts about the
//! construction, not judgement calls. An `unresolved` entry appearing here means
//! a fact was lost, not that an ambiguity was found.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashSet;

use remus_math::mat::Mat4;
use remus_operations::evolution::EvolutionMap;
use remus_operations::{blend_ops, boolean, primitives, transform};
use remus_topology::Topology;
use remus_topology::arena::Id;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::solid::SolidId;

/// The modelling units the same body is built in. 1000x and 0.001x are the axis
/// the centroid budget was made dimensionless for; a rule that reads a face's
/// position against an absolute constant answers differently at each of them.
const SCALES: [f64; 3] = [1.0, 1000.0, 0.001];

fn faces_of(topo: &Topology, solid: SolidId) -> HashSet<usize> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(Id::index)
        .collect()
}

/// Enforce the invariant in the module docs, in both directions.
fn assert_lineage_accounts_for_everything(
    label: &str,
    evo: &EvolutionMap,
    sources: &HashSet<usize>,
    result: &HashSet<usize>,
) {
    // ── Result side ────────────────────────────────────────────────
    let mut claimed: HashSet<usize> = HashSet::new();
    for outs in evo.modified.values().chain(evo.generated.values()) {
        claimed.extend(outs.iter().copied());
    }

    let unclaimed: Vec<usize> = sorted(result.difference(&claimed));
    assert!(
        unclaimed.is_empty(),
        "{label}: result faces {unclaimed:?} are claimed by no lineage record. \
         A consumer cannot tell them from faces that are not in the result. \
         map = {}",
        evo.to_json()
    );

    let phantom: Vec<usize> = sorted(claimed.difference(result));
    assert!(
        phantom.is_empty(),
        "{label}: the map claims faces {phantom:?} that are not in the result. \
         map = {}",
        evo.to_json()
    );

    assert!(
        evo.unresolved.is_empty(),
        "{label}: the map refuses to place {:?}. These origins are construction \
         facts, so a refusal here is a lost fact rather than a found ambiguity. \
         map = {}",
        evo.unresolved.keys().collect::<Vec<_>>(),
        evo.to_json()
    );

    // ── Source side ────────────────────────────────────────────────
    let mut accounted: HashSet<usize> = evo.modified.keys().copied().collect();
    accounted.extend(evo.deleted.iter().copied());

    let unaccounted: Vec<usize> = sorted(sources.difference(&accounted));
    assert!(
        unaccounted.is_empty(),
        "{label}: input faces {unaccounted:?} are neither carried into the result \
         nor reported deleted. map = {}",
        evo.to_json()
    );

    let invented: Vec<usize> = sorted(accounted.difference(sources));
    assert!(
        invented.is_empty(),
        "{label}: the map names {invented:?} as input faces of an operation that \
         never had them. map = {}",
        evo.to_json()
    );

    // A face a new face was generated from has to be a real input face. It is
    // not required to survive — a boolean can consume the face its new geometry
    // grew out of — so this is a subset check, not an equality.
    let bad_sources: Vec<usize> = sorted(
        evo.generated
            .keys()
            .copied()
            .collect::<HashSet<_>>()
            .difference(sources),
    );
    assert!(
        bad_sources.is_empty(),
        "{label}: faces were generated from {bad_sources:?}, which are not inputs. \
         map = {}",
        evo.to_json()
    );
}

fn sorted<'a>(it: impl Iterator<Item = &'a usize>) -> Vec<usize> {
    let mut v: Vec<usize> = it.copied().collect();
    v.sort_unstable();
    v
}

/// The faces a blend band was built between, as recorded by `generated`.
fn generated_sources_of(evo: &EvolutionMap, face: usize) -> Vec<usize> {
    let mut v: Vec<usize> = evo
        .generated
        .iter()
        .filter(|(_, outs)| outs.contains(&face))
        .map(|(src, _)| *src)
        .collect();
    v.sort_unstable();
    v
}

// ── Fillet ─────────────────────────────────────────────────────────

/// One edge of a cube, rounded. Seven faces out: six planes and one cylinder.
///
/// The cylinder is the face this whole file exists for. It is not a re-trimmed
/// piece of either plane the rounded edge separated — it lies on neither, and
/// its normal bisects theirs — so it is new geometry, and the two faces it was
/// built between are what `generated` records. That is the same answer the
/// walking builder's own construction record gives for the equivalent band on a
/// cylinder rim, and the two engines for one operation must not disagree about
/// what a blend face descends from.
#[test]
fn box_edge_fillet_attributes_the_blend_band_to_both_faces_the_edge_separated() {
    for scale in SCALES {
        let mut topo = Topology::new();
        let cube =
            primitives::make_box(&mut topo, 10.0 * scale, 10.0 * scale, 10.0 * scale).expect("box");
        let edges = solid_edges(&topo, cube).unwrap();

        let before = faces_of(&topo, cube);
        assert_eq!(before.len(), 6, "a box has six faces");

        let (result, evo) =
            blend_ops::fillet_with_evolution(&mut topo, cube, &[edges[0]], 1.0 * scale)
                .expect("box edge fillet");
        let after = faces_of(&topo, result.solid);

        let label = format!("box edge fillet at {scale}x");
        assert_lineage_accounts_for_everything(&label, &evo, &before, &after);

        assert!(
            evo.deleted.is_empty(),
            "{label}: rounding an edge removes no face, got {:?}",
            evo.deleted
        );

        // Every input face survives into exactly one output face, and no two of
        // them land on the same one.
        let mut survivors: Vec<usize> = Vec::new();
        for fid in &before {
            let outs = evo
                .modified
                .get(fid)
                .unwrap_or_else(|| panic!("{label}: input face {fid} lost"));
            assert_eq!(outs.len(), 1, "{label}: input face {fid} -> {outs:?}");
            survivors.push(outs[0]);
        }
        survivors.sort_unstable();
        survivors.dedup();
        assert_eq!(survivors.len(), 6, "{label}: six in, six distinct out");

        // The seventh face is the band, and it is generated from exactly the
        // two input faces the rounded edge ran between — not claimed as a
        // modified version of either, which would hand a selection stored
        // against a plane a face the user never picked.
        let bands: Vec<usize> = after
            .iter()
            .copied()
            .filter(|f| !survivors.contains(f))
            .collect();
        assert_eq!(bands.len(), 1, "{label}: one rounded edge, one band");
        let band = bands[0];

        let sources = generated_sources_of(&evo, band);
        assert_eq!(
            sources.len(),
            2,
            "{label}: a band is built between two base faces, got {sources:?}"
        );
        for src in &sources {
            assert!(
                before.contains(src),
                "{label}: band attributed to {src}, not an input face"
            );
            assert!(
                !evo.modified
                    .get(src)
                    .is_some_and(|outs| outs.contains(&band)),
                "{label}: the band must not also be claimed as a survivor of {src}"
            );
        }
    }
}

/// The same map at 1x, 1000x and 0.001x, up to the face indices the arena
/// happens to hand out. A rule that measures a centroid against an absolute
/// constant fails here and nowhere else.
#[test]
fn box_edge_fillet_lineage_is_the_same_shape_at_every_modelling_unit() {
    let shapes: Vec<(usize, usize, usize, usize)> = SCALES
        .iter()
        .map(|&scale| {
            let mut topo = Topology::new();
            let cube =
                primitives::make_box(&mut topo, 10.0 * scale, 10.0 * scale, 10.0 * scale).unwrap();
            let edges = solid_edges(&topo, cube).unwrap();
            let (_, evo) =
                blend_ops::fillet_with_evolution(&mut topo, cube, &[edges[0]], 1.0 * scale)
                    .expect("box edge fillet");
            (
                evo.modified.len(),
                evo.generated.values().flatten().count(),
                evo.deleted.len(),
                evo.unresolved.len(),
            )
        })
        .collect();

    assert!(
        shapes.windows(2).all(|w| w[0] == w[1]),
        "the map changes with the modelling unit: {shapes:?}"
    );
    assert_eq!(
        shapes[0],
        (6, 2, 0, 0),
        "six survivors, one band named by its two base faces, nothing deleted or refused"
    );
}

/// Two edges sharing a face. The shared face is a base face of both bands, so
/// it generates two of them — a many-to-many record a one-source-per-face rule
/// cannot express. One repaired case can hide a still-broken general rule, which
/// is what this covers.
#[test]
fn multi_edge_fillet_lets_one_face_generate_several_bands() {
    let mut topo = Topology::new();
    let cube = primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, cube).unwrap();

    let before = faces_of(&topo, cube);
    let (result, evo) =
        blend_ops::fillet_with_evolution(&mut topo, cube, &[edges[0], edges[2]], 1.0)
            .expect("two-edge fillet");
    let after = faces_of(&topo, result.solid);

    assert_lineage_accounts_for_everything("two-edge box fillet", &evo, &before, &after);

    let bands: HashSet<usize> = evo.generated.values().flatten().copied().collect();
    assert_eq!(bands.len(), 2, "two rounded edges, two bands: {bands:?}");
    for band in &bands {
        assert_eq!(
            generated_sources_of(&evo, *band).len(),
            2,
            "band {band} is built between two base faces"
        );
    }
    assert!(
        evo.generated.values().any(|outs| outs.len() == 2),
        "the face both rounded edges touch generates both bands: {:?}",
        evo.generated
    );
}

/// A fillet whose band is built against a curved face rather than two planes.
/// This one goes to the walking builder, which keeps its own construction
/// record — so it is the reference answer the planar builder's assembly record
/// above is held to, not just another case.
#[test]
fn cylinder_rim_fillet_lineage_is_complete_and_construction_derived() {
    for scale in SCALES {
        let mut topo = Topology::new();
        let cyl = primitives::make_cylinder(&mut topo, 45.0 * scale, 10.0 * scale).unwrap();
        let edges = solid_edges(&topo, cyl).unwrap();
        let rim = *edges.last().expect("a cylinder has edges");

        let before = faces_of(&topo, cyl);
        let (result, evo) = blend_ops::fillet_with_evolution(&mut topo, cyl, &[rim], 2.0 * scale)
            .expect("cylinder rim fillet");
        let after = faces_of(&topo, result.solid);

        let label = format!("cylinder rim fillet at {scale}x");
        assert_lineage_accounts_for_everything(&label, &evo, &before, &after);

        assert!(
            evo.origin.is_exact(),
            "{label}: the annular assembler records its own provenance"
        );

        let bands: HashSet<usize> = evo.generated.values().flatten().copied().collect();
        assert_eq!(bands.len(), 1, "{label}: one rim, one band");
        let band = *bands.iter().next().unwrap();
        assert_eq!(
            generated_sources_of(&evo, band).len(),
            2,
            "{label}: the band descends from the cap and the wall"
        );
    }
}

// ── Chamfer ────────────────────────────────────────────────────────

/// A chamfer's bevel is new geometry between the two faces its edge separated,
/// exactly as a fillet's band is. It reaches the same rule by the same route,
/// so it is checked by the same invariant rather than assumed to follow.
#[test]
fn box_edge_chamfer_attributes_the_bevel_to_both_faces_the_edge_separated() {
    for scale in SCALES {
        let mut topo = Topology::new();
        let cube =
            primitives::make_box(&mut topo, 10.0 * scale, 10.0 * scale, 10.0 * scale).unwrap();
        let edges = solid_edges(&topo, cube).unwrap();

        let before = faces_of(&topo, cube);
        let (result, evo) = blend_ops::chamfer_with_evolution(
            &mut topo,
            cube,
            &[edges[0]],
            1.0 * scale,
            1.0 * scale,
        )
        .expect("box edge chamfer");
        let after = faces_of(&topo, result.solid);

        let label = format!("box edge chamfer at {scale}x");
        assert_lineage_accounts_for_everything(&label, &evo, &before, &after);

        let bevels: HashSet<usize> = evo.generated.values().flatten().copied().collect();
        assert_eq!(bevels.len(), 1, "{label}: one bevelled edge, one bevel");
        let bevel = *bevels.iter().next().unwrap();
        assert_eq!(
            generated_sources_of(&evo, bevel).len(),
            2,
            "{label}: the bevel descends from both faces the edge separated"
        );
    }
}

/// The chamfer equivalent of the cylinder rim fillet: a cone band recorded by
/// the builder that assembled it.
#[test]
fn cylinder_rim_chamfer_lineage_is_complete_and_construction_derived() {
    let mut topo = Topology::new();
    let cyl = primitives::make_cylinder(&mut topo, 45.0, 10.0).unwrap();
    let edges = solid_edges(&topo, cyl).unwrap();
    let rim = *edges.last().unwrap();

    let before = faces_of(&topo, cyl);
    let (result, evo) = blend_ops::chamfer_with_evolution(&mut topo, cyl, &[rim], 2.0, 2.0)
        .expect("cylinder rim chamfer");
    let after = faces_of(&topo, result.solid);

    assert_lineage_accounts_for_everything("cylinder rim chamfer", &evo, &before, &after);
    assert!(evo.origin.is_exact());
}

// ── Booleans ───────────────────────────────────────────────────────

/// Two 10-cubes overlapping by 4 — the configuration the scale defect was found
/// on. Every result face descends from one operand face; faces the operation
/// consumes are reported deleted, and nothing is left silent.
#[test]
fn boolean_lineage_is_complete_at_every_modelling_unit() {
    for (name, op) in [
        ("fuse", boolean::BooleanOp::Fuse),
        ("cut", boolean::BooleanOp::Cut),
        ("intersect", boolean::BooleanOp::Intersect),
    ] {
        for scale in SCALES {
            let mut topo = Topology::new();
            let a =
                primitives::make_box(&mut topo, 10.0 * scale, 10.0 * scale, 10.0 * scale).unwrap();
            let b =
                primitives::make_box(&mut topo, 10.0 * scale, 10.0 * scale, 10.0 * scale).unwrap();
            transform::transform_solid(&mut topo, b, &Mat4::translation(6.0 * scale, 0.0, 0.0))
                .unwrap();

            let mut before = faces_of(&topo, a);
            before.extend(faces_of(&topo, b));

            let (result, evo) = boolean::boolean_with_evolution(&mut topo, op, a, b)
                .unwrap_or_else(|e| panic!("{name} at {scale}x: {e}"));
            let after = faces_of(&topo, result);

            let label = format!("{name} at {scale}x");
            assert_lineage_accounts_for_everything(&label, &evo, &before, &after);

            assert!(
                !evo.deleted.is_empty(),
                "{label}: overlapping operands consume faces; none were reported"
            );
        }
    }
}

// ── The route the WASM binding takes ───────────────────────────────

/// `filletWithEvolution` does not call [`blend_ops::fillet_with_evolution`]. It
/// drives the engine cascade itself and hands whatever record came back to
/// [`blend_ops::evolution_from_blend_origins`], so it could in principle report
/// something different from the in-process API for the same body. It must not,
/// and this pins the two together on the exact reproduction the defect was
/// reported against — a 10-cube with one edge rounded at radius 1.
///
/// [`blend_ops::evolution_from_blend_origins`]: remus_operations::blend_ops::evolution_from_blend_origins
#[test]
fn the_binding_route_reports_the_same_map_as_the_in_process_api() {
    // The binding's sequence: snapshot the input faces BEFORE the blend (a
    // successful blend trims them in place), run the engine, then turn whatever
    // record it kept into a map.
    let mut topo = Topology::new();
    let cube = primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, cube).unwrap();
    let before = faces_of(&topo, cube);

    let input_signatures = boolean::collect_face_signatures(&topo, cube).unwrap();
    let result = blend_ops::fillet_v2(&mut topo, cube, &[edges[0]], 1.0).expect("fillet");
    let via_binding = blend_ops::evolution_from_blend_origins(
        &topo,
        result.solid,
        result.face_origins.as_ref(),
        &input_signatures,
    )
    .expect("evolution from blend origins");

    let after = faces_of(&topo, result.solid);
    assert_lineage_accounts_for_everything("binding route", &via_binding, &before, &after);

    // Same body, same edge, through the in-process API.
    let mut topo2 = Topology::new();
    let cube2 = primitives::make_box(&mut topo2, 10.0, 10.0, 10.0).unwrap();
    let edges2 = solid_edges(&topo2, cube2).unwrap();
    let (_, direct) =
        blend_ops::fillet_with_evolution(&mut topo2, cube2, &[edges2[0]], 1.0).expect("fillet");

    assert_eq!(
        via_binding.to_json(),
        direct.to_json(),
        "the binding and the in-process API disagree about the same fillet"
    );

    // And the answer itself, spelled out — this is the JSON a caller receives,
    // and it is the exact output the defect was reported against.
    //
    // The face indices are the arena's, so a change in allocation order will
    // move them. If that happens the fix is to renumber this string, never to
    // relax it: the shape is the point — six survivors one-to-one, face 6
    // generated from 0 and 2, nothing deleted and nothing refused.
    assert_eq!(
        direct.to_json(),
        "{\"modified\":{\"0\":[7],\"1\":[8],\"2\":[9],\"3\":[10],\"4\":[11],\"5\":[12]},\
         \"generated\":{\"0\":[6],\"2\":[6]},\"deleted\":[],\
         \"unresolved\":{},\"origin\":\"construction\"}",
        "the reported map for the box-edge fillet changed"
    );
}

// ── Patterns ───────────────────────────────────────────────────────

/// A pattern is the third family that produces a history, and the only one
/// whose result is a compound. Every face of every instance has to be claimed,
/// not just the faces of the instance that happens to reuse the source body.
///
/// It is also the one case where a source face legitimately appears in both
/// buckets: it is its own first instance (`modified`) and the origin of every
/// copy of itself (`generated`).
#[test]
fn pattern_lineage_accounts_for_every_instance_face() {
    for count in [2_usize, 3, 5] {
        let mut topo = Topology::new();
        let src = primitives::make_box(&mut topo, 10.0, 4.0, 4.0).unwrap();
        let before = faces_of(&topo, src);

        let (compound, evo) = remus_operations::pattern::linear_pattern_with_evolution(
            &mut topo,
            src,
            remus_math::vec::Vec3::new(1.0, 0.0, 0.0),
            25.0,
            count,
        )
        .expect("linear pattern");

        // A compound's faces are every instance's faces together.
        let mut after: HashSet<usize> = HashSet::new();
        for &inst in topo.compound(compound).unwrap().solids() {
            after.extend(faces_of(&topo, inst));
        }

        let label = format!("linear pattern of {count}");
        assert_lineage_accounts_for_everything(&label, &evo, &before, &after);

        assert_eq!(
            after.len(),
            before.len() * count,
            "{label}: every instance carries the source body's faces"
        );
        assert!(
            evo.deleted.is_empty(),
            "{label}: a pattern deletes nothing: {:?}",
            evo.deleted
        );
        assert!(evo.origin.is_exact(), "{label}: a copy is not an inference");

        for f in &before {
            assert_eq!(
                evo.modified.get(f),
                Some(&vec![*f]),
                "{label}: source face {f} is its own first instance"
            );
            let copies = evo
                .generated
                .get(f)
                .unwrap_or_else(|| panic!("{label}: source face {f} generated no instances"));
            assert_eq!(
                copies.len(),
                count - 1,
                "{label}: source face {f} -> {copies:?}"
            );
        }
    }
}

// ── Geometry did not move ──────────────────────────────────────────

/// Provenance is a bookkeeping layer, so repairing it must leave the solid
/// alone. The check is against a closed form derived by hand, not against a
/// second integrator: rounding a convex edge of length `L` at radius `r`
/// removes the prism between the two planes and the cylinder, whose section is
/// the square corner less the quarter disc — `r²(1 − π/4)` — so
///
/// ```text
/// V = 10³ − 10 · 1² · (1 − π/4) = 990 + 2.5π
/// ```
#[test]
fn filleted_cube_volume_matches_the_hand_derived_closed_form() {
    let mut topo = Topology::new();
    let cube = primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, cube).unwrap();
    let (result, _) =
        blend_ops::fillet_with_evolution(&mut topo, cube, &[edges[0]], 1.0).expect("fillet");

    let expected = 990.0 + 2.5 * std::f64::consts::PI;
    // The band is tessellated, and a chordal approximation of a convex arc
    // under-fills it, so the tolerance is the tessellation's error budget and
    // not a fudge factor: it is one-sided and shrinks with the deflection.
    let actual = remus_operations::measure::solid_volume(&topo, result.solid, 1e-5).unwrap();
    assert!(
        (actual - expected).abs() < 1e-3,
        "filleted cube volume {actual} != closed form {expected}"
    );
}

#[test]
fn failed_blend_evolution_calls_leave_source_topology_unchanged() {
    for operation in ["fillet", "chamfer"] {
        let mut topo = Topology::new();
        let cube = primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edge = solid_edges(&topo, cube).unwrap()[0];
        let before_faces = faces_of(&topo, cube);
        let before_volume = remus_operations::measure::solid_volume(&topo, cube, 0.01).unwrap();

        let failed = match operation {
            "fillet" => {
                blend_ops::fillet_with_evolution(&mut topo, cube, &[edge], 100.0).map(|_| ())
            }
            "chamfer" => blend_ops::chamfer_with_evolution(&mut topo, cube, &[edge], 100.0, 100.0)
                .map(|_| ()),
            _ => unreachable!(),
        };
        assert!(failed.is_err(), "degenerate {operation} must be rejected");

        assert_eq!(
            faces_of(&topo, cube),
            before_faces,
            "failed {operation} changed the source face set"
        );
        let after_volume = remus_operations::measure::solid_volume(&topo, cube, 0.01).unwrap();
        assert!(
            (after_volume - before_volume).abs() < 1e-9,
            "failed {operation} changed source volume: {before_volume} -> {after_volume}"
        );
        let shell = topo.solid(cube).unwrap().outer_shell();
        remus_topology::validation::validate_shell_closed(topo.shell(shell).unwrap(), &topo)
            .unwrap();
    }
}
