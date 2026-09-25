//! B18 family qualification: default V2 offset face, edge and vertex
//! history against the actual result entity sets.
//!
//! Before this slice the offset journaled a faces-only entry: every edge and
//! vertex reference severed across it even though its one-to-one face map is
//! construction-derived. [`offset_solid_v2_with_entity_evolution`] now
//! induces each result edge's and vertex's claim from that face map by exact
//! incidence, and [`offset_journaled`] records all three kinds.
//!
//! Evidence here:
//!
//! - every face, edge and vertex of the result is attributed or typed
//!   unresolved, compared with the real result sets at three modelling units
//!   and under rigid placements, with an independent nearest-entity
//!   geometric oracle for every resolved claim (a role oracle for seams);
//! - deliberately dropped and phantom records are reported, and the
//!   producer's own gate refuses them;
//! - split and merge claims attribute every child, in the report and
//!   through journal resolution;
//! - torus seams and the sphere's equator ring stay typed unresolved
//!   (`ambiguous_incidence`) instead of being guessed, and references to
//!   them fail closed while the unambiguous ones bind;
//! - the history path leaves the offset geometry bit-identical.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::{BTreeMap, BTreeSet};

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boundary_evolution::{
    BoundaryEvent, BoundaryEvolution, EntityCompletenessReport, UnresolvedReason,
    completeness_for_result_solids, require_accounted,
};
use remus_operations::journal_ops::{offset_journaled, offset_journaled_with_entities};
use remus_operations::offset_v2::{offset_solid_v2, offset_solid_v2_with_entity_evolution};
use remus_operations::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::arena::Id;
use remus_topology::explorer::{edge_to_face_map, solid_edges, solid_faces, solid_vertices};
use remus_topology::journal::{EntityKey, EntityKind, EvolutionDraft};
use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};
use remus_topology::solid::SolidId;

// ─── Helpers ────────────────────────────────────────────────────────────

fn edge_indices(topo: &Topology, solid: SolidId) -> BTreeSet<usize> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .map(Id::index)
        .collect()
}

fn vertex_indices(topo: &Topology, solid: SolidId) -> BTreeSet<usize> {
    solid_vertices(topo, solid)
        .unwrap()
        .into_iter()
        .map(Id::index)
        .collect()
}

fn face_indices(topo: &Topology, solid: SolidId) -> BTreeSet<usize> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(Id::index)
        .collect()
}

/// Points along an edge, endpoints included.
fn edge_samples(topo: &Topology, edge: usize) -> Vec<Point3> {
    let id = topo.edge_id_from_index(edge).unwrap();
    let data = topo.edge(id).unwrap();
    let start = topo.vertex(data.start()).unwrap().point();
    let end = topo.vertex(data.end()).unwrap().point();
    let (t0, t1) = data
        .strict_domain()
        .unwrap_or_else(|error| panic!("edge {edge} lacks parameter authority: {error}"));
    (0..=64)
        .map(|i| {
            let t = t0 + (t1 - t0) * f64::from(i) / 64.0;
            data.curve().evaluate_with_endpoints(t, start, end)
        })
        .collect()
}

fn edge_midpoint(topo: &Topology, edge: usize) -> Point3 {
    edge_samples(topo, edge)[32]
}

fn distance_to_samples(point: Point3, samples: &[Point3]) -> f64 {
    samples
        .iter()
        .map(|&sample| (sample - point).length())
        .fold(f64::INFINITY, f64::min)
}

fn vertex_point(topo: &Topology, vertex: usize) -> Point3 {
    topo.vertex(topo.vertex_id_from_index(vertex).unwrap())
        .unwrap()
        .point()
}

/// The source entity nearest to `point`, required to be strictly nearest.
fn unique_nearest(distances: &BTreeMap<usize, f64>, what: &str) -> usize {
    let mut sorted: Vec<(usize, f64)> = distances.iter().map(|(&i, &d)| (i, d)).collect();
    sorted.sort_by(|a, b| a.1.total_cmp(&b.1));
    assert!(
        sorted.len() < 2 || sorted[0].1 < 0.9 * sorted[1].1,
        "{what}: nearest source is not separated: {sorted:?}"
    );
    sorted[0].0
}

/// Every resolved claim is a real source entity of the right kind, each
/// source is claimed at most once (no split on these primitives), and the
/// claim passes an oracle independent of the incidence induction:
///
/// - an ordinary edge or a vertex must be the geometrically nearest source
///   entity, strictly separated from the runner-up;
/// - a seam (one face used twice) is a parametrisation artifact the engine
///   rebuilds at its own circle start, so its position is not carried; the
///   oracle instead checks its role: the claimed source is a seam of the
///   very source face the result seam's face was built from.
fn assert_claims_are_true(
    topo: &Topology,
    source: SolidId,
    result: SolidId,
    faces: &remus_operations::evolution::EvolutionMap,
    boundary: &BoundaryEvolution,
) {
    let source_edges = edge_indices(topo, source);
    let source_vertices = vertex_indices(topo, source);
    let source_uses = edge_to_face_map(topo, source).unwrap();
    let result_uses = edge_to_face_map(topo, result).unwrap();
    let face_source: BTreeMap<usize, usize> = faces
        .modified
        .iter()
        .flat_map(|(&input, outputs)| outputs.iter().map(move |&output| (output, input)))
        .collect();
    let edge_samples_by_source: BTreeMap<usize, Vec<Point3>> = source_edges
        .iter()
        .map(|&edge| (edge, edge_samples(topo, edge)))
        .collect();

    let mut claimed_edges = BTreeSet::new();
    for (&result_edge, event) in &boundary.edges {
        let BoundaryEvent::Modified { from } = event else {
            panic!("edge {result_edge}: expected a resolved modification, got {event:?}");
        };
        assert!(
            source_edges.contains(from),
            "edge {result_edge} names {from}"
        );
        assert!(
            claimed_edges.insert(*from),
            "source edge {from} claimed twice"
        );
        let uses = &result_uses[&result_edge];
        if uses.len() == 2 && uses[0] == uses[1] {
            let source_face = face_source[&uses[0].index()];
            let source_seam = &source_uses[from];
            assert!(
                source_seam.len() == 2
                    && source_seam.iter().all(|face| face.index() == source_face),
                "seam {result_edge} must name a seam of source face {source_face}"
            );
            continue;
        }
        let midpoint = edge_midpoint(topo, result_edge);
        let distances = edge_samples_by_source
            .iter()
            .map(|(&edge, samples)| (edge, distance_to_samples(midpoint, samples)))
            .collect();
        assert_eq!(
            unique_nearest(&distances, "edge"),
            *from,
            "edge {result_edge}: incidence and geometry disagree"
        );
    }
    assert_eq!(claimed_edges, source_edges, "every source edge carried");

    let mut claimed_vertices = BTreeSet::new();
    for (&result_vertex, event) in &boundary.vertices {
        let BoundaryEvent::Modified { from } = event else {
            panic!("vertex {result_vertex}: expected a resolved modification, got {event:?}");
        };
        assert!(source_vertices.contains(from));
        assert!(
            claimed_vertices.insert(*from),
            "source vertex {from} claimed twice"
        );
        let point = vertex_point(topo, result_vertex);
        let distances = source_vertices
            .iter()
            .map(|&vertex| (vertex, (vertex_point(topo, vertex) - point).length()))
            .collect();
        assert_eq!(unique_nearest(&distances, "vertex"), *from);
    }
    assert_eq!(
        claimed_vertices, source_vertices,
        "every source vertex carried"
    );
}

fn assert_report_is_accounted(report: &EntityCompletenessReport) {
    assert!(report.is_accounted(), "{report:?}");
    for kind in [&report.faces, &report.edges, &report.vertices] {
        assert!(
            kind.omitted.is_empty() && kind.phantom.is_empty(),
            "{report:?}"
        );
    }
}

type Build = fn(&mut Topology, f64) -> SolidId;

fn build_box(topo: &mut Topology, s: f64) -> SolidId {
    make_box(topo, 2.0 * s, 3.0 * s, 4.0 * s).unwrap()
}

fn build_cylinder(topo: &mut Topology, s: f64) -> SolidId {
    make_cylinder(topo, 1.0 * s, 2.0 * s).unwrap()
}

fn build_cone(topo: &mut Topology, s: f64) -> SolidId {
    make_cone(topo, 1.0 * s, 0.5 * s, 2.0 * s).unwrap()
}

/// Each primitive with the modelling units it is qualified at. Curved
/// intersection-joint offsets refuse from 100 units up in the offset engine
/// itself (B55, `crates/offset/tests/regress_curved_offset_scale.rs`), so
/// their band stops at 10; `refused_offset_publishes_no_history` pins that
/// refusal as atomic.
const RESOLVED_FAMILY: [(&str, Build, [f64; 3]); 3] = [
    ("box", build_box, [1e-3, 1.0, 1e3]),
    ("cylinder", build_cylinder, [1e-3, 1.0, 10.0]),
    ("cone", build_cone, [1e-3, 1.0, 10.0]),
];

// ─── Total history on the resolved primitives ───────────────────────────

/// Box, cylinder and cone: every result face, edge and vertex resolves to
/// its unique source, the claims match the real result sets and the
/// independent oracle, across each primitive's qualified units, outward and
/// inward, at the origin and under a rigid placement.
#[test]
fn offset_history_is_total_resolved_and_geometrically_true() {
    for (name, build, scales) in RESOLVED_FAMILY {
        for scale in scales {
            let placements = [
                Mat4::identity(),
                Mat4::translation(100.0 * scale, -50.0 * scale, 30.0 * scale)
                    * Mat4::rotation_z(0.7)
                    * Mat4::rotation_x(0.3),
            ];
            for placement in &placements {
                for sign in [1.0, -1.0] {
                    let mut topo = Topology::new();
                    let source = build(&mut topo, scale);
                    transform_solid(&mut topo, source, placement).unwrap();
                    let distance = sign * 0.2 * scale;
                    let context = format!("{name} scale {scale} distance {distance}");

                    let result = offset_solid_v2_with_entity_evolution(&mut topo, source, distance)
                        .unwrap_or_else(|error| panic!("{context}: {error}"));

                    assert!(result.faces.origin.is_exact(), "{context}");
                    assert!(result.completeness.is_resolved(), "{context}");
                    assert_report_is_accounted(&result.completeness);
                    let recomputed = completeness_for_result_solids(
                        &topo,
                        &result.faces,
                        &result.boundary,
                        &[result.solid],
                    )
                    .unwrap();
                    assert_eq!(recomputed, result.completeness, "{context}");
                    assert_eq!(
                        result
                            .boundary
                            .edges
                            .keys()
                            .copied()
                            .collect::<BTreeSet<_>>(),
                        edge_indices(&topo, result.solid),
                        "{context}"
                    );
                    assert_eq!(
                        result
                            .boundary
                            .vertices
                            .keys()
                            .copied()
                            .collect::<BTreeSet<_>>(),
                        vertex_indices(&topo, result.solid),
                        "{context}"
                    );
                    assert_claims_are_true(
                        &topo,
                        source,
                        result.solid,
                        &result.faces,
                        &result.boundary,
                    );
                }
            }
        }
    }
}

/// The history path changes no geometry: the same offset through the plain
/// entry point yields the same entity counts, vertex positions and volume.
#[test]
fn history_path_leaves_offset_geometry_unchanged() {
    for (name, build, _) in RESOLVED_FAMILY {
        let mut plain_topo = Topology::new();
        let plain_source = build(&mut plain_topo, 1.0);
        let plain = offset_solid_v2(&mut plain_topo, plain_source, 0.3).unwrap();

        let mut topo = Topology::new();
        let source = build(&mut topo, 1.0);
        let journaled = offset_journaled_with_entities(&mut topo, source, 0.3).unwrap();

        for (a, b) in [
            (
                face_indices(&plain_topo, plain),
                face_indices(&topo, journaled.solid),
            ),
            (
                edge_indices(&plain_topo, plain),
                edge_indices(&topo, journaled.solid),
            ),
            (
                vertex_indices(&plain_topo, plain),
                vertex_indices(&topo, journaled.solid),
            ),
        ] {
            assert_eq!(a, b, "{name}: identical allocation expected");
        }
        for vertex in vertex_indices(&topo, journaled.solid) {
            let p = vertex_point(&plain_topo, vertex);
            let q = vertex_point(&topo, vertex);
            assert_eq!(
                (p.x().to_bits(), p.y().to_bits(), p.z().to_bits()),
                (q.x().to_bits(), q.y().to_bits(), q.z().to_bits()),
                "{name}: vertex {vertex} moved"
            );
        }
        let v_plain = remus_operations::measure::solid_volume(&plain_topo, plain, 0.01).unwrap();
        let v_journaled =
            remus_operations::measure::solid_volume(&topo, journaled.solid, 0.01).unwrap();
        assert_eq!(v_plain.to_bits(), v_journaled.to_bits(), "{name}");
    }

    // Closed form on the box: (a + 2d)(b + 2d)(c + 2d).
    let mut topo = Topology::new();
    let source = build_box(&mut topo, 1.0);
    let journaled = offset_journaled_with_entities(&mut topo, source, 0.3).unwrap();
    let volume = remus_operations::measure::solid_volume(&topo, journaled.solid, 0.01).unwrap();
    assert!((volume - 2.6 * 3.6 * 4.6).abs() < 1e-9, "{volume}");
}

/// Outside the engine's curved band the history path refuses exactly like
/// the plain offset, and publishes nothing: topology counts, the journal,
/// and an outstanding unjournaled gap are all unchanged until the next
/// successful journaled operation.
#[test]
fn refused_offset_publishes_no_history() {
    let mut plain_topo = Topology::new();
    let plain_source = build_cylinder(&mut plain_topo, 1e3);
    let plain_error = offset_solid_v2(&mut plain_topo, plain_source, 200.0)
        .unwrap_err()
        .to_string();

    let mut topo = Topology::new();
    let source = build_cylinder(&mut topo, 1e3);
    let pending = topo.journal_begin("source_fixture");
    remus_operations::journal_ops::record_barrier_over_solid(&mut topo, pending, source).unwrap();
    // An unjournaled edit after the anchor: an outstanding gap.
    let unrelated = build_box(&mut topo, 1.0);
    let counts = |topo: &Topology| {
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
            topo.num_loops(),
            topo.num_coedges(),
        )
    };
    let before_counts = counts(&topo);
    let before = topo.journal().snapshot();

    let error = offset_journaled_with_entities(&mut topo, source, 200.0)
        .unwrap_err()
        .to_string();
    assert_eq!(error, plain_error, "same refusal as the plain offset");
    assert!(error.contains("no reconstructed wire loops"), "{error}");
    let after = topo.journal().snapshot();
    assert_eq!(after.entries, before.entries, "refusal published history");
    assert_eq!(after.index, before.index);
    assert_eq!(after.next_ordinal, before.next_ordinal);
    assert_eq!(counts(&topo), before_counts);

    // The outstanding gap is published exactly once, by the next successful
    // journaled operation, directly before it.
    let op = offset_journaled(&mut topo, unrelated, 0.1).unwrap().op;
    let entries = topo.journal().entries();
    assert_eq!(entries.len(), before.entries.len() + 2);
    assert_eq!(
        entries[entries.len() - 2].kind(),
        remus_topology::journal::UNJOURNALED_MUTATIONS
    );
    assert_eq!(entries.last().unwrap().op(), op);
}

// ─── Deliberately incomplete records ────────────────────────────────────

/// Dropping one face, edge or vertex record, or adding a phantom claim, is
/// reported per kind and refused by the producer's accounting gate.
#[test]
fn dropped_or_phantom_records_are_reported_incomplete() {
    let mut topo = Topology::new();
    let source = build_box(&mut topo, 1.0);
    let result = offset_solid_v2_with_entity_evolution(&mut topo, source, 0.25).unwrap();
    let check = |faces: &remus_operations::evolution::EvolutionMap,
                 boundary: &BoundaryEvolution| {
        completeness_for_result_solids(&topo, faces, boundary, &[result.solid]).unwrap()
    };
    assert!(check(&result.faces, &result.boundary).is_resolved());

    // One edge record dropped.
    let mut boundary = result.boundary.clone();
    let (&dropped_edge, _) = boundary.edges.iter().nth(5).unwrap();
    boundary.edges.remove(&dropped_edge);
    let report = check(&result.faces, &boundary);
    assert_eq!(report.edges.omitted, vec![dropped_edge]);
    assert!(report.faces.is_accounted() && report.vertices.is_accounted());
    assert!(!report.is_accounted());
    let error = require_accounted("offset", &report)
        .unwrap_err()
        .to_string();
    assert!(error.contains("does not account"), "{error}");
    assert!(
        error.contains(&format!("edges omitted [{dropped_edge}]")),
        "{error}"
    );

    // One vertex record dropped.
    let mut boundary = result.boundary.clone();
    let (&dropped_vertex, _) = boundary.vertices.iter().next().unwrap();
    boundary.vertices.remove(&dropped_vertex);
    let report = check(&result.faces, &boundary);
    assert_eq!(report.vertices.omitted, vec![dropped_vertex]);
    assert!(require_accounted("offset", &report).is_err());

    // One face record dropped.
    let mut faces = result.faces.clone();
    let source_face = *faces.modified.keys().min().unwrap();
    let dropped_face = faces.modified.remove(&source_face).unwrap();
    let report = check(&faces, &result.boundary);
    assert_eq!(report.faces.omitted, dropped_face);
    assert!(require_accounted("offset", &report).is_err());

    // A phantom edge claim naming a source edge, not a result edge.
    let mut boundary = result.boundary.clone();
    let phantom = *edge_indices(&topo, source).iter().next().unwrap();
    boundary
        .edges
        .insert(phantom, BoundaryEvent::Modified { from: phantom });
    let report = check(&result.faces, &boundary);
    assert_eq!(report.edges.phantom, vec![phantom]);
    assert!(report.edges.omitted.is_empty());
    assert!(require_accounted("offset", &report).is_err());
}

// ─── Split and merge ────────────────────────────────────────────────────

/// A split (one source edge into two result edges) and a merge (two source
/// edges into one result edge) attribute every child: the report accounts
/// for both pieces and drops exactly the one whose record is removed, and
/// the journal resolves the split source to both pieces and each merged
/// source to the merged edge.
#[test]
fn split_and_merge_claims_attribute_every_child() {
    let mut topo = Topology::new();
    let base = build_box(&mut topo, 1.0);
    // Built before the anchoring offset, so it is not an unjournaled gap
    // between the anchor and the synthetic entry below.
    let target = build_box(&mut topo, 1.0);
    // Anchor the source entities as journaled outputs.
    let first = offset_journaled(&mut topo, base, 0.25).unwrap();
    let source = first.solid;

    let sources: Vec<usize> = edge_indices(&topo, source).into_iter().collect();
    let results: Vec<usize> = edge_indices(&topo, target).into_iter().collect();
    // results[0], results[1] <- sources[0] (split);
    // results[2] <- sources[1] + sources[2] (merge);
    // results[3..12] <- sources[3..12] one to one.
    let mut boundary = BoundaryEvolution::default();
    boundary
        .edges
        .insert(results[0], BoundaryEvent::Modified { from: sources[0] });
    boundary
        .edges
        .insert(results[1], BoundaryEvent::Modified { from: sources[0] });
    boundary.edges.insert(
        results[2],
        BoundaryEvent::Merged {
            from: vec![sources[1], sources[2]],
        },
    );
    for i in 3..results.len() {
        boundary
            .edges
            .insert(results[i], BoundaryEvent::Modified { from: sources[i] });
    }
    let source_vertices: Vec<usize> = vertex_indices(&topo, source).into_iter().collect();
    for (i, v) in vertex_indices(&topo, target).into_iter().enumerate() {
        boundary.vertices.insert(
            v,
            BoundaryEvent::Modified {
                from: source_vertices[i],
            },
        );
    }
    let mut faces = remus_operations::evolution::EvolutionMap::exact();
    let source_faces: Vec<usize> = face_indices(&topo, source).into_iter().collect();
    for (i, f) in face_indices(&topo, target).into_iter().enumerate() {
        faces.add_modified(source_faces[i], f);
    }

    let report = completeness_for_result_solids(&topo, &faces, &boundary, &[target]).unwrap();
    assert!(report.is_resolved(), "{report:?}");

    let mut dropped = boundary.clone();
    dropped.edges.remove(&results[1]);
    let report = completeness_for_result_solids(&topo, &faces, &dropped, &[target]).unwrap();
    assert_eq!(
        report.edges.omitted,
        vec![results[1]],
        "the other piece stays"
    );

    // Journal the claims and resolve the source references through them.
    let pending =
        remus_operations::journal_ops::begin_scoped(&mut topo, "synthetic", &[source]).unwrap();
    let mut draft = EvolutionDraft::construction();
    for (&input, outs) in &faces.modified {
        for &out in outs {
            draft.push(
                EntityKey::face(out),
                remus_topology::journal::EventDraft::Modified {
                    from: EntityKey::face(input),
                },
            );
        }
    }
    for (subject, event) in boundary.journal_events() {
        draft.push(subject, event);
    }
    topo.journal_record_evolution(pending, draft).unwrap();

    let reference_to = |topo: &Topology, edge: usize| {
        let index = (0..sources.len())
            .find(|&i| {
                first_entry_output_of_kind(topo, first.op, EntityKind::Edge, i) == Some(edge)
            })
            .unwrap();
        PersistentRef::operation_output(first.op, EntityKind::Edge, index)
    };
    assert_eq!(
        resolve(&topo, &reference_to(&topo, sources[0])),
        Resolution::BoundMany {
            entities: vec![EntityKey::edge(results[0]), EntityKey::edge(results[1])],
            provenance: Provenance::Construction,
        },
        "a split source binds every piece"
    );
    for merged_source in [sources[1], sources[2]] {
        assert_eq!(
            resolve(&topo, &reference_to(&topo, merged_source)),
            Resolution::Bound {
                entity: EntityKey::edge(results[2]),
                provenance: Provenance::Construction,
            },
            "each merged source binds the merged edge"
        );
    }
    // The one-to-one remainder binds too, so the synthetic entry is a
    // complete record rather than a severing one.
    assert_eq!(
        resolve(&topo, &reference_to(&topo, sources[3])),
        Resolution::Bound {
            entity: EntityKey::edge(results[3]),
            provenance: Provenance::Construction,
        }
    );
}

// ─── Typed unresolved outcomes ──────────────────────────────────────────

/// A torus's two seams share one face-use multiset, so neither result seam
/// can be bound to a source seam: both are unresolved with
/// `ambiguous_incidence` naming exactly the two source seams, while the
/// face and the single vertex resolve. The record is still accounted.
#[test]
fn torus_seams_are_typed_unresolved_not_guessed() {
    for scale in [1e-3, 1.0, 1e3] {
        for sign in [1.0, -1.0] {
            let mut topo = Topology::new();
            let source = make_torus(&mut topo, 3.0 * scale, 1.0 * scale, 16).unwrap();
            let source_seams: Vec<usize> = edge_indices(&topo, source).into_iter().collect();
            assert_eq!(source_seams.len(), 2);

            let result =
                offset_solid_v2_with_entity_evolution(&mut topo, source, sign * 0.3 * scale)
                    .unwrap();
            assert!(result.completeness.is_accounted());
            assert!(!result.completeness.is_resolved());
            assert!(result.completeness.faces.is_resolved());
            assert!(result.completeness.vertices.is_resolved());
            assert_eq!(
                result.completeness.edges.unresolved_outputs,
                edge_indices(&topo, result.solid)
                    .into_iter()
                    .collect::<Vec<_>>()
            );
            for event in result.boundary.edges.values() {
                assert_eq!(
                    event,
                    &BoundaryEvent::Unresolved {
                        candidates: source_seams.clone(),
                        reason: UnresolvedReason::AmbiguousIncidence,
                    }
                );
            }
            let unresolved = result.boundary.unresolved();
            assert_eq!(unresolved.len(), 2);
            assert!(unresolved.iter().all(|(kind, _, _, reason)| {
                *kind == EntityKind::Edge && reason.as_str() == "ambiguous_incidence"
            }));
        }
    }
}

/// The sphere's equator is a symmetric ring: every equator edge and vertex
/// has the same two-hemisphere incidence as every other, so all of them
/// stay typed unresolved. The count comes from the source, not a constant.
#[test]
fn sphere_equator_ring_is_typed_unresolved() {
    let mut topo = Topology::new();
    let source = make_sphere(&mut topo, 1.0, 16).unwrap();
    let ring_edges: Vec<usize> = edge_to_face_map(&topo, source)
        .unwrap()
        .into_iter()
        .filter(|(_, faces)| faces.len() == 2 && faces[0] != faces[1])
        .map(|(edge, _)| edge)
        .collect();
    assert!(ring_edges.len() > 1);

    let result = offset_solid_v2_with_entity_evolution(&mut topo, source, 0.25).unwrap();
    assert!(result.completeness.is_accounted());
    let unresolved_edges = result
        .boundary
        .edges
        .values()
        .filter(|event| {
            matches!(
                event,
                BoundaryEvent::Unresolved { candidates, reason: UnresolvedReason::AmbiguousIncidence }
                    if *candidates == ring_edges
            )
        })
        .count();
    assert_eq!(unresolved_edges, ring_edges.len());
    assert!(
        result
            .boundary
            .vertices
            .values()
            .all(|event| matches!(event, BoundaryEvent::Unresolved { .. }))
    );
}

// ─── Journal resolution ─────────────────────────────────────────────────

/// Edge and vertex references now follow an offset instead of severing:
/// the box's references bind the claimed successor with construction
/// provenance through a second offset, and a torus seam reference fails
/// closed naming the offset while its vertex reference binds.
#[test]
fn journaled_offset_carries_edge_and_vertex_references() {
    let mut topo = Topology::new();
    let source = build_box(&mut topo, 1.0);
    let first = offset_journaled(&mut topo, source, 0.25).unwrap();
    let entry = topo.journal().entries().last().unwrap();
    let remus_topology::journal::EntryPayload::Evolution { events, .. } = entry.payload() else {
        panic!("offset must journal evolution");
    };
    assert_eq!(events.len(), 6 + 12 + 8, "one event per result entity");

    let second = offset_journaled_with_entities(&mut topo, first.solid, 0.25).unwrap();
    assert!(second.completeness.is_resolved());
    for kind in [EntityKind::Edge, EntityKind::Vertex] {
        let count = if kind == EntityKind::Edge { 12 } else { 8 };
        for index in 0..count {
            let reference = PersistentRef::operation_output(first.op, kind, index);
            let Resolution::Bound {
                entity,
                provenance: Provenance::Construction,
            } = resolve(&topo, &reference)
            else {
                panic!("{kind:?} output {index} must follow the second offset exactly");
            };
            let claims = if kind == EntityKind::Edge {
                &second.boundary.edges
            } else {
                &second.boundary.vertices
            };
            let BoundaryEvent::Modified { from } = claims[&entity.index] else {
                panic!("bound entity must carry a modification claim");
            };
            let anchored = first_entry_output_of_kind(&topo, first.op, kind, index).unwrap();
            assert_eq!(from, anchored, "the binding follows the recorded claim");
        }
    }

    let mut topo = Topology::new();
    let torus = make_torus(&mut topo, 3.0, 1.0, 16).unwrap();
    let first = offset_journaled(&mut topo, torus, 0.2).unwrap();
    offset_journaled(&mut topo, first.solid, 0.2).unwrap();
    for index in 0..2 {
        let reference = PersistentRef::operation_output(first.op, EntityKind::Edge, index);
        match resolve(&topo, &reference) {
            Resolution::UnresolvedAcrossOperation { kind, .. } => assert_eq!(kind, "offset"),
            other => panic!("an ambiguous seam must fail closed, got {other:?}"),
        }
    }
    let vertex = PersistentRef::operation_output(first.op, EntityKind::Vertex, 0);
    assert!(matches!(
        resolve(&topo, &vertex),
        Resolution::Bound {
            provenance: Provenance::Construction,
            ..
        }
    ));
}

fn first_entry_output_of_kind(
    topo: &Topology,
    op: remus_topology::journal::OpId,
    kind: EntityKind,
    index: usize,
) -> Option<usize> {
    let journal = topo.journal();
    let entry = journal.entries().iter().find(|entry| entry.op() == op)?;
    let remus_topology::journal::EntryPayload::Evolution { events, .. } = entry.payload() else {
        return None;
    };
    events
        .iter()
        .filter_map(|(ordinal, _)| journal.key_of(*ordinal))
        .filter(|key| key.kind == kind)
        .nth(index)
        .map(|key| key.index)
}
