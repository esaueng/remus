//! Closed-form oracles for the N-way region fuse and its lineage export.
//!
//! The mutation oracle is per package, so the store's caller maps, the
//! store-agnostic evolution export and the builder's per-source tally are
//! pinned here rather than through `remus-operations`. Every expectation is
//! derived by hand from unit cubes: volumes by inclusion–exclusion, entity
//! counts by counting corners, and each lineage claim read back as geometry.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::{BTreeMap, BTreeSet};

use remus_math::vec::{Point3, Vec3};
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::face::FaceSurface;
use remus_topology::test_utils::make_unit_cube_manifold_at;

use super::*;
use crate::builder::SourceFaceTally;
use crate::ds::shape_store::EvolutionStore;
use crate::error::NaryFuseRefusal;

const EPS: f64 = 1e-12;

fn cubes(topo: &mut Topology, origins: &[[f64; 3]]) -> Vec<SolidId> {
    origins
        .iter()
        .map(|o| make_unit_cube_manifold_at(topo, o[0], o[1], o[2]))
        .collect()
}

fn fuse(topo: &mut Topology, sources: &[SolidId]) -> NaryFuseRegions {
    fuse_n_regions_with_entity_evolution(
        topo,
        sources,
        None,
        &OperationContext::new(),
        &mut |_, _, _| Ok(()),
    )
    .unwrap()
}

/// A wire as an ordered polygon.
fn loop_points(topo: &Topology, wire: remus_topology::wire::WireId) -> Vec<Point3> {
    topo.wire(wire)
        .unwrap()
        .edges()
        .iter()
        .map(|oriented| {
            let edge = topo.edge(oriented.edge()).unwrap();
            let start = if oriented.is_forward() {
                edge.start()
            } else {
                edge.end()
            };
            topo.vertex(start).unwrap().point()
        })
        .collect()
}

fn loop_area(points: &[Point3], normal: Vec3) -> f64 {
    let mut twice = Vec3::new(0.0, 0.0, 0.0);
    for (index, &from) in points.iter().enumerate() {
        let to = points[(index + 1) % points.len()];
        let (a, b) = (from - points[0], to - points[0]);
        twice += a.cross(b);
    }
    (twice.dot(normal) / 2.0).abs()
}

fn plane(topo: &Topology, face: FaceId) -> (Vec3, f64) {
    let stored = topo.face(face).unwrap();
    let FaceSurface::Plane { normal, d } = *stored.surface() else {
        panic!("face {face:?} is not planar");
    };
    if stored.is_reversed() {
        (normal * -1.0, -d)
    } else {
        (normal, d)
    }
}

/// Volume of a planar-faced solid by the divergence theorem: a face with
/// outward unit normal `n` on the plane `n · x = d` contributes
/// `d · area / 3`. Holes subtract their area; cavity shells are walked too.
fn polyhedron_volume(topo: &Topology, solid: SolidId) -> f64 {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(|face| {
            let (normal, d) = plane(topo, face);
            let stored = topo.face(face).unwrap();
            let outer = loop_area(&loop_points(topo, stored.outer_wire()), normal);
            let holes: f64 = stored
                .inner_wires()
                .iter()
                .map(|&wire| loop_area(&loop_points(topo, wire), normal))
                .sum();
            d * (outer - holes) / 3.0
        })
        .sum()
}

fn face_set(topo: &Topology, solid: SolidId) -> BTreeSet<usize> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(FaceId::index)
        .collect()
}

fn edge_ends(topo: &Topology, solids: &[SolidId]) -> BTreeMap<usize, (Point3, Point3)> {
    let mut ends = BTreeMap::new();
    for &solid in solids {
        for edge in solid_edges(topo, solid).unwrap() {
            let stored = topo.edge(edge).unwrap();
            ends.insert(
                edge.index(),
                (
                    topo.vertex(stored.start()).unwrap().point(),
                    topo.vertex(stored.end()).unwrap().point(),
                ),
            );
        }
    }
    ends
}

fn vertex_points(topo: &Topology, solids: &[SolidId]) -> BTreeMap<usize, Point3> {
    solids
        .iter()
        .flat_map(|&solid| solid_vertices(topo, solid).unwrap())
        .map(|vertex| (vertex.index(), topo.vertex(vertex).unwrap().point()))
        .collect()
}

fn on_segment(p: Point3, a: Point3, b: Point3) -> bool {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.dot(ab)).clamp(0.0, 1.0);
    (p - (a + ab * t)).length() <= EPS
}

fn on_plane(p: Point3, (normal, d): (Vec3, f64)) -> bool {
    (normal.dot(Vec3::new(p.x(), p.y(), p.z())) - d).abs() <= EPS
}

/// Hold one region's whole evolution to the geometry it describes, and
/// return its `(preserved, modified, generated)` edge counts and
/// `(preserved, created)` vertex counts.
fn read_back_evolution(
    topo: &Topology,
    sources: &[SolidId],
    region: &BooleanRegion,
) -> ((usize, usize, usize), (usize, usize)) {
    let source_faces: Vec<BTreeSet<usize>> = sources.iter().map(|&s| face_set(topo, s)).collect();
    let owner = |face: usize| source_faces.iter().position(|set| set.contains(&face));
    let caller_face = |index: usize| {
        sources
            .iter()
            .flat_map(|&s| solid_faces(topo, s).unwrap())
            .find(|face| face.index() == index)
            .unwrap()
    };

    // Faces: one claim per result face; the result face lies in its source
    // face's plane and faces the same way.
    let result_faces = solid_faces(topo, region.solid).unwrap();
    let claims: BTreeSet<usize> = region.evolution.faces.iter().map(|&(f, _)| f).collect();
    assert_eq!(claims, face_set(topo, region.solid));
    assert_eq!(claims.len(), region.evolution.faces.len());
    for &(face, source) in &region.evolution.faces {
        let source = source.expect("every N-way result face has a source");
        assert!(
            owner(source).is_some(),
            "face {face} names a non-source face"
        );
        let result = *result_faces.iter().find(|f| f.index() == face).unwrap();
        let (normal, d) = plane(topo, result);
        let (source_normal, source_d) = plane(topo, caller_face(source));
        assert!((normal - source_normal).length() <= EPS && (d - source_d).abs() <= EPS);
    }

    // Edges and vertices.
    let mut all = sources.to_vec();
    all.push(region.solid);
    let ends = edge_ends(topo, &all);
    let result_edges: BTreeSet<usize> = solid_edges(topo, region.solid)
        .unwrap()
        .into_iter()
        .map(remus_topology::EdgeId::index)
        .collect();
    assert_eq!(
        region
            .evolution
            .edges
            .iter()
            .map(|(e, _)| *e)
            .collect::<BTreeSet<_>>(),
        result_edges
    );
    assert_eq!(region.evolution.edges.len(), result_edges.len());
    assert!(region.evolution.edges.windows(2).all(|w| w[0].0 < w[1].0));
    let (mut preserved, mut modified, mut generated) = (0, 0, 0);
    for (edge, event) in &region.evolution.edges {
        let (from, to) = ends[edge];
        match event {
            EdgeEvent::Preserved(source) => {
                preserved += 1;
                let (a, b) = ends[source];
                assert!((from - a).length() <= EPS && (to - b).length() <= EPS);
            }
            EdgeEvent::Modified(source) => {
                modified += 1;
                let (a, b) = ends[source];
                assert!(on_segment(from, a, b) && on_segment(to, a, b));
                assert!((to - from).length() > EPS);
            }
            EdgeEvent::Generated { face_a, face_b } => {
                generated += 1;
                let (face_a, face_b) = (face_a.unwrap(), face_b.unwrap());
                assert_ne!(owner(face_a).unwrap(), owner(face_b).unwrap());
                for face in [face_a, face_b] {
                    let carrier = plane(topo, caller_face(face));
                    assert!(on_plane(from, carrier) && on_plane(to, carrier));
                }
            }
            EdgeEvent::Unresolved => panic!("edge {edge} is unresolved"),
        }
    }

    let source_vertices = vertex_points(topo, sources);
    let result_vertices = vertex_points(topo, &[region.solid]);
    assert_eq!(
        region
            .evolution
            .vertices
            .iter()
            .map(|(v, _)| *v)
            .collect::<BTreeSet<_>>(),
        result_vertices.keys().copied().collect::<BTreeSet<_>>()
    );
    assert!(
        region
            .evolution
            .vertices
            .windows(2)
            .all(|w| w[0].0 < w[1].0)
    );
    let (mut kept, mut created) = (0, 0);
    for (vertex, event) in &region.evolution.vertices {
        let at = result_vertices[vertex];
        match event {
            VertexEvent::Preserved(source) => {
                kept += 1;
                assert!((at - source_vertices[source]).length() <= EPS);
            }
            // `Created` says only that the vertex is no copy of an input
            // vertex: the splitter rebuilds some corners in place, so its
            // position proves nothing either way.
            VertexEvent::Created => created += 1,
        }
    }
    ((preserved, modified, generated), (kept, created))
}

/// Result faces per source, read from the exported lineage.
fn faces_per_source(topo: &Topology, sources: &[SolidId], regions: &[BooleanRegion]) -> Vec<usize> {
    let source_faces: Vec<BTreeSet<usize>> = sources.iter().map(|&s| face_set(topo, s)).collect();
    let mut counts = vec![0; sources.len()];
    for region in regions {
        for &(_, source) in &region.evolution.faces {
            let owner = source_faces
                .iter()
                .position(|set| set.contains(&source.unwrap()))
                .unwrap();
            counts[owner] += 1;
        }
    }
    counts
}

/// Two cubes offset by half a cube on every axis. Each cube has one corner
/// inside the other, so:
///
/// - volume `1 + 1 − 1/8`;
/// - each cube keeps its three far faces whole and the L-shaped remainder of
///   the three near ones, and loses the three squares inside the other:
///   6 kept, 3 inside, none coincident;
/// - vertices: 16 corners less the two buried ones, plus the six points where
///   an edge pierces a face;
/// - edges: nine untouched per cube, three shortened per cube, and the six
///   sides of the hexagonal section loop.
#[test]
fn two_offset_cubes_have_the_hand_counted_union() {
    let mut topo = Topology::new();
    let sources = cubes(&mut topo, &[[0.0; 3], [0.5; 3]]);
    let fused = fuse(&mut topo, &sources);

    assert_eq!(fused.regions.len(), 1);
    let region = &fused.regions[0];
    assert!((polyhedron_volume(&topo, region.solid) - 1.875).abs() <= EPS);
    let tally = SourceFaceTally {
        kept: 6,
        inside: 3,
        coincident: 0,
    };
    assert_eq!(fused.sources, vec![tally, tally]);
    assert_eq!(
        faces_per_source(&topo, &sources, &fused.regions),
        vec![6, 6]
    );
    assert_eq!(
        read_back_evolution(&topo, &sources, region),
        ((18, 6, 6), (14, 6))
    );
}

/// W-a at half scale: three cubes with no two faces coplanar.
/// `3 − (0.328125 + 0.234375 + 0.421875) + 0.15625 = 2.171875`.
#[test]
fn three_cubes_fuse_to_the_inclusion_exclusion_volume_with_total_lineage() {
    let mut topo = Topology::new();
    let sources = cubes(
        &mut topo,
        &[[0.0, 0.0, 0.0], [0.5, 0.25, 0.125], [0.25, 0.5, 0.375]],
    );
    let fused = fuse(&mut topo, &sources);

    assert_eq!(fused.regions.len(), 1);
    let region = &fused.regions[0];
    assert!((polyhedron_volume(&topo, region.solid) - 2.171_875).abs() <= EPS);

    // The tally and the exported lineage count the same kept faces, source by
    // source, and no face is coincident.
    let per_source = faces_per_source(&topo, &sources, &fused.regions);
    assert_eq!(
        fused.sources.iter().map(|t| t.kept).collect::<Vec<_>>(),
        per_source
    );
    assert_eq!(
        per_source.iter().sum::<usize>(),
        region.evolution.faces.len()
    );
    for tally in &fused.sources {
        assert!(
            tally.kept >= 6,
            "every cube keeps a piece of each face: {tally:?}"
        );
        assert!(
            tally.inside > 0,
            "every cube has a face piece inside another"
        );
        assert_eq!(tally.coincident, 0);
    }

    let ((_, _, generated), (kept, created)) = read_back_evolution(&topo, &sources, region);
    assert!(generated > 0 && created > 0);
    // Each cube buries the one corner the other two share material around;
    // the rest of the 24 corners survive.
    assert!(kept < 24);

    // Euler: one closed genus-0 shell whose faces have no holes.
    let faces = solid_faces(&topo, region.solid).unwrap();
    assert!(
        faces
            .iter()
            .all(|&f| topo.face(f).unwrap().inner_wires().is_empty())
    );
    let (v, e, f) = (
        region.evolution.vertices.len(),
        region.evolution.edges.len(),
        faces.len(),
    );
    assert_eq!(v + f, e + 2, "V={v} E={e} F={f}");
}

/// Two cubes meeting face to face. The shared square is an interior
/// interface: both copies are dropped as coincident, nothing is inside
/// anything, and the union is a 1×1×2 block of ten faces.
#[test]
fn abutting_cubes_drop_their_interface_as_coincident() {
    let mut topo = Topology::new();
    let sources = cubes(&mut topo, &[[0.0; 3], [1.0, 0.0, 0.0]]);
    let fused = fuse(&mut topo, &sources);

    assert_eq!(fused.regions.len(), 1);
    assert!((polyhedron_volume(&topo, fused.regions[0].solid) - 2.0).abs() <= EPS);
    let tally = SourceFaceTally {
        kept: 5,
        inside: 0,
        coincident: 1,
    };
    assert_eq!(fused.sources, vec![tally, tally]);
    assert_eq!(
        faces_per_source(&topo, &sources, &fused.regions),
        vec![5, 5]
    );
    let ((_, _, generated), (kept, created)) =
        read_back_evolution(&topo, &sources, &fused.regions[0]);
    // Nothing is cut: no section edge, no new vertex, and the four corners
    // of the interface survive once each.
    assert_eq!((generated, created), (0, 0));
    assert_eq!(kept, 12);
}

/// Two cubes overlapping by half along x. Each has its end face buried in
/// the other (1 inside). Each of the four side planes is split in two at the
/// other cube's end plane: the outer half is kept, and of the two coincident
/// inner halves — one from each cube — exactly one survives.
#[test]
fn half_overlapping_cubes_keep_one_of_each_coincident_pair() {
    let mut topo = Topology::new();
    let sources = cubes(&mut topo, &[[0.0; 3], [0.5, 0.0, 0.0]]);
    let fused = fuse(&mut topo, &sources);

    assert_eq!(fused.regions.len(), 1);
    assert!((polyhedron_volume(&topo, fused.regions[0].solid) - 1.5).abs() <= EPS);
    for tally in &fused.sources {
        assert_eq!(tally.inside, 1, "{tally:?}");
        assert_eq!(
            tally.kept + tally.inside + tally.coincident,
            10,
            "{tally:?}"
        );
    }
    let kept: usize = fused.sources.iter().map(|t| t.kept).sum();
    let coincident: usize = fused.sources.iter().map(|t| t.coincident).sum();
    assert_eq!((kept, coincident), (14, 4));
    assert_eq!(
        faces_per_source(&topo, &sources, &fused.regions),
        fused.sources.iter().map(|t| t.kept).collect::<Vec<_>>()
    );
    read_back_evolution(&topo, &sources, &fused.regions[0]);
}

/// Two overlapping cubes and one far away: two regions, the far one a
/// face-for-face, edge-for-edge, vertex-for-vertex image of its source.
/// Listing only the pair that can touch gives the same answer.
#[test]
fn a_far_source_is_its_own_region_with_identity_lineage() {
    let build = |pairs: Option<&[(usize, usize)]>| {
        let mut topo = Topology::new();
        let sources = cubes(&mut topo, &[[0.0; 3], [5.0; 3], [0.5; 3]]);
        let fused = fuse_n_regions_with_entity_evolution(
            &mut topo,
            &sources,
            pairs,
            &OperationContext::new(),
            &mut |_, _, _| Ok(()),
        )
        .unwrap();
        (topo, sources, fused)
    };
    let (topo, sources, fused) = build(None);
    assert_eq!(fused.regions.len(), 2);

    let far_faces = face_set(&topo, sources[1]);
    let far = fused
        .regions
        .iter()
        .find(|region| {
            region
                .evolution
                .faces
                .iter()
                .all(|&(_, source)| far_faces.contains(&source.unwrap()))
        })
        .expect("one region is the far cube alone");
    let sources_named: BTreeSet<usize> = far
        .evolution
        .faces
        .iter()
        .map(|&(_, s)| s.unwrap())
        .collect();
    assert_eq!(sources_named, far_faces, "each far face is named once");
    assert_eq!(
        read_back_evolution(&topo, &sources, far),
        ((12, 0, 0), (8, 0))
    );
    let named_edges: BTreeSet<usize> = far
        .evolution
        .edges
        .iter()
        .map(|(_, event)| match event {
            EdgeEvent::Preserved(source) => *source,
            other => panic!("{other:?}"),
        })
        .collect();
    assert_eq!(named_edges.len(), 12, "twelve distinct source edges");
    assert!((polyhedron_volume(&topo, far.solid) - 1.0).abs() <= EPS);

    assert_eq!(
        fused.sources,
        vec![
            SourceFaceTally {
                kept: 6,
                inside: 3,
                coincident: 0
            },
            SourceFaceTally {
                kept: 6,
                inside: 0,
                coincident: 0
            },
            SourceFaceTally {
                kept: 6,
                inside: 3,
                coincident: 0
            },
        ]
    );
    let total: f64 = fused
        .regions
        .iter()
        .map(|region| polyhedron_volume(&topo, region.solid))
        .sum();
    assert!((total - 2.875).abs() <= EPS);

    let (_, _, pruned) = build(Some(&[(0, 2)]));
    assert_eq!(pruned, fused, "the pruned pair list changes nothing");
}

/// The store records where every input entity came from, and a store built
/// without the maps says so instead of answering.
#[test]
fn the_store_maps_every_input_entity_back_to_its_caller_entity() {
    let mut topo = Topology::new();
    // A unit cube built first so caller and store indices differ.
    let _spacer = make_unit_cube_manifold_at(&mut topo, 9.0, 9.0, 9.0);
    let sources = cubes(&mut topo, &[[0.0; 3], [2.0, 0.0, 0.0], [0.0, 3.0, 0.0]]);

    let store = GfaShapeStoreN::with_caller_maps(&topo, &sources).unwrap();
    let maps = store.caller.as_ref().unwrap();
    assert_eq!(
        (maps.faces.len(), maps.edges.len(), maps.vertices.len()),
        (18, 36, 24)
    );
    for (position, &store_solid) in store.sources.iter().enumerate() {
        let caller_faces = face_set(&topo, sources[position]);
        for face in solid_faces(&store.topo, store_solid).unwrap() {
            assert_eq!(store.face_source[&face], position);
            let caller = store.caller_face(face.index()).unwrap();
            assert!(caller_faces.contains(&caller));
            let caller_id = solid_faces(&topo, sources[position])
                .unwrap()
                .into_iter()
                .find(|f| f.index() == caller)
                .unwrap();
            assert_eq!(plane(&store.topo, face), plane(&topo, caller_id));
        }
        let caller_ends = edge_ends(&topo, &[sources[position]]);
        for (index, ends) in edge_ends(&store.topo, &[store_solid]) {
            assert_eq!(caller_ends[&store.caller_edge(index).unwrap()], ends);
        }
        let caller_points = vertex_points(&topo, &[sources[position]]);
        for (index, point) in vertex_points(&store.topo, &[store_solid]) {
            assert_eq!(caller_points[&store.caller_vertex(index).unwrap()], point);
        }
    }
    // An index that is no input entity of the store has no caller.
    assert_eq!(store.caller_face(10_000), None);
    assert_eq!(store.caller_edge(10_000), None);
    assert_eq!(store.caller_vertex(10_000), None);

    let bare = GfaShapeStoreN::new(&topo, &sources).unwrap();
    assert!(bare.caller.is_none());
    let any_face = solid_faces(&bare.topo, bare.sources[0]).unwrap()[0];
    assert_eq!(bare.caller_face(any_face.index()), None);
    assert_eq!(bare.caller_edge(0), None);
    assert_eq!(bare.caller_vertex(0), None);
    assert!(GfaShapeStoreN::with_caller_maps(&topo, &[]).is_err());
}

/// The two-operand store answers through the same trait, so the shared
/// export reads it unchanged.
#[test]
fn the_two_operand_store_answers_the_same_questions() {
    let mut topo = Topology::new();
    let _spacer = make_unit_cube_manifold_at(&mut topo, 9.0, 9.0, 9.0);
    let sources = cubes(&mut topo, &[[0.0; 3], [2.0, 0.0, 0.0]]);
    let store = crate::ds::GfaShapeStore::new(&topo, sources[0], sources[1]).unwrap();
    for (store_solid, caller_solid) in [(store.solid_a, sources[0]), (store.solid_b, sources[1])] {
        let caller_faces = face_set(&topo, caller_solid);
        for face in solid_faces(&store.topo, store_solid).unwrap() {
            assert!(caller_faces.contains(&store.caller_face(face.index()).unwrap()));
        }
        let caller_ends = edge_ends(&topo, &[caller_solid]);
        for (index, ends) in edge_ends(&store.topo, &[store_solid]) {
            assert_eq!(caller_ends[&store.caller_edge(index).unwrap()], ends);
        }
        let caller_points = vertex_points(&topo, &[caller_solid]);
        for (index, point) in vertex_points(&store.topo, &[store_solid]) {
            assert_eq!(caller_points[&store.caller_vertex(index).unwrap()], point);
        }
    }
    assert_eq!(store.caller_face(10_000), None);
    assert_eq!(store.caller_edge(10_000), None);
    assert_eq!(store.caller_vertex(10_000), None);
}

fn counts(topo: &Topology) -> [usize; 5] {
    [
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_solids(),
    ]
}

fn refusal(result: Result<NaryFuseRegions, AlgoError>) -> NaryFuseRefusal {
    match result {
        Err(AlgoError::UnsupportedNaryFuse { cause, .. }) => cause,
        other => panic!("expected a typed N-ary refusal, got {other:?}"),
    }
}

#[test]
fn out_of_domain_source_lists_are_refused_typed_and_change_nothing() {
    let context = OperationContext::new();
    let mut topo = Topology::new();
    let sources = cubes(&mut topo, &[[0.0; 3], [0.5; 3]]);
    let before = counts(&topo);

    // Fewer than two sources.
    for list in [&sources[..0], &sources[..1]] {
        assert_eq!(
            refusal(fuse_n_regions_with_entity_evolution(
                &mut topo,
                list,
                None,
                &context,
                &mut |_, _, _| Ok(())
            )),
            NaryFuseRefusal::TooFewOperands
        );
    }

    // A malformed pair list is an error before anything is built.
    assert!(
        fuse_n_regions_with_entity_evolution(
            &mut topo,
            &sources,
            Some(&[(1, 0)]),
            &context,
            &mut |_, _, _| Ok(())
        )
        .is_err()
    );
    assert_eq!(counts(&topo), before);

    // A curved face, then a curved edge, on an otherwise fine source.
    let curved_face = make_unit_cube_manifold_at(&mut topo, 0.25, 0.25, 0.25);
    let face = solid_faces(&topo, curved_face).unwrap()[0];
    let cylinder = remus_math::surfaces::CylindricalSurface::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        1.0,
    )
    .unwrap();
    topo.face_mut(face)
        .unwrap()
        .set_surface(FaceSurface::Cylinder(cylinder));
    let curved_edge = make_unit_cube_manifold_at(&mut topo, 0.75, 0.75, 0.75);
    let edge = solid_edges(&topo, curved_edge).unwrap()[0];
    let circle = remus_math::curves::Circle3D::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        1.0,
    )
    .unwrap();
    topo.edge_mut(edge)
        .unwrap()
        .set_curve(remus_topology::edge::EdgeCurve::Circle(circle));
    let before = counts(&topo);
    for curved in [curved_face, curved_edge] {
        // Refused wherever the curved source sits in the list.
        for list in [[sources[0], curved], [curved, sources[0]]] {
            assert_eq!(
                refusal(fuse_n_regions_with_entity_evolution(
                    &mut topo,
                    &list,
                    None,
                    &context,
                    &mut |_, _, _| Ok(())
                )),
                NaryFuseRefusal::CurvedCluster
            );
        }
    }
    assert_eq!(counts(&topo), before);
}

/// 91 sources have 4095 pairs and 92 have 4186; the budget is 4096. The
/// all-pairs count and an explicit list are held to the same limit, and the
/// refusal comes before any arrangement is built.
#[test]
fn the_source_pair_budget_is_enforced_at_its_edge() {
    let context = OperationContext::new();
    let mut topo = Topology::new();
    let origins: Vec<[f64; 3]> = (0..92).map(|i| [3.0 * f64::from(i), 0.0, 0.0]).collect();
    let sources = cubes(&mut topo, &origins);
    let before = counts(&topo);

    assert_eq!(
        refusal(fuse_n_regions_with_entity_evolution(
            &mut topo,
            &sources,
            None,
            &context,
            &mut |_, _, _| Ok(())
        )),
        NaryFuseRefusal::PairBudgetExceeded
    );
    let all_pairs: Vec<(usize, usize)> = (0..92)
        .flat_map(|i| ((i + 1)..92).map(move |j| (i, j)))
        .collect();
    assert_eq!(all_pairs.len(), 4186);
    assert_eq!(
        refusal(fuse_n_regions_with_entity_evolution(
            &mut topo,
            &sources,
            Some(&all_pairs[..=MAX_NWAY_SOURCE_PAIRS]),
            &context,
            &mut |_, _, _| Ok(())
        )),
        NaryFuseRefusal::PairBudgetExceeded
    );
    assert_eq!(counts(&topo), before);

    // Exactly at the budget the list is accepted: 92 separate cubes, each
    // its own region.
    let fused = fuse_n_regions_with_entity_evolution(
        &mut topo,
        &sources,
        Some(&all_pairs[..MAX_NWAY_SOURCE_PAIRS]),
        &context,
        &mut |_, _, _| Ok(()),
    )
    .unwrap();
    assert_eq!(fused.regions.len(), 92);
    assert!(fused.sources.iter().all(|tally| *tally
        == SourceFaceTally {
            kept: 6,
            inside: 0,
            coincident: 0
        }));
}

/// `fuse_n` shares the builder's front half with the region fuse. It still
/// returns the same single solid.
#[test]
fn the_single_solid_fuse_is_unchanged_by_the_shared_selection() {
    let mut topo = Topology::new();
    let sources = cubes(
        &mut topo,
        &[[0.0, 0.0, 0.0], [0.5, 0.25, 0.125], [0.25, 0.5, 0.375]],
    );
    let solid = fuse_n(&mut topo, &sources).unwrap();
    assert!((polyhedron_volume(&topo, solid) - 2.171_875).abs() <= EPS);
    let regions = fuse(&mut topo, &sources);
    assert_eq!(
        solid_faces(&topo, solid).unwrap().len(),
        solid_faces(&topo, regions.regions[0].solid).unwrap().len()
    );
}

/// The gate sees the result before the caller's arena does. It is handed the
/// store copies of the sources in order and the store regions; refusing
/// returns its error and allocates nothing, accepting exports.
#[test]
fn the_gate_runs_in_the_store_before_anything_is_exported() {
    let context = OperationContext::new();
    let mut topo = Topology::new();
    let sources = cubes(&mut topo, &[[0.0; 3], [5.0; 3], [0.5; 3]]);
    let before = (counts(&topo), topo.allocated_slot_count());

    let mut seen = Vec::new();
    let refused = fuse_n_regions_with_entity_evolution(
        &mut topo,
        &sources,
        None,
        &context,
        &mut |store, store_sources, regions| {
            // Each store source is the cube it was copied from.
            let origins: Vec<Point3> = store_sources
                .iter()
                .map(|&solid| {
                    let points = vertex_points(store, &[solid]);
                    points.values().fold(Point3::new(9.0, 9.0, 9.0), |low, p| {
                        Point3::new(low.x().min(p.x()), low.y().min(p.y()), low.z().min(p.z()))
                    })
                })
                .collect();
            let volumes: Vec<f64> = regions
                .iter()
                .map(|&region| polyhedron_volume(store, region))
                .collect();
            seen.push((origins, volumes));
            Err(AlgoError::UnsupportedNaryFuse {
                cause: NaryFuseRefusal::ResultUnqualified,
                reason: "the gate said no".into(),
            })
        },
    );
    assert_eq!(refusal(refused), NaryFuseRefusal::ResultUnqualified);
    assert_eq!((counts(&topo), topo.allocated_slot_count()), before);

    assert_eq!(seen.len(), 1, "the gate runs exactly once");
    let (origins, mut volumes) = seen.remove(0);
    assert_eq!(
        origins,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 5.0, 5.0),
            Point3::new(0.5, 0.5, 0.5)
        ]
    );
    volumes.sort_by(f64::total_cmp);
    assert_eq!(volumes.len(), 2);
    assert!((volumes[0] - 1.0).abs() <= EPS && (volumes[1] - 1.875).abs() <= EPS);

    // An accepting gate is called once too, and the result is exported.
    let mut calls = 0;
    let fused = fuse_n_regions_with_entity_evolution(
        &mut topo,
        &sources,
        None,
        &context,
        &mut |_, _, _| {
            calls += 1;
            Ok(())
        },
    )
    .unwrap();
    assert_eq!((calls, fused.regions.len()), (1, 2));
    assert!(topo.allocated_slot_count() > before.1);
}

#[test]
fn prepared_regions_leave_caller_slots_untouched_until_export() {
    let mut topo = Topology::new();
    let sources = cubes(&mut topo, &[[0.0; 3], [5.0; 3], [0.5; 3]]);
    let before = format!("{:?}", topo.to_image());
    let mut eager_topo = topo.clone();
    let eager = fuse(&mut eager_topo, &sources);
    let prepare = || {
        prepare_fuse_n_regions_with_entity_evolution(
            &topo,
            &sources,
            None,
            &OperationContext::new(),
            &mut |_, _, _| Ok(()),
        )
        .unwrap()
    };
    let dropped = prepare();
    assert_eq!(format!("{:?}", topo.to_image()), before);
    drop(dropped);
    assert_eq!(format!("{:?}", topo.to_image()), before);

    let prepared = prepare();
    let face_sources = prepared.face_sources().to_vec();
    let exported = prepared.export(&mut topo).unwrap();
    assert_eq!(exported, eager);
    assert_eq!(
        format!("{:?}", topo.to_image()),
        format!("{:?}", eager_topo.to_image())
    );
    for (sources, region) in face_sources.iter().zip(&exported.regions) {
        assert_eq!(sources.len(), region.evolution.faces.len());
        for (&source, &(_, exported_source)) in sources.iter().zip(&region.evolution.faces) {
            assert_eq!(exported_source, Some(source));
        }
    }
}
