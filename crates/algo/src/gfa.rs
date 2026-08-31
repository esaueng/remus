//! Top-level GFA orchestrator.
//!
//! Runs the complete General Fuse Algorithm pipeline:
//! PaveFiller -> Builder -> BOP -> assemble.

use remus_math::context::OperationContext;
use remus_math::tolerance::Tolerance;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

use crate::bop::BooleanOp;
use crate::builder::Builder;
use crate::ds::{GfaArena, GfaShapeStoreN};
use crate::error::AlgoError;
use crate::pave_filler;

/// Result-face provenance in caller-topology face indices:
/// `(result face index, Some(input face index) | None)`.
pub type FaceOriginIndices = Vec<(usize, Option<usize>)>;

/// Run the complete GFA boolean operation with default tolerance.
///
/// This is the single entry point for boolean operations via the GFA.
///
/// # Errors
///
/// Returns [`AlgoError`] if any stage fails.
pub fn boolean(
    topo: &mut Topology,
    op: BooleanOp,
    solid_a: SolidId,
    solid_b: SolidId,
) -> Result<SolidId, AlgoError> {
    // Identical-solid fast path: avoid the full GFA pipeline when both
    // operands are the same topology entity.
    if solid_a == solid_b {
        return match op {
            BooleanOp::Fuse | BooleanOp::Intersect => {
                // A ∪ A = A, A ∩ A = A — return the original solid.
                // The caller (operations crate) copies if needed.
                Ok(solid_a)
            }
            BooleanOp::Cut => {
                // A \ A = empty
                Err(AlgoError::AssemblyFailed(
                    "Cut of identical solids produces empty result".into(),
                ))
            }
        };
    }
    boolean_with_context(topo, op, solid_a, solid_b, &OperationContext::new())
}

/// Run the complete GFA boolean operation under an explicit
/// [`OperationContext`].
///
/// This is the context-carrying entry point the pipeline converges on. The
/// context's tolerance drives every GFA stage, and its work budgets bound
/// NURBS surface-surface marching in the pave filler. The default context
/// reproduces [`boolean_with_tolerance`] with the default tolerance exactly;
/// like that function, this runs the full pipeline even
/// for identical operands ([`boolean`] carries the identical-solid fast
/// path).
///
/// # Errors
///
/// Returns [`AlgoError`] if any stage fails.
pub fn boolean_with_context(
    topo: &mut Topology,
    op: BooleanOp,
    solid_a: SolidId,
    solid_b: SolidId,
    context: &OperationContext,
) -> Result<SolidId, AlgoError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        boolean_with_context_impl(topo, op, solid_a, solid_b, context)
    })
}

/// Run the complete GFA boolean operation with custom tolerance.
///
/// Stages:
/// 1. **PaveFiller** — intersect shapes, build pave blocks
/// 2. **Builder** — split faces, classify sub-faces
/// 3. **BOP + assemble** — select faces, build result solid
///
/// # Errors
///
/// Returns [`AlgoError`] if any stage fails.
pub fn boolean_with_tolerance(
    topo: &mut Topology,
    op: BooleanOp,
    solid_a: SolidId,
    solid_b: SolidId,
    tol: Tolerance,
) -> Result<SolidId, AlgoError> {
    let context = OperationContext::new().with_tolerance(tol);
    boolean_with_context(topo, op, solid_a, solid_b, &context)
}

fn boolean_with_context_impl(
    topo: &mut Topology,
    op: BooleanOp,
    solid_a: SolidId,
    solid_b: SolidId,
    context: &OperationContext,
) -> Result<SolidId, AlgoError> {
    context.check_cancelled()?;
    let tol = context.tolerance;
    // Refuse unsupported curve types up front, by name. The pave filler,
    // face splitter and classifier all lack hyperbola/parabola support;
    // letting such an input through would make them fall back to a chord
    // or a straight line and return a plausible but geometrically wrong
    // solid. Failing closed here is the only honest option until the
    // intersection phases learn these conics.
    reject_unsupported_curves(topo, solid_a)?;
    reject_unsupported_curves(topo, solid_b)?;

    // Create an isolated shape store with deep-copied input solids.
    // The GFA pipeline operates entirely within the store, avoiding
    // vertex/edge identity conflicts with the caller's topology.
    let mut store = crate::ds::GfaShapeStore::new(topo, solid_a, solid_b)?;

    // Stage 1: PaveFiller — intersection + pave block construction
    context.check_cancelled()?;
    let mut arena = GfaArena::new();
    pave_filler::run_pave_filler_with_context(
        &mut store.topo,
        store.solid_a,
        store.solid_b,
        context,
        &mut arena,
    )?;

    // Stage 2: Builder — face splitting + classification
    context.check_cancelled()?;
    let mut builder = Builder::with_tolerance(
        std::mem::take(&mut store.topo),
        arena,
        store.solid_a,
        store.solid_b,
        tol,
    );
    builder.perform()?;

    // Stage 3: BOP selection + assembly
    context.check_cancelled()?;
    let (store_topo, store_result) = builder.build_result(op)?;
    store.topo = store_topo;

    // Export result solid back to the caller's topology
    context.check_cancelled()?;
    let result = store.export_solid(topo, store_result)?;

    context.check_cancelled()?;

    Ok(result)
}

/// Fuse **N** solids into one via a single GFA arrangement.
///
/// One pass over all operands instead of the sequential pairwise fuse's
/// re-processing of a growing accumulator (O(n²)). Runs the N-way pave filler
/// (pairwise intersection into one arena, spatially pruned) then the N-way
/// builder (split once, classify each sub-face against all other sources, keep
/// the union boundary, resolve coincident faces).
///
/// Fuse-only, and currently limited to the planar-coincidence cases the N-way
/// builder handles — it returns an error (for the caller to fall back to the
/// sequential fuse) on a non-planar coincident contact or an unresolved
/// coincidence. `sources` must be non-empty; a single source returns a copy.
///
/// # Errors
///
/// Returns [`AlgoError`] if `sources` is empty or any GFA stage fails.
pub fn fuse_n(topo: &mut Topology, sources: &[SolidId]) -> Result<SolidId, AlgoError> {
    fuse_n_with_context(topo, sources, &OperationContext::new())
}

/// Fuse **N** solids under an explicit [`OperationContext`].
///
/// # Errors
///
/// Returns [`AlgoError`] if `sources` is empty or any GFA stage fails.
pub fn fuse_n_with_context(
    topo: &mut Topology,
    sources: &[SolidId],
    context: &OperationContext,
) -> Result<SolidId, AlgoError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        fuse_n_with_context_impl(topo, sources, context)
    })
}

fn fuse_n_with_context_impl(
    topo: &mut Topology,
    sources: &[SolidId],
    context: &OperationContext,
) -> Result<SolidId, AlgoError> {
    // Same up-front refusal as `boolean_with_tolerance`; see that function.
    for &sid in sources {
        reject_unsupported_curves(topo, sid)?;
    }
    match sources {
        [] => Err(AlgoError::AssemblyFailed(
            "fuse_n needs at least one source solid".into(),
        )),
        [only] => {
            // A copy so the caller owns a distinct result, mirroring the
            // store round-trip the multi-source path performs.
            let store = GfaShapeStoreN::new(topo, &[*only])?;
            store.export_solid(topo, store.sources[0])
        }
        _ => {
            let tol = context.tolerance;
            let mut store = GfaShapeStoreN::new(topo, sources)?;

            let mut arena = GfaArena::new();
            pave_filler::run_pave_filler_n_with_context(
                &mut store.topo,
                &store.sources,
                context,
                &mut arena,
            )?;

            let (store_topo, store_result) = crate::builder::build_fuse_n(
                std::mem::take(&mut store.topo),
                arena,
                &store.sources,
                &store.face_source,
                tol,
            )?;
            store.topo = store_topo;

            store.export_solid(topo, store_result)
        }
    }
}

/// Run a GFA boolean, also returning each result face's provenance.
///
/// Provenance is `(result face index, Some(input face index) | None)` in the
/// **caller's** topology — the basis for faithful shape-evolution tracking.
/// Indices are `FaceId::index()` values (matching the convention the evolution
/// map uses). `None` marks a synthesised face with no input origin. Unlike
/// [`boolean`], there is no identical-operand fast path; callers handle `A == B`.
///
/// # Errors
///
/// Returns [`AlgoError`] if any GFA stage fails.
pub fn boolean_with_face_origins(
    topo: &mut Topology,
    op: BooleanOp,
    solid_a: SolidId,
    solid_b: SolidId,
) -> Result<(SolidId, FaceOriginIndices), AlgoError> {
    // Same up-front refusal as `boolean_with_tolerance`; see that function.
    reject_unsupported_curves(topo, solid_a)?;
    reject_unsupported_curves(topo, solid_b)?;

    let tol = Tolerance::default();
    let mut store = crate::ds::GfaShapeStore::new(topo, solid_a, solid_b)?;

    let mut arena = GfaArena::new();
    pave_filler::run_pave_filler(
        &mut store.topo,
        store.solid_a,
        store.solid_b,
        tol,
        &mut arena,
    )?;

    let mut builder = Builder::with_tolerance(
        std::mem::take(&mut store.topo),
        arena,
        store.solid_a,
        store.solid_b,
        tol,
    );
    builder.perform()?;

    let (store_topo, store_result, store_origins, _lineage) =
        builder.build_result_with_origins(op)?;
    store.topo = store_topo;

    let (result, export_map) = store.export_solid_with_face_map(topo, store_result)?;

    // Translate store-space provenance to caller-space face indices. The export
    // map is total over result faces, so a miss is a real provenance desync and
    // is surfaced as an error rather than silently dropped (which would mark a
    // real input as `deleted`). A missing input source is `None` by design — a
    // synthesised face with no input origin.
    let mut origins = Vec::with_capacity(store_origins.len());
    for (store_out, store_src) in store_origins {
        let caller_out = export_map
            .get(&store_out.index())
            .ok_or_else(|| {
                AlgoError::AssemblyFailed(
                    "result face missing from export map (provenance desync)".into(),
                )
            })?
            .index();
        let caller_src =
            store_src.and_then(|s| store.input_face_to_caller.get(&s.index()).copied());
        origins.push((caller_out, caller_src));
    }

    Ok((result, origins))
}

/// Refuse a solid whose edges carry a curve type the GFA pipeline cannot
/// handle, naming the variant.
///
/// The pave filler's EE/EF/FF phases, the face splitter's split searches
/// and the analytic classifier all enumerate `EdgeCurve` explicitly and
/// have no hyperbola or parabola support. Without this gate those sites
/// would take their line/chord fallbacks and produce a result that looks
/// like a solid but has the wrong geometry — the failure mode that is
/// hardest to notice downstream. Refusing here fails closed instead.
fn reject_unsupported_curves(topo: &Topology, solid_id: SolidId) -> Result<(), AlgoError> {
    use remus_topology::edge::EdgeCurve;
    use remus_topology::explorer::solid_faces;

    for fid in solid_faces(topo, solid_id)? {
        let face = topo.face(fid)?;
        for &wid in std::iter::once(&face.outer_wire()).chain(face.inner_wires()) {
            let wire = topo.wire(wid)?;
            for oe in wire.edges() {
                let curve = topo.edge(oe.edge())?.curve();
                match curve {
                    EdgeCurve::Line
                    | EdgeCurve::Circle(_)
                    | EdgeCurve::Ellipse(_)
                    | EdgeCurve::NurbsCurve(_) => {}
                    EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
                        return Err(AlgoError::UnsupportedCurve {
                            variant: curve.type_tag(),
                        });
                    }
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod context_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use remus_topology::test_utils::make_unit_cube_manifold_at;

    use super::*;

    #[test]
    fn boolean_with_default_context_matches_boolean() {
        // Two overlapping unit cubes, fused twice: once through the legacy
        // entry point, once through the context entry point with the default
        // context. Both results must have identical face counts (the
        // pipelines must be the same code path, not merely similar).
        let mut topo_a = Topology::new();
        let a1 = make_unit_cube_manifold_at(&mut topo_a, 0.0, 0.0, 0.0);
        let a2 = make_unit_cube_manifold_at(&mut topo_a, 0.5, 0.5, 0.5);
        let legacy = boolean(&mut topo_a, BooleanOp::Fuse, a1, a2).unwrap();

        let mut topo_b = Topology::new();
        let b1 = make_unit_cube_manifold_at(&mut topo_b, 0.0, 0.0, 0.0);
        let b2 = make_unit_cube_manifold_at(&mut topo_b, 0.5, 0.5, 0.5);
        let ctx = boolean_with_context(
            &mut topo_b,
            BooleanOp::Fuse,
            b1,
            b2,
            &OperationContext::new(),
        )
        .unwrap();

        let faces_legacy = remus_topology::explorer::solid_faces(&topo_a, legacy)
            .unwrap()
            .len();
        let faces_ctx = remus_topology::explorer::solid_faces(&topo_b, ctx)
            .unwrap()
            .len();
        assert_eq!(faces_legacy, faces_ctx);
    }

    #[test]
    fn boolean_with_context_carries_tolerance() {
        // Cubes separated by 5e-5 are distinct under the 1e-7 default but
        // coincident at the loose context's 1e-4 linear tolerance. This runs
        // GFA directly (no operations-layer disjoint shortcut): the default
        // keeps two closed regions, while the loose run treats the gap as a
        // coincident interface and refuses the resulting open growth shell.
        // That outcome change proves the caller tolerance reached the pave
        // filler and builder rather than being replaced with a default.
        let gap = 5e-5;

        let mut topo_a = Topology::new();
        let a1 = make_unit_cube_manifold_at(&mut topo_a, 0.0, 0.0, 0.0);
        let a2 = make_unit_cube_manifold_at(&mut topo_a, 1.0 + gap, 0.0, 0.0);
        let default = boolean_with_context(
            &mut topo_a,
            BooleanOp::Fuse,
            a1,
            a2,
            &OperationContext::new(),
        )
        .unwrap();

        let mut topo_b = Topology::new();
        let b1 = make_unit_cube_manifold_at(&mut topo_b, 0.0, 0.0, 0.0);
        let b2 = make_unit_cube_manifold_at(&mut topo_b, 1.0 + gap, 0.0, 0.0);
        let loose =
            OperationContext::new().with_tolerance(remus_math::tolerance::Tolerance::loose());
        let context_result = boolean_with_context(&mut topo_b, BooleanOp::Fuse, b1, b2, &loose);

        let default_faces = remus_topology::explorer::solid_faces(&topo_a, default)
            .unwrap()
            .len();
        assert_eq!(
            default_faces, 12,
            "default tolerance keeps both cube boundaries"
        );
        assert!(
            context_result.is_err(),
            "loose tolerance must materially change the sub-tolerance gap classification"
        );
    }
}

/// One result edge's construction-derived history event (Issue 12).
///
/// Claims follow the evolution discipline: `Preserved`/`Modified` bind a
/// result edge to a caller input edge and are made only from construction
/// records (copy maps and pave blocks); `Generated` names the caller faces
/// whose intersection created the edge; `Unresolved` is the honest bucket
/// for edges the builder rebuilt without construction records — never
/// guessed from geometry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EdgeEvent {
    /// The result edge IS a caller input edge, carried through unchanged
    /// (caller edge index).
    Preserved(usize),
    /// The result edge is a piece of a caller input edge, split by the pave
    /// filler (caller edge index of the parent).
    Modified(usize),
    /// The result edge was generated by a face–face intersection; the
    /// caller face indices of the generating pair (an entry is `None` when
    /// that face was itself synthesised in the store).
    Generated {
        /// Caller face index of the first generating face, when it maps.
        face_a: Option<usize>,
        /// Caller face index of the second generating face, when it maps.
        face_b: Option<usize>,
    },
    /// No construction record reaches this edge (builder rebuild paths are
    /// not yet recorded).
    Unresolved,
}

/// One result vertex's construction-derived history event (Issue 12).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VertexEvent {
    /// The result vertex IS a caller input vertex (caller vertex index).
    Preserved(usize),
    /// The vertex did not exist in either input: the pipeline created it.
    /// (Existence is construction-derived from the copy maps; the specific
    /// generating interference is not yet recorded.)
    Created,
}

/// Construction-derived vertex/edge/face history of one GFA boolean, in
/// caller-space indices.
#[derive(Debug, Clone)]
pub struct EntityEvolution {
    /// Result-face provenance, as in [`boolean_with_face_origins`].
    pub faces: FaceOriginIndices,
    /// Result edge index → its event. Total over the result's edges.
    pub edges: Vec<(usize, EdgeEvent)>,
    /// Result vertex index → its event. Total over the result's vertices.
    pub vertices: Vec<(usize, VertexEvent)>,
}

/// Run a GFA boolean, returning construction-derived vertex, edge, and face
/// history alongside the result (Issue 12).
///
/// Lineage sources, all construction records: the store copy maps
/// (input identity across the isolation copy), the pave filler's pave
/// blocks (`original edge → split edge`), the FF phase's section-edge
/// origin table (`section edge → generating face pair`), and the export
/// maps. Result entities no record reaches are reported
/// [`EdgeEvent::Unresolved`] — never bound by geometric guessing.
///
/// Like [`boolean_with_face_origins`], there is no identical-operand fast
/// path; callers handle `A == B`.
///
/// # Errors
///
/// Returns [`AlgoError`] if any GFA stage fails.
#[allow(clippy::too_many_lines)]
pub fn boolean_with_entity_evolution(
    topo: &mut Topology,
    op: BooleanOp,
    solid_a: SolidId,
    solid_b: SolidId,
) -> Result<(SolidId, EntityEvolution), AlgoError> {
    use std::collections::HashMap;

    reject_unsupported_curves(topo, solid_a)?;
    reject_unsupported_curves(topo, solid_b)?;

    let tol = Tolerance::default();
    let mut store = crate::ds::GfaShapeStore::new(topo, solid_a, solid_b)?;

    let mut arena = GfaArena::new();
    pave_filler::run_pave_filler(
        &mut store.topo,
        store.solid_a,
        store.solid_b,
        tol,
        &mut arena,
    )?;

    // Snapshot construction lineage before the builder consumes the arena.
    // split edge (store idx) → original edge (store idx), and pave block
    // index → its original edge (the link wire-edge materialization records).
    let mut split_parent: HashMap<usize, usize> = HashMap::new();
    let mut pb_original: HashMap<usize, usize> = HashMap::new();
    for (pb_id, pb) in arena.pave_blocks.iter() {
        pb_original.insert(pb_id.index(), pb.original_edge.index());
        if let Some(split) = pb.split_edge {
            split_parent.insert(split.index(), pb.original_edge.index());
        }
    }
    for (_, cb) in arena.common_blocks.iter() {
        if let (Some(split), Some(&first_pb)) = (cb.split_edge, cb.pave_blocks.first())
            && let Some(pb) = arena.pave_blocks.get(first_pb)
        {
            split_parent.insert(split.index(), pb.original_edge.index());
        }
    }
    let section_origins = arena.section_edge_origins.clone();

    let mut builder = Builder::with_tolerance(
        std::mem::take(&mut store.topo),
        arena,
        store.solid_a,
        store.solid_b,
        tol,
    );
    builder.perform()?;

    // The returned log includes both the perform-phase records and the
    // assembly-rebuild records (weld and collinear splits), so result
    // edges rebuilt during assembly chase back to their parents.
    let (store_topo, store_result, store_origins, lineage) =
        builder.build_result_with_origins(op)?;
    store.topo = store_topo;

    let (result, export) = store.export_solid_with_entity_maps(topo, store_result)?;

    // Faces: translate exactly as boolean_with_face_origins does.
    let mut faces = Vec::with_capacity(store_origins.len());
    for (store_out, store_src) in store_origins {
        let caller_out = export
            .faces
            .get(&store_out.index())
            .ok_or_else(|| {
                AlgoError::AssemblyFailed(
                    "result face missing from export map (provenance desync)".into(),
                )
            })?
            .index();
        let caller_src =
            store_src.and_then(|s| store.input_face_to_caller.get(&s.index()).copied());
        faces.push((caller_out, caller_src));
    }

    // Resolve one store edge index through the construction records.
    // Chases split chains (a split of a split, and splits OF section
    // edges) with a hard bound so a malformed record cannot loop.
    let resolve_edge = |store_idx: usize| -> EdgeEvent {
        let mut current = store_idx;
        let mut transformed = false;
        for _ in 0..64 {
            if let Some(&caller) = store.input_edge_to_caller.get(&current) {
                return if transformed {
                    EdgeEvent::Modified(caller)
                } else {
                    EdgeEvent::Preserved(caller)
                };
            }
            if let Some(&(fa, fb)) = section_origins.get(&current) {
                return EdgeEvent::Generated {
                    face_a: store.input_face_to_caller.get(&fa).copied(),
                    face_b: store.input_face_to_caller.get(&fb).copied(),
                };
            }
            if let Some(&old) = lineage.rewrites.get(&current) {
                current = old;
                transformed = true;
                continue;
            }
            if let Some(&pb) = lineage.to_pave_block.get(&current) {
                if let Some(&original) = pb_original.get(&pb) {
                    current = original;
                    transformed = true;
                    continue;
                }
                break;
            }
            if let Some(&parent) = split_parent.get(&current) {
                current = parent;
                transformed = true;
                continue;
            }
            break;
        }
        EdgeEvent::Unresolved
    };

    // The export edge map is store idx → caller edge: total over the
    // result's edges by construction, so inverting it enumerates exactly
    // the result's edge set.
    let mut edges: Vec<(usize, EdgeEvent)> = export
        .edges
        .iter()
        .map(|(&store_idx, caller_edge)| (caller_edge.index(), resolve_edge(store_idx)))
        .collect();
    edges.sort_by_key(|(idx, _)| *idx);

    let mut vertices: Vec<(usize, VertexEvent)> = export
        .vertices
        .iter()
        .map(|(&store_idx, caller_vertex)| {
            let event = store
                .input_vertex_to_caller
                .get(&store_idx)
                .map_or(VertexEvent::Created, |&caller| {
                    VertexEvent::Preserved(caller)
                });
            (caller_vertex.index(), event)
        })
        .collect();
    vertices.sort_by_key(|(idx, _)| *idx);

    Ok((
        result,
        EntityEvolution {
            faces,
            edges,
            vertices,
        },
    ))
}

#[cfg(test)]
mod entity_evolution_tests {
    #![allow(clippy::unwrap_used)]

    use remus_topology::test_utils::make_unit_cube_manifold_at;

    use super::*;

    fn count_events(evolution: &EntityEvolution) -> (usize, usize, usize, usize) {
        let mut preserved = 0;
        let mut modified = 0;
        let mut generated = 0;
        let mut unresolved = 0;
        for (_, event) in &evolution.edges {
            match event {
                EdgeEvent::Preserved(_) => preserved += 1,
                EdgeEvent::Modified(_) => modified += 1,
                EdgeEvent::Generated { .. } => generated += 1,
                EdgeEvent::Unresolved => unresolved += 1,
            }
        }
        (preserved, modified, generated, unresolved)
    }

    #[test]
    fn cube_fuse_history_is_total_and_construction_derived() {
        let mut topo = Topology::new();
        let a = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);
        let b = make_unit_cube_manifold_at(&mut topo, 0.5, 0.5, 0.5);
        let (result, evolution) =
            boolean_with_entity_evolution(&mut topo, BooleanOp::Fuse, a, b).unwrap();

        // Totality: exactly the result's edge and vertex sets, each entity
        // once, every entity in some bucket.
        let faces = remus_topology::explorer::solid_faces(&topo, result).unwrap();
        let mut result_edges = std::collections::BTreeSet::new();
        let mut result_vertices = std::collections::BTreeSet::new();
        for &fid in &faces {
            let face = topo.face(fid).unwrap();
            for &wid in std::iter::once(&face.outer_wire()).chain(face.inner_wires().iter()) {
                for oe in topo.wire(wid).unwrap().edges() {
                    result_edges.insert(oe.edge().index());
                    let edge = topo.edge(oe.edge()).unwrap();
                    result_vertices.insert(edge.start().index());
                    result_vertices.insert(edge.end().index());
                }
            }
        }
        let mapped_edges: std::collections::BTreeSet<usize> =
            evolution.edges.iter().map(|(idx, _)| *idx).collect();
        let mapped_vertices: std::collections::BTreeSet<usize> =
            evolution.vertices.iter().map(|(idx, _)| *idx).collect();
        assert_eq!(mapped_edges, result_edges, "edge history must be total");
        assert_eq!(
            mapped_vertices, result_vertices,
            "vertex history must be total"
        );
        assert_eq!(evolution.edges.len(), mapped_edges.len(), "no duplicates");

        // The fuse of two offset cubes preserves far edges, splits crossing
        // edges, and generates section edges from crossing face pairs.
        let (preserved, modified, generated, unresolved) = count_events(&evolution);
        assert!(preserved > 0, "far cube edges must be preserved");
        assert!(modified > 0, "crossing cube edges must be split");
        assert!(
            generated > 0,
            "the intersection must generate section edges"
        );
        // Every Generated edge must name at least one caller face.
        for (_, event) in &evolution.edges {
            if let EdgeEvent::Generated { face_a, face_b } = event {
                assert!(
                    face_a.is_some() || face_b.is_some(),
                    "a section edge must name a generating input face"
                );
            }
        }
        // Every assembly-rebuild path records lineage (perform-phase wire
        // images, vertex-merge rebuilds, welds, collinear splits), so a cube
        // fuse's edge history is total construction fact: any regression
        // that reintroduces an unrecorded rebuild fails here.
        assert_eq!(
            unresolved, 0,
            "every result edge must chase to a construction record \
             (preserved={preserved} modified={modified} generated={generated})"
        );

        // Vertices: both preserved corners and created intersections exist.
        let created = evolution
            .vertices
            .iter()
            .filter(|(_, e)| matches!(e, VertexEvent::Created))
            .count();
        let kept = evolution.vertices.len() - created;
        assert!(kept > 0 && created > 0);

        // Determinism: an identical run reports identical history.
        let mut topo2 = Topology::new();
        let a2 = make_unit_cube_manifold_at(&mut topo2, 0.0, 0.0, 0.0);
        let b2 = make_unit_cube_manifold_at(&mut topo2, 0.5, 0.5, 0.5);
        let (_, evolution2) =
            boolean_with_entity_evolution(&mut topo2, BooleanOp::Fuse, a2, b2).unwrap();
        assert_eq!(evolution.edges, evolution2.edges);
        assert_eq!(evolution.vertices, evolution2.vertices);
    }
}
