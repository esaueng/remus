//! Top-level GFA orchestrator.
//!
//! Runs the complete General Fuse Algorithm pipeline:
//! PaveFiller -> Builder -> BOP -> assemble.

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
    boolean_with_tolerance(topo, op, solid_a, solid_b, Tolerance::default())
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
    let mut arena = GfaArena::new();
    pave_filler::run_pave_filler(
        &mut store.topo,
        store.solid_a,
        store.solid_b,
        tol,
        &mut arena,
    )?;

    // Stage 2: Builder — face splitting + classification
    let mut builder = Builder::with_tolerance(
        std::mem::take(&mut store.topo),
        arena,
        store.solid_a,
        store.solid_b,
        tol,
    );
    builder.perform()?;

    // Stage 3: BOP selection + assembly
    let (store_topo, store_result) = builder.build_result(op)?;
    store.topo = store_topo;

    // Export result solid back to the caller's topology
    let result = store.export_solid(topo, store_result)?;

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
            let tol = Tolerance::default();
            let mut store = GfaShapeStoreN::new(topo, sources)?;

            let mut arena = GfaArena::new();
            pave_filler::run_pave_filler_n(&mut store.topo, &store.sources, tol, &mut arena)?;

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

    let (store_topo, store_result, store_origins) = builder.build_result_with_origins(op)?;
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
