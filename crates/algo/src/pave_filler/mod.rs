//! PaveFiller — intersection engine that builds pave blocks.
//!
//! Runs phases in two stages:
//!
//! **Stage 1 — Intersection** (reads `&Topology`, writes `&mut GfaArena`):
//! VV, VE, EE, VF, EF, FF.
//!
//! **Stage 2 — Resolution** (writes `&mut Topology` from arena data):
//! `MakeBlocks`, `MakeSplitEdges`, `MakePCurves`, `FillFaceInfo`.

pub mod fill_face_info;
pub mod force_interf_ee;
mod helpers;
pub mod link_existing;
pub mod make_blocks;
pub mod make_pcurves;
pub mod make_split_edges;
pub mod phase_ee;
pub mod phase_ef;
pub mod phase_ff;
pub mod phase_ff_coplanar;
pub mod phase_ve;
pub mod phase_vf;
pub mod phase_vv;

#[cfg(test)]
mod tests;

use std::collections::BTreeSet;

use remus_math::context::OperationContext;
use remus_math::tolerance::Tolerance;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::solid::SolidId;
use remus_topology::vertex::VertexId;

use crate::ds::GfaArena;
use crate::error::AlgoError;

/// PaveFiller intersects all shape pairs between two solids,
/// building pave blocks and populating the GFA arena.
pub struct PaveFiller<'a> {
    /// The topology containing both solids.
    topo: &'a mut Topology,
    /// Solid A (first boolean argument).
    solid_a: SolidId,
    /// Solid B (second boolean argument).
    solid_b: SolidId,
    /// Caller-visible tolerance and work budgets.
    context: OperationContext,
}

impl<'a> PaveFiller<'a> {
    /// Creates a new `PaveFiller` for two solids.
    #[allow(dead_code)]
    pub fn new(topo: &'a mut Topology, solid_a: SolidId, solid_b: SolidId) -> Self {
        Self {
            topo,
            solid_a,
            solid_b,
            context: OperationContext::new(),
        }
    }

    /// Creates a `PaveFiller` with custom tolerance.
    #[allow(dead_code)]
    pub fn with_tolerance(
        topo: &'a mut Topology,
        solid_a: SolidId,
        solid_b: SolidId,
        tol: Tolerance,
    ) -> Self {
        Self {
            topo,
            solid_a,
            solid_b,
            context: OperationContext::new().with_tolerance(tol),
        }
    }

    /// Creates a `PaveFiller` under an explicit operation context.
    pub fn with_context(
        topo: &'a mut Topology,
        solid_a: SolidId,
        solid_b: SolidId,
        context: OperationContext,
    ) -> Self {
        Self {
            topo,
            solid_a,
            solid_b,
            context,
        }
    }

    /// Run intersection phases (VV through FF), populating the GFA arena.
    ///
    /// Creates new vertices in `Topology` when intersection points have
    /// no nearby existing vertex (EE, EF, FF phases).
    /// Call [`run_pave_filler`] instead to run both stages.
    ///
    /// # Errors
    ///
    /// Returns [`AlgoError`] if any topology lookup or intersection fails.
    pub fn perform(&mut self, arena: &mut GfaArena) -> Result<(), AlgoError> {
        let tol = self.context.tolerance;
        self.context.check_cancelled()?;
        self.init_pave_blocks(arena)?;

        self.context.check_cancelled()?;
        phase_vv::perform(self.topo, self.solid_a, self.solid_b, tol, arena)?;
        // VV is the only phase that registers same-domain vertices, and
        // `edge_pave_blocks` is fixed at init — so the pave-vertex coincidence
        // index is stable for the remaining phases. Build it once here instead
        // of linear-scanning every pave block per intersection endpoint.
        arena.build_pave_vertex_index(self.topo, tol.linear);
        self.context.check_cancelled()?;
        phase_ve::perform(self.topo, self.solid_a, self.solid_b, tol, arena)?;
        self.context.check_cancelled()?;
        phase_ee::perform(self.topo, self.solid_a, self.solid_b, tol, arena)?;
        self.context.check_cancelled()?;
        phase_vf::perform(self.topo, self.solid_a, self.solid_b, tol, arena)?;
        self.context.check_cancelled()?;
        phase_ef::perform(self.topo, self.solid_a, self.solid_b, tol, arena)?;
        self.context.check_cancelled()?;
        phase_ff::perform_with_context(
            self.topo,
            self.solid_a,
            self.solid_b,
            &self.context,
            arena,
        )?;

        // Coplanar face splitting: parallel planes are skipped by Phase FF.
        self.context.check_cancelled()?;
        phase_ff_coplanar::perform(self.topo, self.solid_a, self.solid_b, tol, arena)?;

        Ok(())
    }

    /// Initialize pave blocks for all edges of both solids.
    fn init_pave_blocks(&self, arena: &mut GfaArena) -> Result<(), AlgoError> {
        let mut pending: Vec<(EdgeId, VertexId, f64, VertexId, f64)> = Vec::new();
        let mut seen: BTreeSet<EdgeId> = arena.edge_pave_blocks.keys().copied().collect();
        for &solid in &[self.solid_a, self.solid_b] {
            let edges = remus_topology::explorer::solid_edges(self.topo, solid)?;
            for edge_id in edges {
                // Skip if already initialized (shared edges between solids)
                if !seen.insert(edge_id) {
                    continue;
                }
                let edge = self.topo.edge(edge_id)?;
                self.topo.vertex(edge.start())?;
                self.topo.vertex(edge.end())?;
                let (t0, t1) =
                    helpers::authoritative_edge_domain(edge, edge_id, "pave-block initialization")?;
                pending.push((edge_id, edge.start(), t0, edge.end(), t1));
            }
        }
        for (edge_id, start, t0, end, t1) in pending {
            arena.init_edge_pave_block(edge_id, start, t0, end, t1);
        }
        Ok(())
    }
}

/// Run the complete PaveFiller pipeline (both stages).
///
/// **Stage 1 — Intersection** (reads `&Topology`):
/// Runs VV, VE, EE, VF, EF, FF phases to discover all interferences
/// and populate pave blocks with extra paves.
///
/// **Stage 2 — Resolution** (writes `&mut Topology`):
/// - `MakeBlocks` — splits pave blocks at extra paves
/// - `MakeSplitEdges` — creates new topology edges for leaf pave blocks
/// - `MakePCurves` — builds 2D curves on faces (stub)
/// - `FillFaceInfo` — classifies pave blocks as On/In/Sc per face
///
/// # Errors
///
/// Returns [`AlgoError`] if any topology lookup or intersection fails.
pub fn run_pave_filler(
    topo: &mut Topology,
    solid_a: SolidId,
    solid_b: SolidId,
    tol: Tolerance,
    arena: &mut GfaArena,
) -> Result<(), AlgoError> {
    let context = OperationContext::new().with_tolerance(tol);
    run_pave_filler_with_context(topo, solid_a, solid_b, &context, arena)
}

/// Run the complete PaveFiller pipeline under an explicit operation context.
///
/// # Errors
///
/// Returns [`AlgoError`] if any topology lookup or intersection fails.
pub fn run_pave_filler_with_context(
    topo: &mut Topology,
    solid_a: SolidId,
    solid_b: SolidId,
    context: &OperationContext,
    arena: &mut GfaArena,
) -> Result<(), AlgoError> {
    let tol = context.tolerance;
    // Stage 1: Intersection (may create new vertices for EE/EF/FF crossings)
    {
        let mut filler = PaveFiller::with_context(topo, solid_a, solid_b, context.clone());
        filler.perform(arena)?;
    }

    // Stage 2: Resolution (mutable Topology)
    context.check_cancelled()?;
    make_blocks::perform(arena)?;
    context.check_cancelled()?;
    force_interf_ee::perform(topo, tol, arena)?;
    context.check_cancelled()?;
    link_existing::perform(topo, tol, arena)?;
    context.check_cancelled()?;
    make_split_edges::perform(topo, arena)?;
    context.check_cancelled()?;
    make_pcurves::perform(topo, arena)?;
    context.check_cancelled()?;
    fill_face_info::perform_with_tolerance(topo, arena, tol)?;

    Ok(())
}

/// Run the PaveFiller pipeline over **N** source solids for an N-way fuse.
///
/// The Stage-1 intersection phases are inherently pairwise (each section is
/// between the faces of two solids) and, crucially, deposit only geometric
/// split data into the shared `arena` — they carry no `Rank`. So the two-solid
/// phase code is reused verbatim: run each phase for every spatially-interacting
/// source pair into ONE arena, and the paves/sections accumulate correctly. A
/// bbox broad-phase skips non-interacting pairs, keeping the stage O(n·k) for
/// the sparse interaction graphs a fused lattice produces rather than O(n²).
///
/// Phase order preserves the two-solid pipeline's invariant that the
/// pave-vertex coincidence index is built ONCE, after all VV coincidences are
/// registered and before the phases that query it: every pair's VV runs first,
/// then the index is built, then the remaining phases run per pair. Stage-2
/// resolution (which is already solid-agnostic — it reads the accumulated arena)
/// runs once.
///
/// For `sources.len() == 2` this is behaviourally identical to
/// [`run_pave_filler`]. Cut/Intersect are unaffected; this path is fuse-only.
///
/// # Errors
///
/// Returns [`AlgoError`] if `sources` is empty or any stage fails.
#[allow(dead_code)]
pub fn run_pave_filler_n(
    topo: &mut Topology,
    sources: &[SolidId],
    tol: Tolerance,
    arena: &mut GfaArena,
) -> Result<(), AlgoError> {
    let context = OperationContext::new().with_tolerance(tol);
    run_pave_filler_n_with_context(topo, sources, &context, arena)
}

/// Run the N-way PaveFiller pipeline under an explicit operation context.
///
/// # Errors
///
/// Returns [`AlgoError`] if `sources` is empty or any stage fails.
pub fn run_pave_filler_n_with_context(
    topo: &mut Topology,
    sources: &[SolidId],
    context: &OperationContext,
    arena: &mut GfaArena,
) -> Result<(), AlgoError> {
    const MAX_SOURCE_PAIRS: usize = 4_096;
    let tol = context.tolerance;
    context.check_cancelled()?;

    if sources.is_empty() {
        return Err(AlgoError::AssemblyFailed(
            "N-way pave filler needs at least one source solid".into(),
        ));
    }
    let pair_count = sources
        .len()
        .checked_mul(sources.len().saturating_sub(1))
        .and_then(|count| count.checked_div(2))
        .ok_or_else(|| {
            AlgoError::AssemblyFailed("N-way pave filler source count overflows".into())
        })?;
    if pair_count > MAX_SOURCE_PAIRS {
        return Err(AlgoError::AssemblyFailed(format!(
            "N-way pave filler needs {pair_count} source pairs; limit is {MAX_SOURCE_PAIRS}"
        )));
    }

    // Stage 1: Intersection over every source pair, accumulating into one arena.
    init_pave_blocks_n(topo, sources, arena)?;
    let pairs = source_pairs(sources);

    for &(i, j) in &pairs {
        context.check_cancelled()?;
        phase_vv::perform(topo, sources[i], sources[j], tol, arena)?;
    }
    // VV is the only phase that registers same-domain vertices and the edge
    // pave blocks are fixed at init, so the coincidence index is stable for the
    // remaining phases — build it once, after every pair's VV (mirrors the
    // two-solid `PaveFiller::perform`).
    arena.build_pave_vertex_index(topo, tol.linear);
    for &(i, j) in &pairs {
        context.check_cancelled()?;
        phase_ve::perform(topo, sources[i], sources[j], tol, arena)?;
    }
    for &(i, j) in &pairs {
        context.check_cancelled()?;
        phase_ee::perform(topo, sources[i], sources[j], tol, arena)?;
    }
    for &(i, j) in &pairs {
        context.check_cancelled()?;
        phase_vf::perform(topo, sources[i], sources[j], tol, arena)?;
    }
    for &(i, j) in &pairs {
        context.check_cancelled()?;
        phase_ef::perform(topo, sources[i], sources[j], tol, arena)?;
    }
    for &(i, j) in &pairs {
        context.check_cancelled()?;
        phase_ff::perform_with_context(topo, sources[i], sources[j], context, arena)?;
    }
    for &(i, j) in &pairs {
        context.check_cancelled()?;
        phase_ff_coplanar::perform(topo, sources[i], sources[j], tol, arena)?;
    }

    // Stage 2: Resolution (solid-agnostic — reads the accumulated arena).
    context.check_cancelled()?;
    make_blocks::perform(arena)?;
    context.check_cancelled()?;
    force_interf_ee::perform(topo, tol, arena)?;
    context.check_cancelled()?;
    link_existing::perform(topo, tol, arena)?;
    context.check_cancelled()?;
    make_split_edges::perform(topo, arena)?;
    context.check_cancelled()?;
    make_pcurves::perform(topo, arena)?;
    context.check_cancelled()?;
    fill_face_info::perform_with_tolerance(topo, arena, tol)?;

    Ok(())
}

/// Initialize a pave block for every edge across all `sources`, de-duplicating
/// edges shared between solids (coincident walls). Mirrors
/// [`PaveFiller::init_pave_blocks`] but over N solids.
fn init_pave_blocks_n(
    topo: &Topology,
    sources: &[SolidId],
    arena: &mut GfaArena,
) -> Result<(), AlgoError> {
    let mut pending: Vec<(EdgeId, VertexId, f64, VertexId, f64)> = Vec::new();
    let mut seen: BTreeSet<EdgeId> = arena.edge_pave_blocks.keys().copied().collect();
    for &solid in sources {
        for edge_id in remus_topology::explorer::solid_edges(topo, solid)? {
            if !seen.insert(edge_id) {
                continue;
            }
            let edge = topo.edge(edge_id)?;
            topo.vertex(edge.start())?;
            topo.vertex(edge.end())?;
            let (t0, t1) = helpers::authoritative_edge_domain(
                edge,
                edge_id,
                "N-way pave-block initialization",
            )?;
            pending.push((edge_id, edge.start(), t0, edge.end(), t1));
        }
    }
    for (edge_id, start, t0, end, t1) in pending {
        arena.init_edge_pave_block(edge_id, start, t0, end, t1);
    }
    Ok(())
}

/// Every source-index pair `(i, j)` with `i < j`.
///
/// Do not prune this list using vertex-only bounds: analytic curves and
/// surfaces can extend beyond their topological vertices, so such bounds are
/// not conservative and can silently omit real intersections.
fn source_pairs(sources: &[SolidId]) -> Vec<(usize, usize)> {
    let mut pairs = Vec::new();
    for i in 0..sources.len() {
        for j in (i + 1)..sources.len() {
            pairs.push((i, j));
        }
    }
    pairs
}
