//! Fillet builder: orchestrates the full fillet pipeline.
//!
//! Spine construction, analytic/walking stripe computation, face trimming,
//! and solid assembly. Supports constant and variable radius fillets on
//! planar face pairs (v1).

use std::collections::{HashMap, HashSet};

use remus_math::curves::Circle3D;
use remus_math::curves2d::{Curve2D, NurbsCurve2D};
use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use crate::analytic;
use crate::blend_func::{BorrowedEvolRadBlend, ConstRadBlend};
use crate::builder_utils::{
    FlippedNormalSurface, add_certified_curve_edge, project_onto_axis, radial_distance,
    refuse_non_line_rim_neighbors, surface_ref_or_adapter, wire_axial_range, wire_radial_extremum,
};
use crate::corner;
use crate::g1_chain;
use crate::radius_law::RadiusLaw;
use crate::spine::Spine;
use crate::stripe::{Stripe, StripeResult};
use crate::trimmer;
use crate::walker::{
    Walker, WalkerConfig, approximate_blend_surface, approximate_blend_surface_linear_v,
};
use crate::{BlendError, BlendFaceOrigins, BlendResult};

/// Builder for fillet (rounding) operations on solid edges.
///
/// Collects edge sets with their radius laws, then computes and assembles
/// the filleted solid in a single `build()` call.
pub struct FilletBuilder<'a> {
    topo: &'a mut Topology,
    solid: SolidId,
    /// Edge sets to fillet, each with their radius/law.
    edge_sets: Vec<(Vec<EdgeId>, RadiusLaw)>,
}

impl<'a> FilletBuilder<'a> {
    /// Create a new fillet builder for the given solid.
    #[must_use]
    pub fn new(topo: &'a mut Topology, solid: SolidId) -> Self {
        Self {
            topo,
            solid,
            edge_sets: Vec::new(),
        }
    }

    /// Add edges to fillet with a constant radius.
    ///
    /// Returns `&mut Self` for method chaining.
    pub fn add_edges(&mut self, edges: &[EdgeId], radius: f64) -> &mut Self {
        self.edge_sets
            .push((edges.to_vec(), RadiusLaw::Constant(radius)));
        self
    }

    /// Add edges with variable radius law.
    ///
    /// Returns `&mut Self` for method chaining.
    pub fn add_edges_with_law(&mut self, edges: &[EdgeId], law: RadiusLaw) -> &mut Self {
        self.edge_sets.push((edges.to_vec(), law));
        self
    }

    /// Compute and build the filleted solid.
    ///
    /// # Algorithm
    ///
    /// 1. Build adjacency index for the solid.
    /// 2. For each target edge, find the two adjacent faces.
    /// 3. Expand each edge set into G1 ridgeline chains and build a spine per chain.
    /// 4. Compute stripes via analytic fast path or walking engine.
    /// 5. Trim adjacent faces along contact curves.
    /// 6. Assemble new solid from trimmed faces, blend faces, and untouched
    ///    original faces.
    ///
    /// # Errors
    ///
    /// Returns [`BlendError`] if no edges were specified, or if topology
    /// lookups fail. Individual edge failures are recorded in
    /// [`BlendResult::failed`] rather than aborting the whole operation.
    #[allow(clippy::too_many_lines)]
    pub fn build(self) -> Result<BlendResult, BlendError> {
        // Keep each edge set beside its RadiusLaw via a shared index.
        let mut seeds_by_law: Vec<Vec<EdgeId>> = Vec::with_capacity(self.edge_sets.len());
        let mut laws: Vec<RadiusLaw> = Vec::with_capacity(self.edge_sets.len());
        for (edges, law) in self.edge_sets {
            seeds_by_law.push(edges);
            laws.push(law);
        }

        if seeds_by_law.iter().all(Vec::is_empty) {
            return Err(BlendError::Topology(remus_topology::TopologyError::Empty {
                entity: "fillet edge set",
            }));
        }

        // Prove every standard law over its complete normalized domain before
        // the builder can allocate or trim topology. Linear and S-curve laws
        // are monotone, so their endpoint extrema are exact. An opaque custom
        // callback has no analytic bound; sample it deterministically here for
        // early refusal, then the walker re-validates every station it actually
        // consumes. It is never replaced by a different law.
        let tol = remus_math::tolerance::Tolerance::new();
        for (seeds, law) in seeds_by_law.iter().zip(&laws) {
            if seeds.is_empty() {
                continue;
            }
            if law.exact_bounds().is_some() {
                law.validate_at(0.0, tol.linear)?;
                law.validate_at(1.0, tol.linear)?;
            } else {
                for i in 0..=256 {
                    law.validate_at(f64::from(i) / 256.0, tol.linear)?;
                }
            }
        }

        let topo = self.topo;

        let adjacency = topo.build_adjacency(self.solid)?;

        let solid_data = topo.solid(self.solid)?;
        let shell_id = solid_data.outer_shell();
        let inner_shells = solid_data.inner_shells().to_vec();
        let original_faces: Vec<FaceId> = topo.shell(shell_id)?.faces().to_vec();

        // Track which faces are touched (adjacent to a fillet edge).
        let mut touched_faces: HashSet<FaceId> = HashSet::new();

        let mut succeeded: Vec<EdgeId> = Vec::new();
        let mut failed: Vec<(EdgeId, BlendError)> = Vec::new();
        let mut stripe_results: Vec<StripeResult> = Vec::new();

        // Blend whole G1 ridgelines, not the individual edges the caller
        // happened to name. A tangent-continuous run that is split into
        // several edges — a corner column interrupted where a wall seats into
        // a plate, say — cannot be filleted piecewise: a stripe covering only
        // part of it has to run out in the middle of a smooth edge, where
        // there is no cap face to close against. Expanding to the chain gives
        // the stripe real ends. This matches the v1 rolling-ball engine, which
        // has always called `expand_g1_chain`.
        let mut chain_work: Vec<(Vec<EdgeId>, usize)> = Vec::new();
        for (law_idx, seeds) in seeds_by_law.iter().enumerate() {
            if seeds.is_empty() {
                continue;
            }
            for chain in g1_chain::g1_chains(topo, self.solid, seeds, tol)? {
                chain_work.push((chain, law_idx));
            }
        }

        // Two or more chains touching the same vertex need a vertex blend
        // there. The corner solver computes exact geometry for that
        // (`corner::compute_corners` already returns patches), but this builder
        // cannot yet assemble it watertight. Fail fast with a typed error
        // before any stripe work so callers (`try_fillet`, `fillet_v2`) fall
        // through to an engine that closes corners, instead of paying for a
        // doomed build plus rollback.
        //
        // Removing this guard does not currently reach the corner code at all.
        // Measured on a two-edge box corner with the guard disabled, the
        // blockers are, in the order they bite:
        //
        //  1. `trimmer` cannot cut one base face twice. Both stripes trim the
        //     shared cap and the second cut fails outright with
        //     `TrimmingFailure`, so nothing downstream ever runs.
        //  2. Stripes are not set back at the shared vertex — each runs to the
        //     vertex plane, leaving no tangency circle for a patch to meet.
        //  3. Corner patches mint their own boundary edges instead of reusing
        //     those set-back boundaries, so stripe-to-corner adjacency would be
        //     coincidental rather than topological.
        //  4. The trimmer does not let a base face consume the corner patch's
        //     arc boundary when its wire is rewritten.
        //
        // Until all four are addressed the shell has free edges by
        // construction. The planar fast path in `fillet_rolling_ball` closes
        // these corners today — which is why a plain box and a drilled plate
        // both fillet corner chains and whole perimeters — so what this guard
        // still blocks is corner chains on curved or imported geometry the fast
        // path declines.
        {
            let mut chains_at_vertex: HashMap<usize, (remus_topology::vertex::VertexId, usize)> =
                HashMap::new();
            for (chain_idx, (chain, _)) in chain_work.iter().enumerate() {
                let mut seen_here: HashSet<usize> = HashSet::new();
                for &eid in chain {
                    let Ok(edge) = topo.edge(eid) else { continue };
                    for vid in [edge.start(), edge.end()] {
                        if !seen_here.insert(vid.index()) {
                            continue; // count each chain once per vertex
                        }
                        let entry = chains_at_vertex
                            .entry(vid.index())
                            .or_insert((vid, chain_idx));
                        if entry.1 != chain_idx {
                            return Err(BlendError::UnsupportedVertexBlend {
                                vertex: entry.0,
                                stripes: 2,
                            });
                        }
                    }
                }
            }
        }

        for (chain, law_idx) in &chain_work {
            // Report against the edges the caller asked for, not the ones the
            // chain expansion pulled in.
            let requested: Vec<EdgeId> = chain
                .iter()
                .copied()
                .filter(|eid| seeds_by_law[*law_idx].contains(eid))
                .collect();
            let report_edge = requested
                .first()
                .copied()
                .or_else(|| chain.first().copied());

            let spine = match Spine::from_chain(topo, chain.clone()) {
                Ok(spine) => spine,
                Err(e) => {
                    if let Some(edge) = report_edge {
                        failed.push((edge, e));
                    }
                    continue;
                }
            };
            match compute_stripe_for_spine(topo, &adjacency, spine, &laws[*law_idx]) {
                Ok(sr) => {
                    touched_faces.insert(sr.stripe.face1);
                    touched_faces.insert(sr.stripe.face2);
                    stripe_results.push(sr);
                    succeeded.extend(requested.iter().copied());
                }
                // A radius the geometry cannot accommodate is a verdict on the
                // radius, not on this engine, so it must reach the caller as
                // itself. Filed among `failed` it becomes a bare partial
                // result, which reads exactly like an internal failure and
                // leaves the caller no way to say "try a smaller radius".
                // Same treatment the rim assembler's own bound already gets.
                Err(e @ (BlendError::InvalidInput { .. } | BlendError::RadiusTooLarge { .. })) => {
                    return Err(e);
                }
                Err(e) => {
                    if let Some(edge) = report_edge {
                        failed.push((edge, e));
                    }
                }
            }
        }

        if stripe_results.is_empty() {
            let is_partial = !failed.is_empty();
            return Ok(BlendResult {
                solid: self.solid,
                succeeded: Vec::new(),
                failed,
                is_partial,
                // Nothing was blended, so the input solid is the result and
                // every face is itself.
                face_origins: Some(BlendFaceOrigins {
                    survived: original_faces.iter().map(|&f| (f, f)).collect(),
                    deleted: Vec::new(),
                    created: Vec::new(),
                    created_unattributed: Vec::new(),
                }),
            });
        }

        // Partition out closed-revolution rim stripes (a full circular rim
        // between a bounded disc cap and a cylinder/cone wall). These need an
        // annular assembly that rebuilds the cap, shortens the wall, and emits
        // a toroidal band — all sharing the two contact-circle edges — which
        // the per-face line-based trimmer cannot produce (a closed interior
        // contact circle crosses no boundary edge). Regular stripes still flow
        // through the trim + corner + blend-face path below.
        let mut blend_face_ids: Vec<FaceId> = Vec::new();
        // Every blend face beside the two base faces it was built between —
        // exact provenance, taken from the stripe that produced it.
        let mut blend_face_origins: Vec<(FaceId, Vec<FaceId>)> = Vec::new();
        let mut face_replacements: std::collections::HashMap<FaceId, FaceId> =
            std::collections::HashMap::new();
        let mut regular_results: Vec<&StripeResult> = Vec::new();
        for sr in &stripe_results {
            if let Some(rim) = closed_rim_info(topo, &sr.stripe)? {
                match assemble_closed_rim(topo, &sr.stripe, &rim, &mut face_replacements) {
                    Ok(band) => {
                        blend_face_ids.push(band);
                        blend_face_origins.push((band, vec![sr.stripe.face1, sr.stripe.face2]));
                    }
                    // A radius the geometry cannot accommodate is a verdict,
                    // not a reason to try another assembler: no engine below
                    // can fit a blend that does not fit. Report it and let the
                    // caller lower the radius.
                    Err(e @ BlendError::RadiusTooLarge { .. }) => return Err(e),
                    Err(e) => {
                        log::warn!("closed-rim assembly failed: {e}, falling back to trim path");
                        regular_results.push(sr);
                    }
                }
            } else if let Some(rim) = closed_quadric_rim_info(topo, &sr.stripe)? {
                match assemble_closed_quadric_rim(topo, &sr.stripe, &rim, &mut face_replacements) {
                    Ok(band) => {
                        blend_face_ids.push(band);
                        blend_face_origins.push((band, vec![sr.stripe.face1, sr.stripe.face2]));
                    }
                    Err(e @ BlendError::RadiusTooLarge { .. }) => return Err(e),
                    Err(e) => {
                        log::warn!(
                            "closed curved-rim assembly failed: {e}, falling back to trim path"
                        );
                        regular_results.push(sr);
                    }
                }
            } else if closed_walking_rim_info(topo, &sr.stripe)? {
                match assemble_closed_walking_rim(topo, &sr.stripe, &mut face_replacements) {
                    Ok(band) => {
                        blend_face_ids.push(band);
                        blend_face_origins.push((band, vec![sr.stripe.face1, sr.stripe.face2]));
                    }
                    Err(e @ BlendError::RadiusTooLarge { .. }) => return Err(e),
                    Err(e) => {
                        log::warn!(
                            "closed walking-rim assembly failed: {e}, falling back to trim path"
                        );
                        regular_results.push(sr);
                    }
                }
            } else {
                regular_results.push(sr);
            }
        }

        let stripes: Vec<Stripe> = regular_results.iter().map(|sr| sr.stripe.clone()).collect();
        let corner_results = corner::compute_corners(topo, &stripes, self.solid)?;

        // Trim results per regular stripe, kept so the blend faces can share
        // the trimmer's contact edges instead of minting duplicates.
        let mut trim_pairs: Vec<(trimmer::TrimResult, trimmer::TrimResult)> = Vec::new();

        let mut corner_face_ids: Vec<FaceId> = Vec::new();
        for cr in &corner_results {
            corner_face_ids.push(cr.face_id);
        }

        let mut stripe_contact_edges: Vec<(
            Option<remus_topology::edge::EdgeId>,
            Option<remus_topology::edge::EdgeId>,
        )> = Vec::new();
        for sr in &regular_results {
            let stripe = &sr.stripe;
            stripe_contact_edges.push((None, None));

            // Keep the side of the contact line AWAY from the spine edge: the
            // strip between the contact line and the old edge is what the
            // blend face replaces. The side is resolved inside the trimmer,
            // whose Left/Right frame follows each face's wire traversal and
            // cannot be predicted here; a ball-centre plane-side test flips
            // for concave edges even though the in-plane keep side does not.
            let spine_pt = stripe.spine.evaluate(topo, 0.0)?;
            let keep = trimmer::TrimKeep::AwayFrom(spine_pt);

            // Trim face 1 — use current replacement if face was already trimmed.
            let current_face1 = face_replacements
                .get(&stripe.face1)
                .copied()
                .unwrap_or(stripe.face1);
            let trim1 = trimmer::trim_face_general(
                topo,
                current_face1,
                &stripe.contact1,
                keep,
                stripe.spine.edges(),
            );

            let tr1 = match trim1 {
                Ok(tr) if tr.trimmed_face != current_face1 => {
                    if let Some(slot) = stripe_contact_edges.last_mut() {
                        slot.0 = tr.contact_edge;
                    }
                    face_replacements.insert(stripe.face1, tr.trimmed_face);
                    tr
                }
                Ok(_) | Err(_) => {
                    return Err(BlendError::TrimmingFailure { face: stripe.face1 });
                }
            };

            let current_face2 = face_replacements
                .get(&stripe.face2)
                .copied()
                .unwrap_or(stripe.face2);
            let trim2 = trimmer::trim_face_general(
                topo,
                current_face2,
                &stripe.contact2,
                keep,
                stripe.spine.edges(),
            );

            let tr2 = match trim2 {
                Ok(tr) if tr.trimmed_face != current_face2 => {
                    if let Some(slot) = stripe_contact_edges.last_mut() {
                        slot.1 = tr.contact_edge;
                    }
                    face_replacements.insert(stripe.face2, tr.trimmed_face);
                    tr
                }
                Ok(_) | Err(_) => {
                    return Err(BlendError::TrimmingFailure { face: stripe.face2 });
                }
            };
            trim_pairs.push((tr1, tr2));
        }

        let mut blend_cross_edges: Vec<(
            remus_topology::edge::EdgeId,
            remus_topology::vertex::VertexId,
            remus_topology::vertex::VertexId,
        )> = Vec::new();
        for (si, (sr, (tr1, tr2))) in regular_results.iter().zip(&trim_pairs).enumerate() {
            let stripe = &sr.stripe;

            // Preserve the fork's stitched planar path when it can close the
            // spine ends directly; otherwise reuse the upstream trimmer
            // contact edges and notch the remaining end caps below.
            match stitch_planar_blend(topo, stripe, tr1, tr2, &face_replacements) {
                Ok(Some(mut faces)) => {
                    blend_face_origins
                        .extend(faces.iter().map(|&f| (f, vec![stripe.face1, stripe.face2])));
                    blend_face_ids.append(&mut faces);
                    continue;
                }
                Ok(None) => {}
                Err(e) => {
                    log::warn!(
                        "stitched blend assembly failed ({e}); using shared-contact blend face"
                    );
                }
            }

            // Reuse the trimmed neighbours' contact edges so the blend flank
            // shares one edge entity per contact instead of minting a
            // duplicate that leaves both faces' copies use-1.
            let (c1, c2) = stripe_contact_edges
                .get(si)
                .copied()
                .unwrap_or((None, None));
            let info = crate::builder_utils::create_blend_face_with_contacts(topo, stripe, c1, c2)?;
            blend_face_ids.push(info.face);
            blend_face_origins.push((info.face, vec![stripe.face1, stripe.face2]));
            blend_cross_edges.push(info.cross_end);
            blend_cross_edges.push(info.cross_start);
        }

        // Notch the fillet's end cross-section arcs out of the faces that
        // still cover the scooped corner (the untouched end caps): replace
        // each cap's two-edge corner path with the blend's own cross edge so
        // both sides share one edge entity.
        for arc in &blend_cross_edges {
            let candidates: Vec<(FaceId, FaceId)> = original_faces
                .iter()
                .map(|&f| (f, face_replacements.get(&f).copied().unwrap_or(f)))
                .collect();
            for (orig, fid) in candidates {
                if let Some(nf) = crate::builder_utils::notch_face_corner_with_arc(topo, fid, *arc)?
                {
                    face_replacements.insert(orig, nf);
                    break;
                }
            }
        }

        let mut result_faces: Vec<FaceId> = Vec::new();

        for &fid in &original_faces {
            if !touched_faces.contains(&fid) {
                // An untouched face may still have been rebuilt by the
                // end-cap notch pass.
                result_faces.push(face_replacements.get(&fid).copied().unwrap_or(fid));
            }
        }

        // Shell face order is downstream-visible (face indexing, provenance,
        // replay determinism); iterating the HashSet here made it vary run to
        // run, so fix the order by id before appending.
        let mut touched_in_order: Vec<FaceId> = touched_faces.iter().copied().collect();
        touched_in_order.sort_unstable();
        for fid in touched_in_order {
            let replacement = face_replacements.get(&fid).copied();
            result_faces.push(replacement.unwrap_or(fid));
        }

        result_faces.extend(&blend_face_ids);
        result_faces.extend(&corner_face_ids);

        // Provenance, straight from the bookkeeping above: an untouched face is
        // itself, a trimmed one is its replacement, and each blend band names
        // the two base faces its stripe ran between. Corner patches are the one
        // thing this builder cannot name a source for — `CornerResult` records
        // no stripe — so they are reported as created-with-no-origin rather
        // than attributed to whichever face happens to be nearest.
        let mut survived: Vec<(FaceId, FaceId)> = Vec::with_capacity(original_faces.len());
        for &fid in &original_faces {
            survived.push((fid, face_replacements.get(&fid).copied().unwrap_or(fid)));
        }
        let face_origins = BlendFaceOrigins {
            survived,
            deleted: Vec::new(),
            created: blend_face_origins,
            created_unattributed: corner_face_ids.clone(),
        };

        let new_shell = Shell::new(result_faces)?;
        let new_shell_id = topo.add_shell(new_shell);
        let new_solid = Solid::new(new_shell_id, inner_shells);
        let new_solid_id = topo.add_solid(new_solid);

        let is_partial = !failed.is_empty();
        Ok(BlendResult {
            solid: new_solid_id,
            succeeded,
            failed,
            is_partial,
            face_origins: Some(face_origins),
        })
    }
}

/// Find how `edge` is traversed (forward flag) in `face`'s outer wire.
fn wire_traversal_of(topo: &Topology, face: FaceId, edge: EdgeId) -> Option<bool> {
    let wire_id = topo.face(face).ok()?.outer_wire();
    let wire = topo.wire(wire_id).ok()?;
    wire.edges()
        .iter()
        .find(|oe| oe.edge() == edge)
        .map(OrientedEdge::is_forward)
}

/// Create the circular end-arc edge of a straight-spine fillet, ordered so
/// the CCW span from the edge's start vertex traces the quarter arc nearest
/// the removed corner vertex `v_pt` (Circle edges sample the CCW span from
/// start to end around the circle normal).
fn make_end_arc(
    topo: &mut Topology,
    c_a: VertexId,
    c_b: VertexId,
    center: Point3,
    axis: Vec3,
    radius: f64,
    v_pt: Point3,
) -> Result<Option<EdgeId>, BlendError> {
    use remus_math::traits::ParametricCurve;
    const TAU: f64 = std::f64::consts::TAU;

    let pa = topo.vertex(c_a)?.point();
    let pb = topo.vertex(c_b)?.point();
    let Ok(circle) = Circle3D::new(center, axis, radius) else {
        return Ok(None);
    };
    let mid_of = |from: Point3, to: Point3| -> Point3 {
        let a0 = circle.project(from);
        let delta = (circle.project(to) - a0).rem_euclid(TAU);
        let delta = if delta < 1e-12 { TAU } else { delta };
        ParametricCurve::evaluate(&circle, a0 + delta / 2.0)
    };
    let (start, end, start_point, end_point) =
        if (mid_of(pa, pb) - v_pt).length() <= (mid_of(pb, pa) - v_pt).length() {
            (c_a, c_b, pa, pb)
        } else {
            (c_b, c_a, pb, pa)
        };
    let start_parameter = circle.project(start_point);
    let delta = (circle.project(end_point) - start_parameter).rem_euclid(TAU);
    let delta = if delta < 1e-12 { TAU } else { delta };
    let edge = add_certified_curve_edge(
        topo,
        start,
        end,
        EdgeCurve::Circle(circle),
        (start_parameter, start_parameter + delta),
    )?;
    Ok(Some(edge))
}

/// Close one spine end of a straight-edge fillet against the surrounding
/// faces.
///
/// After the two side faces are trimmed, the corner vertex `v_id` survives
/// only in the wires of end faces. Two configurations occur:
///
/// - **Cap trim**: one wire holds both sub-edges `c → v → c'` (a
///   perpendicular cap face, e.g. the bottom of a plate). The pair is
///   replaced in place by the end arc.
/// - **Corner patch**: the sub-edges live in two different wires (coplanar
///   neighbor faces continuing past the spine end, e.g. a wall seated on the
///   plate). A new planar patch face `arc → sub → v → sub` is created,
///   reusing the existing sub-edges so the shell stays manifold. Its outward
///   normal is `cap_normal` (pointing back along the spine, into the removed
///   corner column).
///
/// Returns a newly created patch face, if any. Logs and returns `Ok(None)`
/// without mutation when neither pattern matches.
#[allow(clippy::too_many_lines)]
#[allow(clippy::too_many_arguments)]
fn stitch_end(
    topo: &mut Topology,
    spine_edges: &[EdgeId],
    v_id: VertexId,
    arc: EdgeId,
    arc_blend_from: VertexId,
    c1: VertexId,
    c2: VertexId,
    cap_normal: Vec3,
) -> Result<Option<FaceId>, BlendError> {
    // --- Case A: a single wire traverses c → v → c' consecutively. ---
    let wire_ids: Vec<_> = topo.wires().iter().map(|(id, _)| id).collect();
    let mut case_a: Option<(
        remus_topology::wire::WireId,
        usize,
        usize,
        VertexId,
        VertexId,
    )> = None;
    'outer: for &wid in &wire_ids {
        let wire = topo.wire(wid)?;
        let oes = wire.edges();
        if oes
            .iter()
            .any(|oe| spine_edges.contains(&oe.edge()) || oe.edge() == arc)
        {
            continue;
        }
        let n = oes.len();
        for i in 0..n {
            let j = (i + 1) % n;
            let ei = topo.edge(oes[i].edge())?;
            let ej = topo.edge(oes[j].edge())?;
            if oes[i].oriented_end(ei) == v_id && oes[j].oriented_start(ej) == v_id {
                let xa = oes[i].oriented_start(ei);
                let yb = oes[j].oriented_end(ej);
                if (xa == c1 && yb == c2) || (xa == c2 && yb == c1) {
                    case_a = Some((wid, i, j, xa, yb));
                    break 'outer;
                }
            }
        }
    }

    if let Some((wid, i, j, xa, _yb)) = case_a {
        let arc_forward = topo.edge(arc)?.start() == xa;
        let oes = topo.wire(wid)?.edges().to_vec();
        let mut new_edges = Vec::with_capacity(oes.len() - 1);
        for (pos, oe) in oes.iter().enumerate() {
            if pos == i {
                new_edges.push(OrientedEdge::new(arc, arc_forward));
            } else if pos == j {
                // consumed by the arc
            } else {
                new_edges.push(*oe);
            }
        }
        let replacement = Wire::new(new_edges, true)?;
        topo.replace_boundary_wire(wid, replacement)?;
        return Ok(None);
    }

    // --- Case B: the sub-edges live in two separate live wires. ---
    let mut sub1: Option<EdgeId> = None;
    let mut sub2: Option<EdgeId> = None;
    for &wid in &wire_ids {
        let wire = topo.wire(wid)?;
        let oes = wire.edges();
        if oes
            .iter()
            .any(|oe| spine_edges.contains(&oe.edge()) || oe.edge() == arc)
        {
            continue;
        }
        for oe in oes {
            let e = topo.edge(oe.edge())?;
            let ends = (e.start(), e.end());
            if ends == (v_id, c1) || ends == (c1, v_id) {
                sub1 = Some(oe.edge());
            } else if ends == (v_id, c2) || ends == (c2, v_id) {
                sub2 = Some(oe.edge());
            }
        }
    }
    let (Some(sub1), Some(sub2)) = (sub1, sub2) else {
        log::warn!("fillet end stitch: no cap pattern found at spine end vertex {v_id:?}");
        return Ok(None);
    };

    // Traverse the arc opposite to the blend face; then chain through v_id.
    let cap_from = if arc_blend_from == c1 { c2 } else { c1 };
    let cap_to = if cap_from == c1 { c2 } else { c1 };
    let arc_e = topo.edge(arc)?;
    let arc_forward = arc_e.start() == cap_from;

    let (first_sub, second_sub) = if cap_to == c1 {
        (sub1, sub2)
    } else {
        (sub2, sub1)
    };
    let fs = topo.edge(first_sub)?;
    let first_forward = fs.start() == cap_to && fs.end() == v_id;
    let ss = topo.edge(second_sub)?;
    let second_forward = ss.start() == v_id && ss.end() == cap_from;

    let wire = Wire::new(
        vec![
            OrientedEdge::new(arc, arc_forward),
            OrientedEdge::new(first_sub, first_forward),
            OrientedEdge::new(second_sub, second_forward),
        ],
        true,
    )?;
    let wire_id = topo.add_wire(wire);
    let d = {
        let p = topo.vertex(v_id)?.point();
        cap_normal.dot(Vec3::new(p.x(), p.y(), p.z()))
    };
    let face = Face::new(
        wire_id,
        Vec::new(),
        FaceSurface::Plane {
            normal: cap_normal,
            d,
        },
    );
    Ok(Some(topo.add_face(face)))
}

/// Watertight assembly for a finite straight-edge fillet: reuse the
/// trimmer's contact edges as the blend wall's long boundaries, close the
/// two spine ends with exact circular arcs, and stitch those arcs into the
/// surrounding cap faces.
///
/// Returns `Ok(None)` (caller falls back to the detached quad) when the
/// stripe is not a single open line-edge spine or the topology does not
/// match the expected pattern. Only mutates after all applicability checks
/// pass.
#[allow(clippy::too_many_lines)]
#[allow(clippy::similar_names)]
fn stitch_planar_blend(
    topo: &mut Topology,
    stripe: &Stripe,
    tr1: &trimmer::TrimResult,
    tr2: &trimmer::TrimResult,
    face_replacements: &std::collections::HashMap<FaceId, FaceId>,
) -> Result<Option<Vec<FaceId>>, BlendError> {
    // A straight run: every edge on the chain is a line, and the chain is an
    // open path so it has two free ends to close against. A single edge is
    // the degenerate case of this.
    let spine_edges = stripe.spine.edges();
    if spine_edges.is_empty() {
        return Ok(None);
    }
    let mut ends: std::collections::HashMap<VertexId, usize> = std::collections::HashMap::new();
    for &eid in spine_edges {
        let e = topo.edge(eid)?;
        if e.start() == e.end() || !matches!(e.curve(), EdgeCurve::Line) {
            return Ok(None);
        }
        *ends.entry(e.start()).or_insert(0) += 1;
        *ends.entry(e.end()).or_insert(0) += 1;
    }
    if ends.values().filter(|count| **count == 1).count() != 2 {
        // A closed ridgeline has no free end; that is the closed-rim path.
        return Ok(None);
    }
    // Take the ends from the ordered chain rather than from the incidence
    // map, whose iteration order is not stable — `v0`/`v1` set the spine
    // direction, and flipping it between runs would flip the end arcs with it.
    let first = topo.edge(spine_edges[0])?;
    let (v0, v1) = if spine_edges.len() == 1 {
        (first.start(), first.end())
    } else {
        let second = topo.edge(spine_edges[1])?;
        let joins_second = [second.start(), second.end()];
        let v0 = if joins_second.contains(&first.start()) {
            first.end()
        } else {
            first.start()
        };
        let last = topo.edge(spine_edges[spine_edges.len() - 1])?;
        let prev = topo.edge(spine_edges[spine_edges.len() - 2])?;
        let joins_prev = [prev.start(), prev.end()];
        let v1 = if joins_prev.contains(&last.start()) {
            last.end()
        } else {
            last.start()
        };
        (v0, v1)
    };
    let (Some(ce1), Some(ce2)) = (tr1.contact_edge, tr2.contact_edge) else {
        return Ok(None);
    };
    if stripe.sections.is_empty() {
        return Ok(None);
    }

    let p0 = topo.vertex(v0)?.point();
    let p1 = topo.vertex(v1)?.point();
    let Ok(dir) = (p1 - p0).normalize() else {
        return Ok(None);
    };

    // Classify each contact-edge endpoint to a spine end by axial position.
    let classify = |topo: &Topology, ce: EdgeId| -> Result<[VertexId; 2], BlendError> {
        let e = topo.edge(ce)?;
        let (s, t) = (e.start(), e.end());
        let ps = topo.vertex(s)?.point();
        let pt = topo.vertex(t)?.point();
        Ok(if dir.dot(ps - p0) <= dir.dot(pt - p0) {
            [s, t]
        } else {
            [t, s]
        })
    };
    let c1 = classify(topo, ce1)?;
    let c2 = classify(topo, ce2)?;
    if c1[0] == c2[0] || c1[1] == c2[1] {
        return Ok(None);
    }

    // Pick the walker section nearest each spine end for the arc centers.
    let (Some(sec_first), Some(sec_last)) = (stripe.sections.first(), stripe.sections.last())
    else {
        return Ok(None);
    };
    let (sec0, sec1) =
        if (dir.dot(sec_first.center - p0)).abs() <= (dir.dot(sec_last.center - p0)).abs() {
            (sec_first, sec_last)
        } else {
            (sec_last, sec_first)
        };

    // Blend wall traverses each contact edge opposite to its trimmed face.
    let f1_now = face_replacements
        .get(&stripe.face1)
        .copied()
        .unwrap_or(tr1.trimmed_face);
    let f2_now = face_replacements
        .get(&stripe.face2)
        .copied()
        .unwrap_or(tr2.trimmed_face);
    let (Some(f1_fwd), Some(f2_fwd)) = (
        wire_traversal_of(topo, f1_now, ce1),
        wire_traversal_of(topo, f2_now, ce2),
    ) else {
        return Ok(None);
    };

    let ce1_e = topo.edge(ce1)?;
    let ce2_e = topo.edge(ce2)?;
    let ce1_pair = (ce1_e.start(), ce1_e.end());
    let ce2_pair = (ce2_e.start(), ce2_e.end());
    // Traversal start/end of ce1 in the blend wire.
    let (b1_from, b1_to) = if f1_fwd {
        (ce1_pair.1, ce1_pair.0)
    } else {
        ce1_pair
    };
    let (b2_from, b2_to) = if f2_fwd {
        (ce2_pair.1, ce2_pair.0)
    } else {
        ce2_pair
    };
    // The loop ce1 → arc → ce2 → arc must alternate ends: after traversing
    // ce1 to its end-k vertex, ce2 must start from ITS end-k vertex.
    let end_of = |v: VertexId| -> Option<usize> {
        if v == c1[0] || v == c2[0] {
            Some(0)
        } else if v == c1[1] || v == c2[1] {
            Some(1)
        } else {
            None
        }
    };
    let (Some(k1), Some(k2)) = (end_of(b1_to), end_of(b2_from)) else {
        return Ok(None);
    };
    if k1 != k2 {
        // The two trimmed faces traverse their contact edges in a pattern
        // that cannot close a quad loop; leave to the fallback.
        log::warn!("fillet stitch: contact edge traversals do not alternate; skipping");
        return Ok(None);
    }

    // Arcs at both ends (created before any wire mutation).
    let radius0 = sec0.radius;
    let radius1 = sec1.radius;
    let Some(arc_at_k1) = make_end_arc(
        topo,
        b1_to,
        b2_from,
        if k1 == 0 { sec0.center } else { sec1.center },
        dir,
        if k1 == 0 { radius0 } else { radius1 },
        if k1 == 0 { p0 } else { p1 },
    )?
    else {
        return Ok(None);
    };
    let Some(arc_at_other) = make_end_arc(
        topo,
        b2_to,
        b1_from,
        if k1 == 0 { sec1.center } else { sec0.center },
        dir,
        if k1 == 0 { radius1 } else { radius0 },
        if k1 == 0 { p1 } else { p0 },
    )?
    else {
        return Ok(None);
    };

    // Blend wall wire: ce1, arc, ce2, arc.
    let fwd_between = |topo: &Topology, e: EdgeId, from: VertexId| -> Result<bool, BlendError> {
        Ok(topo.edge(e)?.start() == from)
    };
    let wire = Wire::new(
        vec![
            OrientedEdge::new(ce1, !f1_fwd),
            OrientedEdge::new(arc_at_k1, fwd_between(topo, arc_at_k1, b1_to)?),
            OrientedEdge::new(ce2, !f2_fwd),
            OrientedEdge::new(arc_at_other, fwd_between(topo, arc_at_other, b2_to)?),
        ],
        true,
    )?;
    let wire_id = topo.add_wire(wire);

    // Orient the blend wall: outward is radially away from the ball center
    // for a convex edge, toward it for a concave edge.
    let secm = &stripe.sections[stripe.sections.len() / 2];
    let f1_face = topo.face(stripe.face1)?;
    let n1_stored = f1_face.surface().normal(0.0, 0.0);
    let n1_out = if f1_face.is_reversed() {
        -n1_stored
    } else {
        n1_stored
    };
    let convex = n1_out.dot(secm.center - secm.p1) < 0.0;
    let Ok(radial) = (secm.p1 - secm.center).normalize() else {
        return Ok(None);
    };
    let desired = if convex { radial } else { -radial };
    let reversed = match stripe.surface.project_point(secm.p1) {
        Some((u, v)) => stripe.surface.normal(u, v).dot(desired) < 0.0,
        None => false,
    };

    let mut blend_face = Face::new(wire_id, Vec::new(), stripe.surface.clone());
    blend_face.set_reversed(reversed);
    let blend_face_id = topo.add_face(blend_face);

    let mut faces = vec![blend_face_id];

    // Close both spine ends. Arc endpoints at end k: c1[k] / c2[k]; the
    // blend traverses arc_at_k1 from b1_to, and arc_at_other from b2_to.
    let ends = [
        (
            if k1 == 0 { v0 } else { v1 },
            arc_at_k1,
            b1_to,
            if k1 == 0 { dir } else { -dir },
            k1,
        ),
        (
            if k1 == 0 { v1 } else { v0 },
            arc_at_other,
            b2_to,
            if k1 == 0 { -dir } else { dir },
            1 - k1,
        ),
    ];
    for &(v_end, arc, arc_from, cap_normal, k) in &ends {
        if let Some(patch) = stitch_end(
            topo,
            spine_edges,
            v_end,
            arc,
            arc_from,
            c1[k],
            c2[k],
            cap_normal,
        )? {
            faces.push(patch);
        }
    }

    Ok(Some(faces))
}

/// Geometry of a full-revolution rim fillet (a closed circular edge between a
/// planar cap and an axisymmetric wall), recovered from a stripe whose blend
/// surface is a torus.
struct ClosedRimInfo {
    /// The planar cap face.
    plane_face: FaceId,
    /// The axisymmetric wall face (`Cylinder` or `Cone`).
    wall_face: FaceId,
    /// The original closed rim edge on the wall, to be replaced by the
    /// wall-contact circle.
    rim_edge: EdgeId,
    /// Whether the rim is one of the cap's INNER loops (a hole drilled through
    /// a plate) rather than its outer boundary (a bounded disc cap).
    ///
    /// The two differ in which way the setback runs. On a disc cap the contact
    /// circle shrinks the outer boundary to `r_c − r`; on a hole rim it grows
    /// the inner loop to `r_c + r`, replacing that loop and leaving the outer
    /// wire and the cap's other holes alone.
    rim_is_inner: bool,
    /// Whether the rim is convex (the fillet removes material) rather than
    /// concave (it fills the re-entrant corner).
    convex: bool,
    /// Contact circle on the plate (radius `r_c ∓ r`), in the plane.
    plate_circle: Circle3D,
    /// Contact circle on the wall (radius `r_c` for a cylinder), one fillet
    /// radius along the axis from the plate.
    wall_circle: Circle3D,
}

/// A closed circular ridge shared directly by two curved analytic supports.
///
/// Both neighbours are periodic walls, so each contact circle replaces the
/// original rim in one wall; the blend band then shares those two circles.
struct ClosedQuadricRimInfo {
    faces: [FaceId; 2],
    rim_edge: EdgeId,
    contacts: [Circle3D; 2],
}

fn qualified_closed_curved_pair(first: &FaceSurface, second: &FaceSurface) -> bool {
    match first {
        FaceSurface::Cylinder(_) => matches!(
            second,
            FaceSurface::Cylinder(_) | FaceSurface::Cone(_) | FaceSurface::Sphere(_)
        ),
        FaceSurface::Cone(_) => {
            matches!(second, FaceSurface::Cylinder(_) | FaceSurface::Cone(_))
        }
        FaceSurface::Sphere(_) => matches!(second, FaceSurface::Cylinder(_)),
        FaceSurface::Plane { .. } | FaceSurface::Torus(_) | FaceSurface::Nurbs(_) => false,
    }
}

fn closed_quadric_rim_info(
    topo: &Topology,
    stripe: &Stripe,
) -> Result<Option<ClosedQuadricRimInfo>, BlendError> {
    if !matches!(stripe.surface, FaceSurface::Torus(_))
        || !stripe.spine.is_closed()
        || stripe.spine.edges().len() != 1
    {
        return Ok(None);
    }
    let rim_edge = stripe.spine.edges()[0];
    let edge = topo.edge(rim_edge)?;
    let EdgeCurve::Circle(rim) = edge.curve() else {
        return Ok(None);
    };
    let pair = (
        topo.face(stripe.face1)?.surface(),
        topo.face(stripe.face2)?.surface(),
    );
    if !edge.is_closed() || !qualified_closed_curved_pair(pair.0, pair.1) {
        return Ok(None);
    }
    let Some(section) = stripe.sections.first() else {
        return Ok(None);
    };
    let axis = rim.normal().normalize()?;
    let make_contact = |point: Point3| -> Result<Circle3D, BlendError> {
        let center = project_onto_axis(point, rim.center(), axis);
        let radial = point - center;
        let radius = radial.length();
        if radius <= remus_math::tolerance::Tolerance::new().linear {
            return Err(BlendError::TrimmingFailure { face: stripe.face1 });
        }
        Ok(Circle3D::new_with_ref(
            center,
            rim.normal(),
            radius,
            radial,
        )?)
    };
    let contacts = [make_contact(section.p1)?, make_contact(section.p2)?];
    let tolerance = 20.0
        * remus_math::tolerance::Tolerance::new().linear
        * contacts[0].radius().max(contacts[1].radius()).max(1.0);
    if stripe.sections.iter().any(|candidate| {
        [candidate.p1, candidate.p2]
            .into_iter()
            .zip(&contacts)
            .any(|(point, circle)| {
                let projected = circle.evaluate(circle.project(point));
                (projected - point).length() > tolerance
            })
    }) {
        return Ok(None);
    }
    Ok(Some(ClosedQuadricRimInfo {
        faces: [stripe.face1, stripe.face2],
        rim_edge,
        contacts,
    }))
}

/// Detect a full-revolution rim-fillet stripe and recover its annular geometry.
///
/// Returns `Some` when the blend surface is a torus, the spine is a single
/// closed circular edge (start vertex == end vertex), and the two adjacent
/// faces are a plane (the disc cap) and a cylinder/cone (the wall). Returns
/// `None` for every other configuration (so the caller uses the normal trim
/// path).
///
/// # Errors
///
/// Returns [`BlendError`] if topology lookups or circle construction fail.
fn closed_rim_info(topo: &Topology, stripe: &Stripe) -> Result<Option<ClosedRimInfo>, BlendError> {
    if !matches!(stripe.surface, FaceSurface::Torus(_)) {
        return Ok(None);
    }

    // Spine must be a single closed circular edge.
    let edges = stripe.spine.edges();
    if edges.len() != 1 {
        return Ok(None);
    }
    let rim_edge = edges[0];
    let rim_normal = {
        let e = topo.edge(rim_edge)?;
        if e.start() != e.end() {
            return Ok(None);
        }
        let EdgeCurve::Circle(circle) = e.curve() else {
            return Ok(None);
        };
        circle.normal()
    };

    // One side is the plane (cap), the other the cylinder/cone wall.
    let s1 = topo.face(stripe.face1)?.surface().clone();
    let s2 = topo.face(stripe.face2)?.surface().clone();
    let (plane_face, wall_face) = match (&s1, &s2) {
        (FaceSurface::Plane { .. }, FaceSurface::Cylinder(_) | FaceSurface::Cone(_)) => {
            (stripe.face1, stripe.face2)
        }
        (FaceSurface::Cylinder(_) | FaceSurface::Cone(_), FaceSurface::Plane { .. }) => {
            (stripe.face2, stripe.face1)
        }
        _ => return Ok(None),
    };

    // The annular rebuild replaces exactly one of the cap's loops with the
    // plate-contact circle, so it applies whenever some loop is precisely this
    // rim and nothing else.
    //
    //   * the OUTER wire — a bounded disc cap, e.g. a primitive cylinder's end
    //     face. The cap may still carry holes (a drilled flange's rim cap is an
    //     annulus with a central opening and bolt holes) and those are
    //     preserved verbatim by the rebuild.
    //   * an INNER wire — a hole drilled through a plate. Here the rim IS a
    //     hole, so the rebuild swaps that one loop and leaves the outer wire
    //     and the cap's other holes untouched.
    //
    // Anything else falls back to the normal trim path.
    let rim_is_inner = {
        let cap = topo.face(plane_face)?;
        let is_lone_rim = |wid| -> Result<bool, BlendError> {
            let edges = topo.wire(wid)?.edges();
            Ok(edges.len() == 1 && edges[0].edge() == rim_edge)
        };
        if is_lone_rim(cap.outer_wire())? {
            false
        } else if cap.inner_wires().iter().try_fold(false, |acc, &wid| {
            Ok::<_, BlendError>(acc || is_lone_rim(wid)?)
        })? {
            true
        } else {
            return Ok(None);
        }
    };

    // The plane-side contact curve is the one whose face is the plane.
    let (plate_contact, wall_contact) = if plane_face == stripe.face1 {
        (&stripe.contact1, &stripe.contact2)
    } else {
        (&stripe.contact2, &stripe.contact1)
    };

    // Recover the wall axis line from the wall surface.
    let wall_surf = topo.face(wall_face)?.surface().clone();
    let (axis, axis_origin) = match &wall_surf {
        FaceSurface::Cylinder(c) => (c.axis(), c.origin()),
        FaceSurface::Cone(c) => (c.axis(), c.apex()),
        _ => return Ok(None),
    };

    // Each contact is a full circle perpendicular to the axis; recover its
    // centre (foot on the axis line) and radius (radial distance) from one
    // sampled point.
    let (pt0, _) = plate_contact.domain();
    let plate_pt = plate_contact.evaluate(pt0);
    let plate_center = project_onto_axis(plate_pt, axis_origin, axis);
    let plate_radius = radial_distance(plate_pt, axis_origin, axis);

    let (wt0, _) = wall_contact.domain();
    let wall_pt = wall_contact.evaluate(wt0);
    let wall_center = project_onto_axis(wall_pt, axis_origin, axis);
    let wall_radius = radial_distance(wall_pt, axis_origin, axis);

    // Pin both contact circles' `evaluate(0)` to the ray the rim's own seam
    // vertex sits on. The rebuild shortens the wall by swapping the rim circle
    // for the wall-contact circle and re-pointing the wall's seam edge at the
    // new circle's seam vertex — while keeping the seam's CURVE. A circle built
    // without a reference direction puts its seam wherever `Frame3::from_normal`
    // happens to land (for an axis of `+z`, a quarter turn away), so the seam
    // line was re-pointed at a vertex a quarter turn from where it starts: a
    // straight chord through the inside of the wall, an edge of the wall face
    // that is not on the wall surface. It survives every topological check —
    // the shell closes, nothing is free or non-manifold, and the tessellator
    // and volume integrator both work from the surface rather than that edge —
    // but any consumer that reads the wire as the face's real boundary sees a
    // face a fraction of its true size. `mass_properties` does, and reported
    // half the volume of a rim-filleted cylinder.
    let seam_dir = {
        let v = topo.vertex(topo.edge(rim_edge)?.start())?.point() - axis_origin;
        v - axis * axis.dot(v)
    };
    // Keep the source rim's parameter direction. The cap and wall retain the
    // source edge-use flags, so rebuilding around the unsigned wall axis can
    // silently reverse a bore loop whose source circle uses the opposite axis.
    let plate_circle = Circle3D::new_with_ref(plate_center, rim_normal, plate_radius, seam_dir)?;
    let wall_circle = Circle3D::new_with_ref(wall_center, rim_normal, wall_radius, seam_dir)?;

    // When the contact moves INTO the wall, the rebuild shortens the wall to
    // meet it — and can only do that with material the wall HAS. On a 6 mm
    // plate an R9 hole-rim fillet puts the contact 3 mm below the underside,
    // and the rebuild emits it happily: the shell still closes, no edge is
    // free, and the tessellation is still watertight, because every one of
    // those checks is topological. Only the wall's own axial extent says the
    // geometry is nonsense.
    //
    // A concave rim blend moves the contact the other way and EXTENDS the wall
    // instead — a cone's base rim flares into a foot below its own base plane,
    // which is legitimate and has no such bound. That case shows up as the wall
    // having no extent at all in the setback direction, and is left alone.
    {
        let setback = axis.dot(wall_center - plate_center);
        let wall_wire = topo.face(wall_face)?.outer_wire();
        let (s_min, s_max) = wire_axial_range(topo, wall_wire, plate_center, axis)?;
        // The rim sits at s = 0 (the cap's plane); `available` is how far the
        // wall reaches from there in the direction the contact moved.
        let available = if setback >= 0.0 { s_max } else { -s_min };
        let shortening = available > 1e-9 * (1.0 + setback.abs());
        if shortening && setback.abs() >= available {
            // Report the achievable fillet radius, not the axial setback: they
            // coincide for a cylinder and differ by the generator slope on a
            // cone.
            let radius = match &stripe.surface {
                FaceSurface::Torus(t) => t.minor_radius(),
                _ => setback.abs(),
            };
            let scale = if setback.abs() > 0.0 {
                available / setback.abs()
            } else {
                0.0
            };
            return Err(BlendError::RadiusTooLarge {
                edge: rim_edge,
                max_radius: (radius * scale).max(0.0),
            });
        }
    }

    // The cap's other loops survive the rebuild unchanged, which is only
    // correct if the moved contact circle still clears every one of them. A
    // radius that reaches past one would need the fillet and that loop to merge
    // into a single surface — real geometry this rebuild cannot express — and
    // the resulting cap wire would cross its own boundary while still passing
    // the closed-shell and Euler checks, i.e. ship as a self-intersecting body.
    //
    // Which loops are obstacles, and on which side, follows from the direction
    // the contact moved:
    //
    //   * disc cap (rim is the outer wire): the boundary SHRINKS to `r_c − r`,
    //     so the cap's holes must all stay inside it.
    //   * hole rim (rim is an inner wire): the loop GROWS to `r_c + r`, so the
    //     outer wire and every other hole must stay outside it.
    {
        let cap = topo.face(plane_face)?;
        let others: Vec<WireId> = if rim_is_inner {
            std::iter::once(cap.outer_wire())
                .chain(cap.inner_wires().iter().copied())
                .filter(|&wid| {
                    topo.wire(wid)
                        .is_ok_and(|w| w.edges().first().is_none_or(|oe| oe.edge() != rim_edge))
                })
                .collect()
        } else {
            cap.inner_wires().to_vec()
        };
        for wid in others {
            let clearance = wire_radial_extremum(topo, wid, axis_origin, axis, rim_is_inner)?;
            let collides = if rim_is_inner {
                clearance <= plate_radius
            } else {
                clearance >= plate_radius
            };
            if !collides {
                continue;
            }
            if rim_is_inner {
                // The cause is genuinely the radius: the same rim rounds fine
                // below the clearance, so say so with the achievable maximum
                // rather than failing as a trimming problem.
                return Err(BlendError::RadiusTooLarge {
                    edge: rim_edge,
                    max_radius: (clearance - wall_radius).max(0.0),
                });
            }
            // The shrinking-disc direction keeps its established behaviour:
            // defer to the trim path rather than change how existing shapes
            // report.
            return Ok(None);
        }
    }

    // Keep the analytic arm's convexity fact for assembly. For cylinders the
    // rim-loop position plus the wall's reversed flag reproduces
    // `plane_cylinder_fillet`'s bounded/material table exactly. The cone arm
    // carries its own convention: reversed means a tapered hole (concave).
    let wall_reversed = topo.face(wall_face)?.is_reversed();
    let convex = match wall_surf {
        FaceSurface::Cone(_) => !wall_reversed,
        _ => rim_is_inner == wall_reversed,
    };

    Ok(Some(ClosedRimInfo {
        plane_face,
        wall_face,
        rim_edge,
        rim_is_inner,
        convex,
        plate_circle,
        wall_circle,
    }))
}

/// Assemble a full-revolution rim fillet: rebuild the disc cap bounded by the
/// plate-contact circle, shorten the wall to the wall-contact circle, and emit
/// the toroidal band between them. The cap and wall edges are shared with the
/// band so the result is watertight.
///
/// Updates `face_replacements` for the cap and wall (so a later stripe sees the
/// shortened wall). Returns the new toroidal band face.
///
/// # Errors
///
/// Returns [`BlendError`] if topology lookups or wire/face construction fail.
fn assemble_closed_rim(
    topo: &mut Topology,
    stripe: &Stripe,
    rim: &ClosedRimInfo,
    face_replacements: &mut std::collections::HashMap<FaceId, FaceId>,
) -> Result<FaceId, BlendError> {
    const TOL: f64 = 1e-7;

    // Snapshot the cap and wall (resolving any prior replacement) before
    // mutating the arena.
    let plane_surf = topo.face(rim.plane_face)?.surface().clone();
    let plane_reversed = topo.face(rim.plane_face)?.is_reversed();

    let current_wall = face_replacements
        .get(&rim.wall_face)
        .copied()
        .unwrap_or(rim.wall_face);
    let wall_surf = topo.face(current_wall)?.surface().clone();
    let wall_reversed = topo.face(current_wall)?.is_reversed();
    let wall_outer_wire = topo.face(current_wall)?.outer_wire();
    let wall_inner = topo.face(current_wall)?.inner_wires().to_vec();
    let wall_oriented: Vec<OrientedEdge> = topo.wire(wall_outer_wire)?.edges().to_vec();

    let torus = match &stripe.surface {
        FaceSurface::Torus(t) => t.clone(),
        _ => {
            return Err(BlendError::TrimmingFailure {
                face: rim.wall_face,
            });
        }
    };

    // `closed_rim_info` measured the setback against the wall as it was; by the
    // time this runs, an earlier rim on the SAME wall may already have eaten
    // into it. Two R2 fillets on the two rims of a 3 mm bore each pass on their
    // own and together invert the wall, so re-measure what is actually left.
    {
        let ax = torus.z_axis();
        let setback = ax.dot(rim.wall_circle.center() - rim.plate_circle.center());
        let (s_min, s_max) =
            wire_axial_range(topo, wall_outer_wire, rim.plate_circle.center(), ax)?;
        let available = if setback >= 0.0 { s_max } else { -s_min };
        let shortening = available > 1e-9 * (1.0 + setback.abs());
        if shortening && setback.abs() >= available {
            let scale = if setback.abs() > 0.0 {
                available / setback.abs()
            } else {
                0.0
            };
            return Err(BlendError::RadiusTooLarge {
                edge: rim.rim_edge,
                max_radius: (torus.minor_radius() * scale).max(0.0),
            });
        }
    }

    let old_rim_vertex = topo.edge(rim.rim_edge)?.start();
    refuse_non_line_rim_neighbors(
        topo,
        &wall_oriented,
        rim.rim_edge,
        old_rim_vertex,
        rim.wall_face,
    )?;

    // Vertices for the two closed contact circles (start == end → degenerate).
    let plate_point = rim.plate_circle.evaluate(0.0);
    let wall_point = rim.wall_circle.evaluate(0.0);
    let plate_v = topo.add_vertex(Vertex::new(plate_point, TOL));
    let wall_v = topo.add_vertex(Vertex::new(wall_point, TOL));

    // Shared contact-circle edges.
    let plate_edge = add_certified_curve_edge(
        topo,
        plate_v,
        plate_v,
        EdgeCurve::Circle(rim.plate_circle.clone()),
        (0.0, std::f64::consts::TAU),
    )?;
    let wall_edge = add_certified_curve_edge(
        topo,
        wall_v,
        wall_v,
        EdgeCurve::Circle(rim.wall_circle.clone()),
        (0.0, std::f64::consts::TAU),
    )?;
    // Exact minor-circle seam connecting the two contacts. A straight chord is
    // not on the torus and makes paired rim fillets lose volume during surface
    // integration. Choose the circle normal from the ordered contact vectors
    // so the edge follows the short blend arc from plate to wall.
    let axis = torus.z_axis();
    let radial = wall_point - torus.center();
    let radial = (radial - axis * axis.dot(radial)).normalize()?;
    let seam_center = torus.center() + radial * torus.major_radius();
    let seam_normal = (plate_point - seam_center)
        .cross(wall_point - seam_center)
        .normalize()?;
    let seam_circle = Circle3D::new_with_ref(
        seam_center,
        seam_normal,
        torus.minor_radius(),
        plate_point - seam_center,
    )?;
    let seam_end = seam_circle
        .project(wall_point)
        .rem_euclid(std::f64::consts::TAU);
    let seam_edge = add_certified_curve_edge(
        topo,
        plate_v,
        wall_v,
        EdgeCurve::Circle(seam_circle),
        (0.0, seam_end),
    )?;

    // --- Rebuild the cap with the rim loop replaced by the plate contact. ---
    // Exactly one of the cap's loops is the rim; that loop becomes the
    // plate-contact circle with the orientation the cap had on the original rim
    // edge, and every other loop is carried through verbatim. Handing the
    // rebuilt face an empty inner-wire list would fill in every hole — a
    // drilled flange's rim cap would lose its bore and bolt openings, and each
    // bore wall would lose the face it pairs with, opening the shell.
    let cap_orig = topo.face(
        face_replacements
            .get(&rim.plane_face)
            .copied()
            .unwrap_or(rim.plane_face),
    )?;
    let cap_orig_outer = cap_orig.outer_wire();
    let cap_orig_inner = cap_orig.inner_wires().to_vec();
    let rim_loop = if rim.rim_is_inner {
        *cap_orig_inner
            .iter()
            .find(|&&wid| {
                topo.wire(wid)
                    .is_ok_and(|w| w.edges().iter().any(|oe| oe.edge() == rim.rim_edge))
            })
            .ok_or(BlendError::TrimmingFailure {
                face: rim.plane_face,
            })?
    } else {
        cap_orig_outer
    };
    let cap_forward = topo
        .wire(rim_loop)?
        .edges()
        .iter()
        .find(|oe| oe.edge() == rim.rim_edge)
        .is_some_and(OrientedEdge::is_forward);
    let contact_wire = Wire::new(vec![OrientedEdge::new(plate_edge, cap_forward)], true)?;
    let contact_wire_id = topo.add_wire(contact_wire);
    let (cap_wire_id, cap_inner) = if rim.rim_is_inner {
        let inner = cap_orig_inner
            .iter()
            .map(|&wid| {
                if wid == rim_loop {
                    contact_wire_id
                } else {
                    wid
                }
            })
            .collect();
        (cap_orig_outer, inner)
    } else {
        (contact_wire_id, cap_orig_inner)
    };
    let mut cap_face = Face::new(cap_wire_id, cap_inner, plane_surf);
    cap_face.set_reversed(plane_reversed);
    let cap_face_id = topo.add_face(cap_face);
    face_replacements.insert(rim.plane_face, cap_face_id);

    // --- Shorten the wall to the wall-contact circle. ---
    // The wall's outer wire references the rim circle plus (for the cylinder /
    // cone primitive) a degenerate seam line whose lower endpoint is the rim
    // vertex. Replace the rim circle with the wall-contact circle, and rebuild
    // any seam edge touching the old rim vertex so its lower endpoint becomes
    // the new wall-circle vertex (otherwise the wire no longer closes — the
    // seam would still start at the old rim height).
    // A seam edge may appear twice in the wall wire (fwd + rev); rebuild each
    // distinct edge once so both references share the new edge (otherwise the
    // two copies each become a free edge).
    let mut rebuilt: std::collections::HashMap<EdgeId, EdgeId> = std::collections::HashMap::new();
    let mut new_wall_edges: Vec<OrientedEdge> = Vec::with_capacity(wall_oriented.len());
    let mut wall_forward = None;
    for oe in &wall_oriented {
        if oe.edge() == rim.rim_edge {
            new_wall_edges.push(OrientedEdge::new(wall_edge, oe.is_forward()));
            wall_forward = Some(oe.is_forward());
            continue;
        }
        let e = topo.edge(oe.edge())?;
        let touches_rim = e.start() == old_rim_vertex || e.end() == old_rim_vertex;
        if touches_rim {
            let new_eid = if let Some(&id) = rebuilt.get(&oe.edge()) {
                id
            } else {
                // Rebuild this edge with `wall_v` substituted for the old rim vertex.
                if !matches!(e.curve(), EdgeCurve::Line) {
                    return Err(BlendError::TrimmingFailure {
                        face: rim.wall_face,
                    });
                }
                let new_start = if e.start() == old_rim_vertex {
                    wall_v
                } else {
                    e.start()
                };
                let new_end = if e.end() == old_rim_vertex {
                    wall_v
                } else {
                    e.end()
                };
                let id = topo.add_edge(Edge::new(new_start, new_end, EdgeCurve::Line));
                rebuilt.insert(oe.edge(), id);
                id
            };
            new_wall_edges.push(OrientedEdge::new(new_eid, oe.is_forward()));
        } else {
            new_wall_edges.push(*oe);
        }
    }
    let Some(wall_forward) = wall_forward else {
        return Err(BlendError::TrimmingFailure {
            face: rim.wall_face,
        });
    };
    let new_wall_wire = Wire::new(new_wall_edges, true)?;
    let new_wall_wire_id = topo.add_wire(new_wall_wire);
    let mut new_wall_face = Face::new(new_wall_wire_id, wall_inner, wall_surf);
    new_wall_face.set_reversed(wall_reversed);
    let new_wall_face_id = topo.add_face(new_wall_face);
    face_replacements.insert(rim.wall_face, new_wall_face_id);

    // --- Toroidal band between the two contact circles. ---
    // Degenerate-seam wire (plate circle, seam up, wall circle reversed, seam
    // down). The seam runs plate_v → wall_v, so this fixed order always closes
    // (plate_v → plate_v → wall_v → wall_v → plate_v). The shared circle edges
    // are used opposite to the standard-wound cap and wall, keeping the shell
    // manifold.
    let band_reversed = torus_band_needs_reversal(&torus, rim);
    let cap_effective_forward = cap_forward != plane_reversed;
    let wall_effective_forward = wall_forward != wall_reversed;
    let band_plate_forward = cap_effective_forward == band_reversed;
    let band_wall_forward = wall_effective_forward == band_reversed;
    let band_wire = Wire::new(
        vec![
            OrientedEdge::new(plate_edge, band_plate_forward),
            OrientedEdge::new(seam_edge, true),
            OrientedEdge::new(wall_edge, band_wall_forward),
            OrientedEdge::new(seam_edge, false),
        ],
        true,
    )?;
    let band_wire_id = topo.add_wire(band_wire);
    let mut band_face = Face::new(band_wire_id, Vec::new(), stripe.surface.clone());
    // Orient the band so its outward normal points away from the solid. The
    // solid tessellator orients a torus band's triangles from the surface's
    // intrinsic (u, v) frame, then applies the face `reversed` flag; pick the
    // flag that makes the geometric normal at the band's mid-arc point outward.
    // Outward at a rim fillet points away from the cylinder axis (positive
    // radial) and away from the material along the axis; the torus geometric
    // normal at the mid-arc already has the correct radial sign, so we compare
    // its axial component against the material side.
    //
    // The band must traverse each shared contact circle in the EFFECTIVE
    // sense (is_forward XOR is_reversed) OPPOSITE its other user: the cap
    // holds `plate_edge` at `cap_forward` under `plane_reversed`, the wall
    // holds `wall_edge` at `wall_forward` under `wall_reversed`. Both
    // circles are degenerate (start == end vertex), so the chain closes
    // for any sense choice and the two senses are picked independently. A
    // fixed wire order cannot serve both rims of a cylinder — their caps
    // traverse the shared circles in opposite directions.
    if band_reversed {
        band_face.set_reversed(true);
    }
    let band_face_id = topo.add_face(band_face);

    Ok(band_face_id)
}

/// Replace one closed rim of a periodic cylinder/cone support with a contact
/// circle and retarget the two uses of its seam to the new circle vertex.
fn rebuild_periodic_rim_support(
    topo: &mut Topology,
    face_id: FaceId,
    rim_edge: EdgeId,
    contact_edge: EdgeId,
    contact_vertex: VertexId,
) -> Result<(FaceId, bool, bool), BlendError> {
    let face = topo.face(face_id)?;
    if !matches!(
        face.surface(),
        FaceSurface::Cylinder(_) | FaceSurface::Cone(_)
    ) {
        return Err(BlendError::TrimmingFailure { face: face_id });
    }
    let surface = face.surface().clone();
    let reversed = face.is_reversed();
    let inner_wires = face.inner_wires().to_vec();
    let wire_id = face.outer_wire();
    let oriented = topo.wire(wire_id)?.edges().to_vec();
    let old_vertex = topo.edge(rim_edge)?.start();
    refuse_non_line_rim_neighbors(topo, &oriented, rim_edge, old_vertex, face_id)?;

    let mut rebuilt = HashMap::new();
    let mut new_edges = Vec::with_capacity(oriented.len());
    let mut contact_forward = None;
    for use_ in oriented {
        if use_.edge() == rim_edge {
            contact_forward = Some(use_.is_forward());
            new_edges.push(OrientedEdge::new(contact_edge, use_.is_forward()));
            continue;
        }
        let edge = topo.edge(use_.edge())?;
        let touches_rim = edge.start() == old_vertex || edge.end() == old_vertex;
        if !touches_rim {
            new_edges.push(use_);
            continue;
        }
        let replacement = if let Some(existing) = rebuilt.get(&use_.edge()) {
            *existing
        } else {
            let start = if edge.start() == old_vertex {
                contact_vertex
            } else {
                edge.start()
            };
            let end = if edge.end() == old_vertex {
                contact_vertex
            } else {
                edge.end()
            };
            let replacement = topo.add_edge(Edge::new(start, end, EdgeCurve::Line));
            rebuilt.insert(use_.edge(), replacement);
            replacement
        };
        new_edges.push(OrientedEdge::new(replacement, use_.is_forward()));
    }
    let contact_forward = contact_forward.ok_or(BlendError::TrimmingFailure { face: face_id })?;
    let wire = topo.add_wire(Wire::new(new_edges, true)?);
    let mut face = Face::new(wire, inner_wires, surface);
    face.set_reversed(reversed);
    Ok((topo.add_face(face), contact_forward, reversed))
}

/// Assemble a closed curved-support shoulder whose analytic solver proved an
/// exact toroidal surface of revolution.
fn assemble_closed_quadric_rim(
    topo: &mut Topology,
    stripe: &Stripe,
    rim: &ClosedQuadricRimInfo,
    face_replacements: &mut HashMap<FaceId, FaceId>,
) -> Result<FaceId, BlendError> {
    const TOL: f64 = 1e-7;
    let FaceSurface::Torus(torus) = &stripe.surface else {
        return Err(BlendError::TrimmingFailure { face: rim.faces[0] });
    };
    let section = stripe
        .sections
        .first()
        .ok_or(BlendError::TrimmingFailure { face: rim.faces[0] })?;

    let points = [rim.contacts[0].evaluate(0.0), rim.contacts[1].evaluate(0.0)];
    let vertices = [
        topo.add_vertex(Vertex::new(points[0], TOL)),
        topo.add_vertex(Vertex::new(points[1], TOL)),
    ];
    let contacts = [
        add_certified_curve_edge(
            topo,
            vertices[0],
            vertices[0],
            EdgeCurve::Circle(rim.contacts[0].clone()),
            (0.0, std::f64::consts::TAU),
        )?,
        add_certified_curve_edge(
            topo,
            vertices[1],
            vertices[1],
            EdgeCurve::Circle(rim.contacts[1].clone()),
            (0.0, std::f64::consts::TAU),
        )?,
    ];

    let mut rebuilt_faces = [rim.faces[0], rim.faces[1]];
    let mut support_forward = [false; 2];
    let mut support_reversed = [false; 2];
    let spine_start = stripe.spine.evaluate(topo, 0.0)?;
    for index in 0..2 {
        let current = face_replacements
            .get(&rim.faces[index])
            .copied()
            .unwrap_or(rim.faces[index]);
        let (face, forward, reversed) =
            if matches!(topo.face(current)?.surface(), FaceSurface::Sphere(_)) {
                let (face, forward, reversed, _) = rebuild_closed_walking_support(
                    topo,
                    current,
                    &[rim.rim_edge],
                    contacts[index],
                    vertices[index],
                    spine_start,
                )?;
                (face, forward, reversed)
            } else {
                rebuild_periodic_rim_support(
                    topo,
                    current,
                    rim.rim_edge,
                    contacts[index],
                    vertices[index],
                )?
            };
        rebuilt_faces[index] = face;
        support_forward[index] = forward;
        support_reversed[index] = reversed;
        face_replacements.insert(rim.faces[index], face);
    }

    // One exact minor-circle arc closes the periodic band seam. Its duplicate
    // reverse use is the second parameter-space branch of that seam.
    let from_center = points[0] - section.center;
    let to_center = points[1] - section.center;
    let seam_normal = from_center.cross(to_center).normalize()?;
    let seam_circle =
        Circle3D::new_with_ref(section.center, seam_normal, section.radius, from_center)?;
    let seam_end = seam_circle
        .project(points[1])
        .rem_euclid(std::f64::consts::TAU);
    if seam_end <= remus_math::tolerance::Tolerance::new().angular {
        return Err(BlendError::TrimmingFailure { face: rim.faces[0] });
    }
    let seam = add_certified_curve_edge(
        topo,
        vertices[0],
        vertices[1],
        EdgeCurve::Circle(seam_circle),
        (0.0, seam_end),
    )?;

    // Match the first support's outward normal at tangency. This works for
    // both convex and concave orientations and avoids a pair-specific winding
    // table.
    let support = topo.face(rebuilt_faces[0])?;
    let (su, sv) = support
        .surface()
        .project_point(points[0])
        .ok_or(BlendError::TrimmingFailure { face: rim.faces[0] })?;
    let mut support_normal = support.surface().normal(su, sv);
    if support.is_reversed() {
        support_normal = -support_normal;
    }
    let (tu, tv) = torus.project_point(points[0]);
    let band_reversed = torus.normal(tu, tv).dot(support_normal) < 0.0;
    let band_forward = [0, 1].map(|index| {
        let support_effective = support_forward[index] != support_reversed[index];
        support_effective == band_reversed
    });
    let wire = topo.add_wire(Wire::new(
        vec![
            OrientedEdge::new(contacts[0], band_forward[0]),
            OrientedEdge::new(seam, true),
            OrientedEdge::new(contacts[1], band_forward[1]),
            OrientedEdge::new(seam, false),
        ],
        true,
    )?);
    let mut band = Face::new(wire, Vec::new(), stripe.surface.clone());
    band.set_reversed(band_reversed);
    Ok(topo.add_face(band))
}

fn closed_walking_rim_info(topo: &Topology, stripe: &Stripe) -> Result<bool, BlendError> {
    if !stripe.spine.is_closed()
        || stripe.spine.edges().is_empty()
        || !matches!(stripe.surface, FaceSurface::Nurbs(_))
    {
        return Ok(false);
    }
    if !qualified_closed_curved_pair(
        topo.face(stripe.face1)?.surface(),
        topo.face(stripe.face2)?.surface(),
    ) {
        return Ok(false);
    }
    let tolerance = 50.0 * remus_math::tolerance::Tolerance::new().linear;
    for contact in [&stripe.contact1, &stripe.contact2] {
        let (t0, t1) = contact.domain();
        if (contact.evaluate(t1) - contact.evaluate(t0)).length() > tolerance {
            return Ok(false);
        }
    }
    let carries = |face_id: FaceId, edge_id: EdgeId| -> Result<bool, BlendError> {
        let face = topo.face(face_id)?;
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            if topo
                .wire(wire)?
                .edges()
                .iter()
                .any(|use_| use_.edge() == edge_id)
            {
                return Ok(true);
            }
        }
        Ok(false)
    };
    for edge in stripe.spine.edges() {
        if !carries(stripe.face1, *edge)? || !carries(stripe.face2, *edge)? {
            return Ok(false);
        }
    }
    Ok(true)
}

/// Replace one closed multi-edge rim in a face by a single closed contact
/// curve. The rim may be a complete inner wire (the stock cylinder side of a
/// drilled opening) or one contiguous block of a periodic outer wire (the
/// bore wall side).
fn rebuild_closed_walking_support(
    topo: &mut Topology,
    face_id: FaceId,
    rim_edges: &[EdgeId],
    contact_edge: EdgeId,
    contact_vertex: VertexId,
    spine_start: Point3,
) -> Result<(FaceId, bool, bool, bool), BlendError> {
    let face = topo.face(face_id)?;
    let surface = face.surface().clone();
    let reversed = face.is_reversed();
    let outer = face.outer_wire();
    let inner = face.inner_wires().to_vec();
    let target = std::iter::once(outer)
        .chain(inner.iter().copied())
        .find(|wire| {
            topo.wire(*wire).is_ok_and(|candidate| {
                rim_edges
                    .iter()
                    .all(|rim| candidate.edges().iter().any(|use_| use_.edge() == *rim))
            })
        })
        .ok_or(BlendError::TrimmingFailure { face: face_id })?;
    let uses = topo.wire(target)?.edges().to_vec();
    let rim_set: HashSet<EdgeId> = rim_edges.iter().copied().collect();
    let rim_count = uses
        .iter()
        .filter(|use_| rim_set.contains(&use_.edge()))
        .count();
    if rim_count != rim_edges.len() {
        return Err(BlendError::TrimmingFailure { face: face_id });
    }
    let transitions = (0..uses.len())
        .filter(|index| {
            rim_set.contains(&uses[*index].edge())
                != rim_set.contains(&uses[(*index + 1) % uses.len()].edge())
        })
        .count();
    if uses.len() != rim_edges.len() && transitions != 2 {
        return Err(BlendError::TrimmingFailure { face: face_id });
    }

    let first_spine = topo.edge(rim_edges[0])?;
    let spine_forward = (topo.vertex(first_spine.start())?.point() - spine_start).length()
        <= (topo.vertex(first_spine.end())?.point() - spine_start).length();
    let source_use = uses
        .iter()
        .find(|use_| use_.edge() == rim_edges[0])
        .ok_or(BlendError::TrimmingFailure { face: face_id })?;
    let contact_forward = source_use.is_forward() == spine_forward;

    let mut rim_vertices = HashSet::new();
    for rim in rim_edges {
        let edge = topo.edge(*rim)?;
        rim_vertices.insert(edge.start());
        rim_vertices.insert(edge.end());
    }
    let mut replacements = HashMap::new();
    let mut inserted = false;
    let mut rebuilt_uses = Vec::with_capacity(uses.len() - rim_edges.len() + 1);
    for use_ in uses {
        if rim_set.contains(&use_.edge()) {
            if !inserted {
                rebuilt_uses.push(OrientedEdge::new(contact_edge, contact_forward));
                inserted = true;
            }
            continue;
        }
        let edge = topo.edge(use_.edge())?;
        let touches_start = rim_vertices.contains(&edge.start());
        let touches_end = rim_vertices.contains(&edge.end());
        if !touches_start && !touches_end {
            rebuilt_uses.push(use_);
            continue;
        }
        if !matches!(edge.curve(), EdgeCurve::Line) {
            return Err(BlendError::TrimmingFailure { face: face_id });
        }
        let edge_start = edge.start();
        let edge_end = edge.end();
        let replacement = if let Some(existing) = replacements.get(&use_.edge()) {
            *existing
        } else {
            let replacement = topo.add_edge(Edge::new(
                if touches_start {
                    contact_vertex
                } else {
                    edge_start
                },
                if touches_end {
                    contact_vertex
                } else {
                    edge_end
                },
                EdgeCurve::Line,
            ));
            replacements.insert(use_.edge(), replacement);
            replacement
        };
        rebuilt_uses.push(OrientedEdge::new(replacement, use_.is_forward()));
    }
    let rebuilt_wire = topo.add_wire(Wire::new(rebuilt_uses, true)?);
    let (new_outer, new_inner) = if target == outer {
        (rebuilt_wire, inner)
    } else {
        (
            outer,
            inner
                .into_iter()
                .map(|wire| if wire == target { rebuilt_wire } else { wire })
                .collect(),
        )
    };
    let mut face = Face::new(new_outer, new_inner, surface);
    face.set_reversed(reversed);
    let face = topo.add_face(face);
    Ok((face, contact_forward, reversed, target == outer))
}

fn face_effective_edge_uses(
    topo: &Topology,
    face_id: FaceId,
) -> Result<HashMap<EdgeId, bool>, BlendError> {
    let face = topo.face(face_id)?;
    let mut uses = HashMap::new();
    let mut repeated = HashSet::new();
    for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for edge_use in topo.wire(wire)?.edges() {
            let effective = edge_use.is_forward() != face.is_reversed();
            if uses.insert(edge_use.edge(), effective).is_some() {
                repeated.insert(edge_use.edge());
            }
        }
    }
    uses.retain(|edge, _| !repeated.contains(edge));
    Ok(uses)
}

fn reverse_face_outer_wire(topo: &mut Topology, face_id: FaceId) -> Result<FaceId, BlendError> {
    let face = topo.face(face_id)?;
    let surface = face.surface().clone();
    let reversed = face.is_reversed();
    let inner = face.inner_wires().to_vec();
    let uses = topo.wire(face.outer_wire())?.edges().to_vec();
    let uses = uses
        .into_iter()
        .rev()
        .map(|edge_use| OrientedEdge::new(edge_use.edge(), !edge_use.is_forward()))
        .collect();
    let outer = topo.add_wire(Wire::new(uses, true)?);
    let mut face = Face::new(outer, inner, surface);
    face.set_reversed(reversed);
    Ok(topo.add_face(face))
}

fn assemble_closed_walking_rim(
    topo: &mut Topology,
    stripe: &Stripe,
    face_replacements: &mut HashMap<FaceId, FaceId>,
) -> Result<FaceId, BlendError> {
    let section = stripe
        .sections
        .first()
        .ok_or(BlendError::TrimmingFailure { face: stripe.face1 })?;
    let curves = [&stripe.contact1, &stripe.contact2];
    let faces = [stripe.face1, stripe.face2];
    let mut vertices = Vec::with_capacity(2);
    let mut contact_edges = Vec::with_capacity(2);
    for curve in curves {
        let domain = curve.domain();
        let start = curve.evaluate(domain.0);
        let closure = (curve.evaluate(domain.1) - start).length();
        let vertex = topo.add_vertex(Vertex::new(start, closure.max(1e-7)));
        let edge = add_certified_curve_edge(
            topo,
            vertex,
            vertex,
            EdgeCurve::NurbsCurve(curve.clone()),
            domain,
        )?;
        vertices.push(vertex);
        contact_edges.push(edge);
    }
    let spine_start = stripe.spine.evaluate(topo, 0.0)?;
    let mut support_forward = [false; 2];
    let mut support_reversed = [false; 2];
    let mut target_is_outer = [false; 2];
    let mut rebuilt_faces = faces;
    for index in 0..2 {
        let current = face_replacements
            .get(&faces[index])
            .copied()
            .unwrap_or(faces[index]);
        let (face, forward, reversed, outer) = rebuild_closed_walking_support(
            topo,
            current,
            stripe.spine.edges(),
            contact_edges[index],
            vertices[index],
            spine_start,
        )?;
        rebuilt_faces[index] = face;
        support_forward[index] = forward;
        support_reversed[index] = reversed;
        target_is_outer[index] = outer;
        face_replacements.insert(faces[index], face);
    }

    let uses0 = face_effective_edge_uses(topo, rebuilt_faces[0])?;
    let uses1 = face_effective_edge_uses(topo, rebuilt_faces[1])?;
    let common: Vec<EdgeId> = uses0
        .keys()
        .filter(|edge| uses1.contains_key(edge))
        .copied()
        .collect();
    if !common.is_empty() && common.iter().all(|edge| uses0.get(edge) == uses1.get(edge)) {
        let Some(index) = target_is_outer.iter().position(|outer| *outer) else {
            return Err(BlendError::TrimmingFailure { face: faces[0] });
        };
        let face = reverse_face_outer_wire(topo, rebuilt_faces[index])?;
        support_forward[index] = !support_forward[index];
        face_replacements.insert(faces[index], face);
    }

    let points = [
        topo.vertex(vertices[0])?.point(),
        topo.vertex(vertices[1])?.point(),
    ];
    let from_center = points[0] - section.center;
    let to_center = points[1] - section.center;
    let seam_normal = from_center.cross(to_center).normalize()?;
    let seam_circle =
        Circle3D::new_with_ref(section.center, seam_normal, section.radius, from_center)?;
    let seam_end = seam_circle
        .project(points[1])
        .rem_euclid(std::f64::consts::TAU);
    let seam = add_certified_curve_edge(
        topo,
        vertices[0],
        vertices[1],
        EdgeCurve::Circle(seam_circle),
        (0.0, seam_end),
    )?;

    let band_surface = match &stripe.surface {
        FaceSurface::Nurbs(surface) => FaceSurface::Nurbs(reverse_nurbs_surface_u(surface)?),
        other => other.clone(),
    };
    let desired_normal = -from_center.normalize()?;
    let surface_normal = band_surface.normal(0.0, 0.0);
    let band_reversed = surface_normal.dot(desired_normal) < 0.0;
    let band_forward = [0, 1].map(|index| {
        let support_effective = support_forward[index] != support_reversed[index];
        support_effective == band_reversed
    });
    let wire = topo.add_wire(Wire::new(
        vec![
            OrientedEdge::new(contact_edges[0], band_forward[0]),
            OrientedEdge::new(seam, true),
            OrientedEdge::new(contact_edges[1], band_forward[1]),
            OrientedEdge::new(seam, false),
        ],
        true,
    )?);
    let mut face = Face::new(wire, Vec::new(), band_surface);
    face.set_reversed(band_reversed);
    let face = topo.add_face(face);

    Ok(face)
}

fn reverse_nurbs_surface_u(
    surface: &remus_math::nurbs::surface::NurbsSurface,
) -> Result<remus_math::nurbs::surface::NurbsSurface, BlendError> {
    let mut control_points = surface.control_points().to_vec();
    control_points.reverse();
    let mut weights = surface.weights().to_vec();
    weights.reverse();
    let first = surface.knots_u()[0];
    let last = *surface.knots_u().last().unwrap_or(&first);
    let knots_u = surface
        .knots_u()
        .iter()
        .rev()
        .map(|knot| first + last - knot)
        .collect();
    Ok(remus_math::nurbs::surface::NurbsSurface::new(
        surface.degree_u(),
        surface.degree_v(),
        knots_u,
        surface.knots_v().to_vec(),
        control_points,
        weights,
    )?)
}

/// Decide whether a rim-fillet torus band must carry `reversed` so its outward
/// normal points away from the solid.
///
/// The band's mid-arc geometric normal points radially out from the tube; we
/// need it to also point to the *empty* side along the axis. The empty side is
/// opposite the wall material: for a non-reversed cylinder/cone wall the
/// material is on the axis-interior side, and the band sits one fillet radius
/// from the plate toward the material — so the band's outward axial direction is
/// the one pointing from the wall-contact circle back toward the plate.
fn torus_band_needs_reversal(
    torus: &remus_math::surfaces::ToroidalSurface,
    rim: &ClosedRimInfo,
) -> bool {
    // The torus geometric normal at the mid-arc point (halfway between the two
    // contacts) should point away from the segment plate→wall along the axis.
    // The "away from material" axial direction is plate_center → (plate_center −
    // wall_center) i.e. from the wall contact toward the plate.
    let axis = torus.z_axis();
    let to_plate = rim.plate_circle.center() - rim.wall_circle.center();
    // The empty side is toward the plate on a convex rim. A concave rim fills
    // the void corner, so its empty side is the axial opposite.
    let mut outward_axial = axis * axis.dot(to_plate);
    if !rim.convex {
        outward_axial = -outward_axial;
    }
    // Mid-arc point and its geometric normal.
    let v_plate = torus.project_point(rim.plate_circle.evaluate(0.0)).1;
    let v_wall = torus.project_point(rim.wall_circle.evaluate(0.0)).1;
    // Shortest signed mid-angle between the two contact v-parameters (periodic):
    // reduce the raw difference into (−π, π].
    let dv = (v_wall - v_plate + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
        - std::f64::consts::PI;
    let v_mid = v_plate + dv * 0.5;
    let n = torus.normal(0.0, v_mid);
    // If the geometric normal's axial part opposes the outward axial direction,
    // the band must be reversed.
    n.dot(outward_axial) < 0.0
}

/// Upgrade a completed constant-radius walk around a coaxial cylinder/cone
/// rim to its exact surface of revolution.
///
/// The nonlinear solve remains authoritative for the two contact locations.
/// Recognition only replaces the sampled tensor-product surface when every
/// section proves the same rolling-ball radius and circular centre locus.
fn recognize_closed_cylinder_cone_torus(
    surface1: &FaceSurface,
    surface2: &FaceSurface,
    spine: &Spine,
    topo: &Topology,
    sections: &[crate::section::CircSection],
    radius: f64,
) -> Result<Option<ToroidalSurface>, BlendError> {
    let is_cylinder_cone = matches!(
        (surface1, surface2),
        (FaceSurface::Cylinder(_), FaceSurface::Cone(_))
            | (FaceSurface::Cone(_), FaceSurface::Cylinder(_))
    );
    if !is_cylinder_cone || !spine.is_closed() || spine.edges().len() != 1 || sections.len() < 3 {
        return Ok(None);
    }
    let edge = topo.edge(spine.edges()[0])?;
    let EdgeCurve::Circle(rim) = edge.curve() else {
        return Ok(None);
    };
    let axis = rim.normal().normalize()?;
    let axis_origin = rim.center();

    let (origin1, axis1) = match surface1 {
        FaceSurface::Cylinder(surface) => (surface.origin(), surface.axis()),
        FaceSurface::Cone(surface) => (surface.apex(), surface.axis()),
        _ => return Ok(None),
    };
    let (origin2, axis2) = match surface2 {
        FaceSurface::Cylinder(surface) => (surface.origin(), surface.axis()),
        FaceSurface::Cone(surface) => (surface.apex(), surface.axis()),
        _ => return Ok(None),
    };
    let axis1 = axis1.normalize()?;
    let axis2 = axis2.normalize()?;
    let tolerance = remus_math::tolerance::Tolerance::new();
    if axis1.dot(axis2).abs() < 1.0 - tolerance.angular
        || axis.dot(axis1).abs() < 1.0 - tolerance.angular
    {
        return Ok(None);
    }
    for origin in [origin1, origin2] {
        let offset = origin - axis_origin;
        if (offset - axis * offset.dot(axis)).length() > tolerance.linear {
            return Ok(None);
        }
    }

    let first = &sections[0];
    if !radius.is_finite() || radius <= tolerance.linear {
        return Ok(None);
    }
    let center = project_onto_axis(first.center, axis_origin, axis);
    let reference = first.center - center;
    let major_radius = reference.length();
    if major_radius <= tolerance.linear {
        return Ok(None);
    }
    let scale = major_radius.max(radius).max(1.0);
    let recognition_tolerance = 20.0 * tolerance.linear * scale;
    if sections.iter().any(|section| {
        (section.radius - radius).abs() > recognition_tolerance
            || ((section.center - center).dot(axis)).abs() > recognition_tolerance
            || ((section.center - center).length() - major_radius).abs() > recognition_tolerance
    }) {
        return Ok(None);
    }

    let torus =
        ToroidalSurface::with_axis_and_ref_dir(center, major_radius, radius, axis, reference)?;
    for section in sections {
        for point in [section.p1, section.p2] {
            let (u, v) = torus.project_point(point);
            if (torus.evaluate(u, v) - point).length() > recognition_tolerance {
                return Ok(None);
            }
        }
    }
    Ok(Some(torus))
}

/// Compute a stripe for a single edge using the adjacency index.
///
/// # Errors
///
/// Returns [`BlendError`] if the edge is non-manifold, if topology lookups
/// fail, or if neither the analytic nor walking path can produce a result.
#[allow(clippy::too_many_lines)]
fn compute_stripe_for_spine(
    topo: &Topology,
    adjacency: &remus_topology::adjacency::AdjacencyIndex,
    spine: Spine,
    law: &RadiusLaw,
) -> Result<StripeResult, BlendError> {
    // Every edge on a G1 chain shares one face pair (that is what makes it a
    // ridgeline), so the first edge speaks for the whole spine.
    let edge_id = spine.edges().first().copied().ok_or(BlendError::Topology(
        remus_topology::TopologyError::Empty {
            entity: "fillet spine",
        },
    ))?;
    let adj_faces = adjacency.faces_for_edge(edge_id);
    if adj_faces.len() != 2 {
        // Non-manifold (3+ faces) or boundary (0-1 faces) edge cannot be filleted.
        log::warn!(
            "edge {edge_id:?} has {} adjacent faces (expected 2) — cannot fillet non-manifold or boundary edges",
            adj_faces.len()
        );
        return Err(BlendError::StartSolutionFailure {
            edge: edge_id,
            t: 0.0,
        });
    }
    let face1 = adj_faces[0];
    let face2 = adj_faces[1];

    // Snapshot surface data, respecting face orientation.
    let face1_data = topo.face(face1)?;
    let surf1 = face1_data.surface().clone();
    let face1_reversed = face1_data.is_reversed();
    let face2_data = topo.face(face2)?;
    let surf2 = face2_data.surface().clone();
    let face2_reversed = face2_data.is_reversed();

    // Get radius at the spine midpoint for the analytic path.
    let radius = law.evaluate(0.5);

    // Try analytic fast path (only for constant radius).
    // The analytic fillet expects INWARD-pointing normals (toward material).
    // Compute inward normals from the surface normals and face reversal:
    // - Not reversed: outward = surface_normal → inward = -surface_normal
    // - Reversed: outward = -surface_normal → inward = surface_normal
    // The closed cone/cone helper chooses its offset signs from stored face
    // reversal alone. That is insufficient at a shoulder where both analytic
    // cones are unreversed but their material wedges lie on opposite sides;
    // the resulting torus can land outside the solid. Route this qualified
    // pair through the orientation-aware walker below.
    let cone_cone = matches!(
        (&surf1, &surf2),
        (FaceSurface::Cone(_), FaceSurface::Cone(_))
    );
    if matches!(law, RadiusLaw::Constant(_)) && !cone_cone {
        let flipped1 = orient_plane_surface(&surf1);
        let flipped2 = orient_plane_surface(&surf2);
        let inward_surf1 = if face1_reversed { &surf1 } else { &flipped1 };
        let inward_surf2 = if face2_reversed { &surf2 } else { &flipped2 };
        if let Some(result) = analytic::try_analytic_fillet(
            inward_surf1,
            inward_surf2,
            &spine,
            topo,
            radius,
            face1,
            face2,
        )? {
            return Ok(result);
        }
    }

    log::debug!(
        target: "remus_approx",
        "fillet: analytic fast-path unavailable for {}+{} ({} radius) — using Newton-Raphson walker (approximate NURBS blend surface)",
        surf1.type_tag(),
        surf2.type_tag(),
        if matches!(law, RadiusLaw::Constant(_)) { "constant" } else { "variable" }
    );

    // The rolling-ball constraints place the centre on each surface's
    // +normal side, so present inward normals for every surface type. A face's
    // intrinsic normal is inward exactly when that face is reversed; otherwise
    // wrap it. The old path only negated planes and silently sent curved faces
    // to the external common-tangent branch.
    let mut adapter1 = None;
    let mut adapter2 = None;
    let base1 = surface_ref_or_adapter(&surf1, &mut adapter1);
    let base2 = surface_ref_or_adapter(&surf2, &mut adapter2);
    let flipped1 = FlippedNormalSurface::new(base1);
    let flipped2 = FlippedNormalSurface::new(base2);
    let ps1: &dyn remus_math::traits::ParametricSurface =
        if face1_reversed { base1 } else { &flipped1 };
    let ps2: &dyn remus_math::traits::ParametricSurface =
        if face2_reversed { base2 } else { &flipped2 };

    let mut config = WalkerConfig::default();
    if !surf1.is_planar() || !surf2.is_planar() {
        // Quadric projection and finite-differenced normal derivatives carry
        // a few ulps more noise than the plane pair. Stay within the topology
        // weld band while avoiding a continuation collapse at residuals only
        // infinitesimally above the default Newton gate.
        config.tol_3d = 5.0 * remus_math::tolerance::Tolerance::new().linear;
    }

    let walk_result = if let RadiusLaw::Constant(r) = law {
        let blend = ConstRadBlend { radius: *r };
        let walker = Walker::new(&blend, ps1, ps2, &spine, topo, config);
        let start = walker.find_start(0.0)?;
        walker.walk(start, 0.0, spine.length())?
    } else {
        let evol = BorrowedEvolRadBlend { law };
        let walker = Walker::new(&evol, ps1, ps2, &spine, topo, config);
        let start = walker.find_start(0.0)?;
        walker.walk(start, 0.0, spine.length())?
    };

    let blend_face_surface = if let Some(torus) = recognize_closed_cylinder_cone_torus(
        &surf1,
        &surf2,
        &spine,
        topo,
        &walk_result.sections,
        radius,
    )? {
        FaceSurface::Torus(torus)
    } else {
        let surface = if spine.is_closed() {
            approximate_blend_surface_linear_v(&walk_result.sections)?
        } else {
            approximate_blend_surface(&walk_result.sections)?
        };
        FaceSurface::Nurbs(surface)
    };

    let contact1 = sections_to_contact_curve(&walk_result.sections, |s| s.p1)?;
    let contact2 = sections_to_contact_curve(&walk_result.sections, |s| s.p2)?;

    let pcurve1 = build_pcurve_from_contact(ps1, &surf1, &contact1)?;
    let pcurve2 = build_pcurve_from_contact(ps2, &surf2, &contact2)?;

    let stripe = Stripe {
        spine,
        surface: blend_face_surface,
        pcurve1,
        pcurve2,
        contact1,
        contact2,
        face1,
        face2,
        sections: walk_result.sections,
    };

    Ok(StripeResult {
        stripe,
        new_edges: Vec::new(),
    })
}

/// A single cross-section of a rolling-ball blend: the two surface contact
/// points, the rational-quadratic arc apex (middle control point), and its
/// weight `cos(half_angle)`.
#[derive(Debug, Clone, Copy)]
pub struct BlendCrossSection {
    /// Contact point on the first surface (`u = 0` end of the arc).
    pub contact1: remus_math::vec::Point3,
    /// Arc apex / middle control point (tangent intersection).
    pub apex: remus_math::vec::Point3,
    /// Contact point on the second surface (`u = 1` end of the arc).
    pub contact2: remus_math::vec::Point3,
    /// Rational-quadratic weight of the apex (`cos(half_angle)`).
    pub weight: f64,
}

/// Compute the true rolling-ball blend cross-sections for a constant-radius
/// fillet of `edge_id`, at the requested spine `fractions` (each in `[0, 1]`).
///
/// Unlike a tangent-plane offset (`contact = p + dir·r`), this solves the
/// actual ball-tangent-to-both-surfaces constraint via the walking engine, so
/// the contacts land *on* curved neighbours (cylinders, NURBS blend faces).
/// Newton continuation seeds each station from the previous one for robustness.
///
/// `surf1`/`surf2` are the neighbour surfaces with their face `reversed` flags
/// (so plane normals point outward consistently with the walker convention).
///
/// # Errors
///
/// Returns [`BlendError`] if the spine cannot be built or Newton fails to
/// converge at a requested station.
#[allow(clippy::too_many_arguments)]
pub fn blend_cross_sections(
    topo: &Topology,
    edge_id: EdgeId,
    surf1: &remus_topology::face::FaceSurface,
    surf1_reversed: bool,
    surf2: &remus_topology::face::FaceSurface,
    surf2_reversed: bool,
    radius: f64,
    fractions: &[f64],
) -> Result<Vec<BlendCrossSection>, BlendError> {
    use remus_math::vec::Point3;

    let spine = Spine::from_single_edge(topo, edge_id)?;
    let len = spine.length();

    let mut adapter1 = None;
    let mut adapter2 = None;
    let base1 = surface_ref_or_adapter(surf1, &mut adapter1);
    let base2 = surface_ref_or_adapter(surf2, &mut adapter2);
    // The walker places the ball centre on the `+normal` side of each surface,
    // so feed it INWARD (toward-material) normals or it solves the external
    // common-tangent branch (fillet outside the solid). The face's outward
    // normal equals the surface normal when the face is not reversed, so flip
    // then; keep it when the face is reversed.
    let flip1 = FlippedNormalSurface::new(base1);
    let flip2 = FlippedNormalSurface::new(base2);
    let ps1: &dyn remus_math::traits::ParametricSurface =
        if surf1_reversed { base1 } else { &flip1 };
    let ps2: &dyn remus_math::traits::ParametricSurface =
        if surf2_reversed { base2 } else { &flip2 };

    let blend = ConstRadBlend { radius };
    let walker = Walker::new(&blend, ps1, ps2, &spine, topo, WalkerConfig::default());

    let mut out = Vec::with_capacity(fractions.len());
    let mut prev: Option<crate::blend_func::BlendParams> = None;
    for &f in fractions {
        let s = f.clamp(0.0, 1.0) * len;
        let (params, sec) =
            walker
                .solve_section(s, prev)
                .ok_or(BlendError::StartSolutionFailure {
                    edge: edge_id,
                    t: f,
                })?;
        prev = Some(params);

        let half_angle = sec.half_angle();
        let w = half_angle.cos();
        let midpoint = Point3::new(
            (sec.p1.x() + sec.p2.x()) * 0.5,
            (sec.p1.y() + sec.p2.y()) * 0.5,
            (sec.p1.z() + sec.p2.z()) * 0.5,
        );
        // Apex at the tangent intersection (r/cos θ from the centre), matching
        // `approximate_blend_surface`. Falls back to the chord midpoint when the
        // arc approaches a half-turn (cos θ → 0).
        let apex = if w.abs() > 1e-15 {
            let scale = 1.0 / (w * w);
            Point3::new(
                sec.center.x() + (midpoint.x() - sec.center.x()) * scale,
                sec.center.y() + (midpoint.y() - sec.center.y()) * scale,
                sec.center.z() + (midpoint.z() - sec.center.z()) * scale,
            )
        } else {
            midpoint
        };

        out.push(BlendCrossSection {
            contact1: sec.p1,
            apex,
            contact2: sec.p2,
            weight: w,
        });
    }
    Ok(out)
}

/// Flip the normal of a `Plane` surface to account for face reversal.
///
/// For non-plane surfaces, returns a clone unchanged — the walker already
/// accounts for orientation through the `ParametricSurface` trait.
fn orient_plane_surface(
    surface: &remus_topology::face::FaceSurface,
) -> remus_topology::face::FaceSurface {
    match surface {
        remus_topology::face::FaceSurface::Plane { normal, d } => {
            remus_topology::face::FaceSurface::Plane {
                normal: -*normal,
                d: -*d,
            }
        }
        other => other.clone(),
    }
}

/// Build a degree-1 NURBS curve from section contact points.
fn sections_to_contact_curve(
    sections: &[crate::section::CircSection],
    pick: impl Fn(&crate::section::CircSection) -> remus_math::vec::Point3,
) -> Result<remus_math::nurbs::curve::NurbsCurve, BlendError> {
    let pts: Vec<remus_math::vec::Point3> = sections.iter().map(&pick).collect();
    if pts.len() < 2 {
        return Err(BlendError::Math(remus_math::MathError::EmptyInput));
    }
    let n = pts.len();
    let degree = 1.min(n - 1);
    let mut knots = vec![0.0; degree + 1];
    if n > 2 {
        for i in 1..n - 1 {
            #[allow(clippy::cast_precision_loss)]
            knots.push(i as f64 / (n - 1) as f64);
        }
    }
    knots.extend(vec![1.0; degree + 1]);
    let weights = vec![1.0; n];
    let curve = remus_math::nurbs::curve::NurbsCurve::new(degree, knots, pts, weights)?;
    Ok(curve)
}

/// Build a PCurve by projecting every contact control point onto the support.
///
/// Angular coordinates are unwrapped before constructing the UV NURBS. This
/// preserves a closed contact's full turn instead of collapsing its coincident
/// endpoints into a zero-length line at the parameter seam.
fn build_pcurve_from_contact(
    surf: &dyn remus_math::traits::ParametricSurface,
    face_surface: &FaceSurface,
    contact: &remus_math::nurbs::curve::NurbsCurve,
) -> Result<remus_math::curves2d::Curve2D, BlendError> {
    let (u_period, v_period) = match face_surface {
        FaceSurface::Cylinder(_) | FaceSurface::Cone(_) | FaceSurface::Sphere(_) => {
            (Some(std::f64::consts::TAU), None)
        }
        FaceSurface::Torus(_) => (Some(std::f64::consts::TAU), Some(std::f64::consts::TAU)),
        FaceSurface::Nurbs(surface) => {
            let u_period = surface
                .is_periodic_u()
                .then(|| surface.domain_u().1 - surface.domain_u().0);
            let v_period = surface
                .is_periodic_v()
                .then(|| surface.domain_v().1 - surface.domain_v().0);
            (u_period, v_period)
        }
        FaceSurface::Plane { .. } => (None, None),
    };
    let mut points: Vec<Point2> = Vec::with_capacity(contact.control_points().len());
    for point in contact.control_points() {
        let (mut u, mut v) = surf.project_point(*point);
        if let Some(previous) = points.last() {
            if let Some(period) = u_period {
                u = unwrap_periodic(u, previous.x(), period);
            }
            if let Some(period) = v_period {
                v = unwrap_periodic(v, previous.y(), period);
            }
        }
        points.push(Point2::new(u, v));
    }
    Ok(Curve2D::Nurbs(NurbsCurve2D::new(
        contact.degree(),
        contact.knots().to_vec(),
        points,
        contact.weights().to_vec(),
    )?))
}

fn unwrap_periodic(value: f64, previous: f64, period: f64) -> f64 {
    value - period * ((value - previous) / period).round()
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use remus_topology::adjacency::AdjacencyIndex;
    use remus_topology::face::FaceSurface;
    use remus_topology::test_utils::make_unit_cube_manifold;

    #[test]
    fn straight_spine_end_arc_stores_selected_quarter_turn_authority() {
        let mut topo = Topology::new();
        let center = Point3::new(0.0, 0.0, 0.0);
        let first = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let second = topo.add_vertex(Vertex::new(Point3::new(0.0, 1.0, 0.0), 1e-7));
        let diagonal = 0.5_f64.sqrt();
        let selected_midpoint = Point3::new(diagonal, diagonal, 0.0);

        let arc = make_end_arc(
            &mut topo,
            first,
            second,
            center,
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            selected_midpoint,
        )
        .unwrap()
        .expect("quarter-turn end arc");
        let edge = topo.edge(arc).unwrap();
        let range = edge.strict_domain().unwrap();
        assert!((range.1 - range.0 - std::f64::consts::FRAC_PI_2).abs() < 1e-12);

        let start = topo.vertex(edge.start()).unwrap().point();
        let end = topo.vertex(edge.end()).unwrap().point();
        assert!(
            (edge.curve().evaluate_with_endpoints(range.0, start, end) - start).length() < 1e-12
        );
        assert!((edge.curve().evaluate_with_endpoints(range.1, start, end) - end).length() < 1e-12);
        let midpoint =
            edge.curve()
                .evaluate_with_endpoints(f64::midpoint(range.0, range.1), start, end);
        assert!((midpoint - selected_midpoint).length() < 1e-12);
    }

    #[test]
    fn fillet_builder_empty_edges_error() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);

        let builder = FilletBuilder::new(&mut topo, solid);
        let result = builder.build();
        assert!(result.is_err(), "empty edge set should produce an error");
    }

    #[test]
    fn fillet_builder_plane_plane_box_edge() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);

        let adjacency = AdjacencyIndex::build(&topo, solid).unwrap();
        let shell_id = topo.solid(solid).unwrap().outer_shell();
        let faces = topo.shell(shell_id).unwrap().faces().to_vec();

        let mut target_edge = None;
        'outer: for &fid in &faces {
            let face = topo.face(fid).unwrap();
            let wire = topo.wire(face.outer_wire()).unwrap();
            for oe in wire.edges() {
                let adj = adjacency.faces_for_edge(oe.edge());
                if adj.len() == 2 {
                    target_edge = Some(oe.edge());
                    break 'outer;
                }
            }
        }
        let target_edge = target_edge.expect("cube should have manifold edges");

        let original_face_count = faces.len();
        let mut builder = FilletBuilder::new(&mut topo, solid);
        builder.add_edges(&[target_edge], 0.1);
        let result = builder.build().expect("fillet build should succeed");

        let result_solid = topo.solid(result.solid).unwrap();
        let result_shell = topo.shell(result_solid.outer_shell()).unwrap();

        // More faces than the original (6 original + 1 blend, minus possibly trimmed).
        assert!(
            result_shell.faces().len() > original_face_count,
            "expected more faces after fillet: got {}, original {}",
            result_shell.faces().len(),
            original_face_count,
        );

        assert!(result.succeeded.contains(&target_edge));
        assert!(result.failed.is_empty());
        assert!(!result.is_partial);

        let mut found_cylinder = false;
        for &fid in result_shell.faces() {
            let face = topo.face(fid).unwrap();
            if matches!(face.surface(), FaceSurface::Cylinder(_)) {
                found_cylinder = true;
            }
        }
        assert!(
            found_cylinder,
            "fillet should produce a cylindrical blend surface"
        );
    }

    #[test]
    fn stripe_walk_preserves_custom_law_instead_of_linearizing_endpoints() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let adjacency = AdjacencyIndex::build(&topo, solid).unwrap();
        let edge = remus_topology::explorer::solid_edges(&topo, solid).unwrap()[0];
        let spine = Spine::from_single_edge(&topo, edge).unwrap();
        let law = RadiusLaw::Custom(Box::new(|t| 0.1 + 0.05 * t * t));

        let result = compute_stripe_for_spine(&topo, &adjacency, spine, &law).unwrap();
        let interior = result
            .stripe
            .sections
            .iter()
            .find(|section| section.t > 0.05 && section.t < 0.95)
            .unwrap();
        let expected = law.evaluate(interior.t);
        let endpoint_linear = 0.1 + 0.05 * interior.t;
        assert!((interior.radius - expected).abs() < 1e-9);
        assert!((interior.radius - endpoint_linear).abs() > 1e-4);
    }

    #[test]
    fn fillet_builder_refuses_invalid_law_before_topology_allocation() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let edge = remus_topology::explorer::solid_edges(&topo, solid).unwrap()[0];
        let before = (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
        );

        let mut builder = FilletBuilder::new(&mut topo, solid);
        builder.add_edges_with_law(
            &[edge],
            RadiusLaw::SCurve {
                start: 0.1,
                end: 0.0,
            },
        );
        assert!(matches!(
            builder.build(),
            Err(BlendError::InvalidInput { .. })
        ));
        assert_eq!(
            before,
            (
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_wires(),
                topo.num_faces(),
                topo.num_shells(),
                topo.num_solids(),
            )
        );
    }

    #[test]
    fn fillet_builder_does_not_validate_an_unused_empty_edge_law() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let edge = remus_topology::explorer::solid_edges(&topo, solid).unwrap()[0];

        let mut builder = FilletBuilder::new(&mut topo, solid);
        builder.add_edges_with_law(&[], RadiusLaw::Constant(f64::NAN));
        builder.add_edges(&[edge], 0.1);
        let result = builder.build().unwrap();
        assert_eq!(result.succeeded, vec![edge]);
    }

    #[test]
    fn fillet_builder_records_failed_edges() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);

        let v0 = topo.add_vertex(remus_topology::vertex::Vertex::new(
            remus_math::vec::Point3::new(10.0, 10.0, 10.0),
            1e-7,
        ));
        let v1 = topo.add_vertex(remus_topology::vertex::Vertex::new(
            remus_math::vec::Point3::new(11.0, 10.0, 10.0),
            1e-7,
        ));
        let fake_edge = topo.add_edge(remus_topology::edge::Edge::new(
            v0,
            v1,
            remus_topology::edge::EdgeCurve::Line,
        ));

        let mut builder = FilletBuilder::new(&mut topo, solid);
        builder.add_edges(&[fake_edge], 0.2);
        let result = builder.build().expect("build should succeed (partial)");

        assert!(result.failed.len() == 1);
        assert_eq!(result.failed[0].0, fake_edge);
        assert!(result.is_partial);
        // With no successes, the original solid is returned.
        assert_eq!(result.solid, solid);
    }
}
