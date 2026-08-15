//! Fillet builder: orchestrates the full fillet pipeline.
//!
//! Spine construction, analytic/walking stripe computation, face trimming,
//! and solid assembly. Supports constant and variable radius fillets on
//! planar face pairs (v1).

use std::collections::{HashMap, HashSet};

use brepkit_math::curves::Circle3D;
use brepkit_math::vec::{Point3, Vec3};
use brepkit_topology::Topology;
use brepkit_topology::edge::{Edge, EdgeCurve, EdgeId};
use brepkit_topology::face::{Face, FaceId, FaceSurface};
use brepkit_topology::shell::Shell;
use brepkit_topology::solid::{Solid, SolidId};
use brepkit_topology::vertex::{Vertex, VertexId};
use brepkit_topology::wire::{OrientedEdge, Wire, WireId};

use crate::analytic;
use crate::blend_func::{ConstRadBlend, EvolRadBlend};
use crate::builder_utils::{
    FlippedNormalSurface, project_onto_axis, radial_distance, sample_nurbs_endpoints,
    surface_ref_or_adapter, wire_axial_range, wire_radial_extremum,
};
use crate::corner;
use crate::g1_chain;
use crate::radius_law::RadiusLaw;
use crate::spine::Spine;
use crate::stripe::{Stripe, StripeResult};
use crate::trimmer;
use crate::walker::{Walker, WalkerConfig, approximate_blend_surface};
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
            return Err(BlendError::Topology(
                brepkit_topology::TopologyError::Empty {
                    entity: "fillet edge set",
                },
            ));
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
        let tol = brepkit_math::tolerance::Tolerance::new();
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
            let mut chains_at_vertex: HashMap<usize, (brepkit_topology::vertex::VertexId, usize)> =
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
                Err(e @ BlendError::RadiusTooLarge { .. }) => return Err(e),
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
            Option<brepkit_topology::edge::EdgeId>,
            Option<brepkit_topology::edge::EdgeId>,
        )> = Vec::new();
        for sr in &regular_results {
            let stripe = &sr.stripe;
            stripe_contact_edges.push((None, None));

            let contact1_pts = sample_nurbs_endpoints(&stripe.contact1);
            let contact2_pts = sample_nurbs_endpoints(&stripe.contact2);

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
                &contact1_pts,
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
                &contact2_pts,
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
            brepkit_topology::edge::EdgeId,
            brepkit_topology::vertex::VertexId,
            brepkit_topology::vertex::VertexId,
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

        for &fid in &touched_faces {
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
    use brepkit_math::traits::ParametricCurve;
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
    let (start, end) = if (mid_of(pa, pb) - v_pt).length() <= (mid_of(pb, pa) - v_pt).length() {
        (c_a, c_b)
    } else {
        (c_b, c_a)
    };
    Ok(Some(topo.add_edge(Edge::new(
        start,
        end,
        EdgeCurve::Circle(circle),
    ))))
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
        brepkit_topology::wire::WireId,
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
        *topo.wire_mut(wid)? = Wire::new(new_edges, true)?;
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
    {
        let e = topo.edge(rim_edge)?;
        if e.start() != e.end() {
            return Ok(None);
        }
        if !matches!(e.curve(), EdgeCurve::Circle(_)) {
            return Ok(None);
        }
    }

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
    let plate_circle = Circle3D::new_with_ref(plate_center, axis, plate_radius, seam_dir)?;
    let wall_circle = Circle3D::new_with_ref(wall_center, axis, wall_radius, seam_dir)?;

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

    // Vertices for the two closed contact circles (start == end → degenerate).
    let plate_point = rim.plate_circle.evaluate(0.0);
    let wall_point = rim.wall_circle.evaluate(0.0);
    let plate_v = topo.add_vertex(Vertex::new(plate_point, TOL));
    let wall_v = topo.add_vertex(Vertex::new(wall_point, TOL));

    // Shared contact-circle edges.
    let plate_edge = topo.add_edge(Edge::new(
        plate_v,
        plate_v,
        EdgeCurve::Circle(rim.plate_circle.clone()),
    ));
    let wall_edge = topo.add_edge(Edge::new(
        wall_v,
        wall_v,
        EdgeCurve::Circle(rim.wall_circle.clone()),
    ));
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
    let seam_circle = Circle3D::new(seam_center, seam_normal, torus.minor_radius())?;
    let seam_edge = topo.add_edge(Edge::new(plate_v, wall_v, EdgeCurve::Circle(seam_circle)));

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
    let old_rim_vertex = topo.edge(rim.rim_edge)?.start();
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
                let curve = e.curve().clone();
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
                let id = topo.add_edge(Edge::new(new_start, new_end, curve));
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
    torus: &brepkit_math::surfaces::ToroidalSurface,
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

/// Compute a stripe for a single edge using the adjacency index.
///
/// # Errors
///
/// Returns [`BlendError`] if the edge is non-manifold, if topology lookups
/// fail, or if neither the analytic nor walking path can produce a result.
#[allow(clippy::too_many_lines)]
fn compute_stripe_for_spine(
    topo: &Topology,
    adjacency: &brepkit_topology::adjacency::AdjacencyIndex,
    spine: Spine,
    law: &RadiusLaw,
) -> Result<StripeResult, BlendError> {
    // Every edge on a G1 chain shares one face pair (that is what makes it a
    // ridgeline), so the first edge speaks for the whole spine.
    let edge_id = spine.edges().first().copied().ok_or(BlendError::Topology(
        brepkit_topology::TopologyError::Empty {
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
    if matches!(law, RadiusLaw::Constant(_)) {
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
        target: "brepkit_approx",
        "fillet: analytic fast-path unavailable for {}+{} ({} radius) — using Newton-Raphson walker (approximate NURBS blend surface)",
        surf1.type_tag(),
        surf2.type_tag(),
        if matches!(law, RadiusLaw::Constant(_)) { "constant" } else { "variable" }
    );

    // Build ParametricSurface references via PlaneAdapter for planes.
    // When a face is reversed, the outward normal is flipped. For PlaneAdapter,
    // we negate the normal. For analytic/NURBS surfaces the ParametricSurface
    // impl already returns the geometric normal; the walker uses the sign
    // convention from the face orientation.
    let oriented_surf1 = if face1_reversed {
        orient_plane_surface(&surf1)
    } else {
        surf1
    };
    let oriented_surf2 = if face2_reversed {
        orient_plane_surface(&surf2)
    } else {
        surf2
    };
    let mut adapter1 = None;
    let mut adapter2 = None;

    let ps1 = surface_ref_or_adapter(&oriented_surf1, &mut adapter1);
    let ps2 = surface_ref_or_adapter(&oriented_surf2, &mut adapter2);

    let config = WalkerConfig::default();

    let walk_result = if let RadiusLaw::Constant(r) = law {
        let blend = ConstRadBlend { radius: *r };
        let walker = Walker::new(&blend, ps1, ps2, &spine, topo, config);
        let start = walker.find_start(0.0)?;
        walker.walk(start, 0.0, spine.length())?
    } else {
        let evol = EvolRadBlend {
            law: mirror_law(law),
        };
        let walker = Walker::new(&evol, ps1, ps2, &spine, topo, config);
        let start = walker.find_start(0.0)?;
        walker.walk(start, 0.0, spine.length())?
    };

    let blend_surface = approximate_blend_surface(&walk_result.sections)?;
    let blend_face_surface = brepkit_topology::face::FaceSurface::Nurbs(blend_surface);

    let contact1 = sections_to_contact_curve(&walk_result.sections, |s| s.p1)?;
    let contact2 = sections_to_contact_curve(&walk_result.sections, |s| s.p2)?;

    let pcurve1 = build_pcurve_from_contact(ps1, &contact1)?;
    let pcurve2 = build_pcurve_from_contact(ps2, &contact2)?;

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
    pub contact1: brepkit_math::vec::Point3,
    /// Arc apex / middle control point (tangent intersection).
    pub apex: brepkit_math::vec::Point3,
    /// Contact point on the second surface (`u = 1` end of the arc).
    pub contact2: brepkit_math::vec::Point3,
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
    surf1: &brepkit_topology::face::FaceSurface,
    surf1_reversed: bool,
    surf2: &brepkit_topology::face::FaceSurface,
    surf2_reversed: bool,
    radius: f64,
    fractions: &[f64],
) -> Result<Vec<BlendCrossSection>, BlendError> {
    use brepkit_math::vec::Point3;

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
    let ps1: &dyn brepkit_math::traits::ParametricSurface =
        if surf1_reversed { base1 } else { &flip1 };
    let ps2: &dyn brepkit_math::traits::ParametricSurface =
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
    surface: &brepkit_topology::face::FaceSurface,
) -> brepkit_topology::face::FaceSurface {
    match surface {
        brepkit_topology::face::FaceSurface::Plane { normal, d } => {
            brepkit_topology::face::FaceSurface::Plane {
                normal: -*normal,
                d: -*d,
            }
        }
        other => other.clone(),
    }
}

/// Mirror a `RadiusLaw` into a new instance with the same behavior.
///
/// This is needed because `RadiusLaw::Custom` contains a `Box<dyn Fn>`
/// which is not `Clone`. For non-custom laws, we reconstruct the same
/// variant. For custom laws, we evaluate at a fixed set of points and
/// create a linear interpolation.
fn mirror_law(law: &RadiusLaw) -> RadiusLaw {
    match law {
        RadiusLaw::Constant(r) => RadiusLaw::Constant(*r),
        RadiusLaw::Linear { start, end } => RadiusLaw::Linear {
            start: *start,
            end: *end,
        },
        RadiusLaw::SCurve { start, end } => RadiusLaw::SCurve {
            start: *start,
            end: *end,
        },
        RadiusLaw::Custom(_) => {
            // Sample the custom law at endpoints and build a linear
            // approximation. This is a v1 simplification; a proper
            // implementation would share the closure via Arc.
            let r0 = law.evaluate(0.0);
            let r1 = law.evaluate(1.0);
            RadiusLaw::Linear { start: r0, end: r1 }
        }
    }
}

/// Build a degree-1 NURBS curve from section contact points.
fn sections_to_contact_curve(
    sections: &[crate::section::CircSection],
    pick: impl Fn(&crate::section::CircSection) -> brepkit_math::vec::Point3,
) -> Result<brepkit_math::nurbs::curve::NurbsCurve, BlendError> {
    let pts: Vec<brepkit_math::vec::Point3> = sections.iter().map(&pick).collect();
    if pts.len() < 2 {
        return Err(BlendError::Math(brepkit_math::MathError::EmptyInput));
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
    let curve = brepkit_math::nurbs::curve::NurbsCurve::new(degree, knots, pts, weights)?;
    Ok(curve)
}

/// Build a PCurve (2D UV line) by projecting 3D contact endpoints onto a surface.
fn build_pcurve_from_contact(
    surf: &dyn brepkit_math::traits::ParametricSurface,
    contact: &brepkit_math::nurbs::curve::NurbsCurve,
) -> Result<brepkit_math::curves2d::Curve2D, BlendError> {
    let (t0, t1) = contact.domain();
    let p_start = contact.evaluate(t0);
    let p_end = contact.evaluate(t1);

    let (u0, v0) = surf.project_point(p_start);
    let (u1, v1) = surf.project_point(p_end);

    let origin = brepkit_math::vec::Point2::new(u0, v0);
    let dir = brepkit_math::vec::Vec2::new(u1 - u0, v1 - v0);

    let line = brepkit_math::curves2d::Line2D::new(origin, dir)?;
    Ok(brepkit_math::curves2d::Curve2D::Line(line))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use brepkit_topology::adjacency::AdjacencyIndex;
    use brepkit_topology::face::FaceSurface;
    use brepkit_topology::test_utils::make_unit_cube_manifold;

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
    fn fillet_builder_records_failed_edges() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);

        let v0 = topo.add_vertex(brepkit_topology::vertex::Vertex::new(
            brepkit_math::vec::Point3::new(10.0, 10.0, 10.0),
            1e-7,
        ));
        let v1 = topo.add_vertex(brepkit_topology::vertex::Vertex::new(
            brepkit_math::vec::Point3::new(11.0, 10.0, 10.0),
            1e-7,
        ));
        let fake_edge = topo.add_edge(brepkit_topology::edge::Edge::new(
            v0,
            v1,
            brepkit_topology::edge::EdgeCurve::Line,
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
