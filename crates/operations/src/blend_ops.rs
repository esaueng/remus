//! Thin wrappers around `brepkit-blend` for the operations API.

use brepkit_blend::BlendResult;
use brepkit_blend::chamfer_builder::ChamferBuilder;
use brepkit_blend::fillet_builder::FilletBuilder;
pub use brepkit_blend::{BlendError, BlendFaceOrigins};
use brepkit_topology::Topology;
use brepkit_topology::edge::{EdgeCurve, EdgeId};
use brepkit_topology::face::FaceSurface;
use brepkit_topology::solid::SolidId;

use crate::OperationsError;
use crate::evolution::EvolutionMap;

/// Run a blend attempt transactionally: on any failure, roll the arena back
/// to its pre-attempt state.
///
/// The blend engines mutate shared topology in place — `trim_face`'s
/// `propagate_split` rewrites the wires of every face referencing a split
/// edge, and the stitched assembly rewrites cap wires. A failure partway
/// through therefore leaves the INPUT solid mutated: trimmed side faces,
/// arcs where sharp corners were, and free edges from half-applied splits.
/// A caller that reports the failure and keeps using its original solid
/// handle (as the OpenZCAD adapter does) then ships a corrupted body that
/// meshes with holes. Snapshot/restore makes a failed blend a true no-op.
///
/// Handle slots are preserved so IDs handed out before the attempt stay
/// valid after a rollback.
fn transactional<T>(
    topo: &mut Topology,
    attempt: impl FnOnce(&mut Topology) -> Result<T, OperationsError>,
) -> Result<T, OperationsError> {
    let snapshot = topo.clone();
    match attempt(topo) {
        Ok(value) => Ok(value),
        Err(e) => {
            topo.restore_preserving_handle_slots(&snapshot);
            Err(e)
        }
    }
}

/// Collapse repeated seed edges, keeping the caller's order.
///
/// Selections are routinely assembled from face adjacency, where every shared
/// edge is named once per face; a repeat carries no extra information about the
/// geometry, so it is dropped rather than refused. First occurrence wins
/// because blend results depend on the order the seeds are walked in.
fn dedup_seed_edges(edges: &[EdgeId]) -> Vec<EdgeId> {
    let mut seen = std::collections::HashSet::with_capacity(edges.len());
    let mut unique = Vec::with_capacity(edges.len());
    for &edge in edges {
        if seen.insert(edge) {
            unique.push(edge);
        }
    }
    unique
}

/// Reject a seed selection that names more distinct edges than the solid has.
///
/// The only defensible ceiling on a blend selection is the body itself.
/// "Blend every edge" is everyday work — baseplates, heat sinks, lattices run
/// to hundreds or thousands of edges — so it must be admissible by
/// construction, while a de-duplicated selection larger than the solid's own
/// edge count cannot be naming its geometry and is bounded input abuse.
///
/// Call with an already de-duplicated selection.
fn reject_seed_selection_larger_than_solid(
    topo: &Topology,
    solid: SolidId,
    operation: &str,
    edges: &[EdgeId],
) -> Result<(), OperationsError> {
    let available = brepkit_topology::explorer::solid_edges(topo, solid)?.len();
    if edges.len() > available {
        return Err(OperationsError::InvalidInput {
            reason: format!(
                "{operation} selection names {} distinct edges but the solid has {available}",
                edges.len()
            ),
        });
    }
    Ok(())
}

/// Classify a blended edge as convex (material on the inside of the dihedral)
/// or concave, by testing a point just inside the inward normal bisector.
///
/// Returns `None` when the edge's neighbourhood cannot be classified
/// (non-manifold edge, degenerate normals, on-boundary sample).
pub(crate) fn edge_is_convex(
    topo: &Topology,
    solid: SolidId,
    edge: EdgeId,
    probe: f64,
) -> Option<bool> {
    let adjacency = topo.build_adjacency(solid).ok()?;
    let faces = adjacency.faces_for_edge(edge);
    if faces.len() != 2 {
        return None;
    }
    let e = topo.edge(edge).ok()?;
    let start = topo.vertex(e.start()).ok()?.point();
    let end = topo.vertex(e.end()).ok()?.point();

    // For planar faces, use only the two boundary edges incident to the
    // target edge.  Looking at the complete face boundary makes this local
    // property depend on unrelated holes or distant concave portions.
    let face1_data = topo.face(faces[0]).ok()?;
    let face2_data = topo.face(faces[1]).ok()?;
    if let (FaceSurface::Plane { normal: n1, .. }, FaceSurface::Plane { normal: n2, .. }) =
        (face1_data.surface(), face2_data.surface())
    {
        let inward1 = if face1_data.is_reversed() { *n1 } else { -*n1 };
        let inward2 = if face2_data.is_reversed() { *n2 } else { -*n2 };
        let local_witness = |face: &brepkit_topology::face::Face,
                             other_normal: brepkit_math::vec::Vec3|
         -> Option<f64> {
            for wire_id in
                std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                let wire = topo.wire(wire_id).ok()?;
                let edges = wire.edges();
                let Some(index) = edges.iter().position(|oriented| oriented.edge() == edge) else {
                    continue;
                };
                let mut extreme = 0.0_f64;
                for neighbor in [
                    edges[(index + edges.len() - 1) % edges.len()].edge(),
                    edges[(index + 1) % edges.len()].edge(),
                ] {
                    let neighbor = topo.edge(neighbor).ok()?;
                    for vertex in [neighbor.start(), neighbor.end()] {
                        if vertex != e.start() && vertex != e.end() {
                            let signed =
                                other_normal.dot(topo.vertex(vertex).ok()?.point() - start);
                            if signed.abs() > extreme.abs() {
                                extreme = signed;
                            }
                        }
                    }
                }
                return Some(extreme);
            }
            None
        };
        let w1 = local_witness(face1_data, inward2)?;
        let w2 = local_witness(face2_data, inward1)?;
        if w1.abs() > 1e-9 && w2.abs() > 1e-9 {
            return Some(!(w1 < -1e-9 && w2 < -1e-9));
        }
    }

    let mid = e.curve().evaluate_with_endpoints(
        match e.curve() {
            EdgeCurve::Line => 0.5,
            other => {
                let (t0, t1) = other.domain_with_endpoints(start, end);
                f64::midpoint(t0, t1)
            }
        },
        start,
        end,
    );

    let outward = |fid: brepkit_topology::face::FaceId| {
        let face = topo.face(fid).ok()?;
        let (u, v) = face.surface().project_point(mid)?;
        let n = face.surface().normal(u, v);
        let n = if face.is_reversed() { -n } else { n };
        n.normalize().ok()
    };
    let n1 = outward(faces[0])?;
    let n2 = outward(faces[1])?;
    let bisector = (n1 + n2).normalize().ok()?;

    // Step inward along the bisector. Inside the material ⇒ convex edge.
    let sample = mid - bisector * probe;
    match crate::classify::classify_point_robust(topo, solid, sample, 0.01, 1e-7).ok()? {
        crate::classify::PointClassification::Inside => Some(true),
        crate::classify::PointClassification::Outside => Some(false),
        crate::classify::PointClassification::OnBoundary => None,
    }
}

/// Reject a blend whose volume change is geometrically impossible.
///
/// A blend only moves material inside a tube of radius `size` around each
/// blended edge, so `|Δvolume|` is bounded by `size²·length` per edge plus
/// `2·size³` of end effects. And the sign is fixed by convexity: rounding a
/// convex edge cuts material away, a concave one fills it in. A result that
/// breaks either rule is wrong even when it is a topologically valid closed
/// solid — the failure mode a wrong-side trim produces, which the shell and
/// Euler checks alone accept.
fn validate_blend_volume(
    topo: &Topology,
    operation: &'static str,
    input_solid: SolidId,
    result_solid: SolidId,
    edges: &[EdgeId],
    size: f64,
) -> Result<(), OperationsError> {
    let before = crate::measure::solid_volume(topo, input_solid, 0.1)?;
    let after = crate::measure::solid_volume(topo, result_solid, 0.1)?;
    let delta = after - before;

    let mut budget = 0.0;
    for &edge in edges {
        let e = topo.edge(edge)?;
        let start = topo.vertex(e.start())?.point();
        let end = topo.vertex(e.end())?.point();
        let length = if e.start() == e.end() {
            // Closed edge: use the curve's own extent.
            let (t0, t1) = e.curve().domain_with_endpoints(start, end);
            let mut len = 0.0;
            let mut prev = e.curve().evaluate_with_endpoints(t0, start, end);
            for i in 1..=32 {
                let t = t0 + (t1 - t0) * f64::from(i) / 32.0;
                let p = e.curve().evaluate_with_endpoints(t, start, end);
                len += (p - prev).length();
                prev = p;
            }
            len
        } else {
            (end - start).length()
        };
        budget += size * size * length + 2.0 * size * size * size;
    }

    if delta.abs() > budget {
        return Err(OperationsError::InvalidInput {
            reason: format!(
                "{operation} changed volume by {delta:+.3}, beyond the {budget:.3} \
                 a blend of this size can move — the result is geometrically wrong"
            ),
        });
    }

    // Sign rule, applied only when every blended edge shares one convexity
    // (a mixed set can legitimately net out either way).
    let convexities: Vec<bool> = edges
        .iter()
        .filter_map(|&e| {
            match crate::query::edge_concavity(topo, input_solid, e, size * 0.25).ok()? {
                crate::query::EdgeConcavity::Convex => Some(true),
                crate::query::EdgeConcavity::Concave => Some(false),
                crate::query::EdgeConcavity::Tangent | crate::query::EdgeConcavity::Unknown => None,
            }
        })
        .collect();
    if convexities.len() == edges.len() && !convexities.is_empty() {
        let all_convex = convexities.iter().all(|&c| c);
        let all_concave = convexities.iter().all(|&c| !c);
        // Allow a hair of tessellation noise either way.
        let noise = budget * 1e-3;
        if all_convex && delta > noise {
            return Err(OperationsError::InvalidInput {
                reason: format!(
                    "{operation} on convex edges added {delta:+.3} of material; \
                     rounding a convex edge must remove it"
                ),
            });
        }
        if all_concave && delta < -noise {
            return Err(OperationsError::InvalidInput {
                reason: format!(
                    "{operation} on concave edges removed {:.3} of material; \
                     rounding a concave edge must add it",
                    -delta
                ),
            });
        }
    }

    Ok(())
}

/// Per-check error magnitudes of a solid's validation report: the summed
/// deviation (or 1 per issue when absent) of Error-severity issues.
fn error_magnitudes(
    topo: &Topology,
    solid: SolidId,
) -> Result<std::collections::HashMap<brepkit_check::validate::CheckId, f64>, OperationsError> {
    let report = brepkit_check::validate::validate_solid(
        topo,
        solid,
        &brepkit_check::validate::ValidateOptions::default(),
    )?;
    let mut map = std::collections::HashMap::new();
    for issue in &report.issues {
        if issue.severity == brepkit_check::validate::Severity::Error {
            *map.entry(issue.check).or_insert(0.0) += issue.deviation.unwrap_or(1.0);
        }
    }
    Ok(map)
}

/// Validate the blend result against the INPUT solid as a baseline: defects
/// already present in the input (e.g. boolean-inherited orientation quirks
/// on closed circle edges) do not fail the blend; only regressions do.
fn validate_complete_blend(
    topo: &Topology,
    operation: &'static str,
    input_solid: SolidId,
    result: &BlendResult,
) -> Result<(), OperationsError> {
    if result.is_partial {
        return Err(OperationsError::PartialResult {
            operation,
            succeeded: result.succeeded.len(),
            failed: result.failed.len(),
        });
    }
    let report = brepkit_check::validate::validate_solid(
        topo,
        result.solid,
        &brepkit_check::validate::ValidateOptions::default(),
    )?;
    if report.is_valid() {
        return Ok(());
    }
    let after = {
        let mut map = std::collections::HashMap::new();
        for issue in &report.issues {
            if issue.severity == brepkit_check::validate::Severity::Error {
                *map.entry(issue.check).or_insert(0.0) += issue.deviation.unwrap_or(1.0);
            }
        }
        map
    };
    let before = error_magnitudes(topo, input_solid)?;
    let regressed = after
        .iter()
        .any(|(check, &mag)| mag > before.get(check).copied().unwrap_or(0.0));
    if regressed {
        let summary = report
            .issues
            .iter()
            .filter(|issue| issue.severity == brepkit_check::validate::Severity::Error)
            .take(3)
            .map(|issue| issue.description.as_str())
            .collect::<Vec<_>>()
            .join("; ");
        return Err(OperationsError::InvalidInput {
            reason: format!(
                "{operation} postcondition validation failed with {} error(s): {summary}",
                report.error_count(),
            ),
        });
    }
    Ok(())
}

/// Shortest distance from `p` to the segment `a`–`b`.
fn point_segment_distance(
    p: brepkit_math::vec::Point3,
    a: brepkit_math::vec::Point3,
    b: brepkit_math::vec::Point3,
) -> f64 {
    let ab = b - a;
    let len_sq = ab.dot(ab);
    if len_sq <= 0.0 {
        return (p - a).length();
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    (p - (a + ab * t)).length()
}

/// Sample a closed or open edge into points, endpoints included.
fn sample_edge(
    topo: &Topology,
    edge: EdgeId,
    samples: usize,
) -> Result<Vec<brepkit_math::vec::Point3>, OperationsError> {
    let e = topo.edge(edge)?;
    let start = topo.vertex(e.start())?.point();
    let end = topo.vertex(e.end())?.point();
    let (t0, t1) = e.curve().domain_with_endpoints(start, end);
    Ok((0..=samples)
        .map(|i| {
            let t = t0
                + (t1 - t0) * f64::from(u32::try_from(i).unwrap_or(u32::MAX))
                    / f64::from(u32::try_from(samples).unwrap_or(1).max(1));
            e.curve().evaluate_with_endpoints(t, start, end)
        })
        .collect())
}

/// Refuse a blend whose setback would run into a hole in one of its own
/// neighbouring faces.
///
/// The blend's contact curve on a planar neighbour lies `size` away from the
/// blended edge, so the neighbour's boundary is rebuilt that far in. If an
/// inner loop of that face — a drilled hole, a pocket — sits closer than
/// `size`, the rebuilt boundary crosses its own hole. Expressing that would
/// require the blend and the hole to merge into one surface, which none of
/// these engines can do.
///
/// Emitting it anyway is the dangerous option: a wire crossing its own inner
/// loop still passes the closed-shell and Euler checks, so the result looks
/// valid and meshes into a self-intersecting body. The cause is genuinely the
/// radius — the same edge blends fine below the clearance — so this reports
/// `RadiusTooLarge` with the clearance as the achievable maximum.
///
/// Only straight blended edges are considered: a closed rim edge IS an inner
/// loop, and the analytic rim assemblers have their own clearance guard.
fn reject_blend_into_hole(
    topo: &Topology,
    solid: SolidId,
    edges: &[EdgeId],
    size: f64,
) -> Result<(), OperationsError> {
    let adjacency = topo.build_adjacency(solid)?;
    for &edge_id in edges {
        let e = topo.edge(edge_id)?;
        if !matches!(e.curve(), EdgeCurve::Line) {
            continue;
        }
        let a = topo.vertex(e.start())?.point();
        let b = topo.vertex(e.end())?.point();

        for &face_id in adjacency.faces_for_edge(edge_id) {
            let face = topo.face(face_id)?;
            if !matches!(face.surface(), FaceSurface::Plane { .. }) {
                continue;
            }
            for &inner in face.inner_wires() {
                let wire_edges: Vec<_> = topo.wire(inner)?.edges().to_vec();
                if wire_edges.iter().any(|oe| oe.edge() == edge_id) {
                    // The blended edge is part of this loop; the loop is the
                    // blend's own spine, not an obstacle.
                    continue;
                }
                let mut clearance = f64::INFINITY;
                for oe in &wire_edges {
                    for p in sample_edge(topo, oe.edge(), 64)? {
                        clearance = clearance.min(point_segment_distance(p, a, b));
                    }
                }
                if clearance <= size {
                    return Err(OperationsError::Blend(BlendError::RadiusTooLarge {
                        edge: edge_id,
                        max_radius: clearance,
                    }));
                }
            }
        }
    }
    Ok(())
}

/// Return whether every requested edge is a manifold line between two planar
/// faces. These inputs are handled by the polygon-rebuilding chamfer path,
/// which also closes the two end faces of a finite chamfer. The walking
/// builder remains necessary for analytic curved edges and surfaces.
fn is_planar_line_blend(
    topo: &Topology,
    solid: SolidId,
    edges: &[EdgeId],
) -> Result<bool, OperationsError> {
    let adjacency = topo.build_adjacency(solid)?;

    for &edge_id in edges {
        if !matches!(topo.edge(edge_id)?.curve(), EdgeCurve::Line) {
            return Ok(false);
        }

        let faces = adjacency.faces_for_edge(edge_id);
        if faces.len() != 2 {
            return Ok(false);
        }
        for &face_id in faces {
            if !matches!(topo.face(face_id)?.surface(), FaceSurface::Plane { .. }) {
                return Ok(false);
            }
        }
    }

    Ok(true)
}
fn planar_chamfer_result(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    d1: f64,
    d2: f64,
) -> Result<BlendResult, OperationsError> {
    let (result_solid, face_origins) =
        crate::chamfer::chamfer_asymmetric_with_origins(topo, solid, edges, d1, d2)?;
    let result = BlendResult {
        solid: result_solid,
        succeeded: edges.to_vec(),
        failed: Vec::new(),
        is_partial: false,
        face_origins: Some(face_origins),
    };
    validate_complete_blend(topo, "chamfer", solid, &result)?;
    // The fast path gets the same volume guard as the walking path. Closedness
    // and manifoldness alone do not prove a bevel is right: a setback that
    // overruns its face folds the polygon through itself and still validates.
    validate_blend_volume(topo, "chamfer", solid, result_solid, edges, d1.max(d2))?;
    Ok(result)
}

/// Run the production planar chamfer engine and return its construction
/// history. This exists for the WASM engine cascade, which must preserve the
/// same first-choice geometry as `chamfer` while exposing provenance.
#[doc(hidden)]
pub fn planar_chamfer_with_origins(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    distance: f64,
) -> Result<(SolidId, BlendFaceOrigins), OperationsError> {
    crate::chamfer::chamfer_with_origins(topo, solid, edges, distance)
}

#[allow(deprecated)]
fn planar_fillet_result(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    radius: f64,
) -> Result<BlendResult, OperationsError> {
    let (result_solid, face_origins) =
        crate::fillet::fillet_rolling_ball_with_origins(topo, solid, edges, radius)?;
    let result = BlendResult {
        solid: result_solid,
        succeeded: edges.to_vec(),
        failed: Vec::new(),
        is_partial: false,
        face_origins: Some(face_origins),
    };
    validate_complete_blend(topo, "fillet", solid, &result)?;
    // The fast path gets the same volume guard as the walking path — and needs
    // it more since it learned to rebuild holed caps: a setback that crosses an
    // inner loop folds the cap through itself and still validates as a closed
    // 2-manifold. Only the volume rules catch that.
    validate_blend_volume(topo, "fillet", solid, result_solid, edges, radius)?;
    Ok(result)
}

/// Sample an edge into a polyline whose chords are no longer than `step`.
///
/// The count is capped so a very long edge beside a very small radius cannot
/// turn a proximity test into a quadratic blow-up; at the cap the chords are
/// coarser than `step`, which the caller absorbs with its own slack.
fn edge_polyline(
    topo: &Topology,
    edge: EdgeId,
    step: f64,
) -> Result<Vec<brepkit_math::vec::Point3>, OperationsError> {
    const MAX_SAMPLES: usize = 256;
    let coarse = sample_edge(topo, edge, 8)?;
    let rough: f64 = coarse.windows(2).map(|w| (w[1] - w[0]).length()).sum();
    let wanted = if step > 0.0 && rough.is_finite() {
        (rough / step).ceil().max(8.0)
    } else {
        8.0
    };
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let samples = if wanted >= MAX_SAMPLES as f64 {
        MAX_SAMPLES
    } else {
        wanted as usize
    };
    sample_edge(topo, edge, samples)
}

/// Shortest distance between two sampled polylines.
fn polyline_distance(a: &[brepkit_math::vec::Point3], b: &[brepkit_math::vec::Point3]) -> f64 {
    let mut best = f64::INFINITY;
    for &p in a {
        for w in b.windows(2) {
            best = best.min(point_segment_distance(p, w[0], w[1]));
        }
    }
    for &p in b {
        for w in a.windows(2) {
            best = best.min(point_segment_distance(p, w[0], w[1]));
        }
    }
    best
}

/// Union-find root of `i`, path-compressing on the way up.
fn union_find_root(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    i
}

/// Partition a selection into groups of edges whose blends cannot reach each
/// other.
///
/// A constant-size blend only reshapes material within `size` of its own edge,
/// so two edges further apart than `2·size` — and not joined through the same
/// tangent-continuous ridgeline — round into surfaces that never meet. They
/// are separate features that happen to have been named in one call, and
/// nothing about how one of them is built constrains the other.
///
/// This matters because the two fillet engines are complementary rather than
/// ranked. The planar rebuild is the only one that closes a vertex blend where
/// two rounded edges meet at a corner; the walking builder is the only one
/// that assembles a closed rim. The choice between them was made once for the
/// whole selection, on an all-or-nothing "every edge is a straight line
/// between two planes" test, so a plate's top perimeter picked together with a
/// bore rim sent the perimeter to the walking builder as well — and that
/// builder refuses every corner it is handed. The rim's own seam vertex was
/// never the problem: the vertex named in the refusal is a plate corner far
/// from the bore.
///
/// Grouping is deliberately conservative: any pair that could possibly interact
/// stays in one group, so nothing is ever applied in sequence that has to be
/// solved together. It is also only consulted after one engine has already
/// refused the selection whole, so a selection that works today never reaches
/// it at all — four corner edges of a box are `2·size` apart and would split,
/// but the planar rebuild takes them together and is never asked twice.
///
/// Groups keep the caller's edge order, and are themselves ordered by first
/// appearance, so the partition is deterministic.
fn independent_blend_groups(
    topo: &Topology,
    solid: SolidId,
    edges: &[EdgeId],
    size: f64,
) -> Result<Vec<Vec<EdgeId>>, OperationsError> {
    if edges.len() < 2 {
        return Ok(vec![edges.to_vec()]);
    }
    let tol = brepkit_math::tolerance::Tolerance::new();

    // What each seed actually blends: both engines expand a seed to its whole
    // G1 ridgeline first, so two distant seeds on one smooth run belong
    // together even though the seeds themselves are far apart.
    let mut chains: Vec<Vec<EdgeId>> = Vec::with_capacity(edges.len());
    let mut outlines: Vec<Vec<brepkit_math::vec::Point3>> = Vec::with_capacity(edges.len());
    for &edge in edges {
        let chain = brepkit_blend::g1_chain::expand_g1_chain(topo, solid, &[edge], tol)?;
        let chain = if chain.is_empty() { vec![edge] } else { chain };
        let mut outline = Vec::new();
        for &member in &chain {
            outline.extend(edge_polyline(topo, member, size * 0.25)?);
        }
        chains.push(chain);
        outlines.push(outline);
    }

    // Union-find over the seeds.
    let mut parent: Vec<usize> = (0..edges.len()).collect();
    // A polyline chord may sit up to half its own length from the true curve,
    // so widen the reach test by that much rather than risk splitting a pair
    // that in fact touches.
    let reach = 2.0f64.mul_add(size, size * 0.25);
    for i in 0..edges.len() {
        for j in (i + 1)..edges.len() {
            let shares_ridgeline = chains[i].iter().any(|e| chains[j].contains(e));
            let touches =
                shares_ridgeline || polyline_distance(&outlines[i], &outlines[j]) <= reach;
            if touches {
                let (ri, rj) = (
                    union_find_root(&mut parent, i),
                    union_find_root(&mut parent, j),
                );
                if ri != rj {
                    parent[ri] = rj;
                }
            }
        }
    }

    let mut order: Vec<usize> = Vec::new();
    let mut groups: Vec<Vec<EdgeId>> = Vec::new();
    for i in 0..edges.len() {
        let root = union_find_root(&mut parent, i);
        let slot = if let Some(pos) = order.iter().position(|&r| r == root) {
            pos
        } else {
            order.push(root);
            groups.push(Vec::new());
            groups.len() - 1
        };
        groups[slot].push(edges[i]);
    }
    Ok(groups)
}

/// Fillet one group of edges, choosing the engine that fits its shape.
///
/// The planar rebuild is tried first when every edge is a straight line
/// between two planes — it is the only engine that closes a vertex blend — and
/// the walking builder takes everything else, including closed rims. Both
/// paths validate the group against the solid they were given.
fn fillet_group(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    radius: f64,
) -> Result<BlendResult, OperationsError> {
    if is_planar_line_blend(topo, solid, edges)? {
        // The rolling-ball rebuild handles the validated planar classes
        // (simple prisms), closes multi-edge corner patches, and carries the
        // inner loops of a holed cap through as topology. On richer topology
        // (L-shaped side faces, coplanar slivers) it emits an open shell; fall
        // through to the walking builder, whose stitched planar assembly
        // handles those shapes. Each attempt is transactional, so the
        // fall-through starts from a clean arena.
        match transactional(topo, |t| planar_fillet_result(t, solid, edges, radius)) {
            Ok(result) => return Ok(result),
            Err(e) => {
                log::warn!("planar fillet fast path failed ({e}); falling back to walking builder");
            }
        }
    }
    // Preserve the fork's constant-radius compatibility route as the next
    // choice. Run it transactionally as well so a failed legacy attempt can
    // never contaminate the repaired concave fallback below.
    let legacy_refusal = match transactional(topo, |t| {
        let mut builder = FilletBuilder::new(t, solid);
        builder.add_edges(edges, radius);
        let result = builder.build()?;
        validate_complete_blend(t, "fillet", solid, &result)?;
        validate_blend_volume(t, "fillet", solid, result.solid, edges, radius)?;
        Ok(result)
    }) {
        Ok(result) => return Ok(result),
        Err(error) => error,
    };

    // The radius-law builder contains the upstream concave-side and
    // orientation repairs, but selecting it for every constant-radius call
    // would lift the fork's compatibility pin. Retry only when every requested
    // edge is positively classified as concave and the legacy route above has
    // already failed closed.
    let all_concave = edges
        .iter()
        .all(|&edge| edge_is_convex(topo, solid, edge, radius * 0.25) == Some(false));
    if !all_concave {
        return Err(legacy_refusal);
    }

    transactional(topo, |t| {
        let mut builder = FilletBuilder::new(t, solid);
        builder.add_edges_with_law(
            edges,
            brepkit_blend::radius_law::RadiusLaw::Constant(radius),
        );
        let result = builder.build()?;
        validate_complete_blend(t, "fillet", solid, &result)?;
        validate_blend_volume(t, "fillet", solid, result.solid, edges, radius)?;
        Ok(result)
    })
}

/// Which of `edges` no longer name a manifold edge of `solid`.
fn stale_edges(
    topo: &Topology,
    solid: SolidId,
    edges: &[EdgeId],
) -> Result<Vec<EdgeId>, OperationsError> {
    let adjacency = topo.build_adjacency(solid)?;
    Ok(edges
        .iter()
        .copied()
        .filter(|&e| adjacency.faces_for_edge(e).len() != 2)
        .collect())
}

/// Fillet a selection that splits into features which cannot reach each other,
/// one feature at a time, on whichever engine each one needs.
///
/// This runs only after a single engine has refused the selection whole, so it
/// never displaces a working route; it turns a refusal into an answer or into a
/// better-aimed refusal.
///
/// The order is not arbitrary. The planar rebuild re-mints the loops of every
/// cap it rebuilds, so a bore rim that has not been blended yet loses its edge
/// identity when the cap above it is rebuilt; the rim assembler, by contrast,
/// carries the cap's other loops through verbatim, so straight edges named for
/// a later feature survive it. Features the planar path cannot take therefore
/// go first.
///
/// The identity assumption is checked rather than trusted: a feature whose
/// edges did not survive an earlier one is reported as
/// [`BlendError::EdgesNotBlended`] naming them, never dropped. Failure anywhere
/// aborts the whole call, and the caller's `transactional` wrapper puts the
/// input back exactly as it was.
fn fillet_by_feature(
    topo: &mut Topology,
    solid: SolidId,
    groups: &[Vec<EdgeId>],
    edges: &[EdgeId],
    radius: f64,
) -> Result<BlendResult, OperationsError> {
    let mut ordered: Vec<(bool, &[EdgeId])> = Vec::with_capacity(groups.len());
    for group in groups {
        ordered.push((is_planar_line_blend(topo, solid, group)?, group.as_slice()));
    }
    ordered.sort_by_key(|&(planar, _)| planar);

    let mut current = solid;
    for (_, group) in ordered {
        let stale = stale_edges(topo, current, group)?;
        if !stale.is_empty() {
            return Err(OperationsError::Blend(BlendError::EdgesNotBlended {
                edges: stale,
                reason: "an earlier feature in the same selection rebuilt the faces \
                         carrying these edges, so they no longer name anything to blend"
                    .into(),
            }));
        }
        current = fillet_group(topo, current, group, radius)?.solid;
    }

    let result = BlendResult {
        solid: current,
        succeeded: edges.to_vec(),
        failed: Vec::new(),
        is_partial: false,
        // Each feature was blended on whichever engine fitted it, and a face
        // created by one step can be trimmed by the next. Composing the steps'
        // records is not the same as concatenating them, so this path reports
        // none rather than a record that is right only when every step happened
        // to take the walking builder.
        face_origins: None,
    };
    // Per-feature validation compared each step with the step before it; this
    // compares the finished body with what the caller actually handed in, so
    // the volume budget covers every named edge at once.
    validate_complete_blend(topo, "fillet", solid, &result)?;
    validate_blend_volume(topo, "fillet", solid, current, edges, radius)?;
    Ok(result)
}

/// Fillet edges with constant radius (v2 walking-based engine).
///
/// # Errors
/// Returns `OperationsError` if radius is non-positive, edges are empty,
/// or the blend computation fails.
pub fn fillet_v2(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    radius: f64,
) -> Result<BlendResult, OperationsError> {
    if radius <= 0.0 {
        return Err(OperationsError::InvalidInput {
            reason: "radius must be positive".into(),
        });
    }
    if edges.is_empty() {
        return Err(OperationsError::InvalidInput {
            reason: "no edges specified".into(),
        });
    }
    let deduped = dedup_seed_edges(edges);
    let edges = deduped.as_slice();
    reject_seed_selection_larger_than_solid(topo, solid, "fillet", edges)?;
    reject_blend_into_hole(topo, solid, edges, radius)?;

    // The whole selection on one engine, first and unchanged: everything that
    // works today keeps working exactly as it does, and this is the only path a
    // selection that is one feature ever takes.
    let refusal = match transactional(topo, |t| fillet_group(t, solid, edges, radius)) {
        Ok(result) => return Ok(result),
        Err(e) => e,
    };

    // One engine could not take the selection whole. When the selection is
    // really several features that cannot reach each other, that is not a
    // verdict on any of them — it only says no single engine covers the mix —
    // so give each feature the engine that fits it.
    let groups = independent_blend_groups(topo, solid, edges, radius)?;
    if groups.len() < 2 {
        return Err(refusal);
    }
    transactional(topo, |t| {
        fillet_by_feature(t, solid, &groups, edges, radius)
    })
}

/// Stable machine-readable code for a blend failure.
///
/// Consumers on the far side of the WASM boundary (e.g. the OpenZCAD
/// adapter) receive errors as strings, so the bindings prefix messages with
/// this code to let callers branch on the cause without matching prose.
/// Codes are API: never rename one, only add.
#[must_use]
pub fn blend_failure_code(error: &OperationsError) -> &'static str {
    match error {
        OperationsError::Blend(BlendError::UnsupportedVertexBlend { .. }) => {
            "unsupported-vertex-blend"
        }
        OperationsError::Blend(BlendError::TrimmingFailure { .. }) => "trimming-failure",
        OperationsError::Blend(BlendError::RadiusTooLarge { .. }) => "radius-too-large",
        OperationsError::Blend(BlendError::CornerFailure { .. }) => "corner-failure",
        OperationsError::Blend(BlendError::EdgesNotBlended { .. }) => "edges-not-blended",
        OperationsError::Blend(BlendError::UnsupportedSurface { .. }) => "unsupported-surface",
        OperationsError::Blend(_) => "blend-failed",
        OperationsError::PartialResult { .. } => "partial-result",
        OperationsError::InvalidInput { .. } => "invalid-input",
        _ => "fillet-failed",
    }
}

/// Chamfer edges with two distances (v2 engine).
///
/// # Errors
/// Returns `OperationsError` if distances are non-positive, edges are empty,
/// or the blend computation fails.
pub fn chamfer_v2(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    d1: f64,
    d2: f64,
) -> Result<BlendResult, OperationsError> {
    if d1 <= 0.0 || d2 <= 0.0 {
        return Err(OperationsError::InvalidInput {
            reason: "distances must be positive".into(),
        });
    }
    if edges.is_empty() {
        return Err(OperationsError::InvalidInput {
            reason: "no edges specified".into(),
        });
    }
    // A repeated seed chamfers the same edge twice and lands a non-manifold
    // shell, so collapse repeats here for the same reason fillet does.
    let deduped = dedup_seed_edges(edges);
    let edges = deduped.as_slice();
    reject_seed_selection_larger_than_solid(topo, solid, "chamfer", edges)?;
    if is_planar_line_blend(topo, solid, edges)? {
        return transactional(topo, |t| planar_chamfer_result(t, solid, edges, d1, d2));
    }
    transactional(topo, |t| {
        let mut builder = ChamferBuilder::new(t, solid);
        builder.add_edges_asymmetric(edges, d1, d2);
        let result = builder.build()?;
        validate_complete_blend(t, "chamfer", solid, &result)?;
        validate_blend_volume(t, "chamfer", solid, result.solid, edges, d1.max(d2))?;
        Ok(result)
    })
}

/// Chamfer edges with distance and angle (v2 engine).
///
/// # Errors
/// Returns `OperationsError` if distance is non-positive, angle is out of
/// range (0, pi/2), edges are empty, or the blend computation fails.
pub fn chamfer_distance_angle(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    distance: f64,
    angle: f64,
) -> Result<BlendResult, OperationsError> {
    if distance <= 0.0 {
        return Err(OperationsError::InvalidInput {
            reason: "distance must be positive".into(),
        });
    }
    if angle <= 0.0 || angle >= std::f64::consts::FRAC_PI_2 {
        return Err(OperationsError::InvalidInput {
            reason: "angle must be between 0 and \u{03c0}/2".into(),
        });
    }
    if edges.is_empty() {
        return Err(OperationsError::InvalidInput {
            reason: "no edges specified".into(),
        });
    }
    // See `chamfer_v2`: repeated seeds chamfer the same edge twice.
    let deduped = dedup_seed_edges(edges);
    let edges = deduped.as_slice();
    reject_seed_selection_larger_than_solid(topo, solid, "chamfer", edges)?;
    let d2 = distance * angle.tan();
    if is_planar_line_blend(topo, solid, edges)? {
        return transactional(topo, |t| {
            planar_chamfer_result(t, solid, edges, distance, d2)
        });
    }
    transactional(topo, |t| {
        let mut builder = ChamferBuilder::new(t, solid);
        builder.add_edges_distance_angle(edges, distance, angle);
        let result = builder.build()?;
        validate_complete_blend(t, "chamfer", solid, &result)?;
        validate_blend_volume(t, "chamfer", solid, result.solid, edges, distance.max(d2))?;
        Ok(result)
    })
}

// ── Face provenance ────────────────────────────────────────────

/// [`evolution_from_blend_origins`] for a [`BlendResult`] this crate produced.
fn evolution_for_blend(
    topo: &Topology,
    result: &BlendResult,
    input_signatures: &[crate::evolution::FaceSignature],
) -> Result<EvolutionMap, OperationsError> {
    evolution_from_blend_origins(
        topo,
        result.solid,
        result.face_origins.as_ref(),
        input_signatures,
    )
}

/// Turn a blend engine's own construction record into an [`EvolutionMap`],
/// falling back to geometric matching when the engine that ran kept none.
///
/// `input_signatures` must have been snapshotted **before** the blend: a
/// successful blend trims the input solid's faces in place, so collecting them
/// afterwards would compare the result against itself.
///
/// The construction record is checked against the result shell rather than
/// trusted. A result face the record does not name is reported as unresolved,
/// so a record that has drifted from the assembler produces a refusal rather
/// than a confident half-answer.
///
/// Exposed for callers that drive the blend engines themselves — the WASM
/// bindings run their own engine cascade — and so still need to turn whatever
/// record came back into a map.
///
/// # Errors
///
/// Returns an error if the result solid's faces cannot be read.
pub fn evolution_from_blend_origins(
    topo: &Topology,
    result_solid: SolidId,
    origins: Option<&BlendFaceOrigins>,
    input_signatures: &[crate::evolution::FaceSignature],
) -> Result<EvolutionMap, OperationsError> {
    use brepkit_topology::explorer::solid_faces;

    let Some(origins) = origins else {
        let output_signatures = crate::boolean::collect_face_signatures(topo, result_solid)?;
        return Ok(crate::evolution::build_evolution_by_geometry(
            input_signatures,
            &output_signatures,
        ));
    };

    let result_faces: std::collections::HashSet<usize> = solid_faces(topo, result_solid)?
        .into_iter()
        .map(brepkit_topology::arena::Id::index)
        .collect();

    let mut evo = EvolutionMap::exact();
    let mut named: std::collections::HashSet<usize> = std::collections::HashSet::new();

    for &(src, dst) in &origins.survived {
        if result_faces.contains(&dst.index()) {
            evo.add_modified(src.index(), dst.index());
            named.insert(dst.index());
        } else {
            // The assembler dropped this face from the shell after recording
            // it. Saying "deleted" would be a claim; the record disagrees with
            // the result, so say nothing about the input and let the consumer
            // fail closed.
            log::debug!(
                "blend provenance: recorded survivor {} is not in the result shell",
                dst.index()
            );
        }
    }
    for &src in &origins.deleted {
        evo.add_deleted(src.index());
    }
    for (face, sources) in &origins.created {
        if !result_faces.contains(&face.index()) {
            continue;
        }
        named.insert(face.index());
        if sources.is_empty() {
            evo.add_unresolved(face.index(), Vec::new());
            continue;
        }
        for src in sources {
            evo.add_generated(src.index(), face.index());
        }
    }
    for face in &origins.created_unattributed {
        if result_faces.contains(&face.index()) {
            named.insert(face.index());
            evo.add_unresolved(face.index(), Vec::new());
        }
    }

    for &fid in &result_faces {
        if !named.contains(&fid) {
            evo.add_unresolved(fid, Vec::new());
        }
    }

    Ok(evo)
}

/// Fillet edges and report how each input face evolved.
///
/// The walking builder trims the faces it touches rather than re-minting them,
/// so it knows exactly which output face carries each input face and which two
/// base faces every blend band was built between. When that builder ran, the
/// returned map is [`EvolutionOrigin::Construction`] — fact, not inference.
///
/// The planar rolling-ball path records each face specification through
/// assembly and same-surface unification, so it is construction-derived too.
/// The per-feature fallback composes independently rebuilt operations and
/// cannot yet compose their histories; that route returns an
/// [`EvolutionOrigin::Geometry`] map in the Rust API, with anything the matcher
/// cannot separate left in
/// [`EvolutionMap::unresolved`]. Check
/// [`EvolutionMap::origin`](crate::evolution::EvolutionMap::origin) before
/// binding a persistent reference.
///
/// [`EvolutionOrigin::Construction`]: crate::evolution::EvolutionOrigin::Construction
/// [`EvolutionOrigin::Geometry`]: crate::evolution::EvolutionOrigin::Geometry
/// [`EvolutionMap::unresolved`]: crate::evolution::EvolutionMap::unresolved
///
/// # Errors
/// Returns whatever [`fillet_v2`] returns.
pub fn fillet_with_evolution(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    radius: f64,
) -> Result<(BlendResult, EvolutionMap), OperationsError> {
    let input_signatures = crate::boolean::collect_face_signatures(topo, solid)?;
    let result = fillet_v2(topo, solid, edges, radius)?;
    let evo = evolution_for_blend(topo, &result, &input_signatures)?;
    Ok((result, evo))
}

/// Chamfer edges with two distances and report how each input face evolved.
///
/// Same provenance contract as [`fillet_with_evolution`].
///
/// # Errors
/// Returns whatever [`chamfer_v2`] returns.
pub fn chamfer_with_evolution(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    d1: f64,
    d2: f64,
) -> Result<(BlendResult, EvolutionMap), OperationsError> {
    let input_signatures = crate::boolean::collect_face_signatures(topo, solid)?;
    let result = chamfer_v2(topo, solid, edges, d1, d2)?;
    let evo = evolution_for_blend(topo, &result, &input_signatures)?;
    Ok((result, evo))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use brepkit_math::vec::Point3;
    use brepkit_topology::edge::{Edge, EdgeCurve};
    use brepkit_topology::vertex::Vertex;

    use super::*;

    #[test]
    fn fillet_v2_rejects_all_failed_partial_result() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let v0 = topo.add_vertex(Vertex::new(Point3::new(10.0, 10.0, 10.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(11.0, 10.0, 10.0), 1e-7));
        let unrelated_edge = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));

        let result = fillet_v2(&mut topo, solid, &[unrelated_edge], 0.2);
        assert!(result.is_err());
        let Err(error) = result else { return };
        assert!(matches!(
            error,
            OperationsError::PartialResult {
                operation: "fillet",
                succeeded: 0,
                failed: 1,
            }
        ));
    }

    #[test]
    fn dedup_seed_edges_keeps_first_occurrence_order() {
        let mut topo = Topology::new();
        let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v2 = topo.add_vertex(Vertex::new(Point3::new(2.0, 0.0, 0.0), 1e-7));
        let a = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let b = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
        let c = topo.add_edge(Edge::new(v2, v0, EdgeCurve::Line));

        assert_eq!(dedup_seed_edges(&[c, a, b, a, c, b, c]), vec![c, a, b]);
    }

    #[test]
    fn fillet_v2_rejects_selection_larger_than_solid() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let mut edges = brepkit_topology::explorer::solid_edges(&topo, solid).unwrap();
        // One more distinct edge than the body owns: cannot name its geometry.
        let v0 = topo.add_vertex(Vertex::new(Point3::new(10.0, 10.0, 10.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(11.0, 10.0, 10.0), 1e-7));
        edges.push(topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line)));

        let result = fillet_v2(&mut topo, solid, &edges, 0.2);
        assert!(result.is_err_and(|error| error.to_string().contains("distinct edges")));
    }
}
