//! Shell sewing — close an open shell by sharing coincident free edges.
//!
//! A shell assembled patch-by-patch (a mesh import, a set of separately
//! built faces) is geometrically closed but topologically open: each face
//! carries its own copy of every boundary edge, so no edge is shared and
//! every one of them reads as free. Sewing repairs that by making the two
//! faces on either side of a seam reference **one** edge.
//!
//! Two coincident endpoints are not evidence that two edges carry the same
//! curve — a chord and the arc it subtends share both — so every candidate
//! pair is sampled along its interior before it is merged, and a pair that
//! disagrees is declined rather than reported as sewn.

use std::collections::{HashMap, HashSet};

use remus_math::det_hash::DetHashMap;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::pcurve::PCurve;
use remus_topology::shell::ShellId;
use remus_topology::vertex::VertexId;
use remus_topology::wire::{OrientedEdge, WireId};

use crate::HealError;

/// Interior samples used to decide whether two coincident free edges carry
/// the same 3D curve. The endpoints are already known to match, so only the
/// interior carries information.
const CURVE_SAMPLES: u32 = 7;

/// What a sewing pass did, and what it refused to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SewReport {
    /// Free-edge pairs merged into one shared edge. Each merge removes two
    /// free edges from the shell.
    pub sewn: usize,
    /// Coincident free-edge pairs that were **not** merged: the curves
    /// between the shared endpoints disagreed, the partner was ambiguous, or
    /// the merge would have collided in the pcurve registry. A non-zero
    /// count means the shell is still open on purpose.
    pub declined: usize,
}

/// Committed replacements within the sewn shell.
///
/// Entities may still be used outside that shell; these records do not assert
/// that their source handles were globally deleted.
#[derive(Debug, Clone, Default)]
pub struct SewHistory {
    /// Dropped boundary edge and the retained edge used in its place.
    pub edges: Vec<(EdgeId, EdgeId)>,
    /// Merged vertex and its final representative after all seam joins.
    pub vertices: Vec<(VertexId, VertexId)>,
}

/// Sew coincident free boundary edges in a shell.
///
/// Returns the number of edges sewn — that is, the number of free-edge pairs
/// collapsed onto a single shared edge. Zero means nothing was sewn, whether
/// because there was nothing to sew or because every candidate was declined;
/// [`sew_shell_report`] distinguishes the two.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups fail.
pub fn sew_shell(
    topo: &mut Topology,
    shell_id: ShellId,
    tolerance: f64,
) -> Result<usize, HealError> {
    Ok(sew_shell_report(topo, shell_id, tolerance)?.sewn)
}

/// Sew coincident free boundary edges in a shell, reporting declines.
///
/// Each pair of free edges whose endpoints coincide within `tolerance` is
/// sampled along its interior; only a pair that agrees everywhere is merged.
/// The dropped edge's wire uses are rewritten onto the retained edge (with
/// the traversal sense flipped when the retained edge runs the other way),
/// its pcurves are re-keyed onto the retained edge, and its vertices are
/// merged into the retained edge's, so the wires on both sides stay
/// connected chains.
///
/// A free edge with more than one geometrically valid partner is left alone
/// rather than paired arbitrarily.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups fail.
pub fn sew_shell_report(
    topo: &mut Topology,
    shell_id: ShellId,
    tolerance: f64,
) -> Result<SewReport, HealError> {
    sew_shell_impl(topo, shell_id, tolerance, None)
}

/// Sew a shell and retain the replacements committed by the sewing pass.
///
/// Uses the same candidate checks, tolerance, and rollback as [`sew_shell_report`].
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups or boundary updates fail.
pub fn sew_shell_with_history(
    topo: &mut Topology,
    shell_id: ShellId,
    tolerance: f64,
) -> Result<(SewReport, SewHistory), HealError> {
    let mut history = SewHistory::default();
    let report = sew_shell_impl(topo, shell_id, tolerance, Some(&mut history))?;
    Ok((report, history))
}

fn sew_shell_impl(
    topo: &mut Topology,
    shell_id: ShellId,
    tolerance: f64,
    history: Option<&mut SewHistory>,
) -> Result<SewReport, HealError> {
    let (wire_ids, usage) = survey_shell(topo, shell_id)?;

    let mut free_ids: Vec<EdgeId> = usage
        .iter()
        .filter(|&(_, &count)| count == 1)
        .map(|(&eid, _)| eid)
        .collect();
    free_ids.sort_unstable();

    if free_ids.len() < 2 {
        return Ok(SewReport::default());
    }

    let mut free = Vec::with_capacity(free_ids.len());
    for id in free_ids {
        let edge = topo.edge(id)?;
        let (start, end) = (edge.start(), edge.end());
        free.push(FreeEdge {
            id,
            start_pos: topo.vertex(start)?.point(),
            end_pos: topo.vertex(end)?.point(),
        });
    }

    let (plans, declined) = plan_merges(topo, &free, tolerance)?;
    if plans.is_empty() {
        return Ok(SewReport { sewn: 0, declined });
    }

    let snapshot = topo.clone();
    if let Err(error) = apply_merges(topo, shell_id, &wire_ids, &plans, history) {
        topo.restore_for_rollback(&snapshot);
        return Err(error);
    }

    Ok(SewReport {
        sewn: plans.len(),
        declined,
    })
}

/// A boundary edge used by exactly one face wire, with its endpoints
/// snapshotted before any merging moves them.
struct FreeEdge {
    id: EdgeId,
    start_pos: Point3,
    end_pos: Point3,
}

/// One planned merge: `drop` disappears from every wire in favour of `keep`.
struct Merge {
    keep: EdgeId,
    drop: EdgeId,
    /// `true` when `keep` traces `drop`'s curve backwards, so every use of
    /// `drop` flips its traversal sense when rewritten onto `keep`.
    reversed: bool,
}

/// Every distinct wire in the shell, and how many oriented-edge uses each
/// edge has across it.
///
/// Uses are counted per face traversal, matching
/// [`crate::analysis::free_bounds::find_free_bounds`], so the two agree on
/// which edges are free.
fn survey_shell(
    topo: &Topology,
    shell_id: ShellId,
) -> Result<(Vec<WireId>, HashMap<EdgeId, usize>), HealError> {
    let face_ids = topo.shell(shell_id)?.faces().to_vec();
    let mut wire_ids = Vec::new();
    let mut seen = HashSet::new();
    let mut usage: HashMap<EdgeId, usize> = HashMap::new();

    for face_id in face_ids {
        let face = topo.face(face_id)?;
        let face_wires: Vec<WireId> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();
        for wire_id in face_wires {
            for oe in topo.wire(wire_id)?.edges() {
                *usage.entry(oe.edge()).or_insert(0) += 1;
            }
            if seen.insert(wire_id) {
                wire_ids.push(wire_id);
            }
        }
    }

    Ok((wire_ids, usage))
}

/// Cell coordinate in the endpoint-neighborhood grid (PERF-H02).
type EndpointCell = (i64, i64, i64);

/// Largest exactly-representable grid coordinate (`2^53`), matching
/// `fix::vertex_merge` so both indexes fall back on the same inputs.
const MAX_EXACT_ENDPOINT_CELL: f64 = 9_007_199_254_740_992.0;

/// Outcome of [`plan_merges_with_stats`], including the exact-pair work the
/// index skipped. `plans` and `declined` are the only values that reach
/// sewing; the counters exist for equivalence tests and scaling reports.
#[allow(dead_code)]
struct SewPlanOutcome {
    plans: Vec<Merge>,
    declined: usize,
    /// Exact `endpoints_coincide` evaluations performed.
    endpoint_checks: u64,
    /// Exact `curves_agree` evaluations performed.
    curve_checks: u64,
    /// Exact `pcurve_keys_available` evaluations performed.
    pcurve_checks: u64,
    /// Midpoint-descriptor comparisons performed on endpoint-coincident
    /// certified pairs (bounded analytic domain only).
    descriptor_checks: u64,
    /// Endpoint-coincident certified pairs rejected by the midpoint
    /// descriptor without running `curves_agree`.
    pruned_by_descriptor: u64,
    /// Index-proposed `(i, j)` examinations, including merged-source skips
    /// that never reach the exact predicate.
    candidate_exams: u64,
    /// True when degenerate input forced the exact all-pairs fallback.
    fell_back_to_all_pairs: bool,
}

/// Pair up free edges that share both endpoints *and* the curve between
/// them. Returns the merges to perform and the number of coincident pairs
/// declined.
fn plan_merges(
    topo: &Topology,
    free: &[FreeEdge],
    tolerance: f64,
) -> Result<(Vec<Merge>, usize), HealError> {
    let outcome = plan_merges_with_stats(topo, free, tolerance)?;
    Ok((outcome.plans, outcome.declined))
}

/// Indexed planner entry point.
///
/// The endpoint-neighborhood grid only selects candidates; the existing
/// `endpoints_coincide` + `curves_agree` + `pcurve_keys_available` decision
/// process is the final authority, unchanged from the legacy scan. A
/// midpoint-descriptor prune (bounded analytic domain only) skips
/// `curves_agree` for endpoint-coincident certified pairs whose midpoints
/// already disagree; it only rejects, never accepts.
fn plan_merges_with_stats(
    topo: &Topology,
    free: &[FreeEdge],
    tolerance: f64,
) -> Result<SewPlanOutcome, HealError> {
    if free.len() < 2 {
        return Ok(SewPlanOutcome {
            plans: Vec::new(),
            declined: 0,
            endpoint_checks: 0,
            curve_checks: 0,
            pcurve_checks: 0,
            descriptor_checks: 0,
            pruned_by_descriptor: 0,
            candidate_exams: 0,
            fell_back_to_all_pairs: false,
        });
    }
    if !tolerance.is_finite() || tolerance <= 0.0 || !endpoint_grid_applicable(free, tolerance) {
        return plan_merges_all_pairs(topo, free, tolerance, true);
    }
    plan_merges_indexed(topo, free, tolerance)
}

/// Exact legacy all-pairs loop, preserved as the degenerate-input fallback
/// and as the equivalence oracle for tests.
///
/// Predicate (`dist² < tol²`, strict), tolerance policy, canonical survivor
/// (`free[i]` keeps, `free[j]` drops), deterministic `i`-then-`j` order,
/// consumed-edge skips, ambiguity refusals, and `declined` accounting are
/// bit-identical to the pre-PERF-H02 scan.
fn plan_merges_all_pairs(
    topo: &Topology,
    free: &[FreeEdge],
    tolerance: f64,
    fell_back: bool,
) -> Result<SewPlanOutcome, HealError> {
    let tol_sq = tolerance * tolerance;
    // An edge is consumed once it has been sewn, or once it has been ruled
    // out as an arbitrary choice among several valid partners.
    let mut consumed = vec![false; free.len()];
    let mut plans = Vec::new();
    let mut declined = 0;
    let mut endpoint_checks = 0u64;
    let mut curve_checks = 0u64;
    let mut pcurve_checks = 0u64;

    for i in 0..free.len() {
        if consumed[i] {
            continue;
        }

        let mut candidates: Vec<(usize, bool)> = Vec::new();
        let mut any_coincident = false;

        for j in (i + 1)..free.len() {
            if consumed[j] {
                continue;
            }
            endpoint_checks += 1;
            let (fwd_ok, rev_ok) = endpoints_coincide(&free[i], &free[j], tol_sq);
            if !fwd_ok && !rev_ok {
                continue;
            }
            any_coincident = true;
            // Shared endpoints prove nothing about the span between them.
            if fwd_ok {
                curve_checks += 1;
                if curves_agree(topo, &free[i], &free[j], false, tolerance)? {
                    candidates.push((j, false));
                }
            } else if rev_ok {
                curve_checks += 1;
                if curves_agree(topo, &free[i], &free[j], true, tolerance)? {
                    candidates.push((j, true));
                }
            }
        }

        declined += decide_merge(
            topo,
            free,
            &mut consumed,
            &mut plans,
            i,
            &candidates,
            any_coincident,
            &mut pcurve_checks,
        );
    }

    let candidate_exams = endpoint_checks;
    Ok(SewPlanOutcome {
        plans,
        declined,
        endpoint_checks,
        curve_checks,
        pcurve_checks,
        descriptor_checks: 0,
        pruned_by_descriptor: 0,
        candidate_exams,
        fell_back_to_all_pairs: fell_back,
    })
}

/// Indexed candidate discovery.
///
/// Endpoints are bucketed into a single uniform grid with cell size
/// `tolerance`: each free edge contributes its start cell and (when distinct)
/// its end cell. When `dist < tol`, per-axis `|d| < tol` forces the floored
/// cell coordinates to differ by at most one per axis, so every pair the
/// existing endpoint predicate would admit shares a 27-neighborhood around at
/// least one endpoint pair. The index is therefore *conservative*: it can
/// propose extra nearby pairs (edges sharing a single endpoint, e.g. corner
/// neighbors), but never misses an eligible one. Every candidate is
/// re-checked with the exact `endpoints_coincide` predicate before
/// `curves_agree` runs, so single-endpoint neighbors cost one cheap exact
/// rejection and no curve sampling.
///
/// The union is sorted ascending so the inner scan visits `j` in the same
/// order as the legacy loop restricted to coincident pairs; far pairs the
/// legacy loop would have skipped with `continue` cannot change the outcome.
///
/// A midpoint-descriptor prune (bounded analytic domain only) skips
/// `curves_agree` for endpoint-coincident certified pairs whose midpoints
/// already disagree; see [`certified_midpoint`] for the conservativeness
/// argument. The descriptor only rejects, never accepts: a passing pair
/// still runs the full exact check.
#[allow(clippy::too_many_lines)]
fn plan_merges_indexed(
    topo: &Topology,
    free: &[FreeEdge],
    tolerance: f64,
) -> Result<SewPlanOutcome, HealError> {
    let tol_sq = tolerance * tolerance;
    let n = free.len();

    let start_cells: Vec<EndpointCell> = free
        .iter()
        .map(|e| endpoint_cell(&e.start_pos, tolerance))
        .collect();
    let end_cells: Vec<EndpointCell> = free
        .iter()
        .map(|e| endpoint_cell(&e.end_pos, tolerance))
        .collect();
    // Single endpoint map: each edge appears under its start cell and (when
    // distinct) its end cell. Buckets grow in index order.
    let mut endpoint_map: DetHashMap<EndpointCell, Vec<usize>> = DetHashMap::default();
    for (idx, (s, e)) in start_cells.iter().zip(end_cells.iter()).enumerate() {
        endpoint_map.entry(*s).or_default().push(idx);
        if e != s {
            endpoint_map.entry(*e).or_default().push(idx);
        }
    }

    // Bounded-analytic certifiability, one cheap type lookup per edge.
    // Uncertified edges (NURBS, Parabola, Hyperbola, missing/invalid trim)
    // stay on the exact path; this precompute never changes decisions.
    let certifiable: Vec<bool> = free
        .iter()
        .map(|e| is_midpoint_certifiable(topo, e.id))
        .collect();
    // Lazily computed certified midpoints, one evaluation per edge at most.
    // `None` means not yet computed or uncertified/failed (exact path).
    let mut mid_cache: Vec<Option<Point3>> = vec![None; n];

    let mut consumed = vec![false; n];
    let mut plans = Vec::new();
    let mut declined = 0;
    let mut endpoint_checks = 0u64;
    let mut curve_checks = 0u64;
    let mut pcurve_checks = 0u64;
    let mut descriptor_checks = 0u64;
    let mut pruned_by_descriptor = 0u64;
    let mut candidate_exams = 0u64;
    // Scratch buffers reused across `i` to keep sparse memory linear.
    let mut union = Vec::new();

    for i in 0..n {
        if consumed[i] {
            continue;
        }

        collect_two_neighborhoods(&endpoint_map, &start_cells[i], &end_cells[i], &mut union);
        // Legacy order is ascending `j`; the index must not reorder it.
        union.retain(|&j| j > i);
        candidate_exams += union.len() as u64;

        let mut candidates: Vec<(usize, bool)> = Vec::new();
        let mut any_coincident = false;

        for &j in &union {
            if consumed[j] {
                continue;
            }
            endpoint_checks += 1;
            let (fwd_ok, rev_ok) = endpoints_coincide(&free[i], &free[j], tol_sq);
            if !fwd_ok && !rev_ok {
                continue;
            }
            any_coincident = true;
            // Midpoint prune before the expensive 7-sample check.
            // Orientation-invariant (`frac=0.5` maps to itself under
            // reversal), so one comparison covers both `fwd_ok` and `rev_ok`.
            // Forward precedence is preserved: when both orientations match,
            // the legacy scan checks forward only; pruning a forward-midpoint
            // mismatch therefore declines without trying reverse, identically.
            if certifiable[i] && certifiable[j] {
                descriptor_checks += 1;
                if midpoint_prunes(topo, &free[i], &free[j], &mut mid_cache, i, j, tolerance) {
                    pruned_by_descriptor += 1;
                    continue;
                }
            }
            // Shared endpoints prove nothing about the span between them.
            if fwd_ok {
                curve_checks += 1;
                if curves_agree(topo, &free[i], &free[j], false, tolerance)? {
                    candidates.push((j, false));
                }
            } else if rev_ok {
                curve_checks += 1;
                if curves_agree(topo, &free[i], &free[j], true, tolerance)? {
                    candidates.push((j, true));
                }
            }
        }

        declined += decide_merge(
            topo,
            free,
            &mut consumed,
            &mut plans,
            i,
            &candidates,
            any_coincident,
            &mut pcurve_checks,
        );
    }

    Ok(SewPlanOutcome {
        plans,
        declined,
        endpoint_checks,
        curve_checks,
        pcurve_checks,
        descriptor_checks,
        pruned_by_descriptor,
        candidate_exams,
        fell_back_to_all_pairs: false,
    })
}

/// Whether an edge qualifies for the midpoint descriptor.
///
/// Only the bounded analytic domain (`Line`, `Circle`, `Ellipse`) is
/// certified. `NurbsCurve`, `Hyperbola`, and `Parabola` are deliberately
/// uncertified and stay on the exact `curves_agree` path, as are edges whose
/// lookups fail (the exact path will surface the same error).
fn is_midpoint_certifiable(topo: &Topology, id: EdgeId) -> bool {
    match topo.edge(id) {
        Ok(edge) => matches!(
            edge.curve(),
            EdgeCurve::Line | EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_)
        ),
        Err(_) => false,
    }
}

/// Certified midpoint of a bounded-analytic edge, using the exact same
/// evaluation as [`curves_agree`]'s 4th interior sample (`k=4`,
/// `frac=4/8=0.5`).
///
/// Returns `None` for uncertified curve kinds, missing/invalid trims, or
/// non-finite results; the caller then stays on the exact path. A `None`
/// never prunes, so representation equivalence is preserved: different
/// variants or parameterizations describing compatible geometry share the
/// same midpoint (within tolerance) and are never rejected here.
///
/// Conservativeness: `frac=0.5` is one of the 7 samples `curves_agree`
/// checks with the same `tolerance` and the same `>` comparison. The
/// computation below is bit-identical (`a0 + (a1-a0)*0.5` via the stored
/// strict domain, same `evaluate_with_endpoints` call), and `0.5` maps to
/// itself under reversal (`1.0-0.5=0.5`), so one comparison covers both
/// orientations. If the midpoint distance exceeds `tolerance`, the exact
/// predicate would also reject at `k=4`; pruning therefore never discards a
/// pair the predicate accepts. Passing the descriptor establishes nothing:
/// the caller still runs the full 7-sample check.
fn certified_midpoint(topo: &Topology, edge: &FreeEdge) -> Option<Point3> {
    let stored = topo.edge(edge.id).ok()?;
    if !matches!(
        stored.curve(),
        EdgeCurve::Line | EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_)
    ) {
        return None;
    }
    let (a0, a1) = stored.strict_domain().ok()?;
    if !a0.is_finite() || !a1.is_finite() {
        return None;
    }
    // Bit-identical to `curves_agree` k=4: `frac=4/8`, `frac_b=1-frac=0.5`.
    let mid =
        stored
            .curve()
            .evaluate_with_endpoints(a0 + (a1 - a0) * 0.5, edge.start_pos, edge.end_pos);
    if !mid.x().is_finite() || !mid.y().is_finite() || !mid.z().is_finite() {
        return None;
    }
    Some(mid)
}

/// Whether the midpoint descriptor rejects an endpoint-coincident pair
/// without running `curves_agree`.
///
/// Both edges must be certifiable; otherwise returns `false` (proceed to
/// exact). Midpoints are lazily computed once per edge and cached; a pair
/// whose midpoint distance exceeds `tolerance` (same `>` comparison as the
/// exact predicate) is pruned. All other cases return `false`.
///
/// Declined descriptors (explicitly not implemented because conservativeness
/// could not be proven):
/// - Curve-type equality (`Line` vs `Circle` etc.): a near-straight circular
///   arc (sagitta `< tol`) is compatible with its chord, so type mismatch
///   cannot reject.
/// - Radius/center/plane equality for `Circle`/`Ellipse`: different
///   parameterizations (rotated `u_axis`, coterminal trims like `[0,π]` vs
///   `[2π,3π]`, circle-vs-circular-ellipse) describe identical geometry with
///   different stored parameters.
/// - Sampled or endpoint-only bounding boxes: sampled boxes do not enclose
///   the full qualified finite span for curved edges and are not
///   conservative certificates (roadmap audit constraint).
fn midpoint_prunes(
    topo: &Topology,
    a: &FreeEdge,
    b: &FreeEdge,
    cache: &mut [Option<Point3>],
    ai: usize,
    bi: usize,
    tolerance: f64,
) -> bool {
    let ma = if let Some(p) = cache[ai] {
        Some(p)
    } else {
        let computed = certified_midpoint(topo, a);
        // Cache only successful certifications; uncertified stays `None`
        // and retries cheaply (one type check) on later pairs.
        if let Some(p) = computed {
            cache[ai] = Some(p);
        }
        computed
    };
    let mb = if let Some(p) = cache[bi] {
        Some(p)
    } else {
        let computed = certified_midpoint(topo, b);
        if let Some(p) = computed {
            cache[bi] = Some(p);
        }
        computed
    };
    match (ma, mb) {
        (Some(pa), Some(pb)) => (pa - pb).length() > tolerance,
        // Uncertified or failed: stay on the exact path.
        _ => false,
    }
}

/// Shared merge decision for one `i`, identical for both planners.
///
/// Returns the `declined` increment (0 or 1). Consumes `i` (and its partners
/// on ambiguity) and pushes at most one plan, preserving canonical survivor
/// choice (`free[i]` keeps), per-use pcurve gating, and ambiguity refusals:
/// several eligible partners remain ambiguous rather than becoming an
/// arbitrary pair.
#[allow(clippy::too_many_arguments)]
fn decide_merge(
    topo: &Topology,
    free: &[FreeEdge],
    consumed: &mut [bool],
    plans: &mut Vec<Merge>,
    i: usize,
    candidates: &[(usize, bool)],
    any_coincident: bool,
    pcurve_checks: &mut u64,
) -> usize {
    if candidates.len() == 1 {
        let (j, reversed) = candidates[0];
        *pcurve_checks += 1;
        if pcurve_keys_available(topo, free[i].id, free[j].id, reversed) {
            consumed[i] = true;
            consumed[j] = true;
            plans.push(Merge {
                keep: free[i].id,
                drop: free[j].id,
                reversed,
            });
            0
        } else {
            // Merging would put two uses of one edge on one face in the
            // same direction, which is not a manifold boundary.
            log::debug!(
                "sew_shell: declining {:?}/{:?} — pcurve use key already occupied",
                free[i].id,
                free[j].id
            );
            1
        }
    } else if candidates.len() > 1 {
        // More than one valid partner is a non-manifold junction. Picking
        // one would be arbitrary, so pick none — and consume the whole
        // group, or the leftovers would pair off by iteration order.
        log::debug!(
            "sew_shell: declining {:?} — {} coincident partners",
            free[i].id,
            candidates.len()
        );
        consumed[i] = true;
        for &(j, _) in candidates {
            consumed[j] = true;
        }
        1
    } else if any_coincident {
        log::debug!(
            "sew_shell: declining {:?} — endpoints match a partner but the curves do not",
            free[i].id
        );
        1
    } else {
        0
    }
}

/// Whether the endpoint grid can represent every free edge conservatively.
fn endpoint_grid_applicable(free: &[FreeEdge], tolerance: f64) -> bool {
    free.iter().all(|e| {
        endpoint_finite_in_range(&e.start_pos, tolerance)
            && endpoint_finite_in_range(&e.end_pos, tolerance)
    })
}

fn endpoint_finite_in_range(p: &Point3, tolerance: f64) -> bool {
    p.x().is_finite()
        && p.y().is_finite()
        && p.z().is_finite()
        && (p.x() / tolerance).abs() <= MAX_EXACT_ENDPOINT_CELL
        && (p.y() / tolerance).abs() <= MAX_EXACT_ENDPOINT_CELL
        && (p.z() / tolerance).abs() <= MAX_EXACT_ENDPOINT_CELL
}

fn endpoint_cell(p: &Point3, tolerance: f64) -> EndpointCell {
    (
        (p.x() / tolerance).floor() as i64,
        (p.y() / tolerance).floor() as i64,
        (p.z() / tolerance).floor() as i64,
    )
}

/// Union of the 27 neighborhoods around `center`, sorted ascending.
///
/// Each index appears in exactly one bucket, so the union has no duplicates;
/// Union of the 27-neighborhoods around two endpoint cells, sorted ascending
/// and deduplicated.
///
/// Each edge appears under at most two cells, so the same index can surface
/// twice (once via `start`, once via `end`); sorting plus dedup restores one
/// ascending list for the deterministic scan.
fn collect_two_neighborhoods(
    map: &DetHashMap<EndpointCell, Vec<usize>>,
    start: &EndpointCell,
    end: &EndpointCell,
    out: &mut Vec<usize>,
) {
    out.clear();
    for center in [start, end] {
        for dx in -1..=1_i64 {
            for dy in -1..=1_i64 {
                for dz in -1..=1_i64 {
                    if let Some(bucket) = map.get(&(center.0 + dx, center.1 + dy, center.2 + dz)) {
                        out.extend_from_slice(bucket);
                    }
                }
            }
        }
    }
    out.sort_unstable();
    out.dedup();
}

/// Whether two free edges share both endpoints, forwards and/or reversed.
///
/// A closed edge (`start == end`) satisfies both; the interior sampling in
/// [`curves_agree`] is what settles the direction.
fn endpoints_coincide(a: &FreeEdge, b: &FreeEdge, tol_sq: f64) -> (bool, bool) {
    let fwd = (a.start_pos - b.start_pos).length_squared() < tol_sq
        && (a.end_pos - b.end_pos).length_squared() < tol_sq;
    let rev = (a.start_pos - b.end_pos).length_squared() < tol_sq
        && (a.end_pos - b.start_pos).length_squared() < tol_sq;
    (fwd, rev)
}

/// Whether two free edges describe the same 3D curve between their shared
/// endpoints, sampling `b` backwards when `reversed`.
fn curves_agree(
    topo: &Topology,
    a: &FreeEdge,
    b: &FreeEdge,
    reversed: bool,
    tolerance: f64,
) -> Result<bool, HealError> {
    let edge_a = topo.edge(a.id)?;
    let edge_b = topo.edge(b.id)?;
    let (a0, a1) = edge_a
        .strict_domain()
        .map_err(crate::error::upgrade_edge_domain)?;
    let (b0, b1) = edge_b
        .strict_domain()
        .map_err(crate::error::upgrade_edge_domain)?;

    for k in 1..=CURVE_SAMPLES {
        let frac = f64::from(k) / f64::from(CURVE_SAMPLES + 1);
        let frac_b = if reversed { 1.0 - frac } else { frac };
        let pa =
            edge_a
                .curve()
                .evaluate_with_endpoints(a0 + (a1 - a0) * frac, a.start_pos, a.end_pos);
        let pb =
            edge_b
                .curve()
                .evaluate_with_endpoints(b0 + (b1 - b0) * frac_b, b.start_pos, b.end_pos);
        if (pa - pb).length() > tolerance {
            return Ok(false);
        }
    }

    Ok(true)
}

/// Whether every pcurve use of `drop` can be re-keyed onto `keep` without
/// displacing a use already stored there.
fn pcurve_keys_available(topo: &Topology, keep: EdgeId, drop: EdgeId, reversed: bool) -> bool {
    topo.pcurves_for_edge(drop)
        .iter()
        .all(|(face, forward, _)| {
            topo.pcurve_oriented(keep, *face, *forward != reversed)
                .is_none()
        })
}

/// Carry out the planned merges: snapshot per-use authority, rewrite wire
/// uses, attach authority to the new coedges, then merge vertices.
fn apply_merges(
    topo: &mut Topology,
    shell_id: ShellId,
    wire_ids: &[WireId],
    plans: &[Merge],
    history: Option<&mut SewHistory>,
) -> Result<(), HealError> {
    // Snapshot authority before replacing wires retires the dropped coedges.
    // The retained edge is not yet a use of the dropped edge's face, so the
    // authority cannot truthfully be attached until after that replacement.
    let mut carried_authority = Vec::new();
    let merge_by_drop: HashMap<_, _> = plans
        .iter()
        .map(|merge| (merge.drop, (merge.keep, merge.reversed)))
        .collect();
    for &face in topo.shell(shell_id)?.faces() {
        for &loop_id in topo.face(face)?.boundary_loops() {
            for &coedge_id in topo.face_loop(loop_id)?.coedges() {
                let coedge = topo.coedge(coedge_id)?;
                let Some(&(keep, reversed)) = merge_by_drop.get(&coedge.edge()) else {
                    continue;
                };
                let (pcurve, forward) = if reversed {
                    (
                        coedge.pcurve().map(|pcurve| {
                            PCurve::new(pcurve.curve().clone(), pcurve.t_end(), pcurve.t_start())
                        }),
                        !coedge.is_forward(),
                    )
                } else {
                    (coedge.pcurve().cloned(), coedge.is_forward())
                };
                carried_authority.push((keep, face, forward, pcurve, coedge.periodic_winding()));
            }
        }
    }

    // 1. Rewrite every wire use of a dropped edge onto its retained twin.
    //    This — not rewriting the dropped edge's endpoints — is what makes
    //    two faces share one edge.
    let redirect = merge_by_drop;
    for &wire_id in wire_ids {
        let old: Vec<OrientedEdge> = topo.wire(wire_id)?.edges().to_vec();
        let mut changed = false;
        let new: Vec<OrientedEdge> = old
            .iter()
            .map(|oe| match redirect.get(&oe.edge()) {
                Some(&(keep, reversed)) => {
                    changed = true;
                    OrientedEdge::new(keep, oe.is_forward() != reversed)
                }
                None => *oe,
            })
            .collect();
        if changed {
            let closed = topo.wire(wire_id)?.is_closed();
            let replacement = remus_topology::wire::Wire::new(new, closed)?;
            topo.replace_boundary_wire(wire_id, replacement)?;
        }
    }

    // 2. The replacement above installed the retained edge's authoritative
    // coedge on every affected face. Attach each carried branch and periodic
    // lift to that exact use.
    for (edge, face, forward, pcurve, winding) in carried_authority {
        let mut matching = Vec::new();
        for &loop_id in topo.face(face)?.boundary_loops() {
            for &coedge_id in topo.face_loop(loop_id)?.coedges() {
                let coedge = topo.coedge(coedge_id)?;
                if coedge.edge() == edge && coedge.is_forward() == forward {
                    matching.push(coedge_id);
                }
            }
        }
        let [coedge_id] = matching.as_slice() else {
            return Err(HealError::UpgradeFailed(format!(
                "sewn edge {edge:?} does not have exactly one {} use on face {face:?}",
                if forward { "forward" } else { "reverse" }
            )));
        };
        topo.set_coedge_periodic_winding(*coedge_id, winding)?;
        if let Some(pcurve) = pcurve {
            topo.set_coedge_pcurve(*coedge_id, pcurve)?;
        }
    }

    // 3. Merge the dropped edges' vertices into the retained ones. Without
    //    this the neighbouring edges in each rewritten wire still terminate
    //    at their own copies and the chain is broken.
    let mut vertex_map: HashMap<VertexId, VertexId> = HashMap::new();
    for merge in plans {
        let keep = topo.edge(merge.keep)?;
        let (keep_start, keep_end) = if merge.reversed {
            (keep.end(), keep.start())
        } else {
            (keep.start(), keep.end())
        };
        let dropped = topo.edge(merge.drop)?;
        let (drop_start, drop_end) = (dropped.start(), dropped.end());
        union_vertices(&mut vertex_map, drop_start, keep_start);
        union_vertices(&mut vertex_map, drop_end, keep_end);
    }
    apply_vertex_map(topo, wire_ids, &vertex_map)?;

    // 4. Any face whose loops were derived now has a stale derivation
    //    (RFC 0002, Stage 1 keeps loops and wires in agreement).
    for face_id in topo.shell(shell_id)?.faces().to_vec() {
        if topo.loops_of_face(face_id).is_some() {
            topo.build_face_loops(face_id)?;
        }
    }

    if let Some(history) = history {
        history.edges = plans.iter().map(|merge| (merge.drop, merge.keep)).collect();
        history.vertices = vertex_map
            .keys()
            .map(|&source| (source, resolve_vertex(&vertex_map, source)))
            .collect();
        history.edges.sort_by_key(|(source, _)| source.index());
        history.vertices.sort_by_key(|(source, _)| source.index());
    }
    Ok(())
}

/// Repoint every edge still referenced by the shell at its merged vertices.
fn apply_vertex_map(
    topo: &mut Topology,
    wire_ids: &[WireId],
    vertex_map: &HashMap<VertexId, VertexId>,
) -> Result<(), HealError> {
    if vertex_map.is_empty() {
        return Ok(());
    }

    let mut seen = HashSet::new();
    let mut edge_ids = Vec::new();
    for &wire_id in wire_ids {
        for oe in topo.wire(wire_id)?.edges() {
            if seen.insert(oe.edge()) {
                edge_ids.push(oe.edge());
            }
        }
    }

    let mut updates = Vec::new();
    for edge_id in edge_ids {
        let edge = topo.edge(edge_id)?;
        let start = resolve_vertex(vertex_map, edge.start());
        let end = resolve_vertex(vertex_map, edge.end());
        if start != edge.start() || end != edge.end() {
            updates.push((edge_id, start, end));
        }
    }

    for (edge_id, start, end) in updates {
        // `set_start`/`set_end` rather than rebuilding through `Edge::new`:
        // an edge's explicit trim and edge-specific tolerance are not
        // recoverable from its endpoints.
        let edge = topo.edge_mut(edge_id)?;
        edge.set_start(start);
        edge.set_end(end);
    }

    Ok(())
}

/// Follow a vertex through the merge chain to its surviving representative.
///
/// Links are only ever made root-to-root, so the map is a forest and the walk
/// cannot cycle. It is bounded by the map's own size — an exact bound, unlike
/// a fixed cap, which would silently return a mid-chain vertex and split a
/// point that had been merged.
fn resolve_vertex(vertex_map: &HashMap<VertexId, VertexId>, mut vertex: VertexId) -> VertexId {
    for _ in 0..vertex_map.len() {
        match vertex_map.get(&vertex) {
            Some(&next) => vertex = next,
            None => break,
        }
    }
    vertex
}

/// Record that `from` and `to` are the same point, linking their roots.
fn union_vertices(vertex_map: &mut HashMap<VertexId, VertexId>, from: VertexId, to: VertexId) {
    let from = resolve_vertex(vertex_map, from);
    let to = resolve_vertex(vertex_map, to);
    if from != to {
        vertex_map.insert(from, to);
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    use remus_math::curves::{Circle3D, Ellipse3D, Parabola3D};
    use remus_math::curves2d::{Curve2D, Line2D};
    use remus_math::vec::{Point2, Point3, Vec2, Vec3};
    use remus_topology::coedge::PeriodicWinding;
    use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
    use remus_topology::face::{Face, FaceId, FaceSurface};
    use remus_topology::shell::Shell;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    const TOL: f64 = 1e-7;

    fn circle_edge(
        topo: &mut Topology,
        start: VertexId,
        end: VertexId,
        circle: Circle3D,
    ) -> EdgeId {
        let start_parameter = circle.project(topo.vertex(start).unwrap().point());
        let canonical_end = circle.project(topo.vertex(end).unwrap().point());
        let end_parameter = if start == end {
            start_parameter + std::f64::consts::TAU
        } else if canonical_end <= start_parameter {
            canonical_end + std::f64::consts::TAU
        } else {
            canonical_end
        };
        let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle));
        edge.set_trim(Some((start_parameter, end_parameter)));
        topo.add_edge(edge)
    }

    /// Build a planar quad face from four corner points, allocating fresh
    /// vertices and edges for every corner. Independent allocation is the
    /// point: it is what a mesh import or a patch-by-patch build produces,
    /// and it is the input sewing exists to repair.
    fn quad_face(topo: &mut Topology, pts: [Point3; 4], normal: Vec3, d: f64) -> FaceId {
        let vs: Vec<_> = pts
            .iter()
            .map(|p| topo.add_vertex(Vertex::new(*p, TOL)))
            .collect();
        let es: Vec<_> = (0..4)
            .map(|i| topo.add_edge(Edge::new(vs[i], vs[(i + 1) % 4], EdgeCurve::Line)))
            .collect();
        let wire = Wire::new(
            es.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
            true,
        )
        .unwrap();
        let wid = topo.add_wire(wire);
        topo.add_face(Face::new(wid, vec![], FaceSurface::Plane { normal, d }))
    }

    /// Six faces of a unit cube, each built from its own vertices and edges,
    /// assembled into one shell. Geometrically closed, topologically 24
    /// separate free edges — exactly the shape `sew_shell` is meant to close.
    fn disjoint_cube_shell(topo: &mut Topology) -> ShellId {
        let p = Point3::new;
        let faces = vec![
            // bottom, -Z
            quad_face(
                topo,
                [
                    p(0.0, 0.0, 0.0),
                    p(0.0, 1.0, 0.0),
                    p(1.0, 1.0, 0.0),
                    p(1.0, 0.0, 0.0),
                ],
                Vec3::new(0.0, 0.0, -1.0),
                0.0,
            ),
            // top, +Z
            quad_face(
                topo,
                [
                    p(0.0, 0.0, 1.0),
                    p(1.0, 0.0, 1.0),
                    p(1.0, 1.0, 1.0),
                    p(0.0, 1.0, 1.0),
                ],
                Vec3::new(0.0, 0.0, 1.0),
                1.0,
            ),
            // front, -Y
            quad_face(
                topo,
                [
                    p(0.0, 0.0, 0.0),
                    p(1.0, 0.0, 0.0),
                    p(1.0, 0.0, 1.0),
                    p(0.0, 0.0, 1.0),
                ],
                Vec3::new(0.0, -1.0, 0.0),
                0.0,
            ),
            // back, +Y
            quad_face(
                topo,
                [
                    p(0.0, 1.0, 0.0),
                    p(0.0, 1.0, 1.0),
                    p(1.0, 1.0, 1.0),
                    p(1.0, 1.0, 0.0),
                ],
                Vec3::new(0.0, 1.0, 0.0),
                1.0,
            ),
            // left, -X
            quad_face(
                topo,
                [
                    p(0.0, 0.0, 0.0),
                    p(0.0, 0.0, 1.0),
                    p(0.0, 1.0, 1.0),
                    p(0.0, 1.0, 0.0),
                ],
                Vec3::new(-1.0, 0.0, 0.0),
                0.0,
            ),
            // right, +X
            quad_face(
                topo,
                [
                    p(1.0, 0.0, 0.0),
                    p(1.0, 1.0, 0.0),
                    p(1.0, 1.0, 1.0),
                    p(1.0, 0.0, 1.0),
                ],
                Vec3::new(1.0, 0.0, 0.0),
                1.0,
            ),
        ];
        topo.add_shell(Shell::new(faces).unwrap())
    }

    fn free_edge_count(topo: &Topology, shell_id: ShellId) -> usize {
        crate::analysis::free_bounds::find_free_bounds(topo, shell_id)
            .unwrap()
            .iter()
            .map(Vec::len)
            .sum()
    }

    /// Every wire in the shell must be a connected closed chain: each edge's
    /// oriented end vertex is the next edge's oriented start vertex.
    fn assert_wires_chain(topo: &Topology, shell_id: ShellId) {
        for &fid in topo.shell(shell_id).unwrap().faces() {
            let face = topo.face(fid).unwrap();
            for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                let wire = topo.wire(wid).unwrap();
                remus_topology::validation::validate_wire_closed(wire, topo).unwrap_or_else(|e| {
                    panic!("wire {wid:?} on face {fid:?} is not a valid closed chain: {e}")
                });
                // Identity-level check: position-coincident but distinct
                // vertex IDs are accepted by `validate_wire_closed`, and
                // sewing is exactly the operation that must remove them.
                let oes = wire.edges();
                for k in 0..oes.len() {
                    let cur = oes[k];
                    let next = oes[(k + 1) % oes.len()];
                    let cur_e = topo.edge(cur.edge()).unwrap();
                    let next_e = topo.edge(next.edge()).unwrap();
                    assert_eq!(
                        cur.oriented_end(cur_e),
                        next.oriented_start(next_e),
                        "wire {wid:?} on face {fid:?}: edge {:?} ends at a different vertex than \
                         edge {:?} starts",
                        cur.edge(),
                        next.edge()
                    );
                }
            }
        }
    }

    #[test]
    fn sewing_history_maps_every_consumed_edge_and_vertex_to_its_live_representative() {
        let mut topo = Topology::new();
        let shell = disjoint_cube_shell(&mut topo);
        let (_, before) = survey_shell(&topo, shell).unwrap();
        let vertices: HashSet<_> = before
            .keys()
            .flat_map(|&id| {
                let edge = topo.edge(id).unwrap();
                [edge.start(), edge.end()]
            })
            .collect();
        let mut ordinary = topo.clone();
        let ordinary_report = sew_shell_report(&mut ordinary, shell, 1e-6).unwrap();
        let (report, history) = sew_shell_with_history(&mut topo, shell, 1e-6).unwrap();
        assert_eq!(report, ordinary_report);
        assert_eq!(report.sewn, 12);
        assert_eq!(history.edges.len(), 12);
        assert_eq!(history.vertices.len(), 16);
        let (_, live) = survey_shell(&topo, shell).unwrap();
        let live_vertices: HashSet<_> = live
            .keys()
            .flat_map(|&id| {
                let edge = topo.edge(id).unwrap();
                [edge.start(), edge.end()]
            })
            .collect();
        for source in before.keys() {
            let target = history
                .edges
                .iter()
                .find(|(from, _)| from == source)
                .map_or(*source, |(_, target)| *target);
            assert!(live.contains_key(&target));
            if target != *source {
                assert!(!live.contains_key(source));
            }
        }
        for source in vertices {
            let target = history
                .vertices
                .iter()
                .find(|(from, _)| *from == source)
                .map_or(source, |(_, target)| *target);
            assert!(live_vertices.contains(&target));
            if target != source {
                assert!(!live_vertices.contains(&source));
            }
        }
        assert_eq!(topo.allocated_slot_count(), ordinary.allocated_slot_count());
        let (wires, _) = survey_shell(&topo, shell).unwrap();
        for wire in wires {
            assert_eq!(
                topo.wire(wire)
                    .unwrap()
                    .edges()
                    .iter()
                    .map(|oe| (oe.edge(), oe.is_forward()))
                    .collect::<Vec<_>>(),
                ordinary
                    .wire(wire)
                    .unwrap()
                    .edges()
                    .iter()
                    .map(|oe| (oe.edge(), oe.is_forward()))
                    .collect::<Vec<_>>()
            );
        }
        assert_eq!(free_edge_count(&topo, shell), 0);
        let (repeat, repeated_history) = sew_shell_with_history(&mut topo, shell, 1e-6).unwrap();
        assert_eq!(repeat.sewn, 0);
        assert!(repeated_history.edges.is_empty());
        assert!(repeated_history.vertices.is_empty());
    }

    #[test]
    fn sewing_pipeline_keeps_sources_still_referenced_by_another_shell() {
        use remus_topology::journal::EntityKey;
        for retain_sources in [false, true] {
            let mut topo = Topology::new();
            let shell = disjoint_cube_shell(&mut topo);
            let mut inner = Vec::new();
            if retain_sources {
                // An open inner patch isolates shared-entity identity semantics;
                // this fixture is not a closed-solid qualification witness.
                let source = topo
                    .face(*topo.shell(shell).unwrap().faces().last().unwrap())
                    .unwrap()
                    .clone();
                let wire = topo.wire(source.outer_wire()).unwrap().clone();
                let wire = topo.add_wire(wire);
                let face = topo.add_face(Face::new(wire, Vec::new(), source.surface().clone()));
                inner.push(topo.add_shell(Shell::new(vec![face]).unwrap()));
            }
            let solid = topo.add_solid(remus_topology::solid::Solid::new(shell, inner));
            let mut direct = topo.clone();
            let (_, raw) = sew_shell_with_history(&mut direct, shell, 1e-7).unwrap();
            let mut process = crate::pipeline::process::HealProcess::new();
            process.add_step("sew_shells");
            let (_, reports, steps) = process.execute_with_history(&mut topo, solid).unwrap();
            assert_eq!(reports[0].actions_taken, 12);
            let claims = steps[0].replacements.entity_history().unwrap();
            let mut retained = 0;
            let pairs = raw
                .edges
                .iter()
                .map(|(from, to)| (EntityKey::edge(from.index()), EntityKey::edge(to.index())))
                .chain(raw.vertices.iter().map(|(from, to)| {
                    (
                        EntityKey::vertex(from.index()),
                        EntityKey::vertex(to.index()),
                    )
                }));
            for (source, target) in pairs {
                if steps[0].result.contains(&source) {
                    assert!(!claims.contains_key(&source));
                    retained += 1;
                } else {
                    assert_eq!(claims[&source], vec![target]);
                }
            }
            assert_eq!(retained > 0, retain_sources);
        }
    }

    #[test]
    fn wireframe_pipeline_repairs_outer_and_inner_shells_with_history() {
        let mut topo = Topology::new();
        let outer = disjoint_cube_shell(&mut topo);
        let inner = disjoint_cube_shell(&mut topo);
        // Separate shell identities isolate traversal and lineage from cavity containment.
        let solid = topo.add_solid(remus_topology::solid::Solid::new(outer, vec![inner]));
        let mut process = crate::pipeline::process::HealProcess::new();
        process.add_step("fix_wireframe");
        process.add_step("fix_wireframe");
        let (_, reports, history) = process.execute_with_history(&mut topo, solid).unwrap();
        assert_eq!(reports[0].actions_taken, 24);
        assert_eq!(reports[1].actions_taken, 0);
        for shell in [outer, inner] {
            assert_eq!(free_edge_count(&topo, shell), 0);
            assert_wires_chain(&topo, shell);
        }
        let claims = history[0].replacements.entity_history().unwrap();
        assert_eq!(claims.len(), 56);
        assert!(
            claims
                .values()
                .flatten()
                .all(|key| history[0].result.contains(key))
        );
        assert!(history[1].replacements.entity_history().unwrap().is_empty());
    }

    #[test]
    fn disabled_wireframe_repair_preserves_disjoint_boundaries() {
        let mut topo = Topology::new();
        let shell = disjoint_cube_shell(&mut topo);
        let mut ctx = crate::context::HealContext::new();
        let config = crate::fix::config::FixConfig {
            fix_wireframe: crate::fix::config::FixMode::Off,
            ..Default::default()
        };
        let (result, history) =
            crate::fix::wireframe::fix_wireframe_with_history(&mut topo, shell, &mut ctx, &config)
                .unwrap();
        assert_eq!(result.actions_taken, 0);
        assert_eq!(free_edge_count(&topo, shell), 24);
        assert!(history.edges.is_empty());
        assert!(history.vertices.is_empty());
    }

    #[test]
    fn wireframe_repair_closes_a_disjoint_cube_shell() {
        let mut topo = Topology::new();
        let shell = disjoint_cube_shell(&mut topo);
        assert_eq!(free_edge_count(&topo, shell), 24);
        let report = crate::fix::wireframe::fix_wireframe(
            &mut topo,
            shell,
            &mut crate::context::HealContext::new(),
            &crate::fix::config::FixConfig::default(),
        )
        .unwrap();
        let remaining = free_edge_count(&topo, shell);
        assert_eq!(
            remaining, 0,
            "reported {} repairs but retained {remaining} free edges",
            report.actions_taken
        );
        assert_eq!(report.actions_taken, 12);
        remus_topology::validation::validate_shell_closed(topo.shell(shell).unwrap(), &topo)
            .unwrap();
        assert_wires_chain(&topo, shell);
    }

    #[test]
    fn sew_shell_closes_a_disjoint_cube_shell() {
        let mut topo = Topology::new();
        let shell_id = disjoint_cube_shell(&mut topo);

        let before = free_edge_count(&topo, shell_id);
        assert_eq!(before, 24, "every edge of the disjoint cube starts free");

        let sewn = sew_shell(&mut topo, shell_id, 1e-6).unwrap();

        let after = free_edge_count(&topo, shell_id);

        assert_eq!(
            after, 0,
            "sew_shell reported {sewn} edges sewn but left {after} free edges"
        );
        assert_eq!(sewn, 12, "a cube has 12 edges to sew");

        let shell = topo.shell(shell_id).unwrap();
        remus_topology::validation::validate_shell_closed(shell, &topo)
            .expect("sewn cube shell must be a closed 2-manifold");
        assert_wires_chain(&topo, shell_id);
    }

    #[test]
    fn sew_shell_moves_pcurves_only_after_the_new_coedges_exist() {
        let mut topo = Topology::new();
        let shell_id = disjoint_cube_shell(&mut topo);
        let faces = topo.shell(shell_id).unwrap().faces().to_vec();
        for (face_position, &face) in faces.iter().enumerate() {
            let winding_u = i32::try_from(face_position).unwrap() + 1;
            let wire = topo.face(face).unwrap().outer_wire();
            for oriented in topo.wire(wire).unwrap().edges().to_vec() {
                let offset = face.index() as f64;
                topo.set_pcurve_oriented(
                    oriented.edge(),
                    face,
                    oriented.is_forward(),
                    PCurve::new(
                        Curve2D::Line(
                            Line2D::new(Point2::new(offset, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
                        ),
                        0.0,
                        1.0,
                    ),
                )
                .unwrap();
                let coedge = topo
                    .coedges_of_edge(oriented.edge())
                    .into_iter()
                    .find(|&coedge_id| {
                        let coedge = topo.coedge(coedge_id).unwrap();
                        coedge.is_forward() == oriented.is_forward()
                            && topo.face_loop(coedge.parent_loop()).unwrap().face() == face
                    })
                    .unwrap();
                topo.set_coedge_periodic_winding(coedge, PeriodicWinding::new(winding_u, -1))
                    .unwrap();
            }
        }
        assert_eq!(topo.num_pcurves(), 24);

        assert_eq!(sew_shell(&mut topo, shell_id, 1e-6).unwrap(), 12);

        assert_eq!(topo.num_pcurves(), 24);
        for (face_position, &face) in faces.iter().enumerate() {
            let winding_u = i32::try_from(face_position).unwrap() + 1;
            assert_eq!(topo.pcurves_for_face(face).len(), 4);
            for loop_id in topo.loops_of_face(face).unwrap() {
                for &coedge_id in topo.face_loop(*loop_id).unwrap().coedges() {
                    assert_eq!(
                        topo.coedge(coedge_id).unwrap().periodic_winding(),
                        PeriodicWinding::new(winding_u, -1)
                    );
                }
            }
            remus_topology::validation::validate_face_loops(&topo, face).unwrap();
        }
    }

    #[test]
    fn sew_shell_leaves_every_wire_a_connected_chain() {
        // Independent of the free-edge count: whatever `sew_shell` decides to
        // do, it must not leave a wire whose consecutive edges terminate at
        // different vertices. Rewriting one edge's endpoints in isolation
        // does exactly that.
        let mut topo = Topology::new();
        let shell_id = disjoint_cube_shell(&mut topo);
        assert_wires_chain(&topo, shell_id);

        sew_shell(&mut topo, shell_id, 1e-6).unwrap();

        assert_wires_chain(&topo, shell_id);
    }

    #[test]
    fn sew_shell_preserves_trim_and_tolerance_of_retained_edges() {
        // Sewing rewrites wire membership and merges coincident vertices. It
        // must not rebuild edges from scratch on the way: an edge's explicit
        // trim (RFC 0002) and edge-specific tolerance are not recoverable
        // from its endpoints.
        let mut topo = Topology::new();
        let shell_id = disjoint_cube_shell(&mut topo);

        // Stamp every edge in the shell with a distinguishable trim and
        // tolerance. A `Line` reads its domain as [0, 1]; the trim below is
        // deliberately different so a drop is visible.
        let mut edge_ids = Vec::new();
        for &fid in topo.shell(shell_id).unwrap().faces() {
            let wid = topo.face(fid).unwrap().outer_wire();
            for oe in topo.wire(wid).unwrap().edges() {
                edge_ids.push(oe.edge());
            }
        }
        for &eid in &edge_ids {
            let e = topo.edge_mut(eid).unwrap();
            e.set_trim(Some((0.0, 1.0)));
            e.set_tolerance(Some(3.5e-8)).unwrap();
        }

        sew_shell(&mut topo, shell_id, 1e-6).unwrap();

        // Check the edges that survived into the sewn wires.
        let mut checked = 0;
        for &fid in topo.shell(shell_id).unwrap().faces() {
            let wid = topo.face(fid).unwrap().outer_wire();
            for oe in topo.wire(wid).unwrap().edges() {
                let e = topo.edge(oe.edge()).unwrap();
                assert_eq!(
                    e.trim(),
                    Some((0.0, 1.0)),
                    "edge {:?} lost its explicit trim during sewing",
                    oe.edge()
                );
                assert_eq!(
                    e.tolerance(),
                    Some(3.5e-8),
                    "edge {:?} lost its edge-specific tolerance during sewing",
                    oe.edge()
                );
                checked += 1;
            }
        }
        assert_eq!(checked, 24, "all 24 edge uses inspected");
    }

    /// A quad face whose edges are supplied by the caller, so a test can
    /// hand it a specific curve or reuse a specific vertex.
    fn quad_face_from_edges(
        topo: &mut Topology,
        edges: [EdgeId; 4],
        normal: Vec3,
        d: f64,
    ) -> FaceId {
        let wire = Wire::new(
            edges.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
            true,
        )
        .unwrap();
        let wid = topo.add_wire(wire);
        topo.add_face(Face::new(wid, vec![], FaceSurface::Plane { normal, d }))
    }

    fn line_loop(
        topo: &mut Topology,
        pts: [Point3; 4],
    ) -> ([EdgeId; 4], Vec<remus_topology::vertex::VertexId>) {
        let vs: Vec<_> = pts
            .iter()
            .map(|p| topo.add_vertex(Vertex::new(*p, TOL)))
            .collect();
        let es: Vec<EdgeId> = (0..4)
            .map(|i| topo.add_edge(Edge::new(vs[i], vs[(i + 1) % 4], EdgeCurve::Line)))
            .collect();
        ([es[0], es[1], es[2], es[3]], vs)
    }

    #[test]
    fn sew_shell_flips_the_traversal_sense_on_a_reversed_match() {
        // Two coplanar squares meeting along x = 1, each wound CCW. The
        // shared segment is traversed (1,0,0)->(1,1,0) by the left face and
        // (1,1,0)->(1,0,0) by the right one — the ordinary manifold case,
        // and a reversed endpoint match.
        let p = Point3::new;
        let mut topo = Topology::new();
        let n = Vec3::new(0.0, 0.0, 1.0);

        let (left_edges, _) = line_loop(
            &mut topo,
            [
                p(0.0, 0.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, 1.0, 0.0),
                p(0.0, 1.0, 0.0),
            ],
        );
        let (right_edges, _) = line_loop(
            &mut topo,
            [
                p(1.0, 0.0, 0.0),
                p(2.0, 0.0, 0.0),
                p(2.0, 1.0, 0.0),
                p(1.0, 1.0, 0.0),
            ],
        );
        let seam_left = left_edges[1]; // (1,0,0) -> (1,1,0)
        let seam_right = right_edges[3]; // (1,1,0) -> (1,0,0)

        let fl = quad_face_from_edges(&mut topo, left_edges, n, 0.0);
        let fr = quad_face_from_edges(&mut topo, right_edges, n, 0.0);
        let shell_id = topo.add_shell(Shell::new(vec![fl, fr]).unwrap());

        assert_eq!(free_edge_count(&topo, shell_id), 8);
        let report = sew_shell_report(&mut topo, shell_id, 1e-6).unwrap();
        assert_eq!(
            report,
            SewReport {
                sewn: 1,
                declined: 0
            }
        );
        assert_eq!(free_edge_count(&topo, shell_id), 6, "one seam closed");

        // The retained edge appears in both wires, and the face that lost its
        // copy now traverses the retained edge backwards.
        let keep = seam_left.min(seam_right);
        let dropped = seam_left.max(seam_right);
        let mut senses = Vec::new();
        for fid in [fl, fr] {
            let wid = topo.face(fid).unwrap().outer_wire();
            for oe in topo.wire(wid).unwrap().edges() {
                assert_ne!(oe.edge(), dropped, "dropped edge still referenced");
                if oe.edge() == keep {
                    senses.push(oe.is_forward());
                }
            }
        }
        assert_eq!(senses.len(), 2, "the retained edge is used by both faces");
        assert_ne!(
            senses[0], senses[1],
            "a reversed match must flip the rewritten use's sense"
        );
        assert_wires_chain(&topo, shell_id);
    }

    #[test]
    fn sew_shell_declines_when_the_curves_between_shared_endpoints_disagree() {
        // Both edges run between (0,0,0) and (1,0,0); one is the chord, the
        // other the semicircular arc. Coincident endpoints, different curves.
        let p = Point3::new;
        let mut topo = Topology::new();

        let (top_edges, _) = line_loop(
            &mut topo,
            [
                p(0.0, 0.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, 1.0, 0.0),
                p(0.0, 1.0, 0.0),
            ],
        );
        let v_a = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
        let v_b = topo.add_vertex(Vertex::new(p(1.0, -1.0, 0.0), TOL));
        let v_c = topo.add_vertex(Vertex::new(p(0.0, -1.0, 0.0), TOL));
        let v_d = topo.add_vertex(Vertex::new(p(0.0, 0.0, 0.0), TOL));
        let arc = Circle3D::new(p(0.5, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.5).unwrap();
        let bottom_edges = [
            topo.add_edge(Edge::new(v_a, v_b, EdgeCurve::Line)),
            topo.add_edge(Edge::new(v_b, v_c, EdgeCurve::Line)),
            topo.add_edge(Edge::new(v_c, v_d, EdgeCurve::Line)),
            // (0,0,0) -> (1,0,0) along the arc, not the chord.
            circle_edge(&mut topo, v_d, v_a, arc),
        ];

        let n = Vec3::new(0.0, 0.0, 1.0);
        let ft = quad_face_from_edges(&mut topo, top_edges, n, 0.0);
        let fb = quad_face_from_edges(&mut topo, bottom_edges, n, 0.0);
        let shell_id = topo.add_shell(Shell::new(vec![ft, fb]).unwrap());

        let before = free_edge_count(&topo, shell_id);
        let wireframe = crate::fix::wireframe::fix_wireframe(
            &mut topo,
            shell_id,
            &mut crate::context::HealContext::new(),
            &crate::fix::config::FixConfig::default(),
        )
        .unwrap();
        assert_eq!(wireframe.actions_taken, 0);
        assert!(wireframe.status.contains(crate::status::Status::FAIL1));
        assert_eq!(free_edge_count(&topo, shell_id), before);
        let (report, history) = sew_shell_with_history(&mut topo, shell_id, 1e-6).unwrap();
        assert!(history.edges.is_empty());
        assert!(history.vertices.is_empty());

        assert_eq!(
            report,
            SewReport {
                sewn: 0,
                declined: 1
            },
            "the chord and the arc must not be reported as sewn"
        );
        assert_eq!(
            free_edge_count(&topo, shell_id),
            before,
            "declining must leave the shell exactly as it was"
        );
        assert_wires_chain(&topo, shell_id);
    }

    #[test]
    fn sew_shell_declines_an_ambiguous_partner() {
        // Three faces meet along one segment. Any two of them could be sewn;
        // choosing a pair would be an arbitrary answer to a non-manifold
        // junction, so none of the three is touched.
        let p = Point3::new;
        let mut topo = Topology::new();
        let fans = [
            [
                p(0.0, 0.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, 1.0, 0.0),
                p(0.0, 1.0, 0.0),
            ],
            [
                p(0.0, 0.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, -1.0, 0.0),
                p(0.0, -1.0, 0.0),
            ],
            [
                p(0.0, 0.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, 0.0, 1.0),
                p(0.0, 0.0, 1.0),
            ],
        ];
        let mut faces = Vec::new();
        for pts in fans {
            let (edges, _) = line_loop(&mut topo, pts);
            faces.push(quad_face_from_edges(
                &mut topo,
                edges,
                Vec3::new(0.0, 0.0, 1.0),
                0.0,
            ));
        }
        let shell_id = topo.add_shell(Shell::new(faces).unwrap());

        let before = free_edge_count(&topo, shell_id);
        let wireframe = crate::fix::wireframe::fix_wireframe(
            &mut topo,
            shell_id,
            &mut crate::context::HealContext::new(),
            &crate::fix::config::FixConfig::default(),
        )
        .unwrap();
        assert_eq!(wireframe.actions_taken, 0);
        assert!(wireframe.status.contains(crate::status::Status::FAIL1));
        assert_eq!(free_edge_count(&topo, shell_id), before);
        let (report, history) = sew_shell_with_history(&mut topo, shell_id, 1e-6).unwrap();
        assert!(history.edges.is_empty());
        assert!(history.vertices.is_empty());

        assert_eq!(report.sewn, 0, "an ambiguous junction must not be sewn");
        assert_eq!(report.declined, 1);
        assert_eq!(free_edge_count(&topo, shell_id), before);
        assert_wires_chain(&topo, shell_id);
    }

    #[test]
    fn sew_shells_operator_closes_the_shell_through_the_pipeline() {
        // The consumer-reachable path: `sew_shells` is registered in the
        // operator registry and named in the WASM heal bindings, so a JS
        // caller reaches this code by name.
        use crate::pipeline::process::HealProcess;

        let mut topo = Topology::new();
        let shell_id = disjoint_cube_shell(&mut topo);
        let solid_id = topo.add_solid(remus_topology::solid::Solid::new(shell_id, vec![]));

        assert_eq!(free_edge_count(&topo, shell_id), 24);

        let mut process = HealProcess::new();
        process.add_step("sew_shells");
        let (_, results) = process.execute(&mut topo, solid_id).unwrap();

        assert_eq!(results.len(), 1);
        assert_eq!(results[0].actions_taken, 12);
        assert!(
            results[0].status.contains(crate::status::Status::DONE1),
            "a successful sew must report DONE1, got {:?}",
            results[0].status
        );
        assert!(
            !results[0].status.is_fail(),
            "nothing was declined, so no FAIL flag belongs here"
        );
        assert_eq!(
            free_edge_count(&topo, shell_id),
            0,
            "the pipeline operator must actually close the shell"
        );
        assert_wires_chain(&topo, shell_id);
    }

    #[test]
    fn sew_shells_operator_reports_fail_when_it_declines() {
        // Reporting DONE on a shell it could not close is the defect this
        // whole change exists to remove.
        use crate::pipeline::process::HealProcess;

        let p = Point3::new;
        let mut topo = Topology::new();

        let (top_edges, _) = line_loop(
            &mut topo,
            [
                p(0.0, 0.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, 1.0, 0.0),
                p(0.0, 1.0, 0.0),
            ],
        );
        let v_a = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
        let v_b = topo.add_vertex(Vertex::new(p(1.0, -1.0, 0.0), TOL));
        let v_c = topo.add_vertex(Vertex::new(p(0.0, -1.0, 0.0), TOL));
        let v_d = topo.add_vertex(Vertex::new(p(0.0, 0.0, 0.0), TOL));
        let arc = Circle3D::new(p(0.5, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.5).unwrap();
        let bottom_edges = [
            topo.add_edge(Edge::new(v_a, v_b, EdgeCurve::Line)),
            topo.add_edge(Edge::new(v_b, v_c, EdgeCurve::Line)),
            topo.add_edge(Edge::new(v_c, v_d, EdgeCurve::Line)),
            circle_edge(&mut topo, v_d, v_a, arc),
        ];
        let n = Vec3::new(0.0, 0.0, 1.0);
        let ft = quad_face_from_edges(&mut topo, top_edges, n, 0.0);
        let fb = quad_face_from_edges(&mut topo, bottom_edges, n, 0.0);
        let shell_id = topo.add_shell(Shell::new(vec![ft, fb]).unwrap());
        let solid_id = topo.add_solid(remus_topology::solid::Solid::new(shell_id, vec![]));

        let mut process = HealProcess::new();
        process.add_step("sew_shells");
        let (_, results) = process.execute(&mut topo, solid_id).unwrap();

        assert_eq!(results[0].actions_taken, 0);
        assert!(
            results[0].status.is_fail(),
            "declining every candidate must not be reported as success, got {:?}",
            results[0].status
        );
        assert!(!results[0].status.is_done());
    }

    #[test]
    fn sew_shell_closes_a_seam_of_closed_circular_edges() {
        // Closed edges (`start == end`) are the periodic case: both endpoint
        // orientations match trivially, so only the interior sampling can
        // tell whether the two rims run the same way. Two discs sharing one
        // rim is the smallest shape that exercises it.
        let p = Point3::new;
        let mut topo = Topology::new();
        let circle = Circle3D::new(p(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();

        let mut disc = |normal: Vec3| {
            let seam = topo.add_vertex(Vertex::new(circle.evaluate(0.0), TOL));
            let rim = circle_edge(&mut topo, seam, seam, circle.clone());
            let wire = Wire::new(vec![OrientedEdge::new(rim, true)], true).unwrap();
            let wid = topo.add_wire(wire);
            topo.add_face(Face::new(
                wid,
                vec![],
                FaceSurface::Plane { normal, d: 0.0 },
            ))
        };
        let top = disc(Vec3::new(0.0, 0.0, 1.0));
        let bottom = disc(Vec3::new(0.0, 0.0, -1.0));
        let shell_id = topo.add_shell(Shell::new(vec![top, bottom]).unwrap());

        assert_eq!(free_edge_count(&topo, shell_id), 2, "two unshared rims");

        let report = sew_shell_report(&mut topo, shell_id, 1e-6).unwrap();
        assert_eq!(
            report,
            SewReport {
                sewn: 1,
                declined: 0
            }
        );
        assert_eq!(
            free_edge_count(&topo, shell_id),
            0,
            "the closed rim must end up shared by both discs"
        );
        remus_topology::validation::validate_shell_closed(topo.shell(shell_id).unwrap(), &topo)
            .expect("sewn rim leaves a closed shell");
        assert_wires_chain(&topo, shell_id);
    }

    // PERF-H02 equivalence: the indexed planner must return every pair the
    // legacy endpoint predicate would admit, with identical merge decisions.
    // Each fixture runs both planners on the same `free` snapshot and asserts
    // identical plans (canonical survivor, reversed sense), declined counts,
    // and that the index never evaluates more exact pairs than all-pairs.
    fn free_snapshot(topo: &Topology, ids: &[EdgeId]) -> Vec<FreeEdge> {
        let mut free: Vec<FreeEdge> = ids
            .iter()
            .map(|&id| {
                let edge = topo.edge(id).unwrap();
                let (start, end) = (edge.start(), edge.end());
                FreeEdge {
                    id,
                    start_pos: topo.vertex(start).unwrap().point(),
                    end_pos: topo.vertex(end).unwrap().point(),
                }
            })
            .collect();
        free.sort_by_key(|e| e.id.index());
        free
    }

    fn assert_planners_agree(topo: &Topology, free: &[FreeEdge], tolerance: f64) -> SewPlanOutcome {
        let reference = plan_merges_all_pairs(topo, free, tolerance, false).unwrap();
        let indexed = plan_merges_with_stats(topo, free, tolerance).unwrap();
        // Indexed must not fall back on well-formed inputs covered here;
        // fallback cases assert `fell_back_to_all_pairs` explicitly.
        assert!(
            !indexed.fell_back_to_all_pairs,
            "indexed planner fell back on a well-formed fixture"
        );
        assert_eq!(
            reference.declined, indexed.declined,
            "declined counts must match the reference"
        );
        assert_eq!(
            reference.plans.len(),
            indexed.plans.len(),
            "plan counts must match the reference"
        );
        for (a, b) in reference.plans.iter().zip(indexed.plans.iter()) {
            assert_eq!(a.keep, b.keep, "canonical survivor must match");
            assert_eq!(a.drop, b.drop, "dropped edge must match");
            assert_eq!(a.reversed, b.reversed, "reversed sense must match");
        }
        assert!(
            indexed.endpoint_checks <= reference.endpoint_checks.max(1),
            "index must not evaluate more endpoint pairs than all-pairs ({} vs {})",
            indexed.endpoint_checks,
            reference.endpoint_checks
        );
        // Curve pruning only rejects pairs the exact predicate would reject,
        // so expensive checks never increase and decisions stay identical.
        assert!(
            indexed.curve_checks <= reference.curve_checks,
            "pruned planner must not run more curve checks than all-pairs ({} vs {})",
            indexed.curve_checks,
            reference.curve_checks
        );
        assert_eq!(
            indexed.pcurve_checks, reference.pcurve_checks,
            "pcurve availability checks must match exactly"
        );
        // Every saved curve check must be accounted for by a descriptor prune:
        // the indexed coincident set equals the reference coincident set
        // (conservative endpoint index), partitioned into pruned vs exact.
        assert_eq!(
            indexed.curve_checks + indexed.pruned_by_descriptor,
            reference.curve_checks,
            "pruned + exact curve checks must account for every reference curve check"
        );
        assert!(
            indexed.pruned_by_descriptor <= indexed.descriptor_checks,
            "pruned pairs must be a subset of descriptor examinations"
        );
        indexed
    }

    fn line_edge_between(topo: &mut Topology, a: Point3, b: Point3) -> EdgeId {
        let va = topo.add_vertex(Vertex::new(a, TOL));
        let vb = topo.add_vertex(Vertex::new(b, TOL));
        topo.add_edge(Edge::new(va, vb, EdgeCurve::Line))
    }

    #[test]
    fn indexed_matches_reference_on_reversed_edges() {
        // Same segment opposite directions: forward match on one pair,
        // reversed match on the other. Both planners must pick the same
        // survivor and sense.
        let p = Point3::new;
        let mut topo = Topology::new();
        let e0 = line_edge_between(&mut topo, p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
        let e1 = line_edge_between(&mut topo, p(1.0, 0.0, 0.0), p(0.0, 0.0, 0.0));
        let e2 = line_edge_between(&mut topo, p(5.0, 5.0, 5.0), p(6.0, 5.0, 5.0));
        let free = free_snapshot(&topo, &[e0, e1, e2]);
        let outcome = assert_planners_agree(&topo, &free, 1e-6);
        assert_eq!(outcome.plans.len(), 1);
        assert!(outcome.plans[0].reversed);
        assert_eq!(outcome.declined, 0);
    }

    #[test]
    fn indexed_matches_reference_on_closed_circular_edges() {
        // Two closed rims sharing one seam point: both endpoint orientations
        // match trivially, interior sampling settles the direction.
        let p = Point3::new;
        let mut topo = Topology::new();
        let circle = Circle3D::new(p(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let seam = topo.add_vertex(Vertex::new(circle.evaluate(0.0), TOL));
        let rim0 = circle_edge(&mut topo, seam, seam, circle);
        let circle1 = Circle3D::new(p(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let seam1 = topo.add_vertex(Vertex::new(circle1.evaluate(0.0), TOL));
        let rim1 = circle_edge(&mut topo, seam1, seam1, circle1);
        // A far closed rim that must never become a candidate.
        let far_circle = Circle3D::new(p(50.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let far_seam = topo.add_vertex(Vertex::new(far_circle.evaluate(0.0), TOL));
        let far_rim = circle_edge(&mut topo, far_seam, far_seam, far_circle);
        let free = free_snapshot(&topo, &[rim0, rim1, far_rim]);
        let outcome = assert_planners_agree(&topo, &free, 1e-6);
        assert_eq!(outcome.plans.len(), 1);
        assert_eq!(outcome.declined, 0);
    }

    #[test]
    fn indexed_matches_reference_on_seam_uses_with_opposite_winding() {
        // Two closed rims on the same circle but opposite orientation (seam
        // uses): endpoints coincide trivially, interior sampling settles the
        // sense. Both planners must reach the same reversed decision.
        let p = Point3::new;
        let mut topo = Topology::new();
        let circle_fwd = Circle3D::new(p(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let circle_rev = Circle3D::new(p(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, -1.0), 1.0).unwrap();
        let seam0 = topo.add_vertex(Vertex::new(circle_fwd.evaluate(0.0), TOL));
        let rim0 = circle_edge(&mut topo, seam0, seam0, circle_fwd);
        let seam1 = topo.add_vertex(Vertex::new(circle_rev.evaluate(0.0), TOL));
        let rim1 = circle_edge(&mut topo, seam1, seam1, circle_rev);
        let free = free_snapshot(&topo, &[rim0, rim1]);
        let reference = plan_merges_all_pairs(&topo, &free, 1e-6, false).unwrap();
        let indexed = plan_merges_with_stats(&topo, &free, 1e-6).unwrap();
        assert_eq!(reference.declined, indexed.declined);
        assert_eq!(reference.plans.len(), indexed.plans.len());
        for (a, b) in reference.plans.iter().zip(indexed.plans.iter()) {
            assert_eq!(a.keep, b.keep);
            assert_eq!(a.drop, b.drop);
            assert_eq!(a.reversed, b.reversed);
        }
        // Whatever the winding decides (merge or decline), the index must not
        // invent a second opinion.
        assert!(indexed.endpoint_checks <= reference.endpoint_checks.max(1));
    }

    #[test]
    fn closed_reversed_curve_retains_legacy_forward_precedence() {
        let mut topo = Topology::new();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let reverse = circle.reversed();
        let seam = circle.evaluate(0.0);
        assert_eq!(seam, reverse.evaluate(0.0));
        let a = topo.add_vertex(Vertex::new(seam, TOL));
        let b = topo.add_vertex(Vertex::new(seam, TOL));
        let rim0 = circle_edge(&mut topo, a, a, circle);
        let rim1 = circle_edge(&mut topo, b, b, reverse);
        let free = free_snapshot(&topo, &[rim0, rim1]);
        let (forward, reversed) = endpoints_coincide(&free[0], &free[1], 1e-12);
        assert!(forward && reversed);
        assert!(!curves_agree(&topo, &free[0], &free[1], false, 1e-6).unwrap());
        assert!(curves_agree(&topo, &free[0], &free[1], true, 1e-6).unwrap());

        let outcome = assert_planners_agree(&topo, &free, 1e-6);
        assert!(outcome.plans.is_empty());
        assert_eq!(outcome.declined, 1);
    }

    #[test]
    fn indexed_matches_reference_on_pcurve_key_availability() {
        // `pcurve_keys_available` is the final gate after curve agreement.
        // Both planners call it identically; pin that a fresh pair is
        // available and stays available through the indexed path.
        let p = Point3::new;
        let mut topo = Topology::new();
        let e0 = line_edge_between(&mut topo, p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
        let e1 = line_edge_between(&mut topo, p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
        assert!(pcurve_keys_available(&topo, e0, e1, false));
        assert!(pcurve_keys_available(&topo, e0, e1, true));
        let free = free_snapshot(&topo, &[e0, e1]);
        let outcome = assert_planners_agree(&topo, &free, 1e-6);
        assert_eq!(outcome.plans.len(), 1);
    }

    #[test]
    fn indexed_matches_reference_on_chord_arc_pairs() {
        // Chord and arc share both endpoints but carry different curves:
        // coincident endpoints, declined curves.
        let p = Point3::new;
        let mut topo = Topology::new();
        let chord = line_edge_between(&mut topo, p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
        let va = topo.add_vertex(Vertex::new(p(0.0, 0.0, 0.0), TOL));
        let vb = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
        let arc_circle = Circle3D::new(p(0.5, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.5).unwrap();
        let arc = circle_edge(&mut topo, va, vb, arc_circle);
        let far = line_edge_between(&mut topo, p(20.0, 0.0, 0.0), p(21.0, 0.0, 0.0));
        let free = free_snapshot(&topo, &[chord, arc, far]);
        let outcome = assert_planners_agree(&topo, &free, 1e-6);
        assert_eq!(outcome.plans.len(), 0);
        assert_eq!(outcome.declined, 1);
    }

    #[test]
    fn indexed_matches_reference_on_unequal_trims_sharing_endpoints() {
        // Same circle, same endpoints, opposite bulges (upper vs lower
        // semicircle): unequal trims, disagreeing interiors.
        let p = Point3::new;
        let mut topo = Topology::new();
        let upper = Circle3D::new(p(0.5, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.5).unwrap();
        let lower = Circle3D::new(p(0.5, 0.0, 0.0), Vec3::new(0.0, 0.0, -1.0), 0.5).unwrap();
        let ua = topo.add_vertex(Vertex::new(p(0.0, 0.0, 0.0), TOL));
        let ub = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
        let la = topo.add_vertex(Vertex::new(p(0.0, 0.0, 0.0), TOL));
        let lb = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
        let e_upper = circle_edge(&mut topo, ua, ub, upper);
        let e_lower = circle_edge(&mut topo, la, lb, lower);
        let free = free_snapshot(&topo, &[e_upper, e_lower]);
        let outcome = assert_planners_agree(&topo, &free, 1e-6);
        assert_eq!(outcome.plans.len(), 0);
        assert_eq!(outcome.declined, 1);
    }

    #[test]
    fn indexed_matches_reference_across_cell_boundaries() {
        // Pair straddling the cell border at x = 10*tol, 4e-9 apart:
        // eligible and in neighboring cells.
        let tol = 1e-7;
        let mut topo = Topology::new();
        let a0 = Point3::new(10.0 * tol - 2e-9, 0.0, 0.0);
        let a1 = Point3::new(11.0 * tol - 2e-9, 0.0, 0.0);
        let b0 = Point3::new(10.0 * tol + 2e-9, 0.0, 0.0);
        let b1 = Point3::new(11.0 * tol + 2e-9, 0.0, 0.0);
        let e0 = line_edge_between(&mut topo, a0, a1);
        let e1 = line_edge_between(&mut topo, b0, b1);
        // Pair exactly tol apart: strict `<` keeps them distinct.
        let e2 = line_edge_between(
            &mut topo,
            Point3::new(0.0, 5.0, 0.0),
            Point3::new(1.0, 5.0, 0.0),
        );
        let e3 = line_edge_between(
            &mut topo,
            Point3::new(tol, 5.0, 0.0),
            Point3::new(1.0 + tol, 5.0, 0.0),
        );
        let free = free_snapshot(&topo, &[e0, e1, e2, e3]);
        let outcome = assert_planners_agree(&topo, &free, tol);
        assert_eq!(outcome.plans.len(), 1);
        assert_eq!(outcome.declined, 0);
    }

    #[test]
    fn indexed_matches_reference_on_negative_and_large_coordinates() {
        let tol = 1e-7;
        let mut topo = Topology::new();
        let e0 = line_edge_between(
            &mut topo,
            Point3::new(-100.0, -200.0, -300.0),
            Point3::new(-99.0, -200.0, -300.0),
        );
        let e1 = line_edge_between(
            &mut topo,
            Point3::new(-100.0 + 5e-8, -200.0, -300.0),
            Point3::new(-99.0 + 5e-8, -200.0, -300.0),
        );
        let e2 = line_edge_between(
            &mut topo,
            Point3::new(1e6, 2e6, -1e6),
            Point3::new(1e6 + 1.0, 2e6, -1e6),
        );
        let e3 = line_edge_between(
            &mut topo,
            Point3::new(1e6 + 5e-8, 2e6, -1e6),
            Point3::new(1e6 + 1.0 + 5e-8, 2e6, -1e6),
        );
        let e4 = line_edge_between(
            &mut topo,
            Point3::new(1e6 + 100.0, 2e6, -1e6),
            Point3::new(1e6 + 101.0, 2e6, -1e6),
        );
        let free = free_snapshot(&topo, &[e0, e1, e2, e3, e4]);
        let outcome = assert_planners_agree(&topo, &free, tol);
        assert_eq!(outcome.plans.len(), 2);
        assert_eq!(outcome.declined, 0);
    }

    #[test]
    fn indexed_matches_reference_on_tolerance_boundaries() {
        let tol = 1e-7;
        // Just below tol: eligible. Exactly tol: strict `<` declines.
        // Just above tol: declines.
        let mut topo = Topology::new();
        let base0 = Point3::new(0.0, 0.0, 0.0);
        let base1 = Point3::new(1.0, 0.0, 0.0);
        let e0 = line_edge_between(&mut topo, base0, base1);
        let e1 = line_edge_between(
            &mut topo,
            Point3::new(tol * (1.0 - 1e-9), 0.0, 0.0),
            Point3::new(1.0 + tol * (1.0 - 1e-9), 0.0, 0.0),
        );
        let free = free_snapshot(&topo, &[e0, e1]);
        let outcome = assert_planners_agree(&topo, &free, tol);
        assert_eq!(outcome.plans.len(), 1);

        let mut topo = Topology::new();
        let f0 = line_edge_between(&mut topo, base0, base1);
        let f1 = line_edge_between(
            &mut topo,
            Point3::new(tol, 0.0, 0.0),
            Point3::new(1.0 + tol, 0.0, 0.0),
        );
        let free = free_snapshot(&topo, &[f0, f1]);
        let outcome = assert_planners_agree(&topo, &free, tol);
        assert_eq!(outcome.plans.len(), 0);
        assert_eq!(outcome.declined, 0);
    }

    #[test]
    fn indexed_preserves_ambiguity_for_three_or_more_partners() {
        // Four coincident lines along one segment: every eligible partner
        // must remain ambiguous rather than becoming an arbitrary pair.
        let p = Point3::new;
        let mut topo = Topology::new();
        let ids: Vec<EdgeId> = (0..4)
            .map(|_| line_edge_between(&mut topo, p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0)))
            .collect();
        let far = line_edge_between(&mut topo, p(30.0, 0.0, 0.0), p(31.0, 0.0, 0.0));
        let mut all = ids;
        all.push(far);
        let free = free_snapshot(&topo, &all);
        let outcome = assert_planners_agree(&topo, &free, 1e-6);
        assert_eq!(outcome.plans.len(), 0);
        assert_eq!(outcome.declined, 1);
        // A second group of three elsewhere declines independently.
        let mut topo = Topology::new();
        let group_a: Vec<EdgeId> = (0..3)
            .map(|_| line_edge_between(&mut topo, p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0)))
            .collect();
        let group_b: Vec<EdgeId> = (0..3)
            .map(|_| line_edge_between(&mut topo, p(10.0, 0.0, 0.0), p(11.0, 0.0, 0.0)))
            .collect();
        let mut all = group_a;
        all.extend(group_b);
        let free = free_snapshot(&topo, &all);
        let outcome = assert_planners_agree(&topo, &free, 1e-6);
        assert_eq!(outcome.plans.len(), 0);
        assert_eq!(outcome.declined, 2);
    }

    #[test]
    fn degenerate_inputs_fall_back_without_changing_the_plan() {
        // Non-finite coordinates force the exact all-pairs fallback with an
        // identical plan.
        let mut topo = Topology::new();
        let e0 = line_edge_between(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        let e1 = line_edge_between(
            &mut topo,
            Point3::new(5e-8, 0.0, 0.0),
            Point3::new(1.0 + 5e-8, 0.0, 0.0),
        );
        let mut free = free_snapshot(&topo, &[e0, e1]);
        free[1].start_pos = Point3::new(f64::NAN, 0.0, 0.0);
        let reference = plan_merges_all_pairs(&topo, &free, 1e-6, false).unwrap();
        let indexed = plan_merges_with_stats(&topo, &free, 1e-6).unwrap();
        assert!(indexed.fell_back_to_all_pairs);
        assert_eq!(reference.declined, indexed.declined);
        assert_eq!(reference.plans.len(), indexed.plans.len());

        // Degenerate tolerance falls back too.
        let mut topo = Topology::new();
        let d0 = line_edge_between(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        let d1 = line_edge_between(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        let free = free_snapshot(&topo, &[d0, d1]);
        for tol in [0.0, -1e-7, f64::NAN, f64::INFINITY] {
            let indexed = plan_merges_with_stats(&topo, &free, tol).unwrap();
            assert!(indexed.fell_back_to_all_pairs, "tol {tol}");
            let reference = plan_merges_all_pairs(&topo, &free, tol, false).unwrap();
            assert_eq!(reference.declined, indexed.declined);
            assert_eq!(reference.plans.len(), indexed.plans.len());
        }

        // Coordinates beyond the exact-integer grid range fall back.
        let mut topo = Topology::new();
        let b0 = line_edge_between(
            &mut topo,
            Point3::new(1e16, 0.0, 0.0),
            Point3::new(1e16 + 1.0, 0.0, 0.0),
        );
        let b1 = line_edge_between(
            &mut topo,
            Point3::new(1e16 + 5e-8, 0.0, 0.0),
            Point3::new(1e16 + 1.0 + 5e-8, 0.0, 0.0),
        );
        let free = free_snapshot(&topo, &[b0, b1]);
        let indexed = plan_merges_with_stats(&topo, &free, 1e-7).unwrap();
        assert!(indexed.fell_back_to_all_pairs);
    }

    #[test]
    fn sparse_model_pays_no_endpoint_checks() {
        // 400 far-apart lines: nothing eligible, index proposes nothing.
        let mut topo = Topology::new();
        let ids: Vec<EdgeId> = (0..400)
            .map(|i| {
                line_edge_between(
                    &mut topo,
                    Point3::new(i as f64 * 10.0, 0.0, 0.0),
                    Point3::new(i as f64 * 10.0 + 1.0, 0.0, 0.0),
                )
            })
            .collect();
        let free = free_snapshot(&topo, &ids);
        let reference = plan_merges_all_pairs(&topo, &free, 1e-7, false).unwrap();
        let indexed = plan_merges_with_stats(&topo, &free, 1e-7).unwrap();
        assert_eq!(reference.plans.len(), 0);
        assert_eq!(indexed.plans.len(), 0);
        assert_eq!(indexed.endpoint_checks, 0);
        assert_eq!(indexed.candidate_exams, 0);
        assert_eq!(indexed.curve_checks, 0);
        assert_eq!(indexed.pcurve_checks, 0);
        assert_eq!(indexed.descriptor_checks, 0);
        assert_eq!(indexed.pruned_by_descriptor, 0);
        assert!(reference.endpoint_checks > 70_000);
    }

    #[test]
    fn randomized_fixtures_match_reference() {
        struct Lcg(u64);
        impl Lcg {
            fn next(&mut self) -> u64 {
                self.0 = self
                    .0
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                (self.0 >> 33) & 0x7fff_ffff
            }
            fn next_f64(&mut self) -> f64 {
                f64::from(self.next() as u32) / f64::from(u32::MAX)
            }
        }
        let tol = 1e-7;
        for (size, seed) in [(17, 0xabcd), (100, 0x55aa), (200, 0xbeef)] {
            // Sparse grid with a negative offset (cell-boundary coverage).
            let mut topo = Topology::new();
            let ids: Vec<EdgeId> = (0..size)
                .map(|i| {
                    line_edge_between(
                        &mut topo,
                        Point3::new(-50.0 + i as f64 * 1.7, 3.25, -1.5),
                        Point3::new(-50.0 + i as f64 * 1.7 + 1.0, 3.25, -1.5),
                    )
                })
                .collect();
            let free = free_snapshot(&topo, &ids);
            assert_planners_agree(&topo, &free, tol);

            // Clustered triples with deterministic jitter.
            let mut topo = Topology::new();
            let mut rng = Lcg(seed);
            let ids: Vec<EdgeId> = (0..size)
                .map(|i| {
                    let base = (i / 3) as f64 * 2.3 - 100.0;
                    let jx = (rng.next_f64() - 0.5) * 1.2e-7;
                    let jy = (rng.next_f64() - 0.5) * 1.2e-7;
                    line_edge_between(
                        &mut topo,
                        Point3::new(base + jx, jy, 0.0),
                        Point3::new(base + jx + 1.0, jy, 0.0),
                    )
                })
                .collect();
            let free = free_snapshot(&topo, &ids);
            assert_planners_agree(&topo, &free, tol);
        }
    }

    #[test]
    fn plan_is_deterministic_across_runs() {
        let mut topo = Topology::new();
        let ids: Vec<EdgeId> = (0..200)
            .map(|i| {
                line_edge_between(
                    &mut topo,
                    Point3::new(i as f64 * 1.1, 0.0, 0.0),
                    Point3::new(i as f64 * 1.1 + 1.0, 0.0, 0.0),
                )
            })
            .collect();
        let free = free_snapshot(&topo, &ids);
        let a = plan_merges_with_stats(&topo, &free, 1e-7).unwrap();
        let b = plan_merges_with_stats(&topo, &free, 1e-7).unwrap();
        assert_eq!(a.plans.len(), b.plans.len());
        for (x, y) in a.plans.iter().zip(b.plans.iter()) {
            assert_eq!(x.keep, y.keep);
            assert_eq!(x.drop, y.drop);
            assert_eq!(x.reversed, y.reversed);
        }
        assert_eq!(a.declined, b.declined);
    }

    // PERF-H02 curve-pruning slice: conservativeness + representation
    // equivalence + remaining fallbacks. Each fixture runs both planners on
    // the same `free` snapshot and asserts identical merge decisions
    // (canonical survivor, reversed sense, consumed-edge behavior, ambiguity
    // refusals, declined counts) with pruned expensive checks accounted for
    // (`curve + pruned == reference curve`).

    /// Circular arc through (0,0,0)-(1,0,0) with sagitta `s` (upper bulge
    /// for the reversed direction, major lower for the forward direction
    /// given `+Z` normal and CCW trim convention). Distinct `s` values give
    /// distinct midpoints, hence pairwise incompatible arcs.
    fn distinct_arc_edge(topo: &mut Topology, s: f64) -> EdgeId {
        let r = (0.25 + s * s) / (2.0 * s);
        let yc = s - r;
        let circle = Circle3D::new(Point3::new(0.5, yc, 0.0), Vec3::new(0.0, 0.0, 1.0), r).unwrap();
        let va = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), TOL));
        let vb = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), TOL));
        circle_edge(topo, va, vb, circle)
    }

    fn ellipse_edge(
        topo: &mut Topology,
        start: VertexId,
        end: VertexId,
        ellipse: Ellipse3D,
    ) -> EdgeId {
        let start_parameter = ellipse.project(topo.vertex(start).unwrap().point());
        let canonical_end = ellipse.project(topo.vertex(end).unwrap().point());
        let end_parameter = if start == end {
            start_parameter + std::f64::consts::TAU
        } else if canonical_end <= start_parameter {
            canonical_end + std::f64::consts::TAU
        } else {
            canonical_end
        };
        let mut edge = Edge::new(start, end, EdgeCurve::Ellipse(ellipse));
        edge.set_trim(Some((start_parameter, end_parameter)));
        topo.add_edge(edge)
    }

    fn parabola_edge(
        topo: &mut Topology,
        start: VertexId,
        end: VertexId,
        parabola: Parabola3D,
    ) -> EdgeId {
        let start_parameter = parabola.project(topo.vertex(start).unwrap().point());
        let canonical_end = parabola.project(topo.vertex(end).unwrap().point());
        let end_parameter = if canonical_end <= start_parameter {
            canonical_end + 2.0
        } else {
            canonical_end
        };
        // Ensure a forward span; degenerate projection still exercises the
        // uncertified path (exact check as final authority).
        let span = if end_parameter <= start_parameter {
            (start_parameter, start_parameter + 1.0)
        } else {
            (start_parameter, end_parameter)
        };
        let mut edge = Edge::new(start, end, EdgeCurve::Parabola(parabola));
        edge.set_trim(Some(span));
        topo.add_edge(edge)
    }

    #[test]
    fn pruning_preserves_reversed_arc_uses() {
        // Same upper arc opposite directions: reversed endpoint match,
        // compatible via reverse sampling. Midpoints coincide (orientation
        // invariant), so no pruning; full reverse check accepts.
        let mut topo = Topology::new();
        let s = 0.2;
        let r = (0.25 + s * s) / (2.0 * s);
        let yc = s - r;
        let circle = Circle3D::new(Point3::new(0.5, yc, 0.0), Vec3::new(0.0, 0.0, 1.0), r).unwrap();
        // Forward: (0,0)->(1,0) major lower; reversed twin uses swapped
        // vertices with the same circle to trace the minor upper back.
        let a0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), TOL));
        let a1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), TOL));
        let b0 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), TOL));
        let b1 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), TOL));
        let e0 = circle_edge(&mut topo, a0, a1, circle.clone());
        let e1 = circle_edge(&mut topo, b0, b1, circle);
        let free = free_snapshot(&topo, &[e0, e1]);
        let outcome = assert_planners_agree(&topo, &free, 1e-6);
        // Whether this particular circle/trim pair merges or declines, both
        // planners must agree; pruning must not invent a second opinion.
        // Midpoints for major-lower vs minor-upper differ, so pruning fires
        // when incompatible and stays silent when compatible.
        assert_eq!(outcome.plans.len() + outcome.declined, 1);
    }

    #[test]
    fn pruning_preserves_closed_rims_with_different_radii() {
        // Same seam point (1,0,0), different radii with valid on-curve seams:
        // opposite points differ by the diameter difference, pruned without
        // full sampling.
        let p = Point3::new;
        let mut topo = Topology::new();
        // Both circles pass through (1,0,0): c0 center (0,0,0) r=1.0,
        // c1 center (-0.5,0,0) r=1.5. Opposite points (-1,0,0) vs (-2,0,0).
        let c0 = Circle3D::new(p(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let c1 = Circle3D::new(p(-0.5, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.5).unwrap();
        let v0 = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
        let v1 = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
        let rim0 = circle_edge(&mut topo, v0, v0, c0);
        let rim1 = circle_edge(&mut topo, v1, v1, c1);
        let free = free_snapshot(&topo, &[rim0, rim1]);
        let outcome = assert_planners_agree(&topo, &free, 1e-6);
        assert_eq!(outcome.plans.len(), 0);
        assert_eq!(outcome.declined, 1);
        assert_eq!(outcome.pruned_by_descriptor, 1);
        assert_eq!(outcome.curve_checks, 0);
    }

    #[test]
    fn pruning_preserves_major_minor_arcs_sharing_endpoints() {
        // Same circle, same endpoints, minor upper (reversed) vs major lower
        // (forward): midpoints far apart, pruned.
        let mut topo = Topology::new();
        let s = 0.15;
        let r = (0.25 + s * s) / (2.0 * s);
        let yc = s - r;
        let circle = Circle3D::new(Point3::new(0.5, yc, 0.0), Vec3::new(0.0, 0.0, 1.0), r).unwrap();
        let a0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), TOL));
        let a1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), TOL));
        let b0 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), TOL));
        let b1 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), TOL));
        let major = circle_edge(&mut topo, a0, a1, circle.clone());
        let minor = circle_edge(&mut topo, b0, b1, circle);
        let free = free_snapshot(&topo, &[major, minor]);
        let outcome = assert_planners_agree(&topo, &free, 1e-6);
        assert_eq!(outcome.plans.len(), 0);
        assert_eq!(outcome.declined, 1);
        // Major vs minor mids differ by more than tolerance: pruned.
        assert_eq!(outcome.pruned_by_descriptor, 1);
        assert_eq!(outcome.curve_checks, 0);
    }

    #[test]
    fn pruning_accounts_for_chord_arc_decline() {
        // Chord vs semicircle: coincident endpoints, disagreeing interiors.
        // The existing chord/arc fixture must now prune instead of sampling.
        let p = Point3::new;
        let mut topo = Topology::new();
        let chord = line_edge_between(&mut topo, p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
        let va = topo.add_vertex(Vertex::new(p(0.0, 0.0, 0.0), TOL));
        let vb = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
        let arc_circle = Circle3D::new(p(0.5, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.5).unwrap();
        let arc = circle_edge(&mut topo, va, vb, arc_circle);
        let free = free_snapshot(&topo, &[chord, arc]);
        let outcome = assert_planners_agree(&topo, &free, 1e-6);
        assert_eq!(outcome.plans.len(), 0);
        assert_eq!(outcome.declined, 1);
        assert_eq!(outcome.pruned_by_descriptor, 1);
        assert_eq!(outcome.curve_checks, 0);
        assert_eq!(outcome.descriptor_checks, 1);
    }

    #[test]
    fn pruning_preserves_coterminal_trims_as_compatible() {
        // Same circle, same endpoints, coterminal trims `[0,π]` vs `[2π,3π]`:
        // identical point sets via periodicity. Midpoints coincide, so no
        // pruning; full check accepts (representation equivalence).
        let p = Point3::new;
        let mut topo = Topology::new();
        let circle = Circle3D::new(p(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let v0 = topo.add_vertex(Vertex::new(circle.evaluate(0.0), TOL));
        let v1 = topo.add_vertex(Vertex::new(circle.evaluate(std::f64::consts::PI), TOL));
        let w0 = topo.add_vertex(Vertex::new(circle.evaluate(0.0), TOL));
        let w1 = topo.add_vertex(Vertex::new(circle.evaluate(std::f64::consts::PI), TOL));
        let mut e0 = Edge::new(v0, v1, EdgeCurve::Circle(circle.clone()));
        e0.set_trim(Some((0.0, std::f64::consts::PI)));
        let e0 = topo.add_edge(e0);
        let mut e1 = Edge::new(w0, w1, EdgeCurve::Circle(circle));
        e1.set_trim(Some((
            2.0 * std::f64::consts::PI,
            3.0 * std::f64::consts::PI,
        )));
        let e1 = topo.add_edge(e1);
        let free = free_snapshot(&topo, &[e0, e1]);
        let outcome = assert_planners_agree(&topo, &free, 1e-6);
        assert_eq!(outcome.plans.len(), 1);
        assert_eq!(outcome.declined, 0);
        assert_eq!(outcome.pruned_by_descriptor, 0);
    }

    #[test]
    fn pruning_preserves_circular_ellipse_as_compatible() {
        // Circle radius 0.5 vs circular ellipse (a=b=0.5): identical point
        // sets, different variants. Must not be pruned; full check accepts.
        let p = Point3::new;
        let mut topo = Topology::new();
        let circle = Circle3D::new(p(0.5, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.5).unwrap();
        let ellipse = Ellipse3D::new(p(0.5, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.5, 0.5).unwrap();
        let a0 = topo.add_vertex(Vertex::new(p(0.0, 0.0, 0.0), TOL));
        let a1 = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
        let b0 = topo.add_vertex(Vertex::new(p(0.0, 0.0, 0.0), TOL));
        let b1 = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
        let e_circle = circle_edge(&mut topo, a0, a1, circle);
        let e_ellipse = ellipse_edge(&mut topo, b0, b1, ellipse);
        let free = free_snapshot(&topo, &[e_circle, e_ellipse]);
        let outcome = assert_planners_agree(&topo, &free, 1e-6);
        // Both trace the same lower semicircle (same center/radius/trim
        // convention): compatible, sewn.
        assert_eq!(outcome.plans.len(), 1);
        assert_eq!(outcome.declined, 0);
        assert_eq!(outcome.pruned_by_descriptor, 0);
    }

    #[test]
    fn pruning_rejects_distinct_ellipses_sharing_endpoints() {
        // Same endpoints, different ellipse bulges: midpoints differ, pruned.
        let p = Point3::new;
        let mut topo = Topology::new();
        let e_flat = Ellipse3D::new(p(0.5, -1.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.2, 1.0).unwrap();
        let e_tall = Ellipse3D::new(p(0.5, -2.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.2, 2.0).unwrap();
        let a0 = topo.add_vertex(Vertex::new(p(0.0, 0.0, 0.0), TOL));
        let a1 = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
        let b0 = topo.add_vertex(Vertex::new(p(0.0, 0.0, 0.0), TOL));
        let b1 = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
        let e0 = ellipse_edge(&mut topo, a0, a1, e_flat);
        let e1 = ellipse_edge(&mut topo, b0, b1, e_tall);
        let free = free_snapshot(&topo, &[e0, e1]);
        // Whatever the exact predicate decides for these two ellipses
        // (merge or decline), pruning must agree with it.
        let reference = plan_merges_all_pairs(&topo, &free, 1e-6, false).unwrap();
        let indexed = plan_merges_with_stats(&topo, &free, 1e-6).unwrap();
        assert_eq!(reference.declined, indexed.declined);
        assert_eq!(reference.plans.len(), indexed.plans.len());
        assert_eq!(
            indexed.curve_checks + indexed.pruned_by_descriptor,
            reference.curve_checks
        );
    }

    #[test]
    fn pruning_preserves_ambiguity_with_distractors() {
        // Three identical lines (ambiguous) plus one distinct arc sharing the
        // same endpoints. The arc pairs prune; the line trio still declines
        // as ambiguous with identical survivor behavior.
        let p = Point3::new;
        let mut topo = Topology::new();
        let l0 = line_edge_between(&mut topo, p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
        let l1 = line_edge_between(&mut topo, p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
        let l2 = line_edge_between(&mut topo, p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
        let arc = distinct_arc_edge(&mut topo, 0.3);
        let free = free_snapshot(&topo, &[l0, l1, l2, arc]);
        let outcome = assert_planners_agree(&topo, &free, 1e-6);
        assert_eq!(outcome.plans.len(), 0);
        assert_eq!(outcome.declined, 1);
        // `i=0` (first line) sees two compatible lines (full checks, pass)
        // plus one incompatible arc (pruned). Ambiguity consumes all three
        // lines, so the remaining arc pairs are never examined (consumed-edge
        // skips preserve order): 2 curve + 1 pruned = 3 reference checks.
        assert_eq!(outcome.pruned_by_descriptor, 1);
        assert_eq!(outcome.curve_checks, 2);
    }

    #[test]
    fn pruning_preserves_negative_and_large_arc_coordinates() {
        // Distinct arcs clustered at negative and large translations:
        // endpoint index finds them, midpoint pruning rejects incompatible
        // bulges without sampling.
        let tol = 1e-7;
        let mut topo = Topology::new();
        let mut ids = Vec::new();
        for (base, s) in [(-100.0, 0.08), (-100.0, 0.16), (1e6, 0.08), (1e6, 0.16)] {
            let r = (0.25 + s * s) / (2.0 * s);
            let yc = s - r;
            let circle = Circle3D::new(
                Point3::new(base + 0.5, yc, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                r,
            )
            .unwrap();
            let va = topo.add_vertex(Vertex::new(Point3::new(base, 0.0, 0.0), TOL));
            let vb = topo.add_vertex(Vertex::new(Point3::new(base + 1.0, 0.0, 0.0), TOL));
            ids.push(circle_edge(&mut topo, va, vb, circle));
        }
        let free = free_snapshot(&topo, &ids);
        let outcome = assert_planners_agree(&topo, &free, tol);
        // Two clusters (negative, large), each with two incompatible arcs:
        // each cluster declines once; far clusters never meet.
        assert_eq!(outcome.plans.len(), 0);
        assert_eq!(outcome.declined, 2);
        assert!(outcome.pruned_by_descriptor >= 2);
    }

    #[test]
    fn pruning_respects_tolerance_boundaries_for_arcs() {
        // Arcs whose midpoint separation sits just below vs just above `tol`.
        // Below: compatible enough to survive pruning (full check decides);
        // above: pruned. Both must match the reference exactly.
        let tol = 1e-6;
        // Base sagitta 0.2 -> mid -1.25; d(mid)/ds = 0.25/s^2 = 6.25.
        let s0 = 0.2;
        let delta_below = 0.5 * tol / 6.25;
        let delta_above = 2.0 * tol / 6.25;
        for (delta, expect_pruned) in [(delta_below, false), (delta_above, true)] {
            let mut topo = Topology::new();
            let a = distinct_arc_edge(&mut topo, s0);
            let b = distinct_arc_edge(&mut topo, s0 + delta);
            let free = free_snapshot(&topo, &[a, b]);
            let reference = plan_merges_all_pairs(&topo, &free, tol, false).unwrap();
            let indexed = plan_merges_with_stats(&topo, &free, tol).unwrap();
            assert_eq!(reference.declined, indexed.declined);
            assert_eq!(reference.plans.len(), indexed.plans.len());
            assert_eq!(
                indexed.curve_checks + indexed.pruned_by_descriptor,
                reference.curve_checks
            );
            assert_eq!(indexed.pruned_by_descriptor > 0, expect_pruned);
        }
    }

    #[test]
    fn pruning_preserves_nontransitive_midpoint_neighborhoods() {
        // Three arcs sharing endpoints with mids at 0, +0.6*tol, +1.2*tol:
        // A~B compatible, B~C compatible, A~C incompatible (midpoint prune).
        // Deterministic `i`-order sews A-B and leaves C; both planners agree.
        let tol = 1e-6;
        let s0 = 0.2;
        // d(mid)/ds = 6.25 at s0=0.2, so ds = d(mid)/6.25.
        let ds_ab = 0.6 * tol / 6.25;
        let ds_ac = 1.2 * tol / 6.25;
        let mut topo = Topology::new();
        let a = distinct_arc_edge(&mut topo, s0);
        let b = distinct_arc_edge(&mut topo, s0 + ds_ab);
        let c = distinct_arc_edge(&mut topo, s0 + ds_ac);
        let free = free_snapshot(&topo, &[a, b, c]);
        // Pairwise ground truth (nontransitive): A-B and B-C pass the exact
        // predicate only if their full 7-sample deviation stays within tol.
        // For these near-identical major arcs the midpoint dominates, so
        // A-B passes, A-C fails at the midpoint; B-C outcome is whatever the
        // predicate says — planners must still agree on the sequential
        // consume/decline outcome.
        let ab_fwd = endpoints_coincide(&free[0], &free[1], tol * tol);
        assert!(ab_fwd.0 || ab_fwd.1);
        let outcome = assert_planners_agree(&topo, &free, tol);
        // A and B are near-identical (0.6*tol mid separation): they sew,
        // consuming B before C is examined. C then stands alone.
        // If the full predicate instead declines A-B (shape differs beyond
        // the midpoint), both planners still decline identically.
        assert!(outcome.plans.len() <= 1);
        assert_eq!(outcome.plans.len() + outcome.declined, 1);
    }

    #[test]
    fn uncertified_parabolas_stay_on_the_exact_path() {
        // Parabola/Hyperbola are deliberately uncertified: no descriptor
        // pruning even when incompatible; exact predicate decides alone.
        let p = Point3::new;
        let mut topo = Topology::new();
        let par0 = Parabola3D::new(p(0.5, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 0.5).unwrap();
        let par1 = Parabola3D::new(p(0.5, -2.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 1.0).unwrap();
        let a0 = topo.add_vertex(Vertex::new(p(0.0, 0.0, 0.0), TOL));
        let a1 = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
        let b0 = topo.add_vertex(Vertex::new(p(0.0, 0.0, 0.0), TOL));
        let b1 = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
        let e0 = parabola_edge(&mut topo, a0, a1, par0);
        let e1 = parabola_edge(&mut topo, b0, b1, par1);
        let free = free_snapshot(&topo, &[e0, e1]);
        let outcome = assert_planners_agree(&topo, &free, 1e-6);
        assert_eq!(outcome.pruned_by_descriptor, 0);
        assert_eq!(outcome.descriptor_checks, 0);
    }

    #[test]
    fn dense_incompatible_arcs_prune_all_curve_checks() {
        // Targeted workload (Milestone 1/7): 100 distinct arcs sharing
        // (0,0,0)-(1,0,0), pairwise incompatible via distinct midpoints.
        // Endpoint index alone proposes every pair; midpoint pruning rejects
        // all without sampling.
        let tol = 1e-6;
        let n = 100;
        let mut topo = Topology::new();
        let ids: Vec<EdgeId> = (0..n)
            .map(|k| distinct_arc_edge(&mut topo, 0.05 + k as f64 * 0.005))
            .collect();
        let free = free_snapshot(&topo, &ids);
        let reference = plan_merges_all_pairs(&topo, &free, tol, false).unwrap();
        let indexed = plan_merges_with_stats(&topo, &free, tol).unwrap();
        assert_eq!(reference.plans.len(), 0);
        assert_eq!(indexed.plans.len(), 0);
        assert_eq!(reference.declined, indexed.declined);
        assert_eq!(reference.declined, n - 1);
        // All 4950 pairs endpoint-coincident and certified: pruned, no sampling.
        assert_eq!(reference.curve_checks, (n * (n - 1) / 2) as u64);
        assert_eq!(indexed.curve_checks, 0);
        assert_eq!(indexed.pruned_by_descriptor, reference.curve_checks);
        assert_eq!(indexed.descriptor_checks, reference.curve_checks);
        assert_eq!(indexed.pcurve_checks, 0);
        assert_eq!(reference.pcurve_checks, 0);
    }

    #[test]
    fn dense_compatible_pairs_keep_exact_checks_for_twins_only() {
        // 50 distinct arc geometries, each appearing twice (100 edges): each
        // twin pair is compatible, cross-geometry pairs are pruned. Expensive
        // checks drop from quadratic to linear in the twin count.
        let tol = 1e-6;
        let pairs = 50;
        let mut topo = Topology::new();
        let mut ids = Vec::new();
        for k in 0..pairs {
            let s = 0.05 + k as f64 * 0.008;
            ids.push(distinct_arc_edge(&mut topo, s));
            ids.push(distinct_arc_edge(&mut topo, s));
        }
        let free = free_snapshot(&topo, &ids);
        let reference = plan_merges_all_pairs(&topo, &free, tol, false).unwrap();
        let indexed = plan_merges_with_stats(&topo, &free, tol).unwrap();
        assert_eq!(reference.plans.len(), indexed.plans.len());
        assert_eq!(reference.plans.len(), pairs);
        assert_eq!(reference.declined, indexed.declined);
        assert_eq!(indexed.declined, 0);
        // Reference pays quadratic curve checks (each i scans all later
        // twins + distractors); indexed pays only twin checks + cheap
        // descriptor rejections. For 50 pairs: 99+97+...+1 = 2500.
        assert_eq!(reference.curve_checks, 2500);
        assert_eq!(indexed.curve_checks, pairs as u64);
        assert_eq!(
            indexed.curve_checks + indexed.pruned_by_descriptor,
            reference.curve_checks
        );
        assert_eq!(indexed.pcurve_checks, reference.pcurve_checks);
        assert_eq!(indexed.pcurve_checks, pairs as u64);
    }

    #[test]
    fn sewing_with_distractors_qualifies_output() {
        // End-to-end qualification with pruning active (Milestone 6): two
        // line quads sharing a seam (compatible, reversed) plus two arc quads
        // sharing the same seam endpoints with distinct bulges (incompatible,
        // pruned). Sewing must close the line seam with correct coedge
        // orientation, pcurve re-keying, attributes, lineage, and idempotence,
        // while leaving distractors untouched (rollback/no-partial-merge).
        let p = Point3::new;
        let n = Vec3::new(0.0, 0.0, 1.0);
        let mut topo = Topology::new();

        // Line quads: above [(0,0),(1,0),(1,1),(0,1)] seam (0,0)->(1,0) fwd;
        // below [(1,0),(0,0),(0,-1),(1,-1)] seam (1,0)->(0,0) rev (manifold).
        let (top_edges, _) = line_loop(
            &mut topo,
            [
                p(0.0, 0.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, 1.0, 0.0),
                p(0.0, 1.0, 0.0),
            ],
        );
        let (bottom_edges, _) = line_loop(
            &mut topo,
            [
                p(1.0, 0.0, 0.0),
                p(0.0, 0.0, 0.0),
                p(0.0, -1.0, 0.0),
                p(1.0, -1.0, 0.0),
            ],
        );
        let seam_top = top_edges[0];
        let seam_bottom = bottom_edges[0];
        let fl = quad_face_from_edges(&mut topo, top_edges, n, 0.0);
        let fb = quad_face_from_edges(&mut topo, bottom_edges, n, 0.0);

        // Distractor quads: same seam endpoints (0,0)-(1,0) fwd, distinct arc
        // bulges (major lower, far-down mids), plus three line edges closing
        // each quad below (y=-2) to avoid interfering with the line seam.
        let mut distractor_faces = Vec::new();
        for s in [0.08, 0.16] {
            let r = (0.25 + s * s) / (2.0 * s);
            let yc = s - r;
            let circle =
                Circle3D::new(Point3::new(0.5, yc, 0.0), Vec3::new(0.0, 0.0, 1.0), r).unwrap();
            let va = topo.add_vertex(Vertex::new(p(0.0, 0.0, 0.0), TOL));
            let vb = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
            let vc = topo.add_vertex(Vertex::new(p(1.0, -2.0, 0.0), TOL));
            let vd = topo.add_vertex(Vertex::new(p(0.0, -2.0, 0.0), TOL));
            let arc = circle_edge(&mut topo, va, vb, circle);
            let e1 = topo.add_edge(Edge::new(vb, vc, EdgeCurve::Line));
            let e2 = topo.add_edge(Edge::new(vc, vd, EdgeCurve::Line));
            let e3 = topo.add_edge(Edge::new(vd, va, EdgeCurve::Line));
            distractor_faces.push(quad_face_from_edges(&mut topo, [arc, e1, e2, e3], n, 0.0));
        }

        let mut all_faces = vec![fl, fb];
        all_faces.extend(distractor_faces);
        let shell_id = topo.add_shell(Shell::new(all_faces.clone()).unwrap());

        // Attributes: stamp faces (preserved through sewing, which only
        // rewrites wires, never faces).
        for &fid in &all_faces {
            topo.set_face_attributes(
                fid,
                remus_topology::attributes::EntityAttributes {
                    name: Some(format!("face-{}", fid.index())),
                    color: None,
                },
            )
            .unwrap();
        }
        // Pcurves + windings on the line faces (re-keying check, all edges).
        for (face_position, &face) in [fl, fb].iter().enumerate() {
            let wid = topo.face(face).unwrap().outer_wire();
            for oe in topo.wire(wid).unwrap().edges().to_vec() {
                let offset = face.index() as f64;
                topo.set_pcurve_oriented(
                    oe.edge(),
                    face,
                    oe.is_forward(),
                    PCurve::new(
                        Curve2D::Line(
                            Line2D::new(Point2::new(offset, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
                        ),
                        0.0,
                        1.0,
                    ),
                )
                .unwrap();
                let coedge = topo
                    .coedges_of_edge(oe.edge())
                    .into_iter()
                    .find(|&cid| {
                        let c = topo.coedge(cid).unwrap();
                        c.is_forward() == oe.is_forward()
                            && topo.face_loop(c.parent_loop()).unwrap().face() == face
                    })
                    .unwrap();
                topo.set_coedge_periodic_winding(
                    coedge,
                    PeriodicWinding::new(i32::try_from(face_position).unwrap() + 1, -1),
                )
                .unwrap();
            }
        }
        let _slots_before = topo.allocated_slot_count();
        let free_before = free_edge_count(&topo, shell_id);
        assert_eq!(free_before, 16);

        let (report, history) = sew_shell_with_history(&mut topo, shell_id, 1e-6).unwrap();
        // Four line-line pairs sew (one line seam reversed + three shared
        // distractor sides); two arc distractors share the same seam
        // endpoints but prune (midpoints far down), so one decline group.
        assert_eq!(report.sewn, 4);
        assert_eq!(report.declined, 1);

        // Coedge orientation: retained line seam used by both line faces in
        // opposite senses (reversed match flips).
        let keep = seam_top.min(seam_bottom);
        let dropped = seam_top.max(seam_bottom);
        assert!(history.edges.contains(&(dropped, keep)));
        assert_eq!(history.edges.len(), 4);
        let mut senses = Vec::new();
        for fid in [fl, fb] {
            let wid = topo.face(fid).unwrap().outer_wire();
            for oe in topo.wire(wid).unwrap().edges() {
                assert_ne!(oe.edge(), dropped);
                if oe.edge() == keep {
                    senses.push(oe.is_forward());
                }
            }
        }
        assert_eq!(senses.len(), 2);
        assert_ne!(senses[0], senses[1]);
        assert_wires_chain(&topo, shell_id);

        // Pcurve re-keying: both line faces retain 4 pcurves each, windings
        // preserved, face loops valid.
        assert_eq!(topo.pcurves_for_face(fl).len(), 4);
        assert_eq!(topo.pcurves_for_face(fb).len(), 4);
        for (face_position, &face) in [fl, fb].iter().enumerate() {
            let winding = PeriodicWinding::new(i32::try_from(face_position).unwrap() + 1, -1);
            for loop_id in topo.loops_of_face(face).unwrap() {
                for &coedge_id in topo.face_loop(*loop_id).unwrap().coedges() {
                    assert_eq!(topo.coedge(coedge_id).unwrap().periodic_winding(), winding);
                }
            }
            remus_topology::validation::validate_face_loops(&topo, face).unwrap();
        }

        // Attributes preserved (faces untouched by sewing).
        for &fid in &all_faces {
            let attrs = topo.attributes().face(fid).unwrap();
            assert_eq!(attrs.name, Some(format!("face-{}", fid.index())));
        }

        // Journal lineage: history maps dropped edges + merged vertices to live
        // representatives (one line seam + three shared sides: 4 edges, 6
        // vertices); pipeline converts the same mapping to entity_history.
        assert_eq!(history.edges.len(), 4);
        assert_eq!(history.vertices.len(), 6);
        let (_, live) = survey_shell(&topo, shell_id).unwrap();
        assert!(live.contains_key(&keep));
        assert!(!live.contains_key(&dropped));

        // Rollback/no-partial-merge: distractors untouched (still free, still
        // referenced). Slots grow via `build_face_loops` derivation rebuilds
        // (new loop/coedge entities), never shrink; free edges drop by two per
        // sewn pair (four pairs).
        assert_eq!(free_edge_count(&topo, shell_id), free_before - 8);

        // Persistent references: dropped edge resolves via history to kept.
        let target = history
            .edges
            .iter()
            .find(|(from, _)| *from == dropped)
            .map(|(_, to)| *to)
            .unwrap();
        assert_eq!(target, keep);
        assert!(live.contains_key(&target));

        // Idempotence: second sew finds lines already shared (no re-sew);
        // distractors still decline identically, topology unchanged.
        let slots_mid = topo.allocated_slot_count();
        let free_mid = free_edge_count(&topo, shell_id);
        let (repeat, repeat_history) = sew_shell_with_history(&mut topo, shell_id, 1e-6).unwrap();
        assert_eq!(repeat.sewn, 0);
        assert_eq!(repeat.declined, 1);
        assert!(repeat_history.edges.is_empty());
        assert!(repeat_history.vertices.is_empty());
        assert_eq!(topo.allocated_slot_count(), slots_mid);
        assert_eq!(free_edge_count(&topo, shell_id), free_mid);
        assert_wires_chain(&topo, shell_id);
    }
}
