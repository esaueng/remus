//! Duplicate-face repair — recognize and remove coincident duplicate face uses.
//!
//! Equivalence and removal contract: see
//! `docs/kernel-maturity/b17-h03-duplicate-face-contract.md`. In short: two
//! faces are duplicates only when their supporting carriers agree, their
//! oriented trimmed regions coincide (outer boundary plus one-to-one hole
//! correspondence), their effective orientations agree, and the pair is a
//! shell-local use of two distinct identities with compatible attributes.
//! Equal area, centroid, endpoint sets, or sampled distance never prove
//! duplication. The geometric predicate ([`faces_are_duplicates`]) is the
//! final authority — candidate buckets (PERF-H03) only select pairs to test.
//!
//! Currently supported domain: planar faces whose outer and hole boundaries
//! are built from `Line` segments and `Circle` arcs (open arcs and closed
//! rims). Supporting-plane agreement, oriented outer-region equality, and
//! one-to-one hole correspondence are all required; cyclic wire starts and
//! collinear/same-circle subdivision differences are normalized, while chords
//! are never equated with arcs. Curved carriers, ellipse/hyperbola/parabola/
//! NURBS boundaries, and anything unclassifiable are skipped (left in place,
//! fail-closed).
//!
//! ## Candidate discovery (PERF-H03)
//!
//! [`plan_duplicate_removals`] replaces only candidate *discovery*. The exact
//! predicate, the tolerance policy, the deterministic survivor (lowest
//! `FaceId` index in each duplicate group), and the nontransitive-nearness
//! semantics (a removed face never anchors another removal) are preserved:
//! on any input the plan equals the all-pairs reference over the same
//! index-ordered descriptors, including which faces are removed — only
//! provably non-matching pairs skip the exact predicate.
//!
//! Descriptors and their conservativeness proof (a true match can never be
//! split across buckets):
//!
//! - **Boundary signature** (exact key components): outer segment count plus
//!   the sorted hole segment counts. The predicate requires equal counts on
//!   both, so different signatures never match.
//! - **Effective-normal cell** (halo 1, cell `8e-3` per component): a true
//!   match satisfies `na·nb ≥ 1 − 1e-6`, i.e. an angle below ~`1.5e-3` rad,
//!   hence per-component `|Δ| < 8e-3`. Two values closer than one cell width
//!   differ by at most one in floored cell coordinates, so every true match
//!   shares the 27-neighborhood.
//! - **Boundary-centroid cell** (halo 1, cell = `tol` per axis): the anchor
//!   means of coincident outer boundaries differ per-axis by `< tol` —
//!   anchors are segment starts, except closed rims which anchor at their
//!   phase-invariant circle centers (seam vertices would make the bucket
//!   seam-dependent) — so true matches again share the 27-neighborhood.
//!
//! Faces whose descriptors cannot be established conservatively (non-finite
//! normal or centroid, centroid quotient beyond the exact-integer grid range,
//! non-finite or non-positive tolerance) fall back to the exact ordered
//! all-pairs loop with identical order and predicate, flagged by
//! [`DuplicatePlan::fell_back_to_all_pairs`]. The plane offset `d` is
//! deliberately never bucketed: near-parallel large faces can carry an
//! offset difference far above tolerance while still matching, so any offset
//! bucket would risk excluding a true match (contract §3.1).
//!
//! Processing streams faces in ascending `FaceId` order: for each unremoved
//! face `j`, candidates `i < j` are gathered from the bucket neighborhood in
//! ascending order and the first exact match marks `j` removed. This visits
//! the same decision dependencies as the legacy `i`-outer loop (a decision
//! for `j` only reads removal flags of `i < j`, final by the time `j` runs),
//! so the outcome equals the reference; only pruned pairs — which provably
//! fail the predicate — skip exact evaluation. Memory stays linear: buckets
//! hold one entry per face and each face materializes only its own candidate
//! prefix (dense coincident inputs still cost O(n²) time, reported honestly
//! via the plan stats).

use std::collections::HashMap;

use remus_math::det_hash::DetHashMap;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use super::FixResult;
use crate::HealError;
use crate::context::HealContext;
use crate::status::Status;

/// Cosine threshold for treating two face normals as parallel/anti-parallel.
/// Fixed (not derived from the model's linear tolerance) so a coarse linear
/// tolerance can't widen the angular test into matching clearly-different
/// orientations: `1 - cos θ ≈ θ²/2`, so 1e-6 ≈ 0.08°.
const NORMAL_PARALLEL_COS_TOL: f64 = 1e-6;

/// Normal-bucket cell width per unit-normal component.
///
/// A true match differs by less than ~`1.5e-3` rad (see
/// [`NORMAL_PARALLEL_COS_TOL`]), an order of magnitude below this cell, so a
/// halo of one cell provably covers every true match (module docs).
const NORMAL_CELL: f64 = 8e-3;

/// Largest exactly-representable grid coordinate (`2^53`).
const MAX_EXACT_CELL: f64 = 9_007_199_254_740_992.0;

/// Bucket coordinates for one candidate face: outer segment count, sorted
/// hole segment counts, normal cell, centroid cell.
type Signature = (usize, Vec<usize>);
/// Integer cell coordinates.
type Cell3 = (i64, i64, i64);
/// Bucket coordinates for one candidate face.
type Bucket = (Signature, (Cell3, Cell3));

/// One directed boundary segment of a supported planar face.
#[derive(Debug, Clone)]
enum BoundarySeg {
    /// Straight segment starting at `start` (end is the next segment's start;
    /// the loop closes onto the first start).
    Line {
        /// Segment start point.
        start: Point3,
    },
    /// Circular arc from `start` to `end` on one supporting circle.
    ///
    /// `sweep` is the canonical sweep as seen from the face-normal side, in
    /// `(0, TAU]`: the stored-circle `u_axis` choice is construction
    /// metadata, so spans are compared through frame-invariant start/end
    /// points plus this canonical sweep instead of raw parameters. Closed
    /// rims (`closed`, full circle) carry no seam phase at all: two rims on
    /// one circle match regardless of seam vertices, and an open arc whose
    /// gap is invisible at tolerance compares by circle and sweep alone.
    Arc {
        /// Arc start point.
        start: Point3,
        /// Arc end point.
        end: Point3,
        /// Circle center (lies in the face plane).
        center: Point3,
        /// Circle radius.
        radius: f64,
        /// Canonical sweep in `(0, TAU]`.
        sweep: f64,
        /// Full-circle rim (seam phase is meaningless).
        closed: bool,
    },
}

impl BoundarySeg {
    /// Segment start point (arcs and lines alike).
    fn start(&self) -> Point3 {
        match *self {
            Self::Line { start } => start,
            Self::Arc { start, .. } => start,
        }
    }
}

/// Spatial-bucket anchor for one segment: closed rims anchor at their
/// (phase-invariant) center, everything else at its start.
///
/// Conservativeness: matching loops have elementwise-compatible segments —
/// line/open-arc starts coincide and closed-rim centers coincide (compared
/// within tolerance) — so the anchors' means coincide within tolerance too.
/// Using the seam vertex for rims would make the bucket seam-dependent and
/// split true matches.
fn bucket_anchor(seg: &BoundarySeg) -> Point3 {
    match *seg {
        BoundarySeg::Line { start } => start,
        BoundarySeg::Arc {
            start,
            center,
            closed,
            ..
        } => {
            if closed {
                center
            } else {
                start
            }
        }
    }
}

/// Eligible-face descriptor for duplicate comparison, in ascending `FaceId`
/// order.
#[derive(Debug, Clone)]
struct FaceDescriptor {
    /// Face identity (ascending order ⇒ lowest index survives each group).
    face: FaceId,
    /// Effective plane normal (stored normal, negated when reversed).
    normal: Vec3,
    /// Canonicalized outer boundary loop.
    outer: Vec<BoundarySeg>,
    /// Canonicalized hole loops (stored wire order; matched bijectively).
    holes: Vec<Vec<BoundarySeg>>,
    /// Mean of the outer-loop segment starts (spatial bucket center).
    centroid: Point3,
}

/// Result of [`plan_duplicate_removals`].
#[derive(Debug, Clone)]
struct DuplicatePlan {
    /// `(survivor, removed)` pairs in ascending removed-face order.
    pairs: Vec<(FaceId, FaceId)>,
    /// Candidate examinations, including removed-anchor skips that never
    /// reach the exact predicate.
    candidate_exams: u64,
    /// Exact predicate evaluations performed.
    exact_comparisons: u64,
    /// True when degenerate input forced the exact all-pairs fallback.
    fell_back_to_all_pairs: bool,
}

/// Detect and remove geometrically duplicate faces within each shell of a solid.
///
/// Compares planar faces whose boundaries are built from `Line` segments and
/// `Circle` arcs, with and without holes. Ordered outer boundaries must
/// coincide with the same winding (allowing cyclic shifts and normalized
/// subdivision differences), holes must correspond one-to-one, and effective
/// normals must agree. Oppositely oriented coincident faces are preserved:
/// removing one would silently choose a side of a zero-thickness or
/// otherwise malformed region. Other carriers and boundary curves are
/// skipped because proving their trimmed regions equal is outside the
/// supported contract.
///
/// Comparisons stay within each shell: coincident boundaries in different
/// shells do not establish that either face use can be removed.
pub(super) fn fix_duplicate_faces(
    topo: &Topology,
    solid_id: SolidId,
    ctx: &mut HealContext,
) -> Result<FixResult, HealError> {
    let solid = topo.solid(solid_id)?;
    let shells: Vec<_> = std::iter::once(solid.outer_shell())
        .chain(solid.inner_shells().iter().copied())
        .collect();
    let mut face_shell_uses = HashMap::new();
    for &shell in &shells {
        for &face in topo.shell(shell)?.faces() {
            *face_shell_uses.entry(face).or_insert(0usize) += 1;
        }
    }
    let mut result = FixResult::ok();
    for shell in shells {
        result.merge(&fix_shell_duplicate_faces(
            topo,
            shell,
            &face_shell_uses,
            ctx,
        )?);
    }
    Ok(result)
}

fn fix_shell_duplicate_faces(
    topo: &Topology,
    shell_id: remus_topology::shell::ShellId,
    face_shell_uses: &HashMap<FaceId, usize>,
    ctx: &mut HealContext,
) -> Result<FixResult, HealError> {
    let tol = ctx.tolerance.linear;
    let shell = topo.shell(shell_id)?;
    let face_ids: Vec<_> = shell.faces().to_vec();

    // Eligible-face descriptors in ascending FaceId order, so the survivor of
    // each duplicate group is deterministic (lowest index) regardless of
    // shell face order or bucket layout.
    let mut descriptors: Vec<FaceDescriptor> = Vec::new();
    for &fid in &face_ids {
        if let Some(descriptor) = describe_face(topo, fid, tol)? {
            descriptors.push(descriptor);
        }
    }
    descriptors.sort_by_key(|d| d.face.index());

    let plan = plan_duplicate_removals(&descriptors, tol);

    // Attribute compatibility (contract §8): face attributes are application
    // vocabulary the kernel never synthesizes or merges, so a removed face
    // carrying attributes the survivor lacks — or conflicting values —
    // vetoes that pair and both faces stay. Dropping vetoed pairs is always
    // safe: a removed face never anchors another pair, so keeping more can
    // never strand a dangling reference.
    let mut pairs: Vec<(FaceId, FaceId)> = Vec::with_capacity(plan.pairs.len());
    let mut vetoed: Vec<String> = Vec::new();
    for (survivor, removed) in &plan.pairs {
        let survivor_attrs = topo.attributes().face(*survivor);
        let removed_attrs = topo.attributes().face(*removed);
        let compatible = match (survivor_attrs, removed_attrs) {
            (_, None) => true,
            (Some(kept), Some(dropped)) => kept == dropped,
            (None, Some(_)) => false,
        };
        if compatible {
            pairs.push((*survivor, *removed));
        } else {
            vetoed.push(format!("F{}<-F{}", survivor.index(), removed.index()));
        }
    }
    vetoed.sort();
    if !vetoed.is_empty() {
        ctx.info(format!(
            "kept {} duplicate pair(s) with incompatible attributes [{}]",
            vetoed.len(),
            vetoed.join(", ")
        ));
    }

    if pairs.is_empty() {
        return Ok(FixResult::ok());
    }

    // ReShape removals are global to this solid; a shared use cannot be dropped locally.
    if pairs
        .iter()
        .any(|(_, removed)| face_shell_uses.get(removed).is_some_and(|&uses| uses > 1))
    {
        return Err(HealError::FixFailed(
            "duplicate face is shared by multiple shells".into(),
        ));
    }

    // The lowest-index member of each group is never recorded as removed, so
    // at least one face always survives — the shell can't be emptied.
    for (_, removed) in &pairs {
        ctx.reshape.remove_face(*removed);
    }
    let removed = pairs.len();
    let mut provenance: Vec<String> = pairs
        .iter()
        .map(|(survivor, removed)| format!("F{}<-F{}", survivor.index(), removed.index()))
        .collect();
    provenance.sort();
    // The counts below are the scaling evidence trail (PERF-H03): how many
    // pairs the buckets drew versus how many reached the exact predicate.
    // They also keep the plan stats live outside tests.
    let fallback_note = if plan.fell_back_to_all_pairs {
        " via all-pairs fallback"
    } else {
        ""
    };
    ctx.info(format!(
        "removed {removed} duplicate face(s) [{}] ({} candidates, {} exact comparisons{fallback_note})",
        provenance.join(", "),
        plan.candidate_exams,
        plan.exact_comparisons
    ));

    Ok(FixResult::changed(
        Status::DONE2,
        super::RepairActionKind::DuplicateFaceRemoved,
        removed,
    ))
}

/// Build the duplicate descriptor for one face, or `None` when the face is
/// outside the supported domain (left in place, fail-closed).
///
/// Supported: planar carrier with outer and hole boundaries built from `Line`
/// segments and `Circle` arcs. Refused (`None`): non-planar carriers,
/// ellipse/hyperbola/parabola/NURBS boundary curves, degenerate or
/// unclassifiable loops (open loops, off-circle vertices, off-plane circle
/// centers, non-transverse circle axes, invalid spans).
///
/// # Errors
///
/// Returns [`HealError`] when entity lookups fail.
fn describe_face(
    topo: &Topology,
    face_id: FaceId,
    tolerance: f64,
) -> Result<Option<FaceDescriptor>, HealError> {
    let face = topo.face(face_id)?;
    let FaceSurface::Plane { normal, .. } = face.surface() else {
        return Ok(None);
    };
    let normal = if face.is_reversed() {
        -*normal
    } else {
        *normal
    };
    if !normal.x().is_finite() || !normal.y().is_finite() || !normal.z().is_finite() {
        return Ok(None);
    }

    let outer = match describe_wire(topo, face.outer_wire(), normal, tolerance)? {
        Some(boundary) => boundary,
        None => return Ok(None),
    };
    let mut holes = Vec::with_capacity(face.inner_wires().len());
    for &hole in face.inner_wires() {
        match describe_wire(topo, hole, normal, tolerance)? {
            Some(boundary) => holes.push(boundary),
            None => return Ok(None),
        }
    }

    let starts: Vec<Point3> = outer.iter().map(bucket_anchor).collect();
    let centroid = mean_point(&starts);
    Ok(Some(FaceDescriptor {
        face: face_id,
        normal,
        outer,
        holes,
        centroid,
    }))
}

/// Build the canonicalized segment loop for one boundary wire, or `None`
/// when the wire leaves the supported domain.
fn describe_wire(
    topo: &Topology,
    wire_id: remus_topology::wire::WireId,
    face_normal: Vec3,
    tolerance: f64,
) -> Result<Option<Vec<BoundarySeg>>, HealError> {
    use remus_topology::edge::EdgeCurve;

    let wire = topo.wire(wire_id)?;
    if wire.edges().is_empty() {
        return Ok(None);
    }
    let mut segs = Vec::with_capacity(wire.edges().len());
    // Interior connectivity is validated joint by joint: every segment must
    // start where the previous one ended (within tolerance). The merge
    // machinery below relies on connected chains; a gapped wire is refused.
    let mut first_start: Option<Point3> = None;
    let mut prev_end: Option<Point3> = None;
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let start: Point3 = topo.vertex(oe.oriented_start(edge))?.point();
        let end: Point3 = topo.vertex(oe.oriented_end(edge))?.point();
        if let Some(prev) = prev_end {
            if (start - prev).length() >= tolerance {
                return Ok(None);
            }
        } else {
            first_start = Some(start);
        }
        prev_end = Some(end);
        match edge.curve() {
            EdgeCurve::Line => segs.push(BoundarySeg::Line { start }),
            EdgeCurve::Circle(circle) => {
                match describe_arc(circle, edge, start, end, face_normal, tolerance) {
                    Some(seg) => segs.push(seg),
                    None => return Ok(None),
                }
            }
            EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_)
            | EdgeCurve::NurbsCurve(_) => return Ok(None),
        }
    }
    // The loop must close: the last segment must return to the first start.
    match (first_start, prev_end) {
        (Some(first), Some(last)) if (last - first).length() < tolerance => {}
        _ => return Ok(None),
    }
    let normalized = normalize_subdivision(segs, tolerance);
    if normalized.is_empty() {
        return Ok(None);
    }
    Ok(Some(normalized))
}

/// Closed-edge position coincidence band, mirroring
/// [`EdgeCurve::reconstruct_domain_from_endpoints`](remus_topology::edge::EdgeCurve::reconstruct_domain_from_endpoints).
/// Below this band an open arc's endpoints are read as a closed rim; the
/// choice cannot change a match verdict (comment at the call site).
const CLOSED_POSITION_EPS: f64 = 1e-9;

/// Describe one circle-arc segment, or `None` when it leaves the supported
/// domain.
///
/// The span comes from the edge's authoritative domain (stored trim when
/// present, endpoint reconstruction otherwise). Closed edges (shared vertex
/// or coincident endpoints) canonicalize to a full sweep with no seam phase,
/// so rims with different seam vertices still match. Open-arc sweeps are
/// canonicalized to the face-normal side, making the comparison independent
/// of the stored circle's arbitrary `u_axis`/normal choices.
fn describe_arc(
    circle: &remus_math::curves::Circle3D,
    edge: &remus_topology::edge::Edge,
    start: Point3,
    end: Point3,
    face_normal: Vec3,
    tolerance: f64,
) -> Option<BoundarySeg> {
    use std::f64::consts::TAU;

    let center = circle.center();
    let radius = circle.radius();
    let axis = circle.normal();
    for v in [center, start, end] {
        if !v.x().is_finite() || !v.y().is_finite() || !v.z().is_finite() {
            return None;
        }
    }
    if !radius.is_finite() || radius <= 0.0 || !tolerance.is_finite() || tolerance <= 0.0 {
        return None;
    }
    if !axis.x().is_finite() || !axis.y().is_finite() || !axis.z().is_finite() {
        return None;
    }
    // The circle must lie in the face plane: transverse axis or off-plane
    // center means degenerate input — refuse, never approximate.
    if axis.dot(face_normal).abs() < 1.0 - NORMAL_PARALLEL_COS_TOL {
        return None;
    }
    if ((center - start).dot(face_normal)).abs() >= tolerance {
        return None;
    }
    // Both endpoints must sit on the circle: vertices off a fitted curve are
    // exactly the case endpoint reconstruction gets wrong, so refuse.
    if ((start - center).length() - radius).abs() >= tolerance {
        return None;
    }
    if ((end - center).length() - radius).abs() >= tolerance {
        return None;
    }

    let closed = edge.is_closed() || (start - end).length() < CLOSED_POSITION_EPS;
    if closed {
        return Some(BoundarySeg::Arc {
            start,
            end,
            center,
            radius,
            sweep: TAU,
            closed: true,
        });
    }
    let (t0, t1) = edge.domain_with_endpoints(start, end);
    if !t0.is_finite() || !t1.is_finite() {
        return None;
    }
    let delta = t1 - t0;
    // A valid open-arc span lies strictly inside one full turn. Anything else
    // is degenerate authority — refuse rather than reinterpret. (Whether a
    // sub-`CLOSED_POSITION_EPS` gap reads as closed above cannot change a
    // match verdict: the reconstructed sweep of such a gap agrees with `TAU`
    // within the angular tolerance the predicate allows.)
    if delta <= 1e-12 || delta > TAU + 1e-9 {
        return None;
    }
    let sweep = if axis.dot(face_normal) > 0.0 {
        delta
    } else {
        TAU - delta
    };
    Some(BoundarySeg::Arc {
        start,
        end,
        center,
        radius,
        sweep,
        closed: false,
    })
}

/// Normalize valid subdivision differences without moving any vertex.
///
/// Drops degenerate (zero-length within tolerance) line segments, merges
/// consecutive collinear line segments, and merges consecutive same-circle
/// arcs. Only intermediate vertices strictly inside the surviving chord/arc
/// disappear, so no thin region can be snapped away: opposite sides are never
/// neighbors, and any deviation above tolerance blocks the merge. Restart
/// after each merge keeps indices trivially valid; at most `n` merges happen
/// on an `n`-segment loop. Deterministic: fixed circular order,
/// first-mergeable-joint wins, no hashing.
fn normalize_subdivision(mut segs: Vec<BoundarySeg>, tolerance: f64) -> Vec<BoundarySeg> {
    loop {
        if segs.len() < 2 {
            break;
        }
        let mut joint = None;
        for i in 0..segs.len() {
            if mergeable_joint(&segs, i, tolerance) {
                joint = Some(i);
                break;
            }
        }
        let Some(i) = joint else { break };
        let j = (i + 1) % segs.len();
        let merged = merge_joint(&segs, i, j, tolerance);
        if j == 0 {
            // Wrap joint (last, first): the merged segment takes the head.
            segs = std::iter::once(merged)
                .chain(segs.drain(1..segs.len() - 1))
                .collect();
        } else {
            segs[i] = merged;
            segs.remove(j);
        }
    }
    segs
}

/// Whether the circular joint `(i, j = i+1 mod n)` may be merged.
fn mergeable_joint(segs: &[BoundarySeg], i: usize, tolerance: f64) -> bool {
    let n = segs.len();
    let j = (i + 1) % n;
    let k = (j + 1) % n;
    match (&segs[i], &segs[j]) {
        (BoundarySeg::Line { start: a }, BoundarySeg::Line { start: b }) => {
            let c = segs[k].start();
            if (*b - *a).length() < tolerance {
                // Degenerate segment: traces nothing, always droppable.
                return true;
            }
            if (c - *a).length() < tolerance {
                // Degenerate chord with a real segment: keep, never collapse.
                return false;
            }
            // `b` strictly between `a` and `c`, within tolerance of the chord.
            let chord = c - *a;
            let deviation = (*b - *a).cross(chord).length() / chord.length();
            deviation < tolerance && (*b - *a).dot(chord) > 0.0 && (*b - c).dot(*a - c) > 0.0
        }
        (BoundarySeg::Arc { .. }, BoundarySeg::Line { start: b }) => {
            // Degenerate line after an arc: drop the line, keep the arc.
            (segs[k].start() - *b).length() < tolerance
        }
        (BoundarySeg::Line { start: a }, arc @ BoundarySeg::Arc { .. }) => {
            // Degenerate line before an arc: drop the line, keep the arc.
            (arc.start() - *a).length() < tolerance
        }
        (
            BoundarySeg::Arc {
                center: c1,
                radius: r1,
                sweep: s1,
                ..
            },
            BoundarySeg::Arc {
                start,
                center: c2,
                radius: r2,
                sweep: s2,
                ..
            },
        ) => {
            use std::f64::consts::TAU;
            let prev_end = match &segs[i] {
                BoundarySeg::Arc { end, .. } => *end,
                BoundarySeg::Line { .. } => return false,
            };
            // Contiguous on one circle (canonical sweeps are comparable: both
            // were canonicalized to the face-normal side at build time), and
            // the merged arc must not exceed one full turn (so closed rims,
            // sweep TAU, never absorb a neighbor).
            (*start - prev_end).length() < tolerance
                && (*c1 - *c2).length() < tolerance
                && (*r1 - *r2).abs() < tolerance
                && *s1 + *s2 <= TAU + angle_tolerance(tolerance, r1.max(*r2))
        }
    }
}

/// Merge the circular joint `(i, j = i+1 mod n)`; caller checked
/// [`mergeable_joint`].
fn merge_joint(segs: &[BoundarySeg], i: usize, j: usize, _tolerance: f64) -> BoundarySeg {
    match (&segs[i], &segs[j]) {
        (BoundarySeg::Line { start: a }, BoundarySeg::Line { .. }) => {
            BoundarySeg::Line { start: *a }
        }
        (arc, BoundarySeg::Line { .. }) => arc.clone(),
        (BoundarySeg::Line { .. }, arc) => arc.clone(),
        (
            BoundarySeg::Arc {
                start,
                center,
                radius,
                sweep: s1,
                ..
            },
            BoundarySeg::Arc { end, sweep: s2, .. },
        ) => BoundarySeg::Arc {
            start: *start,
            end: *end,
            center: *center,
            radius: *radius,
            sweep: *s1 + *s2,
            // Merged arcs keep explicit endpoints (a two-edge full circle
            // still has a seam vertex); only single closed-rim edges are
            // phase-invariant.
            closed: false,
        },
    }
}

/// Angular tolerance for comparing two arc sweeps of radius `radius`:
/// the angle subtending one linear tolerance.
fn angle_tolerance(tolerance: f64, radius: f64) -> f64 {
    tolerance / radius.max(tolerance)
}

/// Plan duplicate removals over index-ordered descriptors.
///
/// Streaming `j`-ascending scan with an in-neighborhood ascending-`i`
/// candidate walk: equivalent to the all-pairs reference over the same order
/// (module docs), minus provably non-matching pairs.
fn plan_duplicate_removals(descriptors: &[FaceDescriptor], tolerance: f64) -> DuplicatePlan {
    let n = descriptors.len();
    if n < 2 {
        return DuplicatePlan {
            pairs: Vec::new(),
            candidate_exams: 0,
            exact_comparisons: 0,
            fell_back_to_all_pairs: false,
        };
    }
    if !tolerance.is_finite() || tolerance <= 0.0 {
        return all_pairs_plan(descriptors, tolerance, true);
    }

    // Per-face bucket keys; faces without a conservative key fall back to
    // universal candidacy (compared against every other face).
    let mut keys: Vec<Option<Bucket>> = Vec::with_capacity(n);
    let mut any_fallback = false;
    for d in descriptors {
        let key = bucket_key(d, tolerance);
        any_fallback |= key.is_none();
        keys.push(key);
    }
    if any_fallback && keys.iter().all(Option::is_none) {
        return all_pairs_plan(descriptors, tolerance, true);
    }

    // Buckets grow in index order, so every bucket is ascending by construction.
    // Two levels (signature, then spatial cells) so halo lookups never clone
    // the hole-signature vector.
    let mut buckets: DetHashMap<Signature, DetHashMap<(Cell3, Cell3), Vec<usize>>> =
        DetHashMap::default();
    for (idx, key) in keys.iter().enumerate() {
        if let Some((signature, cells)) = key {
            buckets
                .entry(signature.clone())
                .or_default()
                .entry(*cells)
                .or_default()
                .push(idx);
        }
    }

    let mut removed: Vec<bool> = vec![false; n];
    let mut pairs: Vec<(FaceId, FaceId)> = Vec::new();
    let mut candidate_exams = 0u64;
    let mut exact_comparisons = 0u64;
    let mut scratch: Vec<usize> = Vec::new();

    for j in 0..n {
        if removed[j] {
            continue;
        }
        scratch.clear();
        match &keys[j] {
            // Degenerate descriptor: every earlier face is a candidate.
            None => scratch.extend(0..j),
            Some((signature, ((nx, ny, nz), (cx, cy, cz)))) => {
                let inner = buckets.get(signature);
                for dx in -1..=1_i64 {
                    for dy in -1..=1_i64 {
                        for dz in -1..=1_i64 {
                            for dnx in -1..=1_i64 {
                                for dny in -1..=1_i64 {
                                    for dnz in -1..=1_i64 {
                                        let hit = inner.as_ref().and_then(|inner| {
                                            inner.get(&(
                                                (nx + dnx, ny + dny, nz + dnz),
                                                (cx + dx, cy + dy, cz + dz),
                                            ))
                                        });
                                        if let Some(bucket) = hit {
                                            let len = bucket.partition_point(|&i| i < j);
                                            scratch.extend_from_slice(&bucket[..len]);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                // Universal candidates: fallback faces below `j`.
                for (i, key) in keys.iter().enumerate().take(j) {
                    if key.is_none() {
                        scratch.push(i);
                    }
                }
                scratch.sort_unstable();
                scratch.dedup();
            }
        }
        for &i in &scratch {
            candidate_exams += 1;
            if removed[i] {
                continue;
            }
            exact_comparisons += 1;
            if faces_are_duplicates(&descriptors[i], &descriptors[j], tolerance) {
                removed[j] = true;
                pairs.push((descriptors[i].face, descriptors[j].face));
                break;
            }
        }
    }

    DuplicatePlan {
        pairs,
        candidate_exams,
        exact_comparisons,
        fell_back_to_all_pairs: false,
    }
}

/// Exact legacy all-pairs loop over the same descriptor order.
///
/// Preserved as the degenerate-input fallback and as the production-code
/// shape of the equivalence oracle (tests re-implement it independently in
/// `reference_all_pairs`).
fn all_pairs_plan(
    descriptors: &[FaceDescriptor],
    tolerance: f64,
    fell_back: bool,
) -> DuplicatePlan {
    let n = descriptors.len();
    let mut removed = vec![false; n];
    let mut pairs: Vec<(FaceId, FaceId)> = Vec::new();
    let mut exact_comparisons = 0u64;
    for i in 0..n {
        if removed[i] {
            continue;
        }
        for j in (i + 1)..n {
            if removed[j] {
                continue;
            }
            exact_comparisons += 1;
            if faces_are_duplicates(&descriptors[i], &descriptors[j], tolerance) {
                removed[j] = true;
                pairs.push((descriptors[i].face, descriptors[j].face));
            }
        }
    }
    DuplicatePlan {
        pairs,
        candidate_exams: exact_comparisons,
        exact_comparisons,
        fell_back_to_all_pairs: fell_back,
    }
}

/// Conservative bucket key for one descriptor, or `None` when no safe key
/// exists (the face becomes a universal candidate).
fn bucket_key(descriptor: &FaceDescriptor, tolerance: f64) -> Option<Bucket> {
    let n = descriptor.normal;
    if !n.x().is_finite() || !n.y().is_finite() || !n.z().is_finite() {
        return None;
    }
    let c = descriptor.centroid;
    if !c.x().is_finite() || !c.y().is_finite() || !c.z().is_finite() {
        return None;
    }
    let cell = |v: f64| v / tolerance;
    if cell(c.x()).abs() > MAX_EXACT_CELL
        || cell(c.y()).abs() > MAX_EXACT_CELL
        || cell(c.z()).abs() > MAX_EXACT_CELL
    {
        return None;
    }
    Some((
        boundary_signature(descriptor),
        (
            (
                (n.x() / NORMAL_CELL).floor() as i64,
                (n.y() / NORMAL_CELL).floor() as i64,
                (n.z() / NORMAL_CELL).floor() as i64,
            ),
            (
                cell(c.x()).floor() as i64,
                cell(c.y()).floor() as i64,
                cell(c.z()).floor() as i64,
            ),
        ),
    ))
}

/// Exact boundary signature: outer segment count plus sorted hole segment
/// counts. The predicate requires equality on both, so this key component
/// can never exclude a true match.
fn boundary_signature(descriptor: &FaceDescriptor) -> Signature {
    let mut hole_lens: Vec<usize> = descriptor.holes.iter().map(Vec::len).collect();
    hole_lens.sort_unstable();
    (descriptor.outer.len(), hole_lens)
}

/// Exact duplicate predicate — the final authority.
///
/// Effective normals must agree within [`NORMAL_PARALLEL_COS_TOL`], the
/// oriented outer boundaries must coincide under some cyclic shift with the
/// same winding, and holes must correspond one-to-one with coincident
/// boundaries (hole storage order is arbitrary, so matching is by search).
fn faces_are_duplicates(a: &FaceDescriptor, b: &FaceDescriptor, tolerance: f64) -> bool {
    if a.outer.len() != b.outer.len() || a.holes.len() != b.holes.len() {
        return false;
    }
    if a.normal.dot(b.normal) < 1.0 - NORMAL_PARALLEL_COS_TOL {
        return false;
    }
    if !loops_coincide_with_same_winding(&a.outer, &b.outer, tolerance) {
        return false;
    }
    holes_correspond(&a.holes, &b.holes, tolerance)
}

/// Whether two boundary loops trace the same oriented region: equal segment
/// counts with elementwise-compatible segments under some cyclic shift, same
/// winding (reversed order never matches).
fn loops_coincide_with_same_winding(a: &[BoundarySeg], b: &[BoundarySeg], tolerance: f64) -> bool {
    if a.is_empty() || a.len() != b.len() {
        return false;
    }
    (0..b.len()).any(|offset| {
        segs_compatible(&a[0], &b[offset], tolerance)
            && (1..a.len())
                .all(|index| segs_compatible(&a[index], &b[(offset + index) % b.len()], tolerance))
    })
}

/// Whether two same-position segments trace the same directed curve piece:
/// same kind (a chord never matches an arc) and — for arcs — the same
/// supporting circle and canonical sweep. Endpoints must coincide except
/// where a closed rim is involved: rims carry no seam phase, so rim–rim
/// pairs compare by circle and sweep alone, and rim–arc pairs additionally
/// require the arc's gap to be invisible at tolerance (via sweep agreement —
/// a real gap changes the sweep beyond the angular tolerance).
fn segs_compatible(a: &BoundarySeg, b: &BoundarySeg, tolerance: f64) -> bool {
    match (a, b) {
        (BoundarySeg::Line { start: s1 }, BoundarySeg::Line { start: s2 }) => {
            (*s1 - *s2).length() < tolerance
        }
        (
            BoundarySeg::Arc {
                start: s1,
                end: e1,
                center: c1,
                radius: r1,
                sweep: w1,
                closed: k1,
            },
            BoundarySeg::Arc {
                start: s2,
                end: e2,
                center: c2,
                radius: r2,
                sweep: w2,
                closed: k2,
            },
        ) => {
            (*c1 - *c2).length() < tolerance
                && (*r1 - *r2).abs() < tolerance
                && (*w1 - *w2).abs() < angle_tolerance(tolerance, r1.max(*r2))
                && (*k1 && *k2
                    || (*s1 - *s2).length() < tolerance && (*e1 - *e2).length() < tolerance)
        }
        (BoundarySeg::Line { .. }, BoundarySeg::Arc { .. })
        | (BoundarySeg::Arc { .. }, BoundarySeg::Line { .. }) => false,
    }
}

/// Whether every hole of `a` coincides with exactly one hole of `b` and vice
/// versa (equal counts checked by the caller).
///
/// Deterministic augmenting-path matching in stored order: complete (finds a
/// bijection whenever one exists), so recognition never depends on hole
/// storage order. Greedy failure would keep genuine duplicates; search
/// completeness is part of the contract proof (one-to-one correspondence).
fn holes_correspond(a: &[Vec<BoundarySeg>], b: &[Vec<BoundarySeg>], tolerance: f64) -> bool {
    if a.len() != b.len() {
        return false;
    }
    // match_b[j] = index in `a` currently claiming hole `j` of `b`.
    let mut match_b: Vec<Option<usize>> = vec![None; b.len()];
    for i in 0..a.len() {
        let mut seen = vec![false; b.len()];
        if !augment_hole(i, a, b, tolerance, &mut seen, &mut match_b) {
            return false;
        }
    }
    true
}

/// One DFS step of the hole bijection search.
fn augment_hole(
    i: usize,
    a: &[Vec<BoundarySeg>],
    b: &[Vec<BoundarySeg>],
    tolerance: f64,
    seen: &mut [bool],
    match_b: &mut [Option<usize>],
) -> bool {
    for (j, hole_b) in b.iter().enumerate() {
        if seen[j] {
            continue;
        }
        if !loops_coincide_with_same_winding(&a[i], hole_b, tolerance) {
            continue;
        }
        seen[j] = true;
        if match_b[j].is_none_or(|prev| augment_hole(prev, a, b, tolerance, seen, match_b)) {
            match_b[j] = Some(i);
            return true;
        }
    }
    false
}

fn mean_point(points: &[Point3]) -> Point3 {
    let n = points.len() as f64;
    let (sx, sy, sz) = points.iter().fold((0.0, 0.0, 0.0), |(sx, sy, sz), p| {
        (sx + p.x(), sy + p.y(), sz + p.z())
    });
    Point3::new(sx / n, sy / n, sz / n)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::print_stderr)]
mod tests {
    use super::*;
    use remus_math::curves::Circle3D;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::Face;
    use remus_topology::shell::Shell;
    use remus_topology::solid::Solid;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    /// Add a planar (+Z) triangle face with the given corner points.
    fn add_triangle(topo: &mut Topology, a: Point3, b: Point3, c: Point3) -> FaceId {
        let va = topo.add_vertex(Vertex::new(a, 1e-7));
        let vb = topo.add_vertex(Vertex::new(b, 1e-7));
        let vc = topo.add_vertex(Vertex::new(c, 1e-7));
        let eab = topo.add_edge(Edge::new(va, vb, EdgeCurve::Line));
        let ebc = topo.add_edge(Edge::new(vb, vc, EdgeCurve::Line));
        let eca = topo.add_edge(Edge::new(vc, va, EdgeCurve::Line));
        let wire = Wire::new(
            vec![
                OrientedEdge::new(eab, true),
                OrientedEdge::new(ebc, true),
                OrientedEdge::new(eca, true),
            ],
            true,
        )
        .unwrap();
        let wid = topo.add_wire(wire);
        topo.add_face(Face::new(
            wid,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    /// Add a planar (+Z) quad face with the given corners in order.
    fn add_quad(topo: &mut Topology, corners: [Point3; 4]) -> FaceId {
        let vs: Vec<_> = corners
            .iter()
            .map(|p| topo.add_vertex(Vertex::new(*p, 1e-7)))
            .collect();
        let es: Vec<_> = (0..4)
            .map(|i| topo.add_edge(Edge::new(vs[i], vs[(i + 1) % 4], EdgeCurve::Line)))
            .collect();
        let wire = Wire::new(
            es.into_iter().map(|e| OrientedEdge::new(e, true)).collect(),
            true,
        )
        .unwrap();
        let wid = topo.add_wire(wire);
        topo.add_face(Face::new(
            wid,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    /// Descriptors for every eligible face of `shell`, in ascending `FaceId`
    /// order (mirrors `fix_shell_duplicate_faces` without recording).
    fn describe_shell(
        topo: &Topology,
        shell: remus_topology::shell::ShellId,
        tol: f64,
    ) -> Vec<FaceDescriptor> {
        let mut out = Vec::new();
        for &fid in topo.shell(shell).unwrap().faces() {
            if let Some(descriptor) = describe_face(topo, fid, tol).unwrap() {
                out.push(descriptor);
            }
        }
        out.sort_by_key(|d| d.face.index());
        out
    }

    /// Independent all-pairs oracle over the same descriptor order: legacy
    /// `i`-outer loop, own predicate evaluation written separately from the
    /// production predicate (own loop walk, own hole bijection, own arc
    /// comparison), no buckets.
    fn reference_all_pairs(
        descriptors: &[FaceDescriptor],
        tolerance: f64,
    ) -> Vec<(FaceId, FaceId)> {
        let n = descriptors.len();
        let mut removed = vec![false; n];
        let mut pairs = Vec::new();
        for i in 0..n {
            if removed[i] {
                continue;
            }
            for j in (i + 1)..n {
                if removed[j] {
                    continue;
                }
                if reference_duplicates(&descriptors[i], &descriptors[j], tolerance) {
                    removed[j] = true;
                    pairs.push((descriptors[i].face, descriptors[j].face));
                }
            }
        }
        pairs
    }

    /// Independently written duplicate check over descriptor data.
    fn reference_duplicates(a: &FaceDescriptor, b: &FaceDescriptor, tol: f64) -> bool {
        if a.outer.len() != b.outer.len() || a.holes.len() != b.holes.len() {
            return false;
        }
        if a.normal.dot(b.normal) < 1.0 - NORMAL_PARALLEL_COS_TOL {
            return false;
        }
        if !reference_loops_match(&a.outer, &b.outer, tol) {
            return false;
        }
        if a.holes.len() != b.holes.len() {
            return false;
        }
        // Independent hole bijection: exhaustive permutation search over the
        // (small) hole sets instead of the production augmenting-path match.
        let mut order: Vec<usize> = (0..b.holes.len()).collect();
        loop {
            let matched = a
                .holes
                .iter()
                .zip(order.iter())
                .all(|(ha, &jb)| reference_loops_match(ha, &b.holes[jb], tol));
            if matched {
                return true;
            }
            if !next_permutation(&mut order) {
                return false;
            }
        }
    }

    /// Independently written oriented-loop coincidence: cyclic-shift walk with
    /// per-kind endpoint comparison.
    fn reference_loops_match(a: &[BoundarySeg], b: &[BoundarySeg], tol: f64) -> bool {
        if a.is_empty() || a.len() != b.len() {
            return false;
        }
        for offset in 0..b.len() {
            let mut ok = true;
            for (index, sa) in a.iter().enumerate() {
                if !reference_segs_match(sa, &b[(offset + index) % b.len()], tol) {
                    ok = false;
                    break;
                }
            }
            if ok {
                return true;
            }
        }
        false
    }

    /// Independently written segment comparison (mirrors the production
    /// closed-rim rule: rims compare by circle and sweep alone).
    fn reference_segs_match(a: &BoundarySeg, b: &BoundarySeg, tol: f64) -> bool {
        match (a, b) {
            (BoundarySeg::Line { start: s1 }, BoundarySeg::Line { start: s2 }) => {
                (*s1 - *s2).length() < tol
            }
            (
                BoundarySeg::Arc {
                    start: s1,
                    end: e1,
                    center: c1,
                    radius: r1,
                    sweep: w1,
                    closed: k1,
                },
                BoundarySeg::Arc {
                    start: s2,
                    end: e2,
                    center: c2,
                    radius: r2,
                    sweep: w2,
                    closed: k2,
                },
            ) => {
                (*c1 - *c2).length() < tol
                    && (*r1 - *r2).abs() < tol
                    && (*w1 - *w2).abs() < tol / r1.max(*r2).max(tol)
                    && (*k1 && *k2 || (*s1 - *s2).length() < tol && (*e1 - *e2).length() < tol)
            }
            _ => false,
        }
    }

    /// Lexicographic next permutation; false when the last one was reached.
    fn next_permutation(order: &mut [usize]) -> bool {
        if order.len() < 2 {
            return false;
        }
        let mut i = order.len() - 2;
        loop {
            if order[i] < order[i + 1] {
                break;
            }
            if i == 0 {
                order.reverse();
                return false;
            }
            i -= 1;
        }
        let mut j = order.len() - 1;
        while order[j] <= order[i] {
            j -= 1;
        }
        order.swap(i, j);
        order[i + 1..].reverse();
        true
    }

    fn assert_plan_equals_reference(
        descriptors: &[FaceDescriptor],
        tolerance: f64,
    ) -> DuplicatePlan {
        let plan = plan_duplicate_removals(descriptors, tolerance);
        let reference = reference_all_pairs(descriptors, tolerance);
        // Same decisions (survivor and removed per pair), up to emission
        // order: the plan streams `j`-ascending (pairs sorted by removed
        // face), the reference scans `i`-outer (pairs sorted by survivor).
        // Both orders are deterministic; only the sets must agree.
        let mut planned = plan.pairs.clone();
        let mut expected = reference.clone();
        planned.sort_by_key(|p| (p.0.index(), p.1.index()));
        expected.sort_by_key(|p| (p.0.index(), p.1.index()));
        assert_eq!(
            planned, expected,
            "indexed plan must equal the all-pairs reference (survivor, removals)"
        );
        assert!(
            plan.pairs
                .windows(2)
                .all(|w| w[0].1.index() < w[1].1.index()),
            "plan pairs must stream in ascending removed-face order"
        );
        // The index may only skip exact evaluations, never change the outcome.
        let dense = all_pairs_plan(descriptors, tolerance, false);
        assert_eq!(dense.pairs, reference);
        assert!(
            plan.exact_comparisons <= dense.exact_comparisons.max(1),
            "index must not evaluate more pairs than all-pairs"
        );
        plan
    }

    /// Deterministic LCG for reproducible fixtures (no external deps).
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

    #[test]
    fn duplicate_repair_refuses_removal_of_a_shared_face_identity() {
        let mut topo = Topology::new();
        let mut faces = Vec::new();
        for _ in 0..2 {
            faces.push(add_triangle(
                &mut topo,
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ));
        }
        let outer = topo.add_shell(Shell::new(vec![faces[1]]).unwrap());
        let inner = topo.add_shell(Shell::new(faces).unwrap());
        let solid = topo.add_solid(Solid::new(outer, vec![inner]));
        let mut ctx = HealContext::new();
        let error = fix_duplicate_faces(&topo, solid, &mut ctx).unwrap_err();
        assert!(error.to_string().contains("shared by multiple shells"));
        assert!(ctx.reshape.is_empty());
    }

    #[test]
    fn duplicate_repair_visits_cavities_without_merging_across_shells() {
        let mut topo = Topology::new();
        let mut triangles = Vec::new();
        for _ in 0..3 {
            triangles.push(add_triangle(
                &mut topo,
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ));
        }
        // Coincident shell-local fixtures isolate duplicate scope, not cavity containment.
        let outer = topo.add_shell(Shell::new(vec![triangles[0]]).unwrap());
        let inner = topo.add_shell(Shell::new(vec![triangles[1], triangles[2]]).unwrap());
        let solid = topo.add_solid(Solid::new(outer, vec![inner]));
        let mut ctx = HealContext::new();
        let report = fix_duplicate_faces(&topo, solid, &mut ctx).unwrap();
        assert_eq!(report.actions_taken, 1);
        assert!(!ctx.reshape.is_face_removed(triangles[0]));
        assert!(!ctx.reshape.is_face_removed(triangles[1]));
        assert!(ctx.reshape.is_face_removed(triangles[2]));
        ctx.reshape.apply(&mut topo, solid).unwrap();
        assert_eq!(
            topo.shell(topo.solid(solid).unwrap().outer_shell())
                .unwrap()
                .faces()
                .len(),
            1
        );
        assert_eq!(
            topo.shell(topo.solid(solid).unwrap().inner_shells()[0])
                .unwrap()
                .faces()
                .len(),
            1
        );
    }

    #[test]
    fn flags_and_removes_a_coincident_duplicate_face() {
        let mut topo = Topology::new();
        // Three distinct faces plus an exact geometric duplicate of the first.
        let a = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let b = add_triangle(
            &mut topo,
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(6.0, 0.0, 0.0),
            Point3::new(5.0, 1.0, 0.0),
        );
        let c = add_triangle(
            &mut topo,
            Point3::new(0.0, 5.0, 0.0),
            Point3::new(1.0, 5.0, 0.0),
            Point3::new(0.0, 6.0, 0.0),
        );
        let dup = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let shell = topo.add_shell(Shell::new(vec![a, b, c, dup]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));

        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();

        assert_eq!(result.actions_taken, 1, "exactly one duplicate expected");
        assert!(
            ctx.reshape.is_face_removed(dup),
            "the later face is removed"
        );
        assert!(!ctx.reshape.is_face_removed(a), "the original is kept");
        assert!(!ctx.reshape.is_face_removed(b));

        // Applying the reshape drops the duplicate from the shell.
        ctx.reshape.apply(&mut topo, solid_id).unwrap();
        let faces = topo
            .shell(topo.solid(solid_id).unwrap().outer_shell())
            .unwrap();
        assert_eq!(faces.faces().len(), 3, "shell drops the duplicate");
    }

    #[test]
    fn keeps_all_distinct_faces() {
        let mut topo = Topology::new();
        let a = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let b = add_triangle(
            &mut topo,
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(6.0, 0.0, 0.0),
            Point3::new(5.0, 1.0, 0.0),
        );
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));

        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();
        assert_eq!(
            result.actions_taken, 0,
            "no duplicates among distinct faces"
        );
    }

    #[test]
    fn keeps_same_centroid_faces_with_different_boundaries() {
        let mut topo = Topology::new();
        let small = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(0.0, 3.0, 0.0),
        );
        let large = add_triangle(
            &mut topo,
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(5.0, -1.0, 0.0),
            Point3::new(-1.0, 5.0, 0.0),
        );
        let shell = topo.add_shell(Shell::new(vec![small, large]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));

        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();

        assert_eq!(result.actions_taken, 0);
        assert!(!ctx.reshape.is_face_removed(small));
        assert!(!ctx.reshape.is_face_removed(large));
    }

    #[test]
    fn keeps_coincident_face_with_opposite_winding() {
        let mut topo = Topology::new();
        let original = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let opposite = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        let shell = topo.add_shell(Shell::new(vec![original, opposite]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));

        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();

        assert_eq!(result.actions_taken, 0);
        assert!(!ctx.reshape.is_face_removed(original));
        assert!(!ctx.reshape.is_face_removed(opposite));
    }

    #[test]
    fn keeps_coincident_face_with_opposite_effective_normal() {
        let mut topo = Topology::new();
        let original = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let opposite = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        topo.face_mut(opposite).unwrap().set_reversed(true);
        let shell = topo.add_shell(Shell::new(vec![original, opposite]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));

        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();

        assert_eq!(result.actions_taken, 0);
        assert!(!ctx.reshape.is_face_removed(original));
        assert!(!ctx.reshape.is_face_removed(opposite));
    }

    // ── PERF-H03 indexing proofs ──

    #[test]
    fn survivor_is_lowest_index_regardless_of_shell_order() {
        let mut topo = Topology::new();
        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let first = add_quad(&mut topo, corners);
        let second = add_quad(&mut topo, corners);
        // Shell order is reverse index order: the survivor must still be the
        // lowest FaceId, not the first shell entry.
        let shell = topo.add_shell(Shell::new(vec![second, first]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));
        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();
        assert_eq!(result.actions_taken, 1);
        assert!(!ctx.reshape.is_face_removed(first));
        assert!(ctx.reshape.is_face_removed(second));
    }

    #[test]
    fn sparse_model_pays_no_exact_comparisons() {
        // 400 disjoint coplanar quads on an integer grid: same edge count and
        // same normal cell, but centroid buckets never meet.
        let mut topo = Topology::new();
        let mut faces = Vec::new();
        for i in 0..400 {
            let x = f64::from(i) * 10.0;
            faces.push(add_quad(
                &mut topo,
                [
                    Point3::new(x, 0.0, 0.0),
                    Point3::new(x + 1.0, 0.0, 0.0),
                    Point3::new(x + 1.0, 1.0, 0.0),
                    Point3::new(x, 1.0, 0.0),
                ],
            ));
        }
        let shell = topo.add_shell(Shell::new(faces).unwrap());
        let descriptors = describe_shell(&topo, shell, 1e-7);
        let plan = assert_plan_equals_reference(&descriptors, 1e-7);
        assert!(plan.pairs.is_empty());
        assert_eq!(plan.exact_comparisons, 0);
        assert_eq!(plan.candidate_exams, 0);
        assert!(!plan.fell_back_to_all_pairs);
    }

    #[test]
    fn dense_coincident_exits_early_on_the_survivor() {
        // 60 identical quads: one bucket, but each face after the first
        // matches the survivor immediately — linear exams and exact calls.
        let mut topo = Topology::new();
        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let mut faces = Vec::new();
        for _ in 0..60 {
            faces.push(add_quad(&mut topo, corners));
        }
        let shell = topo.add_shell(Shell::new(faces).unwrap());
        let descriptors = describe_shell(&topo, shell, 1e-7);
        let plan = assert_plan_equals_reference(&descriptors, 1e-7);
        assert_eq!(plan.pairs.len(), 59);
        assert_eq!(plan.exact_comparisons, 59);
        assert_eq!(plan.candidate_exams, 59);
        assert!(!plan.fell_back_to_all_pairs);
    }

    #[test]
    fn dense_bucket_distinct_faces_pay_quadratic_exams() {
        // Honest worst case: 60 faces sharing one bucket (identical centroid
        // by cancelling perturbations, same normal, same edge count) that are
        // pairwise all distinct (corner drift of 2*tol per step, so no cyclic
        // shift can align any two within tol). Every pair is drawn and reaches
        // the exact predicate: exams and exact calls both stay quadratic.
        let tol = 1e-7;
        let mut topo = Topology::new();
        let mut faces = Vec::new();
        for k in 0..60 {
            let d = f64::from(k) * 2.0 * tol;
            faces.push(add_quad(
                &mut topo,
                [
                    Point3::new(d, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(1.0 - d, 1.0, 0.0),
                    Point3::new(0.0, 1.0, 0.0),
                ],
            ));
        }
        let shell = topo.add_shell(Shell::new(faces).unwrap());
        let descriptors = describe_shell(&topo, shell, tol);
        // All centroids coincide: single bucket by construction.
        let first = &descriptors[0];
        for d in &descriptors {
            assert!((d.centroid - first.centroid).length() < 1e-9);
        }
        let plan = assert_plan_equals_reference(&descriptors, tol);
        assert!(plan.pairs.is_empty(), "all faces are pairwise distinct");
        assert_eq!(plan.candidate_exams, 60 * 59 / 2);
        assert_eq!(plan.exact_comparisons, 60 * 59 / 2);
        assert!(!plan.fell_back_to_all_pairs);
    }

    #[test]
    fn tolerance_boundary_pairs_do_not_match() {
        let tol = 1e-7;
        let mut topo = Topology::new();
        let base = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        // Exactly tol away in one corner: strict `< tol` keeps both faces.
        let at_tol = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0 + tol, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        // Just inside tol: duplicates.
        let inside = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0 + tol * (1.0 - 1e-6), 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let a = add_quad(&mut topo, base);
        let b = add_quad(&mut topo, at_tol);
        let c = add_quad(&mut topo, inside);
        let shell = topo.add_shell(Shell::new(vec![a, b, c]).unwrap());
        let descriptors = describe_shell(&topo, shell, tol);
        let plan = assert_plan_equals_reference(&descriptors, tol);
        // b is kept (boundary distance == tol), c duplicates a.
        assert_eq!(
            plan.pairs,
            vec![(a, c)],
            "at-tolerance pair is kept, inside-tolerance pair merges"
        );
    }

    #[test]
    fn nontransitive_nearness_keeps_the_distant_face() {
        // Chain at 0.6*tol shifts: A≈B and B≈C match, but A≉C (1.2*tol).
        // B is removed into A; C must stay — no transitive chaining.
        let tol = 1e-7;
        let shift = |dx: f64| {
            [
                Point3::new(dx, 0.0, 0.0),
                Point3::new(1.0 + dx, 0.0, 0.0),
                Point3::new(1.0 + dx, 1.0, 0.0),
                Point3::new(dx, 1.0, 0.0),
            ]
        };
        let mut topo = Topology::new();
        let a = add_quad(&mut topo, shift(0.0));
        let b = add_quad(&mut topo, shift(0.6 * tol));
        let c = add_quad(&mut topo, shift(1.2 * tol));
        let shell = topo.add_shell(Shell::new(vec![a, b, c]).unwrap());
        let descriptors = describe_shell(&topo, shell, tol);
        let plan = assert_plan_equals_reference(&descriptors, tol);
        assert_eq!(plan.pairs, vec![(a, b)]);
    }

    #[test]
    fn cell_boundary_straddlers_neither_miss_nor_invent() {
        let tol = 1e-7;
        // Centroid straddling a cell border at x = 10*tol, 4e-9 apart overall:
        // the shifted copy is a duplicate and must be found across the border.
        let mut topo = Topology::new();
        let base_x = 10.0 * tol - 2e-9;
        let mk = |dx: f64| {
            [
                Point3::new(base_x + dx, 0.0, 0.0),
                Point3::new(base_x + dx + 1.0, 0.0, 0.0),
                Point3::new(base_x + dx + 1.0, 1.0, 0.0),
                Point3::new(base_x + dx, 1.0, 0.0),
            ]
        };
        let a = add_quad(&mut topo, mk(0.0));
        let b = add_quad(&mut topo, mk(4e-9));
        // Far quad sharing the normal cell but nothing else: never a candidate.
        let c = add_quad(
            &mut topo,
            [
                Point3::new(500.0, 0.0, 0.0),
                Point3::new(501.0, 0.0, 0.0),
                Point3::new(501.0, 1.0, 0.0),
                Point3::new(500.0, 1.0, 0.0),
            ],
        );
        let shell = topo.add_shell(Shell::new(vec![a, b, c]).unwrap());
        let descriptors = describe_shell(&topo, shell, tol);
        let plan = assert_plan_equals_reference(&descriptors, tol);
        assert_eq!(plan.pairs, vec![(a, b)]);
    }

    #[test]
    fn degenerate_tolerance_falls_back_without_changing_the_map() {
        let mut topo = Topology::new();
        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let a = add_quad(&mut topo, corners);
        let b = add_quad(&mut topo, corners);
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let descriptors = describe_shell(&topo, shell, 1e-7);
        for tol in [0.0, -1e-7, f64::NAN, f64::INFINITY] {
            let plan = plan_duplicate_removals(&descriptors, tol);
            assert!(plan.fell_back_to_all_pairs, "tol {tol}");
            assert_eq!(plan.pairs, reference_all_pairs(&descriptors, tol));
        }
    }

    #[test]
    fn beyond_grid_range_falls_back() {
        // 1e16 with tol 1e-7 needs centroid cell index 1e23: beyond exact
        // integers, so both faces fall back — still deciding correctly.
        let mut topo = Topology::new();
        let mk = |dx: f64| {
            [
                Point3::new(1e16 + dx, 0.0, 0.0),
                Point3::new(1e16 + dx + 1.0, 0.0, 0.0),
                Point3::new(1e16 + dx + 1.0, 1.0, 0.0),
                Point3::new(1e16 + dx, 1.0, 0.0),
            ]
        };
        let a = add_quad(&mut topo, mk(0.0));
        let b = add_quad(&mut topo, mk(0.0));
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let descriptors = describe_shell(&topo, shell, 1e-7);
        let plan = plan_duplicate_removals(&descriptors, 1e-7);
        assert!(plan.fell_back_to_all_pairs);
        assert_eq!(plan.pairs, vec![(a, b)]);
    }

    #[test]
    fn randomized_fixtures_match_reference() {
        let tol = 1e-7;
        for size in [0, 1, 2, 3, 17, 100, 250] {
            // Sparse grid with a negative offset (cell-boundary coverage).
            let mut topo = Topology::new();
            let mut faces = Vec::new();
            for i in 0..size {
                let x = -50.0 + f64::from(i) * 1.7;
                faces.push(add_quad(
                    &mut topo,
                    [
                        Point3::new(x, 3.25, -1.5),
                        Point3::new(x + 1.0, 3.25, -1.5),
                        Point3::new(x + 1.0, 4.25, -1.5),
                        Point3::new(x, 4.25, -1.5),
                    ],
                ));
            }
            // One exact duplicate of the first face when non-empty.
            if size > 0 {
                let x = -50.0;
                faces.push(add_quad(
                    &mut topo,
                    [
                        Point3::new(x, 3.25, -1.5),
                        Point3::new(x + 1.0, 3.25, -1.5),
                        Point3::new(x + 1.0, 4.25, -1.5),
                        Point3::new(x, 4.25, -1.5),
                    ],
                ));
            }
            if faces.is_empty() {
                continue;
            }
            let shell = topo.add_shell(Shell::new(faces).unwrap());
            let descriptors = describe_shell(&topo, shell, tol);
            let plan = assert_plan_equals_reference(&descriptors, tol);
            if size > 0 {
                assert_eq!(plan.pairs.len(), 1, "size {size}: one duplicate");
            }

            // Clustered near-duplicates with deterministic jitter around
            // shared bases (some inside tol, some outside).
            let mut topo = Topology::new();
            let mut rng = Lcg(0xabcd + size as u64);
            let mut faces = Vec::new();
            for i in 0..size {
                let base = f64::from(i / 3) * 2.3 - 100.0;
                let jx = (rng.next_f64() - 0.5) * 1.2e-7;
                let jy = (rng.next_f64() - 0.5) * 1.2e-7;
                faces.push(add_quad(
                    &mut topo,
                    [
                        Point3::new(base + jx, jy, 0.0),
                        Point3::new(base + jx + 1.0, jy, 0.0),
                        Point3::new(base + jx + 1.0, jy + 1.0, 0.0),
                        Point3::new(base + jx, jy + 1.0, 0.0),
                    ],
                ));
            }
            if faces.is_empty() {
                continue;
            }
            let shell = topo.add_shell(Shell::new(faces).unwrap());
            let descriptors = describe_shell(&topo, shell, tol);
            assert_plan_equals_reference(&descriptors, tol);

            // Adversarial: many coincident copies plus jittered neighbors.
            let mut topo = Topology::new();
            let mut rng = Lcg(0x55aa + size as u64);
            let mut faces = Vec::new();
            for _ in 0..size {
                let jx = rng.next_f64() * 5e-8;
                let jy = rng.next_f64() * 5e-8;
                faces.push(add_quad(
                    &mut topo,
                    [
                        Point3::new(-7.0 + jx, 11.0 + jy, 0.0),
                        Point3::new(-6.0 + jx, 11.0 + jy, 0.0),
                        Point3::new(-6.0 + jx, 12.0 + jy, 0.0),
                        Point3::new(-7.0 + jx, 12.0 + jy, 0.0),
                    ],
                ));
            }
            if faces.is_empty() {
                continue;
            }
            let shell = topo.add_shell(Shell::new(faces).unwrap());
            let descriptors = describe_shell(&topo, shell, tol);
            assert_plan_equals_reference(&descriptors, tol);
        }
    }

    #[test]
    fn randomized_holed_fixtures_match_reference() {
        // 0-2 holes per face from deterministic placement; every third face
        // duplicates an earlier one (sometimes with holes reordered).
        let tol = 1e-7;
        for size in [3, 17, 80] {
            let mut topo = Topology::new();
            let mut rng = Lcg(0xB017 + size as u64);
            let mut faces = Vec::new();
            let hole_spot = |k: usize| {
                let bx = (k % 5) as f64 * 10.0;
                let by = (k / 5 % 5) as f64 * 10.0;
                [
                    [bx + 1.0, by + 1.0, 0.0],
                    [bx + 2.0, by + 1.0, 0.0],
                    [bx + 2.0, by + 2.0, 0.0],
                    [bx + 1.0, by + 2.0, 0.0],
                ]
            };
            for i in 0..size {
                if i % 3 == 2 && !faces.is_empty() {
                    // Duplicate face 0's geometry with fresh topology; even
                    // iterations reverse the hole storage order.
                    let flip = (i / 3) % 2 == 0;
                    let mut hs = vec![hole_spot(0)];
                    if i % 2 == 0 {
                        hs.push(hole_spot(1));
                    }
                    if flip {
                        hs.reverse();
                    }
                    let ox = 0.0 + (rng.next_f64() - 0.5) * 1e-9;
                    let shifted: [Point3; 4] =
                        UNIT_OUTER.map(|p| Point3::new(p.x() + ox, p.y(), p.z()));
                    faces.push(add_holed_quad(&mut topo, shifted, &hs));
                    continue;
                }
                let ox = (i as f64) * 10.0;
                let outer = [
                    Point3::new(ox, 0.0, 0.0),
                    Point3::new(ox + 4.0, 0.0, 0.0),
                    Point3::new(ox + 4.0, 4.0, 0.0),
                    Point3::new(ox, 4.0, 0.0),
                ];
                let mut hs = Vec::new();
                if rng.next_f64() < 0.7 {
                    hs.push(hole_spot(i));
                }
                if rng.next_f64() < 0.3 {
                    hs.push(hole_spot(i + 100));
                }
                faces.push(add_holed_quad(&mut topo, outer, &hs));
            }
            let shell = topo.add_shell(Shell::new(faces).unwrap());
            let descriptors = describe_shell(&topo, shell, tol);
            assert_plan_equals_reference(&descriptors, tol);
        }
    }

    /// Scaling evidence (M5): prints candidate/exact counts, bucket spread
    /// and wall time per fixture. Manual measurement only (`--ignored`):
    /// asserts correctness against the reference, never timing.
    #[ignore = "measurement: prints the PERF-H03 scaling table; run release with --nocapture"]
    #[test]
    fn scaling_measurement_report() {
        use std::collections::BTreeSet;
        use std::time::Instant;

        // Each entry: one quad's (x, y) origin; quads are unit squares.
        fn sparse_origins(n: usize) -> Vec<[f64; 2]> {
            (0..n).map(|i| [i as f64 * 10.0, 0.0]).collect()
        }
        fn clustered_origins(n: usize) -> Vec<[f64; 2]> {
            let side = n.isqrt().max(1);
            (0..n)
                .map(|i| [(i % side) as f64 * 1.5e-7, (i / side) as f64 * 1.5e-7])
                .collect()
        }
        fn coincident_origins(n: usize) -> Vec<[f64; 2]> {
            vec![[0.0, 0.0]; n]
        }
        let cases: Vec<(&str, Vec<usize>)> = vec![
            ("sparse", vec![200, 800, 2000]),
            ("clustered", vec![200, 800]),
            ("coincident", vec![200, 800]),
            ("dense-distinct", vec![60, 200]),
        ];
        eprintln!(
            "{:>14} {:>6} {:>8} {:>10} {:>10} {:>10} {:>10}",
            "case", "n", "buckets", "candidates", "exact", "ref-exact", "ms"
        );
        for (name, sizes) in cases {
            for size in sizes {
                let mut topo = Topology::new();
                let mut faces = Vec::new();
                if name == "dense-distinct" {
                    // Centroid-preserving 2*tol drifts: one bucket, all
                    // pairwise distinct (honest quadratic worst case).
                    for k in 0..size {
                        let d = k as f64 * 2.0 * 1e-7;
                        faces.push(add_quad(
                            &mut topo,
                            [
                                Point3::new(d, 0.0, 0.0),
                                Point3::new(1.0, 0.0, 0.0),
                                Point3::new(1.0 - d, 1.0, 0.0),
                                Point3::new(0.0, 1.0, 0.0),
                            ],
                        ));
                    }
                } else {
                    let origins = match name {
                        "sparse" => sparse_origins(size),
                        "clustered" => clustered_origins(size),
                        _ => coincident_origins(size),
                    };
                    for [x, y] in origins {
                        faces.push(add_quad(
                            &mut topo,
                            [
                                Point3::new(x, y, 0.0),
                                Point3::new(x + 1.0, y, 0.0),
                                Point3::new(x + 1.0, y + 1.0, 0.0),
                                Point3::new(x, y + 1.0, 0.0),
                            ],
                        ));
                    }
                }
                let shell = topo.add_shell(Shell::new(faces).unwrap());
                let descriptors = describe_shell(&topo, shell, 1e-7);
                let buckets: BTreeSet<Bucket> = descriptors
                    .iter()
                    .map(|d| bucket_key(d, 1e-7).unwrap())
                    .collect();
                let start = Instant::now();
                let plan = plan_duplicate_removals(&descriptors, 1e-7);
                let elapsed = start.elapsed();
                let reference = reference_all_pairs(&descriptors, 1e-7);
                let mut planned = plan.pairs.clone();
                let mut expected = reference.clone();
                planned.sort_by_key(|p| (p.0.index(), p.1.index()));
                expected.sort_by_key(|p| (p.0.index(), p.1.index()));
                assert_eq!(planned, expected, "plan must equal reference");
                let dense = all_pairs_plan(&descriptors, 1e-7, false);
                eprintln!(
                    "{:>14} {:>6} {:>8} {:>10} {:>10} {:>10} {:>10.2}",
                    name,
                    size,
                    buckets.len(),
                    plan.candidate_exams,
                    plan.exact_comparisons,
                    dense.exact_comparisons,
                    elapsed.as_secs_f64() * 1000.0,
                );
            }
        }
    }

    #[test]
    fn plan_is_deterministic_across_runs() {
        let mut topo = Topology::new();
        let mut rng = Lcg(0xbeef);
        let mut faces = Vec::new();
        for i in 0..600 {
            let base = f64::from(i / 3) * 1.1;
            let jx = (rng.next_f64() - 0.5) * 1.2e-7;
            let jy = (rng.next_f64() - 0.5) * 1.2e-7;
            faces.push(add_quad(
                &mut topo,
                [
                    Point3::new(base + jx, jy, 0.0),
                    Point3::new(base + jx + 1.0, jy, 0.0),
                    Point3::new(base + jx + 1.0, jy + 1.0, 0.0),
                    Point3::new(base + jx, jy + 1.0, 0.0),
                ],
            ));
        }
        let shell = topo.add_shell(Shell::new(faces).unwrap());
        let descriptors = describe_shell(&topo, shell, 1e-7);
        let a = plan_duplicate_removals(&descriptors, 1e-7);
        let b = plan_duplicate_removals(&descriptors, 1e-7);
        assert_eq!(a.pairs, b.pairs);
        assert_eq!(a.candidate_exams, b.candidate_exams);
        assert_eq!(a.exact_comparisons, b.exact_comparisons);
    }

    // ── M3: perforated planar faces ──

    /// Hole corner specification (4 corners as [x, y, z] triples).
    type HoleCorners = [[f64; 3]; 4];

    /// Add a planar (+Z) quad with `holes` inner quad wires.
    fn add_holed_quad(topo: &mut Topology, outer: [Point3; 4], holes: &[HoleCorners]) -> FaceId {
        let mut quad_wire = |corners: &[Point3]| {
            let vs: Vec<_> = corners
                .iter()
                .map(|p| topo.add_vertex(Vertex::new(*p, 1e-7)))
                .collect();
            let es: Vec<_> = (0..4)
                .map(|i| topo.add_edge(Edge::new(vs[i], vs[(i + 1) % 4], EdgeCurve::Line)))
                .collect();
            topo.add_wire(
                Wire::new(
                    es.into_iter().map(|e| OrientedEdge::new(e, true)).collect(),
                    true,
                )
                .unwrap(),
            )
        };
        let outer_wire = quad_wire(&outer);
        let hole_wires: Vec<_> = holes
            .iter()
            .map(|h| quad_wire(&h.map(|p| Point3::new(p[0], p[1], p[2]))))
            .collect();
        topo.add_face(Face::new(
            outer_wire,
            hole_wires,
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    const UNIT_OUTER: [Point3; 4] = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(4.0, 0.0, 0.0),
        Point3::new(4.0, 4.0, 0.0),
        Point3::new(0.0, 4.0, 0.0),
    ];
    const HOLE_A: HoleCorners = [
        [1.0, 1.0, 0.0],
        [2.0, 1.0, 0.0],
        [2.0, 2.0, 0.0],
        [1.0, 2.0, 0.0],
    ];
    const HOLE_B: HoleCorners = [
        [2.5, 2.5, 0.0],
        [3.0, 2.5, 0.0],
        [3.0, 3.0, 0.0],
        [2.5, 3.0, 0.0],
    ];

    fn assert_shell_removals(
        topo: &Topology,
        shell: remus_topology::shell::ShellId,
        tol: f64,
    ) -> DuplicatePlan {
        let descriptors = describe_shell(topo, shell, tol);
        assert_plan_equals_reference(&descriptors, tol)
    }

    #[test]
    fn holed_duplicates_match_with_same_holes() {
        let mut topo = Topology::new();
        let a = add_holed_quad(&mut topo, UNIT_OUTER, &[HOLE_A]);
        let b = add_holed_quad(&mut topo, UNIT_OUTER, &[HOLE_A]);
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let plan = assert_shell_removals(&topo, shell, 1e-7);
        assert_eq!(plan.pairs.len(), 1);
    }

    #[test]
    fn holed_duplicates_match_with_reordered_holes() {
        // Hole storage order is arbitrary: two holes stored in opposite
        // order still correspond one-to-one via the bijection search.
        let mut topo = Topology::new();
        let a = add_holed_quad(&mut topo, UNIT_OUTER, &[HOLE_A, HOLE_B]);
        let b = add_holed_quad(&mut topo, UNIT_OUTER, &[HOLE_B, HOLE_A]);
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let plan = assert_shell_removals(&topo, shell, 1e-7);
        assert_eq!(plan.pairs.len(), 1);
    }

    #[test]
    fn holed_faces_keep_genuine_hole_differences() {
        // Same outer, but: missing hole, extra hole, shifted hole.
        let shifted: HoleCorners = [
            [1.0 + 5e-6, 1.0, 0.0],
            [2.0 + 5e-6, 1.0, 0.0],
            [2.0 + 5e-6, 2.0, 0.0],
            [1.0 + 5e-6, 2.0, 0.0],
        ];
        let cases: [(&str, Vec<HoleCorners>, Vec<HoleCorners>); 3] = [
            ("missing_hole", vec![HOLE_A, HOLE_B], vec![HOLE_A]),
            ("extra_hole", vec![HOLE_A], vec![HOLE_A, HOLE_B]),
            ("shifted_hole", vec![HOLE_A], vec![shifted]),
        ];
        for (label, holes_a, holes_b) in cases {
            let mut topo = Topology::new();
            let a = add_holed_quad(&mut topo, UNIT_OUTER, &holes_a);
            let b = add_holed_quad(&mut topo, UNIT_OUTER, &holes_b);
            let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
            let plan = assert_shell_removals(&topo, shell, 1e-7);
            assert!(
                plan.pairs.is_empty(),
                "{label}: hole difference must be kept"
            );
        }
    }

    #[test]
    fn holed_face_keeps_unholed_twin() {
        let mut topo = Topology::new();
        let a = add_holed_quad(&mut topo, UNIT_OUTER, &[HOLE_A]);
        let b = add_quad(&mut topo, UNIT_OUTER);
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let plan = assert_shell_removals(&topo, shell, 1e-7);
        assert!(plan.pairs.is_empty(), "perforated vs clean must be kept");
    }

    #[test]
    fn holed_face_keeps_reversed_hole_winding() {
        // Same hole region, opposite hole traversal: a reversed
        // representation the comparator does not normalize — fail-closed.
        let mut topo = Topology::new();
        let a = add_holed_quad(&mut topo, UNIT_OUTER, &[HOLE_A]);
        let mut reversed_hole = HOLE_A;
        reversed_hole.reverse();
        let b = add_holed_quad(&mut topo, UNIT_OUTER, &[reversed_hole]);
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let plan = assert_shell_removals(&topo, shell, 1e-7);
        assert!(
            plan.pairs.is_empty(),
            "reversed hole winding is kept, not normalized"
        );
    }

    // ── M3: subdivision normalization ──

    /// Quad whose first side is split at `fractions` (collinear vertices).
    fn add_subdivided_quad(topo: &mut Topology, fractions: &[f64]) -> FaceId {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(4.0, 0.0, 0.0);
        let c = Point3::new(4.0, 4.0, 0.0);
        let d = Point3::new(0.0, 4.0, 0.0);
        let mut pts = vec![a];
        for f in fractions {
            pts.push(Point3::new(4.0 * f, 0.0, 0.0));
        }
        pts.push(b);
        pts.push(c);
        pts.push(d);
        let vs: Vec<_> = pts
            .iter()
            .map(|p| topo.add_vertex(Vertex::new(*p, 1e-7)))
            .collect();
        let es: Vec<_> = (0..vs.len())
            .map(|i| topo.add_edge(Edge::new(vs[i], vs[(i + 1) % vs.len()], EdgeCurve::Line)))
            .collect();
        let wire = topo.add_wire(
            Wire::new(
                es.into_iter().map(|e| OrientedEdge::new(e, true)).collect(),
                true,
            )
            .unwrap(),
        );
        topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    #[test]
    fn collinear_subdivision_matches_clean_boundary() {
        let mut topo = Topology::new();
        let clean = add_quad(&mut topo, UNIT_OUTER);
        let split = add_subdivided_quad(&mut topo, &[0.25, 0.5, 0.75]);
        let shell = topo.add_shell(Shell::new(vec![clean, split]).unwrap());
        let plan = assert_shell_removals(&topo, shell, 1e-7);
        assert_eq!(plan.pairs.len(), 1, "collinear splits normalize away");
    }

    #[test]
    fn stepped_side_is_not_snapped_away() {
        // A 5*tol inward step on one side is a genuine region difference:
        // normalization moves no vertex, so the step survives and the pair
        // is kept.
        let mut topo = Topology::new();
        let clean = add_quad(&mut topo, UNIT_OUTER);
        let stepped = add_quad(
            &mut topo,
            [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 5e-7, 0.0),
                Point3::new(4.0, 4.0, 0.0),
                Point3::new(0.0, 4.0, 0.0),
            ],
        );
        let shell = topo.add_shell(Shell::new(vec![clean, stepped]).unwrap());
        let plan = assert_shell_removals(&topo, shell, 1e-7);
        assert!(plan.pairs.is_empty(), "stepped side must be kept");
    }

    #[test]
    fn inward_spike_is_not_snapped_away() {
        // Same endpoints, but one boundary detours inward by 5*tol through
        // an extra vertex: less material, so the pair is kept.
        let mut topo = Topology::new();
        let clean = add_quad(&mut topo, UNIT_OUTER);
        let vs: Vec<_> = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 5e-7, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(4.0, 4.0, 0.0),
            Point3::new(0.0, 4.0, 0.0),
        ]
        .iter()
        .map(|p| topo.add_vertex(Vertex::new(*p, 1e-7)))
        .collect();
        let es: Vec<_> = (0..vs.len())
            .map(|i| topo.add_edge(Edge::new(vs[i], vs[(i + 1) % vs.len()], EdgeCurve::Line)))
            .collect();
        let wire = topo.add_wire(
            Wire::new(
                es.into_iter().map(|e| OrientedEdge::new(e, true)).collect(),
                true,
            )
            .unwrap(),
        );
        let spiked = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![clean, spiked]).unwrap());
        let plan = assert_shell_removals(&topo, shell, 1e-7);
        assert!(plan.pairs.is_empty(), "inward spike must be kept");
    }

    #[test]
    fn degenerate_zero_length_edge_normalizes_away() {
        // A doubled vertex (zero-length edge) traces nothing: it normalizes
        // away and the pair matches.
        let mut topo = Topology::new();
        let clean = add_quad(&mut topo, UNIT_OUTER);
        let vs: Vec<_> = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(4.0, 4.0, 0.0),
            Point3::new(0.0, 4.0, 0.0),
        ]
        .iter()
        .map(|p| topo.add_vertex(Vertex::new(*p, 1e-7)))
        .collect();
        let es: Vec<_> = (0..vs.len())
            .map(|i| topo.add_edge(Edge::new(vs[i], vs[(i + 1) % vs.len()], EdgeCurve::Line)))
            .collect();
        let wire = topo.add_wire(
            Wire::new(
                es.into_iter().map(|e| OrientedEdge::new(e, true)).collect(),
                true,
            )
            .unwrap(),
        );
        let doubled = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![clean, doubled]).unwrap());
        let plan = assert_shell_removals(&topo, shell, 1e-7);
        assert_eq!(plan.pairs.len(), 1, "doubled vertex normalizes away");
    }

    // ── M3: circle/arc boundaries ──

    /// Add a planar (+Z, d=0) disc face: one closed circle edge with the seam
    /// vertex at `seam_angle`.
    fn add_disc(topo: &mut Topology, center: Point3, radius: f64, seam_angle: f64) -> FaceId {
        let circle = Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), radius).unwrap();
        let seam = Point3::new(
            center.x() + radius * seam_angle.cos(),
            center.y() + radius * seam_angle.sin(),
            center.z(),
        );
        let v = topo.add_vertex(Vertex::new(seam, 1e-7));
        let edge = topo.add_edge(Edge::new(v, v, EdgeCurve::Circle(circle)));
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
        topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    #[test]
    fn disc_rims_match_regardless_of_seam_vertex() {
        // Same geometric rim, seam vertices 90° apart: the closed rim carries
        // no phase, so the pair matches.
        let mut topo = Topology::new();
        let center = Point3::new(1.0, 2.0, 0.0);
        let a = add_disc(&mut topo, center, 1.5, 0.0);
        let b = add_disc(&mut topo, center, 1.5, std::f64::consts::FRAC_PI_2);
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let plan = assert_shell_removals(&topo, shell, 1e-7);
        assert_eq!(plan.pairs.len(), 1, "seam position must not matter");
    }

    #[test]
    fn discs_keep_radius_and_center_differences() {
        let center = Point3::new(0.0, 0.0, 0.0);
        for (label, other) in [
            ("radius", (Point3::new(0.0, 0.0, 0.0), 1.5 + 5e-6)),
            ("center", (Point3::new(5e-6, 0.0, 0.0), 1.5)),
        ] {
            let mut topo = Topology::new();
            let a = add_disc(&mut topo, center, 1.5, 0.0);
            let b = add_disc(&mut topo, other.0, other.1, 0.0);
            let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
            let plan = assert_shell_removals(&topo, shell, 1e-7);
            assert!(plan.pairs.is_empty(), "{label} difference must be kept");
        }
    }

    #[test]
    fn disc_keeps_same_area_square() {
        // Equal area proves nothing: a disc and a square of equal area bound
        // different regions (and different segment kinds).
        let mut topo = Topology::new();
        let disc = add_disc(&mut topo, Point3::new(0.0, 0.0, 0.0), 1.0, 0.0);
        let side = std::f64::consts::PI.sqrt();
        let square = add_quad(
            &mut topo,
            [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(side, 0.0, 0.0),
                Point3::new(side, side, 0.0),
                Point3::new(0.0, side, 0.0),
            ],
        );
        let shell = topo.add_shell(Shell::new(vec![disc, square]).unwrap());
        let plan = assert_shell_removals(&topo, shell, 1e-7);
        assert!(plan.pairs.is_empty(), "disc vs square must be kept");
    }

    /// Build the rounded-corner square: [0,2]² with the (2,2) corner rounded
    /// at radius 0.5 (3 lines + 1 quarter arc). `rotate` starts the wire at a
    /// different edge (cyclic shift); `split_arc` halves the arc into two
    /// same-circle arcs (subdivision difference).
    fn add_rounded_corner(topo: &mut Topology, rotate: usize, split_arc: bool) -> FaceId {
        let pt = |x: f64, y: f64| Point3::new(x, y, 0.0);
        let p00 = topo.add_vertex(Vertex::new(pt(0.0, 0.0), 1e-7));
        let p20 = topo.add_vertex(Vertex::new(pt(2.0, 0.0), 1e-7));
        let p215 = topo.add_vertex(Vertex::new(pt(2.0, 1.5), 1e-7));
        let p152 = topo.add_vertex(Vertex::new(pt(1.5, 2.0), 1e-7));
        let p02 = topo.add_vertex(Vertex::new(pt(0.0, 2.0), 1e-7));
        let center = Point3::new(1.5, 1.5, 0.0);
        let circle = Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), 0.5).unwrap();
        let e0 = topo.add_edge(Edge::new(p00, p20, EdgeCurve::Line));
        let e1 = topo.add_edge(Edge::new(p20, p215, EdgeCurve::Line));
        let mut oes = vec![OrientedEdge::new(e0, true), OrientedEdge::new(e1, true)];
        if split_arc {
            let pmid = topo.add_vertex(Vertex::new(
                pt(
                    1.5 + 0.5 * std::f64::consts::FRAC_1_SQRT_2,
                    1.5 + 0.5 * std::f64::consts::FRAC_1_SQRT_2,
                ),
                1e-7,
            ));
            let ea = topo.add_edge(Edge::new(p215, pmid, EdgeCurve::Circle(circle.clone())));
            let eb = topo.add_edge(Edge::new(pmid, p152, EdgeCurve::Circle(circle)));
            oes.push(OrientedEdge::new(ea, true));
            oes.push(OrientedEdge::new(eb, true));
        } else {
            let ea = topo.add_edge(Edge::new(p215, p152, EdgeCurve::Circle(circle)));
            oes.push(OrientedEdge::new(ea, true));
        }
        let e3 = topo.add_edge(Edge::new(p152, p02, EdgeCurve::Line));
        let e4 = topo.add_edge(Edge::new(p02, p00, EdgeCurve::Line));
        oes.push(OrientedEdge::new(e3, true));
        oes.push(OrientedEdge::new(e4, true));
        let rot = rotate % oes.len();
        oes.rotate_left(rot);
        let wire = topo.add_wire(Wire::new(oes, true).unwrap());
        topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    #[test]
    fn rounded_corners_match_across_cyclic_shift_and_arc_split() {
        let mut topo = Topology::new();
        let a = add_rounded_corner(&mut topo, 0, false);
        let b = add_rounded_corner(&mut topo, 3, true);
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let plan = assert_shell_removals(&topo, shell, 1e-7);
        assert_eq!(
            plan.pairs.len(),
            1,
            "cyclic shift + arc subdivision must match"
        );
    }

    #[test]
    fn chord_is_never_equated_with_arc() {
        // Same corner points, but one face closes the rounded corner with a
        // straight chord while the other carries the arc: same endpoints,
        // different regions — kept.
        let mut topo = Topology::new();
        let arc_face = add_rounded_corner(&mut topo, 0, false);
        let q = |x: f64, y: f64| Point3::new(x, y, 0.0);
        let p00 = topo.add_vertex(Vertex::new(q(0.0, 0.0), 1e-7));
        let p20 = topo.add_vertex(Vertex::new(q(2.0, 0.0), 1e-7));
        let p215 = topo.add_vertex(Vertex::new(q(2.0, 1.5), 1e-7));
        let p152 = topo.add_vertex(Vertex::new(q(1.5, 2.0), 1e-7));
        let p02 = topo.add_vertex(Vertex::new(q(0.0, 2.0), 1e-7));
        let mk = |a, b| Edge::new(a, b, EdgeCurve::Line);
        let e0 = topo.add_edge(mk(p00, p20));
        let e1 = topo.add_edge(mk(p20, p215));
        let e2 = topo.add_edge(mk(p215, p152));
        let e3 = topo.add_edge(mk(p152, p02));
        let e4 = topo.add_edge(mk(p02, p00));
        let oes = vec![
            OrientedEdge::new(e0, true),
            OrientedEdge::new(e1, true),
            OrientedEdge::new(e2, true),
            OrientedEdge::new(e3, true),
            OrientedEdge::new(e4, true),
        ];
        let wire = topo.add_wire(Wire::new(oes, true).unwrap());
        let chord_face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![arc_face, chord_face]).unwrap());
        let plan = assert_shell_removals(&topo, shell, 1e-7);
        assert!(plan.pairs.is_empty(), "chord vs arc must be kept");
    }

    #[test]
    fn gapped_arc_loop_keeps_closed_rim() {
        // A nearly-closed arc (350° span) closed by a chord vs the true disc
        // rim: genuinely different boundaries — kept.
        let mut topo = Topology::new();
        let center = Point3::new(0.0, 0.0, 0.0);
        let rim = add_disc(&mut topo, center, 1.0, 0.0);
        let circle = Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let gap = 10.0_f64.to_radians();
        let a0 = gap / 2.0;
        let a1 = std::f64::consts::TAU - gap / 2.0;
        let pa = Point3::new(a0.cos(), a0.sin(), 0.0);
        let pb = Point3::new(a1.cos(), a1.sin(), 0.0);
        let va = topo.add_vertex(Vertex::new(pa, 1e-7));
        let vb = topo.add_vertex(Vertex::new(pb, 1e-7));
        let arc = topo.add_edge(Edge::new(va, vb, EdgeCurve::Circle(circle)));
        let chord = topo.add_edge(Edge::new(vb, va, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![OrientedEdge::new(arc, true), OrientedEdge::new(chord, true)],
                true,
            )
            .unwrap(),
        );
        let gapped = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![rim, gapped]).unwrap());
        let plan = assert_shell_removals(&topo, shell, 1e-7);
        assert!(plan.pairs.is_empty(), "gapped arc must not match the rim");
    }

    #[test]
    fn disc_rims_match_across_flipped_circle_frame() {
        // Same rim on a circle whose stored frame is flipped (opposite
        // normal): closed rims canonicalize to a full sweep either way.
        let mut topo = Topology::new();
        let center = Point3::new(1.0, 2.0, 0.0);
        let a = add_disc(&mut topo, center, 1.5, 0.0);
        let circle = Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), 1.5).unwrap();
        let flipped = circle.reversed();
        let seam = flipped.evaluate(1.0);
        let v = topo.add_vertex(Vertex::new(seam, 1e-7));
        let edge = topo.add_edge(Edge::new(v, v, EdgeCurve::Circle(flipped)));
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
        let b = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let plan = assert_shell_removals(&topo, shell, 1e-7);
        assert_eq!(plan.pairs.len(), 1, "flipped rim frame must match");
    }

    #[test]
    fn same_geometric_arc_matches_across_flipped_frame() {
        // Same geometric quarter arc, endpoints shared as frame-invariant
        // points, on circles whose stored frames are flipped: the canonical
        // sweep absorbs the frame flip. The flipped frame traverses
        // (1,0)->(0,1) the short way only when the edge direction is
        // complemented too, so the twin edge runs (0,1)->(1,0) on the flipped
        // circle while the wire keeps the same traversal via a reversed use.
        let mut topo = Topology::new();
        let center = Point3::new(0.0, 0.0, 0.0);
        let circle = Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let flipped = circle.reversed();
        let pa = Point3::new(1.0, 0.0, 0.0);
        let pb = Point3::new(0.0, 1.0, 0.0);
        let pc = Point3::new(-0.5, -0.5, 0.0);
        // Face A: arc (1,0)->(0,1), quarter sweep, +Z frame.
        let va = topo.add_vertex(Vertex::new(pa, 1e-7));
        let vb = topo.add_vertex(Vertex::new(pb, 1e-7));
        let vc = topo.add_vertex(Vertex::new(pc, 1e-7));
        let arc_a = topo.add_edge(Edge::new(va, vb, EdgeCurve::Circle(circle)));
        let lab_a = topo.add_edge(Edge::new(vb, vc, EdgeCurve::Line));
        let lac_a = topo.add_edge(Edge::new(vc, va, EdgeCurve::Line));
        let wire_a = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(arc_a, true),
                    OrientedEdge::new(lab_a, true),
                    OrientedEdge::new(lac_a, true),
                ],
                true,
            )
            .unwrap(),
        );
        let a = topo.add_face(Face::new(
            wire_a,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        // Face B: same geometric arc as (0,1)->(1,0) on the flipped circle,
        // used backwards so the wire traverses (1,0)->(0,1) identically.
        let wa = topo.add_vertex(Vertex::new(pa, 1e-7));
        let wb = topo.add_vertex(Vertex::new(pb, 1e-7));
        let wc = topo.add_vertex(Vertex::new(pc, 1e-7));
        let arc_b = topo.add_edge(Edge::new(wb, wa, EdgeCurve::Circle(flipped)));
        let lab_b = topo.add_edge(Edge::new(wb, wc, EdgeCurve::Line));
        let lac_b = topo.add_edge(Edge::new(wc, wa, EdgeCurve::Line));
        let wire_b = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(arc_b, false),
                    OrientedEdge::new(lab_b, true),
                    OrientedEdge::new(lac_b, true),
                ],
                true,
            )
            .unwrap(),
        );
        let b = topo.add_face(Face::new(
            wire_b,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let plan = assert_shell_removals(&topo, shell, 1e-7);
        assert_eq!(plan.pairs.len(), 1, "frame flip must canonicalize away");
    }

    #[test]
    fn reversed_arc_traversal_is_kept() {
        // Same rounded corner traced backwards: opposite winding, same
        // unoriented region — kept, like the line-only opposite-winding pin.
        let mut topo = Topology::new();
        let forward = add_rounded_corner(&mut topo, 0, false);
        // Rebuild the same loop backwards: reversed edge order, flipped uses.
        let face = topo.face(forward).unwrap().clone();
        let wire = topo.wire(face.outer_wire()).unwrap().clone();
        let oes: Vec<OrientedEdge> = wire
            .edges()
            .iter()
            .rev()
            .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
            .collect();
        let wire_id = topo.add_wire(Wire::new(oes, true).unwrap());
        let backward = topo.add_face(Face::new(
            wire_id,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![forward, backward]).unwrap());
        let plan = assert_shell_removals(&topo, shell, 1e-7);
        assert!(plan.pairs.is_empty(), "reversed traversal must be kept");
    }

    #[test]
    fn unsupported_curves_and_carriers_are_refused() {
        // Ellipse / NURBS boundaries and non-planar carriers never enter the
        // candidate set: identical twins are left in place with no removal
        // and no error.
        use remus_math::curves::Ellipse3D;
        use remus_math::nurbs::curve::NurbsCurve;
        use remus_math::surfaces::CylindricalSurface;
        let mut topo = Topology::new();
        // Ellipse-bounded planar twins.
        let ellipse = Ellipse3D::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            2.0,
            1.0,
        )
        .unwrap();
        let mut ellipse_faces = Vec::new();
        for _ in 0..2 {
            let v0 = topo.add_vertex(Vertex::new(Point3::new(2.0, 0.0, 0.0), 1e-7));
            let edge = topo.add_edge(Edge::new(v0, v0, EdgeCurve::Ellipse(ellipse.clone())));
            let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
            ellipse_faces.push(topo.add_face(Face::new(
                wire,
                vec![],
                FaceSurface::Plane {
                    normal: Vec3::new(0.0, 0.0, 1.0),
                    d: 0.0,
                },
            )));
        }
        // NURBS-bounded planar twins (straight segment as degree-1 NURBS).
        let nurbs = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::new(5.0, 0.0, 0.0), Point3::new(6.0, 0.0, 0.0)],
            vec![1.0, 1.0],
        )
        .unwrap();
        let mut nurbs_faces = Vec::new();
        for _ in 0..2 {
            let v0 = topo.add_vertex(Vertex::new(Point3::new(5.0, 0.0, 0.0), 1e-7));
            let v1 = topo.add_vertex(Vertex::new(Point3::new(6.0, 0.0, 0.0), 1e-7));
            let v2 = topo.add_vertex(Vertex::new(Point3::new(5.5, 1.0, 0.0), 1e-7));
            let e0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::NurbsCurve(nurbs.clone())));
            let e1 = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
            let e2 = topo.add_edge(Edge::new(v2, v0, EdgeCurve::Line));
            let wire = topo.add_wire(
                Wire::new(
                    vec![
                        OrientedEdge::new(e0, true),
                        OrientedEdge::new(e1, true),
                        OrientedEdge::new(e2, true),
                    ],
                    true,
                )
                .unwrap(),
            );
            nurbs_faces.push(topo.add_face(Face::new(
                wire,
                vec![],
                FaceSurface::Plane {
                    normal: Vec3::new(0.0, 0.0, 1.0),
                    d: 0.0,
                },
            )));
        }
        // Cylindrical-carrier planar-boundary twins.
        let cyl =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0)
                .unwrap();
        let mut cyl_faces = Vec::new();
        for _ in 0..2 {
            cyl_faces.push(add_triangle(
                &mut topo,
                Point3::new(10.0, 0.0, 0.0),
                Point3::new(11.0, 0.0, 0.0),
                Point3::new(10.0, 1.0, 0.0),
            ));
            let last = *cyl_faces.last().unwrap();
            topo.face_mut(last)
                .unwrap()
                .set_surface(FaceSurface::Cylinder(cyl.clone()));
        }
        let mut all = ellipse_faces;
        all.extend(nurbs_faces);
        all.extend(cyl_faces);
        let shell = topo.add_shell(Shell::new(all).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid, &mut ctx).unwrap();
        assert_eq!(result.actions_taken, 0, "unsupported twins are refused");
        assert!(ctx.reshape.is_empty());
    }

    #[test]
    fn off_circle_and_off_plane_arcs_are_refused() {
        // Arc endpoints off the circle, off-plane circle center, and a
        // transverse circle axis all refuse the face (kept, no removal).
        let mut topo = Topology::new();
        let center = Point3::new(0.0, 0.0, 0.0);
        let good = add_disc(&mut topo, center, 1.0, 0.0);
        // Off-circle vertex: radius 1 + 5*tol at the seam.
        let bad_circle = Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let off_pt = Point3::new(1.0 + 5e-7, 0.0, 0.0);
        let v_off = topo.add_vertex(Vertex::new(off_pt, 1e-7));
        let e_off = topo.add_edge(Edge::new(v_off, v_off, EdgeCurve::Circle(bad_circle)));
        let w_off = topo.add_wire(Wire::new(vec![OrientedEdge::new(e_off, true)], true).unwrap());
        let off_face = topo.add_face(Face::new(
            w_off,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        // Transverse axis: 5° tilt off the face normal.
        let tilt = 5.0_f64.to_radians();
        let tilted_circle =
            Circle3D::new(center, Vec3::new(tilt.sin(), 0.0, tilt.cos()), 1.0).unwrap();
        let tilt_pt = tilted_circle.evaluate(0.0);
        let v_tilt = topo.add_vertex(Vertex::new(tilt_pt, 1e-7));
        let e_tilt = topo.add_edge(Edge::new(v_tilt, v_tilt, EdgeCurve::Circle(tilted_circle)));
        let w_tilt = topo.add_wire(Wire::new(vec![OrientedEdge::new(e_tilt, true)], true).unwrap());
        let tilt_face = topo.add_face(Face::new(
            w_tilt,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![good, off_face, tilt_face]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid, &mut ctx).unwrap();
        assert_eq!(result.actions_taken, 0, "degenerate arcs are refused");
        assert!(ctx.reshape.is_empty());
    }

    // ── M4: removal qualification ──

    use crate::fix::config::{FixConfig, FixMode};

    /// Config with every fixer off except duplicate-face removal.
    fn duplicate_only_config() -> FixConfig {
        FixConfig {
            fix_reorder: FixMode::Off,
            fix_connectivity: FixMode::Off,
            fix_closure: FixMode::Off,
            fix_small_edges: FixMode::Off,
            fix_self_intersection: FixMode::Off,
            fix_degenerate_edges: FixMode::Off,
            fix_gaps_2d: FixMode::Off,
            fix_gaps_3d: FixMode::Off,
            fix_lacking: FixMode::Off,
            fix_notched: FixMode::Off,
            fix_tail: FixMode::Off,
            fix_intersecting_edges: FixMode::Off,
            fix_wire_orientation: FixMode::Off,
            fix_add_natural_bound: FixMode::Off,
            fix_missing_seam: FixMode::Off,
            fix_small_area: FixMode::Off,
            fix_duplicate_faces: FixMode::Auto,
            fix_intersecting_wires: FixMode::Off,
            fix_orientation: FixMode::Off,
            fix_same_parameter: FixMode::Off,
            fix_vertex_tolerance: FixMode::Off,
            fix_pcurve: FixMode::Off,
            fix_coincident_vertices: FixMode::Off,
            fix_wireframe: FixMode::Off,
            fix_split_common_vertex: FixMode::Off,
            fix_small_faces: FixMode::Off,
        }
    }

    fn named(name: &str) -> remus_topology::attributes::EntityAttributes {
        remus_topology::attributes::EntityAttributes {
            name: Some(name.to_string()),
            color: None,
        }
    }

    #[test]
    fn conflicting_attributes_veto_removal() {
        // (survivor attrs, removed attrs, expect removal).
        let cases: [(&str, Option<&str>, Option<&str>, bool); 4] = [
            ("both_bare", None, None, true),
            ("same_name", Some("wall"), Some("wall"), true),
            ("removed_named", None, Some("wall"), false),
            ("conflict", Some("wall"), Some("floor"), false),
        ];
        for (label, survivor_name, removed_name, expect_removal) in cases {
            let mut topo = Topology::new();
            let a = add_triangle(
                &mut topo,
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            );
            let b = add_triangle(
                &mut topo,
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            );
            if let Some(name) = survivor_name {
                topo.set_face_attributes(a, named(name)).unwrap();
            }
            if let Some(name) = removed_name {
                topo.set_face_attributes(b, named(name)).unwrap();
            }
            let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
            let solid = topo.add_solid(Solid::new(shell, vec![]));
            let mut ctx = HealContext::new();
            let result = fix_duplicate_faces(&topo, solid, &mut ctx).unwrap();
            assert_eq!(
                result.actions_taken,
                usize::from(expect_removal),
                "{label}: removal expectation"
            );
            assert_eq!(
                ctx.reshape.is_face_removed(b),
                expect_removal,
                "{label}: reshape expectation"
            );
            if !expect_removal {
                assert!(
                    ctx.messages
                        .iter()
                        .any(|m| m.description.contains("incompatible attributes")),
                    "{label}: veto must be disclosed"
                );
            }
        }
    }

    #[test]
    fn survivor_named_removed_bare_still_removes() {
        // Survivor metadata is never the veto: only the dropped face's
        // attributes gate removal.
        let mut topo = Topology::new();
        let a = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let b = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        topo.set_face_attributes(a, named("wall")).unwrap();
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid, &mut ctx).unwrap();
        assert_eq!(result.actions_taken, 1);
        assert!(ctx.reshape.is_face_removed(b));
        assert_eq!(
            topo.attributes().face(a),
            Some(&named("wall")),
            "survivor keeps its own attributes"
        );
    }

    #[test]
    fn removal_preserves_pcurves_and_history() {
        use remus_math::curves2d::{Curve2D, Line2D};
        use remus_math::vec::{Point2, Vec2};
        use remus_topology::journal::EntityKey;
        use remus_topology::pcurve::PCurve;

        let mut topo = Topology::new();
        let survivor = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let removed = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        // One pcurve per survivor edge (each edge used exactly once).
        let wire_id = topo.face(survivor).unwrap().outer_wire();
        let edge_ids: Vec<_> = topo
            .wire(wire_id)
            .unwrap()
            .edges()
            .iter()
            .map(remus_topology::wire::OrientedEdge::edge)
            .collect();
        assert_eq!(edge_ids.len(), 3);
        for (k, edge_id) in edge_ids.iter().enumerate() {
            let line =
                Line2D::new(Point2::new(f64::from(k as u32), 0.0), Vec2::new(1.0, 0.0)).unwrap();
            topo.set_pcurve(
                *edge_id,
                survivor,
                PCurve::new(Curve2D::Line(line), 0.0, 1.0),
            )
            .unwrap();
        }
        let pcurves_before = topo.num_pcurves();
        assert_eq!(pcurves_before, 3);

        let shell = topo.add_shell(Shell::new(vec![survivor, removed]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        let config = duplicate_only_config();
        let (repaired, result, history) =
            crate::fix::fix_shape_with_history(&mut topo, solid, &config, None).unwrap();
        assert_eq!(result.actions_taken, 1);

        // Surviving topology: shell holds exactly the survivor.
        let faces = topo
            .shell(topo.solid(repaired).unwrap().outer_shell())
            .unwrap()
            .faces()
            .to_vec();
        assert_eq!(faces, vec![survivor]);
        // Survivor pcurves are byte-identical; the pass neither adds nor
        // deletes pcurve entries (removed-face entries become unreferenced
        // arena garbage, like the removed face's own wires and edges).
        assert_eq!(topo.pcurves_for_face(survivor).len(), 3);
        assert_eq!(topo.num_pcurves(), pcurves_before);
        // History records the deletion with no survivor claim (deletion, not
        // substitution); the survivor carries no claim.
        let claims = history.entity_history().unwrap();
        assert_eq!(
            claims.get(&EntityKey::face(removed.index())),
            Some(&Vec::new())
        );
        assert!(!claims.contains_key(&EntityKey::face(survivor.index())));
    }

    #[test]
    fn repair_is_idempotent() {
        let mut topo = Topology::new();
        let a = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let b = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        let config = duplicate_only_config();
        let (repaired, first, _) =
            crate::fix::fix_shape_with_history(&mut topo, solid, &config, None).unwrap();
        assert_eq!(first.actions_taken, 1);
        let (_, second, reshape) =
            crate::fix::fix_shape_with_history(&mut topo, repaired, &config, None).unwrap();
        assert_eq!(second.actions_taken, 0, "second run must be a no-op");
        assert!(!second.status.is_fail());
        assert!(reshape.is_empty());
        let _ = (a, b);
    }

    #[test]
    fn refusal_rolls_back_fully() {
        // Shared-face identity across shells aborts the pass before any
        // removal is recorded: with duplicate-only config nothing else can
        // mutate either, so the topology is bit-identical afterwards.
        let mut topo = Topology::new();
        let mut faces = Vec::new();
        for _ in 0..2 {
            faces.push(add_triangle(
                &mut topo,
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ));
        }
        let outer = topo.add_shell(Shell::new(vec![faces[1]]).unwrap());
        let inner = topo.add_shell(Shell::new(faces).unwrap());
        let solid = topo.add_solid(Solid::new(outer, vec![inner]));
        let before_outer: Vec<usize> = topo
            .shell(outer)
            .unwrap()
            .faces()
            .iter()
            .map(|f| f.index())
            .collect();
        let before_inner: Vec<usize> = topo
            .shell(inner)
            .unwrap()
            .faces()
            .iter()
            .map(|f| f.index())
            .collect();
        let config = duplicate_only_config();
        let error =
            crate::fix::fix_shape_with_history(&mut topo, solid, &config, None).unwrap_err();
        assert!(error.to_string().contains("shared by multiple shells"));
        let after_outer: Vec<usize> = topo
            .shell(outer)
            .unwrap()
            .faces()
            .iter()
            .map(|f| f.index())
            .collect();
        let after_inner: Vec<usize> = topo
            .shell(inner)
            .unwrap()
            .faces()
            .iter()
            .map(|f| f.index())
            .collect();
        assert_eq!(before_outer, after_outer, "outer shell unchanged");
        assert_eq!(before_inner, after_inner, "inner shell unchanged");
    }

    #[test]
    fn nested_hole_inside_hole_region_is_kept() {
        // B's hole sits strictly inside A's hole: B carries more material,
        // so the pair is kept.
        let inner_a: HoleCorners = [
            [1.0, 1.0, 0.0],
            [2.0, 1.0, 0.0],
            [2.0, 2.0, 0.0],
            [1.0, 2.0, 0.0],
        ];
        let inner_b: HoleCorners = [
            [1.2, 1.2, 0.0],
            [1.8, 1.2, 0.0],
            [1.8, 1.8, 0.0],
            [1.2, 1.8, 0.0],
        ];
        let mut topo = Topology::new();
        let a = add_holed_quad(&mut topo, UNIT_OUTER, &[inner_a]);
        let b = add_holed_quad(&mut topo, UNIT_OUTER, &[inner_b]);
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid, &mut ctx).unwrap();
        assert_eq!(result.actions_taken, 0, "nested holes must be kept");
        assert!(ctx.reshape.is_empty());
    }

    #[test]
    fn adjacent_coplanar_tiles_are_kept() {
        // Distinct coincident sheets: same plane, adjacent regions sharing
        // one edge — kept, with no removal recorded.
        let mut topo = Topology::new();
        let left = add_quad(
            &mut topo,
            [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        );
        let right = add_quad(
            &mut topo,
            [
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(2.0, 1.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
            ],
        );
        let shell = topo.add_shell(Shell::new(vec![left, right]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid, &mut ctx).unwrap();
        assert_eq!(result.actions_taken, 0, "adjacent tiles must be kept");
        assert!(ctx.reshape.is_empty());
    }
}
