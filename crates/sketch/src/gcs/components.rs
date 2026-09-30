//! Independent-component decomposition of the sketch constraint graph (PERF-S02).
//!
//! A sketch system with `K` disconnected constraint groups pays the dense
//! solver once per iteration on the full `m × n` Jacobian, even though the
//! matrix is block-diagonal up to permutation. The PERF-S01 baseline measures
//! the prize: `independent_solved_1000` (500 disjoint 2-parameter pairs) spends
//! ~5.9 s per solve on a 1000×1000 dense QR per iteration, where per-component
//! QR would be trivial. This module splits the system into its connected
//! components so every cubic factorization scales with block size, while all
//! linear assembly, reporting and rollback stay global.
//!
//! # Structural rule (not numerical)
//!
//! A constraint depends on the free parameters reachable through the entities
//! it references: directly named points, both endpoints of a named line, the
//! center point plus the radius parameter of a named circle, and the
//! center/start/end points of a named arc. Fixed points contribute no
//! parameter, so two constraints sharing only a fixed point are independent.
//! The mapping is deliberately conservative: it names every parameter the
//! residual formula can read through those entities, whether or not the
//! current Jacobian entry happens to be zero. A degenerate configuration
//! (coincident line endpoints, point at circle center) zeroes analytic
//! gradient entries through the `1e-300` guards in `constraint.rs`, but the
//! structural edge stays: such a system must not silently split and must not
//! silently rejoin when the degeneracy lifts.
//!
//! The match in [`direct_refs`] is exhaustive over [`Constraint`] with no
//! wildcard arm, so adding a constraint variant fails to compile here until
//! its references are declared. That is the whole future-proofing mechanism:
//! a new variant cannot silently escape the graph.
//!
//! # Semantics preserved (and where they deliberately differ)
//!
//! Components are logically separate sketches sharing one arena. Each one is
//! solved by its own DogLeg loop with the caller's `tolerance` and the full
//! `max_iterations` budget, and its own trust region derived from its own
//! parameter norm. This differs from one global loop in three analyzed ways:
//!
//! 1. **Iteration budget.** A shared budget would couple unrelated systems:
//!    whichever component runs first could starve the rest, and the outcome
//!    would depend on solve order. Per-component budgets give every independent
//!    system what a standalone sketch would get. Total factorizations still
//!    collapse from one `n³` to a sum of `n_c³`.
//! 2. **Trust region.** The global initial radius couples unrelated scales
//!    (a component placed at 1e5 offset would inflate the radius of a unit
//!    component and vice versa). Per-component radii are the principled choice
//!    for decoupled systems.
//! 3. **Step interpolation.** The global steepest-descent blend factor and the
//!    accept/reject ratio average over unrelated blocks; per-component loops
//!    decide locally. Trajectories can therefore differ from the dense path,
//!    and rank-deficient blocks can select different free variables than a
//!    global pivoting order would. Agreement is verified within solver
//!    tolerances against the dense path, never claimed bitwise.
//!
//! What is exact: `converged` is the AND over components, `max_residual` is
//! the NaN-propagating max over components (identical to the max over all
//! equations), `num_params`/`num_equations` are sums, and whole-call rollback
//! in `solve_detailed` restores every component when any one fails. Reported
//! `iterations` is the max over components (the parallel critical path), which
//! keeps component results comparable with the dense baseline; evaluation
//! counts are sums (total work).
//!
//! # Rank policy
//!
//! Rank still uses the single global `1e-10·|R00|` policy from `qr.rs`. Each
//! block is factorized independently and its pivots are counted against one
//! absolute threshold derived from the global leading magnitude (the max over
//! blocks, which equals the leading magnitude a global factorization would
//! produce, since Householder reflections never couple zero-separated blocks).
//! Choosing a per-component relative threshold instead would silently re-rank
//! mixed-scale systems and is explicitly rejected; see
//! [`dof::analyze_blocks`](super::dof::analyze_blocks).
//!
//! # Determinism and degenerate groups
//!
//! Components are sorted by smallest global parameter index; parameters inside
//! a component ascend in `param_map` order and constraints stay in arena order,
//! so results reconstruct in the original public order. Union-find unions run
//! in arena order over a fixed node layout, and grouping uses a `BTreeMap`, so
//! repeated decompositions of the same system are identical.
//!
//! Two degenerate groups complete the partition: constraints with no free
//! parameters (everything they touch is fixed — they can only pass or fail a
//! residual check) are merged into one trailing pinned group, and free
//! parameters touched by no constraint are merged into one equation-free
//! group. A system with no equations, or with only fixed geometry, therefore
//! decomposes instead of special-casing at every call site, while `system.rs`
//! keeps its existing degenerate fast paths for the fully trivial shapes.

use std::collections::{BTreeMap, HashMap};

use super::constraint::{Constraint, ConstraintId, EntitySnapshot, residual_count};
use super::entity::{ArcId, CircleId, EllipseId, LineId, ParamRef, PointId};
use super::system::GcsSystem;

/// One independent block of the decomposed system.
///
/// A normal component carries both parameters and constraints. The two
/// degenerate shapes are a free group (parameters, no constraints: nothing to
/// solve, each parameter keeps one degree of freedom) and a pinned group
/// (constraints, no parameters: a pure residual check over fixed geometry).
#[derive(Debug, Clone)]
pub struct Component {
    /// Global parameter indices in ascending `param_map` order.
    pub params: Vec<usize>,
    /// Constraint ids in arena order.
    pub constraint_ids: Vec<ConstraintId>,
    /// Cloned constraints aligned with [`Self::constraint_ids`].
    pub constraints: Vec<Constraint>,
    /// Residual row counts aligned with [`Self::constraint_ids`].
    pub row_counts: Vec<usize>,
}

impl Component {
    /// Number of residual equations in this component.
    pub fn num_equations(&self) -> usize {
        self.row_counts.iter().sum()
    }

    /// Whether this component carries no constraints (free parameters only).
    pub fn is_free(&self) -> bool {
        self.constraint_ids.is_empty()
    }

    /// Whether this component carries no parameters (fixed geometry only).
    pub fn is_pinned(&self) -> bool {
        self.params.is_empty() && !self.constraint_ids.is_empty()
    }
}

/// The full partition of a system into independent components.
#[derive(Debug, Clone)]
pub struct Decomposition {
    /// Normal components sorted by smallest parameter index, followed by the
    /// free group (if any) and then the pinned group (if any).
    pub components: Vec<Component>,
    /// Total free parameters (sum over components).
    pub num_params: usize,
    /// Total residual equations (sum over components).
    pub num_equations: usize,
}

impl Decomposition {
    /// Whether the system is fully connected with no degenerate groups, in
    /// which case the caller should use the existing dense path bit-for-bit.
    #[must_use]
    pub fn is_single_connected(&self) -> bool {
        self.components.len() == 1
            && !self.components[0].params.is_empty()
            && !self.components[0].constraint_ids.is_empty()
    }
}

/// Direct structural references named by a constraint.
///
/// Each entry carries coordinate precision: a point reference names whether
/// its X, Y, or both coordinates feed the residual formula, and likewise for
/// line endpoints (both endpoints share the mask) and circle centers. Radii
/// and arcs are named separately — a circle reference may use its center, its
/// radius, or both, while an arc reference always names the whole
/// center/start/end triple (the arc's internal tie couples the triple through
/// every live arc, so finer arc splitting could never separate anything).
/// An ellipse reference names the center point and/or the scalar parameters
/// (`a`, `b`, `phi`) per variant, following the same precision rule as
/// circles: `ConcentricEllipseCircle` names the circle center but not its
/// radius, and the ellipse center but not its scalars, because those
/// residuals never read them. Unlike arcs — whose internal tie couples the
/// center/start/end triple through every live arc — ellipses carry no
/// internal constraint, so a center-only reference is exact and keeps a
/// center-coincidence system independent from axis-driving constraints on
/// the same ellipse. Variants whose residual reads the implicit form
/// (`PointOnEllipse`, `TangentLineEllipse`) name the full center-plus-scalars
/// set, which re-joins everything the formula couples.
///
/// The contract per variant is the union of parameters appearing in any of its
/// residual rows — never a numerical observation. `FixX` names only X,
/// `Horizontal` only the endpoints' Y, `CircleRadius` only the radius: those
/// are formula facts, true at every iterate, including degenerate ones where
/// unrelated Jacobian entries vanish through the `1e-300` guards. Conversely
/// a variant keeps every parameter any row can read, even when the current
/// values zero its gradient.
///
/// There is intentionally no wildcard arm: a new [`Constraint`] variant fails
/// to compile here until its references are declared, which keeps future
/// variants from silently escaping the graph.
struct StructuralRefs {
    /// `(point, use_x, use_y)` for directly named points.
    points: Vec<(PointId, bool, bool)>,
    /// `(line, use_x, use_y)` applied to both endpoints.
    lines: Vec<(LineId, bool, bool)>,
    /// `(circle, use_x, use_y)` applied to the center point.
    circle_centers: Vec<(CircleId, bool, bool)>,
    /// Circles whose radius parameter the formula reads.
    circle_radii: Vec<CircleId>,
    /// Arcs, always expanded to the full center/start/end triple.
    arcs: Vec<ArcId>,
    /// `(ellipse, use_center, use_a, use_b, use_phi)`.
    ellipses: Vec<(EllipseId, bool, bool, bool, bool)>,
}

/// Direct entity handles named by a constraint (exhaustive over variants).
fn direct_refs(c: &Constraint) -> StructuralRefs {
    let mut refs = StructuralRefs {
        points: Vec::new(),
        lines: Vec::new(),
        circle_centers: Vec::new(),
        circle_radii: Vec::new(),
        arcs: Vec::new(),
        ellipses: Vec::new(),
    };
    match c {
        Constraint::Coincident(p1, p2) => {
            refs.points.push((*p1, true, true));
            refs.points.push((*p2, true, true));
        }
        Constraint::Distance(p1, p2, _) => {
            refs.points.push((*p1, true, true));
            refs.points.push((*p2, true, true));
        }
        Constraint::PointLineDistance(pt, line, _) => {
            refs.points.push((*pt, true, true));
            refs.lines.push((*line, true, true));
        }
        Constraint::FixX(p, _) => {
            refs.points.push((*p, true, false));
        }
        Constraint::FixY(p, _) => {
            refs.points.push((*p, false, true));
        }
        Constraint::Horizontal(line) => {
            refs.lines.push((*line, false, true));
        }
        Constraint::Vertical(line) => {
            refs.lines.push((*line, true, false));
        }
        Constraint::Angle(l1, l2, _)
        | Constraint::Perpendicular(l1, l2)
        | Constraint::Parallel(l1, l2)
        | Constraint::EqualLength(l1, l2) => {
            refs.lines.push((*l1, true, true));
            refs.lines.push((*l2, true, true));
        }
        Constraint::PointOnCircle(pt, circ) => {
            refs.points.push((*pt, true, true));
            refs.circle_centers.push((*circ, true, true));
            refs.circle_radii.push(*circ);
        }
        Constraint::PointOnArc(pt, arc) => {
            // Reads the point, the center and the start radius only; the end
            // point is named through the arc triple below (and, in any live
            // system, through the arc's own internal tie).
            refs.points.push((*pt, true, true));
            refs.arcs.push(*arc);
        }
        Constraint::TangentLineArc(line, arc, shared) => {
            refs.lines.push((*line, true, true));
            refs.arcs.push(*arc);
            refs.points.push((*shared, true, true));
        }
        Constraint::TangentArcArc(a1, a2, shared) => {
            refs.arcs.push(*a1);
            refs.arcs.push(*a2);
            refs.points.push((*shared, true, true));
        }
        Constraint::EqualRadiusArcArc(a1, a2) | Constraint::ConcentricArcArc(a1, a2) => {
            refs.arcs.push(*a1);
            refs.arcs.push(*a2);
        }
        Constraint::EqualRadiusArcCircle(arc, circ) => {
            refs.arcs.push(*arc);
            refs.circle_radii.push(*circ);
        }
        Constraint::ConcentricArcCircle(arc, circ) => {
            // Center-only on the circle side: the residual never reads the
            // radius, so a radius constraint on the same circle stays separate.
            refs.arcs.push(*arc);
            refs.circle_centers.push((*circ, true, true));
        }
        Constraint::ArcLength(arc, _) => {
            refs.arcs.push(*arc);
        }
        Constraint::CircleRadius(circ, _) => {
            refs.circle_radii.push(*circ);
        }
        Constraint::EqualRadiusCircleCircle(c1, c2) => {
            refs.circle_radii.push(*c1);
            refs.circle_radii.push(*c2);
        }
        Constraint::Midpoint(pt, line) => {
            refs.points.push((*pt, true, true));
            refs.lines.push((*line, true, true));
        }
        Constraint::Symmetric(p1, p2, axis) => {
            refs.points.push((*p1, true, true));
            refs.points.push((*p2, true, true));
            refs.lines.push((*axis, true, true));
        }
        Constraint::TangentLineCircle(line, circ) => {
            refs.lines.push((*line, true, true));
            refs.circle_centers.push((*circ, true, true));
            refs.circle_radii.push(*circ);
        }
        Constraint::SymmetricAboutPoint(p1, p2, center) => {
            // Rows separate by coordinate, but a constraint's rows always stay
            // in one component, so the union names all six coordinates.
            refs.points.push((*p1, true, true));
            refs.points.push((*p2, true, true));
            refs.points.push((*center, true, true));
        }
        Constraint::PointOnEllipse(pt, ell) => {
            refs.points.push((*pt, true, true));
            refs.ellipses.push((*ell, true, true, true, true));
        }
        Constraint::ConcentricEllipseEllipse(e1, e2) => {
            // Center-only on both sides: the residuals never read the axes
            // or the orientation.
            refs.ellipses.push((*e1, true, false, false, false));
            refs.ellipses.push((*e2, true, false, false, false));
        }
        Constraint::ConcentricEllipseCircle(ell, circ) => {
            refs.ellipses.push((*ell, true, false, false, false));
            refs.circle_centers.push((*circ, true, true));
        }
        Constraint::ConcentricEllipseArc(ell, arc) => {
            refs.ellipses.push((*ell, true, false, false, false));
            refs.arcs.push(*arc);
        }
        Constraint::TangentLineEllipse(line, ell, contact) => {
            refs.lines.push((*line, true, true));
            refs.ellipses.push((*ell, true, true, true, true));
            refs.points.push((*contact, true, true));
        }
        Constraint::EllipseAxisA(ell, _) => {
            refs.ellipses.push((*ell, false, true, false, false));
        }
        Constraint::EllipseAxisB(ell, _) => {
            refs.ellipses.push((*ell, false, false, true, false));
        }
        Constraint::EllipseAngle(ell, _) => {
            refs.ellipses.push((*ell, false, false, false, true));
        }
        Constraint::EqualEllipseRadii(e1, e2) => {
            refs.ellipses.push((*e1, false, true, true, false));
            refs.ellipses.push((*e2, false, true, true, false));
        }
    }
    refs
}

/// Free-parameter indices a constraint structurally depends on, ascending.
///
/// Expands [`direct_refs`] through the live system with its coordinate masks:
/// a point contributes its X and/or Y coordinates when those coordinates
/// occur in the solver's parameter map, every referenced circle contributes
/// its radius parameter when named, and arcs expand to their full
/// center/start/end triples. A fixed point absent from the map contributes
/// nothing, so constraints sharing only fixed geometry stay independent.
/// Missing entities are skipped defensively — removal is refused while
/// referenced, so this is unreachable in production, and the residual path
/// poisons to NaN there rather than solving something wrong.
pub fn constraint_param_indices(
    c: &Constraint,
    sys: &GcsSystem,
    param_index: &HashMap<ParamRef, usize>,
) -> Vec<usize> {
    let refs = direct_refs(c);
    let mut params: Vec<ParamRef> = Vec::new();

    let push_point = |id: PointId, use_x: bool, use_y: bool, params: &mut Vec<ParamRef>| {
        if use_x {
            params.push(ParamRef::PointX(id));
        }
        if use_y {
            params.push(ParamRef::PointY(id));
        }
    };

    for (id, use_x, use_y) in refs.points {
        push_point(id, use_x, use_y, &mut params);
    }
    for (id, use_x, use_y) in refs.lines {
        if let Some(line) = sys.line(id) {
            push_point(line.p1, use_x, use_y, &mut params);
            push_point(line.p2, use_x, use_y, &mut params);
        }
    }
    for (id, use_x, use_y) in refs.circle_centers {
        if let Some(circle) = sys.circle(id) {
            push_point(circle.center, use_x, use_y, &mut params);
        }
    }
    for id in refs.circle_radii {
        if sys.circle(id).is_some() {
            params.push(ParamRef::CircleRadius(id));
        }
    }
    for id in refs.arcs {
        if let Some(arc) = sys.arc(id) {
            push_point(arc.center, true, true, &mut params);
            push_point(arc.start, true, true, &mut params);
            push_point(arc.end, true, true, &mut params);
        }
    }
    for (id, use_center, use_a, use_b, use_phi) in refs.ellipses {
        if let Some(ellipse) = sys.ellipse(id) {
            if use_center {
                push_point(ellipse.center, true, true, &mut params);
            }
            if use_a {
                params.push(ParamRef::EllipseA(id));
            }
            if use_b {
                params.push(ParamRef::EllipseB(id));
            }
            if use_phi {
                params.push(ParamRef::EllipsePhi(id));
            }
        }
    }

    let mut indices: Vec<usize> = params
        .iter()
        .filter_map(|r| param_index.get(r).copied())
        .collect();
    indices.sort_unstable();
    indices.dedup();
    indices
}

/// Entity subset backing one component's solve-local snapshots.
///
/// Contains exactly the entities the component's constraints can read,
/// including fixed points at their base values. Residual and Jacobian
/// evaluation over this subset agrees with full-snapshot evaluation because no
/// constraint in the component names anything outside it; stale-handle
/// poisoning is preserved (a referenced-but-removed entity is absent here just
/// as its lookup would poison there — and removal while referenced is refused,
/// so this stays unreachable in production).
pub struct ComponentEntities {
    /// `(point, base x, base y)` for every point the component can read.
    pub points: Vec<(PointId, f64, f64)>,
    /// `(line, endpoint pair)` for every line the component can read.
    pub lines: Vec<(LineId, (PointId, PointId))>,
    /// `(circle, (center, radius))` for every circle the component can read.
    pub circles: Vec<(CircleId, (PointId, f64))>,
    /// `(arc, (center, start, end))` for every arc the component can read.
    pub arcs: Vec<(ArcId, (PointId, PointId, PointId))>,
    /// `(ellipse, (center, a, b, phi))` for every ellipse the component reads.
    pub ellipses: Vec<(EllipseId, (PointId, f64, f64, f64))>,
}

/// Collect the entity subset for a constraint list, in arena order.
///
/// The subset covers every entity the component's constraints can read: named
/// points at both coordinates, both endpoints of named lines, circle centers
/// and radii, full arc triples, and ellipse centers plus their scalar
/// parameters. Masks are widened to full entities here —
/// snapshots are keyed by entity, and a point's fixed coordinates ride along
/// at base value either way.
pub fn component_entities(constraints: &[Constraint], sys: &GcsSystem) -> ComponentEntities {
    let refs_list: Vec<StructuralRefs> = constraints.iter().map(direct_refs).collect();
    let mut points: Vec<PointId> = Vec::new();
    let mut lines: Vec<LineId> = Vec::new();
    let mut circles: Vec<CircleId> = Vec::new();
    let mut arcs: Vec<ArcId> = Vec::new();
    let mut ellipses: Vec<EllipseId> = Vec::new();
    for refs in &refs_list {
        points.extend(refs.points.iter().map(|(id, _, _)| *id));
        lines.extend(refs.lines.iter().map(|(id, _, _)| *id));
        circles.extend(refs.circle_centers.iter().map(|(id, _, _)| *id));
        circles.extend(refs.circle_radii.iter().copied());
        arcs.extend(refs.arcs.iter().copied());
        ellipses.extend(refs.ellipses.iter().map(|(id, _, _, _, _)| *id));
    }
    // Snapshots are keyed by entity: expand structural references to the
    // points they read through (line endpoints, circle centers, arc triples).
    // Without this a line-only constraint would evaluate against absent
    // points and poison to NaN.
    for id in &lines {
        if let Some(line) = sys.line(*id) {
            points.push(line.p1);
            points.push(line.p2);
        }
    }
    for id in &circles {
        if let Some(circle) = sys.circle(*id) {
            points.push(circle.center);
        }
    }
    for id in &arcs {
        if let Some(arc) = sys.arc(*id) {
            points.push(arc.center);
            points.push(arc.start);
            points.push(arc.end);
        }
    }
    for id in &ellipses {
        if let Some(ellipse) = sys.ellipse(*id) {
            points.push(ellipse.center);
        }
    }
    points.sort_by_key(|id: &PointId| id.index());
    points.dedup();
    lines.sort_by_key(|id: &LineId| id.index());
    lines.dedup();
    circles.sort_by_key(|id: &CircleId| id.index());
    circles.dedup();
    arcs.sort_by_key(|id: &ArcId| id.index());
    arcs.dedup();
    ellipses.sort_by_key(|id: &EllipseId| id.index());
    ellipses.dedup();

    let point_bases: Vec<(PointId, f64, f64)> = points
        .into_iter()
        .filter_map(|id| sys.point(id).map(|p| (id, p.x, p.y)))
        .collect();
    let line_defs: Vec<(LineId, (PointId, PointId))> = lines
        .into_iter()
        .filter_map(|id| sys.line(id).map(|l| (id, (l.p1, l.p2))))
        .collect();
    let circle_defs: Vec<(CircleId, (PointId, f64))> = circles
        .into_iter()
        .filter_map(|id| sys.circle(id).map(|c| (id, (c.center, c.radius))))
        .collect();
    let arc_defs: Vec<(ArcId, (PointId, PointId, PointId))> = arcs
        .into_iter()
        .filter_map(|id| sys.arc(id).map(|a| (id, (a.center, a.start, a.end))))
        .collect();
    let ellipse_defs: Vec<(EllipseId, (PointId, f64, f64, f64))> = ellipses
        .into_iter()
        .filter_map(|id| sys.ellipse(id).map(|e| (id, (e.center, e.a, e.b, e.angle))))
        .collect();
    ComponentEntities {
        points: point_bases,
        lines: line_defs,
        circles: circle_defs,
        arcs: arc_defs,
        ellipses: ellipse_defs,
    }
}

/// Build a snapshot of exactly the component's entities at base values.
///
/// Structure entries (lines, circles, arcs, ellipses) are static for the
/// solve; point coordinates and scalar parameters are refreshed per
/// evaluation by [`refresh_subset_snapshot`], [`refresh_subset_radii`], and
/// [`refresh_subset_ellipses`].
pub fn subset_snapshot(entities: &ComponentEntities) -> EntitySnapshot {
    EntitySnapshot {
        points: entities
            .points
            .iter()
            .map(|(id, x, y)| (*id, (*x, *y)))
            .collect(),
        lines: entities
            .lines
            .iter()
            .map(|(id, ends)| (*id, *ends))
            .collect(),
        circles: entities
            .circles
            .iter()
            .map(|(id, def)| (*id, *def))
            .collect(),
        arcs: entities.arcs.iter().map(|(id, def)| (*id, *def)).collect(),
        ellipses: entities
            .ellipses
            .iter()
            .map(|(id, def)| (*id, *def))
            .collect(),
    }
}

/// Refresh a subset snapshot's points from component-local parameters.
///
/// `bases` carries every point's base value; coordinates whose parameter is in
/// `local_index` are overwritten from `local`, the rest (fixed points) keep
/// their base. Only the component's own points are visited, so per-evaluation
/// work scales with block size rather than document size.
pub fn refresh_subset_snapshot(
    snap: &mut EntitySnapshot,
    bases: &[(PointId, f64, f64)],
    local: &[f64],
    local_index: &HashMap<ParamRef, usize>,
) {
    snap.points.clear();
    for (id, bx, by) in bases {
        let x = local_index
            .get(&ParamRef::PointX(*id))
            .map_or(*bx, |&i| local[i]);
        let y = local_index
            .get(&ParamRef::PointY(*id))
            .map_or(*by, |&i| local[i]);
        snap.points.insert(*id, (x, y));
    }
}

/// Refresh a subset snapshot's circle radii from component-local parameters.
///
/// Point coordinates are handled by [`refresh_subset_snapshot`]; this covers
/// the radius half of circle definitions, which that function leaves at base.
pub fn refresh_subset_radii(
    snap: &mut EntitySnapshot,
    circles: &[(CircleId, (PointId, f64))],
    local: &[f64],
    local_index: &HashMap<ParamRef, usize>,
) {
    snap.circles.clear();
    for (id, (center, base_r)) in circles {
        let r = local_index
            .get(&ParamRef::CircleRadius(*id))
            .map_or(*base_r, |&i| local[i]);
        snap.circles.insert(*id, (*center, r));
    }
}

/// Refresh a subset snapshot's ellipse scalars from component-local parameters.
///
/// Covers the `(a, b, phi)` half of ellipse definitions; the center point
/// rides with [`refresh_subset_snapshot`] at its base unless free.
#[allow(clippy::type_complexity)]
pub fn refresh_subset_ellipses(
    snap: &mut EntitySnapshot,
    ellipses: &[(EllipseId, (PointId, f64, f64, f64))],
    local: &[f64],
    local_index: &HashMap<ParamRef, usize>,
) {
    snap.ellipses.clear();
    for (id, (center, base_a, base_b, base_phi)) in ellipses {
        let a = local_index
            .get(&ParamRef::EllipseA(*id))
            .map_or(*base_a, |&i| local[i]);
        let b = local_index
            .get(&ParamRef::EllipseB(*id))
            .map_or(*base_b, |&i| local[i]);
        let phi = local_index
            .get(&ParamRef::EllipsePhi(*id))
            .map_or(*base_phi, |&i| local[i]);
        snap.ellipses.insert(*id, (*center, a, b, phi));
    }
}

/// Disjoint-set union over a fixed node layout, path-halved iteratively.
struct UnionFind {
    parent: Vec<usize>,
    rank: Vec<u8>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
            rank: vec![0; n],
        }
    }

    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]];
            x = self.parent[x];
        }
        x
    }

    fn union(&mut self, a: usize, b: usize) {
        let ra = self.find(a);
        let rb = self.find(b);
        if ra == rb {
            return;
        }
        if self.rank[ra] < self.rank[rb] {
            self.parent[ra] = rb;
        } else {
            self.parent[rb] = ra;
            if self.rank[ra] == self.rank[rb] {
                self.rank[ra] = self.rank[ra].saturating_add(1);
            }
        }
    }
}

/// Split the live system into its independent components.
///
/// Rebuilds a stale parameter map first, so the partition always describes the
/// current arena state; callers must not retain a decomposition across edits.
/// Unions run in arena order over the layout `[params 0..n, constraints n..n+k]`
/// and groups assemble through a `BTreeMap`, keeping the whole partition
/// deterministic for a given add/remove history.
pub fn decompose(sys: &mut GcsSystem) -> Decomposition {
    sys.rebuild_if_dirty();
    let param_map = sys.param_map_slice().to_vec();
    let param_index = sys.param_index_map().clone();
    let entries = sys.ordered_constraints();

    let n = param_map.len();
    let k = entries.len();
    let mut uf = UnionFind::new(n.saturating_add(k));
    for (j, (_, c)) in entries.iter().enumerate() {
        for p in constraint_param_indices(c, sys, &param_index) {
            uf.union(n.saturating_add(j), p);
        }
    }

    // Group nodes by root. Params ascend, constraints in arena order, roots via
    // BTreeMap: every ordering choice here is deterministic.
    let mut groups: BTreeMap<usize, (Vec<usize>, Vec<usize>)> = BTreeMap::new();
    for (i, _) in param_map.iter().enumerate() {
        let root = uf.find(i);
        groups.entry(root).or_default().0.push(i);
    }
    for (j, _) in entries.iter().enumerate() {
        let root = uf.find(n.saturating_add(j));
        groups.entry(root).or_default().1.push(j);
    }

    let mut normals: Vec<Component> = Vec::new();
    let mut free_params: Vec<usize> = Vec::new();
    let mut pinned: Vec<usize> = Vec::new();
    for (_, (params, cidx)) in groups {
        if cidx.is_empty() {
            free_params.extend(params);
        } else if params.is_empty() {
            pinned.extend(cidx);
        } else {
            normals.push(make_component(params, cidx, &entries));
        }
    }

    normals.sort_by_key(|c| c.params[0]);
    free_params.sort_unstable();
    pinned.sort_unstable();

    let mut components = normals;
    if !free_params.is_empty() {
        components.push(Component {
            params: free_params,
            constraint_ids: Vec::new(),
            constraints: Vec::new(),
            row_counts: Vec::new(),
        });
    }
    if !pinned.is_empty() {
        let all_ids: Vec<ConstraintId> = pinned.iter().map(|&j| entries[j].0).collect();
        let all_cs: Vec<Constraint> = pinned.iter().map(|&j| entries[j].1.clone()).collect();
        let all_rows: Vec<usize> = pinned
            .iter()
            .map(|&j| residual_count(&entries[j].1))
            .collect();
        components.push(Component {
            params: Vec::new(),
            constraint_ids: all_ids,
            constraints: all_cs,
            row_counts: all_rows,
        });
    }

    let num_equations = entries.iter().map(|(_, c)| residual_count(c)).sum();
    Decomposition {
        components,
        num_params: n,
        num_equations,
    }
}

/// Assemble one normal component from ascending param indices and arena-order
/// constraint positions.
fn make_component(
    params: Vec<usize>,
    cidx: Vec<usize>,
    entries: &[(ConstraintId, Constraint)],
) -> Component {
    let constraint_ids = cidx.iter().map(|&j| entries[j].0).collect();
    let constraints = cidx.iter().map(|&j| entries[j].1.clone()).collect();
    let row_counts = cidx
        .iter()
        .map(|&j| residual_count(&entries[j].1))
        .collect();
    Component {
        params,
        constraint_ids,
        constraints,
        row_counts,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests;
