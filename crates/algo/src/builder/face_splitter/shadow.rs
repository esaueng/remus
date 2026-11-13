//! Shadow differential between the legacy special-case emitters and the
//! UV-arrangement adapters (Open Kernel O2.3d, step 0).
//!
//! When a legacy emitter's result is adopted by `split_face_2d_impl`, the
//! call site hands the same input to the arrangement adapter that is meant
//! to replace it, compares the two partitions, and records the outcome. The
//! legacy result is always the one returned: this module only measures.
//!
//! The module exists in test and `perf-counters` builds only, and even there
//! it is inert until a run opts in, so the workspace suites and the scaling
//! guards (which also build with `perf-counters`) do no shadow work:
//!
//! - `REMUS_SPLITTER_SHADOW=default` runs the adapter under the operation's
//!   own work budgets, and `REMUS_SPLITTER_SHADOW=raised` under
//!   [`RAISED_BUDGETS`];
//! - `REMUS_SPLITTER_SHADOW_LOG=<path>` appends one tab-separated line per
//!   shadow run (see [`log_event`]), which
//!   `scripts/splitter-shadow-census.py` aggregates across test binaries;
//! - this crate's own tests opt in per thread with `with_mode`.
//!
//! The adapters read the topology immutably and allocate none. The shadow
//! passes them no split registry, so a shadow run leaves no trace in shared
//! state; its counts go to the open [`crate::gfa::SectionDropScope`]s.
//!
//! Agreement is strict: the same number of regions, and for each region a
//! partner whose wires carry the same ordered carrier kinds up to rotation,
//! every vertex within the linear tolerance, and every curved edge on the
//! same native carrier over the same signed sub-span. A partition that
//! differs only in where it breaks a carrier (an arc split at a cardinal
//! angle, a lens arc split at its midpoint) is still a disagreement; it is
//! labelled `segmentation-only` so the census can tell it from a different
//! partition.

use std::f64::consts::TAU;

use remus_math::context::{OperationContext, WorkBudgets};
use remus_math::tolerance::Tolerance;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

use super::super::plane_frame::PlaneFrame;
use super::super::split_types::{OrientedPCurveEdge, SectionEdge, SplitSubFace};
use super::{arrangement_cyl, arrangement_prod};
use crate::ds::Rank;
use crate::error::AlgoError;
use crate::gfa::{SplitterShadowEntry, SplitterShadowOutcome, note_splitter_shadow};

pub(super) mod work;

/// Environment variable selecting the shadow budgets (`default` or `raised`).
const MODE_VAR: &str = "REMUS_SPLITTER_SHADOW";
/// Environment variable naming the file shadow events are appended to.
const LOG_VAR: &str = "REMUS_SPLITTER_SHADOW_LOG";

/// Work budgets of a `raised` shadow run: the budgets the adapters' own
/// qualification tests run under (`arrangement_prod::tests::test_context`).
const RAISED_BUDGETS: WorkBudgets = WorkBudgets::new()
    .with_march_steps(2_000_000)
    .with_queue_size(20_000)
    .with_segments(10_000);

/// Which work budgets a shadow run gives the arrangement adapter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ShadowBudgets {
    /// The budgets of the operation being shadowed.
    Inherit,
    /// [`RAISED_BUDGETS`].
    Raised,
}

impl ShadowBudgets {
    const fn label(self) -> &'static str {
        match self {
            Self::Inherit => "default",
            Self::Raised => "raised",
        }
    }
}

/// A thread's say over the differential.
#[cfg(test)]
#[derive(Clone, Copy)]
enum Override {
    /// The environment decides.
    Environment,
    /// Forced off or on, whatever the environment says.
    Forced(Option<ShadowBudgets>),
}

#[cfg(test)]
thread_local! {
    /// This thread's opt-in, taking precedence over the environment.
    static OVERRIDE: std::cell::Cell<Override> =
        const { std::cell::Cell::new(Override::Environment) };
    /// The reason of every shadow run on this thread since [`take_reasons`].
    static REASONS: std::cell::RefCell<Vec<String>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

/// The reasons recorded on this thread since the previous call, in order.
#[cfg(test)]
fn take_reasons() -> Vec<String> {
    REASONS.with(std::cell::RefCell::take)
}

/// Run `f` with the shadow differential set to `mode` on this thread
/// (`None` forces it off whatever the environment says).
#[cfg(test)]
fn with_mode<R>(mode: Option<ShadowBudgets>, f: impl FnOnce() -> R) -> R {
    struct Restore(Override);
    impl Drop for Restore {
        fn drop(&mut self) {
            OVERRIDE.with(|cell| cell.set(self.0));
        }
    }
    let _restore = Restore(OVERRIDE.with(|cell| cell.replace(Override::Forced(mode))));
    f()
}

fn env_mode() -> Option<ShadowBudgets> {
    static MODE: std::sync::OnceLock<Option<ShadowBudgets>> = std::sync::OnceLock::new();
    *MODE.get_or_init(|| match std::env::var(MODE_VAR).ok()?.as_str() {
        "default" => Some(ShadowBudgets::Inherit),
        "raised" => Some(ShadowBudgets::Raised),
        _ => None,
    })
}

fn mode() -> Option<ShadowBudgets> {
    #[cfg(test)]
    if let Override::Forced(mode) = OVERRIDE.with(std::cell::Cell::get) {
        return mode;
    }
    env_mode()
}

/// The context of one shadow run.
///
/// Both adapters turn budget exhaustion into a silent decline only when the
/// budgets equal [`WorkBudgets::new`], and into a typed
/// `WorkBudgetExceeded` error otherwise. The shadow needs the typed form to
/// tell a budget decline from an out-of-domain one, so default budgets are
/// made unequal in `blend_walk_steps`, a field no arrangement code reads.
/// The caps the arrangement answers to are unchanged.
fn shadow_context(context: &OperationContext, budgets: ShadowBudgets) -> OperationContext {
    let budgets = match budgets {
        ShadowBudgets::Raised => RAISED_BUDGETS,
        ShadowBudgets::Inherit if context.budgets == WorkBudgets::new() => context
            .budgets
            .with_blend_walk_steps(context.budgets.blend_walk_steps.saturating_sub(1)),
        ShadowBudgets::Inherit => context.budgets,
    };
    context.clone().with_budgets(budgets)
}

/// Shadow a legacy plane emitter with the planar arrangement adapter.
#[allow(clippy::too_many_arguments)]
pub(super) fn plane(
    entry: SplitterShadowEntry,
    topo: &Topology,
    face_id: FaceId,
    sections: &[SectionEdge],
    rank: Rank,
    frame: &PlaneFrame,
    tol: &Tolerance,
    context: &OperationContext,
    legacy: &[SplitSubFace],
    legacy_rerun: &dyn Fn(),
) {
    work::probe(
        work::Site::Legacy(entry.name()),
        &work::Probe {
            topo,
            face_id,
            sections,
            rank,
            tol,
            context,
            adapter: work::Adapter::Plane(frame),
        },
        "plane",
        Some((legacy_rerun, legacy.len())),
    );
    let Some(budgets) = mode() else {
        return;
    };
    let context = shadow_context(context, budgets);
    let shadow = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        arrangement_prod::try_split_plane_face_by_provenance_arrangement(
            topo, face_id, sections, rank, frame, tol, &context, None,
        )
    }));
    record(
        entry, budgets, face_id, "plane", sections, legacy, shadow, tol,
    );
}

/// Shadow a legacy cylinder emitter with the cylinder arrangement adapter.
/// Faces on any other surface are not counted: the adapter owns cylinders
/// only, so they are outside the family this entry point is measured on.
#[allow(clippy::too_many_arguments)]
pub(super) fn cylinder(
    entry: SplitterShadowEntry,
    topo: &Topology,
    face_id: FaceId,
    sections: &[SectionEdge],
    rank: Rank,
    tol: &Tolerance,
    context: &OperationContext,
    legacy: &[SplitSubFace],
    legacy_rerun: &dyn Fn(),
) {
    if !topo
        .face(face_id)
        .is_ok_and(|face| matches!(face.surface(), FaceSurface::Cylinder(_)))
    {
        return;
    }
    work::probe(
        work::Site::Legacy(entry.name()),
        &work::Probe {
            topo,
            face_id,
            sections,
            rank,
            tol,
            context,
            adapter: work::Adapter::Cylinder,
        },
        "cylinder",
        Some((legacy_rerun, legacy.len())),
    );
    let Some(budgets) = mode() else {
        return;
    };
    let context = shadow_context(context, budgets);
    let shadow = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        arrangement_cyl::try_split_cylinder_face_by_provenance_arrangement(
            topo, face_id, sections, rank, tol, &context, None,
        )
    }));
    record(
        entry, budgets, face_id, "cylinder", sections, legacy, shadow, tol,
    );
}

type ShadowResult = std::thread::Result<Result<Option<Vec<SplitSubFace>>, AlgoError>>;

#[allow(clippy::too_many_arguments)]
fn record(
    entry: SplitterShadowEntry,
    budgets: ShadowBudgets,
    face_id: FaceId,
    surface: &str,
    sections: &[SectionEdge],
    legacy: &[SplitSubFace],
    shadow: ShadowResult,
    tol: &Tolerance,
) {
    let (outcome, reason, shadow_regions) = match shadow {
        Err(_) => (SplitterShadowOutcome::Refused, "panic".to_owned(), None),
        Ok(Err(error)) => {
            let text = format!("{error:?}");
            if text.contains("WorkBudgetExceeded") {
                (
                    SplitterShadowOutcome::DeclinedOnBudget,
                    "work-budget".to_owned(),
                    None,
                )
            } else {
                (SplitterShadowOutcome::Refused, text, None)
            }
        }
        Ok(Ok(None)) => (
            SplitterShadowOutcome::Declined,
            "out-of-domain".to_owned(),
            None,
        ),
        Ok(Ok(Some(parts))) => {
            let (outcome, reason) = match compare(legacy, &parts, tol.linear) {
                Verdict::Agreed => (SplitterShadowOutcome::Agreed, "-".to_owned()),
                Verdict::SegmentationOnly(strict) => (
                    SplitterShadowOutcome::Disagreed,
                    format!("segmentation-only({})", strict.label()),
                ),
                Verdict::Different(mismatch) => (
                    SplitterShadowOutcome::Disagreed,
                    mismatch.label().to_owned(),
                ),
            };
            (outcome, reason, Some(parts.len()))
        }
    };
    note_splitter_shadow(entry, outcome);
    #[cfg(test)]
    REASONS.with(|reasons| reasons.borrow_mut().push(reason.clone()));
    log_event(&Event {
        entry,
        outcome,
        reason: &reason,
        budgets,
        face_id,
        surface,
        sections,
        legacy_regions: legacy.len(),
        shadow_regions,
    });
}

struct Event<'a> {
    entry: SplitterShadowEntry,
    outcome: SplitterShadowOutcome,
    reason: &'a str,
    budgets: ShadowBudgets,
    face_id: FaceId,
    surface: &'a str,
    sections: &'a [SectionEdge],
    legacy_regions: usize,
    shadow_regions: Option<usize>,
}

const fn outcome_label(outcome: SplitterShadowOutcome) -> &'static str {
    match outcome {
        SplitterShadowOutcome::Agreed => "agreed",
        SplitterShadowOutcome::Disagreed => "disagreed",
        SplitterShadowOutcome::Declined => "declined",
        SplitterShadowOutcome::DeclinedOnBudget => "declined-on-budget",
        SplitterShadowOutcome::Refused => "refused",
    }
}

/// Append one event to the file named by `REMUS_SPLITTER_SHADOW_LOG`.
///
/// Columns, tab-separated: budgets, entry point, outcome, reason, test
/// thread name (the foil), face index in the operation's shape store,
/// surface, section count, legacy region count, arrangement region count
/// (`-` when it produced none), and the first section's start point as a
/// model-space locator for the face.
fn log_event(event: &Event<'_>) {
    use std::io::Write as _;
    static SINK: std::sync::OnceLock<Option<std::sync::Mutex<std::fs::File>>> =
        std::sync::OnceLock::new();
    let sink = SINK.get_or_init(|| {
        let path = std::env::var_os(LOG_VAR)?;
        std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .ok()
            .map(std::sync::Mutex::new)
    });
    let Some(sink) = sink else {
        return;
    };
    let thread = std::thread::current();
    let locator = event.sections.first().map_or_else(
        || "-".to_owned(),
        |s| format!("({:.6},{:.6},{:.6})", s.start.x(), s.start.y(), s.start.z()),
    );
    let shadow_regions = event
        .shadow_regions
        .map_or_else(|| "-".to_owned(), |n| n.to_string());
    let line = format!(
        "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n",
        event.budgets.label(),
        event.entry.name(),
        outcome_label(event.outcome),
        event.reason.replace(['\t', '\n', '\r'], " "),
        thread.name().unwrap_or("-"),
        event.face_id.index(),
        event.surface,
        event.sections.len(),
        event.legacy_regions,
        shadow_regions,
        locator,
    );
    if let Ok(mut file) = sink.lock() {
        // A diagnostic sink: a failed write loses a census line, never a
        // result.
        let _ = file.write_all(line.as_bytes());
    }
}

/// The first property on which two partitions part, shallowest first.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Mismatch {
    /// The partitions have different numbers of regions.
    RegionCount,
    /// A region has no partner with the same number of holes.
    HoleCount,
    /// A wire has no partner with the same number of edges.
    EdgeCount,
    /// No rotation aligns a wire's carrier kinds with its partner's.
    KindSequence,
    /// The kinds align, but a vertex is farther than the linear tolerance
    /// from its twin.
    Vertex,
    /// Kinds and vertices align, but a curved edge lies on another carrier
    /// (or on the same curve under another parametrisation).
    Carrier,
    /// The carrier is shared, but the signed parameter sub-span differs.
    SubSpan,
    /// The carrier and both interval ends are shared, but one side's
    /// traversal interval runs the other way: its `trim` is stored in
    /// traversal order on a reversed use, where
    /// [`OrientedPCurveEdge::trim`] is native-direction.
    SpanDirection,
    /// Every wire matches, but the regions' `reversed` flags differ.
    Orientation,
}

impl Mismatch {
    const fn label(self) -> &'static str {
        match self {
            Self::RegionCount => "region-count",
            Self::HoleCount => "hole-count",
            Self::EdgeCount => "edge-count",
            Self::KindSequence => "kind-sequence",
            Self::Vertex => "vertex",
            Self::Carrier => "carrier",
            Self::SubSpan => "sub-span",
            Self::SpanDirection => "span-direction",
            Self::Orientation => "orientation",
        }
    }
}

/// The result of comparing a legacy partition with the arrangement's.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Verdict {
    /// Strictly equal (see the module documentation).
    Agreed,
    /// Equal once consecutive edges on one carrier are merged; carries the
    /// strict mismatch that merging resolved.
    SegmentationOnly(Mismatch),
    /// Different even after merging.
    Different(Mismatch),
}

/// Compare the legacy partition with the arrangement's.
fn compare(legacy: &[SplitSubFace], shadow: &[SplitSubFace], tol: f64) -> Verdict {
    match compare_regions(legacy, shadow, tol, false) {
        Ok(()) => Verdict::Agreed,
        Err(strict) => match compare_regions(legacy, shadow, tol, true) {
            Ok(()) => Verdict::SegmentationOnly(strict),
            Err(merged) => Verdict::Different(merged),
        },
    }
}

fn compare_regions(
    legacy: &[SplitSubFace],
    shadow: &[SplitSubFace],
    tol: f64,
    merge: bool,
) -> Result<(), Mismatch> {
    if legacy.len() != shadow.len() {
        return Err(Mismatch::RegionCount);
    }
    let mut taken = vec![false; shadow.len()];
    for region in legacy {
        // The deepest mismatch among the free candidates names the nearest
        // miss, which is the informative one.
        let mut deepest = Mismatch::RegionCount;
        let mut hit = None;
        for (index, candidate) in shadow.iter().enumerate() {
            if taken[index] {
                continue;
            }
            match compare_region(region, candidate, tol, merge) {
                Ok(()) => {
                    hit = Some(index);
                    break;
                }
                Err(mismatch) => deepest = deepest.max(mismatch),
            }
        }
        match hit {
            Some(index) => taken[index] = true,
            None => return Err(deepest),
        }
    }
    Ok(())
}

fn compare_region(
    legacy: &SplitSubFace,
    shadow: &SplitSubFace,
    tol: f64,
    merge: bool,
) -> Result<(), Mismatch> {
    if legacy.inner_wires.len() != shadow.inner_wires.len() {
        return Err(Mismatch::HoleCount);
    }
    compare_wire(&legacy.outer_wire, &shadow.outer_wire, tol, merge)?;
    let mut taken = vec![false; shadow.inner_wires.len()];
    for hole in &legacy.inner_wires {
        let mut deepest = Mismatch::EdgeCount;
        let mut hit = None;
        for (index, candidate) in shadow.inner_wires.iter().enumerate() {
            if taken[index] {
                continue;
            }
            match compare_wire(hole, candidate, tol, merge) {
                Ok(()) => {
                    hit = Some(index);
                    break;
                }
                Err(mismatch) => deepest = deepest.max(mismatch),
            }
        }
        match hit {
            Some(index) => taken[index] = true,
            None => return Err(deepest),
        }
    }
    if legacy.reversed != shadow.reversed {
        return Err(Mismatch::Orientation);
    }
    Ok(())
}

/// One wire edge, or one run of consecutive edges merged along a carrier.
struct Seg<'a> {
    curve: &'a EdgeCurve,
    start: Point3,
    end: Point3,
    /// Signed carrier interval in traversal order.
    span: (f64, f64),
}

const fn kind(curve: &EdgeCurve) -> u8 {
    match curve {
        EdgeCurve::Line => 0,
        EdgeCurve::NurbsCurve(_) => 1,
        EdgeCurve::Circle(_) => 2,
        EdgeCurve::Ellipse(_) => 3,
        EdgeCurve::Hyperbola(_) => 4,
        EdgeCurve::Parabola(_) => 5,
    }
}

fn seg(edge: &OrientedPCurveEdge) -> Seg<'_> {
    Seg {
        curve: &edge.curve_3d,
        start: edge.start_3d,
        end: edge.end_3d,
        span: edge.traversal_domain(),
    }
}

/// Parameter tolerance matching the linear tolerance on this carrier.
fn span_tolerance(curve: &EdgeCurve, tol: f64) -> f64 {
    match curve {
        EdgeCurve::Circle(circle) => (tol / circle.radius().max(f64::MIN_POSITIVE)).max(1e-9),
        EdgeCurve::Line
        | EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => 1e-7,
    }
}

/// Whether both carriers trace the same points at the same parameters over
/// `a`'s span: three shared points fix a conic, and equal parameters fix
/// its reference direction.
fn same_carrier(a: &Seg<'_>, b: &Seg<'_>, tol: f64) -> bool {
    if matches!(a.curve, EdgeCurve::Line) {
        return true;
    }
    let (t0, t1) = a.span;
    [t0, 0.5 * (t0 + t1), t1].into_iter().all(|t| {
        let pa = a.curve.evaluate_with_endpoints(t, a.start, a.end);
        let pb = b.curve.evaluate_with_endpoints(t, b.start, b.end);
        (pa - pb).length() <= tol
    })
}

/// Whether both segments cover the same signed span, up to a whole number
/// of turns on a closed carrier.
fn same_span(a: &Seg<'_>, b: &Seg<'_>, tol: f64) -> bool {
    if matches!(a.curve, EdgeCurve::Line) {
        return true;
    }
    let band = span_tolerance(a.curve, tol);
    let mut shift = b.span.0 - a.span.0;
    if matches!(a.curve, EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_)) {
        shift -= (shift / TAU).round() * TAU;
    }
    let length = (b.span.1 - b.span.0) - (a.span.1 - a.span.0);
    shift.abs() <= band && length.abs() <= band
}

/// The same segment with its interval read the other way.
fn flipped<'a>(seg: &Seg<'a>) -> Seg<'a> {
    Seg {
        curve: seg.curve,
        start: seg.start,
        end: seg.end,
        span: (seg.span.1, seg.span.0),
    }
}

/// Whether `next` continues `run` along one carrier in one direction.
fn continues(run: &Seg<'_>, next: &Seg<'_>, tol: f64) -> bool {
    if kind(run.curve) != kind(next.curve) || (run.end - next.start).length() > tol {
        return false;
    }
    if matches!(run.curve, EdgeCurve::Line) {
        let (d0, d1) = (run.end - run.start, next.end - next.start);
        return d0.dot(d1) > 0.0 && d0.cross(d1).length() <= tol * (d0.length() + d1.length());
    }
    let same_sense = (run.span.1 - run.span.0) * (next.span.1 - next.span.0) > 0.0;
    same_sense && same_carrier(next, run, tol)
}

/// Merge consecutive edges that continue one carrier, cyclically.
fn merged(wire: &[OrientedPCurveEdge], tol: f64) -> Vec<Seg<'_>> {
    let n = wire.len();
    let segs: Vec<Seg<'_>> = wire.iter().map(seg).collect();
    if n < 2 {
        return segs;
    }
    // Start at a carrier change so no run straddles the wire's stored start.
    let Some(first) = (0..n).find(|&i| !continues(&segs[(i + n - 1) % n], &segs[i], tol)) else {
        // One carrier all the way round: a single closed run.
        let length: f64 = segs.iter().map(|s| s.span.1 - s.span.0).sum();
        return vec![Seg {
            curve: segs[0].curve,
            start: segs[0].start,
            end: segs[0].start,
            span: (segs[0].span.0, segs[0].span.0 + length),
        }];
    };
    let mut runs: Vec<Seg<'_>> = Vec::new();
    for offset in 0..n {
        let next = &segs[(first + offset) % n];
        match runs.last_mut() {
            Some(run) if continues(run, next, tol) => {
                run.end = next.end;
                run.span.1 += next.span.1 - next.span.0;
            }
            _ => runs.push(Seg {
                curve: next.curve,
                start: next.start,
                end: next.end,
                span: next.span,
            }),
        }
    }
    runs
}

fn compare_wire(
    legacy: &[OrientedPCurveEdge],
    shadow: &[OrientedPCurveEdge],
    tol: f64,
    merge: bool,
) -> Result<(), Mismatch> {
    let (a, b) = if merge {
        (merged(legacy, tol), merged(shadow, tol))
    } else {
        (
            legacy.iter().map(seg).collect(),
            shadow.iter().map(seg).collect(),
        )
    };
    if a.len() != b.len() {
        return Err(Mismatch::EdgeCount);
    }
    let n = a.len();
    if n == 0 {
        return Ok(());
    }
    // A wire merged into one closed run has no vertex to compare: its start
    // is wherever the emitter happened to cut the carrier.
    let closed_run = merge && n == 1 && (a[0].start - a[0].end).length() <= tol;
    let mut deepest = Mismatch::KindSequence;
    for rotation in 0..n {
        let pairs = || (0..n).map(|i| (&a[i], &b[(i + rotation) % n]));
        if !pairs().all(|(x, y)| kind(x.curve) == kind(y.curve)) {
            continue;
        }
        if closed_run {
            let length = (a[0].span.1 - a[0].span.0).abs() - (b[0].span.1 - b[0].span.0).abs();
            if !same_carrier(&a[0], &b[0], tol) {
                deepest = deepest.max(Mismatch::Carrier);
            } else if length.abs() > span_tolerance(a[0].curve, tol) {
                deepest = deepest.max(Mismatch::SubSpan);
            } else {
                return Ok(());
            }
            continue;
        }
        let near = |p: Point3, q: Point3| (p - q).length() <= tol;
        if !pairs().all(|(x, y)| near(x.start, y.start) && near(x.end, y.end)) {
            deepest = deepest.max(Mismatch::Vertex);
            continue;
        }
        if !pairs().all(|(x, y)| same_carrier(x, y, tol)) {
            deepest = deepest.max(Mismatch::Carrier);
            continue;
        }
        if pairs().all(|(x, y)| same_span(x, y, tol)) {
            return Ok(());
        }
        deepest = deepest.max(
            if pairs().all(|(x, y)| same_span(x, y, tol) || same_span(x, &flipped(y), tol)) {
                Mismatch::SpanDirection
            } else {
                Mismatch::SubSpan
            },
        );
    }
    Err(deepest)
}

#[cfg(test)]
mod tests;
