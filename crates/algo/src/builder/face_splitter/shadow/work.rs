//! Work probe for the arrangement adapters' budget policy (Open Kernel
//! O2.3d, budget-policy research).
//!
//! Test and `perf-counters` builds only, and inert until
//! `REMUS_SPLITTER_WORK_LOG=<path>` names a log (or a unit test calls
//! [`measure`] directly). Wherever the shadow differential runs, and at the
//! two production adapter call sites (the early cylinder arrangement and the
//! last-resort planar arrangement), it measures what the adapter needs on
//! that face, without changing any result:
//!
//! - the smallest `march_steps`, `queue_size` and `segments` that reach the
//!   raised-budget answer, each found by bisection with the other two raised
//!   (the adapters are deterministic and every cap is monotone);
//! - which caps the operation's own budgets would trip, one at a time;
//! - where the march steps go, from phase marks the adapters and the core
//!   record through [`mark`] while a probe is active;
//! - the input and arrangement sizes the work could be scaled by;
//! - wall time of the adapter (raised budgets) and of the legacy emitter it
//!   would replace, each the minimum of [`TIMING_RUNS`] runs.
//!
//! Every adapter run here passes no split registry and reads the topology
//! immutably, so a probe leaves no trace in shared state.

use std::cell::RefCell;
use std::fmt::Write as _;
use std::hash::Hasher as _;
use std::time::Instant;

use remus_math::context::{OperationContext, WorkBudgets};
use remus_math::curves2d::Curve2D;
use remus_math::det_hash::DetHasher;
use remus_math::tolerance::Tolerance;
use remus_topology::Topology;
use remus_topology::face::FaceId;

use super::super::super::plane_frame::PlaneFrame;
use super::super::super::split_types::{SectionEdge, SplitSubFace};
use super::super::arrangement::Arrangement;
use super::super::{arrangement_cyl, arrangement_prod};
use super::RAISED_BUDGETS;
use crate::ds::Rank;
use crate::error::AlgoError;

/// Environment variable naming the file work records are appended to.
const LOG_VAR: &str = "REMUS_SPLITTER_WORK_LOG";
/// Runs per timing; the minimum is reported.
const TIMING_RUNS: usize = 5;

thread_local! {
    /// Phase marks of the adapter run in progress, when a probe asked.
    static MARKS: RefCell<Option<Vec<(&'static str, usize)>>> = const { RefCell::new(None) };
}

/// Record a phase mark if a probe is listening on this thread.
///
/// Collection phases record the steps used so far; core phases record the
/// core's remaining steps (see [`Phases`]).
pub(in crate::builder::face_splitter) fn mark(phase: &'static str, value: usize) {
    MARKS.with(|marks| {
        if let Some(marks) = marks.borrow_mut().as_mut() {
            marks.push((phase, value));
        }
    });
}

fn with_marks<R>(f: impl FnOnce() -> R) -> (R, Vec<(&'static str, usize)>) {
    // `f` runs the adapter under `catch_unwind`, so it always returns here.
    let previous = MARKS.with(|marks| marks.borrow_mut().replace(Vec::new()));
    let result = f();
    let marks = MARKS.with(|marks| std::mem::replace(&mut *marks.borrow_mut(), previous));
    (result, marks.unwrap_or_default())
}

/// Where a probe was taken.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(in crate::builder::face_splitter) enum Site {
    /// A legacy special case fired; the shadow differential ran.
    Legacy(&'static str),
    /// The early cylinder arrangement in `split_face_2d_impl` (any
    /// result is adopted).
    EarlyCylinder,
    /// The last-resort planar arrangement in `split_face_2d_impl`.
    LastResortPlane {
        /// Regions the wire builder traced.
        greedy_regions: usize,
        /// Whether the greedy trace is demonstrably broken.
        greedy_broken: bool,
    },
}

impl Site {
    fn label(self) -> &'static str {
        match self {
            Self::Legacy(name) => name,
            Self::EarlyCylinder => "prod:early-cylinder",
            Self::LastResortPlane { .. } => "prod:last-resort-plane",
        }
    }
}

/// Which adapter a probe drives.
#[derive(Clone, Copy, Debug)]
pub(in crate::builder::face_splitter) enum Adapter<'a> {
    Plane(&'a PlaneFrame),
    Cylinder,
}

/// One face's adapter input.
pub(in crate::builder::face_splitter) struct Probe<'a> {
    pub topo: &'a Topology,
    pub face_id: FaceId,
    pub sections: &'a [SectionEdge],
    pub rank: Rank,
    pub tol: &'a Tolerance,
    pub context: &'a OperationContext,
    pub adapter: Adapter<'a>,
}

/// What one adapter run produced.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(in crate::builder::face_splitter) enum Outcome {
    Split(usize),
    Declined,
    Budget,
    Refused(String),
    Panic,
}

impl Outcome {
    fn label(&self) -> String {
        match self {
            Self::Split(n) => format!("split:{n}"),
            Self::Declined => "declined".to_owned(),
            Self::Budget => "budget".to_owned(),
            Self::Refused(text) => format!("refused:{}", text.replace(['\t', '\n', '\r'], " ")),
            Self::Panic => "panic".to_owned(),
        }
    }
}

impl Probe<'_> {
    /// Content identity for this diagnostic schema, independent of arena
    /// lineage and timing. Debug floats round-trip; the exact topology view
    /// includes every arena slot and pcurve, but no process-local cache IDs.
    /// Hash the stream directly to avoid allocating a topology-sized string.
    fn fingerprint(&self, site: Site) -> String {
        struct Digest(DetHasher);
        impl std::fmt::Write for Digest {
            fn write_str(&mut self, text: &str) -> std::fmt::Result {
                self.0.write(text.as_bytes());
                Ok(())
            }
        }
        let mut digest = Digest(DetHasher::new());
        let _ = write!(
            digest,
            "{:?}|{:?}|{:?}|{:?}|{:?}|{:?}|{:?}|{:?}",
            self.topo.image_view(),
            self.face_id,
            self.sections,
            self.rank,
            self.tol,
            self.context,
            self.adapter,
            site,
        );
        format!("v1:{:016x}", digest.0.finish())
    }

    fn call(
        &self,
        budgets: WorkBudgets,
    ) -> std::thread::Result<Result<Option<Vec<SplitSubFace>>, AlgoError>> {
        // The adapters report exhaustion typed whatever the budgets, so a
        // budget decline is told apart from an out-of-domain one directly.
        let context = self.context.clone().with_budgets(budgets);
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| match self.adapter {
            Adapter::Plane(frame) => {
                arrangement_prod::try_split_plane_face_by_provenance_arrangement(
                    self.topo,
                    self.face_id,
                    self.sections,
                    self.rank,
                    frame,
                    self.tol,
                    &context,
                    None,
                )
            }
            Adapter::Cylinder => {
                arrangement_cyl::try_split_cylinder_face_by_provenance_arrangement(
                    self.topo,
                    self.face_id,
                    self.sections,
                    self.rank,
                    self.tol,
                    &context,
                    None,
                )
            }
        }))
    }

    fn run(&self, budgets: WorkBudgets) -> Outcome {
        match self.call(budgets) {
            Err(_) => Outcome::Panic,
            Ok(Ok(Some(parts))) => Outcome::Split(parts.len()),
            Ok(Ok(None)) => Outcome::Declined,
            Ok(Err(error)) => {
                let text = format!("{error:?}");
                if text.contains("WorkBudgetExceeded") {
                    Outcome::Budget
                } else {
                    Outcome::Refused(text)
                }
            }
        }
    }

    /// The arrangement itself under raised budgets, for its sizes.
    fn arrangement(&self) -> Option<(Arrangement, Vec<remus_topology::edge::EdgeCurve>, usize)> {
        let context = self.context.clone().with_budgets(RAISED_BUDGETS);
        match self.adapter {
            Adapter::Plane(frame) => {
                let inputs = arrangement_prod::collect_planar_uses(
                    self.topo,
                    self.face_id,
                    self.sections,
                    self.rank,
                    frame,
                    self.tol,
                    &context,
                )
                .ok()??;
                let arrangement =
                    arrangement_prod::run_planar_arrangement(&inputs, &context).ok()?;
                let boundary = inputs
                    .uses
                    .iter()
                    .filter(|u| u.boundary_loop.is_some())
                    .count();
                let kinds = inputs.uses.iter().map(|u| u.curve_3d.clone()).collect();
                Some((arrangement, kinds, boundary))
            }
            Adapter::Cylinder => {
                let inputs = arrangement_cyl::collect_cylinder_uses(
                    self.topo,
                    self.face_id,
                    self.sections,
                    self.rank,
                    self.tol,
                    &context,
                )
                .ok()??;
                let arrangement =
                    arrangement_cyl::run_cylinder_arrangement(&inputs, &context).ok()?;
                let boundary = inputs
                    .uses
                    .iter()
                    .filter(|u| u.boundary_loop.is_some())
                    .count();
                let kinds = inputs.uses.iter().map(|u| u.curve_3d.clone()).collect();
                Some((arrangement, kinds, boundary))
            }
        }
    }

    /// Smallest value of one budget field (the others raised) that does
    /// not exhaust the budget, or `None` when even `hi` does.
    fn least(&self, set: impl Fn(WorkBudgets, usize) -> WorkBudgets, hi: usize) -> Option<usize> {
        let fits = |value: usize| self.run(set(RAISED_BUDGETS, value)) != Outcome::Budget;
        if !fits(hi) {
            return None;
        }
        let (mut lo, mut hi) = (0usize, hi);
        // Invariant: `hi` fits; every value below `lo` does not.
        while lo < hi {
            let mid = lo + (hi - lo) / 2;
            if fits(mid) {
                hi = mid;
            } else {
                lo = mid + 1;
            }
        }
        Some(hi)
    }
}

/// Phase totals of one adapter run: cumulative march steps after each
/// phase, collection and core on one scale.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(in crate::builder::face_splitter) struct Phases(pub Vec<(&'static str, usize)>);

impl Phases {
    fn from_marks(marks: &[(&'static str, usize)]) -> Self {
        let mut out = Vec::new();
        let mut collected = 0usize;
        let mut core_budget = None;
        for &(phase, value) in marks {
            match phase {
                "collect-concurrency" | "collect-prejoin" | "scan" => {
                    collected = value;
                    out.push((phase, value));
                }
                "core-budget" => core_budget = Some(value),
                p if p.starts_with('#') => out.push((phase, value)),
                _ => {
                    if let Some(budget) = core_budget {
                        out.push((phase, collected + budget.saturating_sub(value)));
                    }
                }
            }
        }
        Self(out)
    }

    fn render(&self) -> String {
        if self.0.is_empty() {
            return "-".to_owned();
        }
        self.0
            .iter()
            .map(|(phase, value)| format!("{phase}={value}"))
            .collect::<Vec<_>>()
            .join(";")
    }
}

/// One face's measurements.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(in crate::builder::face_splitter) struct Record {
    pub raised: Outcome,
    pub default: Outcome,
    /// Which of march / queue / segments, alone at the operation's value
    /// with the other two raised, exhausts the budget.
    pub trips: [bool; 3],
    pub march: Option<usize>,
    pub queue: Option<usize>,
    pub segments: Option<usize>,
    pub phases: Phases,
    pub loop_edges: usize,
    pub uses: usize,
    pub boundary_uses: usize,
    pub circle_uses: usize,
    pub vertices: usize,
    pub edges: usize,
    pub regions: usize,
    pub max_degree: usize,
    pub sum_degree_sq: usize,
    pub adapter_ns: Option<u128>,
}

fn min_time(mut f: impl FnMut()) -> u128 {
    (0..TIMING_RUNS)
        .map(|_| {
            let start = Instant::now();
            f();
            start.elapsed().as_nanos()
        })
        .min()
        .unwrap_or(0)
}

/// Measure one face. Pure: results are not adopted anywhere.
pub(in crate::builder::face_splitter) fn measure(probe: &Probe<'_>) -> Record {
    let (raised, marks) = with_marks(|| probe.run(RAISED_BUDGETS));
    let own = probe.context.budgets;
    let default = probe.run(own);
    let trips = [
        probe.run(RAISED_BUDGETS.with_march_steps(own.march_steps)) == Outcome::Budget,
        probe.run(RAISED_BUDGETS.with_queue_size(own.queue_size)) == Outcome::Budget,
        probe.run(RAISED_BUDGETS.with_segments(own.segments)) == Outcome::Budget,
    ];
    let march = probe.least(WorkBudgets::with_march_steps, RAISED_BUDGETS.march_steps);
    let queue = probe.least(WorkBudgets::with_queue_size, RAISED_BUDGETS.queue_size);
    let segments = probe.least(WorkBudgets::with_segments, RAISED_BUDGETS.segments);
    let loop_edges = probe.topo.loops_of_face(probe.face_id).map_or(0, |loops| {
        loops
            .iter()
            .filter_map(|&l| probe.topo.face_loop(l).ok())
            .map(|l| l.coedges().len())
            .sum()
    });
    let mut record = Record {
        adapter_ns: matches!(raised, Outcome::Split(_))
            .then(|| min_time(|| drop(probe.call(RAISED_BUDGETS)))),
        raised,
        default,
        trips,
        march,
        queue,
        segments,
        phases: Phases::from_marks(&marks),
        loop_edges,
        uses: 0,
        boundary_uses: 0,
        circle_uses: 0,
        vertices: 0,
        edges: 0,
        regions: 0,
        max_degree: 0,
        sum_degree_sq: 0,
    };
    if let Some((arrangement, kinds, boundary)) = probe.arrangement() {
        record.uses = kinds.len();
        record.boundary_uses = boundary;
        record.circle_uses = arrangement
            .sources
            .iter()
            .filter(|u| matches!(u.pcurve, Curve2D::Circle(_)))
            .count();
        record.vertices = arrangement.vertices.len();
        record.edges = arrangement.half_edges.len() / 2;
        record.regions = arrangement.regions.len();
        let mut degree = vec![0usize; arrangement.vertices.len()];
        for half in &arrangement.half_edges {
            degree[half.from] += 1;
        }
        record.max_degree = degree.iter().copied().max().unwrap_or(0);
        record.sum_degree_sq = degree.iter().map(|d| d * d).sum();
    }
    record
}

#[cfg(all(test, feature = "perf-counters"))]
enum SinkOverride {
    Environment,
    Forced(Option<std::sync::Mutex<std::fs::File>>),
}

#[cfg(all(test, feature = "perf-counters"))]
thread_local! {
    static TEST_SINK: RefCell<SinkOverride> = const { RefCell::new(SinkOverride::Environment) };
}

#[cfg(all(test, feature = "perf-counters"))]
pub(super) fn with_log<R>(file: Option<std::fs::File>, f: impl FnOnce() -> R) -> R {
    struct Restore(SinkOverride);
    impl Drop for Restore {
        fn drop(&mut self) {
            TEST_SINK.with(|sink| {
                *sink.borrow_mut() = std::mem::replace(&mut self.0, SinkOverride::Environment);
            });
        }
    }
    let _restore = Restore(
        TEST_SINK.with(|sink| sink.replace(SinkOverride::Forced(file.map(std::sync::Mutex::new)))),
    );
    f()
}

fn log_sink() -> Option<&'static std::sync::Mutex<std::fs::File>> {
    static SINK: std::sync::OnceLock<Option<std::sync::Mutex<std::fs::File>>> =
        std::sync::OnceLock::new();
    SINK.get_or_init(|| {
        let path = std::env::var_os(LOG_VAR)?;
        std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .ok()
            .map(std::sync::Mutex::new)
    })
    .as_ref()
}

fn opt<T: ToString>(value: Option<T>) -> String {
    value.map_or_else(|| "-".to_owned(), |v| v.to_string())
}

/// Probe one face and append a record, when `REMUS_SPLITTER_WORK_LOG` is
/// set. `legacy` re-runs the legacy emitter for timing (legacy sites only);
/// `legacy_regions` is its region count.
pub(in crate::builder::face_splitter) fn probe(
    site: Site,
    probe: &Probe<'_>,
    surface: &str,
    legacy: Option<(&dyn Fn(), usize)>,
) {
    #[cfg(all(test, feature = "perf-counters"))]
    {
        let overridden = TEST_SINK.with(|sink| {
            let borrowed = sink.borrow();
            if let SinkOverride::Forced(sink) = &*borrowed {
                if let Some(sink) = sink {
                    probe_to_sink(sink, site, probe, surface, legacy);
                }
                true
            } else {
                false
            }
        });
        if overridden {
            return;
        }
    }
    if let Some(sink) = log_sink() {
        probe_to_sink(sink, site, probe, surface, legacy);
    }
}

fn probe_to_sink(
    sink: &std::sync::Mutex<std::fs::File>,
    site: Site,
    probe: &Probe<'_>,
    surface: &str,
    legacy: Option<(&dyn Fn(), usize)>,
) {
    use std::io::Write as _;
    // Diagnostic reruns must not become production work in the existing
    // complexity guards. Restore on unwind as well as on normal return.
    let (record, legacy_ns) = crate::perf::with_preserved_work_counters(|| {
        (measure(probe), legacy.map(|(rerun, _)| min_time(rerun)))
    });
    let adopt = match (site, &record.raised) {
        (Site::Legacy(_), _) => "-",
        (Site::EarlyCylinder, Outcome::Split(_)) => "yes",
        (
            Site::LastResortPlane {
                greedy_regions,
                greedy_broken,
            },
            Outcome::Split(n),
        ) if *n > greedy_regions || greedy_broken => "yes",
        _ => "no",
    };
    let thread = std::thread::current();
    let trips: Vec<&str> = ["march", "queue", "segments"]
        .into_iter()
        .zip(record.trips)
        .filter_map(|(name, hit)| hit.then_some(name))
        .collect();
    let fields = [
        site.label().to_owned(),
        thread.name().unwrap_or("-").to_owned(),
        probe.face_id.index().to_string(),
        surface.to_owned(),
        probe.sections.len().to_string(),
        record.loop_edges.to_string(),
        record.uses.to_string(),
        record.boundary_uses.to_string(),
        record.circle_uses.to_string(),
        record.raised.label(),
        record.default.label(),
        if trips.is_empty() {
            "-".to_owned()
        } else {
            trips.join("+")
        },
        opt(record.march),
        opt(record.queue),
        opt(record.segments),
        record.vertices.to_string(),
        record.edges.to_string(),
        record.regions.to_string(),
        record.max_degree.to_string(),
        record.sum_degree_sq.to_string(),
        opt(record.adapter_ns),
        opt(legacy_ns),
        opt(legacy.map(|(_, n)| n)),
        adopt.to_owned(),
        format!("{:?}", probe.context.budgets == WorkBudgets::new()),
        record.phases.render(),
        probe.fingerprint(site),
    ];
    let line = fields.join("\t") + "\n";
    if let Ok(mut file) = sink.lock() {
        // Diagnostic sink: a failed write loses a record, never a result.
        let _ = file.write_all(line.as_bytes());
    }
}
