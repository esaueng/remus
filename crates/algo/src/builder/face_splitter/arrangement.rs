//! Isolated O2.3b subdivision of certified line/circle pcurve uses.
//!
//! The caller supplies authoritative uses in one lifted chart, with an affine
//! pcurve-to-source parameter correspondence. No projection, fitting or
//! production splitter fallback happens here. Coincidence is never a weld.

use std::collections::BTreeMap;
use std::f64::consts::FRAC_PI_2;

use remus_math::context::OperationContext;
use remus_math::curves2d::Curve2D;
use remus_math::vec::{Point2, Point3};
use remus_topology::coedge::CoedgeId;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceId;
use remus_topology::face_loop::LoopId;

mod geometry;
mod periodic;
mod regions;
#[cfg(test)]
mod tests;

use geometry::{intersections, same, tangent_order};

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub(super) enum ArrangementError {
    #[error("non-finite arrangement input or computation")]
    NonFiniteInput,
    #[error("invalid or disconnected authoritative boundary")]
    InvalidBoundary,
    #[error("unsupported pcurve or parameter domain")]
    UnsupportedCurve,
    #[error("intersection could not be resolved without proximity identity")]
    IntersectionRefinementFailed,
    #[error("coincident source intervals require an overlap certificate")]
    AmbiguousOverlap,
    #[error("tangent contact has no certified limiting order")]
    AmbiguousContact,
    #[error("non-manifold arrangement embedding")]
    NonManifoldEmbedding,
    #[error("arrangement region is open or degenerate")]
    OpenRegion,
    #[error("arrangement work budget exceeded")]
    WorkBudgetExceeded,
    #[error("arrangement cancelled")]
    Cancelled,
    #[error("unsupported periodic identification")]
    UnsupportedDomain,
}
type Result<T> = std::result::Result<T, ArrangementError>;

/// Boundary identity is the coedge use, not just its underlying edge.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct BoundarySource {
    pub face: FaceId,
    pub boundary_loop: LoopId,
    pub coedge: CoedgeId,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct CurveSource {
    pub use_id: u64,
    pub boundary: Option<BoundarySource>,
    pub section: Option<usize>,
    pub source_edge_idx: Option<usize>,
    pub pave_block_id: Option<usize>,
}

#[derive(Debug, Clone)]
pub(super) struct CurveUse {
    pub source: CurveSource,
    pub pcurve: Curve2D,
    /// Signed native pcurve interval. Reversal swaps both intervals and IDs.
    pub range: [f64; 2],
    pub curve_3d: EdgeCurve,
    pub source_range: [f64; 2],
    pub endpoints_3d: [Point3; 2],
    /// Certificates supplied by topology/pave events, scoped to this chart.
    pub endpoints: [u64; 2],
    /// Material boundary loop key; sections have no loop key.
    pub boundary_loop: Option<u64>,
}

impl CurveUse {
    fn point(&self, t: f64) -> Point2 {
        self.pcurve.evaluate(t)
    }
    fn source_parameter(&self, t: f64) -> f64 {
        if same(t, self.range[0]) {
            return self.source_range[0];
        }
        if same(t, self.range[1]) {
            return self.source_range[1];
        }
        self.source_range[0]
            + (t - self.range[0]) / (self.range[1] - self.range[0])
                * (self.source_range[1] - self.source_range[0])
    }
    fn point_3d(&self, t: f64) -> Point3 {
        if same(t, self.range[0]) {
            return self.endpoints_3d[0];
        }
        if same(t, self.range[1]) {
            return self.endpoints_3d[1];
        }
        let parameter = if matches!(self.curve_3d, EdgeCurve::Line) {
            (t - self.range[0]) / (self.range[1] - self.range[0])
        } else {
            self.source_parameter(t)
        };
        self.curve_3d
            .evaluate_with_endpoints(parameter, self.endpoints_3d[0], self.endpoints_3d[1])
    }
}

/// The cylinder strip is already cut along two explicitly paired seam uses.
/// Both seam uses must be straight, vertical, and cover the same v interval.
#[derive(Debug, Clone, Copy)]
pub(super) enum ParamDomain {
    Plane,
    CylinderStrip { seam_uses: [u64; 2], radius: f64 },
}

pub(super) struct ArrangementInput<'a> {
    pub uses: &'a [CurveUse],
    pub domain: ParamDomain,
    pub context: &'a OperationContext,
}

#[derive(Debug, Clone, PartialEq)]
pub(super) struct EventIncidence {
    pub source_use: u64,
    pub parameter: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub(super) struct ArrangementVertex {
    pub uv: Point2,
    pub point_3d: Point3,
    pub incidences: Vec<EventIncidence>,
}

#[derive(Debug, Clone, PartialEq)]
pub(super) struct ArrangementHalfEdge {
    pub from: usize,
    pub to: usize,
    pub twin: usize,
    pub next: usize,
    pub source: CurveSource,
    pub range: [f64; 2],
    pub source_range: [f64; 2],
    pub endpoints_3d: [Point3; 2],
}

#[derive(Debug, Clone, PartialEq)]
pub(super) struct Cycle {
    pub edges: Vec<usize>,
    pub signed_area: f64,
    pub interior_left: Point2,
}

#[derive(Debug, Clone, PartialEq)]
pub(super) struct ArrangementRegion {
    /// Positive outer cycle and directly owned negative hole cycles.
    pub outer: usize,
    pub holes: Vec<usize>,
    pub interior: Point2,
    pub area: f64,
    pub material: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub(super) struct DomainIdentification {
    pub vertices: [usize; 2],
    pub u_lift: i32,
}

#[derive(Debug, Clone, PartialEq)]
pub(super) struct PeriodicRegion {
    /// Lifted cells joined across the declared seam, in canonical order.
    pub cells: Vec<usize>,
    /// Boundary cycles after seam removal; each retains integer u winding.
    pub boundaries: Vec<(Vec<usize>, i32)>,
    pub area: f64,
    pub euler_characteristic: isize,
}

#[derive(Debug, Clone)]
pub(super) struct Arrangement {
    /// Canonical, unmodified exact carriers; half-edges reference their use IDs.
    pub sources: Vec<CurveUse>,
    pub vertices: Vec<ArrangementVertex>,
    pub half_edges: Vec<ArrangementHalfEdge>,
    pub cycles: Vec<Cycle>,
    pub regions: Vec<ArrangementRegion>,
    /// All negative cycles belonging to the one unbounded face.
    pub exterior: Vec<usize>,
    pub identifications: Vec<DomainIdentification>,
    pub periodic_regions: Vec<PeriodicRegion>,
}

/// `march_steps` charges every inner work step; `queue_size` caps inputs and
/// event records; `segments` caps emitted undirected edges. No hidden retry
/// budget: seed refinement consumes the same operation-local work counter.
struct Work<'a> {
    context: &'a OperationContext,
    remaining: usize,
    u_scale: f64,
}
impl<'a> Work<'a> {
    const fn new(context: &'a OperationContext) -> Self {
        Self {
            context,
            remaining: context.budgets.march_steps,
            u_scale: 1.0,
        }
    }
    fn uv_distance(&self, a: Point2, b: Point2) -> f64 {
        ((a.x() - b.x()) * self.u_scale).hypot(a.y() - b.y())
    }
    fn step(&mut self) -> Result<()> {
        self.context
            .check_cancelled()
            .map_err(|_| ArrangementError::Cancelled)?;
        self.remaining = self
            .remaining
            .checked_sub(1)
            .ok_or(ArrangementError::WorkBudgetExceeded)?;
        Ok(())
    }
    fn capacity(&mut self, n: usize) -> Result<()> {
        self.step()?;
        if n > self.context.budgets.queue_size {
            return Err(ArrangementError::WorkBudgetExceeded);
        }
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct Cut {
    parameter: f64,
    event: usize,
}

struct Events {
    parent: Vec<usize>,
}
impl Events {
    fn add(&mut self, work: &mut Work<'_>) -> Result<usize> {
        work.capacity(self.parent.len() + 1)?;
        let id = self.parent.len();
        self.parent.push(id);
        Ok(id)
    }
    fn root(&self, mut i: usize, work: &mut Work<'_>) -> Result<usize> {
        while self.parent[i] != i {
            work.step()?;
            i = self.parent[i];
        }
        Ok(i)
    }
    fn join(&mut self, a: usize, b: usize, work: &mut Work<'_>) -> Result<()> {
        let a = self.root(a, work)?;
        let b = self.root(b, work)?;
        self.parent[a.max(b)] = a.min(b);
        Ok(())
    }
}

/// Returns only a fully checked arrangement. All staging is operation-local.
#[allow(clippy::too_many_lines)]
pub(super) fn build_arrangement(input: &ArrangementInput<'_>) -> Result<Arrangement> {
    let mut work = Work::new(input.context);
    if let ParamDomain::CylinderStrip { radius, .. } = input.domain {
        if !radius.is_finite() || radius <= 0.0 {
            return Err(ArrangementError::UnsupportedDomain);
        }
        work.u_scale = radius;
    }
    work.capacity(input.uses.len())?;
    let tol = input.context.tolerance.linear;
    if !tol.is_finite() || tol <= 0.0 {
        return Err(ArrangementError::NonFiniteInput);
    }
    for _ in 0..input.uses.len() {
        work.step()?;
    }
    let mut uses = input.uses.to_vec();
    uses.sort_by_key(|u| u.source.use_id);
    for i in 0..uses.len() {
        work.step()?;
        if i > 0 && uses[i - 1].source.use_id == uses[i].source.use_id {
            return Err(ArrangementError::InvalidBoundary);
        }
        geometry::validate_use(&uses[i], tol)?;
        if uses[i].range[0] > uses[i].range[1] {
            uses[i].range.swap(0, 1);
            uses[i].source_range.swap(0, 1);
            uses[i].endpoints.swap(0, 1);
            uses[i].endpoints_3d.swap(0, 1);
        }
    }
    validate_boundaries(&uses, &mut work)?;
    let mut events = Events { parent: Vec::new() };
    let mut cuts = vec![Vec::<Cut>::new(); uses.len()];
    let mut endpoints = BTreeMap::<u64, (usize, Point2, Point3)>::new();
    for (i, u) in uses.iter().enumerate() {
        for end in 0..2 {
            work.step()?;
            let p = u.point(u.range[end]);
            let event = if let Some(&(event, uv, xyz)) = endpoints.get(&u.endpoints[end]) {
                if work.uv_distance(uv, p) > tol || (xyz - u.endpoints_3d[end]).length() > tol {
                    return Err(ArrangementError::InvalidBoundary);
                }
                event
            } else {
                let event = events.add(&mut work)?;
                endpoints.insert(u.endpoints[end], (event, p, u.endpoints_3d[end]));
                event
            };
            cuts[i].push(Cut {
                parameter: u.range[end],
                event,
            });
        }
        if matches!(u.pcurve, Curve2D::Circle(_)) {
            // Native cardinal cuts certify monotone ray crossings and preserve
            // full circles even when the authoritative endpoints coincide.
            for k in 0..=4 {
                work.step()?;
                let t = f64::from(k) * FRAC_PI_2;
                if t > u.range[0] && t < u.range[1] {
                    cuts[i].push(Cut {
                        parameter: t,
                        event: events.add(&mut work)?,
                    });
                }
            }
        }
    }
    for i in 0..uses.len() {
        for j in i + 1..uses.len() {
            work.step()?;
            for (a, b) in intersections(&uses[i], &uses[j], &mut work)? {
                let event = events.add(&mut work)?;
                cuts[i].push(Cut {
                    parameter: a,
                    event,
                });
                cuts[j].push(Cut {
                    parameter: b,
                    event,
                });
            }
        }
    }
    for (i, list) in cuts.iter_mut().enumerate() {
        list.sort_by(|a, b| a.parameter.total_cmp(&b.parameter));
        for pair in list.windows(2) {
            work.step()?;
            if same(pair[0].parameter, pair[1].parameter) {
                events.join(pair[0].event, pair[1].event, &mut work)?;
            } else if (uses[i].point(pair[0].parameter) - uses[i].point(pair[1].parameter)).length()
                <= geometry::roundoff(uses[i].point(pair[0].parameter))
            {
                return Err(ArrangementError::IntersectionRefinementFailed);
            }
        }
        list.dedup_by(|a, b| same(a.parameter, b.parameter));
    }
    let mut vertices = Vec::<ArrangementVertex>::new();
    let mut roots = BTreeMap::new();
    for (i, list) in cuts.iter_mut().enumerate() {
        for cut in list {
            work.step()?;
            let root = events.root(cut.event, &mut work)?;
            let id = *roots.entry(root).or_insert_with(|| {
                let id = vertices.len();
                vertices.push(ArrangementVertex {
                    uv: uses[i].point(cut.parameter),
                    point_3d: uses[i].point_3d(cut.parameter),
                    incidences: Vec::new(),
                });
                id
            });
            let xyz = uses[i].point_3d(cut.parameter);
            if !xyz.x().is_finite() || !xyz.y().is_finite() || !xyz.z().is_finite() {
                return Err(ArrangementError::NonFiniteInput);
            }
            if work.uv_distance(vertices[id].uv, uses[i].point(cut.parameter)) > tol
                || (vertices[id].point_3d - xyz).length() > tol
            {
                return Err(ArrangementError::IntersectionRefinementFailed);
            }
            vertices[id].incidences.push(EventIncidence {
                source_use: uses[i].source.use_id,
                parameter: cut.parameter,
            });
            cut.event = id;
        }
    }
    // Distinct pair roots may round to the same source parameter. A multiway
    // junction needs an additional common-endpoint or exact line-incidence
    // certificate; residual proximity alone cannot erase a tiny cell.
    for vertex in &vertices {
        work.step()?;
        let mut distinct_sources = 0;
        let mut previous = None;
        for incidence in &vertex.incidences {
            work.step()?;
            if previous != Some(incidence.source_use) {
                distinct_sources += 1;
            }
            previous = Some(incidence.source_use);
        }
        if distinct_sources < 3 {
            continue;
        }
        let mut common_endpoint = None;
        let mut all_endpoints = true;
        for incidence in &vertex.incidences {
            work.step()?;
            let u = &uses[uses
                .binary_search_by_key(&incidence.source_use, |u| u.source.use_id)
                .map_err(|_| ArrangementError::NonManifoldEmbedding)?];
            let endpoint = (0..2)
                .find(|&i| same(u.range[i], incidence.parameter))
                .map(|i| u.endpoints[i]);
            if common_endpoint.is_none() {
                common_endpoint = endpoint;
            }
            all_endpoints &= endpoint.is_some() && endpoint == common_endpoint;
        }
        if all_endpoints {
            continue;
        }
        for incidence in &vertex.incidences {
            work.step()?;
            let u = &uses[uses
                .binary_search_by_key(&incidence.source_use, |u| u.source.use_id)
                .map_err(|_| ArrangementError::NonManifoldEmbedding)?];
            if !matches!(u.pcurve, Curve2D::Line(_))
                || !same(
                    remus_math::predicates::orient2d(
                        u.point(u.range[0]),
                        u.point(u.range[1]),
                        vertex.uv,
                    ),
                    0.0,
                )
            {
                return Err(ArrangementError::IntersectionRefinementFailed);
            }
        }
    }
    let mut halves = Vec::new();
    let mut rotations = vec![Vec::new(); vertices.len()];
    for (i, list) in cuts.iter().enumerate() {
        for pair in list.windows(2) {
            work.step()?;
            if halves.len() / 2 >= input.context.budgets.segments {
                return Err(ArrangementError::WorkBudgetExceeded);
            }
            if pair[0].event == pair[1].event {
                return Err(ArrangementError::OpenRegion);
            }
            let h = halves.len();
            for (a, b, twin) in [(pair[0], pair[1], h + 1), (pair[1], pair[0], h)] {
                let source_range = [
                    uses[i].source_parameter(a.parameter),
                    uses[i].source_parameter(b.parameter),
                ];
                if !source_range.iter().all(|t| t.is_finite()) {
                    return Err(ArrangementError::NonFiniteInput);
                }
                if same(source_range[0], source_range[1]) {
                    return Err(ArrangementError::IntersectionRefinementFailed);
                }
                rotations[a.event].push(halves.len());
                halves.push(ArrangementHalfEdge {
                    from: a.event,
                    to: b.event,
                    twin,
                    next: usize::MAX,
                    source: uses[i].source.clone(),
                    range: [a.parameter, b.parameter],
                    source_range,
                    endpoints_3d: [uses[i].point_3d(a.parameter), uses[i].point_3d(b.parameter)],
                });
            }
        }
    }
    let lookup: BTreeMap<_, _> = uses.iter().map(|u| (u.source.use_id, u)).collect();
    for rotation in &mut rotations {
        // Charge the comparison bound before the non-fallible standard sort.
        for _ in 0..rotation.len().saturating_mul(rotation.len()) {
            work.step()?;
        }
        rotation.sort_by(|&a, &b| tangent_order(&halves[a], &halves[b], &lookup));
        for pair in rotation.windows(2) {
            if tangent_order(&halves[pair[0]], &halves[pair[1]], &lookup).is_eq() {
                return Err(ArrangementError::AmbiguousContact);
            }
        }
    }
    let twins: Vec<_> = halves.iter().map(|h| h.twin).collect();
    let next =
        crate::builder::rotation_system::successors(&rotations, &twins, &mut || work.step())?
            .ok_or(ArrangementError::NonManifoldEmbedding)?;
    for (h, n) in halves.iter_mut().zip(next) {
        h.next = n;
    }
    let mut result = Arrangement {
        sources: uses.clone(),
        vertices,
        half_edges: halves,
        cycles: Vec::new(),
        regions: Vec::new(),
        exterior: Vec::new(),
        identifications: Vec::new(),
        periodic_regions: Vec::new(),
    };
    regions::extract(&mut result, &lookup, &mut work)?;
    periodic::quotient(&mut result, &lookup, input.domain, &mut work)?;
    work.step()?;
    Ok(result)
}

fn validate_boundaries(uses: &[CurveUse], work: &mut Work<'_>) -> Result<()> {
    let mut loops = BTreeMap::<u64, BTreeMap<u64, Vec<u64>>>::new();
    for u in uses {
        work.step()?;
        if let Some(key) = u.boundary_loop {
            let graph = loops.entry(key).or_default();
            graph
                .entry(u.endpoints[0])
                .or_default()
                .push(u.endpoints[1]);
            graph
                .entry(u.endpoints[1])
                .or_default()
                .push(u.endpoints[0]);
        }
    }
    if loops.is_empty() {
        return Err(ArrangementError::InvalidBoundary);
    }
    for graph in loops.values() {
        let Some(&first) = graph.keys().next() else {
            return Err(ArrangementError::InvalidBoundary);
        };
        let mut seen = std::collections::BTreeSet::new();
        let mut pending = vec![first];
        while let Some(v) = pending.pop() {
            work.step()?;
            if !seen.insert(v) {
                continue;
            }
            if graph[&v].len() != 2 {
                return Err(ArrangementError::InvalidBoundary);
            }
            for &n in &graph[&v] {
                work.step()?;
                pending.push(n);
            }
        }
        if seen.len() != graph.len() {
            return Err(ArrangementError::InvalidBoundary);
        }
    }
    Ok(())
}
