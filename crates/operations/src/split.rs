//! Split a solid into two halves along a cutting plane.
//!
//! The two halves are a *decomposition* of the input: every face of the body
//! either lands whole on one side or is trimmed into two pieces, and the only
//! new geometry is the cut cap that closes each half. [`split`] is written to
//! preserve that, so a face the plane never touches is carried through
//! verbatim — exact surface, curved edges, orientation, and every one of its
//! inner wires — rather than rebuilt from a list of corner positions.
//!
//! The cap is minted fresh, but it is not featureless: where the plane crosses
//! a bore, the cap gains that bore's cross-section as a hole, sharing the very
//! edge the bore wall was trimmed back to.
//!
//! Anything the plane meets in a way this has no exact construction for is
//! refused by name with [`OperationsError::Unsupported`], never approximated.
//!
//! A face is placed by the extent of its own boundary curves, computed in
//! closed form for lines and circles. That is exact for everything this
//! builds — a cylindrical band with full circular rims reaches no further
//! along the plane's normal than its rims do — but a curved patch could in
//! principle bow across the plane while its boundary stays clear of it. The
//! closing check catches that: the halves are a decomposition of the body, so
//! their volumes must add back up to it, and a face on the wrong side of the
//! cap shows up as material counted twice or not at all.

use std::collections::{BTreeMap, BTreeSet};

use remus_math::curves::Circle3D;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use crate::OperationsError;
use crate::boolean::{FaceSpec, assemble_solid_mixed_with_history};
use crate::dot_normal_point;
use crate::evolution::EvolutionMap;

/// Operation name carried by [`OperationsError::Unsupported`] refusals.
const OP: &str = "split";

/// Deflection for the closing volume checks. `solid_volume` integrates
/// analytic surfaces in closed form, so this only bounds the fallback paths.
const VOLUME_DEFLECTION: f64 = 0.01;

/// Samples used to outline a curved cap loop for the containment test.
const LOOP_SAMPLES: usize = 32;

/// Samples used to bound a curve with no closed-form extremes.
const RANGE_SAMPLES: usize = 128;

/// How nearly parallel a rim circle's axis must be to the cutting plane's
/// normal before the section through that rim is accepted as a circle.
const AXIS_PARALLEL_COSINE: f64 = 1.0 - 1e-9;

/// Relative slack on the volume-sum identity. The halves reuse the input's own
/// surfaces and `solid_volume` is exact on them, so the only difference is
/// accumulated floating-point rounding. Never widened to admit a case: a split
/// whose halves do not add up is wrong.
const VOLUME_SUM_RELATIVE_SLACK: f64 = 1e-9;

fn unsupported(reason: impl Into<String>) -> OperationsError {
    OperationsError::Unsupported {
        operation: OP,
        reason: reason.into(),
    }
}

/// Result of splitting a solid: two halves.
#[derive(Debug)]
pub struct SplitResult {
    /// The half on the positive side of the cutting plane (same side as normal).
    pub positive: SolidId,
    /// The half on the negative side of the cutting plane.
    pub negative: SolidId,
}

/// Which half of the cut a piece of the body belongs to.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Side {
    Pos,
    Neg,
}

impl Side {
    const fn name(self) -> &'static str {
        match self {
            Self::Pos => "positive",
            Self::Neg => "negative",
        }
    }
}

/// How the cutting plane meets one edge of the body.
#[derive(Clone, Copy)]
enum EdgeCut {
    /// The edge lies wholly on one side. Touching the plane at a point is
    /// allowed; straddling it is not.
    Whole(Side),
    /// A straight edge the plane crosses once, already split in two.
    Crossed {
        /// The point of the edge on the cutting plane.
        vertex: VertexId,
        /// The piece on the positive side, oriented like the source edge.
        pos: EdgeId,
        /// The piece on the negative side, oriented like the source edge.
        neg: EdgeId,
    },
}

/// The boundary the cut opened in one face, and the edge that closed it again.
///
/// The same curve bounds the trimmed face and the cap, so the two must share
/// one edge: that is what makes the cap's hole the bore's own rim rather than
/// a second circle lying on top of it.
#[derive(Clone, Copy)]
struct Connector {
    edge: EdgeId,
    /// The STORED wire sense the trimmed face uses for `edge`. The cap must
    /// traverse the edge opposite to the trimmed face's EFFECTIVE sense
    /// (stored XOR `face_reversed`), which is what pairs them across it.
    forward: bool,
    /// A closed rim (a bore's cross-section) rather than a straight chord.
    ring: bool,
    /// Whether the trimmed face is stored reversed. A reversed face (a bore
    /// wall) flips every stored sense's effective direction, so the cap's
    /// opposing sense must fold this in — negating `forward` alone pairs
    /// correctly only against non-reversed faces.
    face_reversed: bool,
}

/// One half under construction: the faces it takes whole or trimmed, and the
/// cut curves its cap has to close.
struct Half {
    side: Side,
    faces: Vec<FaceId>,
    /// Input-face arena index each `faces` entry derives from, in step.
    sources: Vec<usize>,
    connectors: Vec<Connector>,
}

/// Construction-derived face evolution of a split, one map per half.
///
/// Carried faces and trimmed faces are `modified` from their source; each
/// half's cap is synthesised by the operation and reported `unresolved`
/// with no candidates, following the boolean faithful path's convention.
#[derive(Debug)]
pub struct SplitEvolution {
    /// Evolution of the positive half's faces.
    pub positive: EvolutionMap,
    /// Evolution of the negative half's faces.
    pub negative: EvolutionMap,
}

/// Split a solid into two halves along a plane.
///
/// The cutting plane is defined by a point and a normal. The `positive` half
/// contains the geometry on the side the normal points toward; the `negative`
/// half contains the rest.
///
/// # Algorithm
///
/// 1. Every edge of the shell is classified against the plane and, where the
///    plane crosses it, split in two at the crossing point — once, so both
///    adjacent faces reuse the same pieces.
/// 2. A face wholly on one side is carried through verbatim, keeping its
///    surface, its curved edges, its orientation and all of its inner wires.
/// 3. A face the plane trims keeps the whole edges on its side, the split
///    pieces of the edges it crosses, and the inner wires that stay with it;
///    the gap left in its boundary is closed with the curve the plane cut
///    across it — a chord for a planar face, a rim circle for a bore wall.
/// 4. Those same cut curves, traversed the other way round, are assembled into
///    the cap that closes each half. A rim circle enclosed by the section
///    becomes a hole in the cap.
/// 5. Both halves are validated, required to enclose positive volume, and
///    required to add back up to the input.
///
/// # Errors
///
/// Returns [`OperationsError::InvalidInput`] if `plane_normal` is zero-length
/// or the plane leaves the body entirely on one side.
///
/// Returns [`OperationsError::Unsupported`] when the split has no exact
/// construction here — the solid has cavity shells, the plane contains an edge
/// of the body, the plane crosses a curved edge or one of a face's inner
/// wires, the plane crosses a non-planar face other than square to a
/// cylinder's axis, the plane cuts a face into more than one piece, the
/// section does not close into a single outer loop, or a half fails
/// validation, encloses no volume, or does not account for its share of the
/// input's.
pub fn split(
    topo: &mut Topology,
    solid: SolidId,
    plane_point: Point3,
    plane_normal: Vec3,
) -> Result<SplitResult, OperationsError> {
    Ok(split_with_evolution(topo, solid, plane_point, plane_normal)?.0)
}

/// [`split`] with construction-derived face evolution for both halves.
///
/// The mapping is recorded while the halves are built — which source face
/// each carried or trimmed face came from — never recovered geometrically
/// after the fact.
///
/// # Errors
///
/// Exactly [`split`]'s errors.
pub fn split_with_evolution(
    topo: &mut Topology,
    solid: SolidId,
    plane_point: Point3,
    plane_normal: Vec3,
) -> Result<(SplitResult, SplitEvolution), OperationsError> {
    let tol = Tolerance::new();
    let normal = plane_normal.normalize()?;
    let d = dot_normal_point(normal, plane_point);

    let (cavities, outer_shell) = {
        let data = topo.solid(solid)?;
        (data.inner_shells().len(), data.outer_shell())
    };
    if cavities > 0 {
        return Err(unsupported(format!(
            "solid has {cavities} cavity shell(s); split only operates on the outer shell"
        )));
    }
    let all_faces: Vec<FaceId> = topo.shell(outer_shell)?.faces().to_vec();

    // Scale-relative slack for "is this point on the plane", following the
    // crate's own `approx_eq` convention. Never loosened to admit a case.
    let eps = tol.linear.max(model_span(topo, &all_faces)? * tol.relative);

    let cuts = classify_edges(topo, &all_faces, normal, d, eps, tol)?;

    let mut positive = Half {
        side: Side::Pos,
        faces: Vec::new(),
        sources: Vec::new(),
        connectors: Vec::new(),
    };
    let mut negative = Half {
        side: Side::Neg,
        faces: Vec::new(),
        sources: Vec::new(),
        connectors: Vec::new(),
    };
    let mut straddled = 0usize;

    for &fid in &all_faces {
        match face_side(topo, fid, &cuts)? {
            // Untouched: surface, curved edges, orientation and every inner
            // wire travel through as topology rather than as positions.
            Some(Side::Pos) => {
                positive.faces.push(fid);
                positive.sources.push(fid.index());
            }
            Some(Side::Neg) => {
                negative.faces.push(fid);
                negative.sources.push(fid.index());
            }
            None => {
                straddled += 1;
                for half in [&mut positive, &mut negative] {
                    let (face, connector) = trim_face(topo, fid, half.side, &cuts, normal, d, eps)?;
                    half.faces.push(face);
                    half.sources.push(fid.index());
                    half.connectors.push(connector);
                }
            }
        }
    }

    if straddled == 0 {
        return Err(OperationsError::InvalidInput {
            reason: "cutting plane does not split the solid (entirely on one side)".into(),
        });
    }

    let mut built = Vec::with_capacity(2);
    for half in [positive, negative] {
        let cap = build_cap(topo, &half.connectors, half.side, normal, d, eps)?;
        let sources = half.sources;
        let specs: Vec<FaceSpec> = half
            .faces
            .into_iter()
            .chain(std::iter::once(cap))
            .map(|face| FaceSpec::Existing { face, outer: None })
            .collect();
        let assembly = assemble_solid_mixed_with_history(topo, &specs, tol)?;
        let assembled = assembly.solid;
        let mut evo = EvolutionMap::exact();
        for (i, out) in assembly.faces_by_spec.iter().enumerate() {
            let Some(out) = out else { continue };
            match sources.get(i) {
                Some(&src) => evo.add_modified(src, out.index()),
                // The cap: synthesised by the operation, not derived from
                // any one input face.
                None => evo.add_unresolved(out.index(), Vec::new()),
            }
        }
        let volume = gate(topo, assembled, half.side)?;
        built.push((assembled, volume, evo));
    }

    // The halves are a decomposition of the input, so this identity is exact
    // up to rounding. It is the strongest single check available here: it
    // catches a filled hole, a dropped face and an inside-out cap alike.
    let input_volume = crate::measure::solid_volume(topo, solid, VOLUME_DEFLECTION)?;
    let sum: f64 = built.iter().map(|(_, v, _)| v).sum();
    let slack = (input_volume.abs() * VOLUME_SUM_RELATIVE_SLACK).max(tol.linear);
    if (sum - input_volume).abs() > slack {
        return Err(unsupported(format!(
            "the two halves enclose {sum} but the body encloses {input_volume}; \
             the split did not conserve the model"
        )));
    }

    if built.len() != 2 {
        return Err(unsupported("split did not produce two halves"));
    }
    let (neg_solid, _, neg_evo) = built.pop().ok_or_else(|| unsupported("missing half"))?;
    let (pos_solid, _, pos_evo) = built.pop().ok_or_else(|| unsupported("missing half"))?;
    Ok((
        SplitResult {
            positive: pos_solid,
            negative: neg_solid,
        },
        SplitEvolution {
            positive: pos_evo,
            negative: neg_evo,
        },
    ))
}

/// Require a half to be a valid solid enclosing positive volume, and return
/// that volume.
fn gate(topo: &Topology, half: SolidId, side: Side) -> Result<f64, OperationsError> {
    let report = crate::validate::validate_solid(topo, half)?;
    if !report.is_valid() {
        let detail: Vec<&str> = report
            .issues
            .iter()
            .filter(|i| i.severity == crate::validate::Severity::Error)
            .map(|i| i.description.as_str())
            .collect();
        return Err(unsupported(format!(
            "the {} half failed validation ({})",
            side.name(),
            detail.join("; ")
        )));
    }
    // A shell can pass the structural checks and still be turned inside out.
    let volume = crate::measure::solid_volume(topo, half, VOLUME_DEFLECTION)?;
    if !volume.is_finite() || volume <= 0.0 {
        return Err(unsupported(format!(
            "the {} half encloses no volume ({volume})",
            side.name()
        )));
    }
    Ok(volume)
}

/// Bounding-box diagonal over every vertex the shell references.
fn model_span(topo: &Topology, all_faces: &[FaceId]) -> Result<f64, OperationsError> {
    let mut bounds: Option<(Point3, Point3)> = None;
    for &fid in all_faces {
        for eid in face_edges(topo, fid)? {
            let edge = topo.edge(eid)?;
            for vid in [edge.start(), edge.end()] {
                let p = topo.vertex(vid)?.point();
                bounds = Some(match bounds {
                    None => (p, p),
                    Some((lo, hi)) => (
                        Point3::new(lo.x().min(p.x()), lo.y().min(p.y()), lo.z().min(p.z())),
                        Point3::new(hi.x().max(p.x()), hi.y().max(p.y()), hi.z().max(p.z())),
                    ),
                });
            }
        }
    }
    Ok(bounds.map_or(0.0, |(lo, hi)| (hi - lo).length()))
}

/// Every edge referenced by `fid`, outer wire first.
fn face_edges(topo: &Topology, fid: FaceId) -> Result<Vec<EdgeId>, OperationsError> {
    let face = topo.face(fid)?;
    let mut ids = Vec::new();
    for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oe in topo.wire(wid)?.edges() {
            ids.push(oe.edge());
        }
    }
    Ok(ids)
}

/// Classify every edge of the shell against the plane, splitting the ones it
/// crosses. Splitting here rather than per-face is what keeps the two faces
/// meeting along a cut edge still meeting along it afterwards.
fn classify_edges(
    topo: &mut Topology,
    all_faces: &[FaceId],
    normal: Vec3,
    d: f64,
    eps: f64,
    tol: Tolerance,
) -> Result<BTreeMap<usize, EdgeCut>, OperationsError> {
    let mut seen: BTreeSet<usize> = BTreeSet::new();
    let mut order: Vec<EdgeId> = Vec::new();
    for &fid in all_faces {
        for eid in face_edges(topo, fid)? {
            if seen.insert(eid.index()) {
                order.push(eid);
            }
        }
    }

    let mut cuts: BTreeMap<usize, EdgeCut> = BTreeMap::new();
    for eid in order {
        let (lo, hi) = edge_range(topo, eid, normal, d)?;
        let cut = if lo >= -eps && hi <= eps {
            // The plane contains this edge. Its two faces would each want it,
            // and only one of them can have it.
            return Err(unsupported(format!(
                "the cutting plane contains edge {} of the solid; splitting along \
                 an existing edge has no unambiguous answer",
                eid.index()
            )));
        } else if lo >= -eps {
            EdgeCut::Whole(Side::Pos)
        } else if hi <= eps {
            EdgeCut::Whole(Side::Neg)
        } else {
            split_edge(topo, eid, normal, d, tol)?
        };
        cuts.insert(eid.index(), cut);
    }
    Ok(cuts)
}

/// The range of signed distance to the cutting plane over the whole of `eid`'s
/// curve — not just its endpoints, which say nothing about an arc that bulges
/// across the plane between them.
fn edge_range(
    topo: &Topology,
    eid: EdgeId,
    normal: Vec3,
    d: f64,
) -> Result<(f64, f64), OperationsError> {
    let edge = topo.edge(eid)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    let at = |p: Point3| dot_normal_point(normal, p) - d;

    Ok(match edge.curve() {
        EdgeCurve::Line => (at(start).min(at(end)), at(start).max(at(end))),
        EdgeCurve::Circle(circle) => {
            let (t0, t1) = edge.domain_with_endpoints(start, end);
            circle_range(circle, normal, d, t0, t1)
        }
        curve => {
            // No closed form to hand: sample the trimmed span densely. A
            // curved edge the plane crosses is refused anyway, so this only
            // has to decide which side an edge clear of the plane is on.
            let (t0, t1) = edge.domain_with_endpoints(start, end);
            let mut lo = f64::INFINITY;
            let mut hi = f64::NEG_INFINITY;
            for i in 0..=RANGE_SAMPLES {
                let t = (t1 - t0).mul_add(i as f64 / RANGE_SAMPLES as f64, t0);
                let x = at(curve.evaluate_with_endpoints(t, start, end));
                lo = lo.min(x);
                hi = hi.max(x);
            }
            (lo, hi)
        }
    })
}

/// Exact signed-distance range of a circular arc: the distance along the arc
/// is `base + amp * cos(t - phase)`, so the extremes are at whichever critical
/// angles fall inside the span, plus its endpoints.
fn circle_range(circle: &Circle3D, normal: Vec3, d: f64, t0: f64, t1: f64) -> (f64, f64) {
    let center = circle.center();
    let base = dot_normal_point(normal, center) - d;
    let radius = circle.radius();
    let u = (circle.evaluate(0.0) - center) * (1.0 / radius);
    let v = (circle.evaluate(std::f64::consts::FRAC_PI_2) - center) * (1.0 / radius);
    let (a, b) = (radius * normal.dot(u), radius * normal.dot(v));
    let amp = a.hypot(b);
    let phase = b.atan2(a);

    let (span_lo, span_hi) = (t0.min(t1), t0.max(t1));
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    let mut consider = |t: f64| {
        let x = amp.mul_add((t - phase).cos(), base);
        lo = lo.min(x);
        hi = hi.max(x);
    };
    consider(span_lo);
    consider(span_hi);
    // The critical angles repeat every pi; step over the ones that could land
    // inside the span (an arc never exceeds a full turn).
    for k in -2..=4 {
        let critical = f64::from(k).mul_add(std::f64::consts::PI, phase);
        if critical >= span_lo && critical <= span_hi {
            consider(critical);
        }
    }
    (lo, hi)
}

/// Split a straight edge at the plane, minting the crossing vertex and the two
/// pieces once. Anything curved is refused: a chord across a trimmed arc is
/// not the same edge, and using one would quietly turn a bore into a facet.
fn split_edge(
    topo: &mut Topology,
    eid: EdgeId,
    normal: Vec3,
    d: f64,
    tol: Tolerance,
) -> Result<EdgeCut, OperationsError> {
    let (start, end, is_line) = {
        let edge = topo.edge(eid)?;
        (
            edge.start(),
            edge.end(),
            matches!(edge.curve(), EdgeCurve::Line),
        )
    };
    if !is_line {
        return Err(unsupported(format!(
            "the cutting plane crosses curved edge {}; trimming a curve to the \
             plane is not implemented here, and a chord across it would not be \
             the same edge",
            eid.index()
        )));
    }

    let p0 = topo.vertex(start)?.point();
    let p1 = topo.vertex(end)?.point();
    let s0 = dot_normal_point(normal, p0) - d;
    let s1 = dot_normal_point(normal, p1) - d;
    let denom = s0 - s1;
    if denom == 0.0 {
        return Err(unsupported(format!(
            "edge {} runs parallel to the cutting plane yet straddles it",
            eid.index()
        )));
    }
    let point = p0 + (p1 - p0) * (s0 / denom);
    let vertex = topo.add_vertex(Vertex::new(point, tol.linear));

    // Keep each piece pointing the way the source edge did, so a wire that
    // traverses the source backwards traverses its piece backwards too.
    let (pos, neg) = if s0 > 0.0 {
        (
            topo.add_edge(Edge::new(start, vertex, EdgeCurve::Line)),
            topo.add_edge(Edge::new(vertex, end, EdgeCurve::Line)),
        )
    } else {
        (
            topo.add_edge(Edge::new(vertex, end, EdgeCurve::Line)),
            topo.add_edge(Edge::new(start, vertex, EdgeCurve::Line)),
        )
    };
    Ok(EdgeCut::Crossed { vertex, pos, neg })
}

/// Which half `fid` belongs to whole, or `None` when the plane trims it.
fn face_side(
    topo: &Topology,
    fid: FaceId,
    cuts: &BTreeMap<usize, EdgeCut>,
) -> Result<Option<Side>, OperationsError> {
    let face = topo.face(fid)?;
    let outer = wire_side(topo, face.outer_wire(), cuts)?;

    for (slot, &wid) in face.inner_wires().iter().enumerate() {
        match (outer, wire_side(topo, wid, cuts)?) {
            (_, None) => {
                return Err(unsupported(format!(
                    "the cutting plane crosses inner wire {slot} of face {}; that \
                     hole would stop being a hole and become a notch in the face's \
                     own boundary, which is not implemented",
                    fid.index()
                )));
            }
            (Some(o), Some(i)) if o != i => {
                return Err(unsupported(format!(
                    "face {} lies wholly on the {} side but its inner wire {slot} \
                     lies on the {} side; a hole and the face it is cut in cannot \
                     land in different halves",
                    fid.index(),
                    o.name(),
                    i.name()
                )));
            }
            _ => {}
        }
    }
    Ok(outer)
}

/// Which half a wire lies on, or `None` when the plane crosses it.
fn wire_side(
    topo: &Topology,
    wid: WireId,
    cuts: &BTreeMap<usize, EdgeCut>,
) -> Result<Option<Side>, OperationsError> {
    let mut seen: Option<Side> = None;
    for oe in topo.wire(wid)?.edges() {
        match cuts.get(&oe.edge().index()) {
            Some(&EdgeCut::Whole(side)) => match seen {
                None => seen = Some(side),
                Some(prev) if prev != side => return Ok(None),
                Some(_) => {}
            },
            _ => return Ok(None),
        }
    }
    Ok(seen)
}

/// One step of a trimmed wire: a piece that survives, or a piece the cut took.
enum Step {
    Kept(OrientedEdge),
    Cut,
}

/// The opening the cut left in a trimmed wire.
struct Gap {
    /// Where the surviving run ends, and where it has to be picked back up.
    from: VertexId,
    to: VertexId,
    /// The one closed rim the cut dropped, and how the wire traversed it.
    dropped_ring: Option<(EdgeId, bool)>,
}

/// Build one half of a face the plane trims, plus the curve that closes it.
#[allow(clippy::too_many_arguments)]
fn trim_face(
    topo: &mut Topology,
    fid: FaceId,
    side: Side,
    cuts: &BTreeMap<usize, EdgeCut>,
    normal: Vec3,
    d: f64,
    eps: f64,
) -> Result<(FaceId, Connector), OperationsError> {
    let (surface, reversed, outer_wire, source_inner) = {
        let face = topo.face(fid)?;
        (
            face.surface().clone(),
            face.is_reversed(),
            face.outer_wire(),
            face.inner_wires().to_vec(),
        )
    };

    let (mut kept, gap) = trim_wire(topo, outer_wire, side, cuts, fid)?;
    let connector = close_gap(topo, fid, &surface, &gap, normal, d, eps)?;
    kept.push(OrientedEdge::new(connector.edge, connector.forward));

    // Inner wires stay whole; `face_side` already refused any the plane cuts.
    let mut inner = Vec::new();
    for wid in source_inner {
        if wire_side(topo, wid, cuts)? == Some(side) {
            inner.push(wid);
        }
    }

    let wire = Wire::new(kept, true).map_err(OperationsError::Topology)?;
    let wid = topo.add_wire(wire);
    let face = if reversed {
        topo.add_face(Face::new_reversed(wid, inner, surface))
    } else {
        topo.add_face(Face::new(wid, inner, surface))
    };
    Ok((face, connector))
}

/// Walk a wire keeping the pieces on `side`, and report the single gap the cut
/// left. More than one gap means the plane cut this face into more than one
/// piece, which this cannot build.
///
/// The walk records where each edge's *dropped* portion sits as well as its
/// kept one, because a rim circle collapses to a single vertex: a wire that
/// loses one still joins up end to end, and vertex adjacency alone would
/// report no gap at all.
fn trim_wire(
    topo: &Topology,
    wid: WireId,
    side: Side,
    cuts: &BTreeMap<usize, EdgeCut>,
    fid: FaceId,
) -> Result<(Vec<OrientedEdge>, Gap), OperationsError> {
    let oriented = topo.wire(wid)?.edges().to_vec();
    let mut steps: Vec<Step> = Vec::with_capacity(oriented.len() * 2);
    let mut dropped_rings: Vec<(EdgeId, bool)> = Vec::new();

    for oe in &oriented {
        let source = oe.edge();
        let cut = *cuts
            .get(&source.index())
            .ok_or_else(|| unsupported(format!("edge {} was never classified", source.index())))?;
        match cut {
            EdgeCut::Whole(s) if s == side => steps.push(Step::Kept(*oe)),
            EdgeCut::Whole(_) => {
                if topo.edge(source)?.is_closed() {
                    dropped_rings.push((source, oe.is_forward()));
                }
                steps.push(Step::Cut);
            }
            EdgeCut::Crossed { vertex, pos, neg } => {
                let piece = match side {
                    Side::Pos => pos,
                    Side::Neg => neg,
                };
                let kept = OrientedEdge::new(piece, oe.is_forward());
                // The kept piece leads if the traversal starts away from the
                // crossing point and runs into it.
                if kept.oriented_start(topo.edge(piece)?) == vertex {
                    steps.push(Step::Cut);
                    steps.push(Step::Kept(kept));
                } else {
                    steps.push(Step::Kept(kept));
                    steps.push(Step::Cut);
                }
            }
        }
    }

    let n = steps.len();
    let is_kept = |i: usize| matches!(steps[i % n], Step::Kept(_));
    // Start at the first surviving step whose predecessor was cut away.
    let Some(start) = (0..n).find(|&i| is_kept(i) && !is_kept(i + n - 1)) else {
        return Err(unsupported(format!(
            "the cutting plane leaves face {} whole on the {} side, yet it was \
             classified as trimmed",
            fid.index(),
            side.name()
        )));
    };

    let mut kept: Vec<OrientedEdge> = Vec::new();
    let mut consumed = 0;
    while consumed < n {
        match &steps[(start + consumed) % n] {
            Step::Kept(oe) => kept.push(*oe),
            Step::Cut => break,
        }
        consumed += 1;
    }
    for offset in consumed..n {
        if is_kept(start + offset) {
            return Err(unsupported(format!(
                "the cutting plane opens more than one gap in the boundary of face \
                 {}; it cuts that face into more than one piece, which is not \
                 implemented",
                fid.index()
            )));
        }
    }

    let (Some(first), Some(last)) = (kept.first(), kept.last()) else {
        return Err(unsupported(format!(
            "the cutting plane leaves nothing of face {} on the {} side",
            fid.index(),
            side.name()
        )));
    };
    let to = first.oriented_start(topo.edge(first.edge())?);
    let from = last.oriented_end(topo.edge(last.edge())?);

    let dropped_ring = match dropped_rings.as_slice() {
        [] => None,
        [only] => Some(*only),
        rings => {
            return Err(unsupported(format!(
                "the cut drops {} closed rims from the boundary of face {}; only \
                 one can be replaced by the section through it",
                rings.len(),
                fid.index()
            )));
        }
    };
    Ok((
        kept,
        Gap {
            from,
            to,
            dropped_ring,
        },
    ))
}

/// Mint the curve the plane cut across a face: a chord between two boundary
/// points, or — where the cut dropped a whole rim — the rim's own section.
///
/// The edge is created once per face and per side, and handed to the cap too,
/// so the trimmed face and the cap that closes against it share it.
fn close_gap(
    topo: &mut Topology,
    fid: FaceId,
    surface: &FaceSurface,
    gap: &Gap,
    normal: Vec3,
    d: f64,
    eps: f64,
) -> Result<Connector, OperationsError> {
    for vid in [gap.from, gap.to] {
        let p = topo.vertex(vid)?.point();
        if (dot_normal_point(normal, p) - d).abs() > eps {
            return Err(unsupported(format!(
                "the boundary of face {} leaves the cut at a point that is not on \
                 the cutting plane; the plane meets that face tangentially",
                fid.index()
            )));
        }
    }

    if gap.from == gap.to {
        let Some((ring, forward)) = gap.dropped_ring else {
            return Err(unsupported(format!(
                "the cut opens a gap in face {} that begins and ends at the same \
                 point without dropping a rim; there is no curve to close it with",
                fid.index()
            )));
        };
        let face_reversed = topo.face(fid)?.is_reversed();
        let edge = section_ring(topo, fid, surface, ring, gap.from, normal, d, eps)?;
        // The section stands in for the rim in place, so the wire traverses it
        // exactly as it traversed the rim.
        return Ok(Connector {
            edge,
            forward,
            ring: true,
            face_reversed,
        });
    }

    if gap.dropped_ring.is_some() {
        return Err(unsupported(format!(
            "the cut drops a closed rim from face {} but reopens its boundary \
             elsewhere; that section is not a simple curve",
            fid.index()
        )));
    }
    if !matches!(surface, FaceSurface::Plane { .. }) {
        return Err(unsupported(format!(
            "the cutting plane crosses face {}, whose surface is not planar; the \
             section across it is not a straight edge and is not implemented",
            fid.index()
        )));
    }
    let face_reversed = topo.face(fid)?.is_reversed();
    Ok(Connector {
        edge: topo.add_edge(Edge::new(gap.from, gap.to, EdgeCurve::Line)),
        forward: true,
        ring: false,
        face_reversed,
    })
}

/// Add a square cylinder section as one seam-anchored full circle after its
/// start, antipode, and end agree with the cut construction.
fn add_certified_section_ring(
    topo: &mut Topology,
    seam_vertex: VertexId,
    circle: Circle3D,
    radial: Vec3,
) -> Result<EdgeId, OperationsError> {
    let vertex = topo.vertex(seam_vertex)?;
    let seam = vertex.point();
    let vertex_tolerance = vertex.tolerance();
    if !vertex_tolerance.is_finite() || vertex_tolerance.is_sign_negative() {
        return Err(unsupported(format!(
            "the section ring seam has invalid vertex tolerance {vertex_tolerance}"
        )));
    }
    let tolerance = vertex_tolerance.max(Tolerance::new().linear);
    let range = (0.0, std::f64::consts::TAU);
    let antipode = circle.center() - radial;
    for (label, parameter, expected) in [
        ("start seam", range.0, seam),
        ("antipode", f64::midpoint(range.0, range.1), antipode),
        ("end seam", range.1, seam),
    ] {
        let residual = (circle.evaluate(parameter) - expected).length();
        if !residual.is_finite() || residual > tolerance {
            return Err(unsupported(format!(
                "the section ring {label} misses its exact oracle by {residual} mm \
                 (tolerance {tolerance} mm)"
            )));
        }
    }

    let mut edge = Edge::with_tolerance(
        seam_vertex,
        seam_vertex,
        EdgeCurve::Circle(circle),
        Some(tolerance),
    );
    edge.set_trim(Some(range));
    edge.strict_domain().map_err(|error| {
        unsupported(format!(
            "the section ring has no authoritative full-turn domain: {error}"
        ))
    })?;
    Ok(topo.add_edge(edge))
}

/// The circle the cutting plane cuts out of a cylindrical wall, standing in
/// for the rim the cut dropped: same axis, same radius, passing through the
/// point where the wall's seam met the plane.
#[allow(clippy::too_many_arguments)]
fn section_ring(
    topo: &mut Topology,
    fid: FaceId,
    surface: &FaceSurface,
    dropped: EdgeId,
    through: VertexId,
    normal: Vec3,
    d: f64,
    eps: f64,
) -> Result<EdgeId, OperationsError> {
    let FaceSurface::Cylinder(cylinder) = surface else {
        return Err(unsupported(format!(
            "the cut drops a closed rim from face {}, whose surface is not a \
             cylinder; the section through it is not a circle",
            fid.index()
        )));
    };
    let EdgeCurve::Circle(rim) = topo.edge(dropped)?.curve().clone() else {
        return Err(unsupported(format!(
            "the cut drops a closed but non-circular rim from face {}; its section \
             is not a circle",
            fid.index()
        )));
    };
    if cylinder.axis().dot(normal).abs() < AXIS_PARALLEL_COSINE
        || rim.normal().dot(normal).abs() < AXIS_PARALLEL_COSINE
    {
        return Err(unsupported(format!(
            "the cutting plane is not square to the axis of cylindrical face {}; \
             its section is an ellipse, which is not implemented",
            fid.index()
        )));
    }

    // Where the cylinder's own axis meets the cutting plane.
    let origin = cylinder.origin();
    let axis = cylinder.axis();
    let center = origin + axis * ((d - dot_normal_point(normal, origin)) / normal.dot(axis));

    let radial = topo.vertex(through)?.point() - center;
    if (radial.length() - rim.radius()).abs() > eps.max(rim.radius() * Tolerance::new().relative) {
        return Err(unsupported(format!(
            "the section of cylindrical face {} does not pass through the point \
             where its seam met the plane",
            fid.index()
        )));
    }
    // `new_with_ref` pins `evaluate(0)` to the seam point, so the closed edge's
    // single vertex really is where its curve starts. Keeping the rim's own
    // axis keeps the section turning the same way the rim did.
    let circle = Circle3D::new_with_ref(center, rim.normal(), rim.radius(), radial)?;
    add_certified_section_ring(topo, through, circle, radial)
}

/// Build the planar cap that closes one half.
///
/// Its boundary is exactly the curves the cut opened in the trimmed faces,
/// traversed the other way round, so cap and wall share every edge. A rim
/// enclosed by the section is a hole in the cap — that is a bore coming
/// through the cut.
fn build_cap(
    topo: &mut Topology,
    connectors: &[Connector],
    side: Side,
    normal: Vec3,
    d: f64,
    eps: f64,
) -> Result<FaceId, OperationsError> {
    // The cap faces out of its own half.
    let (cap_normal, cap_d) = match side {
        Side::Pos => (-normal, -d),
        Side::Neg => (normal, d),
    };

    let mut loops: Vec<Vec<OrientedEdge>> = Vec::new();
    let mut chords: Vec<OrientedEdge> = Vec::new();
    for c in connectors {
        // Opposite to the face's EFFECTIVE traversal (stored sense XOR its
        // reversal flag): that is what makes cap and wall the two manifold
        // partners along this edge.
        let oe = OrientedEdge::new(c.edge, c.forward == c.face_reversed);
        if c.ring {
            loops.push(vec![oe]);
        } else {
            chords.push(oe);
        }
    }
    loops.extend(chain_chords(topo, chords)?);

    if loops.is_empty() {
        return Err(unsupported(format!(
            "the {} half has no cut boundary to cap",
            side.name()
        )));
    }

    let outer_index = outermost_loop(topo, &loops, cap_normal, eps)?;
    let mut outer = None;
    let mut inner = Vec::new();
    for (i, edges) in loops.into_iter().enumerate() {
        let wire = topo.add_wire(Wire::new(edges, true).map_err(OperationsError::Topology)?);
        if i == outer_index {
            outer = Some(wire);
        } else {
            inner.push(wire);
        }
    }
    let Some(outer) = outer else {
        return Err(unsupported("the cut section has no outer loop"));
    };
    Ok(topo.add_face(Face::new(
        outer,
        inner,
        FaceSurface::Plane {
            normal: cap_normal,
            d: cap_d,
        },
    )))
}

/// Chain cut chords head to tail into closed loops.
fn chain_chords(
    topo: &Topology,
    chords: Vec<OrientedEdge>,
) -> Result<Vec<Vec<OrientedEdge>>, OperationsError> {
    let mut remaining = chords;
    let mut loops = Vec::new();
    while let Some(first) = remaining.pop() {
        let start = first.oriented_start(topo.edge(first.edge())?);
        let mut current = first.oriented_end(topo.edge(first.edge())?);
        let mut chain = vec![first];
        while current != start {
            let mut next = None;
            for (i, oe) in remaining.iter().enumerate() {
                if oe.oriented_start(topo.edge(oe.edge())?) == current {
                    next = Some(i);
                    break;
                }
            }
            let Some(i) = next else {
                return Err(unsupported(
                    "the cut section does not close: the chord across one face does \
                     not meet the chord across the next",
                ));
            };
            let oe = remaining.remove(i);
            current = oe.oriented_end(topo.edge(oe.edge())?);
            chain.push(oe);
        }
        loops.push(chain);
    }
    Ok(loops)
}

/// The loop that contains all the others: the cap's outer boundary.
fn outermost_loop(
    topo: &Topology,
    loops: &[Vec<OrientedEdge>],
    cap_normal: Vec3,
    eps: f64,
) -> Result<usize, OperationsError> {
    if loops.len() == 1 {
        return Ok(0);
    }
    let (u_axis, v_axis) = plane_basis(cap_normal);
    let project = |p: Point3| (dot_normal_point(u_axis, p), dot_normal_point(v_axis, p));

    let mut polygons: Vec<Vec<(f64, f64)>> = Vec::with_capacity(loops.len());
    for edges in loops {
        let mut pts = Vec::new();
        for oe in edges {
            let edge = topo.edge(oe.edge())?;
            let start = topo.vertex(edge.start())?.point();
            let end = topo.vertex(edge.end())?.point();
            if matches!(edge.curve(), EdgeCurve::Line) {
                pts.push(project(topo.vertex(oe.oriented_start(edge))?.point()));
            } else {
                let curve = edge.curve();
                let (t0, t1) = edge.domain_with_endpoints(start, end);
                for i in 0..LOOP_SAMPLES {
                    let t = (t1 - t0).mul_add(i as f64 / LOOP_SAMPLES as f64, t0);
                    pts.push(project(curve.evaluate_with_endpoints(t, start, end)));
                }
            }
        }
        polygons.push(pts);
    }

    let mut best = 0;
    for i in 1..polygons.len() {
        if polygon_area(&polygons[i]) > polygon_area(&polygons[best]) {
            best = i;
        }
    }
    for (i, poly) in polygons.iter().enumerate() {
        if i == best {
            continue;
        }
        let Some(&probe) = poly.first() else {
            return Err(unsupported("the cut section produced an empty loop"));
        };
        if !point_in_polygon(probe, &polygons[best], eps) {
            return Err(unsupported(
                "the cut section falls into more than one outer loop; the plane \
                 breaks the body into more than two pieces, which is not \
                 implemented",
            ));
        }
    }
    Ok(best)
}

/// Unsigned area of a 2D polygon.
fn polygon_area(poly: &[(f64, f64)]) -> f64 {
    let mut acc = 0.0;
    for i in 0..poly.len() {
        let j = (i + 1) % poly.len();
        acc += poly[i].0.mul_add(poly[j].1, -(poly[j].0 * poly[i].1));
    }
    acc.abs() * 0.5
}

/// An orthonormal basis of the plane with the given normal.
fn plane_basis(normal: Vec3) -> (Vec3, Vec3) {
    let seed = if normal.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u = normal.cross(seed);
    let len = u.length();
    let u = Vec3::new(u.x() / len, u.y() / len, u.z() / len);
    (u, normal.cross(u))
}

/// Even-odd containment of `p` in `poly`.
fn point_in_polygon(p: (f64, f64), poly: &[(f64, f64)], eps: f64) -> bool {
    let mut inside = false;
    for i in 0..poly.len() {
        let j = (i + poly.len() - 1) % poly.len();
        let (xi, yi) = poly[i];
        let (xj, yj) = poly[j];
        if (yi > p.1) != (yj > p.1) {
            let t = (p.1 - yi) / (yj - yi);
            if (xj - xi).mul_add(t, xi) > p.0 - eps {
                inside = !inside;
            }
        }
    }
    inside
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::tolerance::Tolerance;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::Topology;
    use remus_topology::test_utils::make_unit_cube_manifold;

    use super::*;

    #[test]
    fn split_cube_at_half_height() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        let result = split(
            &mut topo,
            cube,
            Point3::new(0.0, 0.0, 0.5),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();

        let vol_pos = crate::measure::solid_volume(&topo, result.positive, 0.1).unwrap();
        let vol_neg = crate::measure::solid_volume(&topo, result.negative, 0.1).unwrap();

        assert!(
            vol_pos > 0.1,
            "positive half should have volume, got {vol_pos}"
        );
        assert!(
            vol_neg > 0.1,
            "negative half should have volume, got {vol_neg}"
        );

        let tol = Tolerance::loose();
        let total = vol_pos + vol_neg;
        assert!(
            tol.approx_eq(total, 1.0),
            "halves should sum to ~1.0, got {total} ({vol_pos} + {vol_neg})"
        );
    }

    #[test]
    fn split_box_at_quarter() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        // Box extends from (0,0,0) to (2,2,2). Cut at z=0.5 (quarter height).
        let result = split(
            &mut topo,
            solid,
            Point3::new(0.0, 0.0, 0.5),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();

        let vol_pos = crate::measure::solid_volume(&topo, result.positive, 0.1).unwrap();
        let vol_neg = crate::measure::solid_volume(&topo, result.negative, 0.1).unwrap();

        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(vol_pos + vol_neg, 8.0),
            "halves should sum to ~8.0, got {}",
            vol_pos + vol_neg
        );
        assert!(
            tol.approx_eq(vol_pos, 6.0),
            "positive is three quarters, got {vol_pos}"
        );
        assert!(
            tol.approx_eq(vol_neg, 2.0),
            "negative is one quarter, got {vol_neg}"
        );
    }

    #[test]
    fn split_plane_misses_solid() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        // Plane above the cube.
        let result = split(
            &mut topo,
            cube,
            Point3::new(0.0, 0.0, 5.0),
            Vec3::new(0.0, 0.0, 1.0),
        );
        assert!(result.is_err(), "plane above cube should fail");
    }

    #[test]
    fn split_along_x_axis() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        let result = split(
            &mut topo,
            cube,
            Point3::new(0.5, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();

        let vol_pos = crate::measure::solid_volume(&topo, result.positive, 0.1).unwrap();
        let vol_neg = crate::measure::solid_volume(&topo, result.negative, 0.1).unwrap();

        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(vol_pos + vol_neg, 1.0),
            "halves should sum to ~1.0, got {}",
            vol_pos + vol_neg
        );
    }

    /// A plane lying along an edge of the body has no unambiguous answer: the
    /// edge's two faces would each want it and only one can have it.
    #[test]
    fn split_along_an_edge_of_the_body_is_refused() {
        let mut topo = Topology::new();
        let cube = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

        let err = split(
            &mut topo,
            cube,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap_err();
        assert!(
            matches!(err, OperationsError::Unsupported { operation, .. } if operation == "split"),
            "expected a typed refusal, got {err:?}"
        );
    }
}
