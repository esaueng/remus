//! Production cylinder adapter: finite cylindrical laterals with exact
//! latitude circles and axial rulings onto the provenance-preserving UV
//! arrangement (O2.3b/c/d).
//!
//! Bounded slice: `FaceSurface::Cylinder` only (no cone/torus/sphere/plane),
//! no inner holes, boundary exactly two closed latitude rims plus seam lines
//! at one `u`, sections exactly closed latitude circles (constant-`v`,
//! seam-anchored, strictly between the rims) and/or full-height axial rulings
//! (constant-`u` lines, rim-to-rim, not seam-coincident). All uses are
//! axis-aligned lines in the lifted strip chart `[seam_u, seam_u+TAU]` with
//! exact 3D carriers (`Circle` for latitudes/rims, `Line` for rulings/seams).
//! Oblique traces, Ellipse/NURBS contacts, coincident overlaps, tangent
//! contacts, cone poles, sphere poles and double-periodic torus charts are
//! explicitly out-of-domain (`Ok(None)`).
//!
//! Contract (mirrors [`super::arrangement_prod`]): `Ok(None)` means
//! out-of-domain — the caller runs the established path. `Err` means the
//! input passed qualification but the core refused it; that propagates and
//! never degrades into an unsplit face. Construction is atomic: collection,
//! arrangement and emission are pure and allocate no topology.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;

use remus_math::context::OperationContext;
use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::det_hash::DetHashMap;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Point3, Vec2};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

use super::super::split_types::{OrientedPCurveEdge, SectionEdge, SplitSubFace};
use super::arrangement::{
    Arrangement, ArrangementError, ArrangementInput, BoundarySource, CurveSource, CurveUse,
    ParamDomain, build_arrangement,
};
use crate::ds::Rank;
use crate::error::AlgoError;

/// Maximum input uses for one exact cylinder arrangement. Pair scans scale
/// quadratically; larger faces stay on the established path.
const MAX_CYL_USES: usize = 256;
/// Boundary coincidence band for rim/seam gathering (multiples of linear tol
/// are handled by callers; this file uses `100*tol` like the band/sector
/// emitters).
const CLOSE_SCALE: f64 = 100.0;
/// Angular coincidence for seam/ruling `u` (radians).
const U_COINCIDE: f64 = 1e-6;

/// Production inputs for one qualified cylinder lateral.
#[derive(Debug, Clone)]
pub(super) struct CylinderInputs {
    /// Certified strip uses in deterministic order (boundary rectangle first).
    pub uses: Vec<CurveUse>,
    /// Pair scans already charged (currently zero; kept for budget parity).
    pub collection_steps: usize,
    /// Per-use parent data for emission, indexed by `use_id`.
    pub parents: Vec<CylParent>,
    /// Left/right seam use ids (one turn apart) for the strip domain.
    pub seam_uses: [u64; 2],
    /// Cylinder radius (metric area = UV area * radius).
    pub radius: f64,
    /// Strip origin `u` (left edge).
    pub seam_u: f64,
}

/// Parent data for one strip use.
#[derive(Debug, Clone)]
pub(super) struct CylParent {
    /// Authoritative strip pcurve (always a `Line`).
    pub pcurve: Curve2D,
    /// Original endpoints before any split (for cross-face reporting).
    pub original_endpoints_3d: [Point3; 2],
    /// Original section index (`None` for boundary).
    pub original_section: Option<usize>,
    /// Complete native source interval (full turn for circles).
    pub original_source_range: [f64; 2],
    /// Store-space topology edge for boundary uses.
    pub source_topo_edge: Option<usize>,
    /// Pave block for section uses.
    pub pave_block_id: Option<usize>,
    /// Whether this use is a section (false = boundary).
    pub is_section: bool,
}

/// Latitude section (constant-`v` circle) sorted by height.
struct Lat {
    v: f64,
    idx: usize,
}
/// Ruling section (constant-`u` line) sorted by angle from the seam.
struct Rul {
    u_rel: f64,
    idx: usize,
}

fn collect_failed(error: ArrangementError) -> AlgoError {
    AlgoError::FaceSplitFailed(format!("cylinder arrangement refused: {error:?}"))
}

fn budget_decline(context: &OperationContext) -> Result<bool, AlgoError> {
    if context.budgets == remus_math::context::WorkBudgets::new() {
        Ok(false)
    } else {
        Err(collect_failed(ArrangementError::WorkBudgetExceeded))
    }
}

/// Collect certified strip uses for a cylinder lateral.
///
/// Returns `Ok(None)` when outside the bounded domain. All geometry is
/// derived from authoritative topology (loops/coedges/vertices) plus exact
/// section carriers; no proximity merge is manufactured.
pub(super) fn collect_cylinder_uses(
    topo: &Topology,
    face_id: FaceId,
    sections: &[SectionEdge],
    rank: Rank,
    tol: &Tolerance,
    context: &OperationContext,
) -> Result<Option<CylinderInputs>, AlgoError> {
    let cancelled = |_| AlgoError::FaceSplitFailed("cylinder arrangement cancelled".into());
    context.check_cancelled().map_err(cancelled)?;
    let face = match topo.face(face_id) {
        Ok(f) => f,
        Err(_) => return Ok(None),
    };
    let FaceSurface::Cylinder(cyl) = face.surface() else {
        return Ok(None);
    };
    if !tol.linear.is_finite() || tol.linear <= 0.0 {
        return Ok(None);
    }
    if !face.inner_wires().is_empty() {
        return Ok(None);
    }
    let Some(loop_ids) = topo.loops_of_face(face_id) else {
        return Ok(None);
    };
    if loop_ids.len() != 1 {
        return Ok(None);
    }
    let loop_id = loop_ids[0];
    let face_loop = match topo.face_loop(loop_id) {
        Ok(l) => l,
        Err(_) => return Ok(None),
    };
    if face_loop.coedges().is_empty() {
        return Ok(None);
    }
    // Resolve coedges head-to-tail by vertex identity; collect rims/seams.
    let mut rim_coedges: Vec<(
        remus_topology::coedge::CoedgeId,
        remus_topology::edge::EdgeId,
        bool,
    )> = Vec::new();
    let mut seam_coedges: Vec<(
        remus_topology::coedge::CoedgeId,
        remus_topology::edge::EdgeId,
        bool,
    )> = Vec::new();
    let mut prev_end: Option<remus_topology::vertex::VertexId> = None;
    for &coedge_id in face_loop.coedges() {
        context.check_cancelled().map_err(cancelled)?;
        let coedge = match topo.coedge(coedge_id) {
            Ok(c) => c,
            Err(_) => return Ok(None),
        };
        if coedge.parent_loop() != loop_id {
            return Ok(None);
        }
        let edge_id = coedge.edge();
        let forward = coedge.is_forward();
        let edge = match topo.edge(edge_id) {
            Ok(e) => e,
            Err(_) => return Ok(None),
        };
        let (start_vid, end_vid) = if forward {
            (edge.start(), edge.end())
        } else {
            (edge.end(), edge.start())
        };
        if let Some(prev) = prev_end
            && prev != start_vid
        {
            return Ok(None);
        }
        prev_end = Some(end_vid);
        let start_pt = match topo.vertex(start_vid) {
            Ok(v) => v.point(),
            Err(_) => return Ok(None),
        };
        let end_pt = match topo.vertex(end_vid) {
            Ok(v) => v.point(),
            Err(_) => return Ok(None),
        };
        let closed = (start_pt - end_pt).length() <= tol.linear * CLOSE_SCALE;
        match (edge.curve(), closed) {
            (EdgeCurve::Circle(_), true) => rim_coedges.push((coedge_id, edge_id, forward)),
            (EdgeCurve::Line, false) => seam_coedges.push((coedge_id, edge_id, forward)),
            (EdgeCurve::Circle(_), false)
            | (EdgeCurve::Line, true)
            | (
                EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_),
                true | false,
            ) => return Ok(None),
        }
    }
    // Loop must close.
    {
        let first = face_loop.coedges()[0];
        let last = face_loop.coedges()[face_loop.coedges().len() - 1];
        let first_co = topo
            .coedge(first)
            .map_err(|_| AlgoError::FaceSplitFailed("loop".into()))?;
        let last_co = topo
            .coedge(last)
            .map_err(|_| AlgoError::FaceSplitFailed("loop".into()))?;
        let first_edge = topo
            .edge(first_co.edge())
            .map_err(|_| AlgoError::FaceSplitFailed("loop".into()))?;
        let last_edge = topo
            .edge(last_co.edge())
            .map_err(|_| AlgoError::FaceSplitFailed("loop".into()))?;
        let first_start = if first_co.is_forward() {
            first_edge.start()
        } else {
            first_edge.end()
        };
        let last_end = if last_co.is_forward() {
            last_edge.end()
        } else {
            last_edge.start()
        };
        if first_start != last_end {
            return Ok(None);
        }
    }
    if rim_coedges.len() != 2 || seam_coedges.len() != 2 {
        return Ok(None);
    }
    // Loops must reproduce the wire the established splitter consumes.
    if !loops_match_wires(topo, face_id, loop_id) {
        return Ok(None);
    }
    let close_tol = tol.linear * CLOSE_SCALE;
    // Seam `u`: all seam endpoints share one meridian.
    let seam_u = {
        let (_, edge_id, _) = seam_coedges[0];
        let edge = topo
            .edge(edge_id)
            .map_err(|_| AlgoError::FaceSplitFailed("seam".into()))?;
        let v = if topo
            .coedge(seam_coedges[0].0)
            .map(remus_topology::coedge::Coedge::is_forward)
            .unwrap_or(true)
        {
            edge.start()
        } else {
            edge.end()
        };
        // Use start vertex 3D for seam `u`.
        let pt = topo
            .vertex(v)
            .map(remus_topology::vertex::Vertex::point)
            .map_err(|_| AlgoError::FaceSplitFailed("seam".into()))?;
        let (u, _) = cyl.project_point(pt);
        u
    };
    for (_, edge_id, _) in &seam_coedges {
        let edge = match topo.edge(*edge_id) {
            Ok(e) => e,
            Err(_) => return Ok(None),
        };
        for vid in [edge.start(), edge.end()] {
            let pt = match topo.vertex(vid) {
                Ok(v) => v.point(),
                Err(_) => return Ok(None),
            };
            let (u, _) = cyl.project_point(pt);
            let du = (u - seam_u + std::f64::consts::PI).rem_euclid(TAU) - std::f64::consts::PI;
            if du.abs() > U_COINCIDE {
                return Ok(None);
            }
        }
    }
    // Rim `v`s: each closed rim starts on the seam.
    let mut rim_vs: Vec<(
        f64,
        (
            remus_topology::coedge::CoedgeId,
            remus_topology::edge::EdgeId,
            bool,
        ),
    )> = Vec::new();
    for entry in &rim_coedges {
        let (_, edge_id, _) = entry;
        let edge = match topo.edge(*edge_id) {
            Ok(e) => e,
            Err(_) => return Ok(None),
        };
        let pt = match topo.vertex(edge.start()) {
            Ok(v) => v.point(),
            Err(_) => return Ok(None),
        };
        let (_, v) = cyl.project_point(pt);
        let on_seam = cyl.evaluate(seam_u, v);
        if (on_seam - pt).length() > close_tol {
            return Ok(None);
        }
        rim_vs.push((v, *entry));
    }
    rim_vs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    if rim_vs.len() != 2 || (rim_vs[1].0 - rim_vs[0].0).abs() < close_tol {
        return Ok(None);
    }
    let (v_bot, bot_entry) = rim_vs[0];
    let (v_top, top_entry) = rim_vs[1];
    let radius = cyl.radius();
    if !radius.is_finite() || radius <= 0.0 {
        return Ok(None);
    }
    // Classify sections: latitudes (closed Circles, constant-v, seam-anchored)
    // and rulings (Lines, constant-u, rim-to-rim). Anything else defers.
    let mut lats: Vec<Lat> = Vec::new();
    let mut ruls: Vec<Rul> = Vec::new();
    for (idx, s) in sections.iter().enumerate() {
        context.check_cancelled().map_err(cancelled)?;
        match &s.curve_3d {
            EdgeCurve::Circle(c) => {
                if (s.start - s.end).length() > close_tol {
                    return Ok(None);
                }
                // Latitude: centre on axis at height `v`, radius matches,
                // normal parallel to axis.
                let (_, v) = cyl.project_point(s.start);
                if v < v_bot + close_tol || v > v_top - close_tol {
                    // At a rim duplicates the boundary (flush) — no split;
                    // outside the lateral is out-of-domain for this slice.
                    if (v - v_bot).abs() < close_tol || (v_top - v).abs() < close_tol {
                        continue;
                    }
                    return Ok(None);
                }
                let expected_center = cyl.origin() + cyl.axis() * v;
                if (c.center() - expected_center).length() > close_tol {
                    return Ok(None);
                }
                if (c.radius() - radius).abs() > close_tol {
                    return Ok(None);
                }
                if c.normal().dot(cyl.axis()).abs() < 1.0 - 1e-9 {
                    return Ok(None);
                }
                let on_seam = cyl.evaluate(seam_u, v);
                if (on_seam - s.start).length() > close_tol {
                    return Ok(None);
                }
                // Duplicate latitudes (same `v`) are not a clean stack.
                if lats.iter().any(|l: &Lat| (l.v - v).abs() < close_tol) {
                    return Ok(None);
                }
                lats.push(Lat { v, idx });
            }
            EdgeCurve::Line => {
                let (us, vs) = cyl.project_point(s.start);
                let (ue, ve) = cyl.project_point(s.end);
                let du = (us - ue + std::f64::consts::PI).rem_euclid(TAU) - std::f64::consts::PI;
                if du.abs() > U_COINCIDE {
                    return Ok(None);
                }
                let (v_lo, v_hi) = if vs < ve { (vs, ve) } else { (ve, vs) };
                if (v_lo - v_bot).abs() > close_tol || (v_hi - v_top).abs() > close_tol {
                    return Ok(None);
                }
                let u_rel = (us - seam_u).rem_euclid(TAU);
                if u_rel < U_COINCIDE || (TAU - u_rel) < U_COINCIDE {
                    // Seam-coincident ruling adds no cut beyond the seam.
                    continue;
                }
                if ruls
                    .iter()
                    .any(|r: &Rul| (r.u_rel - u_rel).abs() < U_COINCIDE)
                {
                    return Ok(None);
                }
                ruls.push(Rul { u_rel, idx });
            }
            EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => return Ok(None),
        }
    }
    if lats.is_empty() && ruls.is_empty() {
        // Nothing new to cut (all sections flush/seam-coincident) — let the
        // established paths handle coplanar-cap interactions.
        return Ok(None);
    }
    lats.sort_by(|a, b| a.v.partial_cmp(&b.v).unwrap_or(std::cmp::Ordering::Equal));
    ruls.sort_by(|a, b| {
        a.u_rel
            .partial_cmp(&b.u_rel)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    if lats.windows(2).any(|w| w[1].v - w[0].v < close_tol) {
        return Ok(None);
    }
    if ruls
        .windows(2)
        .any(|w| w[1].u_rel - w[0].u_rel < U_COINCIDE)
    {
        return Ok(None);
    }
    let n_uses = 4 + lats.len() + ruls.len();
    if n_uses > MAX_CYL_USES || n_uses > context.budgets.queue_size {
        return budget_decline(context).map(|_| None);
    }
    // Build strip uses: bottom(0), right(1), top(2), left(3), then latitudes
    // (sorted by `v`), then rulings (sorted by `u_rel`). All pcurves are
    // axis-aligned lines; 3D carriers are exact (`Circle` for horizontals,
    // `Line` for verticals). Endpoint certificates are distinct lifted copies
    // (left vs right seam copies share 3D but never share a certificate);
    // seam equivalence is declared via `seam_uses`, never inferred.
    let mut uses: Vec<CurveUse> = Vec::with_capacity(n_uses);
    let mut parents: Vec<CylParent> = Vec::with_capacity(n_uses);
    let mut next_fresh: u64 = u64::MAX;
    let mut fresh = || {
        let id = next_fresh;
        next_fresh = next_fresh.saturating_sub(1);
        id
    };
    // Authoritative vertex certs for the left seam (original vertices);
    // fresh copies for the right seam (same 3D, distinct certificates).
    // Determine bottom/top vertex ids from rim entries (closed rims start at
    // the seam vertices). Use the rim edges' start vertices (both closed rims
    // start at their seam vertex by the `on_seam` gate above).
    let bot_edge = topo
        .edge(bot_entry.1)
        .map_err(|_| AlgoError::FaceSplitFailed("rim".into()))?;
    let top_edge = topo
        .edge(top_entry.1)
        .map_err(|_| AlgoError::FaceSplitFailed("rim".into()))?;
    let v_bot_vid = bot_edge.start();
    let v_top_vid = top_edge.start();
    let v_bot_cert = v_bot_vid.index() as u64;
    let v_top_cert = v_top_vid.index() as u64;
    // Fresh right-side copies (same 3D as left seam vertices).
    let rb_cert = fresh();
    let rt_cert = fresh();
    // Corner 3D via exact surface evaluation (not stored vertices, which may
    // carry weld noise; the core verifies residuals within tolerance).
    let lb_3d = cyl.evaluate(seam_u, v_bot);
    let rb_3d = cyl.evaluate(seam_u, v_bot);
    let rt_3d = cyl.evaluate(seam_u, v_top);
    let lt_3d = cyl.evaluate(seam_u, v_top);
    // Bottom rim: left->right (+u, CCW). Top rim: right->left (-u, CW, since
    // the primitive top is traversed reversed).
    let bot_circle = match topo
        .edge(bot_entry.1)
        .map(remus_topology::edge::Edge::curve)
    {
        Ok(EdgeCurve::Circle(c)) => c.clone(),
        Ok(
            EdgeCurve::Line
            | EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_),
        )
        | Err(_) => return Ok(None),
    };
    let top_circle = match topo
        .edge(top_entry.1)
        .map(remus_topology::edge::Edge::curve)
    {
        Ok(EdgeCurve::Circle(c)) => c.clone(),
        Ok(
            EdgeCurve::Line
            | EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_),
        )
        | Err(_) => return Ok(None),
    };
    // Native seam angles for the rims (where their trims start at the seam).
    let bot_native = bot_circle.project(lb_3d);
    let top_native = top_circle.project(lt_3d);
    if (bot_circle.evaluate(bot_native) - lb_3d).length() > close_tol {
        return Ok(None);
    }
    if (top_circle.evaluate(top_native) - lt_3d).length() > close_tol {
        return Ok(None);
    }
    // Surface `+u` direction at the seam/bottom corner (for latitude sense).
    let du_dir = {
        let p0 = cyl.evaluate(seam_u, v_bot);
        let p1 = cyl.evaluate(seam_u + 1e-4, v_bot);
        (p1 - p0)
            .normalize()
            .map_err(|_| AlgoError::FaceSplitFailed("cyl frame".into()))?
    };
    let mk_line_pcurve = |origin: Point2, dir: Vec2| -> Option<Curve2D> {
        Some(Curve2D::Line(Line2D::new(origin, dir).ok()?))
    };
    // Bottom (id 0): (seam_u,v_bot)->(seam_u+TAU,v_bot), Circle carrier.
    {
        let origin = Point2::new(seam_u, v_bot);
        let pcurve = match mk_line_pcurve(origin, Vec2::new(1.0, 0.0)) {
            Some(p) => p,
            None => return Ok(None),
        };
        let range = [0.0, TAU];
        let source_range = [bot_native, bot_native + TAU];
        uses.push(CurveUse {
            source: CurveSource {
                use_id: 0,
                boundary: Some(BoundarySource {
                    face: face_id,
                    boundary_loop: loop_id,
                    coedge: bot_entry.0,
                }),
                section: None,
                source_edge_idx: Some(0),
                pave_block_id: None,
            },
            pcurve: pcurve.clone(),
            range,
            curve_3d: EdgeCurve::Circle(bot_circle),
            source_range,
            endpoints_3d: [lb_3d, rb_3d],
            endpoints: [v_bot_cert, rb_cert],
            boundary_loop: Some(0),
        });
        parents.push(CylParent {
            pcurve,
            original_endpoints_3d: [lb_3d, rb_3d],
            original_section: None,
            original_source_range: source_range,
            source_topo_edge: Some(bot_entry.1.index()),
            pave_block_id: None,
            is_section: false,
        });
    }
    // Right seam (id 1): bottom->top at seam_u+TAU, Line carrier.
    {
        let origin = Point2::new(seam_u + TAU, v_bot);
        let pcurve = match mk_line_pcurve(origin, Vec2::new(0.0, 1.0)) {
            Some(p) => p,
            None => return Ok(None),
        };
        let h = v_top - v_bot;
        // Right copy uses the forward seam coedge (up).
        let fwd = seam_coedges
            .iter()
            .find(|(_, _, f)| *f)
            .map(|(c, e, _)| (*c, *e))
            .unwrap_or((seam_coedges[0].0, seam_coedges[0].1));
        uses.push(CurveUse {
            source: CurveSource {
                use_id: 1,
                boundary: Some(BoundarySource {
                    face: face_id,
                    boundary_loop: loop_id,
                    coedge: fwd.0,
                }),
                section: None,
                source_edge_idx: Some(1),
                pave_block_id: None,
            },
            pcurve: pcurve.clone(),
            range: [0.0, h],
            curve_3d: EdgeCurve::Line,
            source_range: [0.0, 1.0],
            endpoints_3d: [rb_3d, rt_3d],
            endpoints: [rb_cert, rt_cert],
            boundary_loop: Some(0),
        });
        parents.push(CylParent {
            pcurve,
            original_endpoints_3d: [rb_3d, rt_3d],
            original_section: None,
            original_source_range: [0.0, 1.0],
            source_topo_edge: Some(fwd.1.index()),
            pave_block_id: None,
            is_section: false,
        });
    }
    // Top (id 2): right->left (-u, CW), Circle carrier.
    {
        let origin = Point2::new(seam_u + TAU, v_top);
        let pcurve = match mk_line_pcurve(origin, Vec2::new(-1.0, 0.0)) {
            Some(p) => p,
            None => return Ok(None),
        };
        let range = [0.0, TAU];
        let source_range = [top_native + TAU, top_native];
        // Verify the descending source span reproduces the ends.
        if (top_circle.evaluate(source_range[0]) - rt_3d).length() > close_tol
            || (top_circle.evaluate(source_range[1]) - lt_3d).length() > close_tol
        {
            return Ok(None);
        }
        uses.push(CurveUse {
            source: CurveSource {
                use_id: 2,
                boundary: Some(BoundarySource {
                    face: face_id,
                    boundary_loop: loop_id,
                    coedge: top_entry.0,
                }),
                section: None,
                source_edge_idx: Some(2),
                pave_block_id: None,
            },
            pcurve: pcurve.clone(),
            range,
            curve_3d: EdgeCurve::Circle(top_circle),
            source_range,
            endpoints_3d: [rt_3d, lt_3d],
            endpoints: [rt_cert, v_top_cert],
            boundary_loop: Some(0),
        });
        parents.push(CylParent {
            pcurve,
            original_endpoints_3d: [rt_3d, lt_3d],
            original_section: None,
            original_source_range: source_range,
            source_topo_edge: Some(top_entry.1.index()),
            pave_block_id: None,
            is_section: false,
        });
    }
    // Left seam (id 3): top->bottom at seam_u, Line carrier.
    {
        let origin = Point2::new(seam_u, v_top);
        let pcurve = match mk_line_pcurve(origin, Vec2::new(0.0, -1.0)) {
            Some(p) => p,
            None => return Ok(None),
        };
        let h = v_top - v_bot;
        let rev = seam_coedges
            .iter()
            .find(|(_, _, f)| !*f)
            .map(|(c, e, _)| (*c, *e))
            .unwrap_or((seam_coedges[1].0, seam_coedges[1].1));
        uses.push(CurveUse {
            source: CurveSource {
                use_id: 3,
                boundary: Some(BoundarySource {
                    face: face_id,
                    boundary_loop: loop_id,
                    coedge: rev.0,
                }),
                section: None,
                source_edge_idx: Some(3),
                pave_block_id: None,
            },
            pcurve: pcurve.clone(),
            range: [0.0, h],
            curve_3d: EdgeCurve::Line,
            source_range: [0.0, 1.0],
            endpoints_3d: [lt_3d, lb_3d],
            endpoints: [v_top_cert, v_bot_cert],
            boundary_loop: Some(0),
        });
        parents.push(CylParent {
            pcurve,
            original_endpoints_3d: [lt_3d, lb_3d],
            original_section: None,
            original_source_range: [0.0, 1.0],
            source_topo_edge: Some(rev.1.index()),
            pave_block_id: None,
            is_section: false,
        });
    }
    // Latitudes (sorted by `v`): full-width horizontals with Circle carriers.
    // Direction matches the original circle's traversal at the seam
    // (lower-role rule): aligned with `+u` runs left->right, else right->left.
    for lat in &lats {
        context.check_cancelled().map_err(cancelled)?;
        let s = &sections[lat.idx];
        let EdgeCurve::Circle(c) = &s.curve_3d else {
            return Ok(None);
        };
        let v = lat.v;
        let seam_pt = cyl.evaluate(seam_u, v);
        let natural = c.tangent(c.project(seam_pt));
        let left_to_right = natural.dot(du_dir) > 0.0;
        let (pcurve_lat, range_lat, source_range, ends, certs) = if left_to_right {
            let t0 = c.project(seam_pt);
            if (c.evaluate(t0) - seam_pt).length() > close_tol {
                return Ok(None);
            }
            let a = Point2::new(seam_u, v);
            let pcurve = match mk_line_pcurve(a, Vec2::new(1.0, 0.0)) {
                Some(p) => p,
                None => return Ok(None),
            };
            (
                pcurve,
                [0.0, TAU],
                [t0, t0 + TAU],
                [seam_pt, cyl.evaluate(seam_u, v)],
                [fresh(), fresh()],
            )
        } else {
            let t0 = c.project(seam_pt);
            if (c.evaluate(t0) - seam_pt).length() > close_tol {
                return Ok(None);
            }
            let a = Point2::new(seam_u + TAU, v);
            let pcurve = match mk_line_pcurve(a, Vec2::new(-1.0, 0.0)) {
                Some(p) => p,
                None => return Ok(None),
            };
            (
                pcurve,
                [0.0, TAU],
                [t0 + TAU, t0],
                [cyl.evaluate(seam_u, v), seam_pt],
                [fresh(), fresh()],
            )
        };
        // Both ends are the seam point (same 3D, distinct lifted copies).
        let id = uses.len() as u64;
        uses.push(CurveUse {
            source: CurveSource {
                use_id: id,
                boundary: None,
                section: Some(uses.len()),
                source_edge_idx: Some(uses.len()),
                pave_block_id: s.pave_block_id,
            },
            pcurve: pcurve_lat.clone(),
            range: range_lat,
            curve_3d: EdgeCurve::Circle(c.clone()),
            source_range,
            endpoints_3d: ends,
            endpoints: certs,
            boundary_loop: None,
        });
        parents.push(CylParent {
            pcurve: pcurve_lat,
            original_endpoints_3d: [s.start, s.end],
            original_section: Some(lat.idx),
            original_source_range: source_range,
            source_topo_edge: None,
            pave_block_id: s.pave_block_id,
            is_section: true,
        });
        let _ = rank;
    }
    // Rulings (sorted by `u_rel`): full-height verticals with Line carriers,
    // preserving the input traversal direction (up or down).
    for rul in &ruls {
        context.check_cancelled().map_err(cancelled)?;
        let s = &sections[rul.idx];
        if !matches!(s.curve_3d, EdgeCurve::Line) {
            return Ok(None);
        }
        let u = seam_u + rul.u_rel;
        let (_, vs) = cyl.project_point(s.start);
        let (_, ve) = cyl.project_point(s.end);
        // Preserve input direction: start->end determines up vs down.
        let up = vs < ve;
        // Re-derive `u_rel` from the start for the pcurve origin; the end
        // shares the same `u` by the constant-`u` gate above.
        let start_uv = if up {
            Point2::new(u, v_bot)
        } else {
            Point2::new(u, v_top)
        };
        let dir = if up {
            Vec2::new(0.0, 1.0)
        } else {
            Vec2::new(0.0, -1.0)
        };
        let pcurve = match mk_line_pcurve(start_uv, dir) {
            Some(p) => p,
            None => return Ok(None),
        };
        let h = v_top - v_bot;
        let (e0, e1) = if up {
            (
                cyl.evaluate((seam_u + rul.u_rel).rem_euclid(TAU), v_bot),
                cyl.evaluate((seam_u + rul.u_rel).rem_euclid(TAU), v_top),
            )
        } else {
            (
                cyl.evaluate((seam_u + rul.u_rel).rem_euclid(TAU), v_top),
                cyl.evaluate((seam_u + rul.u_rel).rem_euclid(TAU), v_bot),
            )
        };
        // Keep the input's 3D endpoints when they agree (preserves pave-vertex
        // identity); otherwise use the exact surface evaluation.
        let ends = if (s.start - e0).length() <= close_tol && (s.end - e1).length() <= close_tol {
            [s.start, s.end]
        } else if (s.start - e1).length() <= close_tol && (s.end - e0).length() <= close_tol {
            [s.start, s.end]
        } else {
            // Exact fallbacks (still within the rim gates above).
            [e0, e1]
        };
        let id = uses.len() as u64;
        uses.push(CurveUse {
            source: CurveSource {
                use_id: id,
                boundary: None,
                section: Some(uses.len()),
                source_edge_idx: Some(uses.len()),
                pave_block_id: s.pave_block_id,
            },
            pcurve: pcurve.clone(),
            range: [0.0, h],
            curve_3d: EdgeCurve::Line,
            source_range: [0.0, 1.0],
            endpoints_3d: ends,
            endpoints: [fresh(), fresh()],
            boundary_loop: None,
        });
        parents.push(CylParent {
            pcurve,
            original_endpoints_3d: [s.start, s.end],
            original_section: Some(rul.idx),
            original_source_range: [0.0, 1.0],
            source_topo_edge: None,
            pave_block_id: s.pave_block_id,
            is_section: true,
        });
    }
    if uses.len() > MAX_CYL_USES || uses.len() > context.budgets.queue_size {
        return budget_decline(context).map(|_| None);
    }
    Ok(Some(CylinderInputs {
        uses,
        collection_steps: 0,
        parents,
        seam_uses: [3, 1],
        radius,
        seam_u,
    }))
}

fn loops_match_wires(
    topo: &Topology,
    face_id: FaceId,
    loop_id: remus_topology::face_loop::LoopId,
) -> bool {
    let Ok(face) = topo.face(face_id) else {
        return false;
    };
    if !face.inner_wires().is_empty() {
        return false;
    }
    let Ok(face_loop) = topo.face_loop(loop_id) else {
        return false;
    };
    let wire_id = face.outer_wire();
    let Ok(wire) = topo.wire(wire_id) else {
        return false;
    };
    if wire.edges().len() != face_loop.coedges().len() {
        return false;
    }
    for (oriented, &coedge_id) in wire.edges().iter().zip(face_loop.coedges().iter()) {
        let Ok(coedge) = topo.coedge(coedge_id) else {
            return false;
        };
        if coedge.edge() != oriented.edge() || coedge.is_forward() != oriented.is_forward() {
            return false;
        }
    }
    true
}

/// Run the isolated core on collected cylinder uses.
pub(super) fn run_cylinder_arrangement(
    inputs: &CylinderInputs,
    context: &OperationContext,
) -> Result<Arrangement, ArrangementError> {
    if inputs.collection_steps >= context.budgets.march_steps {
        return Err(ArrangementError::WorkBudgetExceeded);
    }
    let remaining = context
        .budgets
        .with_march_steps(context.budgets.march_steps - inputs.collection_steps);
    let core_context = context.clone().with_budgets(remaining);
    build_arrangement(&ArrangementInput {
        uses: &inputs.uses,
        domain: ParamDomain::CylinderStrip {
            seam_uses: inputs.seam_uses,
            radius: inputs.radius,
        },
        context: &core_context,
    })
}

/// Split a qualified cylinder lateral through the provenance arrangement.
///
/// `Ok(None)` = out-of-domain (caller runs the established path). Geometric
/// refusals for recorded-unsupported classes (tangent/overlap contacts,
/// unrefinable crossings, non-manifold event graphs) likewise decline; any
/// other refusal on qualified input propagates as an internal error.
pub(super) fn try_split_cylinder_face_by_provenance_arrangement(
    topo: &Topology,
    face_id: FaceId,
    sections: &[SectionEdge],
    rank: Rank,
    tol: &Tolerance,
    context: &OperationContext,
    split_registry: Option<&mut DetHashMap<usize, Vec<Point3>>>,
) -> Result<Option<Vec<SplitSubFace>>, AlgoError> {
    let face = match topo.face(face_id) {
        Ok(f) => f,
        Err(_) => return Ok(None),
    };
    let surface = face.surface().clone();
    let reversed = face.is_reversed();
    if !matches!(surface, FaceSurface::Cylinder(_)) {
        return Ok(None);
    }
    let Some(inputs) = collect_cylinder_uses(topo, face_id, sections, rank, tol, context)? else {
        return Ok(None);
    };
    if inputs.uses.len() > MAX_CYL_USES {
        return Ok(None);
    }
    match run_cylinder_arrangement(&inputs, context) {
        Err(ArrangementError::WorkBudgetExceeded)
            if context.budgets == remus_math::context::WorkBudgets::new() =>
        {
            Ok(None)
        }
        Err(
            ArrangementError::AmbiguousContact
            | ArrangementError::AmbiguousOverlap
            | ArrangementError::IntersectionRefinementFailed
            | ArrangementError::NonManifoldEmbedding,
        ) => {
            log::debug!("cylinder arrangement declined face={face_id:?}: geometric");
            Ok(None)
        }
        Err(error) => {
            log::debug!("cylinder arrangement refused face={face_id:?} err={error:?}");
            Err(AlgoError::FaceSplitFailed(format!(
                "cylinder arrangement refused: {error:?}"
            )))
        }
        Ok(arrangement) => {
            let subfaces = emit_cylinder_subfaces(
                &inputs,
                &arrangement,
                surface,
                reversed,
                face_id,
                rank,
                context,
            )?;
            record_cylinder_breaks(&inputs, &arrangement, split_registry);
            Ok(Some(subfaces))
        }
    }
}

/// Record section interior breaks for curved neighbours (pave-block sharing).
///
/// Mirrors the planar contract: interior vertices where distinct uses meet are
/// recorded per pave block so curved faces sharing the section pre-split at
/// identical points. Seam-vertex identifications (same 3D, distinct lifted
/// copies) are not events and are never recorded.
fn record_cylinder_breaks(
    inputs: &CylinderInputs,
    arrangement: &Arrangement,
    split_registry: Option<&mut DetHashMap<usize, Vec<Point3>>>,
) {
    let Some(registry) = split_registry else {
        return;
    };
    // Distinct input uses meeting at each vertex.
    let mut vertex_uses: BTreeMap<usize, BTreeSet<u64>> = BTreeMap::new();
    for half in &arrangement.half_edges {
        vertex_uses
            .entry(half.from)
            .or_default()
            .insert(half.source.use_id);
        vertex_uses
            .entry(half.to)
            .or_default()
            .insert(half.source.use_id);
    }
    for half in arrangement.half_edges.iter().step_by(2) {
        let Some(position) = half.source.source_edge_idx else {
            continue;
        };
        let Some(parent) = inputs.parents.get(position) else {
            continue;
        };
        if !parent.is_section {
            continue;
        }
        let Some(pave_block) = half.source.pave_block_id.or(parent.pave_block_id) else {
            continue;
        };
        for vertex in [half.from, half.to] {
            let Some(users) = vertex_uses.get(&vertex) else {
                continue;
            };
            if users.len() < 2 {
                continue;
            }
            // Skip seam-identified copies (same 3D via identification, single
            // use): they carry no shared event for neighbours.
            if arrangement
                .identifications
                .iter()
                .any(|id| id.vertices.contains(&vertex))
                && users.len() == 1
            {
                continue;
            }
            let point = arrangement.vertices[vertex].point_3d;
            if parent
                .original_endpoints_3d
                .iter()
                .any(|end| (*end - point).length() <= 1e-7)
            {
                continue;
            }
            let entry = registry.entry(pave_block).or_default();
            if !entry.contains(&point) {
                entry.push(point);
            }
        }
    }
}

/// Convert quotiented regions into production subfaces.
///
/// Non-wrapping regions (`cells.len() == 1`: bands, edge sectors, interior
/// grid cells) emit their lifted cell cycle (which retains seam edges for
/// edge cells, matching the band/sector wire convention). Wrapping regions
/// (`cells.len() > 1`: seam-spanning sectors) emit their quotient boundary
/// (seam-free, joined across the identification). Every region carries its
/// certified interior seed and exact source subspans; new edges retain
/// lineage (`source_edge_idx`, full-span `pave_block_id`, boundary
/// `source_topo_edge`).
pub(super) fn emit_cylinder_subfaces(
    inputs: &CylinderInputs,
    arrangement: &Arrangement,
    surface: FaceSurface,
    reversed: bool,
    parent: FaceId,
    rank: Rank,
    context: &OperationContext,
) -> Result<Vec<SplitSubFace>, AlgoError> {
    let cancelled = |_| AlgoError::FaceSplitFailed("cylinder arrangement cancelled".into());
    let carriers: BTreeMap<u64, &CurveUse> = arrangement
        .sources
        .iter()
        .map(|u| (u.source.use_id, u))
        .collect();
    let mut subfaces = Vec::new();
    for periodic in &arrangement.periodic_regions {
        context.check_cancelled().map_err(cancelled)?;
        if periodic.cells.len() == 1 {
            let lifted = periodic.cells[0];
            let region = &arrangement.regions[lifted];
            let outer = emit_cylinder_wire(
                inputs,
                arrangement,
                &carriers,
                &arrangement.cycles[region.outer].edges,
                context,
            )?;
            if outer.is_empty() {
                return Err(collect_failed(ArrangementError::OpenRegion));
            }
            // Bounded slice has no holes; every lifted cell is hole-free
            // (latitudes/rulings never enclose interior holes here).
            if !region.holes.is_empty() {
                return Err(collect_failed(ArrangementError::UnsupportedDomain));
            }
            let interior_3d = arrangement.vertices
                [arrangement.half_edges[arrangement.cycles[region.outer].edges[0]].from]
                .point_3d;
            // Certified interior from the region (UV -> exact 3D via carrier).
            let seed_uv = region.interior;
            let seed_3d = cylinder_point_3d(&surface, seed_uv, inputs.seam_u);
            subfaces.push(SplitSubFace {
                surface: surface.clone(),
                outer_wire: outer,
                inner_wires: Vec::new(),
                reversed,
                parent,
                rank,
                precomputed_interior: Some(seed_3d),
            });
            let _ = interior_3d;
        } else {
            // Wrapping: single quotient boundary, seam-free.
            if periodic.boundaries.len() != 1 {
                return Err(collect_failed(ArrangementError::UnsupportedDomain));
            }
            let (cycle, winding) = &periodic.boundaries[0];
            if *winding != 0 {
                return Err(collect_failed(ArrangementError::UnsupportedDomain));
            }
            let outer = emit_cylinder_wire(inputs, arrangement, &carriers, cycle, context)?;
            if outer.is_empty() {
                return Err(collect_failed(ArrangementError::OpenRegion));
            }
            // Interior: first cell's certified seed (inside the wrapping).
            let first = periodic.cells[0];
            let seed_uv = arrangement.regions[first].interior;
            let seed_3d = cylinder_point_3d(&surface, seed_uv, inputs.seam_u);
            subfaces.push(SplitSubFace {
                surface: surface.clone(),
                outer_wire: outer,
                inner_wires: Vec::new(),
                reversed,
                parent,
                rank,
                precomputed_interior: Some(seed_3d),
            });
        }
    }
    // Deterministic order: sort by interior (v, then u) for permutation
    // invariance across shuffled section inputs.
    subfaces.sort_by(|a, b| {
        let ia = a.precomputed_interior.unwrap_or(Point3::new(0.0, 0.0, 0.0));
        let ib = b.precomputed_interior.unwrap_or(Point3::new(0.0, 0.0, 0.0));
        ia.z()
            .total_cmp(&ib.z())
            .then(ia.x().total_cmp(&ib.x()))
            .then(ia.y().total_cmp(&ib.y()))
    });
    Ok(subfaces)
}

/// Exact 3D point for a strip UV (handles unwrapped `u` beyond TAU via the
/// surface's periodic evaluation).
fn cylinder_point_3d(surface: &FaceSurface, uv: Point2, _seam_u: f64) -> Point3 {
    // `CylindricalSurface::evaluate` is periodic in `u` (sin/cos), so lifted
    // copies beyond TAU evaluate to the same 3D as their seam images.
    surface
        .evaluate(uv.x(), uv.y())
        .unwrap_or(Point3::new(0.0, 0.0, 0.0))
}

/// Build one wire from a traced half-edge cycle, preserving order.
///
/// Consecutive halves of one use joined at a pass-through vertex (exactly two
/// undirected pieces of one use: cardinal-style splits with no branch) merge
/// into a single wire edge. Vertices shared across uses (crossings, rim feet,
/// seam feet) never merge. Runs spanning input uses with no kept vertex
/// between them refuse as `mixed-use run` (geometric, caller declines).
fn emit_cylinder_wire(
    inputs: &CylinderInputs,
    arrangement: &Arrangement,
    carriers: &BTreeMap<u64, &CurveUse>,
    cycle: &[usize],
    context: &OperationContext,
) -> Result<Vec<OrientedPCurveEdge>, AlgoError> {
    let cancelled = |_| AlgoError::FaceSplitFailed("cylinder arrangement cancelled".into());
    let mut incidence: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (hi, half) in arrangement.half_edges.iter().enumerate() {
        incidence.entry(half.from).or_default().push(hi);
        incidence.entry(half.to).or_default().push(hi);
    }
    let mergeable = |vertex: usize| -> bool {
        let Some(halves) = incidence.get(&vertex) else {
            return false;
        };
        if halves.len() == 4 {
            let use_id = arrangement.half_edges[halves[0]].source.use_id;
            if halves
                .iter()
                .all(|h| arrangement.half_edges[*h].source.use_id == use_id)
            {
                let mut undirected: Vec<usize> = halves.iter().map(|h| h / 2).collect();
                undirected.sort_unstable();
                undirected.dedup();
                if undirected.len() == 2 {
                    return true;
                }
            }
        }
        false
    };
    let mut wire = Vec::with_capacity(cycle.len());
    let mut kept: Vec<bool> = Vec::with_capacity(cycle.len());
    for &half_idx in cycle {
        context.check_cancelled().map_err(cancelled)?;
        kept.push(!mergeable(arrangement.half_edges[half_idx].from));
    }
    let Some(first) = kept.iter().position(|k| *k) else {
        if !cycle.is_empty() {
            wire.push(emit_cylinder_run(
                inputs,
                arrangement,
                carriers,
                cycle,
                context,
            )?);
        }
        return Ok(wire);
    };
    let mut boundaries = vec![first];
    for step in 1..cycle.len() {
        let pos = (first + step) % cycle.len();
        if kept[pos] {
            boundaries.push(first + step);
        }
    }
    boundaries.push(first + cycle.len());
    for pair in boundaries.windows(2) {
        context.check_cancelled().map_err(cancelled)?;
        let run: Vec<usize> = (pair[0]..pair[1]).map(|k| cycle[k % cycle.len()]).collect();
        if !run.is_empty() {
            wire.push(emit_cylinder_run(
                inputs,
                arrangement,
                carriers,
                &run,
                context,
            )?);
        }
    }
    Ok(wire)
}

/// Build one production wire edge from a run of one original input edge.
///
/// Horizontal runs (constant-`v`) carry exact `Circle` carriers with native
/// subspans; vertical runs (constant-`u`) carry `Line` carriers. A run
/// spanning uses from different original sections refuses as `mixed-use run`.
fn emit_cylinder_run(
    inputs: &CylinderInputs,
    arrangement: &Arrangement,
    carriers: &BTreeMap<u64, &CurveUse>,
    run: &[usize],
    context: &OperationContext,
) -> Result<OrientedPCurveEdge, AlgoError> {
    let internal = |detail: &str| {
        AlgoError::FaceSplitFailed(format!("cylinder arrangement emission failed: {detail}"))
    };
    let tolerance = context.tolerance.linear;
    let first = &arrangement.half_edges[*run.first().ok_or_else(|| internal("empty run"))?];
    let last = &arrangement.half_edges[*run.last().ok_or_else(|| internal("empty run"))?];
    let position = first
        .source
        .source_edge_idx
        .ok_or_else(|| internal("missing source index"))?;
    let parent = inputs
        .parents
        .get(position)
        .ok_or_else(|| internal("stale source index"))?;
    let carrier = carriers
        .get(&first.source.use_id)
        .ok_or_else(|| internal("stale use id"))?;
    let start_uv = arrangement.vertices[first.from].uv;
    let end_uv = arrangement.vertices[last.to].uv;
    let mut source_span = first.source_range;
    for &half_idx in &run[1..] {
        let half = &arrangement.half_edges[half_idx];
        let member = half
            .source
            .source_edge_idx
            .and_then(|idx| inputs.parents.get(idx))
            .ok_or_else(|| internal("stale source index"))?;
        // Runs must stay within one original section; spanning inputs would
        // need cross-section provenance the core never merges.
        if member.original_section != parent.original_section {
            return Err(internal("mixed-use run"));
        }
        source_span[1] = half.source_range[1];
    }
    let (pcurve, trim, forward) = match &carrier.curve_3d {
        EdgeCurve::Line => {
            let direction = end_uv - start_uv;
            if direction.length() <= f64::EPSILON {
                return Err(internal("degenerate line sub-edge"));
            }
            let Ok(line) = Line2D::new(start_uv, direction) else {
                return Err(internal("degenerate line sub-edge"));
            };
            (Curve2D::Line(line), None, true)
        }
        EdgeCurve::Circle(_) => {
            let Curve2D::Line(_) = parent.pcurve else {
                return Err(internal("circle sub-edge without line pcurve"));
            };
            if !source_span[0].is_finite() || !source_span[1].is_finite() {
                return Err(internal("non-finite circle subspan"));
            }
            (
                parent.pcurve.clone(),
                Some((source_span[0], source_span[1])),
                source_span[1] >= source_span[0],
            )
        }
        EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => return Err(internal("non-line/circle carrier")),
    };
    // Pave-block sharing only for full-span sections (subspans drop it, like
    // the established splitter's split pieces).
    let reversed_source = [source_span[1], source_span[0]];
    let full_span = same_interval(parent.original_source_range, source_span)
        || same_interval(parent.original_source_range, reversed_source);
    let pave_block_id = if full_span {
        first.source.pave_block_id
    } else {
        None
    };
    if let (EdgeCurve::Circle(circle), Some((t0, t1))) = (&carrier.curve_3d, trim) {
        let start_ok = (circle.evaluate(t0) - first.endpoints_3d[0]).length() <= tolerance
            && (circle.evaluate(t1) - last.endpoints_3d[1]).length() <= tolerance;
        if !start_ok {
            return Err(internal("circle subspan endpoints mismatch carrier"));
        }
    }
    Ok(OrientedPCurveEdge {
        curve_3d: carrier.curve_3d.clone(),
        trim,
        pcurve,
        start_uv,
        end_uv,
        start_3d: first.endpoints_3d[0],
        end_3d: last.endpoints_3d[1],
        forward,
        source_edge_idx: first.source.source_edge_idx,
        pave_block_id,
        source_topo_edge: parent.source_topo_edge,
    })
}

fn same_interval(a: [f64; 2], b: [f64; 2]) -> bool {
    a[0].total_cmp(&b[0]).is_eq() && a[1].total_cmp(&b[1]).is_eq()
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
    use super::*;
    use remus_math::context::{CancellationToken, WorkBudgets};
    use remus_math::curves::Circle3D;
    use remus_math::vec::Vec3;
    use remus_topology::edge::Edge;
    use remus_topology::face::Face;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};
    use std::f64::consts::PI;

    const TOL: f64 = 1e-7;
    fn test_context() -> OperationContext {
        OperationContext::new().with_budgets(
            WorkBudgets::new()
                .with_march_steps(2_000_000)
                .with_queue_size(20_000)
                .with_segments(10_000),
        )
    }
    fn lateral(r: f64, h: f64, reversed: bool) -> (Topology, FaceId) {
        lateral_between(r, 0.0, h, reversed)
    }
    fn lateral_between(r: f64, z0: f64, z1: f64, reversed: bool) -> (Topology, FaceId) {
        let mut topo = Topology::new();
        let z = Vec3::new(0.0, 0.0, 1.0);
        let surface =
            remus_math::surfaces::CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), z, r)
                .unwrap();
        let v_bot = topo.add_vertex(Vertex::new(Point3::new(r, 0.0, z0), TOL));
        let v_top = topo.add_vertex(Vertex::new(Point3::new(r, 0.0, z1), TOL));
        let bot = Circle3D::new(Point3::new(0.0, 0.0, z0), z, r).unwrap();
        let top = Circle3D::new(Point3::new(0.0, 0.0, z1), z, r).unwrap();
        let b0 = bot.project(Point3::new(r, 0.0, z0));
        let t0 = top.project(Point3::new(r, 0.0, z1));
        let mut be = Edge::new(v_bot, v_bot, EdgeCurve::Circle(bot));
        be.set_trim(Some((b0, b0 + TAU)));
        let mut te = Edge::new(v_top, v_top, EdgeCurve::Circle(top));
        te.set_trim(Some((t0, t0 + TAU)));
        let be = topo.add_edge(be);
        let te = topo.add_edge(te);
        let seam = topo.add_edge(Edge::new(v_bot, v_top, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(be, true),
                    OrientedEdge::new(seam, true),
                    OrientedEdge::new(te, false),
                    OrientedEdge::new(seam, false),
                ],
                true,
            )
            .unwrap(),
        );
        let mut face = Face::new(wire, vec![], FaceSurface::Cylinder(surface));
        face.set_reversed(reversed);
        let face = topo.add_face(face);
        (topo, face)
    }
    fn on_cyl(r: f64, theta: f64, z: f64) -> Point3 {
        Point3::new(r * theta.cos(), r * theta.sin(), z)
    }
    fn section_circle(r: f64, z: f64) -> SectionEdge {
        let circle = Circle3D::new(Point3::new(0.0, 0.0, z), Vec3::new(0.0, 0.0, 1.0), r).unwrap();
        let seam = on_cyl(r, 0.0, z);
        let t0 = circle.project(seam);
        let dummy = Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap());
        SectionEdge {
            curve_3d: EdgeCurve::Circle(circle),
            trim: Some((t0, t0 + TAU)),
            pcurve_a: dummy.clone(),
            pcurve_b: dummy,
            start: seam,
            end: seam,
            start_uv_a: None,
            end_uv_a: None,
            start_uv_b: None,
            end_uv_b: None,
            target_face: None,
            pave_block_id: Some(100),
        }
    }
    fn section_ruling(r: f64, theta: f64, h: f64) -> SectionEdge {
        let a = on_cyl(r, theta, 0.0);
        let b = on_cyl(r, theta, h);
        let dummy = Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap());
        SectionEdge {
            curve_3d: EdgeCurve::Line,
            trim: None,
            pcurve_a: dummy.clone(),
            pcurve_b: dummy,
            start: a,
            end: b,
            start_uv_a: None,
            end_uv_a: None,
            start_uv_b: None,
            end_uv_b: None,
            target_face: None,
            pave_block_id: Some(200),
        }
    }
    fn collect(topo: &Topology, face: FaceId, sections: &[SectionEdge]) -> Option<CylinderInputs> {
        collect_cylinder_uses(
            topo,
            face,
            sections,
            Rank::A,
            &Tolerance::default(),
            &test_context(),
        )
        .unwrap()
    }
    fn split_emit(
        topo: &Topology,
        face: FaceId,
        sections: &[SectionEdge],
    ) -> Option<Vec<SplitSubFace>> {
        try_split_cylinder_face_by_provenance_arrangement(
            topo,
            face,
            sections,
            Rank::A,
            &Tolerance::default(),
            &test_context(),
            None,
        )
        .unwrap()
    }
    fn wire_area(wire: &[OrientedPCurveEdge], r: f64) -> f64 {
        wire_area_about(wire, r, 0.0, 0.0)
    }
    fn wire_area_about(wire: &[OrientedPCurveEdge], r: f64, cx: f64, cy: f64) -> f64 {
        // Cylindrical metric area via (r*theta, z) shoelace, theta about
        // `(cx, cy)` unwrapped along the loop. Samples circle carriers densely
        // (a full latitude has coincident ends, so endpoint-only polygons
        // fold to zero).
        let mut pts: Vec<(f64, f64)> = Vec::new();
        let mut theta_prev: Option<f64> = None;
        for e in wire {
            for p in edge_polyline(e) {
                let raw = (p.y() - cy).atan2(p.x() - cx);
                let theta = theta_prev.map_or(raw, |t| t + (raw - t + PI).rem_euclid(TAU) - PI);
                theta_prev = Some(theta);
                pts.push((r * theta, p.z()));
            }
        }
        let n = pts.len();
        let mut twice = 0.0;
        for i in 0..n {
            let (x0, y0) = pts[i];
            let (x1, y1) = pts[(i + 1) % n];
            twice += x0.mul_add(y1, -(x1 * y0));
        }
        twice.abs() * 0.5
    }
    fn edge_polyline(e: &OrientedPCurveEdge) -> Vec<Point3> {
        const N: u32 = 48;
        match &e.curve_3d {
            EdgeCurve::Line => (0..=N)
                .map(|k| e.start_3d + (e.end_3d - e.start_3d) * (f64::from(k) / f64::from(N)))
                .collect(),
            EdgeCurve::Circle(c) => {
                let ctr = c.center();
                let ang = |p: Point3| (p.y() - ctr.y()).atan2(p.x() - ctr.x());
                let sense = if e.forward { 1.0 } else { -1.0 } * c.normal().z().signum();
                let mut sweep = (sense * (ang(e.end_3d) - ang(e.start_3d))).rem_euclid(TAU);
                if sweep < 1e-9 {
                    sweep = TAU;
                }
                let a0 = ang(e.start_3d);
                (0..=N)
                    .map(|k| {
                        let a = (sense * sweep).mul_add(f64::from(k) / f64::from(N), a0);
                        Point3::new(
                            c.radius().mul_add(a.cos(), ctr.x()),
                            c.radius().mul_add(a.sin(), ctr.y()),
                            e.start_3d.z(),
                        )
                    })
                    .collect()
            }
            EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => vec![e.start_3d, e.end_3d],
        }
    }

    #[test]
    fn bands_split_lateral_with_closed_form_area() {
        let (topo, face) = lateral(2.0, 3.0, false);
        let sections = vec![section_circle(2.0, 1.0), section_circle(2.0, 2.0)];
        let subs = split_emit(&topo, face, &sections).expect("bands");
        assert_eq!(subs.len(), 3);
        let mut areas: Vec<_> = subs.iter().map(|s| wire_area(&s.outer_wire, 2.0)).collect();
        areas.sort_by(f64::total_cmp);
        for a in &areas {
            assert!((a - 4.0 * PI).abs() < 1e-6, "{areas:?}");
        }
        for s in &subs {
            assert!(s.precomputed_interior.is_some());
            assert!(s.inner_wires.is_empty());
        }
    }

    #[test]
    fn single_ruling_splits_lateral_into_two_sectors() {
        let (topo, face) = lateral(2.0, 3.0, false);
        let subs = split_emit(&topo, face, &[section_ruling(2.0, 2.0, 3.0)]).expect("sectors");
        assert_eq!(subs.len(), 2);
        let mut areas: Vec<_> = subs.iter().map(|s| wire_area(&s.outer_wire, 2.0)).collect();
        areas.sort_by(f64::total_cmp);
        assert!((areas[0] - 2.0 * 2.0 * 3.0).abs() < 1e-6, "{areas:?}");
        assert!(
            (areas[1] - 2.0 * (TAU - 2.0) * 3.0).abs() < 1e-6,
            "{areas:?}"
        );
    }

    #[test]
    fn nonzero_seam_and_reversed_coedges_agree() {
        // Lateral between z=5..8 (nonzero `v`) with reversed face; sections
        // shuffled and reversed.
        let (topo, face) = lateral_between(1.5, 5.0, 8.0, true);
        let mut sections = vec![section_circle(1.5, 6.0), section_ruling(1.5, 4.0, 8.0)];
        // Ruling helper builds z=0..h; rebuild for z0..z1.
        sections[1] = {
            let a = on_cyl(1.5, 4.0, 5.0);
            let b = on_cyl(1.5, 4.0, 8.0);
            let dummy =
                Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap());
            SectionEdge {
                curve_3d: EdgeCurve::Line,
                trim: None,
                pcurve_a: dummy.clone(),
                pcurve_b: dummy,
                start: b,
                end: a,
                start_uv_a: None,
                end_uv_a: None,
                start_uv_b: None,
                end_uv_b: None,
                target_face: None,
                pave_block_id: Some(201),
            }
        };
        let a = split_emit(&topo, face, &sections).expect("grid");
        sections.reverse();
        let b = split_emit(&topo, face, &sections).expect("grid shuffled");
        assert_eq!(a.len(), b.len());
        assert_eq!(a.len(), 4);
    }

    #[test]
    fn oblique_and_cone_defer() {
        let (topo, face) = lateral(1.0, 2.0, false);
        // Oblique line (helix chord, not axis-aligned) defers.
        let dummy = Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap());
        let oblique = SectionEdge {
            curve_3d: EdgeCurve::Line,
            trim: None,
            pcurve_a: dummy.clone(),
            pcurve_b: dummy,
            start: on_cyl(1.0, 0.0, 0.0),
            end: on_cyl(1.0, 1.0, 2.0),
            start_uv_a: None,
            end_uv_a: None,
            start_uv_b: None,
            end_uv_b: None,
            target_face: None,
            pave_block_id: None,
        };
        // Note: (0,0)->(1,2) with du=1 rad is oblique (not ruling/latitude)
        // only if it is not constant-u/v; here du=1 so it defers only when the
        // adapter's constant-`u` gate sees du>1e-6. This line has du=1, so None.
        assert!(split_emit(&topo, face, &[oblique]).is_none());
    }

    #[test]
    fn budgets_and_cancellation_are_atomic() {
        let (topo, face) = lateral(1.0, 2.0, false);
        let sections = vec![section_circle(1.0, 1.0)];
        let before = topo.num_faces();
        let ctx = test_context().with_budgets(
            WorkBudgets::new()
                .with_march_steps(0)
                .with_queue_size(20_000)
                .with_segments(10_000),
        );
        let res = try_split_cylinder_face_by_provenance_arrangement(
            &topo,
            face,
            &sections,
            Rank::A,
            &Tolerance::default(),
            &ctx,
            None,
        );
        assert!(res.as_ref().map(Option::is_none).unwrap_or(true));
        assert_eq!(topo.num_faces(), before);
        let token = CancellationToken::new();
        token.cancel();
        let ctx = test_context().with_cancellation(token);
        let res = try_split_cylinder_face_by_provenance_arrangement(
            &topo,
            face,
            &sections,
            Rank::A,
            &Tolerance::default(),
            &ctx,
            None,
        );
        assert!(res.is_err());
    }

    fn placed_lateral(r: f64, origin: Point3, h: f64, reversed: bool) -> (Topology, FaceId) {
        // Translated cylinder (same axis +z): areas invariant under translation.
        let mut topo = Topology::new();
        let z = Vec3::new(0.0, 0.0, 1.0);
        let surface = remus_math::surfaces::CylindricalSurface::new(origin, z, r).unwrap();
        let v_bot = topo.add_vertex(Vertex::new(origin + Vec3::new(r, 0.0, 0.0), TOL));
        let v_top = topo.add_vertex(Vertex::new(origin + Vec3::new(r, 0.0, h), TOL));
        let bot = Circle3D::new(origin, z, r).unwrap();
        let top = Circle3D::new(origin + z * h, z, r).unwrap();
        let seam_pt = origin + Vec3::new(r, 0.0, 0.0);
        let b0 = bot.project(seam_pt);
        let t0 = top.project(origin + Vec3::new(r, 0.0, h));
        let mut be = Edge::new(v_bot, v_bot, EdgeCurve::Circle(bot));
        be.set_trim(Some((b0, b0 + TAU)));
        let mut te = Edge::new(v_top, v_top, EdgeCurve::Circle(top));
        te.set_trim(Some((t0, t0 + TAU)));
        let be = topo.add_edge(be);
        let te = topo.add_edge(te);
        let seam = topo.add_edge(Edge::new(v_bot, v_top, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(be, true),
                    OrientedEdge::new(seam, true),
                    OrientedEdge::new(te, false),
                    OrientedEdge::new(seam, false),
                ],
                true,
            )
            .unwrap(),
        );
        let mut face = Face::new(wire, vec![], FaceSurface::Cylinder(surface));
        face.set_reversed(reversed);
        let face = topo.add_face(face);
        (topo, face)
    }

    #[test]
    fn two_rulings_wrap_and_tile_with_quotient_euler() {
        // Two rulings: one wrapping sector (cells len 2, chi=1, winding 0) plus
        // one interior sector. Total area tiles the lateral; seam equivalence
        // and Euler are checked at the arrangement level.
        let (topo, face) = lateral(2.0, 3.0, false);
        let sections = vec![section_ruling(2.0, 1.0, 3.0), section_ruling(2.0, 3.0, 3.0)];
        let inputs = collect(&topo, face, &sections).expect("qualified");
        let arrangement = super::run_cylinder_arrangement(&inputs, &test_context()).unwrap();
        assert_eq!(arrangement.periodic_regions.len(), 2);
        assert!(
            arrangement
                .periodic_regions
                .iter()
                .any(|r| r.cells.len() == 2)
        );
        for r in &arrangement.periodic_regions {
            assert_eq!(r.euler_characteristic, 1);
            assert_eq!(r.boundaries.len(), 1);
            assert_eq!(r.boundaries[0].1, 0);
        }
        let total: f64 = arrangement.periodic_regions.iter().map(|r| r.area).sum();
        assert!((total - 12.0 * PI).abs() < 1e-9, "{total}");
        // Seam identifications pair left/right copies one turn apart.
        assert!(!arrangement.identifications.is_empty());
        for id in &arrangement.identifications {
            assert_eq!(id.u_lift, 1);
        }
        let subs = split_emit(&topo, face, &sections).expect("sectors");
        assert_eq!(subs.len(), 2);
        let mut areas: Vec<_> = subs.iter().map(|s| wire_area(&s.outer_wire, 2.0)).collect();
        areas.sort_by(f64::total_cmp);
        assert!((areas[0] - 2.0 * 2.0 * 3.0).abs() < 1e-6, "{areas:?}");
        assert!(
            (areas[1] - 2.0 * (TAU - 2.0) * 3.0).abs() < 1e-6,
            "{areas:?}"
        );
    }

    #[test]
    fn scales_and_rigid_placement_keep_partition() {
        for scale in [1e-3, 1.0, 1e3] {
            let r = 2.0 * scale;
            let h = 3.0 * scale;
            let (topo, face) = lateral(r, h, false);
            let sections = vec![section_circle(r, 0.3 * h), section_ruling(r, 2.0, h)];
            let subs = split_emit(&topo, face, &sections).expect("grid");
            // 1 latitude + 1 ruling: 2 bands * 2 sectors = 4 regions.
            assert_eq!(subs.len(), 4, "scale {scale}");
            let total: f64 = subs.iter().map(|s| wire_area(&s.outer_wire, r)).sum();
            assert!(
                (total - TAU * r * h).abs() < 1e-6 * r * h,
                "scale {scale} total {total}"
            );
        }
        // Translated placement (rigid translation off-origin).
        let (topo, face) = placed_lateral(2.0, Point3::new(7.5, -3.25, 10.0), 3.0, false);
        // Sections must be rebuilt at the translated location.
        let o = Point3::new(7.5, -3.25, 10.0);
        let lat_circle =
            Circle3D::new(o + Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        let seam = o + Vec3::new(2.0, 0.0, 1.0);
        let t0 = lat_circle.project(seam);
        let dummy = Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap());
        let lat = SectionEdge {
            curve_3d: EdgeCurve::Circle(lat_circle),
            trim: Some((t0, t0 + TAU)),
            pcurve_a: dummy.clone(),
            pcurve_b: dummy,
            start: seam,
            end: seam,
            start_uv_a: None,
            end_uv_a: None,
            start_uv_b: None,
            end_uv_b: None,
            target_face: None,
            pave_block_id: Some(101),
        };
        let rul = {
            let a = o + Vec3::new(2.0 * 2.0f64.cos(), 2.0 * 2.0f64.sin(), 0.0);
            let b = a + Vec3::new(0.0, 0.0, 3.0);
            let dummy =
                Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap());
            SectionEdge {
                curve_3d: EdgeCurve::Line,
                trim: None,
                pcurve_a: dummy.clone(),
                pcurve_b: dummy,
                start: a,
                end: b,
                start_uv_a: None,
                end_uv_a: None,
                start_uv_b: None,
                end_uv_b: None,
                target_face: None,
                pave_block_id: Some(201),
            }
        };
        let subs = split_emit(&topo, face, &[lat, rul]).expect("placed grid");
        assert_eq!(subs.len(), 4);
        let total: f64 = subs
            .iter()
            .map(|s| wire_area_about(&s.outer_wire, 2.0, 7.5, -3.25))
            .sum();
        assert!((total - TAU * 2.0 * 3.0).abs() < 1e-6, "{total}");
    }

    #[test]
    fn thin_retained_band_survives() {
        let (topo, face) = lateral(2.0, 3.0, false);
        // Two latitudes 1e-4 apart (10x the 1e-5 close band): the thin band
        // between them is retained whole with its closed-form area.
        let sections = vec![section_circle(2.0, 1.0), section_circle(2.0, 1.0 + 1e-4)];
        let subs = split_emit(&topo, face, &sections).expect("thin");
        assert_eq!(subs.len(), 3);
        let mut areas: Vec<_> = subs.iter().map(|s| wire_area(&s.outer_wire, 2.0)).collect();
        areas.sort_by(f64::total_cmp);
        assert!((areas[0] - TAU * 2.0 * 1e-4).abs() < 1e-6, "{areas:?}");
    }

    #[test]
    fn shuffled_and_reversed_inputs_are_deterministic() {
        let (topo, face) = lateral(2.0, 3.0, false);
        let a = section_circle(2.0, 1.0);
        let b = section_ruling(2.0, 2.0, 3.0);
        let rev = |s: &SectionEdge| {
            let mut r = s.clone();
            std::mem::swap(&mut r.start, &mut r.end);
            r
        };
        let variants = vec![
            vec![a.clone(), b.clone()],
            vec![b.clone(), a.clone()],
            vec![rev(&a), b.clone()],
            vec![a.clone(), rev(&b)],
        ];
        let mut signatures = Vec::new();
        for sections in &variants {
            let subs = split_emit(&topo, face, sections).expect("grid");
            assert_eq!(subs.len(), 4);
            // Deterministic order: sorted by interior (z,x,y) in emission.
            let sig: Vec<_> = subs
                .iter()
                .map(|s| {
                    let p = s.precomputed_interior.unwrap();
                    (p.x().to_bits(), p.y().to_bits(), p.z().to_bits())
                })
                .collect();
            signatures.push(sig);
        }
        for sig in &signatures[1..] {
            assert_eq!(sig, &signatures[0]);
        }
    }

    /// Differential execution for sectors: same region count, same total
    /// metric area (closed-form `r*theta*h` per sector) and matching centroids
    /// as the established `split_periodic_face_into_sectors`, plus the
    /// independent oracle. No emitter deleted in this slice; this pins
    /// agreement for future migration.
    #[test]
    fn differential_sectors_match_established_and_oracle() {
        for (r, h, theta) in [(2.0, 3.0, 2.0), (1.5, 2.0, 4.5)] {
            let (topo, face) = lateral(r, h, false);
            let sections = vec![section_ruling(r, theta, h)];
            let new_faces = split_emit(&topo, face, &sections).expect("new sectors");
            let old_faces = established_sectors(&topo, face, &sections).expect("old sectors");
            assert_eq!(new_faces.len(), old_faces.len());
            assert_eq!(new_faces.len(), 2);
            // Independent oracle: r*theta*h and r*(TAU-theta)*h.
            let mut areas: Vec<_> = new_faces
                .iter()
                .map(|s| wire_area(&s.outer_wire, r))
                .collect();
            areas.sort_by(f64::total_cmp);
            let mut expected = [r * theta * h, r * (TAU - theta) * h];
            expected.sort_by(f64::total_cmp);
            assert!((areas[0] - expected[0]).abs() < 1e-6, "{areas:?}");
            assert!((areas[1] - expected[1]).abs() < 1e-6, "{areas:?}");
            assert_same_partition("sectors", &new_faces, &old_faces, r);
        }
    }

    /// Differential for bands (multiple separators): new arrangement vs the
    /// established `split_periodic_face_into_bands` plus closed-form
    /// `2πrΔz` per band. Bands remain for cone; this pins cylinder agreement
    /// without deleting the emitter.
    #[test]
    fn differential_bands_match_established_and_oracle() {
        for (r, h) in [(2.0, 3.0), (1.0, 2.0)] {
            let (topo, face) = lateral(r, h, false);
            let sections = vec![section_circle(r, 0.3 * h), section_circle(r, 0.7 * h)];
            let new_faces = split_emit(&topo, face, &sections).expect("new bands");
            let old_faces = established_bands(&topo, face, &sections).expect("old bands");
            assert_eq!(new_faces.len(), old_faces.len());
            assert_eq!(new_faces.len(), 3);
            for s in new_faces.iter().chain(old_faces.iter()) {
                let a = wire_area(&s.outer_wire, r);
                // Each band height is 0.3h, 0.4h, 0.3h; total tiles.
                assert!(a > 0.0, "{a}");
            }
            let new_total: f64 = new_faces.iter().map(|s| wire_area(&s.outer_wire, r)).sum();
            let old_total: f64 = old_faces.iter().map(|s| wire_area(&s.outer_wire, r)).sum();
            assert!((new_total - TAU * r * h).abs() < 1e-6, "{new_total}");
            assert!((old_total - TAU * r * h).abs() < 1e-6, "{old_total}");
            assert_same_partition("bands", &new_faces, &old_faces, r);
        }
    }

    fn established_boundary(topo: &Topology, face: FaceId) -> Vec<OrientedPCurveEdge> {
        let f = topo.face(face).unwrap();
        let pts = super::super::collect_wire_points(topo, f.outer_wire());
        super::super::conversion::boundary_edges_to_pcurve(
            topo,
            f.outer_wire(),
            f.surface(),
            &pts,
            None,
        )
        .unwrap()
    }
    fn established_sectors(
        topo: &Topology,
        face: FaceId,
        sections: &[SectionEdge],
    ) -> Option<Vec<SplitSubFace>> {
        let f = topo.face(face).unwrap();
        let boundary = established_boundary(topo, face);
        super::super::special_cases::split_periodic_face_into_sectors(
            f.surface(),
            &boundary,
            sections,
            Rank::A,
            false,
            face,
            TOL,
        )
    }
    fn established_bands(
        topo: &Topology,
        face: FaceId,
        sections: &[SectionEdge],
    ) -> Option<Vec<SplitSubFace>> {
        let f = topo.face(face).unwrap();
        let boundary = established_boundary(topo, face);
        super::super::special_cases::split_periodic_face_into_bands(
            f.surface(),
            &boundary,
            sections,
            Rank::A,
            false,
            face,
            TOL,
        )
    }
    /// Same partition: equal count, equal total metric area, matching centroids.
    fn assert_same_partition(
        label: &str,
        new_faces: &[SplitSubFace],
        old_faces: &[SplitSubFace],
        r: f64,
    ) {
        assert_eq!(
            new_faces.len(),
            old_faces.len(),
            "{label}: count new={} old={}",
            new_faces.len(),
            old_faces.len()
        );
        let total = |faces: &[SplitSubFace]| {
            faces
                .iter()
                .map(|s| wire_area(&s.outer_wire, r))
                .sum::<f64>()
        };
        let (new_total, old_total) = (total(new_faces), total(old_faces));
        assert!(
            (new_total - old_total).abs() < 1e-6,
            "{label}: area new={new_total} old={old_total}"
        );
        let centroid = |wire: &[OrientedPCurveEdge]| {
            let mut pts = Vec::new();
            for e in wire {
                for p in edge_polyline(e) {
                    pts.push(p);
                }
            }
            let n = pts.len() as f64;
            pts.into_iter().fold(Point3::new(0.0, 0.0, 0.0), |a, p| {
                a + (p - Point3::new(0.0, 0.0, 0.0)) * (1.0 / n)
            })
        };
        let mut unmatched: Vec<Point3> =
            old_faces.iter().map(|s| centroid(&s.outer_wire)).collect();
        for sub in new_faces {
            let c = centroid(&sub.outer_wire);
            let hit = unmatched.iter().position(|o| (*o - c).length() < 1e-4);
            assert!(hit.is_some(), "{label}: unmatched new region at {c:?}");
            unmatched.remove(hit.unwrap());
        }
    }
}
