//! Torus and cylinder faces cut open along their seams and partitioned as a
//! planar arrangement in the lifted `(u, v)` chart.
//!
//! A face whose boundary is made of iso-parameter curves and which is closed
//! in at least one direction (see [`crate::periodic_chart`]) — a swept or
//! imported tube bounded by its two end circles and a seam, a whole torus
//! bounded only by its two seam circles, a cylinder lateral bounded by its
//! rims and seam line — lifts to an axis-aligned rectangle whose periodic
//! sides are copies of one seam edge. Torus faces of that family take this
//! path first; a cylinder lateral takes it only when its established
//! partition fails (see `split_face_2d`).
//! The band, sector and internal-loop emitters each assume one layout of the
//! sections on such a face; sections that wrap the tube, cross a seam and
//! also end on each other (a plane and a cylinder cutting the same tube,
//! OpenZCAD issues 1242 and 1244) match none of them and were dropped as
//! orphaned arcs.
//!
//! Here the sections are cut exactly where they cross a seam (bisection on
//! the exact 3D curve), every piece is placed in the one chart cell, the seam
//! and boundary sides are split at the resulting nodes (a node on one copy of
//! a seam is a node of the other copy too), and the certified arrangement of
//! [`super::chart_arrangement`] is traced. Every emitted edge is a
//! sub-interval of an input edge's exact curve.
//!
//! The sampled checks below (a section leaving the bounded extent, grazing a
//! seam line, crossing both seams at one point, a piece outside the cell)
//! only decline early; they prove nothing. That the sections stay on the
//! face, cross seams and rims only at their nodes and touch nothing between
//! samples is proven by the arrangement's noding certificate, in which the
//! seam copies and rims are pieces like any other.
//!
//! A face is *admitted* when it is a torus or cylinder face without holes
//! whose boundary lifts to a chart rectangle and whose sections are all
//! supported exact curves ([`super::chart_certificate::exact_curve_supported`]).
//! Outside that, the splitter reports [`ChartSplit::Unsupported`] and the
//! established paths own the face; an admitted face whose certificates fail
//! reports [`ChartSplit::Uncertified`], which the caller counts for the
//! operation's gate.
//!
//! Sub-faces never wrap a seam: a region the seam runs through is emitted as
//! the chart pieces on either side, sharing the seam piece as an ordinary
//! edge, and a region touching both copies of a seam piece uses that piece
//! twice, as the parent face did. A section cut at a seam gains a vertex
//! there; the partner face's copy of the same curve is refined at that vertex
//! by the builder's global collinear-vertex pass.

use std::f64::consts::TAU;

use remus_math::compact_sort::CompactSort;

use remus_math::vec::{Point2, Point3};
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

use super::super::split_types::{OrientedPCurveEdge, SectionEdge, SplitSubFace};
use super::chart_arrangement::ChartSplit;
use super::chart_arrangement::{
    Authority, FaceChart, GraphEdge, Node, Piece, interior_clearance, node_for, trace_subfaces,
};
use super::chart_certificate::{CertConfig, ChartMap, exact_curve_supported};
use crate::ds::Rank;
use crate::periodic_chart::{
    self, BoundaryUse, ChartCarrier, PeriodicChart, Side, lift_continuous, unwrap_near,
};

/// Decline with the reason's source line in the debug log.
macro_rules! decline {
    () => {{
        log::debug!("periodic_face_chart: declined at line {}", line!());
        return None;
    }};
}

/// Chart distance within which a section sample counts as touching a seam
/// line it does not cross. Such contacts are refused early; contacts between
/// samples are the noding certificate's to refuse.
const SEAM_GRAZE: f64 = 2e-3;
/// Samples per section along its whole trim before refinement.
const SECTION_SAMPLES: usize = 512;
/// Samples per emitted piece's chart polyline.
pub(super) const PIECE_SAMPLES: usize = 96;
/// Chart distance separating two copies of one 3D point.
pub(super) const CHART_COPY: f64 = 0.5;
/// Chart distance within which a node is snapped onto a side.
pub(super) const NODE_SNAP: f64 = 2e-6;

/// Lift the face's outer boundary (as the splitter received it) and
/// recognise the chart rectangle.
#[allow(clippy::type_complexity)]
fn lift_boundary(
    carrier: &ChartCarrier,
    boundary: &[OrientedPCurveEdge],
    tol: f64,
) -> Option<(PeriodicChart, f64, Vec<(Piece, Side)>)> {
    let mut uses = Vec::with_capacity(boundary.len());
    for be in boundary {
        if (be.start_3d - be.end_3d).length() <= tol && be.trim.is_none() {
            return None;
        }
        let (ns, ne) = if be.forward {
            (be.start_3d, be.end_3d)
        } else {
            (be.end_3d, be.start_3d)
        };
        let domain = be
            .trim
            .unwrap_or_else(|| be.curve_3d.reconstruct_domain_from_endpoints(ns, ne));
        let (t_start, t_end) = if be.forward {
            (domain.0, domain.1)
        } else {
            (domain.1, domain.0)
        };
        uses.push(BoundaryUse {
            curve: &be.curve_3d,
            t_start,
            t_end,
            native_start: ns,
            native_end: ne,
        });
    }
    let (chart, orientation, lifted) = periodic_chart::lift_boundary(carrier, &uses)?;
    let pieces = boundary
        .iter()
        .zip(uses.iter())
        .zip(lifted)
        .map(|((be, u), l)| {
            (
                Piece {
                    curve: be.curve_3d.clone(),
                    t_a: u.t_start,
                    t_b: u.t_end,
                    p3_a: be.start_3d,
                    p3_b: be.end_3d,
                    auth_a: Authority::Boundary,
                    auth_b: Authority::Boundary,
                    poly: l.poly,
                    pave_block_id: be.pave_block_id,
                    source_topo_edge: be.source_topo_edge,
                    boundary: true,
                },
                l.side,
            )
        })
        .collect();
    Some((chart, orientation, pieces))
}

/// One span of an exact curve to be placed in a chart: a section, or a
/// boundary edge that does not run along a side of the chart.
pub(super) struct CurveSpan<'a> {
    pub(super) curve: &'a EdgeCurve,
    /// Native parameters at the span's start and end.
    pub(super) t0: f64,
    pub(super) t1: f64,
    pub(super) start: Point3,
    pub(super) end: Point3,
    pub(super) pave_block_id: Option<usize>,
}

/// Cut one section at its seam crossings and place every piece in the chart.
fn section_pieces(
    carrier: &ChartCarrier,
    chart: &PeriodicChart,
    section: &SectionEdge,
    tol: f64,
) -> Option<Vec<Piece>> {
    let (t0, t1) = section.domain();
    curve_pieces(
        carrier,
        chart,
        &CurveSpan {
            curve: &section.curve_3d,
            t0,
            t1,
            start: section.start,
            end: section.end,
            pave_block_id: section.pave_block_id,
        },
        tol,
    )
}

/// Cut one curve span at its seam crossings and place every piece in the
/// chart. The pieces are section pieces; a caller placing a boundary edge
/// marks them so itself.
#[allow(clippy::too_many_lines)]
pub(super) fn curve_pieces(
    carrier: &ChartCarrier,
    chart: &PeriodicChart,
    section: &CurveSpan<'_>,
    tol: f64,
) -> Option<Vec<Piece>> {
    if matches!(section.curve, EdgeCurve::Line) {
        decline!();
    }
    let (t0, t1) = (section.t0, section.t1);
    if !(t0.is_finite() && t1.is_finite()) || (t1 - t0).abs() <= 1e-12 {
        decline!();
    }
    let at = |t: f64| {
        section
            .curve
            .evaluate_with_endpoints(t, section.start, section.end)
    };
    #[allow(clippy::cast_precision_loss)]
    let ts: Vec<f64> = (0..=SECTION_SAMPLES)
        .map(|k| t0 + (t1 - t0) * (k as f64) / (SECTION_SAMPLES as f64))
        .collect();
    let pts: Vec<Point3> = ts.iter().map(|&t| at(t)).collect();
    let first = chart.place_uv(carrier.project(pts[0]));
    let lifted = lift_continuous(carrier, &pts, first);

    // Bounded directions: the section stays on the face.
    for p in &lifted {
        if !chart.periodic_u && (p.x() < chart.u_lo - 1e-6 || p.x() > chart.u_hi + 1e-6) {
            decline!();
        }
        if !chart.periodic_v && (p.y() < chart.v_lo - 1e-6 || p.y() > chart.v_hi + 1e-6) {
            decline!();
        }
    }

    // Seam crossings, refined on the exact curve. `axis` 0 = u, 1 = v.
    let mut cuts: Vec<f64> = Vec::new();
    for axis in 0..2 {
        let (periodic, lo) = if axis == 0 {
            (chart.periodic_u, chart.u_lo)
        } else {
            (chart.periodic_v, chart.v_lo)
        };
        if !periodic {
            continue;
        }
        let coord = |p: Point2| if axis == 0 { p.x() } else { p.y() };
        #[allow(clippy::cast_possible_truncation)]
        let cell = |x: f64| ((x - lo) / TAU).floor() as i64;
        let line_dist = |x: f64| {
            let r = (x - lo).rem_euclid(TAU);
            r.min(TAU - r)
        };
        let n = lifted.len();
        let mut crossing_at = vec![false; n];
        for i in 0..n - 1 {
            let (a, b) = (coord(lifted[i]), coord(lifted[i + 1]));
            let (ka, kb) = (cell(a), cell(b));
            if ka == kb {
                continue;
            }
            if ka.abs_diff(kb) > 1 {
                decline!();
            }
            #[allow(clippy::cast_precision_loss)]
            let line = lo + TAU * ka.max(kb) as f64;
            // Ends touching the line are the section's own endpoints.
            if (i == 0 && (a - line).abs() <= 1e-9) || (i + 1 == n - 1 && (b - line).abs() <= 1e-9)
            {
                continue;
            }
            let (mut lo_t, mut hi_t) = (ts[i], ts[i + 1]);
            let anchor = coord(lifted[i]);
            let side = |t: f64| {
                let (u, v) = carrier.project(at(t));
                let raw = if axis == 0 { u } else { v };
                unwrap_near(raw, anchor) - line
            };
            let below = side(lo_t).is_sign_negative();
            for _ in 0..100 {
                let mid = 0.5 * (lo_t + hi_t);
                if side(mid).is_sign_negative() == below {
                    lo_t = mid;
                } else {
                    hi_t = mid;
                }
            }
            cuts.push(0.5 * (lo_t + hi_t));
            crossing_at[i] = true;
            crossing_at[i + 1] = true;
        }
        // A sample close to a seam line with no crossing beside it grazes the
        // seam: the arrangement cannot tell a touch from a double crossing.
        let dist: Vec<f64> = lifted.iter().map(|&p| line_dist(coord(p))).collect();
        for i in 1..n - 1 {
            let local_min = dist[i] <= dist[i - 1] && dist[i] <= dist[i + 1];
            if local_min && dist[i] < SEAM_GRAZE && !crossing_at[i] {
                decline!();
            }
        }
    }
    cuts.compact_sort_by(f64::total_cmp);
    if t1 < t0 {
        cuts.reverse();
    }
    // Two crossings at one point is a corner crossing: refuse.
    for w in cuts.windows(2) {
        if (at(w[0]) - at(w[1])).length() <= 1e-6 {
            decline!();
        }
    }
    let mut params = vec![t0];
    params.extend(cuts.iter().copied());
    params.push(t1);

    let mut out = Vec::with_capacity(params.len() - 1);
    for (k, w) in params.windows(2).enumerate() {
        let (ta, tb) = (w[0], w[1]);
        let last_piece = k + 2 == params.len();
        let p3_a = if k == 0 { section.start } else { at(ta) };
        let p3_b = if last_piece { section.end } else { at(tb) };
        if (p3_a - p3_b).length() <= tol && params.len() > 2 {
            decline!();
        }
        #[allow(clippy::cast_precision_loss)]
        let pts: Vec<Point3> = (0..=PIECE_SAMPLES)
            .map(|j| at(ta + (tb - ta) * (j as f64) / (PIECE_SAMPLES as f64)))
            .collect();
        // Place the piece's middle in the chart and unwrap outward from it.
        let mid = PIECE_SAMPLES / 2;
        let mid_uv = chart.place_uv(carrier.project(pts[mid]));
        let fwd = lift_continuous(carrier, &pts[mid..], mid_uv);
        let mut back_pts: Vec<Point3> = pts[..=mid].to_vec();
        back_pts.reverse();
        let mut back = lift_continuous(carrier, &back_pts, mid_uv);
        back.reverse();
        let mut poly = back;
        poly.extend_from_slice(&fwd[1..]);
        // Cut ends lie on a seam line exactly; snap them onto it.
        let snap = |p: Point2| {
            let mut x = p.x();
            let mut y = p.y();
            for s in [chart.u_lo, chart.u_hi] {
                if (x - s).abs() <= 1e-6 {
                    x = s;
                }
            }
            for s in [chart.v_lo, chart.v_hi] {
                if (y - s).abs() <= 1e-6 {
                    y = s;
                }
            }
            Point2::new(x, y)
        };
        let last = poly.len() - 1;
        poly[0] = snap(poly[0]);
        poly[last] = snap(poly[last]);
        if !poly.iter().all(|&p| chart.contains(p, 1e-9)) {
            decline!();
        }
        out.push(Piece {
            curve: section.curve.clone(),
            t_a: ta,
            t_b: tb,
            p3_a,
            p3_b,
            auth_a: if k == 0 {
                Authority::SectionEnd
            } else {
                Authority::Computed
            },
            auth_b: if last_piece {
                Authority::SectionEnd
            } else {
                Authority::Computed
            },
            poly,
            pave_block_id: section.pave_block_id,
            source_topo_edge: None,
            boundary: false,
        });
    }
    Some(out)
}

/// Split a torus or cylinder face as a planar arrangement in its cut-open
/// chart.
///
/// Returns `None` when the face or its sections fall outside the qualified
/// family (see the module docs); the caller then keeps its established paths.
#[allow(clippy::too_many_lines, clippy::too_many_arguments)]
pub(super) fn split_periodic_face_by_chart(
    surface: &FaceSurface,
    boundary_edges: &[OrientedPCurveEdge],
    has_inner_wires: bool,
    sections: &[SectionEdge],
    rank: Rank,
    reversed: bool,
    face_id: FaceId,
    tol: f64,
) -> ChartSplit {
    let Some(carrier) = ChartCarrier::of(surface) else {
        return ChartSplit::Unsupported;
    };
    if has_inner_wires || sections.is_empty() || boundary_edges.is_empty() {
        return ChartSplit::Unsupported;
    }
    let Some((chart, orientation, boundary_pieces)) = lift_boundary(&carrier, boundary_edges, tol)
    else {
        return ChartSplit::Unsupported;
    };
    if !sections.iter().all(|s| exact_curve_supported(&s.curve_3d)) {
        return ChartSplit::Unsupported;
    }
    // Admitted: from here on a decline is a failed certificate.
    ChartSplit::admitted(partition_admitted(
        surface,
        &carrier,
        &chart,
        orientation,
        boundary_pieces,
        sections,
        rank,
        reversed,
        face_id,
        tol,
    ))
}

/// The certified partition of an admitted face, or `None`.
#[allow(clippy::too_many_lines, clippy::too_many_arguments)]
fn partition_admitted(
    surface: &FaceSurface,
    carrier: &ChartCarrier,
    chart: &PeriodicChart,
    orientation: f64,
    boundary_pieces: Vec<(Piece, Side)>,
    sections: &[SectionEdge],
    rank: Rank,
    reversed: bool,
    face_id: FaceId,
    tol: f64,
) -> Option<Vec<SplitSubFace>> {
    let merge = (tol * 10.0).max(1e-6);

    let mut section_list: Vec<Piece> = Vec::new();
    for section in sections {
        section_list.extend(section_pieces(carrier, chart, section, tol)?);
    }

    // Nodes: boundary endpoints first (they carry the face's own vertices),
    // then section endpoints and seam cuts.
    let mut nodes: Vec<Node> = Vec::new();
    for piece in boundary_pieces
        .iter()
        .map(|(p, _)| p)
        .chain(section_list.iter())
    {
        let last = piece.poly.len() - 1;
        node_for(
            &mut nodes,
            piece.poly[0],
            piece.p3_a,
            piece.auth_a,
            merge,
            CHART_COPY,
        );
        node_for(
            &mut nodes,
            piece.poly[last],
            piece.p3_b,
            piece.auth_b,
            merge,
            CHART_COPY,
        );
    }
    // A node on one copy of a seam is a vertex of the seam edge, so it is a
    // node of the other copy too.
    let originals = nodes.len();
    for i in 0..originals {
        let n = nodes[i].clone();
        let Some(side) = chart.side_of(n.uv) else {
            continue;
        };
        let mirror = match side {
            Side::Left if chart.periodic_u => Point2::new(chart.u_hi, n.uv.y()),
            Side::Right if chart.periodic_u => Point2::new(chart.u_lo, n.uv.y()),
            Side::Bottom if chart.periodic_v => Point2::new(n.uv.x(), chart.v_hi),
            Side::Top if chart.periodic_v => Point2::new(n.uv.x(), chart.v_lo),
            Side::Left | Side::Right | Side::Bottom | Side::Top => continue,
        };
        node_for(&mut nodes, mirror, n.p3, n.authority, merge, CHART_COPY);
    }
    // Copies of one 3D point share the most authoritative position.
    for i in 0..nodes.len() {
        for j in 0..nodes.len() {
            if i != j
                && (nodes[i].p3 - nodes[j].p3).length() <= merge
                && nodes[j].authority > nodes[i].authority
            {
                nodes[i].p3 = nodes[j].p3;
                nodes[i].authority = nodes[j].authority;
            }
        }
    }
    // Nodes on a side sit exactly on it.
    for n in &mut nodes {
        if let Some(side) = chart.side_of(n.uv) {
            n.uv = match side {
                Side::Left => Point2::new(chart.u_lo, n.uv.y()),
                Side::Right => Point2::new(chart.u_hi, n.uv.y()),
                Side::Bottom => Point2::new(n.uv.x(), chart.v_lo),
                Side::Top => Point2::new(n.uv.x(), chart.v_hi),
            };
        }
    }

    // Split boundary pieces at the nodes lying on their side.
    let mut edges: Vec<GraphEdge> = Vec::new();
    // Boundary graph edges on a rim: a side the chart does not wrap, which
    // is a genuine boundary of the face rather than a seam copy.
    let mut rim_edges: Vec<usize> = Vec::new();
    for (piece, side) in &boundary_pieces {
        let rim = match side {
            Side::Left | Side::Right => !chart.periodic_u,
            Side::Bottom | Side::Top => !chart.periodic_v,
        };
        // Parameter of a node on the side's curve, inside the traversal span.
        let parameter_of = |p: Point3| -> Option<f64> {
            let (lo, hi) = (piece.t_a.min(piece.t_b), piece.t_a.max(piece.t_b));
            match &piece.curve {
                EdgeCurve::Circle(circle) => {
                    let raw = circle.project(p);
                    Some(raw + TAU * ((0.5 * (lo + hi) - raw) / TAU).round())
                }
                EdgeCurve::Line => {
                    let (ns, ne) = if piece.t_b > piece.t_a {
                        (piece.p3_a, piece.p3_b)
                    } else {
                        (piece.p3_b, piece.p3_a)
                    };
                    let d = ne - ns;
                    let len2 = d.length_squared();
                    (len2 > 0.0).then(|| (p - ns).dot(d) / len2)
                }
                EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => None,
            }
        };
        let along = |p: Point2| match side {
            Side::Left | Side::Right => p.y(),
            Side::Bottom | Side::Top => p.x(),
        };
        let last = piece.poly.len() - 1;
        let (sa, sb) = (along(piece.poly[0]), along(piece.poly[last]));
        let mut inner: Vec<(f64, usize)> = nodes
            .iter()
            .enumerate()
            .filter(|(_, n)| chart.side_of(n.uv) == Some(*side))
            .map(|(i, n)| (along(n.uv), i))
            .filter(|&(s, _)| {
                (s - sa) * (s - sb) < 0.0 && (s - sa).abs() > 1e-9 && (s - sb).abs() > 1e-9
            })
            .collect();
        inner.compact_sort_by(|x, y| ((x.0 - sa) / (sb - sa)).total_cmp(&((y.0 - sa) / (sb - sa))));
        let na = node_for(
            &mut nodes,
            piece.poly[0],
            piece.p3_a,
            piece.auth_a,
            merge,
            CHART_COPY,
        );
        let nb = node_for(
            &mut nodes,
            piece.poly[last],
            piece.p3_b,
            piece.auth_b,
            merge,
            CHART_COPY,
        );
        let (lo, hi) = (piece.t_a.min(piece.t_b), piece.t_a.max(piece.t_b));
        let mut chain = vec![(piece.t_a, na)];
        for &(_, ni) in &inner {
            let Some(t) = parameter_of(nodes[ni].p3) else {
                decline!();
            };
            if !(t > lo && t < hi) {
                decline!();
            }
            chain.push((t, ni));
        }
        chain.push((piece.t_b, nb));
        let frac = |t: f64| (t - piece.t_a) / (piece.t_b - piece.t_a);
        for w in chain.windows(2) {
            let ((ta, a), (tb, b)) = (w[0], w[1]);
            if frac(tb) <= frac(ta) {
                decline!();
            }
            let poly: Vec<Point2> = (0..=8)
                .map(|k| {
                    let f = f64::from(k) / 8.0;
                    nodes[a].uv + (nodes[b].uv - nodes[a].uv) * f
                })
                .collect();
            // Line pieces carry their geometry in their endpoints.
            let (ta, tb) = if matches!(piece.curve, EdgeCurve::Line) {
                (0.0, 1.0)
            } else {
                (ta, tb)
            };
            if rim {
                rim_edges.push(edges.len());
            }
            edges.push(GraphEdge {
                piece: Piece {
                    t_a: ta,
                    t_b: tb,
                    p3_a: nodes[a].p3,
                    p3_b: nodes[b].p3,
                    poly,
                    ..piece.clone()
                },
                node_a: a,
                node_b: b,
            });
        }
    }
    for mut piece in section_list {
        let last = piece.poly.len() - 1;
        let a = node_for(
            &mut nodes,
            piece.poly[0],
            piece.p3_a,
            piece.auth_a,
            merge,
            CHART_COPY,
        );
        let b = node_for(
            &mut nodes,
            piece.poly[last],
            piece.p3_b,
            piece.auth_b,
            merge,
            CHART_COPY,
        );
        piece.poly[0] = nodes[a].uv;
        piece.poly[last] = nodes[b].uv;
        let section = GraphEdge {
            piece,
            node_a: a,
            node_b: b,
        };
        // A section re-tracing a rim edge of the face (a plane holding the
        // rim circle meets the face only along it) cuts nothing: the rim
        // already bounds the face there. Its coincident exact curve can never
        // be separated from the rim by the noding certificate, so it leaves
        // the arrangement. Anything not proven identical stays a piece.
        if rim_edges
            .iter()
            .any(|&r| section_retraces_rim(&section, &edges[r], tol))
        {
            log::debug!("periodic_face_chart: face {face_id:?}: a section re-traces a rim edge");
            continue;
        }
        edges.push(section);
    }

    let map = ChartMap::of_carrier(carrier);
    // A chart step `(du, dv)` moves at least `floor·|(du, dv)|` in 3D: the
    // smaller of the two metric scales, times the chord factor `2/π` of an
    // angle step up to `π`.
    let floor = std::f64::consts::FRAC_2_PI
        * match carrier {
            ChartCarrier::Torus(t) => (t.major_radius() - t.minor_radius()).min(t.minor_radius()),
            ChartCarrier::Cylinder(c) => c.radius().min(1.0),
        };
    if floor.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
        decline!();
    }
    let mismatch_cap = NODE_SNAP + 2.0 * merge / floor;
    let to_3d = |p: Point2| Some(carrier.evaluate(p.x(), p.y()));
    let face = FaceChart {
        surface,
        reversed,
        face_id,
        rank,
        orientation,
        map: &map,
        to_3d: &to_3d,
        cert: CertConfig {
            mismatch_cap,
            ball_cap: 100.0 * mismatch_cap,
        },
        min_clearance: interior_clearance(tol) / floor,
    };
    let out = trace_subfaces(&face, &nodes, &edges)?;
    log::debug!(
        "periodic_face_chart: face {face_id:?} split into {} regions",
        out.len()
    );
    Some(out)
}

/// Whether `section` is the same exact arc as the rim boundary edge `rim`.
///
/// Both must be circle arcs between the same two nodes, with an
/// analytically coincident carrier (see [`circles_coincide`]) and the same
/// arc of it. The two arcs of a circle between two points have antipodal
/// midpoints, so agreeing midpoints select the same arc; the section then
/// lies within `tol` of the rim everywhere. Shared pave-block identity is
/// required when the section carries one.
fn section_retraces_rim(section: &GraphEdge, rim: &GraphEdge, tol: f64) -> bool {
    let same_nodes = (section.node_a == rim.node_a && section.node_b == rim.node_b)
        || (section.node_a == rim.node_b && section.node_b == rim.node_a);
    if !same_nodes || section.piece.boundary || !rim.piece.boundary {
        return false;
    }
    if section
        .piece
        .pave_block_id
        .is_some_and(|id| rim.piece.pave_block_id != Some(id))
    {
        return false;
    }
    let (EdgeCurve::Circle(cs), EdgeCurve::Circle(cr)) = (&section.piece.curve, &rim.piece.curve)
    else {
        return false;
    };
    if !circles_coincide(cs, cr, tol) {
        return false;
    }
    let (ss, sr) = (
        (section.piece.t_b - section.piece.t_a).abs(),
        (rim.piece.t_b - rim.piece.t_a).abs(),
    );
    // Each arc spans more than nothing and at most one full turn.
    let turn = TAU * (1.0 + 1e-12);
    if !(ss > 0.0 && sr > 0.0 && ss <= turn && sr <= turn) {
        return false;
    }
    if (ss * cs.radius() - sr * cr.radius()).abs() > tol {
        return false;
    }
    let mid = |p: &Piece| {
        p.curve
            .evaluate_with_endpoints(0.5 * (p.t_a + p.t_b), p.p3_a, p.p3_b)
    };
    (mid(&section.piece) - mid(&rim.piece)).length() <= tol
}

/// Whether two circles coincide within `tol`: an analytic bound on the
/// distance from every point of `a` to `b`.
///
/// A point `p = c_a + r_a w` of `a` lies at height `h ≤ |Δc| + r_a sin θ`
/// over `b`'s plane (`θ` the angle between the normals) and at in-plane
/// radius within `h + |Δc| + |Δr|` of `r_b`, so its distance to `b` is at
/// most `3|Δc| + |Δr| + 2 r_a sin θ`.
fn circles_coincide(
    a: &remus_math::curves::Circle3D,
    b: &remus_math::curves::Circle3D,
    tol: f64,
) -> bool {
    let (na, nb) = (a.normal(), b.normal());
    let scale = na.length() * nb.length();
    if !(scale.is_finite() && scale > 0.0) {
        return false;
    }
    let sin_theta = na.cross(nb).length() / scale;
    let bound = 3.0 * (a.center() - b.center()).length()
        + (a.radius() - b.radius()).abs()
        + 2.0 * a.radius().max(b.radius()) * sin_theta;
    bound.is_finite() && bound <= tol
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic)]
    use std::f64::consts::{FRAC_PI_3, PI};

    use remus_math::curves::Circle3D;
    use remus_math::curves2d::{Curve2D, Line2D};
    use remus_math::nurbs::interpolate;
    use remus_math::surfaces::ToroidalSurface;
    use remus_math::vec::Vec3;

    use super::*;

    fn torus() -> ToroidalSurface {
        ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 10.0, 2.0).unwrap()
    }

    fn pcurve() -> Curve2D {
        Curve2D::Line(
            Line2D::new(Point2::new(0.0, 0.0), remus_math::vec::Vec2::new(1.0, 0.0)).unwrap(),
        )
    }

    fn parallel(v: f64) -> EdgeCurve {
        EdgeCurve::Circle(
            Circle3D::new_with_ref(
                Point3::new(0.0, 0.0, 2.0 * v.sin()),
                Vec3::new(0.0, 0.0, 1.0),
                10.0 + 2.0 * v.cos(),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
        )
    }

    /// The meridian at major angle `u`, parameterised by the minor angle.
    fn meridian(u: f64) -> EdgeCurve {
        let radial = Vec3::new(u.cos(), u.sin(), 0.0);
        EdgeCurve::Circle(
            Circle3D::new_with_ref(
                Point3::new(0.0, 0.0, 0.0) + radial * 10.0,
                radial.cross(Vec3::new(0.0, 0.0, 1.0)),
                2.0,
                radial,
            )
            .unwrap(),
        )
    }

    fn use_of(curve: EdgeCurve, trim: (f64, f64), forward: bool) -> OrientedPCurveEdge {
        let o = Point3::new(0.0, 0.0, 0.0);
        let (s, e) = (
            curve.evaluate_with_endpoints(trim.0, o, o),
            curve.evaluate_with_endpoints(trim.1, o, o),
        );
        let (start_3d, end_3d) = if forward { (s, e) } else { (e, s) };
        OrientedPCurveEdge {
            curve_3d: curve,
            trim: Some(trim),
            pcurve: pcurve(),
            start_uv: Point2::new(0.0, 0.0),
            end_uv: Point2::new(0.0, 0.0),
            start_3d,
            end_3d,
            forward,
            source_edge_idx: None,
            pave_block_id: None,
            source_topo_edge: None,
        }
    }

    /// The band `0 ≤ v ≤ π/3` of the torus, closed around `u`.
    fn band() -> Vec<OrientedPCurveEdge> {
        vec![
            use_of(parallel(0.0), (0.0, TAU), true),
            use_of(meridian(0.0), (0.0, FRAC_PI_3), true),
            use_of(parallel(FRAC_PI_3), (0.0, TAU), false),
            use_of(meridian(0.0), (0.0, FRAC_PI_3), false),
        ]
    }

    fn section(curve: EdgeCurve, trim: (f64, f64)) -> SectionEdge {
        let o = Point3::new(0.0, 0.0, 0.0);
        SectionEdge {
            start: curve.evaluate_with_endpoints(trim.0, o, o),
            end: curve.evaluate_with_endpoints(trim.1, o, o),
            curve_3d: curve,
            trim: Some(trim),
            pcurve_a: pcurve(),
            pcurve_b: pcurve(),
            start_uv_a: None,
            end_uv_a: None,
            start_uv_b: None,
            end_uv_b: None,
            target_face: None,
            pave_block_id: None,
        }
    }

    fn split(sections: &[SectionEdge], holes: bool) -> ChartSplit {
        let surface = FaceSurface::Torus(torus());
        split_periodic_face_by_chart(
            &surface,
            &band(),
            holes,
            sections,
            Rank::A,
            false,
            super::super::chart_arrangement::tests::dummy_face(),
            1e-7,
        )
    }

    #[test]
    fn a_rim_to_rim_meridian_is_certified() {
        let cut = section(meridian(PI), (0.0, FRAC_PI_3));
        let ChartSplit::Certified(parts) = split(&[cut], false) else {
            panic!("an admitted, cleanly cut band is certified");
        };
        assert_eq!(
            parts.len(),
            2,
            "the seam and the cut bound two chart regions"
        );
    }

    #[test]
    fn crossing_sections_on_an_admitted_band_are_uncertified_not_unsupported() {
        // A diagonal section crossing the meridian away from any node.
        let t = torus();
        let pts: Vec<Point3> = (0..=64)
            .map(|k| {
                let s = f64::from(k) / 64.0;
                t.evaluate(PI - 0.3 + 0.6 * s, FRAC_PI_3 * s)
            })
            .collect();
        let diagonal = section(
            EdgeCurve::NurbsCurve(interpolate(&pts, 3).unwrap()),
            (0.0, 1.0),
        );
        let cut = section(meridian(PI), (0.0, FRAC_PI_3));
        assert!(matches!(
            split(&[cut, diagonal], false),
            ChartSplit::Uncertified
        ));
    }

    #[test]
    fn faces_or_sections_outside_the_family_are_unsupported() {
        let cut = section(meridian(PI), (0.0, FRAC_PI_3));
        assert!(matches!(
            split(std::slice::from_ref(&cut), true),
            ChartSplit::Unsupported
        ));
        let mut line = cut;
        line.curve_3d = EdgeCurve::Line;
        line.trim = None;
        assert!(matches!(split(&[line], false), ChartSplit::Unsupported));
        assert!(matches!(split(&[], false), ChartSplit::Unsupported));
    }

    /// A plane holding a rim circle meets the band only along that rim
    /// (the walking-stick foot's R20 band resize: the tool cap lies on the
    /// fillet's equator rim). Such sections cut nothing; their exact curves
    /// coincide with the rim, so the noding certificate cannot separate them
    /// and the admitted face was refused as uncertified.
    #[test]
    fn sections_retracing_a_rim_are_not_cuts() {
        let halves = [
            section(parallel(0.0), (0.0, PI)),
            section(parallel(0.0), (PI, TAU)),
        ];
        let ChartSplit::Certified(parts) = split(&halves, false) else {
            panic!("rim re-traces leave the band whole and certified");
        };
        assert_eq!(parts.len(), 1, "a rim re-trace splits nothing");
        let top = [section(parallel(FRAC_PI_3), (0.0, TAU))];
        let ChartSplit::Certified(parts) = split(&top, false) else {
            panic!("a whole-rim re-trace leaves the band whole and certified");
        };
        assert_eq!(parts.len(), 1);
    }

    /// A rim re-trace leaves the arrangement; a genuine cut stays in it.
    #[test]
    fn a_rim_retrace_beside_a_real_cut_keeps_the_cut() {
        let sections = [
            section(parallel(0.0), (0.0, PI)),
            section(parallel(0.0), (PI, TAU)),
            section(meridian(PI), (0.0, FRAC_PI_3)),
        ];
        let ChartSplit::Certified(parts) = split(&sections, false) else {
            panic!("the meridian cut is certified");
        };
        assert_eq!(parts.len(), 2, "the seam and the cut bound two regions");
    }

    /// A parallel off the rim by more than the tolerance is a genuine
    /// section, never a re-trace: it is not dropped as one.
    #[test]
    fn a_parallel_just_off_the_rim_is_not_a_retrace() {
        let off = [
            section(parallel(1e-4), (0.0, PI)),
            section(parallel(1e-4), (PI, TAU)),
        ];
        assert!(!matches!(
            split(&off, false),
            ChartSplit::Certified(ref parts) if parts.len() == 1
        ));
    }

    fn graph_edge(
        curve: EdgeCurve,
        t: (f64, f64),
        nodes: (usize, usize),
        boundary: bool,
    ) -> GraphEdge {
        let o = Point3::new(0.0, 0.0, 0.0);
        GraphEdge {
            piece: Piece {
                p3_a: curve.evaluate_with_endpoints(t.0, o, o),
                p3_b: curve.evaluate_with_endpoints(t.1, o, o),
                curve,
                t_a: t.0,
                t_b: t.1,
                auth_a: Authority::Boundary,
                auth_b: Authority::Boundary,
                poly: Vec::new(),
                pave_block_id: None,
                source_topo_edge: None,
                boundary,
            },
            node_a: nodes.0,
            node_b: nodes.1,
        }
    }

    #[test]
    fn only_the_same_arc_of_a_coincident_circle_is_a_retrace() {
        let tol = 1e-7;
        let rim = graph_edge(parallel(0.0), (0.0, PI), (0, 1), true);
        // The same arc, traversed either way.
        let same = graph_edge(parallel(0.0), (PI, 0.0), (1, 0), false);
        assert!(section_retraces_rim(&same, &rim, tol));
        // The complementary arc between the same two nodes.
        let other_half = graph_edge(parallel(0.0), (PI, TAU), (1, 0), false);
        assert!(!section_retraces_rim(&other_half, &rim, tol));
        // A circle off the rim by 1e-6 (> tol).
        let off = graph_edge(parallel(5e-7), (0.0, PI), (0, 1), false);
        assert!(!section_retraces_rim(&off, &rim, tol));
        // A different node pair, a boundary piece, or a foreign pave block.
        assert!(!section_retraces_rim(
            &graph_edge(parallel(0.0), (0.0, PI), (0, 2), false),
            &rim,
            tol
        ));
        assert!(!section_retraces_rim(
            &graph_edge(parallel(0.0), (0.0, PI), (0, 1), true),
            &rim,
            tol
        ));
        let mut foreign = graph_edge(parallel(0.0), (0.0, PI), (0, 1), false);
        foreign.piece.pave_block_id = Some(7);
        assert!(!section_retraces_rim(&foreign, &rim, tol));
        // A tilted circle through the same rim points is not the rim.
        let tilted = EdgeCurve::Circle(
            Circle3D::new_with_ref(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 1e-3, 1.0),
                12.0,
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
        );
        assert!(!section_retraces_rim(
            &graph_edge(tilted, (0.0, PI), (0, 1), false),
            &rim,
            tol
        ));
    }
}
