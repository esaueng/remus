//! A cylinder face trimmed by holes, or by a bay in its outer wire,
//! partitioned as a planar arrangement in its lifted `(u, v)` chart.
//!
//! [`super::periodic_face_chart`] admits a cylinder lateral only while its
//! boundary lifts to a chart rectangle. A wall an earlier cut has left with a
//! hole (a bore another bore crosses) or with a bay in its outer wire (a
//! tunnel wall a bore has bitten out of, the bite reaching the wall's seam)
//! is outside that family, and the established emitters have no layout for
//! it either: they route such a face to the internal-loops path, which keeps
//! closed section loops only and drops every section chain that ends on the
//! hole or the bay. That is the consumer side of B7 — the section-pave pass
//! delivers pieces ending exactly on the trimming edges' pave vertices, and
//! this partition is what joins them to those edges.
//!
//! The certified arrangement of [`super::chart_arrangement`] is general: any
//! boundary pieces, holes included. This front end supplies it with
//!
//! - the outer wire lifted continuously into one chart cell, bay edges and
//!   all (a seam-cut wall runs its seam once at each end of the period);
//! - every hole wire, cut at the cell's seam lines and placed in the cell
//!   like a section, oriented against the outer wire;
//! - the sections, cut the same way (a straight section on a cylinder is a
//!   ruling and never crosses a seam line);
//! - every boundary piece split at the nodes lying on it, by projection onto
//!   its own exact curve;
//! - and, where a hole straddles the seam, the outer wire's seam pieces
//!   inside that hole removed: the seam edge there bounds nothing.
//!
//! The face is *admitted* when it is a cylinder face whose outer wire closes
//! in the chart within one period, every inner wire closes there too, every
//! curve is one the certificate supports, and the boundary is not a plain
//! rectangle without holes (the rectangle path owns that). Everything after
//! admission is proven by the arrangement's certificates; a face whose
//! certificate fails is reported as [`ChartSplit::Uncertified`]. The caller
//! keeps its established result, which had already failed its own checks,
//! and counts the face for the operation's gate, so an exact-only operation
//! refuses instead of returning it.

use std::f64::consts::TAU;

use remus_math::compact_sort::CompactSort;
use remus_math::vec::{Point2, Point3};
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

use super::super::split_types::{OrientedPCurveEdge, SectionEdge, SplitSubFace};
use super::chart_arrangement::{
    Authority, ChartSplit, FaceChart, GraphEdge, Node, Piece, interior_clearance, node_for,
    point_in_polygon, signed_area, trace_subfaces,
};
use super::chart_certificate::{CertConfig, ChartMap, exact_curve_supported};
use super::periodic_face_chart::{CHART_COPY, CurveSpan, NODE_SNAP, PIECE_SAMPLES, curve_pieces};
use crate::ds::Rank;
use crate::periodic_chart::{ChartCarrier, PeriodicChart, lift_continuous, unwrap_near};

/// Decline with the reason's source line in the debug log.
macro_rules! decline {
    () => {{
        log::debug!("trimmed_cylinder_chart: declined at line {}", line!());
        return None;
    }};
}

/// Chart distance within which a boundary sample sits on a side.
const SIDE_EPS: f64 = 1e-7;

/// One boundary edge use in traversal order.
struct BoundaryUse<'a> {
    edge: &'a OrientedPCurveEdge,
    /// Native curve parameters at the traversal start and end.
    t_start: f64,
    t_end: f64,
    /// Native start and end points, for a line's evaluation.
    native: (Point3, Point3),
}

impl<'a> BoundaryUse<'a> {
    fn of(edge: &'a OrientedPCurveEdge) -> Self {
        let native = if edge.forward {
            (edge.start_3d, edge.end_3d)
        } else {
            (edge.end_3d, edge.start_3d)
        };
        let domain = edge.trim.unwrap_or_else(|| {
            edge.curve_3d
                .reconstruct_domain_from_endpoints(native.0, native.1)
        });
        let (t_start, t_end) = if edge.forward {
            (domain.0, domain.1)
        } else {
            (domain.1, domain.0)
        };
        Self {
            edge,
            t_start,
            t_end,
            native,
        }
    }

    fn at(&self, t: f64) -> Point3 {
        self.edge
            .curve_3d
            .evaluate_with_endpoints(t, self.native.0, self.native.1)
    }

    /// `PIECE_SAMPLES + 1` points along the use, in traversal order.
    fn samples(&self) -> Vec<Point3> {
        #[allow(clippy::cast_precision_loss)]
        (0..=PIECE_SAMPLES)
            .map(|k| {
                let f = k as f64 / PIECE_SAMPLES as f64;
                self.at((self.t_end - self.t_start).mul_add(f, self.t_start))
            })
            .collect()
    }
}

/// Lift a wire continuously into the chart; `None` unless it closes there.
fn lift_wire(carrier: &ChartCarrier, uses: &[BoundaryUse<'_>]) -> Option<Vec<Vec<Point2>>> {
    let mut polys: Vec<Vec<Point2>> = Vec::with_capacity(uses.len());
    let mut prev: Option<Point2> = None;
    for u in uses {
        if !(u.t_start.is_finite() && u.t_end.is_finite()) || (u.t_end - u.t_start).abs() <= 1e-12 {
            return None;
        }
        let pts = u.samples();
        let (pu, pv) = carrier.project(pts[0]);
        let first = match prev {
            None => Point2::new(pu, pv),
            Some(p) => Point2::new(unwrap_near(pu, p.x()), pv),
        };
        let poly = lift_continuous(carrier, &pts, first);
        prev = poly.last().copied();
        polys.push(poly);
    }
    let first = polys.first()?.first().copied()?;
    let last = polys.last()?.last().copied()?;
    ((first - last).length() <= 1e-6).then_some(polys)
}

fn flat(polys: &[Vec<Point2>]) -> Vec<Point2> {
    polys
        .iter()
        .flat_map(|p| p[..p.len() - 1].iter().copied())
        .collect()
}

/// A boundary edge use as one arrangement piece with its lifted polyline.
fn boundary_piece(u: &BoundaryUse<'_>, poly: Vec<Point2>) -> Piece {
    Piece {
        curve: u.edge.curve_3d.clone(),
        t_a: u.t_start,
        t_b: u.t_end,
        p3_a: u.edge.start_3d,
        p3_b: u.edge.end_3d,
        auth_a: Authority::Boundary,
        auth_b: Authority::Boundary,
        poly,
        pave_block_id: u.edge.pave_block_id,
        source_topo_edge: u.edge.source_topo_edge,
        boundary: true,
    }
}

/// Place a ruling (a straight curve on the cylinder) in the chart cell.
fn ruling_piece(
    carrier: &ChartCarrier,
    chart: &PeriodicChart,
    (start, end): (Point3, Point3),
    pave_block_id: Option<usize>,
) -> Option<Piece> {
    let mid = chart.place_uv(carrier.project(start + (end - start) * 0.5));
    let (_, va) = carrier.project(start);
    let (_, vb) = carrier.project(end);
    // A straight curve on a cylinder runs along the axis: both ends at the
    // middle's angle.
    for p in [start, end] {
        let (u, _) = carrier.project(p);
        if (unwrap_near(u, mid.x()) - mid.x()).abs() > 1e-6 {
            decline!();
        }
    }
    if (va - vb).abs() <= 1e-12 {
        decline!();
    }
    #[allow(clippy::cast_precision_loss)]
    let poly: Vec<Point2> = (0..=8)
        .map(|k| Point2::new(mid.x(), (vb - va).mul_add(k as f64 / 8.0, va)))
        .collect();
    Some(Piece {
        curve: EdgeCurve::Line,
        t_a: 0.0,
        t_b: 1.0,
        p3_a: start,
        p3_b: end,
        auth_a: Authority::SectionEnd,
        auth_b: Authority::SectionEnd,
        poly,
        pave_block_id,
        source_topo_edge: None,
        boundary: false,
    })
}

/// Parameter of `piece`'s curve at the point nearest `p`, in the piece's own
/// traversal parameterisation, with that point.
fn parameter_on(piece: &Piece, p: Point3) -> Option<(f64, Point3)> {
    let (lo, hi) = (piece.t_a.min(piece.t_b), piece.t_a.max(piece.t_b));
    match &piece.curve {
        EdgeCurve::Line => {
            let d = piece.p3_b - piece.p3_a;
            let len2 = d.length_squared();
            if len2 <= 0.0 {
                return None;
            }
            let s = (p - piece.p3_a).dot(d) / len2;
            Some((
                (piece.t_b - piece.t_a).mul_add(s, piece.t_a),
                piece.p3_a + d * s,
            ))
        }
        EdgeCurve::Circle(c) => {
            let raw = c.project(p);
            let t = raw + TAU * ((0.5 * (lo + hi) - raw) / TAU).round();
            Some((t, c.evaluate(t)))
        }
        EdgeCurve::Ellipse(e) => {
            let raw = e.project(p);
            let t = raw + TAU * ((0.5 * (lo + hi) - raw) / TAU).round();
            Some((t, e.evaluate(t)))
        }
        EdgeCurve::NurbsCurve(n) => {
            let hit = remus_math::nurbs::projection::project_point_to_curve(n, p, 1e-12).ok()?;
            Some((hit.parameter, hit.point))
        }
        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => None,
    }
}

/// The chart point of `piece` at traversal fraction `f`, read off its
/// uniformly sampled polyline.
fn poly_at(piece: &Piece, f: f64) -> Point2 {
    let last = piece.poly.len() - 1;
    #[allow(clippy::cast_precision_loss)]
    let x = f.clamp(0.0, 1.0) * last as f64;
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let i = (x.floor() as usize).min(last.saturating_sub(1));
    #[allow(clippy::cast_precision_loss)]
    let w = x - i as f64;
    piece.poly[i] + (piece.poly[i + 1] - piece.poly[i]) * w
}

/// Split one boundary piece at the nodes lying strictly inside it.
///
/// A node lies on the piece within `weld`, not `merge`: a node computed on
/// a marched hole edge (where that edge crosses the seam line) sits up to
/// the marcher's ~1e-6 off the exact seam ruling it must split.
fn split_at_nodes(
    carrier: &ChartCarrier,
    piece: &Piece,
    nodes: &mut Vec<Node>,
    merge: f64,
    weld: f64,
) -> Option<Vec<GraphEdge>> {
    let last = piece.poly.len() - 1;
    let na = node_for(
        nodes,
        piece.poly[0],
        piece.p3_a,
        piece.auth_a,
        merge,
        CHART_COPY,
    );
    let nb = node_for(
        nodes,
        piece.poly[last],
        piece.p3_b,
        piece.auth_b,
        merge,
        CHART_COPY,
    );
    let span = piece.t_b - piece.t_a;
    let mut inner: Vec<(f64, usize)> = Vec::new();
    for (i, node) in nodes.iter().enumerate() {
        if i == na || i == nb {
            continue;
        }
        let Some((t, foot)) = parameter_on(piece, node.p3) else {
            continue;
        };
        let f = (t - piece.t_a) / span;
        if !(f > 0.0 && f < 1.0) || (foot - node.p3).length() > weld {
            continue;
        }
        // The same 3D point has a copy at each end of the period; only the
        // one in this piece's copy of the chart splits it.
        if (poly_at(piece, f) - node.uv).length() > CHART_COPY {
            continue;
        }
        if (node.p3 - piece.p3_a).length() <= merge || (node.p3 - piece.p3_b).length() <= merge {
            continue;
        }
        inner.push((f, i));
    }
    inner.compact_sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut chain = vec![(0.0, na)];
    chain.extend(inner);
    chain.push((1.0, nb));

    let mut out = Vec::with_capacity(chain.len() - 1);
    for w in chain.windows(2) {
        let ((fa, a), (fb, b)) = (w[0], w[1]);
        if fb <= fa || (a == b && chain.len() > 2) {
            decline!();
        }
        let (ta, tb) = (span.mul_add(fa, piece.t_a), span.mul_add(fb, piece.t_a));
        let line = matches!(piece.curve, EdgeCurve::Line);
        let poly: Vec<Point2> = if line {
            (0..=8)
                .map(|k| nodes[a].uv + (nodes[b].uv - nodes[a].uv) * (f64::from(k) / 8.0))
                .collect()
        } else {
            #[allow(clippy::cast_precision_loss)]
            let pts: Vec<Point3> = (0..=PIECE_SAMPLES)
                .map(|k| {
                    let f = k as f64 / PIECE_SAMPLES as f64;
                    piece.curve.evaluate_with_endpoints(
                        (tb - ta).mul_add(f, ta),
                        piece.p3_a,
                        piece.p3_b,
                    )
                })
                .collect();
            let mut poly = lift_continuous(carrier, &pts, nodes[a].uv);
            let end = poly.len() - 1;
            if (poly[end] - nodes[b].uv).length() > CHART_COPY {
                decline!();
            }
            poly[end] = nodes[b].uv;
            poly
        };
        // Line pieces carry their geometry in their endpoints.
        let (ta, tb) = if line { (0.0, 1.0) } else { (ta, tb) };
        out.push(GraphEdge {
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
    Some(out)
}

/// Split a cylinder face with holes or a bayed outer wire as a planar
/// arrangement in its chart.
///
/// [`ChartSplit::Unsupported`] when the face or its sections fall outside
/// the admitted family (see the module docs); [`ChartSplit::Uncertified`]
/// when an admitted face's partition cannot be certified.
#[allow(clippy::too_many_arguments)]
pub(super) fn split_trimmed_cylinder_by_chart(
    surface: &FaceSurface,
    outer: &[OrientedPCurveEdge],
    holes: &[Vec<OrientedPCurveEdge>],
    sections: &[SectionEdge],
    rank: Rank,
    reversed: bool,
    face_id: FaceId,
    tol: f64,
) -> ChartSplit {
    let FaceSurface::Cylinder(cylinder) = surface else {
        return ChartSplit::Unsupported;
    };
    let carrier = ChartCarrier::Cylinder(cylinder.clone());
    if sections.is_empty() || outer.is_empty() {
        return ChartSplit::Unsupported;
    }
    let supported =
        |curve: &EdgeCurve| matches!(curve, EdgeCurve::Line) || exact_curve_supported(curve);
    if !outer
        .iter()
        .chain(holes.iter().flatten())
        .all(|e| supported(&e.curve_3d))
        || !sections.iter().all(|s| supported(&s.curve_3d))
    {
        return ChartSplit::Unsupported;
    }
    let outer_uses: Vec<BoundaryUse<'_>> = outer.iter().map(BoundaryUse::of).collect();
    let Some(outer_polys) = lift_wire(&carrier, &outer_uses) else {
        return ChartSplit::Unsupported;
    };
    let outline = flat(&outer_polys);
    let (mut u_lo, mut u_hi, mut v_lo, mut v_hi) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for p in &outline {
        u_lo = u_lo.min(p.x());
        u_hi = u_hi.max(p.x());
        v_lo = v_lo.min(p.y());
        v_hi = v_hi.max(p.y());
    }
    let area = signed_area(&outline);
    if u_hi - u_lo > TAU + 1e-7 || v_hi - v_lo <= 1e-9 || area == 0.0 || !area.is_finite() {
        return ChartSplit::Unsupported;
    }
    let on_side = |p: &Point2| {
        (p.x() - u_lo).abs() <= SIDE_EPS
            || (p.x() - u_hi).abs() <= SIDE_EPS
            || (p.y() - v_lo).abs() <= SIDE_EPS
            || (p.y() - v_hi).abs() <= SIDE_EPS
    };
    if holes.is_empty() && outline.iter().all(on_side) {
        // A plain chart rectangle: the rectangle path's face.
        return ChartSplit::Unsupported;
    }
    let mut hole_uses: Vec<Vec<BoundaryUse<'_>>> = Vec::with_capacity(holes.len());
    let mut hole_outlines: Vec<Vec<Point2>> = Vec::with_capacity(holes.len());
    for hole in holes {
        let uses: Vec<BoundaryUse<'_>> = hole.iter().map(BoundaryUse::of).collect();
        let Some(polys) = lift_wire(&carrier, &uses) else {
            return ChartSplit::Unsupported;
        };
        hole_outlines.push(flat(&polys));
        hole_uses.push(uses);
    }
    let chart = PeriodicChart {
        u_lo,
        u_hi,
        v_lo,
        v_hi,
        periodic_u: (u_hi - u_lo - TAU).abs() <= 1e-7,
        periodic_v: false,
        v_angular: false,
    };
    // Admitted. The caller asks for this partition only after its established
    // one failed its own checks (a section left out, a spur, a region set
    // that does not tile the face), so a failure from here on is a face
    // nothing certifies: it is reported to the operation's gate, which
    // refuses an exact-only result, never handed back as outside the family.
    ChartSplit::admitted(partition(
        surface,
        &carrier,
        &chart,
        area.signum(),
        &outer_uses,
        outer_polys,
        &hole_uses,
        &hole_outlines,
        sections,
        rank,
        reversed,
        face_id,
        tol,
    ))
}

/// Whether some periodic image of chart point `p` lies inside `outline`.
fn inside_any_image(p: Point2, outline: &[Point2]) -> bool {
    (-2..=2).any(|k| {
        point_in_polygon(
            Point2::new(f64::from(k).mul_add(TAU, p.x()), p.y()),
            outline,
        )
    })
}

#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn partition(
    surface: &FaceSurface,
    carrier: &ChartCarrier,
    chart: &PeriodicChart,
    orientation: f64,
    outer_uses: &[BoundaryUse<'_>],
    outer_polys: Vec<Vec<Point2>>,
    hole_uses: &[Vec<BoundaryUse<'_>>],
    hole_outlines: &[Vec<Point2>],
    sections: &[SectionEdge],
    rank: Rank,
    reversed: bool,
    face_id: FaceId,
    tol: f64,
) -> Option<Vec<SplitSubFace>> {
    let merge = (tol * 10.0).max(1e-6);
    let weld = (tol * 100.0).max(merge);

    // Boundary pieces: the outer wire as lifted, then every hole edge cut at
    // the cell's seam lines and placed in the cell.
    let mut boundary: Vec<(Piece, bool)> = outer_uses
        .iter()
        .zip(outer_polys)
        .map(|(u, poly)| (boundary_piece(u, poly), true))
        .collect();
    for (uses, outline) in hole_uses.iter().zip(hole_outlines) {
        // A hole runs against the outer wire, so the face's net area is the
        // signed sum over all boundary pieces.
        let flip = signed_area(outline) * orientation > 0.0;
        for u in uses {
            let (t0, t1, start, end) = if flip {
                (u.t_end, u.t_start, u.edge.end_3d, u.edge.start_3d)
            } else {
                (u.t_start, u.t_end, u.edge.start_3d, u.edge.end_3d)
            };
            if matches!(u.edge.curve_3d, EdgeCurve::Line) {
                let mut piece = ruling_piece(carrier, chart, (start, end), u.edge.pave_block_id)?;
                piece.boundary = true;
                piece.auth_a = Authority::Boundary;
                piece.auth_b = Authority::Boundary;
                piece.source_topo_edge = u.edge.source_topo_edge;
                boundary.push((piece, false));
                continue;
            }
            let pieces = curve_pieces(
                carrier,
                chart,
                &CurveSpan {
                    curve: &u.edge.curve_3d,
                    t0,
                    t1,
                    start,
                    end,
                    pave_block_id: u.edge.pave_block_id,
                },
                tol,
            )?;
            let count = pieces.len();
            for (k, mut piece) in pieces.into_iter().enumerate() {
                piece.boundary = true;
                piece.source_topo_edge = u.edge.source_topo_edge;
                if k == 0 {
                    piece.auth_a = Authority::Boundary;
                }
                if k + 1 == count {
                    piece.auth_b = Authority::Boundary;
                }
                boundary.push((piece, false));
            }
        }
    }

    let mut section_list: Vec<Piece> = Vec::new();
    for section in sections {
        if matches!(section.curve_3d, EdgeCurve::Line) {
            section_list.push(ruling_piece(
                carrier,
                chart,
                (section.start, section.end),
                section.pave_block_id,
            )?);
            continue;
        }
        let (t0, t1) = section.domain();
        section_list.extend(curve_pieces(
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
        )?);
    }

    // Nodes: boundary endpoints first (they carry the face's own vertices),
    // then section endpoints and seam cuts.
    let mut nodes: Vec<Node> = Vec::new();
    for piece in boundary.iter().map(|(p, _)| p).chain(section_list.iter()) {
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
    // A node on one copy of the seam line is a node of the other copy too.
    if chart.periodic_u {
        let originals = nodes.len();
        for i in 0..originals {
            let n = nodes[i].clone();
            let mirror = if (n.uv.x() - chart.u_lo).abs() <= 1e-6 {
                Point2::new(chart.u_hi, n.uv.y())
            } else if (n.uv.x() - chart.u_hi).abs() <= 1e-6 {
                Point2::new(chart.u_lo, n.uv.y())
            } else {
                continue;
            };
            node_for(&mut nodes, mirror, n.p3, n.authority, merge, CHART_COPY);
        }
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
    // Nodes at a seam line sit exactly on it.
    if chart.periodic_u {
        for n in &mut nodes {
            for side in [chart.u_lo, chart.u_hi] {
                if (n.uv.x() - side).abs() <= 1e-6 {
                    n.uv = Point2::new(side, n.uv.y());
                }
            }
        }
    }

    let mut edges: Vec<GraphEdge> = Vec::new();
    for (piece, is_outer) in &boundary {
        for edge in split_at_nodes(carrier, piece, &mut nodes, merge, weld)? {
            // The outer wire's seam edge runs straight through a hole that
            // straddles the seam; inside the hole it bounds nothing.
            if *is_outer {
                let poly = &edge.piece.poly;
                let mid = poly[poly.len() / 2];
                if hole_outlines.iter().any(|h| inside_any_image(mid, h)) {
                    continue;
                }
            }
            edges.push(edge);
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
        piece.p3_a = nodes[a].p3;
        piece.p3_b = nodes[b].p3;
        edges.push(GraphEdge {
            piece,
            node_a: a,
            node_b: b,
        });
    }

    let ChartCarrier::Cylinder(cylinder) = carrier else {
        decline!();
    };
    let map = ChartMap::of_carrier(carrier);
    // A chart step `(du, dv)` moves at least `floor·|(du, dv)|` in 3D: the
    // smaller of the two metric scales, times the chord factor `2/π` of an
    // angle step up to `π`.
    let floor = std::f64::consts::FRAC_2_PI * cylinder.radius().min(1.0);
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
        "trimmed_cylinder_chart: face {face_id:?} split into {} regions",
        out.len()
    );
    Some(out)
}

#[cfg(test)]
mod tests {
    //! The unit wall `v ∈ [0, 4]` with the hole `u ∈ [1, 2] × v ∈ [1, 3]`
    //! ([`crate::chart_trim::fixtures`]), split through the production entry.
    #![allow(clippy::unwrap_used)]

    use remus_math::context::OperationContext;
    use remus_math::curves2d::{Curve2D, Line2D};
    use remus_math::det_hash::DetHashMap;
    use remus_math::surfaces::CylindricalSurface;
    use remus_math::tolerance::Tolerance;
    use remus_math::vec::Vec2;
    use remus_topology::Topology;

    use super::super::super::split_types::SurfaceInfo;
    use super::super::split_face_2d;
    use super::*;
    use crate::chart_trim::fixtures::{Trim, wall};
    use crate::gfa::SectionDropScope;

    /// The ruling at chart angle `u` from height `v0` to `v1`.
    fn ruling(cyl: &CylindricalSurface, u: f64, (v0, v1): (f64, f64)) -> SectionEdge {
        let dummy = Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap());
        SectionEdge {
            curve_3d: EdgeCurve::Line,
            trim: None,
            pcurve_a: dummy.clone(),
            pcurve_b: dummy,
            start: cyl.evaluate(u, v0),
            end: cyl.evaluate(u, v1),
            start_uv_a: None,
            end_uv_a: None,
            start_uv_b: None,
            end_uv_b: None,
            target_face: None,
            pave_block_id: None,
        }
    }

    /// The production split and the uncertified rescues it reported.
    fn split(topo: &Topology, face: FaceId, sections: &[SectionEdge]) -> (usize, u64) {
        let scope = SectionDropScope::enter();
        let parts = split_face_2d(
            topo,
            face,
            sections,
            Rank::A,
            &Tolerance::default(),
            None,
            Some(&SurfaceInfo::Parametric {
                u_periodic: true,
                v_periodic: false,
            }),
            &DetHashMap::default(),
            &OperationContext::new(),
            None,
        )
        .unwrap();
        (parts.len(), scope.uncertified_rescues())
    }

    /// Two rim-to-rim rulings clear of the hole: the control the admitted
    /// family certifies, so nothing is reported.
    #[test]
    fn rulings_clear_of_the_hole_report_nothing() {
        let (topo, face, cyl) = wall(Trim::Hole);
        let sections = [ruling(&cyl, 3.0, (0.0, 4.0)), ruling(&cyl, 5.0, (0.0, 4.0))];
        let (parts, uncertified) = split(&topo, face, &sections);
        assert_eq!(uncertified, 0);
        // The band between the rulings, and the holed rest, which the seam
        // runs through and which is emitted as its chart pieces either side.
        assert_eq!(parts, 3);
    }

    /// A ruling crossing the hole with no node on its rims (an unresolved
    /// contact) is admitted, and its arrangement cannot be certified. The
    /// established partition the rescue was asked to replace failed its own
    /// checks, so the operation's gate must hear of it: the face is reported
    /// uncertified, never silently handed back as outside the family.
    #[test]
    fn an_unnoded_crossing_of_the_hole_is_reported_uncertified() {
        let (topo, face, cyl) = wall(Trim::Hole);
        let sections = [ruling(&cyl, 1.5, (0.0, 4.0))];
        let (_, uncertified) = split(&topo, face, &sections);
        assert_eq!(uncertified, 1);
    }
}
