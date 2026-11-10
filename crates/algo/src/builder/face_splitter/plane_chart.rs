//! A planar face re-partitioned as a certified arrangement in its own frame.
//!
//! The established planar paths weave sections into the boundary with a
//! greedy wire walker and dedicated hole integration. A section bridging the
//! outer boundary and a hole (an annular cap crossed by two arcs of a
//! plane × torus oval, OpenZCAD issue 1242) comes out of that weave as an
//! out-and-back spur, or not at all, and the face stays whole. Such a result
//! is never a partition, so the caller asks for this one instead.
//!
//! The face's own wires (outer and holes; lines and circles only) are split
//! exactly at the section endpoints lying on them, the sections are taken
//! whole, and [`super::chart_arrangement`] certifies and traces the
//! arrangement. A face is admitted when its wires are lines and circles and
//! its sections supported exact curves; outside that the splitter reports
//! [`ChartSplit::Unsupported`]. An admitted face whose certificate fails
//! reports [`ChartSplit::Uncertified`]: the caller keeps its established
//! result and counts it for the operation's gate.

use remus_math::compact_sort::CompactSort;
use remus_math::vec::{Point2, Point3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

use super::super::plane_frame::PlaneFrame;
use super::super::split_types::{SectionEdge, SplitSubFace};
use super::chart_arrangement::ChartSplit;
use super::chart_arrangement::{
    Authority, FaceChart, GraphEdge, Node, Piece, interior_clearance, node_for, signed_area,
    trace_subfaces,
};
use super::chart_certificate::{CertConfig, ChartMap, exact_curve_supported};
use crate::ds::Rank;

/// Decline with the reason's source line in the debug log.
macro_rules! decline {
    () => {{
        log::debug!("plane_chart: declined at line {}", line!());
        return None;
    }};
}

/// Samples per curved boundary edge and per section polyline.
const CURVE_SAMPLES: usize = 128;

struct BoundaryEdge {
    curve: EdgeCurve,
    /// Native parameters at the traversal start and end.
    t_start: f64,
    t_end: f64,
    native_start: Point3,
    native_end: Point3,
    start: Point3,
    end: Point3,
    source: usize,
}

impl BoundaryEdge {
    fn at(&self, t: f64) -> Point3 {
        self.curve
            .evaluate_with_endpoints(t, self.native_start, self.native_end)
    }
}

/// Traversal parameter of `p` on a boundary edge, when it lies on it
/// strictly between its ends.
fn parameter_on(edge: &BoundaryEdge, p: Point3, merge: f64) -> Option<f64> {
    let (ta, tb) = (edge.t_start, edge.t_end);
    let t = match &edge.curve {
        EdgeCurve::Line => {
            let d = edge.native_end - edge.native_start;
            let len2 = d.length_squared();
            if len2 <= 0.0 {
                return None;
            }
            (p - edge.native_start).dot(d) / len2
        }
        EdgeCurve::Circle(circle) => {
            let raw = circle.project(p);
            let (lo, hi) = (ta.min(tb), ta.max(tb));
            raw + std::f64::consts::TAU * ((0.5 * (lo + hi) - raw) / std::f64::consts::TAU).round()
        }
        EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => return None,
    };
    let f = (t - ta) / (tb - ta);
    if !(f > 1e-9 && f < 1.0 - 1e-9) {
        return None;
    }
    ((edge.at(t) - p).length() <= merge).then_some(t)
}

/// Re-partition a planar face by its sections.
pub(super) fn split_plane_face_by_chart(
    topo: &Topology,
    face_id: FaceId,
    sections: &[SectionEdge],
    rank: Rank,
    tol: f64,
) -> ChartSplit {
    let Ok(face) = topo.face(face_id) else {
        return ChartSplit::Unsupported;
    };
    let admitted = matches!(face.surface(), FaceSurface::Plane { .. })
        && !sections.is_empty()
        && sections.iter().all(|s| exact_curve_supported(&s.curve_3d))
        && std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .all(|wid| {
                topo.wire(wid).is_ok_and(|w| {
                    w.edges().iter().all(|oe| {
                        topo.edge(oe.edge()).is_ok_and(|e| {
                            matches!(e.curve(), EdgeCurve::Line | EdgeCurve::Circle(_))
                        })
                    })
                })
            });
    if !admitted {
        return ChartSplit::Unsupported;
    }
    // Admitted: from here on a decline is a failed certificate.
    ChartSplit::admitted(partition_admitted(topo, face_id, sections, rank, tol))
}

/// The certified partition of an admitted planar face, or `None`.
#[allow(clippy::too_many_lines)]
fn partition_admitted(
    topo: &Topology,
    face_id: FaceId,
    sections: &[SectionEdge],
    rank: Rank,
    tol: f64,
) -> Option<Vec<SplitSubFace>> {
    let face = topo.face(face_id).ok()?;
    let surface = face.surface().clone();
    let FaceSurface::Plane { normal, d } = &surface else {
        return None;
    };
    if sections.is_empty() {
        return None;
    }
    let reversed = face.is_reversed();
    let origin = Point3::new(normal.x() * d, normal.y() * d, normal.z() * d);
    let frame = PlaneFrame::from_normal_and_point(*normal, origin);
    let merge = (tol * 10.0).max(1e-6);

    // The face's wires, in traversal order, with their chart polygons.
    let mut wires: Vec<Vec<BoundaryEdge>> = Vec::new();
    for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        let wire = topo.wire(wid).ok()?;
        let mut out = Vec::with_capacity(wire.edges().len());
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge()).ok()?;
            if !matches!(edge.curve(), EdgeCurve::Line | EdgeCurve::Circle(_)) {
                decline!();
            }
            let ns = topo.vertex(edge.start()).ok()?.point();
            let ne = topo.vertex(edge.end()).ok()?.point();
            let (d0, d1) = match edge.curve() {
                EdgeCurve::Line => (0.0, 1.0),
                EdgeCurve::Circle(_)
                | EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => edge.strict_domain().ok()?,
            };
            let (t_start, t_end, start, end) = if oe.is_forward() {
                (d0, d1, ns, ne)
            } else {
                (d1, d0, ne, ns)
            };
            out.push(BoundaryEdge {
                curve: edge.curve().clone(),
                t_start,
                t_end,
                native_start: ns,
                native_end: ne,
                start,
                end,
                source: oe.edge().index(),
            });
        }
        wires.push(out);
    }
    let sample = |e: &BoundaryEdge| -> Vec<Point2> {
        let n = if matches!(e.curve, EdgeCurve::Line) {
            8
        } else {
            CURVE_SAMPLES
        };
        (0..=n)
            .map(|k| {
                #[allow(clippy::cast_precision_loss)]
                let f = k as f64 / n as f64;
                frame.project(e.at(e.t_start + (e.t_end - e.t_start) * f))
            })
            .collect()
    };
    let polygons: Vec<Vec<Point2>> = wires
        .iter()
        .map(|w| {
            let mut poly = Vec::new();
            for e in w {
                let s = sample(e);
                poly.extend_from_slice(&s[..s.len() - 1]);
            }
            poly
        })
        .collect();
    let outer_area = signed_area(&polygons[0]);
    if outer_area == 0.0 {
        decline!();
    }
    let orientation = outer_area.signum();

    // Nodes: boundary vertices, then section endpoints.
    let mut nodes: Vec<Node> = Vec::new();
    for e in wires.iter().flatten() {
        node_for(
            &mut nodes,
            frame.project(e.start),
            e.start,
            Authority::Boundary,
            merge,
            f64::MAX,
        );
        node_for(
            &mut nodes,
            frame.project(e.end),
            e.end,
            Authority::Boundary,
            merge,
            f64::MAX,
        );
    }
    let mut section_ends = Vec::with_capacity(sections.len());
    for s in sections {
        let a = node_for(
            &mut nodes,
            frame.project(s.start),
            s.start,
            Authority::SectionEnd,
            merge,
            f64::MAX,
        );
        let b = node_for(
            &mut nodes,
            frame.project(s.end),
            s.end,
            Authority::SectionEnd,
            merge,
            f64::MAX,
        );
        section_ends.push((a, b));
    }

    let mut edges: Vec<GraphEdge> = Vec::new();
    // Boundary edges split at the nodes lying on them.
    for e in wires.iter().flatten() {
        let a = node_for(
            &mut nodes,
            frame.project(e.start),
            e.start,
            Authority::Boundary,
            merge,
            f64::MAX,
        );
        let b = node_for(
            &mut nodes,
            frame.project(e.end),
            e.end,
            Authority::Boundary,
            merge,
            f64::MAX,
        );
        let mut cuts: Vec<(f64, usize)> = nodes
            .iter()
            .enumerate()
            .filter(|&(i, _)| i != a && i != b)
            .filter_map(|(i, n)| parameter_on(e, n.p3, merge).map(|t| (t, i)))
            .collect();
        let frac = |t: f64| (t - e.t_start) / (e.t_end - e.t_start);
        cuts.compact_sort_by(|x, y| frac(x.0).total_cmp(&frac(y.0)));
        let mut chain = vec![(e.t_start, a)];
        chain.extend(cuts);
        chain.push((e.t_end, b));
        for w in chain.windows(2) {
            let ((ta, na), (tb, nb)) = (w[0], w[1]);
            if na == nb {
                decline!();
            }
            let piece_edge = BoundaryEdge {
                curve: e.curve.clone(),
                t_start: ta,
                t_end: tb,
                native_start: e.native_start,
                native_end: e.native_end,
                start: nodes[na].p3,
                end: nodes[nb].p3,
                source: e.source,
            };
            let mut poly = sample(&piece_edge);
            let last = poly.len() - 1;
            poly[0] = nodes[na].uv;
            poly[last] = nodes[nb].uv;
            // Line pieces carry their geometry in their endpoints.
            let (t_a, t_b) = if matches!(e.curve, EdgeCurve::Line) {
                (0.0, 1.0)
            } else {
                (ta, tb)
            };
            edges.push(GraphEdge {
                piece: Piece {
                    curve: e.curve.clone(),
                    t_a,
                    t_b,
                    p3_a: nodes[na].p3,
                    p3_b: nodes[nb].p3,
                    auth_a: Authority::Boundary,
                    auth_b: Authority::Boundary,
                    poly,
                    pave_block_id: None,
                    source_topo_edge: Some(piece_edge.source),
                    boundary: true,
                },
                node_a: na,
                node_b: nb,
            });
        }
    }
    // Sections, whole.
    for (s, &(a, b)) in sections.iter().zip(&section_ends) {
        if matches!(s.curve_3d, EdgeCurve::Line) {
            decline!();
        }
        let (t0, t1) = s.domain();
        if !(t0.is_finite() && t1.is_finite()) || (t1 - t0).abs() <= 1e-12 {
            decline!();
        }
        let mut poly: Vec<Point2> = (0..=CURVE_SAMPLES)
            .map(|k| {
                #[allow(clippy::cast_precision_loss)]
                let f = k as f64 / CURVE_SAMPLES as f64;
                frame.project(s.curve_3d.evaluate_with_endpoints(
                    t0 + (t1 - t0) * f,
                    s.start,
                    s.end,
                ))
            })
            .collect();
        let last = poly.len() - 1;
        poly[0] = nodes[a].uv;
        poly[last] = nodes[b].uv;
        edges.push(GraphEdge {
            piece: Piece {
                curve: s.curve_3d.clone(),
                t_a: t0,
                t_b: t1,
                p3_a: nodes[a].p3,
                p3_b: nodes[b].p3,
                auth_a: Authority::SectionEnd,
                auth_b: Authority::SectionEnd,
                poly,
                pave_block_id: s.pave_block_id,
                source_topo_edge: None,
                boundary: false,
            },
            node_a: a,
            node_b: b,
        });
    }

    let map = ChartMap::plane(&frame);
    let mismatch_cap = 2.0 * merge + 1e-9;
    let to_3d = |p: Point2| Some(frame.evaluate(p.x(), p.y()));
    let chart = FaceChart {
        surface: &surface,
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
        min_clearance: interior_clearance(tol),
    };
    let out = trace_subfaces(&chart, &nodes, &edges)?;
    log::debug!(
        "plane_chart: face {face_id:?} split into {} regions",
        out.len()
    );
    Some(out)
}
