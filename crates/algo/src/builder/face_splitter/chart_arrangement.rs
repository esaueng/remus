//! A face partition traced as a planar arrangement in one 2D chart.
//!
//! The caller lifts every boundary edge and section of a face into a chart
//! where the face is a planar region (a plane's frame; a torus cut open along
//! its seams), as pieces whose geometry is a sub-interval of an exact input
//! curve. This module certifies and traces that arrangement:
//!
//! - every node closes (no dangling section end: that would be an unresolved
//!   chain, never silently dropped);
//! - the exact curves meet only at shared nodes, and leave every node in
//!   disjoint direction sectors: proven on certified chart enclosures of the
//!   exact curves by [`super::chart_certificate`], never on sampled
//!   polylines. The certificate's polylines (through nodes and span break
//!   points) form a plane graph equivalent to the exact one;
//! - faces are traced on those polylines with the rotation system (the next
//!   half-edge is the one clockwise from the twin, in certified leaving
//!   order), counter-clockwise cycles are regions, clockwise cycles of
//!   components detached from the outer boundary are holes of the smallest
//!   region containing them;
//! - each region's interior point lies outside every span enclosure by a
//!   clearance well above the curve tolerance, so it is inside the exact
//!   region;
//! - the kept regions' net area equals the face's, both measured on the same
//!   certified polylines: a consistency check of the traced combinatorics,
//!   not the coverage proof (that is the noding certificate).
//!
//! Any failed certificate declines (`None`); see [`ChartSplit`] for how
//! callers separate a face outside the family from an admitted face whose
//! certificate failed. Emitted edges take their geometry from the pieces
//! and their vertex positions from one canonical 3D point per node, so the
//! builder's quantized vertex dedup joins neighbouring sub-faces.

use std::f64::consts::TAU;

use remus_math::compact_sort::CompactSort;
use remus_math::curves2d::{Curve2D, Line2D, NurbsCurve2D};
use remus_math::vec::{Point2, Point3, Vec2};
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

use super::super::split_types::{OrientedPCurveEdge, SplitSubFace};
use super::chart_certificate::{CertConfig, ChartMap, certify, polygon_distance};
use crate::ds::Rank;

/// Decline with the reason's source line in the debug log.
macro_rules! decline {
    () => {{
        log::debug!("chart_arrangement: declined at line {}", line!());
        return None;
    }};
}

/// Where a node's 3D point came from; the most authoritative member of a
/// cluster supplies the shared vertex position.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub(super) enum Authority {
    /// A point computed here (a seam crossing).
    Computed = 0,
    /// A section endpoint (shared with the partner face's copy).
    SectionEnd = 1,
    /// A vertex of the face's own boundary.
    Boundary = 2,
}

#[derive(Clone, Debug)]
pub(super) struct Node {
    pub(super) uv: Point2,
    pub(super) p3: Point3,
    pub(super) authority: Authority,
}

/// One piece of an exact curve, with its chart polyline.
#[derive(Clone)]
pub(super) struct Piece {
    pub(super) curve: EdgeCurve,
    /// Native curve parameters at the piece's first and last polyline point.
    pub(super) t_a: f64,
    pub(super) t_b: f64,
    pub(super) p3_a: Point3,
    pub(super) p3_b: Point3,
    pub(super) auth_a: Authority,
    pub(super) auth_b: Authority,
    pub(super) poly: Vec<Point2>,
    pub(super) pave_block_id: Option<usize>,
    pub(super) source_topo_edge: Option<usize>,
    pub(super) boundary: bool,
}

#[derive(Clone)]
pub(super) struct GraphEdge {
    pub(super) piece: Piece,
    pub(super) node_a: usize,
    pub(super) node_b: usize,
}

/// Find (or add) the node for an endpoint: same 3D point within `merge` and
/// the same chart copy (chart distance under `chart_merge`).
pub(super) fn node_for(
    nodes: &mut Vec<Node>,
    uv: Point2,
    p3: Point3,
    auth: Authority,
    merge: f64,
    chart_merge: f64,
) -> usize {
    for (i, n) in nodes.iter_mut().enumerate() {
        if (n.p3 - p3).length() <= merge && (n.uv - uv).length() <= chart_merge {
            if auth > n.authority {
                n.p3 = p3;
                n.authority = auth;
            }
            return i;
        }
    }
    nodes.push(Node {
        uv,
        p3,
        authority: auth,
    });
    nodes.len() - 1
}

pub(super) fn signed_area(poly: &[Point2]) -> f64 {
    let n = poly.len();
    let mut a = 0.0;
    for i in 0..n {
        let (p, q) = (poly[i], poly[(i + 1) % n]);
        a += p.x() * q.y() - q.x() * p.y();
    }
    0.5 * a
}

pub(super) fn point_in_polygon(p: Point2, poly: &[Point2]) -> bool {
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (poly[i], poly[j]);
        if (a.y() > p.y()) != (b.y() > p.y()) {
            let x = (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x();
            if p.x() < x {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

fn segment_distance(p: Point2, a: Point2, b: Point2) -> f64 {
    let ab = b - a;
    let len2 = ab.x() * ab.x() + ab.y() * ab.y();
    let t = if len2 <= 0.0 {
        0.0
    } else {
        (((p - a).x() * ab.x() + (p - a).y() * ab.y()) / len2).clamp(0.0, 1.0)
    };
    (p - (a + ab * t)).length()
}

/// Union-find root of `x`, compressing the path.
fn find(c: &mut [usize], x: usize) -> usize {
    let mut r = x;
    while c[r] != r {
        r = c[r];
    }
    let mut y = x;
    while c[y] != r {
        let n = c[y];
        c[y] = r;
        y = n;
    }
    r
}

/// A counter-clockwise cycle of the arrangement and the holes assigned to it.
struct Region {
    cycle: Vec<usize>,
    poly: Vec<Point2>,
    area: f64,
    holes: Vec<(Vec<usize>, Vec<Point2>)>,
    root: usize,
}

/// What the caller knows about the face being partitioned.
pub(super) struct FaceChart<'a> {
    pub(super) surface: &'a FaceSurface,
    pub(super) reversed: bool,
    pub(super) face_id: FaceId,
    pub(super) rank: Rank,
    /// `+1` when the face's outer wire runs counter-clockwise in the chart.
    pub(super) orientation: f64,
    /// The chart the pieces are lifted into.
    pub(super) map: &'a ChartMap,
    /// Chart point to its 3D point on the surface.
    pub(super) to_3d: &'a dyn Fn(Point2) -> Option<Point3>,
    /// Node mismatch and node ball limits of the certificate.
    pub(super) cert: CertConfig,
    /// Smallest chart distance from an emitted interior point to every
    /// span enclosure (a 3D clearance well above the curve tolerance).
    pub(super) min_clearance: f64,
}

/// What a chart partition made of a face.
pub(super) enum ChartSplit {
    /// The face or one of its sections is outside the qualified family (see
    /// the callers' admission rules); the established paths own it.
    Unsupported,
    /// The face was admitted, but a certificate failed: no partition is
    /// proven. The caller must not present an established result for it as
    /// certified.
    Uncertified,
    /// The certified partition.
    Certified(Vec<SplitSubFace>),
}

impl ChartSplit {
    /// The outcome of an admitted face's partition attempt.
    pub(super) fn admitted(parts: Option<Vec<SplitSubFace>>) -> Self {
        parts.map_or(Self::Uncertified, Self::Certified)
    }
}

/// Smallest 3D clearance of an emitted interior point from the sub-face's
/// boundary curves: far above the curve tolerance, so classifying the point
/// against the other operand is not decided by the boundary it sits next to.
pub(super) fn interior_clearance(tol: f64) -> f64 {
    (100.0 * tol).max(1e-6)
}

/// Certify and trace the arrangement, emitting one sub-face per region.
#[allow(clippy::too_many_lines)]
pub(super) fn trace_subfaces(
    face: &FaceChart<'_>,
    nodes: &[Node],
    edges: &[GraphEdge],
) -> Option<Vec<SplitSubFace>> {
    if edges.is_empty() || !edges.iter().any(|e| e.piece.boundary) {
        decline!();
    }
    // Every node closes: a dangling end is an unresolved chain.
    let mut degree = vec![0usize; nodes.len()];
    for e in edges {
        degree[e.node_a] += 1;
        degree[e.node_b] += 1;
    }
    if degree.contains(&1) {
        if log::log_enabled!(log::Level::Trace) {
            for (i, n) in nodes.iter().enumerate() {
                log::trace!(
                    "chart_arrangement: node {i} deg {} uv ({:.5},{:.5}) p3 {:?} {:?}",
                    degree[i],
                    n.uv.x(),
                    n.uv.y(),
                    n.p3,
                    n.authority
                );
            }
        }
        decline!();
    }

    // The exact curves meet only at nodes: certified on per-span chart
    // enclosures (see `chart_certificate`), never on sampled polylines.
    let Some(cert) = certify(face.map, edges, nodes, &face.cert) else {
        decline!();
    };
    log::debug!("chart_arrangement: exact curve contacts certified");
    let polys: Vec<&[Point2]> = cert.edges.iter().map(|c| c.poly.as_slice()).collect();

    // Half-edges: 2k runs node_a → node_b, 2k + 1 the reverse.
    let half_poly = |h: usize| -> Vec<Point2> {
        let poly = polys[h / 2];
        if h.is_multiple_of(2) {
            poly.to_vec()
        } else {
            poly.iter().rev().copied().collect()
        }
    };
    let origin = |h: usize| {
        if h.is_multiple_of(2) {
            edges[h / 2].node_a
        } else {
            edges[h / 2].node_b
        }
    };
    // Leaving directions are the certified sectors' centres; the sectors at
    // a node are pairwise disjoint, so their order is the exact curves'.
    let mut outgoing: Vec<Vec<(f64, usize)>> = vec![Vec::new(); nodes.len()];
    for h in 0..edges.len() * 2 {
        let c = &cert.edges[h / 2];
        let angle = if h.is_multiple_of(2) {
            c.leave_a
        } else {
            c.leave_b
        };
        outgoing[origin(h)].push((angle.rem_euclid(TAU), h));
    }
    for list in &mut outgoing {
        list.compact_sort_by(|a, b| a.0.total_cmp(&b.0));
        let n = list.len();
        if n < 2 {
            continue;
        }
        for k in 0..n {
            let gap = (list[(k + 1) % n].0 - list[k].0).rem_euclid(TAU);
            if gap <= 0.0 || TAU - gap <= 0.0 {
                decline!();
            }
        }
    }
    let next = |h: usize| -> Option<usize> {
        let twin = h ^ 1;
        let list = &outgoing[origin(twin)];
        let i = list.iter().position(|&(_, x)| x == twin)?;
        Some(list[(i + list.len() - 1) % list.len()].1)
    };
    let mut used = vec![false; edges.len() * 2];
    let mut cycles: Vec<Vec<usize>> = Vec::new();
    for start in 0..edges.len() * 2 {
        if used[start] {
            continue;
        }
        let mut cycle = Vec::new();
        let mut h = start;
        loop {
            if used[h] {
                if h != start {
                    decline!();
                }
                break;
            }
            used[h] = true;
            cycle.push(h);
            let Some(n) = next(h) else {
                decline!();
            };
            h = n;
            if cycle.len() > edges.len() * 2 {
                decline!();
            }
        }
        cycles.push(cycle);
    }
    let cycle_poly = |c: &[usize]| -> Vec<Point2> {
        let mut out = Vec::new();
        for &h in c {
            let p = half_poly(h);
            out.extend_from_slice(&p[..p.len() - 1]);
        }
        out
    };

    // Connected components (by shared nodes).
    let mut comp: Vec<usize> = (0..nodes.len()).collect();
    for e in edges {
        let (a, b) = (find(&mut comp, e.node_a), find(&mut comp, e.node_b));
        comp[a] = b;
    }
    // Components carrying boundary edges: their clockwise cycle is the
    // outside of the face (outer wire) or the inside of a hole wire.
    let mut boundary_roots: Vec<usize> = Vec::new();
    for e in edges.iter().filter(|e| e.piece.boundary) {
        let r = find(&mut comp, e.node_a);
        if !boundary_roots.contains(&r) {
            boundary_roots.push(r);
        }
    }

    let mut regions: Vec<Region> = Vec::new();
    let mut holes: Vec<(Vec<usize>, Vec<Point2>, usize)> = Vec::new();
    for c in cycles {
        let poly = cycle_poly(&c);
        let area = signed_area(&poly);
        if area == 0.0 || !area.is_finite() {
            decline!();
        }
        let root = find(&mut comp, origin(c[0]));
        if area > 0.0 {
            regions.push(Region {
                cycle: c,
                poly,
                area,
                holes: Vec::new(),
                root,
            });
        } else {
            holes.push((c, poly, root));
        }
    }
    // A clockwise cycle bounds the unbounded side of its component; it is a
    // hole of the smallest region (of another component) around it, or the
    // outside of the face when no region contains it.
    for (c, poly, root) in holes {
        let probe = poly[0];
        let mut best: Option<usize> = None;
        for (ri, r) in regions.iter().enumerate() {
            if r.root == root || !point_in_polygon(probe, &r.poly) {
                continue;
            }
            if best.is_none_or(|b| r.area < regions[b].area) {
                best = Some(ri);
            }
        }
        match best {
            Some(b) => regions[b].holes.push((c, poly)),
            None if boundary_roots.contains(&root) => {}
            None => decline!(),
        }
    }

    // The face itself: the parity of the boundary pieces' certified
    // polylines (outer wire and hole wires), and its net area from the same
    // polylines. A point outside every enclosure has the same winding number
    // about the exact boundary as about these polylines.
    let boundary_segments: Vec<(Point2, Point2)> = edges
        .iter()
        .zip(&polys)
        .filter(|(e, _)| e.piece.boundary)
        .flat_map(|(_, p)| p.windows(2).map(|w| (w[0], w[1])))
        .collect();
    let in_face = |p: Point2| {
        let mut inside = false;
        for &(a, b) in &boundary_segments {
            if (a.y() > p.y()) != (b.y() > p.y()) {
                let x = (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x();
                if p.x() < x {
                    inside = !inside;
                }
            }
        }
        inside
    };
    let face_area = face.orientation
        * 0.5
        * boundary_segments
            .iter()
            .map(|(p, q)| p.x() * q.y() - q.x() * p.y())
            .sum::<f64>();
    if face_area.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
        decline!();
    }
    let periods = face.map.angular().map(|a| a.then_some(TAU));

    // Keep the regions on the face (a hole wire's inside traces as a
    // counter-clockwise cycle too). Each region's interior point lies
    // outside every span enclosure by `min_clearance`, so it is inside the
    // exact region, not just its polyline.
    let mut kept = Vec::with_capacity(regions.len());
    let mut net = 0.0;
    for r in regions {
        let Some(interior) = interior_point(
            &r.poly,
            &r.holes,
            &cert.enclosures,
            periods,
            face.min_clearance,
        ) else {
            decline!();
        };
        if !in_face(interior) {
            continue;
        }
        net += r.area
            - r.holes
                .iter()
                .map(|(_, p)| signed_area(p).abs())
                .sum::<f64>();
        kept.push((r, interior));
    }
    // Consistency of the traced combinatorics: the kept regions and the face
    // are measured on the same certified polylines, so they agree to
    // rounding. Geometric coverage of the exact face follows from the
    // noding certificate, not from this sum.
    if kept.is_empty() || (net - face_area).abs() > 1e-9 * face_area.abs().max(1e-12) {
        log::debug!("chart_arrangement: regions cover {net} of the face's {face_area}");
        decline!();
    }

    let to_oriented = |h: usize| -> Option<OrientedPCurveEdge> {
        let e = &edges[h / 2];
        let p = &e.piece;
        let (t_from, t_to, n_from, n_to) = if h.is_multiple_of(2) {
            (p.t_a, p.t_b, e.node_a, e.node_b)
        } else {
            (p.t_b, p.t_a, e.node_b, e.node_a)
        };
        let poly = if h.is_multiple_of(2) {
            cert.edges[h / 2].dense.clone()
        } else {
            cert.edges[h / 2].dense.iter().rev().copied().collect()
        };
        let (s_uv, e_uv) = (nodes[n_from].uv, nodes[n_to].uv);
        let pcurve = if poly.len() <= 2 || matches!(p.curve, EdgeCurve::Line) {
            let d = e_uv - s_uv;
            let len = d.length();
            if len <= 0.0 {
                return None;
            }
            Curve2D::Line(Line2D::new(s_uv, Vec2::new(d.x() / len, d.y() / len)).ok()?)
        } else {
            let n = poly.len();
            let mut knots = vec![0.0];
            #[allow(clippy::cast_precision_loss)]
            knots.extend((0..n).map(|k| k as f64 / (n - 1) as f64));
            knots.push(1.0);
            Curve2D::Nurbs(NurbsCurve2D::new(1, knots, poly, vec![1.0; n]).ok()?)
        };
        let line = matches!(p.curve, EdgeCurve::Line);
        Some(OrientedPCurveEdge {
            curve_3d: p.curve.clone(),
            trim: if line {
                None
            } else {
                Some((t_from.min(t_to), t_from.max(t_to)))
            },
            pcurve,
            start_uv: s_uv,
            end_uv: e_uv,
            start_3d: nodes[n_from].p3,
            end_3d: nodes[n_to].p3,
            forward: line || t_to > t_from,
            source_edge_idx: None,
            pave_block_id: p.pave_block_id,
            source_topo_edge: p.source_topo_edge,
        })
    };
    let wire = |c: &[usize]| -> Option<Vec<OrientedPCurveEdge>> {
        if face.orientation > 0.0 {
            c.iter().map(|&h| to_oriented(h)).collect()
        } else {
            c.iter().rev().map(|&h| to_oriented(h ^ 1)).collect()
        }
    };

    let mut out = Vec::with_capacity(kept.len());
    for (r, interior) in &kept {
        let Some(p3) = (face.to_3d)(*interior) else {
            decline!();
        };
        let mut inner_wires = Vec::with_capacity(r.holes.len());
        for (c, _) in &r.holes {
            let Some(w) = wire(c) else {
                decline!();
            };
            inner_wires.push(w);
        }
        let Some(outer_wire) = wire(&r.cycle) else {
            decline!();
        };
        out.push(SplitSubFace {
            surface: face.surface.clone(),
            outer_wire,
            inner_wires,
            reversed: face.reversed,
            parent: face.face_id,
            rank: face.rank,
            precomputed_interior: Some(p3),
        });
    }
    if log::log_enabled!(log::Level::Trace) {
        for (i, sf) in out.iter().enumerate() {
            for (w, wire) in std::iter::once(&sf.outer_wire)
                .chain(sf.inner_wires.iter())
                .enumerate()
            {
                for e in wire {
                    log::trace!(
                        "chart_arrangement: region {i} wire {w} {} fwd={} trim={:?} uv ({:.4},{:.4})->({:.4},{:.4}) 3d ({:.4},{:.4},{:.4})->({:.4},{:.4},{:.4})",
                        e.curve_3d.type_tag(),
                        e.forward,
                        e.trim,
                        e.start_uv.x(),
                        e.start_uv.y(),
                        e.end_uv.x(),
                        e.end_uv.y(),
                        e.start_3d.x(),
                        e.start_3d.y(),
                        e.start_3d.z(),
                        e.end_3d.x(),
                        e.end_3d.y(),
                        e.end_3d.z()
                    );
                }
            }
            log::trace!(
                "chart_arrangement: region {i} interior {:?}",
                sf.precomputed_interior
            );
        }
    }
    Some(out)
}

/// A point inside `outer` and outside every hole that keeps `min_clearance`
/// from every span enclosure and from their copies a period away (a point
/// near one seam copy is near the other one in 3D), or `None`.
///
/// Candidates are ranked by their distance from a thinned copy of the
/// region's polylines, a search heuristic only. A candidate is accepted on
/// the full polylines' membership test and its enclosure clearance: outside
/// every enclosure, its winding number about each exact cycle equals that
/// about the cycle's polyline, so it lies in the exact region.
fn interior_point(
    outer: &[Point2],
    holes: &[(Vec<usize>, Vec<Point2>)],
    enclosures: &[Vec<Point2>],
    periods: [Option<f64>; 2],
    min_clearance: f64,
) -> Option<Point2> {
    /// Candidates verified, best first.
    const CANDIDATES: usize = 12;
    let (mut lo_x, mut hi_x, mut lo_y, mut hi_y) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for p in outer {
        lo_x = lo_x.min(p.x());
        hi_x = hi_x.max(p.x());
        lo_y = lo_y.min(p.y());
        hi_y = hi_y.max(p.y());
    }
    let thin = |poly: &[Point2]| -> Vec<Point2> {
        let step = poly.len().div_ceil(256).max(1);
        poly.iter().step_by(step).copied().collect()
    };
    let rough: Vec<Vec<Point2>> = std::iter::once(thin(outer))
        .chain(holes.iter().map(|(_, h)| thin(h)))
        .collect();
    let inside = |p: Point2| {
        point_in_polygon(p, outer) && !holes.iter().any(|(_, h)| point_in_polygon(p, h))
    };
    let rough_clearance = |p: Point2| -> f64 {
        if !inside(p) {
            return -1.0;
        }
        let mut d = f64::MAX;
        for poly in &rough {
            let n = poly.len();
            for i in 0..n {
                d = d.min(segment_distance(p, poly[i], poly[(i + 1) % n]));
            }
        }
        d
    };
    let n = 40;
    let mut grid: Vec<(f64, Point2)> = Vec::with_capacity(n * n);
    for i in 0..n {
        for j in 0..n {
            #[allow(clippy::cast_precision_loss)]
            let p = Point2::new(
                lo_x + (hi_x - lo_x) * (i as f64 + 0.5) / n as f64,
                lo_y + (hi_y - lo_y) * (j as f64 + 0.5) / n as f64,
            );
            let c = rough_clearance(p);
            if c > 0.0 {
                grid.push((c, p));
            }
        }
    }
    // A sliver may hold no grid point: seed from the outer polyline's
    // segment midpoints nudged inward along the normal.
    if grid.is_empty() {
        let m = outer.len();
        for i in 0..m {
            let (a, b) = (outer[i], outer[(i + 1) % m]);
            let d = b - a;
            let len = d.length();
            if len <= 0.0 {
                continue;
            }
            for f in [1e-3, 1e-2, 1e-1] {
                let p = Point2::new(
                    0.5 * (a.x() + b.x()) - d.y() / len * len * f,
                    0.5 * (a.y() + b.y()) + d.x() / len * len * f,
                );
                let c = rough_clearance(p);
                if c > 0.0 {
                    grid.push((c, p));
                }
            }
        }
    }
    grid.compact_sort_by(|a, b| b.0.total_cmp(&a.0));
    grid.truncate(CANDIDATES);
    #[allow(clippy::cast_precision_loss)]
    let mut step = ((hi_x - lo_x) / n as f64).max((hi_y - lo_y) / n as f64);
    for _ in 0..6 {
        for cand in &mut grid {
            let centre = cand.1;
            for di in -2..=2 {
                for dj in -2..=2 {
                    let p = Point2::new(
                        centre.x() + step * f64::from(di) * 0.5,
                        centre.y() + step * f64::from(dj) * 0.5,
                    );
                    let c = rough_clearance(p);
                    if c > cand.0 {
                        *cand = (c, p);
                    }
                }
            }
        }
        step *= 0.5;
    }
    grid.compact_sort_by(|a, b| b.0.total_cmp(&a.0));

    let boxes: Vec<[f64; 4]> = enclosures
        .iter()
        .map(|e| {
            e.iter()
                .fold([f64::MAX, f64::MIN, f64::MAX, f64::MIN], |b, p| {
                    [
                        b[0].min(p.x()),
                        b[1].max(p.x()),
                        b[2].min(p.y()),
                        b[3].max(p.y()),
                    ]
                })
        })
        .collect();
    let shifts = |period: Option<f64>| -> Vec<f64> {
        period.map_or_else(|| vec![0.0], |t| vec![-t, 0.0, t])
    };
    let (su, sv) = (shifts(periods[0]), shifts(periods[1]));
    let clear = |p: Point2| -> bool {
        for &du in &su {
            for &dv in &sv {
                let q = Point2::new(p.x() + du, p.y() + dv);
                for (b, e) in boxes.iter().zip(enclosures) {
                    let bx = (b[0] - q.x()).max(q.x() - b[1]).max(0.0);
                    let by = (b[2] - q.y()).max(q.y() - b[3]).max(0.0);
                    if bx.hypot(by) >= min_clearance {
                        continue;
                    }
                    if polygon_distance(q, e) < min_clearance {
                        return false;
                    }
                }
            }
        }
        true
    };
    grid.into_iter()
        .map(|(_, p)| p)
        .find(|&p| inside(p) && clear(p))
}

#[cfg(test)]
pub(super) mod tests {
    #![allow(clippy::unwrap_used, clippy::cast_precision_loss)]
    use super::*;
    use remus_math::vec::Vec3;

    /// The unit square face in the `z = 0` plane, chart = `(x, y)`.
    fn unit_square(nodes: &mut Vec<Node>) -> Vec<GraphEdge> {
        let corners = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let ids: Vec<usize> = corners
            .iter()
            .map(|&(x, y)| {
                node_for(
                    nodes,
                    Point2::new(x, y),
                    Point3::new(x, y, 0.0),
                    Authority::Boundary,
                    1e-9,
                    f64::MAX,
                )
            })
            .collect();
        (0..4)
            .map(|k| line(nodes, ids[k], ids[(k + 1) % 4], true))
            .collect()
    }

    fn line(nodes: &[Node], a: usize, b: usize, boundary: bool) -> GraphEdge {
        let poly = (0..=8)
            .map(|k| nodes[a].uv + (nodes[b].uv - nodes[a].uv) * (f64::from(k) / 8.0))
            .collect();
        GraphEdge {
            piece: Piece {
                curve: EdgeCurve::Line,
                t_a: 0.0,
                t_b: 1.0,
                p3_a: nodes[a].p3,
                p3_b: nodes[b].p3,
                auth_a: Authority::SectionEnd,
                auth_b: Authority::SectionEnd,
                poly,
                pave_block_id: None,
                source_topo_edge: None,
                boundary,
            },
            node_a: a,
            node_b: b,
        }
    }

    fn node(nodes: &mut Vec<Node>, x: f64, y: f64) -> usize {
        node_for(
            nodes,
            Point2::new(x, y),
            Point3::new(x, y, 0.0),
            Authority::SectionEnd,
            1e-9,
            f64::MAX,
        )
    }

    /// A face id for the sub-faces' bookkeeping.
    pub(in super::super) fn dummy_face() -> FaceId {
        use remus_topology::edge::Edge;
        use remus_topology::face::Face;
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};
        let mut topo = remus_topology::Topology::new();
        let v: Vec<_> = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
            .iter()
            .map(|&(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), 1e-7)))
            .collect();
        let edges: Vec<_> = (0..3)
            .map(|k| {
                OrientedEdge::new(
                    topo.add_edge(Edge::new(v[k], v[(k + 1) % 3], EdgeCurve::Line)),
                    true,
                )
            })
            .collect();
        let wire = topo.add_wire(Wire::new(edges, true).unwrap());
        topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    fn trace(nodes: &[Node], edges: &[GraphEdge]) -> Option<Vec<SplitSubFace>> {
        trace_oriented(nodes, edges, 1.0)
    }

    fn trace_oriented(
        nodes: &[Node],
        edges: &[GraphEdge],
        orientation: f64,
    ) -> Option<Vec<SplitSubFace>> {
        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };
        let map = ChartMap::Plane {
            origin: Point3::new(0.0, 0.0, 0.0),
            axes: [
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ],
        };
        let to_3d = |p: Point2| Some(Point3::new(p.x(), p.y(), 0.0));
        let chart = FaceChart {
            surface: &surface,
            reversed: false,
            face_id: dummy_face(),
            rank: Rank::A,
            orientation,
            map: &map,
            to_3d: &to_3d,
            cert: CertConfig {
                mismatch_cap: 1e-9,
                ball_cap: 1e-7,
            },
            min_clearance: 1e-6,
        };
        trace_subfaces(&chart, nodes, edges)
    }

    #[test]
    fn a_crossing_section_splits_the_face_in_two() {
        let mut nodes = Vec::new();
        let mut edges = unit_square(&mut nodes);
        // Split the bottom and top sides at x = 0.5 and run a section between.
        let (b, t) = (node(&mut nodes, 0.5, 0.0), node(&mut nodes, 0.5, 1.0));
        edges[0] = line(&nodes, 0, b, true);
        edges.push(line(&nodes, b, 1, true));
        edges[2] = line(&nodes, 2, t, true);
        edges.push(line(&nodes, t, 3, true));
        edges.push(line(&nodes, b, t, false));
        let parts = trace(&nodes, &edges).unwrap();
        assert_eq!(parts.len(), 2);
        for part in &parts {
            assert_eq!(part.outer_wire.len(), 4);
            assert!(part.inner_wires.is_empty());
            let p = part.precomputed_interior.unwrap();
            assert!((p.x() - 0.25).abs() < 0.2 || (p.x() - 0.75).abs() < 0.2);
        }
    }

    #[test]
    fn a_dangling_section_end_declines() {
        let mut nodes = Vec::new();
        let mut edges = unit_square(&mut nodes);
        let (b, m) = (node(&mut nodes, 0.5, 0.0), node(&mut nodes, 0.5, 0.5));
        edges[0] = line(&nodes, 0, b, true);
        edges.push(line(&nodes, b, 1, true));
        edges.push(line(&nodes, b, m, false));
        assert!(trace(&nodes, &edges).is_none());
    }

    #[test]
    fn sections_crossing_away_from_a_node_decline() {
        let mut nodes = Vec::new();
        let mut edges = unit_square(&mut nodes);
        let ids = [
            node(&mut nodes, 0.5, 0.0),
            node(&mut nodes, 0.5, 1.0),
            node(&mut nodes, 0.0, 0.5),
            node(&mut nodes, 1.0, 0.5),
        ];
        edges[0] = line(&nodes, 0, ids[0], true);
        edges.push(line(&nodes, ids[0], 1, true));
        edges[1] = line(&nodes, 1, ids[3], true);
        edges.push(line(&nodes, ids[3], 2, true));
        edges[2] = line(&nodes, 2, ids[1], true);
        edges.push(line(&nodes, ids[1], 3, true));
        edges[3] = line(&nodes, 3, ids[2], true);
        edges.push(line(&nodes, ids[2], 0, true));
        // Two sections crossing at (0.5, 0.5) with no node there.
        edges.push(line(&nodes, ids[0], ids[1], false));
        edges.push(line(&nodes, ids[2], ids[3], false));
        assert!(trace(&nodes, &edges).is_none());
    }

    #[test]
    fn a_floating_closed_section_is_a_hole_and_a_region() {
        let mut nodes = Vec::new();
        let mut edges = unit_square(&mut nodes);
        // A closed polygonal loop around the centre, as four sections.
        let loop_ids = [
            node(&mut nodes, 0.4, 0.4),
            node(&mut nodes, 0.6, 0.4),
            node(&mut nodes, 0.6, 0.6),
            node(&mut nodes, 0.4, 0.6),
        ];
        for k in 0..4 {
            edges.push(line(&nodes, loop_ids[k], loop_ids[(k + 1) % 4], false));
        }
        let parts = trace(&nodes, &edges).unwrap();
        assert_eq!(parts.len(), 2);
        let holed: Vec<_> = parts.iter().filter(|p| !p.inner_wires.is_empty()).collect();
        assert_eq!(holed.len(), 1, "the square keeps the loop as its hole");
        assert_eq!(holed[0].inner_wires[0].len(), 4);
        let island = parts.iter().find(|p| p.inner_wires.is_empty()).unwrap();
        let c = island.precomputed_interior.unwrap();
        assert!((c.x() - 0.5).abs() < 0.1 && (c.y() - 0.5).abs() < 0.1);
    }

    #[test]
    fn a_boundary_against_its_stated_orientation_declines() {
        let mut nodes = Vec::new();
        let mut edges = unit_square(&mut nodes);
        let (b, t) = (node(&mut nodes, 0.5, 0.0), node(&mut nodes, 0.5, 1.0));
        edges[0] = line(&nodes, 0, b, true);
        edges.push(line(&nodes, b, 1, true));
        edges[2] = line(&nodes, 2, t, true);
        edges.push(line(&nodes, t, 3, true));
        edges.push(line(&nodes, b, t, false));
        // The square runs counter-clockwise; a caller claiming otherwise
        // measures a negative face and gets no partition.
        assert!(trace_oriented(&nodes, &edges, -1.0).is_none());
        assert!(trace_oriented(&nodes, &edges, 1.0).is_some());
    }

    /// The unit square cut by an exact NURBS section from `(0.2, 0)` to
    /// `(0.8, 0)` with control heights `ys` (a Bezier over `s ∈ [0, 1]`).
    fn bulge(ys: &[f64]) -> (Vec<Node>, Vec<GraphEdge>) {
        use remus_math::nurbs::NurbsCurve;
        let mut nodes = Vec::new();
        let mut edges = unit_square(&mut nodes);
        let (a, b) = (node(&mut nodes, 0.2, 0.0), node(&mut nodes, 0.8, 0.0));
        edges[0] = line(&nodes, 0, a, true);
        edges.push(line(&nodes, a, b, true));
        edges.push(line(&nodes, b, 1, true));
        let deg = ys.len() - 1;
        let cps: Vec<Point3> = ys
            .iter()
            .enumerate()
            .map(|(i, &y)| Point3::new(0.2 + 0.6 * i as f64 / deg as f64, y, 0.0))
            .collect();
        let mut knots = vec![0.0; deg + 1];
        knots.extend(vec![1.0; deg + 1]);
        let curve = NurbsCurve::new(deg, knots, cps, vec![1.0; deg + 1]).unwrap();
        let mut sec = line(&nodes, a, b, false);
        sec.piece.curve = EdgeCurve::NurbsCurve(curve);
        edges.push(sec);
        (nodes, edges)
    }

    #[test]
    fn a_sliver_region_without_a_clear_interior_point_declines() {
        // A parabolic sliver at most 1e-7 thick: no point keeps the 1e-6
        // clearance from the enclosures, so no interior point is emitted
        // (a point on the sliver's polyline middle would sit 5e-8 from the
        // exact curves).
        let (nodes, edges) = bulge(&[0.0, 2e-7, 0.0]);
        assert!(trace(&nodes, &edges).is_none());
        let (nodes, edges) = bulge(&[0.0, 0.2, 0.0]);
        assert_eq!(trace(&nodes, &edges).unwrap().len(), 2);
    }

    #[test]
    fn a_skew_bulge_region_gets_an_interior_point_inside_the_exact_curve() {
        // y = H·s³(1 − s) leaves its node tangent to the edge it starts on
        // (third-order contact): no sector separates them, so it declines.
        let (nodes, edges) = bulge(&[0.0, 0.0, 0.0, 0.125, 0.0]);
        assert!(trace(&nodes, &edges).is_none());
        // A skewed cubic bulge with transversal ends (Bernstein heights
        // 0, 0.02, 0.3, 0): its peak sits well off the chord's middle.
        let ys = [0.0, 0.02, 0.3, 0.0];
        let (nodes, edges) = bulge(&ys);
        let parts = trace(&nodes, &edges).unwrap();
        assert_eq!(parts.len(), 2);
        // x is linear in s, so the exact height at x is the Bernstein sum.
        let height = |x: f64| {
            let s = (x - 0.2) / 0.6;
            if (0.0..=1.0).contains(&s) {
                let t = 1.0 - s;
                3.0 * ys[1] * s * t * t + 3.0 * ys[2] * s * s * t
            } else {
                0.0
            }
        };
        let under: Vec<_> = parts
            .iter()
            .map(|p| p.precomputed_interior.unwrap())
            .filter(|p| p.y() < height(p.x()))
            .collect();
        assert_eq!(under.len(), 1, "one region lies under the bulge");
        let p = under[0];
        assert!(p.y() > 1e-6 && height(p.x()) - p.y() > 1e-6);
    }
}
