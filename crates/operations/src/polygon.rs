//! Polygon approximations of wires and faces.
//!
//! Sampled outlines used across booleans, measurement, validation and
//! construction. Lives below all of them so none has to reach into
//! `boolean` for it (RFC 0007 stage 0); `boolean` re-exports the public items
//! at their original paths.

use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceId;

/// Number of samples used when discretizing closed curves (circles, ellipses)
/// in the analytic boolean path. All code paths must use this constant so that
/// band fragments, cap face polygons, and holed-face inner wires share the
/// same vertices and edges at their boundaries.
pub const CLOSED_CURVE_SAMPLES: usize = 32;
/// Endpoint distance below which a sampled curve is treated as closed, in mm.
const CLOSED_CURVE_ENDPOINT_TOL_MM: f64 = 1e-6;

/// Sample `n` evenly-spaced points along a closed edge curve.
///
/// For `Circle` and `Ellipse`, samples at `TAU * i / n`.
/// For closed `NurbsCurve`, samples across the domain avoiding endpoint
/// duplication. Returns an empty vec for `Line` (no sampling possible).
pub fn sample_edge_curve(curve: &EdgeCurve, n: usize) -> Vec<Point3> {
    match curve {
        EdgeCurve::Circle(c) => (0..n)
            .map(|i| {
                #[allow(clippy::cast_precision_loss)]
                let t = std::f64::consts::TAU * (i as f64) / (n as f64);
                c.evaluate(t)
            })
            .collect(),
        EdgeCurve::Ellipse(e) => (0..n)
            .map(|i| {
                #[allow(clippy::cast_precision_loss)]
                let t = std::f64::consts::TAU * (i as f64) / (n as f64);
                e.evaluate(t)
            })
            .collect(),
        EdgeCurve::NurbsCurve(nc) => {
            let (u0, u1) = nc.domain();
            // For closed curves (start ~ end), use n as divisor to avoid
            // duplicating the first point at t=u_max.
            let start_pt = nc.evaluate(u0);
            let end_pt = nc.evaluate(u1);
            // A 1e-6 mm endpoint gap is treated as closed to avoid
            // duplicating the first point at t=u_max.
            let is_closed = (start_pt - end_pt).length() < CLOSED_CURVE_ENDPOINT_TOL_MM;
            let divisor = if is_closed { n } else { n - 1 };
            (0..n)
                .map(|i| {
                    #[allow(clippy::cast_precision_loss)]
                    let t = u0 + (u1 - u0) * (i as f64) / (divisor as f64);
                    nc.evaluate(t)
                })
                .collect()
        }
        // Never closed: an unbounded branch has no periodic domain, and
        // this entry point carries no endpoints to trim with.
        EdgeCurve::Line | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => vec![],
    }
}

/// Get a polygon approximation of a face by sampling curved edges.
///
/// Samples circle/ellipse edges into 32 points so faces with a
/// single closed-curve edge (e.g. cylinder caps) get a proper polygon.
///
/// # Errors
///
/// Returns an error if the face or its wire cannot be resolved.
pub fn face_polygon(
    topo: &Topology,
    face_id: FaceId,
) -> Result<Vec<Point3>, crate::OperationsError> {
    wire_polygon(topo, topo.face(face_id)?.outer_wire())
}

/// Get a polygon approximation of any wire by sampling its curved edges.
///
/// The same sampling [`face_polygon`] applies to a face's outer wire, exposed
/// for a face's INNER wires — a hole's rim is often a single closed circle
/// edge, which is one vertex and no polygon at all until it is sampled. Both
/// take the same number of samples, so a bore's rim traced as a hole in its
/// cap lands on the same positions as the same rim traced along its wall, and
/// the assembler's vertex dedup joins them.
///
/// # Errors
///
/// Returns an error if the wire or any of its edges cannot be resolved.
pub fn wire_polygon(
    topo: &Topology,
    wire_id: remus_topology::wire::WireId,
) -> Result<Vec<Point3>, crate::OperationsError> {
    let wire = topo.wire(wire_id)?;
    let mut pts = Vec::new();

    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let curve = edge.curve();
        // Sample closed parametric edges (start == end vertex).
        // Partial arcs fall through to the vertex-based path.
        let start_vid = edge.start();
        let end_vid = edge.end();
        let is_closed_edge = start_vid == end_vid
            && matches!(
                curve,
                EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) | EdgeCurve::NurbsCurve(_)
            );
        if is_closed_edge {
            // Must use CLOSED_CURVE_SAMPLES (not a larger value) — vertex count
            // must match create_band_fragments and inner-wire dedup for sharing.
            let mut sampled = sample_edge_curve(curve, CLOSED_CURVE_SAMPLES);
            if !oe.is_forward() {
                sampled.reverse();
            }
            pts.extend(sampled);
        } else {
            let vid = oe.oriented_start(edge);
            pts.push(topo.vertex(vid)?.point());
        }
    }

    Ok(pts)
}

/// Like [`wire_polygon`], but every closed edge returns to its own start
/// before the traversal moves on.
///
/// [`wire_polygon`] lays a closed circle edge down as an open chain of
/// samples, so the polygon's last-to-first step cuts across from the end of
/// one circle to the start of the next. On a cylindrical wall — two rim
/// circles joined by a seam — that means the wall's own rims are never
/// closed: it publishes 31 of each rim's 32 chords plus two diagonals, and
/// the cap that meets it along those rims cannot find the chords to share.
///
/// Here each circle closes, the seam is traversed once each way as a BRep
/// seam should be, and the rims come out as the full loops their neighbours
/// also trace. The repeat leaves consecutive duplicate positions, which the
/// assembler skips as degenerate.
///
/// # Errors
///
/// Returns an error if the wire or any of its edges cannot be resolved.
pub fn wire_polygon_closed_subloops(
    topo: &Topology,
    wire_id: remus_topology::wire::WireId,
) -> Result<Vec<Point3>, crate::OperationsError> {
    let wire = topo.wire(wire_id)?;
    let mut pts = Vec::new();

    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let curve = edge.curve();
        let is_closed_edge = edge.start() == edge.end()
            && matches!(
                curve,
                EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) | EdgeCurve::NurbsCurve(_)
            );
        if is_closed_edge {
            let mut sampled = sample_edge_curve(curve, CLOSED_CURVE_SAMPLES);
            if sampled.is_empty() {
                continue;
            }
            // `sample_edge_curve` starts at the CURVE's parameter origin,
            // which need not be the vertex the wire enters this edge at — a
            // cylinder's rim circle can be stored a quarter turn off its own
            // seam. Rotate so the loop opens and closes where the wire joins
            // it, or the sub-loop's closing chord lands a quarter of the way
            // round and the seam runs diagonally across the face.
            let anchor = topo.vertex(edge.start())?.point();
            if let Some(at) = sampled
                .iter()
                .enumerate()
                .min_by(|a, b| {
                    (*a.1 - anchor)
                        .length()
                        .partial_cmp(&(*b.1 - anchor).length())
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .map(|(i, _)| i)
            {
                sampled.rotate_left(at);
            }
            if !oe.is_forward() {
                // Reverse the traversal but keep the start vertex first: the
                // wire arrives at it, whichever way round the circle goes.
                sampled[1..].reverse();
            }
            pts.extend(sampled.iter().copied());
            pts.push(sampled[0]);
        } else {
            let vid = oe.oriented_start(edge);
            pts.push(topo.vertex(vid)?.point());
        }
    }

    Ok(pts)
}
