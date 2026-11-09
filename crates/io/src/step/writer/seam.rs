//! Seam placement for exported cylindrical bands.
//!
//! A remus face may keep its periodic seam wherever its carrier's `u = 0`
//! half-plane happens to fall, even when an inner wire (a window cut by a
//! boolean) straddles that half-plane: the kernel classifies such a face by
//! winding, so the seam edge passing through the window is harmless inside
//! remus. STEP has no such convention. ISO 10303-42 bounds a face by its
//! loops, so a seam edge running through a hole puts a boundary edge where
//! the face has no material, and a consumer that parameterizes the face by
//! its loops cannot close the window's p-curves across `u = 0`. A
//! reference reader can read such a file as one solid of the right volume, misclassify
//! interior points depending on which input owned the seam, and meshes it
//! with open and non-manifold edges (OpenZCAD #1243: two overlapping axial
//! cylinders, either Union order).
//!
//! The export therefore moves such a seam, on a private copy of the arena,
//! into the widest angular gap the inner wires leave. Only one layout is
//! qualified, because only there is the move a pure change of
//! parameterization:
//!
//! - the face lies on a `CylindricalSurface`;
//! - its outer wire is a full band: two closed full-turn circle edges, one
//!   on each rim, joined by one straight seam edge used once in each
//!   direction, the seam's two vertices being the circles' only vertices;
//! - no other exported edge touches those two vertices, and every other face
//!   using a rim circle is planar (a cap), so moving the rim vertex along its
//!   circle changes no other boundary;
//! - every inner wire consists of authoritative coaxial Circle arcs and
//!   axial Line generators, with exact angular coverage narrower than one
//!   full turn. Other representations retain their existing export route.
//!
//! The carrier is rotated about its own axis so the new seam is again
//! `u = 0`; geometry, vertices of every other edge, orientation and volume
//! are unchanged. Stored p-curves of the moved band and of the rim circles
//! are dropped from the copy rather than re-derived: every affected carrier
//! is analytic and the 3D edge curves stay exact, which is how the writer
//! already exports a use without a stored p-curve. Any other layout is
//! written exactly as before.

use std::f64::consts::{PI, TAU};

use remus_math::surfaces::CylindricalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Vec3;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::shell::ShellId;
use remus_topology::solid::SolidId;
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, WireId};

use super::topo_err;
use crate::IoError;

/// A qualified band whose seam must move, with everything needed to move it.
struct SeamMove {
    face: FaceId,
    seam: EdgeId,
    rims: [(EdgeId, VertexId); 2],
    /// Angle of the new seam in the carrier's current parameterization.
    angle: f64,
}

/// Returns a copy of `topo` whose qualified cylindrical bands no longer run a
/// seam through an inner wire, or `None` when no exported face needs it.
pub(super) fn relocate_crossed_seams(
    topo: &Topology,
    solids: &[SolidId],
    sheets: &[ShellId],
) -> Result<Option<Topology>, IoError> {
    let mut faces = Vec::new();
    for &solid in solids {
        faces.extend(solid_faces(topo, solid)?);
    }
    for &sheet in sheets {
        faces.extend_from_slice(topo.shell(sheet).map_err(topo_err)?.faces());
    }
    faces.sort_unstable_by_key(|face| face.index());
    faces.dedup();

    let mut moves = Vec::new();
    for &face in &faces {
        if let Some(seam_move) = crossed_band(topo, face)? {
            moves.push(seam_move);
        }
    }
    if moves.is_empty() {
        return Ok(None);
    }

    // Vertex incidence over every exported wire: a rim vertex that any
    // other edge touches cannot slide along its circle.
    let mut incident: std::collections::HashMap<VertexId, Vec<EdgeId>> =
        std::collections::HashMap::new();
    let mut circle_faces: std::collections::HashMap<EdgeId, Vec<FaceId>> =
        std::collections::HashMap::new();
    for &face in &faces {
        let face_data = topo.face(face).map_err(topo_err)?;
        for wire in
            std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire).map_err(topo_err)?.edges() {
                let edge = topo.edge(oriented.edge()).map_err(topo_err)?;
                for vertex in [edge.start(), edge.end()] {
                    let edges = incident.entry(vertex).or_default();
                    if !edges.contains(&oriented.edge()) {
                        edges.push(oriented.edge());
                    }
                }
                let users = circle_faces.entry(oriented.edge()).or_default();
                if !users.contains(&face) {
                    users.push(face);
                }
            }
        }
    }
    moves.retain(|seam_move| {
        seam_move.rims.iter().all(|&(circle, vertex)| {
            incident.get(&vertex).is_some_and(|edges| {
                edges.len() == 2 && edges.contains(&circle) && edges.contains(&seam_move.seam)
            }) && circle_faces.get(&circle).is_some_and(|users| {
                users.iter().all(|&user| {
                    user == seam_move.face
                        || topo
                            .face(user)
                            .is_ok_and(|face| matches!(face.surface(), FaceSurface::Plane { .. }))
                })
            })
        })
    });
    // Two moved bands sharing a rim circle would each slide its vertex.
    let mut claimed = std::collections::HashSet::new();
    moves.retain(|seam_move| {
        seam_move
            .rims
            .iter()
            .all(|&(circle, _)| claimed.insert(circle))
    });
    if moves.is_empty() {
        return Ok(None);
    }

    let mut copy = topo.clone();
    for seam_move in &moves {
        apply(&mut copy, seam_move)?;
    }
    Ok(Some(copy))
}

/// Recognizes the qualified band layout and decides whether, and where, its
/// seam must move.
fn crossed_band(topo: &Topology, face_id: FaceId) -> Result<Option<SeamMove>, IoError> {
    let face = topo.face(face_id).map_err(topo_err)?;
    let FaceSurface::Cylinder(cylinder) = face.surface() else {
        return Ok(None);
    };
    if face.inner_wires().is_empty() {
        return Ok(None);
    }
    let outer = topo.wire(face.outer_wire()).map_err(topo_err)?;
    let uses = outer.edges();
    if uses.len() != 4 {
        return Ok(None);
    }
    let mut seam = None;
    let mut rims = Vec::with_capacity(2);
    for oriented in uses {
        if analytic_angular_span(topo, oriented, cylinder)?.is_none() {
            return Ok(None);
        }
        let edge = topo.edge(oriented.edge()).map_err(topo_err)?;
        match edge.curve() {
            EdgeCurve::Line => match seam {
                None => seam = Some((oriented.edge(), oriented.is_forward(), 1_usize)),
                Some((id, forward, count))
                    if id == oriented.edge() && forward != oriented.is_forward() =>
                {
                    seam = Some((id, forward, count + 1));
                }
                Some(_) => return Ok(None),
            },
            EdgeCurve::Circle(_) => {
                let full_turn = edge.strict_domain().is_ok_and(|(t0, t1)| {
                    ((t1 - t0).abs() - TAU).abs() <= 4.0 * f64::EPSILON * TAU
                });
                if edge.start() != edge.end() || !full_turn {
                    return Ok(None);
                }
                rims.push((oriented.edge(), edge.start()));
            }
            EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => return Ok(None),
        }
    }
    let (Some((seam, _, 2)), [first, second]) = (seam, rims.as_slice()) else {
        return Ok(None);
    };
    let (first, second) = (*first, *second);
    if first.0 == second.0 || first.1 == second.1 {
        return Ok(None);
    }
    let seam_edge = topo.edge(seam).map_err(topo_err)?;
    if seam_edge.strict_domain().is_err() {
        return Ok(None);
    }
    let ends = [seam_edge.start(), seam_edge.end()];
    if !(ends.contains(&first.1) && ends.contains(&second.1)) {
        return Ok(None);
    }

    let seam_point = topo.vertex(first.1).map_err(topo_err)?.point();
    let seam_angle = cylinder.project_point(seam_point).0;
    // Angular clearance worth one linear tolerance at the band's radius.
    let margin = (remus_math::tolerance::Tolerance::new().linear / cylinder.radius()).max(1e-12);

    let mut intervals = Vec::with_capacity(face.inner_wires().len());
    for &wire in face.inner_wires() {
        let Some(interval) = angular_footprint(topo, wire, cylinder)? else {
            return Ok(None);
        };
        intervals.push(interval);
    }
    let crossed = intervals.iter().any(|&(lo, hi)| {
        let offset = (seam_angle - lo + margin).rem_euclid(TAU);
        offset <= hi - lo + 2.0 * margin
    });
    if !crossed {
        return Ok(None);
    }
    let Some(angle) = widest_gap_midpoint(&intervals, margin) else {
        return Ok(None);
    };
    Ok(Some(SeamMove {
        face: face_id,
        seam,
        rims: [first, second],
        angle,
    }))
}

/// Exact angular endpoints for a qualified coaxial circular arc or axial
/// line. Other curves keep the original export route: parameter sampling
/// cannot certify the angular extent of arbitrary rational boundaries.
fn analytic_angular_span(
    topo: &Topology,
    oriented: &OrientedEdge,
    cylinder: &CylindricalSurface,
) -> Result<Option<(f64, f64)>, IoError> {
    let edge = topo.edge(oriented.edge()).map_err(topo_err)?;
    let Ok((t0, t1)) = edge.strict_domain() else {
        return Ok(None);
    };
    let first = topo.vertex(edge.start()).map_err(topo_err)?.point();
    let last = topo.vertex(edge.end()).map_err(topo_err)?.point();
    let (start, end, t0, t1) = if oriented.is_forward() {
        (first, last, t0, t1)
    } else {
        (last, first, t1, t0)
    };
    let tol = Tolerance::new();
    let axis = cylinder.axis();
    let angle = cylinder.project_point(start).0;
    match edge.curve() {
        EdgeCurve::Circle(circle) => {
            let offset = circle.center() - cylinder.origin();
            let radial = offset - axis * offset.dot(axis);
            let tilt = circle.normal().cross(axis).length();
            if tilt > tol.angular
                || radial.length()
                    + circle.radius() * tilt
                    + (circle.radius() - cylinder.radius()).abs()
                    > tol.linear
                || (circle.evaluate(t0) - start).length() > tol.linear
                || (circle.evaluate(t1) - end).length() > tol.linear
            {
                return Ok(None);
            }
            let span = (t1 - t0) * circle.normal().dot(axis).signum();
            Ok(Some((angle, span)))
        }
        EdgeCurve::Line => {
            let from = start - cylinder.origin();
            let to = end - cylinder.origin();
            let a = from - axis * from.dot(axis);
            let b = to - axis * to.dot(axis);
            if (end - start).length() <= tol.linear
                || (end - start).cross(axis).length() > tol.linear
                || (a.length() - cylinder.radius()).abs() > tol.linear
                || (b.length() - cylinder.radius()).abs() > tol.linear
                || a.dot(b) <= 0.0
            {
                return Ok(None);
            }
            let finish = cylinder.project_point(end).0;
            Ok(Some((angle, (finish - angle + PI).rem_euclid(TAU) - PI)))
        }
        EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => Ok(None),
    }
}

/// Conservative unwrapped angular interval of a non-winding analytic window.
/// Circle spans use their authoritative domains, including major arcs;
/// constant-angle generators never require parameter-space sampling.
fn angular_footprint(
    topo: &Topology,
    wire: WireId,
    cylinder: &CylindricalSurface,
) -> Result<Option<(f64, f64)>, IoError> {
    let margin = (2.0 * Tolerance::new().linear / cylinder.radius()).max(128.0 * f64::EPSILON);
    let mut first = None;
    let mut previous = 0.0;
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for oriented in topo.wire(wire).map_err(topo_err)?.edges() {
        let Some((angle, span)) = analytic_angular_span(topo, oriented, cylinder)? else {
            return Ok(None);
        };
        if !angle.is_finite() || !span.is_finite() {
            return Ok(None);
        }
        let begin = if first.is_none() {
            first = Some(angle);
            angle
        } else {
            let lifted = angle + ((previous - angle) / TAU).round() * TAU;
            if (lifted - previous).abs() > 2.0 * margin {
                return Ok(None);
            }
            lifted
        };
        let end = begin + span;
        lo = lo.min(begin).min(end);
        hi = hi.max(begin).max(end);
        previous = end;
    }
    let Some(first) = first else {
        return Ok(None);
    };
    if (previous - first).abs() > 2.0 * margin || hi - lo + 2.0 * margin >= TAU {
        return Ok(None);
    }
    Ok(Some((lo - margin, hi + margin)))
}

/// Midpoint of the widest free angular gap between the intervals, if it
/// leaves at least `margin` of clearance on both sides.
fn widest_gap_midpoint(intervals: &[(f64, f64)], margin: f64) -> Option<f64> {
    let mut arcs: Vec<(f64, f64)> = intervals
        .iter()
        .map(|&(lo, hi)| {
            let start = (lo - margin).rem_euclid(TAU);
            (start, start + (hi - lo) + 2.0 * margin)
        })
        .collect();
    arcs.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut merged: Vec<(f64, f64)> = Vec::with_capacity(arcs.len());
    for (start, end) in arcs {
        match merged.last_mut() {
            Some(last) if start <= last.1 => last.1 = last.1.max(end),
            _ => merged.push((start, end)),
        }
    }
    let (first, last) = (*merged.first()?, *merged.last()?);
    // The last arc may run past a full turn and cover the first gaps.
    let wrapped = last.1 - TAU;
    let mut best: Option<(f64, f64)> = None;
    let mut consider = |from: f64, to: f64| {
        let from = from.max(wrapped);
        let width = to - from;
        if width > 2.0 * margin && best.is_none_or(|(_, w)| width > w) {
            best = Some((f64::midpoint(from, to), width));
        }
    };
    for pair in merged.windows(2) {
        consider(pair[0].1, pair[1].0);
    }
    consider(last.1, first.0 + TAU);
    best.map(|(angle, _)| angle.rem_euclid(TAU))
}

fn apply(topo: &mut Topology, seam_move: &SeamMove) -> Result<(), IoError> {
    let invalid = |reason: String| IoError::InvalidTopology { reason };
    let FaceSurface::Cylinder(cylinder) = topo.face(seam_move.face).map_err(topo_err)?.surface()
    else {
        return Err(invalid(
            "seam relocation lost its cylindrical carrier".into(),
        ));
    };
    let (sin, cos) = seam_move.angle.sin_cos();
    let reference: Vec3 = cylinder.x_axis() * cos + cylinder.y_axis() * sin;
    let rotated = remus_math::surfaces::CylindricalSurface::with_ref_dir(
        cylinder.origin(),
        cylinder.axis(),
        cylinder.radius(),
        reference,
    )
    .map_err(|e| invalid(format!("seam relocation frame: {e}")))?;

    for &(circle_id, vertex_id) in &seam_move.rims {
        let old = topo.vertex(vertex_id).map_err(topo_err)?.clone();
        let height = rotated.axis().dot(old.point() - rotated.origin());
        let point = rotated.evaluate(0.0, height);
        let edge = topo.edge(circle_id).map_err(topo_err)?;
        let EdgeCurve::Circle(circle) = edge.curve() else {
            return Err(invalid("seam relocation lost its rim circle".into()));
        };
        let (t0, t1) = edge
            .strict_domain()
            .map_err(|e| invalid(format!("rim circle domain: {e}")))?;
        let moved = circle.project(point);
        let start = t0 + (moved - t0).rem_euclid(TAU);
        let residual = (circle.evaluate(start) - point).length();
        if !residual.is_finite() || residual > old.tolerance() {
            return Err(invalid(format!(
                "seam relocation leaves its rim vertex {residual} off the rim circle"
            )));
        }
        topo.edge_mut(circle_id)
            .map_err(topo_err)?
            .set_trim(Some((start, start + (t1 - t0))));
        *topo.vertex_mut(vertex_id).map_err(topo_err)? = Vertex::new(point, old.tolerance());
    }
    topo.face_mut(seam_move.face)
        .map_err(topo_err)?
        .set_surface(FaceSurface::Cylinder(rotated));

    let mut stale = topo.coedges_of_edge(seam_move.seam);
    for &(circle, _) in &seam_move.rims {
        stale.extend(topo.coedges_of_edge(circle));
    }
    let loops = topo
        .face(seam_move.face)
        .map_err(topo_err)?
        .boundary_loops()
        .to_vec();
    for loop_id in loops {
        stale.extend_from_slice(topo.face_loop(loop_id).map_err(topo_err)?.coedges());
    }
    stale.sort_unstable_by_key(|coedge| coedge.index());
    stale.dedup();
    for coedge in stale {
        topo.remove_coedge_pcurve(coedge).map_err(topo_err)?;
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use remus_math::curves::Circle3D;
    use remus_math::nurbs::NurbsCurve;
    use remus_operations::primitives::make_cylinder;
    use remus_topology::edge::Edge;
    use remus_topology::wire::Wire;

    use super::*;

    /// A cylindrical band with a window bounded by two circular arcs and
    /// two generators. Optionally store the exact same arcs as rational
    /// quadratics whose first three spans occupy only 3e-9 of the domain.
    fn window_band(start: f64, end: f64, compressed: bool) -> (Topology, FaceId, WireId) {
        let mut topo = Topology::new();
        let solid = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
        let face = solid_faces(&topo, solid)
            .unwrap()
            .into_iter()
            .find(|&id| matches!(topo.face(id).unwrap().surface(), FaceSurface::Cylinder(_)))
            .unwrap();
        let FaceSurface::Cylinder(cylinder) = topo.face(face).unwrap().surface().clone() else {
            unreachable!();
        };
        // Give the source seam parameter zero. The native cylinder's
        // default carrier frame and its seam vertices use different axes.
        let outer = topo.face(face).unwrap().outer_wire();
        let edge = topo
            .edge(topo.wire(outer).unwrap().edges()[0].edge())
            .unwrap();
        let offset = topo.vertex(edge.start()).unwrap().point() - cylinder.origin();
        let reference = offset - cylinder.axis() * offset.dot(cylinder.axis());
        let cylinder = CylindricalSurface::with_ref_dir(
            cylinder.origin(),
            cylinder.axis(),
            cylinder.radius(),
            reference,
        )
        .unwrap();
        topo.face_mut(face)
            .unwrap()
            .set_surface(FaceSurface::Cylinder(cylinder.clone()));
        let mut arcs = Vec::new();
        for height in [0.5, 1.5] {
            let circle = Circle3D::new_with_ref(
                cylinder.origin() + cylinder.axis() * height,
                cylinder.axis(),
                cylinder.radius(),
                cylinder.x_axis(),
            )
            .unwrap();
            let first = topo.add_vertex(Vertex::new(circle.evaluate(start), 1e-7));
            let last = topo.add_vertex(Vertex::new(circle.evaluate(end), 1e-7));
            let (curve, domain) = if compressed {
                let step = (end - start) / 4.0;
                let weight = (step / 2.0).cos();
                let mut points = vec![circle.evaluate(start)];
                let mut weights = vec![1.0];
                for i in 0..4 {
                    let middle = start + (f64::from(i) + 0.5) * step;
                    points.push(
                        circle.center()
                            + (circle.evaluate(middle) - circle.center()) * weight.recip(),
                    );
                    points.push(circle.evaluate(start + f64::from(i + 1) * step));
                    weights.extend([weight, 1.0]);
                }
                let knots = vec![
                    0.0, 0.0, 0.0, 1e-9, 1e-9, 2e-9, 2e-9, 3e-9, 3e-9, 1.0, 1.0, 1.0,
                ];
                (
                    EdgeCurve::NurbsCurve(NurbsCurve::new(2, knots, points, weights).unwrap()),
                    (0.0, 1.0),
                )
            } else {
                (EdgeCurve::Circle(circle), (start, end))
            };
            let mut edge = Edge::new(first, last, curve);
            edge.set_trim(Some(domain));
            arcs.push((topo.add_edge(edge), first, last));
        }
        let right = topo.add_edge(Edge::new(arcs[0].2, arcs[1].2, EdgeCurve::Line));
        let left = topo.add_edge(Edge::new(arcs[1].1, arcs[0].1, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(arcs[0].0, true),
                    OrientedEdge::new(right, true),
                    OrientedEdge::new(arcs[1].0, false),
                    OrientedEdge::new(left, true),
                ],
                true,
            )
            .unwrap(),
        );
        let outer = topo.face(face).unwrap().outer_wire();
        topo.set_face_boundary_wires(face, outer, vec![wire])
            .unwrap();
        (topo, face, wire)
    }

    #[test]
    fn major_circle_domains_keep_the_entire_window_footprint() {
        let (topo, face, wire) = window_band(0.5, 6.0, false);
        let FaceSurface::Cylinder(cylinder) = topo.face(face).unwrap().surface() else {
            unreachable!();
        };
        let (lo, hi) = angular_footprint(&topo, wire, cylinder).unwrap().unwrap();
        assert!(lo <= 0.5 && hi >= 6.0 && hi - lo < TAU, "{lo}..{hi}");
        assert!(
            crossed_band(&topo, face).unwrap().is_none(),
            "the original seam is clear"
        );
    }

    #[test]
    fn compressed_rational_spans_cannot_move_a_clear_seam_into_a_window() {
        let (topo, face, _) = window_band(0.5, 6.0, true);
        // Uniform 64- and 1024-step samples miss three exact arc spans.
        // The old writer chose angle 2.57245, inside this major-arc window.
        // This representation needs certified bounds before relocation.
        if let Some(seam_move) = crossed_band(&topo, face).unwrap() {
            assert!(
                seam_move.angle < 0.5 || seam_move.angle > 6.0,
                "{}",
                seam_move.angle
            );
        }
    }

    #[test]
    fn explicit_canonical_line_domains_allow_the_same_seam_repair() {
        let (mut topo, face, _) = window_band(-0.4, 0.4, false);
        let expected = crossed_band(&topo, face).unwrap().unwrap().angle;
        let outer = topo.face(face).unwrap().outer_wire();
        let seam = topo
            .wire(outer)
            .unwrap()
            .edges()
            .iter()
            .find(|oriented| matches!(topo.edge(oriented.edge()).unwrap().curve(), EdgeCurve::Line))
            .unwrap()
            .edge();
        topo.edge_mut(seam).unwrap().set_trim(Some((0.0, 1.0)));
        let actual = crossed_band(&topo, face).unwrap().unwrap().angle;
        assert!((actual - expected).abs() < 1e-12);
    }

    #[test]
    fn antiparallel_arc_normals_with_reversed_domains_keep_the_same_footprint() {
        let (mut topo, face, wire) = window_band(-0.4, 0.4, false);
        let expected = crossed_band(&topo, face).unwrap().unwrap().angle;
        let uses = topo.wire(wire).unwrap().edges().to_vec();
        for oriented in uses {
            let edge = topo.edge_mut(oriented.edge()).unwrap();
            if let EdgeCurve::Circle(circle) = edge.curve() {
                let reversed = circle.reversed();
                let (t0, t1) = edge.strict_domain().unwrap();
                edge.set_curve(EdgeCurve::Circle(reversed));
                edge.set_trim(Some((-t0, -t1)));
            }
        }
        let actual = crossed_band(&topo, face).unwrap().unwrap().angle;
        assert!((actual - expected).abs() < 1e-12);
    }

    #[test]
    fn widest_gap_skips_arcs_that_wrap_past_a_full_turn() {
        // [5.5, 7.3] wraps to cover [0, 1.0168...], hiding the gap between
        // the two small arcs near zero; the free gap is (1.2, 3.0).
        let intervals = [(0.2, 0.3), (0.5, 0.6), (3.0, 4.0), (5.5, 7.3), (1.1, 1.2)];
        let angle = widest_gap_midpoint(&intervals, 1e-12).unwrap();
        assert!((angle - 2.1).abs() < 1e-9, "{angle}");
    }

    #[test]
    fn widest_gap_crosses_zero_when_it_is_the_widest() {
        let intervals = [(-0.3 + TAU, 0.1 + TAU), (1.0, 2.0)];
        let angle = widest_gap_midpoint(&intervals, 1e-12).unwrap();
        // Free arcs: (0.1, 1.0) and (2.0, 5.983...): the second is widest.
        assert!(
            (angle - f64::midpoint(2.0, TAU - 0.3)).abs() < 1e-9,
            "{angle}"
        );
    }

    #[test]
    fn no_gap_when_the_arcs_cover_the_circle() {
        assert!(widest_gap_midpoint(&[(0.0, 3.2), (3.1, 6.4)], 1e-12).is_none());
    }
}
