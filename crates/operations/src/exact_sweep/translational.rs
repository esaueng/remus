//! Exact sweep that keeps the profile's orientation: the profile is
//! translated along the path, never turned, so every section is the profile
//! itself moved to the path point it stands on.
//!
//! Translation carries a profile curve `Q(u) = Σ R_i(u)·Q_i` along a path
//! piece `C(v) = Σ S_j(v)·C_j` to the translational surface
//!
//! ```text
//! T(u, v) = Q(u) + C(v) − C(v₀)
//! ```
//!
//! which is exactly the tensor-product NURBS with control points
//! `Q_i + C_j − C(v₀)` and weights `w_i·ω_j`: over the common denominator
//! `Σ w_i R_i · Σ ω_j S_j` the sum of the two rational curves is one rational
//! surface. So any path piece the kernel holds exactly — a line, a circular
//! or elliptic arc, a conic or a spline — sweeps exactly, and so does any
//! profile curve. A line piece sweeps the same faces as the rigid sweep
//! (planes, cylinders, ruled NURBS); every other piece sweeps the
//! translational NURBS. Rails are the path pieces translated, sections the
//! profile translated, edge for edge.
//!
//! The path need not be smooth: at a corner the sections simply continue,
//! and the body creases there. Nothing is mitred, sampled or fitted.
//!
//! ## Self-clearance
//!
//! The body is the region `{x + C(t) − C(t₀)}` over profile points `x` and
//! path parameters `t`. If the path is strictly monotone along the profile's
//! normal `n`, two sections never meet: `x + C(t) = y + C(t')` gives
//! `n·(C(t') − C(t)) = n·(x − y) = 0`, so `t = t'` and `x = y`. That is
//! certified, not sampled, on the path's Bézier segments: with positive
//! weights a rational Bézier whose control points rise strictly along `n`
//! rises strictly itself (for Bernstein polynomials of degree `p`,
//! `B_i·B_j' − B_i'·B_j = (j − i)·B_i·B_j / (t(1 − t)) > 0` for `j > i`). A
//! path the check cannot certify, a closed one among them, is refused.

use remus_math::nurbs::decompose::curve_to_bezier_segments;

use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use super::{
    Loop, Motion, RELATIVE_MARGIN, Rigid, Section, analytic_side, invalid, is_ccw,
    read_profile_loop, to_vec,
};
use crate::OperationsError;
use crate::helix::in_place::exact_piece;

/// One path piece, oriented along the chain.
#[derive(Debug, Clone)]
struct Step {
    start: Point3,
    end: Point3,
    kind: StepKind,
}

#[derive(Debug, Clone)]
enum StepKind {
    /// A line: the rigid sweep's translation.
    Line(Vec3),
    /// Any other curve, as its exact NURBS from `start` to `end`.
    Curve(NurbsCurve),
}

impl Step {
    fn reversed(&self) -> Self {
        Self {
            start: self.end,
            end: self.start,
            kind: match &self.kind {
                StepKind::Line(d) => StepKind::Line(-*d),
                StepKind::Curve(c) => StepKind::Curve(c.reversed()),
            },
        }
    }
}

/// Read one path edge in its natural direction.
fn read_step(topo: &Topology, edge_id: EdgeId, index: usize) -> Result<Step, OperationsError> {
    let edge = topo.edge(edge_id)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    if matches!(edge.curve(), EdgeCurve::Line) {
        let d = end - start;
        if d.normalize().is_err() {
            return Err(invalid(format!(
                "exact sweep path edge {index} has zero length"
            )));
        }
        return Ok(Step {
            start,
            end,
            kind: StepKind::Line(d),
        });
    }
    let domain = match edge.strict_domain() {
        Ok(domain) => domain,
        Err(_) => crate::reconstruct_legacy_edge_domain(topo, edge_id, "exact sweep path edge")?,
    };
    if !(domain.1 - domain.0).is_finite() || (domain.1 - domain.0).abs() <= 0.0 {
        return Err(invalid(format!(
            "exact sweep path edge {index} has an empty parameter range"
        )));
    }
    let (_, _, mut nurbs) = exact_piece(edge.curve(), domain, start, end)?;
    // The rails and sections share the path's vertices exactly: the curve
    // must start and end on its control polygon's ends (a clamped spline;
    // an unclamped one's ends are not control points) and on the vertices.
    let linear = Tolerance::new().linear;
    let snap = 10.0
        * linear
            .max(topo.vertex(edge.start())?.tolerance())
            .max(topo.vertex(edge.end())?.tolerance());
    let mut points = nurbs.control_points().to_vec();
    let last = points.len() - 1;
    let (d0, d1) = nurbs.domain();
    let unclamped = (nurbs.evaluate(d0) - points[0])
        .length()
        .max((nurbs.evaluate(d1) - points[last]).length());
    if unclamped.is_nan() || unclamped > snap {
        return Err(invalid(format!(
            "exact sweep path edge {index} is a spline that does not end on its control \
             points (an unclamped knot vector); clamp it before sweeping"
        )));
    }
    let miss = (points[0] - start)
        .length()
        .max((points[last] - end).length());
    if miss.is_nan() || miss > snap {
        return Err(invalid(format!(
            "exact sweep path edge {index} misses its vertices by {miss}"
        )));
    }
    points[0] = start;
    points[last] = end;
    nurbs = NurbsCurve::new(
        nurbs.degree(),
        nurbs.knots().to_vec(),
        points,
        nurbs.weights().to_vec(),
    )?;
    Ok(Step {
        start,
        end,
        kind: StepKind::Curve(nurbs),
    })
}

/// Order the path edges into one open chain, whatever order they come in.
fn read_steps(topo: &Topology, path: &[EdgeId], linear: f64) -> Result<Vec<Step>, OperationsError> {
    if path.is_empty() {
        return Err(invalid("exact sweep needs at least one path edge"));
    }
    let raw: Vec<Step> = path
        .iter()
        .enumerate()
        .map(|(i, &e)| read_step(topo, e, i))
        .collect::<Result<_, _>>()?;
    let scale = raw
        .iter()
        .map(|s| to_vec(s.start).length().max(to_vec(s.end).length()))
        .fold(1.0, f64::max);
    let join = (10.0 * linear).max(RELATIVE_MARGIN * scale);
    let meets = |a: Point3, b: Point3| (a - b).length() <= join;
    let ends_at = |p: Point3| {
        raw.iter()
            .map(|s| usize::from(meets(s.start, p)) + usize::from(meets(s.end, p)))
            .sum::<usize>()
    };
    for (i, s) in raw.iter().enumerate() {
        for point in [s.start, s.end] {
            if ends_at(point) > 2 {
                return Err(invalid(format!(
                    "exact sweep path branches at an end of path edge {i}; a sweep path is \
                     one chain"
                )));
            }
        }
    }
    let Some(first) = raw.iter().enumerate().find_map(|(i, s)| {
        if ends_at(s.start) == 1 {
            Some((i, s.clone()))
        } else if ends_at(s.end) == 1 {
            Some((i, s.reversed()))
        } else {
            None
        }
    }) else {
        return Err(invalid(
            "a closed sweep path cannot keep the profile's orientation: the profile would \
             come back through itself",
        ));
    };
    let mut used = vec![false; raw.len()];
    used[first.0] = true;
    let mut chain = vec![first.1];
    while chain.len() < raw.len() {
        let tail = chain[chain.len() - 1].end;
        let next = raw.iter().enumerate().find_map(|(i, s)| {
            if used[i] {
                None
            } else if meets(s.start, tail) {
                Some((i, s.clone()))
            } else if meets(s.end, tail) {
                Some((i, s.reversed()))
            } else {
                None
            }
        });
        let Some((i, step)) = next else {
            return Err(invalid(format!(
                "exact sweep path edges do not form one connected chain ({} of {} meet end to \
                 end)",
                chain.len(),
                raw.len()
            )));
        };
        used[i] = true;
        chain.push(step);
    }
    Ok(chain)
}

/// The sign with which `step` runs along `normal`, when it provably runs
/// strictly one way by more than `margin` over every Bézier span.
fn monotone_sign(step: &Step, normal: Vec3, margin: f64) -> Option<f64> {
    match &step.kind {
        StepKind::Line(d) => {
            let rise = d.dot(normal);
            (rise.abs() > margin).then(|| rise.signum())
        }
        StepKind::Curve(curve) => {
            if curve.weights().iter().any(|&w| w.is_nan() || w <= 0.0) {
                return None;
            }
            let mut sign: Option<f64> = None;
            // A curve the decomposition cannot split is not certified.
            let Ok(segments) = curve_to_bezier_segments(curve) else {
                return None;
            };
            for segment in segments {
                if segment.weights().iter().any(|&w| w.is_nan() || w <= 0.0) {
                    return None;
                }
                let heights: Vec<f64> = segment
                    .control_points()
                    .iter()
                    .map(|&p| to_vec(p).dot(normal))
                    .collect();
                for pair in heights.windows(2) {
                    let rise = pair[1] - pair[0];
                    if rise.abs() <= margin {
                        return None;
                    }
                    if (*sign.get_or_insert_with(|| rise.signum()) > 0.0) != (rise > 0.0) {
                        return None;
                    }
                }
            }
            sign
        }
    }
}

/// Boundary samples in traversal order, several in every knot span of every
/// piece, so a loop's winding is read from its whole shape (uniform samples
/// in a piece's parameter can all fall on a few spans of a spline).
fn winding_samples(lp: &Loop) -> Vec<Point3> {
    const PER_SPAN: usize = 8;
    let mut out = Vec::new();
    for piece in &lp.pieces {
        let (u0, u1) = piece.nurbs.domain();
        let mut breaks: Vec<f64> = piece
            .nurbs
            .knots()
            .iter()
            .copied()
            .filter(|&k| k > u0 && k < u1)
            .collect();
        breaks.insert(0, u0);
        breaks.push(u1);
        breaks.dedup();
        let mut params = Vec::with_capacity(breaks.len() * PER_SPAN);
        for pair in breaks.windows(2) {
            for j in 0..PER_SPAN {
                #[allow(clippy::cast_precision_loss)]
                let f = j as f64 / PER_SPAN as f64;
                params.push((pair[1] - pair[0]).mul_add(f, pair[0]));
            }
        }
        if !piece.forward {
            params.reverse();
            // Traversal starts at the piece's natural end.
            params.rotate_right(1);
            params[0] = u1;
        }
        out.extend(params.iter().map(|&u| piece.nurbs.evaluate(u)));
    }
    out
}

/// The translational surface of the placed profile curve `q` along `path`
/// (see the [module docs](self)): `u` is `q`'s parameter, `v` the path's.
fn translational_surface(
    q: &NurbsCurve,
    path: &NurbsCurve,
) -> Result<NurbsSurface, OperationsError> {
    let origin = to_vec(path.control_points()[0]);
    let control_points: Vec<Vec<Point3>> = q
        .control_points()
        .iter()
        .map(|&qi| {
            path.control_points()
                .iter()
                .map(|&cj| qi + (to_vec(cj) - origin))
                .collect()
        })
        .collect();
    let weights: Vec<Vec<f64>> = q
        .weights()
        .iter()
        .map(|&wi| path.weights().iter().map(|&wj| wi * wj).collect())
        .collect();
    Ok(NurbsSurface::new(
        q.degree(),
        path.degree(),
        q.knots().to_vec(),
        path.knots().to_vec(),
        control_points,
        weights,
    )?)
}

/// `curve` moved by `offset`.
fn translated(curve: &NurbsCurve, offset: Vec3) -> Result<NurbsCurve, OperationsError> {
    Ok(NurbsCurve::new(
        curve.degree(),
        curve.knots().to_vec(),
        curve.control_points().iter().map(|&p| p + offset).collect(),
        curve.weights().to_vec(),
    )?)
}

/// Sweep the planar face `profile` along `path` keeping its orientation (see
/// the [module docs](self)), from the chain end nearer the profile's plane.
#[allow(clippy::too_many_lines)]
pub(super) fn sweep_keeping_orientation_impl(
    topo: &mut Topology,
    profile: FaceId,
    path: &[EdgeId],
) -> Result<SolidId, OperationsError> {
    let linear = Tolerance::new().linear;
    let face = topo.face(profile)?;
    let FaceSurface::Plane { normal, d: plane_d } = *face.surface() else {
        return Err(invalid(format!(
            "an exact sweep needs a planar profile, got a {} face",
            face.surface().type_tag()
        )));
    };
    let normal = normal.normalize()?;
    let outer_wire = face.outer_wire();
    let inner_wires = face.inner_wires().to_vec();
    let mut loops: Vec<Loop> = vec![read_profile_loop(topo, outer_wire, true, "exact sweep")?];
    for wire in inner_wires {
        loops.push(read_profile_loop(topo, wire, false, "exact sweep")?);
    }

    let mut chain = read_steps(topo, path, linear)?;
    let plane_distance = |p: Point3| (to_vec(p).dot(normal) - plane_d).abs();
    if plane_distance(chain[chain.len() - 1].end) < plane_distance(chain[0].start) {
        chain = chain.iter().rev().map(Step::reversed).collect();
    }
    let n = chain.len();
    let samples: Vec<Vec<Point3>> = loops.iter().map(Loop::samples).collect();
    let windings: Vec<Vec<Point3>> = loops.iter().map(winding_samples).collect();
    let scale = samples
        .iter()
        .flatten()
        .map(|&p| to_vec(p).length())
        .chain(chain.iter().map(|s| to_vec(s.end).length()))
        .fold(1.0, f64::max);
    let margin = (RELATIVE_MARGIN * scale).max(linear);

    // Certified monotone along the profile's normal, one way throughout.
    let mut sigma = None;
    for (k, step) in chain.iter().enumerate() {
        let sign = monotone_sign(step, normal, margin);
        let Some(sign) = sign else {
            return Err(invalid(format!(
                "path segment {k} cannot be shown to run steadily through the profile's plane: \
                 a sweep that keeps the profile's orientation could meet itself (the path must \
                 keep moving away from the profile)"
            )));
        };
        if (*sigma.get_or_insert(sign) > 0.0) != (sign > 0.0) {
            return Err(invalid(format!(
                "path segment {k} turns back through the profile's plane: a sweep that keeps \
                 the profile's orientation would meet itself"
            )));
        }
    }
    let sigma = sigma.unwrap_or(1.0);

    // `placements[k]` moves the profile to the start of step k.
    // Each section is the last one moved by the step between them, so the
    // sections are exactly where each step's surfaces and rails end even
    // where the chain joined two pieces within the join tolerance.
    let mut placements: Vec<Rigid> = Vec::with_capacity(n + 1);
    placements.push(Rigid::IDENTITY);
    for step in &chain {
        let last = placements[placements.len() - 1];
        placements.push(Rigid {
            translation: last.translation + (step.end - step.start),
            ..Rigid::IDENTITY
        });
    }

    let mut sections: Vec<Section> = Vec::with_capacity(n + 1);
    for (k, placement) in placements.iter().enumerate() {
        let mut vertices = Vec::with_capacity(loops.len());
        let mut edges = Vec::with_capacity(loops.len());
        for lp in &loops {
            let ids: Vec<VertexId> = lp
                .points
                .iter()
                .map(|&(p, tol)| topo.add_vertex(Vertex::new(placement.point(p), tol)))
                .collect();
            let mut loop_edges = Vec::with_capacity(lp.pieces.len());
            for (i, piece) in lp.pieces.iter().enumerate() {
                let (ns, ne) = lp.natural_ends(i);
                let curve = if k == 0 {
                    piece.cap.clone()
                } else {
                    placement.curve(&piece.cap)?
                };
                let mut edge = Edge::new(ids[ns], ids[ne], curve);
                edge.set_trim(piece.trim);
                loop_edges.push(topo.add_edge(edge));
            }
            vertices.push(ids);
            edges.push(loop_edges);
        }
        sections.push(Section { vertices, edges });
    }

    let mut side_faces = Vec::new();
    for (k, step) in chain.iter().enumerate() {
        let placement = placements[k];
        for (l, lp) in loops.iter().enumerate() {
            let ccw = is_ccw(&windings[l], normal);
            let mut rails = Vec::with_capacity(lp.points.len());
            for (i, &(p, _)) in lp.points.iter().enumerate() {
                let (curve, trim) = match &step.kind {
                    StepKind::Line(_) => (EdgeCurve::Line, None),
                    StepKind::Curve(c) => {
                        let rail = translated(c, placement.point(p) - step.start)?;
                        let domain = rail.domain();
                        (EdgeCurve::NurbsCurve(rail), Some(domain))
                    }
                };
                let mut edge = Edge::new(
                    sections[k].vertices[l][i],
                    sections[k + 1].vertices[l][i],
                    curve,
                );
                edge.set_trim(trim);
                rails.push(topo.add_edge(edge));
            }
            for (i, piece) in lp.pieces.iter().enumerate() {
                let (ns, ne) = lp.natural_ends(i);
                let nurbs = placement.nurbs(&piece.nurbs)?;
                let (u0, u1) = nurbs.domain();
                let u_mid = f64::midpoint(u0, u1);
                let probe = nurbs.evaluate(u_mid);
                let tangent = nurbs.tangent(u_mid)?;
                let traversal = if piece.forward { tangent } else { -tangent };
                let side = if ccw == lp.outer { 1.0 } else { -1.0 };
                let outward = traversal.cross(normal) * side;
                let analytic = match &step.kind {
                    StepKind::Line(d) => analytic_side(
                        &placement.curve(&piece.cap)?,
                        placement.point(lp.points[ns].0),
                        placement.point(lp.points[ne].0),
                        &Motion::Translate(*d),
                        probe,
                        linear,
                    )?
                    .map(|surface| (surface, *d)),
                    StepKind::Curve(_) => None,
                };
                let (surface, flip, reversed) = match analytic {
                    Some((FaceSurface::Plane { normal: n, .. }, d)) => {
                        let natural = tangent.cross(d);
                        let n = if n.dot(outward) < 0.0 { -n } else { n };
                        (
                            FaceSurface::Plane {
                                normal: n,
                                d: crate::dot_normal_point(n, probe),
                            },
                            n.dot(natural) < 0.0,
                            false,
                        )
                    }
                    Some((surface, d)) => {
                        let natural = tangent.cross(d);
                        let (u, v) = surface
                            .project_point(probe)
                            .ok_or_else(|| invalid("exact sweep side surface has no chart"))?;
                        let n = surface.normal(u, v);
                        let flip = n.dot(natural) < 0.0;
                        let reversed = n.dot(outward) < 0.0;
                        (surface, flip, reversed)
                    }
                    None => {
                        let (surface, v0) = match &step.kind {
                            StepKind::Line(d) => {
                                (Motion::Translate(*d).nurbs_surface(&nurbs)?, 0.0)
                            }
                            StepKind::Curve(c) => (translational_surface(&nurbs, c)?, c.domain().0),
                        };
                        let reversed = surface.normal(u_mid, v0)?.dot(outward) < 0.0;
                        (FaceSurface::Nurbs(surface), false, reversed)
                    }
                };
                let bottom = sections[k].edges[l][i];
                let top = sections[k + 1].edges[l][i];
                let edges = if flip {
                    vec![
                        OrientedEdge::new(rails[ns], true),
                        OrientedEdge::new(top, true),
                        OrientedEdge::new(rails[ne], false),
                        OrientedEdge::new(bottom, false),
                    ]
                } else {
                    vec![
                        OrientedEdge::new(bottom, true),
                        OrientedEdge::new(rails[ne], true),
                        OrientedEdge::new(top, false),
                        OrientedEdge::new(rails[ns], false),
                    ]
                };
                let wire = topo.add_wire(Wire::new(edges, true)?);
                let mut face = Face::new(wire, vec![], surface);
                if reversed {
                    face.set_reversed(true);
                }
                side_faces.push(topo.add_face(face));
            }
        }
    }

    // Caps: the start cap faces −σ n, the end cap σ n; a translation keeps
    // the winding.
    let cap_wire = |topo: &mut Topology,
                    lp: &Loop,
                    edges: &[EdgeId],
                    ccw: bool,
                    along_n: bool|
     -> Result<WireId, OperationsError> {
        let oriented: Vec<OrientedEdge> = if (ccw == along_n) == lp.outer {
            edges
                .iter()
                .zip(&lp.pieces)
                .map(|(&e, piece)| OrientedEdge::new(e, piece.forward))
                .collect()
        } else {
            edges
                .iter()
                .zip(&lp.pieces)
                .rev()
                .map(|(&e, piece)| OrientedEdge::new(e, !piece.forward))
                .collect()
        };
        Ok(topo.add_wire(Wire::new(oriented, true)?))
    };
    let mut start_wires = Vec::with_capacity(loops.len());
    let mut end_wires = Vec::with_capacity(loops.len());
    for (l, lp) in loops.iter().enumerate() {
        let ccw = is_ccw(&windings[l], normal);
        start_wires.push(cap_wire(topo, lp, &sections[0].edges[l], ccw, sigma < 0.0)?);
        end_wires.push(cap_wire(topo, lp, &sections[n].edges[l], ccw, sigma > 0.0)?);
    }
    let start_point = loops[0].points[0].0;
    let start_normal = -normal * sigma;
    let end_normal = normal * sigma;
    let end_point = placements[n].point(start_point);
    let mut faces = Vec::with_capacity(side_faces.len() + 2);
    faces.push(topo.add_face(Face::new(
        start_wires[0],
        start_wires[1..].to_vec(),
        FaceSurface::Plane {
            normal: start_normal,
            d: crate::dot_normal_point(start_normal, start_point),
        },
    )));
    faces.extend(side_faces);
    faces.push(topo.add_face(Face::new(
        end_wires[0],
        end_wires[1..].to_vec(),
        FaceSurface::Plane {
            normal: end_normal,
            d: crate::dot_normal_point(end_normal, end_point),
        },
    )));
    let shell = topo.add_shell(Shell::new(faces)?);
    Ok(topo.add_solid(Solid::new(shell, vec![])))
}
