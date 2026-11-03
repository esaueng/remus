//! Exact circular caps and full cylindrical bands with two seam branches.

use remus_math::curves2d::{Circle2D, Curve2D, Line2D};
use remus_math::frame::Frame3;
use remus_math::vec::{Point2, Point3, Vec2};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::pcurve::PCurve;
use remus_topology::validation::{check_same_parameter_strict, check_same_range_strict};
use remus_topology::wire::{OrientedEdge, Wire};
use remus_topology::{
    Topology,
    edge::{EdgeCurve, EdgeId},
};

use super::carriers::{Circle2, Curve2, Curve3, Cylinder, Surface, full_turn};
use super::{
    Boundaries, Document, Geometry, IoError, Plane, Usage, ensure_limit, malformed, unsupported,
};

#[derive(Clone)]
struct BoundUse {
    usage: Usage,
    pcurve: PCurve,
}

struct Band<'a> {
    choices: &'a [Vec<BoundUse>],
    edges: &'a [Option<EdgeId>],
    topo: &'a Topology,
    radius: f64,
    tolerance: f64,
    solutions: Vec<Vec<BoundUse>>,
}

impl Band<'_> {
    fn joins(&self, left: &BoundUse, right: &BoundUse) -> Result<bool, IoError> {
        let edge = self
            .topo
            .edge(self.edges[left.usage.index].ok_or_else(|| malformed("missing band edge"))?)?;
        let next = self
            .topo
            .edge(self.edges[right.usage.index].ok_or_else(|| malformed("missing band edge"))?)?;
        let end = if left.usage.forward {
            edge.end()
        } else {
            edge.start()
        };
        let start = if right.usage.forward {
            next.start()
        } else {
            next.end()
        };
        let a = left.pcurve.evaluate(left.pcurve.t_end());
        let b = right.pcurve.evaluate(right.pcurve.t_start());
        let residual = ((a.x() - b.x()) * self.radius).hypot(a.y() - b.y());
        Ok(end == start && residual.is_finite() && residual <= self.tolerance)
    }

    // Depth is at most four: this is a fixed periodic-band boundary, not a
    // recursive traversal of the attacker-controlled shape graph.
    fn search(&mut self, path: &mut Vec<(usize, usize)>, mask: u8) -> Result<(), IoError> {
        if self.solutions.len() > 1 {
            return Ok(());
        }
        if path.len() == 4 {
            let first = &self.choices[path[0].0][path[0].1];
            let last = &self.choices[path[3].0][path[3].1];
            if self.joins(last, first)? {
                self.solutions.push(
                    path.iter()
                        .map(|&(i, j)| self.choices[i][j].clone())
                        .collect(),
                );
            }
            return Ok(());
        }
        for next in 0..4 {
            if mask & (1 << next) != 0 || (path.is_empty() && next != 0) {
                continue;
            }
            for branch in 0..self.choices[next].len() {
                if let Some(&(i, j)) = path.last()
                    && !self.joins(&self.choices[i][j], &self.choices[next][branch])?
                {
                    continue;
                }
                path.push((next, branch));
                self.search(path, mask | (1 << next))?;
                path.pop();
            }
        }
        Ok(())
    }
}

fn chart(plane: &Plane) -> Result<Frame3, IoError> {
    let d = plane.normal.dot(plane.origin - Point3::new(0., 0., 0.));
    let k = d / plane.normal.length_squared();
    Frame3::from_normal(
        Point3::new(
            plane.normal.x() * k,
            plane.normal.y() * k,
            plane.normal.z() * k,
        ),
        plane.normal,
    )
    .map_err(|_| malformed("invalid circular plane chart"))
}

fn ordered_range(curve: Curve2D, first: f64, last: f64, forward: bool) -> PCurve {
    let (a, b) = if forward {
        (first, last)
    } else {
        (last, first)
    };
    PCurve::new(curve, a, b)
}

fn circular_trace(
    plane: &Plane,
    source: &Circle2,
    first: f64,
    last: f64,
    forward: bool,
    tolerance: f64,
) -> Result<PCurve, IoError> {
    let frame = chart(plane)?;
    let center = plane.origin + plane.x * source.center.x() + plane.y * source.center.y();
    let cosine = (plane.x * source.x.x() + plane.y * source.x.y()) * source.radius;
    let sine = (plane.x * source.y.x() + plane.y * source.y.y()) * source.radius;
    let delta = center - frame.origin;
    let uv = Point2::new(delta.dot(frame.x), delta.dot(frame.y));
    let phase = cosine.dot(frame.y).atan2(cosine.dot(frame.x));
    let sign = if cosine.cross(sine).dot(plane.normal) > 0.0 {
        1.
    } else {
        -1.
    };
    let (s, c) = phase.sin_cos();
    let mapped_center = frame.origin + frame.x * uv.x() + frame.y * uv.y();
    let mapped_cosine = (frame.x * c + frame.y * s) * source.radius;
    let mapped_sine = (frame.x * (-s) + frame.y * c) * (source.radius * sign);
    let bound = (center - mapped_center).length()
        + (cosine - mapped_cosine).length()
        + (sine - mapped_sine).length();
    if !bound.is_finite() {
        return Err(malformed("non-finite circular pcurve"));
    }
    if bound > tolerance {
        return Err(unsupported("circular pcurve chart precision"));
    }
    let curve = Curve2D::Circle(
        Circle2D::new(uv, source.radius).map_err(|_| malformed("invalid circular pcurve"))?,
    );
    Ok(ordered_range(
        curve,
        phase + sign * first,
        phase + sign * last,
        forward,
    ))
}

fn verify(
    topo: &Topology,
    edge_id: EdgeId,
    face_id: FaceId,
    forward: bool,
    tolerance: f64,
) -> Result<(), IoError> {
    let edge = topo.edge(edge_id)?;
    let face = topo.face(face_id)?;
    let pcurve = topo
        .pcurve_oriented(edge_id, face_id, forward)
        .ok_or_else(|| malformed("missing curved pcurve"))?;
    let (a, b) = edge
        .strict_domain()
        .map_err(|_| malformed("missing curve domain"))?;
    let (a, b) = if forward { (a, b) } else { (b, a) };
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    // Witnesses diagnose failures only. Acceptance needs the whole-interval
    // certificate below; agreeing samples never authorize a curve use.
    for fraction in [0., 0.25, 0.5, 0.75, 1.] {
        let expected = edge
            .curve()
            .evaluate_with_endpoints(a + (b - a) * fraction, start, end);
        let uv = pcurve.evaluate(pcurve.t_start() + (pcurve.t_end() - pcurve.t_start()) * fraction);
        let actual = match face.surface() {
            FaceSurface::Plane { normal, d } => {
                let k = *d / normal.length_squared();
                let frame = Frame3::from_normal(
                    Point3::new(normal.x() * k, normal.y() * k, normal.z() * k),
                    *normal,
                )
                .map_err(|_| malformed("invalid plane chart"))?;
                frame.origin + frame.x * uv.x() + frame.y * uv.y()
            }
            surface @ FaceSurface::Cylinder(_) => surface
                .evaluate(uv.x(), uv.y())
                .ok_or_else(|| malformed("unsupported pcurve evaluation"))?,
            FaceSurface::Nurbs(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => return Err(unsupported("native curved surface family")),
        };
        let residual = (expected - actual).length();
        if !residual.is_finite() || residual > tolerance {
            return Err(malformed("pcurve disagrees with its 3D edge"));
        }
    }
    let report = check_same_parameter_strict(topo, edge_id, face_id, forward, 5)
        .map_err(|_| unsupported("whole-interval curve-use proof"))?
        .ok_or_else(|| malformed("missing curve-use proof"))?;
    let range = check_same_range_strict(topo, edge_id, face_id, forward)
        .map_err(|_| unsupported("curve-use endpoint proof"))?
        .ok_or_else(|| malformed("missing curve-use range"))?;
    if report.max_deviation > tolerance || range > tolerance {
        return Err(unsupported("curve-use arithmetic exceeds source tolerance"));
    }
    Ok(())
}

impl Document {
    fn cylindrical_band(
        &self,
        wire: Usage,
        surface: usize,
        cylinder: &Cylinder,
        edges: &[Option<EdgeId>],
        topo: &Topology,
    ) -> Result<Vec<BoundUse>, IoError> {
        let source = &self.shapes[wire.index];
        if !matches!(source.geometry, Geometry::Wire) {
            return Err(malformed("non-wire cylindrical boundary"));
        }
        if source.children.len() != 4 {
            return Err(unsupported("general cylindrical trim boundary"));
        }
        let mut choices = Vec::new();
        let mut circles = Vec::new();
        let mut seams = Vec::new();
        let mut tolerance = 0.0_f64;
        for usage in &source.children {
            let usage = Usage {
                forward: (usage.forward == wire.forward) ^ cylinder.flip(),
                ..*usage
            };
            let edge_id = edges[usage.index].ok_or_else(|| malformed("missing cylinder edge"))?;
            let edge = topo.edge(edge_id)?;
            let domain = edge
                .strict_domain()
                .map_err(|_| malformed("missing cylinder edge domain"))?;
            match edge.curve() {
                EdgeCurve::Circle(_) if edge.is_closed() && full_turn(domain.0, domain.1) => {
                    circles.push(edge_id);
                }
                EdgeCurve::Line if !edge.is_closed() => seams.push((edge_id, usage.forward)),
                EdgeCurve::Line
                | EdgeCurve::Circle(_)
                | EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => {
                    return Err(unsupported("cylindrical band edge family"));
                }
            }
            tolerance = tolerance.max(self.curve_tolerance(usage.index)?);
            let traces = self.traces(usage, surface)?;
            if traces.is_empty() || traces.len() > 2 {
                return Err(unsupported("cylindrical band pcurve count"));
            }
            for trace in &traces {
                let a = trace.evaluate(trace.t_start());
                let b = trace.evaluate(trace.t_end());
                if matches!(edge.curve(), EdgeCurve::Circle(_))
                    && !full_turn(a.x().min(b.x()), a.x().max(b.x()))
                {
                    return Err(unsupported("partial cylindrical chart"));
                }
            }
            choices.push(
                traces
                    .into_iter()
                    .map(|pcurve| BoundUse { usage, pcurve })
                    .collect(),
            );
        }
        if circles.len() != 2
            || circles[0] == circles[1]
            || seams.len() != 2
            || seams[0].0 != seams[1].0
            || seams[0].1 == seams[1].1
        {
            return Err(unsupported(
                "cylindrical band needs two rims and paired seam uses",
            ));
        }
        let mut band = Band {
            choices: &choices,
            edges,
            topo,
            radius: cylinder.radius,
            tolerance,
            solutions: vec![],
        };
        band.search(&mut Vec::new(), 0)?;
        if band.solutions.len() != 1 {
            return Err(malformed("open or ambiguous cylindrical UV boundary"));
        }
        band.solutions
            .pop()
            .ok_or_else(|| malformed("missing cylindrical boundary"))
    }

    fn curve_tolerance(&self, index: usize) -> Result<f64, IoError> {
        match self.shapes[index].geometry {
            Geometry::Edge { tolerance, .. } => Ok(tolerance),
            _ => Err(malformed("non-edge circular boundary")),
        }
    }

    #[allow(
        clippy::float_cmp,
        reason = "Only exact axis-aligned source iso-lines are qualified"
    )]
    fn traces(&self, usage: Usage, surface: usize) -> Result<Vec<PCurve>, IoError> {
        let Geometry::Edge {
            tolerance,
            curve,
            pcurves,
        } = &self.shapes[usage.index].geometry
        else {
            return Err(malformed("non-edge curved boundary"));
        };
        let mut result = Vec::new();
        for p in pcurves.iter().filter(|p| p.surface == surface) {
            if p.first >= p.last {
                return Err(malformed("invalid curved pcurve range"));
            }
            let trace = match (&self.surfaces[surface], &self.curves2[p.curve]) {
                (Surface::Plane(plane), Curve2::Circle(circle)) => {
                    circular_trace(plane, circle, p.first, p.last, usage.forward, *tolerance)?
                }
                (Surface::Cylinder(cylinder), Curve2::Line(line)) => {
                    let native = cylinder.native()?;
                    let sign = if cylinder.flip() { -1. } else { 1. };
                    let max_v = (line.origin.y() + line.y * p.first)
                        .abs()
                        .max((line.origin.y() + line.y * p.last).abs());
                    let frame_bound = cylinder.radius
                        * ((cylinder.frame.x - native.x_axis()).length()
                            + (cylinder.frame.y - native.y_axis()).length())
                        + max_v * (cylinder.frame.normal - native.axis() * sign).length();
                    if !frame_bound.is_finite() {
                        return Err(malformed("non-finite cylindrical pcurve frame"));
                    }
                    if frame_bound > *tolerance {
                        return Err(unsupported("cylindrical frame precision"));
                    }
                    let qualified = match self.curves3[curve.curve] {
                        Curve3::Line(_) => line.x == 0. && line.y.abs() == 1.,
                        Curve3::Circle(_) => line.y == 0. && line.x.abs() == 1.,
                    };
                    if !qualified {
                        return Err(unsupported("non-isoparametric cylinder trim"));
                    }
                    let native = Curve2D::Line(
                        Line2D::new(
                            Point2::new(line.origin.x(), sign * line.origin.y()),
                            Vec2::new(line.x, sign * line.y),
                        )
                        .map_err(|_| malformed("invalid cylinder pcurve"))?,
                    );
                    ordered_range(native, p.first, p.last, usage.forward)
                }
                _ => return Err(unsupported("curved pcurve/carrier combination")),
            };
            result.push(trace);
        }
        Ok(result)
    }

    fn circular_caps(
        &self,
        face_index: usize,
        surface: usize,
        plane: &Plane,
        edges: &[Option<EdgeId>],
        topo: &Topology,
        budget: &mut Boundaries,
    ) -> Result<(Vec<Vec<BoundUse>>, usize), IoError> {
        let shape = &self.shapes[face_index];
        let Geometry::Face { tolerance, .. } = shape.geometry else {
            return Err(malformed("non-face circular cap"));
        };
        let wire_uses = &shape.children;
        ensure_limit("BREP circular loops per face", wire_uses.len(), 128)?;
        let mut loops = Vec::new();
        let mut circles = Vec::new();
        for &wire in wire_uses {
            Self::claim(&mut budget.used, wire.index)?;
            let source = &self.shapes[wire.index];
            let [usage] = source.children.as_slice() else {
                return Err(unsupported("mixed or partial circular planar boundary"));
            };
            if !matches!(source.geometry, Geometry::Wire) {
                return Err(malformed("non-wire circular boundary"));
            }
            let usage = Usage {
                forward: usage.forward == wire.forward,
                ..*usage
            };
            let edge =
                topo.edge(edges[usage.index].ok_or_else(|| malformed("missing rim edge"))?)?;
            let EdgeCurve::Circle(circle) = edge.curve() else {
                return Err(unsupported("mixed polygonal/circular face loops"));
            };
            let domain = edge
                .strict_domain()
                .map_err(|_| malformed("missing circular domain"))?;
            if !edge.is_closed() || !full_turn(domain.0, domain.1) {
                return Err(unsupported("partial circular planar loop"));
            }
            let tolerance = tolerance.max(self.curve_tolerance(usage.index)?);
            let normal_bound = plane.normal.dot(circle.center() - plane.origin).abs()
                + circle.radius()
                    * (plane.normal.dot(circle.u_axis()).abs()
                        + plane.normal.dot(circle.v_axis()).abs());
            if !normal_bound.is_finite() || normal_bound > tolerance {
                return Err(malformed("circular rim is outside its plane"));
            }
            let mut traces = self.traces(usage, surface)?;
            if traces.len() > 1 {
                return Err(unsupported("multiple pcurves on circular cap"));
            }
            if traces.is_empty() {
                let frame = chart(plane)?;
                let delta = circle.center() - frame.origin;
                let phase = circle
                    .u_axis()
                    .dot(frame.y)
                    .atan2(circle.u_axis().dot(frame.x));
                let sign = if circle.u_axis().cross(circle.v_axis()).dot(plane.normal) > 0. {
                    1.
                } else {
                    -1.
                };
                traces.push(ordered_range(
                    Curve2D::Circle(
                        Circle2D::new(
                            Point2::new(delta.dot(frame.x), delta.dot(frame.y)),
                            circle.radius(),
                        )
                        .map_err(|_| malformed("invalid rim radius"))?,
                    ),
                    phase + sign * domain.0,
                    phase + sign * domain.1,
                    usage.forward,
                ));
            }
            circles.push((circle, usage.forward, tolerance));
            loops.push(vec![BoundUse {
                usage,
                pcurve: traces.remove(0),
            }]);
        }
        let outer = circles
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.0.radius().total_cmp(&b.0.radius()))
            .map(|(index, _)| index)
            .ok_or_else(|| malformed("circular cap has no boundary"))?;
        let winding = |i: usize| {
            (circles[i]
                .0
                .u_axis()
                .cross(circles[i].0.v_axis())
                .dot(plane.normal)
                > 0.)
                == circles[i].1
        };
        for i in 0..circles.len() {
            if i == outer {
                continue;
            }
            let distance = (circles[i].0.center() - circles[outer].0.center()).length();
            let margin = circles[i].2.max(circles[outer].2)
                + 64.
                    * f64::EPSILON
                    * (distance + circles[outer].0.radius() + circles[i].0.radius());
            if !distance.is_finite()
                || distance + circles[i].0.radius() + margin >= circles[outer].0.radius()
                || winding(i) == winding(outer)
            {
                return Err(malformed("uncontained or same-wound circular hole"));
            }
            for j in 0..i {
                if j == outer {
                    continue;
                }
                budget.work.charge(1)?;
                let distance = (circles[i].0.center() - circles[j].0.center()).length();
                let radius = circles[i].0.radius() + circles[j].0.radius();
                if !distance.is_finite()
                    || distance
                        <= radius
                            + circles[i].2.max(circles[j].2)
                            + 64. * f64::EPSILON * (distance + radius)
                {
                    return Err(malformed("intersecting or nested circular holes"));
                }
            }
        }
        Ok((loops, outer))
    }

    pub(super) fn curved_face(
        &self,
        topo: &mut Topology,
        usage: Usage,
        parent_reversed: bool,
        edges: &[Option<EdgeId>],
        budget: &mut Boundaries,
    ) -> Result<FaceId, IoError> {
        let shape = &self.shapes[usage.index];
        let Geometry::Face { surface, .. } = shape.geometry else {
            return Err(malformed("non-face curved boundary"));
        };
        let (loops, outer, native, flip) = match &self.surfaces[surface] {
            Surface::Plane(plane) => {
                let (loops, outer) =
                    self.circular_caps(usage.index, surface, plane, edges, topo, budget)?;
                let d = plane.normal.dot(plane.origin - Point3::new(0., 0., 0.));
                (
                    loops,
                    outer,
                    FaceSurface::Plane {
                        normal: plane.normal,
                        d,
                    },
                    false,
                )
            }
            Surface::Cylinder(cylinder) => {
                let [wire] = shape.children.as_slice() else {
                    return Err(unsupported("multiple cylindrical trim loops"));
                };
                Self::claim(&mut budget.used, wire.index)?;
                budget.work.charge(384)?;
                let boundary = self.cylindrical_band(*wire, surface, cylinder, edges, topo)?;
                (
                    vec![boundary],
                    0,
                    FaceSurface::Cylinder(cylinder.native()?),
                    cylinder.flip(),
                )
            }
        };
        let mut wires = Vec::new();
        for boundary in &loops {
            let edges = boundary
                .iter()
                .map(|u| {
                    Ok(OrientedEdge::new(
                        edges[u.usage.index].ok_or_else(|| malformed("missing curved edge"))?,
                        u.usage.forward,
                    ))
                })
                .collect::<Result<_, IoError>>()?;
            wires.push(topo.add_wire(Wire::new(edges, true)?));
        }
        let outer = wires.remove(outer);
        let mut face = Face::new(outer, wires, native);
        face.set_reversed(!usage.forward ^ parent_reversed ^ flip);
        let face_id = topo.add_face(face);
        for u in loops.into_iter().flatten() {
            let edge = edges[u.usage.index].ok_or_else(|| malformed("missing curved edge"))?;
            topo.set_pcurve_oriented(edge, face_id, u.usage.forward, u.pcurve)?;
            verify(
                topo,
                edge,
                face_id,
                u.usage.forward,
                self.curve_tolerance(u.usage.index)?,
            )?;
        }
        // Use the exchange writer's branch certificate rather than guessing a
        // lift from a rounded seam coordinate. Both branches are authoritative.
        for loop_id in topo.face(face_id)?.boundary_loops().to_vec() {
            for coedge_id in topo.face_loop(loop_id)?.coedges().to_vec() {
                let coedge = topo.coedge(coedge_id)?;
                let edge = topo.edge(coedge.edge())?;
                let (a, b) = if coedge.is_forward() {
                    (edge.start(), edge.end())
                } else {
                    (edge.end(), edge.start())
                };
                let pcurve = coedge
                    .pcurve()
                    .ok_or_else(|| malformed("missing curved trace"))?;
                let winding = crate::step::writer::export_pcurve_winding(
                    topo.face(face_id)?.surface(),
                    pcurve,
                    topo.vertex(a)?.point(),
                    topo.vertex(b)?.point(),
                )
                .map_err(|_| unsupported("periodic branch precision"))?;
                topo.set_coedge_periodic_winding(coedge_id, winding)?;
            }
        }
        Ok(face_id)
    }
}
