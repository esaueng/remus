//! Recover the parameters of a candidate single-start cylindrical thread.
//! The independent wall comparison checks this inference before a solid edit.
use super::{Bore, Frame, ThreadForm, ThreadHand, mouths};
use crate::thread::validate_form;
use remus_math::{surfaces::CylindricalSurface, tolerance::Tolerance, vec::Point3};
use remus_topology::{
    Topology,
    adjacency::AdjacencyIndex,
    edge::EdgeCurve,
    explorer::face_edges,
    face::{FaceId, FaceSurface},
};
use std::{collections::BTreeSet, f64::consts::TAU};
#[derive(Clone, Copy)]
struct Arc {
    start: f64,
    span: f64,
}

impl Bore {
    pub(super) fn recognize(
        topo: &Topology,
        adj: &AdjacencyIndex,
        first: super::Mouth,
    ) -> Option<Self> {
        let cap = first.cap;
        let mut wall = BTreeSet::new();
        let mut stack = vec![first.cone];
        let mut other = None;
        let mut cylinders = vec![];
        while let Some(face) = stack.pop() {
            if !wall.insert(face) {
                continue;
            }
            match topo.face(face).ok()?.surface() {
                FaceSurface::Cylinder(c) => cylinders.push((face, c)),
                FaceSurface::Nurbs(_) | FaceSurface::Cone(_) => {}
                FaceSurface::Plane { .. } | FaceSurface::Sphere(_) | FaceSurface::Torus(_) => {
                    return None;
                }
            }
            for &neighbor in adj.neighbors_of_face(face) {
                if neighbor == cap || wall.contains(&neighbor) {
                    continue;
                }
                if topo.face(neighbor).ok()?.effective_plane_normal().is_some() {
                    if other.replace(neighbor).is_some_and(|old| old != neighbor) {
                        return None;
                    }
                } else {
                    stack.push(neighbor);
                }
            }
        }
        let [last]: [super::Mouth; 1] = mouths(topo, adj, other?)?
            .into_iter()
            .filter(|m| wall.contains(&m.cone))
            .collect::<Vec<_>>()
            .try_into()
            .ok()?;

        let axis = cylinders.first()?.1.axis();
        let ends = if (last.center - first.center).dot(axis) > 0.0 {
            [first, last]
        } else {
            [last, first]
        };
        if topo
            .face(ends[0].cap)
            .ok()?
            .effective_plane_normal()?
            .dot(axis)
            > -1.0 + Tolerance::new().angular
            || topo
                .face(ends[1].cap)
                .ok()?
                .effective_plane_normal()?
                .dot(axis)
                < 1.0 - Tolerance::new().angular
        {
            return None;
        }
        let separation = ends[1].center - ends[0].center;
        if separation.cross(axis).length() > Tolerance::new().linear {
            return None;
        }
        let length = separation.dot(axis);
        let origin = ends[0].center;
        let frame = Frame::new(origin, axis, cylinders.first()?.1.x_axis()).ok()?;
        let (frame, crest, root, form) = recover_form(topo, &cylinders, &wall, frame)?;
        Some(Self {
            frame,
            ends,
            wall: wall.into_iter().collect(),
            form,
            crest,
            root,
            length,
        })
    }
}
/// On a land, the axial helix tangent determines the signed twist rate.
/// The two circular arcs on one transverse section then determine all four
/// widths of the trapezoid and the crest's phase at the start plane.
fn recover_form(
    topo: &Topology,
    cylinders: &[(FaceId, &CylindricalSurface)],
    wall: &BTreeSet<FaceId>,
    frame: Frame,
) -> Option<(Frame, f64, f64, ThreadForm)> {
    let origin = frame.origin;
    let axis = frame.axis;
    let &(land, _) = cylinders
        .iter()
        .min_by(|a, b| a.1.radius().total_cmp(&b.1.radius()))?;
    let theta = |point: Point3| {
        let v = point - origin;
        v.dot(frame.y).atan2(v.dot(frame.x))
    };
    let z = |p: Point3| (p - origin).dot(axis);
    let edges = face_edges(topo, land).ok()?;
    let mut rate = None;
    let mut level = None;
    for e in edges {
        let edge = topo.edge(e).ok()?;
        let start = topo.vertex(edge.start()).ok()?.point();
        match edge.curve() {
            EdgeCurve::NurbsCurve(curve) => {
                let dx = *curve.derivatives(curve.domain().0, 1).get(1)?;
                let radial = start - origin - axis * z(start);
                let k = axis.dot(radial.cross(dx)) / (radial.length_squared() * axis.dot(dx));
                if !k.is_finite() || k.abs() < f64::EPSILON {
                    return None;
                }
                rate = Some(k);
            }
            EdgeCurve::Circle(_) => level = Some(z(start)),
            EdgeCurve::Line
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => return None,
        }
    }
    let rate = rate?;
    let level = level?;
    // A land spanning a full turn is stored as a rational spline surface.
    // Its transverse boundary remains circular in either representation.
    let mut arcs = Vec::new();
    let mut seen = BTreeSet::new();
    for &face in wall {
        for e in face_edges(topo, face).ok()? {
            if !seen.insert(e) {
                continue;
            }
            let edge = topo.edge(e).ok()?;
            let EdgeCurve::Circle(circle) = edge.curve() else {
                continue;
            };
            if edge.is_closed() || (z(circle.center()) - level).abs() > Tolerance::new().linear {
                continue;
            }
            let (a, b) = edge.strict_domain().ok()?;
            let sign = -rate.signum();
            let start = if (b - a) * axis.dot(circle.normal()) * sign > 0.0 {
                a
            } else {
                b
            };
            let arc = Arc {
                start: sign * theta(circle.evaluate(start)),
                span: (b - a).abs(),
            };
            arcs.push((circle.radius(), arc));
        }
    }
    let [a, b]: [_; 2] = arcs.try_into().ok()?;
    let [(crest, crest_arc), (root, root_arc)] = if a.0 < b.0 { [a, b] } else { [b, a] };
    let depth = root - crest;
    if depth <= Tolerance::new().linear {
        return None;
    }
    let gap = |delta: f64| {
        let arc = delta.rem_euclid(TAU);
        if TAU - arc <= Tolerance::new().angular || arc <= Tolerance::new().angular {
            0.0
        } else {
            arc / rate.abs()
        }
    };
    let upper = gap(root_arc.start - crest_arc.start - crest_arc.span);
    let lower = gap(crest_arc.start - root_arc.start - root_arc.span);
    let form = ThreadForm {
        pitch: TAU / rate.abs(),
        starts: 1,
        hand: if rate > 0.0 {
            ThreadHand::Right
        } else {
            ThreadHand::Left
        },
        depth,
        crest_width: crest_arc.span / rate.abs(),
        upper_flank_angle: upper.atan2(depth),
        lower_flank_angle: lower.atan2(depth),
    };
    validate_form(&form).ok()?;
    let phase = (-rate.signum() * crest_arc.start - rate * level).rem_euclid(TAU);
    let frame = Frame::new(origin, axis, frame.x * phase.cos() + frame.y * phase.sin()).ok()?;
    Some((frame, crest, root, form))
}
