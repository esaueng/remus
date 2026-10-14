//! Equality-geometric coalescing of over-budget exact equator partitions.
//! Other wires retain the existing per-edge sampler and its work accounting.

use super::{
    CheckError, Circle3D, EdgeCurve, Face, I, PartitionSamples, SphericalSurface, TAU, Topology,
    WireId, dot, native_point, norm1, periodic_continuity, sub, vector,
};
use remus_topology::vertex::VertexId;

struct Arc {
    circle: Circle3D,
    from: f64,
    to: f64,
    start: VertexId,
    end: VertexId,
    equator: bool,
    pieces: usize,
}
enum Plan {
    Arcs(Vec<Arc>),
    Legacy(WireId, usize),
}

fn same_carrier(a: &Circle3D, b: &Circle3D) -> bool {
    a.center() == b.center()
        && a.radius() == b.radius()
        && a.u_axis() == b.u_axis()
        && a.v_axis() == b.v_axis()
}
fn endpoint(
    topo: &Topology,
    circle: &Circle3D,
    t: f64,
    id: VertexId,
    budget: f64,
) -> Result<bool, CheckError> {
    let Some(point) = native_point(circle, t) else {
        return Ok(false);
    };
    let error = norm1(sub(point, topo.vertex(id)?.point().0.map(I::point)));
    Ok(error.finite() && error.hi <= budget && error.div(circle.radius()).hi <= 1e-10)
}
fn arc_plan(
    topo: &Topology,
    wire_id: WireId,
    sphere: &SphericalSurface,
    total_edges: usize,
) -> Result<Option<Vec<Arc>>, CheckError> {
    let wire = topo.wire(wire_id)?;
    if !wire.is_closed() || wire.edges().is_empty() {
        return Ok(None);
    }
    let budget = remus_math::tolerance::Tolerance::new().linear / 64.0 / total_edges as f64;
    let mut arcs = Vec::new();
    for (index, use_) in wire.edges().iter().enumerate() {
        let edge = topo.edge(use_.edge())?;
        let next = topo.edge(wire.edges()[(index + 1) % wire.edges().len()].edge())?;
        let EdgeCurve::Circle(circle) = edge.curve() else {
            return Ok(None);
        };
        if circle.center() != sphere.center()
            || circle.radius() != sphere.radius()
            || !circle.radius().is_finite()
            || circle.radius() <= 0.0
            || use_.oriented_end(edge)
                != wire.edges()[(index + 1) % wire.edges().len()].oriented_start(next)
        {
            return Ok(None);
        }
        let u = vector(circle.u_axis());
        let v = vector(circle.v_axis());
        let z = vector(sphere.z_axis());
        let gram = dot(u, u)
            .sub(I::point(1.0))
            .abs()
            .add(dot(v, v).sub(I::point(1.0)).abs())
            .add(dot(u, v).abs().mul(I::point(2.0)));
        if !gram.finite() || gram.hi > 1e-12 || gram.mul(I::point(circle.radius())).hi > budget {
            return Ok(None);
        }
        let Ok((a, b)) = edge.strict_domain() else {
            return Ok(None);
        };
        let (from, to) = if use_.is_forward() { (a, b) } else { (b, a) };
        let span = I::point(to).sub(I::point(from));
        if !span.finite()
            || span.abs().hi > TAU.next_up().next_up()
            || !(span.lo > 0.0 || span.hi < 0.0)
        {
            return Ok(None);
        }
        let (start, end) = (use_.oriented_start(edge), use_.oriented_end(edge));
        if !endpoint(topo, circle, from, start, budget)?
            || !endpoint(topo, circle, to, end, budget)?
        {
            return Ok(None);
        }
        let equator_error = dot(u, z)
            .abs()
            .add(dot(v, z).abs())
            .mul(I::point(circle.radius()));
        let equator = equator_error.finite() && equator_error.hi <= budget;
        arcs.push(Arc {
            circle: circle.clone(),
            from,
            to,
            start,
            end,
            equator,
            pieces: 1,
        });
    }
    // Start at a carrier change, so a cyclic equator run is represented once.
    let break_at = (0..arcs.len())
        .find(|&i| {
            let a = &arcs[(i + arcs.len() - 1) % arcs.len()];
            let b = &arcs[i];
            !(a.equator && b.equator && same_carrier(&a.circle, &b.circle))
        })
        .unwrap_or(0);
    arcs.rotate_left(break_at);
    let mut groups: Vec<Arc> = Vec::new();
    for arc in arcs {
        if let Some(last) = groups.last_mut()
            && last.equator
            && arc.equator
            && same_carrier(&last.circle, &arc.circle)
        {
            if last.end != arc.start
                || (last.to - last.from).signum() != (arc.to - arc.from).signum()
                || !periodic_continuity(last.to, arc.from, arc.circle.radius(), budget)
            {
                return Ok(None);
            }
            let end = last.to + (arc.to - arc.from);
            let span = I::point(end).sub(I::point(last.from));
            if !span.finite()
                || span.abs().hi > TAU.next_up().next_up()
                || !periodic_continuity(end, arc.to, arc.circle.radius(), budget)
                || !endpoint(topo, &arc.circle, end, arc.end, budget)?
            {
                return Ok(None);
            }
            last.to = end;
            last.end = arc.end;
            last.pieces += arc.pieces;
        } else {
            groups.push(arc);
        }
    }
    if groups.len() == 1 && groups[0].equator && super::ring(topo, wire_id, sphere)?.is_none() {
        return Ok(None);
    }
    Ok(Some(groups))
}

pub(super) fn build(
    topo: &Topology,
    face: &Face,
    sphere: &SphericalSurface,
) -> Result<Option<PartitionSamples>, CheckError> {
    let z = vector(sphere.z_axis());
    let axis_error = dot(z, z).sub(I::point(1.0)).abs();
    if !axis_error.finite() || axis_error.hi > 1e-12 {
        return Ok(None);
    }
    let wires: Vec<_> = std::iter::once(face.outer_wire())
        .chain(face.inner_wires().iter().copied())
        .collect();
    let mut total_edges = 0usize;
    for &wire in &wires {
        total_edges = total_edges.saturating_add(topo.wire(wire)?.edges().len());
    }
    if total_edges == 0 || total_edges > super::super::MAX_TRIM_POINTS {
        return Ok(None);
    }
    let mut plans = Vec::new();
    let mut count = 0usize;
    let mut merged = false;
    for &wire in &wires {
        let plan = if let Some(arcs) = arc_plan(topo, wire, sphere, total_edges)? {
            merged |= arcs.iter().any(|a| a.pieces > 1);
            count = count.saturating_add(arcs.len().saturating_mul(super::super::TRIM_SAMPLES));
            Plan::Arcs(arcs)
        } else {
            let mut points = 0usize;
            for use_ in topo.wire(wire)?.edges() {
                points = points.saturating_add(
                    if matches!(topo.edge(use_.edge())?.curve(), EdgeCurve::Line) {
                        1
                    } else {
                        super::super::TRIM_SAMPLES
                    },
                );
            }
            count = count.saturating_add(points);
            Plan::Legacy(wire, points)
        };
        if count > super::super::MAX_TRIM_POINTS {
            return Ok(None);
        }
        plans.push(plan);
    }
    if !merged {
        return Ok(None);
    }
    let mut polygons = Vec::new();
    let mut actual_count = 0usize;
    for plan in plans {
        let points = match plan {
            Plan::Legacy(wire, expected) => {
                let points = crate::util::wire_polygon_curve_sampled(
                    topo,
                    wire,
                    super::super::TRIM_SAMPLES,
                    super::super::TRIM_SAMPLES,
                )?;
                if points.len() > expected {
                    return Ok(None);
                }
                points
            }
            Plan::Arcs(arcs) => {
                let mut points = Vec::with_capacity(arcs.len() * super::super::TRIM_SAMPLES);
                for arc in arcs {
                    for i in 0..super::super::TRIM_SAMPLES {
                        let t = (arc.to - arc.from)
                            .mul_add(i as f64 / super::super::TRIM_SAMPLES as f64, arc.from);
                        let point = arc.circle.evaluate(t);
                        if point.0.iter().any(|value| !value.is_finite()) {
                            return Ok(None);
                        }
                        points.push(point);
                    }
                }
                points
            }
        };
        actual_count = actual_count.saturating_add(points.len());
        if actual_count > super::super::MAX_TRIM_POINTS {
            return Ok(None);
        }
        polygons.push(points);
    }
    let outer = polygons.remove(0);
    Ok(Some(PartitionSamples {
        outer,
        holes: polygons,
    }))
}
