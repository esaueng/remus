//! Face orbits, analytic nesting and material parity in the lifted chart.
use super::{
    Arrangement, ArrangementError, ArrangementRegion, CurveUse, Cycle, Result, Work, geometry,
};
use remus_math::vec::{Point2, Vec2};
use std::collections::BTreeMap;

pub(super) fn winding(
    a: &Arrangement,
    cycle: usize,
    p: Point2,
    uses: &BTreeMap<u64, &CurveUse>,
    work: &mut Work<'_>,
) -> Result<i32> {
    let mut winding = 0;
    for &h in &a.cycles[cycle].edges {
        work.step()?;
        let e = &a.half_edges[h];
        winding += geometry::ray_crossing(uses[&e.source.use_id], e.range, p);
    }
    Ok(winding)
}

fn left_seed(
    a: &Arrangement,
    edge: usize,
    uses: &BTreeMap<u64, &CurveUse>,
    work: &mut Work<'_>,
) -> Result<Point2> {
    let h = &a.half_edges[edge];
    let u = uses[&h.source.use_id];
    let t = h.range[0] + (h.range[1] - h.range[0]) * 0.5;
    let p = u.point(t);
    let tangent = u.pcurve.tangent(t) * (h.range[1] - h.range[0]).signum();
    let normal = Vec2::new(-tangent.y(), tangent.x()) * (1.0 / tangent.length());
    let mut clearance = (u.point(h.range[0]) - p)
        .length()
        .min((u.point(h.range[1]) - p).length());
    for (i, other) in a.half_edges.iter().enumerate().step_by(2) {
        work.step()?;
        if i / 2 != edge / 2 {
            clearance = clearance.min(geometry::distance(
                uses[&other.source.use_id],
                other.range,
                p,
            ));
        }
    }
    let mut offset = clearance * 0.125;
    // The clearance ball excludes every other exact subcurve. Shrink if the
    // same curved support folds into it or floating-point movement is lost.
    loop {
        work.step()?;
        if !offset.is_finite() || offset <= geometry::roundoff(p) {
            return Err(ArrangementError::IntersectionRefinementFailed);
        }
        let seed = p + normal * offset;
        let distance = geometry::distance(u, h.range, seed);
        if distance >= offset * 0.5 {
            return Ok(seed);
        }
        offset *= 0.5;
    }
}

#[allow(clippy::too_many_lines)]
pub(super) fn extract(
    a: &mut Arrangement,
    uses: &BTreeMap<u64, &CurveUse>,
    work: &mut Work<'_>,
) -> Result<()> {
    let mut visited = vec![false; a.half_edges.len()];
    for start in 0..a.half_edges.len() {
        work.step()?;
        if visited[start] {
            continue;
        }
        let mut edges = Vec::new();
        let mut h = start;
        loop {
            work.step()?;
            if visited[h] {
                if h != start {
                    return Err(ArrangementError::OpenRegion);
                }
                break;
            }
            visited[h] = true;
            edges.push(h);
            let next = a.half_edges[h].next;
            if a.half_edges[h].to != a.half_edges[next].from {
                return Err(ArrangementError::OpenRegion);
            }
            h = next;
        }
        let origin = a.vertices[a.half_edges[start].from].uv;
        let mut area = 0.0;
        let mut correction = 0.0;
        for &h in &edges {
            work.step()?;
            let e = &a.half_edges[h];
            let term = geometry::integral(uses[&e.source.use_id], e.range, origin) - correction;
            let sum = area + term;
            correction = (sum - area) - term;
            area = sum;
        }
        if !area.is_finite() {
            return Err(ArrangementError::NonFiniteInput);
        }
        if geometry::same(area, 0.0) {
            return Err(ArrangementError::OpenRegion);
        }
        // A bridge repeats an undirected edge in the same orbit. It is a slit,
        // not a qualified closed region in this first slice.
        let mut undirected: Vec<_> = edges.iter().map(|h| h / 2).collect();
        undirected.sort_unstable();
        if undirected.windows(2).any(|p| p[0] == p[1]) {
            return Err(ArrangementError::OpenRegion);
        }
        let seed = left_seed(a, start, uses, work)?;
        a.cycles.push(Cycle {
            edges,
            signed_area: area,
            interior_left: seed,
        });
    }
    for i in 0..a.cycles.len() {
        work.step()?;
        if a.cycles[i].signed_area > 0.0 {
            if winding(a, i, a.cycles[i].interior_left, uses, work)? != 1 {
                return Err(ArrangementError::NonManifoldEmbedding);
            }
            a.regions.push(ArrangementRegion {
                outer: i,
                holes: Vec::new(),
                interior: a.cycles[i].interior_left,
                area: a.cycles[i].signed_area,
                material: false,
            });
        }
    }
    for i in 0..a.cycles.len() {
        work.step()?;
        if a.cycles[i].signed_area > 0.0 {
            continue;
        }
        let mut owner = None;
        let mut owner_area = f64::INFINITY;
        for (r, region) in a.regions.iter().enumerate() {
            work.step()?;
            if a.cycles[region.outer].signed_area < owner_area
                && winding(a, region.outer, a.cycles[i].interior_left, uses, work)? != 0
            {
                owner = Some(r);
                owner_area = a.cycles[region.outer].signed_area;
            }
        }
        if let Some(r) = owner {
            a.regions[r].holes.push(i);
            a.regions[r].area += a.cycles[i].signed_area;
        } else {
            a.exterior.push(i);
        }
    }
    for region in &mut a.regions {
        work.step()?;
        if region.area <= 0.0 {
            return Err(ArrangementError::NonManifoldEmbedding);
        }
        let mut crossings = 0;
        for e in a.half_edges.iter().step_by(2) {
            work.step()?;
            let u = uses[&e.source.use_id];
            if u.boundary_loop.is_some() {
                crossings += geometry::ray_crossing(u, e.range, region.interior);
            }
        }
        region.material = crossings.rem_euclid(2) != 0;
    }
    let mut seen = vec![false; a.vertices.len()];
    let mut components = 0;
    let mut neighbors = vec![Vec::new(); a.vertices.len()];
    for e in &a.half_edges {
        work.step()?;
        neighbors[e.from].push(e.to);
    }
    for v in 0..a.vertices.len() {
        work.step()?;
        if seen[v] {
            continue;
        }
        components += 1;
        seen[v] = true;
        let mut stack = vec![v];
        while let Some(v) = stack.pop() {
            work.step()?;
            for &n in &neighbors[v] {
                work.step()?;
                if !seen[n] {
                    seen[n] = true;
                    stack.push(n);
                }
            }
        }
    }
    // F includes the one explicit exterior, including all disconnected rims.
    if a.vertices.len() + a.regions.len() != a.half_edges.len() / 2 + components {
        return Err(ArrangementError::NonManifoldEmbedding);
    }
    Ok(())
}
