//! Quotient of an explicitly cut singly periodic cylinder strip.
use super::{
    Arrangement, ArrangementError, CurveUse, DomainIdentification, ParamDomain, PeriodicRegion,
    Result, Work, geometry,
};
use remus_math::curves2d::Curve2D;
use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;

#[allow(clippy::too_many_lines)]
pub(super) fn quotient(
    a: &mut Arrangement,
    uses: &BTreeMap<u64, &CurveUse>,
    domain: ParamDomain,
    work: &mut Work<'_>,
) -> Result<()> {
    let ParamDomain::CylinderStrip { seam_uses, radius } = domain else {
        return Ok(());
    };
    if !radius.is_finite() || radius <= 0.0 || seam_uses[0] == seam_uses[1] {
        return Err(ArrangementError::UnsupportedDomain);
    }
    let seams = seam_uses.map(|id| uses.get(&id).copied());
    let [Some(left), Some(right)] = seams else {
        return Err(ArrangementError::UnsupportedDomain);
    };
    let (Curve2D::Line(l), Curve2D::Line(r)) = (&left.pcurve, &right.pcurve) else {
        return Err(ArrangementError::UnsupportedDomain);
    };
    if !geometry::same(l.direction().x(), 0.0)
        || !geometry::same(r.direction().x(), 0.0)
        || left.boundary_loop.is_none()
        || right.boundary_loop != left.boundary_loop
        || ((r.origin().x() - l.origin().x()) - TAU).abs() > geometry::roundoff(r.origin())
    {
        return Err(ArrangementError::UnsupportedDomain);
    }
    // This slice accepts exact cylinder rulings and latitude circles in the
    // strip. Oblique UV lines would require a different exact 3D carrier.
    for u in uses.values() {
        work.step()?;
        let Curve2D::Line(line) = &u.pcurve else {
            return Err(ArrangementError::UnsupportedDomain);
        };
        if !geometry::same(line.direction().x(), 0.0) && !geometry::same(line.direction().y(), 0.0)
        {
            return Err(ArrangementError::UnsupportedDomain);
        }
        for t in u.range {
            let p = u.point(t);
            if p.x() < l.origin().x() || p.x() > r.origin().x() {
                return Err(ArrangementError::UnsupportedDomain);
            }
        }
    }
    let mut seam_edges: [Vec<usize>; 2] = [Vec::new(), Vec::new()];
    for (h, e) in a.half_edges.iter().enumerate().step_by(2) {
        work.step()?;
        for side in 0..2 {
            if e.source.use_id == seam_uses[side] {
                seam_edges[side].push(h);
            }
        }
    }
    let lower = |h: usize| {
        let e = &a.half_edges[h];
        a.vertices[e.from].uv.y().min(a.vertices[e.to].uv.y())
    };
    for edges in &mut seam_edges {
        edges.sort_by(|&x, &y| lower(x).total_cmp(&lower(y)));
    }
    if seam_edges[0].len() != seam_edges[1].len() {
        return Err(ArrangementError::UnsupportedDomain);
    }
    let mut counterpart = BTreeMap::new();
    let mut vertex_alias: Vec<_> = (0..a.vertices.len()).collect();
    let mut face_of = vec![None; a.half_edges.len()];
    for (id, region) in a.regions.iter().enumerate() {
        for cycle in std::iter::once(&region.outer).chain(&region.holes) {
            for &h in &a.cycles[*cycle].edges {
                work.step()?;
                face_of[h] = Some(id);
            }
        }
    }
    let mut adjacency = vec![Vec::new(); a.regions.len()];
    for (&x, &y) in seam_edges[0].iter().zip(&seam_edges[1]) {
        work.step()?;
        let ex = &a.half_edges[x];
        let ey = &a.half_edges[y];
        let mut vx = [ex.from, ex.to];
        let mut vy = [ey.from, ey.to];
        vx.sort_by(|&x, &y| a.vertices[x].uv.y().total_cmp(&a.vertices[y].uv.y()));
        vy.sort_by(|&x, &y| a.vertices[x].uv.y().total_cmp(&a.vertices[y].uv.y()));
        for (x, y) in vx.into_iter().zip(vy) {
            work.step()?;
            // Declared seam equivalence is the certificate; the v coordinate
            // must agree exactly. Near unmatched breaks are refused, not welded.
            if !geometry::same(a.vertices[x].uv.y(), a.vertices[y].uv.y())
                || (a.vertices[x].point_3d - a.vertices[y].point_3d).length()
                    > work.context.tolerance.linear
            {
                return Err(ArrangementError::UnsupportedDomain);
            }
            vertex_alias[y] = x;
            if !a.identifications.iter().any(|p| p.vertices == [x, y]) {
                a.identifications.push(DomainIdentification {
                    vertices: [x, y],
                    u_lift: 1,
                });
            }
        }
        let hx = if face_of[x].is_some() { x } else { x + 1 };
        let hy = if face_of[y].is_some() { y } else { y + 1 };
        let (Some(fx), Some(fy)) = (face_of[hx], face_of[hy]) else {
            return Err(ArrangementError::UnsupportedDomain);
        };
        if a.regions[fx].material != a.regions[fy].material {
            return Err(ArrangementError::UnsupportedDomain);
        }
        counterpart.insert(hx, hy);
        counterpart.insert(hy, hx);
        adjacency[fx].push(fy);
        adjacency[fy].push(fx);
    }
    a.identifications.sort_by_key(|p| p.vertices);
    let mut seen = vec![false; a.regions.len()];
    for region in 0..a.regions.len() {
        work.step()?;
        if seen[region] || !a.regions[region].material {
            continue;
        }
        let mut cells = Vec::new();
        let mut pending = vec![region];
        seen[region] = true;
        while let Some(f) = pending.pop() {
            work.step()?;
            cells.push(f);
            for &n in &adjacency[f] {
                work.step()?;
                if !seen[n] {
                    seen[n] = true;
                    pending.push(n);
                }
            }
        }
        cells.sort_unstable();
        let mut boundary = BTreeSet::new();
        let mut vertices = BTreeSet::new();
        let mut edges = BTreeSet::new();
        let mut chi_cells = 0isize;
        for &f in &cells {
            work.step()?;
            chi_cells += 1 - isize::try_from(a.regions[f].holes.len())
                .map_err(|_| ArrangementError::WorkBudgetExceeded)?;
            for c in std::iter::once(&a.regions[f].outer).chain(&a.regions[f].holes) {
                for &h in &a.cycles[*c].edges {
                    work.step()?;
                    let e = &a.half_edges[h];
                    vertices.insert(vertex_alias[e.from]);
                    let edge = counterpart
                        .get(&h)
                        .map_or(h / 2, |other| (h / 2).min(other / 2));
                    edges.insert(edge);
                    if !counterpart.contains_key(&h) {
                        boundary.insert(h);
                    }
                }
            }
        }
        let mut boundaries = Vec::new();
        let mut used = BTreeSet::new();
        for &start in &boundary {
            work.step()?;
            if used.contains(&start) {
                continue;
            }
            let mut h = start;
            let mut cycle = Vec::new();
            let mut delta = 0.0;
            loop {
                work.step()?;
                if !used.insert(h) {
                    if h != start {
                        return Err(ArrangementError::OpenRegion);
                    }
                    break;
                }
                cycle.push(h);
                let e = &a.half_edges[h];
                delta += a.vertices[e.to].uv.x() - a.vertices[e.from].uv.x();
                let mut next = e.next;
                while let Some(&other) = counterpart.get(&next) {
                    work.step()?;
                    next = a.half_edges[other].next;
                }
                if !boundary.contains(&next)
                    || vertex_alias[e.to] != vertex_alias[a.half_edges[next].from]
                {
                    return Err(ArrangementError::OpenRegion);
                }
                h = next;
            }
            let turns = (delta / TAU).round();
            if (delta - turns * TAU).abs() > 64.0 * f64::EPSILON * (1.0 + delta.abs())
                || turns.abs() > 1.0
            {
                return Err(ArrangementError::UnsupportedDomain);
            }
            let winding = if turns > 0.0 {
                1
            } else if turns < 0.0 {
                -1
            } else {
                0
            };
            boundaries.push((cycle, winding));
        }
        let chi = isize::try_from(vertices.len())
            .map_err(|_| ArrangementError::WorkBudgetExceeded)?
            - isize::try_from(edges.len()).map_err(|_| ArrangementError::WorkBudgetExceeded)?
            + chi_cells;
        // Connected orientable subsurfaces of a cylinder have genus zero.
        let expected = 2 - isize::try_from(boundaries.len())
            .map_err(|_| ArrangementError::WorkBudgetExceeded)?;
        if chi != expected || boundaries.iter().map(|b| b.1).sum::<i32>() != 0 {
            return Err(ArrangementError::NonManifoldEmbedding);
        }
        let area = cells.iter().map(|&f| a.regions[f].area * radius).sum();
        a.periodic_regions.push(PeriodicRegion {
            cells,
            boundaries,
            area,
            euler_characteristic: chi,
        });
    }
    Ok(())
}
