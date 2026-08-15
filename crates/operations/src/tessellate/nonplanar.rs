//! Non-planar CDT and fallback paths for face tessellation.

use brepkit_math::det_hash::{DetHashMap, DetHashSet};
use brepkit_math::vec::{Point3, Vec3};
use brepkit_topology::Topology;
use brepkit_topology::edge::EdgeCurve;
use brepkit_topology::face::{FaceId, FaceSurface};

use std::f64::consts::TAU;

use super::edge_sampling::{sample_edge, segments_for_chord_deviation_a};
use super::rim_chain::collect_full_turn_rim_cycles;
use super::{MERGE_GRID, TriangleMesh, point_merge_key};

/// Maps a 3D point to its `(u, v)` surface parameters.
type ProjectFn = Box<dyn Fn(Point3) -> (f64, f64)>;
/// Maps `(u, v)` surface parameters to a 3D surface point.
type EvalFn = Box<dyn Fn(f64, f64) -> Point3>;
/// Maps `(u, v)` surface parameters to the outward surface normal.
type NormalFn = Box<dyn Fn(f64, f64) -> Vec3>;

fn rim_angles_match(a: &[f64], b: &[f64], tolerance: f64) -> bool {
    a.len() == b.len()
        && a.iter().zip(b).all(|(&ua, &ub)| {
            let delta = (ua - ub + TAU / 2.0).rem_euclid(TAU) - TAU / 2.0;
            delta.is_finite() && delta.abs() <= tolerance
        })
}

/// Per-face variant of the cycle-rim structured band: rims are sampled
/// LOCALLY at the requested deflection instead of pulled from the solid
/// tessellation's shared edge pool, so the `tessellate(topo, face, defl)`
/// route (which feeds `classify_point`'s meshes) gets the same watertight
/// wavy-band handling as the solid path. Returns `Ok(None)` when the face is
/// not a two-full-winding-rim band; the caller falls back.
pub(super) fn tessellate_band_face_local(
    topo: &Topology,
    face_data: &brepkit_topology::face::Face,
    deflection: f64,
    angular_tol: f64,
) -> Result<Option<super::TriangleMeshUV>, crate::OperationsError> {
    if !face_data.inner_wires().is_empty() {
        return Ok(None);
    }
    let (project, surf_normal): (ProjectFn, NormalFn) = match face_data.surface() {
        FaceSurface::Cylinder(c) => {
            let (c1, c2) = (c.clone(), c.clone());
            (
                Box::new(move |p| c1.project_point(p)),
                Box::new(move |u, v| c2.normal(u, v)),
            )
        }
        FaceSurface::Cone(c) => {
            let (c1, c2) = (c.clone(), c.clone());
            (
                Box::new(move |p| c1.project_point(p)),
                Box::new(move |u, v| c2.normal(u, v)),
            )
        }
        _ => return Ok(None),
    };

    // Curved wire edges → endpoint-connected cycles (the pool version's
    // structure; a closed single-edge NURBS loop has no by-construction
    // winding, so decline).
    let wire = topo.wire(face_data.outer_wire())?;
    let mut curved: Vec<(
        brepkit_topology::edge::EdgeId,
        brepkit_topology::vertex::VertexId,
        brepkit_topology::vertex::VertexId,
    )> = Vec::new();
    let mut seen: std::collections::HashSet<usize> = std::collections::HashSet::new();
    for oe in wire.edges() {
        let e = topo.edge(oe.edge())?;
        match e.curve() {
            EdgeCurve::NurbsCurve(_) if e.start() == e.end() => return Ok(None),
            EdgeCurve::Circle(_) | EdgeCurve::NurbsCurve(_) => {
                if seen.insert(oe.edge().index()) {
                    curved.push((oe.edge(), e.start(), e.end()));
                }
            }
            EdgeCurve::Line => {}
            EdgeCurve::Ellipse(_) | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
                return Ok(None);
            }
        }
    }
    let mut by_vertex: std::collections::HashMap<brepkit_topology::vertex::VertexId, Vec<usize>> =
        std::collections::HashMap::new();
    for (j, &(_, sv, ev)) in curved.iter().enumerate() {
        by_vertex.entry(sv).or_default().push(j);
        by_vertex.entry(ev).or_default().push(j);
    }
    let mut used = vec![false; curved.len()];
    let mut cycles: Vec<Vec<usize>> = Vec::new();
    for start in 0..curved.len() {
        if used[start] {
            continue;
        }
        let (_, origin, mut at) = curved[start];
        used[start] = true;
        let mut cycle = vec![start];
        let mut closed = curved[start].1 == curved[start].2 || at == origin;
        while !closed {
            let Some(&next) = by_vertex
                .get(&at)
                .and_then(|c| c.iter().find(|&&j| !used[j]))
            else {
                break;
            };
            used[next] = true;
            at = if curved[next].1 == at {
                curved[next].2
            } else {
                curved[next].1
            };
            cycle.push(next);
            closed = at == origin;
        }
        if !closed {
            return Ok(None);
        }
        cycles.push(cycle);
    }
    if cycles.len() != 2 {
        return Ok(None);
    }
    let wrap_pi = |d: f64| -> f64 { (d + TAU / 2.0).rem_euclid(TAU) - TAU / 2.0 };
    for cycle in &cycles {
        let mut winding = 0.0_f64;
        let mut whole_turn = false;
        let mut at: Option<brepkit_topology::vertex::VertexId> = None;
        for &ci in cycle {
            let (_, sv, ev) = curved[ci];
            if sv == ev {
                whole_turn = true;
                continue;
            }
            let (from, to) = match at {
                None => (sv, ev),
                Some(v) if v == sv => (sv, ev),
                Some(_) => (ev, sv),
            };
            let (u0, _) = project(topo.vertex(from)?.point());
            let (u1, _) = project(topo.vertex(to)?.point());
            if !u0.is_finite() || !u1.is_finite() {
                return Ok(None);
            }
            winding += wrap_pi(u1 - u0);
            at = Some(to);
        }
        if !whole_turn && (!winding.is_finite() || (winding.abs() - TAU).abs() > 1e-6) {
            return Ok(None);
        }
    }

    // Sample each rim's edges locally, dedup by quantized position, sort by
    // angle around the axis.
    let mut rims: Vec<Vec<Point3>> = Vec::with_capacity(2);
    for cycle in &cycles {
        let mut pts: Vec<Point3> = Vec::new();
        let mut keys: std::collections::HashSet<(i64, i64, i64)> = std::collections::HashSet::new();
        for &ci in cycle {
            let edge = topo.edge(curved[ci].0)?;
            for p in sample_edge(topo, edge, deflection, angular_tol, false)? {
                let k = point_merge_key(p, MERGE_GRID);
                if keys.insert(k) {
                    pts.push(p);
                }
            }
        }
        if pts.len() < 3 {
            return Ok(None);
        }
        if pts.iter().any(|&point| {
            let (u, v) = project(point);
            !u.is_finite() || !v.is_finite()
        }) {
            return Ok(None);
        }
        pts.sort_by(|a, b| {
            project(*a)
                .0
                .partial_cmp(&project(*b).0)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        rims.push(pts);
    }

    // Assemble the vertex arrays: ring 0 then ring 1.
    let n = rims[0].len();
    let m = rims[1].len();
    let mut positions: Vec<Point3> = Vec::with_capacity(n + m);
    positions.extend_from_slice(&rims[0]);
    positions.extend_from_slice(&rims[1]);
    let mut normals: Vec<Vec3> = Vec::with_capacity(n + m);
    let mut uvs: Vec<[f64; 2]> = Vec::with_capacity(n + m);
    for p in &positions {
        let (u, v) = project(*p);
        normals.push(surf_normal(u, v));
        uvs.push([u, v]);
    }

    // Angular zipper (the pool version's sweep, on local indices). Rotate
    // ring 1 to start just after ring 0's start angle.
    let ang = |i: usize| -> f64 { uvs[i][0] };
    let base = ang(0);
    let start1 = (0..m)
        .min_by(|&a, &b| {
            let ka = (ang(n + a) - base).rem_euclid(TAU);
            let kb = (ang(n + b) - base).rem_euclid(TAU);
            ka.partial_cmp(&kb).unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or(0);
    let ring0: Vec<usize> = (0..n).collect();
    let mut ring1: Vec<usize> = (n..n + m).collect();
    ring1.rotate_left(start1);
    let unwrap = |a: f64| (a - base).rem_euclid(TAU);
    let a0: Vec<f64> = ring0.iter().map(|&i| unwrap(ang(i))).collect();
    let a1: Vec<f64> = ring1.iter().map(|&i| unwrap(ang(i))).collect();

    let mut indices: Vec<u32> = Vec::with_capacity((n + m) * 3);
    let mut emit = |a: usize, b: usize, c: usize| {
        let (pa, pb, pc) = (positions[a], positions[b], positions[c]);
        let geo = (pb - pa).cross(pc - pa);
        if geo.length() < 1e-20 {
            return;
        }
        let (u, v) = project(pa);
        let outward = surf_normal(u, v);
        #[allow(clippy::cast_possible_truncation)]
        let mut tri = [a as u32, b as u32, c as u32];
        if geo.dot(outward) < 0.0 {
            tri.swap(1, 2);
        }
        indices.extend_from_slice(&tri);
    };
    let (mut i, mut j) = (0usize, 0usize);
    let (mut done0, mut done1) = (0usize, 0usize);
    while done0 < n || done1 < m {
        let next0 = if done0 >= n {
            f64::INFINITY
        } else if i + 1 < n {
            a0[i + 1]
        } else {
            a0[0] + TAU
        };
        let next1 = if done1 >= m {
            f64::INFINITY
        } else if j + 1 < m {
            a1[j + 1]
        } else {
            a1[0] + TAU
        };
        if next0 <= next1 {
            let ni = (i + 1) % n;
            emit(ring0[i], ring1[j], ring0[ni]);
            i = ni;
            done0 += 1;
        } else {
            let nj = (j + 1) % m;
            emit(ring0[i], ring1[j], ring1[nj]);
            j = nj;
            done1 += 1;
        }
    }

    Ok(Some(super::TriangleMeshUV {
        mesh: TriangleMesh {
            positions,
            normals,
            indices,
        },
        uvs,
    }))
}

/// Tessellate a cylinder/cone lateral "standard band" face directly from the
/// shared rim edge vertices, bypassing the snap path's proximity reconciliation.
///
/// The snap path tessellates the cylinder independently and snaps its rim
/// vertices to the shared edge pool by 1e-6 proximity; when the independent rim
/// sampling and the shared-edge sampling diverge by one segment (a radius/
/// deflection-dependent off-by-one) the rim vertices land at different angles,
/// fail the snap, and become near-coincident duplicates that crack the mesh
/// (issue #696: a drilled magnet hole). Reusing the shared rim vertices makes
/// the band watertight by construction.
///
/// Returns `Ok(true)` when the face is a simple two-rim band that was handled
/// here, `Ok(false)` when it is not (the caller then falls back to the snap or
/// CDT path). A "simple band" has no inner wires and exactly two rims
/// (everything else a seam line). Each rim is either one **closed** circle
/// edge or a CHAIN of open circle arcs at one constant `v` whose spans sum to
/// a full revolution — a boolean that splits a rim at tangency or crossing
/// points (e.g. the cone∪box inscribed-rim fuse, whose z=6 rim arrives as
/// four arcs each shared with a different corner face) still gets the
/// structured watertight band. Rims with equal shared-vertex counts sweep
/// index-paired exactly as before; unequal counts (each rim's sampling is
/// dictated by its own neighbours) are stitched with an angular zipper merge.
pub(super) fn tessellate_revolution_band_shared(
    topo: &Topology,
    face_data: &brepkit_topology::face::Face,
    edge_global_indices: &DetHashMap<usize, Vec<u32>>,
    merged: &mut TriangleMesh,
) -> Result<bool, crate::OperationsError> {
    if !face_data.inner_wires().is_empty() {
        return Ok(false);
    }

    let (project, surf_normal): (ProjectFn, NormalFn) = match face_data.surface() {
        FaceSurface::Cylinder(c) => {
            let (c1, c2) = (c.clone(), c.clone());
            (
                Box::new(move |p| c1.project_point(p)),
                Box::new(move |u, v| c2.normal(u, v)),
            )
        }
        FaceSurface::Cone(c) => {
            let (c1, c2) = (c.clone(), c.clone());
            (
                Box::new(move |p| c1.project_point(p)),
                Box::new(move |u, v| c2.normal(u, v)),
            )
        }
        _ => return Ok(false),
    };

    // Collect rim edges as endpoint-connected CYCLES of curved edges;
    // everything else must be a seam line. A rim is any cycle whose net
    // surface-u winding is a full revolution: one closed circle, a chain of
    // ring arcs, or a wavy mixed circle+NURBS chain (the winding-chain band
    // separator). A cycle that does not wind — a lens hole, a partial band
    // arc run bounded by non-seam generators — declines the structured
    // sweep, which would otherwise skin across the removed region.
    let wire = topo.wire(face_data.outer_wire())?;
    let mut curved: Vec<(
        usize,
        brepkit_topology::vertex::VertexId,
        brepkit_topology::vertex::VertexId,
    )> = Vec::new();
    let mut seen: std::collections::HashSet<usize> = std::collections::HashSet::new();
    for oe in wire.edges() {
        let e = topo.edge(oe.edge())?;
        match e.curve() {
            // A closed single-edge NURBS loop has no by-construction winding
            // (unlike a closed circle) — decline rather than guess.
            EdgeCurve::NurbsCurve(_) if e.start() == e.end() => return Ok(false),
            EdgeCurve::Circle(_) | EdgeCurve::NurbsCurve(_) => {
                if seen.insert(oe.edge().index()) {
                    curved.push((oe.edge().index(), e.start(), e.end()));
                }
            }
            EdgeCurve::Line => {}
            // Ellipse rims keep the CDT path.
            EdgeCurve::Ellipse(_) | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
                return Ok(false);
            }
        }
    }
    let project_u = |point| project(point).0;
    let Some(cycles) = collect_full_turn_rim_cycles(topo, &curved, &project_u, 2)? else {
        return Ok(false);
    };

    // Pull each rim's shared global vertex IDs. Chained pieces share their
    // joint vertices through the pool, so id-dedup merges the chain into one
    // ring; a closed circle carries its closing duplicate instead.
    let mut rims: Vec<Vec<u32>> = Vec::with_capacity(2);
    for cycle in &cycles {
        let mut ids: Vec<u32> = Vec::new();
        for &edge_index in &cycle.edge_indices {
            let Some(edge_ids) = edge_global_indices.get(&edge_index) else {
                return Ok(false);
            };
            ids.extend_from_slice(edge_ids);
        }
        ids.sort_unstable();
        ids.dedup();
        if ids.len() < 3 {
            return Ok(false);
        }
        rims.push(ids);
    }
    // Sort each rim by angle around the axis so the two rings align by index.
    let angle_of = |gid: u32, merged: &TriangleMesh| project(merged.positions[gid as usize]).0;
    if rims.iter().flatten().any(|&gid| {
        let Some(&point) = merged.positions.get(gid as usize) else {
            return true;
        };
        let (u, v) = project(point);
        !u.is_finite() || !v.is_finite()
    }) {
        return Ok(false);
    }
    for rim in &mut rims {
        rim.sort_by(|&a, &b| {
            angle_of(a, merged)
                .partial_cmp(&angle_of(b, merged))
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    }
    let equal_counts = rims[0].len() == rims[1].len();
    let n = rims[0].len();

    let angular_samples_align = if equal_counts {
        let angles_a: Vec<f64> = rims[0].iter().map(|&gid| angle_of(gid, merged)).collect();
        let angles_b: Vec<f64> = rims[1].iter().map(|&gid| angle_of(gid, merged)).collect();
        rim_angles_match(&angles_a, &angles_b, 1e-6)
    } else {
        false
    };

    // Emit default-oriented (non-reversed) triangles: the geometric normal
    // matches the surface outward normal, the convention `tessellate_analytic`
    // uses. The caller (`tessellate_face_with_shared_edges`) applies the global
    // `is_reversed` winding flip afterward, so we must NOT apply it here.
    let emit = |merged: &mut TriangleMesh, a: u32, b: u32, c: u32| {
        let (pa, pb, pc) = (
            merged.positions[a as usize],
            merged.positions[b as usize],
            merged.positions[c as usize],
        );
        // Skip degenerate triangles (two rim points at the same position).
        let geo = (pb - pa).cross(pc - pa);
        if geo.length() < 1e-20 {
            return;
        }
        let (u, v) = project(pa);
        let outward = surf_normal(u, v);
        let mut tri = [a, b, c];
        if geo.dot(outward) < 0.0 {
            tri.swap(1, 2);
        }
        merged.indices.extend_from_slice(&tri);
    };

    if angular_samples_align {
        // Equal counts: the historical index-paired sweep (kept byte-identical
        // for the calibrated closed-rim cases).
        for i in 0..n {
            let j = (i + 1) % n;
            let (b0, b1) = (rims[0][i], rims[0][j]);
            let (t0, t1) = (rims[1][i], rims[1][j]);
            emit(merged, b0, b1, t1);
            emit(merged, b0, t1, t0);
        }
        return Ok(true);
    }

    // Rims with different counts or angular phases cannot pair
    // index-for-index. Merge them by angle instead, which still uses only
    // shared-pool vertices, so both neighbours stay crack-free without
    // twisting equal-length but differently refined imported rims.
    let with_angles = |rim: &[u32], merged: &TriangleMesh| -> LatRing {
        rim.iter()
            .map(|&gid| (angle_of(gid, merged), gid))
            .collect()
    };
    let lo = with_angles(&rims[0], merged);
    let hi = with_angles(&rims[1], merged);
    stitch_rings(merged, &lo, &hi, &emit);

    Ok(true)
}

/// Tessellate a point-tipped cone's lateral face as a fan from the shared rim
/// vertices to the shared apex vertex.
///
/// This is the one-rim sibling of [`tessellate_revolution_band_shared`]. A
/// pointed cone's lateral face is bounded by a SINGLE closed rim circle plus a
/// doubled degenerate seam line running out to the apex, so the two-rim band
/// path above declines it, the CDT path emits nothing (the seam collapses the
/// UV boundary), and the face used to fall through to
/// [`tessellate_nonplanar_snap`].
///
/// That fallback is what cracked the cone. The snap path tessellates the face
/// from the cone's OWN parametric grid, whose `u = 0` ray is the surface
/// frame's x-axis, and then reconciles with the shared pool by 1e-6 proximity.
/// The rim circle's `t = 0` ray is the *circle's* frame x-axis, and for a cone
/// built with `make_cone` the two frames are half a turn apart (the base circle
/// is normal `+axis`, the cone axis runs apex→base, i.e. `-axis`). With `n`
/// segments the two rings therefore coincide only when `n` is EVEN; when `n` is
/// odd every rim sample lands exactly half a segment (≈ `πr/n`, four orders of
/// magnitude past the snap tolerance at r = 3, n = 209) from its pool
/// counterpart, nothing snaps, and the cone and its base cap end up sharing no
/// vertices at all.
///
/// Fanning the shared rim to the shared apex removes the parity coincidence
/// entirely: the face is watertight against its cap by construction, at any
/// radius, deflection and scale.
///
/// Returns `Ok(true)` when the face matched this pattern and was tessellated,
/// `Ok(false)` when it did not (the caller falls back to CDT/snap unchanged).
pub(super) fn tessellate_cone_apex_fan_shared(
    topo: &Topology,
    face_data: &brepkit_topology::face::Face,
    edge_global_indices: &DetHashMap<usize, Vec<u32>>,
    merged: &mut TriangleMesh,
) -> Result<bool, crate::OperationsError> {
    let FaceSurface::Cone(cone) = face_data.surface() else {
        return Ok(false);
    };
    if !face_data.inner_wires().is_empty() {
        return Ok(false);
    }

    // Exactly one closed rim circle; every other edge must be a (seam) line.
    let wire = topo.wire(face_data.outer_wire())?;
    let mut rim_edge_idx: Option<usize> = None;
    let mut seam_edge_indices: Vec<usize> = Vec::new();
    for oe in wire.edges() {
        let e = topo.edge(oe.edge())?;
        let idx = oe.edge().index();
        match e.curve() {
            EdgeCurve::Circle(_) if e.start() == e.end() => match rim_edge_idx {
                None => rim_edge_idx = Some(idx),
                Some(existing) if existing == idx => {}
                Some(_) => return Ok(false),
            },
            EdgeCurve::Line => {
                if !seam_edge_indices.contains(&idx) {
                    seam_edge_indices.push(idx);
                }
            }
            // An open rim arc, a trimmed cone, a NURBS boundary: not the
            // pointed-cone pattern. Let the caller decide.
            _ => return Ok(false),
        }
    }
    let Some(rim_idx) = rim_edge_idx else {
        return Ok(false);
    };
    if seam_edge_indices.is_empty() {
        return Ok(false);
    }

    // Shared rim vertices, closing duplicate dropped.
    let Some(rim_ids) = edge_global_indices.get(&rim_idx) else {
        return Ok(false);
    };
    let mut rim: Vec<u32> = rim_ids.clone();
    if rim.len() > 1 && rim.first() == rim.last() {
        rim.pop();
    }
    if rim.len() < 3 {
        return Ok(false);
    }
    if rim.iter().any(|&g| (g as usize) >= merged.positions.len()) {
        return Ok(false);
    }

    // The fan tip is the shared pool vertex sitting on the cone's apex. Take it
    // from the seam lines rather than interning a fresh point, so the cone and
    // any neighbour that also meets the apex agree on one vertex id.
    //
    // The match is relative to the rim's own radius, so it carries no length
    // unit: a scaled copy of the same cone accepts or rejects identically.
    let apex_pos = cone.apex();
    let rim_extent = rim
        .iter()
        .map(|&g| (merged.positions[g as usize] - apex_pos).length())
        .fold(0.0_f64, f64::max);
    if rim_extent <= 0.0 || !rim_extent.is_finite() {
        return Ok(false);
    }
    let apex_slack = rim_extent * 1e-9;
    let mut apex_gid: Option<u32> = None;
    for &se in &seam_edge_indices {
        let Some(ids) = edge_global_indices.get(&se) else {
            return Ok(false);
        };
        for &gid in ids {
            let Some(&pos) = merged.positions.get(gid as usize) else {
                return Ok(false);
            };
            if (pos - apex_pos).length() <= apex_slack {
                match apex_gid {
                    None => apex_gid = Some(gid),
                    // Two distinct vertex ids at the apex would leave the fan
                    // stitched to only one of them. Decline instead.
                    Some(existing) if existing == gid => {}
                    Some(_) => return Ok(false),
                }
            }
        }
    }
    let Some(apex_gid) = apex_gid else {
        return Ok(false);
    };
    if rim.contains(&apex_gid) {
        return Ok(false);
    }

    // Order the rim by its angle around the cone axis so consecutive pairs are
    // adjacent on the circle. The ids are distinct, so an unstable sort is both
    // equivalent and cheaper (no merge buffer, less codegen).
    rim.sort_unstable_by(|&a, &b| {
        let ua = cone.project_point(merged.positions[a as usize]).0;
        let ub = cone.project_point(merged.positions[b as usize]).0;
        ua.partial_cmp(&ub).unwrap_or(std::cmp::Ordering::Equal)
    });

    // Emit default-oriented (non-reversed) triangles, matching the convention
    // of `tessellate_analytic`; `tessellate_face_with_shared_edges` applies the
    // face's `is_reversed` flip afterwards.
    // Built first and committed only if EVERY wedge is sound: a fan that
    // silently dropped one wedge would leave a two-triangle hole, which is the
    // failure this path exists to prevent.
    let n = rim.len();
    let mut tris: Vec<u32> = Vec::with_capacity(n * 3);
    for i in 0..n {
        let (a, b, c) = (rim[i], rim[(i + 1) % n], apex_gid);
        let (pa, pb, pc) = (
            merged.positions[a as usize],
            merged.positions[b as usize],
            merged.positions[c as usize],
        );
        let (e1, e2) = (pb - pa, pc - pa);
        let geo = e1.cross(e2);
        // Relative degeneracy test: a sliver is one whose normal is negligible
        // against the product of its own edge lengths, which is scale-free.
        if geo.length() <= e1.length() * e2.length() * 1e-12 {
            return Ok(false);
        }
        let (u, v) = cone.project_point(pa);
        let mut tri = [a, b, c];
        if geo.dot(cone.normal(u, v)) < 0.0 {
            tri.swap(1, 2);
        }
        tris.extend_from_slice(&tri);
    }
    merged.indices.extend_from_slice(&tris);

    Ok(true)
}

/// Tessellate a torus band bounded by two full-turn circular rims and seamed by
/// ONE doubled open arc edge, in either orientation. Each rim may be one closed
/// circle edge or an endpoint-connected chain of open circle arcs:
///   * constant-`v` rims (latitude circles wrapping the ring angle `u`) — a
///     full analytic revolve of a profile arc, seamed by that arc; interior
///     full-`u` rows are swept along the tube angle;
///   * constant-`u` rims (tube circles wrapping `v`) — a PARTIAL-turn revolve
///     of a full circle profile, seamed by the vertex's sweep arc; interior
///     full-`v` rings are swept along the ring angle.
///
/// The rims split their periodic direction into two arcs; the seam arc's
/// midpoint picks which one the band covers (sweeping the wrong one would skin
/// the band across the material). Both rims reuse their SHARED pool vertices,
/// so the band meets its neighbour caps/walls crack-free — the CDT path
/// degenerates on these fully-wrapping UV images and the snap path re-samples
/// the rims independently (the #696 crack class).
///
/// Returns `Ok(false)` (caller falls back to CDT/snap) for any other torus
/// face.
pub(super) fn tessellate_torus_two_rim_band(
    topo: &Topology,
    face_data: &brepkit_topology::face::Face,
    deflection: f64,
    angular_tol: f64,
    edge_global_indices: &DetHashMap<usize, Vec<u32>>,
    merged: &mut TriangleMesh,
    point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
) -> Result<bool, crate::OperationsError> {
    use std::f64::consts::TAU;
    let FaceSurface::Torus(torus) = face_data.surface() else {
        return Ok(false);
    };
    if !face_data.inner_wires().is_empty() {
        return Ok(false);
    }

    let wire = topo.wire(face_data.outer_wire())?;
    let mut wire_edge_counts: DetHashMap<usize, usize> = DetHashMap::default();
    for oe in wire.edges() {
        let uses = wire_edge_counts.entry(oe.edge().index()).or_default();
        *uses += 1;
        if *uses > 2 {
            return Ok(false);
        }
    }

    // The seam is the one OPEN edge used exactly twice. A NURBS seam is the
    // analytic revolve of a recognised NURBS-circle profile arc; a line seam
    // is the rim-fillet band's degenerate chord between its contact circles.
    // The seam is midpoint-sampled only to select the covered arc, so its open
    // curve type is otherwise unrestricted.
    let mut seam_candidates = Vec::new();
    let mut seen: DetHashSet<usize> = DetHashSet::default();
    for oe in wire.edges() {
        if !seen.insert(oe.edge().index()) {
            continue;
        }
        let edge = topo.edge(oe.edge())?;
        if edge.start() != edge.end() && wire_edge_counts.get(&oe.edge().index()) == Some(&2) {
            seam_candidates.push(oe.edge());
        }
    }
    let [seam_eid] = seam_candidates.as_slice() else {
        return Ok(false);
    };
    let seam_eid = *seam_eid;

    // Every remaining edge must be a once-used circle. Closed circles form a
    // rim by themselves; open arcs are chained by endpoint identity below.
    let mut curved = Vec::new();
    seen.clear();
    for oe in wire.edges() {
        if !seen.insert(oe.edge().index()) || oe.edge() == seam_eid {
            continue;
        }
        if wire_edge_counts.get(&oe.edge().index()) != Some(&1) {
            return Ok(false);
        }
        let edge = topo.edge(oe.edge())?;
        match edge.curve() {
            EdgeCurve::Circle(_) => {
                curved.push((oe.edge().index(), edge.start(), edge.end()));
            }
            EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Line
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => return Ok(false),
        }
    }

    let (t1, t2, t3) = (torus.clone(), torus.clone(), torus.clone());
    let project = move |p: Point3| t1.project_point(p);
    let surf_eval = move |u: f64, v: f64| t2.evaluate(u, v);
    let surf_normal = move |u: f64, v: f64| t3.normal(u, v);

    // The periodic direction depends on which kind of torus band this is.
    // Walk the topology once for each candidate surface parameter; after the
    // constant-level test below establishes the mode, only its matching
    // full-turn result is accepted.
    let cycles_u = collect_full_turn_rim_cycles(topo, &curved, &|p| project(p).0, 2)?;
    let cycles_v = collect_full_turn_rim_cycles(topo, &curved, &|p| project(p).1, 2)?;
    let Some(pool_cycles) = cycles_u.as_ref().or(cycles_v.as_ref()) else {
        return Ok(false);
    };

    // Circular mean and max wrapped deviation of a set of angles.
    let circ_mean_spread = |angles: &[f64]| -> (f64, f64) {
        let (mut sx, mut sy) = (0.0_f64, 0.0_f64);
        for &a in angles {
            sx += a.cos();
            sy += a.sin();
        }
        let mean = sy.atan2(sx);
        let spread = angles
            .iter()
            .map(|&a| {
                let d = (a - mean + std::f64::consts::PI).rem_euclid(TAU) - std::f64::consts::PI;
                d.abs()
            })
            .fold(0.0_f64, f64::max);
        (mean.rem_euclid(TAU), spread)
    };

    // Project each rim's shared pool vertices (wrap-safe: a rim at angle 0
    // projects samples on both sides of the period).
    let mut raw: Vec<Vec<(f64, f64, u32)>> = Vec::with_capacity(2);
    for cycle in pool_cycles {
        let mut seen_gids: DetHashSet<u32> = DetHashSet::default();
        let mut pts: Vec<(f64, f64, u32)> = Vec::new();
        for &edge_index in &cycle.edge_indices {
            let Some(gids) = edge_global_indices.get(&edge_index) else {
                return Ok(false);
            };
            for &g in gids {
                if !seen_gids.insert(g) {
                    continue;
                }
                let (u, v) = project(merged.positions[g as usize]);
                pts.push((u, v, g));
            }
        }
        if pts.len() < 3 {
            return Ok(false);
        }
        raw.push(pts);
    }

    // Both rims must be constant in the SAME parameter: constant-v (latitude
    // rims, swept along the tube angle) or constant-u (tube rims, swept along
    // the ring angle).
    let spread_of = |pts: &[(f64, f64, u32)], pick_u: bool| -> (f64, f64) {
        let angles: Vec<f64> = pts
            .iter()
            .map(|&(u, v, _)| if pick_u { u } else { v })
            .collect();
        circ_mean_spread(&angles)
    };
    let (u_stats0, v_stats0) = (spread_of(&raw[0], true), spread_of(&raw[0], false));
    let (u_stats1, v_stats1) = (spread_of(&raw[1], true), spread_of(&raw[1], false));
    let lat_mode = if v_stats0.1 <= 1e-6 && v_stats1.1 <= 1e-6 {
        true
    } else if u_stats0.1 <= 1e-6 && u_stats1.1 <= 1e-6 {
        false
    } else {
        return Ok(false);
    };
    if if lat_mode {
        cycles_u.is_none()
    } else {
        cycles_v.is_none()
    } {
        return Ok(false);
    }
    let (lvl0, lvl1) = if lat_mode {
        (v_stats0.0, v_stats1.0)
    } else {
        (u_stats0.0, u_stats1.0)
    };

    // Rings keyed by the wrapping parameter, sorted, covering its full circle.
    let mut rims: Vec<LatRing> = Vec::with_capacity(2);
    for pts in &raw {
        let mut ring: LatRing = pts
            .iter()
            .map(|&(u, v, g)| if lat_mode { (u, g) } else { (v, g) })
            .collect();
        ring.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let max_gap = ring
            .windows(2)
            .map(|w| w[1].0 - w[0].0)
            .chain(std::iter::once(ring[0].0 + TAU - ring[ring.len() - 1].0))
            .fold(0.0_f64, f64::max);
        if max_gap > std::f64::consts::PI {
            return Ok(false);
        }
        rims.push(ring);
    }

    // The seam arc's midpoint picks which of the two swept-parameter arcs
    // between the rims the band covers.
    let seam_edge = topo.edge(seam_eid)?;
    let sp = topo.vertex(seam_edge.start())?.point();
    let ep = topo.vertex(seam_edge.end())?.point();
    let (d0, d1) = seam_edge.curve().domain_with_endpoints(sp, ep);
    let seam_mid = seam_edge
        .curve()
        .evaluate_with_endpoints(f64::midpoint(d0, d1), sp, ep);
    let (mid_u, mid_v) = project(seam_mid);
    let mid = if lat_mode { mid_v } else { mid_u };
    let fwd_span = (lvl1 - lvl0).rem_euclid(TAU);
    if fwd_span < 1e-9 || (TAU - fwd_span) < 1e-9 {
        return Ok(false);
    }
    let mid_off = (mid - lvl0).rem_euclid(TAU);
    let sweep = if mid_off <= fwd_span {
        fwd_span
    } else {
        -(TAU - fwd_span)
    };

    // Interior rows along the swept parameter; each row wraps the other
    // parameter's full circle.
    let (sweep_radius, wrap_radius) = if lat_mode {
        (
            torus.minor_radius(),
            torus.major_radius() + torus.minor_radius(),
        )
    } else {
        (
            torus.major_radius() + torus.minor_radius(),
            torus.minor_radius(),
        )
    };
    let n_rows =
        segments_for_chord_deviation_a(sweep_radius, sweep.abs(), deflection, angular_tol, true)
            .max(1);
    let full_circle_cols =
        segments_for_chord_deviation_a(wrap_radius, TAU, deflection, angular_tol, true);
    let n_cols = rims[0].len().max(rims[1].len()).max(full_circle_cols);

    let emit = make_band_emit(&project, &surf_normal);
    let mut prev_ring: LatRing = rims[0].clone();
    for i in 1..n_rows {
        #[allow(clippy::cast_precision_loss)]
        let t = i as f64 / n_rows as f64;
        let level = lvl0 + sweep * t;
        let mut row: LatRing = Vec::with_capacity(n_cols);
        for j in 0..n_cols {
            #[allow(clippy::cast_precision_loss)]
            let a = TAU * (j as f64) / (n_cols as f64);
            let (u, v) = if lat_mode { (a, level) } else { (level, a) };
            let p = surf_eval(u, v);
            let key = point_merge_key(p, MERGE_GRID);
            let gid = *point_to_global.entry(key).or_insert_with(|| {
                #[allow(clippy::cast_possible_truncation)]
                let idx = merged.positions.len() as u32;
                merged.positions.push(p);
                merged.normals.push(surf_normal(u, v));
                idx
            });
            row.push((a, gid));
        }
        stitch_rings(merged, &prev_ring, &row, &emit);
        prev_ring = row;
    }
    stitch_rings(merged, &prev_ring, &rims[1], &emit);
    Ok(true)
}

/// A boundary ring of a latitude band: each entry is `(u_angle, global_id)`,
/// with `u_angle ∈ [0, 2π)`. Sorted ascending by angle so two rings align by
/// longitude during stitching.
type LatRing = Vec<(f64, u32)>;

fn has_single_period_winding(angles: &[f64]) -> bool {
    use std::f64::consts::{PI, TAU};

    if angles.len() < 2 {
        return false;
    }
    let unwrap_delta = |from: f64, to: f64| {
        let delta = to - from;
        delta - TAU * ((delta + PI) / TAU).floor()
    };
    let winding = angles
        .windows(2)
        .fold(0.0, |acc, pair| acc + unwrap_delta(pair[0], pair[1]))
        + unwrap_delta(angles[angles.len() - 1], angles[0]);
    (winding.abs() - TAU).abs() <= 1.0e-6
}

/// Collect a torus face wire's boundary as a ring of `(tube-angle v, shared gid)`
/// sorted by `v`, taking the SHARED global vertices (so the ring shares the
/// notch walls' vertices) and projecting to the torus `(u, v)`. Accepts edges of
/// any curve type (the notch seam arcs are NURBS). Returns `None` if any edge is
/// missing from the shared pool, or its ordered edge samples do not have a
/// single full-period winding in `v`.
fn collect_torus_phi_ring(
    topo: &Topology,
    wire_id: brepkit_topology::wire::WireId,
    torus: &brepkit_math::surfaces::ToroidalSurface,
    edge_global_indices: &DetHashMap<usize, Vec<u32>>,
    merged: &TriangleMesh,
) -> Result<Option<Vec<(f64, u32)>>, crate::OperationsError> {
    let wire = topo.wire(wire_id)?;
    let mut gids: Vec<u32> = Vec::new();
    let mut phi_path = Vec::new();
    for oe in wire.edges() {
        let Some(edge_gids) = edge_global_indices.get(&oe.edge().index()) else {
            return Ok(None);
        };
        gids.extend_from_slice(edge_gids);
        let edge = topo.edge(oe.edge())?;
        let (start, end) = (
            topo.vertex(edge.start())?.point(),
            topo.vertex(edge.end())?.point(),
        );
        for k in 0..=8 {
            let f = f64::from(k) / 8.0;
            let t = if oe.is_forward() { f } else { 1.0 - f };
            let point = edge.curve().evaluate_with_endpoints(t, start, end);
            phi_path.push(torus.project_point(point).1);
        }
    }
    if !has_single_period_winding(&phi_path) {
        return Ok(None);
    }
    let mut seen: DetHashSet<u32> = DetHashSet::default();
    let mut ring: Vec<(f64, u32)> = Vec::with_capacity(gids.len());
    for g in gids {
        if !seen.insert(g) {
            continue;
        }
        let (_, v) = torus.project_point(merged.positions[g as usize]);
        ring.push((v, g));
    }
    if ring.len() < 3 {
        return Ok(None);
    }
    ring.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    Ok(Some(ring))
}

/// Tessellate the `torus − box`-style notch band: a kept toroidal patch that
/// WRAPS the tube angle `v` fully and is bounded by TWO `v`-wrapping seam-arc
/// loops at the two ends of a ring-angle (`u`) span (the box notch's `±y` walls).
/// The band is swept structurally along `u` from one boundary loop to the other
/// the LONG way (through `u = π`, the 294° kept side), with full-`v` interior
/// rings; both boundary loops use their SHARED wall vertices, so the band and the
/// plane notch walls meet crack-free (watertight). Returns `false` (defer to the
/// CDT path) for any torus face that is not this two-`v`-loop notch band.
///
/// Distinct from [`tessellate_latitude_band_shared`]: there the two boundaries
/// are constant-`v` latitude circles swept along `v`; here they wrap `v` and the
/// sweep is along `u`.
pub(super) fn tessellate_torus_notch_band(
    topo: &Topology,
    face_data: &brepkit_topology::face::Face,
    deflection: f64,
    angular_tol: f64,
    edge_global_indices: &DetHashMap<usize, Vec<u32>>,
    merged: &mut TriangleMesh,
    point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
) -> Result<bool, crate::OperationsError> {
    use std::f64::consts::{PI, TAU};
    let FaceSurface::Torus(torus) = face_data.surface() else {
        return Ok(false);
    };
    if face_data.inner_wires().len() != 1 {
        return Ok(false);
    }
    let t1 = torus.clone();
    let t2 = torus.clone();
    let project = move |p: Point3| t1.project_point(p);
    let surf_normal = move |u: f64, v: f64| t2.normal(u, v);

    // Both boundary loops wrap the tube (v) once, with their shared wall gids.
    let Some(ring_a) = collect_torus_phi_ring(
        topo,
        face_data.outer_wire(),
        torus,
        edge_global_indices,
        merged,
    )?
    else {
        return Ok(false);
    };
    let Some(ring_b) = collect_torus_phi_ring(
        topo,
        face_data.inner_wires()[0],
        torus,
        edge_global_indices,
        merged,
    )?
    else {
        return Ok(false);
    };

    // Ring-angle (u) of each loop: each loop sits at a u-BAND (the box wall's cut
    // varies in u with the tube angle), one near u_a, the other near u_b, the
    // kept band the LONG way between them. Take each loop's mean u (wrap-safe)
    // plus its half-u-spread, so the interior rows start at each loop's KEPT-SIDE
    // edge (mean ± spread toward the band midpoint), NOT its mean — otherwise the
    // first/last interior row sits INSIDE the loop's u-band and the stitch folds
    // back over the boundary strip, under-covering the band.
    let mean_u = |ring: &[(f64, u32)]| -> f64 {
        let (mut sx, mut sy) = (0.0, 0.0);
        for &(_, g) in ring {
            let (u, _) = project(merged.positions[g as usize]);
            sx += u.cos();
            sy += u.sin();
        }
        sy.atan2(sx).rem_euclid(TAU)
    };
    // Max signed u-offset of a ring's vertices from its mean (wrap into (-π,π]).
    let half_spread = |ring: &[(f64, u32)], mean: f64| -> f64 {
        ring.iter()
            .map(|&(_, g)| {
                let (u, _) = project(merged.positions[g as usize]);
                let d = (u - mean + PI).rem_euclid(TAU) - PI;
                d.abs()
            })
            .fold(0.0_f64, f64::max)
    };
    let u_a = mean_u(&ring_a);
    let u_b = mean_u(&ring_b);
    let spread_a = half_spread(&ring_a, u_a);
    let spread_b = half_spread(&ring_b, u_b);

    // Sweep the LONG way from ring_a toward ring_b (through the kept far side).
    let fwd_span = (u_b - u_a).rem_euclid(TAU); // a -> b increasing u
    // The interior must lie on the long arc; start just past each loop's
    // kept-side edge so no interior row overlaps a boundary loop's u-band.
    let (u_start, u_end) = if fwd_span >= PI {
        // a -> b the long way is INCREASING u: kept edge of a is u_a+spread_a,
        // of b is u_b-spread_b (i.e. u_a+fwd_span-spread_b).
        (u_a + spread_a, u_a + fwd_span - spread_b)
    } else {
        // a -> b the long way is DECREASING u.
        (u_a - spread_a, u_a - (TAU - fwd_span) + spread_b)
    };
    let span = (u_end - u_start).abs();
    if span < 1e-6 {
        return Ok(false);
    }

    // Interior rows: full-v circles at constant u, stepped along the sweep. Count
    // from chord deviation over the band's u-arc-length (radius ≈ R, the ring).
    let n_u =
        segments_for_chord_deviation_a(torus.major_radius(), span, deflection, angular_tol, true)
            .max(2);
    // v-resolution: a full tube circle.
    let n_v =
        segments_for_chord_deviation_a(torus.minor_radius(), TAU, deflection, angular_tol, true)
            .max(8);

    // Build interior rings as `LatRing` (sorted by v) of fresh vertices.
    let build_u_ring = |u: f64,
                        merged: &mut TriangleMesh,
                        point_to_global: &mut DetHashMap<(i64, i64, i64), u32>|
     -> LatRing {
        let mut row: LatRing = Vec::with_capacity(n_v);
        for j in 0..n_v {
            #[allow(clippy::cast_precision_loss)]
            let v = TAU * (j as f64) / (n_v as f64);
            let p = torus.evaluate(u, v);
            let key = point_merge_key(p, MERGE_GRID);
            let gid = *point_to_global.entry(key).or_insert_with(|| {
                let idx = merged.positions.len() as u32;
                merged.positions.push(p);
                merged.normals.push(surf_normal(u, v));
                idx
            });
            row.push((v, gid));
        }
        row.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        row
    };

    let emit = make_band_emit(&project, &surf_normal);
    let idx_start = merged.indices.len();

    // Stitch ring_a -> interior rows -> ring_b. All rings sorted by v; `v` is the
    // ring parameter passed to `stitch_rings` (it walks the shared tube angle).
    let mut prev: LatRing = ring_a;
    for iu in 1..n_u {
        #[allow(clippy::cast_precision_loss)]
        let u = u_start + (u_end - u_start) * (iu as f64) / (n_u as f64);
        let row = build_u_ring(u.rem_euclid(TAU), merged, point_to_global);
        stitch_rings(merged, &prev, &row, &emit);
        prev = row;
    }
    stitch_rings(merged, &prev, &ring_b, &emit);

    // Orient the whole band once against the torus outward normal.
    orient_triangle_run(merged, idx_start, &project, &surf_normal);
    Ok(true)
}

/// Tessellate a sphere/torus latitude band (the annular region between two
/// constant-`v` full-revolution boundaries) as a structured UV grid.
///
/// The CDT path cannot bound this band: each constant-`v` latitude boundary
/// projects to a back-and-forth horizontal segment of zero UV area, so the
/// 2D polygon degenerates and the triangulation fills the removed polar cap
/// (the tunnel mouth on a bored sphere is skinned over). Like the cylinder/cone
/// `tessellate_revolution_band_shared`, this builds the band directly from the
/// shared boundary vertices instead.
///
/// Unlike the ruled cylinder/cone band (whose two rims connect directly because
/// the surface is straight in `v`), a sphere/torus band bulges between its two
/// latitudes, so intermediate latitude rows are inserted until the chord error
/// in `v` stays within `deflection`. The two boundary rows reuse the shared rim
/// global vertex IDs (watertight by construction); interior-row vertices are new
/// face-local points evaluated on the surface across the full `u` ring.
///
/// Returns `Ok(true)` when the face is such a band and was handled here, else
/// `Ok(false)` (the caller then takes the CDT/snap path). Detection is
/// deliberately conservative: a face qualifies only if its surface is a sphere
/// or torus, it has exactly one inner wire, and both the outer and inner wires
/// are closed full-revolution loops, each at a single constant `v`, built only
/// from `Line`/`Circle` edges, at two distinct `v` levels.
#[allow(clippy::too_many_lines)]
pub(super) fn tessellate_latitude_band_shared(
    topo: &Topology,
    face_data: &brepkit_topology::face::Face,
    deflection: f64,
    angular_tol: f64,
    edge_global_indices: &DetHashMap<usize, Vec<u32>>,
    merged: &mut TriangleMesh,
    point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
) -> Result<bool, crate::OperationsError> {
    if face_data.inner_wires().len() != 1 {
        return Ok(false);
    }

    let (project, surf_eval, surf_normal): (ProjectFn, EvalFn, NormalFn) = match face_data.surface()
    {
        FaceSurface::Sphere(s) => {
            let (s1, s2, s3) = (s.clone(), s.clone(), s.clone());
            (
                Box::new(move |p| s1.project_point(p)),
                Box::new(move |u, v| s2.evaluate(u, v)),
                Box::new(move |u, v| s3.normal(u, v)),
            )
        }
        FaceSurface::Torus(t) => {
            let (t1, t2, t3) = (t.clone(), t.clone(), t.clone());
            (
                Box::new(move |p| t1.project_point(p)),
                Box::new(move |u, v| t2.evaluate(u, v)),
                Box::new(move |u, v| t3.normal(u, v)),
            )
        }
        _ => return Ok(false),
    };

    let band_radius = match face_data.surface() {
        FaceSurface::Sphere(s) => s.radius(),
        FaceSurface::Torus(t) => t.minor_radius(),
        _ => return Ok(false),
    };
    let emit = make_band_emit(project.as_ref(), surf_normal.as_ref());
    let full_circle_cols = segments_for_chord_deviation_a(
        band_radius,
        std::f64::consts::TAU,
        deflection,
        angular_tol,
        true,
    );

    let outer_wid = face_data.outer_wire();
    let inner_wid = face_data.inner_wires()[0];

    // Case 1 — both boundaries are single constant-v latitude circles (the
    // bored-quadric band, e.g. sphere − through-cylinder). Sweep constant-v
    // interior rows between them.
    let outer_const = collect_constant_v_ring(
        topo,
        outer_wid,
        project.as_ref(),
        edge_global_indices,
        merged,
    )?;
    let inner_const = collect_constant_v_ring(
        topo,
        inner_wid,
        project.as_ref(),
        edge_global_indices,
        merged,
    )?;

    if let (Some((v_outer, ring_outer)), Some((v_inner, ring_inner))) = (&outer_const, &inner_const)
    {
        let mut rings = [
            (*v_outer, ring_outer.clone()),
            (*v_inner, ring_inner.clone()),
        ];
        rings.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let (v_lo, ring_lo) = (&rings[0].0, &rings[0].1);
        let (v_hi, ring_hi) = (&rings[1].0, &rings[1].1);
        let (v_lo, v_hi) = (*v_lo, *v_hi);
        if (v_hi - v_lo).abs() < 1e-9 {
            return Ok(false);
        }
        let n_v =
            segments_for_chord_deviation_a(band_radius, v_hi - v_lo, deflection, angular_tol, true)
                .max(1);
        let n_u_interior = ring_lo.len().max(ring_hi.len()).max(full_circle_cols);
        let mut prev_ring: LatRing = ring_lo.clone();
        for iv in 1..n_v {
            #[allow(clippy::cast_precision_loss)]
            let t = iv as f64 / n_v as f64;
            let v = v_lo + (v_hi - v_lo) * t;
            let row = build_interior_row(
                v,
                n_u_interior,
                surf_eval.as_ref(),
                surf_normal.as_ref(),
                merged,
                point_to_global,
            );
            stitch_rings(merged, &prev_ring, &row, &emit);
            prev_ring = row;
        }
        stitch_rings(merged, &prev_ring, ring_hi, &emit);
        return Ok(true);
    }

    // Case 2 — a COLLAR: the inner wire is a constant-v cap circle, the outer
    // wire is a full-longitude-wrap "floor" at varying v (great-circle/seam
    // arcs, e.g. a box ∩ sphere patch). Sweep interior rows whose per-column v
    // interpolates from the scalloped floor up to the cap.
    let Some((v_cap, cap_ring)) = inner_const else {
        return Ok(false);
    };
    let Some(floor) = collect_var_v_ring(
        topo,
        outer_wid,
        project.as_ref(),
        edge_global_indices,
        merged,
    )?
    else {
        return Ok(false);
    };
    // The collar must straddle the cap (the floor sits on the far side of the
    // cap latitude). Reject a near-flat outer wire (would be Case 1).
    let floor_v_min = floor.iter().map(|r| r.1).fold(f64::INFINITY, f64::min);
    let floor_v_max = floor.iter().map(|r| r.1).fold(f64::NEG_INFINITY, f64::max);
    if (floor_v_max - floor_v_min) <= 1e-6 {
        return Ok(false); // constant-v outer — Case 1 already tried it
    }
    let floor_v_near = if (v_cap - floor_v_max).abs() >= (v_cap - floor_v_min).abs() {
        floor_v_max
    } else {
        floor_v_min
    };
    if (v_cap - floor_v_near).abs() < 1e-9 {
        return Ok(false);
    }

    // The outer (scalloped) ring is the lower boundary; sweep up to the cap.
    // Use the absolute band height: the floor can sit above the cap latitude
    // (a southern collar), and a negative range trips the chord-deviation
    // helper's `<= 0` fallback (a fixed count) instead of scaling with height.
    let n_v = segments_for_chord_deviation_a(
        band_radius,
        (v_cap - floor_v_near).abs(),
        deflection,
        angular_tol,
        true,
    )
    .max(1);

    // Lower boundary ring as a LatRing (drop the v component; the gid carries
    // the shared scalloped-floor vertex).
    let floor_ring: LatRing = floor.iter().map(|&(u, _, g)| (u, g)).collect();

    // Emit the collar's triangles in the rings' consistent walk order WITHOUT a
    // per-triangle normal flip, then orient the whole collar once below. (The
    // per-triangle normal fix that the bored-band path uses is unstable for the
    // thin stitch triangles bridging the clustered floor to the even cap — it
    // flips neighbours inconsistently. A single decision keeps the collar a
    // coherent 2-manifold.)
    let collar_idx_start = merged.indices.len();
    let emit_raw = |merged: &mut TriangleMesh, a: u32, b: u32, c: u32| {
        if a == b || b == c || a == c {
            return;
        }
        let (pa, pb, pc) = (
            merged.positions[a as usize],
            merged.positions[b as usize],
            merged.positions[c as usize],
        );
        if (pb - pa).cross(pc - pa).length() < 1e-20 {
            return;
        }
        merged.indices.extend_from_slice(&[a, b, c]);
    };

    // Connect the floor to each interior row as COLUMN-ALIGNED quad strips
    // (same longitudes, same count), then zipper only the topmost interior row
    // to the cap (different longitude sampling) with `stitch_rings`.
    let mut prev_ring: LatRing = floor_ring;
    for iv in 1..n_v {
        #[allow(clippy::cast_precision_loss)]
        let t = iv as f64 / n_v as f64;
        let row = build_collar_row(
            &floor,
            v_cap,
            t,
            surf_eval.as_ref(),
            surf_normal.as_ref(),
            merged,
            point_to_global,
        );
        emit_aligned_quad_strip(merged, &prev_ring, &row, &emit_raw);
        prev_ring = row;
    }
    stitch_rings(merged, &prev_ring, &cap_ring, &emit_raw);

    // Orient the collar as a whole: pick the best-conditioned triangle (largest
    // area), compare its geometric normal to the surface outward normal at its
    // centroid, and flip every collar triangle's winding if they disagree.
    orient_triangle_run(
        merged,
        collar_idx_start,
        project.as_ref(),
        surf_normal.as_ref(),
    );

    Ok(true)
}

/// Make a contiguous run of triangles (added from `idx_start` onward) wind
/// consistently outward. The run is already wound coherently (one orientation)
/// by construction; this only decides whether that single orientation needs a
/// global flip, using the largest-area triangle (most reliable normal) against
/// the surface outward normal at its centroid.
fn orient_triangle_run(
    merged: &mut TriangleMesh,
    idx_start: usize,
    project: &dyn Fn(Point3) -> (f64, f64),
    surf_normal: &dyn Fn(f64, f64) -> Vec3,
) {
    let mut best_area = 0.0_f64;
    let mut flip = false;
    let mut t = idx_start;
    while t + 3 <= merged.indices.len() {
        let (a, b, c) = (
            merged.indices[t],
            merged.indices[t + 1],
            merged.indices[t + 2],
        );
        let (pa, pb, pc) = (
            merged.positions[a as usize],
            merged.positions[b as usize],
            merged.positions[c as usize],
        );
        let geo = (pb - pa).cross(pc - pa);
        let area = geo.length();
        if area > best_area {
            best_area = area;
            let centroid = Point3::new(
                (pa.x() + pb.x() + pc.x()) / 3.0,
                (pa.y() + pb.y() + pc.y()) / 3.0,
                (pa.z() + pb.z() + pc.z()) / 3.0,
            );
            let (u, v) = project(centroid);
            flip = geo.dot(surf_normal(u, v)) < 0.0;
        }
        t += 3;
    }
    if flip {
        let mut t = idx_start;
        while t + 3 <= merged.indices.len() {
            merged.indices.swap(t + 1, t + 2);
            t += 3;
        }
    }
}

/// Connect two column-aligned rings (identical longitude order and count) as a
/// quad strip: column `i` of `lo` joins column `i` of `hi`. Each quad is split
/// into two triangles via the supplied `emit` closure. The collar path passes
/// `emit_raw` (no per-triangle winding correction — the whole run is oriented
/// once afterward by [`orient_triangle_run`], which is stable for the thin
/// stitch triangles). Watertight by construction when the rings share columns.
fn emit_aligned_quad_strip(
    merged: &mut TriangleMesh,
    lo: &LatRing,
    hi: &LatRing,
    emit: &impl Fn(&mut TriangleMesh, u32, u32, u32),
) {
    let n = lo.len();
    if n < 2 || hi.len() != n {
        // Counts diverged (a merged-away duplicate column) — fall back to the
        // longitude zipper, which tolerates unequal counts.
        stitch_rings(merged, lo, hi, emit);
        return;
    }
    for i in 0..n {
        let j = (i + 1) % n;
        let (l0, l1) = (lo[i].1, lo[j].1);
        let (h0, h1) = (hi[i].1, hi[j].1);
        emit(merged, l0, l1, h1);
        emit(merged, l0, h1, h0);
    }
}

/// Collect a wire's shared boundary vertices as a `(v_level, ring)` pair, or
/// `None` if the wire is not a closed full-revolution loop at a single constant
/// `v` (built only from `Line`/`Circle` edges).
fn collect_constant_v_ring(
    topo: &Topology,
    wire_id: brepkit_topology::wire::WireId,
    project: &dyn Fn(Point3) -> (f64, f64),
    edge_global_indices: &DetHashMap<usize, Vec<u32>>,
    merged: &TriangleMesh,
) -> Result<Option<(f64, LatRing)>, crate::OperationsError> {
    let wire = topo.wire(wire_id)?;
    let mut gids: Vec<u32> = Vec::new();
    for oe in wire.edges() {
        let e = topo.edge(oe.edge())?;
        match e.curve() {
            EdgeCurve::Line | EdgeCurve::Circle(_) => {}
            _ => return Ok(None),
        }
        let Some(edge_gids) = edge_global_indices.get(&oe.edge().index()) else {
            return Ok(None);
        };
        for &g in edge_gids {
            gids.push(g);
        }
    }
    if gids.len() < 3 {
        return Ok(None);
    }

    // Deduplicate to unique global IDs and check they all sit at one constant v
    // while their longitudes cover the full circle (a full revolution).
    let mut seen: DetHashSet<u32> = DetHashSet::default();
    let mut ring: LatRing = Vec::with_capacity(gids.len());
    let mut v_sum = 0.0;
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;
    for g in gids {
        if !seen.insert(g) {
            continue;
        }
        let p = merged.positions[g as usize];
        let (u, v) = project(p);
        v_sum += v;
        v_min = v_min.min(v);
        v_max = v_max.max(v);
        ring.push((u, g));
    }
    if ring.len() < 3 {
        return Ok(None);
    }
    if (v_max - v_min) > 1e-6 {
        return Ok(None);
    }
    ring.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    // Full-revolution check: the largest angular gap between consecutive
    // longitudes (including the wrap-around) must be well under a full turn —
    // otherwise this is a partial arc, not a closed latitude loop.
    let max_gap = ring
        .windows(2)
        .map(|w| w[1].0 - w[0].0)
        .chain(std::iter::once(
            ring[0].0 + std::f64::consts::TAU - ring[ring.len() - 1].0,
        ))
        .fold(0.0_f64, f64::max);
    if max_gap > std::f64::consts::PI {
        return Ok(None);
    }

    let v_level = v_sum / ring.len() as f64;
    Ok(Some((v_level, ring)))
}

/// A boundary ring whose latitude varies with longitude: `(u_angle, v, gid)`
/// sorted ascending by `u_angle`. Used for a collar's scalloped outer wire (the
/// great-circle/seam-arc "floor" of a box∩sphere patch), which encircles
/// longitude fully but at a non-constant `v`.
type VarRing = Vec<(f64, f64, u32)>;

/// Collect a wire's shared boundary vertices as a longitude-sorted [`VarRing`],
/// or `None` if the wire is not a closed full-revolution loop (built only from
/// `Line`/`Circle` edges). Unlike [`collect_constant_v_ring`], the latitude may
/// vary with longitude.
fn collect_var_v_ring(
    topo: &Topology,
    wire_id: brepkit_topology::wire::WireId,
    project: &dyn Fn(Point3) -> (f64, f64),
    edge_global_indices: &DetHashMap<usize, Vec<u32>>,
    merged: &TriangleMesh,
) -> Result<Option<VarRing>, crate::OperationsError> {
    let wire = topo.wire(wire_id)?;
    let mut gids: Vec<u32> = Vec::new();
    for oe in wire.edges() {
        let e = topo.edge(oe.edge())?;
        match e.curve() {
            EdgeCurve::Line | EdgeCurve::Circle(_) => {}
            _ => return Ok(None),
        }
        let Some(edge_gids) = edge_global_indices.get(&oe.edge().index()) else {
            return Ok(None);
        };
        gids.extend_from_slice(edge_gids);
    }
    if gids.len() < 3 {
        return Ok(None);
    }
    let mut seen: DetHashSet<u32> = DetHashSet::default();
    let mut ring: VarRing = Vec::with_capacity(gids.len());
    for g in gids {
        if !seen.insert(g) {
            continue;
        }
        let (u, v) = project(merged.positions[g as usize]);
        ring.push((u, v, g));
    }
    if ring.len() < 3 {
        return Ok(None);
    }
    ring.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    // Full-revolution check: the largest longitude gap (including wrap-around)
    // must be under a full turn — else it is a partial arc, not a closed loop.
    let max_gap = ring
        .windows(2)
        .map(|w| w[1].0 - w[0].0)
        .chain(std::iter::once(
            ring[0].0 + std::f64::consts::TAU - ring[ring.len() - 1].0,
        ))
        .fold(0.0_f64, f64::max);
    if max_gap > std::f64::consts::PI {
        return Ok(None);
    }
    Ok(Some(ring))
}

/// Build an interior latitude row of `n` evenly-spaced new vertices at constant
/// `v`, returning them as a ring sorted by longitude.
fn build_interior_row(
    v: f64,
    n: usize,
    surf_eval: &dyn Fn(f64, f64) -> Point3,
    surf_normal: &dyn Fn(f64, f64) -> Vec3,
    merged: &mut TriangleMesh,
    point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
) -> LatRing {
    let mut row: LatRing = Vec::with_capacity(n);
    for i in 0..n {
        let u = std::f64::consts::TAU * (i as f64) / (n as f64);
        let p = surf_eval(u, v);
        let key = point_merge_key(p, MERGE_GRID);
        let gid = *point_to_global.entry(key).or_insert_with(|| {
            let idx = merged.positions.len() as u32;
            merged.positions.push(p);
            merged.normals.push(surf_normal(u, v));
            idx
        });
        row.push((u, gid));
    }
    row
}

/// Build a collar interior row at the floor ring's exact longitudes — one
/// column per floor vertex — each column's `v` interpolated a fraction `t` from
/// that floor vertex's `v` up to the constant cap latitude `v_cap`. Keeping the
/// interior rows column-aligned with the scalloped floor lets them connect as
/// clean quad strips (no longitude zippering, so the scallop corners — where
/// the floor dips to the seam — produce no flipped slivers).
fn build_collar_row(
    floor: &VarRing,
    v_cap: f64,
    t: f64,
    surf_eval: &dyn Fn(f64, f64) -> Point3,
    surf_normal: &dyn Fn(f64, f64) -> Vec3,
    merged: &mut TriangleMesh,
    point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
) -> LatRing {
    let mut row: LatRing = Vec::with_capacity(floor.len());
    for &(u, v_floor, _) in floor {
        let v = v_floor + (v_cap - v_floor) * t;
        let p = surf_eval(u, v);
        let key = point_merge_key(p, MERGE_GRID);
        let gid = *point_to_global.entry(key).or_insert_with(|| {
            let idx = merged.positions.len() as u32;
            merged.positions.push(p);
            merged.normals.push(surf_normal(u, v));
            idx
        });
        row.push((u, gid));
    }
    row
}

/// Emit a default-oriented (non-reversed) triangle, mirroring the orientation
/// convention of [`tessellate_revolution_band_shared`]: the geometric normal is
/// flipped to match the surface outward normal. The caller applies the global
/// `is_reversed` winding flip afterward.
fn make_band_emit<'a>(
    project: &'a dyn Fn(Point3) -> (f64, f64),
    surf_normal: &'a dyn Fn(f64, f64) -> Vec3,
) -> impl Fn(&mut TriangleMesh, u32, u32, u32) + 'a {
    move |merged: &mut TriangleMesh, a: u32, b: u32, c: u32| {
        if a == b || b == c || a == c {
            return;
        }
        let (pa, pb, pc) = (
            merged.positions[a as usize],
            merged.positions[b as usize],
            merged.positions[c as usize],
        );
        let geo = (pb - pa).cross(pc - pa);
        if geo.length() < 1e-20 {
            return;
        }
        // Reference the outward normal at all three vertices (averaged), not just
        // `pa`: a thin stitch triangle bridging a clustered ring to an even one
        // can sit nearly tangent to the surface, where the single-vertex normal
        // makes `geo.dot(outward)` sign-unstable and flips the triangle relative
        // to its neighbours. The averaged normal is stable across the triangle.
        let n_at = |p: Point3| -> Vec3 {
            let (u, v) = project(p);
            surf_normal(u, v)
        };
        let outward = n_at(pa) + n_at(pb) + n_at(pc);
        let mut tri = [a, b, c];
        if geo.dot(outward) < 0.0 {
            tri.swap(1, 2);
        }
        merged.indices.extend_from_slice(&tri);
    }
}

/// Triangulate the band between two coaxial latitude rings, both sorted by
/// longitude in `[0, 2π)`, whose vertex counts/phases may differ. Walks both
/// rings forward in longitude, at each step advancing whichever ring's next
/// vertex has the smaller longitude (relative to a monotonically increasing
/// base) and emitting one triangle per advance. Watertight by construction:
/// every interior quad diagonal is shared by exactly two triangles, and after
/// `nl + nh` advances each ring has been traversed once back to its start.
fn stitch_rings(
    merged: &mut TriangleMesh,
    lo: &LatRing,
    hi: &LatRing,
    emit: &impl Fn(&mut TriangleMesh, u32, u32, u32),
) {
    if lo.len() < 2 || hi.len() < 2 {
        return;
    }
    let (nl, nh) = (lo.len(), hi.len());
    // Precompute the unwrapped (strictly increasing) longitude reached after
    // `k` forward steps on each ring, k = 0..=len. Step 0 is the ring's first
    // longitude; step len returns to it plus one full turn.
    let unwrap_ring = |ring: &LatRing| -> Vec<f64> {
        let mut acc = Vec::with_capacity(ring.len() + 1);
        let mut prev = ring[0].0;
        acc.push(prev);
        for k in 1..=ring.len() {
            let raw = ring[k % ring.len()].0;
            // Forward gap to the next vertex, in (0, 2π]: a full turn on the
            // wrap-around step (k == len), the spacing otherwise.
            let mut gap = (raw - prev).rem_euclid(std::f64::consts::TAU);
            if gap <= 0.0 {
                gap = std::f64::consts::TAU;
            }
            prev += gap;
            acc.push(prev);
        }
        acc
    };
    let lo_ang = unwrap_ring(lo);
    let hi_ang = unwrap_ring(hi);

    // Each ring is advanced exactly once around (nl + nh advances total). Once a
    // ring has completed its revolution (`i == nl` / `j == nh`) it must not
    // advance again, so its "next longitude" is treated as +inf.
    let (mut i, mut j) = (0usize, 0usize);
    for _ in 0..(nl + nh) {
        let li = lo[i % nl].1;
        let hj = hi[j % nh].1;
        let lo_next = if i < nl { lo_ang[i + 1] } else { f64::INFINITY };
        let hi_next = if j < nh { hi_ang[j + 1] } else { f64::INFINITY };
        // Advance whichever ring's next vertex comes first in longitude; the new
        // triangle's apex stays on the ring that did not advance.
        if lo_next <= hi_next {
            let li_next = lo[(i + 1) % nl].1;
            emit(merged, li, li_next, hj);
            i += 1;
        } else {
            let hj_next = hi[(j + 1) % nh].1;
            emit(merged, li, hj_next, hj);
            j += 1;
        }
    }
}

/// CDT-based tessellation for non-planar faces with exact boundary constraints.
///
/// Projects shared edge points into (u,v) parameter space, generates interior
/// sample points, then runs Constrained Delaunay Triangulation. Boundary
/// vertices use their pre-existing global IDs (watertight by construction).
#[allow(clippy::too_many_lines, clippy::too_many_arguments)]
pub(super) fn tessellate_nonplanar_cdt(
    topo: &Topology,
    face_id: FaceId,
    face_data: &brepkit_topology::face::Face,
    deflection: f64,
    angular_tol: f64,
    circle_floor: bool,
    edge_global_indices: &DetHashMap<usize, Vec<u32>>,
    merged: &mut TriangleMesh,
    point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
) -> Result<(), crate::OperationsError> {
    use brepkit_math::cdt::Cdt;
    use brepkit_math::vec::Point2;
    use brepkit_topology::edge::EdgeId;

    let wire = topo.wire(face_data.outer_wire())?;
    let tol_dup = 1e-10;

    // Fourth element: is_forward flag -- needed for seam UV assignment.
    let mut boundary_3d: Vec<(Point3, u32, EdgeId, bool)> = Vec::new();
    for oe in wire.edges() {
        let edge_id_local = oe.edge();
        let edge_idx = edge_id_local.index();
        let is_fwd = oe.is_forward();
        if let Some(global_ids) = edge_global_indices.get(&edge_idx) {
            let ordered: Vec<u32> = if is_fwd {
                global_ids.clone()
            } else {
                global_ids.iter().rev().copied().collect()
            };
            for (j, &gid) in ordered.iter().enumerate() {
                if j == 0 && !boundary_3d.is_empty() {
                    let (_, last_gid, _, _) = boundary_3d[boundary_3d.len() - 1];
                    if last_gid == gid
                        || (merged.positions[last_gid as usize] - merged.positions[gid as usize])
                            .length()
                            < tol_dup
                    {
                        continue;
                    }
                }
                boundary_3d.push((merged.positions[gid as usize], gid, edge_id_local, is_fwd));
            }
        } else {
            // Edge not in shared pool -- insert directly.
            let edge_data = topo.edge(oe.edge())?;
            let points = sample_edge(topo, edge_data, deflection, angular_tol, circle_floor)?;
            let ordered: Vec<Point3> = if is_fwd {
                points
            } else {
                points.into_iter().rev().collect()
            };
            for (j, &pt) in ordered.iter().enumerate() {
                if j == 0 && !boundary_3d.is_empty() {
                    let (last_pos, _, _, _) = boundary_3d[boundary_3d.len() - 1];
                    if (last_pos - pt).length() < tol_dup {
                        continue;
                    }
                }
                let key = point_merge_key(pt, MERGE_GRID);
                let gid = *point_to_global.entry(key).or_insert_with(|| {
                    let idx = merged.positions.len() as u32;
                    merged.positions.push(pt);
                    merged.normals.push(Vec3::new(0.0, 0.0, 0.0));
                    idx
                });
                boundary_3d.push((pt, gid, edge_id_local, is_fwd));
            }
        }
    }

    if boundary_3d.len() > 2
        && let (Some(&(_, first_gid, _, _)), Some(&(_, last_gid, _, _))) =
            (boundary_3d.first(), boundary_3d.last())
        && (first_gid == last_gid
            || (merged.positions[first_gid as usize] - merged.positions[last_gid as usize])
                .length()
                < tol_dup)
    {
        boundary_3d.pop();
    }

    let n_boundary = boundary_3d.len();
    if n_boundary < 3 {
        return Err(crate::OperationsError::InvalidInput {
            reason: "non-planar face has fewer than 3 boundary vertices".to_string(),
        });
    }

    let mut boundary_uv: Vec<(f64, f64)> = boundary_3d
        .iter()
        .map(|(pt, _, edge_id_local, _)| {
            if let Some(pcurve) = topo.pcurves().get(*edge_id_local, face_id) {
                let uv = project_via_pcurve(pcurve, *pt, face_data.surface());
                if let Some(uv) = uv {
                    return Ok(uv);
                }
            }
            project_to_surface_uv(face_data.surface(), *pt)
        })
        .collect::<Result<Vec<_>, _>>()?;

    // Step 2a: Unwrap periodic u across the seam for polyline boundaries.
    {
        let is_periodic = matches!(
            face_data.surface(),
            FaceSurface::Cylinder(_)
                | FaceSurface::Cone(_)
                | FaceSurface::Sphere(_)
                | FaceSurface::Torus(_)
        );
        if is_periodic && !boundary_uv.is_empty() {
            for i in 1..boundary_uv.len() {
                let prev_u = boundary_uv[i - 1].0;
                let mut u = boundary_uv[i].0;
                let diff = u - prev_u;
                let shifts = (diff / std::f64::consts::TAU + 0.5).floor();
                u -= shifts * std::f64::consts::TAU;
                boundary_uv[i].0 = u;
            }
            let first_u = boundary_uv[0].0;
            let last_u = boundary_uv.last().map_or(first_u, |p| p.0);
            let close_diff = first_u - last_u;
            if close_diff.abs() > std::f64::consts::PI {
                let u_mid = boundary_uv.iter().map(|p| p.0).sum::<f64>() / boundary_uv.len() as f64;
                let target_mid = std::f64::consts::PI;
                let shift = target_mid - u_mid;
                for pt in &mut boundary_uv {
                    pt.0 += shift;
                }
            }
        }

        // The tube angle (v) is periodic on a torus too. A toroidal band (a rim
        // fillet) is bounded by two rims at distinct v, joined by a seam where v
        // jumps by nearly a full turn; without unwrapping, the v-bbox spans the
        // long arc (the bulging 270° of the tube) instead of the short fillet
        // arc, and the interior CDT samples cover the wrong side. Unwrap v the
        // same way u is unwrapped so consecutive boundary points stay within
        // half a turn, collapsing the band to its true (short-arc) v-extent.
        if matches!(face_data.surface(), FaceSurface::Torus(_)) && !boundary_uv.is_empty() {
            for i in 1..boundary_uv.len() {
                let prev_v = boundary_uv[i - 1].1;
                let mut v = boundary_uv[i].1;
                let diff = v - prev_v;
                let shifts = (diff / std::f64::consts::TAU + 0.5).floor();
                v -= shifts * std::f64::consts::TAU;
                boundary_uv[i].1 = v;
            }
        }
    }

    // Compute (u,v) bounding box from a set of UV pairs.
    #[allow(clippy::items_after_statements)]
    fn uv_bounds(uvs: &[(f64, f64)]) -> (f64, f64, f64, f64) {
        uvs.iter().fold(
            (
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
            ),
            |(u_lo, u_hi, v_lo, v_hi), &(u, v)| {
                (u_lo.min(u), u_hi.max(u), v_lo.min(v), v_hi.max(v))
            },
        )
    }
    let (u_min, u_max, v_min, v_max) = uv_bounds(&boundary_uv);

    // Step 2b: Detect and fix degenerate seam edges.
    let (u_min, u_max, v_min, v_max) = {
        let mut wire_edge_counts: DetHashMap<usize, usize> = DetHashMap::default();
        for oe in wire.edges() {
            *wire_edge_counts.entry(oe.edge().index()).or_default() += 1;
        }
        let seam_edge_indices: DetHashSet<usize> = wire_edge_counts
            .iter()
            .filter(|&(_, &c)| c > 1)
            .map(|(&idx, _)| idx)
            .collect();

        if !seam_edge_indices.is_empty() {
            let non_seam_uvs: Vec<(f64, f64)> = boundary_uv
                .iter()
                .enumerate()
                .filter(|(i, _)| !seam_edge_indices.contains(&boundary_3d[*i].2.index()))
                .map(|(_, &uv)| uv)
                .collect();
            let (u_min_bnd, u_max_bnd, v_min_bnd, v_max_bnd) = if non_seam_uvs.is_empty() {
                (u_min, u_max, v_min, v_max)
            } else {
                uv_bounds(&non_seam_uvs)
            };

            #[allow(clippy::items_after_statements)]
            struct SeamRun {
                indices: Vec<usize>,
                is_forward: bool,
            }
            let mut seam_runs: Vec<SeamRun> = Vec::new();
            let mut current_indices: Vec<usize> = Vec::new();
            let mut current_fwd: Option<bool> = None;
            for i in 0..n_boundary {
                let (_, _, edge_id, is_fwd) = boundary_3d[i];
                if seam_edge_indices.contains(&edge_id.index()) {
                    current_indices.push(i);
                    if current_fwd.is_none() {
                        current_fwd = Some(is_fwd);
                    }
                } else if !current_indices.is_empty() {
                    seam_runs.push(SeamRun {
                        indices: std::mem::take(&mut current_indices),
                        is_forward: current_fwd.unwrap_or(true),
                    });
                    current_fwd = None;
                }
            }
            if !current_indices.is_empty() {
                let tail_fwd = current_fwd.unwrap_or(true);
                if !seam_runs.is_empty()
                    && seam_edge_indices.contains(&boundary_3d[0].2.index())
                    && seam_runs[0].is_forward == tail_fwd
                {
                    current_indices.extend(seam_runs.remove(0).indices);
                }
                seam_runs.push(SeamRun {
                    indices: current_indices,
                    is_forward: tail_fwd,
                });
            }

            for run in &seam_runs {
                // The periodic walk may already have placed a genuine seam
                // inside the non-seam boundary's unwrapped u interval.  Keep
                // that valid CDT boundary instead of snapping it back to an
                // endpoint; re-pin only the degenerate out-of-range case this
                // fallback was introduced to repair.
                let already_unwrapped = run.indices.iter().all(|&i| {
                    let u = boundary_uv[i].0;
                    u >= u_min_bnd - 1e-6 && u <= u_max_bnd + 1e-6
                });
                if already_unwrapped {
                    continue;
                }

                let u_assign = if run.is_forward { u_max_bnd } else { u_min_bnd };
                let n_pts = run.indices.len();

                let v_first = boundary_uv[run.indices[0]].1;
                let (v_start, v_end) = if (v_first - v_min_bnd).abs() < (v_first - v_max_bnd).abs()
                {
                    (v_min_bnd, v_max_bnd)
                } else {
                    (v_max_bnd, v_min_bnd)
                };

                for (k, &i) in run.indices.iter().enumerate() {
                    let t = if n_pts > 1 {
                        k as f64 / (n_pts - 1) as f64
                    } else {
                        0.5
                    };
                    let v = v_start + t * (v_end - v_start);
                    boundary_uv[i] = (u_assign, v);
                }
            }
        }

        // Recompute UV bounding box after seam fix.
        uv_bounds(&boundary_uv)
    };

    let du = u_max - u_min;
    let dv = v_max - v_min;

    // Triangulate in a SQUARE parameter box, not the raw (u, v) one.
    //
    // A cylindrical or conical band is an angle across and a length along, and
    // those are not the same units: a 90-degree blend wall 39.5 mm tall lands
    // in a 1.57 x 39.5 box, a 25:1 sliver. The incremental Delaunay insertion
    // loses triangles on such a box — the wall above meshes to 181.6 mm2
    // against its own exact 186.1 — so scale the long axis down to the short
    // one's length before inserting, and scale back when reading vertices out.
    // The triangulation is affine-invariant in topology, so nothing else about
    // the result changes.
    let v_scale = if du > 1e-15 && dv > 1e-15 {
        du / dv
    } else {
        1.0
    };
    let to_cdt = |u: f64, v: f64| Point2::new(u, v * v_scale);
    let from_cdt = |p: Point2| (p.x(), p.y() / v_scale);

    let margin = 0.01;
    let bounds = (
        to_cdt(u_min - margin, v_min - margin),
        to_cdt(u_max + margin, v_max + margin),
    );
    let mut cdt = Cdt::with_capacity(bounds, n_boundary);

    let mut cdt_to_global: Vec<Option<u32>> = vec![None; 3]; // 3 super-triangle verts

    let boundary_pts: Vec<Point2> = boundary_uv.iter().map(|&(u, v)| to_cdt(u, v)).collect();
    let boundary_cdt_ids = cdt
        .insert_points_hilbert(&boundary_pts)
        .map_err(crate::OperationsError::Math)?;
    let max_cdt_idx = boundary_cdt_ids.iter().copied().max().unwrap_or(2);
    if cdt_to_global.len() <= max_cdt_idx {
        cdt_to_global.resize(max_cdt_idx + 1, None);
    }
    for (i, &cdt_idx) in boundary_cdt_ids.iter().enumerate() {
        cdt_to_global[cdt_idx] = Some(boundary_3d[i].1);
    }

    for i in 0..n_boundary {
        let v0 = boundary_cdt_ids[i];
        let v1 = boundary_cdt_ids[(i + 1) % n_boundary];
        cdt.insert_constraint(v0, v1)
            .map_err(crate::OperationsError::Math)?;
    }

    if du > 1e-15 && dv > 1e-15 {
        let (n_u, n_v) =
            interior_grid_resolution(face_data.surface(), du, dv, deflection, angular_tol);

        let boundary_uv_ref = &boundary_uv;
        let interior_pts: Vec<Point2> = (1..n_u)
            .flat_map(|iu| {
                (1..n_v).filter_map(move |iv| {
                    let u = u_min + du * (iu as f64 / n_u as f64);
                    let v = v_min + dv * (iv as f64 / n_v as f64);
                    point_in_polygon_2d(boundary_uv_ref, Point2::new(u, v)).then(|| to_cdt(u, v))
                })
            })
            .collect();
        if !interior_pts.is_empty() {
            let interior_cdt_ids = cdt
                .insert_points_hilbert(&interior_pts)
                .map_err(crate::OperationsError::Math)?;
            let max_interior = interior_cdt_ids.iter().copied().max().unwrap_or(0);
            if cdt_to_global.len() <= max_interior {
                cdt_to_global.resize(max_interior + 1, None);
            }
        }
    }

    let boundary_pairs: Vec<(usize, usize)> = (0..n_boundary)
        .map(|i| (boundary_cdt_ids[i], boundary_cdt_ids[(i + 1) % n_boundary]))
        .collect();
    cdt.remove_exterior(&boundary_pairs);

    let cdt_verts = cdt.vertices();
    let triangles = cdt.triangles();

    let mut final_global_ids: Vec<u32> = vec![0; cdt_to_global.len()];

    for i in 0..cdt_to_global.len() {
        if let Some(gid) = cdt_to_global[i] {
            final_global_ids[i] = gid;
        } else if i >= 3 {
            let (pu, pv) = from_cdt(cdt_verts[i]);
            let surface = face_data.surface();
            let pt3 = eval_surface_point(surface, pu, pv);
            let nrm = surface.normal(pu, pv);

            let key = point_merge_key(pt3, MERGE_GRID);
            let gid = *point_to_global.entry(key).or_insert_with(|| {
                let idx = merged.positions.len() as u32;
                merged.positions.push(pt3);
                merged.normals.push(nrm);
                idx
            });
            final_global_ids[i] = gid;
        }
    }

    for (i0, i1, i2) in triangles {
        if i0 < 3 || i1 < 3 || i2 < 3 {
            continue; // Skip super-triangle vertices
        }
        merged.indices.push(final_global_ids[i0]);
        merged.indices.push(final_global_ids[i1]);
        merged.indices.push(final_global_ids[i2]);
    }

    Ok(())
}

/// Project a 3D point onto a face surface, returning (u, v) parameters.
fn project_to_surface_uv(
    surface: &FaceSurface,
    pt: Point3,
) -> Result<(f64, f64), crate::OperationsError> {
    match surface {
        FaceSurface::Cylinder(cyl) => Ok(cyl.project_point(pt)),
        FaceSurface::Cone(cone) => Ok(cone.project_point(pt)),
        FaceSurface::Sphere(sphere) => Ok(sphere.project_point(pt)),
        FaceSurface::Torus(torus) => Ok(torus.project_point(pt)),
        FaceSurface::Nurbs(surface) => {
            brepkit_math::nurbs::projection::project_point_to_surface(surface, pt, 1e-6)
                .map(|proj| (proj.u, proj.v))
                .map_err(crate::OperationsError::Math)
        }
        FaceSurface::Plane { .. } => Err(crate::OperationsError::InvalidInput {
            reason: "planar faces should not use CDT tessellation".to_string(),
        }),
    }
}

/// Try to find (u,v) coordinates for a 3D point using a PCurve.
fn project_via_pcurve(
    pcurve: &brepkit_topology::pcurve::PCurve,
    pt: Point3,
    surface: &FaceSurface,
) -> Option<(f64, f64)> {
    let t_start = pcurve.t_start();
    let t_end = pcurve.t_end();
    let n_samples = 16;

    let mut best_t = t_start;
    let mut best_dist = f64::MAX;

    for i in 0..=n_samples {
        let t = t_start + (t_end - t_start) * (i as f64) / (n_samples as f64);
        let uv = pcurve.evaluate(t);
        let p_surf = eval_surface_point(surface, uv.x(), uv.y());
        let d = (p_surf - pt).length();
        if d < best_dist {
            best_dist = d;
            best_t = t;
        }
    }

    // Refine with bisection around best_t.
    let dt = (t_end - t_start) / (n_samples as f64);
    let mut lo = (best_t - dt).max(t_start);
    let mut hi = (best_t + dt).min(t_end);
    for _ in 0..10 {
        let mid = 0.5 * (lo + hi);
        let uv_lo = pcurve.evaluate(lo);
        let uv_hi = pcurve.evaluate(hi);
        let d_lo = (eval_surface_point(surface, uv_lo.x(), uv_lo.y()) - pt).length();
        let d_hi = (eval_surface_point(surface, uv_hi.x(), uv_hi.y()) - pt).length();
        if d_lo < d_hi {
            hi = mid;
        } else {
            lo = mid;
        }
    }

    let t_final = 0.5 * (lo + hi);
    let uv = pcurve.evaluate(t_final);
    let p_final = eval_surface_point(surface, uv.x(), uv.y());

    if (p_final - pt).length() < brepkit_math::tolerance::Tolerance::default().linear {
        Some((uv.x(), uv.y()))
    } else {
        None
    }
}

/// Evaluate a non-planar surface at `(u, v)` and return a 3D point.
fn eval_surface_point(surface: &FaceSurface, u: f64, v: f64) -> Point3 {
    surface.evaluate(u, v).unwrap_or(Point3::new(0.0, 0.0, 0.0))
}

/// Estimate the effective radius of a surface for sample density calculation.
fn estimate_surface_radius(surface: &FaceSurface) -> f64 {
    match surface {
        FaceSurface::Cylinder(cyl) => cyl.radius(),
        FaceSurface::Cone(_) => 1.0,
        FaceSurface::Sphere(sphere) => sphere.radius(),
        FaceSurface::Torus(torus) => torus.major_radius() + torus.minor_radius(),
        FaceSurface::Nurbs(_) | FaceSurface::Plane { .. } => 1.0,
    }
}

/// Compute interior grid resolution for `tessellate_nonplanar_cdt`.
fn interior_grid_resolution(
    surface: &FaceSurface,
    du: f64,
    dv: f64,
    deflection: f64,
    angular_tol: f64,
) -> (usize, usize) {
    // This is the non-standard-boundary CDT fallback (boolean-result faces).
    // Watertightness comes from the explicit boundary samples, not from these
    // interior grid counts, and one radius drives both directions; keep the
    // curvature floor on for all surfaces here as the conservative default.
    match surface {
        FaceSurface::Sphere(sphere) => {
            let r = sphere.radius();
            let n_u = segments_for_chord_deviation_a(r, du, deflection, angular_tol, true).max(2);
            let n_v = segments_for_chord_deviation_a(r, dv, deflection, angular_tol, true).max(2);
            (n_u, n_v)
        }
        FaceSurface::Torus(torus) => {
            let n_u = segments_for_chord_deviation_a(
                torus.major_radius(),
                du,
                deflection,
                angular_tol,
                true,
            )
            .max(2);
            let n_v = segments_for_chord_deviation_a(
                torus.minor_radius(),
                dv,
                deflection,
                angular_tol,
                true,
            )
            .max(2);
            (n_u, n_v)
        }
        FaceSurface::Cylinder(_) | FaceSurface::Cone(_) => {
            // u is the periodic direction (radians): curvature-driven. v runs
            // along the straight rulings (a length, not an angle): zero chord
            // sag, so feeding it to the chord formula would treat millimeters
            // as radians and emit hundreds of interior rows on a tall wall.
            // Two rows suffice for CDT quality on a developable band.
            let r = estimate_surface_radius(surface);
            let n_u = segments_for_chord_deviation_a(r, du, deflection, angular_tol, true).max(2);
            (n_u, 2)
        }
        FaceSurface::Plane { .. } | FaceSurface::Nurbs(_) => {
            let r = estimate_surface_radius(surface);
            let n_u = segments_for_chord_deviation_a(r, du, deflection, angular_tol, true).max(2);
            let n_v = segments_for_chord_deviation_a(r, dv, deflection, angular_tol, true).max(2);
            (n_u, n_v)
        }
    }
}

/// Fill a spherical polar cap from a shared constant-latitude boundary.
///
/// A primitive sphere's hemispheres meet on one faceted equatorial wire. After
/// a box removes part of only one hemisphere, the retained band is tessellated
/// from that shared wire while the untouched hemisphere used to fall back to an
/// independent analytic grid. Its different longitude count cracked the
/// equator. This path keeps the boundary IDs verbatim, inserts latitude rows for
/// the requested deflection, and closes them with one pole vertex.
///
/// Returns false without emitting anything unless boundary is a
/// full-revolution loop at one constant sphere latitude.
#[allow(clippy::too_many_lines)]
fn fill_sphere_latitude_cap(
    sphere: &brepkit_math::surfaces::SphericalSurface,
    boundary: &[u32],
    deflection: f64,
    angular_tol: f64,
    merged: &mut TriangleMesh,
    point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
) -> bool {
    let n = boundary.len();
    let radius = sphere.radius();
    if n < 3 || radius <= 0.0 {
        return false;
    }

    let mut seen: DetHashSet<u32> = DetHashSet::default();
    let mut ring: LatRing = Vec::with_capacity(n);
    let mut v_sum = 0.0;
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;
    let mut winding = 0.0;

    for i in 0..n {
        let gid = boundary[i];
        let next_gid = boundary[(i + 1) % n];
        let p = merged.positions[gid as usize];
        let next = merged.positions[next_gid as usize];
        let radial = p - sphere.center();
        if (radial.length() - radius).abs() > radius * 1e-6 {
            return false;
        }
        winding += radial.cross(next - sphere.center()).dot(sphere.z_axis());

        if seen.insert(gid) {
            let (u, v) = sphere.project_point(p);
            v_sum += v;
            v_min = v_min.min(v);
            v_max = v_max.max(v);
            ring.push((u, gid));
        }
    }
    if ring.len() < 3 || v_max - v_min > 1e-6 {
        return false;
    }
    ring.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    let max_gap = ring
        .windows(2)
        .map(|w| w[1].0 - w[0].0)
        .chain(std::iter::once(
            ring[0].0 + std::f64::consts::TAU - ring[ring.len() - 1].0,
        ))
        .fold(0.0_f64, f64::max);
    if max_gap > std::f64::consts::PI {
        return false;
    }

    let winding_tol = radius * radius * 1e-10;
    let pole_v = if winding > winding_tol {
        std::f64::consts::FRAC_PI_2
    } else if winding < -winding_tol {
        -std::f64::consts::FRAC_PI_2
    } else {
        return false;
    };
    let boundary_v = v_sum / ring.len() as f64;
    let v_span = (pole_v - boundary_v).abs();
    if v_span < 1e-9 {
        return false;
    }

    let n_v = segments_for_chord_deviation_a(radius, v_span, deflection, angular_tol, true).max(1);
    let n_u = ring.len().max(segments_for_chord_deviation_a(
        radius,
        std::f64::consts::TAU,
        deflection,
        angular_tol,
        true,
    ));
    let surf_eval = |u, v| sphere.evaluate(u, v);
    let surf_normal = |u, v| sphere.normal(u, v);
    let project = |p| sphere.project_point(p);
    let emit = make_band_emit(&project, &surf_normal);

    let mut prev = ring;
    for iv in 1..n_v {
        let t = iv as f64 / n_v as f64;
        let v = boundary_v + (pole_v - boundary_v) * t;
        let row = build_interior_row(v, n_u, &surf_eval, &surf_normal, merged, point_to_global);
        stitch_rings(merged, &prev, &row, &emit);
        prev = row;
    }

    let pole = sphere.evaluate(0.0, pole_v);
    let pole_key = point_merge_key(pole, MERGE_GRID);
    let pole_gid = *point_to_global.entry(pole_key).or_insert_with(|| {
        let idx = merged.positions.len() as u32;
        merged.positions.push(pole);
        merged.normals.push(sphere.normal(0.0, pole_v));
        idx
    });
    for i in 0..prev.len() {
        emit(merged, prev[i].1, prev[(i + 1) % prev.len()].1, pole_gid);
    }
    true
}

/// Fill a spherical cap as a structured "web": concentric rings slerped from
/// the boundary loop toward the patch's spherical centroid, closed by a fan to
/// the centroid vertex.
///
/// `boundary` holds the cap's rim as ordered global vertex ids (an open list —
/// no closing duplicate) whose positions already sit on the sphere of the
/// given `center`/`radius`. Every generated vertex is evaluated exactly on the
/// sphere, so no parameterization (and none of the UV pitfalls of the CDT
/// path) is involved; triangles wind against the outward radial.
///
/// Returns `false` without emitting anything when the cap is unsuitable: rim
/// off the sphere, a degenerate centroid, or a rim spreading beyond ~80° from
/// its centroid (the slerp toward the centroid is no longer a bijection there).
fn fill_sphere_cap_web(
    center: Point3,
    radius: f64,
    boundary: &[u32],
    deflection: f64,
    angular_tol: f64,
    merged: &mut TriangleMesh,
    point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
) -> bool {
    let n = boundary.len();
    if n < 3 || radius <= 0.0 {
        return false;
    }

    let mut dirs: Vec<Vec3> = Vec::with_capacity(n);
    for &gid in boundary {
        let d = merged.positions[gid as usize] - center;
        if (d.length() - radius).abs() > radius * 1e-6 {
            return false;
        }
        let Ok(unit) = d.normalize() else {
            return false;
        };
        dirs.push(unit);
    }

    let mut centroid_raw = Vec3::new(0.0, 0.0, 0.0);
    for d in &dirs {
        centroid_raw += *d;
    }
    let Ok(centroid) = centroid_raw.normalize() else {
        return false;
    };

    // Angle from the centroid per rim vertex; keep the cap comfortably inside
    // a hemisphere so every slerp is well-defined and injective.
    let mut theta_max = 0.0_f64;
    let mut thetas = Vec::with_capacity(n);
    for d in &dirs {
        let cos = d.dot(centroid).clamp(-1.0, 1.0);
        if cos < 0.17 {
            return false; // beyond ~80 degrees
        }
        let theta = cos.acos();
        theta_max = theta_max.max(theta);
        thetas.push(theta);
    }

    // The web fills by slerping every rim vertex toward the centroid, which
    // stays inside the patch only when the rim is star-shaped about the
    // centroid: each rim edge's great-circle plane must keep the centroid on
    // a consistent side. Reject rims that fail this (a concave boolean
    // fragment) so they fall back to the CDT.
    let mut orientation = 0_i8;
    for i in 0..n {
        let j = (i + 1) % n;
        let edge_normal = dirs[i].cross(dirs[j]);
        let len = edge_normal.length();
        if len < 1e-14 {
            continue; // duplicate or antipodal samples: no constraint
        }
        let side = edge_normal.dot(centroid) / len;
        if side.abs() < 1e-9 {
            continue; // centroid on the edge's great circle: degenerate rim
        }
        let sign: i8 = if side > 0.0 { 1 } else { -1 };
        if orientation == 0 {
            orientation = sign;
        } else if sign != orientation {
            return false;
        }
    }
    if orientation == 0 {
        return false;
    }

    let rings =
        segments_for_chord_deviation_a(radius, theta_max, deflection, angular_tol, true).max(1);

    let emit = |merged: &mut TriangleMesh, a: u32, b: u32, c: u32| {
        if a == b || b == c || a == c {
            return;
        }
        let (pa, pb, pc) = (
            merged.positions[a as usize],
            merged.positions[b as usize],
            merged.positions[c as usize],
        );
        let geo = (pb - pa).cross(pc - pa);
        if geo.length() < 1e-20 {
            return;
        }
        let mid = Point3::new(
            (pa.x() + pb.x() + pc.x()) / 3.0,
            (pa.y() + pb.y() + pc.y()) / 3.0,
            (pa.z() + pb.z() + pc.z()) / 3.0,
        );
        let mut tri = [a, b, c];
        if geo.dot(mid - center) < 0.0 {
            tri.swap(1, 2);
        }
        merged.indices.extend_from_slice(&tri);
    };

    let intern = |merged: &mut TriangleMesh,
                  point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
                  dir: Vec3| {
        let pt = center + dir * radius;
        let key = point_merge_key(pt, MERGE_GRID);
        *point_to_global.entry(key).or_insert_with(|| {
            let idx = merged.positions.len() as u32;
            merged.positions.push(pt);
            merged.normals.push(dir);
            idx
        })
    };

    let mut prev: Vec<u32> = boundary.to_vec();
    for k in 1..rings {
        let t = k as f64 / rings as f64;
        let mut ring = Vec::with_capacity(n);
        for (d, &theta) in dirs.iter().zip(&thetas) {
            let dir = if theta < 1e-12 {
                centroid
            } else {
                let s = theta.sin();
                let blended =
                    *d * (((1.0 - t) * theta).sin() / s) + centroid * ((t * theta).sin() / s);
                blended.normalize().unwrap_or(centroid)
            };
            ring.push(intern(merged, point_to_global, dir));
        }
        for i in 0..n {
            let j = (i + 1) % n;
            emit(merged, prev[i], prev[j], ring[j]);
            emit(merged, prev[i], ring[j], ring[i]);
        }
        prev = ring;
    }

    let apex = intern(merged, point_to_global, centroid);
    for i in 0..n {
        let j = (i + 1) % n;
        emit(merged, prev[i], prev[j], apex);
    }
    true
}

/// Collect a face's outer-wire boundary as an ordered open loop of global
/// vertex ids, drawing samples from the shared edge pool when available and
/// sampling the edge geometry directly otherwise. Consecutive duplicates and
/// the closing duplicate are removed.
fn collect_boundary_loop(
    topo: &Topology,
    face_data: &brepkit_topology::face::Face,
    deflection: f64,
    angular_tol: f64,
    edge_global_indices: &DetHashMap<usize, Vec<u32>>,
    merged: &mut TriangleMesh,
    point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
) -> Result<Vec<u32>, crate::OperationsError> {
    let wire = topo.wire(face_data.outer_wire())?;
    let tol_dup = 1e-10;
    let mut boundary: Vec<u32> = Vec::new();

    let push_gid = |boundary: &mut Vec<u32>, merged: &TriangleMesh, gid: u32| {
        if let Some(&last) = boundary.last()
            && (last == gid
                || (merged.positions[last as usize] - merged.positions[gid as usize]).length()
                    < tol_dup)
        {
            return;
        }
        boundary.push(gid);
    };

    for oe in wire.edges() {
        let is_fwd = oe.is_forward();
        if let Some(global_ids) = edge_global_indices.get(&oe.edge().index()) {
            let ordered: Vec<u32> = if is_fwd {
                global_ids.clone()
            } else {
                global_ids.iter().rev().copied().collect()
            };
            for gid in ordered {
                push_gid(&mut boundary, merged, gid);
            }
        } else {
            let edge_data = topo.edge(oe.edge())?;
            let points = sample_edge(topo, edge_data, deflection, angular_tol, false)?;
            let ordered: Vec<Point3> = if is_fwd {
                points
            } else {
                points.into_iter().rev().collect()
            };
            for pt in ordered {
                let key = point_merge_key(pt, MERGE_GRID);
                let gid = *point_to_global.entry(key).or_insert_with(|| {
                    let idx = merged.positions.len() as u32;
                    merged.positions.push(pt);
                    merged.normals.push(Vec3::new(0.0, 0.0, 0.0));
                    idx
                });
                push_gid(&mut boundary, merged, gid);
            }
        }
    }

    if boundary.len() > 2
        && let (Some(&first), Some(&last)) = (boundary.first(), boundary.last())
        && (first == last
            || (merged.positions[first as usize] - merged.positions[last as usize]).length()
                < tol_dup)
    {
        boundary.pop();
    }
    Ok(boundary)
}

/// Tessellate a spherical face with no inner wires from shared boundary samples.
///
/// Constant-latitude polar caps use structured latitude rows. Other suitable
/// caps (for example, the corner ball patch a fillet leaves at a box corner)
/// use a structured web. The CDT path degrades on these caps: their boundary
/// arcs project to (near-)collinear UV polylines, and collinear constraint
/// chains drive the planar CDT into zero-UV-area triangles that carry real 3D
/// area (rendered as flaps across the cap) and, at unlucky deflections, cracks.
/// Both structured paths reuse the shared edge-pool vertices verbatim, so seams
/// stay watertight by construction. The latitude-cap path is enabled only when
/// solid-level adjacency finds a trimmed face on the same sphere. It remains
/// disabled for primitive/standalone sphere tessellation and the mesh-boolean
/// fallback, preserving their triangle semantics and boolean acceptance.
///
/// Returns `Ok(true)` when the face is such a cap and was tessellated here;
/// `Ok(false)` defers to the CDT/snap path.
#[allow(clippy::too_many_arguments)]
pub(super) fn tessellate_sphere_cap_shared(
    topo: &Topology,
    face_data: &brepkit_topology::face::Face,
    deflection: f64,
    angular_tol: f64,
    allow_latitude_cap: bool,
    edge_global_indices: &DetHashMap<usize, Vec<u32>>,
    merged: &mut TriangleMesh,
    point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
) -> Result<bool, crate::OperationsError> {
    let FaceSurface::Sphere(sphere) = face_data.surface() else {
        return Ok(false);
    };
    if !face_data.inner_wires().is_empty() {
        return Ok(false);
    }

    let pos_save = merged.positions.len();
    let idx_save = merged.indices.len();
    let boundary = collect_boundary_loop(
        topo,
        face_data,
        deflection,
        angular_tol,
        edge_global_indices,
        merged,
        point_to_global,
    )?;

    if (allow_latitude_cap
        && fill_sphere_latitude_cap(
            sphere,
            &boundary,
            deflection,
            angular_tol,
            merged,
            point_to_global,
        ))
        || fill_sphere_cap_web(
            sphere.center(),
            sphere.radius(),
            &boundary,
            deflection,
            angular_tol,
            merged,
            point_to_global,
        )
    {
        Ok(true)
    } else {
        // Roll back any boundary vertices this attempt interned so the CDT
        // path starts from the same state it would have seen.
        merged.positions.truncate(pos_save);
        merged.normals.truncate(pos_save);
        merged.indices.truncate(idx_save);
        point_to_global.retain(|_, v| (*v as usize) < pos_save);
        Ok(false)
    }
}

/// Tessellate a trimmed sphere face standalone (single-face path), returning
/// per-vertex UVs.
///
/// The rectangular UV sweep in `tessellate_with_uvs_a` covers the boundary's
/// UV bounding box, which over-covers any sphere face whose boundary is not
/// iso-parametric — e.g. the spherical vertex-blend cap a fillet leaves at a
/// box corner (three arcs, none at constant `u` or `v`). This path samples the
/// wire itself and fills the cap with the structured web (falling back to the
/// boundary-constrained CDT for boundaries the web declines), then rebuilds
/// every vertex normal exactly from the surface.
///
/// Returns `None` when neither path produces triangles; the caller falls back
/// to the rectangular sweep so behavior never degrades.
pub(super) fn tessellate_trimmed_sphere_uvs(
    topo: &Topology,
    face_id: FaceId,
    face_data: &brepkit_topology::face::Face,
    sphere: &brepkit_math::surfaces::SphericalSurface,
    deflection: f64,
    angular_tol: f64,
    try_cdt: bool,
) -> Option<super::TriangleMeshUV> {
    let empty_pool: DetHashMap<usize, Vec<u32>> = DetHashMap::default();
    let mut merged = TriangleMesh::default();
    let mut point_to_global: DetHashMap<(i64, i64, i64), u32> = DetHashMap::default();

    let web_ok = tessellate_sphere_cap_shared(
        topo,
        face_data,
        deflection,
        angular_tol,
        false,
        &empty_pool,
        &mut merged,
        &mut point_to_global,
    )
    .unwrap_or(false);

    if !web_ok {
        if !try_cdt {
            return None;
        }
        let result = tessellate_nonplanar_cdt(
            topo,
            face_id,
            face_data,
            deflection,
            angular_tol,
            false,
            &empty_pool,
            &mut merged,
            &mut point_to_global,
        );
        if result.is_err() {
            return None;
        }
    }
    if merged.indices.is_empty() {
        return None;
    }

    let mut uvs = Vec::with_capacity(merged.positions.len());
    for i in 0..merged.positions.len() {
        let (u, v) = sphere.project_point(merged.positions[i]);
        uvs.push([u, v]);
        merged.normals[i] = sphere.normal(u, v);
    }
    Some(super::TriangleMeshUV { mesh: merged, uvs })
}

/// Check if a 2D point is inside a polygon defined by (u, v) coordinates.
/// Uses the winding number algorithm for robustness.
pub(super) fn point_in_polygon_2d(polygon: &[(f64, f64)], pt: brepkit_math::vec::Point2) -> bool {
    let n = polygon.len();
    let mut winding = 0i32;
    for i in 0..n {
        let j = (i + 1) % n;
        let yi = polygon[i].1;
        let yj = polygon[j].1;
        if yi <= pt.y() {
            if yj > pt.y() {
                let cross = (polygon[j].0 - polygon[i].0) * (pt.y() - yi)
                    - (pt.x() - polygon[i].0) * (yj - yi);
                if cross > 0.0 {
                    winding += 1;
                }
            }
        } else if yj <= pt.y() {
            let cross =
                (polygon[j].0 - polygon[i].0) * (pt.y() - yi) - (pt.x() - polygon[i].0) * (yj - yi);
            if cross < 0.0 {
                winding -= 1;
            }
        }
    }
    winding != 0
}

/// Snap-based fallback tessellation for non-planar faces.
#[allow(clippy::too_many_arguments)]
pub(super) fn tessellate_nonplanar_snap(
    topo: &Topology,
    face_id: FaceId,
    face_data: &brepkit_topology::face::Face,
    deflection: f64,
    angular_tol: f64,
    circle_floor: bool,
    edge_global_indices: &DetHashMap<usize, Vec<u32>>,
    merged: &mut TriangleMesh,
    point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
) -> Result<(), crate::OperationsError> {
    let mut face_mesh = super::face::tessellate_with_uvs_floor(
        topo,
        face_id,
        deflection,
        angular_tol,
        circle_floor,
    )
    .map(|uv| uv.mesh)?;

    // `tessellate()` already applies the `is_reversed` flip. The caller
    // `tessellate_face_with_shared_edges` will apply its own flip, so undo
    // the one from `tessellate()` to avoid a double-flip.
    if face_data.is_reversed() {
        let tri_count = face_mesh.indices.len() / 3;
        for t in 0..tri_count {
            face_mesh.indices.swap(t * 3 + 1, t * 3 + 2);
        }
        for n in &mut face_mesh.normals {
            *n = -*n;
        }
    }

    let mut local_to_global: Vec<u32> = Vec::with_capacity(face_mesh.positions.len());

    let wire = topo.wire(face_data.outer_wire())?;
    let mut snap_targets: Vec<(Point3, u32)> = Vec::new();
    for oe in wire.edges() {
        if let Some(global_ids) = edge_global_indices.get(&oe.edge().index()) {
            for &gid in global_ids {
                if (gid as usize) < merged.positions.len() {
                    snap_targets.push((merged.positions[gid as usize], gid));
                }
            }
        }
    }
    for &inner_wire_id in face_data.inner_wires() {
        if let Ok(inner_wire) = topo.wire(inner_wire_id) {
            for oe in inner_wire.edges() {
                if let Some(global_ids) = edge_global_indices.get(&oe.edge().index()) {
                    for &gid in global_ids {
                        if (gid as usize) < merged.positions.len() {
                            snap_targets.push((merged.positions[gid as usize], gid));
                        }
                    }
                }
            }
        }
    }

    // Build spatial hash for O(1) snap lookups.
    let snap_tol = 1e-6;
    let inv_cell = 1.0 / snap_tol;
    let mut snap_grid: DetHashMap<(i64, i64, i64), Vec<u32>> =
        DetHashMap::with_capacity_and_hasher(snap_targets.len(), brepkit_math::det_hash::DetState);
    for &(target_pos, gid) in &snap_targets {
        let cx = (target_pos.x() * inv_cell).round() as i64;
        let cy = (target_pos.y() * inv_cell).round() as i64;
        let cz = (target_pos.z() * inv_cell).round() as i64;
        snap_grid.entry((cx, cy, cz)).or_default().push(gid);
    }

    for (i, &pos) in face_mesh.positions.iter().enumerate() {
        let cx = (pos.x() * inv_cell).round() as i64;
        let cy = (pos.y() * inv_cell).round() as i64;
        let cz = (pos.z() * inv_cell).round() as i64;
        let mut best_gid = None;
        let mut best_dist = snap_tol;
        // Check 3x3x3 neighborhood for snap matches.
        for dx in -1_i64..=1 {
            for dy in -1_i64..=1 {
                for dz in -1_i64..=1 {
                    if let Some(gids) = snap_grid.get(&(cx + dx, cy + dy, cz + dz)) {
                        for &gid in gids {
                            let target_pos = merged.positions[gid as usize];
                            let dist = (pos - target_pos).length();
                            if dist < best_dist {
                                best_dist = dist;
                                best_gid = Some(gid);
                            }
                        }
                    }
                }
            }
        }

        if let Some(gid) = best_gid {
            local_to_global.push(gid);
        } else {
            let key = point_merge_key(pos, MERGE_GRID);
            let gid = point_to_global.entry(key).or_insert_with(|| {
                let idx = merged.positions.len() as u32;
                merged.positions.push(pos);
                merged.normals.push(
                    face_mesh
                        .normals
                        .get(i)
                        .copied()
                        .unwrap_or(Vec3::new(0.0, 0.0, 1.0)),
                );
                idx
            });
            local_to_global.push(*gid);
        }
    }

    for &li in &face_mesh.indices {
        merged.indices.push(local_to_global[li as usize]);
    }

    Ok(())
}

#[cfg(test)]
mod rim_angle_tests {
    use super::rim_angles_match;
    use std::f64::consts::TAU;

    #[test]
    fn accepts_matching_angles_across_surface_seam() {
        let a = [0.0, 1.0, TAU - 5e-7];
        let b = [TAU, 1.0 + 5e-7, 0.0];
        assert!(rim_angles_match(&a, &b, 1e-6));
    }

    #[test]
    fn rejects_equal_count_phase_mismatch() {
        let a = [0.0, 1.0, 2.0];
        let b = [0.25, 1.25, 2.25];
        assert!(!rim_angles_match(&a, &b, 1e-6));
    }
}

#[cfg(test)]
mod torus_winding_tests {
    use super::has_single_period_winding;

    #[test]
    fn partial_torus_rim_is_not_a_full_period_winding() {
        let partial_rim = [0.0, 1.0, 2.0, 3.0, 3.8, 3.0, 2.0, 1.0, 0.0];
        assert!(!has_single_period_winding(&partial_rim));
    }

    #[test]
    fn complete_torus_rim_has_a_single_period_winding() {
        let full_rim = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0];
        assert!(has_single_period_winding(&full_rim));
    }
}
