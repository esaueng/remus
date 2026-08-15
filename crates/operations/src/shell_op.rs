//! Shell (hollow/offset) operation for creating thin-walled solids.
//!
//! Offsets faces of a solid inward to create a hollow shell with
//! uniform wall thickness. Optionally removes specified faces to
//! create openings.
//!
//! The outer surface of the result is the input's own surface, minus the
//! faces opened. It is therefore carried through verbatim rather than
//! rebuilt: exact surfaces, exact curved edges, orientation, and every inner
//! wire. A body with a bore keeps its bore.
//!
//! The inner surface is genuinely new geometry — each kept face displaced one
//! wall thickness along the negative of its OUTWARD normal — so it is built
//! from positions. A hole in a face is displaced with it, by the same miter
//! vectors that place the face's own corners, which is why a bore's mouth in
//! the inner cap lands exactly on the rim of the offset bore wall.
//!
//! "Inward" is signed by the face, not by the surface: a bore's wall faces
//! its own axis, so offsetting into the metal WIDENS it, where the same
//! offset on a boss narrows it.

use std::collections::{HashMap, HashSet};

use brepkit_math::tolerance::Tolerance;
use brepkit_math::vec::{Point3, Vec3};
use brepkit_topology::Topology;
use brepkit_topology::face::{FaceId, FaceSurface};
use brepkit_topology::solid::SolidId;
use brepkit_topology::wire::OrientedEdge;

use crate::boolean::{FaceSpec, assemble_solid_mixed};
use crate::dot_normal_point;

/// Operation name carried by [`crate::OperationsError::Unsupported`] refusals.
const OP: &str = "shell";

/// Deflection for the closing volume check. `solid_volume` integrates the
/// analytic surfaces a shell is made of in closed form, so this only bounds
/// the fallback paths.
const VOLUME_DEFLECTION: f64 = 0.01;

fn unsupported(reason: impl Into<String>) -> crate::OperationsError {
    crate::OperationsError::Unsupported {
        operation: OP,
        reason: reason.into(),
    }
}

/// Compute the inner vertex position using miter-vector offset.
///
/// Given a vertex with normals from adjacent faces, solves for the offset
/// direction that satisfies `m · n_i = 1` for all non-open face normals
/// (open face normals contribute 0). The inner position is:
///   `inner = outer - thickness * m`
///
/// For 3 linearly independent normals, this is equivalent to 3-plane
/// intersection. For 2 normals, it produces the least-norm miter (the
/// shortest offset vector satisfying both constraints). For 1 normal,
/// it offsets along that normal.
fn compute_miter_offset(outer: Point3, unique_normals: &[(Vec3, bool)], thickness: f64) -> Point3 {
    // Build system: for each unique normal, m · n_i = weight_i
    // where weight_i = 1.0 for non-open faces, 0.0 for open faces.
    let mut normals: Vec<Vec3> = Vec::new();
    let mut weights: Vec<f64> = Vec::new();

    for &(n, is_open) in unique_normals {
        normals.push(n);
        weights.push(if is_open { 0.0 } else { 1.0 });
    }

    let miter = match normals.len() {
        0 => return outer,
        1 => {
            // Single normal: offset along it.
            normals[0] * weights[0]
        }
        2 => {
            // Two normals: least-norm solution of [n1; n2] · m = [w1; w2].
            // m = N^T (N N^T)^{-1} w
            let n1 = normals[0];
            let n2 = normals[1];
            let w1 = weights[0];
            let w2 = weights[1];

            let g11 = n1.dot(n1);
            let g12 = n1.dot(n2);
            let g22 = n2.dot(n2);
            let det = g11 * g22 - g12 * g12;

            if det.abs() < 1e-12 {
                // Nearly parallel normals: just use the first non-open one.
                if w1 > 0.5 { n1 * w1 } else { n2 * w2 }
            } else {
                let inv_det = 1.0 / det;
                let a1 = (g22 * w1 - g12 * w2) * inv_det;
                let a2 = (-g12 * w1 + g11 * w2) * inv_det;
                n1 * a1 + n2 * a2
            }
        }
        _ => {
            // Three or more normals: use the first 3 linearly independent
            // normals and solve via Cramer's rule (3-plane intersection).
            let n1 = normals[0];
            let n2 = normals[1];
            let n3 = normals[2];
            let w1 = weights[0];
            let w2 = weights[1];
            let w3 = weights[2];

            let n2_cross_n3 = n2.cross(n3);
            let det = n1.dot(n2_cross_n3);

            if det.abs() < 1e-12 {
                // Degenerate: fall back to 2-normal solution with first two.
                let g11 = n1.dot(n1);
                let g12 = n1.dot(n2);
                let g22 = n2.dot(n2);
                let d2 = g11 * g22 - g12 * g12;
                if d2.abs() < 1e-12 {
                    n1 * w1
                } else {
                    let inv = 1.0 / d2;
                    let a1 = (g22 * w1 - g12 * w2) * inv;
                    let a2 = (-g12 * w1 + g11 * w2) * inv;
                    n1 * a1 + n2 * a2
                }
            } else {
                let n3_cross_n1 = n3.cross(n1);
                let n1_cross_n2 = n1.cross(n2);
                let inv_det = 1.0 / det;
                let mx =
                    (w1 * n2_cross_n3.x() + w2 * n3_cross_n1.x() + w3 * n1_cross_n2.x()) * inv_det;
                let my =
                    (w1 * n2_cross_n3.y() + w2 * n3_cross_n1.y() + w3 * n1_cross_n2.y()) * inv_det;
                let mz =
                    (w1 * n2_cross_n3.z() + w2 * n3_cross_n1.z() + w3 * n1_cross_n2.z()) * inv_det;
                Vec3::new(mx, my, mz)
            }
        }
    };

    Point3::new(
        outer.x() - thickness * miter.x(),
        outer.y() - thickness * miter.y(),
        outer.z() - thickness * miter.z(),
    )
}

/// Create a hollow shell from a solid by offsetting faces inward.
///
/// Each face is offset inward by `thickness` along its OUTWARD normal, which
/// is not its surface's normal when the face is reversed: hollowing widens a
/// bore and narrows a boss. Supports planar, NURBS, and analytic surface
/// faces, and carries every face's inner wires onto both skins, so a body
/// with a bore comes back with the bore.
///
/// If `open_faces` is non-empty, those faces are removed from both the outer
/// and inner shells, and the wall they expose is rimmed with one annular face
/// per nested pair of boundary loops in the opened face's plane.
///
/// The result is closed: every edge is shared by exactly two face uses. A
/// result that is not is refused rather than returned.
///
/// # Errors
///
/// Returns [`crate::OperationsError::InvalidInput`] if `thickness` is
/// non-positive, if a face in `open_faces` is not part of the solid, or if
/// the offset would collapse a sphere through its own centre.
///
/// Returns [`crate::OperationsError::Unsupported`] when the hollow body has
/// no exact construction: an opened face that is not planar, a free boundary
/// that lies in none of the opened faces' planes or does not close into a
/// loop, or a result that comes back open, invalid or enclosing no volume.
#[allow(clippy::too_many_lines)]
pub fn shell(
    topo: &mut Topology,
    solid: SolidId,
    thickness: f64,
    open_faces: &[FaceId],
) -> Result<SolidId, crate::OperationsError> {
    let tol = Tolerance::new();

    if thickness <= tol.linear {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("shell thickness must be positive, got {thickness}"),
        });
    }

    let solid_data = topo.solid(solid)?;
    let shell_data = topo.shell(solid_data.outer_shell())?;
    let all_face_ids: Vec<FaceId> = shell_data.faces().to_vec();

    let open_set: HashSet<usize> = open_faces.iter().map(|f| f.index()).collect();

    let solid_face_set: HashSet<usize> = all_face_ids.iter().map(|f| f.index()).collect();
    for &of in open_faces {
        if !solid_face_set.contains(&of.index()) {
            return Err(crate::OperationsError::InvalidInput {
                reason: format!("face {} is not part of the solid", of.index()),
            });
        }
    }

    // Collect face vertex data (samples curved edges for proper polygons).
    // A face's holes are collected alongside its boundary: they are part of
    // the face, they move with it, and dropping them here is what filled a
    // shelled body's bores back in.
    let mut face_verts: Vec<(FaceId, Vec<Point3>)> = Vec::new();
    let mut face_holes: Vec<Vec<Vec<Point3>>> = Vec::new();
    for &fid in &all_face_ids {
        let verts =
            crate::boolean::wire_polygon_closed_subloops(topo, topo.face(fid)?.outer_wire())?;
        face_verts.push((fid, verts));
        let hole_wires: Vec<_> = topo.face(fid)?.inner_wires().to_vec();
        let mut holes = Vec::with_capacity(hole_wires.len());
        for wid in hole_wires {
            holes.push(crate::boolean::wire_polygon(topo, wid)?);
        }
        face_holes.push(holes);
    }

    let mut result_specs: Vec<FaceSpec> = Vec::new();

    // ─── Phase 1: Build vertex→normals map using ALL face types ───────────
    //
    // For each vertex, collect the outward surface normals from ALL adjacent
    // faces (planar and non-planar). We use these to compute a miter vector
    // that gives the correct inner vertex position at the intersection of
    // all offset surfaces meeting at that vertex.
    let inv_tol = 1.0 / tol.linear;
    let quantize_pt = |p: Point3| -> (i64, i64, i64) {
        (
            (p.x() * inv_tol).round() as i64,
            (p.y() * inv_tol).round() as i64,
            (p.z() * inv_tol).round() as i64,
        )
    };

    let mut vertex_normals: HashMap<(i64, i64, i64), Vec<(Vec3, bool)>> = HashMap::new();

    for (&(fid, ref verts), holes) in face_verts.iter().zip(&face_holes) {
        let face = topo.face(fid)?;
        let is_open = open_set.contains(&fid.index());

        // A convex fillet whose radius the thickness swallows does not offset
        // to a smaller fillet — it collapses to a sharp edge where the two
        // NEIGHBOURING offset surfaces meet. Its own normal is useless for
        // that: at each tangent vertex it equals the neighbour's normal, so
        // the miter sees one direction, offsets perpendicular only, and the
        // neighbours overshoot past each other by (thickness - radius) instead
        // of meeting. Feeding every vertex of the collapsing face BOTH extreme
        // normals puts the miter on the intersection of the two offset
        // surfaces, which is exactly the sharp corner.
        let collapsing = match face.surface() {
            FaceSurface::Cylinder(cyl) => {
                !face.is_reversed() && cyl.radius() - thickness <= tol.linear
            }
            FaceSurface::Plane { .. }
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_)
            | FaceSurface::Nurbs(_) => false,
        };
        let extreme_normals = if collapsing {
            extreme_face_normals(&face_surface_normals(face, verts))
        } else {
            None
        };

        // A hole's rim is normally also on the wall that bounds it, so its
        // normals arrive twice over; a rim shared by two holed faces would
        // otherwise contribute none at all and be left un-offset.
        for v in verts.iter().chain(holes.iter().flatten()) {
            let (u, v_param) = face.surface().project_point(*v).unwrap_or((0.0, 0.0));
            let mut normal = face.surface().normal(u, v_param);
            // Account for the face's reversal flag: when a face is reversed,
            // the native surface normal points in the wrong direction.
            if face.is_reversed() {
                normal = -normal;
            }
            let entry = vertex_normals.entry(quantize_pt(*v)).or_default();
            if let Some((n_a, n_b)) = extreme_normals {
                entry.push((n_a, is_open));
                entry.push((n_b, is_open));
            } else {
                entry.push((normal, is_open));
            }
        }
    }

    // ─── Phase 2: Compute inner vertex positions via miter vectors ────────
    //
    // The miter vector m at a vertex satisfies m · n_i = 1 for each unique
    // face normal n_i. The inner position is: inner = outer - thickness * m.
    // This correctly handles vertices where 2 or 3 offset surfaces intersect
    // (including non-planar surfaces like cylinders at tangent points).
    //
    // For open faces, the offset distance is 0 (the rim vertex stays on the
    // original plane), so we use n_i with a weight of 0 in that direction.
    let mut inner_pos: HashMap<(i64, i64, i64), Point3> = HashMap::new();

    for (&key, normals) in &vertex_normals {
        // Deduplicate nearly-parallel normals, keeping track of whether
        // each unique normal is offset (non-open) or stays (open).
        let mut unique: Vec<(Vec3, bool)> = Vec::new();
        for &(n, is_open) in normals {
            // Use cosine similarity to deduplicate nearly-parallel normals.
            // At tangent points (where a flat face meets a curved face),
            // normals can differ by small amounts that still cause near-singular
            // miter vectors if treated as independent.
            let dominated = unique.iter_mut().any(|(un, existing_open)| {
                let dot = un.dot(n);
                if dot > 0.995 {
                    // Nearly parallel — merge. Prefer the non-open (offset) variant.
                    if *existing_open && !is_open {
                        *un = n;
                        *existing_open = false;
                    }
                    true
                } else {
                    false
                }
            });
            if !dominated {
                unique.push((n, is_open));
            }
        }

        // Reconstruct the outer point from the quantized key.
        let outer_pt = Point3::new(
            key.0 as f64 / inv_tol,
            key.1 as f64 / inv_tol,
            key.2 as f64 / inv_tol,
        );

        // Build the miter offset: solve N · m = b where b_i = thickness
        // for non-open faces, 0 for open faces.
        let inner = compute_miter_offset(outer_pt, &unique, thickness);
        inner_pos.insert(key, inner);
    }

    // ─── Phase 3: Outer faces — the non-open faces, kept as-is ─────────────
    //
    // "As-is" means copied, not re-described. Rebuilding them from a list of
    // outer-wire positions lost three things at once: every inner wire, so a
    // bore's mouth filled in and its wall was left referenced by nothing; the
    // face's orientation, hardcoded un-reversed, so a bore's wall came back
    // facing into the metal; and the exact curves of its edges, so that wall
    // was faceted into a polygon on the way.
    for &(fid, _) in &face_verts {
        if open_set.contains(&fid.index()) {
            continue;
        }
        result_specs.push(FaceSpec::Existing {
            face: fid,
            outer: None,
        });
    }

    // ─── Phase 4: Inner faces (offset of non-open faces) ──────────────────
    //
    // All inner vertex positions come from the miter vector computation in
    // Phase 2. This ensures watertight geometry at ALL junctions, including
    // where planar faces meet cylindrical faces at tangent points.

    for (&(fid, ref outer_verts), holes) in face_verts.iter().zip(&face_holes) {
        if open_set.contains(&fid.index()) {
            continue;
        }
        let face = topo.face(fid)?;
        // A reversed face's outward normal is the negative of its surface's,
        // so it is displaced the other way: a bore widens where a boss
        // narrows, and the inner face ends up on the far side of the surface
        // from where an un-reversed face's would.
        let concave = face.is_reversed();

        let displaced = |p: Point3| inner_pos.get(&quantize_pt(p)).copied().unwrap_or(p);
        // Reversed winding gives the inner face an inward-pointing normal.
        let inner_verts: Vec<Point3> = outer_verts.iter().map(|v| displaced(*v)).rev().collect();
        // The holes travel with the face, through the same miter vectors, so
        // a bore's mouth in the inner cap meets the rim of the offset bore
        // wall rather than floating a wall thickness away from it.
        let inner_holes: Vec<Vec<Point3>> = holes
            .iter()
            .map(|rim| rim.iter().map(|v| displaced(*v)).rev().collect())
            .collect();

        match face.surface() {
            FaceSurface::Plane { normal, .. } => {
                let inner_normal = if concave { *normal } else { -*normal };
                let inner_d = dot_normal_point(inner_normal, inner_verts[0]);
                result_specs.push(FaceSpec::Planar {
                    vertices: inner_verts,
                    normal: inner_normal,
                    d: inner_d,
                    inner_wires: inner_holes,
                });
            }
            FaceSurface::Cylinder(cyl) => {
                let new_radius = if concave {
                    cyl.radius() + thickness
                } else {
                    cyl.radius() - thickness
                };
                if new_radius <= tol.linear {
                    // A swallowed convex fillet collapses to a sharp chamfer
                    // strip joining the neighbouring offset walls. Its
                    // vertices already carry the extreme-normal miter above.
                    let wire = topo.wire(face.outer_wire())?;
                    let mut strip: Vec<Point3> = Vec::new();
                    for oe in wire.edges() {
                        let edge = topo.edge(oe.edge())?;
                        let vertex = topo.vertex(oe.oriented_start(edge))?.point();
                        let point = inner_pos
                            .get(&quantize_pt(vertex))
                            .copied()
                            .unwrap_or(vertex);
                        if strip
                            .last()
                            .is_none_or(|previous| (*previous - point).length() > tol.linear)
                        {
                            strip.push(point);
                        }
                    }
                    if strip.len() > 2 && (strip[0] - strip[strip.len() - 1]).length() <= tol.linear
                    {
                        strip.pop();
                    }
                    if strip.len() >= 3
                        && let Some((normal_a, normal_b)) =
                            extreme_face_normals(&face_surface_normals(face, outer_verts))
                        && let Ok(outward) = (normal_a + normal_b).normalize()
                    {
                        strip.reverse();
                        let inner_normal = -outward;
                        let inner_d = dot_normal_point(inner_normal, strip[0]);
                        result_specs.push(FaceSpec::Planar {
                            vertices: strip,
                            normal: inner_normal,
                            d: inner_d,
                            inner_wires: vec![],
                        });
                    }
                } else if let Ok(new_cyl) = brepkit_math::surfaces::CylindricalSurface::new(
                    cyl.origin(),
                    cyl.axis(),
                    new_radius,
                ) {
                    // `CylindricalFace` mints Circle edges along the constant
                    // -height boundaries, so the wall's rims are the arcs the
                    // caps meeting them also trace, and both a full circle's
                    // seam — traversed once each way — and a partial arc's
                    // angular range survive. `Surface` would drop the seam's
                    // second traversal as a duplicate edge and leave the rims
                    // as free chords.
                    result_specs.push(FaceSpec::CylindricalFace {
                        vertices: inner_verts,
                        cylinder: new_cyl,
                        reversed: !concave,
                        inner_wires: inner_holes,
                    });
                }
            }
            FaceSurface::Cone(_) | FaceSurface::Nurbs(_) | FaceSurface::Torus(_) => {
                // `offset_face` moves along the SURFACE normal, which is the
                // face's outward normal only when the face is not reversed.
                let along_surface = if concave { thickness } else { -thickness };
                let inner_fid = crate::offset_face::offset_face(topo, fid, along_surface, 8)?;
                let inner_face = topo.face(inner_fid)?;
                result_specs.push(FaceSpec::Surface {
                    vertices: inner_verts,
                    surface: inner_face.surface().clone(),
                    reversed: !concave,
                    inner_wires: inner_holes,
                });
            }
            FaceSurface::Sphere(sphere) => {
                let new_r = if concave {
                    sphere.radius() + thickness
                } else {
                    sphere.radius() - thickness
                };
                if new_r <= 0.0 {
                    return Err(crate::OperationsError::InvalidInput {
                        reason: format!(
                            "shell thickness ({thickness}) exceeds sphere radius ({}), \
                             resulting inner sphere would have non-positive radius ({new_r})",
                            sphere.radius(),
                        ),
                    });
                }
                let new_sph = brepkit_math::surfaces::SphericalSurface::new(sphere.center(), new_r)
                    .map_err(crate::OperationsError::Math)?;
                result_specs.push(FaceSpec::Surface {
                    vertices: inner_verts,
                    surface: FaceSurface::Sphere(new_sph),
                    reversed: !concave,
                    inner_wires: inner_holes,
                });
            }
        }
    }

    // ─── Phase 5: Assemble outer + inner faces, then close rim ─────────────
    //
    // Instead of creating disconnected rim quads (which don't share edges
    // with the outer/inner faces), we first assemble the outer + inner faces
    // into a solid with open boundaries, then find the boundary edges and
    // create a single annular rim face per open face. This guarantees edge
    // sharing and produces a manifold shell.

    if result_specs.is_empty() {
        return Err(crate::OperationsError::InvalidInput {
            reason: "shell operation produced no faces".into(),
        });
    }

    let solid = assemble_solid_mixed(topo, &result_specs, tol)?;

    let edge_face_map = brepkit_topology::explorer::edge_to_face_map(topo, solid)?;
    let mut boundary_edge_ids: Vec<brepkit_topology::edge::EdgeId> = Vec::new();
    for (&edge_idx, faces) in &edge_face_map {
        if faces.len() == 1
            && let Some(eid) = topo.edge_id_from_index(edge_idx)
        {
            boundary_edge_ids.push(eid);
        }
    }
    // `edge_face_map` is a HashMap, so this collection order is seed-dependent.
    // It decides where `sort_edges_into_loops` starts each chain, and a
    // different starting edge splits the rim into a different NUMBER of loops
    // — the cup's rim came back with two or three inner wires depending on the
    // process, which moved its measured volume by hundreds of units. Sort so
    // the rim is decomposed the same way every run.
    boundary_edge_ids.sort_unstable_by_key(|e| e.index());

    if boundary_edge_ids.is_empty() {
        // No open boundary — shell is already closed (no open faces, or all faces present).
        return gate(topo, solid);
    }

    // Determine the oriented direction of each boundary edge relative to its single face.
    // The rim face must use the OPPOSITE orientation so the edge is shared correctly.
    let mut boundary_oriented: Vec<OrientedEdge> = Vec::new();
    for &eid in &boundary_edge_ids {
        let face_id = edge_face_map[&eid.index()][0];
        let face = topo.face(face_id)?;
        let wire = topo.wire(face.outer_wire())?;
        let mut found = false;
        for oe in wire.edges() {
            if oe.edge() == eid {
                boundary_oriented.push(OrientedEdge::new(eid, !oe.is_forward()));
                found = true;
                break;
            }
        }
        if !found {
            for &iw_id in face.inner_wires() {
                let iw = topo.wire(iw_id)?;
                for oe in iw.edges() {
                    if oe.edge() == eid {
                        boundary_oriented.push(OrientedEdge::new(eid, !oe.is_forward()));
                        found = true;
                        break;
                    }
                }
                if found {
                    break;
                }
            }
            if !found {
                // Fallback: use forward orientation.
                boundary_oriented.push(OrientedEdge::new(eid, true));
            }
        }
    }

    let loops = sort_edges_into_loops(topo, &boundary_oriented)?;
    if loops.is_empty() {
        return Err(unsupported(
            "the shell has free edges that do not close into any loop, so its opening \
             cannot be rimmed",
        ));
    }

    // Materialise each rim loop as a wire and sample the polygon it traces —
    // a rim can be a single closed circle edge, which is one vertex until it
    // is sampled and no polygon to nest at all.
    let mut rim_wires = Vec::with_capacity(loops.len());
    let mut rim_polys: Vec<Vec<Point3>> = Vec::with_capacity(loops.len());
    for lp in &loops {
        let wire = brepkit_topology::wire::Wire::new(lp.clone(), true)
            .map_err(crate::OperationsError::Topology)?;
        let wid = topo.add_wire(wire);
        let poly = crate::boolean::wire_polygon(topo, wid)?;
        if poly.is_empty() {
            return Err(unsupported(
                "a free loop of the shelled body traces no points, so its rim has no \
                 shape to close",
            ));
        }
        rim_polys.push(poly);
        rim_wires.push(wid);
    }

    // Every rim lies in the plane of a face that was opened: the outer wall's
    // rim is that face's own boundary, and the inner wall's rim is where the
    // miter offset ran out — offset zero in the opened face's direction, so
    // it stays on that plane. Group the rims by the plane they belong to,
    // because two openings — or one opening that a bore passes through —
    // must be closed with several rim faces, not one face swallowing every
    // other loop as a hole.
    let mut rim_planes: Vec<(Vec3, f64)> = Vec::new();
    for &(fid, _) in &face_verts {
        if !open_set.contains(&fid.index()) {
            continue;
        }
        let f = topo.face(fid)?;
        let FaceSurface::Plane { normal, d } = f.surface() else {
            return Err(unsupported(format!(
                "face {} was opened but is not planar, so the wall it exposes has no \
                 plane to be rimmed in",
                fid.index()
            )));
        };
        // Outward, away from the material the opening exposes.
        let plane = if f.is_reversed() {
            (-*normal, -*d)
        } else {
            (*normal, *d)
        };
        // Two faces opened in the SAME plane expose one rim between them, not
        // one each; a second entry for that plane would collect no loops and
        // read as an opening that exposed nothing.
        if !rim_planes.iter().any(|&(n, d)| {
            (n - plane.0).length() <= tol.linear && (d - plane.1).abs() <= tol.linear
        }) {
            rim_planes.push(plane);
        }
    }

    // Rim vertices reach their plane through the quantised miter grid, so
    // allow the grid's own step rather than exact incidence.
    let on_plane_tol = tol.linear * 16.0;
    let mut groups: Vec<Vec<usize>> = vec![Vec::new(); rim_planes.len()];
    for (i, poly) in rim_polys.iter().enumerate() {
        let plane = rim_planes.iter().position(|&(n, d)| {
            poly.iter()
                .all(|p| (dot_normal_point(n, *p) - d).abs() <= on_plane_tol)
        });
        let Some(plane) = plane else {
            return Err(unsupported(
                "a free loop of the shelled body lies in none of the opened faces' \
                 planes, so there is no face that closes it",
            ));
        };
        groups[plane].push(i);
    }

    let mut rim_face_ids = Vec::new();
    for (plane_idx, members) in groups.iter().enumerate() {
        let (normal, d) = rim_planes[plane_idx];
        if members.is_empty() {
            return Err(unsupported(
                "an opened face left no free boundary, so the wall behind it was never \
                 exposed",
            ));
        }
        let (u, v) = plane_basis(normal);

        // How many of the group's other loops enclose each loop. A loop at
        // even depth bounds material — the rim of the wall, or the rim of a
        // bore's own wall inside it — and the loops directly inside it are
        // its holes.
        let depth = |i: usize| -> usize {
            members
                .iter()
                .filter(|&&j| j != i && point_in_loop(&rim_polys[j], rim_polys[i][0], u, v))
                .count()
        };
        let depths: Vec<usize> = members.iter().map(|&i| depth(i)).collect();

        for (slot, &i) in members.iter().enumerate() {
            if !depths[slot].is_multiple_of(2) {
                continue;
            }
            let holes: Vec<_> = members
                .iter()
                .enumerate()
                .filter(|&(other, &j)| {
                    depths[other] == depths[slot] + 1
                        && point_in_loop(&rim_polys[i], rim_polys[j][0], u, v)
                })
                .map(|(_, &j)| rim_wires[j])
                .collect();
            let rim_face = brepkit_topology::face::Face::new(
                rim_wires[i],
                holes,
                FaceSurface::Plane { normal, d },
            );
            rim_face_ids.push(topo.add_face(rim_face));
        }
    }

    let solid_data = topo.solid(solid)?;
    let shell_id = solid_data.outer_shell();
    let shell = topo.shell(shell_id)?;
    let mut new_faces: Vec<FaceId> = shell.faces().to_vec();
    new_faces.extend(rim_face_ids);
    let new_shell =
        brepkit_topology::shell::Shell::new(new_faces).map_err(crate::OperationsError::Topology)?;
    *topo.shell_mut(shell_id)? = new_shell;

    gate(topo, solid)
}

/// An orthonormal pair spanning the plane with normal `n`.
fn plane_basis(n: Vec3) -> (Vec3, Vec3) {
    let seed = if n.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u = n
        .cross(seed)
        .normalize()
        .unwrap_or(Vec3::new(1.0, 0.0, 0.0));
    (u, n.cross(u))
}

/// Whether `p` is inside the closed loop `poly`, both flattened onto the
/// plane spanned by `u` and `v`. Even-odd crossing count.
fn point_in_loop(poly: &[Point3], p: Point3, u: Vec3, v: Vec3) -> bool {
    let flat = |q: Point3| {
        let w = Vec3::new(q.x(), q.y(), q.z());
        (u.dot(w), v.dot(w))
    };
    let (px, py) = flat(p);
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let (xi, yi) = flat(poly[i]);
        let (xj, yj) = flat(poly[(i + 1) % n]);
        if (yi > py) != (yj > py) {
            let t = (py - yi) / (yj - yi);
            if px < xi + t * (xj - xi) {
                inside = !inside;
            }
        }
    }
    inside
}

/// Refuse the result unless every edge of it is shared by exactly two face
/// uses, its wires and faces are well-formed, and it encloses positive
/// volume.
///
/// A shell used to be handed back whatever the assembler produced, and there
/// were several ways for that to come back open: a wall whose rim nothing
/// met because the cap that should have met it had its hole filled in, or a
/// rim broken into loops that were then stitched into one impossible face.
/// Both leave free edges, and free edges are what this refuses.
///
/// The manifold count is taken here rather than through
/// [`crate::validate::validate_solid`] so that the Euler identity
/// `V - E + F = 2 + L` is not applied: a hollow body is TWO closed surfaces,
/// its outside and its cavity, and that identity describes one. It rejects
/// the shell of a plain box as readily as anything else. Everything else
/// `validate_solid` would check is checked, by way of its relaxed form plus
/// the edge-sharing count that form leaves out.
fn gate(topo: &Topology, solid: SolidId) -> Result<SolidId, crate::OperationsError> {
    let edge_uses = brepkit_topology::explorer::edge_to_face_map(topo, solid)?;
    let free = edge_uses.values().filter(|f| f.len() < 2).count();
    let non_manifold = edge_uses.values().filter(|f| f.len() > 2).count();
    if free > 0 || non_manifold > 0 {
        return Err(unsupported(format!(
            "the shelled body is not closed: {free} free edges and {non_manifold} \
             edges shared by more than two faces"
        )));
    }

    let report = crate::validate::validate_solid_relaxed(topo, solid)?;
    if !report.is_valid() {
        let detail: Vec<&str> = report
            .issues
            .iter()
            .filter(|i| i.severity == crate::validate::Severity::Error)
            .map(|i| i.description.as_str())
            .collect();
        return Err(unsupported(format!(
            "the shelled body failed validation ({})",
            detail.join("; ")
        )));
    }

    let volume = crate::measure::solid_volume(topo, solid, VOLUME_DEFLECTION)?;
    if !volume.is_finite() || volume <= 0.0 {
        return Err(unsupported(format!(
            "the shelled body encloses no volume ({volume}); the walls turned inside out"
        )));
    }

    Ok(solid)
}

/// Sort oriented edges into connected loops.
///
/// Takes a set of oriented boundary edges and groups them into closed loops
/// by following edge connectivity (end vertex → start vertex of next edge).
fn sort_edges_into_loops(
    topo: &Topology,
    edges: &[OrientedEdge],
) -> Result<Vec<Vec<OrientedEdge>>, crate::OperationsError> {
    use brepkit_topology::vertex::VertexId;

    if edges.is_empty() {
        return Ok(Vec::new());
    }

    let mut start_map: HashMap<usize, Vec<usize>> = HashMap::new();
    let mut edge_endpoints: Vec<(VertexId, VertexId)> = Vec::new();
    for (i, oe) in edges.iter().enumerate() {
        let edge = topo.edge(oe.edge())?;
        let (sv, ev) = if oe.is_forward() {
            (edge.start(), edge.end())
        } else {
            (edge.end(), edge.start())
        };
        start_map.entry(sv.index()).or_default().push(i);
        edge_endpoints.push((sv, ev));
    }

    let mut used = vec![false; edges.len()];
    let mut loops = Vec::new();

    while let Some(start_idx) = used.iter().position(|&u| !u) {
        let mut current_loop = Vec::new();
        let mut current = start_idx;
        let chain_start_vid = edge_endpoints[current].0.index();

        loop {
            if used[current] {
                break;
            }
            used[current] = true;
            current_loop.push(edges[current]);
            let end_vid = edge_endpoints[current].1.index();

            if end_vid == chain_start_vid {
                break; // Loop closed.
            }

            let mut found = false;
            if let Some(candidates) = start_map.get(&end_vid) {
                for &idx in candidates {
                    if !used[idx] {
                        current = idx;
                        found = true;
                        break;
                    }
                }
            }
            if !found {
                break; // Broken chain — give up on this loop.
            }
        }

        if !current_loop.is_empty() {
            loops.push(current_loop);
        }
    }

    Ok(loops)
}

#[cfg(test)]
mod tests;

/// Outward normals of `face` at each of `verts`, honouring the reversal flag.
fn face_surface_normals(face: &brepkit_topology::face::Face, verts: &[Point3]) -> Vec<Vec3> {
    verts
        .iter()
        .map(|v| {
            let (u, vp) = face.surface().project_point(*v).unwrap_or((0.0, 0.0));
            let n = face.surface().normal(u, vp);
            if face.is_reversed() { -n } else { n }
        })
        .collect()
}

/// The two most widely separated normals in `normals` (the ends of a fillet's
/// angular sweep), or `None` if they are all effectively parallel — a face that
/// spans no angle has no sharp corner to collapse to.
fn extreme_face_normals(normals: &[Vec3]) -> Option<(Vec3, Vec3)> {
    let mut best: Option<(f64, Vec3, Vec3)> = None;
    for (i, a) in normals.iter().enumerate() {
        for b in &normals[i + 1..] {
            let d = a.dot(*b);
            if best.is_none_or(|(bd, _, _)| d < bd) {
                best = Some((d, *a, *b));
            }
        }
    }
    // cos > 0.999 is under a couple of degrees: not a real corner.
    best.filter(|&(d, _, _)| d < 0.999).map(|(_, a, b)| (a, b))
}
