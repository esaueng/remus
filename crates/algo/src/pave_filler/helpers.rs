//! Shared helper functions for PaveFiller phases.
//!
//! Extracted from phase_ee, phase_ef, and phase_ve to eliminate
//! duplicated vertex-lookup and pave-insertion logic.

use remus_math::tolerance::Tolerance;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeId};
use remus_topology::face::FaceSurface;
use remus_topology::vertex::VertexId;

use crate::ds::{GfaArena, Pave};
use crate::error::AlgoError;

/// Clamped quarter-arc knots (`to_nurbs` u) and clamped linear knots (v) of
/// an exact-rational converted cylinder wall.
const WALL_KNOTS_U: [f64; 12] = [
    0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
];
const WALL_KNOTS_V: [f64; 4] = [0.0, 0.0, 1.0, 1.0];

/// Resolve the stored parameter authority for a topology edge.
///
/// PaveFiller must never reconstruct a curved edge's branch from its endpoint
/// positions: periodic seams and major/reversed spans are not recoverable from
/// those points alone. Lines retain their intrinsic endpoint-local `[0, 1]`
/// domain through [`Edge::strict_domain`].
///
/// Visible to `crate::builder` and `crate::classifier` alongside the other
/// helpers in this module (see the `redundant_pub_crate` note on `mod helpers`).
#[allow(clippy::redundant_pub_crate)]
pub(crate) fn authoritative_edge_domain(
    edge: &Edge,
    edge_id: EdgeId,
    stage: &'static str,
) -> Result<(f64, f64), AlgoError> {
    edge.strict_domain().map_err(|error| {
        AlgoError::IntersectionFailed(format!(
            "{stage} edge {edge_id:?} lacks authoritative parameter range: {error}"
        ))
    })
}

/// Validate a complete edge set before a phase starts mutating its arena.
pub(super) fn validate_edge_domains(
    topo: &Topology,
    edges: &[EdgeId],
    stage: &'static str,
) -> Result<(), AlgoError> {
    for &edge_id in edges {
        let edge = topo.edge(edge_id)?;
        authoritative_edge_domain(edge, edge_id, stage)?;
    }
    Ok(())
}

/// Return the part of a vertex ball that widens an operation beyond its
/// global linear floor.
pub(super) fn vertex_tolerance_excess(
    topo: &Topology,
    vertex_id: VertexId,
    floor: f64,
) -> Result<f64, AlgoError> {
    let value = topo.vertex(vertex_id)?.tolerance();
    tolerance_excess(value, floor, "vertex")
}

/// Return the part of an edge tube that widens an operation beyond its
/// global linear floor.
pub(super) fn edge_tolerance_excess(
    topo: &Topology,
    edge_id: EdgeId,
    floor: f64,
) -> Result<f64, AlgoError> {
    let edge = topo.edge(edge_id)?;
    let start_tolerance = topo.vertex(edge.start())?.tolerance();
    let end_tolerance = topo.vertex(edge.end())?.tolerance();
    tolerance_excess(start_tolerance, floor, "vertex")?;
    tolerance_excess(end_tolerance, floor, "vertex")?;
    let vertex_tolerance = start_tolerance.max(end_tolerance);
    tolerance_excess(edge.effective_tolerance(vertex_tolerance), floor, "edge")
}

/// Add tolerance contributions and reject a non-finite acceptance band.
pub(super) fn tolerance_band(
    floor: f64,
    contributions: impl IntoIterator<Item = f64>,
) -> Result<f64, AlgoError> {
    let mut band = floor;
    for contribution in contributions {
        band += contribution;
    }
    if !band.is_finite() || band.is_sign_negative() {
        return Err(remus_topology::TopologyError::InvalidToleranceValue {
            entity: "predicate band",
            value: band,
        }
        .into());
    }
    Ok(band)
}

fn tolerance_excess(value: f64, floor: f64, entity: &'static str) -> Result<f64, AlgoError> {
    if !value.is_finite() || value.is_sign_negative() {
        return Err(remus_topology::TopologyError::InvalidToleranceValue { entity, value }.into());
    }
    Ok((value - floor).max(0.0))
}

/// Find a vertex near the given point among all pave block vertices.
///
/// Returns the resolved (same-domain canonical) vertex first encountered within
/// the operation floor or the candidate vertex's wider tolerance ball,
/// scanning pave blocks in `edge_pave_blocks` order
/// (ascending `EdgeId`, start-before-end). When the arena's spatial index is
/// available (built after Phase VV) the lookup is O(1) and returns the exact
/// same vertex; otherwise it falls back to the linear scan.
pub(super) fn find_nearby_pave_vertex(
    topo: &Topology,
    arena: &GfaArena,
    point: Point3,
    tol: Tolerance,
) -> Option<VertexId> {
    if let Some(index) = &arena.pave_vertex_index {
        return index.find_with_entry_radius(point);
    }
    for pbs in arena.edge_pave_blocks.values() {
        for &pb_id in pbs {
            if let Some(pb) = arena.pave_blocks.get(pb_id) {
                for vid in [pb.start.vertex, pb.end.vertex] {
                    crate::perf::bump_pave_vertex_probe();
                    let resolved = arena.resolve_vertex(vid);
                    if let Ok(v) = topo.vertex(resolved)
                        && (v.point() - point).length() <= v.tolerance().max(tol.linear)
                    {
                        return Some(resolved);
                    }
                }
            }
        }
    }
    None
}

/// Widened variant of [`find_nearby_pave_vertex`] for tangential contacts.
///
/// A grazing crossing's solved position is only accurate to
/// `sqrt(2 * r * residual)`, so the exact junction vertex can sit microns
/// outside the linear tolerance. This scans every pave-block endpoint within
/// `radius` and returns the nearest candidate that passes `accept` (the
/// caller checks genuine curve/surface incidence, which is what makes the
/// widened radius safe). If the accepted candidates span more than one
/// distinct position (beyond `tol_linear` of each other), the contact is
/// ambiguous — two different junctions inside the window — and `None` is
/// returned so the caller keeps the solved point rather than merging
/// distinct junctions. The spatial index uses cells at least as large as the
/// maximum widened radius, keeping this lookup bounded to a 3x3x3 stencil.
pub(super) fn find_nearby_pave_vertex_widened(
    arena: &GfaArena,
    point: Point3,
    radius: f64,
    tol_linear: f64,
    accept: impl Fn(Point3) -> bool,
) -> Option<VertexId> {
    arena
        .pave_vertex_index
        .as_ref()?
        .find_unambiguous_within(point, radius, tol_linear, accept)
}

/// Add a pave to the appropriate pave block of an edge.
///
/// Finds the pave block whose parameter range contains the pave's
/// parameter (with a small guard band) and adds the extra pave to it.
pub(super) fn add_pave_to_edge(arena: &mut GfaArena, edge_id: EdgeId, pave: Pave) {
    if let Some(pb_ids) = arena.edge_pave_blocks.get(&edge_id) {
        let pb_ids_copy: Vec<_> = pb_ids.clone();
        for pb_id in pb_ids_copy {
            if let Some(pb) = arena.pave_blocks.get_mut(pb_id)
                && pb.contains_parameter_interior(pave.parameter, 1e-10)
            {
                pb.add_extra_pave(pave);
            }
        }
    }
}

/// Recognize an exact-rational converted cylinder wall carried as NURBS.
///
/// Matches exactly what `CylindricalSurface::to_nurbs` (`math/src/surfaces.rs`)
/// emits: degree (2, 1), a 9×2 grid, clamped quarter-arc knots in u,
/// clamped linear knots in v, 1/√2 diagonal weights, coincident seam rows,
/// parallel ring axes, and one common radius. Returns the recovered
/// analytic cylinder so callers can measure against it exactly.
///
/// Structural gates only — sampled/freeform sheets (cone/sphere/torus
/// sampled grids, imported freeform) return `None` and keep their previous
/// handling. In particular the `(9, 2)` grid + `(2, 1)` degree + knot
/// fingerprint excludes every other `convert_to_bspline` emitter (bilinear
/// planes are (1,1) 2×2; cone/sphere/torus are 33×9 degree-1 sampled).
/// Callers must additionally bound the result to the face's trimmed region
/// (containment/extent); the carrier alone is unbounded in v.
#[allow(clippy::items_after_statements, clippy::redundant_pub_crate)]
pub(crate) fn rational_cylinder_wall(
    nurbs: &remus_math::nurbs::surface::NurbsSurface,
    tol: Tolerance,
) -> Option<remus_math::surfaces::CylindricalSurface> {
    use remus_math::vec::Vec3;
    if nurbs.degree_u() != 2 || nurbs.degree_v() != 1 {
        return None;
    }
    let cps = nurbs.control_points();
    if cps.len() != 9 || cps.iter().any(|row| row.len() != 2) {
        return None;
    }
    let ws = nurbs.weights();
    if ws.len() != 9 || ws.iter().any(|row| row.len() != 2) {
        return None;
    }
    if nurbs.knots_u() != WALL_KNOTS_U || nurbs.knots_v() != WALL_KNOTS_V {
        return None;
    }
    let w1 = std::f64::consts::FRAC_1_SQRT_2;
    for (i, row) in ws.iter().enumerate() {
        let expected = if i % 2 == 0 { 1.0 } else { w1 };
        if (row[0] - expected).abs() > 1e-12
            || (row[1] - expected).abs() > 1e-12
            || (row[0] - row[1]).abs() > 1e-12
        {
            return None;
        }
    }
    // Closed seam: first and last rows coincide (same gate `is_periodic_u`
    // uses, tightened to the caller's linear tolerance).
    if (cps[0][0] - cps[8][0]).length() > tol.linear
        || (cps[0][1] - cps[8][1]).length() > tol.linear
    {
        return None;
    }
    // All nine ring axes parallel: the v-direction column of every row.
    let mut axis_sum = Vec3::new(0.0, 0.0, 0.0);
    for row in cps {
        let v = row[1] - row[0];
        let len = v.length();
        if !len.is_finite() || len <= tol.linear {
            return None;
        }
        axis_sum += v * (1.0 / len);
    }
    let axis = axis_sum.normalize().ok()?;
    // Ring centre from the four cardinal bottom points; common radius.
    let bot = {
        let (mut sx, mut sy, mut sz) = (0.0, 0.0, 0.0);
        for i in [0, 2, 4, 6] {
            sx += cps[i][0].x();
            sy += cps[i][0].y();
            sz += cps[i][0].z();
        }
        Point3::new(sx / 4.0, sy / 4.0, sz / 4.0)
    };
    let mut radius = 0.0;
    for i in [0, 2, 4, 6] {
        radius += ((cps[i][0] - bot) - axis * axis.dot(cps[i][0] - bot)).length();
    }
    radius /= 4.0;
    if !radius.is_finite() || radius <= tol.linear {
        return None;
    }
    // Every cardinal bottom point sits on the recovered cylinder.
    for i in [0, 2, 4, 6] {
        let radial = (cps[i][0] - bot) - axis * axis.dot(cps[i][0] - bot);
        if (radial.length() - radius).abs() > tol.linear {
            return None;
        }
    }
    remus_math::surfaces::CylindricalSurface::new(bot, axis, radius).ok()
}

/// The plane a NURBS surface lies in when its whole control net is coplanar
/// within `tol.linear` — a rational surface never leaves the convex hull of
/// its net, so coplanar control points certify a planar surface exactly.
/// `None` for a genuinely curved net or for anything but a NURBS.
#[allow(clippy::redundant_pub_crate)]
pub(crate) fn planar_nurbs_as_plane(surface: &FaceSurface, tol: Tolerance) -> Option<FaceSurface> {
    let FaceSurface::Nurbs(nurbs) = surface else {
        return None;
    };
    let points: Vec<Point3> = nurbs
        .control_points()
        .iter()
        .flat_map(|row| row.iter().copied())
        .collect();
    let origin = *points.first()?;
    // A well-conditioned normal in two linear passes: the net point farthest
    // from the origin spans the first direction, and the point whose cross
    // product with it is largest spans the second.
    let far = points
        .iter()
        .copied()
        .max_by(|a, b| (*a - origin).length().total_cmp(&(*b - origin).length()))?;
    let axis = far - origin;
    let (len, normal) = points
        .iter()
        .map(|&p| {
            let n = axis.cross(p - origin);
            (n.length(), n)
        })
        .filter(|(l, _)| l.is_finite())
        .max_by(|a, b| a.0.total_cmp(&b.0))?;
    if len <= tol.linear * tol.linear {
        return None;
    }
    let normal = normal * (1.0 / len);
    let d = normal.dot(origin - Point3::new(0.0, 0.0, 0.0));
    let coplanar = points
        .iter()
        .all(|p| (normal.dot(*p - Point3::new(0.0, 0.0, 0.0)) - d).abs() <= tol.linear);
    coplanar.then_some(FaceSurface::Plane { normal, d })
}
