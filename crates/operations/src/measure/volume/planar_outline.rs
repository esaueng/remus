//! A closed-form stand-in for the area-consistency probe on simple polygons.
//!
//! [`super::planar_face_area_is_consistent`] triangulates a planar face and
//! accepts its Green's-theorem area only when the face's own mesh area agrees
//! within a budget. For a face bounded by straight edges that budget is
//! `1e-9·max(area, 1)`: round-off only. This module proves, without building
//! the mesh, that the probe would accept, for the commonest such face: an
//! unholed polygon of few vertices lying exactly in a coordinate plane. Any
//! face it cannot certify goes to the probe unchanged, so it can only skip a
//! triangulation whose verdict is already known.
//!
//! # Why a certified face passes the probe
//!
//! The certificate holds when:
//!
//! 1. the face is planar with no inner wire, every edge a `Line`, its oriented
//!    edges chained end to start by vertex id, and at most
//!    [`OUTLINE_MAX_VERTICES`] of them;
//! 2. its stored normal has two exactly zero components, and every vertex has
//!    the same value of the remaining coordinate;
//! 3. every edge is longer than [`OUTLINE_SEPARATION`], every vertex is
//!    farther than that from every edge it is not an end of, and no two edges
//!    cross (exact `orient2d` signs); and
//! 4. the two round-off bounds below sum to at most half the probe's budget.
//!
//! By (1) the face's mesher reads exactly the polygon Green's theorem
//! integrates: the start vertex of each edge, in order. (3) makes it a simple
//! polygon whose vertices are far apart compared with every length the mesher
//! merges or snaps at (`1e-10` for consecutive points, `1e-8` for CDT
//! duplicates and for vertices on a constraint), so it meshes the polygon as
//! given: a constrained Delaunay triangulation, or the ear-clip fallback,
//! of `n − 2` triangles tiling it. (2) makes every triangle's 3-D normal point
//! exactly along the axis, so each triangle's computed area is its exact area
//! up to the rounding of its own two coordinate differences, two products, a
//! subtraction and a square root, and their sum is the polygon's area up to
//! `32·n·ε·Wx·Wy` (`Wx`, `Wy` the polygon's extent, `ε = 2⁻⁵³`), at least
//! four times the rounding those steps can produce. Green's sum is taken in
//! the face's own frame from vertices at most `R` from the origin; its
//! rounding is below `(32 + 2n)·n·ε·R² + 8·ε·area`, again at least twice
//! the forward error of the dot products, products and running sum. So the
//! probe's difference is at most the sum of the two bounds, which (4) holds
//! below half its budget, and the probe would accept.
//!
//! What the argument assumes is that the mesher triangulates a simple polygon
//! correctly. Where it would not, the old probe declined the exact path in
//! favour of a route that reads that same faulty face mesh; the certificate
//! takes the exact area instead.

use remus_math::predicates::orient2d;
use remus_math::vec::Point2;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

/// Most vertices a certified outline may have. The checks are quadratic in
/// it, so above this the triangulation is the cheaper proof.
const OUTLINE_MAX_VERTICES: usize = 16;

/// Least distance between a certified outline's vertices and the edges they
/// are not an end of: a hundred times the CDT's duplicate tolerance.
const OUTLINE_SEPARATION: f64 = 1e-6;

/// Largest coordinate magnitude a certified outline may have, so the float
/// distance tests resolve [`OUTLINE_SEPARATION`] with room to spare.
const OUTLINE_COORD_LIMIT: f64 = 1e6;

/// Unit round-off.
const EPS: f64 = f64::EPSILON / 2.0;

/// Whether the face is an outline the module docs prove the probe would
/// accept, with `area` its Green's-theorem area and `budget` the probe's.
pub(super) fn outline_area_is_certified(
    topo: &Topology,
    face_id: FaceId,
    area: f64,
    budget: f64,
) -> bool {
    let Some(outline) = axis_outline(topo, face_id) else {
        return false;
    };
    if !budget.is_finite() || !area.is_finite() {
        return false;
    }
    let points = &outline.points[..outline.len];
    let n = points.len();
    if !separated_and_simple(points) {
        return false;
    }

    #[allow(clippy::cast_precision_loss)]
    let n_f = n as f64;
    let (mut lo, mut hi) = (points[0], points[0]);
    for p in points {
        lo = Point2::new(lo.x().min(p.x()), lo.y().min(p.y()));
        hi = Point2::new(hi.x().max(p.x()), hi.y().max(p.y()));
    }
    let extent = (hi.x() - lo.x()) * (hi.y() - lo.y());
    let mesh_rounding = 32.0 * n_f * EPS * extent;
    let green_rounding =
        (32.0 + 2.0 * n_f) * n_f * EPS * outline.radius_sq + 8.0 * EPS * area.abs();
    mesh_rounding + green_rounding <= 0.5 * budget
}

/// A face's outline projected onto its coordinate plane.
struct AxisOutline {
    points: [Point2; OUTLINE_MAX_VERTICES],
    len: usize,
    /// Largest squared distance of a vertex from the world origin.
    radius_sq: f64,
}

/// The outline of an unholed, straight-edged face lying exactly in a
/// coordinate plane (conditions 1 and 2 of the module docs), else `None`.
fn axis_outline(topo: &Topology, face_id: FaceId) -> Option<AxisOutline> {
    let face = topo.face(face_id).ok()?;
    if !face.inner_wires().is_empty() {
        return None;
    }
    let FaceSurface::Plane { normal, .. } = face.surface() else {
        return None;
    };
    // The coordinate the plane fixes. The in-plane pair is the one the
    // planar mesher projects onto for an axis normal.
    let axis = match (normal.x() == 0.0, normal.y() == 0.0, normal.z() == 0.0) {
        (true, true, false) => 2,
        (true, false, true) => 1,
        (false, true, true) => 0,
        _ => return None,
    };
    let wire = topo.wire(face.outer_wire()).ok()?;
    let edges = wire.edges();
    if edges.len() < 3 || edges.len() > OUTLINE_MAX_VERTICES {
        return None;
    }
    let mut outline = AxisOutline {
        points: [Point2::new(0.0, 0.0); OUTLINE_MAX_VERTICES],
        len: edges.len(),
        radius_sq: 0.0,
    };
    let mut level = None;
    let mut previous_end = None;
    let mut first_start = None;
    for (slot, oe) in outline.points.iter_mut().zip(edges) {
        let edge = topo.edge(oe.edge()).ok()?;
        if !matches!(edge.curve(), EdgeCurve::Line) {
            return None;
        }
        let (start, end) = if oe.is_forward() {
            (edge.start(), edge.end())
        } else {
            (edge.end(), edge.start())
        };
        if previous_end.is_some_and(|previous| previous != start) {
            return None;
        }
        first_start.get_or_insert(start);
        previous_end = Some(end);

        let p = topo.vertex(start).ok()?.point();
        let coords = [p.x(), p.y(), p.z()];
        if coords
            .iter()
            .any(|c| !c.is_finite() || c.abs() > OUTLINE_COORD_LIMIT)
        {
            return None;
        }
        let fixed = coords[axis];
        // Exact equality is the condition: only then is every coordinate
        // difference along the axis exactly zero.
        #[allow(clippy::float_cmp)]
        if *level.get_or_insert(fixed) != fixed {
            return None;
        }
        *slot = match axis {
            2 => Point2::new(p.x(), p.y()),
            1 => Point2::new(p.x(), p.z()),
            _ => Point2::new(p.y(), p.z()),
        };
        outline.radius_sq = outline
            .radius_sq
            .max(p.x() * p.x() + p.y() * p.y() + p.z() * p.z());
    }
    // The chain closes on the first edge's start.
    (previous_end == first_start).then_some(outline)
}

/// Condition 3 of the module docs.
fn separated_and_simple(points: &[Point2]) -> bool {
    let n = points.len();
    let edge = |j: usize| (points[j], points[(j + 1) % n]);
    for j in 0..n {
        let (a, b) = edge(j);
        if (b - a).length_squared() <= OUTLINE_SEPARATION * OUTLINE_SEPARATION {
            return false;
        }
    }
    for (i, &p) in points.iter().enumerate() {
        for j in 0..n {
            // Edge `j` runs from vertex `j` to vertex `j + 1`.
            if j == i || (j + 1) % n == i {
                continue;
            }
            let (a, b) = edge(j);
            if segment_distance_sq(p, a, b) <= OUTLINE_SEPARATION * OUTLINE_SEPARATION {
                return false;
            }
        }
    }
    for j in 0..n {
        // Adjacent edges share a vertex; their overlap is excluded above.
        for k in j + 2..n {
            if j == 0 && k == n - 1 {
                continue;
            }
            let (a, b) = edge(j);
            let (c, d) = edge(k);
            if segments_cross(a, b, c, d) {
                return false;
            }
        }
    }
    true
}

/// Squared distance from `p` to the segment `[a, b]`.
fn segment_distance_sq(p: Point2, a: Point2, b: Point2) -> f64 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
    let foot = Point2::new(ab.x().mul_add(t, a.x()), ab.y().mul_add(t, a.y()));
    (p - foot).length_squared()
}

/// Whether `[a, b]` and `[c, d]` cross at a point interior to both.
fn segments_cross(a: Point2, b: Point2, c: Point2, d: Point2) -> bool {
    let opposite = |s: f64, t: f64| (s > 0.0 && t < 0.0) || (s < 0.0 && t > 0.0);
    opposite(orient2d(a, b, c), orient2d(a, b, d)) && opposite(orient2d(c, d, a), orient2d(c, d, b))
}
