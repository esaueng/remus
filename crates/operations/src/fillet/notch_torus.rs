//! Torus corner patch geometry for mixed-side 90-degree notch vertices.
//!
//! At a vertex where two convex planar edges and one concave planar edge
//! meet over mutually perpendicular supports (the L-bracket notch family),
//! the constant-radius corner closes with an exact torus patch — not a
//! sphere. The torus ring runs through the three stripe stations; each
//! stripe meets it in a smooth (G1) circular seam, and it rests tangent on
//! the cap face along a tangency circle. Verified against an independent
//! executable's whole-edge bracket result (ring center `V+(R,R,R)`, ring
//! radius `2R`, tube `R`, stations at contact crossings).
//!
//! ```text
//! Bottom notch vertex V=(8,8,0), R=1 (S1: y=8, S2: x=8, B: z=0):
//!   C2 (ring center) = (9,9,1), ring radius 2, tube 1, axis -Z.
//!   M1 = (9,8,1)   convex/S1-contact x concave/S1-contact crossing.
//!   M2 = (8,9,1)   convex/S2-contact x concave/S2-contact crossing.
//!   B1 = (9,7,0)   convex station circle meets B-contact.
//!   B2 = (7,9,0)   convex station circle meets B-contact.
//!   Seams: T1 (x=9 circle M1->B1), T2 (y=9 circle M2->B2),
//!          T3 (z=1 circle M1->M2 void-quarter),
//!          B-tangency (z=0 circle radius 2, B1->B2 V-quarter).
//! ```
//!
//! All computation here is pure (positions, frames, circles); topology
//! assembly consumes the results. Only the concave-singleton,
//! 90-degree-rectangular family is qualified; anything else returns `None`
//! and the caller keeps its typed refusal.

use remus_math::curves::Circle3D;
use remus_math::det_hash::DetHashMap;
use remus_math::surfaces::ToroidalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};

/// Qualified notch frame: cap outward normal plus the two wall outward
/// normals (pairwise perpendicular). The concave edge's supports are always
/// the two walls here (concave-singleton family).
#[derive(Debug, Clone, Copy)]
pub struct NotchFrame {
    /// Cap-face outward normal (unit).
    pub cap_outward: Vec3,
    /// First wall outward normal (unit): carries the first convex edge.
    pub wall_a: Vec3,
    /// Second wall outward normal (unit): carries the second convex edge.
    pub wall_b: Vec3,
}

/// A support plane: outward unit normal plus the offset `n·p`, shared by
/// coplanar faces from fuses (a lip and its plate meet flush, so one
/// logical plane arrives as two face indices).
#[derive(Debug, Clone, Copy)]
pub struct SupportPlane {
    /// Outward unit normal.
    pub normal: Vec3,
    /// Plane offset `n·p`.
    pub d: f64,
}

/// Resolve an edge's two support faces to planes. Returns `None` when a
/// normal or offset is missing (the caller keeps its typed refusal).
fn support_planes(
    faces: &[usize; 2],
    outward: &DetHashMap<usize, Vec3>,
    offsets: &DetHashMap<usize, f64>,
) -> Option<[SupportPlane; 2]> {
    let mut planes = Vec::with_capacity(2);
    for f in faces {
        planes.push(SupportPlane {
            normal: outward.get(f).copied()?,
            d: offsets.get(f).copied()?,
        });
    }
    Some([planes[0], planes[1]])
}

/// Whether two support entries are the same logical plane: same outward
/// direction (angular tolerance) and same offset (linear tolerance scaled
/// by magnitude, so large coordinates do not false-split planes).
pub fn same_plane(a: SupportPlane, b: SupportPlane, tol: Tolerance) -> bool {
    (a.normal.dot(b.normal) - 1.0).abs() <= tol.angular
        && (a.d - b.d).abs() <= tol.linear * (1.0 + a.d.abs().max(b.d.abs()))
}

/// Whether either of an edge's support planes matches the given plane.
fn edge_has_plane(planes: &[SupportPlane; 2], plane: SupportPlane, tol: Tolerance) -> bool {
    same_plane(planes[0], plane, tol) || same_plane(planes[1], plane, tol)
}

/// Qualify a mixed-side vertex as a 90-degree rectangular notch with a
/// concave singleton.
///
/// Inputs are per selected incident edge (exactly three): `edge_faces[k]`
/// the two support faces of edge `k` (as arena indices), `outward` maps a
/// support-face index to its outward unit normal, `offsets` maps it to the
/// plane offset `n·p`, and `convex[k]` marks convex edges. Faces sharing
/// one logical plane (coplanar fuse splits) pair by plane, not by index.
/// Returns the frame (cap = the plane incident to both convex edges; walls
/// = the planes incident to the concave edge, each paired with its convex
/// edge) plus the concave edge index, when exactly one edge is concave,
/// exactly three distinct support planes appear, and the three planes are
/// pairwise perpendicular. Otherwise returns `None` and the caller keeps
/// its typed refusal.
pub fn qualify_notch(
    edge_faces: &[[usize; 2]],
    outward: &DetHashMap<usize, Vec3>,
    offsets: &DetHashMap<usize, f64>,
    convex: [bool; 3],
    tol: Tolerance,
) -> Option<(NotchFrame, usize)> {
    if edge_faces.len() != 3 {
        return None;
    }
    if convex.iter().filter(|c| !**c).count() != 1 {
        return None;
    }
    let concave_idx = convex.iter().position(|c| !*c)?;
    let convex_idx: Vec<usize> = convex
        .iter()
        .enumerate()
        .filter_map(|(i, c)| c.then_some(i))
        .collect();
    let (ca, cb) = (convex_idx[0], convex_idx[1]);
    let pa = support_planes(&edge_faces[ca], outward, offsets)?;
    let pb = support_planes(&edge_faces[cb], outward, offsets)?;
    let pc = support_planes(&edge_faces[concave_idx], outward, offsets)?;
    // The concave edge's supports must be two distinct planes.
    if same_plane(pc[0], pc[1], tol) {
        return None;
    }
    // Cap: a plane of a convex edge shared with the other convex edge,
    // absent from the concave edge.
    let cap = pa
        .iter()
        .find(|p| edge_has_plane(&pb, **p, tol) && !edge_has_plane(&pc, **p, tol))
        .copied()?;
    // Walls: the concave edge's planes, each paired with its convex edge.
    let (wall_a, wall_b) = if edge_has_plane(&pa, pc[0], tol) && edge_has_plane(&pb, pc[1], tol) {
        (pc[0], pc[1])
    } else if edge_has_plane(&pa, pc[1], tol) && edge_has_plane(&pb, pc[0], tol) {
        (pc[1], pc[0])
    } else {
        return None;
    };
    // Exactly three distinct support planes overall.
    if same_plane(cap, wall_a, tol) || same_plane(cap, wall_b, tol) {
        return None;
    }
    let (cap_n, na, nb) = (cap.normal, wall_a.normal, wall_b.normal);
    for (x, y) in [(cap_n, na), (cap_n, nb), (na, nb)] {
        if x.dot(y).abs() > tol.angular {
            return None;
        }
    }
    Some((
        NotchFrame {
            cap_outward: cap_n,
            wall_a: na,
            wall_b: nb,
        },
        concave_idx,
    ))
}

/// Qualify a mixed-side vertex as a 90-degree rectangular rib-base with a
/// convex singleton.
///
/// Mirror of [`qualify_notch`]: exactly one edge is convex (the singleton,
/// e.g. a rib's vertical edge), the other two concave. The cap is the face
/// shared by both concave edges and absent from the singleton's pair; the
/// walls are the singleton's faces, each paired with its concave edge.
/// Returns the frame plus the singleton (convex) edge index. The
/// concave-singleton pattern is refused here (it belongs to
/// [`qualify_notch`]); the two families are disjoint by construction.
pub fn qualify_convex_singleton(
    edge_faces: &[[usize; 2]],
    outward: &DetHashMap<usize, Vec3>,
    offsets: &DetHashMap<usize, f64>,
    convex: [bool; 3],
    tol: Tolerance,
) -> Option<(NotchFrame, usize)> {
    if edge_faces.len() != 3 {
        return None;
    }
    if convex.iter().filter(|c| **c).count() != 1 {
        return None;
    }
    let singleton_idx = convex.iter().position(|c| *c)?;
    let concave_idx: Vec<usize> = convex
        .iter()
        .enumerate()
        .filter_map(|(i, c)| (!c).then_some(i))
        .collect();
    let (ca, cb) = (concave_idx[0], concave_idx[1]);
    let pa = support_planes(&edge_faces[ca], outward, offsets)?;
    let pb = support_planes(&edge_faces[cb], outward, offsets)?;
    let ps = support_planes(&edge_faces[singleton_idx], outward, offsets)?;
    // The singleton's supports must be two distinct planes.
    if same_plane(ps[0], ps[1], tol) {
        return None;
    }
    // Cap: a plane shared by both concave edges, absent from the singleton.
    let cap = pa
        .iter()
        .find(|p| edge_has_plane(&pb, **p, tol) && !edge_has_plane(&ps, **p, tol))
        .copied()?;
    // Walls: the singleton's planes, each paired with its concave edge.
    let (wall_a, wall_b) = if edge_has_plane(&pa, ps[0], tol) && edge_has_plane(&pb, ps[1], tol) {
        (ps[0], ps[1])
    } else if edge_has_plane(&pa, ps[1], tol) && edge_has_plane(&pb, ps[0], tol) {
        (ps[1], ps[0])
    } else {
        return None;
    };
    // Exactly three distinct support planes overall.
    if same_plane(cap, wall_a, tol) || same_plane(cap, wall_b, tol) {
        return None;
    }
    let (cap_n, na, nb) = (cap.normal, wall_a.normal, wall_b.normal);
    for (x, y) in [(cap_n, na), (cap_n, nb), (na, nb)] {
        if x.dot(y).abs() > tol.angular {
            return None;
        }
    }
    Some((
        NotchFrame {
            cap_outward: cap_n,
            wall_a: na,
            wall_b: nb,
        },
        singleton_idx,
    ))
}

/// Solved torus corner: ring, stations, and seam arcs.
#[derive(Debug, Clone)]
pub struct NotchTorus {
    /// Ring (spine) center.
    pub ring_center: Point3,
    /// Torus carrier surface (major radius `2R`, minor radius `R`).
    pub torus: ToroidalSurface,
    /// Convex-A station contact crossing (on wall A).
    pub m1: Point3,
    /// Convex-B station contact crossing (on wall B).
    pub m2: Point3,
    /// Convex-A station meets cap contact.
    pub b1: Point3,
    /// Convex-B station meets cap contact.
    pub b2: Point3,
    /// Fillet radius.
    pub radius: f64,
    /// Frame used (cap + walls).
    pub frame: NotchFrame,
    /// Convex-singleton mirror (rib base): ring center, stations, and seam
    /// centers flip side with the ring (see [`notch_torus`]); the emitted
    /// loop is oriented CCW about the parametric normal and the assembled
    /// face marked reversed (the outward is negated there). False for the
    /// concave-singleton notch.
    pub mirror: bool,
}

/// Solve the torus corner at `vertex` with fillet radius `radius`.
///
/// `dir_pair_a` / `dir_pair_b` are the unit edge directions from the vertex
/// along the two same-side edges (the convex pair for a notch, the concave
/// pair for a rib mirror); the singleton direction is implied. With
/// `mirror = false` the ring sits in the void (`C2 = V + R(wa+wb) - R·cap`,
/// concave singleton); with `mirror = true` both center terms flip
/// (`C2 = V - R(wa+wb) + R·cap`, convex singleton) and the patch keeps the
/// opposite tube side. Returns `None` for non-positive radius or degenerate
/// input.
#[allow(clippy::too_many_arguments)]
pub fn notch_torus(
    frame: NotchFrame,
    vertex: Point3,
    dir_pair_a: Vec3,
    dir_pair_b: Vec3,
    radius: f64,
    mirror: bool,
    tol: Tolerance,
) -> Option<NotchTorus> {
    if radius <= tol.linear {
        return None;
    }
    let s = if mirror { -1.0 } else { 1.0 };
    // Ring center: one radius along each wall outward plus one radius off
    // the cap (signs flip together for the mirror).
    let ring_center =
        vertex + (frame.wall_a + frame.wall_b) * (s * radius) - frame.cap_outward * (s * radius);
    let torus =
        ToroidalSurface::with_axis(ring_center, 2.0 * radius, radius, frame.cap_outward).ok()?;
    // Contact crossings: each pair stripe's wall-contact meets the
    // singleton's wall-contact one radius along the pair edge from the
    // vertex and one radius off the cap.
    let m1 = vertex + dir_pair_a * radius - frame.cap_outward * (s * radius);
    let m2 = vertex + dir_pair_b * radius - frame.cap_outward * (s * radius);
    // Station/cap-contact points: one radius along the pair edge and one
    // radius off the wall into the material.
    let b1 = vertex + dir_pair_a * radius - frame.wall_a * (s * radius);
    let b2 = vertex + dir_pair_b * radius - frame.wall_b * (s * radius);
    Some(NotchTorus {
        ring_center,
        torus,
        m1,
        m2,
        b1,
        b2,
        radius,
        frame,
        mirror,
    })
}

/// One oriented seam arc of the torus patch loop.
#[derive(Debug, Clone)]
pub struct SeamArc {
    /// Exact 3D circle carrying the arc.
    pub circle: Circle3D,
    /// Arc start point.
    pub start: Point3,
    /// Arc end point.
    pub end: Point3,
}

/// Build a seam arc on the exact circle through `center` with `axis`,
/// spanning `start -> end`. The axis is normalized (non-unit input is
/// refused, never silently mis-validated) and endpoints are checked against
/// the kernel linear tolerance.
fn seam_arc(
    center: Point3,
    axis: Vec3,
    radius: f64,
    start: Point3,
    end: Point3,
    tol: Tolerance,
) -> Option<SeamArc> {
    let axis = axis.normalize().ok()?;
    let circle = Circle3D::new(center, axis, radius).ok()?;
    // Both endpoints must lie on the circle; the assembly trim pins the
    // exact span.
    for p in [start, end] {
        let rel = p - center;
        let axial = rel.dot(axis);
        let radial = (rel - axis * axial).length();
        if axial.abs() > tol.linear || (radial - radius).abs() > tol.linear {
            return None;
        }
    }
    Some(SeamArc { circle, start, end })
}

/// Orient a seam circle for assembly: returns the circle (axis possibly
/// negated) with a CCW trim span covering the short arc `start -> end`.
/// Quarter-circle seams always span less than pi, so the short span is
/// unambiguous; anything else returns `None` (fail-closed, never a chord).
pub fn oriented_seam(
    circle: &Circle3D,
    start: Point3,
    end: Point3,
) -> Option<(Circle3D, (f64, f64))> {
    let normalize_ccw = |circle: &Circle3D| -> Option<(f64, f64)> {
        let a0 = circle.project(start);
        let a1 = circle.project(end);
        // CCW span from a0 into (0, TAU]; rem_euclid folds the wrap-around
        // without a float loop.
        let mut span = (a1 - a0).rem_euclid(std::f64::consts::TAU);
        if span <= 0.0 {
            span += std::f64::consts::TAU;
        }
        if span >= std::f64::consts::PI {
            return None;
        }
        Some((a0, a0 + span))
    };
    if let Some(trim) = normalize_ccw(circle) {
        return Some((circle.clone(), trim));
    }
    // The short arc runs clockwise in this frame: negate the axis so the
    // stored arc is the CCW short span (mirrors the cylinder-arc rule).
    let flipped = Circle3D::new(circle.center(), -circle.normal(), circle.radius()).ok()?;
    normalize_ccw(&flipped).map(|trim| (flipped, trim))
}
/// Split points where a cap-tangency arc must break at coplanar seams.
///
/// `circle`/`trim` is the tangency arc (`trim` a CCW span under pi, as
/// returned by [`oriented_seam`]); `segments` are straight cap-plane seam
/// segments as `(start, end)` pairs. Returns the circle/segment
/// intersections strictly inside both the segments and the arc span,
/// sorted along the arc and deduplicated.
///
/// A tangency arc crossing a coplanar face boundary (a lip fused flush
/// with its plate splits the cap plane in two) must split there, or
/// neither cap face can adopt its half and the shell stays open. Pure
/// geometry for the torus emission; uncrossed arcs return empty and the
/// caller keeps its single span.
pub fn split_tangency_span(
    circle: &Circle3D,
    trim: (f64, f64),
    segments: &[(Point3, Point3)],
    tol: Tolerance,
) -> Vec<Point3> {
    let (t0, t1) = trim;
    let span = t1 - t0;
    if !(span > 0.0 && span < std::f64::consts::PI) {
        return Vec::new();
    }
    let center = circle.center();
    let radius = circle.radius();
    if radius <= tol.linear {
        return Vec::new();
    }
    let mut hits: Vec<(f64, Point3)> = Vec::new();
    for &(p0, p1) in segments {
        let d = p1 - p0;
        let len_sq = d.dot(d);
        if len_sq <= tol.linear * tol.linear {
            continue;
        }
        // |P0 + t·D − C|² = r².
        let oc = p0 - center;
        let a = len_sq;
        let b = 2.0 * d.dot(oc);
        let c = oc.dot(oc) - radius * radius;
        let disc = b * b - 4.0 * a * c;
        if disc < 0.0 {
            continue;
        }
        let root = disc.sqrt();
        for t in [(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)] {
            // Strictly interior: endpoints are already loop vertices, and
            // grazing the seam end keeps the single span.
            let margin = tol.linear / len_sq.sqrt();
            if t <= margin || t >= 1.0 - margin {
                continue;
            }
            let q = p0 + d * t;
            let rel = (circle.project(q) - t0).rem_euclid(std::f64::consts::TAU);
            if rel < tol.linear / radius || rel > span - tol.linear / radius {
                continue;
            }
            hits.push((rel, q));
        }
    }
    hits.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut out: Vec<Point3> = Vec::new();
    for (_, q) in hits {
        if out.iter().all(|p: &Point3| (*p - q).length() > tol.linear) {
            out.push(q);
        }
    }
    out
}
/// pair-A station quarter, cap tangency quarter, pair-B station quarter,
/// singleton station quarter. Each arc is stored in loop-traversal
/// direction. Returns `None` if any arc degenerates. `mirror` selects the
/// rib-base side (station-circle centers and tangency foot flip with the
/// ring); the loop topology and helpers are shared.
pub fn seam_arcs(
    corner: &NotchTorus,
    dir_pair_a: Vec3,
    dir_pair_b: Vec3,
    dir_singleton: Vec3,
    mirror: bool,
    tol: Tolerance,
) -> Option<[SeamArc; 4]> {
    let r = corner.radius;
    if r <= tol.linear {
        return None;
    }
    let s = if mirror { -1.0 } else { 1.0 };
    // T1 station circle: plane perpendicular to the pair-A edge at the
    // station, centered on the stripe axis (M1 pulled one radius off wall A).
    let t1_center = corner.m1 - corner.frame.wall_a * (s * r);
    // T2 station circle: mirror on wall B.
    let t2_center = corner.m2 - corner.frame.wall_b * (s * r);
    // T3 station circle: cap-parallel plane through the ring center.
    let t3_center = corner.ring_center;
    // Cap tangency circle: cap plane through the ring-center foot, radius 2R.
    let foot = corner.ring_center + corner.frame.cap_outward * (s * r);
    let s1 = seam_arc(t1_center, dir_pair_a, r, corner.m1, corner.b1, tol)?;
    let s2 = seam_arc(t2_center, dir_pair_b, r, corner.b2, corner.m2, tol)?;
    let s3 = seam_arc(t3_center, dir_singleton, r, corner.m2, corner.m1, tol)?;
    let s4 = seam_arc(
        foot,
        corner.frame.cap_outward,
        2.0 * r,
        corner.b1,
        corner.b2,
        tol,
    )?;
    Some([s1, s2, s3, s4])
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use super::*;

    /// Bottom notch vertex of the L-bracket, R=1: cap B (face 0), walls S1
    /// (face 1) and S2 (face 2); edges: e1 (B,S1) convex +X, e2 (B,S2)
    /// convex +Y, e3 (S1,S2) concave +Z.
    #[allow(clippy::type_complexity)]
    fn bracket_input() -> (
        [[usize; 2]; 3],
        DetHashMap<usize, Vec3>,
        DetHashMap<usize, f64>,
        [bool; 3],
    ) {
        let mut outward = DetHashMap::default();
        outward.insert(0, Vec3::new(0.0, 0.0, -1.0));
        outward.insert(1, Vec3::new(0.0, 1.0, 0.0));
        outward.insert(2, Vec3::new(1.0, 0.0, 0.0));
        // Offsets n.V for V=(8,8,0).
        let mut offsets = DetHashMap::default();
        offsets.insert(0, 0.0);
        offsets.insert(1, 8.0);
        offsets.insert(2, 8.0);
        (
            [[0, 1], [0, 2], [1, 2]],
            outward,
            offsets,
            [true, true, false],
        )
    }

    #[test]
    fn ring_and_stations_match_reference() {
        let tol = Tolerance::new();
        let (faces, outward, offsets, convex) = bracket_input();
        let (frame, concave_idx) = qualify_notch(&faces, &outward, &offsets, convex, tol).unwrap();
        assert_eq!(concave_idx, 2);
        let nt = notch_torus(
            frame,
            Point3::new(8.0, 8.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            1.0,
            false,
            tol,
        )
        .unwrap();
        for (got, want) in [
            (nt.ring_center, Point3::new(9.0, 9.0, 1.0)),
            (nt.m1, Point3::new(9.0, 8.0, 1.0)),
            (nt.m2, Point3::new(8.0, 9.0, 1.0)),
            (nt.b1, Point3::new(9.0, 7.0, 0.0)),
            (nt.b2, Point3::new(7.0, 9.0, 0.0)),
        ] {
            assert!((got - want).length() < 1e-9, "got {got:?}, want {want:?}");
        }
        assert!((nt.torus.major_radius() - 2.0).abs() < 1e-12);
        assert!((nt.torus.minor_radius() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn seams_are_exact_quarter_circles() {
        let tol = Tolerance::new();
        let (faces, outward, offsets, convex) = bracket_input();
        let (frame, _) = qualify_notch(&faces, &outward, &offsets, convex, tol).unwrap();
        let nt = notch_torus(
            frame,
            Point3::new(8.0, 8.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            1.0,
            false,
            tol,
        )
        .unwrap();
        let [s1, s2, s3, s4] = seam_arcs(
            &nt,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            false,
            tol,
        )
        .unwrap();
        // T1 station: center (9,7,1), axis +X, M1->B1.
        assert!((s1.circle.center() - Point3::new(9.0, 7.0, 1.0)).length() < 1e-9);
        // T3 station: center (9,9,1), axis +Z, M1->M2.
        assert!((s3.circle.center() - Point3::new(9.0, 9.0, 1.0)).length() < 1e-9);
        // Tangency: center (9,9,0), radius 2, B1->B2.
        assert!((s4.circle.center() - Point3::new(9.0, 9.0, 0.0)).length() < 1e-9);
        assert!((s4.circle.radius() - 2.0).abs() < 1e-12);
        let _ = s2;
    }

    #[test]
    fn seams_are_g1_tangent_to_stripes_and_supports() {
        let tol = Tolerance::new();
        let (faces, outward, offsets, convex) = bracket_input();
        let (frame, _) = qualify_notch(&faces, &outward, &offsets, convex, tol).unwrap();
        let nt = notch_torus(
            frame,
            Point3::new(8.0, 8.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            1.0,
            false,
            tol,
        )
        .unwrap();
        // Torus normal at M1 is parallel to wall A (S1 tangency).
        let (u, w) = nt.torus.project_point(nt.m1);
        let n = nt.torus.normal(u, w);
        assert!(
            n.dot(frame.wall_a).abs() > 1.0 - 1e-9,
            "torus normal at M1 must be tangent to S1: {n:?}"
        );
        // Torus normal at B1 is parallel to the cap (resting contact).
        let (u, w) = nt.torus.project_point(nt.b1);
        let n = nt.torus.normal(u, w);
        assert!(
            n.dot(frame.cap_outward).abs() > 1.0 - 1e-9,
            "torus normal at B1 must match the cap: {n:?}"
        );
    }

    #[test]
    fn oriented_seams_span_quarters() {
        let tol = Tolerance::new();
        let (faces, outward, offsets, convex) = bracket_input();
        let (frame, _) = qualify_notch(&faces, &outward, &offsets, convex, tol).unwrap();
        let nt = notch_torus(
            frame,
            Point3::new(8.0, 8.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            1.0,
            false,
            tol,
        )
        .unwrap();
        let [s1, _, _, _] = seam_arcs(
            &nt,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            false,
            tol,
        )
        .unwrap();
        let (circle, (t0, t1)) = oriented_seam(&s1.circle, s1.start, s1.end).unwrap();
        let span = t1 - t0;
        assert!(
            (span - std::f64::consts::FRAC_PI_2).abs() < 1e-9,
            "station quarter must span pi/2, got {span}"
        );
        // Arc midpoint lands on the quarter bisector (independent check).
        let mid_expected = Point3::new(
            9.0,
            7.0 + std::f64::consts::FRAC_1_SQRT_2,
            1.0 - std::f64::consts::FRAC_1_SQRT_2,
        );
        let mid = circle.evaluate((t0 + t1) * 0.5);
        assert!(
            (mid - mid_expected).length() < 1e-9,
            "arc mid {mid:?} must bisect M1->B1, want {mid_expected:?}"
        );
    }

    #[test]
    fn qualify_accepts_concave_singleton_rectangular_only() {
        let tol = Tolerance::new();
        let (faces, outward, offsets, _) = bracket_input();
        // Concave singleton (e3) qualifies, whichever edge is concave.
        assert!(qualify_notch(&faces, &outward, &offsets, [true, true, false], tol).is_some());
        assert!(qualify_notch(&faces, &outward, &offsets, [false, true, true], tol).is_some());
        assert!(qualify_notch(&faces, &outward, &offsets, [true, false, true], tol).is_some());
        // All-convex, all-concave, two-concave (convex singleton): refused
        // here — the convex singleton is the mirror family's pattern (see
        // qualify_convex_singleton); the two families are disjoint.
        assert!(qualify_notch(&faces, &outward, &offsets, [true, true, true], tol).is_none());
        assert!(qualify_notch(&faces, &outward, &offsets, [false, false, false], tol).is_none());
        assert!(qualify_notch(&faces, &outward, &offsets, [true, false, false], tol).is_none());
        // The notch pattern itself is refused by the mirror qualifier.
        assert!(
            qualify_convex_singleton(&faces, &outward, &offsets, [true, true, false], tol)
                .is_none()
        );
        // Wrong edge count: refused.
        assert!(qualify_notch(&faces[..2], &outward, &offsets, [true, true, false], tol).is_none());
        // Oblique walls: refused.
        let mut oblique = outward.clone();
        oblique.insert(2, Vec3::new(1.0, 1.0, 0.0).normalize().unwrap());
        assert!(qualify_notch(&faces, &oblique, &offsets, [true, true, false], tol).is_none());
        // Shared-face degeneracy (all edges on two faces): refused.
        let degenerate = [[0usize, 1], [0, 1], [0, 1]];
        assert!(qualify_notch(&degenerate, &outward, &offsets, [true, true, false], tol).is_none());
        // Coplanar fuse split: the cap arrives as two faces (0 and 3) on
        // one logical plane — still qualifies with the same frame.
        let mut outward_split = outward;
        outward_split.insert(3, Vec3::new(0.0, 0.0, -1.0));
        let mut offsets_split = offsets;
        offsets_split.insert(3, 0.0);
        let split_faces = [[0usize, 1], [3, 2], [1, 2]];
        let (split_frame, _) = qualify_notch(
            &split_faces,
            &outward_split,
            &offsets_split,
            [true, true, false],
            tol,
        )
        .unwrap();
        assert!((split_frame.cap_outward - Vec3::new(0.0, 0.0, -1.0)).length() < 1e-9);
        // Parallel-but-offset splits are distinct planes: refused.
        let mut offsets_shifted = offsets_split.clone();
        offsets_shifted.insert(3, 5.0);
        assert!(
            qualify_notch(
                &split_faces,
                &outward_split,
                &offsets_shifted,
                [true, true, false],
                tol
            )
            .is_none()
        );
    }

    /// Rib-base vertex V=(12,10,6), R=1: cap P (face 0, z=6), wall A
    /// (face 1, x=12), wall B (face 2, y=10); edges: a (P,A) concave +Y,
    /// b (P,B) concave +X, c (A,B) convex +Z (the singleton).
    #[allow(clippy::type_complexity)]
    fn rib_input() -> (
        [[usize; 2]; 3],
        DetHashMap<usize, Vec3>,
        DetHashMap<usize, f64>,
        [bool; 3],
    ) {
        let mut outward = DetHashMap::default();
        outward.insert(0, Vec3::new(0.0, 0.0, 1.0));
        outward.insert(1, Vec3::new(-1.0, 0.0, 0.0));
        outward.insert(2, Vec3::new(0.0, -1.0, 0.0));
        // Offsets n.V for V=(12,10,6).
        let mut offsets = DetHashMap::default();
        offsets.insert(0, 6.0);
        offsets.insert(1, -12.0);
        offsets.insert(2, -10.0);
        (
            [[0, 1], [0, 2], [1, 2]],
            outward,
            offsets,
            [false, false, true],
        )
    }

    #[test]
    fn qualify_accepts_convex_singleton_rectangular_only() {
        let tol = Tolerance::new();
        let (faces, outward, offsets, _) = rib_input();
        // Convex singleton qualifies, whichever edge is convex.
        assert!(
            qualify_convex_singleton(&faces, &outward, &offsets, [false, false, true], tol)
                .is_some()
        );
        assert!(
            qualify_convex_singleton(&faces, &outward, &offsets, [true, false, false], tol)
                .is_some()
        );
        assert!(
            qualify_convex_singleton(&faces, &outward, &offsets, [false, true, false], tol)
                .is_some()
        );
        // All-convex, all-concave, two-convex (concave singleton): refused
        // here — the concave singleton belongs to qualify_notch.
        assert!(
            qualify_convex_singleton(&faces, &outward, &offsets, [true, true, true], tol).is_none()
        );
        assert!(
            qualify_convex_singleton(&faces, &outward, &offsets, [false, false, false], tol)
                .is_none()
        );
        assert!(
            qualify_convex_singleton(&faces, &outward, &offsets, [false, true, true], tol)
                .is_none()
        );
        // Wrong edge count: refused.
        assert!(
            qualify_convex_singleton(&faces[..2], &outward, &offsets, [false, false, true], tol)
                .is_none()
        );
        // Oblique walls: refused.
        let mut oblique = outward.clone();
        oblique.insert(2, Vec3::new(1.0, 1.0, 0.0).normalize().unwrap());
        assert!(
            qualify_convex_singleton(&faces, &oblique, &offsets, [false, false, true], tol)
                .is_none()
        );
        // Shared-face degeneracy (all edges on two faces): refused.
        let degenerate = [[0usize, 1], [0, 1], [0, 1]];
        assert!(
            qualify_convex_singleton(&degenerate, &outward, &offsets, [false, false, true], tol)
                .is_none()
        );
    }

    #[test]
    fn mirror_ring_and_stations_match_reference() {
        let tol = Tolerance::new();
        let (faces, outward, offsets, convex) = rib_input();
        let (frame, singleton_idx) =
            qualify_convex_singleton(&faces, &outward, &offsets, convex, tol).unwrap();
        assert_eq!(singleton_idx, 2);
        let nt = notch_torus(
            frame,
            Point3::new(12.0, 10.0, 6.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
            true,
            tol,
        )
        .unwrap();
        assert!(nt.mirror);
        // Independent-reference pins: C2'=(13,11,7), major 2, tube 1.
        for (got, want) in [
            (nt.ring_center, Point3::new(13.0, 11.0, 7.0)),
            (nt.m1, Point3::new(12.0, 11.0, 7.0)),
            (nt.m2, Point3::new(13.0, 10.0, 7.0)),
            (nt.b1, Point3::new(11.0, 11.0, 6.0)),
            (nt.b2, Point3::new(13.0, 9.0, 6.0)),
        ] {
            assert!((got - want).length() < 1e-9, "got {got:?}, want {want:?}");
        }
        assert!((nt.torus.major_radius() - 2.0).abs() < 1e-12);
        assert!((nt.torus.minor_radius() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn tangency_split_finds_coplanar_seam_crossing() {
        let tol = Tolerance::new();
        // Lip S4': foot (0,41,7), r=2, B1=(0,41,5) -> B2=(0,43,7).
        let circle =
            Circle3D::new(Point3::new(0.0, 41.0, 7.0), Vec3::new(-1.0, 0.0, 0.0), 2.0).unwrap();
        let b1 = Point3::new(0.0, 41.0, 5.0);
        let b2 = Point3::new(0.0, 43.0, 7.0);
        let (arc, trim) = oriented_seam(&circle, b1, b2).unwrap();
        // Coplanar seam (plate-west/lip-west boundary) through the disk.
        let seam = [(Point3::new(0.0, 42.0, 6.0), Point3::new(0.0, 50.0, 6.0))];
        let qs = split_tangency_span(&arc, trim, &seam, tol);
        assert_eq!(qs.len(), 1, "one crossing expected, got {qs:?}");
        let want = Point3::new(0.0, 41.0 + 3.0_f64.sqrt(), 6.0);
        assert!(
            (qs[0] - want).length() < 1e-9,
            "Q must be the arc/seam crossing, got {:?}",
            qs[0]
        );
        // A seam missing the disk splits nothing.
        let far = [(Point3::new(0.0, 60.0, 6.0), Point3::new(0.0, 70.0, 6.0))];
        assert!(split_tangency_span(&arc, trim, &far, tol).is_empty());
        // A segment ending at the arc endpoint keeps the single span.
        let touch = [(Point3::new(0.0, 41.0, 5.0), Point3::new(0.0, 41.0, 0.0))];
        assert!(split_tangency_span(&arc, trim, &touch, tol).is_empty());
    }

    #[test]
    fn mirror_seams_are_exact_quarter_circles() {
        let tol = Tolerance::new();
        let (faces, outward, offsets, convex) = rib_input();
        let (frame, _) = qualify_convex_singleton(&faces, &outward, &offsets, convex, tol).unwrap();
        let nt = notch_torus(
            frame,
            Point3::new(12.0, 10.0, 6.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
            true,
            tol,
        )
        .unwrap();
        let [s1, _, s3, s4] = seam_arcs(
            &nt,
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            true,
            tol,
        )
        .unwrap();
        // T1 station: center (11,11,7), axis +Y, M1'->B1'.
        assert!((s1.circle.center() - Point3::new(11.0, 11.0, 7.0)).length() < 1e-9);
        // T3 station: center (13,11,7), axis +Z (the convex singleton).
        assert!((s3.circle.center() - Point3::new(13.0, 11.0, 7.0)).length() < 1e-9);
        // Tangency: center (13,11,6), radius 2, B1'->B2'.
        assert!((s4.circle.center() - Point3::new(13.0, 11.0, 6.0)).length() < 1e-9);
        assert!((s4.circle.radius() - 2.0).abs() < 1e-12);
        // Every seam is a quarter turn (the volume route's patch signature).
        for s in [&s1, &s3, &s4] {
            let (_, (t0, t1)) = oriented_seam(&s.circle, s.start, s.end).unwrap();
            assert!(
                (t1 - t0 - std::f64::consts::FRAC_PI_2).abs() < 1e-9,
                "mirror seams must span pi/2"
            );
        }
    }
}
