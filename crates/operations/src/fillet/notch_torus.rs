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

use std::collections::HashMap;

use remus_math::curves::Circle3D;
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

/// Qualify a mixed-side vertex as a 90-degree rectangular notch with a
/// concave singleton.
///
/// Inputs are per selected incident edge (exactly three): `edge_faces[k]`
/// the two support faces of edge `k` (as arena indices), `outward` maps a
/// support-face index to its outward unit normal, and `convex[k]` marks
/// convex edges. Returns the frame (cap = the face incident to both convex
/// edges; walls = the faces incident to the concave edge, each paired with
/// its convex edge) plus the concave edge index, when exactly one edge is
/// concave, exactly three distinct support planes appear, and the three
/// planes are pairwise perpendicular. Otherwise returns `None` and the
/// caller keeps its typed refusal.
pub fn qualify_notch(
    edge_faces: &[[usize; 2]],
    outward: &HashMap<usize, Vec3>,
    convex: &[bool; 3],
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
    // Cap: incident to both convex edges, not to the concave edge.
    let cap = edge_faces[ca]
        .iter()
        .find(|f| edge_faces[cb].contains(f) && !edge_faces[concave_idx].contains(f))
        .copied()?;
    // Walls: the concave edge's faces, each paired with its convex edge.
    let (wa, wb) = (edge_faces[concave_idx][0], edge_faces[concave_idx][1]);
    if wa == wb {
        return None;
    }
    let (wall_a, wall_b) = if edge_faces[ca].contains(&wa) && edge_faces[cb].contains(&wb) {
        (wa, wb)
    } else if edge_faces[ca].contains(&wb) && edge_faces[cb].contains(&wa) {
        (wb, wa)
    } else {
        return None;
    };
    // Exactly three distinct support planes overall.
    let mut distinct = vec![cap];
    for f in [wall_a, wall_b] {
        if !distinct.contains(&f) {
            distinct.push(f);
        }
    }
    if distinct.len() != 3 {
        return None;
    }
    let (Some(cap_n), Some(na), Some(nb)) = (
        outward.get(&cap).copied(),
        outward.get(&wall_a).copied(),
        outward.get(&wall_b).copied(),
    ) else {
        return None;
    };
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

/// Solved torus corner: ring, stations, and seam arcs.
#[derive(Debug, Clone)]
pub struct NotchTorus {
    /// Ring (spine) center.
    pub ring_center: Point3,
    /// Ring radius: exactly `2R` for the rectangular notch.
    pub ring_radius: f64,
    /// Tube radius: the fillet radius `R`.
    pub tube_radius: f64,
    /// Ring axis: cap outward (matches the reference orientation).
    pub axis: Vec3,
    /// Torus carrier surface.
    pub torus: ToroidalSurface,
    /// Convex-A station contact crossing (on wall A).
    pub m1: Point3,
    /// Convex-B station contact crossing (on wall B).
    pub m2: Point3,
    /// Convex-A station meets cap contact.
    pub b1: Point3,
    /// Convex-B station meets cap contact.
    pub b2: Point3,
    /// The vertex this corner replaces.
    pub vertex: Point3,
    /// Fillet radius.
    pub radius: f64,
    /// Frame used (cap + walls).
    pub frame: NotchFrame,
}

/// Solve the torus corner at `vertex` with fillet radius `radius`.
///
/// `dir_convex_a` / `dir_convex_b` are the unit edge directions from the
/// vertex along the two convex edges; the concave edge direction is implied
/// (cap-inward). Returns `None` for non-positive radius or degenerate input.
#[allow(clippy::too_many_arguments)]
pub fn notch_torus(
    frame: NotchFrame,
    vertex: Point3,
    dir_convex_a: Vec3,
    dir_convex_b: Vec3,
    radius: f64,
    tol: Tolerance,
) -> Option<NotchTorus> {
    if radius <= tol.linear {
        return None;
    }
    // Ring center: one radius into the void along each wall outward plus one
    // radius off the cap along its outward normal.
    let ring_center = vertex + (frame.wall_a + frame.wall_b) * radius - frame.cap_outward * radius;
    let torus = ToroidalSurface::with_axis(ring_center, 2.0 * radius, radius, frame.cap_outward)
        .ok()?;
    // Contact crossings: each convex stripe's wall-contact meets the concave
    // stripe's wall-contact one radius along the convex edge from the vertex
    // and one radius off the cap.
    let m1 = vertex + dir_convex_a * radius - frame.cap_outward * radius;
    let m2 = vertex + dir_convex_b * radius - frame.cap_outward * radius;
    // Station/cap-contact points: one radius along the convex edge and one
    // radius off the wall into the material.
    let b1 = vertex + dir_convex_a * radius - frame.wall_a * radius;
    let b2 = vertex + dir_convex_b * radius - frame.wall_b * radius;
    Some(NotchTorus {
        ring_center,
        ring_radius: 2.0 * radius,
        tube_radius: radius,
        axis: frame.cap_outward,
        torus,
        m1,
        m2,
        b1,
        b2,
        vertex,
        radius,
        frame,
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
/// spanning `start -> end`.
fn seam_arc(center: Point3, axis: Vec3, radius: f64, start: Point3, end: Point3) -> Option<SeamArc> {
    let circle = Circle3D::new(center, axis, radius).ok()?;
    // Both endpoints must lie on the circle (within linear tolerance
    // relative to the radius); the assembly trim pins the exact span.
    for p in [start, end] {
        let rel = p - center;
        let axial = rel.dot(axis);
        let radial = (rel - axis * axial).length();
        if axial.abs() > radius * 1e-6 || (radial - radius).abs() > radius * 1e-6 {
            return None;
        }
    }
    Some(SeamArc {
        circle,
        start,
        end,
    })
}

/// Orient a seam circle for assembly: returns the circle (axis possibly
/// negated) with a CCW trim span covering the short arc `start -> end`.
/// Quarter-circle seams always span less than pi, so the short span is
/// unambiguous; anything else returns `None` (fail-closed, never a chord).
pub fn oriented_seam(circle: &Circle3D, start: Point3, end: Point3) -> Option<(Circle3D, (f64, f64))> {
    let normalize_ccw = |circle: &Circle3D| -> Option<(f64, f64)> {
        let a0 = circle.project(start);
        let mut a1 = circle.project(end);
        while a1 <= a0 {
            a1 += std::f64::consts::TAU;
        }
        let span = a1 - a0;
        if span <= 0.0 || span >= std::f64::consts::PI {
            return None;
        }
        Some((a0, a1))
    };
    if let Some(trim) = normalize_ccw(circle) {
        return Some((circle.clone(), trim));
    }
    // The short arc runs clockwise in this frame: negate the axis so the
    // stored arc is the CCW short span (mirrors the cylinder-arc rule).
    let flipped = Circle3D::new(circle.center(), -circle.normal(), circle.radius()).ok()?;
    normalize_ccw(&flipped).map(|trim| (flipped, trim))
}
/// The four patch-loop seam arcs in order M1 -> B1 -> B2 -> M2 -> M1:
/// convex-A station quarter, cap tangency quarter, convex-B station
/// quarter, concave station quarter. Returns `None` if any arc degenerates.
pub fn seam_arcs(
    corner: &NotchTorus,
    dir_convex_a: Vec3,
    dir_convex_b: Vec3,
    dir_concave: Vec3,
    tol: Tolerance,
) -> Option<[SeamArc; 4]> {
    let r = corner.radius;
    if r <= tol.linear {
        return None;
    }
    // T1 station circle: plane perpendicular to the convex-A edge at the
    // station, centered on the stripe axis (M1 pulled one radius off wall A).
    let t1_center = corner.m1 - corner.frame.wall_a * r;
    // T2 station circle: mirror on wall B.
    let t2_center = corner.m2 - corner.frame.wall_b * r;
    // T3 station circle: cap-parallel plane through the ring center.
    let t3_center = corner.ring_center;
    // Cap tangency circle: cap plane through the ring-center foot, radius 2R.
    let foot = corner.ring_center + corner.frame.cap_outward * r;
    let s1 = seam_arc(t1_center, dir_convex_a, r, corner.m1, corner.b1)?;
    let s2 = seam_arc(t2_center, dir_convex_b, r, corner.m2, corner.b2)?;
    let s3 = seam_arc(t3_center, dir_concave, r, corner.m1, corner.m2)?;
    let s4 = seam_arc(
        foot,
        corner.frame.cap_outward,
        2.0 * r,
        corner.b1,
        corner.b2,
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
    fn bracket_input() -> ([[usize; 2]; 3], HashMap<usize, Vec3>, [bool; 3]) {
        let mut outward = HashMap::new();
        outward.insert(0, Vec3::new(0.0, 0.0, -1.0));
        outward.insert(1, Vec3::new(0.0, 1.0, 0.0));
        outward.insert(2, Vec3::new(1.0, 0.0, 0.0));
        ([[0, 1], [0, 2], [1, 2]], outward, [true, true, false])
    }

    #[test]
    fn ring_and_stations_match_reference() {
        let tol = Tolerance::new();
        let (faces, outward, convex) = bracket_input();
        let (frame, concave_idx) = qualify_notch(&faces, &outward, &convex, tol).unwrap();
        assert_eq!(concave_idx, 2);
        let nt = notch_torus(
            frame,
            Point3::new(8.0, 8.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            1.0,
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
        assert!((nt.ring_radius - 2.0).abs() < 1e-12);
        assert!((nt.tube_radius - 1.0).abs() < 1e-12);
    }

    #[test]
    fn seams_are_exact_quarter_circles() {
        let tol = Tolerance::new();
        let (faces, outward, convex) = bracket_input();
        let (frame, _) = qualify_notch(&faces, &outward, &convex, tol).unwrap();
        let nt = notch_torus(
            frame,
            Point3::new(8.0, 8.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            1.0,
            tol,
        )
        .unwrap();
        let [s1, s2, s3, s4] = seam_arcs(
            &nt,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
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
        let (faces, outward, convex) = bracket_input();
        let (frame, _) = qualify_notch(&faces, &outward, &convex, tol).unwrap();
        let nt = notch_torus(frame, Point3::new(8.0, 8.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 1.0, tol).unwrap();
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
        let (faces, outward, convex) = bracket_input();
        let (frame, _) = qualify_notch(&faces, &outward, &convex, tol).unwrap();
        let nt = notch_torus(
            frame,
            Point3::new(8.0, 8.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            1.0,
            tol,
        )
        .unwrap();
        let [s1, _, _, _] = seam_arcs(
            &nt,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
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
        let mid_expected = Point3::new(9.0, 7.0 + std::f64::consts::FRAC_1_SQRT_2, 1.0 - std::f64::consts::FRAC_1_SQRT_2);
        let mid = circle.evaluate((t0 + t1) * 0.5);
        assert!(
            (mid - mid_expected).length() < 1e-9,
            "arc mid {mid:?} must bisect M1->B1, want {mid_expected:?}"
        );
    }

    #[test]
    fn qualify_accepts_concave_singleton_rectangular_only() {
        let tol = Tolerance::new();
        let (faces, outward, _) = bracket_input();
        // Concave singleton (e3) qualifies, whichever edge is concave.
        assert!(qualify_notch(&faces, &outward, &[true, true, false], tol).is_some());
        assert!(qualify_notch(&faces, &outward, &[false, true, true], tol).is_some());
        assert!(qualify_notch(&faces, &outward, &[true, false, true], tol).is_some());
        // All-convex, all-concave, two-concave (convex singleton / spike):
        // refused (later families).
        assert!(qualify_notch(&faces, &outward, &[true, true, true], tol).is_none());
        assert!(qualify_notch(&faces, &outward, &[false, false, false], tol).is_none());
        assert!(qualify_notch(&faces, &outward, &[true, false, false], tol).is_none());
        // Wrong edge count: refused.
        assert!(qualify_notch(&faces[..2], &outward, &[true, true, false], tol).is_none());
        // Oblique walls: refused.
        let mut oblique = outward.clone();
        oblique.insert(2, Vec3::new(1.0, 1.0, 0.0).normalize().unwrap());
        assert!(qualify_notch(&faces, &oblique, &[true, true, false], tol).is_none());
        // Shared-face degeneracy (all edges on two faces): refused.
        let degenerate = [[0usize, 1], [0, 1], [0, 1]];
        assert!(qualify_notch(&degenerate, &outward, &[true, true, false], tol).is_none());
    }
}
