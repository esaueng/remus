//! A cap ball wedged where a sampled concave canal hands over to its torus.
//!
//! A joined cap rolls its ball, centre in the cap's inward parallel plane,
//! against each retained support in turn. Where a concave canal (a seam
//! round turning into the boss shoulder) meets the shoulder torus at a
//! contour node, the canal's spine can bend more tightly than the ball and
//! canal radii together. The ball's centre contour on the canal then runs on
//! past the torus band's centre circle and folds back to the node: the balls
//! beyond the crossing dig into the shoulder, and those on the torus band
//! between the node and the crossing dig into the canal. The two bands
//! overlap there, and their meshes fold over the cap.
//!
//! The ball cannot roll past the crossing: there it touches the cap, the
//! canal and the torus at once. The canal band stops at that wedged ball
//! and the torus band starts at it. Between the two contacts the supports
//! stand clear of the ball by a few hundredths of the radius at most, so no
//! rolling ball of the cap's radius bridges them. The canal band carries on
//! instead through sections of the same radius from the cap point of the
//! wedged ball to the supports' points nearest its centre, section by
//! section: a collapsed cap side (a pole at the cap point) above a contact
//! that crosses from the canal onto the torus. Those sections are not
//! rolling sections, so the reconstruction stays sampled.

use std::f64::consts::{PI, TAU};

use remus_math::compact_sort::CompactSort;
use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};

use super::{CanalNormal, CapContour, Node, edge_domain, edge_point, height, refusal};
use crate::BlendError;
use crate::section::CircSection;

/// Contact samples on each side of the junction between the wedged
/// ball's two contacts.
const CANAL_TRIM_SAMPLES: usize = 48;
const TORUS_TRIM_SAMPLES: usize = 16;
/// Pieces each crossing segment is resampled into per refinement.
const PIECES: usize = 16;

/// A wedged cap ball at one contour node and the trim it leaves.
pub(super) struct Wedge {
    /// The canal support.
    pub(super) canal: FaceId,
    /// The axial torus support.
    pub(super) torus: FaceId,
    /// The wedged ball's centre.
    pub(super) centre: Point3,
    /// Its contact on the torus, where the torus band starts.
    pub(super) torus_contact: Point3,
    /// The node edge's parameter where the trim crosses it.
    pub(super) junction: f64,
    /// The trim's point on the node edge.
    pub(super) junction_point: Point3,
    /// The trim on the canal, from the canal contact to the junction.
    pub(super) canal_trim: Vec<Point3>,
    /// The trim on the torus, from the junction to the torus contact.
    pub(super) torus_trim: Vec<Point3>,
}

impl Wedge {
    /// The sections that carry the canal band on past the wedged ball,
    /// from its canal contact to its torus contact: arcs of the cap radius
    /// from each trim point up to the ball's cap point, centred as near the
    /// ball's centre as that radius allows.
    pub(super) fn sections(&self, axis: Vec3, radius: f64) -> Result<Vec<CircSection>, BlendError> {
        let top = self.centre + axis * radius;
        self.canal_trim
            .iter()
            .chain(self.torus_trim.iter().skip(1))
            .map(|&point| section_through(point, top, self.centre, radius))
            .collect()
    }
}

fn section_through(
    point: Point3,
    top: Point3,
    near: Point3,
    radius: f64,
) -> Result<CircSection, BlendError> {
    let chord = top - point;
    let half = chord.length() * 0.5;
    if !(half > 0.0 && half < radius * (1.0 - 1e-9)) {
        return Err(refusal("wedged cap section does not span its trim"));
    }
    let across = chord.normalize()?;
    let middle = point + chord * 0.5;
    let towards = near - middle;
    let offset = towards - across * towards.dot(across);
    let rise = (radius * radius - half * half).sqrt();
    let center = middle + offset.normalize()? * rise;
    Ok(CircSection {
        p1: point,
        p2: top,
        center,
        radius,
        uv1: (0.0, 0.0),
        uv2: (0.0, 0.0),
        t: 0.0,
    })
}

fn wrap(angle: f64) -> f64 {
    (angle + PI).rem_euclid(TAU) - PI
}

/// The minimum of `f` on `[lo, hi]` by golden-section search: its argument
/// and value. `f` must be unimodal there.
fn minimum(
    mut lo: f64,
    mut hi: f64,
    mut f: impl FnMut(f64) -> Result<f64, BlendError>,
) -> Result<(f64, f64), BlendError> {
    const RATIO: f64 = 0.618_033_988_749_894_9;
    let mut a = hi - (hi - lo) * RATIO;
    let mut b = lo + (hi - lo) * RATIO;
    let mut fa = f(a)?;
    let mut fb = f(b)?;
    for _ in 0..100 {
        if fa <= fb {
            hi = b;
            b = a;
            fb = fa;
            a = hi - (hi - lo) * RATIO;
            fa = f(a)?;
        } else {
            lo = a;
            a = b;
            fa = fb;
            b = lo + (hi - lo) * RATIO;
            fb = f(b)?;
        }
    }
    let x = f64::midpoint(lo, hi);
    Ok((x, f(x)?))
}

/// The point of each section of a sampled canal nearest `target`, and its
/// distance; `v` names the section.
fn section_nearest(
    surface: &remus_math::nurbs::surface::NurbsSurface,
    v: f64,
    target: Point3,
) -> Result<(Point3, f64), BlendError> {
    let mut best = (0.0, f64::INFINITY);
    for i in 0..=32 {
        let u = f64::from(i) / 32.0;
        let distance = (surface.evaluate(u, v) - target).length();
        if distance < best.1 {
            best = (u, distance);
        }
    }
    let (u, distance) = minimum(
        (best.0 - 1.0 / 32.0).max(0.0),
        (best.0 + 1.0 / 32.0).min(1.0),
        |u| Ok((surface.evaluate(u, v) - target).length()),
    )?;
    Ok((surface.evaluate(u, v), distance))
}

/// The wedged cap ball at `node`, if the node joins a sampled concave
/// canal to an axial concave torus and the canal band's centre contour,
/// rolled from the canal's `other` node, leaves the torus band's centre
/// circle before it reaches `node`.
///
/// # Errors
/// Refuses a wedge whose contacts or trim cannot be resolved.
pub(super) fn wedged_ball(
    topo: &Topology,
    node: &Node,
    other: &Node,
    contour: CapContour,
) -> Result<Option<Wedge>, BlendError> {
    let CapContour {
        axis,
        level,
        radius,
        ..
    } = contour;
    let concave_torus = |face: FaceId| -> Result<Option<ToroidalSurface>, BlendError> {
        let data = topo.face(face)?;
        Ok(match data.surface() {
            FaceSurface::Torus(torus)
                if torus.z_axis().dot(axis) > 1.0 - 1e-10 && data.is_reversed() =>
            {
                Some(torus.clone())
            }
            FaceSurface::Plane { .. }
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_)
            | FaceSurface::Nurbs(_) => None,
        })
    };
    let nurbs = |face: FaceId| -> Result<bool, BlendError> {
        Ok(matches!(topo.face(face)?.surface(), FaceSurface::Nurbs(_)))
    };
    let [first, second] = node.faces;
    let (torus_face, torus, canal_face) = if let Some(torus) = concave_torus(first)?
        && nurbs(second)?
    {
        (first, torus, second)
    } else if let Some(torus) = concave_torus(second)?
        && nurbs(first)?
    {
        (second, torus, first)
    } else {
        return Ok(None);
    };
    if !other.faces.contains(&canal_face) {
        return Err(refusal("wedged cap canal does not reach its other node"));
    }
    let canal_data = topo.face(canal_face)?;
    // Only a strictly read concave canal has one radius and sections that
    // are its own normals' circles.
    let Ok(field) = CanalNormal::new(canal_data) else {
        return Ok(None);
    };
    if field.sign > 0.0 {
        return Ok(None);
    }
    let surface = field.surface;

    // The torus band's centre circle, as `analytic_sections` rolls it.
    let offset = torus.minor_radius() + radius;
    let dz = level - height(torus.center(), axis);
    if dz >= 0.0 || dz <= -offset {
        return Ok(None);
    }
    let radial = -(offset * offset - dz * dz).sqrt();
    let contact_v = dz.atan2(radial).rem_euclid(TAU);
    let major = torus.major_radius() + radial;
    if major <= 0.0 {
        return Ok(None);
    }
    let base = torus.center() + axis * dz;
    // How far a centre stands outside the circle, about the torus axis.
    let outside = |centre: Point3| {
        let delta = centre - base;
        (delta - axis * delta.dot(axis)).length() - major
    };

    // The canal band's contour from the other node to this one. A centre
    // beyond the circle is wedged: its ball digs into the torus.
    let sections = super::nurbs_sections(topo, canal_face, other, node, contour)?;
    let tolerance = radius * 1e-6;
    let side = sections
        .first()
        .map_or(0.0, |section| outside(section.center).signum());
    let Some(beyond) = sections
        .iter()
        .position(|section| outside(section.center) * side < -tolerance)
    else {
        return Ok(None);
    };
    if beyond == 0 || beyond + 1 >= sections.len() {
        return Err(refusal("wedged cap contour leaves the torus at its ends"));
    }
    let (mut lo, mut hi) = (sections[beyond - 1].uv1.0, sections[beyond].uv1.0);
    for _ in 0..60 {
        let middle = f64::midpoint(lo, hi);
        let (_, centre, _) = super::contour_root(&field, middle, contour)?;
        if outside(centre) * side < 0.0 {
            hi = middle;
        } else {
            lo = middle;
        }
    }
    let (canal_contact, centre, (_, contact_v_canal)) =
        super::contour_root(&field, f64::midpoint(lo, hi), contour)?;
    let angle = {
        let delta = centre - base;
        delta.dot(torus.y_axis()).atan2(delta.dot(torus.x_axis()))
    };
    // Put the centre on the circle exactly; the canal contact then stands
    // one radius from it to the contour's own precision.
    let centre = base + (torus.x_axis() * angle.cos() + torus.y_axis() * angle.sin()) * major;
    if ((canal_contact - centre).length() - radius).abs() > radius * 1e-6 {
        return Err(refusal("wedged cap ball misses its canal contact"));
    }
    let torus_contact = torus.evaluate(angle, contact_v);

    // Where the trim crosses the node edge: its point nearest the centre.
    let (t0, t1) = edge_domain(topo, node.edge)?;
    let mut best = (t0, f64::INFINITY);
    for i in 0..=256 {
        let t = t0 + (t1 - t0) * f64::from(i) / 256.0;
        let distance = (edge_point(topo, node.edge, t)? - centre).length();
        if distance < best.1 {
            best = (t, distance);
        }
    }
    let span = (t1 - t0).abs() / 256.0;
    let (junction, _) = minimum(
        (best.0 - span).max(t0.min(t1)),
        (best.0 + span).min(t0.max(t1)),
        |t| Ok((edge_point(topo, node.edge, t)? - centre).length()),
    )?;
    let junction_point = edge_point(topo, node.edge, junction)?;

    // The canal trim: each section's point nearest the centre, from the
    // section through the canal contact to the node's boundary section.
    let Some(last) = sections.last() else {
        return Err(refusal("wedged cap contour is empty"));
    };
    let junction_v = last.uv1.1;
    if junction_v.abs() > 1e-6 && (junction_v - 1.0).abs() > 1e-6 {
        return Err(refusal("wedged cap node is not on a canal section"));
    }
    let junction_v = junction_v.round();
    let mut canal_trim = Vec::with_capacity(CANAL_TRIM_SAMPLES + 1);
    for i in 0..=CANAL_TRIM_SAMPLES {
        let fraction = i as f64 / CANAL_TRIM_SAMPLES as f64;
        let v = contact_v_canal + (junction_v - contact_v_canal) * fraction;
        canal_trim.push(section_nearest(surface, v, centre)?.0);
    }
    let end = canal_trim.len() - 1;
    if (canal_trim[0] - canal_contact).length() > radius * 1e-5
        || (canal_trim[end] - junction_point).length() > radius * 1e-5
    {
        return Err(refusal("wedged cap trim leaves its canal contacts"));
    }
    canal_trim[0] = canal_contact;
    canal_trim[end] = junction_point;

    // The torus trim: each meridian's point nearest the centre.
    let junction_u = torus.project_point(junction_point).0;
    let sweep = wrap(angle - junction_u);
    let mut torus_trim = Vec::with_capacity(TORUS_TRIM_SAMPLES + 1);
    for i in 0..=TORUS_TRIM_SAMPLES {
        let u = junction_u + sweep * (i as f64 / TORUS_TRIM_SAMPLES as f64);
        let direction = torus.x_axis() * u.cos() + torus.y_axis() * u.sin();
        let spine = torus.center() + direction * torus.major_radius();
        let towards = centre - spine;
        let tangent = axis.cross(direction);
        let in_plane = towards - tangent * towards.dot(tangent);
        torus_trim.push(spine + in_plane.normalize()? * torus.minor_radius());
    }
    let end = torus_trim.len() - 1;
    if (torus_trim[0] - junction_point).length() > radius * 1e-5
        || (torus_trim[end] - torus_contact).length() > radius * 1e-5
    {
        return Err(refusal("wedged cap trim leaves its torus contacts"));
    }
    torus_trim[0] = junction_point;
    torus_trim[end] = torus_contact;
    Ok(Some(Wedge {
        canal: canal_face,
        torus: torus_face,
        centre,
        torus_contact,
        junction,
        junction_point,
        canal_trim,
        torus_trim,
    }))
}

/// Remove the loops a canal band's centre contour makes across itself.
///
/// Where the canal's spine turns about as tightly as the ball and canal
/// radii together, the contour's centre path can run on, turn back and
/// cross its own earlier course before it reaches a node: the band then
/// folds over itself and over the cap. The ball at the crossing touches
/// the canal twice and stands clear of it in between, so the band stops
/// rolling there; sections of its radius from that ball's cap point to the
/// canal's points nearest its centre carry it from the first contact to
/// the second, as at a wedge against a torus.
///
/// # Errors
/// Refuses a crossing that cannot be resolved on the carrier.
pub(super) fn untangle(
    field: &CanalNormal<'_>,
    sections: Vec<CircSection>,
    contour: CapContour,
) -> Result<Vec<CircSection>, BlendError> {
    let axis = contour.axis;
    let flat = |point: Point3| {
        let delta = point - Point3::new(0.0, 0.0, 0.0);
        delta - axis * delta.dot(axis)
    };
    let centre = |u: f64| -> Result<Vec3, BlendError> {
        Ok(flat(super::contour_root(field, u, contour)?.1))
    };
    // The contour's own samples; each resolved loop leaves its bridge
    // after the sample where it starts.
    let mut samples = sections;
    let mut bridges: Vec<(usize, Vec<CircSection>)> = Vec::new();
    // Each pass removes the first loop; a contour has at most a few.
    for _ in 0..8 {
        let centres: Vec<Vec3> = samples.iter().map(|section| flat(section.center)).collect();
        let crossing = first_crossing(&centres, axis).filter(|&(first, second, _, _)| {
            // A loop never spans an earlier bridge: its ball is the
            // contour's only point between the two samples it joins.
            !bridges
                .iter()
                .any(|&(at, _)| first <= at && at < second + 1)
        });
        let Some((first, second, s, t)) = crossing else {
            break;
        };
        let at = |index: usize, fraction: f64| {
            let (a, b) = (samples[index].uv1.0, samples[index + 1].uv1.0);
            a + (b - a) * fraction
        };
        // Narrow both segments on the contour itself until they meet,
        // widened by their neighbours: the contour may meet itself just
        // beyond the chords that cross.
        let last = samples.len() - 1;
        let mut one = (
            samples[first.saturating_sub(1)].uv1.0,
            samples[(first + 2).min(last)].uv1.0,
        );
        let mut two = (
            samples[second.saturating_sub(1)].uv1.0,
            samples[(second + 2).min(last)].uv1.0,
        );
        let (mut u1, mut u2) = (at(first, s), at(second, t));
        for _ in 0..8 {
            let sample = |(a, b): (f64, f64)| -> Result<Vec<(f64, Vec3)>, BlendError> {
                (0..=PIECES)
                    .map(|k| {
                        let u = a + (b - a) * (k as f64 / PIECES as f64);
                        Ok((u, centre(u)?))
                    })
                    .collect()
            };
            let (left, right) = (sample(one)?, sample(two)?);
            let mut found = None;
            'search: for p in 0..PIECES {
                for q in 0..PIECES {
                    let (a, b) = (left[p].1, left[p + 1].1);
                    let (c, d) = (right[q].1, right[q + 1].1);
                    let (r, w) = (b - a, d - c);
                    let denominator = r.cross(w).dot(axis);
                    if denominator.abs() < 1e-300 {
                        continue;
                    }
                    let f = (c - a).cross(w).dot(axis) / denominator;
                    let g = (c - a).cross(r).dot(axis) / denominator;
                    if (0.0..=1.0).contains(&f) && (0.0..=1.0).contains(&g) {
                        found = Some((p, q, f, g));
                        break 'search;
                    }
                }
            }
            let Some((p, q, f, g)) = found else {
                break;
            };
            one = (left[p].0, left[p + 1].0);
            two = (right[q].0, right[q + 1].0);
            u1 = one.0 + (one.1 - one.0) * f;
            u2 = two.0 + (two.1 - two.0) * g;
        }
        let (contact, ball, (_, v1)) = super::contour_root(field, u1, contour)?;
        let (other, twin, (_, v2)) = super::contour_root(field, u2, contour)?;
        if (ball - twin).length() > contour.radius * 1e-6 {
            return Err(refusal("canal contour crossing did not resolve"));
        }
        let ball = ball + (twin - ball) * 0.5;
        let top = ball + axis * contour.radius;
        let surface = field.surface;
        let mut trim = Vec::with_capacity(CANAL_TRIM_SAMPLES + 1);
        for i in 0..=CANAL_TRIM_SAMPLES {
            let fraction = i as f64 / CANAL_TRIM_SAMPLES as f64;
            let v = v1 + (v2 - v1) * fraction;
            trim.push(section_nearest(surface, v, ball)?.0);
        }
        let end = trim.len() - 1;
        if (trim[0] - contact).length() > contour.radius * 1e-5
            || (trim[end] - other).length() > contour.radius * 1e-5
        {
            return Err(refusal("canal contour crossing leaves its contacts"));
        }
        trim[0] = contact;
        trim[end] = other;
        let bridge = trim
            .into_iter()
            .map(|point| section_through(point, top, ball, contour.radius))
            .collect::<Result<Vec<_>, _>>()?;
        // The loop's samples go; later bridges shift with them.
        let removed = second - first;
        for (at, _) in &mut bridges {
            if *at > second {
                *at -= removed;
            }
        }
        samples.drain(first + 1..=second);
        bridges.push((first, bridge));
    }
    if bridges.is_empty() {
        return Ok(samples);
    }
    bridges.compact_sort_by_key(|&(at, _)| at);
    let mut out = Vec::with_capacity(samples.len() + bridges.len() * (CANAL_TRIM_SAMPLES + 1));
    let mut pending = bridges.into_iter().peekable();
    for (index, section) in samples.into_iter().enumerate() {
        out.push(section);
        while let Some((_, bridge)) = pending.next_if(|&(at, _)| at == index) {
            out.extend(bridge);
        }
    }
    Ok(out)
}

/// The first pair of non-adjacent segments of a planar polyline that
/// cross, with the crossing's fraction along each.
fn first_crossing(points: &[Vec3], normal: Vec3) -> Option<(usize, usize, f64, f64)> {
    let n = points.len();
    for second in 2..n.saturating_sub(1) {
        let (c, d) = (points[second], points[second + 1]);
        for first in 0..second - 1 {
            let (a, b) = (points[first], points[first + 1]);
            let (r, q) = (b - a, d - c);
            let denominator = r.cross(q).dot(normal);
            if denominator.abs() < 1e-300 {
                continue;
            }
            let s = (c - a).cross(q).dot(normal) / denominator;
            let t = (c - a).cross(r).dot(normal) / denominator;
            if (0.0..1.0).contains(&s) && (0.0..1.0).contains(&t) {
                return Some((first, second, s, t));
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    #[test]
    fn a_section_through_a_trim_point_keeps_the_cap_radius() {
        let near = Point3::new(1.0, 2.0, 3.0);
        let top = near + Vec3::new(0.0, 0.0, 2.0);
        // A point one radius from the ball: the ball's own great circle.
        let on_ball = near + Vec3::new(1.2, -1.6, 0.0);
        let section = section_through(on_ball, top, near, 2.0).unwrap();
        assert!((section.center - near).length() < 1e-12);
        // A point standing clear of the ball: the radius holds at both
        // ends, and the centre moves off the ball's by about that clearance.
        let clear = near + Vec3::new(1.23, -1.64, -0.1);
        let section = section_through(clear, top, near, 2.0).unwrap();
        for end in [section.p1, section.p2] {
            assert!(((end - section.center).length() - 2.0).abs() < 1e-12);
        }
        let clearance = (clear - near).length() - 2.0;
        assert!((section.center - near).length() < clearance * 1.01);
        // Out of reach of a section of that radius.
        assert!(section_through(near - Vec3::new(0.0, 0.0, 2.5), top, near, 2.0).is_err());
    }

    #[test]
    fn a_contour_loop_is_its_first_crossing_of_non_adjacent_segments() {
        let z = Vec3::new(0.0, 0.0, 1.0);
        let path = |points: &[(f64, f64)]| -> Vec<Vec3> {
            points.iter().map(|&(x, y)| Vec3::new(x, y, 0.0)).collect()
        };
        // Out, round a small loop, and on: segments 1 and 4 cross.
        let looped = path(&[
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
            (2.0, 1.0),
            (1.5, 1.0),
            (1.5, -1.0),
            (3.0, -1.0),
        ]);
        let (first, second, s, t) = first_crossing(&looped, z).unwrap();
        assert_eq!((first, second), (1, 4));
        assert!((s - 0.5).abs() < 1e-12 && (t - 0.5).abs() < 1e-12);
        // A path that only turns never crosses itself.
        let turning = path(&[(0.0, 0.0), (1.0, 0.0), (2.0, 0.5), (2.5, 1.5), (2.5, 3.0)]);
        assert!(first_crossing(&turning, z).is_none());
    }
}
