//! Trim membership of a ray hit on a sphere face bounded by circular arcs on
//! its sphere, for the ray-cast classifier.
//!
//! Without it a sphere face falls to the planar polygon fallback, which
//! replaces the face by its chorded boundary flattened into one plane: a
//! hemisphere bounded by the equator becomes the equatorial disc, and every
//! point inside a trimmed sphere reads as outside (#1816: a sphere already cut
//! above `z = 1` then cut below `z = -1` dropped the closing disc).
//!
//! The face lies on one side of every one of its loops, looking down its
//! normal (the outward sphere normal, flipped for a reversed face). No single
//! loop says which side, so the side is read off the shell: in a consistently
//! oriented shell every face lies on the same side of its loops, and a planar
//! face measures that side exactly (its loop's vector area against its
//! effective normal). From the query point, walk a great circle and find the
//! first boundary arc it crosses: the point is inside exactly when the walk
//! leaves the face there. Every crossing is a closed-form circle-circle event
//! on the sphere, so no chord approximates the boundary. A walk that meets a
//! vertex, touches an arc tangentially or starts on the boundary proves
//! nothing and the next direction is tried; when every direction fails the
//! hit is undecided and the ray reports itself suspicious.
//!
//! The walk is the check crate's `sphere_arcs` trim (RFC 0008 F2); the two
//! ray classifiers are separate until RFC 0008 merges them.

use remus_math::compact_sort::CompactSort;
use remus_math::surfaces::SphericalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{Face, FaceId};

use crate::error::AlgoError;

/// Angles (radians) closer than this to a decision boundary (an arc end, a
/// tangency, the start of the walk) make a walk inconclusive.
const ANGLE_EPS: f64 = 1e-9;

/// Two crossings this close along a walk are the same boundary point (the two
/// traversals of a doubled seam).
const SAME_CROSSING: f64 = 1e-9;

/// One oriented boundary arc, in the form the walk tests it.
#[derive(Debug, Clone)]
struct TrimArc {
    /// Unit normal of the arc's circle (its `u x v`).
    normal: Vec3,
    /// The circle's plane is `x . normal = h` for unit `x` about the sphere
    /// centre.
    h: f64,
    center: Point3,
    radius: f64,
    /// Unit radial directions of the arc's ends, in increasing parameter.
    start: Vec3,
    end: Vec3,
    /// Whether the arc spans more than half its circle.
    major: bool,
    /// Whether it is (almost) the whole circle.
    full: bool,
    /// `1` when the loop traverses it toward increasing parameter, else `-1`.
    sense: f64,
}

impl TrimArc {
    /// Whether the unit radial direction `r` lies inside the arc; `None`
    /// at an end.
    fn contains(&self, r: Vec3) -> Option<bool> {
        if (r - self.start).length() <= ANGLE_EPS || (r - self.end).length() <= ANGLE_EPS {
            return None;
        }
        if self.full {
            return Some(true);
        }
        // Sines of the turns from the start to `r` and from `r` to the end.
        let from_start = self.start.cross(r).dot(self.normal);
        let to_end = r.cross(self.end).dot(self.normal);
        Some(if self.major {
            !(from_start < 0.0 && to_end < 0.0)
        } else {
            from_start > 0.0 && to_end > 0.0
        })
    }

    /// Distance from `point` to the arc (to its circle where the point's
    /// radial direction falls inside the arc, else to the nearer end).
    fn distance(&self, point: Point3) -> f64 {
        let offset = point - self.center;
        let axial = offset.dot(self.normal);
        let in_plane = offset - self.normal * axial;
        let Ok(r) = in_plane.normalize() else {
            // On the axis: every point of the circle is equally far.
            return axial.hypot(self.radius);
        };
        if self.contains(r) != Some(false) {
            return axial.hypot(in_plane.length() - self.radius);
        }
        let ends =
            [self.start, self.end].map(|end| (self.center + end * self.radius - point).length());
        ends[0].min(ends[1])
    }
}

/// Every boundary arc of a sphere face whose edges are all circles on its
/// sphere.
#[derive(Debug, Clone)]
pub(super) struct SphereTrim {
    center: Point3,
    radius: f64,
    /// `1` when the face lies left of its loops looking down the outward
    /// sphere normal, `-1` when it lies right.
    sense: f64,
    arcs: Vec<TrimArc>,
}

impl SphereTrim {
    /// The trim of `face`, whose shell lies left of its loops looking down
    /// each face's effective normal when `shell_left`. `None` when an edge is
    /// not a circle on the sphere or a domain is invalid.
    pub(super) fn build(
        topo: &Topology,
        face: &Face,
        sphere: &SphericalSurface,
        shell_left: bool,
    ) -> Result<Option<Self>, AlgoError> {
        let mut arcs = Vec::new();
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oriented in topo.wire(wire)?.edges() {
                let edge = topo.edge(oriented.edge())?;
                let EdgeCurve::Circle(circle) = edge.curve() else {
                    return Ok(None);
                };
                let Ok((t0, t1)) = edge.strict_domain() else {
                    return Ok(None);
                };
                let tolerance = edge.effective_tolerance(
                    topo.vertex(edge.start())?
                        .tolerance()
                        .max(topo.vertex(edge.end())?.tolerance()),
                );
                let midpoint = circle.evaluate((t1 - t0).mul_add(0.5, t0));
                let residual = ((midpoint - sphere.center()).length() - sphere.radius()).abs();
                if !residual.is_finite() || residual > tolerance.max(1e-9) {
                    return Ok(None);
                }
                // A degenerate edge (a pole) bounds nothing.
                if (t1 - t0).abs() * circle.radius() <= tolerance.max(1e-12) {
                    continue;
                }
                let (lo, hi) = (t0.min(t1), t0.max(t1));
                let radial = |t: f64| (circle.evaluate(t) - circle.center()).normalize();
                let (Ok(start), Ok(end)) = (radial(lo), radial(hi)) else {
                    return Ok(None);
                };
                let span = hi - lo;
                arcs.push(TrimArc {
                    normal: circle.normal(),
                    h: (circle.center() - sphere.center()).dot(circle.normal()) / sphere.radius(),
                    center: circle.center(),
                    radius: circle.radius(),
                    start,
                    end,
                    major: span > std::f64::consts::PI,
                    full: span >= std::f64::consts::TAU - ANGLE_EPS,
                    sense: if (t1 > t0) == oriented.is_forward() {
                        1.0
                    } else {
                        -1.0
                    },
                });
            }
        }
        if arcs.is_empty() {
            return Ok(None);
        }
        let reversed = if face.is_reversed() { -1.0 } else { 1.0 };
        Ok(Some(Self {
            center: sphere.center(),
            radius: sphere.radius(),
            sense: if shell_left { reversed } else { -reversed },
            arcs,
        }))
    }

    /// Whether `point` lies within `eps` of a boundary arc.
    pub(super) fn near_boundary(&self, point: Point3, eps: f64) -> bool {
        self.arcs.iter().any(|arc| arc.distance(point) <= eps)
    }

    /// Whether `point`, on the sphere, lies in the face; `None` when no walk
    /// decides it.
    pub(super) fn contains(&self, point: Point3) -> Option<bool> {
        let p = (point - self.center).normalize().ok()?;
        // Tangent walk directions from fixed oblique references; a reference
        // nearly parallel to `p` gives no direction and is skipped.
        [
            Vec3::new(
                0.267_261_241_912_424_4,
                0.534_522_483_824_848_8,
                0.801_783_725_737_273_2,
            ),
            Vec3::new(
                -0.872_871_560_943_969_6,
                0.218_217_890_235_992_4,
                0.436_435_780_471_984_8,
            ),
            Vec3::new(
                0.408_248_290_463_863,
                -0.816_496_580_927_726,
                0.408_248_290_463_863,
            ),
        ]
        .into_iter()
        .find_map(|reference| {
            let d = p.cross(reference);
            if d.length() < 0.1 {
                return None;
            }
            self.walk(p, d.normalize().ok()?)
        })
    }

    /// The verdict of the first crossing along the great circle leaving unit
    /// `p` in unit tangent `d`, or `None` when the walk is inconclusive.
    ///
    /// The walk is `u(θ) = p cos θ + d sin θ`. It meets the plane
    /// `u . n = h` where `cos(θ - φ) = h / |(a, b)|`, `(a, b) = (p . n, d . n)`;
    /// both meeting points come from the unit vectors along and across `(a,
    /// b)` in the walk's plane, without evaluating `θ` itself. Crossings are
    /// ordered by a pseudo-angle monotone in `θ`.
    fn walk(&self, p: Vec3, d: Vec3) -> Option<bool> {
        // The walk's axis: its heading at `u` is `axis x u`.
        let axis = p.cross(d);
        // (pseudo-angle along the walk, leaving the face there)
        let mut crossings: Vec<(f64, bool)> = Vec::new();
        for arc in &self.arcs {
            let (a, b) = (p.dot(arc.normal), d.dot(arc.normal));
            let amplitude = a.hypot(b);
            if amplitude <= ANGLE_EPS {
                // The walk runs in the circle's plane: on it, or never meets it.
                if arc.h.abs() <= ANGLE_EPS {
                    return None;
                }
                continue;
            }
            let ratio = arc.h / amplitude;
            if ratio.abs() >= 1.0 + ANGLE_EPS {
                continue;
            }
            if ratio.abs() > 1.0 - ANGLE_EPS {
                return None; // tangent to the circle
            }
            let along = (p * a + d * b) * amplitude.recip();
            let across = (d * a - p * b) * amplitude.recip();
            let offset = ratio.mul_add(-ratio, 1.0).sqrt();
            for u in [
                along * ratio - across * offset,
                along * ratio + across * offset,
            ] {
                if (u - p).length() <= ANGLE_EPS {
                    return None; // the point is on this circle
                }
                let Ok(r) = (self.center + u * self.radius - arc.center).normalize() else {
                    return None;
                };
                if !arc.contains(r)? {
                    continue;
                }
                let tangent = arc.normal.cross(r) * arc.sense;
                let left = (u * self.sense).cross(tangent);
                let side = axis.cross(u).dot(left);
                if side.abs() <= ANGLE_EPS {
                    return None;
                }
                crossings.push((pseudo_angle(u.dot(p), u.dot(d)), side < 0.0));
            }
        }
        crossings.compact_sort_by(|l, r| l.0.total_cmp(&r.0));
        let mut rest = crossings.as_slice();
        while let Some(&(angle, leaving)) = rest.first() {
            let group = rest
                .iter()
                .take_while(|(other, _)| *other - angle <= SAME_CROSSING)
                .count();
            match group {
                1 => return Some(leaving),
                // The two traversals of a doubled seam cancel.
                2 if rest[0].1 != rest[1].1 => rest = &rest[2..],
                _ => return None,
            }
        }
        None
    }
}

/// Whether the faces of a shell lie left of their loops looking down their
/// effective normals, read off its planar faces: a loop's vector area points
/// along the face's effective normal exactly when an outer loop winds the
/// face on its left (an inner loop the other way). `None` when no planar
/// loop measures it or two disagree (an inconsistently oriented shell).
pub(super) fn shell_winds_left(
    topo: &Topology,
    faces: &[FaceId],
    loop_polygon: impl Fn(remus_topology::wire::WireId) -> Result<Vec<Point3>, AlgoError>,
) -> Result<Option<bool>, AlgoError> {
    let mut verdict = None;
    for &face_id in faces {
        let face = topo.face(face_id)?;
        let Some(normal) = face.effective_plane_normal() else {
            continue;
        };
        for (k, wire) in std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .enumerate()
        {
            let area = vector_area(&loop_polygon(wire)?);
            let Ok(unit) = area.normalize() else {
                continue;
            };
            let along = unit.dot(normal);
            // A degenerate or folded loop's direction says nothing.
            if along.abs() < 0.5 {
                continue;
            }
            let left = (along > 0.0) == (k == 0);
            match verdict {
                None => verdict = Some(left),
                Some(seen) if seen != left => return Ok(None),
                Some(_) => {}
            }
        }
    }
    Ok(verdict)
}

/// Twice the vector area of the closed polygon through `points`, fanned from
/// its first vertex.
fn vector_area(points: &[Point3]) -> Vec3 {
    let mut sum = Vec3::new(0.0, 0.0, 0.0);
    let Some(&origin) = points.first() else {
        return sum;
    };
    for pair in points[1..].windows(2) {
        sum += (pair[0] - origin).cross(pair[1] - origin);
    }
    sum
}

/// A value in `[0, 4)` increasing with the angle of `(x, y)` from the `+x`
/// axis, counter-clockwise (the "diamond angle").
fn pseudo_angle(x: f64, y: f64) -> f64 {
    if y >= 0.0 {
        if x >= 0.0 {
            y / (x + y)
        } else {
            1.0 - x / (y - x)
        }
    } else if x < 0.0 {
        2.0 - y / (-x - y)
    } else {
        3.0 + x / (x - y)
    }
}
