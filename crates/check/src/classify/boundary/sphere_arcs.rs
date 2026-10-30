//! Trim membership of a point on a sphere face bounded by circular arcs that
//! lie on the sphere, whatever the number of supporting circles (RFC 0008 F2).
//!
//! The cap, great-circle and two-plane paths each take a restricted family; a
//! patch cut by three or more circles, which need not be convex (a sphere
//! protruding through several faces of a box), fell through to a chorded 3D
//! polygon projected onto one plane, which does not bound a strongly curved
//! patch.
//!
//! The face lies on one side of every one of its loops, looking down its
//! normal (the outward sphere normal, flipped for a reversed face): the left
//! for boolean results, the right for some analytic shortcuts (the box-sphere
//! octant), since no rule fixes a winding. A consistently oriented shell has
//! one side throughout, so the side is measured exactly on a planar face
//! sharing an edge, where the loop's Newell normal against the plane normal
//! gives it; with no planar neighbour the test declines. From the query
//! point, walk a great circle and find the first boundary arc it crosses:
//! the point is inside exactly when the walk leaves the face there.
//! Every crossing is a closed-form circle-circle event on the sphere, so no
//! chord approximates the boundary. A walk that meets a vertex, touches an arc
//! tangentially or starts on the boundary proves nothing and the next
//! direction is tried; when every direction fails the test declines and the
//! caller keeps its previous path.

use remus_math::compact_sort::CompactSort;
use remus_math::surfaces::SphericalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::FaceId;

use crate::CheckError;

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
}

/// Every boundary arc of a sphere face whose edges are all circles on its
/// sphere.
#[derive(Debug, Clone)]
pub(super) struct SphereArcTrim {
    center: Point3,
    radius: f64,
    /// `1` when the face lies left of its loops looking down the outward
    /// sphere normal, `-1` when it lies right.
    sense: f64,
    arcs: Vec<TrimArc>,
}

impl SphereArcTrim {
    /// `None` when an edge is not a circle on the sphere, or a domain is
    /// invalid.
    pub(super) fn build(
        topo: &Topology,
        face_id: FaceId,
        sphere: &SphericalSurface,
    ) -> Result<Option<Self>, CheckError> {
        let face = topo.face(face_id)?;
        let mut arcs = Vec::new();
        let mut edges = Vec::new();
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oriented in topo.wire(wire)?.edges() {
                let edge = topo.edge(oriented.edge())?;
                let EdgeCurve::Circle(circle) = edge.curve() else {
                    return Ok(None);
                };
                edges.push(oriented.edge());
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
        let Some(left) = shell_winds_left(topo, face_id, &edges)? else {
            return Ok(None);
        };
        let reversed = if face.is_reversed() { -1.0 } else { 1.0 };
        Ok(Some(Self {
            center: sphere.center(),
            radius: sphere.radius(),
            sense: if left { reversed } else { -reversed },
            arcs,
        }))
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

/// Whether faces around `face_id` lie left of their loops looking down their
/// normals, read off the first planar face that shares one of `edges`: its
/// loop's Newell normal points along its effective normal exactly when an
/// outer loop winds the face on its left (an inner loop the other way).
/// `None` when no planar face shares an edge.
fn shell_winds_left(
    topo: &Topology,
    face_id: FaceId,
    edges: &[EdgeId],
) -> Result<Option<bool>, CheckError> {
    for (other, face) in topo.faces().iter() {
        if other == face_id {
            continue;
        }
        let Some(normal) = face.effective_plane_normal() else {
            continue;
        };
        for (k, wire) in std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .enumerate()
        {
            if !topo
                .wire(wire)?
                .edges()
                .iter()
                .any(|oriented| edges.contains(&oriented.edge()))
            {
                continue;
            }
            let along =
                crate::util::polygon_normal(&crate::util::wire_polygon(topo, wire)?).dot(normal);
            // A degenerate loop's Newell normal is a placeholder.
            if along.abs() < 0.5 {
                continue;
            }
            return Ok(Some((along > 0.0) == (k == 0)));
        }
    }
    Ok(None)
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
