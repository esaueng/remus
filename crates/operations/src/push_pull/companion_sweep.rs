//! Clearance of the faces that translate rigidly with a swept selection
//! (blend bands, chamfers and supports a blend translation carries).
//!
//! A swept face's own clearance proofs say nothing about the faces it
//! carries: a cap's coaxial exemption clears a collar from the cap but not
//! from the chamfer running down into the opening, and an obstacle wholly
//! outside the cap's swept box can still lie in the chamfer's (#1949).
//!
//! A companion on a surface of revolution about an axis parallel to the move
//! (cone, cylinder, torus, sphere) is therefore checked against every face
//! that neither moves nor shares an edge with it. A pair is cleared when the
//! companion's swept box misses the candidate's box, or when the radii the
//! companion sweeps through over the candidate's heights miss the
//! candidate's radii about that axis ([`Meridian`]); any other pair refuses
//! the move. A fixed rim the companion meets on a shared cylindrical support
//! is left to the exact axial travel limit the blend translation enforces
//! for that pair.
//!
//! Other companions (a torus band about an axis across the move, planar and
//! freeform faces) keep only the swept faces' companion test: a candidate a
//! swept face may reach must also be cleared from every companion
//! ([`super::SweptRegionCheck`]). Their exact swept regions are not read
//! here; a plain box test on them refuses valid moves (the hammer holder's
//! arm), so they stay a known gap.

use remus_math::{
    aabb::Aabb3,
    tolerance::Tolerance,
    vec::{Point3, Vec3},
};
use remus_topology::{
    Topology,
    face::{FaceId, FaceSurface},
};

use super::radial_clearance;

/// A face translating rigidly with the swept selection, and, when it has an
/// axis along the move, every face it may reach.
#[derive(Debug, Clone)]
pub(super) struct Companion {
    face: FaceId,
    original: Aabb3,
    planar: bool,
    meridian: Option<Meridian>,
    candidates: Vec<Reach>,
}

/// A face a companion may reach: its box, and its extent about the
/// companion's axis when the companion has one and the extent is readable.
#[derive(Debug, Clone, Copy)]
struct Reach {
    face: FaceId,
    bounds: Aabb3,
    radial: Option<RadialExtent>,
}

/// Axial heights and radii, about a [`Meridian`]'s axis, that bound every
/// point of a face.
#[derive(Debug, Clone, Copy)]
struct RadialExtent {
    height: (f64, f64),
    radius: (f64, f64),
}

/// A companion on a surface of revolution whose axis is parallel to the
/// move, read in the half-plane of (radius, height) about that axis. The
/// face lies, at every height of `height`, within the radii `profile` gives.
#[derive(Debug, Clone, Copy)]
struct Meridian {
    origin: Point3,
    /// Unit axis.
    axis: Vec3,
    height: (f64, f64),
    profile: Profile,
    /// Largest coordinate magnitude read, for the rounding allowance.
    scale: f64,
}

#[derive(Debug, Clone, Copy)]
enum Profile {
    /// A cone with its apex at the origin: radius `|h| / tan`.
    Cone { tan: f64 },
    /// Radii within `(lo, hi)` at every height (cylinder, torus, sphere).
    Band { lo: f64, hi: f64 },
}

impl Companion {
    /// Reads `face` and its candidates: every face of `source_faces` that is
    /// not `excluded` (it moves or stretches with the selection) nor
    /// `skipped` (it shares an edge with `face`, or the caller bounds the
    /// pair another way).
    pub(super) fn prepare(
        topo: &Topology,
        face: FaceId,
        source_faces: &[FaceId],
        excluded: &dyn Fn(FaceId) -> bool,
        skipped: &dyn Fn(FaceId) -> bool,
        direction: Vec3,
    ) -> Result<Self, crate::OperationsError> {
        let original = crate::measure::face_bounding_box(topo, face)?;
        let surface = topo.face(face)?.surface();
        let planar = matches!(surface, FaceSurface::Plane { .. });
        let meridian = Meridian::read(topo, face, original, direction)?;
        let mut candidates = Vec::new();
        for &candidate in source_faces {
            let Some(meridian) = &meridian else {
                break;
            };
            if candidate == face || excluded(candidate) || skipped(candidate) {
                continue;
            }
            let bounds = crate::measure::face_bounding_box(topo, candidate)?;
            let radial = meridian.extent(topo, candidate, bounds)?;
            candidates.push(Reach {
                face: candidate,
                bounds,
                radial,
            });
        }
        Ok(Self {
            face,
            original,
            planar,
            meridian,
            candidates,
        })
    }

    pub(super) const fn face(&self) -> FaceId {
        self.face
    }

    pub(super) const fn original(&self) -> Aabb3 {
        self.original
    }

    pub(super) const fn planar(&self) -> bool {
        self.planar
    }

    /// The first candidate the companion may reach over a translation by
    /// `delta`, in face order, or `None` when every one is cleared.
    pub(super) fn blocking(&self, delta: Vec3) -> Option<FaceId> {
        let destination = Aabb3 {
            min: self.original.min + delta,
            max: self.original.max + delta,
        };
        let swept = self.original.union(destination);
        self.candidates
            .iter()
            .find(|reach| swept.intersects(reach.bounds) && !self.radially_clear(reach, delta))
            .map(|reach| reach.face)
    }

    fn radially_clear(&self, reach: &Reach, delta: Vec3) -> bool {
        let (Some(meridian), Some(extent)) = (&self.meridian, &reach.radial) else {
            return false;
        };
        meridian.clears(extent, delta)
    }
}

impl Meridian {
    fn read(
        topo: &Topology,
        face: FaceId,
        bounds: Aabb3,
        direction: Vec3,
    ) -> Result<Option<Self>, crate::OperationsError> {
        let tol = Tolerance::new();
        let Ok(direction) = direction.normalize() else {
            return Ok(None);
        };
        let parallel = |axis: Vec3| axis.cross(direction).length() <= tol.angular;
        let (origin, axis, profile) = match topo.face(face)?.surface() {
            FaceSurface::Cone(cone) if parallel(cone.axis()) => (
                cone.apex(),
                cone.axis(),
                Profile::Cone {
                    tan: cone.half_angle().tan(),
                },
            ),
            FaceSurface::Cylinder(cylinder) if parallel(cylinder.axis()) => {
                let Some(hi) = radial_clearance::cylinder_envelope(topo, face, cylinder) else {
                    return Ok(None);
                };
                (
                    cylinder.origin(),
                    cylinder.axis(),
                    Profile::Band {
                        lo: cylinder.radius(),
                        hi: hi.max(cylinder.radius()),
                    },
                )
            }
            FaceSurface::Torus(torus) if parallel(torus.z_axis()) => (
                torus.center(),
                torus.z_axis(),
                Profile::Band {
                    lo: (torus.major_radius() - torus.minor_radius()).max(0.0),
                    hi: torus.major_radius() + torus.minor_radius(),
                },
            ),
            FaceSurface::Sphere(sphere) => (
                sphere.center(),
                direction,
                Profile::Band {
                    lo: 0.0,
                    hi: sphere.radius(),
                },
            ),
            _ => return Ok(None),
        };
        let Ok(axis) = axis.normalize() else {
            return Ok(None);
        };
        // Orient the axis with the move so heights grow along it.
        let axis = if axis.dot(direction) < 0.0 {
            -axis
        } else {
            axis
        };
        let scale = magnitude(origin)
            .max(magnitude(bounds.min))
            .max(magnitude(bounds.max));
        let mut height = height_range(bounds, origin, axis);
        // Height along its own axis is linear in a cone's or cylinder's
        // generator parameter, so its extremes lie on the boundary, which
        // a tilted face's box overstates.
        if matches!(profile, Profile::Cone { .. })
            || matches!(topo.face(face)?.surface(), FaceSurface::Cylinder(_))
        {
            height = narrowed(height, boundary_heights(topo, face, origin, axis)?);
        }
        Ok(Some(Self {
            origin,
            axis,
            height,
            profile,
            scale,
        }))
    }

    /// The candidate's heights and radii about the axis, or `None` when no
    /// finite bound is readable.
    fn extent(
        &self,
        topo: &Topology,
        candidate: FaceId,
        bounds: Aabb3,
    ) -> Result<Option<RadialExtent>, crate::OperationsError> {
        let tol = Tolerance::new();
        let mut height = height_range(bounds, self.origin, self.axis);
        // Height is linear on a plane, and on a cone or cylinder about a
        // parallel axis, so its extremes lie on the boundary.
        let linear = match topo.face(candidate)?.surface() {
            FaceSurface::Plane { .. } => true,
            FaceSurface::Cylinder(cylinder) => {
                cylinder.axis().cross(self.axis).length() <= tol.angular
            }
            FaceSurface::Cone(cone) => cone.axis().cross(self.axis).length() <= tol.angular,
            FaceSurface::Sphere(_) | FaceSurface::Torus(_) | FaceSurface::Nurbs(_) => false,
        };
        if linear {
            height = narrowed(
                height,
                boundary_heights(topo, candidate, self.origin, self.axis)?,
            );
        }
        // Distance from a line is convex, so a box's largest is at a corner.
        let mut upper = 0.0_f64;
        for x in [bounds.min.x(), bounds.max.x()] {
            for y in [bounds.min.y(), bounds.max.y()] {
                for z in [bounds.min.z(), bounds.max.z()] {
                    upper = upper.max(self.radius_of(Point3::new(x, y, z)));
                }
            }
        }
        // No point of the box is nearer the axis than its centre less its
        // half-diagonal.
        let half = (bounds.max - bounds.min) * 0.5;
        let mut lower = (self.radius_of(bounds.min + half) - half.length()).max(0.0);
        // The offset of a carrier axis from this one, when the two are
        // parallel; a point at radius `r` about the carrier's axis lies
        // within `r ± offset` of this one.
        let coaxial = |origin: Point3, axis: Vec3| {
            (axis.cross(self.axis).length() <= tol.angular).then(|| self.radius_of(origin))
        };
        match topo.face(candidate)?.surface() {
            FaceSurface::Cylinder(cylinder) => {
                if let Some(offset) = coaxial(cylinder.origin(), cylinder.axis()) {
                    lower = lower.max(cylinder.radius() - offset);
                    if let Some(hi) = radial_clearance::cylinder_envelope(topo, candidate, cylinder)
                    {
                        upper = upper.min(hi.max(cylinder.radius()) + offset);
                    }
                }
            }
            FaceSurface::Torus(torus) => {
                if let Some(offset) = coaxial(torus.center(), torus.z_axis()) {
                    lower = lower.max(torus.major_radius() - torus.minor_radius() - offset);
                    upper = upper.min(torus.major_radius() + torus.minor_radius() + offset);
                }
            }
            FaceSurface::Cone(cone) => {
                if let Some(offset) = coaxial(cone.apex(), cone.axis()) {
                    let apex = (cone.apex() - self.origin).dot(self.axis);
                    let (near, far) =
                        cone_radii((height.0 - apex, height.1 - apex), cone.half_angle().tan());
                    lower = lower.max(near - offset);
                    upper = upper.min(far + offset);
                }
            }
            FaceSurface::Plane { normal, .. } => {
                if normal.cross(self.axis).length() <= tol.angular
                    && let Some(clearance) = radial_clearance::planar_face_axis_clearance(
                        topo,
                        candidate,
                        self.origin,
                        self.axis,
                    )?
                {
                    lower = lower.max(clearance);
                }
            }
            FaceSurface::Sphere(_) | FaceSurface::Nurbs(_) => {}
        }
        Ok(
            (lower.is_finite()
                && upper.is_finite()
                && height.0.is_finite()
                && height.1.is_finite())
            .then_some(RadialExtent {
                height,
                radius: (lower, upper),
            }),
        )
    }

    /// Whether the companion, translated through every fraction of `delta`,
    /// stays off the candidate's radii over the candidate's heights.
    fn clears(&self, extent: &RadialExtent, delta: Vec3) -> bool {
        let tol = Tolerance::new();
        let travel = delta.dot(self.axis);
        // A move not exactly along the axis drifts it sideways, which moves
        // any point's radius by at most the drift.
        let drift = (delta - self.axis * travel).length();
        let (t_min, t_max) = (travel.min(0.0), travel.max(0.0));
        // Heights of the source face that land in the candidate's heights at
        // some fraction of the travel.
        let low = self.height.0.max(extent.height.0 - t_max);
        let high = self.height.1.min(extent.height.1 - t_min);
        let margin = 1e-12_f64.mul_add(self.scale + travel.abs(), tol.linear);
        if low > high + margin {
            return true;
        }
        let (low, high) = (low.min(high), high.max(low));
        let (lo, hi) = match self.profile {
            Profile::Cone { tan } => cone_radii((low, high), tan),
            Profile::Band { lo, hi } => (lo, hi),
        };
        let (lo, hi) = (lo - drift, hi + drift);
        extent.radius.1 + margin < lo || extent.radius.0 > hi + margin
    }

    fn radius_of(&self, point: Point3) -> f64 {
        (point - self.origin).cross(self.axis).length()
    }
}

/// The radii of a cone of the given `tan` (of its angle from the radial
/// plane) over the heights `(low, high)` from its apex: radius `|h| / tan`,
/// zero where the heights cross the apex.
fn cone_radii((low, high): (f64, f64), tan: f64) -> (f64, f64) {
    let (a, b) = (low.abs() / tan, high.abs() / tan);
    let near = if low <= 0.0 && high >= 0.0 {
        0.0
    } else {
        a.min(b)
    };
    (near, a.max(b))
}

/// `range` intersected with `other`, when that is readable and not empty.
fn narrowed(range: (f64, f64), other: Option<(f64, f64)>) -> (f64, f64) {
    match other {
        Some((low, high)) if low.max(range.0) <= high.min(range.1) => {
            (low.max(range.0), high.min(range.1))
        }
        _ => range,
    }
}

/// The heights along the unit `axis` from `origin` that bound every edge of
/// `face`'s boundary, or `None` for an edge curve without a certified bound
/// (only lines, circles and positive-weight NURBS are read). A circle's
/// heights are its centre's within its radius times the sine of its tilt
/// from the axis, whatever arc of it the edge is.
fn boundary_heights(
    topo: &Topology,
    face: FaceId,
    origin: Point3,
    axis: Vec3,
) -> Result<Option<(f64, f64)>, crate::OperationsError> {
    use remus_topology::edge::EdgeCurve;
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    let mut take = |h: f64, spread: f64| {
        low = low.min(h - spread);
        high = high.max(h + spread);
    };
    let at = |p: Point3| (p - origin).dot(axis);
    for id in remus_topology::explorer::face_edges(topo, face)? {
        let edge = topo.edge(id)?;
        for vertex in [edge.start(), edge.end()] {
            take(at(topo.vertex(vertex)?.point()), 0.0);
        }
        match edge.curve() {
            EdgeCurve::Line => {}
            EdgeCurve::Circle(circle) => {
                let Ok(normal) = circle.normal().normalize() else {
                    return Ok(None);
                };
                take(
                    at(circle.center()),
                    circle.radius() * normal.cross(axis).length(),
                );
            }
            EdgeCurve::NurbsCurve(curve) if curve.weights().iter().all(|&w| w > 0.0) => {
                for &p in curve.control_points() {
                    take(at(p), 0.0);
                }
            }
            EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Parabola(_)
            | EdgeCurve::Hyperbola(_) => return Ok(None),
        }
    }
    Ok((low.is_finite() && high.is_finite()).then_some((low, high)))
}

/// The heights along the unit `axis` from `origin` that bound `bounds`.
fn height_range(bounds: Aabb3, origin: Point3, axis: Vec3) -> (f64, f64) {
    let half = (bounds.max - bounds.min) * 0.5;
    let middle = (bounds.min + half - origin).dot(axis);
    let extent = axis.x().abs().mul_add(
        half.x(),
        axis.y().abs().mul_add(half.y(), axis.z().abs() * half.z()),
    );
    (middle - extent, middle + extent)
}

fn magnitude(p: Point3) -> f64 {
    p.x().abs().max(p.y().abs()).max(p.z().abs())
}
