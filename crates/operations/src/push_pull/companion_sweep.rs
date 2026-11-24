//! Clearance of the faces that translate rigidly with a swept selection
//! (blend bands, chamfers and supports a blend translation carries).
//!
//! A swept face's own clearance proofs say nothing about the faces it
//! carries: a cap's coaxial exemption clears a collar from the cap but not
//! from the chamfer running down into the opening, and an obstacle wholly
//! outside the cap's swept box can still lie in the chamfer's (#1949).
//!
//! A companion on a cone or cylinder about an axis parallel to the move is
//! therefore checked against every face that neither moves nor shares an
//! edge with it. A pair is cleared when their certified boxes miss over the
//! whole travel, or when, about the companion's axis, the radii the
//! companion sweeps through miss the candidate's radii at every height of
//! the candidate. Both faces are read as profiles in the (radius, height)
//! half-plane: a cone's generator, a cylinder's or a plane's band of radii,
//! a torus's whole tube or a sphere's whole ball over the heights of its
//! certified box. The candidate's heights are bisected until each slab
//! separates or a work limit is spent; any pair left undecided refuses the
//! move. A fixed rim the companion meets on a shared cylindrical support
//! separates this way too, away from the contact the axial travel limit
//! bounds.
//!
//! Every bound read here is conservative and needs no decision of which
//! side of its boundary a face lies on: boxes come from the certified face
//! bounds ([`remus_check::distance::face_bounds::face_bound`]) or, for
//! cones and cylinders, from their carriers over the heights of their
//! boundary edges; a carrier whose axis is parallel to the companion's only
//! within tolerance has its offset and tilt charged over its whole extent.
//!
//! Other companions (torus bands, sphere patches, planar and freeform
//! faces, and cones and cylinders across the move) keep only the swept
//! faces' companion test: a candidate a swept face may reach must also be
//! cleared from every companion ([`super::SweptRegionCheck`]). Bounding a
//! torus band or sphere patch needs the side of its boundary it lies on,
//! which its rims alone do not say (a band between two rims may be the
//! short one or the long one), so they stay a known gap.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use remus_math::{
    aabb::Aabb3,
    compact_sort::CompactSort,
    tolerance::Tolerance,
    vec::{Point3, Vec3},
};
use remus_topology::{
    Topology,
    edge::EdgeCurve,
    face::{FaceId, FaceSurface},
};

use super::radial_clearance;

/// Bisection depth and the number of slabs one pair may split.
const MAX_DEPTH: u32 = 24;
const SLAB_BUDGET: u32 = 4096;

/// Minor angles are read with this much slack, so rounding in `asin` or in
/// the boundary's angles never drops a sliver of an arc.
const ANGLE_SLACK: f64 = 1e-9;

/// A face translating rigidly with the swept selection, and, when it has an
/// axis along the move, every face it may reach.
#[derive(Debug, Clone)]
pub(super) struct Companion {
    face: FaceId,
    /// A box for the swept faces' companion test. For a NURBS face this
    /// is available only when the bound is certified.
    original: Option<Aabb3>,
    planar: bool,
    sweep: Option<Sweep>,
    candidates: Vec<Reach>,
}

/// A companion with an axis along the move: the axis frame, its profile in
/// that frame and its certified box.
#[derive(Debug, Clone)]
pub(super) struct Sweep {
    origin: Point3,
    /// Unit axis, oriented with the move.
    axis: Vec3,
    profile: Profile,
    bounds: Option<Aabb3>,
    /// Largest coordinate magnitude read, for the rounding allowance.
    scale: f64,
}

/// A face a companion may reach: its certified box, and its profile about
/// the companion's axis when one is readable.
#[derive(Debug, Clone)]
struct Reach {
    face: FaceId,
    bounds: Option<Aabb3>,
    profile: Option<Profile>,
    scale: f64,
}

/// Where a face lies in the (radius, height) half-plane about an axis.
///
/// The shape is read in its own carrier's frame; a point at own height `h`
/// lies at the axis frame's height `shift + h` within `height_slack`, and
/// its radius there is its own within `radial_slack` (both zero for a face
/// read in its own frame).
#[derive(Debug, Clone)]
struct Profile {
    shift: f64,
    height_slack: f64,
    radial_slack: f64,
    /// Own heights that bound the face.
    heights: (f64, f64),
    shape: Shape,
    /// Magnitude of the carrier's frame origin, for the rounding allowance.
    scale: f64,
}

#[derive(Debug, Clone)]
enum Shape {
    /// Radii within `(lo, hi)` at every height.
    Band { lo: f64, hi: f64 },
    /// A cone with its apex at own height zero: radius `|h| / tan`.
    Cone { tan: f64 },
    /// The whole disk inside the cone at each swept height.
    ConicDisk { tan: f64 },
    /// Points `(major + minor cos φ, minor sin φ)` for minor angles `φ` in
    /// `phi` (a torus about own height zero; a sphere has `major` zero).
    Arc {
        major: f64,
        minor: f64,
        phi: (f64, f64),
    },
    /// A face within `radius` of a carrier axis segment across the frame's
    /// axis (read in the frame itself: no shift or slack).
    Tube(Tube),
}

/// A cone or cylinder whose axis is not parallel to the frame's, read in
/// the frame: its points are `P(t) + ρ (cos θ f1 + sin θ f2)` about the
/// carrier axis `P(t) = base + direction t`, `t` in `span`, with `ρ` in
/// `radii` and `θ` in `angles` (`f1` the carrier-perpendicular direction
/// nearest the frame axis, `f2 = direction × f1`).
///
/// Such a point lies `ρ sine cos θ` above `P(t)` (`sine` that of the angle
/// between the axes), so a height slab bounds `cos θ`; across the frame
/// axis it sits at `offset(t) + ρ cos θ lean + ρ sin θ f2`, `offset(t) =
/// across + drift t` and `lean` the across part of `f1`. The points across
/// lie in the segment `offset(t)` plus a box in the plane spanned by `f2`
/// and `lean`; the exact nearest and farthest distances of that convex
/// hull from the axis bound the radii.
#[derive(Debug, Clone)]
struct Tube {
    across: Vec3,
    drift: Vec3,
    level: f64,
    rate: f64,
    span: (f64, f64),
    /// `|t| / tan` for a cone (zero at the apex), else none.
    cone_tan: Option<f64>,
    /// Radii about the carrier axis: a cylinder's radius and envelope.
    radii: (f64, f64),
    sine: f64,
    lean: Vec3,
    f2: Vec3,
    /// The across-plane frame: `q1 = f2`, `q2 = frame axis × q1`.
    q2: Vec3,
    /// Angles `θ` covering the face (a full turn when unread).
    angles: (f64, f64),
}

impl Companion {
    /// Reads `face` and its candidates: every face of `source_faces` that is
    /// not `excluded` (it moves or stretches with the selection) nor
    /// `adjacent` to it.
    pub(super) fn prepare(
        topo: &Topology,
        face: FaceId,
        source_faces: &[FaceId],
        excluded: &dyn Fn(FaceId) -> bool,
        adjacent: &dyn Fn(FaceId) -> bool,
        direction: Vec3,
    ) -> Result<Self, crate::OperationsError> {
        let surface = topo.face(face)?.surface();
        // The measured trim domain samples the boundary and can miss a
        // narrow lobe. Clearance needs the certified NURBS bound (#1950).
        let original = if matches!(surface, FaceSurface::Nurbs(_)) {
            certified_box(topo, face)?
        } else {
            Some(crate::measure::face_bounding_box(topo, face)?)
        };
        let planar = matches!(surface, FaceSurface::Plane { .. });
        let sweep = Sweep::read(topo, face, direction)?;
        let mut candidates = Vec::new();
        if let Some(sweep) = &sweep {
            for &candidate in source_faces {
                if candidate == face || excluded(candidate) || adjacent(candidate) {
                    continue;
                }
                let bounds = certified_box(topo, candidate)?;
                let profile = sweep.candidate_profile(topo, candidate, bounds)?;
                candidates.push(Reach {
                    face: candidate,
                    bounds,
                    scale: bounds
                        .map_or(0.0, box_magnitude)
                        .max(profile.as_ref().map_or(0.0, |profile| profile.scale)),
                    profile,
                });
            }
        }
        Ok(Self {
            face,
            original,
            planar,
            sweep,
            candidates,
        })
    }

    pub(super) const fn face(&self) -> FaceId {
        self.face
    }

    pub(super) const fn original(&self) -> Option<Aabb3> {
        self.original
    }

    pub(super) const fn planar(&self) -> bool {
        self.planar
    }

    /// The first candidate the companion may reach over a translation by
    /// `delta`, in face order, or `None` when every one is cleared.
    pub(super) fn blocking(&self, delta: Vec3) -> Option<FaceId> {
        let sweep = self.sweep.as_ref()?;
        let swept = sweep.bounds.map(|original| {
            original.union(Aabb3 {
                min: original.min + delta,
                max: original.max + delta,
            })
        });
        self.candidates
            .iter()
            .find(|reach| {
                let boxes_miss = matches!(
                    (swept, reach.bounds),
                    (Some(swept), Some(bounds)) if !swept.intersects(bounds)
                );
                !boxes_miss && !sweep.clears(reach, delta)
            })
            .map(|reach| reach.face)
    }
}

impl Sweep {
    fn read(
        topo: &Topology,
        face: FaceId,
        direction: Vec3,
    ) -> Result<Option<Self>, crate::OperationsError> {
        let tol = Tolerance::new();
        let Ok(direction) = direction.normalize() else {
            return Ok(None);
        };
        let parallel = |axis: Vec3| axis.cross(direction).length() <= tol.angular;
        // The companion is read in its own carrier's frame. Only cones and
        // cylinders are: a torus band or sphere patch is bounded only once
        // it is known which side of its boundary it lies on, which its rims
        // alone do not say, so those stay with the companions this check
        // leaves alone.
        let (origin, axis) = match topo.face(face)?.surface() {
            FaceSurface::Cone(cone) if parallel(cone.axis()) => (cone.apex(), cone.axis()),
            FaceSurface::Cylinder(cylinder) if parallel(cylinder.axis()) => {
                (cylinder.origin(), cylinder.axis())
            }
            FaceSurface::Plane { .. }
            | FaceSurface::Sphere(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Cylinder(_)
            | FaceSurface::Torus(_)
            | FaceSurface::Nurbs(_) => return Ok(None),
        };
        let Ok(axis) = axis.normalize() else {
            return Ok(None);
        };
        let axis = if axis.dot(direction) < 0.0 {
            -axis
        } else {
            axis
        };
        let Some(profile) = own_profile(topo, face, origin, axis)? else {
            return Ok(None);
        };
        let bounds = certified_box(topo, face)?;
        Ok(Some(Self {
            origin,
            axis,
            profile,
            bounds,
            scale: magnitude(origin).max(bounds.map_or(0.0, box_magnitude)),
        }))
    }

    /// The planar disk a widening cap traces along its single conical wall.
    /// The cone fixes the radius at every intermediate height. Check both
    /// rims against the cone's axis and account for their measured extent.
    pub(super) fn for_widening_cap(
        topo: &Topology,
        source: FaceId,
        rebuilt: FaceId,
        support: FaceId,
        direction: Vec3,
    ) -> Result<Option<Self>, crate::OperationsError> {
        let FaceSurface::Cone(cone) = topo.face(support)?.surface() else {
            return Ok(None);
        };
        let axis = cone.axis().normalize()?;
        let tilt = axis.cross(direction).length();
        let tol = Tolerance::new();
        if tilt > tol.angular {
            return Ok(None);
        }
        let (source_face, rebuilt_face) = (topo.face(source)?, topo.face(rebuilt)?);
        if !source_face.inner_wires().is_empty() || !rebuilt_face.inner_wires().is_empty() {
            return Ok(None);
        }
        let (Some(source), Some(rebuilt)) = (
            radial_clearance::circular_wire(topo, source_face.outer_wire(), axis)?,
            radial_clearance::circular_wire(topo, rebuilt_face.outer_wire(), axis)?,
        ) else {
            return Ok(None);
        };

        let origin = cone.apex();
        let tan = cone.half_angle().tan();
        let start = source.center - origin;
        let end = rebuilt.center - origin;
        let (start_height, end_height) = (start.dot(axis), end.dot(axis));
        if start_height.is_sign_negative() != end_height.is_sign_negative()
            || end_height.abs() <= start_height.abs()
        {
            return Ok(None);
        }

        let scale = magnitude(origin)
            .max(magnitude(source.center))
            .max(magnitude(rebuilt.center));
        let rounding = 1e-12_f64.mul_add(scale, tol.linear);
        let radial_slack = (start.cross(axis).length() + source.max_reach
            - start_height.abs() / tan)
            .max(end.cross(axis).length() + rebuilt.max_reach - end_height.abs() / tan)
            .max(0.0);
        let height_slack = tilt.mul_add(source.max_reach.max(rebuilt.max_reach), rounding);
        let heights = (start_height.min(end_height), start_height.max(end_height));
        let profile = finite_profile(Profile {
            shift: 0.0,
            height_slack,
            radial_slack,
            heights,
            shape: Shape::ConicDisk { tan },
            scale,
        });
        Ok(profile.map(|profile| Self {
            origin,
            axis,
            profile,
            bounds: None,
            scale,
        }))
    }

    /// Prove that the fixed face misses the widening cap at all swept heights.
    pub(super) fn clears_fixed_face(
        &self,
        topo: &Topology,
        face: FaceId,
    ) -> Result<bool, crate::OperationsError> {
        let bounds = certified_box(topo, face)?;
        if let Some(bounds) = bounds {
            let margin =
                1e-12_f64.mul_add(self.scale + box_magnitude(bounds), Tolerance::new().linear);
            let (low, high) = height_range(bounds, self.origin, self.axis);
            if self
                .profile
                .radii((low - margin, high + margin))
                .is_none_or(|(_, hi)| self.box_lower_radius(bounds) > hi + margin)
            {
                return Ok(true);
            }
        }

        let profile = self.candidate_profile(topo, face, bounds)?;
        Ok(self.clears(
            &Reach {
                face,
                bounds,
                scale: bounds
                    .map_or(0.0, box_magnitude)
                    .max(profile.as_ref().map_or(0.0, |p| p.scale)),
                profile,
            },
            Vec3::new(0.0, 0.0, 0.0),
        ))
    }

    fn radius_of(&self, point: Point3) -> f64 {
        (point - self.origin).cross(self.axis).length()
    }

    /// The candidate's profile about this axis, or `None` when no finite
    /// bound is readable.
    fn candidate_profile(
        &self,
        topo: &Topology,
        candidate: FaceId,
        bounds: Option<Aabb3>,
    ) -> Result<Option<Profile>, crate::OperationsError> {
        let tol = Tolerance::new();
        let surface = topo.face(candidate)?.surface();
        // A sphere whose centre lies `d` from this axis cuts each height in a
        // circle of radius `s` about a point `d` from the axis, whose radii
        // run `d ± s`: the full tube of a torus of major radius `d`. The face
        // is read as its whole sphere (which side of its boundary a patch
        // lies on is not decided here), over the heights of its box.
        if let FaceSurface::Sphere(sphere) = surface {
            let center = sphere.center();
            return Ok(
                arc_profile(self.radius_of(center), sphere.radius(), (-PI, PI)).and_then(
                    |(heights, shape)| {
                        let shift = (center - self.origin).dot(self.axis);
                        finite_profile(Profile {
                            shift,
                            height_slack: 0.0,
                            radial_slack: 0.0,
                            heights: self.box_heights(heights, bounds, shift, 0.0),
                            shape,
                            scale: magnitude(center),
                        })
                    },
                ),
            );
        }
        // A carrier on an axis parallel to this one within tolerance: read in
        // its own frame and charged its offset and tilt.
        let carrier = match surface {
            FaceSurface::Cone(cone) => Some((cone.apex(), cone.axis())),
            FaceSurface::Cylinder(cylinder) => Some((cylinder.origin(), cylinder.axis())),
            FaceSurface::Torus(torus) => Some((torus.center(), torus.z_axis())),
            FaceSurface::Plane { .. } | FaceSurface::Sphere(_) | FaceSurface::Nurbs(_) => None,
        };
        if let Some((origin, axis)) = carrier
            && let Ok(axis) = axis.normalize()
            && axis.cross(self.axis).length() <= tol.angular
        {
            let axis = if axis.dot(self.axis) < 0.0 {
                -axis
            } else {
                axis
            };
            if let Some(own) = own_profile(topo, candidate, origin, axis)? {
                let sine = axis.cross(self.axis).length();
                let cosine = axis.dot(self.axis);
                let reach = own.heights.0.abs().max(own.heights.1.abs());
                let radius = own.shape.max_radius(own.heights);
                // A point at own height `t` and radius `ρ` lies at height
                // `shift + t cos` here within `ρ sin`, and its distance from
                // this axis differs from `ρ` by at most the axes' distance at
                // that height (`offset + |t| sin`) plus `ρ sin`.
                let shift = (origin - self.origin).dot(self.axis);
                let height_slack = reach.mul_add(1.0 - cosine, radius * sine);
                let offset = self.radius_of(origin);
                let tilt_slack = sine * (reach + radius);
                let (shape, radial_slack) = match own.shape {
                    // For a constant band, the two triangle inequalities
                    // also give clearance when the offset exceeds the radius.
                    Shape::Band { lo, hi } => (
                        Shape::Band {
                            lo: (lo - offset).max(offset - hi).max(0.0),
                            hi: hi + offset,
                        },
                        tilt_slack,
                    ),
                    shape => (shape, offset + tilt_slack),
                };
                return Ok(finite_profile(Profile {
                    shift,
                    height_slack,
                    radial_slack,
                    heights: self.box_heights(own.heights, bounds, shift, height_slack),
                    shape,
                    scale: magnitude(origin),
                }));
            }
        }
        // A cone or cylinder across the axis: the tube about its axis segment.
        if let Some((origin, axis)) = carrier
            && let Ok(axis) = axis.normalize()
            && let Some((low, high, tube)) =
                Tube::read(topo, candidate, origin, axis, self.origin, self.axis)?
        {
            return Ok(finite_profile(Profile {
                shift: 0.0,
                height_slack: 0.0,
                radial_slack: 0.0,
                heights: (low, high),
                shape: Shape::Tube(tube),
                scale: magnitude(origin),
            }));
        }
        // Otherwise a band of radii over the face's heights here.
        let mut heights = bounds.map(|bounds| height_range(bounds, self.origin, self.axis));
        let mut upper = bounds.map(|bounds| self.box_upper_radius(bounds));
        let mut lower = bounds.map_or(0.0, |bounds| self.box_lower_radius(bounds));
        if let FaceSurface::Plane { normal, .. } = surface {
            // Height is linear and distance from a line convex on a plane,
            // so both reach their extremes on the boundary.
            if let Some(boundary) = boundary_heights(topo, candidate, self.origin, self.axis)? {
                heights = Some(heights.map_or(boundary, |range| narrowed(range, Some(boundary))));
            }
            if let Some(boundary) = self.boundary_upper_radius(topo, candidate)? {
                upper = Some(upper.map_or(boundary, |box_upper| box_upper.min(boundary)));
            }
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
        let (Some(heights), Some(upper)) = (heights, upper) else {
            return Ok(None);
        };
        Ok(finite_profile(Profile {
            shift: 0.0,
            height_slack: 0.0,
            radial_slack: 0.0,
            heights,
            shape: Shape::Band {
                lo: lower,
                hi: upper,
            },
            scale: 0.0,
        }))
    }

    /// Own `heights` narrowed to those a point in the certified box can
    /// have: a point at own height `t` lies at height `shift + t` here
    /// within `slack`. The box's heights are widened by a rounding
    /// allowance relative to every coordinate read, so a small box far from
    /// this frame's origin never rounds to a single height.
    fn box_heights(
        &self,
        heights: (f64, f64),
        bounds: Option<Aabb3>,
        shift: f64,
        slack: f64,
    ) -> (f64, f64) {
        narrowed(
            heights,
            bounds.map(|bounds| {
                let (low, high) = height_range(bounds, self.origin, self.axis);
                let rounding = 1e-12_f64.mul_add(
                    magnitude(self.origin) + box_magnitude(bounds) + shift.abs(),
                    Tolerance::new().linear,
                );
                (
                    low - shift - slack - rounding,
                    high - shift + slack + rounding,
                )
            }),
        )
    }

    /// Distance from a line is convex, so a box's largest is at a corner.
    fn box_upper_radius(&self, bounds: Aabb3) -> f64 {
        let mut upper = 0.0_f64;
        for x in [bounds.min.x(), bounds.max.x()] {
            for y in [bounds.min.y(), bounds.max.y()] {
                for z in [bounds.min.z(), bounds.max.z()] {
                    upper = upper.max(self.radius_of(Point3::new(x, y, z)));
                }
            }
        }
        upper
    }

    /// Project the box along the radial direction to its centre. This
    /// distance along a unit vector normal to the axis bounds the radius
    /// below, even for a tall box with little width across the axis.
    fn box_lower_radius(&self, bounds: Aabb3) -> f64 {
        let half = (bounds.max - bounds.min) * 0.5;
        let radial = self
            .axis
            .cross((bounds.min + half - self.origin).cross(self.axis));
        let Ok(direction) = radial.normalize() else {
            return 0.0;
        };
        let extent = direction.x().abs() * half.x()
            + direction.y().abs() * half.y()
            + direction.z().abs() * half.z();
        (radial.length() - extent).max(0.0)
    }

    /// The largest distance from the axis of any point of a planar face's
    /// boundary, which bounds the face: distance from a line is convex. A
    /// circle lies within its radius of its centre; a positive-weight NURBS
    /// curve within its control polygon's hull.
    fn boundary_upper_radius(
        &self,
        topo: &Topology,
        face: FaceId,
    ) -> Result<Option<f64>, crate::OperationsError> {
        let mut upper = 0.0_f64;
        for id in remus_topology::explorer::face_edges(topo, face)? {
            let edge = topo.edge(id)?;
            for vertex in [edge.start(), edge.end()] {
                upper = upper.max(self.radius_of(topo.vertex(vertex)?.point()));
            }
            match edge.curve() {
                EdgeCurve::Line => {}
                EdgeCurve::Circle(circle) => {
                    upper = upper.max(self.radius_of(circle.center()) + circle.radius());
                }
                EdgeCurve::NurbsCurve(curve) if curve.weights().iter().all(|&w| w > 0.0) => {
                    for &p in curve.control_points() {
                        upper = upper.max(self.radius_of(p));
                    }
                }
                EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Parabola(_)
                | EdgeCurve::Hyperbola(_) => return Ok(None),
            }
        }
        Ok(upper.is_finite().then_some(upper))
    }

    /// Whether the companion, translated through every fraction of `delta`,
    /// stays off the candidate: at every height of the candidate, the radii
    /// the companion sweeps through there miss the candidate's.
    fn clears(&self, reach: &Reach, delta: Vec3) -> bool {
        let Some(candidate) = &reach.profile else {
            return false;
        };
        let travel = delta.dot(self.axis);
        // A move not exactly along the axis drifts it sideways, which moves
        // any point's radius by at most the drift.
        let drift = (delta - self.axis * travel).length();
        let margin = 1e-12_f64.mul_add(
            self.scale.max(reach.scale) + travel.abs(),
            Tolerance::new().linear,
        );
        let slab = Slab {
            companion: &self.profile,
            candidate,
            low_travel: travel.min(0.0),
            high_travel: travel.max(0.0),
            drift,
            margin,
        };
        let (low, high) = candidate.span();
        let mut budget = SLAB_BUDGET;
        slab.clears(low, high, MAX_DEPTH, &mut budget)
    }
}

/// One pair's bisection over the candidate's heights.
struct Slab<'a> {
    companion: &'a Profile,
    candidate: &'a Profile,
    low_travel: f64,
    high_travel: f64,
    drift: f64,
    margin: f64,
}

impl Slab<'_> {
    fn clears(&self, low: f64, high: f64, depth: u32, budget: &mut u32) -> bool {
        let m = self.margin;
        // The companion's heights that land in `low..high` at some fraction
        // of the travel.
        let Some((lo, hi)) = self
            .companion
            .radii((low - self.high_travel - m, high - self.low_travel + m))
        else {
            return true;
        };
        let Some((cand_lo, cand_hi)) = self.candidate.radii((low - m, high + m)) else {
            return true;
        };
        if cand_hi + m < lo - self.drift || cand_lo > hi + self.drift + m {
            return true;
        }
        if depth == 0 || *budget == 0 {
            return false;
        }
        *budget -= 1;
        let middle = f64::midpoint(low, high);
        self.clears(low, middle, depth - 1, budget) && self.clears(middle, high, depth - 1, budget)
    }
}

impl Profile {
    /// Heights in the axis frame that bound the face.
    fn span(&self) -> (f64, f64) {
        (
            self.shift + self.heights.0 - self.height_slack,
            self.shift + self.heights.1 + self.height_slack,
        )
    }

    /// Radii in the axis frame of the face's points at heights `low..high`
    /// there, or `None` when it has none.
    fn radii(&self, (low, high): (f64, f64)) -> Option<(f64, f64)> {
        let low = (low - self.shift - self.height_slack).max(self.heights.0);
        let high = (high - self.shift + self.height_slack).min(self.heights.1);
        if low > high {
            return None;
        }
        let (lo, hi) = self.shape.radii((low, high))?;
        Some(((lo - self.radial_slack).max(0.0), hi + self.radial_slack))
    }
}

impl Shape {
    /// Radii of the shape's points at own heights `low..high`.
    fn radii(&self, (low, high): (f64, f64)) -> Option<(f64, f64)> {
        match self {
            Self::Band { lo, hi } => Some((*lo, *hi)),
            Self::Cone { tan } => Some(cone_radii((low, high), *tan)),
            Self::ConicDisk { tan } => Some((0.0, cone_radii((low, high), *tan).1)),
            Self::Arc { major, minor, phi } => arc_radii(*major, *minor, *phi, (low, high)),
            Self::Tube(tube) => tube.radii((low, high)),
        }
    }

    fn max_radius(&self, heights: (f64, f64)) -> f64 {
        self.radii(heights).map_or(0.0, |(_, hi)| hi)
    }
}

/// A face's profile in its own carrier's frame (`origin`, unit `axis`
/// parallel to the carrier's), or `None` for a carrier it does not read.
fn own_profile(
    topo: &Topology,
    face: FaceId,
    origin: Point3,
    axis: Vec3,
) -> Result<Option<Profile>, crate::OperationsError> {
    let profile = |heights: (f64, f64), shape: Shape| {
        finite_profile(Profile {
            shift: 0.0,
            height_slack: 0.0,
            radial_slack: 0.0,
            heights,
            shape,
            scale: 0.0,
        })
    };
    Ok(match topo.face(face)?.surface() {
        // Height along its own axis is linear in a cone's or cylinder's
        // generator parameter, so its extremes lie on the boundary.
        FaceSurface::Cone(cone) => boundary_heights(topo, face, origin, axis)?.and_then(|h| {
            profile(
                h,
                Shape::Cone {
                    tan: cone.half_angle().tan(),
                },
            )
        }),
        FaceSurface::Cylinder(cylinder) => {
            let hi = radial_clearance::cylinder_envelope(topo, face, cylinder);
            match (boundary_heights(topo, face, origin, axis)?, hi) {
                (Some(h), Some(hi)) => profile(
                    h,
                    Shape::Band {
                        lo: cylinder.radius(),
                        hi: hi.max(cylinder.radius()),
                    },
                ),
                _ => None,
            }
        }
        // Two rims do not say whether a torus band is the short one between
        // them or the long one, so the face is read as its whole tube (the
        // caller narrows it to its certified box).
        FaceSurface::Torus(torus) => {
            arc_profile(torus.major_radius(), torus.minor_radius(), (-PI, PI))
                .and_then(|(h, s)| profile(h, s))
        }
        FaceSurface::Plane { .. } | FaceSurface::Sphere(_) | FaceSurface::Nurbs(_) => None,
    })
}

/// The heights and shape of an arc of minor angles `phi`.
fn arc_profile(major: f64, minor: f64, phi: (f64, f64)) -> Option<((f64, f64), Shape)> {
    if minor <= 0.0 {
        return None;
    }
    let (lo, hi) = sine_range(phi);
    Some(((minor * lo, minor * hi), Shape::Arc { major, minor, phi }))
}

fn finite_profile(profile: Profile) -> Option<Profile> {
    let values = [
        profile.shift,
        profile.height_slack,
        profile.radial_slack,
        profile.scale,
        profile.heights.0,
        profile.heights.1,
    ];
    let shape_finite = match &profile.shape {
        Shape::Band { lo, hi } => lo.is_finite() && hi.is_finite(),
        Shape::Cone { tan } | Shape::ConicDisk { tan } => tan.is_finite() && *tan > 0.0,
        Shape::Arc { major, minor, phi } => {
            major.is_finite() && minor.is_finite() && phi.0.is_finite() && phi.1.is_finite()
        }
        Shape::Tube(tube) => [
            tube.across.length(),
            tube.drift.length(),
            tube.level,
            tube.rate,
            tube.span.0,
            tube.span.1,
            tube.radii.0,
            tube.radii.1,
            tube.sine,
            tube.lean.length(),
            tube.q2.length(),
            tube.angles.0,
            tube.angles.1,
        ]
        .iter()
        .all(|v| v.is_finite()),
    };
    (shape_finite && values.iter().all(|v| v.is_finite())).then_some(profile)
}

impl Tube {
    /// Reads a cone or cylinder (`origin`, unit `axis` of its carrier) about
    /// the frame axis through `frame_origin` along unit `frame_axis`:
    /// its heights there and the tube, or `None` when the axes are too close
    /// to parallel to set up the frame or a bound is unreadable.
    fn read(
        topo: &Topology,
        face: FaceId,
        origin: Point3,
        axis: Vec3,
        frame_origin: Point3,
        frame_axis: Vec3,
    ) -> Result<Option<(f64, f64, Self)>, crate::OperationsError> {
        let sine = axis.cross(frame_axis).length().min(1.0);
        if sine < 1e-6 {
            return Ok(None);
        }
        let Some(span) = boundary_heights(topo, face, origin, axis)? else {
            return Ok(None);
        };
        let (cone_tan, radii) = match topo.face(face)?.surface() {
            FaceSurface::Cylinder(cylinder) => {
                let Some(envelope) = radial_clearance::cylinder_envelope(topo, face, cylinder)
                else {
                    return Ok(None);
                };
                (None, (cylinder.radius(), envelope.max(cylinder.radius())))
            }
            FaceSurface::Cone(cone) => {
                let tan = cone.half_angle().tan();
                (Some(tan), cone_radii(span, tan))
            }
            FaceSurface::Plane { .. }
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_)
            | FaceSurface::Nurbs(_) => return Ok(None),
        };
        let Ok(f1) = (frame_axis - axis * axis.dot(frame_axis)).normalize() else {
            return Ok(None);
        };
        let f2 = axis.cross(f1);
        let across_of = |v: Vec3| v - frame_axis * v.dot(frame_axis);
        let Ok(q2) = frame_axis.cross(f2).normalize() else {
            return Ok(None);
        };
        let angles = carrier_angles(topo, face, origin, axis, f1, f2)?.unwrap_or((-PI, PI));
        let tube = Self {
            across: across_of(origin - frame_origin),
            drift: across_of(axis),
            level: (origin - frame_origin).dot(frame_axis),
            rate: axis.dot(frame_axis),
            span,
            cone_tan,
            radii,
            sine,
            lean: across_of(f1),
            f2,
            q2,
            angles,
        };
        let ends = [tube.level_at(span.0), tube.level_at(span.1)];
        let rise = radii.1 * sine;
        Ok(Some((
            ends[0].min(ends[1]) - rise,
            ends[0].max(ends[1]) + rise,
            tube,
        )))
    }

    fn level_at(&self, t: f64) -> f64 {
        self.rate.mul_add(t, self.level)
    }

    fn radii(&self, (low, high): (f64, f64)) -> Option<(f64, f64)> {
        let rise = self.radii.1 * self.sine;
        // Carrier parameters whose points can carry face points in the slab.
        let (mut t0, mut t1) = self.span;
        if self.rate.abs() > 0.0 {
            let a = (low - rise - self.level) / self.rate;
            let b = (high + rise - self.level) / self.rate;
            t0 = t0.max(a.min(b));
            t1 = t1.min(a.max(b));
        } else if self.level < low - rise || self.level > high + rise {
            return None;
        }
        if t0 > t1 {
            return None;
        }
        // Radii about the carrier axis over those parameters.
        let (r_lo, r_hi) = match self.cone_tan {
            Some(tan) => cone_radii((t0, t1), tan),
            None => self.radii,
        };
        // cos θ in the slab: `(h − level) / (ρ sine)` with `ρ ≥ r_lo`.
        let (l0, l1) = (self.level_at(t0), self.level_at(t1));
        let (drop_lo, drop_hi) = (low - l0.max(l1), high - l0.min(l1));
        let cosines = if r_lo > 0.0 {
            let (lo_den, hi_den) = (r_lo * self.sine, r_hi * self.sine);
            let lo = (drop_lo / lo_den).min(drop_lo / hi_den);
            let hi = (drop_hi / lo_den).max(drop_hi / hi_den);
            if lo > 1.0 || hi < -1.0 {
                return None;
            }
            (lo.max(-1.0), hi.min(1.0))
        } else {
            (-1.0, 1.0)
        };
        // Angles in the face with those cosines: [acos hi, acos lo] and its
        // mirror, each with a little slack.
        let (near, far) = (cosines.1.acos(), cosines.0.acos());
        let mut sin_range: Option<(f64, f64)> = None;
        let mut cos_range: Option<(f64, f64)> = None;
        for window in [
            (near - ANGLE_SLACK, far + ANGLE_SLACK),
            (-far - ANGLE_SLACK, -near + ANGLE_SLACK),
        ] {
            for piece in circular_overlaps(window, self.angles) {
                let c = cosine_range(piece);
                let s = sine_range(piece);
                cos_range = Some(cos_range.map_or(c, |(x, y)| (x.min(c.0), y.max(c.1))));
                sin_range = Some(sin_range.map_or(s, |(x, y)| (x.min(s.0), y.max(s.1))));
            }
        }
        let (Some(cos_range), Some(sin_range)) = (cos_range, sin_range) else {
            return None;
        };
        // The box of `ρ cos θ lean + ρ sin θ f2` in (q1 = f2, q2).
        let scale = |(a, b): (f64, f64)| {
            let products = [a * r_lo, a * r_hi, b * r_lo, b * r_hi];
            (
                products.iter().copied().fold(f64::INFINITY, f64::min),
                products.iter().copied().fold(f64::NEG_INFINITY, f64::max),
            )
        };
        let lean_q2 = self.lean.dot(self.q2);
        let lean_q1 = self.lean.dot(self.f2);
        let (sin_lo, sin_hi) = scale(sin_range);
        let (cos_lo, cos_hi) = scale(cos_range);
        let mut corners = Vec::with_capacity(8);
        for t in [t0, t1] {
            let offset = self.across + self.drift * t;
            let (x, y) = (offset.dot(self.f2), offset.dot(self.q2));
            for (rs, rc) in [
                (sin_lo, cos_lo),
                (sin_lo, cos_hi),
                (sin_hi, cos_lo),
                (sin_hi, cos_hi),
            ] {
                corners.push((x + rs + rc * lean_q1, y + rc * lean_q2));
            }
        }
        let (near, far) = hull_distances(&corners);
        Some((near, far))
    }
}

/// The angles a cone or cylinder face covers about its carrier axis
/// (`origin`, unit `axis`; `θ` from `f1` towards `f2`) when every boundary
/// edge is a line or a positive-weight NURBS curve whose (control) hull
/// stays clear of the axis, or an arc of a circle about the axis: the union
/// of those angles, when it is one arc short of a turn. `None` for any other
/// boundary.
fn carrier_angles(
    topo: &Topology,
    face: FaceId,
    origin: Point3,
    axis: Vec3,
    f1: Vec3,
    f2: Vec3,
) -> Result<Option<(f64, f64)>, crate::OperationsError> {
    let tol = Tolerance::new();
    let angle = |p: Point3| {
        let d = p - origin;
        d.dot(f2).atan2(d.dot(f1))
    };
    let mut arcs: Vec<(f64, f64)> = Vec::new();
    for id in remus_topology::explorer::face_edges(topo, face)? {
        let edge = topo.edge(id)?;
        let start = topo.vertex(edge.start())?.point();
        let end = topo.vertex(edge.end())?.point();
        match edge.curve() {
            // A segment kept clear of the axis turns less than a half turn
            // about it, monotonically between its ends' angles.
            EdgeCurve::Line => {
                let across = |p: Point3| {
                    let d = p - origin;
                    (d.dot(f1), d.dot(f2))
                };
                let (p, q) = (across(start), across(end));
                if hull_distances(&[p, q]).0 <= tol.linear {
                    return Ok(None);
                }
                let (a, b) = (p.1.atan2(p.0), q.1.atan2(q.0));
                let turn = (b - a + PI).rem_euclid(TAU) - PI;
                arcs.push(if turn >= 0.0 {
                    (a, a + turn)
                } else {
                    (a + turn, a)
                });
            }
            EdgeCurve::Circle(circle) => {
                let Ok(normal) = circle.normal().normalize() else {
                    return Ok(None);
                };
                if normal.cross(axis).length() > tol.angular
                    || (circle.center() - origin).cross(axis).length() > tol.linear
                {
                    return Ok(None);
                }
                let Ok((first, last)) = edge.strict_domain() else {
                    return Ok(None);
                };
                let middle =
                    edge.curve()
                        .evaluate_with_endpoints(f64::midpoint(first, last), start, end);
                let (a, m, b) = (angle(start), angle(middle), angle(end));
                let up = |x: f64| a + (x - a).rem_euclid(TAU);
                let (m, b) = (up(m), up(b));
                if (b - a).abs() <= ANGLE_SLACK || edge.start() == edge.end() {
                    // A whole circle covers every angle.
                    return Ok(None);
                }
                arcs.push(if m <= b { (a, b) } else { (b - TAU, a) });
            }
            // A positive-weight curve lies in its control points' hull; seen
            // from an axis that hull stays clear of, its angles lie within
            // the control points' (the hull spans less than a half turn).
            EdgeCurve::NurbsCurve(curve) if curve.weights().iter().all(|&w| w > 0.0) => {
                let across: Vec<(f64, f64)> = curve
                    .control_points()
                    .iter()
                    .chain([start, end].iter())
                    .map(|&p| {
                        let d = p - origin;
                        (d.dot(f1), d.dot(f2))
                    })
                    .collect();
                if hull_distances(&across).0 <= tol.linear {
                    return Ok(None);
                }
                let angles: Vec<f64> = across.iter().map(|&(x, y)| y.atan2(x)).collect();
                let Some(arc) = covering_arc(&angles) else {
                    return Ok(None);
                };
                arcs.push(arc);
            }
            EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Parabola(_)
            | EdgeCurve::Hyperbola(_) => return Ok(None),
        }
    }
    Ok(circular_union(&arcs).map(|(a, b)| (a - ANGLE_SLACK, b + ANGLE_SLACK)))
}

/// The least and greatest distances from the origin of the convex hull of
/// `points` in the plane: the greatest at a point, the least zero when the
/// hull holds the origin, else at the nearest hull edge.
fn hull_distances(points: &[(f64, f64)]) -> (f64, f64) {
    let far = points
        .iter()
        .map(|&(x, y)| x.hypot(y))
        .fold(0.0_f64, f64::max);
    // Andrew's monotone chain.
    let mut sorted = points.to_vec();
    sorted.compact_sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    let cross = |o: (f64, f64), a: (f64, f64), b: (f64, f64)| {
        (a.0 - o.0).mul_add(b.1 - o.1, -((a.1 - o.1) * (b.0 - o.0)))
    };
    let mut hull: Vec<(f64, f64)> = Vec::with_capacity(2 * sorted.len());
    for pass in 0..2 {
        let start = hull.len();
        let ordered: Vec<(f64, f64)> = if pass == 0 {
            sorted.clone()
        } else {
            sorted.iter().rev().copied().collect()
        };
        for p in ordered {
            while hull.len() >= start + 2
                && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0
            {
                hull.pop();
            }
            hull.push(p);
        }
        hull.pop();
    }
    let segment_distance = |a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let length = dx.mul_add(dx, dy * dy);
        let t = if length > 0.0 {
            (-a.0).mul_add(dx, -a.1 * dy) / length
        } else {
            0.0
        }
        .clamp(0.0, 1.0);
        t.mul_add(dx, a.0).hypot(t.mul_add(dy, a.1))
    };
    let count = hull.len();
    if count < 3 {
        let near = match count {
            0 => 0.0,
            1 => hull[0].0.hypot(hull[0].1),
            _ => segment_distance(hull[0], hull[1]),
        };
        return (near, far);
    }
    let mut inside = true;
    let mut near = f64::INFINITY;
    for i in 0..count {
        let (a, b) = (hull[i], hull[(i + 1) % count]);
        // Counter-clockwise hull: the origin is inside when left of every edge.
        if cross(a, b, (0.0, 0.0)) < 0.0 {
            inside = false;
        }
        near = near.min(segment_distance(a, b));
    }
    (if inside { 0.0 } else { near }, far)
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

/// Radii `major + minor cos φ` of the arc's points (`φ` in `phi`) whose
/// heights `minor sin φ` lie in `low..high`.
fn arc_radii(
    major: f64,
    minor: f64,
    phi: (f64, f64),
    (low, high): (f64, f64),
) -> Option<(f64, f64)> {
    let lo_sine = low / minor;
    let hi_sine = high / minor;
    if lo_sine > 1.0 || hi_sine < -1.0 {
        return None;
    }
    let s0 = lo_sine.max(-1.0).asin() - ANGLE_SLACK;
    let s1 = hi_sine.min(1.0).asin() + ANGLE_SLACK;
    // Angles with sine in range: [s0, s1] and its mirror [π − s1, π − s0].
    let mut cos_range: Option<(f64, f64)> = None;
    for window in [(s0, s1), (PI - s1, PI - s0)] {
        for (a, b) in circular_overlaps(window, phi) {
            let (c_lo, c_hi) = cosine_range((a, b));
            cos_range = Some(cos_range.map_or((c_lo, c_hi), |(x, y)| (x.min(c_lo), y.max(c_hi))));
        }
    }
    let (c_lo, c_hi) = cos_range?;
    Some((minor.mul_add(c_lo, major), minor.mul_add(c_hi, major)))
}

/// The pieces of `window` that the circular interval `arc` covers, both
/// given as increasing angles.
fn circular_overlaps(window: (f64, f64), arc: (f64, f64)) -> Vec<(f64, f64)> {
    if arc.1 - arc.0 >= TAU {
        return vec![window];
    }
    let mut pieces = Vec::new();
    // Shift the arc by whole turns to every position that can meet `window`.
    let first = ((window.0 - arc.1) / TAU).floor();
    let last = ((window.1 - arc.0) / TAU).ceil();
    // Windows and arcs here span at most a turn each, so a handful of shifts
    // covers them; anything wider (or unreadable) keeps the whole window.
    if !(last - first).is_finite() || last - first > 4.0 {
        return vec![window];
    }
    for step in [0.0, 1.0, 2.0, 3.0, 4.0] {
        let k = first + step;
        if k > last {
            break;
        }
        let a = window.0.max(TAU.mul_add(k, arc.0));
        let b = window.1.min(TAU.mul_add(k, arc.1));
        if a <= b {
            pieces.push((a, b));
        }
    }
    pieces
}

/// The range of `cos` over the angles `a..b`.
fn cosine_range((a, b): (f64, f64)) -> (f64, f64) {
    if b - a >= TAU {
        return (-1.0, 1.0);
    }
    let contains = |target: f64| TAU.mul_add(((a - target) / TAU).ceil(), target) <= b;
    let hi = if contains(0.0) {
        1.0
    } else {
        a.cos().max(b.cos())
    };
    let lo = if contains(PI) {
        -1.0
    } else {
        a.cos().min(b.cos())
    };
    (lo, hi)
}

/// The range of `sin` over the angles `phi`.
fn sine_range(phi: (f64, f64)) -> (f64, f64) {
    cosine_range((phi.0 - FRAC_PI_2, phi.1 - FRAC_PI_2))
}

/// The one arc that `arcs` cover together, when their union is a single arc
/// short of a turn (merged on the circle, across the seam too). `None` when
/// they leave more than one piece (which side of a gap a face takes is not
/// decided by its boundary alone), cover the circle, or are none.
///
/// A face that does not wrap its carrier is bounded in angle by its outer
/// loop, whose angles lifted continuously form one interval; when every
/// boundary edge's angles merge into one arc short of a turn, that arc holds
/// the projection of that interval, and so the face.
fn circular_union(arcs: &[(f64, f64)]) -> Option<(f64, f64)> {
    let mut pieces: Vec<(f64, f64)> = arcs
        .iter()
        .map(|&(a, b)| {
            let start = a.rem_euclid(TAU);
            (start, start + (b - a))
        })
        .collect();
    if pieces.is_empty()
        || pieces
            .iter()
            .any(|&(a, b)| (b - a).partial_cmp(&TAU) != Some(std::cmp::Ordering::Less))
    {
        return None;
    }
    pieces.compact_sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut merged: Vec<(f64, f64)> = Vec::new();
    for (a, b) in pieces {
        match merged.last_mut() {
            Some(last) if a <= last.1 => last.1 = last.1.max(b),
            _ => merged.push((a, b)),
        }
    }
    // A last piece reaching past a turn joins the first across the seam.
    while merged.len() > 1 {
        let first = merged[0];
        let last = merged.len() - 1;
        if merged[last].1 >= first.0 + TAU {
            merged[last].1 = merged[last].1.max(first.1 + TAU);
            merged.remove(0);
        } else {
            break;
        }
    }
    match merged.as_slice() {
        [(a, b)] if b - a < TAU => Some((*a, *b)),
        _ => None,
    }
}

/// The smallest arc holding every angle of `angles`, all of which lie within
/// less than a half turn of each other (the complement of the largest gap
/// between them). `None` when there are none or they spread wider.
fn covering_arc(angles: &[f64]) -> Option<(f64, f64)> {
    let mut sorted: Vec<f64> = angles.iter().map(|a| a.rem_euclid(TAU)).collect();
    if sorted.is_empty() || sorted.iter().any(|a| !a.is_finite()) {
        return None;
    }
    sorted.compact_sort_by(f64::total_cmp);
    let count = sorted.len();
    let (mut gap, mut after) = (sorted[0] + TAU - sorted[count - 1], 0);
    for i in 1..count {
        let candidate = sorted[i] - sorted[i - 1];
        if candidate > gap {
            (gap, after) = (candidate, i);
        }
    }
    let start = sorted[after];
    let span = TAU - gap;
    (span < PI).then_some((start, start + span))
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

/// A box certified to hold `face`: the face bound where it is certified,
/// and for a cone or cylinder, the box of its carrier between the heights
/// of its boundary along the carrier's axis. `None` otherwise.
fn certified_box(topo: &Topology, face: FaceId) -> Result<Option<Aabb3>, crate::OperationsError> {
    if let Ok(bound) = remus_check::distance::face_bounds::face_bound(topo, face)
        && bound.prunable
    {
        return Ok(Some(bound.aabb));
    }
    // A circle of `radius` about `axis` lies within this box of its centre.
    let disc_box = |center: Point3, axis: Vec3, radius: f64| {
        let spread = |a: f64| radius * (1.0 - a * a).max(0.0).sqrt();
        let half = Vec3::new(spread(axis.x()), spread(axis.y()), spread(axis.z()));
        Aabb3 {
            min: center - half,
            max: center + half,
        }
    };
    let tol = Tolerance::new().linear;
    let pad = |bounds: Aabb3| Aabb3 {
        min: bounds.min - Vec3::new(tol, tol, tol),
        max: bounds.max + Vec3::new(tol, tol, tol),
    };
    match topo.face(face)?.surface() {
        FaceSurface::Cylinder(cylinder) => {
            let Ok(axis) = cylinder.axis().normalize() else {
                return Ok(None);
            };
            let origin = cylinder.origin();
            let (Some((low, high)), Some(radius)) = (
                boundary_heights(topo, face, origin, axis)?,
                radial_clearance::cylinder_envelope(topo, face, cylinder),
            ) else {
                return Ok(None);
            };
            let radius = radius.max(cylinder.radius());
            Ok(Some(pad(disc_box(origin + axis * low, axis, radius)
                .union(disc_box(origin + axis * high, axis, radius)))))
        }
        FaceSurface::Cone(cone) => {
            let Ok(axis) = cone.axis().normalize() else {
                return Ok(None);
            };
            let apex = cone.apex();
            let Some((low, high)) = boundary_heights(topo, face, apex, axis)? else {
                return Ok(None);
            };
            let tan = cone.half_angle().tan();
            let mut bounds = disc_box(apex + axis * low, axis, low.abs() / tan).union(disc_box(
                apex + axis * high,
                axis,
                high.abs() / tan,
            ));
            if low <= 0.0 && high >= 0.0 {
                bounds = bounds.union(Aabb3 {
                    min: apex,
                    max: apex,
                });
            }
            Ok(Some(pad(bounds)))
        }
        FaceSurface::Plane { .. }
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_)
        | FaceSurface::Nurbs(_) => Ok(None),
    }
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

fn box_magnitude(bounds: Aabb3) -> f64 {
    magnitude(bounds.min).max(magnitude(bounds.max))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
    use super::{
        Companion, Profile, Shape, Sweep, arc_radii, circular_union, cone_radii, cosine_range,
    };
    use crate::primitives::{make_cone, make_cylinder, make_torus};
    use crate::transform::transform_solid;
    use remus_math::aabb::Aabb3;
    use remus_math::{
        mat::Mat4,
        surfaces::CylindricalSurface,
        vec::{Point3, Vec3},
    };
    use remus_topology::{
        Topology,
        explorer::solid_faces,
        face::{FaceId, FaceSurface},
        solid::SolidId,
    };
    use std::f64::consts::{FRAC_PI_2, PI, TAU};

    const UP: Vec3 = Vec3::new(0.0, 0.0, 1.0);

    fn faces_where(
        topo: &Topology,
        solid: SolidId,
        keep: impl Fn(&FaceSurface) -> bool,
    ) -> Vec<FaceId> {
        solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .filter(|&face| keep(topo.face(face).unwrap().surface()))
            .collect()
    }

    /// The conical face of a frustum from radius `bottom` at `z0` to `top`
    /// one unit higher: a chamfer travelling with a cap.
    fn chamfer(topo: &mut Topology, bottom: f64, top: f64, z0: f64) -> FaceId {
        let cone = make_cone(topo, bottom, top, 1.0).unwrap();
        transform_solid(topo, cone, &Mat4::translation(0.0, 0.0, z0)).unwrap();
        let [face] = faces_where(topo, cone, |s| matches!(s, FaceSurface::Cone(_)))[..] else {
            panic!("one conical face");
        };
        face
    }

    /// Whether `moving`, carried up by `travel`, may reach any of `fixed`.
    fn blocked(topo: &Topology, moving: FaceId, fixed: &[FaceId], travel: f64) -> bool {
        Companion::prepare(topo, moving, fixed, &|_| false, &|_| false, UP)
            .unwrap()
            .blocking(UP * travel)
            .is_some()
    }

    /// Review (Sol, #1956): a fixed torus bulging over the chamfer from the
    /// far end of their shared support. The torus underside is 0.226 above
    /// the chamfer at its nearest, so a 0.35 move reaches it and a 0.1 move
    /// does not, however far apart their contacts with the support stay.
    #[test]
    fn a_torus_overhanging_the_chamfer_is_reached_before_its_rim() {
        let mut topo = Topology::new();
        let moving = chamfer(&mut topo, 9.0, 12.0, -1.0);
        let torus = make_torus(&mut topo, 9.5, 2.0, 32).unwrap();
        transform_solid(&mut topo, torus, &Mat4::translation(0.0, 0.0, 1.5)).unwrap();
        let fixed = faces_where(&topo, torus, |s| matches!(s, FaceSurface::Torus(_)));
        assert!(!fixed.is_empty());
        assert!(blocked(&topo, moving, &fixed, 0.35));
        assert!(!blocked(&topo, moving, &fixed, 0.1));
    }

    /// Review (Sol, #1956): a disc whose circular edge has its seam at 45°,
    /// where a sampled box would stop at radius 9.12 about the axis though
    /// the disc reaches 9.5. The #1949 chamfer moved 0.4 sweeps radii
    /// 9.3..9.7 at the disc's height.
    #[test]
    fn a_disc_is_bounded_by_its_whole_circle_not_its_samples() {
        let mut topo = Topology::new();
        let moving = chamfer(&mut topo, 9.0, 10.0, -1.0);
        let disc = make_cylinder(&mut topo, 1.5, 0.1).unwrap();
        transform_solid(&mut topo, disc, &Mat4::rotation_z(PI / 4.0)).unwrap();
        transform_solid(&mut topo, disc, &Mat4::translation(8.0, 0.0, -0.4)).unwrap();
        let top: Vec<FaceId> = faces_where(&topo, disc, |s| matches!(s, FaceSurface::Plane { .. }))
            .into_iter()
            .filter(|&f| topo.face(f).unwrap().effective_plane_normal().unwrap().z() > 0.5)
            .collect();
        assert_eq!(top.len(), 1);
        assert!(blocked(&topo, moving, &top, 0.4));
        // Moved 0.1 the chamfer stays at radius 9.6 or more there.
        assert!(!blocked(&topo, moving, &top, 0.1));
    }

    /// Review (Sol, #1956): a cylinder whose carrier origin lies 1e12 below
    /// the face and whose axis is tilted 5e-13 (parallel within tolerance)
    /// passes 0.5 off the companion's axis at the face. Its wall reaches
    /// radius 9.7 there, inside the chamfer's sweep.
    #[test]
    fn a_far_tilted_carrier_is_charged_its_offset_at_the_face() {
        let mut topo = Topology::new();
        let moving = chamfer(&mut topo, 9.0, 10.0, -1.0);
        let band = make_cylinder(&mut topo, 9.2, 0.1).unwrap();
        transform_solid(&mut topo, band, &Mat4::translation(0.0, 0.0, -0.25)).unwrap();
        let [wall] = faces_where(&topo, band, |s| matches!(s, FaceSurface::Cylinder(_)))[..] else {
            panic!("one cylindrical face");
        };
        let tilted = CylindricalSurface::new(
            Point3::new(0.0, 0.0, -1e12),
            Vec3::new(5e-13, 0.0, 1.0),
            9.2,
        )
        .unwrap();
        topo.face_mut(wall)
            .unwrap()
            .set_surface(FaceSurface::Cylinder(tilted));
        assert!(blocked(&topo, moving, &[wall], 0.4));
    }

    /// A cone near the world origin read about an axis whose origin lies
    /// `2^40` away: its box's heights round together in that frame, and must
    /// still hold the cone's own heights (review of #1956).
    #[test]
    fn box_heights_survive_a_far_frame_origin() {
        let far = 2.0_f64.powi(40);
        let sweep = Sweep {
            origin: Point3::new(0.0, 0.0, far),
            axis: UP,
            profile: Profile {
                shift: 0.0,
                height_slack: 0.0,
                radial_slack: 0.0,
                heights: (-2.0, -1.0),
                shape: Shape::Band { lo: 16.0, hi: 16.0 },
                scale: far,
            },
            bounds: None,
            scale: far,
        };
        let top = 2.0_f64.powi(-15);
        let tol = 1e-7;
        let bounds = Aabb3 {
            min: Point3::new(-32.0 - tol, -32.0 - tol, -tol),
            max: Point3::new(32.0 + tol, 32.0 + tol, top + tol),
        };
        let (low, high) = sweep.box_heights((0.0, top), Some(bounds), -far, 0.0);
        assert!(low <= 0.0 && high >= top, "narrowed to {low}..{high}");
    }

    #[test]
    fn cone_radii_cross_the_apex() {
        assert_eq!(cone_radii((1.0, 2.0), 1.0), (1.0, 2.0));
        assert_eq!(cone_radii((-2.0, -1.0), 2.0), (0.5, 1.0));
        assert_eq!(cone_radii((-1.0, 3.0), 1.0), (0.0, 3.0));
    }

    #[test]
    fn cosine_range_finds_interior_extremes() {
        let (lo, hi) = cosine_range((-0.5, 0.5));
        assert!((hi - 1.0).abs() < 1e-15 && (lo - 0.5_f64.cos()).abs() < 1e-15);
        assert!((cosine_range((3.0, 3.5)).0 + 1.0).abs() < 1e-15);
        let (lo, hi) = cosine_range((TAU + 0.1, TAU + 0.2));
        assert!((hi - 0.1_f64.cos()).abs() < 1e-12 && (lo - 0.2_f64.cos()).abs() < 1e-12);
    }

    #[test]
    fn arc_radii_follow_the_tube() {
        // Torus R = 10, r = 2, the inner half (φ in [π/2, 3π/2]).
        let inner = (FRAC_PI_2, 3.0 * FRAC_PI_2);
        let (lo, hi) = arc_radii(10.0, 2.0, inner, (-0.1, 0.1)).unwrap();
        assert!((lo - 8.0).abs() < 1e-8 && hi < 8.01 && hi > 8.0);
        // Above the tube nothing lies.
        assert!(arc_radii(10.0, 2.0, inner, (2.5, 3.0)).is_none());
        // The full tube at mid-height covers both walls.
        let (lo, hi) = arc_radii(10.0, 2.0, (-PI, PI), (-0.1, 0.1)).unwrap();
        assert!((lo - 8.0).abs() < 1e-8 && (hi - 12.0).abs() < 1e-8);
        // A quarter (φ in [0, π/2]) at heights 1.9..2.0 is near the top.
        let (lo, hi) = arc_radii(10.0, 2.0, (0.0, FRAC_PI_2), (1.9, 2.0)).unwrap();
        assert!(lo >= 10.0 - 1e-8 && hi < 10.7);
    }

    #[test]
    fn circular_union_takes_one_piece_only() {
        let (a, b) =
            circular_union(&[(0.0, 0.0), (0.0, FRAC_PI_2), (FRAC_PI_2, FRAC_PI_2)]).unwrap();
        assert!(a.abs() < 1e-15 && (b - FRAC_PI_2).abs() < 1e-15);
        // An arc across the wrap.
        let (a, b) = circular_union(&[(-0.5, 0.5)]).unwrap();
        assert!((b - a - 1.0).abs() < 1e-12);
        // Pieces meeting across the seam join.
        let (a, b) = circular_union(&[(-FRAC_PI_2, 0.0), (0.0, PI), (-0.2, 0.0)]).unwrap();
        assert!((b - a - 1.5 * PI).abs() < 1e-12);
        // Two rims alone do not say which band lies between them.
        assert!(circular_union(&[(-0.5, -0.5), (0.5, 0.5)]).is_none());
        assert!(circular_union(&[(0.0, TAU)]).is_none());
    }

    #[test]
    fn covering_arc_spans_close_angles_only() {
        let (a, b) = super::covering_arc(&[3.0, -3.0, PI]).unwrap();
        assert!((b - a - (TAU - 6.0)).abs() < 1e-12);
        assert!(super::covering_arc(&[0.0, 2.0, 4.0]).is_none());
    }

    /// A flat NURBS patch `(10u, v, 0)` trimmed to `x ≤ 2`, except for a
    /// tongue its right edge pushes out to `x = 8` at `y = 0.5` between the
    /// edge's boundary samples (issue #1950).
    fn lobed_nurbs_face(topo: &mut Topology) -> FaceId {
        use remus_math::nurbs::{curve::NurbsCurve, surface::NurbsSurface};
        use remus_topology::{
            edge::{Edge, EdgeCurve},
            face::Face,
            vertex::Vertex,
            wire::{OrientedEdge, Wire},
        };
        let surface = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
                vec![Point3::new(10.0, 0.0, 0.0), Point3::new(10.0, 1.0, 0.0)],
            ],
            vec![vec![1.0; 2]; 2],
        )
        .unwrap();
        let tongue = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 0.02, 0.06, 0.1, 1.0, 1.0],
            vec![
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(2.0, 0.45, 0.0),
                Point3::new(8.0, 0.5, 0.0),
                Point3::new(2.0, 0.55, 0.0),
                Point3::new(2.0, 1.0, 0.0),
            ],
            vec![1.0; 5],
        )
        .unwrap();
        let corners = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]
            .map(|(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), 1e-7)));
        let curves = [
            EdgeCurve::Line,
            EdgeCurve::NurbsCurve(tongue),
            EdgeCurve::Line,
            EdgeCurve::Line,
        ];
        let uses = curves
            .into_iter()
            .enumerate()
            .map(|(i, curve)| {
                let trim = matches!(curve, EdgeCurve::NurbsCurve(_)).then_some((0.0, 1.0));
                let mut edge = Edge::new(corners[i], corners[(i + 1) % 4], curve);
                edge.set_trim(trim);
                OrientedEdge::new(topo.add_edge(edge), true)
            })
            .collect();
        let wire = topo.add_wire(Wire::new(uses, true).unwrap());
        topo.add_face(Face::new(wire, Vec::new(), FaceSurface::Nurbs(surface)))
    }

    /// Issue #1950: a carried NURBS support with a narrow trim lobe between
    /// its boundary samples. An L-shaped cap at the support's height has a
    /// box over a fixed block in the lobe's path but a footprint clear of
    /// it, so only the companion test stands between the lobe and the
    /// block: the support's box must hold the lobe for the move to refuse.
    #[test]
    fn a_nurbs_companion_lobe_between_samples_still_reaches() {
        use super::super::{SweptCandidate, SweptFace, SweptRegionCheck};
        use crate::primitives::make_box;
        use remus_topology::{
            edge::{Edge, EdgeCurve},
            face::Face,
            vertex::Vertex,
            wire::{OrientedEdge, Wire},
        };

        let mut topo = Topology::new();
        let support = lobed_nurbs_face(&mut topo);
        let sampled = crate::measure::face_bounding_box(&topo, support).unwrap();
        assert!(
            sampled.max.x() < 3.0,
            "fixture no longer hides the tongue from the sampled box: {sampled:?}"
        );

        // The cap: the strips y in [-2, -1] and x in [11, 12] at z = 0.
        let outline = [
            (-1.0, -2.0),
            (12.0, -2.0),
            (12.0, 3.0),
            (11.0, 3.0),
            (11.0, -1.0),
            (-1.0, -1.0),
        ]
        .map(|(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), 1e-7)));
        let uses = (0..outline.len())
            .map(|i| {
                let edge = topo.add_edge(Edge::new(
                    outline[i],
                    outline[(i + 1) % outline.len()],
                    EdgeCurve::Line,
                ));
                OrientedEdge::new(edge, true)
            })
            .collect();
        let wire = topo.add_wire(Wire::new(uses, true).unwrap());
        let cap = topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane { normal: UP, d: 0.0 },
        ));

        // A fixed block over the tongue, 0.5 above it.
        let block = make_box(&mut topo, 1.0, 0.2, 0.1).unwrap();
        transform_solid(&mut topo, block, &Mat4::translation(5.0, 0.4, 0.5)).unwrap();
        let obstacle = faces_where(&topo, block, |_| true)
            .into_iter()
            .find(|&f| topo.face(f).unwrap().effective_plane_normal().unwrap().z() < -0.5)
            .unwrap();
        let bounds = crate::measure::face_bounding_box(&topo, obstacle).unwrap();

        let check = |travel: f64| {
            SweptRegionCheck {
                normal: UP,
                swept: vec![SweptFace {
                    face: cap,
                    original: crate::measure::face_bounding_box(&topo, cap).unwrap(),
                    prismatic: false,
                    limiting_cone: None,
                    candidates: vec![SweptCandidate {
                        face: obstacle,
                        bounds,
                        overlaps_source: false,
                        radially_clear: false,
                    }],
                }],
                companions: vec![
                    Companion::prepare(&topo, support, &[obstacle], &|_| false, &|_| false, UP)
                        .unwrap(),
                ],
            }
            .check_rigid_translation(&topo, travel)
        };
        // Moved 1, the tongue at x = 5..6 rises through the block's base.
        let error = check(1.0).expect_err("the tongue's sweep reaches the block");
        assert!(matches!(
            error,
            crate::OperationsError::Offset(remus_offset::OffsetError::TopologyChange {
                face: Some(face), ..
            }) if face == cap
        ));
        // Moved 0.25 nothing reaches it.
        assert!(check(0.25).is_ok());
    }
}
