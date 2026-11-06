//! Closed-form sharp geometry of blend supports: the edge two supports meet
//! in, and the points where that edge meets a third face.
//!
//! Only intersections with an exact elementary answer are offered: plane,
//! cylinder and sphere pairs whose section is a line or a circle. Anything
//! else (an oblique plane through a cylinder, a torus or cone support) is a
//! refusal; this module never samples or iterates.

use remus_math::vec::{Point3, Vec3};
use remus_topology::face::FaceSurface;

/// Relative scale used to decide that two directions are parallel or that a
/// discriminant vanishes.
const PARALLEL: f64 = 1e-12;

/// Elementary carrier of a support face, orientation ignored.
#[derive(Clone, Copy, Debug)]
pub(super) enum Carrier {
    /// Points with `normal · p = d`, `normal` unit.
    Plane { normal: Vec3, d: f64 },
    /// Points at `radius` from the line through `origin` along unit `axis`.
    Cylinder {
        origin: Point3,
        axis: Vec3,
        radius: f64,
    },
    /// Points at `radius` from `center`.
    Sphere { center: Point3, radius: f64 },
}

impl Carrier {
    pub(super) fn of(surface: &FaceSurface) -> Option<Self> {
        match surface {
            FaceSurface::Plane { normal, d } => {
                let length = normal.length();
                (length > 0.0).then(|| Self::Plane {
                    normal: *normal * (1.0 / length),
                    d: d / length,
                })
            }
            FaceSurface::Cylinder(cylinder) => Some(Self::Cylinder {
                origin: cylinder.origin(),
                axis: cylinder.axis(),
                radius: cylinder.radius(),
            }),
            FaceSurface::Sphere(sphere) => Some(Self::Sphere {
                center: sphere.center(),
                radius: sphere.radius(),
            }),
            // A torus or cone support meets its neighbours in curves with no
            // line/circle form in general; NURBS has no closed form at all.
            FaceSurface::Torus(_) | FaceSurface::Cone(_) | FaceSurface::Nurbs(_) => None,
        }
    }

    /// Unsigned distance from `point` to the carrier.
    pub(super) fn distance(self, point: Point3) -> f64 {
        match self {
            Self::Plane { normal, d } => (normal.dot(position(point)) - d).abs(),
            Self::Cylinder {
                origin,
                axis,
                radius,
            } => (radial(point - origin, axis).length() - radius).abs(),
            Self::Sphere { center, radius } => ((point - center).length() - radius).abs(),
        }
    }

    const fn name(self) -> &'static str {
        match self {
            Self::Plane { .. } => "plane",
            Self::Cylinder { .. } => "cylinder",
            Self::Sphere { .. } => "sphere",
        }
    }
}

/// The exact section of two supports near a band.
#[derive(Clone, Copy, Debug)]
pub(super) enum Section {
    /// The line through `point` along unit `direction`.
    Line { point: Point3, direction: Vec3 },
    /// The circle about unit `axis` through `center`.
    Circle {
        center: Point3,
        axis: Vec3,
        radius: f64,
    },
}

impl Section {
    /// Closest point of the section to `point`.
    pub(super) fn project(self, point: Point3) -> Option<Point3> {
        match self {
            Self::Line {
                point: base,
                direction,
            } => Some(base + direction * (point - base).dot(direction)),
            Self::Circle {
                center,
                axis,
                radius,
            } => {
                let planar = radial(point - center, axis);
                let length = planar.length();
                (length > PARALLEL * radius.max(1.0)).then(|| center + planar * (radius / length))
            }
        }
    }
}

fn radial(vector: Vec3, axis: Vec3) -> Vec3 {
    vector - axis * vector.dot(axis)
}

fn nearest(points: Vec<Point3>, reference: Point3) -> Option<Point3> {
    points.into_iter().min_by(|a, b| {
        (*a - reference)
            .length()
            .total_cmp(&(*b - reference).length())
    })
}

/// The exact section of two support carriers, choosing the branch nearest
/// `reference` (a point on the band the section replaces).
pub(super) fn section(
    first: Carrier,
    second: Carrier,
    reference: Point3,
) -> Result<Section, String> {
    let refuse = || {
        format!(
            "{} x {} has no line or circle section here",
            first.name(),
            second.name()
        )
    };
    match (first, second) {
        (Carrier::Plane { normal: n1, d: d1 }, Carrier::Plane { normal: n2, d: d2 }) => {
            let direction = n1.cross(n2);
            let square = direction.dot(direction);
            if square < PARALLEL {
                return Err("parallel planes have no sharp edge".into());
            }
            let point = at((n2.cross(direction) * d1 + direction.cross(n1) * d2) * (1.0 / square));
            Ok(Section::Line {
                point,
                direction: direction * (1.0 / square.sqrt()),
            })
        }
        (
            Carrier::Plane { normal, d },
            Carrier::Cylinder {
                origin,
                axis,
                radius,
            },
        )
        | (
            Carrier::Cylinder {
                origin,
                axis,
                radius,
            },
            Carrier::Plane { normal, d },
        ) => {
            let along = normal.dot(axis);
            if (along.abs() - 1.0).abs() < 1e-9 {
                let travel = (d - normal.dot(position(origin))) / along;
                return Ok(Section::Circle {
                    center: origin + axis * travel,
                    axis,
                    radius,
                });
            }
            if along.abs() > 1e-9 {
                return Err(refuse());
            }
            let height = normal.dot(position(origin)) - d;
            let square = radius.mul_add(radius, -height * height);
            if square < -PARALLEL * radius * radius {
                return Err("plane misses the cylinder".into());
            }
            let half = square.max(0.0).sqrt();
            let base = origin - normal * height;
            let lateral = axis.cross(normal);
            let lines = vec![base + lateral * half, base - lateral * half];
            let point = nearest(
                lines
                    .into_iter()
                    .map(|line| line + axis * (reference - line).dot(axis))
                    .collect(),
                reference,
            )
            .ok_or_else(refuse)?;
            Ok(Section::Line {
                point,
                direction: axis,
            })
        }
        (Carrier::Plane { normal, d }, Carrier::Sphere { center, radius })
        | (Carrier::Sphere { center, radius }, Carrier::Plane { normal, d }) => {
            let height = normal.dot(position(center)) - d;
            let square = radius.mul_add(radius, -height * height);
            if square <= 0.0 {
                return Err("plane misses the sphere".into());
            }
            Ok(Section::Circle {
                center: center - normal * height,
                axis: normal,
                radius: square.sqrt(),
            })
        }
        (
            Carrier::Cylinder {
                origin: o1,
                axis: a1,
                radius: r1,
            },
            Carrier::Cylinder {
                origin: o2,
                axis: a2,
                radius: r2,
            },
        ) => {
            if a1.cross(a2).length() > 1e-9 {
                return Err(refuse());
            }
            let points =
                circle_circle(o1, r1, o2 + a1 * (o1 - o2).dot(a1), r2, a1).ok_or_else(refuse)?;
            let point = nearest(
                points
                    .into_iter()
                    .map(|line| line + a1 * (reference - line).dot(a1))
                    .collect(),
                reference,
            )
            .ok_or_else(refuse)?;
            Ok(Section::Line {
                point,
                direction: a1,
            })
        }
        (
            Carrier::Cylinder {
                origin,
                axis,
                radius,
            },
            Carrier::Sphere {
                center,
                radius: sphere,
            },
        )
        | (
            Carrier::Sphere {
                center,
                radius: sphere,
            },
            Carrier::Cylinder {
                origin,
                axis,
                radius,
            },
        ) => {
            if radial(center - origin, axis).length() > 1e-9 * radius.max(1.0) || sphere <= radius {
                return Err(refuse());
            }
            let offset = sphere.mul_add(sphere, -radius * radius).sqrt();
            let centers = vec![center + axis * offset, center - axis * offset];
            let center = nearest(centers, reference).ok_or_else(refuse)?;
            Ok(Section::Circle {
                center,
                axis,
                radius,
            })
        }
        (
            Carrier::Sphere {
                center: c1,
                radius: r1,
            },
            Carrier::Sphere {
                center: c2,
                radius: r2,
            },
        ) => {
            let between = c2 - c1;
            let distance = between.length();
            if distance <= PARALLEL || distance >= r1 + r2 || distance <= (r1 - r2).abs() {
                return Err(refuse());
            }
            let axis = between * (1.0 / distance);
            let along = (distance * distance + r1 * r1 - r2 * r2) / (2.0 * distance);
            Ok(Section::Circle {
                center: c1 + axis * along,
                axis,
                radius: r1.mul_add(r1, -along * along).sqrt(),
            })
        }
    }
}

/// Intersections of two circles in one plane (both about `axis`).
fn circle_circle(c1: Point3, r1: f64, c2: Point3, r2: f64, axis: Vec3) -> Option<Vec<Point3>> {
    let between = radial(c2 - c1, axis);
    let distance = between.length();
    if distance <= PARALLEL * r1.max(r2).max(1.0) {
        return None;
    }
    let scale = r1.max(r2).max(1.0);
    if distance > r1 + r2 + 1e-9 * scale || distance < (r1 - r2).abs() - 1e-9 * scale {
        return None;
    }
    let toward = between * (1.0 / distance);
    let along = (distance * distance + r1 * r1 - r2 * r2) / (2.0 * distance);
    let half = r1.mul_add(r1, -along * along).max(0.0).sqrt();
    let across = axis.cross(toward);
    let foot = c1 + toward * along;
    Some(vec![foot + across * half, foot - across * half])
}

/// Points where a section meets a third carrier.
pub(super) fn meet(section: Section, carrier: Carrier) -> Result<Vec<Point3>, String> {
    match (section, carrier) {
        (Section::Line { point, direction }, Carrier::Plane { normal, d }) => {
            let denominator = normal.dot(direction);
            if denominator.abs() < 1e-12 {
                return Ok(Vec::new());
            }
            let travel = (d - normal.dot(position(point))) / denominator;
            Ok(vec![point + direction * travel])
        }
        (
            Section::Line { point, direction },
            Carrier::Cylinder {
                origin,
                axis,
                radius,
            },
        ) => Ok(line_quadric(
            radial(point - origin, axis),
            radial(direction, axis),
            radius,
        )
        .into_iter()
        .map(|travel| point + direction * travel)
        .collect()),
        (Section::Line { point, direction }, Carrier::Sphere { center, radius }) => {
            Ok(line_quadric(point - center, direction, radius)
                .into_iter()
                .map(|travel| point + direction * travel)
                .collect())
        }
        (
            Section::Circle {
                center,
                axis,
                radius,
            },
            Carrier::Plane { normal, d },
        ) => {
            let direction = axis.cross(normal);
            let square = direction.dot(direction);
            if square < PARALLEL {
                return Ok(Vec::new());
            }
            let own = axis.dot(position(center));
            let point =
                at((normal.cross(direction) * own + direction.cross(axis) * d) * (1.0 / square));
            let direction = direction * (1.0 / square.sqrt());
            Ok(line_quadric(point - center, direction, radius)
                .into_iter()
                .map(|travel| point + direction * travel)
                .collect())
        }
        (
            Section::Circle {
                center,
                axis,
                radius,
            },
            Carrier::Cylinder {
                origin,
                axis: cylinder_axis,
                radius: cylinder_radius,
            },
        ) => {
            if axis.cross(cylinder_axis).length() > 1e-9 {
                return Err("circle meets an oblique cylinder".into());
            }
            Ok(circle_circle(
                center,
                radius,
                origin + axis * (center - origin).dot(axis),
                cylinder_radius,
                axis,
            )
            .unwrap_or_default())
        }
        (
            Section::Circle {
                center,
                axis,
                radius,
            },
            Carrier::Sphere {
                center: sphere_center,
                radius: sphere_radius,
            },
        ) => {
            let height = axis.dot(sphere_center - center);
            let square = sphere_radius.mul_add(sphere_radius, -height * height);
            if square <= 0.0 {
                return Ok(Vec::new());
            }
            Ok(circle_circle(
                center,
                radius,
                sphere_center - axis * height,
                square.sqrt(),
                axis,
            )
            .unwrap_or_default())
        }
    }
}

/// Real roots `t` of `|offset + t·direction|² = radius²`, with a vanishing
/// discriminant (a tangent meeting) read as one double root.
fn line_quadric(offset: Vec3, direction: Vec3, radius: f64) -> Vec<f64> {
    let a = direction.dot(direction);
    if a < PARALLEL {
        return Vec::new();
    }
    let b = offset.dot(direction);
    let c = offset.dot(offset) - radius * radius;
    let discriminant = b.mul_add(b, -a * c);
    let scale = b.mul_add(b, (a * c).abs()).max(PARALLEL);
    if discriminant < -1e-9 * scale {
        return Vec::new();
    }
    let root = discriminant.max(0.0).sqrt();
    if root <= 1e-9 * scale.sqrt() {
        return vec![-b / a];
    }
    vec![(-b - root) / a, (-b + root) / a]
}

const fn position(point: Point3) -> Vec3 {
    Vec3::new(point.x(), point.y(), point.z())
}

const fn at(vector: Vec3) -> Point3 {
    Point3::new(vector.x(), vector.y(), vector.z())
}
