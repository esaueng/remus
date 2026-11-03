//! Independently decoded analytic carriers and their saved parameter frames.

use remus_math::curves::Circle3D;
use remus_math::mat::Mat4;
use remus_math::surfaces::CylindricalSurface;
use remus_math::vec::{Point2, Point3, Vec2};

use super::locations::{point, vector};
use super::{IoError, Line, Line2, Plane, Tokens, malformed, unsupported};

#[derive(Clone)]
pub(super) struct Circle2 {
    pub center: Point2,
    pub x: Vec2,
    pub y: Vec2,
    pub radius: f64,
}

#[derive(Clone)]
pub(super) enum Curve2 {
    Line(Line2),
    Circle(Circle2),
}

#[derive(Clone)]
pub(super) enum Curve3 {
    Line(Line),
    Circle(Circle3D),
}

#[derive(Clone)]
pub(super) struct Cylinder {
    pub frame: Plane,
    pub radius: f64,
}

#[derive(Clone)]
pub(super) enum Surface {
    Plane(Plane),
    Cylinder(Cylinder),
}

impl Tokens<'_> {
    fn frame(&mut self, left: bool) -> Result<Plane, IoError> {
        let frame = Plane {
            origin: self.point()?,
            normal: self.direction()?,
            x: self.direction()?,
            y: self.direction()?,
        };
        let handed = frame.x.cross(frame.y).dot(frame.normal);
        if (handed.abs() - 1.0).abs() > 1e-12
            || frame.x.dot(frame.y).abs() > 1e-12
            || frame.x.dot(frame.normal).abs() > 1e-12
            || frame.y.dot(frame.normal).abs() > 1e-12
        {
            return Err(malformed("invalid analytic frame"));
        }
        if !left && handed < 0.0 {
            return Err(unsupported("left-handed plane frame"));
        }
        Ok(frame)
    }

    fn radius(&mut self) -> Result<f64, IoError> {
        let radius = self.real()?;
        if radius < 0.0 {
            return Err(malformed("negative circular radius"));
        }
        if radius == 0.0 {
            return Err(unsupported("degenerate circular carrier"));
        }
        Ok(radius)
    }

    pub(super) fn curve2(&mut self) -> Result<Curve2, IoError> {
        match self.next()? {
            "1" => {
                let origin = Point2::new(self.real()?, self.real()?);
                let (x, y) = (self.real()?, self.real()?);
                if (x * x + y * y - 1.0).abs() > 1e-12 {
                    return Err(malformed("non-unit pcurve direction"));
                }
                Ok(Curve2::Line(Line2 { origin, x, y }))
            }
            "2" => {
                let center = Point2::new(self.real()?, self.real()?);
                let x = Vec2::new(self.real()?, self.real()?);
                let y = Vec2::new(self.real()?, self.real()?);
                if (x.length_squared() - 1.).abs() > 1e-12
                    || (y.length_squared() - 1.).abs() > 1e-12
                    || x.dot(y).abs() > 1e-12
                {
                    return Err(malformed("invalid circular pcurve frame"));
                }
                Ok(Curve2::Circle(Circle2 {
                    center,
                    x,
                    y,
                    radius: self.radius()?,
                }))
            }
            _ => Err(unsupported("pcurve family")),
        }
    }

    pub(super) fn curve3(&mut self) -> Result<Curve3, IoError> {
        match self.next()? {
            "1" => Ok(Curve3::Line(Line {
                origin: self.point()?,
                direction: self.direction()?,
            })),
            "2" => {
                let frame = self.frame(true)?;
                let radius = self.radius()?;
                Ok(Curve3::Circle(
                    Circle3D::with_axes(
                        frame.origin,
                        frame.x.cross(frame.y),
                        radius,
                        frame.x,
                        frame.y,
                    )
                    .map_err(|_| malformed("invalid circle"))?,
                ))
            }
            _ => Err(unsupported("3D curve family")),
        }
    }

    pub(super) fn surface(&mut self) -> Result<Surface, IoError> {
        match self.next()? {
            "1" => Ok(Surface::Plane(self.frame(false)?)),
            "2" => Ok(Surface::Cylinder(Cylinder {
                frame: self.frame(true)?,
                radius: self.radius()?,
            })),
            _ => Err(unsupported("surface family")),
        }
    }
}

impl Plane {
    fn moved(&self, matrix: Mat4) -> Result<Self, IoError> {
        let frame = Self {
            origin: point(matrix, self.origin),
            normal: vector(matrix, self.normal),
            x: vector(matrix, self.x),
            y: vector(matrix, self.y),
        };
        if frame
            .origin
            .0
            .iter()
            .chain(frame.normal.0.iter())
            .chain(frame.x.0.iter())
            .chain(frame.y.0.iter())
            .any(|value| !value.is_finite())
        {
            return Err(malformed("non-finite located surface"));
        }
        Ok(frame)
    }
}

impl Curve3 {
    pub(super) fn moved(&self, matrix: Mat4) -> Result<Self, IoError> {
        let result = match self {
            Self::Line(line) => Self::Line(Line {
                origin: point(matrix, line.origin),
                direction: vector(matrix, line.direction),
            }),
            Self::Circle(circle) => Self::Circle(
                Circle3D::with_axes(
                    point(matrix, circle.center()),
                    vector(matrix, circle.normal()),
                    circle.radius(),
                    vector(matrix, circle.u_axis()),
                    vector(matrix, circle.v_axis()),
                )
                .map_err(|_| malformed("invalid located circle"))?,
            ),
        };
        for parameter in [0., 1.] {
            if result
                .evaluate(parameter)
                .0
                .iter()
                .any(|value| !value.is_finite())
            {
                return Err(malformed("non-finite located curve"));
            }
        }
        Ok(result)
    }

    pub(super) fn evaluate(&self, parameter: f64) -> Point3 {
        match self {
            Self::Line(line) => line.origin + line.direction * parameter,
            Self::Circle(circle) => circle.evaluate(parameter),
        }
    }
}

impl Surface {
    pub(super) fn moved(&self, matrix: Mat4) -> Result<Self, IoError> {
        Ok(match self {
            Self::Plane(plane) => Self::Plane(plane.moved(matrix)?),
            Self::Cylinder(cylinder) => Self::Cylinder(Cylinder {
                frame: cylinder.frame.moved(matrix)?,
                radius: cylinder.radius,
            }),
        })
    }
}

impl Cylinder {
    pub(super) fn flip(&self) -> bool {
        self.frame.x.cross(self.frame.y).dot(self.frame.normal) < 0.0
    }

    pub(super) fn native(&self) -> Result<CylindricalSurface, IoError> {
        CylindricalSurface::with_ref_dir(
            self.frame.origin,
            self.frame.x.cross(self.frame.y),
            self.radius,
            self.frame.x,
        )
        .map_err(|_| malformed("invalid cylindrical carrier"))
    }
}

pub(super) fn full_turn(first: f64, last: f64) -> bool {
    ((last - first) - std::f64::consts::TAU).abs() <= 32. * f64::EPSILON * std::f64::consts::TAU
}
