//! Pcurve re-expression for transformed faces (B89).
//!
//! A transform moves a face's 3D boundary, but its pcurves are coordinates
//! in the face's UV chart. Every surface arm of the face transform therefore
//! reports how its chart moved as a [`ChartMap`], and
//! [`reexpress_face_pcurves`] carries each boundary use into the new chart.
//!
//! Every [`UvAffine`] below is an exact identity between the two charts:
//! `new_surface(map(uv)) = matrix · old_surface(uv)`. A pcurve mapped by it
//! keeps its parameterization up to an affine reparameterization (a scale
//! for lines, a sign and phase for conics), so a use that was same-parameter
//! with its edge stays same-parameter with the transformed edge.
//!
//! Periodic lift counts are then re-derived from the moved geometry. A
//! vertex on a seam projects to principal `0` or `2π − ε` depending on the
//! rounding of its transformed coordinates, and the STEP writer certifies
//! each count against exactly that projection.

use remus_math::curves2d::{Circle2D, Curve2D, Ellipse2D, Line2D, NurbsCurve2D};
use remus_math::frame::Frame3;
use remus_math::mat::Mat4;
use remus_math::vec::{Point2, Point3, Vec2, Vec3};
use remus_topology::Topology;
use remus_topology::coedge::PeriodicWinding;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::pcurve::PCurve;

/// Relative roundoff under which a mapped circle's conjugate radii still
/// describe a circle. The radii come out of a few dot products of unit
/// vectors, so a similarity leaves them equal to a handful of ulps; a real
/// anisotropy sits many orders above.
const CIRCLE_ROUNDOFF_REL: f64 = 64.0 * f64::EPSILON;

/// Radians below which a frame rotation is the roundoff of normalizing a
/// moved unit axis, not a turn.
const ANGLE_ROUNDOFF: f64 = 16.0 * f64::EPSILON;

/// How one face's UV chart changed under a transform.
#[derive(Debug, Clone, Copy)]
pub enum ChartMap {
    /// The carrier kept its chart; stored pcurves stay valid as they are.
    Unchanged,
    /// The new chart is an affine image of the old one.
    Affine(UvAffine),
    /// The carrier was replaced by one with an unrelated chart (an analytic
    /// face converted to NURBS, or a NURBS face re-oriented): its pcurves
    /// are dropped and the 3D edges stay authoritative.
    Replaced,
}

/// `new (u, v) = origin + u·u_image + v·v_image` for an old `(u, v)`.
#[derive(Debug, Clone, Copy)]
pub struct UvAffine {
    origin: Point2,
    u_image: Vec2,
    v_image: Vec2,
}

impl UvAffine {
    /// `(u, v) ↦ (u_scale·u, v_scale·v)`.
    #[cfg(test)]
    pub const fn diagonal(u_scale: f64, v_scale: f64) -> Self {
        Self {
            origin: Point2::new(0.0, 0.0),
            u_image: Vec2::new(u_scale, 0.0),
            v_image: Vec2::new(0.0, v_scale),
        }
    }

    /// The map for a revolved carrier (cylinder, cone, sphere, torus) whose
    /// angular frame `(x, y)` went from `old` to the rebuilt `new`:
    /// `(u, v) ↦ (±u + φ, v_scale·v)`.
    ///
    /// `φ` is the angle of the moved `old.x` in the new frame, and the sign
    /// is negative when the map reflects the frame (a carrier rebuilt
    /// right-handed then sweeps `u` the other way). `v_scale` is the
    /// similarity's scale for an axial or slant `v`, `1` for an angle.
    pub fn revolved(matrix: &Mat4, old: (Vec3, Vec3), new: (Vec3, Vec3), v_scale: f64) -> Self {
        let in_new = |d: Vec3| {
            let image = linear(matrix, d);
            Vec2::new(image.dot(new.0), image.dot(new.1))
        };
        let (x, y) = (in_new(old.0), in_new(old.1));
        let sign = if x.x().mul_add(y.y(), -(x.y() * y.x())) < 0.0 {
            -1.0
        } else {
            1.0
        };
        let phi = x.y().atan2(x.x());
        // A frame carried exactly (sphere, torus) lands on `φ = 0` up to the
        // roundoff of normalizing the moved axes.
        let phi = if phi.abs() <= ANGLE_ROUNDOFF {
            0.0
        } else {
            phi
        };
        Self {
            origin: Point2::new(phi, 0.0),
            u_image: Vec2::new(sign, 0.0),
            v_image: Vec2::new(0.0, v_scale),
        }
    }

    /// The map between the canonical charts of a plane before and after
    /// `matrix` carried it from `old` to `new` (`(normal, d)` with
    /// `normal · p = d`).
    ///
    /// Plane pcurves live in the canonical chart that `FaceSurface::Plane`
    /// cannot store a frame for: origin at the foot of the world origin,
    /// x axis `normal × seed` (the same chart the `SameParameter` proofs and
    /// the STEP reader and writer use). It is rebuilt from the new normal
    /// alone, so the map is a general 2D affine map, not an identity.
    pub fn plane(
        old: (Vec3, f64),
        new: (Vec3, f64),
        matrix: &Mat4,
    ) -> Result<Self, crate::OperationsError> {
        let before = plane_chart(old.0, old.1)?;
        let after = plane_chart(new.0, new.1)?;
        let offset = matrix.mul_point(before.origin) - after.origin;
        let u_image = linear(matrix, before.x);
        let v_image = linear(matrix, before.y);
        let in_chart = |v: Vec3| Vec2::new(v.dot(after.x), v.dot(after.y));
        Ok(Self {
            origin: Point2::new(offset.dot(after.x), offset.dot(after.y)),
            u_image: in_chart(u_image),
            v_image: in_chart(v_image),
        })
    }

    #[allow(clippy::float_cmp)] // Only a bit-exact identity may skip the rewrite.
    fn is_identity(self) -> bool {
        self.origin.x() == 0.0
            && self.origin.y() == 0.0
            && self.u_image.x() == 1.0
            && self.u_image.y() == 0.0
            && self.v_image.x() == 0.0
            && self.v_image.y() == 1.0
    }

    fn vector(self, v: Vec2) -> Vec2 {
        self.u_image * v.x() + self.v_image * v.y()
    }

    fn point(self, p: Point2) -> Point2 {
        self.origin + self.vector(Vec2::new(p.x(), p.y()))
    }

    /// The pcurve's image, reparameterized so that the image's parameter `t'`
    /// at each point is an affine function of the source parameter `t`.
    pub fn apply(self, pcurve: &PCurve) -> Result<PCurve, crate::OperationsError> {
        let (curve, scale, shift) = match pcurve.curve() {
            Curve2D::Line(line) => {
                let direction = self.vector(line.direction());
                // `Line2D::new` re-normalizes, so the parameter stretches by
                // the direction's growth.
                let scale = direction.length() / line.direction().length();
                let image = Line2D::new(self.point(line.origin()), direction)?;
                (Curve2D::Line(image), scale, 0.0)
            }
            Curve2D::Nurbs(nurbs) => {
                // Affine maps commute with rational evaluation: mapping the
                // control points maps every curve point at the same parameter.
                let image = NurbsCurve2D::new(
                    nurbs.degree(),
                    nurbs.knots().to_vec(),
                    nurbs
                        .control_points()
                        .iter()
                        .map(|p| self.point(*p))
                        .collect(),
                    nurbs.weights().to_vec(),
                )?;
                (Curve2D::Nurbs(image), 1.0, 0.0)
            }
            Curve2D::Circle(circle) => conic_image(
                self.point(circle.center()),
                self.vector(Vec2::new(circle.radius(), 0.0)),
                self.vector(Vec2::new(0.0, circle.radius())),
                true,
            )?,
            Curve2D::Ellipse(ellipse) => {
                let (sin_r, cos_r) = ellipse.rotation().sin_cos();
                conic_image(
                    self.point(ellipse.center()),
                    self.vector(Vec2::new(cos_r, sin_r) * ellipse.semi_major()),
                    self.vector(Vec2::new(-sin_r, cos_r) * ellipse.semi_minor()),
                    false,
                )?
            }
        };
        Ok(PCurve::new(
            curve,
            pcurve.t_start().mul_add(scale, shift),
            pcurve.t_end().mul_add(scale, shift),
        ))
    }
}

/// The canonical chart of the plane `normal · p = d`.
fn plane_chart(normal: Vec3, d: f64) -> Result<Frame3, crate::OperationsError> {
    let foot = normal * (d / normal.dot(normal));
    Ok(Frame3::from_normal(
        Point3::new(foot.x(), foot.y(), foot.z()),
        normal,
    )?)
}

/// The linear part of `matrix` applied to a direction (no translation).
pub fn linear(matrix: &Mat4, d: Vec3) -> Vec3 {
    let m = &matrix.0;
    Vec3::new(
        m[0][0].mul_add(d.x(), m[0][1].mul_add(d.y(), m[0][2] * d.z())),
        m[1][0].mul_add(d.x(), m[1][1].mul_add(d.y(), m[1][2] * d.z())),
        m[2][0].mul_add(d.x(), m[2][1].mul_add(d.y(), m[2][2] * d.z())),
    )
}

/// Re-expresses `center + p·cos t + q·sin t` as a native circle or ellipse.
///
/// Returns the curve with `(sign, shift)` such that the image's parameter is
/// `sign·t + shift`. `p` and `q` are conjugate semi-diameters: an anisotropic
/// map skews them, and a reflection turns them clockwise.
fn conic_image(
    center: Point2,
    p: Vec2,
    q: Vec2,
    prefer_circle: bool,
) -> Result<(Curve2D, f64, f64), crate::OperationsError> {
    // c + p cos t + q sin t = c + p cos(−t) + (−q) sin(−t): a clockwise pair
    // becomes counter-clockwise under t ↦ −t.
    let clockwise = p.x().mul_add(q.y(), -(p.y() * q.x())) < 0.0;
    let (q, sign) = if clockwise { (-q, -1.0) } else { (q, 1.0) };
    let (lp, lq) = (p.length(), q.length());
    let scale = lp.max(lq);
    if prefer_circle
        && (lp - lq).abs() <= CIRCLE_ROUNDOFF_REL * scale
        && p.dot(q).abs() <= CIRCLE_ROUNDOFF_REL * scale * scale
    {
        // Counter-clockwise equal orthogonal radii: c + R(cos(s + ρ), sin(s + ρ)).
        let circle = Circle2D::new(center, 0.5 * (lp + lq))?;
        return Ok((Curve2D::Circle(circle), sign, p.y().atan2(p.x())));
    }
    // Principal axes: rotating the conjugate pair by φ reparameterizes
    // s ↦ s − φ, and this φ makes the rotated pair orthogonal with the major
    // axis first.
    let phi = 0.5 * (2.0 * p.dot(q)).atan2(lp.mul_add(lp, -(lq * lq)));
    let (sin_phi, cos_phi) = phi.sin_cos();
    let major = p * cos_phi + q * sin_phi;
    let minor = q * cos_phi - p * sin_phi;
    let (a, b) = (major.length(), minor.length());
    let (ellipse, shift) = if a >= b {
        (
            Ellipse2D::new(center, a, b, major.y().atan2(major.x()))?,
            -phi,
        )
    } else {
        // Roundoff only: major·cos s + minor·sin s
        // = minor·cos(s − π/2) − major·sin(s − π/2).
        (
            Ellipse2D::new(center, b, a, minor.y().atan2(minor.x()))?,
            -phi - std::f64::consts::FRAC_PI_2,
        )
    };
    Ok((Curve2D::Ellipse(ellipse), sign, shift))
}

/// Carries every boundary use of `face` into the chart `map` describes.
///
/// Call once the face's new surface and transformed vertices are in place:
/// re-derived lift counts project the transformed endpoints onto it.
///
/// # Errors
///
/// Returns an error for stale topology or an unrepresentable image (a
/// collapsed pcurve direction).
pub fn reexpress_face_pcurves(
    topo: &mut Topology,
    face: FaceId,
    map: &ChartMap,
) -> Result<(), crate::OperationsError> {
    let affine = match map {
        ChartMap::Unchanged => None,
        ChartMap::Affine(affine) => (!affine.is_identity()).then_some(*affine),
        ChartMap::Replaced => {
            for id in face_uses(topo, face)? {
                let coedge = topo.coedge(id)?;
                if coedge.pcurve().is_none() {
                    continue;
                }
                // A winding without its pcurve fails boundary authority.
                let had_winding = coedge.periodic_winding() != PeriodicWinding::ZERO;
                topo.remove_coedge_pcurve(id)?;
                if had_winding {
                    topo.set_coedge_periodic_winding(id, PeriodicWinding::ZERO)?;
                }
            }
            return Ok(());
        }
    };
    let periodic = matches!(
        topo.face(face)?.surface(),
        FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_)
    );
    if affine.is_none() && !periodic {
        return Ok(());
    }
    for id in face_uses(topo, face)? {
        let coedge = topo.coedge(id)?;
        let Some(pcurve) = coedge.pcurve() else {
            continue;
        };
        let image = affine.map(|affine| affine.apply(pcurve)).transpose()?;
        let winding = if periodic {
            let edge = topo.edge(coedge.edge())?;
            let (start, end) = if coedge.is_forward() {
                (edge.start(), edge.end())
            } else {
                (edge.end(), edge.start())
            };
            let (start, end) = (topo.vertex(start)?.point(), topo.vertex(end)?.point());
            derived_winding(
                topo.face(face)?.surface(),
                image.as_ref().unwrap_or(pcurve),
                start,
                end,
            )
            .filter(|derived| *derived != coedge.periodic_winding())
        } else {
            None
        };
        if let Some(image) = image {
            topo.set_coedge_pcurve(id, image)?;
        }
        if let Some(winding) = winding {
            topo.set_coedge_periodic_winding(id, winding)?;
        }
    }
    Ok(())
}

fn face_uses(
    topo: &Topology,
    face: FaceId,
) -> Result<Vec<remus_topology::coedge::CoedgeId>, crate::OperationsError> {
    let mut uses = Vec::new();
    for &loop_id in topo.loops_of_face(face).unwrap_or(&[]) {
        uses.extend(topo.face_loop(loop_id)?.coedges().iter().copied());
    }
    Ok(uses)
}

/// The periodic lift count of a use: per periodic direction, the whole
/// number of periods between each pcurve endpoint and the principal
/// projection of its 3D vertex, kept when both endpoints agree and zero
/// otherwise. The rule the STEP reader stamps and the writer certifies.
///
/// `None` when an endpoint is not an integral lift (a use that already
/// missed its vertex): the stored count is then left alone.
fn derived_winding(
    surface: &FaceSurface,
    pcurve: &PCurve,
    start: Point3,
    end: Point3,
) -> Option<PeriodicWinding> {
    use std::f64::consts::TAU;
    /// The writer's certification bound on a lift's distance from a whole period.
    const BRANCH_TOLERANCE: f64 = 1e-10;
    let v_periodic = matches!(surface, FaceSurface::Torus(_));
    let principal_start = surface.project_point(start)?;
    let principal_end = surface.project_point(end)?;
    let uv_start = pcurve.evaluate(pcurve.t_start());
    let uv_end = pcurve.evaluate(pcurve.t_end());
    let lift = |value: f64, principal: f64| -> Option<i32> {
        let turns = (value - principal) / TAU;
        let nearest = turns.round();
        let certified = turns.is_finite()
            && (turns - nearest).abs() * TAU <= BRANCH_TOLERANCE
            && nearest.abs() <= f64::from(i32::MAX);
        #[allow(clippy::cast_possible_truncation)] // Range-checked above.
        certified.then_some(nearest as i32)
    };
    let axis = |a: f64, b: f64, pa: f64, pb: f64| -> Option<i32> {
        let (first, second) = (lift(a, pa)?, lift(b, pb)?);
        Some(if first == second { first } else { 0 })
    };
    let u = axis(uv_start.x(), uv_end.x(), principal_start.0, principal_end.0)?;
    let v = if v_periodic {
        axis(uv_start.y(), uv_end.y(), principal_start.1, principal_end.1)?
    } else {
        0
    };
    Some(PeriodicWinding::new(u, v))
}

#[cfg(test)]
mod tests;
