//! The exact torus of a fillet along a circular edge between a plane and a
//! coaxial cylinder.
//!
//! When the circle lies in the plane and the cylinder shares its axis, the
//! rolling ball's centre stays one radius off the plane and a fixed distance
//! from the axis, so the blend is a torus about that axis. This is the arc an
//! earlier fillet leaves where its band meets an end face: a later fillet
//! whose tangent chain runs across that arc rounds it with this torus, whose
//! ring radius is the band's radius plus the new one beside a concave band
//! and minus it beside a convex one.

use remus_math::curves::Circle3D;
use remus_math::surfaces::ToroidalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;

/// A torus stripe and its four exact boundary arcs, each as
/// `(circle, start, end)` along the stripe's corner points.
pub struct ArcTorus {
    pub torus: ToroidalSurface,
    pub seams: [(Circle3D, Point3, Point3); 4],
}

/// The torus a radius-`radius` fillet sweeps along `edge`, or `None` when the
/// edge is not a circle in a plane beside a coaxial cylinder, the torus would
/// not be a ring torus, or the stripe's contacts do not lie on it.
///
/// `corners` are the stripe's contact points `[c1_start, c2_start, c2_end,
/// c1_end]`, the `1` contacts on `surf1`.
pub fn arc_stripe_torus(
    edge: &EdgeCurve,
    surf1: &FaceSurface,
    surf2: &FaceSurface,
    corners: [Point3; 4],
    radius: f64,
    tol: Tolerance,
) -> Option<ArcTorus> {
    let EdgeCurve::Circle(spine) = edge else {
        return None;
    };
    let [c1s, c2s, c2e, c1e] = corners;
    // Walker contacts land on their surfaces to the solver's tolerance, not
    // the modelling one.
    let eps = (radius * 1e-6).max(tol.linear * 10.0);
    let (o, axis, a) = (spine.center(), spine.normal(), spine.radius());

    // Only a plane beside a cylinder qualifies, in either face order.
    let ((ps, pe), (cs, ce), cylinder) =
        if let (FaceSurface::Plane { normal, d }, FaceSurface::Cylinder(cylinder)) = (surf1, surf2)
        {
            on_plane(*normal, *d, o, axis, eps)?;
            ((c1s, c1e), (c2s, c2e), cylinder)
        } else if let (FaceSurface::Cylinder(cylinder), FaceSurface::Plane { normal, d }) =
            (surf1, surf2)
        {
            on_plane(*normal, *d, o, axis, eps)?;
            ((c2s, c2e), (c1s, c1e), cylinder)
        } else {
            return None;
        };
    if cylinder.axis().cross(axis).length() > tol.angular.max(1e-9)
        || (cylinder.radius() - a).abs() > eps
        || (cylinder.origin() - o).cross(axis).length() > eps
    {
        return None;
    }

    // Plane contacts run on the ring circle, cylinder contacts on the band
    // circle one radius off the plane.
    let ring = (ps - o).length();
    let height = (cs - o).dot(axis);
    let centre = o + axis * height;
    let radial = |p: Point3, from: Point3| {
        let offset = p - from;
        offset - axis * offset.dot(axis)
    };
    if [ps, pe].iter().any(|&p| (p - o).dot(axis).abs() > eps)
        || ((pe - o).length() - ring).abs() > eps
        || ((ce - o).dot(axis) - height).abs() > eps
        || [cs, ce]
            .iter()
            .any(|&p| (radial(p, o).length() - a).abs() > eps)
        || (height.abs() - radius).abs() > eps
        || ((ring - a).abs() - radius).abs() > eps
        || ring <= radius + eps
    {
        return None;
    }
    let torus = ToroidalSurface::with_axis(centre, ring, radius, axis).ok()?;

    // Each end section is the quarter circle about that station's ball
    // centre, in the meridian plane through it.
    let section = |plane_contact: Point3, band_contact: Point3, plane_first: bool| {
        let u = radial(plane_contact, o).normalize().ok()?;
        let ball = centre + u * ring;
        if ((plane_contact - ball).length() - radius).abs() > eps
            || ((band_contact - ball).length() - radius).abs() > eps
        {
            return None;
        }
        let circle = Circle3D::new(ball, axis.cross(u), radius).ok()?;
        Some(if plane_first {
            (circle, plane_contact, band_contact)
        } else {
            (circle, band_contact, plane_contact)
        })
    };
    let plane_first = matches!(surf1, FaceSurface::Plane { .. });
    let start = section(ps, cs, plane_first)?;
    let end = section(pe, ce, plane_first)?;
    Some(ArcTorus {
        torus,
        seams: [
            (Circle3D::new(o, axis, ring).ok()?, ps, pe),
            (Circle3D::new(centre, axis, a).ok()?, cs, ce),
            start,
            end,
        ],
    })
}

fn on_plane(normal: Vec3, d: f64, o: Point3, axis: Vec3, eps: f64) -> Option<()> {
    let normal = normal.normalize().ok()?;
    let through_centre = (normal.dot(Vec3::new(o.x(), o.y(), o.z())) - d).abs() <= eps;
    (through_centre && normal.cross(axis).length() <= 1e-9).then_some(())
}
