//! The stripe of a fillet between a plane and a cylinder that crosses it
//! obliquely, from one of the cylinder's tangent flanks to the other.
//!
//! An earlier fillet leaves a cylindrical band between two walls. A later
//! fillet round the face those walls end on runs along a wall's edge, over
//! the band's end and on along the next wall. Where the band's axis is square
//! to that face its end is a circle and the stripe over it a torus
//! ([`super::arc_torus`]). On a drafted body the axis leans, the end is an
//! ellipse, and two things change.
//!
//! The stripe has no closed surface form, but its sections do. The ball's
//! centre stays one radius off the plane and a fixed distance from the band's
//! axis, so it runs on the ellipse those two conditions cut, and at the
//! band's angle `φ` the section is the arc about that centre between its
//! feet on the plane and on the band. Both feet trace ellipses: the plane's
//! contact is the centre ellipse moved one radius along the plane's normal,
//! and the band's contact is a plane section of the band.
//!
//! And the ball does not change supports at the chain's vertex. It leaves a
//! wall for the band where its contact reaches the seam between them, the
//! ruling the two are tangent along, and a leaning ruling meets the wall's
//! contact line before or after the vertex's own station. So the stripe is
//! built over the band's whole angle between its two seams, and each
//! neighbouring wall stripe ends on this one's first or last section instead
//! of at the vertex.

use remus_math::surfaces::CylindricalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};

/// One section of the stripe: the ball's centre and its two contacts.
#[derive(Debug, Clone, Copy)]
pub(super) struct ObliqueSection {
    /// The ball's centre.
    pub(super) centre: Point3,
    /// The contact on the plane.
    pub(super) plane: Point3,
    /// The contact on the band.
    pub(super) band: Point3,
}

/// The stripe's sections at `stations` band angles spread evenly from the
/// seam with the start flank to the seam with the end flank, or `None` when
/// the configuration is not this one.
///
/// `plane` is the plane's outward unit normal and offset `n·p`. The band's
/// face normal is its radial when `band_outward_radial` (material inside the
/// cylinder). `side` is `-1.0` for a convex edge (the ball in the material)
/// and `1.0` for a concave one. `flank_start` and `flank_end` are the outward
/// unit normals of the planar walls tangent to the band at the stripe's two
/// ends.
///
/// Declined: a plane along the band's axis (its edge is a ruling, not an
/// arc), a flank that is not square to the axis, flanks less than
/// `tol.angular` or a half turn apart, and a ball too large to roll inside a
/// band it rounds from within.
#[allow(clippy::too_many_arguments)]
pub(super) fn sections(
    plane: (Vec3, f64),
    band: &CylindricalSurface,
    band_outward_radial: bool,
    side: f64,
    radius: f64,
    flank_start: Vec3,
    flank_end: Vec3,
    stations: usize,
    tol: Tolerance,
) -> Option<Vec<ObliqueSection>> {
    if stations < 2 {
        return None;
    }
    let (normal, d) = plane;
    let axis = band.axis();
    let along = normal.dot(axis);
    if along.abs() <= tol.angular {
        return None;
    }
    // The band's radial at a seam is its face normal there, the flank's own,
    // turned to the cylinder's side.
    let sign = if band_outward_radial { 1.0 } else { -1.0 };
    let radial_start = flank_start * sign;
    let radial_end = flank_end * sign;
    if radial_start.dot(axis).abs() > tol.angular || radial_end.dot(axis).abs() > tol.angular {
        return None;
    }
    // The centre is `side * radius` along the face normal from each contact.
    let ring = side * radius * sign + band.radius();
    if ring <= tol.linear {
        return None;
    }
    let x = radial_start.normalize().ok()?;
    let y = axis.cross(x);
    let sweep = radial_end.dot(y).atan2(radial_end.dot(x));
    if sweep.abs() <= tol.angular || sweep.abs() >= std::f64::consts::PI - tol.angular {
        return None;
    }
    let origin = band.origin();
    let offset = |p: Point3| Vec3::new(p.x(), p.y(), p.z());
    let mut out = Vec::with_capacity(stations);
    for i in 0..stations {
        #[allow(clippy::cast_precision_loss)]
        let phi = sweep * (i as f64 / (stations - 1) as f64);
        let (sin, cos) = phi.sin_cos();
        let radial = x * cos + y * sin;
        // Height along the axis putting the centre `side * radius` off the plane.
        let foot = origin + radial * ring;
        let height = (d + side * radius - normal.dot(offset(foot))) / along;
        let centre = foot + axis * height;
        out.push(ObliqueSection {
            centre,
            plane: centre - normal * (side * radius),
            band: origin + axis * height + radial * band.radius(),
        });
    }
    Some(out)
}

/// The cylinder the plane contacts lie on: the centres' own, moved to the
/// plane. Its section by the plane is the plane contact's ellipse.
pub(super) fn plane_contact_cylinder(
    band: &CylindricalSurface,
    band_outward_radial: bool,
    plane_normal: Vec3,
    side: f64,
    radius: f64,
) -> Option<CylindricalSurface> {
    let sign = if band_outward_radial { 1.0 } else { -1.0 };
    let ring = side * radius * sign + band.radius();
    CylindricalSurface::new(
        band.origin() - plane_normal * (side * radius),
        band.axis(),
        ring,
    )
    .ok()
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
    const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
    const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

    fn radial_distance(band: &CylindricalSurface, p: Point3) -> f64 {
        let offset = p - band.origin();
        (offset - band.axis() * offset.dot(band.axis())).length()
    }

    /// A band leaning 3 degrees off the plane's normal, rounded from inside.
    #[test]
    fn every_section_is_a_ball_tangent_to_the_plane_and_the_band() {
        let axis = (Z + X * 0.03 + Y * 0.04).normalize().unwrap();
        let band = CylindricalSurface::new(Point3::new(2.0, 3.0, 0.0), axis, 5.0).unwrap();
        // Flanks square to the axis, a quarter turn apart.
        let flank_start = X.cross(axis).cross(axis).normalize().unwrap() * -1.0;
        let flank_end = axis.cross(flank_start);
        let (r, d) = (2.0, 20.0);
        let sections = sections(
            (Z, d),
            &band,
            true,
            -1.0,
            r,
            flank_start,
            flank_end,
            17,
            Tolerance::new(),
        )
        .unwrap();
        assert_eq!(sections.len(), 17);
        for s in &sections {
            // On the plane and on the band, each one radius from the centre.
            assert!((s.plane.z() - d).abs() < 1e-12);
            assert!((radial_distance(&band, s.band) - 5.0).abs() < 1e-12);
            assert!(((s.plane - s.centre).length() - r).abs() < 1e-12);
            assert!(((s.band - s.centre).length() - r).abs() < 1e-12);
            // Tangent to both: the centre is along each normal from its contact.
            assert!(((s.plane - s.centre).cross(Z)).length() < 1e-12);
            assert!((radial_distance(&band, s.centre) - 3.0).abs() < 1e-12);
        }
        // The stripe starts and ends on the seams: the band's radial there is
        // the flank's normal.
        let radial_at = |p: Point3| {
            let offset = p - band.origin();
            (offset - band.axis() * offset.dot(band.axis()))
                .normalize()
                .unwrap()
        };
        assert!((radial_at(sections[0].band) - flank_start).length() < 1e-12);
        assert!((radial_at(sections[16].band) - flank_end).length() < 1e-12);

        // Both contacts trace plane sections of cylinders.
        let band_n = (sections[8].band - sections[0].band)
            .cross(sections[16].band - sections[0].band)
            .normalize()
            .unwrap();
        let first = sections[0].band;
        let band_d = band_n.dot(Vec3::new(first.x(), first.y(), first.z()));
        let carrier = plane_contact_cylinder(&band, true, Z, -1.0, r).unwrap();
        for s in &sections {
            let b = Vec3::new(s.band.x(), s.band.y(), s.band.z());
            assert!((band_n.dot(b) - band_d).abs() < 1e-11);
            assert!((radial_distance(&carrier, s.plane) - 3.0).abs() < 1e-12);
        }
    }

    #[test]
    fn a_concave_band_rounded_from_the_void_keeps_the_ball_inside_it() {
        // Material outside the cylinder (a pocket corner), concave floor edge:
        // the ball is in the void, inside the band.
        let axis = (Z + X * 0.05).normalize().unwrap();
        let band = CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), axis, 5.0).unwrap();
        let radial_start = Y;
        let radial_end = axis.cross(Y);
        // Face normals point into the void: against the radial.
        let sections = sections(
            (Z, 0.0),
            &band,
            false,
            1.0,
            1.0,
            -radial_start,
            -radial_end,
            9,
            Tolerance::new(),
        )
        .unwrap();
        for s in &sections {
            assert!(
                (s.centre.z() - 1.0).abs() < 1e-12,
                "one radius above the floor"
            );
            assert!((radial_distance(&band, s.centre) - 4.0).abs() < 1e-12);
            assert!(s.plane.z().abs() < 1e-12);
        }
    }

    #[test]
    fn a_plane_along_the_axis_and_a_ball_filling_the_band_are_declined() {
        let band = CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Z, 5.0).unwrap();
        let tol = Tolerance::new();
        assert!(sections((X, 1.0), &band, true, -1.0, 1.0, X, Y, 9, tol).is_none());
        assert!(sections((Z, 1.0), &band, true, -1.0, 5.0, X, Y, 9, tol).is_none());
        // Flanks a half turn apart leave no unique sweep.
        assert!(sections((Z, 1.0), &band, true, -1.0, 1.0, X, -X, 9, tol).is_none());
    }
}
