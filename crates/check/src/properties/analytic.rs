//! Closed-form geometric properties for primitive solids.
//!
//! Each function returns a [`GProps`] with volume, center of mass, and inertia
//! tensor for a canonical primitive placement (origin-based, axis-aligned).

use std::f64::consts::PI;

use remus_math::vec::Point3;

use super::accumulator::GProps;

/// Properties of a rectangular box with dimensions `(dx, dy, dz)`, corner at origin.
///
/// Volume = `dx * dy * dz`, center at `(dx/2, dy/2, dz/2)`.
#[must_use]
pub fn box_props(dx: f64, dy: f64, dz: f64) -> GProps {
    let v = dx * dy * dz;
    let center = Point3::new(dx / 2.0, dy / 2.0, dz / 2.0);
    // Inertia at CoM for uniform density=1 box:
    // Ixx = V/12 * (dy^2 + dz^2), etc.
    let ixx = v / 12.0 * (dy * dy + dz * dz);
    let iyy = v / 12.0 * (dx * dx + dz * dz);
    let izz = v / 12.0 * (dx * dx + dy * dy);
    GProps {
        mass: v,
        center,
        inertia: [ixx, iyy, izz, 0.0, 0.0, 0.0],
    }
}

/// Properties of a sphere with given `radius`, centered at origin.
///
/// Volume = `4/3 * pi * r^3`.
#[must_use]
pub fn sphere_props(radius: f64) -> GProps {
    let v = 4.0 / 3.0 * PI * radius.powi(3);
    let center = Point3::new(0.0, 0.0, 0.0);
    // I = 2/5 * m * r^2 (all axes, by symmetry)
    let i = 2.0 / 5.0 * v * radius * radius;
    GProps {
        mass: v,
        center,
        inertia: [i, i, i, 0.0, 0.0, 0.0],
    }
}

/// Properties of a cylinder with given `radius` and `height`, base at z=0.
///
/// Volume = `pi * r^2 * h`, center at `(0, 0, h/2)`.
#[must_use]
pub fn cylinder_props(radius: f64, height: f64) -> GProps {
    let v = PI * radius * radius * height;
    let center = Point3::new(0.0, 0.0, height / 2.0);
    // Ixx = Iyy = V/12 * (3*r^2 + h^2)
    let ixx = v / 12.0 * (3.0 * radius * radius + height * height);
    let iyy = ixx;
    // Izz = V/2 * r^2
    let izz = v / 2.0 * radius * radius;
    GProps {
        mass: v,
        center,
        inertia: [ixx, iyy, izz, 0.0, 0.0, 0.0],
    }
}

/// Properties of a cone frustum with `r_bottom`, `r_top`, `height`, base at z=0.
///
/// When `r_top = 0` this is a full cone.
/// Volume = `pi * h / 3 * (rb^2 + rb*rt + rt^2)`.
#[must_use]
#[allow(clippy::similar_names)]
pub fn cone_props(r_bottom: f64, r_top: f64, height: f64) -> GProps {
    let rb2 = r_bottom * r_bottom;
    let rt2 = r_top * r_top;
    let rbrt = r_bottom * r_top;
    let r_sum2 = rb2 + rbrt + rt2;

    // Degenerate: both radii zero — zero-volume line segment.
    if r_sum2 < 1e-30 {
        return GProps {
            mass: 0.0,
            center: Point3::new(0.0, 0.0, height / 2.0),
            inertia: [0.0; 6],
        };
    }

    let v = PI * height / 3.0 * r_sum2;
    // CoM for frustum:
    // z_com = h * (rb^2 + 2*rb*rt + 3*rt^2) / (4*(rb^2 + rb*rt + rt^2))
    let z_com = height * (rb2 + 2.0 * rbrt + 3.0 * rt2) / (4.0 * r_sum2);
    let center = Point3::new(0.0, 0.0, z_com);

    // Izz for frustum: 3V/10 * (rb^5 - rt^5) / (rb^3 - rt^3) when rb != rt
    // For full cone (rt=0): Izz = 3/10 * V * rb^2
    // General formula: Izz = 3*V/10 * (rb^4 + rb^3*rt + rb^2*rt^2 + rb*rt^3 + rt^4) / (rb^2 + rb*rt + rt^2)
    let r_sum4 = rb2 * rb2 + rb2 * rbrt + rb2 * rt2 + rbrt * rt2 + rt2 * rt2;
    let izz = 3.0 * v / 10.0 * r_sum4 / r_sum2;

    // Ixx = Iyy for frustum (about CoM):
    // Ixx_origin = 3V/80 * (4*(rb^4+rb^3*rt+rb^2*rt^2+rb*rt^3+rt^4)/(rb^2+rb*rt+rt^2) + h^2*(rb^2+3*rb*rt+6*rt^2)/(rb^2+rb*rt+rt^2))
    // Then shift to CoM via parallel axis
    let ixx_about_base = 3.0 * v / 20.0 * r_sum4 / r_sum2
        + v * height * height * (rb2 + 3.0 * rbrt + 6.0 * rt2) / (10.0 * r_sum2);
    // Shift from base-axis to CoM: Ixx_com = Ixx_base - V * z_com^2
    let ixx = ixx_about_base - v * z_com * z_com;
    let iyy = ixx;

    GProps {
        mass: v,
        center,
        inertia: [ixx, iyy, izz, 0.0, 0.0, 0.0],
    }
}

/// Properties of a torus with `major_r` (center to tube center) and `minor_r`
/// (tube radius), centered at origin in the XY plane.
///
/// Volume = `2 * pi^2 * R * r^2`.
#[must_use]
pub fn torus_props(major_r: f64, minor_r: f64) -> GProps {
    let v = 2.0 * PI * PI * major_r * minor_r * minor_r;
    let center = Point3::new(0.0, 0.0, 0.0);
    // Izz = V * (R^2 + 3/4 * r^2)
    let izz = v * (major_r * major_r + 0.75 * minor_r * minor_r);
    // Ixx = Iyy = V * (R^2/2 + 5/8 * r^2)
    let ixx = v * (major_r * major_r / 2.0 + 5.0 / 8.0 * minor_r * minor_r);
    let iyy = ixx;
    GProps {
        mass: v,
        center,
        inertia: [ixx, iyy, izz, 0.0, 0.0, 0.0],
    }
}

/// Surface area of a box with dimensions `(dx, dy, dz)`.
#[must_use]
pub fn box_area(dx: f64, dy: f64, dz: f64) -> f64 {
    2.0 * (dx * dy + dy * dz + dx * dz)
}

/// Surface area of a sphere with given `radius`.
#[must_use]
pub fn sphere_area(radius: f64) -> f64 {
    4.0 * PI * radius * radius
}

/// Surface area of a cylinder (including caps) with given `radius` and `height`.
#[must_use]
pub fn cylinder_area(radius: f64, height: f64) -> f64 {
    2.0 * PI * radius * height + 2.0 * PI * radius * radius
}

/// Surface area of a torus with `major_r` and `minor_r`.
#[must_use]
pub fn torus_area(major_r: f64, minor_r: f64) -> f64 {
    4.0 * PI * PI * major_r * minor_r
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::unwrap_used, clippy::unreadable_literal)]
    // `unreadable_literal`: expected values are pasted bit-exact `repr` outputs;
    // digit separators would obscure the exact-decimal correspondence.

    use super::*;

    /// Relative comparison with a tight tolerance; `expected` is always nonzero here.
    fn assert_close(actual: f64, expected: f64) {
        let tol = 1e-12 * expected.abs();
        assert!(
            (actual - expected).abs() <= tol,
            "expected {expected}, got {actual}"
        );
    }

    fn assert_zero(actual: f64) {
        assert!(actual.abs() <= 1e-300, "expected 0.0, got {actual}");
    }

    #[test]
    fn box_exact_props() {
        // Box 2 x 3 x 5, corner at origin. V = 30, CoM = (1, 1.5, 2.5).
        // Ixx = V/12*(dy^2+dz^2) = 30/12*(9+25) = 85
        // Iyy = V/12*(dx^2+dz^2) = 30/12*(4+25) = 72.5
        // Izz = V/12*(dx^2+dy^2) = 30/12*(4+9) = 32.5
        let p = box_props(2.0, 3.0, 5.0);
        assert_close(p.mass, 30.0);
        assert_close(p.center.x(), 1.0);
        assert_close(p.center.y(), 1.5);
        assert_close(p.center.z(), 2.5);
        assert_close(p.inertia[0], 85.0);
        assert_close(p.inertia[1], 72.5);
        assert_close(p.inertia[2], 32.5);
        assert_zero(p.inertia[3]);
        assert_zero(p.inertia[4]);
        assert_zero(p.inertia[5]);
    }

    #[test]
    fn sphere_exact_props() {
        // r = 3: V = 4/3*pi*27, I = 2/5*V*r^2 (all axes).
        // (r != 2 so `radius * radius` -> `+` stays visible: 9 != 6.)
        let p = sphere_props(3.0);
        assert_close(p.mass, 113.09733552923254);
        assert_zero(p.center.x());
        assert_zero(p.center.y());
        assert_zero(p.center.z());
        assert_close(p.inertia[0], 407.1504079052372);
        assert_close(p.inertia[1], 407.1504079052372);
        assert_close(p.inertia[2], 407.1504079052372);
        assert_zero(p.inertia[3]);
        assert_zero(p.inertia[4]);
        assert_zero(p.inertia[5]);
    }

    #[test]
    fn cylinder_exact_props() {
        // r = 2, h = 5: V = pi*4*5 = 20*pi, CoM z = 2.5.
        // Ixx = V/12*(3*r^2+h^2) = V/12*37; Izz = V/2*r^2 = 40*pi.
        let p = cylinder_props(2.0, 5.0);
        assert_close(p.mass, 62.83185307179586);
        assert_zero(p.center.x());
        assert_zero(p.center.y());
        assert_close(p.center.z(), 2.5);
        assert_close(p.inertia[0], 193.7315469713706);
        assert_close(p.inertia[1], 193.7315469713706);
        assert_close(p.inertia[2], 125.66370614359172);
        assert_zero(p.inertia[3]);
        assert_zero(p.inertia[4]);
        assert_zero(p.inertia[5]);
    }

    #[test]
    fn cone_frustum_exact_props() {
        // rb = 4, rt = 3, h = 5: r_sum2 = 16+12+9 = 37, V = pi*5/3*37.
        // z_com = h*(16+24+27)/(4*37) = 5*67/148.
        // (Radii avoid 0/1 so no `*`->`+`/`/` mutant hides: rt+rt != rt^2.)
        let p = cone_props(4.0, 3.0, 5.0);
        assert_close(p.mass, 193.7315469713706);
        assert_zero(p.center.x());
        assert_zero(p.center.y());
        assert_close(p.center.z(), 2.2635135135135136);
        assert_close(p.inertia[0], 1008.3504136597253);
        assert_close(p.inertia[1], 1008.3504136597253);
        assert_close(p.inertia[2], 1226.7919312268143);
        assert_zero(p.inertia[3]);
        assert_zero(p.inertia[4]);
        assert_zero(p.inertia[5]);
    }

    #[test]
    fn cone_full_cone_exact_props() {
        // Full cone rb = 2, rt = 0, h = 3: V = 4*pi, z_com = h/4,
        // Izz = 3/10*V*rb^2 = 4.8*pi.
        let p = cone_props(2.0, 0.0, 3.0);
        assert_close(p.mass, 12.566370614359172);
        assert_zero(p.center.x());
        assert_zero(p.center.y());
        assert_close(p.center.z(), 0.75);
        assert_close(p.inertia[0], 11.780972450961723);
        assert_close(p.inertia[1], 11.780972450961723);
        assert_close(p.inertia[2], 15.079644737231007);
        assert_zero(p.inertia[3]);
        assert_zero(p.inertia[4]);
        assert_zero(p.inertia[5]);
    }

    #[test]
    fn cone_degenerate_zero_radii() {
        // Both radii zero: zero-volume segment, CoM at h/2, zero inertia.
        let p = cone_props(0.0, 0.0, 5.0);
        assert_zero(p.mass);
        assert_zero(p.center.x());
        assert_zero(p.center.y());
        assert_close(p.center.z(), 2.5);
        for &c in &p.inertia {
            assert_zero(c);
        }
    }

    #[test]
    fn cone_degenerate_threshold_boundary_takes_normal_path() {
        // rb = 1e-15, rt = 0: r_sum2 == 1e-30 exactly, which is NOT < 1e-30,
        // so the normal (non-degenerate) path applies: V = pi*h/3*1e-30,
        // z_com = h/4 for a full cone.
        let p = cone_props(1e-15, 0.0, 2.0);
        assert_close(p.mass, 2.0943951023931956e-30);
        assert_close(p.center.z(), 0.5);
    }

    #[test]
    fn torus_exact_props() {
        // R = 4, r = 2: V = 2*pi^2*4*4, Izz = V*(16+3), Ixx = V*(8+2.5).
        // (R != 3 so `major_r * major_r / 2` -> `+` stays visible:
        // R+R/2 == R^2/2 only at R = 3; r != 1 keeps `r * r` -> `/` visible.)
        let p = torus_props(4.0, 2.0);
        assert_close(p.mass, 315.82734083485946);
        assert_zero(p.center.x());
        assert_zero(p.center.y());
        assert_zero(p.center.z());
        assert_close(p.inertia[0], 3316.187078766024);
        assert_close(p.inertia[1], 3316.187078766024);
        assert_close(p.inertia[2], 6000.71947586233);
        assert_zero(p.inertia[3]);
        assert_zero(p.inertia[4]);
        assert_zero(p.inertia[5]);
    }

    #[test]
    fn box_area_exact() {
        // 2*(6 + 15 + 10) = 62.
        assert_close(box_area(2.0, 3.0, 5.0), 62.0);
    }

    #[test]
    fn sphere_area_exact() {
        // 4*pi*9 = 36*pi.
        assert_close(sphere_area(3.0), 113.09733552923255);
    }

    #[test]
    fn cylinder_area_exact() {
        // 2*pi*2*5 + 2*pi*4 = 28*pi.
        assert_close(cylinder_area(2.0, 5.0), 87.96459430051421);
    }

    #[test]
    fn torus_area_exact() {
        // 4*pi^2*3*2 = 24*pi^2.
        assert_close(torus_area(3.0, 2.0), 236.8705056261446);
    }
}
