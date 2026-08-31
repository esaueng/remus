//! Surface curvature primitives with a single kernel-wide sign convention.
//!
//! # Sign convention
//!
//! Every curvature in this module (and in every curvature query built on top
//! of it) is reported relative to a chosen unit surface normal `N` — for all
//! surface types in this kernel the natural `normal(u, v)` output, which for
//! the analytic types equals the normalized cross product of the parametric
//! partial derivatives and points outward from the solid the surface bounds.
//!
//! The convention is **positive for convex-outward**: `k > 0` when the center
//! of curvature lies on the side *opposite* `N` (the surface bends away from
//! the normal — a ball's exterior), `k < 0` when it lies on the `+N` side
//! (the surface bends toward the normal — a bowl's interior, or the inner
//! equator of a torus). Formally, the principal curvatures are the
//! eigenvalues of the shape operator `S = I⁻¹·II` with the second
//! fundamental form taken as `II_ij = −N · X_ij`, i.e. `L = −N·X_uu`,
//! `M = −N·X_uv`, `N₂ = −N·X_vv`. Flipping the reference normal flips the
//! sign of both principal curvatures (and therefore of the mean), while the
//! Gaussian curvature is orientation-independent.
//!
//! Worked values under this convention: unit sphere `k1 = k2 = +1`;
//! cylinder `k = (1/r, 0)`; cone `k = (tan α/s, 0)` at slant distance `s`;
//! torus `k = (cos v/(R + r·cos v), 1/r)`; plane `(0, 0)`.

use crate::MathError;
use crate::vec::Vec3;

/// Relative threshold below which the two principal curvatures of a NURBS
/// surface are treated as equal (an umbilic point, where principal
/// directions are undefined). The k1 − k2 half-split is extracted from a
/// discriminant whose double-precision noise floor sits near √ε relative,
/// so anything below ~1e-6 of the curvature scale is not distinguishable
/// from a true umbilic and reporting two directions there would fabricate
/// information.
const UMBILIC_REL_TOL: f64 = 1e-6;

/// The two principal curvatures at a surface point, sorted `k1 >= k2`.
///
/// See the [module documentation](self) for the sign convention.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PrincipalCurvatures {
    /// Largest principal curvature.
    pub k1: f64,
    /// Smallest principal curvature.
    pub k2: f64,
}

impl PrincipalCurvatures {
    /// Gaussian curvature `K = k1 · k2`. Orientation-independent.
    #[must_use]
    pub fn gaussian(self) -> f64 {
        self.k1 * self.k2
    }

    /// Mean curvature `H = (k1 + k2) / 2`. Flips sign with the reference
    /// normal.
    #[must_use]
    pub fn mean(self) -> f64 {
        (self.k1 + self.k2) * 0.5
    }

    /// True at an umbilic point, where `k1 ≈ k2` and every tangent direction
    /// is principal.
    #[must_use]
    pub fn is_umbilic(self) -> bool {
        (self.k1 - self.k2).abs() <= UMBILIC_REL_TOL * (self.k1.abs() + self.k2.abs()).max(1.0)
    }
}

/// Principal curvatures and principal directions at a surface point.
///
/// `directions` is `None` at umbilic points (sphere, plane, and near-umbilic
/// NURBS regions), where every tangent direction is principal and reporting
/// two of them would fabricate information. Otherwise the pair holds the unit
/// tangent direction of `k1` first and `k2` second; the two directions are
/// orthogonal for a non-umbilic point of a `C²` surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceCurvature {
    /// Largest principal curvature (`k1 >= k2`).
    pub k1: f64,
    /// Smallest principal curvature.
    pub k2: f64,
    /// Unit principal directions `(d1, d2)` matching `(k1, k2)`, or `None` at
    /// an umbilic point.
    pub directions: Option<(Vec3, Vec3)>,
}

impl SurfaceCurvature {
    /// Gaussian curvature `K = k1 · k2`. Orientation-independent.
    #[must_use]
    pub fn gaussian(self) -> f64 {
        self.k1 * self.k2
    }

    /// Mean curvature `H = (k1 + k2) / 2`. Flips sign with the reference
    /// normal.
    #[must_use]
    pub fn mean(self) -> f64 {
        (self.k1 + self.k2) * 0.5
    }
}

/// Principal curvatures of a spherical surface of `radius`.
///
/// `k1 = k2 = 1/radius` everywhere: a sphere is umbilic, convex-outward
/// positive under the module sign convention with the outward normal.
#[must_use]
pub fn sphere_principal_curvatures(radius: f64) -> PrincipalCurvatures {
    PrincipalCurvatures {
        k1: 1.0 / radius,
        k2: 1.0 / radius,
    }
}

/// Principal curvatures of a cylindrical surface of `radius`.
///
/// With `P(u, v) = origin + r·(cos u, sin u, 0) + v·axis`, the axial
/// (`v`) direction is straight (`k = 0`) and the circumferential (`u`)
/// direction bends away from the outward normal with `k = 1/r`.
#[must_use]
pub fn cylinder_principal_curvatures(radius: f64) -> PrincipalCurvatures {
    PrincipalCurvatures {
        k1: 1.0 / radius,
        k2: 0.0,
    }
}

/// Principal curvatures of a conical surface at slant distance `v` from the apex.
///
/// `half_angle` is the angle between the generator and the radial plane
/// (this kernel's `ConicalSurface` convention, `0 < α < π/2`). Derivation
/// for `X(u, v) = apex + v·(cos α·radial(u) + sin α·axis)`, with
/// the kernel's outward normal `N = radial(u)·sin α − axis·cos α`:
///
/// - `X_u = v·cos α·radial'(u)`, `X_v = cos α·radial(u) + sin α·axis`, so
///   `E = v²·cos²α`, `F = 0`, `G = 1`;
/// - `X_uu = −v·cos α·radial(u)`, `X_uv = cos α·radial'(u)`, `X_vv = 0`, so
///   `L = −N·X_uu = v·cos α·sin α`, `M = 0`, `N₂ = 0`;
/// - the shape operator is diagonal with `k_u = L/E = tan α/v` and
///   `k_v = 0`.
///
/// The ruling direction is straight, and the circumferential direction has
/// the curvature of the parallel circle of radius `ρ = v·cos α` — the
/// distance from the axis — divided by `sin α` projected along the normal:
/// `k = sin α/ρ = tan α/v`. Note this is `tan α/s`, **not** `cos α/s`: the
/// circle's curvature vector (`1/ρ`, toward the axis) has a `sin α` component
/// along the outward normal, not a `cos α` one.
///
/// # Errors
///
/// Returns [`MathError::ParameterOutOfRange`] for `v <= 0`: the apex (`v = 0`)
/// is a singular point where the first fundamental form degenerates
/// (`E = v²·cos²α → 0`) and curvature is undefined, and `v < 0` lies outside
/// the surface's domain.
pub fn cone_principal_curvatures(
    half_angle: f64,
    v: f64,
) -> Result<PrincipalCurvatures, MathError> {
    if v <= 0.0 {
        return Err(MathError::ParameterOutOfRange {
            value: v,
            min: 0.0,
            max: f64::MAX,
        });
    }
    let k_ring = half_angle.tan() / v;
    Ok(PrincipalCurvatures {
        k1: k_ring,
        k2: 0.0,
    })
}

/// Principal curvatures of a toroidal surface at cross-section angle `v`.
///
/// `major` is the ring radius `R`, `minor` the tube radius `r`, and `v` is
/// measured from the outermost equator (`v = 0`) around the tube as in this
/// kernel's `ToroidalSurface` (`v = π` at the inner equator). Derivation for
/// `X(u, v) = ((R + r·cos v)·cos u, (R + r·cos v)·sin u,
/// r·sin v)` with the kernel's outward normal `N = cos v·radial(u) +
/// sin v·axis` (pointing away from the tube center):
///
/// - `E = (R + r·cos v)²`, `F = 0`, `G = r²`;
/// - `X_uu = −(R + r·cos v)·radial(u)`, so `L = −N·X_uu =
///   (R + r·cos v)·cos v`;
/// - `X_vv = −r·(cos v·radial(u) + sin v·axis) = −r·N`, so `N₂ = −N·X_vv = r`;
/// - `k_ring = L/E = cos v/(R + r·cos v)`, `k_tube = N₂/G = 1/r`.
///
/// Sign checks under the module convention: at the outer equator
/// (`v = 0`) both are positive — convex. At the inner equator (`v = π`) the
/// tube stays convex (`+1/r`, the tube circle center is behind the surface)
/// while the ring direction turns concave (`−1/(R − r)`, the ring circle's
/// center is on the normal side): the classic negative-Gaussian saddle. At
/// the top and bottom circles (`v = ±π/2`) the ring curvature vanishes.
///
/// # Errors
///
/// Returns [`MathError::SingularMatrix`] when `R + r·cos v <= 0` (only
/// reachable for self-intersecting spindle/horn configurations with
/// `r >= R`): the parallel circle degenerates and curvature is undefined.
pub fn torus_principal_curvatures(
    major: f64,
    minor: f64,
    v: f64,
) -> Result<PrincipalCurvatures, MathError> {
    let parallel = major + minor * v.cos();
    if parallel <= 0.0 {
        return Err(MathError::SingularMatrix);
    }
    let k_ring = v.cos() / parallel;
    let k_tube = 1.0 / minor;
    Ok(if k_ring >= k_tube {
        PrincipalCurvatures {
            k1: k_ring,
            k2: k_tube,
        }
    } else {
        PrincipalCurvatures {
            k1: k_tube,
            k2: k_ring,
        }
    })
}

/// Principal curvatures and directions from first- and second-order surface
/// derivatives at a point, via the first and second fundamental forms.
///
/// `xu`, `xv` are the first partials and `xuu`, `xuv`, `xvv` the second
/// partials of any `C²` parametrization `X(u, v)`. The reference normal is
/// the parametric normal `N = (xu × xv)/|xu × xv|` — the same normal every
/// surface type in this kernel reports from its `normal(u, v)` — so the
/// [module sign convention](self) applies unchanged.
///
/// The principal curvatures are the roots of
/// `det(II − k·I) = (L−kE)(N₂−kG) − (M−kF)² = 0`, i.e. of the quadratic
/// `A·k² − B·k + C = 0` with `A = EG − F² = |xu × xv|²`,
/// `B = L·G + E·N₂ − 2·M·F`, `C = L·N₂ − M²`. Principal directions solve the
/// singular system `(II − k·I)·d = 0` in parameter space and are mapped
/// through `d_u·Xu + d_v·Xv`.
///
/// # Errors
///
/// Returns [`MathError::SingularMatrix`] when the first fundamental form is
/// degenerate (`|xu × xv| ≈ 0`: the parametrization collapses to a point or
/// a line at this parameter, e.g. a sphere pole or a cone apex).
pub fn curvature_from_fundamental_forms(
    xu: Vec3,
    xv: Vec3,
    xuu: Vec3,
    xuv: Vec3,
    xvv: Vec3,
) -> Result<SurfaceCurvature, MathError> {
    let e = xu.dot(xu);
    let f = xu.dot(xv);
    let g = xv.dot(xv);
    // A = EG − F² equals |xu × xv|²; compare relatively so the degeneracy
    // guard holds at any parameter scale.
    let a = e * g - f * f;
    let scale = (e * g).max(f64::MIN_POSITIVE);
    if !a.is_finite() || a <= 1e-14 * scale {
        return Err(MathError::SingularMatrix);
    }
    let normal = xu.cross(xv).normalize()?;

    // Second fundamental form under the module convention: II_ij = −N·X_ij.
    let l = -normal.dot(xuu);
    let m = -normal.dot(xuv);
    let n2 = -normal.dot(xvv);

    let b = l * g + e * n2 - 2.0 * m * f;
    // Discriminant, via the algebraically equivalent but numerically stable
    // identity Δ = (L·G − E·N₂)² + 4(L·F − E·M)(M·F − N₂·G) rather than
    // B² − 4·A·C: at a near-umbilic point B² and 4·A·C nearly cancel, while
    // the identity squares differences of same-order terms.
    let delta = (l * g - e * n2).mul_add(l * g - e * n2, 4.0 * (l * f - e * m) * (m * f - n2 * g));
    // Mean H = B/(2A) and half-split s = √Δ/(2A); k1,2 = H ± s. Both H and
    // the products under √ carry double-precision form noise, so the split
    // is only trustworthy down to roughly √ε relative — below that, the
    // sign of k1 − k2 (and any principal direction, whose eigenproblem is
    // equally ill-conditioned) would be fabricated noise. Snap such points
    // to the stable mean and report them as umbilic.
    let mean = b / (2.0 * a);
    let half_split = delta.max(0.0).sqrt() / (2.0 * a);
    let umbilic_threshold = UMBILIC_REL_TOL * (mean.abs() + 1.0);
    if half_split <= umbilic_threshold {
        return Ok(SurfaceCurvature {
            k1: mean,
            k2: mean,
            directions: None,
        });
    }
    let k1 = mean + half_split;
    let k2 = mean - half_split;

    let forms = FundamentalForms {
        xu,
        xv,
        l,
        m,
        n2,
        e,
        f,
        g,
    };
    let dir1 = forms.principal_direction(k1)?;
    let dir2 = forms.principal_direction(k2)?;
    Ok(SurfaceCurvature {
        k1,
        k2,
        directions: Some((dir1, dir2)),
    })
}

/// First- and second-form coefficients plus the derivative basis vectors at
/// one surface point.
struct FundamentalForms {
    xu: Vec3,
    xv: Vec3,
    l: f64,
    m: f64,
    n2: f64,
    e: f64,
    f: f64,
    g: f64,
}

impl FundamentalForms {
    /// Unit 3D principal direction for eigenvalue `k` of the shape operator.
    ///
    /// Solves `(II − k·I)·(du, dv)ᵀ = 0` using whichever matrix row has the
    /// larger norm (at a non-umbilic point they cannot both vanish), then
    /// lifts the parameter-space direction through the first fundamental
    /// form basis.
    fn principal_direction(&self, k: f64) -> Result<Vec3, MathError> {
        // Rows of (II − k·I): (L − kE, M − kF) and (M − kF, N₂ − kG).
        let r1 = (self.l - k * self.e, self.m - k * self.f);
        let r2 = (self.m - k * self.f, self.n2 - k * self.g);
        let (du, dv) = if r1.0.hypot(r1.1) >= r2.0.hypot(r2.1) {
            // A vector orthogonal to the chosen row: (−b, a) for row (a, b).
            (-r1.1, r1.0)
        } else {
            (-r2.1, r2.0)
        };
        (self.xu * du + self.xv * dv).normalize()
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::{
        PrincipalCurvatures, cone_principal_curvatures, cylinder_principal_curvatures,
        sphere_principal_curvatures, torus_principal_curvatures,
    };
    use crate::MathError;

    fn assert_close(actual: f64, expected: f64, what: &str) {
        assert!(
            (actual - expected).abs() <= 1e-12 * expected.abs().max(1.0),
            "{what}: {actual} vs {expected}"
        );
    }

    fn expect(k1: f64, k2: f64, p: PrincipalCurvatures) {
        assert_close(p.k1, k1, "k1");
        assert_close(p.k2, k2, "k2");
    }

    #[test]
    fn sphere_is_umbilic_positive() {
        expect(2.0, 2.0, sphere_principal_curvatures(0.5));
        assert!(sphere_principal_curvatures(0.5).is_umbilic());
        assert_close(sphere_principal_curvatures(0.5).gaussian(), 4.0, "sphere K");
    }

    #[test]
    fn cylinder_has_axial_zero() {
        expect(0.25, 0.0, cylinder_principal_curvatures(4.0));
        assert!(!cylinder_principal_curvatures(4.0).is_umbilic());
        assert_close(
            cylinder_principal_curvatures(4.0).mean(),
            0.125,
            "cylinder H",
        );
    }

    #[test]
    fn cone_curvature_is_tan_alpha_over_slant() {
        // Half angle 45°: tan α = 1 → k = 1/v exactly.
        expect(
            1.0 / 2.0,
            0.0,
            cone_principal_curvatures(std::f64::consts::FRAC_PI_4, 2.0).unwrap(),
        );
        // sin α / ρ equivalence: ρ = v·cos α.
        let alpha = 0.6_f64;
        let v = 3.0;
        let rho = v * alpha.cos();
        expect(
            alpha.sin() / rho,
            0.0,
            cone_principal_curvatures(alpha, v).unwrap(),
        );
    }

    #[test]
    fn cone_curvature_undefined_at_apex() {
        assert!(matches!(
            cone_principal_curvatures(0.5, 0.0),
            Err(MathError::ParameterOutOfRange { .. })
        ));
        assert!(cone_principal_curvatures(0.5, -1.0).is_err());
    }

    #[test]
    fn torus_special_parallels_match_closed_forms() {
        let (r, mnr) = (4.0_f64, 1.0_f64);
        let pi = std::f64::consts::PI;
        // Outer equator: both convex; tube curvature dominates.
        expect(
            1.0 / mnr,
            1.0 / (r + mnr),
            torus_principal_curvatures(r, mnr, 0.0).unwrap(),
        );
        // Inner equator: ring direction concave (saddle).
        expect(
            1.0 / mnr,
            -1.0 / (r - mnr),
            torus_principal_curvatures(r, mnr, pi).unwrap(),
        );
        // Top / bottom: ring curvature vanishes.
        expect(
            1.0 / mnr,
            0.0,
            torus_principal_curvatures(r, mnr, pi * 0.5).unwrap(),
        );
        expect(
            1.0 / mnr,
            0.0,
            torus_principal_curvatures(r, mnr, -pi * 0.5).unwrap(),
        );
        // Gaussian at the inner equator is negative.
        assert!(torus_principal_curvatures(r, mnr, pi).unwrap().gaussian() < 0.0);
    }

    #[test]
    fn torus_degenerate_parallel_is_singular() {
        // Spindle configuration: R + r·cos v = 0 at cos v = -R/r.
        assert!(matches!(
            torus_principal_curvatures(0.5, 1.0, std::f64::consts::PI),
            Err(MathError::SingularMatrix)
        ));
    }
}
