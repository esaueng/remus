//! The wheel a double-enveloping (Hindley) worm drives.
//!
//! A Hindley worm's thread is the surface the wheel's straight flank lines
//! sweep, so in the wheel's mid-plane (the plane through the worm axis
//! perpendicular to the wheel axis) the wheel's tooth is exactly that
//! straight-sided tooth, tangent to the base circle `r_b = r₂·sin α`, and
//! worm and wheel touch along its flank lines. Off the mid-plane the pair
//! has no second line of contact: the meshing condition has no root on the
//! worm's flanks there, so what a hob shaped like the worm leaves of the
//! wheel's flank is cut by the hob's ends, not enveloped by its thread.
//!
//! This wheel keeps the mid-plane tooth exact. Off the mid-plane each
//! flank's transverse section stays a straight line, `x·n(ν(s)) = d(s)`
//! with `ν` and `d` quadratic in the height `s` from the mid-plane, chosen
//! so that across the face width the tooth stays inside the tooth a hob of
//! the worm's own length cuts. Every position of the hob passes through the
//! mid-plane line with its own tangent plane, so the hobbed tooth is
//! creased along that line and no single smooth flank through it stays
//! inside on both sides: each flank is two faces, one law below the
//! mid-plane and one above, meeting along the exact line. The plus flank
//! mirrors the minus flank. The fit is checked against the hobbed tooth at
//! sampled heights and radii, not proven between them, and the largest
//! relief it leaves is reported.
//!
//! The hobbed tooth here is the complement of every position of the worm's
//! material as wheel and worm turn together ([`HobSweep`]): a point is cut
//! when some wheel angle puts it inside the worm.

use std::f64::consts::{PI, TAU};

use remus_math::compact_sort::CompactSort;
use remus_math::curves::Circle3D;
use remus_math::nurbs::NurbsSurface;
use remus_math::nurbs::hermite_fit::{HermiteFit, fit_cubic_hermite};
use remus_math::surfaces::{CylindricalSurface, ToroidalSurface};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use super::enveloping::{EnvelopingWormParams, Hindley, cap_loop, enveloping_worm_geometry};
use super::jet::{Jet, compose, product, sqrt_jet};
use super::{
    DEFAULT_TOLERANCE_PER_MODULE, Frame, GearEnd, GearError, GearFace, GearFaceRole, GearKind,
    GearPlacement, GearRefusal, ToothSide, positive, require,
};

/// The worm used as a hob, in the wheel's mid-plane frame: `x` along the
/// wheel axis (`s` from the mid-plane), `(y, z)` the transverse plane about
/// the wheel center, with the worm axis on the `−y` side, and polar angles
/// `θ` about the wheel axis measured from `+y` toward `+z`, so the wheel's
/// tooth 0 faces the worm at `θ = π`.
pub(super) struct HobSweep {
    hindley: Hindley,
    /// Wheel angle over which some part of the worm's blank can overlap a
    /// point, either side of the angle that brings it under the worm axis.
    reach: f64,
    /// Samples per radian of wheel angle before refinement.
    density: f64,
}

impl HobSweep {
    pub(super) fn new(worm: &EnvelopingWormParams) -> Self {
        let hindley = Hindley::new(worm, 0.0);
        let reach = 1.05 * (hindley.half / hindley.rho_tip).min(1.0).asin();
        // The groove pattern repeats every tooth pitch of wheel angle; four
        // hundred samples a pitch resolve the thinnest land or groove.
        let pitch = 2.0 * PI / f64::from(hindley.wheel_teeth);
        Self {
            hindley,
            reach,
            density: 400.0 / pitch,
        }
    }

    pub(super) const fn hindley(&self) -> &Hindley {
        &self.hindley
    }

    /// The worm-frame image of a wheel point `(s, y, z)` with the wheel
    /// turned by `phi` and the worm with it.
    fn to_worm(&self, w: Point3, phi: f64) -> Point3 {
        let h = &self.hindley;
        let (sin, cos) = phi.sin_cos();
        let y = cos.mul_add(w.y(), -sin * w.z()) + h.a;
        let z = sin.mul_add(w.y(), cos * w.z());
        let turn = -h.sigma * h.ratio * phi;
        let (sb, cb) = turn.sin_cos();
        Point3::new(cb.mul_add(w.x(), -sb * y), sb.mul_add(w.x(), cb * y), z)
    }

    /// How deep the hob reaches into a wheel point: the largest material
    /// margin over every wheel angle. Positive where the hob cuts.
    pub(super) fn depth(&self, w: Point3) -> f64 {
        let theta = w.z().atan2(w.y()).rem_euclid(2.0 * PI);
        let center = PI - theta;
        let (lo, hi) = (center - self.reach, center + self.reach);
        let count = ((hi - lo) * self.density).ceil().max(8.0) as usize;
        let step = (hi - lo) / count as f64;
        let margin = |phi: f64| self.hindley.material_margin(self.to_worm(w, phi));
        let mut samples: Vec<(f64, f64)> = (0..=count)
            .map(|i| {
                let phi = lo + step * i as f64;
                (phi, margin(phi))
            })
            .collect();
        // Refine around the best few samples by golden-section search: the
        // margin is piecewise smooth, so the maximum sits near one of them.
        samples.compact_sort_by(|a, b| b.1.total_cmp(&a.1));
        let mut best = samples[0].1;
        for &(phi, _) in samples.iter().take(4) {
            let (mut a, mut b) = (phi - step, phi + step);
            let ratio = 0.5 * (5.0_f64.sqrt() - 1.0);
            let mut c = b - ratio * (b - a);
            let mut d = a + ratio * (b - a);
            let (mut fc, mut fd) = (margin(c), margin(d));
            for _ in 0..40 {
                if fc > fd {
                    b = d;
                    d = c;
                    fd = fc;
                    c = b - ratio * (b - a);
                    fc = margin(c);
                } else {
                    a = c;
                    c = d;
                    fc = fd;
                    d = a + ratio * (b - a);
                    fd = margin(d);
                }
            }
            best = best.max(fc).max(fd);
        }
        best
    }

    /// The polar angle of the hobbed tooth 0's `side` edge at height `s`
    /// and radius `rho`: where the hob's depth crosses zero next to the
    /// straight flank line.
    pub(super) fn edge(&self, side: ToothSide, s: f64, rho: f64) -> HobEdge {
        let at = |theta: f64| {
            let (sin, cos) = theta.sin_cos();
            self.depth(Point3::new(s, rho * cos, rho * sin))
        };
        // Bracket between the tooth's center (θ = π), which the hob must
        // leave, and the center of the space beside this flank, which it
        // cuts wherever it reaches at all.
        let pitch = 2.0 * PI / f64::from(self.hindley.wheel_teeth);
        let (mut cut, mut kept) = match side {
            ToothSide::Minus => (PI - 0.5 * pitch, PI),
            ToothSide::Plus => (PI + 0.5 * pitch, PI),
        };
        if at(kept) > 0.0 {
            return HobEdge::Blocked;
        }
        if at(cut) <= 0.0 {
            return HobEdge::Clear;
        }
        for _ in 0..50 {
            let mid = 0.5 * (cut + kept);
            if at(mid) > 0.0 {
                cut = mid;
            } else {
                kept = mid;
            }
        }
        HobEdge::At(0.5 * (cut + kept))
    }
}

/// Where the hob leaves a tooth edge at one height and radius.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum HobEdge {
    /// The edge's polar angle.
    At(f64),
    /// The hob never reaches the tooth or its neighbouring space here.
    Clear,
    /// The hob cuts into the tooth well inside its flank line.
    Blocked,
}

/// A wheel flank's straight transverse section at height `s` from the
/// mid-plane: the line `x·n(ν) = d` about the wheel axis, with
/// `ν(s) = ν₀ + α₁·s + α₂·s²` and `d(s) = r_b + β₁·s + β₂·s²`. In the
/// mid-plane it is the Hindley flank line, tangent to the base circle.
///
/// This is tooth 0's minus flank in the hob frame (the tooth centered at
/// `θ = π`); the plus flank is its mirror image under
/// `(s, θ) → (−s, 2π − θ)`, the symmetry of the hobbed tooth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct FlankLaw {
    pub(super) nu0: f64,
    pub(super) rb: f64,
    pub(super) alpha: [f64; 2],
    pub(super) beta: [f64; 2],
}

impl FlankLaw {
    fn nu(self, s: f64) -> (f64, f64) {
        (
            self.alpha[1].mul_add(s * s, self.alpha[0].mul_add(s, self.nu0)),
            (2.0 * self.alpha[1]).mul_add(s, self.alpha[0]),
        )
    }

    fn d(self, s: f64) -> (f64, f64) {
        (
            self.beta[1].mul_add(s * s, self.beta[0].mul_add(s, self.rb)),
            (2.0 * self.beta[1]).mul_add(s, self.beta[0]),
        )
    }

    /// The minus flank's polar angle at radius `rho` and height `s`, and its
    /// derivative along `s` at a fixed radius.
    pub(super) fn minus(self, rho: f64, s: f64) -> (f64, f64) {
        let (nu, dnu) = self.nu(s);
        let (d, dd) = self.d(s);
        let x = d / rho;
        (nu + x.acos(), dnu - dd / (rho * x.mul_add(-x, 1.0).sqrt()))
    }

    /// Either flank's polar angle in the hob frame and its height
    /// derivative.
    pub(super) fn angle(self, side: ToothSide, rho: f64, s: f64) -> (f64, f64) {
        match side {
            ToothSide::Minus => self.minus(rho, s),
            ToothSide::Plus => {
                let (theta, rate) = self.minus(rho, -s);
                (2.0 * PI - theta, rate)
            }
        }
    }

    /// Either flank's `∂θ/∂ρ` at a fixed height.
    fn radial(self, side: ToothSide, rho: f64, s: f64) -> f64 {
        let at = match side {
            ToothSide::Minus => s,
            ToothSide::Plus => -s,
        };
        let x = self.d(at).0 / rho;
        let slope = x / (rho * x.mul_add(-x, 1.0).sqrt());
        match side {
            ToothSide::Minus => slope,
            ToothSide::Plus => -slope,
        }
    }

    /// `|d(s)|` bounds over `|s| ≤ half`.
    fn d_range(self, half: f64) -> (f64, f64) {
        let spread = self.beta[0]
            .abs()
            .mul_add(half, self.beta[1].abs() * half * half);
        (self.rb - spread, self.rb + spread)
    }
}

/// The flank laws below and above the mid-plane that keep the straight-sided
/// tooth inside the hobbed tooth at every sampled height `s` in
/// `[−half, half]` and radius in `[rho_lo, top(s)]`; and the largest sampled
/// relief (arc length at the sample radius) they leave. `None` when the hob
/// cuts into a tooth's center somewhere.
///
/// The hob's surfaces at every mesh phase all pass through the mid-plane
/// line with different tangent planes, so the hobbed tooth is creased along
/// it: no one smooth law through the line stays inside on both sides, and
/// each side gets its own. At each height the minus flank's section must
/// stay on the tooth side of every hobbed edge point, `θ(ρ) ≥ e(ρ)`; the
/// plus samples join as the mirror image. Each half's law is a least
/// squares fit of its four coefficients to the edge samples, pushed up
/// where it cuts in until none does.
pub(super) fn fit_flank(
    hob: &HobSweep,
    half: f64,
    rho_lo: f64,
    top: impl Fn(f64) -> f64,
    heights: usize,
    radii: usize,
) -> Option<([FlankLaw; 2], f64)> {
    let h = hob.hindley();
    // (height, radius, hobbed minus edge) constraints, mirrored plus ones
    // included, split below and above the mid-plane.
    let mut samples: [Vec<(f64, f64, f64)>; 2] = [Vec::new(), Vec::new()];
    for i in 0..=heights {
        let s = half * (2.0 * i as f64 / heights as f64 - 1.0);
        if s.abs() < 1e-9 * half {
            continue;
        }
        let rho_hi = top(s);
        for j in 0..=radii {
            let rho = rho_lo + (rho_hi - rho_lo) * j as f64 / radii as f64;
            for side in [ToothSide::Minus, ToothSide::Plus] {
                let sample = match hob.edge(side, s, rho) {
                    HobEdge::At(edge) => match side {
                        ToothSide::Minus => (s, rho, edge),
                        ToothSide::Plus => (-s, rho, 2.0 * PI - edge),
                    },
                    HobEdge::Clear => continue,
                    HobEdge::Blocked => return None,
                };
                samples[usize::from(sample.0 > 0.0)].push(sample);
            }
        }
    }
    let base = FlankLaw {
        // At the base radius acos(r_b/ρ) vanishes, leaving the line's normal.
        nu0: h.line_angle(ToothSide::Minus, h.rb),
        rb: h.rb,
        alpha: [0.0, 0.0],
        beta: [0.0, 0.0],
    };
    let lower = fit_half(&samples[0], base)?;
    let upper = fit_half(&samples[1], base)?;
    let relief = samples[0]
        .iter()
        .map(|&(s, rho, e)| (lower.minus(rho, s).0 - e) * rho)
        .chain(
            samples[1]
                .iter()
                .map(|&(s, rho, e)| (upper.minus(rho, s).0 - e) * rho),
        )
        .fold(0.0, f64::max);
    Some(([lower, upper], relief))
}

/// One half's law: Gauss-Newton least squares to targets that start at the
/// edge samples and rise wherever the fit cuts in, then `α₁` raised (`s`
/// keeps one sign on a half) until no sample is cut.
fn fit_half(samples: &[(f64, f64, f64)], base: FlankLaw) -> Option<FlankLaw> {
    if samples.is_empty() {
        return Some(base);
    }
    let mut targets: Vec<f64> = samples.iter().map(|e| e.2).collect();
    let mut law = base;
    for _ in 0..24 {
        for _ in 0..8 {
            let mut normal = [[0.0; 5]; 4];
            for (&(s, rho, _), &target) in samples.iter().zip(&targets) {
                let x = law.d(s).0 / rho;
                let slope = 1.0 / (rho * x.mul_add(-x, 1.0).sqrt());
                // Rows in arc length.
                let row = [s, s * s, -s * slope, -s * s * slope].map(|j| j * rho);
                let residual = (target - law.minus(rho, s).0) * rho;
                for i in 0..4 {
                    for k in 0..4 {
                        normal[i][k] += row[i] * row[k];
                    }
                    normal[i][4] += row[i] * residual;
                }
            }
            let step = solve4(normal)?;
            law.alpha[0] += step[0];
            law.alpha[1] += step[1];
            law.beta[0] += step[2];
            law.beta[1] += step[3];
            if step.iter().all(|v| v.abs() < 1e-15) {
                break;
            }
        }
        let mut cut = false;
        for (&(s, rho, e), target) in samples.iter().zip(&mut targets) {
            let over = e - law.minus(rho, s).0;
            if over > 0.0 {
                *target += over;
                cut = true;
            }
        }
        if !cut {
            break;
        }
    }
    let raise = samples
        .iter()
        .map(|&(s, rho, e)| (e - law.minus(rho, s).0) / s.abs())
        .fold(0.0, f64::max);
    let sign = samples[0].0.signum();
    law.alpha[0] += sign * raise * (1.0 + 1e-6);
    Some(law)
}

/// Solves a 4×4 system given as rows `[a₀ a₁ a₂ a₃ | b]` by Gaussian
/// elimination with partial pivoting; `None` when it is singular.
fn solve4(mut m: [[f64; 5]; 4]) -> Option<[f64; 4]> {
    for col in 0..4 {
        let pivot = (col..4).max_by(|&a, &b| m[a][col].abs().total_cmp(&m[b][col].abs()))?;
        m.swap(col, pivot);
        if m[col][col].abs() < 1e-300 {
            return None;
        }
        for row in col + 1..4 {
            let factor = m[row][col] / m[col][col];
            for k in col..5 {
                m[row][k] -= factor * m[col][k];
            }
        }
    }
    let mut x = [0.0; 4];
    for row in (0..4).rev() {
        let tail: f64 = (row + 1..4).map(|k| m[row][k] * x[k]).sum();
        x[row] = (m[row][4] - tail) / m[row][row];
    }
    Some(x)
}

/// Uniform Hermite spans allowed along one fitted wheel curve.
const MAX_WHEEL_SPANS: usize = 1024;

/// Options for [`make_enveloping_worm_wheel`].
#[derive(Debug, Clone, PartialEq)]
pub struct EnvelopingWheelOptions {
    /// Face width along the wheel axis, centered on the worm.
    pub face_width: f64,
    /// Thinning of each wheel tooth along its pitch circle (length).
    pub backlash: f64,
    /// Outside diameter beside the throat; `None` is the throat diameter
    /// plus one module.
    pub outside_diameter: Option<f64>,
    /// Flank fit tolerance; `None` uses
    /// [`DEFAULT_TOLERANCE_PER_MODULE`]·`m`.
    pub tolerance: Option<f64>,
    /// Optional concentric bore diameter.
    pub bore_diameter: Option<f64>,
}

impl EnvelopingWheelOptions {
    /// A wheel of `face_width` with no backlash or bore and the default
    /// outside diameter and tolerance.
    #[must_use]
    pub const fn new(face_width: f64) -> Self {
        Self {
            face_width,
            backlash: 0.0,
            outside_diameter: None,
            tolerance: None,
            bore_diameter: None,
        }
    }
}

/// A built double-enveloping wheel.
#[derive(Debug, Clone, PartialEq)]
pub struct EnvelopingWheelOutcome {
    /// The wheel body.
    pub solid: SolidId,
    /// Center distance to the worm axis.
    pub center_distance: f64,
    /// Pitch diameter `z₂·m`.
    pub pitch_diameter: f64,
    /// Tip (throat) diameter in the mid-plane, `d₂ + 2·h_a·m`.
    pub throat_diameter: f64,
    /// Root diameter `d₂ − 2·h_f·m`.
    pub root_diameter: f64,
    /// Outside diameter beside the throat.
    pub outside_diameter: f64,
    /// Radius of the throat torus about the worm axis.
    pub throat_radius: f64,
    /// Width of the throat along the wheel axis.
    pub throat_width: f64,
    /// The minus flank's section off the mid-plane is the line `x·n(ν) = d`
    /// with `ν(s) = ν₀ + α₁·s + α₂·s²` (`s` from the mid-plane): `[α₁, α₂]`
    /// below the mid-plane, then above it, where the flank creases along the
    /// mid-plane line as the hobbed tooth does. The plus flank is its mirror
    /// image, `(s, θ) → (−s, −θ)` about the tooth center.
    pub flank_turn: [[f64; 2]; 2],
    /// And `d(s) = r_b + β₁·s + β₂·s²`: `[β₁, β₂]` below, then above.
    pub flank_offset: [[f64; 2]; 2],
    /// The largest relief the flanks leave inside the hobbed tooth at the
    /// sampled heights and radii, as arc length: sampled, not proven.
    pub relief: f64,
    /// Proven bound on the flank, tip and throat fits' distance from the
    /// modelled surfaces, with a floating-point allowance.
    pub flank_deviation_bound: f64,
    /// Where to place the worm (with [`make_enveloping_worm`]) so it meshes
    /// with this wheel.
    ///
    /// [`make_enveloping_worm`]: super::make_enveloping_worm
    pub worm_placement: GearPlacement,
    /// The role of every face (`tooth[k].flank.plus.start-half`, `tooth[k].throat`,
    /// `tooth[k].tip.start-half`, `root[k]`, `cap.start`, `bore`).
    pub face_roles: Vec<(FaceId, GearFace)>,
}

fn refused(code: GearRefusal, reason: impl Into<String>) -> GearError {
    GearError::refused(code, reason)
}

fn fit_failure(error: remus_math::MathError, tolerance: f64) -> GearError {
    match error {
        remus_math::MathError::ResourceLimitExceeded { .. } => refused(
            GearRefusal::ToleranceTooTight,
            format!(
                "the wheel's flanks need more than {MAX_WHEEL_SPANS} spans for a tolerance of \
                 {tolerance}"
            ),
        ),
        other => other.into(),
    }
}

/// The wheel's construction quantities in its local frame: axis `z`, the
/// worm on `+x` at height `c` (half the face width), the tooth angles `ψ`
/// about the axis from `+x`.
struct Wheel {
    hindley: Hindley,
    /// The minus flank's laws below and above the mid-plane.
    laws: [FlankLaw; 2],
    teeth: u32,
    phase: f64,
    /// Wheel backlash as an angle on each flank.
    thinning: f64,
    root: f64,
    tip: f64,
    outside: f64,
    tube: f64,
    mid: f64,
    half_throat: f64,
}

impl Wheel {
    /// The law flank `side` follows on `half` (0 below the mid-plane, 1
    /// above): the plus flank mirrors the minus flank's other half.
    const fn law(&self, side: ToothSide, half: usize) -> FlankLaw {
        match side {
            ToothSide::Minus => self.laws[half],
            ToothSide::Plus => self.laws[1 - half],
        }
    }

    /// The half holding height `s`; the mid-plane itself is on both.
    fn half_at(&self, s: f64) -> usize {
        usize::from(s > self.mid)
    }

    /// The tooth `k` flank `side`'s polar angle at radius `rho` and height
    /// `s` (from the first cap) on `half`.
    fn angle(&self, k: u32, side: ToothSide, rho: f64, s: f64, half: usize) -> f64 {
        let shift = match side {
            ToothSide::Minus => self.thinning,
            ToothSide::Plus => -self.thinning,
        };
        TAU.mul_add(f64::from(k) / f64::from(self.teeth), self.phase)
            + (self.law(side, half).angle(side, rho, s - self.mid).0 - PI)
            + shift
    }

    /// Its derivative along the height at a fixed radius.
    fn angle_rate(&self, side: ToothSide, rho: f64, s: f64, half: usize) -> f64 {
        self.law(side, half).angle(side, rho, s - self.mid).1
    }

    /// Its derivative along the radius at a fixed height.
    fn angle_radial(&self, side: ToothSide, rho: f64, s: f64, half: usize) -> f64 {
        self.law(side, half).radial(side, rho, s - self.mid)
    }

    /// The throat radius at height `s`, within the throat.
    fn throat_radius(&self, s: f64) -> (f64, f64) {
        let u = s - self.mid;
        let g = u.mul_add(-u, self.tube * self.tube).max(0.0).sqrt();
        (self.hindley.a - g, u / g)
    }

    /// Whether a local point is inside tooth `k`'s material.
    fn in_tooth(&self, k: u32, p: Point3) -> bool {
        let rho = p.x().hypot(p.y());
        let s = p.z();
        let top = if (s - self.mid).abs() < self.half_throat {
            self.throat_radius(s).0
        } else {
            self.outside
        };
        if rho < self.root || rho > top {
            return false;
        }
        let half = self.half_at(s);
        let lo = self.angle(k, ToothSide::Minus, rho, s, half);
        let hi = self.angle(k, ToothSide::Plus, rho, s, half);
        (p.y().atan2(p.x()) - lo).rem_euclid(TAU) <= hi - lo
    }
}

/// Bounds on `acos x` and its derivatives over `x ∈ [lo, hi]`, `0 ≤ lo ≤
/// hi < 1`: `1/√(1−x²)`, `x/(1−x²)^{3/2}`, `(1 + 2x²)/(1−x²)^{5/2}` and
/// `(9x + 6x³)/(1−x²)^{7/2}` all grow with `x`, so peak at `hi`.
fn acos_jet(hi: f64) -> Jet {
    let q = hi.mul_add(-hi, 1.0);
    [
        PI,
        1.0 / q.sqrt(),
        hi / q.powf(1.5),
        (2.0 * hi).mul_add(hi, 1.0) / q.powf(2.5),
        (6.0 * hi * hi).mul_add(hi, 9.0 * hi) / q.powf(3.5),
    ]
}

/// The jets of `ν(s)` and `d(s)` over `|s| ≤ half`.
fn law_jets(law: FlankLaw, half: f64) -> (Jet, Jet) {
    let nu: Jet = [
        PI,
        law.alpha[0].abs() + 2.0 * law.alpha[1].abs() * half,
        2.0 * law.alpha[1].abs(),
        0.0,
        0.0,
    ];
    let d: Jet = [
        law.d_range(half).1,
        law.beta[0].abs() + 2.0 * law.beta[1].abs() * half,
        2.0 * law.beta[1].abs(),
        0.0,
        0.0,
    ];
    (nu, d)
}

/// `ν + acos x` from their jets.
fn angle_jet(nu: Jet, x: Jet, x_hi: f64) -> Jet {
    let arc = compose(acos_jet(x_hi), x);
    [
        PI,
        nu[1] + arc[1],
        nu[2] + arc[2],
        nu[3] + arc[3],
        nu[4] + arc[4],
    ]
}

/// Bound on the fourth derivative of a flank rail: the section line's point
/// at radius `r` over `|s| ≤ half`, at polar angle `ν(s) ± acos(d(s)/r)`.
fn rail_bound(law: FlankLaw, r: f64, half: f64) -> f64 {
    let (nu, d) = law_jets(law, half);
    let x = d.map(|v| v / r);
    r * compose([1.0; 5], angle_jet(nu, x, x[0]))[4]
}

/// Bound on the fourth derivative of a throat curve: the section line's
/// point at the throat radius `R(s) = a − √(ρ² − (s − c)²)`, `|s − c| ≤ w`,
/// at polar angle `ν ± acos(d/R)`.
fn throat_bound(wheel: &Wheel, law: FlankLaw) -> f64 {
    let (a, rho, w) = (wheel.hindley.a, wheel.tube, wheel.half_throat);
    let q: Jet = [rho * rho, 2.0 * w, 2.0, 0.0, 0.0];
    let g = compose(sqrt_jet(w.mul_add(-w, rho * rho), rho * rho), q);
    let r: Jet = [wheel.outside, g[1], g[2], g[3], g[4]];
    let low = a - rho;
    let reciprocal = compose(
        [
            1.0 / low,
            1.0 / (low * low),
            2.0 / low.powi(3),
            6.0 / low.powi(4),
            24.0 / low.powi(5),
        ],
        r,
    );
    let (nu, d) = law_jets(law, w);
    let x = product(d, reciprocal);
    let x_hi = law.d_range(w).1 / low;
    product(r, compose([1.0; 5], angle_jet(nu, x, x_hi)))[4]
}

/// Build the wheel a double-enveloping (Hindley) worm drives.
///
/// In the mid-plane its teeth are exactly the straight-sided teeth the worm
/// was swept by. Off it each flank stays straight in every section, fitted
/// inside the tooth a hob of the worm's shape (without its backlash) cuts:
/// one law below the mid-plane and one above, since the hobbed tooth is
/// creased along the mid-plane line. The fit is sampled, not proven
/// ([`EnvelopingWheelOutcome::relief`]).
///
/// The first cap is centered on the placement origin and the wheel runs
/// `face_width` along the placement axis; the worm it meshes with crosses
/// the axis at 90° at half the face width, on the placement's x direction
/// at the center distance ([`EnvelopingWheelOutcome::worm_placement`]).
/// Tooth 0 faces the worm at `phase`.
///
/// The whole construction runs in one transaction: a refusal or failure
/// leaves the topology as it was.
///
/// # Errors
///
/// [`GearError::Refused`] for out-of-range parameters, a throat wider than
/// the face or outside diameters it cannot meet
/// ([`GearRefusal::InvalidParameter`]), a tooth that comes to a point or
/// closes the root ([`GearRefusal::PointedTip`],
/// [`GearRefusal::RootLandClosed`]), a bore reaching the root
/// ([`GearRefusal::BoreTooLarge`]), a hob that cuts into a tooth's center
/// ([`GearRefusal::InvalidParameter`]), or a tolerance needing too many spans
/// ([`GearRefusal::ToleranceTooTight`]).
#[allow(clippy::too_many_lines)]
pub fn make_enveloping_worm_wheel(
    topo: &mut Topology,
    worm: &EnvelopingWormParams,
    options: &EnvelopingWheelOptions,
    placement: &GearPlacement,
) -> Result<EnvelopingWheelOutcome, GearError> {
    enveloping_worm_geometry(worm)?;
    positive(options.face_width, "face width")?;
    require(
        options.backlash.is_finite() && options.backlash >= 0.0,
        || {
            format!(
                "backlash must be non-negative and finite, got {}",
                options.backlash
            )
        },
    )?;
    if let Some(tolerance) = options.tolerance {
        positive(tolerance, "tolerance")?;
    }
    if let Some(bore) = options.bore_diameter {
        positive(bore, "bore diameter")?;
    }
    let m = worm.module;
    // The wheel is cut by the worm without its thread thinning, so the
    // worm's backlash and the wheel's add up in the pair.
    let nominal = EnvelopingWormParams {
        backlash: 0.0,
        ..worm.clone()
    };
    let hindley = Hindley::new(&nominal, 0.0);
    let r2 = 0.5 * m * f64::from(worm.wheel_teeth);
    let tip = worm.addendum.mul_add(m, r2);
    let root = worm.dedendum.mul_add(-m, r2);
    require(root > hindley.rb, || {
        "the wheel's root reaches inside its base circle; use a smaller pressure angle or \
         dedendum"
            .to_owned()
    })?;
    let outside = options
        .outside_diameter
        .map_or_else(|| 0.5f64.mul_add(m, tip), |d| 0.5 * d);
    positive(outside, "outside diameter")?;
    let tube = hindley.a - tip;
    require(outside > tip && hindley.a - outside < tube, || {
        format!(
            "the outside diameter {} must lie between the throat diameter {} and {}",
            2.0 * outside,
            2.0 * tip,
            2.0 * (hindley.a + tube)
        )
    })?;
    let half_throat = (tube * tube - (hindley.a - outside).powi(2)).sqrt();
    let width = options.face_width;
    let mid = 0.5 * width;
    require(2.0 * half_throat < width, || {
        format!(
            "the throat ({:.6} wide) must be narrower than the face width {width}",
            2.0 * half_throat
        )
    })?;
    require(options.backlash < 0.5 * PI * m, || {
        "backlash must be less than half the circular pitch".to_owned()
    })?;
    if let Some(bore) = options.bore_diameter
        && bore >= 2.0 * root
    {
        return Err(refused(
            GearRefusal::BoreTooLarge,
            format!("the bore {bore} reaches the root diameter {}", 2.0 * root),
        ));
    }

    let hob = HobSweep::new(&nominal);
    // Sample each height up to its own tip: the throat radius across the
    // throat, the outside radius beside it.
    let top = |s: f64| {
        if s.abs() < half_throat {
            hindley.a - s.mul_add(-s, tube * tube).sqrt()
        } else {
            outside
        }
    };
    let (laws, relief) = fit_flank(&hob, mid, root, top, 16, 8).ok_or_else(|| {
        refused(
            GearRefusal::InvalidParameter,
            "the worm cuts into the wheel's teeth near its faces; use a narrower face or a \
             smaller outside diameter",
        )
    })?;
    let wheel = Wheel {
        hindley,
        laws,
        teeth: worm.wheel_teeth,
        phase: placement.phase,
        thinning: options.backlash / (2.0 * r2),
        root,
        tip,
        outside,
        tube,
        mid,
        half_throat,
    };
    // Every tooth keeps its tip and every gap its root across the face.
    let pitch = TAU / f64::from(wheel.teeth);
    for (s, half, top) in [(0.0, 0, outside), (mid, 0, tip), (width, 1, outside)] {
        let thick = wheel.angle(0, ToothSide::Plus, top, s, half)
            - wheel.angle(0, ToothSide::Minus, top, s, half);
        if thick <= 0.0 {
            return Err(refused(
                GearRefusal::PointedTip,
                format!(
                    "the wheel's teeth come to a point at the diameter {}",
                    2.0 * top
                ),
            ));
        }
        let gap = wheel.angle(0, ToothSide::Minus, root, s, half) + pitch
            - wheel.angle(0, ToothSide::Plus, root, s, half);
        if gap <= 0.0 {
            return Err(refused(
                GearRefusal::RootLandClosed,
                "the wheel's teeth close the root",
            ));
        }
    }
    let frame = Frame::new(placement, GearKind::External)?;
    let tolerance = options
        .tolerance
        .unwrap_or(DEFAULT_TOLERANCE_PER_MODULE * m);
    let worm_placement = GearPlacement {
        origin: frame.origin
            + frame.axis * mid
            + frame.x * hindley_a(&wheel)
            + frame.y * (0.5 * worm.length),
        axis: -frame.y,
        x_direction: -frame.x,
        phase: wheel.hindley.sigma * wheel.hindley.ratio * placement.phase,
    };
    remus_topology::transaction::run_transacted(topo, |topo| {
        build_wheel(
            topo,
            &wheel,
            &frame,
            width,
            tolerance,
            options.bore_diameter,
        )
        .map(|(solid, face_roles, bound)| EnvelopingWheelOutcome {
            solid,
            center_distance: wheel.hindley.a,
            pitch_diameter: 2.0 * r2,
            throat_diameter: 2.0 * tip,
            root_diameter: 2.0 * root,
            outside_diameter: 2.0 * outside,
            throat_radius: tube,
            throat_width: 2.0 * half_throat,
            flank_turn: [laws[0].alpha, laws[1].alpha],
            flank_offset: [laws[0].beta, laws[1].beta],
            relief,
            flank_deviation_bound: bound,
            worm_placement,
            face_roles,
        })
    })
}

const fn hindley_a(wheel: &Wheel) -> f64 {
    wheel.hindley.a
}

/// A circle about the wheel axis at height `s`, its parameter origin turned
/// opposite the arc `[lo, hi]` so the arc never crosses it; and the trim.
fn arc(
    frame: &Frame,
    s: f64,
    radius: f64,
    lo: f64,
    hi: f64,
) -> Result<(Circle3D, (f64, f64)), GearError> {
    let reference = 0.5 * (lo + hi) - PI;
    let (sin, cos) = reference.sin_cos();
    let u = frame.x * cos + frame.y * sin;
    let v = frame.axis.cross(u);
    let circle = Circle3D::with_axes(frame.origin + frame.axis * s, frame.axis, radius, u, v)?;
    Ok((circle, (lo - reference, hi - reference)))
}

/// The direction at polar angle `angle` in the transverse plane.
fn radial(frame: &Frame, angle: f64) -> Vec3 {
    let (sin, cos) = angle.sin_cos();
    frame.x * cos + frame.y * sin
}

/// Per tooth `k` and flank: its rails and throat curves below and above the
/// mid-plane, vertices and edges.
struct FlankParts {
    /// Root and tip rail fits per half.
    root: [HermiteFit; 2],
    tip: [HermiteFit; 2],
    /// Root vertices at heights `0, c, b`.
    root_vertices: [VertexId; 3],
    /// Tip vertices at heights `0, s₁, s₂, b`.
    tip_vertices: [VertexId; 4],
    /// Root rail per half.
    root_edges: [EdgeId; 2],
    /// Tip rail below and above the throat.
    tip_edges: [EdgeId; 2],
    /// Throat curve per half.
    throat_edges: [EdgeId; 2],
    /// Cap lines at the first and far caps.
    cap_lines: [EdgeId; 2],
    /// The exact straight section in the mid-plane, root to throat.
    mid_line: EdgeId,
}

#[allow(clippy::too_many_lines, clippy::type_complexity)]
fn build_wheel(
    topo: &mut Topology,
    wheel: &Wheel,
    frame: &Frame,
    width: f64,
    tolerance: f64,
    bore: Option<f64>,
) -> Result<(SolidId, Vec<(FaceId, GearFace)>, f64), GearError> {
    let mid = wheel.mid;
    let (s1, s2) = (mid - wheel.half_throat, mid + wheel.half_throat);
    let heights = [0.0, s1, s2, width];
    // Halves of heights[j]: the lower two below the mid-plane.
    let height_half = [0, 0, 1, 1];
    let spans = [(0.0, mid), (mid, width)];
    let mut rail_fourth: f64 = 0.0;
    let mut throat_fourth: f64 = 0.0;
    for law in wheel.laws {
        rail_fourth = rail_fourth
            .max(rail_bound(law, wheel.outside, mid))
            .max(rail_bound(law, wheel.root, mid));
        throat_fourth = throat_fourth.max(throat_bound(wheel, law));
    }
    let point =
        |rho: f64, angle: f64, s: f64| frame.origin + radial(frame, angle) * rho + frame.axis * s;
    let mut deviation: f64 = 0.0;
    let fit_rail = |k: u32, side: ToothSide, rho: f64, half: usize| {
        let (lo, hi) = spans[half];
        fit_cubic_hermite(
            |s| {
                let angle = wheel.angle(k, side, rho, s, half);
                let rate = wheel.angle_rate(side, rho, s, half);
                let (sin, cos) = angle.sin_cos();
                (
                    point(rho, angle, s),
                    (frame.y * cos - frame.x * sin) * (rho * rate) + frame.axis,
                )
            },
            lo,
            hi,
            rail_fourth,
            tolerance,
            MAX_WHEEL_SPANS,
        )
        .map_err(|error| fit_failure(error, tolerance))
    };
    let fit_throat = |k: u32, side: ToothSide, half: usize| {
        let (lo, hi) = [(s1, mid), (mid, s2)][half];
        fit_cubic_hermite(
            |s| {
                let (r, dr) = wheel.throat_radius(s);
                let angle = wheel.angle(k, side, r, s, half);
                let rate = wheel
                    .angle_radial(side, r, s, half)
                    .mul_add(dr, wheel.angle_rate(side, r, s, half));
                let (sin, cos) = angle.sin_cos();
                let out = frame.x * cos + frame.y * sin;
                let along = frame.y * cos - frame.x * sin;
                (
                    point(r, angle, s),
                    out * dr + along * (r * rate) + frame.axis,
                )
            },
            lo,
            hi,
            throat_fourth,
            tolerance,
            MAX_WHEEL_SPANS,
        )
        .map_err(|error| fit_failure(error, tolerance))
    };
    let scale = wheel.hindley.a + wheel.outside + width;
    let fp = 64.0 * f64::EPSILON * scale;
    let mut parts: Vec<[FlankParts; 2]> = Vec::with_capacity(wheel.teeth as usize);
    for k in 0..wheel.teeth {
        let mut both = Vec::with_capacity(2);
        for side in [ToothSide::Minus, ToothSide::Plus] {
            let root_fits = [
                fit_rail(k, side, wheel.root, 0)?,
                fit_rail(k, side, wheel.root, 1)?,
            ];
            let tip_fits = [
                fit_rail(k, side, wheel.outside, 0)?,
                fit_rail(k, side, wheel.outside, 1)?,
            ];
            let throats = [fit_throat(k, side, 0)?, fit_throat(k, side, 1)?];
            let rail_error = root_fits
                .iter()
                .chain(&tip_fits)
                .map(|fit| fit.remainder_bound)
                .fold(0.0, f64::max);
            let throat_error = throats
                .iter()
                .map(|fit| fit.remainder_bound)
                .fold(0.0, f64::max);
            deviation = deviation.max(rail_error).max(throat_error);
            let vertex = |topo: &mut Topology, rho: f64, s: f64, half: usize| {
                topo.add_vertex(Vertex::new(
                    point(rho, wheel.angle(k, side, rho, s, half), s),
                    tolerance.max(fp),
                ))
            };
            let root_vertices = [
                vertex(topo, wheel.root, 0.0, 0),
                vertex(topo, wheel.root, mid, 0),
                vertex(topo, wheel.root, width, 1),
            ];
            let tip_vertices = [
                vertex(topo, wheel.outside, 0.0, 0),
                vertex(topo, wheel.outside, s1, 0),
                vertex(topo, wheel.outside, s2, 1),
                vertex(topo, wheel.outside, width, 1),
            ];
            let throat_vertex = vertex(topo, wheel.tip, mid, 0);
            let bound = Some(rail_error + fp);
            let fitted = |topo: &mut Topology, from, to, curve: &HermiteFit, trim, bound| {
                let mut edge = Edge::with_tolerance(
                    from,
                    to,
                    EdgeCurve::NurbsCurve(curve.curve.clone()),
                    bound,
                );
                edge.set_trim(Some(trim));
                topo.add_edge(edge)
            };
            let root_edges = [
                fitted(
                    topo,
                    root_vertices[0],
                    root_vertices[1],
                    &root_fits[0],
                    (0.0, mid),
                    bound,
                ),
                fitted(
                    topo,
                    root_vertices[1],
                    root_vertices[2],
                    &root_fits[1],
                    (mid, width),
                    bound,
                ),
            ];
            let tip_edges = [
                fitted(
                    topo,
                    tip_vertices[0],
                    tip_vertices[1],
                    &tip_fits[0],
                    (0.0, s1),
                    bound,
                ),
                fitted(
                    topo,
                    tip_vertices[2],
                    tip_vertices[3],
                    &tip_fits[1],
                    (s2, width),
                    bound,
                ),
            ];
            let throat_bound = Some(throat_error + rail_error + fp);
            let throat_edges = [
                fitted(
                    topo,
                    tip_vertices[1],
                    throat_vertex,
                    &throats[0],
                    (s1, mid),
                    throat_bound,
                ),
                fitted(
                    topo,
                    throat_vertex,
                    tip_vertices[2],
                    &throats[1],
                    (mid, s2),
                    throat_bound,
                ),
            ];
            let line =
                |topo: &mut Topology, from, to| topo.add_edge(Edge::new(from, to, EdgeCurve::Line));
            let cap_lines = [
                line(topo, root_vertices[0], tip_vertices[0]),
                line(topo, root_vertices[2], tip_vertices[3]),
            ];
            let mid_line = line(topo, root_vertices[1], throat_vertex);
            both.push(FlankParts {
                root: root_fits,
                tip: tip_fits,
                root_vertices,
                tip_vertices,
                root_edges,
                tip_edges,
                throat_edges,
                cap_lines,
                mid_line,
            });
        }
        let plus = both
            .pop()
            .ok_or_else(|| GearError::invalid("a tooth has no plus flank"))?;
        let minus = both
            .pop()
            .ok_or_else(|| GearError::invalid("a tooth has no minus flank"))?;
        parts.push([minus, plus]);
    }
    let deviation = deviation + fp;

    let mut faces = Vec::new();
    let mut roles: Vec<(FaceId, GearFace)> = Vec::new();
    let mut cap_edges: [Vec<EdgeId>; 2] = [Vec::new(), Vec::new()];
    let tooth_arc = |topo: &mut Topology,
                     k: u32,
                     j: usize,
                     flanks: &[FlankParts; 2]|
     -> Result<EdgeId, GearError> {
        let s = heights[j];
        let half = height_half[j];
        let lo = wheel.angle(k, ToothSide::Minus, wheel.outside, s, half);
        let hi = wheel.angle(k, ToothSide::Plus, wheel.outside, s, half);
        let (circle, trim) = arc(frame, s, wheel.outside, lo, hi)?;
        let mut edge = Edge::with_tolerance(
            flanks[0].tip_vertices[j],
            flanks[1].tip_vertices[j],
            EdgeCurve::Circle(circle),
            Some(tolerance.max(fp)),
        );
        edge.set_trim(Some(trim));
        Ok(topo.add_edge(edge))
    };
    let add_face = |topo: &mut Topology,
                    sides: Vec<OrientedEdge>,
                    surface: FaceSurface,
                    reversed: bool|
     -> Result<FaceId, GearError> {
        let wire = topo.add_wire(Wire::new(sides, true)?);
        Ok(topo.add_face(if reversed {
            Face::new_reversed(wire, vec![], surface)
        } else {
            Face::new(wire, vec![], surface)
        }))
    };
    let local = |q: Point3| {
        let d = q - frame.origin;
        Point3::new(d.dot(frame.x), d.dot(frame.y), d.dot(frame.axis))
    };
    for k in 0..wheel.teeth {
        let flanks = &parts[k as usize];
        let arcs = [
            tooth_arc(topo, k, 0, flanks)?,
            tooth_arc(topo, k, 1, flanks)?,
            tooth_arc(topo, k, 2, flanks)?,
            tooth_arc(topo, k, 3, flanks)?,
        ];
        cap_edges[0].push(arcs[0]);
        cap_edges[1].push(arcs[3]);
        let mid_angle = 0.5
            * (wheel.angle(k, ToothSide::Minus, wheel.outside, mid, 0)
                + wheel.angle(k, ToothSide::Plus, wheel.outside, mid, 0));
        let reference = radial(frame, mid_angle - PI);
        let cylinder =
            CylindricalSurface::with_ref_dir(frame.origin, frame.axis, wheel.outside, reference)?;
        // Tip strips, counter-clockwise in the cylinder's (u, v): the lower
        // arc minus → plus, the plus rail up, the upper arc back, the minus
        // rail down.
        let (minus, plus) = (&flanks[0], &flanks[1]);
        let low = add_face(
            topo,
            vec![
                OrientedEdge::new(arcs[0], true),
                OrientedEdge::new(plus.tip_edges[0], true),
                OrientedEdge::new(arcs[1], false),
                OrientedEdge::new(minus.tip_edges[0], false),
            ],
            FaceSurface::Cylinder(cylinder.clone()),
            false,
        )?;
        let high = add_face(
            topo,
            vec![
                OrientedEdge::new(arcs[2], true),
                OrientedEdge::new(plus.tip_edges[1], true),
                OrientedEdge::new(arcs[3], false),
                OrientedEdge::new(minus.tip_edges[1], false),
            ],
            FaceSurface::Cylinder(cylinder),
            false,
        )?;
        // The throat patch, counter-clockwise in the torus's (u, v): v rises
        // as the height falls, so the upper arc runs first.
        let torus = ToroidalSurface::with_axis_and_ref_dir(
            frame.origin + frame.axis * mid,
            wheel.hindley.a,
            wheel.tube,
            frame.axis,
            reference,
        )?;
        let throat = add_face(
            topo,
            vec![
                OrientedEdge::new(arcs[2], true),
                OrientedEdge::new(plus.throat_edges[1], false),
                OrientedEdge::new(plus.throat_edges[0], false),
                OrientedEdge::new(arcs[1], false),
                OrientedEdge::new(minus.throat_edges[0], true),
                OrientedEdge::new(minus.throat_edges[1], true),
            ],
            FaceSurface::Torus(torus),
            true,
        )?;
        let tip_role = |half| GearFace {
            role: GearFaceRole::Tip { tooth: k },
            half: Some(half),
        };
        for (face, role) in [
            (low, tip_role(GearEnd::Start)),
            (throat, GearFaceRole::Throat { tooth: k }.into()),
            (high, tip_role(GearEnd::End)),
        ] {
            faces.push(face);
            roles.push((face, role));
        }
        // Flank halves: ruled between the root and tip rails, counter-
        // clockwise in (s, t): the root rail up, the upper line out, the tip
        // rail and throat curve back down, the lower line in. They meet along
        // the mid-plane line.
        for (index, side) in [ToothSide::Minus, ToothSide::Plus].into_iter().enumerate() {
            let flank = &flanks[index];
            for half in 0..2 {
                let (root, tip) = (&flank.root[half], &flank.tip[half]);
                let net: Vec<Vec<Point3>> = root
                    .curve
                    .control_points()
                    .iter()
                    .zip(tip.curve.control_points())
                    .map(|(&r, &t)| vec![r, t])
                    .collect();
                let weights = vec![vec![1.0; 2]; net.len()];
                let surface = NurbsSurface::new(
                    3,
                    1,
                    root.curve.knots().to_vec(),
                    vec![0.0, 0.0, 1.0, 1.0],
                    net,
                    weights,
                )?;
                let sides = if half == 0 {
                    vec![
                        OrientedEdge::new(flank.root_edges[0], true),
                        OrientedEdge::new(flank.mid_line, true),
                        OrientedEdge::new(flank.throat_edges[0], false),
                        OrientedEdge::new(flank.tip_edges[0], false),
                        OrientedEdge::new(flank.cap_lines[0], false),
                    ]
                } else {
                    vec![
                        OrientedEdge::new(flank.root_edges[1], true),
                        OrientedEdge::new(flank.cap_lines[1], true),
                        OrientedEdge::new(flank.tip_edges[1], false),
                        OrientedEdge::new(flank.throat_edges[1], false),
                        OrientedEdge::new(flank.mid_line, false),
                    ]
                };
                // ∂s × ∂t faces out where a step along it leaves the tooth.
                let s = 0.5 * (spans[half].0 + spans[half].1);
                let rho = 0.5 * (wheel.root + wheel.tip);
                let angle = wheel.angle(k, side, rho, s, half);
                let p = point(rho, angle, s);
                let (sin, cos) = angle.sin_cos();
                let along_s = (frame.y * cos - frame.x * sin)
                    * (rho * wheel.angle_rate(side, rho, s, half))
                    + frame.axis;
                let a_root = wheel.angle(k, side, wheel.root, s, half);
                let a_tip = wheel.angle(k, side, wheel.outside, s, half);
                let along_t = point(wheel.outside, a_tip, s) - point(wheel.root, a_root, s);
                let normal = along_s
                    .cross(along_t)
                    .normalize()
                    .map_err(|_| GearError::invalid("a wheel flank is degenerate"))?;
                let step = 1e-3 * (wheel.tip - wheel.root);
                let ahead = wheel.in_tooth(k, local(p + normal * step));
                let behind = wheel.in_tooth(k, local(p - normal * step));
                if ahead == behind {
                    return Err(GearError::invalid(
                        "a wheel flank's material side is ambiguous",
                    ));
                }
                let face = add_face(topo, sides, FaceSurface::Nurbs(surface), ahead)?;
                faces.push(face);
                roles.push((
                    face,
                    GearFace {
                        role: GearFaceRole::Flank { tooth: k, side },
                        half: Some(if half == 0 {
                            GearEnd::Start
                        } else {
                            GearEnd::End
                        }),
                    },
                ));
            }
            cap_edges[0].push(flank.cap_lines[0]);
            cap_edges[1].push(flank.cap_lines[1]);
        }
    }
    // Root lands between tooth k's plus flank and tooth k + 1's minus flank.
    for k in 0..wheel.teeth {
        let next = (k + 1) % wheel.teeth;
        let (plus, minus) = (&parts[k as usize][1], &parts[next as usize][0]);
        let mut arcs = [None, None];
        for (j, &s) in [0.0, width].iter().enumerate() {
            let lo = wheel.angle(k, ToothSide::Plus, wheel.root, s, j);
            let hi = wheel.angle(next, ToothSide::Minus, wheel.root, s, j)
                + if next == 0 { TAU } else { 0.0 };
            let (circle, trim) = arc(frame, s, wheel.root, lo, hi)?;
            let mut edge = Edge::with_tolerance(
                plus.root_vertices[2 * j],
                minus.root_vertices[2 * j],
                EdgeCurve::Circle(circle),
                Some(tolerance.max(fp)),
            );
            edge.set_trim(Some(trim));
            let id = topo.add_edge(edge);
            cap_edges[j].push(id);
            arcs[j] = Some(id);
        }
        let [Some(bottom), Some(top)] = arcs else {
            return Err(GearError::invalid("a root land has no arcs"));
        };
        let lo = wheel.angle(k, ToothSide::Plus, wheel.root, wheel.mid, 0);
        let hi = wheel.angle(next, ToothSide::Minus, wheel.root, wheel.mid, 0)
            + if next == 0 { TAU } else { 0.0 };
        let cylinder = CylindricalSurface::with_ref_dir(
            frame.origin,
            frame.axis,
            wheel.root,
            radial(frame, 0.5 * (lo + hi) - PI),
        )?;
        let face = add_face(
            topo,
            vec![
                OrientedEdge::new(bottom, true),
                OrientedEdge::new(minus.root_edges[0], true),
                OrientedEdge::new(minus.root_edges[1], true),
                OrientedEdge::new(top, false),
                OrientedEdge::new(plus.root_edges[1], false),
                OrientedEdge::new(plus.root_edges[0], false),
            ],
            FaceSurface::Cylinder(cylinder),
            false,
        )?;
        faces.push(face);
        roles.push((face, GearFaceRole::Root { gap: k }.into()));
    }
    // The bore and the caps.
    let mut bore_circles = [None, None];
    if let Some(diameter) = bore {
        let radius = 0.5 * diameter;
        let mut seam = Vec::with_capacity(2);
        for (j, &s) in [0.0, width].iter().enumerate() {
            let center = frame.origin + frame.axis * s;
            let vertex = topo.add_vertex(Vertex::new(center + frame.x * radius, tolerance.max(fp)));
            let circle = Circle3D::with_axes(center, frame.axis, radius, frame.x, frame.y)?;
            let mut edge = Edge::new(vertex, vertex, EdgeCurve::Circle(circle));
            edge.set_trim(Some((0.0, TAU)));
            bore_circles[j] = Some(topo.add_edge(edge));
            seam.push(vertex);
        }
        let line = topo.add_edge(Edge::new(seam[0], seam[1], EdgeCurve::Line));
        let [Some(low), Some(high)] = bore_circles else {
            return Err(GearError::invalid("the bore has no circles"));
        };
        let cylinder = CylindricalSurface::with_ref_dir(frame.origin, frame.axis, radius, frame.x)?;
        let face = add_face(
            topo,
            vec![
                OrientedEdge::new(low, true),
                OrientedEdge::new(line, true),
                OrientedEdge::new(high, false),
                OrientedEdge::new(line, false),
            ],
            FaceSurface::Cylinder(cylinder),
            true,
        )?;
        faces.push(face);
        roles.push((face, GearFaceRole::Bore.into()));
    }
    for (j, &s) in [0.0, width].iter().enumerate() {
        let outward = if j == 0 { -frame.axis } else { frame.axis };
        let outline = cap_loop(topo, &cap_edges[j], outward)?;
        let wire = topo.add_wire(Wire::new(outline, true)?);
        let mut inner = Vec::new();
        if let Some(circle) = bore_circles[j] {
            inner.push(topo.add_wire(Wire::new(vec![OrientedEdge::new(circle, j == 0)], true)?));
        }
        let center = frame.origin + frame.axis * s;
        let face = topo.add_face(Face::new(
            wire,
            inner,
            FaceSurface::Plane {
                normal: outward,
                d: outward.dot(Vec3::new(center.x(), center.y(), center.z())),
            },
        ));
        faces.push(face);
        let end = if j == 0 { GearEnd::Start } else { GearEnd::End };
        roles.push((face, GearFaceRole::Cap(end).into()));
    }
    let shell = topo.add_shell(Shell::new(faces)?);
    let solid = topo.add_solid(Solid::new(shell, vec![]));
    Ok((solid, roles, deviation))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic)]

    use super::*;

    /// The worm of the probe this module's model came from.
    fn probe_worm() -> EnvelopingWormParams {
        EnvelopingWormParams::standard(2.0, 1, 30, 10.0, 24.0)
    }

    #[test]
    fn the_hobbed_tooth_is_the_straight_tooth_in_the_mid_plane() {
        let hob = HobSweep::new(&probe_worm());
        for rho in [28.5, 30.0, 31.5] {
            for side in [ToothSide::Minus, ToothSide::Plus] {
                let HobEdge::At(edge) = hob.edge(side, 0.0, rho) else {
                    panic!("no edge at {rho}");
                };
                let line = hob.hindley().line_angle(side, rho);
                assert!(
                    (edge - line).abs() < 2e-5,
                    "{side:?} {rho}: {edge} vs {line}"
                );
            }
        }
    }

    #[test]
    fn the_hobbed_tooth_turns_and_narrows_off_the_mid_plane() {
        // A brute-force sweep (24001 wheel angles, 3601 polar angles) put the
        // edges at ρ = 30 and s = +3 at π − 0.04375 and π + 0.05940.
        let hob = HobSweep::new(&probe_worm());
        let at = |side, s| match hob.edge(side, s, 30.0) {
            HobEdge::At(edge) => edge - PI,
            other => panic!("{other:?}"),
        };
        let minus = at(ToothSide::Minus, 3.0);
        let plus = at(ToothSide::Plus, 3.0);
        assert!((minus + 0.04375).abs() < 1e-4, "{minus}");
        assert!((plus - 0.05940).abs() < 1e-4, "{plus}");
        // Mirror symmetry: (s, θ − π) → (−s, π − θ).
        let mirrored = at(ToothSide::Plus, -3.0);
        assert!((mirrored + minus).abs() < 1e-5, "{mirrored} vs {minus}");
    }
}
