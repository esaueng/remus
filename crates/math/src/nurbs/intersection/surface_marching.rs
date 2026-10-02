//! SSI marching: tracing intersection curves from seed points, including
//! branch detection, RKF45 adaptive stepping, and singular tangent analysis.

use crate::MathError;
use crate::context::OperationContext;
use crate::nurbs::surface::{DerivativeScratch, NurbsSurface};
use crate::vec::{Point3, Vec3};

use super::IntersectionPoint;
use super::surface_seeding::refine_ssi_point_with_context;

/// Per-surface derivative scratch pair threaded through SSI marching and
/// refinement.
///
/// [`DerivativeScratch`] span hints are knot indices that mean nothing on any
/// other knot vector, so each scratch must stay tied to one surface: surface
/// 1's halves only ever see `surface1`, surface 2's halves only `surface2`.
/// All state is overwritten per call; nothing is read before it is written.
pub(super) struct SsiScratch {
    /// Derivative-solve buffers tied to surface 1's knot vectors.
    scratch1: DerivativeScratch,
    /// Derivative-solve buffers tied to surface 2's knot vectors.
    scratch2: DerivativeScratch,
}

impl SsiScratch {
    /// Empty scratch; buffers grow on first use.
    pub(super) fn new() -> Self {
        Self {
            scratch1: DerivativeScratch::new(),
            scratch2: DerivativeScratch::new(),
        }
    }

    /// `surface1.normal(u, v)` on the reusable buffers; bit-identical.
    pub(super) fn normal1(&mut self, s1: &NurbsSurface, u: f64, v: f64) -> Result<Vec3, MathError> {
        self.scratch1.normal_from(s1, u, v)
    }

    /// `surface2.normal(u, v)` on the reusable buffers; bit-identical.
    pub(super) fn normal2(&mut self, s2: &NurbsSurface, u: f64, v: f64) -> Result<Vec3, MathError> {
        self.scratch2.normal_from(s2, u, v)
    }

    /// `surface1`'s first partials on the reusable buffers; bit-identical to
    /// `(partial_u, partial_v)`.
    pub(super) fn partials1(&mut self, s1: &NurbsSurface, u: f64, v: f64) -> (Vec3, Vec3) {
        self.scratch1.partials_from(s1, u, v)
    }

    /// `surface2`'s first partials on the reusable buffers; bit-identical to
    /// `(partial_u, partial_v)`.
    pub(super) fn partials2(&mut self, s2: &NurbsSurface, u: f64, v: f64) -> (Vec3, Vec3) {
        self.scratch2.partials_from(s2, u, v)
    }

    /// `surface1`'s normal plus both first partials in one base solve;
    /// bit-identical to `(normal, partial_u, partial_v)`.
    pub(super) fn normal_partials1(
        &mut self,
        s1: &NurbsSurface,
        u: f64,
        v: f64,
    ) -> (Result<Vec3, MathError>, Vec3, Vec3) {
        self.scratch1.normal_partials_from(s1, u, v)
    }

    /// `surface2`'s normal plus both first partials in one base solve;
    /// bit-identical to `(normal, partial_u, partial_v)`.
    pub(super) fn normal_partials2(
        &mut self,
        s2: &NurbsSurface,
        u: f64,
        v: f64,
    ) -> (Result<Vec3, MathError>, Vec3, Vec3) {
        self.scratch2.normal_partials_from(s2, u, v)
    }

    /// The reusable derivative-solve scratch for surface 1.
    pub(super) fn solve1(&mut self) -> &mut DerivativeScratch {
        &mut self.scratch1
    }

    /// The reusable derivative-solve scratch for surface 2.
    pub(super) fn solve2(&mut self) -> &mut DerivativeScratch {
        &mut self.scratch2
    }
}

/// Second-order branch geometry in the shared tangent plane (B66).
///
/// At a tangential contact the two surfaces share a tangent plane with unit
/// normal `n`. Writing each surface as a height graph over orthonormal
/// coordinates `(alpha, beta)` of that plane gives, to second order,
/// `h_k(alpha, beta) = 1/2 [alpha beta] Q_k [alpha; beta]`, where `Q_k` is the
/// second fundamental form expressed in the orthonormal frame. The
/// intersection satisfies `h_1 = h_2` to second order, i.e. the quadratic
///
/// ```text
/// [alpha beta] Q [alpha; beta] = 0,   Q = Q_1 - s * Q_2,
/// ```
///
/// with `s = sign(n1 . n2)` so the two heights are measured along the same
/// normal. `det(Q) < 0` (indefinite) is a transverse crossing with two null
/// (asymptotic) directions; `det(Q) > 0` (definite) is an isolated touch with
/// none; `det(Q) = 0` with `Q != 0` is a single ruling (contact line);
/// `Q = 0` is osculating/overlap and carries no direction.
///
/// Per-surface orthonormal form: with first fundamental `I_k`, second
/// fundamental `II_k`, and `B_k = [[Su.e1, Su.e2],[Sv.e1, Sv.e2]]`,
/// `C_k = I_k^{-1} B_k` maps `(alpha, beta)` to `(du, dv)`, and
/// `Q_k = C_k^T II_k C_k`.
///
/// Under a parameter rescaling `(u, v) -> (a*u, b*v)` with
/// `D = diag(a, b)`, `I -> D I D`, `II -> D II D`, `B -> D B`, so
/// `C = I^{-1} B -> D^{-1} C` and `Q = C^T II C` is unchanged. The same
/// cancellation holds for any invertible linear reparameterization
/// (`D` general 2x2) and for swapped/reversed parameters (orthogonal change
/// of basis, possibly flipping the sign of `Q`, which preserves its null
/// cone and `det` since `det(-Q) = det(Q)` in 2D). Rigid placements rotate
/// `e1, e2` with the surfaces, leaving eigenvalues and `det` unchanged.
/// Raw `II_1 - II_2` in parameter coordinates has none of these invariances:
/// its entries scale as model-units-per-parameter^2, which is why the old
/// absolute `|lambda| < 0.1` gate moved with knot domains.
///
/// All thresholds below are dimensionless roundoff guards, never geometry
/// gates: `REL = 1e-12` on `|det| / ||Q||^2` separates indefinite from
/// parabolic within double-precision derivative error, and the `1e-24`
/// metric guard separates a degenerate first fundamental form
/// (`sin(theta) <= 2e-12`) from a regular one. No absolute curvature or
/// angle threshold is introduced.
const BRANCH_REL: f64 = 1e-12;

/// Orthonormal curvature difference at a tangential contact.
struct OrthoDiff {
    a: f64,
    b: f64,
    d: f64,
    c1: [f64; 4],
    c2: [f64; 4],
    e1: Vec3,
    e2: Vec3,
    norm: f64,
    det: f64,
}

/// Orthonormal curvature difference `Q = [[a, b], [b, d]]` at a tangential
/// contact, with the frame and parameter maps needed for seeding.
///
/// Returns `None` when either surface is degenerate there, the normals do not
/// share a plane (`|n1 . n2| < 0.99`), or second-order tables are unavailable:
/// all explicitly unsupported, never an arbitrary direction.
#[allow(clippy::too_many_lines, clippy::similar_names)]
fn orthonormal_difference(
    s1: &NurbsSurface,
    s2: &NurbsSurface,
    u1: f64,
    v1: f64,
    u2: f64,
    v2: f64,
    scratch: &mut SsiScratch,
) -> Option<OrthoDiff> {
    let mut table1: Vec<Vec<Vec3>> = Vec::new();
    let mut table2: Vec<Vec<Vec3>> = Vec::new();
    scratch
        .solve1()
        .derivative_table_from(s1, u1, v1, 2, &mut table1);
    scratch
        .solve2()
        .derivative_table_from(s2, u2, v2, 2, &mut table2);
    let d1: &[Vec<Vec3>] = &table1;
    let d2: &[Vec<Vec3>] = &table2;
    if d1.len() < 3 || d1[0].len() < 3 || d2.len() < 3 || d2[0].len() < 3 {
        return None;
    }
    let s1u = d1[1][0];
    let s1v = d1[0][1];
    let s2u = d2[1][0];
    let s2v = d2[0][1];

    let n1_raw = s1u.cross(s1v);
    let n2_raw = s2u.cross(s2v);
    let n1_lensq = n1_raw.length_squared();
    let n2_lensq = n2_raw.length_squared();
    let s1u_lensq = s1u.length_squared();
    let s1v_lensq = s1v.length_squared();
    let s2u_lensq = s2u.length_squared();
    let s2v_lensq = s2v.length_squared();
    // Scale-invariant degeneracy: sin(theta) = |Su x Sv| / (|Su| |Sv|).
    let sin1_sq = if s1u_lensq > 0.0 && s1v_lensq > 0.0 {
        n1_lensq / (s1u_lensq * s1v_lensq)
    } else {
        0.0
    };
    let sin2_sq = if s2u_lensq > 0.0 && s2v_lensq > 0.0 {
        n2_lensq / (s2u_lensq * s2v_lensq)
    } else {
        0.0
    };
    // Scale-invariant degeneracy: sin(theta) at/below roundoff (or non-finite
    // lens) means a collapsed partial: explicitly unsupported.
    if !sin1_sq.is_finite() || !sin2_sq.is_finite() || sin1_sq <= 1e-24 || sin2_sq <= 1e-24 {
        return None;
    }
    let (Ok(n1), Ok(n2)) = (n1_raw.normalize(), n2_raw.normalize()) else {
        return None;
    };
    let cos = n1.dot(n2);
    if cos.abs() < 0.99 {
        // Tangent planes differ: the shared-plane height model does not apply.
        return None;
    }
    let s = if cos >= 0.0 { 1.0 } else { -1.0 };

    // Deterministic orthonormal frame from the normal alone (smallest-component
    // reference), so the frame — and hence Q up to an orthogonal change of
    // basis — is independent of either parameterization.
    let ax = n1.x().abs();
    let ay = n1.y().abs();
    let az = n1.z().abs();
    let reference = if ax <= ay && ax <= az {
        Vec3::new(1.0, 0.0, 0.0)
    } else if ay <= az {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        Vec3::new(0.0, 0.0, 1.0)
    };
    let e1 = (reference - n1 * reference.dot(n1)).normalize().ok()?;
    let e2 = n1.cross(e1).normalize().ok()?;

    // Per-surface orthonormal forms Q_k = C_k^T II_k C_k.
    let quadric_form = |su: Vec3,
                        sv: Vec3,
                        suu: Vec3,
                        suv: Vec3,
                        svv: Vec3,
                        n: Vec3|
     -> Option<([f64; 3], [f64; 4])> {
        let e = su.dot(su);
        let f = su.dot(sv);
        let g = sv.dot(sv);
        let det_i = e * g - f * f;
        // Relative guard: det(I) / (E+G)^2 ~ sin^2/4; at/below 1e-24 (or
        // non-finite) the parameterization is degenerate there.
        let scale = (e + g) * (e + g);
        if !det_i.is_finite() || !scale.is_finite() || det_i <= 1e-24 * scale {
            return None;
        }
        let l = suu.dot(n);
        let m = suv.dot(n);
        let nn = svv.dot(n);
        // B rows: Su.e, Sv.e.
        let b00 = su.dot(e1);
        let b01 = su.dot(e2);
        let b10 = sv.dot(e1);
        let b11 = sv.dot(e2);
        // C = I^{-1} B.
        let inv00 = g / det_i;
        let inv01 = -f / det_i;
        let inv11 = e / det_i;
        let c00 = inv00 * b00 + inv01 * b10;
        let c01 = inv00 * b01 + inv01 * b11;
        let c10 = inv01 * b00 + inv11 * b10;
        let c11 = inv01 * b01 + inv11 * b11;
        // Q = C^T II C with II = [[l, m],[m, nn]].
        // First M = II * C.
        let m00 = l * c00 + m * c10;
        let m01 = l * c01 + m * c11;
        let m10 = m * c00 + nn * c10;
        let m11 = m * c01 + nn * c11;
        let q00 = c00 * m00 + c10 * m10;
        let q01 = c00 * m01 + c10 * m11;
        let q11 = c01 * m01 + c11 * m11;
        Some(([q00, q01, q11], [c00, c01, c10, c11]))
    };

    let (q1, c1) = quadric_form(s1u, s1v, d1[2][0], d1[1][1], d1[0][2], n1)?;
    // Measure surface 2's heights along n1: II_2(n1) = s * II_2(n2).
    let (q2_raw, c2) = quadric_form(s2u, s2v, d2[2][0], d2[1][1], d2[0][2], n2)?;
    let (q2_0, q2_1, q2_2) = (s * q2_raw[0], s * q2_raw[1], s * q2_raw[2]);

    let a = q1[0] - q2_0;
    let b = q1[1] - q2_1;
    let d = q1[2] - q2_2;
    let norm = (a * a + 2.0 * b * b + d * d).sqrt();
    if !norm.is_finite() {
        return None;
    }
    let det = a * d - b * b;
    Some(OrthoDiff {
        a,
        b,
        d,
        c1,
        c2,
        e1,
        e2,
        norm,
        det,
    })
}

/// Null (asymptotic) directions of `Q = [[a, b], [b, d]]` as unit 3D vectors.
///
/// Solves `a alpha^2 + 2 b alpha beta + d beta^2 = 0` in the `(e1, e2)` frame:
/// two directions when `det < -REL * norm^2` (transverse crossing), one
/// (the zero eigenvector) when `|det| <= REL * norm^2` with `Q != 0`
/// (parabolic ruling), none when `det > REL * norm^2` (isolated touch) or
/// `Q = 0` (osculating/overlap). The last two are explicit `None`: ambiguity
/// keeps the existing failure contract instead of an arbitrary pick.
fn null_directions(a: f64, b: f64, d: f64, e1: Vec3, e2: Vec3, norm: f64, det: f64) -> Vec<Vec3> {
    if !norm.is_finite() || !det.is_finite() || norm <= 0.0 {
        return Vec::new();
    }
    let guard = BRANCH_REL * norm * norm;
    if det < -guard {
        // Indefinite: two distinct null directions from the quadratic.
        // Solve stably: if |a| >= |d|, solve for r = beta/alpha, else for
        // r = alpha/beta, so the leading coefficient is the larger diagonal.
        let mut dirs = Vec::with_capacity(2);
        if a.abs() >= d.abs() {
            // a r^2? No: a + 2b r + d r^2 = 0 with r = beta/alpha.
            let disc = b * b - a * d;
            if disc <= 0.0 {
                return Vec::new();
            }
            let sq = disc.sqrt();
            for num in [(-b + sq), (-b - sq)] {
                let den = d;
                let (alpha, beta) = if den.abs() > 1e-300 {
                    (den, num)
                } else {
                    // d = 0 (e.g. saddle [[0, c],[c, 0]]): roots are
                    // alpha = 0 and beta = 0 via the swapped solve below;
                    // this arm only runs when |a| >= |d| = 0, i.e. a != 0.
                    (1.0, 0.0)
                };
                let t = (e1 * alpha + e2 * beta).normalize();
                if let Ok(t) = t {
                    dirs.push(t);
                }
            }
            // Handle the d = 0 sub-case explicitly: a alpha^2 + 2b alpha beta
            // = alpha (a alpha + 2b beta) = 0 gives alpha = 0 and
            // a alpha + 2b beta = 0. The generic formula above divides by d.
            // Triggers on d ~= 0 with b != 0 (including the pure saddle
            // a = d = 0, whose nulls are the two axes).
            if d.abs() <= 1e-300 && b.abs() > 1e-300 {
                dirs.clear();
                if let Ok(t1) = (e1 * 0.0 + e2 * 1.0).normalize() {
                    dirs.push(t1);
                }
                // alpha = -2b, beta = a (from a alpha + 2b beta = 0 with
                // beta = a): direction (-2b, a); for a = 0 this is the
                // alpha axis.
                if let Ok(t2) = (e1 * (-2.0 * b) + e2 * a).normalize() {
                    dirs.push(t2);
                }
            }
        } else {
            // d + 2b r + a r^2 = 0 with r = alpha/beta.
            let disc = b * b - a * d;
            if disc <= 0.0 {
                return Vec::new();
            }
            let sq = disc.sqrt();
            for num in [(-b + sq), (-b - sq)] {
                let den = a;
                let (alpha, beta) = if den.abs() > 1e-300 {
                    (num, den)
                } else {
                    (0.0, 1.0)
                };
                let t = (e1 * alpha + e2 * beta).normalize();
                if let Ok(t) = t {
                    dirs.push(t);
                }
            }
            // Symmetric a = 0 sub-case: beta (d beta + 2b alpha) = 0 gives
            // beta = 0 and d beta + 2b alpha = 0 (including a = d = 0).
            if a.abs() <= 1e-300 && b.abs() > 1e-300 {
                dirs.clear();
                if let Ok(t1) = (e1 * 1.0 + e2 * 0.0).normalize() {
                    dirs.push(t1);
                }
                if let Ok(t2) = (e1 * d + e2 * (-2.0 * b)).normalize() {
                    dirs.push(t2);
                }
            }
        }
        // Deterministic order: sort by (x, y, z) so swapped parameters and
        // re-runs agree; both nulls are valid intersection directions, the
        // caller picks continuity, this order only breaks ties.
        dirs.sort_by(|p, q| {
            p.x()
                .total_cmp(&q.x())
                .then(p.y().total_cmp(&q.y()))
                .then(p.z().total_cmp(&q.z()))
        });
        dirs.dedup_by(|p, q| (*p - *q).length() < 1e-12 || (*p + *q).length() < 1e-12);
        dirs
    } else if det > guard {
        // Definite: isolated touch, no real null direction.
        Vec::new()
    } else {
        // Parabolic within roundoff: single ruling = zero eigenvector of Q.
        if norm <= 0.0 {
            return Vec::new();
        }
        let trace = a + d;
        // Zero eigenvalue's eigenvector: (b, -a) or (d, -b) whichever larger.
        let (alpha, beta) = if b.abs() >= a.abs() && b.abs() >= d.abs() {
            // Q ~ [[a, b],[b, d]] with det 0: null is (d, -b) or (-b, a)?
            // Use (-b, a) unless a is tiny, else (d, -b).
            if a.abs() >= d.abs() { (-b, a) } else { (d, -b) }
        } else if a.abs() >= d.abs() {
            (-b, a)
        } else {
            (d, -b)
        };
        if alpha == 0.0 && beta == 0.0 {
            return Vec::new();
        }
        // Silence unused warning for trace in this arm (kept for symmetry
        // with the eigenvalue derivation in the PR rationale).
        let _ = trace;
        match (e1 * alpha + e2 * beta).normalize() {
            Ok(t) => vec![t],
            Err(_) => Vec::new(),
        }
    }
}

/// March along an intersection curve, detecting branch points.
///
/// Returns the traced points and any branch seed points found during
/// marching. Branch points are detected when the surface normals become
/// near-parallel (`|n1 x n2|` drops below threshold), confirmed via
/// second-order curvature analysis. At confirmed branch points, new
/// seeds are spawned in divergent directions.
pub(super) fn march_with_branches(
    s1: &NurbsSurface,
    s2: &NurbsSurface,
    seed: &IntersectionPoint,
    step_size: f64,
    tolerance: f64,
    context: &OperationContext,
    scratch: &mut SsiScratch,
) -> Result<(Vec<IntersectionPoint>, Vec<IntersectionPoint>), MathError> {
    context.check_cancelled()?;
    let mut branch_seeds: Vec<IntersectionPoint> = Vec::new();

    // March forward, collecting branch points.
    let (forward, fwd_branches) =
        march_direction_with_branches(s1, s2, seed, true, step_size, tolerance, context, scratch)?;
    branch_seeds.extend(fwd_branches);

    // March backward, collecting branch points.
    let (backward, bwd_branches) =
        march_direction_with_branches(s1, s2, seed, false, step_size, tolerance, context, scratch)?;
    branch_seeds.extend(bwd_branches);

    // Combine: backward (reversed) + seed + forward.
    let mut result: Vec<IntersectionPoint> = backward.into_iter().rev().collect();
    result.push(*seed);
    result.extend(forward);

    Ok((result, branch_seeds))
}

/// Detect the transverse branch at a confirmed crossing.
///
/// At a point where `|n1 x n2|` is small, the orthonormal difference `Q` is
/// indefinite with two null directions: one continues the incoming branch and
/// the other is the transverse branch. The transverse null, least aligned
/// with `current_tangent`, is stepped in both orientations and Newton
/// refined; seeds that refine onto the intersection, clear of the point, and
/// more than 30 deg from the incoming direction are returned. Isolated
/// touches, rulings, and osculating overlaps yield no seeds here, so ambiguity
/// keeps the existing failure contract instead of an arbitrary pick.
#[allow(clippy::too_many_arguments)]
fn find_branch_directions(
    s1: &NurbsSurface,
    s2: &NurbsSurface,
    point: &IntersectionPoint,
    current_tangent: Vec3,
    step_size: f64,
    tolerance: f64,
    context: &OperationContext,
    scratch: &mut SsiScratch,
) -> Result<Vec<IntersectionPoint>, MathError> {
    let min_branch_angle = 30.0_f64.to_radians();
    let (u1, v1) = point.param1;
    let (u2, v2) = point.param2;

    let Some(diff) = orthonormal_difference(s1, s2, u1, v1, u2, v2, scratch) else {
        return Ok(Vec::new());
    };
    if !diff.det.is_finite()
        || !diff.norm.is_finite()
        || diff.det >= -BRANCH_REL * diff.norm * diff.norm
    {
        return Ok(Vec::new());
    }
    let nulls = null_directions(
        diff.a, diff.b, diff.d, diff.e1, diff.e2, diff.norm, diff.det,
    );
    if nulls.len() < 2 {
        return Ok(Vec::new());
    }
    // Incoming-orthogonal transverse: least |dot| with the march direction.
    // Tie (bisector arrival) spawns both nulls so neither arm is missed.
    let dot0 = nulls[0].dot(current_tangent).abs();
    let dot1 = nulls[1].dot(current_tangent).abs();
    let transverse: Vec<(Vec3, (f64, f64))> = if (dot0 - dot1).abs() <= 1e-12 {
        // Bisector arrival: both nulls are transverse candidates. Recover
        // their orthonormal (alpha, beta) by projection onto (e1, e2).
        vec![
            (nulls[0], (nulls[0].dot(diff.e1), nulls[0].dot(diff.e2))),
            (nulls[1], (nulls[1].dot(diff.e1), nulls[1].dot(diff.e2))),
        ]
    } else if dot0 < dot1 {
        vec![(nulls[0], (nulls[0].dot(diff.e1), nulls[0].dot(diff.e2)))]
    } else {
        vec![(nulls[1], (nulls[1].dot(diff.e1), nulls[1].dot(diff.e2)))]
    };

    let c1 = diff.c1;
    let c2 = diff.c2;

    let mut branch_seeds = Vec::new();
    for (t, (alpha, beta)) in transverse {
        context.check_cancelled()?;
        // Normalize the orthonormal coords so the larger param component on
        // either surface spans step_size * 0.5: a small offset that clears
        // the incoming branch's Newton basin yet stays in-patch.
        let du1 = c1[0] * alpha + c1[1] * beta;
        let dv1 = c1[2] * alpha + c1[3] * beta;
        let du2 = c2[0] * alpha + c2[1] * beta;
        let dv2 = c2[2] * alpha + c2[3] * beta;
        let peak = du1.abs().max(dv1.abs()).max(du2.abs()).max(dv2.abs());
        if !peak.is_finite() || peak <= 0.0 {
            continue;
        }
        let scale = step_size * 0.5 / peak;
        for side in [1.0, -1.0] {
            context.check_cancelled()?;
            let u1p = u1 + side * scale * (c1[0] * alpha + c1[1] * beta);
            let v1p = v1 + side * scale * (c1[2] * alpha + c1[3] * beta);
            let u2p = u2 + side * scale * (c2[0] * alpha + c2[1] * beta);
            let v2p = v2 + side * scale * (c2[2] * alpha + c2[3] * beta);
            if branch_seeds.len() >= 4 {
                break;
            }
            if let Some(refined) = refine_ssi_point_with_context(
                s1, s2, u1p, v1p, u2p, v2p, tolerance, context, scratch,
            )? {
                let dvec = refined.point - point.point;
                let dist = dvec.length();
                if dist < tolerance {
                    continue;
                }
                if let Ok(dir) = dvec.normalize() {
                    let cos_angle = dir.dot(current_tangent).abs().clamp(-1.0, 1.0);
                    if cos_angle.acos() > min_branch_angle {
                        // Deduplicate seeds on the same arm.
                        let dup = branch_seeds.iter().any(|s: &IntersectionPoint| {
                            (s.point - refined.point).length() < tolerance * 100.0
                        });
                        if !dup {
                            // Keep the 3D direction for the unused-binding lint.
                            let _ = t;
                            branch_seeds.push(refined);
                        }
                    }
                }
            }
        }
    }

    Ok(branch_seeds)
}

/// March in one direction, detecting branch points where `|n1 x n2|`
/// drops below threshold. Returns traced points and branch seed points.
#[allow(
    clippy::too_many_lines,
    clippy::many_single_char_names,
    clippy::too_many_arguments
)]
fn march_direction_with_branches(
    s1: &NurbsSurface,
    s2: &NurbsSurface,
    seed: &IntersectionPoint,
    forward: bool,
    step_size: f64,
    tolerance: f64,
    context: &OperationContext,
    scratch: &mut SsiScratch,
) -> Result<(Vec<IntersectionPoint>, Vec<IntersectionPoint>), MathError> {
    let traced = march_direction(
        s1,
        s2,
        seed,
        forward,
        step_size,
        tolerance,
        context.budgets.march_steps,
        context,
        scratch,
    )?;
    let mut branch_seeds: Vec<IntersectionPoint> = Vec::new();

    // Post-process: scan traced points for near-tangential locations.
    // `tolerance * 1000` is a weld-scale band on the unit-normal sine (1000x
    // the SSI residual contract), unchanged: it only selects candidates, the
    // confirmation below is the scale-invariant sign of det(Q).
    let branch_threshold = tolerance * 1000.0;

    // First pass: candidate cross magnitudes. A crossing valley contributes
    // several adjacent near-tangential points (the exact crossing plus RKF45
    // micro-steps around it); only the local minimum per valley spawns
    // transverse seeds, so off-crossing neighbors cannot emit off-line seeds
    // or duplicate branches.
    let mut cross_mags: Vec<Option<f64>> = Vec::with_capacity(traced.len());
    for pt in &traced {
        context.check_cancelled()?;
        let Ok(n1) = scratch.normal1(s1, pt.param1.0, pt.param1.1) else {
            cross_mags.push(None);
            continue;
        };
        let Ok(n2) = scratch.normal2(s2, pt.param2.0, pt.param2.1) else {
            cross_mags.push(None);
            continue;
        };
        cross_mags.push(Some(n1.cross(n2).length()));
    }

    for (idx, pt) in traced.iter().enumerate() {
        context.check_cancelled()?;
        if branch_seeds.len() >= context.budgets.branches_per_direction {
            break;
        }

        let Some(cross_mag) = cross_mags[idx] else {
            continue;
        };
        if cross_mag >= branch_threshold {
            continue; // Not near-tangential, no branch point
        }
        // Local-minimum per valley: skip shoulders of the same crossing so
        // only the closest point seeds the transverse branch.
        let dominated_by_prev = idx > 0 && cross_mags[idx - 1].is_some_and(|prev| prev < cross_mag);
        let dominated_by_next =
            idx + 1 < cross_mags.len() && cross_mags[idx + 1].is_some_and(|next| next < cross_mag);
        if dominated_by_prev || dominated_by_next {
            continue;
        }

        let n1 = match scratch.normal1(s1, pt.param1.0, pt.param1.1) {
            Ok(n) => n,
            Err(_) => continue,
        };
        let n2 = match scratch.normal2(s2, pt.param2.0, pt.param2.1) {
            Ok(n) => n,
            Err(_) => continue,
        };

        // Near-tangential point found. Confirm a transverse crossing via the
        // orthonormal difference: indefinite Q (det < -REL * ||Q||^2) has two
        // null directions; definite/parabolic/osculating do not branch here.
        let has_branch = orthonormal_difference(
            s1,
            s2,
            pt.param1.0,
            pt.param1.1,
            pt.param2.0,
            pt.param2.1,
            scratch,
        )
        .is_some_and(|diff| {
            diff.det.is_finite()
                && diff.norm.is_finite()
                && diff.det < -BRANCH_REL * diff.norm * diff.norm
        });

        if !has_branch {
            continue;
        }

        // Confirmed branch point: incoming direction from the traced polyline
        // (central difference) so an exactly-crossing point — where n1 x n2
        // vanishes and carries no direction — still preserves continuity.
        // Falls back to n1 x n2 with the march sign, then to +x.
        let sign = if forward { 1.0 } else { -1.0 };
        let poly_tangent = if traced.len() >= 2 {
            let prev = if idx > 0 {
                traced[idx - 1].point
            } else {
                pt.point
            };
            let next = if idx + 1 < traced.len() {
                traced[idx + 1].point
            } else {
                pt.point
            };
            (next - prev).normalize().ok()
        } else {
            None
        };
        let current_tangent = poly_tangent
            .or_else(|| {
                let t = n1.cross(n2);
                t.normalize()
                    .ok()
                    .map(|t| Vec3::new(t.x() * sign, t.y() * sign, t.z() * sign))
            })
            .unwrap_or(Vec3::new(1.0, 0.0, 0.0));

        let new_seeds = find_branch_directions(
            s1,
            s2,
            pt,
            current_tangent,
            step_size,
            tolerance,
            context,
            scratch,
        )?;
        branch_seeds.extend(new_seeds);
    }

    Ok((traced, branch_seeds))
}

/// March along an intersection curve from a seed point.
///
/// Uses the tangent direction (cross product of surface normals) to step
/// forward, then corrects back to the intersection with Newton.
/// The `step_size` is the *initial* step size. The marcher adapts it based
/// on both RKF45 integration error and geometric curvature (angular
/// deviation between successive tangent vectors). This ensures fine
/// resolution on high-curvature portions and efficient large steps on
/// flat portions.
#[cfg(test)]
pub(super) fn march_intersection(
    s1: &NurbsSurface,
    s2: &NurbsSurface,
    seed: &IntersectionPoint,
    step_size: f64,
    tolerance: f64,
) -> Vec<IntersectionPoint> {
    let max_steps = 200;
    let context = OperationContext::new();
    let mut scratch = SsiScratch::new();

    // March forward.
    let forward = march_direction(
        s1,
        s2,
        seed,
        true,
        step_size,
        tolerance,
        max_steps,
        &context,
        &mut scratch,
    )
    .unwrap_or_default();
    // March backward.
    let backward = march_direction(
        s1,
        s2,
        seed,
        false,
        step_size,
        tolerance,
        max_steps,
        &context,
        &mut scratch,
    )
    .unwrap_or_default();

    // Combine: backward (reversed) + seed + forward.
    let mut result: Vec<IntersectionPoint> = backward.into_iter().rev().collect();
    result.push(*seed);
    result.extend(forward);

    result
}

/// Compute the SSI tangent in parameter space at the given parameters.
/// Returns `(du1, dv1, du2, dv2)` or `None` if normals are degenerate.
///
/// `prev` is the incoming 3D march direction (with `sign` already applied):
/// at a singular point with two null directions, the null most aligned with
/// `prev` is chosen so marching preserves branch continuity instead of
/// turning onto the transverse branch or the old bisector.
#[allow(clippy::similar_names, clippy::too_many_arguments)]
fn ssi_tangent_params(
    s1: &NurbsSurface,
    s2: &NurbsSurface,
    u1: f64,
    v1: f64,
    u2: f64,
    v2: f64,
    sign: f64,
    prev: Option<Vec3>,
    context: &OperationContext,
    scratch: &mut SsiScratch,
) -> Result<Option<[f64; 4]>, MathError> {
    context.check_cancelled()?;
    // One base solve per surface serves the normal and the tangent
    // projection; both match the separate `normal`/`derivatives` calls
    // bit-for-bit.
    let (n1, su1, sv1) = scratch.normal_partials1(s1, u1, v1);
    let (n2, su2, sv2) = scratch.normal_partials2(s2, u2, v2);
    let (Ok(n1), Ok(n2)) = (n1, n2) else {
        return Ok(None);
    };

    let tangent_raw = n1.cross(n2);
    let tangent = if let Ok(t) = tangent_raw.normalize() {
        t
    } else {
        // Tangential intersection (normals parallel/antiparallel).
        // Second-order null most aligned with the incoming direction keeps
        // branch continuity; without `prev` the deterministic first null is
        // still an exact intersection direction (never the old bisector).
        let pt = IntersectionPoint {
            point: s1.evaluate(u1, v1),
            param1: (u1, v1),
            param2: (u2, v2),
        };
        // Un-apply `sign` for the continuity comparison: `prev` already
        // carries it, the singular choice is oriented below.
        let incoming = prev.map(|p| Vec3::new(p.x() * sign, p.y() * sign, p.z() * sign));
        let Some(tangent) = singular_tangent_direction(s1, s2, &pt, incoming, context, scratch)?
        else {
            return Ok(None);
        };
        tangent
    };

    let t = Vec3::new(tangent.x() * sign, tangent.y() * sign, tangent.z() * sign);

    let (du1, dv1) = project_tangent_to_params(su1, sv1, t, 1.0);
    let (du2, dv2) = project_tangent_to_params(su2, sv2, t, 1.0);

    // Normalize so the maximum component magnitude is 1.0.
    let max_comp = du1.abs().max(dv1.abs()).max(du2.abs()).max(dv2.abs());
    if max_comp < 1e-20 {
        return Ok(None);
    }

    Ok(Some([
        du1 / max_comp,
        dv1 / max_comp,
        du2 / max_comp,
        dv2 / max_comp,
    ]))
}

/// At a singular point (where surface normals are parallel/antiparallel),
/// determine the intersection curve direction from second-order nulls.
///
/// At a tangential point the first-order tangent `n1 x n2` vanishes. The
/// orthonormal difference `Q` supplies the exact asymptotic directions: two
/// nulls at a transverse crossing, one ruling on a contact line, none at an
/// isolated touch or osculating overlap. With two nulls, `incoming` (the
/// march direction without the RKF45 sign applied) selects the most aligned
/// oriented candidate (`max dot`, both signs considered) so marching
/// continues its branch; without it the deterministic first null is returned
/// (still exact, never the bisector).
///
/// Falls back to perturbation-based search if second-order analysis
/// is degenerate (e.g., surfaces are osculating to second order).
#[allow(clippy::similar_names, clippy::too_many_lines)]
fn singular_tangent_direction(
    s1: &NurbsSurface,
    s2: &NurbsSurface,
    point: &IntersectionPoint,
    incoming: Option<Vec3>,
    context: &OperationContext,
    scratch: &mut SsiScratch,
) -> Result<Option<Vec3>, MathError> {
    let (u1, v1) = point.param1;
    let (u2, v2) = point.param2;

    // Try second-order analysis first.
    if let Some(diff) = orthonormal_difference(s1, s2, u1, v1, u2, v2, scratch) {
        let nulls = null_directions(
            diff.a, diff.b, diff.d, diff.e1, diff.e2, diff.norm, diff.det,
        );
        if !nulls.is_empty() {
            if let Some(prev) = incoming {
                // Oriented continuity: consider both signs of each null.
                let mut best: Option<Vec3> = None;
                let mut best_dot = f64::NEG_INFINITY;
                for t in &nulls {
                    context.check_cancelled()?;
                    for cand in [*t, Vec3::new(-t.x(), -t.y(), -t.z())] {
                        let d = cand.dot(prev);
                        if d > best_dot {
                            best_dot = d;
                            best = Some(cand);
                        }
                    }
                }
                if let Some(choice) = best {
                    return Ok(Some(choice));
                }
            }
            // Deterministic single: first null in sorted order. At a
            // transverse crossing either null is an exact branch direction.
            if let Some(first) = nulls.into_iter().next() {
                return Ok(Some(first));
            }
        }
    } else if let Some(dir) = second_order_tangent(s1, s2, u1, v1, u2, v2, scratch) {
        return Ok(Some(dir));
    }

    // Fallback: perturbation-based search (original method).
    perturbation_tangent(s1, s2, point, context, scratch)
}

/// Second-order curvature analysis for tangential intersection direction.
///
/// Computes the orthonormal curvature difference `Q` of the two surfaces at
/// the touch point (see [`orthonormal_difference`]). At a transverse crossing
/// (`det(Q) < 0`) the intersection follows `Q`'s null (asymptotic) directions
/// — the two lines `x = 0`, `y = 0` for the plane/saddle witness — never the
/// old eigenvector-of-smallest-eigenvalue bisector. This returns the
/// deterministic first null (sorted order); callers with a march direction
/// use [`singular_tangent_direction`] for the continuity-aware choice among
/// the two. A single ruling is returned for the parabolic contact-line case;
/// isolated touches and osculating/overlap (`Q = 0`) return `None` under the
/// existing failure contract.
#[allow(clippy::similar_names)]
pub(super) fn second_order_tangent(
    s1: &NurbsSurface,
    s2: &NurbsSurface,
    u1: f64,
    v1: f64,
    u2: f64,
    v2: f64,
    scratch: &mut SsiScratch,
) -> Option<Vec3> {
    let diff = orthonormal_difference(s1, s2, u1, v1, u2, v2, scratch)?;
    let nulls = null_directions(
        diff.a, diff.b, diff.d, diff.e1, diff.e2, diff.norm, diff.det,
    );
    nulls.into_iter().next()
}

/// Perturbation-based tangent direction finder (fallback).
///
/// Samples 8 directions around the current point in parameter space, attempts
/// Newton refinement at each, and returns the direction to the most distant
/// successfully refined point.
fn perturbation_tangent(
    s1: &NurbsSurface,
    s2: &NurbsSurface,
    point: &IntersectionPoint,
    context: &OperationContext,
    scratch: &mut SsiScratch,
) -> Result<Option<Vec3>, MathError> {
    let eps = 1e-4;
    let (u1, v1) = point.param1;
    let (u2, v2) = point.param2;

    let directions: [(f64, f64); 8] = [
        (eps, 0.0),
        (-eps, 0.0),
        (0.0, eps),
        (0.0, -eps),
        (eps, eps),
        (eps, -eps),
        (-eps, eps),
        (-eps, -eps),
    ];

    let mut best_dir: Option<Vec3> = None;
    let mut best_dist = 0.0_f64;

    for &(du, dv) in &directions {
        context.check_cancelled()?;
        let u1p = (u1 + du).clamp(0.001, 0.999);
        let v1p = (v1 + dv).clamp(0.001, 0.999);

        if let Some(refined) =
            refine_ssi_point_with_context(s1, s2, u1p, v1p, u2, v2, 1e-8, context, scratch)?
        {
            let d = refined.point - point.point;
            let dist = d.length();
            if dist > best_dist
                && dist > 1e-12
                && let Ok(normalized) = d.normalize()
            {
                best_dist = dist;
                best_dir = Some(normalized);
            }
        }
    }

    Ok(best_dir)
}

/// Constrain a parameter value to the domain, wrapping if periodic or
/// clamping if not.
///
/// For periodic parameters, values that exceed the domain are wrapped
/// modulo the period (e.g., `u = 6.5` on a `[0, 2pi]` cylinder wraps
/// to `u ~ 0.217`). For non-periodic parameters, values are clamped
/// with a 0.1% margin to avoid evaluation at exact boundaries.
pub(super) fn constrain_param(v: f64, min: f64, max: f64, periodic: bool) -> f64 {
    if periodic {
        let span = max - min;
        if span <= 0.0 {
            return min;
        }
        let wrapped = min + (v - min).rem_euclid(span);
        // Tiny margin to stay within evaluable domain.
        let margin = 1e-10 * span;
        wrapped.clamp(min + margin, max - margin)
    } else {
        let margin = 0.001 * (max - min);
        v.clamp(min + margin, max - margin)
    }
}

/// Constrain all four parameters to their respective surface domains,
/// wrapping periodic parameters instead of clamping.
pub(super) fn constrain_state(state: &[f64; 4], s1: &NurbsSurface, s2: &NurbsSurface) -> [f64; 4] {
    let (u1_min, u1_max) = s1.domain_u();
    let (v1_min, v1_max) = s1.domain_v();
    let (u2_min, u2_max) = s2.domain_u();
    let (v2_min, v2_max) = s2.domain_v();
    [
        constrain_param(state[0], u1_min, u1_max, s1.is_periodic_u()),
        constrain_param(state[1], v1_min, v1_max, s1.is_periodic_v()),
        constrain_param(state[2], u2_min, u2_max, s2.is_periodic_u()),
        constrain_param(state[3], v2_min, v2_max, s2.is_periodic_v()),
    ]
}

/// Check if a parameter state is at the domain boundary of either surface.
///
/// For periodic parameters, wrapping means we never hit a boundary -- the
/// surface seamlessly continues. Only non-periodic parameters can be "at
/// boundary" (e.g., the v-direction of a cylinder, or any NURBS surface).
fn at_boundary(state: &[f64; 4], s1: &NurbsSurface, s2: &NurbsSurface) -> bool {
    let periodic = [
        s1.is_periodic_u(),
        s1.is_periodic_v(),
        s2.is_periodic_u(),
        s2.is_periodic_v(),
    ];
    let constrained = constrain_state(state, s1, s2);
    state
        .iter()
        .zip(constrained.iter())
        .zip(periodic.iter())
        .any(|((&s, &c), &is_per)| !is_per && (s - c).abs() > f64::EPSILON)
}

/// March in one direction along the intersection curve using RKF45
/// adaptive stepping with closed-loop detection and curvature-based
/// step adaptation.
///
/// Combines two adaptation strategies:
/// - **RKF45 error control**: halves step when integration error exceeds tolerance
/// - **Angular deviation**: halves step when the tangent turns more than 10 deg per
///   step, doubles when less than 2 deg. This ensures fine resolution on
///   high-curvature regions (tight bends) and efficient large steps on
///   straight portions.
#[allow(clippy::too_many_lines, clippy::many_single_char_names)]
#[allow(clippy::too_many_arguments)]
fn march_direction(
    s1: &NurbsSurface,
    s2: &NurbsSurface,
    seed: &IntersectionPoint,
    forward: bool,
    step_size: f64,
    tolerance: f64,
    max_steps: usize,
    context: &OperationContext,
    scratch: &mut SsiScratch,
) -> Result<Vec<IntersectionPoint>, MathError> {
    // Maximum number of turning points (tangent reversals) to traverse.
    // Realistic SSI curves have at most 2-3 turning points; the limit
    // prevents infinite loops on degenerate near-tangential cases.
    const MAX_TURNING_POINTS: usize = 3;

    let mut points = Vec::new();
    let mut current = *seed;

    let mut sign = if forward { 1.0 } else { -1.0 };
    let mut h = step_size;
    // Minimum step: don't go below 1/1000th of the initial step.
    // Using tolerance (1e-6) as h_min caused runaway step-halving in
    // near-tangential cases -- thousands of tiny steps each requiring
    // full RKF45 + Newton cycles.
    let h_min = (step_size * 1e-3).max(tolerance);
    let max_h = step_size * 4.0;
    // Angular thresholds in radians for curvature adaptation.
    let max_angle = 10.0_f64.to_radians(); // halve step if tangent turns > 10 deg
    let min_angle = 2.0_f64.to_radians(); // double step if tangent turns < 2 deg

    // Track previous 3D tangent for angular deviation.
    let mut prev_tangent: Option<Vec3> = {
        let n1 = scratch.normal1(s1, seed.param1.0, seed.param1.1).ok();
        let n2 = scratch.normal2(s2, seed.param2.0, seed.param2.1).ok();
        match (n1, n2) {
            (Some(n1), Some(n2)) => {
                let t = n1.cross(n2);
                t.normalize()
                    .ok()
                    .map(|t| Vec3::new(t.x() * sign, t.y() * sign, t.z() * sign))
            }
            _ => None,
        }
    };

    let mut total_evals = 0_usize;
    let max_evals = max_steps * 3;
    let mut turning_points_count = 0_usize;

    for _ in 0..max_steps {
        context.check_cancelled()?;
        let y = [
            current.param1.0,
            current.param1.1,
            current.param2.0,
            current.param2.1,
        ];

        // Try RKF45 with adaptive step, allowing a few retries per accepted step.
        let (y4, accepted_h) = loop {
            context.check_cancelled()?;
            total_evals += 1;
            if total_evals > max_evals {
                return Ok(points);
            }

            let Some(result) = rkf45_step(s1, s2, &y, h, sign, prev_tangent, context, scratch)?
            else {
                return Ok(points);
            };

            let (y4, y5) = result;

            // Compute error estimate.
            let err = ((y5[0] - y4[0]).powi(2)
                + (y5[1] - y4[1]).powi(2)
                + (y5[2] - y4[2]).powi(2)
                + (y5[3] - y4[3]).powi(2))
            .sqrt();

            if err > tolerance && h > h_min {
                // Reject step, halve h and retry.
                h = (h * 0.5).max(h_min);
                continue;
            }

            // Accept this step.
            let accepted = h;

            // Adjust h for next step based on integration error.
            if err < tolerance / 10.0 {
                h = (h * 2.0).min(max_h);
            }

            break (y4, accepted);
        };
        let _ = accepted_h;

        // Accept the 4th-order solution (more conservative).
        let next = constrain_state(&y4, s1, s2);

        // Newton-refine to stay on the intersection curve.
        if let Some(refined) = refine_ssi_point_with_context(
            s1, s2, next[0], next[1], next[2], next[3], tolerance, context, scratch,
        )? {
            // Check that we actually moved.
            if (refined.point - current.point).length() < tolerance {
                break;
            }

            // Curvature-based step adaptation: compute tangent at the new
            // point and check angular deviation from the previous tangent.
            let cur_tangent = {
                let n1 = scratch.normal1(s1, refined.param1.0, refined.param1.1).ok();
                let n2 = scratch.normal2(s2, refined.param2.0, refined.param2.1).ok();
                match (n1, n2) {
                    (Some(n1), Some(n2)) => {
                        let t = n1.cross(n2);
                        t.normalize()
                            .ok()
                            .map(|t| Vec3::new(t.x() * sign, t.y() * sign, t.z() * sign))
                    }
                    _ => None,
                }
            };

            if let (Some(prev_t), Some(cur_t)) = (prev_tangent, cur_tangent) {
                let cos_angle = prev_t.dot(cur_t).clamp(-1.0, 1.0);
                let angle = cos_angle.acos();

                if cos_angle < 0.0 {
                    // Turning point detected: tangent reversed direction.
                    // This happens when the intersection curve has a cusp
                    // or reversal in parameter space. Use bisection to
                    // locate the turning point precisely, then continue
                    // marching through it by flipping the sign.
                    if h > h_min * 4.0 {
                        // Retry with much smaller step to get closer to the
                        // turning point before continuing.
                        h = (h * 0.25).max(h_min);
                        // Don't add this point; we'll re-step.
                        continue;
                    }
                    // At minimum step: accept the turning point and
                    // continue marching past it. The tangent has reversed,
                    // so flip the sign to keep following the curve in the
                    // new direction.
                    points.push(refined);
                    current = refined;
                    sign = -sign;
                    // cur_tangent was computed with the old sign. After
                    // flipping, negate it so the angular deviation check
                    // on the next step sees a consistent forward direction.
                    prev_tangent = cur_tangent.map(|t| Vec3::new(-t.x(), -t.y(), -t.z()));
                    h = step_size; // Reset step size for the new direction.
                    turning_points_count += 1;
                    if turning_points_count >= MAX_TURNING_POINTS {
                        break;
                    }
                    continue;
                } else if angle > max_angle && h > h_min {
                    // High curvature -- reduce step size for next iteration.
                    h = (h * 0.5).max(h_min);
                } else if angle < min_angle {
                    // Low curvature -- increase step size.
                    h = (h * 2.0).min(max_h);
                }
                // Otherwise keep current step size.
            }

            prev_tangent = cur_tangent;

            // Check boundary.
            let ref_state = [
                refined.param1.0,
                refined.param1.1,
                refined.param2.0,
                refined.param2.1,
            ];
            if at_boundary(&ref_state, s1, s2) {
                points.push(refined);
                break;
            }

            // Closed-loop detection: check if the current 3D point is close
            // to the first traced segment (not just the seed). Using 3D
            // distance instead of 4D parameter distance avoids issues when
            // surfaces have very different parameterization scales.
            if points.len() >= 5 {
                let d_3d = (refined.point - seed.point).length();
                // Also check against the second point to detect crossing
                // the start segment, not just proximity to the seed.
                // 100x: generous loop-closure detection radius to avoid missed closures
                let near_seed = d_3d < tolerance * 100.0;
                let near_first_seg = if points.len() >= 2 {
                    point_to_segment_dist(refined.point, points[0].point, points[1].point)
                        < tolerance * 100.0
                } else {
                    false
                };

                if near_seed || near_first_seg {
                    // Close the loop by adding the seed point.
                    points.push(*seed);
                    break;
                }
            }

            points.push(refined);
            current = refined;
        } else {
            break;
        }
    }

    Ok(points)
}

/// Perform one RKF45 step. Returns `(y_4th, y_5th)` or `None` if
/// tangent evaluation fails at any stage.
#[allow(
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::type_complexity,
    clippy::too_many_arguments
)]
fn rkf45_step(
    s1: &NurbsSurface,
    s2: &NurbsSurface,
    y: &[f64; 4],
    h: f64,
    sign: f64,
    prev: Option<Vec3>,
    context: &OperationContext,
    scratch: &mut SsiScratch,
) -> Result<Option<([f64; 4], [f64; 4])>, MathError> {
    // Helper: evaluate f at a state, scaling by h. `prev` carries the
    // incoming march direction so singular nulls preserve continuity.
    let mut f = |state: &[f64; 4]| -> Result<Option<[f64; 4]>, MathError> {
        let clamped = constrain_state(state, s1, s2);
        let Some(t) = ssi_tangent_params(
            s1, s2, clamped[0], clamped[1], clamped[2], clamped[3], sign, prev, context, scratch,
        )?
        else {
            return Ok(None);
        };
        Ok(Some([t[0] * h, t[1] * h, t[2] * h, t[3] * h]))
    };

    // Helper: y + sum of scaled k vectors.
    let add = |base: &[f64; 4], terms: &[(&[f64; 4], f64)]| -> [f64; 4] {
        let mut out = *base;
        for &(k, coeff) in terms {
            for i in 0..4 {
                out[i] += k[i] * coeff;
            }
        }
        out
    };

    let Some(k1) = f(y)? else {
        return Ok(None);
    };
    let Some(k2) = f(&add(y, &[(&k1, 1.0 / 4.0)]))? else {
        return Ok(None);
    };
    let Some(k3) = f(&add(y, &[(&k1, 3.0 / 32.0), (&k2, 9.0 / 32.0)]))? else {
        return Ok(None);
    };
    let Some(k4) = f(&add(
        y,
        &[
            (&k1, 1932.0 / 2197.0),
            (&k2, -7200.0 / 2197.0),
            (&k3, 7296.0 / 2197.0),
        ],
    ))?
    else {
        return Ok(None);
    };
    let Some(k5) = f(&add(
        y,
        &[
            (&k1, 439.0 / 216.0),
            (&k2, -8.0),
            (&k3, 3680.0 / 513.0),
            (&k4, -845.0 / 4104.0),
        ],
    ))?
    else {
        return Ok(None);
    };
    let Some(k6) = f(&add(
        y,
        &[
            (&k1, -8.0 / 27.0),
            (&k2, 2.0),
            (&k3, -3544.0 / 2565.0),
            (&k4, 1859.0 / 4104.0),
            (&k5, -11.0 / 40.0),
        ],
    ))?
    else {
        return Ok(None);
    };

    // 4th-order solution.
    let y4 = add(
        y,
        &[
            (&k1, 25.0 / 216.0),
            (&k3, 1408.0 / 2565.0),
            (&k4, 2197.0 / 4104.0),
            (&k5, -1.0 / 5.0),
        ],
    );

    // 5th-order solution.
    let y5 = add(
        y,
        &[
            (&k1, 16.0 / 135.0),
            (&k3, 6656.0 / 12825.0),
            (&k4, 28561.0 / 56430.0),
            (&k5, -9.0 / 50.0),
            (&k6, 2.0 / 55.0),
        ],
    );

    Ok(Some((y4, y5)))
}

/// Project a 3D tangent vector onto surface parameter space.
///
/// Given a surface's first partials `(su, sv)` (the same values
/// `surface.derivatives(u, v, 1)` returns at `[1][0]`/`[0][1]`) and a 3D
/// tangent direction scaled by `step`, compute the parameter
/// increments (du, dv) that move along the tangent on the surface.
fn project_tangent_to_params(su: Vec3, sv: Vec3, tangent: Vec3, step: f64) -> (f64, f64) {
    let t = Vec3::new(tangent.x() * step, tangent.y() * step, tangent.z() * step);

    // Solve [su*su, su*sv; su*sv, sv*sv] [du; dv] = [su*t; sv*t]
    let a11 = su.dot(su);
    let a12 = su.dot(sv);
    let a22 = sv.dot(sv);
    let b1 = su.dot(t);
    let b2 = sv.dot(t);

    let det = a11.mul_add(a22, -(a12 * a12));
    if det.abs() < 1e-20 {
        return (0.0, 0.0);
    }

    let du = b1.mul_add(a22, -(b2 * a12)) / det;
    let dv = a11.mul_add(b2, -(a12 * b1)) / det;

    (du, dv)
}

/// Compute a Newton step to move (u, v) on the surface closer to a target
/// 3D point. Solves the 2x2 system from the surface's first derivatives.
#[allow(clippy::too_many_arguments)]
pub(super) fn surface_newton_step(
    scratch: &mut DerivativeScratch,
    surface: &NurbsSurface,
    u: f64,
    v: f64,
    target: Point3,
) -> (f64, f64) {
    let pt = surface.evaluate(u, v);
    let r = target - pt;
    let r_vec = Vec3::new(r.x(), r.y(), r.z());

    let (su, sv) = scratch.partials_from(surface, u, v);

    // Solve: [su*su, su*sv; su*sv, sv*sv] [du; dv] = [su*r; sv*r]
    let a11 = su.dot(su);
    let a12 = su.dot(sv);
    let a22 = sv.dot(sv);
    let b1 = su.dot(r_vec);
    let b2 = sv.dot(r_vec);

    let det = a11.mul_add(a22, -(a12 * a12));

    // Relative singularity threshold scales with the Jacobian magnitude,
    // catching singularities near surface poles/apex where derivatives
    // shrink toward zero (absolute 1e-20 would be too lenient there).
    if det.abs() < (a11 + a22).max(1e-30) * 1e-12 {
        // Near-degenerate Jacobian -- surface singularity (pole, apex, seam).
        // Apply Tikhonov regularization: add lI to the normal equations.
        // This biases toward smaller steps, preventing divergence.
        let lambda = (a11 + a22).max(1e-10) * 1e-4;
        let a11r = a11 + lambda;
        let a22r = a22 + lambda;
        let det_r = a11r.mul_add(a22r, -(a12 * a12));
        if det_r.abs() < 1e-30 {
            // Still degenerate -- try stepping along whichever derivative is non-zero.
            let su_len = su.dot(su);
            let sv_len = sv.dot(sv);
            if su_len > 1e-30 {
                return (b1 / su_len, 0.0);
            }
            if sv_len > 1e-30 {
                return (0.0, b2 / sv_len);
            }
            return (0.0, 0.0);
        }
        let du = b1.mul_add(a22r, -(b2 * a12)) / det_r;
        let dv = a11r.mul_add(b2, -(a12 * b1)) / det_r;
        return (du, dv);
    }

    let du = b1.mul_add(a22, -(b2 * a12)) / det;
    let dv = a11.mul_add(b2, -(a12 * b1)) / det;

    (du, dv)
}

/// Minimum distance from point `p` to the line segment `a`-`b`.
pub(super) fn point_to_segment_dist(p: Point3, a: Point3, b: Point3) -> f64 {
    let ab = b - a;
    let ap = p - a;
    let len_sq = ab.dot(ab);
    if len_sq < 1e-30 {
        return ap.length();
    }
    let t = (ap.dot(ab) / len_sq).clamp(0.0, 1.0);
    let proj = Point3::new(a.x() + t * ab.x(), a.y() + t * ab.y(), a.z() + t * ab.z());
    (p - proj).length()
}

/// Check if a point is near any polyline segment in traced curves.
///
/// Uses segment distance (not point distance) to avoid false positives:
/// a seed near the *middle* of a traced curve won't be rejected just
/// because it's close to an interior point -- it must be within `dist`
/// of the actual polyline path. This prevents discarding seeds that
/// could reach a different branch.
pub(super) fn near_existing_segment(
    segments: &[Vec<IntersectionPoint>],
    point: &IntersectionPoint,
    dist: f64,
) -> bool {
    for seg in segments {
        if seg.len() < 2 {
            // Single-point segment: fallback to point distance.
            if let Some(p) = seg.first()
                && (p.point - point.point).length() < dist
            {
                return true;
            }
            continue;
        }
        for w in seg.windows(2) {
            if point_to_segment_dist(point.point, w[0].point, w[1].point) < dist {
                return true;
            }
        }
    }
    false
}

#[cfg(test)]
mod oracle_tests;
