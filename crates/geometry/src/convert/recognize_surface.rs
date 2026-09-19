//! Recognize NURBS surfaces as elementary analytic forms.
//!
//! Ported from `remus-heal/analysis/canonical.rs` but expressed entirely
//! in terms of `remus-math` types (no topology dependency). The result is
//! a [`RecognizedSurface`] enum describing the best-fit analytic surface, or
//! [`RecognizedSurface::NotRecognized`] when no match is found.

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point3, Vec3};

/// The analytic surface form recognized from a NURBS surface.
#[derive(Debug, Clone, PartialEq)]
pub enum RecognizedSurface {
    /// Recognized as a plane.
    Plane {
        /// Outward normal (unit vector).
        normal: Vec3,
        /// Signed distance from origin: `normal · (any point on plane)`.
        d: f64,
    },
    /// Recognized as a cylinder.
    Cylinder {
        /// A point on the cylinder axis.
        origin: Point3,
        /// Axis direction (unit vector).
        axis: Vec3,
        /// Cylinder radius.
        radius: f64,
    },
    /// Recognized as a sphere.
    Sphere {
        /// Center of the sphere.
        center: Point3,
        /// Sphere radius.
        radius: f64,
    },
    /// Recognized as a cone.
    Cone {
        /// The cone's apex (point where radius = 0).
        apex: Point3,
        /// Cone axis direction (from apex into the cone, unit vector).
        axis: Vec3,
        /// Half-angle from the radial plane to the cone generator
        /// (radians, in `(0, π/2)`).
        half_angle: f64,
    },
    /// Recognized as a torus.
    Torus {
        /// Torus center (the axis passes through this point).
        center: Point3,
        /// Torus axis direction (perpendicular to the major-circle
        /// plane, unit vector).
        axis: Vec3,
        /// Major radius (distance from torus center to tube center).
        major_radius: f64,
        /// Minor radius (tube cross-section radius).
        minor_radius: f64,
    },
    /// The surface could not be matched to any elementary form.
    NotRecognized,
}

/// Attempt to recognize a NURBS surface as an elementary analytic surface.
///
/// Tries recognition in order: plane, cylinder, sphere, cone. Returns
/// the first match whose maximum sample deviation is within
/// `tolerance`. Cylinder is tested before cone so that constant-radius
/// surfaces are classified as `Cylinder`, not as `Cone` with apex at
/// infinity.
#[must_use]
pub fn recognize_surface(surface: &NurbsSurface, tolerance: f64) -> RecognizedSurface {
    if let Some((normal, d)) = try_recognize_plane(surface, tolerance) {
        return RecognizedSurface::Plane { normal, d };
    }
    if let Some((origin, axis, radius)) = try_recognize_cylinder(surface, tolerance) {
        return RecognizedSurface::Cylinder {
            origin,
            axis,
            radius,
        };
    }
    if let Some((center, radius)) = try_recognize_sphere(surface, tolerance) {
        return RecognizedSurface::Sphere { center, radius };
    }
    if let Some((apex, axis, half_angle)) = try_recognize_cone(surface, tolerance) {
        return RecognizedSurface::Cone {
            apex,
            axis,
            half_angle,
        };
    }
    if let Some((center, axis, major_radius, minor_radius)) =
        try_recognize_torus(surface, tolerance)
    {
        return RecognizedSurface::Torus {
            center,
            axis,
            major_radius,
            minor_radius,
        };
    }
    RecognizedSurface::NotRecognized
}

// ── Plane recognition ─────────────────────────────────────────────────────────

/// Check if all control points of a NURBS surface lie on a single plane.
///
/// Returns `(normal, d)` if recognized, where `d = normal · p0`.
fn try_recognize_plane(surface: &NurbsSurface, tolerance: f64) -> Option<(Vec3, f64)> {
    let cps = surface.control_points();
    if cps.is_empty() || cps[0].is_empty() {
        return None;
    }

    // Collect all control points.
    let mut all_pts: Vec<Point3> = Vec::new();
    for row in cps {
        for pt in row {
            all_pts.push(*pt);
        }
    }

    if all_pts.len() < 3 {
        return None;
    }

    // Find a normal from the first 3 non-collinear points.
    let p0 = all_pts[0];
    let mut normal: Option<Vec3> = None;
    'outer: for i in 1..all_pts.len() {
        let v1 = all_pts[i] - p0;
        for pt in all_pts.iter().skip(i + 1) {
            let v2 = *pt - p0;
            let n = v1.cross(v2);
            if n.length() > tolerance
                && let Ok(normalized) = n.normalize()
            {
                normal = Some(normalized);
                break 'outer;
            }
        }
    }

    let n = normal?;
    let d = n.dot(Vec3::new(p0.x(), p0.y(), p0.z()));

    // Check all control points lie within tolerance of the plane.
    for pt in &all_pts {
        let dist = n.dot(Vec3::new(pt.x(), pt.y(), pt.z())) - d;
        if dist.abs() > tolerance {
            return None;
        }
    }

    // Align to the surface's own du x dv normal. The cross product above is
    // taken over control points in flattened row-major order, so its sign is an
    // accident of the control grid's layout rather than a statement about the
    // surface: for every planar face `convert_solid_to_bspline` produces, it
    // comes out OPPOSED. Callers replace a `Nurbs` face with
    // `FaceSurface::Plane { normal, d }`, and a plane that disagrees with the
    // surface it was recognized from is a trap for any consumer that reads the
    // normal as the face's outward direction.
    let (u0, u1) = surface.domain_u();
    let (v0, v1) = surface.domain_v();
    match surface.normal(0.5 * (u0 + u1), 0.5 * (v0 + v1)) {
        Ok(du_cross_dv) if n.dot(du_cross_dv) < 0.0 => Some((-n, -d)),
        _ => Some((n, d)),
    }
}

// ── Cylinder recognition ──────────────────────────────────────────────────────

/// Check if a NURBS surface is a cylinder.
///
/// Estimates the axis from the v-direction within each control-point row
/// (averaged across all rows), then verifies that an 8×8 sample grid lies
/// at a consistent radial distance from that axis.
///
/// This handles both the exact rational form (9 u-rows × 2 v-columns) and
/// the sampled bilinear form (nu rows × nv columns).
#[allow(clippy::items_after_statements)]
fn try_recognize_cylinder(surface: &NurbsSurface, tolerance: f64) -> Option<(Point3, Vec3, f64)> {
    let cps = surface.control_points();
    if cps.len() < 2 {
        return None;
    }
    for row in cps {
        if row.len() < 2 {
            return None;
        }
    }

    // Estimate axis as average of (last_col - first_col) across all rows.
    let mut axis_sum = Vec3::new(0.0, 0.0, 0.0);
    for row in cps {
        let v = row[row.len() - 1] - row[0];
        axis_sum += v;
    }
    #[allow(clippy::cast_precision_loss)]
    let axis_avg = axis_sum * (1.0 / cps.len() as f64);
    let axis_len = axis_avg.length();
    if axis_len < tolerance {
        return None;
    }
    let axis = axis_avg.normalize().ok()?;

    // Sample at an 8×8 grid of evaluated surface points.
    // We use evaluated points (not control points) for the axis origin because
    // rational NURBS control points are NOT on the surface — the centroid of
    // weighted CPs would be skewed.
    let (u0, u1) = surface.domain_u();
    let (v0, v1) = surface.domain_v();
    const N: usize = 8;

    let mut samples: Vec<Point3> = Vec::with_capacity(N * N);
    for iu in 0..N {
        #[allow(clippy::cast_precision_loss)]
        let u = u0 + (u1 - u0) * (iu as f64) / ((N - 1) as f64);
        for iv in 0..N {
            #[allow(clippy::cast_precision_loss)]
            let v = v0 + (v1 - v0) * (iv as f64) / ((N - 1) as f64);
            samples.push(surface.evaluate(u, v));
        }
    }

    // Find the axis position by least-squares circle fitting in the plane
    // perpendicular to the axis. Project each sample to 2D (removing the
    // axial component), then solve the algebraic circle equation:
    //   x² + y² = 2·cx·x + 2·cy·y + (r² - cx² - cy²)
    // This is linear in (cx, cy, C) and gives the circle center.
    let ref_pt = samples[0];

    // Build a 2D coordinate system perpendicular to the axis.
    let perp1 = {
        let trial = if axis.x().abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let p = trial - axis * axis.dot(trial);
        p.normalize().unwrap_or(Vec3::new(1.0, 0.0, 0.0))
    };
    let perp2 = axis.cross(perp1);

    // Project samples to 2D (perpendicular to axis).
    let pts_2d: Vec<(f64, f64)> = samples
        .iter()
        .map(|pt| {
            let v = *pt - ref_pt;
            (perp1.dot(v), perp2.dot(v))
        })
        .collect();

    // Solve least-squares: for each (x,y), x²+y² = 2*cx*x + 2*cy*y + C
    // ATA * [cx, cy, C/2] = ATb where A[i] = [2x, 2y, 1] and b[i] = x²+y²
    let mut ata = [[0.0_f64; 3]; 3];
    let mut atb = [0.0_f64; 3];
    for &(x, y) in &pts_2d {
        let rhs = x * x + y * y;
        let row = [2.0 * x, 2.0 * y, 1.0];
        for i in 0..3 {
            for j in 0..3 {
                ata[i][j] += row[i] * row[j];
            }
            atb[i] += row[i] * rhs;
        }
    }

    let sol = solve_3x3(ata, atb)?;
    let cx = sol[0];
    let cy = sol[1];
    // Recover axis origin in 3D.
    let origin = ref_pt + perp1 * cx + perp2 * cy;

    let mut radii: Vec<f64> = Vec::with_capacity(samples.len());
    for pt in &samples {
        let to_pt = *pt - origin;
        let along = axis.dot(to_pt);
        let radial = to_pt - axis * along;
        radii.push(radial.length());
    }

    if radii.is_empty() {
        return None;
    }

    let sum: f64 = radii.iter().sum();
    #[allow(clippy::cast_precision_loss)]
    let mean_radius = sum / radii.len() as f64;

    if mean_radius < tolerance {
        return None; // Degenerate — axis passes through all points.
    }

    let max_dev = radii
        .iter()
        .map(|r| (r - mean_radius).abs())
        .fold(0.0_f64, f64::max);
    if max_dev > tolerance {
        return None;
    }

    Some((origin, axis, mean_radius))
}

// ── Sphere recognition ────────────────────────────────────────────────────────

/// Check if a NURBS surface is a sphere.
///
/// Samples an 8×8 grid, estimates the center by solving a 3×3 least-squares
/// system, then verifies all sample points are equidistant from that center.
#[allow(clippy::items_after_statements)]
fn try_recognize_sphere(surface: &NurbsSurface, tolerance: f64) -> Option<(Point3, f64)> {
    let (u0, u1) = surface.domain_u();
    let (v0, v1) = surface.domain_v();
    const N: usize = 8;

    let mut samples: Vec<Point3> = Vec::with_capacity(N * N);

    for iu in 0..N {
        #[allow(clippy::cast_precision_loss)]
        let u = u0 + (u1 - u0) * (iu as f64) / ((N - 1) as f64);
        for iv in 0..N {
            #[allow(clippy::cast_precision_loss)]
            let v = v0 + (v1 - v0) * (iv as f64) / ((N - 1) as f64);
            samples.push(surface.evaluate(u, v));
        }
    }

    if samples.len() < 4 {
        return None;
    }

    // Solve least-squares for center using algebraic approach.
    // For each pair (p0, pi), the difference equation eliminates R²:
    //   2*(pi - p0) · c = pi² - p0²
    let sq = |p: Point3| p.x() * p.x() + p.y() * p.y() + p.z() * p.z();

    let n = samples.len();
    let mut ata = [[0.0_f64; 3]; 3];
    let mut atb = [0.0_f64; 3];

    let p0 = samples[0];
    let sq0 = sq(p0);

    for i in 1..n {
        let pi = samples[i];
        let a_row = [
            2.0 * (pi.x() - p0.x()),
            2.0 * (pi.y() - p0.y()),
            2.0 * (pi.z() - p0.z()),
        ];
        let bi = sq(pi) - sq0;

        for r in 0..3 {
            for c in 0..3 {
                ata[r][c] += a_row[r] * a_row[c];
            }
            atb[r] += a_row[r] * bi;
        }
    }

    let center = solve_3x3(ata, atb)?;
    let center_pt = Point3::new(center[0], center[1], center[2]);

    let mut distances: Vec<f64> = Vec::with_capacity(n);
    for pt in &samples {
        let d = Vec3::new(
            pt.x() - center_pt.x(),
            pt.y() - center_pt.y(),
            pt.z() - center_pt.z(),
        )
        .length();
        distances.push(d);
    }

    let sum: f64 = distances.iter().sum();
    #[allow(clippy::cast_precision_loss)]
    let mean_radius = sum / distances.len() as f64;

    if mean_radius < tolerance {
        return None;
    }

    let max_dev = distances
        .iter()
        .map(|d| (d - mean_radius).abs())
        .fold(0.0_f64, f64::max);

    if max_dev > tolerance {
        return None;
    }

    Some((center_pt, mean_radius))
}

// ── Cone recognition ──────────────────────────────────────────────────────────

/// Check if all sampled surface points lie on a cone.
///
/// Estimates the axis from the average of "last column − first column"
/// across all CP rows (same as cylinder, since cone has the same
/// rotational structure). Then verifies samples lie on a cone by
/// checking that:
///
/// 1. The axial-component vs radial-component relationship is linear
///    (samples lie on a 2D wedge in `(axial, radial)` space).
/// 2. The radial component is consistent for all u at each fixed v
///    (each iso-v line is a circle around the axis).
///
/// The slope of the radial-vs-axial line gives `cot(half_angle)`; the
/// apex is the (axial, 0) intercept extrapolated from this line.
fn try_recognize_cone(surface: &NurbsSurface, tolerance: f64) -> Option<(Point3, Vec3, f64)> {
    const N: usize = 8;
    let cps = surface.control_points();
    if cps.len() < 2 {
        return None;
    }
    for row in cps {
        if row.len() < 2 {
            return None;
        }
    }

    // Estimate axis direction. For cones (unlike cylinders), the
    // (last_col - first_col) vector at row i has both an axial AND a
    // radial component (cos_a · radial_dir(u_i) + sin_a · axis) — so
    // averaging across u must cancel the radial part. The 33×9 CP
    // grid produced by `analytic_to_nurbs_sampled` duplicates the
    // u=0 and u=2π seam (CP[0] and CP[N-1] at the same 3D point),
    // which biases the unweighted sum. Skip the last row to remove
    // the duplicate before averaging.
    let n_rows = cps.len();
    let row_count = if n_rows >= 3 && (cps[0][0] - cps[n_rows - 1][0]).length() < tolerance {
        n_rows - 1
    } else {
        n_rows
    };
    let mut axis_sum = Vec3::new(0.0, 0.0, 0.0);
    for row in cps.iter().take(row_count) {
        let v = row[row.len() - 1] - row[0];
        axis_sum += v;
    }
    #[allow(clippy::cast_precision_loss)]
    let axis_avg = axis_sum * (1.0 / row_count as f64);
    if axis_avg.length() < tolerance {
        return None;
    }
    let axis = axis_avg.normalize().ok()?;

    // Sample at an 8×8 grid. CRITICAL: use OPEN range in u to avoid
    // duplicating the closing seam point (u_nurbs=0 and u_nurbs=1
    // coincide for full-revolution surfaces). Duplicates bias the
    // centroid off-axis, which throws off the radial-component
    // computation for samples near the duplicate.
    let (u0, u1) = surface.domain_u();
    let (v0, v1) = surface.domain_v();
    let mut samples: Vec<Point3> = Vec::with_capacity(N * N);
    for iu in 0..N {
        #[allow(clippy::cast_precision_loss)]
        let u = u0 + (u1 - u0) * (iu as f64 + 0.5) / (N as f64);
        for iv in 0..N {
            #[allow(clippy::cast_precision_loss)]
            let v = v0 + (v1 - v0) * (iv as f64) / ((N - 1) as f64);
            samples.push(surface.evaluate(u, v));
        }
    }

    // Estimate the apex (axis origin) by linear-fitting (axial,
    // radial) pairs. For each sample, `axial = axis · (p − sample[0])`
    // is a relative axial distance; `radial` is the perpendicular
    // distance from sample[0]'s axial projection. Wait — for cone
    // recognition we need a robust BUT axis-relative reference. Use
    // the centroid of all samples as the "anchor" for axial measurement.
    #[allow(clippy::cast_precision_loss)]
    let inv_n = 1.0 / samples.len() as f64;
    let mut anchor_x = 0.0_f64;
    let mut anchor_y = 0.0_f64;
    let mut anchor_z = 0.0_f64;
    for p in &samples {
        anchor_x += p.x();
        anchor_y += p.y();
        anchor_z += p.z();
    }
    let anchor = Point3::new(anchor_x * inv_n, anchor_y * inv_n, anchor_z * inv_n);

    // Measure axial and radial offsets from anchor along the axis.
    // For each sample, compute axial = axis · (p − anchor) and
    // radial = |(p − anchor) − axial · axis|. For a true cone with
    // apex at (anchor + axial_apex · axis), the radial component is
    // a linear function of axial: radial = |slope · (axial − axial_apex)|.
    let mut axials: Vec<f64> = Vec::with_capacity(samples.len());
    let mut radials: Vec<f64> = Vec::with_capacity(samples.len());
    for p in &samples {
        let to_p = *p - anchor;
        let along = axis.dot(to_p);
        let radial_vec = to_p - axis * along;
        axials.push(along);
        radials.push(radial_vec.length());
    }

    // Reject degenerate (all radials zero or all the same): would be
    // a line/cylinder, not a cone.
    let max_r = radials.iter().fold(0.0_f64, |m, &r| m.max(r));
    let min_r = radials.iter().fold(f64::INFINITY, |m, &r| m.min(r));
    if max_r - min_r < tolerance {
        return None; // Constant radius → cylinder (handled earlier).
    }

    // Linear fit: radial = m · axial + b. Then cone apex is at
    // axial_apex = -b / m, with radial_apex = 0. For axisymmetry,
    // the radial side should be an ABSOLUTE value (always >= 0); we
    // exploit the fact that radial is a vector magnitude, so on the
    // cone the relationship `radial = slope · (axial − axial_apex)`
    // holds with `slope > 0` for axials > axial_apex.
    //
    // We use unsigned-radial least-squares: pick the slope from a
    // simple linear regression of (axial, radial). For a true cone
    // the residual should be near zero.
    let n_f = samples.len() as f64;
    let sum_a: f64 = axials.iter().sum();
    let sum_r: f64 = radials.iter().sum();
    let mean_a = sum_a / n_f;
    let mean_r = sum_r / n_f;
    let mut s_aa = 0.0_f64;
    let mut s_ar = 0.0_f64;
    for i in 0..samples.len() {
        let da = axials[i] - mean_a;
        let dr = radials[i] - mean_r;
        s_aa += da * da;
        s_ar += da * dr;
    }
    if s_aa < 1e-30 {
        return None;
    }
    let slope = s_ar / s_aa;
    let intercept = mean_r - slope * mean_a;
    if slope.abs() < tolerance {
        return None; // Slope ≈ 0 → cylinder.
    }
    // Apex axial position relative to anchor.
    let axial_apex = -intercept / slope;

    // Verify residuals.
    for i in 0..samples.len() {
        let pred = slope * axials[i] + intercept;
        if (radials[i] - pred).abs() > tolerance {
            return None;
        }
    }

    // Compute half-angle from slope. The cone equation in local
    // (axial, radial) coords is `radial = (axial - axial_apex) ·
    // |slope|` for axial > axial_apex. The slope equals
    // cos(half_angle) / sin(half_angle) = cot(half_angle), so
    // half_angle = atan(1 / |slope|).
    //
    // remus's half_angle convention is the angle from the RADIAL
    // plane to the generator, so half_angle ∈ (0, π/2).
    let half_angle = (1.0 / slope.abs()).atan();
    if !(0.0 < half_angle && half_angle < std::f64::consts::FRAC_PI_2) {
        return None;
    }

    // Apex in 3D. Cone axis points from apex INTO the cone (positive
    // axial direction). If our slope is negative (radial decreases
    // with positive axial), the apex is in the +axial direction;
    // axis should point AWAY from the apex (negative-axial-from-apex
    // direction, i.e., positive `slope` convention).
    let apex_offset = axis * axial_apex;
    let apex = anchor + apex_offset;
    let cone_axis = if slope > 0.0 { axis } else { -axis };

    Some((apex, cone_axis, half_angle))
}

// ── Torus recognition ────────────────────────────────────────────────────────

/// Check if all sampled surface points lie on a torus.
///
/// A torus is the surface of revolution of a cross-section circle of
/// radius `minor_radius` whose center lies at distance `major_radius`
/// from the axis. In `(axial, radial)` space relative to the axis, the
/// samples lie on a circle of radius `minor_radius` centered at
/// `(0, major_radius)`.
///
/// Algorithm:
/// 1. Estimate axis (skip seam-duplicate row, same as cone).
/// 2. Sample 8×8 with open-range u to avoid centroid bias.
/// 3. Compute `(axial, radial)` per sample relative to the centroid.
/// 4. Fit a circle to the `(axial, radial)` points via algebraic
///    least-squares.
/// 5. Verify all points lie on this circle within tolerance.
/// 6. Center of fit circle gives `(axial_center, major_radius)`;
///    radius gives `minor_radius`.
#[allow(clippy::items_after_statements)]
fn try_recognize_torus(surface: &NurbsSurface, tolerance: f64) -> Option<(Point3, Vec3, f64, f64)> {
    const N: usize = 8;
    let cps = surface.control_points();
    if cps.len() < 2 {
        return None;
    }
    for row in cps {
        if row.len() < 2 {
            return None;
        }
    }

    // Estimate axis: for a torus, the U direction is revolution
    // around the major axis, so the FIRST column of CPs (cps[i][0]
    // for varying i) traces a circle in a plane perpendicular to the
    // axis. The cylinder/cone trick (last_col − first_col averaged)
    // doesn't work because v is closed (last_col ≈ first_col).
    //
    // Find three non-collinear CPs in the first column and take the
    // cross-product of their relative offsets — that gives the
    // plane normal, i.e., the torus axis.
    let n_rows = cps.len();
    if n_rows < 3 {
        return None;
    }
    let p0 = cps[0][0];
    let mut axis: Option<Vec3> = None;
    'outer: for i in 1..n_rows {
        let v1 = cps[i][0] - p0;
        for j in (i + 1)..n_rows {
            let v2 = cps[j][0] - p0;
            let cross = v1.cross(v2);
            if cross.length() > tolerance
                && let Ok(normalized) = cross.normalize()
            {
                axis = Some(normalized);
                break 'outer;
            }
        }
    }
    let axis = axis?;

    // Sample with open-range u.
    let (u0, u1) = surface.domain_u();
    let (v0, v1) = surface.domain_v();
    let mut samples: Vec<Point3> = Vec::with_capacity(N * N);
    for iu in 0..N {
        #[allow(clippy::cast_precision_loss)]
        let u = u0 + (u1 - u0) * (iu as f64 + 0.5) / (N as f64);
        for iv in 0..N {
            #[allow(clippy::cast_precision_loss)]
            let v = v0 + (v1 - v0) * (iv as f64) / ((N - 1) as f64);
            samples.push(surface.evaluate(u, v));
        }
    }

    // Centroid as anchor.
    #[allow(clippy::cast_precision_loss)]
    let inv_n = 1.0 / samples.len() as f64;
    let mut ax = 0.0_f64;
    let mut ay = 0.0_f64;
    let mut az = 0.0_f64;
    for p in &samples {
        ax += p.x();
        ay += p.y();
        az += p.z();
    }
    let anchor = Point3::new(ax * inv_n, ay * inv_n, az * inv_n);

    // (axial, radial) for each sample.
    let mut axials: Vec<f64> = Vec::with_capacity(samples.len());
    let mut radials: Vec<f64> = Vec::with_capacity(samples.len());
    for p in &samples {
        let to_p = *p - anchor;
        let along = axis.dot(to_p);
        let radial = (to_p - axis * along).length();
        axials.push(along);
        radials.push(radial);
    }

    // Fit a circle to (axial, radial) points: solve algebraic
    //   x² + y² = 2·cx·x + 2·cy·y + (R² − cx² − cy²)
    // ⇒ row = [2x, 2y, 1], rhs = x² + y², solve for [cx, cy, K].
    let mut ata = [[0.0_f64; 3]; 3];
    let mut atb = [0.0_f64; 3];
    for i in 0..samples.len() {
        let x = axials[i];
        let y = radials[i];
        let row = [2.0 * x, 2.0 * y, 1.0];
        let rhs = x * x + y * y;
        for r in 0..3 {
            for c in 0..3 {
                ata[r][c] += row[r] * row[c];
            }
            atb[r] += row[r] * rhs;
        }
    }
    let sol = solve_3x3(ata, atb)?;
    let center_axial = sol[0];
    let major_radius = sol[1];
    let k = sol[2]; // R² − cx² − cy²
    let r_sq = k + center_axial * center_axial + major_radius * major_radius;
    if r_sq <= 0.0 || major_radius <= 0.0 {
        return None;
    }
    let minor_radius = r_sq.sqrt();

    // Reject if minor >= major (degenerate torus / cylinder-with-radius).
    if minor_radius >= major_radius - tolerance {
        return None;
    }

    // Verify residuals.
    for i in 0..samples.len() {
        let dx = axials[i] - center_axial;
        let dy = radials[i] - major_radius;
        let dist = (dx * dx + dy * dy).sqrt();
        if (dist - minor_radius).abs() > tolerance {
            return None;
        }
    }

    let center = anchor + axis * center_axial;
    Some((center, axis, major_radius, minor_radius))
}

// ── Utilities ─────────────────────────────────────────────────────────────────

/// Solve a 3×3 linear system `A * x = b` via Cramer's rule.
///
/// Returns `None` if the determinant is near zero (singular system).
/// Exposed at `pub(super)` so [`super::recognize_curve`] can reuse it
/// (avoids duplicating the same 3×3 solver).
pub(super) fn solve_3x3(a: [[f64; 3]; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    if det.abs() < 1e-30 {
        return None;
    }

    let inv = 1.0 / det;

    let x0 = (b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2])
        + a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2]))
        * inv;

    let x1 = (a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2])
        - b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0]))
        * inv;

    let x2 = (a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1])
        - a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0])
        + b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
        * inv;

    Some([x0, x1, x2])
}

// ── Lightweight detection ────────────────────────────────────────────────────

/// Detected geometric kind of a NURBS surface (without recovering full analytic
/// parameters). Cheaper than [`recognize_surface`] when you only need a type tag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectedSurfaceKind {
    /// All sampled points lie on a plane.
    Plane,
    /// All sampled points are equidistant from a center (sphere).
    Sphere,
    /// All sampled points are equidistant from an axis (cylinder).
    Cylinder,
    /// Generic B-spline surface.
    BSpline,
}

impl DetectedSurfaceKind {
    /// Returns the lowercase string tag for this surface kind.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Plane => "plane",
            Self::Sphere => "sphere",
            Self::Cylinder => "cylinder",
            Self::BSpline => "bspline",
        }
    }
}

/// Detect the geometric kind of a NURBS surface by sampling.
///
/// Samples an 8x8 grid and checks for sphere (equidistant from centroid) or
/// cylinder (equidistant from a PCA axis). Falls back to `BSpline`.
///
/// This is a lightweight heuristic — use [`recognize_surface`] for full analytic
/// parameter recovery.
#[must_use]
#[allow(clippy::cast_precision_loss)]
pub fn detect_surface_kind(surface: &NurbsSurface) -> DetectedSurfaceKind {
    let (u_min, u_max) = surface.domain_u();
    let (v_min, v_max) = surface.domain_v();
    let n = 8; // 8x8 grid = 64 sample points

    let mut points = Vec::with_capacity(n * n);
    for i in 0..n {
        for j in 0..n {
            let u = u_min + (u_max - u_min) * (i as f64) / ((n - 1) as f64);
            let v = v_min + (v_max - v_min) * (j as f64) / ((n - 1) as f64);
            points.push(surface.evaluate(u, v));
        }
    }

    // Compute center as average.
    let mut cx = 0.0_f64;
    let mut cy = 0.0_f64;
    let mut cz = 0.0_f64;
    for p in &points {
        cx += p.x();
        cy += p.y();
        cz += p.z();
    }
    let np = points.len() as f64;
    let center = Point3::new(cx / np, cy / np, cz / np);

    // Plane test: check if all points are coplanar.
    // Find a normal from the first non-degenerate cross product.
    let mut plane_normal = None;
    for i in 1..points.len() {
        for j in (i + 1)..points.len() {
            let v0 = points[i] - center;
            let v1 = points[j] - center;
            let n = v0.cross(v1);
            if let Ok(normalized) = n.normalize() {
                plane_normal = Some(normalized);
                break;
            }
        }
        if plane_normal.is_some() {
            break;
        }
    }
    if let Some(normal) = plane_normal {
        let is_plane = points
            .iter()
            .all(|p| (*p - center).dot(normal).abs() < 1e-6);
        if is_plane {
            return DetectedSurfaceKind::Plane;
        }
    }

    // Check if all points equidistant from center (sphere test).
    let distances: Vec<f64> = points.iter().map(|p| (*p - center).length()).collect();
    let avg_dist = distances.iter().sum::<f64>() / np;

    if avg_dist < 1e-10 {
        return DetectedSurfaceKind::BSpline;
    }

    let tol = avg_dist * 1e-3; // 0.1% relative tolerance
    let is_sphere = distances.iter().all(|d| (d - avg_dist).abs() < tol);

    if is_sphere {
        return DetectedSurfaceKind::Sphere;
    }

    // Cylinder test: points should be equidistant from an axis line.
    if let Some(axis_dir) = estimate_cylinder_axis(&points, center) {
        let projected_distances: Vec<f64> = points
            .iter()
            .map(|p| {
                let v = *p - center;
                let along_axis = v.dot(axis_dir);
                let radial = Vec3::new(
                    v.x() - axis_dir.x() * along_axis,
                    v.y() - axis_dir.y() * along_axis,
                    v.z() - axis_dir.z() * along_axis,
                );
                radial.length()
            })
            .collect();

        let avg_r = projected_distances.iter().sum::<f64>() / np;
        if avg_r > 1e-10 {
            let r_tol = avg_r * 1e-3;
            let is_cylinder = projected_distances
                .iter()
                .all(|d| (d - avg_r).abs() < r_tol);
            if is_cylinder {
                return DetectedSurfaceKind::Cylinder;
            }
        }
    }

    DetectedSurfaceKind::BSpline
}

/// Estimate the cylinder axis direction from a set of surface sample points
/// using a simple PCA-like approach (direction of maximum variance).
fn estimate_cylinder_axis(points: &[Point3], center: Point3) -> Option<Vec3> {
    // Build covariance matrix.
    let mut cxx = 0.0_f64;
    let mut cxy = 0.0_f64;
    let mut cxz = 0.0_f64;
    let mut cyy = 0.0_f64;
    let mut cyz = 0.0_f64;
    let mut czz = 0.0_f64;

    for p in points {
        let dx = p.x() - center.x();
        let dy = p.y() - center.y();
        let dz = p.z() - center.z();
        cxx += dx * dx;
        cxy += dx * dy;
        cxz += dx * dz;
        cyy += dy * dy;
        cyz += dy * dz;
        czz += dz * dz;
    }

    // Power iteration to find the principal eigenvector.
    let mut v = Vec3::new(1.0, 0.0, 0.0);
    for _ in 0..20 {
        let new_v = Vec3::new(
            v.x().mul_add(cxx, v.y().mul_add(cxy, v.z() * cxz)),
            v.x().mul_add(cxy, v.y().mul_add(cyy, v.z() * cyz)),
            v.x().mul_add(cxz, v.y().mul_add(cyz, v.z() * czz)),
        );
        let len = new_v.length();
        if len < 1e-15 {
            return None;
        }
        v = Vec3::new(new_v.x() / len, new_v.y() / len, new_v.z() / len);
    }
    Some(v)
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use remus_math::nurbs::knot_ops::surface_knot_insert_v;
    use remus_math::surfaces::{
        ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface,
    };
    use remus_math::vec::{Point3, Vec3};

    use super::*;
    use crate::convert::surface_to_nurbs::{
        cone_to_nurbs, cylinder_to_nurbs, sphere_to_nurbs, torus_to_nurbs,
    };

    fn origin() -> Point3 {
        Point3::new(0.0, 0.0, 0.0)
    }

    fn z_axis() -> Vec3 {
        Vec3::new(0.0, 0.0, 1.0)
    }

    #[test]
    fn recognize_cylinder_round_trip() {
        let cyl = CylindricalSurface::new(origin(), z_axis(), 3.0).unwrap();
        let nurbs = cylinder_to_nurbs(&cyl, (0.0, 5.0)).unwrap();

        let result = recognize_surface(&nurbs, 1e-4);
        match result {
            RecognizedSurface::Cylinder { radius, .. } => {
                assert!((radius - 3.0).abs() < 0.01, "radius {radius} != 3.0");
            }
            other => panic!("expected Cylinder, got {other:?}"),
        }
    }

    #[test]
    fn recognize_sphere_round_trip() {
        let sphere = SphericalSurface::new(origin(), 5.0).unwrap();
        let nurbs = sphere_to_nurbs(&sphere).unwrap();

        let result = recognize_surface(&nurbs, 0.1);
        match result {
            RecognizedSurface::Sphere { center, radius } => {
                let dist = Vec3::new(center.x(), center.y(), center.z()).length();
                assert!(dist < 0.5, "center too far from origin: {dist}");
                assert!((radius - 5.0).abs() < 0.5, "radius {radius} != 5.0");
            }
            other => panic!("expected Sphere, got {other:?}"),
        }
    }

    #[test]
    fn recognize_cone_round_trip() {
        // Cone with apex at origin, axis +z, half-angle π/6 (from
        // radial plane). At v=1 from apex (along generator),
        // radial = cos(π/6) ≈ 0.866, axial = sin(π/6) = 0.5.
        let half_angle = std::f64::consts::PI / 6.0;
        let cone = ConicalSurface::new(origin(), z_axis(), half_angle).unwrap();
        let nurbs = cone_to_nurbs(&cone, (1.0, 4.0)).unwrap();

        match recognize_surface(&nurbs, 0.05) {
            RecognizedSurface::Cone {
                apex,
                axis,
                half_angle: ha,
            } => {
                // Apex should be at origin within tolerance.
                assert!(
                    Vec3::new(apex.x(), apex.y(), apex.z()).length() < 0.05,
                    "apex {apex:?}"
                );
                // Axis should be along +z (or -z; both describe the same cone).
                assert!(
                    axis.dot(z_axis()).abs() > 1.0 - 1e-3,
                    "axis {axis:?} not aligned with z"
                );
                assert!(
                    (ha - half_angle).abs() < 1e-3,
                    "half_angle {ha} vs {half_angle}"
                );
            }
            other => panic!("expected Cone, got {other:?}"),
        }
    }

    #[test]
    fn cylinder_is_recognized_as_cylinder_not_cone() {
        // True cylinders should match Cylinder (tested first), not
        // fall through to Cone.
        let cyl = CylindricalSurface::new(origin(), z_axis(), 2.0).unwrap();
        let nurbs = cylinder_to_nurbs(&cyl, (0.0, 5.0)).unwrap();
        assert!(matches!(
            recognize_surface(&nurbs, 1e-4),
            RecognizedSurface::Cylinder { .. }
        ));
    }

    #[test]
    fn recognize_torus_round_trip() {
        let torus = ToroidalSurface::new(origin(), 3.0, 0.5).unwrap();
        let nurbs = torus_to_nurbs(&torus).unwrap();

        match recognize_surface(&nurbs, 0.05) {
            RecognizedSurface::Torus {
                center,
                axis,
                major_radius,
                minor_radius,
            } => {
                assert!(
                    Vec3::new(center.x(), center.y(), center.z()).length() < 0.05,
                    "center {center:?} not at origin"
                );
                assert!(
                    axis.dot(z_axis()).abs() > 1.0 - 1e-3,
                    "axis {axis:?} not aligned with z"
                );
                assert!(
                    (major_radius - 3.0).abs() < 0.05,
                    "major_radius {major_radius} vs 3.0"
                );
                assert!(
                    (minor_radius - 0.5).abs() < 0.05,
                    "minor_radius {minor_radius} vs 0.5"
                );
            }
            other => panic!("expected Torus, got {other:?}"),
        }
    }

    /// Build a flat bilinear patch in the z=0 plane. `flip` transposes the
    /// control grid, which reverses du x dv without moving a single point.
    fn flat_patch(flip: bool) -> NurbsSurface {
        let pts = [[(0.0, 0.0), (0.0, 4.0)], [(4.0, 0.0), (4.0, 4.0)]];
        let grid: Vec<Vec<Point3>> = (0..2)
            .map(|i| {
                (0..2)
                    .map(|j| {
                        let (x, y) = if flip { pts[j][i] } else { pts[i][j] };
                        Point3::new(x, y, 0.0)
                    })
                    .collect()
            })
            .collect();
        NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            grid,
            vec![vec![1.0; 2]; 2],
        )
        .unwrap()
    }

    /// The recognized normal is taken from a cross product over control points
    /// in flattened order, so its raw sign follows the grid layout rather than
    /// the surface. Consumers replace the NURBS face with
    /// `FaceSurface::Plane { normal, d }` and then read that normal as the
    /// face's outward direction, so it has to agree with the surface's own
    /// du x dv. Both layouts describe the same plane; each must be recognized
    /// with the normal its own parameterization implies.
    #[test]
    fn recognized_plane_normal_agrees_with_du_cross_dv() {
        for flip in [false, true] {
            let surface = flat_patch(flip);
            let expected = surface.normal(0.5, 0.5).unwrap();
            let RecognizedSurface::Plane { normal, d } = recognize_surface(&surface, 1e-6) else {
                panic!("flip={flip}: a flat bilinear patch must recognize as a plane");
            };
            assert!(
                normal.dot(expected) > 0.0,
                "flip={flip}: recognized normal {normal:?} opposes du x dv {expected:?}"
            );
            // The plane equation must still hold with the returned sign.
            let on_plane = surface.evaluate(0.5, 0.5);
            let residual = normal.dot(Vec3::new(on_plane.x(), on_plane.y(), on_plane.z())) - d;
            assert!(
                residual.abs() < 1e-9,
                "flip={flip}: normal and d disagree, residual {residual}"
            );
        }
    }

    #[test]
    fn cylinder_is_not_recognized_as_torus() {
        // A cylinder must hit Cylinder (tested first), not fall
        // through to Torus.
        let cyl = CylindricalSurface::new(origin(), z_axis(), 2.0).unwrap();
        let nurbs = cylinder_to_nurbs(&cyl, (0.0, 5.0)).unwrap();
        assert!(matches!(
            recognize_surface(&nurbs, 1e-4),
            RecognizedSurface::Cylinder { .. }
        ));
    }

    // ── Off-origin, tilted fixtures ───────────────────────────────────────
    //
    // Axis-aligned-at-origin fixtures hide sign and offset errors, because a
    // zero coordinate makes `a - b`, `a + b` and `a * b` agree. The fixtures
    // below are deliberately off-origin with a tilted axis and no zero
    // component, and use radii that are neither 0 nor 1.

    /// A unit direction with no zero component: `(2, -3, 6) / 7`.
    fn tilted_axis() -> Vec3 {
        Vec3::new(2.0 / 7.0, -3.0 / 7.0, 6.0 / 7.0)
    }

    /// An orthonormal in-plane basis `(e1, e2)` for the plane whose normal is
    /// [`tilted_axis`], chosen so that `e1 x e2 == tilted_axis()`.
    fn plane_basis() -> (Vec3, Vec3) {
        let s = 13.0_f64.sqrt();
        let e1 = Vec3::new(3.0 / s, 2.0 / s, 0.0);
        let e2 = tilted_axis().cross(e1);
        (e1, e2)
    }

    /// A point on the test plane, away from the origin.
    fn plane_anchor() -> Point3 {
        Point3::new(1.3, -2.1, 0.7)
    }

    fn plane_point(a: f64, b: f64) -> Point3 {
        let (e1, e2) = plane_basis();
        plane_anchor() + e1 * a + e2 * b
    }

    fn surface_from_grid(degree: usize, knots: &[f64], grid: Vec<Vec<Point3>>) -> NurbsSurface {
        let rows = grid.len();
        let cols = grid[0].len();
        NurbsSurface::new(
            degree,
            degree,
            knots.to_vec(),
            knots.to_vec(),
            grid,
            vec![vec![1.0; cols]; rows],
        )
        .unwrap()
    }

    /// Bilinear patch in the tilted plane. Its flattened control-point cross
    /// product is `e2 x e1 = -n`, i.e. OPPOSED to the patch's own du x dv, so
    /// recognition has to flip both the normal and `d`.
    fn tilted_plane_patch_opposed() -> NurbsSurface {
        let grid = [0.0_f64, 3.0]
            .iter()
            .map(|&a| [0.0_f64, 5.0].iter().map(|&b| plane_point(a, b)).collect())
            .collect();
        surface_from_grid(1, &[0.0, 0.0, 1.0, 1.0], grid)
    }

    /// Biquadratic patch in the same plane whose first control-point row is
    /// bowed, so the flattened cross product comes out as `+n` — AGREEING with
    /// du x dv. Recognition must leave this one's sign alone.
    fn tilted_plane_patch_agreeing() -> NurbsSurface {
        let bump = [0.0_f64, 1.0, 0.0];
        let grid = [0.0_f64, 2.0, 4.0]
            .iter()
            .map(|&a| {
                [0.0_f64, 2.0, 4.0]
                    .iter()
                    .zip(bump.iter())
                    .map(|(&b, &bp)| plane_point(a + bp, b))
                    .collect()
            })
            .collect();
        surface_from_grid(2, &[0.0, 0.0, 0.0, 1.0, 1.0, 1.0], grid)
    }

    /// A biquadratic saddle that is not any elementary surface.
    fn free_form_patch() -> NurbsSurface {
        let z = [[0.0, 1.3, -0.4], [1.1, -1.7, 2.2], [-0.6, 2.4, 0.9]];
        let xs = [0.0_f64, 2.0, 4.0];
        let grid = z
            .iter()
            .zip(xs.iter())
            .map(|(zrow, &x)| {
                zrow.iter()
                    .zip(xs.iter())
                    .map(|(&zv, &y)| Point3::new(x, y, zv))
                    .collect()
            })
            .collect();
        surface_from_grid(2, &[0.0, 0.0, 0.0, 1.0, 1.0, 1.0], grid)
    }

    fn to_vec(p: Point3) -> Vec3 {
        Vec3::new(p.x(), p.y(), p.z())
    }

    /// Distance from `p` to the infinite line through `origin` with unit
    /// direction `dir`.
    fn distance_to_axis(p: Point3, origin: Point3, dir: Vec3) -> f64 {
        let to_p = p - origin;
        (to_p - dir * dir.dot(to_p)).length()
    }

    /// Centroid of a 5x5 sample grid — used to check which way a recovered
    /// cone axis points.
    fn sample_centroid(surface: &NurbsSurface) -> Point3 {
        let (u0, u1) = surface.domain_u();
        let (v0, v1) = surface.domain_v();
        let (mut x, mut y, mut z) = (0.0, 0.0, 0.0);
        for iu in 0..5 {
            for iv in 0..5 {
                let p = surface.evaluate(
                    u0 + (u1 - u0) * f64::from(iu) / 4.0,
                    v0 + (v1 - v0) * f64::from(iv) / 4.0,
                );
                x += p.x();
                y += p.y();
                z += p.z();
            }
        }
        Point3::new(x / 25.0, y / 25.0, z / 25.0)
    }

    /// The surface's own direction of increasing v, at mid-u.
    fn v_direction(surface: &NurbsSurface) -> Vec3 {
        let (u0, u1) = surface.domain_u();
        let (v0, v1) = surface.domain_v();
        let um = 0.5 * (u0 + u1);
        surface.evaluate(um, v1) - surface.evaluate(um, v0)
    }

    // ── Exact rational fixtures whose 8x8 sample grid lands on arc junctions
    //
    // `detect_surface_kind` and the recognizers sample an 8x8 grid at
    // `t = i/7`. A 7-arc rational chain spanning 315 degrees puts an arc
    // junction at every one of those parameters, so all 64 samples are exactly
    // on the analytic surface, and the eight angles 0, 45, ..., 315 degrees
    // are balanced — their unit vectors sum to zero, so the sample centroid
    // lands exactly on the axis. (The full-turn forms used elsewhere duplicate
    // the seam sample, which pulls the centroid off-axis.)

    /// Radial factor and weight of the 15 control points of a 315-degree
    /// chain of seven exact rational quadratic 45-degree arcs.
    fn arc_chain(count: usize, span_deg: f64) -> Vec<(f64, f64, f64)> {
        let step = span_deg / (count as f64);
        let half = (0.5 * step).to_radians();
        (0..=2 * count)
            .map(|i| {
                let ang = (0.5 * step * i as f64).to_radians();
                let even = i % 2 == 0;
                let scale = if even { 1.0 } else { 1.0 / half.cos() };
                let w = if even { 1.0 } else { half.cos() };
                (scale * ang.cos(), scale * ang.sin(), w)
            })
            .collect()
    }

    /// Clamped degree-2 knot vector for a chain of `count` arcs
    /// (`2*count + 1` control points, double interior knots at `i/count`).
    fn arc_chain_knots(count: usize) -> Vec<f64> {
        let mut k = vec![0.0, 0.0, 0.0];
        for i in 1..count {
            let t = i as f64 / count as f64;
            k.push(t);
            k.push(t);
        }
        k.extend([1.0, 1.0, 1.0]);
        k
    }

    /// Map local `(p, q, h)` coordinates (in the tilted frame `e1, e2, n`)
    /// to world space.
    fn tilted_point(base: Point3, p: f64, q: f64, h: f64) -> Point3 {
        let (e1, e2) = plane_basis();
        base + e1 * p + e2 * q + tilted_axis() * h
    }

    /// An exact 315-degree cylindrical patch about [`tilted_axis`] through
    /// `base`, ruled linearly over `length`.
    fn exact_tilted_cylinder_patch(base: Point3, radius: f64, length: f64) -> NurbsSurface {
        let ring = arc_chain(7, 315.0);
        let grid: Vec<Vec<Point3>> = ring
            .iter()
            .map(|&(fx, fy, _)| {
                [0.0, length]
                    .iter()
                    .map(|&h| tilted_point(base, radius * fx, radius * fy, h))
                    .collect()
            })
            .collect();
        let weights = ring.iter().map(|&(_, _, w)| vec![w, w]).collect();
        NurbsSurface::new(
            2,
            1,
            arc_chain_knots(7),
            vec![0.0, 0.0, 1.0, 1.0],
            grid,
            weights,
        )
        .unwrap()
    }

    /// An exact spherical patch about `center`: a 315-degree revolution of a
    /// 140-degree meridian arc chain (latitudes -70..+70 degrees), built as a
    /// standard surface of revolution with product weights. Both directions
    /// use seven arcs, so every one of the 64 samples is an arc junction
    /// exactly on the sphere, and the junction latitudes are symmetric about
    /// the equator — the sample centroid is exactly the sphere centre.
    fn exact_tilted_sphere_patch(center: Point3, radius: f64) -> NurbsSurface {
        let ring = arc_chain(7, 315.0);
        // Meridian chain, rotated to start at -70 degrees latitude.
        let meridian = arc_chain(7, 140.0);
        let start = (-70.0_f64).to_radians();
        let (sin_s, cos_s) = start.sin_cos();
        let grid: Vec<Vec<Point3>> = ring
            .iter()
            .map(|&(fx, fy, _)| {
                meridian
                    .iter()
                    .map(|&(mc, ms, _)| {
                        // Rotate the meridian control point by `start`.
                        let rho = radius * (mc * cos_s - ms * sin_s);
                        let h = radius * (mc * sin_s + ms * cos_s);
                        tilted_point(center, rho * fx, rho * fy, h)
                    })
                    .collect()
            })
            .collect();
        let weights = ring
            .iter()
            .map(|&(_, _, wu)| meridian.iter().map(|&(_, _, wv)| wu * wv).collect())
            .collect();
        NurbsSurface::new(2, 2, arc_chain_knots(7), arc_chain_knots(7), grid, weights).unwrap()
    }

    // ── solve_3x3 ─────────────────────────────────────────────────────────

    /// `solve_3x3` must return the solution of `A x = b`. The fixture is a
    /// dense, non-symmetric integer system with `det = 77`, built backwards
    /// from `x = (3, -2, 4)` (`b = A x`, computed by hand), so every cofactor
    /// of Cramer's rule contributes to the answer.
    #[test]
    fn solve_3x3_recovers_a_known_dense_solution() {
        let a = [[2.0, -3.0, 1.0], [4.0, 1.0, -2.0], [-1.0, 5.0, 3.0]];
        let b = [16.0, 2.0, -1.0];
        let x = solve_3x3(a, b).expect("det = 77, so the system is non-singular");
        for (got, want) in x.iter().zip([3.0_f64, -2.0, 4.0].iter()) {
            assert!(
                (got - want).abs() < 1e-9,
                "solved {x:?}, expected [3, -2, 4]"
            );
        }
        // Residual check: the returned vector must satisfy the system.
        for r in 0..3 {
            let lhs = a[r][0] * x[0] + a[r][1] * x[1] + a[r][2] * x[2];
            assert!((lhs - b[r]).abs() < 1e-9, "row {r}: {lhs} != {}", b[r]);
        }
    }

    /// A singular system (row 2 = row 0 + row 1, `det = 0`) must be reported
    /// as unsolvable rather than divided through by zero.
    #[test]
    fn solve_3x3_rejects_a_singular_system() {
        let a = [[2.0, -3.0, 1.0], [4.0, 1.0, -2.0], [6.0, -2.0, -1.0]];
        assert!(solve_3x3(a, [16.0, 2.0, 18.0]).is_none());
    }

    // ── Plane recognition ─────────────────────────────────────────────────

    /// The recognized plane must satisfy `normal . p = d` at points on the
    /// surface — with a non-zero `d`, so a sign slip on either side of the
    /// equation shows up — and its normal must agree with du x dv for both
    /// control-grid layouts (opposed and agreeing).
    #[test]
    fn recognized_plane_off_origin_matches_the_analytic_plane() {
        let n = tilted_axis();
        let d_expected = n.dot(to_vec(plane_anchor()));
        assert!(
            d_expected.abs() > 1.0,
            "fixture must not pass through the origin"
        );

        for (label, surface) in [
            ("opposed", tilted_plane_patch_opposed()),
            ("agreeing", tilted_plane_patch_agreeing()),
        ] {
            let RecognizedSurface::Plane { normal, d } = recognize_surface(&surface, 1e-9) else {
                panic!("{label}: a planar patch must be recognized as a plane");
            };
            assert!(
                normal.dot(n) > 1.0 - 1e-9,
                "{label}: normal {normal:?} != analytic normal {n:?}"
            );
            assert!(
                (d - d_expected).abs() < 1e-9,
                "{label}: d {d} != normal . anchor {d_expected}"
            );
            let du_cross_dv = surface.normal(0.5, 0.5).unwrap();
            assert!(
                normal.dot(du_cross_dv) > 0.0,
                "{label}: recognized normal opposes du x dv"
            );
            // The plane equation must hold at an interior surface point too.
            let p = surface.evaluate(0.25, 0.75);
            assert!(
                (normal.dot(to_vec(p)) - d).abs() < 1e-9,
                "{label}: plane equation violated at an on-surface point"
            );
        }
    }

    // ── Cylinder recognition ──────────────────────────────────────────────

    /// A tilted, off-origin cylinder must be recovered exactly: the reported
    /// origin lies on the analytic axis line, the axis is parallel to it and
    /// runs along increasing v (the direction this estimator is documented to
    /// take), and the radius matches. `cylinder_to_nurbs` is exact, so all of
    /// this holds to round-off.
    #[test]
    fn recognize_tilted_off_origin_cylinder() {
        let origin = Point3::new(1.5, -2.5, 0.75);
        let axis = tilted_axis();
        let cyl = CylindricalSurface::new(origin, axis, 2.5).unwrap();
        let nurbs = cylinder_to_nurbs(&cyl, (0.0, 7.0)).unwrap();

        let RecognizedSurface::Cylinder {
            origin: got_origin,
            axis: got_axis,
            radius,
        } = recognize_surface(&nurbs, 1e-6)
        else {
            panic!("an exact cylinder must be recognized as a cylinder");
        };
        assert!((radius - 2.5).abs() < 1e-6, "radius {radius} != 2.5");
        assert!(
            got_axis.dot(axis) > 1.0 - 1e-9,
            "axis {got_axis:?} is not the analytic axis {axis:?}"
        );
        assert!(
            got_axis.dot(v_direction(&nurbs)) > 0.0,
            "axis {got_axis:?} does not follow increasing v"
        );
        let off = distance_to_axis(got_origin, origin, axis);
        assert!(off < 1e-6, "reported origin is {off} off the analytic axis");
    }

    /// The perpendicular-frame seed only picks `x` when the axis is not
    /// x-dominant; a cylinder whose axis IS `+x` must still be recognized.
    #[test]
    fn recognize_x_axis_cylinder() {
        let origin = Point3::new(0.5, -1.25, 2.0);
        let axis = Vec3::new(1.0, 0.0, 0.0);
        let cyl = CylindricalSurface::new(origin, axis, 1.75).unwrap();
        let nurbs = cylinder_to_nurbs(&cyl, (0.0, 6.0)).unwrap();

        let RecognizedSurface::Cylinder {
            origin: got_origin,
            axis: got_axis,
            radius,
        } = recognize_surface(&nurbs, 1e-6)
        else {
            panic!("an x-aligned cylinder must be recognized as a cylinder");
        };
        assert!((radius - 1.75).abs() < 1e-6, "radius {radius} != 1.75");
        assert!(got_axis.dot(axis).abs() > 1.0 - 1e-9, "axis {got_axis:?}");
        assert!(distance_to_axis(got_origin, origin, axis) < 1e-6);
    }

    /// The doc contract admits any `nu x nv` grid, not just the exact 9x2
    /// rational form. Inserting a v-knot leaves exactly the same cylinder with
    /// three control-point columns; it must still be recognized.
    #[test]
    fn recognize_cylinder_with_three_control_point_columns() {
        let origin = Point3::new(1.5, -2.5, 0.75);
        let axis = tilted_axis();
        let cyl = CylindricalSurface::new(origin, axis, 2.5).unwrap();
        let nurbs = cylinder_to_nurbs(&cyl, (0.0, 7.0)).unwrap();
        let refined = surface_knot_insert_v(&nurbs, 0.5, 1).unwrap();
        assert_eq!(refined.control_points()[0].len(), 3);

        let RecognizedSurface::Cylinder {
            origin: got_origin,
            radius,
            ..
        } = recognize_surface(&refined, 1e-6)
        else {
            panic!("a knot-refined cylinder must still be recognized as a cylinder");
        };
        assert!((radius - 2.5).abs() < 1e-6, "radius {radius} != 2.5");
        assert!(distance_to_axis(got_origin, origin, axis) < 1e-6);
    }

    // ── Sphere recognition ────────────────────────────────────────────────

    /// A sphere away from the origin, given as an exact rational patch, must
    /// recover its centre and radius to round-off.
    #[test]
    fn recognize_off_origin_sphere() {
        let center = Point3::new(2.5, -1.75, 3.25);
        let radius = 4.5;
        let nurbs = exact_tilted_sphere_patch(center, radius);

        let RecognizedSurface::Sphere {
            center: got_center,
            radius: got_radius,
        } = recognize_surface(&nurbs, 1e-6)
        else {
            panic!("an exact spherical patch must be recognized as a sphere");
        };
        let off = (got_center - center).length();
        assert!(off < 1e-6, "centre off by {off}");
        assert!(
            (got_radius - radius).abs() < 1e-6,
            "radius {got_radius} != {radius}"
        );
    }

    /// The sampled 33x9 sphere form must also be recognized, within the
    /// chord-height error its own docs state: at most `R * (1 - cos(pi/16))`
    /// over the eight spans across the v half-turn.
    #[test]
    fn recognize_sampled_off_origin_sphere_within_chord_error() {
        let center = Point3::new(2.5, -1.75, 3.25);
        let radius = 4.5;
        let chord_err = radius * (1.0 - (std::f64::consts::PI / 16.0).cos());
        let sphere = SphericalSurface::with_axis(center, radius, tilted_axis()).unwrap();
        let nurbs = sphere_to_nurbs(&sphere).unwrap();

        let RecognizedSurface::Sphere {
            center: got_center,
            radius: got_radius,
        } = recognize_surface(&nurbs, 2.0 * chord_err)
        else {
            panic!("a sampled sphere must be recognized as a sphere");
        };
        let off = (got_center - center).length();
        assert!(
            off < chord_err,
            "centre off by {off} (chord error {chord_err})"
        );
        assert!(
            (got_radius - radius).abs() < chord_err,
            "radius {got_radius} != {radius} within chord error {chord_err}"
        );
    }

    // ── Cone recognition ──────────────────────────────────────────────────

    /// A cone whose apex is off the origin and whose axis is tilted must
    /// recover apex, axis and half-angle. The axis is checked for direction as
    /// well as for line: the contract says it points from the apex INTO the
    /// cone.
    #[test]
    fn recognize_tilted_off_origin_cone() {
        let apex = Point3::new(1.5, -2.0, 0.5);
        let axis = tilted_axis();
        let half_angle = 0.35;
        let cone = ConicalSurface::new(apex, axis, half_angle).unwrap();
        let nurbs = cone_to_nurbs(&cone, (1.5, 5.0)).unwrap();

        let RecognizedSurface::Cone {
            apex: got_apex,
            axis: got_axis,
            half_angle: got_ha,
        } = recognize_surface(&nurbs, 0.05)
        else {
            panic!("a sampled cone must be recognized as a cone");
        };
        let off = (got_apex - apex).length();
        assert!(off < 0.05, "apex off by {off}");
        assert!(
            got_axis.dot(axis) > 1.0 - 1e-3,
            "axis {got_axis:?} != analytic axis {axis:?}"
        );
        assert!(
            got_axis.dot(sample_centroid(&nurbs) - got_apex) > 0.0,
            "axis {got_axis:?} points away from the cone body"
        );
        assert!(
            (got_ha - half_angle).abs() < 1e-3,
            "half_angle {got_ha} != {half_angle}"
        );
    }

    /// The same cone sampled with v DECREASING: the control-grid estimator now
    /// points at the apex, so the builder has to flip it. The returned axis
    /// must still run from the apex into the cone.
    #[test]
    fn recognize_cone_sampled_with_decreasing_v() {
        let apex = Point3::new(1.5, -2.0, 0.5);
        let axis = tilted_axis();
        let half_angle = 0.35;
        let cone = ConicalSurface::new(apex, axis, half_angle).unwrap();
        let nurbs = cone_to_nurbs(&cone, (5.0, 1.5)).unwrap();

        let RecognizedSurface::Cone {
            apex: got_apex,
            axis: got_axis,
            half_angle: got_ha,
        } = recognize_surface(&nurbs, 0.05)
        else {
            panic!("a reversed-v cone must still be recognized as a cone");
        };
        assert!((got_apex - apex).length() < 0.05, "apex {got_apex:?}");
        assert!(
            got_axis.dot(axis) > 1.0 - 1e-3,
            "axis {got_axis:?} != analytic axis {axis:?}"
        );
        assert!(
            got_axis.dot(sample_centroid(&nurbs) - got_apex) > 0.0,
            "axis {got_axis:?} points away from the cone body"
        );
        assert!((got_ha - half_angle).abs() < 1e-3, "half_angle {got_ha}");
    }

    // ── Torus recognition ─────────────────────────────────────────────────

    /// A tilted, off-origin torus must recover centre, axis and both radii.
    /// The sampled form's chord error is bounded by `minor * (1 - cos(pi/8))`
    /// in the minor direction (8 spans over the full minor turn).
    #[test]
    fn recognize_tilted_off_origin_torus() {
        let center = Point3::new(1.25, -2.5, 0.75);
        let axis = tilted_axis();
        let (major, minor) = (3.5, 0.9);
        let chord_err = minor * (1.0 - (std::f64::consts::PI / 8.0).cos());
        let torus = ToroidalSurface::with_axis(center, major, minor, axis).unwrap();
        let nurbs = torus_to_nurbs(&torus).unwrap();

        let RecognizedSurface::Torus {
            center: got_center,
            axis: got_axis,
            major_radius,
            minor_radius,
        } = recognize_surface(&nurbs, 3.0 * chord_err)
        else {
            panic!("a sampled torus must be recognized as a torus");
        };
        let off = (got_center - center).length();
        assert!(off < 3.0 * chord_err, "centre off by {off}");
        assert!(
            got_axis.dot(axis).abs() > 1.0 - 1e-6,
            "axis {got_axis:?} != analytic axis {axis:?}"
        );
        assert!(
            (major_radius - major).abs() < 3.0 * chord_err,
            "major_radius {major_radius} != {major}"
        );
        assert!(
            (minor_radius - minor).abs() < 3.0 * chord_err,
            "minor_radius {minor_radius} != {minor}"
        );
    }

    /// An exact toroidal patch about [`tilted_axis`] through `center`: a
    /// 315-degree revolution of a 315-degree tube arc chain, built as a
    /// surface of revolution with product weights. The tube arc starts at
    /// -100 degrees so the eight junction angles are NOT symmetric about the
    /// major-circle plane — the sample centroid then sits off that plane,
    /// which is what makes the fitted axial centre non-zero.
    fn exact_tilted_torus_patch(center: Point3, major: f64, minor: f64) -> NurbsSurface {
        // Full turn in u: the torus fit measures radial distance from the axis
        // through the SAMPLE CENTROID, which only coincides with the real axis
        // when the angular samples are balanced.
        let ring = arc_chain(8, 360.0);
        // 280 degrees over seven arcs: the eight tube junctions are then NOT a
        // closed regular polygon, so their heights do not cancel and the
        // fitted axial centre is genuinely non-zero.
        let tube = arc_chain(7, 280.0);
        let start = (-100.0_f64).to_radians();
        let (sin_s, cos_s) = start.sin_cos();
        let grid: Vec<Vec<Point3>> = ring
            .iter()
            .map(|&(fx, fy, _)| {
                tube.iter()
                    .map(|&(tc, ts, _)| {
                        let rho = minor.mul_add(tc * cos_s - ts * sin_s, major);
                        let h = minor * (tc * sin_s + ts * cos_s);
                        tilted_point(center, rho * fx, rho * fy, h)
                    })
                    .collect()
            })
            .collect();
        let weights = ring
            .iter()
            .map(|&(_, _, wu)| tube.iter().map(|&(_, _, wv)| wu * wv).collect())
            .collect();
        NurbsSurface::new(2, 2, arc_chain_knots(8), arc_chain_knots(7), grid, weights).unwrap()
    }

    /// An exact conical frustum patch about [`tilted_axis`] from `apex`, over
    /// generator distances `v_range`, as a full-turn ring of exact rational
    /// arcs ruled linearly along the generators. Only TWO control-point
    /// columns — the documented `nu x nv` contract admits that, and it is the
    /// minimum a ruled surface needs.
    fn exact_tilted_cone_patch(apex: Point3, half_angle: f64, v_range: (f64, f64)) -> NurbsSurface {
        let ring = arc_chain(8, 360.0);
        let (sin_a, cos_a) = half_angle.sin_cos();
        let grid: Vec<Vec<Point3>> = ring
            .iter()
            .map(|&(fx, fy, _)| {
                [v_range.0, v_range.1]
                    .iter()
                    .map(|&v| {
                        let r = v * cos_a;
                        tilted_point(apex, r * fx, r * fy, v * sin_a)
                    })
                    .collect()
            })
            .collect();
        let weights = ring.iter().map(|&(_, _, w)| vec![w, w]).collect();
        NurbsSurface::new(
            2,
            1,
            arc_chain_knots(8),
            vec![0.0, 0.0, 1.0, 1.0],
            grid,
            weights,
        )
        .unwrap()
    }

    /// A cone given exactly, on the minimum two-column ruled grid, must be
    /// recovered to round-off.
    #[test]
    fn recognize_exact_cone_patch_on_a_two_column_grid() {
        let apex = Point3::new(1.5, -2.0, 0.5);
        let axis = tilted_axis();
        let half_angle = 0.35;
        let nurbs = exact_tilted_cone_patch(apex, half_angle, (1.5, 5.0));
        assert_eq!(nurbs.control_points()[0].len(), 2);

        let RecognizedSurface::Cone {
            apex: got_apex,
            axis: got_axis,
            half_angle: got_ha,
        } = recognize_surface(&nurbs, 1e-6)
        else {
            panic!("an exact conical patch must be recognized as a cone");
        };
        let off = (got_apex - apex).length();
        assert!(off < 1e-6, "apex off by {off}");
        assert!(got_axis.dot(axis) > 1.0 - 1e-9, "axis {got_axis:?}");
        assert!(
            got_axis.dot(sample_centroid(&nurbs) - got_apex) > 0.0,
            "axis {got_axis:?} points away from the cone body"
        );
        assert!(
            (got_ha - half_angle).abs() < 1e-9,
            "half_angle {got_ha} != {half_angle}"
        );
    }

    /// The exact cylinder patch starts its u-parameterization from `e1`, which
    /// is not the perpendicular-frame seed the recognizer builds for itself.
    /// That makes BOTH least-squares coordinates of the fitted centre non-zero
    /// — the full-turn form produced by `cylinder_to_nurbs` happens to give one
    /// of them as zero, which hides a slip in either the design matrix or the
    /// centre reconstruction.
    #[test]
    fn recognize_exact_cylinder_patch_with_both_centre_coordinates_non_zero() {
        let base = Point3::new(1.5, -2.5, 0.75);
        let axis = tilted_axis();
        let radius = 2.5;
        let nurbs = exact_tilted_cylinder_patch(base, radius, 7.0);

        let RecognizedSurface::Cylinder {
            origin: got_origin,
            axis: got_axis,
            radius: got_radius,
        } = recognize_surface(&nurbs, 1e-6)
        else {
            panic!("an exact cylindrical patch must be recognized as a cylinder");
        };
        assert!(
            (got_radius - radius).abs() < 1e-6,
            "radius {got_radius} != {radius}"
        );
        assert!(got_axis.dot(axis) > 1.0 - 1e-9, "axis {got_axis:?}");
        let off = distance_to_axis(got_origin, base, axis);
        assert!(off < 1e-6, "reported origin is {off} off the analytic axis");
    }

    /// An exact toroidal patch, recovered to round-off. Unlike the sampled
    /// full torus, its samples are not symmetric about the major-circle plane,
    /// so the fitted axial centre is genuinely non-zero and the radial
    /// projection has to be right rather than merely close.
    #[test]
    fn recognize_exact_torus_patch_off_the_major_circle_plane() {
        let center = Point3::new(1.25, -2.5, 0.75);
        let axis = tilted_axis();
        let (major, minor) = (3.5, 0.9);
        let nurbs = exact_tilted_torus_patch(center, major, minor);

        let RecognizedSurface::Torus {
            center: got_center,
            axis: got_axis,
            major_radius,
            minor_radius,
        } = recognize_surface(&nurbs, 1e-6)
        else {
            panic!("an exact toroidal patch must be recognized as a torus");
        };
        let off = (got_center - center).length();
        assert!(off < 1e-6, "centre off by {off}");
        assert!(got_axis.dot(axis).abs() > 1.0 - 1e-9, "axis {got_axis:?}");
        assert!(
            (major_radius - major).abs() < 1e-6,
            "major_radius {major_radius} != {major}"
        );
        assert!(
            (minor_radius - minor).abs() < 1e-6,
            "minor_radius {minor_radius} != {minor}"
        );
    }

    // ── Rejection ─────────────────────────────────────────────────────────

    /// A free-form saddle is none of the elementary forms; every recognizer
    /// must decline it.
    #[test]
    fn free_form_patch_is_not_recognized() {
        assert_eq!(
            recognize_surface(&free_form_patch(), 1e-6),
            RecognizedSurface::NotRecognized
        );
    }

    /// A sphere must not be mistaken for a cylinder, a cone or a torus, even
    /// at the loose tolerance its sampled form needs.
    #[test]
    fn sphere_is_not_recognized_as_cylinder_cone_or_torus() {
        let sphere =
            SphericalSurface::with_axis(Point3::new(2.5, -1.75, 3.25), 4.5, tilted_axis()).unwrap();
        let nurbs = sphere_to_nurbs(&sphere).unwrap();
        assert!(matches!(
            recognize_surface(&nurbs, 0.2),
            RecognizedSurface::Sphere { .. }
        ));
    }

    /// A torus must not be mistaken for a sphere or a cylinder.
    #[test]
    fn torus_is_not_recognized_as_sphere() {
        let torus =
            ToroidalSurface::with_axis(Point3::new(1.25, -2.5, 0.75), 3.5, 0.9, tilted_axis())
                .unwrap();
        let nurbs = torus_to_nurbs(&torus).unwrap();
        assert!(matches!(
            recognize_surface(&nurbs, 0.2),
            RecognizedSurface::Torus { .. }
        ));
    }

    // ── Lightweight detection ─────────────────────────────────────────────

    #[test]
    fn detected_surface_kind_tags() {
        assert_eq!(DetectedSurfaceKind::Plane.as_str(), "plane");
        assert_eq!(DetectedSurfaceKind::Sphere.as_str(), "sphere");
        assert_eq!(DetectedSurfaceKind::Cylinder.as_str(), "cylinder");
        assert_eq!(DetectedSurfaceKind::BSpline.as_str(), "bspline");
    }

    #[test]
    fn detect_surface_kind_of_tilted_off_origin_plane() {
        assert_eq!(
            detect_surface_kind(&tilted_plane_patch_opposed()),
            DetectedSurfaceKind::Plane
        );
        assert_eq!(
            detect_surface_kind(&tilted_plane_patch_agreeing()),
            DetectedSurfaceKind::Plane
        );
    }

    #[test]
    fn detect_surface_kind_of_off_origin_sphere() {
        // The heuristic's sphere test allows only 0.1% relative spread, so it
        // needs the exact rational patch: the sampled 33x9 form carries ~2%
        // chord error and is correctly reported as `BSpline`.
        let nurbs = exact_tilted_sphere_patch(Point3::new(2.5, -1.75, 3.25), 4.5);
        assert_eq!(detect_surface_kind(&nurbs), DetectedSurfaceKind::Sphere);
    }

    /// The cylinder branch depends on the PCA axis estimate, which only finds
    /// the axis when the axial spread beats the radial spread — hence a length
    /// well above `r * sqrt(6)`. Tilted and off-origin so that a centroid slip
    /// cannot cancel out.
    #[test]
    fn detect_surface_kind_of_tilted_off_origin_cylinder() {
        let nurbs = exact_tilted_cylinder_patch(Point3::new(1.5, -2.5, 0.75), 2.5, 12.0);
        assert_eq!(detect_surface_kind(&nurbs), DetectedSurfaceKind::Cylinder);
    }

    #[test]
    fn detect_surface_kind_of_free_form_patch_is_bspline() {
        assert_eq!(
            detect_surface_kind(&free_form_patch()),
            DetectedSurfaceKind::BSpline
        );
    }

    /// A cone is neither a sphere nor a cylinder: its sample distances vary
    /// with height under both tests, so the heuristic must fall back to
    /// `BSpline`.
    #[test]
    fn detect_surface_kind_of_cone_is_bspline() {
        let cone = ConicalSurface::new(Point3::new(1.5, -2.0, 0.5), tilted_axis(), 0.35).unwrap();
        let nurbs = cone_to_nurbs(&cone, (1.5, 5.0)).unwrap();
        assert_eq!(detect_surface_kind(&nurbs), DetectedSurfaceKind::BSpline);
    }
}
