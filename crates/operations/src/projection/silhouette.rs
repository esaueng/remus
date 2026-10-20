//! Silhouette (outline) curves of curved faces in an orthographic view.
//!
//! A silhouette is where a face's normal is perpendicular to the view
//! direction: the line a drawing shows at the side of a cylinder, a cone or a
//! sphere or a torus, which no topology edge carries. For the analytic
//! surfaces the curve has a closed form — two rulings of a cylinder or cone,
//! a great circle of a sphere, the two branches `v(u)` of a torus — and is
//! clipped to the face's trims, then sampled and split into visible and
//! hidden runs with the same occlusion probe as [`super::project_edges`].
//! NURBS faces have no closed form here; they are reported in
//! [`ProjectedSilhouettes::unsupported`] rather than approximated.

use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use super::{View, occlusion_step, push_classified_runs};

/// Bound on trim-test samples over all silhouette curves of one solid, each
/// counted once per face it is tested against: every one is a
/// face-containment query.
const MAX_SILHOUETTE_SAMPLES: usize = 100_000;

/// Fewest samples along one silhouette curve, before trim refinement.
const MIN_CURVE_SAMPLES: usize = 16;

/// Bisection steps that place a trim crossing between two samples: 2⁻⁴⁰ of
/// the sample spacing, far below any drawing tolerance.
const TRIM_BISECTIONS: usize = 40;

/// Bound on the trim queries spent refining crossings, which the sample
/// preflight cannot count: 2 500 crossings on single faces, each
/// [`TRIM_BISECTIONS`] queries per face of the curve's surface.
const MAX_TRIM_REFINEMENT_QUERIES: usize = 100_000;

/// Below this, the view is taken to run along a cylinder's or cone's axis,
/// where its outline is its rims and no ruling is a silhouette, or to lie
/// in a torus's equatorial plane, where its outline is four circles.
const AXIAL_VIEW_TOLERANCE: f64 = 1e-12;

/// Halvings of one sample interval while refining a torus silhouette: enough
/// to follow a near-side view's turn through a meridian, which is about
/// `2·|axis·view|/|view in plane|` wide in `u`, down to that tolerance.
const MAX_TORUS_REFINEMENT_DEPTH: u32 = 48;

/// Silhouette curves projected onto the view plane, split like
/// [`super::ProjectedEdges`] into visible and hidden 2D polylines.
#[derive(Debug, Clone, Default)]
pub struct ProjectedSilhouettes {
    /// Silhouette polylines not occluded by the solid.
    pub visible: Vec<Vec<Point2>>,
    /// Silhouette polylines hidden behind the solid (empty when
    /// `hidden_lines` is false).
    pub hidden: Vec<Vec<Point2>>,
    /// Curved faces whose silhouette this function does not compute.
    pub unsupported: Vec<UnsupportedSilhouette>,
}

/// A curved face left out of [`ProjectedSilhouettes`], named so a drawing can
/// say its outline is incomplete instead of showing a silently partial one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnsupportedSilhouette {
    /// The face.
    pub face: FaceId,
    /// Its surface type tag (`"nurbs"`).
    pub surface: &'static str,
}

/// A silhouette curve on one face, parameterized by `t ∈ [0, 1]`.
pub(super) enum Curve {
    /// `start + dir·t`.
    Segment { start: Point3, dir: Vec3 },
    /// `center + radius·(cos 2πt·u + sin 2πt·v)`, closed.
    Circle {
        center: Point3,
        radius: f64,
        u: Vec3,
        v: Vec3,
    },
    /// The torus points where `cos v·(radial(u)·d) + sin v·(axis·d) = 0`:
    /// with `u = phi + 2πt` and `h·cos(2πt) = radial(u)·d`, the branch
    /// `v = atan2(−h·cos 2πt, az) + offset`, `offset` 0 or π. Closed, and
    /// continuous for `az ≠ 0`.
    Torus {
        center: Point3,
        x: Vec3,
        y: Vec3,
        z: Vec3,
        major: f64,
        minor: f64,
        phi: f64,
        h: f64,
        az: f64,
        offset: f64,
    },
}

impl Curve {
    pub(super) fn at(&self, t: f64) -> Point3 {
        match *self {
            Self::Segment { start, dir } => start + dir * t,
            Self::Circle {
                center,
                radius,
                u,
                v,
            } => {
                let (sin, cos) = (std::f64::consts::TAU * t).sin_cos();
                center + (u * cos + v * sin) * radius
            }
            Self::Torus {
                center,
                x,
                y,
                z,
                major,
                minor,
                phi,
                h,
                az,
                offset,
            } => {
                let w = std::f64::consts::TAU * t;
                let (sin_u, cos_u) = (phi + w).sin_cos();
                let (sin_v, cos_v) = ((-h * w.cos()).atan2(az) + offset).sin_cos();
                center + (x * cos_u + y * sin_u) * (minor * cos_v + major) + z * (minor * sin_v)
            }
        }
    }

    const fn is_closed(&self) -> bool {
        matches!(self, Self::Circle { .. } | Self::Torus { .. })
    }

    /// The parameters to sample, sorted from 0 to 1, or `None` when there
    /// would be more than `limit` intervals. Segments and circles are
    /// sampled uniformly at [`Self::sample_count`]; a torus branch starts
    /// from its outer equator's spacing and halves every interval whose
    /// chord strays more than `deflection` from the curve at its middle, or
    /// is longer than a chord of the tube at that deflection.
    pub(super) fn sample_ts(&self, deflection: f64, limit: usize) -> Option<Vec<f64>> {
        let count = self.sample_count(deflection);
        if count > limit {
            return None;
        }
        let uniform = |i: usize| i as f64 / count as f64;
        let Self::Torus { minor, .. } = *self else {
            return Some((0..=count).map(uniform).collect());
        };
        // The tube chord at sagitta `d` grows to the diameter at `d = r`; past
        // that it would shrink again, and from `d = 2r` the whole tube is
        // within the deflection, so the bound is dropped.
        let max_chord = if deflection >= 2.0 * minor {
            f64::INFINITY
        } else {
            let sagitta = deflection.min(minor);
            2.0 * (sagitta * (2.0 * minor - sagitta)).sqrt()
        };
        let mut ts = vec![0.0];
        let mut start = self.at(0.0);
        for i in 1..=count {
            let (a, b) = (uniform(i - 1), uniform(i));
            let end = self.at(b);
            self.refine(a, start, b, end, deflection, max_chord, 0, &mut ts, limit)?;
            start = end;
        }
        Some(ts)
    }

    /// Push the samples after `a` up to and including `b`, halving
    /// `[a, b]` while its chord is too far from the curve or too long.
    #[allow(clippy::too_many_arguments)]
    fn refine(
        &self,
        a: f64,
        pa: Point3,
        b: f64,
        pb: Point3,
        deflection: f64,
        max_chord: f64,
        depth: u32,
        ts: &mut Vec<f64>,
        limit: usize,
    ) -> Option<()> {
        let mid = 0.5 * (a + b);
        let pm = self.at(mid);
        let chord_mid = Point3::new(
            0.5 * (pa.x() + pb.x()),
            0.5 * (pa.y() + pb.y()),
            0.5 * (pa.z() + pb.z()),
        );
        let coarse = (pm - chord_mid).length() > deflection || (pb - pa).length() > max_chord;
        if coarse && depth < MAX_TORUS_REFINEMENT_DEPTH {
            self.refine(a, pa, mid, pm, deflection, max_chord, depth + 1, ts, limit)?;
            self.refine(mid, pm, b, pb, deflection, max_chord, depth + 1, ts, limit)?;
        } else {
            if ts.len() > limit {
                return None;
            }
            ts.push(b);
        }
        Some(())
    }

    /// Uniform samples for the trim test and the occlusion split: one per
    /// `deflection` of length on a segment, chord-deflection spacing on a
    /// circle, and on a torus branch the spacing of its outer equator.
    /// Unbounded above: the solid's sample budget refuses a count too large.
    fn sample_count(&self, deflection: f64) -> usize {
        let circle = |radius: f64| {
            let ratio = (1.0 - deflection / radius).clamp(-1.0, 1.0);
            std::f64::consts::TAU / (2.0 * ratio.acos())
        };
        let raw = match *self {
            Self::Segment { dir, .. } => dir.length() / deflection,
            Self::Circle { radius, .. } => circle(radius),
            Self::Torus { major, minor, .. } => circle(major + minor),
        };
        // A float-to-int cast saturates: an infinite count is usize::MAX.
        (raw.ceil() as usize).max(MIN_CURVE_SAMPLES)
    }
}

/// Project the silhouettes of a solid's curved faces onto a view plane.
///
/// The plane passes through `origin` with in-plane x-axis `x_axis`, viewed
/// along `direction` (orthographic); each segment is classified as visible
/// or hidden.
///
/// The view conventions and `hidden_lines` match [`super::project_edges`];
/// edges and silhouettes together make the outline drawing. Cylinder, cone
/// and sphere silhouettes are exact curves clipped to the face's trims, with
/// clip points refined by bisection so a silhouette meets the rim edge it
/// ends on, as exactly as the trim test places that edge: it samples
/// curved trims, so a clip point on one carries that sampling's error
/// (1.04e-4 on the notched-torus oracle). Torus silhouettes are their exact
/// branches sampled adaptively to `deflection`. Planes have none. NURBS faces are listed in
/// [`ProjectedSilhouettes::unsupported`]. `deflection` sets the sampling of
/// each curve, which also bounds how small a trim feature crossing a
/// silhouette is resolved.
///
/// # Errors
///
/// Returns [`crate::OperationsError::InvalidInput`] if `direction` or `x_axis`
/// is degenerate, `deflection` is invalid, or the silhouettes need more
/// trim-test samples, or cross their faces' trims more often, than the
/// projection work limits. Propagates topology, trim-classification and
/// point-classification errors.
pub fn project_silhouettes(
    topo: &Topology,
    solid: SolidId,
    origin: Point3,
    direction: Vec3,
    x_axis: Vec3,
    hidden_lines: bool,
    deflection: f64,
) -> Result<ProjectedSilhouettes, crate::OperationsError> {
    silhouettes_within(
        topo,
        solid,
        origin,
        direction,
        x_axis,
        hidden_lines,
        deflection,
        MAX_TRIM_REFINEMENT_QUERIES,
    )
}

/// [`project_silhouettes`] with `refinement_queries` trim queries to spend
/// placing crossings.
#[allow(clippy::too_many_arguments)]
pub(super) fn silhouettes_within(
    topo: &Topology,
    solid: SolidId,
    origin: Point3,
    direction: Vec3,
    x_axis: Vec3,
    hidden_lines: bool,
    deflection: f64,
    mut refinement_queries: usize,
) -> Result<ProjectedSilhouettes, crate::OperationsError> {
    if !deflection.is_finite() || deflection <= 0.0 {
        return Err(crate::OperationsError::InvalidInput {
            reason: "projection deflection must be finite and positive".into(),
        });
    }
    let view = View::new(origin, direction, x_axis)?;

    // Faces on one support surface share its silhouette: two hemispheres
    // share the equator, two half-cylinders a ruling along their split
    // edge. Each surface's curves are trimmed once, against all its faces,
    // so a curve on a boundary between them is drawn once.
    let mut result = ProjectedSilhouettes::default();
    let mut carriers: Vec<(&FaceSurface, Vec<FaceId>)> = Vec::new();
    for face in remus_topology::explorer::solid_faces(topo, solid)? {
        let surface = topo.face(face)?.surface();
        let unsupported = match surface {
            FaceSurface::Plane { .. } => continue,
            FaceSurface::Nurbs(_) => "nurbs",
            FaceSurface::Sphere(_)
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Torus(_) => {
                match carriers
                    .iter_mut()
                    .find(|(known, _)| same_carrier(known, surface))
                {
                    Some((_, faces)) => faces.push(face),
                    None => carriers.push((surface, vec![face])),
                }
                continue;
            }
        };
        result.unsupported.push(UnsupportedSilhouette {
            face,
            surface: unsupported,
        });
    }

    // Every curve is built and counted before any trim query runs, so an
    // over-budget request is refused before the expensive work.
    let mut curves: Vec<(&[FaceId], Curve, Vec<f64>)> = Vec::new();
    let mut samples = 0usize;
    for (surface, faces) in &carriers {
        for curve in carrier_silhouettes(topo, surface, faces, view.dir)? {
            let limit = (MAX_SILHOUETTE_SAMPLES - samples) / faces.len().max(1);
            let ts = curve.sample_ts(deflection, limit).ok_or_else(|| {
                crate::OperationsError::InvalidInput {
                    reason: format!(
                        "silhouette sampling exceeds the {MAX_SILHOUETTE_SAMPLES}-point limit; increase deflection"
                    ),
                }
            })?;
            samples += (ts.len() - 1) * faces.len();
            curves.push((faces, curve, ts));
        }
    }
    if curves.is_empty() {
        return Ok(result);
    }

    // Each segment is probed at the curve point halfway along it, not at its
    // chord's midpoint as for edges: a circle's sagitta would put that
    // inside a sphere, and the step toward the camera, tangent to the
    // surface here, would not leave it.
    let eps = occlusion_step(topo, solid)?;
    for (faces, curve, ts) in &curves {
        for run in trimmed_runs(topo, faces, curve, ts, &mut refinement_queries)? {
            let points: Vec<Point3> = run.iter().map(|&t| curve.at(t)).collect();
            let seg_hidden = run
                .windows(2)
                .map(|pair| {
                    view.is_hidden(
                        topo,
                        solid,
                        eps,
                        deflection,
                        curve.at(0.5 * (pair[0] + pair[1])),
                    )
                })
                .collect::<Result<Vec<bool>, _>>()?;
            push_classified_runs(
                &points,
                &seg_hidden,
                &view,
                hidden_lines,
                &mut result.visible,
                &mut result.hidden,
            );
        }
    }
    Ok(result)
}

/// Positions and radii closer than this are one carrier: the kernel's
/// default linear tolerance.
const SAME_CARRIER_LINEAR: f64 = 1e-7;

/// Axis directions and half-angles closer than this (radians, or the sine
/// of the angle between axes) are one carrier.
const SAME_CARRIER_ANGULAR: f64 = 1e-9;

/// Whether two support surfaces are the same sphere, cylinder, cone or
/// torus, so have the same silhouette in every view. A cylinder's frame and
/// origin along its axis, and a cylinder's or torus's axis sense, do not
/// change it.
fn same_carrier(a: &FaceSurface, b: &FaceSurface) -> bool {
    let parallel = |p: Vec3, q: Vec3| p.cross(q).length() <= SAME_CARRIER_ANGULAR;
    if let (FaceSurface::Sphere(a), FaceSurface::Sphere(b)) = (a, b) {
        return (a.center() - b.center()).length() <= SAME_CARRIER_LINEAR
            && (a.radius() - b.radius()).abs() <= SAME_CARRIER_LINEAR;
    }
    if let (FaceSurface::Cylinder(a), FaceSurface::Cylinder(b)) = (a, b) {
        let offset = b.origin() - a.origin();
        let off_axis = offset - a.axis() * offset.dot(a.axis());
        return parallel(a.axis(), b.axis())
            && off_axis.length() <= SAME_CARRIER_LINEAR
            && (a.radius() - b.radius()).abs() <= SAME_CARRIER_LINEAR;
    }
    if let (FaceSurface::Cone(a), FaceSurface::Cone(b)) = (a, b) {
        return (a.apex() - b.apex()).length() <= SAME_CARRIER_LINEAR
            && a.axis().dot(b.axis()) > 0.0
            && parallel(a.axis(), b.axis())
            && (a.half_angle() - b.half_angle()).abs() <= SAME_CARRIER_ANGULAR;
    }
    if let (FaceSurface::Torus(a), FaceSurface::Torus(b)) = (a, b) {
        return (a.center() - b.center()).length() <= SAME_CARRIER_LINEAR
            && parallel(a.z_axis(), b.z_axis())
            && (a.major_radius() - b.major_radius()).abs() <= SAME_CARRIER_LINEAR
            && (a.minor_radius() - b.minor_radius()).abs() <= SAME_CARRIER_LINEAR;
    }
    false
}

/// The silhouette curves of a support surface, untrimmed and long enough to
/// cover all of `faces`: up to four curves. Planes and NURBS surfaces have
/// none here.
pub(super) fn carrier_silhouettes(
    topo: &Topology,
    surface: &FaceSurface,
    faces: &[FaceId],
    view: Vec3,
) -> Result<Vec<Curve>, crate::OperationsError> {
    Ok(match surface {
        FaceSurface::Plane { .. } | FaceSurface::Nurbs(_) => Vec::new(),
        FaceSurface::Torus(torus) => {
            // Normal at (u, v) is radial(u)·cos v + axis·sin v, so the
            // silhouette is cos v·(radial(u)·d) + sin v·(axis·d) = 0.
            let (x, y, z) = (torus.x_axis(), torus.y_axis(), torus.z_axis());
            let (dx, dy, az) = (x.dot(view), y.dot(view), z.dot(view));
            let (center, major, minor) =
                (torus.center(), torus.major_radius(), torus.minor_radius());
            let phi = dy.atan2(dx);
            if az.abs() <= AXIAL_VIEW_TOLERANCE {
                // A view in the equatorial plane: the condition holds all
                // round the top and bottom circles (v = ±π/2), and all round
                // the two meridians whose plane holds the view
                // (radial(u)·d = 0).
                let mut curves: Vec<Curve> = [minor, -minor]
                    .into_iter()
                    .map(|height| Curve::Circle {
                        center: center + z * height,
                        radius: major,
                        u: x,
                        v: y,
                    })
                    .collect();
                for u in [
                    phi + std::f64::consts::FRAC_PI_2,
                    phi - std::f64::consts::FRAC_PI_2,
                ] {
                    let (sin, cos) = u.sin_cos();
                    let radial = x * cos + y * sin;
                    curves.push(Curve::Circle {
                        center: center + radial * major,
                        radius: minor,
                        u: radial,
                        v: z,
                    });
                }
                curves
            } else {
                let h = dx.hypot(dy);
                [0.0, std::f64::consts::PI]
                    .into_iter()
                    .map(|offset| Curve::Torus {
                        center,
                        x,
                        y,
                        z,
                        major,
                        minor,
                        phi,
                        h,
                        az,
                        offset,
                    })
                    .collect()
            }
        }
        FaceSurface::Sphere(sphere) => {
            // The great circle in the plane through the centre normal to the view.
            let (u, v) = plane_basis(view)?;
            vec![Curve::Circle {
                center: sphere.center(),
                radius: sphere.radius(),
                u,
                v,
            }]
        }
        FaceSurface::Cylinder(cylinder) => {
            // Normal at u is x·cos u + y·sin u: perpendicular to the view at
            // u₀ = atan2(−x·d, y·d) and u₀ + π.
            let (dx, dy) = (cylinder.x_axis().dot(view), cylinder.y_axis().dot(view));
            if dx.hypot(dy) <= AXIAL_VIEW_TOLERANCE {
                return Ok(Vec::new());
            }
            let u0 = (-dx).atan2(dy);
            let axis = cylinder.axis();
            let (v_min, v_max) = axial_range(topo, faces, cylinder.origin(), axis, 1.0)?;
            [u0, u0 + std::f64::consts::PI]
                .into_iter()
                .map(|u| {
                    let (sin, cos) = u.sin_cos();
                    let foot = cylinder.origin()
                        + (cylinder.x_axis() * cos + cylinder.y_axis() * sin) * cylinder.radius();
                    Curve::Segment {
                        start: foot + axis * v_min,
                        dir: axis * (v_max - v_min),
                    }
                })
                .collect()
        }
        FaceSurface::Cone(cone) => {
            // Normal at u is radial(u)·sin a − axis·cos a: perpendicular to
            // the view where cos(u − φ) = (axis·d)·cot a / hypot(x·d, y·d).
            let (dx, dy) = (cone.x_axis().dot(view), cone.y_axis().dot(view));
            let reach = dx.hypot(dy);
            if reach <= AXIAL_VIEW_TOLERANCE {
                return Ok(Vec::new());
            }
            let (sin_a, cos_a) = cone.half_angle().sin_cos();
            let c = cone.axis().dot(view) * cos_a / (sin_a * reach);
            // At |c| = 1 the two rulings meet in the one generator that runs
            // along the view: it projects to a point, so there is no outline.
            if c.abs() >= 1.0 - AXIAL_VIEW_TOLERANCE {
                return Ok(Vec::new());
            }
            let phi = dy.atan2(dx);
            let spread = c.acos();
            let axis = cone.axis();
            // Generator distance v is axial height / sin a.
            let (v_min, v_max) = axial_range(topo, faces, cone.apex(), axis, sin_a)?;
            [phi - spread, phi + spread]
                .into_iter()
                .map(|u| {
                    let (sin, cos) = u.sin_cos();
                    let generator =
                        (cone.x_axis() * cos + cone.y_axis() * sin) * cos_a + axis * sin_a;
                    Curve::Segment {
                        start: cone.apex() + generator * v_min,
                        dir: generator * (v_max - v_min),
                    }
                })
                .collect()
        }
    })
}

/// An orthonormal basis of the plane normal to `normal`.
fn plane_basis(normal: Vec3) -> Result<(Vec3, Vec3), crate::OperationsError> {
    let seed = if normal.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u = normal.cross(seed).normalize()?;
    Ok((u, normal.cross(u)))
}

/// A parameter range along `axis` from `base` that contains the faces: the
/// axial heights of their bounding boxes' corners, divided by `scale` (1 for a
/// cylinder, sin a for a cone's generator distance), padded slightly so a
/// rim lying exactly on a corner height is inside. Trimming then cuts it to
/// the faces exactly.
fn axial_range(
    topo: &Topology,
    faces: &[FaceId],
    base: Point3,
    axis: Vec3,
    scale: f64,
) -> Result<(f64, f64), crate::OperationsError> {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for &face in faces {
        let aabb = remus_check::util::face_aabb(topo, face)?;
        for corner in 0..8u8 {
            let p = Point3::new(
                if corner & 1 == 0 {
                    aabb.min.x()
                } else {
                    aabb.max.x()
                },
                if corner & 2 == 0 {
                    aabb.min.y()
                } else {
                    aabb.max.y()
                },
                if corner & 4 == 0 {
                    aabb.min.z()
                } else {
                    aabb.max.z()
                },
            );
            let v = (p - base).dot(axis) / scale;
            lo = lo.min(v);
            hi = hi.max(v);
        }
    }
    let pad = (hi - lo).abs().max(1.0) * 1e-9;
    Ok((lo - pad, hi + pad))
}

/// Parameter runs of `curve` inside any of `faces`, each a sorted list of `t`
/// values starting and ending on a trim crossing (or the curve's ends).
/// Each crossing spends [`TRIM_BISECTIONS`] queries per face of
/// `refinement_queries`.
fn trimmed_runs(
    topo: &Topology,
    faces: &[FaceId],
    curve: &Curve,
    ts: &[f64],
    refinement_queries: &mut usize,
) -> Result<Vec<Vec<f64>>, crate::OperationsError> {
    let inside = |t: f64| -> Result<bool, crate::OperationsError> {
        let p = curve.at(t);
        for &face in faces {
            if remus_check::classify::surface_point_in_face(topo, face, p)? {
                return Ok(true);
            }
        }
        Ok(false)
    };
    let flags = ts
        .iter()
        .map(|&t| inside(t))
        .collect::<Result<Vec<bool>, _>>()?;
    // The point between an inside and an outside sample where the trim lies.
    let mut crossing = |mut a: f64,
                        mut b: f64,
                        a_inside: bool|
     -> Result<f64, crate::OperationsError> {
        *refinement_queries = refinement_queries
            .checked_sub(TRIM_BISECTIONS.saturating_mul(faces.len()))
            .ok_or_else(|| crate::OperationsError::InvalidInput {
                reason:
                    "silhouettes cross their faces' trims more often than the projection work limit"
                        .into(),
            })?;
        for _ in 0..TRIM_BISECTIONS {
            let mid = 0.5 * (a + b);
            if inside(mid)? == a_inside {
                a = mid;
            } else {
                b = mid;
            }
        }
        Ok(0.5 * (a + b))
    };

    let mut runs: Vec<Vec<f64>> = Vec::new();
    let mut current: Option<Vec<f64>> = None;
    for i in 0..ts.len() {
        if i > 0 && flags[i] != flags[i - 1] {
            let edge = crossing(ts[i - 1], ts[i], flags[i - 1])?;
            if flags[i] {
                current = Some(vec![edge]);
            } else if let Some(mut run) = current.take() {
                run.push(edge);
                runs.push(run);
            }
        }
        if flags[i] {
            current.get_or_insert_with(Vec::new).push(ts[i]);
        }
    }
    if let Some(run) = current {
        runs.push(run);
    }

    // A closed curve's run through t = 0 is split across both ends: join it.
    if curve.is_closed() && runs.len() > 1 && flags[0] && flags[ts.len() - 1] {
        let head = runs.remove(0);
        if let Some(tail) = runs.last_mut() {
            // t = 1 is t = 0: continue past 1 rather than jump back.
            tail.extend(head.into_iter().skip(1).map(|t| t + 1.0));
        }
    }
    Ok(runs.into_iter().filter(|run| run.len() >= 2).collect())
}
