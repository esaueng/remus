//! Silhouette (outline) curves of curved faces in an orthographic view.
//!
//! A silhouette is where a face's normal is perpendicular to the view
//! direction: the line a drawing shows at the side of a cylinder, a cone or a
//! sphere, which no topology edge carries. For the analytic surfaces with a
//! closed form the curve is exact — two rulings of a cylinder or cone, a
//! great circle of a sphere — and is clipped to the face's trims, then
//! sampled and split into visible and hidden runs with the same occlusion
//! probe as [`super::project_edges`]. Tori and NURBS faces have no closed
//! form here; they are reported in [`ProjectedSilhouettes::unsupported`]
//! rather than approximated.

use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use super::{View, occlusion_step, push_classified_runs};

/// Bound on trim-test samples over all silhouette curves of one solid: each
/// sample is a face-containment query.
const MAX_SILHOUETTE_SAMPLES: usize = 100_000;

/// Fewest samples along one silhouette curve, before trim refinement.
const MIN_CURVE_SAMPLES: usize = 16;

/// Bisection steps that place a trim crossing between two samples: 2⁻⁴⁰ of
/// the sample spacing, far below any drawing tolerance.
const TRIM_BISECTIONS: usize = 40;

/// Bound on the trim queries spent refining crossings, which the sample
/// preflight cannot count: 2 500 crossings, each [`TRIM_BISECTIONS`] queries.
const MAX_TRIM_REFINEMENT_QUERIES: usize = 100_000;

/// Below this, the view is taken to run along a cylinder's or cone's axis,
/// where its outline is its rims and no ruling is a silhouette.
const AXIAL_VIEW_TOLERANCE: f64 = 1e-12;

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
    /// Its surface type tag (`"torus"`, `"nurbs"`).
    pub surface: &'static str,
}

/// A silhouette curve on one face, parameterized by `t ∈ [0, 1]`.
enum Curve {
    /// `start + dir·t`.
    Segment { start: Point3, dir: Vec3 },
    /// `center + radius·(cos 2πt·u + sin 2πt·v)`, closed.
    Circle {
        center: Point3,
        radius: f64,
        u: Vec3,
        v: Vec3,
    },
}

impl Curve {
    fn at(&self, t: f64) -> Point3 {
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
        }
    }

    const fn is_closed(&self) -> bool {
        matches!(self, Self::Circle { .. })
    }

    /// Samples for the trim test and the occlusion split: one per
    /// `deflection` of length on a segment, chord-deflection spacing on a
    /// circle. Unbounded above: the solid's sample budget refuses a count
    /// too large.
    fn sample_count(&self, deflection: f64) -> usize {
        let raw = match *self {
            Self::Segment { dir, .. } => dir.length() / deflection,
            Self::Circle { radius, .. } => {
                let ratio = (1.0 - deflection / radius).clamp(-1.0, 1.0);
                let step = 2.0 * ratio.acos();
                std::f64::consts::TAU / step
            }
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
/// ends on. Planes have none. Torus and NURBS faces are listed in
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

    // Every curve is built and counted before any trim query runs, so an
    // over-budget request is refused before the expensive work.
    let mut curves: Vec<(FaceId, Curve, usize)> = Vec::new();
    let mut result = ProjectedSilhouettes::default();
    let mut samples = 0usize;
    for face in remus_topology::explorer::solid_faces(topo, solid)? {
        let built = match face_silhouettes(topo, face, view.dir)? {
            Ok(built) => built,
            Err(surface) => {
                result
                    .unsupported
                    .push(UnsupportedSilhouette { face, surface });
                continue;
            }
        };
        for curve in built {
            let count = curve.sample_count(deflection);
            samples = samples.saturating_add(count);
            if samples > MAX_SILHOUETTE_SAMPLES {
                return Err(crate::OperationsError::InvalidInput {
                    reason: format!(
                        "silhouette sampling exceeds the {MAX_SILHOUETTE_SAMPLES}-point limit; increase deflection"
                    ),
                });
            }
            curves.push((face, curve, count));
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
    for (face, curve, count) in &curves {
        for run in trimmed_runs(topo, *face, curve, *count, &mut refinement_queries)? {
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

/// The silhouette curves of one face's support surface, untrimmed: `Ok` with
/// zero, one or two curves, or `Err` with the type tag of a surface this
/// module does not handle.
fn face_silhouettes(
    topo: &Topology,
    face: FaceId,
    view: Vec3,
) -> Result<Result<Vec<Curve>, &'static str>, crate::OperationsError> {
    Ok(match topo.face(face)?.surface() {
        FaceSurface::Plane { .. } => Ok(Vec::new()),
        FaceSurface::Torus(_) => Err("torus"),
        FaceSurface::Nurbs(_) => Err("nurbs"),
        FaceSurface::Sphere(sphere) => {
            // The great circle in the plane through the centre normal to the view.
            let (u, v) = plane_basis(view)?;
            Ok(vec![Curve::Circle {
                center: sphere.center(),
                radius: sphere.radius(),
                u,
                v,
            }])
        }
        FaceSurface::Cylinder(cylinder) => {
            // Normal at u is x·cos u + y·sin u: perpendicular to the view at
            // u₀ = atan2(−x·d, y·d) and u₀ + π.
            let (dx, dy) = (cylinder.x_axis().dot(view), cylinder.y_axis().dot(view));
            if dx.hypot(dy) <= AXIAL_VIEW_TOLERANCE {
                return Ok(Ok(Vec::new()));
            }
            let u0 = (-dx).atan2(dy);
            let axis = cylinder.axis();
            let (v_min, v_max) = axial_range(topo, face, cylinder.origin(), axis, 1.0)?;
            Ok([u0, u0 + std::f64::consts::PI]
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
                .collect())
        }
        FaceSurface::Cone(cone) => {
            // Normal at u is radial(u)·sin a − axis·cos a: perpendicular to
            // the view where cos(u − φ) = (axis·d)·cot a / hypot(x·d, y·d).
            let (dx, dy) = (cone.x_axis().dot(view), cone.y_axis().dot(view));
            let reach = dx.hypot(dy);
            if reach <= AXIAL_VIEW_TOLERANCE {
                return Ok(Ok(Vec::new()));
            }
            let (sin_a, cos_a) = cone.half_angle().sin_cos();
            let c = cone.axis().dot(view) * cos_a / (sin_a * reach);
            // At |c| = 1 the two rulings meet in the one generator that runs
            // along the view: it projects to a point, so there is no outline.
            if c.abs() >= 1.0 - AXIAL_VIEW_TOLERANCE {
                return Ok(Ok(Vec::new()));
            }
            let phi = dy.atan2(dx);
            let spread = c.acos();
            let axis = cone.axis();
            // Generator distance v is axial height / sin a.
            let (v_min, v_max) = axial_range(topo, face, cone.apex(), axis, sin_a)?;
            Ok([phi - spread, phi + spread]
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
                .collect())
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

/// A parameter range along `axis` from `base` that contains the face: the
/// axial heights of its bounding box's corners, divided by `scale` (1 for a
/// cylinder, sin a for a cone's generator distance), padded slightly so a
/// rim lying exactly on a corner height is inside. Trimming then cuts it to
/// the face exactly.
fn axial_range(
    topo: &Topology,
    face: FaceId,
    base: Point3,
    axis: Vec3,
    scale: f64,
) -> Result<(f64, f64), crate::OperationsError> {
    let aabb = remus_check::util::face_aabb(topo, face)?;
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
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
    let pad = (hi - lo).abs().max(1.0) * 1e-9;
    Ok((lo - pad, hi + pad))
}

/// Parameter runs of `curve` inside the face, each a sorted list of `t`
/// values starting and ending on a trim crossing (or the curve's ends).
/// Each crossing spends [`TRIM_BISECTIONS`] of `refinement_queries`.
fn trimmed_runs(
    topo: &Topology,
    face: FaceId,
    curve: &Curve,
    count: usize,
    refinement_queries: &mut usize,
) -> Result<Vec<Vec<f64>>, crate::OperationsError> {
    let inside = |t: f64| -> Result<bool, crate::OperationsError> {
        Ok(remus_check::classify::surface_point_in_face(
            topo,
            face,
            curve.at(t),
        )?)
    };
    let ts: Vec<f64> = (0..=count).map(|i| i as f64 / count as f64).collect();
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
            .checked_sub(TRIM_BISECTIONS)
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
