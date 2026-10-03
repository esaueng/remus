//! Directional projection of edges onto faces, solids, and sketch planes
//! (P-Class 7.4).
//!
//! Contract: `docs/design/p74-curve-projection.md`; acceptance oracles in
//! `crates/operations/tests/qualify_project_curve.rs`.
//!
//! Slice 1 implements the projection half of 7.4 as a bounded,
//! exact-where-possible operation: analytic sketch curves (line segments,
//! circular arcs, ellipses) projected along a fixed direction onto planar and
//! analytic curved (cylinder, cone, sphere) faces, returning on-face free
//! edges with a disclosed quality, plus typed refusals outside the qualified
//! cell. Torus and NURBS targets are refused; wrap/emboss, silhouettes and
//! the surface-extension half are later rows.
//!
//! Geometry strategy (all closed forms from the design note):
//!
//! - Onto a plane the projection is affine (`P(x) = x + λd̂`); lines map to
//!   lines and conics to conics of the same affine class (Gram rule).
//! - A segment swept along `d̂` fills a plane `Π`; `Π ∩ S` is a conic
//!   (line, circle, ellipse, parabola, hyperbola) computed exactly per
//!   carrier, and the image is its front arc (first-hit selection, silhouette
//!   split parameters included).
//! - A coaxial arc sweeps a circular cylinder about the target axis; the
//!   image is a translated circle. Non-coaxial arcs on curved quadrics are
//!   the approximate cell (fitted cubic NURBS with disclosed deviation) and
//!   are refused unless the caller opts in.
//! - Clipping is by exact split parameters (boundary edges ∩ `Π`, silhouette
//!   points, source ends); open intervals are classified by midpoint
//!   first-hit ray casts. Seam crossings split pieces with a shared vertex.
//!
//! Results are free edges (no wires, coedges, p-curves or journal entries).
//! Every entry point runs inside `run_append_only`; a refusal leaves the
//! topology untouched.

use std::f64::consts::TAU;

use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use remus_math::curves2d::{Circle2D, Curve2D, Ellipse2D, Line2D, NurbsCurve2D};
use remus_math::det_hash::{DetHashMap, DetHashSet};
use remus_math::frame::Frame3;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface};
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Point3, Vec2, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::pcurve::PCurve;
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;

use crate::OperationsError;

/// Default cap on control points of an approximate (fitted) projection.
pub const DEFAULT_MAX_CONTROL_POINTS: usize = 512;

/// Caller options for [`project_curve_onto_face`] and
/// [`project_curves_onto_solid`].
#[derive(Debug, Clone)]
pub struct ProjectCurveOptions {
    /// Accept a fitted NURBS image where no exact construction exists
    /// (arc onto a non-coaxial curved quadric). `false` refuses with
    /// [`ProjectCurveError::ApproximationRequired`].
    pub allow_approximate: bool,
    /// Absolute deviation bound for approximate images. `None` means
    /// `1e-6 · scale`. Values below `1e-9 · scale`, non-finite, or
    /// non-positive values are refused as
    /// [`ProjectCurveError::InvalidOptions`].
    pub approximation_tolerance: Option<f64>,
    /// Control-point budget per approximate edge (minimum 4).
    pub max_control_points: usize,
    /// Frame for 2D output on planar targets. Its `z` must be parallel to
    /// the target plane normal and its origin must lie on that plane.
    pub plane_frame: Option<Frame3>,
}

impl Default for ProjectCurveOptions {
    fn default() -> Self {
        Self {
            allow_approximate: false,
            approximation_tolerance: None,
            max_control_points: DEFAULT_MAX_CONTROL_POINTS,
            plane_frame: None,
        }
    }
}

/// Geometric quality of a projection result.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ProjectionQuality {
    /// Every edge carries the closed-form image.
    Exact,
    /// At least one edge is a fitted NURBS.
    Approximate {
        /// Sampled maximum of the deviation metric defined in the design
        /// note. Not a certified bound.
        max_deviation: f64,
    },
}

/// One projected edge.
#[derive(Debug, Clone)]
pub struct ProjectedEdge {
    /// New free edge (owned by the caller; not in any wire or face).
    pub edge: EdgeId,
    /// Face the edge lies on.
    pub face: FaceId,
    /// Increasing sub-interval of the source edge's domain that maps onto
    /// this edge. The edge's start vertex is the image of `source_range.0`.
    pub source_range: (f64, f64),
    /// The edge in `plane_frame` coordinates, when a frame was supplied
    /// for a planar target.
    pub plane_curve: Option<PCurve>,
}

/// Result of projecting one source edge onto one face.
#[derive(Debug, Clone)]
pub struct ProjectedCurves {
    /// Projected edges in source order.
    pub edges: Vec<ProjectedEdge>,
    /// Target face.
    pub face: FaceId,
    /// Exact or disclosed-approximate.
    pub quality: ProjectionQuality,
    /// Some part of the source domain has no image on the face.
    pub clipped: bool,
}

/// Projection of one source edge onto a solid.
#[derive(Debug, Clone)]
pub struct SourceProjection {
    /// The source edge.
    pub source: EdgeId,
    /// Projected edges in source order, possibly on several faces. Empty
    /// when the source misses the solid entirely.
    pub edges: Vec<ProjectedEdge>,
    /// Some part of the source domain has no image on the solid.
    pub clipped: bool,
}

/// Result of [`project_curves_onto_solid`].
#[derive(Debug, Clone)]
pub struct SolidProjection {
    /// One entry per source edge, in input order.
    pub sources: Vec<SourceProjection>,
    /// Worst quality over all edges.
    pub quality: ProjectionQuality,
}

/// Typed refusal from curve projection. Codes from [`Self::code`] are API.
#[derive(Debug, thiserror::Error)]
pub enum ProjectCurveError {
    /// The direction is zero-length or non-finite.
    #[error("projection direction must be finite and non-zero")]
    InvalidDirection,
    /// An option value is out of range.
    #[error("invalid projection options: {reason}")]
    InvalidOptions {
        /// What is wrong.
        reason: String,
    },
    /// The source edge has zero length or invalid geometry.
    #[error("degenerate source edge: {reason}")]
    DegenerateSource {
        /// What is degenerate.
        reason: &'static str,
    },
    /// The source curve type has no projection cell for this target.
    #[error("unsupported source curve {curve} for target {surface}")]
    UnsupportedSourceCurve {
        /// Source curve type tag.
        curve: &'static str,
        /// Target surface type tag (`"plane"` for sketch planes).
        surface: &'static str,
    },
    /// The target face's surface type is not projectable in this slice.
    #[error("unsupported target surface {surface}")]
    UnsupportedTargetSurface {
        /// Target surface type tag.
        surface: &'static str,
    },
    /// A segment parallel to the direction projects to a point.
    #[error("source segment is parallel to the projection direction")]
    SourceParallelToDirection,
    /// The image collapses or folds (a conic seen edge-on, a NURBS image
    /// with a stationary point).
    #[error("projected image is degenerate")]
    DegenerateImage,
    /// The direction is parallel to the target plane or to a cylinder axis.
    #[error("projection direction grazes the target surface")]
    GrazingDirection,
    /// The swept carrier is tangent to the target surface.
    #[error("swept carrier is tangent to the target surface")]
    TangentSection,
    /// The sweep plane passes through the cone apex.
    #[error("sweep plane passes through the cone apex")]
    SectionThroughApex,
    /// The cone section lies in the unresolved band around the parabola.
    #[error("cone section is near-parabolic (A = {a_coefficient:e})")]
    NearParabolicSection {
        /// Conic classifier `A = p² − sin²(half_angle)`.
        a_coefficient: f64,
    },
    /// No exact image exists and approximation was not allowed.
    #[error("no exact projection exists; approximation was not allowed")]
    ApproximationRequired,
    /// The fitted image cannot meet the tolerance within the budget.
    #[error("approximation tolerance {requested:e} unattainable (achieved {achieved:e})")]
    ToleranceUnattainable {
        /// Requested absolute tolerance.
        requested: f64,
        /// Best deviation reached within the control-point budget.
        achieved: f64,
    },
    /// An approximate image would need clipping against the face boundary.
    #[error("approximate projections cannot be clipped in this slice")]
    ApproximateClipUnsupported,
    /// The image runs along a face boundary edge over a positive length.
    #[error("projected image runs along a face boundary")]
    ImageAlongBoundary,
    /// No part of the source has an image on the target.
    #[error("projection is empty")]
    EmptyProjection,
    /// The supplied plane frame does not lie in the planar target.
    #[error("plane frame does not match the target plane")]
    PlaneFrameMismatch,
    /// The constructed image failed the on-face residual post-check.
    #[error("projection residual {measured:e} exceeds bound {bound:e}")]
    ResidualExceeded {
        /// Largest sampled residual.
        measured: f64,
        /// Allowed residual.
        bound: f64,
    },
    /// One source of a multi-source call was refused; nothing was created.
    #[error("source {index} refused: {error}")]
    SourceRefused {
        /// Index into the caller's source slice.
        index: usize,
        /// The refusal for that source.
        #[source]
        error: Box<Self>,
    },
    /// Lookup or kernel failure (stale handles, invalid topology).
    #[error(transparent)]
    Operations(#[from] OperationsError),
}

impl ProjectCurveError {
    /// Stable machine-readable refusal code.
    #[must_use]
    pub const fn code(&self) -> &'static str {
        match self {
            Self::InvalidDirection => "invalid-direction",
            Self::InvalidOptions { .. } => "invalid-options",
            Self::DegenerateSource { .. } => "degenerate-source",
            Self::UnsupportedSourceCurve { .. } => "unsupported-source-curve",
            Self::UnsupportedTargetSurface { .. } => "unsupported-target-surface",
            Self::SourceParallelToDirection => "source-parallel-to-direction",
            Self::DegenerateImage => "degenerate-image",
            Self::GrazingDirection => "grazing-direction",
            Self::TangentSection => "tangent-section",
            Self::SectionThroughApex => "section-through-apex",
            Self::NearParabolicSection { .. } => "near-parabolic-section",
            Self::ApproximationRequired => "approximation-required",
            Self::ToleranceUnattainable { .. } => "tolerance-unattainable",
            Self::ApproximateClipUnsupported => "approximate-clip-unsupported",
            Self::ImageAlongBoundary => "image-along-boundary",
            Self::EmptyProjection => "empty-projection",
            Self::PlaneFrameMismatch => "plane-frame-mismatch",
            Self::ResidualExceeded { .. } => "residual-exceeded",
            Self::SourceRefused { .. } => "source-refused",
            Self::Operations(_) => "operations",
        }
    }
}

// ---------------------------------------------------------------------------
// Internal thresholds (contract §4/§7; scale-relative where stated).
// ---------------------------------------------------------------------------

/// Absolute parameter tolerance for source-range agreement.
const PARAM_ABS: f64 = 1e-9;
/// Deduplication band for split parameters.
const PARAM_DEDUPE: f64 = 1e-12;
/// Plane graze band on `|n·d̂|`.
const GRAZE_PLANE: f64 = 1e-9;
/// Cylinder-axis graze band on `|a×d̂|`.
const GRAZE_CYLINDER: f64 = 1e-9;
/// Segment-parallel band on `|ê×d̂|`.
const PARALLEL_SOURCE: f64 = 1e-9;
/// Conic edge-on band on `|N·d̂|`.
const DEGENERATE_CONIC: f64 = 1e-9;
/// Sweep-plane ∥/⟂ axis bands on `|m·a|` / `|m×a|`.
const SWEEP_AXIS_BAND: f64 = 1e-12;
/// Coaxial bands on `|N×d̂|` and center-line distance (`·scale`).
const COAXIAL_BAND: f64 = 1e-9;
/// Tangency / apex bands (`·scale`).
const TANGENT_BAND: f64 = 1e-9;
/// Cone classifier bands on `A` (absolute, part of the contract).
const PARABOLA_EXACT_BAND: f64 = 1e-12;
const PARABOLA_REFUSE_BAND: f64 = 1e-6;
/// Post-check samples per piece (contract §7: 33 points).
const POST_SAMPLES: usize = 33;
/// Samples for the approximate clip probe.
const APPROX_CLIP_SAMPLES: usize = 2049;
/// Samples for the approximate deviation disclosure (denser than the
/// oracle's 4096, per the design note).
const APPROX_DISCLOSE_SAMPLES: usize = 16385;
/// Fit point counts tried in order (control points equal point count).
const APPROX_FIT_COUNTS: &[usize] = &[8, 16, 32, 64, 128, 256, 512];
/// Samples for boundary-overlap and stationary-point probes.
const PROBE_SAMPLES: usize = 1024;

// ---------------------------------------------------------------------------
// Small helpers.
// ---------------------------------------------------------------------------

fn linear_tol() -> f64 {
    Tolerance::new().linear
}

fn invalid_options(reason: impl Into<String>) -> ProjectCurveError {
    ProjectCurveError::InvalidOptions {
        reason: reason.into(),
    }
}

fn math_to_ops(context: &'static str, error: remus_math::MathError) -> ProjectCurveError {
    ProjectCurveError::Operations(OperationsError::InvalidInput {
        reason: format!("{context}: {error}"),
    })
}

fn unit(vector: Vec3, context: &'static str) -> Result<Vec3, ProjectCurveError> {
    vector
        .normalize()
        .map_err(|error| math_to_ops(context, error))
}

fn validate_direction(direction: Vec3) -> Result<Vec3, ProjectCurveError> {
    if !direction.0.iter().all(|v| v.is_finite()) {
        return Err(ProjectCurveError::InvalidDirection);
    }
    let largest = direction.0.iter().map(|v| v.abs()).fold(0.0, f64::max);
    if largest == 0.0 {
        return Err(ProjectCurveError::InvalidDirection);
    }
    unit(
        Vec3::new(
            direction.x() / largest,
            direction.y() / largest,
            direction.z() / largest,
        ),
        "projection direction",
    )
}

fn is_full_turn(range: (f64, f64)) -> bool {
    ((range.1 - range.0).abs() - TAU).abs() <= 1e-12
}

/// Ascending stable roots of `a·t² + b·t + c = 0`, mirroring the
/// acceptance oracle's tangency snap (a grazing discriminant within
/// `1e-12·b²` of zero counts as a double root).
fn quadratic_roots(a: f64, b: f64, c: f64) -> Vec<f64> {
    let magnitude = a.abs().max(b.abs()).max(c.abs());
    if magnitude == 0.0 || !magnitude.is_finite() {
        return Vec::new();
    }
    if a.abs() <= 1e-14 * magnitude {
        if b.abs() > 0.0 {
            return vec![-c / b];
        }
        return Vec::new();
    }
    let discriminant = b.mul_add(b, -4.0 * a * c);
    if discriminant < -1e-12 * b * b {
        return Vec::new();
    }
    if discriminant.abs() <= 1e-12 * b * b {
        // Grazing contact (either sign of fp dust): the double root
        // directly. Splitting the dust would straddle the contact by its
        // square root (~1e-9 relative), far above the vertex tolerance.
        return vec![-0.5 * b / a];
    }
    let root = discriminant.sqrt();
    let q = -0.5 * (b + b.signum() * root);
    if q == 0.0 {
        return vec![0.0];
    }
    let mut roots = vec![q / a, c / q];
    roots.sort_by(f64::total_cmp);
    roots
}

// ---------------------------------------------------------------------------
// Resolved source and target.
// ---------------------------------------------------------------------------

/// A supported source curve with its domain and extent.
#[derive(Debug, Clone)]
enum SourceGeom {
    Segment {
        a: Point3,
        b: Point3,
    },
    Circle {
        circle: Circle3D,
    },
    Ellipse {
        ellipse: Ellipse3D,
    },
    /// NURBS, parabola or hyperbola: refused on faces, exact on sketch planes.
    Other {
        curve: EdgeCurve,
    },
}

#[derive(Debug, Clone)]
struct ResolvedSource {
    geom: SourceGeom,
    domain: (f64, f64),
    extent: f64,
    closed: bool,
}

fn resolve_source(topo: &Topology, source: EdgeId) -> Result<ResolvedSource, ProjectCurveError> {
    let edge = topo.edge(source).map_err(OperationsError::from)?;
    let start = topo.vertex(edge.start()).map_err(OperationsError::from)?;
    let end = topo.vertex(edge.end()).map_err(OperationsError::from)?;
    let (a, b) = (start.point(), end.point());
    match edge.curve() {
        EdgeCurve::Line => {
            let length = (b - a).length();
            if !length.is_finite() || length < linear_tol() {
                return Err(ProjectCurveError::DegenerateSource {
                    reason: "segment is shorter than the linear tolerance",
                });
            }
            Ok(ResolvedSource {
                geom: SourceGeom::Segment { a, b },
                domain: (0.0, 1.0),
                extent: length,
                closed: false,
            })
        }
        EdgeCurve::Circle(circle) => {
            let range = edge
                .strict_domain()
                .map_err(|_| ProjectCurveError::DegenerateSource {
                    reason: "circle edge has no valid parameter range",
                })?;
            if range.1 <= range.0 {
                return Err(ProjectCurveError::DegenerateSource {
                    reason: "face projection requires an increasing source trim",
                });
            }
            if !circle.radius().is_finite() || circle.radius() <= 0.0 {
                return Err(ProjectCurveError::DegenerateSource {
                    reason: "circle source has invalid radius",
                });
            }
            Ok(ResolvedSource {
                extent: 2.0 * circle.radius(),
                closed: is_full_turn(range),
                geom: SourceGeom::Circle {
                    circle: circle.clone(),
                },
                domain: range,
            })
        }
        EdgeCurve::Ellipse(ellipse) => {
            let range = edge
                .strict_domain()
                .map_err(|_| ProjectCurveError::DegenerateSource {
                    reason: "ellipse edge has no valid parameter range",
                })?;
            if range.1 <= range.0 {
                return Err(ProjectCurveError::DegenerateSource {
                    reason: "face projection requires an increasing source trim",
                });
            }
            Ok(ResolvedSource {
                extent: 2.0 * ellipse.semi_major(),
                closed: is_full_turn(range),
                geom: SourceGeom::Ellipse {
                    ellipse: ellipse.clone(),
                },
                domain: range,
            })
        }
        other => Ok(ResolvedSource {
            geom: SourceGeom::Other {
                curve: other.clone(),
            },
            domain: (0.0, 1.0),
            extent: 0.0,
            closed: false,
        }),
    }
}

/// A projectable target surface with owned carrier data.
#[derive(Debug, Clone)]
pub(crate) enum TargetGeom {
    Plane { normal: Vec3, delta: f64 },
    Cylinder(CylindricalSurface),
    Cone(ConicalSurface),
    Sphere(SphericalSurface),
}

fn resolve_target(topo: &Topology, face: FaceId) -> Result<TargetGeom, ProjectCurveError> {
    let surface = topo.face(face).map_err(OperationsError::from)?;
    match surface.surface() {
        FaceSurface::Plane { normal, d } => {
            let length = normal.length();
            if !length.is_finite() || length <= 0.0 {
                return Err(ProjectCurveError::Operations(
                    OperationsError::InvalidInput {
                        reason: "planar target has an invalid normal".to_string(),
                    },
                ));
            }
            Ok(TargetGeom::Plane {
                normal: *normal * (1.0 / length),
                delta: *d / length,
            })
        }
        FaceSurface::Cylinder(c) => Ok(TargetGeom::Cylinder(c.clone())),
        FaceSurface::Cone(c) => Ok(TargetGeom::Cone(c.clone())),
        FaceSurface::Sphere(s) => Ok(TargetGeom::Sphere(s.clone())),
        FaceSurface::Torus(_) => {
            Err(ProjectCurveError::UnsupportedTargetSurface { surface: "torus" })
        }
        FaceSurface::Nurbs(_) => {
            Err(ProjectCurveError::UnsupportedTargetSurface { surface: "nurbs" })
        }
    }
}

fn target_tag(target: &TargetGeom) -> &'static str {
    match target {
        TargetGeom::Plane { .. } => "plane",
        TargetGeom::Cylinder(_) => "cylinder",
        TargetGeom::Cone(_) => "cone",
        TargetGeom::Sphere(_) => "sphere",
    }
}

/// Boundary vertex positions of a face (deduplicated by handle).
fn face_boundary_points(topo: &Topology, face: FaceId) -> Result<Vec<Point3>, ProjectCurveError> {
    let oriented = topo
        .face_oriented_edges(face)
        .map_err(OperationsError::from)?;
    let mut seen = DetHashSet::default();
    let mut points = Vec::new();
    for oriented_edge in &oriented {
        let edge = topo
            .edge(oriented_edge.edge())
            .map_err(OperationsError::from)?;
        for vertex in [edge.start(), edge.end()] {
            if seen.insert(vertex.index()) {
                points.push(topo.vertex(vertex).map_err(OperationsError::from)?.point());
            }
        }
    }
    Ok(points)
}

/// Target extent per the design note: plane = boundary-box diagonal;
/// cylinder/sphere = radius; cone = largest apex-to-boundary distance.
fn target_extent(
    topo: &Topology,
    face: FaceId,
    target: &TargetGeom,
) -> Result<f64, ProjectCurveError> {
    match target {
        TargetGeom::Cylinder(c) => Ok(c.radius()),
        TargetGeom::Sphere(s) => Ok(s.radius()),
        TargetGeom::Cone(cone) => {
            let mut largest = 0.0f64;
            for point in face_boundary_points(topo, face)? {
                largest = largest.max((point - cone.apex()).length());
            }
            Ok(largest)
        }
        TargetGeom::Plane { .. } => {
            let points = face_boundary_points(topo, face)?;
            if points.is_empty() {
                return Ok(0.0);
            }
            let mut minimum = points[0];
            let mut maximum = points[0];
            for point in &points[1..] {
                for axis in 0..3 {
                    minimum.0[axis] = minimum.0[axis].min(point.0[axis]);
                    maximum.0[axis] = maximum.0[axis].max(point.0[axis]);
                }
            }
            Ok((maximum - minimum).length())
        }
    }
}

/// Seam edges of a face: boundary edges the wire uses twice.
fn seam_edges(topo: &Topology, face: FaceId) -> Result<Vec<EdgeId>, ProjectCurveError> {
    let oriented = topo
        .face_oriented_edges(face)
        .map_err(OperationsError::from)?;
    let mut counts = DetHashMap::default();
    for oriented_edge in &oriented {
        *counts.entry(oriented_edge.edge().index()).or_insert(0usize) += 1;
    }
    let mut seams: Vec<EdgeId> = oriented
        .iter()
        .map(remus_topology::OrientedEdge::edge)
        .filter(|edge| counts.get(&edge.index()).is_some_and(|count| *count == 2))
        .collect();
    seams.sort_by_key(|edge| edge.index());
    seams.dedup();
    Ok(seams)
}

// ---------------------------------------------------------------------------
// Carriers: the exact image curve of one (source, face) pair.
// ---------------------------------------------------------------------------

/// The closed-form image carrier of one (source, face) pair.
#[derive(Debug, Clone)]
pub(crate) enum Carrier {
    Line { p0: Point3, p1: Point3 },
    Circle(Circle3D),
    Ellipse(Ellipse3D),
    Hyperbola(Hyperbola3D),
    Parabola(Parabola3D),
}

impl Carrier {
    fn to_edge_curve(&self) -> EdgeCurve {
        match self {
            Self::Line { .. } => EdgeCurve::Line,
            Self::Circle(c) => EdgeCurve::Circle(c.clone()),
            Self::Ellipse(e) => EdgeCurve::Ellipse(e.clone()),
            Self::Hyperbola(h) => EdgeCurve::Hyperbola(h.clone()),
            Self::Parabola(p) => EdgeCurve::Parabola(p.clone()),
        }
    }

    /// Project a near-carrier point to a carrier parameter.
    fn project(&self, point: Point3) -> f64 {
        match self {
            Self::Line { p0, p1 } => {
                let axis = *p1 - *p0;
                (point - *p0).dot(axis) / axis.dot(axis)
            }
            Self::Circle(c) => c.project(point),
            Self::Ellipse(e) => e.project(point),
            Self::Hyperbola(h) => h.project(point),
            Self::Parabola(p) => p.project(point),
        }
    }

    fn evaluate(&self, t: f64) -> Point3 {
        match self {
            Self::Line { p0, p1 } => *p0 + (*p1 - *p0) * t,
            Self::Circle(c) => c.evaluate(t),
            Self::Ellipse(e) => e.evaluate(t),
            Self::Hyperbola(h) => h.evaluate(t),
            Self::Parabola(p) => p.evaluate(t),
        }
    }
}

/// Affine projection onto a plane along `direction`: `P(x)`.
#[derive(Debug, Clone, Copy)]
struct AffineMap {
    normal: Vec3,
    delta: f64,
    direction: Vec3,
    denom: f64,
}

impl AffineMap {
    fn new(normal: Vec3, delta: f64, direction: Vec3) -> Self {
        Self {
            normal,
            delta,
            direction,
            denom: normal.dot(direction),
        }
    }

    fn apply(&self, point: Point3) -> Point3 {
        point
            + self.direction
                * ((self.delta - self.normal.dot(point - Point3::new(0.0, 0.0, 0.0))) / self.denom)
    }

    /// Linear part applied to a free vector: `L(w)`.
    fn apply_vector(&self, vector: Vec3) -> Vec3 {
        vector - self.direction * (self.normal.dot(vector) / self.denom)
    }
}

/// Semi-axes and major direction of the ellipse with conjugate
/// semi-diameters `a` and `b` (design note §4.1 Gram rule).
fn gram_ellipse(a: Vec3, b: Vec3) -> Option<(Vec3, f64, f64)> {
    let (g11, g12, g22) = (a.dot(a), a.dot(b), b.dot(b));
    let trace = g11 + g22;
    if !trace.is_finite() || trace <= 0.0 {
        return None;
    }
    let determinant = g11.mul_add(g22, -(g12 * g12));
    let root = trace.mul_add(trace, -4.0 * determinant).max(0.0).sqrt();
    let (major_squared, minor_squared) = (0.5 * (trace + root), 0.5 * (trace - root));
    if minor_squared <= 0.0 {
        return None;
    }
    let (w1, w2) = if g12.abs() > 1e-15 * trace {
        (g12, major_squared - g11)
    } else if g11 >= g22 {
        (1.0, 0.0)
    } else {
        (0.0, 1.0)
    };
    let major = (a * w1 + b * w2).normalize().ok()?;
    Some((major, major_squared.sqrt(), minor_squared.sqrt()))
}

/// Conic source frame for affine images: center, plane normal, in-plane
/// axes with their extents (circle: `ru == rv == radius`).
#[derive(Debug, Clone, Copy)]
struct ConicFrame {
    center: Point3,
    normal: Vec3,
    u: Vec3,
    v: Vec3,
    ru: f64,
    rv: f64,
}

/// Build the affine image of a circle/ellipse source on a plane target.
/// Returns the carrier plus the conjugate semi-diameters (for trim mapping).
fn affine_conic_image(
    map: &AffineMap,
    source: &ConicFrame,
    plane_normal: Vec3,
) -> Result<(Carrier, Vec3, Vec3), ProjectCurveError> {
    let image_center = map.apply(source.center);
    let a_vec = map.apply_vector(source.u) * source.ru;
    let b_vec = map.apply_vector(source.v) * source.rv;
    let (major, major_len, minor_len) =
        gram_ellipse(a_vec, b_vec).ok_or(ProjectCurveError::DegenerateImage)?;
    // Circle when the semi-axes agree within 1e-12 relative.
    if (major_len - minor_len).abs() <= 1e-12 * major_len.max(minor_len) {
        let radius = 0.5 * (major_len + minor_len);
        let circle = Circle3D::new(image_center, plane_normal, radius)
            .map_err(|error| math_to_ops("affine circle image", error))?;
        return Ok((Carrier::Circle(circle), a_vec, b_vec));
    }
    let minor = unit(plane_normal.cross(major), "affine ellipse frame")?;
    let ellipse = Ellipse3D::with_axes(
        image_center,
        plane_normal,
        major_len,
        minor_len,
        major,
        minor,
    )
    .map_err(|error| math_to_ops("affine ellipse image", error))?;
    Ok((Carrier::Ellipse(ellipse), a_vec, b_vec))
}

// ---------------------------------------------------------------------------
// Exact models: carrier plus the plane used for boundary-split parameters.
// ---------------------------------------------------------------------------

/// How the image of one (source, face) pair is computed and clipped.
#[derive(Debug, Clone)]
enum FaceModel {
    /// No image is possible on this face (miss, wrong nappe); yields nothing.
    Empty,
    /// First-hit ray casts against the face; split parameters come from
    /// boundary edges ∩ `clip_plane` plus silhouette parameters.
    RayCast {
        carrier: Carrier,
        clip_plane: (Point3, Vec3),
        silhouettes: Vec<Point3>,
    },
    /// Affine image on a plane target; split parameters come from roots of
    /// the image conic implicit equation composed with boundary edges.
    AffineConic { carrier: Carrier, conic: Conic2D },
}

impl FaceModel {
    fn carrier(&self) -> Option<&Carrier> {
        match self {
            Self::Empty => None,
            Self::RayCast { carrier, .. } | Self::AffineConic { carrier, .. } => Some(carrier),
        }
    }
}

/// Implicit conic in a target-plane 2D frame for `Q(B(t)) = 0` clipping.
#[derive(Debug, Clone)]
enum Conic2D {
    Line {
        point: Point2,
        direction: Vec2,
    },
    Circle {
        center: Point2,
        radius: f64,
    },
    Ellipse {
        center: Point2,
        semi_major: f64,
        semi_minor: f64,
        rotation: f64,
    },
}

impl Conic2D {
    fn evaluate(&self, p: Point2) -> f64 {
        match *self {
            Self::Line { point, direction } => {
                let w = p - point;
                w.x() * direction.y() - w.y() * direction.x()
            }
            Self::Circle { center, radius } => {
                let w = p - center;
                w.x().mul_add(w.x(), w.y() * w.y()) - radius * radius
            }
            Self::Ellipse {
                center,
                semi_major,
                semi_minor,
                rotation,
            } => {
                let w = p - center;
                let (sin_r, cos_r) = rotation.sin_cos();
                let local_x = w.x().mul_add(cos_r, w.y() * sin_r) / semi_major;
                let local_y = w.x().mul_add(-sin_r, w.y() * cos_r) / semi_minor;
                local_x.mul_add(local_x, local_y * local_y) - 1.0
            }
        }
    }
}

/// Orthonormal 2D frame in a target plane for implicit clipping.
#[derive(Debug, Clone, Copy)]
struct PlaneFrame2D {
    origin: Point3,
    x: Vec3,
    y: Vec3,
}

impl PlaneFrame2D {
    fn of(normal: Vec3, point: Point3) -> Result<Self, ProjectCurveError> {
        let frame = Frame3::from_normal(point, normal)
            .map_err(|error| math_to_ops("target plane frame", error))?;
        Ok(Self {
            origin: frame.origin,
            x: frame.x,
            y: frame.y,
        })
    }

    fn to_2d(self, point: Point3) -> Point2 {
        let w = point - self.origin;
        Point2::new(w.dot(self.x), w.dot(self.y))
    }
}

/// Build the exact model for a segment source on one face.
fn segment_model(
    a: Point3,
    b: Point3,
    direction: Vec3,
    target: &TargetGeom,
    scale: f64,
) -> Result<FaceModel, ProjectCurveError> {
    let span = b - a;
    let cross = span.cross(direction);
    if cross.length() / span.length() <= PARALLEL_SOURCE {
        return Err(ProjectCurveError::SourceParallelToDirection);
    }
    let sweep_m = unit(cross, "sweep plane normal")?;
    match target {
        TargetGeom::Plane { normal, delta } => {
            if normal.dot(direction).abs() <= GRAZE_PLANE {
                return Err(ProjectCurveError::GrazingDirection);
            }
            let map = AffineMap::new(*normal, *delta, direction);
            Ok(FaceModel::RayCast {
                carrier: Carrier::Line {
                    p0: map.apply(a),
                    p1: map.apply(b),
                },
                clip_plane: (a, sweep_m),
                silhouettes: Vec::new(),
            })
        }
        TargetGeom::Cylinder(cyl) => {
            if cyl.axis().cross(direction).length() <= GRAZE_CYLINDER {
                return Err(ProjectCurveError::GrazingDirection);
            }
            cylinder_section(a, sweep_m, direction, cyl, scale)
        }
        TargetGeom::Sphere(sph) => sphere_section(a, sweep_m, direction, sph, scale),
        TargetGeom::Cone(cone) => cone_section(a, sweep_m, direction, cone, scale),
    }
}

/// Plane ∩ cylinder section (design note §4.2).
fn cylinder_section(
    plane_point: Point3,
    m: Vec3,
    direction: Vec3,
    cyl: &CylindricalSurface,
    scale: f64,
) -> Result<FaceModel, ProjectCurveError> {
    let axis = cyl.axis();
    let m_dot_axis = m.dot(axis);
    if m_dot_axis.abs() <= SWEEP_AXIS_BAND {
        // Sweep plane parallel to the axis: two rulings; the front one is
        // first along the sweep direction. With `dy = d̂·along`, the signed
        // gap `λ₊ − λ₋ = 2·offset/dy` is source-independent, so the front
        // ruling is fixed by the sign of `dy` alone.
        let rho = m.dot(cyl.origin() - plane_point).abs();
        let radius = cyl.radius();
        if (rho - radius).abs() <= TANGENT_BAND * scale {
            return Err(ProjectCurveError::TangentSection);
        }
        if rho > radius {
            return Ok(FaceModel::Empty);
        }
        let along = (m.cross(axis))
            .normalize()
            .map_err(|error| math_to_ops("ruling offset", error))?;
        let offset = (radius * radius - rho * rho).max(0.0).sqrt();
        let base = cyl.origin() - m * m.dot(cyl.origin() - plane_point);
        let dy = direction.dot(along);
        let front = if dy >= 0.0 {
            base - along * offset
        } else {
            base + along * offset
        };
        Ok(FaceModel::RayCast {
            carrier: Carrier::Line {
                p0: front,
                p1: front + axis,
            },
            clip_plane: (plane_point, m),
            silhouettes: Vec::new(),
        })
    } else if m.cross(axis).length() <= SWEEP_AXIS_BAND {
        // Sweep plane perpendicular to the axis: circle.
        let center = cyl.origin() + axis * (m.dot(plane_point - cyl.origin()) / m_dot_axis);
        let circle = Circle3D::new(center, axis, cyl.radius())
            .map_err(|error| math_to_ops("cylinder circle section", error))?;
        circle_silhouettes(center, cyl.radius(), m, direction).map(|silhouettes| {
            FaceModel::RayCast {
                carrier: Carrier::Circle(circle),
                clip_plane: (plane_point, m),
                silhouettes,
            }
        })
    } else {
        // Oblique: ellipse, semi-minor r, semi-major r/cos θ.
        let cos_theta = m_dot_axis.abs();
        let center = cyl.origin() + axis * (m.dot(plane_point - cyl.origin()) / m_dot_axis);
        let minor_dir = unit(axis.cross(m), "cylinder section minor")?;
        let major_dir = unit(axis - m * m_dot_axis, "cylinder section major")?;
        let minor = cyl.radius();
        let major = cyl.radius() / cos_theta;
        // In-plane frame: `major_dir` carries the major extent, `minor_dir`
        // the minor one (both unit, both perpendicular, both in the plane).
        let ellipse = Ellipse3D::with_axes(center, m, major, minor, major_dir, minor_dir)
            .map_err(|error| math_to_ops("cylinder ellipse section", error))?;
        let silhouettes = ellipse_silhouettes(&ellipse, direction);
        Ok(FaceModel::RayCast {
            carrier: Carrier::Ellipse(ellipse),
            clip_plane: (plane_point, m),
            silhouettes,
        })
    }
}

/// Silhouette points of a circle carrier: where `direction` is tangent.
fn circle_silhouettes(
    center: Point3,
    radius: f64,
    plane_normal: Vec3,
    direction: Vec3,
) -> Result<Vec<Point3>, ProjectCurveError> {
    let tangent = unit(plane_normal.cross(direction), "circle silhouette")?;
    Ok(vec![center + tangent * radius, center - tangent * radius])
}

/// Silhouette points of an ellipse carrier: tangent ∥ `direction`.
fn ellipse_silhouettes(ellipse: &Ellipse3D, direction: Vec3) -> Vec<Point3> {
    let normal = ellipse.normal();
    let u = ellipse.u_axis();
    let v = ellipse.v_axis();
    let (a, b) = (ellipse.semi_major(), ellipse.semi_minor());
    let cu = u.cross(direction).dot(normal);
    let cv = v.cross(direction).dot(normal);
    let angle = (b * cv).atan2(a * cu);
    vec![
        ellipse.evaluate(angle),
        ellipse.evaluate(angle + std::f64::consts::PI),
    ]
}

/// Silhouette parameter of a hyperbola carrier (`tanh t`), if on this branch.
fn hyperbola_silhouette(
    hyperbola: &Hyperbola3D,
    plane_normal: Vec3,
    direction: Vec3,
) -> Option<f64> {
    let u = hyperbola.u_axis();
    let v = hyperbola.v_axis();
    let (a, b) = (hyperbola.semi_major(), hyperbola.semi_minor());
    let cu = u.cross(direction).dot(plane_normal);
    let cv = v.cross(direction).dot(plane_normal);
    let ratio = -b * cv / (a * cu);
    if ratio.abs() < 1.0 && cu != 0.0 {
        Some(ratio.atanh())
    } else {
        None
    }
}

/// Silhouette parameter of a parabola carrier, if the tangent aligns.
fn parabola_silhouette(parabola: &Parabola3D, plane_normal: Vec3, direction: Vec3) -> Option<f64> {
    let denom = parabola.axis_dir().cross(direction).dot(plane_normal);
    if denom.abs() <= 1e-15 {
        return None;
    }
    Some(
        -2.0 * parabola.focal_length() * parabola.u_axis().cross(direction).dot(plane_normal)
            / denom,
    )
}

/// Plane ∩ sphere section (design note §4.2).
fn sphere_section(
    plane_point: Point3,
    m: Vec3,
    direction: Vec3,
    sph: &SphericalSurface,
    scale: f64,
) -> Result<FaceModel, ProjectCurveError> {
    let signed = m.dot(sph.center() - plane_point);
    let height = signed.abs();
    let radius = sph.radius();
    if (height - radius).abs() <= TANGENT_BAND * scale {
        return Err(ProjectCurveError::TangentSection);
    }
    if height > radius {
        return Ok(FaceModel::Empty);
    }
    let center = sph.center() - m * signed;
    let section_radius = (radius * radius - height * height).max(0.0).sqrt();
    let circle = Circle3D::new(center, m, section_radius)
        .map_err(|error| math_to_ops("sphere circle section", error))?;
    circle_silhouettes(center, section_radius, m, direction).map(|silhouettes| FaceModel::RayCast {
        carrier: Carrier::Circle(circle),
        clip_plane: (plane_point, m),
        silhouettes,
    })
}

/// Plane ∩ cone section with the §4.2 classifier and bands.
#[allow(clippy::too_many_lines)]
fn cone_section(
    plane_point: Point3,
    m: Vec3,
    direction: Vec3,
    cone: &ConicalSurface,
    scale: f64,
) -> Result<FaceModel, ProjectCurveError> {
    let axis = cone.axis();
    let apex = cone.apex();
    let sine_squared = cone.half_angle().sin().powi(2);
    let c = m.dot(axis);
    let plane_offset = m.dot(plane_point - apex);
    if plane_offset.abs() <= TANGENT_BAND * scale {
        return Err(ProjectCurveError::SectionThroughApex);
    }
    let axis_cross = m.cross(axis);
    if axis_cross.length() <= SWEEP_AXIS_BAND {
        // Sweep plane perpendicular to the axis: circle.
        let tau = plane_offset / c;
        if tau <= 0.0 {
            return Ok(FaceModel::Empty);
        }
        let center = apex + axis * tau;
        let radius = tau * cone.half_angle().cos() / cone.half_angle().sin();
        let circle = Circle3D::new(center, axis, radius)
            .map_err(|error| math_to_ops("cone circle section", error))?;
        return circle_silhouettes(center, radius, axis, direction).map(|silhouettes| {
            FaceModel::RayCast {
                carrier: Carrier::Circle(circle),
                clip_plane: (plane_point, m),
                silhouettes,
            }
        });
    }
    let in_plane = axis - m * c;
    let in_plane_len = in_plane.length();
    let e1 = in_plane * (1.0 / in_plane_len);
    let e2 = m.cross(e1);
    let p_squared = 1.0 - c * c;
    let classifier = p_squared - sine_squared;
    if classifier < -PARABOLA_REFUSE_BAND {
        // Ellipse (real nappe iff e·c > 0).
        if plane_offset * c <= 0.0 {
            return Ok(FaceModel::Empty);
        }
        let magnitude = classifier.abs();
        let center = apex + m * plane_offset + e1 * (plane_offset * c * in_plane_len / magnitude);
        let root = plane_offset * plane_offset * sine_squared * (1.0 - sine_squared) / magnitude;
        let (along_major, along_minor) = (root / magnitude, root / sine_squared);
        if along_major <= 0.0 || along_minor <= 0.0 {
            return Ok(FaceModel::Empty);
        }
        let ellipse =
            Ellipse3D::with_axes(center, m, along_major.sqrt(), along_minor.sqrt(), e1, e2)
                .map_err(|error| math_to_ops("cone ellipse section", error))?;
        let silhouettes = ellipse_silhouettes(&ellipse, direction);
        Ok(FaceModel::RayCast {
            carrier: Carrier::Ellipse(ellipse),
            clip_plane: (plane_point, m),
            silhouettes,
        })
    } else if classifier > PARABOLA_REFUSE_BAND {
        // Hyperbola, real branch along +e₁.
        let center = apex + m * plane_offset - e1 * (plane_offset * c * in_plane_len / classifier);
        let real = plane_offset.abs() * (sine_squared * (1.0 - sine_squared)).sqrt() / classifier;
        let imaginary = plane_offset.abs() * ((1.0 - sine_squared) / classifier).sqrt();
        let hyperbola = Hyperbola3D::with_axes(center, m, e1, real, imaginary)
            .map_err(|error| math_to_ops("cone hyperbola section", error))?;
        let mut silhouettes = Vec::new();
        if let Some(t) = hyperbola_silhouette(&hyperbola, m, direction) {
            silhouettes.push(hyperbola.evaluate(t));
        }
        Ok(FaceModel::RayCast {
            carrier: Carrier::Hyperbola(hyperbola),
            clip_plane: (plane_point, m),
            silhouettes,
        })
    } else if classifier.abs() <= PARABOLA_EXACT_BAND {
        // Parabola (real iff e·c > 0).
        if plane_offset * c <= 0.0 {
            return Ok(FaceModel::Empty);
        }
        let vertex = apex
            + m * plane_offset
            + e1 * (-plane_offset * (1.0 - 2.0 * sine_squared) / (2.0 * c * in_plane_len));
        let focal = plane_offset * c * in_plane_len / (2.0 * sine_squared);
        if !focal.is_finite() || focal <= 0.0 {
            return Ok(FaceModel::Empty);
        }
        let parabola = Parabola3D::with_axes(vertex, e1, e2, focal)
            .map_err(|error| math_to_ops("cone parabola section", error))?;
        let mut silhouettes = Vec::new();
        if let Some(t) = parabola_silhouette(&parabola, m, direction) {
            silhouettes.push(parabola.evaluate(t));
        }
        Ok(FaceModel::RayCast {
            carrier: Carrier::Parabola(parabola),
            clip_plane: (plane_point, m),
            silhouettes,
        })
    } else {
        Err(ProjectCurveError::NearParabolicSection {
            a_coefficient: classifier,
        })
    }
}

// ---------------------------------------------------------------------------
// Conic sources: plane-affine images and coaxial circles.
// ---------------------------------------------------------------------------

/// Outcome of conic-on-curved dispatch: exact, or approximation needed.
enum CurvedArcOutcome {
    Exact(FaceModel),
    NeedsApproximate,
}

/// Build the exact model for a circle/ellipse source on one face.
/// Errors are face-call refusals; solid callers turn per-face skips into
/// clipped coverage (see `face_model_for_solid`).
fn conic_model(
    circle: Option<&Circle3D>,
    ellipse: Option<&Ellipse3D>,
    direction: Vec3,
    target: &TargetGeom,
    scale: f64,
) -> Result<CurvedArcOutcome, ProjectCurveError> {
    if circle.is_none() && ellipse.is_none() {
        return Err(ProjectCurveError::Operations(
            OperationsError::InvalidInput {
                reason: "conic source is neither a circle nor an ellipse".to_string(),
            },
        ));
    }
    match target {
        TargetGeom::Plane { normal, delta } => {
            if normal.dot(direction).abs() <= GRAZE_PLANE {
                return Err(ProjectCurveError::GrazingDirection);
            }
            plane_conic_model(*normal, *delta, direction, circle, ellipse)
        }
        TargetGeom::Cylinder(_) | TargetGeom::Cone(_) | TargetGeom::Sphere(_) => {
            if ellipse.is_some() {
                return Err(ProjectCurveError::UnsupportedSourceCurve {
                    curve: "ellipse",
                    surface: target_tag(target),
                });
            }
            let Some(arc) = circle else {
                return Err(ProjectCurveError::Operations(
                    OperationsError::InvalidInput {
                        reason: "curved target needs a circle source".to_string(),
                    },
                ));
            };
            coaxial_or_approximate(arc, direction, target, scale)
        }
    }
}

/// Affine image of a circle/ellipse source on a plane target.
fn plane_conic_model(
    plane_normal: Vec3,
    delta: f64,
    direction: Vec3,
    circle: Option<&Circle3D>,
    ellipse: Option<&Ellipse3D>,
) -> Result<CurvedArcOutcome, ProjectCurveError> {
    let source = match (circle, ellipse) {
        (Some(c), None) => ConicFrame {
            center: c.center(),
            normal: c.normal(),
            u: c.u_axis(),
            v: c.v_axis(),
            ru: c.radius(),
            rv: c.radius(),
        },
        (None, Some(e)) => ConicFrame {
            center: e.center(),
            normal: e.normal(),
            u: e.u_axis(),
            v: e.v_axis(),
            ru: e.semi_major(),
            rv: e.semi_minor(),
        },
        _ => {
            return Err(ProjectCurveError::Operations(
                OperationsError::InvalidInput {
                    reason: "conic source is neither a circle nor an ellipse".to_string(),
                },
            ));
        }
    };
    if source.normal.dot(direction).abs() <= DEGENERATE_CONIC {
        return Err(ProjectCurveError::DegenerateImage);
    }
    let map = AffineMap::new(plane_normal, delta, direction);
    let (carrier, _a_vec, _b_vec) = affine_conic_image(&map, &source, plane_normal)?;
    // Boundary equations must use the same frame as collect_candidates.
    let frame = PlaneFrame2D::of(
        plane_normal,
        Point3::new(0.0, 0.0, 0.0) + plane_normal * delta,
    )?;
    let conic = carrier_to_conic2d(&carrier, &frame)?;
    Ok(CurvedArcOutcome::Exact(FaceModel::AffineConic {
        carrier,
        conic,
    }))
}

/// Express an affine image carrier in a target-plane 2D frame.
fn carrier_to_conic2d(
    carrier: &Carrier,
    frame: &PlaneFrame2D,
) -> Result<Conic2D, ProjectCurveError> {
    match carrier {
        Carrier::Line { p0, p1 } => {
            let a = frame.to_2d(*p0);
            let b = frame.to_2d(*p1);
            let direction = Vec2::new(b.x() - a.x(), b.y() - a.y());
            if direction.length() <= 0.0 {
                return Err(ProjectCurveError::DegenerateImage);
            }
            Ok(Conic2D::Line {
                point: a,
                direction,
            })
        }
        Carrier::Circle(c) => Ok(Conic2D::Circle {
            center: frame.to_2d(c.center()),
            radius: c.radius(),
        }),
        Carrier::Ellipse(e) => {
            let major_2d = {
                let tip = frame.to_2d(e.center() + e.u_axis());
                let org = frame.to_2d(e.center());
                Vec2::new(tip.x() - org.x(), tip.y() - org.y())
            };
            Ok(Conic2D::Ellipse {
                center: frame.to_2d(e.center()),
                semi_major: e.semi_major(),
                semi_minor: e.semi_minor(),
                rotation: major_2d.y().atan2(major_2d.x()),
            })
        }
        Carrier::Hyperbola(_) | Carrier::Parabola(_) => Err(ProjectCurveError::Operations(
            OperationsError::InvalidInput {
                reason: "affine images on planes are lines, circles or ellipses".to_string(),
            },
        )),
    }
}

/// Coaxial arc on a curved target (design note §4.3), or the approximate cell.
fn coaxial_or_approximate(
    arc: &Circle3D,
    direction: Vec3,
    target: &TargetGeom,
    scale: f64,
) -> Result<CurvedArcOutcome, ProjectCurveError> {
    if arc.normal().dot(direction).abs() <= DEGENERATE_CONIC {
        return Err(ProjectCurveError::DegenerateImage);
    }
    let coaxial_normal = arc.normal().cross(direction).length() <= COAXIAL_BAND;
    match target {
        TargetGeom::Plane { .. } => Err(ProjectCurveError::Operations(
            OperationsError::InvalidInput {
                reason: "plane targets take the affine path".to_string(),
            },
        )),
        TargetGeom::Cylinder(cyl) => {
            if cyl.axis().cross(direction).length() <= GRAZE_CYLINDER {
                return Err(ProjectCurveError::GrazingDirection);
            }
            // A non-grazing arc on a cylinder is never coaxial in this
            // slice (coaxial needs d̂ ∥ axis, which grazes).
            Ok(CurvedArcOutcome::NeedsApproximate)
        }
        TargetGeom::Sphere(sph) => {
            let axial = (arc.center() - sph.center()).dot(direction);
            let radial = (arc.center() - sph.center()) - direction * axial;
            if !coaxial_normal || radial.length() > COAXIAL_BAND * scale {
                return Ok(CurvedArcOutcome::NeedsApproximate);
            }
            let radius = sph.radius();
            let arc_radius = arc.radius();
            if (arc_radius - radius).abs() <= TANGENT_BAND * scale {
                return Err(ProjectCurveError::TangentSection);
            }
            if arc_radius > radius {
                return Ok(CurvedArcOutcome::Exact(FaceModel::Empty));
            }
            let height = (radius * radius - arc_radius * arc_radius).max(0.0).sqrt();
            let Some(lambda) = [-axial - height, -axial + height]
                .into_iter()
                .find(|lambda| *lambda >= 0.0)
            else {
                return Ok(CurvedArcOutcome::Exact(FaceModel::Empty));
            };
            let center = arc.center() + direction * lambda;
            let image =
                Circle3D::with_axes(center, arc.normal(), arc_radius, arc.u_axis(), arc.v_axis())
                    .map_err(|error| math_to_ops("coaxial sphere circle", error))?;
            // Pure translation along d̂: no silhouette; the clip plane is
            // perpendicular to the projection direction through the image.
            Ok(CurvedArcOutcome::Exact(FaceModel::RayCast {
                carrier: Carrier::Circle(image),
                clip_plane: (center, direction),
                silhouettes: Vec::new(),
            }))
        }
        TargetGeom::Cone(cone) => {
            let axis = cone.axis();
            if !coaxial_normal || axis.cross(direction).length() > COAXIAL_BAND {
                return Ok(CurvedArcOutcome::NeedsApproximate);
            }
            let axial = (arc.center() - cone.apex()).dot(axis);
            let radial = (arc.center() - cone.apex()) - axis * axial;
            if radial.length() > COAXIAL_BAND * scale {
                return Ok(CurvedArcOutcome::NeedsApproximate);
            }
            let distance = arc.radius() * cone.half_angle().tan();
            if !distance.is_finite() || distance <= 0.0 {
                return Ok(CurvedArcOutcome::Exact(FaceModel::Empty));
            }
            let center = cone.apex() + axis * distance;
            let image = Circle3D::with_axes(
                center,
                arc.normal(),
                arc.radius(),
                arc.u_axis(),
                arc.v_axis(),
            )
            .map_err(|error| math_to_ops("coaxial cone circle", error))?;
            Ok(CurvedArcOutcome::Exact(FaceModel::RayCast {
                carrier: Carrier::Circle(image),
                clip_plane: (center, direction),
                silhouettes: Vec::new(),
            }))
        }
    }
}

// ---------------------------------------------------------------------------
// Ray casts: first root on a support surface, first hit on faces.
// ---------------------------------------------------------------------------

/// Closed-form ray ∩ support-surface roots, ascending.
fn ray_roots(target: &TargetGeom, point: Point3, direction: Vec3) -> Vec<f64> {
    match target {
        TargetGeom::Plane { normal, delta } => {
            let denom = normal.dot(direction);
            if denom.abs() < 1e-15 {
                Vec::new()
            } else {
                vec![(*delta - normal.dot(point - Point3::new(0.0, 0.0, 0.0))) / denom]
            }
        }
        TargetGeom::Cylinder(cyl) => {
            let axis = cyl.axis();
            let w = point - cyl.origin();
            let dp = direction - axis * direction.dot(axis);
            let wp = w - axis * w.dot(axis);
            quadratic_roots(
                dp.dot(dp),
                2.0 * wp.dot(dp),
                wp.dot(wp) - cyl.radius() * cyl.radius(),
            )
        }
        TargetGeom::Sphere(sph) => {
            let w = point - sph.center();
            quadratic_roots(
                1.0,
                2.0 * w.dot(direction),
                w.dot(w) - sph.radius() * sph.radius(),
            )
        }
        TargetGeom::Cone(cone) => {
            let axis = cone.axis();
            let sine_squared = cone.half_angle().sin().powi(2);
            let w = point - cone.apex();
            let (axial_w, axial_d) = (w.dot(axis), direction.dot(axis));
            let a = axial_d.mul_add(axial_d, -sine_squared * direction.dot(direction));
            let b = 2.0 * axial_w.mul_add(axial_d, -sine_squared * w.dot(direction));
            let c = axial_w.mul_add(axial_w, -sine_squared * w.dot(w));
            quadratic_roots(a, b, c)
                .into_iter()
                .filter(|lambda| {
                    (w + direction * *lambda).dot(axis) >= -1e-12 * w.length().max(1.0)
                })
                .collect()
        }
    }
}

fn planar_loop_inside(
    topo: &Topology,
    wire: remus_topology::wire::WireId,
    point: Point3,
    normal: Vec3,
) -> Result<bool, ProjectCurveError> {
    let edges = topo.wire(wire).map_err(OperationsError::from)?.edges();
    if edges.len() == 1 {
        let data = topo.edge(edges[0].edge()).map_err(OperationsError::from)?;
        if data
            .strict_domain()
            .is_ok_and(|range| is_full_turn((range.0.min(range.1), range.0.max(range.1))))
        {
            match data.curve() {
                EdgeCurve::Circle(c) => {
                    let radial = point - c.center();
                    return Ok(radial.dot(c.u_axis()).hypot(radial.dot(c.v_axis())) <= c.radius());
                }
                EdgeCurve::Ellipse(e) => {
                    let radial = point - e.center();
                    return Ok((radial.dot(e.u_axis()) / e.semi_major())
                        .hypot(radial.dot(e.v_axis()) / e.semi_minor())
                        <= 1.0);
                }
                EdgeCurve::Line
                | EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => {}
            }
        }
    }
    let frame = PlaneFrame2D::of(normal, point)?;
    let mut polygon = Vec::new();
    for oriented in edges {
        let data = topo.edge(oriented.edge()).map_err(OperationsError::from)?;
        if !matches!(data.curve(), EdgeCurve::Line) {
            return Err(ProjectCurveError::Operations(
                OperationsError::InvalidInput {
                    reason:
                        "exact planar trim membership requires line polygons or whole-conic loops"
                            .to_string(),
                },
            ));
        }
        let vertex = topo
            .vertex(oriented.oriented_start(data))
            .map_err(OperationsError::from)?
            .point();
        polygon.push(frame.to_2d(vertex));
    }
    Ok(remus_math::predicates::point_in_polygon(
        Point2::new(0.0, 0.0),
        &polygon,
    ))
}

/// Recognize the full-period axial slabs used by primitive cylinders and
/// cones. Other trim families stay refused until their membership is
/// algebraically qualified; the sampled UV classifier is not a certificate.
fn full_axial_trim(
    topo: &Topology,
    face: FaceId,
    origin: Point3,
    axis: Vec3,
) -> Result<bool, ProjectCurveError> {
    let data = topo.face(face).map_err(OperationsError::from)?;
    if !data.inner_wires().is_empty() {
        return Ok(false);
    }
    let seams = seam_edges(topo, face)?;
    let boundaries = boundary_edges(topo, face)?;
    let mut rims = 0usize;
    let mut lines = 0usize;
    for boundary in boundaries {
        match boundary.curve {
            EdgeCurve::Circle(circle) => {
                let offset = circle.center() - origin;
                if !is_full_turn((
                    boundary.trim.0.min(boundary.trim.1),
                    boundary.trim.0.max(boundary.trim.1),
                )) || circle.normal().cross(axis).length() > COAXIAL_BAND
                    || (offset - axis * offset.dot(axis)).length()
                        > TANGENT_BAND * offset.length().max(circle.radius())
                {
                    return Ok(false);
                }
                rims += 1;
            }
            EdgeCurve::Line if seams.contains(&boundary.id) => lines += 1,
            EdgeCurve::Line
            | EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => return Ok(false),
        }
    }
    Ok((1..=2).contains(&rims) && lines == 1 && seams.len() == 1)
}

fn sphere_trim_rim(
    topo: &Topology,
    face: FaceId,
    sphere: &SphericalSurface,
) -> Result<Option<Circle3D>, ProjectCurveError> {
    let data = topo.face(face).map_err(OperationsError::from)?;
    if !data.inner_wires().is_empty() {
        return Ok(None);
    }
    let edges = topo
        .face_oriented_edges(face)
        .map_err(OperationsError::from)?;
    if edges.len() == 1 {
        let edge = topo.edge(edges[0].edge()).map_err(OperationsError::from)?;
        if let EdgeCurve::Circle(circle) = edge.curve()
            && edge
                .strict_domain()
                .is_ok_and(|range| is_full_turn((range.0.min(range.1), range.0.max(range.1))))
        {
            return Ok(Some(circle.clone()));
        }
    }
    let mut vertices = Vec::new();
    let mut normal = Vec3::new(0.0, 0.0, 0.0);
    for oriented in edges {
        let edge = topo.edge(oriented.edge()).map_err(OperationsError::from)?;
        if !matches!(edge.curve(), EdgeCurve::Line) {
            return Ok(None);
        }
        let a = topo
            .vertex(oriented.oriented_start(edge))
            .map_err(OperationsError::from)?
            .point();
        let b = topo
            .vertex(oriented.oriented_end(edge))
            .map_err(OperationsError::from)?
            .point();
        normal += (a - sphere.center()).cross(b - sphere.center());
        vertices.push(a);
    }
    let Ok(normal) = normal.normalize() else {
        return Ok(None);
    };
    if vertices.len() < 3
        || vertices.iter().any(|point| {
            (normal.dot(*point - sphere.center())).abs() > TANGENT_BAND * sphere.radius()
                || ((*point - sphere.center()).length() - sphere.radius()).abs()
                    > TANGENT_BAND * sphere.radius()
        })
    {
        return Ok(None);
    }
    let mut turn = 0.0;
    for k in 0..vertices.len() {
        let a = vertices[k] - sphere.center();
        let b = vertices[(k + 1) % vertices.len()] - sphere.center();
        let angle = normal.dot(a.cross(b)).atan2(a.dot(b));
        if angle <= 0.0 {
            return Ok(None);
        }
        turn += angle;
    }
    if (turn - TAU).abs() > PARAM_ABS {
        return Ok(None);
    }
    Circle3D::new(sphere.center(), normal, sphere.radius())
        .map(Some)
        .map_err(|error| math_to_ops("spherical hemisphere trim", error))
}

fn in_region(topo: &Topology, face: FaceId, point: Point3) -> Result<bool, ProjectCurveError> {
    let data = topo.face(face).map_err(OperationsError::from)?;
    if let FaceSurface::Plane { normal, .. } = data.surface() {
        if !planar_loop_inside(topo, data.outer_wire(), point, *normal)? {
            return Ok(false);
        }
        for hole in data.inner_wires() {
            if planar_loop_inside(topo, *hole, point, *normal)? {
                return Ok(false);
            }
        }
        return Ok(true);
    }
    let qualified = match data.surface() {
        FaceSurface::Cylinder(c) => full_axial_trim(topo, face, c.origin(), c.axis())?,
        FaceSurface::Cone(c) => full_axial_trim(topo, face, c.apex(), c.axis())?,
        FaceSurface::Sphere(s) => sphere_trim_rim(topo, face, s)?.is_some(),
        FaceSurface::Plane { .. } | FaceSurface::Nurbs(_) | FaceSurface::Torus(_) => false,
    };
    if !qualified {
        return Err(ProjectCurveError::Operations(
            OperationsError::InvalidInput {
                reason: "exact projection trim membership is not qualified for this face"
                    .to_string(),
            },
        ));
    }
    remus_check::classify::surface_point_in_face(topo, face, point)
        .map_err(OperationsError::Check)
        .map_err(ProjectCurveError::Operations)
}

/// Distance from a point to a boundary edge (exact for lines and circles,
/// projection-based for ellipses, sampled otherwise).
fn point_to_boundary_edge(
    topo: &Topology,
    edge: EdgeId,
    point: Point3,
) -> Result<f64, ProjectCurveError> {
    let data = topo.edge(edge).map_err(OperationsError::from)?;
    let start = topo
        .vertex(data.start())
        .map_err(OperationsError::from)?
        .point();
    let end = topo
        .vertex(data.end())
        .map_err(OperationsError::from)?
        .point();
    match data.curve() {
        EdgeCurve::Line => {
            let axis = end - start;
            let length_squared = axis.dot(axis);
            if length_squared <= 0.0 {
                return Ok((point - start).length());
            }
            let t = ((point - start).dot(axis) / length_squared).clamp(0.0, 1.0);
            Ok((point - (start + axis * t)).length())
        }
        EdgeCurve::Circle(circle) => {
            // Distance to the ring: radial gap and axial gap combined.
            let axial = (point - circle.center()).dot(circle.normal());
            let radial = (point - circle.center()) - circle.normal() * axial;
            let radial_length = radial.length();
            if radial_length <= 1e-15 {
                Ok(axial.abs().hypot(circle.radius()))
            } else {
                Ok((radial_length - circle.radius()).hypot(axial))
            }
        }
        EdgeCurve::Ellipse(ellipse) => {
            let t = ellipse.project(point);
            Ok((point - ellipse.evaluate(t)).length())
        }
        curve => {
            let (t0, t1) = data.strict_domain().map_err(|_| {
                ProjectCurveError::Operations(OperationsError::InvalidInput {
                    reason: "boundary edge has no valid parameter range".to_string(),
                })
            })?;
            let _ = curve;
            let mut best = f64::INFINITY;
            for k in 0..=PROBE_SAMPLES {
                let t = t0 + (t1 - t0) * (k as f64 / PROBE_SAMPLES as f64);
                best = best
                    .min((point - data.curve().evaluate_with_endpoints(t, start, end)).length());
            }
            Ok(best)
        }
    }
}

/// Relaxed membership: the strict trim test, or within the weld band of a
/// boundary edge (covers images ending exactly on a boundary).
fn in_region_relaxed(
    topo: &Topology,
    face: FaceId,
    point: Point3,
    scale: f64,
) -> Result<bool, ProjectCurveError> {
    if in_region(topo, face, point)? {
        return Ok(true);
    }
    let oriented = topo
        .face_oriented_edges(face)
        .map_err(OperationsError::from)?;
    for oriented_edge in &oriented {
        if point_to_boundary_edge(topo, oriented_edge.edge(), point)? <= TANGENT_BAND * scale {
            return Ok(true);
        }
    }
    Ok(false)
}

/// First hit of a ray restricted to one face: smallest `λ ≥ 0` whose point
/// lies in the trimmed region.
fn hit_on_face(
    topo: &Topology,
    face: FaceId,
    target: &TargetGeom,
    point: Point3,
    direction: Vec3,
    scale: f64,
) -> Result<Option<Point3>, ProjectCurveError> {
    for lambda in ray_roots(target, point, direction) {
        if lambda < -PARAM_DEDUPE * scale {
            continue;
        }
        let hit = point + direction * lambda.max(0.0);
        if in_region_relaxed(topo, face, hit, scale)? {
            return Ok(Some(hit));
        }
    }
    Ok(None)
}

/// First hit across faces: smallest `λ ≥ 0` in-region hit on any face.
fn first_hit(
    topo: &Topology,
    faces: &[(FaceId, &TargetGeom, f64)],
    point: Point3,
    direction: Vec3,
) -> Result<Option<(Point3, usize)>, ProjectCurveError> {
    let mut best: Option<(f64, Point3, usize)> = None;
    for (index, (face, target, scale)) in faces.iter().enumerate() {
        for lambda in ray_roots(target, point, direction) {
            if lambda < -PARAM_DEDUPE * scale {
                continue;
            }
            let hit = point + direction * lambda.max(0.0);
            if in_region_relaxed(topo, *face, hit, *scale)?
                && best
                    .as_ref()
                    .is_none_or(|(previous, _, _)| lambda < *previous)
            {
                best = Some((lambda, hit, index));
            }
        }
    }
    Ok(best.map(|(_, hit, index)| (hit, index)))
}

// ---------------------------------------------------------------------------
// Source evaluation and pushback (3D image point → source parameter).
// ---------------------------------------------------------------------------

/// Source point at parameter `s` (in the source edge's domain).
fn source_point(source: &ResolvedSource, s: f64) -> Point3 {
    match &source.geom {
        SourceGeom::Segment { a, b } => *a + (*b - *a) * s,
        SourceGeom::Circle { circle, .. } => circle.evaluate(s),
        SourceGeom::Ellipse { ellipse, .. } => ellipse.evaluate(s),
        SourceGeom::Other { .. } => Point3::new(0.0, 0.0, 0.0),
    }
}

/// Unwrap a raw periodic parameter into `[anchor, anchor + TAU)`.
fn unwrap_periodic(raw: f64, anchor: f64) -> f64 {
    anchor + (raw - anchor).rem_euclid(TAU)
}

/// Invert the source map for an on-image point: the source parameter `s`
/// whose image is `point`, clamped into the domain with a `PARAM_ABS` slack.
/// Returns `None` when the point pulls back outside the source domain.
fn pushback(source: &ResolvedSource, direction: Vec3, point: Point3) -> Option<f64> {
    let (lo, hi) = source.domain;
    match &source.geom {
        SourceGeom::Segment { a, b } => {
            let span = *b - *a;
            let m = span.cross(direction);
            let norm_squared = m.dot(m);
            // A parallel source is refused upstream; a zero cross here
            // means the direction is degenerate.
            if norm_squared <= 0.0 {
                return None;
            }
            let normal = m * (1.0 / norm_squared.sqrt());
            let cross = direction.cross(normal);
            let denom = span.dot(cross);
            if denom.abs() <= 1e-30 {
                return None;
            }
            let s = (point - *a).dot(cross) / denom;
            if s >= lo - PARAM_ABS && s <= hi + PARAM_ABS {
                Some(s.clamp(lo, hi))
            } else {
                None
            }
        }
        SourceGeom::Circle { circle, .. } => {
            let normal = circle.normal();
            let denom = direction.dot(normal);
            if denom.abs() <= 1e-15 {
                return None;
            }
            let lambda = (point - circle.center()).dot(normal) / denom;
            let back = point - direction * lambda;
            let raw = circle.project(back);
            pushback_periodic(raw, source)
        }
        SourceGeom::Ellipse { ellipse, .. } => {
            let normal = ellipse.normal();
            let denom = direction.dot(normal);
            if denom.abs() <= 1e-15 {
                return None;
            }
            let lambda = (point - ellipse.center()).dot(normal) / denom;
            let back = point - direction * lambda;
            let raw = ellipse.project(back);
            pushback_periodic(raw, source)
        }
        SourceGeom::Other { .. } => None,
    }
}

fn pushback_periodic(raw: f64, source: &ResolvedSource) -> Option<f64> {
    let (lo, hi) = source.domain;
    let lifted = unwrap_periodic(raw, lo);
    if source.closed {
        Some(lifted)
    } else if lifted >= lo - PARAM_ABS && lifted <= hi + PARAM_ABS {
        Some(lifted.clamp(lo, hi))
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// Boundary edges and split-parameter candidates.
// ---------------------------------------------------------------------------

/// A face boundary edge with its domain.
struct BoundaryEdge {
    id: EdgeId,
    curve: EdgeCurve,
    start: Point3,
    end: Point3,
    trim: (f64, f64),
}

fn boundary_edges(topo: &Topology, face: FaceId) -> Result<Vec<BoundaryEdge>, ProjectCurveError> {
    let oriented = topo
        .face_oriented_edges(face)
        .map_err(OperationsError::from)?;
    let mut edges = Vec::new();
    for oriented_edge in &oriented {
        let id = oriented_edge.edge();
        if edges.iter().any(|edge: &BoundaryEdge| edge.id == id) {
            continue;
        }
        let data = topo.edge(id).map_err(OperationsError::from)?;
        let start = topo
            .vertex(data.start())
            .map_err(OperationsError::from)?
            .point();
        let end = topo
            .vertex(data.end())
            .map_err(OperationsError::from)?
            .point();
        let trim = match data.curve() {
            EdgeCurve::Line => (0.0, 1.0),
            EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Circle(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => data.strict_domain().map_err(|_| {
                ProjectCurveError::Operations(OperationsError::InvalidInput {
                    reason: "face boundary edge has no valid parameter range".to_string(),
                })
            })?,
        };
        edges.push(BoundaryEdge {
            id,
            curve: data.curve().clone(),
            start,
            end,
            trim,
        });
    }
    Ok(edges)
}

/// Map an edge parameter into its trim window (periodic edges accept any
/// angle, folded into the stored window).
fn in_trim(t: f64, trim: (f64, f64)) -> Option<f64> {
    let (t0, t1) = (trim.0.min(trim.1), trim.0.max(trim.1));
    if (t1 - t0 - TAU).abs() <= 1e-9 {
        return Some(unwrap_periodic(t, t0));
    }
    let lifted = unwrap_periodic(t, t0);
    if lifted >= t0 - PARAM_ABS && lifted <= t1 + PARAM_ABS {
        Some(lifted.clamp(t0, t1))
    } else {
        None
    }
}

/// Intersections of a boundary edge with a plane: `(edge_t, point)`.
fn edge_plane_crossings(
    edge: &BoundaryEdge,
    plane_point: Point3,
    m: Vec3,
) -> Result<Vec<(f64, Point3)>, ProjectCurveError> {
    let offset = |point: Point3| m.dot(point - plane_point);
    let roots = match &edge.curve {
        EdgeCurve::Line => {
            let denom = m.dot(edge.end - edge.start);
            if denom.abs() <= 1e-15 * (edge.end - edge.start).length().max(1e-300) {
                return Ok(Vec::new());
            }
            let t = -offset(edge.start) / denom;
            if (-PARAM_DEDUPE..=1.0 + PARAM_DEDUPE).contains(&t) {
                let clamped = t.clamp(0.0, 1.0);
                vec![(clamped, edge.start + (edge.end - edge.start) * clamped)]
            } else {
                Vec::new()
            }
        }
        EdgeCurve::Circle(circle) => {
            let alpha = circle.radius() * m.dot(circle.u_axis());
            let beta = circle.radius() * m.dot(circle.v_axis());
            let gamma = m.dot(plane_point - circle.center());
            trig_roots(alpha, beta, gamma)
                .into_iter()
                .filter_map(|t| in_trim(t, edge.trim).map(|t| (t, circle.evaluate(t))))
                .collect()
        }
        EdgeCurve::Ellipse(ellipse) => {
            let alpha = ellipse.semi_major() * m.dot(ellipse.u_axis());
            let beta = ellipse.semi_minor() * m.dot(ellipse.v_axis());
            let gamma = m.dot(plane_point - ellipse.center());
            trig_roots(alpha, beta, gamma)
                .into_iter()
                .filter_map(|t| in_trim(t, edge.trim).map(|t| (t, ellipse.evaluate(t))))
                .collect()
        }
        EdgeCurve::Parabola(parabola) => quadratic_roots(
            m.dot(parabola.axis_dir()) / (4.0 * parabola.focal_length()),
            m.dot(parabola.u_axis()),
            offset(parabola.vertex()),
        )
        .into_iter()
        .filter(|t| *t >= edge.trim.0 - PARAM_ABS && *t <= edge.trim.1 + PARAM_ABS)
        .map(|t| (t, parabola.evaluate(t)))
        .collect(),
        EdgeCurve::Hyperbola(hyperbola) => {
            let a = hyperbola.semi_major() * m.dot(hyperbola.u_axis());
            let b = hyperbola.semi_minor() * m.dot(hyperbola.v_axis());
            quadratic_roots(a + b, 2.0 * offset(hyperbola.center()), a - b)
                .into_iter()
                .filter(|root| *root > 0.0)
                .map(f64::ln)
                .filter(|t| *t >= edge.trim.0 - PARAM_ABS && *t <= edge.trim.1 + PARAM_ABS)
                .map(|t| (t, hyperbola.evaluate(t)))
                .collect()
        }
        curve @ EdgeCurve::NurbsCurve(_) => return Err(uncertified_boundary(curve)),
    };
    Ok(roots)
}

fn uncertified_boundary(curve: &EdgeCurve) -> ProjectCurveError {
    ProjectCurveError::Operations(OperationsError::InvalidInput {
        reason: format!(
            "exact projection clipping of {} boundary curves is not qualified",
            curve.type_tag()
        ),
    })
}

/// Roots of `α·cos t + β·sin t = γ`.
fn trig_roots(alpha: f64, beta: f64, gamma: f64) -> Vec<f64> {
    let radius = alpha.hypot(beta);
    if radius <= 0.0 {
        return Vec::new();
    }
    let ratio = gamma / radius;
    if ratio.abs() > 1.0 + 1e-15 {
        return Vec::new();
    }
    let base = beta.atan2(alpha);
    if (ratio.abs() - 1.0).abs() <= 1e-15 {
        return vec![
            base + if ratio > 0.0 {
                0.0
            } else {
                std::f64::consts::PI
            },
        ];
    }
    let delta = ratio.clamp(-1.0, 1.0).acos();
    vec![base + delta, base - delta]
}

/// Isolate every real root of a bounded polynomial by partitioning at
/// all derivative roots. Each open partition is monotone, so sign tests
/// cannot lose a close root pair between samples. Repeated roots occur
/// at the partition endpoints and are retained too.
fn polynomial_roots_in(coefficients: &[f64], lo: f64, hi: f64) -> Vec<f64> {
    let mut coefficients = coefficients.to_vec();
    while coefficients.len() > 1 && coefficients.last() == Some(&0.0) {
        coefficients.pop();
    }
    if coefficients.len() <= 1 {
        return Vec::new();
    }
    let magnitude = coefficients.iter().map(|x| x.abs()).fold(0.0, f64::max);
    if !magnitude.is_finite() || magnitude == 0.0 {
        return Vec::new();
    }
    for value in &mut coefficients {
        *value /= magnitude;
    }
    let evaluate = |t: f64| {
        coefficients
            .iter()
            .rev()
            .fold(0.0_f64, |v, c| v.mul_add(t, *c))
    };
    if coefficients.len() == 2 {
        let root = -coefficients[0] / coefficients[1];
        return if root >= lo && root <= hi {
            vec![root]
        } else {
            Vec::new()
        };
    }
    let derivative: Vec<f64> = coefficients
        .iter()
        .enumerate()
        .skip(1)
        .map(|(power, value)| power as f64 * value)
        .collect();
    let mut cuts = vec![lo];
    cuts.extend(polynomial_roots_in(&derivative, lo, hi));
    cuts.push(hi);
    cuts.sort_by(f64::total_cmp);
    let residual_guard = 64.0 * f64::EPSILON * coefficients.iter().map(|c| c.abs()).sum::<f64>();
    let mut roots: Vec<f64> = cuts
        .iter()
        .copied()
        .filter(|t| evaluate(*t).abs() <= residual_guard)
        .collect();
    for window in cuts.windows(2) {
        let (mut a, mut b) = (window[0], window[1]);
        let mut fa = evaluate(a);
        let fb = evaluate(b);
        if fa == 0.0 || fb == 0.0 || fa.signum() == fb.signum() {
            continue;
        }
        for _ in 0..64 {
            let mid = 0.5 * (a + b);
            let fm = evaluate(mid);
            if fm == 0.0 {
                a = mid;
                b = mid;
                break;
            }
            if fm.signum() == fa.signum() {
                a = mid;
                fa = fm;
            } else {
                b = mid;
            }
        }
        roots.push(0.5 * (a + b));
    }
    roots.sort_by(f64::total_cmp);
    roots.dedup_by(|a, b| (*a - *b).abs() <= PARAM_DEDUPE);
    roots
}

/// Circle/ellipse boundary composed with a conic is a quartic in
/// tan(t/2). Use quarter-turn windows to keep that variable bounded and
/// avoid its pole, then isolate all roots algebraically.
fn conic_angular_crossings(
    edge: &BoundaryEdge,
    frame: &PlaneFrame2D,
    conic: &Conic2D,
) -> Result<Option<Vec<(f64, Point3)>>, ProjectCurveError> {
    let (center, u, v) = match &edge.curve {
        EdgeCurve::Circle(c) => (c.center(), c.u_axis() * c.radius(), c.v_axis() * c.radius()),
        EdgeCurve::Ellipse(e) => (
            e.center(),
            e.u_axis() * e.semi_major(),
            e.v_axis() * e.semi_minor(),
        ),
        EdgeCurve::Line
        | EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => return Ok(None),
    };
    let mut roots = Vec::new();
    let (mut lo, end) = (edge.trim.0.min(edge.trim.1), edge.trim.0.max(edge.trim.1));
    for _ in 0..8 {
        if lo >= end {
            break;
        }
        let hi = (lo + std::f64::consts::FRAC_PI_2).min(end);
        if hi <= lo {
            return Err(ProjectCurveError::Operations(OperationsError::InvalidInput {
                reason: "periodic boundary parameters cannot be partitioned within the clipping budget".to_string(),
            }));
        }
        let mid = 0.5 * (lo + hi);
        let (sin, cos) = mid.sin_cos();
        let a = u * cos + v * sin;
        let b = v * cos - u * sin;
        let f = |p| conic.evaluate(frame.to_2d(p));
        let constant = f(center);
        let ac = 0.5 * (f(center + a) - f(center - a));
        let bs = 0.5 * (f(center + b) - f(center - b));
        let aa = 0.5 * (f(center + a) + f(center - a)) - constant;
        let bb = 0.5 * (f(center + b) + f(center - b)) - constant;
        let ab = f(center + a + b) - constant - ac - bs - aa - bb;
        let coefficients = [
            constant + ac + aa,
            2.0 * (bs + ab),
            2.0 * constant - 2.0 * aa + 4.0 * bb,
            2.0 * (bs - ab),
            constant - ac + aa,
        ];
        if coefficients.iter().any(|value| !value.is_finite()) {
            return Err(ProjectCurveError::Operations(
                OperationsError::InvalidInput {
                    reason: "conic boundary polynomial is not finite".to_string(),
                },
            ));
        }
        let limit = (0.25 * (hi - lo)).tan();
        for root in polynomial_roots_in(&coefficients, -limit, limit) {
            let t = mid + 2.0 * root.atan();
            let point = edge.curve.evaluate_with_endpoints(t, edge.start, edge.end);
            roots.push((t, point));
        }
        lo = hi;
    }
    if lo < end {
        return Err(ProjectCurveError::Operations(
            OperationsError::InvalidInput {
                reason: "periodic boundary clipping budget exhausted".to_string(),
            },
        ));
    }
    Ok(Some(roots))
}

/// Roots of the image-conic implicit equation composed with a boundary edge.
fn edge_conic_crossings(
    edge: &BoundaryEdge,
    frame: &PlaneFrame2D,
    conic: &Conic2D,
) -> Result<Vec<(f64, Point3)>, ProjectCurveError> {
    let roots = if matches!(&edge.curve, EdgeCurve::Line) {
        let a = frame.to_2d(edge.start);
        let b = frame.to_2d(edge.end);
        let (f0, f1) = (conic.evaluate(a), conic.evaluate(b));
        if let Conic2D::Line { .. } = conic {
            let denom = f1 - f0;
            if denom.abs() <= 1e-300 {
                return Ok(Vec::new());
            }
            let t = -f0 / denom;
            if (-PARAM_DEDUPE..=1.0 + PARAM_DEDUPE).contains(&t) {
                let clamped = t.clamp(0.0, 1.0);
                vec![(clamped, edge.start + (edge.end - edge.start) * clamped)]
            } else {
                Vec::new()
            }
        } else {
            let mid2d = Point2::new(0.5 * (a.x() + b.x()), 0.5 * (a.y() + b.y()));
            let f05 = conic.evaluate(mid2d);
            // Quadratic through f(0), f(1/2), f(1).
            let c = f0;
            let b_coef = 4.0f64.mul_add(f05, -f1) - 3.0 * f0;
            let a_coef = 2.0f64.mul_add(f0, -4.0 * f05) + 2.0 * f1;
            quadratic_roots(a_coef, b_coef, c)
                .into_iter()
                .filter(|t| (-PARAM_DEDUPE..=1.0 + PARAM_DEDUPE).contains(t))
                .map(|t| {
                    let clamped = t.clamp(0.0, 1.0);
                    (clamped, edge.start + (edge.end - edge.start) * clamped)
                })
                .collect()
        }
    } else if let Some(roots) = conic_angular_crossings(edge, frame, conic)? {
        roots
    } else {
        return Err(uncertified_boundary(&edge.curve));
    };
    Ok(roots)
}

// ---------------------------------------------------------------------------
// Driver: split parameters, first-hit classification, pieces.
// ---------------------------------------------------------------------------

/// One face under projection with its model and boundary data.
struct FaceCtx {
    face: FaceId,
    target: TargetGeom,
    model: FaceModel,
    scale: f64,
    boundaries: Vec<BoundaryEdge>,
    seams: Vec<EdgeId>,
}

impl FaceCtx {
    fn build(
        topo: &Topology,
        face: FaceId,
        target: TargetGeom,
        model: FaceModel,
        scale: f64,
    ) -> Result<Self, ProjectCurveError> {
        let mut boundaries = boundary_edges(topo, face)?;
        let seams = seam_edges(topo, face)?;
        if let TargetGeom::Sphere(sphere) = &target
            && let Some(rim) = sphere_trim_rim(topo, face, sphere)?
        {
            // Primitive hemisphere wires contain straight equatorial
            // chords. Its exact trim is the great-circle rim, including
            // exact sweep-plane split events between stored vertices.
            let id = boundaries
                .first()
                .ok_or_else(|| {
                    ProjectCurveError::Operations(OperationsError::InvalidInput {
                        reason: "sphere face has no boundary".to_string(),
                    })
                })?
                .id;
            let point = rim.evaluate(0.0);
            boundaries = vec![BoundaryEdge {
                id,
                curve: EdgeCurve::Circle(rim),
                start: point,
                end: point,
                trim: (0.0, TAU),
            }];
        }
        Ok(Self {
            face,
            target,
            model,
            scale,
            boundaries,
            seams,
        })
    }
}

/// A classified image piece ready to commit.
struct PieceSpec {
    face: FaceId,
    carrier: Carrier,
    trim: (f64, f64),
    start: Point3,
    end: Point3,
    closed: bool,
    source_range: (f64, f64),
}

fn wrap_pi(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(TAU) - std::f64::consts::PI
}

/// Choose the carrier span `(u0, u1)` through the raw projections whose
/// midpoint matches the imaged midpoint (handles wraps and direction).
/// Ties (a span and its `±TAU` alias share a midpoint) break toward the
/// shorter span: open pieces never wind a full extra turn.
fn unwrap_with_mid(u0: f64, u1raw: f64, umraw: f64) -> (f64, f64) {
    let base = wrap_pi(u1raw - u0);
    let mut best = (u0, u0 + base);
    let mut best_key = (f64::INFINITY, f64::INFINITY);
    for k in -1..=1 {
        let u1 = u0 + base + (k as f64) * TAU;
        let mid = u0 + 0.5 * (u1 - u0);
        let key = (wrap_pi(mid - umraw).abs(), (u1 - u0).abs());
        if key.0 < best_key.0 - 1e-12 || (key.0 <= best_key.0 + 1e-12 && key.1 < best_key.1) {
            best_key = key;
            best = (u0, u1);
        }
    }
    best
}

/// Resolve an open piece trim from its imaged endpoints and midpoint.
fn resolve_trim_open(carrier: &Carrier, x0: Point3, x1: Point3, xm: Point3) -> (f64, f64) {
    match carrier {
        Carrier::Line { .. } => (0.0, 1.0),
        Carrier::Hyperbola(_) | Carrier::Parabola(_) => (carrier.project(x0), carrier.project(x1)),
        Carrier::Circle(_) | Carrier::Ellipse(_) => unwrap_with_mid(
            carrier.project(x0),
            carrier.project(x1),
            carrier.project(xm),
        ),
    }
}

/// Image point of a source parameter on one face context: the restricted
/// first hit (ray-cast models) or the affine map (plane-conic models).
fn image_at(
    topo: &Topology,
    ctx: &FaceCtx,
    source: &ResolvedSource,
    direction: Vec3,
    s: f64,
) -> Result<Option<Point3>, ProjectCurveError> {
    let point = source_point(source, s);
    match &ctx.model {
        FaceModel::Empty => Ok(None),
        FaceModel::RayCast { .. } => {
            hit_on_face(topo, ctx.face, &ctx.target, point, direction, ctx.scale)
        }
        FaceModel::AffineConic { .. } => {
            let TargetGeom::Plane { normal, delta } = &ctx.target else {
                return Err(ProjectCurveError::Operations(
                    OperationsError::InvalidInput {
                        reason: "affine image on a non-planar target".to_string(),
                    },
                ));
            };
            let map = AffineMap::new(*normal, *delta, direction);
            let lambda =
                (*delta - normal.dot(point - Point3::new(0.0, 0.0, 0.0))) / normal.dot(direction);
            if lambda < -PARAM_DEDUPE * ctx.scale {
                return Ok(None);
            }
            let image = map.apply(point);
            if in_region_relaxed(topo, ctx.face, image, ctx.scale)? {
                Ok(Some(image))
            } else {
                Ok(None)
            }
        }
    }
}

/// Sorted clip candidates: source parameter plus an exactly known image
/// point (boundary/silhouette crossing) when one exists.
type ClipCandidates = Vec<(f64, Option<Point3>)>;
/// Per-face seam splits: source parameter plus the seam crossing point.
type SeamCandidates = Vec<Vec<(f64, Point3)>>;

/// Collect clip candidates and per-face seam splits.
#[allow(clippy::too_many_lines)]
fn collect_candidates(
    source: &ResolvedSource,
    direction: Vec3,
    faces: &[FaceCtx],
) -> Result<(ClipCandidates, SeamCandidates), ProjectCurveError> {
    let mut clip: ClipCandidates = Vec::new();
    let mut seams: SeamCandidates = faces.iter().map(|_| Vec::new()).collect();
    for (index, ctx) in faces.iter().enumerate() {
        // A ray starts on its target when lambda crosses zero. These are
        // clipping events even when the projected carrier stays inside
        // every face boundary.
        match (&source.geom, &ctx.target) {
            (SourceGeom::Segment { a, b }, target) => {
                let span = *b - *a;
                let length = span.length();
                for distance in ray_roots(target, *a, span * (1.0 / length)) {
                    let s = distance / length;
                    if s >= source.domain.0 && s <= source.domain.1 {
                        clip.push((s, Some(source_point(source, s))));
                    }
                }
            }
            (SourceGeom::Circle { circle }, TargetGeom::Plane { normal, delta }) => {
                for raw in trig_roots(
                    circle.radius() * normal.dot(circle.u_axis()),
                    circle.radius() * normal.dot(circle.v_axis()),
                    *delta - normal.dot(circle.center() - Point3::new(0.0, 0.0, 0.0)),
                ) {
                    if let Some(s) = pushback_periodic(raw, source) {
                        clip.push((s, Some(source_point(source, s))));
                    }
                }
            }
            (SourceGeom::Ellipse { ellipse }, TargetGeom::Plane { normal, delta }) => {
                for raw in trig_roots(
                    ellipse.semi_major() * normal.dot(ellipse.u_axis()),
                    ellipse.semi_minor() * normal.dot(ellipse.v_axis()),
                    *delta - normal.dot(ellipse.center() - Point3::new(0.0, 0.0, 0.0)),
                ) {
                    if let Some(s) = pushback_periodic(raw, source) {
                        clip.push((s, Some(source_point(source, s))));
                    }
                }
            }
            _ => {}
        }
        match &ctx.model {
            FaceModel::Empty => {}
            FaceModel::RayCast {
                clip_plane,
                silhouettes,
                ..
            } => {
                let (plane_point, m) = *clip_plane;
                for edge in &ctx.boundaries {
                    let is_seam = ctx.seams.contains(&edge.id);
                    for (_, point) in edge_plane_crossings(edge, plane_point, m)? {
                        if let Some(s) = pushback(source, direction, point) {
                            clip.push((s, Some(point)));
                            if is_seam {
                                seams[index].push((s, point));
                            }
                        }
                    }
                }
                for point in silhouettes {
                    if let Some(s) = pushback(source, direction, *point) {
                        clip.push((s, Some(*point)));
                    }
                }
            }
            FaceModel::AffineConic { carrier, conic } => {
                let TargetGeom::Plane { normal, delta } = &ctx.target else {
                    return Err(ProjectCurveError::Operations(
                        OperationsError::InvalidInput {
                            reason: "affine image on a non-planar target".to_string(),
                        },
                    ));
                };
                let frame =
                    PlaneFrame2D::of(*normal, Point3::new(0.0, 0.0, 0.0) + *normal * *delta)?;
                let _ = carrier;
                for edge in &ctx.boundaries {
                    for (_, point) in edge_conic_crossings(edge, &frame, conic)? {
                        if let Some(s) = pushback(source, direction, point) {
                            clip.push((s, Some(point)));
                        }
                    }
                }
            }
        }
    }
    // Domain ends (no known point yet).
    clip.push((source.domain.0, None));
    clip.push((source.domain.1, None));
    // Sort, clamp into the domain, dedupe (keep known points).
    clip.sort_by(|a, b| a.0.total_cmp(&b.0));
    let (lo, hi) = source.domain;
    let mut merged: Vec<(f64, Option<Point3>)> = Vec::new();
    for (s, point) in clip {
        if s < lo - PARAM_ABS || s > hi + PARAM_ABS {
            continue;
        }
        let s = s.clamp(lo, hi);
        if let Some(last) = merged.last_mut()
            && (s - last.0).abs() <= PARAM_DEDUPE
        {
            if last.1.is_none() {
                last.1 = point;
            }
            continue;
        }
        merged.push((s, point));
    }
    for seam_list in &mut seams {
        seam_list.sort_by(|a, b| a.0.total_cmp(&b.0));
        seam_list.retain(|(s, _)| *s > lo + PARAM_DEDUPE && *s < hi - PARAM_DEDUPE);
    }
    Ok((merged, seams))
}

/// Classify every open interval between consecutive candidates by its
/// midpoint first hit; returns inside runs `(ctx, a, b)`.
fn classify_intervals(
    topo: &Topology,
    source: &ResolvedSource,
    direction: Vec3,
    faces: &[FaceCtx],
    candidates: &[(f64, Option<Point3>)],
) -> Result<Vec<(usize, f64, f64)>, ProjectCurveError> {
    let hit_faces: Vec<(FaceId, &TargetGeom, f64)> = faces
        .iter()
        .filter(|ctx| ctx.model.carrier().is_some())
        .map(|ctx| (ctx.face, &ctx.target, ctx.scale))
        .collect();
    let mut runs: Vec<(usize, f64, f64)> = Vec::new();
    for window in candidates.windows(2) {
        let (a, b) = (window[0].0, window[1].0);
        if b - a <= PARAM_DEDUPE {
            continue;
        }
        let mid = 0.5 * (a + b);
        let probe = source_point(source, mid);
        let winner = first_hit(topo, &hit_faces, probe, direction)?;
        let Some((hit, position)) = winner else {
            continue;
        };
        // Locate the winning context.
        let mut ctx_index = usize::MAX;
        for (index, ctx) in faces.iter().enumerate() {
            if ctx.face == hit_faces[position].0 {
                ctx_index = index;
                break;
            }
        }
        if ctx_index == usize::MAX {
            continue;
        }
        // An image running along a boundary edge over a positive length
        // makes the midpoint test meaningless: refuse.
        let ctx = &faces[ctx_index];
        for edge in &ctx.boundaries {
            if point_to_boundary_edge(topo, edge.id, hit)? <= TANGENT_BAND * ctx.scale {
                return Err(ProjectCurveError::ImageAlongBoundary);
            }
        }
        if let Some(last) = runs.last_mut()
            && last.0 == ctx_index
            && (a - last.2).abs() <= PARAM_DEDUPE
        {
            last.2 = b;
            continue;
        }
        runs.push((ctx_index, a, b));
    }
    Ok(runs)
}

/// Resolve a piece endpoint: the known candidate point when the live image
/// touches it (a boundary/seam crossing is exactly on the image), else the
/// live restricted image at that source parameter.
fn endpoint_point(
    topo: &Topology,
    ctx: &FaceCtx,
    source: &ResolvedSource,
    direction: Vec3,
    s: f64,
    known: Option<Point3>,
) -> Result<Point3, ProjectCurveError> {
    let live = image_at(topo, ctx, source, direction, s)?;
    match (live, known) {
        (Some(q), Some(x)) if (q - x).length() <= 1e-9 * ctx.scale => Ok(x),
        (Some(q), _) => Ok(q),
        _ => Err(ProjectCurveError::Operations(
            OperationsError::InvalidInput {
                reason: "covered piece end has no image on its face".to_string(),
            },
        )),
    }
}

fn known_at(candidates: &[(f64, Option<Point3>)], s: f64) -> Option<Point3> {
    candidates
        .iter()
        .find(|(c, _)| (*c - s).abs() <= PARAM_DEDUPE)
        .and_then(|(_, point)| *point)
}

/// Support-surface distance (no region test).
fn support_distance(target: &TargetGeom, point: Point3) -> f64 {
    use remus_geometry::extrema::{
        point_to_cone, point_to_cylinder, point_to_plane, point_to_sphere,
    };
    match target {
        TargetGeom::Plane { normal, delta } => {
            point_to_plane(
                point,
                Point3::new(0.0, 0.0, 0.0) + *normal * *delta,
                *normal,
            )
            .distance
        }
        TargetGeom::Cylinder(cyl) => point_to_cylinder(point, cyl).distance,
        TargetGeom::Cone(cone) => point_to_cone(point, cone).distance,
        TargetGeom::Sphere(sph) => point_to_sphere(point, sph).distance,
    }
}

/// Post-check (§7): 33 samples per piece against support distance plus the
/// region test. Endpoints lie exactly on face boundaries (boundary
/// included), so membership uses the relaxed test with its weld band.
/// `pub(crate)` so the residual gate itself is unit-testable.
pub(crate) fn post_check_piece(
    topo: &Topology,
    face: FaceId,
    target: &TargetGeom,
    carrier: &Carrier,
    trim: (f64, f64),
    scale: f64,
    bound: f64,
) -> Result<(), ProjectCurveError> {
    let mut worst = 0.0f64;
    let mut outside = false;
    for k in 0..POST_SAMPLES {
        let t = trim.0 + (trim.1 - trim.0) * (k as f64 / (POST_SAMPLES - 1) as f64);
        let point = carrier.evaluate(t);
        worst = worst.max(support_distance(target, point));
        if !in_region_relaxed(topo, face, point, scale)? {
            outside = true;
        }
    }
    if outside || worst > bound {
        return Err(ProjectCurveError::ResidualExceeded {
            measured: worst.max(if outside { bound } else { 0.0 }),
            bound,
        });
    }
    Ok(())
}

/// Build committed pieces from classified runs (seam splits, short-piece
/// drops, closed-image merging, trims, post-check).
#[allow(clippy::too_many_lines)]
fn build_pieces(
    topo: &Topology,
    source: &ResolvedSource,
    direction: Vec3,
    faces: &[FaceCtx],
    candidates: &[(f64, Option<Point3>)],
    seams: &[Vec<(f64, Point3)>],
    runs: &[(usize, f64, f64)],
) -> Result<(Vec<PieceSpec>, bool), ProjectCurveError> {
    let (lo, hi) = source.domain;
    let domain_length = hi - lo;
    // Split runs at interior seam parameters where the image truly meets
    // the seam (a seam edge can cross the clip plane off-image, e.g. on a
    // hidden sheet: splitting there would cut a clean piece in two).
    let mut split_runs: Vec<(usize, f64, f64)> = Vec::new();
    for (ctx, a, b) in runs {
        let mut cuts = vec![*a];
        for (s, x) in &seams[*ctx] {
            if *s > *a + PARAM_DEDUPE
                && *s < *b - PARAM_DEDUPE
                && let Some(q) = image_at(topo, &faces[*ctx], source, direction, *s)?
                && (q - *x).length() <= 1e-9 * faces[*ctx].scale
            {
                cuts.push(*s);
            }
        }
        cuts.push(*b);
        for window in cuts.windows(2) {
            split_runs.push((*ctx, window[0], window[1]));
        }
    }
    // Closed source with full coverage: one closed edge. Exactly one
    // interior seam crossing puts the vertex on the seam (K5); otherwise
    // the vertex is the image of the source start.
    if source.closed {
        let covered: f64 = split_runs.iter().map(|(_, a, b)| b - a).sum();
        let single_ctx = split_runs.first().map(|(ctx, _, _)| *ctx);
        let one_loop =
            single_ctx.is_some_and(|first| split_runs.iter().all(|(ctx, _, _)| *ctx == first));
        if covered >= TAU - PARAM_ABS && one_loop {
            let interior_seams: Vec<(f64, Point3)> = seams
                .iter()
                .flat_map(|list| list.iter())
                .filter(|(s, _)| *s > lo + PARAM_DEDUPE && *s < hi - PARAM_DEDUPE)
                .map(|(s, p)| (*s, *p))
                .collect();
            if interior_seams.len() <= 1 {
                let (anchor_s, anchor_known) = interior_seams
                    .first()
                    .map(|(s, p)| (*s, Some(*p)))
                    .unwrap_or_else(|| (lo, known_at(candidates, lo)));
                // The single context covering the loop.
                let ctx_index = split_runs.first().map_or(0, |(ctx, _, _)| *ctx);
                let ctx = &faces[ctx_index];
                let carrier = ctx
                    .model
                    .carrier()
                    .ok_or_else(|| {
                        ProjectCurveError::Operations(OperationsError::InvalidInput {
                            reason: "closed image without a carrier".to_string(),
                        })
                    })?
                    .clone();
                let anchor_point =
                    endpoint_point(topo, ctx, source, direction, anchor_s, anchor_known)?;
                // Traversal sign from a small forward probe.
                let probe_s = anchor_s + 1e-3 * TAU;
                let probe_point =
                    image_at(topo, ctx, source, direction, probe_s)?.unwrap_or(anchor_point);
                let sign = if wrap_pi(carrier.project(probe_point) - carrier.project(anchor_point))
                    >= 0.0
                {
                    1.0
                } else {
                    -1.0
                };
                let u0 = carrier.project(anchor_point);
                let trim = (u0, u0 + sign * TAU);
                post_check_piece(
                    topo,
                    ctx.face,
                    &ctx.target,
                    &carrier,
                    trim,
                    ctx.scale,
                    1e-7 * ctx.scale,
                )?;
                return Ok((
                    vec![PieceSpec {
                        face: ctx.face,
                        carrier,
                        trim,
                        start: anchor_point,
                        end: anchor_point,
                        closed: true,
                        source_range: (anchor_s, anchor_s + TAU),
                    }],
                    false,
                ));
            }
        }
    }
    // Open pieces.
    let mut pieces = Vec::new();
    let mut covered = 0.0f64;
    for (ctx_index, a, b) in split_runs {
        let ctx = &faces[ctx_index];
        let mut carrier = ctx
            .model
            .carrier()
            .ok_or_else(|| {
                ProjectCurveError::Operations(OperationsError::InvalidInput {
                    reason: "classified run without a carrier".to_string(),
                })
            })?
            .clone();
        let start = endpoint_point(topo, ctx, source, direction, a, known_at(candidates, a))?;
        let end = endpoint_point(topo, ctx, source, direction, b, known_at(candidates, b))?;
        if (end - start).length() < TANGENT_BAND * ctx.scale {
            continue;
        }
        // A line carrier evaluates by interpolating its endpoints
        // (mirroring `EdgeCurve::Line`): re-anchor it to this piece so
        // sampling and trims trace the piece, not the model reference.
        if matches!(carrier, Carrier::Line { .. }) {
            carrier = Carrier::Line { p0: start, p1: end };
        }
        let mid = image_at(topo, ctx, source, direction, 0.5 * (a + b))?.ok_or_else(|| {
            ProjectCurveError::Operations(OperationsError::InvalidInput {
                reason: "classified run midpoint has no image".to_string(),
            })
        })?;
        let trim = resolve_trim_open(&carrier, start, end, mid);
        post_check_piece(
            topo,
            ctx.face,
            &ctx.target,
            &carrier,
            trim,
            ctx.scale,
            1e-7 * ctx.scale,
        )?;
        covered += b - a;
        pieces.push(PieceSpec {
            face: ctx.face,
            carrier,
            trim,
            start,
            end,
            closed: false,
            source_range: (a, b),
        });
    }
    Ok((pieces, covered < domain_length - PARAM_ABS))
}

// ---------------------------------------------------------------------------
// Commit: vertices, free edges, 2D output.
// ---------------------------------------------------------------------------

/// 2D image of a committed piece in a validated plane frame.
fn plane_curve_2d(
    frame: &Frame3,
    carrier: &Carrier,
    trim: (f64, f64),
    start: Point3,
    end: Point3,
    closed: bool,
) -> Result<PCurve, ProjectCurveError> {
    let to_2d = |point: Point3| {
        let w = point - frame.origin;
        Point2::new(w.dot(frame.x), w.dot(frame.y))
    };
    match carrier {
        Carrier::Line { .. } => {
            let (a, b) = (to_2d(start), to_2d(end));
            let delta = Vec2::new(b.x() - a.x(), b.y() - a.y());
            // `Line2D` normalizes its direction: the trim spans the length.
            let length = delta.length();
            let line =
                Line2D::new(a, delta).map_err(|error| math_to_ops("2D line image", error))?;
            Ok(PCurve::new(Curve2D::Line(line), 0.0, length))
        }
        Carrier::Circle(circle) => {
            let center = to_2d(circle.center());
            let angle_of = |point: Point3| {
                let p = to_2d(point) - center;
                p.y().atan2(p.x())
            };
            let (w0, w1) = if closed {
                let sign = (trim.1 - trim.0).signum();
                (angle_of(start), angle_of(start) + sign * TAU)
            } else {
                unwrap_with_mid(
                    angle_of(start),
                    angle_of(end),
                    angle_of(circle.evaluate(0.5 * (trim.0 + trim.1))),
                )
            };
            let result = Circle2D::new(center, circle.radius())
                .map_err(|error| math_to_ops("2D circle image", error))?;
            Ok(PCurve::new(Curve2D::Circle(result), w0, w1))
        }
        Carrier::Ellipse(ellipse) => {
            let center = to_2d(ellipse.center());
            let tip = to_2d(ellipse.center() + ellipse.u_axis());
            let rotation = (tip.y() - center.y()).atan2(tip.x() - center.x());
            let angle_of = |point: Point3| {
                let p = to_2d(point) - center;
                let (sin_r, cos_r) = rotation.sin_cos();
                let local_x = p.x().mul_add(cos_r, p.y() * sin_r) / ellipse.semi_major();
                let local_y = p.x().mul_add(-sin_r, p.y() * cos_r) / ellipse.semi_minor();
                local_y.atan2(local_x)
            };
            let (w0, w1) = if closed {
                let sign = (trim.1 - trim.0).signum();
                (angle_of(start), angle_of(start) + sign * TAU)
            } else {
                unwrap_with_mid(
                    angle_of(start),
                    angle_of(end),
                    angle_of(ellipse.evaluate(0.5 * (trim.0 + trim.1))),
                )
            };
            let result =
                Ellipse2D::new(center, ellipse.semi_major(), ellipse.semi_minor(), rotation)
                    .map_err(|error| math_to_ops("2D ellipse image", error))?;
            Ok(PCurve::new(Curve2D::Ellipse(result), w0, w1))
        }
        Carrier::Hyperbola(_) | Carrier::Parabola(_) => Err(ProjectCurveError::Operations(
            OperationsError::InvalidInput {
                reason: "2D output exists for line, circle and ellipse images only".to_string(),
            },
        )),
    }
}

/// Allocate free edges for committed pieces with vertex sharing between
/// consecutive coincident ends. Returns the projected edges in order.
fn commit_pieces(
    topo: &mut Topology,
    pieces: &[PieceSpec],
    plane_frame: Option<&Frame3>,
) -> Result<Vec<ProjectedEdge>, ProjectCurveError> {
    let mut committed = Vec::new();
    let mut last_vertex: Option<(remus_topology::vertex::VertexId, Point3, f64)> = None;
    for piece in pieces {
        let start_vertex = match last_vertex {
            Some((id, point, source_end))
                if (piece.source_range.0 - source_end).abs() <= PARAM_DEDUPE
                    && (piece.start - point).length() <= linear_tol() =>
            {
                id
            }
            _ => topo.add_vertex(Vertex::new(piece.start, linear_tol())),
        };
        let end_vertex = if piece.closed {
            start_vertex
        } else {
            topo.add_vertex(Vertex::new(piece.end, linear_tol()))
        };
        let mut edge = Edge::new(start_vertex, end_vertex, piece.carrier.to_edge_curve());
        if !matches!(piece.carrier, Carrier::Line { .. }) {
            edge.set_trim(Some(piece.trim));
        }
        let edge_id = topo.add_edge(edge);
        last_vertex = Some((end_vertex, piece.end, piece.source_range.1));
        let plane_curve = match plane_frame {
            Some(frame) => Some(plane_curve_2d(
                frame,
                &piece.carrier,
                piece.trim,
                piece.start,
                piece.end,
                piece.closed,
            )?),
            None => None,
        };
        committed.push(ProjectedEdge {
            edge: edge_id,
            face: piece.face,
            source_range: piece.source_range,
            plane_curve,
        });
    }
    Ok(committed)
}

// ---------------------------------------------------------------------------
// Options and plane-frame validation.
// ---------------------------------------------------------------------------

fn validate_options(options: &ProjectCurveOptions, scale: f64) -> Result<(), ProjectCurveError> {
    if !scale.is_finite() || scale <= 0.0 {
        return Err(invalid_options(
            "projection scale is not positive and finite",
        ));
    }
    if options.max_control_points < 4 {
        return Err(invalid_options("max_control_points must be at least 4"));
    }
    let tolerance = effective_tolerance(options, scale);
    if !tolerance.is_finite() || tolerance <= 0.0 || tolerance < 1e-9 * scale {
        return Err(invalid_options(
            "approximation tolerance must be finite, positive and at least 1e-9 · scale",
        ));
    }
    Ok(())
}

/// Effective absolute approximation tolerance (`1e-6 · scale` by default).
fn effective_tolerance(options: &ProjectCurveOptions, scale: f64) -> f64 {
    options.approximation_tolerance.unwrap_or(1e-6 * scale)
}

/// Validate `plane_frame` against a planar target (contract §3.3).
fn validate_plane_frame(
    options: &ProjectCurveOptions,
    target: &TargetGeom,
    scale: f64,
) -> Result<Option<Frame3>, ProjectCurveError> {
    let Some(frame) = &options.plane_frame else {
        return Ok(None);
    };
    let TargetGeom::Plane { normal, delta } = target else {
        return Err(ProjectCurveError::PlaneFrameMismatch);
    };
    let z = unit(frame.z, "plane frame axis")?;
    if z.cross(*normal).length() > GRAZE_PLANE {
        return Err(ProjectCurveError::PlaneFrameMismatch);
    }
    if (normal.dot(frame.origin - Point3::new(0.0, 0.0, 0.0)) - *delta).abs() > 1e-7 * scale {
        return Err(ProjectCurveError::PlaneFrameMismatch);
    }
    Ok(Some(*frame))
}

// ---------------------------------------------------------------------------
// Face-level and solid-level orchestration.
// ---------------------------------------------------------------------------

/// Errors that only skip one face of a solid call (that face contributes
/// no pieces); everything else refuses the source being processed.
fn is_face_skip(
    topo: &Topology,
    face: FaceId,
    error: &ProjectCurveError,
    source: &ResolvedSource,
    direction: Vec3,
    target: &TargetGeom,
) -> Result<bool, ProjectCurveError> {
    use remus_geometry::bounds::curve::{
        circle_arc_bounds, ellipse_arc_bounds, line_segment_bounds,
    };
    if !matches!(error, ProjectCurveError::GrazingDirection) {
        return Ok(false);
    }
    let TargetGeom::Plane { normal, delta } = target else {
        return Ok(false);
    };
    let dot = |vector: Vec3| {
        (0..3).fold(ScalarInterval::point(0.0), |sum, axis| {
            sum.add(
                ScalarInterval::point(vector.0[axis]).mul(ScalarInterval::point(normal.0[axis])),
            )
        })
    };
    let signed =
        |point: Point3| dot(point - Point3::new(0.0, 0.0, 0.0)).sub(ScalarInterval::point(*delta));
    let amplitude = |a: Vec3, b: Vec3| {
        let x = dot(a);
        let y = dot(b);
        x.mul(x).add(y.mul(y)).hi.max(0.0).sqrt().next_up()
    };
    let (distance, source_bound) = match &source.geom {
        SourceGeom::Segment { a, b } => {
            let a_signed = signed(*a);
            let b_signed = signed(*b);
            (
                ScalarInterval {
                    lo: a_signed.lo.min(b_signed.lo),
                    hi: a_signed.hi.max(b_signed.hi),
                },
                line_segment_bounds(*a, *b),
            )
        }
        SourceGeom::Circle { circle } => {
            let radius = amplitude(
                circle.u_axis() * circle.radius(),
                circle.v_axis() * circle.radius(),
            );
            let offset = ScalarInterval {
                lo: -radius,
                hi: radius,
            };
            (
                signed(circle.center()).add(offset),
                circle_arc_bounds(circle, source.domain.0, source.domain.1),
            )
        }
        SourceGeom::Ellipse { ellipse } => {
            let radius = amplitude(
                ellipse.u_axis() * ellipse.semi_major(),
                ellipse.v_axis() * ellipse.semi_minor(),
            );
            let offset = ScalarInterval {
                lo: -radius,
                hi: radius,
            };
            (
                signed(ellipse.center()).add(offset),
                ellipse_arc_bounds(ellipse, source.domain.0, source.domain.1),
            )
        }
        SourceGeom::Other { .. } => return Ok(false),
    };
    let face_bound = remus_check::distance::face_bounds::face_bound(topo, face)
        .map_err(OperationsError::Check)?;
    if !source_bound.is_prunable() || !face_bound.prunable {
        return Ok(false);
    }
    let lower_distance = if distance.lo > 0.0 {
        distance.lo
    } else if distance.hi < 0.0 {
        -distance.hi
    } else {
        0.0
    };
    let bounds = source_bound.aabb().union(face_bound.aabb);
    let diagonal_squared = (0..3).fold(ScalarInterval::point(0.0), |sum, axis| {
        let width = ScalarInterval::point(bounds.max.0[axis])
            .sub(ScalarInterval::point(bounds.min.0[axis]));
        sum.add(width.mul(width))
    });
    let travel = diagonal_squared.hi.max(0.0).sqrt().next_up();
    let denominator = dot(direction);
    let max_change = ScalarInterval::point(denominator.lo.abs().max(denominator.hi.abs()))
        .mul(ScalarInterval::point(travel))
        .hi;
    // Any point of the finite face is at most `travel` from any source
    // point. A ray too parallel to change the source's plane distance
    // over that travel cannot meet this face, even after rigid placement.
    Ok(travel.is_finite() && lower_distance > max_change)
}

struct ApproxSpec {
    curve: NurbsCurve,
    trim: (f64, f64),
    point: Point3,
    end: Point3,
    closed: bool,
    source_range: (f64, f64),
    max_deviation: f64,
}

/// Compute pieces for one source over face contexts (shared by face and
/// solid calls). Returns the pieces plus the clipped flag.
fn project_one_source(
    topo: &Topology,
    source: &ResolvedSource,
    direction: Vec3,
    faces: &[FaceCtx],
) -> Result<(Vec<PieceSpec>, bool), ProjectCurveError> {
    let (candidates, seams) = collect_candidates(source, direction, faces)?;
    let runs = classify_intervals(topo, source, direction, faces, &candidates)?;
    build_pieces(topo, source, direction, faces, &candidates, &seams, &runs)
}

/// Face-level projection: compute, then commit free edges.
fn project_face_inner(
    topo: &mut Topology,
    source_id: EdgeId,
    direction_raw: Vec3,
    face: FaceId,
    options: &ProjectCurveOptions,
) -> Result<ProjectedCurves, ProjectCurveError> {
    let direction = validate_direction(direction_raw)?;
    let source = resolve_source(topo, source_id)?;
    let target = resolve_target(topo, face)?;
    let extent = target_extent(topo, face, &target)?;
    let scale = source.extent.max(extent);
    validate_options(options, scale)?;
    let frame = validate_plane_frame(options, &target, scale)?;
    // Exact dispatch; the arc-on-curved approximate cell diverts.
    let model = match &source.geom {
        SourceGeom::Segment { a, b } => {
            let model = segment_model(*a, *b, direction, &target, scale)?;
            let ctx = FaceCtx::build(topo, face, target, model, scale)?;
            let faces = [ctx];
            let (pieces, clipped) = project_one_source(topo, &source, direction, &faces)?;
            return commit_face_result(
                topo,
                face,
                pieces,
                clipped,
                ProjectionQuality::Exact,
                frame.as_ref(),
            );
        }
        SourceGeom::Circle { circle, .. } => {
            match conic_model(Some(circle), None, direction, &target, scale)? {
                CurvedArcOutcome::Exact(model) => model,
                CurvedArcOutcome::NeedsApproximate => {
                    if !options.allow_approximate {
                        return Err(ProjectCurveError::ApproximationRequired);
                    }
                    let tolerance = effective_tolerance(options, scale);
                    let job = ApproxJob {
                        source: &source,
                        face,
                        target: &target,
                        direction,
                        tolerance,
                        max_control_points: options.max_control_points,
                        scale,
                    };
                    let spec = approximate_spec(topo, &job)?;
                    return Ok(commit_approx_result(topo, face, &spec));
                }
            }
        }
        SourceGeom::Ellipse { ellipse, .. } => {
            match conic_model(None, Some(ellipse), direction, &target, scale)? {
                CurvedArcOutcome::Exact(model) => model,
                CurvedArcOutcome::NeedsApproximate => {
                    return Err(ProjectCurveError::Operations(
                        OperationsError::InvalidInput {
                            reason: "ellipse sources never take the approximate path".to_string(),
                        },
                    ));
                }
            }
        }
        SourceGeom::Other { curve } => {
            return Err(ProjectCurveError::UnsupportedSourceCurve {
                curve: curve.type_tag(),
                surface: target_tag(&target),
            });
        }
    };
    let ctx = FaceCtx::build(topo, face, target, model, scale)?;
    let faces = [ctx];
    let (pieces, clipped) = project_one_source(topo, &source, direction, &faces)?;
    commit_face_result(
        topo,
        face,
        pieces,
        clipped,
        ProjectionQuality::Exact,
        frame.as_ref(),
    )
}

fn commit_face_result(
    topo: &mut Topology,
    face: FaceId,
    pieces: Vec<PieceSpec>,
    clipped: bool,
    quality: ProjectionQuality,
    frame: Option<&Frame3>,
) -> Result<ProjectedCurves, ProjectCurveError> {
    if pieces.is_empty() {
        return Err(ProjectCurveError::EmptyProjection);
    }
    let edges = commit_pieces(topo, &pieces, frame)?;
    Ok(ProjectedCurves {
        edges,
        face,
        quality,
        clipped,
    })
}

fn commit_approx_result(topo: &mut Topology, face: FaceId, spec: &ApproxSpec) -> ProjectedCurves {
    let vertex = topo.add_vertex(Vertex::new(spec.point, linear_tol()));
    let end = if spec.closed {
        vertex
    } else {
        topo.add_vertex(Vertex::new(spec.end, linear_tol()))
    };
    let mut edge = Edge::new(vertex, end, EdgeCurve::NurbsCurve(spec.curve.clone()));
    edge.set_trim(Some(spec.trim));
    let edge_id = topo.add_edge(edge);
    ProjectedCurves {
        edges: vec![ProjectedEdge {
            edge: edge_id,
            face,
            source_range: spec.source_range,
            plane_curve: None,
        }],
        face,
        quality: ProjectionQuality::Approximate {
            max_deviation: spec.max_deviation,
        },
        clipped: false,
    }
}

/// One source over a solid's faces: per-face models, skipping faces that
/// cannot contribute exactly (the clipped flag discloses the gap).
fn project_source_onto_faces(
    topo: &Topology,
    source: &ResolvedSource,
    source_tag: Option<&'static str>,
    direction: Vec3,
    targets: &[(FaceId, TargetGeom, f64)],
) -> Result<(Vec<PieceSpec>, bool), ProjectCurveError> {
    if let Some(tag) = source_tag {
        let surface = targets
            .first()
            .map(|(_, target, _)| target_tag(target))
            .unwrap_or("plane");
        return Err(ProjectCurveError::UnsupportedSourceCurve {
            curve: tag,
            surface,
        });
    }
    let mut faces = Vec::new();
    for (face, target, extent) in targets {
        let scale = source.extent.max(*extent);
        let model = match &source.geom {
            SourceGeom::Segment { a, b } => segment_model(*a, *b, direction, target, scale),
            SourceGeom::Circle { circle, .. } => {
                match conic_model(Some(circle), None, direction, target, scale) {
                    Ok(CurvedArcOutcome::Exact(model)) => Ok(model),
                    Ok(CurvedArcOutcome::NeedsApproximate) => {
                        Err(ProjectCurveError::ApproximationRequired)
                    }
                    Err(error) => Err(error),
                }
            }
            SourceGeom::Ellipse { ellipse, .. } => {
                match conic_model(None, Some(ellipse), direction, target, scale) {
                    Ok(CurvedArcOutcome::Exact(model)) => Ok(model),
                    Ok(CurvedArcOutcome::NeedsApproximate) => {
                        Err(ProjectCurveError::ApproximationRequired)
                    }
                    Err(error) => Err(error),
                }
            }
            SourceGeom::Other { .. } => {
                return Err(ProjectCurveError::Operations(
                    OperationsError::InvalidInput {
                        reason: "unsupported source curve".to_string(),
                    },
                ));
            }
        };
        match model {
            Ok(model) => faces.push(FaceCtx::build(topo, *face, target.clone(), model, scale)?),
            Err(error) => {
                if !is_face_skip(topo, *face, &error, source, direction, target)? {
                    return Err(error);
                }
            }
        }
    }
    if faces.iter().all(|ctx| ctx.model.carrier().is_none()) {
        return Ok((Vec::new(), true));
    }
    project_one_source(topo, source, direction, &faces)
}

/// Solid-level projection: atomic over sources, first-hit across faces.
fn project_solid_inner(
    topo: &mut Topology,
    source_ids: &[EdgeId],
    direction_raw: Vec3,
    solid: SolidId,
    options: &ProjectCurveOptions,
) -> Result<SolidProjection, ProjectCurveError> {
    let direction = validate_direction(direction_raw)?;
    let face_ids = solid_faces(topo, solid).map_err(OperationsError::from)?;
    if source_ids.is_empty() {
        return Err(ProjectCurveError::EmptyProjection);
    }
    // Resolve sources first: a stale handle refuses its index atomically.
    let mut sources = Vec::new();
    for (index, id) in source_ids.iter().enumerate() {
        match resolve_source(topo, *id) {
            Ok(source) => sources.push(source),
            Err(error) => {
                return Err(ProjectCurveError::SourceRefused {
                    index,
                    error: Box::new(error),
                });
            }
        }
    }
    // Resolve faces, skipping torus/NURBS carriers (opaque to this slice).
    let mut targets: Vec<(FaceId, TargetGeom, f64)> = Vec::new();
    for face in face_ids {
        match resolve_target(topo, face) {
            Ok(target) => {
                let extent = target_extent(topo, face, &target)?;
                targets.push((face, target, extent));
            }
            Err(ProjectCurveError::UnsupportedTargetSurface { surface }) => {
                return Err(ProjectCurveError::SourceRefused {
                    index: 0,
                    error: Box::new(ProjectCurveError::UnsupportedTargetSurface { surface }),
                });
            }
            Err(error) => return Err(error),
        }
    }
    let mut global_scale = 1e-300f64;
    for source in &sources {
        global_scale = global_scale.max(source.extent);
    }
    for (_, _, extent) in &targets {
        global_scale = global_scale.max(*extent);
    }
    validate_options(options, global_scale)?;
    // Compute every source before allocating anything.
    let mut computed: Vec<(Vec<PieceSpec>, bool)> = Vec::new();
    let mut any_piece = false;
    for (index, source) in sources.iter().enumerate() {
        let unsupported = match &source.geom {
            SourceGeom::Other { curve } => Some(curve.type_tag()),
            _ => None,
        };
        match project_source_onto_faces(topo, source, unsupported, direction, &targets) {
            Ok((pieces, clipped)) => {
                any_piece = any_piece || !pieces.is_empty();
                computed.push((pieces, clipped));
            }
            Err(error) => {
                return Err(ProjectCurveError::SourceRefused {
                    index,
                    error: Box::new(error),
                });
            }
        }
    }
    if !any_piece {
        return Err(ProjectCurveError::EmptyProjection);
    }
    let mut out = Vec::new();
    for (index, (pieces, clipped)) in computed.iter().enumerate() {
        let edges = commit_pieces(topo, pieces, None)?;
        out.push(SourceProjection {
            source: source_ids[index],
            edges,
            clipped: *clipped,
        });
    }
    Ok(SolidProjection {
        sources: out,
        quality: ProjectionQuality::Exact,
    })
}

/// Project `source` along `direction` onto the trimmed region of `face`.
///
/// Creates free edges only; records no journal entry; leaves the source
/// edge and the face unchanged. A refusal leaves the topology unchanged.
///
/// # Errors
///
/// Every refusal listed in the design note, as a typed
/// [`ProjectCurveError`].
pub fn project_curve_onto_face(
    topo: &mut Topology,
    source: EdgeId,
    direction: Vec3,
    face: FaceId,
    options: &ProjectCurveOptions,
) -> Result<ProjectedCurves, ProjectCurveError> {
    remus_topology::transaction::run_append_only(topo, |topo| {
        project_face_inner(topo, source, direction, face, options)
    })
    .map(|(value, _)| value)
}

/// Project every source along `direction` onto the first-hit faces of
/// `solid`. Atomic: any per-source refusal is returned as
/// [`ProjectCurveError::SourceRefused`] and nothing is created.
///
/// # Errors
///
/// See [`project_curve_onto_face`].
pub fn project_curves_onto_solid(
    topo: &mut Topology,
    sources: &[EdgeId],
    direction: Vec3,
    solid: SolidId,
    options: &ProjectCurveOptions,
) -> Result<SolidProjection, ProjectCurveError> {
    remus_topology::transaction::run_append_only(topo, |topo| {
        project_solid_inner(topo, sources, direction, solid, options)
    })
    .map(|(value, _)| value)
}

/// Read-only projection of every source along `direction` onto the
/// unbounded plane of `frame`, returned as 2D curves in frame coordinates
/// (one per source, input order).
///
/// # Errors
///
/// [`ProjectCurveError::InvalidDirection`] and
/// [`ProjectCurveError::GrazingDirection`] come back bare; per-source
/// refusals are wrapped as [`ProjectCurveError::SourceRefused`].
pub fn project_curves_onto_plane(
    topo: &Topology,
    sources: &[EdgeId],
    direction: Vec3,
    frame: &Frame3,
) -> Result<Vec<PCurve>, ProjectCurveError> {
    let direction = validate_direction(direction)?;
    let normal =
        unit(frame.z, "sketch plane normal").map_err(|_| ProjectCurveError::InvalidDirection)?;
    if direction.dot(normal).abs() <= GRAZE_PLANE {
        return Err(ProjectCurveError::GrazingDirection);
    }
    let mut out = Vec::new();
    for (index, id) in sources.iter().enumerate() {
        match sketch_one(topo, *id, direction, frame, normal) {
            Ok(curve) => out.push(curve),
            Err(error) => {
                return Err(ProjectCurveError::SourceRefused {
                    index,
                    error: Box::new(error),
                });
            }
        }
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// Sketch-plane projection (read-only, unbounded, all source types exact).
// ---------------------------------------------------------------------------

/// Affine map onto the sketch plane: `P(x) = x + λd̂` with the plane
/// `(frame.origin, normal)`.
fn sketch_map(frame: &Frame3, normal: Vec3, direction: Vec3) -> AffineMap {
    AffineMap::new(
        normal,
        normal.dot(frame.origin - Point3::new(0.0, 0.0, 0.0)),
        direction,
    )
}

fn sketch_to_2d(frame: &Frame3, point: Point3) -> Point2 {
    let w = point - frame.origin;
    Point2::new(w.dot(frame.x), w.dot(frame.y))
}

/// Unwrap a 2D image span with a known traversal sign (`+1` CCW in frame).
fn unwrap_span_oriented(w0: f64, w1raw: f64, sign: f64, full_turn: bool) -> (f64, f64) {
    if full_turn {
        return (w0, w0 + sign * TAU);
    }
    let delta = wrap_pi(w1raw - w0);
    let span = if sign >= 0.0 {
        if delta >= 0.0 { delta } else { delta + TAU }
    } else if delta <= 0.0 {
        delta
    } else {
        delta - TAU
    };
    (w0, w0 + span)
}

/// 2D image of a circle/ellipse source on a sketch plane.
fn sketch_conic_2d(
    frame: &Frame3,
    normal: Vec3,
    direction: Vec3,
    source: &ConicFrame,
    range: (f64, f64),
    closed: bool,
    project: impl Fn(f64) -> Point3,
) -> Result<PCurve, ProjectCurveError> {
    if source.normal.dot(direction).abs() <= DEGENERATE_CONIC {
        return Err(ProjectCurveError::DegenerateImage);
    }
    let map = sketch_map(frame, normal, direction);
    let linear = |w: Vec3| map.apply_vector(w);
    let a_vec = linear(source.u) * source.ru;
    let b_vec = linear(source.v) * source.rv;
    let orient = a_vec.cross(b_vec).dot(normal);
    if orient.abs() <= 1e-300 {
        return Err(ProjectCurveError::DegenerateImage);
    }
    let sign = orient.signum() * (range.1 - range.0).signum();
    let image_center = map.apply(source.center);
    let center_2d = sketch_to_2d(frame, image_center);
    let (major_3d, major_len, minor_len) =
        gram_ellipse(a_vec, b_vec).ok_or(ProjectCurveError::DegenerateImage)?;
    if (major_len - minor_len).abs() <= 1e-12 * major_len.max(minor_len) {
        let angle_of = |point: Point3| {
            let p = sketch_to_2d(frame, point) - center_2d;
            p.y().atan2(p.x())
        };
        let w0 = angle_of(project(range.0));
        let w1raw = angle_of(project(range.1));
        let radius = 0.5 * (major_len + minor_len);
        let circle = Circle2D::new(center_2d, radius)
            .map_err(|error| math_to_ops("sketch circle image", error))?;
        let (t0, t1) = unwrap_span_oriented(w0, w1raw, sign, closed);
        return Ok(PCurve::new(Curve2D::Circle(circle), t0, t1));
    }
    let tip = sketch_to_2d(frame, image_center + major_3d) - center_2d;
    let rotation = tip.y().atan2(tip.x());
    let ellipse = Ellipse2D::new(center_2d, major_len, minor_len, rotation)
        .map_err(|error| math_to_ops("sketch ellipse image", error))?;
    let angle_of = |point: Point3| {
        let p = sketch_to_2d(frame, point) - center_2d;
        let (sin, cos) = rotation.sin_cos();
        let x = p.x().mul_add(cos, p.y() * sin) / major_len;
        let y = p.y().mul_add(cos, -p.x() * sin) / minor_len;
        y.atan2(x)
    };
    let w0 = angle_of(project(range.0));
    let w1raw = angle_of(project(range.1));
    let (t0, t1) = unwrap_span_oriented(w0, w1raw, sign, closed);
    Ok(PCurve::new(Curve2D::Ellipse(ellipse), t0, t1))
}

/// Exact rational-quadratic 2D image of a hyperbola arc on a sketch plane.
/// Control points are the mapped ends and tangent intersection; the middle
/// weight puts the Bézier midpoint on the shoulder point (tangent parallel
/// to the chord), which fixes the conic uniquely.
fn sketch_hyperbola_2d(
    frame: &Frame3,
    normal: Vec3,
    direction: Vec3,
    hyperbola: &Hyperbola3D,
    range: (f64, f64),
) -> Result<PCurve, ProjectCurveError> {
    if hyperbola.normal().dot(direction).abs() <= DEGENERATE_CONIC {
        return Err(ProjectCurveError::DegenerateImage);
    }
    let map = sketch_map(frame, normal, direction);
    let map_point = |point: Point3| sketch_to_2d(frame, map.apply(point));
    let (t0, t1) = range;
    let q0 = hyperbola.evaluate(t0);
    let q2 = hyperbola.evaluate(t1);
    let q1 = hyperbola.tangent_intersection(t0, t1);
    // Shoulder: arc point whose tangent is parallel to the chord.
    let chord = q2 - q0;
    let axis = hyperbola.normal();
    let fun = |t: f64| hyperbola.tangent(t).cross(chord).dot(axis);
    let (mut flo, fhi) = (fun(t0), fun(t1));
    if flo == 0.0 || fhi == 0.0 || flo.signum() == fhi.signum() {
        return Err(ProjectCurveError::DegenerateImage);
    }
    let (mut lo, mut hi) = (t0, t1);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        let fmid = fun(mid);
        if fmid.signum() == flo.signum() {
            lo = mid;
            flo = fmid;
        } else {
            hi = mid;
        }
    }
    let shoulder = hyperbola.evaluate(0.5 * (lo + hi));
    let chord_mid = q0 + (q2 - q0) * 0.5;
    let numerator = (shoulder - chord_mid).length();
    let denominator = (q1 - shoulder).length();
    if denominator <= 0.0 || numerator <= 0.0 {
        return Err(ProjectCurveError::DegenerateImage);
    }
    // Collinearity of the diameter line is structural (pole-polar); verify
    // rather than assume: the shoulder must lie on the Q1–chord-mid line.
    let line_distance =
        (shoulder - q1).cross(q1 - chord_mid).length() / (q1 - chord_mid).length().max(1e-300);
    if line_distance > 1e-9 * numerator.max(denominator).max(1e-300) + 1e-12 {
        return Err(ProjectCurveError::DegenerateImage);
    }
    let weight = numerator / denominator;
    if !weight.is_finite() || weight <= 0.0 {
        return Err(ProjectCurveError::DegenerateImage);
    }
    let curve = NurbsCurve2D::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![map_point(q0), map_point(q1), map_point(q2)],
        vec![1.0, weight, 1.0],
    )
    .map_err(|error| math_to_ops("sketch hyperbola image", error))?;
    Ok(PCurve::new(Curve2D::Nurbs(curve), 0.0, 1.0))
}

fn sketch_one(
    topo: &Topology,
    source: EdgeId,
    direction: Vec3,
    frame: &Frame3,
    normal: Vec3,
) -> Result<PCurve, ProjectCurveError> {
    let edge = topo.edge(source).map_err(OperationsError::from)?;
    let start = topo
        .vertex(edge.start())
        .map_err(OperationsError::from)?
        .point();
    let end = topo
        .vertex(edge.end())
        .map_err(OperationsError::from)?
        .point();
    let map = sketch_map(frame, normal, direction);
    match edge.curve() {
        EdgeCurve::Line => {
            let a = sketch_to_2d(frame, map.apply(start));
            let b = sketch_to_2d(frame, map.apply(end));
            let delta = Vec2::new(b.x() - a.x(), b.y() - a.y());
            if delta.length() <= 1e-15 * (end - start).length().max(1e-300) {
                return Err(ProjectCurveError::DegenerateImage);
            }
            let line = Line2D::new(a, delta).map_err(|_| ProjectCurveError::DegenerateImage)?;
            Ok(PCurve::new(Curve2D::Line(line), 0.0, delta.length()))
        }
        EdgeCurve::Circle(circle) => {
            let range = edge
                .strict_domain()
                .map_err(|_| ProjectCurveError::DegenerateSource {
                    reason: "circle edge has no valid parameter range",
                })?;
            sketch_conic_2d(
                frame,
                normal,
                direction,
                &ConicFrame {
                    center: circle.center(),
                    normal: circle.normal(),
                    u: circle.u_axis(),
                    v: circle.v_axis(),
                    ru: circle.radius(),
                    rv: circle.radius(),
                },
                range,
                is_full_turn(range),
                |t| map.apply(circle.evaluate(t)),
            )
        }
        EdgeCurve::Ellipse(ellipse) => {
            let range = edge
                .strict_domain()
                .map_err(|_| ProjectCurveError::DegenerateSource {
                    reason: "ellipse edge has no valid parameter range",
                })?;
            sketch_conic_2d(
                frame,
                normal,
                direction,
                &ConicFrame {
                    center: ellipse.center(),
                    normal: ellipse.normal(),
                    u: ellipse.u_axis(),
                    v: ellipse.v_axis(),
                    ru: ellipse.semi_major(),
                    rv: ellipse.semi_minor(),
                },
                range,
                is_full_turn(range),
                |t| map.apply(ellipse.evaluate(t)),
            )
        }
        EdgeCurve::NurbsCurve(nurbs) => {
            let (d0, d1) =
                edge.strict_domain()
                    .map_err(|_| ProjectCurveError::DegenerateSource {
                        reason: "nurbs edge has no valid parameter range",
                    })?;
            // A sampling grid cannot exclude a stationary point between
            // samples. Prove that the rational derivative numerator stays
            // away from zero over every trimmed Bezier span instead.
            if !regular_nurbs_image(nurbs, (d0, d1), &map)? {
                return Err(ProjectCurveError::DegenerateImage);
            }
            let points_2d: Vec<Point2> = nurbs
                .control_points()
                .iter()
                .map(|point| sketch_to_2d(frame, map.apply(*point)))
                .collect();
            let image = NurbsCurve2D::new(
                nurbs.degree(),
                nurbs.knots().to_vec(),
                points_2d,
                nurbs.weights().to_vec(),
            )
            .map_err(|error| math_to_ops("sketch nurbs image", error))?;
            // Also certify the control points actually emitted after
            // affine mapping: large origins can round distinct mapped
            // points to one point despite a nonzero ideal derivative.
            let emitted = NurbsCurve::new(
                nurbs.degree(),
                nurbs.knots().to_vec(),
                image
                    .control_points()
                    .iter()
                    .map(|p| Point3::new(p.x(), p.y(), 0.0))
                    .collect(),
                nurbs.weights().to_vec(),
            )
            .map_err(|error| math_to_ops("sketch emitted image certificate", error))?;
            let identity = AffineMap {
                normal: Vec3::new(0.0, 0.0, 0.0),
                delta: 0.0,
                direction: Vec3::new(0.0, 0.0, 0.0),
                denom: 1.0,
            };
            if !regular_nurbs_image(&emitted, (d0, d1), &identity)? {
                return Err(ProjectCurveError::DegenerateImage);
            }
            Ok(PCurve::new(Curve2D::Nurbs(image), d0, d1))
        }
        EdgeCurve::Parabola(parabola) => {
            if parabola
                .axis_dir()
                .cross(parabola.u_axis())
                .dot(direction)
                .abs()
                <= DEGENERATE_CONIC
            {
                return Err(ProjectCurveError::DegenerateImage);
            }
            let (t0, t1) =
                edge.strict_domain()
                    .map_err(|_| ProjectCurveError::DegenerateSource {
                        reason: "parabola edge has no valid parameter range",
                    })?;
            let q0 = parabola.evaluate(t0);
            let q2 = parabola.evaluate(t1);
            let q1 = parabola.tangent_intersection(t0, t1);
            let map_point = |point: Point3| sketch_to_2d(frame, map.apply(point));
            let curve = NurbsCurve2D::new(
                2,
                vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                vec![map_point(q0), map_point(q1), map_point(q2)],
                vec![1.0, 1.0, 1.0],
            )
            .map_err(|error| math_to_ops("sketch parabola image", error))?;
            Ok(PCurve::new(Curve2D::Nurbs(curve), 0.0, 1.0))
        }
        EdgeCurve::Hyperbola(hyperbola) => {
            let range = edge
                .strict_domain()
                .map_err(|_| ProjectCurveError::DegenerateSource {
                    reason: "hyperbola edge has no valid parameter range",
                })?;
            sketch_hyperbola_2d(frame, normal, direction, hyperbola, range)
        }
    }
}

/// Outward-rounded arithmetic for the clipping/regularity certificates.
#[derive(Clone, Copy)]
struct ScalarInterval {
    lo: f64,
    hi: f64,
}

impl ScalarInterval {
    fn point(value: f64) -> Self {
        Self {
            lo: value,
            hi: value,
        }
    }
    fn whole() -> Self {
        Self {
            lo: f64::NEG_INFINITY,
            hi: f64::INFINITY,
        }
    }
    fn add(self, other: Self) -> Self {
        Self {
            lo: (self.lo + other.lo).next_down(),
            hi: (self.hi + other.hi).next_up(),
        }
    }
    fn sub(self, other: Self) -> Self {
        Self {
            lo: (self.lo - other.hi).next_down(),
            hi: (self.hi - other.lo).next_up(),
        }
    }
    fn mul(self, other: Self) -> Self {
        let values = [
            self.lo * other.lo,
            self.lo * other.hi,
            self.hi * other.lo,
            self.hi * other.hi,
        ];
        if values.iter().any(|x| x.is_nan()) {
            return Self::whole();
        }
        Self {
            lo: values
                .iter()
                .copied()
                .fold(f64::INFINITY, f64::min)
                .next_down(),
            hi: values
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, f64::max)
                .next_up(),
        }
    }
    fn div(self, other: Self) -> Self {
        if other.lo <= 0.0 && other.hi >= 0.0 {
            return Self::whole();
        }
        self.mul(Self {
            lo: (1.0 / other.hi).next_down(),
            hi: (1.0 / other.lo).next_up(),
        })
    }
    fn finite(self) -> bool {
        self.lo.is_finite() && self.hi.is_finite()
    }
}

type DerivativeCoefficient = [ScalarInterval; 3];

fn binomial(n: usize, k: usize) -> ScalarInterval {
    let mut value = ScalarInterval::point(1.0);
    for j in 1..=k.min(n - k) {
        value = value
            .mul(ScalarInterval::point((n - j + 1) as f64).div(ScalarInterval::point(j as f64)));
    }
    value
}

/// De Casteljau split, carrying outward coefficient enclosures.
fn split_bernstein(
    values: &[DerivativeCoefficient],
    t: f64,
) -> (Vec<DerivativeCoefficient>, Vec<DerivativeCoefficient>) {
    let mut row = values.to_vec();
    let mut left = vec![row[0]];
    let mut right = vec![row[row.len() - 1]];
    let t = ScalarInterval::point(t);
    let complement = ScalarInterval::point(1.0).sub(t);
    while row.len() > 1 {
        row = row
            .windows(2)
            .map(|pair| {
                std::array::from_fn(|axis| pair[0][axis].mul(complement).add(pair[1][axis].mul(t)))
            })
            .collect();
        left.push(row[0]);
        right.push(row[row.len() - 1]);
    }
    right.reverse();
    (left, right)
}

/// Every admitted window has one strictly signed coordinate hull. An
/// unresolved/nonfinite window refuses; subdivision/work limits cannot
/// silently approve a stationary point.
fn nonzero_bernstein(values: Vec<DerivativeCoefficient>) -> bool {
    let mut windows = vec![(values, 0usize)];
    let mut visited = 0usize;
    while let Some((coefficients, depth)) = windows.pop() {
        visited += 1;
        if visited > 65536 || coefficients.iter().flatten().any(|v| !v.finite()) {
            return false;
        }
        let excludes_zero = |coefficient: &DerivativeCoefficient| {
            coefficient.iter().any(|v| v.lo > 0.0 || v.hi < 0.0)
        };
        if (0..3).any(|axis| {
            coefficients.iter().all(|v| v[axis].lo > 0.0)
                || coefficients.iter().all(|v| v[axis].hi < 0.0)
        }) {
            continue;
        }
        if depth >= 40
            || !excludes_zero(&coefficients[0])
            || !excludes_zero(&coefficients[coefficients.len() - 1])
        {
            return false;
        }
        let (left, right) = split_bernstein(&coefficients, 0.5);
        windows.push((left, depth + 1));
        windows.push((right, depth + 1));
    }
    true
}

fn regular_nurbs_image(
    curve: &NurbsCurve,
    trim: (f64, f64),
    map: &AffineMap,
) -> Result<bool, ProjectCurveError> {
    let trim = (trim.0.min(trim.1), trim.0.max(trim.1));
    let p = curve.degree();
    let domain = curve.domain();
    if p > 64
        || curve.control_points().len() != p + 1
        || curve.knots()[..=p]
            .iter()
            .any(|knot| (*knot - domain.0).abs() > 0.0)
        || curve.knots()[p + 1..]
            .iter()
            .any(|knot| (*knot - domain.1).abs() > 0.0)
    {
        // Rounded knot insertion is not an enclosure of the original
        // curve; multi-span/unclamped data needs an interval decomposition.
        return Ok(false);
    }
    let segments = remus_math::nurbs::decompose::curve_to_bezier_segments(curve)
        .map_err(|error| math_to_ops("sketch image derivative certificate", error))?;
    for segment in segments {
        let (a, b) = segment.domain();
        if b < trim.0 || a > trim.1 {
            continue;
        }
        let p = segment.degree();
        let max_weight = segment.weights().iter().copied().fold(0.0, f64::max);
        let weights: Vec<ScalarInterval> = segment
            .weights()
            .iter()
            .map(|w| ScalarInterval::point(*w).div(ScalarInterval::point(max_weight)))
            .collect();
        let anchor = segment.control_points()[0];
        let homogeneous: Vec<DerivativeCoefficient> = segment
            .control_points()
            .iter()
            .zip(&weights)
            .map(|(point, weight)| {
                let vector: DerivativeCoefficient = std::array::from_fn(|axis| {
                    ScalarInterval::point(point.0[axis]).sub(ScalarInterval::point(anchor.0[axis]))
                });
                let dot = (0..3).fold(ScalarInterval::point(0.0), |sum, axis| {
                    sum.add(vector[axis].mul(ScalarInterval::point(map.normal.0[axis])))
                });
                let distance = dot.div(ScalarInterval::point(map.denom));
                std::array::from_fn(|axis| {
                    vector[axis]
                        .sub(distance.mul(ScalarInterval::point(map.direction.0[axis])))
                        .mul(*weight)
                })
            })
            .collect();
        let mut numerator = vec![[ScalarInterval::point(0.0); 3]; 2 * p];
        for i in 0..p {
            let degree = ScalarInterval::point(p as f64);
            let dw = weights[i + 1].sub(weights[i]).mul(degree);
            for j in 0..=p {
                let factor = binomial(p - 1, i)
                    .mul(binomial(p, j))
                    .div(binomial(2 * p - 1, i + j));
                for axis in 0..3 {
                    let dh = homogeneous[i + 1][axis]
                        .sub(homogeneous[i][axis])
                        .mul(degree);
                    let contribution = dh
                        .mul(weights[j])
                        .sub(homogeneous[j][axis].mul(dw))
                        .mul(factor);
                    numerator[i + j][axis] = numerator[i + j][axis].add(contribution);
                }
            }
        }
        let width = ScalarInterval::point(b).sub(ScalarInterval::point(a));
        let lo = ScalarInterval::point(trim.0.max(a))
            .sub(ScalarInterval::point(a))
            .div(width)
            .lo
            .clamp(0.0, 1.0);
        let hi = ScalarInterval::point(trim.1.min(b))
            .sub(ScalarInterval::point(a))
            .div(width)
            .hi
            .clamp(0.0, 1.0);
        if hi < 1.0 {
            numerator = split_bernstein(&numerator, hi).0;
        }
        if lo > 0.0 && hi > 0.0 {
            numerator = split_bernstein(
                &numerator,
                ScalarInterval::point(lo)
                    .div(ScalarInterval::point(hi))
                    .lo
                    .clamp(0.0, 1.0),
            )
            .1;
        }
        if !nonzero_bernstein(numerator) {
            return Ok(false);
        }
    }
    Ok(true)
}

// ---------------------------------------------------------------------------
// The approximate cell: arc onto a non-coaxial curved quadric (§6).
// ---------------------------------------------------------------------------

/// First root of a ray against the support surface (no region test).
/// Cones keep the real-nappe filter; planes always hit (non-grazing here).
fn first_support_root(
    target: &TargetGeom,
    point: Point3,
    direction: Vec3,
    scale: f64,
) -> Option<Point3> {
    let mut roots = ray_roots(target, point, direction);
    roots.sort_by(f64::total_cmp);
    for lambda in roots {
        if lambda < -PARAM_DEDUPE * scale {
            continue;
        }
        return Some(point + direction * lambda.max(0.0));
    }
    None
}

/// Deviation metric (§6): support distance and source-plane gap.
fn approximate_deviation(
    target: &TargetGeom,
    source_center: Point3,
    source_normal: Vec3,
    source_radius: f64,
    direction: Vec3,
    point: Point3,
) -> f64 {
    let support = support_distance(target, point);
    let denom = direction.dot(source_normal);
    let gap = if denom.abs() <= 1e-15 {
        f64::INFINITY
    } else {
        let lambda = (point - source_center).dot(source_normal) / denom;
        let back = point - direction * lambda;
        ((back - source_center).length() - source_radius).abs()
    };
    support.max(gap)
}

/// Inputs to the approximate-cell fitter (bundled for arity).
struct ApproxJob<'a> {
    source: &'a ResolvedSource,
    face: FaceId,
    target: &'a TargetGeom,
    direction: Vec3,
    tolerance: f64,
    max_control_points: usize,
    scale: f64,
}

/// Fit the approximate image: exact first-root samples, clamped cubic
/// interpolation, refinement until the disclosed deviation fits the budget.
#[allow(clippy::too_many_lines)]
fn approximate_spec(topo: &Topology, job: &ApproxJob<'_>) -> Result<ApproxSpec, ProjectCurveError> {
    let ApproxJob {
        source,
        face,
        target,
        direction,
        tolerance,
        max_control_points,
        scale,
    } = *job;
    let SourceGeom::Circle { circle: arc } = &source.geom else {
        return Err(ProjectCurveError::Operations(
            OperationsError::InvalidInput {
                reason: "the approximate cell takes circle sources only".to_string(),
            },
        ));
    };
    let (lo, hi) = source.domain;
    let closed = source.closed;
    // Clip probe: every exact sample must lie in the face region.
    let mut seen_inside = false;
    let mut seen_outside = false;
    for k in 0..APPROX_CLIP_SAMPLES {
        let s = lo + (hi - lo) * (k as f64 / (APPROX_CLIP_SAMPLES - 1) as f64);
        let probe = source_point(source, s);
        match first_support_root(target, probe, direction, scale) {
            Some(hit) if in_region(topo, face, hit)? => seen_inside = true,
            _ => seen_outside = true,
        }
    }
    if !seen_inside {
        return Err(ProjectCurveError::EmptyProjection);
    }
    if seen_outside {
        return Err(ProjectCurveError::ApproximateClipUnsupported);
    }
    let center = arc.center();
    let normal = arc.normal();
    let radius = arc.radius();
    let deviation =
        |point: Point3| approximate_deviation(target, center, normal, radius, direction, point);
    // Disclosure grids: dense plus a half-step-shifted twin.
    let disclose = |curve: &NurbsCurve| {
        let (d0, d1) = curve.domain();
        let mut worst = 0.0f64;
        for pass in 0..2 {
            for k in 0..APPROX_DISCLOSE_SAMPLES {
                let shift = pass as f64 * 0.5 / APPROX_DISCLOSE_SAMPLES as f64;
                let u =
                    d0 + (d1 - d0) * ((k as f64 / APPROX_DISCLOSE_SAMPLES as f64 + shift) % 1.0);
                worst = worst.max(deviation(curve.evaluate(u)));
            }
        }
        worst
    };
    // Fit-point budgets within the caller's control-point budget
    // (interpolation uses one control point per sample; a closed loop
    // repeats its start point, costing one more).
    let mut attempt_counts: Vec<usize> = APPROX_FIT_COUNTS
        .iter()
        .copied()
        .filter(|count| count + usize::from(closed) <= max_control_points)
        .collect();
    if attempt_counts.is_empty() {
        attempt_counts.push(
            max_control_points
                .saturating_sub(usize::from(closed))
                .max(2),
        );
    }
    let mut best_achieved = f64::INFINITY;
    for count in attempt_counts {
        let samples = count + usize::from(closed);
        let mut points = Vec::with_capacity(samples);
        let mut missing = false;
        for k in 0..samples {
            let s = if closed && k == samples - 1 {
                lo
            } else {
                let intervals = samples - 1;
                lo + (hi - lo) * (k as f64 / intervals as f64)
            };
            if let Some(hit) = first_support_root(target, source_point(source, s), direction, scale)
            {
                points.push(hit);
            } else {
                missing = true;
                break;
            }
        }
        if missing {
            // The clip probe passed on its grid but this fit grid lands
            // off the image: the image is not cleanly inside the face.
            return Err(ProjectCurveError::ApproximateClipUnsupported);
        }
        if closed {
            // Exact closure: reuse the start value, not a recomputation.
            let first = points[0];
            points[samples - 1] = first;
        }
        let curve = remus_math::nurbs::fitting::interpolate(&points, 3)
            .map_err(|error| math_to_ops("approximate fit", error))?;
        let achieved = disclose(&curve);
        best_achieved = best_achieved.min(achieved);
        if achieved <= tolerance {
            let domain = curve.domain();
            // Apply the contract's final on-face check before allocation,
            // including both fitted endpoints.
            for k in 0..POST_SAMPLES {
                let t = domain.0 + (domain.1 - domain.0) * (k as f64 / (POST_SAMPLES - 1) as f64);
                let point = curve.evaluate(t);
                let measured = support_distance(target, point);
                if !measured.is_finite()
                    || measured > achieved
                    || !in_region_relaxed(topo, face, point, scale)?
                {
                    return Err(ProjectCurveError::ResidualExceeded {
                        measured: if measured > achieved {
                            measured
                        } else {
                            f64::INFINITY
                        },
                        bound: achieved,
                    });
                }
            }
            return Ok(ApproxSpec {
                curve,
                trim: domain,
                point: points[0],
                end: points[samples - 1],
                closed,
                source_range: (lo, hi),
                max_deviation: achieved,
            });
        }
    }
    Err(ProjectCurveError::ToleranceUnattainable {
        requested: tolerance,
        achieved: best_achieved,
    })
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use remus_math::vec::{Point3, Vec3};

    use super::*;

    fn box_top_face() -> (Topology, FaceId, TargetGeom, f64) {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_box(&mut topo, 10.0, 8.0, 4.0).expect("box");
        let faces = solid_faces(&topo, solid).expect("faces");
        let face = faces
            .iter()
            .find(|face| {
                matches!(topo.face(**face).expect("face").surface(), FaceSurface::Plane { normal, .. } if normal.z() > 0.0)
            })
            .expect("top face");
        let target = resolve_target(&topo, *face).expect("target");
        (topo, *face, target, 10.0)
    }

    #[test]
    fn post_check_accepts_the_exact_carrier() {
        let (topo, face, target, scale) = box_top_face();
        let carrier = Carrier::Line {
            p0: Point3::new(1.9, 1.4, 4.0),
            p1: Point3::new(9.5, 4.0, 4.0),
        };
        post_check_piece(
            &topo,
            face,
            &target,
            &carrier,
            (0.0, 1.0),
            scale,
            1e-7 * scale,
        )
        .expect("exact carrier passes");
    }

    #[test]
    fn post_check_rejects_a_perturbed_carrier() {
        let (topo, face, target, scale) = box_top_face();
        // Shifted 1e-5 off the plane: support residual 1e-5 exceeds the
        // 1e-7 · scale bound.
        let carrier = Carrier::Line {
            p0: Point3::new(1.9, 1.4, 4.0 + 1e-5),
            p1: Point3::new(9.5, 4.0, 4.0 + 1e-5),
        };
        match post_check_piece(
            &topo,
            face,
            &target,
            &carrier,
            (0.0, 1.0),
            scale,
            1e-7 * scale,
        ) {
            Err(ProjectCurveError::ResidualExceeded { measured, bound }) => {
                assert!((measured - 1e-5).abs() <= 1e-9, "measured {measured:e}");
                assert!((bound - 1e-6).abs() <= 1e-12, "bound {bound:e}");
            }
            other => panic!("expected ResidualExceeded, got {other:?}"),
        }
    }

    #[test]
    fn gram_rule_recovers_circle_and_ellipse() {
        let (major, a, b) =
            gram_ellipse(Vec3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 2.0, 0.0)).expect("gram");
        assert!((a - 2.0).abs() <= 1e-12 && (b - 2.0).abs() <= 1e-12);
        assert!(
            (major - Vec3::new(1.0, 0.0, 0.0)).length() <= 1e-12
                || (major - Vec3::new(0.0, 1.0, 0.0)).length() <= 1e-12
        );
        let (major, a, b) = gram_ellipse(
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(-0.519_615_242_270_663_2, 1.346_410_161_513_775_5, 0.0),
        )
        .expect("gram");
        assert!((a - 2.110_742_241_242_658_4).abs() <= 1e-9, "a = {a}");
        assert!((b - 1.275_769_381_221_179_4).abs() <= 1e-9, "b = {b}");
        // The major direction is sign-ambiguous; it must be a unit
        // in-plane eigenvector (here: in the z = 0 plane).
        assert!((major.length() - 1.0).abs() <= 1e-12, "major = {major:?}");
        assert!(major.z().abs() <= 1e-12, "major = {major:?}");
    }
}
