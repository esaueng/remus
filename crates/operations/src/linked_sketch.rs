//! Persistent planar sketch attachments and exact external line/conic inputs.
//!
//! This bounded 6.6 slice consumes construction history, never geometry guesses.
//! Resolving inputs does not allocate topology or publish journal records.

use remus_math::frame::Frame3;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;
use remus_topology::journal::{EntityKey, EntityKind};
use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};

/// Refusals leave both topology and the caller's last resolved sketch intact.
#[derive(Debug, thiserror::Error)]
pub enum LinkedSketchError {
    /// The attachment frame is not a finite orthonormal right-handed frame.
    #[error("invalid linked sketch frame")]
    InvalidFrame,
    /// An anchor addresses the wrong entity kind.
    #[error("linked sketch reference must address a {0}")]
    InvalidReferenceKind(&'static str),
    /// Construction history cannot bind exactly one entity.
    #[error("linked sketch {role} reference is unbound: {resolution:?}")]
    Unbound {
        /// Attachment or external edge.
        role: &'static str,
        /// The existing persistent naming outcome, including splits/deletions.
        resolution: Resolution,
    },
    /// Geometry-based inference is not accepted as construction identity.
    #[error("linked sketch requires construction-derived references")]
    InferredReference,
    /// Only parallel planar attachments are qualified in this slice.
    #[error("linked sketch attachment requires a parallel planar face")]
    UnsupportedAttachment,
    /// The source is outside the requested exact external-curve family.
    #[error("linked sketch external input requires a qualified line or complete circle/ellipse")]
    UnsupportedCurve,
    /// Vertex lineage does not describe the resolved edge endpoints.
    #[error("linked sketch endpoint references do not match the resolved edge")]
    EndpointMismatch,
    /// Linked fixed input coordinates can only change through reference refresh.
    #[error("linked sketch external points are read-only")]
    ReadOnlyExternalPoint,
    /// Existing topology validation error.
    #[error(transparent)]
    Topology(#[from] remus_topology::TopologyError),
}

impl LinkedSketchError {
    /// Stable operation-specific refusal code.
    #[must_use]
    pub fn code(&self) -> &'static str {
        match self {
            Self::InvalidFrame => "invalid_frame",
            Self::InvalidReferenceKind(_) => "invalid_reference_kind",
            Self::Unbound { .. } => "reference_unbound",
            Self::InferredReference => "inferred_reference",
            Self::UnsupportedAttachment => "unsupported_attachment",
            Self::UnsupportedCurve => "unsupported_curve",
            Self::Topology(_) => "invalid_topology",
            Self::EndpointMismatch => "endpoint_mismatch",
            Self::ReadOnlyExternalPoint => "read_only_external_point",
        }
    }
}

fn one_entity(
    topo: &Topology,
    reference: &PersistentRef,
    kind: EntityKind,
    role: &'static str,
) -> Result<EntityKey, LinkedSketchError> {
    if reference.entity_kind != kind {
        return Err(LinkedSketchError::InvalidReferenceKind(kind.as_str()));
    }
    match resolve(topo, reference) {
        Resolution::Bound {
            entity,
            provenance: Provenance::Construction,
        } if entity.kind == kind => Ok(entity),
        Resolution::Bound {
            provenance: Provenance::Inferred,
            ..
        } => Err(LinkedSketchError::InferredReference),
        resolution => Err(LinkedSketchError::Unbound { role, resolution }),
    }
}

fn frame_valid(frame: &Frame3) -> bool {
    let finite = |v: Vec3| v.x().is_finite() && v.y().is_finite() && v.z().is_finite();
    finite(frame.origin - Point3::new(0.0, 0.0, 0.0))
        && [frame.x, frame.y, frame.z]
            .into_iter()
            .all(|v| finite(v) && (v.dot(v) - 1.0).abs() <= 1e-9)
        && (frame.x.cross(frame.y) - frame.z).length() <= 1e-9
        && frame.x.dot(frame.y).abs() <= 1e-9
}

/// A persistent attachment with an explicit, stable in-plane coordinate frame.
#[derive(Debug, Clone)]
pub struct PlanarSketchAttachment {
    face: PersistentRef,
    frame: Frame3,
}

impl PlanarSketchAttachment {
    /// Attach a frame lying on one construction-resolved planar face.
    ///
    /// # Errors
    /// Refuses malformed frames, non-planar faces, split/unbound/inferred identity
    /// and frames whose origin or normal does not agree with the face support.
    pub fn new(
        topo: &Topology,
        face: PersistentRef,
        frame: Frame3,
    ) -> Result<Self, LinkedSketchError> {
        if !frame_valid(&frame) {
            return Err(LinkedSketchError::InvalidFrame);
        }
        let attachment = Self { face, frame };
        let resolved = attachment.resolve_frame(topo)?;
        if (resolved.origin - frame.origin).length() > Tolerance::default().linear {
            return Err(LinkedSketchError::InvalidFrame);
        }
        Ok(attachment)
    }

    /// Persistent face identity, suitable for the existing reference codec.
    #[must_use]
    pub fn reference(&self) -> &PersistentRef {
        &self.face
    }

    /// Resolve a parallel plane edit without changing the in-plane axes.
    ///
    /// # Errors
    /// Refuses missing construction history, splits, deletion, rotations and
    /// curved replacement supports; it never chooses a nearest face.
    pub fn resolve_frame(&self, topo: &Topology) -> Result<Frame3, LinkedSketchError> {
        let key = one_entity(topo, &self.face, EntityKind::Face, "attachment")?;
        let id = topo
            .face_id_from_index(key.index)
            .ok_or(LinkedSketchError::UnsupportedAttachment)?;
        let FaceSurface::Plane { normal, d } = topo.face(id)?.surface() else {
            return Err(LinkedSketchError::UnsupportedAttachment);
        };
        let length = normal.length();
        if !length.is_finite() || length == 0.0 || !d.is_finite() {
            return Err(LinkedSketchError::UnsupportedAttachment);
        }
        let normal = *normal * (1.0 / length);
        if normal.cross(self.frame.z).length() > 1e-9 {
            return Err(LinkedSketchError::UnsupportedAttachment);
        }
        let origin = self.frame.origin
            + normal * (*d / length - normal.dot(self.frame.origin - Point3::new(0.0, 0.0, 0.0)));
        // Normalize accepted angular roundoff onto the actual support. Keeping
        // a slightly tilted requested normal would lift long external edges
        // farther off the plane than the unchanged linear tolerance.
        let z = normal
            * if normal.dot(self.frame.z) >= 0.0 {
                1.0
            } else {
                -1.0
            };
        let x = (self.frame.x - z * self.frame.x.dot(z))
            .normalize()
            .map_err(|_| LinkedSketchError::InvalidFrame)?;
        let frame = Frame3 {
            origin,
            x,
            y: z.cross(x),
            z,
        };
        if !frame_valid(&frame) {
            return Err(LinkedSketchError::InvalidFrame);
        }
        Ok(frame)
    }

    /// Project a construction-resolved line orthogonally into the current plane.
    ///
    /// # Errors
    /// Refuses non-line or collapsed images and split/unbound/inferred sources.
    /// Projection is unbounded: source edges need not lie inside the face trim.
    pub fn resolve_line(
        &self,
        topo: &Topology,
        source: &ExternalLineReference,
    ) -> Result<ExternalSketchLine, LinkedSketchError> {
        let frame = self.resolve_frame(topo)?;
        let key = one_entity(topo, &source.edge, EntityKind::Edge, "edge")?;
        let id = topo
            .edge_id_from_index(key.index)
            .ok_or(LinkedSketchError::UnsupportedCurve)?;
        let edge = topo.edge(id)?;
        if !matches!(edge.curve(), EdgeCurve::Line) {
            return Err(LinkedSketchError::UnsupportedCurve);
        }
        let project = |p: Point3| {
            let w = p - frame.origin;
            Point2::new(w.dot(frame.x), w.dot(frame.y))
        };
        let start_key = one_entity(topo, &source.start, EntityKind::Vertex, "start vertex")?;
        let end_key = one_entity(topo, &source.end, EntityKind::Vertex, "end vertex")?;
        let (start, end) = if edge.start().index() == start_key.index
            && edge.end().index() == end_key.index
        {
            (edge.start(), edge.end())
        } else if edge.end().index() == start_key.index && edge.start().index() == end_key.index {
            (edge.end(), edge.start())
        } else {
            return Err(LinkedSketchError::EndpointMismatch);
        };
        let start = project(topo.vertex(start)?.point());
        let end = project(topo.vertex(end)?.point());
        let image_length = (end - start).length();
        if ![start.x(), start.y(), end.x(), end.y()]
            .into_iter()
            .all(f64::is_finite)
            || !image_length.is_finite()
            || image_length <= Tolerance::default().linear
        {
            return Err(LinkedSketchError::UnsupportedCurve);
        }
        Ok(ExternalSketchLine { start, end })
    }

    /// Project one complete native circle into the current parallel plane.
    ///
    /// A circle has no distinguished angular endpoint: changing seam phase or
    /// stored traversal does not change the returned center/radius identity.
    ///
    /// # Errors
    /// Refuses missing/nonunique/inferred identity, non-native circles,
    /// incomplete stored domains, unsound carriers/seams, and oblique carriers
    /// whose orthogonal images would be ellipses. Refusal never mutates topology.
    pub fn resolve_circle(
        &self,
        topo: &Topology,
        source: &ExternalCircleReference,
    ) -> Result<ExternalSketchCircle, LinkedSketchError> {
        let frame = self.resolve_frame(topo)?;
        let key = one_entity(topo, &source.edge, EntityKind::Edge, "edge")?;
        let id = topo
            .edge_id_from_index(key.index)
            .ok_or(LinkedSketchError::UnsupportedCurve)?;
        let edge = topo.edge(id)?;
        let EdgeCurve::Circle(circle) = edge.curve() else {
            return Err(LinkedSketchError::UnsupportedCurve);
        };
        let tolerance = Tolerance::default();
        let finite = |v: Vec3| v.0.into_iter().all(f64::is_finite);
        let axes = [circle.u_axis(), circle.v_axis(), circle.normal()];
        if !edge.is_closed()
            || !circle.radius().is_finite()
            || circle.radius() <= 0.0
            || !finite(circle.center() - Point3::new(0.0, 0.0, 0.0))
            || axes
                .iter()
                .any(|axis| !finite(*axis) || (axis.dot(*axis) - 1.0).abs() > tolerance.angular)
        {
            return Err(LinkedSketchError::UnsupportedCurve);
        }
        // `with_axes` and arena documents can bypass constructor frame checks.
        // Bound accepted carrier roundoff in model units as well as angle;
        // multiplying a tiny frame defect by a large radius must not invent a
        // different circular image outside the unchanged linear tolerance.
        let defect = (circle.u_axis().cross(circle.v_axis()) - circle.normal())
            .length()
            .max(circle.u_axis().dot(circle.v_axis()).abs())
            .max(
                axes.into_iter()
                    .map(|a| (a.dot(a) - 1.0).abs())
                    .fold(0.0, f64::max),
            );
        if !defect.is_finite()
            || defect > tolerance.angular
            || circle.radius() * (defect + 16.0 * f64::EPSILON) > tolerance.linear
        {
            return Err(LinkedSketchError::UnsupportedCurve);
        }
        let normal = circle.normal() * (1.0 / circle.normal().length());
        // Equality of canonical directions is deliberately conservative.
        // An angular-nearness test would silently replace an oblique ellipse
        // projection by a circle. Antiparallel circle sense is immaterial.
        if normal != frame.z && normal != -frame.z {
            return Err(LinkedSketchError::UnsupportedCurve);
        }
        let (start, end) = edge
            .strict_domain()
            .map_err(|_| LinkedSketchError::UnsupportedCurve)?;
        // strict_domain's closed-periodic invariant proves one signed full
        // turn, including anchored subtraction roundoff. Check both ends:
        // a closed edge shares one vertex, but each endpoint is an authority.
        let vertex = topo.vertex(edge.start())?;
        let vertex_tolerance = vertex.tolerance();
        if !vertex_tolerance.is_finite()
            || vertex_tolerance.is_sign_negative()
            || !finite(vertex.point() - Point3::new(0.0, 0.0, 0.0))
            || [start, end].into_iter().any(|parameter| {
                let point = circle.evaluate(parameter);
                !finite(point - Point3::new(0.0, 0.0, 0.0))
                    || (point - vertex.point()).length() > vertex_tolerance
            })
        {
            return Err(LinkedSketchError::UnsupportedCurve);
        }
        let offset = circle.center() - frame.origin;
        let center = Point2::new(offset.dot(frame.x), offset.dot(frame.y));
        if !center.0.into_iter().all(f64::is_finite) {
            return Err(LinkedSketchError::UnsupportedCurve);
        }
        Ok(ExternalSketchCircle {
            center,
            radius: circle.radius(),
        })
    }

    /// Project one complete native ellipse into a parallel attached plane.
    ///
    /// The source's explicit major axis defines the in-plane angle; no axis
    /// matching is inferred from coordinates. Reversing either traversal or
    /// the carrier's normal does not change the represented full ellipse.
    ///
    /// # Errors
    /// Refuses incomplete/oblique/unsound carriers and unresolved construction
    /// identity. Circular, near-circular and noncanonical swapped axes lie
    /// outside this bounded principal-axis family and return `UnsupportedCurve`.
    pub fn resolve_ellipse(
        &self,
        topo: &Topology,
        source: &ExternalEllipseReference,
    ) -> Result<ExternalSketchEllipse, LinkedSketchError> {
        let frame = self.resolve_frame(topo)?;
        let key = one_entity(topo, &source.edge, EntityKind::Edge, "edge")?;
        let id = topo
            .edge_id_from_index(key.index)
            .ok_or(LinkedSketchError::UnsupportedCurve)?;
        let edge = topo.edge(id)?;
        let EdgeCurve::Ellipse(ellipse) = edge.curve() else {
            return Err(LinkedSketchError::UnsupportedCurve);
        };
        let tolerance = Tolerance::default();
        let a = ellipse.semi_major();
        let b = ellipse.semi_minor();
        let finite = |v: Vec3| v.0.into_iter().all(f64::is_finite);
        let axes = [ellipse.u_axis(), ellipse.v_axis(), ellipse.normal()];
        if !edge.is_closed()
            || !a.is_finite()
            || !b.is_finite()
            || b <= 0.0
            || a - b <= tolerance.linear
            || !finite(ellipse.center() - Point3::new(0.0, 0.0, 0.0))
            || axes.iter().any(|axis| !finite(*axis))
        {
            return Err(LinkedSketchError::UnsupportedCurve);
        }
        let defect = (ellipse.u_axis().cross(ellipse.v_axis()) - ellipse.normal())
            .length()
            .max(ellipse.u_axis().dot(ellipse.v_axis()).abs())
            .max(
                axes.into_iter()
                    .map(|axis| (axis.dot(axis) - 1.0).abs())
                    .fold(0.0, f64::max),
            );
        if !defect.is_finite()
            || defect > tolerance.angular
            || a * (defect + 16.0 * f64::EPSILON) > tolerance.linear
        {
            return Err(LinkedSketchError::UnsupportedCurve);
        }
        let normal = ellipse.normal() * (1.0 / ellipse.normal().length());
        if normal != frame.z && normal != -frame.z {
            return Err(LinkedSketchError::UnsupportedCurve);
        }
        let (start, end) = edge
            .strict_domain()
            .map_err(|_| LinkedSketchError::UnsupportedCurve)?;
        let vertex = topo.vertex(edge.start())?;
        let vertex_tolerance = vertex.tolerance();
        if !vertex_tolerance.is_finite()
            || vertex_tolerance.is_sign_negative()
            || !finite(vertex.point() - Point3::new(0.0, 0.0, 0.0))
            || [start, end].into_iter().any(|parameter| {
                let point = ellipse.evaluate(parameter);
                !finite(point - Point3::new(0.0, 0.0, 0.0))
                    || (point - vertex.point()).length() > vertex_tolerance
            })
        {
            return Err(LinkedSketchError::UnsupportedCurve);
        }
        let offset = ellipse.center() - frame.origin;
        let center = Point2::new(offset.dot(frame.x), offset.dot(frame.y));
        let angle = ellipse
            .u_axis()
            .dot(frame.y)
            .atan2(ellipse.u_axis().dot(frame.x));
        if !center.0.into_iter().all(f64::is_finite) || !angle.is_finite() {
            return Err(LinkedSketchError::UnsupportedCurve);
        }
        Ok(ExternalSketchEllipse {
            center,
            a,
            b,
            angle,
        })
    }
}

/// Construction identity of a complete native ellipse, independent of seam.
#[derive(Debug, Clone)]
pub struct ExternalEllipseReference {
    /// Complete native edge identity.
    pub edge: PersistentRef,
}

/// Exact ellipse coordinates in the attached sketch's explicit plane frame.
#[derive(Debug, Clone, Copy)]
pub struct ExternalSketchEllipse {
    /// Orthogonally projected center.
    pub center: Point2,
    /// Source major semiaxis.
    pub a: f64,
    /// Source minor semiaxis.
    pub b: f64,
    /// Source major-axis direction in radians, modulo the ellipse's pi symmetry.
    pub angle: f64,
}

/// Construction identity for a complete circular edge; seam identity is not
/// part of a full circle's center/radius definition.
#[derive(Debug, Clone)]
pub struct ExternalCircleReference {
    /// Complete native circle edge identity.
    pub edge: PersistentRef,
}

/// Exact circle coordinates in the attached sketch's explicit plane frame.
#[derive(Debug, Clone, Copy)]
pub struct ExternalSketchCircle {
    /// Orthogonally projected center.
    pub center: Point2,
    /// Unchanged radius of the parallel source carrier.
    pub radius: f64,
}

/// Construction identities for an external edge and its ordered endpoints.
///
/// The explicit vertex references preserve endpoint constraints even when an
/// operation reverses the edge's stored orientation. No geometric endpoint
/// matching or nearest-point recovery is performed.
#[derive(Debug, Clone)]
pub struct ExternalLineReference {
    /// Edge identity.
    pub edge: PersistentRef,
    /// Identity of the endpoint exposed as the external line's start.
    pub start: PersistentRef,
    /// Identity of the endpoint exposed as the external line's end.
    pub end: PersistentRef,
}

/// Exact planar coordinates of one external straight edge.
#[derive(Debug, Clone, Copy)]
pub struct ExternalSketchLine {
    /// Image of the source edge's start vertex.
    pub start: Point2,
    /// Image of the source edge's end vertex.
    pub end: Point2,
}
