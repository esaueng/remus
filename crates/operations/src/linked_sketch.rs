//! Persistent planar sketch attachments and exact external line inputs.
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
    /// Only straight source edges are qualified.
    #[error("linked sketch external input requires a non-degenerate line")]
    UnsupportedCurve,
    /// Vertex lineage does not describe the resolved edge endpoints.
    #[error("linked sketch endpoint references do not match the resolved edge")]
    EndpointMismatch,
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
        if !frame_valid(&Frame3 {
            origin,
            ..self.frame
        }) {
            return Err(LinkedSketchError::InvalidFrame);
        }
        Ok(Frame3 {
            origin,
            ..self.frame
        })
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
