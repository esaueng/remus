//! # remus-operations
//!
//! CAD modeling operations for B-Rep solids. Layer L3, depending on
//! `remus-math`, `remus-topology`, `remus-geometry`, `remus-algo`,
//! `remus-blend`, `remus-heal`, `remus-check`, and `remus-offset`.
//!
//! # Module families
//!
//! | Family | Modules | Purpose |
//! |--------|---------|---------|
//! | **Core** | [`primitives`], [`extrude`], [`revolve`], [`sweep`], [`loft`], [`pipe`], [`helix`] | Shape creation |
//! | **Transform** | [`transform`], [`copy`], [`mirror`], [`pattern`] | Spatial operations |
//! | **Boolean** | [`boolean`], [`mesh_boolean`] | Set operations |
//! | **Blend** | [`fillet`], [`chamfer`], [`blend_ops`], [`resize_blend`] | Edge smoothing and exact band editing |
//! | **Offset** | [`offset_face`], [`offset_trim`], [`offset_v2`], [`offset_wire`] | Wall thickness |
//! | **Direct edit** | [`push_pull`] | Move a face of an existing solid |
//! | **Surface** | [`fill_face`], [`thicken`], [`shell_op`], [`draft`], [`section`], [`split`] | Surface/solid modification |
//! | **Repair** | [`heal`], [`defeature`], [`sew`], [`untrim`] | Shape fixing |
//! | **Analysis** | [`measure`], [`distance`], [`classify`], [`validate`], [`query`], [`feature_recognition`] | Interrogation |
//! | **Tessellation** | [`tessellate`] | Mesh generation |
//! | **Infrastructure** | [`assembly`], [`compound_ops`], [`evolution`], [`sketch`] | Utilities |

use remus_math::vec::{Point3, Vec3};

pub mod extrude;
pub mod helix;
pub mod loft;
pub mod pipe;
pub mod primitives;
pub mod projection;
pub mod revolve;
pub mod sweep;

pub mod copy;
pub mod mirror;
pub mod pattern;
pub mod transform;

pub mod boolean;
pub mod mesh_boolean;

pub mod blend_ops;
pub mod chamfer;
pub mod face_face_blend;
pub mod fillet;
pub mod resize_blend;

pub mod offset_face;
pub mod offset_trim;
pub mod offset_v2;
pub mod offset_wire;
pub mod push_pull;
pub mod replace_surface;

pub mod draft;
pub mod fill_face;
pub mod imprint;
pub mod section;
pub mod shell_op;
pub mod split;
pub mod thicken;

pub mod defeature;
pub mod heal;
pub mod sew;
pub mod untrim;

pub mod classify;
pub mod distance;
pub mod feature_recognition;
pub mod measure;
pub mod query;
pub mod validate;

pub mod tessellate;

pub mod assembly;
pub(crate) mod cap;
pub mod compound_ops;
pub mod evolution;
pub mod journal_ops;
pub mod sketch;
pub(crate) mod winding;

#[cfg(test)]
pub(crate) mod test_helpers;

/// Compute `n · p` treating a `Point3` as a direction vector.
///
/// Equivalent to the dot product `n.x*p.x + n.y*p.y + n.z*p.z`, used
/// for the plane equation `n · point = d`.
fn dot_normal_point(n: Vec3, p: Point3) -> f64 {
    n.dot(Vec3::new(p.x(), p.y(), p.z()))
}

/// Resolve an edge's stored parameter authority without reconstructing it
/// from endpoint projection.
pub(crate) fn authoritative_edge_domain(
    edge: &remus_topology::edge::Edge,
    context: &str,
) -> Result<(f64, f64), OperationsError> {
    edge.strict_domain()
        .map_err(|error| OperationsError::InvalidInput {
            reason: format!("{context} requires an authoritative edge domain: {error}"),
        })
}

pub(crate) fn preflight_face_edge_domains(
    topo: &remus_topology::Topology,
    faces: &[remus_topology::face::FaceId],
    context: &str,
) -> Result<(), OperationsError> {
    let mut seen = std::collections::BTreeSet::new();
    for &face_id in faces {
        let face = topo.face(face_id)?;
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire_id)?.edges() {
                if seen.insert(oriented.edge().index()) {
                    authoritative_edge_domain(topo.edge(oriented.edge())?, context)?;
                }
            }
        }
    }
    Ok(())
}

/// Compatibility adapter for public raw-wire operations.
///
/// Legacy callers may provide a curved edge without a stored range. Establish
/// that authority once, validate it against the actual vertices, and persist
/// it before any downstream consumer runs. Malformed stored authority is never
/// replaced.
pub(crate) fn normalize_legacy_edge_domain(
    topo: &mut remus_topology::Topology,
    edge_id: remus_topology::edge::EdgeId,
    context: &str,
) -> Result<(f64, f64), OperationsError> {
    use remus_topology::edge::{Edge, EdgeDomainError};

    let edge = topo.edge(edge_id)?;
    match edge.strict_domain() {
        Ok(range) => return Ok(range),
        Err(EdgeDomainError::Missing { .. }) => {}
        Err(error) => {
            return Err(OperationsError::InvalidInput {
                reason: format!("{context} has invalid stored parameter authority: {error}"),
            });
        }
    }

    let (start_id, end_id, curve, edge_tolerance) = {
        let edge = topo.edge(edge_id)?;
        (
            edge.start(),
            edge.end(),
            edge.curve().clone(),
            edge.tolerance(),
        )
    };
    let start_vertex = topo.vertex(start_id)?;
    let end_vertex = topo.vertex(end_id)?;
    let start_tolerance = start_vertex.tolerance();
    let end_tolerance = end_vertex.tolerance();
    if !start_tolerance.is_finite()
        || start_tolerance < 0.0
        || !end_tolerance.is_finite()
        || end_tolerance < 0.0
        || edge_tolerance.is_some_and(|value| !value.is_finite() || value < 0.0)
    {
        return Err(OperationsError::InvalidInput {
            reason: format!("{context} has invalid tolerance authority"),
        });
    }
    let start = start_vertex.point();
    let end = end_vertex.point();
    let reconstructed = curve.reconstruct_domain_from_endpoints(start, end);
    let tolerance = Edge::with_tolerance(start_id, end_id, curve.clone(), edge_tolerance)
        .effective_tolerance(start_tolerance.max(end_tolerance));
    let endpoint_residuals = |range: (f64, f64)| {
        (
            (curve.evaluate_with_endpoints(range.0, start, end) - start).length(),
            (curve.evaluate_with_endpoints(range.1, start, end) - end).length(),
        )
    };
    let direct_residuals = endpoint_residuals(reconstructed);
    let reversed = (reconstructed.1, reconstructed.0);
    let reversed_residuals = endpoint_residuals(reversed);
    let (range, endpoint_residuals) =
        if direct_residuals.0 <= tolerance && direct_residuals.1 <= tolerance {
            (reconstructed, direct_residuals)
        } else if reversed_residuals.0 <= tolerance && reversed_residuals.1 <= tolerance {
            (reversed, reversed_residuals)
        } else {
            (reconstructed, direct_residuals)
        };
    let mut probe = Edge::with_tolerance(start_id, end_id, curve.clone(), edge_tolerance);
    probe.set_trim(Some(range));
    probe
        .strict_domain()
        .map_err(|error| OperationsError::InvalidInput {
            reason: format!("{context} cannot establish parameter authority: {error}"),
        })?;
    let midpoint = f64::midpoint(range.0, range.1);
    let evaluated_mid = curve.evaluate_with_endpoints(midpoint, start, end);
    let tangent_mid = curve.tangent_with_endpoints(midpoint, start, end);
    if evaluated_mid.0.iter().any(|value| !value.is_finite())
        || tangent_mid.0.iter().any(|value| !value.is_finite())
    {
        return Err(OperationsError::InvalidInput {
            reason: format!("{context} reconstructed interior is not finite"),
        });
    }
    for (label, residual) in [
        ("start", endpoint_residuals.0),
        ("end", endpoint_residuals.1),
    ] {
        if !residual.is_finite() || residual > tolerance {
            return Err(OperationsError::InvalidInput {
                reason: format!(
                    "{context} reconstructed {label} is not certified (residual {residual}, \
                     tolerance {tolerance})"
                ),
            });
        }
    }

    topo.edge_mut(edge_id)?.set_trim(Some(range));
    Ok(range)
}

/// Errors from modeling operations.
#[derive(Debug, thiserror::Error)]
pub enum OperationsError {
    /// The exact pipeline could not produce this result and the caller's
    /// fallback policy is `ExactOnly`, so the approximate path was declined
    /// (kernel operation contract; taxonomy category `quality_refused`).
    #[error(
        "exact-only policy: the exact boolean pipeline could not produce \
         this result and the approximate fallback was declined"
    )]
    ExactOnlyUnattainable,

    /// The input shape is invalid for this operation.
    #[error("invalid input: {reason}")]
    InvalidInput {
        /// Description of what is wrong.
        reason: String,
    },

    /// The operation produced a non-manifold result.
    #[error("non-manifold result")]
    NonManifoldResult,

    /// The operation produced an empty result (no geometry).
    ///
    /// Boolean operations return this when the algebraic outcome is the
    /// empty set: `Cut(A, B)` when `A ⊆ B`, or any operation on
    /// pre-collapsed inputs. Distinguishable from [`InvalidInput`] so
    /// callers can apply empty-operand identity rules without
    /// string-matching the error message.
    ///
    /// [`InvalidInput`]: Self::InvalidInput
    #[error("empty result: {reason}")]
    EmptyResult {
        /// Description of the empty-result scenario.
        reason: String,
    },

    /// The input is well-formed but the operation has no exact construction
    /// for this configuration.
    ///
    /// Returned in place of an approximate, degraded or invalid result, so
    /// callers can distinguish "this cannot be built exactly" from "these
    /// arguments are wrong" ([`InvalidInput`]) without matching on message
    /// text. Retrying with the same inputs will not help; the caller must
    /// either change the model or accept an explicitly approximate route.
    ///
    /// [`InvalidInput`]: Self::InvalidInput
    #[error("{operation}: unsupported configuration: {reason}")]
    Unsupported {
        /// Name of the operation that declined.
        operation: &'static str,
        /// Why this configuration has no exact construction.
        reason: String,
    },

    /// A measurement is undefined for the supplied dimensional body class.
    #[error("{operation} requires a {expected} body, but the supplied body is {actual}")]
    BodyClassMeasureMismatch {
        /// Measurement that was requested.
        operation: &'static str,
        /// Body class on which the measurement is defined.
        expected: &'static str,
        /// Actual body class supplied by the caller.
        actual: &'static str,
    },

    /// An operation has no qualified implementation for this body class.
    #[error("{operation} does not support {actual} bodies")]
    BodyClassOperationUnsupported {
        /// Operation that was requested.
        operation: &'static str,
        /// Actual body class supplied by the caller.
        actual: &'static str,
    },

    /// A newly constructed body failed its class-specific postcondition.
    #[error("constructed {body_class} body failed validation with {error_count} error(s)")]
    BodyValidationFailed {
        /// Class of body that was being constructed.
        body_class: &'static str,
        /// Number of error-severity validation findings.
        error_count: usize,
    },

    /// Healing completed its mutation pass, but the result did not satisfy
    /// both independent solid validators. The attempted repairs were rolled
    /// back and remain available for disclosure on the error value.
    #[error(
        "healing result refused: operations validator found {operations_errors} error(s), \
         check validator found {check_errors} error(s)"
    )]
    HealingValidationFailed {
        /// Error count from the L3 operations validator.
        operations_errors: usize,
        /// Error count from the independent L2 check validator.
        check_errors: usize,
        /// Exact repair categories attempted before rollback.
        healing: crate::heal::HealingReport,
    },

    /// A validator errored before it could establish the healed result's
    /// validity. The attempted repairs were rolled back.
    #[error("healing verification unavailable from {validator}: {reason}")]
    HealingVerificationUnavailable {
        /// Validator that could not produce a verdict.
        validator: &'static str,
        /// Typed lower-layer failure rendered for diagnostics.
        reason: String,
        /// Exact repair categories attempted before rollback.
        healing: crate::heal::HealingReport,
    },

    /// A configured fixer declined at least one requested repair. Any repairs
    /// made earlier in the pass were rolled back and are disclosed here.
    #[error("configured healing refused {refusal_count} unresolved repair site(s)")]
    HealingRepairRefused {
        /// Total number of sites whose requested repair was declined.
        refusal_count: usize,
        /// Repairs attempted before the refusal was known.
        actions: Vec<remus_heal::fix::RepairAction>,
        /// Typed reasons and affected-site counts.
        refusals: Vec<remus_heal::fix::RepairRefusal>,
    },

    /// Configured healing completed, but the result failed validation. The
    /// complete attempted repair disclosure is retained after rollback.
    #[error(
        "configured healing result refused: operations validator found {operations_errors} \
         error(s), check validator found {check_errors} error(s)"
    )]
    ConfiguredHealingValidationFailed {
        /// Error count from the L3 operations validator.
        operations_errors: usize,
        /// Error count from the independent L2 check validator.
        check_errors: usize,
        /// Repairs attempted before validation vetoed the commit.
        actions: Vec<remus_heal::fix::RepairAction>,
    },

    /// A validator errored before it could establish configured healing's
    /// validity. The complete attempted repair disclosure is retained.
    #[error("configured healing verification unavailable from {validator}: {reason}")]
    ConfiguredHealingVerificationUnavailable {
        /// Validator that could not produce a verdict.
        validator: &'static str,
        /// Typed lower-layer failure rendered for diagnostics.
        reason: String,
        /// Repairs attempted before verification failed.
        actions: Vec<remus_heal::fix::RepairAction>,
    },

    /// A pattern would place two instances over the same material volume.
    ///
    /// Returning the instances as an ordinary compound would double-count
    /// mass and present intersecting bodies as a valid pattern. Until pattern
    /// fusing can also preserve truthful face evolution, the exact operation
    /// refuses this configuration instead.
    #[error(
        "pattern instances {first} and {second} overlap by {overlap_volume:e} \
         model-unit^3 (material-overlap floor {threshold:e}); exact instance \
         fusing with face evolution is not yet supported"
    )]
    PatternInstancesOverlap {
        /// Zero-based index of the first overlapping pattern instance.
        first: usize,
        /// Zero-based index of the second overlapping pattern instance.
        second: usize,
        /// Measured volume of their exact intersection.
        overlap_volume: f64,
        /// Scale-relative volume below which contact is non-material.
        threshold: f64,
    },

    /// A referenced topology entity was not found.
    #[error(transparent)]
    Topology(#[from] remus_topology::TopologyError),

    /// A math error occurred during the operation.
    #[error(transparent)]
    Math(#[from] remus_math::MathError),

    /// A GFA algorithm error occurred.
    #[error("algo: {0}")]
    Algo(#[from] remus_algo::error::AlgoError),

    /// A blend (fillet/chamfer v2) error occurred.
    #[error("blend: {0}")]
    Blend(#[from] remus_blend::BlendError),

    /// An exact blend-band resize was refused.
    #[error("resize blend: {0}")]
    ResizeBlend(#[from] resize_blend::ResizeBlendError),

    /// A check (classification/validation/distance) error occurred.
    #[error("check: {0}")]
    Check(#[from] remus_check::CheckError),

    /// A geometry conversion error occurred.
    #[error("geometry: {0}")]
    Geometry(#[from] remus_geometry::error::GeomError),

    /// A shape-healing operation failed.
    #[error("heal: {0}")]
    Heal(#[from] remus_heal::HealError),

    /// A topology-preserving offset or move-face operation was refused.
    #[error("offset: {0}")]
    Offset(#[from] remus_offset::OffsetError),

    /// An operation completed only a subset of the requested items.
    #[error("{operation} produced a partial result: {succeeded} succeeded, {failed} failed")]
    PartialResult {
        /// Name of the operation.
        operation: &'static str,
        /// Number of requested items that succeeded.
        succeeded: usize,
        /// Number of requested items that failed.
        failed: usize,
    },
}

#[cfg(test)]
mod edge_domain_authority_tests {
    #![allow(clippy::expect_used, clippy::unwrap_used)]

    use super::*;
    use remus_math::curves::Circle3D;
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::explorer::solid_edges;
    use remus_topology::vertex::Vertex;

    #[test]
    fn authoritative_edge_domain_preserves_descending_and_lifted_ranges() {
        let mut topo = Topology::new();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();

        for range in [(5.5, 0.5), (5.5, std::f64::consts::TAU + 0.5)] {
            let start = topo.add_vertex(Vertex::new(circle.evaluate(range.0), 1.0e-7));
            let end = topo.add_vertex(Vertex::new(circle.evaluate(range.1), 1.0e-7));
            let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle.clone()));
            edge.set_trim(Some(range));
            assert_eq!(authoritative_edge_domain(&edge, "test").unwrap(), range);
        }

        let start = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1.0e-7));
        let missing = Edge::new(start, start, EdgeCurve::Circle(circle));
        let error = authoritative_edge_domain(&missing, "test probe").unwrap_err();
        assert!(matches!(error, OperationsError::InvalidInput { .. }));
        assert!(
            error
                .to_string()
                .contains("test probe requires an authoritative edge domain")
        );

        let line = Edge::new(start, start, EdgeCurve::Line);
        assert_eq!(
            authoritative_edge_domain(&line, "line").unwrap(),
            (0.0, 1.0)
        );
    }

    #[test]
    fn bounding_box_refuses_missing_curved_authority_without_mutation() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_cylinder(&mut topo, 2.0, 3.0).unwrap();
        let rim = solid_edges(&topo, solid)
            .unwrap()
            .into_iter()
            .find(|&edge_id| {
                topo.edge(edge_id)
                    .is_ok_and(|edge| matches!(edge.curve(), EdgeCurve::Circle(_)))
            })
            .expect("cylinder must have a circular rim");
        topo.edge_mut(rim).unwrap().set_trim(None);
        let before = (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
            topo.allocated_slot_count(),
        );

        let bbox_error = crate::measure::solid_bounding_box(&topo, solid).unwrap_err();
        assert!(bbox_error.to_string().contains("authoritative edge domain"));
        // The primitive closed form consumes no edge parameters, so it remains
        // available; strict refusal is attached to actual edge-domain readers,
        // not imposed as a blanket solid-validity check.
        assert!(crate::measure::solid_volume(&topo, solid, 0.01).is_ok());

        assert_eq!(
            before,
            (
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_wires(),
                topo.num_faces(),
                topo.num_shells(),
                topo.num_solids(),
                topo.allocated_slot_count(),
            )
        );
    }
}
