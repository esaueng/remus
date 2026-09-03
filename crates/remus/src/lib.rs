#![doc = include_str!("../README.md")]

mod model;

pub use model::Model;

// Stable, typed errors are available without making users learn the internal
// crate graph.
pub use remus_check::CheckError;
pub use remus_check::properties::GProps;
pub use remus_heal::HealError;
pub use remus_io::IoError;
pub use remus_math::MathError;
pub use remus_operations::OperationsError;
pub use remus_sketch::SketchError;
pub use remus_topology::TopologyError;

// Policy and the common geometry values passed through the facade.
pub use remus_math::aabb::Aabb3;
pub use remus_math::context::{CancellationToken, FallbackPolicy, OperationContext, WorkBudgets};
pub use remus_math::nurbs::NurbsCurve;
pub use remus_math::vec::{Point3, Vec3};

// Session-owned topology, handles, journal, and persistent references.
pub use remus_topology::journal::{EntityKind, Journal, OpId};
pub use remus_topology::naming::{
    Anchor, Discriminator, PersistentRef, Provenance, Resolution, resolve,
};
pub use remus_topology::{EdgeId, FaceId, SolidId, Topology, WireId};

// Construction and modeling operations deliberately exposed by the facade.
pub use remus_operations::blend_ops::{
    BlendError, BlendFaceOrigins, BlendResult, chamfer_v2, fillet_v2,
};
pub use remus_operations::boolean::{
    BooleanOp, BooleanOutcome, BooleanQuality, boolean_with_context,
};
pub use remus_operations::extrude::extrude;
pub use remus_operations::journal_ops::{
    JournaledBlend, JournaledBoolean, boolean_journaled_with_operation as boolean_journaled,
    chamfer_journaled, fillet_journaled,
};
pub use remus_operations::loft::loft;
pub use remus_operations::measure::{
    edge_length, face_area, face_perimeter, mass_properties, solid_bounding_box,
    solid_center_of_mass, solid_surface_area, solid_volume, wire_length,
};
pub use remus_operations::pipe::pipe;
pub use remus_operations::primitives::{
    make_box, make_cone, make_cylinder, make_sphere, make_torus,
};
pub use remus_operations::revolve::revolve;
pub use remus_operations::sweep::sweep;
pub use remus_operations::tessellate::{
    EdgeLines, TriangleMesh, TriangleMeshUV, WeldedMeshQuality, boundary_edge_count, is_watertight,
    non_manifold_edge_count, tessellate, tessellate_solid, tessellate_solid_with_tolerance,
    tessellate_with_tolerance, tessellate_with_uvs, welded_mesh_quality,
};
pub use remus_operations::validate::{
    OrientationCheck, Severity, ValidationIssue, ValidationOptions, ValidationReport,
    euler_characteristic, validate_solid, validate_solid_with_options,
};

// STEP and repair are needed by the native import-repair workflow.
pub use remus_heal::fix::{FixConfig, FixMode, FixResult, fix_shape};
pub use remus_io::ImportLimits;
pub use remus_io::step::{
    StepImportDiagnostic, StepReadResult, StepValidationDiagnostic, StepValidationDiagnosticCode,
    StepValidationOptions, StepValidationProperties, StepValidationProperty, StepValidationReport,
    StepWriteOptions, read_step, read_step_with_limits, read_step_with_limits_and_report,
    read_step_with_report, read_step_with_validation, write_step, write_step_with_options,
};

// The constraint system is independent of B-Rep topology, but belongs at the
// same front door so a solved sketch can feed modeling construction.
pub use remus_sketch::{
    ArcData, ArcId, CircleData, CircleId, Constraint, ConstraintId, DofAnalysis, GcsSystem,
    LineData, LineId, PointData, PointId, SolveClassification, SolveDiagnostics, SolveResult,
};

/// Imports the curated native modeling surface in one statement.
pub mod prelude {
    pub use crate::{
        Aabb3, Anchor, ArcData, ArcId, BlendError, BlendFaceOrigins, BlendResult, BooleanOp,
        BooleanOutcome, BooleanQuality, CancellationToken, CheckError, CircleData, CircleId,
        Constraint, ConstraintId, Discriminator, DofAnalysis, EdgeId, EdgeLines, EntityKind,
        FaceId, FallbackPolicy, FixConfig, FixMode, FixResult, GProps, GcsSystem, HealError,
        ImportLimits, IoError, Journal, JournaledBlend, JournaledBoolean, LineData, LineId,
        MathError, Model, NurbsCurve, OpId, OperationContext, OperationsError, OrientationCheck,
        PersistentRef, Point3, PointData, PointId, Provenance, Resolution, Severity, SketchError,
        SolidId, SolveClassification, SolveDiagnostics, SolveResult, StepImportDiagnostic,
        StepReadResult, StepValidationDiagnostic, StepValidationDiagnosticCode,
        StepValidationOptions, StepValidationProperties, StepValidationProperty,
        StepValidationReport, StepWriteOptions, Topology, TopologyError, TriangleMesh,
        TriangleMeshUV, ValidationIssue, ValidationOptions, ValidationReport, Vec3,
        WeldedMeshQuality, WireId, WorkBudgets, boolean_journaled, boolean_with_context,
        boundary_edge_count, chamfer_journaled, chamfer_v2, edge_length, euler_characteristic,
        extrude, face_area, face_perimeter, fillet_journaled, fillet_v2, fix_shape, is_watertight,
        loft, make_box, make_cone, make_cylinder, make_sphere, make_torus, mass_properties,
        non_manifold_edge_count, pipe, read_step, read_step_with_limits,
        read_step_with_limits_and_report, read_step_with_report, read_step_with_validation,
        resolve, revolve, solid_bounding_box, solid_center_of_mass, solid_surface_area,
        solid_volume, sweep, tessellate, tessellate_solid, tessellate_solid_with_tolerance,
        tessellate_with_tolerance, tessellate_with_uvs, validate_solid,
        validate_solid_with_options, welded_mesh_quality, wire_length, write_step,
        write_step_with_options,
    };
}
