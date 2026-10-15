//! STEP (ISO 10303) data exchange.

pub mod reader;
pub mod writer;

use remus_math::vec::{Point3, Vec3};

pub use reader::{
    StepImportDiagnostic, StepReadResult, StepValidationDiagnostic, StepValidationDiagnosticCode,
    StepValidationOptions, StepValidationProperties, StepValidationProperty, StepValidationReport,
    read_step, read_step_bodies, read_step_bodies_with_limits, read_step_with_limits,
    read_step_with_limits_and_report, read_step_with_report, read_step_with_validation,
};
pub use writer::{
    StepWriteOptions, write_step, write_step_bodies, write_step_bodies_with_options,
    write_step_sheets, write_step_with_options,
};

/// Origin of a plane's STEP chart: the foot of `normal · p = d`.
///
/// `FaceSurface::Plane` may store a non-unit normal, so the foot is
/// `normal * d / |normal|²`, not `normal * d`. The arithmetic matches
/// `PlaneChart::canonical` in the topology SameParameter proof bit for bit.
fn plane_chart_origin(normal: Vec3, d: f64) -> Point3 {
    let normal_sq = normal.dot(normal);
    Point3::new(
        normal.x() * d / normal_sq,
        normal.y() * d / normal_sq,
        normal.z() * d / normal_sq,
    )
}
