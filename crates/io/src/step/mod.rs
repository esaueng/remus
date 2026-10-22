//! STEP (ISO 10303) data exchange.

pub mod reader;
pub mod writer;

use remus_math::vec::{Point3, Vec3};

pub use reader::assembly::{
    StepAssemblyReadResult, read_step_assembly, read_step_assembly_with_limits,
};
pub use reader::{
    StepImportDiagnostic, StepReadResult, StepValidationDiagnostic, StepValidationDiagnosticCode,
    StepValidationOptions, StepValidationProperties, StepValidationProperty, StepValidationReport,
    read_step, read_step_bodies, read_step_bodies_with_limits, read_step_with_limits,
    read_step_with_limits_and_report, read_step_with_report, read_step_with_validation,
};
pub use writer::assembly::write_step_assembly;
pub use writer::{
    StepWriteOptions, write_step, write_step_bodies, write_step_bodies_with_options,
    write_step_sheets, write_step_with_options,
};

/// Origin of a plane's STEP chart: the foot of `normal · p = d`.
///
/// `FaceSurface::Plane` may store a non-unit normal, so the foot is
/// `normal * d / |normal|²`, not `normal * d`. The arithmetic matches
/// `PlaneChart::canonical` in the topology SameParameter proof bit for bit.
/// Out-of-range inputs refuse the chart rather than changing its certified
/// arithmetic or emitting non-finite STEP coordinates.
fn plane_chart_origin(normal: Vec3, d: f64) -> Option<Point3> {
    let normal_sq = normal.dot(normal);
    if !normal_sq.is_finite() || normal_sq <= 0.0 || !d.is_finite() {
        return None;
    }
    let origin = Point3::new(
        normal.x() * d / normal_sq,
        normal.y() * d / normal_sq,
        normal.z() * d / normal_sq,
    );
    origin
        .0
        .iter()
        .all(|value| value.is_finite())
        .then_some(origin)
}
