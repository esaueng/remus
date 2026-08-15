//! V2 offset operations delegating to remus-offset.

use remus_offset::{JointType, OffsetError, OffsetOptions};
use remus_topology::Topology;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

use crate::OperationsError;

/// Map an `OffsetError` to the most appropriate `OperationsError` variant,
/// preserving structured error information where possible.
fn map_offset_error(e: OffsetError) -> OperationsError {
    match e {
        OffsetError::Topology(t) => OperationsError::Topology(t),
        OffsetError::Math(m) => OperationsError::Math(m),

        other => OperationsError::InvalidInput {
            reason: format!("{other}"),
        },
    }
}

fn validate_offset_postcondition(
    topo: &Topology,
    operation: &'static str,
    solid: SolidId,
) -> Result<SolidId, OperationsError> {
    let report = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )?;
    if !report.is_valid() {
        let summary = report
            .issues
            .iter()
            .filter(|issue| issue.severity == remus_check::validate::Severity::Error)
            .take(3)
            .map(|issue| issue.description.as_str())
            .collect::<Vec<_>>()
            .join("; ");
        return Err(OperationsError::InvalidInput {
            reason: format!(
                "{operation} postcondition validation failed with {} error(s): {summary}",
                report.error_count()
            ),
        });
    }
    Ok(solid)
}

/// Offset all faces of a solid (V2 pipeline).
///
/// # Errors
///
/// Returns an error if the offset fails.
pub fn offset_solid_v2(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
) -> Result<SolidId, OperationsError> {
    let result = remus_offset::offset_solid(topo, solid, distance, OffsetOptions::default())
        .map_err(map_offset_error)?;
    validate_offset_postcondition(topo, "offset", result)
}

/// Shell (hollow solid) operation (V2 pipeline).
///
/// # Errors
///
/// Returns an error if the offset fails.
pub fn shell_v2(
    topo: &mut Topology,
    solid: SolidId,
    thickness: f64,
    exclude: &[FaceId],
) -> Result<SolidId, OperationsError> {
    let result =
        remus_offset::thick_solid(topo, solid, thickness, exclude, OffsetOptions::default())
            .map_err(map_offset_error)?;
    validate_offset_postcondition(topo, "shell", result)
}

/// Offset with arc joints (V2 pipeline).
///
/// # Errors
///
/// Returns an error if the offset fails.
pub fn offset_solid_arc_v2(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
) -> Result<SolidId, OperationsError> {
    let options = OffsetOptions {
        joint: JointType::Arc,
        ..Default::default()
    };
    let result =
        remus_offset::offset_solid(topo, solid, distance, options).map_err(map_offset_error)?;
    validate_offset_postcondition(topo, "arc offset", result)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use remus_topology::Topology;

    #[test]
    fn offset_v2_box() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let result = offset_solid_v2(&mut topo, solid, 0.5).unwrap();
        let shell = topo
            .shell(topo.solid(result).unwrap().outer_shell())
            .unwrap();
        assert_eq!(shell.faces().len(), 6);
    }

    #[test]
    fn offset_v2_rejects_cavity_without_dropping_it() {
        let mut topo = Topology::new();
        let outer = crate::primitives::make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
        let inner = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let cavity_shell = topo.solid(inner).unwrap().outer_shell();
        topo.solid_mut(outer).unwrap().add_inner_shell(cavity_shell);

        let error = offset_solid_v2(&mut topo, outer, 0.5).unwrap_err();
        assert!(matches!(
            error,
            OperationsError::InvalidInput { ref reason }
                if reason.contains("cavity shells")
        ));
        assert_eq!(topo.solid(outer).unwrap().inner_shells(), &[cavity_shell]);
    }
}
