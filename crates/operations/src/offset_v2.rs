//! V2 offset operations delegating to remus-offset.

use remus_offset::{JointType, OffsetError, OffsetOptions};
use remus_topology::Topology;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

use crate::OperationsError;
use crate::evolution::EvolutionMap;

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
    ensure_not_collapsed(topo, operation, solid)?;
    Ok(solid)
}

/// Reject an offset that pushed the boundary through itself.
///
/// An inward offset larger than the body's own half-thickness carries every
/// face past its opposite number. Nothing in the per-face offset notices:
/// the radius guards in `crates/offset/src/offset.rs` catch a cylinder,
/// cone, sphere, or torus whose radius goes non-positive, but a plane has no
/// radius — it is simply translated — so an all-planar solid has no
/// per-face collapse condition at all. Assembly then succeeds on the
/// inverted arrangement and returns a solid that is inside out.
///
/// On a 10 mm box: -4.9 gives the correct 0.008 mm^3 result, -5.0 happens to
/// fail in assembly, and -6, -10, and -1e6 all returned `Ok` with volumes of
/// 8, 1000 (the untouched input), and 8e18. `validate_solid` above does not
/// see it, because the check crate has no shell-orientation check.
///
/// A negative signed volume on the outer shell is exactly that inversion,
/// and it is the one signature every collapsed case shares, so it is checked
/// here rather than by widening the general validator.
fn ensure_not_collapsed(
    topo: &Topology,
    operation: &'static str,
    solid: SolidId,
) -> Result<(), OperationsError> {
    let gauss_order = remus_check::properties::PropertiesOptions::default().gauss_order;
    let Some(floor) = crate::measure::negligible_volume(topo, solid) else {
        return Ok(());
    };
    let outer = topo.solid(solid)?.outer_shell();
    let Some(signed) = crate::measure::shell_signed_volume(topo, outer, gauss_order) else {
        return Ok(());
    };
    if signed < -floor {
        return Err(OperationsError::InvalidInput {
            reason: format!(
                "{operation} collapsed the solid: the result's outer shell is inside out \
                 (signed volume {signed}), which means the offset carried the boundary \
                 through itself"
            ),
        });
    }
    Ok(())
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

/// Offset every face and return its construction-derived face evolution.
///
/// Each source face is carried to exactly one result face. The lower-level
/// offset engine validates that this correspondence covers both the complete
/// source and result face sets before exposing it.
///
/// # Errors
///
/// Returns an error if the offset fails, its result fails validation, or the
/// construction cannot prove a total one-to-one face map.
pub fn offset_solid_v2_with_evolution(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
) -> Result<(SolidId, EvolutionMap), OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        let result = remus_offset::offset_solid_with_face_map(
            topo,
            solid,
            distance,
            OffsetOptions::default(),
        )
        .map_err(map_offset_error)?;
        let solid = validate_offset_postcondition(topo, "offset", result.solid)?;
        let mut evolution = EvolutionMap::exact();
        for (source, result_face) in result.face_map {
            evolution.add_modified(source, result_face.index());
        }
        Ok((solid, evolution))
    })
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
    #![allow(clippy::unwrap_used, clippy::panic)]
    use super::*;
    use remus_math::det_hash::DetHashMap;
    use remus_topology::Topology;
    use remus_topology::explorer::solid_faces;
    use remus_topology::face::FaceSurface;

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
    fn offset_evolution_is_total_exact_and_geometrically_true() {
        let mut topo = Topology::new();
        let source = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let source_faces = solid_faces(&topo, source).unwrap();
        let source_planes = source_faces
            .iter()
            .map(|face| match topo.face(*face).unwrap().surface() {
                FaceSurface::Plane { normal, d } => (
                    face.index(),
                    (*normal, *d, topo.face(*face).unwrap().is_reversed()),
                ),
                other => panic!("box face must be planar, got {}", other.type_tag()),
            })
            .collect::<DetHashMap<_, _>>();

        let (result, evolution) = offset_solid_v2_with_evolution(&mut topo, source, 0.5).unwrap();

        assert!(evolution.origin.is_exact());
        assert!(evolution.generated.is_empty());
        assert!(evolution.deleted.is_empty());
        assert!(evolution.unresolved.is_empty());
        assert_eq!(evolution.modified.len(), source_faces.len());

        let result_faces = solid_faces(&topo, result).unwrap();
        let claimed = evolution
            .modified
            .values()
            .flatten()
            .copied()
            .collect::<std::collections::BTreeSet<_>>();
        assert_eq!(claimed.len(), result_faces.len());
        assert!(
            result_faces
                .iter()
                .all(|face| claimed.contains(&face.index()))
        );

        for (source_index, outputs) in &evolution.modified {
            assert_eq!(outputs.len(), 1, "offset face identity must be one-to-one");
            let (source_normal, source_d, source_reversed) = {
                let (normal, d, reversed) = source_planes[source_index];
                (normal, d, reversed)
            };
            let result_face = topo.face_id_from_index(outputs[0]).unwrap();
            let result_face_data = topo.face(result_face).unwrap();
            let FaceSurface::Plane {
                normal: result_normal,
                d: result_d,
            } = result_face_data.surface()
            else {
                panic!("mapped box face must remain planar");
            };
            assert!((*result_normal - source_normal).length() < 1e-12);
            assert_eq!(result_face_data.is_reversed(), source_reversed);
            let signed_distance = if source_reversed { -0.5 } else { 0.5 };
            assert!(
                (*result_d - source_d - signed_distance).abs() < 1e-12,
                "mapped face must lie exactly 0.5 model units outward"
            );
        }

        let volume = crate::measure::mass_properties(&topo, result).unwrap().mass;
        assert!((volume - 27.0).abs() < 1e-9, "3x3x3 volume: {volume}");
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
