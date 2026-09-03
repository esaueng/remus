//! Imports bounded STEP healing diagnostics, validates, and continues safely.

use std::error::Error;

use remus::prelude::*;

const IMPERFECT_STEP: &str = include_str!("data/imperfect.step");
const EXPECTED_VOLUME: f64 = 1.062_909_27;

fn import_repair_and_boolean() -> Result<(), Box<dyn Error>> {
    let mut model = Model::new();
    let imported = model.read_step_with_report(IMPERFECT_STEP)?;
    assert_eq!(imported.solids().len(), 1);
    let recovery_diagnostics = imported
        .diagnostics()
        .iter()
        .filter(|diagnostic| {
            matches!(
                diagnostic,
                StepImportDiagnostic::UntrimmedNurbsDomainRecovered { .. }
            )
        })
        .count();
    assert_eq!(recovery_diagnostics, 2);
    let original = imported.solids()[0];

    // A caller may tolerate bounded, typed recovery diagnostics when the
    // resulting body validates; otherwise it runs the healing pipeline.
    let validation = model.validate(original)?;
    let usable = if validation.is_valid() {
        original
    } else {
        let (repaired, _report) = model.heal(original, &FixConfig::default())?;
        assert!(model.validate(repaired)?.is_valid());
        repaired
    };

    let volume = model.volume(usable, 0.02)?;
    assert!((volume - EXPECTED_VOLUME).abs() < 5.0e-5);

    // Intersect with a strictly enclosing box. The exact containment path
    // must preserve the imported body and may not silently mesh-fallback.
    let bounds = model.bounding_box(usable)?;
    let margin = 1.0;
    let enclosure = model.make_box(
        bounds.max.x() - bounds.min.x() + 2.0 * margin,
        bounds.max.y() - bounds.min.y() + 2.0 * margin,
        bounds.max.z() - bounds.min.z() + 2.0 * margin,
    )?;
    model.transform(
        enclosure,
        &Mat4::translation(
            bounds.min.x() - margin,
            bounds.min.y() - margin,
            bounds.min.z() - margin,
        ),
    )?;
    let intersection = model.intersect(usable, enclosure)?;
    assert_eq!(intersection.quality, BooleanQuality::Exact);
    assert!(model.validate(intersection.solid)?.is_valid());
    let result_volume = model.volume(intersection.solid, 0.02)?;
    assert!((result_volume - volume).abs() < 1.0e-9);
    let mesh = model.tessellate(intersection.solid, 0.02)?;
    let quality = welded_mesh_quality(&mesh);
    assert!(quality.is_watertight());
    assert_eq!(quality.non_manifold_edges, 0);

    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    import_repair_and_boolean()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn import_validate_heal_or_tolerate_then_boolean_workflow() -> Result<(), Box<dyn Error>> {
        import_repair_and_boolean()
    }
}
