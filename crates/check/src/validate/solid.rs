//! Solid validation checks.

use std::collections::HashSet;

use remus_topology::Topology;
use remus_topology::explorer;
use remus_topology::solid::SolidId;

use super::checks::{CheckId, EntityRef, Severity, ValidationIssue};
use crate::CheckError;

/// Check Euler-Poincare formula: V - E + F = 2 per genus-0 closed shell.
///
/// The counts are taken over the WHOLE solid — every shell, not just the outer
/// one — so the expected total is `2 * shells`, not 2. Each closed genus-0
/// surface contributes 2 independently, and a body hollowed by a fully
/// enclosed cavity carries that cavity as a second shell.
///
/// Comparing that total against a flat 2 called every hollow body anomalous.
/// Measured on a 20^3 blank with a 6^3 void: V=16 E=24 F=12, so chi = 4 over
/// two shells — exactly right, and reported as "expected 2 for genus-0".
///
/// WHAT THIS CHECK CANNOT SEE, stated because the old wording implied
/// otherwise. On a B-rep these counts are not the topological invariants: a
/// closed edge is one edge carrying one vertex, and a face may hold inner
/// wires. A 20^3 block with a through hole is genus 1 and should give chi = 0,
/// but its entity counts are V=10 E=15 F=7 and give 2 — it passes for the
/// wrong reason rather than failing for the right one. A shell whose genus is
/// not 0 is therefore outside what this can judge, which is why it stays a
/// `Warning`: evidence of an anomaly, never proof of one.
#[allow(clippy::cast_possible_wrap)]
pub fn check_euler(topo: &Topology, solid_id: SolidId) -> Result<Vec<ValidationIssue>, CheckError> {
    let (faces, edges, vertices) = explorer::solid_entity_counts(topo, solid_id)?;
    let shells = 1 + topo.solid(solid_id)?.inner_shells().len();
    let euler = vertices as i64 - edges as i64 + faces as i64;
    let expected = 2 * shells as i64;
    if euler != expected {
        return Ok(vec![ValidationIssue {
            check: CheckId::SolidEulerCharacteristic,
            severity: Severity::Warning,
            entity: EntityRef::Solid(solid_id),
            description: format!(
                "Euler characteristic V-E+F = {euler} (expected {expected} for {shells} genus-0 shell(s))"
            ),
            deviation: Some((euler - expected).unsigned_abs() as f64),
        }]);
    }
    Ok(vec![])
}

/// Check that no face ID appears in multiple shells.
pub fn check_duplicate_faces(
    topo: &Topology,
    solid_id: SolidId,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let solid = topo.solid(solid_id)?;
    let mut seen = HashSet::new();
    let mut issues = Vec::new();

    let all_shells =
        std::iter::once(solid.outer_shell()).chain(solid.inner_shells().iter().copied());

    for sid in all_shells {
        let shell = topo.shell(sid)?;
        for &fid in shell.faces() {
            if !seen.insert(fid) {
                issues.push(ValidationIssue {
                    check: CheckId::SolidDuplicateFaces,
                    severity: Severity::Error,
                    entity: EntityRef::Solid(solid_id),
                    description: "face appears in multiple shells".into(),
                    deviation: None,
                });
                break;
            }
        }
    }
    Ok(issues)
}
