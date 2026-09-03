//! Hierarchical shape validation.

pub mod checks;
pub(crate) mod edge;
pub(crate) mod face;
pub(crate) mod finite;
pub mod shell;
pub(crate) mod solid;
pub(crate) mod vertex;
pub(crate) mod wire;

pub use checks::{CheckId, EntityRef, Severity, ValidationIssue, ValidationReport};
pub use face::check_face_inner_wire_orientation;
pub use wire::check_wire_self_intersection;

use std::collections::{HashMap, HashSet};

use remus_topology::shell::ShellId;
use remus_topology::solid::SolidId;
use remus_topology::wire::WireId;
use remus_topology::{BodyClass, Topology};

use crate::CheckError;

/// Options controlling which checks run.
#[derive(Debug, Clone)]
pub struct ValidateOptions {
    /// Geometric tolerance scale factor (default 1.0).
    pub tolerance_scale: f64,
    /// Checks to skip.
    pub disabled_checks: HashSet<CheckId>,
}

impl Default for ValidateOptions {
    fn default() -> Self {
        Self {
            tolerance_scale: 1.0,
            disabled_checks: HashSet::new(),
        }
    }
}

/// Validate a solid (full check suite).
///
/// Runs solid-level checks, then shell and wire checks on each shell.
///
/// # Errors
///
/// Returns an error if topology lookups fail.
pub fn validate_solid(
    topo: &Topology,
    solid_id: SolidId,
    options: &ValidateOptions,
) -> Result<ValidationReport, CheckError> {
    let mut report = ValidationReport::default();
    let solid_data = topo.solid(solid_id)?;

    if !options
        .disabled_checks
        .contains(&CheckId::SolidEulerCharacteristic)
    {
        report.issues.extend(solid::check_euler(topo, solid_id)?);
    }
    if !options
        .disabled_checks
        .contains(&CheckId::SolidDuplicateFaces)
    {
        report
            .issues
            .extend(solid::check_duplicate_faces(topo, solid_id)?);
    }

    let shells: Vec<_> = std::iter::once(solid_data.outer_shell())
        .chain(solid_data.inner_shells().iter().copied())
        .collect();
    for &sid in &shells {
        let actual = topo.shell(sid)?.body_class();
        if actual == BodyClass::Solid {
            report.issues.extend(validate_shell_checks(
                topo,
                sid,
                options,
                ShellValidationProfile::Solid,
            )?);
        } else if !options
            .disabled_checks
            .contains(&CheckId::BodyClassResolved)
        {
            report.issues.push(body_class_issue(
                EntityRef::Shell(sid),
                BodyClass::Solid,
                actual,
            ));
        }
    }

    Ok(report)
}

/// Validate a single shell.
///
/// Runs shell-level checks and wire checks for each face.
///
/// # Errors
///
/// Returns an error if topology lookups fail.
pub fn validate_shell(
    topo: &Topology,
    shell_id: ShellId,
    options: &ValidateOptions,
) -> Result<ValidationReport, CheckError> {
    let mut report = ValidationReport::default();
    let profile = match topo.shell(shell_id)?.body_class() {
        BodyClass::Solid => ShellValidationProfile::Solid,
        BodyClass::Sheet => ShellValidationProfile::Sheet,
        actual => {
            if !options
                .disabled_checks
                .contains(&CheckId::BodyClassResolved)
            {
                report.issues.push(body_class_issue(
                    EntityRef::Shell(shell_id),
                    BodyClass::Sheet,
                    actual,
                ));
            }
            return Ok(report);
        }
    };
    report
        .issues
        .extend(validate_shell_checks(topo, shell_id, options, profile)?);
    Ok(report)
}

/// Validate a shell specifically as a first-class sheet body.
///
/// Free boundary edges are warnings; non-manifold use and inconsistent
/// shared-edge orientation remain errors.
///
/// # Errors
///
/// Returns an error if topology lookups fail.
pub fn validate_sheet_body(
    topo: &Topology,
    shell_id: ShellId,
    options: &ValidateOptions,
) -> Result<ValidationReport, CheckError> {
    let mut report = ValidationReport::default();
    let actual = topo.shell(shell_id)?.body_class();
    if actual != BodyClass::Sheet {
        if !options
            .disabled_checks
            .contains(&CheckId::BodyClassResolved)
        {
            report.issues.push(body_class_issue(
                EntityRef::Shell(shell_id),
                BodyClass::Sheet,
                actual,
            ));
        }
        return Ok(report);
    }
    report.issues.extend(validate_shell_checks(
        topo,
        shell_id,
        options,
        ShellValidationProfile::Sheet,
    )?);
    Ok(report)
}

/// Validate a first-class wire body.
///
/// # Errors
///
/// Returns an error if topology lookups fail.
pub fn validate_wire_body(
    topo: &Topology,
    wire_id: WireId,
    options: &ValidateOptions,
) -> Result<ValidationReport, CheckError> {
    let mut report = ValidationReport::default();
    let actual = topo.wire(wire_id)?.body_class();
    if actual != BodyClass::Wire {
        if !options
            .disabled_checks
            .contains(&CheckId::BodyClassResolved)
        {
            report.issues.push(body_class_issue(
                EntityRef::Wire(wire_id),
                BodyClass::Wire,
                actual,
            ));
        }
        return Ok(report);
    }
    report
        .issues
        .extend(validate_wire_checks(topo, wire_id, options)?);
    Ok(report)
}

#[derive(Debug, Clone, Copy)]
enum ShellValidationProfile {
    Solid,
    Sheet,
}

fn body_class_issue(entity: EntityRef, expected: BodyClass, actual: BodyClass) -> ValidationIssue {
    ValidationIssue {
        check: CheckId::BodyClassResolved,
        severity: Severity::Error,
        entity,
        description: format!(
            "body class unresolved: expected {}, found {}",
            expected.as_str(),
            actual.as_str()
        ),
        deviation: None,
    }
}

/// Internal: run shell + wire checks on a shell.
fn validate_shell_checks(
    topo: &Topology,
    shell_id: ShellId,
    options: &ValidateOptions,
    profile: ShellValidationProfile,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let mut issues = Vec::new();

    if !options.disabled_checks.contains(&CheckId::ShellEmpty) {
        issues.extend(shell::check_shell_empty(topo, shell_id)?);
    }
    if !options.disabled_checks.contains(&CheckId::ShellConnected) {
        issues.extend(shell::check_shell_connected(topo, shell_id)?);
    }
    match profile {
        ShellValidationProfile::Solid => {
            if !options.disabled_checks.contains(&CheckId::ShellClosed) {
                issues.extend(shell::check_shell_closed(topo, shell_id)?);
            }
            if !options
                .disabled_checks
                .contains(&CheckId::ShellOrientationConsistent)
            {
                issues.extend(shell::check_shell_orientation(topo, shell_id)?);
            }
        }
        ShellValidationProfile::Sheet => {
            issues.extend(
                shell::check_sheet_boundary(topo, shell_id)?
                    .into_iter()
                    .filter(|issue| !options.disabled_checks.contains(&issue.check)),
            );
            if !options
                .disabled_checks
                .contains(&CheckId::SheetOrientationConsistent)
            {
                issues.extend(shell::check_sheet_orientation(topo, shell_id)?);
            }
        }
    }

    let shell = topo.shell(shell_id)?;
    let mut wire_periodicity = HashMap::new();
    for &fid in shell.faces() {
        let face = topo.face(fid)?;
        let periodic = match face.surface() {
            remus_topology::face::FaceSurface::Plane { .. } => false,
            remus_topology::face::FaceSurface::Nurbs(surface) => {
                surface.is_periodic_u() || surface.is_periodic_v()
            }
            remus_topology::face::FaceSurface::Cylinder(_)
            | remus_topology::face::FaceSurface::Cone(_)
            | remus_topology::face::FaceSurface::Sphere(_)
            | remus_topology::face::FaceSurface::Torus(_) => true,
        };
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            wire_periodicity
                .entry(wid)
                .and_modify(|all_periodic| *all_periodic &= periodic)
                .or_insert(periodic);
        }
    }
    let mut checked_wires = HashSet::new();
    for &fid in shell.faces() {
        let face = topo.face(fid)?;
        let mut wire_ids = vec![face.outer_wire()];
        wire_ids.extend(face.inner_wires().iter().copied());
        for wid in wire_ids {
            if checked_wires.insert(wid) {
                if !options.disabled_checks.contains(&CheckId::WireEmpty) {
                    issues.extend(wire::check_wire_empty(topo, wid)?);
                }
                if !options.disabled_checks.contains(&CheckId::WireNotConnected) {
                    issues.extend(wire::check_wire_connected(topo, wid)?);
                }
                if !options.disabled_checks.contains(&CheckId::WireClosure3D) {
                    issues.extend(wire::check_wire_closure(topo, wid)?);
                }
                if !options
                    .disabled_checks
                    .contains(&CheckId::WireRedundantEdge)
                {
                    issues.extend(wire::check_wire_redundant(topo, wid)?);
                }
                if !options
                    .disabled_checks
                    .contains(&CheckId::WireSelfIntersection)
                {
                    let tolerance = options.tolerance_scale * 1e-6;
                    if wire_periodicity.get(&wid).copied().unwrap_or(false) {
                        issues.extend(wire::check_wire_self_intersection_on_periodic_surface(
                            topo, wid, tolerance,
                        )?);
                    } else {
                        issues.extend(wire::check_wire_self_intersection(topo, wid, tolerance)?);
                    }
                }
            }
        }
    }

    let mut checked_faces = HashSet::new();
    for &fid in shell.faces() {
        if checked_faces.insert(fid) {
            if !options.disabled_checks.contains(&CheckId::GeometryFinite) {
                issues.extend(finite::check_face_finite(topo, fid)?);
            }
            if !options.disabled_checks.contains(&CheckId::FaceNoSurface) {
                issues.extend(face::check_face_has_surface(topo, fid)?);
            }
            if !options
                .disabled_checks
                .contains(&CheckId::FaceOrientationConsistency)
            {
                issues.extend(face::check_face_orientation(topo, fid)?);
            }
        }
    }

    let mut checked_edges = HashSet::new();
    let mut checked_vertices = HashSet::new();
    for &fid in shell.faces() {
        let face = topo.face(fid)?;
        let mut wire_ids = vec![face.outer_wire()];
        wire_ids.extend(face.inner_wires().iter().copied());
        for wid in wire_ids {
            let wire_data = topo.wire(wid)?;
            for oe in wire_data.edges() {
                let eid = oe.edge();
                if checked_edges.insert(eid) {
                    if !options.disabled_checks.contains(&CheckId::GeometryFinite) {
                        let edge_data = topo.edge(eid)?;
                        for vid in [edge_data.start(), edge_data.end()] {
                            if checked_vertices.insert(vid) {
                                issues.extend(finite::check_vertex_finite(topo, vid)?);
                            }
                        }
                        issues.extend(finite::check_edge_finite(topo, eid)?);
                    }
                    if !options.disabled_checks.contains(&CheckId::EdgeRangeValid) {
                        issues.extend(edge::check_edge_range(
                            topo,
                            eid,
                            options.tolerance_scale * 1e-7,
                        )?);
                    }
                    if !options.disabled_checks.contains(&CheckId::EdgeDegenerate) {
                        issues.extend(edge::check_edge_degenerate(
                            topo,
                            eid,
                            options.tolerance_scale * 1e-7,
                        )?);
                    }
                    if !options.disabled_checks.contains(&CheckId::VertexOnCurve) {
                        let edge_data = topo.edge(eid)?;
                        issues.extend(vertex::check_vertex_on_curve(
                            topo,
                            edge_data.start(),
                            eid,
                            options.tolerance_scale * 1e-4,
                        )?);
                        if edge_data.start() != edge_data.end() {
                            issues.extend(vertex::check_vertex_on_curve(
                                topo,
                                edge_data.end(),
                                eid,
                                options.tolerance_scale * 1e-4,
                            )?);
                        }
                    }
                    if !options.disabled_checks.contains(&CheckId::VertexOnSurface) {
                        let edge_data = topo.edge(eid)?;
                        issues.extend(vertex::check_vertex_on_surface(
                            topo,
                            edge_data.start(),
                            fid,
                            options.tolerance_scale * 1e-4,
                        )?);
                        if edge_data.start() != edge_data.end() {
                            issues.extend(vertex::check_vertex_on_surface(
                                topo,
                                edge_data.end(),
                                fid,
                                options.tolerance_scale * 1e-4,
                            )?);
                        }
                    }
                }
            }
        }
    }

    // SameParameter: check edge's 3D curve vs PCurve on each adjacent face.
    if !options
        .disabled_checks
        .contains(&CheckId::EdgeSameParameter)
    {
        let mut sp_checked = HashSet::new();
        for &fid in shell.faces() {
            let face = topo.face(fid)?;
            let mut wire_ids = vec![face.outer_wire()];
            wire_ids.extend(face.inner_wires().iter().copied());
            for wid in wire_ids {
                let wire_data = topo.wire(wid)?;
                for oe in wire_data.edges() {
                    let eid = oe.edge();
                    let forward = oe.is_forward();
                    if sp_checked.insert((eid, fid, forward)) {
                        issues.extend(edge::check_edge_same_parameter(
                            topo,
                            eid,
                            fid,
                            forward,
                            options.tolerance_scale * 1e-4,
                        )?);
                    }
                }
            }
        }
    }

    Ok(issues)
}

fn validate_wire_checks(
    topo: &Topology,
    wire_id: WireId,
    options: &ValidateOptions,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let mut issues = Vec::new();
    if !options.disabled_checks.contains(&CheckId::WireEmpty) {
        issues.extend(wire::check_wire_empty(topo, wire_id)?);
    }
    if !options.disabled_checks.contains(&CheckId::WireNotConnected) {
        issues.extend(wire::check_wire_connected(topo, wire_id)?);
    }
    if !options.disabled_checks.contains(&CheckId::WireClosure3D) {
        issues.extend(wire::check_wire_closure(topo, wire_id)?);
    }
    if !options
        .disabled_checks
        .contains(&CheckId::WireRedundantEdge)
    {
        issues.extend(wire::check_wire_redundant(topo, wire_id)?);
    }
    if !options
        .disabled_checks
        .contains(&CheckId::WireSelfIntersection)
    {
        issues.extend(wire::check_wire_self_intersection(
            topo,
            wire_id,
            options.tolerance_scale * 1e-6,
        )?);
    }

    let mut edges = HashSet::new();
    let mut vertices = HashSet::new();
    for oriented in topo.wire(wire_id)?.edges() {
        let edge_id = oriented.edge();
        if !edges.insert(edge_id) {
            continue;
        }
        let edge_data = topo.edge(edge_id)?;
        if !options.disabled_checks.contains(&CheckId::GeometryFinite) {
            issues.extend(finite::check_edge_finite(topo, edge_id)?);
            for vertex_id in [edge_data.start(), edge_data.end()] {
                if vertices.insert(vertex_id) {
                    issues.extend(finite::check_vertex_finite(topo, vertex_id)?);
                }
            }
        }
        if !options.disabled_checks.contains(&CheckId::EdgeRangeValid) {
            issues.extend(edge::check_edge_range(
                topo,
                edge_id,
                options.tolerance_scale * 1e-7,
            )?);
        }
        if !options.disabled_checks.contains(&CheckId::EdgeDegenerate) {
            issues.extend(edge::check_edge_degenerate(
                topo,
                edge_id,
                options.tolerance_scale * 1e-7,
            )?);
        }
        if !options.disabled_checks.contains(&CheckId::VertexOnCurve) {
            issues.extend(vertex::check_vertex_on_curve(
                topo,
                edge_data.start(),
                edge_id,
                options.tolerance_scale * 1e-4,
            )?);
            if edge_data.start() != edge_data.end() {
                issues.extend(vertex::check_vertex_on_curve(
                    topo,
                    edge_data.end(),
                    edge_id,
                    options.tolerance_scale * 1e-4,
                )?);
            }
        }
    }
    Ok(issues)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use std::f64::consts::TAU;

    use remus_math::curves2d::{Curve2D, Line2D};
    use remus_math::vec::{Point2, Point3, Vec2, Vec3};
    use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
    use remus_topology::face::{Face, FaceId, FaceSurface};
    use remus_topology::pcurve::PCurve;
    use remus_topology::shell::{Shell, ShellId};
    use remus_topology::solid::Solid;
    use remus_topology::test_utils::{make_unit_cube_manifold, make_unit_square_face};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};
    use remus_topology::{BodyClass, Topology, TopologyError};

    use remus_math::surfaces::CylindricalSurface;

    use super::*;

    #[test]
    fn open_shell_flips_from_solid_error_to_sheet_warning() {
        let mut topo = Topology::new();
        let face = make_unit_square_face(&mut topo);
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        let options = ValidateOptions::default();

        let solid_profile = validate_shell(&topo, shell, &options).unwrap();
        assert!(solid_profile.issues.iter().any(|issue| {
            issue.check == CheckId::ShellClosed && issue.severity == Severity::Error
        }));

        topo.set_shell_body_class(shell, BodyClass::Sheet).unwrap();
        let sheet_profile = validate_sheet_body(&topo, shell, &options).unwrap();
        assert!(sheet_profile.is_valid(), "{:?}", sheet_profile.issues);
        let free_boundary = sheet_profile
            .issues
            .iter()
            .find(|issue| issue.check == CheckId::ShellFreeBoundary)
            .unwrap();
        assert_eq!(free_boundary.severity, Severity::Warning);
        assert_eq!(free_boundary.deviation, Some(4.0));
        assert!(
            sheet_profile
                .issues
                .iter()
                .all(|issue| issue.check != CheckId::ShellClosed)
        );
    }

    #[test]
    fn solid_validation_rejects_a_sheet_tagged_boundary() {
        let mut topo = Topology::new();
        let face = make_unit_square_face(&mut topo);
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        topo.set_shell_body_class(shell, BodyClass::Sheet).unwrap();
        let solid = topo.add_solid(Solid::new(shell, Vec::new()));

        let report = validate_solid(&topo, solid, &ValidateOptions::default()).unwrap();

        assert!(report.issues.iter().any(|issue| {
            issue.check == CheckId::BodyClassResolved && issue.severity == Severity::Error
        }));
        assert!(!report.is_valid());
    }

    #[test]
    fn valid_box_no_issues() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);
        let opts = ValidateOptions::default();
        let report = validate_solid(&topo, cube, &opts).unwrap();
        assert!(
            report.is_valid(),
            "unit cube should have no errors, got: {:?}",
            report.issues
        );
        assert_eq!(report.error_count(), 0);
    }

    #[test]
    fn edge_range_valid_for_box() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);
        let opts = ValidateOptions::default();
        let report = validate_solid(&topo, cube, &opts).unwrap();
        let range_issues: Vec<_> = report
            .issues
            .iter()
            .filter(|i| i.check == CheckId::EdgeRangeValid)
            .collect();
        assert!(
            range_issues.is_empty(),
            "box edges should have valid ranges, got: {range_issues:?}"
        );
    }

    #[test]
    fn valid_box_detailed() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);
        let opts = ValidateOptions::default();
        let report = validate_solid(&topo, cube, &opts).unwrap();
        assert_eq!(report.error_count(), 0, "errors: {:?}", report.issues);
        assert_eq!(report.warning_count(), 0, "warnings: {:?}", report.issues);
    }

    #[test]
    fn euler_characteristic_correct() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);
        let opts = ValidateOptions::default();
        let report = validate_solid(&topo, cube, &opts).unwrap();
        // Cube: V=8, E=12, F=6 → V-E+F = 2
        let euler_issues: Vec<_> = report
            .issues
            .iter()
            .filter(|i| i.check == CheckId::SolidEulerCharacteristic)
            .collect();
        assert!(
            euler_issues.is_empty(),
            "cube Euler characteristic should be 2, got issues: {euler_issues:?}"
        );
    }

    fn seam_pcurve(u: f64, forward: bool) -> PCurve {
        let (v0, dv) = if forward { (0.0, 1.0) } else { (1.0, -1.0) };
        PCurve::new(
            Curve2D::Line(Line2D::new(Point2::new(u, v0), Vec2::new(0.0, dv)).unwrap()),
            0.0,
            1.0,
        )
    }

    fn cylinder_seam_shell() -> (Topology, EdgeId, FaceId, ShellId) {
        let mut topo = Topology::new();
        let bottom = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let top = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 1.0), 1e-7));
        let seam = topo.add_edge(Edge::new(bottom, top, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(seam, true),
                    OrientedEdge::new(seam, false),
                ],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Cylinder(
                CylindricalSurface::with_ref_dir(
                    Point3::new(0.0, 0.0, 0.0),
                    Vec3::new(0.0, 0.0, 1.0),
                    1.0,
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .unwrap(),
            ),
        ));
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        (topo, seam, face, shell)
    }

    #[test]
    fn shell_validation_checks_both_seam_branches_by_orientation() {
        let (mut topo, seam, face, shell) = cylinder_seam_shell();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.0, true))
            .unwrap();
        topo.set_pcurve_oriented(seam, face, false, seam_pcurve(TAU, false))
            .unwrap();

        let options = ValidateOptions::default();
        let report = validate_shell(&topo, shell, &options).unwrap();
        assert!(
            report
                .issues
                .iter()
                .all(|issue| issue.check != CheckId::EdgeSameParameter),
            "both exact seam branches must pass: {:?}",
            report.issues
        );

        topo.set_pcurve_oriented(seam, face, false, seam_pcurve(TAU + 0.2, false))
            .unwrap();
        let report = validate_shell(&topo, shell, &options).unwrap();
        let seam_issues: Vec<_> = report
            .issues
            .iter()
            .filter(|issue| issue.check == CheckId::EdgeSameParameter)
            .collect();
        assert_eq!(
            seam_issues.len(),
            2,
            "SameParameter and SameRange must both fail"
        );
        assert!(
            seam_issues
                .iter()
                .all(|issue| issue.description.contains("reversed"))
        );

        assert!(
            edge::check_edge_same_parameter(&topo, seam, face, true, 1e-7)
                .unwrap()
                .is_empty()
        );
        assert_eq!(
            edge::check_edge_same_parameter(&topo, seam, face, false, 1e-7)
                .unwrap()
                .len(),
            2
        );
        let mut disabled = ValidateOptions::default();
        disabled.disabled_checks.insert(CheckId::EdgeSameParameter);
        let report = validate_shell(&topo, shell, &disabled).unwrap();
        assert!(
            report
                .issues
                .iter()
                .all(|issue| issue.check != CheckId::EdgeSameParameter)
        );
        assert!(matches!(
            topo.pcurve(seam, face),
            Err(TopologyError::SeamPcurveAmbiguous { .. })
        ));
    }
}
