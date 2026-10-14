//! Built-in pipeline operators.
//!
//! These wrap the analysis/fix/upgrade functions as [`super::operator::HealOperator`]
//! implementations for use in [`super::process::HealProcess`] pipelines.

use remus_topology::Topology;
use remus_topology::solid::SolidId;

use super::operator::HealOperator;
use super::registry::OperatorRegistry;
use crate::HealError;
use crate::context::HealContext;
use crate::fix::FixResult;
use crate::fix::config::FixConfig;

/// Register all built-in operators into a registry.
pub fn register_builtins(registry: &mut OperatorRegistry) {
    registry.register("fix_shape", Box::new(FixShapeOp));
    registry.register("unify_same_domain", Box::new(UnifySameDomainOp));
    registry.register("direct_faces", Box::new(DirectFacesOp));
    registry.register("same_parameter", Box::new(SameParameterOp));
    registry.register("merge_vertices", Box::new(MergeVerticesOp));
    registry.register("drop_small_edges", Box::new(DropSmallEdgesOp));
    registry.register("drop_small_faces", Box::new(DropSmallFacesOp));
    registry.register("remove_internal_wires", Box::new(RemoveInternalWiresOp));
    registry.register("sew_shells", Box::new(SewShellsOp));
    registry.register("split_common_vertex", Box::new(SplitCommonVertexOp));
    registry.register("convert_to_bspline", Box::new(ConvertToBSplineOp));
    registry.register("convert_to_elementary", Box::new(ConvertToElementaryOp));
    registry.register("fix_wireframe", Box::new(FixWireframeOp));
}

/// The outer shell followed by every cavity (inner) shell of `solid_id`.
///
/// Shell-scoped repairs (orientation, sewing) must run once per shell: a
/// cavity is its own closed 2-manifold, so walking only the outer shell
/// silently leaves cavity defects in place.
fn solid_shells(
    topo: &Topology,
    solid_id: SolidId,
) -> Result<Vec<remus_topology::shell::ShellId>, HealError> {
    let solid = topo.solid(solid_id)?;
    Ok(std::iter::once(solid.outer_shell())
        .chain(solid.inner_shells().iter().copied())
        .collect())
}

/// Full recursive shape fix (solid → shell → face → wire → edge).
#[derive(Debug)]
struct FixShapeOp;

impl HealOperator for FixShapeOp {
    fn name(&self) -> &'static str {
        "fix_shape"
    }

    fn execute(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        _ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult), HealError> {
        let config = FixConfig::default();
        let (new_solid, result) = crate::fix::fix_shape(topo, solid_id, &config)?;
        Ok((new_solid, result))
    }

    fn execute_with_history(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        _ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult, crate::reshape::ReShape), HealError> {
        crate::fix::fix_shape_with_history(topo, solid_id, &FixConfig::default(), None)
    }
}

/// Merge adjacent faces sharing the same surface.
#[derive(Debug)]
struct UnifySameDomainOp;

impl HealOperator for UnifySameDomainOp {
    fn name(&self) -> &'static str {
        "unify_same_domain"
    }

    fn execute(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        _ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult), HealError> {
        let options = crate::upgrade::unify_same_domain::UnifyOptions::default();
        let (new_solid, unify) =
            crate::upgrade::unify_same_domain::unify_same_domain(topo, solid_id, &options)?;
        let mut result = FixResult::changed(
            crate::status::Status::DONE1,
            crate::fix::RepairActionKind::SameDomainFaceUnified,
            unify.faces_merged,
        );
        result.merge(&FixResult::changed(
            crate::status::Status::DONE1,
            crate::fix::RepairActionKind::SameDomainEdgeUnified,
            unify.edges_merged,
        ));
        Ok((new_solid, result))
    }

    fn execute_with_history(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        _ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult, crate::reshape::ReShape), HealError> {
        let (solid, unify, history) =
            crate::upgrade::unify_same_domain::unify_same_domain_with_history(
                topo,
                solid_id,
                &crate::upgrade::unify_same_domain::UnifyOptions::default(),
            )?;
        let mut replacements = crate::reshape::ReShape::new();
        for (sources, targets) in history.face_groups {
            // A disconnected group needs per-region contributors before it can
            // assert which output inherited each source face.
            if let [target] = targets.as_slice() {
                for source in sources {
                    replacements.replace_face(source, *target);
                }
            }
        }
        for (source, targets) in history.face_regions {
            replacements.split_face(source, targets);
        }
        for (sources, target) in history.edge_runs {
            for source in sources {
                replacements.replace_edge(source, target);
            }
        }
        for edge in history.removed_edges {
            replacements.remove_edge(edge);
        }
        for vertex in history.removed_vertices {
            replacements.remove_vertex(vertex);
        }
        let mut report = FixResult::changed(
            crate::status::Status::DONE1,
            crate::fix::RepairActionKind::SameDomainFaceUnified,
            unify.faces_merged,
        );
        report.merge(&FixResult::changed(
            crate::status::Status::DONE1,
            crate::fix::RepairActionKind::SameDomainEdgeUnified,
            unify.edges_merged,
        ));
        Ok((solid, report, replacements))
    }
}

/// Orient all faces so normals point outward.
#[derive(Debug)]
struct DirectFacesOp;

impl HealOperator for DirectFacesOp {
    fn name(&self) -> &'static str {
        "direct_faces"
    }

    fn execute(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult), HealError> {
        let config = FixConfig {
            fix_orientation: crate::fix::FixMode::On,
            ..Default::default()
        };
        let mut result = FixResult::ok();
        for shell_id in solid_shells(topo, solid_id)? {
            result.merge(&crate::fix::shell::fix_shell(topo, shell_id, ctx, &config)?);
        }
        Ok((solid_id, result))
    }
}

/// Fix PCurve/3D curve consistency for all edges.
#[derive(Debug)]
struct SameParameterOp;

impl HealOperator for SameParameterOp {
    fn name(&self) -> &'static str {
        "same_parameter"
    }

    fn execute(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult), HealError> {
        let config = FixConfig {
            fix_same_parameter: crate::fix::FixMode::On,
            ..Default::default()
        };
        let face_ids = remus_topology::explorer::solid_faces(topo, solid_id)?;

        // Call the FACE-AWARE PCurve fixer for each (edge, face) pair
        // rather than `fix_edge`. The latter dispatches to
        // `fix_same_parameter_stub` (no face context), which reports
        // DONE3 with actions_taken=0 — misleading status flags would
        // otherwise propagate up despite no actual repair occurring.
        let mut aggregate = FixResult::ok();
        for &fid in &face_ids {
            let face = topo.face(fid)?;
            let mut edge_ids = Vec::new();
            for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                edge_ids.extend(
                    topo.wire(wid)?
                        .edges()
                        .iter()
                        .map(remus_topology::wire::OrientedEdge::edge),
                );
            }
            for eid in edge_ids {
                let r = crate::fix::edge::fix_same_parameter_on_face(topo, eid, fid, ctx, &config)?;
                aggregate.merge(&r);
            }
        }

        Ok((solid_id, aggregate))
    }
}

/// Merge coincident vertices across the solid.
#[derive(Debug)]
struct MergeVerticesOp;

impl HealOperator for MergeVerticesOp {
    fn name(&self) -> &'static str {
        "merge_vertices"
    }

    fn execute(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult), HealError> {
        let config = FixConfig {
            fix_coincident_vertices: crate::fix::FixMode::On,
            ..Default::default()
        };
        let result = crate::fix::solid::fix_solid(topo, solid_id, ctx, &config)?;
        let new_solid = ctx.reshape.apply(topo, solid_id)?;
        Ok((new_solid, result))
    }
}

/// Remove edges shorter than tolerance.
#[derive(Debug)]
struct DropSmallEdgesOp;

impl HealOperator for DropSmallEdgesOp {
    fn name(&self) -> &'static str {
        "drop_small_edges"
    }

    fn execute(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult), HealError> {
        let config = FixConfig {
            fix_small_edges: crate::fix::FixMode::On,
            ..Default::default()
        };
        let result = crate::fix::solid::fix_solid(topo, solid_id, ctx, &config)?;
        let new_solid = ctx.reshape.apply(topo, solid_id)?;
        Ok((new_solid, result))
    }
}

/// Remove faces with area below tolerance.
#[derive(Debug)]
struct DropSmallFacesOp;

impl HealOperator for DropSmallFacesOp {
    fn name(&self) -> &'static str {
        "drop_small_faces"
    }

    fn execute(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult), HealError> {
        let config = FixConfig {
            fix_small_faces: crate::fix::FixMode::On,
            ..Default::default()
        };
        let result = crate::fix::small_face::fix_small_faces(topo, solid_id, ctx, &config)?;
        let new_solid = ctx.reshape.apply(topo, solid_id)?;
        Ok((new_solid, result))
    }
}

/// Drop internal (hole) wires from all faces.
#[derive(Debug)]
struct RemoveInternalWiresOp;

impl HealOperator for RemoveInternalWiresOp {
    fn name(&self) -> &'static str {
        "remove_internal_wires"
    }

    fn execute(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        _ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult), HealError> {
        let removed = crate::upgrade::remove_internal_wires::remove_internal_wires(topo, solid_id)?;
        Ok((
            solid_id,
            FixResult::changed(
                crate::status::Status::DONE1,
                crate::fix::RepairActionKind::InternalWireRemoved,
                removed,
            ),
        ))
    }
    fn execute_with_history(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        _ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult, crate::reshape::ReShape), HealError> {
        let (removed, history) =
            crate::upgrade::remove_internal_wires::remove_internal_wires_with_history(
                topo, solid_id,
            )?;
        let mut replacements = crate::reshape::ReShape::new();
        for edge in history.edges {
            replacements.remove_edge(edge);
        }
        for vertex in history.vertices {
            replacements.remove_vertex(vertex);
        }
        Ok((
            solid_id,
            FixResult::changed(
                crate::status::Status::DONE1,
                crate::fix::RepairActionKind::InternalWireRemoved,
                removed,
            ),
            replacements,
        ))
    }
}

/// Sew free boundaries in shells.
#[derive(Debug)]
struct SewShellsOp;

impl HealOperator for SewShellsOp {
    fn name(&self) -> &'static str {
        "sew_shells"
    }

    fn execute(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult), HealError> {
        let mut report = crate::upgrade::shell_sewing::SewReport::default();
        for shell_id in solid_shells(topo, solid_id)? {
            let shell_report = crate::upgrade::shell_sewing::sew_shell_report(
                topo,
                shell_id,
                ctx.tolerance.linear,
            )?;
            report.sewn += shell_report.sewn;
            report.declined += shell_report.declined;
        }

        Ok((solid_id, Self::report_result(report, ctx)))
    }

    fn execute_with_history(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult, crate::reshape::ReShape), HealError> {
        use remus_topology::explorer::{solid_edges, solid_vertices};
        let mut report = crate::upgrade::shell_sewing::SewReport::default();
        let mut histories = Vec::new();
        for shell in solid_shells(topo, solid_id)? {
            let (shell_report, history) = crate::upgrade::shell_sewing::sew_shell_with_history(
                topo,
                shell,
                ctx.tolerance.linear,
            )?;
            report.sewn += shell_report.sewn;
            report.declined += shell_report.declined;
            histories.push(history);
        }
        let live_edges: std::collections::BTreeSet<_> =
            solid_edges(topo, solid_id)?.into_iter().collect();
        let live_vertices: std::collections::BTreeSet<_> =
            solid_vertices(topo, solid_id)?.into_iter().collect();
        let mut replacements = crate::reshape::ReShape::new();
        // A per-shell redirect does not consume a source still used by another shell.
        for history in histories {
            for (source, target) in history.edges {
                if !live_edges.contains(&source) && live_edges.contains(&target) {
                    replacements.replace_edge(source, target);
                }
            }
            for (source, target) in history.vertices {
                if !live_vertices.contains(&source) && live_vertices.contains(&target) {
                    replacements.replace_vertex(source, target);
                }
            }
        }
        Ok((solid_id, Self::report_result(report, ctx), replacements))
    }
}

impl SewShellsOp {
    fn report_result(
        report: crate::upgrade::shell_sewing::SewReport,
        ctx: &mut HealContext,
    ) -> FixResult {
        let mut result = FixResult::changed(
            crate::status::Status::DONE1,
            crate::fix::RepairActionKind::FreeEdgePairSewn,
            report.sewn,
        );
        if report.declined > 0 {
            // Coincident free edges the pass refused to merge: the shell is
            // still open there, and saying so beats reporting success.
            ctx.send_message(
                crate::context::MessageSeverity::Warning,
                format!(
                    "sew_shells: declined {} coincident free-edge pair(s) whose curves disagree \
                     or whose partner was ambiguous",
                    report.declined
                ),
                crate::status::Status::FAIL1,
            );
            result.merge(&FixResult::refused(
                crate::status::Status::FAIL1,
                crate::fix::RepairRefusalKind::FreeEdgesRemain,
                report.declined,
            ));
        }
        result
    }
}

/// Split vertices shared by too many non-adjacent edges.
#[derive(Debug)]
struct SplitCommonVertexOp;

impl HealOperator for SplitCommonVertexOp {
    fn name(&self) -> &'static str {
        "split_common_vertex"
    }

    fn execute(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult), HealError> {
        let config = FixConfig {
            fix_split_common_vertex: crate::fix::FixMode::On,
            ..Default::default()
        };
        let result =
            crate::fix::split_vertex::fix_split_common_vertex(topo, solid_id, ctx, &config)?;
        Ok((solid_id, result))
    }

    fn execute_with_history(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult, crate::reshape::ReShape), HealError> {
        let config = FixConfig {
            fix_split_common_vertex: crate::fix::FixMode::On,
            ..Default::default()
        };
        let (result, history) = crate::fix::split_vertex::fix_split_common_vertex_with_history(
            topo, solid_id, ctx, &config,
        )?;
        let mut replacements = crate::reshape::ReShape::new();
        for (source, targets) in history.vertices {
            replacements.record_applied_vertex_split(source, targets)?;
        }
        Ok((solid_id, result, replacements))
    }
}

/// Convert all geometry to B-Spline representation.
#[derive(Debug)]
struct ConvertToBSplineOp;

impl HealOperator for ConvertToBSplineOp {
    fn name(&self) -> &'static str {
        "convert_to_bspline"
    }

    fn execute(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        _ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult), HealError> {
        let converted =
            crate::custom::convert_to_bspline::convert_solid_to_bspline(topo, solid_id)?;
        Ok((
            solid_id,
            FixResult::changed(
                crate::status::Status::DONE1,
                crate::fix::RepairActionKind::GeometryConvertedToBspline,
                converted,
            ),
        ))
    }
}

/// Recognize and replace NURBS surfaces AND curves with their elementary
/// analytic forms. Runs surface recognition (face NURBS → analytic
/// surface) and curve recognition (edge NURBS → analytic curve) in
/// sequence; reports the combined count.
#[derive(Debug)]
struct ConvertToElementaryOp;

impl HealOperator for ConvertToElementaryOp {
    fn name(&self) -> &'static str {
        "convert_to_elementary"
    }

    fn execute(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult), HealError> {
        let surfaces = crate::custom::convert_to_elementary::convert_to_elementary(
            topo,
            solid_id,
            &ctx.tolerance,
        )?;
        let edges = crate::custom::convert_to_elementary::convert_edges_to_elementary(
            topo,
            solid_id,
            &ctx.tolerance,
        )?;
        let mut result = FixResult::changed(
            crate::status::Status::DONE1,
            crate::fix::RepairActionKind::SurfaceConvertedToElementary,
            surfaces,
        );
        result.merge(&FixResult::changed(
            crate::status::Status::DONE1,
            crate::fix::RepairActionKind::CurveConvertedToElementary,
            edges,
        ));
        Ok((solid_id, result))
    }
}

/// Repair missing or misaligned edges in shells.
#[derive(Debug)]
struct FixWireframeOp;

impl HealOperator for FixWireframeOp {
    fn name(&self) -> &'static str {
        "fix_wireframe"
    }

    fn execute(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult), HealError> {
        let (solid, report, _) = self.execute_with_history(topo, solid_id, ctx)?;
        Ok((solid, report))
    }

    fn execute_with_history(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
        ctx: &mut HealContext,
    ) -> Result<(SolidId, FixResult, crate::reshape::ReShape), HealError> {
        use remus_topology::explorer::{solid_edges, solid_vertices};
        let config = FixConfig {
            fix_wireframe: crate::fix::FixMode::On,
            ..Default::default()
        };
        let mut result = FixResult::ok();
        let mut histories = Vec::new();
        for shell in solid_shells(topo, solid_id)? {
            let (report, history) =
                crate::fix::wireframe::fix_wireframe_with_history(topo, shell, ctx, &config)?;
            result.merge(&report);
            histories.push(history);
        }
        let live_edges: std::collections::BTreeSet<_> =
            solid_edges(topo, solid_id)?.into_iter().collect();
        let live_vertices: std::collections::BTreeSet<_> =
            solid_vertices(topo, solid_id)?.into_iter().collect();
        let mut replacements = crate::reshape::ReShape::new();
        for history in histories {
            // A shell-local redirect does not consume identities retained by another shell.
            for (source, target) in history.edges {
                if !live_edges.contains(&source) {
                    replacements.replace_edge(source, target);
                }
            }
            for (source, target) in history.vertices {
                if !live_vertices.contains(&source) {
                    replacements.replace_vertex(source, target);
                }
            }
        }
        Ok((solid_id, result, replacements))
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use remus_math::vec::{Point3, Vec3};
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::shell::{Shell, ShellId};
    use remus_topology::solid::Solid;
    use remus_topology::test_utils::make_unit_cube_manifold_at;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    use super::*;
    use crate::pipeline::process::HealProcess;

    /// A unit cube solid whose cavity is a second manifold unit cube shell.
    /// Only topology matters to these operators, so the cavity is placed
    /// apart from the outer cube rather than nested inside it.
    fn cube_with_cavity(topo: &mut Topology) -> (SolidId, ShellId, ShellId) {
        let outer_solid = make_unit_cube_manifold_at(topo, 0.0, 0.0, 0.0);
        let cavity_solid = make_unit_cube_manifold_at(topo, 5.0, 5.0, 5.0);
        let outer = topo.solid(outer_solid).unwrap().outer_shell();
        let cavity = topo.solid(cavity_solid).unwrap().outer_shell();
        let solid = topo.add_solid(Solid::new(outer, vec![cavity]));
        (solid, outer, cavity)
    }

    fn run_step(topo: &mut Topology, solid: SolidId, step: &str) -> FixResult {
        let mut process = HealProcess::new();
        process.add_step(step);
        let (_, mut results) = process.execute(topo, solid).unwrap();
        results.remove(0)
    }

    #[test]
    fn direct_faces_orients_cavity_shells() {
        let mut topo = Topology::new();
        let (solid, outer, cavity) = cube_with_cavity(&mut topo);
        // One cavity face traverses its four edges the same way as its
        // neighbours.
        let flipped = topo.shell(cavity).unwrap().faces()[2];
        let outer_wire = topo.face(flipped).unwrap().outer_wire();
        let reversed: Vec<_> = topo
            .wire(outer_wire)
            .unwrap()
            .edges()
            .iter()
            .rev()
            .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
            .collect();
        let reversed = topo.add_wire(Wire::new(reversed, true).unwrap());
        topo.set_face_boundary_wires(flipped, reversed, vec![])
            .unwrap();
        assert_eq!(misoriented_edges(&topo, cavity), 4);

        let result = run_step(&mut topo, solid, "direct_faces");

        assert!(result.actions_taken > 0, "{result:?}");
        assert_eq!(misoriented_edges(&topo, outer), 0);
        assert_eq!(misoriented_edges(&topo, cavity), 0);
    }

    /// Edges two faces of `shell` traverse in the same effective direction
    /// (wire sense composed with the face's reversal flag).
    fn misoriented_edges(topo: &Topology, shell: ShellId) -> usize {
        let mut uses: std::collections::BTreeMap<usize, Vec<bool>> =
            std::collections::BTreeMap::new();
        for &fid in topo.shell(shell).unwrap().faces() {
            let face = topo.face(fid).unwrap();
            for oe in topo.wire(face.outer_wire()).unwrap().edges() {
                uses.entry(oe.edge().index())
                    .or_default()
                    .push(oe.is_forward() != face.is_reversed());
            }
        }
        uses.values()
            .filter(|senses| senses.len() == 2 && senses[0] == senses[1])
            .count()
    }

    #[test]
    fn same_parameter_covers_cavity_shells_and_hole_loops() {
        let mut topo = Topology::new();
        let (solid, outer, cavity) = cube_with_cavity(&mut topo);
        // A hole loop on the outer cube's top face (z = 1).
        let p = |x, y| Point3::new(x, y, 1.0);
        let hole_vertices: Vec<_> = [p(0.4, 0.4), p(0.4, 0.6), p(0.6, 0.6), p(0.6, 0.4)]
            .into_iter()
            .map(|point| topo.add_vertex(Vertex::new(point, 1e-7)))
            .collect();
        let hole_edges: Vec<_> = (0..4)
            .map(|i| {
                let edge = Edge::new(
                    hole_vertices[i],
                    hole_vertices[(i + 1) % 4],
                    EdgeCurve::Line,
                );
                OrientedEdge::new(topo.add_edge(edge), true)
            })
            .collect();
        let hole = topo.add_wire(Wire::new(hole_edges.clone(), true).unwrap());
        let top = topo.shell(outer).unwrap().faces()[1];
        let top_outer = topo.face(top).unwrap().outer_wire();
        topo.set_face_boundary_wires(top, top_outer, vec![hole])
            .unwrap();

        let result = run_step(&mut topo, solid, "same_parameter");

        // Cube faces use each of their 4 edges once; the hole adds 4 more.
        assert_eq!(result.actions_taken, 2 * 6 * 4 + 4, "{result:?}");
        for &face in topo.shell(cavity).unwrap().faces() {
            let wire = topo.wire(topo.face(face).unwrap().outer_wire()).unwrap();
            for oe in wire.edges() {
                assert!(
                    topo.pcurve_oriented(oe.edge(), face, oe.is_forward())
                        .is_some(),
                    "cavity face {face:?} edge {:?} has no pcurve",
                    oe.edge()
                );
            }
        }
        for oe in hole_edges {
            assert!(topo.pcurve_oriented(oe.edge(), top, true).is_some());
        }
    }

    #[test]
    fn sew_shells_sews_cavity_shells() {
        let mut topo = Topology::new();
        let outer_solid = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);
        let outer = topo.solid(outer_solid).unwrap().outer_shell();
        // A cavity cube whose six faces share no edges: 24 free edges.
        let corner = |i: usize| {
            let bit = |b: usize| if i & b == 0 { 5.0 } else { 6.0 };
            Point3::new(bit(1), bit(2), bit(4))
        };
        let quads = [
            ([0, 2, 3, 1], Vec3::new(0.0, 0.0, -1.0), -5.0),
            ([4, 5, 7, 6], Vec3::new(0.0, 0.0, 1.0), 6.0),
            ([0, 1, 5, 4], Vec3::new(0.0, -1.0, 0.0), -5.0),
            ([2, 6, 7, 3], Vec3::new(0.0, 1.0, 0.0), 6.0),
            ([0, 4, 6, 2], Vec3::new(-1.0, 0.0, 0.0), -5.0),
            ([1, 3, 7, 5], Vec3::new(1.0, 0.0, 0.0), 6.0),
        ];
        let faces: Vec<_> = quads
            .into_iter()
            .map(|(corners, normal, d)| {
                let vertices: Vec<_> = corners
                    .iter()
                    .map(|&c| topo.add_vertex(Vertex::new(corner(c), 1e-7)))
                    .collect();
                let edges = (0..4)
                    .map(|i| {
                        let edge = Edge::new(vertices[i], vertices[(i + 1) % 4], EdgeCurve::Line);
                        OrientedEdge::new(topo.add_edge(edge), true)
                    })
                    .collect();
                let wire = topo.add_wire(Wire::new(edges, true).unwrap());
                topo.add_face(Face::new(wire, vec![], FaceSurface::Plane { normal, d }))
            })
            .collect();
        let cavity = topo.add_shell(Shell::new(faces).unwrap());
        let solid = topo.add_solid(Solid::new(outer, vec![cavity]));
        let free = |topo: &Topology| -> usize {
            crate::analysis::free_bounds::find_free_bounds(topo, cavity)
                .unwrap()
                .iter()
                .map(Vec::len)
                .sum()
        };
        assert_eq!(free(&topo), 24);

        let result = run_step(&mut topo, solid, "sew_shells");

        assert_eq!(result.actions_taken, 12, "{result:?}");
        assert_eq!(free(&topo), 0, "the cavity shell must be sewn closed");
    }
}
