//! Wireframe repair using shared boundary edges and connected vertex identities.

use remus_topology::Topology;
use remus_topology::shell::ShellId;

use super::FixResult;
use super::config::FixConfig;
use crate::HealError;
use crate::context::HealContext;
use crate::status::Status;

/// Sew geometrically coincident free edges in a shell.
///
/// Candidates use the shell-sewing curve, ambiguity, and pcurve checks.
/// Unshared boundaries remaining after repair are reported as refusals.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups or boundary updates fail.
pub fn fix_wireframe(
    topo: &mut Topology,
    shell_id: ShellId,
    ctx: &mut HealContext,
    config: &FixConfig,
) -> Result<FixResult, HealError> {
    Ok(fix_wireframe_with_history(topo, shell_id, ctx, config)?.0)
}

/// Repair wireframe boundaries and retain the committed entity replacements.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups or boundary updates fail.
pub fn fix_wireframe_with_history(
    topo: &mut Topology,
    shell_id: ShellId,
    ctx: &mut HealContext,
    config: &FixConfig,
) -> Result<(FixResult, crate::upgrade::shell_sewing::SewHistory), HealError> {
    if config.fix_wireframe == super::config::FixMode::Off {
        return Ok((
            FixResult::ok(),
            crate::upgrade::shell_sewing::SewHistory::default(),
        ));
    }
    let (report, history) =
        crate::upgrade::shell_sewing::sew_shell_with_history(topo, shell_id, ctx.tolerance.linear)?;
    let remaining: usize = crate::analysis::free_bounds::find_free_bounds(topo, shell_id)?
        .iter()
        .map(Vec::len)
        .sum();
    if report.sewn > 0 {
        ctx.info(format!(
            "sewn {} free-edge pairs into shared boundaries",
            report.sewn
        ));
    }
    if remaining > 0 {
        ctx.warn(format!(
            "{remaining} free edges remain after sewing (may need manual repair)"
        ));
    }
    let mut result = FixResult::changed(
        Status::DONE4,
        super::RepairActionKind::FreeEdgePairSewn,
        report.sewn,
    );
    if remaining > 0 {
        result.merge(&FixResult::refused(
            Status::FAIL1,
            super::RepairRefusalKind::FreeEdgesRemain,
            remaining,
        ));
    }
    Ok((result, history))
}
