//! Exact solid-face imprint with construction-derived journal history.

use remus_algo::gfa::{EdgeEvent, EntityEvolution};
use remus_topology::Topology;
use remus_topology::journal::{EventDraft, OpId};
use remus_topology::solid::SolidId;

use crate::OperationsError;
use crate::journal_ops::{begin_scoped, draft_from_entity_evolution, solid_entity_keys};

/// A completed imprint and the journal entry that records its exact lineage.
#[derive(Debug, Clone)]
pub struct ImprintResult {
    /// New solid with every target patch retained.
    pub solid: SolidId,
    /// Construction-derived `imprint` journal entry.
    pub op: OpId,
    /// Total result-entity lineage used to build the entry.
    pub evolution: EntityEvolution,
}

/// Imprint the tool solid's intersection edges onto the target solid's faces.
///
/// The operation creates a new target solid, retains every target face patch,
/// and records the split as one construction-derived journal entry. The tool
/// is not modified; its faces, edges, and vertices are explicitly preserved.
/// The bounded qualified subset requires at least one transversal face split,
/// no same-domain face overlap, and total result edge lineage.
///
/// # Errors
///
/// Returns a typed unsupported error outside the qualified subset. An invalid
/// result or malformed journal entry rolls back every allocation and journal
/// mutation.
pub fn imprint(
    topo: &mut Topology,
    target: SolidId,
    tool: SolidId,
) -> Result<ImprintResult, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        let target_entities = solid_entity_keys(topo, target)?;
        let tool_entities = solid_entity_keys(topo, tool)?;
        let target_entity_set: std::collections::BTreeSet<_> =
            target_entities.iter().copied().collect();
        if tool_entities
            .iter()
            .any(|entity| target_entity_set.contains(entity))
        {
            return Err(remus_algo::error::AlgoError::UnsupportedImprint {
                reason: "target and tool must not share topology entities".into(),
            }
            .into());
        }
        let pending = begin_scoped(topo, "imprint", &[target, tool])?;
        let (solid, evolution) =
            remus_algo::gfa::imprint_with_entity_evolution(topo, target, tool)?;

        let report = remus_check::validate::validate_solid(
            topo,
            solid,
            &remus_check::validate::ValidateOptions::default(),
        )?;
        if !report.is_valid() {
            return Err(OperationsError::BodyValidationFailed {
                body_class: remus_topology::BodyClass::Solid.as_str(),
                error_count: report.error_count(),
            });
        }
        if evolution
            .edges
            .iter()
            .any(|(_, event)| matches!(event, EdgeEvent::Unresolved))
        {
            return Err(remus_algo::error::AlgoError::UnsupportedImprint {
                reason: "result edge lineage contains an unresolved event".into(),
            }
            .into());
        }

        let mut draft = draft_from_entity_evolution(&evolution);
        draft.add_scope(solid_entity_keys(topo, solid)?);
        for key in tool_entities {
            draft.push(key, EventDraft::Preserved { from: key });
        }
        let op = topo.journal_record_evolution(pending, draft)?;

        Ok(ImprintResult {
            solid,
            op,
            evolution,
        })
    })
}
