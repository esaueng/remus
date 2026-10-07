//! Diagnostic estimates include allocated/retired slots and shared NURBS nets.
//! They exclude allocator overhead, evaluation caches, journals, attributes,
//! pcurves and undo logs.
//! This is a checkpoint admission estimate, not a process-memory bound.

use std::collections::HashSet;

use super::Topology;
use crate::edge::EdgeCurve;
use crate::face::FaceSurface;

/// Accounted topology storage. All byte values are estimates, not RSS.
#[derive(Debug, Clone, Copy, Default)]
pub struct MemoryEstimate {
    /// Arena vector capacities, including permanently retired slots.
    pub arena_bytes: usize,
    /// Entity adjacency lists, measured at their current lengths.
    pub entity_list_bytes: usize,
    /// Unique immutable NURBS allocations, including row capacity slack.
    pub nurbs_bytes: usize,
    /// Allocated entity slots, including retired slots.
    pub allocated_slots: usize,
    /// Slots retained solely to prevent handle reuse.
    pub retired_slots: usize,
}

impl MemoryEstimate {
    /// Sum of the accounted categories.
    #[must_use]
    pub const fn bytes(&self) -> usize {
        self.arena_bytes
            .saturating_add(self.entity_list_bytes)
            .saturating_add(self.nurbs_bytes)
    }
}

impl Topology {
    /// Estimate storage while deduplicating NURBS allocations across snapshots.
    /// `shared` contains process-local allocation identities only; never persist
    /// it or use it as a geometry-validity key. Retired payloads remain counted.
    #[must_use]
    pub fn memory_estimate(&self, shared: &mut HashSet<usize>) -> MemoryEstimate {
        let mut out = MemoryEstimate::default();
        macro_rules! arena {
            ($field:ident) => {
                out.arena_bytes = out.arena_bytes.saturating_add(self.$field.storage_bytes());
                out.allocated_slots += self.$field.slot_len();
                out.retired_slots += self.$field.slot_len() - self.$field.len();
            };
        }
        arena!(vertices);
        arena!(edges);
        arena!(wires);
        arena!(faces);
        arena!(shells);
        arena!(solids);
        arena!(compounds);
        arena!(compsolids);
        arena!(loops);
        arena!(coedges);
        let mut visit = |id, bytes| {
            if shared.insert(id) {
                out.nurbs_bytes = out.nurbs_bytes.saturating_add(bytes);
            }
        };
        for face in self.faces.retained_items() {
            if let FaceSurface::Nurbs(surface) = face.surface() {
                surface.visit_shared_storage(&mut visit);
            }
            out.entity_list_bytes += std::mem::size_of_val(face.inner_wires());
            out.entity_list_bytes += std::mem::size_of_val(face.boundary_loops());
        }
        for edge in self.edges.retained_items() {
            if let EdgeCurve::NurbsCurve(curve) = edge.curve() {
                curve.visit_shared_storage(&mut visit);
            }
        }
        for wire in self.wires.retained_items() {
            out.entity_list_bytes += std::mem::size_of_val(wire.edges());
        }
        for shell in self.shells.retained_items() {
            out.entity_list_bytes += std::mem::size_of_val(shell.faces());
        }
        for solid in self.solids.retained_items() {
            out.entity_list_bytes += std::mem::size_of_val(solid.inner_shells());
        }
        for compound in self.compounds.retained_items() {
            out.entity_list_bytes += std::mem::size_of_val(compound.solids());
        }
        for compsolid in self.compsolids.retained_items() {
            out.entity_list_bytes += std::mem::size_of_val(compsolid.solids());
        }
        out
    }
}
