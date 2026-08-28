//! Shape rebuilding via entity replacement tracking.
//!
//! [`ReShape`] records vertex/edge/wire/face/shell replacements and
//! removals during fixing.  After all fixes are recorded, call
//! [`ReShape::apply`] to rebuild the solid with all substitutions
//! applied atomically.
//!
//! Modelled after industry-standard B-Rep reshape utilities.

use std::collections::{HashMap, HashSet};

use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::FaceId;
use remus_topology::shell::ShellId;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::VertexId;
use remus_topology::wire::{OrientedEdge, WireId};

use crate::HealError;

/// Action to perform on a vertex during reshape.
#[derive(Debug, Clone)]
pub enum VertexAction {
    /// Replace with another vertex.
    Replace(VertexId),
    /// Remove the vertex entirely.
    Remove,
}

/// Action to perform on an edge during reshape.
#[derive(Debug, Clone)]
pub enum EdgeAction {
    /// Replace with a single edge.
    Replace(EdgeId),
    /// Replace with multiple edges (split).
    Split(Vec<EdgeId>),
    /// Remove the edge entirely.
    Remove,
}

/// Action to perform on a wire during reshape.
#[derive(Debug, Clone)]
pub enum WireAction {
    /// Replace with another wire.
    Replace(WireId),
    /// Remove the wire entirely.
    Remove,
}

/// Action to perform on a face during reshape.
#[derive(Debug, Clone)]
pub enum FaceAction {
    /// Replace with another face.
    Replace(FaceId),
    /// Replace with multiple faces (split).
    Split(Vec<FaceId>),
    /// Remove the face entirely.
    Remove,
}

/// Action to perform on a shell during reshape.
#[derive(Debug, Clone)]
pub enum ShellAction {
    /// Replace with another shell.
    Replace(ShellId),
    /// Remove the shell entirely.
    Remove,
}

/// Tracks entity replacements and removals for atomic shape rebuilding.
#[derive(Debug, Default, Clone)]
pub struct ReShape {
    vertices: HashMap<VertexId, VertexAction>,
    edges: HashMap<EdgeId, EdgeAction>,
    wires: HashMap<WireId, WireAction>,
    faces: HashMap<FaceId, FaceAction>,
    shells: HashMap<ShellId, ShellAction>,
}

impl ReShape {
    /// Create an empty reshape tracker.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    // ── Vertex operations ───────────────────────────────────────────

    /// Record that `from` should be replaced by `to`.
    pub fn replace_vertex(&mut self, from: VertexId, to: VertexId) {
        self.vertices.insert(from, VertexAction::Replace(to));
    }

    /// Record that a vertex should be removed.
    pub fn remove_vertex(&mut self, id: VertexId) {
        self.vertices.insert(id, VertexAction::Remove);
    }

    /// Resolve a vertex through the replacement chain.
    #[must_use]
    pub fn resolve_vertex(&self, mut id: VertexId) -> VertexId {
        let mut depth = 0;
        while let Some(VertexAction::Replace(target)) = self.vertices.get(&id) {
            id = *target;
            depth += 1;
            if depth > 100 {
                break; // prevent infinite loops
            }
        }
        id
    }

    /// Check if a vertex is marked for removal.
    #[must_use]
    pub fn is_vertex_removed(&self, id: VertexId) -> bool {
        matches!(self.vertices.get(&id), Some(VertexAction::Remove))
    }

    // ── Edge operations ─────────────────────────────────────────────

    /// Record that `from` should be replaced by `to`.
    pub fn replace_edge(&mut self, from: EdgeId, to: EdgeId) {
        self.edges.insert(from, EdgeAction::Replace(to));
    }

    /// Record that an edge was split into multiple edges.
    pub fn split_edge(&mut self, from: EdgeId, into: Vec<EdgeId>) {
        self.edges.insert(from, EdgeAction::Split(into));
    }

    /// Record that an edge should be removed.
    pub fn remove_edge(&mut self, id: EdgeId) {
        self.edges.insert(id, EdgeAction::Remove);
    }

    /// Resolve an edge through the replacement chain.
    #[must_use]
    pub fn resolve_edge(&self, mut id: EdgeId) -> Option<EdgeId> {
        let mut depth = 0;
        loop {
            match self.edges.get(&id) {
                Some(EdgeAction::Replace(target)) => {
                    id = *target;
                    depth += 1;
                    if depth > 100 {
                        return Some(id);
                    }
                }
                Some(EdgeAction::Remove) => return None,
                Some(EdgeAction::Split(_)) | None => return Some(id),
            }
        }
    }

    /// Check if an edge is marked for removal.
    #[must_use]
    pub fn is_edge_removed(&self, id: EdgeId) -> bool {
        matches!(self.edges.get(&id), Some(EdgeAction::Remove))
    }

    /// Get edge action if any.
    #[must_use]
    pub fn edge_action(&self, id: EdgeId) -> Option<&EdgeAction> {
        self.edges.get(&id)
    }

    // ── Wire operations ─────────────────────────────────────────────

    /// Record that `from` should be replaced by `to`.
    pub fn replace_wire(&mut self, from: WireId, to: WireId) {
        self.wires.insert(from, WireAction::Replace(to));
    }

    /// Record that a wire should be removed.
    pub fn remove_wire(&mut self, id: WireId) {
        self.wires.insert(id, WireAction::Remove);
    }

    /// Resolve a wire through the replacement chain.
    fn resolve_wire(&self, mut id: WireId) -> Option<WireId> {
        let mut seen = HashSet::new();
        while seen.insert(id) {
            match self.wires.get(&id) {
                Some(WireAction::Replace(target)) => id = *target,
                Some(WireAction::Remove) => return None,
                None => return Some(id),
            }
        }
        Some(id)
    }

    // ── Face operations ─────────────────────────────────────────────

    /// Record that `from` should be replaced by `to`.
    pub fn replace_face(&mut self, from: FaceId, to: FaceId) {
        self.faces.insert(from, FaceAction::Replace(to));
    }

    /// Record that a face was split into multiple faces.
    pub fn split_face(&mut self, from: FaceId, into: Vec<FaceId>) {
        self.faces.insert(from, FaceAction::Split(into));
    }

    /// Record that a face should be removed.
    pub fn remove_face(&mut self, id: FaceId) {
        self.faces.insert(id, FaceAction::Remove);
    }

    /// Check if a face is marked for removal.
    #[must_use]
    pub fn is_face_removed(&self, id: FaceId) -> bool {
        matches!(self.faces.get(&id), Some(FaceAction::Remove))
    }

    /// Resolve a face through a chain of single-face replacements.
    fn resolve_face(&self, mut id: FaceId) -> Option<FaceId> {
        let mut seen = HashSet::new();
        while seen.insert(id) {
            match self.faces.get(&id) {
                Some(FaceAction::Replace(target)) => id = *target,
                Some(FaceAction::Remove) => return None,
                Some(FaceAction::Split(_)) | None => return Some(id),
            }
        }
        Some(id)
    }

    // ── Shell operations ────────────────────────────────────────────

    /// Record that `from` should be replaced by `to`.
    pub fn replace_shell(&mut self, from: ShellId, to: ShellId) {
        self.shells.insert(from, ShellAction::Replace(to));
    }

    /// Record that a shell should be removed.
    pub fn remove_shell(&mut self, id: ShellId) {
        self.shells.insert(id, ShellAction::Remove);
    }

    /// Resolve a shell through the replacement chain.
    fn resolve_shell(&self, mut id: ShellId) -> Option<ShellId> {
        let mut seen = HashSet::new();
        while seen.insert(id) {
            match self.shells.get(&id) {
                Some(ShellAction::Replace(target)) => id = *target,
                Some(ShellAction::Remove) => return None,
                None => return Some(id),
            }
        }
        Some(id)
    }

    /// Final shells referenced by the solid after shell substitutions.
    fn final_shell_ids(
        &self,
        topo: &Topology,
        solid_id: SolidId,
    ) -> Result<Vec<ShellId>, HealError> {
        let solid = topo.solid(solid_id)?;
        let mut shells = Vec::new();
        let mut seen = HashSet::new();
        if let Some(outer) = self.resolve_shell(solid.outer_shell())
            && seen.insert(outer)
        {
            shells.push(outer);
        }
        for &inner in solid.inner_shells() {
            if let Some(inner) = self.resolve_shell(inner)
                && seen.insert(inner)
            {
                shells.push(inner);
            }
        }
        Ok(shells)
    }

    /// Expand a face action to the final face IDs, including split targets.
    fn resolved_faces(&self, id: FaceId) -> Vec<FaceId> {
        let mut pending = vec![id];
        let mut resolved = Vec::new();
        let mut seen = HashSet::new();
        while let Some(id) = pending.pop() {
            let Some(id) = self.resolve_face(id) else {
                continue;
            };
            if !seen.insert(id) {
                continue;
            }
            match self.faces.get(&id) {
                Some(FaceAction::Split(targets)) => {
                    pending.extend(targets.iter().rev().copied());
                }
                Some(FaceAction::Remove) => {}
                Some(FaceAction::Replace(_)) | None => resolved.push(id),
            }
        }
        resolved
    }

    /// Final faces after shell and face substitutions.
    fn final_face_ids(&self, topo: &Topology, solid_id: SolidId) -> Result<Vec<FaceId>, HealError> {
        let mut faces = Vec::new();
        let mut seen = HashSet::new();
        for shell_id in self.final_shell_ids(topo, solid_id)? {
            for &face_id in topo.shell(shell_id)?.faces() {
                for resolved in self.resolved_faces(face_id) {
                    if seen.insert(resolved) {
                        faces.push(resolved);
                    }
                }
            }
        }
        Ok(faces)
    }

    /// Expand an edge action to the final edge IDs, including split targets.
    fn resolved_edges(&self, id: EdgeId) -> Vec<EdgeId> {
        let mut pending = vec![id];
        let mut resolved = Vec::new();
        let mut seen = HashSet::new();
        while let Some(id) = pending.pop() {
            let Some(id) = self.resolve_edge(id) else {
                continue;
            };
            if !seen.insert(id) {
                continue;
            }
            match self.edges.get(&id) {
                Some(EdgeAction::Split(targets)) => {
                    pending.extend(targets.iter().rev().copied());
                }
                Some(EdgeAction::Remove) => {}
                Some(EdgeAction::Replace(_)) | None => resolved.push(id),
            }
        }
        resolved
    }

    // ── Apply ───────────────────────────────────────────────────────

    /// Whether any replacements or removals have been recorded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
            && self.edges.is_empty()
            && self.wires.is_empty()
            && self.faces.is_empty()
            && self.shells.is_empty()
    }

    /// Apply all recorded replacements to a solid, rebuilding the shape tree.
    ///
    /// Returns the (possibly new) solid ID after all substitutions.
    ///
    /// # Errors
    ///
    /// Returns [`HealError`] if entity lookups fail during rebuilding.
    pub fn apply(&self, topo: &mut Topology, solid_id: SolidId) -> Result<SolidId, HealError> {
        if self.is_empty() {
            return Ok(solid_id);
        }

        // 1. Apply vertex replacements to all final edges.
        if !self.vertices.is_empty() {
            self.apply_vertex_replacements(topo, solid_id)?;
        }

        // 2. Substitute wires before rebuilding their edge lists.
        if !self.wires.is_empty() {
            self.apply_wire_replacements(topo, solid_id)?;
        }

        // 3. Rebuild wires (remove/split edges, rebuild edge lists).
        if !self.edges.is_empty() {
            self.apply_edge_replacements(topo, solid_id)?;
        }

        // 4. Rebuild each shell's face list.
        if !self.faces.is_empty() {
            self.apply_face_replacements(topo, solid_id)?;
        }

        // 5. Rebuild the solid's outer and inner shell references.
        self.apply_shell_replacements(topo, solid_id)?;

        Ok(solid_id)
    }

    /// Apply vertex replacements: update edge start/end vertices.
    fn apply_vertex_replacements(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
    ) -> Result<(), HealError> {
        let face_ids = self.final_face_ids(topo, solid_id)?;
        let mut edge_ids = Vec::new();
        for fid in face_ids {
            let face = topo.face(fid)?;
            let wire_ids =
                std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied());
            for wire_id in wire_ids {
                let Some(wire_id) = self.resolve_wire(wire_id) else {
                    continue;
                };
                for oe in topo.wire(wire_id)?.edges() {
                    edge_ids.extend(self.resolved_edges(oe.edge()));
                }
            }
        }
        edge_ids.sort_by_key(|e| e.index());
        edge_ids.dedup_by_key(|e| e.index());

        let mut updates = Vec::new();
        for eid in edge_ids {
            let edge = topo.edge(eid)?;
            let new_start = self.resolve_vertex(edge.start());
            let new_end = self.resolve_vertex(edge.end());
            if new_start != edge.start() || new_end != edge.end() {
                updates.push((eid, new_start, new_end));
            }
        }

        for (eid, new_start, new_end) in updates {
            let edge = topo.edge_mut(eid)?;
            // `set_start`/`set_end`, never a whole-`Edge` rebuild: an explicit
            // trim (RFC 0002, Stage 3) and an edge-specific tolerance are not
            // recoverable from the endpoints, and a vertex merge changes neither
            // the curve nor the parameter interval on it.
            edge.set_start(new_start);
            edge.set_end(new_end);
        }

        Ok(())
    }

    /// Apply wire replacements to every final face on every shell.
    fn apply_wire_replacements(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
    ) -> Result<(), HealError> {
        for face_id in self.final_face_ids(topo, solid_id)? {
            let face = topo.face(face_id)?;
            let old_outer = face.outer_wire();
            let old_inner = face.inner_wires().to_vec();
            let new_outer = self.resolve_wire(old_outer).ok_or_else(|| {
                HealError::FixFailed(format!(
                    "cannot remove outer wire {old_outer:?} from retained face {face_id:?}"
                ))
            })?;
            let new_inner: Vec<_> = old_inner
                .iter()
                .filter_map(|&wire| self.resolve_wire(wire))
                .collect();

            if new_outer != old_outer || new_inner != old_inner {
                let face = topo.face_mut(face_id)?;
                face.set_outer_wire(new_outer);
                *face.inner_wires_mut() = new_inner;
            }
        }
        Ok(())
    }

    /// Apply edge replacements: rebuild wires with new edge lists.
    fn apply_edge_replacements(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
    ) -> Result<(), HealError> {
        let face_ids = self.final_face_ids(topo, solid_id)?;
        for fid in face_ids {
            let face = topo.face(fid)?;
            let all_wires: Vec<_> = std::iter::once(face.outer_wire())
                .chain(face.inner_wires().iter().copied())
                .collect();

            for wire_id in all_wires {
                let wire = topo.wire(wire_id)?;
                let old_edges: Vec<OrientedEdge> = wire.edges().to_vec();
                let is_closed = wire.is_closed();
                let mut new_edges = Vec::new();
                let mut any_changed = false;

                for oe in &old_edges {
                    let mut replacements = self.resolved_edges(oe.edge());
                    if replacements.len() != 1 || replacements[0] != oe.edge() {
                        any_changed = true;
                    }
                    if !oe.is_forward() {
                        replacements.reverse();
                    }
                    for replacement in replacements {
                        new_edges.push(OrientedEdge::new(replacement, oe.is_forward()));
                    }
                }

                if any_changed {
                    if new_edges.is_empty() {
                        return Err(HealError::FixFailed(format!(
                            "edge replacements would empty wire {wire_id:?} on retained face {fid:?}"
                        )));
                    }
                    let new_wire = remus_topology::wire::Wire::new(new_edges, is_closed)?;
                    let new_wire_id = topo.add_wire(new_wire);

                    let face_mut = topo.face_mut(fid)?;
                    if face_mut.outer_wire() == wire_id {
                        face_mut.set_outer_wire(new_wire_id);
                    } else {
                        let iw = face_mut.inner_wires_mut();
                        for w in iw.iter_mut() {
                            if *w == wire_id {
                                *w = new_wire_id;
                            }
                        }
                    }
                }
            }
        }

        Ok(())
    }

    /// Apply face replacements: update shell face list.
    fn apply_face_replacements(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
    ) -> Result<(), HealError> {
        for shell_id in self.final_shell_ids(topo, solid_id)? {
            let old_faces = topo.shell(shell_id)?.faces().to_vec();
            let mut new_faces = Vec::new();
            for &face_id in &old_faces {
                new_faces.extend(self.resolved_faces(face_id));
            }

            if new_faces != old_faces {
                if new_faces.is_empty() {
                    return Err(HealError::FixFailed(format!(
                        "face replacements would empty retained shell {shell_id:?}"
                    )));
                }
                *topo.shell_mut(shell_id)? = remus_topology::shell::Shell::new(new_faces)?;
            }
        }
        Ok(())
    }

    /// Apply shell replacements (if any shells themselves were replaced).
    fn apply_shell_replacements(
        &self,
        topo: &mut Topology,
        solid_id: SolidId,
    ) -> Result<(), HealError> {
        if self.shells.is_empty() {
            return Ok(());
        }

        let solid = topo.solid(solid_id)?;
        let old_outer = solid.outer_shell();
        let old_inner = solid.inner_shells().to_vec();
        let new_outer = self.resolve_shell(old_outer).ok_or_else(|| {
            HealError::FixFailed(format!(
                "cannot remove outer shell {old_outer:?} from solid {solid_id:?}"
            ))
        })?;
        let new_inner: Vec<_> = old_inner
            .iter()
            .filter_map(|&shell| self.resolve_shell(shell))
            .collect();

        if new_outer != old_outer || new_inner != old_inner {
            *topo.solid_mut(solid_id)? = Solid::new(new_outer, new_inner);
        }

        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
    use remus_topology::face::{Face, FaceId, FaceSurface};
    use remus_topology::shell::{Shell, ShellId};
    use remus_topology::solid::{Solid, SolidId};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire, WireId};

    use super::ReShape;

    fn add_edge(topo: &mut Topology, start: Point3, end: Point3) -> EdgeId {
        let start = topo.add_vertex(Vertex::new(start, 1e-7));
        let end = topo.add_vertex(Vertex::new(end, 1e-7));
        topo.add_edge(Edge::new(start, end, EdgeCurve::Line))
    }

    fn add_wire(topo: &mut Topology, edge: EdgeId) -> WireId {
        topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], false).unwrap())
    }

    fn add_face(topo: &mut Topology, wire: WireId) -> FaceId {
        topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    fn add_shell(topo: &mut Topology, faces: Vec<FaceId>) -> ShellId {
        topo.add_shell(Shell::new(faces).unwrap())
    }

    fn add_solid(topo: &mut Topology, outer: ShellId, inner: Vec<ShellId>) -> SolidId {
        topo.add_solid(Solid::new(outer, inner))
    }

    #[test]
    fn apply_resolves_edge_wire_and_face_replacement_chains() {
        let mut topo = Topology::new();
        let edge_a = add_edge(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        let edge_b = add_edge(
            &mut topo,
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        );
        let edge_c = add_edge(
            &mut topo,
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
        );
        let wire_a = add_wire(&mut topo, edge_a);
        let wire_b = add_wire(&mut topo, edge_a);
        let wire_c = add_wire(&mut topo, edge_a);
        let face_a = add_face(&mut topo, wire_a);
        let face_b = add_face(&mut topo, wire_a);
        let face_c = add_face(&mut topo, wire_a);
        let shell = add_shell(&mut topo, vec![face_a]);
        let solid = add_solid(&mut topo, shell, vec![]);

        let mut reshape = ReShape::new();
        reshape.replace_edge(edge_a, edge_b);
        reshape.replace_edge(edge_b, edge_c);
        reshape.replace_wire(wire_a, wire_b);
        reshape.replace_wire(wire_b, wire_c);
        reshape.replace_face(face_a, face_b);
        reshape.replace_face(face_b, face_c);

        reshape.apply(&mut topo, solid).unwrap();

        let final_face = topo.shell(shell).unwrap().faces()[0];
        assert_eq!(final_face, face_c, "face replacement chain stopped early");
        let final_wire = topo.face(final_face).unwrap().outer_wire();
        let final_edges = topo.wire(final_wire).unwrap().edges();
        assert_eq!(final_edges.len(), 1);
        assert_eq!(
            final_edges[0].edge(),
            edge_c,
            "edge replacement chain stopped early"
        );
        assert_ne!(final_wire, wire_a, "wire replacement was not applied");
    }

    #[test]
    fn apply_updates_vertices_edges_wires_faces_and_shells_on_inner_shells() {
        let mut topo = Topology::new();
        let outer_edge = add_edge(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        let outer_wire = add_wire(&mut topo, outer_edge);
        let outer_face = add_face(&mut topo, outer_wire);
        let outer_shell = add_shell(&mut topo, vec![outer_face]);

        let old_start = topo.add_vertex(Vertex::new(Point3::new(0.0, 1.0, 0.0), 1e-7));
        let end = topo.add_vertex(Vertex::new(Point3::new(1.0, 1.0, 0.0), 1e-7));
        let new_start = topo.add_vertex(Vertex::new(Point3::new(0.0, 1.0, 1.0), 1e-7));
        let inner_edge = topo.add_edge(Edge::new(old_start, end, EdgeCurve::Line));
        let replacement_edge = topo.add_edge(Edge::new(old_start, end, EdgeCurve::Line));
        let inner_wire = add_wire(&mut topo, inner_edge);
        let replacement_wire = add_wire(&mut topo, inner_edge);
        let kept_face = add_face(&mut topo, inner_wire);
        let removed_face = add_face(&mut topo, inner_wire);
        let old_inner_shell = add_shell(&mut topo, vec![kept_face, removed_face]);
        let new_inner_shell = add_shell(&mut topo, vec![kept_face, removed_face]);
        let solid = add_solid(&mut topo, outer_shell, vec![old_inner_shell]);

        let mut reshape = ReShape::new();
        reshape.replace_vertex(old_start, new_start);
        reshape.replace_edge(inner_edge, replacement_edge);
        reshape.replace_wire(inner_wire, replacement_wire);
        reshape.remove_face(removed_face);
        reshape.replace_shell(old_inner_shell, new_inner_shell);

        reshape.apply(&mut topo, solid).unwrap();

        let solid_data = topo.solid(solid).unwrap();
        assert_eq!(solid_data.outer_shell(), outer_shell);
        assert_eq!(solid_data.inner_shells(), &[new_inner_shell]);
        assert_eq!(topo.shell(new_inner_shell).unwrap().faces(), &[kept_face]);

        let final_wire = topo.face(kept_face).unwrap().outer_wire();
        let final_edge = topo.wire(final_wire).unwrap().edges()[0].edge();
        assert_eq!(final_edge, replacement_edge);
        assert_eq!(topo.edge(final_edge).unwrap().start(), new_start);
    }

    #[test]
    fn final_shell_ids_deduplicates_resolved_shells_in_reference_order() {
        let mut topo = Topology::new();
        let edge = add_edge(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        let wire = add_wire(&mut topo, edge);
        let face = add_face(&mut topo, wire);
        let outer = add_shell(&mut topo, vec![face]);
        let old_inner = add_shell(&mut topo, vec![face]);
        let new_inner = add_shell(&mut topo, vec![face]);
        let solid = add_solid(
            &mut topo,
            outer,
            vec![old_inner, new_inner, old_inner, outer],
        );

        let mut reshape = ReShape::new();
        reshape.replace_shell(old_inner, new_inner);

        assert_eq!(
            reshape.final_shell_ids(&topo, solid).unwrap(),
            vec![outer, new_inner]
        );
    }

    #[test]
    fn apply_reverses_split_edge_order_for_reversed_wire_use() {
        let mut topo = Topology::new();
        let start = Point3::new(0.0, 0.0, 0.0);
        let middle = Point3::new(1.0, 0.0, 0.0);
        let end = Point3::new(2.0, 0.0, 0.0);
        let original = add_edge(&mut topo, start, end);
        let first = add_edge(&mut topo, start, middle);
        let second = add_edge(&mut topo, middle, end);
        let wire =
            topo.add_wire(Wire::new(vec![OrientedEdge::new(original, false)], false).unwrap());
        let face = add_face(&mut topo, wire);
        let shell = add_shell(&mut topo, vec![face]);
        let solid = add_solid(&mut topo, shell, vec![]);

        let mut reshape = ReShape::new();
        reshape.split_edge(original, vec![first, second]);
        reshape.apply(&mut topo, solid).unwrap();

        let final_wire = topo.face(face).unwrap().outer_wire();
        let final_edges = topo.wire(final_wire).unwrap().edges();
        assert_eq!(final_edges.len(), 2);
        assert_eq!(final_edges[0].edge(), second);
        assert!(!final_edges[0].is_forward());
        assert_eq!(final_edges[1].edge(), first);
        assert!(!final_edges[1].is_forward());
    }
}
