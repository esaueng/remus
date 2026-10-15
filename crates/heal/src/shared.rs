//! Ownership checks before shell-scoped mutation of live topology.

use std::collections::BTreeSet;

use remus_topology::Topology;
use remus_topology::shell::ShellId;
use remus_topology::solid::SolidId;

use crate::HealError;

/// Refuse an aliased boundary before an in-place pass can change another
/// body, sheet, or standalone wire. Copying and naming such a boundary is a
/// separate operation; these low-level passes cannot silently claim it.
pub fn ensure_exclusive_shell(
    topo: &Topology,
    solid_id: SolidId,
    shell_id: ShellId,
) -> Result<(), HealError> {
    let faces: BTreeSet<_> = topo.shell(shell_id)?.faces().iter().copied().collect();
    let mut wires = BTreeSet::new();
    let mut edges = BTreeSet::new();
    for &fid in &faces {
        let face = topo.face(fid)?;
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            wires.insert(wid);
            edges.extend(
                topo.wire(wid)?
                    .edges()
                    .iter()
                    .map(remus_topology::wire::OrientedEdge::edge),
            );
        }
    }
    let refused = || {
        HealError::FixFailed(
            "shell repair requires an exclusive boundary; shared topology retained".into(),
        )
    };
    for (other, solid) in topo.solids().iter() {
        if other != solid_id
            && std::iter::once(solid.outer_shell())
                .chain(solid.inner_shells().iter().copied())
                .any(|sid| sid == shell_id)
        {
            return Err(refused());
        }
    }
    let selected = topo.solid(solid_id)?;
    let owned_shells: BTreeSet<_> = std::iter::once(selected.outer_shell())
        .chain(selected.inner_shells().iter().copied())
        .collect();
    for (sid, shell) in topo.shells().iter() {
        if !owned_shells.contains(&sid) && shell.faces().iter().any(|fid| faces.contains(fid)) {
            return Err(refused());
        }
    }
    let selected_faces: BTreeSet<_> = remus_topology::explorer::solid_faces(topo, solid_id)?
        .into_iter()
        .collect();
    let mut selected_wires = BTreeSet::new();
    for &fid in &selected_faces {
        let face = topo.face(fid)?;
        selected_wires
            .extend(std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()));
    }
    for (fid, face) in topo.faces().iter() {
        if selected_faces.contains(&fid) {
            continue;
        }
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            if wires.contains(&wid)
                || topo
                    .wire(wid)?
                    .edges()
                    .iter()
                    .any(|oe| edges.contains(&oe.edge()))
            {
                return Err(refused());
            }
        }
    }
    for (wid, wire) in topo.wires().iter() {
        if !selected_wires.contains(&wid)
            && wire.edges().iter().any(|oe| edges.contains(&oe.edge()))
        {
            return Err(refused());
        }
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::pipeline::process::HealProcess;
    use crate::test_fixtures::{cavity_shell, hollow_block, open_corner_gap};
    use remus_topology::face::Face;
    use remus_topology::shell::Shell;
    use remus_topology::solid::Solid;

    #[test]
    fn cavity_merge_refuses_foreign_boundary_and_rolls_back() {
        for alias in 0..3 {
            let mut topo = Topology::new();
            let solid = hollow_block(&mut topo);
            let shell = cavity_shell(&topo, solid);
            open_corner_gap(&mut topo, shell, 1e-9);
            let face = topo.shell(shell).unwrap().faces()[0];
            let foreign = if alias == 0 {
                topo.add_face(topo.face(face).unwrap().clone())
            } else {
                face
            };
            let foreign_shell = if alias == 2 {
                shell
            } else {
                topo.add_shell(Shell::new(vec![foreign]).unwrap())
            };
            let _other = topo.add_solid(Solid::new(foreign_shell, vec![]));
            let mut ctx = crate::context::HealContext::new();
            let mut config = crate::fix::FixConfig {
                fix_reorder: crate::fix::FixMode::Off,
                fix_connectivity: crate::fix::FixMode::Off,
                fix_closure: crate::fix::FixMode::Off,
                fix_small_edges: crate::fix::FixMode::Off,
                fix_self_intersection: crate::fix::FixMode::Off,
                fix_degenerate_edges: crate::fix::FixMode::Off,
                fix_gaps_2d: crate::fix::FixMode::Off,
                fix_gaps_3d: crate::fix::FixMode::Off,
                fix_lacking: crate::fix::FixMode::Off,
                fix_notched: crate::fix::FixMode::Off,
                fix_tail: crate::fix::FixMode::Off,
                fix_intersecting_edges: crate::fix::FixMode::Off,
                fix_wire_orientation: crate::fix::FixMode::Off,
                fix_add_natural_bound: crate::fix::FixMode::Off,
                fix_missing_seam: crate::fix::FixMode::Off,
                fix_small_area: crate::fix::FixMode::Off,
                fix_duplicate_faces: crate::fix::FixMode::Off,
                fix_intersecting_wires: crate::fix::FixMode::Off,
                fix_orientation: crate::fix::FixMode::Off,
                fix_same_parameter: crate::fix::FixMode::Off,
                fix_vertex_tolerance: crate::fix::FixMode::Off,
                fix_pcurve: crate::fix::FixMode::Off,
                fix_coincident_vertices: crate::fix::FixMode::Off,
                fix_wireframe: crate::fix::FixMode::Off,
                fix_split_common_vertex: crate::fix::FixMode::Off,
                fix_small_faces: crate::fix::FixMode::Off,
            };
            config.fix_coincident_vertices = crate::fix::FixMode::On;
            let result = crate::fix::solid::fix_solid(&mut topo, solid, &mut ctx, &config);
            assert!(matches!(result, Err(HealError::FixFailed(_))), "{result:?}");
            // Transaction bookkeeping may advance; live boundary authority may not.
            assert_eq!(
                topo.face(foreign).unwrap().outer_wire(),
                topo.face(face).unwrap().outer_wire()
            );
            assert_eq!(crate::test_fixtures::broken_junctions(&topo, shell), 2);
        }
    }

    #[test]
    fn sewing_refuses_shared_cavity_wires_in_both_execution_paths() {
        for with_history in [false, true] {
            let mut topo = Topology::new();
            let solid = hollow_block(&mut topo);
            let shell = cavity_shell(&topo, solid);
            let face = topo.shell(shell).unwrap().faces()[0];
            let wire = topo.face(face).unwrap().outer_wire();
            let original = topo.face(face).unwrap().clone();
            let foreign = topo.add_face(Face::new(
                wire,
                original.inner_wires().to_vec(),
                original.surface().clone(),
            ));
            let before = topo.wire(wire).unwrap().edges().to_vec();
            let mut process = HealProcess::new();
            process.add_step("sew_shells");
            let refused = if with_history {
                process.execute_with_history(&mut topo, solid).is_err()
            } else {
                process.execute(&mut topo, solid).is_err()
            };
            assert!(refused);
            assert_eq!(topo.face(foreign).unwrap().outer_wire(), wire);
            assert_eq!(
                topo.wire(wire)
                    .unwrap()
                    .edges()
                    .iter()
                    .map(|oe| (oe.edge(), oe.is_forward()))
                    .collect::<Vec<_>>(),
                before
                    .iter()
                    .map(|oe| (oe.edge(), oe.is_forward()))
                    .collect::<Vec<_>>()
            );
        }
    }
}
