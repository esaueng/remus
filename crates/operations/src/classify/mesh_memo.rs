//! Operation-local reuse of identical robust-classification meshes.
//!
//! Fillet attempts and their volume/convexity checks revisit the same pristine
//! solid after rollback. Reuse the complete analysis mesh and its winding tree
//! under every mesher input (including native handle order, pcurves, tolerances,
//! geometry, and request). This scratch memo exists only during a scoped blend,
//! holds at most 4 MiB of estimated storage including keys, and never memoizes failures.

use remus_math::context::CacheScope;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex, PoisonError};

use remus_topology::{Topology, solid::SolidId};

use super::winding_tree::WindingTree;
use crate::OperationsError;
use crate::measure::volume_memo::{ContentKey, classification_mesh_key};
use crate::tessellate::TriangleMesh;

const BYTE_BUDGET: usize = 4 * 1024 * 1024;
const KEY_BYTE_BUDGET: usize = BYTE_BUDGET;

pub(super) struct AnalysisMesh {
    pub(super) mesh: TriangleMesh,
    pub(super) winding: Option<WindingTree>,
}

struct Entry {
    key: ContentKey,
    deflection: u64,
    value: Arc<AnalysisMesh>,
    bytes: usize,
}

#[derive(Default)]
struct Memo {
    depth: usize,
    entries: VecDeque<Entry>,
    bytes: usize,
}

#[derive(Default)]
struct MeshLeaf(Mutex<Option<Memo>>);

impl MeshLeaf {
    fn with<T>(&self, run: impl FnOnce(&mut Option<Memo>) -> T) -> T {
        let mut slot = self.0.lock().unwrap_or_else(PoisonError::into_inner);
        run(&mut slot)
    }
}

fn leaf() -> Option<Arc<MeshLeaf>> {
    CacheScope::ambient()?.existing::<MeshLeaf>()
}

struct Scope(Arc<MeshLeaf>);
impl Drop for Scope {
    fn drop(&mut self) {
        self.0.with(|slot| {
            if let Some(memo) = slot.as_mut() {
                memo.depth -= 1;
                if memo.depth == 0 {
                    *slot = None;
                }
            }
        });
    }
}

/// Share bounded scratch meshes during one blend cascade, including nested attempts.
pub fn scoped<T>(run: impl FnOnce() -> T) -> T {
    let leaf = crate::cache::thread_default_scope().part::<MeshLeaf>();
    leaf.with(|slot| slot.get_or_insert_with(Memo::default).depth += 1);
    let _scope = Scope(leaf);
    run()
}

pub(super) fn mesh(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
) -> Result<Arc<AnalysisMesh>, OperationsError> {
    let owner = leaf();
    let enabled = owner
        .as_ref()
        .is_some_and(|leaf| leaf.with(|slot| slot.is_some()));
    let key = enabled
        .then(|| classification_mesh_key(topo, solid, KEY_BYTE_BUDGET))
        .flatten();
    if let Some(key) = &key {
        let found = owner.as_ref().and_then(|leaf| {
            leaf.with(|slot| {
                let memo = slot.as_mut()?;
                let index = memo.entries.iter().position(|entry| {
                    entry.deflection == deflection.to_bits() && entry.key == *key
                })?;
                let entry = memo.entries.remove(index)?;
                let value = Arc::clone(&entry.value);
                memo.entries.push_back(entry);
                Some(value)
            })
        });
        if let Some(value) = found {
            let _hit = crate::performance::span("classification.mesh.hit");
            return Ok(value);
        }
    }
    let _miss = crate::performance::span(if key.is_some() {
        "classification.mesh.miss"
    } else {
        "classification.mesh.uncacheable"
    });
    let mesh = crate::tessellate::tessellate_solid_for_analysis(
        topo,
        solid,
        deflection,
        remus_math::chord::DEFAULT_ANGULAR_TOL,
    )?;
    #[cfg(test)]
    super::ROBUST_MESH_BUILDS.with(|builds| builds.set(builds.get() + 1));
    let winding = WindingTree::new(&mesh);
    let value = Arc::new(AnalysisMesh { mesh, winding });
    if let Some(key) = key {
        let m = &value.mesh;
        let bytes = key.bytes()
            + m.positions.capacity() * std::mem::size_of::<remus_math::vec::Point3>()
            + m.normals.capacity() * std::mem::size_of::<remus_math::vec::Vec3>()
            + m.indices.capacity() * std::mem::size_of::<u32>()
            + value
                .winding
                .as_ref()
                .map_or(0, WindingTree::retained_bytes)
            + std::mem::size_of::<Entry>()
            + std::mem::size_of::<AnalysisMesh>()
            + 2 * std::mem::size_of::<usize>();
        if bytes <= BYTE_BUDGET
            && let Some(owner) = &owner
        {
            owner.with(|slot| {
                if let Some(memo) = slot.as_mut() {
                    while memo.bytes + bytes > BYTE_BUDGET {
                        let Some(old) = memo.entries.pop_front() else {
                            break;
                        };
                        memo.bytes -= old.bytes;
                    }
                    memo.bytes += bytes;
                    memo.entries.push_back(Entry {
                        key,
                        deflection: deflection.to_bits(),
                        value: Arc::clone(&value),
                        bytes,
                    });
                }
            });
        }
    }
    Ok(value)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use crate::primitives::make_cylinder;
    use crate::transform::transform_solid;
    use remus_math::mat::Mat4;

    #[test]
    fn scoped_meshes_survive_unrelated_mutations_but_never_geometry_or_request_changes() {
        let mut topo = Topology::new();
        let solid = make_cylinder(&mut topo, 2., 5.).unwrap();
        assert!(!Arc::ptr_eq(
            &mesh(&topo, solid, 0.01).unwrap(),
            &mesh(&topo, solid, 0.01).unwrap()
        ));
        scoped(|| {
            let first = mesh(&topo, solid, 0.01).unwrap();
            make_cylinder(&mut topo, 1., 2.).unwrap();
            let copy = topo.clone();
            let same = scoped(|| mesh(&copy, solid, 0.01).unwrap());
            assert!(Arc::ptr_eq(&first, &same));
            assert!(!Arc::ptr_eq(&first, &mesh(&topo, solid, 0.02).unwrap()));
            transform_solid(&mut topo, solid, &Mat4::translation(3., 0., 0.)).unwrap();
            let moved = mesh(&topo, solid, 0.01).unwrap();
            assert!(!Arc::ptr_eq(&first, &moved));
            let fresh = crate::tessellate::tessellate_solid_for_analysis(
                &topo,
                solid,
                0.01,
                remus_math::chord::DEFAULT_ANGULAR_TOL,
            )
            .unwrap();
            assert_eq!(moved.mesh.positions, fresh.positions);
            assert_eq!(moved.mesh.normals, fresh.normals);
            assert_eq!(moved.mesh.indices, fresh.indices);
            leaf()
                .unwrap()
                .with(|slot| assert!(slot.as_ref().unwrap().bytes <= BYTE_BUDGET));
        });
        leaf().unwrap().with(|slot| assert!(slot.is_none()));
    }
}
