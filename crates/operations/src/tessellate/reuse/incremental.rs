//! Incremental proofs over complete mesh connectivity. A small triangle
//! change updates only its directed edges / coincident-triangle keys. Large
//! changes, changed position partitions, and resource limits fall back to
//! the existing fresh pass. No geometry is modified here.

use super::super::mesh_ops::HalfEdgeTable;
use remus_math::compact_sort::CompactSort;
use remus_math::det_hash::{DetLookupHashMap, DetLookupHashSet, DetLookupState};

const MAX_CHANGED_TRIANGLES: usize = 4096;
const MAX_EDGE_DELTAS: usize = 16_384;

fn changed_triangles(before: &[u32], after: &[u32]) -> Option<Vec<usize>> {
    if before.len() != after.len() || !before.len().is_multiple_of(3) {
        return None;
    }
    let mut changed = Vec::new();
    for (i, (a, b)) in before
        .chunks_exact(3)
        .zip(after.chunks_exact(3))
        .enumerate()
    {
        if a != b {
            if changed.len() == MAX_CHANGED_TRIANGLES {
                return None;
            }
            changed.push(i);
        }
    }
    Some(changed)
}
fn edges(triangle: &[u32]) -> [(u32, u32); 3] {
    [
        (triangle[0], triangle[1]),
        (triangle[1], triangle[2]),
        (triangle[2], triangle[0]),
    ]
}

#[derive(Clone)]
pub(super) struct WeldUpdate {
    base: HalfEdgeTable,
    indices: Vec<u32>,
    deltas: DetLookupHashMap<(u32, u32), i32>,
    delta_limit: usize,
    unmatched: DetLookupHashSet<(u32, u32)>,
    boundary_uses: Vec<u32>,
    pub(super) updated_triangles: usize,
}
impl WeldUpdate {
    pub(super) fn delta_limit(indices: usize) -> usize {
        MAX_EDGE_DELTAS.min(indices.saturating_mul(4))
    }
    pub(super) fn capture(
        base: HalfEdgeTable,
        indices: &[u32],
        vertices: usize,
        missing: &[(u32, u32)],
    ) -> Option<Self> {
        if !indices.len().is_multiple_of(3) {
            return None;
        }
        let unmatched: DetLookupHashSet<_> = missing.iter().copied().collect();
        let mut boundary_uses = vec![0_u32; vertices];
        for &(a, b) in &unmatched {
            *boundary_uses.get_mut(a as usize)? += 1;
            *boundary_uses.get_mut(b as usize)? += 1;
        }
        Some(Self {
            base,
            indices: indices.to_vec(),
            deltas: DetLookupHashMap::with_capacity_and_hasher(
                Self::delta_limit(indices.len()),
                DetLookupState,
            ),
            delta_limit: Self::delta_limit(indices.len()),
            unmatched,
            boundary_uses,
            updated_triangles: 0,
        })
    }
    pub(super) fn bytes(&self) -> usize {
        // Base table is compressed rows: one u32 start per vertex, one
        // (end, triangle) pair per half-edge. Include conservative map slack.
        size_of::<Self>()
            + (self.boundary_uses.len() + 1) * 4
            + self.indices.capacity() * 12
            + self.boundary_uses.capacity() * 4
            + self.deltas.capacity() * 16
            + self.unmatched.capacity() * 16
    }
    fn count(&self, a: u32, b: u32) -> i64 {
        self.base
            .row(a)
            .iter()
            .filter(|&&(end, _)| end == b)
            .fold(0_i64, |count, _| count + 1)
            + i64::from(self.deltas.get(&(a, b)).copied().unwrap_or(0))
    }
    pub(super) fn update(&mut self, indices: &[u32], vertices: usize) -> Option<Vec<u32>> {
        if vertices != self.boundary_uses.len() {
            return None;
        }
        let changed = changed_triangles(&self.indices, indices)?;
        let mut delta = DetLookupHashMap::<(u32, u32), i32>::default();
        for &triangle in &changed {
            let old = &self.indices[3 * triangle..3 * triangle + 3];
            let new = &indices[3 * triangle..3 * triangle + 3];
            if new.iter().any(|&index| index as usize >= vertices) {
                return None;
            }
            for edge in edges(old) {
                *delta.entry(edge).or_default() -= 1;
            }
            for edge in edges(new) {
                *delta.entry(edge).or_default() += 1;
            }
        }
        delta.retain(|_, count| *count != 0);
        if self.deltas.len().saturating_add(delta.len()) > self.delta_limit {
            return None;
        }
        // Only presence of a changed directed edge or its reverse can change
        // the boundary set. Apply all count changes before testing either.
        for (&edge, &change) in &delta {
            let count = self.deltas.entry(edge).or_default();
            *count = count.checked_add(change)?;
            if *count == 0 {
                self.deltas.remove(&edge);
            }
        }
        for &(a, b) in delta.keys() {
            for (a, b) in [(a, b), (b, a)] {
                let boundary = self.count(a, b) > 0 && self.count(b, a) == 0;
                if boundary && self.unmatched.insert((a, b)) {
                    self.boundary_uses[a as usize] += 1;
                    self.boundary_uses[b as usize] += 1;
                } else if !boundary && self.unmatched.remove(&(a, b)) {
                    self.boundary_uses[a as usize] -= 1;
                    self.boundary_uses[b as usize] -= 1;
                }
            }
        }
        for &triangle in &changed {
            self.indices[3 * triangle..3 * triangle + 3]
                .copy_from_slice(&indices[3 * triangle..3 * triangle + 3]);
        }
        self.updated_triangles = changed.len();
        self.boundary_uses
            .iter()
            .enumerate()
            .filter(|&(_, count)| *count > 0)
            .map(|(vertex, _)| u32::try_from(vertex).ok())
            .collect()
    }
}

fn triangle_key(triangle: &[u32], partition: &[u32]) -> Option<[u32; 3]> {
    let mut key = [
        *partition.get(triangle[0] as usize)?,
        *partition.get(triangle[1] as usize)?,
        *partition.get(triangle[2] as usize)?,
    ];
    key.compact_sort_unstable();
    (key[0] != key[1] && key[1] != key[2]).then_some(key)
}

#[derive(Clone)]
pub(super) struct DedupeUpdate {
    indices: Vec<u32>,
    partition: Vec<u32>,
    triangles: DetLookupHashSet<[u32; 3]>,
    pub(super) updated_triangles: usize,
}
impl DedupeUpdate {
    pub(super) fn capture(indices: &[u32], partition: &[u32]) -> Option<Self> {
        if !indices.len().is_multiple_of(3)
            || indices.iter().any(|&v| v as usize >= partition.len())
        {
            return None;
        }
        let mut triangles =
            DetLookupHashSet::with_capacity_and_hasher(indices.len() / 3, DetLookupState);
        for tri in indices.chunks_exact(3) {
            if let Some(key) = triangle_key(tri, partition)
                && !triangles.insert(key)
            {
                return None;
            }
        }
        Some(Self {
            indices: indices.to_vec(),
            partition: partition.to_vec(),
            triangles,
            updated_triangles: 0,
        })
    }
    pub(super) fn bytes(&self) -> usize {
        size_of::<Self>()
            + self.indices.capacity() * 4
            + self.partition.capacity() * 4
            + self.triangles.capacity() * 16
    }
    pub(super) fn update(&mut self, indices: &[u32], partition: &[u32]) -> Option<()> {
        if partition != self.partition {
            return None;
        }
        let changed = changed_triangles(&self.indices, indices)?;
        if changed.iter().any(|&tri| {
            indices[3 * tri..3 * tri + 3]
                .iter()
                .any(|&v| v as usize >= partition.len())
        }) {
            return None;
        }
        // Remove every old key first: a changed triangle may legitimately
        // reuse a key freed by another changed triangle in the same update.
        for &tri in &changed {
            if let Some(key) = triangle_key(&self.indices[3 * tri..3 * tri + 3], partition) {
                self.triangles.remove(&key);
            }
        }
        for &tri in &changed {
            if let Some(key) = triangle_key(&indices[3 * tri..3 * tri + 3], partition)
                && !self.triangles.insert(key)
            {
                return None; // Drop the state and run the complete removal policy.
            }
        }
        for &tri in &changed {
            self.indices[3 * tri..3 * tri + 3].copy_from_slice(&indices[3 * tri..3 * tri + 3]);
        }
        self.updated_triangles = changed.len();
        Some(())
    }
}
