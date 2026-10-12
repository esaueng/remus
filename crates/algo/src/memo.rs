//! A bounded, content-keyed store shared by the crate's opt-in memos.
//!
//! Each memo keeps one per thread, keyed on the exact bits of everything its
//! value depends on, so a hit returns what recomputing would. Past its
//! capacity the oldest entries go first; a zero capacity disables it.

use std::collections::VecDeque;
use std::hash::Hash;

use remus_math::det_hash::DetHashMap;

/// Statistics of one memo.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MemoStats {
    /// Lookups answered from the memo.
    pub hits: u64,
    /// Lookups the enabled memo could not answer (it computed and stored).
    pub misses: u64,
    /// Entries held.
    pub len: usize,
    /// Most entries held; zero when the memo is off.
    pub capacity: usize,
}

pub struct BoundedMemo<K, V> {
    capacity: usize,
    values: DetHashMap<K, V>,
    order: VecDeque<K>,
    hits: u64,
    misses: u64,
}

impl<K, V> Default for BoundedMemo<K, V> {
    fn default() -> Self {
        Self {
            capacity: 0,
            values: DetHashMap::default(),
            order: VecDeque::new(),
            hits: 0,
            misses: 0,
        }
    }
}

impl<K: Hash + Eq + Clone, V: Clone> BoundedMemo<K, V> {
    /// Enable at `capacity` unless already enabled.
    pub const fn enable(&mut self, capacity: usize) {
        if self.capacity == 0 {
            self.capacity = capacity;
        }
    }

    /// Set the capacity; zero disables the memo and drops its contents.
    pub fn set_capacity(&mut self, capacity: usize) {
        self.capacity = capacity;
        self.evict_to_capacity();
    }

    pub fn stats(&self) -> MemoStats {
        MemoStats {
            hits: self.hits,
            misses: self.misses,
            len: self.values.len(),
            capacity: self.capacity,
        }
    }

    /// The stored value for `key`, counting a hit; `None` when absent or off.
    pub fn get(&mut self, key: &K) -> Option<V> {
        if self.capacity == 0 {
            return None;
        }
        let found = self.values.get(key).cloned();
        if found.is_some() {
            self.hits += 1;
        }
        found
    }

    /// Store a value computed after a miss; ignored when off.
    pub fn insert(&mut self, key: &K, value: &V) {
        if self.capacity == 0 {
            return;
        }
        self.misses += 1;
        if self.values.insert(key.clone(), value.clone()).is_none() {
            self.order.push_back(key.clone());
            self.evict_to_capacity();
        }
    }

    fn evict_to_capacity(&mut self) {
        while self.values.len() > self.capacity {
            let Some(oldest) = self.order.pop_front() else {
                self.values.clear();
                break;
            };
            self.values.remove(&oldest);
        }
        if self.capacity == 0 {
            self.order.clear();
        }
    }
}
