//! A bounded, content-keyed store shared by the crate's opt-in memos.
//!
//! Each memo keeps one per thread, keyed on the exact bits of everything its
//! value depends on, so a hit returns what recomputing would. Past its
//! capacity, or past its weight budget when a memo weighs its entries, the
//! oldest entries go first; a zero capacity disables it.

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
    /// Most total weight held (see [`BoundedMemo::insert_weighted`]).
    budget: usize,
    /// Total weight held.
    weight: usize,
    values: DetHashMap<K, V>,
    /// Keys oldest first, each with the weight it was stored with.
    order: VecDeque<(K, usize)>,
    hits: u64,
    misses: u64,
}

impl<K, V> Default for BoundedMemo<K, V> {
    fn default() -> Self {
        Self {
            capacity: 0,
            budget: usize::MAX,
            weight: 0,
            values: DetHashMap::default(),
            order: VecDeque::new(),
            hits: 0,
            misses: 0,
        }
    }
}

impl<K, V> BoundedMemo<K, V> {
    /// A disabled memo whose entries, once enabled, may weigh `budget` in all
    /// (see [`BoundedMemo::insert_weighted`]).
    pub fn with_budget(budget: usize) -> Self {
        Self {
            budget,
            ..Self::default()
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

    /// Set the most total weight held; the oldest entries go first.
    pub fn set_budget(&mut self, budget: usize) {
        self.budget = budget;
        self.evict_to_capacity();
    }

    /// Total weight held.
    pub const fn weight(&self) -> usize {
        self.weight
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
        self.insert_weighted(key, value, 0);
    }

    /// Store a value computed after a miss, counting `weight` against the
    /// budget; ignored when off. An entry heavier than the whole budget is not
    /// stored, so it evicts nothing. A key already held keeps its stored
    /// weight: its value is a function of the key, so it is the same value.
    /// The store keeps two clones of the key, one in its map and one in its
    /// eviction queue, so a weight in bytes counts the key twice.
    pub fn insert_weighted(&mut self, key: &K, value: &V, weight: usize) {
        if self.capacity == 0 {
            return;
        }
        self.misses += 1;
        if weight > self.budget {
            return;
        }
        if self.values.insert(key.clone(), value.clone()).is_none() {
            self.order.push_back((key.clone(), weight));
            self.weight = self.weight.saturating_add(weight);
            self.evict_to_capacity();
        }
    }

    fn evict_to_capacity(&mut self) {
        while self.values.len() > self.capacity || self.weight > self.budget {
            let Some((oldest, weight)) = self.order.pop_front() else {
                self.values.clear();
                self.weight = 0;
                break;
            };
            self.values.remove(&oldest);
            self.weight = self.weight.saturating_sub(weight);
        }
        if self.capacity == 0 {
            self.order.clear();
            self.weight = 0;
        }
    }
}
