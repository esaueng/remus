//! Deterministic hashing primitives.
//!
//! The standard-library `HashMap`/`HashSet` seed their hasher from a
//! process-global source, so iteration order — and therefore any algorithm
//! whose output depends on the order it visits map entries — varies between
//! runs (and even between successive map instances within one process, since
//! the seed counter advances per instance). For geometry kernels this surfaces
//! as nondeterministic mesh welding and triangulation: the same input can
//! produce different vertex merges and face counts run-to-run.
//!
//! [`DetHashMap`] / [`DetHashSet`] use a fixed-seed FNV-1a hasher so iteration
//! order is fully reproducible for a given set of keys. Use them anywhere map
//! iteration order feeds geometry construction (vertex welds, triangulation,
//! shell assembly).
//!
//! # Security
//!
//! This is a deterministic, fixed-seed FNV-1a hasher. It is **not**
//! DoS-resistant: an adversary who controls the keys can trivially force
//! collisions. Use it only for internal geometry processing where keys are
//! derived from trusted topology, never for attacker-controlled input.

use std::hash::{BuildHasher, Hasher};

const FNV_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

/// Fixed-seed FNV-1a hasher. Deterministic across processes and instances.
///
/// Not DoS-resistant — see the [module docs](self#security). For internal
/// geometry processing only.
#[derive(Clone)]
pub struct DetHasher(u64);

impl DetHasher {
    /// Create a hasher seeded with the FNV-1a offset basis.
    #[must_use]
    pub const fn new() -> Self {
        Self(FNV_OFFSET)
    }
}

impl Default for DetHasher {
    fn default() -> Self {
        Self::new()
    }
}

impl Hasher for DetHasher {
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, bytes: &[u8]) {
        let mut h = self.0;
        for &b in bytes {
            h ^= u64::from(b);
            h = h.wrapping_mul(FNV_PRIME);
        }
        self.0 = h;
    }
}

/// `BuildHasher` for [`DetHasher`], seeding each hasher at the FNV-1a offset.
#[derive(Default, Clone)]
pub struct DetState;

impl BuildHasher for DetState {
    type Hasher = DetHasher;

    fn build_hasher(&self) -> Self::Hasher {
        DetHasher::new()
    }
}

/// A `HashMap` with deterministic, reproducible iteration order.
pub type DetHashMap<K, V> = std::collections::HashMap<K, V, DetState>;

/// A `HashSet` with deterministic, reproducible iteration order.
pub type DetHashSet<K> = std::collections::HashSet<K, DetState>;

/// Fixed-seed, word-at-a-time hasher for maps that are only ever looked up.
///
/// [`DetHasher`] folds keys in byte by byte, so an `(i64, i64, i64)` grid key
/// costs 24 serially dependent multiplies. This one folds each integer word
/// in with one multiply and avalanches in [`Hasher::finish`], which keeps the
/// high bits `hashbrown` reads well mixed. It is as deterministic as
/// [`DetHasher`], but it orders entries differently, so a map whose
/// iteration order reaches a result keeps [`DetHasher`]: switching it would
/// change that result. Use [`DetLookupHashMap`] / [`DetLookupHashSet`] only
/// where entries are never iterated in an order-dependent way (lookups,
/// inserts, removals, `len`, order-free `retain`).
///
/// Not DoS-resistant — see the [module docs](self#security).
#[derive(Clone, Default)]
pub struct DetLookupHasher(u64);

/// Multiplier of the word fold (the 64-bit Fx constant).
const LOOKUP_MULTIPLIER: u64 = 0x517c_c1b7_2722_0a95;

impl Hasher for DetLookupHasher {
    fn finish(&self) -> u64 {
        // MurmurHash3's 64-bit finalizer.
        let mut h = self.0;
        h ^= h >> 33;
        h = h.wrapping_mul(0xff51_afd7_ed55_8ccd);
        h ^= h >> 33;
        h = h.wrapping_mul(0xc4ce_b9fe_1a85_ec53);
        h ^ (h >> 33)
    }

    fn write(&mut self, bytes: &[u8]) {
        for chunk in bytes.chunks(8) {
            let mut word = [0_u8; 8];
            word[..chunk.len()].copy_from_slice(chunk);
            self.write_u64(u64::from_le_bytes(word));
        }
    }

    fn write_u8(&mut self, value: u8) {
        self.write_u64(u64::from(value));
    }

    fn write_u32(&mut self, value: u32) {
        self.write_u64(u64::from(value));
    }

    fn write_u64(&mut self, value: u64) {
        self.0 = (self.0.rotate_left(5) ^ value).wrapping_mul(LOOKUP_MULTIPLIER);
    }

    fn write_usize(&mut self, value: usize) {
        // usize is at most 64 bits on every supported target.
        self.write_u64(value as u64);
    }

    fn write_i32(&mut self, value: i32) {
        self.write_u32(value.cast_unsigned());
    }

    fn write_i64(&mut self, value: i64) {
        self.write_u64(value.cast_unsigned());
    }
}

/// `BuildHasher` for [`DetLookupHasher`].
#[derive(Default, Clone)]
pub struct DetLookupState;

impl BuildHasher for DetLookupState {
    type Hasher = DetLookupHasher;

    fn build_hasher(&self) -> Self::Hasher {
        DetLookupHasher::default()
    }
}

/// A deterministic `HashMap` for lookup-only use; see [`DetLookupHasher`].
pub type DetLookupHashMap<K, V> = std::collections::HashMap<K, V, DetLookupState>;

/// A deterministic `HashSet` for lookup-only use; see [`DetLookupHasher`].
pub type DetLookupHashSet<K> = std::collections::HashSet<K, DetLookupState>;

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;

    fn hash_writes(writes: &[&[u8]]) -> u64 {
        let mut h = DetHasher::new();
        for w in writes {
            h.write(w);
        }
        h.finish()
    }

    #[test]
    fn lookup_hasher_is_fixed_and_word_sensitive() {
        let hash = |key: (i64, i64, i64)| DetLookupState.hash_one(key);
        // Fixed seed: the same key hashes the same in every map and process.
        assert_eq!(hash((1, -2, 3)), hash((1, -2, 3)));
        assert_ne!(hash((1, -2, 3)), hash((3, -2, 1)));
        assert_ne!(hash((0, 0, 0)), hash((0, 0, 1)));
        // Byte writes fold in word by word.
        let mut bytes = DetLookupHasher::default();
        bytes.write(&7_u64.to_le_bytes());
        let mut word = DetLookupHasher::default();
        word.write_u64(7);
        assert_eq!(bytes.finish(), word.finish());
    }

    #[test]
    fn lookup_map_round_trips_grid_keys() {
        let mut map: DetLookupHashMap<(i64, i64, i64), u32> = DetLookupHashMap::default();
        let mut next = 0_u32;
        for x in -20..20_i64 {
            for y in -20..20_i64 {
                map.insert((x, y, x * y), next);
                next += 1;
            }
        }
        let mut expected = 0_u32;
        for x in -20..20_i64 {
            for y in -20..20_i64 {
                assert_eq!(map.get(&(x, y, x * y)), Some(&expected));
                expected += 1;
            }
        }
        assert_eq!(map.len(), 1600);
    }

    #[test]
    fn empty_hash_is_offset_basis() {
        assert_eq!(DetHasher::new().finish(), FNV_OFFSET);
    }

    #[test]
    fn distinct_sequences_differ() {
        assert_ne!(hash_writes(&[b"abc"]), hash_writes(&[b"abd"]));
    }

    #[test]
    fn write_is_order_sensitive() {
        assert_ne!(hash_writes(&[b"ab", b"cd"]), hash_writes(&[b"cd", b"ab"]),);
    }

    #[test]
    fn zero_state_does_not_restart() {
        // A running state that reaches exactly zero must keep folding in
        // subsequent bytes rather than silently reverting to the offset
        // basis. Find a single byte that drives the state to 0, then verify
        // that appending more bytes keeps the two distinct streams distinct.
        let target = (0u16..=u16::from(u8::MAX)).map(|b| b as u8).find(|&b| {
            let mut h = DetHasher::new();
            h.write(&[b]);
            h.finish() == 0
        });
        if let Some(zeroing) = target {
            let mut a = DetHasher::new();
            a.write(&[zeroing]);
            a.write(&[0x01]);

            // A hasher that began life at zero would, under the old buggy
            // logic, hash [0x01] identically. Build that comparison stream.
            let mut b = DetHasher::new();
            b.write(&[0x01]);
            assert_ne!(a.finish(), b.finish());
        }

        // Regardless of whether a zeroing byte exists for this FNV constant,
        // assert the invariant that distinct streams stay distinct even when
        // one of them is the empty (offset-basis) stream.
        assert_ne!(hash_writes(&[]), hash_writes(&[b"x"]));
    }
}
