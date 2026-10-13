//! Stable sorts that share one compiled sort routine across call sites.
//!
//! `slice::sort_by` and `slice::sort_by_key` monomorphize the whole standard
//! stable sort (driftsort: quicksort, merge runs, small-sort networks) for
//! every distinct comparator closure, because each closure is its own type.
//! A few kilobytes per call site add up: before this module the kernel's
//! WebAssembly build carried about 270 such copies, 0.6–1.2 MB of code
//! depending on the optimization level.
//!
//! [`CompactSort`] produces exactly the order `sort_by` / `sort_by_key`
//! produce, but sorts a permutation of indices through one type-erased
//! comparator, so the sort itself is compiled once. Each call site keeps only
//! its comparator and a per-element-type permutation step.
//!
//! # When to use it
//!
//! Use it where sorting is not a measurable share of the work: sorting a
//! face's few dozen edges, an operation's candidate list, report rows. Keep
//! `sort_by` in genuinely sort-heavy loops (meshes, BVH builds, CDT), where
//! the indirect comparator call and the index buffer would cost time.
//!
//! # Equivalence
//!
//! A stable sort's output is fully determined by its comparator when the
//! comparator is a total preorder (what `sort_by` requires anyway), so the
//! result is identical to `sort_by` with the same comparator, ties included.
//! A comparator that is not a total preorder (for example
//! `partial_cmp(..).unwrap_or(Equal)` meeting a NaN) has no defined order
//! under either sort, and the two may then disagree.

use std::cmp::Ordering;

/// Stable sorts compiled once for all comparators; see the
/// [module docs](self).
pub trait CompactSort<T> {
    /// Stable sort with a comparator; same order as `slice::sort_by`.
    fn compact_sort_by<F>(&mut self, compare: F)
    where
        F: FnMut(&T, &T) -> Ordering;

    /// Stable sort by a key; same order as `slice::sort_by_key`.
    fn compact_sort_by_key<K, F>(&mut self, key: F)
    where
        K: Ord,
        F: FnMut(&T) -> K;
}

impl<T> CompactSort<T> for [T] {
    #[inline]
    fn compact_sort_by<F>(&mut self, mut compare: F)
    where
        F: FnMut(&T, &T) -> Ordering,
    {
        if self.len() < 2 {
            return;
        }
        let mut order = Order::new(self.len());
        let items: &[T] = self;
        order.sort(&mut |a, b| compare(&items[a], &items[b]));
        apply_permutation(self, order.as_mut_slice());
    }

    #[inline]
    fn compact_sort_by_key<K, F>(&mut self, mut key: F)
    where
        K: Ord,
        F: FnMut(&T) -> K,
    {
        self.compact_sort_by(|a, b| key(a).cmp(&key(b)));
    }
}

/// Index permutations up to this length live on the stack.
const INLINE_LEN: usize = 32;

/// A permutation of `0..len`, on the stack for short slices so the common
/// small sort allocates nothing (the standard sort does not either).
struct Order {
    len: usize,
    inline: [usize; INLINE_LEN],
    heap: Vec<usize>,
}

impl Order {
    /// The identity permutation of `0..len`.
    #[inline(never)]
    fn new(len: usize) -> Self {
        let mut order = Self {
            len,
            inline: [0; INLINE_LEN],
            heap: Vec::new(),
        };
        if len > INLINE_LEN {
            order.heap = (0..len).collect();
        } else {
            for (i, slot) in order.inline[..len].iter_mut().enumerate() {
                *slot = i;
            }
        }
        order
    }

    fn as_mut_slice(&mut self) -> &mut [usize] {
        if self.len > INLINE_LEN {
            &mut self.heap
        } else {
            &mut self.inline[..self.len]
        }
    }

    /// Stable-sort the indices under `compare`, after which entry `k` is the
    /// index of the element that belongs at position `k`.
    ///
    /// Not generic, so the standard stable sort is instantiated once here.
    #[inline(never)]
    fn sort(&mut self, compare: &mut dyn FnMut(usize, usize) -> Ordering) {
        // The one shared instantiation behind CompactSort.
        #[allow(clippy::disallowed_methods)]
        self.as_mut_slice().sort_by(|&a, &b| compare(a, b));
    }
}

/// Rearrange `items` so that position `k` receives the element previously at
/// `order[k]`. `order` must be a permutation of `0..items.len()`; it is left
/// as the identity.
#[inline(never)]
fn apply_permutation<T>(items: &mut [T], order: &mut [usize]) {
    for start in 0..order.len() {
        // Follow the cycle through `start`, pulling each source element into
        // place with one swap per step and marking visited slots as fixed.
        let mut slot = start;
        loop {
            let source = order[slot];
            order[slot] = slot;
            if source == start {
                break;
            }
            items.swap(slot, source);
            slot = source;
        }
    }
}

#[cfg(test)]
mod tests {
    // The reference sorts spell out the same comparators the compact sorts get.
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::unnecessary_sort_by)]
    use super::*;
    use proptest::prelude::*;

    #[test]
    fn empty_and_single_are_untouched() {
        let mut empty: Vec<u8> = Vec::new();
        empty.compact_sort_by(u8::cmp);
        assert!(empty.is_empty());
        let mut one = vec![7_u8];
        one.compact_sort_by_key(|&x| x);
        assert_eq!(one, [7]);
    }

    #[test]
    fn ties_keep_input_order() {
        let mut v = vec![(1, 'a'), (0, 'b'), (1, 'c'), (0, 'd'), (1, 'e')];
        v.compact_sort_by_key(|&(k, _)| k);
        assert_eq!(v, [(0, 'b'), (0, 'd'), (1, 'a'), (1, 'c'), (1, 'e')]);
    }

    #[test]
    fn float_comparator_matches_sort_by() {
        let base = [3.5, -1.0, 2.25, -1.0, 0.0, 9.0, -0.0, 2.25];
        let mut want: Vec<(f64, usize)> = base.iter().copied().zip(0..).collect();
        want.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut got: Vec<(f64, usize)> = base.iter().copied().zip(0..).collect();
        got.compact_sort_by(|a, b| a.0.total_cmp(&b.0));
        assert_eq!(got, want);
    }

    #[test]
    fn permutation_cycles_apply() {
        let mut items = ['a', 'b', 'c', 'd', 'e'];
        let mut order = [2, 0, 1, 4, 3];
        apply_permutation(&mut items, &mut order);
        assert_eq!(items, ['c', 'a', 'b', 'e', 'd']);
        assert_eq!(order, [0, 1, 2, 3, 4]);
    }

    proptest! {
        // Ties are frequent (keys in 0..8) and the payload records input
        // position, so any stability difference shows up as a mismatch.
        #[test]
        fn matches_std_stable_sort(keys in proptest::collection::vec(0_u8..8, 0..300)) {
            let tagged: Vec<(u8, usize)> = keys.iter().copied().zip(0..).collect();

            let mut want = tagged.clone();
            want.sort_by(|a, b| b.0.cmp(&a.0));
            let mut got = tagged.clone();
            got.compact_sort_by(|a, b| b.0.cmp(&a.0));
            prop_assert_eq!(&got, &want);

            let mut want_key = tagged.clone();
            want_key.sort_by_key(|&(k, _)| k);
            let mut got_key = tagged;
            got_key.compact_sort_by_key(|&(k, _)| k);
            prop_assert_eq!(got_key, want_key);
        }

        #[test]
        fn matches_std_on_vec_payloads(keys in proptest::collection::vec(-4_i32..4, 0..64)) {
            // A non-Copy element type exercises the swap-based permutation.
            let tagged: Vec<(i32, Vec<usize>)> =
                keys.iter().copied().enumerate().map(|(i, k)| (k, vec![i; i % 3])).collect();
            let mut want = tagged.clone();
            want.sort_by_key(|e| e.0);
            let mut got = tagged;
            got.compact_sort_by_key(|e| e.0);
            prop_assert_eq!(got, want);
        }
    }
}
