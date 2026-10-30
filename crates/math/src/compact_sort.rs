//! Sorts that share one compiled sort routine across call sites.
//!
//! `slice::sort_by` and `slice::sort_by_key` monomorphize the whole standard
//! stable sort (driftsort: quicksort, merge runs, small-sort networks) for
//! every distinct comparator closure, because each closure is its own type;
//! `slice::sort_unstable*` do the same with the unstable sort (ipnsort).
//! A few kilobytes per call site add up: before this module the kernel's
//! WebAssembly build carried about 270 such copies, 0.6–1.2 MB of code
//! depending on the optimization level.
//!
//! [`CompactSort`] produces exactly the order `sort_by` / `sort_by_key`
//! produce, but sorts a permutation of indices through one type-erased
//! comparator, so the sort itself is compiled once (once more for the
//! unstable variants). Each call site keeps only its comparator and a
//! per-element-type permutation step.
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
//!
//! The unstable variants are different. Where elements compare equal, the
//! order `slice::sort_unstable_by` leaves them in depends on the element
//! type, not just on the comparisons: the standard library picks its small
//! sort by the type's size and whether it is `Copy`, and the index sort runs
//! the `usize` choice. So [`CompactSort::compact_sort_unstable_by`] agrees
//! with `sort_unstable_by` exactly when no two distinguishable elements
//! compare equal: when ties occur only between identical values (`Ord` on
//! integers, handles and tuples of them, `f64::total_cmp`) or the keys are
//! unique. Then the sorted output is unique and every correct sort, stable or
//! not, returns it. With ties between distinguishable elements the tie order
//! is deterministic but may differ from `sort_unstable_by`'s, so convert such
//! a call only after showing its ties do not reach the result.

use std::cmp::Ordering;

/// Sorts compiled once for all comparators; see the [module docs](self).
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

    /// Unstable sort with a comparator; same order as
    /// `slice::sort_unstable_by` when no two distinguishable elements compare
    /// equal (see the [module docs](self)).
    fn compact_sort_unstable_by<F>(&mut self, compare: F)
    where
        F: FnMut(&T, &T) -> Ordering;

    /// Unstable sort by a key; same order as `slice::sort_unstable_by_key`
    /// under the same condition as [`Self::compact_sort_unstable_by`].
    fn compact_sort_unstable_by_key<K, F>(&mut self, key: F)
    where
        K: Ord,
        F: FnMut(&T) -> K;

    /// Unstable sort by `Ord`; same order as `slice::sort_unstable` when
    /// equal elements are identical, as for integers and handles.
    fn compact_sort_unstable(&mut self)
    where
        T: Ord;
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

    #[inline]
    fn compact_sort_unstable_by<F>(&mut self, mut compare: F)
    where
        F: FnMut(&T, &T) -> Ordering,
    {
        if self.len() <= SHORT_UNSTABLE_LEN {
            insertion_sort(self, &mut compare);
            return;
        }
        let mut order = Order::new(self.len());
        let items: &[T] = self;
        order.sort_unstable(&mut |a, b| compare(&items[a], &items[b]));
        apply_permutation(self, order.as_mut_slice());
    }

    #[inline]
    fn compact_sort_unstable_by_key<K, F>(&mut self, mut key: F)
    where
        K: Ord,
        F: FnMut(&T) -> K,
    {
        self.compact_sort_unstable_by(|a, b| key(a).cmp(&key(b)));
    }

    #[inline]
    fn compact_sort_unstable(&mut self)
    where
        T: Ord,
    {
        self.compact_sort_unstable_by(T::cmp);
    }
}

/// Unstable sorts up to this length run an insertion sort in place: a few
/// dozen bytes per call site, and much cheaper on short slices than building
/// and applying an index permutation. Its output is the same sorted sequence
/// whenever the unstable sorts promise one (no distinguishable ties).
const SHORT_UNSTABLE_LEN: usize = 12;

/// Sort `items` in place by insertion under `compare`.
#[inline]
fn insertion_sort<T, F>(items: &mut [T], compare: &mut F)
where
    F: FnMut(&T, &T) -> Ordering,
{
    for i in 1..items.len() {
        let mut j = i;
        while j > 0 && compare(&items[j - 1], &items[j]) == Ordering::Greater {
            items.swap(j - 1, j);
            j -= 1;
        }
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

    /// [`Self::sort`] with the standard unstable sort, instantiated once
    /// here for the unstable variants.
    #[inline(never)]
    fn sort_unstable(&mut self, compare: &mut dyn FnMut(usize, usize) -> Ordering) {
        // The one shared unstable instantiation behind CompactSort.
        #[allow(clippy::disallowed_methods)]
        self.as_mut_slice().sort_unstable_by(|&a, &b| compare(a, b));
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

        // Unstable sorts: equal elements are identical, so the sorted output
        // is unique and must match the standard unstable sort exactly.
        #[test]
        fn unstable_matches_std_when_ties_are_identical(
            keys in proptest::collection::vec(-8_i64..8, 0..300),
            floats in proptest::collection::vec(
                prop_oneof![
                    Just(0.0_f64), Just(-0.0), Just(1.5), Just(-2.25),
                    Just(f64::INFINITY), Just(f64::NEG_INFINITY),
                    -4.0_f64..4.0,
                ],
                0..300,
            ),
        ) {
            let mut want = keys.clone();
            want.sort_unstable();
            let mut got = keys.clone();
            got.compact_sort_unstable();
            prop_assert_eq!(&got, &want);

            let mut want_by = keys.clone();
            want_by.sort_unstable_by(|a, b| b.cmp(a));
            let mut got_by = keys.clone();
            got_by.compact_sort_unstable_by(|a, b| b.cmp(a));
            prop_assert_eq!(&got_by, &want_by);

            // A key that is a bijection of the element: (k mod 3, k) orders
            // differently from `k` alone, and still has no distinguishable ties.
            let mut want_key = keys.clone();
            want_key.sort_unstable_by_key(|&k| (k.rem_euclid(3), k));
            let mut got_key = keys;
            got_key.compact_sort_unstable_by_key(|&k| (k.rem_euclid(3), k));
            prop_assert_eq!(got_key, want_key);

            // `total_cmp` tells -0.0 from 0.0, so equal floats are identical.
            let mut want_f = floats.clone();
            want_f.sort_unstable_by(f64::total_cmp);
            let mut got_f = floats;
            got_f.compact_sort_unstable_by(f64::total_cmp);
            prop_assert_eq!(
                got_f.iter().map(|x| x.to_bits()).collect::<Vec<_>>(),
                want_f.iter().map(|x| x.to_bits()).collect::<Vec<_>>()
            );
        }

        // Ties between distinguishable elements (equal key, different input
        // position): the keys come out in the standard order and the result
        // is a permutation of the input; only the order within a run of equal
        // keys is the sort's own, and it is deterministic.
        #[test]
        fn unstable_orders_keys_like_std_with_distinguishable_ties(
            keys in proptest::collection::vec(0_u8..6, 0..300),
        ) {
            let tagged: Vec<(u8, usize)> = keys.iter().copied().zip(0..).collect();
            let mut want = tagged.clone();
            want.sort_unstable_by(|a, b| b.0.cmp(&a.0));
            let mut got = tagged.clone();
            got.compact_sort_unstable_by(|a, b| b.0.cmp(&a.0));
            prop_assert_eq!(
                got.iter().map(|e| e.0).collect::<Vec<_>>(),
                want.iter().map(|e| e.0).collect::<Vec<_>>()
            );
            let mut positions: Vec<usize> = got.iter().map(|e| e.1).collect();
            positions.sort_unstable();
            prop_assert_eq!(positions, (0..tagged.len()).collect::<Vec<_>>());
            let mut again = tagged;
            again.compact_sort_unstable_by(|a, b| b.0.cmp(&a.0));
            prop_assert_eq!(again, got);
        }

        #[test]
        fn matches_std_on_vec_payloads(keys in proptest::collection::vec(-4_i32..4, 0..64)) {
            // A non-Copy element type exercises the swap-based permutation.
            let tagged: Vec<(i32, Vec<usize>)> =
                keys.iter().copied().enumerate().map(|(i, k)| (k, vec![i; i % 3])).collect();
            let mut want = tagged.clone();
            want.sort_by_key(|e| e.0);
            let mut got = tagged.clone();
            got.compact_sort_by_key(|e| e.0);
            prop_assert_eq!(got, want);

            // Keys tie only between identical elements (equal key, empty
            // payload); a non-empty payload starts with its unique input
            // position. So the unstable output is unique and must match.
            let mut want_unstable = tagged.clone();
            want_unstable.sort_unstable_by_key(|e| (e.0, e.1.len(), e.1.first().copied()));
            let mut got_unstable = tagged;
            got_unstable.compact_sort_unstable_by_key(|e| (e.0, e.1.len(), e.1.first().copied()));
            prop_assert_eq!(got_unstable, want_unstable);
        }
    }
}
