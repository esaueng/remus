//! Narrow chunk/page copy-on-write prototype for the PERF-T02 mechanism
//! comparison (test-only; never shipped).
//!
//! Models the two candidate rollback stores over the same scripted
//! workloads and counts copied bytes exactly:
//!
//! - [`PagedArena`]: slots grouped into `Arc`-shared pages of [`PAGE_SLOTS`]
//!   slots. A snapshot clones the page handles; the first write to a
//!   shared page copies the whole page. This is the most favorable sound
//!   COW shape: pages are small and writes route through the barrier.
//! - [`UndoSketch`]: one `(index, old value)` record per write, the shape
//!   implemented in production (`undo_log.rs`).
//!
//! Findings (asserted below as exact byte counts, so they are reviewed
//! with the code rather than pasted from a run):
//!
//! 1. Scattered writes amplify by the page size under COW (one touched
//!    slot copies its whole page); undo copies only touched slots.
//! 2. Clustered appends favor COW on transient metadata (one tail page vs
//!    one identity record per allocation) — but pure appends need neither
//!    durably, which is what the PERF-T03 append-only path exploits.
//! 3. A large payload co-resident with a touched small slot is copied whole
//!    under COW and untouched under undo.
//! 4. COW at the arena level cannot see writes through an already-escaped
//!    `&mut` (the shape of every `Topology::*_mut` accessor): any write
//!    path that skips the exclusivity check aliases the snapshot, so
//!    covering the audited mutation routes would require changing every
//!    accessor's return type across all crates. Undo captures before escape
//!    and needs no signature change.
//!
//! Selection: undo ships (findings 1, 3, 4); COW is retained here only as
//! the documented comparison.

#![allow(clippy::unwrap_used, clippy::expect_used, missing_docs)]

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::Arc;

/// Slots per copy-on-write page.
const PAGE_SLOTS: usize = 64;
/// Words per small payload (a vertex plus padding is smaller; this favors COW).
const SMALL_WORDS: usize = 4;

/// Heap payload: empty or small models analytic entities, large models a
/// NURBS control net co-resident on the same page.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
struct Payload(Vec<u64>);

impl Payload {
    fn small() -> Self {
        Self(vec![0u64; SMALL_WORDS])
    }

    fn large(words: usize) -> Self {
        Self(vec![7u64; words])
    }

    fn heap_bytes(&self) -> usize {
        self.0.len() * 8
    }
}

/// Page-shared arena with a write barrier. Snapshots share pages; the first
/// write to a shared page copies it whole.
#[derive(Debug, Clone, Default)]
struct PagedArena {
    pages: Vec<Arc<Vec<Payload>>>,
    len: usize,
    copied_bytes: usize,
}

impl PagedArena {
    fn snapshot(&self) -> Self {
        Self {
            pages: self.pages.clone(),
            len: self.len,
            copied_bytes: 0,
        }
    }

    fn ensure_exclusive_page(&mut self, page: usize) {
        if Arc::strong_count(&self.pages[page]) > 1 {
            let cloned = Arc::new(self.pages[page].as_ref().clone());
            self.copied_bytes += cloned.iter().map(Payload::heap_bytes).sum::<usize>();
            self.pages[page] = cloned;
        }
    }

    fn alloc(&mut self, value: Payload) {
        if self.len.is_multiple_of(PAGE_SLOTS) {
            self.pages
                .push(Arc::new(vec![Payload::default(); PAGE_SLOTS]));
        }
        let page = self.len / PAGE_SLOTS;
        self.ensure_exclusive_page(page);
        Arc::get_mut(&mut self.pages[page]).unwrap()[self.len % PAGE_SLOTS] = value;
        self.len += 1;
    }

    fn write(&mut self, index: usize, value: Payload) {
        let page = index / PAGE_SLOTS;
        self.ensure_exclusive_page(page);
        Arc::get_mut(&mut self.pages[page]).unwrap()[index % PAGE_SLOTS] = value;
    }
}

/// One old value per write, the production undo shape.
#[derive(Debug, Default)]
struct UndoSketch {
    records: Vec<(usize, Payload)>,
    recorded_bytes: usize,
}

impl UndoSketch {
    fn mark(&self) -> usize {
        self.records.len()
    }

    fn alloc(&mut self, index: usize) {
        // Allocation records carry no payload, only the slot identity.
        self.records.push((index, Payload::default()));
        self.recorded_bytes += 16;
    }

    fn write(&mut self, old: Payload) {
        let index = self.records.len();
        self.recorded_bytes += old.heap_bytes() + 16;
        self.records.push((index, old));
    }

    fn rewind_to(&mut self, mark: usize) {
        self.records.truncate(mark);
    }
}

#[test]
fn scattered_writes_amplify_by_page_size_under_cow() {
    const PAGES: usize = 1000;
    let mut base = PagedArena::default();
    for _ in 0..PAGES * PAGE_SLOTS {
        base.alloc(Payload::small());
    }

    // One scattered write per page: the fixed-local-edit shape at scale.
    let mut cow = base.snapshot();
    for page in 0..PAGES {
        cow.write(page * PAGE_SLOTS, Payload::small());
    }
    let mut undo = UndoSketch::default();
    for _ in 0..PAGES {
        undo.write(Payload::small());
    }
    let _ = undo.mark();

    // COW copies every touched page whole: 1000 pages x 64 slots x 4 words.
    assert_eq!(cow.copied_bytes, PAGES * PAGE_SLOTS * SMALL_WORDS * 8);
    // Undo copies only the 1000 touched slots plus per-record identity.
    assert_eq!(undo.recorded_bytes, PAGES * (SMALL_WORDS * 8 + 16));
    assert!(
        cow.copied_bytes >= 40 * undo.recorded_bytes,
        "scattered writes must amplify COW by roughly the page size"
    );
    undo.rewind_to(0);
}

#[test]
fn clustered_appends_favor_cow_on_metadata_only() {
    // 150 box-shaped appends of 64 small slots (the PERF-T03 workload shape).
    const BOXES: usize = 150;
    let mut base = PagedArena::default();
    // A mid-document snapshot with a partially filled shared tail page
    // (100 slots: one full page plus 36 shared slots).
    for _ in 0..100 {
        base.alloc(Payload::small());
    }
    let mut cow = base.snapshot();
    for _ in 0..BOXES * PAGE_SLOTS {
        cow.alloc(Payload::small());
    }
    let mut undo = UndoSketch::default();
    for i in 0..BOXES * PAGE_SLOTS {
        undo.alloc(i);
    }
    // COW copies at most the one shared tail page it started on.
    assert_eq!(cow.copied_bytes, 36 * SMALL_WORDS * 8);
    // Undo records one identity entry per allocation.
    assert_eq!(undo.recorded_bytes, BOXES * PAGE_SLOTS * 16);
    // Pure appends need neither durably: the append-only path truncates by
    // high-water mark at O(1). This cell only bounds the transient metadata
    // both general mechanisms would carry.
    assert!(undo.recorded_bytes < 200_000);
}

#[test]
fn coresident_large_payload_is_copied_whole_under_cow() {
    // One 48x48-class net (2304 words) sharing its page with small slots;
    // a single small write lands on that page.
    let mut base = PagedArena::default();
    base.alloc(Payload::large(2304));
    for _ in 1..PAGE_SLOTS {
        base.alloc(Payload::small());
    }
    let mut cow = base.snapshot();
    cow.write(1, Payload::small());
    let mut undo = UndoSketch::default();
    undo.write(Payload::small());

    assert_eq!(
        cow.copied_bytes,
        2304 * 8 + (PAGE_SLOTS - 1) * SMALL_WORDS * 8
    );
    assert_eq!(undo.recorded_bytes, SMALL_WORDS * 8 + 16);
    assert!(
        cow.copied_bytes > 300 * undo.recorded_bytes,
        "a touched page copies its NURBS payload under COW, untouched under undo"
    );
}

#[test]
fn escaped_mutable_references_bypass_arena_level_barriers() {
    // The decisive API point, in safe Rust: a write path that never consults
    // the sharing state aliases every snapshot. `Topology::*_mut` hands out
    // `&mut T`, and every later write through that reference performs no
    // arena-level check — exactly the `borrow_mut` below, which no page
    // barrier observes.
    let live = Rc::new(RefCell::new(vec![Payload::small(); PAGE_SLOTS]));
    let snapshot = Rc::clone(&live);
    live.borrow_mut()[3] = Payload::large(8);
    assert_eq!(snapshot.borrow()[3], Payload::large(8));
    // Covering the audited routes under COW would therefore require removing
    // these escapes — changing every accessor's return type in every crate —
    // while undo captures before escape with no signature change.
}
