//! Diagnostic-only allocator. Injected into a disposable measurement checkout.
#![allow(unsafe_code, missing_docs, clippy::undocumented_unsafe_blocks)]
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
static ALLOCATED: AtomicUsize = AtomicUsize::new(0);
static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);
static CLONES: AtomicUsize = AtomicUsize::new(0);
static CLONE_BYTES: AtomicUsize = AtomicUsize::new(0);
struct Meter;
fn add(bytes: usize) {
    ALLOCATED.fetch_add(bytes, Relaxed);
    let live = LIVE.fetch_add(bytes, Relaxed) + bytes;
    PEAK.fetch_max(live, Relaxed);
}
unsafe impl GlobalAlloc for Meter {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let ptr = unsafe { System.alloc(layout) };
        if !ptr.is_null() { add(layout.size()); }
        ptr
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let ptr = unsafe { System.alloc_zeroed(layout) };
        if !ptr.is_null() { add(layout.size()); }
        ptr
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        LIVE.fetch_sub(layout.size(), Relaxed);
        unsafe { System.dealloc(ptr, layout); }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        let result = unsafe { System.realloc(ptr, layout, size) };
        if !result.is_null() {
            LIVE.fetch_sub(layout.size(), Relaxed);
            add(size);
        }
        result
    }
}
#[global_allocator]
static METER: Meter = Meter;
pub fn allocated() -> usize { ALLOCATED.load(Relaxed) }
pub fn record_clone(before: usize) {
    CLONES.fetch_add(1, Relaxed);
    CLONE_BYTES.fetch_add(allocated() - before, Relaxed);
}
pub fn reset() {
    CLONES.store(0, Relaxed);
    CLONE_BYTES.store(0, Relaxed);
    PEAK.store(LIVE.load(Relaxed), Relaxed);
}
pub fn read() -> [usize; 4] {
    [CLONES.load(Relaxed), CLONE_BYTES.load(Relaxed), LIVE.load(Relaxed), PEAK.load(Relaxed)]
}
