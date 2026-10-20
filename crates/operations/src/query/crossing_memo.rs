//! Exact-content memo of per-face ray crossing counts for edge concavity.
//!
//! Edge relations and feature recognition classify four quadrant probes per
//! non-tangent edge, and each probe casts whole-solid rays whose crossing
//! count is summed face by face. After a direct edit almost every face is
//! unchanged and almost every probe sits where it sat before, so the same
//! face meets the same ray again. This memo stores each face's count under
//! the face's content and the ray's exact bits, and the classifier adds the
//! stored count in place of counting (see
//! [`remus_check::classify::FaceCrossingMemo`]).
//!
//! # Key
//!
//! A face's content enters as the words of [`face_content_words`]: its
//! reversal, every field of its surface, and every field of the wires,
//! edges, vertices, loops, coedges and pcurves under it, with handles as
//! first-occurrence ordinals. That is a superset of what the crossing count
//! reads (surface, reversal, and the boundary in wire order; it orders
//! nothing by handle value). Each distinct content is interned once, under
//! its whole word list, to an id that is never reused; the count is stored
//! under that id plus the bits of the ray's origin and direction. Words and
//! ray bits compare exactly, so a hit returns the count a fresh count would.
//! Planar faces are not stored: their count is one plane hit and a cached
//! polygon test, no dearer than a lookup. Errors are never stored.
//!
//! # Bounds and lifetime
//!
//! Off by default; applications opt in per thread, as for the face-box and
//! NURBS-area memos (the WASM kernel does on construction). Counts and
//! interned contents are each evicted in FIFO order under their own limits; a
//! count whose content was evicted is never looked up again and ages out.

use std::cell::RefCell;
use std::collections::VecDeque;

use remus_math::det_hash::DetHashMap;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use crate::measure::face_content_words;

/// Default retained crossing counts per thread once enabled.
pub const DEFAULT_CROSSING_MEMO_CAPACITY: usize = 65_536;

/// Default retained face contents per thread once enabled.
pub const DEFAULT_CROSSING_MEMO_FACE_CAPACITY: usize = 4_096;

/// Default bound on the retained face-content words, in bytes.
pub const DEFAULT_CROSSING_MEMO_FACE_BYTE_BUDGET: usize = 8 * 1024 * 1024;

/// Counters and bounds of a thread's crossing memo.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CrossingMemoStats {
    /// Counts answered from the memo.
    pub hits: u64,
    /// Counts computed and stored.
    pub misses: u64,
    /// Counts held.
    pub len: usize,
    /// Most counts held.
    pub capacity: usize,
    /// Face contents held.
    pub faces: usize,
}

/// One stored count's key: the interned face content and the ray's bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct CountKey {
    face: u64,
    ray: [u64; 6],
}

impl CountKey {
    fn new(face: u64, origin: Point3, direction: Vec3) -> Self {
        Self {
            face,
            ray: [
                origin.x().to_bits(),
                origin.y().to_bits(),
                origin.z().to_bits(),
                direction.x().to_bits(),
                direction.y().to_bits(),
                direction.z().to_bits(),
            ],
        }
    }
}

#[derive(Debug)]
struct Memo {
    counts: DetHashMap<CountKey, u32>,
    count_order: VecDeque<CountKey>,
    capacity: usize,
    contents: DetHashMap<Vec<u64>, u64>,
    content_order: VecDeque<Vec<u64>>,
    content_bytes: usize,
    face_capacity: usize,
    face_byte_budget: usize,
    next_id: u64,
    hits: u64,
    misses: u64,
}

impl Memo {
    fn new(capacity: usize, face_capacity: usize, face_byte_budget: usize) -> Self {
        Self {
            counts: DetHashMap::default(),
            count_order: VecDeque::new(),
            capacity,
            contents: DetHashMap::default(),
            content_order: VecDeque::new(),
            content_bytes: 0,
            face_capacity,
            face_byte_budget,
            next_id: 0,
            hits: 0,
            misses: 0,
        }
    }

    const fn enabled(&self) -> bool {
        self.capacity > 0 && self.face_capacity > 0 && self.face_byte_budget > 0
    }

    fn content_id(&mut self, words: Vec<u64>) -> Option<u64> {
        if let Some(&id) = self.contents.get(&words) {
            return Some(id);
        }
        let bytes = words_bytes(&words);
        if bytes > self.face_byte_budget {
            return None;
        }
        let id = self.next_id;
        self.next_id += 1;
        self.contents.insert(words.clone(), id);
        self.content_order.push_back(words);
        self.content_bytes += bytes;
        self.enforce_limits();
        Some(id)
    }

    fn enforce_limits(&mut self) {
        while self.counts.len() > self.capacity {
            let Some(oldest) = self.count_order.pop_front() else {
                break;
            };
            self.counts.remove(&oldest);
        }
        while self.contents.len() > self.face_capacity || self.content_bytes > self.face_byte_budget
        {
            let Some(oldest) = self.content_order.pop_front() else {
                break;
            };
            self.content_bytes -= words_bytes(&oldest);
            self.contents.remove(&oldest);
        }
    }

    fn stats(&self) -> CrossingMemoStats {
        CrossingMemoStats {
            hits: self.hits,
            misses: self.misses,
            len: self.counts.len(),
            capacity: self.capacity,
            faces: self.contents.len(),
        }
    }
}

/// Retained bytes of one interned content: the map's and the queue's copy.
const fn words_bytes(words: &[u64]) -> usize {
    2 * std::mem::size_of_val(words)
}

thread_local! {
    static MEMO: RefCell<Option<Memo>> = const { RefCell::new(None) };
}

/// Initialize this thread's memo with default bounds when no instance exists.
///
/// Existing bounds, entries and counters are preserved.
pub fn enable_thread_crossing_memo() {
    MEMO.with(|cell| {
        if let Ok(mut slot) = cell.try_borrow_mut() {
            slot.get_or_insert_with(|| {
                Memo::new(
                    DEFAULT_CROSSING_MEMO_CAPACITY,
                    DEFAULT_CROSSING_MEMO_FACE_CAPACITY,
                    DEFAULT_CROSSING_MEMO_FACE_BYTE_BUDGET,
                )
            });
        }
    });
}

/// Set this thread's memo bounds, evicting oldest entries immediately to fit.
///
/// A zero bound disables lookups and retention without discarding counters.
pub fn set_thread_crossing_memo_limits(
    capacity: usize,
    face_capacity: usize,
    face_byte_budget: usize,
) {
    MEMO.with(|cell| {
        if let Ok(mut slot) = cell.try_borrow_mut() {
            let memo =
                slot.get_or_insert_with(|| Memo::new(capacity, face_capacity, face_byte_budget));
            memo.capacity = capacity;
            memo.face_capacity = face_capacity;
            memo.face_byte_budget = face_byte_budget;
            memo.enforce_limits();
        }
    });
}

/// Disable this thread's memo and release its entries and counters.
pub fn disable_thread_crossing_memo() {
    MEMO.with(|cell| {
        if let Ok(mut slot) = cell.try_borrow_mut() {
            *slot = None;
        }
    });
}

/// Return counters and bounds, or `None` without an available memo instance.
#[must_use]
pub fn thread_crossing_memo_stats() -> Option<CrossingMemoStats> {
    MEMO.with(|cell| {
        cell.try_borrow()
            .ok()
            .and_then(|slot| slot.as_ref().map(Memo::stats))
    })
}

/// The interned content id of every non-planar face of one solid, for the
/// classifier to consult this thread's memo with.
#[derive(Debug)]
pub struct SolidCrossingMemo {
    ids: DetHashMap<FaceId, u64>,
}

impl SolidCrossingMemo {
    /// Intern `solid`'s non-planar faces, or `None` while the memo is off (no
    /// key is built then).
    pub fn for_solid(topo: &Topology, solid: SolidId) -> Option<Self> {
        let enabled = MEMO.with(|cell| {
            cell.try_borrow()
                .ok()
                .is_some_and(|slot| slot.as_ref().is_some_and(Memo::enabled))
        });
        if !enabled {
            return None;
        }
        let faces = remus_topology::explorer::solid_faces(topo, solid).ok()?;
        let mut ids = DetHashMap::default();
        for face in faces {
            let Ok(data) = topo.face(face) else {
                continue;
            };
            if matches!(data.surface(), FaceSurface::Plane { .. }) {
                continue;
            }
            let Some(words) = face_content_words(topo, face) else {
                continue;
            };
            let id = MEMO.with(|cell| {
                cell.try_borrow_mut()
                    .ok()
                    .and_then(|mut slot| slot.as_mut().and_then(|memo| memo.content_id(words)))
            });
            if let Some(id) = id {
                ids.insert(face, id);
            }
        }
        Some(Self { ids })
    }
}

impl remus_check::classify::FaceCrossingMemo for SolidCrossingMemo {
    fn crossings(&self, face: FaceId, origin: Point3, direction: Vec3) -> Option<u32> {
        let &id = self.ids.get(&face)?;
        let key = CountKey::new(id, origin, direction);
        MEMO.with(|cell| {
            let mut slot = cell.try_borrow_mut().ok()?;
            let memo = slot.as_mut().filter(|memo| memo.enabled())?;
            let count = memo.counts.get(&key).copied()?;
            memo.hits += 1;
            Some(count)
        })
    }

    fn record(&self, face: FaceId, origin: Point3, direction: Vec3, crossings: u32) {
        let Some(&id) = self.ids.get(&face) else {
            return;
        };
        let key = CountKey::new(id, origin, direction);
        MEMO.with(|cell| {
            if let Ok(mut slot) = cell.try_borrow_mut()
                && let Some(memo) = slot.as_mut().filter(|memo| memo.enabled())
            {
                memo.misses += 1;
                if memo.counts.insert(key, crossings).is_none() {
                    memo.count_order.push_back(key);
                    memo.enforce_limits();
                }
            }
        });
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use remus_math::mat::Mat4;
    use remus_topology::Topology;
    use remus_topology::face::FaceSurface;
    use remus_topology::solid::SolidId;

    use super::*;
    use crate::boolean::{BooleanOp, boolean};
    use crate::feature_recognition::recognize_features;
    use crate::primitives::{make_box, make_cylinder};
    use crate::push_pull::move_faces;
    use crate::query::solid_edge_relations;
    use crate::transform::transform_solid;

    /// Disables this thread's memo when dropped, so a failing test leaves
    /// none behind.
    struct Reset;
    impl Drop for Reset {
        fn drop(&mut self) {
            disable_thread_crossing_memo();
        }
    }

    /// A 20 x 12 x 6 block drilled through Z twice.
    fn drilled_block(topo: &mut Topology) -> SolidId {
        let mut solid = make_box(topo, 20.0, 12.0, 6.0).unwrap();
        for (x, y) in [(5.0, 6.0), (12.0, 6.0)] {
            let tool = make_cylinder(topo, 2.0, 10.0).unwrap();
            transform_solid(topo, tool, &Mat4::translation(x, y, -2.0)).unwrap();
            solid = boolean(topo, BooleanOp::Cut, solid, tool).unwrap();
        }
        solid
    }

    /// The block's +X side, far from both holes.
    fn far_side(topo: &Topology, solid: SolidId) -> FaceId {
        remus_topology::explorer::solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .find(|&face| {
                matches!(topo.face(face).unwrap().surface(),
                    FaceSurface::Plane { normal, d } if normal.x() > 0.99 && *d > 19.0)
            })
            .unwrap()
    }

    /// Both consumers' full output, as their `Debug` text.
    fn analysis(topo: &Topology, solid: SolidId) -> String {
        format!(
            "{:#?}\n{:#?}",
            solid_edge_relations(topo, solid, None).unwrap(),
            recognize_features(topo, solid, 0.08).unwrap()
        )
    }

    #[test]
    fn remembered_crossings_reproduce_the_analysis_across_an_edit() {
        let _reset = Reset;
        disable_thread_crossing_memo();
        let mut topo = Topology::new();
        let solid = drilled_block(&mut topo);
        let face = far_side(&topo, solid);
        let moved = move_faces(&mut topo, solid, &[face], -3.0).unwrap();
        let expected = [analysis(&topo, solid), analysis(&topo, moved)];
        assert!(thread_crossing_memo_stats().is_none());

        enable_thread_crossing_memo();
        assert_eq!(analysis(&topo, solid), expected[0], "cold");
        let cold = thread_crossing_memo_stats().unwrap();
        assert!(cold.misses > 0);
        // The two bores are untouched by the move, so their faces keep their
        // contents and the probes at their rims meet the same rays.
        assert_eq!(analysis(&topo, moved), expected[1], "after the edit");
        let edited = thread_crossing_memo_stats().unwrap();
        assert!(edited.hits > cold.hits);
        assert_eq!(analysis(&topo, moved), expected[1], "warm");
        let warm = thread_crossing_memo_stats().unwrap();
        assert_eq!(warm.misses, edited.misses);
        assert!(warm.hits > edited.hits);
    }

    #[test]
    fn evicting_memo_still_reproduces_the_analysis() {
        let _reset = Reset;
        disable_thread_crossing_memo();
        let mut topo = Topology::new();
        let solid = drilled_block(&mut topo);
        let expected = analysis(&topo, solid);

        set_thread_crossing_memo_limits(4, 1, 1 << 20);
        for _ in 0..2 {
            assert_eq!(analysis(&topo, solid), expected);
            let stats = thread_crossing_memo_stats().unwrap();
            assert!(stats.len <= 4 && stats.faces <= 1);
        }
        // A face too large for the content budget is never interned.
        set_thread_crossing_memo_limits(4, 1, 8);
        assert_eq!(analysis(&topo, solid), expected);
        assert_eq!(thread_crossing_memo_stats().unwrap().faces, 0);
    }
}
