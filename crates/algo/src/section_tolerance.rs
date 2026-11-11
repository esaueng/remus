//! The length tolerances a section's vertices on a planar face are held to,
//! stated once for every stage that reads them.
//!
//! A section of a planar face ends at vertices on the face's loops. Four
//! stages decide whether two such points are one vertex, and they have to
//! agree, or the faces sharing the section get different vertex sets:
//!
//! | Stage | Takes two points for one within |
//! |---|---|
//! | Phase FF (which crossings exist, which spans are kept) | [`SectionTolerance::band`] |
//! | Face splitter, boundary split and arrangement endpoint weld | [`split_weld`], at most [`SPLIT_WELD_FACTOR`] × linear |
//! | Shell assembly (`weld_coincident_vertices`) | [`VERTEX_WELD_FACTOR`] × [`VERTEX_TOL`] |
//! | Validator (a vertex on its edge and face) | the vertex tolerance, [`VERTEX_TOL`] |
//!
//! All four are absolute lengths, as the kernel's linear tolerance is. The
//! model has one scale term, the magnitude of the coordinates a crossing is
//! computed in, which only matters once their rounding reaches the linear
//! tolerance:
//!
//! - `band = max(linear, vertex tolerance, 32 ε · coord)`. Nearer than this,
//!   a point cannot be told from lying on a curve. A probe is certified on one side of a loop
//!   only farther than the band from it; two crossings within it are the
//!   same point.
//! - `weld = 10 · band`: the distance at which assembly welds two vertices.
//! - `separate = 100 · band`: the least distance between two DISTINCT
//!   vertices phase FF may put on a planar face. It is the largest weld any
//!   later stage applies ([`split_weld`] never exceeds `100 · linear`), so
//!   two vertices phase FF keeps apart are kept apart by every stage after
//!   it. Vertices between `band` and `separate` apart are never emitted: the
//!   section is refused by name.
//!
//! | Model scale (coordinates) | `band` | `weld` | `separate` | splitter weld cap | assembly weld | vertex tolerance |
//! |---|---|---|---|---|---|---|
//! | 1e-3 (≤ 1) | 1e-7 | 1e-6 | 1e-5 | 1e-5 | 1e-6 | 1e-7 |
//! | 1 (≤ 1e2) | 1e-7 | 1e-6 | 1e-5 | 1e-5 | 1e-6 | 1e-7 |
//! | 1e6 (≤ 1.4e7) | 1e-7 | 1e-6 | 1e-5 | 1e-5 | 1e-6 | 1e-7 |
//! | 1e6, far from the origin (5.5e7) | 3.9e-7 | 3.9e-6 | 3.9e-5 | 1e-5 | 1e-6 | 1e-7 |
//!
//! In every row `vertex tolerance ≤ band`, `assembly weld ≤ weld` and
//! `splitter weld ≤ separate`, which is the consistency the stages need:
//! nothing downstream merges what phase FF separated, and nothing
//! downstream separates what phase FF took for one point (two crossings
//! within `band` become one vertex before any other stage sees them).
//!
//! # Every weld after phase FF
//!
//! The separation distance has to dominate every distance at which a later
//! stage takes two vertices for one, at the caller's linear tolerance `L`
//! as well as the default. Some of those welds are fixed lengths that do
//! not follow `L`, so the band is floored at the vertex tolerance: a caller
//! asking for `L = 1e-10` does not get a separation of `1e-8`, under
//! assembly's fixed weld.
//!
//! | Stage | Takes two points for one within | Largest value |
//! |---|---|---|
//! | Face splitter, boundary acceptance and dedup; arrangement weld | [`split_weld`]: `clamp(1 % of the edge or loop, L, 100 L)` | `100 L` |
//! | Face splitter, exact hole split ([`exact_line_splits`]) | [`VERTEX_WELD_FACTOR`] times `L` for one vertex; refuses under [`split_weld`] | `100 L` |
//! | Assembly, position cells (`MERGE_TOL`) and vertex weld | [`VERTEX_TOL`]; [`VERTEX_WELD_FACTOR`] times that at a vertex on a spline | [`ASSEMBLY_WELD`] = 1e-6, fixed |
//! | Assembly, straight-edge refinement | search within [`ASSEMBLY_WELD`], accepted on the line within [`VERTEX_TOL`] | 1e-6, fixed |
//! | Operations, `merge_result_vertices` | rounded cells `L` wide | `√3 · L` |
//! | Operations, open-result repair | rounded cells `max(10 L, 1e-6)` wide | `√3 · max(10 L, 1e-6)` ([`repair_weld`]) |
//! | Healing (optional) | the linear tolerance it is given | `L` |
//!
//! With `band ≥ max(L, VERTEX_TOL)`, `separate = 100 · band` is at least
//! `100 L` and at least 1e-5, which is every entry of the last column.
//! `every_later_weld_is_within_the_separation_distance` asserts it for `L`
//! from 1e-10 to 1e-4.

use remus_math::compact_sort::CompactSort;
use remus_math::vec::Point3;

/// The kernel's linear tolerance: a vertex's own tolerance, the cell size of
/// assembly's position quantisation (`MERGE_TOL`), and the floor of every
/// length in this model.
pub(crate) const VERTEX_TOL: f64 = 1e-7;

/// Assembly welds two vertices closer than this many vertex tolerances.
pub(crate) const VERTEX_WELD_FACTOR: f64 = 10.0;

/// No weld of the face splitter exceeds this many linear tolerances.
pub(crate) const SPLIT_WELD_FACTOR: f64 = 100.0;

/// The largest distance at which shell assembly takes two vertices for one,
/// whatever the caller's linear tolerance: its weld at a vertex on a spline
/// and the search radius of its straight-edge refinement.
pub(crate) const ASSEMBLY_WELD: f64 = VERTEX_WELD_FACTOR * VERTEX_TOL;

/// The largest distance at which the operations layer's open-result repair
/// takes two vertices for one at linear tolerance `linear`: the diagonal of
/// its rounding cell, `max(10 · linear, 1e-6)` wide. (Its vertex merge uses
/// cells `linear` wide, and healing the tolerance it is given: both are
/// under this.) Stated here, beside the distances it must stay under,
/// because nothing in this crate can read it off the operations crate.
#[cfg(test)]
pub(crate) fn repair_weld(linear: f64) -> f64 {
    3.0_f64.sqrt() * (10.0 * linear).max(1e-6)
}

/// The weld the face splitter applies to anchors on a boundary of size
/// `extent` (an edge's length, or a loop's diagonal): one percent of it,
/// kept between the linear tolerance and [`SPLIT_WELD_FACTOR`] times it.
pub(crate) fn split_weld(linear: f64, extent: f64) -> f64 {
    (linear * SPLIT_WELD_FACTOR).min(extent * 0.01).max(linear)
}

/// How far from a straight boundary edge an anchor may lie and still be a
/// point of it, for a split that must land exactly where phase FF put the
/// vertex: twice the linear tolerance (the anchor may have been snapped to
/// an existing vertex within one tolerance of the edge).
pub(crate) fn on_edge_band(linear: f64) -> f64 {
    2.0 * linear
}

/// Two split points of one boundary edge (or a split point and an end of
/// the edge) closer than the weld the arrangement applies to it: the pair
/// and that weld.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct CloseAnchors {
    pub(crate) first: Point3,
    pub(crate) second: Point3,
    pub(crate) weld: f64,
}

/// Where the straight boundary edge `start → end` of a planar face's HOLE
/// is split by `anchors`, the ends of the sections that reach the hole:
/// the fraction along the edge and the foot of each split, in order.
///
/// This is the one rule both sides read. The face splitter splits a hole
/// by it; phase FF runs it over the ends of the spans it keeps and
/// requires as many splits on each hole edge as it certified crossings
/// there, so the two cannot disagree about a hole's vertices.
///
/// An anchor splits the edge only when it is a point of it (within
/// [`on_edge_band`]). One within [`VERTEX_WELD_FACTOR`] tolerances of an
/// end of the edge is that end's vertex, and anchors that close to one
/// another are one split: assembly welds vertices at that distance whatever
/// is done here.
///
/// # Errors
///
/// [`CloseAnchors`] when two distinct splits, or a split and an end of the
/// edge, are closer than [`split_weld`] for the edge's length: the
/// arrangement would merge them.
pub(crate) fn exact_line_splits(
    start: Point3,
    end: Point3,
    anchors: &[Point3],
    linear: f64,
) -> Result<Vec<(f64, Point3)>, CloseAnchors> {
    let direction = end - start;
    let length = direction.length();
    if length < linear {
        return Ok(Vec::new());
    }
    let one_vertex = VERTEX_WELD_FACTOR * linear;
    let mut splits: Vec<(f64, Point3)> = Vec::new();
    for &anchor in anchors {
        let t = (anchor - start).dot(direction) / (length * length);
        if t * length <= one_vertex || (1.0 - t) * length <= one_vertex {
            continue;
        }
        let foot = start + direction * t;
        if (anchor - foot).length() <= on_edge_band(linear) {
            splits.push((t, foot));
        }
    }
    splits.compact_sort_by(|a, b| a.0.total_cmp(&b.0));
    splits.dedup_by(|a, b| (a.1 - b.1).length() <= one_vertex);
    let weld = split_weld(linear, length);
    let mut previous = start;
    // The last split is held to the end of the edge too.
    let closing = splits.last().map(|_| end);
    for point in splits.iter().map(|split| split.1).chain(closing) {
        if (point - previous).length() < weld {
            return Err(CloseAnchors {
                first: previous,
                second: point,
                weld,
            });
        }
        previous = point;
    }
    Ok(splits)
}

/// The lengths phase FF certifies a planar face's section with. See the
/// module documentation for the model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SectionTolerance {
    /// Nearer than this, a point cannot be told from lying on a curve.
    pub(crate) band: f64,
    /// Assembly welds two vertices closer than this.
    pub(crate) weld: f64,
    /// Two distinct section vertices on a planar face are farther apart
    /// than this.
    pub(crate) separate: f64,
}

impl SectionTolerance {
    /// The model at linear tolerance `linear` for coordinates of magnitude
    /// `coord`.
    ///
    /// The band is never under the vertex tolerance: assembly's welds are
    /// fixed lengths ([`ASSEMBLY_WELD`]) that a finer `linear` does not
    /// shrink, and the separation distance has to stay over them.
    pub(crate) fn at(linear: f64, coord: f64) -> Self {
        let band = linear
            .max(VERTEX_TOL)
            .max(32.0 * f64::EPSILON * coord.abs());
        Self {
            band,
            weld: VERTEX_WELD_FACTOR * band,
            separate: SPLIT_WELD_FACTOR * band,
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;

    /// The stages agree at model scales 1e-3, 1 and 1e6: the vertex
    /// tolerance is within the band, assembly's weld within the model's,
    /// and no splitter weld reaches the separation distance.
    #[test]
    fn the_stages_agree_at_every_model_scale() {
        let linear = remus_math::tolerance::Tolerance::new().linear;
        assert!((linear - VERTEX_TOL).abs() <= f64::EPSILON);
        for scale in [1e-3_f64, 1.0, 1e6] {
            // Coordinates up to ten model sizes from the origin, and (the
            // last) a model far from it.
            for coord in [scale, 10.0 * scale, 55.0 * scale] {
                let model = SectionTolerance::at(linear, coord);
                assert!(model.band >= VERTEX_TOL, "scale {scale}");
                assert!(model.band >= 32.0 * f64::EPSILON * coord, "scale {scale}");
                assert!(
                    VERTEX_WELD_FACTOR * VERTEX_TOL <= model.weld,
                    "scale {scale}"
                );
                assert!(model.weld < model.separate, "scale {scale}");
                // Edges and loops from a thousandth of the model to a
                // thousand times it.
                for extent in [1e-3 * scale, scale, 1e3 * scale, f64::INFINITY] {
                    let weld = split_weld(linear, extent);
                    assert!(
                        weld <= model.separate,
                        "scale {scale} extent {extent}: splitter weld {weld} merges \
                         vertices {} apart",
                        model.separate
                    );
                    assert!(weld >= linear);
                }
                assert!(on_edge_band(linear) < model.weld);
            }
        }
        // The values the table in the module documentation states.
        let near = SectionTolerance::at(linear, 1.0e7);
        for (value, stated) in [(near.band, 1e-7), (near.weld, 1e-6), (near.separate, 1e-5)] {
            assert!(
                (value - stated).abs() <= 1e-9 * stated,
                "{value} vs {stated}"
            );
        }
        let far = SectionTolerance::at(linear, 5.5e7);
        assert!((far.band - 3.9e-7).abs() < 1e-8, "{}", far.band);
    }

    /// The separation distance dominates every weld a later stage applies,
    /// at the caller's linear tolerance as well as the default: the face
    /// splitter's, assembly's fixed ones, and the operations layer's vertex
    /// merge, open-result repair and healing.
    #[test]
    fn every_later_weld_is_within_the_separation_distance() {
        for linear in [1e-10_f64, 1e-9, 1e-8, 1e-7, 1e-6, 1e-5, 1e-4] {
            for coord in [1e-3_f64, 1.0, 1e3, 1e6, 5.5e7] {
                let model = SectionTolerance::at(linear, coord);
                let case = format!("linear {linear:e}, coordinates {coord:e}");
                assert!(model.band >= linear.max(VERTEX_TOL), "{case}");
                assert!(model.weld >= ASSEMBLY_WELD, "{case}");
                for extent in [1e-6 * coord, coord, 1e3 * coord, f64::INFINITY] {
                    assert!(split_weld(linear, extent) <= model.separate, "{case}");
                }
                // Assembly, whatever the caller's tolerance.
                assert!(ASSEMBLY_WELD < model.separate, "{case}");
                // Operations: vertex merge, open-result repair, healing.
                assert!(3.0_f64.sqrt() * linear < model.separate, "{case}");
                assert!(repair_weld(linear) < model.separate, "{case}");
                assert!(linear < model.separate, "{case}");
                // The exact hole split takes anchors within this of one
                // another for one vertex; two distinct crossings are
                // farther apart.
                assert!(VERTEX_WELD_FACTOR * linear < model.separate, "{case}");
                assert!(on_edge_band(linear) <= 2.0 * model.band, "{case}");
            }
        }
        // A finer caller tolerance does not shrink the separation under
        // assembly's fixed weld: before the floor it was 1e-8 here.
        let fine = SectionTolerance::at(1e-10, 1.0);
        assert!((fine.separate - 1e-5).abs() <= 1e-9 * 1e-5, "{fine:?}");
        // A coarser one widens it with the splitter's weld.
        let coarse = SectionTolerance::at(1e-5, 1.0);
        assert!((coarse.separate - 1e-3).abs() <= 1e-9 * 1e-3, "{coarse:?}");
    }

    /// The splitter's rule for a hole edge: a point of the edge splits it
    /// where it is, one off it or at an end does not, the same point twice
    /// is one split, and two splits (or a split and an end) closer than the
    /// edge's weld are an error rather than one split.
    #[test]
    fn a_hole_edge_is_split_exactly_or_not_at_all() {
        let linear = VERTEX_TOL;
        let p = |x: f64, y: f64| Point3::new(x, y, 0.0);
        let (start, end) = (p(0.0, 0.0), p(4.0, 0.0));
        let splits = |anchors: &[Point3]| exact_line_splits(start, end, anchors, linear);

        let found = splits(&[p(3.0, 0.0), p(1.0, 1e-7), p(1.0, 0.0)]).unwrap();
        assert_eq!(found.len(), 2);
        assert!((found[0].0 - 0.25).abs() < 1e-15 && (found[1].0 - 0.75).abs() < 1e-15);
        assert!((found[0].1 - p(1.0, 0.0)).length() < 1e-15);

        // Off the edge by more than two tolerances, at its ends, past them.
        for anchor in [
            p(1.0, 3e-7),
            p(0.0, 0.0),
            p(4.0, 0.0),
            p(5.0, 0.0),
            p(5e-7, 0.0),
        ] {
            assert!(splits(&[anchor]).unwrap().is_empty(), "{anchor:?}");
        }
        // Two splits 3e-6 apart, and one 3e-6 from an end: the weld on a
        // 4-long edge is 1e-5.
        for anchors in [
            vec![p(1.0, 0.0), p(1.000_003, 0.0)],
            vec![p(3e-6, 0.0)],
            vec![p(4.0 - 3e-6, 0.0)],
        ] {
            let close = splits(&anchors).expect_err("vertices closer than the weld");
            assert!(
                (close.weld - 1e-5).abs() < 1e-18 && (close.first - close.second).length() < 1e-5
            );
        }
        // 2e-5 apart they are two.
        assert_eq!(splits(&[p(1.0, 0.0), p(1.000_02, 0.0)]).unwrap().len(), 2);
        // On a short edge the weld is a hundredth of its length: 3e-6
        // apart on an edge 1e-4 long (weld 1e-6) is two splits.
        let short = exact_line_splits(start, p(1e-4, 0.0), &[p(4e-5, 0.0), p(4.3e-5, 0.0)], linear);
        assert_eq!(short.unwrap().len(), 2);
    }
}
