//! Builder — splits faces and classifies sub-faces for boolean assembly.
//!
//! Takes the PaveFiller's output ([`GfaArena`] with pave blocks, face info,
//! and intersection curves) and produces classified sub-faces ready for
//! boolean operation selection.
//!
//! # Flow
//!
//! 1. **`fill_images`** — map original edges to their split images
//! 2. **`fill_images_faces`** — build sub-faces from face info
//! 3. **`same_domain`** — detect coplanar face pairs
//! 4. **`classify_sub_faces`** — classify each sub-face as IN/OUT
//!
//! [`GfaArena`]: crate::ds::GfaArena

pub mod assemble;
pub mod builder_solid;
pub mod classify_2d;
pub mod face_class;
pub mod face_splitter;
pub mod fill_images;
pub mod fill_images_faces;
pub mod pcurve_compute;
pub mod plane_frame;
pub mod same_domain;
pub mod split_types;
pub mod wire_builder;

pub use face_class::FaceClass;

use std::collections::HashMap;
use std::fmt::Write as _;

use remus_math::tolerance::Tolerance;

use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

use crate::bop::{self, BooleanOp};
use crate::classifier;
use crate::ds::{GfaArena, Rank};
use crate::error::AlgoError;

/// Provenance of result faces: for each face, the input source face it was
/// derived from, or `None` for a synthesised face. Face IDs are store-local.
pub type FaceProvenance = Vec<(FaceId, Option<FaceId>)>;

/// A sub-face produced by the Builder after splitting.
#[derive(Debug, Clone)]
pub struct SubFace {
    /// The face entity in topology (same as parent if no split occurred).
    pub face_id: FaceId,
    /// The original input face this sub-face was derived from. Provenance for
    /// shape evolution — every sub-face traces back to one input argument face.
    pub source_face: FaceId,
    /// Classification relative to the opposing solid.
    pub classification: FaceClass,
    /// Which boolean argument this face came from.
    pub rank: Rank,
    /// Pre-computed interior sample point for classification.
    /// When `Some`, the classifier uses this instead of sampling from face geometry.
    /// Set by the face splitter for split sub-faces.
    pub interior_point: Option<Point3>,
}

/// `BK_SUBFACE_BOX=x0,x1,y0,y1,z0,z1`: report every sub-face whose vertices
/// touch that box, with its surface kind, classification and selection.
///
/// The decisive probe for a missing result face: it separates "the splitter
/// never produced a sub-face here" from "it did, and selection dropped it".
/// Those need opposite fixes, so guessing between them wastes a whole dig.
/// `BK_SUBFACE_SRC=<n>`: total the sub-faces produced from ONE source face and
/// compare against the source's own area.
///
/// A boolean emits only trimmed patches of input surfaces, so a source face's
/// pieces must tile it. When they do not, a region of the result simply has no
/// face and the shell comes back open — which reads downstream as a selection
/// or classification bug and is neither.
fn log_source_face_partition(topo: &Topology, subs: &[SubFace], selected: &[bop::SelectedFace]) {
    let Ok(want) = std::env::var("BK_SUBFACE_SRC") else {
        return;
    };
    // `BK_SUBFACE_SRC=all` scans every source face instead of one, reporting
    // only those whose pieces fail to tile them — the cheap way to find a
    // missing result face without guessing which source it came from.
    if want.trim() == "all" {
        let chosen: std::collections::HashSet<FaceId> =
            selected.iter().map(|s| s.face_id).collect();
        let mut by_src: std::collections::HashMap<FaceId, (f64, usize, usize)> =
            std::collections::HashMap::new();
        for sf in subs {
            let e = by_src.entry(sf.source_face).or_insert((0.0, 0, 0));
            e.0 += face_area_estimate(topo, sf.face_id);
            e.1 += 1;
            if chosen.contains(&sf.face_id) {
                e.2 += 1;
            }
        }
        let by_src2 = by_src.clone();
        let mut rows: Vec<_> = by_src
            .into_iter()
            .map(|(src, (tot, n, sel))| (face_area_estimate(topo, src) - tot, src, tot, n, sel))
            .filter(|(gap, _, _, _, _)| gap.abs() > 1e-6)
            .collect();
        rows.sort_by(|a, b| b.0.abs().total_cmp(&a.0.abs()));
        // A source whose pieces TILE it but where NONE was selected leaves a
        // hole in the result boundary just as surely as an under-partition, and
        // an area-gap scan cannot see it — the pieces all exist.
        let mut dropped: Vec<_> = by_src2
            .iter()
            .filter(|(_, (_, _, sel))| *sel == 0)
            .map(|(src, (tot, n, _))| (*src, *tot, *n))
            .collect();
        dropped.sort_by(|a, b| b.1.total_cmp(&a.1));
        log::debug!(
            "SRCPART scan: {} source faces do not tile, {} fully dropped (no piece selected)",
            rows.len(),
            dropped.len()
        );
        for (src, tot, n) in dropped.iter().take(12) {
            log::debug!("SRCPART fully-dropped src={src:?} pieces={n} area={tot:.6}");
        }
        for (gap, src, tot, n, sel) in rows.iter().take(12) {
            log::debug!(
                "SRCPART gap={gap:.6} src={src:?} pieces={n} selected={sel} pieceTotal={tot:.6}"
            );
        }
        return;
    }
    let Ok(want) = want.trim().parse::<usize>() else {
        return;
    };
    let chosen: std::collections::HashSet<FaceId> = selected.iter().map(|s| s.face_id).collect();
    let mut total = 0.0;
    let mut n = 0;
    let mut src_id: Option<FaceId> = None;
    for sf in subs {
        if sf.source_face.index() != want {
            continue;
        }
        src_id = Some(sf.source_face);
        let a = face_area_estimate(topo, sf.face_id);
        total += a;
        n += 1;
        log::debug!(
            "SRCPART sub {:?} area={a:.6} class={:?} selected={}",
            sf.face_id,
            sf.classification,
            chosen.contains(&sf.face_id)
        );
    }
    let src_area = src_id.map_or(f64::NAN, |f| face_area_estimate(topo, f));
    log::debug!(
        "SRCPART source Id({want}) area={src_area:.6} pieces={n} pieceTotal={total:.6}          uncovered={:.6}",
        src_area - total
    );
}

/// Fan-triangulated area of a face's outer wire, minus its inner wires. Good
/// enough to compare a partition against its source; exact for planar faces.
fn face_area_estimate(topo: &Topology, fid: FaceId) -> f64 {
    let Ok(face) = topo.face(fid) else {
        return 0.0;
    };
    let ring = |wid| -> f64 {
        let Ok(w) = topo.wire(wid) else { return 0.0 };
        let mut pts: Vec<remus_math::vec::Point3> = Vec::new();
        for oe in w.edges() {
            let Ok(e) = topo.edge(oe.edge()) else {
                continue;
            };
            let vid = if oe.is_forward() { e.start() } else { e.end() };
            if let Ok(v) = topo.vertex(vid) {
                pts.push(v.point());
            }
        }
        if pts.len() < 3 {
            return 0.0;
        }
        let mut acc = remus_math::vec::Vec3::new(0.0, 0.0, 0.0);
        for i in 1..pts.len() - 1 {
            let u = pts[i] - pts[0];
            let v = pts[i + 1] - pts[0];
            acc += u.cross(v);
        }
        acc.length() * 0.5
    };
    let outer = ring(face.outer_wire());
    let inner: f64 = face.inner_wires().iter().map(|w| ring(*w)).sum();
    (outer - inner).max(0.0)
}

fn log_subfaces_in_box(
    topo: &Topology,
    subs: &[SubFace],
    selected: &[bop::SelectedFace],
) -> Result<(), AlgoError> {
    let Ok(spec) = std::env::var("BK_SUBFACE_BOX") else {
        return Ok(());
    };
    let v: Vec<f64> = spec
        .split(',')
        .filter_map(|t| t.trim().parse().ok())
        .collect();
    if v.len() != 6 {
        log::debug!("BK_SUBFACE_BOX needs x0,x1,y0,y1,z0,z1");
        return Ok(());
    }
    let (lo, hi) = ([v[0], v[2], v[4]], [v[1], v[3], v[5]]);
    let chosen: std::collections::HashSet<FaceId> = selected.iter().map(|s| s.face_id).collect();
    for sf in subs {
        let Ok(f) = topo.face(sf.face_id) else {
            continue;
        };
        let mut touches = false;
        let mut flo = [f64::MAX; 3];
        let mut fhi = [f64::MIN; 3];
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            let Ok(w) = topo.wire(wid) else { continue };
            for oe in w.edges() {
                let Ok(e) = topo.edge(oe.edge()) else {
                    continue;
                };
                for vid in [e.start(), e.end()] {
                    let Ok(vtx) = topo.vertex(vid) else { continue };
                    let p = vtx.point();
                    let c = [p.x(), p.y(), p.z()];
                    for k in 0..3 {
                        flo[k] = flo[k].min(c[k]);
                        fhi[k] = fhi[k].max(c[k]);
                    }
                    if c.into_iter()
                        .enumerate()
                        .all(|(k, v)| v >= lo[k] && v <= hi[k])
                    {
                        touches = true;
                    }
                }
            }
        }
        if touches {
            log::debug!(
                "SUBFACE {:?} {} rev={} src={:?} class={:?} rank={:?} selected={} ip={} x[{:.3},{:.3}] y[{:.3},{:.3}] z[{:.3},{:.3}]",
                sf.face_id,
                f.surface().type_tag(),
                f.is_reversed(),
                sf.source_face,
                sf.classification,
                sf.rank,
                chosen.contains(&sf.face_id),
                sf.interior_point.map_or_else(
                    || "none".to_string(),
                    |p| format!("({:.3},{:.3},{:.3})", p.x(), p.y(), p.z())
                ),
                flo[0],
                fhi[0],
                flo[1],
                fhi[1],
                flo[2],
                fhi[2]
            );
            if std::env::var("BK_SUBFACE_VERTS").is_ok() {
                for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
                    let Ok(w) = topo.wire(wid) else { continue };
                    let mut s = String::new();
                    for oe in w.edges() {
                        let Ok(e) = topo.edge(oe.edge()) else {
                            continue;
                        };
                        let (Ok(a), Ok(b)) = (topo.vertex(e.start()), topo.vertex(e.end())) else {
                            continue;
                        };
                        let (p, q) = (a.point(), b.point());
                        let (d0, d1) = e.strict_domain().map_err(|error| {
                            AlgoError::AssemblyFailed(format!(
                                "debug sub-face {:?} edge {:?} lacks authoritative parameter range: {error}",
                                sf.face_id,
                                oe.edge()
                            ))
                        })?;
                        let span =
                            if matches!(e.curve(), remus_topology::edge::EdgeCurve::Circle(_)) {
                                format!("{:.1}deg", (d1 - d0).to_degrees())
                            } else {
                                String::new()
                            };
                        let _ = write!(
                            s,
                            " [{:?}{} ({:.3},{:.3},{:.3})->({:.3},{:.3},{:.3}) {}{span}]",
                            oe.edge(),
                            if oe.is_forward() { "+" } else { "-" },
                            p.x(),
                            p.y(),
                            p.z(),
                            q.x(),
                            q.y(),
                            q.z(),
                            e.curve().type_tag()
                        );
                    }
                    log::debug!("   WIRE {wid:?}{s}");
                }
            }
        }
    }
    Ok(())
}

/// Builder — orchestrates face splitting and classification.
///
/// Owns both the `Topology` and `GfaArena`, mutating them as needed.
/// After `perform()`, call `build_result()` to extract the results.
pub struct Builder {
    /// The topology containing both solids (owned, mutable).
    topo: Topology,
    /// GFA transient state from the PaveFiller (owned).
    arena: GfaArena,
    /// First boolean argument.
    solid_a: SolidId,
    /// Second boolean argument.
    solid_b: SolidId,
    /// Geometric tolerance.
    tol: Tolerance,
    /// Sub-faces produced by splitting.
    sub_faces: Vec<SubFace>,
    /// Construction lineage recorded while materializing and assembling
    /// result edges (Issue 12).
    edge_lineage: split_types::EdgeLineageLog,
    /// Map from face ID to its argument rank.
    face_ranks: HashMap<FaceId, Rank>,
    /// Same-domain face pairs detected by `same_domain`.
    sd_pairs: Vec<same_domain::SameDomainPair>,
    /// Within-rank SD duplicates (boolean residue accumulated across
    /// sequential operations — issue #696). Excluded before classification.
    sd_within_rank_dups: Vec<same_domain::WithinRankDuplicate>,
}

impl Builder {
    /// Create a Builder with custom tolerance.
    #[must_use]
    pub fn with_tolerance(
        topo: Topology,
        arena: GfaArena,
        solid_a: SolidId,
        solid_b: SolidId,
        tol: Tolerance,
    ) -> Self {
        Self {
            topo,
            arena,
            solid_a,
            solid_b,
            tol,
            sub_faces: Vec::new(),
            edge_lineage: split_types::EdgeLineageLog::default(),
            face_ranks: HashMap::new(),
            sd_pairs: Vec::new(),
            sd_within_rank_dups: Vec::new(),
        }
    }

    /// Run the Builder pipeline: fill images, split faces, classify.
    ///
    /// # Errors
    ///
    /// Returns [`AlgoError`] if topology lookups or classification fails.
    pub fn perform(&mut self) -> Result<(), AlgoError> {
        self.build_face_ranks()?;
        self.fill_images()?;
        self.classify_sub_faces()?;
        if let Ok(v) = std::env::var("BK_CLS3")
            && let Ok(want) = v.parse::<usize>()
        {
            for (i, sf) in self.sub_faces.iter().enumerate() {
                if sf.source_face.index() == want {
                    log::debug!(
                        "CLS3 idx={i} face={:?} src={:?} rank={:?} class={:?} pt={:?}",
                        sf.face_id,
                        sf.source_face,
                        sf.rank,
                        sf.classification,
                        sf.interior_point
                    );
                }
            }
        }
        Ok(())
    }

    /// Select faces for the given boolean operation and assemble them
    /// into a solid.
    ///
    /// Consumes the Builder, returning the (potentially modified) topology
    /// and the result solid ID.
    ///
    /// # Errors
    ///
    /// Returns [`AlgoError`] if face selection produces no faces or
    /// assembly fails.
    pub fn build_result(mut self, op: BooleanOp) -> Result<(Topology, SolidId), AlgoError> {
        let selected = bop::select_faces(
            &self.sub_faces,
            op,
            &self.sd_pairs,
            &self.sd_within_rank_dups,
        );
        if op == BooleanOp::Fuse {
            orient_selected_fuse_analytic_holes(&mut self.topo, &self.sub_faces, &selected);
        }
        log_subfaces_in_box(&self.topo, &self.sub_faces, &selected)?;
        log_source_face_partition(&self.topo, &self.sub_faces, &selected);
        let cap_planes = self.partial_overlap_cap_planes(&selected);
        let solid_id = assemble::assemble_solid(
            &mut self.topo,
            &selected,
            &cap_planes,
            &mut self.edge_lineage,
        )?;
        Ok((self.topo, solid_id))
    }

    /// Like [`Builder::build_result`], but also returns the provenance of each
    /// result face: `(result face, Some(input source face) | None)`. The source
    /// face IDs are in this builder's (store-local) topology — callers crossing
    /// the GFA shape-store boundary must translate them to caller IDs.
    pub fn build_result_with_origins(
        mut self,
        op: BooleanOp,
    ) -> Result<
        (
            Topology,
            SolidId,
            FaceProvenance,
            split_types::EdgeLineageLog,
        ),
        AlgoError,
    > {
        let selected = bop::select_faces(
            &self.sub_faces,
            op,
            &self.sd_pairs,
            &self.sd_within_rank_dups,
        );
        if op == BooleanOp::Fuse {
            orient_selected_fuse_analytic_holes(&mut self.topo, &self.sub_faces, &selected);
        }
        log_subfaces_in_box(&self.topo, &self.sub_faces, &selected)?;
        log_source_face_partition(&self.topo, &self.sub_faces, &selected);
        let cap_planes = self.partial_overlap_cap_planes(&selected);
        let (solid_id, origins) = assemble::assemble_solid_with_origins(
            &mut self.topo,
            &selected,
            &cap_planes,
            &mut self.edge_lineage,
        )?;
        Ok((self.topo, solid_id, origins, self.edge_lineage))
    }

    /// Candidate cap planes for partial coplanar same-domain overlaps.
    ///
    /// For each planar `geometric_overlap` SD pair whose two faces were BOTH
    /// discarded by `select_faces` (the partial-overlap signature: their
    /// coincident contact is interior, but the larger face's overhang remainder
    /// is exterior and left as free edges), record the larger face's plane so
    /// the assembler can synthesise the missing remainder cap. Coextensive pairs
    /// (one face kept) and non-planar pairs are excluded.
    fn partial_overlap_cap_planes(
        &self,
        selected: &[bop::SelectedFace],
    ) -> Vec<assemble::CapPlane> {
        use remus_topology::face::FaceSurface;
        let kept: std::collections::HashSet<FaceId> = selected.iter().map(|s| s.face_id).collect();
        let mut planes = Vec::new();
        for pair in &self.sd_pairs {
            if !pair.geometric_overlap {
                continue;
            }
            let fa = self.sub_faces[pair.idx_a].face_id;
            let fb = self.sub_faces[pair.idx_b].face_id;
            // Only when both faces were discarded — a kept face already covers
            // the plane and capping would double it.
            if kept.contains(&fa) || kept.contains(&fb) {
                continue;
            }
            let larger = self.sub_faces[pair.representative].face_id;
            let Ok(face) = self.topo.face(larger) else {
                continue;
            };
            let FaceSurface::Plane { normal, d } = *face.surface() else {
                continue;
            };
            let Some(out_normal) = face.effective_plane_normal() else {
                continue;
            };
            planes.push(assemble::CapPlane {
                normal,
                d,
                out_normal,
            });
        }
        planes
    }

    /// Get the sub-faces, SD pairs, and topology for testing.
    #[cfg(test)]
    pub(crate) fn debug_info(&self) -> (&[SubFace], &[same_domain::SameDomainPair], &Topology) {
        (&self.sub_faces, &self.sd_pairs, &self.topo)
    }

    /// Build the face-to-rank mapping from both solids.
    fn build_face_ranks(&mut self) -> Result<(), AlgoError> {
        let faces_a = remus_topology::explorer::solid_faces(&self.topo, self.solid_a)?;
        for fid in faces_a {
            self.face_ranks.insert(fid, Rank::A);
        }

        let faces_b = remus_topology::explorer::solid_faces(&self.topo, self.solid_b)?;
        for fid in faces_b {
            self.face_ranks.insert(fid, Rank::B);
        }

        Ok(())
    }

    /// Phase 1: map edges to split images and build sub-faces.
    fn fill_images(&mut self) -> Result<(), AlgoError> {
        let edge_images = fill_images::fill_edge_images(&self.arena);
        log::debug!(
            "Builder: {} original edges mapped to split images",
            edge_images.len()
        );

        self.sub_faces = fill_images_faces::fill_images_faces(
            &mut self.topo,
            &self.arena,
            &edge_images,
            &self.face_ranks,
            self.tol,
            &mut self.edge_lineage,
        )?;
        log::debug!("Builder: {} sub-faces created", self.sub_faces.len());

        // Step 3: same-domain detection (records pairs, does NOT set FaceClass)
        let face_shells = self.build_face_shell_map();
        let sd_result = same_domain::detect_same_domain_with_shells(
            &self.topo,
            &self.arena,
            &self.sub_faces,
            &self.face_ranks,
            self.tol,
            Some(&face_shells),
        );
        self.sd_pairs = sd_result.pairs;
        self.sd_within_rank_dups = sd_result.within_rank_dups;

        // Note: SD representative replacement (replacing B's face_id with
        // A's face_id) was attempted but produces degenerate 2-edge faces
        // because both sub-face entries then point to the same face entity,
        // and the BOP selector can't distinguish them. The correct approach
        // is to let BOP keep A's face and discard B's (which it already does),
        // then fix edge sharing at the BuilderSolid level via
        // merge_duplicate_edges.
        Ok(())
    }

    /// Map each input face to an ordinal unique per shell across both operand
    /// solids, so SD emission can tell a cross-shell structural coincidence
    /// (an internal void's face coplanar with an outer face) from within-shell
    /// boolean residue. A lookup failure just leaves faces unmapped, which
    /// keeps the historic dedup for them.
    fn build_face_shell_map(&self) -> HashMap<FaceId, usize> {
        let mut map = HashMap::new();
        let mut ordinal = 0usize;
        for sid in [self.solid_a, self.solid_b] {
            let Ok(solid) = self.topo.solid(sid) else {
                continue;
            };
            let shells =
                std::iter::once(solid.outer_shell()).chain(solid.inner_shells().iter().copied());
            for shell_id in shells {
                let Ok(shell) = self.topo.shell(shell_id) else {
                    continue;
                };
                for &fid in shell.faces() {
                    map.insert(fid, ordinal);
                }
                ordinal += 1;
            }
        }
        map
    }

    /// Phase 2: classify each sub-face as inside/outside the opposing solid.
    #[allow(clippy::too_many_lines)]
    fn classify_sub_faces(&mut self) -> Result<(), AlgoError> {
        // SD faces are excluded from non-SD BOP selection, so their
        // classification doesn't affect the result. But the ray-cast
        // classifier is non-deterministic at coplanar boundaries,
        // which can produce non-manifold results for near-tangent
        // geometries. Mark SD faces deterministically to skip ray-cast.
        //
        // Skip SD index construction entirely when no SD pairs exist
        // (common case for non-overlapping solids).
        // Only the cross-rank SD pair indices and the within-rank duplicates
        // (NOT their representatives) should bypass ray-cast classification.
        // The representative still needs normal IN/OUT classification because
        // `select_faces` routes it through the standard truth table — adding
        // it to `sd_indices` would force it to "On" with no matching pair
        // record, so `apply_sd_selection` would never pick it up and the
        // face would silently drop out.
        let sd_indices: std::collections::HashSet<usize> =
            if self.sd_pairs.is_empty() && self.sd_within_rank_dups.is_empty() {
                std::collections::HashSet::new()
            } else {
                let cross = self.sd_pairs.iter().flat_map(|p| [p.idx_a, p.idx_b]);
                let within = self.sd_within_rank_dups.iter().map(|d| d.duplicate);
                cross.chain(within).collect()
            };

        // Collect ray-cast geometry for each argument solid ONCE. Each sub-face
        // is classified against the opposing solid; rebuilding the opposing
        // solid's face geometry per sub-face was O(faces) × O(sub-faces). A
        // collection failure leaves `None`, which falls back to the per-call
        // path (identical behaviour, just not memoised).
        let geoms_a = classifier::RayCastGeoms::new(&self.topo, self.solid_a).ok();
        let geoms_b = classifier::RayCastGeoms::new(&self.topo, self.solid_b).ok();

        for (idx, sf) in self.sub_faces.iter_mut().enumerate() {
            if !sd_indices.is_empty() && sd_indices.contains(&idx) {
                // Same-domain faces are coincident by construction; the
                // ray-cast classifier is unstable at a coplanar boundary
                // (the interior sample sits on the opposing solid's face).
                // Force them "On" so `apply_sd_selection` keeps exactly one
                // representative per cross-rank pair. This includes disc
                // sub-faces (single closed-curve loops): when a disc is part
                // of an SD pair it is a flush, coincident cap on the result's
                // exterior — ray-casting it offsets the sample into the
                // opposing solid and wrongly drops the pair, leaving a hole in
                // the coincident face (e.g. a cylinder resting flush on a box
                // floor). Within-rank duplicates are dropped by `select_faces`
                // regardless of classification, so "On" is safe for them too.
                sf.classification = FaceClass::On;
                continue;
            }

            let opposing_solid = match sf.rank {
                Rank::A => self.solid_b,
                Rank::B => self.solid_a,
            };
            let opposing_geoms = match sf.rank {
                Rank::A => geoms_b.as_ref(),
                Rank::B => geoms_a.as_ref(),
            };

            let sample = if let Some(pt) = sf.interior_point {
                Ok(pt)
            } else if face_splitter::face_has_curved_lens_holes(&self.topo, sf.face_id) {
                // A curved-lens-hole wall (cylinder/cone with closed
                // Circle/Ellipse/NURBS holes) whose contained-interior search
                // failed: every generic interior sample risks landing inside the
                // removed lens, which would misclassify and drop the wall. Abort
                // the analytic split so the boolean falls back to mesh (correct)
                // rather than build a wrong B-rep.
                return Err(AlgoError::ClassificationFailed(format!(
                    "no contained interior for curved-lens wall {:?}; aborting analytic split",
                    sf.face_id
                )));
            } else {
                sample_face_interior(&self.topo, sf.face_id, self.tol)
            };

            match sample {
                Ok(point) => {
                    // Coincident-coplanar fast path: a planar sub-face lying in
                    // a plane coincident with an opposing-solid face cannot be
                    // classified by ray-cast — its interior point sits on the
                    // opposing plane and cardinal rays graze the coincident cap
                    // (voting wrongly Inside). Classify by 2D containment in the
                    // opposing face's region instead.
                    let coincident = if let remus_topology::face::FaceSurface::Plane { normal, d } =
                        self.topo.face(sf.face_id)?.surface()
                    {
                        classifier::classify_coincident_coplanar(
                            &self.topo,
                            opposing_solid,
                            opposing_geoms,
                            sf.face_id,
                            *normal,
                            *d,
                            Some(point),
                            self.tol,
                        )?
                    } else {
                        None
                    };
                    sf.classification = match coincident {
                        Some(class) => class,
                        None => classifier::classify_point_cached_with_tolerance(
                            &self.topo,
                            opposing_solid,
                            opposing_geoms,
                            point,
                            self.tol,
                        )?,
                    };
                    log::trace!(
                        "classify_sub_faces: idx={idx} face={:?} rank={:?} pt={point:?} class={:?}",
                        sf.face_id,
                        sf.rank,
                        sf.classification
                    );
                    if std::env::var("BK_CLS2").is_ok()
                        && let Ok(face) = self.topo.face(sf.face_id)
                    {
                        let mut wires = vec![face.outer_wire()];
                        wires.extend(face.inner_wires().iter().copied());
                        let mut touches = false;
                        'w: for wid in wires {
                            let Ok(w) = self.topo.wire(wid) else { continue };
                            for oe in w.edges() {
                                let Ok(e) = self.topo.edge(oe.edge()) else {
                                    continue;
                                };
                                for vid in [e.start(), e.end()] {
                                    if let Ok(v) = self.topo.vertex(vid) {
                                        let q = v.point();
                                        if (37.9..38.11).contains(&q.x())
                                            && (-41.8..-40.0).contains(&q.y())
                                            && (31.4..34.9).contains(&q.z())
                                        {
                                            touches = true;
                                            break 'w;
                                        }
                                    }
                                }
                            }
                        }
                        if touches {
                            log::debug!(
                                "CLS2 face={:?} {} rank={:?} src={:?} pt=({:.3},{:.3},{:.3}) class={:?}",
                                sf.face_id,
                                self.topo.face(sf.face_id)?.surface().type_tag(),
                                sf.rank,
                                sf.source_face,
                                point.x(),
                                point.y(),
                                point.z(),
                                sf.classification
                            );
                        }
                    }
                    if std::env::var("BK_CLS").is_ok()
                        && (37.9..38.11).contains(&point.x())
                        && (-41.8..-40.0).contains(&point.y())
                        && (31.4..34.9).contains(&point.z())
                    {
                        let tag = self.topo.face(sf.face_id)?.surface().type_tag();
                        log::debug!(
                            "CLS face={:?} {tag} rank={:?} src={:?} pt=({:.3},{:.3},{:.3}) class={:?}",
                            sf.face_id,
                            sf.rank,
                            sf.source_face,
                            point.x(),
                            point.y(),
                            point.z(),
                            sf.classification
                        );
                    }
                }
                Err(e) => {
                    return Err(AlgoError::ClassificationFailed(format!(
                        "could not sample interior of face {:?}: {e}",
                        sf.face_id
                    )));
                }
            }
        }

        let unknown_count = self
            .sub_faces
            .iter()
            .filter(|sf| sf.classification == FaceClass::Unknown)
            .count();
        let total = self.sub_faces.len();
        log::debug!(
            "Builder: {}/{total} sub-faces classified",
            total - unknown_count
        );

        if unknown_count > 0 {
            return Err(AlgoError::ClassificationFailed(format!(
                "{unknown_count} sub-faces could not be classified"
            )));
        }

        Ok(())
    }
}

/// Build an N-way FUSE result from the shared arena of an N-way pave filler.
///
/// Reuses the two-solid Builder machinery, generalized to N sources:
///
/// - **Splitting** — sections are stored face-relative (`pcurve_a == pcurve_b`),
///   so the face splitter is rank-invariant; every face is split with a
///   constant `Rank::A`. A sub-face's *global* source is tracked separately via
///   `face_source` (original input face → source index).
/// - **Classification** — each sub-face is kept iff its interior sample is
///   OUTSIDE every OTHER source (the union boundary). One `RayCastGeoms` is
///   built per source and reused across all sub-faces of that source's rivals.
/// - **Assembly** — the kept faces feed the standard solid assembler.
///
/// This slice handles the interpenetrating case (no coincident faces). If a
/// sub-face classifies `On` against another source — the signature of a
/// coincident/flush contact that needs cross-source same-domain resolution —
/// the fuse bails with an error so the caller can fall back to the sequential
/// path. Coincident-face support is the next increment.
///
/// # Errors
///
/// Returns [`AlgoError`] if a coincident contact is detected, a sub-face cannot
/// be sampled or classified, or assembly fails.
pub fn build_fuse_n<S: std::hash::BuildHasher>(
    mut topo: Topology,
    arena: GfaArena,
    sources: &[SolidId],
    face_source: &HashMap<FaceId, usize, S>,
    tol: Tolerance,
) -> Result<(Topology, SolidId), AlgoError> {
    // Split every source face. Sections are face-relative, so a constant rank
    // is correct for all of them (see the doc comment).
    let edge_images = fill_images::fill_edge_images(&arena);
    let all_a_ranks: HashMap<FaceId, Rank> = face_source.keys().map(|&f| (f, Rank::A)).collect();
    let mut nway_lineage = split_types::EdgeLineageLog::default();
    let sub_faces = fill_images_faces::fill_images_faces(
        &mut topo,
        &arena,
        &edge_images,
        &all_a_ranks,
        tol,
        &mut nway_lineage,
    )?;

    // The global source of each sub-face is its parent input face's source.
    let sub_source: Vec<usize> = sub_faces
        .iter()
        .map(|sf| {
            face_source.get(&sf.source_face).copied().ok_or_else(|| {
                AlgoError::AssemblyFailed(format!(
                    "sub-face parent {:?} has no source index",
                    sf.source_face
                ))
            })
        })
        .collect::<Result<_, _>>()?;

    // Resolve coincident faces across sources first: opposite-oriented groups
    // are interior interfaces (all dropped); same-oriented groups keep one
    // representative (still verified against sources outside the group). The
    // remaining sub-faces are classified by inside/outside as usual.
    let sd = same_domain::detect_same_domain_fuse_n(&topo, &arena, &sub_faces, &sub_source, tol)?;
    let keep_reprs: HashMap<usize, std::collections::HashSet<usize>> =
        sd.keep_reprs.into_iter().collect();

    // One ray-cast geometry per source, reused across every rival sub-face.
    let geoms: Vec<Option<classifier::RayCastGeoms>> = sources
        .iter()
        .map(|&s| classifier::RayCastGeoms::new(&topo, s).ok())
        .collect();

    // Keep a sub-face iff its interior sample is outside every source in
    // `others`. A coincident `On` against a non-coincident source means the
    // same-domain pass missed a coincidence — bail to the sequential fallback.
    let keep_if_outside = |topo: &Topology,
                           sample: Point3,
                           own: usize,
                           others: &dyn Fn(usize) -> bool|
     -> Result<bool, AlgoError> {
        for (j, &other) in sources.iter().enumerate() {
            if j == own || !others(j) {
                continue;
            }
            match classifier::classify_point_cached_with_tolerance(
                topo,
                other,
                geoms[j].as_ref(),
                sample,
                tol,
            )? {
                FaceClass::Inside => return Ok(false),
                FaceClass::On | FaceClass::CoplanarSame | FaceClass::CoplanarOpposite => {
                    return Err(AlgoError::AssemblyFailed(
                        "N-way fuse: unresolved coincident face; sequential fallback".into(),
                    ));
                }
                FaceClass::Outside | FaceClass::Unknown => {}
            }
        }
        Ok(true)
    };

    let mut selected = Vec::new();
    for (idx, sf) in sub_faces.iter().enumerate() {
        let own = sub_source[idx];
        let sample = match sf.interior_point {
            Some(pt) => pt,
            None => sample_face_interior(&topo, sf.face_id, tol)?,
        };

        let keep = if sd.grouped.contains(&idx) {
            // A coincident face: kept only if it is this group's same-oriented
            // representative AND outside every source not in the group.
            match keep_reprs.get(&idx) {
                Some(group_sources) => {
                    keep_if_outside(&topo, sample, own, &|j| !group_sources.contains(&j))?
                }
                None => false,
            }
        } else {
            // A normal sub-face: kept iff outside every other source.
            keep_if_outside(&topo, sample, own, &|_| true)?
        };

        if keep {
            selected.push(bop::SelectedFace {
                face_id: sf.face_id,
                source_face: sf.source_face,
                reversed: false,
            });
        }
    }

    orient_selected_fuse_analytic_holes(&mut topo, &sub_faces, &selected);
    let solid_id = assemble::assemble_solid(&mut topo, &selected, &[], &mut nway_lineage)?;
    Ok((topo, solid_id))
}

/// Restore the stored-CW hole convention on selected analytic fuse remainders.
///
/// The face splitter reverses internal loops into a generic hole orientation
/// before classification. That representation is required by cuts, but a
/// selected analytic fuse remainder needs the historical stored-CW winding at
/// assembly. Apply the correction only after selection so classification and
/// non-fuse operations remain unchanged.
fn orient_selected_fuse_analytic_holes(
    topo: &mut Topology,
    sub_faces: &[SubFace],
    selected: &[bop::SelectedFace],
) {
    let mut processed = std::collections::HashSet::new();

    for selected_face in selected {
        if !processed.insert(selected_face.face_id) {
            continue;
        }
        let Some(sub_face) = sub_faces
            .iter()
            .find(|sub_face| sub_face.face_id == selected_face.face_id)
        else {
            continue;
        };
        if sub_face.face_id == sub_face.source_face {
            continue;
        }

        let Ok(face) = topo.face(selected_face.face_id) else {
            continue;
        };
        if face.is_reversed()
            || matches!(
                face.surface(),
                remus_topology::face::FaceSurface::Plane { .. }
            )
            || face.inner_wires().is_empty()
        {
            continue;
        }
        let inner_wires = face.inner_wires().to_vec();
        let outer_wire = face.outer_wire();
        let mut replacements = Vec::with_capacity(inner_wires.len());

        for wire_id in inner_wires {
            let Ok(wire) = topo.wire(wire_id) else {
                replacements.clear();
                break;
            };
            let edges: Vec<_> = wire
                .edges()
                .iter()
                .rev()
                .map(|edge| {
                    remus_topology::wire::OrientedEdge::new(edge.edge(), !edge.is_forward())
                })
                .collect();
            let Ok(wire) = remus_topology::wire::Wire::new(edges, wire.is_closed()) else {
                replacements.clear();
                break;
            };
            replacements.push(topo.add_wire(wire));
        }

        if !replacements.is_empty() {
            let _ = topo.set_face_boundary_wires(selected_face.face_id, outer_wire, replacements);
        }
    }
}

/// Sample a point in the interior of a face.
///
/// Uses the midpoint of the first boundary edge, then offsets slightly
/// inward along (edge_tangent x face_normal) to get a point that is
/// reliably inside the face — unlike a vertex centroid, which can fall
/// outside non-convex faces.
///
/// The offset distance is scaled relative to the face's bounding box
/// diagonal to handle both very small and very large faces correctly.
/// Sample a planar face that has holes, as deep into the material as possible.
///
/// Returns the candidate whose smallest distance to ANY rim (outer or hole) is
/// greatest — the middle of the annulus on a bored cap, rather than a point
/// hugging one of the rims where every containment test against a polygonised
/// opposing hole is a coin flip.
///
/// Candidates are the midpoints between each outer-rim sample and its nearest
/// hole-rim sample (which is exactly mid-material for an annulus), plus the
/// centroid. Returns `None` if none of them lands in the material, leaving the
/// caller's generic path in charge.
/// Sample a planar face whose outer boundary is built from closed curves.
///
/// Such a face (a disc bounded by one circular edge) has fewer boundary
/// vertices than it has corners, so the caller's vertex polygon degenerates.
/// Polygonise the curve itself and hand it to the shared interior-point
/// sampler, which prefers the ring's centroid — the point furthest from the
/// rim, and the only stable place to probe when the rim is shared with an
/// opposing solid's wall.
///
/// Returns `None` when the boundary still cannot be polygonised or the sampled
/// point does not verify as interior, leaving the caller's generic path in
/// charge.
fn sample_closed_boundary_interior(
    topo: &Topology,
    face_id: FaceId,
    normal: remus_math::vec::Vec3,
) -> Result<Option<Point3>, AlgoError> {
    /// Samples per boundary edge. A closed circle spans the whole ring, so
    /// this is the ring's resolution; the sampler only needs enough points for
    /// the centroid and the containment test to be meaningful.
    const RING_SAMPLES: usize = 32;

    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;
    let mut pts = Vec::new();
    for oe in wire.edges() {
        let e = topo.edge(oe.edge())?;
        let sp = topo.vertex(e.start())?.point();
        let ep = topo.vertex(e.end())?.point();
        let (t0, t1) = e.strict_domain().map_err(|error| {
            AlgoError::FaceSplitFailed(format!(
                "face {face_id:?} edge {:?} lacks authoritative parameter range: {error}",
                oe.edge()
            ))
        })?;
        let (from, to) = if oe.is_forward() { (t0, t1) } else { (t1, t0) };
        for index in 0..RING_SAMPLES {
            #[allow(clippy::cast_precision_loss)]
            let fraction = index as f64 / RING_SAMPLES as f64;
            pts.push(e.curve().evaluate_with_endpoints(
                (to - from).mul_add(fraction, from),
                sp,
                ep,
            ));
        }
    }
    if pts.len() < 3 {
        return Ok(None);
    }

    let frame = plane_frame::PlaneFrame::from_plane_face(normal, &pts);
    let poly: Vec<_> = pts.iter().map(|p| frame.project(*p)).collect();
    let interior = classify_2d::sample_interior_point(&poly);
    if !classify_2d::point_in_polygon_2d(interior, &poly) {
        return Ok(None);
    }
    Ok(Some(frame.evaluate(interior.x(), interior.y())))
}

fn sample_holed_face_interior(
    topo: &Topology,
    face_id: FaceId,
    normal: remus_math::vec::Vec3,
) -> Result<Option<Point3>, AlgoError> {
    const CURVE_SAMPLES: u32 = 16;

    let face = topo.face(face_id)?;
    let sample_wire = |wid: remus_topology::wire::WireId| -> Result<Vec<Point3>, AlgoError> {
        let wire = topo.wire(wid)?;
        let mut pts = Vec::new();
        for oe in wire.edges() {
            let e = topo.edge(oe.edge())?;
            let sp = topo.vertex(e.start())?.point();
            let ep = topo.vertex(e.end())?.point();
            pts.push(sp);
            if !matches!(e.curve(), remus_topology::edge::EdgeCurve::Line) {
                let (t0, t1) = e.strict_domain().map_err(|error| {
                    AlgoError::FaceSplitFailed(format!(
                        "face {face_id:?} edge {:?} lacks authoritative parameter range: {error}",
                        oe.edge()
                    ))
                })?;
                for k in 1..CURVE_SAMPLES {
                    let t = f64::from(k).mul_add((t1 - t0) / f64::from(CURVE_SAMPLES), t0);
                    pts.push(e.curve().evaluate_with_endpoints(t, sp, ep));
                }
            }
        }
        Ok(pts)
    };

    let outer_pts = sample_wire(face.outer_wire())?;
    let mut hole_polys = Vec::new();
    for &iw in face.inner_wires() {
        let pts = sample_wire(iw)?;
        if pts.len() >= 3 {
            hole_polys.push(pts);
        }
    }
    if outer_pts.len() < 3 || hole_polys.is_empty() {
        return Ok(None);
    }

    let frame = plane_frame::PlaneFrame::from_plane_face(normal, &outer_pts);
    let outer2d: Vec<_> = outer_pts.iter().map(|p| frame.project(*p)).collect();
    let holes2d: Vec<Vec<_>> = hole_polys
        .iter()
        .map(|h| h.iter().map(|p| frame.project(*p)).collect::<Vec<_>>())
        .collect();

    let mut candidates: Vec<remus_math::vec::Point2> = Vec::new();
    for &o in &outer2d {
        if let Some(nearest) = holes2d
            .iter()
            .flatten()
            .min_by(|a, b| {
                (**a - o)
                    .length()
                    .partial_cmp(&(**b - o).length())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .copied()
        {
            candidates.push(remus_math::vec::Point2::new(
                f64::midpoint(o.x(), nearest.x()),
                f64::midpoint(o.y(), nearest.y()),
            ));
        }
    }
    #[allow(clippy::cast_precision_loss)]
    let inv = 1.0 / outer2d.len() as f64;
    candidates.push(remus_math::vec::Point2::new(
        outer2d.iter().map(|p| p.x()).sum::<f64>() * inv,
        outer2d.iter().map(|p| p.y()).sum::<f64>() * inv,
    ));

    let mut best: Option<(f64, remus_math::vec::Point2)> = None;
    for c in candidates {
        if !classify_2d::point_in_polygon_2d(c, &outer2d)
            || holes2d
                .iter()
                .any(|h| classify_2d::point_in_polygon_2d(c, h))
        {
            continue;
        }
        let clearance = std::iter::once(&outer2d)
            .chain(holes2d.iter())
            .map(|poly| classify_2d::distance_to_polygon_boundary(c, poly))
            .fold(f64::INFINITY, f64::min);
        if best.is_none_or(|(b, _)| clearance > b) {
            best = Some((clearance, c));
        }
    }

    Ok(best.map(|(_, c)| frame.evaluate(c.x(), c.y())))
}

fn sample_face_interior(
    topo: &Topology,
    face_id: FaceId,
    tol: Tolerance,
) -> Result<Point3, AlgoError> {
    use remus_math::vec::Vec3;

    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;
    let edges = wire.edges();

    if edges.is_empty() {
        return Err(AlgoError::FaceSplitFailed(format!(
            "face {face_id:?} has empty outer wire"
        )));
    }

    // Periodic faces bounded by closed curves (e.g. an unsplit cylinder
    // lateral wall between two full boundary circles): the closed-edge
    // midpoint lies on a v-extreme of the face, and the tangent-cross-normal
    // offset direction is unreliable there. Sample at the closed edge's u
    // and the midpoint of the face's v-range instead — interior in v by
    // construction, interior in u because the boundary curve spans the
    // full period.
    if !face.surface().is_planar() {
        let mut closed_mid: Option<Point3> = None;
        let mut v_min = f64::MAX;
        let mut v_max = f64::MIN;
        for oe in edges {
            let e = topo.edge(oe.edge())?;
            let sp = topo.vertex(e.start())?.point();
            let ep = topo.vertex(e.end())?.point();
            let (t0, t1) = e.strict_domain().map_err(|error| {
                AlgoError::FaceSplitFailed(format!(
                    "face {face_id:?} edge {:?} lacks authoritative parameter range: {error}",
                    oe.edge()
                ))
            })?;
            let mid = e
                .curve()
                .evaluate_with_endpoints(0.5_f64.mul_add(t1 - t0, t0), sp, ep);
            if e.start() == e.end()
                && !matches!(e.curve(), remus_topology::edge::EdgeCurve::Line)
                && closed_mid.is_none()
            {
                closed_mid = Some(mid);
            }
            for p in [sp, ep, mid] {
                if let Some((_, v)) = face.surface().project_point(p) {
                    v_min = v_min.min(v);
                    v_max = v_max.max(v);
                }
            }
        }
        if let Some(mid) = closed_mid
            && v_max - v_min > tol.linear
            && let Some((u, _)) = face.surface().project_point(mid)
            && let Some(pt) = face.surface().evaluate(u, 0.5 * (v_min + v_max))
        {
            return Ok(pt);
        }
    }

    let mut min_pt = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max_pt = Point3::new(f64::MIN, f64::MIN, f64::MIN);
    let mut point_count = 0_usize;
    for oe in edges {
        let e = topo.edge(oe.edge())?;
        let sp = topo.vertex(e.start())?.point();
        let ep = topo.vertex(e.end())?.point();
        // Sample ALONG each curved edge, not just its endpoints. A wire that
        // is one closed circle (an annular cap's outer rim, a disc) has
        // start == end, so an endpoint-only box collapses to a single point
        // and `offset_scale` degenerates to the linear tolerance — placing
        // the sample ~1e-7 off its own boundary, right on top of any
        // coincident wall, where the ray cast is a coin flip.
        let mut pts = vec![sp, ep];
        if !matches!(e.curve(), remus_topology::edge::EdgeCurve::Line) {
            let (t0, t1) = e.strict_domain().map_err(|error| {
                AlgoError::FaceSplitFailed(format!(
                    "face {face_id:?} edge {:?} lacks authoritative parameter range: {error}",
                    oe.edge()
                ))
            })?;
            for k in 1..4 {
                let t = f64::from(k).mul_add((t1 - t0) / 4.0, t0);
                pts.push(e.curve().evaluate_with_endpoints(t, sp, ep));
            }
        }
        for p in pts {
            min_pt = Point3::new(
                min_pt.x().min(p.x()),
                min_pt.y().min(p.y()),
                min_pt.z().min(p.z()),
            );
            max_pt = Point3::new(
                max_pt.x().max(p.x()),
                max_pt.y().max(p.y()),
                max_pt.z().max(p.z()),
            );
            point_count += 1;
        }
    }
    if point_count == 0 {
        return Err(AlgoError::FaceSplitFailed(format!(
            "face {face_id:?}: could not compute bounding box (no valid edge vertices)"
        )));
    }
    let diag = (max_pt - min_pt).length();
    let offset_scale = (diag * 1e-4).max(tol.linear);

    // Take the longest boundary edge and evaluate at its midpoint. The
    // longest edge gives the most room for the inward offset, and its
    // midpoint is least likely to sit on a shared junction plane where
    // the axis-aligned classification rays graze adjacent faces.
    let mut first_oe = &edges[0];
    let mut best_len = 0.0_f64;
    for oe in edges {
        let e = topo.edge(oe.edge())?;
        let sp = topo.vertex(e.start())?.point();
        let ep = topo.vertex(e.end())?.point();
        let len = (ep - sp).length();
        if len > best_len {
            best_len = len;
            first_oe = oe;
        }
    }
    let edge = topo.edge(first_oe.edge())?;
    let start_pos = topo.vertex(edge.start())?.point();
    let end_pos = topo.vertex(edge.end())?.point();
    let (t0, t1) = edge.strict_domain().map_err(|error| {
        AlgoError::FaceSplitFailed(format!(
            "face {face_id:?} edge {:?} lacks authoritative parameter range: {error}",
            first_oe.edge()
        ))
    })?;
    let t_mid = 0.5_f64.mul_add(t1 - t0, t0);
    let mid_pt = edge
        .curve()
        .evaluate_with_endpoints(t_mid, start_pos, end_pos);

    let tangent = edge
        .curve()
        .tangent_with_endpoints(t_mid, start_pos, end_pos);
    let surface = face.surface();

    let face_normal = if let Some((u, v)) = surface.project_point(mid_pt) {
        surface.normal(u, v)
    } else {
        // Plane: normal is constant
        match surface {
            remus_topology::face::FaceSurface::Plane { normal, .. } => *normal,
            _ => Vec3::new(0.0, 0.0, 1.0),
        }
    };

    // Inward direction: tangent x face_normal points into the face interior
    // (assuming CCW winding when viewed from the face normal direction)
    let inward = tangent.cross(face_normal);
    let inward_len = inward.length();

    let base_offset = if inward_len > 1e-12 {
        inward * (offset_scale / inward_len)
    } else {
        // Degenerate — use a tiny offset along the face normal instead
        face_normal * offset_scale
    };

    // For a planar face, verify the sample lands strictly inside the
    // (possibly concave) boundary polygon. The tangent×normal sign is
    // unreliable on inner/notch edges and reversed winding, and a
    // boundary-vertex centroid can fall in a concavity — e.g. the notch of an
    // L-shaped face left by a corner cut — so a centroid-based flip points the
    // sample OUTSIDE the face and into the opposing solid, misclassifying a
    // thin sliver. Project the boundary, then pick the offset sign (shrinking
    // the magnitude for strips thinner than the offset) that lands inside.
    if let remus_topology::face::FaceSurface::Plane { normal, .. } = surface {
        // A face with holes needs a sample in the MATERIAL, well clear of both
        // rims. Offsetting inward from the outer boundary is not enough: on an
        // annular cap it lands a hair inside its own rim, and the opposing
        // solid's matching hole is polygonised as an inscribed polygon, so a
        // point that close to the true circle reads as outside the hole and
        // the face is classified against the wrong region.
        if !face.inner_wires().is_empty()
            && let Some(pt) = sample_holed_face_interior(topo, face_id, *normal)?
        {
            return Ok(pt);
        }
        let mut poly = Vec::with_capacity(edges.len());
        for oe in edges {
            let e = topo.edge(oe.edge())?;
            poly.push(topo.vertex(oe.oriented_start(e))?.point());
        }
        // A boundary made of closed curves contributes one VERTEX per edge, so
        // a disc bounded by a single circle yields a one-point "polygon" and
        // the containment ladder below has nothing to test against. Without a
        // polygon the fallback returns one inward offset from the rim — a hair
        // off the disc's own boundary. Where that boundary is shared with an
        // opposing solid (a plug dropped into a bore of its own radius, whose
        // wall is coincident over its whole area) the sample sits ON the shared
        // wall and the ray cast is a coin flip: on a 30x30x10 plate the plug's
        // two caps classified differently at most bore radii, one was dropped,
        // and the open shell sent the fuse to the mesh fallback. Polygonise the
        // closed curve instead and take a properly interior point of the ring —
        // for a disc, its centre.
        if poly.len() < 3
            && face.inner_wires().is_empty()
            && let Some(pt) = sample_closed_boundary_interior(topo, face_id, *normal)?
        {
            return Ok(pt);
        }
        // A boundary with >= 3 vertices forms a real polygon to test against.
        if inward_len > 1e-12 && poly.len() >= 3 {
            let frame = plane_frame::PlaneFrame::from_plane_face(*normal, &poly);
            let poly2d: Vec<_> = poly.iter().map(|p| frame.project(*p)).collect();
            let eps = classify_2d::boundary_eps(&poly2d);
            // Try LARGE offsets first, then shrink. The base offset is
            // diag·1e-4 — a sample that close to the boundary edge can hug a
            // coincident interface plane of the opposing solid (a frustum
            // wall's longest edge lies ON the coincident bottom cap), where
            // the ray-cast classifier turns unstable and mirror-image walls
            // classify differently. A deeper sample is strictly better
            // whenever the polygon containment check admits it; thin slivers
            // reject the large candidates and fall through to the fine
            // scales. 28 halvings from 64 reach scale ~2.4e-7 (min offset
            // ~diag·2.4e-11), below any physically meaningful strip width,
            // so the loop only exits to the fallback for a near-zero-area
            // (degenerate) face.
            let mut scale = 64.0_f64;
            for _ in 0..28 {
                for sign in [1.0_f64, -1.0] {
                    let cand = mid_pt + base_offset * (sign * scale);
                    let c2 = frame.project(cand);
                    if classify_2d::point_in_polygon_2d(c2, &poly2d)
                        && classify_2d::distance_to_polygon_boundary(c2, &poly2d) > eps
                    {
                        return Ok(cand);
                    }
                }
                scale *= 0.5;
            }
            // Near-zero-area face: try a robust interior point of the projected
            // boundary. Verify it before use — its last-resort path returns the
            // vertex centroid, which can fall outside a concave boundary. If
            // even that is exterior, fall back to the edge midpoint (on the
            // boundary, never exterior) rather than a known-bad sample.
            let ip = classify_2d::sample_interior_point(&poly2d);
            if classify_2d::point_in_polygon_2d(ip, &poly2d) {
                return Ok(frame.evaluate(ip.x(), ip.y()));
            }
            return Ok(mid_pt);
        }
    }

    // Non-planar surfaces: the tangent×normal direction assumes CCW winding;
    // reversed or CW-wound faces flip it, sending the sample outside the
    // face. Use the boundary vertex centroid to pick the side that points
    // into the face.
    let mut offset = base_offset;
    let centroid = {
        let mut sum = Vec3::new(0.0, 0.0, 0.0);
        let mut n = 0_usize;
        for oe in edges {
            let e = topo.edge(oe.edge())?;
            for vid in [e.start(), e.end()] {
                let p = topo.vertex(vid)?.point();
                sum += Vec3::new(p.x(), p.y(), p.z());
                n += 1;
            }
        }
        #[allow(clippy::cast_precision_loss)]
        Point3::new(sum.x() / n as f64, sum.y() / n as f64, sum.z() / n as f64)
    };
    if offset.dot(centroid - mid_pt) < 0.0 {
        offset = offset * -1.0;
    }

    let interior_pt = mid_pt + offset;

    // Project back onto the surface to ensure the point is on-surface
    if let Some((u, v)) = surface.project_point(interior_pt)
        && let Some(on_surface) = surface.evaluate(u, v)
    {
        return Ok(on_surface);
    }

    // Planes have no UV projection, but the inward offset is already in-plane,
    // so the offset point itself is the on-surface sample. This reaches a
    // planar face only when its boundary has < 3 vertices (a single closed
    // circle/ellipse edge); the centroid above is the disc center, so the
    // flipped offset points into the disc.
    if matches!(surface, remus_topology::face::FaceSurface::Plane { .. }) && inward_len > 1e-12 {
        return Ok(interior_pt);
    }

    // Fallback: use the midpoint itself (it's on the boundary, not ideal
    // but better than a centroid that may be outside the face)
    Ok(mid_pt)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;
    use remus_math::vec::Vec3;
    use remus_topology::builder::{make_face_from_wire, make_polygon_wire};

    #[test]
    fn sample_face_interior_thin_l_frame_lands_in_strip() {
        // L-frame: a side-1.0001 square with a side-1.0 corner notch removed at
        // the origin, leaving a 0.0001-thin strip. The boundary-vertex centroid
        // (~0.667, ~0.667) falls inside the removed notch, so the old
        // centroid-based flip placed the sample outside the face. The sample
        // must instead land in the strip (one coordinate >= 1.0).
        let mut topo = Topology::new();
        let s = 1.0001;
        let n = 1.0;
        let pts = vec![
            Point3::new(n, 0.0, 0.0),
            Point3::new(s, 0.0, 0.0),
            Point3::new(s, s, 0.0),
            Point3::new(0.0, s, 0.0),
            Point3::new(0.0, n, 0.0),
            Point3::new(n, n, 0.0),
        ];
        let wire = make_polygon_wire(&mut topo, &pts, 1e-7).unwrap();
        let face = make_face_from_wire(&mut topo, wire).unwrap();

        let pt = sample_face_interior(&topo, face, Tolerance::default()).unwrap();
        // Strip check (not in the notch)...
        assert!(
            pt.x() >= n - 1e-9 || pt.y() >= n - 1e-9,
            "sample {pt:?} fell in the notch instead of the L-frame strip"
        );
        // ...and a direct interior-membership proof against the L-polygon.
        let frame = plane_frame::PlaneFrame::from_plane_face(Vec3::new(0.0, 0.0, 1.0), &pts);
        let poly2d: Vec<_> = pts.iter().map(|p| frame.project(*p)).collect();
        assert!(
            classify_2d::point_in_polygon_2d(frame.project(pt), &poly2d),
            "sample {pt:?} is not inside the L-frame polygon"
        );
    }

    /// A planar disc bounded by a single closed circle edge, radius `r`,
    /// centred on the origin in the z=0 plane.
    fn unit_disc(topo: &mut Topology, radius: f64) -> FaceId {
        use remus_topology::builder::make_circle_edge;
        use remus_topology::wire::{OrientedEdge, Wire};

        let edge = make_circle_edge(
            topo,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            radius,
            1e-7,
        )
        .unwrap();
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
        make_face_from_wire(topo, wire).unwrap()
    }

    #[test]
    fn sample_face_interior_planar_disc_lands_well_clear_of_the_rim() {
        // A planar disc bounded by a single closed circle edge has < 3 boundary
        // vertices, so the point-in-polygon path cannot apply.
        //
        // "Strictly inside" is not enough. This test used to allow anything
        // below r = 1 - 1e-9, which the old sample (one inward offset from the
        // rim, r ~ 1 - 1e-4) satisfied while sitting close enough to the
        // boundary to land ON a coincident opposing wall — where the ray cast
        // that classifies the face is a coin flip. That is exactly how the
        // through-hole plug fuse degenerated into a mesh boolean. Require the
        // sample to be robustly interior instead.
        let mut topo = Topology::new();
        let face = unit_disc(&mut topo, 1.0);

        let pt = sample_face_interior(&topo, face, Tolerance::default()).unwrap();
        let r = pt.x().hypot(pt.y());
        assert!(
            r < 0.5,
            "disc sample (r={r}) hugs the bounding circle; it must be well \
             clear of the rim to classify reliably against a coincident wall"
        );
        assert!(
            pt.z().abs() < 1e-12,
            "disc sample {pt:?} left the face's own plane"
        );
    }

    #[test]
    fn sample_face_interior_disc_clearance_scales_with_the_disc() {
        // The clearance must be a property of the disc, not an absolute
        // offset: a 0.05-radius cap in a matching bore needs the same relative
        // margin as a 50-radius one.
        for radius in [0.05_f64, 1.0, 50.0] {
            let mut topo = Topology::new();
            let face = unit_disc(&mut topo, radius);
            let pt = sample_face_interior(&topo, face, Tolerance::default()).unwrap();
            let r = pt.x().hypot(pt.y());
            assert!(
                r < 0.5 * radius,
                "disc of radius {radius}: sample at r={r} is not robustly interior"
            );
        }
    }
}
