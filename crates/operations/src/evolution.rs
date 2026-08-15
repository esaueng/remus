//! Evolution tracking for modeling operations.
//!
//! Records how faces evolve through booleans, fillets, and other operations,
//! enabling downstream consumers to track face provenance (e.g., for applying
//! persistent attributes like color or constraints).
//!
//! # Wrong provenance is worse than none
//!
//! A consumer that stores a user's face selection against these indices acts on
//! whatever this map says. Naming the wrong face silently moves that selection
//! onto different geometry; naming no face at all lets the consumer drop the
//! reference and say so. Every classifier here is therefore built to refuse
//! rather than guess: an output face the matcher cannot separate is reported in
//! [`EvolutionMap::unresolved`], not bound to the best of several poor
//! candidates. [`EvolutionMap::origin`] tells the consumer whether it is
//! looking at construction-derived fact or a geometric inference.
//!
//! # Refusing is not free either
//!
//! Silence about a face is its own failure. A result face in no bucket cannot be
//! told from a face that is not in the result, so a consumer neither rebinds it
//! nor knows to fail closed on it — and unlike a wrong answer, nothing about the
//! result looks unusual. Refusal is therefore reserved for questions that are
//! genuinely unanswerable, not spent on ties that only look like one.
//!
//! The distinction that does the work is between the two claims. `modified` says
//! an output face *is* an input face carried forward, and that is the claim a
//! stored selection rides on, so it is never made on a guess. `generated` says
//! an output face is new and names what it was built from; it cannot move a
//! selection anywhere, so it can name several sources at once. A blend band ties
//! between the two faces its rounded edge separated because it was built from
//! both — an unanswerable question under the first claim, and a plain fact under
//! the second.

use std::collections::{BTreeMap, HashMap, HashSet};

use brepkit_math::vec::{Point3, Vec3};

/// How an [`EvolutionMap`] was derived.
///
/// Consumers that bind persistent references should treat the two differently:
/// a [`Construction`](EvolutionOrigin::Construction) map is what the operation
/// itself recorded while building the result, and a
/// [`Geometry`](EvolutionOrigin::Geometry) map is an inference that can be
/// wrong even when it reports no ambiguity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EvolutionOrigin {
    /// The operation reported each output face's true input source while
    /// building the result. Exact.
    Construction,
    /// Output faces were matched to input faces from geometry alone (normal +
    /// centroid). Correct on the shapes it is tested against, but an inference.
    #[default]
    Geometry,
}

impl EvolutionOrigin {
    /// Whether entries in this map are construction-derived fact rather than
    /// geometric inference.
    #[must_use]
    pub fn is_exact(self) -> bool {
        matches!(self, Self::Construction)
    }

    /// Stable lowercase name, used in the JSON encoding and by consumers that
    /// branch on provenance quality.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Construction => "construction",
            Self::Geometry => "geometry",
        }
    }
}

/// Tracks how faces evolve through a modeling operation.
///
/// After a boolean, fillet, or other operation, this map records:
/// - **modified**: input face -> output faces that replace it
/// - **generated**: input face -> new faces created adjacent to it
/// - **deleted**: input faces that were completely removed
/// - **unresolved**: output faces whose origin could not be established
///
/// The four buckets are claims of different strength. `modified`, `generated`
/// and `deleted` are assertions; `unresolved` is an admission. An input face
/// that appears in none of them — because it only ever turned up as a losing
/// candidate in an `unresolved` tie — is *unknown*, not surviving and not
/// deleted, and a consumer must fail closed on it exactly as it would on an
/// unresolved output.
///
/// # `modified` and `generated` say different things
///
/// `modified` is an identity claim: this output face *is* that input face, cut
/// back. It is what a consumer rebinding a stored selection reads, so a wrong
/// entry moves the selection onto geometry the user never picked.
///
/// `generated` is an adjacency claim: this output face is new, and these are the
/// input faces it was built from. It cannot rebind anything, which is why a new
/// face may name several sources without hazard. A blend band names both faces
/// its rounded edge separated — many-to-one is the normal case here, not a
/// degenerate one, and it is how both the walking builder's construction record
/// and the geometric matcher report a band.
///
/// # Completeness
///
/// Every face of the result should appear in `modified`, `generated` or
/// `unresolved`. A face in none of them is invisible to a consumer, which cannot
/// distinguish it from a face that is not in the result — so it neither rebinds
/// nor fails closed. Note that a face count cannot detect this: the count is
/// right while the attribution is missing.
#[derive(Debug, Clone, Default)]
pub struct EvolutionMap {
    /// Input face -> output faces that are modified versions of it.
    pub modified: HashMap<usize, Vec<usize>>,
    /// Input face -> new faces generated from it (e.g., blend faces from fillet).
    pub generated: HashMap<usize, Vec<usize>>,
    /// Input faces that were completely removed.
    pub deleted: HashSet<usize>,
    /// Output faces with no established origin, each with the input faces that
    /// were plausible sources. An empty candidate list means nothing was
    /// plausible at all.
    ///
    /// Ordered so the map serializes deterministically.
    pub unresolved: BTreeMap<usize, Vec<usize>>,
    /// Whether the entries above are construction-derived or inferred.
    pub origin: EvolutionOrigin,
}

impl EvolutionMap {
    /// Create an empty evolution map whose entries are geometric inferences.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Create an empty evolution map whose entries the caller will fill in from
    /// its own construction records.
    #[must_use]
    pub fn exact() -> Self {
        Self {
            origin: EvolutionOrigin::Construction,
            ..Self::default()
        }
    }

    /// Record that `input` was modified into `output`.
    pub fn add_modified(&mut self, input: usize, output: usize) {
        self.modified.entry(input).or_default().push(output);
    }

    /// Record that `output` was generated from `input`.
    pub fn add_generated(&mut self, input: usize, output: usize) {
        self.generated.entry(input).or_default().push(output);
    }

    /// Record that `input` was deleted.
    pub fn add_deleted(&mut self, input: usize) {
        self.deleted.insert(input);
    }

    /// Record that `output`'s origin could not be established, listing the
    /// input faces that could not be told apart (empty if there were none).
    pub fn add_unresolved(&mut self, output: usize, candidates: Vec<usize>) {
        self.unresolved.insert(output, candidates);
    }

    /// Whether every output face this map saw was attributed.
    ///
    /// A consumer rebinding stored selections should check this before trusting
    /// the map as a complete account of the operation.
    #[must_use]
    pub fn is_complete(&self) -> bool {
        self.unresolved.is_empty()
    }

    /// Serialize to JSON without serde.
    ///
    /// Produces a JSON object with `modified`, `generated`, `deleted`,
    /// `unresolved` and `origin` fields. `origin` is `"construction"` or
    /// `"geometry"`; see [`EvolutionOrigin`].
    #[must_use]
    pub fn to_json(&self) -> String {
        fn index_map(m: &HashMap<usize, Vec<usize>>) -> String {
            // Sort so the encoding is deterministic across runs.
            let mut keys: Vec<usize> = m.keys().copied().collect();
            keys.sort_unstable();
            keys.iter()
                .map(|k| {
                    let vals: Vec<String> = m[k].iter().map(ToString::to_string).collect();
                    format!("\"{k}\":[{}]", vals.join(","))
                })
                .collect::<Vec<_>>()
                .join(",")
        }

        let unresolved_entries: Vec<String> = self
            .unresolved
            .iter()
            .map(|(k, vs)| {
                let vals: Vec<String> = vs.iter().map(ToString::to_string).collect();
                format!("\"{k}\":[{}]", vals.join(","))
            })
            .collect();

        let mut deleted: Vec<usize> = self.deleted.iter().copied().collect();
        deleted.sort_unstable();
        let deleted_vals: Vec<String> = deleted.iter().map(ToString::to_string).collect();

        format!(
            "{{\"modified\":{{{}}},\"generated\":{{{}}},\"deleted\":[{}],\
             \"unresolved\":{{{}}},\"origin\":\"{}\"}}",
            index_map(&self.modified),
            index_map(&self.generated),
            deleted_vals.join(","),
            unresolved_entries.join(","),
            self.origin.as_str()
        )
    }
}

/// A face's matching signature: `(index, normal, centroid)`.
pub type FaceSignature = (usize, Vec3, Point3);

/// A modified face is a trimmed piece of the same surface, so its normal barely
/// moves; the cone is wide only because non-planar face normals are sampled
/// from a boundary polygon and wander when the face is re-trimmed.
const NORMAL_MIN_DOT: f64 = 0.707; // cos 45°

/// Centroid budget as a fraction of the body's own diagonal.
///
/// 0.5 reproduces the budget the absolute constant it replaced happened to give
/// on a 10-unit box — the size every existing test models at — so this changes
/// the answer only where the answer was scale-dependent.
const CENTROID_BUDGET_FRACTION: f64 = 0.5;

/// Two candidates whose scores differ by less than this are not separable.
const SCORE_TIE: f64 = 0.05;

/// Normal agreement required before two input faces can be called pieces of one
/// surface (cos 2.6°).
const COSURFACE_MIN_DOT: f64 = 0.999;

/// Out-of-plane separation, as a fraction of the body diagonal, still counted as
/// "the same plane".
const COSURFACE_PLANE_FRACTION: f64 = 1e-3;

/// The characteristic length of a set of face signatures: the diagonal of the
/// axis-aligned box containing every centroid.
///
/// This is what makes the matcher answer the same question at every modelling
/// unit. It is derived from the faces themselves rather than taken from a
/// caller, so a caller that forgets to pass a scale cannot silently get the
/// wrong one.
///
/// Returns 0.0 when every centroid coincides (or there are none), which the
/// matcher reads as "no room to be approximate" and handles by requiring exact
/// coincidence — refusing rather than matching everything to everything.
#[must_use]
pub fn characteristic_length(faces: &[FaceSignature], more: &[FaceSignature]) -> f64 {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for &(_, _, c) in faces.iter().chain(more.iter()) {
        for (i, v) in [c.x(), c.y(), c.z()].into_iter().enumerate() {
            if v < min[i] {
                min[i] = v;
            }
            if v > max[i] {
                max[i] = v;
            }
        }
    }
    if !min[0].is_finite() {
        return 0.0;
    }
    let d: f64 = (0..3)
        .map(|i| {
            let e = max[i] - min[i];
            e * e
        })
        .sum();
    d.sqrt()
}

/// Build an [`EvolutionMap`] by matching output faces to input faces purely
/// from geometry (face normal + centroid signatures `(index, normal, centroid)`).
///
/// This is operation-agnostic — any op that can snapshot face signatures before
/// and after (booleans, fillets, …) reuses it:
/// - An output face whose normal+centroid clearly beats every alternative is a
///   **modified** version of that input.
/// - An output face matching no input at all is **generated**, attributed to the
///   nearest input — or to all of them when several are equally near.
/// - An output face that ties between several inputs it is parallel to none of
///   is also **generated**, from every one of them: it cannot be any of them
///   re-trimmed, so it is new geometry built between them. A blend band is the
///   case this exists for.
/// - An input face matched by no output is **deleted**.
/// - An output face whose best candidates cannot be separated, *and* which
///   could be one of them carried forward, is **unresolved** and is deliberately
///   left unattributed.
///
/// Distances are measured against the body's own diagonal
/// ([`characteristic_length`]), so the same body modelled in metres, millimetres
/// or inches produces the same map.
///
/// The result's [`EvolutionMap::origin`] is always
/// [`EvolutionOrigin::Geometry`]: this is an inference, and a consumer holding
/// persistent references should treat it as one.
#[must_use]
pub fn build_evolution_by_geometry(
    input_faces: &[FaceSignature],
    output_faces: &[FaceSignature],
) -> EvolutionMap {
    let scale = characteristic_length(input_faces, output_faces);
    build_evolution_by_geometry_with_scale(input_faces, output_faces, scale)
}

/// [`build_evolution_by_geometry`] with the characteristic length supplied.
///
/// Callers that know the body's true extent — a solid's bounding-box diagonal,
/// say — should pass it. It matters when the faces themselves do not span the
/// body: a single closed face (a sphere, a full torus) has one centroid, and a
/// length derived from centroids alone would collapse to zero.
///
/// A non-finite or negative `characteristic_length` is treated as zero, which
/// makes the matcher demand exact coincidence rather than match everything.
#[must_use]
pub fn build_evolution_by_geometry_with_scale(
    input_faces: &[FaceSignature],
    output_faces: &[FaceSignature],
    characteristic_length: f64,
) -> EvolutionMap {
    let mut evo = EvolutionMap::new();
    let mut matched_inputs: HashSet<usize> = HashSet::new();
    let mut unmatched_outputs: Vec<FaceSignature> = Vec::new();

    let scale = if characteristic_length.is_finite() && characteristic_length > 0.0 {
        characteristic_length
    } else {
        0.0
    };
    let max_dist = CENTROID_BUDGET_FRACTION * scale;
    let max_dist_sq = max_dist * max_dist;
    let plane_tol = COSURFACE_PLANE_FRACTION * scale;

    // A zero budget cannot divide, and a zero-scale body can only be matched by
    // exact coincidence. Normalise the score by 1.0 in that case: dist_sq is
    // then either 0 (score 1) or already rejected by the gate.
    let score_denom = if max_dist_sq > 0.0 { max_dist_sq } else { 1.0 };

    for &(out_idx, out_normal, out_centroid) in output_faces {
        let mut candidates: Vec<(usize, f64)> = Vec::new();
        let mut best_score = f64::NEG_INFINITY;

        for &(in_idx, in_normal, in_centroid) in input_faces {
            let dot = out_normal.dot(in_normal);
            if dot < NORMAL_MIN_DOT {
                continue;
            }
            let dist_sq = dist_sq(out_centroid, in_centroid);
            if dist_sq > max_dist_sq {
                continue;
            }
            let score = dot - dist_sq / score_denom;
            if score > best_score {
                best_score = score;
            }
            candidates.push((in_idx, score));
        }

        if candidates.is_empty() {
            unmatched_outputs.push((out_idx, out_normal, out_centroid));
            continue;
        }

        let tied: Vec<usize> = candidates
            .iter()
            .filter(|&&(_, score)| score >= best_score - SCORE_TIE)
            .map(|&(in_idx, _)| in_idx)
            .collect();

        // More than one tied candidate is only an answer when those candidates
        // are pieces of one surface — the two halves of a same-domain merge,
        // which genuinely both flow into this output. Candidates on different
        // surfaces that merely score alike are not a merge, and this map does
        // not toss coins about which of them the output *is*.
        //
        // But "which of them is it?" is the wrong question when the answer is
        // "none of them". An output parallel to no candidate cannot be any of
        // them carried forward, so the tie is not an ambiguity at all — it is
        // the signature of a face built *between* them. A rolling-ball blend
        // band is exactly that: tangent to each face its edge separated,
        // equidistant from both, and facing along their bisector. That is why
        // it ties, and `generated` is the bucket for it.
        //
        // The walking builder, which keeps a real construction record, reports
        // the same band as generated from the same two base faces. Reaching
        // that answer here too keeps the two engines behind one operation from
        // disagreeing about what a blend face descends from.
        //
        // A tie among candidates the output *is* parallel to stays refused: any
        // of them could be this face re-trimmed or moved, and picking one is
        // the coin toss this map does not make. So no `modified` entry is ever
        // added here, and the mis-binding hazard the refusal exists to prevent
        // stays closed.
        if tied.len() > 1 && !all_cosurface(input_faces, &tied, plane_tol) {
            evo.add_unresolved(out_idx, tied);
            continue;
        }

        for in_idx in tied {
            evo.add_modified(in_idx, out_idx);
            matched_inputs.insert(in_idx);
        }
    }

    // An output resembling no input is new geometry. It is attributed to the
    // nearest input within the same centroid budget — or, when several are
    // equally near, to every one of them.
    //
    // Several equally-near inputs is not the ambiguity it looks like. This face
    // resembles no input at all, so there is no question of it *being* one of
    // them and no selection to relocate; the only claim on offer is which
    // inputs it was built between, and "all of the equally-nearest" is the
    // honest answer to that. Naming one of them at random would be a guess;
    // naming none of them loses the one fact the matcher does have.
    for &(out_idx, _out_normal, out_centroid) in &unmatched_outputs {
        let mut by_dist: Vec<(usize, f64)> = input_faces
            .iter()
            .map(|&(in_idx, _, in_centroid)| (in_idx, dist_sq(out_centroid, in_centroid)))
            .filter(|&(_, d)| d <= max_dist_sq)
            .collect();
        if by_dist.is_empty() {
            evo.add_unresolved(out_idx, Vec::new());
            continue;
        }
        by_dist.sort_by(|a, b| a.1.total_cmp(&b.1).then_with(|| a.0.cmp(&b.0)));
        let (nearest, nearest_d) = by_dist[0];
        let tie_band = SCORE_TIE * score_denom;
        let contenders: Vec<usize> = by_dist
            .iter()
            .filter(|&&(_, d)| d <= nearest_d + tie_band)
            .map(|&(i, _)| i)
            .collect();
        if contenders.len() > 1 {
            evo.add_unresolved(out_idx, contenders);
            continue;
        }
        evo.add_generated(nearest, out_idx);
        matched_inputs.insert(nearest);
    }

    // An input no output claimed, and that never turned up as a candidate the
    // matcher could not decide, is gone. One that only ever lost a tie is
    // neither claimed nor deleted — it is unknown, and stays out of every
    // bucket so a consumer cannot read it as either.
    let contested: HashSet<usize> = evo.unresolved.values().flatten().copied().collect();
    for &(in_idx, _, _) in input_faces {
        if !matched_inputs.contains(&in_idx) && !contested.contains(&in_idx) {
            evo.add_deleted(in_idx);
        }
    }

    evo
}

fn dist_sq(a: Point3, b: Point3) -> f64 {
    let dx = a.x() - b.x();
    let dy = a.y() - b.y();
    let dz = a.z() - b.z();
    dx.mul_add(dx, dy.mul_add(dy, dz * dz))
}

#[cfg(test)]
fn could_be_carried_forward(input_faces: &[FaceSignature], want: usize, out_normal: Vec3) -> bool {
    input_faces
        .iter()
        .find(|&&(index, _, _)| index == want)
        .is_some_and(|&(_, normal, _)| normal.dot(out_normal) >= COSURFACE_MIN_DOT)
}

/// Whether every listed input face lies on one surface: parallel normals, and
/// no out-of-plane separation beyond `plane_tol`.
///
/// This is what separates a genuine same-domain merge (two coplanar halves of
/// one wall flowing into one output face) from two unrelated faces that happen
/// to score alike.
fn all_cosurface(input_faces: &[FaceSignature], indices: &[usize], plane_tol: f64) -> bool {
    let mut picked: Vec<(Vec3, Point3)> = Vec::with_capacity(indices.len());
    for &want in indices {
        match input_faces.iter().find(|&&(i, _, _)| i == want) {
            Some(&(_, n, c)) => picked.push((n, c)),
            None => return false,
        }
    }
    let Some(&(n0, c0)) = picked.first() else {
        return false;
    };
    picked.iter().skip(1).all(|&(n, c)| {
        if n.dot(n0) < COSURFACE_MIN_DOT {
            return false;
        }
        let offset = (c - c0).dot(n0).abs();
        offset <= plane_tol
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use brepkit_math::vec::{Point3, Vec3};

    use super::*;

    #[test]
    fn matcher_classifies_modified_generated_deleted() {
        let pz = Vec3::new(0.0, 0.0, 1.0);
        let nz = Vec3::new(0.0, 0.0, -1.0);
        let px = Vec3::new(1.0, 0.0, 0.0);
        let inputs = [
            (0usize, pz, Point3::new(0.0, 0.0, 0.0)),
            (1usize, nz, Point3::new(0.0, 0.0, -10.0)),
        ];
        let outputs = [
            // Same normal+position as input 0 → modified.
            (100usize, pz, Point3::new(0.0, 0.0, 0.0)),
            // Orthogonal normal, matches nothing → generated, nearest input is 0.
            (200usize, px, Point3::new(1.0, 0.0, 0.0)),
        ];
        let evo = build_evolution_by_geometry(&inputs, &outputs);
        assert_eq!(evo.modified.get(&0), Some(&vec![100]));
        assert_eq!(evo.generated.get(&0), Some(&vec![200]));
        assert!(evo.deleted.contains(&1), "input 1 had no output → deleted");
        assert!(evo.is_complete(), "nothing should be unresolved here");
        assert_eq!(evo.origin, EvolutionOrigin::Geometry);
    }

    #[test]
    fn two_coplanar_halves_merging_into_one_output_keep_both_origins() {
        let pz = Vec3::new(0.0, 0.0, 1.0);
        let inputs = [
            (0usize, pz, Point3::new(-2.0, 0.0, 0.0)),
            (1usize, pz, Point3::new(2.0, 0.0, 0.0)),
            // Something far away to give the set a scale.
            (2usize, -pz, Point3::new(0.0, 0.0, -10.0)),
        ];
        let outputs = [(100usize, pz, Point3::new(0.0, 0.0, 0.0))];
        let evo = build_evolution_by_geometry(&inputs, &outputs);
        assert_eq!(evo.modified.get(&0), Some(&vec![100]));
        assert_eq!(evo.modified.get(&1), Some(&vec![100]));
        assert!(evo.is_complete());
    }

    #[test]
    fn two_parallel_faces_on_different_planes_are_refused_not_guessed() {
        let pz = Vec3::new(0.0, 0.0, 1.0);
        // Equidistant from the output and NOT coplanar: no honest answer.
        let inputs = [
            (0usize, pz, Point3::new(0.0, 0.0, -2.0)),
            (1usize, pz, Point3::new(0.0, 0.0, 2.0)),
        ];
        let outputs = [(100usize, pz, Point3::new(0.0, 0.0, 0.0))];
        let evo = build_evolution_by_geometry(&inputs, &outputs);
        assert!(
            evo.modified.is_empty(),
            "a coin toss must not be recorded as a match: {:?}",
            evo.modified
        );
        assert_eq!(evo.unresolved.get(&100), Some(&vec![0, 1]));
        assert!(!evo.is_complete());
        // Neither contested input may be called deleted: they are unknown.
        assert!(evo.deleted.is_empty(), "{:?}", evo.deleted);
    }

    /// The six planes of a 10-cube, as the matcher sees them.
    fn cube_face_signatures() -> [FaceSignature; 6] {
        [
            (0, Vec3::new(0.0, 0.0, 1.0), Point3::new(5.0, 5.0, 10.0)),
            (1, Vec3::new(0.0, 0.0, -1.0), Point3::new(5.0, 5.0, 0.0)),
            (2, Vec3::new(1.0, 0.0, 0.0), Point3::new(10.0, 5.0, 5.0)),
            (3, Vec3::new(-1.0, 0.0, 0.0), Point3::new(0.0, 5.0, 5.0)),
            (4, Vec3::new(0.0, 1.0, 0.0), Point3::new(5.0, 10.0, 5.0)),
            (5, Vec3::new(0.0, -1.0, 0.0), Point3::new(5.0, 0.0, 5.0)),
        ]
    }

    /// The blend-band signature, reduced to face signatures. Geometry alone
    /// cannot prove that a bisector-facing output was generated from both
    /// equidistant inputs; operation-specific construction provenance can,
    /// but this fallback matcher must report the ambiguity.
    ///
    /// These are the real numbers for a 10-cube with the edge between the top
    /// (+z) and the right (+x) face rounded at radius 1: the band's axis lies
    /// at x = z = 9, so its surface centroid sits at 9 + cos 45° on both axes.
    #[test]
    fn a_band_facing_between_two_inputs_is_refused_without_provenance() {
        let inputs = cube_face_signatures();
        let bisector = Vec3::new(1.0, 0.0, 1.0).normalize().unwrap();
        let c = 9.0 + std::f64::consts::FRAC_1_SQRT_2;
        let outputs = [(100usize, bisector, Point3::new(c, 5.0, c))];

        let evo = build_evolution_by_geometry(&inputs, &outputs);

        assert!(
            evo.modified.is_empty(),
            "a band is not a modified copy of either base face: {:?}",
            evo.modified
        );
        assert!(evo.generated.is_empty(), "{:?}", evo.generated);
        assert_eq!(evo.unresolved.get(&100), Some(&vec![0, 2]));
        assert!(!evo.is_complete());
        assert!(!evo.deleted.contains(&0));
        assert!(!evo.deleted.contains(&2));
    }

    /// The discriminator itself. Orientation decides: a face parallel to a
    /// candidate might be that candidate re-trimmed or moved and is never
    /// declared new, however far it has travelled; a face at 45° to it cannot
    /// be, however close it sits. Position is not consulted, on purpose — a
    /// face that slid off its plane is still that face.
    #[test]
    fn carried_forward_is_decided_by_orientation_not_position() {
        let inputs = cube_face_signatures();
        let pz = Vec3::new(0.0, 0.0, 1.0);
        let bisector = Vec3::new(1.0, 0.0, 1.0).normalize().unwrap();

        assert!(
            could_be_carried_forward(&inputs, 0, pz),
            "the top face could always be itself"
        );
        assert!(
            !could_be_carried_forward(&inputs, 0, bisector),
            "a band at 45° to the top face is not the top face"
        );
        assert!(
            !could_be_carried_forward(&inputs, 2, bisector),
            "nor the right face"
        );
        assert!(
            !could_be_carried_forward(&inputs, 99, pz),
            "an index that is not an input yields no answer"
        );
    }

    #[test]
    fn an_output_far_from_everything_is_unresolved_not_attributed() {
        let pz = Vec3::new(0.0, 0.0, 1.0);
        let inputs = [
            (0usize, pz, Point3::new(0.0, 0.0, 0.0)),
            (1usize, -pz, Point3::new(0.0, 0.0, 1.0)),
        ];
        // Orthogonal normal and far outside the body: nothing plausible.
        let outputs = [(
            100usize,
            Vec3::new(1.0, 0.0, 0.0),
            Point3::new(500.0, 0.0, 0.0),
        )];
        let evo = build_evolution_by_geometry(&inputs, &outputs);
        assert_eq!(evo.unresolved.get(&100), Some(&Vec::new()));
        assert!(evo.generated.is_empty());
    }

    #[test]
    fn characteristic_length_is_the_centroid_box_diagonal() {
        let n = Vec3::new(0.0, 0.0, 1.0);
        let faces = [
            (0usize, n, Point3::new(0.0, 0.0, 0.0)),
            (1usize, n, Point3::new(3.0, 4.0, 0.0)),
        ];
        assert!((characteristic_length(&faces, &[]) - 5.0).abs() < 1e-12);
        assert!(characteristic_length(&[], &[]).abs() < f64::EPSILON);
    }

    #[test]
    fn json_round_trip_shape() {
        let mut evo = EvolutionMap::exact();
        evo.add_modified(1, 7);
        evo.add_generated(1, 8);
        evo.add_deleted(2);
        evo.add_unresolved(9, vec![3, 4]);
        assert_eq!(
            evo.to_json(),
            "{\"modified\":{\"1\":[7]},\"generated\":{\"1\":[8]},\"deleted\":[2],\
             \"unresolved\":{\"9\":[3,4]},\"origin\":\"construction\"}"
        );
    }

    #[test]
    fn fillet_evolution_accepts_valid_planar_topology() {
        use brepkit_topology::explorer::{solid_edges, solid_faces};

        let mut topo = brepkit_topology::Topology::new();
        let cube = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edges(&topo, cube).unwrap();
        let result = crate::blend_ops::fillet_v2(&mut topo, cube, &[edges[0]], 1.0)
            .expect("planar fillet should produce validated topology");
        assert!(result.failed.is_empty());
        assert!(!result.is_partial);
        assert!(solid_faces(&topo, result.solid).unwrap().len() > 6);

        let report = brepkit_check::validate::validate_solid(
            &topo,
            result.solid,
            &brepkit_check::validate::ValidateOptions::default(),
        )
        .unwrap();
        assert!(report.is_valid(), "{:#?}", report.issues);
    }
}
