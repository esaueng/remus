//! Mesh-independent material-side acceptance for convex blends (B71).
//!
//! A convex edge rounds material away, so a correct convex blend only removes:
//! the result must be contained in the input up to the input's own boundary
//! uncertainty. The mesh-volume oracle in [`crate::blend_ops`] cannot see a
//! violation of this when the mesh error at the measuring deflection exceeds
//! the moved material: on the B71 cross-drilled rim the true net is `-0.008`
//! (analytic ray-cast Monte Carlo, `+0.037` added against `-0.046` removed),
//! while whole-solid mesh deltas read `+0.473` (coarse, defective bore wall),
//! `-0.135` (dense), and drift through zero across fine deflections. The sign
//! flips with tessellation density, so no mesh density is an oracle here.
//!
//! This module measures added material directly with analytic ray casting
//! ([`crate::classify::classify_points`], exact for every supported surface
//! type, no tessellation involved) over a weighted sample of the blend's reach
//! tube. It only ever *adds* refusals on positive evidence; acceptance still
//! requires the volume, validation, and closed-shell guards, so an uncertain
//! Monte Carlo verdict never becomes a success on its own.
//!
//! # Error budgets
//!
//! * Ray tolerance is `1e-6` (the check-crate default). The defects this
//!   catches sit up to ~1.7e-1 deep (B71 liner) against rim-curve wobble of
//!   `~2e-4`: two to three orders of margin. Points exactly on a boundary
//!   classify `OnBoundary` and are abstained on (never counted either way).
//! * The estimate uses inverse-overlap weighted sampling of local boxes:
//!   `se` is the sample standard error with `N = 6000` deterministic draws.
//!   Refusal needs `added - 3·se` above the allowance
//!   *and* at least 5 added samples (quorum against single-ray flukes).
//! * The allowance is `2e-3·reach·(chain_length·reach) + 1e-6·V`: rim-wobble
//!   slivers (marched curves ride their carriers by ~1e-3 of the reach over
//!   the band area) stay an order of magnitude below it, while B71's `0.037`
//!   exceeds it ~100x. Both terms scale with the model, so the verdict is
//!   scale-invariant; an absolute floor would dwarf small-scale signals.
//! * Sampling uses the union of local edge-segment boxes grown by `4·reach`.
//!   Boxes are chosen by volume, and each sample is weighted by the reciprocal
//!   of its box-overlap count. This gives an unbiased added-volume estimate
//!   over that union, without diluting a large circular rim into its enclosing
//!   AABB. The generator is a fixed-seed LCG over `u64` wrapping
//!   arithmetic: verdicts are bit-deterministic across runs and platforms
//!   (no `HashMap` iteration, no floating-point reassociation).
//! * Scope is deliberately narrow: only selections whose every blended edge
//!   classifies analytically [`crate::query::EdgeConcavity::Convex`], and only
//!   results carrying a NURBS band. Concave blends add material by design (a
//!   cylinder-sphere shoulder keeps 16/18 under-strip probes `Inside`), and
//!   analytic-band results are already qualified by the existing oracles, so
//!   both classes skip this check vacuously.
//!
//! # Cost
//!
//! The work is bounded: `N` fixed samples, two batched prepared
//! classifications. Analytic inputs classify in bulk in well under a second;
//! a result carrying a NURBS band costs ray-vs-NURBS intersections per sample
//! (B71: ~1 ms/sample, ~7 s total in release). The check runs only for the
//! all-convex class that produces a NURBS band, which has no valid in-repo
//! precedent (the walking engine never converges tangent on curved convex
//! rims) — ordinary box/planar and analytic-shoulder fillets never reach the
//! sampling loop.

use remus_math::det_hash::DetHashSet;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::solid::SolidId;

use crate::OperationsError;
use crate::classify::{PointClassification, classify_points};

/// Monte Carlo samples in the reach tube. Calibrated so B71's `0.037` of
/// added material in a 14-unit tube reads ~16 counts (`±4`, lower-3σ bound
/// still 4 counts above zero).
pub const ADDED_SAMPLES: usize = 6000;

/// Half-margin around each edge segment, in units of the blend reach.
/// B71's liner sits up to ~4.5 reaches from its rim samples.
pub const TUBE_MARGIN_RADII: f64 = 4.0;

/// Minimum added samples for a refusal (quorum against single-ray flukes).
pub const ADDED_QUORUM: u64 = 5;

/// Allowance for numerical dust: rim-wobble slivers plus one part per million
/// of the tube volume.
///
/// The wobble term scales with the model so the oracle stays scale-invariant:
/// marched rim curves ride their analytic carriers by ~1e-3 of the blend
/// reach (measured 2e-4 at reach 0.15), and the sliver they can trap is that
/// wobble times the band area (chain length times reach). An absolute floor
/// would dwarf small-scale signals instead, so there is none.
pub fn added_allowance(
    pristine: &Topology,
    chains: &[Vec<EdgeId>],
    reach: f64,
    tube_volume: f64,
) -> Result<f64, OperationsError> {
    let mut chain_length = 0.0;
    for chain in chains {
        for &edge in chain {
            chain_length += crate::measure::edge_length(pristine, edge)?;
        }
    }
    Ok(2e-3 * reach * (chain_length * reach) + 1e-6 * tube_volume)
}

/// Deterministic 64-bit LCG (`Numerical Recipes` constants). Fixed seed, so
/// the tube sample — and therefore every verdict — is reproducible.
struct TubeRng(u64);

impl TubeRng {
    const fn next_u64(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0
    }

    fn next_unit(&mut self) -> f64 {
        // Top 53 bits over 2^53: exactly representable, in [0, 1).
        const SCALE: f64 = 1.0 / ((1u64 << 53) as f64);
        ((self.next_u64() >> 11) as f64) * SCALE
    }
}

/// Sample of the union of local boxes around a blend's selected edges.
/// Drawn from pristine topology so consumed edges cannot move the sample.
pub struct ReachTube {
    points: Vec<Point3>,
    inverse_multiplicities: Vec<f64>,
    proposal_volume: f64,
    volume: f64,
}

impl ReachTube {
    /// Estimated volume of the union of local reach boxes.
    pub const fn volume(&self) -> f64 {
        self.volume
    }
}

/// Added-material estimate with its weighted-sample standard error.
#[derive(Debug)]
pub struct AddedEstimate {
    /// Added volume from inverse-overlap weighted strict flips.
    pub added: f64,
    /// Binomial standard error of [`AddedEstimate::added`].
    pub se: f64,
    /// Strict outside-to-inside flips (the quorum input).
    pub added_counts: u64,
}

/// Oriented local box along one edge chord. Its transverse width is tied
/// to reach and sampled chord deviation, even on a large diagonal rim.
struct ReachBox {
    center: Point3,
    axes: [Vec3; 3],
    half: [f64; 3],
    volume: f64,
}

impl ReachBox {
    fn around(a: Point3, b: Point3, margin: f64, deviation: f64) -> Option<Self> {
        let chord = b - a;
        let length = chord.length();
        let tangent = if length > 0.0 {
            chord.normalize().ok()?
        } else {
            Vec3::new(1.0, 0.0, 0.0)
        };
        let reference = if tangent.x().abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let normal = tangent.cross(reference).normalize().ok()?;
        let axes = [tangent, normal, tangent.cross(normal)];
        let half = [
            length * 0.5 + margin,
            margin + deviation,
            margin + deviation,
        ];
        let volume = 8.0 * half[0] * half[1] * half[2];
        (volume.is_finite() && volume > 0.0).then_some(Self {
            center: a + chord * 0.5,
            axes,
            half,
            volume,
        })
    }

    fn contains(&self, p: Point3) -> bool {
        let offset = p - self.center;
        (0..3).all(|k| offset.dot(self.axes[k]).abs() <= self.half[k])
    }

    fn draw(&self, rng: &mut TubeRng) -> Point3 {
        let mut p = self.center;
        for k in 0..3 {
            p = p + self.axes[k] * ((2.0 * rng.next_unit() - 1.0) * self.half[k]);
        }
        p
    }
}

const MAX_REACH_BOXES: usize = 8192;
const MAX_REACH_DEPTH: u8 = 12;

/// Subdivide until the sampled midpoint is close to its chord. A limit is an
/// explicit refusal: skipping a very curved reach could accept wrong-side
/// material without ever sampling it.
#[allow(clippy::too_many_arguments)]
fn append_reach_boxes(
    curve: &EdgeCurve,
    start: Point3,
    end: Point3,
    t0: f64,
    t1: f64,
    p0: Point3,
    p1: Point3,
    reach: f64,
    depth: u8,
    boxes: &mut Vec<ReachBox>,
) -> Result<(), OperationsError> {
    let tm = f64::midpoint(t0, t1);
    let pm = curve.evaluate_with_endpoints(tm, start, end);
    let chord = p1 - p0;
    let length_sq = chord.dot(chord);
    let nearest = if length_sq > 0.0 {
        p0 + chord * ((pm - p0).dot(chord) / length_sq).clamp(0.0, 1.0)
    } else {
        p0
    };
    let deviation = (pm - nearest).length();
    if !deviation.is_finite() {
        return Err(OperationsError::InvalidInput {
            reason: "blend reach contains non-finite curve geometry".into(),
        });
    }
    if deviation > reach {
        if depth >= MAX_REACH_DEPTH || boxes.len() >= MAX_REACH_BOXES {
            return Err(OperationsError::InvalidInput {
                reason: "blend reach exceeds the local sampling budget".into(),
            });
        }
        append_reach_boxes(curve, start, end, t0, tm, p0, pm, reach, depth + 1, boxes)?;
        append_reach_boxes(curve, start, end, tm, t1, pm, p1, reach, depth + 1, boxes)?;
    } else {
        if boxes.len() >= MAX_REACH_BOXES {
            return Err(OperationsError::InvalidInput {
                reason: "blend reach exceeds the local sampling budget".into(),
            });
        }
        boxes.push(
            ReachBox::around(p0, p1, TUBE_MARGIN_RADII * reach, deviation).ok_or_else(|| {
                OperationsError::InvalidInput {
                    reason: "blend reach has non-finite local volume".into(),
                }
            })?,
        );
    }
    Ok(())
}

/// Sample local boxes around `chains` using pristine input geometry.
///
/// A box is selected with probability proportional to its volume. Overlap
/// multiplicity corrects the duplicate coverage in both the volume estimate
/// and the added-material estimate. No whole-chain AABB is sampled.
pub fn sample_reach_tube(
    pristine: &Topology,
    chains: &[Vec<EdgeId>],
    reach: f64,
) -> Result<Option<ReachTube>, OperationsError> {
    if !(reach.is_finite() && reach > 0.0) {
        return Err(OperationsError::InvalidInput {
            reason: "blend reach must be positive and finite".into(),
        });
    }
    let mut boxes = Vec::new();
    for chain in chains {
        for &edge_id in chain {
            let edge = pristine.edge(edge_id)?;
            let start = pristine.vertex(edge.start())?.point();
            let end = pristine.vertex(edge.end())?.point();
            let (t0, t1) = crate::authoritative_edge_domain(edge, "blend reach-tube sampling")?;
            let intervals = if matches!(edge.curve(), EdgeCurve::Line) {
                1
            } else {
                32
            };
            let mut previous_t = t0;
            let mut previous = edge.curve().evaluate_with_endpoints(t0, start, end);
            for i in 1..=intervals {
                #[allow(clippy::cast_precision_loss)]
                let t = (t1 - t0).mul_add(i as f64 / intervals as f64, t0);
                let point = edge.curve().evaluate_with_endpoints(t, start, end);
                append_reach_boxes(
                    edge.curve(),
                    start,
                    end,
                    previous_t,
                    t,
                    previous,
                    point,
                    reach,
                    0,
                    &mut boxes,
                )?;
                previous_t = t;
                previous = point;
            }
        }
    }
    if boxes.is_empty() {
        return Ok(None);
    }
    let mut cumulative = Vec::with_capacity(boxes.len());
    let mut proposal_volume = 0.0;
    for local in &boxes {
        proposal_volume += local.volume;
        cumulative.push(proposal_volume);
    }
    if !proposal_volume.is_finite() {
        return Err(OperationsError::InvalidInput {
            reason: "blend reach has non-finite total volume".into(),
        });
    }
    let mut rng = TubeRng(0xB71u64);
    let mut points = Vec::with_capacity(ADDED_SAMPLES);
    let mut inverse_multiplicities = Vec::with_capacity(ADDED_SAMPLES);
    for _ in 0..ADDED_SAMPLES {
        let draw = rng.next_unit() * proposal_volume;
        let index = cumulative
            .partition_point(|&limit| limit <= draw)
            .min(boxes.len() - 1);
        let local = &boxes[index];
        let p = local.draw(&mut rng);
        let multiplicity = boxes
            .iter()
            .filter(|local| local.contains(p))
            .count()
            .max(1);
        #[allow(clippy::cast_precision_loss)]
        inverse_multiplicities.push(1.0 / multiplicity as f64);
        points.push(p);
    }
    #[allow(clippy::cast_precision_loss)]
    let volume =
        proposal_volume * inverse_multiplicities.iter().sum::<f64>() / ADDED_SAMPLES as f64;
    Ok(Some(ReachTube {
        points,
        inverse_multiplicities,
        proposal_volume,
        volume,
    }))
}

/// Estimate added material: tube points `Outside` the pristine input but
/// `Inside` the result (strict flips only; `OnBoundary` either way abstains).
///
/// Both classifications are analytic ray casts over prepared solids — no
/// tessellation at any point, so the estimate is independent of mesh density
/// by construction.
pub fn estimate_added_material(
    pristine: &Topology,
    input: SolidId,
    live: &Topology,
    result: SolidId,
    tube: &ReachTube,
) -> Result<AddedEstimate, OperationsError> {
    const TOLERANCE: f64 = 1e-6;
    let before = classify_points(pristine, input, &tube.points, 0.01, TOLERANCE)?;
    let after = classify_points(live, result, &tube.points, 0.01, TOLERANCE)?;
    let mut added_counts = 0u64;
    for (a, b) in before.iter().zip(after.iter()) {
        if matches!(a, PointClassification::Outside) && matches!(b, PointClassification::Inside) {
            added_counts += 1;
        }
    }
    let mut sum = 0.0;
    let mut sum_sq = 0.0;
    for ((a, b), &weight) in before
        .iter()
        .zip(after.iter())
        .zip(&tube.inverse_multiplicities)
    {
        if matches!(a, PointClassification::Outside) && matches!(b, PointClassification::Inside) {
            sum += weight;
            sum_sq += weight * weight;
        }
    }
    #[allow(clippy::cast_precision_loss)]
    let n = tube.points.len() as f64;
    let variance = ((sum_sq - sum * sum / n) / (n * (n - 1.0))).max(0.0);
    Ok(AddedEstimate {
        added: tube.proposal_volume * sum / n,
        se: tube.proposal_volume * variance.sqrt(),
        added_counts,
    })
}

/// Refuse a convex blend on positive evidence of added material.
///
/// Fires only on quorum (`ADDED_QUORUM` strict flips) with the lower-3σ bound
/// above the dust allowance. Anything else — including an uncertain estimate
/// — passes silently here; the volume oracle stays responsible for those.
pub fn refuse_if_added_material(
    operation: &'static str,
    estimate: &AddedEstimate,
    allowance: f64,
) -> Result<(), OperationsError> {
    if estimate.added_counts >= ADDED_QUORUM && estimate.added - 3.0 * estimate.se > allowance {
        return Err(OperationsError::InvalidInput {
            reason: format!(
                "{operation} on convex edges added {:.3} of material outside the input; \
                 rounding a convex edge must remove it",
                estimate.added
            ),
        });
    }
    Ok(())
}

/// True when the result has a newly created or newly NURBS face.
/// Pre-existing NURBS support faces do not make an analytic blend pay for the
/// expensive material sampler.
pub fn result_has_nurbs_band(
    pristine: &Topology,
    input: SolidId,
    live: &Topology,
    result: SolidId,
) -> Result<bool, OperationsError> {
    let input_faces: DetHashSet<_> = remus_topology::explorer::solid_faces(pristine, input)?
        .into_iter()
        .collect();
    for face_id in remus_topology::explorer::solid_faces(live, result)? {
        if !matches!(
            live.face(face_id)?.surface(),
            remus_topology::face::FaceSurface::Nurbs(_)
        ) {
            continue;
        }
        if !input_faces.contains(&face_id)
            || !matches!(
                pristine.face(face_id)?.surface(),
                remus_topology::face::FaceSurface::Nurbs(_)
            )
        {
            return Ok(true);
        }
    }
    Ok(false)
}

/// True when every blended edge is analytically convex: one `Inside` quadrant
/// out of four under exact ray casting (never the mesh-backed winding path,
/// so the gate itself is density-independent).
pub fn all_convex_analytic(
    topo: &Topology,
    solid: SolidId,
    edges: &[EdgeId],
    probe: f64,
) -> Result<bool, OperationsError> {
    if edges.is_empty() {
        return Ok(false);
    }
    let adjacency = topo.build_adjacency(solid)?;
    for &edge in edges {
        let faces = adjacency.faces_for_edge(edge);
        if faces.len() != 2 || faces[0] == faces[1] {
            return Ok(false);
        }
        if crate::query::edge_concavity_from_faces(topo, solid, edge, faces[0], faces[1], probe)?
            != crate::query::EdgeConcavity::Convex
        {
            return Ok(false);
        }
    }
    Ok(true)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    use crate::boolean::{BooleanOp, boolean};
    use crate::primitives::{make_box, make_cylinder};
    use crate::transform::transform_solid;

    /// The pure estimator on analytic boolean pairs: a cut adds nothing and
    /// removes its tool bite; a fuse of a disjoint-equivalent body adds.
    #[test]
    fn estimator_sees_boolean_material_moves() {
        let mut topo = Topology::new();
        let body = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
        let tool = make_cylinder(&mut topo, 1.0, 6.0).unwrap();
        transform_solid(
            &mut topo,
            tool,
            &remus_math::mat::Mat4::translation(2.0, 2.0, -1.0),
        )
        .unwrap();
        let cut = boolean(&mut topo, BooleanOp::Cut, body, tool).unwrap();

        // Reach tube around a box edge of the input body.
        let edge = remus_topology::explorer::solid_edges(&topo, body)
            .unwrap()
            .into_iter()
            .next()
            .expect("box must have edges");
        let chains = vec![vec![edge]];
        let tube = sample_reach_tube(&topo, &chains, 0.3)
            .unwrap()
            .expect("box edge tube must sample");
        assert_eq!(tube.points.len(), ADDED_SAMPLES);

        // Cut removes near the edge tube only where the tool bites; it must
        // never add material anywhere in the tube.
        let cut_est = estimate_added_material(&topo, body, &topo, cut, &tube).unwrap();
        assert_eq!(cut_est.added_counts, 0);
        assert!(cut_est.added <= 0.0);
        let cut_allowance = added_allowance(&topo, &chains, 0.3, tube.volume).unwrap();
        refuse_if_added_material("fillet", &cut_est, cut_allowance).unwrap();

        // Fusing a separate box overlapping the tube must read as added.
        let mut topo2 = Topology::new();
        let body2 = make_box(&mut topo2, 4.0, 4.0, 4.0).unwrap();
        let extra = make_box(&mut topo2, 1.0, 1.0, 1.0).unwrap();
        transform_solid(
            &mut topo2,
            extra,
            &remus_math::mat::Mat4::translation(0.0, 0.0, 4.0),
        )
        .unwrap();
        let fused = boolean(&mut topo2, BooleanOp::Fuse, body2, extra).unwrap();
        // A top-perimeter edge, so the small reach tube actually contains the
        // fused tab (a 1-unit tab in a ~100-unit tube reads ~50 counts).
        let edge2 = remus_topology::explorer::solid_edges(&topo2, body2)
            .unwrap()
            .into_iter()
            .find(|edge| {
                let data = topo2.edge(*edge).unwrap();
                [data.start(), data.end()]
                    .iter()
                    .all(|vertex| (topo2.vertex(*vertex).unwrap().point().z() - 4.0).abs() < 1e-9)
            })
            .expect("box must have a top edge");
        let tube2 = sample_reach_tube(&topo2, &[vec![edge2]], 0.5)
            .unwrap()
            .expect("tube must sample");
        let fuse_est = estimate_added_material(&topo2, body2, &topo2, fused, &tube2).unwrap();
        assert!(
            fuse_est.added_counts >= ADDED_QUORUM,
            "fused tab must read as added: {fuse_est:?}"
        );
        let fuse_chains = [vec![edge2]];
        let fuse_allowance = added_allowance(&topo2, &fuse_chains, 0.5, tube2.volume).unwrap();
        assert!(fuse_est.added - 3.0 * fuse_est.se > fuse_allowance);
        let refusal = refuse_if_added_material("fillet", &fuse_est, fuse_allowance).unwrap_err();
        assert!(
            refusal.to_string().contains("convex edges added"),
            "{refusal}"
        );
    }

    /// Degenerate reach refuses to sample rather than guessing.
    #[test]
    fn degenerate_reach_is_an_input_error() {
        let mut topo = Topology::new();
        let _body = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let chains: Vec<Vec<EdgeId>> = vec![vec![]];
        assert!(sample_reach_tube(&topo, &chains, 0.0).is_err());
        assert!(sample_reach_tube(&topo, &chains, f64::NAN).is_err());
    }

    #[test]
    fn large_circular_rim_samples_near_the_reach_not_its_enclosing_box() {
        let mut topo = Topology::new();
        let solid = make_cylinder(&mut topo, 100.0, 1.0).unwrap();
        let rim = remus_topology::explorer::solid_edges(&topo, solid)
            .unwrap()
            .into_iter()
            .find(|&id| matches!(topo.edge(id).unwrap().curve(), EdgeCurve::Circle(_)))
            .unwrap();
        let tube = sample_reach_tube(&topo, &[vec![rim]], 0.1)
            .unwrap()
            .unwrap();
        assert_eq!(tube.points.len(), ADDED_SAMPLES);
        assert!(tube.volume() < 3000.0, "volume {}", tube.volume());
        let max_radial_error = tube
            .points
            .iter()
            .map(|p| (p.x().hypot(p.y()) - 100.0).abs())
            .fold(0.0_f64, f64::max);
        assert!(max_radial_error < 2.0, "radial error {max_radial_error}");
    }

    #[test]
    fn duplicate_reach_boxes_do_not_double_estimated_volume() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let edge = remus_topology::explorer::solid_edges(&topo, solid).unwrap()[0];
        let single = sample_reach_tube(&topo, &[vec![edge]], 0.2)
            .unwrap()
            .unwrap();
        let doubled = sample_reach_tube(&topo, &[vec![edge, edge]], 0.2)
            .unwrap()
            .unwrap();
        assert!((single.volume() - doubled.volume()).abs() < 1e-9);
    }

    #[test]
    fn preexisting_nurbs_face_does_not_trigger_new_band_guard() {
        let mut topo = Topology::new();
        let body = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let faces = remus_topology::explorer::solid_faces(&topo, body).unwrap();
        let patch = crate::cap::bilinear_cap_patch(&[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.0, 2.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
        ])
        .unwrap();
        topo.face_mut(faces[0])
            .unwrap()
            .set_surface(remus_topology::face::FaceSurface::Nurbs(patch.clone()));
        let pristine = topo.clone();
        assert!(!result_has_nurbs_band(&pristine, body, &topo, body).unwrap());

        topo.face_mut(faces[1])
            .unwrap()
            .set_surface(remus_topology::face::FaceSurface::Nurbs(patch));
        assert!(result_has_nurbs_band(&pristine, body, &topo, body).unwrap());
    }

    /// Estimates are deterministic: same tube twice, identical counts.
    #[test]
    fn estimator_is_deterministic() {
        let mut topo = Topology::new();
        let body = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
        let edge = remus_topology::explorer::solid_edges(&topo, body)
            .unwrap()
            .into_iter()
            .next()
            .expect("box must have edges");
        let chains = vec![vec![edge]];
        let first = sample_reach_tube(&topo, &chains, 0.3).unwrap().unwrap();
        let second = sample_reach_tube(&topo, &chains, 0.3).unwrap().unwrap();
        assert_eq!(first.points.len(), second.points.len());
        for (a, b) in first.points.iter().zip(second.points.iter()) {
            assert_eq!((a.x(), a.y(), a.z()), (b.x(), b.y(), b.z()));
        }
    }
}
