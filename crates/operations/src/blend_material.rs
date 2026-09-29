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
//! type, no tessellation involved) over a uniform sample of the blend's reach
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
//! * The estimate is binomial: `se = V·sqrt(p·(1-p)/N)` with `N = 6000`
//!   deterministic samples. Refusal needs `added - 3·se` above the allowance
//!   *and* at least 5 added samples (quorum against single-ray flukes).
//! * The allowance is `2e-3·reach·(chain_length·reach) + 1e-6·V`: rim-wobble
//!   slivers (marched curves ride their carriers by ~1e-3 of the reach over
//!   the band area) stay an order of magnitude below it, while B71's `0.037`
//!   exceeds it ~100x. Both terms scale with the model, so the verdict is
//!   scale-invariant; an absolute floor would dwarf small-scale signals.
//! * Sampling is uniform in the G1-chain bbox grown by `4·reach`, so the
//!   estimate is an unbiased added *volume* with a scale-invariant
//!   signal-to-noise ratio (both signal and standard error scale with the
//!   tube volume). The generator is a fixed-seed LCG over `u64` wrapping
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

use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::solid::SolidId;

use crate::OperationsError;
use crate::classify::{PointClassification, classify_points};

/// Monte Carlo samples in the reach tube. Calibrated so B71's `0.037` of
/// added material in a 14-unit tube reads ~16 counts (`±4`, lower-3σ bound
/// still 4 counts above zero).
pub const ADDED_SAMPLES: usize = 6000;

/// Tube half-margin around the G1-chain bbox, in units of the blend reach.
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

/// Uniform sample of a blend's reach tube, drawn from the pristine
/// (pre-build) topology so consumed/rewritten edges cannot move the tube.
pub struct ReachTube {
    points: Vec<Point3>,
    volume: f64,
}

impl ReachTube {
    /// The tube's volume (the scale the added fraction multiplies).
    pub const fn volume(&self) -> f64 {
        self.volume
    }
}

/// Added-material estimate with its binomial standard error.
#[derive(Debug)]
pub struct AddedEstimate {
    /// Added volume: tube volume times the strict outside-to-inside fraction.
    pub added: f64,
    /// Binomial standard error of [`AddedEstimate::added`].
    pub se: f64,
    /// Strict outside-to-inside flips (the quorum input).
    pub added_counts: u64,
}

/// Sample the reach tube around `chains` ( pristine input geometry).
///
/// Returns `None` when the tube degenerates (zero-volume bbox): there is
/// nothing to measure, so the caller skips the check rather than guessing.
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
    let mut samples = Vec::new();
    for chain in chains {
        for &edge_id in chain {
            let edge = pristine.edge(edge_id)?;
            let start = pristine.vertex(edge.start())?.point();
            let end = pristine.vertex(edge.end())?.point();
            let (t0, t1) = crate::authoritative_edge_domain(edge, "blend reach-tube sampling")?;
            for i in 0..=32 {
                #[allow(clippy::cast_precision_loss)]
                let fraction = i as f64 / 32.0;
                let t = (t1 - t0).mul_add(fraction, t0);
                samples.push(edge.curve().evaluate_with_endpoints(t, start, end));
            }
        }
    }
    if samples.is_empty() {
        return Ok(None);
    }
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in &samples {
        for (k, v) in [(0, p.x()), (1, p.y()), (2, p.z())] {
            lo[k] = lo[k].min(v);
            hi[k] = hi[k].max(v);
        }
    }
    let margin = TUBE_MARGIN_RADII * reach;
    for k in 0..3 {
        lo[k] -= margin;
        hi[k] += margin;
    }
    let volume = (hi[0] - lo[0]) * (hi[1] - lo[1]) * (hi[2] - lo[2]);
    if !volume.is_finite() || volume <= 0.0 {
        return Ok(None);
    }
    let mut rng = TubeRng(0xB71u64);
    let mut points = Vec::with_capacity(ADDED_SAMPLES);
    for _ in 0..ADDED_SAMPLES {
        points.push(Point3::new(
            lo[0] + rng.next_unit() * (hi[0] - lo[0]),
            lo[1] + rng.next_unit() * (hi[1] - lo[1]),
            lo[2] + rng.next_unit() * (hi[2] - lo[2]),
        ));
    }
    Ok(Some(ReachTube { points, volume }))
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
    #[allow(clippy::cast_precision_loss)]
    let n = tube.points.len() as f64;
    #[allow(clippy::cast_precision_loss)]
    let fraction = added_counts as f64 / n;
    Ok(AddedEstimate {
        added: tube.volume * fraction,
        se: tube.volume * (fraction * (1.0 - fraction) / n).sqrt(),
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
    if estimate.added_counts >= ADDED_QUORUM
        && estimate.added - 3.0 * estimate.se > allowance
    {
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

/// True when `solid` carries a NURBS band face (the walking engine's output
/// family, and the only one this oracle second-guesses).
pub fn result_has_nurbs_band(topo: &Topology, solid: SolidId) -> Result<bool, OperationsError> {
    for face_id in remus_topology::explorer::solid_faces(topo, solid)? {
        if matches!(
            topo.face(face_id)?.surface(),
            remus_topology::face::FaceSurface::Nurbs(_)
        ) {
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
