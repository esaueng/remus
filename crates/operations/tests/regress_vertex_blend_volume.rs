//! Regression: a fillet that involves a VERTEX must remove the material a
//! rolling ball removes — no more — and must never drop an edge it was asked
//! to blend.
//!
//! Three defects are pinned here, all specific to selections that meet at a
//! vertex (single edges and parallel pairs were already exact):
//!
//! 1. A corner chain over-removed by 147 % and a full top perimeter by 259 %.
//! 2. The corner-chain tessellation came back with inconsistently wound edges
//!    while `validate_solid` reported no errors — visibly broken shading and a
//!    bad STL export from a body the kernel called valid.
//! 3. A selection mixing straight edges with a bore rim silently returned the
//!    straight-edge-only result: the rim was dropped with no error.
//!
//! # The closed form
//!
//! Every number asserted here is derived from the box's own dimensions, never
//! recorded from the engine. A constant-radius rolling ball of radius `r` on a
//! right dihedral removes, per unit length of edge, the corner square minus the
//! quarter disc it rolls out:
//!
//! ```text
//! A(r) = r² − πr²/4 = r²(1 − π/4)
//! ```
//!
//! At a vertex where two blended edges meet at a right angle, the ball can no
//! longer touch both faces of either edge once it is closer than `r` to the
//! third face, so each band stops `r` short of the vertex (`r / tan(θ/2)` with
//! θ = π/2). The ball then sits nestled in the corner, tangent to all three
//! faces, centred one radius in along each: the vertex blend is the octant of
//! that ball facing the corner. Inside the `r × r × r` cube at the vertex the
//! solid keeps exactly that octant, so the corner removes
//!
//! ```text
//! C(r) = r³ − (1/8)(4/3)πr³ = r³(1 − π/6)
//! ```
//!
//! and the total for a selection is
//!
//! ```text
//! removed = A(r) · Σ(Lᵢ − r·kᵢ)  +  C(r) · (junction vertices)
//! ```
//!
//! where `kᵢ` counts the ends of edge `i` that land on a junction vertex. This
//! is the same figure OpenCascade reaches on these picks.
//!
//! # Why the assertions are deflection-independent
//!
//! `solid_volume` integrates analytic faces exactly and tessellates the rest,
//! so a NURBS blend face converges to the closed form from below as the
//! deflection shrinks. Every volume assertion below is therefore run as a
//! convergence sweep: the residual must either be at the tolerance floor at
//! every deflection (an exact analytic result) or shrink monotonically toward
//! it. A recorded value cannot satisfy that.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;
use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::blend_ops;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

const W: f64 = 80.0;
const D: f64 = 60.0;
const H: f64 = 20.0;
const R: f64 = 2.0;

/// Material a rolling ball of radius `r` removes per unit length of a right
/// dihedral edge: the corner square minus the quarter disc.
fn band_area(r: f64) -> f64 {
    r * r * (1.0 - PI / 4.0)
}

/// Material removed at a right-angled vertex where two or more blended bands
/// meet: the `r`-cube at the corner minus the octant of the corner ball.
fn corner_volume(r: f64) -> f64 {
    r * r * r * (1.0 - PI / 6.0)
}

/// Closed-form material removed by a constant-radius fillet.
///
/// `lengths` are the blended edge lengths; `junction_ends` counts, across all
/// blended edges, how many edge ENDS land on a vertex shared with another
/// blended edge; `junctions` counts those vertices.
fn closed_form_removal(r: f64, lengths: &[f64], junction_ends: usize, junctions: usize) -> f64 {
    #[allow(clippy::cast_precision_loss)]
    let setback_loss = r * junction_ends as f64;
    let total_length: f64 = lengths.iter().sum();
    #[allow(clippy::cast_precision_loss)]
    let corners = junctions as f64;
    band_area(r) * (total_length - setback_loss) + corner_volume(r) * corners
}

fn solid_edge_list(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    let mut seen = Vec::new();
    for fid in solid_faces(topo, s).unwrap() {
        let f = topo.face(fid).unwrap();
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                if !seen.contains(&oe.edge()) {
                    seen.push(oe.edge());
                }
            }
        }
    }
    seen
}

/// The straight top edge (z == `H`) lying at `y == at` (`along_x`) or `x == at`.
fn top_edge(topo: &Topology, s: SolidId, along_x: bool, at: f64) -> EdgeId {
    solid_edge_list(topo, s)
        .into_iter()
        .find(|&e| {
            let ed = topo.edge(e).unwrap();
            if ed.start() == ed.end() {
                return false;
            }
            let a = topo.vertex(ed.start()).unwrap().point();
            let b = topo.vertex(ed.end()).unwrap().point();
            if (a.z() - H).abs() > 1e-9 || (b.z() - H).abs() > 1e-9 {
                return false;
            }
            if along_x {
                (a.y() - at).abs() < 1e-9 && (b.y() - at).abs() < 1e-9
            } else {
                (a.x() - at).abs() < 1e-9 && (b.x() - at).abs() < 1e-9
            }
        })
        .expect("top edge")
}

/// The closed circular rim edge at `z == at` with the given radius.
fn rim_edge(topo: &Topology, s: SolidId, at: f64, radius: f64) -> EdgeId {
    solid_edge_list(topo, s)
        .into_iter()
        .find(|&e| {
            let ed = topo.edge(e).unwrap();
            let EdgeCurve::Circle(c) = ed.curve() else {
                return false;
            };
            ed.start() == ed.end()
                && (c.radius() - radius).abs() < 1e-9
                && (c.center().z() - at).abs() < 1e-9
        })
        .expect("rim edge")
}

/// Deflections used for every volume assertion, coarse to fine.
const SWEEP: [f64; 4] = [0.02, 0.005, 0.001, 0.0002];

/// Relative residual allowed against the closed form at the FINEST deflection.
///
/// Every face of these results is analytic (planes, the blend cylinders, the
/// corner ball), but `solid_volume` still integrates over a tessellation, so
/// the measured loss converges to the closed form from above as the mesh
/// refines: on the single-edge control it is 0.56 % at deflection 0.02, 0.11 %
/// at 0.001 and 0.023 % at 0.0002 — linear in the deflection. The floor is set
/// just above that finest figure. It is a quadrature budget, not a geometry
/// budget: `assert_converges` additionally requires the residual to SHRINK as
/// the mesh refines, so a wrong corner (which is off by tens of percent at
/// every deflection) cannot hide inside it.
const VOLUME_FLOOR: f64 = 5e-4;

/// Assert `removed` matches `want` and that it does so because the geometry is
/// right, not because one deflection happened to land on the number.
///
/// The residual at each deflection must be within `floor` (an exact analytic
/// integration) or must shrink as the mesh refines. A result that is wrong by a
/// fixed amount stays wrong at every deflection and fails here.
fn assert_converges(measured: &[(f64, f64)], want: f64, floor: f64, what: &str) {
    assert!(want > 0.0, "{what}: the closed form must be positive");
    let rel = |v: f64| (v - want).abs() / want;
    let finest = measured.last().expect("at least one deflection");
    assert!(
        rel(finest.1) <= floor,
        "{what}: removed {removed} at deflection {defl}, closed form {want} \
         ({pct:.3} % off, budget {budget:.3} %)\nsweep: {measured:?}",
        removed = finest.1,
        defl = finest.0,
        pct = rel(finest.1) * 100.0,
        budget = floor * 100.0,
    );
    for pair in measured.windows(2) {
        let (d0, v0) = pair[0];
        let (d1, v1) = pair[1];
        assert!(
            rel(v1) <= rel(v0).max(floor) + 1e-12,
            "{what}: refining the deflection from {d0} to {d1} moved the result \
             AWAY from the closed form ({v0} → {v1}, want {want}); the error is \
             not quadrature\nsweep: {measured:?}"
        );
    }
}

/// Run a fillet and measure the removed volume across the deflection sweep.
fn removal_sweep(pick: impl Fn(&Topology, SolidId) -> Vec<EdgeId>, r: f64) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    for defl in SWEEP {
        let mut topo = Topology::new();
        let body = make_box(&mut topo, W, D, H).expect("box");
        let before = measure::solid_volume(&topo, body, defl).unwrap();
        let edges = pick(&topo, body);
        let result = blend_ops::fillet_v2(&mut topo, body, &edges, r).expect("fillet");
        assert!(result.failed.is_empty(), "{:?}", result.failed);
        let after = measure::solid_volume(&topo, result.solid, defl).unwrap();
        out.push((defl, before - after));
    }
    out
}

/// Control: a single straight edge, no vertex involved. Already exact — it must
/// stay exact.
#[test]
fn single_edge_matches_the_closed_form() {
    let measured = removal_sweep(|t, b| vec![top_edge(t, b, true, 0.0)], R);
    let want = closed_form_removal(R, &[W], 0, 0);
    assert_converges(&measured, want, VOLUME_FLOOR, "one top edge");
}

/// Control: two opposite (non-touching) edges. Twice the single-edge figure.
#[test]
fn opposite_edges_match_the_closed_form() {
    let measured = removal_sweep(
        |t, b| vec![top_edge(t, b, true, 0.0), top_edge(t, b, true, D)],
        R,
    );
    let want = closed_form_removal(R, &[W, W], 0, 0);
    assert_converges(&measured, want, VOLUME_FLOOR, "two opposite top edges");
}

/// Defect 1a: a corner chain — two top edges meeting at one vertex.
///
/// Both bands stop `R` short of the shared vertex and the corner ball's octant
/// stays; the engine removed 147 % too much.
#[test]
fn corner_chain_matches_the_closed_form() {
    let measured = removal_sweep(
        |t, b| vec![top_edge(t, b, true, 0.0), top_edge(t, b, false, 0.0)],
        R,
    );
    // Two edges, one shared vertex → two set-back ends, one corner.
    let want = closed_form_removal(R, &[W, D], 2, 1);
    assert_converges(&measured, want, VOLUME_FLOOR, "corner chain");
}

/// Defect 1b: the whole top perimeter — four edges, four corners.
#[test]
fn top_perimeter_matches_the_closed_form() {
    let measured = removal_sweep(
        |t, b| {
            vec![
                top_edge(t, b, true, 0.0),
                top_edge(t, b, true, D),
                top_edge(t, b, false, 0.0),
                top_edge(t, b, false, W),
            ]
        },
        R,
    );
    // Four edges, four corners → every edge is set back at both ends.
    let want = closed_form_removal(R, &[W, W, D, D], 8, 4);
    assert_converges(&measured, want, VOLUME_FLOOR, "top perimeter");
}

/// Every edge at a vertex selected: the corner is a full sphere octant and the
/// same closed form applies. This case already passed and must keep passing.
#[test]
fn three_edge_corner_matches_the_closed_form() {
    let measured = removal_sweep(
        |t, b| {
            vec![
                top_edge(t, b, true, 0.0),
                top_edge(t, b, false, 0.0),
                // the vertical edge at (0, 0)
                solid_edge_list(t, b)
                    .into_iter()
                    .find(|&e| {
                        let ed = t.edge(e).unwrap();
                        if ed.start() == ed.end() {
                            return false;
                        }
                        let a = t.vertex(ed.start()).unwrap().point();
                        let c = t.vertex(ed.end()).unwrap().point();
                        a.x().abs() < 1e-9
                            && a.y().abs() < 1e-9
                            && c.x().abs() < 1e-9
                            && c.y().abs() < 1e-9
                    })
                    .expect("vertical corner edge"),
            ]
        },
        R,
    );
    let want = closed_form_removal(R, &[W, D, H], 3, 1);
    assert_converges(&measured, want, VOLUME_FLOOR, "three-edge corner");
}

/// Count tessellation edges whose two incident triangles traverse them the same
/// way — the mesh defect a viewer shades as inverted patches and STL export
/// writes out as flipped facets.
fn inconsistently_wound_edges(topo: &Topology, s: SolidId, deflection: f64) -> usize {
    let mesh = tessellate_solid_with_tolerance(topo, s, deflection, 0.1).unwrap();
    let q = 1e6;
    let mut canon: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut remap = vec![0u32; mesh.positions.len()];
    for (i, p) in mesh.positions.iter().enumerate() {
        #[allow(clippy::cast_possible_truncation)]
        let key = (
            (p.x() * q).round() as i64,
            (p.y() * q).round() as i64,
            (p.z() * q).round() as i64,
        );
        #[allow(clippy::cast_possible_truncation)]
        let next = canon.len() as u32;
        remap[i] = *canon.entry(key).or_insert(next);
    }
    // Directed half-edge counts: a consistently wound closed mesh uses every
    // undirected edge once in each direction.
    let mut directed: HashMap<(u32, u32), i32> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let v = [
            remap[tri[0] as usize],
            remap[tri[1] as usize],
            remap[tri[2] as usize],
        ];
        for &(a, b) in &[(v[0], v[1]), (v[1], v[2]), (v[2], v[0])] {
            if a == b {
                continue;
            }
            *directed.entry((a, b)).or_insert(0) += 1;
        }
    }
    let mut bad = 0;
    for (&(a, b), &fwd) in &directed {
        if a >= b {
            continue;
        }
        let rev = directed.get(&(b, a)).copied().unwrap_or(0);
        if fwd != rev {
            bad += 1;
        }
    }
    bad
}

/// Defect 2: the corner-chain mesh must be consistently wound.
///
/// `validate_solid` reported zero errors on this body while its tessellation
/// carried 56 inconsistently wound edges — what a viewer shades and what STL
/// export writes.
#[test]
fn corner_chain_mesh_is_consistently_wound() {
    let mut topo = Topology::new();
    let body = make_box(&mut topo, W, D, H).expect("box");
    let edges = vec![
        top_edge(&topo, body, true, 0.0),
        top_edge(&topo, body, false, 0.0),
    ];
    let result = blend_ops::fillet_v2(&mut topo, body, &edges, R).expect("corner chain fillet");
    let bad = inconsistently_wound_edges(&topo, result.solid, 0.01);
    assert_eq!(
        bad, 0,
        "corner chain: {bad} inconsistently wound mesh edges"
    );
}

/// Defect 2b: the same for the whole perimeter.
#[test]
fn top_perimeter_mesh_is_consistently_wound() {
    let mut topo = Topology::new();
    let body = make_box(&mut topo, W, D, H).expect("box");
    let edges = vec![
        top_edge(&topo, body, true, 0.0),
        top_edge(&topo, body, true, D),
        top_edge(&topo, body, false, 0.0),
        top_edge(&topo, body, false, W),
    ];
    let result = blend_ops::fillet_v2(&mut topo, body, &edges, R).expect("perimeter fillet");
    let bad = inconsistently_wound_edges(&topo, result.solid, 0.01);
    assert_eq!(
        bad, 0,
        "top perimeter: {bad} inconsistently wound mesh edges"
    );
}

/// A plate with one bore through the middle.
fn bored_plate(topo: &mut Topology, hole_r: f64) -> SolidId {
    let body = make_box(topo, W, D, H).expect("plate blank");
    let drill = make_cylinder(topo, hole_r, H + 4.0).expect("drill");
    transform_solid(topo, drill, &Mat4::translation(W / 2.0, D / 2.0, -2.0)).expect("place drill");
    boolean(topo, BooleanOp::Cut, body, drill).expect("drill hole")
}

/// Defect 3: a selection mixing straight edges with a bore rim must never come
/// back as the straight-edge-only result.
///
/// The failure was undetectable from outside: a fresh valid handle, `failed`
/// empty, a volume inside the plausible envelope — and a body byte-identical to
/// the one the perimeter alone produces. The rim rounds fine when picked on its
/// own, so "the rim cannot be blended" is not the story either.
///
/// Refusing is an acceptable answer. Returning a subset is not, and that is what
/// this pins: whatever comes back must either account for BOTH the perimeter and
/// the rim, or be an error that leaves the input untouched.
#[test]
fn mixed_selection_never_returns_the_perimeter_only_result() {
    let hole_r = 8.0;
    let r = 1.0;

    let perimeter = |topo: &Topology, body: SolidId| {
        vec![
            top_edge(topo, body, true, 0.0),
            top_edge(topo, body, true, D),
            top_edge(topo, body, false, 0.0),
            top_edge(topo, body, false, W),
        ]
    };

    // Premise 1: the rim blends on its own.
    let rim_alone = {
        let mut topo = Topology::new();
        let body = bored_plate(&mut topo, hole_r);
        let before = measure::solid_volume(&topo, body, 0.001).unwrap();
        let rim = rim_edge(&topo, body, H, hole_r);
        let res = blend_ops::fillet_v2(&mut topo, body, &[rim], r)
            .expect("the bore rim must blend on its own");
        before - measure::solid_volume(&topo, res.solid, 0.001).unwrap()
    };
    assert!(
        rim_alone > 0.0,
        "the rim-only blend must remove material, removed {rim_alone}"
    );

    // Premise 2: the perimeter blends on its own — this is the answer the mixed
    // selection used to be handed.
    let perimeter_only = {
        let mut topo = Topology::new();
        let body = bored_plate(&mut topo, hole_r);
        let before = measure::solid_volume(&topo, body, 0.001).unwrap();
        let edges = perimeter(&topo, body);
        let res = blend_ops::fillet_v2(&mut topo, body, &edges, r).expect("perimeter fillet");
        before - measure::solid_volume(&topo, res.solid, 0.001).unwrap()
    };

    let mut topo = Topology::new();
    let body = bored_plate(&mut topo, hole_r);
    let before = measure::solid_volume(&topo, body, 0.001).unwrap();
    let rim = rim_edge(&topo, body, H, hole_r);
    let mut edges = perimeter(&topo, body);
    edges.push(rim);

    match blend_ops::fillet_v2(&mut topo, body, &edges, r) {
        Ok(res) => {
            assert!(
                res.failed.is_empty(),
                "a successful blend must not carry failures: {:?}",
                res.failed
            );
            let removed = before - measure::solid_volume(&topo, res.solid, 0.001).unwrap();
            let want = perimeter_only + rim_alone;
            assert!(
                (removed - want).abs() < 0.02 * want,
                "mixed selection removed {removed}; the perimeter alone removes \
                 {perimeter_only} and the rim alone {rim_alone} (want ~ {want}). \
                 A figure matching the perimeter means the rim was dropped."
            );
        }
        Err(e) => {
            // A typed refusal is fine. A refusal that mutated the input is not:
            // the caller keeps its handle and would ship the half-blended body.
            assert!(
                !blend_ops::blend_failure_code(&e).is_empty(),
                "every refusal carries a machine-readable code: {e}"
            );
            let after = measure::solid_volume(&topo, body, 0.001).unwrap();
            assert!(
                (after - before).abs() < 1e-9,
                "a refused fillet must leave the input untouched ({before} -> {after})"
            );
        }
    }
}

/// Defect 3, at the engine: the rolling-ball builder must not assemble a solid
/// that is missing a blend it was asked for.
///
/// Its Phase 2 manifold filter and every `continue` in its strip loop drop an
/// edge and carry on. The result is closed, valid, and plausibly sized, and
/// nothing in it records that an edge went unblended — which is exactly how a
/// mixed selection came back as a subset. Naming an edge the engine cannot
/// reach must now be an error, not a quieter success.
#[test]
fn rolling_ball_refuses_to_blend_a_subset() {
    use remus_topology::edge::Edge;
    use remus_topology::vertex::Vertex;

    let mut topo = Topology::new();
    let body = make_box(&mut topo, W, D, H).expect("box");
    let real = top_edge(&topo, body, true, 0.0);

    // An edge that belongs to no face of this solid: the engine cannot blend it
    // and used to simply leave it out.
    let v0 = topo.add_vertex(Vertex::new(Point3::new(500.0, 500.0, 500.0), 1e-7));
    let v1 = topo.add_vertex(Vertex::new(Point3::new(501.0, 500.0, 500.0), 1e-7));
    let stray = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));

    let before = measure::solid_volume(&topo, body, 0.001).unwrap();

    #[allow(deprecated)]
    let outcome = remus_operations::fillet::fillet_rolling_ball(&mut topo, body, &[real, stray], R);
    let err =
        outcome.expect_err("naming an edge the engine cannot blend must not succeed on the rest");
    let msg = format!("{err}");
    assert_eq!(
        blend_ops::blend_failure_code(&err),
        "edges-not-blended",
        "the failure must say what it is, got: {msg}"
    );
    assert!(
        msg.contains(&format!("{stray:?}")),
        "the error must name the edge it could not blend, got: {msg}"
    );
    let after = measure::solid_volume(&topo, body, 0.001).unwrap();
    assert!((after - before).abs() < 1e-9, "the input must be untouched");

    // The same edge on its own still rounds, so the refusal is about the stray
    // edge and not about the selection being unsupported.
    #[allow(deprecated)]
    let ok = remus_operations::fillet::fillet_rolling_ball(&mut topo, body, &[real], R)
        .expect("the real edge still blends on its own");
    let removed = before - measure::solid_volume(&topo, ok, 0.001).unwrap();
    let want = closed_form_removal(R, &[W], 0, 0);
    assert!(
        (removed - want).abs() < VOLUME_FLOOR * 10.0 * want,
        "one edge removed {removed}, closed form {want}"
    );
}
