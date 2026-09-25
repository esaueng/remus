//! B53 finding-17 neighbourhood regressions (2026-09-25).
//!
//! Roadmap row B53: the B26 proptest's excluded box × cone(r1 = 2.5) cell
//! (tool at `translation(0.5, -1.5, -0.5) · rotation_z(3π/2)`) carried 23
//! unit-scale CUT failures in two families, with two unrelated roots.
//!
//! - Tangent-rim family (box(dx, 1, dz) × cone(r0 = 3, h = dz + 0.5)): the
//!   cone's top rim lies in the box's top plane and is TANGENT to the box's
//!   `y = 1` top edge at `(0.5, 1, dz)`. The cut is two lumps that touch only
//!   there. EE paved the tangency on the edge and on the rim, but the box-top
//!   section (the rim, clipped to the face) ran straight through it, so the
//!   plane splitter saw one region pinched at a non-vertex and emitted both
//!   lobes of the top as one face whose wire revisits the tangency vertex
//!   (check-crate `WireSelfIntersection`). Fixed in the algo builder: a plane
//!   face's curved sections split at its boundary pave junctions. The split
//!   result is two lumps sharing one vertex, which the boolean gate then
//!   refused on three counts it had never met: whole-solid Euler counts the
//!   shared vertex once (3, not 2 · 2), the cut-safety probe sampled a
//!   crescent lobe's box centre (inside the cone), and the disjointness narrow
//!   phase clashed the untrimmed parametric rectangle a standalone per-face
//!   mesh gives a trimmed cone wall. The gate now balances Euler per
//!   component, re-probes from a certified interior point, and clashes
//!   trim-honouring component meshes, exempting contact confined to a shared
//!   vertex.
//! - Fine-mesh family (box(3, 3, 1) / (3.5, 2.5, 1) / (4, 2, 1) ×
//!   cone(r0 = 1, h = 2.5)): a valid B-Rep whose cone wall meshed with 87
//!   non-manifold edges at `bbox · 1e-5`. The non-planar CDT's base grid and
//!   its dense trim rows share their columns; at some deflections a dense row
//!   lands inside the vertex-merge cell of a base row, the two samples weld
//!   into one mesh vertex, and the triangles between them repeat edges. Fixed
//!   in the tessellator: one interior sample per merge cell.
//!
//! Every boolean runs `ExactOnly` with quality gating; no tolerance, healing
//! or mesh-fallback change. Volumes are pinned two ways: inclusion–exclusion
//! of the three legs' Gauss integrals against the closed-form operands, and
//! the cut's fine watertight mesh against an independent slice integral
//! (exact disc ∩ rectangle area per z).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI};

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cone};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

/// Axis centre of the placed cone (the finding-17 offset).
const AXIS: (f64, f64) = (0.5, -1.5);
/// Top radius shared by every cone of the neighbourhood.
const R1: f64 = 2.5;

fn placement() -> Mat4 {
    Mat4::translation(AXIS.0, AXIS.1, -0.5) * Mat4::rotation_z(3.0 * FRAC_PI_2)
}

/// Stock box at the origin and the placed tool cone in one arena.
fn operands(bx: (f64, f64, f64), r0: f64, h: f64) -> (Topology, SolidId, SolidId) {
    let mut topo = Topology::new();
    let a = make_box(&mut topo, bx.0, bx.1, bx.2).unwrap();
    let b = make_cone(&mut topo, r0, R1, h).unwrap();
    remus_operations::transform::transform_solid(&mut topo, b, &placement()).unwrap();
    (topo, a, b)
}

fn exact(bx: (f64, f64, f64), r0: f64, h: f64, op: BooleanOp) -> (Topology, SolidId) {
    let (mut topo, a, b) = operands(bx, r0, h);
    let outcome = boolean_with_context(
        &mut topo,
        op,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap_or_else(|e| panic!("{op:?} box{bx:?} cone({r0}, {h}): {e:?}"));
    assert!(
        matches!(outcome.quality, BooleanQuality::Exact),
        "{op:?} box{bx:?} cone({r0}, {h}): non-exact quality"
    );
    (topo, outcome.solid)
}

fn cone_volume(r0: f64, h: f64) -> f64 {
    PI * h * R1.mul_add(R1, r0.mul_add(r0, r0 * R1)) / 3.0
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / a.abs().max(b.abs()).max(1e-12)
}

/// Both validators, strictly: the operations validator (no relaxed retry) and
/// every check-crate ERROR except the by-edge-id shell connectivity check,
/// which rejects any multi-lump result by construction.
fn assert_strict_valid(topo: &Topology, s: SolidId, what: &str) {
    let strict = remus_operations::validate::validate_solid(topo, s).unwrap();
    assert!(
        strict.is_valid(),
        "{what}: ops validator: {:?}",
        strict
            .issues
            .iter()
            .map(|i| &i.description)
            .collect::<Vec<_>>()
    );
    let mut opts = remus_check::validate::ValidateOptions::default();
    opts.disabled_checks
        .insert(remus_check::validate::CheckId::ShellConnected);
    let report = remus_check::validate::validate_solid(topo, s, &opts).unwrap();
    let errors: Vec<_> = report
        .issues
        .iter()
        .filter(|i| i.severity == remus_check::validate::Severity::Error)
        .collect();
    assert!(errors.is_empty(), "{what}: check-crate errors: {errors:?}");
}

/// The B26 harness deflection: result bbox diagonal · 1e-5.
fn harness_deflection(topo: &Topology, s: SolidId) -> f64 {
    let bb = remus_operations::measure::solid_bounding_box(topo, s).unwrap();
    ((bb.max - bb.min).length() * 1e-5).max(1e-7)
}

fn assert_watertight_at(topo: &Topology, s: SolidId, d: f64, what: &str) {
    let mesh = tessellate_solid(topo, s, d).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0, "{what}: boundary at d={d:e}");
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "{what}: non-manifold at d={d:e}"
    );
}

fn assert_watertight(topo: &Topology, s: SolidId, what: &str) {
    for d in [0.1, 0.01, 1e-4, harness_deflection(topo, s)] {
        assert_watertight_at(topo, s, d, what);
    }
}

/// Rigid translation leaves the B-Rep's own mesh volume unchanged (1e-6 at
/// d = 1e-4) and the kernel's measured volume within the B26 oracle's 1 %.
/// (The measured bar stays at the B26 slack: on the general-position dy = 1.1
/// neighbour `solid_volume` takes the Gauss route for the NURBS-trimmed cone
/// wall and moves by 0.27 % under translation — a measurement defect present
/// on main, see row B53.)
fn assert_translation_invariant(topo: &Topology, s: SolidId, what: &str) {
    let mut moved = topo.clone();
    remus_operations::transform::transform_solid(
        &mut moved,
        s,
        &Mat4::translation(13.0, -7.0, 5.0),
    )
    .unwrap();
    let (m0, m1) = (mesh_volume(topo, s, 1e-4), mesh_volume(&moved, s, 1e-4));
    assert!(
        rel(m0, m1) <= 1e-6,
        "{what}: mesh volume moved {m0:.12} -> {m1:.12}"
    );
    for d in [0.1, 1e-4] {
        let v0 = solid_volume(topo, s, d).unwrap();
        let v1 = solid_volume(&moved, s, d).unwrap();
        assert!(
            rel(v0, v1) <= 1e-2,
            "{what}: volume moved {v0:.12} -> {v1:.12} at d={d}"
        );
    }
}

/// Signed-tetrahedron volume of the watertight mesh at deflection `d`.
fn mesh_volume(topo: &Topology, s: SolidId, d: f64) -> f64 {
    let mesh = tessellate_solid(topo, s, d).unwrap();
    let o = Point3::new(0.0, 0.0, 0.0);
    mesh.indices
        .chunks_exact(3)
        .map(|t| {
            let [p, q, r] = [t[0], t[1], t[2]].map(|i| mesh.positions[i as usize] - o);
            p.dot(q.cross(r)) / 6.0
        })
        .sum()
}

/// Exact area of `[x0, x1] × [y0, y1]` ∩ the disc of radius `r` about `AXIS`.
fn rect_disc_area(x0: f64, x1: f64, y0: f64, y1: f64, r: f64) -> f64 {
    let (cx, cy) = AXIS;
    let half = |x: f64| (r * r - (x - cx) * (x - cx)).max(0.0).sqrt();
    // Antiderivative of the half-chord: ∫ sqrt(r² − (x − cx)²) dx.
    let prim = |x: f64| {
        let t = ((x - cx) / r).clamp(-1.0, 1.0);
        0.5 * r * r * (t * (1.0 - t * t).max(0.0).sqrt() + t.asin())
    };
    let mut cuts = vec![x0, x1];
    for k in [y0 - cy, y1 - cy] {
        if k.abs() <= r {
            let w = (r * r - k * k).sqrt();
            cuts.extend([cx - w, cx + w]);
        }
    }
    cuts.extend([cx - r, cx + r]);
    cuts.retain(|x| (x0..=x1).contains(x));
    cuts.sort_by(f64::total_cmp);
    let mut area = 0.0;
    for w in cuts.windows(2) {
        let (a, b) = (w[0], w[1]);
        if b - a <= 0.0 {
            continue;
        }
        let m = 0.5 * (a + b);
        let s = half(m);
        if (m - cx).abs() >= r {
            continue;
        }
        let upper_is_chord = cy + s < y1;
        let lower_is_chord = cy - s > y0;
        let upper = if upper_is_chord { cy + s } else { y1 };
        let lower = if lower_is_chord { cy - s } else { y0 };
        if upper <= lower {
            continue;
        }
        let chord_integral = prim(b) - prim(a);
        let mut piece = 0.0;
        piece += if upper_is_chord {
            cy.mul_add(b - a, chord_integral)
        } else {
            y1 * (b - a)
        };
        piece -= if lower_is_chord {
            cy.mul_add(b - a, -chord_integral)
        } else {
            y0 * (b - a)
        };
        area += piece;
    }
    area
}

/// Independent slice integral of the box ∖ cone volume: the exact disc ∩
/// rectangle area per z, integrated by 4000-panel 5-point Gauss–Legendre.
fn slice_cut_volume(bx: (f64, f64, f64), r0: f64, h: f64) -> f64 {
    const NODES: [(f64, f64); 5] = [
        (0.0, 0.568_888_888_888_888_9),
        (-0.538_469_310_105_683_1, 0.478_628_670_499_366_5),
        (0.538_469_310_105_683_1, 0.478_628_670_499_366_5),
        (-0.906_179_845_938_664, 0.236_926_885_056_189_1),
        (0.906_179_845_938_664, 0.236_926_885_056_189_1),
    ];
    let (z_lo, z_hi) = (-0.5, h - 0.5);
    let (a, b) = (0.0_f64.max(z_lo), bx.2.min(z_hi));
    let covered = |z: f64| {
        let r = (R1 - r0).mul_add((z + 0.5) / h, r0);
        rect_disc_area(0.0, bx.0, 0.0, bx.1, r)
    };
    let panels = 4000;
    let mut inside = 0.0;
    if b > a {
        let step = (b - a) / f64::from(panels);
        for k in 0..panels {
            let mid = (f64::from(k) + 0.5).mul_add(step, a);
            for (x, w) in NODES {
                inside += w * 0.5 * step * covered((0.5 * step).mul_add(x, mid));
            }
        }
    }
    bx.0 * bx.1 * bx.2 - inside
}

/// The cut leg of one member: exact, analytic, strictly valid, watertight at
/// every deflection, translation-invariant, and — through its mesh at the
/// harness deflection — within 2e-4 of the stock volume of the slice
/// integral.
fn assert_cut(bx: (f64, f64, f64), r0: f64, h: f64) -> (Topology, SolidId) {
    let what = format!("box{bx:?} x cone({r0}, {R1}, {h}) cut");
    let (topo, cut) = exact(bx, r0, h, BooleanOp::Cut);
    assert_strict_valid(&topo, cut, &what);
    assert_watertight(&topo, cut, &what);
    assert_translation_invariant(&topo, cut, &what);
    for f in solid_faces(&topo, cut).unwrap() {
        let tag = topo.face(f).unwrap().surface().type_tag();
        assert!(
            tag == "plane" || tag == "cone",
            "{what}: non-analytic face {tag}"
        );
    }
    let box_volume = bx.0 * bx.1 * bx.2;
    let (mesh, slices) = (
        mesh_volume(&topo, cut, harness_deflection(&topo, cut)),
        slice_cut_volume(bx, r0, h),
    );
    assert!(
        (mesh - slices).abs() <= 2e-4 * box_volume,
        "{what}: mesh volume {mesh:.9} vs slice integral {slices:.9}"
    );
    (topo, cut)
}

/// [`assert_cut`] plus the intersect and fuse legs, each exact and strictly
/// valid, with the three legs' fine-mesh volumes consistent with the closed
/// forms and the cut's with the slice integral.
fn assert_member(bx: (f64, f64, f64), r0: f64, h: f64) -> (Topology, SolidId) {
    assert_member_at(bx, r0, h, 2e-5, 1e-5)
}

/// [`assert_member`] with the mesh deflection `d` and the volume band as a
/// fraction of the stock (`band`) chosen by the caller.
fn assert_member_at(
    bx: (f64, f64, f64),
    r0: f64,
    h: f64,
    d: f64,
    band: f64,
) -> (Topology, SolidId) {
    let (topo, cut) = assert_cut(bx, r0, h);
    let what = format!("box{bx:?} x cone({r0}, {R1}, {h})");
    let (ti, inter) = exact(bx, r0, h, BooleanOp::Intersect);
    assert_strict_valid(&ti, inter, &format!("{what} intersect"));
    let (tf, fuse) = exact(bx, r0, h, BooleanOp::Fuse);
    assert_strict_valid(&tf, fuse, &format!("{what} fuse"));

    // Material identities and the independent slice integral, all through the
    // B-Rep's own fine watertight meshes. (Not through the Gauss integral: on
    // these NURBS-trimmed cone walls `mass_properties` reads up to 1 % high on
    // the smallest pieces and moves under rigid translation — a measurement
    // defect present on main, general-position members included.)
    let (vc, vi, vf) = (
        mesh_volume(&topo, cut, d),
        mesh_volume(&ti, inter, d),
        mesh_volume(&tf, fuse, d),
    );
    let box_volume = bx.0 * bx.1 * bx.2;
    let both = box_volume + cone_volume(r0, h);
    assert!(
        (vc + vi - box_volume).abs() <= band * box_volume,
        "{what}: cut {vc:.9} + inter {vi:.9} != box {box_volume:.9}"
    );
    assert!(
        (vf + vi - both).abs() <= band * both,
        "{what}: fuse {vf:.9} + inter {vi:.9} != box + cone {both:.9}"
    );
    let slices = slice_cut_volume(bx, r0, h);
    assert!(
        (vc - slices).abs() <= band * box_volume,
        "{what}: cut mesh volume {vc:.9} vs slice integral {slices:.9}"
    );
    (topo, cut)
}

fn classify(topo: &Topology, s: SolidId, p: Point3) -> PointClassification {
    classify_point(topo, s, p, &ClassifyOptions::default()).unwrap()
}

/// Edge-connected face components of the result's outer shell and the
/// vertices each one uses.
fn lumps(topo: &Topology, s: SolidId) -> Vec<std::collections::BTreeSet<usize>> {
    let faces = solid_faces(topo, s).unwrap();
    let edges_of = |f| {
        let face = topo.face(f).unwrap();
        std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .flat_map(|w| topo.wire(w).unwrap().edges().to_vec())
            .map(|oe| oe.edge())
            .collect::<Vec<_>>()
    };
    let mut component = vec![usize::MAX; faces.len()];
    let mut count = 0;
    for seed in 0..faces.len() {
        if component[seed] != usize::MAX {
            continue;
        }
        component[seed] = count;
        let mut stack = vec![seed];
        while let Some(i) = stack.pop() {
            let mine = edges_of(faces[i]);
            for j in 0..faces.len() {
                if component[j] == usize::MAX && edges_of(faces[j]).iter().any(|e| mine.contains(e))
                {
                    component[j] = count;
                    stack.push(j);
                }
            }
        }
        count += 1;
    }
    (0..count)
        .map(|c| {
            faces
                .iter()
                .zip(&component)
                .filter(|&(_, &k)| k == c)
                .flat_map(|(&f, _)| edges_of(f))
                .flat_map(|e| {
                    let edge = topo.edge(e).unwrap();
                    [edge.start().index(), edge.end().index()]
                })
                .collect()
        })
        .collect()
}

/// The box top (plane z = `top`) comes out as one face per lobe, and no face's
/// outer wire revisits a vertex (the pinched-wire signature).
fn assert_top_split_without_pinch(topo: &Topology, s: SolidId, top: f64) {
    let faces = solid_faces(topo, s).unwrap();
    let tops = faces
        .iter()
        .filter(|&&f| {
            let face = topo.face(f).unwrap();
            matches!(face.surface(), FaceSurface::Plane { normal, .. } if normal.z().abs() > 0.5)
                && topo
                    .wire(face.outer_wire())
                    .unwrap()
                    .edges()
                    .iter()
                    .all(|oe| {
                        let v = topo.edge(oe.edge()).unwrap().start();
                        (topo.vertex(v).unwrap().point().z() - top).abs() < 1e-9
                    })
        })
        .count();
    assert_eq!(
        tops, 2,
        "the box top (z = {top}) splits into one face per lobe"
    );
    for &f in &faces {
        let wire = topo.wire(topo.face(f).unwrap().outer_wire()).unwrap();
        let mut starts: Vec<usize> = wire
            .edges()
            .iter()
            .map(|oe| oe.oriented_start(topo.edge(oe.edge()).unwrap()).index())
            .collect();
        starts.sort_unstable();
        let n = starts.len();
        starts.dedup();
        assert_eq!(starts.len(), n, "face {f:?}: outer wire revisits a vertex");
    }
}

/// The pinned witness (`b26_finding17_nbhd_coplanar_cap_cut`): the cut is two
/// lumps sharing exactly the tangency vertex, the box top splits into one face
/// per lobe (no wire revisits a vertex), and ray-cast probes land in both
/// lobes and not in the removed material.
#[test]
fn b53_tangent_rim_cut_is_two_lumps_touching_at_the_tangency() {
    let bx = (2.5, 1.0, 2.0);
    let (topo, cut) = assert_member(bx, 3.0, 2.5);

    assert_eq!(
        solid_faces(&topo, cut).unwrap().len(),
        9,
        "7 planes + 2 cone walls"
    );
    assert_top_split_without_pinch(&topo, cut, bx.2);

    let lumps = lumps(&topo, cut);
    assert_eq!(lumps.len(), 2, "two lumps");
    assert_eq!(
        lumps[0].intersection(&lumps[1]).count(),
        1,
        "the lumps share exactly one vertex"
    );

    for p in [
        Point3::new(2.4, 0.95, 1.0),   // corner lobe
        Point3::new(2.45, 0.2, 1.9),   // corner lobe, near the rim
        Point3::new(0.05, 0.99, 1.95), // tip lobe
    ] {
        assert_eq!(
            classify(&topo, cut, p),
            PointClassification::Inside,
            "{p:?}"
        );
    }
    for p in [
        Point3::new(1.5, 0.5, 1.0),  // inside the cone
        Point3::new(0.5, 0.9, 1.99), // just under the tangency
    ] {
        assert_eq!(
            classify(&topo, cut, p),
            PointClassification::Outside,
            "{p:?}"
        );
    }
}

/// The whole tangent-rim family: every box(dx ∈ [1, 4], 1, dz ∈ {1.5, 2,
/// 2.5}) × cone(r0 = 3, h = dz + 0.5) — the rim tangent to the `y = 1` top
/// edge at `(0.5, 1, dz)` — passes every oracle, except the one member that
/// refuses typed on main too.
#[test]
fn b53_tangent_rim_family_is_exact_and_valid() {
    for dx in 2u8..=8 {
        for dz in [1.5, 2.0, 2.5] {
            let bx = (f64::from(dx) * 0.5, 1.0, dz);
            if dx == 6 && dz > 2.25 {
                // The 21st member refuses typed, before and after B53: the
                // rim also crosses the box's x = 3, y = 0 corner edge 0.007
                // above the floor, and the wire builder discards the corner
                // lobe's cone wall there. A refusal, not a wrong answer.
                let (mut topo, a, b) = operands(bx, 3.0, dz + 0.5);
                let refused = boolean_with_context(
                    &mut topo,
                    BooleanOp::Cut,
                    a,
                    b,
                    &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
                );
                assert!(
                    matches!(
                        refused,
                        Err(remus_operations::OperationsError::ExactOnlyUnattainable)
                    ),
                    "box{bx:?}: expected the typed refusal"
                );
                continue;
            }
            let (topo, cut) = assert_cut(bx, 3.0, dz + 0.5);
            assert_eq!(lumps(&topo, cut).len(), 2, "dx={dx} dz={dz}");
            assert_top_split_without_pinch(&topo, cut, dz);
        }
    }
}

/// The same tangency with the cone widening upward (r0 = 2): the plane of the
/// `y = 1` wall meets the cone only at the rim tangency, so the lobes join
/// below the top and the cut is ONE lump whose top still splits in two. The
/// EF scan of the box edge against the cone wall solved the grazing contact
/// 1.6e-7 along the edge from the EE vertex (sqrt-of-residual), minting a
/// near-duplicate vertex that paired the section and the rim at different
/// points; EF now snaps onto the edge's own EE pave. (Refused typed before
/// B53; an ops-invalid exact cut once the top split landed without the snap.)
#[test]
fn b53_tangent_rim_single_lump_family() {
    for dz in [1.5, 2.0, 2.5] {
        let bx = (4.0, 1.0, dz);
        let (topo, cut) = assert_cut(bx, 2.0, dz + 0.5);
        assert_eq!(lumps(&topo, cut).len(), 1, "dz={dz}");
        assert_top_split_without_pinch(&topo, cut, dz);
    }
    // At d = 1e-4: finer, this cone wall drops its dense trim rows (see
    // `b53_single_lump_cut_fine_mesh_volume`).
    assert_member_at((4.0, 1.0, 2.0), 2.0, 2.5, 1e-4, 1e-4);
}

/// Ready-repro (B53 remainder): the single-lump tangent cut's cone wall
/// meshes sparse below d ≈ 8e-5. The non-planar CDT skips its dense trim rows
/// once they would exceed the boundary-test budget, so the wall drops from
/// 25,328 triangles at d = 1e-4 to 563 at 7e-5 with the same area: watertight,
/// but the chords sag into the cone and the cut's mesh reads 5.2920 against
/// the exact 5.2594 (slice integral = Gauss). The B-Rep is exact; the
/// harness deflection (1e-4 here) is clean. The fix belongs to the mesher's
/// budget fallback, not to this boolean.
#[test]
#[ignore = "open: cone wall meshes sparse below d = 8e-5 when the dense trim rows exceed the budget (B53)"]
fn b53_single_lump_cut_fine_mesh_volume() {
    let bx = (4.0, 1.0, 2.0);
    let (topo, cut) = exact(bx, 2.0, 2.5, BooleanOp::Cut);
    let slices = slice_cut_volume(bx, 2.0, 2.5);
    for d in [7e-5, 2e-5] {
        let mesh = mesh_volume(&topo, cut, d);
        assert!(
            (mesh - slices).abs() <= 1e-4 * 8.0,
            "d={d:e}: mesh volume {mesh:.9} vs slice integral {slices:.9}"
        );
    }
}

/// General position on both sides of the tangency take the unchanged paths:
/// a narrower box (the rim crosses the top edge at x = 0.5 ± 0.7, so the tip
/// lobe vanishes — one lump) and a deeper one (the lobes join past the rim —
/// one lump).
#[test]
fn b53_tangent_rim_general_position_neighbours() {
    for dy in [0.9, 1.1] {
        let (topo, cut) = assert_member((2.5, dy, 2.0), 3.0, 2.5);
        assert_eq!(lumps(&topo, cut).len(), 1, "dy={dy}");
    }
}

/// The raw GFA cut (below the operations gate) already splits the box top at
/// the tangency: the builder, not a later pass, owns the split.
#[test]
fn b53_raw_gfa_splits_the_top_at_the_tangency() {
    let (mut topo, a, b) = operands((2.5, 1.0, 2.0), 3.0, 2.5);
    let s = remus_algo::gfa::boolean(&mut topo, remus_algo::bop::BooleanOp::Cut, a, b).unwrap();
    assert_eq!(solid_faces(&topo, s).unwrap().len(), 9);
    assert_strict_valid(&topo, s, "raw GFA cut");
}

/// The fine-mesh family (`b26_finding17_nbhd_cut_mesh`): the cone wall meshes
/// manifold at the harness deflection and across the band of deflections
/// where a dense trim row met a base grid row (7.70e-5 to 7.82e-5 on the
/// pinned member).
#[test]
fn b53_fine_mesh_family_is_watertight() {
    for bx in [(3.0, 3.0, 1.0), (3.5, 2.5, 1.0), (4.0, 2.0, 1.0)] {
        let (topo, cut) = assert_member(bx, 1.0, 2.5);
        // 7.9e-5 down to 7.6e-5 in 0.5 % steps.
        for step in 0..8 {
            let d = 7.9e-5 * 0.995_f64.powi(step);
            assert_watertight_at(&topo, cut, d, &format!("box{bx:?} cut"));
        }
    }
}

/// Corner-touching cubes: the same vertex-sharing lump pair from a fuse, with
/// no curved geometry at all — the gate's component-wise Euler balance, not
/// anything cone-specific, is what accepts it.
#[test]
fn b53_corner_touching_cubes_fuse_exact() {
    let mut topo = Topology::new();
    let a = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let b = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    remus_operations::transform::transform_solid(&mut topo, b, &Mat4::translation(1.0, 1.0, 1.0))
        .unwrap();
    let outcome = boolean_with_context(
        &mut topo,
        BooleanOp::Fuse,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap();
    assert!(matches!(outcome.quality, BooleanQuality::Exact));
    let s = outcome.solid;
    assert_strict_valid(&topo, s, "corner-touch fuse");
    assert_watertight(&topo, s, "corner-touch fuse");
    assert_eq!(solid_faces(&topo, s).unwrap().len(), 12);
    let pieces = lumps(&topo, s);
    assert_eq!(pieces.len(), 2);
    assert_eq!(pieces[0].intersection(&pieces[1]).count(), 1);
    assert!(rel(mesh_volume(&topo, s, 1e-4), 2.0) <= 1e-12);
}
