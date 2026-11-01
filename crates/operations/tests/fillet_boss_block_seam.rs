//! Fillets and chamfers along the whole seam where a round boss meets a
//! block.
//!
//! A cylinder fused onto the corner of a block, and rising past its top,
//! meets the block in three concave edges: a ruling up the block's front
//! wall, a quarter circle across its top and a ruling down its side wall.
//! Rounding or bevelling all three at once is one feature in any CAD tool,
//! but the edges turn two sharp corners of the block: the walking builder
//! refused the selection (`unsupported-vertex-blend`), the arc on its own
//! (`trimming-failure`, its contact ends inside the boss) and every chamfer of
//! the seam.
//!
//! Each run gets an exact band — a cylinder or a plane up a wall, a torus or a
//! cone across the top — and at each corner the two bands carry on until they
//! meet in a mitre. These tests hold the bands to their closed forms, the
//! mitre to where the two rolling balls' envelopes cross, and the material
//! added to the closed-form filled corners rather than to the kernel's own
//! measurements of itself.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::{FRAC_PI_2, PI};

use remus_math::vec::Point3;
use remus_operations::blend_ops::{
    blend_failure_code, chamfer_v2, display_only_blends, fillet_cascade, set_display_only_blends,
};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces, solid_vertices};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const BOSS: f64 = 6.0;
const BLOCK: (f64, f64, f64) = (30.0, 18.0, 18.0);
const BOSS_HEIGHT: f64 = 28.0;
const MEASURE_DEFLECTION: f64 = 0.005;

/// The boss on the block's corner, rising 10 past its top.
fn native_body(topo: &mut Topology) -> SolidId {
    let boss = make_cylinder(topo, BOSS, BOSS_HEIGHT).unwrap();
    let block = make_box(topo, BLOCK.0, BLOCK.1, BLOCK.2).unwrap();
    boolean(topo, BooleanOp::Fuse, boss, block).unwrap()
}

/// The same body as a user's STEP export: the boss is 28.5 tall with an r1.8
/// round on its top rim, and its parameter seam runs up from the corner
/// vertex where the front ruling meets the arc.
fn imported_body(topo: &mut Topology) -> SolidId {
    remus_io::step::reader::read_step(
        include_str!("data/box_corner_rounded_cylinder_boss.step"),
        topo,
    )
    .unwrap()[0]
}

fn native_volume() -> f64 {
    BLOCK.0 * BLOCK.1 * BLOCK.2
        + 0.75 * PI * BOSS * BOSS * BLOCK.2
        + PI * BOSS * BOSS * (BOSS_HEIGHT - BLOCK.2)
}

fn edge_between(topo: &Topology, solid: SolidId, a: Point3, b: Point3) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            ((start - a).length() < 1e-6 && (end - b).length() < 1e-6)
                || ((start - b).length() < 1e-6 && (end - a).length() < 1e-6)
        })
        .unwrap_or_else(|| panic!("edge {a:?} -- {b:?}"))
}

/// The seam: up the front wall, across the top, down the side wall.
fn seam(topo: &Topology, solid: SolidId) -> [EdgeId; 3] {
    let p = Point3::new;
    [
        edge_between(topo, solid, p(BOSS, 0.0, 0.0), p(BOSS, 0.0, BLOCK.2)),
        edge_between(topo, solid, p(BOSS, 0.0, BLOCK.2), p(0.0, BOSS, BLOCK.2)),
        edge_between(topo, solid, p(0.0, BOSS, BLOCK.2), p(0.0, BOSS, 0.0)),
    ]
}

fn has_vertex(topo: &Topology, solid: SolidId, at: Point3) -> bool {
    solid_vertices(topo, solid)
        .unwrap()
        .into_iter()
        .any(|v| (topo.vertex(v).unwrap().point() - at).length() < 1e-6)
}

fn has_edge(topo: &Topology, solid: SolidId, a: Point3, b: Point3) -> bool {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .any(|edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            ((start - a).length() < 1e-6 && (end - b).length() < 1e-6)
                || ((start - b).length() < 1e-6 && (end - a).length() < 1e-6)
        })
}

fn assert_valid(topo: &Topology, solid: SolidId, label: &str) {
    let shell = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    assert!(
        validate_shell_closed(shell, topo).is_ok(),
        "{label}: open shell"
    );
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {:?}", report.issues);
}

/// `∮ x dy − y dx` along a circle arc of radius `rad` about `(cx, cy)` from
/// angle `t0` to `t1`.
fn arc_moment(cx: f64, cy: f64, rad: f64, t0: f64, t1: f64) -> f64 {
    rad * rad * (t1 - t0) + rad * (cx * (t1.sin() - t0.sin()) - cy * (t1.cos() - t0.cos()))
}

/// The filled corner where a wall through the boss's axis meets it, rounded
/// by a ball of radius `r` (Green's theorem round the corner's boundary).
fn ruling_fillet_section(r: f64) -> f64 {
    let xc = ((BOSS + r).powi(2) - r * r).sqrt();
    let ball = arc_moment(xc, -r, r, PI / 2.0, r.atan2(-xc));
    let cylinder = arc_moment(0.0, 0.0, BOSS, (-r).atan2(xc), 0.0);
    0.5 * (ball + cylinder).abs()
}

/// The quarter ring rounding the boss's foot on the block's top: the
/// corner's area `(1 − π/4)·r²` swept a quarter turn about the axis at its
/// centroid's radius (Pappus).
fn cap_fillet_volume(r: f64) -> f64 {
    let centroid = BOSS + r * (10.0 - 3.0 * PI) / (3.0 * (4.0 - PI));
    (1.0 - PI / 4.0) * r * r * centroid * FRAC_PI_2
}

/// The chamfered corner up a wall: the triangle from the ruling to the two
/// contacts `d` along the wall and `d` round the boss, less the sliver of
/// boss between the chord and the arc.
fn ruling_chamfer_section(d: f64) -> f64 {
    let angle = d / BOSS;
    0.5 * d * BOSS * angle.sin() - 0.5 * BOSS * BOSS * (angle - angle.sin())
}

/// The quarter cone ring bevelling the boss's foot (Pappus).
fn cap_chamfer_volume(d: f64) -> f64 {
    0.5 * d * d * (BOSS + d / 3.0) * FRAC_PI_2
}

/// The material one mitred corner adds past the two runs' own fillings.
///
/// Past the corner each band carries on until it meets the other, so the
/// corner holds what lies behind both: inside the wall run's filled section
/// (in plan, beside the front wall: `y < 0`) and under the top run's filled
/// section (in each point's own meridian), above the block's top. Integrated
/// over the wall section in plan, the height of the top section at that
/// radius.
fn mitre_corner_volume(size: f64, chamfer: bool) -> f64 {
    let n = 1200;
    let (x0, x1) = (BOSS * 0.5, BOSS + size);
    let (y0, y1) = (-size, 0.0);
    let (hx, hy) = ((x1 - x0) / f64::from(n), (y1 - y0) / f64::from(n));
    let xf = ((BOSS + size).powi(2) - size * size).sqrt();
    // The wall section's boundary away from the corner, as a test on which
    // side of it a point lies.
    let in_wall_section = |x: f64, y: f64| -> bool {
        if x.hypot(y) < BOSS || y > 0.0 {
            return false;
        }
        if chamfer {
            let angle = size / BOSS;
            let (rx, ry) = (BOSS * angle.cos(), -BOSS * angle.sin());
            let (sx, sy) = (BOSS + size, 0.0);
            // The ruling `(BOSS, 0)` is on the material side of the bevel.
            let side = |px: f64, py: f64| (sx - rx) * (py - ry) - (sy - ry) * (px - rx);
            side(x, y) * side(BOSS, 0.0) > 0.0
        } else {
            let (cx, cy) = (xf, -size);
            if (x - cx).hypot(y - cy) < size {
                return false;
            }
            // Between the ball's two contacts, on the ruling's side of their
            // chord.
            let (px, py) = (BOSS * cx / (BOSS + size), BOSS * cy / (BOSS + size));
            let side = |qx: f64, qy: f64| (xf - px) * (qy - py) - (0.0 - py) * (qx - px);
            side(x, y) * side(BOSS, 0.0) > 0.0
        }
    };
    // Height of the top run's filled section above the block at radius `rho`.
    let top_height = |rho: f64| -> f64 {
        let out = rho - BOSS;
        if !(0.0..=size).contains(&out) {
            return 0.0;
        }
        if chamfer {
            size - out
        } else {
            size - (size * size - (size - out).powi(2)).sqrt()
        }
    };
    let mut volume = 0.0;
    for i in 0..n {
        let x = (f64::from(i) + 0.5).mul_add(hx, x0);
        for j in 0..n {
            let y = (f64::from(j) + 0.5).mul_add(hy, y0);
            if in_wall_section(x, y) {
                volume += top_height(x.hypot(y));
            }
        }
    }
    volume * hx * hy
}

/// Where the wall band's contact with the front wall meets the top band,
/// and where the top band's contact with the block's top leaves its front
/// edge: the mitre's foot `m` and the block edge's new end `k`.
fn fillet_corner(r: f64) -> (Point3, Point3) {
    let xf = ((BOSS + r).powi(2) - r * r).sqrt();
    let zm = BLOCK.2 + r - (r * r - (BOSS + r - xf).powi(2)).sqrt();
    (
        Point3::new(xf, 0.0, zm),
        Point3::new(BOSS + r, 0.0, BLOCK.2),
    )
}

#[test]
fn fillet_the_whole_seam_mitres_both_corners() {
    for (label, make) in [
        ("native", native_body as fn(&mut Topology) -> SolidId),
        ("imported", imported_body),
    ] {
        for r in [1.0, 2.6, 5.0] {
            let label = format!("{label} r{r}");
            let mut topo = Topology::new();
            let body = make(&mut topo);
            let edges = seam(&topo, body);
            let faces_before = solid_faces(&topo, body).unwrap();
            let before = solid_volume(&topo, body, MEASURE_DEFLECTION).unwrap();

            let result = fillet_cascade(&mut topo, body, &edges, r)
                .unwrap_or_else(|e| panic!("{label}: {}: {e}", blend_failure_code(&e)));
            assert!(result.failed.is_empty(), "{label}: partial blend");
            assert_valid(&topo, result.solid, &label);

            // Three exact bands: a cylinder up each wall, a torus across the
            // top.
            let created: Vec<FaceSurface> = solid_faces(&topo, result.solid)
                .unwrap()
                .into_iter()
                .filter(|f| !faces_before.contains(f))
                .map(|f| topo.face(f).unwrap().surface().clone())
                .collect();
            let cylinders = created
                .iter()
                .filter(|s| matches!(s, FaceSurface::Cylinder(c) if (c.radius() - r).abs() < 1e-12))
                .count();
            let tori = created
                .iter()
                .filter(|s| {
                    matches!(s, FaceSurface::Torus(t)
                        if (t.minor_radius() - r).abs() < 1e-12
                            && (t.major_radius() - (BOSS + r)).abs() < 1e-12)
                })
                .count();
            assert_eq!((cylinders, tori), (2, 1), "{label}: bands");

            // The mitres end on the walls just above the block's top, and the
            // block's sharp top edges now start where the torus leaves them.
            let (m, k) = fillet_corner(r);
            let swap = |p: Point3| Point3::new(p.y(), p.x(), p.z());
            for at in [m, k, swap(m), swap(k)] {
                assert!(
                    has_vertex(&topo, result.solid, at),
                    "{label}: no vertex at {at:?}"
                );
            }
            assert!(
                has_edge(&topo, result.solid, k, Point3::new(BLOCK.0, 0.0, BLOCK.2)),
                "{label}: front top edge"
            );

            // The body gains the two wall corners and the quarter ring, plus
            // what the mitres add where the bands carry on past the corners. The imported boss's round only changes what
            // the input measures, not what the fillet adds.
            if label.starts_with("native") {
                let filled = 2.0 * ruling_fillet_section(r) * BLOCK.2 + cap_fillet_volume(r);
                let gained =
                    solid_volume(&topo, result.solid, MEASURE_DEFLECTION).unwrap() - before;
                assert!(
                    (native_volume() - before).abs() < 1e-6 * before,
                    "{label}: input measures {before}"
                );
                let mitres = 2.0 * mitre_corner_volume(r, false);
                assert!(
                    (gained - filled - mitres).abs() < 2e-3 * mitres + 1e-6 * before,
                    "{label}: gained {gained}, corners {filled}, mitres {mitres}"
                );
            }
        }
    }
}

#[test]
fn chamfer_the_whole_seam_mitres_both_corners() {
    for (label, make) in [
        ("native", native_body as fn(&mut Topology) -> SolidId),
        ("imported", imported_body),
    ] {
        for d in [1.0, 2.6, 5.0] {
            let label = format!("{label} d{d}");
            let mut topo = Topology::new();
            let body = make(&mut topo);
            let edges = seam(&topo, body);
            let faces_before = solid_faces(&topo, body).unwrap();
            let before = solid_volume(&topo, body, MEASURE_DEFLECTION).unwrap();

            let result = chamfer_v2(&mut topo, body, &edges, d, d)
                .unwrap_or_else(|e| panic!("{label}: {}: {e}", blend_failure_code(&e)));
            assert!(result.failed.is_empty(), "{label}: partial blend");
            assert_valid(&topo, result.solid, &label);

            let created: Vec<FaceSurface> = solid_faces(&topo, result.solid)
                .unwrap()
                .into_iter()
                .filter(|f| !faces_before.contains(f))
                .map(|f| topo.face(f).unwrap().surface().clone())
                .collect();
            let cones = created
                .iter()
                .filter(|s| matches!(s, FaceSurface::Cone(_)))
                .count();
            assert_eq!(cones, 1, "{label}: one cone across the top");

            // An equal chamfer's mitre lands exactly on the block's edge,
            // `d` from the boss on both walls.
            for at in [
                Point3::new(BOSS + d, 0.0, BLOCK.2),
                Point3::new(0.0, BOSS + d, BLOCK.2),
            ] {
                assert!(
                    has_vertex(&topo, result.solid, at),
                    "{label}: no vertex at {at:?}"
                );
            }

            if label.starts_with("native") {
                let filled = 2.0 * ruling_chamfer_section(d) * BLOCK.2 + cap_chamfer_volume(d);
                let gained =
                    solid_volume(&topo, result.solid, MEASURE_DEFLECTION).unwrap() - before;
                let mitres = 2.0 * mitre_corner_volume(d, true);
                assert!(
                    (gained - filled - mitres).abs() < 2e-3 * mitres + 1e-6 * before,
                    "{label}: gained {gained}, corners {filled}, mitres {mitres}"
                );
            }
        }
    }
}

/// The arc on its own stops above both walls: each wall grows to the
/// torus's end section, and the body gains exactly the quarter ring.
#[test]
fn fillet_and_chamfer_across_the_top_alone() {
    for (label, make) in [
        ("native", native_body as fn(&mut Topology) -> SolidId),
        ("imported", imported_body),
    ] {
        for r in [1.0, 3.0] {
            for chamfer in [false, true] {
                let label = format!("{label} {} {r}", if chamfer { "chamfer" } else { "fillet" });
                let mut topo = Topology::new();
                let body = make(&mut topo);
                let arc = seam(&topo, body)[1];
                let before = solid_volume(&topo, body, MEASURE_DEFLECTION).unwrap();
                let result = if chamfer {
                    chamfer_v2(&mut topo, body, &[arc], r, r)
                } else {
                    fillet_cascade(&mut topo, body, &[arc], r)
                }
                .unwrap_or_else(|e| panic!("{label}: {}: {e}", blend_failure_code(&e)));
                assert_valid(&topo, result.solid, &label);
                for at in [
                    Point3::new(BOSS + r, 0.0, BLOCK.2),
                    Point3::new(BOSS, 0.0, BLOCK.2 + r),
                    Point3::new(0.0, BOSS + r, BLOCK.2),
                    Point3::new(0.0, BOSS, BLOCK.2 + r),
                ] {
                    assert!(
                        has_vertex(&topo, result.solid, at),
                        "{label}: no vertex at {at:?}"
                    );
                }
                if label.starts_with("native") {
                    let gained =
                        solid_volume(&topo, result.solid, MEASURE_DEFLECTION).unwrap() - before;
                    let expected = if chamfer {
                        cap_chamfer_volume(r)
                    } else {
                        cap_fillet_volume(r)
                    };
                    assert!(
                        (gained - expected).abs() < 1e-6 * before,
                        "{label}: gained {gained}, expected {expected}"
                    );
                }
            }
        }
    }
}

/// One wall and the top: a mitre at the front corner, the torus's end section
/// on the side wall.
#[test]
fn fillet_up_one_wall_and_across_the_top() {
    for r in [1.0, 2.6] {
        let mut topo = Topology::new();
        let body = native_body(&mut topo);
        let [front, arc, _] = seam(&topo, body);
        let result = fillet_cascade(&mut topo, body, &[front, arc], r)
            .unwrap_or_else(|e| panic!("r{r}: {}: {e}", blend_failure_code(&e)));
        assert_valid(&topo, result.solid, &format!("r{r}"));
        let (m, k) = fillet_corner(r);
        for at in [
            m,
            k,
            Point3::new(0.0, BOSS + r, BLOCK.2),
            Point3::new(0.0, BOSS, BLOCK.2 + r),
        ] {
            assert!(
                has_vertex(&topo, result.solid, at),
                "r{r}: no vertex at {at:?}"
            );
        }
    }
}

/// A lone ruling chamfer: the walking builder's chamfer refused it; the band
/// is a plane and its top ends in the block's top face.
#[test]
fn chamfer_up_one_wall() {
    for d in [1.0, 3.0] {
        let mut topo = Topology::new();
        let body = native_body(&mut topo);
        let front = seam(&topo, body)[0];
        let before = solid_volume(&topo, body, MEASURE_DEFLECTION).unwrap();
        let result = chamfer_v2(&mut topo, body, &[front], d, d)
            .unwrap_or_else(|e| panic!("d{d}: {}: {e}", blend_failure_code(&e)));
        assert_valid(&topo, result.solid, &format!("d{d}"));
        let gained = solid_volume(&topo, result.solid, MEASURE_DEFLECTION).unwrap() - before;
        let expected = ruling_chamfer_section(d) * BLOCK.2;
        assert!(
            (gained - expected).abs() < 1e-6 * before,
            "d{d}: gained {gained}, expected {expected}"
        );
    }
}

/// The section a convex round of radius `r` takes off a right-angled edge.
fn convex_round_section(r: f64) -> f64 {
    (1.0 - PI / 4.0) * r * r
}

/// The straight edge of `solid` lying on the segment `a`–`b`: what is left
/// of a block edge once an earlier fillet trimmed one end of it.
fn edge_along(topo: &Topology, solid: SolidId, a: Point3, b: Point3) -> EdgeId {
    let span = b - a;
    let along = span * (1.0 / span.length());
    let on = |q: Point3| {
        let t = (q - a).dot(along);
        ((q - a) - along * t).length() < 1e-6 && t > -1e-6 && t < span.length() + 1e-6
    };
    let found: Vec<EdgeId> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            matches!(edge.curve(), remus_topology::edge::EdgeCurve::Line)
                && on(topo.vertex(edge.start()).unwrap().point())
                && on(topo.vertex(edge.end()).unwrap().point())
        })
        .collect();
    let [edge] = found.as_slice() else {
        panic!("{} edges along {a:?} -- {b:?}", found.len());
    };
    *edge
}

/// The seam, the block's top edges that run into it and the block's front
/// corner edge that meets one of them, rounded as one selection.
///
/// No engine builds the vertices where these stripes cross, so the selection
/// used to refuse whole (`unsupported-vertex-blend`). It builds as the
/// features would be made by hand, and lands on exactly that body: the seam
/// as one band, then the top edges rounding into it (held to closed forms in
/// `fillet_into_seam_band.rs`), then the corner edge rounding into the front
/// top edge's band. The convex rounds also take at least their straight
/// runs' closed-form sections.
#[test]
fn fillet_seam_top_edges_and_corner_edge_as_one_selection() {
    let p = Point3::new;
    let front_top = (p(BOSS, 0.0, BLOCK.2), p(BLOCK.0, 0.0, BLOCK.2));
    let side_top = (p(0.0, BOSS, BLOCK.2), p(0.0, BLOCK.1, BLOCK.2));
    let corner = (p(BLOCK.0, 0.0, 0.0), p(BLOCK.0, 0.0, BLOCK.2));
    for (label, make) in [
        ("native", native_body as fn(&mut Topology) -> SolidId),
        ("imported", imported_body),
    ] {
        for r in [2.0, 4.5] {
            let label = format!("{label} r{r}");
            let fillet = |topo: &mut Topology, body: SolidId, edges: &[EdgeId], step: &str| {
                fillet_cascade(topo, body, edges, r)
                    .unwrap_or_else(|e| panic!("{label} {step}: {}: {e}", blend_failure_code(&e)))
                    .solid
            };

            // By hand: the seam, then the top edges, then the corner edge.
            let mut topo = Topology::new();
            let body = make(&mut topo);
            let seam_edges = seam(&topo, body);
            let seamed = fillet(&mut topo, body, &seam_edges, "seam");
            let seamed_volume = solid_volume(&topo, seamed, MEASURE_DEFLECTION).unwrap();
            let tops = [
                edge_along(&topo, seamed, front_top.0, front_top.1),
                edge_along(&topo, seamed, side_top.0, side_top.1),
            ];
            let topped = fillet(&mut topo, seamed, &tops, "top edges");
            let last = edge_along(&topo, topped, corner.0, corner.1);
            let by_hand = fillet(&mut topo, topped, &[last], "corner edge");
            let by_hand_volume = solid_volume(&topo, by_hand, MEASURE_DEFLECTION).unwrap();

            // As one selection.
            let mut topo = Topology::new();
            let body = make(&mut topo);
            let mut edges = seam(&topo, body).to_vec();
            for (a, b) in [front_top, side_top, corner] {
                edges.push(edge_between(&topo, body, a, b));
            }
            let result = fillet_cascade(&mut topo, body, &edges, r)
                .unwrap_or_else(|e| panic!("{label}: {}: {e}", blend_failure_code(&e)));
            assert!(result.failed.is_empty(), "{label}: partial blend");
            assert_valid(&topo, result.solid, &label);
            for sharp in [front_top.0, side_top.0, corner.1] {
                assert!(
                    !has_vertex(&topo, result.solid, sharp),
                    "{label}: sharp corner left at {sharp:?}"
                );
            }
            let volume = solid_volume(&topo, result.solid, MEASURE_DEFLECTION).unwrap();
            assert!(
                (volume - by_hand_volume).abs() < 1e-6 * by_hand_volume,
                "{label}: {volume} as one selection, {by_hand_volume} by hand"
            );
            let straight: f64 = [BLOCK.0 - BOSS - r, BLOCK.1 - BOSS - r, BLOCK.2 - r]
                .iter()
                .map(|length| convex_round_section(r) * length)
                .sum();
            assert!(
                seamed_volume - volume > straight,
                "{label}: the rounds took {}, their straight runs alone {straight}",
                seamed_volume - volume
            );
        }
    }
}

/// A selection the steps cannot split keeps its typed refusal and leaves the
/// input untouched: the block's front top edge with the corner edge it meets,
/// one convex run whose top edge, alone, cannot end against the boss.
#[test]
fn convex_corner_into_the_boss_refuses_atomically() {
    let p = Point3::new;
    let mut topo = Topology::new();
    let body = imported_body(&mut topo);
    let edges = [
        edge_between(&topo, body, p(BOSS, 0.0, BLOCK.2), p(BLOCK.0, 0.0, BLOCK.2)),
        edge_between(&topo, body, p(BLOCK.0, 0.0, 0.0), p(BLOCK.0, 0.0, BLOCK.2)),
    ];
    let counts = solid_entity_counts(&topo, body).unwrap();
    let volume = solid_volume(&topo, body, MEASURE_DEFLECTION).unwrap();
    let Err(error) = fillet_cascade(&mut topo, body, &edges, 2.0) else {
        panic!("the top edge cannot yet end against the boss");
    };
    assert_eq!(
        blend_failure_code(&error),
        "unsupported-vertex-blend",
        "{error}"
    );
    assert_eq!(solid_entity_counts(&topo, body).unwrap(), counts);
    assert!((solid_volume(&topo, body, MEASURE_DEFLECTION).unwrap() - volume).abs() < 1e-9);
}

/// A display-only build skips the measuring postconditions but builds the
/// same body: the six-edge selection lands on the checked result's volume,
/// and the switch is off again afterwards.
#[test]
fn display_only_build_matches_the_checked_build() {
    let p = Point3::new;
    let selection = |topo: &Topology, body: SolidId| {
        let mut edges = seam(topo, body).to_vec();
        for (a, b) in [
            (p(BOSS, 0.0, BLOCK.2), p(BLOCK.0, 0.0, BLOCK.2)),
            (p(0.0, BOSS, BLOCK.2), p(0.0, BLOCK.1, BLOCK.2)),
            (p(BLOCK.0, 0.0, 0.0), p(BLOCK.0, 0.0, BLOCK.2)),
        ] {
            edges.push(edge_between(topo, body, a, b));
        }
        edges
    };
    let r = 2.0;

    let mut topo = Topology::new();
    let body = imported_body(&mut topo);
    let edges = selection(&topo, body);
    let checked = fillet_cascade(&mut topo, body, &edges, r).unwrap().solid;
    let checked_volume = solid_volume(&topo, checked, MEASURE_DEFLECTION).unwrap();

    let mut topo = Topology::new();
    let body = imported_body(&mut topo);
    let edges = selection(&topo, body);
    assert!(!display_only_blends());
    set_display_only_blends(true);
    let shown = fillet_cascade(&mut topo, body, &edges, r);
    set_display_only_blends(false);
    assert!(!display_only_blends());
    let shown = shown.unwrap();
    assert_valid(&topo, shown.solid, "display only");
    let volume = solid_volume(&topo, shown.solid, MEASURE_DEFLECTION).unwrap();
    assert!(
        (volume - checked_volume).abs() < 1e-6 * checked_volume,
        "{volume} for display, {checked_volume} checked"
    );
}
