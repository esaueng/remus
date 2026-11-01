//! Rounding and bevelling a block's top edges into an earlier seam band.
//!
//! Once the seam where a round boss meets a block has been filleted, the
//! block's sharp top edge no longer reaches the boss: it runs into the seam's
//! torus band at `K = (6 + r, 0, 18)` and carries on tangentially as the
//! band's end section up the front wall, then along the mitre between the
//! band and the wall's seam cylinder, until those two meet the boss
//! tangentially. A fillet of the top edge follows that whole ridgeline, as
//! any CAD tool does: the ball leaves the block's top for the torus before it
//! reaches `K`, leaves the front wall for the seam cylinder where its contact
//! reaches that cylinder's tangent line, and the band fades out where the
//! two meet the boss. The walking builder refused the edge
//! (`unsupported-seam-crossing`), and every chamfer of it (`re-trimming a
//! curved neighbour against the bevel is not implemented`).
//!
//! These tests hold the band to the closed forms of the faces it rolls on:
//! where each handover happens, that every point of the band lies on a ball
//! of the fillet's radius touching one face of each side, that the straight
//! run is the exact quarter cylinder, and that the material removed is at
//! least the straight run's closed-form corner and at most that plus the
//! same corner along the rest of the chain.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::PI;

use remus_math::vec::Point3;
use remus_operations::blend_ops::{blend_failure_code, chamfer_v2, fillet_cascade};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const BOSS: f64 = 6.0;
const BLOCK: (f64, f64, f64) = (30.0, 18.0, 18.0);
/// The seam fillet's radius, as in the recording.
const SEAM: f64 = 2.6;
const MEASURE_DEFLECTION: f64 = 0.005;

/// The boss on the block's corner, rising 10 past its top.
fn native_body(topo: &mut Topology) -> SolidId {
    let boss = make_cylinder(topo, BOSS, 28.0).unwrap();
    let block = make_box(topo, BLOCK.0, BLOCK.1, BLOCK.2).unwrap();
    boolean(topo, BooleanOp::Fuse, boss, block).unwrap()
}

/// The same body as a user's STEP export (boss 28.5 tall, r1.8 top round).
fn imported_body(topo: &mut Topology) -> SolidId {
    remus_io::step::reader::read_step(
        include_str!("data/box_corner_rounded_cylinder_boss.step"),
        topo,
    )
    .unwrap()[0]
}

fn find_edge(topo: &Topology, solid: SolidId, a: Point3, b: Point3) -> EdgeId {
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

fn has_vertex(topo: &Topology, solid: SolidId, at: Point3, tolerance: f64) -> bool {
    solid_vertices(topo, solid)
        .unwrap()
        .into_iter()
        .any(|v| (topo.vertex(v).unwrap().point() - at).length() < tolerance)
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

/// The body with its boss seam filleted at [`SEAM`].
fn seam_filleted(topo: &mut Topology, make: fn(&mut Topology) -> SolidId) -> SolidId {
    let body = make(topo);
    let p = Point3::new;
    let seam = [
        find_edge(topo, body, p(BOSS, 0.0, 0.0), p(BOSS, 0.0, BLOCK.2)),
        find_edge(topo, body, p(BOSS, 0.0, BLOCK.2), p(0.0, BOSS, BLOCK.2)),
        find_edge(topo, body, p(0.0, BOSS, BLOCK.2), p(0.0, BOSS, 0.0)),
    ];
    fillet_cascade(topo, body, &seam, SEAM).unwrap().solid
}

/// The block's top front edge, from where the torus leaves it, and its top
/// side edge.
fn top_edges(topo: &Topology, solid: SolidId) -> (EdgeId, EdgeId) {
    let p = Point3::new;
    (
        find_edge(
            topo,
            solid,
            p(BOSS + SEAM, 0.0, BLOCK.2),
            p(BLOCK.0, 0.0, BLOCK.2),
        ),
        find_edge(
            topo,
            solid,
            p(0.0, BOSS + SEAM, BLOCK.2),
            p(0.0, BLOCK.1, BLOCK.2),
        ),
    )
}

/// Major radius of the seam torus.
fn major() -> f64 {
    BOSS + SEAM
}

/// Where the front wall's seam cylinder touches the wall.
fn wall_tangent() -> f64 {
    (major().powi(2) - SEAM * SEAM).sqrt()
}

/// Distance from the seam torus, positive on the material side.
fn torus_distance(c: Point3) -> f64 {
    let radial = c.x().hypot(c.y()) - major();
    radial.hypot(c.z() - (BLOCK.2 + SEAM)) - SEAM
}

/// Distance from the front wall's seam cylinder, positive on the material
/// side.
fn wall_cylinder_distance(c: Point3) -> f64 {
    (c.x() - wall_tangent()).hypot(c.y() + SEAM) - SEAM
}

/// The two handovers of a ball of radius `rho` rolling along the front top
/// edge: where it leaves the block's top for the torus (it touches both at
/// the top's tangent circle, `rho` in from the wall), and where its contact
/// with the wall reaches the seam cylinder's tangent line (it touches the
/// torus there too).
fn handovers(rho: f64) -> (Point3, Point3) {
    let onto_torus = Point3::new((major().powi(2) - rho * rho).sqrt(), rho, BLOCK.2);
    let x = wall_tangent();
    let radial = x.hypot(rho) - major();
    let z = BLOCK.2 + SEAM - ((SEAM + rho).powi(2) - radial * radial).sqrt();
    (onto_torus, Point3::new(x, 0.0, z))
}

/// The end of the chain, where the seam cylinder, the torus and the boss
/// meet.
fn chain_end() -> Point3 {
    let x = wall_tangent();
    let scale = BOSS / x.hypot(SEAM);
    Point3::new(x * scale, -SEAM * scale, BLOCK.2 + SEAM)
}

/// An upper bound on the length of the chain past the point where the ball
/// leaves the block's top: the rest of the straight edge to `K`, the torus's
/// end section up to the mitre foot (an arc of radius [`SEAM`]), and twice
/// the mitre's chord from its foot to the chain's end.
fn climbing_length_bound(rho: f64) -> f64 {
    let (onto_torus, _) = handovers(rho);
    let k = Point3::new(major(), 0.0, BLOCK.2);
    let foot = Point3::new(
        wall_tangent(),
        0.0,
        BLOCK.2 + SEAM - (SEAM * SEAM - (major() - wall_tangent()).powi(2)).sqrt(),
    );
    let arc = 2.0 * SEAM * ((foot - k).length() / (2.0 * SEAM)).asin();
    (major() - onto_torus.x()) + arc + 2.0 * (chain_end() - foot).length()
}

/// The material a right-angled edge loses per unit length: the corner
/// outside a quarter circle for a fillet, the triangle for a chamfer.
fn corner_area(size: f64, chamfer: bool) -> f64 {
    if chamfer {
        0.5 * size * size
    } else {
        (1.0 - PI / 4.0) * size * size
    }
}

/// Every interior point of the front band against the faces it rolls on,
/// returning the largest stray of the climbing part.
///
/// A fillet band point `p` with outward normal `n` has its circle's centre
/// at `p − rho n`. Along the straight run, well clear of `K`, that is the
/// closed-form quarter cylinder's axis `(x, rho, 18 − rho)`. Past it the
/// centre must lie `rho` from one face of each side: the front wall where its
/// foot on the wall is beyond the seam cylinder's tangent line, the seam
/// cylinder before it; the block's top where its foot on the top is outside
/// the torus's tangent circle, the torus inside it. The walked circles lie in
/// the spine's normal planes, tangent to each face's trace there, so once the
/// spine climbs the torus's end section they sit slightly off a 3D ball
/// touching both faces, though still tangent to both along the contacts; the
/// caller bounds that. Only the walked sections (the band's knots along v)
/// are read: between them the band is interpolated.
fn rolling_ball_stray(topo: &Topology, band: FaceId, rho: f64) -> f64 {
    let face = topo.face(band).unwrap();
    let FaceSurface::Nurbs(surface) = face.surface() else {
        panic!("band is not a walked surface");
    };
    let mut stations: Vec<f64> = surface.knots_v().to_vec();
    stations.dedup();
    let mut climbing: f64 = 0.0;
    for i in 1..20 {
        for &v in &stations {
            let u = f64::from(i) / 20.0;
            // The band closes to a point where it runs out; its normal is
            // undefined there.
            if (surface.evaluate(0.0, v) - surface.evaluate(1.0, v)).length() < 1e-2 * rho {
                continue;
            }
            let p = surface.evaluate(u, v);
            let raw = surface.normal(u, v).unwrap();
            let n = if face.is_reversed() { -raw } else { raw };
            let c = p - n * rho;
            if c.x() > major() + 2.0 {
                let off = (c.y() - rho).hypot(c.z() - (BLOCK.2 - rho));
                assert!(
                    off < 1e-6 * rho,
                    "straight run's axis off by {off} at {c:?}"
                );
                continue;
            }
            let wall = if c.x() >= wall_tangent() {
                c.y()
            } else {
                wall_cylinder_distance(c)
            };
            let top = if c.x().hypot(c.y()) >= major() {
                BLOCK.2 - c.z()
            } else {
                torus_distance(c)
            };
            let here = (wall - rho).abs().max((top - rho).abs());
            climbing = climbing.max(here);
        }
    }
    climbing
}

/// Swap x and y: the side edge's geometry from the front edge's.
fn mirror(p: Point3) -> Point3 {
    Point3::new(p.y(), p.x(), p.z())
}

fn run(chamfer: bool) {
    for (body, make) in [
        ("native", native_body as fn(&mut Topology) -> SolidId),
        ("imported", imported_body),
    ] {
        let mut base = Topology::new();
        let filleted = seam_filleted(&mut base, make);
        let (front, side) = top_edges(&base, filleted);
        for size in [1.0, 2.0] {
            for (which, selection) in [
                ("front", vec![front]),
                ("side", vec![side]),
                ("both", vec![front, side]),
            ] {
                let label = format!(
                    "{body} {which} {} {size}",
                    if chamfer { "chamfer" } else { "fillet" }
                );
                let mut topo = base.clone();
                let faces_before = solid_faces(&topo, filleted).unwrap();
                let before = solid_volume(&topo, filleted, MEASURE_DEFLECTION).unwrap();
                let result = if chamfer {
                    chamfer_v2(&mut topo, filleted, &selection, size, size)
                } else {
                    fillet_cascade(&mut topo, filleted, &selection, size)
                }
                .unwrap_or_else(|e| panic!("{label}: {}: {e}", blend_failure_code(&e)));
                assert!(result.failed.is_empty(), "{label}: partial blend");
                assert_valid(&topo, result.solid, &label);

                let bands: Vec<FaceId> = solid_faces(&topo, result.solid)
                    .unwrap()
                    .into_iter()
                    .filter(|f| !faces_before.contains(f))
                    .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_)))
                    .collect();
                assert_eq!(bands.len(), selection.len(), "{label}: one band per edge");

                // Each band starts square at the block's far face and runs out
                // at the chain end. It hands over to the torus on the top's
                // tangent circle and to the seam cylinder on its tangent line
                // with the wall, near where a 3D ball would (see
                // `rolling_ball_stray` for why not exactly there).
                let (onto_torus, onto_cylinder) = handovers(size);
                let mut ends = Vec::new();
                let mut handover = Vec::new();
                if which != "side" {
                    ends.extend([
                        Point3::new(BLOCK.0, 0.0, BLOCK.2 - size),
                        Point3::new(BLOCK.0, size, BLOCK.2),
                        chain_end(),
                    ]);
                    handover.extend([(onto_torus, false), (onto_cylinder, false)]);
                }
                if which != "front" {
                    ends.extend([
                        Point3::new(0.0, BLOCK.1, BLOCK.2 - size),
                        Point3::new(size, BLOCK.1, BLOCK.2),
                        mirror(chain_end()),
                    ]);
                    handover.extend([(mirror(onto_torus), true), (mirror(onto_cylinder), true)]);
                }
                for at in ends {
                    assert!(
                        has_vertex(&topo, result.solid, at, 1e-6),
                        "{label}: no vertex at {at:?}"
                    );
                }
                for (index, (near, mirrored)) in handover.into_iter().enumerate() {
                    let on_seam = |q: Point3| {
                        let q = if mirrored { mirror(q) } else { q };
                        if index % 2 == 0 {
                            (q.x().hypot(q.y()) - major()).abs() < 1e-6
                                && (q.z() - BLOCK.2).abs() < 1e-9
                        } else {
                            (q.x() - wall_tangent()).abs() < 1e-6 && q.y().abs() < 1e-9
                        }
                    };
                    assert!(
                        solid_vertices(&topo, result.solid)
                            .unwrap()
                            .into_iter()
                            .any(|v| {
                                let q = topo.vertex(v).unwrap().point();
                                (q - near).length() < 1e-2 * size && on_seam(q)
                            }),
                        "{label}: no handover on its seam near {near:?}"
                    );
                }

                // Up the mitre the spine runs well off the ball's path, so its
                // normal planes cut the faces obliquely: measured 0.6% of the
                // radius at 1, 2.1% at 2.
                if !chamfer && which == "front" {
                    let stray = rolling_ball_stray(&topo, bands[0], size);
                    assert!(
                        stray < 3e-2 * size,
                        "{label}: band strays {stray} off a rolling ball"
                    );
                }

                // The straight run, from the block's far face to where the
                // ball leaves its top, loses exactly its corner; the rest of
                // the chain loses no more than that corner along it.
                let area = corner_area(size, chamfer);
                let straight = match which {
                    "front" => area * (BLOCK.0 - onto_torus.x()),
                    "side" => area * (BLOCK.1 - onto_torus.x()),
                    _ => area * (BLOCK.0 + BLOCK.1 - 2.0 * onto_torus.x()),
                };
                let edges = selection.len() as f64;
                let climbing = edges * area * climbing_length_bound(size);
                let removed =
                    before - solid_volume(&topo, result.solid, MEASURE_DEFLECTION).unwrap();
                let slack = 1e-6 * before;
                assert!(
                    removed > straight - slack && removed < straight + climbing + slack,
                    "{label}: removed {removed}, straight run {straight}, climbing at most {climbing}"
                );
            }
        }
    }
}

#[test]
fn fillet_top_edges_into_the_seam_band() {
    run(false);
}

#[test]
fn chamfer_top_edges_into_the_seam_band() {
    run(true);
}
