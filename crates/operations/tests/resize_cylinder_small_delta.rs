//! Regression: `resize_cylindrical_face` refused small radius changes.
//!
//! Two inscribed-polygon tests in the boolean builder decided thin annuli by
//! the chords of their circles. The resize sleeve between the old and new
//! radius leaves exactly such an annulus on each cap, and where it lay inside
//! the chord sagitta (up to 7.6% of the radius for a lone circle edge sampled
//! as an octagon) the cut or fuse dropped faces and the exact-only policy
//! refused it:
//!
//! - same-domain detection tested the annulus's interior point against the
//!   chord polygon of the coplanar face's hole, missed the hole, and paired
//!   the annulus with the face around it, so a growing bore lost both caps;
//! - the interior-point search for a split cap stepped from its outer
//!   vertices in strides wider than the ring and fell back to a chord
//!   midpoint inside the hole, so a shrinking bore's tube lost its caps.
//!
//! A bore whose rim carries a fillet refused separately: the sleeve's cap met
//! the blend at its tangent boundary and the cut lost the ledge face. Such a
//! bore now carries its rim blends to the new radius instead.

#![allow(
    clippy::disallowed_methods,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic
)]

use std::f64::consts::{FRAC_PI_4, PI, TAU};

use remus_math::mat::Mat4;
use remus_operations::blend_ops::fillet_v2;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::heal::unify_faces;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::push_pull::resize_cylindrical_face;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const HEIGHT: f64 = 10.0;
const FILLET: f64 = 1.0;

/// Box 40 × 40 × 10 drilled through at its centre by a flush r = 6 cylinder.
fn bored_block(topo: &mut Topology) -> SolidId {
    let block = make_box(topo, 40.0, 40.0, HEIGHT).unwrap();
    let drill = make_cylinder(topo, 6.0, HEIGHT).unwrap();
    transform_solid(topo, drill, &Mat4::translation(20.0, 20.0, 0.0)).unwrap();
    boolean(topo, BooleanOp::Cut, block, drill).unwrap()
}

/// A cylinder of radius `bore + 8` bored through at radius `bore`, with a
/// `FILLET` round on the top rim of the bore when `rims` is 1 and on both
/// rims when it is 2.
fn bored_cylinder(topo: &mut Topology, bore: f64, rims: usize) -> SolidId {
    let body = make_cylinder(topo, bore + 8.0, HEIGHT).unwrap();
    let drill = make_cylinder(topo, bore, HEIGHT + 4.0).unwrap();
    transform_solid(topo, drill, &Mat4::translation(0.0, 0.0, -2.0)).unwrap();
    let bored = boolean(topo, BooleanOp::Cut, body, drill).unwrap();
    unify_faces(topo, bored).unwrap();
    if rims == 0 {
        return bored;
    }
    let edges: Vec<_> = solid_edges(topo, bored)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Circle(circle)
                if (circle.radius() - bore).abs() < 1e-9
                    && ((circle.center().z() - HEIGHT).abs() < 1e-9
                        || (rims == 2 && circle.center().z().abs() < 1e-9)))
        })
        .collect();
    assert_eq!(edges.len(), rims);
    let blended = fillet_v2(topo, bored, &edges, FILLET).unwrap();
    assert!(!blended.is_partial);
    blended.solid
}

fn bores(topo: &Topology, solid: SolidId) -> Vec<(FaceId, f64)> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|face| {
            let data = topo.face(face).unwrap();
            match data.surface() {
                FaceSurface::Cylinder(cylinder) if data.is_reversed() => {
                    Some((face, cylinder.radius()))
                }
                _ => None,
            }
        })
        .collect()
}

fn only_bore(topo: &Topology, solid: SolidId) -> FaceId {
    let found = bores(topo, solid);
    assert_eq!(found.len(), 1, "expected one bore, found {found:?}");
    found[0].0
}

fn rounds(topo: &Topology, solid: SolidId) -> Vec<(f64, f64)> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|face| match topo.face(face).unwrap().surface() {
            FaceSurface::Torus(torus) => Some((torus.major_radius(), torus.minor_radius())),
            _ => None,
        })
        .collect()
}

fn assert_closed(topo: &Topology, solid: SolidId) {
    let shell = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    validate_shell_closed(shell, topo).expect("result shell must be closed");
}

fn assert_volume(topo: &Topology, solid: SolidId, expected: f64, label: &str) {
    let actual = solid_volume(topo, solid, 0.01).unwrap();
    assert!(
        (actual - expected).abs() <= expected * 1e-6,
        "{label}: volume {actual} differs from {expected}"
    );
}

/// The material a `FILLET` round removes from a convex bore rim at radius
/// `bore`: the corner spandrel, of area ρ²(1 − π/4) with its centroid
/// ρ(10 − 3π)/(12 − 3π) from the corner, swept round the axis (Pappus).
fn rim_round_volume(bore: f64) -> f64 {
    let centroid = FILLET * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
    TAU * (bore + centroid) * FILLET * FILLET * (1.0 - FRAC_PI_4)
}

#[test]
fn bored_block_resizes_to_nearby_radii_as_one_cylinder() {
    for radius in [5.8, 6.25] {
        let mut topo = Topology::new();
        let block = bored_block(&mut topo);
        let bore = only_bore(&topo, block);
        let resized = resize_cylindrical_face(&mut topo, block, bore, radius)
            .unwrap_or_else(|error| panic!("resize the r6 bore to {radius}: {error}"));
        assert_closed(&topo, resized);
        let walls: Vec<_> = solid_faces(&topo, resized)
            .unwrap()
            .into_iter()
            .filter_map(|face| match topo.face(face).unwrap().surface() {
                FaceSurface::Cylinder(cylinder) => Some(cylinder.clone()),
                _ => None,
            })
            .collect();
        assert_eq!(walls.len(), 1, "one analytic wall at r{radius}");
        let wall = &walls[0];
        assert!((wall.radius() - radius).abs() < 1e-9);
        assert!(wall.axis().z().abs() > 1.0 - 1e-12);
        assert!((wall.origin().x() - 20.0).abs() < 1e-9 && (wall.origin().y() - 20.0).abs() < 1e-9);
        assert_volume(
            &topo,
            resized,
            40.0 * 40.0 * HEIGHT - PI * radius * radius * HEIGHT,
            &format!("block bored to r{radius}"),
        );
    }
}

/// Both directions used to refuse once |Δr| fell inside the chord sagitta of
/// the cap circles: r 12.25 refused ±0.05 and r 24.5 refused ±0.05 and ±0.1.
#[test]
fn bored_cylinder_resizes_by_less_than_the_chord_sagitta() {
    for (bore, steps) in [
        (4.0, [0.05, 0.25]),
        (12.25, [0.05, 0.1]),
        (24.5, [0.05, 0.1]),
    ] {
        for step in steps {
            for radius in [bore - step, bore + step] {
                let mut topo = Topology::new();
                let body = bored_cylinder(&mut topo, bore, 0);
                let wall = only_bore(&topo, body);
                let resized = resize_cylindrical_face(&mut topo, body, wall, radius)
                    .unwrap_or_else(|error| panic!("resize bore r{bore} to {radius}: {error}"));
                assert_closed(&topo, resized);
                let found = bores(&topo, resized);
                assert_eq!(found.len(), 1);
                assert!((found[0].1 - radius).abs() < 1e-9, "bore {found:?}");
                let outer = bore + 8.0;
                assert_volume(
                    &topo,
                    resized,
                    PI * (outer * outer - radius * radius) * HEIGHT,
                    &format!("bore r{bore} resized to {radius}"),
                );
            }
        }
    }
}

/// Every step of 0.05 out to ±1.0 around r 6, with a 1 mm round on the bore's
/// top rim. The round follows the wall: the torus keeps its 1 mm section and
/// its major radius moves with the bore.
#[test]
fn rim_filleted_bore_carries_its_blend_across_small_resizes() {
    let bore = 6.0;
    let outer = bore + 8.0;
    for step in (-20..=20).filter(|step| *step != 0) {
        let radius = bore + f64::from(step) * 0.05;
        let mut topo = Topology::new();
        let body = bored_cylinder(&mut topo, bore, 1);
        let source_counts = solid_entity_counts(&topo, body).unwrap();
        assert_volume(
            &topo,
            body,
            PI * (outer * outer - bore * bore) * HEIGHT - rim_round_volume(bore),
            "rim-rounded source",
        );
        let wall = only_bore(&topo, body);
        let resized = resize_cylindrical_face(&mut topo, body, wall, radius)
            .unwrap_or_else(|error| panic!("resize the rounded r6 bore to {radius}: {error}"));
        assert_closed(&topo, resized);
        assert_eq!(solid_entity_counts(&topo, resized).unwrap(), source_counts);
        let found = bores(&topo, resized);
        assert_eq!(found.len(), 1);
        assert!(
            (found[0].1 - radius).abs() < 1e-9,
            "bore {found:?} at {radius}"
        );
        let found = rounds(&topo, resized);
        assert_eq!(found.len(), 1, "one rim round at {radius}: {found:?}");
        assert!(
            (found[0].0 - (radius + FILLET)).abs() < 1e-9 && (found[0].1 - FILLET).abs() < 1e-9,
            "rim round {found:?} at {radius}"
        );
        assert_volume(
            &topo,
            resized,
            PI * (outer * outer - radius * radius) * HEIGHT - rim_round_volume(radius),
            &format!("rounded bore resized to {radius}"),
        );
    }
}

/// Both rims rounded, on bores whose sleeve used to refuse at some sizes
/// (r 4 every growth, r 12.25 at ±0.05).
#[test]
fn bore_rounded_at_both_rims_carries_both_blends() {
    for (bore, deltas) in [(4.0, [-0.35, 0.05, 0.6]), (12.25, [-1.0, -0.05, 0.05])] {
        for delta in deltas {
            let radius = bore + delta;
            let mut topo = Topology::new();
            let body = bored_cylinder(&mut topo, bore, 2);
            let source_counts = solid_entity_counts(&topo, body).unwrap();
            let wall = only_bore(&topo, body);
            let resized = resize_cylindrical_face(&mut topo, body, wall, radius)
                .unwrap_or_else(|error| panic!("resize bore r{bore} to {radius}: {error}"));
            assert_closed(&topo, resized);
            assert_eq!(solid_entity_counts(&topo, resized).unwrap(), source_counts);
            let found = rounds(&topo, resized);
            assert_eq!(found.len(), 2);
            assert!(found.iter().all(|&(major, minor)| {
                (major - (radius + FILLET)).abs() < 1e-9 && (minor - FILLET).abs() < 1e-9
            }));
            let outer = bore + 8.0;
            assert_volume(
                &topo,
                resized,
                PI * (outer * outer - radius * radius) * HEIGHT - 2.0 * rim_round_volume(radius),
                &format!("bore r{bore} rounded at both rims resized to {radius}"),
            );
        }
    }
}
