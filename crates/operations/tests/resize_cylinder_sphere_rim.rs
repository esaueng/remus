//! Coaxial radius change of a cylindrical wall whose rim meets a sphere along
//! an off-axis intersection curve: a boss rising from a sphere and a blind
//! bore sunk into one (the consumer's r 73.5 sphere with an r 6 cylinder).
//!
//! Oracles are independent of the construction: the closed-form sphere
//! volume plus the sleeve integral `∫∫ (t_top − t_bottom) dA` evaluated by
//! composite Simpson in ρ and the trapezoid rule in θ (the kernel uses
//! Gauss–Legendre), the tessellated volume, carrier residuals of every
//! rebuilt rim sample, and an entity census against the source.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_precision_loss,
    clippy::similar_names
)]

use std::f64::consts::{PI, TAU};

use remus_math::mat::Mat4;
use remus_offset::{OffsetError, UnsupportedMoveCause};
use remus_operations::OperationsError;
use remus_operations::blend_ops::fillet_v2;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::journal_ops::resize_cylindrical_face_journaled;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_cylinder, make_sphere};
use remus_operations::push_pull::resize_cylindrical_face;
use remus_operations::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{edge_to_face_map, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const SPHERE_RADIUS: f64 = 73.5;
const SPHERE_CENTER: (f64, f64) = (25.5, -1.5);
const AXIS: (f64, f64) = (35.5, 3.5);
const BASE: f64 = 58.0;
const HEIGHT: f64 = 64.1;
const RADIUS: f64 = 6.0;

/// The consumer body at `scale`: `Fuse` leaves a boss, `Cut` a blind bore.
fn body(topo: &mut Topology, op: BooleanOp, scale: f64) -> (SolidId, FaceId) {
    let sphere = make_sphere(topo, SPHERE_RADIUS * scale, 16).expect("sphere");
    transform_solid(
        topo,
        sphere,
        &Mat4::translation(SPHERE_CENTER.0 * scale, SPHERE_CENTER.1 * scale, 0.0),
    )
    .expect("place sphere");
    let cylinder = make_cylinder(topo, RADIUS * scale, HEIGHT * scale).expect("cylinder");
    transform_solid(
        topo,
        cylinder,
        &Mat4::translation(AXIS.0 * scale, AXIS.1 * scale, BASE * scale),
    )
    .expect("place cylinder");
    let solid = boolean(topo, op, sphere, cylinder).expect("sphere/cylinder boolean");
    (solid, only_wall(topo, solid))
}

fn only_wall(topo: &Topology, solid: SolidId) -> FaceId {
    let walls: Vec<_> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .collect();
    assert_eq!(walls.len(), 1, "one cylindrical wall");
    walls[0]
}

/// Height of the upper sphere branch above `(x, y)` in unit-scale coordinates.
fn sphere_top(x: f64, y: f64) -> f64 {
    let dx = x - SPHERE_CENTER.0;
    let dy = y - SPHERE_CENTER.1;
    (SPHERE_RADIUS * SPHERE_RADIUS - dx * dx - dy * dy).sqrt()
}

/// `∫∫_{ρ ≤ r} f dA` about the cylinder axis: composite Simpson in ρ (400
/// panels) and the periodic trapezoid rule in θ (720 steps).
fn disk_integral(radius: f64, f: impl Fn(f64, f64) -> f64) -> f64 {
    let panels = 400;
    let steps = 720;
    let h = radius / f64::from(panels);
    let mut total = 0.0;
    for k in 0..steps {
        let theta = TAU * f64::from(k) / f64::from(steps);
        let (sin, cos) = theta.sin_cos();
        let mut radial = 0.0;
        for i in 0..=panels {
            let rho = h * f64::from(i);
            let weight = if i == 0 || i == panels {
                1.0
            } else if i % 2 == 1 {
                4.0
            } else {
                2.0
            };
            radial += weight * f(AXIS.0 + rho * cos, AXIS.1 + rho * sin) * rho;
        }
        total += radial * h / 3.0;
    }
    total * TAU / f64::from(steps)
}

/// Closed-form body volume with wall radius `radius`, at `scale`.
fn closed_form_volume(op: BooleanOp, radius: f64, scale: f64) -> f64 {
    let sphere = 4.0 / 3.0 * PI * SPHERE_RADIUS.powi(3);
    let unit = match op {
        BooleanOp::Fuse => sphere + disk_integral(radius, |x, y| BASE + HEIGHT - sphere_top(x, y)),
        BooleanOp::Cut => sphere - disk_integral(radius, |x, y| sphere_top(x, y) - BASE),
        BooleanOp::Intersect => unreachable!(),
    };
    unit * scale.powi(3)
}

/// Every rebuilt boundary sample of the wall lies on the new cylinder, and
/// every sample shared with a sphere lies on that sphere.
fn assert_rims_on_carriers(topo: &Topology, solid: SolidId, wall: FaceId, radius: f64, scale: f64) {
    let edge_faces = edge_to_face_map(topo, solid).unwrap();
    let residual = 1e-6 * scale.max(1.0);
    for (&index, faces) in &edge_faces {
        if !faces.contains(&wall) {
            continue;
        }
        let edge = topo.edge(topo.edge_id_from_index(index).unwrap()).unwrap();
        let start = topo.vertex(edge.start()).unwrap().point();
        let end = topo.vertex(edge.end()).unwrap().point();
        let (lo, hi) = edge.strict_domain().expect("rim edges carry a trim");
        let sphere = faces
            .iter()
            .find_map(|&face| match topo.face(face).unwrap().surface() {
                FaceSurface::Sphere(sphere) => Some(sphere.clone()),
                _ => None,
            });
        for i in 0..=64 {
            let p = edge.curve().evaluate_with_endpoints(
                lo + (hi - lo) * f64::from(i) / 64.0,
                start,
                end,
            );
            let rho = (p.x() - AXIS.0 * scale).hypot(p.y() - AXIS.1 * scale);
            assert!(
                (rho - radius * scale).abs() <= residual,
                "edge {index} sample {i} is {rho} from the axis, expected {}",
                radius * scale
            );
            if let Some(sphere) = &sphere {
                let off = (p - sphere.center()).length() - sphere.radius();
                assert!(
                    off.abs() <= residual,
                    "edge {index} sample {i} off sphere by {off}"
                );
                assert!(
                    matches!(edge.curve(), EdgeCurve::NurbsCurve(_)),
                    "sphere rim is a re-intersected curve"
                );
            }
        }
    }
}

fn check_resize(op: BooleanOp, scale: f64, new_radius: f64) {
    let label = format!("{op:?} scale {scale} r {new_radius}");
    let mut topo = Topology::new();
    let (source, wall) = body(&mut topo, op, scale);
    let counts = solid_entity_counts(&topo, source).unwrap();
    let deflection = 0.002 * scale;
    let before = solid_volume(&topo, source, deflection).unwrap();
    let source_oracle = closed_form_volume(op, RADIUS, scale);
    assert!(
        (before - source_oracle).abs() <= 5e-5 * source_oracle,
        "{label}: source mesh {before} vs closed form {source_oracle}"
    );

    let result = resize_cylindrical_face(&mut topo, source, wall, new_radius * scale)
        .unwrap_or_else(|error| panic!("{label}: {error}"));
    assert!(
        validate_solid(&topo, result).unwrap().is_valid(),
        "{label}: invalid"
    );
    assert_eq!(
        solid_entity_counts(&topo, result).unwrap(),
        counts,
        "{label}: census"
    );
    assert_eq!(
        solid_entity_counts(&topo, source).unwrap(),
        counts,
        "{label}: source kept"
    );
    let mesh = tessellate_solid_with_tolerance(&topo, result, deflection, 0.1).unwrap();
    assert!(is_watertight(&mesh), "{label}: mesh not watertight");

    let resized = only_wall(&topo, result);
    let FaceSurface::Cylinder(cylinder) = topo.face(resized).unwrap().surface() else {
        unreachable!()
    };
    assert!((cylinder.radius() - new_radius * scale).abs() <= 1e-12 * scale.max(1.0));
    assert_eq!(
        topo.face(resized).unwrap().is_reversed(),
        op == BooleanOp::Cut,
        "{label}: wall orientation"
    );
    assert_rims_on_carriers(&topo, result, resized, new_radius, scale);

    let after = solid_volume(&topo, result, deflection).unwrap();
    let oracle = closed_form_volume(op, new_radius, scale);
    let change = oracle - source_oracle;
    assert!(
        (after - oracle).abs() <= 5e-5 * oracle,
        "{label}: mesh {after} vs closed form {oracle}"
    );
    // The mesh error of the untouched sphere cancels in the difference.
    assert!(
        ((after - before) - change).abs() <= 1e-2 * change.abs(),
        "{label}: volume change {} vs closed form {change}",
        after - before
    );
    // The edit moves material the right way.
    let grows = new_radius > RADIUS;
    assert_eq!(change > 0.0, grows == (op == BooleanOp::Fuse), "{label}");
}

#[test]
fn sphere_boss_wall_resizes_both_ways_across_scales() {
    // The source boolean itself does not build this body at 1e2 or above.
    for scale in [1e-3, 1.0, 10.0] {
        for radius in [5.0, 7.0] {
            check_resize(BooleanOp::Fuse, scale, radius);
        }
    }
}

#[test]
fn sphere_blind_bore_wall_resizes_both_ways_across_scales() {
    // The source boolean itself does not build this body at 1e2 or above.
    for scale in [1e-3, 1.0, 10.0] {
        for radius in [5.0, 7.0] {
            check_resize(BooleanOp::Cut, scale, radius);
        }
    }
}

#[test]
fn sphere_rim_resize_matches_a_direct_rebuild() {
    // The resized body and the body built directly at the new radius are
    // the same shape: same census, same volume.
    for op in [BooleanOp::Fuse, BooleanOp::Cut] {
        for radius in [5.0, 7.0] {
            let mut topo = Topology::new();
            let (source, wall) = body(&mut topo, op, 1.0);
            let resized = resize_cylindrical_face(&mut topo, source, wall, radius).unwrap();
            let sphere = make_sphere(&mut topo, SPHERE_RADIUS, 16).unwrap();
            transform_solid(
                &mut topo,
                sphere,
                &Mat4::translation(SPHERE_CENTER.0, SPHERE_CENTER.1, 0.0),
            )
            .unwrap();
            let cylinder = make_cylinder(&mut topo, radius, HEIGHT).unwrap();
            transform_solid(
                &mut topo,
                cylinder,
                &Mat4::translation(AXIS.0, AXIS.1, BASE),
            )
            .unwrap();
            let direct = boolean(&mut topo, op, sphere, cylinder).unwrap();
            let face_count = |solid| solid_faces(&topo, solid).unwrap().len();
            assert_eq!(face_count(resized), face_count(direct), "{op:?} r {radius}");
            let a = solid_volume(&topo, resized, 0.002).unwrap();
            let b = solid_volume(&topo, direct, 0.002).unwrap();
            assert!((a - b).abs() <= 2e-5 * b, "{op:?} r {radius}: {a} vs {b}");
        }
    }
}

#[test]
fn journaled_sphere_rim_resize_reports_a_total_face_map() {
    for op in [BooleanOp::Fuse, BooleanOp::Cut] {
        for radius in [5.0, 7.0] {
            let mut topo = Topology::new();
            let (source, wall) = body(&mut topo, op, 1.0);
            let source_faces = solid_faces(&topo, source).unwrap();
            let journaled = resize_cylindrical_face_journaled(&mut topo, source, wall, radius)
                .unwrap_or_else(|error| panic!("{op:?} r {radius}: {error}"));
            let map = &journaled.map;
            assert!(map.unresolved.is_empty(), "{op:?}: {map:?}");
            assert!(
                map.deleted.is_empty() && map.generated.is_empty(),
                "{op:?}: {map:?}"
            );
            let result_faces = solid_faces(&topo, journaled.solid).unwrap();
            assert_eq!(map.modified.len(), source_faces.len());
            for face in &source_faces {
                let outputs = &map.modified[&face.index()];
                assert_eq!(
                    outputs.len(),
                    1,
                    "{op:?}: face {} is one-to-one",
                    face.index()
                );
                assert!(result_faces.iter().any(|f| f.index() == outputs[0]));
            }
            let wall_output = map.modified[&wall.index()][0];
            assert_eq!(wall_output, only_wall(&topo, journaled.solid).index());
            let plain_volume = {
                let mut plain = Topology::new();
                let (s, w) = body(&mut plain, op, 1.0);
                let r = resize_cylindrical_face(&mut plain, s, w, radius).unwrap();
                solid_volume(&plain, r, 0.002).unwrap()
            };
            let volume = solid_volume(&topo, journaled.solid, 0.002).unwrap();
            assert!(
                (volume - plain_volume).abs() <= 1e-9 * volume,
                "{op:?}: same geometry"
            );
        }
    }
}

#[test]
fn sphere_rim_resize_refusals_are_typed_and_transactional() {
    // A radius whose rim would reach the sphere's silhouette, and a boss
    // grown into a second boss: both refuse without touching the input.
    let mut topo = Topology::new();
    let (source, wall) = body(&mut topo, BooleanOp::Fuse, 1.0);
    let other = make_cylinder(&mut topo, 4.0, HEIGHT).unwrap();
    transform_solid(
        &mut topo,
        other,
        &Mat4::translation(AXIS.0 + 18.0, AXIS.1, BASE),
    )
    .unwrap();
    let twin = boolean(&mut topo, BooleanOp::Fuse, source, other).unwrap();
    let twin_wall = solid_faces(&topo, twin)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(topo.face(face).unwrap().surface(),
                FaceSurface::Cylinder(c) if (c.radius() - RADIUS).abs() < 1e-9)
        })
        .unwrap();
    for (solid, face, radius) in [(source, wall, 70.0), (twin, twin_wall, 16.0)] {
        let counts = solid_entity_counts(&topo, solid).unwrap();
        let volume = solid_volume(&topo, solid, 0.01).unwrap();
        for error in [
            resize_cylindrical_face(&mut topo, solid, face, radius).expect_err("plain"),
            resize_cylindrical_face_journaled(&mut topo, solid, face, radius)
                .expect_err("journaled"),
        ] {
            assert!(
                matches!(
                    error,
                    OperationsError::Offset(OffsetError::TopologyChange { face: Some(f), .. })
                        if f == face
                ),
                "r {radius}: {error:?}"
            );
            assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
            assert!((solid_volume(&topo, solid, 0.01).unwrap() - volume).abs() < 1e-9);
        }
    }
}

fn shared_edges(topo: &Topology, solid: SolidId, kinds: [&str; 2]) -> Vec<remus_topology::EdgeId> {
    edge_to_face_map(topo, solid)
        .unwrap()
        .iter()
        .filter_map(|(&edge, faces)| {
            let tags: Vec<_> = faces
                .iter()
                .map(|&face| topo.face(face).unwrap().surface().type_tag())
                .collect();
            (faces[0] != faces[1] && kinds.iter().all(|kind| tags.contains(kind)))
                .then(|| topo.edge_id_from_index(edge).unwrap())
        })
        .collect()
}

#[test]
fn sphere_boss_with_a_seam_blend_keeps_its_typed_refusal() {
    // The r 2 seam fillet is a NURBS blend band: no exact re-intersection
    // yet, so the refusal names it as the unsupported neighbour.
    let mut topo = Topology::new();
    let (source, _) = body(&mut topo, BooleanOp::Fuse, 1.0);
    let seam = shared_edges(&topo, source, ["cylinder", "sphere"]);
    let blended = fillet_v2(&mut topo, source, &seam, 2.0).unwrap().solid;
    let wall = only_wall(&topo, blended);
    let counts = solid_entity_counts(&topo, blended).unwrap();
    for radius in [5.0, 7.0] {
        for error in [
            resize_cylindrical_face(&mut topo, blended, wall, radius).expect_err("plain"),
            resize_cylindrical_face_journaled(&mut topo, blended, wall, radius)
                .expect_err("journaled"),
        ] {
            let OperationsError::Offset(OffsetError::UnsupportedMoveFace {
                face,
                surface_type,
                cause: UnsupportedMoveCause::UnsupportedNeighbor { selected, .. },
                ..
            }) = error
            else {
                panic!("r {radius}: expected a typed neighbour refusal, got {error:?}");
            };
            assert_eq!(selected, wall);
            assert_eq!(surface_type, "nurbs");
            assert!(solid_faces(&topo, blended).unwrap().contains(&face));
            assert_eq!(solid_entity_counts(&topo, blended).unwrap(), counts);
        }
    }
}
