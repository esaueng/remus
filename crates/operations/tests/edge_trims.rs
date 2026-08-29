//! Explicit edge trims through boolean assembly (RFC 0002, Stage 3).
//!
//! The GFA pave filler and builder record exact sub-span trims on split
//! edges inside the boolean's working store. Result assembly, analytic
//! shortcuts, solid copies, transforms, and the arena format must carry those
//! intervals without reconstructing them from endpoints.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::f64::consts::TAU;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::copy::copy_solid;
use remus_operations::primitives::{make_cone, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::edge::EdgeCurve;
use remus_topology::{SolidId, Topology};

/// A cylinder whose rim circle edge gets an explicit (partial) trim
/// stamped on it, standing in for a boolean split arc.
fn cylinder_with_trimmed_rim() -> (Topology, remus_topology::SolidId) {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 3.0, 4.0).unwrap();
    let rims: Vec<_> = topo
        .edges()
        .iter()
        .filter(|(_, e)| matches!(e.curve(), EdgeCurve::Circle(_)))
        .map(|(id, _)| id)
        .collect();
    for &rim in &rims {
        topo.edge_mut(rim).unwrap().set_trim(None);
    }
    // Not geometrically meaningful for the closed rim; the point is purely
    // that the stored interval survives every copy path bit-for-bit.
    let mut edge = topo.edge(rims[0]).unwrap().clone();
    edge.set_trim(Some((0.5, 2.5)));
    *topo.edge_mut(rims[0]).unwrap() = edge;
    (topo, solid)
}

fn trimmed_edges(topo: &Topology) -> usize {
    topo.edges()
        .iter()
        .filter(|(_, e)| e.trim().is_some())
        .count()
}

fn solid_trims(topo: &Topology, solid: SolidId) -> Vec<(f64, f64)> {
    let mut trims: Vec<_> = remus_topology::explorer::solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|edge| topo.edge(edge).unwrap().trim())
        .collect();
    trims.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    trims
}

fn stamp_exact_full_circle_trims(topo: &mut Topology, solid: SolidId) {
    for edge_id in remus_topology::explorer::solid_edges(topo, solid).unwrap() {
        let edge = topo.edge(edge_id).unwrap();
        let EdgeCurve::Circle(circle) = edge.curve() else {
            continue;
        };
        let start = topo.vertex(edge.start()).unwrap().point();
        let parameter = circle.project(start);
        topo.edge_mut(edge_id)
            .unwrap()
            .set_trim(Some((parameter, parameter + TAU)));
    }
}

#[test]
fn trims_survive_solid_copy() {
    let (mut topo, solid) = cylinder_with_trimmed_rim();
    assert_eq!(trimmed_edges(&topo), 1);
    let copied = copy_solid(&mut topo, solid).unwrap();
    assert_ne!(copied, solid);
    assert_eq!(
        trimmed_edges(&topo),
        2,
        "the copy must carry the stored trim"
    );
}

#[test]
fn trims_survive_arena_round_trip() {
    let (topo, solid) = cylinder_with_trimmed_rim();
    let bytes = remus_io::arena_io::serialize_solid(&topo, solid).unwrap();
    let mut restored = Topology::new();
    let _ = remus_io::arena_io::deserialize_solid(&bytes, &mut restored).unwrap();
    let trims: Vec<_> = restored
        .edges()
        .iter()
        .filter_map(|(_, e)| e.trim())
        .collect();
    assert_eq!(trims, vec![(0.5, 2.5)]);
}

#[test]
fn coaxial_cylinder_fast_path_preserves_full_circle_trims() {
    let mut topo = Topology::new();
    let lower = make_cylinder(&mut topo, 2.0, 2.0).unwrap();
    let upper = make_cylinder(&mut topo, 2.0, 2.0).unwrap();
    stamp_exact_full_circle_trims(&mut topo, lower);
    stamp_exact_full_circle_trims(&mut topo, upper);
    transform_solid(&mut topo, upper, &Mat4::translation(0.0, 0.0, 1.0)).unwrap();

    let result = boolean(&mut topo, BooleanOp::Fuse, lower, upper).unwrap();
    assert_eq!(
        remus_topology::explorer::solid_entity_counts(&topo, result).unwrap(),
        (3, 3, 2),
        "coaxial shortcut must return one analytic cylinder"
    );
    let report = remus_operations::validate::validate_solid(&topo, result).unwrap();
    assert!(
        report.is_valid(),
        "result must validate: {:?}",
        report.issues
    );
    let result_trims = solid_trims(&topo, result);
    assert_eq!(result_trims.len(), 2);
    assert!(
        result_trims
            .iter()
            .all(|(start, end)| ((end - start) - TAU).abs() < 1e-12),
        "both rebuilt circular rims must retain exact full-turn domains: {result_trims:?}"
    );

    let bytes = remus_io::arena_io::serialize_solid(&topo, result).unwrap();
    let mut restored = Topology::new();
    let restored_solid = remus_io::arena_io::deserialize_solid(&bytes, &mut restored).unwrap();
    assert_eq!(
        solid_trims(&restored, restored_solid),
        result_trims,
        "arena round trip must preserve every assembled interval bit-for-bit"
    );
}

/// A partial arc trim on an input rim must not be stamped onto the rebuilt
/// FULL-circle rims of the coaxial-cylinder shortcut: `circle_param_range`
/// prefers the stored trim over the closed-edge full-turn fallback, so a
/// leaked half-turn skins the result over half its circumference.
#[test]
fn coaxial_cylinder_fast_path_rejects_partial_rim_trims() {
    let mut topo = Topology::new();
    let lower = make_cylinder(&mut topo, 2.0, 2.0).unwrap();
    let upper = make_cylinder(&mut topo, 2.0, 2.0).unwrap();

    // One rim of `lower` carries a half-turn, standing in for a rim that an
    // earlier boolean split into arcs.
    let rim = remus_topology::explorer::solid_edges(&topo, lower)
        .unwrap()
        .into_iter()
        .find(|&e| matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Circle(_)))
        .unwrap();
    topo.edge_mut(rim)
        .unwrap()
        .set_trim(Some((0.0, std::f64::consts::PI)));

    transform_solid(&mut topo, upper, &Mat4::translation(0.0, 0.0, 1.0)).unwrap();
    let result = boolean(&mut topo, BooleanOp::Fuse, lower, upper).unwrap();

    for (start, end) in solid_trims(&topo, result) {
        assert!(
            ((end - start).abs() - TAU).abs() < 1e-9,
            "rebuilt full rim must not inherit a partial arc span: ({start}, {end})"
        );
    }

    let mut mesh_area = 0.0;
    for face in remus_topology::explorer::solid_faces(&topo, result).unwrap() {
        let mesh = remus_operations::tessellate::tessellate(&topo, face, 0.001).unwrap();
        for t in mesh.indices.chunks(3) {
            let p = |i: usize| mesh.positions[t[i] as usize];
            mesh_area += (p(1) - p(0)).cross(p(2) - p(0)).length() * 0.5;
        }
    }
    // Lateral 2*pi*r*h + two caps, r = 2, h = 3.
    let expected = TAU * 2.0 * 3.0 + 2.0 * std::f64::consts::PI * 4.0;
    assert!(
        (mesh_area - expected).abs() < 0.05,
        "result must mesh over its whole circumference: {mesh_area} vs {expected}"
    );
}

/// Each rebuilt rim anchors its stored interval at its OWN start vertex.
#[test]
fn coaxial_cylinder_fast_path_keeps_each_rim_in_phase() {
    let mut topo = Topology::new();
    let lower = make_cylinder(&mut topo, 2.0, 2.0).unwrap();
    let upper = make_cylinder(&mut topo, 2.0, 2.0).unwrap();
    stamp_exact_full_circle_trims(&mut topo, lower);
    stamp_exact_full_circle_trims(&mut topo, upper);
    transform_solid(&mut topo, upper, &Mat4::translation(0.0, 0.0, 1.0)).unwrap();

    let result = boolean(&mut topo, BooleanOp::Fuse, lower, upper).unwrap();
    for edge_id in remus_topology::explorer::solid_edges(&topo, result).unwrap() {
        let edge = topo.edge(edge_id).unwrap();
        let EdgeCurve::Circle(circle) = edge.curve() else {
            continue;
        };
        let (t0, _) = edge.trim().expect("rebuilt rim carries an exact interval");
        let anchor = circle.project(topo.vertex(edge.start()).unwrap().point());
        assert!(
            (t0 - anchor).abs() < 1e-9,
            "stored interval must start at this rim's own vertex: {t0} vs {anchor}"
        );
    }
}

/// Same full-circle contract as the cylinder shortcut, on the coaxial-cone
/// frustum merge.
#[test]
fn coaxial_cone_fast_path_preserves_full_circle_trims() {
    let mut topo = Topology::new();
    // Frustums of the same cone (apex z = -2, slope 0.5): r = 1..2 and 2..3.
    let lower = make_cone(&mut topo, 1.0, 2.0, 2.0).unwrap();
    let upper = make_cone(&mut topo, 2.0, 3.0, 2.0).unwrap();
    stamp_exact_full_circle_trims(&mut topo, lower);
    stamp_exact_full_circle_trims(&mut topo, upper);
    transform_solid(&mut topo, upper, &Mat4::translation(0.0, 0.0, 2.0)).unwrap();

    let result = boolean(&mut topo, BooleanOp::Fuse, lower, upper).unwrap();
    assert_eq!(
        remus_topology::explorer::solid_entity_counts(&topo, result).unwrap(),
        (3, 3, 2),
        "coaxial shortcut must return one analytic cone frustum"
    );
    let result_trims = solid_trims(&topo, result);
    assert_eq!(result_trims.len(), 2, "both rims must carry intervals");
    assert!(
        result_trims
            .iter()
            .all(|(start, end)| ((end - start) - TAU).abs() < 1e-12),
        "rebuilt rims must retain exact full-turn domains: {result_trims:?}"
    );

    let vol = remus_operations::measure::solid_volume(&topo, result, 0.01).unwrap();
    // Frustum of slope 0.5 from z=0 (r=1) to z=4 (r=3):
    // V = pi*h/3*(R^2 + R*r + r^2) = pi*4/3*(9+3+1) = 52/3*pi.
    let expected = 52.0 / 3.0 * std::f64::consts::PI;
    assert!(
        (vol - expected).abs() < 1e-3,
        "volume oracle: {vol} vs {expected}"
    );
}

/// Analytic fast paths that mint arc edges into RESULT topology must store
/// the exact CCW span instead of leaving consumers to re-derive it through
/// the projection fallback (RFC 0002 reader migration). The box-sphere
/// octant shortcut's three boundary arcs are quarter circles.
#[test]
fn box_sphere_intersect_shortcut_arcs_carry_quarter_span_trims() {
    let mut topo = Topology::new();
    let bx = remus_operations::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let sp = remus_operations::primitives::make_sphere(&mut topo, 7.0, 16).unwrap();
    let result = boolean(&mut topo, BooleanOp::Intersect, bx, sp).unwrap();

    let mut arc_trims = Vec::new();
    for edge_id in remus_topology::explorer::solid_edges(&topo, result).unwrap() {
        let edge = topo.edge(edge_id).unwrap();
        if let EdgeCurve::Circle(_) = edge.curve() {
            arc_trims.push(edge.trim().expect("octant arc carries an exact span"));
        }
    }
    assert_eq!(arc_trims.len(), 3, "three quarter-arc edges: {arc_trims:?}");
    for (t0, t1) in &arc_trims {
        assert!(
            (t1 - t0 - std::f64::consts::FRAC_PI_2).abs() < 1e-9,
            "each octant arc spans exactly a quarter turn: ({t0}, {t1})"
        );
    }

    // The stored span must anchor at each arc's own start vertex.
    for edge_id in remus_topology::explorer::solid_edges(&topo, result).unwrap() {
        let edge = topo.edge(edge_id).unwrap();
        if let EdgeCurve::Circle(circle) = edge.curve() {
            let (t0, _) = edge.trim().expect("octant arc carries an exact span");
            let anchor = circle.project(topo.vertex(edge.start()).unwrap().point());
            assert!(
                (t0 - anchor).abs() < 1e-9,
                "trim anchors at the arc's own start vertex: {t0} vs {anchor}"
            );
        }
    }

    // The analytic census and oracle are unchanged by storing the spans.
    let (f, e, v) = remus_topology::explorer::solid_entity_counts(&topo, result).unwrap();
    assert_eq!((f, e, v), (4, 6, 4), "octant topology unchanged");
    let vol = remus_operations::measure::solid_volume(&topo, result, 0.1).unwrap();
    assert!(vol > 0.0 && vol < 4.0 / 3.0 * std::f64::consts::PI * 343.0);
}

/// `copy_and_transform_solid` must apply the same exact trim policy as
/// `transform_edges` (they duplicated the curve math until the helper was
/// shared): retain under a similarity, remap the Circle→Ellipse axis swap,
/// drop only where the parameterization provably changes.
#[test]
fn copy_and_transform_solid_follows_the_exact_trim_policy() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 3.0, 4.0).unwrap();
    let rim = remus_topology::explorer::solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&e| matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Circle(_)))
        .unwrap();
    topo.edge_mut(rim).unwrap().set_trim(Some((0.5, 2.5)));

    // Pure translation: the parameterization is untouched — trim retained.
    let moved = remus_operations::copy::copy_and_transform_solid(
        &mut topo,
        solid,
        &Mat4::translation(5.0, 0.0, 0.0),
    )
    .unwrap();
    let moved_trims = solid_trims(&topo, moved);
    assert_eq!(
        moved_trims,
        vec![(0.5, 2.5)],
        "translation retains the trim"
    );

    // Rotation: still a similarity — trim retained.
    let rotated = remus_operations::copy::copy_and_transform_solid(
        &mut topo,
        solid,
        &Mat4::rotation_z(std::f64::consts::FRAC_PI_2),
    )
    .unwrap();
    let rotated_trims = solid_trims(&topo, rotated);
    assert_eq!(rotated_trims, vec![(0.5, 2.5)], "rotation retains the trim");

    // Anisotropic scale: the Circle becomes an Ellipse. Whichever arm the
    // circle's reference frame lands in, the policy outcome is exact:
    // major-first retains the interval; major-second remaps t ↦ π/2 − t.
    let stretched = remus_operations::copy::copy_and_transform_solid(
        &mut topo,
        solid,
        &Mat4::scale(2.0, 1.0, 1.0),
    )
    .unwrap();
    let stretched_trims = solid_trims(&topo, stretched);
    let remapped = (
        std::f64::consts::FRAC_PI_2 - 0.5,
        std::f64::consts::FRAC_PI_2 - 2.5,
    );
    assert!(
        stretched_trims == vec![(0.5, 2.5)] || stretched_trims == vec![remapped],
        "scale follows the exact Circle→Ellipse trim policy: {stretched_trims:?}"
    );
}
