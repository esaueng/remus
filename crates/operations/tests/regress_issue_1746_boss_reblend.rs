//! Issue #1746: a boss on a box corner edge, its joint rounded R2 together
//! with the two box top edges that run into it, then R1 on the boss rim and
//! three more box edges, one fillet each (the later R1 fillets run on round
//! the block's bottom loop and vertical edges). `fillet_cascade` builds every
//! one of those fillets, and the boss diameter edit rebuilds the group they
//! form.
//!
//! The group's removal restores the boss rim as one closed circle and its
//! bottom arc as one 270-degree arc, where the primitive had quarter arcs.
//! On those edges no planned stage order re-fillets the group, at the
//! unchanged diameter too (the R1 stage refuses with two stripes meeting at
//! the block's far top corner). The route then removes the group again with
//! every restored arc cut below a half turn, and the planned order builds.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::compact_sort::CompactSort;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_cluster::{
    BlendCluster, BlendMemberForm, Convexity, recognize_blend_cluster, resize_band_through_group,
    resize_cylinder_through_group,
};
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::query::filter_filletable_edges;
use remus_operations::tessellate::{edge_census, tessellate_solid};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use std::f64::consts::PI;

const EPS: f64 = 1e-6;

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < EPS
}

/// Edges of one kind (circle or not) whose both endpoints satisfy `on`.
fn edges_where(
    topo: &Topology,
    solid: SolidId,
    circle: bool,
    on: impl Fn(Point3) -> bool,
) -> Vec<EdgeId> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let data = topo.edge(edge).unwrap();
            let start = topo.vertex(data.start()).unwrap().point();
            let end = topo.vertex(data.end()).unwrap().point();
            matches!(data.curve(), EdgeCurve::Circle(_)) == circle && on(start) && on(end)
        })
        .collect()
}

/// Fillet as the WASM `fillet` binding does: filletable edges only, then
/// the cascade.
fn fillet(topo: &mut Topology, solid: SolidId, edges: &[EdgeId], radius: f64) -> SolidId {
    assert!(!edges.is_empty(), "empty selection at R{radius}");
    let edges = filter_filletable_edges(topo, solid, edges).unwrap();
    fillet_cascade(topo, solid, &edges, radius)
        .unwrap_or_else(|error| panic!("R{radius} fillet of {edges:?}: {error}"))
        .solid
}

/// The reported part: a 50 x 36 x 12.5 block and an R6 x 28 boss on its
/// x = 0, y = 0 vertical edge, filleted in the reported order.
fn part() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 50.0, 36.0, 12.5).unwrap();
    let boss = make_cylinder(&mut topo, 6.0, 28.0).unwrap();
    let mut solid = boolean(&mut topo, BooleanOp::Fuse, block, boss).unwrap();

    // R2: the joint (its arc on the block top and its two vertical seams)
    // and the two block top edges that run into it.
    let mut joint = edges_where(&topo, solid, true, |p| near(p.z(), 12.5));
    joint.extend(edges_where(&topo, solid, false, |p| {
        (near(p.x(), 6.0) && near(p.y(), 0.0)) || (near(p.x(), 0.0) && near(p.y(), 6.0))
    }));
    joint.extend(edges_where(&topo, solid, false, |p| {
        near(p.z(), 12.5) && (near(p.y(), 0.0) || near(p.x(), 0.0))
    }));
    solid = fillet(&mut topo, solid, &joint, 2.0);

    // R1: the boss rim, then three block edges, one fillet each.
    let rim = edges_where(&topo, solid, true, |p| near(p.z(), 28.0));
    solid = fillet(&mut topo, solid, &rim, 1.0);
    for on in [
        (|p: Point3| near(p.x(), 50.0) && near(p.z(), 12.5)) as fn(Point3) -> bool,
        |p| near(p.y(), 36.0) && near(p.z(), 12.5),
        |p| near(p.y(), 0.0) && near(p.z(), 0.0) && p.x() > 6.0,
    ] {
        let edges = edges_where(&topo, solid, false, on);
        solid = fillet(&mut topo, solid, &edges, 1.0);
    }
    (topo, solid)
}

/// The boss wall: the one cylinder of `radius` about the z axis.
fn boss_wall(topo: &Topology, solid: SolidId, radius: f64) -> FaceId {
    let walls: Vec<FaceId> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
                if near(c.radius(), radius)
                    && c.axis().cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9
                    && c.origin().x().abs() < 1e-9
                    && c.origin().y().abs() < 1e-9)
        })
        .collect();
    assert_eq!(walls.len(), 1, "boss wall of radius {radius}");
    walls[0]
}

/// Each member's kind, side, radius and exactness, sorted: what a rebuild at
/// the recorded radii must reproduce.
fn signature(cluster: &BlendCluster) -> Vec<String> {
    let mut members: Vec<String> = cluster
        .members
        .iter()
        .map(|member| {
            format!(
                "{:?} {:?} R{:.6} exact={}",
                member.kind,
                member.convexity,
                member.radius,
                member.form.is_exact()
            )
        })
        .collect();
    members.compact_sort_unstable();
    members
}

/// Strict validation, and a closed manifold mesh whose volume agrees with
/// the API volume.
fn assert_sound(topo: &Topology, solid: SolidId, label: &str) -> f64 {
    let report = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(report.is_valid(), "{label}: {:?}", report.issues);
    let mesh = tessellate_solid(topo, solid, 0.01).unwrap();
    let census = edge_census(&mesh);
    assert_eq!(census.boundary_edges, 0, "{label}: open mesh");
    assert_eq!(census.non_manifold_edges, 0, "{label}: non-manifold mesh");
    let origin = mesh.positions[0];
    let meshed: f64 = mesh
        .indices
        .chunks_exact(3)
        .map(|t| {
            let a = mesh.positions[t[0] as usize] - origin;
            let b = mesh.positions[t[1] as usize] - origin;
            let c = mesh.positions[t[2] as usize] - origin;
            a.dot(b.cross(c)) / 6.0
        })
        .sum();
    let volume = solid_volume(topo, solid, 0.01).unwrap();
    assert!(
        (meshed - volume).abs() <= 1e-3 * volume,
        "{label}: mesh volume {meshed} against {volume}"
    );
    volume
}

#[test]
fn boss_diameter_rebuilds_the_group_it_carries() {
    let (topo, solid) = part();
    let wall = boss_wall(&topo, solid, 6.0);
    let before = recognize_blend_cluster(&topo, solid, wall).unwrap();
    assert_eq!(before.members.len(), 26);
    let counts = solid_entity_counts(&topo, solid).unwrap();
    let volume = assert_sound(&topo, solid, "input");
    for radius in [6.2, 7.0, 5.5, 14.0] {
        let mut edited = topo.clone();
        let edit = resize_cylinder_through_group(&mut edited, solid, wall, radius)
            .unwrap_or_else(|error| panic!("R{radius}: {error}"))
            .expect("the boss wall carries a blend group");
        let label = format!("R{radius}");
        let result = assert_sound(&edited, edit.solid, &label);
        // The wall is at the new radius, and the group it carries has every
        // member back at its recorded radius, side and exactness.
        let new_wall = boss_wall(&edited, edit.solid, radius);
        let after = recognize_blend_cluster(&edited, edit.solid, new_wall).unwrap();
        assert_eq!(signature(&after), signature(&before), "{label}");
        assert_eq!(
            solid_entity_counts(&edited, edit.solid).unwrap().0,
            counts.0,
            "{label}"
        );
        // Only the boss changed: its full turn above the block and its three
        // quarters outside it, to within the blends' share of the change.
        let area = PI * (radius * radius - 36.0);
        let expected = area * (28.0 - 12.5) + 0.75 * area * 12.5;
        assert!(
            ((result - volume) - expected).abs() <= 0.01 * expected.abs() + 1.0,
            "{label}: volume {volume} -> {result}, expected a change of {expected}"
        );
        // The source is untouched.
        assert_eq!(solid_entity_counts(&edited, solid).unwrap(), counts);
    }
}

#[test]
fn joint_radius_rebuilds_the_group_it_belongs_to() {
    let (mut edited, solid) = part();
    let wall = boss_wall(&edited, solid, 6.0);
    let before = recognize_blend_cluster(&edited, solid, wall).unwrap();
    // The joint's arc round the boss foot on the block top.
    let joint = before
        .members
        .iter()
        .find(|member| {
            member.form == BlendMemberForm::Torus && member.convexity == Convexity::Concave
        })
        .expect("the R2 joint band");
    assert!(near(joint.radius, 2.0));
    let result = resize_band_through_group(&mut edited, solid, joint.faces[0], 2.0, 2.5)
        .unwrap_or_else(|error| panic!("R2.5 joint: {error}"))
        .expect("the joint is a band of a recognized group");
    assert_sound(&edited, result.solid, "R2.5 joint");
    let after =
        recognize_blend_cluster(&edited, result.solid, boss_wall(&edited, result.solid, 6.0))
            .unwrap();
    assert_eq!(after.members.len(), before.members.len());
    let joints = after
        .members
        .iter()
        .filter(|member| {
            member.form == BlendMemberForm::Torus && member.convexity == Convexity::Concave
        })
        .map(|member| member.radius)
        .collect::<Vec<_>>();
    assert_eq!(joints.len(), 1);
    assert!(near(joints[0], 2.5), "{joints:?}");
}
