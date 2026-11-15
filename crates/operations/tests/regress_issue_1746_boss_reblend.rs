//! Issue #1746: a boss on a box corner edge, its joint rounded R2 together
//! with the two box top edges that run into it, then R1 on the boss rim and
//! three more box edges, one fillet each (the later R1 fillets run on round
//! the block's bottom loop and vertical edges). `fillet_cascade` builds every
//! one of those fillets, but the dependent blend group does not re-fillet on
//! the sharp body in any planned stage order: the R1 stage meets the block's
//! far top corner as two stripes at a sharp vertex. That holds at the
//! unchanged diameter too, so the refusal must say the group failed to
//! re-fillet and must not advise a smaller diameter.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::blend_cluster::{
    BlendClusterError, DiameterRefusalCause, resize_cylinder_through_group,
};
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::query::filter_filletable_edges;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

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

fn boss_wall(topo: &Topology, solid: SolidId) -> FaceId {
    let walls: Vec<FaceId> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
                if near(c.radius(), 6.0)
                    && c.axis().cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9
                    && c.origin().x().abs() < 1e-9
                    && c.origin().y().abs() < 1e-9)
        })
        .collect();
    assert_eq!(walls.len(), 1, "boss wall");
    walls[0]
}

#[test]
fn group_reblend_failure_is_not_reported_as_a_size_limit() {
    let (mut topo, solid) = part();
    let wall = boss_wall(&topo, solid);
    let counts = solid_entity_counts(&topo, solid).unwrap();
    let volume = solid_volume(&topo, solid, 0.01).unwrap();
    for radius in [6.2, 7.0] {
        let error = resize_cylinder_through_group(&mut topo, solid, wall, radius).unwrap_err();
        assert!(
            matches!(
                &error,
                OperationsError::BlendCluster(BlendClusterError::DiameterUnbuildable {
                    cause: DiameterRefusalCause::GroupReblendFailed,
                    ..
                })
            ),
            "R{radius}: {error}"
        );
        let OperationsError::BlendCluster(inner) = &error else {
            unreachable!()
        };
        assert_eq!(inner.code(), "cylinder-diameter-unbuildable");
        let message = error.to_string();
        assert!(!message.contains("try a smaller diameter"), "{message}");
        assert!(message.contains("do not re-fillet"), "{message}");
        // The refusal edits nothing.
        assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
        assert!(near(solid_volume(&topo, solid, 0.01).unwrap(), volume));
    }
}
