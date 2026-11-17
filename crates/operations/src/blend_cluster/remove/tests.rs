#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::explorer::{face_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};

use super::remove_blend_cluster;
use crate::OperationsError;
use crate::blend_cluster::{
    BlendCluster, BlendClusterError, BlendMember, BlendMemberForm, BlendMemberKind, Convexity,
    MemberSharp, SharpCarrier, SharpEdge,
};
use crate::blend_ops::fillet_cascade;
use crate::boolean::{BooleanOp, boolean};
use crate::primitives::{make_cylinder, make_sphere};

fn tagged(topo: &Topology, faces: &[FaceId], tag: &str) -> Vec<FaceId> {
    faces
        .iter()
        .copied()
        .filter(|face| topo.face(*face).unwrap().surface().type_tag() == tag)
        .collect()
}

/// A band whose root supports include a sphere is refused before anything
/// is allocated, whatever the recognizer would say about the body.
#[test]
fn sphere_support_is_refused_before_any_allocation() {
    let mut topo = Topology::new();
    let ball = make_sphere(&mut topo, 20.0, 16).unwrap();
    let boss = make_cylinder(&mut topo, 6.0, 20.0).unwrap();
    crate::transform::transform_solid(&mut topo, boss, &Mat4::translation(0.0, 0.0, 10.0)).unwrap();
    let body = boolean(&mut topo, BooleanOp::Fuse, ball, boss).unwrap();
    let faces = solid_faces(&topo, body).unwrap();
    let wall = tagged(&topo, &faces, "cylinder")[0];
    let adjacency = topo.build_adjacency(body).unwrap();
    let seam: Vec<_> = face_edges(&topo, wall)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            adjacency
                .faces_for_edge(edge)
                .iter()
                .any(|f| topo.face(*f).unwrap().surface().type_tag() == "sphere")
        })
        .collect();
    let rounded = fillet_cascade(&mut topo, body, &seam, 2.0).unwrap().solid;
    let faces = solid_faces(&topo, rounded).unwrap();
    let band = tagged(&topo, &faces, "torus")[0];
    let wall = tagged(&topo, &faces, "cylinder")[0];
    let adjacency = topo.build_adjacency(rounded).unwrap();
    let ball = *adjacency
        .neighbors_of_face(band)
        .iter()
        .find(|f| topo.face(**f).unwrap().surface().type_tag() == "sphere")
        .unwrap();
    let FaceSurface::Torus(torus) = topo.face(band).unwrap().surface() else {
        unreachable!()
    };
    let height = torus.center().z();
    let mut supports = vec![wall, ball];
    supports.sort_by_key(|face| face.index());
    let cluster = BlendCluster {
        seed: band,
        seed_is_member: true,
        members: vec![BlendMember {
            faces: vec![band],
            kind: BlendMemberKind::Band,
            form: BlendMemberForm::Torus,
            radius: 2.0,
            convexity: Convexity::Concave,
            supports: supports.clone(),
            root_supports: supports.clone(),
            contact_angle: 0.0,
            sharp: MemberSharp::Edge(0),
        }],
        supports: supports.clone(),
        boundary_faces: Vec::new(),
        sharp_edges: vec![SharpEdge {
            member: 0,
            supports: [supports[0], supports[1]],
            carrier: SharpCarrier::Circle {
                center: Point3::new(0.0, 0.0, height),
                axis: Vec3::new(0.0, 0.0, 1.0),
                radius: 6.0,
            },
            ends: None,
            midpoint: Point3::new(6.0, 0.0, height),
        }],
        sharp_vertices: Vec::new(),
        reblend_order: Vec::new(),
    };
    let slots = topo.allocated_slot_count();
    let counts = solid_entity_counts(&topo, rounded).unwrap();
    match remove_blend_cluster(&mut topo, rounded, &cluster) {
        Err(OperationsError::BlendCluster(BlendClusterError::UnsupportedSupport {
            face,
            surface,
        })) => {
            assert_eq!(face, ball.index());
            assert_eq!(surface, "sphere");
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(topo.allocated_slot_count(), slots);
    assert_eq!(solid_entity_counts(&topo, rounded).unwrap(), counts);
}

/// The cuts [`super::RestoredArcs::UnderHalfTurn`] adds: none below a half
/// turn, then the fewest equal pieces each shorter than one.
#[test]
fn restored_arcs_cut_below_a_half_turn() {
    use std::f64::consts::{PI, TAU};

    use super::RestoredArcs;
    assert!(RestoredArcs::Predicted.cuts(TAU).is_empty());
    assert!(RestoredArcs::UnderHalfTurn.cuts(0.9 * PI).is_empty());
    assert_eq!(RestoredArcs::UnderHalfTurn.cuts(PI), vec![0.5 * PI]);
    assert_eq!(RestoredArcs::UnderHalfTurn.cuts(1.5 * PI), vec![0.75 * PI]);
    let closed = RestoredArcs::UnderHalfTurn.cuts(TAU);
    assert_eq!(closed.len(), 2);
    assert!((closed[0] - TAU / 3.0).abs() < 1e-12 && (closed[1] - 2.0 * TAU / 3.0).abs() < 1e-12);
}

/// A rounded boss rim restores as one closed circle, or cut into three
/// arcs: the same sharp body either way.
#[test]
fn a_rim_restored_with_cut_arcs_is_the_same_body() {
    use super::{RestoredArcs, has_long_arc, remove_freshly_recognized_cluster};
    use crate::blend_cluster::recognize_blend_cluster;
    use remus_topology::edge::EdgeCurve;

    let mut topo = Topology::new();
    let boss = make_cylinder(&mut topo, 6.0, 20.0).unwrap();
    let faces = solid_faces(&topo, boss).unwrap();
    let wall = tagged(&topo, &faces, "cylinder")[0];
    let adjacency = topo.build_adjacency(boss).unwrap();
    let rim: Vec<_> = face_edges(&topo, wall)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Circle(_))
                && adjacency.faces_for_edge(edge).iter().any(|&face| {
                    matches!(topo.face(face).unwrap().surface(), FaceSurface::Plane { d, .. }
                        if d.abs() > 1.0)
                })
        })
        .collect();
    let rounded = fillet_cascade(&mut topo, boss, &rim, 1.0).unwrap().solid;
    let faces = solid_faces(&topo, rounded).unwrap();
    let wall = tagged(&topo, &faces, "cylinder")[0];
    let cluster = recognize_blend_cluster(&topo, rounded, wall).unwrap();
    let whole =
        remove_freshly_recognized_cluster(&mut topo, rounded, &cluster, RestoredArcs::Predicted)
            .unwrap();
    let cut = remove_freshly_recognized_cluster(
        &mut topo,
        rounded,
        &cluster,
        RestoredArcs::UnderHalfTurn,
    )
    .unwrap();
    assert!(has_long_arc(&topo, &whole).unwrap());
    assert!(!has_long_arc(&topo, &cut).unwrap());
    let pieces = |restoration: &super::SharpRestoration| {
        restoration.sharp_edges.iter().map(Vec::len).sum::<usize>()
    };
    assert_eq!(pieces(&whole), 1);
    assert_eq!(pieces(&cut), 3);
    let (faces, _, _) = solid_entity_counts(&topo, whole.solid).unwrap();
    assert_eq!(solid_entity_counts(&topo, cut.solid).unwrap().0, faces);
    let volume = |solid| crate::measure::solid_volume(&topo, solid, 0.001).unwrap();
    assert!((volume(whole.solid) - volume(cut.solid)).abs() < 1e-6 * volume(whole.solid));
    assert!(
        (volume(cut.solid) - std::f64::consts::PI * 36.0 * 20.0).abs() < 1e-3 * volume(cut.solid)
    );
}

/// A box's top loop rounded at R2: where two stripes meet over a sharp
/// vertical edge, the rolling-ball engine closes the corner with the ball's
/// patch over a planar ledge one radius down that edge. The ledge is part of
/// its corner (#1744): the group removes, ledges and all, to the sharp box,
/// with each vertical edge running up to the box corner again.
#[test]
fn a_lid_with_ledged_corners_removes_to_the_sharp_box() {
    use crate::blend_cluster::recognize_blend_cluster;
    use remus_topology::explorer::{solid_edges, solid_vertices};

    let mut topo = Topology::new();
    let body = crate::primitives::make_box(&mut topo, 40.0, 40.0, 20.0).unwrap();
    let top: Vec<_> = solid_edges(&topo, body)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let data = topo.edge(edge).unwrap();
            [data.start(), data.end()]
                .iter()
                .all(|&v| (topo.vertex(v).unwrap().point().z() - 20.0).abs() < 1e-9)
        })
        .collect();
    assert_eq!(top.len(), 4);
    let lid = fillet_cascade(&mut topo, body, &top, 2.0).unwrap().solid;
    let faces = solid_faces(&topo, lid).unwrap();
    let ledges: Vec<FaceId> = tagged(&topo, &faces, "plane")
        .into_iter()
        .filter(|&face| face_edges(&topo, face).unwrap().len() == 3)
        .collect();
    assert_eq!(ledges.len(), 4, "one ledge under each corner ball");

    let band = tagged(&topo, &faces, "cylinder")[0];
    let cluster = recognize_blend_cluster(&topo, lid, band).unwrap();
    assert!(cluster.boundary_faces.is_empty());
    let corners: Vec<&BlendMember> = cluster
        .members
        .iter()
        .filter(|member| member.kind == BlendMemberKind::CornerPatch)
        .collect();
    assert_eq!(corners.len(), 4);
    for corner in corners {
        assert_eq!(corner.form, BlendMemberForm::Sphere);
        let [ball, ledge] = corner.faces[..] else {
            panic!("a corner is its ball and its ledge: {:?}", corner.faces);
        };
        assert_eq!(topo.face(ball).unwrap().surface().type_tag(), "sphere");
        assert!(ledges.contains(&ledge));
    }

    let before = solid_entity_counts(&topo, lid).unwrap();
    let restoration = remove_blend_cluster(&mut topo, lid, &cluster).unwrap();
    let sharp = restoration.solid;
    assert_eq!(solid_entity_counts(&topo, sharp).unwrap(), (6, 12, 8));
    let faces = solid_faces(&topo, sharp).unwrap();
    assert_eq!(tagged(&topo, &faces, "plane").len(), 6);
    let report = crate::validate::validate_solid(&topo, sharp).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);
    let mut corners: Vec<[f64; 3]> = solid_vertices(&topo, sharp)
        .unwrap()
        .into_iter()
        .map(|v| {
            let p = topo.vertex(v).unwrap().point();
            [p.x(), p.y(), p.z()]
        })
        .collect();
    corners.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut expected = Vec::new();
    for x in [0.0, 40.0] {
        for y in [0.0, 40.0] {
            for z in [0.0, 20.0] {
                expected.push([x, y, z]);
            }
        }
    }
    for (found, wanted) in corners.iter().zip(&expected) {
        assert!(
            found.iter().zip(wanted).all(|(a, b)| (a - b).abs() < 1e-9),
            "sharp box corner {found:?}, expected {wanted:?}"
        );
    }
    let volume = crate::measure::solid_volume(&topo, sharp, 0.001).unwrap();
    assert!((volume - 32000.0).abs() < 1e-6 * 32000.0, "volume {volume}");
    // The lid is left as it was.
    assert_eq!(solid_entity_counts(&topo, lid).unwrap(), before);
}
