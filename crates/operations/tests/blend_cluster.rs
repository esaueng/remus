//! Blend-cluster recognition on rounded boss/block bodies.
//!
//! The reference is the Gentle-Gecko part: a 30 x 18 x 24 block with an R6
//! boss on its corner, its boss seams, shoulder and two top edges rounded at
//! R4. Two of its fifteen faces are sampled corner patches (rational
//! degree-(2,1) carriers through 513 exact rolling sections), so no
//! exact-only recognizer can close the group. The recognizer must return the
//! whole group from any face that carries or is a blend, measure the corner
//! patches against the rolling-ball closed form, and predict the sharp edges
//! and vertices of the unfilleted part: those are checked edge for edge
//! against the sharp parent STEP, and against the sharp stages of natively
//! built bodies.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::blend_cluster::{
    BlendCluster, BlendClusterError, BlendMemberForm, BlendMemberKind, Convexity, MemberSharp,
    SharpCarrier, recognize_blend_cluster,
};
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::fillet::{FilletRadiusLaw, fillet_variable};
use remus_operations::heal::convert_to_bspline;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::query::edge_is_g1;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{
    face_edges, face_vertices, solid_edges, solid_faces, solid_vertices,
};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

fn import(text: &str) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solid = remus_io::step::reader::read_step(text, &mut topo).unwrap()[0];
    (topo, solid)
}

fn gecko() -> (Topology, SolidId) {
    import(include_str!("data/gentle_gecko_filleted.step"))
}

fn gecko_sharp() -> (Topology, SolidId) {
    import(include_str!("data/gentle_gecko_sharp.step"))
}

fn refusal(result: Result<BlendCluster, OperationsError>) -> BlendClusterError {
    match result {
        Err(OperationsError::BlendCluster(error)) => error,
        Err(other) => panic!("expected a typed blend-cluster refusal, got {other}"),
        Ok(cluster) => panic!("expected a refusal, got {} members", cluster.members.len()),
    }
}

/// Faces whose surface is a plane with outward normal `normal` at offset `d`.
fn plane(topo: &Topology, solid: SolidId, normal: Vec3, d: f64) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            let data = topo.face(face).unwrap();
            matches!(data.surface(), FaceSurface::Plane { normal: n, d: e }
                if (*n - normal).length() < 1e-9 && (e - d).abs() < 1e-9)
        })
        .unwrap()
}

fn same_carrier(a: &FaceSurface, b: &FaceSurface) -> bool {
    match (a, b) {
        (FaceSurface::Plane { normal: n1, d: d1 }, FaceSurface::Plane { normal: n2, d: d2 }) => {
            (*n1 - *n2).length() < 1e-9 && (d1 - d2).abs() < 1e-9
                || (*n1 + *n2).length() < 1e-9 && (d1 + d2).abs() < 1e-9
        }
        (FaceSurface::Cylinder(c1), FaceSurface::Cylinder(c2)) => {
            let offset = c2.origin() - c1.origin();
            let radial = offset - c1.axis() * offset.dot(c1.axis());
            (c1.radius() - c2.radius()).abs() < 1e-9
                && c1.axis().cross(c2.axis()).length() < 1e-9
                && radial.length() < 1e-9
        }
        _ => false,
    }
}

fn edge_point(topo: &Topology, edge: EdgeId, fraction: f64) -> Point3 {
    let data = topo.edge(edge).unwrap();
    let a = topo.vertex(data.start()).unwrap().point();
    let b = topo.vertex(data.end()).unwrap().point();
    let (lo, hi) = data.strict_domain().expect("edge parameter authority");
    data.curve()
        .evaluate_with_endpoints(lo + (hi - lo) * fraction, a, b)
}

fn on_carrier(carrier: SharpCarrier, point: Point3) -> f64 {
    match carrier {
        SharpCarrier::Line {
            point: base,
            direction,
        } => {
            let offset = point - base;
            (offset - direction * offset.dot(direction)).length()
        }
        SharpCarrier::Circle {
            center,
            axis,
            radius,
        } => {
            let offset = point - center;
            let height = offset.dot(axis);
            let planar = (offset - axis * height).length();
            height.abs().max((planar - radius).abs())
        }
    }
}

/// Every predicted sharp vertex is a vertex of the sharp body, and every
/// predicted sharp edge is one of its edges: same ends, same carrier, same
/// span (the predicted midpoint is the edge's own), between faces on the
/// predicted supports' carriers.
fn assert_sharp_matches(
    cluster: &BlendCluster,
    topo: &Topology,
    sharp_topo: &Topology,
    sharp: SolidId,
) {
    let vertices: Vec<Point3> = solid_vertices(sharp_topo, sharp)
        .unwrap()
        .into_iter()
        .map(|vertex| sharp_topo.vertex(vertex).unwrap().point())
        .collect();
    for vertex in &cluster.sharp_vertices {
        assert!(
            vertices
                .iter()
                .any(|point| (*point - vertex.point).length() < 1e-6),
            "predicted vertex {:?} is not in the sharp body",
            vertex.point
        );
    }
    let adjacency = sharp_topo.build_adjacency(sharp).unwrap();
    let mut matched = Vec::new();
    for edge in &cluster.sharp_edges {
        let found = solid_edges(sharp_topo, sharp)
            .unwrap()
            .into_iter()
            .filter(|&candidate| {
                let data = sharp_topo.edge(candidate).unwrap();
                let a = sharp_topo.vertex(data.start()).unwrap().point();
                let b = sharp_topo.vertex(data.end()).unwrap().point();
                let ends_match = match edge.ends {
                    Some([s, e]) => {
                        let (s, e) = (
                            cluster.sharp_vertices[s].point,
                            cluster.sharp_vertices[e].point,
                        );
                        ((a - s).length() < 1e-6 && (b - e).length() < 1e-6)
                            || ((a - e).length() < 1e-6 && (b - s).length() < 1e-6)
                    }
                    // A closed circle may be split into arcs in the sharp body.
                    None => {
                        on_carrier(edge.carrier, a) < 1e-6 && on_carrier(edge.carrier, b) < 1e-6
                    }
                };
                ends_match
                    && [0.25, 0.5, 0.75].iter().all(|fraction| {
                        on_carrier(edge.carrier, edge_point(sharp_topo, candidate, *fraction))
                            < 1e-6
                    })
                    && (edge.ends.is_none()
                        || (edge_point(sharp_topo, candidate, 0.5) - edge.midpoint).length() < 1e-6)
            })
            .collect::<Vec<_>>();
        if let SharpCarrier::Circle { radius, .. } = edge.carrier
            && edge.ends.is_none()
        {
            // The arcs must close the whole circle.
            let length: f64 = found
                .iter()
                .map(|&arc| {
                    (0..1024)
                        .map(|step| {
                            let a = edge_point(sharp_topo, arc, f64::from(step) / 1024.0);
                            let b = edge_point(sharp_topo, arc, f64::from(step + 1) / 1024.0);
                            (b - a).length()
                        })
                        .sum::<f64>()
                })
                .sum();
            assert!(
                (length - std::f64::consts::TAU * radius).abs() < 1e-3 * radius,
                "closed sharp circle covered to length {length}"
            );
        } else {
            assert_eq!(
                found.len(),
                1,
                "predicted sharp edge {:?} matches {} sharp edges",
                edge.carrier,
                found.len()
            );
        }
        for found in found {
            let faces = adjacency.faces_for_edge(found);
            for support in edge.supports {
                let surface = topo.face(support).unwrap().surface();
                assert!(
                    faces.iter().any(|face| {
                        same_carrier(sharp_topo.face(*face).unwrap().surface(), surface)
                    }),
                    "sharp edge {:?} is not on support {}",
                    edge.carrier,
                    support.index()
                );
            }
            assert!(!matched.contains(&found), "two predictions on one edge");
            matched.push(found);
        }
    }
}

#[test]
fn gecko_group_is_recognized_from_every_face_that_carries_or_is_a_blend() {
    let (topo, solid) = gecko();
    let faces = solid_faces(&topo, solid).unwrap();
    assert_eq!(faces.len(), 15);
    let boss = faces
        .iter()
        .copied()
        .find(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c) if (c.radius() - 6.0).abs() < 1e-9)
        })
        .unwrap();
    let cluster = recognize_blend_cluster(&topo, solid, boss).unwrap();
    assert!(!cluster.seed_is_member);
    assert_eq!(cluster.members.len(), 7);
    assert_eq!(cluster.faces().len(), 7);
    assert!(!cluster.is_exact());

    let count = |kind, form: fn(&BlendMemberForm) -> bool, convexity| {
        cluster
            .members
            .iter()
            .filter(|m| m.kind == kind && form(&m.form) && m.convexity == convexity)
            .count()
    };
    let cylinder = |form: &BlendMemberForm| *form == BlendMemberForm::Cylinder;
    let torus = |form: &BlendMemberForm| *form == BlendMemberForm::Torus;
    let sampled = |form: &BlendMemberForm| matches!(form, BlendMemberForm::SampledSections { .. });
    // Two concave vertical seams, the concave shoulder torus, two convex
    // top edges and the two concave sampled corners between them.
    assert_eq!(
        count(BlendMemberKind::Band, cylinder, Convexity::Concave),
        2
    );
    assert_eq!(count(BlendMemberKind::Band, torus, Convexity::Concave), 1);
    assert_eq!(count(BlendMemberKind::Band, cylinder, Convexity::Convex), 2);
    assert_eq!(
        count(BlendMemberKind::CornerPatch, sampled, Convexity::Concave),
        2
    );
    for member in &cluster.members {
        assert!((member.radius - 4.0).abs() < 1e-9, "{member:?}");
    }

    // Each corner rolls on the boss and on one convex top band; it collapses
    // to the boss/top/wall corner of the sharp part.
    for member in &cluster.members {
        if member.kind != BlendMemberKind::CornerPatch {
            assert_eq!(member.root_supports.len(), 2);
            continue;
        }
        let BlendMemberForm::SampledSections {
            stations,
            deviation,
            angle_tolerance,
        } = member.form
        else {
            unreachable!()
        };
        assert!(stations >= 513);
        assert!(deviation < 1e-4, "measured deviation {deviation}");
        assert!(member.contact_angle < angle_tolerance);
        assert_eq!(member.root_supports, vec![boss]);
        assert_eq!(member.supports.len(), 2);
        let band = member
            .supports
            .iter()
            .find_map(|support| cluster.member_of(*support))
            .unwrap();
        assert_eq!(cluster.members[band].convexity, Convexity::Convex);
        let MemberSharp::Vertex(vertex) = member.sharp else {
            panic!("a corner collapses to a vertex")
        };
        let point = cluster.sharp_vertices[vertex].point;
        assert!(
            (point - Point3::new(6.0, 0.0, 24.0)).length() < 1e-9
                || (point - Point3::new(0.0, 6.0, 24.0)).length() < 1e-9,
            "{point:?}"
        );
        // The sampled cross sections are not G1 to the exact query's 1e-10
        // gate: their tangency was measured against the station spacing.
        let face = member.faces[0];
        let adjacency = topo.build_adjacency(solid).unwrap();
        let rough = face_edges(&topo, face)
            .unwrap()
            .into_iter()
            .filter(|&edge| {
                let other = adjacency
                    .faces_for_edge(edge)
                    .iter()
                    .copied()
                    .find(|f| *f != face)
                    .unwrap();
                !edge_is_g1(&topo, edge, face, other).unwrap()
            })
            .count();
        assert!(rough > 0);
    }
    // The exact per-band recognizer gives a sampled corner no radius, so it
    // can never join an exact removal set: the gap this recognizer closes.
    let corner = cluster
        .members
        .iter()
        .find(|m| m.kind == BlendMemberKind::CornerPatch)
        .unwrap()
        .faces[0];
    assert!(matches!(
        remus_operations::resize_blend::blend_region(&topo, solid, corner),
        Err(OperationsError::ResizeBlend(
            remus_operations::resize_blend::ResizeBlendError::BandNotAnalytic { .. }
        ))
    ));
    assert_eq!(cluster.sharp_edges.len(), 5);
    assert_eq!(cluster.sharp_vertices.len(), 6);

    // Concave runs first, then the convex top edges with the corners they
    // ride into.
    assert_eq!(cluster.reblend_order.len(), 2);
    let [concave, convex] = [&cluster.reblend_order[0], &cluster.reblend_order[1]];
    assert_eq!(concave.convexity, Convexity::Concave);
    assert_eq!(concave.sharp_edges.len(), 3);
    assert_eq!(convex.convexity, Convexity::Convex);
    assert_eq!(convex.sharp_edges.len(), 2);
    assert_eq!(convex.members.len(), 4);

    // Every face that is a blend or that a blend rolls on names the same
    // group; the far end faces, the cap and the bottom name none.
    let mut supported = 0;
    for &seed in &faces {
        match recognize_blend_cluster(&topo, solid, seed) {
            Ok(other) => {
                supported += 1;
                assert_eq!(other.members, cluster.members);
                assert_eq!(other.sharp_edges, cluster.sharp_edges);
                assert_eq!(other.sharp_vertices, cluster.sharp_vertices);
                assert_eq!(other.reblend_order, cluster.reblend_order);
                assert_eq!(other.seed_is_member, cluster.member_of(seed).is_some());
            }
            Err(error) => {
                assert!(matches!(
                    refusal(Err(error)),
                    BlendClusterError::NoDependentBlends { .. }
                ));
            }
        }
    }
    // 7 members + boss + top + the two walls the boss stands out of.
    assert_eq!(supported, 11);
    assert_eq!(cluster.supports.len(), 4);
    assert_eq!(cluster.boundary_faces.len(), 3);
}

#[test]
fn gecko_predicted_sharp_edges_are_the_sharp_parents() {
    let (topo, solid) = gecko();
    let top = plane(&topo, solid, Vec3::new(0.0, 0.0, 1.0), 24.0);
    let cluster = recognize_blend_cluster(&topo, solid, top).unwrap();
    let (sharp_topo, sharp) = gecko_sharp();
    assert_eq!(
        remus_topology::explorer::solid_entity_counts(&sharp_topo, sharp).unwrap(),
        (8, 19, 13)
    );
    assert_sharp_matches(&cluster, &topo, &sharp_topo, sharp);
    // Known radii: the parent's five R4 edges.
    for edge in &cluster.sharp_edges {
        assert!((cluster.members[edge.member].radius - 4.0).abs() < 1e-9);
    }
    // The torus band restores the boss/top quarter circle.
    let circle = cluster
        .sharp_edges
        .iter()
        .find_map(|edge| match edge.carrier {
            SharpCarrier::Circle { radius, center, .. } => Some((radius, center, edge.midpoint)),
            SharpCarrier::Line { .. } => None,
        })
        .unwrap();
    assert!((circle.0 - 6.0).abs() < 1e-12);
    assert!((circle.1 - Point3::new(0.0, 0.0, 24.0)).length() < 1e-12);
    let half = 6.0 * std::f64::consts::FRAC_1_SQRT_2;
    assert!((circle.2 - Point3::new(half, half, 24.0)).length() < 1e-9);
}

fn top_at(topo: &Topology, solid: SolidId, height: f64) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.dot(Vec3::new(0.0, 0.0, 1.0)) > 1.0 - 1e-9)
                && face_vertices(topo, face)
                    .unwrap()
                    .into_iter()
                    .all(|vertex| (topo.vertex(vertex).unwrap().point().z() - height).abs() < 1e-7)
        })
        .unwrap()
}

struct Shoulder {
    topo: Topology,
    blank: SolidId,
    rounded: SolidId,
    solid: SolidId,
}

/// The native Gecko twin of `regress_move_face_shoulder_clearance.rs`'s
/// `shoulder_at`: R1.8 cap rim, then the five boss/block edges at `radius`.
fn shoulder_at(height: f64, cap_height: f64, radius: f64) -> Shoulder {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 30.0, 18.0, height).unwrap();
    let boss = make_cylinder(&mut topo, 6.0, cap_height).unwrap();
    let blank = boolean(&mut topo, BooleanOp::Fuse, boss, block).unwrap();
    let cap = top_at(&topo, blank, cap_height);
    let rim = face_edges(&topo, cap).unwrap();
    let rounded = fillet_cascade(&mut topo, blank, &rim, 1.8).unwrap().solid;
    let perimeter: Vec<_> = solid_edges(&topo, rounded)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let edge = topo.edge(id).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            let at = |a: f64, b: f64| (a - b).abs() < 1e-7;
            let top = at(a.z(), height) && at(b.z(), height);
            match edge.curve() {
                EdgeCurve::Circle(_) => top,
                EdgeCurve::Line => {
                    (top && ((at(a.x(), 0.0) && at(b.x(), 0.0))
                        || (at(a.y(), 0.0) && at(b.y(), 0.0))))
                        || ((a.z() - b.z()).abs() > height - 0.01
                            && ((at(a.x(), 6.0)
                                && at(b.x(), 6.0)
                                && at(a.y(), 0.0)
                                && at(b.y(), 0.0))
                                || (at(a.y(), 6.0)
                                    && at(b.y(), 6.0)
                                    && at(a.x(), 0.0)
                                    && at(b.x(), 0.0))))
                }
                _ => false,
            }
        })
        .collect();
    assert_eq!(perimeter.len(), 5);
    let solid = fillet_cascade(&mut topo, rounded, &perimeter, radius)
        .unwrap()
        .solid;
    Shoulder {
        topo,
        blank,
        rounded,
        solid,
    }
}

#[test]
fn shoulder_groups_at_r4_and_r5_match_their_sharp_stages() {
    for (height, cap, radius) in [
        (26.0, 35.0, 4.0),
        (26.0, 35.0, 5.0),
        (19.1, 28.5, 4.0),
        (19.1, 28.5, 5.0),
    ] {
        let label = format!("h{height} cap{cap} r{radius}");
        let shoulder = shoulder_at(height, cap, radius);
        let topo = &shoulder.topo;
        let solid = shoulder.solid;
        let top = top_at(topo, solid, height);
        let cap_face = top_at(topo, solid, cap);
        let boss = solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .find(|&face| {
                matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c) if (c.radius() - 6.0).abs() < 1e-9)
            })
            .unwrap();

        // The top edit's group: the five rounds and their two corners. The
        // cap rim rests on the boss and cap only, so it is not dependent.
        let group = recognize_blend_cluster(topo, solid, top).unwrap();
        assert_eq!(group.members.len(), 7, "{label}");
        assert!(
            group
                .members
                .iter()
                .all(|m| (m.radius - radius).abs() < 1e-9),
            "{label}"
        );
        assert_eq!(
            group
                .members
                .iter()
                .filter(|m| m.kind == BlendMemberKind::CornerPatch)
                .count(),
            2,
            "{label}"
        );
        assert_sharp_matches(&group, topo, topo, shoulder.rounded);

        // The boss edit's group also carries the cap rim, rebuilt last.
        let boss_group = recognize_blend_cluster(topo, solid, boss).unwrap();
        assert_eq!(boss_group.members.len(), 8, "{label}");
        assert_sharp_matches(&boss_group, topo, topo, shoulder.blank);
        let last = boss_group.reblend_order.last().unwrap();
        assert_eq!(last.convexity, Convexity::Convex);
        assert!((last.radius - 1.8).abs() < 1e-9, "{label}");
        assert_eq!(
            boss_group.reblend_order[0].convexity,
            Convexity::Concave,
            "{label}"
        );

        // The cap edit's group is the rim alone: an exact closed circle.
        let rim = recognize_blend_cluster(topo, solid, cap_face).unwrap();
        assert_eq!(rim.members.len(), 1, "{label}");
        assert!(rim.is_exact());
        assert_eq!(rim.members[0].form, BlendMemberForm::Torus);
        assert_eq!(rim.members[0].convexity, Convexity::Convex);
        assert_eq!(rim.sharp_edges[0].ends, None);
        assert!(rim.sharp_vertices.is_empty());
        assert_sharp_matches(&rim, topo, topo, shoulder.blank);
    }
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

/// `fillet_into_seam_band.rs`'s body: an R2.6 seam round on the boss, then
/// (optionally) the block's top edges rounded into it by the ridge walk.
///
/// The walk's transition into the seam is a circular-section carrier whose
/// sections are not the rolling ball's characteristic circles (its stations'
/// contacts do not share one ball), so it is no sampled corner. It continues
/// its exact top band at the band's radius between the same two supports, so
/// it joins that band: one member, one sharp edge, its measured deviation
/// reported rather than bounded.
#[test]
fn seam_band_is_recognized_and_ridge_walked_bands_join_their_bands() {
    let mut topo = Topology::new();
    let boss = make_cylinder(&mut topo, 6.0, 28.0).unwrap();
    let block = make_box(&mut topo, 30.0, 18.0, 18.0).unwrap();
    let body = boolean(&mut topo, BooleanOp::Fuse, boss, block).unwrap();
    let p = Point3::new;
    let seam = [
        find_edge(&topo, body, p(6.0, 0.0, 0.0), p(6.0, 0.0, 18.0)),
        find_edge(&topo, body, p(6.0, 0.0, 18.0), p(0.0, 6.0, 18.0)),
        find_edge(&topo, body, p(0.0, 6.0, 18.0), p(0.0, 6.0, 0.0)),
    ];
    let filleted = fillet_cascade(&mut topo, body, &seam, 2.6).unwrap().solid;
    let boss_face = solid_faces(&topo, filleted)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c) if (c.radius() - 6.0).abs() < 1e-9)
        })
        .unwrap();
    let group = recognize_blend_cluster(&topo, filleted, boss_face).unwrap();
    assert_eq!(group.members.len(), 3);
    assert!(group.is_exact());
    assert!(
        group
            .members
            .iter()
            .all(|m| m.kind == BlendMemberKind::Band && m.convexity == Convexity::Concave)
    );
    assert_eq!(group.reblend_order.len(), 1);
    // The seam torus overhangs both walls; its sharp arc is still the
    // in-block quarter circle.
    assert_sharp_matches(&group, &topo, &topo, body);

    let front = find_edge(&topo, filleted, p(8.6, 0.0, 18.0), p(30.0, 0.0, 18.0));
    let side = find_edge(&topo, filleted, p(0.0, 8.6, 18.0), p(0.0, 18.0, 18.0));
    for size in [1.0, 2.0] {
        let mut ridge = topo.clone();
        let rounded = fillet_cascade(&mut ridge, filleted, &[front, side], size)
            .unwrap()
            .solid;
        let walked: Vec<FaceId> = solid_faces(&ridge, rounded)
            .unwrap()
            .into_iter()
            .filter(|&f| matches!(ridge.face(f).unwrap().surface(), FaceSurface::Nurbs(_)))
            .collect();
        assert_eq!(walked.len(), 2);
        // The same group from the boss and from either walked face: the
        // three seam faces and the two top bands, each with its transition.
        for seed in [boss_face, walked[0], walked[1]] {
            let group = recognize_blend_cluster(&ridge, rounded, seed).unwrap();
            assert_eq!(group.members.len(), 5, "size {size}");
            assert!(!group.is_exact(), "size {size}");
            for &face in &walked {
                let member = &group.members[group.member_of(face).unwrap()];
                assert_eq!(member.kind, BlendMemberKind::Band, "size {size}");
                assert_eq!(member.convexity, Convexity::Convex, "size {size}");
                assert_eq!(member.faces.len(), 2, "size {size}: band and transition");
                assert!((member.radius - size).abs() < 1e-9, "size {size}");
                let BlendMemberForm::SampledSections { deviation, .. } = member.form else {
                    panic!("size {size}: the walked member reports its sampled form");
                };
                assert!(deviation.is_finite() && deviation > 0.0, "size {size}");
            }
            // Its sharp edges and vertices are the unfilleted body's.
            assert_sharp_matches(&group, &ridge, &ridge, body);
        }
    }
}

#[test]
fn bored_plate_bore_is_not_a_blend_but_its_rim_round_is() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 40.0, 30.0, 10.0).unwrap();
    let tool = make_cylinder(&mut topo, 5.0, 30.0).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        tool,
        &remus_math::mat::Mat4::translation(20.0, 15.0, -10.0),
    )
    .unwrap();
    let plate = boolean(&mut topo, BooleanOp::Cut, block, tool).unwrap();
    for seed in solid_faces(&topo, plate).unwrap() {
        assert!(matches!(
            refusal(recognize_blend_cluster(&topo, plate, seed)),
            BlendClusterError::NoDependentBlends { .. }
        ));
    }

    let top = plane(&topo, plate, Vec3::new(0.0, 0.0, 1.0), 10.0);
    let rim: Vec<EdgeId> = face_edges(&topo, top)
        .unwrap()
        .into_iter()
        .filter(|&edge| matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Circle(_)))
        .collect();
    assert!(!rim.is_empty());
    let rounded = fillet_cascade(&mut topo, plate, &rim, 1.0).unwrap().solid;
    let bore = solid_faces(&topo, rounded)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let group = recognize_blend_cluster(&topo, rounded, bore).unwrap();
    assert_eq!(group.members.len(), 1);
    assert_eq!(group.members[0].form, BlendMemberForm::Torus);
    assert_eq!(group.members[0].convexity, Convexity::Convex);
    assert!((group.members[0].radius - 1.0).abs() < 1e-12);
    assert_sharp_matches(&group, &topo, &topo, plate);
    let bottom = plane(&topo, rounded, Vec3::new(0.0, 0.0, -1.0), 0.0);
    assert!(matches!(
        refusal(recognize_blend_cluster(&topo, rounded, bottom)),
        BlendClusterError::NoDependentBlends { .. }
    ));
}

#[test]
fn freeform_faces_are_refused_not_guessed() {
    // A variable-radius round: freeform, tangent to both walls, outside the
    // constant circular-section family.
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edge = solid_edges(&topo, solid).unwrap()[0];
    let rounded = fillet_variable(
        &mut topo,
        solid,
        &[(
            edge,
            FilletRadiusLaw::Linear {
                start: 1.0,
                end: 1.5,
            },
        )],
    )
    .unwrap();
    let band = solid_faces(&topo, rounded)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Nurbs(_)))
        .unwrap();
    let mut tangent_refusals = 0;
    for seed in solid_faces(&topo, rounded).unwrap() {
        match refusal(recognize_blend_cluster(&topo, rounded, seed)) {
            BlendClusterError::UnrecognizedMember { face, .. } => {
                assert_eq!(face, band.index());
                tangent_refusals += 1;
            }
            BlendClusterError::NoDependentBlends { .. } => {}
            other => panic!("{other:?}"),
        }
    }
    // The band and the two walls it rounds.
    assert_eq!(tangent_refusals, 3);

    // A generic all-NURBS body: no face has a closed-form support, so no
    // group is ever claimed.
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edge = solid_edges(&topo, solid).unwrap()[0];
    let rounded = fillet_cascade(&mut topo, solid, &[edge], 2.0)
        .unwrap()
        .solid;
    convert_to_bspline(&mut topo, rounded).unwrap();
    for seed in solid_faces(&topo, rounded).unwrap() {
        assert!(matches!(
            refusal(recognize_blend_cluster(&topo, rounded, seed)),
            BlendClusterError::UnrecognizedMember { .. }
                | BlendClusterError::NoDependentBlends { .. }
        ));
    }
}

#[test]
fn foreign_seed_is_invalid() {
    let (mut topo, solid) = gecko();
    let foreign = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let face = solid_faces(&topo, foreign).unwrap()[0];
    assert!(matches!(
        refusal(recognize_blend_cluster(&topo, solid, face)),
        BlendClusterError::InvalidSeed { .. }
    ));
}
