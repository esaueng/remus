//! Oracles for [`super::carry_member_identities`] on a band of several
//! faces: a face whose supports match several result faces keeps that
//! ambiguity, and no later key (the blends it touches) may narrow it to one
//! of them.
//!
//! The topology is synthetic: two hexagonal prisms in one arena, the first
//! read as the source, the second as the result. Only adjacency, carrier
//! kind and the face map are read, so plane faces stand in for blends.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::compact_sort::CompactSort;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use super::carry_member_identities;
use crate::blend_cluster::{
    BlendCluster, BlendMember, BlendMemberForm, BlendMemberKind, Convexity, MemberSharp,
};
use crate::evolution::EvolutionMap;

/// A unit hexagonal prism: (solid, top, bottom, sides in angular order).
fn prism(topo: &mut Topology) -> (SolidId, FaceId, FaceId, Vec<FaceId>) {
    let points: Vec<Point3> = (0..6)
        .map(|k| {
            let (sin, cos) = (f64::from(k) * std::f64::consts::FRAC_PI_3).sin_cos();
            Point3::new(cos, sin, 0.0)
        })
        .collect();
    let base = remus_topology::builder::make_planar_face(topo, &points, 1e-7).unwrap();
    let solid = crate::extrude::extrude(topo, base, Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let mut top = None;
    let mut bottom = None;
    let mut sides = Vec::new();
    for face in solid_faces(topo, solid).unwrap() {
        let &FaceSurface::Plane { normal, .. } = topo.face(face).unwrap().surface() else {
            panic!("a prism face is a plane");
        };
        let outward = if topo.face(face).unwrap().is_reversed() {
            -normal
        } else {
            normal
        };
        if outward.z() > 0.5 {
            top = Some(face);
        } else if outward.z() < -0.5 {
            bottom = Some(face);
        } else {
            sides.push((outward.y().atan2(outward.x()), face));
        }
    }
    sides.compact_sort_by(|a, b| a.0.total_cmp(&b.0));
    assert_eq!(sides.len(), 6);
    (
        solid,
        top.unwrap(),
        bottom.unwrap(),
        sides.into_iter().map(|(_, face)| face).collect(),
    )
}

fn band(faces: Vec<FaceId>, supports: Vec<FaceId>) -> BlendMember {
    let mut faces = faces;
    faces.compact_sort_unstable_by_key(|face| face.index());
    let mut supports = supports;
    supports.compact_sort_unstable_by_key(|face| face.index());
    BlendMember {
        faces,
        kind: BlendMemberKind::Band,
        form: BlendMemberForm::Plane,
        radius: 1.0,
        convexity: Convexity::Convex,
        root_supports: supports.clone(),
        supports,
        contact_angle: 0.0,
        sharp: MemberSharp::Edge(0),
    }
}

/// The source band `[s(2), s(5)]` rolls on top and bottom; `s(1)` and
/// `s(3)` are two other blends `s(2)` meets. `claimed` result sides are
/// already images of unrelated faces.
fn carry(claimed: &[usize]) -> (EvolutionMap, Vec<FaceId>, Vec<FaceId>) {
    let mut topo = Topology::new();
    let (source, top, bottom, s) = prism(&mut topo);
    let (result, top_r, bottom_r, r) = prism(&mut topo);
    let cluster = BlendCluster {
        seed: s[2],
        seed_is_member: true,
        members: vec![
            band(vec![s[1]], vec![top, bottom]),
            band(vec![s[2], s[5]], vec![top, bottom]),
            band(vec![s[3]], vec![top, bottom]),
        ],
        supports: vec![top, bottom],
        boundary_faces: Vec::new(),
        sharp_edges: Vec::new(),
        sharp_vertices: Vec::new(),
        reblend_order: Vec::new(),
    };
    let mut map = EvolutionMap::new();
    for (from, to) in [(top, top_r), (bottom, bottom_r), (s[1], r[1]), (s[3], r[3])] {
        map.add_modified(from.index(), to.index());
    }
    // Unrelated source faces (here the source's own sides) claim the
    // listed result sides.
    for &side in claimed {
        map.add_modified(s[side].index(), r[side].index());
    }
    carry_member_identities(&topo, &cluster, source, result, &mut map).unwrap();
    (map, s, r)
}

/// Four result sides touch the images of both supports, so the support
/// match is ambiguous. Only `r(2)` also touches the images of both blends
/// `s(2)` meets: the fallback key must not turn that into a one-to-one
/// binding of `s(2)`.
#[test]
fn an_ambiguous_support_match_is_never_narrowed_by_touched_blends() {
    let (map, s, _) = carry(&[]);
    assert_eq!(map.modified.get(&s[2].index()), None, "{map:?}");
    assert_eq!(map.modified.get(&s[5].index()), None, "{map:?}");
}

/// Control: with the other sides claimed, the support match is unique and
/// binds `s(2)` to `r(2)`.
#[test]
fn a_unique_support_match_binds() {
    let (map, s, r) = carry(&[0, 4, 5]);
    assert_eq!(
        map.modified.get(&s[2].index()),
        Some(&vec![r[2].index()]),
        "{map:?}"
    );
}
