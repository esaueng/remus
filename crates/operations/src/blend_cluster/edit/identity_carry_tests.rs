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

/// A face of a prism: its top, its bottom or side `k`.
#[derive(Clone, Copy, Debug)]
enum F {
    Top,
    Bottom,
    Side(usize),
}

struct Prism {
    solid: SolidId,
    top: FaceId,
    bottom: FaceId,
    sides: Vec<FaceId>,
}

impl Prism {
    fn new(topo: &mut Topology) -> Self {
        let (solid, top, bottom, sides) = prism(topo);
        Self {
            solid,
            top,
            bottom,
            sides,
        }
    }

    fn at(&self, face: F) -> FaceId {
        match face {
            F::Top => self.top,
            F::Bottom => self.bottom,
            F::Side(k) => self.sides[k],
        }
    }
}

/// Carry the identities of `members` (each a band: its faces and its
/// supports on the source prism) given the face map `images` (source face
/// -> result face).
fn carry_with(members: &[(&[F], &[F])], images: &[(F, F)]) -> (EvolutionMap, Prism, Prism) {
    let mut topo = Topology::new();
    let source = Prism::new(&mut topo);
    let result = Prism::new(&mut topo);
    let at = |faces: &[F]| faces.iter().map(|&f| source.at(f)).collect::<Vec<_>>();
    let cluster = BlendCluster {
        seed: source.at(members[0].0[0]),
        seed_is_member: true,
        members: members
            .iter()
            .map(|(faces, supports)| band(at(faces), at(supports)))
            .collect(),
        supports: Vec::new(),
        boundary_faces: Vec::new(),
        sharp_edges: Vec::new(),
        sharp_vertices: Vec::new(),
        reblend_order: Vec::new(),
    };
    let mut map = EvolutionMap::new();
    for &(from, to) in images {
        map.add_modified(source.at(from).index(), result.at(to).index());
    }
    carry_member_identities(&topo, &cluster, source.solid, result.solid, &mut map).unwrap();
    (map, source, result)
}

/// The band `[s(2), s(5)]` rolls on top and bottom, whose images `r(0)` and
/// `r(3)` no unclaimed result face touches both of: the support key reads
/// empty. Only `r(5)` touches both images of the blends `s(2)` meets
/// (`s(1)` -> `r(4)`, `s(3)` -> top), but `r(1)` and `r(2)` also touch the
/// images of its supports or blends, so nothing proves `s(2)` stayed whole:
/// a split of it may be any of them. With `rivals` false they are claimed
/// first, leaving `r(5)` the only face that could carry `s(2)`.
fn empty_support_key(rivals: bool) -> (EvolutionMap, Prism, Prism) {
    let mut images = vec![
        (F::Top, F::Side(0)),
        (F::Bottom, F::Side(3)),
        (F::Side(1), F::Side(4)),
        (F::Side(3), F::Top),
        (F::Side(0), F::Bottom),
    ];
    if !rivals {
        images.extend([(F::Side(4), F::Side(1)), (F::Side(5), F::Side(2))]);
    }
    carry_with(
        &[
            (&[F::Side(1)], &[F::Top, F::Bottom]),
            (&[F::Side(2), F::Side(5)], &[F::Top, F::Bottom]),
            (&[F::Side(3)], &[F::Top, F::Bottom]),
        ],
        &images,
    )
}

#[test]
fn an_empty_support_match_binds_by_touched_blends_only_when_nothing_else_could_be_the_face() {
    let (map, source, _) = empty_support_key(true);
    assert_eq!(
        map.modified.get(&source.at(F::Side(2)).index()),
        None,
        "{map:?}"
    );
}

/// Control: with the rivals claimed, the touched blends bind `s(2)` to
/// `r(5)`.
#[test]
fn an_empty_support_match_binds_by_touched_blends_when_the_face_is_alone() {
    let (map, source, result) = empty_support_key(false);
    assert_eq!(
        map.modified.get(&source.at(F::Side(2)).index()),
        Some(&vec![result.at(F::Side(5)).index()]),
        "{map:?}"
    );
}

/// `s(2)` rolls on top and bottom; the unclaimed result sides touching both
/// images are `r(2)` and `r(5)`: ambiguous in the first round. In that
/// round `s(5)` (rolling on `s(4)` and `s(0)`) claims `r(5)`. The ambiguity
/// `s(2)` met stays: a later round must not bind it to `r(2)`.
#[test]
fn an_ambiguity_found_in_one_round_is_not_erased_by_a_later_claim() {
    let (map, source, result) = carry_with(
        &[
            (&[F::Side(2)], &[F::Top, F::Bottom]),
            (&[F::Side(5)], &[F::Side(4), F::Side(0)]),
        ],
        &[
            (F::Top, F::Top),
            (F::Bottom, F::Bottom),
            (F::Side(0), F::Side(0)),
            (F::Side(1), F::Side(1)),
            (F::Side(3), F::Side(3)),
            (F::Side(4), F::Side(4)),
        ],
    );
    assert_eq!(
        map.modified.get(&source.at(F::Side(5)).index()),
        Some(&vec![result.at(F::Side(5)).index()]),
        "{map:?}"
    );
    assert_eq!(
        map.modified.get(&source.at(F::Side(2)).index()),
        None,
        "{map:?}"
    );
}
