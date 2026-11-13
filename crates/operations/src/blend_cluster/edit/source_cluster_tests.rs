//! Differential oracle for [`super::source_cluster`]: the group a route
//! recognized on its private copy, carried back to the caller's solid,
//! against recognizing the caller's solid again (what [`super::copy_out`]
//! did before), on the filleted Gentle-Gecko part's boss resizes.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeSet;

use remus_math::vec::Vec3;
use remus_topology::Topology;
use remus_topology::explorer::{solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use super::{RECOGNIZE_IN_COPY_OUT, cylinder_group, source_cluster};
use crate::blend_cluster::{BlendCluster, recognize_blend_cluster};
use crate::journal_ops::resize_cylindrical_face_journaled;

fn gecko() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solid = remus_io::step::reader::read_step(
        include_str!("../../../tests/data/gentle_gecko_filleted.step"),
        &mut topo,
    )
    .unwrap()[0];
    (topo, solid)
}

fn boss_of(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
                if c.axis().cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9
                    && c.origin().x().abs() < 1e-9
                    && c.origin().y().abs() < 1e-9)
        })
        .unwrap()
}

/// What identity carrying reads of a group: each member's faces, kind and
/// supports, and the group's supports and boundary faces, as sets.
fn read_as_sets(cluster: &BlendCluster) -> String {
    let set = |faces: &[FaceId]| {
        faces
            .iter()
            .map(|face| face.index())
            .collect::<BTreeSet<_>>()
    };
    let members: BTreeSet<String> = cluster
        .members
        .iter()
        .map(|member| {
            format!(
                "{:?} {:?} {:?} {:?} {:?} {}",
                set(&member.faces),
                member.kind,
                set(&member.supports),
                set(&member.root_supports),
                member.convexity,
                member.radius.to_bits()
            )
        })
        .collect();
    format!(
        "seed {} member {} {members:?} supports {:?} boundary {:?} edges {} vertices {}",
        cluster.seed.index(),
        cluster.seed_is_member,
        set(&cluster.supports),
        set(&cluster.boundary_faces),
        cluster.sharp_edges.len(),
        cluster.sharp_vertices.len()
    )
}

/// An evolution map with its hash maps in key order.
fn canonical(map: &crate::evolution::EvolutionMap) -> String {
    let modified: std::collections::BTreeMap<_, _> = map.modified.iter().collect();
    let generated: std::collections::BTreeMap<_, _> = map.generated.iter().collect();
    let deleted: BTreeSet<_> = map.deleted.iter().collect();
    format!(
        "modified {modified:?} generated {generated:?} deleted {deleted:?} unresolved {:?} origin {:?}",
        map.unresolved, map.origin
    )
}

/// Restores the hook however a test ends.
struct Recognize;

impl Recognize {
    fn on() -> Self {
        RECOGNIZE_IN_COPY_OUT.with(|flag| flag.set(true));
        Self
    }
}

impl Drop for Recognize {
    fn drop(&mut self) {
        RECOGNIZE_IN_COPY_OUT.with(|flag| flag.set(false));
    }
}

#[test]
fn the_carried_group_is_the_group_recognized_on_the_caller_solid() {
    let (topo, solid) = gecko();
    let boss = boss_of(&topo, solid);
    let (group, _) = cylinder_group(&topo, solid, boss).unwrap().unwrap();
    let carried = source_cluster(&topo, &group).expect("no lifted cap on this part");
    let recognized = recognize_blend_cluster(&topo, solid, boss).unwrap();
    assert_eq!(read_as_sets(&carried), read_as_sets(&recognized));
}

#[test]
fn boss_resizes_record_the_same_history_either_way() {
    let (topo, solid) = gecko();
    let (mut carried_topo, mut carried) = (topo.clone(), solid);
    let (mut recognized_topo, mut recognized) = (topo, solid);
    for radius in [7.3, 8.5, 10.55, 7.35] {
        let boss = boss_of(&carried_topo, carried);
        let a =
            resize_cylindrical_face_journaled(&mut carried_topo, carried, boss, radius).unwrap();
        let b = {
            let _hook = Recognize::on();
            let boss = boss_of(&recognized_topo, recognized);
            resize_cylindrical_face_journaled(&mut recognized_topo, recognized, boss, radius)
                .unwrap()
        };
        assert_eq!(a.solid.index(), b.solid.index(), "radius {radius}");
        assert_eq!(canonical(&a.map), canonical(&b.map), "radius {radius}");
        assert_eq!(a.op, b.op, "radius {radius}");
        assert_eq!(
            format!("{:?}", carried_topo.journal().entries()),
            format!("{:?}", recognized_topo.journal().entries()),
            "radius {radius}"
        );
        assert_eq!(
            format!("{:?}", solid_entity_counts(&carried_topo, a.solid).unwrap()),
            format!(
                "{:?}",
                solid_entity_counts(&recognized_topo, b.solid).unwrap()
            )
        );
        carried = a.solid;
        recognized = b.solid;
    }
}
