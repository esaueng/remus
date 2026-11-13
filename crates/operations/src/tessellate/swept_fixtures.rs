//! Freshly generated swept bodies (threads, an in-place helical sweep, a
//! helical gear) and a few ordinary curved ones, for the tessellator's
//! bit-identity oracles.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::make_circle_edge;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::wire::{OrientedEdge, Wire};

use crate::boolean::{BooleanOp, boolean};
use crate::gear::{GearKind, GearParams, GearPlacement, Hand, make_gear};
use crate::tessellate::TriangleMesh;
use crate::thread::{CylinderThreadOptions, ThreadForm, ThreadKind, cut_thread};

fn cylinder_face(topo: &Topology, solid: SolidId, radius: f64) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(topo.face(face).unwrap().surface(),
                FaceSurface::Cylinder(c) if (c.radius() - radius).abs() < 1e-9)
        })
        .unwrap()
}

/// ISO M6×1 external thread over a boss `turns` tall.
pub(super) fn external_thread(topo: &mut Topology, turns: f64) -> SolidId {
    let blank = crate::primitives::make_cylinder(topo, 3.0, turns).unwrap();
    let face = cylinder_face(topo, blank, 3.0);
    let form = ThreadForm::iso_metric(1.0, ThreadKind::External);
    cut_thread(topo, blank, face, &form, &CylinderThreadOptions::default())
        .unwrap()
        .solid
}

/// A round wire swept in place along a helix (one seamed NURBS side).
pub(super) fn coil(topo: &mut Topology, turns: f64) -> SolidId {
    let (center, normal) = (Point3::new(12.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
    let edge = make_circle_edge(topo, center, normal, 1.5, 1e-7).unwrap();
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let profile = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal,
            d: crate::dot_normal_point(normal, center),
        },
    ));
    crate::helix::helical_sweep_in_place(
        topo,
        profile,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        5.0,
        turns,
    )
    .unwrap()
}

/// A 12-tooth helical gear (twisted NURBS flanks).
pub(super) fn helical_gear(topo: &mut Topology) -> SolidId {
    let params = GearParams {
        helix_angle: 20.0_f64.to_radians(),
        hand: Hand::Right,
        ..GearParams::standard(GearKind::External, 2.0, 12, 6.0)
    };
    make_gear(topo, &params, &GearPlacement::default())
        .unwrap()
        .solid
}

/// A shaft drilled across its axis: the wall's trims are saddle curves.
pub(super) fn cross_drilled_shaft(topo: &mut Topology) -> SolidId {
    let shaft = crate::primitives::make_cylinder(topo, 5.0, 20.0).unwrap();
    let drill = crate::primitives::make_cylinder(topo, 2.0, 30.0).unwrap();
    let across = Mat4::translation(0.0, 0.0, 10.0)
        * Mat4::rotation_x(std::f64::consts::FRAC_PI_2)
        * Mat4::translation(0.0, 0.0, -15.0);
    crate::transform::transform_solid(topo, drill, &across).unwrap();
    boolean(topo, BooleanOp::Cut, shaft, drill).unwrap()
}

/// Ordinary curved bodies: a sphere, a torus and an all-edges filleted box.
pub(super) fn ordinary(topo: &mut Topology) -> Vec<SolidId> {
    let sharp = crate::primitives::make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let edges = remus_topology::explorer::solid_edges(topo, sharp).unwrap();
    vec![
        crate::primitives::make_sphere(topo, 10.0, 16).unwrap(),
        crate::primitives::make_torus(topo, 10.0, 3.0, 16).unwrap(),
        crate::blend_ops::fillet_v2(topo, sharp, &edges, 1.5)
            .unwrap()
            .solid,
    ]
}

/// Every position, normal and index bit of both meshes agrees.
pub(super) fn assert_same_bits(a: &TriangleMesh, b: &TriangleMesh, what: &str) {
    assert_eq!(a.indices, b.indices, "{what}: indices");
    assert_eq!(a.positions.len(), b.positions.len(), "{what}: vertices");
    let bits = |p: [f64; 3]| p.map(f64::to_bits);
    for (i, (p, q)) in a.positions.iter().zip(&b.positions).enumerate() {
        assert_eq!(
            bits([p.x(), p.y(), p.z()]),
            bits([q.x(), q.y(), q.z()]),
            "{what}: position {i}"
        );
    }
    for (i, (p, q)) in a.normals.iter().zip(&b.normals).enumerate() {
        assert_eq!(
            bits([p.x(), p.y(), p.z()]),
            bits([q.x(), q.y(), q.z()]),
            "{what}: normal {i}"
        );
    }
}
