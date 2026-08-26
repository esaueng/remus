//! Ground truth for `remus_heal::upgrade::shell_sewing`.
//!
//! The heal crate sits below `remus-check`, so its own tests can only prove
//! the sewn shell is topologically closed. Here, one layer up, the sewn shell
//! is wrapped in a solid and measured: a shell that is merely *reported* sewn
//! does not produce a valid unit-volume cube.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_heal::upgrade::shell_sewing::{SewReport, sew_shell_report};
use remus_math::vec::{Point3, Vec3};
use remus_operations::measure::{solid_surface_area, solid_volume};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::{Shell, ShellId};
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const TOL: f64 = 1e-7;

fn quad_face(topo: &mut Topology, pts: [Point3; 4], normal: Vec3, d: f64) -> FaceId {
    let vs: Vec<_> = pts
        .iter()
        .map(|p| topo.add_vertex(Vertex::new(*p, TOL)))
        .collect();
    let es: Vec<_> = (0..4)
        .map(|i| topo.add_edge(Edge::new(vs[i], vs[(i + 1) % 4], EdgeCurve::Line)))
        .collect();
    let wire = Wire::new(
        es.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
        true,
    )
    .unwrap();
    let wid = topo.add_wire(wire);
    topo.add_face(Face::new(wid, vec![], FaceSurface::Plane { normal, d }))
}

/// Six outward-oriented faces of the unit cube, each built from its own
/// vertices and edges — nothing is shared, so all 24 edges are free.
fn disjoint_cube_shell(topo: &mut Topology) -> ShellId {
    let p = Point3::new;
    let faces = vec![
        quad_face(
            topo,
            [
                p(0.0, 0.0, 0.0),
                p(0.0, 1.0, 0.0),
                p(1.0, 1.0, 0.0),
                p(1.0, 0.0, 0.0),
            ],
            Vec3::new(0.0, 0.0, -1.0),
            0.0,
        ),
        quad_face(
            topo,
            [
                p(0.0, 0.0, 1.0),
                p(1.0, 0.0, 1.0),
                p(1.0, 1.0, 1.0),
                p(0.0, 1.0, 1.0),
            ],
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
        ),
        quad_face(
            topo,
            [
                p(0.0, 0.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, 0.0, 1.0),
                p(0.0, 0.0, 1.0),
            ],
            Vec3::new(0.0, -1.0, 0.0),
            0.0,
        ),
        quad_face(
            topo,
            [
                p(0.0, 1.0, 0.0),
                p(0.0, 1.0, 1.0),
                p(1.0, 1.0, 1.0),
                p(1.0, 1.0, 0.0),
            ],
            Vec3::new(0.0, 1.0, 0.0),
            1.0,
        ),
        quad_face(
            topo,
            [
                p(0.0, 0.0, 0.0),
                p(0.0, 0.0, 1.0),
                p(0.0, 1.0, 1.0),
                p(0.0, 1.0, 0.0),
            ],
            Vec3::new(-1.0, 0.0, 0.0),
            0.0,
        ),
        quad_face(
            topo,
            [
                p(1.0, 0.0, 0.0),
                p(1.0, 1.0, 0.0),
                p(1.0, 1.0, 1.0),
                p(1.0, 0.0, 1.0),
            ],
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
        ),
    ];
    topo.add_shell(Shell::new(faces).unwrap())
}

fn as_solid(topo: &mut Topology, shell_id: ShellId) -> SolidId {
    topo.add_solid(Solid::new(shell_id, vec![]))
}

#[test]
fn sewn_disjoint_cube_measures_as_a_unit_cube() {
    let mut topo = Topology::new();
    let shell_id = disjoint_cube_shell(&mut topo);

    let report = sew_shell_report(&mut topo, shell_id, 1e-6).unwrap();
    assert_eq!(
        report,
        SewReport {
            sewn: 12,
            declined: 0
        }
    );

    let solid = as_solid(&mut topo, shell_id);

    let volume = solid_volume(&topo, solid, 1e-4).unwrap();
    assert!(
        (volume - 1.0).abs() < 1e-9,
        "sewn cube volume {volume} != 1.0"
    );

    let area = solid_surface_area(&topo, solid, 1e-4).unwrap();
    assert!((area - 6.0).abs() < 1e-9, "sewn cube area {area} != 6.0");

    let report = validate_solid(&topo, solid).unwrap();
    assert!(
        report.is_valid(),
        "sewn cube fails validation: {:?}",
        report.issues
    );
}

#[test]
fn unsewn_disjoint_cube_does_not_validate() {
    // The control: without sewing, the same six faces are a pile of
    // unconnected patches. If this ever passed, the test above would prove
    // nothing about sewing.
    let mut topo = Topology::new();
    let shell_id = disjoint_cube_shell(&mut topo);
    let solid = as_solid(&mut topo, shell_id);

    let report = validate_solid(&topo, solid).unwrap();
    assert!(
        !report.is_valid(),
        "an unsewn shell of 24 free edges must not validate"
    );
}
