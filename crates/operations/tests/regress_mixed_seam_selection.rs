//! The two concave cylinder/block seam edges and their convex top edge must
//! round in one transaction, in every seed order.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::Point3;
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

fn between(t: &Topology, s: SolidId, a: Point3, b: Point3) -> EdgeId {
    solid_edges(t, s)
        .unwrap()
        .into_iter()
        .find(|&e| {
            let e = t.edge(e).unwrap();
            let x = t.vertex(e.start()).unwrap().point();
            let y = t.vertex(e.end()).unwrap().point();
            ((x - a).length() < 1e-6 && (y - b).length() < 1e-6)
                || ((y - a).length() < 1e-6 && (x - b).length() < 1e-6)
        })
        .expect("fixture edge")
}

fn fixture(t: &mut Topology) -> (SolidId, [EdgeId; 3]) {
    let c = make_cylinder(t, 6.0, 35.0).unwrap();
    let b = make_box(t, 30.0, 18.0, 26.0).unwrap();
    let body = boolean(t, BooleanOp::Fuse, b, c).unwrap();
    let p = Point3::new;
    let edges = [
        between(t, body, p(6.0, 0.0, 26.0), p(0.0, 6.0, 26.0)),
        between(t, body, p(6.0, 0.0, 0.0), p(6.0, 0.0, 26.0)),
        between(t, body, p(6.0, 0.0, 26.0), p(30.0, 0.0, 26.0)),
    ];
    (body, edges)
}

#[test]
fn three_edges_build_in_all_six_selection_orders() {
    for radius in [0.5, 1.0] {
        // The staged construction is a regression oracle for which material
        // is changed, in addition to independent band/volume/closure checks.
        let mut reference = Topology::new();
        let (body, edges) = fixture(&mut reference);
        let first = fillet_cascade(&mut reference, body, &edges[..2], radius).unwrap();
        let top = between(
            &reference,
            first.solid,
            Point3::new(6.0 + radius, 0.0, 26.0),
            Point3::new(30.0, 0.0, 26.0),
        );
        let last = fillet_cascade(&mut reference, first.solid, &[top], radius).unwrap();
        let expected = solid_volume(&reference, last.solid, 0.005).unwrap();
        for order in [
            [0, 1, 2],
            [0, 2, 1],
            [1, 0, 2],
            [1, 2, 0],
            [2, 0, 1],
            [2, 1, 0],
        ] {
            let mut t = Topology::new();
            let (body, edges) = fixture(&mut t);
            let selected = order.map(|i| edges[i]);
            let result =
                fillet_cascade(&mut t, body, &selected, radius).expect("one three-edge fillet");
            assert_eq!(result.succeeded, selected);
            assert!(result.failed.is_empty() && !result.is_partial);
            assert!(validate_solid(&t, result.solid).unwrap().is_valid());
            let shell = t
                .shell(t.solid(result.solid).unwrap().outer_shell())
                .unwrap();
            validate_shell_closed(shell, &t).unwrap();
            let volume = solid_volume(&t, result.solid, 0.005).unwrap();
            assert!(
                (volume - expected).abs() < 1e-6,
                "{order:?}: {volume} vs {expected}"
            );
            // A planar-only subset removes material; the requested seam adds
            // material as well. Require its exact torus and cylinder bands.
            let surfaces: Vec<_> = solid_faces(&t, result.solid)
                .unwrap()
                .into_iter()
                .map(|f| t.face(f).unwrap().surface())
                .collect();
            assert!(surfaces.iter().any(|s|matches!(s,FaceSurface::Torus(torus) if (torus.minor_radius()-radius).abs()<1e-7)));
            assert!(
                surfaces.iter().any(
                    |s| matches!(s,FaceSurface::Cylinder(c) if (c.radius()-radius).abs()<1e-7)
                )
            );
        }
    }
}

#[test]
fn refused_three_edge_fillet_leaves_input_byte_identical() {
    let mut t = Topology::new();
    let (body, edges) = fixture(&mut t);
    let before = remus_io::arena_io::serialize_solid(&t, body).unwrap();
    assert!(fillet_cascade(&mut t, body, &edges, 40.0).is_err());
    assert_eq!(
        before,
        remus_io::arena_io::serialize_solid(&t, body).unwrap()
    );
}
