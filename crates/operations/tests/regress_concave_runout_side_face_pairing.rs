//! A concave stripe that runs out across a perpendicular side face.
//!
//! A 20x8x30 tab stands on a 40x30x4 plate, flush with its front. Filleting
//! one concave tab-base edge on its own ends the stripe at the tab's back
//! corner, where the back face is perpendicular to it: the stripe's end
//! section lands on that face, one contact on the vertical tab edge and the
//! other on the backward extension of the back face's bottom edge. The side
//! face trim used to pair contacts with edges by distance to `corner + r·dir`,
//! which takes the extension contact for the vertical edge there; the face
//! crossed itself and the shell stayed open, on one hand of the tab or the
//! other. The added material is the full stripe, `A·8` with `A = r²(1 − π/4)`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_edges;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

fn plate_with_tab(topo: &mut Topology) -> SolidId {
    let plate = make_box(topo, 40.0, 30.0, 4.0).unwrap();
    let tab = make_box(topo, 20.0, 8.0, 30.0).unwrap();
    transform_solid(topo, tab, &Mat4::translation(10.0, 0.0, 0.0)).unwrap();
    boolean(topo, BooleanOp::Fuse, plate, tab).unwrap()
}

fn edge_between(topo: &Topology, solid: SolidId, a: Point3, b: Point3) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            ((start - a).length() < 1e-8 && (end - b).length() < 1e-8)
                || ((start - b).length() < 1e-8 && (end - a).length() < 1e-8)
        })
        .unwrap_or_else(|| panic!("edge {a:?} -- {b:?}"))
}

#[test]
fn concave_tab_base_stripe_runs_out_across_the_back_face_on_either_hand() {
    let r = 1.0;
    for x in [10.0, 30.0] {
        let mut topo = Topology::new();
        let input = plate_with_tab(&mut topo);
        let edge = edge_between(
            &topo,
            input,
            Point3::new(x, 0.0, 4.0),
            Point3::new(x, 8.0, 4.0),
        );
        let result = fillet_cascade(&mut topo, input, &[edge], r)
            .unwrap_or_else(|e| panic!("x={x}: concave tab-base fillet must build: {e}"));
        let shell = topo
            .shell(topo.solid(result.solid).unwrap().outer_shell())
            .unwrap();
        validate_shell_closed(shell, &topo).unwrap();
        let expected = 8960.0 + r * r * (1.0 - PI / 4.0) * 8.0;
        let volume = solid_volume(&topo, result.solid, 0.005).unwrap();
        assert!(
            (volume - expected).abs() < 1e-6,
            "x={x}: volume {volume:.9} vs {expected:.9}"
        );
    }
}
