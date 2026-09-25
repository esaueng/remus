//! B63 ready-repro: the variable-radius fillet accepts a radius its support
//! faces cannot hold and returns wrong geometry that still validates.
//!
//! Found while adding the O4.7 `filletVariableDetailed` twin (2026-09-25). On
//! a 10³ box, a constant-law radius 11 or 20 stripe on one edge returns a
//! solid measuring 474.0 and 427.7 where the rounded box would measure
//! 740.3 and 141.6. The walking engine refuses the same request with
//! `cliff-encountered` ("requested radius 11, available radius 10").
//!
//! Acceptance: the variable engine refuses a radius wider than its support
//! faces and leaves the topology untouched.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::Point3;
use remus_operations::fillet::{FilletEdgeSetback, FilletRadiusLaw, fillet_variable_with_setbacks};
use remus_operations::primitives::make_box;
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_entity_counts};

#[test]
#[ignore = "open: B63 variable fillet accepts a radius wider than its support faces"]
fn variable_fillet_refuses_a_radius_wider_than_its_support_faces() {
    for radius in [11.0, 20.0] {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        // The vertical edge through the origin: both support faces are 10 wide.
        let origin = Point3::new(0.0, 0.0, 0.0);
        let top = Point3::new(0.0, 0.0, 10.0);
        let edge = solid_edges(&topo, solid)
            .unwrap()
            .into_iter()
            .find(|&edge| {
                let data = topo.edge(edge).unwrap();
                let a = topo.vertex(data.start()).unwrap().point();
                let b = topo.vertex(data.end()).unwrap().point();
                ((a - origin).length() < 1e-9 && (b - top).length() < 1e-9)
                    || ((a - top).length() < 1e-9 && (b - origin).length() < 1e-9)
            })
            .expect("box edge through the origin along z");
        let before = solid_entity_counts(&topo, solid).unwrap();

        let result = fillet_variable_with_setbacks(
            &mut topo,
            solid,
            &[FilletEdgeSetback {
                edge,
                law: FilletRadiusLaw::Constant(radius),
                start_setback: 0.0,
                end_setback: 0.0,
            }],
        );

        assert!(
            result.is_err(),
            "radius {radius} exceeds the 10-wide support faces but returned {result:?}"
        );
        assert_eq!(
            solid_entity_counts(&topo, solid).unwrap(),
            before,
            "a refused variable fillet must leave the input untouched"
        );
    }
}
