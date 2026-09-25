//! B63 regression: the variable-radius fillet accepted a radius its support
//! faces cannot hold and returned wrong geometry that still validated.
//!
//! Found while adding the O4.7 `filletVariableDetailed` twin (2026-09-25). On
//! a 10³ box, a constant-law radius 11 or 20 stripe on one edge returned a
//! solid measuring 474.0 and 427.7 where the rounded box would measure
//! 740.3 and 141.6. The walking engine refuses the same request with
//! `cliff-encountered` ("requested radius 11, available radius 10").
//!
//! Fixed: `fillet_variable_with_setbacks` casts across each planar support
//! face at stations along the stripe and refuses, with the walking engine's
//! typed cliff error, any radius that reaches the face's far boundary. The
//! refusal leaves the topology untouched.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::Point3;
use remus_operations::OperationsError;
use remus_operations::blend_ops::{BlendError, blend_failure_code, fillet_v2};
use remus_operations::fillet::{FilletEdgeSetback, FilletRadiusLaw, fillet_variable_with_setbacks};
use remus_operations::primitives::make_box;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::solid::SolidId;

/// A 10³ box and its vertical edge through the origin: both support faces
/// are 10 wide.
fn box_and_origin_edge() -> (Topology, SolidId, EdgeId) {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
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
    (topo, solid, edge)
}

fn variable(
    topo: &mut Topology,
    solid: SolidId,
    edge: EdgeId,
    law: FilletRadiusLaw,
) -> Result<SolidId, OperationsError> {
    fillet_variable_with_setbacks(
        topo,
        solid,
        &[FilletEdgeSetback {
            edge,
            law,
            start_setback: 0.0,
            end_setback: 0.0,
        }],
    )
}

#[test]
fn variable_fillet_refuses_a_radius_wider_than_its_support_faces() {
    for radius in [11.0, 20.0] {
        let (mut topo, solid, edge) = box_and_origin_edge();
        let before = solid_entity_counts(&topo, solid).unwrap();
        let arena_before = (topo.num_faces(), topo.num_edges(), topo.num_vertices());

        let result = variable(&mut topo, solid, edge, FilletRadiusLaw::Constant(radius));

        let error = match result {
            Ok(result) => {
                panic!("radius {radius} exceeds the 10-wide support faces but returned {result:?}")
            }
            Err(error) => error,
        };
        assert_eq!(
            blend_failure_code(&error),
            "cliff-encountered",
            "radius {radius}: {error}"
        );
        let OperationsError::Blend(BlendError::CliffEncountered {
            edge: cliff_edge,
            face,
            requested_radius,
            available_radius,
        }) = error
        else {
            panic!("radius {radius}: expected CliffEncountered, got {error}");
        };
        assert_eq!(cliff_edge, edge);
        assert!(solid_faces(&topo, solid).unwrap().contains(&face));
        assert!((requested_radius - radius).abs() < 1e-12);
        assert!(
            (available_radius - 10.0).abs() < 1e-9,
            "available {available_radius}"
        );

        assert_eq!(
            solid_entity_counts(&topo, solid).unwrap(),
            before,
            "a refused variable fillet must leave the input untouched"
        );
        assert_eq!(
            (topo.num_faces(), topo.num_edges(), topo.num_vertices()),
            arena_before,
            "a refused variable fillet must roll the arena back"
        );
    }
}

/// The variable engine and the walking engine now give the same diagnostic
/// for the same over-wide constant radius.
#[test]
fn variable_and_walking_cliff_refusals_agree() {
    let (mut variable_topo, variable_solid, variable_edge) = box_and_origin_edge();
    let variable_error = variable(
        &mut variable_topo,
        variable_solid,
        variable_edge,
        FilletRadiusLaw::Constant(11.0),
    )
    .unwrap_err();
    let (mut walking_topo, walking_solid, walking_edge) = box_and_origin_edge();
    let walking_error = fillet_v2(&mut walking_topo, walking_solid, &[walking_edge], 11.0)
        .map(|result| result.solid)
        .unwrap_err();
    assert_eq!(variable_error.to_string(), walking_error.to_string());
}

/// The bound is the face width: at it the contact would collapse the
/// adjacent edge, so it is refused like the rolling-ball engine refuses it;
/// just inside it the stripe still builds.
#[test]
fn variable_fillet_cliff_bound_is_the_support_width() {
    let (mut topo, solid, edge) = box_and_origin_edge();
    let error = variable(&mut topo, solid, edge, FilletRadiusLaw::Constant(10.0)).unwrap_err();
    assert_eq!(blend_failure_code(&error), "cliff-encountered", "{error}");

    for radius in [1.0, 9.0, 9.9] {
        let (mut topo, solid, edge) = box_and_origin_edge();
        variable(&mut topo, solid, edge, FilletRadiusLaw::Constant(radius))
            .unwrap_or_else(|error| panic!("radius {radius} fits the 10-wide faces: {error}"));
    }
}

/// A graded law is refused at the station that overruns the face, and reports
/// that station's radius.
#[test]
fn variable_fillet_refuses_a_law_that_grows_past_its_support() {
    for law in [
        FilletRadiusLaw::Linear {
            start: 2.0,
            end: 12.0,
        },
        FilletRadiusLaw::SCurve {
            start: 12.0,
            end: 2.0,
        },
    ] {
        let (mut topo, solid, edge) = box_and_origin_edge();
        let before = solid_entity_counts(&topo, solid).unwrap();
        let error = variable(&mut topo, solid, edge, law.clone()).unwrap_err();
        let OperationsError::Blend(BlendError::CliffEncountered {
            requested_radius,
            available_radius,
            ..
        }) = error
        else {
            panic!("{law:?}: expected CliffEncountered, got {error}");
        };
        assert!(
            (requested_radius - 12.0).abs() < 1e-12,
            "{law:?}: {requested_radius}"
        );
        assert!(
            (available_radius - 10.0).abs() < 1e-9,
            "{law:?}: {available_radius}"
        );
        assert_eq!(solid_entity_counts(&topo, solid).unwrap(), before);
    }

    // The same laws kept inside the faces still build.
    let (mut topo, solid, edge) = box_and_origin_edge();
    variable(
        &mut topo,
        solid,
        edge,
        FilletRadiusLaw::Linear {
            start: 2.0,
            end: 9.0,
        },
    )
    .expect("a law inside the support faces builds");
}
