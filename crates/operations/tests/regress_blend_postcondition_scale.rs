//! A fillet on a micron-scale part is judged at the part's own scale.
//!
//! The blend postcondition validates its result with the checker's default
//! options, whose geometric tolerance is an absolute 1e-6. A 2 µm box with a
//! 0.3 µm fillet on its verticals is geometrically exact — sixteen vertices at
//! the expected corners — yet the wire self-intersection check flagged its cap
//! edges as touching at exactly the fillet chord, 0.42 µm, and the fail-closed
//! postcondition refused the fillet. The postcondition now scales the
//! validator's tolerance to the input's shortest edge.
#![allow(deprecated, clippy::unwrap_used, clippy::expect_used)] // `fillet::fillet` is the engine the wasm `fillet` binding falls back to.

use remus_check::validate::{ValidateOptions, validate_solid};
use remus_operations::fillet::fillet;
use remus_operations::primitives::make_box;
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_vertices};
use remus_topology::solid::SolidId;

fn vertical_edges(topo: &Topology, solid: SolidId) -> Vec<remus_topology::edge::EdgeId> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|edge| {
            let data = topo.edge(*edge).unwrap();
            let start = topo.vertex(data.start()).unwrap().point();
            let end = topo.vertex(data.end()).unwrap().point();
            (start.x() - end.x()).abs() < 1e-12 && (start.y() - end.y()).abs() < 1e-12
        })
        .collect()
}

#[test]
fn micron_box_fillet_is_not_refused_by_the_validator_floor() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2e-6, 2e-6, 1e-6).unwrap();
    let verticals = vertical_edges(&topo, solid);
    assert_eq!(verticals.len(), 4);

    let filleted = fillet(&mut topo, solid, &verticals, 3e-7)
        .expect("a correct sub-micron fillet must not be refused");

    let vertices = solid_vertices(&topo, filleted).unwrap();
    assert_eq!(vertices.len(), 16);
    // Every vertex sits on a fillet tangent line: one coordinate at 0.3 µm
    // or 1.7 µm, the other on the box boundary.
    for vertex in &vertices {
        let p = topo.vertex(*vertex).unwrap().point();
        let on_tangent = |c: f64| (c - 3e-7).abs() < 1e-12 || (c - 1.7e-6).abs() < 1e-12;
        let on_boundary = |c: f64| c.abs() < 1e-12 || (c - 2e-6).abs() < 1e-12;
        assert!(
            (on_tangent(p.x()) && on_boundary(p.y())) || (on_boundary(p.x()) && on_tangent(p.y())),
            "vertex off the fillet tangents: {p:?}"
        );
    }

    // The default validator still reads the part as self-intersecting: the
    // refusal was the checker's floor, not the geometry.
    let at_default = validate_solid(&topo, filleted, &ValidateOptions::default()).unwrap();
    assert!(!at_default.is_valid());
    let scaled = ValidateOptions {
        tolerance_scale: 0.1,
        ..Default::default()
    };
    assert!(validate_solid(&topo, filleted, &scaled).unwrap().is_valid());
}

#[test]
fn ordinary_scale_fillets_keep_the_default_floor() {
    // At 2 mm the same construction validates under the default options, so
    // the scaled postcondition must accept exactly what the default accepted.
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2e-3, 2e-3, 1e-3).unwrap();
    let verticals = vertical_edges(&topo, solid);
    let filleted = fillet(&mut topo, solid, &verticals, 3e-4).unwrap();
    assert_eq!(solid_vertices(&topo, filleted).unwrap().len(), 16);
    assert!(
        validate_solid(&topo, filleted, &ValidateOptions::default())
            .unwrap()
            .is_valid()
    );
}
