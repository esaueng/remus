//! Mass properties of a solid whose rim fillet is a revolved torus band.
//!
//! Rolling a ball round a circular rim leaves a torus face bounded by two full
//! circles — the wall's tangent circle and the cap's — that each wrap the
//! torus's MAJOR angle once, with no seam edge between them. The face
//! integrator in `remus-check` only qualified bands whose rims wrap the tube
//! angle, so this face fell to the generic path: its projected outline spans
//! both periods, trims nothing, and the whole closed torus (`2π²Rr²`) was
//! integrated in its place. `solid_volume` reads the band in closed form
//! (`torus_parallel_band_signed_volume`), but `mass_properties` and the centre
//! of mass integrate every face through `remus-check`, and a filleted boss on a
//! block read 12292.9 mm³ there against an exact 12409.9.
//!
//! Every expectation here is a closed form (Pappus for the fillet), not the
//! kernel's reading of itself.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::PI;

use remus_operations::blend_ops::{blend_failure_code, fillet_cascade};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{mass_properties, solid_center_of_mass, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::solid_edges;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

/// A caller's preview deflection: the exact routes must not depend on it.
const DEFLECTION: f64 = 0.001;

/// The arcs of the `radius` circle centred on the z axis at height `z`: the
/// fuse leaves a rim split at its quadrant vertices.
fn rim(topo: &Topology, solid: SolidId, z: f64, radius: f64) -> Vec<EdgeId> {
    let arcs: Vec<_> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            matches!(topo.edge(id).unwrap().curve(), EdgeCurve::Circle(c)
                if (c.radius() - radius).abs() < 1e-9
                    && c.center().x().abs() < 1e-9
                    && c.center().y().abs() < 1e-9
                    && (c.center().z() - z).abs() < 1e-9)
        })
        .collect();
    assert!(!arcs.is_empty(), "rim circle");
    arcs
}

/// The fillet's torus faces in `solid`: each must carry both rims as wires.
fn assert_band_faces(topo: &Topology, solid: SolidId, label: &str) {
    let bands: Vec<_> = remus_topology::explorer::solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .collect();
    assert_eq!(bands.len(), 1, "{label}: one torus band");
    assert_eq!(
        topo.face(bands[0]).unwrap().inner_wires().len(),
        1,
        "{label}: the band is bounded by two full rims, not a seamed rectangle"
    );
}

/// Block 30 × 18 × 18 fused with an r = 6 boss on the z axis standing 28.5
/// high (three quarters of it outside the block below z = 18), its top rim
/// filleted r = 1.8. The fillet's cap above the wall is a solid of revolution
/// of the profile `ρ = 4.2 + √(1.8² − t²)` for `t ∈ [0, 1.8]`.
#[test]
fn filleted_boss_on_a_block_measures_its_fillet_band() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 30.0, 18.0, 18.0).unwrap();
    let boss = make_cylinder(&mut topo, 6.0, 28.5).unwrap();
    let body = boolean(&mut topo, BooleanOp::Fuse, block, boss).unwrap();

    // The un-filleted fuse is already exact; the defect is the band alone.
    let plain = 9720.0 + 0.75 * PI * 36.0 * 18.0 + PI * 36.0 * 10.5;
    let measured = solid_volume(&topo, body, DEFLECTION).unwrap();
    assert!(
        (measured - plain).abs() < 1e-6,
        "fused boss: {measured} vs {plain}"
    );

    let edges = rim(&topo, body, 28.5, 6.0);
    let result = fillet_cascade(&mut topo, body, &edges, 1.8)
        .unwrap_or_else(|e| panic!("fillet: {}: {e}", blend_failure_code(&e)));
    assert!(result.failed.is_empty(), "partial blend");
    assert_band_faces(&topo, result.solid, "filleted boss");

    let (big, small) = (4.2_f64, 1.8_f64);
    let cap =
        PI * (big * big * small + 2.0 * big * PI * small * small / 4.0 + 2.0 / 3.0 * small.powi(3));
    let lower = 0.75 * PI * 36.0 * 18.0;
    let upper = PI * 36.0 * 8.7;
    let expected = 9720.0 + lower + upper + cap;
    let measured = solid_volume(&topo, result.solid, DEFLECTION).unwrap();
    assert!(
        (measured - expected).abs() < 1e-6,
        "filleted boss: {measured} vs {expected} (whole torus 2π²Rr² = {})",
        2.0 * PI * PI * big * small * small
    );

    // First moments: the block about its centre; the three-quarter disc's
    // centroid is minus a third of the quarter disc's `4r/3π` on each axis;
    // the cap's height moment is `π∫ρ²(t)(26.7 + t) dt`.
    let quarter_offset = -(4.0 * 6.0 / (3.0 * PI)) / 3.0;
    let cap_moment_z = 26.7 * cap
        + PI * (big * big * small * small / 2.0
            + 2.0 * big * small.powi(3) / 3.0
            + small.powi(4) / 4.0);
    let center = [
        (9720.0 * 15.0 + lower * quarter_offset) / expected,
        (9720.0 * 9.0 + lower * quarter_offset) / expected,
        (9720.0 * 9.0 + lower * 9.0 + upper * 22.35 + cap_moment_z) / expected,
    ];
    let properties = mass_properties(&topo, result.solid).unwrap();
    assert!(
        (properties.mass - expected).abs() < 1e-6,
        "mass properties: {} vs {expected}",
        properties.mass
    );
    let centroid = solid_center_of_mass(&topo, result.solid, DEFLECTION).unwrap();
    for (label, got) in [
        ("mass properties", properties.center),
        ("centre of mass", centroid),
    ] {
        for (axis, (got, want)) in [got.x(), got.y(), got.z()]
            .into_iter()
            .zip(center)
            .enumerate()
        {
            assert!(
                (got - want).abs() < 1e-8,
                "{label} axis {axis}: {got} vs {want}"
            );
        }
    }
}
