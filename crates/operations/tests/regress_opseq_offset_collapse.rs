//! P-Class 8.1 campaign findings: an inward offset of a plain box that
//! reaches its half-thickness is a typed `OffsetCollapsed` refusal.
//!
//! Before, the campaign counted these as untyped errors:
//!
//! - seeds 10434068600042686585 and 3388039345656333454: a box one unit
//!   thick, offset by -0.5, collapses to zero thickness. The coincident
//!   opposite faces reached assembly and failed as
//!   `Topology(NonManifold { "edge ... is used by 4 wires" })`.
//! - seed 14788754164979531939: a box half a unit thick offset by -0.5
//!   passes through itself. The postcondition caught the inside-out result
//!   but returned a generic `InvalidInput`.
//!
//! An all-planar, straight-edged solid lies between two parallel planes
//! along each of its face normals; an inward offset by `|d|` leaves nothing
//! of positive volume once `2|d|` reaches the smallest such width. That is
//! certified before construction, so nothing is allocated. Just short of it
//! the offset still succeeds with its closed-form volume.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::arena_io::serialize_solids;
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::journal_ops::offset_journaled;
use remus_operations::measure::solid_volume;
use remus_operations::offset_v2::{
    offset_solid_arc_v2, offset_solid_v2, offset_solid_v2_with_entity_evolution,
    offset_solid_v2_with_evolution,
};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

fn live_state(t: &Topology) -> String {
    format!(
        "{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}{}",
        t.vertices().iter().collect::<Vec<_>>(),
        t.edges().iter().collect::<Vec<_>>(),
        t.wires().iter().collect::<Vec<_>>(),
        t.faces().iter().collect::<Vec<_>>(),
        t.shells().iter().collect::<Vec<_>>(),
        t.solids().iter().collect::<Vec<_>>(),
        t.compounds().iter().collect::<Vec<_>>(),
        t.compsolids().iter().collect::<Vec<_>>(),
        (t.num_loops(), t.num_coedges(), t.num_pcurves()),
        t.journal().snapshot(),
        t.mutation_ticks()
    )
}

/// The campaign's quarter-turn placement: rotate about y, then translate.
fn campaign_placement() -> Mat4 {
    Mat4::translation(0.0, -3.0, -2.0) * Mat4::rotation_y(std::f64::consts::FRAC_PI_2)
}

fn oblique_placement(scale: f64) -> Mat4 {
    Mat4::translation(11.0 * scale, -7.0 * scale, 5.0 * scale)
        * Mat4::rotation_z(0.7)
        * Mat4::rotation_x(0.3)
}

const ROUTES: [&str; 5] = [
    "offset",
    "offset+evolution",
    "offset+entities",
    "arc offset",
    "journaled",
];

fn run_route(
    topo: &mut Topology,
    route: &str,
    solid: SolidId,
    d: f64,
) -> Result<(), OperationsError> {
    match route {
        "offset" => offset_solid_v2(topo, solid, d).map(|_| ()),
        "offset+evolution" => offset_solid_v2_with_evolution(topo, solid, d).map(|_| ()),
        "offset+entities" => offset_solid_v2_with_entity_evolution(topo, solid, d).map(|_| ()),
        "arc offset" => offset_solid_arc_v2(topo, solid, d).map(|_| ()),
        "journaled" => offset_journaled(topo, solid, d).map(|_| ()),
        other => panic!("unknown route {other}"),
    }
}

/// Every offset entry refuses with the typed collapse, carrying the
/// requested distance and the certified thickness, and leaves the live
/// model (topology, journal, handles) exactly as it was.
fn assert_typed_collapse(topo: &Topology, solid: SolidId, d: f64, thickness: f64, label: &str) {
    for route in ROUTES {
        let mut t = topo.clone();
        let before = live_state(&t);
        let bytes = serialize_solids(&t, &[solid]).unwrap();
        let slots = t.allocated_slot_count();
        let error = run_route(&mut t, route, solid, d).expect_err(label);
        let OperationsError::OffsetCollapsed {
            operation,
            distance,
            thickness: Some(certified),
        } = error
        else {
            panic!("{label} via {route}: mistyped refusal {error:?}");
        };
        let want = if route == "arc offset" {
            "arc offset"
        } else {
            "offset"
        };
        assert_eq!(operation, want, "{label} via {route}");
        assert_eq!(distance.to_bits(), d.to_bits(), "{label} via {route}");
        assert!(
            (certified - thickness).abs() <= 1e-12 * thickness,
            "{label} via {route}: certified {certified}, want {thickness}"
        );
        assert!(error_message(route, solid, d, topo).contains("collapsed the solid"));
        assert_eq!(live_state(&t), before, "{label} via {route}: model changed");
        assert_eq!(
            serialize_solids(&t, &[solid]).unwrap(),
            bytes,
            "{label} via {route}"
        );
        assert_eq!(
            t.allocated_slot_count(),
            slots,
            "{label} via {route}: refused before construction, nothing allocated"
        );
        assert!(
            t.solid(solid).is_ok(),
            "{label} via {route}: source handle lost"
        );
    }
}

fn error_message(route: &str, solid: SolidId, d: f64, topo: &Topology) -> String {
    let mut t = topo.clone();
    run_route(&mut t, route, solid, d).unwrap_err().to_string()
}

fn assert_closed_form_offset(topo: &Topology, solid: SolidId, d: f64, dims: [f64; 3], label: &str) {
    let mut t = topo.clone();
    let result = offset_solid_v2(&mut t, solid, d)
        .unwrap_or_else(|e| panic!("{label}: offset {d} must still succeed: {e}"));
    let want: f64 = dims.iter().map(|s| d.mul_add(2.0, *s)).product();
    let got = solid_volume(&t, result, 1e-3 * dims[0].min(dims[1]).min(dims[2])).unwrap();
    assert!(
        (got - want).abs() <= 1e-9 * want,
        "{label}: offset {d} volume {got}, closed form {want}"
    );
}

/// Seeds 10434068600042686585 and 3388039345656333454 literally: the box
/// placed by the campaign transform, offset -0.5 to zero thickness.
#[test]
fn campaign_zero_thickness_seeds_refuse_typed() {
    for (dims, label) in [
        ([2.0, 5.5, 1.0], "seed 10434068600042686585"),
        ([5.5, 1.0, 5.0], "seed 3388039345656333454"),
    ] {
        let mut t = Topology::new();
        let source = make_box(&mut t, dims[0], dims[1], dims[2]).unwrap();
        transform_solid(&mut t, source, &campaign_placement()).unwrap();
        assert_typed_collapse(&t, source, -0.5, 1.0, label);
        // The next lattice step in is still a real body.
        assert_closed_form_offset(&t, source, -0.25, dims, label);
    }
}

/// Seed 14788754164979531939 literally: a unit-wide box grown, mirrored and
/// shrunk back down to half a unit thick, then offset through itself.
#[test]
fn campaign_pass_through_seed_refuses_typed() {
    let mut t = Topology::new();
    let source = make_box(&mut t, 1.0, 5.0, 1.0).unwrap();
    let grown = offset_solid_v2(&mut t, source, 0.5).unwrap();
    let mirrored = remus_operations::mirror::mirror(
        &mut t,
        grown,
        Point3::new(0.0, -3.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    assert!((solid_volume(&t, mirrored, 0.01).unwrap() - 24.0).abs() < 1e-9);
    let back = offset_solid_v2(&mut t, grown, -0.5).unwrap();
    let thin = offset_solid_v2(&mut t, back, -0.25).unwrap();
    assert!((solid_volume(&t, thin, 0.01).unwrap() - 1.125).abs() < 1e-9);
    assert_typed_collapse(&t, thin, -0.5, 0.5, "seed 14788754164979531939");
}

/// The collapse boundary across scales and placements: exactly half the
/// thickness (zero thickness), past it (pass-through), and far past it
/// refuse; just short of it the closed form holds.
#[test]
fn box_collapse_boundary_holds_across_scales_and_placements() {
    for scale in [0.01, 1.0, 1000.0] {
        for placed in [false, true] {
            for dims in [[1.0, 5.0, 1.0], [2.0, 5.5, 1.0], [3.0, 3.0, 3.0]] {
                let dims = dims.map(|s| s * scale);
                let thickness = dims.iter().copied().fold(f64::INFINITY, f64::min);
                let mut t = Topology::new();
                let source = make_box(&mut t, dims[0], dims[1], dims[2]).unwrap();
                if placed {
                    transform_solid(&mut t, source, &oblique_placement(scale)).unwrap();
                }
                let label = format!("dims={dims:?} placed={placed}");
                for factor in [0.5, 0.75, 1.0, 1e6] {
                    assert_typed_collapse(&t, source, -factor * thickness, thickness, &label);
                }
                for factor in [0.25, 0.45, 0.49] {
                    assert_closed_form_offset(&t, source, -factor * thickness, dims, &label);
                }
                // Outward offsets are never collapse candidates.
                assert_closed_form_offset(&t, source, 0.5 * thickness, dims, &label);
            }
        }
    }
}

/// The slab argument does not need convexity: an L-section prism two units
/// deep cannot keep anything past a -1 offset, whatever its arms do.
#[test]
fn non_convex_planar_prism_is_certified_by_its_thinnest_slab() {
    let mut t = Topology::new();
    let profile = remus_topology::builder::make_planar_face(
        &mut t,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(4.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, 4.0, 0.0),
            Point3::new(0.0, 4.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let source =
        remus_operations::extrude::extrude(&mut t, profile, Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
    for d in [-1.0, -1.2, -5.0] {
        assert_typed_collapse(&t, source, d, 2.0, "L prism");
    }
}

/// Non-finite arguments must reach input validation rather than the
/// geometric collapse certificate, including negative infinity.
#[test]
fn non_finite_distances_keep_their_invalid_input_refusal() {
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 2.0, 5.5, 1.0).unwrap();
    for distance in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for route in ROUTES {
            let mut t = topo.clone();
            let before = live_state(&t);
            let error = run_route(&mut t, route, source, distance).unwrap_err();
            assert!(
                matches!(error, OperationsError::InvalidInput { ref reason } if reason.contains("finite")),
                "{route} with {distance}: {error:?}"
            );
            assert_eq!(live_state(&t), before, "{route} with {distance}");
        }
    }
}

/// A cylinder's existing radius guard remains unchanged.
#[test]
fn curved_bodies_keep_their_existing_refusal() {
    let mut t = Topology::new();
    let source = make_cylinder(&mut t, 1.0, 10.0).unwrap();
    let before = live_state(&t);
    let error = offset_solid_v2(&mut t, source, -1.0).unwrap_err();
    assert!(
        matches!(error, OperationsError::InvalidInput { ref reason } if reason.contains("cylinder offset collapses")),
        "{error:?}"
    );
    assert_eq!(live_state(&t), before);
}
