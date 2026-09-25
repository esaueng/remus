//! B20 follow-up: the adaptive mass-property controls on a NURBS-heavy import.
//!
//! The Shapr3D hammer holder has 160 faces: 42 trimmed NURBS faces, 42
//! polygon-trimmed cylinders, 14 tori, 8 spheres, 2 cones and 52 planes. Before
//! this change every curved face refused a non-default `adaptive_eps` or
//! `max_depth`, so the controls the native, facade and WASM mass-property
//! paths accept could not be exercised on real imported geometry at all.
//!
//! There is no closed form for this body, so convergence is checked against
//! the finest setting (order 8, `adaptive_eps = 1e-9`), and that reference is
//! checked against two measurements that share no code with the integrator: a
//! signed-tetrahedron volume of the watertight tessellation, and the vendor's
//! own mass report pinned by `regress_shapr3d_reversed_nurbs_faces`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::properties::{GProps, PropertiesOptions};
use remus_io::step::reader::read_step;
use remus_operations::measure::{
    mass_properties, mass_properties_default_options, mass_properties_with_options,
};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const HAMMER_HOLDER: &str = include_str!("data/shapr3d_hammer_holder.step");
/// Vendor mass report, as pinned by `regress_shapr3d_reversed_nurbs_faces`.
const VENDOR_VOLUME: f64 = 50_240.482_8;

fn load() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solids = read_step(HAMMER_HOLDER, &mut topo).expect("import hammer holder");
    assert_eq!(solids.len(), 1);
    (topo, solids[0])
}

fn options(gauss_order: usize, adaptive_eps: f64, max_depth: usize) -> PropertiesOptions {
    PropertiesOptions {
        gauss_order,
        adaptive_eps,
        max_depth,
    }
}

/// Relative distance of volume and the three principal-axis-aligned inertia
/// diagonals, the latter scaled by the largest diagonal so a small component
/// cannot inflate the ratio.
fn distance(a: &GProps, b: &GProps) -> [f64; 4] {
    let scale = b.inertia[..3].iter().fold(0.0_f64, |m, x| m.max(x.abs()));
    [
        (a.mass - b.mass).abs() / b.mass.abs(),
        (a.inertia[0] - b.inertia[0]).abs() / scale,
        (a.inertia[1] - b.inertia[1]).abs() / scale,
        (a.inertia[2] - b.inertia[2]).abs() / scale,
    ]
}

/// Signed-tetrahedron volume of a closed triangle mesh, with no quadrature.
fn mesh_volume(topo: &Topology, solid: SolidId, deflection: f64) -> f64 {
    let mesh = tessellate_solid(topo, solid, deflection).expect("tessellate");
    assert_eq!(boundary_edge_count(&mesh), 0, "mesh must be watertight");
    assert_eq!(non_manifold_edge_count(&mesh), 0, "mesh must be manifold");
    let mut six = 0.0;
    for t in mesh.indices.chunks_exact(3) {
        let [a, b, c] = [t[0], t[1], t[2]].map(|i| mesh.positions[i as usize]);
        let (a, b, c) = (
            remus_math::vec::Vec3::new(a.x(), a.y(), a.z()),
            remus_math::vec::Vec3::new(b.x(), b.y(), b.z()),
            remus_math::vec::Vec3::new(c.x(), c.y(), c.z()),
        );
        six += a.dot(b.cross(c));
    }
    six / 6.0
}

fn reference(topo: &Topology, solid: SolidId) -> GProps {
    mass_properties_with_options(topo, solid, &options(8, 1e-9, 16)).unwrap()
}

/// Measured (2026-09-25): every rung within 2.3e-10 of the reference, eps 1e-3
/// and 1e-5 identical (the knot-aligned cells converge at the first
/// comparison), eps 1e-7 within 4.3e-11; the coarse request is off by 1.1e-6
/// on volume; the default fixed rule by 1.9e-7.
#[test]
fn hammer_holder_adaptive_ladder_converges_monotonically_to_the_finest_reference() {
    let (topo, solid) = load();

    // The default controls are the historical call, bit for bit.
    let historical = mass_properties(&topo, solid).unwrap();
    let defaults =
        mass_properties_with_options(&topo, solid, &mass_properties_default_options()).unwrap();
    assert_eq!(format!("{historical:?}"), format!("{defaults:?}"));

    let reference = reference(&topo, solid);

    // Every setting lands within its own request of the reference, and the
    // distance never grows as the request tightens.
    let mut previous = [f64::INFINITY; 4];
    for eps in [1e-3, 1e-5, 1e-7] {
        let props = mass_properties_with_options(&topo, solid, &options(8, eps, 16)).unwrap();
        let now = distance(&props, &reference);
        for (k, (&n, &p)) in now.iter().zip(&previous).enumerate() {
            assert!(n <= eps, "component {k} at eps {eps:e}: {n:e}");
            assert!(n <= p, "component {k} grew at eps {eps:e}: {n:e} > {p:e}");
        }
        previous = now;
    }
    let tight = previous.iter().copied().fold(0.0, f64::max);
    assert!(tight <= 1e-9, "tightest rung {tight:e}");

    // A deliberately coarse request is accepted and visibly less accurate.
    let coarse = mass_properties_with_options(&topo, solid, &options(2, 1e-1, 2)).unwrap();
    let coarse = distance(&coarse, &reference);
    let worst_coarse = coarse.iter().copied().fold(0.0, f64::max);
    assert!(coarse[0] > 1e-7, "coarse volume {:e}", coarse[0]);
    assert!(
        worst_coarse > 1e3 * tight,
        "coarse {worst_coarse:e} vs tight {tight:e}"
    );

    // The historical fixed rule stays inside B20's 1e-6 measurement bound.
    let fixed = distance(&historical, &reference);
    for (k, &d) in fixed.iter().enumerate() {
        assert!(
            d <= 1e-6,
            "default component {k} is {d:e} from the reference"
        );
    }
}

/// The reference shares no code with two outside measurements. The
/// tessellation is inscribed in the curved faces, so its volume approaches the
/// reference from below, its deficit falls at least linearly with deflection,
/// and a first-order Richardson extrapolation of two deflections lands on the
/// reference. Measured: deficits 33.58 at 0.05 and 6.98 at 0.01 (ratio 4.8),
/// extrapolation 6.5e-6 relative; vendor report 9.8e-5 relative.
#[test]
fn hammer_holder_adaptive_reference_agrees_with_mesh_and_vendor_volumes() {
    let (topo, solid) = load();
    let reference = reference(&topo, solid).mass;
    assert!(
        (reference - VENDOR_VOLUME).abs() <= VENDOR_VOLUME * 1e-3,
        "reference {reference} vs vendor {VENDOR_VOLUME}"
    );
    let coarse = mesh_volume(&topo, solid, 0.05);
    let fine = mesh_volume(&topo, solid, 0.01);
    let (coarse_deficit, fine_deficit) = (reference - coarse, reference - fine);
    assert!(
        fine_deficit > 0.0,
        "fine mesh {fine} above reference {reference}"
    );
    assert!(
        fine_deficit * 4.0 < coarse_deficit,
        "deficit must fall with deflection: {coarse_deficit} then {fine_deficit}"
    );
    let extrapolated = fine + (fine - coarse) / 4.0;
    let relative = (extrapolated - reference).abs() / reference;
    assert!(
        relative <= 1e-5,
        "Richardson {extrapolated} vs reference {reference}: {relative:e}"
    );
}
