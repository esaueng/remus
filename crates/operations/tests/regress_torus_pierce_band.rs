//! A torus tube that enters a tool through one face and leaves through
//! another (a cap plane through the torus axis, an oblique cone wall) yields two
//! CLOSED tube-wrapping sections. The kept torus is the band between them, not
//! the full periodic face with two "holes": that form passes every topology
//! gate, meshes with the holes skinned over, and both volume routes misread it
//! (fuzz `modifier_ops` crash-71dbb118, Fuzz Smoke run 35180928629, 2026-09-17).
#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use remus_math::mat::Mat4;
use remus_operations::blend_ops::fillet_v2;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{mass_properties, solid_bounding_box, solid_volume};
use remus_operations::primitives::{make_cone, make_torus};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::wire::WireId;

const MAJOR: f64 = 4.0;
const MINOR: f64 = 0.5;
const CONE_R0: f64 = 8.0;
const CONE_R1: f64 = 5.5;
const CONE_H: f64 = 5.0;

/// Torus in XY at the origin, fused with a frustum whose axis runs along +X
/// from x = 0 (radius 8) to x = 5 (radius 5.5) at y = z = -4. The tube enters
/// through the base cap (a plane through the torus axis) and leaves through the
/// cone wall.
fn build(topo: &mut Topology) -> SolidId {
    let torus = make_torus(topo, MAJOR, MINOR, 16).unwrap();
    let cone = make_cone(topo, CONE_R0, CONE_R1, CONE_H).unwrap();
    let place = Mat4::translation(0.0, -4.0, -4.0) * Mat4::rotation_y(FRAC_PI_2);
    transform_solid(topo, cone, &place).unwrap();
    boolean(topo, BooleanOp::Fuse, torus, cone).unwrap()
}

/// Closed-form pieces plus the tube-inside-frustum overlap by the midpoint rule
/// in torus coordinates. Converges to 738.30 (3600×720×120 grid); the coarse
/// grid here is within 0.05 of that.
fn expected_volume() -> f64 {
    let (nu, nv, nr) = (720_usize, 240_usize, 48_usize);
    let mut overlap = 0.0;
    for i in 0..nu {
        let u = TAU * (i as f64 + 0.5) / nu as f64;
        for j in 0..nv {
            let v = TAU * (j as f64 + 0.5) / nv as f64;
            for k in 0..nr {
                let rho = MINOR * (k as f64 + 0.5) / nr as f64;
                let ring = MAJOR + rho * v.cos();
                let (x, y, z) = (ring * u.cos(), ring * u.sin(), rho * v.sin());
                if (0.0..=CONE_H).contains(&x) {
                    let radial = ((y + 4.0).powi(2) + (z + 4.0).powi(2)).sqrt();
                    if radial <= CONE_R0 - (CONE_R0 - CONE_R1) * x / CONE_H {
                        overlap += ring * rho;
                    }
                }
            }
        }
    }
    overlap *= (TAU / nu as f64) * (TAU / nv as f64) * (MINOR / nr as f64);
    let torus = 2.0 * PI * PI * MAJOR * MINOR * MINOR;
    let frustum = PI * CONE_H * (CONE_R0 * CONE_R0 + CONE_R0 * CONE_R1 + CONE_R1 * CONE_R1) / 3.0;
    torus + frustum - overlap
}

/// Net tube-angle traversal of a wire on its torus, in turns.
fn tube_turns(topo: &Topology, wire: WireId, torus: &remus_math::surfaces::ToroidalSurface) -> f64 {
    let mut phis = Vec::new();
    for oe in topo.wire(wire).unwrap().edges() {
        let e = topo.edge(oe.edge()).unwrap();
        let (a, b) = e.strict_domain().unwrap();
        let (s, t) = (
            topo.vertex(e.start()).unwrap().point(),
            topo.vertex(e.end()).unwrap().point(),
        );
        let mut pts: Vec<f64> = (0..32)
            .map(|k| {
                let p = e
                    .curve()
                    .evaluate_with_endpoints(a + (b - a) * f64::from(k) / 32.0, s, t);
                torus.project_point(p).1
            })
            .collect();
        if !oe.is_forward() {
            pts.reverse();
        }
        phis.extend(pts);
    }
    let mut acc = 0.0;
    for i in 0..phis.len() {
        let d = phis[(i + 1) % phis.len()] - phis[i];
        acc += (d + PI).rem_euclid(TAU) - PI;
    }
    acc / TAU
}

/// The fuzz harness meshes at four times its volume deflection (~0.004 here).
fn harness_deflection(topo: &Topology, solid: SolidId) -> f64 {
    let aabb = solid_bounding_box(topo, solid).unwrap();
    ((aabb.max - aabb.min).length() * 4e-5).max(1e-7) * 4.0
}

fn assert_watertight(topo: &Topology, solid: SolidId, what: &str, deflections: &[f64]) {
    for &d in deflections {
        let mesh = tessellate_solid(topo, solid, d).unwrap();
        let (b, n) = (boundary_edge_count(&mesh), non_manifold_edge_count(&mesh));
        assert!(
            b == 0 && n == 0,
            "{what}: mesh at deflection {d} has {b} boundary and {n} non-manifold edges"
        );
    }
}

#[test]
fn fused_torus_keeps_one_band_between_the_pierce_loops() {
    let mut topo = Topology::new();
    let solid = build(&mut topo);
    assert!(validate_solid(&topo, solid).unwrap().is_valid());

    let tori: Vec<_> = explorer::solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .collect();
    assert_eq!(tori.len(), 1, "one torus band face expected");
    let face = topo.face(tori[0]).unwrap();
    let FaceSurface::Torus(torus) = face.surface() else {
        unreachable!()
    };
    assert_eq!(
        face.inner_wires().len(),
        1,
        "the band is bounded by its two pierce loops, one outer and one inner"
    );
    let outer = tube_turns(&topo, face.outer_wire(), torus);
    let inner = tube_turns(&topo, face.inner_wires()[0], torus);
    assert!(
        (outer.abs() - 1.0).abs() < 1e-3 && (inner.abs() - 1.0).abs() < 1e-3,
        "both loops must wrap the tube once: outer {outer} turns, inner {inner} turns"
    );
    assert!(
        outer * inner < 0.0,
        "complementary rims traverse the tube in opposite senses: {outer} vs {inner}"
    );
}

#[test]
fn fused_torus_measures_and_meshes_correctly() {
    let mut topo = Topology::new();
    let solid = build(&mut topo);
    let harness = harness_deflection(&topo, solid);
    assert_watertight(&topo, solid, "fuse", &[0.1, 0.01, harness]);

    let expected = expected_volume();
    let aabb = solid_bounding_box(&topo, solid).unwrap();
    let d = ((aabb.max - aabb.min).length() * 4e-5).max(1e-7);
    let mesh_volume = solid_volume(&topo, solid, d).unwrap();
    let exact_volume = mass_properties(&topo, solid).unwrap().mass;
    // The skinned-over form read 743.58 / 742.66 against 738.30; a 0.2 % band
    // (about 1.5) separates the two outcomes by a factor of three.
    for (name, v) in [
        ("solid_volume", mesh_volume),
        ("mass_properties", exact_volume),
    ] {
        assert!(
            ((v - expected) / expected).abs() < 2e-3,
            "{name} {v} vs expected {expected}"
        );
    }
}

/// The fuzz case's actual assertion: a small fillet on the frustum's base rim
/// must leave the solid watertight at the harness deflection. The rim is far
/// from the torus; the fillet only inherits the base body's mesh.
#[test]
fn fillet_on_base_rim_stays_watertight() {
    let mut topo = Topology::new();
    let solid = build(&mut topo);
    let edges = explorer::solid_edges(&topo, solid).unwrap();
    let result = fillet_v2(&mut topo, solid, &[edges[0]], 0.05).unwrap();
    assert!(!result.is_partial);
    let harness = harness_deflection(&topo, result.solid);
    assert_watertight(&topo, result.solid, "fillet", &[harness, 0.001]);
}

/// B38 witness: the same 0.05 fillet band meshes open at coarser deflections.
/// At 0.1 (twice the band radius) the band face degenerates (37 boundary, 8
/// non-manifold edges); at 0.01 its rim is sampled one segment differently from
/// the holed cone wall it meets (63 boundary edges). The fuse itself is clean
/// at these deflections (see [`fused_torus_measures_and_meshes_correctly`]).
#[test]
#[ignore = "B38: blend band on a rim shared with a holed cone wall meshes open at deflection 0.1 and 0.01"]
fn fillet_on_base_rim_stays_watertight_at_coarse_deflection() {
    let mut topo = Topology::new();
    let solid = build(&mut topo);
    let edges = explorer::solid_edges(&topo, solid).unwrap();
    let result = fillet_v2(&mut topo, solid, &[edges[0]], 0.05).unwrap();
    assert!(!result.is_partial);
    assert_watertight(&topo, result.solid, "fillet", &[0.1, 0.01]);
}
