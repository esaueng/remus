//! A torus tube that enters a tool through one face and leaves through
//! another (a cap plane through the torus axis, an oblique cone wall) yields two
//! CLOSED tube-wrapping sections. The kept torus is the band between them, not
//! the full periodic face with two "holes": that form passes every topology
//! gate, meshes with the holes skinned over, and both volume routes misread it
//! (fuzz `modifier_ops` crash-71dbb118, Fuzz Smoke run 35180928629, 2026-09-17).
#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
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

/// Where the frustum sits: its local axis (+z, base at z = 0) mapped onto +X.
fn placement() -> Mat4 {
    Mat4::translation(0.0, -4.0, -4.0) * Mat4::rotation_y(FRAC_PI_2)
}

/// Torus in XY at the origin, fused with a frustum whose axis runs along +X
/// from x = 0 (radius 8) to x = 5 (radius 5.5) at y = z = -4. The tube enters
/// through the base cap (a plane through the torus axis) and leaves through the
/// cone wall.
fn build(topo: &mut Topology) -> SolidId {
    build_op(topo, BooleanOp::Fuse, false)
}

/// The same operands under any boolean; `frustum_first` swaps them.
fn build_op(topo: &mut Topology, op: BooleanOp, frustum_first: bool) -> SolidId {
    let torus = make_torus(topo, MAJOR, MINOR, 16).unwrap();
    let cone = make_cone(topo, CONE_R0, CONE_R1, CONE_H).unwrap();
    transform_solid(topo, cone, &placement()).unwrap();
    if frustum_first {
        boolean(topo, op, cone, torus).unwrap()
    } else {
        boolean(topo, op, torus, cone).unwrap()
    }
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

/// B46 witness (fixed): the same 0.05 fillet band used to mesh open at coarse
/// deflections — at 0.1 the band face degenerated (37 boundary, 8 non-manifold
/// edges) and at 0.01 its rim sampled one segment differently from the holed
/// cone wall it meets (63 boundary edges). The shared pool sampled circles
/// floor-free while the torus two-rim band sized its interior rows with the
/// curvature floor, so the sparse rims stitched against far denser rows. The
/// solid-level rim resampling now densifies torus band rims to the mesher's
/// own wrap density (proportionally by arc for split rims). The fuse itself
/// was always clean at these deflections (see
/// [`fused_torus_measures_and_meshes_correctly`]).
#[test]
fn fillet_on_base_rim_stays_watertight_at_coarse_deflection() {
    let mut topo = Topology::new();
    let solid = build(&mut topo);
    let edges = explorer::solid_edges(&topo, solid).unwrap();
    let result = fillet_v2(&mut topo, solid, &[edges[0]], 0.05).unwrap();
    assert!(!result.is_partial);
    assert_watertight(&topo, result.solid, "fillet", &[0.1, 0.01]);
}

/// The ray-cast classifier against analytic membership on a grid around the
/// band, for every boolean of the two operands.
///
/// The band's two loops each wrap the tube, so projected to the torus `(u, v)`
/// domain neither bounds a polygon: the fused band rejected every ray hit and
/// 843 of 7062 grid points (every one inside the tube and outside the frustum)
/// read `Outside`. The classifier is the ground truth the verification skills
/// reach for, so it has to agree here before it can vouch for anything else.
#[test]
fn ray_cast_classifier_agrees_with_analytic_membership() {
    let inverse = placement().inverse().unwrap();
    // Distance to the tube surface, and signed depth inside the frustum (its
    // radial term is measured square to the axis, so it over-states the
    // distance to the slanted wall slightly: a stricter skip, never a looser).
    let tube = |p: Point3| (p.x().hypot(p.y()) - MAJOR).hypot(p.z()) - MINOR;
    let frustum = |p: Point3| {
        let q = inverse.mul_point(p);
        let radius = CONE_R0 + (CONE_R1 - CONE_R0) * q.z() / CONE_H;
        q.z().min(CONE_H - q.z()).min(radius - q.x().hypot(q.y()))
    };
    let options = ClassifyOptions::default();
    for (op, frustum_first) in [
        (BooleanOp::Fuse, false),
        (BooleanOp::Cut, false),
        (BooleanOp::Cut, true),
        (BooleanOp::Intersect, false),
    ] {
        let mut topo = Topology::new();
        let solid = build_op(&mut topo, op, frustum_first);
        let (mut probed, mut wrong) = (0, Vec::new());
        let (nx, ny, nz) = (30_u32, 30_u32, 8_u32);
        for i in 0..nx {
            for j in 0..ny {
                for k in 0..nz {
                    let along = |lo: f64, hi: f64, s: u32, n: u32| {
                        lo + (hi - lo) * f64::from(s) / f64::from(n - 1)
                    };
                    let p = Point3::new(
                        along(-4.6, 4.6, i, nx),
                        along(-4.6, 4.6, j, ny),
                        along(-0.6, 0.6, k, nz),
                    );
                    let (t, f) = (tube(p), frustum(p));
                    if t.abs() < 0.02 || f.abs() < 0.02 {
                        continue;
                    }
                    let (in_torus, in_frustum) = (t < 0.0, f > 0.0);
                    let (a, b) = if frustum_first {
                        (in_frustum, in_torus)
                    } else {
                        (in_torus, in_frustum)
                    };
                    let expected = match op {
                        BooleanOp::Fuse => a || b,
                        BooleanOp::Cut => a && !b,
                        BooleanOp::Intersect => a && b,
                    };
                    probed += 1;
                    let got = classify_point(&topo, solid, p, &options).unwrap();
                    let agrees = match got {
                        PointClassification::Inside => expected,
                        PointClassification::Outside => !expected,
                        // Every probe is at least 0.02 clear of both surfaces.
                        PointClassification::OnBoundary => false,
                    };
                    if !agrees {
                        wrong.push((p, got));
                    }
                }
            }
        }
        assert!(probed > 7000, "{op:?}: only {probed} probes");
        assert!(
            wrong.is_empty(),
            "{op:?} (frustum first: {frustum_first}): {} of {probed} probes misread, \
             e.g. {:?}",
            wrong.len(),
            &wrong[..wrong.len().min(4)]
        );
    }
}
