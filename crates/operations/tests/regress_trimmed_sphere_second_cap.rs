//! Regression (#1816): a sphere already trimmed by one box keeps its trim
//! when a second box cuts the other cap off, or intersects it.
//!
//! A radius-2 sphere cut by a box covering everything above `z = 1`, then by
//! one covering everything below `z = -1`, is the slab between the planes.
//! The second cut dropped the disc closing the slab at `z = -1`, the part of
//! the lower box's top face inside the trimmed sphere: the GFA classifies
//! that disc by ray casting against the trimmed sphere, and the ray caster
//! had no model of a sphere face, so it flattened each one into its boundary
//! polygon. The lower hemisphere became the equatorial disc and every point
//! inside the trimmed sphere read as outside. The result had a free edge and
//! the exact-only boolean refused it. The ray caster now tests ray/sphere
//! roots against the face's circular-arc trim.
//!
//! Every oracle is a closed form: a cap of height `h` has volume
//! `πh²(3r − h)/3`.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_check::classify::PointClassification::{self, Inside, Outside};
use remus_math::compact_sort::CompactSort;
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::{make_box, make_sphere};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const R: f64 = 2.0;
/// Edge of the cutting boxes, far larger than the sphere.
const SIDE: f64 = 20.0;

fn cap_volume(h: f64) -> f64 {
    PI * h * h * (3.0 * R - h) / 3.0
}

fn sphere_volume() -> f64 {
    4.0 * PI * R.powi(3) / 3.0
}

/// A box covering everything above `z` (when `above`) or below it.
fn half_space_box(topo: &mut Topology, z: f64, above: bool) -> SolidId {
    let block = make_box(topo, SIDE, SIDE, SIDE).unwrap();
    let floor = if above { z } else { z - SIDE };
    transform_solid(
        topo,
        block,
        &Mat4::translation(-SIDE / 2.0, -SIDE / 2.0, floor),
    )
    .unwrap();
    block
}

fn assert_rel(got: f64, want: f64, what: &str) {
    assert!(
        (got - want).abs() <= 1e-9 * want.abs().max(1.0),
        "{what}: {got} vs closed form {want}"
    );
}

/// The verification ladder for a sphere zone between `z = lo` and `z = hi`
/// (each bound inside the sphere, or at a pole): the exact sphere and planes,
/// strictly valid, a closed watertight mesh, material exactly between the
/// planes, and the zone's closed-form volume.
fn verify_zone(topo: &Topology, body: SolidId, lo: f64, hi: f64, what: &str) {
    // Census: only the original sphere and the cutting planes.
    let mut discs = Vec::new();
    for f in solid_faces(topo, body).unwrap() {
        match topo.face(f).unwrap().surface() {
            FaceSurface::Sphere(s) => {
                assert_rel(s.radius(), R, &format!("{what}: sphere radius"));
                assert!(
                    (s.center() - Point3::new(0.0, 0.0, 0.0)).length() <= 1e-12,
                    "{what}: sphere centre {:?}",
                    s.center()
                );
            }
            FaceSurface::Plane { normal, d } => {
                assert_rel(normal.z().abs(), 1.0, &format!("{what}: disc normal"));
                discs.push(d * normal.z());
            }
            other => panic!("{what}: unexpected {} face", other.type_tag()),
        }
    }
    discs.compact_sort_by(f64::total_cmp);
    let mut want_discs = Vec::new();
    if lo > -R {
        want_discs.push(lo);
    }
    if hi < R {
        want_discs.push(hi);
    }
    assert_eq!(discs.len(), want_discs.len(), "{what}: discs {discs:?}");
    for (got, want) in discs.iter().zip(&want_discs) {
        assert_rel(*got, *want, &format!("{what}: disc plane"));
    }

    // Validity: a closed manifold shell, by both validators.
    let report = remus_operations::validate::validate_solid(topo, body).unwrap();
    assert!(report.is_valid(), "{what}: {:?}", report.issues);
    let strict = remus_check::validate::validate_solid(
        topo,
        body,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(strict.is_valid(), "{what}: {:?}", strict.issues);

    // A closed manifold mesh at two deflections.
    for deflection in [1e-3, 2e-4] {
        let mesh = tessellate_solid(topo, body, deflection).unwrap();
        assert_eq!(
            boundary_edge_count(&mesh),
            0,
            "{what}: open mesh at {deflection}"
        );
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "{what}: non-manifold mesh at {deflection}"
        );
    }

    // Material exactly between the planes and inside the sphere.
    let mid = 0.5 * (lo + hi);
    let probes: [(Point3, PointClassification); 6] = [
        (Point3::new(0.0, 0.0, mid), Inside),
        (Point3::new(0.3, -0.2, (hi - lo).mul_add(0.05, lo)), Inside),
        (Point3::new(-0.2, 0.3, (hi - lo).mul_add(-0.05, hi)), Inside),
        (Point3::new(0.0, 0.0, lo - 0.05), Outside),
        (Point3::new(0.0, 0.0, hi + 0.05), Outside),
        (Point3::new(0.0, R + 0.05, mid), Outside),
    ];
    for (probe, expected) in probes {
        let got = remus_check::classify::classify_point(
            topo,
            body,
            probe,
            &remus_check::classify::ClassifyOptions::default(),
        )
        .unwrap();
        assert_eq!(got, expected, "{what}: probe {probe:?}");
    }

    // Volume: the sphere less the caps above `hi` and below `lo`.
    let want = sphere_volume() - cap_volume(R - hi) - cap_volume(R + lo);
    assert_rel(
        mass_properties(topo, body).unwrap().mass,
        want,
        &format!("{what}: mass_properties"),
    );
    assert_rel(
        solid_volume(topo, body, 1e-4).unwrap(),
        want,
        &format!("{what}: solid_volume"),
    );
}

/// The issue's repro and its variants: the second cut from below (top cap
/// first) and from above (bottom cap first), at several sphere segment
/// counts and at symmetric and lopsided heights. Each builds the slab.
#[test]
fn second_cut_of_a_trimmed_sphere_builds_the_slab() {
    for segments in [4, 16, 32] {
        for (lo, hi) in [(-1.0, 1.0), (-0.5, 1.5), (-1.5, 0.25)] {
            for top_first in [true, false] {
                let what = format!(
                    "{segments} segments, slab {lo}..{hi}, {} cap first",
                    if top_first { "top" } else { "bottom" }
                );
                let mut topo = Topology::new();
                let sphere = make_sphere(&mut topo, R, segments).unwrap();
                let order = if top_first {
                    [(hi, true), (lo, false)]
                } else {
                    [(lo, false), (hi, true)]
                };
                let mut body = sphere;
                for (z, above) in order {
                    let tool = half_space_box(&mut topo, z, above);
                    body = boolean(&mut topo, BooleanOp::Cut, body, tool)
                        .unwrap_or_else(|e| panic!("{what}: cut at z = {z}: {e:?}"));
                }
                verify_zone(&topo, body, lo, hi, &what);
            }
        }
    }
}

/// The second operation as an intersect: the trimmed sphere intersected with
/// a box covering everything above `lo` is the same slab, and with a box
/// covering everything below `lo` it is the cap the cut removes.
#[test]
fn trimmed_sphere_intersect_keeps_the_slab_or_the_cap() {
    for segments in [4, 16, 32] {
        for (lo, hi) in [(-1.0, 1.0), (-0.5, 1.5)] {
            for keep_slab in [true, false] {
                let what = format!(
                    "{segments} segments, trim above {hi}, intersect {} {lo}",
                    if keep_slab { "above" } else { "below" }
                );
                let mut topo = Topology::new();
                let sphere = make_sphere(&mut topo, R, segments).unwrap();
                let top = half_space_box(&mut topo, hi, true);
                let trimmed = boolean(&mut topo, BooleanOp::Cut, sphere, top).unwrap();
                let tool = half_space_box(&mut topo, lo, keep_slab);
                let body = boolean(&mut topo, BooleanOp::Intersect, trimmed, tool)
                    .unwrap_or_else(|e| panic!("{what}: {e:?}"));
                let (zone_lo, zone_hi) = if keep_slab { (lo, hi) } else { (-R, lo) };
                verify_zone(&topo, body, zone_lo, zone_hi, &what);
            }
        }
    }
}

/// The GFA's own point classifier reads the trimmed sphere's interior as
/// inside: every probe on the axis between the caps, and none beyond them.
#[test]
fn gfa_classifier_reads_a_trimmed_sphere_interior() {
    for segments in [4, 16, 32] {
        let mut topo = Topology::new();
        let sphere = make_sphere(&mut topo, R, segments).unwrap();
        let top = half_space_box(&mut topo, 1.0, true);
        let trimmed = boolean(&mut topo, BooleanOp::Cut, sphere, top).unwrap();
        for (z, inside) in [
            (-1.9, true),
            (-1.5, true),
            (-1.0, true),
            (0.0, true),
            (0.5, true),
            (0.95, true),
            (1.05, false),
            (-2.05, false),
        ] {
            let class =
                remus_algo::classifier::classify_point(&topo, trimmed, Point3::new(0.1, -0.1, z))
                    .unwrap();
            let want = if inside {
                remus_algo::FaceClass::Inside
            } else {
                remus_algo::FaceClass::Outside
            };
            assert_eq!(class, want, "{segments} segments, z = {z}");
        }
    }
}
