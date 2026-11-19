//! B264: the ray vote and the boundary test on sphere faces whose trim holds a
//! pole of the sphere's own chart, winds its axis, or carries holes close to
//! a planar rim.
//!
//! The `(longitude, latitude)` chart reads none of these, and the old
//! fallbacks projected the chorded trim onto one plane: along its Newell
//! normal for a pole-holding cap bounded by a section curve, along the rim's
//! normal for the holes of a hemisphere. Neither projection bounds a strongly
//! curved patch, so rays through the far side of the piece took the wrong
//! vote. The classifier now reads such trims in a stereographic chart
//! re-centred clear of their loops (`boundary/sphere_stereo.rs`).
//!
//! Each case is a sphere against a cylinder whose material is known in
//! closed form: a bore offset from the sphere's axis (the sphere caps it
//! leaves hold a pole and are bounded by a non-planar curve winding the
//! axis; the band between them is bounded by two such curves), and a thin
//! cross bar through the upper hemisphere near its equator (two holes close
//! to the hemisphere's planar rim). Every operator runs at three scales,
//! unplaced and rigidly placed. Lattice probes at known clearance from both
//! surfaces must classify as the closed form says, through the one-shot and
//! the prepared classifier alike, and points on the kept sphere support must
//! read `OnBoundary` through both.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, PreparedSolid, classify_point};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean_with_context};
use remus_operations::primitives::{make_box, make_cylinder, make_sphere};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// A unit sphere at the origin against a tool solid, in the unit-scale frame.
#[derive(Clone, Copy, Debug)]
struct Case {
    name: &'static str,
    /// Builds the tool at `scale`, before placement.
    tool: fn(&mut Topology, f64) -> SolidId,
    /// Signed clearance of a unit-frame point from the tool: negative
    /// inside. Exact wherever the probes look.
    gap: fn([f64; 3]) -> f64,
    /// Indices into [`placements`] the exact boolean builds at.
    placements: &'static [usize],
}

/// Bore along `z`, radius 0.5, its axis 0.3 off the sphere's: it contains
/// the sphere's axis, so each cap it keeps holds a pole and is bounded by a
/// non-planar curve winding the axis.
const OFFSET_BORE: Case = Case {
    name: "offset bore",
    tool: |topo, s| {
        let bore = make_cylinder(topo, 0.5 * s, 3.0 * s).unwrap();
        transform_solid(topo, bore, &Mat4::translation(0.3 * s, 0.0, -1.5 * s)).unwrap();
        bore
    },
    gap: |[x, y, z]| ((x - 0.3).hypot(y) - 0.5).max(z.abs() - 1.5),
    placements: &[0, 1, 2, 3],
};

/// Box `[0.6, 1.5] x [-0.25, 0.25] x [0.08, 0.6]`: it notches the upper
/// hemisphere next to its rim, leaving one hole bounded by four arcs within
/// 0.08 of the equator.
const RIM_NOTCH: Case = Case {
    name: "rim notch",
    tool: |topo, s| {
        let notch = make_box(topo, 0.9 * s, 0.5 * s, 0.52 * s).unwrap();
        transform_solid(
            topo,
            notch,
            &Mat4::translation(0.6 * s, -0.25 * s, 0.08 * s),
        )
        .unwrap();
        notch
    },
    gap: |[x, y, z]| {
        // Signed distance to the box.
        let d = [
            (x - 1.05).abs() - 0.45,
            y.abs() - 0.25,
            (z - 0.34).abs() - 0.26,
        ];
        let outside = d.iter().map(|c| c.max(0.0).powi(2)).sum::<f64>().sqrt();
        outside + d[0].max(d[1]).max(d[2]).min(0.0)
    },
    // Under the seeds' rotation the box–sphere boolean refuses exact output
    // (`ExactOnlyUnattainable`), unrelated to classification.
    placements: &[0, 1, 2],
};

/// Unplaced, translated far from the origin, a steep rigid placement that
/// tilts the sphere's rim well off every coordinate plane, and the sphere–torus
/// seeds' rigid placement.
fn placements() -> [Mat4; 4] {
    [
        Mat4::identity(),
        Mat4::translation(17.0, -23.0, 31.0),
        Mat4::translation(-5.0, 9.0, -2.0) * Mat4::rotation_y(-0.7) * Mat4::rotation_x(1.2),
        Mat4::translation(17.0, -23.0, 31.0) * Mat4::rotation_x(0.37) * Mat4::rotation_z(-0.61),
    ]
}

/// Signed clearances `(sphere, tool)` of a unit-frame point: negative inside.
fn gaps(case: Case, point: [f64; 3]) -> (f64, f64) {
    let [x, y, z] = point;
    ((x * x + y * y + z * z).sqrt() - 1.0, (case.gap)(point))
}

fn member(op: BooleanOp, in_sphere: bool, in_cylinder: bool) -> bool {
    match op {
        BooleanOp::Fuse => in_sphere || in_cylinder,
        BooleanOp::Cut => in_sphere && !in_cylinder,
        BooleanOp::Intersect => in_sphere && in_cylinder,
    }
}

fn build(topo: &mut Topology, case: Case, op: BooleanOp, scale: f64, place: Mat4) -> SolidId {
    let sphere = make_sphere(topo, scale, 16).unwrap();
    let tool = (case.tool)(topo, scale);
    transform_solid(topo, sphere, &place).unwrap();
    transform_solid(topo, tool, &place).unwrap();
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    boolean_with_context(topo, op, sphere, tool, &context)
        .unwrap_or_else(|error| panic!("{} {op:?} scale {scale} {place:?}: {error:?}", case.name))
        .solid
}

fn check(case: Case, op: BooleanOp, scale: f64, placement: usize) {
    let label = format!("{} {op:?} scale={scale} placement={placement}", case.name);
    let place = placements()[placement];
    let mut topo = Topology::new();
    let solid = build(&mut topo, case, op, scale, place);
    let prepared = PreparedSolid::prepare(&topo, solid).unwrap();
    let options = ClassifyOptions {
        tolerance: 1e-7 * scale.max(1.0),
        ..ClassifyOptions::default()
    };
    let classify = |local: [f64; 3]| {
        let point = place.mul_point(Point3::new(
            local[0] * scale,
            local[1] * scale,
            local[2] * scale,
        ));
        let one_shot = classify_point(&topo, solid, point, &options).unwrap();
        let reused = prepared.classify_point(point, &options).unwrap();
        assert_eq!(
            one_shot, reused,
            "{label}: one-shot vs prepared at {local:?}"
        );
        one_shot
    };

    // Material probes at known clearance from both surfaces.
    let n = 9_u32;
    let mut counted = [0_usize; 2];
    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                let f = |step: u32| -1.3 + 2.6 * (f64::from(step) + 0.43) / f64::from(n);
                let local = [f(i), f(j), f(k)];
                let (sphere, tool) = gaps(case, local);
                if sphere.abs() < 0.03 || tool.abs() < 0.03 {
                    continue;
                }
                let inside = member(op, sphere < 0.0, tool < 0.0);
                counted[usize::from(inside)] += 1;
                let expected = if inside {
                    PointClassification::Inside
                } else {
                    PointClassification::Outside
                };
                assert_eq!(classify(local), expected, "{label}: probe {local:?}");
            }
        }
    }
    assert!(counted[0] > 20 && counted[1] > 0, "{label}: {counted:?}");

    // Points of the kept sphere support, and the same directions 0.05 in
    // and out along the normal.
    let mut on_support = 0;
    for a in 0..24_u32 {
        for b in 1..12_u32 {
            let azimuth = f64::from(a) * std::f64::consts::TAU / 24.0 + 0.1;
            let polar = f64::from(b) * std::f64::consts::PI / 12.0 + 0.05;
            let dir = [
                polar.sin() * azimuth.cos(),
                polar.sin() * azimuth.sin(),
                polar.cos(),
            ];
            let (_, tool) = gaps(case, dir);
            if tool.abs() < 0.03 {
                continue;
            }
            // The sphere surface is kept where the other operand's material
            // does (intersect) or does not (fuse, cut) hold it.
            let kept = if op == BooleanOp::Intersect {
                tool < 0.0
            } else {
                tool > 0.0
            };
            if !kept {
                continue;
            }
            on_support += 1;
            assert_eq!(
                classify(dir),
                PointClassification::OnBoundary,
                "{label}: sphere support {dir:?}"
            );
            for (factor, inside) in [(0.95, true), (1.05, false)] {
                let local = dir.map(|c| c * factor);
                let tool = (case.gap)(local);
                if tool.abs() < 0.03 {
                    continue;
                }
                let expected = if member(op, inside, tool < 0.0) {
                    PointClassification::Inside
                } else {
                    PointClassification::Outside
                };
                assert_eq!(
                    classify(local),
                    expected,
                    "{label}: beside support {local:?}"
                );
            }
        }
    }
    assert!(on_support > 0, "{label}: {on_support} support points");
}

fn matrix(case: Case) {
    for op in [BooleanOp::Fuse, BooleanOp::Cut, BooleanOp::Intersect] {
        for scale in [0.1, 1.0, 10.0] {
            for &placement in case.placements {
                check(case, op, scale, placement);
            }
        }
    }
}

#[test]
fn offset_bore_pole_caps_classify_exactly() {
    matrix(OFFSET_BORE);
}

#[test]
fn rim_notch_hole_classifies_exactly() {
    matrix(RIM_NOTCH);
}
