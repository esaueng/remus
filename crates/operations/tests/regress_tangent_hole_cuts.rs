//! Regression (esaueng/remus#1739): an exact cut of a vertical radius-3
//! cylinder (a drilled hole) from a stepped block, tangent to the step wall,
//! and from a block with an R4 edge fillet, across the fillet band.
//!
//! The outcome used to depend on where the tool's seam sat, so mirror-image
//! setups disagreed and holes a millimetre apart alternated between exact
//! and refused:
//!
//! - A hole tangent to the wall from the tall side leaves a wall of zero
//!   thickness along a line. One orientation came back `Exact`, touching
//!   itself along that line with every structural gate passing; its mirror
//!   was refused. Both are refused now, under either fallback policy.
//! - A hole on the low side touches the floor/wall corner at one point and
//!   is a solid. It was exact only when the seam was away from the touch
//!   point; both orientations are exact now, as is the band hole whose rim
//!   touches the fillet's spring line (x = 33).
//! - Holes across the band at x = 34 and 35 were refused over a rounding
//!   step in a section parameter, and the hole at x = 39, whose wall touches
//!   the spring line from the fillet side, because the fillet was never
//!   split there. They are exact now.
//!
//! Each exact result is checked against the closed-form or integrated
//! volume, the face census, strict validation, a watertight mesh at two
//! deflections, and ray-cast probes placed at the hole and beside the
//! contact.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::f64::consts::PI;

use remus_check::classify::{ClassifyOptions, classify_point};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::OperationsError;
use remus_operations::boolean::{
    BooleanOp, BooleanOutcome, BooleanQuality, boolean, boolean_with_context,
};
use remus_operations::copy::copy_and_transform_solid;
use remus_operations::primitives;
use remus_topology::Topology;
use remus_topology::explorer;
use remus_topology::solid::SolidId;

/// Volume agreement the issue reports for its exact results, in mm³.
const VOLUME_TOLERANCE: f64 = 0.01;

/// The tool: a radius-3 cylinder through the whole block at `(x, 10)`.
fn hole(topo: &mut Topology, x: f64) -> SolidId {
    let cylinder = primitives::make_cylinder(topo, 3.0, 30.0).unwrap();
    copy_and_transform_solid(topo, cylinder, &Mat4::translation(x, 10.0, -10.0)).unwrap()
}

/// A 40×20×10 block with a 5 mm step down at x = 20: tall for x < 20 when
/// `tall_left`, else tall for x > 20 (its mirror image).
fn stepped(topo: &mut Topology, tall_left: bool) -> SolidId {
    let block = primitives::make_box(topo, 40.0, 20.0, 10.0).unwrap();
    let notch = primitives::make_box(topo, 20.0, 20.0, 10.0).unwrap();
    let x = if tall_left { 20.0 } else { 0.0 };
    let notch = copy_and_transform_solid(topo, notch, &Mat4::translation(x, 0.0, 5.0)).unwrap();
    boolean(topo, BooleanOp::Cut, block, notch).unwrap()
}

/// A 40×20×10 block with an R4 fillet on its top edge at x = 40, so the
/// fillet band covers x in 36..40.
fn rounded(topo: &mut Topology) -> SolidId {
    let block = primitives::make_box(topo, 40.0, 20.0, 10.0).unwrap();
    let edge = explorer::solid_edges(topo, block)
        .unwrap()
        .into_iter()
        .find(|&edge| {
            let edge = topo.edge(edge).unwrap();
            [edge.start(), edge.end()].iter().all(|&vertex| {
                let p = topo.vertex(vertex).unwrap().point();
                (p.x() - 40.0).abs() < 1e-9 && (p.z() - 10.0).abs() < 1e-9
            })
        })
        .unwrap();
    remus_operations::blend_ops::fillet_cascade(topo, block, &[edge], 4.0)
        .unwrap()
        .solid
}

/// Volume of the rounded block less a hole at `cx`, the hole integrated on
/// midpoints under the block's top profile (the issue's own oracle).
fn band_volume(cx: f64) -> f64 {
    let top = |x: f64| {
        if x <= 36.0 {
            10.0
        } else if x <= 40.0 {
            6.0 + (16.0 - (x - 36.0).powi(2)).max(0.0).sqrt()
        } else {
            0.0
        }
    };
    let n = 200_000;
    let dx = 6.0 / f64::from(n);
    let removed: f64 = (0..n)
        .map(|i| {
            let x = cx - 3.0 + (f64::from(i) + 0.5) * dx;
            2.0 * (9.0 - (x - cx).powi(2)).max(0.0).sqrt() * top(x) * dx
        })
        .sum();
    8000.0 - 16.0 * (1.0 - PI / 4.0) * 20.0 - removed
}

fn exact_only() -> OperationContext {
    OperationContext::new().with_fallback(FallbackPolicy::ExactOnly)
}

fn cut(
    topo: &mut Topology,
    blank: SolidId,
    x: f64,
    context: &OperationContext,
) -> Result<BooleanOutcome, OperationsError> {
    let tool = hole(topo, x);
    boolean_with_context(topo, BooleanOp::Cut, blank, tool, context)
}

fn census(topo: &Topology, solid: SolidId) -> BTreeMap<&'static str, usize> {
    let mut census = BTreeMap::new();
    for face in explorer::solid_faces(topo, solid).unwrap() {
        *census
            .entry(topo.face(face).unwrap().surface().type_tag())
            .or_default() += 1;
    }
    census
}

/// An exact cut at `x` whose result is a solid of volume `expected` with
/// `cylinders` cylinder faces among fewer than twenty plane and cylinder
/// faces (so not a mesh fallback), and whose material is where the probes
/// say.
fn assert_exact(
    topo: &mut Topology,
    blank: SolidId,
    x: f64,
    expected: f64,
    cylinders: usize,
    probes: &[(Point3, &str)],
    label: &str,
) -> SolidId {
    let outcome = cut(topo, blank, x, &exact_only())
        .unwrap_or_else(|error| panic!("{label}: the cut must build exact, got {error}"));
    assert_eq!(outcome.quality, BooleanQuality::Exact, "{label}");
    let solid = outcome.solid;

    let census = census(topo, solid);
    assert_eq!(
        census.get("cylinder").copied().unwrap_or(0),
        cylinders,
        "{label}: cylinder faces in {census:?}"
    );
    assert!(
        census.values().sum::<usize>() < 20
            && census
                .keys()
                .all(|tag| matches!(*tag, "plane" | "cylinder")),
        "{label}: plane and cylinder faces expected, got {census:?}"
    );

    let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(
        report.issues.is_empty(),
        "{label}: strict validation must pass, got {:?}",
        report.issues
    );

    let volume = remus_operations::measure::solid_volume(topo, solid, 1e-4).unwrap();
    assert!(
        (volume - expected).abs() < VOLUME_TOLERANCE,
        "{label}: volume {volume:.4}, expected {expected:.4}"
    );

    for deflection in [0.005, 0.02] {
        let mesh = remus_operations::tessellate::tessellate_solid(topo, solid, deflection).unwrap();
        let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
        assert!(
            quality.is_watertight(),
            "{label}: mesh at deflection {deflection}: {quality:?}"
        );
    }

    let options = ClassifyOptions::default();
    for &(point, want) in probes {
        let got = format!(
            "{:?}",
            classify_point(topo, solid, point, &options).unwrap()
        );
        assert_eq!(got, want, "{label}: classify {point:?}");
    }
    solid
}

/// A cut at `x` with no solid result: refused by the exact-only policy, and
/// an error rather than an approximation when the fallback is allowed.
fn assert_refused(topo: &mut Topology, blank: SolidId, x: f64, label: &str) {
    match cut(topo, blank, x, &exact_only()) {
        Err(OperationsError::ExactOnlyUnattainable) => {}
        Ok(outcome) => panic!(
            "{label}: a zero-thickness cut came back {:?} with {:?}",
            outcome.quality,
            census(topo, outcome.solid)
        ),
        Err(error) => panic!("{label}: expected the exact-only refusal, got {error}"),
    }
    match cut(topo, blank, x, &OperationContext::new()) {
        Err(OperationsError::NonManifoldResult) => {}
        Ok(outcome) => panic!(
            "{label}: with the fallback allowed, a zero-thickness cut came back {:?} with {:?}",
            outcome.quality,
            census(topo, outcome.solid)
        ),
        Err(error) => panic!("{label}: expected the non-manifold refusal, got {error}"),
    }
}

/// A hole tangent to the step wall from the tall side leaves a wall of zero
/// thickness along the line (20, 10, 5..10). Neither mirror orientation may
/// return it as a solid.
#[test]
fn hole_tangent_to_the_step_wall_from_the_tall_side_is_refused_in_both_orientations() {
    let mut topo = Topology::new();
    let tall_left = stepped(&mut topo, true);
    let tall_right = stepped(&mut topo, false);
    assert_refused(&mut topo, tall_left, 17.0, "tall for x < 20, hole at 17");
    assert_refused(&mut topo, tall_right, 23.0, "tall for x > 20, hole at 23");
}

/// A hole on the low side touches the floor/wall corner at (20, 10, 5) and
/// nowhere else, which is a solid. Both mirror orientations build it exact.
#[test]
fn hole_touching_the_step_corner_at_a_point_is_exact_in_both_orientations() {
    let expected = 6000.0 - PI * 9.0 * 5.0;
    let mut topo = Topology::new();
    for (tall_left, x) in [(true, 23.0), (false, 17.0)] {
        let blank = stepped(&mut topo, tall_left);
        // Just inside the tall side from the touch point, and beside the
        // touch point between the hole and the wall plane (3.75 from the
        // hole axis).
        let (tall_x, crevice_x) = if tall_left {
            (19.5, 20.2)
        } else {
            (20.5, 19.8)
        };
        assert_exact(
            &mut topo,
            blank,
            x,
            expected,
            1,
            &[
                (Point3::new(x, 10.0, 2.5), "Outside"),
                (Point3::new(x, 10.0, 7.5), "Outside"),
                (Point3::new(x, 3.0, 2.5), "Inside"),
                (Point3::new(tall_x, 10.0, 7.5), "Inside"),
                (Point3::new(tall_x, 10.0, 2.5), "Inside"),
                (Point3::new(crevice_x, 12.5, 2.5), "Inside"),
            ],
            &format!("tall_left={tall_left}, hole at {x}"),
        );
    }
}

/// Below the operations gate: the engine writes a rim that touches its
/// face's outline as part of that outline in both orientations. With the
/// tool's seam on the touch point it used to emit the rim as a hole wire
/// sharing a vertex with the outer wire, one boundary component counted as
/// two, which is what the gate refused.
#[test]
fn rim_touching_a_face_outline_is_part_of_the_outline_in_both_orientations() {
    for (tall_left, x) in [(true, 23.0), (false, 17.0)] {
        let mut topo = Topology::new();
        let blank = stepped(&mut topo, tall_left);
        let tool = hole(&mut topo, x);
        let result =
            remus_algo::gfa::boolean(&mut topo, remus_algo::bop::BooleanOp::Cut, blank, tool)
                .unwrap();
        let touch = Point3::new(20.0, 10.0, 5.0);
        let mut floors = 0;
        for face_id in explorer::solid_faces(&topo, result).unwrap() {
            let face = topo.face(face_id).unwrap();
            let outer: Vec<_> = topo
                .wire(face.outer_wire())
                .unwrap()
                .edges()
                .iter()
                .map(|oriented| topo.edge(oriented.edge()).unwrap().start())
                .collect();
            for &inner in face.inner_wires() {
                for oriented in topo.wire(inner).unwrap().edges() {
                    let vertex = topo.edge(oriented.edge()).unwrap().start();
                    assert!(
                        !outer.contains(&vertex),
                        "tall_left={tall_left}: a hole wire shares a vertex with its outer wire"
                    );
                }
            }
            // The floor: the plane face at z = 5.
            let on_floor = outer
                .iter()
                .all(|&v| (topo.vertex(v).unwrap().point().z() - 5.0).abs() < 1e-9);
            if on_floor {
                floors += 1;
                assert!(
                    face.inner_wires().is_empty(),
                    "tall_left={tall_left}: the rim must be in the floor's outline"
                );
                let passes = outer
                    .iter()
                    .filter(|&&v| (topo.vertex(v).unwrap().point() - touch).length() < 1e-9)
                    .count();
                assert_eq!(
                    passes, 2,
                    "tall_left={tall_left}: the outline passes through the touch point twice"
                );
            }
        }
        assert_eq!(floors, 1, "tall_left={tall_left}: one floor face");
    }
}

/// A tenth of a millimetre off tangent the hole is clear of the wall, and
/// the cut is an ordinary through hole in the tall side.
#[test]
fn hole_a_tenth_off_tangent_is_exact_in_both_orientations() {
    let expected = 6000.0 - PI * 9.0 * 10.0;
    let mut topo = Topology::new();
    for (tall_left, x) in [(true, 16.9), (false, 23.1)] {
        let blank = stepped(&mut topo, tall_left);
        assert_exact(
            &mut topo,
            blank,
            x,
            expected,
            1,
            &[
                (Point3::new(x, 10.0, 7.5), "Outside"),
                (Point3::new(x, 10.0, 2.5), "Outside"),
                (Point3::new(x, 3.0, 7.5), "Inside"),
            ],
            &format!("tall_left={tall_left}, hole at {x}"),
        );
    }
}

/// Holes approaching and crossing the fillet band build exact at every
/// position but the wall tangency: clear of the band (30), rim touching the
/// fillet's spring line (33), across the spring line (34, 35, 36), through
/// the wall (38), and through the wall with the hole touching the spring
/// line from the fillet side (39).
#[test]
fn hole_across_the_fillet_band_is_exact() {
    let mut topo = Topology::new();
    let blank = rounded(&mut topo);
    // Cylinder faces: the fillet and the hole wall; at 38 and 39 the hole
    // breaks through the x = 40 wall and splits the fillet in two (at 39 the
    // two pieces still meet at the touch point).
    for (x, cylinders) in [
        (30.0, 2),
        (33.0, 2),
        (34.0, 2),
        (35.0, 2),
        (36.0, 2),
        (38.0, 3),
        (39.0, 3),
    ] {
        assert_exact(
            &mut topo,
            blank,
            x,
            band_volume(x),
            cylinders,
            &[
                (Point3::new(x, 10.0, 3.0), "Outside"),
                (Point3::new(x - 3.5, 10.0, 5.0), "Inside"),
                (Point3::new(x, 14.0, 5.0), "Inside"),
                (Point3::new(x, 6.0, 5.0), "Inside"),
                // Under the top face just short of the spring line: inside
                // every hole from 33 to 38, material at 30 and at 39.
                (
                    Point3::new(35.9, 10.0, 9.9),
                    if (33.0..=38.0).contains(&x) {
                        "Outside"
                    } else {
                        "Inside"
                    },
                ),
            ],
            &format!("band hole at {x}"),
        );
    }
}

/// At x = 37 the hole is tangent to the x = 40 wall from inside, below the
/// fillet: zero thickness along (40, 10, 0..6). Refused like the step wall.
#[test]
fn hole_tangent_to_the_wall_below_the_fillet_is_refused() {
    let mut topo = Topology::new();
    let blank = rounded(&mut topo);
    assert_refused(&mut topo, blank, 37.0, "band hole at 37");
}
