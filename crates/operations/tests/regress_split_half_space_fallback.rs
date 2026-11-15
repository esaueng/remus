//! Regression for #1745: `split(plane)` refused whenever the plane crossed a
//! curved edge, a non-planar face, or a face's inner wire, while a pair of
//! exact half-space booleans produced both halves of the same cut.
//!
//! Every case of the issue's repro is reproduced here at the Rust level:
//!
//!   * a plain R10 x 20 cylinder cut through and parallel to its axis;
//!   * a 60 x 40 x 30 box with R5 vertical rounds, cut across the rounds
//!     (horizontal) and through two of them (vertical);
//!   * that box's 2 mm open-top shell, plain and rounded, cut through its rim
//!     (where the rim is an inner wire of the side walls) and across it.
//!
//! Each split must now return both halves, and each half must:
//!   * be a valid solid of positive volume, on the side of the plane the
//!     contract names (positive = the side the normal points toward);
//!   * match the exact half-space boolean of the same cut;
//!   * keep the input's analytic surfaces (no NURBS where there was none,
//!     and the cylindrical faces still cylinders);
//!
//! The two halves must also add back up to the input.
//!
//! The planar-face controls of the repro keep their native results, and the
//! planes off the coordinate axes check that the fallback is not limited to
//! axis-aligned cuts. A cut neither route can build exactly (an elliptical
//! section) stays a typed refusal that leaves the session untouched.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::split::{SplitResult, split, split_with_evolution};
use remus_operations::transform::transform_solid;
use remus_operations::{OperationsError, measure, validate};
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

/// `solid_volume` integrates analytic surfaces in closed form; this only
/// bounds the sampled paths.
const DEFLECTION: f64 = 0.01;

/// Relative agreement required between a split half and the reference
/// boolean half, and between a half and a closed form. Both routes are exact
/// constructions over the same analytic surfaces.
const RELATIVE: f64 = 1e-9;

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    measure::solid_volume(topo, solid, DEFLECTION).expect("volume")
}

fn assert_relative(what: &str, actual: f64, expected: f64) {
    let err = (actual - expected).abs() / expected.abs().max(1.0);
    assert!(
        err <= RELATIVE,
        "{what}: got {actual}, expected {expected} (relative error {err:e})"
    );
}

fn assert_valid(topo: &Topology, solid: SolidId, what: &str) {
    let report = validate::validate_solid(topo, solid).expect("validate");
    let errors: Vec<_> = report
        .issues
        .iter()
        .filter(|i| i.severity == validate::Severity::Error)
        .map(|i| i.description.clone())
        .collect();
    assert!(errors.is_empty(), "{what} is not a valid solid: {errors:?}");
}

#[derive(Default, Debug, PartialEq, Eq, Clone, Copy)]
struct SurfaceCensus {
    plane: usize,
    cylinder: usize,
    nurbs: usize,
    other: usize,
}

fn census(topo: &Topology, solid: SolidId) -> SurfaceCensus {
    let mut c = SurfaceCensus::default();
    for f in solid_faces(topo, solid).unwrap() {
        match topo.face(f).unwrap().surface() {
            FaceSurface::Plane { .. } => c.plane += 1,
            FaceSurface::Cylinder(_) => c.cylinder += 1,
            FaceSurface::Nurbs(_) => c.nurbs += 1,
            _ => c.other += 1,
        }
    }
    c
}

/// Signed distance range of a solid's vertices from the plane.
fn vertex_range(topo: &Topology, solid: SolidId, point: Point3, normal: Vec3) -> (f64, f64) {
    let n = normal.normalize().unwrap();
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for e in solid_edges(topo, solid).unwrap() {
        let edge = topo.edge(e).unwrap();
        for v in [edge.start(), edge.end()] {
            let s = n.dot(topo.vertex(v).unwrap().point() - point);
            lo = lo.min(s);
            hi = hi.max(s);
        }
    }
    (lo, hi)
}

/// The exact half-space reference: intersect and cut against a box that
/// covers everything on the positive side of the plane.
fn reference_halves(
    topo: &mut Topology,
    solid: SolidId,
    point: Point3,
    normal: Vec3,
) -> (f64, f64) {
    let n = normal.normalize().unwrap();
    let seed = if n.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u = n.cross(seed).normalize().unwrap();
    let v = n.cross(u);
    let span = 500.0;
    let corner = point - (u + v) * span;
    let mut volumes = [0.0; 2];
    for (slot, op) in [BooleanOp::Intersect, BooleanOp::Cut]
        .into_iter()
        .enumerate()
    {
        let tool = make_box(topo, 2.0 * span, 2.0 * span, 2.0 * span).unwrap();
        transform_solid(
            topo,
            tool,
            &Mat4([
                [u.x(), v.x(), n.x(), corner.x()],
                [u.y(), v.y(), n.y(), corner.y()],
                [u.z(), v.z(), n.z(), corner.z()],
                [0.0, 0.0, 0.0, 1.0],
            ]),
        )
        .unwrap();
        let half = boolean(topo, op, solid, tool).expect("reference half-space boolean");
        volumes[slot] = volume(topo, half);
    }
    (volumes[0], volumes[1])
}

/// Split, then hold the halves to every check this file promises.
fn check_split(
    topo: &mut Topology,
    solid: SolidId,
    point: Point3,
    normal: Vec3,
    what: &str,
) -> SplitResult {
    let input_volume = volume(topo, solid);
    let input_census = census(topo, solid);
    let (ref_pos, ref_neg) = reference_halves(topo, solid, point, normal);

    let halves =
        split(topo, solid, point, normal).unwrap_or_else(|e| panic!("{what}: split refused: {e}"));

    for (half, side) in [(halves.positive, "positive"), (halves.negative, "negative")] {
        assert_valid(topo, half, &format!("{what}: {side} half"));
    }
    let pos = volume(topo, halves.positive);
    let neg = volume(topo, halves.negative);
    assert!(pos > 0.0 && neg > 0.0, "{what}: halves {pos} + {neg}");
    assert_relative(&format!("{what}: halves add up"), pos + neg, input_volume);
    assert_relative(&format!("{what}: positive matches intersect"), pos, ref_pos);
    assert_relative(&format!("{what}: negative matches cut"), neg, ref_neg);

    // Each half lies on its own side of the plane.
    let eps = 1e-6 * input_volume.cbrt().max(1.0);
    let (lo, _) = vertex_range(topo, halves.positive, point, normal);
    assert!(
        lo >= -eps,
        "{what}: positive half reaches {lo} below the plane"
    );
    let (_, hi) = vertex_range(topo, halves.negative, point, normal);
    assert!(
        hi <= eps,
        "{what}: negative half reaches {hi} above the plane"
    );

    // No analytic surface was traded for a NURBS or mesh stand-in.
    for (half, side) in [(halves.positive, "positive"), (halves.negative, "negative")] {
        let c = census(topo, half);
        assert_eq!(
            c.nurbs, input_census.nurbs,
            "{what}: {side} half gained NURBS faces: {c:?} from {input_census:?}"
        );
        assert_eq!(
            c.other, 0,
            "{what}: {side} half has an unexpected surface: {c:?}"
        );
    }
    halves
}

/// The repro's 60 x 40 x 30 box, centred on the z axis with its base at z = 0.
fn box_60_40_30(topo: &mut Topology) -> SolidId {
    let b = make_box(topo, 60.0, 40.0, 30.0).unwrap();
    transform_solid(topo, b, &Mat4::translation(-30.0, -20.0, 0.0)).unwrap();
    b
}

/// The box with its four vertical edges rounded R5.
fn rounded_box(topo: &mut Topology) -> SolidId {
    let b = box_60_40_30(topo);
    let vertical: Vec<_> = solid_edges(topo, b)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let z = topo.vertex(edge.end()).unwrap().point();
            (a.z() - z.z()).abs() > 10.0
        })
        .collect();
    assert_eq!(vertical.len(), 4);
    remus_operations::blend_ops::fillet_cascade(topo, b, &vertical, 5.0)
        .expect("R5 vertical rounds")
        .solid
}

/// The planar face with outward normal +z at z = 30.
fn top_face(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| {
            let face = topo.face(f).unwrap();
            face.effective_plane_normal()
                .is_some_and(|n| n.z() > 1.0 - 1e-9)
                && topo
                    .wire(face.outer_wire())
                    .unwrap()
                    .edges()
                    .iter()
                    .all(|oe| {
                        let e = topo.edge(oe.edge()).unwrap();
                        (topo.vertex(e.start()).unwrap().point().z() - 30.0).abs() < 1e-9
                    })
        })
        .expect("top face")
}

/// A 2 mm inward shell with the top face open.
fn open_top_shell(topo: &mut Topology, solid: SolidId) -> SolidId {
    let top = top_face(topo, solid);
    remus_operations::shell_op::shell_outcome_with_evolution(topo, solid, 2.0, &[top], None)
        .expect("2 mm open-top shell")
        .outcome
        .solid
}

const ROUNDED_VOLUME: f64 = 60.0 * 40.0 * 30.0 - 4.0 * (25.0 - 25.0 * PI / 4.0) * 30.0;

/// The fallback's half-space boxes are scaffolding: after a split the only new
/// live solids are the two halves.
#[test]
fn fallback_retires_its_half_space_tools() {
    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, 10.0, 20.0).unwrap();
    let before = topo.num_solids();
    let halves = split(
        &mut topo,
        cyl,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    assert_eq!(topo.num_solids(), before + 2, "only the halves are new");
    for h in [cyl, halves.positive, halves.negative] {
        assert_valid(&topo, h, "after the tools were retired");
    }
}

#[test]
fn cylinder_split_through_its_axis() {
    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, 10.0, 20.0).unwrap();
    let halves = check_split(
        &mut topo,
        cyl,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        "cylinder x=0",
    );
    let half = PI * 100.0 * 20.0 / 2.0;
    assert_relative(
        "cylinder x=0 positive",
        volume(&topo, halves.positive),
        half,
    );
    assert_relative(
        "cylinder x=0 negative",
        volume(&topo, halves.negative),
        half,
    );
    for h in [halves.positive, halves.negative] {
        assert!(census(&topo, h).cylinder >= 1, "the wall stays a cylinder");
    }
}

#[test]
fn cylinder_split_parallel_to_its_axis() {
    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, 10.0, 20.0).unwrap();
    let halves = check_split(
        &mut topo,
        cyl,
        Point3::new(3.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        "cylinder x=3",
    );
    // Circular segment beyond the chord at distance 3 from the centre.
    let segment = 100.0 * (0.3_f64).acos() - 3.0 * 91.0_f64.sqrt();
    assert_relative(
        "cylinder x=3 positive",
        volume(&topo, halves.positive),
        segment * 20.0,
    );
    assert_relative(
        "cylinder x=3 negative",
        volume(&topo, halves.negative),
        PI * 100.0 * 20.0 - segment * 20.0,
    );
}

#[test]
fn cylinder_split_by_a_plane_off_the_coordinate_axes() {
    // Normal (1, 1, 0) through (2, 0, 0): parallel to the axis, sqrt(2) from
    // it, and aligned with no coordinate axis.
    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, 10.0, 20.0).unwrap();
    let halves = check_split(
        &mut topo,
        cyl,
        Point3::new(2.0, 0.0, 0.0),
        Vec3::new(1.0, 1.0, 0.0),
        "cylinder diagonal",
    );
    let a = 2.0_f64.sqrt();
    let segment = 100.0 * (a / 10.0).acos() - a * (100.0 - a * a).sqrt();
    assert_relative(
        "cylinder diagonal positive",
        volume(&topo, halves.positive),
        segment * 20.0,
    );
    assert!(census(&topo, halves.positive).cylinder >= 1);
}

/// A plane oblique to a cylinder's axis cuts an elliptical section, which
/// the exact booleans do not build either. The split stays a typed refusal
/// naming both routes, and leaves the session untouched.
#[test]
fn cut_neither_route_builds_is_refused_typed_and_rolled_back() {
    use remus_topology::explorer::solid_entity_counts;
    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, 10.0, 20.0).unwrap();
    let before = solid_entity_counts(&topo, cyl).unwrap();
    let solids = topo.num_solids();
    let journal = topo.journal().snapshot();
    let err = split(
        &mut topo,
        cyl,
        Point3::new(0.0, 0.0, 10.0),
        Vec3::new(0.3, 0.0, 1.0),
    )
    .unwrap_err();
    match err {
        OperationsError::Unsupported { operation, reason } => {
            assert_eq!(operation, "split");
            assert!(reason.contains("ellipse"), "got: {reason}");
            assert!(
                reason.contains("half-space boolean fallback also refused"),
                "got: {reason}"
            );
        }
        other => panic!("expected a typed refusal, got {other:?}"),
    }
    assert_eq!(solid_entity_counts(&topo, cyl).unwrap(), before);
    assert_eq!(
        topo.num_solids(),
        solids,
        "the half-space tool was rolled back"
    );
    assert_eq!(topo.journal().snapshot(), journal);
}

#[test]
fn rounded_box_split_across_the_rounds() {
    let mut topo = Topology::new();
    let body = rounded_box(&mut topo);
    assert_relative("rounded box volume", volume(&topo, body), ROUNDED_VOLUME);
    let halves = check_split(
        &mut topo,
        body,
        Point3::new(0.0, 0.0, 15.0),
        Vec3::new(0.0, 0.0, 1.0),
        "rounded box z=15",
    );
    for h in [halves.positive, halves.negative] {
        assert_relative(
            "rounded box z=15 half",
            volume(&topo, h),
            ROUNDED_VOLUME / 2.0,
        );
        assert_eq!(
            census(&topo, h).cylinder,
            4,
            "all four rounds stay cylinders"
        );
    }
}

#[test]
fn rounded_box_split_through_two_rounds() {
    let mut topo = Topology::new();
    let body = rounded_box(&mut topo);
    let halves = check_split(
        &mut topo,
        body,
        Point3::new(27.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        "rounded box x=27",
    );
    // Area beyond x = 27 of the rounded rectangle: a 3 x 40 strip less the
    // two corner regions outside the R5 quarter circles centred at x = 25.
    let corner_full = 3.0 * 5.0;
    let under_arc = {
        // Integral of sqrt(25 - (x - 25)^2) for x from 27 to 30.
        let f = |t: f64| 0.5 * (t * (25.0 - t * t).sqrt() + 25.0 * (t / 5.0).asin());
        f(5.0) - f(2.0)
    };
    let area = 3.0 * 40.0 - 2.0 * (corner_full - under_arc);
    assert_relative(
        "rounded box x=27 positive",
        volume(&topo, halves.positive),
        area * 30.0,
    );
    assert_eq!(census(&topo, halves.positive).cylinder, 2);
    assert_eq!(census(&topo, halves.negative).cylinder, 4);
}

#[test]
fn rounded_box_split_obliquely_through_two_rounds() {
    // 2x - 3y = 0 runs through the round regions at (25..30, 15..20) and its
    // opposite; a half turn about z maps the body onto itself and swaps the
    // sides, so the halves are equal.
    let mut topo = Topology::new();
    let body = rounded_box(&mut topo);
    let halves = check_split(
        &mut topo,
        body,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(2.0, -3.0, 0.0),
        "rounded box oblique",
    );
    for h in [halves.positive, halves.negative] {
        assert_relative(
            "rounded box oblique half",
            volume(&topo, h),
            ROUNDED_VOLUME / 2.0,
        );
    }
}

#[test]
fn rounded_box_planar_control_stays_native() {
    let mut topo = Topology::new();
    let body = rounded_box(&mut topo);
    let halves = check_split(
        &mut topo,
        body,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        "rounded box x=0",
    );
    for h in [halves.positive, halves.negative] {
        assert_relative(
            "rounded box x=0 half",
            volume(&topo, h),
            ROUNDED_VOLUME / 2.0,
        );
    }
}

#[test]
fn open_top_shell_split_through_its_rim() {
    let mut topo = Topology::new();
    let b = box_60_40_30(&mut topo);
    let cup = open_top_shell(&mut topo, b);
    let input = 72_000.0 - 56.0 * 36.0 * 28.0;
    assert_relative("cup volume", volume(&topo, cup), input);
    let halves = check_split(
        &mut topo,
        cup,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        "cup x=0",
    );
    for h in [halves.positive, halves.negative] {
        assert_relative("cup x=0 half", volume(&topo, h), input / 2.0);
    }
}

#[test]
fn open_top_shell_control_across_the_walls() {
    let mut topo = Topology::new();
    let b = box_60_40_30(&mut topo);
    let cup = open_top_shell(&mut topo, b);
    let halves = check_split(
        &mut topo,
        cup,
        Point3::new(0.0, 0.0, 15.0),
        Vec3::new(0.0, 0.0, 1.0),
        "cup z=15",
    );
    // Above z = 15: a 15 mm tall ring of wall.
    let ring = (60.0 * 40.0 - 56.0 * 36.0) * 15.0;
    assert_relative("cup z=15 positive", volume(&topo, halves.positive), ring);
}

#[test]
fn rounded_open_top_shell_split_through_its_rim() {
    let mut topo = Topology::new();
    let body = rounded_box(&mut topo);
    let cup = open_top_shell(&mut topo, body);
    let input = volume(&topo, cup);
    let halves = check_split(
        &mut topo,
        cup,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        "rounded cup x=0",
    );
    for h in [halves.positive, halves.negative] {
        assert_relative("rounded cup x=0 half", volume(&topo, h), input / 2.0);
        assert!(census(&topo, h).cylinder >= 4, "rounds stay cylinders");
    }
}

#[test]
fn rounded_open_top_shell_split_across_the_rounds() {
    let mut topo = Topology::new();
    let body = rounded_box(&mut topo);
    let cup = open_top_shell(&mut topo, body);
    let halves = check_split(
        &mut topo,
        cup,
        Point3::new(0.0, 0.0, 15.0),
        Vec3::new(0.0, 0.0, 1.0),
        "rounded cup z=15",
    );
    assert!(
        census(&topo, halves.positive).cylinder >= 8,
        "inner and outer rounds"
    );
}

/// Reversing the normal swaps the halves, through the fallback as well.
#[test]
fn reversed_normal_swaps_the_fallback_halves() {
    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, 10.0, 20.0).unwrap();
    let point = Point3::new(3.0, 0.0, 0.0);
    let forward = split(&mut topo, cyl, point, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let backward = split(&mut topo, cyl, point, Vec3::new(-1.0, 0.0, 0.0)).unwrap();
    assert_relative(
        "reversed positive",
        volume(&topo, backward.positive),
        volume(&topo, forward.negative),
    );
    assert_relative(
        "reversed negative",
        volume(&topo, backward.negative),
        volume(&topo, forward.positive),
    );
}

/// The fallback reports construction-derived evolution like the native path:
/// every carried or trimmed face is `modified` from an input face, and each
/// half's cap is the one synthesised face, `unresolved` with no candidates.
#[test]
fn fallback_evolution_names_every_face() {
    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, 10.0, 20.0).unwrap();
    let inputs: Vec<usize> = solid_faces(&topo, cyl)
        .unwrap()
        .into_iter()
        .map(remus_topology::arena::Id::index)
        .collect();
    let (halves, evo) = split_with_evolution(
        &mut topo,
        cyl,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    for (half, map, side) in [
        (halves.positive, &evo.positive, "positive"),
        (halves.negative, &evo.negative, "negative"),
    ] {
        let outputs: Vec<usize> = solid_faces(&topo, half)
            .unwrap()
            .into_iter()
            .map(remus_topology::arena::Id::index)
            .collect();
        assert_eq!(map.unresolved.len(), 1, "{side}: one cap");
        for (src, outs) in &map.modified {
            assert!(
                inputs.contains(src),
                "{side}: source {src} is an input face"
            );
            for out in outs {
                assert!(outputs.contains(out), "{side}: output {out} is in the half");
            }
        }
        let named = map.modified.values().map(Vec::len).sum::<usize>() + map.unresolved.len();
        assert_eq!(named, outputs.len(), "{side}: every face is accounted for");
    }
}

/// A plane that only touches the body along a face still refuses with the
/// typed split refusal, now saying that it does not cross the interior.
#[test]
fn plane_containing_only_a_face_says_it_does_not_cross_the_interior() {
    let mut topo = Topology::new();
    let b = box_60_40_30(&mut topo);
    let err = split(
        &mut topo,
        b,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap_err();
    match err {
        OperationsError::Unsupported { operation, reason } => {
            assert_eq!(operation, "split");
            assert!(
                reason.contains("does not cross the solid's interior"),
                "got: {reason}"
            );
        }
        other => panic!("expected a typed refusal, got {other:?}"),
    }
}

/// A plane across the prongs of a U-block leaves the upper half in two
/// disjoint pieces. The trimmer refuses a section with more than one outer
/// loop; the boolean route returns that half as one solid holding both
/// prongs, valid, with every volume in closed form.
#[test]
fn u_block_split_across_its_prongs() {
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, 30.0, 10.0, 20.0).unwrap();
    let slot = make_box(&mut topo, 10.0, 14.0, 20.0).unwrap();
    transform_solid(&mut topo, slot, &Mat4::translation(10.0, -2.0, 5.0)).unwrap();
    let u = boolean(&mut topo, BooleanOp::Cut, blank, slot).unwrap();
    assert_relative("U-block volume", volume(&topo, u), 6000.0 - 1500.0);
    let halves = check_split(
        &mut topo,
        u,
        Point3::new(0.0, 0.0, 10.0),
        Vec3::new(0.0, 0.0, 1.0),
        "U-block z=10",
    );
    assert_relative(
        "U-block prongs",
        volume(&topo, halves.positive),
        2.0 * 1000.0,
    );
    assert_relative(
        "U-block base",
        volume(&topo, halves.negative),
        3000.0 - 500.0,
    );
}
