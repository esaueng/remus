#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::{make_circle_edge, make_polygon_wire};
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::wire::{OrientedEdge, Wire};

use super::*;

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

fn origin() -> Point3 {
    Point3::new(0.0, 0.0, 0.0)
}

/// A circle of `radius` about `center` in the plane with `normal`.
fn circle_face(topo: &mut Topology, center: Point3, normal: Vec3, radius: f64) -> FaceId {
    let edge = make_circle_edge(topo, center, normal, radius, 1e-7).unwrap();
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let n = normal.normalize().unwrap();
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: n,
            d: crate::dot_normal_point(n, center),
        },
    ))
}

/// A polygon face in the XZ plane (normal −Y, CCW about it).
fn xz_polygon(topo: &mut Topology, xz: &[(f64, f64)]) -> FaceId {
    let points: Vec<Point3> = xz.iter().map(|&(x, z)| Point3::new(x, 0.0, z)).collect();
    let wire = make_polygon_wire(topo, &points, 1e-7).unwrap();
    let normal = crate::winding::newell_normal(&points).normalize().unwrap();
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal,
            d: crate::dot_normal_point(normal, points[0]),
        },
    ))
}

fn assert_valid(topo: &Topology, solid: SolidId, label: &str) {
    let report = crate::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {report:?}");
}

fn census(topo: &Topology, solid: SolidId) -> (usize, usize, usize) {
    let mut planes = 0;
    let mut nurbs = 0;
    let mut other = 0;
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => planes += 1,
            FaceSurface::Nurbs(_) => nurbs += 1,
            _ => other += 1,
        }
    }
    (planes, nurbs, other)
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    crate::measure::mass_properties(topo, solid).unwrap().mass
}

fn rel(measured: f64, exact: f64) -> f64 {
    (measured - exact).abs() / exact.abs()
}

/// The spring the issue measured: a radius-1.5 wire centred 12 from the Z
/// axis, pitch 5, three turns.
fn spring(topo: &mut Topology) -> (FaceId, SolidId) {
    let profile = circle_face(
        topo,
        Point3::new(12.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        1.5,
    );
    let solid = helical_sweep_in_place(topo, profile, origin(), Z, 5.0, 3.0).unwrap();
    (profile, solid)
}

#[test]
fn circle_spring_is_two_caps_two_round_sides_and_exact_volume() {
    let mut topo = Topology::new();
    let (profile, solid) = spring(&mut topo);
    assert_valid(&topo, solid, "spring");
    assert_eq!(
        census(&topo, solid),
        (2, 2, 0),
        "2 planar caps + 2 NURBS sides"
    );

    // Pappus holds for a screw motion: V = A · 2π x̄ · turns.
    let exact = PI * 1.5 * 1.5 * TAU * 12.0 * 3.0;
    let measured = volume(&topo, solid);
    let error = rel(measured, exact);
    assert!(error < 1e-4, "volume {measured} vs {exact} (rel {error:e})");

    // The start cap is the profile in place: same plane, same circle.
    let profile_plane = match topo.face(profile).unwrap().surface() {
        FaceSurface::Plane { normal, d } => (*normal, *d),
        _ => unreachable!(),
    };
    let caps: Vec<_> = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|f| match topo.face(f).unwrap().surface() {
            FaceSurface::Plane { normal, d } => Some((f, *normal, *d)),
            _ => None,
        })
        .collect();
    let start = caps
        .iter()
        .find(|(_, normal, d)| {
            (normal.cross(profile_plane.0)).length() < 1e-12
                && (d * normal.dot(profile_plane.0) - profile_plane.1).abs() < 1e-12
        })
        .expect("a cap on the profile plane");
    for oe in topo
        .wire(topo.face(start.0).unwrap().outer_wire())
        .unwrap()
        .edges()
    {
        let edge = topo.edge(oe.edge()).unwrap();
        let EdgeCurve::Circle(circle) = edge.curve() else {
            panic!("start cap keeps the exact circle");
        };
        assert!((circle.center() - Point3::new(12.0, 0.0, 0.0)).length() < 1e-12);
        assert!((circle.radius() - 1.5).abs() < 1e-12);
    }
}

/// Recover each sample's turn angle about the axis, unwind the screw motion
/// and measure how far it lands from the profile circle.
fn max_off_profile(topo: &Topology, solid: SolidId, pitch: f64, turns: f64) -> f64 {
    let mut worst: f64 = 0.0;
    for face in solid_faces(topo, solid).unwrap() {
        let FaceSurface::Nurbs(surface) = topo.face(face).unwrap().surface() else {
            continue;
        };
        let (u0, u1) = surface.domain_u();
        let (v0, v1) = surface.domain_v();
        for i in 0..=40 {
            for j in 0..=600 {
                let u = u0 + (u1 - u0) * f64::from(i) / 40.0;
                let v = v0 + (v1 - v0) * f64::from(j) / 600.0;
                let p = surface.evaluate(u, v);
                // Nominal angle from the parameter fixes the turn count; the
                // exact angle comes from atan2 about the axis.
                let nominal = v * TAU * turns;
                let raw = p.y().atan2(p.x());
                let phi = raw + TAU * ((nominal - raw) / TAU).round();
                let (s, c) = (-phi).sin_cos();
                let x = p.x() * c - p.y() * s;
                let y = p.x() * s + p.y() * c;
                let z = p.z() - pitch * phi / TAU;
                // Back on the profile plane y = 0, on the circle about (12, 0, 0).
                let off_plane = y.abs();
                let off_circle = ((x - 12.0).hypot(z) - 1.5).abs();
                worst = worst.max(off_plane.hypot(off_circle));
            }
        }
    }
    worst
}

#[test]
fn spring_side_faces_are_round_not_faceted() {
    let mut topo = Topology::new();
    let (_, solid) = spring(&mut topo);
    let worst = max_off_profile(&topo, solid, 5.0, 3.0);
    let bound = rise_tolerance(5.0);
    assert!(worst <= bound * 1.01, "off profile by {worst}");
}

#[test]
fn rise_deviation_bound_covers_the_exact_maximum() {
    for n in 4..=400_u32 {
        let alpha = TAU / f64::from(n);
        let tau = (alpha / 4.0).tan();
        let mut worst: f64 = 0.0;
        for k in 0..=4000 {
            let s = f64::from(k) / 4000.0;
            let rational = s * (1.0 + tau * tau) / (2.0 * (1.0 + tau * tau * s * s));
            let exact = 2.0 * (tau * s).atan() / alpha;
            worst = worst.max((rational - exact).abs());
        }
        let bound = rise_deviation_bound(alpha, 1.0);
        assert!(worst <= bound, "n {n}: {worst} > {bound}");
        assert!(
            worst >= 0.9 * bound,
            "n {n}: bound is loose, {worst} vs {bound}"
        );
    }
}

#[test]
fn rectangle_beside_the_axis_has_four_sides_and_exact_volume() {
    let mut topo = Topology::new();
    let profile = xz_polygon(&mut topo, &[(4.0, 0.0), (6.0, 0.0), (6.0, 1.5), (4.0, 1.5)]);
    let solid = helical_sweep_in_place(&mut topo, profile, origin(), Z, 2.0, 2.0).unwrap();
    assert_valid(&topo, solid, "rectangle coil");
    assert_eq!(census(&topo, solid), (2, 4, 0));
    let exact = 2.0 * 1.5 * TAU * 5.0 * 2.0;
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-4);
}

/// A 4 × 3 rectangle in the XZ plane with a radius-0.8 circular hole.
fn holed_profile(topo: &mut Topology) -> FaceId {
    let outer_points = [
        Point3::new(6.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 3.0),
        Point3::new(6.0, 0.0, 3.0),
    ];
    let outer = make_polygon_wire(topo, &outer_points, 1e-7).unwrap();
    let normal = crate::winding::newell_normal(&outer_points)
        .normalize()
        .unwrap();
    // The hole runs clockwise about the face normal.
    let hole = make_circle_edge(topo, Point3::new(9.0, 0.0, 1.0), -normal, 0.8, 1e-7).unwrap();
    let hole = topo.add_wire(Wire::new(vec![OrientedEdge::new(hole, true)], true).unwrap());
    topo.add_face(Face::new(
        outer,
        vec![hole],
        FaceSurface::Plane {
            normal,
            d: crate::dot_normal_point(normal, outer_points[0]),
        },
    ))
}

#[test]
fn holed_profile_sweeps_inward_hole_walls_and_exact_volume() {
    let mut topo = Topology::new();
    let profile = holed_profile(&mut topo);
    let solid = helical_sweep_in_place(&mut topo, profile, origin(), Z, 4.0, 1.5).unwrap();
    assert_valid(&topo, solid, "holed coil");
    // 4 rectangle sides + 2 hole halves.
    assert_eq!(census(&topo, solid), (2, 6, 0));
    // Area-weighted centroid distance from the axis.
    let (outer_area, outer_x) = (12.0, 8.0);
    let (hole_area, hole_x) = (PI * 0.8 * 0.8, 9.0);
    let area = outer_area - hole_area;
    let centroid = outer_area.mul_add(outer_x, -hole_area * hole_x) / area;
    let exact = area * TAU * centroid * 1.5;
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-4);
    // The hole walls face into the hole.
    for face in solid_faces(&topo, solid).unwrap() {
        let data = topo.face(face).unwrap();
        let FaceSurface::Nurbs(surface) = data.surface() else {
            continue;
        };
        let (u0, u1) = surface.domain_u();
        let u = f64::midpoint(u0, u1);
        let p = surface.evaluate(u, 0.0);
        let Ok(mut n) = surface.normal(u, 0.0) else {
            continue;
        };
        if data.is_reversed() {
            n = -n;
        }
        let from_hole = p - Point3::new(9.0, 0.0, 1.0);
        if (from_hole.length() - 0.8).abs() < 1e-9 {
            assert!(n.dot(from_hole) < 0.0, "hole wall must face into the hole");
        }
    }
}

/// `p` moved by the exact screw motion about Z through the origin.
fn screwed(p: Point3, pitch: f64, phi: f64) -> Point3 {
    let (s, c) = phi.sin_cos();
    Point3::new(
        p.x() * c - p.y() * s,
        p.x() * s + p.y() * c,
        p.z() + pitch * phi / TAU,
    )
}

fn bounds(topo: &Topology, solid: SolidId) -> (Point3, Point3) {
    let aabb = crate::measure::solid_bounding_box(topo, solid).unwrap();
    (aabb.min, aabb.max)
}

#[test]
fn negative_pitch_climbs_against_the_axis_and_fractional_turns_end_where_predicted() {
    let mut topo = Topology::new();
    let profile = circle_face(
        &mut topo,
        Point3::new(12.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        1.5,
    );
    let solid = helical_sweep_in_place(&mut topo, profile, origin(), Z, -5.0, 2.5).unwrap();
    assert_valid(&topo, solid, "left-climbing coil");
    let (lo, hi) = bounds(&topo, solid);
    assert!(hi.z() <= 1.5 + 1e-6, "climbs down: top {}", hi.z());
    assert!(
        lo.z() <= -12.5 - 1.5 + 1e-3,
        "reaches −12.5: bottom {}",
        lo.z()
    );

    let exact = PI * 1.5 * 1.5 * TAU * 12.0 * 2.5;
    assert!(rel(volume(&topo, solid), exact) < 1e-4);

    // The end cap is the profile moved by the full motion (2.5 turns).
    let phi = TAU * 2.5;
    let end_center = screwed(Point3::new(12.0, 0.0, 0.0), -5.0, phi);
    let end_normal = Vec3::new(-phi.sin(), phi.cos(), 0.0);
    let mut found = false;
    for face in solid_faces(&topo, solid).unwrap() {
        let FaceSurface::Plane { normal, .. } = topo.face(face).unwrap().surface() else {
            continue;
        };
        let edge = topo
            .wire(topo.face(face).unwrap().outer_wire())
            .unwrap()
            .edges()[0];
        let EdgeCurve::Circle(circle) = topo.edge(edge.edge()).unwrap().curve() else {
            panic!("caps keep exact circles");
        };
        if (circle.center() - end_center).length() < 1e-9 {
            assert!(normal.cross(end_normal).length() < 1e-9);
            assert!((circle.radius() - 1.5).abs() < 1e-12);
            found = true;
        }
    }
    assert!(found, "an end cap centred at {end_center:?}");
}

#[test]
fn tilted_axis_and_offset_origin_follow_the_screw_motion() {
    // The same spring about an arbitrary axis: build it about Z, then compare
    // with one built about a tilted axis through an offset origin by moving a
    // vertex of each through the analytic motion.
    let mut topo = Topology::new();
    let axis = Vec3::new(1.0, -2.0, 2.0).normalize().unwrap();
    let o = Point3::new(3.0, -1.0, 7.0);
    let seed = Vec3::new(1.0, 0.0, 0.0);
    let radial = (seed - axis * seed.dot(axis)).normalize().unwrap();
    let center = o + radial * 9.0 + axis * 2.0;
    // Profile plane contains the axis.
    let normal = axis.cross(radial);
    let profile = circle_face(&mut topo, center, normal, 1.0);
    let solid = helical_sweep_in_place(&mut topo, profile, o, axis * 3.0, 4.0, 1.25).unwrap();
    assert_valid(&topo, solid, "tilted coil");
    let exact = PI * TAU * 9.0 * 1.25;
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-4, "{measured} vs {exact}");
}

type Sweep = fn(&mut Topology) -> Result<SolidId, OperationsError>;

#[test]
fn refusals_are_clear_and_leave_the_topology_unchanged() {
    let cases: [(&str, Sweep, &str); 7] = [
        (
            "crossing the axis",
            |topo| {
                let p = xz_polygon(topo, &[(-1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (-1.0, 1.0)]);
                helical_sweep_in_place(topo, p, origin(), Z, 5.0, 2.0)
            },
            "axis",
        ),
        (
            "touching the axis",
            |topo| {
                let p = xz_polygon(topo, &[(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]);
                helical_sweep_in_place(topo, p, origin(), Z, 5.0, 2.0)
            },
            "touches",
        ),
        (
            "overlapping coils",
            |topo| {
                let p = xz_polygon(topo, &[(4.0, 0.0), (6.0, 0.0), (6.0, 3.0), (4.0, 3.0)]);
                helical_sweep_in_place(topo, p, origin(), Z, 2.5, 2.0)
            },
            "overlap",
        ),
        (
            "zero pitch",
            |topo| {
                let p = xz_polygon(topo, &[(4.0, 0.0), (6.0, 0.0), (6.0, 1.0), (4.0, 1.0)]);
                helical_sweep_in_place(topo, p, origin(), Z, 0.0, 2.0)
            },
            "pitch",
        ),
        (
            "zero axis",
            |topo| {
                let p = xz_polygon(topo, &[(4.0, 0.0), (6.0, 0.0), (6.0, 1.0), (4.0, 1.0)]);
                helical_sweep_in_place(topo, p, origin(), Vec3::new(0.0, 0.0, 0.0), 2.0, 2.0)
            },
            "axis direction",
        ),
        (
            "zero turns",
            |topo| {
                let p = xz_polygon(topo, &[(4.0, 0.0), (6.0, 0.0), (6.0, 1.0), (4.0, 1.0)]);
                helical_sweep_in_place(topo, p, origin(), Z, 2.0, 0.0)
            },
            "turns",
        ),
        (
            "too many turns",
            |topo| {
                let p = xz_polygon(topo, &[(4.0, 0.0), (6.0, 0.0), (6.0, 1.0), (4.0, 1.0)]);
                helical_sweep_in_place(topo, p, origin(), Z, 2.0, 1e6)
            },
            "too many spans",
        ),
    ];
    for (label, build, needle) in cases {
        let mut topo = Topology::new();
        let result = build(&mut topo);
        let Err(OperationsError::InvalidInput { reason }) = result else {
            panic!("{label}: expected a refusal, got {result:?}");
        };
        assert!(reason.contains(needle), "{label}: {reason}");
        assert_eq!(
            topo.num_faces(),
            1,
            "{label}: only the profile face is left"
        );
    }
}

#[test]
fn a_single_turn_of_a_tall_profile_is_not_an_overlap() {
    // Under one full turn the coil cannot meet itself.
    let mut topo = Topology::new();
    let profile = xz_polygon(&mut topo, &[(4.0, 0.0), (6.0, 0.0), (6.0, 3.0), (4.0, 3.0)]);
    let solid = helical_sweep_in_place(&mut topo, profile, origin(), Z, 2.5, 0.9).unwrap();
    assert_valid(&topo, solid, "tall single turn");
}

/// Mesh edges used by exactly one triangle; 0 means watertight.
fn open_mesh_edges(mesh: &crate::tessellate::TriangleMesh) -> usize {
    use std::collections::HashMap;
    let mut uses: HashMap<(u32, u32), i32> = HashMap::new();
    for t in mesh.indices.chunks(3) {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            *uses.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    uses.values().filter(|&&c| c == 1).count()
}

fn mesh_volume(mesh: &crate::tessellate::TriangleMesh) -> f64 {
    let mut six = 0.0;
    for t in mesh.indices.chunks(3) {
        let [a, b, c] = [0, 1, 2].map(|k| mesh.positions[t[k] as usize]);
        six += (a - origin()).dot((b - origin()).cross(c - origin()));
    }
    six / 6.0
}

#[test]
fn spring_tessellates_watertight_and_encloses_its_volume() {
    let mut topo = Topology::new();
    let (_, solid) = spring(&mut topo);
    let exact = PI * 1.5 * 1.5 * TAU * 12.0 * 3.0;
    for deflection in [0.05, 0.01] {
        let mesh = crate::tessellate::tessellate_solid(&topo, solid, deflection).unwrap();
        let open = open_mesh_edges(&mesh);
        let enclosed = mesh_volume(&mesh);
        assert_eq!(open, 0, "watertight at deflection {deflection}");
        assert!(
            rel(enclosed, exact) < 1e-2,
            "mesh encloses {enclosed} vs {exact}"
        );
    }
}

/// A radius-0.8 post parallel to the axis, a quarter turn round, piercing
/// the spring's wire twice (near z = 1.25 and z = 6.25).
fn post(topo: &mut Topology) -> SolidId {
    let post = crate::primitives::make_cylinder(topo, 0.8, 8.0).unwrap();
    crate::transform::transform_solid(
        topo,
        post,
        &remus_math::mat::Mat4::translation(0.0, 12.0, -0.5),
    )
    .unwrap();
    post
}

/// Downstream robustness: the coil must fuse with a solid that cuts through
/// it. The exact-only pipeline cannot do it yet — the general plane × NURBS
/// and analytic × NURBS sections seed from a fixed grid over the whole face,
/// which on a face spanning every turn is under one sample per span, so the
/// sections come back fragmented (the same pipeline also refuses the faceted
/// [`crate::helix::helical_sweep`] spring). It refuses typed rather than
/// answering wrong, and the disclosed approximate fallback fuses the pair
/// into a valid solid of the right volume.
#[test]
fn spring_fuses_with_a_post_through_two_coils() {
    use crate::boolean::{BooleanOp, BooleanQuality, boolean, boolean_with_context};
    use remus_math::context::{FallbackPolicy, OperationContext};

    let mut topo = Topology::new();
    let (_, coil) = spring(&mut topo);
    let post = post(&mut topo);
    let (coil_volume, post_volume) = (volume(&topo, coil), volume(&topo, post));

    match boolean(&mut topo, BooleanOp::Fuse, coil, post) {
        Ok(fused) => {
            // An engine that learns these sections must answer correctly.
            assert_valid(&topo, fused, "exact coil ∪ post");
            let v = volume(&topo, fused);
            assert!(v > coil_volume && v < coil_volume + post_volume, "{v}");
        }
        Err(OperationsError::ExactOnlyUnattainable) => {}
        Err(other) => panic!("exact fuse must succeed or refuse typed, got {other:?}"),
    }

    let context =
        OperationContext::new().with_fallback(FallbackPolicy::AllowApproximate { budget: 0.05 });
    let outcome = boolean_with_context(&mut topo, BooleanOp::Fuse, coil, post, &context).unwrap();
    assert!(matches!(
        outcome.quality,
        BooleanQuality::Exact | BooleanQuality::Approximate { .. }
    ));
    assert_valid(&topo, outcome.solid, "coil ∪ post");
    let fused = volume(&topo, outcome.solid);
    // The mesh fallback is inscribed at the 0.05 budget, so allow it 1 %.
    assert!(fused > 0.99 * coil_volume && fused < coil_volume + post_volume);
}

/// A box straddling the wire of a half-turn coil: four corners inside the
/// wire, four outside. The exact fuse currently returns the coil alone (and
/// the exact intersection the whole box) because the fragmented plane ×
/// NURBS sections leave the box unsplit and it is classified by an inside
/// corner. Kept as the reproducer for that boolean-engine defect.
#[test]
#[ignore = "known exact-boolean defect: plane × multi-span NURBS sections fragment and the fuse drops the box"]
fn half_coil_fuses_with_a_box_straddling_its_wire() {
    let mut topo = Topology::new();
    let profile = circle_face(
        &mut topo,
        Point3::new(12.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        1.5,
    );
    let coil = helical_sweep_in_place(&mut topo, profile, origin(), Z, 5.0, 0.5).unwrap();
    let tool = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    crate::transform::transform_solid(
        &mut topo,
        tool,
        &remus_math::mat::Mat4::translation(-1.0, 11.0, 0.5),
    )
    .unwrap();
    let coil_volume = volume(&topo, coil);
    let fused =
        crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Fuse, coil, tool).unwrap();
    assert_valid(&topo, fused, "half coil ∪ box");
    assert!(
        volume(&topo, fused) > coil_volume + 1e-3,
        "the box's outside corners are lost"
    );
}

fn face_from_edges(
    topo: &mut Topology,
    edges: Vec<OrientedEdge>,
    normal: Vec3,
    on: Point3,
) -> FaceId {
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal,
            d: crate::dot_normal_point(normal, on),
        },
    ))
}

/// A "D": a trimmed semicircular arc closed by its chord, traversed either
/// way round, so arcs run both with and against their wire.
#[test]
fn trimmed_arc_profile_sweeps_exactly_in_either_traversal() {
    use remus_math::curves::Circle3D;
    use remus_topology::edge::Edge;
    use remus_topology::vertex::Vertex;

    let normal = Vec3::new(0.0, -1.0, 0.0);
    let center = Point3::new(8.0, 0.0, 0.0);
    let circle = Circle3D::new_with_ref(center, normal, 1.0, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let half = std::f64::consts::FRAC_PI_2;
    // Half-disc area π/2 with its centroid 4/3π beyond the chord.
    let exact = PI / 2.0 * TAU * (8.0 + 4.0 / (3.0 * PI)) * 1.75;
    for forward in [true, false] {
        let mut topo = Topology::new();
        let bottom = topo.add_vertex(Vertex::new(circle.evaluate(-half), 1e-7));
        let top = topo.add_vertex(Vertex::new(circle.evaluate(half), 1e-7));
        let mut arc = Edge::new(bottom, top, EdgeCurve::Circle(circle.clone()));
        arc.set_trim(Some((-half, half)));
        let arc = topo.add_edge(arc);
        let chord = topo.add_edge(Edge::new(top, bottom, EdgeCurve::Line));
        let edges = if forward {
            vec![OrientedEdge::new(arc, true), OrientedEdge::new(chord, true)]
        } else {
            vec![
                OrientedEdge::new(chord, false),
                OrientedEdge::new(arc, false),
            ]
        };
        let face_normal = if forward { normal } else { -normal };
        let profile = face_from_edges(&mut topo, edges, face_normal, center);
        let solid = helical_sweep_in_place(&mut topo, profile, origin(), Z, 3.0, 1.75).unwrap();
        assert_valid(&topo, solid, "D coil");
        assert_eq!(census(&topo, solid), (2, 2, 0));
        let measured = volume(&topo, solid);
        assert!(
            rel(measured, exact) < 1e-4,
            "forward {forward}: {measured} vs {exact}"
        );
    }
}

#[test]
fn ellipse_and_closed_nurbs_profiles_sweep_exactly() {
    use remus_topology::edge::Edge;
    use remus_topology::vertex::Vertex;

    let normal = Vec3::new(0.0, 1.0, 0.0);
    let center = Point3::new(10.0, 0.0, 0.0);

    let mut topo = Topology::new();
    let ellipse =
        remus_topology::builder::make_ellipse_edge(&mut topo, center, normal, 1.2, 0.7, 1e-7)
            .unwrap();
    let profile = face_from_edges(
        &mut topo,
        vec![OrientedEdge::new(ellipse, true)],
        normal,
        center,
    );
    let solid = helical_sweep_in_place(&mut topo, profile, origin(), Z, 4.0, 2.0).unwrap();
    assert_valid(&topo, solid, "elliptic coil");
    assert_eq!(census(&topo, solid), (2, 2, 0));
    let exact = PI * 1.2 * 0.7 * TAU * 10.0 * 2.0;
    assert!(rel(volume(&topo, solid), exact) < 1e-4);

    // A full circle stored as one closed rational NURBS edge.
    let mut topo = Topology::new();
    let circle = remus_math::curves::Circle3D::new(center, normal, 1.0).unwrap();
    let curve = remus_heal::construct::convert_curve::circle_to_nurbs(&circle).unwrap();
    let domain = curve.domain();
    let seam = topo.add_vertex(Vertex::new(curve.evaluate(domain.0), 1e-7));
    let mut edge = Edge::new(seam, seam, EdgeCurve::NurbsCurve(curve));
    edge.set_trim(Some(domain));
    let edge = topo.add_edge(edge);
    let profile = face_from_edges(
        &mut topo,
        vec![OrientedEdge::new(edge, true)],
        normal,
        center,
    );
    let solid = helical_sweep_in_place(&mut topo, profile, origin(), Z, 4.0, 2.0).unwrap();
    assert_valid(&topo, solid, "NURBS-circle coil");
    assert_eq!(census(&topo, solid), (2, 2, 0));
    let exact = PI * TAU * 10.0 * 2.0;
    assert!(rel(volume(&topo, solid), exact) < 1e-4);
}

/// A profile perpendicular to the axis is a twisted extrusion: no coil can
/// overlap and the motion crosses it uniformly.
#[test]
fn perpendicular_profile_beside_the_axis_twists_exactly() {
    let mut topo = Topology::new();
    let profile = circle_face(&mut topo, Point3::new(5.0, 0.0, 0.0), Z, 1.0);
    let solid = helical_sweep_in_place(&mut topo, profile, origin(), Z, 6.0, 1.5).unwrap();
    assert_valid(&topo, solid, "twisted rod");
    // Pappus-for-screws needs the area normal to the trajectory; for a
    // profile perpendicular to the axis the swept volume is area × rise.
    let exact = PI * 6.0 * 1.5;
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-4, "{measured} vs {exact}");
}
