//! Regression: a solid's bounding box must bound the faces, not the surfaces
//! those faces are cut from.
//!
//! `solid_bounding_box` expanded spherical and toroidal faces to their whole
//! analytic surface. That is right for a primitive sphere or a whole torus —
//! their boundary is a seam that bounds nothing, so the surface *is* the face —
//! but wrong for the trimmed blends an imported part is full of. A 1.2 MB CATIA
//! import (72 toroidal faces) reported a box roughly twice the part's true
//! extent in two axes, both hitting the same value: the untrimmed radius of one
//! big ring. Anything that frames a camera or culls by bounds then works from a
//! box the model rattles around inside — OpenZCAD's Fit View shrank the part
//! into a corner of the viewport.
//!
//! The same held for NURBS faces, whose grid spanned the surface's whole knot
//! domain (the part carries 65 B-spline surfaces alongside its tori).
//!
//! The reproductions are synthetic on purpose (the reporting part is
//! proprietary): a 14° revolve of a small circle 200 mm off the axis, and a
//! patch cut from the corner of a tall domed sheet. Each is a sliver of a much
//! larger surface, so an untrimmed expansion overshoots by more than an order
//! of magnitude — the same failure as the customer part, in shapes small enough
//! to state exactly.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::TAU;

use brepkit_math::nurbs::surface::NurbsSurface;
use brepkit_math::surfaces::{CylindricalSurface, ToroidalSurface};
use brepkit_math::vec::{Point3, Vec3};
use brepkit_operations::measure;
use brepkit_operations::primitives;
use brepkit_operations::revolve::revolve;
use brepkit_topology::Topology;
use brepkit_topology::builder::{make_circle_edge, make_face_from_wire};
use brepkit_topology::edge::{Edge, EdgeCurve};
use brepkit_topology::face::{Face, FaceSurface};
use brepkit_topology::shell::Shell;
use brepkit_topology::solid::{Solid, SolidId};
use brepkit_topology::vertex::Vertex;
use brepkit_topology::wire::{OrientedEdge, Wire};

const TOL: f64 = 1e-7;
/// The box is analytically exact for these shapes; this only absorbs the
/// arithmetic.
const EPS: f64 = 1e-6;

/// Revolve a circle of radius `minor`, centred `major` from the axis, through
/// `sweep` about Z. A partial sweep yields one trimmed toroidal band closed by
/// two planar disc caps; a full sweep yields one doubly-periodic torus face.
fn revolved_ring(topo: &mut Topology, major: f64, minor: f64, sweep: f64) -> SolidId {
    let profile = make_circle_edge(
        topo,
        Point3::new(major, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        minor,
        TOL,
    )
    .unwrap();
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(profile, true)], true).unwrap());
    let face = make_face_from_wire(topo, wire).unwrap();
    revolve(
        topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        sweep,
    )
    .unwrap()
}

#[track_caller]
fn assert_box(topo: &Topology, solid: SolidId, expect_min: [f64; 3], expect_max: [f64; 3]) {
    let bb = measure::solid_bounding_box(topo, solid).unwrap();
    let got_min = [bb.min.x(), bb.min.y(), bb.min.z()];
    let got_max = [bb.max.x(), bb.max.y(), bb.max.z()];
    for axis in 0..3 {
        assert!(
            (got_min[axis] - expect_min[axis]).abs() < EPS
                && (got_max[axis] - expect_max[axis]).abs() < EPS,
            "axis {axis}: got [{}, {}], want [{}, {}]",
            got_min[axis],
            got_max[axis],
            expect_min[axis],
            expect_max[axis]
        );
    }
}

/// The band swept by the profile circle, computed by hand.
///
/// Points are `(R + r·cos v)·(cos u, sin u, 0) + (0, 0, r·sin v)`, so the
/// distance from the axis runs over `[R − r, R + r]` and `z` over `[−r, r]`.
/// Over `u ∈ [0, sweep]` with `sweep < π/2`, `x` is largest at `u = 0` on the
/// outer wall and smallest at `u = sweep` on the inner one, while `y` runs from
/// 0 up to the outer wall at `u = sweep`.
fn swept_band_extent(major: f64, minor: f64, sweep: f64) -> ([f64; 3], [f64; 3]) {
    let (r_in, r_out) = (major - minor, major + minor);
    (
        [r_in * sweep.cos(), 0.0, -minor],
        [r_out, r_out * sweep.sin(), minor],
    )
}

#[test]
fn a_trimmed_torus_band_is_bounded_by_the_band_not_the_ring() {
    let (major, minor, sweep) = (200.0, 5.0, 0.25);
    let mut topo = Topology::new();
    let solid = revolved_ring(&mut topo, major, minor, sweep);

    let (want_min, want_max) = swept_band_extent(major, minor, sweep);
    assert_box(&topo, solid, want_min, want_max);

    // Guard the specific regression: the untrimmed ring reaches ±(R + r) in
    // both X and Y, which is what the old expansion reported. The band's own
    // Y extent is under a tenth of that and it never crosses y = 0 at all.
    let bb = measure::solid_bounding_box(&topo, solid).unwrap();
    assert!(
        bb.min.y() > -EPS,
        "band lies at y >= 0; got y_min {} (untrimmed ring would give {})",
        bb.min.y(),
        -(major + minor)
    );
    assert!(
        bb.max.y() < (major + minor) * 0.3,
        "band spans {sweep} rad of the ring; got y_max {}",
        bb.max.y()
    );
}

#[test]
fn a_torus_face_that_wraps_its_surface_still_gets_the_whole_ring() {
    // The complement of the case above: with no trim to find, the expansion
    // must still reach the full analytic extent. A whole torus's boundary is
    // a degenerate seam, so nothing about the face's own edges reveals how far
    // the surface runs.
    let (major, minor) = (200.0, 5.0);
    let rr = major + minor;

    let mut topo = Topology::new();
    let revolved = revolved_ring(&mut topo, major, minor, TAU);
    assert_box(&topo, revolved, [-rr, -rr, -minor], [rr, rr, minor]);

    let mut topo = Topology::new();
    let primitive = primitives::make_torus(&mut topo, 30.0, 7.0, 32).unwrap();
    assert_box(&topo, primitive, [-37.0, -37.0, -7.0], [37.0, 37.0, 7.0]);
}

#[test]
fn an_inner_trim_loop_cannot_shrink_a_wrapping_torus_face() {
    let (major, minor) = (10.0, 3.0);
    let mut topo = Topology::new();

    // Model the primitive torus's degenerate outer seam, then add a small
    // hole near u,v in [0.1, 0.3]. The occupied face is the complement of the
    // hole and therefore still reaches the opposite side at x = -(R + r).
    let seam_point = Point3::new(major + minor, 0.0, 0.0);
    let seam_vertex = topo.add_vertex(Vertex::new(seam_point, TOL));
    let seam_a = topo.add_edge(Edge::new(seam_vertex, seam_vertex, EdgeCurve::Line));
    let seam_b = topo.add_edge(Edge::new(seam_vertex, seam_vertex, EdgeCurve::Line));
    let outer = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(seam_a, true),
                OrientedEdge::new(seam_b, true),
                OrientedEdge::new(seam_a, false),
                OrientedEdge::new(seam_b, false),
            ],
            true,
        )
        .unwrap(),
    );

    let torus_point = |u: f64, v: f64| {
        let radial = major + minor * v.cos();
        Point3::new(radial * u.cos(), radial * u.sin(), minor * v.sin())
    };
    let hole_points = [
        torus_point(0.1, 0.1),
        torus_point(0.3, 0.1),
        torus_point(0.3, 0.3),
        torus_point(0.1, 0.3),
    ];
    let hole_vertices: Vec<_> = hole_points
        .iter()
        .map(|point| topo.add_vertex(Vertex::new(*point, TOL)))
        .collect();
    let hole_edges: Vec<_> = (0..hole_vertices.len())
        .map(|i| {
            topo.add_edge(Edge::new(
                hole_vertices[i],
                hole_vertices[(i + 1) % hole_vertices.len()],
                EdgeCurve::Line,
            ))
        })
        .collect();
    let hole = topo.add_wire(
        Wire::new(
            hole_edges
                .iter()
                .map(|&edge| OrientedEdge::new(edge, true))
                .collect(),
            true,
        )
        .unwrap(),
    );

    let surface = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), major, minor).unwrap();
    let face = topo.add_face(Face::new(outer, vec![hole], FaceSurface::Torus(surface)));
    let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
    let solid = topo.add_solid(Solid::new(shell, vec![]));

    assert_box(
        &topo,
        solid,
        [-(major + minor), -(major + minor), -minor],
        [major + minor, major + minor, minor],
    );
}

#[test]
fn a_whole_sphere_still_gets_its_whole_radius() {
    // A sphere's seam bounds nothing either, and a polar cap's only boundary
    // is one latitude circle — the pole beyond it has to come from the
    // surface. Trimming latitude is only safe once longitude is bounded.
    let mut topo = Topology::new();
    let solid = primitives::make_sphere(&mut topo, 11.0, 32).unwrap();
    assert_box(&topo, solid, [-11.0, -11.0, -11.0], [11.0, 11.0, 11.0]);
}

#[test]
fn a_quarter_ring_is_bounded_on_the_axis_it_does_not_reach() {
    // A quarter turn is wide enough to hold the outer wall's X maximum at
    // u = 0 and its Y maximum at u = π/2, but must still report nothing below
    // zero on either — the half of the ring it never sweeps.
    let (major, minor) = (120.0, 8.0);
    let sweep = std::f64::consts::FRAC_PI_2;
    let mut topo = Topology::new();
    let solid = revolved_ring(&mut topo, major, minor, sweep);

    let bb = measure::solid_bounding_box(&topo, solid).unwrap();
    assert!(
        bb.min.x() > -EPS && bb.min.y() > -EPS,
        "quarter ring stays in the +X+Y quadrant; got min ({}, {})",
        bb.min.x(),
        bb.min.y()
    );
    assert!(
        (bb.max.x() - (major + minor)).abs() < EPS && (bb.max.y() - (major + minor)).abs() < EPS,
        "quarter ring reaches the outer wall on both axes; got max ({}, {})",
        bb.max.x(),
        bb.max.y()
    );
}

// ---------------------------------------------------------------------------
// NURBS faces
// ---------------------------------------------------------------------------

/// A bi-quadratic sheet over x,y in [0,100] with its middle control point
/// lifted to z = 300. Both maps are exact and hand-checkable:
/// `x = 100u`, `y = 100v`, `z = 1200·u(1−u)·v(1−v)` — so the sheet peaks at
/// z = 75 dead centre and falls to zero along every edge of the domain.
fn domed_sheet() -> NurbsSurface {
    let row = |x: f64, mid_z: f64| {
        vec![
            Point3::new(x, 0.0, 0.0),
            Point3::new(x, 50.0, mid_z),
            Point3::new(x, 100.0, 0.0),
        ]
    };
    NurbsSurface::new(
        2,
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![row(0.0, 0.0), row(50.0, 300.0), row(100.0, 0.0)],
        vec![vec![1.0; 3]; 3],
    )
    .unwrap()
}

/// Height of [`domed_sheet`] at `(u, v)`.
fn sheet_z(u: f64, v: f64) -> f64 {
    1200.0 * u * (1.0 - u) * v * (1.0 - v)
}

/// One face on `surf`, trimmed to `[u0,u1] x [v0,v1]`, whose boundary is walked
/// *on the surface* as a fine polyline. Chords straight across the patch would
/// not lie on it, and a face whose edges are off its own surface is not a
/// B-rep face — the projection that recovers the trimmed region would be
/// reading points that are not there.
fn trimmed_patch(
    topo: &mut Topology,
    surf: NurbsSurface,
    (u0, u1): (f64, f64),
    (v0, v1): (f64, f64),
) -> SolidId {
    const PER_SIDE: usize = 12;
    #[allow(clippy::cast_precision_loss)]
    let lerp = |a: f64, b: f64, i: usize| a + (b - a) * (i as f64 / PER_SIDE as f64);
    let mut ring = Vec::with_capacity(PER_SIDE * 4);
    for i in 0..PER_SIDE {
        ring.push(surf.evaluate(lerp(u0, u1, i), v0));
    }
    for i in 0..PER_SIDE {
        ring.push(surf.evaluate(u1, lerp(v0, v1, i)));
    }
    for i in 0..PER_SIDE {
        ring.push(surf.evaluate(lerp(u1, u0, i), v1));
    }
    for i in 0..PER_SIDE {
        ring.push(surf.evaluate(u0, lerp(v1, v0, i)));
    }

    let verts: Vec<_> = ring
        .iter()
        .map(|p| topo.add_vertex(Vertex::new(*p, TOL)))
        .collect();
    let n = verts.len();
    let edges: Vec<_> = (0..n)
        .map(|i| topo.add_edge(Edge::new(verts[i], verts[(i + 1) % n], EdgeCurve::Line)))
        .collect();
    let wire = topo.add_wire(
        Wire::new(
            edges.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(surf)));
    let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

#[track_caller]
fn assert_contains(topo: &Topology, solid: SolidId, lo: [f64; 3], hi: [f64; 3]) {
    let bb = measure::solid_bounding_box(topo, solid).unwrap();
    let got_lo = [bb.min.x(), bb.min.y(), bb.min.z()];
    let got_hi = [bb.max.x(), bb.max.y(), bb.max.z()];
    for axis in 0..3 {
        assert!(
            got_lo[axis] <= lo[axis] + EPS && got_hi[axis] >= hi[axis] - EPS,
            "axis {axis}: box [{}, {}] must contain the face's true [{}, {}]",
            got_lo[axis],
            got_hi[axis],
            lo[axis],
            hi[axis]
        );
    }
}

#[test]
fn a_trimmed_nurbs_patch_is_bounded_by_the_patch_not_the_sheet() {
    // A patch in the far corner of the domain, nowhere near the peak.
    let (u, v) = ((0.80, 0.98), (0.80, 0.98));
    let mut topo = Topology::new();
    let solid = trimmed_patch(&mut topo, domed_sheet(), u, v);

    // Exact extent: x and y are linear in the parameters, and z falls away
    // from the domain's interior, so the patch's own peak is at its inner
    // corner and its floor at the outer one.
    assert_contains(
        &topo,
        solid,
        [100.0 * u.0, 100.0 * v.0, sheet_z(u.1, v.1)],
        [100.0 * u.1, 100.0 * v.1, sheet_z(u.0, v.0)],
    );

    // The regression itself: sampling the whole knot domain put grid points at
    // u = v = 0.5 and reported the sheet's 75 mm peak, plus a box reaching back
    // to x = y = 25 — for a face that lives beyond 80 and never rises past 31.
    let bb = measure::solid_bounding_box(&topo, solid).unwrap();
    assert!(
        bb.max.z() < 45.0,
        "patch tops out at {:.2}; got z_max {} (the sheet's peak is 75)",
        sheet_z(u.0, v.0),
        bb.max.z()
    );
    assert!(
        bb.min.x() > 70.0 && bb.min.y() > 70.0,
        "patch starts at 80; got min ({}, {})",
        bb.min.x(),
        bb.min.y()
    );
}

#[test]
fn an_untrimmed_nurbs_face_still_gets_its_whole_sheet() {
    // The complement: a face spanning its full domain must keep the peak.
    let mut topo = Topology::new();
    let solid = trimmed_patch(&mut topo, domed_sheet(), (0.0, 1.0), (0.0, 1.0));
    assert_contains(&topo, solid, [0.0, 0.0, 0.0], [100.0, 100.0, 75.0]);
}

#[test]
fn a_nurbs_surface_that_closes_on_itself_keeps_its_full_extent_there() {
    // A cylinder as a NURBS surface: periodic in u (around), plain in v (along).
    // A small patch on it must still report the whole ring — the seam bounds
    // nothing, and the two ends of the u domain are the same points in space,
    // so no projection can tell which one a boundary sample came from. The v
    // direction has no such excuse and must trim.
    let cyl = CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 10.0)
        .unwrap();
    let surf = cyl.to_nurbs(0.0, 20.0).unwrap();
    assert!(surf.is_periodic_u() && !surf.is_periodic_v());

    let mut topo = Topology::new();
    let solid = trimmed_patch(&mut topo, surf, (0.0, 0.1), (0.0, 0.25));
    let bb = measure::solid_bounding_box(&topo, solid).unwrap();

    assert!(
        bb.min.x() < -9.9 && bb.max.x() > 9.9 && bb.min.y() < -9.9 && bb.max.y() > 9.9,
        "periodic direction keeps the full ring; got x [{}, {}] y [{}, {}]",
        bb.min.x(),
        bb.max.x(),
        bb.min.y(),
        bb.max.y()
    );
    assert!(
        bb.max.z() < 12.0,
        "the non-periodic direction still trims: patch reaches z = 5 of 20, got {}",
        bb.max.z()
    );
}
