//! Closed-form oracles for the periodic-lateral shortcuts of the face
//! splitter (B19 face-splitter survivor tranche): the rim-chain splitter
//! ([`split_periodic_face_by_rim_chains`], B39's torus-bite cell: marched
//! chains notching one rim or cutting the lateral into two sectors), the
//! closed-ring band splitter, the one-ruling sector rescue, and the
//! box–sphere collar arrangement on a faceted hemisphere.
//!
//! Nothing here reads a splitter's own arithmetic back. Every region is
//! measured from its 3D wire: each edge is re-sampled from its carrier (a rim
//! circle swept in its traversal sense, a fitted section curve, a seam or
//! ruling line), mapped to `(r·θ, z)` with `θ` unwrapped along the loop (or,
//! on the hemisphere, projected onto the equator plane), and its shoelace
//! area compared with the closed-form area of the region the sections were
//! built to bound. Interior points are checked against the same
//! closed-form membership. The stored `(u, v)` endpoints are deliberately not
//! an oracle: the primitive rim circles carry UVs in their own angular
//! parameter, a quarter turn off the cylinder chart that `project_point` uses.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::*;
use remus_math::curves::Circle3D;
use remus_math::surfaces::CylindricalSurface;
use remus_topology::{
    edge::Edge,
    face::Face,
    vertex::Vertex,
    wire::{OrientedEdge, Wire},
};
use std::f64::consts::{PI, TAU};

const TOL: f64 = 1e-7;
/// Section joints and rim contacts are welded at `100 · tol` by the splitter;
/// the 3D wire checks use the same scale.
const WELD: f64 = 1e-5;

/// A primitive-style cylinder lateral (the `make_cylinder` wire): axis +z
/// through the origin, bottom rim at `z = 0` traversed forward, seam up the
/// +x meridian, top rim at `z = h` traversed reversed, seam back down.
fn lateral(r: f64, h: f64, reversed: bool) -> (Topology, FaceId) {
    lateral_between(r, 0.0, h, reversed)
}

/// [`lateral`] with its rims at `z = z0` and `z = z1` on the same carrier,
/// so the rims' `v` is neither zero nor necessarily positive.
fn lateral_between(r: f64, z0: f64, z1: f64, reversed: bool) -> (Topology, FaceId) {
    let mut topo = Topology::new();
    let z = Vec3::new(0.0, 0.0, 1.0);
    let surface = CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), z, r).unwrap();
    let v_bot = topo.add_vertex(Vertex::new(Point3::new(r, 0.0, z0), TOL));
    let v_top = topo.add_vertex(Vertex::new(Point3::new(r, 0.0, z1), TOL));
    let bot = Circle3D::new(Point3::new(0.0, 0.0, z0), z, r).unwrap();
    let top = Circle3D::new(Point3::new(0.0, 0.0, z1), z, r).unwrap();
    let b0 = bot.project(Point3::new(r, 0.0, z0));
    let t0 = top.project(Point3::new(r, 0.0, z1));
    let mut be = Edge::new(v_bot, v_bot, EdgeCurve::Circle(bot));
    be.set_trim(Some((b0, b0 + TAU)));
    let mut te = Edge::new(v_top, v_top, EdgeCurve::Circle(top));
    te.set_trim(Some((t0, t0 + TAU)));
    let be = topo.add_edge(be);
    let te = topo.add_edge(te);
    let seam = topo.add_edge(Edge::new(v_bot, v_top, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(be, true),
                OrientedEdge::new(seam, true),
                OrientedEdge::new(te, false),
                OrientedEdge::new(seam, false),
            ],
            true,
        )
        .unwrap(),
    );
    let mut face = Face::new(wire, vec![], FaceSurface::Cylinder(surface));
    face.set_reversed(reversed);
    let face = topo.add_face(face);
    (topo, face)
}

fn on_cyl(r: f64, theta: f64, z: f64) -> Point3 {
    Point3::new(r * theta.cos(), r * theta.sin(), z)
}

/// A marched (fitted cubic NURBS) section through `pts`. Its knot vector is
/// shifted to start at 2.5: marched curves carry arbitrary parameter
/// domains, and a `[0, 1]` domain hides any slip between `d0` and zero.
fn marched(pts: &[Point3]) -> SectionEdge {
    let fit = remus_math::nurbs::fitting::interpolate(pts, 3).unwrap();
    let shifted = remus_math::nurbs::curve::NurbsCurve::new(
        fit.degree(),
        fit.knots().iter().map(|k| k + 2.5).collect(),
        fit.control_points().to_vec(),
        fit.weights().to_vec(),
    )
    .unwrap();
    let dummy = remus_math::curves2d::Curve2D::Line(
        remus_math::curves2d::Line2D::new(
            Point2::new(0.0, 0.0),
            remus_math::vec::Vec2::new(1.0, 0.0),
        )
        .unwrap(),
    );
    SectionEdge {
        curve_3d: EdgeCurve::NurbsCurve(shifted),
        trim: None,
        pcurve_a: dummy.clone(),
        pcurve_b: dummy,
        start: pts[0],
        end: *pts.last().unwrap(),
        start_uv_a: None,
        end_uv_a: None,
        start_uv_b: None,
        end_uv_b: None,
        target_face: None,
        pave_block_id: None,
    }
}

/// Point lists of the curve `s ↦ (θ(s), z(s))` on the cylinder, cut at
/// `breaks` (increasing values in `[0, 1]`).
fn chain_pieces(r: f64, breaks: &[f64], curve: impl Fn(f64) -> (f64, f64)) -> Vec<Vec<Point3>> {
    breaks
        .windows(2)
        .map(|w| {
            (0..=12)
                .map(|k| {
                    let s = (w[1] - w[0]).mul_add(f64::from(k) / 12.0, w[0]);
                    let (t, z) = curve(s);
                    on_cyl(r, t, z)
                })
                .collect()
        })
        .collect()
}

fn reversed_pts(pts: &[Point3]) -> Vec<Point3> {
    pts.iter().rev().copied().collect()
}

fn cylinder_info() -> SurfaceInfo {
    SurfaceInfo::Parametric {
        u_periodic: true,
        v_periodic: false,
    }
}

/// Split through the public entry with the face's real periodicity.
fn split(topo: &Topology, face: FaceId, sections: &[SectionEdge]) -> Vec<SplitSubFace> {
    split_face_2d(
        topo,
        face,
        sections,
        Rank::A,
        &remus_math::tolerance::Tolerance::default(),
        None,
        Some(&cylinder_info()),
        &std::collections::HashMap::new(),
        None,
    )
    .unwrap()
}

/// Call the rim-chain splitter directly on the face's real boundary pcurves.
fn rim_chains(
    topo: &Topology,
    face: FaceId,
    sections: &[SectionEdge],
) -> Option<Vec<SplitSubFace>> {
    let f = topo.face(face).unwrap();
    let surface = f.surface().clone();
    let pts = collect_wire_points(topo, f.outer_wire());
    let boundary = boundary_edges_to_pcurve(topo, f.outer_wire(), &surface, &pts, None).unwrap();
    split_periodic_face_by_rim_chains(
        &surface,
        &boundary,
        sections,
        Rank::A,
        f.is_reversed(),
        face,
        TOL,
    )
    .unwrap()
}

/// One wire edge re-sampled from its carrier, in traversal order. A rim
/// circle is swept from its start angle to its end angle in the sense its
/// `forward` flag and axis give (a full turn when the ends coincide); a
/// section is its whole fitted curve, oriented by its stored ends.
fn edge_polyline(e: &OrientedPCurveEdge) -> Vec<Point3> {
    const N: u32 = 48;
    match &e.curve_3d {
        EdgeCurve::Line => (0..=N)
            .map(|k| e.start_3d + (e.end_3d - e.start_3d) * (f64::from(k) / f64::from(N)))
            .collect(),
        EdgeCurve::Circle(c) => {
            let ctr = c.center();
            let ang = |p: Point3| (p.y() - ctr.y()).atan2(p.x() - ctr.x());
            let sense = if e.forward { 1.0 } else { -1.0 } * c.normal().z().signum();
            let mut sweep = (sense * (ang(e.end_3d) - ang(e.start_3d))).rem_euclid(TAU);
            if sweep < 1e-9 {
                sweep = TAU;
            }
            let a0 = ang(e.start_3d);
            (0..=N)
                .map(|k| {
                    let a = (sense * sweep).mul_add(f64::from(k) / f64::from(N), a0);
                    Point3::new(
                        c.radius().mul_add(a.cos(), ctr.x()),
                        c.radius().mul_add(a.sin(), ctr.y()),
                        e.start_3d.z(),
                    )
                })
                .collect()
        }
        EdgeCurve::NurbsCurve(n) => {
            let (t0, t1) = n.domain();
            let mut pts: Vec<Point3> = (0..=N)
                .map(|k| n.evaluate((t1 - t0).mul_add(f64::from(k) / f64::from(N), t0)))
                .collect();
            if (pts[0] - e.start_3d).length() > (pts[pts.len() - 1] - e.start_3d).length() {
                pts.reverse();
            }
            assert!(
                (pts[0] - e.start_3d).length() < WELD
                    && (pts[pts.len() - 1] - e.end_3d).length() < WELD,
                "section edge ends are not its curve's ends"
            );
            pts
        }
        other => panic!("unexpected wire curve {}", other.type_tag()),
    }
}

/// Measured facts about one emitted region.
struct Measured {
    /// Shoelace area in `(r·θ, z)`, `θ` unwrapped along the loop; positive
    /// for a loop that keeps the region on its left in `(θ, z)`.
    area: f64,
    /// Net `θ` the loop turns through (0 for a region of a cut-open lateral).
    net_turn: f64,
    /// The interior point as `(θ ∈ [0, 2π), z)` and its radial distance.
    interior: (f64, f64, f64),
}

fn measure(sf: &SplitSubFace, r: f64) -> Measured {
    let wire = &sf.outer_wire;
    assert!(wire.len() >= 2, "degenerate region wire");
    for (i, e) in wire.iter().enumerate() {
        let next = &wire[(i + 1) % wire.len()];
        assert!(
            (e.end_3d - next.start_3d).length() < WELD,
            "wire is open in 3D after edge {i}: {:?} -> {:?}",
            e.end_3d,
            next.start_3d
        );
    }
    let mut pts: Vec<(f64, f64)> = Vec::new();
    let mut theta_prev: Option<f64> = None;
    for e in wire {
        for p in edge_polyline(e) {
            let raw = p.y().atan2(p.x());
            let theta = theta_prev.map_or(raw, |t| t + (raw - t + PI).rem_euclid(TAU) - PI);
            theta_prev = Some(theta);
            pts.push((r * theta, p.z()));
        }
    }
    let n = pts.len();
    let mut twice = 0.0;
    for i in 0..n {
        let (x0, y0) = pts[i];
        let (x1, y1) = pts[(i + 1) % n];
        twice += x0.mul_add(y1, -(x1 * y0));
    }
    let p = sf
        .precomputed_interior
        .expect("rim-chain regions carry an interior");
    Measured {
        area: 0.5 * twice,
        net_turn: (pts[n - 1].0 - pts[0].0) / r,
        interior: (
            p.y().atan2(p.x()).rem_euclid(TAU),
            p.z(),
            p.x().hypot(p.y()),
        ),
    }
}

/// How often each section is used by `region`, as `(forward, backward)`
/// counts matched on the section's 3D ends.
fn section_uses(region: &SplitSubFace, s: &SectionEdge) -> (usize, usize) {
    let near = |a: Point3, b: Point3| (a - b).length() < WELD;
    let mut uses = (0, 0);
    for e in &region.outer_wire {
        if !matches!(e.curve_3d, EdgeCurve::NurbsCurve(_)) {
            continue;
        }
        if near(e.start_3d, s.start) && near(e.end_3d, s.end) {
            uses.0 += 1;
        } else if near(e.start_3d, s.end) && near(e.end_3d, s.start) {
            uses.1 += 1;
        }
    }
    uses
}

/// Every section bounds both regions, once each and in opposite senses.
fn assert_sections_shared(regions: &[SplitSubFace], sections: &[SectionEdge], ctx: &str) {
    for (i, s) in sections.iter().enumerate() {
        let a = section_uses(&regions[0], s);
        let b = section_uses(&regions[1], s);
        assert!(
            (a == (1, 0) && b == (0, 1)) || (a == (0, 1) && b == (1, 0)),
            "{ctx}: section {i} uses {a:?} / {b:?}, want one each way"
        );
    }
}

fn assert_close(got: f64, want: f64, rel: f64, what: &str) {
    assert!(
        (got - want).abs() <= rel * want.abs(),
        "{what}: got {got}, want {want}"
    );
}

/// The notch of half-width `alpha` centred on `θ = π`: the planar section
/// `z = depth · (cos(θ − π) − cos α) / (1 − cos α)` measured from the notched
/// rim (an ellipse arc through the rim at `θ = π ± α`).
fn notch_depth(theta: f64, alpha: f64, depth: f64) -> f64 {
    depth * ((theta - PI).cos() - alpha.cos()) / (1.0 - alpha.cos())
}

/// `∫ notch_depth dθ` over the notch, times `r`: the lens's lateral area.
fn notch_area(r: f64, alpha: f64, depth: f64) -> f64 {
    r * depth * 2.0 * alpha.cos().mul_add(-alpha, alpha.sin()) / (1.0 - alpha.cos())
}

/// Angular distance from `a` to `b` on the circle, in `[0, π]`.
fn ang_dist(a: f64, b: f64) -> f64 {
    ((a - b + PI).rem_euclid(TAU) - PI).abs()
}

/// The chart contract of an emitted region, checked against the cylinder
/// map itself: every section edge's stored `(u, v)` ends map back to its 3D
/// ends and consecutive section edges continue in `u` (no period jump inside
/// a chain), and every seam line synthesized here runs its pcurve from its
/// start `(u, v)` toward its end.
fn assert_chart_consistent(region: &SplitSubFace, surface: &FaceSurface, scale: f64, ctx: &str) {
    let on_surface = |uv: Point2, p: Point3| {
        surface
            .evaluate(uv.x(), uv.y())
            .is_some_and(|q| (q - p).length() < 1e-7 * scale.max(1.0))
    };
    let wire = &region.outer_wire;
    for (i, e) in wire.iter().enumerate() {
        match (&e.curve_3d, &e.pcurve) {
            (EdgeCurve::NurbsCurve(_), _) => {
                assert!(
                    on_surface(e.start_uv, e.start_3d) && on_surface(e.end_uv, e.end_3d),
                    "{ctx}: section edge {i} uv {:?}->{:?} is off its 3D ends",
                    e.start_uv,
                    e.end_uv
                );
                let next = &wire[(i + 1) % wire.len()];
                if matches!(next.curve_3d, EdgeCurve::NurbsCurve(_)) {
                    assert!(
                        (e.end_uv.x() - next.start_uv.x()).abs() < 1e-9,
                        "{ctx}: chain jumps in u at edge {i}: {} -> {}",
                        e.end_uv.x(),
                        next.start_uv.x()
                    );
                }
            }
            (EdgeCurve::Line, remus_math::curves2d::Curve2D::Line(l)) => {
                assert!(
                    on_surface(e.start_uv, e.start_3d) && on_surface(e.end_uv, e.end_3d),
                    "{ctx}: seam edge {i} uv is off its 3D ends"
                );
                let o = l.evaluate(0.0);
                assert!(
                    (o.x() - e.start_uv.x()).abs() < 1e-9 && (o.y() - e.start_uv.y()).abs() < 1e-9,
                    "{ctx}: seam pcurve does not start at the seam's start"
                );
                let t = l.tangent(0.0);
                let d = e.end_uv - e.start_uv;
                assert!(
                    t.x().mul_add(d.x(), t.y() * d.y()) > 0.0,
                    "{ctx}: seam pcurve runs away from the seam's end"
                );
            }
            _ => {}
        }
    }
}

/// Notch configurations: `(centre, half-width)`. The first is symmetric
/// about the antipode of the seam; the second sits in the far half, past
/// the rim's half-turn split; the last two hug the seam from either side.
const NOTCHES: [(f64, f64); 4] = [(PI, PI / 3.0), (4.5, 0.5), (0.9, 0.6), (TAU - 0.9, 0.6)];

/// A chain notching one rim of a cylinder lateral splits it into the lens
/// under the chain and the annular band that keeps both rims, seam and the
/// rest of the notched rim — whether the notch sits on the bottom or the top
/// rim, wherever it sits around the axis, whatever the scale and the rims'
/// heights, the piece order and the piece directions, and whether the face
/// is reversed.
#[test]
fn rim_notch_splits_lateral_into_band_and_lens_of_closed_form_area() {
    for (r, h) in [(1.0, 2.0), (25.0, 10.0), (0.2, 0.5)] {
        let depth = 0.45 * h;
        for z0 in [0.0, -1.5 * h, -0.5 * h, 0.5 * h] {
            for (centre, alpha) in NOTCHES {
                for on_bottom in [true, false] {
                    for reversed in [false, true] {
                        for order in 0..2 {
                            let ctx = format!(
                                "r={r} z0={z0} centre={centre} bottom={on_bottom} \
                                 reversed={reversed} order={order}"
                            );
                            check_notch(
                                r, h, z0, depth, centre, alpha, on_bottom, reversed, order, &ctx,
                            );
                        }
                    }
                }
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn check_notch(
    r: f64,
    h: f64,
    z0: f64,
    depth: f64,
    centre: f64,
    alpha: f64,
    on_bottom: bool,
    reversed: bool,
    order: u32,
    ctx: &str,
) {
    let (topo, face) = lateral_between(r, z0, z0 + h, reversed);
    let surface = topo.face(face).unwrap().surface().clone();
    let d_at = |t: f64| notch_depth(t - centre + PI, alpha, depth);
    let z_of = |t: f64| {
        if on_bottom {
            z0 + d_at(t)
        } else {
            z0 + h - d_at(t)
        }
    };
    let pieces = chain_pieces(r, &[0.0, 0.3, 0.7, 1.0], |s| {
        let t = (2.0 * alpha).mul_add(s, centre - alpha);
        (t, z_of(t))
    });
    // Scrambled orders with reversed pieces, so the chainer attaches
    // pieces at both ends and in both senses.
    let sections = if order == 0 {
        vec![
            marched(&pieces[1]),
            marched(&reversed_pts(&pieces[0])),
            marched(&pieces[2]),
        ]
    } else {
        vec![
            marched(&pieces[0]),
            marched(&reversed_pts(&pieces[1])),
            marched(&reversed_pts(&pieces[2])),
        ]
    };
    // Through the dispatcher, and the rim-chain splitter on its own: a
    // later fallback can happen to trace the same notch, which would hide a
    // splitter that wrongly declines its own cell.
    let direct = rim_chains(&topo, face, &sections)
        .unwrap_or_else(|| panic!("{ctx}: the rim-chain splitter declined its own cell"));
    for (path, regions) in [
        ("dispatch", split(&topo, face, &sections)),
        ("direct", direct),
    ] {
        let ctx = &format!("{ctx} [{path}]");
        check_notch_regions(
            &regions, &sections, &surface, r, h, z0, depth, centre, alpha, on_bottom, reversed,
            face, ctx,
        );
    }
}

#[allow(clippy::too_many_arguments)]
fn check_notch_regions(
    regions: &[SplitSubFace],
    sections: &[SectionEdge],
    surface: &FaceSurface,
    r: f64,
    h: f64,
    z0: f64,
    depth: f64,
    centre: f64,
    alpha: f64,
    on_bottom: bool,
    reversed: bool,
    face: FaceId,
    ctx: &str,
) {
    let d_at = |t: f64| notch_depth(t - centre + PI, alpha, depth);
    assert_eq!(regions.len(), 2, "{ctx}: want band + lens");
    let lens_area = notch_area(r, alpha, depth);
    let total = TAU * r * h;
    let m: Vec<Measured> = regions.iter().map(|sf| measure(sf, r)).collect();
    let (band, lens) = if m[0].area > m[1].area {
        (0, 1)
    } else {
        (1, 0)
    };
    // Fitted cubic chords against the exact ellipse: ~1e-5 relative.
    assert_close(m[lens].area, lens_area, 1e-4, &format!("{ctx}: lens area"));
    assert_close(
        m[band].area,
        total - lens_area,
        1e-4,
        &format!("{ctx}: band area"),
    );
    for (k, mk) in m.iter().enumerate() {
        assert!(
            mk.net_turn.abs() < 1e-6,
            "{ctx}: region {k} winds {}",
            mk.net_turn
        );
        assert!(
            (mk.interior.2 - r).abs() < 1e-9 * r.max(1.0),
            "{ctx}: interior off the wall"
        );
        assert_eq!(regions[k].reversed, reversed, "{ctx}");
        assert_eq!(regions[k].parent, face, "{ctx}");
        assert_eq!(regions[k].rank, Rank::A, "{ctx}");
        assert!(regions[k].inner_wires.is_empty(), "{ctx}");
        assert_chart_consistent(&regions[k], surface, r, ctx);
    }
    // The lens interior lies under the chain, the band's above it or
    // outside the notch.
    let depth_at = |z: f64| if on_bottom { z - z0 } else { z0 + h - z };
    // Both samples also keep clear of their region's boundary: along their
    // meridian they sit in the middle half of the region's extent, not
    // grazing a rim or the chain where a classifier ray starts on-boundary.
    let middle_half = |x: f64, lo: f64, hi: f64| {
        let f = (x - lo) / (hi - lo);
        f > 0.25 && f < 0.75
    };
    let (lt, lz, _) = m[lens].interior;
    assert!(
        ang_dist(lt, centre) < alpha && middle_half(depth_at(lz), 0.0, d_at(lt)),
        "{ctx}: lens interior {:?} is not well under the chain",
        m[lens].interior
    );
    let (bt, bz, _) = m[band].interior;
    let band_floor = if ang_dist(bt, centre) < alpha {
        d_at(bt)
    } else {
        0.0
    };
    assert!(
        middle_half(depth_at(bz), band_floor, h),
        "{ctx}: band interior {:?} is not well inside the band",
        m[band].interior
    );
    assert_sections_shared(regions, sections, ctx);
    // The band keeps the whole far rim and both seam uses.
    let far_z = if on_bottom { z0 + h } else { z0 };
    let band_wire = &regions[band].outer_wire;
    assert_eq!(
        band_wire
            .iter()
            .filter(|e| matches!(e.curve_3d, EdgeCurve::Line))
            .count(),
        2,
        "{ctx}: band must carry the seam both ways"
    );
    assert!(
        band_wire
            .iter()
            .any(|e| matches!(e.curve_3d, EdgeCurve::Circle(_))
                && (e.start_3d - e.end_3d).length() < WELD
                && (e.start_3d.z() - far_z).abs() < WELD),
        "{ctx}: band lost the far rim"
    );
}

/// Two chains, each from one rim to the other, cut the lateral into the
/// sector clear of the seam and the seam-side sector. The chains are helices
/// `θ = θ_i + λ_i·(z − z0)/h` of different leans, so the clear sector's area
/// is `r·h·((θ_b − θ_a) + (λ_b − λ_a)/2)`.
#[test]
fn rim_to_rim_chains_split_lateral_into_two_sectors_of_closed_form_area() {
    // (θ_a, λ_a, θ_b, λ_b): symmetric, asymmetric, and a clear span past
    // the rim's half-turn split.
    let configs: [(f64, f64, f64, f64); 3] = [
        (2.0 * PI / 3.0, 0.4, 4.0 * PI / 3.0, 0.4),
        (2.0, 0.4, 3.9, -0.3),
        (0.5, 0.3, 4.6, -0.2),
    ];
    for (r, h) in [(1.0, 2.0), (25.0, 10.0), (0.2, 0.5)] {
        for z0 in [0.0, -1.5 * h, -0.5 * h, 0.5 * h] {
            for (ta, la, tb, lb) in configs {
                for reversed in [false, true] {
                    for flip in [false, true] {
                        let ctx = format!(
                            "r={r} z0={z0} chains=({ta},{la})/({tb},{lb}) reversed={reversed} flip={flip}"
                        );
                        let (topo, face) = lateral_between(r, z0, z0 + h, reversed);
                        let surface = topo.face(face).unwrap().surface().clone();
                        let a = chain_pieces(r, &[0.0, 0.5, 1.0], |s| {
                            (la.mul_add(s, ta), h.mul_add(s, z0))
                        });
                        let b = chain_pieces(r, &[0.0, 0.4, 1.0], |s| {
                            (lb.mul_add(s, tb), h.mul_add(s, z0))
                        });
                        // One chain walks top-down; pieces interleave across chains.
                        let sections = if flip {
                            vec![
                                marched(&reversed_pts(&b[1])),
                                marched(&a[0]),
                                marched(&reversed_pts(&b[0])),
                                marched(&a[1]),
                            ]
                        } else {
                            vec![
                                marched(&a[1]),
                                marched(&b[0]),
                                marched(&a[0]),
                                marched(&b[1]),
                            ]
                        };
                        let direct = rim_chains(&topo, face, &sections).unwrap_or_else(|| {
                            panic!("{ctx}: the rim-chain splitter declined its own cell")
                        });
                        for (path, regions) in [
                            ("dispatch", split(&topo, face, &sections)),
                            ("direct", direct),
                        ] {
                            let ctx = format!("{ctx} [{path}]");
                            assert_eq!(regions.len(), 2, "{ctx}: want two sectors");
                            let m: Vec<Measured> =
                                regions.iter().map(|sf| measure(sf, r)).collect();
                            let clear_area = r * h * (tb - ta + 0.5 * (lb - la));
                            let clear_is_0 =
                                (m[0].area - clear_area).abs() < (m[1].area - clear_area).abs();
                            let (clear, seam_side) = if clear_is_0 { (0, 1) } else { (1, 0) };
                            assert_close(
                                m[clear].area,
                                clear_area,
                                1e-6,
                                &format!("{ctx}: clear sector"),
                            );
                            assert_close(
                                m[seam_side].area,
                                TAU.mul_add(r * h, -clear_area),
                                1e-6,
                                &format!("{ctx}: seam sector"),
                            );
                            for (k, mk) in m.iter().enumerate() {
                                assert!(
                                    mk.net_turn.abs() < 1e-6,
                                    "{ctx}: region {k} winds {}",
                                    mk.net_turn
                                );
                                assert!((mk.interior.2 - r).abs() < 1e-9 * r.max(1.0), "{ctx}");
                                // Mid-wall along its meridian, clear of both rims.
                                let f = (mk.interior.1 - z0) / h;
                                assert!(f > 0.25 && f < 0.75, "{ctx}: interior height {f}");
                                assert_eq!(regions[k].reversed, reversed, "{ctx}");
                                assert_chart_consistent(&regions[k], &surface, r, &ctx);
                            }
                            let between = |(t, z, _): (f64, f64, f64)| {
                                let f = (z - z0) / h;
                                t > la.mul_add(f, ta) && t < lb.mul_add(f, tb)
                            };
                            assert!(
                                between(m[clear].interior),
                                "{ctx}: clear interior {:?}",
                                m[clear].interior
                            );
                            assert!(
                                !between(m[seam_side].interior),
                                "{ctx}: seam interior {:?}",
                                m[seam_side].interior
                            );
                            assert_sections_shared(&regions, &sections, &ctx);
                            // Only the seam-side sector touches the seam, and it
                            // carries it both ways.
                            let seams = |k: usize| {
                                regions[k]
                                    .outer_wire
                                    .iter()
                                    .filter(|e| matches!(e.curve_3d, EdgeCurve::Line))
                                    .count()
                            };
                            assert_eq!((seams(seam_side), seams(clear)), (2, 0), "{ctx}");
                        }
                    }
                }
            }
        }
    }
}

/// Marched chains close only to within the weld scale: joints between pieces
/// and the chain's rim contacts carry ~1e-6 of fit error. The splitter must
/// chain and anchor them at `100 · tol`, not at `tol`.
#[test]
fn rim_chains_weld_fit_error_gaps_at_the_weld_scale() {
    let (r, h) = (1.0, 2.0);
    let (topo, face) = lateral(r, h, false);
    let alpha = PI / 3.0;
    let depth = 0.9;
    let mut pieces = chain_pieces(r, &[0.0, 0.3, 0.7, 1.0], |s| {
        let t = (2.0 * alpha).mul_add(s, PI - alpha);
        (t, notch_depth(t, alpha, depth))
    });
    // Open the joint by 3e-6 along the wall (30 · tol, a third of the weld).
    let joint = pieces[1][0];
    let nudged = on_cyl(r, joint.y().atan2(joint.x()) + 3e-6, joint.z());
    pieces[1][0] = nudged;
    let sections: Vec<SectionEdge> = pieces.iter().map(|p| marched(p)).collect();
    let regions = split(&topo, face, &sections);
    assert_eq!(regions.len(), 2);
    let m: Vec<Measured> = regions.iter().map(|sf| measure(sf, r)).collect();
    let lens = if m[0].area < m[1].area { 0 } else { 1 };
    assert_close(
        m[lens].area,
        notch_area(r, alpha, depth),
        1e-4,
        "welded lens area",
    );
}

fn notch_sections(r: f64, alpha: f64, depth: f64, centre: f64) -> Vec<SectionEdge> {
    chain_pieces(r, &[0.0, 0.5, 1.0], |s| {
        let t = (2.0 * alpha).mul_add(s, centre - alpha);
        (t, notch_depth(t - centre + PI, alpha, depth))
    })
    .iter()
    .map(|p| marched(p))
    .collect()
}

/// Configurations outside the documented cell decline (`None`) so the caller
/// falls through to the calibrated paths: a band assembled around any of them
/// would be wrong.
#[test]
fn rim_chains_decline_outside_their_cell() {
    let (r, h) = (1.0, 2.0);
    let (topo, face) = lateral(r, h, false);
    let alpha = PI / 3.0;
    // In-cell reference: accepted.
    assert!(rim_chains(&topo, face, &notch_sections(r, alpha, 0.9, PI)).is_some());

    // No sections.
    assert!(rim_chains(&topo, face, &[]).is_none(), "empty");
    // Straight (non-marched) sections keep the ruling/rectilinear paths.
    let line = SectionEdge {
        curve_3d: EdgeCurve::Line,
        ..marched(&[
            on_cyl(r, 2.0, 0.0),
            on_cyl(r, 2.0, 1.0),
            on_cyl(r, 2.0, 2.0),
        ])
    };
    assert!(rim_chains(&topo, face, &[line]).is_none(), "line section");
    // A notch straddling the seam meridian.
    assert!(
        rim_chains(&topo, face, &notch_sections(r, alpha, 0.9, 0.0)).is_none(),
        "notch across the seam"
    );
    // A notch wider than half a period.
    assert!(
        rim_chains(&topo, face, &notch_sections(r, 0.6 * PI, 0.9, PI)).is_none(),
        "notch wider than π"
    );
    // A notch that reaches the far rim is not strictly between the rims.
    let tall = chain_pieces(r, &[0.0, 0.5, 1.0], |s| {
        let t = (2.0 * alpha).mul_add(s, PI - alpha);
        (t, notch_depth(t, alpha, h))
    });
    assert!(
        rim_chains(
            &topo,
            face,
            &tall.iter().map(|p| marched(p)).collect::<Vec<_>>()
        )
        .is_none(),
        "chain touching the far rim"
    );
    // A closed internal loop (a hole, not a rim chain).
    let hole = chain_pieces(r, &[0.0, 0.5, 1.0], |s| {
        let a = TAU * s;
        (0.3f64.mul_add(a.cos(), PI), 0.3f64.mul_add(a.sin(), 1.0))
    });
    assert!(
        rim_chains(
            &topo,
            face,
            &hole.iter().map(|p| marched(p)).collect::<Vec<_>>()
        )
        .is_none(),
        "closed loop"
    );
    // A chain ending mid-wall.
    let dangling = chain_pieces(r, &[0.0, 1.0], |s| (PI, 1.5 * s));
    assert!(
        rim_chains(&topo, face, &[marched(&dangling[0])]).is_none(),
        "chain ending off the rims"
    );
    // A notch plus a rim-to-rim chain is neither a notch nor a sector pair.
    let mut mixed = notch_sections(r, 0.4, 0.5, PI);
    let cross = chain_pieces(r, &[0.0, 1.0], |s| (0.2f64.mul_add(s, 2.0), h * s));
    mixed.push(marched(&cross[0]));
    assert!(
        rim_chains(&topo, face, &mixed).is_none(),
        "notch + rim-to-rim"
    );
    // Three disjoint chains.
    let mut three = notch_sections(r, 0.3, 0.5, 2.0);
    three.extend(notch_sections(r, 0.3, 0.5, 3.0));
    three.extend(notch_sections(r, 0.3, 0.5, 4.2));
    assert!(rim_chains(&topo, face, &three).is_none(), "three chains");
    // A lone rim-to-rim chain is neither a notch nor a sector pair.
    let lone = chain_pieces(r, &[0.0, 1.0], |s| (0.3f64.mul_add(s, 2.0), h * s));
    assert!(
        rim_chains(&topo, face, &[marched(&lone[0])]).is_none(),
        "lone rim-to-rim chain"
    );
    // A sector pair whose second chain crosses the seam meridian.
    let a = chain_pieces(r, &[0.0, 1.0], |s| (0.3f64.mul_add(s, 2.8), h * s));
    let b = chain_pieces(r, &[0.0, 1.0], |s| (0.4f64.mul_add(s, -0.2), h * s));
    assert!(
        rim_chains(&topo, face, &[marched(&a[0]), marched(&b[0])]).is_none(),
        "sector chain across the seam"
    );
    // A boundary with one rim (the top rim dropped) is not a two-rim lateral.
    {
        let f = topo.face(face).unwrap();
        let surface = f.surface().clone();
        let pts = collect_wire_points(&topo, f.outer_wire());
        let mut boundary =
            boundary_edges_to_pcurve(&topo, f.outer_wire(), &surface, &pts, None).unwrap();
        boundary.retain(|e| !(matches!(e.curve_3d, EdgeCurve::Circle(_)) && e.start_3d.z() > 1.0));
        assert!(
            split_periodic_face_by_rim_chains(
                &surface,
                &boundary,
                &notch_sections(r, alpha, 0.9, PI),
                Rank::A,
                false,
                face,
                TOL
            )
            .unwrap()
            .is_none(),
            "one-rim boundary"
        );
    }
    // A W-shaped chain whose middle joint touches the rim is two notches
    // sharing a rim point, not one; the rims sit off `z = 0` so no
    // tolerance band can be mistaken for the rim height.
    let (lifted, lifted_face) = lateral_between(r, 0.5, 2.5, false);
    let bump = |t: f64, t0: f64, t1: f64| 0.6 * ((t - t0) * PI / (t1 - t0)).sin();
    let w: Vec<SectionEdge> = [(2.5_f64, 3.1_f64), (3.1, 3.7)]
        .iter()
        .map(|&(t0, t1)| {
            marched(
                &chain_pieces(r, &[0.0, 1.0], |s| {
                    let t = (t1 - t0).mul_add(s, t0);
                    (t, 0.5 + bump(t, t0, t1))
                })[0],
            )
        })
        .collect();
    assert!(
        rim_chains(&lifted, lifted_face, &w).is_none(),
        "chain touching the rim mid-way"
    );
    // Two rim-to-rim chains that cross each other.
    let a = chain_pieces(r, &[0.0, 1.0], |s| (1.2f64.mul_add(s, 2.0), h * s));
    let b = chain_pieces(r, &[0.0, 1.0], |s| (1.2f64.mul_add(-s, 3.2), h * s));
    assert!(
        rim_chains(&topo, face, &[marched(&a[0]), marched(&b[0])]).is_none(),
        "crossing chains"
    );
}

fn line_section(start: Point3, end: Point3) -> SectionEdge {
    SectionEdge {
        curve_3d: EdgeCurve::Line,
        ..marched(&[start, start + (end - start) * 0.5, end])
    }
}

fn ring_section(r: f64, z: f64) -> SectionEdge {
    let circle = Circle3D::new(Point3::new(0.0, 0.0, z), Vec3::new(0.0, 0.0, 1.0), r).unwrap();
    let seam = on_cyl(r, 0.0, z);
    let t0 = circle.project(seam);
    SectionEdge {
        curve_3d: EdgeCurve::Circle(circle),
        trim: Some((t0, t0 + TAU)),
        ..line_section(seam, seam)
    }
}

/// Closed section rings split the lateral into stacked bands of height
/// `Δz`, each of area `2πr·Δz`, ordered bottom to top and each keeping the
/// seam both ways.
#[test]
fn closed_rings_split_lateral_into_bands_of_closed_form_area() {
    for (r, h) in [(1.0, 2.0), (25.0, 10.0)] {
        for levels in [vec![0.3 * h], vec![0.25 * h, 0.7 * h]] {
            for reversed in [false, true] {
                let (topo, face) = lateral(r, h, reversed);
                let sections: Vec<SectionEdge> =
                    levels.iter().map(|&z| ring_section(r, z)).collect();
                let ctx = format!("r={r} levels={levels:?} reversed={reversed}");
                let regions = split(&topo, face, &sections);
                assert_eq!(regions.len(), levels.len() + 1, "{ctx}");
                let mut cuts = vec![0.0];
                cuts.extend(levels.iter().copied());
                cuts.push(h);
                let mut got: Vec<(f64, f64)> = regions
                    .iter()
                    .map(|sf| {
                        let m = measure(sf, r);
                        assert!(m.net_turn.abs() < 1e-6, "{ctx}: band winds");
                        assert_eq!(sf.reversed, reversed, "{ctx}");
                        assert!(
                            (m.interior.2 - r).abs() < 1e-9 * r,
                            "{ctx}: interior off the wall"
                        );
                        (m.interior.1, m.area)
                    })
                    .collect();
                got.sort_by(|a, b| a.0.total_cmp(&b.0));
                for (k, (z_in, area)) in got.iter().enumerate() {
                    let (lo, hi) = (cuts[k], cuts[k + 1]);
                    assert!(
                        *z_in > lo && *z_in < hi,
                        "{ctx}: band {k} interior z={z_in}"
                    );
                    assert_close(
                        *area,
                        TAU * r * (hi - lo),
                        1e-6,
                        &format!("{ctx}: band {k}"),
                    );
                }
            }
        }
    }
}

/// One full-height ruling plus the seam cut the lateral into two sectors
/// (the mid-wall pad whose second wall crossing rides the seam). The greedy
/// walker reads the once-cut annulus as a single region; the sector rescue
/// must return both, of areas `r·θ·h` and `r·(2π − θ)·h`.
#[test]
fn one_ruling_splits_lateral_into_two_sectors_of_closed_form_area() {
    for (r, h) in [(1.0, 2.0), (25.0, 10.0)] {
        for theta in [2.0, 4.5] {
            for downward in [false, true] {
                let (topo, face) = lateral(r, h, false);
                let (a, b) = (on_cyl(r, theta, 0.0), on_cyl(r, theta, h));
                let ruling = if downward {
                    line_section(b, a)
                } else {
                    line_section(a, b)
                };
                let ctx = format!("r={r} theta={theta} downward={downward}");
                let regions = split(&topo, face, &[ruling]);
                assert_eq!(regions.len(), 2, "{ctx}: want two sectors");
                let mut total = 0.0;
                for sf in &regions {
                    let m = measure(sf, r);
                    assert!(m.net_turn.abs() < 1e-6, "{ctx}: sector winds");
                    let (t, z, _) = m.interior;
                    assert!(z > 0.0 && z < h, "{ctx}");
                    let width = if t < theta { theta } else { TAU - theta };
                    assert_close(
                        m.area,
                        r * width * h,
                        1e-6,
                        &format!("{ctx}: sector at θ={t}"),
                    );
                    total += m.area;
                }
                assert_close(
                    total,
                    TAU * r * h,
                    1e-9,
                    &format!("{ctx}: sectors tile the lateral"),
                );
            }
        }
    }
}

/// The upper hemisphere of a sphere of radius `rad` about the origin, bounded
/// (like a faceted import) by an `n`-gon inscribed in the equator, traversed
/// counter-clockwise from +z so the face lies above it.
fn hemisphere(rad: f64, n: u32) -> (Topology, FaceId) {
    let mut topo = Topology::new();
    let verts: Vec<_> = (0..n)
        .map(|k| {
            let a = TAU * f64::from(k) / f64::from(n);
            topo.add_vertex(Vertex::new(
                Point3::new(rad * a.cos(), rad * a.sin(), 0.0),
                TOL,
            ))
        })
        .collect();
    let edges = (0..verts.len())
        .map(|i| {
            OrientedEdge::new(
                topo.add_edge(Edge::new(
                    verts[i],
                    verts[(i + 1) % verts.len()],
                    EdgeCurve::Line,
                )),
                true,
            )
        })
        .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    let sphere =
        remus_math::surfaces::SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), rad).unwrap();
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Sphere(sphere)));
    (topo, face)
}

/// A section arc of `circle` from `a` to `b` with its pcurve on `surface`
/// computed the way phase FF does.
fn arc_section(circle: &Circle3D, a: Point3, b: Point3, surface: &FaceSurface) -> SectionEdge {
    let curve = EdgeCurve::Circle(circle.clone());
    let pcurve =
        crate::builder::pcurve_compute::compute_pcurve_on_surface(&curve, a, b, surface, &[], None)
            .unwrap();
    // The exact interval, counter-clockwise about the circle's normal.
    let ta = circle.project(a);
    let tb = ta + (circle.project(b) - ta).rem_euclid(TAU);
    SectionEdge {
        curve_3d: curve,
        trim: Some((ta, tb)),
        pcurve_a: pcurve.clone(),
        pcurve_b: pcurve,
        ..line_section(a, b)
    }
}

/// Samples of a circular wire edge. With an exact trim, the trimmed span in
/// the curve's own parameter, oriented by the edge's stored ends (which it
/// must reach). Without one, a closed circle is a full turn in the sense its
/// `forward` flag gives, and an open arc is its minor arc (every untrimmed
/// arc in these fixtures spans less than π).
fn minor_arc(e: &OrientedPCurveEdge) -> Vec<Point3> {
    let EdgeCurve::Circle(c) = &e.curve_3d else {
        panic!(
            "collar wires are all circular, got {}",
            e.curve_3d.type_tag()
        );
    };
    if let Some((t0, t1)) = e.trim.filter(|_| (e.start_3d - e.end_3d).length() > 1e-9) {
        let mut pts: Vec<Point3> = (0..=1024)
            .map(|k| c.evaluate((t1 - t0).mul_add(f64::from(k) / 1024.0, t0)))
            .collect();
        if (pts[0] - e.start_3d).length() > (pts[1024] - e.start_3d).length() {
            pts.reverse();
        }
        assert!(
            (pts[0] - e.start_3d).length() < WELD && (pts[1024] - e.end_3d).length() < WELD,
            "trimmed arc does not run between its stored ends"
        );
        return pts;
    }
    let (n, ctr) = (c.normal(), c.center());
    let (s, t) = (e.start_3d - ctr, e.end_3d - ctr);
    let sweep = if (e.start_3d - e.end_3d).length() < 1e-9 {
        if e.forward { TAU } else { -TAU }
    } else {
        s.cross(t).dot(n).atan2(s.dot(t))
    };
    (0..=1024)
        .map(|k| {
            let (sn, cs) = (sweep * f64::from(k) / 1024.0).sin_cos();
            // Rodrigues rotation of `s` about the unit normal `n`.
            ctr + s * cs + n.cross(s) * sn + n * (n.dot(s) * (1.0 - cs))
        })
        .collect()
}

/// A hemisphere cut by the four walls `|x| = a`, `|y| = a` of a box that
/// swallows its pole (a > r/√2, so the walls' arcs stay disjoint) is the
/// box–sphere collar: the arcs cannot chain, and the arrangement of seam arcs
/// and wall arcs must return all five bounded cells — the collar inside the
/// box and one cap beyond each wall. Seen from +z each wall arc projects onto
/// its wall line, so a cap covers the circular segment
/// `r²·acos(a/r) − a·√(r² − a²)` and the collar the disc minus four of them.
#[test]
fn box_walls_split_faceted_hemisphere_into_collar_and_four_caps() {
    for (rad, frac) in [(2.0_f64, 0.8), (50.0, 0.75)] {
        let a = frac * rad;
        let rho = (rad * rad - a * a).sqrt();
        let (topo, face) = hemisphere(rad, 64);
        let surface = topo.face(face).unwrap().surface().clone();
        let mut sections = Vec::new();
        for k in 0..4 {
            let (sn, cs) = (f64::from(k) * PI / 2.0).sin_cos();
            // Wall k has outward normal (cs, sn, 0); its arc on the hemisphere
            // runs from one equator crossing over the apex to the other, which
            // is counter-clockwise about the INWARD normal (the phase-FF sense,
            // with an exact trim).
            let rot = |x: f64, y: f64, z: f64| Point3::new(x * cs - y * sn, x * sn + y * cs, z);
            let circle = Circle3D::new(rot(a, 0.0, 0.0), Vec3::new(-cs, -sn, 0.0), rho).unwrap();
            let (lo, apex, hi) = (rot(a, -rho, 0.0), rot(a, 0.0, rho), rot(a, rho, 0.0));
            sections.push(arc_section(&circle, lo, apex, &surface));
            sections.push(arc_section(&circle, apex, hi, &surface));
        }
        let ctx = format!("r={rad} a={a}");
        let regions = split(&topo, face, &sections);
        assert_eq!(regions.len(), 5, "{ctx}: collar + four caps");
        let segment = rad * rad * (a / rad).acos() - a * rho;
        let mut caps = [false; 4];
        let mut collar = false;
        for sf in &regions {
            let mut pts = Vec::new();
            for (i, e) in sf.outer_wire.iter().enumerate() {
                let next = &sf.outer_wire[(i + 1) % sf.outer_wire.len()];
                assert!(
                    (e.end_3d - next.start_3d).length() < WELD,
                    "{ctx}: open wire"
                );
                pts.extend(minor_arc(e));
            }
            let twice: f64 = (0..pts.len())
                .map(|i| {
                    let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
                    p.x().mul_add(q.y(), -(q.x() * p.y()))
                })
                .sum();
            // 1024-chord samples of each arc: ~1e-7 relative sagitta deficit.
            let area = 0.5 * twice;
            let p = sf
                .precomputed_interior
                .expect("collar cells carry an interior");
            assert!(
                ((p - Point3::new(0.0, 0.0, 0.0)).length() - rad).abs() < 1e-9 * rad,
                "{ctx}"
            );
            assert!(p.z() > 0.0, "{ctx}: interior below the equator");
            assert!(!sf.reversed, "{ctx}");
            if p.x().abs() < a && p.y().abs() < a {
                assert!(!collar, "{ctx}: two collars");
                collar = true;
                assert_close(
                    area,
                    PI.mul_add(rad * rad, -4.0 * segment),
                    1e-5,
                    &format!("{ctx}: collar"),
                );
            } else {
                let k = if p.x() > a {
                    0
                } else if p.y() > a {
                    1
                } else if p.x() < -a {
                    2
                } else {
                    3
                };
                assert!(!caps[k], "{ctx}: cap {k} twice");
                caps[k] = true;
                assert_close(area, segment, 1e-5, &format!("{ctx}: cap {k}"));
            }
        }
        assert!(collar && caps.iter().all(|&c| c), "{ctx}: missing a cell");
    }
}

/// Signed area of a hemisphere wire projected onto the equator plane
/// (positive counter-clockwise from +z), from exact circle samples.
fn projected_area(wire: &[OrientedPCurveEdge], ctx: &str) -> f64 {
    let mut pts = Vec::new();
    for (i, e) in wire.iter().enumerate() {
        let next = &wire[(i + 1) % wire.len()];
        assert!(
            (e.end_3d - next.start_3d).length() < WELD,
            "{ctx}: open wire"
        );
        pts.extend(minor_arc(e));
    }
    0.5 * (0..pts.len())
        .map(|i| {
            let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
            p.x().mul_add(q.y(), -(q.x() * p.y()))
        })
        .sum::<f64>()
}

/// The two routes of the collar splitter the four-wall test does not take.
///
/// - One wall `x = a` leaves a single open chain and two seam arcs: the
///   exact two-patch route must return the cap beyond the wall (the
///   circular segment `r²·acos(a/r) − a·√(r² − a²)` from +z) and the rest.
/// - Four walls plus a lid `z = c` below the pole (whose latitude circle,
///   of radius `ρ_c = √(r² − c²) < a`, stays inside the walls) add a closed
///   section: the collar must carry it as a hole and the lid's cap
///   (projected area `π·ρ_c²`) come back as its own cell.
#[test]
fn one_wall_and_lidded_box_split_faceted_hemisphere_exactly() {
    for rad in [2.0, 50.0] {
        check_one_wall_and_lid(rad, 0.85);
    }
}

/// B59: with the walls close under the lid the collar's classification
/// sample (the lid latitude nudged a fixed amount toward the equator) lands
/// beyond a wall, in a wall cap.
#[test]
#[ignore = "open: B59 — collar interior sample overshoots a wall close under the lid"]
fn b59_collar_sample_stays_inside_walls_close_under_the_lid() {
    for rad in [2.0, 50.0] {
        check_one_wall_and_lid(rad, 0.75);
    }
}

fn check_one_wall_and_lid(rad: f64, frac: f64) {
    let a = frac * rad;
    let rho = (rad * rad - a * a).sqrt();
    let segment = rad * rad * (a / rad).acos() - a * rho;
    for lid in [false, true] {
        let (topo, face) = hemisphere(rad, 64);
        let surface = topo.face(face).unwrap().surface().clone();
        let walls = if lid { 4 } else { 1 };
        let mut sections = Vec::new();
        for k in 0..walls {
            let (sn, cs) = (f64::from(k) * PI / 2.0).sin_cos();
            let rot = |x: f64, y: f64, z: f64| Point3::new(x * cs - y * sn, x * sn + y * cs, z);
            let circle = Circle3D::new(rot(a, 0.0, 0.0), Vec3::new(-cs, -sn, 0.0), rho).unwrap();
            let (lo, apex, hi) = (rot(a, -rho, 0.0), rot(a, 0.0, rho), rot(a, rho, 0.0));
            sections.push(arc_section(&circle, lo, apex, &surface));
            sections.push(arc_section(&circle, apex, hi, &surface));
        }
        let c = 0.5 * (rho + rad);
        let rho_c = c.mul_add(-c, rad * rad).sqrt();
        if lid {
            let ring =
                Circle3D::new(Point3::new(0.0, 0.0, c), Vec3::new(0.0, 0.0, 1.0), rho_c).unwrap();
            let p = Point3::new(rho_c, 0.0, c);
            let t0 = ring.project(p);
            let curve = EdgeCurve::Circle(ring);
            let pcurve = crate::builder::pcurve_compute::compute_pcurve_on_surface(
                &curve,
                p,
                p,
                &surface,
                &[],
                None,
            )
            .unwrap();
            sections.push(SectionEdge {
                curve_3d: curve,
                trim: Some((t0, t0 + TAU)),
                pcurve_a: pcurve.clone(),
                pcurve_b: pcurve,
                ..line_section(p, p)
            });
        }
        let ctx = format!("r={rad} a={a} lid={lid}");
        let regions = split(&topo, face, &sections);
        let disc = PI * rad * rad;
        let lid_area = PI * rho_c * rho_c;
        let mut seen = Vec::new();
        for sf in &regions {
            let p = sf
                .precomputed_interior
                .expect("collar cells carry an interior");
            assert!(
                ((p - Point3::new(0.0, 0.0, 0.0)).length() - rad).abs() < 1e-9 * rad,
                "{ctx}"
            );
            assert!(p.z() > 0.0, "{ctx}: interior below the equator");
            assert_eq!(sf.parent, face, "{ctx}");
            let outer = projected_area(&sf.outer_wire, &ctx);
            let holes: f64 = sf.inner_wires.iter().map(|w| projected_area(w, &ctx)).sum();
            let beyond_wall = p.x() > a || p.y() > a || p.x() < -a || p.y() < -a;
            let (name, want_outer, want_holes) = if !lid {
                if p.x() > a {
                    ("cap", segment, 0.0)
                } else {
                    ("rest", disc - segment, 0.0)
                }
            } else if beyond_wall {
                ("wall cap", segment, 0.0)
            } else if p.z() > c {
                ("lid cap", lid_area, 0.0)
            } else {
                ("collar", 4.0f64.mul_add(-segment, disc), -lid_area)
            };
            assert_close(outer, want_outer, 1e-5, &format!("{ctx}: {name} outer"));
            if sf.inner_wires.is_empty() {
                assert!(want_holes == 0.0, "{ctx}: {name} lost its hole");
            } else {
                assert_eq!(sf.inner_wires.len(), 1, "{ctx}: {name} holes");
                assert_close(holes, want_holes, 1e-5, &format!("{ctx}: {name} hole"));
            }
            seen.push(name);
        }
        seen.sort_unstable();
        let want: Vec<&str> = if lid {
            vec![
                "collar", "lid cap", "wall cap", "wall cap", "wall cap", "wall cap",
            ]
        } else {
            vec!["cap", "rest"]
        };
        assert_eq!(seen, want, "{ctx}: cells");
    }
}
