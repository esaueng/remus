//! Closed-form oracles for the face splitter's periodic rim-chain machinery
//! (B19 splitter mutant tranche, 2026-09-27 run): the rim-chain splitter
//! ([`split_periodic_face_by_rim_chains`]), the orphaned-rim detector
//! ([`loops_orphan_boundary_edges`]) and the cone-lateral dispatch of
//! [`split_face_2d`].
//!
//! Nothing here reads a splitter's arithmetic back:
//! - orphan detection is probed with hand-built boundary and loop lists whose
//!   coverage is known, with endpoints placed `0.24` and `0.26` cells apart
//!   across a quantization boundary, arcs exactly on either side of the
//!   `span · radius ≤ tol` floor, and two co-endpoint rim halves whose
//!   mutated mid keys would coincide;
//! - every rim-chain decision is probed two-sided at the `100 · tol` weld
//!   scale with dyadic tolerances (`2⁻¹⁰`, `2⁻²⁰`) so a distance can equal
//!   the weld exactly: a section piece exactly one weld long, a junction gap
//!   exactly one weld wide, a rim whose start sits exactly one weld off the
//!   seam meridian, and chains ending half a weld from the seam;
//! - interior points are compared with the closed-form height of the chain
//!   on the probe meridian (an asymmetric notch profile evaluated at the
//!   notch centre; a straight strip's mid-height where no chain crosses);
//! - regions are measured from their 3D wires in `(r·θ, z)`, as in
//!   `closed_form_split_tests.rs`.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp
)]

use super::*;
use remus_math::curves::Circle3D;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface};
use remus_topology::{
    edge::Edge,
    face::Face,
    vertex::Vertex,
    wire::{OrientedEdge, Wire},
};
use std::f64::consts::{PI, TAU};

// ── Shared fixture builders ─────────────────────────────────────────────

fn dummy_pcurve() -> remus_math::curves2d::Curve2D {
    remus_math::curves2d::Curve2D::Line(
        remus_math::curves2d::Line2D::new(
            Point2::new(0.0, 0.0),
            remus_math::vec::Vec2::new(1.0, 0.0),
        )
        .unwrap(),
    )
}

/// A boundary/loop edge carrying only what `loops_orphan_boundary_edges`
/// reads: the 3D curve, the 3D ends and the traversal flag.
fn edge3(curve_3d: EdgeCurve, start: Point3, end: Point3, forward: bool) -> OrientedPCurveEdge {
    OrientedPCurveEdge {
        curve_3d,
        trim: None,
        pcurve: dummy_pcurve(),
        start_uv: Point2::new(0.0, 0.0),
        end_uv: Point2::new(0.0, 0.0),
        start_3d: start,
        end_3d: end,
        forward,
        source_edge_idx: None,
        pave_block_id: None,
        source_topo_edge: None,
    }
}
fn line3(start: Point3, end: Point3) -> OrientedPCurveEdge {
    edge3(EdgeCurve::Line, start, end, true)
}
fn z_circle(center: Point3, r: f64) -> Circle3D {
    Circle3D::new_with_ref(
        center,
        Vec3::new(0.0, 0.0, 1.0),
        r,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap()
}
/// A forward arc of the circle `c` from angle `a0` to `a1`.
fn arc3(c: &Circle3D, a0: f64, a1: f64) -> OrientedPCurveEdge {
    edge3(
        EdgeCurve::Circle(c.clone()),
        c.evaluate(a0),
        c.evaluate(a1),
        true,
    )
}
fn pt(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

// ── loops_orphan_boundary_edges (mod.rs 2740–2790) ──────────────────────

#[test]
fn orphan_detection_is_direction_agnostic_and_skips_zero_extent_lines() {
    let tol = 0.5;
    let a = pt(0.0, 0.0, 0.0);
    let b = pt(5.0, 0.0, 0.0);
    let boundary = vec![line3(a, b)];
    // Uncovered: orphaned. Covered by a clone, or by its reversed twin
    // (swapped ends, `forward = false`): not orphaned.
    assert!(loops_orphan_boundary_edges(&[], &boundary, tol));
    assert!(!loops_orphan_boundary_edges(
        std::slice::from_ref(&boundary),
        &boundary,
        tol
    ));
    let twin = edge3(EdgeCurve::Line, b, a, false);
    assert!(!loops_orphan_boundary_edges(&[vec![twin]], &boundary, tol));
    // A second, uncovered boundary line orphans even when the first is covered.
    let mut two = boundary.clone();
    two.push(line3(b, pt(5.0, 5.0, 0.0)));
    assert!(loops_orphan_boundary_edges(
        std::slice::from_ref(&boundary),
        &two,
        tol
    ));
    // A zero-extent line (ends within tol) carries no region: never orphaned.
    let dot = vec![line3(a, pt(0.4, 0.0, 0.0))];
    assert!(!loops_orphan_boundary_edges(&[], &dot, tol));
    let dot = vec![line3(a, pt(0.6, 0.0, 0.0))];
    assert!(loops_orphan_boundary_edges(&[], &dot, tol));
}

/// Endpoints are quantized to `1/tol` cells by rounding each coordinate: a
/// loop end `0.24` cells from the boundary end rounds to the same cell (and
/// covers it), one `0.26` cells away does not. Probed per axis.
#[test]
fn endpoint_quantization_rounds_each_axis_to_the_nearest_cell() {
    let tol = 0.5;
    let a = pt(0.0, 0.0, 0.0);
    let b = pt(5.0, 0.0, 0.0);
    let boundary = vec![line3(a, b)];
    for axis in 0..3 {
        let shifted = |d: f64| match axis {
            0 => pt(d, 0.0, 0.0),
            1 => pt(0.0, d, 0.0),
            _ => pt(0.0, 0.0, d),
        };
        // 0.24 · 2 = 0.48 rounds to cell 0 with the boundary end.
        let near = vec![line3(shifted(0.24), b)];
        assert!(
            !loops_orphan_boundary_edges(&[near], &boundary, tol),
            "axis {axis}: 0.24 must share the cell"
        );
        let near = vec![line3(shifted(-0.24), b)];
        assert!(!loops_orphan_boundary_edges(&[near], &boundary, tol));
        // 0.26 · 2 = 0.52 rounds to cell 1: a different key, so the boundary
        // edge is orphaned.
        let far = vec![line3(shifted(0.26), b)];
        assert!(
            loops_orphan_boundary_edges(&[far], &boundary, tol),
            "axis {axis}: 0.26 must change the cell"
        );
        // Far along the axis, the same distinction holds at 2.24 vs 2.26.
        let near = vec![line3(shifted(2.0 + 0.24), b)];
        let bnd = vec![line3(shifted(2.0), b)];
        assert!(!loops_orphan_boundary_edges(&[near], &bnd, tol));
        let far = vec![line3(shifted(2.0 + 0.26), b)];
        assert!(loops_orphan_boundary_edges(&[far], &bnd, tol));
    }
}

/// Arcs shorter than `tol / radius` in angle are skipped (their region is
/// nil); longer ones are keyed. Radius 4 separates `span · r` from `span / r`.
#[test]
fn tiny_arcs_are_skipped_only_below_the_span_times_radius_floor() {
    let tol = 0.5;
    let c = z_circle(pt(0.0, 0.0, 0.0), 4.0);
    // span · r = 0.25 ≤ 0.5: skipped, so an uncovered one is not orphaned.
    let skipped = vec![arc3(&c, 1.0, 1.0 + 0.0625)];
    assert!(!loops_orphan_boundary_edges(&[], &skipped, tol));
    // span · r = 1.0 > 0.5: keyed, so an uncovered one is orphaned.
    let kept = vec![arc3(&c, 1.0, 1.25)];
    assert!(loops_orphan_boundary_edges(&[], &kept, tol));
    assert!(!loops_orphan_boundary_edges(
        std::slice::from_ref(&kept),
        &kept,
        tol
    ));
    // A full rim (coincident ends) is a keyed arc, not a zero-extent edge.
    let rim = vec![arc3(&c, 0.0, TAU)];
    assert!(loops_orphan_boundary_edges(&[], &rim, tol));
    assert!(!loops_orphan_boundary_edges(
        std::slice::from_ref(&rim),
        &rim,
        tol
    ));
}

/// Two arcs of one rim sharing both endpoints are told apart by the 3D
/// midpoint of their native span. The split angle `a` solves
/// `0.5/a = a + 0.5/(2π − a)`, the point where the two arcs' midpoints
/// would coincide if the mid angle were `start + 0.5/span` instead of
/// `start + span/2`; the true midpoints stay a half turn apart.
#[test]
fn co_endpoint_rim_halves_never_alias() {
    let tol = 0.5;
    let c = z_circle(pt(0.0, 0.0, 0.0), 4.0);
    let (mut lo, mut hi) = (0.3_f64, 1.0_f64);
    let f = |a: f64| 0.5 / a - a - 0.5 / (TAU - a);
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        if f(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let a = 0.5 * (lo + hi);
    assert!(f(a).abs() < 1e-12);
    let first = arc3(&c, 0.0, a);
    let second = arc3(&c, a, TAU);
    let boundary = vec![first.clone(), second.clone()];
    // Only the first half in the loops: the second is orphaned.
    assert!(loops_orphan_boundary_edges(
        &[vec![first.clone()]],
        &boundary,
        tol
    ));
    assert!(loops_orphan_boundary_edges(
        &[vec![second.clone()]],
        &boundary,
        tol
    ));
    assert!(!loops_orphan_boundary_edges(
        &[vec![first, second]],
        &boundary,
        tol
    ));
    // The classic halves, and a quarter plus its complement.
    for split in [PI, FRAC_PI_2_F64] {
        let first = arc3(&c, 0.0, split);
        let second = arc3(&c, split, TAU);
        let boundary = vec![first.clone(), second.clone()];
        assert!(loops_orphan_boundary_edges(&[vec![first]], &boundary, tol));
        assert!(!loops_orphan_boundary_edges(
            std::slice::from_ref(&boundary),
            &boundary,
            tol
        ));
    }
}
const FRAC_PI_2_F64: f64 = std::f64::consts::FRAC_PI_2;

// ── Cylinder lateral fixtures for the rim-chain splitter ────────────────

/// A primitive-style cylinder lateral of radius `r` between `z0` and `z1`,
/// axis +z through the origin, seam up the +x meridian. `bottom_shift`
/// moves the bottom rim circle (and its seam vertex) by that much along +x,
/// so its start sits that far off the seam meridian.
fn lateral(r: f64, z0: f64, z1: f64, bottom_shift: f64) -> (Topology, FaceId) {
    const TOL: f64 = 1e-7;
    let mut topo = Topology::new();
    let z = Vec3::new(0.0, 0.0, 1.0);
    let surface = CylindricalSurface::new(pt(0.0, 0.0, 0.0), z, r).unwrap();
    let bot_start = pt(r + bottom_shift, 0.0, z0);
    let top_start = pt(r, 0.0, z1);
    let v_bot = topo.add_vertex(Vertex::new(bot_start, TOL));
    let v_top = topo.add_vertex(Vertex::new(top_start, TOL));
    let bot = Circle3D::new(pt(bottom_shift, 0.0, z0), z, r).unwrap();
    let top = Circle3D::new(pt(0.0, 0.0, z1), z, r).unwrap();
    let b0 = bot.project(bot_start);
    let t0 = top.project(top_start);
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
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(surface)));
    (topo, face)
}
fn on_cyl(r: f64, theta: f64, z: f64) -> Point3 {
    pt(r * theta.cos(), r * theta.sin(), z)
}
/// A marched (fitted cubic NURBS) section through `pts`, with its knot
/// vector shifted off zero as marched curves are.
fn marched(pts: &[Point3]) -> SectionEdge {
    let fit = remus_math::nurbs::fitting::interpolate(pts, 3).unwrap();
    let shifted = remus_math::nurbs::curve::NurbsCurve::new(
        fit.degree(),
        fit.knots().iter().map(|k| k + 2.5).collect(),
        fit.control_points().to_vec(),
        fit.weights().to_vec(),
    )
    .unwrap();
    SectionEdge {
        curve_3d: EdgeCurve::NurbsCurve(shifted),
        trim: None,
        pcurve_a: dummy_pcurve(),
        pcurve_b: dummy_pcurve(),
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
/// Thirteen points of `s ↦ (θ(s), z(s))` on the cylinder over `[s0, s1]`.
fn piece(r: f64, s0: f64, s1: f64, curve: impl Fn(f64) -> (f64, f64)) -> Vec<Point3> {
    (0..=12)
        .map(|k| {
            let s = (s1 - s0).mul_add(f64::from(k) / 12.0, s0);
            let (t, z) = curve(s);
            on_cyl(r, t, z)
        })
        .collect()
}
fn reversed(pts: &[Point3]) -> Vec<Point3> {
    pts.iter().rev().copied().collect()
}
/// Call the rim-chain splitter directly on the face's real boundary pcurves.
fn rim_chains(
    topo: &Topology,
    face: FaceId,
    sections: &[SectionEdge],
    tol: f64,
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
        tol,
    )
    .unwrap()
}
/// One wire edge re-sampled from its own carrier, in traversal order.
fn polyline(e: &OrientedPCurveEdge) -> Vec<Point3> {
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
                    pt(
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
            if (pts[0] - e.start_3d).length() > (pts[N as usize] - e.start_3d).length() {
                pts.reverse();
            }
            pts
        }
        EdgeCurve::Ellipse(_) | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
            panic!("unexpected wire curve {}", e.curve_3d.type_tag())
        }
    }
}
/// Shoelace area of a region's wire in `(r·θ, z)` with `θ` unwrapped along
/// the loop, and its interior point as `(θ ∈ [0, 2π), z)`.
fn measure(sf: &SplitSubFace, r: f64) -> Measured {
    let mut pts: Vec<(f64, f64)> = Vec::new();
    let mut prev: Option<f64> = None;
    for e in &sf.outer_wire {
        for p in polyline(e) {
            let raw = p.y().atan2(p.x());
            let theta = prev.map_or(raw, |t| t + (raw - t + PI).rem_euclid(TAU) - PI);
            prev = Some(theta);
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
    (0.5 * twice, (p.y().atan2(p.x()).rem_euclid(TAU), p.z()))
}
fn assert_close(got: f64, want: f64, abs: f64, what: &str) {
    assert!(
        (got - want).abs() <= abs,
        "{what}: got {got}, want {want} (|Δ| = {})",
        (got - want).abs()
    );
}
/// `(area, (θ, z) of the interior point)` of one region.
type Measured = (f64, (f64, f64));
/// The two regions ordered as (band, lens) by area.
fn band_and_lens(regions: &[SplitSubFace], r: f64) -> (Measured, Measured) {
    assert_eq!(regions.len(), 2, "want band + lens");
    let a = measure(&regions[0], r);
    let b = measure(&regions[1], r);
    if a.0 > b.0 { (a, b) } else { (b, a) }
}

// ── split_periodic_face_by_rim_chains: interior points (mod.rs 2160–2175) ─

/// A skewed notch `z = d·(1 − x²)(1 + x)`, `x = (θ − c)/α`, centred at
/// `c = 2` with half-width 1 on a radius-1, height-2 lateral: the lens area
/// is `r·α·d·∫(1 − x²)(1 + x)dx = (4/3)·α·d·r`, its interior sits on the
/// notch-centre meridian halfway up the chain there (`z = d/2`, since the
/// profile is `d` at `x = 0`), and the band's interior sits on the far rim's
/// midpoint meridian `θ = π`, which the chain (θ ∈ [1, 3]) does not cross,
/// halfway between the rims.
#[test]
fn skewed_notch_interiors_sit_halfway_to_the_chain_on_the_probe_meridians() {
    let (r, h, c, alpha, d) = (1.0, 2.0, 2.0, 1.0, 0.9);
    let (topo, face) = lateral(r, 0.0, h, 0.0);
    let profile = |s: f64| {
        let x = 2.0 * s - 1.0;
        (c + alpha * x, d * (1.0 - x * x) * (1.0 + x))
    };
    // Piece breaks off the chain's middle, so the centre meridian falls
    // between two samples of the middle piece.
    let sections = vec![
        marched(&piece(r, 0.0, 0.35, profile)),
        marched(&piece(r, 0.35, 0.8, profile)),
        marched(&piece(r, 0.8, 1.0, profile)),
    ];
    let regions = rim_chains(&topo, face, &sections, 1e-7).expect("in-cell notch");
    let ((band_area, band_int), (lens_area, lens_int)) = band_and_lens(&regions, r);
    let lens = 4.0 / 3.0 * alpha * d * r;
    assert_close(lens_area, lens, 3e-3, "lens area");
    assert_close(band_area, TAU * r * h - lens, 3e-3, "band area");
    assert_close(lens_int.0, c, 1e-9, "lens interior θ");
    assert_close(lens_int.1, d / 2.0, 1e-3, "lens interior z");
    assert_close(band_int.0, PI, 1e-9, "band interior θ");
    assert_close(band_int.1, h / 2.0, 1e-9, "band interior z");
}

/// Two rim-to-rim chains on a radius-1, height-2 lateral: `a` bows toward
/// the seam-side probe meridian without crossing it, `θ = 0.6 + 0.4(z−1)²`
/// (ends at θ = 1 on both rims), and `b` is the straight helix
/// `θ = 3.5 + 0.15·z`, which crosses the antipode of that meridian. Neither
/// crosses the seam sector's probe meridian `θ = 0.5` (half of the rim
/// piece from the seam to `a`) nor the clear sector's `θ = 2.25`, so both
/// interiors sit at mid-height. The clear sector's area is
/// `r·∫₀²(θ_b − θ_a)dz = 5.8 + 0.3 − 0.4·(2/3)`.
#[test]
fn sector_interiors_ignore_chains_beside_and_opposite_the_probe_meridian() {
    let (r, h) = (1.0, 2.0);
    let (topo, face) = lateral(r, 0.0, h, 0.0);
    let a = piece(r, 0.0, 1.0, |s| {
        let z = h * s;
        (0.4f64.mul_add((z - 1.0) * (z - 1.0), 0.6), z)
    });
    let b = piece(r, 0.0, 1.0, |s| {
        let z = h * s;
        (0.15f64.mul_add(z, 3.5), z)
    });
    for sections in [
        vec![marched(&a), marched(&b)],
        vec![marched(&reversed(&b)), marched(&reversed(&a))],
    ] {
        let regions = rim_chains(&topo, face, &sections, 1e-7).expect("sector pair");
        assert_eq!(regions.len(), 2);
        let clear_area = 5.8 + 0.3 - 0.4 * 2.0 / 3.0;
        let mut found_clear = false;
        for sf in &regions {
            let (area, (theta, z)) = measure(sf, r);
            assert_close(z, h / 2.0, 1e-9, "sector interior height");
            if (theta - 2.25).abs() < 1e-9 {
                found_clear = true;
                assert_close(area, clear_area, 3e-3, "clear sector area");
            } else {
                assert_close(theta, 0.5, 1e-9, "seam sector interior θ");
                assert_close(area, TAU * r * h - clear_area, 3e-3, "seam sector area");
            }
        }
        assert!(found_clear);
    }
}

// ── split_periodic_face_by_rim_chains: weld-scale gates ─────────────────

/// A symmetric notch `z = d·sin(π s)` over `θ ∈ [c − α, c + α]`, cut into a
/// piece up to `s = 0.4`, a vertical piece of length `len` at that meridian,
/// and a piece back to the rim that fades the lift out.
fn notch_with_vertical_piece(r: f64, len: f64) -> Vec<SectionEdge> {
    let (c, alpha, d): (f64, f64, f64) = (PI, 1.0, 2.0);
    let theta = |s: f64| (2.0 * alpha).mul_add(s, c - alpha);
    let z = |s: f64| d * (PI * s).sin();
    let up = piece(r, 0.0, 0.4, |s| (theta(s), z(s)));
    let mid: Vec<Point3> = (0..=12)
        .map(|k| on_cyl(r, theta(0.4), z(0.4) + len * f64::from(k) / 12.0))
        .collect();
    let down = piece(r, 0.4, 1.0, |s| {
        (theta(s), z(s) + len * (1.0 - (s - 0.4) / 0.6))
    });
    vec![marched(&up), marched(&mid), marched(&down)]
}

/// A section piece is refused as degenerate only when its chord is strictly
/// shorter than the weld (`100 · tol`); one exactly a weld long is a piece.
/// `tol = 2⁻¹⁰` makes the weld and the vertical chord exact.
#[test]
fn section_piece_exactly_one_weld_long_is_kept() {
    let tol = 2.0_f64.powi(-10);
    let weld = 100.0 * tol;
    let (r, h) = (1.0, 3.0);
    let (topo, face) = lateral(r, 0.0, h, 0.0);
    let exact = notch_with_vertical_piece(r, weld);
    assert_eq!((exact[1].start - exact[1].end).length(), weld);
    let regions = rim_chains(&topo, face, &exact, tol).expect("weld-long piece is kept");
    assert_eq!(regions.len(), 2);
    let ((band, _), (lens, _)) = band_and_lens(&regions, r);
    // Lens ≈ r·∫ z dθ = 2α·d·(2/π)·r plus the lifted tail's extra
    // `len · (2α · 0.6) / 2`.
    let lens_want = 2.0 * 2.0 * 2.0 / PI + weld * 1.2 / 2.0;
    assert_close(lens, lens_want, 2e-2, "lens area");
    assert_close(band, TAU * r * h - lens_want, 2e-2, "band area");
    assert!(rim_chains(&topo, face, &notch_with_vertical_piece(r, 2.0 * weld), tol).is_some());
    assert!(rim_chains(&topo, face, &notch_with_vertical_piece(r, 0.5 * weld), tol).is_none());
}

/// Two pieces of a notch meeting at `s = 0.5` with a vertical gap `gap`
/// between the first piece's end and the second's start. Returned in the
/// four attachment orders the chainer distinguishes.
fn gapped_notch(r: f64, gap: f64) -> [Vec<SectionEdge>; 4] {
    let (c, alpha, d): (f64, f64, f64) = (PI, 1.0, 1.5);
    let theta = |s: f64| (2.0 * alpha).mul_add(s, c - alpha);
    let z = |s: f64| d * (PI * s).sin();
    let first = piece(r, 0.0, 0.5, |s| (theta(s), z(s)));
    let second = piece(r, 0.5, 1.0, |s| {
        (theta(s), z(s) + gap * (1.0 - (s - 0.5) / 0.5))
    });
    [
        vec![marched(&first), marched(&second)],
        vec![marched(&first), marched(&reversed(&second))],
        vec![marched(&second), marched(&first)],
        vec![marched(&reversed(&second)), marched(&first)],
    ]
}

/// Pieces chain when their ends are strictly closer than the weld; a gap of
/// exactly one weld leaves two dangling chains, which decline.
#[test]
fn junction_gap_exactly_one_weld_is_not_chained() {
    let tol = 2.0_f64.powi(-20);
    let weld = 100.0 * tol;
    let (r, h) = (1.0, 3.0);
    let (topo, face) = lateral(r, 0.0, h, 0.0);
    for (order, sections) in gapped_notch(r, weld).into_iter().enumerate() {
        assert!(
            rim_chains(&topo, face, &sections, tol).is_none(),
            "order {order}: a weld-wide gap must not chain"
        );
    }
    for (order, sections) in gapped_notch(r, 0.5 * weld).into_iter().enumerate() {
        let regions = rim_chains(&topo, face, &sections, tol)
            .unwrap_or_else(|| panic!("order {order}: a half-weld gap chains"));
        let ((_, _), (lens, _)) = band_and_lens(&regions, r);
        assert_close(lens, 2.0 * 1.5 * 2.0 / PI, 2e-2, "lens area");
    }
}

/// The notched rim must start on the seam meridian within the weld: a rim
/// whose start sits exactly one weld off it is refused, half a weld is
/// accepted. The rim circle is shifted along +x with its seam vertex, so
/// the offset to the surface's own seam point is exactly the shift.
#[test]
fn rim_start_exactly_one_weld_off_the_seam_meridian_is_refused() {
    let tol = 2.0_f64.powi(-20);
    let weld = 100.0 * tol;
    let (r, h, c, alpha, d) = (1.0, 2.0, PI, 1.0, 0.9);
    let notch = |shift: f64| {
        let mut pts = piece(r, 0.0, 1.0, |s| {
            let x = 2.0 * s - 1.0;
            (c + alpha * x, d * (1.0 - x * x))
        });
        let n = pts.len();
        pts[0] = pts[0] + Vec3::new(shift, 0.0, 0.0);
        pts[n - 1] = pts[n - 1] + Vec3::new(shift, 0.0, 0.0);
        vec![marched(&pts)]
    };
    for (shift, accepted) in [
        (0.0, true),
        (0.5 * weld, true),
        (weld, false),
        (2.0 * weld, false),
    ] {
        let (topo, face) = lateral(r, 0.0, h, shift);
        let result = rim_chains(&topo, face, &notch(shift), tol);
        assert_eq!(result.is_some(), accepted, "shift {shift}");
        if let Some(regions) = result {
            let ((_, _), (lens, _)) = band_and_lens(&regions, r);
            // ∫ d(1 − x²) dx · α · r = (4/3)·α·d·r.
            assert_close(lens, 4.0 / 3.0 * alpha * d * r, 2e-2, "lens area");
        }
    }
}

/// A notch must keep both ends clear of the seam meridian by more than the
/// weld: an end half a weld before the seam (or half a weld past it) is
/// refused, two welds away is accepted.
#[test]
fn chain_ends_half_a_weld_from_the_seam_are_refused() {
    let tol = 1e-7;
    let weld = 100.0 * tol;
    let (r, h, alpha, d) = (1.0, 2.0, 0.8, 0.9);
    let (topo, face) = lateral(r, 0.0, h, 0.0);
    let notch = |t0: f64, t1: f64| {
        vec![marched(&piece(r, 0.0, 1.0, |s| {
            let x = 2.0 * s - 1.0;
            ((t1 - t0).mul_add(s, t0), d * (1.0 - x * x))
        }))]
    };
    // Far end just before the seam.
    assert!(
        rim_chains(
            &topo,
            face,
            &notch(TAU - 0.5 * weld - 2.0 * alpha, TAU - 0.5 * weld),
            tol
        )
        .is_none()
    );
    assert!(
        rim_chains(
            &topo,
            face,
            &notch(TAU - 2.0 * weld - 2.0 * alpha, TAU - 2.0 * weld),
            tol
        )
        .is_some()
    );
    // Near end just past the seam.
    assert!(
        rim_chains(
            &topo,
            face,
            &notch(0.5 * weld, 0.5 * weld + 2.0 * alpha),
            tol
        )
        .is_none()
    );
    assert!(
        rim_chains(
            &topo,
            face,
            &notch(2.0 * weld, 2.0 * weld + 2.0 * alpha),
            tol
        )
        .is_some()
    );
    // Straddling the seam is refused outright.
    assert!(rim_chains(&topo, face, &notch(-alpha, alpha), tol).is_none());
}

// ── split_face_2d dispatch on a cone lateral (mod.rs 7880–7960) ─────────

/// A frustum lateral with half-angle π/4: radius grows one unit per unit of
/// height. Bottom rim radius `r0` at `z = 0`, top rim radius `r0 + h`.
fn cone_lateral(r0: f64, h: f64) -> (Topology, FaceId, FaceSurface) {
    const TOL: f64 = 1e-7;
    let mut topo = Topology::new();
    let z = Vec3::new(0.0, 0.0, 1.0);
    let surface = FaceSurface::Cone(
        ConicalSurface::new(pt(0.0, 0.0, -r0), z, std::f64::consts::FRAC_PI_4).unwrap(),
    );
    let r1 = r0 + h;
    let bot_start = pt(r0, 0.0, 0.0);
    let top_start = pt(r1, 0.0, h);
    let v_bot = topo.add_vertex(Vertex::new(bot_start, TOL));
    let v_top = topo.add_vertex(Vertex::new(top_start, TOL));
    let bot = Circle3D::new(pt(0.0, 0.0, 0.0), z, r0).unwrap();
    let top = Circle3D::new(pt(0.0, 0.0, h), z, r1).unwrap();
    let b0 = bot.project(bot_start);
    let t0 = top.project(top_start);
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
    let face = topo.add_face(Face::new(wire, vec![], surface.clone()));
    (topo, face, surface)
}
/// A straight ruling section of the frustum at angle `theta`, bottom rim to
/// top rim, with its exact `(u, v)` pcurve.
fn ruling(surface: &FaceSurface, r0: f64, h: f64, theta: f64) -> SectionEdge {
    let start = pt(r0 * theta.cos(), r0 * theta.sin(), 0.0);
    let end = pt((r0 + h) * theta.cos(), (r0 + h) * theta.sin(), h);
    let (u0, v0) = surface.project_point(start).unwrap();
    let (_, v1) = surface.project_point(end).unwrap();
    let pcurve = remus_math::curves2d::Curve2D::Line(
        remus_math::curves2d::Line2D::new(
            Point2::new(u0, v0),
            remus_math::vec::Vec2::new(0.0, (v1 - v0).signum()),
        )
        .unwrap(),
    );
    SectionEdge {
        curve_3d: EdgeCurve::Line,
        trim: None,
        pcurve_a: pcurve.clone(),
        pcurve_b: pcurve,
        start,
        end,
        start_uv_a: None,
        end_uv_a: None,
        start_uv_b: None,
        end_uv_b: None,
        target_face: None,
        pave_block_id: None,
    }
}
/// Shoelace area of a region's wire in `(θ, z)` with `θ` unwrapped along
/// the loop (positive for a loop keeping the region on its left), plus the
/// loop's net turn in `θ`.
fn theta_z_area(sf: &SplitSubFace) -> (f64, f64) {
    let mut pts: Vec<(f64, f64)> = Vec::new();
    let mut prev: Option<f64> = None;
    for e in &sf.outer_wire {
        for p in polyline(e) {
            let raw = p.y().atan2(p.x());
            let theta = prev.map_or(raw, |t| t + (raw - t + PI).rem_euclid(TAU) - PI);
            prev = Some(theta);
            pts.push((theta, p.z()));
        }
    }
    let n = pts.len();
    let mut twice = 0.0;
    for i in 0..n {
        let (x0, y0) = pts[i];
        let (x1, y1) = pts[(i + 1) % n];
        twice += x0.mul_add(y1, -(x1 * y0));
    }
    (0.5 * twice, pts[n - 1].0 - pts[0].0)
}

/// Two straight rulings at θ = 1 and θ = 3 cut a frustum lateral into the
/// sector between them and the seam-side remainder: in the developed `(θ, z)`
/// chart the pieces are `2·h` and `(2π − 2)·h`, and neither winds. This is
/// the cone route through the public dispatcher (the rim-chain shortcut
/// declines straight sections), so the cone-only rescue guards below the
/// greedy trace are exercised with a healthy, orphan-free partition.
#[test]
fn frustum_cut_by_two_rulings_is_two_sectors_through_the_dispatcher() {
    let (r0, h) = (1.0, 2.0);
    let (topo, face, surface) = cone_lateral(r0, h);
    let sections = vec![ruling(&surface, r0, h, 1.0), ruling(&surface, r0, h, 3.0)];
    let regions = split_face_2d(
        &topo,
        face,
        &sections,
        Rank::A,
        &remus_math::tolerance::Tolerance::default(),
        None,
        Some(&SurfaceInfo::Parametric {
            u_periodic: true,
            v_periodic: false,
        }),
        &remus_math::det_hash::DetHashMap::default(),
        None,
    )
    .unwrap();
    assert_eq!(regions.len(), 2, "want two sectors");
    let mut areas: Vec<f64> = regions
        .iter()
        .map(|sf| {
            let (area, turn) = theta_z_area(sf);
            assert!(turn.abs() < 1e-9, "a sector must not wind: {turn}");
            assert!(sf.inner_wires.is_empty());
            area.abs()
        })
        .collect();
    areas.sort_by(f64::total_cmp);
    assert_close(areas[0], 2.0 * h, 1e-6, "sector between the rulings");
    assert_close(areas[1], (TAU - 2.0) * h, 1e-6, "seam-side sector");
    // Every ruling bounds both sectors once.
    for s in &sections {
        let uses = regions
            .iter()
            .flat_map(|sf| &sf.outer_wire)
            .filter(|e| {
                matches!(e.curve_3d, EdgeCurve::Line)
                    && ((e.start_3d - s.start).length() < 1e-9
                        && (e.end_3d - s.end).length() < 1e-9
                        || (e.start_3d - s.end).length() < 1e-9
                            && (e.end_3d - s.start).length() < 1e-9)
            })
            .count();
        assert_eq!(uses, 2, "each ruling is shared by both sectors");
    }
}
