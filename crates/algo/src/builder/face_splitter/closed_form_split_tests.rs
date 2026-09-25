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
    let mut topo = Topology::new();
    let z = Vec3::new(0.0, 0.0, 1.0);
    let surface = CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), z, r).unwrap();
    let v_bot = topo.add_vertex(Vertex::new(Point3::new(r, 0.0, 0.0), TOL));
    let v_top = topo.add_vertex(Vertex::new(Point3::new(r, 0.0, h), TOL));
    let bot = Circle3D::new(Point3::new(0.0, 0.0, 0.0), z, r).unwrap();
    let top = Circle3D::new(Point3::new(0.0, 0.0, h), z, r).unwrap();
    let b0 = bot.project(Point3::new(r, 0.0, 0.0));
    let t0 = top.project(Point3::new(r, 0.0, h));
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

/// A marched (fitted cubic NURBS) section through `pts`.
fn marched(pts: &[Point3]) -> SectionEdge {
    let dummy = remus_math::curves2d::Curve2D::Line(
        remus_math::curves2d::Line2D::new(
            Point2::new(0.0, 0.0),
            remus_math::vec::Vec2::new(1.0, 0.0),
        )
        .unwrap(),
    );
    SectionEdge {
        curve_3d: EdgeCurve::NurbsCurve(remus_math::nurbs::fitting::interpolate(pts, 3).unwrap()),
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

/// A chain notching one rim of a cylinder lateral splits it into the lens
/// under the chain and the annular band that keeps both rims, seam and the
/// rest of the notched rim — whether the notch sits on the bottom or the top
/// rim, whatever the scale, the piece order and the piece directions, and
/// whether the face is reversed.
#[test]
fn rim_notch_splits_lateral_into_band_and_lens_of_closed_form_area() {
    let alpha = PI / 3.0;
    for (r, h) in [(1.0, 2.0), (25.0, 10.0), (0.2, 0.5)] {
        let depth = 0.45 * h;
        for on_bottom in [true, false] {
            for reversed in [false, true] {
                let (topo, face) = lateral(r, h, reversed);
                let z_of = |t: f64| {
                    let d = notch_depth(t, alpha, depth);
                    if on_bottom { d } else { h - d }
                };
                let pieces = chain_pieces(r, &[0.0, 0.3, 0.7, 1.0], |s| {
                    let t = (2.0 * alpha).mul_add(s, PI - alpha);
                    (t, z_of(t))
                });
                // Scrambled order, the first piece reversed.
                let sections = vec![
                    marched(&pieces[1]),
                    marched(&reversed_pts(&pieces[0])),
                    marched(&pieces[2]),
                ];
                let ctx = format!("r={r} bottom={on_bottom} reversed={reversed}");
                let regions = split(&topo, face, &sections);
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
                }
                // The lens interior lies under the chain, the band's above it
                // or outside the notch.
                let depth_at = |(_, z, _): (f64, f64, f64)| if on_bottom { z } else { h - z };
                let (lt, _, _) = m[lens].interior;
                let ld = depth_at(m[lens].interior);
                assert!(
                    (lt - PI).abs() < alpha && ld > 0.0 && ld < notch_depth(lt, alpha, depth),
                    "{ctx}: lens interior {:?} is not under the chain",
                    m[lens].interior
                );
                let (bt, bz, _) = m[band].interior;
                let bd = depth_at(m[band].interior);
                assert!(
                    bz > 0.0
                        && bz < h
                        && ((bt - PI).abs() >= alpha || bd > notch_depth(bt, alpha, depth)),
                    "{ctx}: band interior {:?} is inside the lens",
                    m[band].interior
                );
                assert_sections_shared(&regions, &sections, &ctx);
                // The band keeps the whole far rim and both seam uses.
                let far_z = if on_bottom { h } else { 0.0 };
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
        }
    }
}

/// Two chains, each from one rim to the other, cut the lateral into the
/// sector clear of the seam and the seam-side sector. The chains are helices
/// `θ = θ_i + 0.2·z/h`, so both sectors have a constant angular width and a
/// closed-form area.
#[test]
fn rim_to_rim_chains_split_lateral_into_two_sectors_of_closed_form_area() {
    let (ta, tb, lean): (f64, f64, f64) = (2.0 * PI / 3.0, 4.0 * PI / 3.0, 0.4);
    for (r, h) in [(1.0, 2.0), (25.0, 10.0), (0.2, 0.5)] {
        for reversed in [false, true] {
            for flip in [false, true] {
                let (topo, face) = lateral(r, h, reversed);
                let a = chain_pieces(r, &[0.0, 0.5, 1.0], |s| (lean.mul_add(s, ta), h * s));
                let b = chain_pieces(r, &[0.0, 0.4, 1.0], |s| (lean.mul_add(s, tb), h * s));
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
                let ctx = format!("r={r} reversed={reversed} flip={flip}");
                let regions = split(&topo, face, &sections);
                assert_eq!(regions.len(), 2, "{ctx}: want two sectors");
                let m: Vec<Measured> = regions.iter().map(|sf| measure(sf, r)).collect();
                let (seam_side, clear) = if m[0].area > m[1].area {
                    (0, 1)
                } else {
                    (1, 0)
                };
                let clear_area = r * (tb - ta) * h;
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
                    assert!(mk.interior.1 > 0.0 && mk.interior.1 < h, "{ctx}");
                    assert_eq!(regions[k].reversed, reversed, "{ctx}");
                }
                let between = |(t, z, _): (f64, f64, f64)| {
                    let lean_z = lean * z / h;
                    t > ta + lean_z && t < tb + lean_z
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
                // Only the seam-side sector touches the seam, and it carries
                // it both ways.
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

/// Samples of a circular wire edge along its MINOR arc (every arc in the
/// collar fixture spans less than π), independent of stored flags.
fn minor_arc(e: &OrientedPCurveEdge) -> Vec<Point3> {
    let EdgeCurve::Circle(c) = &e.curve_3d else {
        panic!(
            "collar wires are all circular, got {}",
            e.curve_3d.type_tag()
        );
    };
    let (n, ctr) = (c.normal(), c.center());
    let (s, t) = (e.start_3d - ctr, e.end_3d - ctr);
    let sweep = s.cross(t).dot(n).atan2(s.dot(t));
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
