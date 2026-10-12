//! Differential oracle for the phase-FF junction snap's box gate.
//!
//! `snap_to_boundary_junction_band` skips the sampled search on every curved
//! boundary edge whose box lies farther from the query point than the band:
//! no point the search evaluates can be adopted there. The function below is
//! the search as it was before the gate, kept verbatim, and the tests compare
//! the two bit for bit over a plate holding circular, elliptical and NURBS
//! holes, and check the box bound against sampled curve distances. The
//! per-face boundary cache phase FF's registry keeps (`snap_with_boundaries`)
//! is compared the same way, one cache serving every query. A last test
//! checks the untouched-face box gate: a face keeps its conservative box
//! only where its sampled box is rejected against every partner too.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unreachable
)]

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_topology::face::Face;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use super::{
    Edge, EdgeCurve, FaceId, FaceSurface, Point3, Tolerance, Topology, Vec3, Vertex,
    compute_face_bbox, compute_face_bboxes_against, conservative_face_bbox, curve_box_distance,
    snap_to_boundary_junction_band, snap_with_boundaries,
};

/// `snap_to_boundary_junction_band` before the box gate, verbatim.
#[allow(clippy::too_many_lines)]
fn snap_before_box_gate(
    topo: &Topology,
    fa: FaceId,
    fb: FaceId,
    p: Point3,
    tol: Tolerance,
    weld: f64,
) -> Option<Point3> {
    const NS: usize = 64;
    let surf_of = |fid: FaceId| topo.face(fid).ok().map(|f| f.surface().clone());
    // (distance, foot, foot's curve param, owning face, edge)
    let mut best: Option<(f64, Point3, f64, FaceId, remus_topology::edge::EdgeId)> = None;
    for fid in [fa, fb] {
        let Ok(face) = topo.face(fid) else { continue };
        // Inner (hole) wires are legitimate exit boundaries too: a corner
        // crossing leaving through a bore rim must share its exact vertex.
        let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();
        for oe in wire_ids
            .iter()
            .filter_map(|&wid| topo.wire(wid).ok())
            .flat_map(|w| w.edges().to_vec())
        {
            let Ok(edge) = topo.edge(oe.edge()) else {
                continue;
            };
            let (Ok(sv), Ok(ev)) = (topo.vertex(edge.start()), topo.vertex(edge.end())) else {
                continue;
            };
            let (sp, ep) = (sv.point(), ev.point());
            let (d0, d1) = crate::pave_filler::helpers::authoritative_edge_domain(
                edge,
                oe.edge(),
                "boundary-junction search",
            )
            .ok()?;
            if matches!(edge.curve(), EdgeCurve::Line) {
                // A fixed ternary-search count leaves a model-relative error
                // even when the exact section already meets a straight edge.
                let direction = ep - sp;
                let length_squared = direction.length_squared();
                if length_squared > 0.0 {
                    let parameter =
                        ((p - sp).dot(direction) / length_squared).clamp(d0.min(d1), d0.max(d1));
                    let foot = sp + direction * parameter;
                    let distance = (foot - p).length();
                    if distance < weld && best.as_ref().is_none_or(|b| distance < b.0) {
                        best = Some((distance, foot, parameter, fid, oe.edge()));
                    }
                }
                continue;
            }
            let mut best_k = 0;
            let mut best_d = f64::MAX;
            for k in 0..=NS {
                #[allow(clippy::cast_precision_loss)]
                let t = d0 + (d1 - d0) * (k as f64) / (NS as f64);
                let d = (edge.curve().evaluate_with_endpoints(t, sp, ep) - p).length();
                if d < best_d {
                    best_d = d;
                    best_k = k;
                }
            }
            #[allow(clippy::cast_precision_loss)]
            let mut lo = d0 + (d1 - d0) * (best_k.saturating_sub(1) as f64) / (NS as f64);
            #[allow(clippy::cast_precision_loss)]
            let mut hi = d0 + (d1 - d0) * ((best_k + 1).min(NS) as f64) / (NS as f64);
            for _ in 0..48 {
                let m1 = lo + (hi - lo) / 3.0;
                let m2 = hi - (hi - lo) / 3.0;
                let f1 = (edge.curve().evaluate_with_endpoints(m1, sp, ep) - p).length();
                let f2 = (edge.curve().evaluate_with_endpoints(m2, sp, ep) - p).length();
                if f1 < f2 {
                    hi = m2;
                } else {
                    lo = m1;
                }
            }
            let tm = 0.5 * (lo + hi);
            let foot = edge.curve().evaluate_with_endpoints(tm, sp, ep);
            let d = (foot - p).length();
            if d < weld && best.as_ref().is_none_or(|b| d < b.0) {
                best = Some((d, foot, tm, fid, oe.edge()));
            }
        }
    }
    let (_, foot, tm, owner_fid, eid) = best?;
    let other = if owner_fid == fa { fb } else { fa };
    let other_surf = surf_of(other)?;
    let dist_to_surf = |q: Point3| -> f64 {
        match &other_surf {
            FaceSurface::Plane { normal, d } => {
                let length = normal.length();
                if length.is_finite() && length > 0.0 {
                    (normal.dot(q - Point3::new(0.0, 0.0, 0.0)) - d).abs() / length
                } else {
                    f64::MAX
                }
            }
            FaceSurface::Nurbs(_)
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => other_surf
                .project_point(q)
                .and_then(|(u, v)| other_surf.evaluate(u, v))
                .map_or(f64::MAX, |s| (s - q).length()),
        }
    };
    // A foot landing weld-close to the boundary edge's own endpoint can
    // adopt that vertex only if it also lies on the partner surface. The
    // search band accommodates fit error; it must not consume a distinct
    // thin band between the section and a nearby cap. The
    // partner-surface refinement below is degenerate when the boundary
    // curve lies IN the partner surface (coincident faces): the distance
    // objective is flat, the ternary search converges on noise, and every
    // pair mints a different copy of the same corner. The operand vertex is
    // the one position all pairs share.
    if let Ok(e) = topo.edge(eid)
        && let (Ok(sv), Ok(ev)) = (topo.vertex(e.start()), topo.vertex(e.end()))
    {
        let (sp, ep) = (sv.point(), ev.point());
        let (dmin, vp) = if (sp - foot).length() <= (ep - foot).length() {
            ((sp - foot).length(), sp)
        } else {
            ((ep - foot).length(), ep)
        };
        if dmin <= weld && dist_to_surf(vp) <= tol.linear {
            return Some(vp);
        }
    }
    // The foot lies ON the boundary curve but is displaced along it by the
    // section's fit error. The true junction is where that boundary curve
    // meets the PARTNER face's surface — refine to it so every section
    // ending here (this one, and the partner-pair sections computed
    // independently) mints the SAME vertex.
    let edge = topo.edge(eid).ok()?;
    let (Ok(sv), Ok(ev)) = (topo.vertex(edge.start()), topo.vertex(edge.end())) else {
        return None;
    };
    let (sp, ep) = (sv.point(), ev.point());
    let (d0, d1) = crate::pave_filler::helpers::authoritative_edge_domain(
        edge,
        eid,
        "boundary-junction refinement",
    )
    .ok()?;
    let half = (d1 - d0).abs().max(1e-9) * 0.01;
    // Clamp the refine window to the edge's own span so the search never
    // evaluates (and snaps to) a point past the boundary arc's ends.
    let (span_lo, span_hi) = (d0.min(d1), d0.max(d1));
    let (mut lo, mut hi) = ((tm - half).max(span_lo), (tm + half).min(span_hi));
    for _ in 0..64 {
        let m1 = lo + (hi - lo) / 3.0;
        let m2 = hi - (hi - lo) / 3.0;
        let f1 = dist_to_surf(edge.curve().evaluate_with_endpoints(m1, sp, ep));
        let f2 = dist_to_surf(edge.curve().evaluate_with_endpoints(m2, sp, ep));
        if f1 < f2 {
            hi = m2;
        } else {
            lo = m1;
        }
    }
    let tj = 0.5 * (lo + hi);
    let junction = edge.curve().evaluate_with_endpoints(tj, sp, ep);
    if dist_to_surf(junction) <= tol.linear * 10.0 && (junction - foot).length() <= weld {
        Some(junction)
    } else if dist_to_surf(foot) <= tol.linear * 10.0 {
        Some(foot)
    } else {
        None
    }
}

// ── Fixtures ────────────────────────────────────────────────────────────

fn vertex(topo: &mut Topology, p: Point3) -> remus_topology::vertex::VertexId {
    topo.add_vertex(Vertex::new(p, 1e-7))
}

fn closed_edge(topo: &mut Topology, curve: EdgeCurve, trim: Option<(f64, f64)>) -> WireId {
    let start = match &curve {
        EdgeCurve::Circle(c) => c.evaluate(trim.map_or(0.0, |t| t.0)),
        EdgeCurve::Ellipse(e) => e.evaluate(trim.map_or(0.0, |t| t.0)),
        EdgeCurve::NurbsCurve(n) => n.evaluate(n.domain().0),
        EdgeCurve::Line | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
            unreachable!("closed fixture edges are curved")
        }
    };
    let v = vertex(topo, start);
    let mut edge = Edge::with_tolerance(v, v, curve, Some(1e-7));
    edge.set_trim(trim);
    let edge = topo.add_edge(edge);
    topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, false)], true).unwrap())
}

fn polygon_wire(topo: &mut Topology, corners: &[Point3]) -> WireId {
    let vertices: Vec<_> = corners.iter().map(|&p| vertex(topo, p)).collect();
    let edges = (0..vertices.len())
        .map(|i| {
            let edge = Edge::new(
                vertices[i],
                vertices[(i + 1) % vertices.len()],
                EdgeCurve::Line,
            );
            OrientedEdge::new(topo.add_edge(edge), true)
        })
        .collect();
    topo.add_wire(Wire::new(edges, true).unwrap())
}

/// A rational quadratic full circle (nine control points), so the NURBS arm
/// of the box gate is exercised by a real closed rim.
fn nurbs_circle(center: Point3, radius: f64) -> NurbsCurve {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let offsets = [
        (1.0, 0.0),
        (1.0, 1.0),
        (0.0, 1.0),
        (-1.0, 1.0),
        (-1.0, 0.0),
        (-1.0, -1.0),
        (0.0, -1.0),
        (1.0, -1.0),
        (1.0, 0.0),
    ];
    let points = offsets
        .iter()
        .map(|&(x, y)| Point3::new(center.x() + radius * x, center.y() + radius * y, center.z()))
        .collect();
    let weights = vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
    let knots = vec![
        0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
    ];
    NurbsCurve::new(2, knots, points, weights).unwrap()
}

/// A vertical wall face through the plate along `x = c` (`along_y`) or
/// `y = c`.
fn wall(topo: &mut Topology, along_y: bool, c: f64) -> FaceId {
    let corner = |a: f64, z: f64| {
        if along_y {
            Point3::new(c, a, z)
        } else {
            Point3::new(a, c, z)
        }
    };
    let wire = polygon_wire(
        topo,
        &[
            corner(0.0, 0.0),
            corner(60.0, 0.0),
            corner(60.0, 20.0),
            corner(0.0, 20.0),
        ],
    );
    let normal = if along_y {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    topo.add_face(Face::new(
        wire,
        Vec::new(),
        FaceSurface::Plane { normal, d: c },
    ))
}

/// The top of a 60 × 60 plate at z = 10 holding a 4 × 4 grid of circular
/// holes, an elliptical hole and a NURBS hole, and three vertical walls
/// crossing it: `x = 31` through a column of circles, `y = 20` through a row
/// of circles and the ellipse, `y = 40` through the NURBS hole.
fn holed_plate(topo: &mut Topology) -> (FaceId, Vec<FaceId>) {
    let z = 10.0;
    let outer = polygon_wire(
        topo,
        &[
            Point3::new(0.0, 0.0, z),
            Point3::new(60.0, 0.0, z),
            Point3::new(60.0, 60.0, z),
            Point3::new(0.0, 60.0, z),
        ],
    );
    let up = Vec3::new(0.0, 0.0, 1.0);
    let mut holes = Vec::new();
    for row in 0..4 {
        for col in 0..4 {
            let center = Point3::new(7.5 + f64::from(col) * 12.0, 7.5 + f64::from(row) * 12.0, z);
            let circle = Circle3D::new(center, up, 2.0).unwrap();
            holes.push(closed_edge(
                topo,
                EdgeCurve::Circle(circle),
                Some((0.0, std::f64::consts::TAU)),
            ));
        }
    }
    let ellipse = Ellipse3D::new(Point3::new(54.0, 20.0, z), up, 3.0, 1.5).unwrap();
    holes.push(closed_edge(
        topo,
        EdgeCurve::Ellipse(ellipse),
        Some((0.0, std::f64::consts::TAU)),
    ));
    holes.push(closed_edge(
        topo,
        EdgeCurve::NurbsCurve(nurbs_circle(Point3::new(54.0, 40.0, z), 2.5)),
        Some((0.0, 1.0)),
    ));
    let top = topo.add_face(Face::new(
        outer,
        holes,
        FaceSurface::Plane { normal: up, d: z },
    ));
    let walls = vec![
        wall(topo, true, 31.0),
        wall(topo, false, 20.0),
        wall(topo, false, 40.0),
    ];
    (top, walls)
}

/// Query points: a coarse lattice over the plate, points just off every
/// rim and corner, points just above and below the plane, and dense lines
/// along the walls, where rims cross them at real junctions.
fn query_points(topo: &Topology, face: FaceId) -> Vec<Point3> {
    let mut points = Vec::new();
    for step in 0..=150 {
        let a = f64::from(step) * 0.4;
        for delta in [0.0, 2e-4, -5e-4] {
            points.push(Point3::new(31.0 + delta, a, 10.0));
            points.push(Point3::new(a, 20.0 + delta, 10.0));
            points.push(Point3::new(a, 40.0 - delta, 10.0 + 1e-5));
        }
    }
    for i in 0..=12 {
        for j in 0..=12 {
            let (x, y) = (f64::from(i) * 5.0, f64::from(j) * 5.0);
            points.push(Point3::new(x, y, 10.0));
            points.push(Point3::new(x + 3e-4, y - 2e-4, 10.0 + 1e-4));
        }
    }
    let face = topo.face(face).unwrap();
    for &wire in std::iter::once(&face.outer_wire()).chain(face.inner_wires()) {
        for oriented in topo.wire(wire).unwrap().edges() {
            let edge = topo.edge(oriented.edge()).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            let (t0, t1) = edge.strict_domain().unwrap();
            for k in 0..=6 {
                let t = t0 + (t1 - t0) * f64::from(k) / 6.0;
                let on = edge.curve().evaluate_with_endpoints(t, start, end);
                for offset in [
                    Vec3::new(0.0, 0.0, 0.0),
                    Vec3::new(2e-4, 1e-4, 0.0),
                    Vec3::new(-7e-4, 0.0, 5e-5),
                    Vec3::new(0.0, 4e-3, 0.0),
                    Vec3::new(3e-2, -2e-2, 0.0),
                ] {
                    points.push(on + offset);
                }
            }
        }
    }
    points
}

fn bits(p: Point3) -> [u64; 3] {
    [p.x().to_bits(), p.y().to_bits(), p.z().to_bits()]
}

#[test]
fn gated_junction_snap_matches_the_ungated_oracle_bit_for_bit() {
    let mut topo = Topology::new();
    let (top, walls) = holed_plate(&mut topo);
    let tol = Tolerance::default();
    let mut snapped = 0;
    let mut compared = 0;
    // One boundary cache for every query, as a phase-FF pass keeps it.
    let mut boundaries = remus_math::det_hash::DetHashMap::default();
    for point in query_points(&topo, top) {
        for weld in [1e-3, 2.5] {
            for (fa, fb) in walls.iter().flat_map(|&wall| [(top, wall), (wall, top)]) {
                let gated = snap_to_boundary_junction_band(&topo, fa, fb, point, tol, weld);
                let oracle = snap_before_box_gate(&topo, fa, fb, point, tol, weld);
                assert_eq!(
                    gated.map(bits),
                    oracle.map(bits),
                    "snap of {point:?} (weld {weld}) differs from the ungated search"
                );
                let cached = snap_with_boundaries(&mut boundaries, &topo, fa, fb, point, tol, weld);
                assert_eq!(
                    cached.map(bits),
                    oracle.map(bits),
                    "cached-boundary snap of {point:?} (weld {weld}) differs"
                );
                compared += 1;
                snapped += usize::from(gated.is_some());
            }
        }
    }
    // Both outcomes must be exercised, or the comparison proves little.
    assert!(compared > 20_000, "only {compared} comparisons");
    assert!(snapped > 500, "only {snapped} snaps");
}

#[test]
fn curve_box_distance_never_exceeds_a_sampled_distance() {
    let mut topo = Topology::new();
    let (top, _) = holed_plate(&mut topo);
    let tilted = Circle3D::new(
        Point3::new(-3.0, 4.0, 1.5),
        Vec3::new(0.3, -0.5, 0.8).normalize().unwrap(),
        7.25,
    )
    .unwrap();
    let mut curves: Vec<EdgeCurve> = vec![EdgeCurve::Circle(tilted)];
    let face = topo.face(top).unwrap();
    for &wire in face.inner_wires() {
        for oriented in topo.wire(wire).unwrap().edges() {
            curves.push(topo.edge(oriented.edge()).unwrap().curve().clone());
        }
    }
    let probes: Vec<Point3> = (0..400)
        .map(|k| {
            let a = f64::from(k) * 0.37;
            Point3::new(
                30.0 * a.sin(),
                25.0 * (1.3 * a).cos() + 20.0,
                10.0 + 6.0 * (0.7 * a).sin(),
            )
        })
        .collect();
    for curve in &curves {
        let (t0, t1) = match curve {
            EdgeCurve::NurbsCurve(n) => n.domain(),
            EdgeCurve::Line
            | EdgeCurve::Circle(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => (0.0, std::f64::consts::TAU),
        };
        for &p in &probes {
            let bound = curve_box_distance(curve, p).expect("curved fixtures are bounded");
            for k in 0..=720 {
                let t = t0 + (t1 - t0) * f64::from(k) / 720.0;
                let q = curve.evaluate_with_endpoints(t, p, p);
                let d = (q - p).length();
                assert!(
                    bound <= d,
                    "bound {bound} exceeds the distance {d} to {q:?} on {}",
                    curve.type_tag()
                );
            }
        }
    }
    assert!(curve_box_distance(&EdgeCurve::Line, Point3::new(0.0, 0.0, 0.0)).is_none());
}

#[test]
fn face_box_gate_only_replaces_boxes_the_rejection_refuses() {
    let mut topo = Topology::new();
    let (top, walls) = holed_plate(&mut topo);
    let tol = Tolerance::default();
    let mut faces = vec![top];
    faces.extend(&walls);
    let sampled: Vec<_> = faces
        .iter()
        .map(|&f| compute_face_bbox(&topo, f, tol).unwrap())
        .collect();
    for (&face, sampled) in faces.iter().zip(&sampled) {
        let conservative = conservative_face_bbox(&topo, face).unwrap().unwrap();
        assert!(
            conservative.contains_point(sampled.min) && conservative.contains_point(sampled.max),
            "conservative box {conservative:?} misses sampled {sampled:?}"
        );
    }
    // Partners near one wall, far from another, and spanning everything.
    let partner = |lo: Point3, hi: Point3| remus_math::aabb::Aabb3 { min: lo, max: hi };
    let partner_sets = [
        vec![partner(
            Point3::new(29.0, 10.0, 5.0),
            Point3::new(30.5, 12.0, 15.0),
        )],
        vec![partner(
            Point3::new(70.0, 70.0, 70.0),
            Point3::new(71.0, 71.0, 71.0),
        )],
        vec![
            partner(Point3::new(-1.0, -1.0, -1.0), Point3::new(61.0, 61.0, 21.0)),
            partner(
                Point3::new(10.0, 39.9, 9.0),
                Point3::new(11.0, 40.0 - 1e-6, 11.0),
            ),
        ],
    ];
    let mut kept = 0;
    for partners in &partner_sets {
        let gated = compute_face_bboxes_against(&topo, &faces, partners, tol).unwrap();
        for (i, (gated, sampled)) in gated.iter().zip(&sampled).enumerate() {
            if gated == sampled {
                continue;
            }
            kept += 1;
            for partner in partners {
                assert!(
                    !sampled
                        .expanded(tol.linear)
                        .intersects(partner.expanded(tol.linear)),
                    "face {i} kept its conservative box against a partner its sampled box meets"
                );
            }
        }
    }
    assert!(kept > 0, "the gate never applied");
}
