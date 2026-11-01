//! Regression: a cylinder fused onto a box with its axis on the box's corner
//! edge, both based at z = 0, left a stray vertex on the merged bottom face.
//!
//! The cylinder cap and the box floor are coplanar, so the cap's closed rim is
//! cut where the box faces cross it. The face splitter then subdivides any rim
//! piece of half a turn or more. It used to halve such a piece at its own gap
//! midpoint, and the cap and the wall see different gaps — the wall rim also
//! carries the seam-antipodal split the periodic path always adds. The cap
//! got a vertex at 225° and the wall one at 270°; the result carried both,
//! leaving a degree-2 vertex at 225° between two arcs of the same circle,
//! shared by the same two faces. Valid topology, but STEP exports showed the
//! extra vertex and edge. Rim pieces are now cut at quarter-turn stations
//! measured from the rim's seam, which depend only on the rim, so both faces
//! land on the same points — the same 0/90/180/270° vertices the untouched
//! top rim already carries.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::collections::BTreeMap;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

use remus_math::compact_sort::CompactSort;
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, BooleanOptions, boolean_with_options};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{edge_to_face_map, solid_edges, solid_faces};
use remus_topology::solid::SolidId;

const BOX: (f64, f64, f64) = (30.0, 18.0, 24.0);
const RADIUS: f64 = 6.0;
const CYL_HEIGHT: f64 = 28.0;

/// Fuse the corner-origin box with a cylinder whose axis is at `(cx, cy)`,
/// with face unification on (the default).
fn fuse_at(cx: f64, cy: f64) -> (Topology, SolidId) {
    let mut topo = Topology::default();
    let b = make_box(&mut topo, BOX.0, BOX.1, BOX.2).unwrap();
    let c = make_cylinder(&mut topo, RADIUS, CYL_HEIGHT).unwrap();
    if cx != 0.0 || cy != 0.0 {
        transform_solid(&mut topo, c, &Mat4::translation(cx, cy, 0.0)).unwrap();
    }
    let opts = BooleanOptions::default();
    assert!(opts.unify_faces, "the defect is on the default unify path");
    let fused = boolean_with_options(&mut topo, BooleanOp::Fuse, b, c, opts).unwrap();
    (topo, fused)
}

/// Area of the axis-`(cx, cy)` disc inside the box footprint, by composite
/// Simpson over chord lengths — an oracle independent of the kernel.
fn footprint_overlap_area(cx: f64, cy: f64) -> f64 {
    const N: usize = 200_000;
    let (x0, x1) = ((cx - RADIUS).max(0.0), (cx + RADIUS).min(BOX.0));
    if x1 <= x0 {
        return 0.0;
    }
    #[allow(clippy::cast_precision_loss)]
    let h = (x1 - x0) / N as f64;
    let chord = |x: f64| {
        let half = (x - cx).mul_add(-(x - cx), RADIUS * RADIUS).max(0.0).sqrt();
        ((cy + half).min(BOX.1) - (cy - half).max(0.0)).max(0.0)
    };
    let mut sum = chord(x0) + chord(x1);
    for i in 1..N {
        #[allow(clippy::cast_precision_loss)]
        let x = h.mul_add(i as f64, x0);
        sum += chord(x) * if i % 2 == 1 { 4.0 } else { 2.0 };
    }
    sum * h / 3.0
}

fn assert_valid_with_volume(topo: &Topology, solid: SolidId, expected: f64, label: &str) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(
        report.is_valid(),
        "{label}: fused solid must validate: {:?}",
        report.issues
    );
    let volume = solid_volume(topo, solid, 0.01).unwrap();
    assert!(
        (volume - expected).abs() <= 1e-6 * expected,
        "{label}: volume {volume}, expected {expected}"
    );
}

/// Vertices of `solid` that sit between exactly two edges on the same
/// circle, both shared by the same pair of faces — vertices that only split
/// one arc into two. Returned as `(position, angle from +x about the circle
/// centre)`.
fn same_circle_degree_two_vertices(topo: &Topology, solid: SolidId) -> Vec<([f64; 3], f64)> {
    let edge_faces = edge_to_face_map(topo, solid).unwrap();
    let mut incident: BTreeMap<usize, Vec<_>> = BTreeMap::new();
    for e in solid_edges(topo, solid).unwrap() {
        let edge = topo.edge(e).unwrap();
        incident.entry(edge.start().index()).or_default().push(e);
        if edge.end() != edge.start() {
            incident.entry(edge.end().index()).or_default().push(e);
        }
    }
    let mut out = Vec::new();
    for edges in incident.values() {
        let [a, b] = edges.as_slice() else { continue };
        let (ea, eb) = (topo.edge(*a).unwrap(), topo.edge(*b).unwrap());
        let (EdgeCurve::Circle(ca), EdgeCurve::Circle(cb)) = (ea.curve(), eb.curve()) else {
            continue;
        };
        if (ca.center() - cb.center()).length() > 1e-9
            || (ca.radius() - cb.radius()).abs() > 1e-9
            || ca.normal().cross(cb.normal()).length() > 1e-9
        {
            continue;
        }
        let mut fa = edge_faces[&a.index()].to_vec();
        let mut fb = edge_faces[&b.index()].to_vec();
        fa.compact_sort_by_key(|f| f.index());
        fb.compact_sort_by_key(|f| f.index());
        if fa != fb {
            continue;
        }
        let shared = if ea.start() == eb.start() || ea.start() == eb.end() {
            ea.start()
        } else {
            ea.end()
        };
        let p = topo.vertex(shared).unwrap().point();
        let c = ca.center();
        let angle = (p.y() - c.y()).atan2(p.x() - c.x()).rem_euclid(TAU);
        out.push(([p.x(), p.y(), p.z()], angle));
    }
    out
}

/// True when `angle` is a quarter-turn station of a `make_cylinder` rim,
/// whose seam sits at angle 0 about the axis.
fn is_quarter_station(angle: f64) -> bool {
    let q = angle / FRAC_PI_2;
    (q - q.round()).abs() < 1e-9
}

/// Sorted angles (from +x about the axis) of every vertex on the cylinder's
/// circle at height `z`.
fn rim_vertex_angles(topo: &Topology, solid: SolidId, cx: f64, cy: f64, z: f64) -> Vec<f64> {
    let mut angles: Vec<f64> = Vec::new();
    for e in solid_edges(topo, solid).unwrap() {
        let edge = topo.edge(e).unwrap();
        let EdgeCurve::Circle(circle) = edge.curve() else {
            continue;
        };
        if (circle.center().z() - z).abs() > 1e-9 || (circle.radius() - RADIUS).abs() > 1e-9 {
            continue;
        }
        for v in [edge.start(), edge.end()] {
            let p = topo.vertex(v).unwrap().point();
            let a = (p.y() - cy).atan2(p.x() - cx).rem_euclid(TAU);
            if !angles.iter().any(|b| (a - b).abs() < 1e-9) {
                angles.push(a);
            }
        }
    }
    angles.compact_sort_by(f64::total_cmp);
    angles
}

#[test]
fn corner_axis_cylinder_fuse_leaves_no_stray_cap_rim_vertex() {
    let (topo, fused) = fuse_at(0.0, 0.0);

    let expected = (BOX.0 * BOX.1).mul_add(BOX.2, PI * RADIUS * RADIUS * CYL_HEIGHT)
        - PI * RADIUS * RADIUS * BOX.2 / 4.0;
    assert_valid_with_volume(&topo, fused, expected, "corner axis");

    // Unification merges the box floor and the protruding cap into one face.
    let bottoms = solid_faces(&topo, fused)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            let face = topo.face(f).unwrap();
            face.surface().is_planar()
                && topo
                    .wire(face.outer_wire())
                    .unwrap()
                    .edges()
                    .iter()
                    .all(|oe| {
                        let e = topo.edge(oe.edge()).unwrap();
                        [e.start(), e.end()]
                            .iter()
                            .all(|&v| topo.vertex(v).unwrap().point().z().abs() < 1e-9)
                    })
        })
        .count();
    assert_eq!(bottoms, 1, "one merged bottom face");

    // The bottom rim is cut exactly where the untouched top rim is: at the
    // seam and the quarter turns from it. The 90° vertex is also where the
    // box face x = 0 meets the rim. No 225° vertex.
    let quarters = [0.0, FRAC_PI_2, PI, 1.5 * PI];
    for z in [0.0, CYL_HEIGHT] {
        let angles = rim_vertex_angles(&topo, fused, 0.0, 0.0, z);
        assert_eq!(angles.len(), 4, "rim at z={z}: vertex angles {angles:?}");
        for (a, q) in angles.iter().zip(quarters) {
            assert!(
                (a - q).abs() < 1e-9,
                "rim at z={z}: vertex at {}°, expected {}°",
                a.to_degrees(),
                q.to_degrees()
            );
        }
    }

    // Any vertex that only splits one arc of a rim must be one of those
    // stations — never a midpoint picked from one face's view of the rim.
    let stray: Vec<_> = same_circle_degree_two_vertices(&topo, fused)
        .into_iter()
        .filter(|(_, a)| !is_quarter_station(*a))
        .collect();
    assert!(
        stray.is_empty(),
        "degree-2 vertices between two arcs of one circle, off the seam's quarter \
         stations: {stray:?}"
    );
}

/// The same coplanar-cap configuration along both long box edges: the axis on
/// the edge line y = 0 or y = 18, inside, at and outside the corners. Every
/// one of these placed a midpoint vertex on the cap rim before the fix.
#[test]
fn edge_axis_cylinder_fuses_leave_no_stray_cap_rim_vertex() {
    for cy in [0.0, BOX.1] {
        for cx in [-4.0, -2.0, 0.0, 2.0, 3.0, 5.0, 15.0] {
            let label = format!("axis at ({cx}, {cy})");
            let (topo, fused) = fuse_at(cx, cy);
            let expected = (BOX.0 * BOX.1).mul_add(BOX.2, PI * RADIUS * RADIUS * CYL_HEIGHT)
                - footprint_overlap_area(cx, cy) * BOX.2;
            assert_valid_with_volume(&topo, fused, expected, &label);
            let stray: Vec<_> = same_circle_degree_two_vertices(&topo, fused)
                .into_iter()
                .filter(|(p, a)| p[2].abs() < 1e-9 && !is_quarter_station(*a))
                .collect();
            assert!(
                stray.is_empty(),
                "{label}: stray bottom rim vertices {stray:?}"
            );
        }
    }
}

/// General-position placements on the paths the rim subdivision serves: a
/// cylinder swallowing the box's vertical corner edge (the protruding-crescent
/// case the subdivision was introduced for), ones crossing a single side face
/// off centre, one wholly inside the footprint and poking out of the top only.
/// The union must stay valid with the oracle volume.
#[test]
fn general_position_cylinder_fuses_stay_exact() {
    for (cx, cy) in [
        (2.0, 2.0),
        (-3.0, 4.0),
        (5.0, 9.0),
        (15.0, 9.0),
        (15.0, -3.0),
    ] {
        let label = format!("axis at ({cx}, {cy})");
        let (topo, fused) = fuse_at(cx, cy);
        let expected = (BOX.0 * BOX.1).mul_add(BOX.2, PI * RADIUS * RADIUS * CYL_HEIGHT)
            - footprint_overlap_area(cx, cy) * BOX.2;
        assert_valid_with_volume(&topo, fused, expected, &label);
    }
}
