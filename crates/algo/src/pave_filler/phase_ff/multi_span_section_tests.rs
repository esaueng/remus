//! Faces of many spans: a helical sweep side covering every turn of a coil.
//!
//! The broad phase must bound the face's bulge past its boundary rails, and
//! the plane section must seed every span, or sections come back fragmented
//! and the boolean answers without the cut.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::f64::consts::{FRAC_1_SQRT_2, TAU};

use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use super::{compute_face_bbox, compute_face_bboxes_against, plane_nurbs_intersection};

/// The outer half of a radius-1.5 wire coiled three turns at radius 12,
/// pitch 5: a rational quadratic half circle swept by 210 rational quadratic
/// arc spans with the rise added per control row. Its rails lie on the
/// wire's middle radius 12, its bulge reaches 13.5.
fn coil_outer_half() -> NurbsSurface {
    let profile = [
        (12.0, 1.5),
        (13.5, 1.5),
        (13.5, 0.0),
        (13.5, -1.5),
        (12.0, -1.5),
    ];
    let profile_weights = [1.0, FRAC_1_SQRT_2, 1.0, FRAC_1_SQRT_2, 1.0];
    let (turns, pitch, spans) = (3.0_f64, 5.0_f64, 210_usize);
    #[allow(clippy::cast_precision_loss)]
    let n = spans as f64;
    let half = 0.5 * TAU * turns / n;
    let rows: Vec<(f64, f64, f64, f64, f64)> = (0..=2 * spans)
        .map(|k| {
            #[allow(clippy::cast_precision_loss)]
            let f = k as f64 / (2.0 * n);
            let (sin, cos) = (TAU * turns * f).sin_cos();
            let middle = k % 2 == 1;
            (
                cos,
                sin,
                if middle { 1.0 / half.cos() } else { 1.0 },
                pitch * turns * f,
                if middle { half.cos() } else { 1.0 },
            )
        })
        .collect();
    let control_points = profile
        .iter()
        .map(|&(r, z)| {
            rows.iter()
                .map(|&(c, s, scale, rise, _)| Point3::new(r * c * scale, r * s * scale, z + rise))
                .collect()
        })
        .collect();
    let weights = profile_weights
        .iter()
        .map(|&w| rows.iter().map(|row| w * row.4).collect())
        .collect();
    let mut knots_v = vec![0.0; 3];
    for k in 1..spans {
        #[allow(clippy::cast_precision_loss)]
        let t = k as f64 / n;
        knots_v.extend([t, t]);
    }
    knots_v.extend([1.0; 3]);
    NurbsSurface::new(
        2,
        2,
        vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
        knots_v,
        control_points,
        weights,
    )
    .unwrap()
}

/// The coil face bounded by its four iso-boundaries.
fn coil_face(topo: &mut Topology, surface: &NurbsSurface) -> FaceId {
    let points = surface.control_points();
    let weights = surface.weights();
    let (rows, cols) = (points.len(), points[0].len());
    let iso = |along_u: bool, index: usize| {
        let (cps, ws, degree, knots) = if along_u {
            (
                points.iter().map(|row| row[index]).collect(),
                weights.iter().map(|row| row[index]).collect(),
                surface.degree_u(),
                surface.knots_u().to_vec(),
            )
        } else {
            (
                points[index].clone(),
                weights[index].clone(),
                surface.degree_v(),
                surface.knots_v().to_vec(),
            )
        };
        NurbsCurve::new(degree, knots, cps, ws).unwrap()
    };
    let corner = |i: usize, j: usize| points[i][j];
    let v00 = topo.add_vertex(Vertex::new(corner(0, 0), 1e-7));
    let v10 = topo.add_vertex(Vertex::new(corner(rows - 1, 0), 1e-7));
    let v01 = topo.add_vertex(Vertex::new(corner(0, cols - 1), 1e-7));
    let v11 = topo.add_vertex(Vertex::new(corner(rows - 1, cols - 1), 1e-7));
    let edge = |topo: &mut Topology, a, b, curve: NurbsCurve| {
        let domain = curve.domain();
        let mut edge = Edge::new(a, b, EdgeCurve::NurbsCurve(curve));
        edge.set_trim(Some(domain));
        topo.add_edge(edge)
    };
    let start = edge(topo, v00, v10, iso(true, 0));
    let far_rail = edge(topo, v10, v11, iso(false, rows - 1));
    let end = edge(topo, v01, v11, iso(true, cols - 1));
    let near_rail = edge(topo, v00, v01, iso(false, 0));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(start, true),
                OrientedEdge::new(far_rail, true),
                OrientedEdge::new(end, false),
                OrientedEdge::new(near_rail, false),
            ],
            true,
        )
        .unwrap(),
    );
    topo.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(surface.clone())))
}

#[test]
fn a_whole_carrier_nurbs_face_box_holds_its_bulge() {
    let surface = coil_outer_half();
    let mut topo = Topology::new();
    let face = coil_face(&mut topo, &surface);
    let tol = Tolerance::new();
    let bbox = compute_face_bbox(&topo, face, tol).unwrap();
    // The rails stay on radius 12; the tube's outer side reaches 13.5.
    assert!(bbox.max.y() >= 13.5 - 1e-9, "box {bbox:?} misses the bulge");
    assert!(
        bbox.min.x() <= -13.5 + 1e-9,
        "box {bbox:?} misses the bulge"
    );

    // The partner-pruned box must not reject a partner the bulge reaches:
    // a box face's extent on y = 13 a quarter turn round.
    let partner = remus_math::aabb::Aabb3 {
        min: Point3::new(-1.0, 13.0, 0.5),
        max: Point3::new(1.0, 13.0, 2.5),
    };
    let pruned = compute_face_bboxes_against(&topo, &[face], &[partner], tol).unwrap();
    assert!(pruned[0].intersects(partner), "pruned box {:?}", pruned[0]);
}

#[test]
fn plane_sections_of_a_many_span_face_come_back_whole() {
    let surface = coil_outer_half();
    // z = 7.5: one open section from the top rail to the bottom rail.
    let across = plane_nurbs_intersection(Vec3::new(0.0, 0.0, 1.0), 7.5, &surface).unwrap();
    assert_eq!(across.len(), 1);
    assert!((across[0].p_start - across[0].p_end).length() > 1.0);
    // y = 13.2: the outer bulge of every turn, three closed loops.
    let loops = plane_nurbs_intersection(Vec3::new(0.0, 1.0, 0.0), 13.2, &surface).unwrap();
    assert_eq!(loops.len(), 3);
    for raw in &loops {
        assert!(
            (raw.p_start - raw.p_end).length() < 1e-9,
            "loop open between {:?} and {:?}",
            raw.p_start,
            raw.p_end
        );
        assert!((raw.p_start.y() - 13.2).abs() < 1e-9);
    }
}
