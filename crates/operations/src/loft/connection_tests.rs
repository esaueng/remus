//! Loft connections: which vertex of each section the correspondence starts
//! from. A ruled loft between parallel sections moves every section vertex
//! linearly, so its section area is quadratic in height and the prismatoid
//! formula `h/6·(A₀ + 4·A_m + A₁)` is exact, with `A_m` the area of the
//! polygon of corresponding vertices' midpoints.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::explorer::solid_edges;
use remus_topology::face::{Face, FaceId, FaceSurface};

use super::*;

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

/// A 2 × 2 square about the Z axis at height `z`, counter-clockwise from
/// (−1, −1).
fn square(topo: &mut Topology, z: f64) -> FaceId {
    let wire = make_polygon_wire(
        topo,
        &[
            p(-1.0, -1.0, z),
            p(1.0, -1.0, z),
            p(1.0, 1.0, z),
            p(-1.0, 1.0, z),
        ],
        1e-7,
    )
    .unwrap();
    let normal = Vec3::new(0.0, 0.0, 1.0);
    topo.add_face(Face::new(wire, vec![], FaceSurface::Plane { normal, d: z }))
}

fn volume(topo: &Topology, solid: remus_topology::solid::SolidId) -> f64 {
    crate::measure::solid_volume(topo, solid, 0.05).unwrap()
}

/// Whether the solid has a straight edge joining `a` and `b`.
fn joins(topo: &Topology, solid: remus_topology::solid::SolidId, a: Point3, b: Point3) -> bool {
    solid_edges(topo, solid).unwrap().into_iter().any(|e| {
        let edge = topo.edge(e).unwrap();
        let (s, t) = (
            topo.vertex(edge.start()).unwrap().point(),
            topo.vertex(edge.end()).unwrap().point(),
        );
        ((s - a).length() < 1e-9 && (t - b).length() < 1e-9)
            || ((s - b).length() < 1e-9 && (t - a).length() < 1e-9)
    })
}

#[test]
fn connecting_matching_corners_is_the_plain_loft() {
    let mut topo = Topology::new();
    let (a, b) = (square(&mut topo, 0.0), square(&mut topo, 10.0));
    let starts = [p(-1.0, -1.0, 0.0), p(-1.0, -1.0, 10.0)];
    let solid = loft_with_connections(&mut topo, &[a, b], &starts, false).unwrap();
    assert!(
        crate::validate::validate_solid(&topo, solid)
            .unwrap()
            .is_valid()
    );
    assert!((volume(&topo, solid) - 40.0).abs() < 1e-9);
    assert!(joins(&topo, solid, p(-1.0, -1.0, 0.0), p(-1.0, -1.0, 10.0)));
}

#[test]
fn shifting_a_connection_by_one_corner_twists_the_loft_a_quarter_turn() {
    let mut topo = Topology::new();
    let (a, b) = (square(&mut topo, 0.0), square(&mut topo, 10.0));
    // (−1, −1) at the bottom connects to (1, −1) at the top, and so on round.
    let starts = [p(-1.0, -1.0, 0.0), p(1.0, -1.0, 10.0)];
    let solid = loft_with_connections(&mut topo, &[a, b], &starts, false).unwrap();
    assert!(
        crate::validate::validate_solid(&topo, solid)
            .unwrap()
            .is_valid()
    );
    assert!(joins(&topo, solid, p(-1.0, -1.0, 0.0), p(1.0, -1.0, 10.0)));
    assert!(!joins(
        &topo,
        solid,
        p(-1.0, -1.0, 0.0),
        p(-1.0, -1.0, 10.0)
    ));
    // Midpoints (0, −1), (1, 0), (0, 1), (−1, 0): a diamond of area 2.
    let exact = 10.0 / 6.0 * (4.0 + 4.0 * 2.0 + 4.0);
    let measured = volume(&topo, solid);
    assert!(
        (measured - exact).abs() / exact < 1e-9,
        "{measured} vs {exact}"
    );
}

#[test]
fn connection_refusals_are_typed_and_leave_the_topology_unchanged() {
    for (label, starts, phrase) in [
        (
            "a point that is not a vertex",
            vec![p(0.0, -1.0, 0.0), p(1.0, -1.0, 10.0)],
            "is not a vertex of section 0",
        ),
        (
            "one point for two sections",
            vec![p(-1.0, -1.0, 0.0)],
            "one point per section",
        ),
    ] {
        let mut topo = Topology::new();
        let (a, b) = (square(&mut topo, 0.0), square(&mut topo, 10.0));
        let faces = topo.num_faces();
        let error = loft_with_connections(&mut topo, &[a, b], &starts, false).expect_err(label);
        let crate::OperationsError::InvalidInput { reason } = &error else {
            panic!("{label}: untyped {error:?}");
        };
        assert!(reason.contains(phrase), "{label}: {reason}");
        assert_eq!(topo.num_faces(), faces, "{label}: faces left behind");
    }
}

/// A rounded square, half-width 10 with corner radius 2, at height `z`:
/// four lines and four quarter arcs, counter-clockwise from (−8, −10).
fn rounded_square(topo: &mut Topology, z: f64) -> FaceId {
    use remus_math::curves::Circle3D;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};
    let zn = Vec3::new(0.0, 0.0, 1.0);
    let corners = [(8.0, -8.0), (8.0, 8.0), (-8.0, 8.0), (-8.0, -8.0)];
    let mut points = Vec::new();
    for (k, &(cx, cy)) in corners.iter().enumerate() {
        #[allow(clippy::cast_precision_loss)]
        let a0 = -std::f64::consts::FRAC_PI_2 + std::f64::consts::FRAC_PI_2 * k as f64;
        let a1 = a0 + std::f64::consts::FRAC_PI_2;
        points.push((
            p(cx + 2.0 * a0.cos(), cy + 2.0 * a0.sin(), z),
            Some((cx, cy, a0, a1)),
        ));
        points.push((p(cx + 2.0 * a1.cos(), cy + 2.0 * a1.sin(), z), None));
    }
    // Rotate so the loop starts with the bottom line at (−8, −10).
    points.rotate_right(1);
    let vertices: Vec<_> = points
        .iter()
        .map(|&(q, _)| topo.add_vertex(Vertex::new(q, 1e-7)))
        .collect();
    let n = vertices.len();
    let mut edges = Vec::with_capacity(n);
    for i in 0..n {
        let (a, b) = (vertices[i], vertices[(i + 1) % n]);
        let edge = match points[i].1 {
            Some((cx, cy, a0, a1)) => {
                let circle = Circle3D::with_axes(
                    p(cx, cy, z),
                    zn,
                    2.0,
                    Vec3::new(1.0, 0.0, 0.0),
                    Vec3::new(0.0, 1.0, 0.0),
                )
                .unwrap();
                let mut edge = Edge::new(a, b, EdgeCurve::Circle(circle));
                edge.set_trim(Some((a0, a1)));
                edge
            }
            None => Edge::new(a, b, EdgeCurve::Line),
        };
        edges.push(OrientedEdge::new(topo.add_edge(edge), true));
    }
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane { normal: zn, d: z },
    ))
}

/// Review regression: on curved sections a connection keeps the exact
/// curve-preserving loft when the edges still match from the chosen
/// vertices, and is refused, never chorded, when they do not.
#[test]
fn curved_sections_stay_exact_or_are_refused() {
    let mut topo = Topology::new();
    let (a, b) = (
        rounded_square(&mut topo, 0.0),
        rounded_square(&mut topo, 10.0),
    );
    let starts = [p(8.0, -10.0, 0.0), p(8.0, -10.0, 10.0)];
    let solid = loft_with_connections(&mut topo, &[a, b], &starts, false).unwrap();
    assert!(
        crate::validate::validate_solid(&topo, solid)
            .unwrap()
            .is_valid()
    );
    let exact = 10.0 * (16.0f64.mul_add(-1.0, 400.0) + 4.0 * std::f64::consts::PI);
    let measured = volume(&topo, solid);
    assert!(
        (measured - exact).abs() / exact < 1e-9,
        "{measured} vs {exact}"
    );

    let mut topo = Topology::new();
    let (a, b) = (
        rounded_square(&mut topo, 0.0),
        rounded_square(&mut topo, 10.0),
    );
    let faces = topo.num_faces();
    // An arc first below, a line first above: no exact correspondence.
    let starts = [p(8.0, -10.0, 0.0), p(10.0, -8.0, 10.0)];
    let error = loft_with_connections(&mut topo, &[a, b], &starts, false).unwrap_err();
    let crate::OperationsError::InvalidInput { reason } = &error else {
        panic!("untyped {error:?}");
    };
    assert!(reason.contains("same edges"), "{reason}");
    assert_eq!(topo.num_faces(), faces);
}

/// Review regressions: a circle section, a non-finite point and a smooth
/// loft are refused by type.
#[test]
fn circles_nan_and_smooth_connections_are_refused() {
    use remus_topology::builder::make_circle_edge;
    use remus_topology::wire::{OrientedEdge, Wire};
    let mut topo = Topology::new();
    let zn = Vec3::new(0.0, 0.0, 1.0);
    let disc = |topo: &mut Topology, z: f64| {
        let edge = make_circle_edge(topo, p(0.0, 0.0, z), zn, 1.0, 1e-7).unwrap();
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
        topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane { normal: zn, d: z },
        ))
    };
    let (a, b) = (disc(&mut topo, 0.0), disc(&mut topo, 10.0));
    let error = loft_with_connections(
        &mut topo,
        &[a, b],
        &[p(1.0, 0.0, 0.0), p(1.0, 0.0, 10.0)],
        false,
    )
    .unwrap_err();
    assert!(error.to_string().contains("one closed curve"), "{error}");

    let (c, d) = (square(&mut topo, 0.0), square(&mut topo, 10.0));
    let error = loft_with_connections(
        &mut topo,
        &[c, d],
        &[p(f64::NAN, 0.0, 0.0), p(-1.0, -1.0, 10.0)],
        false,
    )
    .unwrap_err();
    assert!(error.to_string().contains("not finite"), "{error}");
    let error = loft_with_connections(
        &mut topo,
        &[c, d],
        &[p(-1.0, -1.0, 0.0), p(-1.0, -1.0, 10.0)],
        true,
    )
    .unwrap_err();
    assert!(error.to_string().contains("ruled loft"), "{error}");
}
