//! Planar faces whose inner wires nest (a hole holding an island holding a
//! hole) alternate void and material by nesting depth. Both planar paths, the
//! exact Green's-theorem rule and the chord-polygon fan, are checked against
//! closed-form rectangle sums and against the signed composition of
//! single-loop faces.

#![allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]

use super::*;
use remus_topology::builder::make_polygon_wire;
use remus_topology::face::Face;

/// `([x0, x1], [y0, y1])`.
type Rect = ([f64; 2], [f64; 2]);

/// Outer `[0, 14] x [0, 8]`; hole `[1, 9] x [1, 7]` holding the island
/// `[2, 6] x [2, 6]`, which holds the hole `[3, 4] x [3, 5]`; and a second,
/// unnested hole `[10, 13] x [2, 5]` beside them.
const OUTER: Rect = ([0.0, 14.0], [0.0, 8.0]);
const INNER: [Rect; 4] = [
    ([1.0, 9.0], [1.0, 7.0]),
    ([2.0, 6.0], [2.0, 6.0]),
    ([3.0, 4.0], [3.0, 5.0]),
    ([10.0, 13.0], [2.0, 5.0]),
];
/// Material sign of each inner loop: hole, island, hole, hole.
const SIGNS: [f64; 4] = [-1.0, 1.0, -1.0, -1.0];

/// A rigid map that tilts the plane off every axis, so every moment of the
/// face is nonzero.
fn tilted(p: Point3) -> Point3 {
    let (sa, ca) = 0.4_f64.sin_cos();
    let (sb, cb) = 0.7_f64.sin_cos();
    // Rotate about x by a, then about z by b, then shift.
    let (x, y, z) = (p.x(), ca * p.y() - sa * p.z(), sa * p.y() + ca * p.z());
    Point3::new(cb * x - sb * y + 3.0, sb * x + cb * y - 2.0, z + 1.5)
}

fn identity(p: Point3) -> Point3 {
    p
}

fn rect_points(rect: Rect, z: f64, ccw: bool, map: fn(Point3) -> Point3) -> Vec<Point3> {
    let ([x0, x1], [y0, y1]) = rect;
    let mut pts = vec![
        Point3::new(x0, y0, z),
        Point3::new(x1, y0, z),
        Point3::new(x1, y1, z),
        Point3::new(x0, y1, z),
    ];
    if !ccw {
        pts.reverse();
    }
    pts.into_iter().map(map).collect()
}

fn plane_of(map: fn(Point3) -> Point3, z: f64) -> FaceSurface {
    let origin = map(Point3::new(0.0, 0.0, z));
    let normal = map(Point3::new(0.0, 0.0, z + 1.0)) - origin;
    FaceSurface::Plane {
        normal,
        d: normal.dot(origin - Point3::new(0.0, 0.0, 0.0)),
    }
}

/// The nested face, and one single-loop face per loop (outer first).
fn faces(topo: &mut Topology, z: f64, map: fn(Point3) -> Point3) -> (FaceId, Vec<FaceId>) {
    let surface = plane_of(map, z);
    let outer = make_polygon_wire(topo, &rect_points(OUTER, z, true, map), 1e-9).unwrap();
    let inner: Vec<_> = INNER
        .iter()
        .map(|&r| make_polygon_wire(topo, &rect_points(r, z, false, map), 1e-9).unwrap())
        .collect();
    let nested = topo.add_face(Face::new(outer, inner, surface.clone()));
    let singles = std::iter::once(OUTER)
        .chain(INNER)
        .map(|r| {
            let wire = make_polygon_wire(topo, &rect_points(r, z, true, map), 1e-9).unwrap();
            topo.add_face(Face::new(wire, vec![], surface.clone()))
        })
        .collect();
    (nested, singles)
}

fn fields(c: &FaceContribution) -> [f64; 14] {
    [
        c.area,
        c.volume,
        c.volume_moment_x,
        c.volume_moment_y,
        c.volume_moment_z,
        c.volume_second_x,
        c.volume_second_y,
        c.volume_second_z,
        c.volume_product_xy,
        c.volume_product_xz,
        c.volume_product_yz,
        c.centroid_x,
        c.centroid_y,
        c.centroid_z,
    ]
}

/// The face's two planar paths about `reference`: the exact rule, then the
/// chord-polygon fan.
fn both_paths(topo: &Topology, face: FaceId, reference: Point3) -> [FaceContribution; 2] {
    let FaceSurface::Plane { normal, .. } = topo.face(face).unwrap().surface() else {
        panic!("planar face expected");
    };
    let normal = normal.normalize().unwrap();
    let exact = integrate_planar_face_exact(topo, face, normal, reference)
        .unwrap()
        .expect("line boundaries take the exact rule");
    let fan = integrate_planar_face_fan(topo, face, normal, reference).unwrap();
    [exact, fan]
}

#[test]
fn nested_inner_wires_alternate_void_and_material_in_closed_form() {
    let mut topo = Topology::new();
    let z = 5.0;
    let (face, _) = faces(&mut topo, z, identity);
    let rect = |([x0, x1], [y0, y1]): Rect| {
        let area = (x1 - x0) * (y1 - y0);
        (area, area * 0.5 * (x0 + x1), area * 0.5 * (y0 + y1))
    };
    let (mut area, mut sx, mut sy) = rect(OUTER);
    for (r, sign) in INNER.into_iter().zip(SIGNS) {
        let (a, x, y) = rect(r);
        area += sign * a;
        sx += sign * x;
        sy += sign * y;
    }
    // 112 − 48 + 16 − 2 − 9: subtracting every inner wire would read
    // 112 − 48 − 16 − 2 − 9 = 37.
    assert_eq!(area.to_bits(), 69.0_f64.to_bits());
    for c in both_paths(&topo, face, Point3::new(0.0, 0.0, 0.0)) {
        let close = |got: f64, want: f64| (got - want).abs() <= 1e-11 * want.abs().max(1.0);
        assert!(close(c.area, area), "area {} != {area}", c.area);
        // ∫ (P · n) / 3 dA with n = +z on z = 5.
        assert!(close(c.volume, z * area / 3.0), "volume {}", c.volume);
        assert!(close(c.centroid_x, sx), "∫x dA {} != {sx}", c.centroid_x);
        assert!(close(c.centroid_y, sy), "∫y dA {} != {sy}", c.centroid_y);
        assert!(close(c.centroid_z, z * area), "∫z dA {}", c.centroid_z);
        assert!(close(c.volume_moment_z, 0.5 * z * z * area));
        assert!(close(c.volume_second_z, z.powi(3) * area / 3.0));
    }
    // The public entry points read the same face.
    let whole = integrate_face(&topo, face, 5).unwrap();
    assert!((whole.area - area).abs() <= 1e-11 * area);
    let exact_area = planar_face_exact_area(&topo, face).unwrap().unwrap();
    assert!((exact_area - area).abs() <= 1e-11 * area);
}

#[test]
fn nested_inner_wires_compose_every_moment_with_alternating_signs() {
    let mut topo = Topology::new();
    let (face, singles) = faces(&mut topo, 0.0, tilted);
    let reference = Point3::new(0.5, -1.0, 2.0);
    let nested = both_paths(&topo, face, reference);
    let parts: Vec<_> = singles
        .iter()
        .map(|&f| both_paths(&topo, f, reference))
        .collect();
    for path in 0..2 {
        let mut want = fields(&parts[0][path]);
        for (part, sign) in parts[1..].iter().zip(SIGNS) {
            for (w, v) in want.iter_mut().zip(fields(&part[path])) {
                *w += sign * v;
            }
        }
        let got = fields(&nested[path]);
        let scale = want.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        for (k, (g, w)) in got.iter().zip(want).enumerate() {
            assert!(
                w.abs() > 1e-6 || k == 0,
                "path {path} field {k} is degenerate ({w}); tilt the plane further"
            );
            assert!(
                (g - w).abs() <= 1e-12 * scale,
                "path {path} field {k}: {g} != {w}"
            );
        }
    }
    assert!((nested[0].area - 69.0).abs() <= 1e-11 * 69.0);
}

#[test]
fn side_by_side_holes_keep_subtracting() {
    // Two holes in a row and a third inside neither: depth 0 for all, so the
    // face reads outer minus every hole as before.
    let mut topo = Topology::new();
    let outer = make_polygon_wire(
        &mut topo,
        &rect_points(([0.0, 10.0], [0.0, 4.0]), 0.0, true, identity),
        1e-9,
    )
    .unwrap();
    let holes: Vec<_> = [
        ([1.0, 3.0], [1.0, 3.0]),
        ([4.0, 6.0], [1.0, 3.0]),
        ([7.0, 9.0], [1.0, 2.0]),
    ]
    .into_iter()
    .map(|r| make_polygon_wire(&mut topo, &rect_points(r, 0.0, false, identity), 1e-9).unwrap())
    .collect();
    let face = topo.add_face(Face::new(outer, holes, plane_of(identity, 0.0)));
    for c in both_paths(&topo, face, Point3::new(0.0, 0.0, 0.0)) {
        assert!((c.area - 30.0).abs() <= 1e-12 * 30.0, "area {}", c.area);
    }
}
