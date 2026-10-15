//! B126: the all-planar-triangle volume and centroid routes must subtract the
//! holes of a pierced face, wherever the body sits.
//!
//! `solid_volume_from_faces` (and through it `solid_volume`) and the
//! face-based centroid read each planar triangle face as its outer triangle
//! only. A tetrahedron pierced by a triangular tunnel keeps triangle outer
//! wires on the two faces the tunnel opens, so it took that route, credited
//! both openings, and — the openings' edges missing from its edge census —
//! summed its tetrahedra about the world origin, so the reading also moved
//! with the body.
//!
//! Oracle, independent of the kernel: the tetrahedron `x, y, z >= 0,
//! x + y + z <= a` minus the vertical prism over the triangle `T` in the
//! `xy`-plane, which runs from `z = 0` up to the slanted face
//! `z = a - x - y`. With `f = a - x - y`, the tunnel's volume is `∫_T f` and
//! its first moments `∫_T x f`, `∫_T y f`, `∫_T f²/2`: quadratics, which the
//! triangle's edge-midpoint rule integrates exactly.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use remus_math::vec::{Point3, Vec3};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::{solid_center_of_mass, solid_volume, solid_volume_from_faces};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

/// Tetrahedron leg length.
const A: f64 = 12.0;
/// The tunnel's cross-section, counter-clockwise in the `xy`-plane.
const T: [[f64; 2]; 3] = [[2.0, 2.0], [5.0, 2.0], [2.0, 5.0]];
const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];
/// Placement far from the origin, so an origin-anchored sum shows.
const OFFSET: [f64; 3] = [37.0, -91.0, 13.0];

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

/// Closed-form volume and centroid of the unit-scale body at the origin.
fn oracle() -> (f64, [f64; 3]) {
    let [p, q, r] = T;
    let area = 0.5 * ((q[0] - p[0]) * (r[1] - p[1]) - (r[0] - p[0]) * (q[1] - p[1]));
    let midpoints =
        [[p, q], [q, r], [r, p]].map(|[u, v]| [0.5 * (u[0] + v[0]), 0.5 * (u[1] + v[1])]);
    let integrate = |g: &dyn Fn(f64, f64) -> f64| {
        area / 3.0 * midpoints.iter().map(|m| g(m[0], m[1])).sum::<f64>()
    };
    let f = |x: f64, y: f64| A - x - y;
    let tunnel = integrate(&f);
    let tunnel_moment = [
        integrate(&|x, y| x * f(x, y)),
        integrate(&|x, y| y * f(x, y)),
        integrate(&|x, y| 0.5 * f(x, y) * f(x, y)),
    ];
    let tet = A * A * A / 6.0;
    let volume = tet - tunnel;
    // The tetrahedron's centroid is A/4 on every axis.
    let centroid = tunnel_moment.map(|m| (tet * A / 4.0 - m) / volume);
    (volume, centroid)
}

/// The pierced tetrahedron as planar triangle faces with line edges, scaled
/// by `scale` and moved by `offset`. The two opened faces carry the tunnel's
/// mouths as inner wires; `holes_wound_with_outer` stores those wires wound
/// the same way as their face's triangle, which the measurement must not
/// depend on.
fn pierced_tetrahedron(
    topo: &mut Topology,
    scale: f64,
    offset: Vec3,
    holes_wound_with_outer: bool,
) -> SolidId {
    let local = |x: f64, y: f64, z: f64| Point3::new(x * scale, y * scale, z * scale) + offset;
    let mut points = vec![
        local(0.0, 0.0, 0.0),
        local(A, 0.0, 0.0),
        local(0.0, A, 0.0),
        local(0.0, 0.0, A),
    ];
    // Tunnel mouths: 4..7 on z = 0, 7..10 on x + y + z = A.
    points.extend(T.iter().map(|t| local(t[0], t[1], 0.0)));
    points.extend(T.iter().map(|t| local(t[0], t[1], A - t[0] - t[1])));
    let tol = 1e-9 * scale;
    let ids: Vec<VertexId> = points
        .iter()
        .map(|&p| topo.add_vertex(Vertex::new(p, tol)))
        .collect();

    let mut pool: BTreeMap<(usize, usize), EdgeId> = BTreeMap::new();
    let mut wire = |topo: &mut Topology, loop_: &[usize]| -> WireId {
        let oriented: Vec<OrientedEdge> = (0..loop_.len())
            .map(|k| {
                let (a, b) = (loop_[k], loop_[(k + 1) % loop_.len()]);
                let key = (a.min(b), a.max(b));
                let eid = *pool.entry(key).or_insert_with(|| {
                    topo.add_edge(Edge::new(ids[key.0], ids[key.1], EdgeCurve::Line))
                });
                OrientedEdge::new(eid, key.0 == a)
            })
            .collect();
        topo.add_wire(Wire::new(oriented, true).unwrap())
    };

    let (b, s) = ([4, 5, 6], [7, 8, 9]);
    // Outer triangles wound outward; each mouth wound against its face
    // (or with it, on request).
    let mut faces: Vec<(Vec<usize>, Option<Vec<usize>>)> = vec![
        (vec![0, 2, 1], Some(vec![b[0], b[1], b[2]])),
        (vec![0, 3, 2], None),
        (vec![0, 1, 3], None),
        (vec![1, 2, 3], Some(vec![s[0], s[2], s[1]])),
    ];
    if holes_wound_with_outer {
        for (_, hole) in &mut faces {
            if let Some(hole) = hole {
                hole.reverse();
            }
        }
    }
    // Tunnel walls, each planar quad split into two triangles facing into
    // the tunnel.
    for i in 0..3 {
        let j = (i + 1) % 3;
        faces.push((vec![b[j], b[i], s[i]], None));
        faces.push((vec![b[j], s[i], s[j]], None));
    }

    let face_ids = faces
        .iter()
        .map(|(outer, hole)| {
            let [p0, p1, p2] = [outer[0], outer[1], outer[2]].map(|i| points[i]);
            let normal = (p1 - p0).cross(p2 - p0).normalize().unwrap();
            let d = normal.dot(p0 - Point3::new(0.0, 0.0, 0.0));
            let outer_wire = wire(topo, outer);
            let inner: Vec<WireId> = hole.iter().map(|h| wire(topo, h)).collect();
            topo.add_face(Face::new(
                outer_wire,
                inner,
                FaceSurface::Plane { normal, d },
            ))
        })
        .collect();
    let shell = topo.add_shell(Shell::new(face_ids).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

#[test]
fn b126_pierced_tetrahedron_volume_subtracts_the_tunnel() {
    let (unit_volume, _) = oracle();
    assert!((unit_volume - 261.0).abs() <= 1e-12, "oracle {unit_volume}");
    for scale in SCALES {
        for offset in [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(OFFSET[0], OFFSET[1], OFFSET[2]),
        ] {
            for wound_with_outer in [false, true] {
                let mut topo = Topology::new();
                let solid = pierced_tetrahedron(&mut topo, scale, offset, wound_with_outer);
                let expected = unit_volume * scale.powi(3);
                let case = format!(
                    "scale {scale}, offset {offset:?}, holes wound with outer {wound_with_outer}"
                );

                let direct = solid_volume_from_faces(&topo, solid, 0.01 * scale).unwrap();
                assert!(
                    rel(direct, expected) <= 1e-9,
                    "{case}: solid_volume_from_faces {direct:e}, closed form {expected:e}"
                );
                let routed = solid_volume(&topo, solid, 0.01 * scale).unwrap();
                assert!(
                    rel(routed, expected) <= 1e-9,
                    "{case}: solid_volume {routed:e}, closed form {expected:e}"
                );
            }
        }
    }
}

#[test]
fn b126_pierced_tetrahedron_centroid_subtracts_the_tunnel() {
    let (_, unit_centroid) = oracle();
    for scale in SCALES {
        let offset = Vec3::new(OFFSET[0], OFFSET[1], OFFSET[2]);
        let mut topo = Topology::new();
        let solid = pierced_tetrahedron(&mut topo, scale, offset, false);
        let [x, y, z] = unit_centroid.map(|c| c * scale);
        let expected = Point3::new(x, y, z) + offset;
        let com = solid_center_of_mass(&topo, solid, 0.01 * scale).unwrap();
        let miss = (com - expected).length();
        assert!(
            miss <= 1e-9 * scale,
            "scale {scale}: centroid {com:?}, closed form {expected:?} ({miss:e} apart)"
        );
    }
}

/// The body is what the oracle says it is: the tunnel is a void and the
/// material beside it is solid.
#[test]
fn b126_pierced_tetrahedron_tunnel_is_empty() {
    let mut topo = Topology::new();
    let solid = pierced_tetrahedron(&mut topo, 1.0, Vec3::new(0.0, 0.0, 0.0), false);
    let probe = |p: Point3| classify_point(&topo, solid, p, 0.01, 1e-7).unwrap();
    // The tunnel's axis at T's centroid (3, 3), where the slanted face sits
    // at z = 6.
    for z in [1.0, 3.0, 5.0] {
        assert_eq!(
            probe(Point3::new(3.0, 3.0, z)),
            PointClassification::Outside,
            "tunnel interior at z = {z}"
        );
    }
    for p in [
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(6.0, 1.0, 1.0),
        Point3::new(1.0, 1.0, 8.0),
    ] {
        assert_eq!(probe(p), PointClassification::Inside, "material at {p:?}");
    }
}
