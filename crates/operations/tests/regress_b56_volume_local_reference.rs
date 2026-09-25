//! B56: the mesh-sum volume routes must not depend on where the body sits.
//!
//! Every route in `measure/volume.rs` that sums signed tetrahedra over
//! triangles used to take them about the WORLD ORIGIN. For a body of size `L`
//! a distance `|offset|` away, each triple product is of order
//! `|offset|²·L` and carries a rounding error of order `ε·|offset|³`, while
//! the answer is of order `L³`: a 1e-3 cone–sphere boolean moved by
//! (13, −7, 5) kept a mesh of identical shape yet read up to 0.7 % off
//! (`regress_b40_conesphere_small_scale::b40_solid_volume_far_translation_small_scale`).
//! The routes now take their tetrahedra about the centre of the body's own
//! bounding box whenever the triangles' directed edges cancel, which makes
//! the sum the same about every point. Bodies with a hole or an inward-wound
//! face keep the old origin sum (see `measure::volume::summation_point`).
//!
//! Oracles, all independent of the kernel's volume routing:
//!
//! - a triangulated icosphere built from its own vertex list, whose polyhedral
//!   volume the test sums about the sphere's centre in local coordinates, and
//!   whose centroid is its centre by symmetry;
//! - the whole-solid mesh of a primitive at the origin, whose shape the
//!   harness's offset leaves unchanged, so its enclosed volume must not move.
//!
//! Each route is asserted at 1e-3 (where the old sum failed), 1 and 1e3.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::measure::{
    oriented_solid_volume, solid_center_of_mass, solid_volume, solid_volume_from_faces,
};
use remus_operations::primitives::{make_cone, make_sphere};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];
/// The B26 harness's absolute placement offset.
const OFFSET: [f64; 3] = [13.0, -7.0, 5.0];

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

/// Unit icosphere: an icosahedron subdivided `levels` times, vertices pushed
/// onto the unit sphere. Returns the vertices and outward-wound triangles.
fn unit_icosphere(levels: usize) -> (Vec<Vec3>, Vec<[usize; 3]>) {
    let t = f64::midpoint(1.0, 5.0_f64.sqrt());
    let mut verts: Vec<Vec3> = [
        (-1.0, t, 0.0),
        (1.0, t, 0.0),
        (-1.0, -t, 0.0),
        (1.0, -t, 0.0),
        (0.0, -1.0, t),
        (0.0, 1.0, t),
        (0.0, -1.0, -t),
        (0.0, 1.0, -t),
        (t, 0.0, -1.0),
        (t, 0.0, 1.0),
        (-t, 0.0, -1.0),
        (-t, 0.0, 1.0),
    ]
    .iter()
    .map(|&(x, y, z)| Vec3::new(x, y, z).normalize().unwrap())
    .collect();
    let mut tris: Vec<[usize; 3]> = vec![
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];
    for _ in 0..levels {
        let mut midpoints = std::collections::BTreeMap::new();
        let mut midpoint = |a: usize, b: usize, verts: &mut Vec<Vec3>| {
            *midpoints.entry((a.min(b), a.max(b))).or_insert_with(|| {
                verts.push((verts[a] + verts[b]).normalize().unwrap());
                verts.len() - 1
            })
        };
        tris = tris
            .iter()
            .flat_map(|&[a, b, c]| {
                let ab = midpoint(a, b, &mut verts);
                let bc = midpoint(b, c, &mut verts);
                let ca = midpoint(c, a, &mut verts);
                [[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]]
            })
            .collect();
    }
    (verts, tris)
}

/// Volume of the icosphere of radius `radius`, summed about its own centre
/// in local coordinates: the polyhedron's exact volume to round-off at any
/// placement.
fn icosphere_volume(radius: f64, levels: usize) -> f64 {
    let (verts, tris) = unit_icosphere(levels);
    let six: f64 = tris
        .iter()
        .map(|&[a, b, c]| (verts[a] * radius).dot((verts[b] * radius).cross(verts[c] * radius)))
        .sum();
    six / 6.0
}

/// The icosphere as a B-Rep of planar triangle faces with shared line edges
/// (what a mesh import looks like), centred at `centre`: the shape that
/// reaches `solid_volume_from_faces` and the face-based centroid.
fn triangulated_icosphere(
    topo: &mut Topology,
    centre: Point3,
    radius: f64,
    levels: usize,
) -> SolidId {
    let (verts, tris) = unit_icosphere(levels);
    let points: Vec<Point3> = verts.iter().map(|&v| centre + v * radius).collect();
    let tol = 1e-9 * radius;
    let ids: Vec<VertexId> = points
        .iter()
        .map(|&p| topo.add_vertex(Vertex::new(p, tol)))
        .collect();
    let mut pool: std::collections::BTreeMap<(usize, usize), EdgeId> =
        std::collections::BTreeMap::new();
    let mut faces = Vec::with_capacity(tris.len());
    for t in tris {
        let (p0, p1, p2) = (points[t[0]], points[t[1]], points[t[2]]);
        let normal = (p1 - p0).cross(p2 - p0).normalize().unwrap();
        let d = normal.dot(p0 - Point3::new(0.0, 0.0, 0.0));
        let oriented: Vec<OrientedEdge> = (0..3)
            .map(|k| {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                let key = (a.min(b), a.max(b));
                let eid = *pool.entry(key).or_insert_with(|| {
                    topo.add_edge(Edge::new(ids[key.0], ids[key.1], EdgeCurve::Line))
                });
                OrientedEdge::new(eid, key.0 == a)
            })
            .collect();
        let wire = topo.add_wire(Wire::new(oriented, true).unwrap());
        faces.push(topo.add_face(Face::new(wire, vec![], FaceSurface::Plane { normal, d })));
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

/// A triangulated body far from the origin: its face-based volume and
/// centroid must match the polyhedron's own, at every scale. At 1e-3 scale
/// the origin-referenced sums were off by far more than the bounds here.
#[test]
fn b56_triangulated_body_far_from_origin() {
    const LEVELS: usize = 3; // 1280 triangles
    for scale in SCALES {
        let centre = Point3::new(OFFSET[0], OFFSET[1], OFFSET[2]);
        let expected = icosphere_volume(scale, LEVELS);
        let mut topo = Topology::new();
        let solid = triangulated_icosphere(&mut topo, centre, scale, LEVELS);

        let v = solid_volume_from_faces(&topo, solid, 0.01 * scale).unwrap();
        assert!(
            rel(v, expected) <= 1e-9,
            "scale {scale}: solid_volume_from_faces {v:e}, polyhedron {expected:e} ({:e})",
            rel(v, expected)
        );
        let v = solid_volume(&topo, solid, 0.01 * scale).unwrap();
        assert!(
            rel(v, expected) <= 1e-9,
            "scale {scale}: solid_volume {v:e}, polyhedron {expected:e} ({:e})",
            rel(v, expected)
        );

        // The icosphere is centrally symmetric about its centre.
        let com = solid_center_of_mass(&topo, solid, 0.01 * scale).unwrap();
        let off = (com - centre).length();
        assert!(
            off <= 1e-9 * scale,
            "scale {scale}: centroid {com:?} is {off:e} from the centre {centre:?}"
        );
    }
}

/// Whole-solid mesh routes: moving a primitive by the harness offset leaves
/// its mesh's shape unchanged, so the enclosed volume must not move with it.
/// `oriented_solid_volume` always reads the mesh; `solid_volume` reaches it
/// for the cone–sphere fuse (see the B40 witness), so the sign-carrying
/// mesh reading is the cleanest probe of the sum itself.
#[test]
fn b56_whole_solid_mesh_volume_is_translation_invariant() {
    for scale in SCALES {
        for (name, build) in [
            (
                "sphere",
                (|topo: &mut Topology, s: f64| make_sphere(topo, 0.7 * s, 16).unwrap())
                    as fn(&mut Topology, f64) -> SolidId,
            ),
            ("cone frustum", |topo: &mut Topology, s: f64| {
                make_cone(topo, s, 0.5 * s, s).unwrap()
            }),
        ] {
            let mut topo = Topology::new();
            let solid = build(&mut topo, scale);
            let deflection = 1e-3 * scale;
            let in_place = oriented_solid_volume(&topo, solid, deflection).unwrap();
            transform_solid(
                &mut topo,
                solid,
                &Mat4::translation(OFFSET[0], OFFSET[1], OFFSET[2]),
            )
            .unwrap();
            let moved = oriented_solid_volume(&topo, solid, deflection).unwrap();
            assert!(
                in_place > 0.0 && rel(moved, in_place) <= 1e-6,
                "{name} at scale {scale}: mesh volume {moved:e} moved, {in_place:e} in place ({:e})",
                rel(moved, in_place)
            );
        }
    }
}
