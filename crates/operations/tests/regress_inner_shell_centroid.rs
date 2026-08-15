//! `solid_center_of_mass` must weigh the body, not just its outer shell.
//!
//! remus#61 widened five VOLUME paths from `solid.outer_shell()` to
//! `explorer::solid_faces`, so a body hollowed by a cavity stopped reading at
//! its un-hollowed volume. It deliberately left the two CENTROID paths —
//! `solid_center_of_mass` and its all-planar-triangle fast path
//! `center_of_mass_from_faces` — carrying the identical blindness.
//!
//! That combination is worse than the original defect. A hollowed body now
//! reports the RIGHT volume and the WRONG centroid, and the correct volume is
//! exactly what makes the body look checked.
//!
//! Every void here is deliberately OFF-CENTRE. A concentric cavity leaves the
//! centroid where it already was, so it passes with the defect completely
//! intact; it is in the sweep only as the control that says so. The expected
//! centroid is the composite, written out:
//!
//! ```text
//! c = (V_outer * c_outer - V_void * c_void) / (V_outer - V_void)
//! ```
//!
//! Both bodies are all-planar, so both routes are EXACT before and after — no
//! mesh residual to hide behind. (#61 measured -1.2e-4 on an off-axis
//! cylindrical void and -3.3e-5 on sphere-in-sphere, because the analytic
//! recogniser refuses when an inner shell is present and a curved cavity then
//! comes back through an inscribed mesh. Nothing curved is asserted here.)
//!
//! Nothing below is a recorded kernel measurement, and `mass_properties` is
//! never the reference: it and `solid_center_of_mass` do not even share a code
//! path, but agreement between kernel routes is not what a closed form is for.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{solid_center_of_mass, solid_volume};
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

/// Scales, coarsest first so nothing can pass by being swept first. Every
/// length carries the factor and every tolerance is relative.
const SCALES: [f64; 3] = [1000.0, 1.0, 0.001];

/// An axis-aligned box: origin corner and side lengths, in model units.
#[derive(Clone, Copy)]
struct Brick {
    origin: [f64; 3],
    size: [f64; 3],
}

impl Brick {
    const fn new(origin: [f64; 3], size: [f64; 3]) -> Self {
        Self { origin, size }
    }

    fn scaled(self, k: f64) -> Self {
        Self {
            origin: self.origin.map(|c| c * k),
            size: self.size.map(|c| c * k),
        }
    }

    fn volume(self) -> f64 {
        self.size[0] * self.size[1] * self.size[2]
    }

    fn centre(self) -> Point3 {
        Point3::new(
            self.origin[0] + self.size[0] / 2.0,
            self.origin[1] + self.size[1] / 2.0,
            self.origin[2] + self.size[2] / 2.0,
        )
    }
}

/// The composite centroid of `outer` with `void` removed, by hand.
fn composite_centroid(outer: Brick, void: Brick) -> Point3 {
    let (vo, vv) = (outer.volume(), void.volume());
    let (co, cv) = (outer.centre(), void.centre());
    let mass = vo - vv;
    Point3::new(
        (vo * co.x() - vv * cv.x()) / mass,
        (vo * co.y() - vv * cv.y()) / mass,
        (vo * co.z() - vv * cv.z()) / mass,
    )
}

// ── Models ────────────────────────────────────────────────────

/// `Cut` a fully contained tool out of a blank: the tool becomes an inner
/// shell, and the result is a closed body with a sealed void.
fn cut_cavity(topo: &mut Topology, outer: Brick, void: Brick) -> SolidId {
    let blank = make_box(topo, outer.size[0], outer.size[1], outer.size[2]).unwrap();
    transform_solid(
        topo,
        blank,
        &Mat4::translation(outer.origin[0], outer.origin[1], outer.origin[2]),
    )
    .unwrap();
    let tool = make_box(topo, void.size[0], void.size[1], void.size[2]).unwrap();
    transform_solid(
        topo,
        tool,
        &Mat4::translation(void.origin[0], void.origin[1], void.origin[2]),
    )
    .unwrap();
    boolean(topo, BooleanOp::Cut, blank, tool).unwrap()
}

/// A TRIANGULATED brick shell: 8 vertices, 12 triangles, every edge shared by
/// exactly two faces. `reversed` marks each face reversed, which is how a
/// cavity shell is carried.
fn triangulated_brick_shell(
    topo: &mut Topology,
    b: Brick,
    reversed: bool,
) -> remus_topology::shell::ShellId {
    let tol = 1e-9 * b.size[0];
    let corner = |i: usize| {
        let (x, y, z) = (i & 1, (i >> 1) & 1, (i >> 2) & 1);
        Point3::new(
            b.origin[0] + b.size[0] * f64::from(u8::try_from(x).unwrap()),
            b.origin[1] + b.size[1] * f64::from(u8::try_from(y).unwrap()),
            b.origin[2] + b.size[2] * f64::from(u8::try_from(z).unwrap()),
        )
    };
    let v: Vec<VertexId> = (0..8)
        .map(|i| topo.add_vertex(Vertex::new(corner(i), tol)))
        .collect();

    // Outward-facing triangles, indices into the corner numbering above
    // (bit 0 = +x, bit 1 = +y, bit 2 = +z).
    let tris: [[usize; 3]; 12] = [
        [0, 2, 3],
        [0, 3, 1], // -z
        [4, 5, 7],
        [4, 7, 6], // +z
        [0, 1, 5],
        [0, 5, 4], // -y
        [2, 6, 7],
        [2, 7, 3], // +y
        [0, 4, 6],
        [0, 6, 2], // -x
        [1, 3, 7],
        [1, 7, 5], // +x
    ];

    // One Edge per undirected corner pair, shared by the two triangles that
    // meet along it — otherwise every edge is a free edge and the shell is not
    // a shell.
    let mut pool: std::collections::HashMap<(usize, usize), remus_topology::edge::EdgeId> =
        std::collections::HashMap::new();
    let mut faces = Vec::with_capacity(12);
    for t in tris {
        let (p0, p1, p2) = (corner(t[0]), corner(t[1]), corner(t[2]));
        let normal = (p1 - p0).cross(p2 - p0).normalize().unwrap();
        let d = normal.dot(Vec3::new(p0.x(), p0.y(), p0.z()));
        let mut oriented = Vec::with_capacity(3);
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            let key = if a < b { (a, b) } else { (b, a) };
            if let Some(&eid) = pool.get(&key) {
                oriented.push(OrientedEdge::new(eid, key.0 == a));
            } else {
                let eid = topo.add_edge(Edge::new(v[key.0], v[key.1], EdgeCurve::Line));
                pool.insert(key, eid);
                oriented.push(OrientedEdge::new(eid, key.0 == a));
            }
        }
        let wire = topo.add_wire(Wire::new(oriented, true).unwrap());
        let mut face = Face::new(wire, vec![], FaceSurface::Plane { normal, d });
        face.set_reversed(reversed);
        faces.push(topo.add_face(face));
    }
    topo.add_shell(Shell::new(faces).unwrap())
}

/// A hollow body whose every face is a planar TRIANGLE — what a mesh import
/// looks like. This is the only shape that reaches `center_of_mass_from_faces`;
/// the boolean-built bodies above are quads and fall through to the tessellated
/// path, so both routes need their own model.
fn triangulated_cavity(topo: &mut Topology, outer: Brick, void: Brick) -> SolidId {
    let outer_shell = triangulated_brick_shell(topo, outer, false);
    let cavity = triangulated_brick_shell(topo, void, true);
    topo.add_solid(Solid::new(outer_shell, vec![cavity]))
}

// ── Cases ─────────────────────────────────────────────────────

/// `(name, outer, void, whether the body must come back as two shells)`.
///
/// The offsets are chosen so no void face is coplanar with an outer wall (that
/// would open the cavity and make it one shell) and so the composite centroid
/// moves by a readable fraction of the body — the 20 x 20 x 20 with a 6 x 6 x 6
/// void at one corner shifts by about 0.44 in x, y and z.
const CASES: [(&str, Brick, Brick); 4] = [
    (
        "void low in every axis",
        Brick::new([0.0, 0.0, 0.0], [20.0, 20.0, 20.0]),
        Brick::new([2.0, 2.0, 2.0], [6.0, 6.0, 6.0]),
    ),
    (
        "void high in x only",
        Brick::new([0.0, 0.0, 0.0], [20.0, 20.0, 20.0]),
        Brick::new([12.0, 7.0, 7.0], [6.0, 6.0, 6.0]),
    ),
    (
        "slab with an off-centre slot",
        Brick::new([0.0, 0.0, 0.0], [30.0, 20.0, 10.0]),
        Brick::new([3.0, 3.0, 3.0], [12.0, 6.0, 4.0]),
    ),
    (
        "CONTROL concentric void",
        Brick::new([0.0, 0.0, 0.0], [20.0, 20.0, 20.0]),
        Brick::new([7.0, 7.0, 7.0], [6.0, 6.0, 6.0]),
    ),
];

/// The centroid must move when material is removed off-centre.
///
/// Asserted as a fraction of the body's own extent, so the statement is the
/// same at every scale: no absolute millimetre anywhere.
#[test]
fn a_cavity_moves_the_centre_of_mass() {
    for k in SCALES {
        for (name, outer, void) in CASES {
            let (outer, void) = (outer.scaled(k), void.scaled(k));
            let mut topo = Topology::new();
            let solid = cut_cavity(&mut topo, outer, void);
            let what = format!("{name} at {k}x");

            let shells = 1 + topo.solid(solid).unwrap().inner_shells().len();
            assert_eq!(
                shells, 2,
                "{what}: expected a sealed cavity (2 shells), got {shells} — the model \
                 this test measures is not the model it means to measure"
            );
            assert_closed_solid(&topo, solid, &what);

            let want = composite_centroid(outer, void);
            let got = solid_center_of_mass(&topo, solid, 1e-2 * k).unwrap();
            assert_centroid(got, want, outer, &what, "solid_center_of_mass");

            // The volume this body reports has been right since #61. Asserted
            // alongside so the pairing is on the record: right volume, wrong
            // centroid was the whole hazard.
            let want_v = outer.volume() - void.volume();
            let got_v = solid_volume(&topo, solid, 1e-2 * k).unwrap();
            let rel = (got_v - want_v).abs() / want_v;
            assert!(
                rel < 1e-12,
                "{what}: volume {got_v} against the closed form {want_v} ({rel:e})"
            );
        }
    }
}

/// The same statement for the all-planar-triangle fast path, which a body has
/// to be a mesh import to reach.
#[test]
fn a_cavity_moves_the_centre_of_mass_of_a_triangulated_body() {
    for k in SCALES {
        for (name, outer, void) in CASES {
            let (outer, void) = (outer.scaled(k), void.scaled(k));
            let mut topo = Topology::new();
            let solid = triangulated_cavity(&mut topo, outer, void);
            let what = format!("triangulated {name} at {k}x");

            assert_closed_solid(&topo, solid, &what);

            let want = composite_centroid(outer, void);
            let got = solid_center_of_mass(&topo, solid, 1e-2 * k).unwrap();
            assert_centroid(got, want, outer, &what, "center_of_mass_from_faces");

            // Same body through the volume fast path. Enumerating the cavity
            // shell is not enough there either: its faces are stored reversed,
            // and without honouring that the void ADDS.
            let want_v = outer.volume() - void.volume();
            let got_v = solid_volume(&topo, solid, 1e-2 * k).unwrap();
            let rel = (got_v - want_v).abs() / want_v;
            assert!(
                rel < 1e-12,
                "{what}: volume {got_v} against the closed form {want_v} ({rel:e})"
            );
        }
    }
}

/// Compare a centroid against its closed form, relative to the body's own
/// diagonal — the only length in the model that is not an arbitrary constant.
fn assert_centroid(got: Point3, want: Point3, outer: Brick, what: &str, route: &str) {
    let extent = (outer.size[0] * outer.size[0]
        + outer.size[1] * outer.size[1]
        + outer.size[2] * outer.size[2])
        .sqrt();
    let err = (got - want).length() / extent;
    assert!(
        err < 1e-12,
        "{what}: {route} put the centroid at ({}, {}, {}) against the composite \
         closed form ({}, {}, {}) — {err:e} of the body's own diagonal",
        got.x(),
        got.y(),
        got.z(),
        want.x(),
        want.y(),
        want.z(),
    );
}

/// Every face of the solid — outer shell AND cavity shells — walked edge by
/// edge: each edge must be used exactly twice, once in each direction. That is
/// closed (no edge used once) and 2-manifold (none used three or more times),
/// stated directly on the topology.
///
/// Not `validate::validate_solid`: its Euler check reads `V - E + F = 2 + L`
/// over the whole solid and takes no account of inner shells, so it calls every
/// hollow body invalid (`V-E+F = 4` for two genus-0 shells, which is correct
/// for a body with one cavity). That is a separate, pre-existing gap; it is not
/// what this test is measuring, and asserting through it would mean asserting
/// the gap.
fn assert_closed_solid(topo: &Topology, solid: SolidId, what: &str) {
    use std::collections::HashMap;

    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    let mut uses: HashMap<usize, (usize, usize)> = HashMap::new();
    for fid in faces {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                let slot = uses.entry(oe.edge().index()).or_insert((0, 0));
                if oe.is_forward() {
                    slot.0 += 1;
                } else {
                    slot.1 += 1;
                }
            }
        }
    }
    let free = uses.values().filter(|&&(f, r)| f + r == 1).count();
    let non_manifold = uses.values().filter(|&&(f, r)| f + r > 2).count();
    let mis_wound = uses
        .values()
        .filter(|&&(f, r)| f + r == 2 && f != 1)
        .count();
    assert!(
        free == 0 && non_manifold == 0 && mis_wound == 0,
        "{what}: {free} free edge(s), {non_manifold} non-manifold edge(s), \
         {mis_wound} edge(s) not used once in each direction"
    );
}
