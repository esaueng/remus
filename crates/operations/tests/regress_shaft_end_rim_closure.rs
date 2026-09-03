//! Regression: an end cap and the wall it meets must share the rim between
//! them, on a body whose wall was pushed off the structured band path.
//!
//! `tessellate_solid` tessellates each edge ONCE and hands the same polyline to
//! every face that touches it, so neighbouring faces meet on identical
//! vertices. Two faces bypass that pool:
//!
//! * A cylindrical/conical band whose outer wire is two closed rims and two
//!   seam lines is stitched straight from the shared rim vertices
//!   (`tessellate_revolution_band_shared`) — watertight by construction.
//! * A band the same shape but carrying INNER WIRES is declined by that path
//!   (it would skin the holes over) and falls through to
//!   `tessellate_nonplanar_snap`, which tessellates the face from its own
//!   analytic grid and then reconciles with the shared pool by 1 µm proximity.
//!
//! The second one only closes while the grid and the pool agree on WHERE a full
//! turn starts. They stopped agreeing in remus#64: that PR moved a closed
//! rim's polyline to begin at the edge's own seam vertex (right, and it must
//! stay), while `compute_angular_range` kept anchoring the grid at the SURFACE
//! FRAME's `u = 0`, which after a boolean is unrelated to the seam. On this
//! body the two are 2.3077° apart at r = 3 — 0.121 mm, five orders of magnitude
//! past the 1 µm snap tolerance — so not one grid column snapped, the wall and
//! its caps ended up sharing no rim vertex at all, and every rim segment on
//! both sides became a boundary edge.
//!
//! The signature is what identifies it: a CONSTANT number of extra open edges,
//! the same at every bore radius, because it is the SHAFT's rims that come
//! apart and the bore never touches them. Measured through
//! `welded_mesh_quality` at deflection 0.01 before the hole-aware CDT, one
//! cylindrical face's worth either way:
//!
//! | bore r | open edges before #64 | with #64 | with the rim fix |
//! |--------|-----------------------|----------|------------------|
//! |   3    |         1526          |   1682   |       1526       |
//! |   2    |          264          |    420   |        264       |
//! |   1    |          206          |    362   |        206       |
//!
//! The dedicated cylindrical CDT now retains the shaft wall's inner wires. A
//! separate approximation error remains on a through-bore wall whose
//! non-standard OUTER boundary wraps the cylinder period (the ignored unit test
//! `a_through_bore_wall_is_drawn_at_its_true_area` pins it). That residue is why
//! `end_rim_open_edges` below still isolates the two end rims in the broad sweep.
//!
//! Every expectation here is a closed form: an end rim is a circle of radius R
//! at z = 0 and z = H, it is shared by exactly two faces, and a shared rim has
//! ZERO one-sided edges on it — no tolerance, no comparison against another
//! integrator.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::{BTreeSet, HashMap};
use std::f64::consts::FRAC_PI_2;

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_cylinder;
use remus_operations::tessellate::{
    TriangleMesh, tessellate_solid_grouped_with_tolerance, welded_mesh_quality,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

/// Shaft radius and height at unit scale.
const R: f64 = 3.0;
const H: f64 = 30.0;

/// Scales in a rotated order, so a result that only holds at whichever one runs
/// first is visible rather than hidden behind a lucky first entry.
const SCALES: [f64; 3] = [1000.0, 0.001, 1.0];

/// A shaft of radius `R*s`, height `H*s`, cross-drilled clean through at
/// mid-height by a bore of radius `bore*s` on the +x axis.
fn cross_drilled_shaft(bore: f64, s: f64) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let shaft = make_cylinder(&mut topo, R * s, H * s).unwrap();
    // Long enough to exit both sides, centred on the shaft's axis at H/2.
    let len = (H + 4.0 * R) * s;
    let tool = make_cylinder(&mut topo, bore * s, len).unwrap();
    transform_solid(&mut topo, tool, &Mat4::rotation_y(FRAC_PI_2)).unwrap();
    transform_solid(
        &mut topo,
        tool,
        &Mat4::translation(-len / 2.0, 0.0, H * s / 2.0),
    )
    .unwrap();
    let res = boolean(&mut topo, BooleanOp::Cut, shaft, tool).unwrap();
    (topo, res)
}

/// 1 µm position weld — the same grid `welded_mesh_quality` uses, so a
/// duplicate vertex at a coincident position cannot fake a hole or hide one.
type Q = (i64, i64, i64);
fn weld(p: Point3, s: f64) -> Q {
    let g = 1e6 / s;
    #[allow(clippy::cast_possible_truncation)]
    (
        (p.x() * g).round() as i64,
        (p.y() * g).round() as i64,
        (p.z() * g).round() as i64,
    )
}

/// Is this point on one of the shaft's two end rims — the circle of radius
/// `R*s` at `z = 0` and at `z = H*s`?
///
/// The band is generous in radius (a chord midpoint of the rim polyline sits
/// slightly inside) and tight in z, which is exact: an end cap is planar.
fn on_end_rim(p: Point3, s: f64) -> bool {
    let z_ok = p.z().abs() < 1e-9 * s.max(1.0) || (p.z() - H * s).abs() < 1e-6 * s.max(1.0);
    let r = p.x().hypot(p.y());
    z_ok && r > 0.9 * R * s
}

/// Open (one-sided) half-edges of the position-welded mesh that lie wholly on
/// an end rim. A rim shared by its cap and its wall has none.
fn end_rim_open_edges(mesh: &TriangleMesh, s: f64) -> usize {
    let mut he: BTreeSet<(Q, Q)> = BTreeSet::new();
    for t in mesh.indices.chunks_exact(3) {
        let (a, b, c) = (
            weld(mesh.positions[t[0] as usize], s),
            weld(mesh.positions[t[1] as usize], s),
            weld(mesh.positions[t[2] as usize], s),
        );
        if a == b || b == c || a == c {
            continue;
        }
        he.insert((a, b));
        he.insert((b, c));
        he.insert((c, a));
    }
    let rim: BTreeSet<Q> = mesh
        .positions
        .iter()
        .filter(|p| on_end_rim(**p, s))
        .map(|p| weld(*p, s))
        .collect();
    he.iter()
        .filter(|(a, b)| !he.contains(&(*b, *a)) && rim.contains(a) && rim.contains(b))
        .count()
}

/// The whole point: an end rim is shared, so it has no open edge — at every
/// bore radius, deflection and scale.
///
/// Before the fix this failed at every entry in the sweep: the shaft has two
/// end rims and each contributed one full ring of open segments from the cap
/// plus the same ring again from the wall.
///
/// The two coarsest entries assert something the fix also changed and that was
/// worth catching on its own: at `deflection >= 0.3` this body used to
/// tessellate to NOTHING — zero triangles for a valid five-face solid, at every
/// bore radius — and a caller reading `is_watertight` on an empty mesh is told
/// `true`. It now returns the same 68-triangle mesh the next step down does.
#[test]
fn the_shaft_end_rims_carry_no_open_edge() {
    for s in SCALES {
        for bore in [3.0_f64, 2.0, 1.0] {
            let (topo, solid) = cross_drilled_shaft(bore, s);
            for defl in [0.5_f64, 0.3, 0.1, 0.05, 0.02, 0.01] {
                let (mesh, _) =
                    tessellate_solid_grouped_with_tolerance(&topo, solid, defl * s, 0.35).unwrap();
                assert!(
                    !mesh.indices.is_empty(),
                    "scale {s}, bore {bore}, deflection {defl}: no mesh at all"
                );
                let open = end_rim_open_edges(&mesh, s);
                assert_eq!(
                    open, 0,
                    "scale {s}, bore {bore}, deflection {defl}: {open} open edges on the \
                     shaft's end rims — the cap and the wall stopped sharing them"
                );
            }
        }
    }
}

/// The mechanism, stated directly: the end cap and the cylindrical wall must
/// use the SAME rim vertices, not merely land close to each other.
///
/// A count-only check would pass on two rings of equal size half a segment
/// apart, which is exactly the state the defect produced (39 vertices each way
/// at deflection 0.01, offset by a quarter of a 9.2308° step).
#[test]
fn the_cap_and_the_wall_share_one_rim_vertex_ring() {
    let s = 1.0;
    let (topo, solid) = cross_drilled_shaft(2.0, s);
    let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();

    // The shaft wall is the cylinder of radius R about the z axis; the caps are
    // the two planes. Identify them from geometry, not from index order.
    let mut cap_groups = Vec::new();
    let mut wall_group = None;
    for (i, &fid) in faces.iter().enumerate() {
        match topo.face(fid).unwrap().surface() {
            FaceSurface::Plane { .. } => cap_groups.push(i),
            FaceSurface::Cylinder(c) if (c.radius() - R * s).abs() < 1e-9 => {
                assert!(wall_group.is_none(), "more than one shaft wall");
                wall_group = Some(i);
            }
            _ => {}
        }
    }
    assert_eq!(cap_groups.len(), 2, "expected exactly two end caps");
    let wall_group = wall_group.expect("no shaft wall face");

    let (mesh, offsets) =
        tessellate_solid_grouped_with_tolerance(&topo, solid, 0.01 * s, 0.35).unwrap();

    // Rim positions each face group actually references, welded at 1 µm.
    let ring_of = |g: usize| -> BTreeSet<Q> {
        let (lo, hi) = (offsets[g] as usize, offsets[g + 1] as usize);
        mesh.indices[lo..hi]
            .iter()
            .map(|&i| mesh.positions[i as usize])
            .filter(|p| on_end_rim(*p, s))
            .map(|p| weld(p, s))
            .collect()
    };

    let wall_ring = ring_of(wall_group);
    assert!(
        wall_ring.len() >= 3,
        "the wall referenced {} end-rim vertices — the probe is not finding the rim",
        wall_ring.len()
    );
    let mut caps_ring: BTreeSet<Q> = BTreeSet::new();
    for &g in &cap_groups {
        let r = ring_of(g);
        assert!(
            r.len() >= 3,
            "a cap referenced {} rim vertices — the probe is not finding the rim",
            r.len()
        );
        caps_ring.extend(r);
    }
    assert_eq!(
        wall_ring,
        caps_ring,
        "the wall and the caps reference different rim vertex sets \
         ({} vs {}, {} in common)",
        wall_ring.len(),
        caps_ring.len(),
        wall_ring.intersection(&caps_ring).count()
    );
}

/// remus#64's invariant, kept live so the fix above cannot be "simplified"
/// into a revert of it: the rim ring must contain the rim EDGE's own start
/// vertex, and now so must the wall's grid.
///
/// Anchoring the grid anywhere else is what broke; anchoring the polyline
/// anywhere else is what #64 fixed. Both are asserted from the same vertex.
#[test]
fn both_the_rim_polyline_and_the_wall_grid_contain_the_rim_edge_start_vertex() {
    let s = 1.0;
    let (topo, solid) = cross_drilled_shaft(2.0, s);
    let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();

    // Collect every closed circle edge of radius R at an end plane, and take
    // its start vertex — the seam the rim polyline is anchored on.
    let mut seam_pts: Vec<Point3> = Vec::new();
    for &fid in &faces {
        let face = topo.face(fid).unwrap();
        let wire = topo.wire(face.outer_wire()).unwrap();
        for oe in wire.edges() {
            let e = topo.edge(oe.edge()).unwrap();
            if e.start() != e.end() {
                continue;
            }
            if !matches!(e.curve(), remus_topology::edge::EdgeCurve::Circle(_)) {
                continue;
            }
            let p = topo.vertex(e.start()).unwrap().point();
            if on_end_rim(p, s) {
                seam_pts.push(p);
            }
        }
    }
    assert_eq!(
        seam_pts.len(),
        4,
        "expected the two end rims, once from the cap and once from the wall"
    );

    let (mesh, offsets) =
        tessellate_solid_grouped_with_tolerance(&topo, solid, 0.01 * s, 0.35).unwrap();
    let mut used: HashMap<Q, BTreeSet<usize>> = HashMap::new();
    for g in 0..offsets.len() - 1 {
        let (lo, hi) = (offsets[g] as usize, offsets[g + 1] as usize);
        for &i in &mesh.indices[lo..hi] {
            used.entry(weld(mesh.positions[i as usize], s))
                .or_default()
                .insert(g);
        }
    }
    for p in seam_pts {
        let k = weld(p, s);
        let groups = used.get(&k).cloned().unwrap_or_default();
        assert!(
            groups.len() >= 2,
            "the rim's own start vertex {p:?} is referenced by {} face group(s); \
             it must be a mesh vertex of the cap AND of the wall",
            groups.len()
        );
    }
}

/// The body under test is the one the defect was reported on, and it is right
/// where the exact integrator can see it — so a green closure result here is
/// not a green result on some other shape.
///
/// Material left = shaft minus the Steinmetz solid the two equal perpendicular
/// cylinders share, `16 r³/3`, written out rather than compared to a second
/// integrator.
#[test]
fn the_body_is_the_cross_drilled_shaft_it_claims_to_be() {
    let s = 1.0;
    let (topo, solid) = cross_drilled_shaft(R, s);
    let expected = std::f64::consts::PI * R * R * H - 16.0 / 3.0 * R * R * R;
    let v = solid_volume(&topo, solid, 0.01).unwrap();
    assert!(
        (v - expected).abs() <= 1e-4 * expected,
        "expected the closed form {expected:.6}, got {v:.6}"
    );

    // The exact ellipse seam splits the equal-radius wall into seam-free
    // bands. Unequal-radius fixtures in this file retain the holed-wall snap
    // coverage; this assertion pins the canonical exact-seam topology.
    let holed = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&fid| {
            let f = topo.face(fid).unwrap();
            matches!(f.surface(), FaceSurface::Cylinder(c) if (c.radius() - R).abs() < 1e-9)
                && !f.inner_wires().is_empty()
        })
        .count();
    assert_eq!(
        holed, 0,
        "the exact-seam equal-radius cross-drill leaves no holed shaft wall"
    );

    // The hole-aware cylindrical path must retain the bore openings and share
    // their seam-crossing samples with the neighbouring bore walls.
    let (mesh, _) = tessellate_solid_grouped_with_tolerance(&topo, solid, 0.01, 0.35).unwrap();
    let q = welded_mesh_quality(&mesh);
    assert_eq!(
        end_rim_open_edges(&mesh, s),
        0,
        "the shaft's end rims must be closed"
    );
    assert!(
        q.is_watertight(),
        "the cross-drilled shaft must be watertight: boundary={} non-manifold={}",
        q.boundary_edges,
        q.non_manifold_edges
    );
}
