//! Display meshes of an exact in-place helical sweep.
//!
//! Each side face of the coil is one NURBS patch over every turn: `u` runs
//! across the profile edge (π long on a unit half circle) and `v` along the
//! coil (2π · 10 per turn about an axis 10 away), both over `[0, 1]`. Squared
//! up in parameter units, the CDT chart compressed the surface's `v` by
//! about 20 per turn, and Delaunay in that chart fanned each cap sample to
//! the interior-grid column beside it: slivers a few thousandths wide and up
//! to two units long within 0.03 turns of either cap, whose facets fold
//! inside out (988 on five turns at display deflection).

use std::f64::consts::TAU;

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::{make_circle_edge, make_polygon_wire};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::wire::{OrientedEdge, Wire};

use crate::helix::helical_sweep_in_place;

use super::super::{is_watertight, tessellate_solid_grouped_with_tolerance};

const AXIS_X: f64 = 10.0;
const PITCH: f64 = 2.5;
/// The display tolerances an app gives this coil (2e-4 of its 22-unit
/// extent, and the display angular limit).
const DEFLECTION: f64 = 0.0044;
const ANGULAR: f64 = 0.06;

/// A unit circle (or a 2 × 2 square) about the origin in the XY plane, swept
/// `turns` about the line through `(10, 0, 0)` along `+Y`.
fn coil(topo: &mut Topology, square: bool, turns: f64) -> SolidId {
    let normal = Vec3::new(0.0, 0.0, 1.0);
    let wire = if square {
        let corners = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
            .map(|(x, y)| Point3::new(x, y, 0.0));
        make_polygon_wire(topo, &corners, 1e-7).unwrap()
    } else {
        let edge = make_circle_edge(topo, Point3::new(0.0, 0.0, 0.0), normal, 1.0, 1e-7).unwrap();
        topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap())
    };
    let profile = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane { normal, d: 0.0 },
    ));
    helical_sweep_in_place(
        topo,
        profile,
        Point3::new(AXIS_X, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        PITCH,
        turns,
    )
    .unwrap()
}

/// The profile centre carried to the turn angle of `p` (the profile plane
/// contains the axis, so a surface point's angle about the axis is its
/// sweep angle).
fn tube_centre(p: Point3, turns: f64) -> Point3 {
    let around = p.z().atan2(AXIS_X - p.x());
    let k = ((p.y() * TAU / PITCH - around) / TAU).round();
    let phi = TAU.mul_add(k, around).clamp(0.0, TAU * turns);
    Point3::new(
        AXIS_X.mul_add(-phi.cos(), AXIS_X),
        PITCH * phi / TAU,
        AXIS_X * phi.sin(),
    )
}

struct SideMesh {
    triangles: usize,
    side_triangles: usize,
    /// Side facets whose normal points into the tube.
    inward: usize,
    /// Side facets of quality `4√3·area / Σ edge²` under 0.05.
    slivers: usize,
    longest_side_edge: f64,
    watertight: bool,
}

fn side_mesh(square: bool, turns: f64) -> SideMesh {
    let mut topo = Topology::new();
    let solid = coil(&mut topo, square, turns);
    let faces = solid_faces(&topo, solid).unwrap();
    let (mesh, offsets) =
        tessellate_solid_grouped_with_tolerance(&topo, solid, DEFLECTION, ANGULAR).unwrap();
    let mut out = SideMesh {
        triangles: mesh.indices.len() / 3,
        side_triangles: 0,
        inward: 0,
        slivers: 0,
        longest_side_edge: 0.0,
        watertight: is_watertight(&mesh),
    };
    for (index, &face) in faces.iter().enumerate() {
        if !matches!(topo.face(face).unwrap().surface(), FaceSurface::Nurbs(_)) {
            continue;
        }
        let range = offsets[index] as usize..offsets[index + 1] as usize;
        for tri in mesh.indices[range].chunks_exact(3) {
            let [a, b, c] = [0, 1, 2].map(|k| mesh.positions[tri[k] as usize]);
            let facet = (b - a).cross(c - a);
            let edges = [b - a, c - b, a - c].map(Vec3::length);
            let squares: f64 = edges.iter().map(|e| e * e).sum();
            out.side_triangles += 1;
            out.longest_side_edge = edges.iter().copied().fold(out.longest_side_edge, f64::max);
            if 2.0 * 3.0_f64.sqrt() * facet.length() < 0.05 * squares {
                out.slivers += 1;
            }
            let centroid = Point3::new(
                (a.x() + b.x() + c.x()) / 3.0,
                (a.y() + b.y() + c.y()) / 3.0,
                (a.z() + b.z() + c.z()) / 3.0,
            );
            if facet.dot(centroid - tube_centre(centroid, turns)) < 0.0 {
                out.inward += 1;
            }
        }
    }
    out
}

#[test]
fn round_coil_side_facets_never_fold_into_the_tube() {
    // Before the metric chart: 0, 244 and 986 inward facets, 188, 1088 and
    // 2966 slivers, longest side edge 0.66, 1.76 and 3.33.
    for turns in [1.0, 2.5, 5.0] {
        let side = side_mesh(false, turns);
        let report = format!(
            "{turns} turns: {} triangles ({} on the sides), {} inward, {} slivers, \
             longest side edge {:.3}",
            side.triangles, side.side_triangles, side.inward, side.slivers, side.longest_side_edge
        );
        assert!(side.watertight, "{report}");
        assert_eq!(side.inward, 0, "{report}");
        assert_eq!(side.slivers, 0, "{report}");
        // Grid cells are 0.06 across the tube by 0.21 along it.
        assert!(side.longest_side_edge < 0.5, "{report}");
    }
}

#[test]
fn square_coil_side_facets_stay_outward() {
    // Ruled sides with straight caps: no cap sample sits beside a grid
    // column, so this mesh was already clean; it must stay so.
    let side = side_mesh(true, 5.0);
    let report = format!(
        "{} triangles, {} inward, {} slivers",
        side.triangles, side.inward, side.slivers
    );
    assert!(side.watertight, "{report}");
    assert_eq!(side.inward, 0, "{report}");
    assert_eq!(side.slivers, 0, "{report}");
}
