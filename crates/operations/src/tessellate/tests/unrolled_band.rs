//! The unrolled chart of a short, wide cylinder band (see `unrolled_radius`
//! in `tessellate_nonplanar_cdt`) is only for bands it helps: a modeled
//! thread's lands, about forty times wider than tall. These pin what it must
//! leave alone: a cross-drilled bore wall, about as wide as long, keeps its
//! squared-box mesh; and a near-flat band of a huge radius, whose sliver-high
//! side or waist the unrolled chart would squeeze under the CDT's duplicate-point
//! tolerance, keeps every face of its solid.

use std::collections::BTreeMap;
use std::f64::consts::FRAC_PI_2;

use remus_math::mat::Mat4;
use remus_math::nurbs::NurbsCurve;
use remus_math::surfaces::CylindricalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::Solid;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use crate::boolean::{BooleanOp, boolean};
use crate::primitives::make_cylinder;
use crate::tessellate::tessellate_solid_grouped_with_tolerance;
use crate::transform::transform_solid;

/// Per-face triangle counts of a radius-3, 30 long shaft cross-drilled at
/// mid-length by a bore of `bore` radius, at deflection 0.05.
fn drilled_shaft_face_triangles(bore: f64) -> Vec<u32> {
    let mut topo = Topology::new();
    let shaft = make_cylinder(&mut topo, 3.0, 30.0).unwrap();
    let tool = make_cylinder(&mut topo, bore, 42.0).unwrap();
    transform_solid(&mut topo, tool, &Mat4::rotation_y(FRAC_PI_2)).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(-21.0, 0.0, 15.0)).unwrap();
    let drilled = boolean(&mut topo, BooleanOp::Cut, shaft, tool).unwrap();
    let (_, offsets) = tessellate_solid_grouped_with_tolerance(&topo, drilled, 0.05, 0.06).unwrap();
    offsets
        .windows(2)
        .map(|pair| (pair[1] - pair[0]) / 3)
        .collect()
}

/// The bore walls (the last face) meshed 21 703 and 19 753 triangles on
/// `main` (aarch64 macOS) with the squared box's dense trim rows; unrolled,
/// the radius-2 wall dropped to 9 945. A floor near the squared-box counts
/// keeps them on that chart without pinning platform-dependent samples.
#[test]
fn cross_drilled_bore_walls_keep_the_squared_chart() {
    for (bore, squared) in [(1.0, 21_703), (2.0, 19_753)] {
        let faces = drilled_shaft_face_triangles(bore);
        let wall = *faces.last().unwrap();
        assert!(
            f64::from(wall) >= 0.9 * f64::from(squared),
            "bore {bore}: wall {wall} triangles, {faces:?}"
        );
    }
}

/// A closed wedge between cylinders of radius 10 000 and 9 999, 0.1 rad
/// around, whose end faces are planes tilted so one side of the outer band
/// is only 4e-5 high. Its band is 1000 times wider than tall, but unrolled
/// that side's two samples would sit 4e-9 apart in the chart.
#[test]
fn a_huge_radius_band_with_a_sliver_side_keeps_every_face() {
    let origin = Point3::new(0.0, 0.0, 0.0);
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let (a, b) = (0.2, 0.3);
    let r = 10_000.0;
    let cylinders =
        [r, r - 1.0].map(|radius| CylindricalSurface::new(origin, axis, radius).unwrap());
    let tangent = (cylinders[0].evaluate(a + FRAC_PI_2, 0.0) - origin) * (1.0 / r);
    let height = |p: Point3, top: bool| {
        if top {
            0.001f64.mul_add((p - origin).dot(tangent), 4e-5)
        } else {
            0.00002 * (p - origin).dot(tangent)
        }
    };
    let on_plane = |p: Point3, top: bool| p + axis * height(p, top);
    let mut topo = Topology::new();
    let points: [Point3; 8] = std::array::from_fn(|i| {
        let angle = if i % 4 == 0 || i % 4 == 3 { a } else { b };
        on_plane(cylinders[i / 4].evaluate(angle, 0.0), i % 4 >= 2)
    });
    let vertices = points.map(|p| topo.add_vertex(Vertex::new(p, 1e-7)));
    let mut edges = BTreeMap::new();
    let mut faces = Vec::new();
    let loops = [
        [0, 1, 2, 3],
        [4, 7, 6, 5],
        [1, 0, 4, 5],
        [3, 2, 6, 7],
        [0, 3, 7, 4],
        [2, 1, 5, 6],
    ];
    for (index, ring) in loops.into_iter().enumerate() {
        let mut uses = Vec::new();
        for j in 0..4 {
            let (from, to) = (ring[j], ring[(j + 1) % 4]);
            let (low, high) = (from.min(to), from.max(to));
            let edge = *edges.entry((low, high)).or_insert_with(|| {
                let arc = low / 4 == high / 4 && matches!((low % 4, high % 4), (0, 1) | (2, 3));
                let curve = if arc {
                    let c = ((b - a) / 2.0).cos();
                    let middle = cylinders[low / 4].evaluate(f64::midpoint(a, b), 0.0);
                    let control = origin + (middle - origin) * (1.0 / c);
                    EdgeCurve::NurbsCurve(
                        NurbsCurve::new(
                            2,
                            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                            vec![points[low], on_plane(control, low % 4 >= 2), points[high]],
                            vec![1.0, c, 1.0],
                        )
                        .unwrap(),
                    )
                } else {
                    EdgeCurve::Line
                };
                let mut edge = Edge::new(vertices[low], vertices[high], curve);
                edge.set_trim(Some((0.0, 1.0)));
                topo.add_edge(edge)
            });
            uses.push(OrientedEdge::new(edge, from < to));
        }
        let wire = topo.add_wire(Wire::new(uses, true).unwrap());
        let surface = if index < 2 {
            FaceSurface::Cylinder(cylinders[index].clone())
        } else {
            let p = points[ring[0]];
            let normal = (points[ring[1]] - p)
                .cross(points[ring[2]] - p)
                .normalize()
                .unwrap();
            FaceSurface::Plane {
                normal,
                d: normal.dot(p - origin),
            }
        };
        faces.push(topo.add_face(if index == 1 {
            Face::new_reversed(wire, vec![], surface)
        } else {
            Face::new(wire, vec![], surface)
        }));
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    let solid = topo.add_solid(Solid::new(shell, vec![]));
    let (mesh, offsets) =
        tessellate_solid_grouped_with_tolerance(&topo, solid, 0.05, 0.06).unwrap();
    let per_face: Vec<u32> = offsets
        .windows(2)
        .map(|pair| (pair[1] - pair[0]) / 3)
        .collect();
    assert!(
        per_face.iter().all(|&count| count > 0),
        "a face meshed empty: {per_face:?}"
    );
    let mut incidence = BTreeMap::new();
    for triangle in mesh.indices.chunks_exact(3) {
        for j in 0..3 {
            let (p, q) = (triangle[j], triangle[(j + 1) % 3]);
            *incidence.entry((p.min(q), p.max(q))).or_insert(0) += 1;
        }
    }
    assert!(
        incidence.values().all(|&count| count == 2),
        "an edge not shared by two triangles"
    );
}

/// A closed band between cylinders of radius 10 000 and 9 999, 0.1 rad
/// around, whose rims are split at its middle angle, where its top plane
/// leaves it only 4e-5 high. The rim samples there are not neighbours along
/// the boundary, but unrolled they would sit 4e-9 apart and merge in the
/// CDT, pinching the band's boundary into two loops.
fn waisted_band_mesh(slope: f64) -> (Vec<u32>, usize, [usize; 2]) {
    let origin = Point3::new(0.0, 0.0, 0.0);
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let r = 10_000.0;
    let angle = [0.2, 0.25, 0.3];
    let cylinders =
        [r, r - 1.0].map(|radius| CylindricalSurface::new(origin, axis, radius).unwrap());
    let radial = (cylinders[0].evaluate(angle[1], 0.0) - origin) * (1.0 / r);
    let on_plane = |p: Point3, top: bool| {
        if top {
            p + axis * slope.mul_add(r - (p - origin).dot(radial), 4e-5)
        } else {
            p
        }
    };
    let mut topo = Topology::new();
    let points: [Point3; 12] =
        std::array::from_fn(|i| on_plane(cylinders[i / 6].evaluate(angle[i % 3], 0.0), i % 6 >= 3));
    let vertices = points.map(|p| topo.add_vertex(Vertex::new(p, 1e-7)));
    let loops: [&[usize]; 6] = [
        &[0, 1, 2, 5, 4, 3],
        &[6, 9, 10, 11, 8, 7],
        &[2, 1, 0, 6, 7, 8],
        &[3, 4, 5, 11, 10, 9],
        &[0, 3, 9, 6],
        &[5, 2, 8, 11],
    ];
    let mut edges = BTreeMap::new();
    let mut faces = Vec::new();
    for (index, ring) in loops.into_iter().enumerate() {
        let mut uses = Vec::new();
        for j in 0..ring.len() {
            let (from, to) = (ring[j], ring[(j + 1) % ring.len()]);
            let (low, high) = (from.min(to), from.max(to));
            let edge = *edges.entry((low, high)).or_insert_with(|| {
                let curve = if low / 3 == high / 3 {
                    let (a, b) = (angle[low % 3], angle[high % 3]);
                    let c = ((b - a) / 2.0).cos();
                    let middle = cylinders[low / 6].evaluate(f64::midpoint(a, b), 0.0);
                    let control = origin + (middle - origin) * (1.0 / c);
                    EdgeCurve::NurbsCurve(
                        NurbsCurve::new(
                            2,
                            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                            vec![points[low], on_plane(control, low % 6 >= 3), points[high]],
                            vec![1.0, c, 1.0],
                        )
                        .unwrap(),
                    )
                } else {
                    EdgeCurve::Line
                };
                let mut edge = Edge::new(vertices[low], vertices[high], curve);
                edge.set_trim(Some((0.0, 1.0)));
                topo.add_edge(edge)
            });
            uses.push(OrientedEdge::new(edge, from < to));
        }
        let wire = topo.add_wire(Wire::new(uses, true).unwrap());
        let surface = if index < 2 {
            FaceSurface::Cylinder(cylinders[index].clone())
        } else {
            let p = points[ring[0]];
            let normal = (points[ring[1]] - p)
                .cross(points[ring[2]] - p)
                .normalize()
                .unwrap();
            FaceSurface::Plane {
                normal,
                d: normal.dot(p - origin),
            }
        };
        faces.push(topo.add_face(if index == 1 {
            Face::new_reversed(wire, vec![], surface)
        } else {
            Face::new(wire, vec![], surface)
        }));
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    let solid = topo.add_solid(Solid::new(shell, vec![]));
    let (mesh, offsets) =
        tessellate_solid_grouped_with_tolerance(&topo, solid, 0.05, 0.06).unwrap();
    let per_face = offsets
        .windows(2)
        .map(|pair| (pair[1] - pair[0]) / 3)
        .collect();
    let mut incidence = BTreeMap::new();
    for triangle in mesh.indices.chunks_exact(3) {
        for j in 0..3 {
            let (p, q) = (triangle[j], triangle[(j + 1) % 3]);
            *incidence.entry((p.min(q), p.max(q))).or_insert(0) += 1;
        }
    }
    let unpaired = incidence.values().filter(|&&count| count != 2).count();
    let outer = &mesh.indices[offsets[0] as usize..offsets[1] as usize];
    let used = |p: Point3| {
        outer
            .iter()
            .filter(|&&index| (mesh.positions[index as usize] - p).length() < 1e-8)
            .count()
    };
    (per_face, unpaired, [used(points[1]), used(points[4])])
}

#[test]
fn a_huge_radius_band_with_a_sliver_waist_keeps_both_rims() {
    for slope in [0.0, 1e-3] {
        let (per_face, unpaired, midpoints) = waisted_band_mesh(slope);
        assert!(
            per_face.iter().all(|&count| count > 0),
            "slope {slope}: {per_face:?}"
        );
        assert_eq!(
            unpaired, 0,
            "slope {slope}: edges not shared by two triangles"
        );
        assert!(
            midpoints.iter().all(|&uses| uses > 0),
            "slope {slope}: rim midpoints {midpoints:?}"
        );
    }
}
