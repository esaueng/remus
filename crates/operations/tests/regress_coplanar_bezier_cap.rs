//! Issue #953: exact sketch-on-face booleans with a Bezier prism.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::extrude::extrude;
use remus_operations::primitives::make_box;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

fn oriented_prism(
    topo: &mut Topology,
    z: f64,
    height: f64,
    curved: bool,
    reverse_spline: bool,
) -> remus_topology::solid::SolidId {
    let points = [
        Point3::new(10.0, 10.0, z),
        Point3::new(22.0, 10.0, z),
        Point3::new(22.0, 16.0, z),
        Point3::new(10.0, 16.0, z),
    ];
    let vertices = points.map(|p| topo.add_vertex(Vertex::new(p, 1e-7)));
    let edges = (0..4)
        .map(|i| {
            let curve = if i == 0 && curved {
                EdgeCurve::NurbsCurve(
                    NurbsCurve::new(
                        2,
                        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                        vec![points[0], Point3::new(16.0, 4.0, z), points[1]],
                        vec![1.0; 3],
                    )
                    .unwrap(),
                )
            } else {
                EdgeCurve::Line
            };
            let reversed = reverse_spline && i == 0 && curved;
            let (start, end, curve) = if reversed {
                let EdgeCurve::NurbsCurve(c) = curve else {
                    unreachable!()
                };
                (
                    vertices[1],
                    vertices[0],
                    EdgeCurve::NurbsCurve(c.reversed()),
                )
            } else {
                (vertices[i], vertices[(i + 1) % 4], curve)
            };
            let mut edge = Edge::new(start, end, curve);
            edge.set_trim(Some((0.0, 1.0)));
            OrientedEdge::new(topo.add_edge(edge), !reversed)
        })
        .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    let face = remus_topology::builder::make_planar_face_from_wire(topo, wire).unwrap();
    extrude(
        topo,
        face,
        Vec3::new(0.0, 0.0, height.signum()),
        height.abs(),
    )
    .unwrap()
}

fn case(op: BooleanOp, z: f64, height: f64, curved: bool) {
    oriented_case(op, z, height, curved, false);
}

fn oriented_case(op: BooleanOp, z: f64, height: f64, curved: bool, reverse_spline: bool) {
    let _ = env_logger::try_init();
    let mut topo = Topology::new();
    let slab = make_box(&mut topo, 62.0, 50.0, 10.0).unwrap();
    let tool = oriented_prism(&mut topo, z, height, curved, reverse_spline);
    let result = boolean(&mut topo, op, slab, tool);
    assert!(
        result.is_ok(),
        "{op:?}, z={z}, h={height}, curved={curved}: {result:?}"
    );
    let result = result.unwrap();
    let area = if curved { 96.0 } else { 72.0 };
    let expected = match op {
        BooleanOp::Cut => 31000.0 - area * (10.0 - (z + height)),
        BooleanOp::Fuse => 31000.0 + area * (z + height - 10.0),
        BooleanOp::Intersect => unreachable!(),
    };
    let volume = remus_check::properties::solid_volume(
        &topo,
        result,
        &remus_check::properties::PropertiesOptions::default(),
    )
    .unwrap();
    assert!(
        (volume - expected).abs() < 0.03,
        "volume {volume}, expected {expected}"
    );
    let adj = remus_topology::adjacency::AdjacencyIndex::build(&topo, result).unwrap();
    assert!(adj.is_manifold(), "free edges: {:?}", adj.boundary_edges());
    if curved {
        let faces = remus_topology::explorer::solid_faces(&topo, result).unwrap();
        assert!(faces.iter().any(|&f| matches!(
            topo.face(f).unwrap().surface(),
            remus_topology::face::FaceSurface::Nurbs(_)
        )));
        let probe_z = if op == BooleanOp::Cut { 9.0 } else { 11.0 };
        let expected_class = if op == BooleanOp::Cut {
            remus_check::classify::PointClassification::Outside
        } else {
            remus_check::classify::PointClassification::Inside
        };
        assert_eq!(
            remus_check::classify::classify_point(
                &topo,
                result,
                Point3::new(16.0, 8.0, probe_z),
                &remus_check::classify::ClassifyOptions::default()
            )
            .unwrap(),
            expected_class
        );
    }
}

#[test]
fn coplanar_reversed_bezier_cut() {
    oriented_case(BooleanOp::Cut, 10.0, -2.0, true, true);
}
#[test]
fn coplanar_reversed_bezier_fuse() {
    oriented_case(BooleanOp::Fuse, 10.0, 2.0, true, true);
}
#[test]
fn coplanar_bezier_cut() {
    case(BooleanOp::Cut, 10.0, -2.0, true);
}
#[test]
fn coplanar_bezier_fuse() {
    case(BooleanOp::Fuse, 10.0, 2.0, true);
}
#[test]
fn offset_bezier_cut_control() {
    case(BooleanOp::Cut, 10.01, -2.01, true);
}
#[test]
fn offset_bezier_fuse_control() {
    case(BooleanOp::Fuse, 9.5, 2.5, true);
}
#[test]
fn coplanar_polyline_cut_control() {
    case(BooleanOp::Cut, 10.0, -2.0, false);
}
#[test]
fn coplanar_polyline_fuse_control() {
    case(BooleanOp::Fuse, 10.0, 2.0, false);
}

// Open Sans Regular outlines at em size 8, at (10, 22), exported as drawing
// commands from the OFL font. No font parser or application adapter is needed.
// Areas come from symbolic integrals of (x*y' - y*x')/2 over the polynomial
// Bezier segments, including the three counters; they are independent of
// Remus's tessellation and property integration.
const GLYPH_AREAS: [f64; 3] = [
    11.100_823_720_296_23,
    7.022_502_263_387_043,
    7.090_915_044_148_765,
];

fn glyph_face(
    topo: &mut Topology,
    commands: &serde_json::Value,
    z: f64,
    scale: f64,
    flattened: bool,
) -> remus_topology::face::FaceId {
    let coordinate = |c: &serde_json::Value, x: &str, y: &str| {
        Point3::new(
            10.0 + (c[x].as_f64().unwrap() - 10.0) * scale,
            22.0 + (c[y].as_f64().unwrap() - 22.0) * scale,
            z,
        )
    };
    let mut loops = Vec::new();
    let mut edges = Vec::new();
    let mut vertices = Vec::new();
    let mut positions = Vec::new();
    let mut start = Point3::new(0.0, 0.0, z);
    let mut current = start;
    let mut signed_area = 0.0;
    for c in commands.as_array().unwrap() {
        let kind = c["type"].as_str().unwrap();
        if kind == "M" {
            start = coordinate(c, "x", "y");
            current = start;
            vertices.clear();
            positions.clear();
            vertices.push(topo.add_vertex(Vertex::new(start, 1e-7)));
            positions.push(start);
            continue;
        }
        let end = if kind == "Z" {
            start
        } else {
            coordinate(c, "x", "y")
        };
        let mut controls = vec![current];
        if kind == "Q" || kind == "C" {
            controls.push(coordinate(c, "x1", "y1"));
        }
        if kind == "C" {
            controls.push(coordinate(c, "x2", "y2"));
        }
        controls.push(end);
        if (end - current).length() > 1e-10 {
            let degree = controls.len() - 1;
            let spline = if degree > 1 {
                let knots = [vec![0.0; degree + 1], vec![1.0; degree + 1]].concat();
                Some(
                    NurbsCurve::new(degree, knots, controls.clone(), vec![1.0; degree + 1])
                        .unwrap(),
                )
            } else {
                None
            };
            let steps = if flattened && degree > 1 { 16 } else { 1 };
            for step in 1..=steps {
                let p = if step == steps {
                    end
                } else {
                    spline
                        .as_ref()
                        .unwrap()
                        .evaluate(f64::from(step) / f64::from(steps))
                };
                let a = *vertices.last().unwrap();
                let b = if (p - start).length() < 1e-10 {
                    vertices[0]
                } else {
                    topo.add_vertex(Vertex::new(p, 1e-7))
                };
                let curve = if flattened {
                    EdgeCurve::Line
                } else {
                    spline
                        .clone()
                        .map_or(EdgeCurve::Line, EdgeCurve::NurbsCurve)
                };
                let mut edge = Edge::new(a, b, curve);
                edge.set_trim(Some((0.0, 1.0)));
                edges.push(OrientedEdge::new(topo.add_edge(edge), true));
                let prev = *positions.last().unwrap();
                signed_area += prev.x() * p.y() - p.x() * prev.y();
                positions.push(p);
                vertices.push(b);
            }
        }
        current = end;
        if kind == "Z" {
            loops.push((
                signed_area.abs(),
                topo.add_wire(Wire::new(std::mem::take(&mut edges), true).unwrap()),
            ));
            signed_area = 0.0;
        }
    }
    loops.sort_by(|a, b| b.0.total_cmp(&a.0));
    let outer = loops.remove(0).1;
    topo.add_face(remus_topology::face::Face::new(
        outer,
        loops.iter().map(|l| l.1).collect(),
        remus_topology::face::FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: z,
        },
    ))
}

fn boa_case(op: BooleanOp, z: f64, height: f64, scale: f64, flattened: bool) {
    let _ = env_logger::try_init();
    let commands: serde_json::Value =
        serde_json::from_str(include_str!("data/issue_953_boa.json")).unwrap();
    let mut topo = Topology::new();
    let mut slab = make_box(&mut topo, 62.0, 50.0, 10.0).unwrap();
    for (i, glyph) in commands.as_array().unwrap().iter().enumerate() {
        let face = glyph_face(&mut topo, glyph, z, scale, flattened);
        let tool = extrude(
            &mut topo,
            face,
            Vec3::new(0.0, 0.0, height.signum()),
            height.abs(),
        )
        .unwrap();
        let result = boolean(&mut topo, op, slab, tool);
        assert!(
            result.is_ok(),
            "glyph {i}: {op:?}, z={z}, height={height}: {result:?}"
        );
        slab = result.unwrap();

        let adj = remus_topology::adjacency::AdjacencyIndex::build(&topo, slab).unwrap();
        assert!(
            adj.is_manifold(),
            "glyph {i}: free {:?}, non-manifold {:?}",
            adj.boundary_edges(),
            adj.non_manifold_edges()
        );
        let top = if op == BooleanOp::Cut {
            10.0
        } else {
            z + height
        };
        for vertex in remus_topology::explorer::solid_vertices(&topo, slab).unwrap() {
            assert!(
                topo.vertex(vertex).unwrap().point().z() <= top + 1e-7,
                "glyph {i}: result extends above its exact top plane {top}"
            );
        }
    }
    let depth = if op == BooleanOp::Cut {
        z.min(10.0) - (z + height)
    } else {
        z + height - 10.0
    };
    let sign = if op == BooleanOp::Cut { -1.0 } else { 1.0 };
    let expected = 31000.0 + sign * GLYPH_AREAS.iter().sum::<f64>() * scale * scale * depth;
    let volume = remus_operations::measure::solid_volume(&topo, slab, 0.001).unwrap();
    // Display/property integration samples curved cap outlines. The B-Rep
    // remains exact; bound the numerical measurement separately from that.
    assert!(
        (volume - expected).abs() < 0.1 * scale * scale,
        "volume {volume}, expected {expected}"
    );
}

#[test]
fn boa_coplanar_bezier_cut() {
    boa_case(BooleanOp::Cut, 10.0, -2.0, 1.0, false);
}
#[test]
fn boa_coplanar_bezier_deep_cut() {
    boa_case(BooleanOp::Cut, 10.0, -5.0, 1.0, false);
}
#[test]
fn boa_coplanar_bezier_fuse() {
    boa_case(BooleanOp::Fuse, 10.0, 2.0, 1.0, false);
}
#[test]
fn boa_offset_cut_control() {
    boa_case(BooleanOp::Cut, 10.01, -2.01, 1.0, false);
}
#[test]
fn boa_offset_fuse_control() {
    boa_case(BooleanOp::Fuse, 9.5, 2.5, 1.0, false);
}
#[test]
fn boa_sunk_cut_control() {
    boa_case(BooleanOp::Cut, 9.5, -1.5, 1.0, false);
}
#[test]
fn boa_polyline_cut_control() {
    boa_case(BooleanOp::Cut, 10.0, -2.0, 1.0, true);
}
#[test]
fn boa_large_bezier_cut() {
    boa_case(BooleanOp::Cut, 10.0, -2.0, 2.5, false);
}

#[test]
fn one_micron_wall_band_preserves_operands() {
    let _ = env_logger::try_init();
    let commands: serde_json::Value =
        serde_json::from_str(include_str!("data/issue_953_boa.json")).unwrap();
    let mut topo = Topology::new();
    let slab = make_box(&mut topo, 62.0, 50.0, 10.0).unwrap();
    let face = glyph_face(&mut topo, &commands[0], 10.001, 1.0, false);
    let tool = extrude(&mut topo, face, Vec3::new(0.0, 0.0, -1.0), 2.001).unwrap();
    let before = remus_io::arena_io::serialize_solids(&topo, &[slab, tool]).unwrap();
    let result = boolean(&mut topo, BooleanOp::Cut, slab, tool).unwrap();
    let adj = remus_topology::adjacency::AdjacencyIndex::build(&topo, result).unwrap();
    assert!(adj.is_manifold());
    let after = remus_io::arena_io::serialize_solids(&topo, &[slab, tool]).unwrap();
    assert_eq!(before, after, "boolean must preserve operands");
}

#[test]
fn boa_one_micron_bezier_cut() {
    boa_case(BooleanOp::Cut, 10.001, -2.001, 1.0, false);
}

#[test]
fn boa_one_micron_bezier_fuse() {
    boa_case(BooleanOp::Fuse, 9.999, 2.001, 1.0, false);
}

#[test]
fn boa_large_one_micron_bezier_deep_cut() {
    boa_case(BooleanOp::Cut, 10.001, -5.001, 2.5, false);
}

#[test]
fn boa_small_offset_bezier_cuts() {
    for offset in [0.0001, 0.0005, 0.002] {
        boa_case(BooleanOp::Cut, 10.0 + offset, -2.0 - offset, 1.0, false);
    }
}

#[test]
fn boa_polyline_fuse_control() {
    boa_case(BooleanOp::Fuse, 10.0, 2.0, 1.0, true);
}
