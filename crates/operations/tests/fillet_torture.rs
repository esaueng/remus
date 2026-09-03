//! Deterministic qualification corpus for fillet failure cliffs.
//!
//! Every row has one admissible disposition: a verified watertight solid or a
//! stable typed refusal. A crash, a partial success, an untyped error, a mesh
//! leak, or a plausible-looking result with the wrong volume fails the suite.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::collections::{HashMap, HashSet};
use std::f64::consts::TAU;

use remus_check::validate::{Severity, ValidateOptions, validate_solid};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::blend_ops::{blend_failure_code, fillet_v2};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::extrude::extrude;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_convex_hull, make_cylinder};
use remus_operations::query::filter_filletable_edges;
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::Solid;
use remus_topology::solid::SolidId;
use remus_topology::validation::{validate_shell_closed, validate_shell_manifold};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const TOL: f64 = 1e-8;
const MESH_DEFLECTION: f64 = 0.01;
const MEASURE_DEFLECTION: f64 = 0.005;
const VOLUME_REL_TOL: f64 = 0.03;

#[derive(Clone, Copy)]
enum Expected {
    Built,
    TypedRefusal(&'static str),
}

struct TortureCase {
    name: &'static str,
    expected: Expected,
    run: fn(&str) -> Observed,
}

enum Observed {
    Built {
        topo: Topology,
        input: SolidId,
        output: SolidId,
    },
    TypedRefusal {
        topo: Topology,
        input: SolidId,
        error: OperationsError,
        before: ShapeSnapshot,
    },
}

#[derive(Debug, PartialEq)]
struct ShapeSnapshot {
    counts: (usize, usize, usize),
    volume: f64,
    surfaces: Vec<&'static str>,
}

fn snapshot(topo: &Topology, solid: SolidId) -> ShapeSnapshot {
    let mut surfaces: Vec<_> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(|face| topo.face(face).unwrap().surface().type_tag())
        .collect();
    surfaces.sort_unstable();
    ShapeSnapshot {
        counts: solid_entity_counts(topo, solid).unwrap(),
        volume: solid_volume(topo, solid, MEASURE_DEFLECTION).unwrap(),
        surfaces,
    }
}

fn run_case(case: &TortureCase) {
    let observed = (case.run)(case.name);
    match (case.expected, observed) {
        (
            Expected::Built,
            Observed::Built {
                topo,
                input,
                output,
            },
        ) => assert_built(&topo, input, output, case.name),
        (
            Expected::TypedRefusal(expected_code),
            Observed::TypedRefusal {
                topo,
                input,
                error,
                before,
            },
        ) => {
            assert_eq!(
                blend_failure_code(&error),
                expected_code,
                "{}: wrong refusal for {error}",
                case.name
            );
            assert_unchanged(&topo, input, &before, case.name);
        }
        (Expected::Built, Observed::TypedRefusal { error, .. }) => {
            panic!("{}: expected Built, got {error}", case.name);
        }
        (Expected::TypedRefusal(code), Observed::Built { .. }) => {
            panic!("{}: expected TypedRefusal({code}), got Built", case.name);
        }
    }
}

fn assert_unchanged(topo: &Topology, input: SolidId, before: &ShapeSnapshot, name: &str) {
    let after = snapshot(topo, input);
    assert_eq!(
        after.counts, before.counts,
        "{name}: refusal changed topology"
    );
    assert_eq!(
        after.surfaces, before.surfaces,
        "{name}: refusal changed support surfaces"
    );
    assert!(
        (after.volume - before.volume).abs() < 1e-9,
        "{name}: refusal changed input volume from {} to {}",
        before.volume,
        after.volume
    );
}

fn validation_errors(topo: &Topology, solid: SolidId) -> HashMap<String, usize> {
    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    let mut errors = HashMap::new();
    for issue in report
        .issues
        .iter()
        .filter(|issue| issue.severity == Severity::Error)
    {
        *errors.entry(format!("{:?}", issue.check)).or_insert(0) += 1;
    }
    errors
}

fn assert_no_validation_regression(topo: &Topology, input: SolidId, output: SolidId, name: &str) {
    let before = validation_errors(topo, input);
    let after = validation_errors(topo, output);
    for (check, count) in after {
        assert!(
            count <= before.get(&check).copied().unwrap_or(0),
            "{name}: introduced {count} {check} validation error(s); input had {}",
            before.get(&check).copied().unwrap_or(0)
        );
    }
}

fn brep_edge_health(topo: &Topology, solid: SolidId) -> (usize, usize) {
    let mut uses: HashMap<usize, usize> = HashMap::new();
    for face_id in solid_faces(topo, solid).unwrap() {
        let face = topo.face(face_id).unwrap();
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire_id).unwrap().edges() {
                *uses.entry(oriented.edge().index()).or_insert(0) += 1;
            }
        }
    }
    (
        uses.values().filter(|&&count| count == 1).count(),
        uses.values().filter(|&&count| count > 2).count(),
    )
}

fn mesh_health_and_volume(topo: &Topology, solid: SolidId) -> ((usize, usize), f64) {
    let mesh = tessellate_solid_with_tolerance(topo, solid, MESH_DEFLECTION, 0.1).unwrap();
    let mut canonical: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut remap = vec![0_u32; mesh.positions.len()];
    for (index, point) in mesh.positions.iter().enumerate() {
        #[allow(clippy::cast_possible_truncation)]
        let key = (
            (point.x() * 1e7).round() as i64,
            (point.y() * 1e7).round() as i64,
            (point.z() * 1e7).round() as i64,
        );
        #[allow(clippy::cast_possible_truncation)]
        let next = canonical.len() as u32;
        remap[index] = *canonical.entry(key).or_insert(next);
    }

    let mut edge_uses: HashMap<(u32, u32), usize> = HashMap::new();
    let mut signed_six_volume = 0.0;
    for triangle in mesh.indices.chunks_exact(3) {
        let points = [
            mesh.positions[triangle[0] as usize],
            mesh.positions[triangle[1] as usize],
            mesh.positions[triangle[2] as usize],
        ];
        signed_six_volume += points[0].x()
            * (points[1].y() * points[2].z() - points[1].z() * points[2].y())
            - points[0].y() * (points[1].x() * points[2].z() - points[1].z() * points[2].x())
            + points[0].z() * (points[1].x() * points[2].y() - points[1].y() * points[2].x());

        let vertices = [
            remap[triangle[0] as usize],
            remap[triangle[1] as usize],
            remap[triangle[2] as usize],
        ];
        for (a, b) in [
            (vertices[0], vertices[1]),
            (vertices[1], vertices[2]),
            (vertices[2], vertices[0]),
        ] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_uses.entry(key).or_insert(0) += 1;
        }
    }
    let health = (
        edge_uses.values().filter(|&&count| count == 1).count(),
        edge_uses.values().filter(|&&count| count > 2).count(),
    );
    (health, (signed_six_volume / 6.0).abs())
}

fn assert_built(topo: &Topology, input: SolidId, output: SolidId, name: &str) {
    let shell = topo
        .shell(topo.solid(output).unwrap().outer_shell())
        .unwrap();
    validate_shell_closed(shell, topo).unwrap_or_else(|error| panic!("{name}: {error}"));
    validate_shell_manifold(shell, topo).unwrap_or_else(|error| panic!("{name}: {error}"));
    assert_eq!(
        brep_edge_health(topo, output),
        (0, 0),
        "{name}: B-Rep must have zero free and non-manifold edges"
    );
    assert_no_validation_regression(topo, input, output, name);

    let (mesh_health, mesh_volume) = mesh_health_and_volume(topo, output);
    assert_eq!(
        mesh_health,
        (0, 0),
        "{name}: mesh must have zero boundary and non-manifold edges"
    );
    let measured = solid_volume(topo, output, MEASURE_DEFLECTION).unwrap();
    let relative = (measured - mesh_volume).abs() / measured.abs().max(1.0);
    assert!(
        measured > 0.0 && mesh_volume > 0.0 && relative < VOLUME_REL_TOL,
        "{name}: measured volume {measured} disagrees with mesh oracle {mesh_volume} ({:.2}%)",
        relative * 100.0
    );
}

fn edge_between(topo: &Topology, solid: SolidId, a: Point3, b: Point3) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            ((start - a).length() < TOL && (end - b).length() < TOL)
                || ((start - b).length() < TOL && (end - a).length() < TOL)
        })
        .unwrap_or_else(|| panic!("edge {a:?} -- {b:?}"))
}

fn successful_fillet(
    mut topo: Topology,
    input: SolidId,
    edges: &[EdgeId],
    radius: f64,
    name: &str,
) -> Observed {
    let result = fillet_v2(&mut topo, input, edges, radius)
        .unwrap_or_else(|error| panic!("{name}: expected supported side, got {error}"));
    assert!(result.failed.is_empty(), "{name}: partial blend");
    Observed::Built {
        topo,
        input,
        output: result.solid,
    }
}

fn refusal_after_supported_side(
    mut topo: Topology,
    input: SolidId,
    edges: &[EdgeId],
    supported_radius: f64,
    refused_radius: f64,
    name: &str,
) -> Observed {
    let mut supported_topo = topo.clone();
    let supported = fillet_v2(&mut supported_topo, input, edges, supported_radius)
        .unwrap_or_else(|error| panic!("{name}: supported side failed: {error}"));
    assert_built(
        &supported_topo,
        input,
        supported.solid,
        &format!("{name}/supported-side"),
    );

    let before = snapshot(&topo, input);
    let error = match fillet_v2(&mut topo, input, edges, refused_radius) {
        Ok(_) => panic!("{name}: torture cliff unexpectedly built"),
        Err(error) => error,
    };
    Observed::TypedRefusal {
        topo,
        input,
        error,
        before,
    }
}

fn cliff_after_supported_side(
    topo: Topology,
    input: SolidId,
    edges: &[EdgeId],
    supported_radius: f64,
    refused_radius: f64,
    expected_available: f64,
    name: &str,
) -> Observed {
    let observed =
        refusal_after_supported_side(topo, input, edges, supported_radius, refused_radius, name);
    let Observed::TypedRefusal {
        topo, input, error, ..
    } = &observed
    else {
        panic!("{name}: cliff witness unexpectedly built");
    };
    let OperationsError::Blend(remus_operations::blend_ops::BlendError::CliffEncountered {
        edge,
        face,
        requested_radius,
        available_radius,
    }) = error
    else {
        panic!("{name}: expected CliffEncountered, got {error}");
    };
    assert!(solid_edges(topo, *input).unwrap().contains(edge));
    assert!(solid_faces(topo, *input).unwrap().contains(face));
    assert!((requested_radius - refused_radius).abs() < TOL);
    assert!((available_radius - expected_available).abs() < TOL);
    observed
}

fn band_consumes_adjacent_face(name: &str) -> Observed {
    let mut topo = Topology::new();
    let input = make_box(&mut topo, 10.0, 10.0, 20.0).unwrap();
    let edge = edge_between(
        &topo,
        input,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 20.0),
    );
    cliff_after_supported_side(topo, input, &[edge], 9.9, 10.0, 10.0, name)
}

fn band_meets_band(name: &str) -> Observed {
    let mut topo = Topology::new();
    let input = make_box(&mut topo, 20.0, 16.0, 8.0).unwrap();
    let edges = [
        edge_between(
            &topo,
            input,
            Point3::new(0.0, 0.0, 8.0),
            Point3::new(20.0, 0.0, 8.0),
        ),
        edge_between(
            &topo,
            input,
            Point3::new(0.0, 0.0, 8.0),
            Point3::new(0.0, 16.0, 8.0),
        ),
    ];
    successful_fillet(topo, input, &edges, 2.0, name)
}

fn thin_wall(name: &str) -> Observed {
    let mut topo = Topology::new();
    let input = make_cylinder(&mut topo, 8.0, 2.0).unwrap();
    let edges: Vec<_> = solid_edges(&topo, input)
        .unwrap()
        .into_iter()
        .filter(|&edge_id| matches!(topo.edge(edge_id).unwrap().curve(), EdgeCurve::Circle(_)))
        .collect();
    assert_eq!(edges.len(), 2, "{name}: cylinder has two rims");
    cliff_after_supported_side(topo, input, &edges, 0.9, 1.1, 0.9, name)
}

fn box_vertex_pileup(name: &str) -> Observed {
    let mut topo = Topology::new();
    let input = make_box(&mut topo, 12.0, 10.0, 8.0).unwrap();
    let origin = Point3::new(0.0, 0.0, 0.0);
    let edges: Vec<_> = solid_edges(&topo, input)
        .unwrap()
        .into_iter()
        .filter(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            (start - origin).length() < TOL || (end - origin).length() < TOL
        })
        .collect();
    assert_eq!(edges.len(), 3, "{name}: box corner degree");
    successful_fillet(topo, input, &edges, 1.0, name)
}

fn pyramid_pileup<const N: usize>(name: &str) -> Observed {
    let mut topo = Topology::new();
    let mut points = Vec::with_capacity(N + 1);
    for index in 0..N {
        let angle =
            TAU * f64::from(u32::try_from(index).unwrap()) / f64::from(u32::try_from(N).unwrap());
        points.push(Point3::new(6.0 * angle.cos(), 6.0 * angle.sin(), 0.0));
    }
    let apex = Point3::new(0.0, 0.0, 9.0);
    points.push(apex);
    let input = make_convex_hull(&mut topo, &points).unwrap();
    remus_operations::heal::unify_faces(&mut topo, input).unwrap();
    let edges: Vec<_> = solid_edges(&topo, input)
        .unwrap()
        .into_iter()
        .filter(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            (start - apex).length() < TOL || (end - apex).length() < TOL
        })
        .collect();
    assert_eq!(edges.len(), N, "{name}: pyramid apex degree");
    let before = snapshot(&topo, input);
    match fillet_v2(&mut topo, input, &edges, 0.5) {
        Ok(result) => Observed::Built {
            topo,
            input,
            output: result.solid,
        },
        Err(error) => Observed::TypedRefusal {
            topo,
            input,
            error,
            before,
        },
    }
}

fn four_edge_pileup(name: &str) -> Observed {
    pyramid_pileup::<4>(name)
}

fn five_edge_pileup(name: &str) -> Observed {
    pyramid_pileup::<5>(name)
}

fn extruded_profile(topo: &mut Topology, points: &[Point3], height: f64) -> SolidId {
    let wire = make_polygon_wire(topo, points, 1e-7).unwrap();
    let face = topo.add_face(Face::new(
        wire,
        Vec::new(),
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), height).unwrap()
}

fn mixed_convexity_chain(_name: &str) -> Observed {
    let mut topo = Topology::new();
    let input = extruded_profile(
        &mut topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(7.0, 0.0, 0.0),
            Point3::new(7.0, 2.0, 0.0),
            Point3::new(2.0, 2.0, 0.0),
            Point3::new(2.0, 7.0, 0.0),
            Point3::new(0.0, 7.0, 0.0),
        ],
        5.0,
    );
    let edges = [
        edge_between(
            &topo,
            input,
            Point3::new(2.0, 2.0, 0.0),
            Point3::new(2.0, 2.0, 5.0),
        ),
        edge_between(
            &topo,
            input,
            Point3::new(2.0, 2.0, 5.0),
            Point3::new(2.0, 7.0, 5.0),
        ),
    ];
    let mut concave_topo = topo.clone();
    let concave_before = snapshot(&concave_topo, input);
    let concave_error = match fillet_v2(&mut concave_topo, input, &[edges[0]], 0.4) {
        Ok(_) => panic!("mixed-convexity-chain/concave unexpectedly built"),
        Err(error) => error,
    };
    assert_eq!(
        blend_failure_code(&concave_error),
        "invalid-input",
        "mixed-convexity-chain/concave: {concave_error}"
    );
    assert_unchanged(
        &concave_topo,
        input,
        &concave_before,
        "mixed-convexity-chain/concave",
    );

    let mut convex_topo = topo.clone();
    let convex = fillet_v2(&mut convex_topo, input, &[edges[1]], 0.4)
        .unwrap_or_else(|error| panic!("mixed-convexity-chain/convex: {error}"));
    assert_built(
        &convex_topo,
        input,
        convex.solid,
        "mixed-convexity-chain/convex",
    );
    let before = snapshot(&topo, input);
    match fillet_v2(&mut topo, input, &edges, 0.4) {
        Ok(result) => Observed::Built {
            topo,
            input,
            output: result.solid,
        },
        Err(error) => Observed::TypedRefusal {
            topo,
            input,
            error,
            before,
        },
    }
}

fn tangent_continuation_chain(name: &str) -> Observed {
    let mut topo = Topology::new();
    let (input, [first, second]) = make_tangent_split_box(&mut topo);
    let expanded = remus_blend::g1_chain::expand_g1_chain(
        &topo,
        input,
        &[first],
        remus_math::tolerance::Tolerance::new(),
    )
    .unwrap();
    assert_eq!(
        expanded.into_iter().collect::<HashSet<_>>(),
        HashSet::from([first, second]),
        "{name}: seed must expand across the tangent split"
    );
    successful_fillet(topo, input, &[first], 0.75, name)
}

fn make_tangent_split_box(topo: &mut Topology) -> (SolidId, [EdgeId; 2]) {
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(8.0, 0.0, 0.0),
        Point3::new(8.0, 5.0, 0.0),
        Point3::new(0.0, 5.0, 0.0),
        Point3::new(0.0, 0.0, 4.0),
        Point3::new(4.0, 0.0, 4.0),
        Point3::new(8.0, 0.0, 4.0),
        Point3::new(8.0, 5.0, 4.0),
        Point3::new(0.0, 5.0, 4.0),
    ];
    let vertices: Vec<_> = points
        .into_iter()
        .map(|point| topo.add_vertex(Vertex::new(point, 1e-7)))
        .collect();
    let edge = |topo: &mut Topology, start, end| {
        topo.add_edge(Edge::new(vertices[start], vertices[end], EdgeCurve::Line))
    };
    let bottom = [
        edge(topo, 0, 1),
        edge(topo, 1, 2),
        edge(topo, 2, 3),
        edge(topo, 3, 0),
    ];
    let top_front = [edge(topo, 4, 5), edge(topo, 5, 6)];
    let top = [
        top_front[0],
        top_front[1],
        edge(topo, 6, 7),
        edge(topo, 7, 8),
        edge(topo, 8, 4),
    ];
    let vertical = [
        edge(topo, 0, 4),
        edge(topo, 1, 6),
        edge(topo, 2, 7),
        edge(topo, 3, 8),
    ];

    let mut face = |edges: &[(EdgeId, bool)], normal: Vec3, d: f64| {
        let wire = Wire::new(
            edges
                .iter()
                .map(|&(edge, forward)| OrientedEdge::new(edge, forward))
                .collect(),
            true,
        )
        .unwrap();
        let wire = topo.add_wire(wire);
        topo.add_face(Face::new(wire, vec![], FaceSurface::Plane { normal, d }))
    };
    let faces = vec![
        face(
            &[
                (bottom[0], false),
                (bottom[3], false),
                (bottom[2], false),
                (bottom[1], false),
            ],
            Vec3::new(0.0, 0.0, -1.0),
            0.0,
        ),
        face(
            &[
                (top[0], true),
                (top[1], true),
                (top[2], true),
                (top[3], true),
                (top[4], true),
            ],
            Vec3::new(0.0, 0.0, 1.0),
            4.0,
        ),
        face(
            &[
                (bottom[0], true),
                (vertical[1], true),
                (top[1], false),
                (top[0], false),
                (vertical[0], false),
            ],
            Vec3::new(0.0, -1.0, 0.0),
            0.0,
        ),
        face(
            &[
                (bottom[2], true),
                (vertical[3], true),
                (top[3], false),
                (vertical[2], false),
            ],
            Vec3::new(0.0, 1.0, 0.0),
            5.0,
        ),
        face(
            &[
                (bottom[3], true),
                (vertical[0], true),
                (top[4], false),
                (vertical[3], false),
            ],
            Vec3::new(-1.0, 0.0, 0.0),
            0.0,
        ),
        face(
            &[
                (bottom[1], true),
                (vertical[2], true),
                (top[2], false),
                (vertical[1], false),
            ],
            Vec3::new(1.0, 0.0, 0.0),
            8.0,
        ),
    ];
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    (topo.add_solid(Solid::new(shell, vec![])), top_front)
}

fn hole_rim(name: &str) -> Observed {
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, 24.0, 20.0, 6.0).unwrap();
    let drill = make_cylinder(&mut topo, 3.0, 10.0).unwrap();
    transform_solid(&mut topo, drill, &Mat4::translation(12.0, 10.0, -2.0)).unwrap();
    let input = boolean(&mut topo, BooleanOp::Cut, blank, drill).unwrap();
    let rim = solid_edges(&topo, input)
        .unwrap()
        .into_iter()
        .find(|&edge_id| {
            matches!(
                topo.edge(edge_id).unwrap().curve(),
                EdgeCurve::Circle(circle)
                    if (circle.center().z() - 6.0).abs() < TOL
            )
        })
        .expect("top hole rim");
    successful_fillet(topo, input, &[rim], 0.75, name)
}

fn fillet_the_fillet(name: &str) -> Observed {
    let mut topo = Topology::new();
    let input = make_box(&mut topo, 16.0, 14.0, 12.0).unwrap();
    let first_edge = edge_between(
        &topo,
        input,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 12.0),
    );
    let first = fillet_v2(&mut topo, input, &[first_edge], 2.0).expect("first fillet");
    assert_built(&topo, input, first.solid, &format!("{name}/first"));

    let adjacency = topo.build_adjacency(first.solid).unwrap();
    let candidates = filter_filletable_edges(
        &topo,
        first.solid,
        &solid_edges(&topo, first.solid).unwrap(),
    )
    .unwrap();
    let second_edge = candidates
        .into_iter()
        .find(|&edge_id| {
            adjacency.faces_for_edge(edge_id).iter().any(|&face_id| {
                matches!(
                    topo.face(face_id).unwrap().surface(),
                    FaceSurface::Cylinder(_) | FaceSurface::Nurbs(_)
                )
            })
        })
        .expect("filletable edge bordering the first blend band");
    let before = snapshot(&topo, first.solid);
    match fillet_v2(&mut topo, first.solid, &[second_edge], 0.5) {
        Ok(second) => Observed::Built {
            topo,
            input: first.solid,
            output: second.solid,
        },
        Err(error) => Observed::TypedRefusal {
            topo,
            input: first.solid,
            error,
            before,
        },
    }
}

const CASES: &[TortureCase] = &[
    TortureCase {
        name: "band-consumes-adjacent-face",
        expected: Expected::TypedRefusal("cliff-encountered"),
        run: band_consumes_adjacent_face,
    },
    TortureCase {
        name: "band-meets-band-at-shared-edge",
        expected: Expected::Built,
        run: band_meets_band,
    },
    TortureCase {
        name: "radius-at-least-support-width-thin-wall",
        expected: Expected::TypedRefusal("cliff-encountered"),
        run: thin_wall,
    },
    TortureCase {
        name: "three-edge-vertex-pileup",
        expected: Expected::Built,
        run: box_vertex_pileup,
    },
    TortureCase {
        name: "four-edge-vertex-pileup",
        expected: Expected::Built,
        run: four_edge_pileup,
    },
    TortureCase {
        name: "five-edge-vertex-pileup",
        expected: Expected::Built,
        run: five_edge_pileup,
    },
    TortureCase {
        name: "mixed-convexity-chain",
        expected: Expected::TypedRefusal("unsupported-vertex-blend"),
        run: mixed_convexity_chain,
    },
    TortureCase {
        name: "tangent-continuation-chain",
        expected: Expected::Built,
        run: tangent_continuation_chain,
    },
    TortureCase {
        name: "fillet-across-hole-rim",
        expected: Expected::Built,
        run: hole_rim,
    },
    TortureCase {
        name: "fillet-the-fillet",
        expected: Expected::TypedRefusal("trimming-failure"),
        run: fillet_the_fillet,
    },
];

#[test]
fn fillet_torture_corpus_is_built_or_typed() {
    let mut names = HashSet::new();
    for case in CASES {
        assert!(
            names.insert(case.name),
            "duplicate case name: {}",
            case.name
        );
        run_case(case);
    }
}
