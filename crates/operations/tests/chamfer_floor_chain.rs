//! Chamfers along runs of edges round one planar floor, mixing straight
//! edges and arcs.
//!
//! A block with a round boss on its corner and concave R4 seam rounds between
//! them has a bottom outline that is one tangent-continuous run — the block's
//! straight bottom edges, the rounds' concave feet and the boss's convex foot
//! circle — closed by three sharp block corners. Chamfering that run used to
//! be refused (`trimming failure` on the bottom face: each per-edge stripe
//! trimmed it in turn, and a stripe ending at a tangent vertex had no end face
//! to close against). These tests hold the result to closed forms that do not
//! read the result: a straight run removes its triangular section times its
//! length, an arc run removes the section swept about the arc's axis (Pappus:
//! area × angle × centroid radius), and each convex 90° mitre counts the corner
//! cube's overlap once (`d³/3` less).
//!
//! The fixture is the bottom of the supplied Gentle-Gecko part, built
//! natively: a 30×18×24 block, an R6 boss on its corner, R4 seam rounds.

#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout,
    clippy::unwrap_used
)]

use std::f64::consts::{FRAC_PI_2, PI};

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::tolerance::Tolerance;
use remus_math::vec::Point3;
use remus_operations::blend_ops::{chamfer_v2, chamfer_v2_tangent_chain, fillet_cascade};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::heal::unify_faces;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::validation::{validate_shell_closed, validate_shell_manifold};

const BLOCK: (f64, f64, f64) = (30.0, 18.0, 24.0);
const BOSS: f64 = 6.0;
const ROUND: f64 = 4.0;
const D: f64 = 0.8;

/// The block, the boss flush with its top, and R4 rounds in both concave
/// seams: the Gentle-Gecko bottom outline.
fn gecko_bottom(topo: &mut Topology) -> SolidId {
    let block = make_box(topo, BLOCK.0, BLOCK.1, BLOCK.2).unwrap();
    let boss = make_cylinder(topo, BOSS, BLOCK.2).unwrap();
    let body = boolean(topo, BooleanOp::Fuse, block, boss).unwrap();
    unify_faces(topo, body).unwrap();
    let seams: Vec<EdgeId> = solid_edges(topo, body)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let (a, b) = ends(topo, id);
            (a.z() - b.z()).abs() > BLOCK.2 - 1e-6
                && [(BOSS, 0.0), (0.0, BOSS)].iter().any(|&(x, y)| {
                    (a.x() - x).abs() + (a.y() - y).abs() + (b.x() - x).abs() + (b.y() - y).abs()
                        < 1e-6
                })
        })
        .collect();
    assert_eq!(seams.len(), 2, "both concave seam rulings");
    let rounded = fillet_cascade(topo, body, &seams, ROUND).unwrap().solid;
    // The rounds fill their corners with planar pieces coplanar with the
    // bottom and top; merge them, as the supplied STEP has them.
    unify_faces(topo, rounded).unwrap();
    rounded
}

/// A 30×18×10 block with all four vertical edges rounded R3: a closed
/// tangent-continuous bottom outline of lines and convex arcs.
fn rounded_block(topo: &mut Topology) -> SolidId {
    let block = make_box(topo, 30.0, 18.0, 10.0).unwrap();
    let verticals: Vec<EdgeId> = solid_edges(topo, block)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let (a, b) = ends(topo, id);
            (a.z() - b.z()).abs() > 9.0
        })
        .collect();
    assert_eq!(verticals.len(), 4);
    fillet_cascade(topo, block, &verticals, 3.0).unwrap().solid
}

fn ends(topo: &Topology, edge: EdgeId) -> (Point3, Point3) {
    let data = topo.edge(edge).unwrap();
    (
        topo.vertex(data.start()).unwrap().point(),
        topo.vertex(data.end()).unwrap().point(),
    )
}

fn bottom_face(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| {
            matches!(topo.face(f).unwrap().surface(),
                FaceSurface::Plane { normal, d } if normal.z().abs() > 0.999 && d.abs() < 1e-9)
        })
        .expect("bottom face")
}

fn bottom_outline(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    let face = topo.face(bottom_face(topo, solid)).unwrap();
    topo.wire(face.outer_wire())
        .unwrap()
        .edges()
        .iter()
        .map(remus_topology::wire::OrientedEdge::edge)
        .collect()
}

/// The straight bottom edge along `y = 0`.
fn seed(topo: &Topology, solid: SolidId) -> EdgeId {
    bottom_outline(topo, solid)
        .into_iter()
        .find(|&e| {
            let (a, b) = ends(topo, e);
            a.y().abs() + b.y().abs() < 1e-9 && (a.x() - b.x()).abs() > 10.0
        })
        .unwrap()
}

/// Closed-form material a 45° chamfer of setback `d` removes along `edges`
/// of the floor: section `d²/2` times each line's length, and for an arc the
/// section swept about its axis at its centroid radius, `r ∓ d/3` (convex
/// boss foot / concave round foot: the material is inside / outside the arc).
fn removed_along(topo: &Topology, solid: SolidId, edges: &[EdgeId], d: f64) -> f64 {
    let section = 0.5 * d * d;
    let adjacency = topo.build_adjacency(solid).unwrap();
    let mut removed = 0.0;
    for &edge in edges {
        let data = topo.edge(edge).unwrap();
        let (a, b) = ends(topo, edge);
        match data.curve() {
            EdgeCurve::Line => removed += section * (b - a).length(),
            EdgeCurve::Circle(circle) => {
                let (t0, t1) = data.strict_domain().unwrap();
                let sweep = (t1 - t0).abs();
                let wall = adjacency
                    .faces_for_edge(edge)
                    .iter()
                    .copied()
                    .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
                    .unwrap();
                // A reversed wall is a concave round: material outside it.
                let concave = topo.face(wall).unwrap().is_reversed();
                let centroid = if concave {
                    circle.radius() + d / 3.0
                } else {
                    circle.radius() - d / 3.0
                };
                removed += section * sweep * centroid;
            }
            other => panic!("unexpected bottom edge {}", other.type_tag()),
        }
    }
    removed
}

/// The verification bar: closed manifold B-Rep, clean validation, watertight
/// meshes at two deflections, and the API volume against the closed form and
/// against an independent mesh-volume sum.
fn assert_sound(
    topo: &Topology,
    input: SolidId,
    result: SolidId,
    expected_removed: f64,
    label: &str,
) {
    let shell = topo
        .shell(topo.solid(result).unwrap().outer_shell())
        .unwrap();
    validate_shell_closed(shell, topo).unwrap_or_else(|e| panic!("{label}: open shell: {e:?}"));
    validate_shell_manifold(shell, topo).unwrap_or_else(|e| panic!("{label}: {e:?}"));
    let report = validate_solid(topo, result).unwrap();
    assert!(report.is_valid(), "{label}: {:?}", report.issues);

    let diagonal = (BLOCK.0 * BLOCK.0 + BLOCK.1 * BLOCK.1 + BLOCK.2 * BLOCK.2).sqrt();
    let before = solid_volume(topo, input, diagonal * 2e-5).unwrap();
    let mut volumes = Vec::new();
    for deflection in [diagonal * 4e-5, diagonal * 1e-5] {
        let mesh = tessellate_solid(topo, result, deflection).unwrap();
        assert_eq!(
            boundary_edge_count(&mesh),
            0,
            "{label}: mesh boundary @{deflection}"
        );
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "{label}: mesh non-manifold @{deflection}"
        );
        let api = solid_volume(topo, result, deflection).unwrap();
        let mesh_volume = signed_mesh_volume(&mesh);
        assert!(
            (api - mesh_volume).abs() <= 1e-4 * api,
            "{label}: API volume {api} vs mesh {mesh_volume}"
        );
        volumes.push(api);
    }
    assert!(
        (volumes[0] - volumes[1]).abs() <= 1e-4 * volumes[1],
        "{label}: volume moves with deflection: {volumes:?}"
    );
    let removed = before - volumes[1];
    println!(
        "{label}: removed {removed:.6}, closed form {expected_removed:.6}, volumes {volumes:?}"
    );
    assert!(
        (removed - expected_removed).abs() <= 2e-3 * expected_removed + 1e-6 * before,
        "{label}: removed {removed}, closed form {expected_removed}"
    );
}

/// Signed-tetrahedron volume of a closed mesh, about a point on the body.
fn signed_mesh_volume(mesh: &remus_operations::tessellate::TriangleMesh) -> f64 {
    let origin = mesh.positions[0];
    let mut six = 0.0;
    for tri in mesh.indices.chunks_exact(3) {
        let [a, b, c] = [tri[0], tri[1], tri[2]].map(|i| mesh.positions[i as usize] - origin);
        six += a.dot(b.cross(c));
    }
    six / 6.0
}

/// Every face the chamfer created is an exact plane or cone.
fn assert_exact_bands(
    topo: &Topology,
    input: SolidId,
    result: SolidId,
    expected: usize,
    label: &str,
) {
    let before = solid_faces(topo, input).unwrap();
    let created: Vec<FaceId> = solid_faces(topo, result)
        .unwrap()
        .into_iter()
        .filter(|f| !before.contains(f))
        .collect();
    let bands: Vec<&str> = created
        .iter()
        .map(|&f| topo.face(f).unwrap().surface().type_tag())
        .collect();
    assert!(
        !solid_faces(topo, result)
            .unwrap()
            .iter()
            .any(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_))),
        "{label}: a NURBS face in the result"
    );
    let count = |tag: &str| bands.iter().filter(|&&t| t == tag).count();
    let new_faces = solid_faces(topo, result).unwrap().len() - before.len();
    assert_eq!(new_faces, expected, "{label}: band count");
    assert!(
        bands
            .iter()
            .all(|&t| t == "plane" || t == "cone" || t == "cylinder"),
        "{label}: {bands:?}"
    );
    assert!(count("cone") >= 1, "{label}: arcs must be cones: {bands:?}");
}

fn classify(topo: &Topology, solid: SolidId, p: Point3) -> PointClassification {
    classify_point(topo, solid, p, &ClassifyOptions::default()).unwrap()
}

/// Probes just inside the removed wedge are outside; probes past it inside.
fn assert_bevelled_at(
    topo: &Topology,
    solid: SolidId,
    edge_point: Point3,
    into: (f64, f64),
    label: &str,
) {
    let (ix, iy) = into;
    let at = |inward: f64, up: f64| {
        Point3::new(
            edge_point.x() + ix * inward,
            edge_point.y() + iy * inward,
            up,
        )
    };
    assert_eq!(
        classify(topo, solid, at(0.2, 0.2)),
        PointClassification::Outside,
        "{label}: wedge kept"
    );
    assert_eq!(
        classify(topo, solid, at(0.6, 0.6)),
        PointClassification::Inside,
        "{label}: over-cut"
    );
    assert_eq!(
        classify(topo, solid, at(1.2, 0.1)),
        PointClassification::Inside,
        "{label}: floor lost"
    );
}

/// Probes on the mixed outline: a straight edge, a concave round's foot and
/// the boss's foot.
fn assert_bevelled_outline(topo: &Topology, solid: SolidId, label: &str) {
    assert_bevelled_at(
        topo,
        solid,
        Point3::new(20.0, 0.0, 0.0),
        (0.0, 1.0),
        &format!("{label} front"),
    );
    assert_bevelled_at(
        topo,
        solid,
        Point3::new(0.0, 14.0, 0.0),
        (1.0, 0.0),
        &format!("{label} side"),
    );
    let boss = (-FRAC_PI_2 - 0.6_f64, 0.0);
    let (c, s) = (boss.0.cos(), boss.0.sin());
    assert_bevelled_at(
        topo,
        solid,
        Point3::new(BOSS * c, BOSS * s, 0.0),
        (-c, -s),
        &format!("{label} boss foot"),
    );
    // The round's foot, halfway round its arc; material lies away from its
    // axis.
    let axis = ((BOSS + ROUND).powi(2) - ROUND * ROUND).sqrt();
    let half = 0.5 * (FRAC_PI_2 + (PI - ROUND.atan2(axis)));
    let (c, s) = (half.cos(), half.sin());
    assert_bevelled_at(
        topo,
        solid,
        Point3::new(axis + ROUND * c, -ROUND + ROUND * s, 0.0),
        (c, s),
        &format!("{label} round foot"),
    );
}

#[test]
fn tangent_chain_from_one_bottom_edge_runs_the_mixed_outline() {
    let mut topo = Topology::new();
    let body = gecko_bottom(&mut topo);
    let seed = seed(&topo, body);
    let chain =
        remus_blend::g1_chain::expand_g1_chain(&topo, body, &[seed], Tolerance::new()).unwrap();
    // Two straight block edges, two round feet and the boss foot.
    assert!(chain.len() >= 5, "chain {chain:?}");
    let expected = removed_along(&topo, body, &chain, D);
    let source_counts = solid_entity_counts(&topo, body).unwrap();

    let result = chamfer_v2_tangent_chain(&mut topo, body, &[seed], D, D).unwrap();
    assert!(!result.is_partial && result.failed.is_empty());
    assert_sound(&topo, body, result.solid, expected, "open chain");
    // Line, round, boss, round, line: five bands.
    assert_exact_bands(&topo, body, result.solid, 5, "open chain");
    assert_bevelled_outline(&topo, result.solid, "open chain");

    // The bottom face is inset: none of its boundary is left on the chain,
    // and every point of its new boundary sits at least `D` in from the old
    // chain, the contacts exactly `D`.
    let bottom = bottom_outline(&topo, result.solid);
    assert!(
        chain.iter().all(|e| !bottom.contains(e)),
        "chain edge left on the bottom"
    );
    let samples = sample_edges(&topo, &chain, 2048);
    let mut on_contact = 0;
    for &edge in &bottom {
        if chain.contains(&edge) {
            continue;
        }
        for p in sample_edges(&topo, &[edge], 16) {
            assert!(p.z().abs() < 1e-9);
            let gap = samples
                .iter()
                .map(|q| (p - *q).length())
                .fold(f64::INFINITY, f64::min);
            assert!(
                gap >= D - 1e-3,
                "bottom point {p:?} only {gap} from the chain"
            );
            if (gap - D).abs() < 1e-3 {
                on_contact += 1;
            }
        }
    }
    assert!(
        on_contact > 5 * 16,
        "the inset contacts run the whole chain"
    );
    // The two block corners the run ends at are notched, not mitred.
    for corner in [Point3::new(BLOCK.0, D, 0.0), Point3::new(D, BLOCK.1, 0.0)] {
        assert!(
            bottom.iter().any(|&e| {
                let (a, b) = ends(&topo, e);
                (a - corner).length() < 1e-9 || (b - corner).length() < 1e-9
            }),
            "no end-notch vertex at {corner:?}"
        );
    }
    // The source body is untouched.
    assert_eq!(solid_entity_counts(&topo, body).unwrap(), source_counts);
}

#[test]
fn whole_bottom_outline_mitres_the_block_corners() {
    let mut topo = Topology::new();
    let body = gecko_bottom(&mut topo);
    let outline = bottom_outline(&topo, body);
    // Three convex 90° mitres, each counting the corner overlap once.
    let expected = removed_along(&topo, body, &outline, D) - 3.0 * D * D * D / 3.0;

    let result = chamfer_v2(&mut topo, body, &outline, D, D).unwrap();
    assert!(!result.is_partial && result.failed.is_empty());
    assert_sound(&topo, body, result.solid, expected, "closed outline");
    // The five chain bands plus one per remaining straight edge.
    assert_exact_bands(&topo, body, result.solid, 7, "closed outline");
    assert_bevelled_outline(&topo, result.solid, "closed outline");
    // Each mitre runs from `(D, D)` in from the corner up to `D` on its edge.
    for (floor, wall) in [
        (
            Point3::new(BLOCK.0 - D, D, 0.0),
            Point3::new(BLOCK.0, 0.0, D),
        ),
        (
            Point3::new(BLOCK.0 - D, BLOCK.1 - D, 0.0),
            Point3::new(BLOCK.0, BLOCK.1, D),
        ),
        (
            Point3::new(D, BLOCK.1 - D, 0.0),
            Point3::new(0.0, BLOCK.1, D),
        ),
    ] {
        assert!(
            solid_edges(&topo, result.solid)
                .unwrap()
                .into_iter()
                .any(|e| {
                    let (a, b) = ends(&topo, e);
                    ((a - floor).length() < 1e-9 && (b - wall).length() < 1e-9)
                        || ((a - wall).length() < 1e-9 && (b - floor).length() < 1e-9)
                }),
            "no mitre {floor:?}–{wall:?}"
        );
    }
    // No bottom-face boundary is left on the old outline.
    let bottom = bottom_outline(&topo, result.solid);
    assert!(outline.iter().all(|e| !bottom.contains(e)));
}

#[test]
fn closed_tangent_loop_of_lines_and_convex_arcs() {
    let mut topo = Topology::new();
    let body = rounded_block(&mut topo);
    let outline = bottom_outline(&topo, body);
    let expected = removed_along(&topo, body, &outline, D);
    let result = chamfer_v2(&mut topo, body, &outline, D, D).unwrap();
    assert!(!result.is_partial && result.failed.is_empty());
    assert_sound(&topo, body, result.solid, expected, "rounded loop");
    // Four straight bands and four conical corners.
    assert_exact_bands(&topo, body, result.solid, 8, "rounded loop");
    // The propagated chamfer from one edge is the same loop.
    let mut again = Topology::new();
    let body2 = rounded_block(&mut again);
    let seed2 = seed(&again, body2);
    let propagated = chamfer_v2_tangent_chain(&mut again, body2, &[seed2], D, D).unwrap();
    assert_sound(
        &again,
        body2,
        propagated.solid,
        expected,
        "rounded loop from one edge",
    );
}

#[test]
fn oversized_chamfer_refuses_and_leaves_the_input_intact() {
    let mut topo = Topology::new();
    let body = gecko_bottom(&mut topo);
    let seed = seed(&topo, body);
    let counts = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_faces(),
        topo.num_solids(),
    );
    let before = solid_volume(&topo, body, 0.01).unwrap();
    // Beyond the boss radius the boss foot's cone passes its axis.
    assert!(chamfer_v2_tangent_chain(&mut topo, body, &[seed], 7.0, 7.0).is_err());
    assert_eq!(
        counts,
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_faces(),
            topo.num_solids()
        ),
        "the refusal must roll the arena back"
    );
    assert!((solid_volume(&topo, body, 0.01).unwrap() - before).abs() < 1e-9);
}

#[test]
fn single_straight_edge_is_not_propagated_by_plain_chamfer() {
    // `chamfer_v2` keeps chamfering exactly the selected edge; only the
    // tangent-chain entry point propagates.
    let mut topo = Topology::new();
    let body = gecko_bottom(&mut topo);
    let seed = seed(&topo, body);
    let (a, b) = ends(&topo, seed);
    let result = chamfer_v2(&mut topo, body, &[seed], D, D).unwrap();
    let removed = solid_volume(&topo, body, 0.002).unwrap()
        - solid_volume(&topo, result.solid, 0.002).unwrap();
    let straight = 0.5 * D * D * (b - a).length();
    assert!(
        removed < 1.5 * straight,
        "plain chamfer propagated: removed {removed} against one edge's {straight}"
    );
}

/// Points along `edges`, `per_edge` + 1 each, ends included.
fn sample_edges(topo: &Topology, edges: &[EdgeId], per_edge: u32) -> Vec<Point3> {
    let mut points = Vec::new();
    for &edge in edges {
        let data = topo.edge(edge).unwrap();
        let (a, b) = ends(topo, edge);
        let (t0, t1) = data.strict_domain().unwrap();
        for i in 0..=per_edge {
            let t = t0 + (t1 - t0) * f64::from(i) / f64::from(per_edge);
            points.push(data.curve().evaluate_with_endpoints(t, a, b));
        }
    }
    points
}

/// The whole Gentle-Gecko W-Fillet part, built as the workflow's step A0
/// builds it: the boss rises 12 past the block, and one R4 cascade rounds
/// both seam rulings, the seam arc across the block's top and the two top
/// edges running into it. The corners where those rounds meet are freeform
/// patches, far above the bottom outline.
fn gecko_w_fillet(topo: &mut Topology) -> SolidId {
    let block = make_box(topo, BLOCK.0, BLOCK.1, BLOCK.2).unwrap();
    let boss = make_cylinder(topo, BOSS, 36.0).unwrap();
    let body = boolean(topo, BooleanOp::Fuse, block, boss).unwrap();
    unify_faces(topo, body).unwrap();
    let pairs = [
        ((BOSS, 0.0, 0.0), (BOSS, 0.0, BLOCK.2)),
        ((BOSS, 0.0, BLOCK.2), (BLOCK.0, 0.0, BLOCK.2)),
        ((0.0, BOSS, BLOCK.2), (BOSS, 0.0, BLOCK.2)),
        ((0.0, BOSS, BLOCK.2), (0.0, BLOCK.1, BLOCK.2)),
        ((0.0, BOSS, 0.0), (0.0, BOSS, BLOCK.2)),
    ];
    let near = |p: Point3, q: (f64, f64, f64)| (p - Point3::new(q.0, q.1, q.2)).length() < 1e-6;
    let edges: Vec<EdgeId> = pairs
        .iter()
        .map(|&(p, q)| {
            solid_edges(topo, body)
                .unwrap()
                .into_iter()
                .find(|&e| {
                    let (a, b) = ends(topo, e);
                    (near(a, p) && near(b, q)) || (near(a, q) && near(b, p))
                })
                .unwrap_or_else(|| panic!("edge {p:?}–{q:?}"))
        })
        .collect();
    let rounded = fillet_cascade(topo, body, &edges, ROUND).unwrap().solid;
    unify_faces(topo, rounded).unwrap();
    rounded
}

#[test]
fn w_fillet_part_chamfers_its_bottom_chain_and_outline() {
    for whole in [false, true] {
        let label = if whole {
            "W-Fillet outline"
        } else {
            "W-Fillet chain"
        };
        let mut topo = Topology::new();
        let body = gecko_w_fillet(&mut topo);
        let outline = bottom_outline(&topo, body);
        assert_eq!(
            outline.len(),
            9,
            "{label}: the supplied part's bottom outline"
        );
        let seed = seed(&topo, body);
        let (result, expected, bands) = if whole {
            let expected = removed_along(&topo, body, &outline, D) - D * D * D;
            (chamfer_v2(&mut topo, body, &outline, D, D), expected, 7)
        } else {
            let chain =
                remus_blend::g1_chain::expand_g1_chain(&topo, body, &[seed], Tolerance::new())
                    .unwrap();
            let expected = removed_along(&topo, body, &chain, D);
            (
                chamfer_v2_tangent_chain(&mut topo, body, &[seed], D, D),
                expected,
                5,
            )
        };
        let result = result.unwrap_or_else(|e| panic!("{label}: {e}"));
        assert_sound(&topo, body, result.solid, expected, label);
        let freeform = |solid| {
            solid_faces(&topo, solid)
                .unwrap()
                .into_iter()
                .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_)))
                .count()
        };
        assert_eq!(
            freeform(result.solid),
            freeform(body),
            "{label}: only the part's own patches"
        );
        let new_faces = solid_faces(&topo, result.solid).unwrap().len()
            - solid_faces(&topo, body).unwrap().len();
        assert_eq!(new_faces, bands, "{label}");
        assert_bevelled_outline(&topo, result.solid, label);
    }
}

// ---------------------------------------------------------------------------
// Two chamfer chains meeting at a block corner.

/// A 30×18×10 block with its two top long edges and the two vertical edges
/// of its `x = 0` end rounded R3. The `x = 30` end's outline (top edge, the
/// top rounds' end arcs, the vertical edges) and the bottom outline (the
/// long bottom edges, the vertical rounds' feet, the `x = 0` bottom edge)
/// are two open tangent chains, each with arcs, that meet at the `x = 30`
/// bottom corners.
fn rounded_free_end(topo: &mut Topology) -> SolidId {
    let block = make_box(topo, 30.0, 18.0, 10.0).unwrap();
    let edges: Vec<EdgeId> = solid_edges(topo, block)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let (a, b) = ends(topo, id);
            let side =
                (a.y() - b.y()).abs() < 1e-9 && (a.y().abs() < 1e-9 || (a.y() - 18.0).abs() < 1e-9);
            let vertical = a.x().abs() + b.x().abs() < 1e-9;
            let top = (a.z() - 10.0).abs() + (b.z() - 10.0).abs() < 1e-9;
            side && (vertical || top)
        })
        .collect();
    assert_eq!(edges.len(), 4);
    fillet_cascade(topo, block, &edges, 3.0).unwrap().solid
}

/// The free end's top edge: the seed of its chain.
fn free_end_seed(topo: &Topology, solid: SolidId) -> EdgeId {
    let found: Vec<EdgeId> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let (a, b) = ends(topo, id);
            matches!(topo.edge(id).unwrap().curve(), EdgeCurve::Line)
                && (a.x() - 30.0).abs() + (b.x() - 30.0).abs() < 1e-9
                && (a.z() - 10.0).abs() + (b.z() - 10.0).abs() < 1e-9
        })
        .collect();
    assert_eq!(found.len(), 1);
    found[0]
}

/// The longest straight bottom edge along `y = 0`: the bottom chain's seed.
fn long_bottom_seed(topo: &Topology, solid: SolidId) -> EdgeId {
    let length = |id: EdgeId| {
        let (a, b) = ends(topo, id);
        (b - a).length()
    };
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let (a, b) = ends(topo, id);
            a.y().abs() + b.y().abs() + a.z().abs() + b.z().abs() < 1e-9
        })
        .max_by(|&p, &q| length(p).total_cmp(&length(q)))
        .unwrap()
}

/// What both chains would take at one corner, where an `a` chamfer up a
/// vertical edge and a `b` chamfer along a bottom edge overlap:
/// `∫₀^min (a − y)(b − y) dy`.
fn corner_overlap(a: f64, b: f64) -> f64 {
    let (small, large) = if a <= b { (a, b) } else { (b, a) };
    small * small * large / 2.0 - small.powi(3) / 6.0
}

/// A chamfer chain whose open run ends on another chain's 45° stripe, in
/// either order and with either the wider or the narrower distance last.
/// The end band runs on until its plane leaves the corner: wholly onto the
/// stripe when it is the narrower one, across the stripe and into the face
/// beyond when it is the wider. The result is the block less the union of
/// the two chamfers: each chain's closed form less the two corner overlaps,
/// and the same body in both orders.
#[test]
fn chamfer_chains_close_on_each_others_stripes() {
    for (free, bottom) in [(0.7, 0.8), (0.8, 0.7), (0.8, 0.8)] {
        let mut topo = Topology::new();
        let body = rounded_free_end(&mut topo);
        let chain_of = |topo: &Topology, seed: EdgeId| {
            remus_blend::g1_chain::expand_g1_chain(topo, body, &[seed], Tolerance::new()).unwrap()
        };
        let free_chain = chain_of(&topo, free_end_seed(&topo, body));
        let bottom_chain = chain_of(&topo, long_bottom_seed(&topo, body));
        // Top edge, two end arcs, two verticals; two long edges, two feet,
        // the x = 0 edge.
        assert_eq!((free_chain.len(), bottom_chain.len()), (5, 5));
        let expected = removed_along(&topo, body, &free_chain, free)
            + removed_along(&topo, body, &bottom_chain, bottom)
            - 2.0 * corner_overlap(free, bottom);
        let before = solid_volume(&topo, body, 1e-3).unwrap();
        let source_counts = solid_entity_counts(&topo, body).unwrap();
        let mut results = Vec::new();
        for free_first in [true, false] {
            let label = format!("free {free}, bottom {bottom}, free end first: {free_first}");
            let mut solid = body;
            for free_now in [free_first, !free_first] {
                let (seed, d) = if free_now {
                    (free_end_seed(&topo, solid), free)
                } else {
                    (long_bottom_seed(&topo, solid), bottom)
                };
                let result = chamfer_v2_tangent_chain(&mut topo, solid, &[seed], d, d).unwrap();
                assert!(!result.is_partial && result.failed.is_empty(), "{label}");
                // The exact floor-chain assembly, not a boolean cutter.
                assert_eq!(result.engine, remus_blend::BlendEngine::Walking, "{label}");
                assert_exact_bands(&topo, solid, result.solid, 5, &label);
                solid = result.solid;
            }
            assert_sound(&topo, body, solid, expected, &label);
            let strict = remus_check::validate::validate_solid(
                &topo,
                solid,
                &remus_check::validate::ValidateOptions::default(),
            )
            .unwrap();
            assert!(strict.is_valid(), "{label}: {:?}", strict.issues);
            let removed = before - solid_volume(&topo, solid, 1e-3).unwrap();
            assert!(
                (removed - expected).abs() <= 1e-9 * before,
                "{label}: removed {removed}, closed form {expected}"
            );
            // Probes round the x = 30, y = 0 corner and its mirror.
            let large = free.max(bottom);
            for mirror in [false, true] {
                let at =
                    |x: f64, y: f64, z: f64| Point3::new(x, if mirror { 18.0 - y } else { y }, z);
                for (point, inside, what) in [
                    (
                        at(29.99, bottom - 0.05, 0.02),
                        false,
                        "bottom band at the end",
                    ),
                    (
                        at(30.0 - free + 0.05, 0.02, bottom + 0.05),
                        false,
                        "free-end band",
                    ),
                    (at(30.0 - large, large, large), true, "past both bands"),
                    (at(29.99, large + 0.05, 0.02), true, "free end above both"),
                    (at(30.0 - large - 0.5, 0.02, 0.02), false, "bottom band"),
                ] {
                    let expected = if inside {
                        PointClassification::Inside
                    } else {
                        PointClassification::Outside
                    };
                    assert_eq!(
                        classify(&topo, solid, point),
                        expected,
                        "{label}: {what} {point:?}"
                    );
                }
            }
            results.push(solid);
        }
        assert_eq!(
            solid_entity_counts(&topo, results[0]).unwrap(),
            solid_entity_counts(&topo, results[1]).unwrap(),
            "free {free}, bottom {bottom}: both orders"
        );
        assert_eq!(solid_entity_counts(&topo, body).unwrap(), source_counts);
    }
}
