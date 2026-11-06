//! Acceptance row A6 on the supplied Gentle-Gecko part: a 0.8 chamfer picked
//! on one bottom edge runs the mixed bottom outline — straight block edges,
//! the R4 seam rounds' concave feet and the R6 boss's foot — and insets the
//! bottom face.
//!
//! The STEP files stay outside the repository, as the workflow baseline keeps
//! them. Set `REMUS_VIDEO_FIXTURES` to their directory and run:
//! `cargo test -p remus-io --test gecko_bottom_chamfer -- --ignored`.
//! CI covers the same outline on a native rebuild of the part's bottom
//! (`crates/operations/tests/chamfer_floor_chain.rs`).

#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout,
    clippy::unwrap_used
)]

use std::path::PathBuf;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_io::step::reader::read_step;
use remus_math::vec::Point3;
use remus_operations::blend_ops::{chamfer_v2, chamfer_v2_tangent_chain};
use remus_operations::measure::{solid_bounding_box, solid_volume};
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::validation::{validate_shell_closed, validate_shell_manifold};

const D: f64 = 0.8;

fn gecko(topo: &mut Topology) -> SolidId {
    let dir = PathBuf::from(
        std::env::var_os("REMUS_VIDEO_FIXTURES")
            .expect("set REMUS_VIDEO_FIXTURES to the Gentle-Gecko fixture directory"),
    );
    let text = std::fs::read_to_string(dir.join("Gentle-Gecko-W-Fillet.step")).unwrap();
    let solids = read_step(&text, topo).unwrap();
    assert_eq!(solids.len(), 1);
    solids[0]
}

fn ends(topo: &Topology, edge: EdgeId) -> (Point3, Point3) {
    let data = topo.edge(edge).unwrap();
    (
        topo.vertex(data.start()).unwrap().point(),
        topo.vertex(data.end()).unwrap().point(),
    )
}

fn bottom_outline(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    let face = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| {
            matches!(topo.face(f).unwrap().surface(),
                FaceSurface::Plane { normal, d } if normal.z().abs() > 0.999 && d.abs() < 1e-9)
        })
        .unwrap();
    let face = topo.face(face).unwrap();
    topo.wire(face.outer_wire())
        .unwrap()
        .edges()
        .iter()
        .map(remus_topology::wire::OrientedEdge::edge)
        .collect()
}

/// Closed form along the floor: section `d²/2` times each line, and swept
/// about each arc's axis at the section's centroid radius (Pappus).
fn removed_along(topo: &Topology, solid: SolidId, edges: &[EdgeId]) -> f64 {
    let section = 0.5 * D * D;
    let adjacency = topo.build_adjacency(solid).unwrap();
    edges
        .iter()
        .map(|&edge| {
            let data = topo.edge(edge).unwrap();
            let (a, b) = ends(topo, edge);
            match data.curve() {
                EdgeCurve::Line => section * (b - a).length(),
                EdgeCurve::Circle(circle) => {
                    let (t0, t1) = data.strict_domain().unwrap();
                    let wall = adjacency
                        .faces_for_edge(edge)
                        .iter()
                        .copied()
                        .find(|&f| {
                            matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_))
                        })
                        .unwrap();
                    let concave = topo.face(wall).unwrap().is_reversed();
                    let centroid = if concave {
                        circle.radius() + D / 3.0
                    } else {
                        circle.radius() - D / 3.0
                    };
                    section * (t1 - t0).abs() * centroid
                }
                other => panic!("unexpected bottom edge {}", other.type_tag()),
            }
        })
        .sum()
}

fn mesh_volume(mesh: &TriangleMesh) -> f64 {
    let origin = mesh.positions[0];
    mesh.indices
        .chunks_exact(3)
        .map(|t| {
            let [a, b, c] = [t[0], t[1], t[2]].map(|i| mesh.positions[i as usize] - origin);
            a.dot(b.cross(c))
        })
        .sum::<f64>()
        / 6.0
}

fn census(topo: &Topology, solid: SolidId) -> std::collections::BTreeMap<&'static str, usize> {
    let mut census = std::collections::BTreeMap::new();
    for f in solid_faces(topo, solid).unwrap() {
        *census
            .entry(topo.face(f).unwrap().surface().type_tag())
            .or_insert(0) += 1;
    }
    census
}

fn verify(
    topo: &Topology,
    input: SolidId,
    result: SolidId,
    expected: f64,
    bands: usize,
    label: &str,
) {
    let shell = topo
        .shell(topo.solid(result).unwrap().outer_shell())
        .unwrap();
    validate_shell_closed(shell, topo).unwrap();
    validate_shell_manifold(shell, topo).unwrap();
    let report = validate_solid(topo, result).unwrap();
    assert!(report.is_valid(), "{label}: {:?}", report.issues);

    let bounds = solid_bounding_box(topo, input).unwrap();
    let diagonal = (bounds.max - bounds.min).length();
    let before = solid_volume(topo, input, diagonal * 1e-5).unwrap();
    let mut api = Vec::new();
    for deflection in [diagonal * 4e-5, diagonal * 1e-5] {
        let mesh = tessellate_solid(topo, result, deflection).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "{label} @{deflection}");
        assert_eq!(non_manifold_edge_count(&mesh), 0, "{label} @{deflection}");
        let volume = solid_volume(topo, result, deflection).unwrap();
        let independent = mesh_volume(&mesh);
        assert!(
            (volume - independent).abs() <= 1e-4 * volume,
            "{label}: API {volume} vs mesh {independent}"
        );
        api.push(volume);
    }
    assert!((api[0] - api[1]).abs() <= 1e-4 * api[1], "{label}: {api:?}");
    let removed = before - api[1];
    println!("{label}: removed {removed:.6}, closed form {expected:.6}");
    assert!(
        (removed - expected).abs() <= 2e-3 * expected,
        "{label}: removed {removed}, closed form {expected}"
    );

    // Exact bands only; the part's two freeform corner patches are its own.
    let (before_census, after_census) = (census(topo, input), census(topo, result));
    assert_eq!(
        before_census.get("nurbs"),
        after_census.get("nurbs"),
        "{label}"
    );
    let created: Vec<FaceId> = solid_faces(topo, result)
        .unwrap()
        .into_iter()
        .filter(|f| !solid_faces(topo, input).unwrap().contains(f))
        .collect();
    let new_faces =
        solid_entity_counts(topo, result).unwrap().0 - solid_entity_counts(topo, input).unwrap().0;
    assert_eq!(new_faces, bands, "{label}: {after_census:?}");
    assert!(created.iter().all(|&f| matches!(
        topo.face(f).unwrap().surface(),
        FaceSurface::Plane { .. } | FaceSurface::Cone(_) | FaceSurface::Cylinder(_)
    )));

    // Probes: the wedge under a straight edge, the boss foot and a round's
    // foot are removed; just past them is material.
    let probe = |p: Point3| classify_point(topo, result, p, &ClassifyOptions::default()).unwrap();
    let boss = -std::f64::consts::FRAC_PI_2 - 0.6;
    for (edge_point, inward) in [
        (Point3::new(20.0, 0.0, 0.0), (0.0, 1.0)),
        (
            Point3::new(6.0 * boss.cos(), 6.0 * boss.sin(), 0.0),
            (-boss.cos(), -boss.sin()),
        ),
    ] {
        let at = |r: f64, z: f64| {
            Point3::new(
                edge_point.x() + inward.0 * r,
                edge_point.y() + inward.1 * r,
                z,
            )
        };
        assert_eq!(probe(at(0.2, 0.2)), PointClassification::Outside, "{label}");
        assert_eq!(probe(at(0.6, 0.6)), PointClassification::Inside, "{label}");
        assert_eq!(probe(at(1.2, 0.1)), PointClassification::Inside, "{label}");
    }
}

#[test]
#[ignore = "needs REMUS_VIDEO_FIXTURES"]
fn a6_bottom_chamfer_runs_the_tangent_chain() {
    let mut topo = Topology::new();
    let body = gecko(&mut topo);
    let seed = bottom_outline(&topo, body)
        .into_iter()
        .find(|&e| {
            let (a, b) = ends(&topo, e);
            a.y().abs() + b.y().abs() < 1e-9 && (a.x() - b.x()).abs() > 10.0
        })
        .unwrap();
    let chain = remus_blend::g1_chain::expand_g1_chain(
        &topo,
        body,
        &[seed],
        remus_math::tolerance::Tolerance::new(),
    )
    .unwrap();
    assert_eq!(chain.len(), 7, "lines, round feet and the boss foot");
    let expected = removed_along(&topo, body, &chain);
    let source = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
    let result = chamfer_v2_tangent_chain(&mut topo, body, &[seed], D, D).unwrap();
    verify(&topo, body, result.solid, expected, 5, "A6 chain");
    assert_eq!(
        remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
        source
    );
}

#[test]
#[ignore = "needs REMUS_VIDEO_FIXTURES"]
fn a6_bottom_chamfer_runs_the_whole_outline() {
    let mut topo = Topology::new();
    let body = gecko(&mut topo);
    let outline = bottom_outline(&topo, body);
    // Three convex block corners, mitred: the corner overlap counts once.
    let expected = removed_along(&topo, body, &outline) - D * D * D;
    let result = chamfer_v2(&mut topo, body, &outline, D, D).unwrap();
    verify(&topo, body, result.solid, expected, 7, "A6 outline");
}
