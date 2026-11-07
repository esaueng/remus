//! A cap lowered into its shoulder past a walked corner transition.
//!
//! The Gentle-Gecko's convex top band along its x = 0 face, re-filleted
//! smaller than the R4 seam round it runs into, leaves a walked transition
//! patch between them: R4 arcs that sample the sweep but are not its
//! characteristic circles, so their radial field leans off the patch by up to
//! 7.6°. The sampled cap join solved the R1 rim's contact on that field. On
//! the patch the contact then started 0.07 mm from the exact contact on the
//! seam cylinder beside it: at R1.5 the contour hooked back over itself and
//! the lowered body's mesh folded (the closure guard refused it, as
//! `translation-invariant blend move does not close: .. 1 non-manifold
//! edges`); at R1.9 it jumped 0.09 mm and left a sliver the 0.005 display
//! mesh opened on. The contact is now solved on the patch's own normal.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::face_move_quality::{
    FaceMoveApproximation, FaceMoveQuality, move_faces_with_quality,
};
use remus_operations::measure::solid_volume;
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count, tessellate_solid,
    tessellate_solid_grouped_with_tolerance, welded_mesh_quality,
};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{face_edges, face_vertices, solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const SAMPLED: Option<FaceMoveApproximation> = Some(FaceMoveApproximation { stations: 256 });
/// The application's display deflection.
const DISPLAY: f64 = 0.005;

fn top_at(topo: &Topology, solid: SolidId, height: f64) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.dot(Vec3::new(0.0, 0.0, 1.0)) > 1.0 - 1e-9)
                && face_vertices(topo, face)
                    .unwrap()
                    .into_iter()
                    .all(|vertex| (topo.vertex(vertex).unwrap().point().z() - height).abs() < 1e-7)
        })
        .expect("planar top at requested height")
}

fn ends(topo: &Topology, edge: EdgeId) -> (Point3, Point3) {
    let data = topo.edge(edge).unwrap();
    (
        topo.vertex(data.start()).unwrap().point(),
        topo.vertex(data.end()).unwrap().point(),
    )
}

/// The sharp Gecko's two seams, shoulder and y = 0 top edge.
fn r4_edges(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    let found: Vec<EdgeId> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let (a, b) = ends(topo, edge);
            match topo.edge(edge).unwrap().curve() {
                EdgeCurve::Circle(circle) => {
                    (circle.radius() - 6.0).abs() < 1e-7
                        && (circle.center().z() - 24.0).abs() < 1e-7
                }
                EdgeCurve::Line => {
                    let vertical = (a.x() - b.x()).abs() < 1e-9 && (a.y() - b.y()).abs() < 1e-9;
                    let seam = vertical
                        && (a.x().hypot(a.y()) - 6.0).abs() < 1e-7
                        && a.z().min(b.z()) < 1e-9
                        && (a.z().max(b.z()) - 24.0).abs() < 1e-7;
                    let top = |p: Point3| (p.z() - 24.0).abs() < 1e-7;
                    let along_x = top(a) && top(b) && a.y().abs() < 1e-7 && b.y().abs() < 1e-7;
                    seam || (along_x && a.x().min(b.x()) > 1e-7)
                }
                _ => false,
            }
        })
        .collect();
    assert_eq!(found.len(), 4);
    found
}

/// The sharp Gecko with its R4 network, the x = 0 top band at `band` and the
/// R1 cap rim, its cap lowered from z36 to z26 with sampled permission.
fn lowered(band: f64) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let sharp =
        remus_io::step::reader::read_step(include_str!("data/gentle_gecko_sharp.step"), &mut topo)
            .unwrap()[0];
    let r4 = r4_edges(&topo, sharp);
    let mut solid = fillet_cascade(&mut topo, sharp, &r4, 4.0).unwrap().solid;
    let x0: Vec<EdgeId> = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Line))
        .filter(|&edge| {
            let (a, b) = ends(&topo, edge);
            [a, b]
                .iter()
                .all(|p| p.x().abs() < 1e-7 && (p.z() - 24.0).abs() < 1e-7)
        })
        .collect();
    assert_eq!(x0.len(), 1, "one x = 0 top edge");
    solid = fillet_cascade(&mut topo, solid, &x0, band).unwrap().solid;
    let cap = top_at(&topo, solid, 36.0);
    let rim = face_edges(&topo, cap).unwrap();
    solid = fillet_cascade(&mut topo, solid, &rim, 1.0).unwrap().solid;
    let cap = top_at(&topo, solid, 36.0);
    let moved = move_faces_with_quality(&mut topo, solid, &[cap], -10.0, SAMPLED)
        .unwrap_or_else(|error| panic!("band R{band}: the cap does not lower: {error}"));
    assert_eq!(
        moved.quality,
        FaceMoveQuality::SampledCapJoin { stations: 256 }
    );
    (topo, moved.result.solid)
}

fn mesh_volume(mesh: &TriangleMesh) -> f64 {
    let origin = mesh.positions[0];
    mesh.indices
        .chunks(3)
        .map(|tri| {
            let a = mesh.positions[tri[0] as usize] - origin;
            let b = mesh.positions[tri[1] as usize] - origin;
            let c = mesh.positions[tri[2] as usize] - origin;
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

/// Both validators, closed manifold meshes at two measuring deflections and
/// at the display deflection (ungrouped and grouped, as the WASM binding
/// meshes), and the API volume against the independent mesh volume.
fn assert_closed(topo: &Topology, solid: SolidId, label: &str) {
    let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {:?}", report.issues);
    let strict = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(strict.is_valid(), "{label}: {:?}", strict.issues);
    let volume = solid_volume(topo, solid, 5e-4).unwrap();
    for deflection in [2.5e-3, 1e-3] {
        let mesh = tessellate_solid(topo, solid, deflection).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "{label} at {deflection}");
        assert_eq!(non_manifold_edge_count(&mesh), 0, "{label} at {deflection}");
        assert!(
            welded_mesh_quality(&mesh).is_watertight(),
            "{label} at {deflection}"
        );
        let independent = mesh_volume(&mesh);
        assert!(
            (independent - volume).abs() <= 1e-4 * volume,
            "{label}: mesh volume {independent} vs {volume} at {deflection}"
        );
    }
    let display = tessellate_solid(topo, solid, DISPLAY).unwrap();
    assert!(
        welded_mesh_quality(&display).is_watertight(),
        "{label} at display {DISPLAY}: {:?}",
        welded_mesh_quality(&display)
    );
    let (grouped, _) = tessellate_solid_grouped_with_tolerance(
        topo,
        solid,
        DISPLAY,
        remus_math::chord::DEFAULT_ANGULAR_TOL,
    )
    .unwrap();
    assert!(
        welded_mesh_quality(&grouped).is_watertight(),
        "{label} grouped at display {DISPLAY}: {:?}",
        welded_mesh_quality(&grouped)
    );
}

/// The rim's contacts on the sampled corner patches leave their ends
/// without turning back: along the first quarter of each contact, the
/// distance from its start vertex only grows.
fn assert_contacts_do_not_hook(topo: &Topology, solid: SolidId, label: &str) {
    let adjacency = topo.build_adjacency(solid).unwrap();
    let sampled =
        |face: FaceId| matches!(topo.face(face).unwrap().surface(), FaceSurface::Nurbs(_));
    let mut checked = 0;
    for edge in solid_edges(topo, solid).unwrap() {
        let faces = adjacency.faces_for_edge(edge);
        if faces.len() != 2 || !faces.iter().all(|&face| sampled(face)) {
            continue;
        }
        let data = topo.edge(edge).unwrap();
        let (start, end) = ends(topo, edge);
        let (t0, t1) = data.strict_domain().unwrap();
        for (from, sign) in [(start, 1.0), (end, -1.0)] {
            let mut reach = 0.0_f64;
            for i in 1..=100 {
                let fraction = f64::from(i) / 400.0;
                let t = if sign > 0.0 {
                    t0 + (t1 - t0) * fraction
                } else {
                    t1 - (t1 - t0) * fraction
                };
                let point = data.curve().evaluate_with_endpoints(t, start, end);
                let distance = (point - from).length();
                assert!(
                    distance >= reach - 1e-9,
                    "{label}: contact edge {} turns back {} mm from its end",
                    edge.index(),
                    reach - distance
                );
                reach = distance;
            }
        }
        checked += 1;
    }
    assert!(checked >= 2, "{label}: {checked} sampled contacts");
}

#[test]
fn cap_lowers_past_the_walked_corner_of_a_smaller_band() {
    for band in [1.5, 1.9] {
        let label = format!("band R{band}");
        let (topo, solid) = lowered(band);
        assert_closed(&topo, solid, &label);
        assert_contacts_do_not_hook(&topo, solid, &label);
        // Planes, cylinders and tori stay analytic: the sampled faces are the
        // two corner patches and the two rim bands rolled over them.
        let sampled = solid_faces(&topo, solid)
            .unwrap()
            .into_iter()
            .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Nurbs(_)))
            .count();
        assert_eq!(sampled, 4, "{label}");
    }
}
