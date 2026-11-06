//! Smaller convex block fillets running into an earlier R4 seam band keep
//! their straight run as an exact cylinder.
//!
//! The sharp Gentle-Gecko body is a 30 x 18 x 24 block with an r6 boss on its
//! corner rising to z36. Its boss seams and shoulder are filleted R4 first;
//! the block's two top edges at the boss then run into that band, and a
//! convex fillet of them (R1.5 or R1.9, as the recording shrinks them) follows
//! the ridgeline up the band (the ridge-chain family of
//! `crates/operations/tests/fillet_into_seam_band.rs`). Along the straight
//! edge the ball touches two planes, so that span is exactly a cylinder of
//! the fillet's radius; only the transition where the ball climbs the band is
//! walked. The whole band used to be stored as one NURBS surface.
//!
//! Each body is held to the solid-verification bar: both validators, a closed
//! shell, meshes at two deflections finer than the volume clamp with no
//! boundary or non-manifold edges, API volume against the independently summed
//! mesh volume, and point classification on both sides of each fillet.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use remus_io::step::reader::read_step;
use remus_math::vec::Point3;
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::measure;
use remus_operations::query::filter_filletable_edges;
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

/// API volumes of the complete targets (with the R1 cap rim) when the convex
/// bands were stored as whole NURBS bands, at kernel f4b5d747
/// (`docs/video-workflow-acceptance.md`, W3, fine deflection).
const R4_VOLUME: f64 = 16_339.872_377;
const R15_VOLUME: f64 = 16_451.495_932;
const R19_VOLUME: f64 = 16_441.371_447;

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-5
}

fn endpoints(topo: &Topology, edge: EdgeId) -> (Point3, Point3) {
    let edge = topo.edge(edge).unwrap();
    (
        topo.vertex(edge.start()).unwrap().point(),
        topo.vertex(edge.end()).unwrap().point(),
    )
}

/// The surviving sharp support intersections of one kind, re-identified on
/// the current body.
fn network_edges(topo: &Topology, solid: SolidId, kind: &str) -> Vec<EdgeId> {
    let adjacency = AdjacencyIndex::build(topo, solid).unwrap();
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let &[a, b] = adjacency.faces_for_edge(edge) else {
                return false;
            };
            let a = topo.face(a).unwrap().surface();
            let b = topo.face(b).unwrap().surface();
            let (p, q) = endpoints(topo, edge);
            let cylinder_plane = matches!(
                (a, b),
                (FaceSurface::Cylinder(_), FaceSurface::Plane { .. })
                    | (FaceSurface::Plane { .. }, FaceSurface::Cylinder(_))
            );
            let plane_plane = matches!(
                (a, b),
                (FaceSurface::Plane { .. }, FaceSurface::Plane { .. })
            );
            let on_top = near(p.z(), 24.0) && near(q.z(), 24.0);
            match kind {
                "seam" => cylinder_plane && (p.z() - q.z()).abs() > 1.0,
                "shoulder" => cylinder_plane && on_top,
                "round" => {
                    plane_plane
                        && on_top
                        && ((near(p.y(), 0.0) && near(q.y(), 0.0))
                            || (near(p.x(), 0.0) && near(q.x(), 0.0)))
                }
                _ => false,
            }
        })
        .collect()
}

fn fillet(topo: &mut Topology, solid: SolidId, edges: &[EdgeId], radius: f64) -> SolidId {
    let edges = filter_filletable_edges(topo, solid, edges).unwrap();
    assert!(!edges.is_empty(), "no filletable edges for R{radius}");
    let result = fillet_cascade(topo, solid, &edges, radius)
        .unwrap_or_else(|e| panic!("R{radius} fillet: {e}"));
    assert!(result.failed.is_empty(), "R{radius}: partial blend");
    result.solid
}

/// The top cap's face.
fn cap(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(topo.face(face).unwrap().surface(),
                FaceSurface::Plane { normal, d } if normal.z().abs() > 0.999 && near(d / normal.z(), 36.0))
        })
        .unwrap()
}

/// The sharp Gecko with its R4 seams and shoulder, the convex top edges at
/// `convex`, and the R1 cap rim.
fn build(convex: f64) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let mut solid = read_step(include_str!("data/Gentle-Gecko.step"), &mut topo).unwrap()[0];
    if near(convex, 4.0) {
        let mut edges = network_edges(&topo, solid, "seam");
        edges.extend(network_edges(&topo, solid, "shoulder"));
        edges.extend(network_edges(&topo, solid, "round"));
        solid = fillet(&mut topo, solid, &edges, 4.0);
    } else {
        let mut edges = network_edges(&topo, solid, "seam");
        edges.extend(network_edges(&topo, solid, "shoulder"));
        solid = fillet(&mut topo, solid, &edges, 4.0);
        let round = network_edges(&topo, solid, "round");
        assert_eq!(round.len(), 2, "the block's two top edges at the boss");
        solid = fillet(&mut topo, solid, &round, convex);
    }
    let rim: Vec<EdgeId> = topo
        .wire(topo.face(cap(&topo, solid)).unwrap().outer_wire())
        .unwrap()
        .edges()
        .iter()
        .map(remus_topology::OrientedEdge::edge)
        .collect();
    solid = fillet(&mut topo, solid, &rim, 1.0);
    (topo, solid)
}

#[derive(Debug, Default, PartialEq, Eq)]
struct Census {
    planes: usize,
    cylinders: usize,
    tori: usize,
    nurbs: usize,
    other: usize,
}

fn census(topo: &Topology, solid: SolidId) -> Census {
    let mut census = Census::default();
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => census.planes += 1,
            FaceSurface::Cylinder(_) => census.cylinders += 1,
            FaceSurface::Torus(_) => census.tori += 1,
            FaceSurface::Nurbs(_) => census.nurbs += 1,
            _ => census.other += 1,
        }
    }
    census
}

/// The cylinders of radius `radius` whose axes run along the block's top
/// edges: x for the front edge at y = radius, y for the side edge at
/// x = radius, both at z = 24 - radius.
fn straight_spans(topo: &Topology, solid: SolidId, radius: f64) -> [usize; 2] {
    let mut spans = [0, 0];
    for face in solid_faces(topo, solid).unwrap() {
        let FaceSurface::Cylinder(cylinder) = topo.face(face).unwrap().surface() else {
            continue;
        };
        if !near(cylinder.radius(), radius) {
            continue;
        }
        let (axis, origin) = (cylinder.axis(), cylinder.origin());
        if axis.x().abs() > 1.0 - 1e-12 && near(origin.y(), radius) {
            assert!(near(origin.z(), 24.0 - radius), "front axis at {origin:?}");
            spans[0] += 1;
        } else if axis.y().abs() > 1.0 - 1e-12 && near(origin.x(), radius) {
            assert!(near(origin.z(), 24.0 - radius), "side axis at {origin:?}");
            spans[1] += 1;
        }
    }
    spans
}

/// The largest angle between a walked transition's normal along the circle
/// it starts from (its first section, `v = 0`) and the normal of the
/// straight run's cylinder there, over both transitions; also checks that
/// circle lies on that cylinder.
fn split_kink(topo: &Topology, solid: SolidId, radius: f64) -> f64 {
    let mut kink: f64 = 0.0;
    let mut transitions = 0;
    for face in solid_faces(topo, solid).unwrap() {
        let FaceSurface::Nurbs(band) = topo.face(face).unwrap().surface() else {
            continue;
        };
        // The front transition's run lies along x at y = radius, the
        // side's along y at x = radius, both at z = 24 - radius.
        let start = band.evaluate(0.5, 0.0);
        let axis_offset = |p: Point3| {
            if start.x() > start.y() {
                (p.y() - radius, p.z() - (24.0 - radius))
            } else {
                (p.x() - radius, p.z() - (24.0 - radius))
            }
        };
        transitions += 1;
        for i in 0..=20 {
            let u = f64::from(i) / 20.0;
            let p = band.evaluate(u, 0.0);
            let (a, b) = axis_offset(p);
            let off = a.hypot(b);
            assert!(
                (off - radius).abs() < 1e-9,
                "transition start {p:?} is {off} from the run's axis"
            );
            let radial = if start.x() > start.y() {
                remus_math::vec::Vec3::new(0.0, a, b)
            } else {
                remus_math::vec::Vec3::new(a, 0.0, b)
            } * (1.0 / off);
            let normal = band.normal(u, 0.0).unwrap();
            kink = kink.max(normal.dot(radial).abs().clamp(-1.0, 1.0).acos());
        }
    }
    assert_eq!(transitions, 2, "two walked transitions");
    kink
}

fn classify(
    topo: &Topology,
    solid: SolidId,
    at: Point3,
) -> remus_check::classify::PointClassification {
    remus_check::classify::classify_point(
        topo,
        solid,
        at,
        &remus_check::classify::ClassifyOptions::default(),
    )
    .unwrap()
}

/// The solid-verification bar; returns the API volume at the finer
/// deflection.
fn verify(topo: &Topology, solid: SolidId, convex: f64, label: &str) -> f64 {
    use remus_check::classify::PointClassification as PointClass;

    let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {:?}", report.issues);
    let strict = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(strict.is_valid(), "{label}: {:?}", strict.issues);
    let shell = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    assert!(
        remus_topology::validation::validate_shell_closed(shell, topo).is_ok(),
        "{label}: open shell"
    );
    let adjacency = AdjacencyIndex::build(topo, solid).unwrap();
    assert!(adjacency.boundary_edges().is_empty(), "{label}: free edges");
    assert!(
        adjacency.non_manifold_edges().is_empty(),
        "{label}: non-manifold edges"
    );

    let bbox = measure::solid_bounding_box(topo, solid).unwrap();
    let diagonal = (bbox.max - bbox.min).length();
    let mut volumes = Vec::new();
    // Both strictly finer than solid_volume's bbox-diagonal * 5e-5 clamp.
    for deflection in [diagonal * 1e-5, diagonal * 5e-6] {
        let api = measure::solid_volume(topo, solid, deflection).unwrap();
        let mesh = remus_operations::tessellate::tessellate_solid_with_tolerance(
            topo, solid, deflection, 0.15,
        )
        .unwrap();
        let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
        assert_eq!(quality.boundary_edges, 0, "{label}: mesh boundary edges");
        assert_eq!(
            quality.non_manifold_edges, 0,
            "{label}: mesh non-manifold edges"
        );
        let origin = bbox.min;
        let summed: f64 = mesh
            .indices
            .chunks_exact(3)
            .map(|ix| {
                let a = mesh.positions[ix[0] as usize] - origin;
                let b = mesh.positions[ix[1] as usize] - origin;
                let c = mesh.positions[ix[2] as usize] - origin;
                a.dot(b.cross(c)) / 6.0
            })
            .sum();
        assert!(
            (api - summed).abs() <= 1e-4 * api,
            "{label}: API volume {api} against mesh volume {summed} at {deflection}"
        );
        volumes.push(api);
    }
    assert!(
        (volumes[0] - volumes[1]).abs() <= 1e-4 * volumes[1],
        "{label}: volume moves with deflection: {volumes:?}"
    );

    // Inside and outside each convex band's straight run, in its normal
    // plane: a point 0.2 in from both faces of the sharp edge is cut away
    // for every tested radius; one 0.9 r from the band's axis towards the
    // edge remains.
    let p = Point3::new;
    let r = convex;
    let kept = r - 0.9 * r * std::f64::consts::FRAC_1_SQRT_2;
    let probes = [
        ("block core", p(20.0, 9.0, 12.0), PointClass::Inside),
        ("outside block", p(35.0, 9.0, 12.0), PointClass::Outside),
        ("boss", p(0.0, 0.0, 30.0), PointClass::Inside),
        ("above cap", p(0.0, 0.0, 36.5), PointClass::Outside),
        ("front corner cut", p(20.0, 0.2, 23.8), PointClass::Outside),
        (
            "front band inside",
            p(20.0, kept, 24.0 - kept),
            PointClass::Inside,
        ),
        ("side corner cut", p(0.2, 15.0, 23.8), PointClass::Outside),
        (
            "side band inside",
            p(kept, 15.0, 24.0 - kept),
            PointClass::Inside,
        ),
        (
            "front axis",
            p(25.0, r, 24.0 - r - 0.1 * r),
            PointClass::Inside,
        ),
    ];
    for (name, at, expected) in probes {
        assert_eq!(classify(topo, solid, at), expected, "{label}: {name}");
    }
    volumes[1]
}

#[test]
fn convex_ridge_fillets_keep_their_straight_runs_as_cylinders() {
    for (convex, reference) in [(1.5, R15_VOLUME), (1.9, R19_VOLUME)] {
        let label = format!("convex R{convex}");
        let (topo, solid) = build(convex);
        let volume = verify(&topo, solid, convex, &label);
        // Each straight run is one exact cylinder of the fillet's radius
        // on the edge's axis; only the two transitions up the band are
        // walked.
        assert_eq!(
            straight_spans(&topo, solid, convex),
            [1, 1],
            "{label}: straight runs are not exact cylinders: {:?}",
            census(&topo, solid)
        );
        assert_eq!(
            census(&topo, solid),
            Census {
                planes: 7,
                cylinders: 5,
                tori: 2,
                nurbs: 2,
                other: 0
            },
            "{label}"
        );
        // The walked transition starts on the run's end circle and meets
        // the cylinder there with no more turn than the walked band allows
        // between its own stations (2 degrees).
        let kink = split_kink(&topo, solid, convex);
        println!("{label}: normal turn across the split {kink:e} rad, volume {volume}");
        assert!(kink < 0.035, "{label}: kink {kink} across the split");
        // The same material as the whole walked band removed.
        assert!(
            (volume - reference).abs() <= 2e-6 * reference,
            "{label}: volume {volume}, whole-NURBS band {reference}"
        );
    }
}

#[test]
fn all_r4_network_is_unchanged() {
    let (topo, solid) = build(4.0);
    let volume = verify(&topo, solid, 4.0, "all R4");
    assert_eq!(
        census(&topo, solid),
        Census {
            planes: 7,
            cylinders: 5,
            tori: 2,
            nurbs: 2,
            other: 0
        }
    );
    assert!(
        (volume - R4_VOLUME).abs() <= 1e-6 * R4_VOLUME,
        "all R4: volume {volume}"
    );
}
