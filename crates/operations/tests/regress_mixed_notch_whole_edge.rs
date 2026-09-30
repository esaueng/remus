//! Pinned regression: whole-edge fillet of the concave L-bracket (slice 1).
//!
//! Fixture (exact task dimensions): closed XY polygon
//! `[(0,0),(40,0),(40,8),(8,8),(8,50),(0,50)]` extruded 20 mm along +Z,
//! all 18 sharp physical edges selected at radius 1 mm and 2 mm through the
//! public cascade (`fillet_cascade`, the engine chain behind the WASM
//! `fillet` binding).
//!
//! Actual cause (rechecked 2026-09-30 on origin/main `594cd308`): the two
//! notch vertices `(8,8,0)` and `(8,8,20)` are mixed-side 3-way planar
//! junctions (two convex edges + one concave edge) for which no qualified
//! corner patch exists yet: the rolling-ball engine's exact corner ball
//! requires one connected material-side orientation
//! (`exact_planar_corner_ball` returns `None` on alternating sides) and the
//! walking builder has no watertight assembly for multi-chain vertices, so
//! the call fails with `unsupported-vertex-blend`. The visible
//! "2 stripes meet" text is the walking-builder guard's hardcoded message
//! (`fillet_builder.rs`), not a geometric classification: the failing
//! junctions each join THREE selected edges.
//!
//! The independent boundary oracle (`oracle_mixed_notch_boundary.rs`) maps
//! the true surface (slivers, lens, ball-tube junctions, station
//! crossings); the corner family remains unqualified until an implementation
//! lands. Slice 2 works toward that patch.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_check::validate::{ValidateOptions, validate_solid};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::{blend_failure_code, fillet_cascade};
use remus_operations::extrude::extrude;
use remus_operations::measure::solid_volume;
use remus_operations::query::{EdgeConcavity, edge_concavity, filter_filletable_edges};
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::vertex::VertexId;

const NOTCH: (f64, f64) = (8.0, 8.0);
const HEIGHT: f64 = 20.0;

/// The exact task fixture.
fn l_bracket(topo: &mut Topology) -> SolidId {
    let profile = make_polygon_wire(
        topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(40.0, 0.0, 0.0),
            Point3::new(40.0, 8.0, 0.0),
            Point3::new(NOTCH.0, NOTCH.1, 0.0),
            Point3::new(8.0, 50.0, 0.0),
            Point3::new(0.0, 50.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let face = topo.add_face(remus_topology::face::Face::new(
        profile,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), HEIGHT).unwrap()
}

fn is_notch_vertex(topo: &Topology, vid: VertexId) -> bool {
    let p = topo.vertex(vid).unwrap().point();
    (p.x() - NOTCH.0).abs() < 1e-9
        && (p.y() - NOTCH.1).abs() < 1e-9
        && (p.z() < 1e-9 || (p.z() - HEIGHT).abs() < 1e-9)
}

/// Junction classification for one vertex over the selected edge set.
#[derive(Debug)]
struct Junction {
    vertex: VertexId,
    position: Point3,
    convex: Vec<EdgeId>,
    concave: Vec<EdgeId>,
    other: Vec<EdgeId>,
}

fn classify_selection_junctions(
    topo: &Topology,
    solid: SolidId,
    selected: &[EdgeId],
) -> HashMap<usize, Junction> {
    let selected_set: std::collections::HashSet<usize> =
        selected.iter().map(|e| e.index()).collect();
    let mut junctions: HashMap<usize, Junction> = HashMap::new();
    for eid in solid_edges(topo, solid).unwrap() {
        if !selected_set.contains(&eid.index()) {
            continue;
        }
        let edge = topo.edge(eid).unwrap();
        let concavity = edge_concavity(topo, solid, eid, 0.1).unwrap();
        for vid in [edge.start(), edge.end()] {
            let entry = junctions.entry(vid.index()).or_insert_with(|| Junction {
                vertex: vid,
                position: topo.vertex(vid).unwrap().point(),
                convex: vec![],
                concave: vec![],
                other: vec![],
            });
            match concavity {
                EdgeConcavity::Convex => entry.convex.push(eid),
                EdgeConcavity::Concave => entry.concave.push(eid),
                EdgeConcavity::Tangent | EdgeConcavity::Unknown => entry.other.push(eid),
            }
        }
    }
    junctions
}

#[derive(Debug, PartialEq)]
struct Fingerprint {
    faces: usize,
    edges: usize,
    surface_tags: Vec<String>,
    volume_nano: i128,
    valid: bool,
}

fn fingerprint(topo: &Topology, solid: SolidId) -> Fingerprint {
    let volume = solid_volume(topo, solid, 0.05).unwrap();
    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    let mut faces = solid_faces(topo, solid).unwrap();
    faces.sort_by_key(|f| f.index());
    let surface_tags: Vec<String> = faces
        .iter()
        .map(|f| match topo.face(*f).unwrap().surface() {
            FaceSurface::Plane { .. } => "Plane".to_string(),
            FaceSurface::Cylinder(_) => "Cylinder".to_string(),
            FaceSurface::Sphere(_) => "Sphere".to_string(),
            FaceSurface::Nurbs(_) => "Nurbs".to_string(),
            FaceSurface::Cone(_) => "Cone".to_string(),
            FaceSurface::Torus(_) => "Torus".to_string(),
        })
        .collect();
    Fingerprint {
        faces: faces.len(),
        edges: solid_edges(topo, solid).unwrap().len(),
        surface_tags,
        #[allow(clippy::cast_possible_truncation)]
        volume_nano: (volume * 1e9).round() as i128,
        valid: report.is_valid(),
    }
}

/// The whole physical edge set: 18 sharp edges, 17 convex + 1 concave.
#[test]
fn whole_edge_selection_is_complete_and_mixed() {
    let mut topo = Topology::new();
    let solid = l_bracket(&mut topo);
    let all = solid_edges(&topo, solid).unwrap();
    assert_eq!(all.len(), 18, "extruded hexagon must have 18 edges");
    let physical = filter_filletable_edges(&topo, solid, &all).unwrap();
    assert_eq!(
        physical.len(),
        18,
        "no seams or smooth subdivisions: every edge is physical and eligible"
    );

    let junctions = classify_selection_junctions(&topo, solid, &physical);
    let mut convex_total = 0;
    let mut concave_total = 0;
    let mut mixed_notch = 0;
    for junction in junctions.values() {
        convex_total += junction.convex.len();
        concave_total += junction.concave.len();
        assert!(
            junction.other.is_empty(),
            "every selected edge must classify convex/concave, got {:?} at {:?}",
            junction.other,
            junction.position
        );
        if is_notch_vertex(&topo, junction.vertex) {
            assert_eq!(
                (junction.convex.len(), junction.concave.len()),
                (2, 1),
                "notch vertex at {:?} must join two convex + one concave selected edge",
                junction.position
            );
            mixed_notch += 1;
        } else {
            assert_eq!(
                junction.concave.len(),
                0,
                "non-notch vertex at {:?} must be all-convex",
                junction.position
            );
            assert_eq!(
                junction.convex.len(),
                3,
                "extruded-polygon vertices join three selected edges"
            );
        }
    }
    // Each edge counted twice (both endpoints).
    assert_eq!((convex_total, concave_total), (34, 2));
    assert_eq!(mixed_notch, 2, "top and bottom notch vertices");
}

/// Oversized radii must also reject without touching the input: r=5 already
/// overruns the 8 mm wall (adjacent bands collide), r=50 exceeds the part.
#[test]
fn oversized_whole_edge_fillet_rejects_unchanged() {
    for radius in [5.0_f64, 50.0] {
        let mut topo = Topology::new();
        let solid = l_bracket(&mut topo);
        let all = solid_edges(&topo, solid).unwrap();
        let physical = filter_filletable_edges(&topo, solid, &all).unwrap();
        let before = fingerprint(&topo, solid);
        assert!(before.valid);

        assert!(
            fillet_cascade(&mut topo, solid, &physical, radius).is_err(),
            "r={radius}: oversized whole-edge fillet must be rejected"
        );
        assert_eq!(
            fingerprint(&topo, solid),
            before,
            "r={radius}: rejected fillet must leave the input bit-identical"
        );
    }
}

/// The refusal is scale-invariant: a 10x bracket at 10x radius refuses the
/// same way (same code, intact input).
#[test]
fn whole_edge_refusal_scales_with_geometry() {
    let mut topo = Topology::new();
    let solid = l_bracket(&mut topo);
    let scale = Mat4::scale(10.0, 10.0, 10.0);
    remus_operations::transform::transform_solid(&mut topo, solid, &scale).unwrap();
    let all = solid_edges(&topo, solid).unwrap();
    let physical = filter_filletable_edges(&topo, solid, &all).unwrap();
    assert_eq!(physical.len(), 18);
    let before = fingerprint(&topo, solid);

    let err = match fillet_cascade(&mut topo, solid, &physical, 10.0) {
        Ok(result) => panic!(
            "10x bracket at r=10 must still refuse; built engine={:?}",
            result.engine
        ),
        Err(e) => e,
    };
    assert_eq!(blend_failure_code(&err), "unsupported-vertex-blend");
    assert_eq!(fingerprint(&topo, solid), before);
}
#[test]
fn whole_edge_fillet_refuses_typed_and_leaves_input_intact() {
    for radius in [1.0_f64, 2.0] {
        let mut topo = Topology::new();
        let solid = l_bracket(&mut topo);
        let all = solid_edges(&topo, solid).unwrap();
        let physical = filter_filletable_edges(&topo, solid, &all).unwrap();
        let before = fingerprint(&topo, solid);
        assert!(before.valid, "fixture must start valid: {before:?}");
        assert_eq!((before.faces, before.edges), (8, 18));

        let err = match fillet_cascade(&mut topo, solid, &physical, radius) {
            Ok(result) => panic!(
                "r={radius}: slice 1 expects refusal (slice 2 implements the mixed-side patch); \
                 unexpectedly built engine={:?}",
                result.engine
            ),
            Err(e) => e,
        };
        assert_eq!(
            blend_failure_code(&err),
            "unsupported-vertex-blend",
            "r={radius}: must fail with the vertex-blend code, got: {err}"
        );
        assert_eq!(
            fingerprint(&topo, solid),
            before,
            "r={radius}: rejected fillet must leave the input bit-identical"
        );
    }
}
