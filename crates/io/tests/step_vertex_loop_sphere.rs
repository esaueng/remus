//! A whole sphere written as one `ADVANCED_FACE` bounded by a `VERTEX_LOOP`
//! imports as the native two-hemisphere sphere (OpenZCAD #1245).
//!
//! Independent AP214 producers can write a radius-10 sphere as a single
//! spherical face whose only bound is a pole VERTEX_LOOP. The fixture
//! reproduces the reported 32-entity layout, including #19/#20. Its generic
//! header is normalized; the geometric entity data is preserved.
//!
//! Any other vertex-loop layout is refused with a typed `UnsupportedEntity`
//! naming the loop, and every refusal leaves the target arena untouched.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::PI;

use remus_io::step::reader::{read_step, read_step_with_limits};
use remus_io::step::writer::write_step;
use remus_io::{ImportLimits, IoError};
use remus_math::vec::Point3;
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const REFERENCE_SPHERE: &str = include_str!("data/reference_vertex_loop_sphere.step");

fn sphere_volume(radius: f64) -> f64 {
    4.0 / 3.0 * PI * radius.powi(3)
}

fn assert_close(actual: f64, expected: f64, label: &str) {
    assert!(
        (actual - expected).abs() <= expected.abs() * 1e-9,
        "{label}: volume {actual} != {expected}"
    );
}

fn import_one(step: &str) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solids = read_step(step, &mut topo).expect("vertex-loop sphere imports");
    assert_eq!(solids.len(), 1);
    (topo, solids[0])
}

fn assert_whole_sphere(topo: &Topology, solid: SolidId, radius: f64, label: &str) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {:?}", report.issues);
    let shell = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    validate_shell_closed(shell, topo).expect("closed shell");
    let faces = solid_faces(topo, solid).unwrap();
    assert_eq!(faces.len(), 2, "{label}: native two-hemisphere layout");
    for face in &faces {
        let FaceSurface::Sphere(sphere) = topo.face(*face).unwrap().surface() else {
            panic!("{label}: hemisphere keeps the exact spherical carrier");
        };
        assert_eq!(sphere.radius().to_bits(), radius.to_bits());
    }
    assert_close(
        solid_volume(topo, solid, 0.005).unwrap(),
        sphere_volume(radius),
        label,
    );
}

/// The real producer file: exact radius-10 sphere, volume 4188.790204...
#[test]
fn reference_vertex_loop_sphere_imports_as_exact_closed_sphere() {
    assert!(REFERENCE_SPHERE.contains("#19 = VERTEX_LOOP('',#20);"));
    let (topo, solid) = import_one(REFERENCE_SPHERE);
    assert_whole_sphere(&topo, solid, 10.0, "reference sphere");
    for (point, expected) in [
        (Point3::new(0.0, 0.0, 0.0), PointClassification::Inside),
        (Point3::new(0.0, 0.0, -9.9), PointClassification::Inside),
        (Point3::new(0.0, 0.0, 9.9), PointClassification::Inside),
        (Point3::new(7.0, 7.0, 0.0), PointClassification::Inside),
        (Point3::new(0.0, 0.0, -10.1), PointClassification::Outside),
        (Point3::new(7.2, 7.2, 0.0), PointClassification::Outside),
    ] {
        assert_eq!(
            classify_point(&topo, solid, point, 0.01, 1e-7).unwrap(),
            expected,
            "{point:?}"
        );
    }
}

/// The imported sphere writes and re-reads as the same exact solid.
#[test]
fn vertex_loop_sphere_round_trips_through_the_writer() {
    let (topo, solid) = import_one(REFERENCE_SPHERE);
    let written = write_step(&topo, &[solid]).unwrap();
    assert!(!written.contains("VERTEX_LOOP"));
    let (again, solid) = import_one(&written);
    assert_whole_sphere(&again, solid, 10.0, "round trip");
}

fn vertex_loop_sphere_step(
    center: [f64; 3],
    axis: [f64; 3],
    radius: f64,
    vertex: [f64; 3],
    same_sense: &str,
) -> String {
    format!(
        "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('vertex loop sphere'),'2;1');\n\
         FILE_NAME('s.step','2026-11-09T00:00:00',(''),(''),'','','');\n\
         FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n\
         #1=CARTESIAN_POINT('',({:?},{:?},{:?}));\n\
         #2=DIRECTION('',({:?},{:?},{:?}));\n\
         #3=DIRECTION('',(1.,0.,0.));\n\
         #4=AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
         #5=SPHERICAL_SURFACE('',#4,{radius:?});\n\
         #6=CARTESIAN_POINT('',({:?},{:?},{:?}));\n\
         #7=VERTEX_POINT('',#6);\n\
         #8=VERTEX_LOOP('',#7);\n\
         #9=FACE_OUTER_BOUND('',#8,.T.);\n\
         #10=ADVANCED_FACE('',(#9),#5,{same_sense});\n\
         #11=CLOSED_SHELL('',(#10));\n\
         #12=MANIFOLD_SOLID_BREP('',#11);\n\
         #13=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));\n\
         #14=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));\n\
         #15=(NAMED_UNIT(*)SOLID_ANGLE_UNIT()SI_UNIT($,.STERADIAN.));\n\
         #16=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNIT_ASSIGNED_CONTEXT((#13,#14,#15))REPRESENTATION_CONTEXT('',''));\n\
         #17=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#12),#16);\nENDSEC;\nEND-ISO-10303-21;\n",
        center[0], center[1], center[2], axis[0], axis[1], axis[2], vertex[0], vertex[1], vertex[2],
    )
}

#[test]
fn hemisphere_names_obey_the_cumulative_import_budget_atomically() {
    let base = vertex_loop_sphere_step([0.0; 3], [0.0, 0.0, 1.0], 10.0, [0.0, 0.0, 10.0], ".T.");
    let named = |name: &str| {
        base.replace(
            "#10=ADVANCED_FACE('',",
            &format!("#10=ADVANCED_FACE('{name}',"),
        )
    };
    let attack = named(&"a".repeat(base.len() * 2));
    let mut topo = Topology::new();
    let anchor = make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
    let snapshot = |topo: &Topology| {
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
            topo.allocated_slot_count(),
            topo.journal().len(),
        )
    };
    let before = snapshot(&topo);
    let error = read_step_with_limits(
        &attack,
        &mut topo,
        ImportLimits {
            max_input_bytes: attack.len(),
            ..ImportLimits::default()
        },
    )
    .unwrap_err();
    assert!(
        matches!(
            error,
            IoError::LimitExceeded {
                resource: "STEP imported name bytes",
                ..
            }
        ),
        "{error}"
    );
    assert_eq!(snapshot(&topo), before);
    assert_close(solid_volume(&topo, anchor, 0.005).unwrap(), 6.0, "anchor");
    let accepted = named("named sphere");
    let roots = read_step_with_limits(
        &accepted,
        &mut topo,
        ImportLimits {
            max_input_bytes: accepted.len(),
            ..ImportLimits::default()
        },
    )
    .unwrap();
    let faces = solid_faces(&topo, roots[0]).unwrap();
    assert_eq!(faces.len(), 2);
    for face in faces {
        assert_eq!(
            topo.attributes().face(face).unwrap().name.as_deref(),
            Some("named sphere")
        );
    }
}

#[test]
fn hemisphere_entity_budget_includes_both_boundary_loops() {
    let step = vertex_loop_sphere_step([0.0; 3], [0.0, 0.0, 1.0], 10.0, [0.0, 0.0, 10.0], ".T.");
    let mut topo = Topology::new();
    let error = read_step_with_limits(
        &step,
        &mut topo,
        ImportLimits {
            max_model_entities: 23,
            ..ImportLimits::default()
        },
    )
    .unwrap_err();
    assert!(
        matches!(
            error,
            IoError::LimitExceeded {
                resource: "STEP generated arena entities",
                ..
            }
        ),
        "{error}"
    );
    assert_eq!(topo.allocated_slot_count(), 0);
    assert_eq!(topo.journal().len(), 0);
    let solids = read_step_with_limits(
        &step,
        &mut topo,
        ImportLimits {
            max_model_entities: 24,
            ..ImportLimits::default()
        },
    )
    .unwrap();
    assert_eq!(solids.len(), 1);
    assert_eq!(topo.allocated_slot_count(), 24);
}

/// An off-origin sphere on a tilted frame splits on its own equator.
#[test]
fn framed_vertex_loop_sphere_keeps_its_placement() {
    let center = [5.0, -3.0, 2.0];
    let radius = 7.0;
    let half = radius / 2.0_f64.sqrt();
    let pole = [center[0], center[1] + half, center[2] + half];
    let step = vertex_loop_sphere_step(center, [0.0, 1.0, 1.0], radius, pole, ".T.");
    let (topo, solid) = import_one(&step);
    assert_whole_sphere(&topo, solid, radius, "framed sphere");
    for (point, expected) in [
        (Point3::new(5.0, -3.0, 2.0), PointClassification::Inside),
        (Point3::new(11.9, -3.0, 2.0), PointClassification::Inside),
        (Point3::new(12.1, -3.0, 2.0), PointClassification::Outside),
        (Point3::new(0.0, 0.0, 0.0), PointClassification::Inside),
        (Point3::new(5.0, 4.1, 2.0), PointClassification::Outside),
    ] {
        assert_eq!(
            classify_point(&topo, solid, point, 0.01, 1e-7).unwrap(),
            expected,
            "{point:?}"
        );
    }
}

/// `ORIENTED_CLOSED_SHELL(.F.)` reverses both hemispheres: a spherical void.
#[test]
fn vertex_loop_sphere_void_subtracts_from_its_host() {
    let mut source = Topology::new();
    let host = make_box(&mut source, 20.0, 20.0, 20.0).unwrap();
    let written = write_step(&source, &[host]).unwrap();
    let (prefix, rest) = written
        .split_once("MANIFOLD_SOLID_BREP('', ")
        .expect("writer emits a manifold solid");
    let (outer, rest) = rest.split_once(')').unwrap();
    let void = "#9001 = CARTESIAN_POINT('', (10., 10., 10.));\n\
                #9002 = DIRECTION('', (0., 0., 1.));\n\
                #9003 = DIRECTION('', (1., 0., 0.));\n\
                #9004 = AXIS2_PLACEMENT_3D('', #9001, #9002, #9003);\n\
                #9005 = SPHERICAL_SURFACE('', #9004, 4.);\n\
                #9006 = CARTESIAN_POINT('', (10., 10., 14.));\n\
                #9007 = VERTEX_POINT('', #9006);\n\
                #9008 = VERTEX_LOOP('', #9007);\n\
                #9009 = FACE_BOUND('', #9008, .T.);\n\
                #9010 = ADVANCED_FACE('', (#9009), #9005, .T.);\n\
                #9011 = CLOSED_SHELL('', (#9010));\n\
                #9012 = ORIENTED_CLOSED_SHELL('', *, #9011, .F.);\n";
    let step = format!("{prefix}BREP_WITH_VOIDS('', {outer}, (#9012)){rest}").replacen(
        "ENDSEC;\nEND-ISO",
        &format!("{void}ENDSEC;\nEND-ISO"),
        1,
    );
    let (topo, solid) = import_one(&step);
    let report = validate_solid(&topo, solid).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);
    let data = topo.solid(solid).unwrap();
    assert_eq!(data.inner_shells().len(), 1);
    let cavity = topo.shell(data.inner_shells()[0]).unwrap();
    validate_shell_closed(cavity, &topo).expect("closed cavity");
    assert_eq!(cavity.faces().len(), 2);
    for face in cavity.faces() {
        assert!(topo.face(*face).unwrap().is_reversed());
    }
    // Cavity volume is deflection-sampled, exactly as for a native
    // `box - sphere` cut (7731.95901 at 0.005 mm): require the
    // same band and convergence toward the exact value.
    let exact = 8000.0 - sphere_volume(4.0);
    let coarse = (solid_volume(&topo, solid, 0.005).unwrap() - exact).abs();
    let fine = (solid_volume(&topo, solid, 0.0005).unwrap() - exact).abs();
    assert!(coarse <= exact * 1e-5, "spherical void error {coarse}");
    assert!(
        fine < coarse,
        "spherical void converges: {fine} !< {coarse}"
    );
    for (point, expected) in [
        (Point3::new(10.0, 10.0, 10.0), PointClassification::Outside),
        (Point3::new(10.0, 10.0, 15.0), PointClassification::Inside),
        (Point3::new(1.0, 1.0, 1.0), PointClassification::Inside),
    ] {
        assert_eq!(
            classify_point(&topo, solid, point, 0.01, 1e-7).unwrap(),
            expected,
            "{point:?}"
        );
    }
}

/// Every refusal is typed, names the loop, and leaves the arena untouched;
/// the same arena then still accepts the supported layout.
#[test]
fn unqualified_vertex_loops_refuse_atomically() {
    let off_surface =
        vertex_loop_sphere_step([0.0; 3], [0.0, 0.0, 1.0], 10.0, [0.0, 0.0, 10.5], ".T.");
    let plane = vertex_loop_sphere_step([0.0; 3], [0.0, 0.0, 1.0], 10.0, [0.0, 0.0, 10.0], ".T.")
        .replace("#5=SPHERICAL_SURFACE('',#4,10.0);", "#5=PLANE('',#4);");
    assert!(plane.contains("#5=PLANE('',#4);"));
    // A second face beside the vertex-loop sphere is not the whole-sphere
    // layout: the loop is refused instead of silently dropped.
    let shared = REFERENCE_SPHERE.replace(
        "#16 = CLOSED_SHELL('',(#17));",
        "#16 = CLOSED_SHELL('',(#17,#17));",
    );
    assert_ne!(shared, REFERENCE_SPHERE);
    // A vertex loop beside an edge loop on one face.
    let mixed = REFERENCE_SPHERE.replace(
        "#17 = ADVANCED_FACE('',(#18),#22,.T.);",
        "#17 = ADVANCED_FACE('',(#18,#18),#22,.T.);",
    );
    assert_ne!(mixed, REFERENCE_SPHERE);

    let mut topo = Topology::new();
    let anchor = make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
    let counts = |topo: &Topology| {
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
            topo.allocated_slot_count(),
            topo.journal().len(),
        )
    };
    let before = counts(&topo);

    let error = read_step(&off_surface, &mut topo).unwrap_err();
    assert!(
        matches!(&error, IoError::ParseError { reason } if reason.contains("VERTEX_LOOP #8") && reason.contains("off SPHERICAL_SURFACE #5")),
        "{error:?}"
    );
    assert_eq!(counts(&topo), before);

    for (label, step) in [
        ("plane", &plane),
        ("two faces", &shared),
        ("two bounds", &mixed),
    ] {
        let error = read_step(step, &mut topo).unwrap_err();
        assert!(
            matches!(&error, IoError::UnsupportedEntity { entity } if entity.starts_with("VERTEX_LOOP #") && entity.contains("SPHERICAL_SURFACE")),
            "{label}: {error:?}"
        );
        assert!(
            !format!("{error}").contains("EDGE_LOOP"),
            "{label}: {error}"
        );
        assert_eq!(counts(&topo), before, "{label}");
    }

    assert_close(solid_volume(&topo, anchor, 0.005).unwrap(), 6.0, "anchor");
    let solids = read_step(REFERENCE_SPHERE, &mut topo).unwrap();
    assert_eq!(solids.len(), 1);
    assert_whole_sphere(&topo, solids[0], 10.0, "after refusals");
}
