//! Exact IGES 5.3 B-rep exchange (`read_iges_brep` / `write_iges_brep`).
//!
//! The fixtures under `tests/data/iges_brep/` were written in B-rep mode by
//! an independent kernel's IGES writer (global section rebuilt without
//! product or author strings). Every expected volume and centroid below was
//! read back from the same files by that kernel's own IGES reader.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::arena_io::serialize_solid;
use remus_io::{ImportLimits, IoError, iges};
use remus_operations::{measure, primitives};
use remus_topology::Topology;
use remus_topology::solid::SolidId;

macro_rules! fixture {
    ($name:literal) => {
        include_str!(concat!("data/iges_brep/", $name, ".igs"))
    };
}

/// (fixture, independent volume mm³, independent centroid mm).
const INDEPENDENT: &[(&str, &str, f64, [f64; 3])] = &[
    ("box", fixture!("box_brep_mm"), 1000.0, [10.0, 5.0, 2.5]),
    (
        "cylinder",
        fixture!("cylinder_brep_mm"),
        785.398_163,
        [0.0, 0.0, 5.0],
    ),
    (
        "cone",
        fixture!("cone_brep_mm"),
        490.088_454,
        [0.0, 0.0, 3.115_385],
    ),
    (
        "sphere",
        fixture!("sphere_brep_mm"),
        904.778_684,
        [0.0, 0.0, 0.0],
    ),
    (
        "torus",
        fixture!("torus_brep_mm"),
        1_776.528_792,
        [0.0, 0.0, 0.0],
    ),
    (
        "holed plate",
        fixture!("holed_plate_brep_mm"),
        2_198.938_070,
        [15.0, 10.0, 2.0],
    ),
    (
        "cavity",
        fixture!("cavity_brep_mm"),
        24_855.339_415,
        [15.0, 15.0, 15.0],
    ),
    (
        "cavity, inches",
        fixture!("cavity_brep_in"),
        24_855.339_415,
        [15.0, 15.0, 15.0],
    ),
    (
        "NURBS box",
        fixture!("nurbs_box_brep_mm"),
        576.0,
        [6.0, 4.0, 3.0],
    ),
    (
        "tilted cylinder",
        fixture!("tilted_cylinder_brep_mm"),
        753.982_237,
        [9.651_650, -5.651_650, 17.495_191],
    ),
    (
        "tilted cylinder, inches",
        fixture!("tilted_cylinder_brep_in"),
        753.982_237,
        [9.651_650, -5.651_650, 17.495_191],
    ),
];

fn read(text: &str) -> (Topology, Vec<SolidId>) {
    let mut topo = Topology::new();
    let result = iges::read_iges_brep(text, &mut topo, ImportLimits::default()).unwrap();
    (topo, result.solids)
}

fn check(name: &str, topo: &Topology, solid: SolidId, volume: f64, centre: [f64; 3]) {
    let props = measure::mass_properties(topo, solid).unwrap();
    assert!(
        (props.mass - volume).abs() <= 2e-6 * volume,
        "{name}: volume {} != {volume}",
        props.mass
    );
    let actual = [props.center.x(), props.center.y(), props.center.z()];
    for axis in 0..3 {
        assert!(
            (actual[axis] - centre[axis]).abs() <= 2e-6,
            "{name}: centroid {actual:?} != {centre:?}"
        );
    }
    let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{name}: {report:?}");
}

#[test]
fn independent_brep_files_read_exactly() {
    for &(name, text, volume, centre) in INDEPENDENT {
        let (topo, solids) = read(text);
        assert_eq!(solids.len(), 1, "{name}");
        check(name, &topo, solids[0], volume, centre);
    }
}

#[test]
fn written_files_read_back_to_the_same_solids() {
    for &(name, text, volume, centre) in INDEPENDENT {
        let (topo, solids) = read(text);
        let written =
            iges::write_iges_brep(&topo, &solids, &iges::IgesWriteOptions::default()).unwrap();
        assert!(
            written.lines().all(|line| line.len() == 80),
            "{name}: every record is 80 columns"
        );
        assert!(written.contains("186,"), "{name}");
        let (again, back) = read(&written);
        assert_eq!(back.len(), 1, "{name}");
        check(name, &again, back[0], volume, centre);
    }
}

#[test]
fn native_primitives_round_trip_and_keep_their_carriers() {
    let mut topo = Topology::new();
    let solids = [
        (
            primitives::make_box(&mut topo, 20.0, 10.0, 5.0).unwrap(),
            "190",
        ),
        (
            primitives::make_cylinder(&mut topo, 5.0, 10.0).unwrap(),
            "192",
        ),
        (
            primitives::make_cone(&mut topo, 6.0, 2.0, 9.0).unwrap(),
            "194",
        ),
        (primitives::make_sphere(&mut topo, 6.0, 16).unwrap(), "196"),
    ];
    for (solid, carrier) in solids {
        let props = measure::mass_properties(&topo, solid).unwrap();
        let written =
            iges::write_iges_brep(&topo, &[solid], &iges::IgesWriteOptions::default()).unwrap();
        assert!(
            written
                .lines()
                .any(|line| line.starts_with(&format!("{carrier},"))),
            "{carrier} carrier written"
        );
        let (again, back) = read(&written);
        check(
            carrier,
            &again,
            back[0],
            props.mass,
            [props.center.x(), props.center.y(), props.center.z()],
        );
    }
    // Several solids in one file stay separate.
    let all: Vec<_> = solids.iter().map(|(solid, _)| *solid).collect();
    let written = iges::write_iges_brep(&topo, &all, &iges::IgesWriteOptions::default()).unwrap();
    assert_eq!(read(&written).1.len(), 4);
}

#[test]
fn a_closed_edge_with_no_carrier_is_refused_by_name() {
    let mut topo = Topology::new();
    let torus = primitives::make_torus(&mut topo, 10.0, 3.0, 16).unwrap();
    let error =
        iges::write_iges_brep(&topo, &[torus], &iges::IgesWriteOptions::default()).unwrap_err();
    assert!(
        error.to_string().contains("closed edge with no carrier"),
        "{error}"
    );
}

#[test]
fn a_surface_model_is_classified_not_imported() {
    let mut topo = Topology::new();
    let error = iges::read_iges_brep(
        fixture!("holed_plate_faces_mm"),
        &mut topo,
        ImportLimits::default(),
    )
    .unwrap_err();
    match error {
        IoError::UnsupportedEntity { entity } => {
            assert_eq!(
                entity,
                "IGES surface model (7 trimmed surfaces, no type 186 solid)"
            );
        }
        other => panic!("{other}"),
    }
    assert_eq!(topo.num_solids(), 0);
}

/// Replaces the parameter record of the first entity of `entity_type`.
fn with_parameters(text: &str, entity_type: u32, parameters: &str) -> String {
    let prefix = format!("{entity_type},");
    let mut done = false;
    text.lines()
        .map(|line| {
            if !done && line.len() == 80 && &line[72..73] == "P" && line.starts_with(&prefix) {
                done = true;
                format!("{:<64}{}", parameters, &line[64..])
            } else {
                line.to_string()
            }
        })
        .collect::<Vec<_>>()
        .join("\n")
}

fn refuses_atomically(text: &str) -> IoError {
    let mut topo = Topology::new();
    let existing = primitives::make_box(&mut topo, 3.0, 4.0, 5.0).unwrap();
    let before = serialize_solid(&topo, existing).unwrap();
    let counts = (topo.num_solids(), topo.num_faces(), topo.num_edges());
    let error = iges::read_iges_brep(text, &mut topo, ImportLimits::default()).unwrap_err();
    assert_eq!(
        counts,
        (topo.num_solids(), topo.num_faces(), topo.num_edges())
    );
    assert_eq!(before, serialize_solid(&topo, existing).unwrap());
    error
}

#[test]
fn non_rigid_frames_and_broken_pointers_are_refused_atomically() {
    let tilted = fixture!("tilted_cylinder_brep_mm");
    // A 124 that scales X by two.
    let scaled = with_parameters(tilted, 124, "124,2.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.;");
    assert!(
        refuses_atomically(&scaled)
            .to_string()
            .contains("not rigid")
    );
    // The 186's shell pointer names a directory entry that is not a shell.
    let box_text = fixture!("box_brep_mm");
    let broken = with_parameters(box_text, 186, "186,1,1,0;");
    assert!(matches!(
        refuses_atomically(&broken),
        IoError::ParseError { .. }
    ));
    // A budget smaller than the file refuses before reading.
    let mut topo = Topology::new();
    let limits = ImportLimits {
        max_input_bytes: 64,
        ..ImportLimits::default()
    };
    assert!(matches!(
        iges::read_iges_brep(box_text, &mut topo, limits),
        Err(IoError::LimitExceeded { .. })
    ));
}

#[test]
fn the_restated_step_states_millimetres() {
    let step =
        iges::iges_brep_as_step(fixture!("cavity_brep_in"), ImportLimits::default()).unwrap();
    assert!(step.contains("SI_UNIT(.MILLI.,.METRE.)"));
    assert!(step.contains("BREP_WITH_VOIDS"));
}

#[test]
fn ellipse_edges_write_exactly() {
    // A STEP file from an independent writer: a cylinder cut by a slanted
    // plane (ELLIPSE edges), with that kernel's volume and centroid.
    let cases = [(
        "ellipse",
        include_str!("data/iges_brep/ellipse_edges.step"),
        "104,",
        942.477_796,
        [0.0, 0.300_703, 6.086_806],
    )];
    for (name, step, carrier, volume, centre) in cases {
        let mut topo = Topology::new();
        let solids = remus_io::step::read_step(step, &mut topo).unwrap();
        assert_eq!(solids.len(), 1, "{name}");
        check(name, &topo, solids[0], volume, centre);
        let written =
            iges::write_iges_brep(&topo, &solids, &iges::IgesWriteOptions::default()).unwrap();
        assert!(
            written.lines().any(|line| line.starts_with(carrier)),
            "{name}: {carrier} written"
        );
        let (again, back) = read(&written);
        check(name, &again, back[0], volume, centre);
    }
}

#[test]
fn an_edge_whose_vertex_is_off_its_curve_is_refused_not_repaired() {
    // An independent slanted cut whose ellipse edges were written as
    // approximating polynomial 126 splines: as read, an edge's vertex lies
    // 0.77 mm off its curve. The import refuses atomically rather than
    // moving geometry (an open cell of the precise subset).
    let error = refuses_atomically(fixture!("slanted_cut_brep_mm"));
    assert!(error.to_string().contains("misses its carrier"), "{error}");
}

#[test]
fn hostile_parameter_text_refuses_or_reads_in_linear_time() {
    let box_text = fixture!("box_brep_mm");
    // A Hollerith count past the end of memory: a typed refusal, no panic.
    for count in ["18446744073709551615", "4294967295", "999999999"] {
        let hostile = with_parameters(box_text, 116, &format!("116,{count}Hx;"));
        let mut topo = Topology::new();
        assert!(iges::read_iges_brep(&hostile, &mut topo, ImportLimits::default()).is_err());
    }
    // A 400,000-digit number spread over parameter records is one linear
    // scan, then a refusal (it is not a usable coordinate).
    let digits = "7".repeat(400_000);
    let mut lines: Vec<String> = box_text.lines().map(str::to_string).collect();
    let at = lines
        .iter()
        .position(|line| line.len() == 80 && &line[72..73] == "P" && line.starts_with("116,"))
        .unwrap();
    let pointer = lines[at][64..72].to_string();
    let records: Vec<String> = format!("116,{digits},0.,0.,0;")
        .as_bytes()
        .chunks(64)
        .map(|chunk| format!("{:<64}{pointer}P{:>7}", String::from_utf8_lossy(chunk), 0))
        .collect();
    lines.splice(at..=at, records);
    let started = std::time::Instant::now();
    let mut topo = Topology::new();
    let _ = iges::read_iges_brep(&lines.join("\n"), &mut topo, ImportLimits::default());
    assert!(started.elapsed() < std::time::Duration::from_secs(10));
}
