//! Issue 4.3 qualification for cellular solid splitting by a curved sheet.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::mat::Mat4;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::sew::make_sheet_body;
use remus_operations::split::split_by_sheet;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;

fn cylindrical_sheet_fixture(
    topo: &mut Topology,
) -> (remus_topology::SolidId, remus_topology::ShellId) {
    let blank = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let carrier = make_cylinder(topo, 2.0, 12.0).unwrap();
    transform_solid(topo, carrier, &Mat4::translation(5.0, 5.0, -1.0)).unwrap();
    let lateral = remus_topology::explorer::solid_faces(topo, carrier)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .expect("primitive cylinder has one lateral face");
    let sheet = make_sheet_body(topo, &[lateral]).unwrap();
    (blank, sheet)
}

fn result_signature() -> Vec<(usize, i64)> {
    let mut topo = Topology::new();
    let (blank, sheet) = cylindrical_sheet_fixture(&mut topo);
    let compound = split_by_sheet(&mut topo, blank, sheet).unwrap();
    topo.compound(compound)
        .unwrap()
        .solids()
        .iter()
        .map(|&solid| {
            let faces = remus_topology::explorer::solid_faces(&topo, solid)
                .unwrap()
                .len();
            let volume = solid_volume(&topo, solid, 0.01).unwrap();
            (faces, (volume * 1.0e9).round() as i64)
        })
        .collect()
}

#[test]
fn cylindrical_sheet_splits_box_into_two_valid_volume_conserving_cells() {
    let mut topo = Topology::new();
    let (blank, sheet) = cylindrical_sheet_fixture(&mut topo);
    let original = solid_volume(&topo, blank, 0.01).unwrap();

    let compound = split_by_sheet(&mut topo, blank, sheet).unwrap();
    let regions = topo.compound(compound).unwrap().solids();
    assert_eq!(regions.len(), 2);

    let volumes: Vec<f64> = regions
        .iter()
        .map(|&region| {
            let report = validate_solid(&topo, region).unwrap();
            assert!(report.is_valid(), "{:#?}", report.issues);
            solid_volume(&topo, region, 0.01).unwrap()
        })
        .collect();
    let expected_inner = std::f64::consts::PI * 2.0_f64.powi(2) * 10.0;
    assert!((volumes[0] - expected_inner).abs() < 1.0e-7, "{volumes:?}");
    assert!((volumes.iter().sum::<f64>() - original).abs() < 1.0e-7);
}

#[test]
fn cylindrical_sheet_split_is_deterministic() {
    assert_eq!(result_signature(), result_signature());
}

#[test]
fn unsupported_sheet_surface_refuses_transactionally() {
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let face = remus_topology::explorer::solid_faces(&topo, blank).unwrap()[0];
    let sheet = make_sheet_body(&mut topo, &[face]).unwrap();
    let counts = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
        topo.num_compounds(),
    );

    let error = split_by_sheet(&mut topo, blank, sheet).unwrap_err();
    assert!(
        error.to_string().contains("unsupported sheet split"),
        "{error}"
    );
    assert_eq!(
        counts,
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
            topo.num_compounds(),
        )
    );
}
