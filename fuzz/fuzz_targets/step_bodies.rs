//! STEP sheet-body and validation-property import paths (IO-12).
//!
//! `step_reader` only drives the solid-only entry point. This target runs the
//! same bytes through `read_step_bodies_with_limits`, which also builds
//! `SHELL_BASED_SURFACE_MODEL` sheets, and through `read_step_with_validation`,
//! which parses CAx-IF validation properties and recomputes volume, area and
//! centroid on every imported solid.
//!
//! Oracles: both imports are atomic, so a refusal must leave the fresh
//! topology with no live entities; an accepted import must return handles that
//! resolve, one validation report per solid, and sheets that are sheet-class
//! shells.

#![no_main]

use libfuzzer_sys::fuzz_target;
use remus_io::step::{
    StepValidationOptions, read_step_bodies_with_limits, read_step_with_validation,
};
use remus_topology::{BodyClass, BodyId, Topology};

mod common;

fn assert_untouched(topo: &Topology, path: &str) {
    assert_eq!(
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
        ),
        (0, 0, 0, 0, 0, 0),
        "a refused {path} import left live entities behind"
    );
}

fuzz_target!(|data: &[u8]| {
    let Ok(input) = std::str::from_utf8(data) else {
        return;
    };

    let mut topo = Topology::new();
    match read_step_bodies_with_limits(input, &mut topo, common::limits()) {
        Ok(result) => {
            for &solid in result.solids() {
                assert!(topo.solid(solid).is_ok(), "returned solid does not resolve");
            }
            for &sheet in result.sheets() {
                assert_eq!(
                    topo.body_class_of(BodyId::Shell(sheet)).ok(),
                    Some(BodyClass::Sheet),
                    "returned sheet is not a sheet-class shell"
                );
            }
        }
        Err(_) => assert_untouched(&topo, "body"),
    }

    let mut topo = Topology::new();
    match read_step_with_validation(
        input,
        &mut topo,
        common::limits(),
        StepValidationOptions::default(),
    ) {
        Ok(result) => {
            assert_eq!(
                result.validation().len(),
                result.solids().len(),
                "every imported solid needs one validation report"
            );
            for &solid in result.solids() {
                assert!(topo.solid(solid).is_ok(), "returned solid does not resolve");
            }
        }
        Err(_) => assert_untouched(&topo, "validation"),
    }
});
