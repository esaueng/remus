//! Regression (B89): every pcurve of an imported STEP body must trace its
//! edge, on import and after any similarity or affine transform.
//!
//! The OCCT-exported fillet plate carries LINE pcurves on four quarter-cylinder
//! bands and LINE or rational B-spline pcurves on every plane.
//!
//! - Import: a band whose LINE origin sat exactly half a turn from one end
//!   lifted that end onto the far periodic branch, so the pcurve swept three
//!   quarters of the cylinder (mid-span deviation 6.0 = the band's diameter).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::step::reader::read_step;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;
use remus_topology::validation::check_same_parameter;

const FILLETED_PLATE: &str = include_str!("data/openzcad_e_analytic_fillet_plate.step");
const TOLERANCE: f64 = 1e-7;

fn import_plate() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solids = read_step(FILLETED_PLATE, &mut topo).expect("import STEP fixture");
    assert_eq!(solids.len(), 1);
    (topo, solids[0])
}

/// The largest sampled `SameParameter` deviation over every boundary use
/// that carries a pcurve on a surface with a UV evaluation, and the number
/// of uses measured.
fn max_same_parameter_deviation(topo: &Topology, solid: SolidId) -> (f64, usize) {
    let mut worst = 0.0_f64;
    let mut measured = 0;
    for face in solid_faces(topo, solid).unwrap() {
        for (edge, forward, _) in topo.pcurves_for_face(face) {
            if let Some(report) = check_same_parameter(topo, edge, face, forward, 64).unwrap() {
                worst = worst.max(report.max_deviation);
                measured += 1;
            }
        }
    }
    (worst, measured)
}

#[test]
fn analytic_fillet_plate_band_pcurves_trace_their_edges_on_import() {
    let (topo, solid) = import_plate();
    let (deviation, measured) = max_same_parameter_deviation(&topo, solid);
    assert_eq!(measured, 16, "four bands, four uses each");
    assert!(
        deviation <= TOLERANCE,
        "imported band pcurves leave their edges by {deviation}"
    );
}
