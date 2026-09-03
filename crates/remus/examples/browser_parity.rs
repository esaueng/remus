//! Replays the native equivalent of the cross-drilled WASM contract fixture.

use std::error::Error;
use std::f64::consts::FRAC_PI_2;

use remus::prelude::*;

const EXPECTED_VOLUME: f64 = 704.230_016_469_242_4;
const VOLUME_TOLERANCE: f64 = 0.08;
const EXPECTED_TRIANGLES: usize = 12_290;

fn build_cross_drilled_shaft() -> Result<(), Box<dyn Error>> {
    let mut model = Model::new();
    let shaft = model.make_cylinder(3.0, 30.0)?;
    let tool = model.make_cylinder(3.0, 40.0)?;

    // Keep this sequence identical to
    // crates/wasm/tests/repro/cross-drilled-render-measure.json.
    model.transform(tool, &Mat4::rotation_y(FRAC_PI_2))?;
    model.transform(tool, &Mat4::translation(-20.0, 0.0, 15.0))?;
    let drilled = model.cut(shaft, tool)?;
    assert_eq!(drilled.quality, BooleanQuality::Exact);

    let volume = model.volume(drilled.solid, 0.08)?;
    assert!(
        (volume - EXPECTED_VOLUME).abs() <= VOLUME_TOLERANCE,
        "native volume {volume} diverged from the WASM fixture oracle"
    );

    let mesh = model.tessellate_with_tolerance(drilled.solid, 0.3, 0.06)?;
    let quality = welded_mesh_quality(&mesh);
    assert_eq!(quality.triangle_count, EXPECTED_TRIANGLES);
    assert_eq!(quality.boundary_edges, 0);
    assert_eq!(quality.non_manifold_edges, 0);
    assert_eq!(quality.euler_characteristic, 2);
    assert!(quality.is_watertight());

    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    build_cross_drilled_shaft()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn native_workflow_matches_the_wasm_contract_fixture() -> Result<(), Box<dyn Error>> {
        build_cross_drilled_shaft()
    }
}
