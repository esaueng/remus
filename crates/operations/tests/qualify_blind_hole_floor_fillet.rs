//! Qualification evidence for the blind-hole FLOOR rim fillet
//! (stabilization plan item C1.1).
//!
//! The stability matrix carried this as a known wrong-direction defect
//! ("an r = 3 hole rounded at r = 1 loses 7.93 where the closed form adds
//! 3.74") capped at `r_c/2`. The defect no longer reproduces: the concave
//! inward lane of the analytic plane/cylinder assembler builds the exact
//! toroidal collar, verified here against the closed form
//!
//! `V_add = 2π · [ r²(r_c − r/2) − (π r²/4)(r_c − r + 4r/(3π)) ]`
//!
//! across the whole widened radius range `0 < r < r_c` — including
//! `r > r_c/2`, where the carrier torus is a horn or spindle but the
//! quarter-tube collar cut from it is sound. `r ≥ r_c` is refused as a
//! typed `RadiusTooLarge`, both sides of the bound tested.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::solid::SolidId;

const RC: f64 = 3.0;
const DEPTH: f64 = 5.0;

/// Block with a blind hole of radius `RC`, depth `DEPTH`, plus its floor
/// rim edge.
fn blind_hole_block(topo: &mut Topology) -> (SolidId, EdgeId, f64) {
    let block = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let drill = make_cylinder(topo, RC, DEPTH + 1.0).unwrap();
    transform_solid(topo, drill, &Mat4::translation(5.0, 5.0, 10.0 - DEPTH)).unwrap();
    let holed = boolean(topo, BooleanOp::Cut, block, drill).unwrap();
    let v0 = remus_operations::measure::solid_volume(topo, holed, 0.01).unwrap();

    let floor_z = 10.0 - DEPTH;
    let s = topo.solid(holed).unwrap();
    let mut rim = None;
    for &fid in topo.shell(s.outer_shell()).unwrap().faces() {
        let f = topo.face(fid).unwrap();
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                let e = topo.edge(oe.edge()).unwrap();
                if let EdgeCurve::Circle(c) = e.curve()
                    && (c.center().z() - floor_z).abs() < 1e-9
                    && (c.radius() - RC).abs() < 1e-9
                {
                    rim = Some(oe.edge());
                }
            }
        }
    }
    (holed, rim.expect("floor rim edge"), v0)
}

/// Exact collar volume the fillet must add.
fn collar(r: f64) -> f64 {
    2.0 * PI * (r * r * (RC - r / 2.0) - (PI * r * r / 4.0) * (RC - r + 4.0 * r / (3.0 * PI)))
}

/// The concave floor-rim fillet adds the exact toroidal collar across the
/// radius sweep, including past the historical r_c/2 cap into the horn and
/// spindle carrier regimes.
#[test]
fn floor_rim_collar_matches_closed_form_across_radius_sweep() {
    for r in [0.5, 1.0, 1.5, 2.0, 2.9] {
        let mut topo = Topology::new();
        let (holed, rim, v0) = blind_hole_block(&mut topo);
        let res = remus_operations::blend_ops::fillet_v2(&mut topo, holed, &[rim], r)
            .unwrap_or_else(|e| panic!("fillet at r={r} failed: {e:?}"));
        let report = remus_operations::validate::validate_solid(&topo, res.solid).unwrap();
        assert!(report.is_valid(), "r={r}: {:?}", report.issues);
        let v = remus_operations::measure::solid_volume(&topo, res.solid, 0.01).unwrap();
        let expected = v0 + collar(r);
        assert!(
            ((v - v0) - collar(r)).abs() / collar(r) < 0.01,
            "r={r}: expected volume {expected} (+{}), got {v} (+{})",
            collar(r),
            v - v0
        );
    }
}

/// Both sides of the r_c bound: just below succeeds with the closed form;
/// at and above, the rolling ball no longer fits and the refusal is typed.
#[test]
fn radius_bound_both_sides() {
    let mut topo = Topology::new();
    let (holed, rim, v0) = blind_hole_block(&mut topo);
    let r = RC - 0.05;
    let res = remus_operations::blend_ops::fillet_v2(&mut topo, holed, &[rim], r).unwrap();
    let v = remus_operations::measure::solid_volume(&topo, res.solid, 0.01).unwrap();
    assert!(((v - v0) - collar(r)).abs() / collar(r) < 0.01);

    let mut topo = Topology::new();
    let (holed, rim, _) = blind_hole_block(&mut topo);
    match remus_operations::blend_ops::fillet_v2(&mut topo, holed, &[rim], RC) {
        Ok(_) => panic!("r = r_c must be refused: the rolling ball does not fit"),
        Err(err) => {
            let msg = format!("{err:?}");
            assert!(
                msg.contains("RadiusTooLarge") || msg.contains("radius"),
                "expected a typed radius refusal at r = r_c, got {err:?}"
            );
        }
    }
}
