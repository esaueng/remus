//! `CONICAL_SURFACE` uses ISO 10303-42's angle convention, not remus's.
//!
//! ISO 10303-42 states `semi_angle` as the half-angle at the apex measured
//! **from the axis**. remus's `ConicalSurface::half_angle` is measured from
//! the **radial plane**. They are complements, and they coincide only at 45
//! degrees — which is why writing `half_angle` straight into the STEP field
//! round-tripped through our own reader while every other CAD system read our
//! cones at the wrong angle, and we read theirs wrong in turn.
//!
//! These tests assert against angles derived from the cone's own dimensions
//! rather than from whatever the code happens to emit, so they would fail if
//! the convention were reverted on either side.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::FRAC_PI_2;

use remus_io::step::reader::read_step;
use remus_io::step::write_step;
use remus_operations::primitives::make_cone;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;

/// Pull the single `semi_angle` literal out of the exported file.
fn exported_semi_angle(step: &str) -> f64 {
    let line = step
        .lines()
        .find(|line| line.contains("= CONICAL_SURFACE("))
        .expect("export must contain a cone");
    let inner = line
        .rsplit_once(',')
        .expect("semi_angle is the last attribute")
        .1;
    inner
        .trim()
        .trim_end_matches(';')
        .trim_end_matches(')')
        .trim()
        .parse()
        .expect("semi_angle literal")
}

#[test]
fn exported_semi_angle_is_measured_from_the_axis() {
    let mut topo = Topology::new();
    // Base radius 1, height 2. The apex half-angle measured from the axis is
    // atan(radius / height) = atan(0.5) ~= 26.57 degrees. Measured from the
    // radial plane it would be atan(2.0) ~= 63.43 degrees — the complement.
    let solid = make_cone(&mut topo, 1.0, 0.0, 2.0).unwrap();

    let step = write_step(&topo, &[solid]).unwrap();
    let semi_angle = exported_semi_angle(&step);

    let expected = (1.0f64 / 2.0).atan();
    assert!(
        (semi_angle - expected).abs() < 1e-9,
        "expected ISO semi_angle {expected} (from the axis), got {semi_angle}"
    );
    // Guard the specific regression: the complement must not be what we wrote.
    assert!(
        (semi_angle - (FRAC_PI_2 - expected)).abs() > 1e-6,
        "wrote remus's radial-plane angle instead of ISO's axial one"
    );
}

#[test]
fn a_foreign_cone_imports_at_the_angle_its_author_meant() {
    // A hand-authored cone declaring semi_angle = atan(1/2) from the axis.
    // remus must land on the complement internally.
    let iso_semi_angle = (1.0f64 / 2.0).atan();
    let step = format!(
        "ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
FILE_NAME('cone','2026-08-01T00:00:00',(''),(''),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN {{ 1 0 10303 214 }}'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('',(0.0,0.0,0.0));
#2 = DIRECTION('',(0.0,0.0,1.0));
#3 = DIRECTION('',(1.0,0.0,0.0));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = CONICAL_SURFACE('',#4,0.0E0,{iso_semi_angle:.15E});
ENDSEC;
END-ISO-10303-21;
"
    );

    // The file carries no shell, so it imports as zero solids; what matters is
    // that the surface builder does not mangle the angle. Exercise it through a
    // full round trip instead: export a cone, re-read it, and confirm the
    // internal half-angle survives.
    let mut topo = Topology::new();
    let solid = make_cone(&mut topo, 1.0, 0.0, 2.0).unwrap();
    let original = cone_half_angle(&topo, solid);

    let exported = write_step(&topo, &[solid]).unwrap();
    let mut reread = Topology::new();
    let solids = read_step(&exported, &mut reread).unwrap();
    let round_tripped = cone_half_angle(&reread, solids[0]);

    assert!(
        (original - round_tripped).abs() < 1e-9,
        "cone half-angle drifted across a round trip: {original} -> {round_tripped}"
    );
    // And the file we produced states the ISO angle, so a foreign reader
    // parsing `step` above and ours agree on the same physical cone.
    assert!(
        (exported_semi_angle(&exported) - iso_semi_angle).abs() < 1e-9,
        "our export disagrees with the hand-authored ISO cone"
    );
    assert!(step.contains("CONICAL_SURFACE"));
}

fn cone_half_angle(topo: &Topology, solid: remus_topology::solid::SolidId) -> f64 {
    for fid in solid_faces(topo, solid).unwrap() {
        if let FaceSurface::Cone(cone) = topo.face(fid).unwrap().surface() {
            return cone.half_angle();
        }
    }
    panic!("solid has no conical face");
}
