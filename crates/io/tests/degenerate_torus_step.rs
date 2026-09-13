//! Exact STEP spindle-torus regression built from synthetic circular geometry.
#![allow(clippy::unwrap_used, clippy::expect_used)]
use remus_topology::{Topology, explorer::solid_faces};

/// A truncated lemon with R=2, r=5 between z=-3 and z=3. Both rims have
/// radius 2; the equator has radius 3. Its analytic volume is
/// pi * integral[-3,3] (sqrt(25-z*z)-2)^2 dz.
fn lemon_band() -> String {
    include_str!("data/lemon-torus-band.step").to_string()
}

#[test]
fn lemon_band_is_closed_oriented_and_roundtrips_exactly() {
    check_band(false);
    check_band(true);
}

fn check_band(outer: bool) {
    let expected = std::f64::consts::PI
        * if outer {
            204.0 + 100.0 * 0.6_f64.asin()
        } else {
            108.0 - 100.0 * 0.6_f64.asin()
        };
    let mut source = lemon_band();
    if outer {
        source = source
            .replace("#8,2.,5.,.F.", "#8,2.,5.,.T.")
            .replace("#6,2.)", "#6,6.)")
            .replace("#7,2.)", "#7,6.)")
            .replace("(2.,0.,-3.)", "(6.,0.,-3.)")
            .replace("(2.,0.,3.)", "(6.,0.,3.)")
            .replace("(-2.,0.,0.)", "(2.,0.,0.)")
            .replace("(#38),#11,.F.", "(#38),#11,.T.");
    }
    for _ in 0..2 {
        let mut topo = Topology::new();
        let solids = remus_io::step::read_step(&source, &mut topo).unwrap();
        assert_eq!(solids.len(), 1);
        assert_eq!(solid_faces(&topo, solids[0]).unwrap().len(), 3);
        let report = remus_operations::validate::validate_solid_relaxed(&topo, solids[0]).unwrap();
        assert!(report.issues.is_empty(), "{report:?}");
        let mesh = remus_operations::tessellate::tessellate_solid(&topo, solids[0], 0.01).unwrap();
        let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
        assert!(quality.is_watertight(), "{quality:?}");
        let mut signed_volume = 0.0;
        for tri in mesh.indices.chunks_exact(3) {
            let a = mesh.positions[tri[0] as usize];
            let b = mesh.positions[tri[1] as usize];
            let c = mesh.positions[tri[2] as usize];
            signed_volume += a.x() * (b.y() * c.z() - b.z() * c.y())
                + a.y() * (b.z() * c.x() - b.x() * c.z())
                + a.z() * (b.x() * c.y() - b.y() * c.x());
        }
        signed_volume /= 6.0;
        assert!(
            (signed_volume - expected).abs() / expected < 0.005,
            "{signed_volume} vs {expected}"
        );
        source = remus_io::step::write_step(&topo, &solids).unwrap();
        assert!(source.contains("RATIONAL_B_SPLINE_SURFACE"));
    }
}
