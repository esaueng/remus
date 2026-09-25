//! B61 ready-repro: the variable-radius fillet closes each stripe end with a
//! straight chord, so its blend face is bounded by edges that do not lie on
//! it.
//!
//! Found diagnosing B59's in-range volume gap (2026-09-25): a constant-law
//! radius-9 stripe on one vertical edge of a 10³ box measures 824.41 on its
//! mesh at every deflection (0.1, 0.01, 0.001) and 749.12 by Gauss, against
//! the closed form 826.1725 that the walking engine hits to 1e-10. The blend
//! wall itself is exact — its samples sit 1.8e-15 from the rolling-ball axis —
//! but the assembler mints `Line` edges between the vertex positions of every
//! spec, so the wall's two end boundaries and the matching cap-face edges are
//! chords of the quarter circle, `r(1 − cos 45°)` = 2.64 away from the wall
//! at their midpoints. `validateSolid` accepts it. The gap scales with r²
//! (0.0083 on the mesh at r = 1), which is why every earlier variable-fillet
//! check passed inside its tolerance.
//!
//! Acceptance: the stripe's end boundaries lie on the blend wall, and the
//! body measures the closed form by Gauss and on its mesh.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::Point3;
use remus_operations::fillet::{FilletEdgeSetback, FilletRadiusLaw, fillet_variable_with_setbacks};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::make_box;
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;

#[test]
#[ignore = "open: B61 variable fillet trims its stripe ends with straight chords"]
fn variable_fillet_end_boundaries_lie_on_the_blend_wall() {
    let radius = 9.0;
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let origin = Point3::new(0.0, 0.0, 0.0);
    let top = Point3::new(0.0, 0.0, 10.0);
    let edge = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&edge| {
            let data = topo.edge(edge).unwrap();
            let a = topo.vertex(data.start()).unwrap().point();
            let b = topo.vertex(data.end()).unwrap().point();
            ((a - origin).length() < 1e-9 && (b - top).length() < 1e-9)
                || ((a - top).length() < 1e-9 && (b - origin).length() < 1e-9)
        })
        .expect("box edge through the origin along z");

    let result = fillet_variable_with_setbacks(
        &mut topo,
        solid,
        &[FilletEdgeSetback {
            edge,
            law: FilletRadiusLaw::Constant(radius),
            start_setback: 0.0,
            end_setback: 0.0,
        }],
    )
    .expect("radius 9 fits the 10-wide support faces");

    // Every boundary edge of the blend wall lies on it: sample each edge and
    // measure its distance from the rolling-ball axis x = y = r.
    let wall = solid_faces(&topo, result)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Nurbs(_)))
        .expect("the variable blend wall");
    let wire = topo.wire(topo.face(wall).unwrap().outer_wire()).unwrap();
    let mut worst: f64 = 0.0;
    for oriented in wire.edges() {
        let data = topo.edge(oriented.edge()).unwrap();
        let a = topo.vertex(data.start()).unwrap().point();
        let b = topo.vertex(data.end()).unwrap().point();
        let (t0, t1) = data.strict_domain().unwrap();
        for i in 0..=16 {
            let t = (t1 - t0).mul_add(f64::from(i) / 16.0, t0);
            let p = data.curve().evaluate_with_endpoints(t, a, b);
            let distance = (p.x() - radius).hypot(p.y() - radius);
            worst = worst.max((distance - radius).abs());
        }
    }
    assert!(
        worst < 1e-7,
        "a blend-wall boundary edge leaves the wall by {worst:.3e}"
    );

    let exact = (1.0 - std::f64::consts::FRAC_PI_4).mul_add(-10.0 * radius * radius, 1000.0);
    let gauss = mass_properties(&topo, result).unwrap().mass;
    assert!(
        (gauss - exact).abs() < 1e-6 * exact,
        "Gauss {gauss:.6} against the closed form {exact:.6}"
    );
    let mesh = solid_volume(&topo, result, 0.01).unwrap();
    assert!(
        (mesh - exact).abs() < 1e-3 * exact,
        "mesh {mesh:.6} against the closed form {exact:.6}"
    );
}
