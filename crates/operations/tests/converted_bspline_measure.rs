//! Regression: a solid whose planes were converted to B-spline carriers must
//! measure and boolean like the analytic solid it came from.
//!
//! `convert_to_bspline` gives each plane a bilinear carrier 10 % wider than
//! the face on every side. Two independent defects made such a solid read
//! wrong (found 2026-09-10):
//!
//! - The non-planar CDT mesher sized its interior grid by feeding the
//!   carrier's knot span — in millimetres for a converted plane — to the
//!   circular-arc chord formula as radians, so a flat 100×10 face demanded
//!   millions of grid points, aborted, and fell back to the untrimmed
//!   rectangular carrier mesh. The converted box measured 12 933 for 10 000.
//! - The GFA face-face phase built no trimmed extent for a NURBS face, so
//!   section curves against it were clipped to the CARRIER rather than the
//!   face, never reached the face boundary, and the splitter dropped them as
//!   floating interior loops: a peg fused into the converted bar vanished.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_algo::bop::BooleanOp;
use remus_check::properties::face_integrator::integrate_face;
use remus_math::mat::Mat4;
use remus_operations::copy::copy_solid;
use remus_operations::heal::convert_to_bspline;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::tessellate::{boundary_edge_count, tessellate_solid};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;

const BAR_VOLUME: f64 = 100.0 * 10.0 * 10.0;

fn converted_bar(topo: &mut Topology) -> remus_topology::solid::SolidId {
    let bar = make_box(topo, 100.0, 10.0, 10.0).unwrap();
    let converted = convert_to_bspline(topo, bar).unwrap();
    assert!(converted > 0, "bar faces should have converted to B-spline");
    assert!(
        solid_faces(topo, bar)
            .unwrap()
            .iter()
            .all(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_))),
        "every bar face should carry a NURBS surface after conversion"
    );
    bar
}

/// A peg standing on the bar's top face, off-centre so that it also pokes
/// through the bar's side: r=3 about (50, 8), z from 7 to 13.
fn peg(topo: &mut Topology) -> remus_topology::solid::SolidId {
    let peg = make_cylinder(topo, 3.0, 6.0).unwrap();
    transform_solid(topo, peg, &Mat4::translation(50.0, 8.0, 7.0)).unwrap();
    peg
}

/// The trimmed faces, not their carriers, are what the solid measures.
#[test]
fn converted_box_volume_matches_the_box() {
    let mut topo = Topology::new();
    let bar = converted_bar(&mut topo);

    let volume = solid_volume(&topo, bar, 0.01).unwrap();
    assert!(
        (volume - BAR_VOLUME).abs() / BAR_VOLUME <= 1e-9,
        "converted box volume {volume} should equal {BAR_VOLUME} to 1e-9 relative"
    );

    // The volume comes off the whole-solid mesh, so pin that mesh too: closed,
    // and covering exactly the box's surface rather than the wider carriers.
    let mesh = tessellate_solid(&topo, bar, 0.01).unwrap();
    assert_eq!(
        boundary_edge_count(&mesh),
        0,
        "converted box mesh should be watertight"
    );
    let area: f64 = mesh
        .indices
        .chunks_exact(3)
        .map(|t| {
            let a = mesh.positions[t[0] as usize];
            let b = mesh.positions[t[1] as usize];
            let c = mesh.positions[t[2] as usize];
            (b - a).cross(c - a).length() * 0.5
        })
        .sum();
    let expected_area = 2.0 * (100.0 * 10.0 + 100.0 * 10.0 + 10.0 * 10.0);
    assert!(
        (area - expected_area).abs() / expected_area <= 1e-9,
        "converted box mesh area {area} should equal {expected_area} to 1e-9 relative"
    );
}

/// Fusing a peg into the converted bar through the raw GFA engine must add
/// exactly the part of the peg that lies outside the bar.
#[test]
fn peg_fused_into_converted_bar_adds_its_outside_part() {
    let mut topo = Topology::new();
    let bar = converted_bar(&mut topo);
    let peg = peg(&mut topo);
    let bar_copy = copy_solid(&mut topo, bar).unwrap();
    let peg_copy = copy_solid(&mut topo, peg).unwrap();

    let outside = remus_algo::gfa::boolean(&mut topo, BooleanOp::Cut, peg_copy, bar_copy).unwrap();
    let fused = remus_algo::gfa::boolean(&mut topo, BooleanOp::Fuse, bar, peg).unwrap();

    let report = validate_solid(&topo, fused).unwrap();
    assert!(report.is_valid(), "fuse result should validate: {report:?}");
    let tags: Vec<&str> = solid_faces(&topo, fused)
        .unwrap()
        .iter()
        .map(|&f| topo.face(f).unwrap().surface().type_tag())
        .collect();
    assert!(
        tags.contains(&"cylinder"),
        "fuse must keep the peg's wall as an exact cylinder, got {tags:?}"
    );

    let bar_volume = BAR_VOLUME;
    let outside_volume = solid_volume(&topo, outside, 0.01).unwrap();
    let fused_volume = solid_volume(&topo, fused, 0.01).unwrap();

    // Closed form of the peg outside the bar: the r=3 disc times the 3 above
    // the bar's top, plus the circular segment beyond y=10 (chord 2 from the
    // centre) times the 3 inside the bar's height.
    let segment = 9.0 * (2.0_f64 / 3.0).acos() - 2.0 * 5.0_f64.sqrt();
    let outside_closed_form = std::f64::consts::PI * 9.0 * 3.0 + segment * 3.0;
    assert!(
        (outside_volume - outside_closed_form).abs() / outside_closed_form <= 1e-3,
        "peg-minus-bar volume {outside_volume} should be near {outside_closed_form}"
    );

    // Both measurements come off inscribed meshes of the same peg-wall
    // pieces at the default volume deflection (5e-5 of the diagonal), so
    // they agree to that chord deficit — 1e-5 of the body. The defect this
    // pins lost the whole peg: 94 units, a thousand times this bound.
    let expected = bar_volume + outside_volume;
    assert!(
        (fused_volume - expected).abs() / expected <= 1e-5,
        "fuse volume {fused_volume} should equal bar {bar_volume} + outside {outside_volume} = {expected}"
    );

    // The exact face integrator has no mesh deficit: every face of both
    // results is an exact plane, cylinder, or bilinear patch bounded by exact
    // arcs and lines, so the integrated volumes must land on the closed form
    // to the integrator's own boundary-sampling error — it chords each curved
    // edge into 128 pieces, which its own documentation puts at ~5e-5 of a
    // body. Before the integrator bounded a NURBS face by its vertices alone
    // and lost 2.09 of the 25.18 disc the peg cuts from the top: 2e-2.
    let integrated = |solid| -> f64 {
        solid_faces(&topo, solid)
            .unwrap()
            .iter()
            .map(|&f| integrate_face(&topo, f, 8).unwrap().volume)
            .sum()
    };
    let fused_exact = integrated(fused);
    let outside_exact = integrated(outside);
    assert!(
        (outside_exact - outside_closed_form).abs() / outside_closed_form <= 1e-4,
        "integrated peg-minus-bar volume {outside_exact} should equal {outside_closed_form}"
    );
    assert!(
        (fused_exact - (bar_volume + outside_closed_form)).abs() / fused_exact <= 1e-4,
        "integrated fuse volume {fused_exact} should equal {}",
        bar_volume + outside_closed_form
    );
}
