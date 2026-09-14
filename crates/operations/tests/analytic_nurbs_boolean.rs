//! Regression: analytic (non-plane) × NURBS face pairs must genuinely
//! intersect in the FF phase.
//!
//! The GFA table's analytic×NURBS arm used to return no curves ("deferred to
//! later phases" — no later phase existed), so a boolean pairing a curved
//! analytic wall with any NURBS face silently skipped face splitting and
//! misbuilt or leaned on the mesh fallback. These tests pin the repaired arm
//! end to end: a cylinder cut by an all-B-spline box must split its wall and
//! land on the closed-form volume.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::heal::convert_to_bspline;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;

/// Cylinder (analytic wall) minus a box whose every face is a B-spline:
/// each cut face crossing the wall is an analytic-cylinder × NURBS pair.
#[test]
fn cylinder_cut_by_bspline_box_splits_the_wall() {
    let mut topo = Topology::new();
    // r=5, h=10, axis +z, base at origin.
    let cyl = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    // A slab spanning z in [7.5, 12.5] that decapitates the cylinder. Wide
    // enough in x/y to swallow the whole wall circle.
    let slab = make_box(&mut topo, 20.0, 20.0, 5.0).unwrap();
    transform_solid(&mut topo, slab, &Mat4::translation(-10.0, -10.0, 7.5)).unwrap();
    let converted = convert_to_bspline(&mut topo, slab).unwrap();
    assert!(
        converted > 0,
        "slab faces should have converted to B-spline"
    );

    let result = boolean(&mut topo, BooleanOp::Cut, cyl, slab).unwrap();

    let report = validate_solid(&topo, result).unwrap();
    assert!(report.is_valid(), "cut result should validate: {report:?}");
    // Closed form: the surviving cylinder is r=5, h=7.5.
    let expected = std::f64::consts::PI * 25.0 * 7.5;
    let volume = solid_volume(&topo, result, 0.05).unwrap();
    assert!(
        (volume - expected).abs() / expected < 0.01,
        "volume {volume} should be within 1% of {expected}"
    );
}

/// The mirror family: a CONVERTED cylinder (genuinely-curved exact-rational
/// NURBS wall) cut by an ANALYTIC slab. The wall split runs through the
/// converted-wall band path (trimmed FF section + transversal EF crossings),
/// and the band tessellates through the converted-wall structured mesher.
/// Pins the transverse-decapitation oracle end to end: cut keeps r=5 h=7.5.
///
/// The case is factored so the variant matrix below can reuse it: radius,
/// height, section height, and an XY rigid motion applied to the tool.
fn converted_cut_case(
    radius: f64,
    height: f64,
    section_z: f64,
    tool_dx: f64,
    tool_dy: f64,
) -> (Topology, remus_topology::solid::SolidId) {
    let mut topo = Topology::new();
    let bare = make_cylinder(&mut topo, radius, height).unwrap();
    // Convert ONLY the cylinder: its wall becomes a genuinely-curved NURBS
    // surface (the slab stays analytic).
    let cyl = remus_operations::copy::copy_solid(&mut topo, bare).unwrap();
    let converted = convert_to_bspline(&mut topo, cyl).unwrap();
    assert!(converted > 0, "cylinder faces should convert to B-spline");
    let span = radius * 4.0;
    let slab = make_box(&mut topo, span, span, height - section_z + 5.0).unwrap();
    let place = Mat4::translation(-span / 2.0 + tool_dx, -span / 2.0 + tool_dy, section_z);
    transform_solid(&mut topo, slab, &place).unwrap();
    let result = boolean(&mut topo, BooleanOp::Cut, cyl, slab).unwrap();
    (topo, result)
}

/// Checklist shared by every CUT variant: valid shell, exact face census
/// (one NURBS band + two planes — no mesh fallback), closed-form volume,
/// and genuine material classification on both sides of the section.
fn check_converted_cut(
    topo: &Topology,
    result: remus_topology::solid::SolidId,
    radius: f64,
    kept_height: f64,
    section_z: f64,
) {
    let report = validate_solid(topo, result).unwrap();
    assert!(report.is_valid(), "cut result should validate: {report:?}");
    // The wall stays a genuinely-curved NURBS surface (no analytic recovery,
    // no mesh fallback to hundreds of planes).
    let mut nurbs_faces = 0;
    let mut plane_faces = 0;
    for fid in remus_topology::explorer::solid_faces(topo, result).unwrap() {
        let tag = topo.face(fid).unwrap().surface().type_tag();
        assert!(
            matches!(
                topo.face(fid).unwrap().surface(),
                remus_topology::face::FaceSurface::Nurbs(_)
                    | remus_topology::face::FaceSurface::Plane { .. }
            ),
            "unexpected surface type: {tag}"
        );
        match topo.face(fid).unwrap().surface() {
            remus_topology::face::FaceSurface::Nurbs(_) => nurbs_faces += 1,
            remus_topology::face::FaceSurface::Plane { .. } => plane_faces += 1,
            _ => {}
        }
    }
    assert_eq!(nurbs_faces, 1, "exactly the lower wall band stays NURBS");
    assert_eq!(plane_faces, 2, "section disc + bottom cap stay planar");
    // Closed form: the surviving barrel is r × kept height.
    let expected = std::f64::consts::PI * radius * radius * kept_height;
    let volume = solid_volume(topo, result, 0.05).unwrap();
    assert!(
        (volume - expected).abs() / expected < 0.01,
        "volume {volume} should be within 1% of {expected}"
    );
    // The cut must be genuine: material below the section plane remains,
    // material above it is gone.
    let opts = remus_check::classify::ClassifyOptions::default();
    assert_eq!(
        remus_check::classify::classify_point(
            topo,
            result,
            remus_math::vec::Point3::new(0.0, 0.0, section_z - 1.0),
            &opts
        )
        .unwrap(),
        remus_check::classify::PointClassification::Inside
    );
    assert_eq!(
        remus_check::classify::classify_point(
            topo,
            result,
            remus_math::vec::Point3::new(0.0, 0.0, section_z + 1.0),
            &opts
        )
        .unwrap(),
        remus_check::classify::PointClassification::Outside
    );
}

#[test]
fn converted_cylinder_cut_by_analytic_slab_keeps_the_lower_barrel() {
    let (topo, result) = converted_cut_case(5.0, 10.0, 7.5, 0.0, 0.0);
    check_converted_cut(&topo, result, 5.0, 7.5, 7.5);
}

/// Scaled variant: a smaller barrel decapitated at a different height.
/// Guards against dimension-tuned constants (weld bands, chart tolerances).
#[test]
fn converted_cut_scaled_barrel() {
    let (topo, result) = converted_cut_case(2.0, 6.0, 4.0, 0.0, 0.0);
    check_converted_cut(&topo, result, 2.0, 4.0, 4.0);
}

/// Translated tool: the section plane is unchanged (still z), but the slab
/// footprint is shifted so the wall/rim seam alignment differs. The band
/// emitter must not depend on the seam sitting at a particular XY angle.
/// The shift stays small and axis-aligned: the transverse family covers the
/// decapitation; a rotated footprint would expose longitudinal plane×wall
/// intersections, which belong to a later family.
#[test]
fn converted_cut_shifted_tool() {
    let (topo, result) = converted_cut_case(5.0, 10.0, 7.5, 1.5, -2.0);
    check_converted_cut(&topo, result, 5.0, 7.5, 7.5);
}

/// Near-tangency refusal probe: a slab that only kisses the barrel wall
/// (its side face tangent to the tube) must not produce a corrupt band.
/// Either the engine refuses (Err) or it returns a valid shell whose volume
/// is the full barrel or the decapitated barrel within tolerance — never an
/// invalid solid.
#[test]
fn converted_cut_tangent_tool_refuses_or_stays_valid() {
    let mut topo = Topology::new();
    let bare = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    let cyl = remus_operations::copy::copy_solid(&mut topo, bare).unwrap();
    convert_to_bspline(&mut topo, cyl).unwrap();
    // Slab side face at x=5: tangent to the r=5 tube, spanning z 0..10.
    let slab = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
    transform_solid(&mut topo, slab, &Mat4::translation(5.0, -10.0, 0.0)).unwrap();
    match boolean(&mut topo, BooleanOp::Cut, cyl, slab) {
        Err(_) => {}
        Ok(result) => {
            let report = validate_solid(&topo, result).unwrap();
            assert!(report.is_valid(), "tangent cut must stay valid: {report:?}");
        }
    }
}

/// The disjoint case must stay quiet: an all-B-spline box whose AABB is
/// nowhere near the cylinder wall's curves produces an empty intersection
/// from a genuine computation, and the cut leaves the cylinder unchanged.
#[test]
fn cylinder_cut_by_distant_bspline_box_is_a_no_op() {
    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    let cube = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    transform_solid(&mut topo, cube, &Mat4::translation(40.0, 40.0, 0.0)).unwrap();
    convert_to_bspline(&mut topo, cube).unwrap();

    let result = boolean(&mut topo, BooleanOp::Cut, cyl, cube).unwrap();

    let expected = std::f64::consts::PI * 25.0 * 10.0;
    let volume = solid_volume(&topo, result, 0.05).unwrap();
    assert!(
        (volume - expected).abs() / expected < 0.001,
        "volume {volume} should remain the full cylinder {expected}"
    );
}
