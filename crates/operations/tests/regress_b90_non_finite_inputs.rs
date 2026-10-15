//! B90: public operation entry points must refuse NaN and infinite scalars.
//!
//! Positivity checks written `x <= 0.0` (or `x <= tol`, `approx_eq(x, 0.0)`)
//! are false for NaN, and most are false for +∞, so a non-finite distance,
//! radius, angle or spacing walked past validation. Some entry points then
//! returned `Ok` with non-finite geometry: a planar `offset_face` by NaN came
//! back `Exact` with NaN vertices, `extrude` and `offset_wire` built NaN
//! points, and a sweep scale law returning NaN was read as "no scaling".
//! Others failed later with a misleading error (a degenerate transform, a
//! NURBS control-point fault, a zero vector).
//!
//! Contract: every entry point below answers `InvalidInput` for NaN, +∞ and
//! (where a negative value is otherwise refused or meaningless) −∞, and
//! leaves no new face behind.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, deprecated)]

use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::sweep::SweepOptions;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;
use remus_topology::test_utils::{make_unit_cube_manifold, make_unit_square_face};

const BAD: [f64; 3] = [f64::NAN, f64::INFINITY, f64::NEG_INFINITY];

struct Fixture {
    topo: Topology,
    cube: SolidId,
    square: FaceId,
    edge: EdgeId,
    top: FaceId,
}

fn fixture() -> Fixture {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    let square = make_unit_square_face(&mut topo);
    let faces = remus_topology::explorer::solid_faces(&topo, cube).unwrap();
    let top = faces[0];
    let wire = topo.wire(topo.face(top).unwrap().outer_wire()).unwrap();
    let edge = wire.edges()[0].edge();
    Fixture {
        topo,
        cube,
        square,
        edge,
        top,
    }
}

/// Live faces: a refusal before any geometry is built leaves none behind.
fn counts(topo: &Topology) -> usize {
    topo.faces().len()
}

/// Run `op` on a fresh fixture with each bad value; it must refuse with
/// `InvalidInput` and allocate nothing.
fn refuses<T>(name: &str, op: impl Fn(&mut Fixture, f64) -> Result<T, OperationsError>) {
    for value in BAD {
        let mut f = fixture();
        let before = counts(&f.topo);
        let result = op(&mut f, value);
        assert!(
            matches!(result, Err(OperationsError::InvalidInput { .. })),
            "{name}({value}) must refuse as InvalidInput, got {:?}",
            result.as_ref().err()
        );
        assert_eq!(counts(&f.topo), before, "{name}({value}) left faces behind");
    }
}

#[test]
fn b90_offsets_and_sweeps_refuse_non_finite_distances() {
    refuses("offset_face_with_quality", |f, d| {
        remus_operations::offset_face::offset_face_with_quality(&mut f.topo, f.square, d, None)
    });
    refuses("extrude distance", |f, d| {
        remus_operations::extrude::extrude(&mut f.topo, f.square, Vec3::new(0.0, 0.0, 1.0), d)
    });
    refuses("extrude direction", |f, d| {
        remus_operations::extrude::extrude(&mut f.topo, f.square, Vec3::new(0.0, 0.0, d), 1.0)
    });
    refuses("offset_wire", |f, d| {
        remus_operations::offset_wire::offset_wire(&mut f.topo, f.square, d)
    });
    refuses("revolve", |f, a| {
        remus_operations::revolve::revolve(
            &mut f.topo,
            f.square,
            Point3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            a,
        )
    });
    refuses("sweep scale law", |f, s| {
        let path = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::new(0.5, 0.5, 0.0), Point3::new(0.5, 0.5, 5.0)],
            vec![1.0, 1.0],
        )
        .unwrap();
        let options = SweepOptions {
            scale_law: Some(Box::new(move |t| if t > 0.5 { s } else { 1.0 })),
            ..SweepOptions::default()
        };
        remus_operations::sweep::sweep_with_options(&mut f.topo, f.square, &path, &options)
    });
    // The corner and closed-path builders never evaluate the law; it is
    // refused before dispatch all the same.
    let kinked = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 0.5, 1.0, 1.0],
        vec![
            Point3::new(0.5, 0.5, 0.0),
            Point3::new(0.5, 0.5, 5.0),
            Point3::new(5.5, 0.5, 5.0),
        ],
        vec![1.0, 1.0, 1.0],
    )
    .unwrap();
    let closed = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, 1.0],
        vec![
            Point3::new(0.5, 0.5, 0.0),
            Point3::new(0.5, 0.5, 5.0),
            Point3::new(5.5, 0.5, 5.0),
            Point3::new(0.5, 0.5, 0.0),
        ],
        vec![1.0; 4],
    )
    .unwrap();
    for (name, path, corner_mode) in [
        (
            "miter",
            kinked.clone(),
            remus_operations::sweep::SweepCornerMode::Miter,
        ),
        (
            "round",
            kinked,
            remus_operations::sweep::SweepCornerMode::Round { radius: 0.5 },
        ),
        (
            "closed",
            closed,
            remus_operations::sweep::SweepCornerMode::Smooth,
        ),
    ] {
        refuses(&format!("sweep scale law ({name})"), |f, s| {
            let options = SweepOptions {
                corner_mode,
                scale_law: Some(Box::new(move |t| if t > 0.5 { s } else { 1.0 })),
                ..SweepOptions::default()
            };
            remus_operations::sweep::sweep_with_options(&mut f.topo, f.square, &path, &options)
        });
    }
}

#[test]
fn b90_helix_refuses_non_finite_radius_and_pitch() {
    refuses("helical_sweep radius", |f, r| {
        remus_operations::helix::helical_sweep(
            &mut f.topo,
            f.square,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            r,
            2.0,
            1.0,
            16,
        )
    });
    refuses("helical_sweep pitch", |f, p| {
        remus_operations::helix::helical_sweep(
            &mut f.topo,
            f.square,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            5.0,
            p,
            1.0,
            16,
        )
    });
    for value in BAD {
        for (radius, pitch) in [(value, 1.0), (1.0, value)] {
            let curve = remus_operations::helix::make_helix_curve(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                radius,
                pitch,
                1.0,
                16,
            );
            assert!(
                matches!(curve, Err(OperationsError::InvalidInput { .. })),
                "make_helix_curve({radius}, {pitch}) must refuse, got {curve:?}"
            );
        }
    }
}

#[test]
fn b90_blends_refuse_non_finite_sizes() {
    refuses("chamfer", |f, d| {
        remus_operations::chamfer::chamfer(&mut f.topo, f.cube, &[f.edge], d)
    });
    refuses("chamfer_asymmetric", |f, d| {
        remus_operations::chamfer::chamfer_asymmetric(&mut f.topo, f.cube, &[f.edge], 0.1, d)
    });
    refuses("fillet_v2", |f, r| {
        remus_operations::blend_ops::fillet_v2(&mut f.topo, f.cube, &[f.edge], r)
    });
    refuses("chamfer_v2", |f, d| {
        remus_operations::blend_ops::chamfer_v2(&mut f.topo, f.cube, &[f.edge], d, 0.1)
    });
    refuses("chamfer_distance_angle distance", |f, d| {
        remus_operations::blend_ops::chamfer_distance_angle(&mut f.topo, f.cube, &[f.edge], d, 0.5)
    });
    refuses("chamfer_distance_angle angle", |f, a| {
        remus_operations::blend_ops::chamfer_distance_angle(&mut f.topo, f.cube, &[f.edge], 0.1, a)
    });
    refuses("fillet_rolling_ball", |f, r| {
        remus_operations::fillet::fillet_rolling_ball(&mut f.topo, f.cube, &[f.edge], r)
    });
    refuses("fillet", |f, r| {
        remus_operations::fillet::fillet(&mut f.topo, f.cube, &[f.edge], r)
    });
}

#[test]
fn b90_body_edits_refuse_non_finite_parameters() {
    refuses("shell", |f, t| {
        remus_operations::shell_op::shell(&mut f.topo, f.cube, t, &[f.top])
    });
    refuses("draft", |f, a| {
        remus_operations::draft::draft(
            &mut f.topo,
            f.cube,
            &[f.top],
            Vec3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 0.0),
            a,
        )
    });
    refuses("linear_pattern", |f, s| {
        remus_operations::pattern::linear_pattern(
            &mut f.topo,
            f.cube,
            Vec3::new(1.0, 0.0, 0.0),
            s,
            3,
        )
    });
    refuses("grid_pattern", |f, s| {
        remus_operations::pattern::grid_pattern(
            &mut f.topo,
            f.cube,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            2.0,
            s,
            2,
            2,
        )
    });
    refuses("detect_small_features threshold", |f, a| {
        remus_operations::defeature::detect_small_features(&f.topo, f.cube, a, 0.01)
    });
    refuses("detect_small_features deflection", |f, d| {
        remus_operations::defeature::detect_small_features(&f.topo, f.cube, 0.5, d)
    });
}

/// The blend engine's own builder refuses before building contact points.
#[test]
fn b90_chamfer_builder_refuses_non_finite_distances() {
    for value in BAD {
        for (d1, d2) in [(value, 0.1), (0.1, value)] {
            let mut f = fixture();
            let mut builder =
                remus_blend::chamfer_builder::ChamferBuilder::new(&mut f.topo, f.cube);
            builder.add_edges_asymmetric(&[f.edge], d1, d2);
            let result = builder.build();
            assert!(
                matches!(result, Err(remus_blend::BlendError::InvalidInput { .. })),
                "ChamferBuilder({d1}, {d2}) must refuse, got {:?}",
                result.err()
            );
        }
        let mut f = fixture();
        let mut builder = remus_blend::chamfer_builder::ChamferBuilder::new(&mut f.topo, f.cube);
        builder.add_edges_distance_angle(&[f.edge], 0.1, value);
        assert!(
            matches!(
                builder.build(),
                Err(remus_blend::BlendError::InvalidInput { .. })
            ),
            "ChamferBuilder angle {value} must refuse"
        );
    }
}

/// The offset engine refuses a non-finite tolerance instead of carrying it
/// into every intersection.
#[test]
fn b90_thick_solid_refuses_a_non_finite_tolerance() {
    for value in BAD {
        for field in ["linear", "angular", "relative"] {
            let mut f = fixture();
            let mut options = remus_offset::OffsetOptions::default();
            match field {
                "linear" => options.tolerance.linear = value,
                "angular" => options.tolerance.angular = value,
                _ => options.tolerance.relative = value,
            }
            let result = remus_offset::thick_solid(&mut f.topo, f.cube, 0.1, &[f.top], options);
            assert!(
                matches!(result, Err(remus_offset::OffsetError::InvalidInput { .. })),
                "thick_solid with {field} tolerance {value} must refuse, got {result:?}"
            );
        }
    }
}
