//! Public property-option regression witnesses; closed-form sphere oracle.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::CheckError;
use remus_check::properties::{
    PropertiesOptions, center_of_mass, solid_area, solid_properties, solid_volume,
};
use remus_operations::primitives::make_sphere;
use remus_topology::Topology;

#[test]
fn invalid_tolerances_are_rejected_by_every_property_entry_point() {
    let mut topo = Topology::new();
    let solid = make_sphere(&mut topo, 2.0, 16).unwrap();
    for eps in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.0, -1e-6] {
        let opts = PropertiesOptions {
            adaptive_eps: eps,
            ..Default::default()
        };
        assert!(
            matches!(
                solid_volume(&topo, solid, &opts),
                Err(CheckError::IntegrationFailed(_))
            ),
            "volume accepted {eps}"
        );
        assert!(matches!(
            solid_area(&topo, solid, &opts),
            Err(CheckError::IntegrationFailed(_))
        ));
        assert!(matches!(
            center_of_mass(&topo, solid, &opts),
            Err(CheckError::IntegrationFailed(_))
        ));
        assert!(matches!(
            solid_properties(&topo, solid, &opts),
            Err(CheckError::IntegrationFailed(_))
        ));
    }
}

#[test]
fn insufficient_depth_refuses_instead_of_returning_unconverged_properties() {
    let mut topo = Topology::new();
    let solid = make_sphere(&mut topo, 2.0, 16).unwrap();
    let opts = PropertiesOptions {
        gauss_order: 2,
        adaptive_eps: 1e-10,
        max_depth: 0,
    };
    assert!(matches!(
        solid_volume(&topo, solid, &opts),
        Err(CheckError::IntegrationFailed(_))
    ));
    assert!(matches!(
        solid_area(&topo, solid, &opts),
        Err(CheckError::IntegrationFailed(_))
    ));
    assert!(matches!(
        center_of_mass(&topo, solid, &opts),
        Err(CheckError::IntegrationFailed(_))
    ));
    assert!(matches!(
        solid_properties(&topo, solid, &opts),
        Err(CheckError::IntegrationFailed(_))
    ));
}

fn near(actual: f64, expected: f64, scale: f64) {
    assert!(
        (actual - expected).abs() <= 1e-8 * scale,
        "actual {actual:e}, expected {expected:e}, scale {scale:e}"
    );
}

#[test]
fn refinement_converges_to_sphere_truth_at_three_physical_scales() {
    use std::f64::consts::PI;
    for scale in [1e-3, 1.0, 1e3] {
        let mut topo = Topology::new();
        let r: f64 = 2.0 * scale;
        let solid = make_sphere(&mut topo, r, 16).unwrap();
        let mut opts = PropertiesOptions {
            gauss_order: 3,
            adaptive_eps: 1e-9,
            max_depth: 0,
        };
        assert!(solid_properties(&topo, solid, &opts).is_err());
        opts.max_depth = 8;
        let props = solid_properties(&topo, solid, &opts).unwrap();
        let volume = 4.0 * PI * r.powi(3) / 3.0;
        near(props.mass, volume, volume);
        near(solid_volume(&topo, solid, &opts).unwrap(), volume, volume);
        near(
            solid_area(&topo, solid, &opts).unwrap(),
            4.0 * PI * r * r,
            4.0 * PI * r * r,
        );
        let center = center_of_mass(&topo, solid, &opts).unwrap();
        for x in [
            props.center.x(),
            props.center.y(),
            props.center.z(),
            center.x(),
            center.y(),
            center.z(),
        ] {
            near(x, 0.0, r);
        }
        let inertia = 2.0 * volume * r * r / 5.0;
        for i in 0..6 {
            near(props.inertia[i], if i < 3 { inertia } else { 0.0 }, inertia);
        }
    }
}

#[test]
fn exact_planar_box_needs_no_refinement_but_still_validates_options() {
    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let options = PropertiesOptions {
        gauss_order: 1,
        adaptive_eps: 1e-14,
        max_depth: 0,
    };
    let props = solid_properties(&topo, solid, &options).unwrap();
    near(props.mass, 24.0, 24.0);
    near(solid_area(&topo, solid, &options).unwrap(), 52.0, 52.0);
    for (got, expected) in [props.center.x(), props.center.y(), props.center.z()]
        .into_iter()
        .zip([1.0, 1.5, 2.0])
    {
        near(got, expected, 4.0);
    }
    for (got, expected) in props
        .inertia
        .into_iter()
        .zip([50.0, 40.0, 26.0, 0.0, 0.0, 0.0])
    {
        near(got, expected, 50.0);
    }
    for order in [0, 21, usize::MAX] {
        let invalid = PropertiesOptions {
            gauss_order: order,
            ..options.clone()
        };
        assert!(solid_volume(&topo, solid, &invalid).is_err());
        assert!(solid_area(&topo, solid, &invalid).is_err());
        assert!(center_of_mass(&topo, solid, &invalid).is_err());
        assert!(solid_properties(&topo, solid, &invalid).is_err());
    }
}

#[test]
fn cavity_signs_centroid_products_and_mm_powers_match_closed_forms() {
    use remus_math::mat::Mat4;
    use remus_operations::transform::transform_solid;
    use remus_topology::{explorer::solid_faces, solid::Solid};
    use std::f64::consts::PI;
    for scale in [1e-3, 1.0, 1e3] {
        let mut topo = Topology::new();
        let ro: f64 = 4.0 * scale;
        let ri: f64 = scale;
        let d = [0.3 * scale, -0.4 * scale, 0.7 * scale];
        let outer = make_sphere(&mut topo, ro, 16).unwrap();
        let inner = make_sphere(&mut topo, ri, 16).unwrap();
        transform_solid(&mut topo, inner, &Mat4::translation(d[0], d[1], d[2])).unwrap();
        for fid in solid_faces(&topo, inner).unwrap() {
            topo.face_mut(fid).unwrap().set_reversed(true);
        }
        let solid = topo.add_solid(Solid::new(
            topo.solid(outer).unwrap().outer_shell(),
            vec![topo.solid(inner).unwrap().outer_shell()],
        ));
        let vo = 4.0 * PI * ro.powi(3) / 3.0;
        let vi = 4.0 * PI * ri.powi(3) / 3.0;
        let volume = vo - vi;
        let c = d.map(|x| -vi * x / volume);
        let q = d.map(|x| vo * ro * ro / 5.0 - vi * (ri * ri / 5.0 + x * x));
        let diagonal = [
            q[1] + q[2] - volume * (c[1] * c[1] + c[2] * c[2]),
            q[0] + q[2] - volume * (c[0] * c[0] + c[2] * c[2]),
            q[0] + q[1] - volume * (c[0] * c[0] + c[1] * c[1]),
        ];
        let options = PropertiesOptions {
            gauss_order: 3,
            adaptive_eps: 1e-9,
            max_depth: 8,
        };
        let props = solid_properties(&topo, solid, &options).unwrap();
        near(props.mass, volume, volume);
        near(
            solid_volume(&topo, solid, &options).unwrap(),
            volume,
            volume,
        );
        let area = 4.0 * PI * (ro * ro + ri * ri);
        near(solid_area(&topo, solid, &options).unwrap(), area, area);
        let center = center_of_mass(&topo, solid, &options).unwrap();
        for (got, expected) in [center.x(), center.y(), center.z()].into_iter().zip(c) {
            near(got, expected, ro);
        }
        for (got, expected) in [props.center.x(), props.center.y(), props.center.z()]
            .into_iter()
            .zip(c)
        {
            near(got, expected, ro);
        }
        for (got, expected) in props.inertia[..3].iter().zip(diagonal) {
            near(*got, expected, diagonal[0]);
        }
        for (k, (i, j)) in [(0, 1), (0, 2), (1, 2)].into_iter().enumerate() {
            near(
                props.inertia[3 + k],
                -vi * d[i] * d[j] - volume * c[i] * c[j],
                diagonal[0],
            );
        }
        let matrix = props.matrix_of_inertia();
        near(
            matrix[0][1],
            vi * d[0] * d[1] + volume * c[0] * c[1],
            diagonal[0],
        );
    }
}

#[test]
fn impossible_tolerance_refuses_without_mutating_topology() {
    let mut topo = Topology::new();
    let solid = make_sphere(&mut topo, 2.0, 16).unwrap();
    let before = format!("{topo:?}");
    let opts = PropertiesOptions {
        gauss_order: 1,
        adaptive_eps: 1e-30,
        max_depth: usize::MAX,
    };
    let error = solid_properties(&topo, solid, &opts).unwrap_err();
    assert!(matches!(error, CheckError::IntegrationFailed(_)));
    assert_eq!(before, format!("{topo:?}"));
    assert!(solid_properties(&topo, solid, &PropertiesOptions::default()).is_ok());
}

#[test]
fn tightening_eps_improves_a_low_order_inertia_estimate() {
    let mut topo = Topology::new();
    let solid = make_sphere(&mut topo, 2.0, 16).unwrap();
    let loose = PropertiesOptions {
        gauss_order: 3,
        adaptive_eps: 1e-2,
        max_depth: 0,
    };
    let tight = PropertiesOptions {
        adaptive_eps: 1e-9,
        max_depth: 8,
        ..loose
    };
    let coarse = solid_properties(&topo, solid, &loose).unwrap();
    let refined = solid_properties(&topo, solid, &tight).unwrap();
    let truth = 2.0 / 5.0 * (4.0 / 3.0 * std::f64::consts::PI * 8.0) * 4.0;
    let coarse_error = (coarse.inertia[0] - truth).abs();
    let refined_error = (refined.inertia[0] - truth).abs();
    assert!(
        coarse_error > 1e-8,
        "fixture must require refinement: {coarse_error:e}"
    );
    assert!(
        refined_error < coarse_error / 100.0,
        "coarse {coarse_error:e}, refined {refined_error:e}"
    );
}

#[test]
fn nonfinite_integrands_and_work_exhaustion_are_typed_refusals() {
    let mut topo = Topology::new();
    let huge = make_sphere(&mut topo, 1e70, 16).unwrap();
    let error = solid_properties(&topo, huge, &PropertiesOptions::default()).unwrap_err();
    assert!(matches!(error, CheckError::IntegrationFailed(_)));
    assert!(error.to_string().contains("non-finite"), "{error}");
    assert!(matches!(
        remus_operations::measure::mass_properties(&topo, huge),
        Err(remus_operations::OperationsError::Check(
            CheckError::IntegrationFailed(_)
        ))
    ));
    assert!(matches!(
        remus_operations::measure::solid_center_of_mass(&topo, huge, 0.1),
        Err(remus_operations::OperationsError::Check(
            CheckError::IntegrationFailed(_)
        ))
    ));
    let solid = make_sphere(&mut topo, 2.0, 16).unwrap();
    let options = PropertiesOptions {
        gauss_order: 1,
        adaptive_eps: 1e-8,
        max_depth: 20,
    };
    let error = solid_area(&topo, solid, &options).unwrap_err();
    assert!(matches!(error, CheckError::IntegrationFailed(_)));
    assert!(error.to_string().contains("work budget"), "{error}");
}

#[test]
fn analytic_revolution_domains_match_independent_closed_form_properties() {
    use remus_check::properties::analytic;
    use remus_operations::primitives::{make_cone, make_cylinder, make_torus};
    let mut topo = Topology::new();
    let fixtures = [
        (
            make_cylinder(&mut topo, 2.0, 3.0).unwrap(),
            analytic::cylinder_props(2.0, 3.0),
            analytic::cylinder_area(2.0, 3.0),
        ),
        (
            make_cone(&mut topo, 2.0, 0.0, 3.0).unwrap(),
            analytic::cone_props(2.0, 0.0, 3.0),
            2.0 * std::f64::consts::PI * (2.0 + 13.0_f64.sqrt()),
        ),
        (
            make_torus(&mut topo, 3.0, 1.0, 16).unwrap(),
            analytic::torus_props(3.0, 1.0),
            analytic::torus_area(3.0, 1.0),
        ),
    ];
    let options = PropertiesOptions {
        adaptive_eps: 1e-9,
        ..Default::default()
    };
    for (index, (solid, expected, area)) in fixtures.into_iter().enumerate() {
        let options = if index == 0 {
            // The primitive cylinder retains a polygon mask around its seam.
            let error = solid_properties(&topo, solid, &options).unwrap_err();
            assert!(error.to_string().contains("unsupported"), "{error}");
            assert!(solid_volume(&topo, solid, &options).is_err());
            assert!(solid_area(&topo, solid, &options).is_err());
            assert!(center_of_mass(&topo, solid, &options).is_err());
            PropertiesOptions::default()
        } else {
            options.clone()
        };
        let actual = solid_properties(&topo, solid, &options).unwrap();
        near(actual.mass, expected.mass, expected.mass);
        near(solid_area(&topo, solid, &options).unwrap(), area, area);
        near(actual.center.x(), expected.center.x(), 3.0);
        near(actual.center.y(), expected.center.y(), 3.0);
        near(actual.center.z(), expected.center.z(), 3.0);
        for (a, e) in actual.inertia.into_iter().zip(expected.inertia) {
            near(a, e, expected.inertia[0]);
        }
    }
}
