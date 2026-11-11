#![allow(clippy::unwrap_used)]
use crate::kernel::RemusKernel;
use remus_math::vec::{Point3, Vec3};
use remus_topology::{
    builder::make_polygon_wire,
    face::{Face, FaceSurface},
    wire::WireId,
};
fn rect(k: &mut RemusKernel, b: [f64; 4], cw: bool) -> WireId {
    let [a, b, c, d] = b;
    let mut p = vec![
        Point3::new(a, c, 5.),
        Point3::new(b, c, 5.),
        Point3::new(b, d, 5.),
        Point3::new(a, d, 5.),
    ];
    if cw {
        p.reverse();
    }
    make_polygon_wire(k.topo_mut(), &p, 1e-7).unwrap()
}
fn setup(k: &mut RemusKernel, curved_island: bool) -> u32 {
    let o = rect(k, [-20., 20., -20., 20.], false);
    let h = rect(k, [-7., 1., -4., 4.], true);
    let island = rect(k, [-3., -1., 0., 2.], false);
    if curved_island {
        let e = k.topo().wire(island).unwrap().edges()[0].edge();
        let d = k.topo().edge(e).unwrap();
        let a = k.topo().vertex(d.start()).unwrap().point();
        let b = k.topo().vertex(d.end()).unwrap().point();
        let n = remus_math::nurbs::NurbsCurve::new(
            2,
            vec![0., 0., 0., 1., 1., 1.],
            vec![a, a + (b - a) * 0.5 + Vec3::new(0., 0.2, 0.), b],
            vec![1.; 3],
        )
        .unwrap();
        k.topo_mut()
            .edge_mut(e)
            .unwrap()
            .set_curve(remus_topology::edge::EdgeCurve::NurbsCurve(n));
    }
    k.topo_mut()
        .add_face(Face::new(
            o,
            vec![h, island],
            FaceSurface::Plane {
                normal: Vec3::new(0., 0., 1.),
                d: 5.,
            },
        ))
        .index() as u32
}
fn snapshot(k: &RemusKernel) -> String {
    format!(
        "{:?}|{:?}|{:?}|{:?}|{:?}|{:?}|{}",
        k.topo(),
        k.assemblies,
        k.checkpoints,
        k.sketches,
        k.gcs_sketches,
        k.poisoned,
        k.topo().num_solids()
    )
}
#[test]
fn profile_nesting_refusal_rollback() {
    // This positive-weight spline is supported; force an undecided work
    // budget to retain the category and complete rollback contract.
    remus_check::properties::loop_nesting::with_nesting_work_limit(0, || {
        for v2 in [false, true] {
            for detailed in [false, true] {
                let mut k = RemusKernel::new();
                let f = setup(&mut k, true);
                let retained = k.make_box_solid(1., 2., 3.).unwrap();
                let document = k.serialize_solids(&[retained]).unwrap();
                k.checkpoint().unwrap();
                let before = snapshot(&k);
                let op = if detailed {
                    "extrudeDetailed"
                } else {
                    "extrude"
                };
                let request=serde_json::json!([{"op":op,"args":{"face":f,"dx":0.,"dy":0.,"dz":1.,"distance":2.}}]).to_string();
                let response = if v2 {
                    k.execute_batch_v2(&request)
                } else {
                    k.execute_batch(&request)
                };
                assert!(
                    response.contains("unsupported")
                        || response.contains("nesting could not be certified")
                );
                assert_eq!(snapshot(&k), before);
                assert_eq!(k.serialize_solids(&[retained]).unwrap(), document);
            }
        }
        let mut k = RemusKernel::new();
        let f = setup(&mut k, true);
        let retained = k.make_box_solid(1., 2., 3.).unwrap();
        let document = k.serialize_solids(&[retained]).unwrap();
        k.checkpoint().unwrap();
        let before = snapshot(&k);
        let response = serde_json::to_string(&k.extrude_detailed_impl(f, 0., 0., 1., 2.)).unwrap();
        assert!(response.contains("unsupported"));
        assert_eq!(snapshot(&k), before);
        assert_eq!(k.serialize_solids(&[retained]).unwrap(), document);
    });
}

#[test]
fn profile_nesting_nested_success() {
    for v2 in [false, true] {
        let mut k = RemusKernel::new();
        let f = setup(&mut k, false);
        let request=serde_json::json!([{"op":"extrudeDetailed","args":{"face":f,"dx":0.,"dy":0.,"dz":1.,"distance":2.}}]).to_string();
        let response = if v2 {
            k.execute_batch_v2(&request)
        } else {
            k.execute_batch(&request)
        };
        let rows: serde_json::Value = serde_json::from_str(&response).unwrap();
        let id = rows[0]["ok"]["value"].as_u64().unwrap() as u32;
        assert!((k.volume(id, 1e-5).unwrap() - 3080.).abs() < 1e-6);
    }
}
#[test]
fn profile_nesting_refusal_categories() {
    // This positive-weight spline is supported; force an undecided work
    // budget to retain the category and complete rollback contract.
    remus_check::properties::loop_nesting::with_nesting_work_limit(0, || {
        let mut failures = vec![];
        for v2 in [false, true] {
            for detailed in [false, true] {
                let mut k = RemusKernel::new();
                let f = setup(&mut k, true);
                let name = if detailed {
                    "extrudeDetailed"
                } else {
                    "extrude"
                };
                let input =
                    serde_json::json!([{"op":name,"args":{"face":f,"distance":2.}}]).to_string();
                let output = if v2 {
                    k.execute_batch_v2(&input)
                } else {
                    k.execute_batch(&input)
                };
                let rows: serde_json::Value = serde_json::from_str(&output).unwrap();
                if v2 || detailed {
                    let category = if detailed {
                        &rows[0]["ok"]["category"]
                    } else {
                        &rows[0]["error"]["category"]
                    };
                    if category != "unsupported" {
                        failures.push(format!("v2={v2} detailed={detailed}: {output}"));
                    }
                }
            }
        }
        let mut k = RemusKernel::new();
        let f = setup(&mut k, true);
        let value = serde_json::to_value(k.extrude_detailed_impl(f, 0., 0., 1., 2.)).unwrap();
        if value["category"] != "unsupported" {
            failures.push(format!("direct detailed: {value}"));
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    });
}

#[test]
fn profile_nesting_revolve_refusals_are_unsupported_and_atomic() {
    for v2 in [false, true] {
        for (op, detailed) in [("revolve", false), ("revolveDetailed", true)] {
            let mut k = RemusKernel::new();
            // Known nested lines also refuse on revolve, before allocation.
            let face = setup(&mut k, false);
            k.checkpoint().unwrap();
            let before = snapshot(&k);
            let request=serde_json::json!([{"op":op,"args":{"face":face,"angle":90.,"originX":-30.,"originZ":5.,"axisY":1.,"axisZ":0.}}]).to_string();
            let output = if v2 {
                k.execute_batch_v2(&request)
            } else {
                k.execute_batch(&request)
            };
            let rows: serde_json::Value = serde_json::from_str(&output).unwrap();
            if detailed {
                assert_eq!(rows[0]["ok"]["category"], "unsupported");
            } else if v2 {
                assert_eq!(rows[0]["error"]["category"], "unsupported");
            } else {
                assert!(rows[0]["error"].as_str().unwrap().contains("unsupported"));
            }
            assert_eq!(snapshot(&k), before);
        }
    }
    let mut k = RemusKernel::new();
    let face = setup(&mut k, false);
    let before = snapshot(&k);
    let response =
        serde_json::to_value(k.revolve_detailed_impl(face, -30., 0., 5., 0., 1., 0., 90.)).unwrap();
    assert_eq!(response["category"], "unsupported");
    assert_eq!(snapshot(&k), before);
}

#[test]
fn profile_nesting_certified_spline_success() {
    for v2 in [false, true] {
        let mut k = RemusKernel::new();
        let face = setup(&mut k, true);
        let request =
            serde_json::json!([{"op":"extrudeDetailed","args":{"face":face,"distance":2.}}])
                .to_string();
        let response = if v2 {
            k.execute_batch_v2(&request)
        } else {
            k.execute_batch(&request)
        };
        let rows: serde_json::Value = serde_json::from_str(&response).unwrap();
        let id = rows[0]["ok"]["value"].as_u64().unwrap() as u32;
        // The quadratic rail removes 2 * 0.2 / 3 from the island area.
        let expected = 3080. - 2. * (2. * 0.2 / 3.);
        assert!((k.volume(id, 1e-5).unwrap() - expected).abs() < 1e-6);
    }
}
