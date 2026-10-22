//! Independent planar run-outs must retain line pairing beside a curved chain.
//!
//! The sloped cap z = 4x makes the convex stripe's x contact lie sqrt(17)r
//! from its endpoint. Radius proximity can pair it with the y boundary;
//! edge-line pairing preserves the cap. The remote rounded corner supplies
//! a separate curved G1 chain, whose existing approximation is not promoted.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, deprecated)]

use std::f64::consts::FRAC_PI_4;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::extrude::extrude;
use remus_operations::fillet::fillet_rolling_ball;
use remus_operations::measure::solid_volume;
use remus_operations::tessellate::{tessellate_solid, welded_mesh_quality};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::solid_edges;
use remus_topology::face::{Face, FaceSurface};
use remus_topology::solid::SolidId;

fn edge_where(topo: &Topology, solid: SolidId, pred: impl Fn(Point3, Point3) -> bool) -> EdgeId {
    let edges: Vec<_> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let edge = topo.edge(id).unwrap();
            pred(
                topo.vertex(edge.start()).unwrap().point(),
                topo.vertex(edge.end()).unwrap().point(),
            )
        })
        .collect();
    assert_eq!(edges.len(), 1, "unique spine");
    edges[0]
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-7
}

fn classify(topo: &Topology, solid: SolidId, p: Point3) -> PointClassification {
    classify_point(topo, solid, p, &ClassifyOptions::default()).unwrap()
}

fn assert_sound(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "invalid solid: {:?}", report.issues);
    let quality = welded_mesh_quality(&tessellate_solid(topo, solid, 0.01).unwrap());
    assert_eq!(quality.boundary_edges, 0, "mesh not watertight");
    assert_eq!(quality.non_manifold_edges, 0, "mesh non-manifold");
}

/// 10x10x20 prism, with its bottom z=4x for x<1 and z=4 elsewhere.
/// The far vertical corner (10,10) is already rounded at r=1.
fn rounded_prism(topo: &mut Topology) -> SolidId {
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 20.0),
        Point3::new(10.0, 0.0, 20.0),
        Point3::new(10.0, 0.0, 4.0),
        Point3::new(1.0, 0.0, 4.0),
    ];
    let wire = make_polygon_wire(topo, &points, 1e-7).unwrap();
    let face = topo.add_face(Face::new(
        wire,
        Vec::new(),
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 1.0, 0.0),
            d: 0.0,
        },
    ));
    let blank = extrude(topo, face, Vec3::new(0.0, 1.0, 0.0), 10.0).unwrap();
    let corner = edge_where(topo, blank, |a, b| {
        near(a.x(), 10.0) && near(b.x(), 10.0) && near(a.y(), 10.0) && near(b.y(), 10.0)
    });
    fillet_rolling_ball(topo, blank, &[corner], 1.0).unwrap()
}

#[test]
fn independent_curved_chain_keeps_planar_runout_pairing() {
    let mut topo = Topology::new();
    let input = rounded_prism(&mut topo);
    let planar = edge_where(&topo, input, |a, b| {
        near(a.x(), 0.0) && near(b.x(), 0.0) && near(a.y(), 0.0) && near(b.y(), 0.0)
    });
    let curved = solid_edges(&topo, input)
        .unwrap()
        .into_iter()
        .find(|&id| {
            matches!(topo.edge(id).unwrap().curve(), EdgeCurve::Circle(c) if near(c.center().z(), 20.0))
        })
        .unwrap();
    let chain =
        remus_blend::g1_chain::expand_g1_chain(&topo, input, &[curved], Tolerance::new()).unwrap();
    assert_eq!(chain.len(), 3, "arc and two planar top stripes");
    let spine = topo.edge(planar).unwrap();
    for id in chain {
        let edge = topo.edge(id).unwrap();
        for v in [edge.start(), edge.end()] {
            assert!(
                v != spine.start() && v != spine.end(),
                "independent components"
            );
        }
    }

    let r = 0.5_f64;
    let planar_only = fillet_rolling_ball(&mut topo, input, &[planar], r).unwrap();
    let curved_only = fillet_rolling_ball(&mut topo, input, &[curved], r).unwrap();
    // This sequence already succeeds on the unchanged engine: its second
    // request contains only the planar stripe. Keep it as a stricter oracle
    // for the existing curved approximation and its measurement error.
    let remapped = edge_where(&topo, curved_only, |a, b| {
        near(a.x(), 0.0) && near(b.x(), 0.0) && near(a.y(), 0.0) && near(b.y(), 0.0)
    });
    let sequential = fillet_rolling_ball(&mut topo, curved_only, &[remapped], r).unwrap();
    // Call the shared rebuild directly: the cascade may split independent
    // features into separate calls and conceal the selection-wide defect.
    let combined = fillet_rolling_ball(&mut topo, input, &[planar, curved], r).unwrap();
    let reversed_order = fillet_rolling_ball(&mut topo, input, &[curved, planar], r).unwrap();
    for solid in [
        input,
        planar_only,
        curved_only,
        sequential,
        combined,
        reversed_order,
    ] {
        assert_sound(&topo, solid);
    }

    // A = square minus quarter disc, Mx its first moment about x=0.
    // Integrating the local height 20-4x over that chip gives its volume.
    let area = r * r * (1.0 - FRAC_PI_4);
    let moment = r.powi(3) * (5.0 / 6.0 - FRAC_PI_4);
    let removed = 20.0 * area - 4.0 * moment;
    let original = 1620.0 - 16.0 * (1.0 - FRAC_PI_4);
    assert!((solid_volume(&topo, input, 1e-4).unwrap() - original).abs() < 1e-6);
    for deflection in [1e-4, 5e-5] {
        let planar_volume = solid_volume(&topo, planar_only, deflection).unwrap();
        // The sloped cap samples its elliptic trim; keep that existing
        // chord error below 5e-5 mm3 against the independent closed form.
        assert!((original - planar_volume - removed).abs() < 5e-5);
        let curved_volume = solid_volume(&topo, curved_only, deflection).unwrap();
        let sequential_volume = solid_volume(&topo, sequential, deflection).unwrap();
        assert!((curved_volume - sequential_volume - removed).abs() < 2e-4);
        for solid in [combined, reversed_order] {
            let volume = solid_volume(&topo, solid, deflection).unwrap();
            // The sequential baseline already has a NURBS approximation
            // and whole-solid mesh error (1.24371e-4 / 6.25266e-5 mm3 at
            // these deflections). Retain its closed-form budget and require
            // this repair to match that baseline far more tightly.
            assert!((curved_volume - volume - removed).abs() < 2e-4);
            assert!((volume - sequential_volume).abs() < 1e-8);
        }
    }
    // The remote feature must still remove its own corner, while retaining
    // material below its band; selecting the planar stripe cannot drop it.
    for solid in [curved_only, combined, reversed_order] {
        let removed_point = Point3::new(4.0, 9.95, 19.95);
        assert_eq!(
            classify(&topo, input, removed_point),
            PointClassification::Inside
        );
        assert_eq!(
            classify(&topo, solid, removed_point),
            PointClassification::Outside
        );
        assert_eq!(
            classify(&topo, solid, Point3::new(4.0, 9.6, 19.6)),
            PointClassification::Inside
        );
    }
    for solid in [planar_only, combined, reversed_order] {
        for z in [2.0, 10.0, 19.5] {
            let removed_point = Point3::new(0.05, 0.05, z);
            assert_eq!(
                classify(&topo, input, removed_point),
                PointClassification::Inside
            );
            assert_eq!(
                classify(&topo, solid, removed_point),
                PointClassification::Outside
            );
            assert_eq!(
                classify(&topo, solid, Point3::new(0.4, 0.4, z)),
                PointClassification::Inside
            );
        }
        for z in [1.59, 1.61] {
            let p = Point3::new(0.4, 0.4, z);
            assert_eq!(classify(&topo, solid, p), classify(&topo, input, p));
        }
    }
}

#[test]
fn connected_curved_g1_runout_still_refuses_atomically() {
    let mut topo = Topology::new();
    let input = rounded_prism(&mut topo);
    let bottom_arc = solid_edges(&topo, input)
        .unwrap()
        .into_iter()
        .find(|&id| {
            matches!(topo.edge(id).unwrap().curve(), EdgeCurve::Circle(c) if near(c.center().z(), 4.0))
        })
        .unwrap();
    let chain =
        remus_blend::g1_chain::expand_g1_chain(&topo, input, &[bottom_arc], Tolerance::new())
            .unwrap();
    assert_eq!(chain.len(), 3, "arc and both straight continuations");
    let counts = |t: &Topology| {
        (
            t.num_vertices(),
            t.num_edges(),
            t.num_wires(),
            t.num_faces(),
            t.num_shells(),
            t.num_solids(),
            t.num_loops(),
            t.num_coedges(),
            t.num_pcurves(),
        )
    };
    let before = counts(&topo);
    let volume = solid_volume(&topo, input, 1e-3).unwrap();
    let error = fillet_rolling_ball(&mut topo, input, &[bottom_arc], 0.5).unwrap_err();
    assert!(
        matches!(error, OperationsError::InvalidInput { ref reason }
        if reason.starts_with("fillet postcondition validation failed")),
        "{error}"
    );
    assert_eq!(counts(&topo), before, "refusal must restore every arena");
    assert_eq!(
        solid_volume(&topo, input, 1e-3).unwrap().to_bits(),
        volume.to_bits()
    );
    assert_eq!(
        classify(&topo, input, Point3::new(9.5, 9.5, 5.0)),
        PointClassification::Inside
    );
    assert_sound(&topo, input);
}
