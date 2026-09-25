//! Adaptive controls on trimmed and freeform domains (B20 follow-up).
//!
//! Default controls keep the historical fixed rule bit for bit. Any other
//! control pair refines the quadrature over the face's resolved domain and is
//! checked here against closed forms computed independently of the
//! integrator. The sampled trim outline stays fixed input: refinement converges
//! quadrature, it does not move the outline.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::CheckError;
use remus_check::properties::{
    PropertiesOptions,
    face_integrator::{FaceContribution, integrate_face, integrate_face_with_options},
};
use remus_math::{
    curves::Circle3D,
    nurbs::surface::NurbsSurface,
    surfaces::CylindricalSurface,
    vec::{Point3, Vec3},
};
use remus_topology::{
    Topology,
    edge::{Edge, EdgeCurve},
    face::{Face, FaceId, FaceSurface},
    vertex::Vertex,
    wire::{OrientedEdge, Wire},
};

/// Defaults keep the fixed rule; invalid controls still refuse.
fn check_default_contract(topo: &Topology, face: FaceId) {
    let options = PropertiesOptions::default();
    let fixed = integrate_face(topo, face, options.gauss_order).unwrap();
    let compatible = integrate_face_with_options(topo, face, &options).unwrap();
    assert_eq!(format!("{fixed:?}"), format!("{compatible:?}"));
    let invalid = PropertiesOptions {
        adaptive_eps: f64::NAN,
        ..Default::default()
    };
    assert!(matches!(
        integrate_face_with_options(topo, face, &invalid),
        Err(CheckError::IntegrationFailed(_))
    ));
}

fn tight(order: usize) -> PropertiesOptions {
    PropertiesOptions {
        gauss_order: order,
        adaptive_eps: 1e-12,
        max_depth: 24,
    }
}

fn rel(actual: f64, expected: f64) -> f64 {
    (actual - expected).abs() / expected.abs()
}

#[test]
fn nurbs_face_keeps_default_result_and_refines_on_request() {
    let s = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        ],
        vec![vec![1.0; 2]; 2],
    )
    .unwrap();
    let mut topo = Topology::new();
    let corners = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
        .map(|(u, v)| topo.add_vertex(Vertex::new(s.evaluate(u, v), 1e-7)));
    let edges = (0..4)
        .map(|i| {
            OrientedEdge::new(
                topo.add_edge(Edge::new(corners[i], corners[(i + 1) % 4], EdgeCurve::Line)),
                true,
            )
        })
        .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(s)));
    check_default_contract(&topo, face);
    // A unit square in z = 0: area 1, and every volume term vanishes.
    for options in [
        tight(1),
        tight(4),
        PropertiesOptions {
            max_depth: 0,
            ..Default::default()
        },
    ] {
        let c = integrate_face_with_options(&topo, face, &options).unwrap();
        assert!((c.area - 1.0).abs() <= 1e-14, "{options:?}: {c:?}");
        assert!((c.centroid_x - 0.5).abs() <= 1e-14, "{options:?}: {c:?}");
        assert!(c.volume.abs() <= 1e-15, "{options:?}: {c:?}");
    }
}

#[test]
fn polygon_trimmed_cylinder_refines_toward_closed_form_moments() {
    let mut topo = Topology::new();
    let s =
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 5.0).unwrap();
    let points = [(0.0, 0.0), (2.0, 0.0), (2.0, 3.0), (0.0, 3.0)].map(|(u, v)| s.evaluate(u, v));
    let vertices = points.map(|p| topo.add_vertex(Vertex::new(p, 1e-7)));
    let mut edges = Vec::new();
    for i in 0..4 {
        let mut edge = if i == 0 || i == 2 {
            let circle = Circle3D::new(
                Point3::new(0.0, 0.0, points[i].z()),
                Vec3::new(0.0, 0.0, if i == 0 { 1.0 } else { -1.0 }),
                5.0,
            )
            .unwrap();
            let start = circle.project(points[i]);
            let mut edge = Edge::new(
                vertices[i],
                vertices[(i + 1) % 4],
                EdgeCurve::Circle(circle),
            );
            edge.set_trim(Some((start, start + 2.0)));
            edge
        } else {
            Edge::new(vertices[i], vertices[(i + 1) % 4], EdgeCurve::Line)
        };
        // Exercise an explicit stored line trim as well as circular ones.
        if i == 1 || i == 3 {
            edge.set_trim(Some((0.0, 1.0)));
        }
        edges.push(OrientedEdge::new(topo.add_edge(edge), true));
    }
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(s.clone())));
    check_default_contract(&topo, face);
    assert!((integrate_face(&topo, face, 5).unwrap().area - 30.0).abs() < 1e-8);

    // r = 5, u in [0, 2], z in [0, 3], and x = r·cos(u + φ) where φ is the
    // surface frame's angle of u = 0. The outline is exact (lines in z and
    // arcs in u sample onto straight UV segments), so the closed forms are the
    // whole truth: A = r·Δu·h, ∫x dA = r²·h·[sin t], and the divergence moment
    // ½∫x²·n_x dA = ½·r³·h·[sin t − sin³t / 3], t from φ to φ + 2.
    let (r, h) = (5.0_f64, 3.0_f64);
    let origin = points[0];
    let phi = origin.y().atan2(origin.x());
    let (t0, t1) = (phi, phi + 2.0);
    let sin_cubed = |t: f64| t.sin() - t.sin().powi(3) / 3.0;
    let area = r * 2.0 * h;
    let centroid_x = r * r * h * (t1.sin() - t0.sin());
    let moment_x = 0.5 * r.powi(3) * h * (sin_cubed(t1) - sin_cubed(t0));
    assert!((s.evaluate(1.0, 0.0).x() - r * (phi + 1.0).cos()).abs() < 1e-12);
    let errors = |c: &FaceContribution| {
        [
            rel(c.area, area),
            rel(c.centroid_x, centroid_x),
            rel(c.volume_moment_x, moment_x),
        ]
    };

    // A coarse request (midpoint rule, loose tolerance, no extra depth) is
    // accepted and visibly less accurate on the trigonometric moments.
    let coarse = PropertiesOptions {
        gauss_order: 1,
        adaptive_eps: 0.5,
        max_depth: 0,
    };
    let coarse = errors(&integrate_face_with_options(&topo, face, &coarse).unwrap());
    // Tightening the tolerance at a fixed low order converges monotonically.
    let mut previous = coarse;
    for eps in [1e-3, 1e-5, 1e-7, 1e-9, 1e-11] {
        let options = PropertiesOptions {
            gauss_order: 2,
            adaptive_eps: eps,
            max_depth: 24,
        };
        let now = errors(&integrate_face_with_options(&topo, face, &options).unwrap());
        for (k, (&n, &p)) in now.iter().zip(&previous).enumerate() {
            assert!(
                n <= p.max(1e-14),
                "component {k} regressed at eps {eps:e}: {n:e} > {p:e}"
            );
        }
        previous = now;
    }
    // Measured: coarse ≈ 2.5e-6 on both moments, tight ≤ 3e-13.
    for (k, (&c, &t)) in coarse.iter().zip(&previous).enumerate().skip(1) {
        assert!(c > 1e-6, "coarse component {k} error {c:e} must be visible");
        assert!(t <= 1e-12, "tight component {k} error {t:e}");
        assert!(c > 1e4 * t, "component {k}: coarse {c:e} vs tight {t:e}");
    }
    assert!(previous[0] <= 1e-13, "area error {:e}", previous[0]);

    // A tolerance the depth cannot reach refuses instead of returning an
    // unconverged value.
    let unreachable = PropertiesOptions {
        gauss_order: 1,
        adaptive_eps: 1e-14,
        max_depth: 1,
    };
    let error = integrate_face_with_options(&topo, face, &unreachable).unwrap_err();
    assert!(error.to_string().contains("max_depth"), "{error}");
}
