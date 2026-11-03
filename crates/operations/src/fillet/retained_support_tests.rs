//! A remote crown trim must survive a fillet of the cylinder's opposite rim.
#![allow(clippy::unwrap_used, clippy::expect_used, deprecated)]

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, FRAC_PI_4};

use remus_math::vec::{Point3, Vec3};
use remus_math::{curves::Circle3D, nurbs::curve::NurbsCurve, surfaces::CylindricalSurface};
use remus_topology::{
    Topology,
    edge::{Edge, EdgeCurve, EdgeId},
    explorer::solid_edges,
};
use remus_topology::{
    face::{Face, FaceSurface},
    shell::Shell,
    solid::{Solid, SolidId},
};
use remus_topology::{
    vertex::Vertex,
    wire::{OrientedEdge, Wire},
};

/// A rounded box cut by z = height + slope*x. The rounded wall's crown is
/// an exact rational quadratic ellipse on both its cylinder and the cut plane.
/// The non-unit knot domain catches accidental parameter-range replacement.
#[allow(clippy::too_many_lines)]
fn rounded_box_with_crown(topo: &mut Topology, scale: f64) -> (SolidId, EdgeId, NurbsCurve) {
    let (w, d, r, h, slope) = (20.0 * scale, 18.0 * scale, 5.0 * scale, 12.0 * scale, 0.2);
    let xy = [(0.0, 0.0), (w - r, 0.0), (w, r), (w, d), (0.0, d)];
    let bottom = xy.map(|(x, y)| Point3::new(x, y, 0.0));
    let top = xy.map(|(x, y)| Point3::new(x, y, h + slope * x));
    let vb = bottom.map(|p| topo.add_vertex(Vertex::new(p, 1e-7)));
    let vt = top.map(|p| topo.add_vertex(Vertex::new(p, 1e-7)));
    let crown = NurbsCurve::new(
        2,
        vec![2.0, 2.0, 2.0, 5.0, 5.0, 5.0],
        vec![top[1], Point3::new(w, 0.0, h + slope * w), top[2]],
        vec![1.0, FRAC_1_SQRT_2, 1.0],
    )
    .unwrap();
    let center = Point3::new(w - r, r, 0.0);
    let circle = Circle3D::new_with_ref(
        center,
        Vec3::new(0.0, 0.0, 1.0),
        r,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let mut lower = Vec::new();
    let mut upper = Vec::new();
    let mut vertical = Vec::new();
    for i in 0..5 {
        let next = (i + 1) % 5;
        let mut low = Edge::new(
            vb[i],
            vb[next],
            if i == 1 {
                EdgeCurve::Circle(circle.clone())
            } else {
                EdgeCurve::Line
            },
        );
        if i == 1 {
            low.set_trim(Some((-FRAC_PI_2, 0.0)));
        }
        lower.push(topo.add_edge(low));
        let mut up = Edge::new(
            vt[i],
            vt[next],
            if i == 1 {
                EdgeCurve::NurbsCurve(crown.clone())
            } else {
                EdgeCurve::Line
            },
        );
        if i == 1 {
            up.set_trim(Some((2.0, 5.0)));
        }
        upper.push(topo.add_edge(up));
        vertical.push(topo.add_edge(Edge::new(vb[i], vt[i], EdgeCurve::Line)));
    }
    let mut face = |uses: Vec<OrientedEdge>, surface| {
        let wire = topo.add_wire(Wire::new(uses, true).unwrap());
        topo.add_face(Face::new(wire, vec![], surface))
    };
    let top_normal = Vec3::new(-slope, 0.0, 1.0).normalize().unwrap();
    let mut faces = vec![
        face(
            lower
                .iter()
                .rev()
                .map(|&e| OrientedEdge::new(e, false))
                .collect(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, -1.0),
                d: 0.0,
            },
        ),
        face(
            upper.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
            FaceSurface::Plane {
                normal: top_normal,
                d: h * top_normal.z(),
            },
        ),
    ];
    let walls = [
        FaceSurface::Plane {
            normal: Vec3::new(0.0, -1.0, 0.0),
            d: 0.0,
        },
        FaceSurface::Cylinder(
            CylindricalSurface::new(center, Vec3::new(0.0, 0.0, 1.0), r).unwrap(),
        ),
        FaceSurface::Plane {
            normal: Vec3::new(1.0, 0.0, 0.0),
            d: w,
        },
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 1.0, 0.0),
            d,
        },
        FaceSurface::Plane {
            normal: Vec3::new(-1.0, 0.0, 0.0),
            d: 0.0,
        },
    ];
    for (i, surface) in walls.into_iter().enumerate() {
        faces.push(face(
            vec![
                OrientedEdge::new(lower[i], true),
                OrientedEdge::new(vertical[(i + 1) % 5], true),
                OrientedEdge::new(upper[i], false),
                OrientedEdge::new(vertical[i], false),
            ],
            surface,
        ));
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    (topo.add_solid(Solid::new(shell, vec![])), lower[2], crown)
}
#[test]
fn opposite_rim_fillet_retains_the_crown_curve_and_its_domain() {
    for scale in [0.5, 1.0, 2.0] {
        let mut topo = Topology::new();
        let (source, rim, crown) = rounded_box_with_crown(&mut topo, scale);
        let source_bytes = remus_io::arena_io::serialize_solids(&topo, &[source]).unwrap();
        assert!(
            crate::validate::validate_solid(&topo, source)
                .unwrap()
                .is_valid()
        );
        let result = super::fillet_rolling_ball(&mut topo, source, &[rim], scale)
            .expect("the opposite rim must not discard the retained elliptical crown");
        let report = crate::validate::validate_solid(&topo, result).unwrap();
        assert!(report.is_valid(), "{:#?}", report.issues);
        let retained=solid_edges(&topo,result).unwrap().into_iter().filter(|&id| {
            let e=topo.edge(id).unwrap();
            matches!(e.curve(),EdgeCurve::NurbsCurve(c) if c.control_points()==crown.control_points()
                && c.knots()==crown.knots() && c.weights()==crown.weights())
                && e.strict_domain().unwrap()==(2.0,5.0)
        }).count();
        assert_eq!(
            retained, 1,
            "the exact authored crown and its parameter range must survive"
        );
        assert_eq!(
            remus_io::arena_io::serialize_solids(&topo, &[source]).unwrap(),
            source_bytes
        );
        let input_volume = crate::measure::solid_volume(&topo, source, 0.001 * scale).unwrap();
        let output_volume = crate::measure::solid_volume(&topo, result, 0.001 * scale).unwrap();
        // Pappus: sweep the removed unit-round section along a quarter circle.
        let removed = ((1.0 - FRAC_PI_4) * (15.0 + 13.0 + FRAC_PI_2 * 5.0)
            - FRAC_PI_2 * (5.0 / 6.0 - FRAC_PI_4))
            * scale.powi(3);
        assert!(
            (input_volume - output_volume - removed).abs() < 1e-3 * scale.powi(3),
            "removed {}, expected {removed}",
            input_volume - output_volume
        );
    }
}
