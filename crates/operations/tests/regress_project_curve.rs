#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
//! Independent review regressions for the P74 projection public contract.
use remus_math::{
    curves::{Circle3D, Ellipse3D, Parabola3D},
    frame::Frame3,
    nurbs::curve::NurbsCurve,
    vec::{Point3, Vec3},
};
use remus_operations::{
    primitives::{make_box, make_cylinder},
    project_curve::{
        ProjectCurveError, ProjectCurveOptions, project_curve_onto_face, project_curves_onto_plane,
        project_curves_onto_solid,
    },
};
use remus_topology::{
    Topology,
    edge::{Edge, EdgeCurve, EdgeId},
    explorer::solid_faces,
    face::{FaceId, FaceSurface},
    vertex::Vertex,
};
use std::f64::consts::{PI, TAU};
fn edge(t: &mut Topology, c: EdgeCurve, a: Point3, b: Point3, trim: Option<(f64, f64)>) -> EdgeId {
    let va = t.add_vertex(Vertex::new(a, 1e-7));
    let vb = t.add_vertex(Vertex::new(b, 1e-7));
    let mut e = Edge::new(va, vb, c);
    e.set_trim(trim);
    t.add_edge(e)
}
fn top(t: &mut Topology) -> FaceId {
    let s = make_box(t, 10., 8., 4.).unwrap();
    solid_faces(t,s).unwrap().into_iter().find(|f|matches!(t.face(*f).unwrap().surface(),FaceSurface::Plane{normal,d} if normal.z()>0.9 && *d>3.9)).unwrap()
}
fn xy() -> Frame3 {
    Frame3::from_normal(Point3::new(0., 0., 0.), Vec3::new(0., 0., 1.)).unwrap()
}
#[test]
fn plane_segment_crossing_target_clips_at_zero_distance() {
    let mut t = Topology::new();
    let f = top(&mut t);
    let s = edge(
        &mut t,
        EdgeCurve::Line,
        Point3::new(2., 3., 5.),
        Point3::new(8., 3., 3.),
        None,
    );
    let r = project_curve_onto_face(
        &mut t,
        s,
        Vec3::new(0., 0., -1.),
        f,
        &ProjectCurveOptions::default(),
    );
    let r = r.unwrap();
    assert!(r.clipped);
    assert!((r.edges[0].source_range.1 - 0.5).abs() < 1e-9);
}
#[test]
fn sketch_rotated_ellipse_endpoints_match() {
    let mut t = Topology::new();
    let c = Ellipse3D::new_with_ref(
        Point3::new(3., 3., 5.),
        Vec3::new(0., 0., 1.),
        2.,
        1.,
        Vec3::new(1., 1., 0.),
    )
    .unwrap();
    let range = (0.4, 1.2);
    let a = c.evaluate(range.0);
    let b = c.evaluate(range.1);
    let s = edge(&mut t, EdgeCurve::Ellipse(c), a, b, Some(range));
    let frame = xy();
    let r = project_curves_onto_plane(&t, &[s], Vec3::new(0., 0., -1.), &frame).unwrap();
    let image = r[0].evaluate(r[0].t_start());
    let p = a - frame.origin;
    let error = (image - remus_math::vec::Point2::new(p.dot(frame.x), p.dot(frame.y))).length();
    assert!(error < 1e-9);
}
#[test]
fn open_approx_arc_has_distinct_endpoint_vertices_and_full_range() {
    let mut t = Topology::new();
    let solid = make_cylinder(&mut t, 2., 10.).unwrap();
    let f = solid_faces(&t, solid)
        .unwrap()
        .into_iter()
        .find(|f| matches!(t.face(*f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let c = Circle3D::new_with_ref(
        Point3::new(0., 5., 5.),
        Vec3::new(0., 1., 0.),
        1.,
        Vec3::new(1., 0., 0.),
    )
    .unwrap();
    let s = edge(
        &mut t,
        EdgeCurve::Circle(c.clone()),
        c.evaluate(0.),
        c.evaluate(PI / 2.),
        Some((0., PI / 2.)),
    );
    let opts = ProjectCurveOptions {
        allow_approximate: true,
        ..ProjectCurveOptions::default()
    };
    let r = project_curve_onto_face(&mut t, s, Vec3::new(0., -1., 0.), f, &opts).unwrap();
    let e = t.edge(r.edges[0].edge).unwrap();
    assert_ne!(e.start(), e.end());
    let end = t.vertex(e.end()).unwrap().point();
    assert!((end - Point3::new(0., 2., 4.)).length() < 1e-5);
}
#[test]
fn solid_never_projects_through_skipped_curved_first_hit() {
    let mut t = Topology::new();
    let solid = make_cylinder(&mut t, 2., 10.).unwrap();
    let c = Circle3D::new_with_ref(
        Point3::new(0., 5., 6.),
        Vec3::new(0., 1., 0.),
        0.5,
        Vec3::new(1., 0., 0.),
    )
    .unwrap();
    let s = edge(
        &mut t,
        EdgeCurve::Circle(c.clone()),
        c.evaluate(0.),
        c.evaluate(TAU),
        Some((0., TAU)),
    );
    let r = project_curves_onto_solid(
        &mut t,
        &[s],
        Vec3::new(0., -1., -1.),
        solid,
        &ProjectCurveOptions::default(),
    );
    if let Ok(r) = r {
        for e in &r.sources[0].edges {
            assert!(
                matches!(t.face(e.face).unwrap().surface(), FaceSurface::Cylinder(_)),
                "returned cap behind a curved first hit"
            );
        }
    }
}
#[test]
fn sketch_edge_on_parabola_is_refused() {
    let mut t = Topology::new();
    let c = Parabola3D::with_axes(
        Point3::new(0., 0., 4.),
        Vec3::new(1., 0., 0.),
        Vec3::new(0., 0., 1.),
        1.,
    )
    .unwrap();
    let s = edge(
        &mut t,
        EdgeCurve::Parabola(c.clone()),
        c.evaluate(-1.),
        c.evaluate(1.),
        Some((-1., 1.)),
    );
    let r = project_curves_onto_plane(&t, &[s], Vec3::new(0., 0., -1.), &xy());
    assert!(
        matches!(r,Err(ProjectCurveError::SourceRefused{error,..}) if matches!(*error,ProjectCurveError::DegenerateImage))
    );
}
#[test]
fn sketch_off_grid_stationary_nurbs_is_refused() {
    let mut t = Topology::new();
    let a = 0.314_159_265_358_979_3;
    let c = NurbsCurve::new(
        2,
        vec![0., 0., 0., 1., 1., 1.],
        vec![
            Point3::new(a * a, 0., 4.),
            Point3::new(a * a - a, 0., 4.5),
            Point3::new((1. - a) * (1. - a), 0., 5.),
        ],
        vec![1., 1., 1.],
    )
    .unwrap();
    let s = edge(
        &mut t,
        EdgeCurve::NurbsCurve(c.clone()),
        c.evaluate(0.),
        c.evaluate(1.),
        Some((0., 1.)),
    );
    let r = project_curves_onto_plane(&t, &[s], Vec3::new(0., 0., -1.), &xy());
    assert!(
        matches!(r,Err(ProjectCurveError::SourceRefused{error,..}) if matches!(*error,ProjectCurveError::DegenerateImage))
    );
}
#[test]
fn plane_tilted_circle_crossing_target_does_not_backproject() {
    let mut t = Topology::new();
    let f = top(&mut t);
    let c = Circle3D::new_with_ref(
        Point3::new(5., 4., 4.5),
        Vec3::new(0., (3f64).sqrt() / 2., 0.5),
        1.,
        Vec3::new(1., 0., 0.),
    )
    .unwrap();
    let s = edge(
        &mut t,
        EdgeCurve::Circle(c.clone()),
        c.evaluate(0.),
        c.evaluate(TAU),
        Some((0., TAU)),
    );
    let r = project_curve_onto_face(
        &mut t,
        s,
        Vec3::new(0., 0., -1.),
        f,
        &ProjectCurveOptions::default(),
    )
    .unwrap();
    assert!(r.clipped);
    for e in r.edges {
        let (a, b) = e.source_range;
        for k in 0..=32 {
            assert!(c.evaluate(a + (b - a) * (k as f64 / 32.)).z() >= 4. - 1e-9);
        }
    }
}
#[test]
fn narrow_circle_cap_overlap_keeps_both_boundary_roots() {
    let mut t = Topology::new();
    let solid = make_cylinder(&mut t, 2., 10.).unwrap();
    let f=solid_faces(&t,solid).unwrap().into_iter().find(|f|matches!(t.face(*f).unwrap().surface(),FaceSurface::Plane{normal,d} if normal.z()>0.9 && *d>9.9)).unwrap();
    let rim = t
        .face_oriented_edges(f)
        .unwrap()
        .into_iter()
        .find_map(|e| match t.edge(e.edge()).unwrap().curve() {
            EdgeCurve::Circle(c) => Some(c.clone()),
            _ => None,
        })
        .unwrap();
    let angle = TAU / 8192.;
    let radial = rim.u_axis() * angle.cos() + rim.v_axis() * angle.sin();
    let center = Point3::new(0., 0., 12.) + radial * (3. - 1e-7);
    let c = Circle3D::new_with_ref(center, Vec3::new(0., 0., 1.), 1., radial).unwrap();
    let s = edge(
        &mut t,
        EdgeCurve::Circle(c.clone()),
        c.evaluate(0.),
        c.evaluate(TAU),
        Some((0., TAU)),
    );
    let r = project_curve_onto_face(
        &mut t,
        s,
        Vec3::new(0., 0., -1.),
        f,
        &ProjectCurveOptions::default(),
    );
    let r = r.unwrap();
    assert!(r.clipped);
    assert_eq!(r.edges.len(), 1);
}

#[test]
fn descending_stationary_nurbs_is_refused() {
    let mut topo = Topology::new();
    let a = 0.314_159_265_358_979_3;
    let curve = NurbsCurve::new(
        2,
        vec![0., 0., 0., 1., 1., 1.],
        vec![
            Point3::new(a * a, 0., 4.),
            Point3::new(a * a - a, 0., 4.5),
            Point3::new((1. - a) * (1. - a), 0., 5.),
        ],
        vec![1., 1., 1.],
    )
    .unwrap();
    let source = edge(
        &mut topo,
        EdgeCurve::NurbsCurve(curve.clone()),
        curve.evaluate(1.),
        curve.evaluate(0.),
        Some((1., 0.)),
    );
    assert!(
        matches!(project_curves_onto_plane(&topo, &[source], Vec3::new(0., 0., -1.), &xy()),
        Err(ProjectCurveError::SourceRefused { error, .. }) if matches!(*error, ProjectCurveError::DegenerateImage))
    );
}

#[test]
fn rounded_constant_nurbs_output_is_refused() {
    let mut topo = Topology::new();
    let curve = NurbsCurve::new(
        1,
        vec![0., 0., 1., 1.],
        vec![
            Point3::new(1e16, 0., 1e16),
            Point3::new(1e16, 0., 1e16 + 2.),
        ],
        vec![1., 1.],
    )
    .unwrap();
    let source = edge(
        &mut topo,
        EdgeCurve::NurbsCurve(curve.clone()),
        curve.evaluate(0.),
        curve.evaluate(1.),
        Some((0., 1.)),
    );
    assert!(
        matches!(project_curves_onto_plane(&topo, &[source], Vec3::new(7.5e-17, 0., 1.), &xy()),
        Err(ProjectCurveError::SourceRefused { error, .. }) if matches!(*error, ProjectCurveError::DegenerateImage))
    );
}

#[test]
fn near_parallel_occluding_face_is_refused_atomically() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10., 8., 4.).unwrap();
    let source = edge(
        &mut topo,
        EdgeCurve::Line,
        Point3::new(1., 2., 4. + 5e-10),
        Point3::new(1., 3., 4. + 5e-10),
        None,
    );
    let before = (topo.num_vertices(), topo.num_edges(), topo.journal().len());
    assert!(
        matches!(project_curves_onto_solid(&mut topo, &[source], Vec3::new(1., 0., -5e-10), solid, &ProjectCurveOptions::default()),
        Err(ProjectCurveError::SourceRefused { error, .. }) if matches!(*error, ProjectCurveError::GrazingDirection))
    );
    assert_eq!(
        before,
        (topo.num_vertices(), topo.num_edges(), topo.journal().len())
    );
}

#[test]
fn open_arc_crossing_period_lifts_boundary_event() {
    let mut topo = Topology::new();
    let face = top(&mut topo);
    let curve = Circle3D::new_with_ref(
        Point3::new(5., 0., 6.),
        Vec3::new(0., 0., 1.),
        1.,
        Vec3::new(1., 0., 0.),
    )
    .unwrap();
    let source = edge(
        &mut topo,
        EdgeCurve::Circle(curve.clone()),
        curve.evaluate(5.5),
        curve.evaluate(6.5),
        Some((5.5, 6.5)),
    );
    let result = project_curve_onto_face(
        &mut topo,
        source,
        Vec3::new(0., 0., -1.),
        face,
        &ProjectCurveOptions::default(),
    )
    .unwrap();
    assert!(result.clipped);
    assert_eq!(result.edges.len(), 1);
    assert!((result.edges[0].source_range.0 - TAU).abs() < 1e-9);
    assert!((result.edges[0].source_range.1 - 6.5).abs() < 1e-9);
}

#[test]
fn descending_face_source_is_refused_without_panic() {
    let mut topo = Topology::new();
    let face = top(&mut topo);
    let curve = Circle3D::new_with_ref(
        Point3::new(5., 4., 6.),
        Vec3::new(0., 0., 1.),
        1.,
        Vec3::new(1., 0., 0.),
    )
    .unwrap();
    let source = edge(
        &mut topo,
        EdgeCurve::Circle(curve.clone()),
        curve.evaluate(TAU),
        curve.evaluate(0.),
        Some((TAU, 0.)),
    );
    assert!(matches!(
        project_curve_onto_face(
            &mut topo,
            source,
            Vec3::new(0., 0., -1.),
            face,
            &ProjectCurveOptions::default()
        ),
        Err(ProjectCurveError::DegenerateSource { .. })
    ));
}

#[test]
fn sketch_descending_circle_keeps_clockwise_traversal() {
    let mut topo = Topology::new();
    let curve = Circle3D::new_with_ref(
        Point3::new(0., 0., 6.),
        Vec3::new(0., 0., 1.),
        1.,
        Vec3::new(1., 0., 0.),
    )
    .unwrap();
    for range in [(1., 0.), (TAU, 0.)] {
        let source = edge(
            &mut topo,
            EdgeCurve::Circle(curve.clone()),
            curve.evaluate(range.0),
            curve.evaluate(range.1),
            Some(range),
        );
        let result =
            project_curves_onto_plane(&topo, &[source], Vec3::new(0., 0., -1.), &xy()).unwrap();
        assert!((result[0].t_end() - result[0].t_start() - (range.1 - range.0)).abs() < 1e-9);
    }
}

#[test]
fn narrow_true_circular_hole_is_not_replaced_by_its_chords() {
    use remus_topology::{
        face::Face,
        wire::{OrientedEdge, Wire},
    };
    let mut topo = Topology::new();
    let old_face = top(&mut topo);
    let outer = topo.face(old_face).unwrap().outer_wire();
    let surface = topo.face(old_face).unwrap().surface().clone();
    let circle = Circle3D::new_with_ref(
        Point3::new(5., 4., 4.),
        Vec3::new(0., 0., 1.),
        1.,
        Vec3::new(1., 0., 0.),
    )
    .unwrap();
    let vertex = topo.add_vertex(Vertex::new(circle.evaluate(0.), 1e-7));
    let mut rim = Edge::new(vertex, vertex, EdgeCurve::Circle(circle));
    rim.set_trim(Some((0., TAU)));
    let rim = topo.add_edge(rim);
    let hole = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true).unwrap());
    let face = topo.add_face(Face::new(outer, vec![hole], surface));
    let angle = PI / 32.;
    let radial = Vec3::new(angle.cos(), angle.sin(), 0.);
    let tangent = Vec3::new(-angle.sin(), angle.cos(), 0.);
    let center = Point3::new(5., 4., 6.) + radial * (1. - 1e-6);
    let source = edge(
        &mut topo,
        EdgeCurve::Line,
        center - tangent * 0.01,
        center + tangent * 0.01,
        None,
    );
    let result = project_curve_onto_face(
        &mut topo,
        source,
        Vec3::new(0., 0., -1.),
        face,
        &ProjectCurveOptions::default(),
    )
    .unwrap();
    assert!(result.clipped);
    assert_eq!(result.edges.len(), 2);
    assert!(result.edges[0].source_range.1 < 0.5);
    assert!(result.edges[1].source_range.0 > 0.5);
}
