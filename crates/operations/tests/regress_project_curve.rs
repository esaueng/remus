#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
//! Independent review regressions for the P74 projection public contract.
use remus_math::{
    curves::{Circle3D, Ellipse3D, Parabola3D},
    frame::Frame3,
    nurbs::curve::NurbsCurve,
    vec::{Point3, Vec3},
};
use remus_operations::{
    primitives::{make_box, make_cone, make_cylinder, make_sphere},
    project_curve::{
        ProjectCurveError, ProjectCurveOptions, ProjectionQuality, project_curve_onto_face,
        project_curves_onto_plane, project_curves_onto_solid,
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

#[test]
fn vertical_segment_hits_cylinder_cap_when_lateral_carrier_is_missed() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 2.0, 10.0).unwrap();
    let source = edge(
        &mut topo,
        EdgeCurve::Line,
        Point3::new(-0.5, 0.0, 12.0),
        Point3::new(0.5, 0.0, 12.0),
        None,
    );
    let result = project_curves_onto_solid(
        &mut topo,
        &[source],
        Vec3::new(0.0, 0.0, -1.0),
        solid,
        &ProjectCurveOptions::default(),
    )
    .unwrap();
    assert_eq!(result.sources[0].edges.len(), 1);
    let image = &result.sources[0].edges[0];
    assert!(
        matches!(topo.face(image.face).unwrap().surface(), FaceSurface::Plane { normal, d } if normal.z() > 0.9 && (*d - 10.0).abs() < 1e-9)
    );
    assert!(!result.sources[0].clipped);
}

#[test]
fn vertical_segment_cylinder_miss_and_unresolved_graze_remain_distinct() {
    for (a, b, is_miss) in [(3.0, 4.0, true), (1.5, 2.5, false)] {
        let mut topo = Topology::new();
        let solid = make_cylinder(&mut topo, 2.0, 10.0).unwrap();
        let source = edge(
            &mut topo,
            EdgeCurve::Line,
            Point3::new(a, 0.0, 12.0),
            Point3::new(b, 0.0, 12.0),
            None,
        );
        let result = project_curves_onto_solid(
            &mut topo,
            &[source],
            Vec3::new(0.0, 0.0, -1.0),
            solid,
            &ProjectCurveOptions::default(),
        );
        if is_miss {
            assert!(matches!(result, Err(ProjectCurveError::EmptyProjection)));
        } else {
            assert!(
                matches!(result, Err(ProjectCurveError::SourceRefused { error, .. }) if matches!(*error, ProjectCurveError::GrazingDirection))
            );
        }
    }
}

#[test]
fn vertical_segment_hits_cone_cap_when_lateral_carrier_is_missed() {
    let mut topo = Topology::new();
    let solid = make_cone(&mut topo, 2.0, 1.0, 10.0).unwrap();
    let source = edge(
        &mut topo,
        EdgeCurve::Line,
        Point3::new(-0.5, 0.0, 12.0),
        Point3::new(0.5, 0.0, 12.0),
        None,
    );
    let result = project_curves_onto_solid(
        &mut topo,
        &[source],
        Vec3::new(0.0, 0.0, -1.0),
        solid,
        &ProjectCurveOptions::default(),
    )
    .unwrap();
    assert_eq!(result.sources[0].edges.len(), 1);
    let image = &result.sources[0].edges[0];
    assert!(
        matches!(topo.face(image.face).unwrap().surface(), FaceSurface::Plane { normal, d } if normal.z() > 0.9 && (*d - 10.0).abs() < 1e-9)
    );
}

#[test]
fn vertical_segment_keeps_first_sphere_hit() {
    let mut topo = Topology::new();
    let solid = make_sphere(&mut topo, 2.0, 16).unwrap();
    let source = edge(
        &mut topo,
        EdgeCurve::Line,
        Point3::new(-0.5, 0.0, 12.0),
        Point3::new(0.5, 0.0, 12.0),
        None,
    );
    let result = project_curves_onto_solid(
        &mut topo,
        &[source],
        Vec3::new(0.0, 0.0, -1.0),
        solid,
        &ProjectCurveOptions::default(),
    )
    .unwrap();
    for image in &result.sources[0].edges {
        let e = topo.edge(image.edge).unwrap();
        assert!(topo.vertex(e.start()).unwrap().point().z() > 1.9);
        assert!(topo.vertex(e.end()).unwrap().point().z() > 1.9);
    }
}

fn solid_approx_fixture() -> (Topology, remus_topology::solid::SolidId, EdgeId) {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 2.0, 10.0).unwrap();
    let circle = Circle3D::new_with_ref(
        Point3::new(0.0, 5.0, 5.0),
        Vec3::new(0.0, 1.0, 0.0),
        1.0,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let source = edge(
        &mut topo,
        EdgeCurve::Circle(circle.clone()),
        circle.evaluate(0.0),
        circle.evaluate(TAU),
        Some((0.0, TAU)),
    );
    (topo, solid, source)
}

#[test]
fn approximate_exterior_sliver_between_clip_probes_is_refused_atomically() {
    for allow_approximate in [false, true] {
        let mut topo = Topology::new();
        let solid = make_cylinder(&mut topo, 2.0, 10.0).unwrap();
        let face = solid_faces(&topo, solid)
            .unwrap()
            .into_iter()
            .find(|id| matches!(topo.face(*id).unwrap().surface(), FaceSurface::Cylinder(_)))
            .unwrap();
        let phase = PI / 2048.0;
        let circle = Circle3D::new_with_ref(
            Point3::new(0.0, 5.0, 9.0 + 5e-7),
            Vec3::new(0.0, 1.0, 0.0),
            1.0,
            Vec3::new(phase.cos(), 0.0, phase.sin()),
        )
        .unwrap();
        // Every legacy clipping probe stays below the cap, while the true
        // maximum halfway between two probes lies outside by 5e-7.
        for k in 0..=2048 {
            assert!(circle.evaluate(TAU * f64::from(k) / 2048.0).z() < 10.0);
        }
        assert!(circle.evaluate(1.5 * PI + phase).z() > 10.0 + 4e-7);
        let source = edge(
            &mut topo,
            EdgeCurve::Circle(circle.clone()),
            circle.evaluate(0.0),
            circle.evaluate(TAU),
            Some((0.0, TAU)),
        );
        let before = topo.clone();
        let result = project_curve_onto_face(
            &mut topo,
            source,
            Vec3::new(0.0, -1.0, 0.0),
            face,
            &ProjectCurveOptions {
                allow_approximate,
                ..ProjectCurveOptions::default()
            },
        );
        assert!(
            matches!(
                result,
                Err(ProjectCurveError::ApproximateClipUnsupported) if allow_approximate
            ) || matches!(result, Err(ProjectCurveError::ApproximationRequired) if !allow_approximate),
            "{result:?}"
        );
        assert_eq!(topo.num_vertices(), before.num_vertices());
        assert_eq!(topo.num_edges(), before.num_edges());
        assert_eq!(topo.allocated_slot_count(), before.allocated_slot_count());
        assert_eq!(topo.mutation_ticks(), before.mutation_ticks());
        assert_eq!(topo.journal().snapshot(), before.journal().snapshot());
    }
}

#[test]
fn approximate_sub_probe_curved_hole_is_refused_atomically() {
    use remus_topology::{
        face::Face,
        wire::{OrientedEdge, Wire},
    };
    for allow_approximate in [false, true] {
        let (mut topo, solid, source) = solid_approx_fixture();
        let face = solid_faces(&topo, solid)
            .unwrap()
            .into_iter()
            .find(|id| matches!(topo.face(*id).unwrap().surface(), FaceSurface::Cylinder(_)))
            .unwrap();
        let source_parameter = TAU * 40.5 / 2048.0;
        let (x, z) = (source_parameter.cos(), 5.0 - source_parameter.sin());
        let angle = (4.0 - x * x).sqrt().atan2(x);
        let (lo, hi) = (angle - 1e-5, angle + 1e-5);
        let rim = |height| {
            Circle3D::new_with_ref(
                Point3::new(0.0, 0.0, height),
                Vec3::new(0.0, 0.0, 1.0),
                2.0,
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap()
        };
        let bottom = rim(z - 1e-5);
        let top = rim(z + 1e-5);
        let a = topo.add_vertex(Vertex::new(bottom.evaluate(lo), 1e-12));
        let b = topo.add_vertex(Vertex::new(bottom.evaluate(hi), 1e-12));
        let c = topo.add_vertex(Vertex::new(top.evaluate(hi), 1e-12));
        let d = topo.add_vertex(Vertex::new(top.evaluate(lo), 1e-12));
        let mut lower = Edge::new(a, b, EdgeCurve::Circle(bottom));
        lower.set_trim(Some((lo, hi)));
        let mut upper = Edge::new(d, c, EdgeCurve::Circle(top));
        upper.set_trim(Some((lo, hi)));
        let edges = [
            topo.add_edge(lower),
            topo.add_edge(Edge::new(b, c, EdgeCurve::Line)),
            topo.add_edge(upper),
            topo.add_edge(Edge::new(d, a, EdgeCurve::Line)),
        ];
        let hole = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(edges[0], true),
                    OrientedEdge::new(edges[1], true),
                    OrientedEdge::new(edges[2], false),
                    OrientedEdge::new(edges[3], true),
                ],
                true,
            )
            .unwrap(),
        );
        let data = topo.face(face).unwrap();
        let modified = topo.add_face(Face::new(
            data.outer_wire(),
            vec![hole],
            data.surface().clone(),
        ));
        let before = topo.clone();
        let result = project_curve_onto_face(
            &mut topo,
            source,
            Vec3::new(0.0, -1.0, 0.0),
            modified,
            &ProjectCurveOptions {
                allow_approximate,
                ..ProjectCurveOptions::default()
            },
        );
        assert!(
            matches!(result, Err(ProjectCurveError::ApproximateClipUnsupported) if allow_approximate)
                || matches!(result, Err(ProjectCurveError::ApproximationRequired) if !allow_approximate)
        );
        assert_eq!(topo.num_vertices(), before.num_vertices());
        assert_eq!(topo.num_edges(), before.num_edges());
        assert_eq!(topo.allocated_slot_count(), before.allocated_slot_count());
        assert_eq!(topo.mutation_ticks(), before.mutation_ticks());
        assert_eq!(topo.journal().snapshot(), before.journal().snapshot());
    }
}

#[test]
fn solid_approximation_is_opt_in_and_discloses_global_quality() {
    let (mut topo, solid, source) = solid_approx_fixture();
    let direction = Vec3::new(0.0, -1.0, 0.0);
    let baseline = (topo.num_vertices(), topo.num_edges(), topo.journal().len());
    assert!(
        matches!(project_curves_onto_solid(&mut topo, &[source], direction, solid, &ProjectCurveOptions::default()), Err(ProjectCurveError::SourceRefused { error, .. }) if matches!(*error, ProjectCurveError::ApproximationRequired))
    );
    assert_eq!(
        baseline,
        (topo.num_vertices(), topo.num_edges(), topo.journal().len())
    );
    let exact = edge(
        &mut topo,
        EdgeCurve::Line,
        Point3::new(0.5, 5.0, 6.0),
        Point3::new(0.5, 5.0, 7.0),
        None,
    );
    let other_circle = Circle3D::new_with_ref(
        Point3::new(0.0, 5.0, 6.0),
        Vec3::new(0.0, 1.0, 0.0),
        0.5,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let other = edge(
        &mut topo,
        EdgeCurve::Circle(other_circle.clone()),
        other_circle.evaluate(0.0),
        other_circle.evaluate(TAU),
        Some((0.0, TAU)),
    );
    let options = ProjectCurveOptions {
        allow_approximate: true,
        ..ProjectCurveOptions::default()
    };
    let face = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|id| matches!(topo.face(*id).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let mut reference = topo.clone();
    let mut expected = 0.0_f64;
    for id in [source, other] {
        let result =
            project_curve_onto_face(&mut reference, id, direction, face, &options).unwrap();
        let ProjectionQuality::Approximate { max_deviation } = result.quality else {
            panic!("expected fitted face image");
        };
        expected = expected.max(max_deviation);
    }
    let result = project_curves_onto_solid(
        &mut topo,
        &[source, exact, other],
        direction,
        solid,
        &options,
    )
    .unwrap();
    let ProjectionQuality::Approximate { max_deviation } = result.quality else {
        panic!("solid quality lost the fitted image");
    };
    assert!((max_deviation - expected).abs() < 1e-15);
    assert!(max_deviation > 0.0);
    assert_eq!(
        result.sources.iter().map(|s| s.source).collect::<Vec<_>>(),
        vec![source, exact, other]
    );
    for item in &result.sources {
        assert!(!item.clipped);
        assert_eq!(item.edges.len(), 1);
        assert_eq!(item.edges[0].face, face);
    }
    assert!(matches!(
        topo.edge(result.sources[1].edges[0].edge).unwrap().curve(),
        EdgeCurve::Line
    ));
    assert!(matches!(
        topo.edge(result.sources[0].edges[0].edge).unwrap().curve(),
        EdgeCurve::NurbsCurve(_)
    ));
}

#[test]
fn solid_approximation_refuses_budget_and_uncertified_visibility_atomically() {
    for (direction, options, budget) in [
        (
            Vec3::new(0.0, -1.0, 0.0),
            ProjectCurveOptions {
                allow_approximate: true,
                approximation_tolerance: Some(1e-6),
                max_control_points: 4,
                ..ProjectCurveOptions::default()
            },
            true,
        ),
        (
            Vec3::new(0.0, -1.0, -1.0),
            ProjectCurveOptions {
                allow_approximate: true,
                ..ProjectCurveOptions::default()
            },
            false,
        ),
    ] {
        let (mut topo, solid, source) = solid_approx_fixture();
        let baseline = (topo.num_vertices(), topo.num_edges(), topo.journal().len());
        let result = project_curves_onto_solid(&mut topo, &[source], direction, solid, &options);
        let Err(ProjectCurveError::SourceRefused { index, error }) = result else {
            panic!("expected atomic source refusal");
        };
        assert_eq!(index, 0);
        if budget {
            assert!(matches!(
                *error,
                ProjectCurveError::ToleranceUnattainable { .. }
            ));
        } else {
            assert!(matches!(
                *error,
                ProjectCurveError::ApproximateClipUnsupported
            ));
        }
        assert_eq!(
            baseline,
            (topo.num_vertices(), topo.num_edges(), topo.journal().len())
        );
    }
}

#[test]
fn later_solid_refusal_does_not_commit_computed_approximate_source() {
    let (mut topo, solid, source) = solid_approx_fixture();
    let parallel = edge(
        &mut topo,
        EdgeCurve::Line,
        Point3::new(0.0, 6.0, 5.0),
        Point3::new(0.0, 5.0, 5.0),
        None,
    );
    let baseline = (topo.num_vertices(), topo.num_edges(), topo.journal().len());
    let options = ProjectCurveOptions {
        allow_approximate: true,
        ..ProjectCurveOptions::default()
    };
    let result = project_curves_onto_solid(
        &mut topo,
        &[source, parallel],
        Vec3::new(0.0, -1.0, 0.0),
        solid,
        &options,
    );
    assert!(
        matches!(result, Err(ProjectCurveError::SourceRefused { index: 1, error }) if matches!(*error, ProjectCurveError::SourceParallelToDirection))
    );
    assert_eq!(
        baseline,
        (topo.num_vertices(), topo.num_edges(), topo.journal().len())
    );
}

fn oblique_cap_fixture(x: f64) -> (Topology, remus_topology::solid::SolidId, EdgeId) {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 2.0, 10.0).unwrap();
    let circle = Circle3D::new_with_ref(
        Point3::new(x, 0.0, 12.0),
        Vec3::new(0.0, 0.0, 1.0),
        0.5,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let source = edge(
        &mut topo,
        EdgeCurve::Circle(circle.clone()),
        circle.evaluate(0.0),
        circle.evaluate(TAU),
        Some((0.0, TAU)),
    );
    (topo, solid, source)
}

#[test]
fn oblique_circle_keeps_exact_cap_before_later_curved_hit() {
    for allow_approximate in [false, true] {
        for scale in [1e-300, 1.0, 1e300] {
            let (mut topo, solid, source) = oblique_cap_fixture(0.0);
            let baseline = topo.journal().len();
            let options = ProjectCurveOptions {
                allow_approximate,
                ..ProjectCurveOptions::default()
            };
            let result = project_curves_onto_solid(
                &mut topo,
                &[source],
                Vec3::new(0.5 * scale, 0.0, -scale),
                solid,
                &options,
            )
            .unwrap();
            assert_eq!(result.quality, ProjectionQuality::Exact);
            assert_eq!(result.sources[0].edges.len(), 1);
            assert!(!result.sources[0].clipped);
            let image = &result.sources[0].edges[0];
            assert_eq!(image.source_range, (0.0, TAU));
            assert!(
                matches!(topo.face(image.face).unwrap().surface(), FaceSurface::Plane { normal, d } if normal.z() > 0.9 && (*d - 10.0).abs() < 1e-9)
            );
            let edge = topo.edge(image.edge).unwrap();
            assert_eq!(edge.start(), edge.end());
            let EdgeCurve::Circle(circle) = edge.curve() else {
                panic!("expected exact circle");
            };
            assert!((circle.center() - Point3::new(1.0, 0.0, 10.0)).length() < 1e-9);
            assert!((circle.radius() - 0.5).abs() < 1e-9);
            assert_eq!(topo.journal().len(), baseline);
            // The same ray starting at the source center later meets the
            // lateral support at (2,0,8), strictly behind its cap hit.
            assert!(circle.center().z() > 8.0);
        }
    }
}

#[test]
fn clipped_or_occluded_cap_cannot_omit_approximate_curved_candidate() {
    for x in [0.75, -3.0] {
        for allow_approximate in [false, true] {
            let (mut topo, solid, source) = oblique_cap_fixture(x);
            let baseline = (topo.num_vertices(), topo.num_edges(), topo.journal().len());
            let options = ProjectCurveOptions {
                allow_approximate,
                ..ProjectCurveOptions::default()
            };
            let result = project_curves_onto_solid(
                &mut topo,
                &[source],
                Vec3::new(0.5, 0.0, -1.0),
                solid,
                &options,
            );
            let Err(ProjectCurveError::SourceRefused { error, .. }) = result else {
                panic!("expected visibility refusal");
            };
            if allow_approximate {
                assert!(matches!(
                    *error,
                    ProjectCurveError::ApproximateClipUnsupported
                ));
            } else {
                assert!(matches!(*error, ProjectCurveError::ApproximationRequired));
            }
            assert_eq!(
                baseline,
                (topo.num_vertices(), topo.num_edges(), topo.journal().len())
            );
        }
    }
}

#[test]
fn tiny_cap_hole_cannot_inherit_manufactured_whole_source_coverage() {
    use remus_topology::wire::{OrientedEdge, Wire};
    for allow_approximate in [false, true] {
        let (mut topo, solid, source) = oblique_cap_fixture(0.0);
        let cap = solid_faces(&topo, solid).unwrap().into_iter().find(|id| matches!(topo.face(*id).unwrap().surface(), FaceSurface::Plane { normal, d } if normal.z() > 0.9 && (*d - 10.0).abs() < 1e-9)).unwrap();
        let theta: f64 = 0.2;
        let hole_circle = Circle3D::new_with_ref(
            Point3::new(1.0 + 0.5 * theta.cos(), 0.5 * theta.sin(), 10.0),
            Vec3::new(0.0, 0.0, 1.0),
            1e-10,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let vertex = topo.add_vertex(Vertex::new(hole_circle.evaluate(0.0), 1e-12));
        let mut rim = Edge::new(vertex, vertex, EdgeCurve::Circle(hole_circle));
        rim.set_trim(Some((0.0, TAU)));
        let rim = topo.add_edge(rim);
        let hole = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true).unwrap());
        let outer = topo.face(cap).unwrap().outer_wire();
        topo.set_face_boundary_wires(cap, outer, vec![hole])
            .unwrap();
        let baseline = (topo.num_vertices(), topo.num_edges(), topo.journal().len());
        let options = ProjectCurveOptions {
            allow_approximate,
            ..ProjectCurveOptions::default()
        };
        let result = project_curves_onto_solid(
            &mut topo,
            &[source],
            Vec3::new(0.5, 0.0, -1.0),
            solid,
            &options,
        );
        assert!(matches!(
            result,
            Err(ProjectCurveError::SourceRefused { .. })
        ));
        assert_eq!(
            baseline,
            (topo.num_vertices(), topo.num_edges(), topo.journal().len())
        );
    }
}
