#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stderr,
    clippy::float_cmp
)]
use super::*;
use remus_math::context::{CancellationToken, WorkBudgets};
use remus_math::curves2d::Line2D;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Vec2, Vec3};
use remus_topology::pcurve::PCurve;
use remus_topology::wire::OrientedEdge;
use remus_topology::{edge::Edge, face::Face, vertex::Vertex, wire::Wire};

fn knots(n: usize, d: (f64, f64)) -> Vec<f64> {
    [vec![d.0; n + 1], vec![d.1; n + 1]].concat()
}
fn near(a: f64, b: f64) {
    assert!((a - b).abs() < 2e-10 * (1.0 + b.abs()), "{a} != {b}");
}
fn place(p: Point3, scale: f64, moved: bool) -> Point3 {
    if moved {
        // Orthogonal rotation with a non-axis-aligned image of every axis.
        Point3::new(
            (p.x() / 3.0 + 2.0 * p.y() / 3.0 + 2.0 * p.z() / 3.0 + 7.0) * scale,
            (2.0 * p.x() / 3.0 + p.y() / 3.0 - 2.0 * p.z() / 3.0 - 11.0) * scale,
            (-2.0 * p.x() / 3.0 + 2.0 * p.y() / 3.0 - p.z() / 3.0 + 3.0) * scale,
        )
    } else {
        Point3::new(p.x() * scale, p.y() * scale, p.z() * scale)
    }
}
fn section_and_surfaces(
    rational: bool,
    scale: f64,
    moved: bool,
    domains: [(f64, f64); 2],
) -> (NurbsCurve, [NurbsSurface; 2]) {
    let points = if rational {
        vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 1.0),
        ]
    } else {
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(1.0, 0.0, 1.0),
        ]
    };
    let weights = if rational {
        vec![1.0, std::f64::consts::FRAC_1_SQRT_2, 1.0]
    } else {
        vec![1.0; 3]
    };
    let section = NurbsCurve::new(
        2,
        knots(2, (-8.0, 24.0)),
        points.iter().map(|p| place(*p, scale, moved)).collect(),
        weights.clone(),
    )
    .unwrap();
    let surfaces = std::array::from_fn(|j| {
        let d = if j == 0 {
            Vec3::new(0.0, 1.0, 0.0)
        } else if rational {
            Vec3::new(1.0, 1.0, 1.0)
        } else {
            Vec3::new(0.0, 1.0, 1.0)
        };
        NurbsSurface::new(
            2,
            1,
            knots(2, domains[j]),
            knots(1, (-2.0, 2.0)),
            points
                .iter()
                .map(|p| {
                    vec![
                        place(*p - d * 0.5, scale, moved),
                        place(*p + d * 0.5, scale, moved),
                    ]
                })
                .collect(),
            weights.iter().map(|w| vec![*w, *w]).collect(),
        )
        .unwrap()
    });
    (section, surfaces)
}
// Independent numeric de Casteljau construction of fixture edges. Expected
// material intervals below come from rectangles, never this construction.
fn split_fixture(h: &[[f64; 4]], t: f64) -> (Vec<[f64; 4]>, Vec<[f64; 4]>) {
    let mut row = h.to_vec();
    let mut a = vec![row[0]];
    let mut b = vec![row[row.len() - 1]];
    while row.len() > 1 {
        row = row
            .windows(2)
            .map(|w| std::array::from_fn(|j| (1.0 - t) * w[0][j] + t * w[1][j]))
            .collect();
        a.push(row[0]);
        b.push(row[row.len() - 1]);
    }
    b.reverse();
    (a, b)
}
fn fixture_edge(s: &NurbsSurface, a: Point2, b: Point2, reverse: bool) -> EdgeCurve {
    if same(a.x(), b.x()) {
        return EdgeCurve::Line;
    }
    let d = s.domain_u();
    let u0 = (a.x() - d.0) / (d.1 - d.0);
    let u1 = (b.x() - d.0) / (d.1 - d.0);
    let v = (a.y() + 2.0) / 4.0;
    let h: Vec<_> = s
        .control_points()
        .iter()
        .zip(s.weights())
        .map(|(p, w)| {
            let q = p[0] + (p[1] - p[0]) * v;
            [q.x() * w[0], q.y() * w[0], q.z() * w[0], w[0]]
        })
        .collect();
    let (_, right) = split_fixture(&h, u0.min(u1));
    let (mut sub, _) = split_fixture(&right, (u0.max(u1) - u0.min(u1)) / (1.0 - u0.min(u1)));
    if (u0 > u1) ^ reverse {
        sub.reverse();
    }
    EdgeCurve::NurbsCurve(
        NurbsCurve::new(
            2,
            knots(2, (7.0, 11.0)),
            sub.iter()
                .map(|p| Point3::new(p[0] / p[3], p[1] / p[3], p[2] / p[3]))
                .collect(),
            sub.iter().map(|p| p[3]).collect(),
        )
        .unwrap(),
    )
}
fn face(topo: &mut Topology, s: NurbsSurface, rects: &[[f64; 4]], reverse: bool) -> FaceId {
    let domain = s.domain_u();
    let u = |x| domain.0 + (domain.1 - domain.0) * x;
    let chart: Vec<_> = rects
        .iter()
        .map(|&[x0, x1, y0, y1]| [u(x0), u(x1), y0, y1])
        .collect();
    face_in_chart(topo, s, &chart, reverse, true)
}
/// `face` with rectangle corners in the surface's native u chart; `closed`
/// is the wire's closure flag, which the loop inherits.
fn face_in_chart(
    topo: &mut Topology,
    s: NurbsSurface,
    rects: &[[f64; 4]],
    reverse: bool,
    closed: bool,
) -> FaceId {
    let mut wires = Vec::new();
    let mut pcurves = Vec::new();
    for (r, rect) in rects.iter().enumerate() {
        let [u0, u1, y0, y1] = *rect;
        let mut points = [
            Point2::new(u0, y0),
            Point2::new(u1, y0),
            Point2::new(u1, y1),
            Point2::new(u0, y1),
        ];
        if r > 0 {
            points.reverse();
        }
        let vertices: Vec<_> = points
            .iter()
            .map(|p| topo.add_vertex(Vertex::new(s.evaluate(p.x(), p.y()), 1e-7)))
            .collect();
        let mut edges = Vec::new();
        for i in 0..4 {
            let (a, b) = (points[i], points[(i + 1) % 4]);
            let rev = reverse;
            let curve = fixture_edge(&s, a, b, rev);
            let (start, end) = if rev {
                (vertices[(i + 1) % 4], vertices[i])
            } else {
                (vertices[i], vertices[(i + 1) % 4])
            };
            let mut edge = Edge::new(start, end, curve);
            if matches!(edge.curve(), EdgeCurve::NurbsCurve(_)) {
                edge.set_trim(Some((7.0, 11.0)));
            }
            let id = topo.add_edge(edge);
            edges.push(OrientedEdge::new(id, !rev));
            // Axis sides: |dx| + |dy| is the exact length, and the unit
            // direction survives subnormal sides whose d·d underflows.
            let d = b - a;
            let length = d.x().abs() + d.y().abs();
            let direction = Vec2::new(d.x() / length, d.y() / length);
            pcurves.push((
                id,
                !rev,
                PCurve::new(
                    Curve2D::Line(Line2D::new(a, direction).unwrap()),
                    0.0,
                    length,
                ),
            ));
        }
        wires.push(topo.add_wire(Wire::new(edges, closed).unwrap()));
    }
    let id = topo.add_face(Face::new(
        wires[0],
        wires[1..].to_vec(),
        FaceSurface::Nurbs(s),
    ));
    for (edge, forward, pc) in pcurves {
        topo.set_pcurve_oriented(edge, id, forward, pc).unwrap();
    }
    id
}
pub(in crate::pave_filler) fn fixture(
    a: &[[f64; 4]],
    b: &[[f64; 4]],
    rational: bool,
    scale: f64,
    moved: bool,
    reverse: bool,
    domains: [(f64, f64); 2],
) -> (Topology, [FaceTrace; 2], NurbsCurve) {
    let (section, surfaces) = section_and_surfaces(rational, scale, moved, domains);
    let mut topo = Topology::new();
    let traces = std::array::from_fn(|j| FaceTrace {
        face: face(
            &mut topo,
            surfaces[j].clone(),
            if j == 0 { a } else { b },
            reverse,
        ),
        v: 0.0,
    });
    (topo, traces, section)
}
/// `fixture` with polynomial geometry, rectangles in each face's native u
/// chart, and the given chart domains.
fn chart_fixture(
    a: &[[f64; 4]],
    b: &[[f64; 4]],
    domains: [(f64, f64); 2],
) -> (Topology, [FaceTrace; 2], NurbsCurve) {
    let (section, surfaces) = section_and_surfaces(false, 1.0, false, domains);
    let mut topo = Topology::new();
    let traces = std::array::from_fn(|j| FaceTrace {
        face: face_in_chart(
            &mut topo,
            surfaces[j].clone(),
            if j == 0 { a } else { b },
            false,
            true,
        ),
        v: 0.0,
    });
    (topo, traces, section)
}
fn run(a: &[[f64; 4]], b: &[[f64; 4]]) -> (Topology, [FaceTrace; 2], NurbsCurve, ClippedSection) {
    let (topo, traces, section) = fixture(a, b, false, 1.0, false, true, [(0.0, 1.0); 2]);
    let clip = clip_section(&topo, traces, &section, &OperationContext::new()).unwrap();
    (topo, traces, section, clip)
}
fn assert_intervals(clip: &ClippedSection, expected: &[[f64; 2]]) {
    assert_eq!(clip.intervals.len(), expected.len());
    for (got, want) in clip.intervals.iter().zip(expected) {
        near(got.source_range[0], -8.0 + 32.0 * want[0]);
        near(got.source_range[1], -8.0 + 32.0 * want[1]);
    }
    eprintln!(
        "retained={:?}; residual_bounds={:?}; event_residuals={:?}",
        clip.intervals
            .iter()
            .map(|i| i.source_range)
            .collect::<Vec<_>>(),
        clip.surface_residual_bounds,
        clip.events
            .iter()
            .map(|e| e.surface_residuals)
            .collect::<Vec<_>>()
    );
}
const FULL: [f64; 4] = [0.0, 1.0, -1.5, 1.5];

#[test]
fn outer_crossings_retain_native_source_and_per_use_identity() {
    let (topo, _, _, clip) = run(&[[0.125, 0.875, -1.0, 1.0]], &[FULL]);
    assert_intervals(&clip, &[[0.125, 0.875]]);
    assert_eq!(clip.intervals[0].endpoints[0].len(), 1);
    for event in &clip.events {
        let coedge = topo.coedge(event.coedge).unwrap();
        assert_eq!(event.edge, coedge.edge());
        assert_eq!(event.forward, coedge.is_forward());
        assert_eq!(event.boundary_loop, coedge.parent_loop());
        let uv = coedge.pcurve().unwrap().evaluate(event.pcurve_parameter);
        let p = topo
            .face(event.face)
            .unwrap()
            .surface()
            .evaluate(uv.x(), uv.y())
            .unwrap();
        let edge = topo.edge(event.edge).unwrap();
        let q = edge.curve().evaluate_with_endpoints(
            event.edge_parameter,
            topo.vertex(edge.start()).unwrap().point(),
            topo.vertex(edge.end()).unwrap().point(),
        );
        near((p - q).length(), 0.0);
    }
}
#[test]
fn hole_and_partner_trim_keep_all_material_intervals() {
    let (_, _, _, clip) = run(
        &[
            [0.125, 0.875, -1.0, 1.0],
            [0.25, 0.375, -0.5, 0.5],
            [0.625, 0.75, -0.5, 0.5],
        ],
        &[[0.1875, 0.8125, -1.5, 1.5]],
    );
    assert_intervals(&clip, &[[0.1875, 0.25], [0.375, 0.625], [0.75, 0.8125]]);
    assert_eq!(clip.events.len(), 8);
}
#[test]
fn wholly_outside_trim_or_inside_hole_is_empty() {
    assert_intervals(&run(&[[0.125, 0.875, 0.5, 1.0]], &[FULL]).3, &[]);
    assert_intervals(
        &run(
            &[FULL, [0.125, 0.875, -0.5, 0.5]],
            &[[0.25, 0.75, -1.0, 1.0]],
        )
        .3,
        &[],
    );
}
#[test]
fn full_source_endpoints_keep_both_boundary_uses() {
    let (_, _, _, clip) = run(&[FULL], &[FULL]);
    assert_intervals(&clip, &[[0.0, 1.0]]);
    assert_eq!(clip.intervals[0].endpoints[0].len(), 2);
    assert_eq!(clip.intervals[0].endpoints[1].len(), 2);
    assert_ne!(
        clip.intervals[0].endpoints[0][0].coedge,
        clip.intervals[0].endpoints[0][1].coedge
    );
}
#[test]
fn curved_nurbs_matrix_has_independent_material_and_locus_oracles() {
    let domains = [
        [(0.0, 1.0); 2],
        [(16.0, 24.0), (-4.0, -2.0)],
        [(0.0, 2_f64.powi(-30)), (0.0, 2_f64.powi(20))],
    ];
    for rational in [false, true] {
        for scale in [1e-3, 1.0, 1e3] {
            for moved in [false, true] {
                for reverse in [false, true] {
                    for d in domains {
                        let (topo, traces, section) = fixture(
                            &[[0.125, 0.875, -1.0, 1.0], [0.375, 0.625, -0.5, 0.5]],
                            &[FULL],
                            rational,
                            scale,
                            moved,
                            reverse,
                            d,
                        );
                        for trace in traces {
                            assert!(matches!(
                                topo.face(trace.face).unwrap().surface(),
                                FaceSurface::Nurbs(_)
                            ));
                            assert!(
                                super::super::helpers::planar_nurbs_as_plane(
                                    topo.face(trace.face).unwrap().surface(),
                                    Tolerance::default()
                                )
                                .is_none()
                            );
                        }
                        let context = OperationContext::new().with_tolerance(Tolerance {
                            linear: 1e-7 * scale,
                            ..Tolerance::default()
                        });
                        let clip = clip_section(&topo, traces, &section, &context).unwrap();
                        eprintln!(
                            "matrix rational={rational} scale={scale} moved={moved} reversed={reverse} domains={d:?}"
                        );
                        assert_intervals(&clip, &[[0.125, 0.375], [0.625, 0.875]]);
                        for k in 0..=32 {
                            let f = f64::from(k) / 32.0;
                            let expected_material =
                                (0.125..=0.875).contains(&f) && !(f > 0.375 && f < 0.625);
                            let actual = clip.intervals.iter().any(|r| {
                                (-8.0 + 32.0 * f) >= r.source_range[0]
                                    && (-8.0 + 32.0 * f) <= r.source_range[1]
                            });
                            assert_eq!(actual, expected_material);
                            let expected = if rational {
                                let w = std::f64::consts::FRAC_1_SQRT_2;
                                let denominator =
                                    (1.0 - f).powi(2) + 2.0 * w * f * (1.0 - f) + f * f;
                                let x = ((1.0 - f).powi(2) + 2.0 * w * f * (1.0 - f)) / denominator;
                                let z = (2.0 * w * f * (1.0 - f) + f * f) / denominator;
                                near(x * x + z * z, 1.0);
                                Point3::new(x, 0.0, z)
                            } else {
                                Point3::new(f, 0.0, f * f)
                            };
                            assert!(
                                (section.evaluate(-8.0 + 32.0 * f) - place(expected, scale, moved))
                                    .length()
                                    < 1e-10 * scale
                            );
                        }
                    }
                }
            }
        }
    }
}
#[test]
fn boundary_overlap_and_corner_contacts_refuse() {
    let (topo, mut traces, section) = fixture(
        &[[0.125, 0.875, 0.0, 1.0]],
        &[FULL],
        false,
        1.0,
        false,
        false,
        [(0.0, 1.0); 2],
    );
    assert!(matches!(
        clip_section(&topo, traces, &section, &OperationContext::new()),
        Err(ClipError::AmbiguousBoundary)
    ));
    traces[0].v = f64::NAN;
    assert!(matches!(
        clip_section(&topo, traces, &section, &OperationContext::new()),
        Err(ClipError::InvalidInput)
    ));
}
#[test]
fn fitted_section_error_is_not_hidden_by_a_weld() {
    let (topo, traces, section) =
        fixture(&[FULL], &[FULL], false, 1.0, false, false, [(0.0, 1.0); 2]);
    let mut points = section.control_points().to_vec();
    points[1] = points[1] + Vec3::new(0.0, 5e-5, 0.0);
    let wrong = NurbsCurve::new(
        2,
        section.knots().to_vec(),
        points,
        section.weights().to_vec(),
    )
    .unwrap();
    assert!(matches!(
        clip_section(&topo, traces, &wrong, &OperationContext::new()),
        Err(ClipError::Residual { .. })
    ));
}
#[test]
fn missing_or_false_pcurve_authority_refuses_without_mutation() {
    let (mut topo, traces, section) =
        fixture(&[FULL], &[FULL], false, 1.0, false, true, [(0.0, 1.0); 2]);
    let cid = topo
        .face_loop(topo.loops_of_face(traces[0].face).unwrap()[0])
        .unwrap()
        .coedges()[0];
    topo.remove_coedge_pcurve(cid).unwrap();
    let before = format!("{topo:?}");
    assert!(matches!(
        clip_section(&topo, traces, &section, &OperationContext::new()),
        Err(ClipError::MissingAuthority)
    ));
    assert_eq!(format!("{topo:?}"), before);
    topo.set_coedge_pcurve(
        cid,
        PCurve::new(
            Curve2D::Line(Line2D::new(Point2::new(0.0, -1.0), Vec2::new(1.0, 0.0)).unwrap()),
            0.0,
            1.0,
        ),
    )
    .unwrap();
    let before = format!("{topo:?}");
    assert!(matches!(
        clip_section(&topo, traces, &section, &OperationContext::new()),
        Err(ClipError::Residual { .. })
    ));
    assert_eq!(format!("{topo:?}"), before);
}
#[test]
fn nontransverse_pair_budget_and_cancellation_refuse() {
    let (topo, mut traces, section) =
        fixture(&[FULL], &[FULL], false, 1.0, false, false, [(0.0, 1.0); 2]);
    traces[1] = traces[0];
    assert!(matches!(
        clip_section(&topo, traces, &section, &OperationContext::new()),
        Err(ClipError::UnresolvedContact)
    ));
    let context = OperationContext::new().with_budgets(WorkBudgets::new().with_segments(0));
    assert!(matches!(
        clip_section(&topo, traces, &section, &context),
        Err(ClipError::WorkBudgetExceeded)
    ));
    let token = CancellationToken::new();
    token.cancel();
    let mut context = OperationContext::new();
    context.cancellation = Some(token);
    assert!(matches!(
        clip_section(&topo, traces, &section, &context),
        Err(ClipError::Cancelled)
    ));
}

#[test]
fn sub_tolerance_hole_is_not_welded_away() {
    let width = 2_f64.powi(-30);
    let (_, _, _, clip) = run(&[FULL, [0.5 - width, 0.5 + width, -0.5, 0.5]], &[FULL]);
    assert_intervals(&clip, &[[0.0, 0.5 - width], [0.5 + width, 1.0]]);
    assert!(clip.intervals[1].source_range[0] > clip.intervals[0].source_range[1]);
}
#[test]
fn nearby_events_and_foreign_chart_coincidence_require_proof() {
    let (topo, traces, section) = fixture(
        &[[0.5, 0.875, -1.0, 1.0]],
        &[[0.5_f64.next_up(), 1.0, -1.5, 1.5]],
        false,
        1.0,
        false,
        false,
        [(0.0, 1.0); 2],
    );
    assert!(matches!(
        clip_section(&topo, traces, &section, &OperationContext::new()),
        Err(ClipError::AmbiguousBoundary)
    ));
    let (topo, traces, section) = fixture(
        &[[0.125, 0.875, -1.0, 1.0]],
        &[[0.125, 1.0, -1.5, 1.5]],
        false,
        1.0,
        false,
        false,
        [(0.0, 1.0), (16.0, 24.0)],
    );
    assert!(matches!(
        clip_section(&topo, traces, &section, &OperationContext::new()),
        Err(ClipError::AmbiguousBoundary)
    ));
}
#[test]
fn periodic_pole_and_nonruled_patches_refuse() {
    for kind in 0..3 {
        let (mut topo, traces, section) =
            fixture(&[FULL], &[FULL], false, 1.0, false, false, [(0.0, 1.0); 2]);
        let FaceSurface::Nurbs(s) = topo.face(traces[0].face).unwrap().surface() else {
            panic!()
        };
        let mut points = s.control_points().to_vec();
        let mut weights = s.weights().to_vec();
        match kind {
            0 => points[2] = points[0].clone(),
            1 => points[0][1] = points[0][0],
            _ => weights[1][1] = 0.5,
        }
        let bad = NurbsSurface::new(
            2,
            1,
            s.knots_u().to_vec(),
            s.knots_v().to_vec(),
            points,
            weights,
        )
        .unwrap();
        topo.face_mut(traces[0].face)
            .unwrap()
            .set_surface(FaceSurface::Nurbs(bad));
        let before = format!("{topo:?}");
        assert!(matches!(
            clip_section(&topo, traces, &section, &OperationContext::new()),
            Err(ClipError::UnsupportedDomain | ClipError::UnresolvedContact)
        ));
        assert_eq!(format!("{topo:?}"), before);
    }
}
#[test]
fn missing_trim_periodic_use_and_touching_holes_refuse() {
    let (mut topo, traces, section) =
        fixture(&[FULL], &[FULL], false, 1.0, false, false, [(0.0, 1.0); 2]);
    let cid = topo
        .face_loop(topo.loops_of_face(traces[0].face).unwrap()[0])
        .unwrap()
        .coedges()[0];
    let eid = topo.coedge(cid).unwrap().edge();
    topo.edge_mut(eid).unwrap().set_trim(None);
    assert!(matches!(
        clip_section(&topo, traces, &section, &OperationContext::new()),
        Err(ClipError::MissingAuthority)
    ));
    topo.edge_mut(eid).unwrap().set_trim(Some((7.0, 11.0)));
    topo.set_coedge_periodic_winding(cid, PeriodicWinding::new(1, 0))
        .unwrap();
    assert!(matches!(
        clip_section(&topo, traces, &section, &OperationContext::new()),
        Err(ClipError::UnsupportedDomain)
    ));
    let (topo, traces, section) = fixture(
        &[FULL, [0.25, 0.5, -0.5, 0.5], [0.5, 0.75, -0.5, 0.5]],
        &[FULL],
        false,
        1.0,
        false,
        false,
        [(0.0, 1.0); 2],
    );
    assert!(matches!(
        clip_section(&topo, traces, &section, &OperationContext::new()),
        Err(ClipError::InvalidBoundary)
    ));
}
#[test]
fn interior_fit_error_with_agreeing_endpoints_and_midpoint_refuses() {
    let (topo, traces, section) =
        fixture(&[FULL], &[FULL], false, 1.0, false, false, [(0.0, 1.0); 2]);
    let wrong = NurbsCurve::new(
        3,
        knots(3, section.domain()),
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0 / 3.0, 1e-3, 0.0),
            Point3::new(2.0 / 3.0, -1e-3, 1.0 / 3.0),
            Point3::new(1.0, 0.0, 1.0),
        ],
        vec![1.0; 4],
    )
    .unwrap();
    for t in [-8.0, 8.0, 24.0] {
        near((wrong.evaluate(t) - section.evaluate(t)).length(), 0.0);
    }
    assert!(matches!(
        clip_section(&topo, traces, &wrong, &OperationContext::new()),
        Err(ClipError::Residual { .. })
    ));
}

#[test]
fn section_native_knot_domains_do_not_change_material() {
    let (topo, traces, section) = fixture(
        &[[0.125, 0.875, -1.0, 1.0], [0.375, 0.625, -0.5, 0.5]],
        &[FULL],
        true,
        1.0,
        false,
        true,
        [(0.0, 1.0); 2],
    );
    for domain in [
        (2_f64.powi(-40), 2_f64.powi(-39)),
        (-1_048_576.0, -1_048_568.0),
    ] {
        let curve = NurbsCurve::new(
            2,
            knots(2, domain),
            section.control_points().to_vec(),
            section.weights().to_vec(),
        )
        .unwrap();
        let clip = clip_section(&topo, traces, &curve, &OperationContext::new()).unwrap();
        assert_eq!(clip.intervals.len(), 2);
        for (interval, expected) in clip.intervals.iter().zip([[0.125, 0.375], [0.625, 0.875]]) {
            for (t, want) in interval.source_range.into_iter().zip(expected) {
                near((t - domain.0) / (domain.1 - domain.0), want);
            }
        }
        for e in clip.events {
            assert!(
                e.section_parameter_bound[0] <= e.section_parameter
                    && e.section_parameter <= e.section_parameter_bound[1]
            );
        }
    }
}

#[test]
fn stored_boundary_subtrim_is_authoritative() {
    let (mut topo, traces, section) = fixture(
        &[[0.125, 0.875, -1.0, 1.0]],
        &[FULL],
        true,
        1.0,
        false,
        false,
        [(0.0, 1.0); 2],
    );
    let surface = topo.face(traces[0].face).unwrap().surface().clone();
    let FaceSurface::Nurbs(s) = surface else {
        panic!()
    };
    let ids = topo
        .face_loop(topo.loops_of_face(traces[0].face).unwrap()[0])
        .unwrap()
        .coedges()
        .to_vec();
    for cid in ids {
        let coedge = topo.coedge(cid).unwrap();
        let id = coedge.edge();
        let pc = coedge.pcurve().unwrap();
        let a = pc.evaluate(pc.t_start());
        let b = pc.evaluate(pc.t_end());
        if a.y() != b.y() {
            continue;
        }
        let reverse = a.x() > b.x();
        let curve = fixture_edge(
            &s,
            Point2::new(0.0, a.y()),
            Point2::new(1.0, a.y()),
            reverse,
        );
        let edge = topo.edge_mut(id).unwrap();
        edge.set_curve(curve);
        edge.set_trim(Some((7.5, 10.5)));
    }
    let clipped = clip_section(&topo, traces, &section, &OperationContext::new()).unwrap();
    assert_intervals(&clipped, &[[0.125, 0.875]]);
}

#[test]
fn bernstein_residual_bounds_include_interior_deviation_and_weight_scaling() {
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.5, 0.0, 0.0),
        Point3::new(1.0, 0.0, 1.0),
    ];
    let mut moved = points;
    moved[1] = moved[1] + Vec3::new(0.0, 0.25, 0.0);
    let h = bernstein::homogeneous(&points, &[1.0, 0.5, 1.0], points[0]);
    let k = bernstein::homogeneous(&moved, &[8.0, 4.0, 8.0], points[0]);
    let bound = bernstein::residual(&h, &k);
    assert!((1.0 / 12.0..0.51).contains(&bound));
    let actual = NurbsCurve::new(
        2,
        knots(2, (0.0, 1.0)),
        points.to_vec(),
        vec![1.0, 0.5, 1.0],
    )
    .unwrap();
    let other =
        NurbsCurve::new(2, knots(2, (0.0, 1.0)), moved.to_vec(), vec![1.0, 0.5, 1.0]).unwrap();
    for k in 0..=128 {
        let t = f64::from(k) / 128.0;
        assert!((actual.evaluate(t) - other.evaluate(t)).length() <= bound);
    }
}

#[test]
fn fraction_is_the_hand_computed_affine_coordinate() {
    // (0.75-0.5)/(1.0-0.5) = 0.5 exactly.
    near(fraction(0.75, (0.5, 1.0)).unwrap(), 0.5);
    // Endpoints map to 0 and 1, not to the constant mutants.
    near(fraction(0.5, (0.5, 1.0)).unwrap(), 0.0);
    near(fraction(1.0, (0.5, 1.0)).unwrap(), 1.0);
    // A quarter point: (0.0-(-1.0))/(1.0-(-1.0)) = 0.5.
    near(fraction(0.0, (-1.0, 1.0)).unwrap(), 0.5);
    // Outside the domain refuses; the || mutant would admit one side.
    assert!(fraction(0.49, (0.5, 1.0)).is_err());
    assert!(fraction(1.01, (0.5, 1.0)).is_err());
    assert!(fraction(0.75, (1.0, 0.5)).is_err());
}

#[test]
fn same_is_exact_equality_and_finite_rejects_non_finite() {
    assert!(same(1.0, 1.0));
    assert!(!same(1.0, 1.0 + 1e-12));
    assert!(finite(1.0).is_ok());
    assert!(finite(f64::NAN).is_err());
    assert!(finite(f64::INFINITY).is_err());
}

// Mutation oracles (B19 F3c). Each input trips exactly one clause of one gate;
// the expected verdict is the gate's documented refusal or a hand-computed
// value, never a re-run of the code under test.

fn outer_coedges(topo: &Topology, face: FaceId) -> Vec<CoedgeId> {
    topo.face_loop(topo.loops_of_face(face).unwrap()[0])
        .unwrap()
        .coedges()
        .to_vec()
}
fn chart_line(topo: &Topology, cid: CoedgeId) -> (Line2D, [f64; 2]) {
    let pc = topo.coedge(cid).unwrap().pcurve().unwrap();
    let Curve2D::Line(line) = pc.curve() else {
        panic!("fixture pcurves are lines")
    };
    (line.clone(), [pc.t_start(), pc.t_end()])
}
/// The first outer-loop coedge of `face` whose chart line is vertical
/// (`vertical`) or horizontal.
fn side(topo: &Topology, face: FaceId, vertical: bool) -> CoedgeId {
    outer_coedges(topo, face)
        .into_iter()
        .find(|&cid| same(chart_line(topo, cid).0.direction().x(), 0.0) == vertical)
        .unwrap()
}
fn set_chart(topo: &mut Topology, cid: CoedgeId, origin: Point2, direction: Vec2, range: [f64; 2]) {
    topo.set_coedge_pcurve(
        cid,
        PCurve::new(
            Curve2D::Line(Line2D::new(origin, direction).unwrap()),
            range[0],
            range[1],
        ),
    )
    .unwrap();
}
fn plain(a: &[[f64; 4]], b: &[[f64; 4]]) -> (Topology, [FaceTrace; 2], NurbsCurve) {
    fixture(a, b, false, 1.0, false, false, [(0.0, 1.0); 2])
}
fn clip(topo: &Topology, traces: [FaceTrace; 2], section: &NurbsCurve) -> Result<ClippedSection> {
    clip_section(topo, traces, section, &OperationContext::new())
}

#[test]
fn exact_sum_is_true_exactly_when_the_float_sum_rounds_nothing() {
    assert!(exact_sum(1.0, 0.5));
    assert!(exact_sum(-2.0, 3.0));
    assert!(exact_sum(0.375, 0.25));
    // 1 + 2^-60 rounds to 1, whichever operand is the larger.
    let tiny = 2_f64.powi(-60);
    assert!(!exact_sum(1.0, tiny));
    assert!(!exact_sum(tiny, 1.0));
    // 0.1 + 0.2 rounds to 0.30000000000000004.
    assert!(!exact_sum(0.1, 0.2));
}

#[test]
fn bezier_accepts_exactly_one_clamped_span_of_degree_one_to_three() {
    assert!(bezier(&knots(2, (0.0, 1.0)), 2, 3).is_ok());
    assert!(bezier(&knots(1, (-4.0, 2.0)), 1, 2).is_ok());
    assert!(bezier(&knots(3, (0.0, 1.0)), 3, 4).is_ok());
    let refused = |k: &[f64], degree, count| {
        matches!(bezier(k, degree, count), Err(ClipError::UnsupportedDomain))
    };
    // Degree 4 with a consistent count and clamped knots.
    assert!(refused(&knots(4, (0.0, 1.0)), 4, 5));
    // Count one more than degree + 1, knots consistent with the degree.
    assert!(refused(&knots(2, (0.0, 1.0)), 2, 4));
    // A decreasing span: clamped on both sides, finite width.
    assert!(refused(&[1.0, 1.0, 1.0, 0.0, 0.0, 0.0], 2, 3));
    // Finite ends whose width overflows: -1e308 .. 1e308.
    assert!(refused(&knots(2, (-1e308, 1e308)), 2, 3));
    // An interior knot in the low clamp, then in the high clamp.
    assert!(refused(&[0.0, 0.5, 1.0, 1.0], 1, 2));
    assert!(refused(&[0.0, 0.0, 0.5, 1.0], 1, 2));
}

#[test]
fn check_residual_accepts_a_bound_equal_to_the_tolerance_and_refuses_above() {
    let origin = Point3::new(0.0, 0.0, 0.0);
    let a = bernstein::homogeneous(&[origin], &[1.0], origin);
    let b = bernstein::homogeneous(&[Point3::new(3.0, 4.0, 0.0)], &[1.0], origin);
    let bound = bernstein::residual(&a, &b);
    // The certificate encloses the 3-4-5 distance, a few ulps outward.
    assert!((5.0..=5.0 * (1.0 + 1e-14)).contains(&bound));
    assert_eq!(check_residual(&a, &b, bound).unwrap(), bound);
    assert!(matches!(
        check_residual(&a, &b, bound.next_down()),
        Err(ClipError::Residual { .. })
    ));
}

#[test]
fn work_budget_admits_exactly_its_segment_count() {
    // One start unit, then four per loop: two one-loop faces peak at 8.
    let (topo, traces, section) = plain(&[FULL], &[FULL]);
    let at = |n| OperationContext::new().with_budgets(WorkBudgets::new().with_segments(n));
    assert!(clip_section(&topo, traces, &section, &at(8)).is_ok());
    assert!(matches!(
        clip_section(&topo, traces, &section, &at(7)),
        Err(ClipError::WorkBudgetExceeded)
    ));
}

#[test]
fn nonpositive_or_nonfinite_tolerance_is_invalid_input() {
    let (topo, traces, section) = plain(&[FULL], &[FULL]);
    for linear in [0.0, -1e-7, f64::NAN, f64::INFINITY] {
        let context = OperationContext::new().with_tolerance(Tolerance {
            linear,
            ..Tolerance::default()
        });
        assert!(
            matches!(
                clip_section(&topo, traces, &section, &context),
                Err(ClipError::InvalidInput)
            ),
            "tolerance {linear}"
        );
    }
}

#[test]
fn trims_wholly_above_or_below_the_trace_contribute_no_events() {
    // Only the partner's two full-width crossings remain.
    for rect in [[0.125, 0.875, 0.5, 1.0], [0.125, 0.875, -1.0, -0.5]] {
        let (_, traces, _, clip) = run(&[rect], &[FULL]);
        assert_intervals(&clip, &[]);
        assert_eq!(clip.events.len(), 2, "{rect:?}");
        assert!(clip.events.iter().all(|e| e.face == traces[1].face));
    }
    // A hole off the trace leaves only the four outer crossings.
    for hole in [[0.25, 0.75, 0.5, 1.0], [0.25, 0.75, -1.0, -0.5]] {
        let clip = run(&[FULL, hole], &[FULL]).3;
        assert_intervals(&clip, &[[0.0, 1.0]]);
        assert_eq!(clip.events.len(), 4, "{hole:?}");
    }
}

#[test]
fn nurbs_side_edges_map_the_trace_through_their_own_trim() {
    let (mut topo, traces, section) = plain(&[[0.125, 0.875, -1.0, 1.0]], &[FULL]);
    for face in traces.map(|t| t.face) {
        for cid in outer_coedges(&topo, face) {
            if !same(chart_line(&topo, cid).0.direction().x(), 0.0) {
                continue;
            }
            let eid = topo.coedge(cid).unwrap().edge();
            let edge = topo.edge(eid).unwrap();
            let ends = vec![
                topo.vertex(edge.start()).unwrap().point(),
                topo.vertex(edge.end()).unwrap().point(),
            ];
            let line = NurbsCurve::new(1, knots(1, (7.0, 11.0)), ends, vec![1.0; 2]).unwrap();
            let edge = topo.edge_mut(eid).unwrap();
            edge.set_curve(EdgeCurve::NurbsCurve(line));
            edge.set_trim(Some((7.0, 11.0)));
        }
    }
    let clipped = clip(&topo, traces, &section).unwrap();
    assert_intervals(&clipped, &[[0.125, 0.875]]);
    assert_eq!(clipped.events.len(), 4);
    for e in &clipped.events {
        // v = 0 halves every side, so the event sits at the trim midpoint 9.
        near(e.edge_parameter, 9.0);
        assert!(e.boundary_residual <= 1e-12);
    }
}

#[test]
fn coincident_interior_events_on_one_chart_are_one_proven_cut() {
    let clip = run(&[[0.125, 0.875, -1.0, 1.0]], &[[0.125, 0.875, -1.5, 1.5]]).3;
    assert_intervals(&clip, &[[0.125, 0.875]]);
    assert_eq!(clip.events.len(), 4);
    assert_eq!(clip.intervals[0].endpoints[0].len(), 2);
    assert_eq!(clip.intervals[0].endpoints[1].len(), 2);
}

#[test]
fn full_source_ends_on_foreign_charts_keep_both_uses() {
    let (topo, traces, section) = fixture(
        &[FULL],
        &[FULL],
        false,
        1.0,
        false,
        false,
        [(0.0, 1.0), (-1.0, 1.0)],
    );
    let clip = clip(&topo, traces, &section).unwrap();
    assert_intervals(&clip, &[[0.0, 1.0]]);
    assert_eq!(clip.intervals[0].endpoints[0].len(), 2);
    assert_eq!(clip.intervals[0].endpoints[1].len(), 2);
}

#[test]
fn overlapping_events_need_a_shared_chart_coordinate_or_source_end() {
    // 0.5 and 0.5 + 2^-50 (8 ulps): the outward fraction intervals overlap,
    // yet the cuts and every material sample stay separable, so only the
    // event-order proof can refuse.
    let (topo, traces, section) = plain(
        &[[0.5, 0.875, -1.0, 1.0]],
        &[[0.5 + 2_f64.powi(-50), 1.0, -1.5, 1.5]],
    );
    assert!(matches!(
        clip(&topo, traces, &section),
        Err(ClipError::AmbiguousBoundary)
    ));
    // Equal fractions 1/8 on charts (0, 1) and (-1, 1): chart u 0.125 vs
    // -0.75, the domains share only their upper end.
    let (topo, traces, section) = fixture(
        &[[0.125, 0.875, -1.0, 1.0]],
        &[[0.125, 1.0, -1.5, 1.5]],
        false,
        1.0,
        false,
        false,
        [(0.0, 1.0), (-1.0, 1.0)],
    );
    assert!(matches!(
        clip(&topo, traces, &section),
        Err(ClipError::AmbiguousBoundary)
    ));
}

#[test]
fn an_inexact_source_end_coincidence_is_not_a_shared_end() {
    // A chart edge whose fraction rounds to exactly 0 or 1 while its enclosure
    // does not, beside the partner's exact end use: 2^-1074 / 4 underflows to
    // 0, and (896 + 2^60) / (1024 + 2^60) rounds to 1 (896 + 2^60 ties to the
    // even 1024 + 2^60). Swapping the faces swaps the pair order. The section
    // is re-knotted over (0, 1) so a subnormal source window stays nonempty
    // in its native parameter and only the event-order proof can refuse.
    let near_zero = (
        (0.0, 4.0),
        [f64::from_bits(1), f64::MIN_POSITIVE, -1.0, 1.0],
        [0.5, 1.0, -1.0, 1.0],
    );
    let near_one = (
        (-(2_f64.powi(60)), 1024.0),
        [-(2_f64.powi(59)), 896.0, -1.0, 1.0],
        [-(2_f64.powi(59)), -(2_f64.powi(58)), -1.0, 1.0],
    );
    let full = [0.0, 1.0, -1.5, 1.5];
    for (domain, coincident, control) in [near_zero, near_one] {
        for swap in [false, true] {
            let pair = |rect: [f64; 4]| {
                let (a, b, d) = if swap {
                    (rect, full, [domain, (0.0, 1.0)])
                } else {
                    (full, rect, [(0.0, 1.0), domain])
                };
                let (topo, traces, section) = chart_fixture(&[a], &[b], d);
                let unit = NurbsCurve::new(
                    2,
                    knots(2, (0.0, 1.0)),
                    section.control_points().to_vec(),
                    section.weights().to_vec(),
                )
                .unwrap();
                clip(&topo, traces, &unit)
            };
            assert!(
                matches!(pair(coincident), Err(ClipError::AmbiguousBoundary)),
                "{domain:?} swap={swap}"
            );
            // The same chart without the coincidence clips.
            assert_eq!(
                pair(control).unwrap().intervals.len(),
                1,
                "{domain:?} swap={swap}"
            );
        }
    }
}

#[test]
fn adjacent_cuts_without_a_representable_midpoint_refuse() {
    // Chart (0, 1024) edge at 2^-1064: its fraction 2^-1074 is the smallest
    // subnormal, so the window (0, 2^-1074) has no midpoint, while its chart
    // sample at u = 0 is clear of every rectangle.
    let (topo, traces, section) = chart_fixture(
        &[[f64::from_bits(1 << 10), f64::MIN_POSITIVE, -1.0, 1.0]],
        &[[0.25, 0.75, -1.5, 1.5]],
        [(0.0, 1024.0), (0.0, 1.0)],
    );
    assert!(matches!(
        clip(&topo, traces, &section),
        Err(ClipError::AmbiguousBoundary)
    ));
}

#[test]
fn rectangle_membership_is_ambiguous_exactly_when_the_interval_touches_a_side() {
    let (topo, traces, section) = plain(&[FULL], &[FULL]);
    let origin = section.control_points()[0];
    let patch = patch(&topo, traces[0], origin).unwrap();
    let loops = rectangles(
        &topo,
        traces[0],
        &patch,
        origin,
        &OperationContext::new(),
        &mut 0,
    )
    .unwrap();
    assert_eq!(loops[0].bounds, [0.0, 1.0, -1.5, 1.5]);
    let u = |lo, hi| bernstein::I { lo, hi };
    assert!(inside(&loops, u(0.25, 0.75), 0.0).unwrap());
    assert!(!inside(&loops, u(-1.0, -0.5), 0.0).unwrap());
    assert!(!inside(&loops, u(1.5, 2.0), 0.0).unwrap());
    assert!(!inside(&loops, u(0.25, 0.75), 1.5).unwrap());
    for (lo, hi) in [(-1.0, 0.0), (1.0, 2.0), (-0.5, 0.5), (0.5, 1.5)] {
        assert!(
            matches!(
                inside(&loops, u(lo, hi), 0.0),
                Err(ClipError::AmbiguousBoundary)
            ),
            "[{lo}, {hi}]"
        );
    }
}

#[test]
fn a_loop_that_is_not_four_closed_uses_is_an_invalid_boundary() {
    let (section, surfaces) = section_and_surfaces(false, 1.0, false, [(0.0, 1.0); 2]);
    // A closed triangle: the diagonal is not an axis chart, but the use
    // count refuses first.
    let mut topo = Topology::new();
    let s = surfaces[0].clone();
    let corners = [
        Point2::new(0.125, -1.0),
        Point2::new(0.875, -1.0),
        Point2::new(0.875, 1.0),
    ];
    let vertices: Vec<_> = corners
        .iter()
        .map(|p| topo.add_vertex(Vertex::new(s.evaluate(p.x(), p.y()), 1e-7)))
        .collect();
    let mut edges = Vec::new();
    let mut pcurves = Vec::new();
    for i in 0..3 {
        let (a, b) = (corners[i], corners[(i + 1) % 3]);
        let curve = if same(a.y(), b.y()) {
            fixture_edge(&s, a, b, false)
        } else {
            EdgeCurve::Line
        };
        let mut edge = Edge::new(vertices[i], vertices[(i + 1) % 3], curve);
        if matches!(edge.curve(), EdgeCurve::NurbsCurve(_)) {
            edge.set_trim(Some((7.0, 11.0)));
        }
        let id = topo.add_edge(edge);
        edges.push(OrientedEdge::new(id, true));
        let line = Line2D::new(a, b - a).unwrap();
        pcurves.push((id, PCurve::new(Curve2D::Line(line), 0.0, (b - a).length())));
    }
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    let triangle = topo.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(s)));
    for (edge, pc) in pcurves {
        topo.set_pcurve_oriented(edge, triangle, true, pc).unwrap();
    }
    let full = face(&mut topo, surfaces[1].clone(), &[FULL], false);
    let traces = [
        FaceTrace {
            face: triangle,
            v: 0.0,
        },
        FaceTrace { face: full, v: 0.0 },
    ];
    assert!(matches!(
        clip(&topo, traces, &section),
        Err(ClipError::InvalidBoundary)
    ));
    // An open four-use loop.
    let mut topo = Topology::new();
    let open = face_in_chart(&mut topo, surfaces[0].clone(), &[FULL], false, false);
    let full = face(&mut topo, surfaces[1].clone(), &[FULL], false);
    assert!(
        !topo
            .face_loop(topo.loops_of_face(open).unwrap()[0])
            .unwrap()
            .is_closed()
    );
    let traces = [
        FaceTrace { face: open, v: 0.0 },
        FaceTrace { face: full, v: 0.0 },
    ];
    assert!(matches!(
        clip(&topo, traces, &section),
        Err(ClipError::InvalidBoundary)
    ));
}

#[test]
fn boundary_charts_must_be_exact_axis_lines_from_parameter_zero() {
    let rect = [0.375, 0.625, -0.125, 0.125];
    let (topo, traces, section) = plain(&[rect], &[FULL]);
    assert_intervals(&clip(&topo, traces, &section).unwrap(), &[[0.375, 0.625]]);
    let refused = |edit: &dyn Fn(&mut Topology, FaceId)| {
        let (mut topo, traces, section) = plain(&[rect], &[FULL]);
        edit(&mut topo, traces[0].face);
        matches!(
            clip(&topo, traces, &section),
            Err(ClipError::UnsupportedDomain)
        )
    };
    // The same segment parameterized over [1, 1 + length].
    assert!(refused(&|topo, f| {
        let cid = side(topo, f, true);
        let (line, range) = chart_line(topo, cid);
        let d = line.direction();
        set_chart(topo, cid, line.origin() - d, d, [1.0, 1.0 + range[1]]);
    }));
    // Directions one subnormal off the axis: (2^-1074, +-1) and (+-1, 2^-1074).
    // Over these quarter-unit sides the off-axis step rounds to zero.
    for vertical in [true, false] {
        assert!(refused(&|topo, f| {
            let cid = side(topo, f, vertical);
            let (line, range) = chart_line(topo, cid);
            let d = line.direction();
            let tilt = f64::from_bits(1);
            let d = if vertical {
                Vec2::new(tilt, d.y())
            } else {
                Vec2::new(d.x(), tilt)
            };
            set_chart(topo, cid, line.origin(), d, range);
        }));
    }
    // An axis line whose end is an inexact sum: 0.1 + 0.2.
    assert!(refused(&|topo, f| {
        let cid = side(topo, f, false);
        let (line, _) = chart_line(topo, cid);
        set_chart(
            topo,
            cid,
            Point2::new(0.1, line.origin().y()),
            Vec2::new(1.0, 0.0),
            [0.0, 0.2],
        );
    }));
}

#[test]
fn a_chart_corner_one_ulp_off_its_neighbour_is_an_invalid_boundary() {
    // The shifted side still certifies against its 3D edge (the shift is
    // 1e-16); only exact corner connectivity can refuse it.
    let (mut topo, traces, section) = plain(&[[0.125, 0.875, -1.0, 1.0]], &[FULL]);
    let cid = side(&topo, traces[0].face, true);
    let (line, range) = chart_line(&topo, cid);
    let o = line.origin();
    set_chart(
        &mut topo,
        cid,
        Point2::new(o.x().next_up(), o.y()),
        line.direction(),
        range,
    );
    assert!(matches!(
        clip(&topo, traces, &section),
        Err(ClipError::InvalidBoundary)
    ));
}

#[test]
fn a_hole_touching_any_outer_side_is_an_invalid_boundary() {
    assert_intervals(
        &run(&[FULL, [0.25, 0.75, -0.5, 0.5]], &[FULL]).3,
        &[[0.0, 0.25], [0.75, 1.0]],
    );
    for hole in [
        [0.0, 0.5, -0.5, 0.5],
        [0.5, 1.0, -0.5, 0.5],
        [0.25, 0.75, -1.5, 0.5],
        [0.25, 0.75, -0.5, 1.5],
    ] {
        let (topo, traces, section) = plain(&[FULL, hole], &[FULL]);
        assert!(
            matches!(
                clip(&topo, traces, &section),
                Err(ClipError::InvalidBoundary)
            ),
            "{hole:?}"
        );
    }
}

#[test]
fn a_patch_reported_periodic_in_u_refuses_although_regular() {
    // Section points scaled by 1e-8: the first and last control rows lie
    // 1.4e-8 apart, under `is_periodic_u`'s absolute 1e-7, while the rows
    // stay a unit apart in v.
    let k = 1e-8;
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.5 * k, 0.0, 0.0),
        Point3::new(k, 0.0, k),
    ];
    let section =
        NurbsCurve::new(2, knots(2, (-8.0, 24.0)), points.to_vec(), vec![1.0; 3]).unwrap();
    let surfaces: [NurbsSurface; 2] = std::array::from_fn(|j| {
        let d = if j == 0 {
            Vec3::new(0.0, 1.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 1.0)
        };
        NurbsSurface::new(
            2,
            1,
            knots(2, (0.0, 1.0)),
            knots(1, (-2.0, 2.0)),
            points
                .iter()
                .map(|p| vec![*p - d * 0.5, *p + d * 0.5])
                .collect(),
            vec![vec![1.0; 2]; 3],
        )
        .unwrap()
    });
    assert!(surfaces[0].is_periodic_u() && !surfaces[0].is_periodic_v());
    let mut topo = Topology::new();
    let traces = std::array::from_fn(|j| FaceTrace {
        face: face(&mut topo, surfaces[j].clone(), &[FULL], false),
        v: 0.0,
    });
    assert!(matches!(
        clip(&topo, traces, &section),
        Err(ClipError::UnsupportedDomain)
    ));
}

fn edited_curve(c: &NurbsCurve, edit: &dyn Fn(&mut serde_json::Value)) -> NurbsCurve {
    let mut value = serde_json::to_value(c).unwrap();
    edit(&mut value);
    serde_json::from_value(value).unwrap()
}
fn edited_surface(s: &NurbsSurface, edit: &dyn Fn(&mut serde_json::Value)) -> NurbsSurface {
    let mut value = serde_json::to_value(s).unwrap();
    edit(&mut value);
    serde_json::from_value(value).unwrap()
}

#[test]
fn unvalidated_section_weights_are_invalid_input() {
    // The validating constructor refuses these; a deserialized curve can
    // still carry them.
    let (topo, traces, section) = plain(&[FULL], &[FULL]);
    let edits: [&dyn Fn(&mut serde_json::Value); 3] = [
        &|v| v["weights"][1] = serde_json::json!(0.0),
        &|v| v["weights"][1] = serde_json::json!(-1.0),
        &|v| {
            v["weights"].as_array_mut().unwrap().pop();
        },
    ];
    for edit in edits {
        let bad = edited_curve(&section, edit);
        assert!(matches!(
            clip(&topo, traces, &bad),
            Err(ClipError::InvalidInput)
        ));
    }
}

#[test]
fn unvalidated_patch_grids_are_invalid_input() {
    let edits: [&dyn Fn(&mut serde_json::Value); 3] = [
        // A zero-weight row.
        &|v| v["weights"][1] = serde_json::json!([0.0, 0.0]),
        // Weight rows of three beside control rows of two.
        &|v| {
            for row in v["weights"].as_array_mut().unwrap() {
                row.as_array_mut().unwrap().push(serde_json::json!(1.0));
            }
        },
        // Control rows of three beside weight rows of two.
        &|v| {
            for row in v["control_points"].as_array_mut().unwrap() {
                let last = row[1].clone();
                row.as_array_mut().unwrap().push(last);
            }
        },
    ];
    for edit in edits {
        let (mut topo, traces, section) = plain(&[FULL], &[FULL]);
        let FaceSurface::Nurbs(s) = topo.face(traces[0].face).unwrap().surface().clone() else {
            panic!()
        };
        let bad = edited_surface(&s, edit);
        topo.face_mut(traces[0].face)
            .unwrap()
            .set_surface(FaceSurface::Nurbs(bad));
        assert!(matches!(
            clip(&topo, traces, &section),
            Err(ClipError::InvalidInput)
        ));
    }
}

#[test]
fn interval_division_follows_the_divisor_sign() {
    let encloses = |x: bernstein::I, lo: f64, hi: f64| {
        x.lo <= lo && x.hi >= hi && x.lo >= lo - 1e-15 && x.hi <= hi + 1e-15
    };
    let one = bernstein::I::exact(1.0);
    // 1 / [-2, -1] = [-1, -0.5]; 1 / [1, 2] = [0.5, 1].
    assert!(encloses(
        one.div(bernstein::I { lo: -2.0, hi: -1.0 }),
        -1.0,
        -0.5
    ));
    assert!(encloses(
        one.div(bernstein::I { lo: 1.0, hi: 2.0 }),
        0.5,
        1.0
    ));
    // A divisor touching or straddling zero has no finite quotient.
    for (lo, hi) in [(-1.0, 1.0), (0.0, 1.0), (-1.0, 0.0)] {
        let q = one.div(bernstein::I { lo, hi });
        assert!(q.lo == f64::NEG_INFINITY && q.hi == f64::INFINITY);
    }
}

fn encloses_all(p: &[bernstein::I], want: &[f64]) -> bool {
    p.len() == want.len()
        && p.iter()
            .zip(want)
            .all(|(x, w)| x.lo <= *w && x.hi >= *w && x.hi - x.lo < 1e-12)
}

#[test]
fn bernstein_product_uses_binomial_degree_elevation() {
    let c = |v: &[f64]| {
        v.iter()
            .map(|x| bernstein::I::exact(*x))
            .collect::<Vec<_>>()
    };
    // 1 * 1 = 1 at every elevated degree.
    assert!(encloses_all(
        &bernstein::product(&c(&[1.0; 2]), &c(&[1.0; 2])),
        &[1.0; 3]
    ));
    assert!(encloses_all(
        &bernstein::product(&c(&[1.0; 3]), &c(&[1.0; 2])),
        &[1.0; 4]
    ));
    // (1 - t) * t = [0, 1/2, 0]; t * t = [0, 0, 1].
    assert!(encloses_all(
        &bernstein::product(&c(&[1.0, 0.0]), &c(&[0.0, 1.0])),
        &[0.0, 0.5, 0.0]
    ));
    assert!(encloses_all(
        &bernstein::product(&c(&[0.0, 1.0]), &c(&[0.0, 1.0])),
        &[0.0, 0.0, 1.0]
    ));
}

#[test]
fn ruled_normals_are_the_hodograph_cross_the_ruling() {
    let origin = Point3::new(0.0, 0.0, 0.0);
    let ruling: bernstein::V = [0.0, 3.0, 0.0].map(|x| vec![bernstein::I::exact(x)]);
    // Line 0 -> (2,0,0): x' = 2, so the normal is (2,0,0) x (0,3,0) = (0,0,6)
    // at both degree-1 coefficients.
    let h = bernstein::homogeneous(&[origin, Point3::new(2.0, 0.0, 0.0)], &[1.0; 2], origin);
    let n = bernstein::normals(&h, &ruling);
    assert!(encloses_all(&n[0], &[0.0; 2]));
    assert!(encloses_all(&n[1], &[0.0; 2]));
    assert!(encloses_all(&n[2], &[6.0; 2]));
    // x = 2t^2 (controls 0, 0, 2): x' = 4t, elevated against the unit weight
    // to degree 3 as [0, 4/3, 8/3, 4]; times the ruling's 3: [0, 4, 8, 12].
    let h = bernstein::homogeneous(
        &[origin, origin, Point3::new(2.0, 0.0, 0.0)],
        &[1.0; 3],
        origin,
    );
    let n = bernstein::normals(&h, &ruling);
    assert!(encloses_all(&n[2], &[0.0, 4.0, 8.0, 12.0]));
}

#[test]
fn residual_has_no_certificate_when_a_weight_enclosure_reaches_zero() {
    let point = |w: f64| -> bernstein::H {
        [
            vec![bernstein::I::exact(0.0)],
            vec![bernstein::I::exact(0.0)],
            vec![bernstein::I::exact(0.0)],
            vec![bernstein::I::exact(w)],
        ]
    };
    assert!(bernstein::residual(&point(1.0), &point(1.0)) < 1e-150);
    for w in [0.0, -1.0] {
        assert_eq!(bernstein::residual(&point(1.0), &point(w)), f64::INFINITY);
    }
}
