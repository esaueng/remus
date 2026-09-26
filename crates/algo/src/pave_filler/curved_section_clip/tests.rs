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
    let mut wires = Vec::new();
    let mut pcurves = Vec::new();
    let domain = s.domain_u();
    for (r, rect) in rects.iter().enumerate() {
        let [x0, x1, y0, y1] = *rect;
        let u = |x| domain.0 + (domain.1 - domain.0) * x;
        let mut points = [
            Point2::new(u(x0), y0),
            Point2::new(u(x1), y0),
            Point2::new(u(x1), y1),
            Point2::new(u(x0), y1),
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
            pcurves.push((
                id,
                !rev,
                PCurve::new(
                    Curve2D::Line(Line2D::new(a, b - a).unwrap()),
                    0.0,
                    (b - a).length(),
                ),
            ));
        }
        wires.push(topo.add_wire(Wire::new(edges, true).unwrap()));
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
