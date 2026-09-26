#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp
)]
use super::*;
use proptest::prelude::*;
use remus_math::context::{CancellationToken, WorkBudgets};
use remus_math::curves::Circle3D;
use remus_math::curves2d::{Circle2D, Line2D};
use remus_math::vec::Vec3;
use std::f64::consts::{PI, TAU};

fn context() -> OperationContext {
    OperationContext::new().with_budgets(
        WorkBudgets::new()
            .with_march_steps(2_000_000)
            .with_queue_size(20_000)
            .with_segments(10_000),
    )
}
fn source(id: u64) -> CurveSource {
    CurveSource {
        use_id: id,
        boundary: None,
        section: Some(id as usize),
        source_edge_idx: Some(id as usize),
        pave_block_id: Some(id as usize + 1000),
    }
}
fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}
fn xyz(p: Point2) -> Point3 {
    Point3::new(p.x(), p.y(), 0.0)
}
fn line(id: u64, a: Point2, b: Point2, endpoints: [u64; 2], boundary: Option<u64>) -> CurveUse {
    CurveUse {
        source: source(id),
        pcurve: Curve2D::Line(Line2D::new(a, b - a).unwrap()),
        range: [0.0, (b - a).length()],
        curve_3d: EdgeCurve::Line,
        source_range: [0.0, 1.0],
        endpoints_3d: [xyz(a), xyz(b)],
        endpoints,
        boundary_loop: boundary,
    }
}
fn rectangle(id: u64, x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<CurveUse> {
    let points = [p(x0, y0), p(x1, y0), p(x1, y1), p(x0, y1)];
    (0..4)
        .map(|i| {
            line(
                id + i as u64,
                points[i],
                points[(i + 1) % 4],
                [id + i as u64, id + ((i + 1) % 4) as u64],
                Some(id),
            )
        })
        .collect()
}
fn circle(id: u64, center: Point2, radius: f64, boundary: Option<u64>) -> CurveUse {
    let c = Circle3D::new_with_ref(
        xyz(center),
        Vec3::new(0.0, 0.0, 1.0),
        radius,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let ends = [c.evaluate(0.0), c.evaluate(TAU)];
    CurveUse {
        source: source(id),
        pcurve: Curve2D::Circle(Circle2D::new(center, radius).unwrap()),
        range: [0.0, TAU],
        curve_3d: EdgeCurve::Circle(c),
        source_range: [0.0, TAU],
        endpoints_3d: ends,
        endpoints: [id, id],
        boundary_loop: boundary,
    }
}
fn build(uses: &[CurveUse]) -> Arrangement {
    build_arrangement(&ArrangementInput {
        uses,
        domain: ParamDomain::Plane,
        context: &context(),
    })
    .unwrap()
}
fn near(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 1e-9 * (1.0 + expected.abs()),
        "{actual} != {expected}"
    );
}
fn area(a: &Arrangement) -> f64 {
    a.regions
        .iter()
        .filter(|r| r.material)
        .map(|r| r.area)
        .sum()
}
fn material_count(a: &Arrangement) -> usize {
    a.regions.iter().filter(|r| r.material).count()
}

fn invariants(a: &Arrangement, uses: &[CurveUse]) {
    let lookup: BTreeMap<_, _> = uses.iter().map(|u| (u.source.use_id, u)).collect();
    let mut incoming = vec![0; a.half_edges.len()];
    let mut visits = vec![0; a.half_edges.len()];
    for (id, e) in a.half_edges.iter().enumerate() {
        let twin = &a.half_edges[e.twin];
        assert_eq!(twin.twin, id);
        assert_eq!(e.from, twin.to);
        assert_eq!(e.to, twin.from);
        assert_eq!(e.to, a.half_edges[e.next].from);
        incoming[e.next] += 1;
        assert_eq!(e.source, twin.source);
        assert_eq!(e.range, [twin.range[1], twin.range[0]]);
        assert_eq!(e.source_range, [twin.source_range[1], twin.source_range[0]]);
        let u = lookup[&e.source.use_id];
        assert!(
            e.range
                .iter()
                .all(|&t| t >= u.range[0].min(u.range[1]) && t <= u.range[0].max(u.range[1]))
        );
        near((u.point(e.range[0]) - a.vertices[e.from].uv).length(), 0.0);
        near((u.point_3d(e.range[0]) - e.endpoints_3d[0]).length(), 0.0);
    }
    let mut components: Vec<std::collections::BTreeSet<usize>> = (0..a.vertices.len())
        .map(|v| std::iter::once(v).collect())
        .collect();
    for e in a.half_edges.iter().step_by(2) {
        let from = components.iter().position(|c| c.contains(&e.from)).unwrap();
        let to = components.iter().position(|c| c.contains(&e.to)).unwrap();
        if from != to {
            let other = components.remove(from.max(to));
            components[from.min(to)].extend(other);
        }
    }
    assert_eq!(
        a.vertices.len() + a.regions.len() + 1,
        a.half_edges.len() / 2 + components.len() + 1
    );
    for e in &a.half_edges {
        let original = a
            .sources
            .iter()
            .find(|s| s.source.use_id == e.source.use_id)
            .unwrap();
        assert_eq!(original.source, e.source);
        assert!(matches!(
            original.curve_3d,
            EdgeCurve::Line | EdgeCurve::Circle(_)
        ));
    }
    assert!(incoming.iter().all(|&n| n == 1));
    for c in &a.cycles {
        for (i, &h) in c.edges.iter().enumerate() {
            visits[h] += 1;
            assert_eq!(a.half_edges[h].next, c.edges[(i + 1) % c.edges.len()]);
        }
    }
    assert!(visits.iter().all(|&n| n == 1));
    for r in &a.regions {
        assert!(a.cycles[r.outer].signed_area > 0.0);
        assert!(r.holes.iter().all(|&h| a.cycles[h].signed_area < 0.0));
    }
    assert!(a.exterior.iter().all(|&c| a.cycles[c].signed_area < 0.0));
}

/// Independent oracle: sample the exact curves densely and use a polygon ray
/// test. This is only a test oracle; no sampled point contributes runtime IDs.
fn sampled_contains(
    a: &Arrangement,
    r: &ArrangementRegion,
    point: Point2,
    uses: &[CurveUse],
) -> bool {
    let contains = |cycle: usize| {
        let mut polygon = Vec::new();
        for &h in &a.cycles[cycle].edges {
            let e = &a.half_edges[h];
            let u = uses
                .iter()
                .find(|u| u.source.use_id == e.source.use_id)
                .unwrap();
            for i in 0..100 {
                polygon
                    .push(u.point(e.range[0] + (e.range[1] - e.range[0]) * f64::from(i) / 100.0));
            }
        }
        remus_math::predicates::point_in_polygon(point, &polygon)
    };
    contains(r.outer) && !r.holes.iter().any(|&h| contains(h))
}
fn check_probes(a: &Arrangement, uses: &[CurveUse], oracle: impl Fn(Point2) -> bool) {
    for x in -13..=13 {
        for y in -13..=13 {
            let point = p(f64::from(x) * 0.31 + 0.037, f64::from(y) * 0.29 + 0.061);
            let count = a
                .regions
                .iter()
                .filter(|r| r.material && sampled_contains(a, r, point, uses))
                .count();
            assert!(count <= 1, "overlapping material at {point:?}");
            assert_eq!(count == 1, oracle(point), "material at {point:?}");
        }
    }
}

#[test]
fn x_t_and_star_have_independent_area_and_material() {
    for arms in [3, 4, 6, 8] {
        let mut uses = rectangle(0, -2.0, -2.0, 2.0, 2.0);
        let ends = [
            p(-2.0, 0.0),
            p(2.0, 0.0),
            p(0.0, 2.0),
            p(0.0, -2.0),
            p(-2.0, -2.0),
            p(2.0, 2.0),
            p(-2.0, 2.0),
            p(2.0, -2.0),
        ];
        for (i, &end) in ends.iter().take(arms).enumerate() {
            let endpoint = match i {
                4 => 0,
                5 => 2,
                6 => 3,
                7 => 1,
                _ => 100 + i as u64,
            };
            uses.push(line(10 + i as u64, p(0.0, 0.0), end, [999, endpoint], None));
        }
        let a = build(&uses);
        assert_eq!(material_count(&a), arms);
        near(area(&a), 16.0);
        invariants(&a, &uses);
        check_probes(&a, &uses, |p| p.x().abs() < 2.0 && p.y().abs() < 2.0);
    }
}

#[test]
fn true_interior_crossings_create_one_event_vertex() {
    let mut uses = rectangle(0, -2.0, -2.0, 2.0, 2.0);
    uses.push(line(10, p(-2.0, 0.0), p(2.0, 0.0), [10, 11], None));
    uses.push(line(11, p(0.0, -2.0), p(0.0, 2.0), [12, 13], None));
    uses.push(line(12, p(-2.0, -2.0), p(2.0, 2.0), [0, 2], None));
    let a = build(&uses);
    assert_eq!(material_count(&a), 6);
    near(area(&a), 16.0);
    assert_eq!(
        a.vertices
            .iter()
            .filter(|v| v.incidences.len() == 3 && v.uv.x().abs() < 1e-10 && v.uv.y().abs() < 1e-10)
            .count(),
        1
    );
    invariants(&a, &uses);
}

#[test]
fn circle_cut_preserves_major_minor_source_arcs_and_area() {
    let mut uses = vec![circle(0, p(0.0, 0.0), 2.0, Some(0))];
    uses.push(line(10, p(-3.0, 1.0), p(3.0, 1.0), [10, 11], None));
    // A section must terminate on the boundary; retain its native support and
    // source parameterization, trimming it at the analytic intersections.
    let root = 3.0_f64.sqrt();
    uses[1].range = [3.0 - root, 3.0 + root];
    uses[1].source_range = [(3.0 - root) / 6.0, (3.0 + root) / 6.0];
    uses[1].endpoints_3d = [xyz(p(-root, 1.0)), xyz(p(root, 1.0))];
    // Endpoint-only 3D line representation uses the trimmed endpoint domain.
    uses[1].source_range = [0.0, 1.0];
    let a = build(&uses);
    assert_eq!(material_count(&a), 2);
    near(area(&a), 4.0 * PI);
    let cap = 4.0 * PI / 3.0 - root;
    let mut areas: Vec<_> = a.regions.iter().map(|r| r.area).collect();
    areas.sort_by(f64::total_cmp);
    near(areas[0], cap);
    near(areas[1], 4.0 * PI - cap);
    invariants(&a, &uses);
    let circle_edges: Vec<_> = a
        .half_edges
        .iter()
        .step_by(2)
        .filter(|e| e.source.use_id == 0)
        .collect();
    near(
        circle_edges
            .iter()
            .map(|e| e.source_range[1] - e.source_range[0])
            .sum(),
        TAU,
    );
    assert!(
        circle_edges
            .iter()
            .any(|e| (e.range[1] - PI / 6.0).abs() < 1e-12)
    );
    check_probes(&a, &uses, |p| p.x() * p.x() + p.y() * p.y() < 4.0);
}

#[test]
fn overlapping_circles_form_analytic_lens_without_merging_coendpoints() {
    let uses = vec![
        circle(0, p(-0.5, 0.0), 1.0, Some(0)),
        circle(1, p(0.5, 0.0), 1.0, None),
    ];
    let a = build(&uses);
    let lens = 2.0 * PI / 3.0 - 3.0_f64.sqrt() / 2.0;
    assert_eq!(a.regions.len(), 3);
    near(area(&a), PI);
    assert!(a.regions.iter().any(|r| (r.area - lens).abs() < 1e-10));
    invariants(&a, &uses);
}

#[test]
fn nested_holes_islands_and_disconnected_material_have_unique_owners() {
    let mut uses = vec![
        circle(0, p(0.0, 0.0), 3.0, Some(0)),
        circle(1, p(0.0, 0.0), 2.0, Some(1)),
        circle(2, p(0.0, 0.0), 1.0, Some(2)),
    ];
    uses.extend(rectangle(10, 4.0, 0.0, 5.0, 1.0));
    let a = build(&uses);
    near(area(&a), 6.0 * PI + 1.0);
    assert_eq!(material_count(&a), 3);
    assert_eq!(a.regions.iter().filter(|r| r.holes.len() == 1).count(), 2);
    assert_eq!(a.exterior.len(), 2);
    invariants(&a, &uses);
    check_probes(&a, &uses, |p| {
        let r2 = p.x() * p.x() + p.y() * p.y();
        r2 < 1.0
            || (r2 > 4.0 && r2 < 9.0)
            || (p.x() > 4.0 && p.x() < 5.0 && p.y() > 0.0 && p.y() < 1.0)
    });
}

fn reversed(mut u: CurveUse) -> CurveUse {
    u.range.swap(0, 1);
    u.source_range.swap(0, 1);
    u.endpoints.swap(0, 1);
    u.endpoints_3d.swap(0, 1);
    u
}
#[test]
fn all_permutations_and_equivalent_reversal_are_byte_identical() {
    let uses = rectangle(0, -1.0, -1.0, 1.0, 1.0);
    let expected = format!("{:?}", build(&uses));
    for a in 0..4 {
        for b in 0..4 {
            for c in 0..4 {
                for d in 0..4 {
                    let order = [a, b, c, d];
                    let mut check = order;
                    check.sort_unstable();
                    if check != [0, 1, 2, 3] {
                        continue;
                    }
                    for mask in 0..16 {
                        let variant: Vec<_> = order
                            .iter()
                            .enumerate()
                            .map(|(i, &j)| {
                                if mask & (1 << i) != 0 {
                                    reversed(uses[j].clone())
                                } else {
                                    uses[j].clone()
                                }
                            })
                            .collect();
                        assert_eq!(format!("{:?}", build(&variant)), expected);
                    }
                }
            }
        }
    }
}

fn cylinder_uses(rulings: &[f64], bands: &[f64]) -> Vec<CurveUse> {
    let mut uses = rectangle(0, 0.0, 0.0, TAU, 3.0);
    for (i, &u) in rulings.iter().enumerate() {
        uses.push(line(
            10 + i as u64,
            p(u, 0.0),
            p(u, 3.0),
            [100 + i as u64 * 2, 101 + i as u64 * 2],
            None,
        ));
    }
    for (i, &v) in bands.iter().enumerate() {
        uses.push(line(
            30 + i as u64,
            p(0.0, v),
            p(TAU, v),
            [300 + i as u64 * 2, 301 + i as u64 * 2],
            None,
        ));
    }
    for u in &mut uses {
        let a = u.point(u.range[0]);
        let b = u.point(u.range[1]);
        u.endpoints_3d = [
            Point3::new(a.x().cos() * 2.0, a.x().sin() * 2.0, a.y()),
            Point3::new(b.x().cos() * 2.0, b.x().sin() * 2.0, b.y()),
        ];
        if (a.y() - b.y()).abs() < 1e-12 {
            u.curve_3d = EdgeCurve::Circle(
                Circle3D::new_with_ref(
                    Point3::new(0.0, 0.0, a.y()),
                    Vec3::new(0.0, 0.0, 1.0),
                    2.0,
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .unwrap(),
            );
            u.source_range = [a.x(), b.x()];
        }
    }
    uses
}
fn cylinder(uses: &[CurveUse]) -> Result<Arrangement> {
    build_arrangement(&ArrangementInput {
        uses,
        domain: ParamDomain::CylinderStrip {
            seam_uses: [3, 1],
            radius: 2.0,
        },
        context: &context(),
    })
}
#[test]
fn cylinder_bands_retain_winding_area_and_quotient_euler() {
    let uses = cylinder_uses(&[], &[1.0, 2.0]);
    let a = cylinder(&uses).unwrap();
    assert_eq!(a.periodic_regions.len(), 3);
    invariants(&a, &uses);
    for r in &a.periodic_regions {
        assert_eq!(r.euler_characteristic, 0);
        near(r.area, 4.0 * PI);
        let mut winding: Vec<_> = r.boundaries.iter().map(|b| b.1).collect();
        winding.sort_unstable();
        assert_eq!(winding, [-1, 1]);
    }
}
#[test]
fn cylinder_sectors_join_across_seam_and_are_permutation_invariant() {
    let uses = cylinder_uses(&[1.0, 3.0], &[]);
    let a = cylinder(&uses).unwrap();
    assert_eq!(a.periodic_regions.len(), 2);
    assert!(a.periodic_regions.iter().any(|r| r.cells.len() == 2));
    for r in &a.periodic_regions {
        assert_eq!(r.euler_characteristic, 1);
        assert_eq!(r.boundaries.len(), 1);
        assert_eq!(r.boundaries[0].1, 0);
    }
    near(a.periodic_regions.iter().map(|r| r.area).sum(), 12.0 * PI);
    let variant: Vec<_> = uses.into_iter().rev().map(reversed).collect();
    assert_eq!(
        format!("{a:?}"),
        format!("{:?}", cylinder(&variant).unwrap())
    );
}

#[test]
fn tangent_overlaps_slits_and_bad_certificates_refuse() {
    let mut uses = vec![
        circle(0, p(0.0, 0.0), 1.0, Some(0)),
        circle(1, p(2.0, 0.0), 1.0, None),
    ];
    assert_eq!(
        build_arrangement(&ArrangementInput {
            uses: &uses,
            domain: ParamDomain::Plane,
            context: &context()
        })
        .unwrap_err(),
        ArrangementError::AmbiguousContact
    );
    uses[1] = circle(1, p(0.0, 0.0), 1.0, None);
    assert_eq!(
        build_arrangement(&ArrangementInput {
            uses: &uses,
            domain: ParamDomain::Plane,
            context: &context()
        })
        .unwrap_err(),
        ArrangementError::AmbiguousOverlap
    );
    let mut uses = rectangle(0, 0.0, 0.0, 2.0, 2.0);
    uses.push(line(10, p(0.5, 0.0), p(1.5, 0.0), [10, 11], None));
    assert_eq!(
        build_arrangement(&ArrangementInput {
            uses: &uses,
            domain: ParamDomain::Plane,
            context: &context()
        })
        .unwrap_err(),
        ArrangementError::AmbiguousOverlap
    );
    uses.pop();
    uses.push(line(10, p(1.0, 0.0), p(1.0, 1.0), [10, 11], None));
    assert_eq!(
        build_arrangement(&ArrangementInput {
            uses: &uses,
            domain: ParamDomain::Plane,
            context: &context()
        })
        .unwrap_err(),
        ArrangementError::OpenRegion
    );
    uses.pop();
    uses[0].endpoints[1] = 999;
    assert_eq!(
        build_arrangement(&ArrangementInput {
            uses: &uses,
            domain: ParamDomain::Plane,
            context: &context()
        })
        .unwrap_err(),
        ArrangementError::InvalidBoundary
    );
}

#[test]
fn all_budget_failure_prefixes_and_cancellation_are_atomic() {
    let uses = rectangle(0, 0.0, 0.0, 2.0, 2.0);
    let before = format!("{uses:?}");
    let expected = build(&uses);
    let mut succeeded = false;
    for steps in 0..3000 {
        let ctx = context().with_budgets(context().budgets.with_march_steps(steps));
        match build_arrangement(&ArrangementInput {
            uses: &uses,
            domain: ParamDomain::Plane,
            context: &ctx,
        }) {
            Ok(a) => {
                assert_eq!(format!("{a:?}"), format!("{expected:?}"));
                succeeded = true;
                break;
            }
            Err(e) => assert_eq!(e, ArrangementError::WorkBudgetExceeded),
        }
    }
    assert!(succeeded);
    assert_eq!(format!("{uses:?}"), before);
    let token = CancellationToken::new();
    token.cancel();
    let ctx = context().with_cancellation(token);
    assert_eq!(
        build_arrangement(&ArrangementInput {
            uses: &uses,
            domain: ParamDomain::Plane,
            context: &ctx
        })
        .unwrap_err(),
        ArrangementError::Cancelled
    );
    for budgets in [
        context().budgets.with_queue_size(0),
        context().budgets.with_segments(0),
    ] {
        let ctx = context().with_budgets(budgets);
        assert_eq!(
            build_arrangement(&ArrangementInput {
                uses: &uses,
                domain: ParamDomain::Plane,
                context: &ctx
            })
            .unwrap_err(),
            ArrangementError::WorkBudgetExceeded
        );
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]
    #[test]
    fn generated_grid_arrangements_have_exact_material_and_canonical_output(xs in prop::collection::btree_set(1i32..30,0..7),ys in prop::collection::btree_set(1i32..30,0..7),shift in -100i32..100){
        let o=f64::from(shift);let mut uses=rectangle(0,o,o,o+32.0,o+32.0);
        for (i,x) in xs.iter().enumerate(){uses.push(line(10+i as u64,p(o+f64::from(*x),o),p(o+f64::from(*x),o+32.0),[100+i as u64*2,101+i as u64*2],None));}
        for (i,y) in ys.iter().enumerate(){uses.push(line(30+i as u64,p(o,o+f64::from(*y)),p(o+32.0,o+f64::from(*y)),[300+i as u64*2,301+i as u64*2],None));}
        let a=build(&uses);prop_assert_eq!(material_count(&a),(xs.len()+1)*(ys.len()+1));near(area(&a),1024.0);invariants(&a,&uses);
        uses.reverse();for u in &mut uses{*u=reversed(u.clone());}prop_assert_eq!(format!("{a:?}"),format!("{:?}",build(&uses)));
    }
}

#[test]
fn authoritative_coedge_provenance_survives_every_subspan() {
    let mut topo = remus_topology::Topology::new();
    let face = remus_topology::builder::make_rectangle_face(&mut topo, 4.0, 4.0, 1e-7).unwrap();
    let loop_id = topo.loops_of_face(face).unwrap()[0];
    let coedges = topo.face_loop(loop_id).unwrap().coedges();
    let mut uses = rectangle(0, -2.0, -2.0, 2.0, 2.0);
    for (u, &coedge) in uses.iter_mut().zip(coedges) {
        u.source.boundary = Some(BoundarySource {
            face,
            boundary_loop: loop_id,
            coedge,
        });
        u.source.section = None;
    }
    uses.push(line(10, p(-2.0, 0.0), p(2.0, 0.0), [10, 11], None));
    let a = build(&uses);
    for edge in &a.half_edges {
        let original = uses
            .iter()
            .find(|u| u.source.use_id == edge.source.use_id)
            .unwrap();
        assert_eq!(edge.source, original.source);
        if let Some(boundary) = &edge.source.boundary {
            assert_eq!(topo.coedge(boundary.coedge).unwrap().parent_loop(), loop_id);
        }
    }
    assert_eq!(
        a.half_edges.iter().filter(|e| e.source.use_id == 1).count(),
        4
    );
}

#[test]
fn sub_tolerance_sliver_is_not_snapped_away() {
    let mut uses = rectangle(0, 0.0, 0.0, 1.0, 1.0);
    for (id, x) in [(10, 0.5), (11, 0.5 + 1e-8)] {
        uses.push(line(id, p(x, 0.0), p(x, 1.0), [id * 2, id * 2 + 1], None));
    }
    let a = build(&uses);
    assert_eq!(material_count(&a), 3);
    let minimum = a
        .regions
        .iter()
        .map(|r| r.area)
        .fold(f64::INFINITY, f64::min);
    assert!((minimum - 1e-8).abs() < 1e-15);
    near(area(&a), 1.0);
}

#[test]
fn malformed_nonfinite_and_inconsistent_spatial_events_refuse() {
    let mut uses = rectangle(0, 0.0, 0.0, 2.0, 2.0);
    uses[0].range[0] = f64::NAN;
    assert_eq!(
        build_arrangement(&ArrangementInput {
            uses: &uses,
            domain: ParamDomain::Plane,
            context: &context()
        })
        .unwrap_err(),
        ArrangementError::NonFiniteInput
    );
    let mut uses = rectangle(0, 0.0, 0.0, 2.0, 2.0);
    uses.push(line(10, p(0.0, 1.0), p(2.0, 1.0), [10, 11], None));
    uses[4].endpoints_3d = [Point3::new(0.0, 1.0, 1.0), Point3::new(2.0, 1.0, 1.0)];
    assert_eq!(
        build_arrangement(&ArrangementInput {
            uses: &uses,
            domain: ParamDomain::Plane,
            context: &context()
        })
        .unwrap_err(),
        ArrangementError::IntersectionRefinementFailed
    );
    let mut uses = rectangle(0, 0.0, 0.0, 2.0, 2.0);
    let mut other = rectangle(10, 3.0, 0.0, 4.0, 1.0);
    for u in &mut other {
        u.boundary_loop = Some(0);
    }
    uses.extend(other);
    assert_eq!(
        build_arrangement(&ArrangementInput {
            uses: &uses,
            domain: ParamDomain::Plane,
            context: &context()
        })
        .unwrap_err(),
        ArrangementError::InvalidBoundary
    );
}

#[test]
fn cancellation_after_work_has_started_stops_the_next_step() {
    let token = CancellationToken::new();
    let ctx = context().with_cancellation(token.clone());
    let mut work = Work::new(&ctx);
    for _ in 0..17 {
        work.step().unwrap();
    }
    token.cancel();
    assert_eq!(work.step().unwrap_err(), ArrangementError::Cancelled);
}

#[test]
fn tangent_line_and_partial_arc_overlap_refuse_but_separated_supports_do_not() {
    let uses = vec![
        circle(0, p(0.0, 0.0), 1.0, Some(0)),
        line(1, p(-2.0, 1.0), p(2.0, 1.0), [1, 2], None),
    ];
    assert_eq!(
        build_arrangement(&ArrangementInput {
            uses: &uses,
            domain: ParamDomain::Plane,
            context: &context()
        })
        .unwrap_err(),
        ArrangementError::AmbiguousContact
    );
    let uses = vec![
        circle(0, p(0.0, 0.0), 1.0, Some(0)),
        circle(1, p(2.01, 0.0), 1.0, Some(1)),
    ];
    let a = build(&uses);
    near(area(&a), 2.0 * PI);
    assert_eq!(a.regions.len(), 2);
    let a = circle(0, p(0.0, 0.0), 1.0, None);
    let mut b = circle(1, p(0.0, 0.0), 1.0, None);
    b.range = [0.5, PI];
    assert_eq!(
        geometry::intersections(&a, &b, &mut Work::new(&context())).unwrap_err(),
        ArrangementError::AmbiguousOverlap
    );
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(48))]
    #[test]
    fn generated_arc_lenses_keep_analytic_area_and_orientation(distance in 0.2f64..1.8, angle in 0.01f64..6.2){
        let mut uses=vec![circle(0,p(0.0,0.0),1.0,Some(0)),circle(1,p(distance*angle.cos(),distance*angle.sin()),1.0,None)];
        let a=build(&uses);invariants(&a,&uses);near(area(&a),PI);
        let lens=2.0*(distance/2.0).acos()-distance/2.0*(4.0-distance*distance).sqrt();
        prop_assert!(a.regions.iter().any(|r|(r.area-lens).abs()<1e-9));
        uses.reverse();for u in &mut uses{*u=reversed(u.clone());}prop_assert_eq!(format!("{a:?}"),format!("{:?}",build(&uses)));
    }
}

#[test]
fn cylinder_with_hole_has_annulus_minus_disc_euler_and_material_area() {
    let mut uses = cylinder_uses(&[], &[]);
    let mut hole = rectangle(50, 1.0, 1.0, 2.0, 2.0);
    for u in &mut hole {
        let a = u.point(u.range[0]);
        let b = u.point(u.range[1]);
        u.endpoints_3d = [
            Point3::new(a.x().cos() * 2.0, a.x().sin() * 2.0, a.y()),
            Point3::new(b.x().cos() * 2.0, b.x().sin() * 2.0, b.y()),
        ];
        if (a.y() - b.y()).abs() < 1e-12 {
            u.curve_3d = EdgeCurve::Circle(
                Circle3D::new_with_ref(
                    Point3::new(0.0, 0.0, a.y()),
                    Vec3::new(0.0, 0.0, 1.0),
                    2.0,
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .unwrap(),
            );
            u.source_range = [a.x(), b.x()];
        }
    }
    uses.extend(hole);
    let a = cylinder(&uses).unwrap();
    assert_eq!(a.periodic_regions.len(), 1);
    let r = &a.periodic_regions[0];
    assert_eq!(r.euler_characteristic, -1);
    assert_eq!(r.boundaries.len(), 3);
    near(r.area, 12.0 * PI - 2.0);
    let mut winding: Vec<_> = r.boundaries.iter().map(|b| b.1).collect();
    winding.sort_unstable();
    assert_eq!(winding, [-1, 0, 1]);
    // A separate rectangle predicate checks kept material in the lifted chart.
    for x in 0..25 {
        for y in 0..12 {
            let point = p(
                (f64::from(x) + 0.31) * TAU / 25.0,
                (f64::from(y) + 0.37) / 4.0,
            );
            let oracle =
                !(point.x() > 1.0 && point.x() < 2.0 && point.y() > 1.0 && point.y() < 2.0);
            assert_eq!(
                a.regions
                    .iter()
                    .any(|r| r.material && sampled_contains(&a, r, point, &uses)),
                oracle
            );
        }
    }
}

#[test]
fn periodic_geometry_and_non_native_circle_branches_refuse_explicitly() {
    let uses = cylinder_uses(&[], &[]);
    for domain in [
        ParamDomain::CylinderStrip {
            seam_uses: [1, 3],
            radius: 2.0,
        },
        ParamDomain::CylinderStrip {
            seam_uses: [3, 1],
            radius: f64::NAN,
        },
    ] {
        assert_eq!(
            build_arrangement(&ArrangementInput {
                uses: &uses,
                domain,
                context: &context()
            })
            .unwrap_err(),
            ArrangementError::UnsupportedDomain
        );
    }
    let mut uses = vec![circle(0, p(0.0, 0.0), 1.0, Some(0))];
    uses[0].range = [-PI, PI];
    assert_eq!(
        build_arrangement(&ArrangementInput {
            uses: &uses,
            domain: ParamDomain::Plane,
            context: &context()
        })
        .unwrap_err(),
        ArrangementError::UnsupportedCurve
    );
}

#[test]
fn near_tangent_both_sides_refuse_without_relaxing_tolerance() {
    for shift in [-f64::EPSILON, f64::EPSILON] {
        let uses = vec![
            circle(0, p(0.0, 0.0), 1.0, Some(0)),
            circle(1, p(2.0 + shift, 0.0), 1.0, None),
        ];
        assert_eq!(
            build_arrangement(&ArrangementInput {
                uses: &uses,
                domain: ParamDomain::Plane,
                context: &context()
            })
            .unwrap_err(),
            ArrangementError::AmbiguousContact
        );
    }
}

#[test]
fn native_line_subspans_are_not_rebased_in_provenance() {
    let mut uses = rectangle(0, 0.0, 0.0, 2.0, 2.0);
    for u in &mut uses {
        u.source_range = [10.0, 20.0];
    }
    uses.push(line(10, p(1.0, 0.0), p(1.0, 2.0), [10, 11], None));
    let a = build(&uses);
    invariants(&a, &uses);
    near(area(&a), 4.0);
    let spans: Vec<_> = a
        .half_edges
        .iter()
        .step_by(2)
        .filter(|e| e.source.use_id == 0)
        .map(|e| e.source_range)
        .collect();
    assert_eq!(spans, [[10.0, 15.0], [15.0, 20.0]]);
}

#[test]
fn distinct_crossings_rounding_to_one_parameter_refuse_instead_of_welding() {
    let mut uses = rectangle(0, -1.0, -1.0, 1.0, 1.0);
    for (id, x) in [(10, 0.0), (11, 1e-17)] {
        uses.push(line(id, p(x, -1.0), p(x, 1.0), [id * 2, id * 2 + 1], None));
    }
    uses.push(line(12, p(-1.0, 0.0), p(1.0, 0.0), [30, 31], None));
    assert_eq!(
        build_arrangement(&ArrangementInput {
            uses: &uses,
            domain: ParamDomain::Plane,
            context: &context()
        })
        .unwrap_err(),
        ArrangementError::IntersectionRefinementFailed
    );
}

#[test]
fn full_circle_seam_hit_by_diameter_is_two_half_discs() {
    let uses = vec![
        circle(0, p(0.0, 0.0), 2.0, Some(0)),
        line(10, p(-2.0, 0.0), p(2.0, 0.0), [10, 11], None),
    ];
    let a = build(&uses);
    assert_eq!(a.regions.len(), 2);
    for region in &a.regions {
        near(region.area, 2.0 * PI);
    }
    invariants(&a, &uses);
}

#[test]
fn native_major_and_minor_arc_boundaries_keep_their_selected_side() {
    for end in [PI / 2.0, 3.0 * PI / 2.0] {
        let mut arc = circle(0, p(0.0, 0.0), 2.0, Some(0));
        arc.range = [0.0, end];
        arc.source_range = [0.0, end];
        arc.endpoints = [0, 1];
        let start = arc.point(0.0);
        let finish = arc.point(end);
        arc.endpoints_3d = [xyz(start), xyz(finish)];
        let chord = line(1, finish, start, [1, 0], Some(0));
        let uses = vec![arc, chord];
        let a = build(&uses);
        assert_eq!(material_count(&a), 1);
        near(area(&a), 2.0 * (end - end.sin()));
        invariants(&a, &uses);
    }
}

#[test]
fn unrepresentable_native_subspans_refuse_atomically() {
    for (range, expected) in [
        ([-f64::MAX, f64::MAX], ArrangementError::NonFiniteInput),
        (
            [1.0, 1.0 + f64::EPSILON],
            ArrangementError::IntersectionRefinementFailed,
        ),
    ] {
        let mut uses = rectangle(0, 0.0, 0.0, 2.0, 2.0);
        uses[0].source_range = range;
        uses.push(line(10, p(1.0, 0.0), p(1.0, 2.0), [10, 11], None));
        let before = format!("{uses:?}");
        assert_eq!(
            build_arrangement(&ArrangementInput {
                uses: &uses,
                domain: ParamDomain::Plane,
                context: &context(),
            })
            .unwrap_err(),
            expected
        );
        assert_eq!(format!("{uses:?}"), before);
    }
}
