//! O2.3c/d planar-arrangement production witnesses.
//!
//! The provenance-preserving UV arrangement migrates qualified planar
//! line/circle face splitting off the chord-quantized paths. These tests
//! prove the migrated production behavior end to end: a boolean whose
//! wall faces partition through the new path must keep exact analytic
//! volume, classify material correctly at probes that encode the cut
//! intent, validate clean on both validators, and tessellate watertight
//! at preview and fine deflections. No tolerance loosening, silent
//! healing, or mesh fallback: every boolean runs `ExactOnly` with
//! `BooleanOutcome` quality gating.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::solid::SolidId;

fn exact_boolean(
    topo: &mut Topology,
    op: BooleanOp,
    a: SolidId,
    b: SolidId,
) -> Result<SolidId, String> {
    let outcome = boolean_with_context(
        topo,
        op,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .map_err(|e| format!("{op:?}: {e:?}"))?;
    assert!(
        matches!(outcome.quality, BooleanQuality::Exact),
        "{op:?}: non-exact quality"
    );
    Ok(outcome.solid)
}

fn assert_strict_valid(topo: &Topology, s: SolidId, what: &str) {
    let strict = remus_operations::validate::validate_solid(topo, s)
        .map_err(|e| format!("{what}: validator error: {e:?}"))
        .unwrap();
    assert!(strict.is_valid(), "{what}: ops validator issues");
    let mut opts = remus_check::validate::ValidateOptions::default();
    opts.disabled_checks
        .insert(remus_check::validate::CheckId::ShellConnected);
    let rep = remus_check::validate::validate_solid(topo, s, &opts).unwrap();
    let errs: Vec<_> = rep
        .issues
        .iter()
        .filter(|i| i.severity == remus_check::validate::Severity::Error)
        .collect();
    assert!(
        errs.is_empty(),
        "{what}: check-crate errors: {}",
        errs.len()
    );
}

fn assert_watertight(topo: &Topology, s: SolidId, what: &str) {
    for d in [0.1, 0.01] {
        let mesh = tessellate_solid(topo, s, d).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "{what}: boundary at d={d}");
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "{what}: non-manifold at d={d}"
        );
    }
}

fn assert_inside(topo: &Topology, s: SolidId, point: Point3, what: &str) {
    let actual = classify_point(topo, s, point, &ClassifyOptions::default()).unwrap();
    assert!(
        matches!(actual, PointClassification::Inside),
        "{what}: {point:?} classified {actual:?}, expected Inside"
    );
}

fn assert_outside(topo: &Topology, s: SolidId, point: Point3, what: &str) {
    let actual = classify_point(topo, s, point, &ClassifyOptions::default()).unwrap();
    assert!(
        matches!(actual, PointClassification::Outside),
        "{what}: {point:?} classified {actual:?}, expected Outside"
    );
}
/// L-plate cut by a crossing tool: the tool's walls meet the L's stepped
/// top face in T-junctions the greedy wire builder weaves into a broken
/// trace, so the provenance arrangement owns the partition (fewer, clean
/// regions replace overlapping loops).
///
/// Closed forms: the fuse joins face-touching boxes (72 + 32 = 104); the
/// cut removes the tool's 2 x 6 x 2 intersection (24), leaving 80.
#[test]
fn cut_l_plate_through_step_is_exact_and_watertight() {
    let mut topo = Topology::new();
    let a = make_box(&mut topo, 6.0, 6.0, 2.0).unwrap();
    let b = make_box(&mut topo, 4.0, 4.0, 2.0).unwrap();
    remus_operations::transform::transform_solid(&mut topo, b, &Mat4::translation(6.0, 0.0, 0.0))
        .unwrap();
    let l = exact_boolean(&mut topo, BooleanOp::Fuse, a, b).unwrap();
    for d in [0.1, 1e-4] {
        let v = solid_volume(&topo, l, d).unwrap();
        assert!(
            (v - 104.0).abs() / 104.0 < 1e-4,
            "L fuse volume {v} vs closed form 104 at d={d}"
        );
    }
    let c = make_box(&mut topo, 2.0, 8.0, 4.0).unwrap();
    remus_operations::transform::transform_solid(&mut topo, c, &Mat4::translation(4.0, -1.0, -1.0))
        .unwrap();
    let result = exact_boolean(&mut topo, BooleanOp::Cut, l, c).unwrap();

    for d in [0.1, 1e-4] {
        let v = solid_volume(&topo, result, d).unwrap();
        assert!(
            (v - 80.0).abs() / 80.0 < 1e-4,
            "L cut volume {v} vs closed form 80 at d={d}"
        );
    }
    // Probes encode the cut intent: the notch is void, both L arms stay
    // solid, and the air above the plate is void.
    assert_inside(&topo, result, Point3::new(1.0, 1.0, 1.0), "kept A block");
    assert_inside(&topo, result, Point3::new(8.0, 2.0, 1.0), "kept B arm");
    assert_outside(&topo, result, Point3::new(5.0, 1.0, 1.0), "cut notch");
    assert_outside(&topo, result, Point3::new(5.0, 5.0, 1.0), "cut notch far");
    assert_outside(&topo, result, Point3::new(1.0, 1.0, 3.0), "air above");
    assert_strict_valid(&topo, result, "L cut");
    assert_watertight(&topo, result, "L cut");
}
#[test]
fn probe_dip_wire_folds() {
    use remus_math::curves::Circle3D;
    use remus_math::vec::Vec3;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};
    // Dip-following wire (my sub0 shape, simplified): rectangle with a
    // rim bite dipping to (56,14.5), vs the same wire chorded across.
    let build = |dip: bool| {
        let mut topo = Topology::new();
        let mut vid = |topo: &mut Topology, x: f64, y: f64| {
            topo.add_vertex(Vertex::new(Point3::new(x, y, 20.0), 1e-7))
        };
        let v00 = vid(&mut topo, 0.0, 0.0);
        let v60 = vid(&mut topo, 60.0, 0.0);
        let v6016 = vid(&mut topo, 60.0, 16.93844718719117);
        let v5216 = vid(&mut topo, 52.0, 16.93844718719117);
        let v528 = vid(&mut topo, 52.0, 8.0);
        let v08 = vid(&mut topo, 8.0, 8.0);
        let v016 = vid(&mut topo, 0.0, 16.938447185853647);
        let mut edges = vec![
            topo.add_edge(Edge::new(v00, v60, EdgeCurve::Line)),
            topo.add_edge(Edge::new(v60, v6016, EdgeCurve::Line)),
        ];
        if dip {
            let dip_pt = vid(&mut topo, 56.0, 14.5);
            let c = Circle3D::new(Point3::new(56.0, 19.0, 20.0), Vec3::new(0.0, 0.0, 1.0), 4.5)
                .unwrap();
            let t0 = c.project(Point3::new(60.0, 16.93844718719117, 20.0));
            let t1 = c.project(Point3::new(56.0, 14.5, 20.0));
            let t2 = c.project(Point3::new(52.0, 16.93844718719117, 20.0));
            let mut e1 = Edge::new(v6016, dip_pt, EdgeCurve::Circle(c.clone()));
            e1.set_trim(Some((t0, t1)));
            let mut e2 = Edge::new(dip_pt, v5216, EdgeCurve::Circle(c));
            e2.set_trim(Some((t1, t2)));
            edges.push(topo.add_edge(e1));
            edges.push(topo.add_edge(e2));
        } else {
            edges.push(topo.add_edge(Edge::new(v6016, v5216, EdgeCurve::Line)));
        }
        edges.push(topo.add_edge(Edge::new(v5216, v528, EdgeCurve::Line)));
        edges.push(topo.add_edge(Edge::new(v528, v08, EdgeCurve::Line)));
        edges.push(topo.add_edge(Edge::new(v08, v016, EdgeCurve::Line)));
        edges.push(topo.add_edge(Edge::new(v016, v00, EdgeCurve::Line)));
        let wire = topo.add_wire(
            Wire::new(
                edges.iter().map(|e| OrientedEdge::new(*e, true)).collect(),
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 20.0,
            },
        ));
        let mesh = remus_operations::tessellate::tessellate(&topo, face, 0.05).unwrap();
        (
            remus_operations::tessellate::boundary_edge_count(&mesh),
            remus_operations::tessellate::non_manifold_edge_count(&mesh),
            mesh.indices.len() / 3,
        )
    };
    let (b1, nm1, t1) = build(true);
    eprintln!("PROBE dip: boundary={b1} nonmanifold={nm1} tris={t1}");
    let (b2, nm2, t2) = build(false);
    eprintln!("PROBE chord: boundary={b2} nonmanifold={nm2} tris={t2}");
}
#[test]
fn probe_subface_meshes() {
    use remus_math::curves::Circle3D;
    use remus_math::vec::Vec3;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};
    // Exact sub1 wire from the adoption dump (middle band + lenses).
    let mut topo = Topology::new();
    let v = |topo: &mut Topology, x: f64, y: f64| {
        topo.add_vertex(Vertex::new(Point3::new(x, y, 20.0), 1e-7))
    };
    let c = Circle3D::new(Point3::new(56.0, 19.0, 20.0), Vec3::new(0.0, 0.0, 1.0), 4.5).unwrap();
    assert!((c.u_axis() - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-12);
    let v6016 = v(&mut topo, 60.0, 16.938447187191166);
    let v6021 = v(&mut topo, 60.0, 21.061552812808827);
    let v5623 = v(&mut topo, 56.0, 23.5);
    let v5221 = v(&mut topo, 52.0, 21.06155281280883);
    let v5216 = v(&mut topo, 52.0, 16.93844718719117);
    let mut ec = |topo: &mut Topology, a, b, t: (f64, f64)| {
        let mut edge = Edge::new(a, b, EdgeCurve::Circle(c.clone()));
        edge.set_trim(Some(t));
        topo.add_edge(edge)
    };
    let mut ln = |topo: &mut Topology, a, b| topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
    let edges = vec![
        ln(&mut topo, v6016, v6021),
        ec(
            &mut topo,
            v6021,
            v5623,
            (5.188271230045106, 6.283185307179586),
        ),
        ec(&mut topo, v5623, v5221, (0.0, 1.09491407713448)),
        ln(&mut topo, v5221, v5216),
        ec(
            &mut topo,
            v5216,
            v6016,
            (2.0466785764553133, 4.2365067307242725),
        ),
    ];
    let wire = topo.add_wire(
        Wire::new(
            edges.iter().map(|e| OrientedEdge::new(*e, true)).collect(),
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(
        wire,
        Vec::new(),
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 20.0,
        },
    ));
    let mesh = remus_operations::tessellate::tessellate(&topo, face, 0.05).unwrap();
    eprintln!(
        "PROBE sub1: boundary={} nonmanifold={} tris={}",
        remus_operations::tessellate::boundary_edge_count(&mesh),
        remus_operations::tessellate::non_manifold_edge_count(&mesh),
        mesh.indices.len() / 3
    );
}
#[test]
fn probe_wire279_folds() {
    use remus_math::curves::Circle3D;
    use remus_math::vec::Vec3;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};
    let mut topo = Topology::new();
    let mut vid = |topo: &mut Topology, x: f64, y: f64| {
        topo.add_vertex(Vertex::new(Point3::new(x, y, 20.0), 1e-7))
    };
    // Exact 279 wire (traversal order, z=20).
    let v00 = vid(&mut topo, 0.0, 0.0);
    let v120 = vid(&mut topo, 12.0, 0.0);
    let v480 = vid(&mut topo, 48.0, 0.0);
    let v600 = vid(&mut topo, 60.0, 0.0);
    let v6016 = vid(&mut topo, 60.0, 16.93844718719117);
    let v5216 = vid(&mut topo, 52.0, 16.93844718719117);
    let v528 = vid(&mut topo, 52.0, 8.0);
    let v488 = vid(&mut topo, 48.0, 8.0);
    let v128 = vid(&mut topo, 12.0, 8.0);
    let v88 = vid(&mut topo, 8.0, 8.0);
    let v816 = vid(&mut topo, 8.0, 16.93844718585366);
    let v016 = vid(&mut topo, 0.0, 16.938447185853647);
    let c1 = Circle3D::new(Point3::new(56.0, 19.0, 20.0), Vec3::new(0.0, 0.0, 1.0), 4.5).unwrap();
    let c2 = Circle3D::new(Point3::new(4.0, 19.0, 20.0), Vec3::new(0.0, 0.0, 1.0), 4.5).unwrap();
    let mut ec = |topo: &mut Topology, a, b, c: Circle3D, t: (f64, f64), fwd: bool| {
        let mut edge = Edge::new(a, b, EdgeCurve::Circle(c));
        edge.set_trim(Some(t));
        (topo.add_edge(edge), fwd)
    };
    let mut el = |topo: &mut Topology, a, b, fwd: bool| {
        (topo.add_edge(Edge::new(a, b, EdgeCurve::Line)), fwd)
    };
    let mut edges: Vec<(remus_topology::edge::EdgeId, bool)> = Vec::new();
    edges.push(el(&mut topo, v00, v120, true));
    edges.push(el(&mut topo, v120, v480, true));
    edges.push(el(&mut topo, v480, v600, true));
    edges.push(el(&mut topo, v600, v6016, true));
    edges.push(ec(
        &mut topo,
        v6016,
        v5216,
        c1,
        (4.2365067307242725, 2.0466785764553133),
        true,
    ));
    edges.push(el(&mut topo, v5216, v528, true));
    edges.push(el(&mut topo, v528, v488, true));
    edges.push(el(&mut topo, v488, v128, true));
    edges.push(el(&mut topo, v128, v88, true));
    edges.push(el(&mut topo, v88, v816, true));
    edges.push(ec(
        &mut topo,
        v816,
        v016,
        c2,
        (2.0466785764553124, 4.236506730724274),
        false,
    ));
    edges.push(el(&mut topo, v016, v00, true));
    let wire = topo.add_wire(
        Wire::new(
            edges
                .iter()
                .map(|(e, f)| OrientedEdge::new(*e, *f))
                .collect(),
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(
        wire,
        Vec::new(),
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 20.0,
        },
    ));
    let mesh = remus_operations::tessellate::tessellate(&topo, face, 0.05).unwrap();
    eprintln!(
        "PROBE w279: boundary={} nonmanifold={} tris={}",
        remus_operations::tessellate::boundary_edge_count(&mesh),
        remus_operations::tessellate::non_manifold_edge_count(&mesh),
        mesh.indices.len() / 3
    );
}
