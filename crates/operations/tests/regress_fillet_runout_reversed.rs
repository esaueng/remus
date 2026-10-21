//! Three straight-edge fillets on fused and cut plate bodies that failed the
//! fillet postcondition through `fillet_cascade`, plus the walking-engine
//! defects the same bodies exposed.
//!
//! - A concave fillet at a T-junction (a tab narrower than its plate) runs
//!   out past the tab's side faces. The rolling-ball rebuild paired the two
//!   strip contacts at each side-face corner by proximity to the boundary
//!   edges, but one contact lies on the extension of the plate-top edge,
//!   beyond the corner, so the x = 30 side face came out wound through itself.
//! - A cut's tool faces are reversed planes. The rolling-ball rebuild read
//!   their stored normal as outward and re-emitted them unreversed, flipping
//!   every groove face (the groove-lip fillet and every fillet on a grooved
//!   bracket, wherever the groove sits). The walking engine read the trimmed
//!   groove floor's wire flags without its reversal, so the blend wall's loop
//!   could not alternate and it fell back to a detached, misoriented band.
//! - The walking engine's corner patch closing a spine end against coplanar
//!   continuation faces took the convex outward normal for a concave fillet.
//!   The flush-tab control "passed" with both end patches facing inward and
//!   a third of the fillet's volume.
//!
//! Oracles: no NURBS and a small analytic face set, a valid closed shell, a
//! watertight welded mesh, volume against the closed form at two deflections
//! below the clamp, and ray-cast probes at the run-out ends.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_4, PI};

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::extrude::extrude;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::tessellate::{tessellate_solid, welded_mesh_quality};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::solid::SolidId;

const T: f64 = 1e-6;

/// Removed (convex) or added (concave) area of one straight fillet section.
fn corner_area(r: f64) -> f64 {
    r * r * (1.0 - FRAC_PI_4)
}

fn placed_box(topo: &mut Topology, lo: [f64; 3], hi: [f64; 3]) -> SolidId {
    let s = make_box(topo, hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]).unwrap();
    transform_solid(topo, s, &Mat4::translation(lo[0], lo[1], lo[2])).unwrap();
    s
}

fn edges_where(topo: &Topology, s: SolidId, pred: impl Fn(Point3, Point3) -> bool) -> Vec<EdgeId> {
    solid_edges(topo, s)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            pred(
                topo.vertex(edge.start()).unwrap().point(),
                topo.vertex(edge.end()).unwrap().point(),
            )
        })
        .collect()
}

/// The ladder every case must clear: analytic census, valid closed shell,
/// watertight mesh, closed-form volume at two deflections below the clamp.
fn assert_exact_fillet(topo: &Topology, s: SolidId, expected_volume: f64, max_faces: usize) {
    let faces = solid_faces(topo, s).unwrap();
    let mut cylinders = 0;
    for &f in &faces {
        match topo.face(f).unwrap().surface() {
            FaceSurface::Plane { .. } => {}
            FaceSurface::Cylinder(_) => cylinders += 1,
            other => panic!("non-analytic or unexpected face {}", other.type_tag()),
        }
    }
    assert!(cylinders > 0, "no cylindrical blend wall");
    assert!(
        faces.len() <= max_faces,
        "{} faces, expected at most {max_faces}",
        faces.len()
    );

    let report = validate_solid(topo, s).unwrap();
    assert!(report.is_valid(), "invalid result: {:?}", report.issues);

    let quality = welded_mesh_quality(&tessellate_solid(topo, s, 0.01).unwrap());
    assert_eq!(quality.boundary_edges, 0, "mesh not watertight");
    assert_eq!(quality.non_manifold_edges, 0, "mesh non-manifold");

    for deflection in [1e-3, 5e-4] {
        let v = solid_volume(topo, s, deflection).unwrap();
        assert!(
            (v - expected_volume).abs() <= 1e-6 * expected_volume,
            "volume {v} at deflection {deflection}, closed form {expected_volume}"
        );
    }
}

fn classify(topo: &Topology, s: SolidId, p: Point3) -> PointClassification {
    classify_point(topo, s, p, &ClassifyOptions::default()).unwrap()
}

/// Plate 40×30×4 with a 20×8×30 tab standing on its front edge, x∈[10,30].
fn t_junction(topo: &mut Topology) -> SolidId {
    let plate = placed_box(topo, [0., 0., 0.], [40., 30., 4.]);
    let tab = placed_box(topo, [10., 0., 0.], [30., 8., 30.]);
    boolean(topo, BooleanOp::Fuse, plate, tab).unwrap()
}

/// The concave edge where the tab's back face meets the plate top.
fn tab_root(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    edges_where(topo, s, |a, b| {
        (a.z() - 4.).abs() < T
            && (b.z() - 4.).abs() < T
            && (a.y() - 8.).abs() < T
            && (b.y() - 8.).abs() < T
    })
}

#[test]
fn concave_fillet_runs_out_past_a_narrow_tab() {
    let mut topo = Topology::new();
    let s = t_junction(&mut topo);
    let root = tab_root(&topo, s);
    assert_eq!(root.len(), 1);

    let r = 3.0;
    let result = fillet_cascade(&mut topo, s, &root, r).unwrap();
    assert!(!result.is_partial);
    let v0 = 40.0 * 30.0 * 4.0 + 20.0 * 8.0 * 26.0;
    assert_exact_fillet(&topo, result.solid, v0 + 20.0 * corner_area(r), 12);

    // The sliver fills the corner along the tab and stops at its side
    // faces: inside just within each end, outside just beyond it.
    for x in [10.5, 29.5] {
        let p = Point3::new(x, 8.3, 4.3);
        assert_eq!(
            classify(&topo, result.solid, p),
            PointClassification::Inside
        );
    }
    for x in [9.5, 30.5] {
        let p = Point3::new(x, 8.3, 4.3);
        assert_eq!(
            classify(&topo, result.solid, p),
            PointClassification::Outside
        );
    }
    // Under the blend arc (inside the rolling ball) stays empty.
    let p = Point3::new(20.0, 10.5, 6.5);
    assert_eq!(
        classify(&topo, result.solid, p),
        PointClassification::Outside
    );
}

#[test]
fn concave_fillet_on_a_flush_tab() {
    let mut topo = Topology::new();
    let plate = placed_box(&mut topo, [0., 0., 0.], [20., 30., 4.]);
    let tab = placed_box(&mut topo, [0., 0., 0.], [20., 8., 30.]);
    let s = boolean(&mut topo, BooleanOp::Fuse, plate, tab).unwrap();
    let root = tab_root(&topo, s);
    let r = 3.0;
    let v = 20.0 * 30.0 * 4.0 + 20.0 * 8.0 * 26.0 + 20.0 * corner_area(r);

    let result = fillet_cascade(&mut topo, s, &root, r).unwrap();
    assert_exact_fillet(&topo, result.solid, v, 12);

    // The walking engine closes both spine ends with corner patches against
    // the coplanar plate and tab sides; they must face out of the sliver.
    let mut builder = remus_blend::fillet_builder::FilletBuilder::new(&mut topo, s);
    builder.add_edges(&root, r);
    let walked = builder.build().unwrap();
    assert_exact_fillet(&topo, walked.solid, v, 20);
}

/// Box 20×10×40 with a groove x∈[-1,21], y∈[8.5,11], z∈[26,29] cut across
/// its y = 10 face.
fn grooved_box(topo: &mut Topology) -> SolidId {
    let b = placed_box(topo, [0., 0., 0.], [20., 10., 40.]);
    let g = placed_box(topo, [-1., 8.5, 26.], [21., 11., 29.]);
    boolean(topo, BooleanOp::Cut, b, g).unwrap()
}

fn groove_lips(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    edges_where(topo, s, |a, b| {
        (a.y() - 10.).abs() < T
            && (b.y() - 10.).abs() < T
            && (a.z() - b.z()).abs() < T
            && ((a.z() - 26.).abs() < T || (a.z() - 29.).abs() < T)
    })
}

#[test]
fn groove_lip_fillets_against_reversed_tool_faces() {
    let r = 0.5;
    let v0 = 20.0 * 10.0 * 40.0 - 20.0 * 1.5 * 3.0;

    let mut topo = Topology::new();
    let s = grooved_box(&mut topo);
    let lips = groove_lips(&topo, s);
    assert_eq!(lips.len(), 2);
    let result = fillet_cascade(&mut topo, s, &lips, r).unwrap();
    assert!(!result.is_partial);
    assert_exact_fillet(&topo, result.solid, v0 - 2.0 * 20.0 * corner_area(r), 12);
    // The rounded lip removed the corner and kept the material inside the
    // ball; the groove stays open.
    for (corner_z, ball_z) in [(25.95, 25.55), (29.05, 29.45)] {
        let p = Point3::new(10.0, 9.95, corner_z);
        assert_eq!(
            classify(&topo, result.solid, p),
            PointClassification::Outside
        );
        let p = Point3::new(10.0, 9.95, ball_z);
        assert_eq!(
            classify(&topo, result.solid, p),
            PointClassification::Inside
        );
    }
    let p = Point3::new(10.0, 9.0, 27.5);
    assert_eq!(
        classify(&topo, result.solid, p),
        PointClassification::Outside
    );

    // Each engine on its own: the rolling-ball rebuild re-emits the groove's
    // reversed faces reversed, and the walking engine stitches its wall
    // against the reversed groove floor or ceiling.
    for &lip in &lips {
        let expected = v0 - 20.0 * corner_area(r);
        #[allow(deprecated)]
        let rolled =
            remus_operations::fillet::fillet_rolling_ball(&mut topo, s, &[lip], r).unwrap();
        assert_exact_fillet(&topo, rolled, expected, 11);

        let mut builder = remus_blend::fillet_builder::FilletBuilder::new(&mut topo, s);
        builder.add_edges(&[lip], r);
        let walked = builder.build().unwrap();
        assert_exact_fillet(&topo, walked.solid, expected, 11);
    }
}

/// The bracket: a 74×60×4 plate with a 30×8 U-slot in its front edge, two
/// 18×10×40 tabs, a groove across the tabs and three through holes.
fn bracket(topo: &mut Topology, groove: bool, holes: bool) -> SolidId {
    let outline: Vec<Point3> = [
        (0., 0.),
        (22., 0.),
        (22., 8.),
        (52., 8.),
        (52., 0.),
        (74., 0.),
        (74., 60.),
        (0., 60.),
    ]
    .iter()
    .map(|&(x, y)| Point3::new(x, y, 0.))
    .collect();
    let wire = make_polygon_wire(topo, &outline, 1e-7).unwrap();
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0., 0., 1.),
            d: 0.,
        },
    ));
    let mut s = extrude(topo, face, Vec3::new(0., 0., 1.), 4.).unwrap();
    for (x0, x1) in [(0., 18.), (56., 74.)] {
        let tab = placed_box(topo, [x0, 0., 0.], [x1, 10., 40.]);
        s = boolean(topo, BooleanOp::Fuse, s, tab).unwrap();
    }
    if groove {
        let g = placed_box(topo, [-1., 8.5, 26.], [75., 11., 29.]);
        s = boolean(topo, BooleanOp::Cut, s, g).unwrap();
    }
    if holes {
        for (x, y) in [(12., 40.), (37., 45.), (62., 40.)] {
            let c = make_cylinder(topo, 2.5, 8.).unwrap();
            transform_solid(topo, c, &Mat4::translation(x, y, -2.)).unwrap();
            s = boolean(topo, BooleanOp::Cut, s, c).unwrap();
        }
    }
    s
}

#[test]
fn slot_corner_fillets_after_groove_and_holes() {
    let r = 2.0;
    for (groove, holes) in [(false, false), (true, false), (false, true), (true, true)] {
        let mut topo = Topology::new();
        let s = bracket(&mut topo, groove, holes);
        let corners = edges_where(&topo, s, |a, b| {
            (a.y() - 8.).abs() < T
                && (b.y() - 8.).abs() < T
                && (a.x() - b.x()).abs() < T
                && ((a.x() - 22.).abs() < T || (a.x() - 52.).abs() < T)
        });
        assert_eq!(corners.len(), 2);

        let mut v0 = 4.0 * (74.0 * 60.0 - 30.0 * 8.0) + 2.0 * 18.0 * 10.0 * 36.0;
        if groove {
            v0 -= 2.0 * 18.0 * 1.5 * 3.0;
        }
        if holes {
            v0 -= 3.0 * PI * 2.5 * 2.5 * 4.0;
        }
        let expected = v0 + 2.0 * 4.0 * corner_area(r);

        // The groove's reversed faces sit far from the slot, yet the
        // rolling-ball rebuild re-emits every face of the body.
        #[allow(deprecated)]
        let rolled = remus_operations::fillet::fillet_rolling_ball(&mut topo, s, &corners, r)
            .unwrap_or_else(|e| panic!("groove={groove} holes={holes}: {e}"));
        assert_exact_fillet(&topo, rolled, expected, 40);

        let result = fillet_cascade(&mut topo, s, &corners, r).unwrap();
        assert_eq!(
            result.engine,
            remus_operations::blend_ops::BlendEngine::RollingBall,
            "groove={groove} holes={holes}"
        );
        assert_exact_fillet(&topo, result.solid, expected, 40);
        // The slot corners filled: the corner point is material now, and
        // the ball's centre stays empty.
        for x in [22.0, 52.0] {
            let inward = if x < 30.0 { 1.0 } else { -1.0 };
            let corner = Point3::new(x + 0.1 * inward, 7.9, 2.0);
            assert_eq!(
                classify(&topo, result.solid, corner),
                PointClassification::Inside
            );
            let centre = Point3::new(x + 2.0 * inward, 6.0, 2.0);
            assert_eq!(
                classify(&topo, result.solid, centre),
                PointClassification::Outside
            );
        }
    }
}
