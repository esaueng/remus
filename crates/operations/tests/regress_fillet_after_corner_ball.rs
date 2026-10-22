//! A rolling-ball fillet on a body that already carries a rolling-ball
//! two-edge corner (a ball octant over a planar ledge, where two filleted
//! edges meet an unfilleted one) must leave that corner exactly as it was.
//!
//! The rebuild re-emits every face of the body from vertex positions. The
//! ball octant and its ledge share one boundary, the z-level great-circle arc
//! under the ball, and no cylindrical spec mints it, so it came back as the
//! chord between its ends: the ledge grew into the lens between chord and arc
//! and the ball lost it. A fillet on an edge far from the corners removed
//! about 0.7 mm³ extra per corner and the result still validated, meshed
//! watertight and stayed all-analytic. The faces the blend does not reach are
//! now copied whole, so their arcs survive.
//!
//! Oracles: the second fillet's removed volume against its closed form at two
//! deflections below the clamp, ray-cast probes in the lens above each ledge
//! and inside the corners, and the ladder (analytic census, valid shell,
//! watertight mesh).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::FRAC_PI_4;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::vec::Point3;
use remus_operations::blend_ops::{BlendEngine, fillet_cascade};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::{tessellate_solid, welded_mesh_quality};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const DX: f64 = 20.0;
const DY: f64 = 10.0;
const DZ: f64 = 40.0;
/// Top-loop radius: four two-edge corners, each a ball octant over a ledge.
const R_TOP: f64 = 2.0;
/// Radius of the later fillet on a bottom edge, far from every corner.
const R_BOTTOM: f64 = 1.0;
/// Both volumes come from the same exact faces away from the new blend, so
/// their difference is the planar-and-cylindrical change alone: it converges
/// to the closed form to ~1e-11 mm³ at both deflections. 1e-6 mm³ leaves room
/// for summation order while the defect is 2.8 mm³.
const VOLUME_TOL: f64 = 1e-6;

fn spandrel(r: f64) -> f64 {
    r * r * (1.0 - FRAC_PI_4)
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

fn classify(topo: &Topology, s: SolidId, p: Point3) -> PointClassification {
    classify_point(topo, s, p, &ClassifyOptions::default()).unwrap()
}

/// A 20 × 10 × 40 block with its top loop rounded at r 2.
fn rounded_top(topo: &mut Topology) -> SolidId {
    let block = make_box(topo, DX, DY, DZ).unwrap();
    let top = edges_where(topo, block, |a, b| {
        (a.z() - DZ).abs() < 1e-9 && (b.z() - DZ).abs() < 1e-9
    });
    assert_eq!(top.len(), 4);
    let result = fillet_cascade(topo, block, &top, R_TOP).unwrap();
    assert_eq!(result.engine, BlendEngine::RollingBall);
    result.solid
}

/// Each corner's ledge is bounded by a quarter circle of the ball's equator.
fn ledge_arcs(topo: &Topology, s: SolidId) -> usize {
    solid_faces(topo, s)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            let face = topo.face(f).unwrap();
            let FaceSurface::Plane { normal, d } = face.surface() else {
                return false;
            };
            if normal.z().abs() < 1.0 - 1e-9 || (d.abs() - (DZ - R_TOP)).abs() > 1e-9 {
                return false;
            }
            topo.wire(face.outer_wire())
                .unwrap()
                .edges()
                .iter()
                .any(|oe| matches!(topo.edge(oe.edge()).unwrap().curve(), EdgeCurve::Circle(_)))
        })
        .count()
}

#[test]
fn far_fillet_keeps_earlier_corner_balls_exact() {
    let mut topo = Topology::new();
    let body = rounded_top(&mut topo);
    assert_eq!(ledge_arcs(&topo, body), 4, "input ledges lost their arcs");

    // The front bottom edge, y = 0, z = 0: nowhere near the top corners.
    let bottom = edges_where(&topo, body, |a, b| {
        a.y().abs() < 1e-9 && b.y().abs() < 1e-9 && a.z().abs() < 1e-9 && b.z().abs() < 1e-9
    });
    assert_eq!(bottom.len(), 1);
    let before: Vec<f64> = [1e-3, 5e-4]
        .iter()
        .map(|&d| solid_volume(&topo, body, d).unwrap())
        .collect();
    let result = fillet_cascade(&mut topo, body, &bottom, R_BOTTOM).unwrap();
    assert_eq!(result.engine, BlendEngine::RollingBall);
    let s = result.solid;

    for &f in &solid_faces(&topo, s).unwrap() {
        let surface = topo.face(f).unwrap().surface();
        assert!(
            !matches!(surface, FaceSurface::Nurbs(_)),
            "non-analytic face"
        );
    }
    let report = validate_solid(&topo, s).unwrap();
    assert!(report.is_valid(), "invalid result: {:?}", report.issues);
    let quality = welded_mesh_quality(&tessellate_solid(&topo, s, 0.01).unwrap());
    assert_eq!(quality.boundary_edges, 0, "mesh not watertight");
    assert_eq!(quality.non_manifold_edges, 0, "mesh non-manifold");
    assert_eq!(ledge_arcs(&topo, s), 4, "a ledge arc became a chord");

    let removed = DX * spandrel(R_BOTTOM);
    for (deflection, before) in [1e-3, 5e-4].into_iter().zip(before) {
        let after = solid_volume(&topo, s, deflection).unwrap();
        assert!(
            ((before - after) - removed).abs() < VOLUME_TOL,
            "removed {} mm³ at deflection {deflection}, closed form {removed} mm³",
            before - after
        );
    }

    // Every top corner, mirrored onto its own ball centre (cx, cy, 38).
    let z = DZ - R_TOP;
    for (cx, sx) in [(R_TOP, -1.0), (DX - R_TOP, 1.0)] {
        for (cy, sy) in [(R_TOP, -1.0), (DY - R_TOP, 1.0)] {
            let at = |u: f64, v: f64, w: f64| Point3::new(cx + sx * u, cy + sy * v, w);
            // In the lens between the equator's chord and its arc, just
            // above the ledge: inside the ball, so still material.
            assert_eq!(
                classify(&topo, s, at(1.25, 1.25, z + 0.1)),
                PointClassification::Inside,
                "lens above the ledge at ({cx}, {cy})"
            );
            // Below the ledge, toward the unfilleted vertical edge.
            assert_eq!(
                classify(&topo, s, at(1.37, 1.37, z - 0.87)),
                PointClassification::Inside,
                "under the corner at ({cx}, {cy})"
            );
            // Above the ledge outside the ball: rounded away.
            assert_eq!(
                classify(&topo, s, at(1.7, 1.7, z + 0.5)),
                PointClassification::Outside,
                "corner above the ledge at ({cx}, {cy})"
            );
        }
    }
}
