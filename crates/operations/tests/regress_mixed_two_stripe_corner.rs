//! Two-stripe mixed-convexity fillet corners (concave + convex, third edge
//! sharp) close with an exact torus quarter and a planar ledge.
//!
//! Local frame at the corner vertex `V`: `e_c` along the concave edge, `e_x`
//! along the convex edge, `e_v` along the unselected sharp edge, mutually
//! perpendicular. The concave stripe is built first; the convex ball then
//! rolls along its edge and around the concave stripe's end, which is the
//! notch torus of `fillet::notch_torus` (axis `e_c`, ring centre
//! `V + r(e_c + e_x + e_v)`, ring radius `2r`, tube `r`). With no stripe on
//! the sharp edge the quarter ends on its station plane `V + r·e_v`, where a
//! planar ledge closes it.
//!
//! In each meridian half-plane of the torus the convex roll removes one
//! fillet chip of area `A = r²(1 − π/4)` whose centroid lies `r(1 + c)` from
//! the axis, `c = (10 − 3π)/(3(4 − π))`. Pappus over the quarter turn:
//!
//! ```text
//! V_t = (π/2)·A·r(1 + c) = π r³ (11 − 3π) / 12
//! V   = V0 + A·L_concave − A·(L_convex − r) − V_t
//! ```
//!
//! with the concave stripe counted to the vertex and the convex stripe from
//! its station one radius out.

#![allow(
    clippy::disallowed_methods,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic
)]

use std::f64::consts::PI;

use remus_check::validate::{ValidateOptions, validate_solid};
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::{BlendEngine, blend_failure_code, fillet_cascade};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::extrude::extrude;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::query::edge_is_g1;
use remus_operations::tessellate::{tessellate_solid_with_tolerance, welded_mesh_quality};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{edge_to_face_map, solid_edges, solid_faces};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const TOL: f64 = 1e-8;

fn chip_area(r: f64) -> f64 {
    r * r * (1.0 - PI / 4.0)
}

/// Material the convex roll removes around the concave stripe's end.
fn torus_quarter(r: f64) -> f64 {
    PI * r.powi(3) * (11.0 - 3.0 * PI) / 12.0
}

fn extruded(topo: &mut Topology, points: &[(f64, f64)], height: f64) -> SolidId {
    let points: Vec<Point3> = points
        .iter()
        .map(|&(x, y)| Point3::new(x, y, 0.0))
        .collect();
    let wire = make_polygon_wire(topo, &points, 1e-7).unwrap();
    let face = topo.add_face(Face::new(
        wire,
        Vec::new(),
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), height).unwrap()
}

/// The `fillet_torture` "mixed-convexity-chain" L: inner corner at (2,2),
/// height 5, volume 120.
fn torture_l(topo: &mut Topology) -> SolidId {
    extruded(
        topo,
        &[
            (0.0, 0.0),
            (7.0, 0.0),
            (7.0, 2.0),
            (2.0, 2.0),
            (2.0, 7.0),
            (0.0, 7.0),
        ],
        5.0,
    )
}

/// A 40x30x4 plate fused with a 20x8x30 tab at x in [10,30], flush with the
/// plate's y=0 side: the cap plane y=0 arrives as coplanar-split faces.
fn flush_tab(topo: &mut Topology) -> SolidId {
    let plate = make_box(topo, 40.0, 30.0, 4.0).unwrap();
    let tab = make_box(topo, 20.0, 8.0, 30.0).unwrap();
    transform_solid(
        topo,
        tab,
        &remus_math::mat::Mat4::translation(10.0, 0.0, 0.0),
    )
    .unwrap();
    boolean(topo, BooleanOp::Fuse, plate, tab).unwrap()
}

fn edge_between(topo: &Topology, solid: SolidId, a: Point3, b: Point3) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            ((start - a).length() < TOL && (end - b).length() < TOL)
                || ((start - b).length() < TOL && (end - a).length() < TOL)
        })
        .unwrap_or_else(|| panic!("edge {a:?} -- {b:?}"))
}

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

/// Closed, valid, watertight; returns the torus count.
fn assert_sound(topo: &Topology, solid: SolidId, name: &str) -> usize {
    let shell = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    validate_shell_closed(shell, topo).unwrap_or_else(|e| panic!("{name}: open shell: {e}"));
    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    assert!(
        report.is_valid(),
        "{name}: invalid solid: {:?}",
        report.issues
    );
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01, 0.1).unwrap();
    let quality = welded_mesh_quality(&mesh);
    assert!(quality.is_watertight(), "{name}: mesh not watertight");
    assert_eq!(quality.non_manifold_edges, 0, "{name}: non-manifold mesh");
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .count()
}

/// Every circular edge of every torus face: `(g1_seams, sharp_seams)`.
fn torus_seams(topo: &Topology, solid: SolidId) -> (usize, usize) {
    let map = edge_to_face_map(topo, solid).unwrap();
    let (mut g1, mut sharp) = (0, 0);
    for face_id in solid_faces(topo, solid).unwrap() {
        let face = topo.face(face_id).unwrap();
        if !matches!(face.surface(), FaceSurface::Torus(_)) {
            continue;
        }
        for oe in topo.wire(face.outer_wire()).unwrap().edges() {
            let edge = topo.edge(oe.edge()).unwrap();
            assert!(
                matches!(edge.curve(), EdgeCurve::Circle(_)),
                "torus corner seams are exact circles"
            );
            let faces = &map[&oe.edge().index()];
            assert_eq!(faces.len(), 2, "seam {:?} must join two faces", oe.edge());
            if edge_is_g1(topo, oe.edge(), faces[0], faces[1]).unwrap() {
                g1 += 1;
            } else {
                sharp += 1;
            }
        }
    }
    (g1, sharp)
}

fn fillet(topo: &mut Topology, solid: SolidId, edges: &[EdgeId], r: f64) -> SolidId {
    let result = fillet_cascade(topo, solid, edges, r)
        .unwrap_or_else(|e| panic!("r={r}: two-stripe mixed corner must build: {e}"));
    assert!(!result.is_partial);
    assert_eq!(result.engine, BlendEngine::RollingBall);
    result.solid
}

#[test]
fn torture_l_two_stripe_corner_matches_closed_form() {
    for r in [0.4, 1.0] {
        let mut topo = Topology::new();
        let input = torture_l(&mut topo);
        let edges = [
            edge_between(&topo, input, p(2.0, 2.0, 0.0), p(2.0, 2.0, 5.0)),
            edge_between(&topo, input, p(2.0, 2.0, 5.0), p(2.0, 7.0, 5.0)),
        ];
        let out = fillet(&mut topo, input, &edges, r);
        assert_eq!(assert_sound(&topo, out, "torture-l"), 1);
        // Concave stripe to the vertex (5), convex from its station (5 - r).
        let expected = 120.0 + chip_area(r) * 5.0 - chip_area(r) * (5.0 - r) - torus_quarter(r);
        let volume = solid_volume(&topo, out, 0.005).unwrap();
        assert!(
            (volume - expected).abs() < 1e-5,
            "r={r}: volume {volume:.9} vs closed form {expected:.9}"
        );
        // Stripe end, cap tangency and concave-stripe seams are G1; the
        // ledge seam is the one sharp edge the corner adds.
        assert_eq!(torus_seams(&topo, out), (3, 1), "r={r}");
    }
}

/// The ledge is the station plane one radius up the sharp edge, facing back
/// down it, bounded by the sharp edge's preserved point and the torus's end
/// section.
#[test]
fn ledge_sits_on_the_sharp_edge_station_plane() {
    let r = 1.0;
    let mut topo = Topology::new();
    let input = torture_l(&mut topo);
    let edges = [
        edge_between(&topo, input, p(2.0, 2.0, 0.0), p(2.0, 2.0, 5.0)),
        edge_between(&topo, input, p(2.0, 2.0, 5.0), p(2.0, 7.0, 5.0)),
    ];
    let out = fillet(&mut topo, input, &edges, r);
    // Sharp edge runs +x from V = (2,2,5): the ledge is x = 3, outward -x.
    let ledges: Vec<_> = solid_faces(&topo, out)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            let face = topo.face(f).unwrap();
            face.effective_plane_normal()
                .is_some_and(|n| (n - Vec3::new(-1.0, 0.0, 0.0)).length() < 1e-12)
                && topo
                    .wire(face.outer_wire())
                    .unwrap()
                    .edges()
                    .iter()
                    .all(|oe| {
                        let e = topo.edge(oe.edge()).unwrap();
                        (topo.vertex(e.start()).unwrap().point().x() - 3.0).abs() < TOL
                    })
        })
        .collect();
    assert_eq!(ledges.len(), 1, "exactly one ledge face on x = 3");
    let mut corners: Vec<Point3> = topo
        .wire(topo.face(ledges[0]).unwrap().outer_wire())
        .unwrap()
        .edges()
        .iter()
        .map(|oe| {
            let e = topo.edge(oe.edge()).unwrap();
            topo.vertex(oe.oriented_start(e)).unwrap().point()
        })
        .collect();
    corners.sort_by(|a, b| a.y().total_cmp(&b.y()).then(a.z().total_cmp(&b.z())));
    // B2 = V - r·e_x + r·e_v, P = V + r·e_v, M2 = V - r·e_c + r·e_v.
    let want = [p(3.0, 1.0, 5.0), p(3.0, 2.0, 4.0), p(3.0, 2.0, 5.0)];
    assert_eq!(corners.len(), 3);
    for (got, want) in corners.iter().zip(want) {
        assert!(
            (*got - want).length() < TOL,
            "ledge corner {got:?} vs {want:?}"
        );
    }
}

/// The cap plane y = 0 arrives split into three coplanar faces around the
/// corner (plate side, overlap, tab side); the torus's tangency arc crosses
/// both seams. Both tab corners, mirrored.
#[test]
fn flush_tab_two_stripe_corners_match_closed_form() {
    let r = 1.0;
    let mut topo = Topology::new();
    let input = flush_tab(&mut topo);
    let edges = [
        edge_between(&topo, input, p(30.0, 0.0, 4.0), p(30.0, 8.0, 4.0)),
        edge_between(&topo, input, p(30.0, 0.0, 4.0), p(40.0, 0.0, 4.0)),
        edge_between(&topo, input, p(10.0, 0.0, 4.0), p(10.0, 8.0, 4.0)),
        edge_between(&topo, input, p(0.0, 0.0, 4.0), p(10.0, 0.0, 4.0)),
    ];
    let out = fillet(&mut topo, input, &edges, r);
    assert_eq!(assert_sound(&topo, out, "flush-tab"), 2);
    let expected =
        8960.0 + 2.0 * (chip_area(r) * 8.0 - chip_area(r) * (10.0 - r) - torus_quarter(r));
    let volume = solid_volume(&topo, out, 0.005).unwrap();
    assert!(
        (volume - expected).abs() < 1e-5,
        "volume {volume:.9} vs closed form {expected:.9}"
    );
    // The tangency arc splits into three sub-arcs per corner at the seams.
    assert_eq!(torus_seams(&topo, out), (2 * 5, 2));
}

/// The same flush corner with its sharp edge filleted too is the three-edge
/// notch over a cap split three ways (the whole-edge bracket's flush corners).
/// Its residual against stripes inset one radius is the notch's
/// `t3 = V_t − A·r = (14π − 3π² − 12) r³ / 12`.
#[test]
fn flush_tab_three_edge_notch_over_three_way_split_cap() {
    let r = 1.0;
    let mut topo = Topology::new();
    let input = flush_tab(&mut topo);
    let edges = [
        edge_between(&topo, input, p(30.0, 0.0, 4.0), p(30.0, 8.0, 4.0)),
        edge_between(&topo, input, p(30.0, 0.0, 4.0), p(40.0, 0.0, 4.0)),
        edge_between(&topo, input, p(30.0, 0.0, 4.0), p(30.0, 0.0, 30.0)),
    ];
    let out = fillet(&mut topo, input, &edges, r);
    assert_eq!(assert_sound(&topo, out, "flush-notch"), 1);
    let a = chip_area(r);
    let t3 = torus_quarter(r) - a * r;
    let expected = 8960.0 + a * (8.0 - r) - a * (10.0 - r) - a * (26.0 - r) - t3;
    let volume = solid_volume(&topo, out, 0.005).unwrap();
    assert!(
        (volume - expected).abs() < 1e-5,
        "volume {volume:.9} vs closed form {expected:.9}"
    );
    assert_eq!(torus_seams(&topo, out), (6, 0));
}

/// Outside the rectangular cell the corner has no closed form, but it builds
/// as the two fillets would by hand: the concave stripe first, then the
/// convex ball rolling along its edge into the concave stripe's band — the
/// order the qualified cell builds in. The result is sound and is exactly
/// the hand-made body. (An oblique inner wall.)
#[test]
fn oblique_two_stripe_corner_builds_as_by_hand() {
    let outline = [
        (0.0, 0.0),
        (7.0, 0.0),
        (7.0, 2.0),
        (3.0, 2.0),
        (2.0, 7.0),
        (0.0, 7.0),
    ];
    let r = 0.4;
    let concave = (p(3.0, 2.0, 0.0), p(3.0, 2.0, 5.0));
    let convex = (p(3.0, 2.0, 5.0), p(2.0, 7.0, 5.0));

    let mut topo = Topology::new();
    let input = extruded(&mut topo, &outline, 5.0);
    let concave_edge = edge_between(&topo, input, concave.0, concave.1);
    let first = fillet_cascade(&mut topo, input, &[concave_edge], r)
        .unwrap()
        .solid;
    // The convex edge, shortened at its concave end by the first fillet.
    let rest = solid_edges(&topo, first)
        .unwrap()
        .into_iter()
        .find(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            let on = |q: Point3| {
                let span = convex.1 - convex.0;
                let t = (q - convex.0).dot(span) / span.dot(span);
                ((q - convex.0) - span * t).length() < 1e-6 && (-1e-9..=1.0 + 1e-9).contains(&t)
            };
            matches!(edge.curve(), EdgeCurve::Line)
                && on(topo.vertex(edge.start()).unwrap().point())
                && on(topo.vertex(edge.end()).unwrap().point())
        })
        .unwrap();
    let by_hand = fillet_cascade(&mut topo, first, &[rest], r).unwrap().solid;
    let by_hand_volume = solid_volume(&topo, by_hand, 0.005).unwrap();

    let mut topo = Topology::new();
    let input = extruded(&mut topo, &outline, 5.0);
    let edges = [
        edge_between(&topo, input, concave.0, concave.1),
        edge_between(&topo, input, convex.0, convex.1),
    ];
    let result = fillet_cascade(&mut topo, input, &edges, r)
        .unwrap_or_else(|e| panic!("{}: {e}", blend_failure_code(&e)));
    assert!(!result.is_partial);
    assert_sound(&topo, result.solid, "oblique");
    let volume = solid_volume(&topo, result.solid, 0.005).unwrap();
    assert!(
        (volume - by_hand_volume).abs() < 1e-9 * by_hand_volume.max(1.0),
        "{volume} as one selection, {by_hand_volume} by hand"
    );
}
