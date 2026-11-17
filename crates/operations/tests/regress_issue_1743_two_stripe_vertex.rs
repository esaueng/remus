//! Regression (#1743): two fillet stripes meeting at a vertex whose third
//! edge stays sharp, and a fillet chain crossing the tangent seam of an
//! earlier round, were refused unless the corner was convex and square.
//!
//! Reported on two everyday parts, at every radius:
//!
//! ```text
//! pocket floor loop (4 edges) fillet 1:  unsupported vertex blend at Id(37): 2 stripes meet
//! pocket floor, 2 adjacent edges:        (same)
//!   wall corners R5, then floor loop:    walking blend cannot follow the ridgeline across the tangent seam
//! drafted box top loop fillet 2:         unsupported vertex blend at Id(377): 2 stripes meet
//!   vertical edges R5, then top loop:    walking blend cannot follow the ridgeline across the tangent seam
//! ```
//!
//! Both refusals were the walking builder's, reported after the rolling-ball
//! rebuild had failed underneath it for three separate reasons.
//!
//! * A two-stripe corner was stationed by the angle between the two edges and
//!   its ledge put one radius down the sharp edge, facing the shared face.
//!   That is the ball tangent to the corner's three faces only when they meet
//!   squarely, so a drafted corner left free edges, and the concave form was
//!   never built exactly. The ball is now solved from the three planes.
//! * A curved stripe's sections always put the ball in the material, so a
//!   concave arc (a pocket floor running under a rounded wall corner) was
//!   rolled on the wrong side.
//! * Over a round that leans (a drafted body), the stripe has no torus and
//!   the ball leaves the wall for the round before or after the chain's
//!   vertex. It is now solved in closed form between the round's two seams.
//!
//! Every result here is checked for a closed, consistently oriented shell,
//! for no new validation error, for the face types the construction is exact
//! in, and against a volume computed without the kernel.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::f64::consts::PI;

use remus_check::validate::{Severity, ValidateOptions, validate_solid};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::{self, BlendEngine};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::draft::draft;
use remus_operations::measure;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// Deflection for the volume readings.
const DEFLECTION: f64 = 0.001;

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

fn solid_edges(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    let mut seen = Vec::new();
    for fid in solid_faces(topo, s).unwrap() {
        for e in face_edges(topo, fid) {
            if !seen.contains(&e) {
                seen.push(e);
            }
        }
    }
    seen
}

fn face_edges(topo: &Topology, f: FaceId) -> Vec<EdgeId> {
    let face = topo.face(f).unwrap();
    std::iter::once(face.outer_wire())
        .chain(face.inner_wires().iter().copied())
        .flat_map(|w| topo.wire(w).unwrap().edges().to_vec())
        .map(|oe| oe.edge())
        .collect()
}

fn ends(topo: &Topology, e: EdgeId) -> (Point3, Point3) {
    let edge = topo.edge(e).unwrap();
    (
        topo.vertex(edge.start()).unwrap().point(),
        topo.vertex(edge.end()).unwrap().point(),
    )
}

/// Edges with both ends at height `z`.
fn edges_at_z(topo: &Topology, s: SolidId, z: f64) -> Vec<EdgeId> {
    solid_edges(topo, s)
        .into_iter()
        .filter(|&e| {
            let (a, b) = ends(topo, e);
            (a.z() - z).abs() < 1e-6 && (b.z() - z).abs() < 1e-6
        })
        .collect()
}

/// Edges that climb more than 2.5 and pass `keep` at both ends.
fn upright_edges(topo: &Topology, s: SolidId, keep: impl Fn(Point3) -> bool) -> Vec<EdgeId> {
    solid_edges(topo, s)
        .into_iter()
        .filter(|&e| {
            let (a, b) = ends(topo, e);
            (a.z() - b.z()).abs() > 2.5 && keep(a) && keep(b)
        })
        .collect()
}

fn plane_faces(topo: &Topology, s: SolidId, outward: Vec3) -> Vec<FaceId> {
    solid_faces(topo, s)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            topo.face(f)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| (n - outward).length() < 1e-6)
        })
        .collect()
}

/// The issue's pocket: an 80 x 60 x 20 block with a 40 x 30 pocket 3 deep,
/// floor at z = 17, walls at x = 20, 60 and y = 15, 45.
fn pocket(topo: &mut Topology) -> SolidId {
    let body = make_box(topo, 80.0, 60.0, 20.0).unwrap();
    let tool = make_box(topo, 40.0, 30.0, 10.0).unwrap();
    transform_solid(topo, tool, &Mat4::translation(20.0, 15.0, 17.0)).unwrap();
    boolean(topo, BooleanOp::Cut, body, tool).unwrap()
}

/// The pocket's four upright wall corners.
fn pocket_corners(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    let corners = upright_edges(topo, s, |p| {
        p.z() > 16.0 && p.x() > 1.0 && p.x() < 79.0 && p.y() > 1.0 && p.y() < 59.0
    });
    assert_eq!(corners.len(), 4, "the pocket has four wall corners");
    corners
}

/// The issue's drafted body: a 40 x 40 x 20 box whose four sides are drafted
/// by `degrees` about its base. Positive widens it upward, so its top corners
/// are sharper than square (88 degrees at 2) and its bottom ones blunter.
fn drafted_box(topo: &mut Topology, degrees: f64) -> SolidId {
    let body = make_box(topo, 40.0, 40.0, 20.0).unwrap();
    let sides: Vec<FaceId> = [X, -X, Y, -Y]
        .into_iter()
        .flat_map(|n| plane_faces(topo, body, n))
        .collect();
    assert_eq!(sides.len(), 4);
    draft(
        topo,
        body,
        &sides,
        Z,
        Point3::new(0.0, 0.0, 0.0),
        degrees.to_radians(),
    )
    .unwrap()
}

fn fillet(topo: &mut Topology, s: SolidId, edges: &[EdgeId], radius: f64, what: &str) -> SolidId {
    let result = blend_ops::fillet_cascade(topo, s, edges, radius)
        .unwrap_or_else(|e| panic!("{what}: the fillet must build, got {e}"));
    assert_eq!(
        result.engine,
        BlendEngine::RollingBall,
        "{what}: the exact rebuild produces this blend"
    );
    assert!(!result.is_partial, "{what}: every edge is blended");
    result.solid
}

// ---------------------------------------------------------------------------
// Checks
// ---------------------------------------------------------------------------

fn census(topo: &Topology, s: SolidId) -> BTreeMap<&'static str, usize> {
    let mut counts = BTreeMap::new();
    for f in solid_faces(topo, s).unwrap() {
        let kind = match topo.face(f).unwrap().surface() {
            FaceSurface::Plane { .. } => "plane",
            FaceSurface::Cylinder(_) => "cylinder",
            FaceSurface::Cone(_) => "cone",
            FaceSurface::Sphere(_) => "sphere",
            FaceSurface::Torus(_) => "torus",
            FaceSurface::Nurbs(_) => "nurbs",
        };
        *counts.entry(kind).or_default() += 1;
    }
    counts
}

fn kinds(pairs: &[(&'static str, usize)]) -> BTreeMap<&'static str, usize> {
    pairs.iter().copied().collect()
}

fn validation_errors(topo: &Topology, s: SolidId) -> usize {
    validate_solid(topo, s, &ValidateOptions::default())
        .unwrap()
        .issues
        .iter()
        .filter(|issue| issue.severity == Severity::Error)
        .count()
}

/// The result is a closed 2-manifold whose every edge is used once each way,
/// carries no validation error the input did not, and differs from the input.
fn assert_sound(topo: &Topology, input: SolidId, result: SolidId, what: &str) {
    let shell = topo
        .shell(topo.solid(result).unwrap().outer_shell())
        .unwrap();
    validate_shell_closed(shell, topo)
        .unwrap_or_else(|e| panic!("{what}: the shell must be closed, got {e}"));
    let mut uses: BTreeMap<usize, Vec<bool>> = BTreeMap::new();
    for f in solid_faces(topo, result).unwrap() {
        let face = topo.face(f).unwrap();
        for w in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(w).unwrap().edges() {
                uses.entry(oe.edge().index())
                    .or_default()
                    .push(oe.is_forward() != face.is_reversed());
            }
        }
    }
    for (edge, uses) in &uses {
        assert!(
            uses.len() == 2 && uses[0] != uses[1],
            "{what}: edge {edge} must be used once in each direction, got {uses:?}"
        );
    }
    assert!(
        validation_errors(topo, result) <= validation_errors(topo, input),
        "{what}: the blend must add no validation error"
    );
    assert_ne!(
        census(topo, input),
        census(topo, result),
        "{what}: a blend that changed no face is a silent no-op"
    );
}

fn volume_change(topo: &Topology, input: SolidId, result: SolidId) -> f64 {
    measure::solid_volume(topo, result, DEFLECTION).unwrap()
        - measure::solid_volume(topo, input, DEFLECTION).unwrap()
}

fn assert_close(actual: f64, expected: f64, budget: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= budget,
        "{what}: volume change {actual:.5}, expected {expected:.5} (budget {budget})"
    );
}

// ---------------------------------------------------------------------------
// Volumes computed without the kernel
// ---------------------------------------------------------------------------

/// Cross-section of the material a radius-`r` fillet moves along an edge
/// whose faces' outward normals are `theta` apart: the kite between the
/// edge and the two contacts, less the ball's sector.
fn stripe_area(r: f64, theta: f64) -> f64 {
    r * r * ((theta / 2.0).tan() - theta / 2.0)
}

/// A two-stripe corner whose third edge stays sharp, from its three outward
/// plane normals alone (`cap` shared by the stripes): the stripes' setbacks
/// along their edges, and the volume the corner moves.
///
/// The corner cell is cut from the trihedral corner by the three planes
/// through the tangent ball's centre across the edges. Its eight vertices
/// are the corner, the ball's centre, the centre's feet on the three edges
/// and on the three faces. The fillet moves the cell less the ball's sector
/// inside it, whose solid angle is the spherical triangle of the normals.
/// `side` is -1 for a convex corner (the ball in the material), 1 concave.
fn corner_cell(r: f64, cap: Vec3, wall_a: Vec3, wall_b: Vec3, side: f64) -> (f64, f64, f64) {
    let det = cap.dot(wall_a.cross(wall_b));
    // The ball's centre, from the corner: `side * r` along every normal.
    let c = (wall_a.cross(wall_b) + wall_b.cross(cap) + cap.cross(wall_a)) * (side * r / det);
    let along = |n1: Vec3, n2: Vec3| {
        let d = n1.cross(n2).normalize().unwrap();
        if c.dot(d) > 0.0 { d } else { -d }
    };
    let (da, db, ds) = (
        along(cap, wall_a),
        along(cap, wall_b),
        along(wall_a, wall_b),
    );
    let v = Vec3::new(0.0, 0.0, 0.0);
    let (pa, pb, ps) = (da * c.dot(da), db * c.dot(db), ds * c.dot(ds));
    let (far, near_a, near_b) = (
        c - cap * (side * r),
        c - wall_a * (side * r),
        c - wall_b * (side * r),
    );
    let quads = [
        [v, pa, far, pb],
        [v, pa, near_a, ps],
        [v, pb, near_b, ps],
        [pa, far, c, near_a],
        [pb, far, c, near_b],
        [ps, near_a, c, near_b],
    ];
    let centroid = (v + pa + pb + ps + far + near_a + near_b + c) * 0.125;
    let mut cell = 0.0;
    for q in quads {
        for i in 1..3 {
            cell += (q[0] - centroid)
                .dot((q[i] - centroid).cross(q[i + 1] - centroid))
                .abs()
                / 6.0;
        }
    }
    let solid_angle = 2.0
        * det
            .abs()
            .atan2(1.0 + cap.dot(wall_a) + wall_a.dot(wall_b) + wall_b.dot(cap));
    (c.dot(da), c.dot(db), cell - solid_angle * r.powi(3) / 3.0)
}

/// Volume a convex radius-`r` fillet removes between the plane `z = top` and
/// a radius-`band_r` round about `axis` through `origin` (material inside),
/// from the round's seam with the wall of outward normal `flank_start` to
/// its seam with `flank_end`'s. Returns it with the ball's first centre.
///
/// The ball's centre runs one radius under the plane, `band_r - r` from the
/// axis. In the plane across its path at band angle `phi`, spanned by the
/// plane's normal and the band's radial `u`, a point is `c + a z + b u`, and
/// the fillet removes those in the wedge `a, b >= 0` that are under the top,
/// inside the round and outside the ball. Sweeping that region along the
/// path, a point moves `(c' + b u') . (z x u)` per unit `da db dphi`.
/// Midpoint rule in `b` with exact limits in `a`, Simpson in `phi`.
fn oblique_stripe_volume(
    r: f64,
    top: f64,
    origin: Point3,
    axis: Vec3,
    band_r: f64,
    flank_start: Vec3,
    flank_end: Vec3,
) -> (f64, Point3) {
    const PHI_STEPS: usize = 240;
    const B_STEPS: usize = 4000;
    let ring = band_r - r;
    let x = flank_start.normalize().unwrap();
    let y = axis.cross(x);
    let sweep = flank_end.dot(y).atan2(flank_end.dot(x));
    let along = Z.dot(axis);
    let z_across = Z - axis * along;
    let radial = |phi: f64| x * phi.cos() + y * phi.sin();
    let centre = |phi: f64| {
        let foot = origin + radial(phi) * ring;
        foot + axis * ((top - r - foot.z()) / along)
    };
    let slab = |phi: f64| {
        let u = radial(phi);
        let h = 1e-6;
        let c_dot = (centre(phi + h) - centre(phi - h)) * (0.5 / h);
        let across = Z.cross(u);
        let (k, speed, turn) = (Z.dot(u), c_dot.dot(across), axis.cross(u).dot(across));
        let b_max = 3.0 * r;
        #[allow(clippy::cast_precision_loss)]
        let db = b_max / B_STEPS as f64;
        let mut total = 0.0;
        for i in 0..B_STEPS {
            #[allow(clippy::cast_precision_loss)]
            let b = (i as f64 + 0.5) * db;
            // Outside the ball: a^2 + 2 a b k + b^2 >= r^2.
            let disc = (b * k).powi(2) - (b * b - r * r);
            let ball = if disc > 0.0 {
                (disc.sqrt() - b * k).max(0.0)
            } else {
                0.0
            };
            // Under the top: a + b k <= r.
            let mut hi = r - b * k;
            let mut lo = ball;
            // Inside the round: |(ring + b) u + a z_across| <= band_r.
            let qa = z_across.dot(z_across);
            let qb = 2.0 * (ring + b) * u.dot(z_across);
            let qc = (ring + b).powi(2) - band_r * band_r;
            if qa > 1e-18 {
                let dq = qb * qb - 4.0 * qa * qc;
                if dq <= 0.0 {
                    continue;
                }
                lo = lo.max((-qb - dq.sqrt()) / (2.0 * qa));
                hi = hi.min((-qb + dq.sqrt()) / (2.0 * qa));
            } else if qc > 0.0 {
                continue;
            }
            if hi > lo {
                total += (speed + b * turn).abs() * (hi - lo) * db;
            }
        }
        total
    };
    let mut sum = 0.0;
    for i in 0..=PHI_STEPS {
        let weight = if i == 0 || i == PHI_STEPS {
            1.0
        } else if i % 2 == 1 {
            4.0
        } else {
            2.0
        };
        #[allow(clippy::cast_precision_loss)]
        let phi = sweep * i as f64 / PHI_STEPS as f64;
        sum += weight * slab(phi);
    }
    #[allow(clippy::cast_precision_loss)]
    let volume = sweep.abs() / PHI_STEPS as f64 / 3.0 * sum;
    (volume, centre(0.0))
}

/// Volume a radius-`r` fillet of the top loop removes from the drafted box
/// once its four upright edges carry radius-5 rounds.
///
/// By symmetry: four oblique stripes over the rounds and four wall stripes,
/// each between the stations where the ball leaves one round and meets the
/// next. At the corner nearest the origin the round's axis is 5 inside both
/// walls, and the ball leaves the `-y` wall at `x = handover`.
fn drafted_rounded_top_loop_removal(degrees: f64, r: f64) -> f64 {
    let (sin, cos) = degrees.to_radians().sin_cos();
    let (wall_x, wall_y) = (Vec3::new(-cos, 0.0, -sin), Vec3::new(0.0, -cos, -sin));
    let axis = wall_x.cross(wall_y).normalize().unwrap();
    let axis = if axis.z() < 0.0 { -axis } else { axis };
    let origin = Point3::new(5.0 / cos, 5.0 / cos, 0.0);
    let (stripe, handover) = oblique_stripe_volume(r, 20.0, origin, axis, 5.0, wall_y, wall_x);
    let wall_stripe = (40.0 - 2.0 * handover.x()) * stripe_area(r, Z.dot(wall_x).acos());
    4.0 * (stripe + wall_stripe)
}

// ---------------------------------------------------------------------------
// The controls the issue reports as working
// ---------------------------------------------------------------------------

#[test]
fn plain_box_controls_keep_their_exact_results() {
    let mut topo = Topology::new();
    let body = make_box(&mut topo, 40.0, 40.0, 20.0).unwrap();
    let top = edges_at_z(&topo, body, 20.0);
    let rounded = fillet(&mut topo, body, &top, 2.0, "box top loop");
    assert_sound(&topo, body, rounded, "box top loop");
    assert_eq!(
        census(&topo, rounded),
        kinds(&[("cylinder", 4), ("plane", 10), ("sphere", 4)])
    );
    // Four stripes set back one radius at both ends, four square corners.
    let expected =
        -(4.0 * (40.0 - 4.0) * stripe_area(2.0, PI / 2.0) + 4.0 * 8.0 * (1.0 - PI / 6.0));
    assert_close(
        volume_change(&topo, body, rounded),
        expected,
        0.02,
        "box top loop",
    );

    let uprights = upright_edges(&topo, body, |_| true);
    let round = fillet(&mut topo, body, &uprights, 5.0, "box uprights");
    let top = edges_at_z(&topo, round, 20.0);
    assert_eq!(top.len(), 8);
    let both = fillet(&mut topo, round, &top, 2.0, "box top loop over rounds");
    assert_sound(&topo, round, both, "box top loop over rounds");
    assert_eq!(
        census(&topo, both),
        kinds(&[("cylinder", 8), ("plane", 6), ("torus", 4)])
    );
    // The same quadrature the drafted body is held to, here against a torus.
    assert_close(
        volume_change(&topo, round, both),
        -drafted_rounded_top_loop_removal(0.0, 2.0),
        0.05,
        "box top loop over rounds",
    );
}

// ---------------------------------------------------------------------------
// 1. Two stripes at a vertex whose third edge stays sharp
// ---------------------------------------------------------------------------

#[test]
fn pocket_floor_loop_rounds_its_concave_corners_with_ball_patches() {
    for radius in [1.0, 0.3] {
        let what = format!("pocket floor loop r{radius}");
        let mut topo = Topology::new();
        let body = pocket(&mut topo);
        let floor = edges_at_z(&topo, body, 17.0);
        assert_eq!(floor.len(), 4);
        let rounded = fillet(&mut topo, body, &floor, radius, &what);
        assert_sound(&topo, body, rounded, &what);
        // Four cylinders, four ball patches, and a ledge under each patch
        // where the sharp wall corner resumes.
        assert_eq!(
            census(&topo, rounded),
            kinds(&[("cylinder", 4), ("plane", 15), ("sphere", 4)]),
            "{what}"
        );
        // Stripes set back one radius at both ends, plus four corner cubes
        // less their ball octants. A concave fillet adds material.
        let stripes = (2.0 * (40.0 + 30.0) - 8.0 * radius) * stripe_area(radius, PI / 2.0);
        let corners = 4.0 * radius.powi(3) * (1.0 - PI / 6.0);
        assert_close(
            volume_change(&topo, body, rounded),
            stripes + corners,
            2e-3,
            &what,
        );
    }
}

#[test]
fn two_adjacent_pocket_floor_edges_share_one_corner_patch() {
    let mut topo = Topology::new();
    let body = pocket(&mut topo);
    let floor = edges_at_z(&topo, body, 17.0);
    // Two that share a vertex: one 40 long, one 30.
    let first = floor[0];
    let (a, b) = ends(&topo, first);
    let second = floor[1..]
        .iter()
        .copied()
        .find(|&e| {
            let (c, d) = ends(&topo, e);
            [c, d]
                .iter()
                .any(|p| (*p - a).length() < 1e-9 || (*p - b).length() < 1e-9)
        })
        .unwrap();
    let what = "two adjacent pocket floor edges";
    let rounded = fillet(&mut topo, body, &[first, second], 1.0, what);
    assert_sound(&topo, body, rounded, what);
    assert_eq!(
        census(&topo, rounded),
        kinds(&[("cylinder", 2), ("plane", 12), ("sphere", 1)])
    );
    // Each stripe is set back at the shared corner and runs out at its other.
    let expected = (40.0 + 30.0 - 2.0) * stripe_area(1.0, PI / 2.0) + (1.0 - PI / 6.0);
    assert_close(volume_change(&topo, body, rounded), expected, 2e-3, what);
}

/// The 2-degree box of the issue (88-degree top corners, 92-degree bottom
/// ones), and its mirror image.
#[test]
fn drafted_box_loops_round_their_oblique_corners_with_ball_patches() {
    for degrees in [2.0, -2.0] {
        for (z, cap) in [(20.0, Z), (0.0, -Z)] {
            for radius in [2.0, 0.1] {
                let what = format!("draft {degrees}, loop at z {z}, r{radius}");
                let mut topo = Topology::new();
                let body = drafted_box(&mut topo, degrees);
                let loop_edges = edges_at_z(&topo, body, z);
                assert_eq!(loop_edges.len(), 4, "{what}");
                let rounded = fillet(&mut topo, body, &loop_edges, radius, &what);
                assert_sound(&topo, body, rounded, &what);
                assert_eq!(
                    census(&topo, rounded),
                    kinds(&[("cylinder", 4), ("plane", 10), ("sphere", 4)]),
                    "{what}"
                );

                let (sin, cos) = degrees.to_radians().sin_cos();
                let (wall_x, wall_y) = (Vec3::new(cos, 0.0, -sin), Vec3::new(0.0, cos, -sin));
                let (setback_x, setback_y, corner) = corner_cell(radius, cap, wall_x, wall_y, -1.0);
                // The loop's side at this height.
                let side = 40.0 + 2.0 * z * degrees.to_radians().tan();
                let stripe =
                    (side - setback_x - setback_y) * stripe_area(radius, cap.dot(wall_x).acos());
                assert_close(
                    volume_change(&topo, body, rounded),
                    -4.0 * (stripe + corner),
                    0.01 * radius,
                    &what,
                );
            }
        }
    }
}

/// A pocket cut by a drafted tool: concave corners that are not square.
#[test]
fn drafted_pocket_floor_loop_rounds_its_oblique_concave_corners() {
    let what = "drafted pocket floor loop";
    let mut topo = Topology::new();
    let body = make_box(&mut topo, 80.0, 60.0, 20.0).unwrap();
    let tool = drafted_box(&mut topo, 3.0);
    transform_solid(&mut topo, tool, &Mat4::translation(20.0, 10.0, 12.0)).unwrap();
    let body = boolean(&mut topo, BooleanOp::Cut, body, tool).unwrap();
    let floor = edges_at_z(&topo, body, 12.0);
    assert_eq!(floor.len(), 4);
    let rounded = fillet(&mut topo, body, &floor, 1.0, what);
    assert_sound(&topo, body, rounded, what);
    assert_eq!(
        census(&topo, rounded),
        kinds(&[("cylinder", 4), ("plane", 15), ("sphere", 4)])
    );
    // The pocket's faces look into it: the floor up, the walls inward and,
    // leaning back as they rise, slightly up.
    let (sin, cos) = 3.0f64.to_radians().sin_cos();
    let (wall_x, wall_y) = (Vec3::new(cos, 0.0, sin), Vec3::new(0.0, cos, sin));
    let (setback_x, setback_y, corner) = corner_cell(1.0, Z, wall_x, wall_y, 1.0);
    let stripe = (40.0 - setback_x - setback_y) * stripe_area(1.0, Z.dot(wall_x).acos());
    assert_close(
        volume_change(&topo, body, rounded),
        4.0 * (stripe + corner),
        2e-3,
        what,
    );
}

// ---------------------------------------------------------------------------
// 2. A chain crossing the tangent seam of an earlier round
// ---------------------------------------------------------------------------

#[test]
fn pocket_floor_loop_follows_its_rounded_wall_corners_with_tori() {
    let what = "pocket floor loop over rounded corners";
    let mut topo = Topology::new();
    let body = pocket(&mut topo);
    let corners = pocket_corners(&topo, body);
    let round = fillet(&mut topo, body, &corners, 5.0, "pocket wall corners");
    let floor = edges_at_z(&topo, round, 17.0);
    assert_eq!(floor.len(), 8, "four lines and four arcs");
    let rounded = fillet(&mut topo, round, &floor, 1.0, what);
    assert_sound(&topo, round, rounded, what);
    // The corner rounds, the four wall stripes, and a torus under each round.
    assert_eq!(
        census(&topo, rounded),
        kinds(&[("cylinder", 8), ("plane", 11), ("torus", 4)])
    );
    // Wall stripes between the rounds, and the four torus quarters: one full
    // turn of the stripe's section about the round's axis (Pappus). The
    // section is the unit square against the wall less the ball's quarter
    // disc, whose centre is 4 from the axis.
    let stripes = 2.0 * (30.0 + 20.0) * stripe_area(1.0, PI / 2.0);
    let first_moment = 4.5 - (4.0 + 4.0 / (3.0 * PI)) * PI / 4.0;
    assert_close(
        volume_change(&topo, round, rounded),
        stripes + 2.0 * PI * first_moment,
        2e-3,
        what,
    );
}

/// Mesh volume: the reading that resolves a 0.1 fillet on this body.
fn mesh_volume(topo: &Topology, s: SolidId, deflection: f64) -> f64 {
    let mesh = tessellate_solid_with_tolerance(topo, s, deflection, 0.05).unwrap();
    mesh.indices
        .chunks(3)
        .map(|t| {
            let p = |i: u32| {
                let p = mesh.positions[i as usize];
                Vec3::new(p.x(), p.y(), p.z())
            };
            p(t[0]).dot(p(t[1]).cross(p(t[2]))) / 6.0
        })
        .sum()
}

/// The issue's drafted box, and its mirror image, on which the ball leaves
/// each wall after the chain's vertex instead of before it.
#[test]
fn drafted_top_loop_follows_its_leaning_rounds() {
    for degrees in [2.0, -2.0] {
        for radius in [2.0, 0.1] {
            let what = format!("draft {degrees}, top loop over rounds, r{radius}");
            let mut topo = Topology::new();
            let body = drafted_box(&mut topo, degrees);
            let uprights = upright_edges(&topo, body, |_| true);
            assert_eq!(uprights.len(), 4);
            let round = fillet(&mut topo, body, &uprights, 5.0, "drafted uprights");
            let top = edges_at_z(&topo, round, 20.0);
            assert_eq!(top.len(), 8, "four lines and four ellipse arcs");
            let rounded = fillet(&mut topo, round, &top, radius, &what);
            assert_sound(&topo, round, rounded, &what);
            // The rounds, the four wall stripes, and one stripe over each
            // round. That stripe has no closed form; everything it touches
            // keeps its own.
            assert_eq!(
                census(&topo, rounded),
                kinds(&[("cylinder", 8), ("nurbs", 4), ("plane", 6)]),
                "{what}"
            );
            // Each stripe is bounded by exact curves: two ellipses, where it
            // touches the top and the round, and its two end circles.
            for f in solid_faces(&topo, rounded).unwrap() {
                if !matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_)) {
                    continue;
                }
                let mut curves = BTreeMap::new();
                for e in face_edges(&topo, f) {
                    let kind = match topo.edge(e).unwrap().curve() {
                        EdgeCurve::Ellipse(_) => "ellipse",
                        EdgeCurve::Circle(_) => "circle",
                        _ => "other",
                    };
                    *curves.entry(kind).or_insert(0) += 1;
                }
                assert_eq!(curves, kinds(&[("circle", 2), ("ellipse", 2)]), "{what}");
            }

            let expected = -drafted_rounded_top_loop_removal(degrees, radius);
            if radius > 1.0 {
                assert_close(volume_change(&topo, round, rounded), expected, 0.15, &what);
            } else {
                let measured = mesh_volume(&topo, rounded, 5e-4) - mesh_volume(&topo, round, 5e-4);
                assert_close(measured, expected, 0.03 * expected.abs(), &what);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 3. A chamfer chain over the leaning rounds of a drafted body
// ---------------------------------------------------------------------------

fn chamfer(topo: &mut Topology, s: SolidId, edges: &[EdgeId], d: f64, what: &str) -> SolidId {
    let result = blend_ops::chamfer_v2(topo, s, edges, d, d)
        .unwrap_or_else(|e| panic!("{what}: the chamfer must build, got {e}"));
    assert!(!result.is_partial, "{what}: every edge is chamfered");
    result.solid
}

/// The four sides of a drafted box, each as its outward normal and offset
/// `n·p = c`, counter-clockwise seen from above. The box is `size` square at
/// `origin`'s height, its corner at `origin`.
fn drafted_sides(degrees: f64, origin: Point3, size: f64) -> [(Vec3, f64); 4] {
    let (sin, cos) = degrees.to_radians().sin_cos();
    let o = Vec3::new(origin.x(), origin.y(), origin.z());
    let side = |n: Vec3, through: Vec3| (n, n.dot(o + through));
    [
        side(Vec3::new(0.0, -cos, -sin), Vec3::new(0.0, 0.0, 0.0)),
        side(Vec3::new(cos, 0.0, -sin), Vec3::new(size, 0.0, 0.0)),
        side(Vec3::new(0.0, cos, -sin), Vec3::new(0.0, size, 0.0)),
        side(Vec3::new(-cos, 0.0, -sin), Vec3::new(0.0, 0.0, 0.0)),
    ]
}

/// The radius-`radius` round between consecutive sides: its rising axis and
/// the point of it at height 0.
fn round_between(a: (Vec3, f64), b: (Vec3, f64), radius: f64) -> (Vec3, Point3) {
    let axis = a.0.cross(b.0).normalize().unwrap();
    let axis = if axis.z() < 0.0 { -axis } else { axis };
    // n_a·p = c_a - r and n_b·p = c_b - r at p.z = 0.
    let (ca, cb) = (a.1 - radius, b.1 - radius);
    let det = a.0.x() * b.0.y() - a.0.y() * b.0.x();
    let x = (ca * b.0.y() - cb * a.0.y()) / det;
    let y = (a.0.x() * cb - b.0.x() * ca) / det;
    (axis, Point3::new(x, y, 0.0))
}

/// Area of the slice at height `z` through the body bounded by `sides` and
/// radius-`radius` rounds between them, every side moved `s` inward (each
/// round keeps its axis and loses `s` of its radius). Exact: the slice is
/// four segments and four arcs, each arc an affine image of a circular one,
/// so Green's theorem integrates it in closed form.
fn rounded_slice_area(sides: &[(Vec3, f64); 4], radius: f64, s: f64, z: f64) -> f64 {
    let cross = |p: Vec3, q: Vec3| p.x() * q.y() - p.y() * q.x();
    let r = radius - s;
    let mut twice = 0.0;
    let mut ends = Vec::new();
    for k in 0..4 {
        let (a, b) = (sides[k], sides[(k + 1) % 4]);
        let (axis, foot) = round_between(a, b, radius);
        let m = axis.cross(a.0);
        let sweep = b.0.dot(m).atan2(b.0.dot(a.0));
        assert!(sweep > 0.0, "the sides run counter-clockwise");
        let foot = Vec3::new(foot.x(), foot.y(), foot.z());
        // A radial `v` at the slice's height: v moved along the axis.
        let lift = |v: Vec3| v - axis * (v.z() / axis.z());
        let centre = foot + axis * ((z - foot.z()) / axis.z());
        let (cb, cc) = (lift(a.0) * r, lift(m) * r);
        twice += cross(centre, cb) * (sweep.cos() - 1.0)
            + cross(centre, cc) * sweep.sin()
            + cross(cb, cc) * sweep;
        ends.push((centre + cb, centre + cb * sweep.cos() + cc * sweep.sin()));
    }
    for k in 0..4 {
        twice += cross(ends[k].1, ends[(k + 1) % 4].0);
    }
    twice / 2.0
}

/// Volume a distance-`d` chamfer moves along the loop where the plane at
/// height `level` meets the rounded drafted sides, on the side of the plane
/// `toward` (-1 below, +1 above).
///
/// The chamfer is the set of points of the body with `h + w < ρ`: `h` the
/// distance to the plane, `w` the distance to the sides, and `ρ = d sin α`
/// with `α` the sides' angle to the plane. Its slice at `h` is what the
/// body's slice loses when the sides move in by `ρ - h`. Simpson in `h`.
fn rounded_loop_chamfer_volume(
    sides: &[(Vec3, f64); 4],
    radius: f64,
    level: f64,
    toward: f64,
    d: f64,
) -> f64 {
    const STEPS: usize = 200;
    let rho = d * sides[0].0.cross(Z).length();
    let slab = |h: f64| {
        let z = level + toward * h;
        rounded_slice_area(sides, radius, 0.0, z) - rounded_slice_area(sides, radius, rho - h, z)
    };
    let mut sum = 0.0;
    for i in 0..=STEPS {
        let weight = if i == 0 || i == STEPS {
            1.0
        } else if i % 2 == 1 {
            4.0
        } else {
            2.0
        };
        #[allow(clippy::cast_precision_loss)]
        let h = rho * i as f64 / STEPS as f64;
        sum += weight * slab(h);
    }
    #[allow(clippy::cast_precision_loss)]
    let step = rho / STEPS as f64;
    step / 3.0 * sum
}

/// Every band over a round is the elliptic cone `h + w = ρ` (`h` from the
/// plane at `level`, `w` from the round), bounded by two ellipses and two
/// lines: sampled across and along, every point is on it.
fn assert_bands_on_the_cone(
    topo: &Topology,
    s: SolidId,
    sides: &[(Vec3, f64); 4],
    radius: f64,
    level: f64,
    rho: f64,
    what: &str,
) {
    let rounds: Vec<(Vec3, Point3)> = (0..4)
        .map(|k| round_between(sides[k], sides[(k + 1) % 4], radius))
        .collect();
    let off_round = |p: Point3, (axis, foot): (Vec3, Point3)| {
        let v = p - foot;
        radius - (v - axis * axis.dot(v)).length()
    };
    let mut bands = 0;
    for f in solid_faces(topo, s).unwrap() {
        let face = topo.face(f).unwrap();
        let FaceSurface::Nurbs(surface) = face.surface() else {
            continue;
        };
        bands += 1;
        let mut curves = BTreeMap::new();
        for e in face_edges(topo, f) {
            let kind = match topo.edge(e).unwrap().curve() {
                EdgeCurve::Ellipse(_) => "ellipse",
                EdgeCurve::Line => "line",
                _ => "other",
            };
            *curves.entry(kind).or_insert(0) += 1;
        }
        assert_eq!(curves, kinds(&[("ellipse", 2), ("line", 2)]), "{what}");
        let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
        let at = |i: u32, j: u32| {
            remus_math::traits::ParametricSurface::evaluate(
                surface,
                u0 + (u1 - u0) * f64::from(i) / 8.0,
                v0 + (v1 - v0) * f64::from(j) / 8.0,
            )
        };
        // The band's round: the one its middle is nearest.
        let middle = at(4, 4);
        let round = *rounds
            .iter()
            .min_by(|a, b| {
                off_round(middle, **a)
                    .abs()
                    .total_cmp(&off_round(middle, **b).abs())
            })
            .unwrap();
        for i in 0..=8 {
            for j in 0..=8 {
                let p = at(i, j);
                let miss = (p.z() - level).abs() + off_round(p, round) - rho;
                assert!(
                    miss.abs() < 1e-9,
                    "{what}: band point {p:?} is {miss:e} off the cone"
                );
            }
        }
    }
    assert_eq!(bands, 4, "{what}: one band over each round");
}

/// The issue's last line: the drafted box's uprights rounded R5, then the
/// top loop chamfered. Also its mirror image (the bands hand over after the
/// vertices instead of before), the bottom loop, and a larger distance.
#[test]
fn drafted_rounded_top_loop_chamfers_with_elliptic_cone_bands() {
    for degrees in [2.0, -2.0] {
        for (z, toward) in [(20.0, -1.0), (0.0, 1.0)] {
            for d in [0.2, 1.0] {
                let what = format!("draft {degrees}, rounds, loop at z {z}, chamfer {d}");
                let mut topo = Topology::new();
                let body = drafted_box(&mut topo, degrees);
                let uprights = upright_edges(&topo, body, |_| true);
                let round = fillet(&mut topo, body, &uprights, 5.0, "drafted uprights");
                let loop_edges = edges_at_z(&topo, round, z);
                assert_eq!(loop_edges.len(), 8, "{what}: four lines and four arcs");
                let chamfered = chamfer(&mut topo, round, &loop_edges, d, &what);
                assert_sound(&topo, round, chamfered, &what);
                // Four plane bands and four elliptic cones; the rounds and
                // walls keep their surfaces.
                assert_eq!(
                    census(&topo, chamfered),
                    kinds(&[("cylinder", 4), ("nurbs", 4), ("plane", 10)]),
                    "{what}"
                );
                let sides = drafted_sides(degrees, Point3::new(0.0, 0.0, 0.0), 40.0);
                let rho = d * sides[0].0.cross(Z).length();
                assert_bands_on_the_cone(&topo, chamfered, &sides, 5.0, z, rho, &what);
                let expected = -rounded_loop_chamfer_volume(&sides, 5.0, z, toward, d);
                assert_close(
                    volume_change(&topo, round, chamfered),
                    expected,
                    1e-3 * expected.abs(),
                    &what,
                );
            }
        }
    }
}

/// The volume oracle above, on the undrafted box, against the exact cone
/// chamfer the square case already builds and its closed form.
#[test]
fn square_rounded_top_loop_chamfer_matches_the_slice_quadrature() {
    let what = "box, rounds, top loop chamfer 1";
    let mut topo = Topology::new();
    let body = make_box(&mut topo, 40.0, 40.0, 20.0).unwrap();
    let uprights = upright_edges(&topo, body, |_| true);
    let round = fillet(&mut topo, body, &uprights, 5.0, "box uprights");
    let top = edges_at_z(&topo, round, 20.0);
    let chamfered = chamfer(&mut topo, round, &top, 1.0, what);
    assert_sound(&topo, round, chamfered, what);
    assert_eq!(
        census(&topo, chamfered),
        kinds(&[("cone", 4), ("cylinder", 4), ("plane", 10)])
    );
    // Four wall prisms of the half unit square, and the four cone quarters
    // (Pappus: the triangle's centroid is 4 + 2/3 from the axis).
    let closed_form = 4.0 * 30.0 * 0.5 + 2.0 * PI * (4.0 + 2.0 / 3.0) * 0.5;
    let sides = drafted_sides(0.0, Point3::new(0.0, 0.0, 0.0), 40.0);
    let quadrature = rounded_loop_chamfer_volume(&sides, 5.0, 20.0, -1.0, 1.0);
    assert_close(quadrature, closed_form, 1e-9, "the quadrature");
    // `solid_volume` reads this body 0.19 high at a deflection of 1e-3 and
    // converges from there (0.04 at 2e-4); the fine mesh resolves it.
    let measured = mesh_volume(&topo, chamfered, 5e-4) - mesh_volume(&topo, round, 5e-4);
    assert_close(measured, -closed_form, 2e-3, what);
}

/// A pocket cut by a drafted tool whose uprights are rounded: the floor loop
/// is concave, and its bands add material under the leaning rounds.
#[test]
fn drafted_rounded_pocket_floor_loop_chamfers_with_concave_elliptic_cones() {
    let what = "drafted rounded pocket floor chamfer";
    let mut topo = Topology::new();
    let body = make_box(&mut topo, 80.0, 60.0, 20.0).unwrap();
    let tool = drafted_box(&mut topo, 3.0);
    let uprights = upright_edges(&topo, tool, |_| true);
    let tool = fillet(&mut topo, tool, &uprights, 5.0, "tool uprights");
    transform_solid(&mut topo, tool, &Mat4::translation(20.0, 10.0, 12.0)).unwrap();
    let pocket = boolean(&mut topo, BooleanOp::Cut, body, tool).unwrap();
    let floor = edges_at_z(&topo, pocket, 12.0);
    assert_eq!(floor.len(), 8, "four lines and four ellipse arcs");
    let chamfered = chamfer(&mut topo, pocket, &floor, 1.0, what);
    assert_sound(&topo, pocket, chamfered, what);
    let (before, after) = (census(&topo, pocket), census(&topo, chamfered));
    assert_eq!(after.get("nurbs"), Some(&4), "{what}: {after:?}");
    assert_eq!(after.get("cylinder"), before.get("cylinder"), "{what}");
    assert_eq!(
        after.get("plane").copied().unwrap_or(0),
        before.get("plane").copied().unwrap_or(0) + 4,
        "{what}"
    );
    let sides = drafted_sides(3.0, Point3::new(20.0, 10.0, 12.0), 40.0);
    let rho = sides[0].0.cross(Z).length();
    assert_bands_on_the_cone(&topo, chamfered, &sides, 5.0, 12.0, rho, what);
    let expected = rounded_loop_chamfer_volume(&sides, 5.0, 12.0, 1.0, 1.0);
    assert_close(
        volume_change(&topo, pocket, chamfered),
        expected,
        1e-3 * expected,
        what,
    );
}
