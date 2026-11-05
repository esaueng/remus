//! B38 / P-Class 8.1 seeds `5201364664492509300` and `1146645986253340484`:
//! exact sphere–torus booleans whose split sphere pieces were wound against
//! their faces.
//!
//! The splitter emitted each kept sphere piece with the right material, but
//! `orient_revolved_face_wires` re-oriented hole-free sphere loops with a 3D
//! oracle (Newell vector against the normal at the projected centroid):
//!
//! - seed 5201: a thin crescent along the sphere's equator (sphere r5 at
//!   (0,3,0) inside a torus R5.5/r2.75) has a vector area nearly
//!   perpendicular to its centroid normal, and the chords of its 240° equator
//!   arc sag through the whole crescent, so the oracle reversed a correct
//!   loop (intersect and torus − sphere: 5 same-sense shared edges);
//! - seed 1146: the two sphere discs inside a torus tube (torus R4/r2 at
//!   (-7,0,-3), sphere r8) are each bounded by ONE closed edge, which the 3D
//!   oracle declines, so the splitter's arbitrary traversal survived
//!   (intersect and both cuts: 2 same-sense shared edges).
//!
//! Sphere loops are now signed by their area in the sphere's own
//! (longitude, latitude) chart, the rule cylinder, cone and torus loops
//! already follow. These tests hold every operator in both operand orders at
//! three scales and two rigid placements to strict validity, opposing
//! coedges, independent Cartesian membership, a slice-integral intersection
//! volume with inclusion–exclusion, operand preservation and raw mesh
//! closure, and refusals to transactional rollback.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::f64::consts::PI;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::context::{CancellationToken, FallbackPolicy, OperationContext, WorkBudgets};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::solid_volume;
use remus_operations::mirror::mirror;
use remus_operations::primitives::{make_sphere, make_torus};
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

/// A sphere and a z-axis torus, both in the seed's unit-scale frame.
#[derive(Clone, Copy, Debug)]
struct Pair {
    name: &'static str,
    sphere_center: [f64; 3],
    sphere_radius: f64,
    torus_center: [f64; 3],
    major: f64,
    minor: f64,
}

/// Seed 5201: the mirrored sphere's centre is (0, 3, 0).
const SEED_5201: Pair = Pair {
    name: "seed5201",
    sphere_center: [0.0, 3.0, 0.0],
    sphere_radius: 5.0,
    torus_center: [0.0, 0.0, 0.0],
    major: 5.5,
    minor: 2.75,
};

/// Seed 1146: the torus mirrored about x = -4, then moved by (1, 0, -3).
const SEED_1146: Pair = Pair {
    name: "seed1146",
    sphere_center: [0.0, 0.0, 0.0],
    sphere_radius: 8.0,
    torus_center: [-7.0, 0.0, -3.0],
    major: 4.0,
    minor: 2.0,
};

fn topology_counts(t: &Topology) -> [usize; 8] {
    [
        t.num_vertices(),
        t.num_edges(),
        t.num_wires(),
        t.num_faces(),
        t.num_shells(),
        t.num_solids(),
        t.num_loops(),
        t.num_coedges(),
    ]
}

/// Area common to two discs of radii `a`, `b` whose centres are `d` apart.
fn lens(a: f64, b: f64, d: f64) -> f64 {
    if a <= 0.0 || b <= 0.0 || d >= a + b {
        return 0.0;
    }
    if d <= (a - b).abs() {
        return PI * a.min(b).powi(2);
    }
    let alpha = ((d * d + a * a - b * b) / (2.0 * d * a))
        .clamp(-1.0, 1.0)
        .acos();
    let beta = ((d * d + b * b - a * a) / (2.0 * d * b))
        .clamp(-1.0, 1.0)
        .acos();
    let kite = ((-d + a + b) * (d + a - b) * (d - a + b) * (d + a + b))
        .max(0.0)
        .sqrt();
    a * a * alpha + b * b * beta - 0.5 * kite
}

/// Generalized winding number of a closed triangle mesh about `point`: the
/// solid angle each triangle subtends (Van Oosterom–Strackee), over 4π.
fn winding_number(mesh: &TriangleMesh, point: Point3) -> f64 {
    let mut total = 0.0;
    for tri in mesh.indices.chunks_exact(3) {
        let [a, b, c] = [tri[0], tri[1], tri[2]].map(|i| mesh.positions[i as usize] - point);
        let (la, lb, lc) = (a.length(), b.length(), c.length());
        let numerator = a.dot(b.cross(c));
        let denominator = la * lb * lc + a.dot(b) * lc + a.dot(c) * lb + b.dot(c) * la;
        total += 2.0 * numerator.atan2(denominator);
    }
    total / (4.0 * PI)
}

/// Intersection volume by horizontal slices, independent of the kernel: each
/// slice is the sphere's disc intersected with the torus's annulus. The tube
/// height `z = c + r sin θ` is substituted so the slice width's square-root
/// ends integrate smoothly; composite Simpson.
fn intersection_volume(pair: Pair) -> f64 {
    let [sx, sy, sz] = pair.sphere_center;
    let [tx, ty, tz] = pair.torus_center;
    let d = (sx - tx).hypot(sy - ty);
    let slice = |theta: f64| {
        let z = tz + pair.minor * theta.sin();
        let width = pair.minor * theta.cos();
        let disc = (pair.sphere_radius.powi(2) - (z - sz).powi(2))
            .max(0.0)
            .sqrt();
        let annulus = lens(pair.major + width, disc, d) - lens(pair.major - width, disc, d);
        annulus * pair.minor * theta.cos()
    };
    let n = 200_000;
    let h = PI / f64::from(n);
    let interior: f64 = (1..n)
        .map(|i| {
            let weight = if i % 2 == 0 { 2.0 } else { 4.0 };
            weight * slice(-PI / 2.0 + f64::from(i) * h)
        })
        .sum();
    h / 3.0 * (slice(-PI / 2.0) + slice(PI / 2.0) + interior)
}

fn sphere_volume(pair: Pair) -> f64 {
    4.0 / 3.0 * PI * pair.sphere_radius.powi(3)
}

fn torus_volume(pair: Pair) -> f64 {
    2.0 * PI * PI * pair.major * pair.minor * pair.minor
}

fn placement(placed: bool) -> Mat4 {
    if placed {
        Mat4::translation(17.0, -23.0, 31.0) * Mat4::rotation_x(0.37) * Mat4::rotation_z(-0.61)
    } else {
        Mat4::identity()
    }
}

/// Build the sphere and torus at `scale`, translated to their seed centres,
/// then rigidly placed. Returns `(sphere, torus)`.
fn build(topo: &mut Topology, pair: Pair, scale: f64, place: Mat4) -> (SolidId, SolidId) {
    let sphere = make_sphere(topo, pair.sphere_radius * scale, 16).unwrap();
    let torus = make_torus(topo, pair.major * scale, pair.minor * scale, 16).unwrap();
    for (solid, [x, y, z]) in [(sphere, pair.sphere_center), (torus, pair.torus_center)] {
        let to_seed = place * Mat4::translation(x * scale, y * scale, z * scale);
        transform_solid(topo, solid, &to_seed).unwrap();
    }
    (sphere, torus)
}

fn exact() -> OperationContext {
    OperationContext::new().with_fallback(FallbackPolicy::ExactOnly)
}

/// Every edge of the solid is used by exactly two coedges running opposite
/// ways (each effective direction composes the face's reversal).
fn assert_opposing_coedges(topo: &Topology, solid: SolidId, label: &str) {
    let mut uses = BTreeMap::new();
    for fid in remus_topology::explorer::solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for edge in topo.wire(wid).unwrap().edges() {
                uses.entry(edge.edge())
                    .or_insert_with(Vec::new)
                    .push(edge.is_forward() != face.is_reversed());
            }
        }
    }
    assert!(!uses.is_empty(), "{label}: no edges");
    for (edge, directions) in uses {
        assert_eq!(directions.len(), 2, "{label}: incidence {edge:?}");
        assert_ne!(directions[0], directions[1], "{label}: same-sense {edge:?}");
    }
}

fn assert_strictly_valid(topo: &Topology, solid: SolidId, label: &str) {
    let validation = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(validation.is_valid(), "{label}: {:?}", validation.issues);
    let validation = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(
        !validation
            .issues
            .iter()
            .any(|i| i.severity == remus_check::validate::Severity::Error),
        "{label}: {:?}",
        validation.issues
    );
    assert_opposing_coedges(topo, solid, label);
}

/// Expected material of each operator for points inside the sphere and/or
/// the torus.
fn member(op: BooleanOp, sphere_first: bool, in_sphere: bool, in_torus: bool) -> bool {
    match op {
        BooleanOp::Fuse => in_sphere || in_torus,
        BooleanOp::Intersect => in_sphere && in_torus,
        BooleanOp::Cut if sphere_first => in_sphere && !in_torus,
        BooleanOp::Cut => in_torus && !in_sphere,
    }
}

fn expected_volume(pair: Pair, op: BooleanOp, sphere_first: bool) -> f64 {
    let overlap = intersection_volume(pair);
    match op {
        BooleanOp::Fuse => sphere_volume(pair) + torus_volume(pair) - overlap,
        BooleanOp::Intersect => overlap,
        BooleanOp::Cut if sphere_first => sphere_volume(pair) - overlap,
        BooleanOp::Cut => torus_volume(pair) - overlap,
    }
}

/// Cartesian membership of a lattice through both operands, from the
/// sphere and torus inequalities alone, against the kernel's classifiers.
/// The winding number of `closed_mesh` (a raw mesh already held closed), or
/// the robust classifier where the mesh is not closed, must agree
/// everywhere; the count of ray-vote disagreements is returned for the
/// caller to hold.
#[allow(clippy::too_many_arguments)]
fn assert_material(
    topo: &Topology,
    solid: SolidId,
    pair: Pair,
    scale: f64,
    place: Mat4,
    op: BooleanOp,
    sphere_first: bool,
    closed_mesh: Option<&TriangleMesh>,
    label: &str,
) -> usize {
    let [sx, sy, sz] = pair.sphere_center;
    let [tx, ty, tz] = pair.torus_center;
    let lo = [
        (sx - pair.sphere_radius).min(tx - pair.major - pair.minor),
        (sy - pair.sphere_radius).min(ty - pair.major - pair.minor),
        (sz - pair.sphere_radius).min(tz - pair.minor),
    ];
    let hi = [
        (sx + pair.sphere_radius).max(tx + pair.major + pair.minor),
        (sy + pair.sphere_radius).max(ty + pair.major + pair.minor),
        (sz + pair.sphere_radius).max(tz + pair.minor),
    ];
    // A point this close to either surface is skipped: classification on the
    // boundary is not the material question.
    let margin = 0.08;
    let options = ClassifyOptions::default();
    let mut coverage = [0_usize; 4];
    let mut ray_mismatches = 0;
    let n = 9_u32;
    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                // Off-lattice offsets keep samples off symmetry planes.
                let f = |axis: usize, step: u32| {
                    lo[axis] + (hi[axis] - lo[axis]) * (f64::from(step) + 0.43) / f64::from(n)
                };
                let (x, y, z) = (f(0, i), f(1, j), f(2, k));
                let sphere_gap = ((x - sx).powi(2) + (y - sy).powi(2) + (z - sz).powi(2)).sqrt()
                    - pair.sphere_radius;
                let ring = (x - tx).hypot(y - ty) - pair.major;
                let torus_gap = ring.hypot(z - tz) - pair.minor;
                if sphere_gap.abs() < margin || torus_gap.abs() < margin {
                    continue;
                }
                let (in_sphere, in_torus) = (sphere_gap < 0.0, torus_gap < 0.0);
                coverage[2 * usize::from(in_sphere) + usize::from(in_torus)] += 1;
                let point = place.mul_point(Point3::new(x * scale, y * scale, z * scale));
                let expected = if member(op, sphere_first, in_sphere, in_torus) {
                    PointClassification::Inside
                } else {
                    PointClassification::Outside
                };
                // The mesh winding number reads the material independently
                // of the face trims; the ray vote reads the trims themselves.
                let inside = if let Some(mesh) = closed_mesh {
                    winding_number(mesh, point) > 0.5
                } else {
                    matches!(
                        remus_operations::classify::classify_point_robust(
                            topo,
                            solid,
                            point,
                            0.01 * scale,
                            1e-7 * scale.max(1.0),
                        )
                        .unwrap(),
                        remus_operations::classify::PointClassification::Inside
                    )
                };
                assert_eq!(
                    inside,
                    expected == PointClassification::Inside,
                    "{label}: winding membership at ({x}, {y}, {z})"
                );
                if classify_point(topo, solid, point, &options).unwrap() != expected {
                    ray_mismatches += 1;
                }
            }
        }
    }
    assert!(
        coverage.into_iter().all(|count| count > 0),
        "{label}: lattice must reach every membership cell: {coverage:?}"
    );
    ray_mismatches
}

fn check(pair: Pair, op: BooleanOp, sphere_first: bool, scale: f64, placed: bool) {
    let label = format!(
        "{} {op:?} sphere_first={sphere_first} scale={scale} placed={placed}",
        pair.name
    );
    let place = placement(placed);
    let mut topo = Topology::new();
    let (sphere, torus) = build(&mut topo, pair, scale, place);
    let operands = remus_io::arena_io::serialize_solids(&topo, &[sphere, torus]).unwrap();
    let (a, b) = if sphere_first {
        (sphere, torus)
    } else {
        (torus, sphere)
    };
    let result = boolean_with_context(&mut topo, op, a, b, &exact())
        .unwrap_or_else(|e| panic!("{label}: {e:?}"));
    assert!(matches!(result.quality, BooleanQuality::Exact), "{label}");
    let solid = result.solid;
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[sphere, torus]).unwrap(),
        operands,
        "{label}: operands must be preserved"
    );

    // Both carriers survive exactly; no mesh or NURBS-surface fallback.
    let mut carriers = [0_usize; 2];
    for fid in remus_topology::explorer::solid_faces(&topo, solid).unwrap() {
        match topo.face(fid).unwrap().surface() {
            FaceSurface::Sphere(_) => carriers[0] += 1,
            FaceSurface::Torus(_) => carriers[1] += 1,
            other => panic!("{label}: unexpected {} carrier", other.type_tag()),
        }
    }
    assert!(carriers[0] > 0 && carriers[1] > 0, "{label}: {carriers:?}");

    assert_strictly_valid(&topo, solid, &label);

    let target = expected_volume(pair, op, sphere_first);
    let volume = solid_volume(&topo, solid, 0.001 * scale).unwrap() / scale.powi(3);
    assert!(
        (volume - target).abs() < 2e-4 * target,
        "{label}: volume {volume} against slice integral {target}"
    );

    // Raw (unwelded) mesh closure at two deflections, on every leg. Seed
    // 1146's fuse and sphere − torus cut keep one sphere face with two
    // section holes; #1546 meshes it closed.
    let mut closed_mesh = None;
    for deflection in [0.1, 0.02] {
        let mesh = tessellate_solid(&topo, solid, deflection * scale).unwrap();
        assert_eq!(
            boundary_edge_count(&mesh),
            0,
            "{label}: raw mesh boundary at {deflection}"
        );
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "{label}: raw mesh non-manifold at {deflection}"
        );
        closed_mesh = Some(mesh);
    }

    let ray_mismatches = assert_material(
        &topo,
        solid,
        pair,
        scale,
        place,
        op,
        sphere_first,
        closed_mesh.as_ref(),
        &label,
    );
    if !ray_pole_piece_cell(pair, op, sphere_first, placed) {
        assert_eq!(ray_mismatches, 0, "{label}: ray-vote membership");
    }

    // Operands are still the closed-form primitives they were built as.
    for (solid, expected, name) in [
        (sphere, sphere_volume(pair), "sphere"),
        (torus, torus_volume(pair), "torus"),
    ] {
        assert_strictly_valid(&topo, solid, &format!("{label}: {name} operand"));
        let volume = solid_volume(&topo, solid, 0.001 * scale).unwrap() / scale.powi(3);
        assert!(
            (volume - expected).abs() < 2e-4 * expected,
            "{label}: {name} operand volume {volume}"
        );
    }
}

/// The ray vote still miscounts a few probes on the rigidly placed seed-5201
/// fuse and sphere − torus cut, whose kept sphere piece holds a pole: its trim
/// cannot be read in the sphere's chart, so it falls back to the 3D polygon
/// projection (B264, identical on the unpatched kernel). The winding
/// classifier, validity and volume hold there; the ray vote is witnessed by
/// `ray_vote_pole_piece_witness`.
fn ray_pole_piece_cell(pair: Pair, op: BooleanOp, sphere_first: bool, placed: bool) -> bool {
    pair.name == "seed5201"
        && placed
        && (op == BooleanOp::Fuse || (op == BooleanOp::Cut && sphere_first))
}

/// What the kernel does with one cell of the matrix.
#[derive(Debug, PartialEq, Eq)]
enum Outcome {
    /// Exact, valid, and held to every oracle in [`check`].
    Exact,
    /// A typed exact-only refusal that leaves the arena untouched.
    Refused,
    /// An open B38 cell, witnessed by an ignored test instead.
    Open,
}

/// The declared domain. Unit scale is qualified, apart from three placed
/// seed-1146 cells the GFA refuses; the 1e-3 and 1e3 scales refuse, apart
/// from seed 1146's small-scale fuse and sphere − torus cut, which return
/// wrong bodies (B38's small-scale class, `small_scale_seed1146_witness`).
/// None of these limits moved with the orientation repair.
fn outcome(pair: Pair, op: BooleanOp, sphere_first: bool, scale: f64, placed: bool) -> Outcome {
    let seed1146 = pair.name == "seed1146";
    let keeps_outer_sphere = op == BooleanOp::Fuse || (op == BooleanOp::Cut && sphere_first);
    if (scale - 1.0).abs() < f64::EPSILON {
        if seed1146 && placed && keeps_outer_sphere {
            Outcome::Refused
        } else {
            Outcome::Exact
        }
    } else if seed1146 && scale < 1.0 && keeps_outer_sphere {
        Outcome::Open
    } else {
        Outcome::Refused
    }
}

/// A refused cell: typed, with every operand and arena count unchanged.
fn check_refused(pair: Pair, op: BooleanOp, sphere_first: bool, scale: f64, placed: bool) {
    let label = format!(
        "{} {op:?} sphere_first={sphere_first} scale={scale} placed={placed}",
        pair.name
    );
    let mut topo = Topology::new();
    let (sphere, torus) = build(&mut topo, pair, scale, placement(placed));
    let operands = remus_io::arena_io::serialize_solids(&topo, &[sphere, torus]).unwrap();
    let counts = topology_counts(&topo);
    let (a, b) = if sphere_first {
        (sphere, torus)
    } else {
        (torus, sphere)
    };
    let error = boolean_with_context(&mut topo, op, a, b, &exact()).unwrap_err();
    assert!(
        matches!(
            error,
            remus_operations::OperationsError::ExactOnlyUnattainable
        ),
        "{label}: {error:?}"
    );
    assert_eq!(topology_counts(&topo), counts, "{label}: rollback");
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[sphere, torus]).unwrap(),
        operands,
        "{label}: refusal must preserve the operands"
    );
}

fn matrix(pair: Pair) {
    for scale in [1.0, 0.001, 1000.0] {
        for placed in [false, true] {
            for op in [BooleanOp::Intersect, BooleanOp::Fuse, BooleanOp::Cut] {
                for sphere_first in [true, false] {
                    match outcome(pair, op, sphere_first, scale, placed) {
                        Outcome::Exact => check(pair, op, sphere_first, scale, placed),
                        Outcome::Refused => check_refused(pair, op, sphere_first, scale, placed),
                        Outcome::Open => {}
                    }
                }
            }
        }
    }
}

/// B38 small-scale class: at 1e-3, seed 1146's fuse and sphere − torus cut
/// return Exact bodies with wrong material (same-sense edges, a 4 % volume
/// error, winding probes outside the lattice's expected cells). Unchanged by
/// the orientation repair. Remove the ignore once the row fixes it.
#[test]
#[ignore = "open: seed-1146 sphere-torus fuse and sphere cut misbuild at 1e-3 (B38)"]
fn small_scale_seed1146_witness() {
    for placed in [false, true] {
        for (op, sphere_first) in [
            (BooleanOp::Fuse, true),
            (BooleanOp::Fuse, false),
            (BooleanOp::Cut, true),
        ] {
            check(SEED_1146, op, sphere_first, 0.001, placed);
        }
    }
}

/// B264: the ray vote miscounts probes of the placed seed-5201 fuse and
/// sphere − torus cut (a pole-holding sphere piece read through the 3D
/// polygon fallback). Remove the ignore once the classifier reads it.
#[test]
#[ignore = "open: ray-vote trim of a pole-holding sphere piece miscounts (B264)"]
fn ray_vote_pole_piece_witness() {
    for (op, sphere_first) in [
        (BooleanOp::Fuse, true),
        (BooleanOp::Fuse, false),
        (BooleanOp::Cut, true),
    ] {
        let place = placement(true);
        let mut topo = Topology::new();
        let (sphere, torus) = build(&mut topo, SEED_5201, 1.0, place);
        let (a, b) = if sphere_first {
            (sphere, torus)
        } else {
            (torus, sphere)
        };
        let solid = boolean_with_context(&mut topo, op, a, b, &exact())
            .unwrap()
            .solid;
        let label = format!("seed5201 placed {op:?} sphere_first={sphere_first}");
        let mesh = tessellate_solid(&topo, solid, 0.02).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "{label}");
        let mismatches = assert_material(
            &topo,
            solid,
            SEED_5201,
            1.0,
            place,
            op,
            sphere_first,
            Some(&mesh),
            &label,
        );
        assert_eq!(mismatches, 0, "{label}: ray-vote membership");
    }
}

#[test]
fn seed5201_equator_crescent_keeps_its_winding() {
    matrix(SEED_5201);
}

#[test]
fn seed1146_closed_disc_rims_oppose_the_band() {
    matrix(SEED_1146);
}

/// The persisted chains themselves, mirrors included: seed 5201 mirrors the
/// sphere about y = 1.5 and seed 1146 mirrors the torus about x = -4 before
/// moving it. Each identity-leg operator in each order must validate with
/// opposing coedges and close inclusion–exclusion.
#[test]
fn persisted_mirror_chains_validate_every_leg() {
    for (pair, mirrored_sphere) in [(SEED_5201, true), (SEED_1146, false)] {
        let mut legs = BTreeMap::new();
        for op in [BooleanOp::Fuse, BooleanOp::Intersect, BooleanOp::Cut] {
            for sphere_first in [true, false] {
                let mut topo = Topology::new();
                let (sphere, torus) = if mirrored_sphere {
                    let sphere = make_sphere(&mut topo, 5.0, 16).unwrap();
                    let torus = make_torus(&mut topo, 5.5, 2.75, 16).unwrap();
                    let image = mirror(
                        &mut topo,
                        sphere,
                        Point3::new(0.0, 1.5, 0.0),
                        Vec3::new(0.0, 1.0, 0.0),
                    )
                    .unwrap();
                    (image, torus)
                } else {
                    let torus = make_torus(&mut topo, 4.0, 2.0, 16).unwrap();
                    let sphere = make_sphere(&mut topo, 8.0, 16).unwrap();
                    let image = mirror(
                        &mut topo,
                        torus,
                        Point3::new(-4.0, 0.0, 0.0),
                        Vec3::new(1.0, 0.0, 0.0),
                    )
                    .unwrap();
                    transform_solid(&mut topo, image, &Mat4::translation(1.0, 0.0, -3.0)).unwrap();
                    (sphere, image)
                };
                let label = format!("{} chain {op:?} sphere_first={sphere_first}", pair.name);
                let (a, b) = if sphere_first {
                    (sphere, torus)
                } else {
                    (torus, sphere)
                };
                let result = boolean_with_context(&mut topo, op, a, b, &exact())
                    .unwrap_or_else(|e| panic!("{label}: {e:?}"));
                assert!(matches!(result.quality, BooleanQuality::Exact), "{label}");
                assert_strictly_valid(&topo, result.solid, &label);
                let volume = solid_volume(&topo, result.solid, 0.001).unwrap();
                let target = expected_volume(pair, op, sphere_first);
                assert!(
                    (volume - target).abs() < 2e-4 * target,
                    "{label}: volume {volume} against {target}"
                );
                legs.insert((format!("{op:?}"), sphere_first), volume);
            }
        }
        // F + I = A + B, and each cut is its operand less the intersection.
        let fuse = legs[&("Fuse".to_owned(), true)];
        let inter = legs[&("Intersect".to_owned(), true)];
        let total = sphere_volume(pair) + torus_volume(pair);
        assert!(
            (fuse + inter - total).abs() < 2e-4 * total,
            "{}: inclusion–exclusion {fuse} + {inter} against {total}",
            pair.name
        );
        let sphere_cut = legs[&("Cut".to_owned(), true)];
        let torus_cut = legs[&("Cut".to_owned(), false)];
        assert!(
            (sphere_cut + inter - sphere_volume(pair)).abs() < 2e-4 * sphere_volume(pair),
            "{}: sphere complement",
            pair.name
        );
        assert!(
            (torus_cut + inter - torus_volume(pair)).abs() < 2e-4 * torus_volume(pair),
            "{}: torus complement",
            pair.name
        );
    }
}

/// Refusals leave the arena exactly as it was: a cancelled operation, an
/// exhausted work budget, and seed 5201's own refused fifth step (the fused
/// body intersected with the unmirrored sphere, outside the qualified
/// domain).
#[test]
fn refusals_roll_back_and_preserve_operands() {
    for pair in [SEED_5201, SEED_1146] {
        for op in [BooleanOp::Intersect, BooleanOp::Fuse, BooleanOp::Cut] {
            let mut topo = Topology::new();
            let (sphere, torus) = build(&mut topo, pair, 1.0, Mat4::identity());
            let before = remus_io::arena_io::serialize_solids(&topo, &[sphere, torus]).unwrap();
            let before_counts = topology_counts(&topo);
            let token = CancellationToken::new();
            token.cancel();
            let contexts = [
                (OperationContext::new().with_cancellation(token), true),
                (
                    OperationContext::new().with_budgets(WorkBudgets::new().with_vertex_pairs(0)),
                    false,
                ),
            ];
            for (context, cancelled) in contexts {
                let error = boolean_with_context(
                    &mut topo,
                    op,
                    sphere,
                    torus,
                    &context.with_fallback(FallbackPolicy::ExactOnly),
                )
                .unwrap_err();
                if cancelled {
                    assert!(
                        matches!(
                            error,
                            remus_operations::OperationsError::Math(
                                remus_math::MathError::Cancelled
                            )
                        ),
                        "{} {op:?}: {error:?}",
                        pair.name
                    );
                } else {
                    assert!(
                        matches!(
                            error,
                            remus_operations::OperationsError::Algo(
                                remus_algo::error::AlgoError::ResourceLimitExceeded {
                                    resource: "GFA vertex pairs",
                                    limit: 0,
                                    actual: _
                                }
                            )
                        ),
                        "{} {op:?}: {error:?}",
                        pair.name
                    );
                }
                assert_eq!(topology_counts(&topo), before_counts, "{}", pair.name);
                assert_eq!(
                    remus_io::arena_io::serialize_solids(&topo, &[sphere, torus]).unwrap(),
                    before,
                    "{} {op:?}: refusal must preserve the operands",
                    pair.name
                );
            }
        }
    }

    // Seed 5201's last step: fuse(mirrored sphere, torus) ∩ original sphere.
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, 5.0, 16).unwrap();
    let torus = make_torus(&mut topo, 5.5, 2.75, 16).unwrap();
    let image = mirror(
        &mut topo,
        sphere,
        Point3::new(0.0, 1.5, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let fused = boolean_with_context(&mut topo, BooleanOp::Fuse, image, torus, &exact())
        .unwrap()
        .solid;
    let before = remus_io::arena_io::serialize_solids(&topo, &[fused, sphere]).unwrap();
    let before_counts = topology_counts(&topo);
    let error =
        boolean_with_context(&mut topo, BooleanOp::Intersect, fused, sphere, &exact()).unwrap_err();
    assert!(
        matches!(
            error,
            remus_operations::OperationsError::ExactOnlyUnattainable
        ),
        "{error:?}"
    );
    assert_eq!(topology_counts(&topo), before_counts);
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[fused, sphere]).unwrap(),
        before
    );
}
