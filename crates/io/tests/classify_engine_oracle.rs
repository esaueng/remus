//! Differential oracle for the two live point-in-solid classifiers (RFC 0008
//! stage 0, method in its section 2.2): `remus-algo`'s
//! `classify_point_cached` and `remus-check`'s `PreparedSolid::classify_point`
//! are each scored against watertight ray–triangle parity on a fine
//! tessellation of the same body.
//!
//! * Bodies: every solid of every `tests/data/*.step` fixture, plus box,
//!   cylinder, cone, sphere, torus, box − cylinder, box ∪ sphere and
//!   box ∩ torus. A body whose fine mesh exceeds [`MAX_TRIANGLES`] is skipped.
//! * Points: a 7³ grid over the bounding box inset by half a cell, plus
//!   near-surface probes: centroids of evenly strided mesh triangles offset
//!   by ±1e-3 and ±2e-4 × the diagonal along the facet normal.
//! * Oracle: parity of watertight ray–triangle hits on a mesh at deflection
//!   2e-5 × the diagonal, along five irrational directions. A point counts
//!   only when all five agree and it lies farther than [`NEAR_MESH`]
//!   deflections from the mesh (a grid point can land on the true surface,
//!   where the inscribed mesh cannot decide it).
//!
//! Each engine's per-body wrong counts are pinned in [`PINNED`]. They may only
//! go down: a count above its pin fails, a count below it is reported so the
//! pin can be lowered. `OnBoundary`, `On`/coplanar classes and errors count as
//! wrong, since every probe is off the surface by far more than any
//! classification tolerance.
//!
//! The default run covers [`FAST_BODIES`] (the bodies of RFC 0008's F1, F2
//! and F4). The whole corpus is `#[ignore]`d for runtime:
//!
//! ```text
//! cargo test --release -p remus-io --test classify_engine_oracle -- --ignored
//! ```
//!
//! Flip analysis (corpus test only): set `CLASSIFY_ORACLE_DUMP=<file>` to
//! write every scored verdict, and `CLASSIFY_ORACLE_BASELINE=<file>` to
//! compare against an earlier dump; any point an engine had right in the
//! baseline and gets wrong now fails the run.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stderr,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::too_many_lines,
    clippy::approx_constant,
    deprecated
)]

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::Path;

use remus_algo::FaceClass;
use remus_algo::classifier::{RayCastGeoms, classify_point_cached};
use remus_check::classify::{ClassifyOptions, PointClassification, PreparedSolid};
use remus_math::aabb::Aabb3;
use remus_math::bvh::Bvh;
use remus_math::mat::Mat4;
use remus_math::ray_triangle::watertight_ray_triangle_intersect;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
use remus_operations::tessellate::{TriangleMesh, tessellate_solid};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// Fine meshes above this many triangles are skipped (RFC 0008 section 2.2).
const MAX_TRIANGLES: usize = 400_000;

/// Points closer to the fine mesh than this many deflections are ambiguous.
const NEAR_MESH: f64 = 3.0;

/// Grid points per axis.
const GRID: usize = 7;

/// Mesh triangles sampled for near-surface probes; each gives four points.
const NEAR_TRIANGLES: usize = 75;

/// Pinned wrong counts per body: `(name, algo, check)`. Lower a pin when the
/// run reports a count below it; never raise one.
const PINNED: &[(&str, usize, usize)] = &[
    ("axis2_optional_attrs_cylinder.step#0", 0, 19),
    ("conical_surface_base_radius_frustum.step#0", 0, 4),
    ("jolly_fox_partial_cylinder.step#0", 0, 0),
    ("lemon-torus-band.step#0", 361, 0),
    ("lipfuse_2x1_body.step#0", 0, 0),
    ("lipfuse_2x1_lip.step#0", 0, 0),
    ("lipfuse_3x3_body.step#0", 0, 0),
    ("lipfuse_3x3_lip.step#0", 0, 0),
    ("lipfuse_body.step#0", 0, 0),
    ("lipfuse_lip.step#0", 0, 0),
    ("multicavity_bin_box.step#0", 0, 0),
    ("multicavity_cavity_0.step#0", 0, 0),
    ("multicavity_cavity_1.step#0", 0, 0),
    ("openzcad_a_export_bored_plate.step#0", 0, 0),
    ("openzcad_e_analytic_fillet_plate.step#0", 0, 0),
    ("openzcad_e_nurbs_fillet_plate.step#0", 150, 6),
    ("oring_nested_holes.step#0", 0, 5),
    ("scale_platform_v1_linear_spline_edge.step#0", 0, 0),
    ("scoop_bin_box.step#0", 0, 0),
    ("scoop_cavity_0.step#0", 0, 0),
    ("scoop_cavity_1.step#0", 0, 0),
    ("scoop_scoop_0.step#0", 0, 0),
    ("shapr3d_hammer_holder.step#0", 51, 0),
    ("shapr3d_walking_stick_foot.step#0", 220, 0),
    ("shapr_untrimmed_nurbs_domain.step#0", 205, 0),
    ("wallcut_body.step#0", 0, 0),
    ("wallcut_tool_0.step#0", 0, 0),
    ("box", 0, 0),
    ("cylinder", 0, 18),
    ("cone", 0, 15),
    ("sphere", 0, 0),
    ("box_minus_cylinder", 0, 0),
    ("box_union_sphere", 188, 108),
    ("box_intersect_torus", 390, 0),
];

/// Bodies scored by the default (non-ignored) run.
const FAST_BODIES: &[&str] = &[
    "cylinder",
    "cone",
    "box_union_sphere",
    "axis2_optional_attrs_cylinder.step#0",
    "oring_nested_holes.step#0",
];

type Body = (String, Topology, SolidId);

fn data_dir() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

/// Every solid of every STEP fixture, or only those `keep` accepts.
fn step_bodies(keep: &dyn Fn(&str) -> bool) -> Vec<Body> {
    let mut files: Vec<_> = std::fs::read_dir(data_dir())
        .unwrap()
        .map(|entry| entry.unwrap().file_name().into_string().unwrap())
        .filter(|name| {
            Path::new(name)
                .extension()
                .is_some_and(|ext| ext.eq_ignore_ascii_case("step"))
        })
        .collect();
    files.sort();
    let mut bodies = Vec::new();
    for file in files {
        if !keep(&format!("{file}#0")) && !keep(&file) {
            continue;
        }
        let text = std::fs::read_to_string(data_dir().join(&file)).unwrap();
        let mut topo = Topology::new();
        let Ok(solids) = remus_io::step::reader::read_step(&text, &mut topo) else {
            continue;
        };
        for (k, solid) in solids.into_iter().enumerate() {
            let name = format!("{file}#{k}");
            if keep(&name) {
                bodies.push((name, topo.clone(), solid));
            }
        }
    }
    bodies
}

fn translated(topo: &mut Topology, solid: SolidId, x: f64, y: f64, z: f64) {
    transform_solid(topo, solid, &Mat4::translation(x, y, z)).unwrap();
}

fn synthetic_bodies(keep: &dyn Fn(&str) -> bool) -> Vec<Body> {
    let mut bodies = Vec::new();
    let mut push = |name: &str, build: &dyn Fn(&mut Topology) -> SolidId| {
        if !keep(name) {
            return;
        }
        let mut topo = Topology::new();
        let solid = build(&mut topo);
        bodies.push((name.to_string(), topo, solid));
    };
    push("box", &|t| make_box(t, 10.0, 8.0, 6.0).unwrap());
    push("cylinder", &|t| make_cylinder(t, 3.0, 8.0).unwrap());
    push("cone", &|t| make_cone(t, 3.0, 1.0, 6.0).unwrap());
    push("sphere", &|t| make_sphere(t, 5.0, 16).unwrap());
    push("torus", &|t| make_torus(t, 3.0, 1.0, 16).unwrap());
    push("box_minus_cylinder", &|t| {
        let plate = make_box(t, 10.0, 10.0, 4.0).unwrap();
        let drill = make_cylinder(t, 2.0, 6.0).unwrap();
        translated(t, drill, 5.0, 5.0, -1.0);
        boolean(t, BooleanOp::Cut, plate, drill).unwrap()
    });
    // RFC 0008 F2: the sphere protrudes through three box faces.
    push("box_union_sphere", &|t| {
        let block = make_box(t, 20.0, 20.0, 10.0).unwrap();
        let ball = make_sphere(t, 6.0, 16).unwrap();
        translated(t, ball, 20.0, 10.0, 5.0);
        boolean(t, BooleanOp::Fuse, block, ball).unwrap()
    });
    push("box_intersect_torus", &|t| {
        let ring = make_torus(t, 10.0, 3.0, 32).unwrap();
        let block = make_box(t, 8.0, 8.0, 8.0).unwrap();
        translated(t, block, 6.0, -4.0, -4.0);
        boolean(t, BooleanOp::Intersect, block, ring).unwrap()
    });
    bodies
}

/// Five irrational oracle directions, none along a coordinate axis or plane.
fn oracle_directions() -> [Vec3; 5] {
    [
        Vec3::new(0.577_215_664_9, 0.693_147_180_6, 0.318_309_886_2).normalize(),
        Vec3::new(-0.414_213_562_4, 0.732_050_807_6, -0.236_067_977_5).normalize(),
        Vec3::new(0.301_029_995_7, -0.618_033_988_7, 0.707_106_781_2).normalize(),
        Vec3::new(-0.866_025_403_8, -0.267_949_192_4, 0.434_294_481_9).normalize(),
        Vec3::new(0.141_421_356_2, 0.173_205_080_8, -0.974_679_434_5).normalize(),
    ]
    .map(Result::unwrap)
}

struct MeshOracle<'a> {
    mesh: &'a TriangleMesh,
    bvh: Bvh,
    directions: [Vec3; 5],
    /// Distance below which a point is too close to the mesh to decide.
    near: f64,
}

impl<'a> MeshOracle<'a> {
    fn new(mesh: &'a TriangleMesh, near: f64) -> Self {
        let boxes: Vec<(usize, Aabb3)> = (0..mesh.indices.len() / 3)
            .map(|k| (k, Aabb3::from_points(Self::triangle(mesh, k))))
            .collect();
        Self {
            mesh,
            bvh: Bvh::build(&boxes),
            directions: oracle_directions(),
            near,
        }
    }

    fn is_near(&self, point: Point3, scratch: &mut Vec<usize>) -> bool {
        let probe = Aabb3::from_points([point]).expanded(self.near);
        self.bvh.query_overlap_into(&probe, scratch);
        scratch.iter().any(|&k| {
            let [a, b, c] = Self::triangle(self.mesh, k);
            (closest_point_on_triangle(point, a, b, c) - point).length() < self.near
        })
    }

    fn triangle(mesh: &TriangleMesh, k: usize) -> [Point3; 3] {
        [0, 1, 2].map(|i| mesh.positions[mesh.indices[3 * k + i] as usize])
    }

    /// `Some(inside)` when the point is clear of the mesh and all five ray
    /// parities agree.
    fn classify(&self, point: Point3, scratch: &mut Vec<usize>) -> Option<bool> {
        if self.is_near(point, scratch) {
            return None;
        }
        let mut verdict = None;
        for direction in self.directions {
            self.bvh.query_ray_into(point, direction, scratch);
            let hits = scratch
                .iter()
                .filter(|&&k| {
                    let [a, b, c] = Self::triangle(self.mesh, k);
                    watertight_ray_triangle_intersect(point, direction, a, b, c)
                        .is_some_and(|hit| hit.t > 0.0)
                })
                .count();
            let inside = hits % 2 == 1;
            match verdict {
                None => verdict = Some(inside),
                Some(v) if v != inside => return None,
                Some(_) => {}
            }
        }
        verdict
    }
}

/// Closest point on a triangle (Ericson, Real-Time Collision Detection,
/// 5.1.5).
#[allow(clippy::many_single_char_names)]
fn closest_point_on_triangle(p: Point3, a: Point3, b: Point3, c: Point3) -> Point3 {
    let (ab, ac, ap) = (b - a, c - a, p - a);
    let (d1, d2) = (ab.dot(ap), ac.dot(ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p - b;
    let (d3, d4) = (ab.dot(bp), ac.dot(bp));
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1.mul_add(d4, -(d3 * d2));
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return a + ab * (d1 / (d1 - d3));
    }
    let cp = p - c;
    let (d5, d6) = (ab.dot(cp), ac.dot(cp));
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5.mul_add(d2, -(d1 * d6));
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return a + ac * (d2 / (d2 - d6));
    }
    let va = d3.mul_add(d6, -(d5 * d4));
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let denom = 1.0 / (va + vb + vc);
    a + ab * (vb * denom) + ac * (vc * denom)
}

fn mesh_bounds(mesh: &TriangleMesh) -> Aabb3 {
    Aabb3::from_points(mesh.positions.iter().copied())
}

/// The 7³ inset grid and the near-surface probes.
fn probe_points(mesh: &TriangleMesh, bounds: Aabb3, diagonal: f64) -> Vec<Point3> {
    let mut points = Vec::new();
    let extent = bounds.max - bounds.min;
    for i in 0..GRID {
        for j in 0..GRID {
            for k in 0..GRID {
                let f = |n: usize| (n as f64 + 0.5) / GRID as f64;
                points.push(Point3::new(
                    extent.x().mul_add(f(i), bounds.min.x()),
                    extent.y().mul_add(f(j), bounds.min.y()),
                    extent.z().mul_add(f(k), bounds.min.z()),
                ));
            }
        }
    }
    let triangles = mesh.indices.len() / 3;
    let stride = (triangles / NEAR_TRIANGLES).max(1);
    for k in (stride / 2..triangles).step_by(stride).take(NEAR_TRIANGLES) {
        let [a, b, c] = MeshOracle::triangle(mesh, k);
        let Ok(normal) = (b - a).cross(c - a).normalize() else {
            continue;
        };
        let centroid = Point3::new(
            (a.x() + b.x() + c.x()) / 3.0,
            (a.y() + b.y() + c.y()) / 3.0,
            (a.z() + b.z() + c.z()) / 3.0,
        );
        for offset in [1e-3, -1e-3, 2e-4, -2e-4] {
            points.push(centroid + normal * (offset * diagonal));
        }
    }
    points
}

/// One scored point: oracle verdict, and whether each engine agreed.
struct Verdict {
    point: Point3,
    inside: bool,
    algo_right: bool,
    check_right: bool,
}

struct BodyScore {
    points: usize,
    ambiguous: usize,
    verdicts: Vec<Verdict>,
}

impl BodyScore {
    fn algo_wrong(&self) -> usize {
        self.verdicts.iter().filter(|v| !v.algo_right).count()
    }
    fn check_wrong(&self) -> usize {
        self.verdicts.iter().filter(|v| !v.check_right).count()
    }
}

/// `None` when the body is skipped (mesh failed or too fine).
fn score_body(name: &str, topo: &Topology, solid: SolidId) -> Option<BodyScore> {
    let coarse = tessellate_solid(topo, solid, 0.1).ok()?;
    let bounds = mesh_bounds(&coarse);
    let diagonal = (bounds.max - bounds.min).length();
    let deflection = 2e-5 * diagonal;
    let mesh = match tessellate_solid(topo, solid, deflection) {
        Ok(mesh) => mesh,
        Err(error) => {
            eprintln!("{name}: skipped, fine mesh failed: {error}");
            return None;
        }
    };
    if mesh.indices.len() / 3 > MAX_TRIANGLES {
        eprintln!(
            "{name}: skipped, fine mesh has {} triangles",
            mesh.indices.len() / 3
        );
        return None;
    }
    let oracle = MeshOracle::new(&mesh, NEAR_MESH * deflection);
    let prepared = PreparedSolid::prepare(topo, solid).unwrap();
    let geoms = RayCastGeoms::new(topo, solid).ok();
    let options = ClassifyOptions::default();
    let points = probe_points(&mesh, mesh_bounds(&mesh), diagonal);
    let mut scratch = Vec::new();
    let mut score = BodyScore {
        points: points.len(),
        ambiguous: 0,
        verdicts: Vec::with_capacity(points.len()),
    };
    for point in points {
        let Some(inside) = oracle.classify(point, &mut scratch) else {
            score.ambiguous += 1;
            score.verdicts.push(Verdict {
                point,
                inside: false,
                algo_right: true,
                check_right: true,
            });
            continue;
        };
        let algo = classify_point_cached(topo, solid, geoms.as_ref(), point);
        let check = prepared.classify_point(point, &options);
        let algo_right = match algo {
            Ok(FaceClass::Inside) => inside,
            Ok(FaceClass::Outside) => !inside,
            _ => false,
        };
        let check_right = match check {
            Ok(PointClassification::Inside) => inside,
            Ok(PointClassification::Outside) => !inside,
            _ => false,
        };
        score.verdicts.push(Verdict {
            point,
            inside,
            algo_right,
            check_right,
        });
    }
    Some(score)
}

/// Per-point lines `body index oracle algo_right check_right x y z`.
fn dump(scores: &BTreeMap<String, BodyScore>) -> String {
    let mut text = String::new();
    for (name, score) in scores {
        for (k, v) in score.verdicts.iter().enumerate() {
            writeln!(
                text,
                "{name}\t{k}\t{}\t{}\t{}\t{:?}\t{:?}\t{:?}",
                u8::from(v.inside),
                u8::from(v.algo_right),
                u8::from(v.check_right),
                v.point.x(),
                v.point.y(),
                v.point.z()
            )
            .unwrap();
        }
    }
    text
}

/// Right→wrong flips against a baseline dump, per engine.
fn flips_against(baseline: &str, scores: &BTreeMap<String, BodyScore>) -> Vec<String> {
    let mut flips = Vec::new();
    let mut fixed = (0usize, 0usize);
    for line in baseline.lines() {
        let fields: Vec<&str> = line.split('\t').collect();
        let [name, index, _, algo, check, ..] = fields[..] else {
            continue;
        };
        let Some(score) = scores.get(name) else {
            continue;
        };
        let Some(now) = score.verdicts.get(index.parse::<usize>().unwrap()) else {
            continue;
        };
        if algo == "1" && !now.algo_right {
            flips.push(format!("{name}#{index}: algo right -> wrong"));
        }
        if check == "1" && !now.check_right {
            flips.push(format!("{name}#{index}: check right -> wrong"));
        }
        fixed.0 += usize::from(algo == "0" && now.algo_right);
        fixed.1 += usize::from(check == "0" && now.check_right);
    }
    eprintln!(
        "against baseline: algo {} wrong->right, check {} wrong->right, {} right->wrong",
        fixed.0,
        fixed.1,
        flips.len()
    );
    flips
}

/// Scores the bodies `keep` accepts and returns how many were scored.
/// `flips` enables the dump and baseline environment variables.
fn run(keep: &dyn Fn(&str) -> bool, flips: bool) -> usize {
    let mut bodies = step_bodies(keep);
    bodies.extend(synthetic_bodies(keep));
    let pins: BTreeMap<&str, (usize, usize)> = PINNED
        .iter()
        .map(|&(name, algo, check)| (name, (algo, check)))
        .collect();
    let mut scores = BTreeMap::new();
    let mut failures = Vec::new();
    let (mut total_points, mut total_algo, mut total_check) = (0, 0, 0);
    for (name, topo, solid) in &bodies {
        let Some(score) = score_body(name, topo, *solid) else {
            continue;
        };
        let (algo, check) = (score.algo_wrong(), score.check_wrong());
        eprintln!(
            "{name}: {} points, {} ambiguous, algo {algo} wrong, check {check} wrong",
            score.points, score.ambiguous
        );
        total_points += score.points;
        total_algo += algo;
        total_check += check;
        match pins.get(name.as_str()) {
            None => failures.push(format!(
                "{name}: unpinned (algo {algo}, check {check}); add it to PINNED"
            )),
            Some(&(pin_algo, pin_check)) => {
                for (engine, got, pin) in [("algo", algo, pin_algo), ("check", check, pin_check)] {
                    if got > pin {
                        failures.push(format!("{name}: {engine} {got} wrong, pinned {pin}"));
                    } else if got < pin {
                        eprintln!("{name}: {engine} improved to {got} (pinned {pin}); re-pin it");
                    }
                }
            }
        }
        scores.insert(name.clone(), score);
    }
    eprintln!(
        "{} bodies, {total_points} points: algo {total_algo} wrong, check {total_check} wrong",
        scores.len()
    );
    if let Some(path) = flips
        .then(|| std::env::var("CLASSIFY_ORACLE_DUMP").ok())
        .flatten()
    {
        std::fs::write(path, dump(&scores)).unwrap();
    }
    if let Some(path) = flips
        .then(|| std::env::var("CLASSIFY_ORACLE_BASELINE").ok())
        .flatten()
    {
        let baseline = std::fs::read_to_string(path).unwrap();
        failures.extend(flips_against(&baseline, &scores));
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
    scores.len()
}

/// The bodies of RFC 0008's F1, F2 and F4.
#[test]
fn classifier_wrong_counts_do_not_rise_on_defect_bodies() {
    assert_eq!(
        run(&|name| FAST_BODIES.contains(&name), false),
        FAST_BODIES.len()
    );
}

/// The whole corpus.
#[test]
#[ignore = "slow: cargo test --release -p remus-io --test classify_engine_oracle -- --ignored"]
fn classifier_wrong_counts_do_not_rise_on_corpus() {
    // Five bodies exceed MAX_TRIANGLES: the whole torus, the b12 torus and
    // both wide sphere caps.
    assert_eq!(run(&|_| true, true), PINNED.len());
}
