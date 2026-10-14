//! Native per-stage timings for interactive CAD edit loops, with the same
//! module-wide caches the browser kernel enables (`BrepKernel::new`).
//!
//! ```text
//! cargo build --profile profiling -p remus-wasm --example kernel_perf_probe
//! target/profiling/examples/kernel_perf_probe hammer [stage] [iters]
//! target/profiling/examples/kernel_perf_probe plate  [stage] [iters]
//! ```
//!
//! `hammer` imports the public 160-face Shapr3D hammer holder and repeats the
//! filleted face-drag pipeline (move, validate, bbox, display mesh, volume,
//! recognition). `plate` builds a pocketed, drilled plate, fillets it in
//! stages and then runs the same pipeline on the filleted body. `stage`
//! (default `all`) restricts the loop to one stage: move, validate, bbox,
//! display, volume, recognize, massprops, fillet, boolean. For an analysis
//! stage every edit is made first, then the stage runs over the prepared
//! bodies contiguously, so a sampling profiler can isolate it.

#![allow(
    clippy::print_stdout,
    clippy::print_stderr,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::too_many_lines,
    clippy::cast_precision_loss,
    missing_docs
)]

use std::time::Instant;

use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::{
    blend_ops, feature_recognition, measure, primitives, push_pull, query, tessellate, transform,
    validate,
};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{edge_to_face_map, face_vertices, solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

fn ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
}

const DISPLAY_DEFLECTION: f64 = 0.02;
const DISPLAY_ANGLE: f64 = 0.35;
const MEASURE_DEFLECTION: f64 = 0.08;

struct Stages {
    names: Vec<&'static str>,
    samples: Vec<Vec<f64>>,
    /// Per-series unit; counts are reported apart from timings.
    units: Vec<&'static str>,
}

impl Stages {
    fn new() -> Self {
        Self {
            names: Vec::new(),
            samples: Vec::new(),
            units: Vec::new(),
        }
    }
    fn push(&mut self, name: &'static str, unit: &'static str, value: f64) {
        if let Some(i) = self.names.iter().position(|n| *n == name) {
            self.samples[i].push(value);
        } else {
            self.names.push(name);
            self.samples.push(vec![value]);
            self.units.push(unit);
        }
    }
    fn record(&mut self, name: &'static str, ms: f64) {
        self.push(name, "ms", ms);
    }
    fn count(&mut self, name: &'static str, value: usize) {
        self.push(name, "count", value as f64);
    }
    fn report(&self, label: &str) {
        println!("== {label} (median / min / max, n samples) ==");
        for (i, (name, s)) in self.names.iter().zip(&self.samples).enumerate() {
            let mut v = s.clone();
            v.sort_by(f64::total_cmp);
            let med = v[v.len() / 2];
            println!(
                "{name:>22}: {med:12.3} {:12.3} {:12.3} {:>5}  n={}",
                v[0],
                v[v.len() - 1],
                self.units[i],
                v.len()
            );
        }
    }
}

fn wants(stage: &str, name: &str) -> bool {
    stage == "all" || stage == name
}

/// Runs the post-edit analysis pipeline a CAD consumer calls after each edit.
fn analyse(topo: &Topology, solid: SolidId, stage: &str, stages: &mut Stages) {
    if wants(stage, "validate") {
        let t = Instant::now();
        let report = validate::validate_solid(topo, solid).expect("validate");
        stages.record("validate", ms(t));
        assert!(report.is_valid(), "invalid result");
    }
    if wants(stage, "bbox") {
        let t = Instant::now();
        let _ = measure::solid_bounding_box(topo, solid).expect("bbox");
        stages.record("bbox", ms(t));
    }
    if wants(stage, "display") {
        let t = Instant::now();
        let (mesh, _) = tessellate::tessellate_solid_grouped_with_tolerance(
            topo,
            solid,
            DISPLAY_DEFLECTION,
            DISPLAY_ANGLE,
        )
        .expect("display mesh");
        stages.record("display", ms(t));
        stages.count("display_tris", mesh.indices.len() / 3);
    }
    if wants(stage, "volume") {
        let t = Instant::now();
        let v = measure::solid_volume(topo, solid, MEASURE_DEFLECTION).expect("volume");
        stages.record("volume", ms(t));
        assert!(v.is_finite() && v > 0.0);
    }
    if wants(stage, "recognize") {
        let t = Instant::now();
        let f = feature_recognition::recognize_features(topo, solid, MEASURE_DEFLECTION)
            .expect("recognize");
        stages.record("recognize", ms(t));
        stages.count("features", f.len());
    }
    if stage == "massprops" {
        let t = Instant::now();
        let _ = measure::mass_properties(topo, solid).expect("mass props");
        stages.record("massprops", ms(t));
    }
}

fn hammer(stage: &str, iters: usize) {
    let mut source = Topology::new();
    let solids = remus_io::step::reader::read_step(
        include_str!("../../io/tests/data/shapr3d_hammer_holder.step"),
        &mut source,
    )
    .expect("fixture parses");
    let solid = solids[0];
    let faces = solid_faces(&source, solid).unwrap();
    assert_eq!(faces.len(), 160);
    let face = faces
        .iter()
        .copied()
        .find(|&face| {
            source
                .face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.x() > 0.999)
                && face_vertices(&source, face)
                    .unwrap()
                    .iter()
                    .all(|&v| (source.vertex(v).unwrap().point().x() - 48.0).abs() < 1e-7)
        })
        .expect("+X arm face");
    let mut stages = Stages::new();
    let edit = |i: usize, stages: &mut Stages| {
        let mut edited = source.clone();
        let distance = 5.0 + i as f64 * 0.02;
        let t = Instant::now();
        let result = push_pull::move_faces(&mut edited, solid, &[face], distance).expect("move");
        stages.record("move", ms(t));
        (edited, result)
    };
    if stage == "all" || stage == "move" {
        for i in 0..iters {
            let (edited, result) = edit(i, &mut stages);
            if stage == "all" {
                analyse(&edited, result, stage, &mut stages);
            }
        }
    } else {
        // Every edit is made before the sampled loop, so a profiler sees the
        // requested stage contiguously. Each edit is still a distinct body, so
        // content-keyed caches behave as in a real drag.
        let edits: Vec<_> = (0..iters).map(|i| edit(i, &mut stages)).collect();
        println!("edits prepared; analysing stage {stage}");
        for (edited, result) in &edits {
            analyse(edited, *result, stage, &mut stages);
        }
    }
    stages.report("hammer holder face drag");
}

fn translate(topo: &mut Topology, solid: SolidId, x: f64, y: f64, z: f64) {
    let m = remus_math::mat::Mat4::translation(x, y, z);
    transform::transform_solid(topo, solid, &m).expect("translate");
}

/// Plate 160 x 100 x 16 with a 4 x 3 grid of 18 x 14 x 8 pockets and a row
/// of through holes. Returns the un-filleted body.
fn build_plate(topo: &mut Topology, stages: &mut Stages) -> SolidId {
    let t = Instant::now();
    let mut body = primitives::make_box(topo, 160.0, 100.0, 16.0).expect("plate");
    for ix in 0..4 {
        for iy in 0..3 {
            let pocket = primitives::make_box(topo, 18.0, 14.0, 8.0).expect("pocket");
            translate(
                topo,
                pocket,
                12.0 + ix as f64 * 38.0,
                10.0 + iy as f64 * 28.0,
                8.0001,
            );
            body = boolean(topo, BooleanOp::Cut, body, pocket).expect("pocket cut");
        }
    }
    for ix in 0..5 {
        let hole = primitives::make_cylinder(topo, 3.0, 40.0).expect("hole");
        translate(topo, hole, 16.0 + ix as f64 * 32.0, 92.0, -10.0);
        body = boolean(topo, BooleanOp::Cut, body, hole).expect("hole cut");
    }
    stages.record("build_booleans", ms(t));
    body
}

fn is_line(topo: &Topology, edge: EdgeId) -> Option<(f64, f64, f64)> {
    let e = topo.edge(edge).ok()?;
    if !matches!(e.curve(), EdgeCurve::Line) {
        return None;
    }
    let a = topo.vertex(e.start()).ok()?.point();
    let b = topo.vertex(e.end()).ok()?.point();
    Some((b.x() - a.x(), b.y() - a.y(), b.z() - a.z()))
}

fn select_edges(
    topo: &Topology,
    solid: SolidId,
    keep: impl Fn(&Topology, EdgeId, &[FaceId]) -> bool,
) -> Vec<EdgeId> {
    let map = edge_to_face_map(topo, solid).unwrap();
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            map.get(&e.index())
                .is_some_and(|faces| keep(topo, e, faces))
        })
        .collect()
}

fn all_planar(topo: &Topology, faces: &[FaceId]) -> bool {
    faces
        .iter()
        .all(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Plane { .. }))
}

fn fillet(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    radius: f64,
    label: &'static str,
    stages: &mut Stages,
) -> SolidId {
    let t = Instant::now();
    let filtered = query::filter_filletable_edges(topo, solid, edges).expect("filter");
    let result = blend_ops::fillet_cascade(topo, solid, &filtered, radius);
    let elapsed = ms(t);
    match result {
        Ok(r) => {
            stages.record(label, elapsed);
            r.solid
        }
        Err(e) => {
            println!(
                "fillet {label} ({} edges) refused after {elapsed:.1} ms: {e}",
                edges.len()
            );
            solid
        }
    }
}

fn plate(stage: &str, iters: usize) {
    let mut stages = Stages::new();
    let mut base = Topology::new();
    let body = build_plate(&mut base, &mut stages);
    let mut filleted_topo = base.clone();
    let mut body_f = body;
    // Outer vertical corners.
    let corners = select_edges(&filleted_topo, body_f, |t, e, faces| {
        all_planar(t, faces)
            && is_line(t, e)
                .is_some_and(|(dx, dy, dz)| dz.abs() > 15.0 && dx.abs() + dy.abs() < 1e-9)
    });
    body_f = fillet(
        &mut filleted_topo,
        body_f,
        &corners,
        6.0,
        "fillet_corners",
        &mut stages,
    );
    // Pocket vertical walls (short vertical lines of length 8).
    let pocket_vert = select_edges(&filleted_topo, body_f, |t, e, faces| {
        all_planar(t, faces)
            && is_line(t, e).is_some_and(|(dx, dy, dz)| {
                (dz.abs() - 8.0).abs() < 1e-3 && dx.abs() + dy.abs() < 1e-9
            })
    });
    body_f = fillet(
        &mut filleted_topo,
        body_f,
        &pocket_vert,
        2.0,
        "fillet_pocket_walls",
        &mut stages,
    );
    if solid_faces(&filleted_topo, body_f).unwrap().len() < 100 {
        // Whole-selection refusal: round each pocket on its own.
        for chunk in pocket_vert.chunks(4) {
            let live: Vec<EdgeId> = chunk
                .iter()
                .copied()
                .filter(|&e| solid_edges(&filleted_topo, body_f).unwrap().contains(&e))
                .collect();
            body_f = fillet(
                &mut filleted_topo,
                body_f,
                &live,
                2.0,
                "fillet_one_pocket",
                &mut stages,
            );
        }
    }
    // Rims of the through holes on the top face (circle between plane and cylinder).
    let rims = select_edges(&filleted_topo, body_f, |t, e, faces| {
        let edge = t.edge(e).unwrap();
        matches!(edge.curve(), EdgeCurve::Circle(_))
            && (t.vertex(edge.start()).unwrap().point().z() - 16.0).abs() < 1e-6
            && faces.len() == 2
    });
    body_f = fillet(
        &mut filleted_topo,
        body_f,
        &rims,
        0.8,
        "fillet_hole_rims",
        &mut stages,
    );
    let face_count = solid_faces(&filleted_topo, body_f).unwrap().len();
    println!(
        "plate: {} faces before fillet, {face_count} after ({} corner, {} pocket edges)",
        solid_faces(&base, body).unwrap().len(),
        corners.len(),
        pocket_vert.len()
    );
    if stage == "fillet" {
        for _ in 0..iters {
            let mut topo = base.clone();
            let mut s = body;
            let c = select_edges(&topo, s, |t, e, faces| {
                all_planar(t, faces)
                    && is_line(t, e)
                        .is_some_and(|(dx, dy, dz)| dz.abs() > 15.0 && dx.abs() + dy.abs() < 1e-9)
            });
            s = fillet(&mut topo, s, &c, 6.0, "fillet_corners", &mut stages);
            let p = select_edges(&topo, s, |t, e, faces| {
                all_planar(t, faces)
                    && is_line(t, e).is_some_and(|(dx, dy, dz)| {
                        (dz.abs() - 8.0).abs() < 1e-3 && dx.abs() + dy.abs() < 1e-9
                    })
            });
            let _ = fillet(&mut topo, s, &p, 2.0, "fillet_pocket_walls", &mut stages);
        }
        stages.report("plate fillet");
        return;
    }
    if stage == "boolean" {
        for i in 0..iters {
            let mut topo = filleted_topo.clone();
            let t = Instant::now();
            let tool = primitives::make_cylinder(&mut topo, 4.0 + i as f64 * 0.01, 40.0).unwrap();
            translate(&mut topo, tool, 31.0, 52.0, -10.0);
            let r = boolean(&mut topo, BooleanOp::Cut, body_f, tool).expect("cut filleted");
            stages.record("cut_filleted", ms(t));
            let t = Instant::now();
            let boss = primitives::make_cylinder(&mut topo, 5.0, 10.0).unwrap();
            translate(&mut topo, boss, 70.0, 52.0, 12.0);
            let _ = boolean(&mut topo, BooleanOp::Fuse, r, boss).expect("fuse filleted");
            stages.record("fuse_filleted", ms(t));
        }
        stages.report("plate booleans on filleted body");
        return;
    }
    // Drag the top face of the filleted plate (a planar face bounded by
    // fillet bands) and analyse each result.
    let faces = solid_faces(&filleted_topo, body_f).unwrap();
    let top = faces
        .iter()
        .copied()
        .find(|&f| {
            filleted_topo
                .face(f)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.z() > 0.999)
                && face_vertices(&filleted_topo, f)
                    .unwrap()
                    .iter()
                    .all(|&v| (filleted_topo.vertex(v).unwrap().point().z() - 16.0).abs() < 1e-7)
        })
        .expect("top face");
    let edit = |i: usize, stages: &mut Stages| {
        let mut edited = filleted_topo.clone();
        let t = Instant::now();
        let result = push_pull::move_faces(&mut edited, body_f, &[top], 2.0 + i as f64 * 0.02);
        let elapsed = ms(t);
        let result = match result {
            Ok(r) => {
                stages.record("move", elapsed);
                r
            }
            Err(e) => {
                println!("move refused: {e}; analysing the unedited filleted body");
                body_f
            }
        };
        (edited, result)
    };
    if stage == "all" || stage == "move" {
        for i in 0..iters {
            let (edited, result) = edit(i, &mut stages);
            if stage == "all" {
                analyse(&edited, result, stage, &mut stages);
            }
        }
    } else {
        // Edits first, so a profiler sees the requested stage contiguously.
        let edits: Vec<_> = (0..iters).map(|i| edit(i, &mut stages)).collect();
        println!("edits prepared; analysing stage {stage}");
        for (edited, result) in &edits {
            analyse(edited, *result, stage, &mut stages);
        }
    }
    stages.report("filleted plate face drag");
}

fn main() {
    // Enables the same module-wide caches the browser kernel uses.
    let _kernel = remus_wasm::kernel::BrepKernel::new();
    let args: Vec<String> = std::env::args().collect();
    let scenario = args.get(1).map_or("hammer", String::as_str);
    let stage = args.get(2).map_or("all", String::as_str);
    let iters = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(6);
    match scenario {
        "hammer" => hammer(stage, iters),
        "plate" => plate(stage, iters),
        other => {
            eprintln!("unknown scenario {other}; expected hammer or plate");
            std::process::exit(2);
        }
    }
}
