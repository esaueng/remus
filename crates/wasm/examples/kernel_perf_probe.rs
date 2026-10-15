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
//! display, volume, recognize, massprops, fillet, boolean. `all` is exactly
//! the per-edit pipeline above; `massprops`, `fillet` and `boolean` run only
//! when named, because the browser face-drag loop never calls them. For an
//! analysis stage every edit is made first, then the stage runs over the
//! prepared bodies contiguously, so a sampling profiler can isolate it.

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
    names: Vec<String>,
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
    fn push(&mut self, name: &str, unit: &'static str, value: f64) {
        if let Some(i) = self.names.iter().position(|n| n == name) {
            self.samples[i].push(value);
        } else {
            self.names.push(name.to_owned());
            self.samples.push(vec![value]);
            self.units.push(unit);
        }
    }
    fn record(&mut self, name: &str, ms: f64) {
        self.push(name, "ms", ms);
    }
    fn count(&mut self, name: &str, value: usize) {
        self.push(name, "count", value as f64);
    }
    fn report(&self, label: &str) {
        println!("== {label} (median / min / max, n samples) ==");
        for (i, (name, s)) in self.names.iter().zip(&self.samples).enumerate() {
            let mut v = s.clone();
            v.sort_by(f64::total_cmp);
            let mid = v.len() / 2;
            let med = if v.len() % 2 == 0 {
                f64::midpoint(v[mid - 1], v[mid])
            } else {
                v[mid]
            };
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
    // Not part of `all`: the face-drag pipeline does not request it.
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
    // Clones are made before any sampled loop, so a profiler attributes the
    // loop to the requested stage, not to copying the 160-face topology.
    let mut clones: Vec<Topology> = (0..iters).map(|_| source.clone()).collect();
    let edit = |i: usize, edited: &mut Topology, stages: &mut Stages| {
        let distance = 5.0 + i as f64 * 0.02;
        let t = Instant::now();
        let result = push_pull::move_faces(edited, solid, &[face], distance).expect("move");
        stages.record("move", ms(t));
        result
    };
    if stage == "all" || stage == "move" {
        for (i, edited) in clones.iter_mut().enumerate() {
            let result = edit(i, edited, &mut stages);
            if stage == "all" {
                analyse(edited, result, stage, &mut stages);
            }
        }
    } else {
        // Every edit is made before the sampled loop, so a profiler sees the
        // requested stage contiguously. Each edit is still a distinct body, so
        // content-keyed caches behave as in a real drag.
        let results: Vec<SolidId> = clones
            .iter_mut()
            .enumerate()
            .map(|(i, edited)| edit(i, edited, &mut stages))
            .collect();
        println!("edits prepared; analysing stage {stage}");
        for (edited, result) in clones.iter().zip(results) {
            analyse(edited, result, stage, &mut stages);
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

/// Fillets `edges`, recording the time under `label` on success and under
/// `<label>_refused` when the engines refuse (the body is then unchanged).
fn fillet(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    radius: f64,
    label: &str,
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
            stages.record(&format!("{label}_refused"), elapsed);
            println!(
                "fillet {label} ({} edges) refused after {elapsed:.1} ms: {e}",
                edges.len()
            );
            solid
        }
    }
}

/// Edge counts selected by [`fillet_plate`].
struct FilletSelection {
    corners: usize,
    pocket_walls: usize,
    rims: usize,
}

/// The plate's fillet sequence: outer vertical corners, pocket walls (per
/// pocket when the whole selection is refused) and top hole rims. Setup and
/// the `fillet` stage run this same sequence.
fn fillet_plate(
    topo: &mut Topology,
    body: SolidId,
    stages: &mut Stages,
) -> (SolidId, FilletSelection) {
    let corners = select_edges(topo, body, |t, e, faces| {
        all_planar(t, faces)
            && is_line(t, e)
                .is_some_and(|(dx, dy, dz)| dz.abs() > 15.0 && dx.abs() + dy.abs() < 1e-9)
    });
    let mut body = fillet(topo, body, &corners, 6.0, "fillet_corners", stages);
    // Pocket vertical walls (short vertical lines of length 8).
    let pocket_walls = select_edges(topo, body, |t, e, faces| {
        all_planar(t, faces)
            && is_line(t, e).is_some_and(|(dx, dy, dz)| {
                (dz.abs() - 8.0).abs() < 1e-3 && dx.abs() + dy.abs() < 1e-9
            })
    });
    let before = body;
    body = fillet(
        topo,
        body,
        &pocket_walls,
        2.0,
        "fillet_pocket_walls",
        stages,
    );
    if body == before {
        // Whole-selection refusal: round each pocket on its own.
        for chunk in pocket_walls.chunks(4) {
            let live_edges = solid_edges(topo, body).unwrap();
            let live: Vec<EdgeId> = chunk
                .iter()
                .copied()
                .filter(|e| live_edges.contains(e))
                .collect();
            body = fillet(topo, body, &live, 2.0, "fillet_one_pocket", stages);
        }
    }
    // Rims of the through holes on the top face (circle between plane and cylinder).
    let rims = select_edges(topo, body, |t, e, faces| {
        let edge = t.edge(e).unwrap();
        matches!(edge.curve(), EdgeCurve::Circle(_))
            && (t.vertex(edge.start()).unwrap().point().z() - 16.0).abs() < 1e-6
            && faces.len() == 2
    });
    body = fillet(topo, body, &rims, 0.8, "fillet_hole_rims", stages);
    let selection = FilletSelection {
        corners: corners.len(),
        pocket_walls: pocket_walls.len(),
        rims: rims.len(),
    };
    (body, selection)
}

fn plate(stage: &str, iters: usize) {
    // One-off construction timings, kept apart from the requested loop.
    let mut setup = Stages::new();
    let mut base = Topology::new();
    let body = build_plate(&mut base, &mut setup);
    let mut filleted_topo = base.clone();
    let (body_f, selection) = fillet_plate(&mut filleted_topo, body, &mut setup);
    let face_count = solid_faces(&filleted_topo, body_f).unwrap().len();
    println!(
        "plate: {} faces before fillet, {face_count} after ({} corner, {} pocket, {} rim edges)",
        solid_faces(&base, body).unwrap().len(),
        selection.corners,
        selection.pocket_walls,
        selection.rims
    );
    setup.report("plate setup (single run)");
    let mut stages = Stages::new();
    if stage == "fillet" {
        let mut clones: Vec<Topology> = (0..iters).map(|_| base.clone()).collect();
        for topo in &mut clones {
            let _ = fillet_plate(topo, body, &mut stages);
        }
        stages.report("plate fillet");
        return;
    }
    if stage == "boolean" {
        // Tools sit in solid web between pockets and holes, so each boolean
        // is a local edit of an otherwise untouched filleted body.
        // Each clone gets its own positioned tools first, so the timed and
        // sampled loop holds only the booleans.
        let mut jobs: Vec<(Topology, SolidId, SolidId)> = (0..iters)
            .map(|i| {
                let mut topo = filleted_topo.clone();
                let tool =
                    primitives::make_cylinder(&mut topo, 3.5 + i as f64 * 0.01, 40.0).unwrap();
                translate(&mut topo, tool, 40.0, 59.0, -10.0);
                let boss = primitives::make_cylinder(&mut topo, 5.0, 10.0).unwrap();
                translate(&mut topo, boss, 140.0, 59.0, 12.0);
                (topo, tool, boss)
            })
            .collect();
        for (topo, tool, boss) in &mut jobs {
            let t = Instant::now();
            let cut = boolean(topo, BooleanOp::Cut, body_f, *tool);
            let elapsed = ms(t);
            let Ok(cut) = cut.inspect_err(|e| {
                stages.record("cut_filleted_refused", elapsed);
                println!("cut refused after {elapsed:.1} ms: {e}");
            }) else {
                continue;
            };
            stages.record("cut_filleted", elapsed);
            let t = Instant::now();
            let fused = boolean(topo, BooleanOp::Fuse, cut, *boss);
            let elapsed = ms(t);
            match fused {
                Ok(_) => stages.record("fuse_filleted", elapsed),
                Err(e) => {
                    stages.record("fuse_filleted_refused", elapsed);
                    println!("fuse refused after {elapsed:.1} ms: {e}");
                }
            }
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
    let mut clones: Vec<Topology> = (0..iters).map(|_| filleted_topo.clone()).collect();
    // A refused move is timed as `move_refused` and its body is not
    // analysed, so analysis series only ever describe edited bodies.
    let edit = |i: usize, edited: &mut Topology, stages: &mut Stages| {
        let t = Instant::now();
        let result = push_pull::move_faces(edited, body_f, &[top], 2.0 + i as f64 * 0.02);
        let elapsed = ms(t);
        match result {
            Ok(r) => {
                stages.record("move", elapsed);
                Some(r)
            }
            Err(e) => {
                stages.record("move_refused", elapsed);
                println!("move {i} refused after {elapsed:.1} ms: {e}");
                None
            }
        }
    };
    if stage == "all" || stage == "move" {
        for (i, edited) in clones.iter_mut().enumerate() {
            if let Some(result) = edit(i, edited, &mut stages)
                && stage == "all"
            {
                analyse(edited, result, stage, &mut stages);
            }
        }
    } else {
        // Edits first, so a profiler sees the requested stage contiguously.
        let results: Vec<Option<SolidId>> = clones
            .iter_mut()
            .enumerate()
            .map(|(i, edited)| edit(i, edited, &mut stages))
            .collect();
        println!("edits prepared; analysing stage {stage}");
        for (edited, result) in clones.iter().zip(results) {
            if let Some(result) = result {
                analyse(edited, result, stage, &mut stages);
            }
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
    let iters = match args.get(3).map(|s| s.parse::<usize>()) {
        None => 6,
        Some(Ok(n)) if n > 0 => n,
        Some(_) => {
            eprintln!("iterations must be a positive integer, got {:?}", args[3]);
            std::process::exit(2);
        }
    };
    let supported: &[&str] = match scenario {
        "hammer" => &[
            "all",
            "move",
            "validate",
            "bbox",
            "display",
            "volume",
            "recognize",
            "massprops",
        ],
        "plate" => &[
            "all",
            "move",
            "validate",
            "bbox",
            "display",
            "volume",
            "recognize",
            "massprops",
            "fillet",
            "boolean",
        ],
        _ => &[],
    };
    if !supported.is_empty() && !supported.contains(&stage) {
        eprintln!("unknown stage {stage} for {scenario}; expected one of {supported:?}");
        std::process::exit(2);
    }
    match scenario {
        "hammer" => hammer(stage, iters),
        "plate" => plate(stage, iters),
        other => {
            eprintln!("unknown scenario {other}; expected hammer or plate");
            std::process::exit(2);
        }
    }
}
