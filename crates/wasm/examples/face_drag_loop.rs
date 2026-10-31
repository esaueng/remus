//! Native per-stage timings of the browser face-drag preview loop, with the
//! module-wide caches the browser kernel enables (`RemusKernel::new`).
//!
//! ```text
//! cargo build --profile profiling -p remus-wasm --example face_drag_loop
//! target/profiling/examples/face_drag_loop [scenario] [frames] [mode]
//! ```
//!
//! Every frame does what a consumer's drag preview does: restore the
//! checkpoint taken before the drag (`restore`), move the selected face to a
//! new source-relative distance with history (`moveFacesJournaled`), check
//! the result (`validateSolidRelaxed`), mesh it for display
//! (`tessellateSolidGroupedBinary`) and read every face's area (`faceArea`).
//!
//! `scenario` (default `both`, meaning all three) is `hammer` (the public
//! 160-face Shapr3D hammer holder, largest +X planar face, offsets -6 ...
//! -7.4: the blend translation path, refusing past about -7.25),
//! `prismatic` (a 24-face pocketed, drilled, notched block, top face,
//! offsets -0.5 ... +2.3: the planar path) or `filleted` (a box with every
//! edge rounded, top face, offsets -4 ... +6: the blend remove/rebuild
//! path). `frames` (default 40) distances sweep the range out and back
//! without repeating one, so no frame is served an earlier frame's
//! content-keyed readings. `mode` (default `unprepared`) is `unprepared`
//! (`move_faces_journaled` every frame) or `prepared` (one
//! `PreparedFaceMove` built at drag start, `move_faces_journaled` on it every
//! frame; its preparation is reported separately). With `PHASES=1` the
//! kernel's phase spans are summed per frame and reported under the move.

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
use remus_operations::{journal_ops, measure, primitives, tessellate, transform, validate};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

const DISPLAY_DEFLECTION: f64 = 0.02;
const DISPLAY_ANGLE: f64 = 0.35;
const AREA_DEFLECTION: f64 = 0.08;

static EPOCH: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();

fn host_clock() -> f64 {
    EPOCH.get_or_init(Instant::now).elapsed().as_secs_f64() * 1000.0
}

fn ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
}

#[derive(Default)]
struct Series {
    names: Vec<String>,
    samples: Vec<Vec<f64>>,
}

impl Series {
    fn record(&mut self, name: &str, value: f64) {
        if let Some(i) = self.names.iter().position(|n| n == name) {
            self.samples[i].push(value);
        } else {
            self.names.push(name.to_owned());
            self.samples.push(vec![value]);
        }
    }

    fn report(&self, label: &str) {
        println!("== {label} (ms: median / p95 / min / max, n) ==");
        for (name, s) in self.names.iter().zip(&self.samples) {
            let mut v = s.clone();
            v.sort_by(f64::total_cmp);
            let pick = |q: f64| v[((v.len() - 1) as f64 * q).round() as usize];
            println!(
                "{name:>40}: {:9.3} {:9.3} {:9.3} {:9.3}  n={}",
                pick(0.5),
                pick(0.95),
                v[0],
                v[v.len() - 1],
                v.len()
            );
        }
    }
}

/// A body, the face a drag moves and the drag's distance range.
type Scenario = (Topology, SolidId, FaceId, (f64, f64));

fn translate(topo: &mut Topology, solid: SolidId, x: f64, y: f64, z: f64) {
    let m = remus_math::mat::Mat4::translation(x, y, z);
    transform::transform_solid(topo, solid, &m).expect("translate");
}

fn planar_faces_along(topo: &Topology, solid: SolidId, axis: usize) -> Vec<FaceId> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| [n.x(), n.y(), n.z()][axis] > 0.999)
        })
        .collect()
}

fn largest(topo: &Topology, faces: &[FaceId]) -> FaceId {
    faces
        .iter()
        .copied()
        .max_by(|&a, &b| {
            let area = |f| measure::face_area(topo, f, AREA_DEFLECTION).unwrap();
            area(a).total_cmp(&area(b))
        })
        .expect("a candidate face")
}

fn hammer() -> Scenario {
    let mut topo = Topology::new();
    let solids = remus_io::step::reader::read_step(
        include_str!("../../io/tests/data/shapr3d_hammer_holder.step"),
        &mut topo,
    )
    .expect("fixture parses");
    let solid = solids[0];
    let face = largest(&topo, &planar_faces_along(&topo, solid, 0));
    (topo, solid, face, sweep(-6.0, -7.4))
}

/// 80 x 50 x 20 block with three 12 x 10 x 8 top pockets, a through hole
/// and a full-height corner notch: 24 faces, every top-face neighbour
/// perpendicular to the top face.
fn prismatic() -> Scenario {
    let mut topo = Topology::new();
    let mut body = primitives::make_box(&mut topo, 80.0, 50.0, 20.0).unwrap();
    for i in 0..3 {
        let pocket = primitives::make_box(&mut topo, 12.0, 10.0, 8.0).unwrap();
        translate(&mut topo, pocket, 8.0 + 20.0 * f64::from(i), 8.0, 12.0);
        body = boolean(&mut topo, BooleanOp::Cut, body, pocket).unwrap();
    }
    let hole = primitives::make_cylinder(&mut topo, 4.0, 40.0).unwrap();
    translate(&mut topo, hole, 30.0, 36.0, -10.0);
    body = boolean(&mut topo, BooleanOp::Cut, body, hole).unwrap();
    let notch = primitives::make_box(&mut topo, 10.0, 10.0, 40.0).unwrap();
    translate(&mut topo, notch, 75.0, 45.0, -10.0);
    body = boolean(&mut topo, BooleanOp::Cut, body, notch).unwrap();
    assert_eq!(solid_faces(&topo, body).unwrap().len(), 24);
    let face = largest(&topo, &planar_faces_along(&topo, body, 2));
    (topo, body, face, sweep(-0.5, 2.3))
}

/// 40 x 30 x 20 box with every edge rounded (r = 3): 6 planar faces, 12
/// cylinders and 8 sphere corners. The top face moves through its blend
/// band.
fn filleted() -> Scenario {
    let mut topo = Topology::new();
    let body = primitives::make_box(&mut topo, 40.0, 30.0, 20.0).unwrap();
    let edges = remus_topology::explorer::solid_edges(&topo, body).unwrap();
    let body = remus_operations::blend_ops::fillet_cascade(&mut topo, body, &edges, 3.0)
        .unwrap()
        .solid;
    assert_eq!(solid_faces(&topo, body).unwrap().len(), 26);
    let face = largest(&topo, &planar_faces_along(&topo, body, 2));
    (topo, body, face, sweep(-4.0, 6.0))
}

/// The drag's range; every frame gets its own distance within it.
fn sweep(from: f64, to: f64) -> (f64, f64) {
    (from, to)
}

/// A drag sweeps the range out and back without repeating a distance, so no
/// frame is served a previous frame's content-keyed readings.
fn frame_distance((from, to): (f64, f64), frame: usize, frames: usize) -> f64 {
    let half = frames.div_ceil(2).max(2);
    let step = (to - from) / (half - 1) as f64;
    if frame < half {
        from + step * frame as f64
    } else {
        to - step * ((frame - half) as f64 + 0.5)
    }
}

fn run(label: &str, scenario: fn() -> Scenario, frames: usize, prepared: bool) {
    let (source, solid, face, distances) = scenario();
    let face_count = solid_faces(&source, solid).unwrap().len();
    // The consumer's checkpoint: the source state every frame restores.
    let checkpoint = source.clone();
    let mut topo = source;
    let mut series = Series::default();
    let mut prepared_move = None;
    if prepared {
        let t = Instant::now();
        prepared_move = Some(
            remus_operations::prepared_move::PreparedFaceMove::prepare(&topo, solid, &[face])
                .expect("prepare"),
        );
        series.record("prepare", ms(t));
    }
    let mut refused = Vec::new();
    let mut volumes = Vec::new();
    for frame in 0..frames {
        let distance = frame_distance(distances, frame, frames);
        let total = Instant::now();

        let _ = remus_operations::performance::drain();
        let t = Instant::now();
        topo.restore_preserving_handle_slots(&checkpoint);
        series.record("restore", ms(t));

        let t = Instant::now();
        let moved = match &mut prepared_move {
            Some(session) => session.move_faces_journaled(&mut topo, distance),
            None => journal_ops::move_faces_journaled(&mut topo, solid, &[face], distance),
        };
        let move_ms = ms(t);
        let moved = match moved {
            Ok(moved) => moved,
            Err(error) => {
                series.record("move_refused", move_ms);
                refused.push(format!("{distance:+.4}: {error}"));
                continue;
            }
        };
        series.record("move", move_ms);
        let mut phases: Vec<(&'static str, f64)> = Vec::new();
        for phase in remus_operations::performance::drain() {
            match phases.iter_mut().find(|(name, _)| *name == phase.name) {
                Some((_, total)) => *total += phase.duration_ms,
                None => phases.push((phase.name, phase.duration_ms)),
            }
        }
        for (name, total) in phases {
            series.record(&format!("  {name}"), total);
        }
        let result = moved.solid;

        let t = Instant::now();
        let report = validate::validate_solid_relaxed(&topo, result).unwrap();
        series.record("validate", ms(t));
        assert!(report.is_valid());

        let t = Instant::now();
        let (mesh, _) = tessellate::tessellate_solid_grouped_with_tolerance(
            &topo,
            result,
            DISPLAY_DEFLECTION,
            DISPLAY_ANGLE,
        )
        .unwrap();
        series.record("display", ms(t));
        assert!(!mesh.indices.is_empty());

        let t = Instant::now();
        let mut area = 0.0;
        for f in solid_faces(&topo, result).unwrap() {
            area += measure::face_area(&topo, f, AREA_DEFLECTION).unwrap();
        }
        series.record("areas", ms(t));
        series.record("frame", ms(total));
        if frame % 8 == 0 {
            volumes.push((distance, area));
        }
    }
    println!(
        "{label}: {face_count} faces, {frames} frames, {} refused",
        refused.len()
    );
    for refusal in &refused {
        println!("  refused {refusal}");
    }
    for (distance, area) in volumes {
        println!("  distance {distance:+.3}: total face area {area:.9}");
    }
    series.report(label);
}

fn main() {
    let _kernel = remus_wasm::kernel::RemusKernel::new();
    if std::env::var_os("PHASES").is_some() {
        remus_operations::performance::configure(Some(host_clock));
    }
    let args: Vec<String> = std::env::args().collect();
    let scenario = args.get(1).map_or("both", String::as_str);
    let frames = args
        .get(2)
        .map_or(40, |s| s.parse::<usize>().expect("frames"));
    let prepared = match args.get(3).map_or("unprepared", String::as_str) {
        "unprepared" => false,
        "prepared" => true,
        other => {
            eprintln!("unknown mode {other}; expected unprepared or prepared");
            std::process::exit(2);
        }
    };
    if matches!(scenario, "hammer" | "both") {
        run("hammer holder +X face drag", hammer, frames, prepared);
    }
    if matches!(scenario, "prismatic" | "both") {
        run(
            "24-face prismatic top-face drag",
            prismatic,
            frames,
            prepared,
        );
    }
    if matches!(scenario, "filleted" | "both") {
        run(
            "26-face filleted box top-face drag",
            filleted,
            frames,
            prepared,
        );
    }
}
