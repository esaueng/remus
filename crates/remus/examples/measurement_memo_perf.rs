//! Time strict validation, whole-body volume, face areas and a planar face
//! move on an imported STEP body, cold and through the opt-in measurement
//! memos.
//!
//! Diagnostic for the strict validator's orientation probe and the
//! measurement reads an application repeats per edit. Each mode prints one
//! line per timed call:
//!
//! * `validate` — strict `validate_solid`, memos off;
//! * `volume` — `solid_volume(.., 0.08)`, memos off;
//! * `areas` — `face_area(.., 0.08)` over every face, memos off;
//! * `move` — `push_pull::move_faces` of the +X planar face closest to the
//!   target area by the distance, on a fresh clone each run, memos off;
//! * `memo` — the same reads with both memos enabled: first (cold) and
//!   repeated (warm) strict validation, a deserialized copy in a fresh
//!   topology, warm face areas, and a repeated volume;
//! * `arena` — write the body's exact arena document to stdout, for timing
//!   the same reads through a WASM build.
//!
//! Run with
//! `cargo run --profile profiling -p remus --example measurement_memo_perf -- <mode> [runs] [step] [distance] [area]`.

#![allow(
    clippy::print_stdout,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_precision_loss
)]

use std::time::Instant;

use remus_check::properties::face_cache::{enable_thread_face_cache, thread_face_cache_stats};
use remus_io::arena_io::{deserialize_solids, serialize_solids};
use remus_io::step::reader::read_step;
use remus_operations::measure::{
    enable_thread_volume_memo, face_area, solid_volume, thread_volume_memo_stats,
};
use remus_operations::push_pull::move_faces;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

fn timed<T>(label: &str, f: impl FnOnce() -> T) -> T {
    let start = Instant::now();
    let out = f();
    println!(
        "{label:<36} {:>9.1} ms",
        start.elapsed().as_secs_f64() * 1e3
    );
    out
}

fn all_areas(topo: &Topology, solid: SolidId) -> f64 {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(|face| face_area(topo, face, 0.08).unwrap())
        .sum()
}

fn move_target(topo: &Topology, solid: SolidId, target_area: f64) -> FaceId {
    let mut best = None;
    let mut best_gap = f64::INFINITY;
    for face in solid_faces(topo, solid).unwrap() {
        if let FaceSurface::Plane { normal, .. } = topo.face(face).unwrap().surface()
            && normal.x() > 0.99
        {
            let gap = (face_area(topo, face, 0.05).unwrap() - target_area).abs();
            if gap < best_gap {
                best_gap = gap;
                best = Some(face);
            }
        }
    }
    best.expect("a +X planar face")
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mode = args.next().unwrap_or_else(|| "validate".into());
    let runs: usize = args.next().map_or(5, |v| v.parse().unwrap());
    let path = args
        .next()
        .unwrap_or_else(|| "crates/io/tests/data/shapr3d_hammer_holder.step".into());
    let distance: f64 = args.next().map_or(-6.0, |v| v.parse().unwrap());
    let target_area: f64 = args.next().map_or(1045.93, |v| v.parse().unwrap());

    let text = std::fs::read_to_string(&path).unwrap();
    let mut topo = Topology::new();
    let solid = read_step(&text, &mut topo).unwrap()[0];

    match mode.as_str() {
        "validate" => {
            for _ in 0..runs {
                let report = timed("validate_solid (strict)", || {
                    validate_solid(&topo, solid).unwrap()
                });
                assert!(report.is_valid());
            }
        }
        "volume" => {
            for _ in 0..runs {
                timed("solid_volume(0.08)", || {
                    solid_volume(&topo, solid, 0.08).unwrap()
                });
            }
        }
        "areas" => {
            for _ in 0..runs {
                timed("face_area(0.08) x all faces", || all_areas(&topo, solid));
            }
        }
        "move" => {
            let face = move_target(&topo, solid, target_area);
            for _ in 0..runs {
                let mut moved = topo.clone();
                timed("push_pull::move_faces", || {
                    move_faces(&mut moved, solid, &[face], distance).unwrap()
                });
            }
        }
        "memo" => {
            enable_thread_face_cache();
            enable_thread_volume_memo();
            for run in 0..runs {
                let label = if run == 0 { "cold" } else { "warm" };
                timed(&format!("validate_solid (strict, {label})"), || {
                    validate_solid(&topo, solid).unwrap()
                });
            }
            let bytes = serialize_solids(&topo, &[solid]).unwrap();
            let mut copy = Topology::new();
            let copied = deserialize_solids(&bytes, &mut copy).unwrap()[0];
            timed("validate_solid (strict, copy)", || {
                validate_solid(&copy, copied).unwrap()
            });
            for run in 0..runs {
                let label = if run == 0 { "cold" } else { "warm" };
                timed(&format!("face_area(0.08) x all ({label})"), || {
                    all_areas(&topo, solid)
                });
            }
            for run in 0..runs {
                let label = if run == 0 { "cold" } else { "warm" };
                timed(&format!("solid_volume(0.08) ({label})"), || {
                    solid_volume(&topo, solid, 0.08).unwrap()
                });
            }
            println!("face cache {:?}", thread_face_cache_stats());
            println!("volume memo {:?}", thread_volume_memo_stats());
        }
        "arena" => {
            use std::io::Write;
            let bytes = serialize_solids(&topo, &[solid]).unwrap();
            std::io::stdout().write_all(&bytes).unwrap();
        }
        other => panic!("unknown mode {other}"),
    }
}
