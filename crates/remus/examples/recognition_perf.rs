//! Time imported-feature recognition and whole-solid edge relations on an
//! imported STEP body.
//!
//! Diagnostic for the OpenZCAD rebuild path: after every direct edit the
//! application re-runs `recognizeFeatures` and `solidEdgeRelations` on the
//! whole imported body. This example times the two operations those bindings
//! wrap and prints result digests, so a candidate can be checked for
//! byte-identical output against a baseline build.
//!
//! Run with
//! `cargo run --profile profiling -p remus --example recognition_perf -- [step] [deflection]`.
//!
//! Environment:
//! - `RECOG_PERF_RUNS` — timed repetitions per call (default 5).
//! - `RECOG_PERF_ONLY` — `recognize` or `relations` to time just one call
//!   (useful under `samply record`).
//! - `RECOG_PERF_DUMP` — directory to write the full `Debug` dumps
//!   (`features.txt`, `relations.txt`) for an exact diff.

#![allow(
    clippy::print_stdout,
    clippy::print_stderr,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss
)]

use std::collections::BTreeMap;
use std::time::Instant;

use remus_io::step::reader::read_step;
use remus_operations::feature_recognition::{Feature, recognize_features};
use remus_operations::query::{EdgeConcavity, solid_edge_relations};
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

fn timed<T>(label: &str, f: impl FnOnce() -> T) -> (T, f64) {
    let start = Instant::now();
    let out = f();
    let ms = start.elapsed().as_secs_f64() * 1e3;
    println!("{label:<40} {ms:>9.1} ms");
    (out, ms)
}

/// FNV-1a over the UTF-8 bytes: a stable, dependency-free digest.
fn fnv1a(text: &str) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    for byte in text.bytes() {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(0x0100_0000_01b3);
    }
    hash
}

fn surface_census(topo: &Topology, solid: SolidId) -> String {
    let mut counts: BTreeMap<&str, usize> = BTreeMap::new();
    for face in solid_faces(topo, solid).unwrap() {
        let tag = match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => "plane",
            FaceSurface::Cylinder(_) => "cylinder",
            FaceSurface::Cone(_) => "cone",
            FaceSurface::Sphere(_) => "sphere",
            FaceSurface::Torus(_) => "torus",
            FaceSurface::Nurbs(_) => "nurbs",
        };
        *counts.entry(tag).or_default() += 1;
    }
    counts
        .iter()
        .map(|(tag, n)| format!("{tag}={n}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn feature_kind(feature: &Feature) -> &'static str {
    match feature {
        Feature::Hole { .. } => "hole",
        Feature::Chamfer { .. } => "chamfer",
        Feature::FilletLike { .. } => "fillet_like",
        Feature::Pocket { .. } => "pocket",
        Feature::Pattern { .. } => "pattern",
        _ => "other",
    }
}

fn median(samples: &mut [f64]) -> f64 {
    samples.sort_by(f64::total_cmp);
    samples[samples.len() / 2]
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .unwrap_or_else(|| "crates/io/tests/data/shapr3d_hammer_holder.step".to_string());
    let deflection: f64 = args.next().map_or(0.08, |v| v.parse().unwrap());
    let runs: usize = std::env::var("RECOG_PERF_RUNS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(5);
    let only = std::env::var("RECOG_PERF_ONLY").ok();
    let dump = std::env::var_os("RECOG_PERF_DUMP").map(std::path::PathBuf::from);

    let text = std::fs::read_to_string(&path)?;
    let mut topo = Topology::new();
    let (solids, _) = timed("read_step", || read_step(&text, &mut topo));
    let solid = solids?[0];
    println!(
        "faces={} edges={} {}",
        solid_faces(&topo, solid)?.len(),
        solid_edges(&topo, solid)?.len(),
        surface_census(&topo, solid)
    );

    if only.as_deref() != Some("relations") {
        let mut times = Vec::with_capacity(runs);
        let mut digest = None;
        for _ in 0..runs {
            let (features, ms) = timed("recognize_features", || {
                recognize_features(&topo, solid, deflection)
            });
            let features = features?;
            times.push(ms);
            let text = format!("{features:#?}");
            let mut kinds: BTreeMap<&str, usize> = BTreeMap::new();
            for feature in &features {
                *kinds.entry(feature_kind(feature)).or_default() += 1;
            }
            let summary = format!(
                "features={} {kinds:?} digest={:016x}",
                features.len(),
                fnv1a(&text)
            );
            if let Some(dir) = &dump {
                std::fs::create_dir_all(dir)?;
                std::fs::write(dir.join("features.txt"), &text)?;
            }
            if let Some(previous) = &digest {
                assert_eq!(previous, &summary, "recognition is not deterministic");
            } else {
                println!("  {summary}");
                digest = Some(summary);
            }
        }
        println!("recognize_features median {:.1} ms", median(&mut times));
    }

    if only.as_deref() != Some("recognize") {
        let mut times = Vec::with_capacity(runs);
        let mut digest = None;
        for _ in 0..runs {
            let (relations, ms) = timed("solid_edge_relations", || {
                solid_edge_relations(&topo, solid, None)
            });
            let relations = relations?;
            times.push(ms);
            let text = format!("{relations:#?}");
            let mut kinds: BTreeMap<&str, usize> = BTreeMap::new();
            for relation in &relations {
                let kind = match relation.concavity {
                    EdgeConcavity::Convex => "convex",
                    EdgeConcavity::Concave => "concave",
                    EdgeConcavity::Tangent => "tangent",
                    EdgeConcavity::Unknown => "unknown",
                };
                *kinds.entry(kind).or_default() += 1;
            }
            let summary = format!(
                "relations={} {kinds:?} digest={:016x}",
                relations.len(),
                fnv1a(&text)
            );
            if let Some(dir) = &dump {
                std::fs::create_dir_all(dir)?;
                std::fs::write(dir.join("relations.txt"), &text)?;
            }
            if let Some(previous) = &digest {
                assert_eq!(previous, &summary, "edge relations are not deterministic");
            } else {
                println!("  {summary}");
                digest = Some(summary);
            }
        }
        println!("solid_edge_relations median {:.1} ms", median(&mut times));
    }
    Ok(())
}
