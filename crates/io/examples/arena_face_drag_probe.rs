//! Native source-relative face-move and display-mesh timings from an external
//! exact arena capture. The adjacent `<arena>.json` records the consumer's
//! meshing parameters, face normal and optional target area.
//!
//! `F=/external/part.arena cargo run --locked --profile profiling -p remus-io
//! --example arena_face_drag_probe`. Optional `DISTANCE=3`, `SAMPLES=21`,
//! `TRACE=1`, or `STEP=0.01337` for distinct source-relative distances.
//! Timings are native kernel costs, not browser input-to-frame costs.

#![allow(clippy::print_stdout)]

use std::sync::OnceLock;
use std::time::Instant;

use remus_operations::{measure, performance, push_pull, tessellate};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;

#[derive(serde::Deserialize)]
#[serde(rename_all = "camelCase")]
struct Settings {
    deflection: f64,
    angular: f64,
    normal: [f64; 3],
    target_area: Option<f64>,
}

fn clock() -> f64 {
    static START: OnceLock<Instant> = OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_secs_f64() * 1000.0
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::var("F")?;
    let settings: Settings = serde_json::from_slice(&std::fs::read(format!("{path}.json"))?)?;
    let distance: f64 = std::env::var("DISTANCE")
        .unwrap_or_else(|_| "3".into())
        .parse()?;
    let samples: usize = std::env::var("SAMPLES")
        .unwrap_or_else(|_| "21".into())
        .parse()?;
    let trace = std::env::var("TRACE").is_ok_and(|v| v == "1");
    let step = std::env::var("STEP")
        .ok()
        .map(|v| v.parse::<f64>())
        .transpose()?;
    let mut source = Topology::new();
    let solid = remus_io::arena_io::deserialize_solid(&std::fs::read(path)?, &mut source)?;
    let mut candidates = Vec::new();
    for face in solid_faces(&source, solid)? {
        if let Some(normal) = source.face(face)?.effective_plane_normal()
            && normal.x() * settings.normal[0]
                + normal.y() * settings.normal[1]
                + normal.z() * settings.normal[2]
                > 0.999
        {
            let area = measure::face_area(&source, face, settings.deflection)?;
            let rank = settings
                .target_area
                .map_or(-area, |target| (target - area).abs());
            candidates.push((face, area, rank));
        }
    }
    let &(face, area, _) = candidates
        .iter()
        .min_by(|a, b| a.2.total_cmp(&b.2))
        .ok_or("No matching planar face")?;
    remus_check::properties::face_cache::enable_thread_face_cache();
    measure::enable_thread_volume_memo();
    measure::enable_default_measurement_reuse();
    measure::enable_thread_nurbs_area_memo();
    measure::enable_thread_face_box_memo();
    tessellate::enable_face_mesh_cache();
    tessellate::enable_mesh_reuse();
    let _source_mesh = tessellate::tessellate_solid_grouped_with_tolerance(
        &source,
        solid,
        settings.deflection,
        settings.angular,
    )?;
    if trace {
        performance::configure(Some(clock));
    }
    for sample in 0..samples {
        let mut edited = source.clone();
        let phase = sample % 20;
        let offset = step.map_or_else(
            || distance - 0.2 * if phase <= 10 { phase } else { 20 - phase } as f64,
            |step| distance - step * sample as f64,
        );
        let started = Instant::now();
        let result = push_pull::move_faces(&mut edited, solid, &[face], offset)?;
        let move_ms = started.elapsed().as_secs_f64() * 1000.0;
        let started = Instant::now();
        let (mesh, groups) = tessellate::tessellate_solid_grouped_with_tolerance(
            &edited,
            result,
            settings.deflection,
            settings.angular,
        )?;
        let mesh_ms = started.elapsed().as_secs_f64() * 1000.0;
        let stats = tessellate::face_mesh_cache_stats().ok_or("Face cache disabled")?;
        println!(
            "{}",
            serde_json::json!({
                "schema": "remus-arena-face-drag-v1", "sample": sample, "distance": offset,
                "face_area": area, "move_ms": move_ms, "mesh_ms": mesh_ms,
                "vertices": mesh.positions.len(), "triangles": mesh.indices.len() / 3,
                "faces": groups.len() - 1, "cache_exact": stats.exact_hits,
                "cache_translated": stats.translated_hits, "cache_translation_refused": stats.translation_refused,
                "phases": performance::drain(),
            })
        );
    }
    Ok(())
}
