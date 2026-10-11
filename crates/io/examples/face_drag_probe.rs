//! Reproducible native face-edit timings on the public 160-face hammer holder.
//!
//! Run an optimized build with `RAYON_NUM_THREADS=1 cargo run --locked
//! --profile profiling -p remus-io --example face_drag_probe`. The first of
//! four samples is a warmup. This measures uncached native validation and
//! explicit blend-preserving `move_faces`, not browser input-to-frame time.

#![allow(clippy::print_stdout, clippy::expect_used, clippy::unwrap_used)]

use std::time::Instant;

use remus_operations::{measure, push_pull, tessellate, validate};
use remus_topology::Topology;
use remus_topology::explorer::{face_vertices, solid_faces};
use remus_topology::face::FaceSurface;

fn ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut source = Topology::new();
    let solids = remus_io::step::reader::read_step(
        include_str!("../tests/data/shapr3d_hammer_holder.step"),
        &mut source,
    )?;
    assert_eq!(solids.len(), 1);
    let solid = solids[0];
    let faces = solid_faces(&source, solid)?;
    assert_eq!(faces.len(), 160);
    assert_eq!(
        faces
            .iter()
            .filter(|&&face| matches!(source.face(face).unwrap().surface(), FaceSurface::Nurbs(_)))
            .count(),
        42
    );
    let candidates: Vec<_> = faces
        .iter()
        .copied()
        .filter(|&face| {
            source
                .face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.x() > 0.999)
                && face_vertices(&source, face)
                    .unwrap()
                    .iter()
                    .all(|&vertex| (source.vertex(vertex).unwrap().point().x() - 48.0).abs() < 1e-7)
        })
        .collect();
    assert_eq!(candidates.len(), 1, "unique +X outer arm face");
    let face = candidates[0];
    for sample in 0..4 {
        let started = Instant::now();
        let validation = validate::validate_solid(&source, solid)?;
        let validation_ms = ms(started);
        assert!(validation.is_valid());
        // Clone before the timed edit, matching an independent drag candidate.
        let mut edited = source.clone();
        let started = Instant::now();
        let result = push_pull::move_faces(&mut edited, solid, &[face], 1.0)?;
        let edit_ms = ms(started);
        let started = Instant::now();
        let validation = validate::validate_solid(&edited, result)?;
        let edited_validation_ms = ms(started);
        assert!(validation.is_valid());
        assert_eq!(solid_faces(&edited, result)?.len(), faces.len());
        let started = Instant::now();
        let mesh = tessellate::tessellate_solid_with_tolerance(&edited, result, 0.0148, 0.06)?;
        let mesh_ms = ms(started);
        let boundary = tessellate::boundary_edge_count(&mesh);
        let non_manifold = tessellate::non_manifold_edge_count(&mesh);
        assert_eq!((boundary, non_manifold), (0, 0));
        let volume = measure::solid_volume(&edited, result, 0.08)?;
        assert!(volume.is_finite() && volume > 0.0);
        println!(
            "{}",
            serde_json::json!({
                "schema": "remus-face-drag-sample-v1",
                "sample": sample,
                "warmup": sample == 0,
                "validation_ms": validation_ms,
                "move_faces_ms": edit_ms,
                "edited_validation_ms": edited_validation_ms,
                "display_mesh_ms": mesh_ms,
                "volume": volume,
                "faces": faces.len(),
                "triangles": mesh.indices.len() / 3,
                "boundary_edges": boundary,
                "non_manifold_edges": non_manifold,
                "validation": "passed"
            })
        );
    }
    Ok(())
}
