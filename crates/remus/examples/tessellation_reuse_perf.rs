//! Time display tessellation of a body before and after a planar face move,
//! with and without the per-face mesh cache (PERF-D01).
//!
//! Diagnostic for the OpenZCAD "offset face" direct edit: import the fixture,
//! pick the +X planar face closest to a target area, mesh the source body at
//! the display tolerances, move the face, then mesh the result cold (cache
//! disabled) and warm (cache filled by the source mesh). The warm mesh is
//! compared with the cold one vertex for vertex.
//!
//! Run with
//! `cargo run --profile profiling -p remus --example tessellation_reuse_perf -- [step] [distance] [area]`.
//! `TESS_DEFLECTION`, `TESS_ANGULAR` and `TESS_REPEAT` override the display
//! tolerances (0.0148, 0.06) and the repeat count (3); `TESS_PROFILE_WARM=n`
//! only times n warm source meshes (for a profiler).

#![allow(
    clippy::print_stdout,
    clippy::print_stderr,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::too_many_lines
)]

use std::time::Instant;

use remus_io::step::reader::read_step;
use remus_operations::measure::face_area;
use remus_operations::push_pull::move_faces;
use remus_operations::tessellate::{
    FaceMeshCacheStats, TriangleMesh, boundary_edge_count, clear_face_mesh_cache,
    disable_face_mesh_cache, enable_face_mesh_cache, face_mesh_cache_stats,
    non_manifold_edge_count, tessellate_solid_grouped_with_tolerance,
};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

fn timed<T>(label: &str, f: impl FnOnce() -> T) -> T {
    let start = Instant::now();
    let out = f();
    println!(
        "{label:<48} {:>9.1} ms",
        start.elapsed().as_secs_f64() * 1e3
    );
    out
}

fn stats() -> FaceMeshCacheStats {
    face_mesh_cache_stats().unwrap_or_default()
}

fn print_delta(label: &str, before: FaceMeshCacheStats, after: FaceMeshCacheStats) {
    println!(
        "  {label}: lookups {} exact {} translated {} miss {} (refused translates {}) conflicts {} uncacheable {} stored {} entries {} bytes {:.1} MiB",
        after.lookups - before.lookups,
        after.exact_hits - before.exact_hits,
        after.translated_hits - before.translated_hits,
        after.misses - before.misses,
        after.translation_refused - before.translation_refused,
        after.replay_conflicts - before.replay_conflicts,
        after.uncacheable - before.uncacheable,
        after.stored - before.stored,
        after.entries,
        after.retained_bytes as f64 / (1024.0 * 1024.0),
    );
}

fn mesh(topo: &Topology, solid: SolidId, d: f64, a: f64) -> (TriangleMesh, Vec<u32>) {
    tessellate_solid_grouped_with_tolerance(topo, solid, d, a).expect("tessellate")
}

/// Compare two grouped meshes: identical indices and offsets, plus the
/// largest position and normal deviation.
fn compare(a: &(TriangleMesh, Vec<u32>), b: &(TriangleMesh, Vec<u32>)) -> (bool, f64, f64) {
    let same_topology =
        a.0.indices == b.0.indices && a.1 == b.1 && a.0.positions.len() == b.0.positions.len();
    let mut pos_dev = 0.0_f64;
    let mut nrm_dev = 0.0_f64;
    for (p, q) in a.0.positions.iter().zip(&b.0.positions) {
        pos_dev = pos_dev.max((*p - *q).length());
    }
    for (p, q) in a.0.normals.iter().zip(&b.0.normals) {
        nrm_dev = nrm_dev.max((*p - *q).length());
    }
    (same_topology, pos_dev, nrm_dev)
}

/// Order-insensitive triangle comparison: each triangle rotated so its
/// lexicographically smallest corner leads (orientation kept), triangles
/// sorted; returns the largest corner and normal deviation, or `None` when
/// the counts differ.
fn set_deviation(a: &TriangleMesh, ta: &[u32], b: &TriangleMesh, tb: &[u32]) -> Option<(f64, f64)> {
    fn canon(m: &TriangleMesh, t: &[u32]) -> Vec<[usize; 3]> {
        let key = |i: usize| {
            let p = m.positions[i];
            (
                (p.x() * 1e6).round() as i64,
                (p.y() * 1e6).round() as i64,
                (p.z() * 1e6).round() as i64,
            )
        };
        let mut tris: Vec<[usize; 3]> = t
            .chunks_exact(3)
            .map(|c| {
                let c = [c[0] as usize, c[1] as usize, c[2] as usize];
                let r = (0..3).min_by_key(|&r| key(c[r])).unwrap();
                [c[r], c[(r + 1) % 3], c[(r + 2) % 3]]
            })
            .collect();
        tris.sort_by_key(|t| (key(t[0]), key(t[1]), key(t[2])));
        tris
    }
    if ta.len() != tb.len() {
        return None;
    }
    let (ca, cb) = (canon(a, ta), canon(b, tb));
    let mut dev = 0.0_f64;
    let mut ndev = 0.0_f64;
    for (x, y) in ca.iter().zip(&cb) {
        for k in 0..3 {
            dev = dev.max((a.positions[x[k]] - b.positions[y[k]]).length());
            ndev = ndev.max((a.normals[x[k]] - b.normals[y[k]]).length());
        }
    }
    Some((dev, ndev))
}

/// Per-face comparison: for each face group, the triangle count and the
/// largest corner deviation when triangles are compared in order.
fn per_face(
    topo: &Topology,
    solid: SolidId,
    a: &(TriangleMesh, Vec<u32>),
    b: &(TriangleMesh, Vec<u32>),
) {
    let faces = solid_faces(topo, solid).unwrap();
    let mut differing = 0;
    for (i, face) in faces.iter().enumerate() {
        let (sa, ea) = (a.1[i] as usize, a.1[i + 1] as usize);
        let (sb, eb) = (b.1[i] as usize, b.1[i + 1] as usize);
        let ta = &a.0.indices[sa..ea];
        let tb = &b.0.indices[sb..eb];
        let mut dev = 0.0_f64;
        let mut ndev = 0.0_f64;
        if ta.len() == tb.len() {
            for (x, y) in ta.iter().zip(tb) {
                dev = dev.max((a.0.positions[*x as usize] - b.0.positions[*y as usize]).length());
                ndev = ndev.max((a.0.normals[*x as usize] - b.0.normals[*y as usize]).length());
            }
        }
        if ta.len() != tb.len() || dev > 1e-9 || ndev > 1e-9 {
            differing += 1;
            let set_dev = set_deviation(&a.0, ta, &b.0, tb);
            println!("    set-based corner/normal dev {set_dev:?}");
            println!(
                "    face {} ({}) tris {} vs {} corner dev {dev:.3e} normal dev {ndev:.3e}",
                face.index(),
                topo.face(*face).unwrap().surface().type_tag(),
                ta.len() / 3,
                tb.len() / 3
            );
        }
    }
    println!("    {differing} faces differ");
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .unwrap_or_else(|| "crates/io/tests/data/shapr3d_hammer_holder.step".to_string());
    let distance: f64 = args.next().map_or(-6.0, |v| v.parse().unwrap());
    let target_area: f64 = args.next().map_or(1045.93, |v| v.parse().unwrap());
    let deflection: f64 = std::env::var("TESS_DEFLECTION")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.0148);
    let angular: f64 = std::env::var("TESS_ANGULAR")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.06);
    let repeat: usize = std::env::var("TESS_REPEAT")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(3);

    let text = std::fs::read_to_string(&path)?;
    let mut topo = Topology::new();
    let solids = timed("read_step", || read_step(&text, &mut topo))?;
    let solid = solids[0];
    let faces = solid_faces(&topo, solid)?;
    println!(
        "faces={} deflection={deflection} angular={angular}",
        faces.len()
    );

    let mut candidates = Vec::new();
    for &face in &faces {
        if let FaceSurface::Plane { normal, .. } = topo.face(face)?.surface()
            && normal.x() > 0.99
        {
            let area = face_area(&topo, face, 0.05)?;
            candidates.push((face, area));
        }
    }
    candidates.sort_by(|a, b| {
        (a.1 - target_area)
            .abs()
            .partial_cmp(&(b.1 - target_area).abs())
            .unwrap()
    });
    let face = candidates[0].0;
    println!("selected face {} distance {distance}", face.index());

    // Profiling aid: `TESS_PROFILE_WARM=n` meshes the source once to fill
    // the cache, then n more times warm, and exits.
    if let Some(runs) = std::env::var("TESS_PROFILE_WARM")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
    {
        enable_face_mesh_cache();
        mesh(&topo, solid, deflection, angular);
        for _ in 0..runs {
            timed("tessellate source, cache warm", || {
                mesh(&topo, solid, deflection, angular)
            });
        }
        return Ok(());
    }

    println!("--- source body ---");
    disable_face_mesh_cache();
    let mut source_fresh = None;
    for _ in 0..repeat {
        source_fresh = Some(timed("tessellate source, cache off", || {
            mesh(&topo, solid, deflection, angular)
        }));
    }
    let source_fresh = source_fresh.unwrap();
    println!("  tris {}", source_fresh.0.indices.len() / 3);

    enable_face_mesh_cache();
    // Capture overhead: every cold run starts from an empty cache.
    for _ in 1..repeat {
        clear_face_mesh_cache();
        timed("tessellate source, cache cold", || {
            mesh(&topo, solid, deflection, angular)
        });
    }
    clear_face_mesh_cache();
    let s0 = stats();
    let source_cold = timed("tessellate source, cache cold", || {
        mesh(&topo, solid, deflection, angular)
    });
    print_delta("cold", s0, stats());
    println!(
        "  cold vs fresh (same indices, pos dev, normal dev): {:?}",
        compare(&source_cold, &source_fresh)
    );
    for _ in 0..repeat {
        let s0 = stats();
        let warm = timed("tessellate source, cache warm", || {
            mesh(&topo, solid, deflection, angular)
        });
        print_delta("warm", s0, stats());
        println!(
            "  warm vs fresh (same indices, pos dev, normal dev): {:?}",
            compare(&warm, &source_fresh)
        );
        per_face(&topo, solid, &warm, &source_fresh);
    }

    println!("--- move_faces ---");
    // The move verifies itself with mesh volumes, which tessellate through
    // the same pipeline: time it on copies with the cache off and warm.
    disable_face_mesh_cache();
    for _ in 0..repeat {
        let mut copy = topo.clone();
        timed("push_pull::move_faces, cache off", || {
            move_faces(&mut copy, solid, &[face], distance)
        })?;
    }
    enable_face_mesh_cache();
    for _ in 0..repeat {
        let mut copy = topo.clone();
        let s0 = stats();
        timed("push_pull::move_faces, cache warm", || {
            move_faces(&mut copy, solid, &[face], distance)
        })?;
        print_delta("move", s0, stats());
    }
    let moved = move_faces(&mut topo, solid, &[face], distance)?;
    println!("  result faces {}", solid_faces(&topo, moved)?.len());

    disable_face_mesh_cache();
    let mut moved_fresh = None;
    for _ in 0..repeat {
        moved_fresh = Some(timed("tessellate moved, cache off", || {
            mesh(&topo, moved, deflection, angular)
        }));
    }
    let moved_fresh = moved_fresh.unwrap();
    println!(
        "  tris {} bd {} nm {}",
        moved_fresh.0.indices.len() / 3,
        boundary_edge_count(&moved_fresh.0),
        non_manifold_edge_count(&moved_fresh.0)
    );

    enable_face_mesh_cache();
    for _ in 0..repeat {
        // Warm from the source body only.
        clear_face_mesh_cache();
        mesh(&topo, solid, deflection, angular);
        let s0 = stats();
        let reused = timed("tessellate moved, cache warm (source)", || {
            mesh(&topo, moved, deflection, angular)
        });
        print_delta("moved", s0, stats());
        let (same, pos_dev, nrm_dev) = compare(&reused, &moved_fresh);
        println!(
            "  reused vs fresh: same indices/offsets {same}, max position dev {pos_dev:.3e}, max normal dev {nrm_dev:.3e}, bd {} nm {}",
            boundary_edge_count(&reused.0),
            non_manifold_edge_count(&reused.0)
        );
        per_face(&topo, moved, &reused, &moved_fresh);
    }
    Ok(())
}
