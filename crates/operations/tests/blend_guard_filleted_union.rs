//! Public reproduction of a filleted union's blocking blend guards.
//!
//! A 30×18×26 box at the origin fused with an r6 × 35 cylinder on the z axis,
//! then an R5 fillet on the five junction edges: the cylinder wall against
//! the box planes `x = 0`, `y = 0` and `z = 26`, plus the two top edges
//! `z = 26 ∩ x = 0` and `z = 26 ∩ y = 0` that meet it. The band is NURBS, so
//! the fillet pays both the convex-material sampler
//! (`blend.material_guard`) and the volume guard (`blend.volume_guard`).
//!
//! `filleted_union_guard_profile` is ignored; run it in an optimized build
//! to print the guard spans:
//! `cargo test --profile profiling -p remus-operations --test
//! blend_guard_filleted_union -- --ignored --nocapture`
//!
//! It enables the module caches the WASM kernel enables (set
//! `REMUS_GUARD_PROFILE_COLD` to leave them off), runs each fillet on a fresh
//! thread so those caches start cold as on a drag's first replay, and takes
//! `REMUS_GUARD_PROFILE_RUNS` (default 7) runs. `REMUS_GUARD_PROFILE_BITS`
//! also prints the result's volume bits, for comparing builds.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout,
    missing_docs
)]

use std::sync::OnceLock;
use std::time::Instant;

use remus_math::compact_sort::CompactSort;
use remus_operations::blend_ops::fillet_with_evolution;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::performance;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_edges;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

/// Which junction plane a face lies on, if any.
fn junction_plane(topo: &Topology, face: FaceId) -> Option<&'static str> {
    let FaceSurface::Plane { normal, d } = topo.face(face).unwrap().surface() else {
        return None;
    };
    let near = |a: f64, b: f64| (a - b).abs() < 1e-9;
    let axis = [normal.x(), normal.y(), normal.z()];
    let along = |k: usize| {
        axis.iter()
            .enumerate()
            .all(|(i, v)| near(v.abs(), f64::from(u8::from(i == k))))
    };
    if along(0) && near(*d, 0.0) {
        Some("x-min")
    } else if along(1) && near(*d, 0.0) {
        Some("y-min")
    } else if along(2) && near(d.abs(), 26.0) {
        Some("z-max")
    } else {
        None
    }
}

fn is_cylinder(topo: &Topology, face: FaceId) -> bool {
    matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_))
}

/// The fused body and its five junction edges.
fn filleted_union_input() -> (Topology, SolidId, Vec<EdgeId>) {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 30.0, 18.0, 26.0).unwrap();
    let post = make_cylinder(&mut topo, 6.0, 35.0).unwrap();
    let body = boolean(&mut topo, BooleanOp::Fuse, block, post).unwrap();
    let adjacency = topo.build_adjacency(body).unwrap();
    let edges: Vec<EdgeId> = solid_edges(&topo, body)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let faces = adjacency.faces_for_edge(edge);
            if faces.len() != 2 {
                return false;
            }
            let planes: Vec<_> = faces.iter().map(|&f| junction_plane(&topo, f)).collect();
            let cylinders = faces.iter().filter(|&&f| is_cylinder(&topo, f)).count();
            // Cylinder wall against a junction plane, or the top meeting x/y-min.
            (cylinders == 1 && planes.iter().any(Option::is_some))
                || (planes.contains(&Some("z-max"))
                    && (planes.contains(&Some("x-min")) || planes.contains(&Some("y-min"))))
        })
        .collect();
    assert_eq!(edges.len(), 5, "expected the five junction edges");
    (topo, body, edges)
}

/// The module caches the WASM kernel enables (`RemusKernel::with_module_caches`),
/// with the thread crossing memo standing in for the session's scoped one.
fn enable_wasm_module_caches() {
    remus_check::properties::face_cache::enable_thread_face_cache();
    remus_operations::measure::enable_thread_volume_memo();
    remus_operations::measure::enable_default_measurement_reuse();
    remus_operations::measure::enable_thread_nurbs_area_memo();
    remus_operations::measure::enable_thread_face_box_memo();
    remus_operations::tessellate::enable_face_mesh_cache();
    remus_operations::tessellate::enable_mesh_reuse();
    remus_algo::pcurve_memo::enable_thread_pcurve_memo();
    remus_algo::wire_area_memo::enable_thread_wire_area_memo();
    remus_algo::ray_geom_memo::enable_thread_ray_geom_memo();
    remus_operations::query::enable_thread_crossing_memo();
}

fn clock_ms() -> f64 {
    static EPOCH: OnceLock<Instant> = OnceLock::new();
    EPOCH.get_or_init(Instant::now).elapsed().as_secs_f64() * 1e3
}

#[test]
fn filleted_union_fillet_succeeds_with_nurbs_band() {
    let (mut topo, body, edges) = filleted_union_input();
    let before = solid_volume(&topo, body, 0.01).unwrap();
    let (result, _) = fillet_with_evolution(&mut topo, body, &edges, 5.0).unwrap();
    assert!(!result.is_partial);
    let after = solid_volume(&topo, result.solid, 0.01).unwrap();
    assert!(after < before, "a convex fillet removes material");
}

#[test]
#[ignore = "timing probe; run in an optimized build with --nocapture"]
fn filleted_union_guard_profile() {
    let runs: usize = std::env::var("REMUS_GUARD_PROFILE_RUNS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(7);
    let mut totals: Vec<(&'static str, Vec<f64>)> = Vec::new();
    let mut calls: Vec<(&'static str, usize)> = Vec::new();
    let cold = std::env::var_os("REMUS_GUARD_PROFILE_COLD").is_some();
    for _ in 0..runs {
        // A fresh thread per run: the module caches are thread-local, and a
        // drag replays the fillet on a body the session has not seen.
        let (elapsed, phases) = std::thread::spawn(move || {
            if !cold {
                enable_wasm_module_caches();
            }
            let (mut topo, body, edges) = filleted_union_input();
            performance::configure(Some(clock_ms));
            let started = clock_ms();
            let (result, _) = fillet_with_evolution(&mut topo, body, &edges, 5.0).unwrap();
            let elapsed = clock_ms() - started;
            let phases = performance::drain();
            performance::configure(None);
            if std::env::var_os("REMUS_GUARD_PROFILE_BITS").is_some() {
                let volume = solid_volume(&topo, result.solid, 0.1).unwrap();
                println!("result volume {volume} bits {:#018x}", volume.to_bits());
            }
            (elapsed, phases)
        })
        .join()
        .unwrap();
        let mut run: Vec<(&'static str, f64)> = vec![("fillet_with_evolution", elapsed)];
        for phase in phases {
            if let Some(slot) = calls.iter_mut().find(|(n, _)| *n == phase.name) {
                slot.1 += 1;
            } else {
                calls.push((phase.name, 1));
            }
            if let Some(slot) = run.iter_mut().find(|(n, _)| *n == phase.name) {
                slot.1 += phase.duration_ms;
            } else {
                run.push((phase.name, phase.duration_ms));
            }
        }
        for (name, ms) in run {
            if let Some(slot) = totals.iter_mut().find(|(n, _)| *n == name) {
                slot.1.push(ms);
            } else {
                totals.push((name, vec![ms]));
            }
        }
    }
    println!("span (median of {runs} runs, ms, summed per run)");
    for (name, mut samples) in totals {
        samples.compact_sort_by(f64::total_cmp);
        let median = samples[samples.len() / 2];
        if median >= 0.5 {
            let count = calls
                .iter()
                .find(|(n, _)| *n == name)
                .map_or(1, |c| c.1 / runs);
            println!("{median:>9.2}  {name} ×{count}");
        }
    }
}
