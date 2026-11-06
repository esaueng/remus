//! Per-phase timing of blend-group direct edits on the filleted
//! Gentle-Gecko part (the reference video's A2 band radius and A7 boss
//! diameter edits), on the plain filleted part and on the cumulative body
//! (A2 R1.9, A3 R1, A4 0.7 and A6 0.8 chamfer chains).
//!
//! ```text
//! cargo run --profile profiling -p remus-operations --example blend_group_edit_profile
//! cargo run --profile profiling -p remus-operations --example blend_group_edit_profile -- a7-cumulative
//! ```
//!
//! Scenarios: `a2`, `a7`, `a7-cumulative`, `a2-cumulative`, `all` (default).
//! The WASM kernel's measurement caches are on (as the kernel enables them;
//! `NO_KERNEL_CACHES=1` turns them off); `VERBOSE=1` lists the phases over 5 ms.
//! Each edit prints its wall time and the summed `remus_operations::performance`
//! phases it ran.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::print_stdout)]

use std::collections::BTreeMap;
use std::sync::OnceLock;
use std::time::Instant;

use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_cluster::{Convexity, PreparedGroupEdit, recognize_blend_cluster};
use remus_operations::blend_ops::{chamfer_v2_tangent_chain, fillet_cascade};
use remus_operations::performance;
use remus_operations::push_pull::resize_cylindrical_face;
use remus_operations::resize_blend::resize_blend;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::solid_edges;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

static START: OnceLock<Instant> = OnceLock::new();

fn clock() -> f64 {
    START.get_or_init(Instant::now).elapsed().as_secs_f64() * 1e3
}

fn gecko() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let text = include_str!("../tests/data/gentle_gecko_filleted.step");
    let solid = remus_io::step::reader::read_step(text, &mut topo).unwrap()[0];
    (topo, solid)
}

fn boss_of(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
                if c.axis().cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9
                    && c.origin().x().abs() < 1e-9
                    && c.origin().y().abs() < 1e-9)
        })
        .unwrap()
}

fn x0_band(topo: &Topology, solid: SolidId) -> FaceId {
    let cluster = recognize_blend_cluster(topo, solid, boss_of(topo, solid)).unwrap();
    cluster
        .members
        .iter()
        .filter(|m| m.convexity == Convexity::Convex)
        .flat_map(|m| m.faces.iter().copied())
        .find(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
            if c.axis().cross(Vec3::new(0.0, 1.0, 0.0)).length() < 1e-9)
        })
        .unwrap()
}

fn line_where(topo: &Topology, solid: SolidId, on: impl Fn(Point3) -> bool) -> EdgeId {
    let found: Vec<EdgeId> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Line))
        .filter(|&edge| {
            let data = topo.edge(edge).unwrap();
            on(topo.vertex(data.start()).unwrap().point())
                && on(topo.vertex(data.end()).unwrap().point())
        })
        .collect();
    assert_eq!(found.len(), 1);
    found[0]
}

/// A2 R1.9 → A3 R1 → A4 0.7 → A6 0.8 on the filleted Gecko.
fn cumulative() -> (Topology, SolidId) {
    let (mut topo, mut solid) = gecko();
    let band = x0_band(&topo, solid);
    solid = resize_blend(&mut topo, solid, band, 4.0, 1.9)
        .unwrap()
        .solid;
    let a3 = line_where(&topo, solid, |p| {
        p.x().abs() < 1e-7 && (p.y() - 18.0).abs() < 1e-7
    });
    solid = fillet_cascade(&mut topo, solid, &[a3], 1.0).unwrap().solid;
    let a4 = line_where(&topo, solid, |p| {
        (p.x() - 30.0).abs() < 1e-7 && (p.z() - 24.0).abs() < 1e-7
    });
    solid = chamfer_v2_tangent_chain(&mut topo, solid, &[a4], 0.7, 0.7)
        .unwrap()
        .solid;
    let a6 = line_where(&topo, solid, |p| p.y().abs() < 1e-7 && p.z().abs() < 1e-7);
    solid = chamfer_v2_tangent_chain(&mut topo, solid, &[a6], 0.8, 0.8)
        .unwrap()
        .solid;
    (topo, solid)
}

fn report(label: &str, millis: f64) {
    let mut phases: BTreeMap<&'static str, (usize, f64)> = BTreeMap::new();
    let drained = performance::drain();
    for phase in drained.iter().cloned() {
        let entry = phases.entry(phase.name).or_default();
        entry.0 += 1;
        entry.1 += phase.duration_ms;
    }
    if std::env::var_os("VERBOSE").is_some() {
        for phase in &drained {
            if phase.duration_ms > 5.0
                && (phase.name.starts_with("blend_group")
                    || phase.name == "volume.total"
                    || phase.name == "mesh.total")
            {
                println!(
                    "      {:>9.1} {:<30} {:>8.1}",
                    phase.start_ms, phase.name, phase.duration_ms
                );
            }
        }
    }
    println!("{label}: {millis:.1} ms");
    for (name, (count, total)) in phases {
        println!("    {name:<36} x{count:<3} {total:>9.1} ms");
    }
}

fn timed<T>(label: &str, run: impl FnOnce() -> T) -> T {
    let _ = performance::drain();
    let start = clock();
    let value = run();
    report(label, clock() - start);
    value
}

fn a2(topo: Topology, solid: SolidId, tag: &str) {
    for radius in [1.5, 1.9] {
        let mut topo = topo.clone();
        let band = x0_band(&topo, solid);
        let current = match topo.face(band).unwrap().surface() {
            FaceSurface::Cylinder(c) => c.radius(),
            _ => unreachable!(),
        };
        let result = timed(
            &format!("{tag} A2 R{current}->{radius} (unprepared)"),
            || resize_blend(&mut topo, solid, band, current, radius),
        );
        if let Err(error) = result {
            println!("    refused: {error}");
        }
    }
    let mut topo = topo;
    let band = x0_band(&topo, solid);
    let prepared = timed(&format!("{tag} A2 prepare"), || {
        let current = match topo.face(band).unwrap().surface() {
            FaceSurface::Cylinder(c) => c.radius(),
            _ => unreachable!(),
        };
        PreparedGroupEdit::prepare_band(&topo, solid, band, current)
    });
    let Ok(prepared) = prepared else {
        println!("    refused: {:?}", prepared.err());
        return;
    };
    println!("    {prepared:?}");
    for radius in [1.5, 1.9, 1.5, 2.5] {
        let frame = timed(&format!("{tag} A2 frame R{radius}"), || {
            prepared.preview(&mut topo, radius)
        });
        if let Err(error) = frame {
            println!("    refused: {error}");
        }
    }
    let commit = timed(&format!("{tag} A2 commit R1.9"), || {
        prepared.commit(&mut topo, 1.9)
    });
    if let Err(error) = commit {
        println!("    refused: {error}");
    }
}

fn a7(topo: Topology, solid: SolidId, tag: &str) {
    {
        let mut topo = topo.clone();
        let mut solid = solid;
        for diameter in [14.6, 17.0, 21.1, 14.7] {
            let boss = boss_of(&topo, solid);
            let result = timed(
                &format!("{tag} A7 Ø{diameter} (unprepared, chained)"),
                || resize_cylindrical_face(&mut topo, solid, boss, diameter / 2.0),
            );
            match result {
                Ok(next) => solid = next,
                Err(error) => println!("    refused: {error}"),
            }
        }
    }
    let mut topo = topo;
    let boss = boss_of(&topo, solid);
    let prepared = timed(&format!("{tag} A7 prepare"), || {
        PreparedGroupEdit::prepare_cylinder(&topo, solid, boss)
    });
    let Ok(prepared) = prepared else {
        println!("    refused: {:?}", prepared.err());
        return;
    };
    println!("    {prepared:?}");
    for diameter in [14.6, 17.0, 21.1, 14.7, 15.0, 15.4] {
        let frame = timed(&format!("{tag} A7 frame Ø{diameter}"), || {
            prepared.preview(&mut topo, diameter / 2.0)
        });
        if let Err(error) = frame {
            println!("    refused: {error}");
        }
    }
    let commit = timed(&format!("{tag} A7 commit Ø14.7"), || {
        prepared.commit(&mut topo, 14.7 / 2.0)
    });
    if let Err(error) = commit {
        println!("    refused: {error}");
    }
}

fn main() {
    performance::configure(Some(clock));
    if std::env::var_os("NO_KERNEL_CACHES").is_none() {
        // What the WASM kernel enables on construction.
        remus_operations::measure::enable_thread_volume_memo();
        remus_operations::measure::enable_default_measurement_reuse();
        remus_operations::tessellate::enable_face_mesh_cache();
    }
    let scenario = std::env::args().nth(1).unwrap_or_else(|| "all".into());
    let run = |name: &str| scenario == "all" || scenario == name;
    if run("a2") {
        let (topo, solid) = gecko();
        a2(topo, solid, "gecko");
    }
    if run("a7") {
        let (topo, solid) = gecko();
        a7(topo, solid, "gecko");
    }
    if run("a7-cumulative") || run("a2-cumulative") {
        let (topo, solid) = timed("build cumulative body", cumulative);
        if run("a7-cumulative") {
            a7(topo.clone(), solid, "cumulative");
        }
        if run("a2-cumulative") {
            a2(topo, solid, "cumulative");
        }
    }
}
