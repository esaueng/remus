//! Time the pieces of a planar face move on an imported STEP body.
//!
//! Diagnostic for the OpenZCAD "offset face" direct edit: import the fixture,
//! pick the +X planar face closest to a target area, then time each public
//! kernel stage the application path touches.
//!
//! Run with
//! `cargo run --profile profiling -p remus --example offset_face_perf -- [step] [distance] [area]`.

#![allow(
    clippy::print_stdout,
    clippy::print_stderr,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]

use std::time::Instant;

use remus_io::step::reader::read_step;
use remus_operations::journal_ops::move_faces_journaled;
use remus_operations::measure::{face_area, mass_properties, solid_bounding_box, solid_volume};
use remus_operations::push_pull::move_faces;
use remus_operations::tessellate::tessellate_solid;
use remus_operations::validate::{validate_solid, validate_solid_relaxed};
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

fn surface_census(topo: &Topology, solid: SolidId) -> String {
    let mut planes = 0;
    let mut cyl = 0;
    let mut nurbs = 0;
    let mut other = 0;
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => planes += 1,
            FaceSurface::Cylinder(_) => cyl += 1,
            FaceSurface::Nurbs(_) => nurbs += 1,
            _ => other += 1,
        }
    }
    format!("planes={planes} cylinders={cyl} nurbs={nurbs} other={other}")
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .unwrap_or_else(|| "crates/io/tests/data/shapr3d_hammer_holder.step".to_string());
    let distance: f64 = args.next().map_or(-6.0, |v| v.parse().unwrap());
    let target_area: f64 = args.next().map_or(1045.93, |v| v.parse().unwrap());

    let text = std::fs::read_to_string(&path)?;
    let mut topo = Topology::new();
    let solids = timed("read_step", || read_step(&text, &mut topo))?;
    let solid = solids[0];
    let faces = solid_faces(&topo, solid)?;
    println!(
        "solids={} faces={} {}",
        solids.len(),
        faces.len(),
        surface_census(&topo, solid)
    );

    // Pick the +X planar face whose area is closest to the target.
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
    for (face, area) in candidates.iter().take(5) {
        println!("  +X plane face {} area {area:.3}", face.index());
    }
    let face = candidates[0].0;
    println!("selected face {} distance {distance}", face.index());

    let bbox = solid_bounding_box(&topo, solid)?;
    let extent = bbox.max - bbox.min;
    let display_deflection = (extent.x().max(extent.y()).max(extent.z()) * 2e-4).max(1e-5);
    println!(
        "bbox extent {:.2} x {:.2} x {:.2}",
        extent.x(),
        extent.y(),
        extent.z()
    );

    if std::env::var_os("OFFSET_PERF_ONLY_MOVE").is_some() {
        let runs: usize = std::env::var("OFFSET_PERF_ONLY_MOVE")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(5);
        for _ in 0..runs {
            let mut t3 = topo.clone();
            let _ = timed("push_pull::move_faces", || {
                move_faces(&mut t3, solid, &[face], distance)
            });
        }
        return Ok(());
    }
    if std::env::var_os("OFFSET_PERF_ONLY_VALIDATE").is_some() {
        for _ in 0..5 {
            let _ = timed("validate_solid (strict)", || validate_solid(&topo, solid));
        }
        return Ok(());
    }

    println!("--- source body ---");
    let v0 = timed("solid_volume(0.08)", || solid_volume(&topo, solid, 0.08))?;
    println!("  volume {v0:.3}");
    timed("validate_solid_relaxed", || {
        validate_solid_relaxed(&topo, solid)
    })?;
    let report = timed("validate_solid (strict)", || validate_solid(&topo, solid))?;
    println!(
        "  strict valid={} errors={}",
        report.is_valid(),
        report.error_count()
    );
    let props = timed("mass_properties", || mass_properties(&topo, solid))?;
    println!("  mass {:.3}", props.mass);
    let mesh = timed("tessellate_solid(display)", || {
        tessellate_solid(&topo, solid, display_deflection)
    })?;
    println!("  display tris {}", mesh.indices.len() / 3);
    timed("tessellate_solid(0.08)", || {
        tessellate_solid(&topo, solid, 0.08)
    })?;

    println!("--- move_faces (plain) ---");
    let mut t1 = topo.clone();
    let moved = timed("push_pull::move_faces", || {
        move_faces(&mut t1, solid, &[face], distance)
    });
    match &moved {
        Ok(result) => {
            println!(
                "  result faces={} {}",
                solid_faces(&t1, *result)?.len(),
                surface_census(&t1, *result)
            );
            let v1 = timed("solid_volume(result, 0.08)", || {
                solid_volume(&t1, *result, 0.08)
            })?;
            println!("  volume {v1:.3} delta {:.3}", v1 - v0);
            timed("validate_solid_relaxed(result)", || {
                validate_solid_relaxed(&t1, *result)
            })?;
            timed("validate_solid(result, strict)", || {
                validate_solid(&t1, *result)
            })?;
            timed("mass_properties(result)", || mass_properties(&t1, *result))?;
            timed("tessellate_solid(result, display)", || {
                tessellate_solid(&t1, *result, display_deflection)
            })?;
        }
        Err(error) => println!("  move_faces failed: {error}"),
    }

    println!("--- move_faces_journaled ---");
    let mut t2 = topo.clone();
    let journaled = timed("journal_ops::move_faces_journaled", || {
        move_faces_journaled(&mut t2, solid, &[face], distance)
    });
    match journaled {
        Ok(op) => println!(
            "  result solid {} faces={}",
            op.solid.index(),
            solid_faces(&t2, op.solid)?.len()
        ),
        Err(error) => println!("  journaled move failed: {error}"),
    }

    if std::env::var_os("OFFSET_PERF_REPEAT").is_some() {
        println!("--- repeat move_faces x3 ---");
        for _ in 0..3 {
            let mut t3 = topo.clone();
            let _ = timed("push_pull::move_faces", || {
                move_faces(&mut t3, solid, &[face], distance)
            });
        }
    }
    Ok(())
}
