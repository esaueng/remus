//! Native probe for the r 2 fillet on the seam of a cylinder boss fused into
//! a sphere (OpenZCAD K02/W01).
//!
//! Prints the fuse, fillet and display-tessellation timings, the fillet's
//! walker step count, and the display mesh's closure at OpenZCAD's display
//! settings. Options:
//!
//! - `--tilted`: tilt the boss 20° about X and 10° about Y before placement.
//! - `--v2`: run `fillet_v2` instead of the cascade.
//! - `PHASES=1`: aggregate the operations performance trace by phase.
//! - `OPEN_DETAIL=1`: attribute open mesh edges to faces and measure how far
//!   each NURBS contact edge lies from its analytic support.
//!
//! ```sh
//! cargo run --profile profiling -p remus-operations --example sphere_boss_fillet_profile -- --tilted
//! ```
#![allow(
    clippy::print_stdout,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]

use std::collections::{BTreeMap, HashMap};
use std::time::Instant;

use remus_math::compact_sort::CompactSort;
use remus_math::context::OperationContext;
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{self, BooleanOp};
use remus_operations::{blend_ops, primitives, tessellate, transform};
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

/// OpenZCAD's display tessellation for this body (2e-4 × its ~147 mm extent).
const DISPLAY_DEFLECTION: f64 = 0.0294;
const DISPLAY_ANGULAR: f64 = 0.06;
const EXTENT: f64 = 147.0;

fn clock() -> f64 {
    static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_secs_f64() * 1000.0
}

fn main() {
    let tilted = std::env::args().any(|a| a == "--tilted");
    let v2 = std::env::args().any(|a| a == "--v2");
    let phases = std::env::var("PHASES").is_ok();
    let detail = std::env::var("OPEN_DETAIL").is_ok();

    let mut topo = Topology::new();
    let sphere = primitives::make_sphere(&mut topo, 73.5, 32).unwrap();
    transform::transform_solid(&mut topo, sphere, &Mat4::translation(25.5, -1.5, 0.0)).unwrap();
    let boss = primitives::make_cylinder(&mut topo, 6.0, 64.1).unwrap();
    let place = if tilted {
        Mat4::translation(0.0, 0.0, 58.0)
            * Mat4::rotation_y(10f64.to_radians())
            * Mat4::rotation_x(20f64.to_radians())
    } else {
        Mat4::translation(0.0, 0.0, 58.0)
    };
    transform::transform_solid(&mut topo, boss, &place).unwrap();
    let t = Instant::now();
    let fused = boolean::boolean(&mut topo, BooleanOp::Fuse, sphere, boss).unwrap();
    println!("fuse: {:?}", t.elapsed());

    let adjacency = AdjacencyIndex::build(&topo, fused).unwrap();
    let kind_of = |face| kind(topo.face(face).unwrap().surface());
    let mut seams: Vec<_> = adjacency
        .edge_faces_iter()
        .filter(|(_, faces)| {
            faces.len() == 2
                && faces.iter().any(|&f| kind_of(f) == "sphere")
                && faces.iter().any(|&f| kind_of(f) == "cylinder")
        })
        .map(|(edge, _)| edge)
        .collect();
    seams.compact_sort_by_key(|e| e.index());
    println!("seam edges: {}", seams.len());

    if phases {
        remus_operations::performance::configure(Some(clock));
    }
    let context = OperationContext::new();
    let t = Instant::now();
    let governed = if v2 {
        blend_ops::fillet_v2_with_context(&mut topo, fused, &seams, 2.0, &context)
    } else {
        blend_ops::fillet_cascade_with_context(&mut topo, fused, &seams, 2.0, &context)
    };
    let elapsed = t.elapsed();
    let governed = match governed {
        Ok(governed) => governed,
        Err(error) => {
            println!("fillet error after {elapsed:?}: {error}");
            return;
        }
    };
    println!(
        "fillet ({}): {elapsed:?}, engine {:?}, {} walker steps",
        if v2 { "v2" } else { "cascade" },
        governed.result.engine,
        governed.walk_steps
    );
    if phases {
        let mut totals: BTreeMap<&str, (usize, f64)> = BTreeMap::new();
        for phase in remus_operations::performance::drain() {
            let entry = totals.entry(phase.name).or_default();
            entry.0 += 1;
            entry.1 += phase.duration_ms;
        }
        for (name, (count, ms)) in totals {
            println!("  phase {name}: n={count} {ms:.1} ms");
        }
    }

    let solid = governed.result.solid;
    let t = Instant::now();
    let (mesh, offsets) = tessellate::tessellate_solid_grouped_with_tolerance(
        &topo,
        solid,
        DISPLAY_DEFLECTION,
        DISPLAY_ANGULAR,
    )
    .unwrap();
    let elapsed = t.elapsed();
    let weld = weld_map(&mesh.positions, EXTENT * 1e-6);
    let (open, nonmanifold, flipped) = closure(&mesh.indices, &weld);
    println!(
        "display mesh: {elapsed:?}, {} triangles, welded open={open} nonmanifold={nonmanifold} flipped={flipped}",
        mesh.indices.len() / 3
    );

    if detail {
        open_edges_by_face(&topo, solid, &mesh.indices, &offsets, &weld);
        contact_deviation(&topo, solid);
    }
}

const fn kind(surface: &FaceSurface) -> &'static str {
    match surface {
        FaceSurface::Plane { .. } => "plane",
        FaceSurface::Cylinder(_) => "cylinder",
        FaceSurface::Cone(_) => "cone",
        FaceSurface::Sphere(_) => "sphere",
        FaceSurface::Torus(_) => "torus",
        FaceSurface::Nurbs(_) => "nurbs",
    }
}

/// Representative vertex per position, welding within `quantum`.
fn weld_map(positions: &[Point3], quantum: f64) -> Vec<u32> {
    let mut cells: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
    let mut map = vec![0_u32; positions.len()];
    for (index, &point) in positions.iter().enumerate() {
        let cell = (
            (point.x() / quantum).floor() as i64,
            (point.y() / quantum).floor() as i64,
            (point.z() / quantum).floor() as i64,
        );
        let mut found = None;
        'search: for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    for &other in cells
                        .get(&(cell.0 + dx, cell.1 + dy, cell.2 + dz))
                        .map_or(&[][..], Vec::as_slice)
                    {
                        if (positions[other as usize] - point).length() <= quantum {
                            found = Some(other);
                            break 'search;
                        }
                    }
                }
            }
        }
        map[index] = found.unwrap_or(index as u32);
        if found.is_none() {
            cells.entry(cell).or_default().push(index as u32);
        }
    }
    map
}

/// Welded edge uses: (open, non-manifold, inconsistently wound) edge counts.
fn closure(indices: &[u32], weld: &[u32]) -> (usize, usize, usize) {
    let mut uses: HashMap<(u32, u32), (i32, i32)> = HashMap::new();
    for triangle in indices.chunks_exact(3) {
        for k in 0..3 {
            let (a, b) = (
                weld[triangle[k] as usize],
                weld[triangle[(k + 1) % 3] as usize],
            );
            if a == b {
                continue;
            }
            let entry = uses.entry((a.min(b), a.max(b))).or_default();
            entry.0 += 1;
            entry.1 += if a < b { 1 } else { -1 };
        }
    }
    let mut counts = (0, 0, 0);
    for &(n, balance) in uses.values() {
        match n {
            1 => counts.0 += 1,
            2 if balance != 0 => counts.2 += 1,
            2 => {}
            _ => counts.1 += 1,
        }
    }
    counts
}

/// Which faces the open mesh edges belong to (`offsets` are index offsets).
fn open_edges_by_face(
    topo: &Topology,
    solid: SolidId,
    indices: &[u32],
    offsets: &[u32],
    weld: &[u32],
) {
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    let face_of = |triangle: usize| {
        offsets
            .iter()
            .rposition(|&offset| offset as usize <= triangle * 3)
            .unwrap_or(0)
    };
    let mut uses: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (triangle, corners) in indices.chunks_exact(3).enumerate() {
        for k in 0..3 {
            let (a, b) = (
                weld[corners[k] as usize],
                weld[corners[(k + 1) % 3] as usize],
            );
            uses.entry((a.min(b), a.max(b))).or_default().push(triangle);
        }
    }
    let mut per_face: BTreeMap<usize, usize> = BTreeMap::new();
    for triangles in uses.values() {
        if let [only] = triangles.as_slice() {
            *per_face.entry(face_of(*only)).or_default() += 1;
        }
    }
    for (index, face) in faces.iter().enumerate() {
        let end = offsets
            .get(index + 1)
            .map_or(indices.len(), |&o| o as usize);
        println!(
            "  face {face:?} {}: {} triangles, {} open edges",
            kind(topo.face(*face).unwrap().surface()),
            (end - offsets[index] as usize) / 3,
            per_face.get(&index).copied().unwrap_or(0)
        );
    }
}

/// How far each NURBS contact edge lies from its sphere or cylinder support,
/// against the tolerance the edge records.
fn contact_deviation(topo: &Topology, solid: SolidId) {
    let adjacency = AdjacencyIndex::build(topo, solid).unwrap();
    for (edge, faces) in adjacency.edge_faces_iter() {
        let data = topo.edge(edge).unwrap();
        let EdgeCurve::NurbsCurve(curve) = data.curve() else {
            continue;
        };
        let (a, b) = curve.domain();
        let samples: Vec<Point3> = (0..=2000)
            .map(|i| curve.evaluate(a + (b - a) * f64::from(i) / 2000.0))
            .collect();
        for &face in faces {
            let worst = match topo.face(face).unwrap().surface() {
                FaceSurface::Sphere(sphere) => samples
                    .iter()
                    .map(|&p| ((p - sphere.center()).length() - sphere.radius()).abs())
                    .fold(0.0_f64, f64::max),
                FaceSurface::Cylinder(cylinder) => samples
                    .iter()
                    .map(|&p| {
                        let (u, v) = cylinder.project_point(p);
                        (cylinder.evaluate(u, v) - p).length()
                    })
                    .fold(0.0_f64, f64::max),
                _ => continue,
            };
            println!(
                "  contact edge {edge:?} on {face:?}: degree {}, {} control points, \
                 max deviation {worst:.3e}, recorded tolerance {:?}",
                curve.degree(),
                curve.control_points().len(),
                data.tolerance()
            );
        }
    }
}
