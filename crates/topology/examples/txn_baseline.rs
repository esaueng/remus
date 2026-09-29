//! Baseline transaction-cost probe for PERF-T02/PERF-T03.
//!
//! Measures the *current* (full-snapshot) implementation before any change:
//! a fixed local edit (commit and rollback) and repeated primitive
//! construction against increasing unrelated document sizes, with large
//! NURBS payload and retained-checkpoint variants.
//!
//! Each timed cell runs one `run_transacted` scope per operation, so the
//! snapshot count per operation is exactly one full `Topology::clone` by
//! construction (see `RollbackSnapshot::capture`). The harness reports wall
//! time plus logical sizes; "payload bytes" is a documented estimator over
//! public accessors (stack sizes plus NURBS heap), a proxy for clone traffic,
//! not allocator bytes. Peak RSS is the process VmHWM high-water mark and
//! includes setup; it is reported for shape, not attributed to snapshots.

#![allow(
    clippy::print_stdout,
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs
)]

use std::time::Instant;

use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point2, Point3, Vec2, Vec3};
use remus_topology::Topology;
use remus_topology::attributes::EntityAttributes;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::pcurve::PCurve;
use remus_topology::shell::Shell;
use remus_topology::solid::SolidId;
use remus_topology::transaction::run_transacted;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const TOL: f64 = 1e-7;
const REPS: usize = 11;

fn raw_box(topo: &mut Topology, ox: f64) -> SolidId {
    let v = [
        topo.add_vertex(Vertex::new(Point3::new(ox, 0.0, 0.0), TOL)),
        topo.add_vertex(Vertex::new(Point3::new(ox + 1.0, 0.0, 0.0), TOL)),
        topo.add_vertex(Vertex::new(Point3::new(ox + 1.0, 1.0, 0.0), TOL)),
        topo.add_vertex(Vertex::new(Point3::new(ox, 1.0, 0.0), TOL)),
        topo.add_vertex(Vertex::new(Point3::new(ox, 0.0, 1.0), TOL)),
        topo.add_vertex(Vertex::new(Point3::new(ox + 1.0, 0.0, 1.0), TOL)),
        topo.add_vertex(Vertex::new(Point3::new(ox + 1.0, 1.0, 1.0), TOL)),
        topo.add_vertex(Vertex::new(Point3::new(ox, 1.0, 1.0), TOL)),
    ];
    let e = [
        topo.add_edge(Edge::new(v[0], v[1], EdgeCurve::Line)),
        topo.add_edge(Edge::new(v[1], v[2], EdgeCurve::Line)),
        topo.add_edge(Edge::new(v[2], v[3], EdgeCurve::Line)),
        topo.add_edge(Edge::new(v[3], v[0], EdgeCurve::Line)),
        topo.add_edge(Edge::new(v[4], v[5], EdgeCurve::Line)),
        topo.add_edge(Edge::new(v[5], v[6], EdgeCurve::Line)),
        topo.add_edge(Edge::new(v[6], v[7], EdgeCurve::Line)),
        topo.add_edge(Edge::new(v[7], v[4], EdgeCurve::Line)),
        topo.add_edge(Edge::new(v[0], v[4], EdgeCurve::Line)),
        topo.add_edge(Edge::new(v[1], v[5], EdgeCurve::Line)),
        topo.add_edge(Edge::new(v[2], v[6], EdgeCurve::Line)),
        topo.add_edge(Edge::new(v[3], v[7], EdgeCurve::Line)),
    ];
    let mk_face = |topo: &mut Topology, edges: [(remus_topology::edge::EdgeId, bool); 4]| {
        let wire = Wire::new(
            edges
                .iter()
                .map(|&(id, fwd)| OrientedEdge::new(id, fwd))
                .collect(),
            true,
        )
        .unwrap();
        let wid = topo.add_wire(wire);
        topo.add_face(Face::new(
            wid,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    };
    let faces = [
        mk_face(
            topo,
            [(e[0], true), (e[1], true), (e[2], true), (e[3], true)],
        ),
        mk_face(
            topo,
            [(e[4], true), (e[5], true), (e[6], true), (e[7], true)],
        ),
        mk_face(
            topo,
            [(e[0], true), (e[9], true), (e[4], false), (e[8], false)],
        ),
        mk_face(
            topo,
            [(e[2], true), (e[11], true), (e[6], false), (e[10], false)],
        ),
        mk_face(
            topo,
            [(e[3], true), (e[8], true), (e[7], false), (e[11], false)],
        ),
        mk_face(
            topo,
            [(e[1], true), (e[10], true), (e[5], false), (e[9], false)],
        ),
    ];
    let shell = topo.add_shell(Shell::new(faces.to_vec()).unwrap());
    topo.add_solid(remus_topology::solid::Solid::new(shell, vec![]))
}

fn flat_nurbs_patch(n: usize, z: f64) -> NurbsSurface {
    let mut control = Vec::with_capacity(n);
    let mut weights = Vec::with_capacity(n);
    for i in 0..n {
        let mut row = Vec::with_capacity(n);
        let mut wrow = Vec::with_capacity(n);
        for j in 0..n {
            row.push(Point3::new(i as f64, j as f64, z));
            wrow.push(1.0);
        }
        control.push(row);
        weights.push(wrow);
    }
    let knots = |m: usize| {
        let mut k = vec![0.0; 4];
        for i in 1..m - 3 {
            k.push(i as f64);
        }
        k.extend(vec![(m - 3) as f64; 4]);
        k
    };
    NurbsSurface::new(3, 3, knots(n), knots(n), control, weights).unwrap()
}

/// Rough live-payload estimate over public accessors: stack sizes plus NURBS
/// heap (control points, weights, knots). Analytic carriers contribute their
/// stack size only. A full `Topology::clone` copies this plus arena
/// bookkeeping and map storage, so this is a lower-bound proxy for clone
/// traffic, reported for shape across document sizes.
fn estimate_live_payload_bytes(topo: &Topology) -> usize {
    let mut bytes = 0usize;
    bytes += topo.num_vertices() * 32;
    for (_, edge) in topo.edges().iter() {
        bytes += std::mem::size_of_val(edge);
        if let EdgeCurve::NurbsCurve(n) = edge.curve() {
            bytes += n.control_points().len() * 24 + n.weights().len() * 8 + n.knots().len() * 8;
        }
    }
    for (_, wire) in topo.wires().iter() {
        bytes += std::mem::size_of_val(wire) + wire.edges().len() * 16;
    }
    for (_, face) in topo.faces().iter() {
        bytes += std::mem::size_of_val(face);
        if let FaceSurface::Nurbs(n) = face.surface() {
            let pts: usize = n.control_points().iter().map(Vec::len).sum();
            bytes += pts * 24 + pts * 8 + (n.knots_u().len() + n.knots_v().len()) * 8;
        }
    }
    for (_, shell) in topo.shells().iter() {
        bytes += std::mem::size_of_val(shell) + shell.faces().len() * 8;
    }
    for (_, solid) in topo.solids().iter() {
        bytes += std::mem::size_of_val(solid) + solid.inner_shells().len() * 8;
    }
    for (face_id, _) in topo.faces().iter() {
        if let Some(loops) = topo.loops_of_face(face_id) {
            for &lid in loops {
                if let Ok(l) = topo.face_loop(lid) {
                    bytes += std::mem::size_of_val(l) + l.coedges().len() * 4;
                    for &cid in l.coedges() {
                        if let Ok(c) = topo.coedge(cid) {
                            bytes += std::mem::size_of_val(c);
                            if let Some(pc) = c.pcurve() {
                                bytes += std::mem::size_of_val(pc);
                                if let Curve2D::Nurbs(n) = pc.curve() {
                                    bytes += n.control_points().len() * 16
                                        + n.weights().len() * 8
                                        + n.knots().len() * 8;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    bytes += topo.journal().len() * 96;
    bytes
}

fn vm_hwm_kb() -> Option<usize> {
    let status = std::fs::read_to_string("/proc/self/status").ok()?;
    for line in status.lines() {
        if let Some(rest) = line.strip_prefix("VmHWM:") {
            return rest.split_whitespace().next()?.parse().ok();
        }
    }
    None
}

fn line_pcurve() -> PCurve {
    PCurve::new(
        Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
        0.0,
        1.0,
    )
}

fn median(mut xs: Vec<f64>) -> f64 {
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    xs[xs.len() / 2]
}

/// Fixed local edit: touch one vertex, one face attribute, one pcurve and one
/// journal entry, then `fail` decides commit vs rollback. Every touch is
/// O(touched state): single-slot writes plus one journal entry.
fn fixed_edit(topo: &mut Topology, fail: bool) -> Result<(), String> {
    run_transacted(topo, |t| {
        let v = t
            .vertices()
            .iter()
            .next()
            .map(|(id, _)| id)
            .ok_or("empty")?;
        t.vertex_mut(v)
            .map_err(|e| e.to_string())?
            .set_point(Point3::new(7.0, 8.0, 9.0));
        let f = t.faces().iter().next().map(|(id, _)| id).ok_or("empty")?;
        t.set_face_attributes(
            f,
            EntityAttributes {
                name: Some("edit".to_owned()),
                color: None,
            },
        )
        .map_err(|e| e.to_string())?;
        let e = t.edges().iter().next().map(|(id, _)| id).ok_or("empty")?;
        // A genuine pcurve write on the face's own boundary use; skipped
        // when the first edge is not singly used by the first face.
        let _ = t.set_pcurve(e, f, line_pcurve());
        let pending = t.journal_begin("probe-edit");
        t.journal_record_barrier(pending, vec![]);
        if fail {
            Err("injected".to_owned())
        } else {
            Ok(())
        }
    })
}

fn build_template(boxes: usize, nurbs_faces: usize, nurbs_n: usize) -> Topology {
    let mut topo = Topology::new();
    for i in 0..boxes {
        raw_box(&mut topo, i as f64 * 3.0);
    }
    for i in 0..nurbs_faces {
        remus_topology::builder::make_nurbs_face(
            &mut topo,
            flat_nurbs_patch(nurbs_n, 100.0 + i as f64),
            TOL,
        )
        .unwrap();
    }
    // One pcurve so the edit workload has pcurve-adjacent traffic.
    let first_edge = topo.edges().iter().next().map(|(id, _)| id);
    let first_face = topo.faces().iter().next().map(|(id, _)| id);
    if let (Some(e), Some(f)) = (first_edge, first_face) {
        let _ = topo.set_pcurve(e, f, line_pcurve());
    }
    topo
}

fn cell(name: &str, template: &Topology, retain_checkpoint: bool, workload: &str) {
    let slots = template.allocated_slot_count();
    let payload = estimate_live_payload_bytes(template);
    let mut samples = Vec::with_capacity(REPS);
    for _ in 0..REPS {
        let mut topo = template.clone();
        let _checkpoint = retain_checkpoint.then(|| template.clone());
        // Headroom for the workload outside the timer, so arena growth
        // reallocations (construction cost, not transaction cost) do not
        // pollute the scaling shape. Covers 150 box builds with margin.
        topo.reserve(2000, 3000, 1500, 1500, 300, 300);
        let start = Instant::now();
        match workload {
            "edit_rollback" => {
                let _ = fixed_edit(&mut topo, true);
            }
            "edit_commit" => {
                let _ = fixed_edit(&mut topo, false);
            }
            "box_build" => {
                let ox = 1_000_000.0;
                let _ = run_transacted(&mut topo, |t| {
                    raw_box(t, ox);
                    Ok::<(), String>(())
                });
            }
            "box_build_150" => {
                for i in 0..150 {
                    let ox = 1_000_000.0 + i as f64 * 3.0;
                    let _ = run_transacted(&mut topo, |t| {
                        raw_box(t, ox);
                        Ok::<(), String>(())
                    });
                }
            }
            "box_naked" => {
                raw_box(&mut topo, 1_000_000.0);
            }
            "txn_empty" => {
                let _ = run_transacted(&mut topo, |_| Ok::<(), String>(()));
            }
            _ => {}
        }
        samples.push(start.elapsed().as_secs_f64() * 1000.0);
    }
    let med = median(samples);
    let hwm = vm_hwm_kb().map_or_else(|| "null".to_owned(), |kb| kb.to_string());
    println!(
        "{{\"cell\":\"{name}\",\"workload\":\"{workload}\",\"slots\":{slots},\
         \"payload_bytes\":{payload},\"scopes_per_op\":1,\"clones_per_op\":1,\
         \"median_ms\":{med:.3},\"reps\":{REPS},\"vmhwm_kb\":{hwm}}}"
    );
}

fn main() {
    let impl_tag = std::env::var("TXN_IMPL").unwrap_or_else(|_| "unknown".to_owned());
    println!("{{\"harness\":\"txn_baseline\",\"impl\":\"{impl_tag}\"}}");
    let only: Vec<String> = std::env::args().skip(1).collect();
    // (boxes, nurbs_faces, nurbs_n, tag)
    let docs = [
        (1usize, 0usize, 0usize, "analytic"),
        (100, 0, 0, "analytic"),
        (1000, 0, 0, "analytic"),
    ];
    let workloads = [
        "edit_rollback",
        "edit_commit",
        "box_build",
        "box_build_150",
        "box_naked",
        "txn_empty",
    ];
    for &(boxes, nurbs_faces, nurbs_n, tag) in &docs {
        let template = build_template(boxes, nurbs_faces, nurbs_n);
        for &cp in &[false, true] {
            let cp_tag = if cp { "cp1" } else { "cp0" };
            for workload in workloads {
                let cell_name = format!("{tag}-boxes{boxes}-{cp_tag}");
                if !only.is_empty()
                    && !only
                        .iter()
                        .any(|f| cell_name.contains(f) | workload.contains(f))
                {
                    continue;
                }
                cell(&cell_name, &template, cp, workload);
            }
        }
    }
    let template = build_template(100, 24, 48);
    for &cp in &[false, true] {
        let cp_tag = if cp { "cp1" } else { "cp0" };
        for workload in ["edit_rollback", "edit_commit", "box_build"] {
            let cell_name = format!("nurbs-boxes100+n24x48-{cp_tag}");
            if !only.is_empty()
                && !only
                    .iter()
                    .any(|f| cell_name.contains(f) | workload.contains(f))
            {
                continue;
            }
            cell(&cell_name, &template, cp, workload);
        }
    }
}
