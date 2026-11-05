//! `fuseWithEntityEvolution` / `cutWithEntityEvolution` keep a round boss's
//! seam lineage on both the direct and the batch route.
//!
//! The consumer case (OpenZCAD, Remus 2026.1.122): a boss placed with
//! `copyAndTransformSolid` in its +Z coordinate frame, fused into a block or
//! an off-pole sphere, or bored into the sphere. The piece of the boss's own
//! seam Line that survives the boolean came back `unresolved`, so the
//! consumer could not name every edge of the result. Each ring piece must be
//! `generated` by the boss wall and one face of the other operand, the seam
//! piece `modified` from the boss's seam edge, and the evolution body must be
//! the plain boolean's (face and edge counts, volume). Native coverage of the
//! same families, with the full placement and scale matrix, lives in
//! `crates/operations/tests/regress_boolean_seam_edge_lineage.rs`.

#![allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]

use crate::kernel::RemusKernel;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;

fn run(kernel: &mut RemusKernel, ops: &serde_json::Value) -> Vec<serde_json::Value> {
    let response = kernel.execute_batch(&ops.to_string());
    let parsed: serde_json::Value = serde_json::from_str(&response).unwrap();
    parsed
        .as_array()
        .unwrap()
        .iter()
        .map(|entry| {
            assert!(entry.get("error").is_none(), "batch error: {entry}");
            entry["ok"].clone()
        })
        .collect()
}

fn handle(value: &serde_json::Value) -> u32 {
    u32::try_from(value.as_u64().unwrap()).unwrap()
}

#[derive(Clone, Copy, Debug)]
enum Family {
    BoxBoss,
    SphereBoss,
    SphereBore,
}

/// OpenZCAD's `coordinateFrameMatrix` for a +Z axis at `origin`, row-major.
fn frame(origin: [f64; 3]) -> serde_json::Value {
    serde_json::json!([
        0.0, 1.0, 0.0, origin[0], -1.0, 0.0, 0.0, origin[1], 0.0, 0.0, 1.0, origin[2], 0.0, 0.0,
        0.0, 1.0
    ])
}

/// Build the operands; returns `(other, boss)` handles.
fn operands(kernel: &mut RemusKernel, family: Family, s: f64) -> (u32, u32) {
    let (other, radius, height, base) = match family {
        Family::BoxBoss => {
            let made = run(
                kernel,
                &serde_json::json!([{"op": "makeBox", "args": {
                    "width": 60.0 * s, "height": 60.0 * s, "depth": 20.0 * s}}]),
            );
            (
                handle(&made[0]),
                6.0 * s,
                30.0 * s,
                [30.0 * s, 30.0 * s, 10.0 * s],
            )
        }
        Family::SphereBoss | Family::SphereBore => {
            let made = run(
                kernel,
                &serde_json::json!([{"op": "makeSphere", "args": {
                    "radius": 73.5 * s, "segments": 32}}]),
            );
            let sphere = handle(&made[0]);
            run(
                kernel,
                &serde_json::json!([{"op": "transform", "args": {"solid": sphere, "matrix": [
                    1.0, 0.0, 0.0, 25.5 * s, 0.0, 1.0, 0.0, -1.5 * s,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]}}]),
            );
            (sphere, 6.0 * s, 64.1 * s, [0.0, 0.0, 58.0 * s])
        }
    };
    let made = run(
        kernel,
        &serde_json::json!([{"op": "makeCylinder", "args": {"radius": radius, "height": height}}]),
    );
    let placed = run(
        kernel,
        &serde_json::json!([{"op": "copyAndTransformSolid", "args": {
            "solid": handle(&made[0]), "matrix": frame(base)}}]),
    );
    (other, handle(&placed[0]))
}

fn check(family: Family, s: f64, boss_first: bool, batch: bool) {
    let label = format!("{family:?} scale {s} boss_first {boss_first} batch {batch}");
    let (op, plain_op) = match family {
        Family::BoxBoss | Family::SphereBoss => ("fuseWithEntityEvolution", "fuse"),
        Family::SphereBore => ("cutWithEntityEvolution", "cut"),
    };
    let mut kernel = RemusKernel::new();
    let (other, boss) = operands(&mut kernel, family, s);

    let topo = kernel.topo();
    let boss_id = kernel.resolve_solid(boss).unwrap();
    let other_id = kernel.resolve_solid(other).unwrap();
    let walls: Vec<u64> = solid_faces(topo, boss_id)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .map(|f| f.index() as u64)
        .collect();
    let seams: Vec<u64> = solid_edges(topo, boss_id)
        .unwrap()
        .into_iter()
        .filter(|&e| matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Line))
        .map(|e| e.index() as u64)
        .collect();
    let other_faces: Vec<u64> = solid_faces(topo, other_id)
        .unwrap()
        .into_iter()
        .map(|f| f.index() as u64)
        .collect();
    let ([wall], [seam]) = (walls.as_slice(), seams.as_slice()) else {
        panic!("{label}: one boss wall and seam, got {walls:?} {seams:?}");
    };

    let (a, b) = if boss_first {
        (boss, other)
    } else {
        (other, boss)
    };
    // The same operand script in a fresh kernel gives the same handles.
    let mut plain_kernel = RemusKernel::new();
    assert_eq!(
        operands(&mut plain_kernel, family, s),
        (other, boss),
        "{label}: deterministic handles"
    );
    let payload: serde_json::Value = if batch {
        run(
            &mut kernel,
            &serde_json::json!([{"op": op, "args": {"solidA": a, "solidB": b}}]),
        )[0]
        .clone()
    } else {
        let json = match family {
            Family::BoxBoss | Family::SphereBoss => kernel.fuse_with_entity_evolution(a, b),
            Family::SphereBore => kernel.cut_with_entity_evolution(a, b),
        }
        .unwrap();
        serde_json::from_str(&json).unwrap()
    };

    let edges = payload["evolution"]["edges"].as_array().unwrap();
    let mut generated = 0;
    let mut seam_pieces = 0;
    for edge in edges {
        match edge["event"].as_str().unwrap() {
            "unresolved" => panic!("{label}: unresolved edge {edge}"),
            "generated" => {
                generated += 1;
                let pair = [edge["faceA"].as_u64(), edge["faceB"].as_u64()];
                let [Some(fa), Some(fb)] = pair else {
                    panic!("{label}: generated edge must name both faces: {edge}");
                };
                let o = if fa == *wall { fb } else { fa };
                assert!(
                    (fa == *wall || fb == *wall) && other_faces.contains(&o),
                    "{label}: generated edge pairs the boss wall with the other operand: {edge}"
                );
            }
            "modified" | "preserved" if edge["from"] == *seam => seam_pieces += 1,
            _ => {}
        }
    }
    let expected = match family {
        Family::BoxBoss => 1,
        Family::SphereBoss | Family::SphereBore => 4,
    };
    assert_eq!(generated, expected, "{label}: generated ring pieces");
    assert_eq!(seam_pieces, 1, "{label}: the kept boss seam piece");

    // The evolution boolean is a separate implementation from the plain one
    // the consumer ships; the two bodies must agree.
    let result = handle(&payload["solid"]);
    let plain = handle(
        &run(
            &mut plain_kernel,
            &serde_json::json!([{"op": plain_op, "args": {"solidA": a, "solidB": b}}]),
        )[0],
    );
    let shape = |kernel: &RemusKernel, solid: u32| {
        let id = kernel.resolve_solid(solid).unwrap();
        (
            solid_faces(kernel.topo(), id).unwrap().len(),
            solid_edges(kernel.topo(), id).unwrap().len(),
        )
    };
    assert_eq!(
        shape(&kernel, result),
        shape(&plain_kernel, plain),
        "{label}: face and edge counts match the plain boolean"
    );
    let volume = |kernel: &mut RemusKernel, solid: u32| {
        run(
            kernel,
            &serde_json::json!([{"op": "volume", "args": {"solid": solid, "deflection": 0.01 * s}}]),
        )[0]
        .as_f64()
        .unwrap()
    };
    let (ev, pv) = (
        volume(&mut kernel, result),
        volume(&mut plain_kernel, plain),
    );
    assert!(
        (ev - pv).abs() <= 1e-6 * pv.abs(),
        "{label}: volume {ev} vs plain {pv}"
    );
}

#[test]
fn box_boss_fuse_names_every_edge_on_both_routes() {
    for s in [1e-3, 1.0, 1e3] {
        for boss_first in [false, true] {
            for batch in [false, true] {
                check(Family::BoxBoss, s, boss_first, batch);
            }
        }
    }
}

#[test]
fn sphere_boss_fuse_names_every_edge_on_both_routes() {
    // From 100x up the exact sphere × cylinder section does not converge for
    // either entry point (see the native file); 10x is the top of the sweep.
    for s in [1e-3, 1.0, 10.0] {
        for boss_first in [false, true] {
            for batch in [false, true] {
                check(Family::SphereBoss, s, boss_first, batch);
            }
        }
    }
}

#[test]
fn sphere_bore_cut_names_every_edge_on_both_routes() {
    for s in [1e-3, 1.0, 10.0] {
        for batch in [false, true] {
            check(Family::SphereBore, s, false, batch);
        }
    }
}
