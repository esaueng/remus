//! Regression: chamfering a CLOSED circular edge (a cylinder rim).
//!
//! `chamfer_v2` refused every closed edge outright — `reject_closed_edges`
//! returned "closed-edge chamfer assembly is not yet supported" — because the
//! per-face, line-based trimmer cannot cut a face along a closed interior
//! contact loop: there are no endpoints to cut at. The v1 flat-bevel engine is
//! planar-only and fails such an edge with "cannot normalize zero vector".
//! Between them, no engine could chamfer a cylinder rim at all.
//!
//! `fillet_builder` already solved exactly this with an annular rebuild
//! (`closed_rim_info` / `assemble_closed_rim`): rebuild the disc cap bounded by
//! the plate-contact circle, shorten the wall to the wall-contact circle, and
//! emit the band between them sharing both edges. The chamfer band is the same
//! construction with a cone instead of a torus and a straight ruled seam
//! instead of a minor arc.
//!
//! This stayed invisible for a long time because the OpenZCAD flange demo
//! chamfers its rim and had been passing: the mesh-boolean fallback handed it a
//! body whose "rim" was a polyline of straight segments, which the planar v1
//! engine handles. Fixing the booleans made the blank analytic, the rim became
//! a real circle, and the chamfer started failing.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use brepkit_check::classify::{ClassifyOptions, PointClassification, classify_point};
use brepkit_math::vec::Point3;
use brepkit_operations::blend_ops;
use brepkit_operations::measure;
use brepkit_operations::primitives;
use brepkit_operations::tessellate::tessellate_solid_with_tolerance;
use brepkit_topology::Topology;
use brepkit_topology::edge::EdgeId;
use brepkit_topology::explorer::solid_faces;
use brepkit_topology::solid::SolidId;

const R: f64 = 45.0;
const H: f64 = 10.0;

fn surface_census(topo: &Topology, s: SolidId) -> HashMap<&'static str, usize> {
    let mut m = HashMap::new();
    for fid in solid_faces(topo, s).unwrap() {
        *m.entry(topo.face(fid).unwrap().surface().type_tag())
            .or_insert(0) += 1;
    }
    m
}

fn brep_edge_health(topo: &Topology, s: SolidId) -> (usize, usize) {
    let mut usage: HashMap<usize, usize> = HashMap::new();
    for fid in solid_faces(topo, s).unwrap() {
        let f = topo.face(fid).unwrap();
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                *usage.entry(oe.edge().index()).or_insert(0) += 1;
            }
        }
    }
    (
        usage.values().filter(|&&c| c == 1).count(),
        usage.values().filter(|&&c| c >= 3).count(),
    )
}

fn mesh_edge_health(topo: &Topology, s: SolidId) -> (usize, usize) {
    let mesh = tessellate_solid_with_tolerance(topo, s, 0.01, 0.1).unwrap();
    let q = 1e6;
    let mut canon: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut remap = vec![0u32; mesh.positions.len()];
    for (i, p) in mesh.positions.iter().enumerate() {
        let key = (
            (p.x() * q).round() as i64,
            (p.y() * q).round() as i64,
            (p.z() * q).round() as i64,
        );
        let next = canon.len() as u32;
        remap[i] = *canon.entry(key).or_insert(next);
    }
    let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let v = [
            remap[tri[0] as usize],
            remap[tri[1] as usize],
            remap[tri[2] as usize],
        ];
        for &(a, b) in &[(v[0], v[1]), (v[1], v[2]), (v[2], v[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    (
        edges.values().filter(|&&c| c == 1).count(),
        edges.values().filter(|&&c| c >= 3).count(),
    )
}

/// Distinct edges of a solid, in discovery order: `[bottom rim, seam, top rim]`
/// for a cylinder primitive.
fn solid_edges(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    let mut seen = Vec::new();
    for fid in solid_faces(topo, s).unwrap() {
        let f = topo.face(fid).unwrap();
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                if !seen.contains(&oe.edge()) {
                    seen.push(oe.edge());
                }
            }
        }
    }
    seen
}

fn closed_rims(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    solid_edges(topo, s)
        .into_iter()
        .filter(|&e| {
            let ed = topo.edge(e).unwrap();
            ed.start() == ed.end() && ed.curve().type_tag() == "circle"
        })
        .collect()
}

/// Material removed by a symmetric chamfer of setback `d` on a rim of radius
/// `R`, by Pappus: the right triangle (legs `d`, `d`, area `d²/2`) revolved
/// about the axis at centroid radius `R − d/3`.
fn expected_volume(d: f64) -> f64 {
    let full = std::f64::consts::PI * R * R * H;
    full - 0.5 * d * d * std::f64::consts::TAU * (R - d / 3.0)
}

#[test]
fn closed_rim_chamfer_is_exact_and_watertight() {
    for d in [0.5_f64, 1.5, 3.0] {
        for rim_index in 0..2 {
            let mut topo = Topology::new();
            let cyl = primitives::make_cylinder(&mut topo, R, H).unwrap();
            let rims = closed_rims(&topo, cyl);
            assert_eq!(rims.len(), 2, "a cylinder has two closed rim circles");

            let r = blend_ops::chamfer_v2(&mut topo, cyl, &[rims[rim_index]], d, d)
                .unwrap_or_else(|e| panic!("d={d} rim={rim_index}: chamfer failed: {e:?}"));
            assert!(r.failed.is_empty(), "d={d} rim={rim_index}: {:?}", r.failed);

            // The band must be an exact analytic cone, not a NURBS approximation.
            let census = surface_census(&topo, r.solid);
            assert_eq!(
                census.get("cone").copied().unwrap_or(0),
                1,
                "d={d} rim={rim_index}: chamfer band must be an analytic cone: {census:?}"
            );
            assert_eq!(
                census.get("cylinder").copied().unwrap_or(0),
                1,
                "d={d} rim={rim_index}: the wall stays a cylinder: {census:?}"
            );
            assert_eq!(
                census.values().sum::<usize>(),
                4,
                "d={d} rim={rim_index}: wall + two caps + band: {census:?}"
            );

            assert_eq!(
                brep_edge_health(&topo, r.solid),
                (0, 0),
                "d={d} rim={rim_index}: B-Rep must be closed and manifold"
            );
            // A closed B-Rep can still mesh open — check the mesh separately.
            assert_eq!(
                mesh_edge_health(&topo, r.solid),
                (0, 0),
                "d={d} rim={rim_index}: tessellation must be watertight"
            );

            // Volume against the Pappus closed form. The band is exact, so this
            // is tight rather than a loose tolerance.
            let vol = measure::solid_volume(&topo, r.solid, 0.02).unwrap();
            let want = expected_volume(d);
            assert!(
                (vol - want).abs() / want < 1e-6,
                "d={d} rim={rim_index}: volume {vol} vs Pappus {want}"
            );
        }
    }
}

#[test]
fn closed_rim_chamfer_removes_material_on_the_right_side() {
    let d = 3.0;
    let mut topo = Topology::new();
    let cyl = primitives::make_cylinder(&mut topo, R, H).unwrap();
    // Chamfer the TOP rim (z = H).
    let rims = closed_rims(&topo, cyl);
    let top = rims
        .into_iter()
        .find(|&e| {
            let p = topo.vertex(topo.edge(e).unwrap().start()).unwrap().point();
            (p.z() - H).abs() < 1e-9
        })
        .expect("top rim");
    let r = blend_ops::chamfer_v2(&mut topo, cyl, &[top], d, d).expect("top rim chamfer");

    let opts = ClassifyOptions::default();
    let probe = |x: f64, z: f64| classify_point(&topo, r.solid, Point3::new(x, 0.0, z), &opts);

    // Just outside the chamfered corner — removed.
    assert_eq!(
        probe(R - 0.4, H - 0.4).unwrap(),
        PointClassification::Outside,
        "the top outer corner must be cut away"
    );
    // The mirrored point at the UNCHAMFERED bottom rim — still material.
    assert_eq!(
        probe(R - 0.4, 0.4).unwrap(),
        PointClassification::Inside,
        "the bottom rim was not chamfered and must be untouched"
    );
    // Deep interior — material.
    assert_eq!(
        probe(0.0, H / 2.0).unwrap(),
        PointClassification::Inside,
        "the core is solid"
    );
    // Well inside the top face but away from the rim — material.
    assert_eq!(
        probe(R - 10.0, H - 0.4).unwrap(),
        PointClassification::Inside,
        "only the rim corner is removed, not the whole top"
    );
}

/// Chamfering BOTH rims in one call must also work, and must be symmetric.
#[test]
fn both_rims_chamfered_in_one_call() {
    let d = 2.0;
    let mut topo = Topology::new();
    let cyl = primitives::make_cylinder(&mut topo, R, H).unwrap();
    let rims = closed_rims(&topo, cyl);

    let r = blend_ops::chamfer_v2(&mut topo, cyl, &rims, d, d).expect("both rims");
    assert!(r.failed.is_empty(), "{:?}", r.failed);

    let census = surface_census(&topo, r.solid);
    assert_eq!(
        census.get("cone").copied().unwrap_or(0),
        2,
        "one cone band per rim: {census:?}"
    );
    assert_eq!(
        brep_edge_health(&topo, r.solid),
        (0, 0),
        "double-chamfered solid must be closed"
    );
    assert_eq!(
        mesh_edge_health(&topo, r.solid),
        (0, 0),
        "double-chamfered solid must mesh watertight"
    );

    // Both chamfers removed, so subtract the Pappus wedge twice.
    let full = std::f64::consts::PI * R * R * H;
    let one = full - expected_volume(d);
    let want = full - 2.0 * one;
    let vol = measure::solid_volume(&topo, r.solid, 0.02).unwrap();
    assert!(
        (vol - want).abs() / want < 1e-6,
        "volume {vol} vs {want} (two chamfers)"
    );
}
