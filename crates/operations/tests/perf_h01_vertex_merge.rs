//! PERF-H01 equivalence: the indexed vertex-merge plan produces byte-identical
//! topology to the legacy all-pairs scan.
//!
//! A test-only reference implementation mirrors the pre-change
//! `merge_coincident_vertices` pair loop exactly (predicate, tolerance policy,
//! canonical representative, order, nontransitive skips). Each fixture runs
//! the production merge on one topology clone and the reference on another;
//! both must agree on merged count, every edge endpoint, survivor positions,
//! and preserved face attributes.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_math::vec::{Point3, Vec3};
use remus_operations::heal::merge_coincident_vertices;
use remus_topology::Topology;
use remus_topology::attributes::EntityAttributes;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

const TOL: f64 = 1e-7;

fn build_soup(
    topo: &mut Topology,
    positions: &[Point3],
    vertex_tolerance: impl Fn(usize) -> f64,
) -> SolidId {
    assert!(positions.len().is_multiple_of(3));
    let mut faces = Vec::new();
    for (t, tri) in positions.chunks(3).enumerate() {
        let base = topo.num_vertices();
        let va = topo.add_vertex(Vertex::new(tri[0], vertex_tolerance(base)));
        let vb = topo.add_vertex(Vertex::new(tri[1], vertex_tolerance(base + 1)));
        let vc = topo.add_vertex(Vertex::new(tri[2], vertex_tolerance(base + 2)));
        let eab = topo.add_edge(Edge::new(va, vb, EdgeCurve::Line));
        let ebc = topo.add_edge(Edge::new(vb, vc, EdgeCurve::Line));
        let eca = topo.add_edge(Edge::new(vc, va, EdgeCurve::Line));
        let wire = Wire::new(
            vec![
                OrientedEdge::new(eab, true),
                OrientedEdge::new(ebc, true),
                OrientedEdge::new(eca, true),
            ],
            true,
        )
        .unwrap();
        let wid = topo.add_wire(wire);
        let fid = topo.add_face(Face::new(
            wid,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: -tri[0].z(),
            },
        ));
        topo.set_face_attributes(
            fid,
            EntityAttributes {
                name: Some(format!("tri-{t}")),
                color: None,
            },
        )
        .unwrap();
        faces.push(fid);
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

/// Test-only reference: the exact pre-PERF-H01 all-pairs discovery loop plus
/// the unchanged edge-update application. Returns the merged count.
fn reference_old_merge(topo: &mut Topology, solid: SolidId, tolerance: f64) -> usize {
    let tol_sq = tolerance * tolerance;
    let solid_data = topo.solid(solid).unwrap();
    let shell = topo.shell(solid_data.outer_shell()).unwrap();
    let face_ids: Vec<_> = shell.faces().to_vec();

    let mut vertex_ids: Vec<VertexId> = Vec::new();
    let mut positions: Vec<Point3> = Vec::new();
    let mut seen = std::collections::HashSet::new();
    for &fid in &face_ids {
        let face = topo.face(fid).unwrap();
        let wire = topo.wire(face.outer_wire()).unwrap();
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge()).unwrap();
            for &vid in &[edge.start(), edge.end()] {
                if seen.insert(vid.index()) {
                    let point = topo.vertex(vid).unwrap().point();
                    vertex_ids.push(vid);
                    positions.push(point);
                }
            }
        }
    }

    let num_verts = vertex_ids.len();
    let mut merge_to: HashMap<usize, VertexId> = HashMap::new();
    let mut merged_count = 0;
    for i in 0..num_verts {
        if merge_to.contains_key(&vertex_ids[i].index()) {
            continue;
        }
        for j in (i + 1)..num_verts {
            if merge_to.contains_key(&vertex_ids[j].index()) {
                continue;
            }
            let dist_sq = (positions[i] - positions[j]).length_squared();
            if dist_sq < tol_sq {
                merge_to.insert(vertex_ids[j].index(), vertex_ids[i]);
                merged_count += 1;
            }
        }
    }
    if merged_count == 0 {
        return 0;
    }

    let mut edge_ids = Vec::new();
    for &fid in &face_ids {
        let face = topo.face(fid).unwrap();
        let wire = topo.wire(face.outer_wire()).unwrap();
        for oe in wire.edges() {
            edge_ids.push(oe.edge());
        }
    }
    edge_ids.sort_by_key(|e| e.index());
    edge_ids.dedup_by_key(|e| e.index());
    for eid in edge_ids {
        let edge = topo.edge(eid).unwrap();
        let new_start = merge_to
            .get(&edge.start().index())
            .copied()
            .unwrap_or_else(|| edge.start());
        let new_end = merge_to
            .get(&edge.end().index())
            .copied()
            .unwrap_or_else(|| edge.end());
        if new_start != edge.start() || new_end != edge.end() {
            let edge = topo.edge_mut(eid).unwrap();
            edge.set_start(new_start);
            edge.set_end(new_end);
        }
    }
    merged_count
}

fn edge_endpoint_index_pairs(topo: &Topology) -> Vec<(usize, usize)> {
    let mut pairs: Vec<(usize, usize)> = topo
        .edges()
        .iter()
        .map(|(_, e)| (e.start().index(), e.end().index()))
        .collect();
    pairs.sort_unstable();
    pairs
}

fn survivor_positions(topo: &Topology) -> Vec<(usize, [f64; 3])> {
    let mut out: Vec<(usize, [f64; 3])> = topo
        .vertices()
        .iter()
        .map(|(id, v)| {
            let p = v.point();
            (id.index(), [p.x(), p.y(), p.z()])
        })
        .collect();
    out.sort_unstable_by_key(|(i, _)| *i);
    out
}

fn face_names(topo: &Topology, solid: SolidId) -> Vec<(usize, Option<String>)> {
    let solid_data = topo.solid(solid).unwrap();
    let shell = topo.shell(solid_data.outer_shell()).unwrap();
    let mut out: Vec<(usize, Option<String>)> = shell
        .faces()
        .iter()
        .map(|fid| {
            (
                fid.index(),
                topo.attributes().face(*fid).and_then(|a| a.name.clone()),
            )
        })
        .collect();
    out.sort_unstable_by_key(|(i, _)| *i);
    out
}

fn check_equivalence(name: &str, positions: Vec<Point3>, vertex_tol: impl Fn(usize) -> f64) {
    let mut topo_new = Topology::new();
    let solid_new = build_soup(&mut topo_new, &positions, &vertex_tol);
    let mut topo_old = topo_new.clone();
    let solid_old = solid_new;

    let merged_new = merge_coincident_vertices(&mut topo_new, solid_new, TOL).unwrap();
    let merged_old = reference_old_merge(&mut topo_old, solid_old, TOL);

    assert_eq!(
        merged_new, merged_old,
        "{name}: disclosed merged count must match the reference"
    );
    assert_eq!(
        edge_endpoint_index_pairs(&topo_new),
        edge_endpoint_index_pairs(&topo_old),
        "{name}: every edge endpoint must match the reference"
    );
    assert_eq!(
        survivor_positions(&topo_new),
        survivor_positions(&topo_old),
        "{name}: survivor positions must match the reference"
    );
    assert_eq!(
        face_names(&topo_new, solid_new),
        face_names(&topo_old, solid_old),
        "{name}: face attributes must survive identically"
    );
}

fn sparse_positions(nverts: usize) -> Vec<Point3> {
    assert!(nverts.is_multiple_of(3));
    let mut out = Vec::with_capacity(nverts);
    for t in 0..(nverts / 3) {
        let bx = (t as f64) * 10.0;
        out.push(Point3::new(bx, 0.0, 0.0));
        out.push(Point3::new(bx + 1.0, 0.0, 0.0));
        out.push(Point3::new(bx, 1.0, 0.0));
    }
    out
}

fn clustered_positions(nverts: usize) -> Vec<Point3> {
    assert!(nverts.is_multiple_of(3));
    let mut out = Vec::with_capacity(nverts);
    for t in 0..(nverts / 3) {
        let bx = (t as f64) * 10.0;
        out.push(Point3::new(bx, 0.0, 0.0));
        out.push(Point3::new(bx + 1.0, 0.0, 0.0));
        if t > 0 && t % 3 == 0 {
            out.push(Point3::new(((t - 1) as f64) * 10.0 + 5e-8, 0.0, 0.0));
        } else {
            out.push(Point3::new(bx, 1.0, 0.0));
        }
    }
    out
}

fn coincident_positions(nverts: usize) -> Vec<Point3> {
    let mut out = Vec::with_capacity(nverts);
    let mut s = 0x1234_5678u64;
    let mut rnd = move || {
        s = s
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        f64::from(((s >> 33) & 0x7fff_ffff) as u32) / f64::from(u32::MAX)
    };
    // Triangle count must stay integral; 150 is a multiple of three.
    for _ in 0..nverts {
        out.push(Point3::new(rnd() * 1e-8, rnd() * 1e-8, rnd() * 1e-8));
    }
    out
}

#[test]
fn sparse_soup_matches_reference_with_zero_merges() {
    check_equivalence("sparse-300", sparse_positions(300), |_| 1e-7);
}

#[test]
fn clustered_soup_matches_reference_with_real_merges() {
    check_equivalence("clustered-300", clustered_positions(300), |_| 1e-7);
}

#[test]
fn coincident_soup_matches_reference_in_dense_worst_case() {
    check_equivalence("coincident-150", coincident_positions(150), |_| 1e-7);
}

#[test]
fn heterogeneous_stored_vertex_tolerances_do_not_change_merges() {
    // Stored per-vertex tolerances vary by 6 orders of magnitude; the run
    // tolerance alone governs, exactly as before the change.
    check_equivalence("clustered-300-mixed-tol", clustered_positions(300), |i| {
        if i % 2 == 0 { 1e-9 } else { 1e-3 }
    });
}
