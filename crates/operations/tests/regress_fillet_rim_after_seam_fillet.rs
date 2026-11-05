//! Regression: filleting a boss's top rim after its seam with the block was
//! filleted.
//!
//! An OpenZCAD part: a 30 × 18 × 10.5 block with an r6 × 28 cylinder fused
//! through its corner, the concave seam where the cylinder wall meets the
//! block filleted r2 (its corners closed by NURBS patches), then the
//! cylinder's top rim — four quarter circles well above that seam band —
//! filleted. Every radius, one arc or all four, was refused with "trimming
//! failure" on the cylinder wall. Straight after the union the same rim
//! rounded fine.
//!
//! The rim goes to the rolling-ball engine (the walking trimmer cannot cut a
//! closed contact across a seamed wall, before or after the seam fillet, and
//! its refusal is what the caller saw). That engine rebuilt the wall
//! correctly — up its seam, round the new contact circle, back down the seam —
//! but the seam band had left NURBS curves in the wall's lower boundary, so
//! the wall was assembled through the generic surface path instead of the
//! cylinder-specific one. The generic path dropped any edge already in the
//! wire, including the seam's return use: the seam became a free edge, the
//! loop opened and the postcondition refused the body.
//!
//! The fixture is the exact body the app filleted, serialized from the
//! session. Rebuilding it natively takes the app's whole feature history
//! (later face offsets reshape the seam corners); a plain block-and-boss seam
//! fillet closes its corners analytically and never reaches the generic path.
//! The assembler rule itself is pinned natively by
//! `generic_surface_wall_keeps_its_seam_return_use` in `boolean/tests.rs`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_math::vec::Point3;
use remus_operations::blend_ops;
use remus_operations::measure;
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const BOSS_RADIUS: f64 = 6.0;
const BOSS_HEIGHT: f64 = 28.0;

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

fn endpoints(topo: &Topology, edge: EdgeId) -> [Point3; 2] {
    let e = topo.edge(edge).unwrap();
    [
        topo.vertex(e.start()).unwrap().point(),
        topo.vertex(e.end()).unwrap().point(),
    ]
}

/// The boss's top rim: the circular arcs at the top of the boss.
fn top_rim(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    solid_edges(topo, s)
        .into_iter()
        .filter(|&e| {
            let [a, b] = endpoints(topo, e);
            matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Circle(_))
                && (a.z() - BOSS_HEIGHT).abs() < 1e-6
                && (b.z() - BOSS_HEIGHT).abs() < 1e-6
        })
        .collect()
}

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

/// Faces on the rolling ball's torus round the top rim: minor radius `r`,
/// major radius `R − r`, centred on the boss axis one radius below the top.
fn rim_tori(topo: &Topology, s: SolidId, r: f64) -> usize {
    solid_faces(topo, s)
        .unwrap()
        .into_iter()
        .filter(|&fid| match topo.face(fid).unwrap().surface() {
            FaceSurface::Torus(t) => {
                (t.minor_radius() - r).abs() < 1e-9
                    && (t.major_radius() - (BOSS_RADIUS - r)).abs() < 1e-9
                    && (t.center().z() - (BOSS_HEIGHT - r)).abs() < 1e-9
                    && t.center().x().abs() < 1e-9
                    && t.center().y().abs() < 1e-9
            }
            _ => false,
        })
        .count()
}

/// Material a rim fillet of radius `r` removes, by Pappus: the corner square
/// less its quarter disc, revolved at the spandrel centroid's radius.
fn rim_removal(r: f64) -> f64 {
    let pi = std::f64::consts::PI;
    let area = r * r * (1.0 - pi / 4.0);
    let centroid = BOSS_RADIUS - r * (10.0 - 3.0 * pi) / (3.0 * (4.0 - pi));
    std::f64::consts::TAU * centroid * area
}

/// The exact body OpenZCAD filleted: the same block and boss with the seam
/// filleted from the app's five-edge selection, whose mitres are NURBS.
fn openzcad_seam_filleted_boss(topo: &mut Topology) -> SolidId {
    remus_io::arena_io::deserialize_solids(
        include_bytes!("data/rim_after_seam_fillet.arena.json"),
        topo,
    )
    .unwrap()[0]
}

/// The boss wall the rim runs round still carries a curved, non-circular
/// edge from the seam band. That is what sends its rebuild through the
/// generic surface path, so a body without one would not exercise the fix.
fn wall_keeps_a_seam_band_curve(topo: &Topology, s: SolidId) -> bool {
    let rim = top_rim(topo, s);
    solid_faces(topo, s).unwrap().into_iter().any(|fid| {
        let f = topo.face(fid).unwrap();
        let edges = topo.wire(f.outer_wire()).unwrap().edges();
        matches!(f.surface(), FaceSurface::Cylinder(_))
            && edges.iter().any(|oe| rim.contains(&oe.edge()))
            && edges.iter().any(|oe| {
                matches!(
                    topo.edge(oe.edge()).unwrap().curve(),
                    EdgeCurve::NurbsCurve(_)
                )
            })
    })
}

fn assert_rim_fillet(topo: &Topology, body: SolidId, label: &str) {
    let rim = top_rim(topo, body);
    assert_eq!(
        rim.len(),
        4,
        "{label}: the union leaves the rim as four arcs"
    );
    assert!(
        wall_keeps_a_seam_band_curve(topo, body),
        "{label}: the boss wall must still carry the seam band's curve"
    );
    let before = measure::solid_volume(topo, body, 0.01).unwrap();
    let census_before = surface_census(topo, body);

    // Every arc at the reported radius, one arc on its own (the G1 chain
    // still rounds the whole rim), and a smaller radius.
    for (r, arcs) in [(1.0, 4), (1.0, 1), (0.5, 4)] {
        let selection = &rim[..arcs];
        let case = format!("{label} r={r} arcs={}", selection.len());
        let mut t = topo.clone();
        let result = blend_ops::fillet_cascade(&mut t, body, selection, r)
            .unwrap_or_else(|e| panic!("{case}: rim fillet refused: {e:?}"));
        assert!(result.failed.is_empty(), "{case}: {:?}", result.failed);
        let s = result.solid;

        // The band is the rolling ball's exact torus, not an approximation,
        // and no other freeform face appears.
        assert!(
            rim_tori(&t, s, r) >= 1,
            "{case}: rim band must be the exact torus"
        );
        let census = surface_census(&t, s);
        assert_eq!(
            census.get("nurbs").copied().unwrap_or(0),
            census_before.get("nurbs").copied().unwrap_or(0),
            "{case}: the rim adds no NURBS faces: {census:?}"
        );

        let report = validate_solid(&t, s).unwrap();
        assert!(report.is_valid(), "{case}: {:?}", report.issues);
        assert_eq!(
            brep_edge_health(&t, s),
            (0, 0),
            "{case}: B-Rep must be closed and manifold"
        );
        assert_eq!(
            mesh_edge_health(&t, s),
            (0, 0),
            "{case}: tessellation must be watertight"
        );

        // Whatever the seam band integrates to, it is the same before and
        // after; the difference is the rim alone, exact by Pappus.
        let removed = before - measure::solid_volume(&t, s, 0.01).unwrap();
        let want = rim_removal(r);
        assert!(
            (removed - want).abs() / want < 1e-6,
            "{case}: removed {removed} vs Pappus {want}"
        );
    }
}

#[test]
fn rim_fillet_after_openzcad_seam_fillet_is_exact_and_closed() {
    let mut topo = Topology::new();
    let body = openzcad_seam_filleted_boss(&mut topo);
    assert_rim_fillet(&topo, body, "openzcad");
}
