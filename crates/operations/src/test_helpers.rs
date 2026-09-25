//! Shared test assertion helpers for geometric and topological validation.
//!
//! These helpers provide descriptive, tolerance-aware assertions for
//! volumes, areas, positions, and topological invariants. They are
//! designed to be used across all test modules in `remus-operations`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, dead_code)]

use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::explorer;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

/// Assert that a solid's volume is within `rel_tol` of `expected`.
///
/// When `expected` is near zero (< 1e-15), `rel_tol` is treated as an
/// absolute tolerance instead (i.e., asserts `|volume| < rel_tol`).
///
/// # Panics
///
/// Panics if the relative error exceeds `rel_tol` or if volume
/// computation fails.
pub fn assert_volume_near(topo: &Topology, solid: SolidId, expected: f64, rel_tol: f64) {
    let vol = crate::measure::solid_volume(topo, solid, 0.05).unwrap();
    let rel_error = if expected.abs() < 1e-15 {
        vol.abs()
    } else {
        (vol - expected).abs() / expected.abs()
    };
    assert!(
        rel_error < rel_tol,
        "volume mismatch: got {vol:.6}, expected {expected:.6} \
         (error: {:.4}%, tolerance: {:.4}%)",
        rel_error * 100.0,
        rel_tol * 100.0,
    );
}

/// Assert that a face's area is within `rel_tol` of `expected`.
///
/// # Panics
///
/// Panics if the relative error exceeds `rel_tol` or if area
/// computation fails.
pub fn assert_area_near(topo: &Topology, face: FaceId, expected: f64, rel_tol: f64) {
    let area = crate::measure::face_area(topo, face, 0.1).unwrap();
    let rel_error = if expected.abs() < 1e-15 {
        area.abs()
    } else {
        (area - expected).abs() / expected.abs()
    };
    assert!(
        rel_error < rel_tol,
        "area mismatch: got {area:.6}, expected {expected:.6} \
         (error: {:.4}%, tolerance: {:.4}%)",
        rel_error * 100.0,
        rel_tol * 100.0,
    );
}

/// Assert that two points are within `abs_tol` of each other.
///
/// # Panics
///
/// Panics if the Euclidean distance exceeds `abs_tol`.
pub fn assert_point_near(actual: Point3, expected: Point3, abs_tol: f64) {
    let dx = actual.x() - expected.x();
    let dy = actual.y() - expected.y();
    let dz = actual.z() - expected.z();
    let dist = (dx * dx + dy * dy + dz * dz).sqrt();
    assert!(
        dist < abs_tol,
        "point mismatch: got ({:.6}, {:.6}, {:.6}), \
         expected ({:.6}, {:.6}, {:.6}), distance={dist:.2e}",
        actual.x(),
        actual.y(),
        actual.z(),
        expected.x(),
        expected.y(),
        expected.z(),
    );
}

/// Compute the Euler characteristic V - E + F for a solid.
///
/// For a closed orientable surface of genus g:
/// - genus 0 (sphere-like): χ = 2
/// - genus 1 (torus-like): χ = 0
/// - genus 2 (double torus): χ = -2
///
/// # Panics
///
/// Panics if topology lookups fail.
#[allow(clippy::cast_possible_wrap)]
pub fn euler_characteristic(topo: &Topology, solid: SolidId) -> i64 {
    let (f, e, v) = explorer::solid_entity_counts(topo, solid).unwrap();
    (v as i64) - (e as i64) + (f as i64)
}

/// Assert that a solid has Euler characteristic 2 (genus-0, sphere-like).
///
/// This is the expected value for any simply-connected closed solid
/// (box, cylinder, cone, sphere, any convex solid, boolean result
/// of convex inputs without holes).
///
/// # Panics
///
/// Panics if the Euler characteristic is not 2.
pub fn assert_euler_genus0(topo: &Topology, solid: SolidId) {
    let chi = euler_characteristic(topo, solid);
    assert_eq!(
        chi, 2,
        "expected Euler characteristic V-E+F = 2 (genus-0), got {chi}"
    );
}

/// Assert that a solid's shell is manifold (every edge shared by exactly 2 faces).
///
/// # Panics
///
/// Panics if the solid is not manifold or if topology lookups fail.
pub fn assert_manifold(topo: &Topology, solid: SolidId) {
    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();
    remus_topology::validation::validate_shell_manifold(sh, topo)
        .expect("solid should be manifold");
}

/// Assert the inclusion-exclusion principle: V(A) + V(B) = V(A∪B) + V(A∩B).
///
/// This is a fundamental conservation law for boolean operations.
///
/// # Panics
///
/// Panics if the identity is violated beyond `rel_tol`.
pub fn assert_volume_conservation(
    vol_a: f64,
    vol_b: f64,
    vol_fused: f64,
    vol_intersected: f64,
    rel_tol: f64,
) {
    let lhs = vol_a + vol_b;
    let rhs = vol_fused + vol_intersected;
    let rel_error = if lhs.abs() < 1e-15 {
        rhs.abs()
    } else {
        (lhs - rhs).abs() / lhs.abs()
    };
    assert!(
        rel_error < rel_tol,
        "volume conservation violated: V(A)+V(B) = {lhs:.6}, \
         V(A∪B)+V(A∩B) = {rhs:.6} (error: {:.4}%, tolerance: {:.4}%)\n\
         V(A)={vol_a:.6}, V(B)={vol_b:.6}, V(A∪B)={vol_fused:.6}, V(A∩B)={vol_intersected:.6}",
        rel_error * 100.0,
        rel_tol * 100.0,
    );
}

/// Assert that a CW-wound profile produces a solid with the expected volume.
///
/// Creates a CW unit square face, passes it to `build_solid`, and asserts
/// the resulting volume is within `rel_tol` of `expected_vol`.
///
/// # Panics
///
/// Panics if the volume deviates beyond `rel_tol` or if any operation fails.
pub fn assert_cw_profile_produces_valid_solid<F>(build_solid: F, expected_vol: f64, rel_tol: f64)
where
    F: Fn(&mut Topology, FaceId) -> SolidId,
{
    let mut topo = Topology::new();
    let face = remus_topology::test_utils::make_cw_unit_square_face(&mut topo);
    let solid = build_solid(&mut topo, face);
    assert_volume_near(&topo, solid, expected_vol, rel_tol);
}

/// Build a non-planar test profile: a 4-corner Coons patch with z-staggered
/// corners, so both its boundary (the four corners) and its surface are
/// non-planar. Centered at the origin with half-extent `half`.
pub fn make_saddle_profile(topo: &mut Topology, half: f64) -> FaceId {
    let h = half;
    let bottom = vec![
        Point3::new(-h, -h, 0.3),
        Point3::new(0.0, -h, 0.6),
        Point3::new(h, -h, -0.3),
    ];
    let right = vec![
        Point3::new(h, -h, -0.3),
        Point3::new(h, 0.0, 0.6),
        Point3::new(h, h, 0.3),
    ];
    let top = vec![
        Point3::new(-h, h, -0.3),
        Point3::new(0.0, h, 0.6),
        Point3::new(h, h, 0.3),
    ];
    let left = vec![
        Point3::new(-h, -h, 0.3),
        Point3::new(-h, 0.0, 0.6),
        Point3::new(-h, h, -0.3),
    ];
    crate::fill_face::fill_coons_patch(topo, &[bottom, right, top, left]).unwrap()
}

// ---- Split-band blend fixtures (B19 survivor tranche) ----

use remus_math::curves::Circle3D;
use remus_math::tolerance::Tolerance;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::Solid;
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

/// A 10 mm box with one r = 1 edge blend.
pub fn blended_box(edges: &[usize]) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let sharp = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let all = solid_edges(&topo, sharp).unwrap();
    let chosen: Vec<_> = edges.iter().map(|index| all[*index]).collect();
    let solid = crate::blend_ops::fillet_v2(&mut topo, sharp, &chosen, 1.0)
        .unwrap()
        .solid;
    (topo, solid)
}

pub fn cylinder_faces(topo: &Topology, solid: SolidId) -> Vec<FaceId> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|face| {
            matches!(
                topo.face(*face).unwrap().surface(),
                FaceSurface::Cylinder(_)
            )
        })
        .collect()
}

fn split_uses(uses: &[OrientedEdge], edge: EdgeId, halves: [EdgeId; 2]) -> Vec<OrientedEdge> {
    let mut rebuilt = Vec::with_capacity(uses.len() + 1);
    for use_ in uses {
        if use_.edge() != edge {
            rebuilt.push(*use_);
        } else if use_.is_forward() {
            rebuilt.push(OrientedEdge::new(halves[0], true));
            rebuilt.push(OrientedEdge::new(halves[1], true));
        } else {
            rebuilt.push(OrientedEdge::new(halves[1], false));
            rebuilt.push(OrientedEdge::new(halves[0], false));
        }
    }
    rebuilt
}

/// Split the single cylindrical band of `solid` across its axis at mid
/// length into two faces on the same exact carrier: both springs gain a
/// midpoint vertex and the halves share one exact circular arc.
pub fn split_band(topo: &mut Topology, solid: SolidId) -> (SolidId, [FaceId; 2]) {
    let [band] = cylinder_faces(topo, solid)[..] else {
        panic!("one band");
    };
    let FaceSurface::Cylinder(cylinder) = topo.face(band).unwrap().surface().clone() else {
        unreachable!("band");
    };
    let reversed = topo.face(band).unwrap().is_reversed();
    let uses = topo
        .wire(topo.face(band).unwrap().outer_wire())
        .unwrap()
        .edges()
        .to_vec();
    assert_eq!(uses.len(), 4);
    let springs: Vec<_> = uses
        .iter()
        .map(OrientedEdge::edge)
        .filter(|edge| matches!(topo.edge(*edge).unwrap().curve(), EdgeCurve::Line))
        .collect();
    assert_eq!(springs.len(), 2);
    let mut middles = Vec::new();
    let mut halves = Vec::new();
    for &spring in &springs {
        let data = topo.edge(spring).unwrap();
        let (start, end) = (data.start(), data.end());
        let a = topo.vertex(start).unwrap().point();
        let b = topo.vertex(end).unwrap().point();
        let middle = topo.add_vertex(Vertex::new(a + (b - a) * 0.5, Tolerance::new().linear));
        let first = topo.add_edge(Edge::new(start, middle, EdgeCurve::Line));
        let second = topo.add_edge(Edge::new(middle, end, EdgeCurve::Line));
        middles.push(middle);
        halves.push([first, second]);
    }
    // Exact cross-section arc between the two spring midpoints.
    let axis = cylinder.axis().normalize().unwrap();
    let point = |vertex: VertexId| topo.vertex(vertex).unwrap().point();
    let m0 = point(middles[0]);
    let m1 = point(middles[1]);
    let center = cylinder.origin() + axis * (m0 - cylinder.origin()).dot(axis);
    let mut normal = axis;
    let mut circle =
        Circle3D::new_with_ref(center, normal, cylinder.radius(), m0 - center).unwrap();
    let mut sweep = circle.project(m1).rem_euclid(std::f64::consts::TAU);
    if sweep > std::f64::consts::PI {
        normal = normal * -1.0;
        circle = Circle3D::new_with_ref(center, normal, cylinder.radius(), m0 - center).unwrap();
        sweep = circle.project(m1).rem_euclid(std::f64::consts::TAU);
    }
    assert!((circle.evaluate(sweep) - m1).length() < 1e-12);
    let mut arc = Edge::new(middles[0], middles[1], EdgeCurve::Circle(circle));
    arc.set_trim(Some((0.0, sweep)));
    let arc = topo.add_edge(arc);

    // Supports: replace each spring by its halves.
    for face in solid_faces(topo, solid).unwrap() {
        if face == band {
            continue;
        }
        let outer = topo.face(face).unwrap().outer_wire();
        let mut sequence = topo.wire(outer).unwrap().edges().to_vec();
        let before = sequence.len();
        for (spring, pair) in springs.iter().zip(&halves) {
            sequence = split_uses(&sequence, *spring, *pair);
        }
        if sequence.len() != before {
            let wire = topo.add_wire(Wire::new(sequence, true).unwrap());
            let inner = topo.face(face).unwrap().inner_wires().to_vec();
            topo.set_face_boundary_wires(face, wire, inner).unwrap();
        }
    }
    // Band: six uses, rotated to start at the half that ends at middle 0.
    let mut sequence = uses;
    for (spring, pair) in springs.iter().zip(&halves) {
        sequence = split_uses(&sequence, *spring, *pair);
    }
    let ends_at = |use_: &OrientedEdge, vertex: VertexId| {
        use_.oriented_end(topo.edge(use_.edge()).unwrap()) == vertex
    };
    let start = sequence
        .iter()
        .position(|use_| ends_at(use_, middles[0]))
        .unwrap();
    sequence.rotate_left(start);
    // [.. -> m0, m0 -> .., cross, .. -> m1, m1 -> .., cross]
    assert!(ends_at(&sequence[3], middles[1]));
    let first_half = vec![
        sequence[1],
        sequence[2],
        sequence[3],
        OrientedEdge::new(arc, false),
    ];
    let second_half = vec![
        sequence[4],
        sequence[5],
        sequence[0],
        OrientedEdge::new(arc, true),
    ];
    let mut faces = Vec::new();
    for half in [first_half, second_half] {
        let wire = topo.add_wire(Wire::new(half, true).unwrap());
        let surface = FaceSurface::Cylinder(cylinder.clone());
        faces.push(topo.add_face(if reversed {
            Face::new_reversed(wire, Vec::new(), surface)
        } else {
            Face::new(wire, Vec::new(), surface)
        }));
    }
    let mut shell_faces: Vec<_> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|face| *face != band)
        .collect();
    shell_faces.extend(&faces);
    let shell = topo.add_shell(Shell::new(shell_faces).unwrap());
    let split = topo.add_solid(Solid::new(shell, Vec::new()));
    let report = crate::validate::validate_solid(topo, split).unwrap();
    assert!(report.is_valid(), "split band fixture: {:?}", report.issues);
    (split, [faces[0], faces[1]])
}
