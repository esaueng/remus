//! PERF-D01: content-keyed per-face mesh reuse.
//!
//! Every test compares a tessellation served (partly) from this thread's face
//! mesh cache with a fresh tessellation of the same body with the cache
//! disabled. Exact hits must reproduce the fresh mesh bit for bit; translated
//! hits must reproduce its indices, face offsets and normals exactly and its
//! positions within roundoff of the body scale.

#![allow(clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

use super::*;

const DEFLECTION: f64 = 0.05;
const ANGULAR: f64 = 0.3;

/// Isolates each test's view of the thread-local cache (tests may share a
/// thread under `cargo test`).
struct CacheGuard;

impl CacheGuard {
    fn enabled() -> Self {
        disable_face_mesh_cache();
        enable_face_mesh_cache();
        Self
    }
}

impl Drop for CacheGuard {
    fn drop(&mut self) {
        disable_face_mesh_cache();
    }
}

fn stats() -> FaceMeshCacheStats {
    face_mesh_cache_stats().expect("cache enabled")
}

/// Counter deltas between two snapshots.
#[derive(Debug, PartialEq, Eq)]
struct Delta {
    exact: u64,
    translated: u64,
    misses: u64,
    refused: u64,
    conflicts: u64,
    uncacheable: u64,
}

fn delta(before: FaceMeshCacheStats, after: FaceMeshCacheStats) -> Delta {
    Delta {
        exact: after.exact_hits - before.exact_hits,
        translated: after.translated_hits - before.translated_hits,
        misses: after.misses - before.misses,
        refused: after.translation_refused - before.translation_refused,
        conflicts: after.replay_conflicts - before.replay_conflicts,
        uncacheable: after.uncacheable - before.uncacheable,
    }
}

fn grouped(topo: &Topology, solid: SolidId, deflection: f64) -> (TriangleMesh, Vec<u32>) {
    tessellate_solid_grouped_with_tolerance(topo, solid, deflection, ANGULAR).expect("tessellate")
}

/// Fresh reference: the same call with this thread's cache parked (off),
/// then the cache restored with its entries and counters intact.
fn fresh(topo: &Topology, solid: SolidId, deflection: f64) -> (TriangleMesh, Vec<u32>) {
    let parked = super::super::face_cache::park();
    let mesh = grouped(topo, solid, deflection);
    super::super::face_cache::unpark(parked);
    mesh
}

/// Body scale for the roundoff bound on translated positions.
fn scale(mesh: &TriangleMesh) -> f64 {
    mesh.positions
        .iter()
        .flat_map(|p| [p.x().abs(), p.y().abs(), p.z().abs()])
        .fold(1.0, f64::max)
}

/// Assert `reused` reproduces `reference`: identical triangle indices, face
/// offsets, vertex count and normals; positions within `ulps` units of
/// roundoff of the body scale (0 demands bit identity).
fn assert_reproduces(
    reused: &(TriangleMesh, Vec<u32>),
    reference: &(TriangleMesh, Vec<u32>),
    ulps: f64,
    context: &str,
) {
    assert_eq!(reused.1, reference.1, "{context}: face offsets differ");
    assert_eq!(
        reused.0.indices, reference.0.indices,
        "{context}: triangle indices differ"
    );
    assert_eq!(
        reused.0.positions.len(),
        reference.0.positions.len(),
        "{context}: vertex counts differ"
    );
    let tol = ulps * f64::EPSILON * scale(&reference.0);
    for (i, (a, b)) in reused
        .0
        .positions
        .iter()
        .zip(&reference.0.positions)
        .enumerate()
    {
        let d = [a.x() - b.x(), a.y() - b.y(), a.z() - b.z()]
            .into_iter()
            .fold(0.0_f64, |m, c| m.max(c.abs()));
        assert!(
            d <= tol,
            "{context}: vertex {i} moved by {d:e} (bound {tol:e})"
        );
    }
    for (i, (a, b)) in reused
        .0
        .normals
        .iter()
        .zip(&reference.0.normals)
        .enumerate()
    {
        assert!(
            a.x().to_bits() == b.x().to_bits()
                && a.y().to_bits() == b.y().to_bits()
                && a.z().to_bits() == b.z().to_bits(),
            "{context}: normal {i} differs: {a:?} vs {b:?}"
        );
    }
}

fn assert_watertight(mesh: &TriangleMesh, context: &str) {
    assert_eq!(boundary_edge_count(mesh), 0, "{context}: open mesh edges");
    assert_eq!(
        non_manifold_edge_count(mesh),
        0,
        "{context}: non-manifold edges"
    );
}

/// The planar face of `solid` whose outward normal is `direction`.
fn face_facing(topo: &Topology, solid: SolidId, direction: Vec3) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            let data = topo.face(face).unwrap();
            data.effective_plane_normal()
                .is_some_and(|n| n.dot(direction) > 1.0 - 1e-9)
        })
        .expect("face with that normal")
}

/// A 20 x 10 x 10 box with the two +X edges parallel to Z filleted (r = 2):
/// moving +X translates the +X face and both blend bands.
fn filleted_box(topo: &mut Topology) -> SolidId {
    let sharp = crate::primitives::make_box(topo, 20.0, 10.0, 10.0).unwrap();
    let edges: Vec<_> = solid_edges(topo, sharp)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let data = topo.edge(edge).unwrap();
            let a = topo.vertex(data.start()).unwrap().point();
            let b = topo.vertex(data.end()).unwrap().point();
            a.x() > 19.0 && b.x() > 19.0 && (a.z() - b.z()).abs() > 9.0
        })
        .collect();
    assert_eq!(edges.len(), 2, "two +X vertical edges");
    crate::blend_ops::fillet_v2(topo, sharp, &edges, 2.0)
        .unwrap()
        .solid
}

#[test]
fn cache_is_opt_in_and_per_thread() {
    disable_face_mesh_cache();
    assert!(face_mesh_cache_stats().is_none());
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
    grouped(&topo, solid, DEFLECTION);
    assert!(
        face_mesh_cache_stats().is_none(),
        "disabled cache stays off"
    );

    let _guard = CacheGuard::enabled();
    grouped(&topo, solid, DEFLECTION);
    let other_thread = std::thread::spawn(face_mesh_cache_stats).join().unwrap();
    assert!(
        other_thread.is_none(),
        "another thread has its own (off) cache"
    );
    assert_eq!(stats().stored, 6);
}

#[test]
fn cold_and_warm_meshes_equal_fresh_bit_for_bit() {
    let _guard = CacheGuard::enabled();
    let mut topo = Topology::new();
    let solid = filleted_box(&mut topo);
    let reference = fresh(&topo, solid, DEFLECTION);
    let faces = solid_faces(&topo, solid).unwrap().len() as u64;

    let s0 = stats();
    let cold = grouped(&topo, solid, DEFLECTION);
    let s1 = stats();
    assert_reproduces(&cold, &reference, 0.0, "cold");
    assert_eq!(delta(s0, s1).misses, faces);
    assert_eq!(s1.stored - s0.stored, faces);

    let warm = grouped(&topo, solid, DEFLECTION);
    let s2 = stats();
    assert_reproduces(&warm, &reference, 0.0, "warm");
    assert_watertight(&warm.0, "warm");
    assert_eq!(
        delta(s1, s2),
        Delta {
            exact: faces,
            translated: 0,
            misses: 0,
            refused: 0,
            conflicts: 0,
            uncacheable: 0
        }
    );
}

#[test]
fn moved_face_of_filleted_box_reuses_untouched_and_translated_faces() {
    let _guard = CacheGuard::enabled();
    let mut topo = Topology::new();
    let solid = filleted_box(&mut topo);
    grouped(&topo, solid, DEFLECTION);

    let plus_x = face_facing(&topo, solid, Vec3::new(1.0, 0.0, 0.0));
    let moved = crate::push_pull::move_faces(&mut topo, solid, &[plus_x], 2.0).unwrap();
    let reference = fresh(&topo, moved, DEFLECTION);

    let s0 = stats();
    let reused = grouped(&topo, moved, DEFLECTION);
    let d = delta(s0, stats());
    assert_reproduces(&reused, &reference, 64.0, "moved filleted box");
    assert_watertight(&reused.0, "moved filleted box");
    // -X is untouched; +X and both blend bands translate by exactly (2, 0, 0)
    // (their charts and the +X projection are bit-identical); the four
    // faces spanning the move (+-Y, +-Z) are re-limited.
    assert_eq!(
        d,
        Delta {
            exact: 1,
            translated: 3,
            misses: 4,
            refused: 0,
            conflicts: 0,
            uncacheable: 0
        }
    );
}

#[test]
fn unrelated_edits_and_tolerances_miss() {
    let _guard = CacheGuard::enabled();
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, 4.0, 5.0, 6.0).unwrap();
    grouped(&topo, solid, DEFLECTION);

    // A different deflection or angular tolerance never shares an entry.
    let s0 = stats();
    let coarser = grouped(&topo, solid, 2.0 * DEFLECTION);
    assert_eq!(delta(s0, stats()).misses, 6);
    assert_reproduces(
        &coarser,
        &fresh(&topo, solid, 2.0 * DEFLECTION),
        0.0,
        "coarser",
    );
    let s1 = stats();
    tessellate_solid_grouped_with_tolerance(&topo, solid, DEFLECTION, 0.5 * ANGULAR).unwrap();
    assert_eq!(delta(s1, stats()).misses, 6);
    // The boolean policy (circle floor) keys separately from display.
    let s2 = stats();
    tessellate_solid_for_boolean(&topo, solid, DEFLECTION, ANGULAR).unwrap();
    assert_eq!(delta(s2, stats()).misses, 6);

    // Moving -Z: -Z translates along its dropped axis, +Z is untouched, the
    // four sides are re-limited.
    let minus_z = face_facing(&topo, solid, Vec3::new(0.0, 0.0, -1.0));
    let moved = crate::push_pull::move_faces(&mut topo, solid, &[minus_z], 1.0).unwrap();
    let reference = fresh(&topo, moved, DEFLECTION);
    let s3 = stats();
    let reused = grouped(&topo, moved, DEFLECTION);
    assert_eq!(
        delta(s3, stats()),
        Delta {
            exact: 1,
            translated: 1,
            misses: 4,
            refused: 0,
            conflicts: 0,
            uncacheable: 0
        }
    );
    assert_reproduces(&reused, &reference, 64.0, "moved box");
}

#[test]
fn in_place_mutations_never_serve_stale_meshes() {
    let _guard = CacheGuard::enabled();
    let mut topo = Topology::new();
    let solid = crate::primitives::make_cylinder(&mut topo, 3.0, 8.0).unwrap();
    let original = grouped(&topo, solid, DEFLECTION);

    // transform_solid: a rotation changes every direction, so nothing hits.
    let mut rotated = topo.clone();
    crate::transform::transform_solid(&mut rotated, solid, &Mat4::rotation_x(0.7)).unwrap();
    let s0 = stats();
    let mesh = grouped(&rotated, solid, DEFLECTION);
    let d = delta(s0, stats());
    assert_eq!(
        (d.exact, d.translated),
        (0, 0),
        "rotation reused a face: {d:?}"
    );
    assert_reproduces(&mesh, &fresh(&rotated, solid, DEFLECTION), 0.0, "rotated");

    // transform_solid: a translation reuses only bit-exact charts.
    let mut shifted = topo.clone();
    crate::transform::transform_solid(&mut shifted, solid, &Mat4::translation(0.0, 0.0, 5.0))
        .unwrap();
    let mesh = grouped(&shifted, solid, DEFLECTION);
    assert_reproduces(
        &mesh,
        &fresh(&shifted, solid, DEFLECTION),
        64.0,
        "translated",
    );
    assert_watertight(&mesh.0, "translated");

    // `*_mut`: lengthen a box in place by moving the +X face's vertices and
    // plane through the arena accessors; handles are unchanged.
    let mut boxed = Topology::new();
    let block = crate::primitives::make_box(&mut boxed, 4.0, 5.0, 6.0).unwrap();
    grouped(&boxed, block, DEFLECTION);
    let plus_x = face_facing(&boxed, block, Vec3::new(1.0, 0.0, 0.0));
    for vertex in remus_topology::explorer::face_vertices(&boxed, plus_x).unwrap() {
        let p = boxed.vertex(vertex).unwrap().point();
        boxed
            .vertex_mut(vertex)
            .unwrap()
            .set_point(Point3::new(p.x() + 3.0, p.y(), p.z()));
    }
    let FaceSurface::Plane { normal, d } = boxed.face(plus_x).unwrap().surface().clone() else {
        panic!("planar face");
    };
    boxed
        .face_mut(plus_x)
        .unwrap()
        .set_surface(FaceSurface::Plane {
            normal,
            d: d + 3.0 * normal.x(),
        });
    let s1 = stats();
    let mesh = grouped(&boxed, block, DEFLECTION);
    let d = delta(s1, stats());
    assert_eq!(
        (d.exact, d.translated, d.misses),
        (1, 1, 4),
        "same handles, new content: {d:?}"
    );
    assert_reproduces(&mesh, &fresh(&boxed, block, DEFLECTION), 64.0, "vertex_mut");

    // Rolled-back edit: a failed transaction restores the content, which
    // then hits exactly.
    let before = grouped(&topo, solid, DEFLECTION);
    let rolled: Result<(), crate::OperationsError> =
        remus_topology::transaction::run_transacted(&mut topo, |t| {
            crate::transform::transform_solid(t, solid, &Mat4::translation(1.0, 2.0, 3.0))?;
            Err(crate::OperationsError::InvalidInput {
                reason: "abort".into(),
            })
        });
    assert!(rolled.is_err());
    let s2 = stats();
    let mesh = grouped(&topo, solid, DEFLECTION);
    assert_eq!(delta(s2, stats()).exact, 3, "rolled-back body hits exactly");
    assert_reproduces(&mesh, &before, 0.0, "rollback");
    assert_reproduces(&mesh, &original, 0.0, "rollback vs original");

    // Checkpoint restore over a moved body.
    let snapshot = topo.clone();
    let cap = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| topo.face(face).unwrap().surface().is_planar())
        .unwrap();
    let moved = crate::push_pull::move_faces(&mut topo, solid, &[cap], 1.5).unwrap();
    let edited = grouped(&topo, moved, DEFLECTION);
    assert_reproduces(
        &edited,
        &fresh(&topo, moved, DEFLECTION),
        64.0,
        "moved cylinder",
    );
    topo.restore_preserving_handle_slots(&snapshot);
    let mesh = grouped(&topo, solid, DEFLECTION);
    assert_reproduces(&mesh, &original, 0.0, "restored");

    // Heal: whatever healing changes, the cached mesh is the fresh one.
    let mut healed = Topology::new();
    let block = crate::primitives::make_box(&mut healed, 3.0, 4.0, 5.0).unwrap();
    grouped(&healed, block, DEFLECTION);
    crate::heal::heal_solid(&mut healed, block, 1e-7).unwrap();
    let mesh = grouped(&healed, block, DEFLECTION);
    assert_reproduces(&mesh, &fresh(&healed, block, DEFLECTION), 0.0, "healed");

    // Delete, then rebuild identical content under new handles: it hits
    // exactly and the deleted handle no longer tessellates.
    topo.delete_solid(solid).unwrap();
    assert!(tessellate_solid_grouped_with_tolerance(&topo, solid, DEFLECTION, ANGULAR).is_err());
    let rebuilt = crate::primitives::make_cylinder(&mut topo, 3.0, 8.0).unwrap();
    let s3 = stats();
    let mesh = grouped(&topo, rebuilt, DEFLECTION);
    assert_eq!(delta(s3, stats()).exact, 3);
    assert_reproduces(&mesh, &original, 0.0, "rebuilt");
}

#[test]
fn holes_seams_cavities_and_sheets_reproduce_fresh_meshes() {
    let _guard = CacheGuard::enabled();

    // Holes: a drilled block (holed planar caps are CDT jobs and stay
    // uncached; the bore wall is cached) and a seamed cylinder wall.
    let mut topo = Topology::new();
    let block = crate::primitives::make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
    let tool = crate::primitives::make_cylinder(&mut topo, 3.0, 20.0).unwrap();
    crate::transform::transform_solid(&mut topo, tool, &Mat4::translation(10.0, 10.0, -5.0))
        .unwrap();
    let drilled =
        crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Cut, block, tool).unwrap();
    // Cavity: a hollowed box carries an inner shell.
    let solid = crate::primitives::make_box(&mut topo, 6.0, 6.0, 6.0).unwrap();
    let open = face_facing(&topo, solid, Vec3::new(0.0, 0.0, 1.0));
    let hollow = crate::shell_op::shell(&mut topo, solid, 1.0, &[open]).unwrap();
    let sphere = crate::primitives::make_sphere(&mut topo, 4.0, 24).unwrap();
    let torus = crate::primitives::make_torus(&mut topo, 5.0, 1.5, 48).unwrap();
    let cone = crate::primitives::make_cone(&mut topo, 3.0, 1.0, 4.0).unwrap();

    for (label, body) in [
        ("drilled", drilled),
        ("hollow", hollow),
        ("sphere", sphere),
        ("torus", torus),
        ("cone", cone),
    ] {
        let reference = fresh(&topo, body, DEFLECTION);
        let cold = grouped(&topo, body, DEFLECTION);
        let s0 = stats();
        let warm = grouped(&topo, body, DEFLECTION);
        let d = delta(s0, stats());
        assert_reproduces(&cold, &reference, 0.0, label);
        assert_reproduces(&warm, &reference, 0.0, label);
        assert_watertight(&warm.0, label);
        assert!(d.exact > 0, "{label}: warm run reused nothing: {d:?}");
        // Only faces that refused capture (a corner on another face's
        // vertex, e.g. the sphere's snapped equator) are re-meshed.
        assert_eq!(d.misses, d.uncacheable, "{label}: {d:?}");
        assert_eq!(d.conflicts, 0, "{label}: {d:?}");
    }

    // Sheets: the open sheet path shares the pipeline and the cache.
    let corner = crate::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let faces: Vec<_> = solid_faces(&topo, corner)
        .unwrap()
        .into_iter()
        .take(3)
        .collect();
    let sheet = crate::sew::make_sheet_body(&mut topo, &faces).unwrap();
    let parked = super::super::face_cache::park();
    let reference = tessellate_sheet(&topo, sheet, DEFLECTION).unwrap();
    super::super::face_cache::unpark(parked);
    tessellate_sheet(&topo, sheet, DEFLECTION).unwrap();
    let s0 = stats();
    let warm = tessellate_sheet(&topo, sheet, DEFLECTION).unwrap();
    assert!(delta(s0, stats()).exact > 0);
    assert_eq!(warm.indices, reference.indices);
    assert_eq!(warm.positions.len(), reference.positions.len());
}

#[test]
fn bounds_evict_fifo_and_refuse_oversized_entries() {
    let _guard = CacheGuard::enabled();
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

    enable_face_mesh_cache_with_limits(4, usize::MAX);
    grouped(&topo, solid, DEFLECTION);
    let s = stats();
    assert_eq!((s.entries, s.stored, s.evictions), (4, 6, 2));
    // FIFO: the first two faces were evicted and miss, the rest hit.
    let s0 = stats();
    let mesh = grouped(&topo, solid, DEFLECTION);
    let d = delta(s0, stats());
    assert_eq!((d.exact, d.misses), (4, 2), "{d:?}");
    assert_reproduces(&mesh, &fresh(&topo, solid, DEFLECTION), 0.0, "fifo");

    // An entry larger than the byte budget is never retained.
    clear_face_mesh_cache();
    enable_face_mesh_cache_with_limits(64, 16);
    grouped(&topo, solid, DEFLECTION);
    let s = stats();
    assert_eq!((s.entries, s.retained_bytes), (0, 0));

    // Shrinking the bounds evicts to fit.
    enable_face_mesh_cache_with_limits(64, usize::MAX);
    grouped(&topo, solid, DEFLECTION);
    assert_eq!(stats().entries, 6);
    enable_face_mesh_cache_with_limits(3, usize::MAX);
    assert_eq!(stats().entries, 3);
    assert!(stats().retained_bytes <= stats().max_bytes);

    // Zero capacity keeps the cache on but retains nothing.
    enable_face_mesh_cache_with_limits(0, usize::MAX);
    grouped(&topo, solid, DEFLECTION);
    assert_eq!(stats().entries, 0);
}

/// A square-to-square smooth loft through a wider middle section: four
/// curved, non-periodic NURBS side faces without pcurves, placed so that
/// every coordinate stays in its binade under the test's +8 X translation.
fn lofted_bulge(topo: &mut Topology) -> SolidId {
    let square = |topo: &mut Topology, half: f64, z: f64| {
        let (cx, cy) = (45.0, 45.0);
        let wire = remus_topology::builder::make_polygon_wire(
            topo,
            &[
                Point3::new(cx - half, cy - half, z),
                Point3::new(cx + half, cy - half, z),
                Point3::new(cx + half, cy + half, z),
                Point3::new(cx - half, cy + half, z),
            ],
            1e-7,
        )
        .unwrap();
        topo.add_face(remus_topology::face::Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: z,
            },
        ))
    };
    let profiles = [
        square(topo, 3.0, 40.0),
        square(topo, 4.5, 44.0),
        square(topo, 2.5, 50.0),
    ];
    crate::loft::loft_smooth(topo, &profiles).unwrap()
}

/// Count of faces of `solid` whose carrier is NURBS.
fn nurbs_faces(topo: &Topology, solid: SolidId) -> u64 {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_)))
        .count() as u64
}

#[test]
fn translated_nurbs_faces_are_reused_exactly() {
    let _guard = CacheGuard::enabled();
    let mut topo = Topology::new();
    // Curved NURBS (smooth loft) and bilinear NURBS (converted box) carriers.
    let loft = lofted_bulge(&mut topo);
    let bar = crate::primitives::make_box(&mut topo, 10.0, 6.0, 4.0).unwrap();
    crate::transform::transform_solid(&mut topo, bar, &Mat4::translation(36.0, 40.0, 40.0))
        .unwrap();
    assert!(crate::heal::convert_to_bspline(&mut topo, bar).unwrap() > 0);

    for (label, body, deflection) in [
        ("loft", loft, DEFLECTION),
        ("loft fine", loft, 0.01),
        ("converted box", bar, DEFLECTION),
    ] {
        let nurbs = nurbs_faces(&topo, body);
        assert!(nurbs >= 4, "{label}: {nurbs} NURBS faces");
        let source = grouped(&topo, body, deflection);

        // Translate the body in place by an exactly representable step that
        // keeps every coordinate in its binade: every carrier, vertex and
        // boundary sample moves by exactly (8, 0, 0).
        let mut moved_topo = topo.clone();
        crate::transform::transform_solid(&mut moved_topo, body, &Mat4::translation(8.0, 0.0, 0.0))
            .unwrap();
        let reference = fresh(&moved_topo, body, deflection);
        let s0 = stats();
        let reused = grouped(&moved_topo, body, deflection);
        let d = delta(s0, stats());

        // The NURBS faces are served by translation (planar caps facing the
        // translation re-mesh: their projection keeps the moved X).
        assert!(
            d.translated >= nurbs,
            "{label}: only {} of {nurbs} NURBS faces translated: {d:?}",
            d.translated
        );
        assert_eq!(d.conflicts, 0, "{label}: {d:?}");
        assert_eq!(d.uncacheable, 0, "{label}: {d:?}");
        // Connectivity and normals are the fresh mesh's exactly; positions
        // within a few units of roundoff of the body scale.
        assert_reproduces(&reused, &reference, 4.0, label);
        assert_watertight(&reused.0, label);
        assert_eq!(reused.0.indices.len(), source.0.indices.len(), "{label}");
    }
}

#[test]
fn inexact_nurbs_translates_are_refused() {
    let _guard = CacheGuard::enabled();
    let mut topo = Topology::new();
    let loft = lofted_bulge(&mut topo);
    grouped(&topo, loft, DEFLECTION);

    // Moving by (20, 20, 0) carries part of every side face across 64 in X
    // or Y, where the unit of roundoff doubles: those coordinates round, so
    // the faces are translates within roundoff but not exact ones. They
    // must be re-meshed (refused), and the result is then the fresh mesh.
    let mut moved_topo = topo.clone();
    crate::transform::transform_solid(&mut moved_topo, loft, &Mat4::translation(20.0, 20.0, 0.0))
        .unwrap();
    let reference = fresh(&moved_topo, loft, DEFLECTION);
    let s0 = stats();
    let reused = grouped(&moved_topo, loft, DEFLECTION);
    let d = delta(s0, stats());
    assert!(d.refused >= 1, "{d:?}");
    assert_eq!(d.refused + d.translated, 6, "{d:?}");
    assert_reproduces(&reused, &reference, 64.0, "translate across a binade");

    // An inexact step within one binade is still an exact translate (by the
    // rounded step), and is reused.
    let mut nudged = topo.clone();
    crate::transform::transform_solid(&mut nudged, loft, &Mat4::translation(0.1, 0.0, 0.0))
        .unwrap();
    let reference = fresh(&nudged, loft, DEFLECTION);
    let s0 = stats();
    let reused = grouped(&nudged, loft, DEFLECTION);
    let d = delta(s0, stats());
    assert_eq!(d.translated, nurbs_faces(&topo, loft), "{d:?}");
    assert_reproduces(&reused, &reference, 4.0, "translate by a rounded step");
}
