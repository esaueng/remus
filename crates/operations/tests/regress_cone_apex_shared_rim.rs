//! Regression: a point-tipped cone tessellated to an open mesh.
//!
//! `make_cone(r = 3, top = 0, h = 1)` at `deflection = diag * 4e-5` produced
//! 416 triangles, 418 boundary edges and 420 vertices where 210 suffice: the
//! lateral cone and the base disc each emitted their own copy of the base
//! circle and shared no vertex at all.
//!
//! The two copies were not near-coincident — the closest pair of vertices in
//! that mesh was 2.25e-2 apart, five orders of magnitude beyond any merge or
//! snap tolerance in the pipeline. They were interleaved: both rings had 209
//! points at the same spacing, offset by exactly HALF a segment.
//!
//! Cause. A pointed cone's lateral face is bounded by one closed rim circle and
//! a doubled degenerate seam to the apex, so `tessellate_revolution_band_shared`
//! (two rims) declined it and the CDT path emitted nothing (the seam collapses
//! the UV boundary). The face fell through to `tessellate_nonplanar_snap`, which
//! tessellates from the cone's OWN parametric grid and then reconciles with the
//! shared edge pool by proximity. The cone surface's `u = 0` ray and the base
//! circle's `t = 0` ray are half a turn apart — `make_cone` gives the base
//! circle normal `+axis` while the cone's axis runs apex→base — so with `n`
//! segments the two rings coincide only when `n` is EVEN. At `r = 3, h = 1` and
//! that deflection `n = 209`, and every rim sample landed half a segment from
//! its counterpart.
//!
//! That parity is why the defect looked scale-dependent: a 0.001x copy took
//! `n = 380` (even, and larger only because the face's segment count is floored
//! by an absolute `max_radius.max(0.01)` clamp) and closed by luck, while 1x and
//! 1000x took 209 and leaked.
//!
//! The fix fans the shared rim to the shared apex instead, so the cone meets its
//! cap by vertex identity at every radius, deflection and scale. The oracle here
//! is the mesh, not a volume: `solid_volume` returned the exact
//! `πr²h/3 = 9.42477796076938` for the broken mesh too.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::det_hash::DetHashSet;
use remus_operations::measure::{solid_bounding_box, solid_volume};
use remus_operations::primitives::make_cone;
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count, tessellate_solid,
    tessellate_solid_grouped_with_tolerance,
};
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// Deflection used by the structured fuzz harness's mesh rung.
const FUZZ_K: f64 = 4e-5;

fn diag(topo: &Topology, solid: SolidId) -> f64 {
    let aabb = solid_bounding_box(topo, solid).unwrap();
    (aabb.max - aabb.min).length()
}

/// Vertex ids actually referenced by a triangle.
fn referenced(mesh: &TriangleMesh) -> DetHashSet<u32> {
    mesh.indices.iter().copied().collect()
}

/// Referenced vertex ids whose position sits on the plane `z = z0`.
///
/// `z` is compared against the model's own extent, so the predicate means the
/// same thing at every scale.
fn ring_ids(mesh: &TriangleMesh, ids: &DetHashSet<u32>, z0: f64, extent: f64) -> Vec<u32> {
    let mut out: Vec<u32> = ids
        .iter()
        .copied()
        .filter(|&i| (mesh.positions[i as usize].z() - z0).abs() <= extent * 1e-9)
        .collect();
    out.sort_unstable();
    out
}

/// Assert a pointed cone of base radius `r` and height `h` tessellates closed,
/// with ONE base ring shared by the lateral face and the cap. Returns the ring
/// length so callers can check its parity.
fn assert_pointed_cone_closed(r: f64, h: f64, k: f64) -> usize {
    let mut topo = Topology::new();
    let solid = make_cone(&mut topo, r, 0.0, h).unwrap();
    let extent = diag(&topo, solid);
    let deflection = extent * k;
    let what = format!("cone r={r} h={h} k={k:e}");

    let mesh = tessellate_solid(&topo, solid, deflection).unwrap();
    assert_eq!(
        boundary_edge_count(&mesh),
        0,
        "{what}: mesh has open edges ({} tris, {} verts)",
        mesh.indices.len() / 3,
        mesh.positions.len()
    );
    assert_eq!(non_manifold_edge_count(&mesh), 0, "{what}: non-manifold");

    let used = referenced(&mesh);
    let base = ring_ids(&mesh, &used, 0.0, extent);
    assert!(base.len() >= 4, "{what}: base ring is only {}", base.len());

    // The whole cone is one ring plus one apex. A duplicated ring shows here as
    // 2n + 1 long before it shows as a leak.
    assert_eq!(
        used.len(),
        base.len() + 1,
        "{what}: {} referenced vertices for a {}-point ring + apex — the ring is \
         duplicated",
        used.len(),
        base.len()
    );

    // Every base vertex is used by BOTH faces, by id. This is the property the
    // shared edge pool exists to provide, and the one the snap fallback lost.
    let (grouped, offsets) = tessellate_solid_grouped_with_tolerance(
        &topo,
        solid,
        deflection,
        remus_math::chord::DEFAULT_ANGULAR_TOL,
    )
    .unwrap();
    let mut per_face_rings: Vec<Vec<u32>> = Vec::new();
    for f in 0..offsets.len() - 1 {
        let (s, e) = (offsets[f] as usize, offsets[f + 1] as usize);
        let ids: DetHashSet<u32> = grouped.indices[s..e].iter().copied().collect();
        let ring = ring_ids(&grouped, &ids, 0.0, extent);
        if !ring.is_empty() {
            per_face_rings.push(ring);
        }
    }
    assert_eq!(
        per_face_rings.len(),
        2,
        "{what}: expected the lateral face and the cap to meet the base plane"
    );
    assert_eq!(
        per_face_rings[0], per_face_rings[1],
        "{what}: the cone and its cap use different base-ring vertex ids"
    );

    // No two distinct mesh vertices may sit on top of each other: a merge that
    // was too eager would show up here rather than as a leak.
    let min_sep = min_separation(&mesh);
    assert!(
        min_sep > extent * 1e-6,
        "{what}: two vertices only {min_sep:e} apart (extent {extent:e})"
    );

    // Not an oracle for the crack — the broken mesh measured exactly right too
    // — but the fan must not have changed what the solid is.
    let exact = std::f64::consts::PI * r * r * h / 3.0;
    let vol = solid_volume(&topo, solid, deflection).unwrap();
    assert!(
        (vol - exact).abs() <= exact * 1e-12,
        "{what}: volume {vol} != {exact}"
    );

    base.len()
}

/// Smallest distance between two distinct mesh vertices (sweep on x).
fn min_separation(mesh: &TriangleMesh) -> f64 {
    let mut pts = mesh.positions.clone();
    pts.sort_by(|a, b| a.x().total_cmp(&b.x()));
    let mut best = f64::INFINITY;
    for i in 0..pts.len() {
        for j in (i + 1)..pts.len() {
            if pts[j].x() - pts[i].x() > best {
                break;
            }
            best = best.min((pts[i] - pts[j]).length());
        }
    }
    best
}

/// The reported reproduction, exactly as the fuzz harness hit it.
#[test]
fn pointed_cone_base_ring_is_shared() {
    let n = assert_pointed_cone_closed(3.0, 1.0, FUZZ_K);
    // The count that used to break it. Pinned so a density change that quietly
    // moves off the odd case cannot make this test pass vacuously.
    assert_eq!(
        n, 209,
        "the reported case samples the base circle 209 times"
    );
    assert!(
        !n.is_multiple_of(2),
        "…and 209 is odd, which is what broke the snap"
    );
}

/// Both parities of the segment count must close. The old snap path closed only
/// on the even ones.
#[test]
fn pointed_cone_closes_at_every_deflection() {
    let mut odd = 0;
    let mut even = 0;
    for k in [
        1.6e-3, 8e-4, 4e-4, 2e-4, 1.6e-4, 8e-5, 4e-5, 2e-5, 1e-5, 4e-6,
    ] {
        for (r, h) in [(3.0, 1.0), (1.0, 4.0), (0.5, 0.2), (12.0, 7.0)] {
            let n = assert_pointed_cone_closed(r, h, k);
            if n.is_multiple_of(2) {
                even += 1;
            } else {
                odd += 1;
            }
        }
    }
    assert!(
        odd > 0 && even > 0,
        "sweep must exercise both parities (odd {odd}, even {even})"
    );
}

/// Same shape at three scales spanning six decades.
#[test]
fn pointed_cone_closes_at_every_scale() {
    let mut counts = Vec::new();
    for s in [0.001, 1.0, 1000.0] {
        counts.push((s, assert_pointed_cone_closed(3.0 * s, 1.0 * s, FUZZ_K)));
    }
    // 1x and 1000x must tessellate identically — the density formula is a pure
    // ratio of lengths there. The 0.001x copy legitimately differs: the face
    // segment count is floored by an absolute `max_radius.max(0.01)` clamp in
    // `tessellate::face`, a separate scale dependence this fix does not touch.
    assert_eq!(
        counts[1].1, counts[2].1,
        "1x and 1000x disagree on the ring: {counts:?}"
    );
}

/// A cone whose tip is nearly, but not exactly, a point is a FRUSTUM: it keeps
/// two rims, and the tiny top rim's vertices — a couple of hundred of them
/// inside a 6e-5-wide disc on a 6-wide model, adjacent pairs under 1e-6 apart —
/// must all stay distinct rather than weld to one. This is the failure mode
/// opposite to the one being fixed: merging harder closes the cone by silently
/// changing what the solid is.
#[test]
fn near_point_frustum_keeps_its_top_ring() {
    let r_top = 3.0 * 1e-5;
    let mut topo = Topology::new();
    let solid = make_cone(&mut topo, 3.0, r_top, 1.0).unwrap();
    let extent = diag(&topo, solid);
    let deflection = extent * FUZZ_K;

    let mesh = tessellate_solid(&topo, solid, deflection).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0, "near-point frustum leaks");
    assert_eq!(non_manifold_edge_count(&mesh), 0, "near-point frustum");

    let used = referenced(&mesh);
    let bottom = ring_ids(&mesh, &used, 0.0, extent);
    let top = ring_ids(&mesh, &used, 1.0, extent);
    assert!(
        top.len() >= 4,
        "the top rim collapsed to {} vertices — a merge welded points that are \
         close in absolute terms but distinct relative to the model",
        top.len()
    );
    assert_eq!(
        top.len(),
        bottom.len(),
        "the two rims should sample alike: {} vs {}",
        top.len(),
        bottom.len()
    );

    // Those top-rim points really are close together: this is the case a
    // length-carrying merge tolerance would swallow.
    let span = top
        .iter()
        .map(|&i| mesh.positions[i as usize])
        .fold(0.0_f64, |acc, p| {
            acc.max((p - mesh.positions[top[0] as usize]).length())
        });
    assert!(
        span < extent * 1e-4,
        "test is not exercising the close-vertex case: top rim spans {span:e}"
    );
}
