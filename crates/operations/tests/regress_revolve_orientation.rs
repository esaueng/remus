//! Regression: a partial revolve must come back wound OUTWARD, like a full one.
//!
//! `revolve` has three build paths: a circle→torus fast path, an analytic
//! full-turn path, and the general segmented NURBS path that handles everything
//! else. The segmented path oriented every face it built off the swept NURBS
//! band's `du × dv = (profile tangent) × (sweep tangent)` — a *consistent*
//! orientation, but only an *outward* one when the sweep runs along the profile
//! wire's CCW normal. Revolve the same profile the other way round the axis (or
//! hand it the opposite wire winding) and every face — bands and caps alike —
//! came out facing inward.
//!
//! Nothing else could see it. The shell stayed closed, 2-manifold, consistently
//! wound and correct-volume; `validate_solid` passed; `solid_volume` reads the
//! magnitude of its integral, so it reported the right number for an inside-out
//! solid. Only the winding SIGN differed, which is exactly what an STL facet
//! normal is derived from — so a wedge exported inside-out.
//!
//! The oracle here is therefore the SIGNED volume of the tessellated mesh,
//! `Σ a · (b × c) / 6`, which is positive only for an outward-wound closed
//! surface. Every case also asserts the properties that already passed before
//! the fix, so a future change cannot trade orientation away for one of them:
//! a watertight 2-manifold mesh (zero free and zero non-manifold edges), the
//! right Euler characteristic for that case's topology, and the volume against
//! a closed form derived here — Pappus's theorem, `V = θ · r̄ · A` — rather than
//! read back out of the kernel.
//!
//! Two traps this deliberately avoids:
//!
//! * `mass_properties` agreeing with `solid_volume` proves nothing — they share
//!   `integrate_face`. The comparison here is against the hand-derived closed
//!   form only.
//! * A single angle can pass by accident on segment-count parity:
//!   `arc_segmentation` uses `ceil(θ / 90°)` segments, so 45° and 90° build 1
//!   band ring, 180° builds 2, 270° builds 3, 359°/359.99° build 4. The sweep
//!   below spans both parities at every scale, and the scale sweep holds the
//!   segment count fixed per angle while moving the coordinates over six orders
//!   of magnitude, so neither can mask the other.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_operations::revolve::revolve;
use remus_operations::tessellate::{TriangleMesh, tessellate_solid};
use remus_topology::Topology;
use remus_topology::face::{Face, FaceId, FaceSurface};

/// Angles swept by every case, in degrees. Deliberately includes two
/// non-integer angles and both sides of the `is_full` cutoff.
const ANGLES_DEG: [f64; 8] = [45.0, 90.0, 137.5, 180.0, 270.0, 359.0, 359.99, 360.0];

/// Signed volume of a closed triangle mesh, `Σ a · (b × c) / 6`. Positive iff
/// the surface is wound outward. This is the property `writeAsciiStl` turns
/// into facet normals, and the only one that saw this defect.
fn signed_mesh_volume(mesh: &TriangleMesh) -> f64 {
    let mut acc = 0.0;
    for t in mesh.indices.chunks(3) {
        let a = mesh.positions[t[0] as usize];
        let b = mesh.positions[t[1] as usize];
        let c = mesh.positions[t[2] as usize];
        let av = Vec3::new(a.x(), a.y(), a.z());
        let bv = Vec3::new(b.x(), b.y(), b.z());
        let cv = Vec3::new(c.x(), c.y(), c.z());
        acc += av.dot(bv.cross(cv));
    }
    acc / 6.0
}

/// `(free_edges, non_manifold_edges, euler_characteristic)` of the mesh's
/// undirected edge graph. A closed 2-manifold has zero of the first two.
fn mesh_topology(mesh: &TriangleMesh) -> (usize, usize, i64) {
    let mut counts: HashMap<(u32, u32), usize> = HashMap::new();
    let mut verts: std::collections::HashSet<u32> = std::collections::HashSet::new();
    for t in mesh.indices.chunks(3) {
        for k in 0..3 {
            verts.insert(t[k]);
            let (a, b) = (t[k], t[(k + 1) % 3]);
            let key = if a < b { (a, b) } else { (b, a) };
            *counts.entry(key).or_insert(0) += 1;
        }
    }
    let free = counts.values().filter(|&&c| c == 1).count();
    let non_manifold = counts.values().filter(|&&c| c > 2).count();
    let v = i64::try_from(verts.len()).unwrap();
    let e = i64::try_from(counts.len()).unwrap();
    let f = i64::try_from(mesh.indices.len() / 3).unwrap();
    (free, non_manifold, v - e + f)
}

/// A planar polygon profile in the z = 0 plane, wound as given.
fn polygon_face(topo: &mut Topology, pts: &[(f64, f64)], normal_z: f64, tol: f64) -> FaceId {
    let pts3: Vec<Point3> = pts.iter().map(|&(x, y)| Point3::new(x, y, 0.0)).collect();
    let wire = remus_topology::builder::make_polygon_wire(topo, &pts3, tol).unwrap();
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, normal_z),
            d: 0.0,
        },
    ))
}

/// Shoelace area (signed) and centroid of a closed polygon — the closed-form
/// inputs to Pappus, computed here rather than read back from the kernel.
fn polygon_area_centroid(pts: &[(f64, f64)]) -> (f64, f64) {
    let n = pts.len();
    let mut a2 = 0.0;
    let mut cx = 0.0;
    for i in 0..n {
        let (x0, y0) = pts[i];
        let (x1, y1) = pts[(i + 1) % n];
        let cross = x0.mul_add(y1, -(x1 * y0));
        a2 += cross;
        cx += (x0 + x1) * cross;
    }
    let area = a2 / 2.0;
    let centroid_x = cx / (3.0 * a2);
    (area.abs(), centroid_x)
}

/// Pappus: a profile that does not cross the axis, revolved by `angle`, sweeps
/// `V = angle · r̄ · A`. Exact for a polygon profile revolved about the Y axis.
fn pappus_volume(pts: &[(f64, f64)], angle_rad: f64) -> f64 {
    let (area, centroid_x) = polygon_area_centroid(pts);
    angle_rad * centroid_x.abs() * area
}

struct Case {
    label: &'static str,
    /// Profile in (x, y), x = radial distance from the Y axis (may be negative:
    /// the mirrored profile on the other side of the axis).
    pts: Vec<(f64, f64)>,
    /// Stored plane normal's z component. `-1.0` pairs with a CW-wound `pts`.
    normal_z: f64,
    /// Revolution axis direction. Both senses are exercised: the defect's sign
    /// flipped with this, so a convention right for one was wrong for the other.
    axis: Vec3,
}

/// Assert one profile × one axis × the full angle sweep, at one scale.
fn check_case(case: &Case, scale: f64) {
    let pts: Vec<(f64, f64)> = case
        .pts
        .iter()
        .map(|&(x, y)| (x * scale, y * scale))
        .collect();
    let tol = 1e-9 * scale.min(1.0);
    let deflection = 0.005 * scale;

    for angle_deg in ANGLES_DEG {
        let angle = angle_deg.to_radians();
        let ctx = format!(
            "{} axis=({:.0},{:.0},{:.0}) scale={scale} angle={angle_deg}",
            case.label,
            case.axis.x(),
            case.axis.y(),
            case.axis.z()
        );

        let mut topo = Topology::new();
        let face = polygon_face(&mut topo, &pts, case.normal_z, tol);
        let solid = revolve(
            &mut topo,
            face,
            Point3::new(0.0, 0.0, 0.0),
            case.axis,
            angle,
        )
        .unwrap_or_else(|e| panic!("{ctx}: revolve failed: {e}"));

        let report = remus_operations::validate::validate_solid(&topo, solid).unwrap();
        assert!(report.is_valid(), "{ctx}: invalid solid: {report:?}");

        let mesh = tessellate_solid(&topo, solid, deflection).unwrap();
        let (free, non_manifold, chi) = mesh_topology(&mesh);
        assert_eq!(free, 0, "{ctx}: mesh has free edges");
        assert_eq!(non_manifold, 0, "{ctx}: mesh has non-manifold edges");

        // A partial revolve of a profile that clears the axis is a solid torus
        // SECTOR — a topological ball, χ = 2. Closed at 360° it is a solid
        // torus, whose boundary is a torus: χ = 0. The two differ, so the
        // right value is asserted per case rather than 2 everywhere.
        let expected_chi = if angle_deg >= 360.0 { 0 } else { 2 };
        assert_eq!(chi, expected_chi, "{ctx}: unexpected Euler characteristic");

        // The oracle. Before the fix this was NEGATIVE for every partial angle
        // (and for a full turn that fell through to the segmented path).
        let signed = signed_mesh_volume(&mesh);
        assert!(
            signed > 0.0,
            "{ctx}: shell is wound inward (signed mesh volume {signed:.6e})"
        );

        // Magnitude against the hand-derived closed form. The kernel's own
        // integrator is compared to Pappus — NOT to `mass_properties`, which
        // shares `integrate_face` with it and so cannot corroborate anything.
        let expected = pappus_volume(&pts, angle);
        let kernel = remus_operations::measure::solid_volume(&topo, solid, deflection).unwrap();
        let kernel_err = (kernel - expected).abs() / expected;
        // The kernel's linear tolerance is ABSOLUTE (1e-7), so its accuracy is
        // bounded relative to the profile's smallest radius (2 × `scale` here).
        // At 1× and 1000× the integral is exact to ~1e-15; at 0.001× it degrades
        // to ~3e-5, exactly `Tolerance::linear / min_radius`. That is a
        // pre-existing property of an absolute-tolerance kernel and is a
        // MAGNITUDE effect — the orientation sign asserted above is clean at all
        // three scales, so this bound must not be read as a scale-dependent
        // orientation weakness.
        let vol_tol = 1e-9 + Tolerance::new().linear / (2.0 * scale);
        assert!(
            kernel_err < vol_tol,
            "{ctx}: solid_volume {kernel:.9e} vs Pappus {expected:.9e} (rel {kernel_err:.2e})"
        );
        // The mesh chords the swept arcs, so its volume sits just under the
        // closed form; the bound doubles as a guard that the positive sign
        // above is not positive by landing on some unrelated shape.
        let mesh_err = (signed - expected).abs() / expected;
        assert!(
            mesh_err < 0.02,
            "{ctx}: signed mesh volume {signed:.9e} vs Pappus {expected:.9e} \
             (rel {mesh_err:.2e})"
        );
    }
}

fn cases() -> Vec<Case> {
    // A rectangle clear of the axis, and a pentagon whose edges classify as all
    // three analytic band kinds (axis-perpendicular → Plane, axis-parallel →
    // Cylinder, oblique → Cone), so the band-orientation flag is exercised for
    // each, not just for cylinders.
    let rect = vec![(2.0, 0.0), (3.0, 0.0), (3.0, 1.0), (2.0, 1.0)];
    let rect_cw = vec![(2.0, 0.0), (2.0, 1.0), (3.0, 1.0), (3.0, 0.0)];
    let pentagon = vec![(2.0, 0.0), (4.0, 0.0), (4.0, 1.0), (3.0, 3.0), (2.0, 3.0)];
    // The same rectangle on the far side of the axis. The sweep tangent points
    // the opposite way there, so a sign convention right for one is wrong for
    // the mirrored case unless it is derived from the geometry.
    let rect_mirrored = vec![(-2.0, 0.0), (-2.0, 1.0), (-3.0, 1.0), (-3.0, 0.0)];

    let mut out = Vec::new();
    for (label, pts, normal_z) in [
        ("ccw rect", rect, 1.0),
        ("cw rect", rect_cw, -1.0),
        ("ccw pentagon", pentagon, 1.0),
        ("mirrored rect", rect_mirrored, 1.0),
    ] {
        for axis in [Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, -1.0, 0.0)] {
            out.push(Case {
                label,
                pts: pts.clone(),
                normal_z,
                axis,
            });
        }
    }
    out
}

#[test]
fn partial_revolve_is_wound_outward() {
    for case in cases() {
        check_case(&case, 1.0);
    }
}

/// Same assertions with every coordinate scaled. The segment count per angle is
/// unchanged by scale, so this varies magnitude alone — it cannot pass by
/// landing on a lucky segment parity, and a scale-dependent tolerance leak
/// would show up as a validity or watertightness failure rather than a silent
/// sign flip.
#[test]
fn revolve_orientation_holds_across_scales() {
    for scale in [1000.0, 0.001] {
        for case in cases() {
            check_case(&case, scale);
        }
    }
}

/// A profile with a hole defers BOTH analytic fast paths, so this exercises the
/// segmented builder at a full turn as well as partial ones — the case that
/// showed the defect is not partial-only. The bore's bands must face into the
/// bore while the outer bands face out; getting only one of the two right still
/// leaves a closed shell, but not a positive signed volume.
#[test]
fn holed_profile_revolve_is_wound_outward() {
    // Outer 4×4 at x ∈ [2, 6], hole 2×2 at x ∈ [3, 5]; both clear of the axis.
    let outer = [(2.0, 0.0), (6.0, 0.0), (6.0, 4.0), (2.0, 4.0)];
    // The hole is wound opposite the outer wire.
    let inner = [(3.0, 1.0), (3.0, 3.0), (5.0, 3.0), (5.0, 1.0)];

    for axis in [Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, -1.0, 0.0)] {
        for angle_deg in ANGLES_DEG {
            let angle = angle_deg.to_radians();
            let ctx = format!("holed axis_y={:.0} angle={angle_deg}", axis.y());

            let mut topo = Topology::new();
            let to3 = |pts: &[(f64, f64)]| -> Vec<Point3> {
                pts.iter().map(|&(x, y)| Point3::new(x, y, 0.0)).collect()
            };
            let ow =
                remus_topology::builder::make_polygon_wire(&mut topo, &to3(&outer), 1e-9).unwrap();
            let iw =
                remus_topology::builder::make_polygon_wire(&mut topo, &to3(&inner), 1e-9).unwrap();
            let face = topo.add_face(Face::new(
                ow,
                vec![iw],
                FaceSurface::Plane {
                    normal: Vec3::new(0.0, 0.0, 1.0),
                    d: 0.0,
                },
            ));

            let solid = revolve(&mut topo, face, Point3::new(0.0, 0.0, 0.0), axis, angle)
                .unwrap_or_else(|e| panic!("{ctx}: revolve failed: {e}"));

            let mesh = tessellate_solid(&topo, solid, 0.01).unwrap();
            let (free, non_manifold, chi) = mesh_topology(&mesh);
            assert_eq!(free, 0, "{ctx}: mesh has free edges");
            assert_eq!(non_manifold, 0, "{ctx}: mesh has non-manifold edges");
            // Annulus × interval is a solid torus (χ = 0 boundary) for a partial
            // turn; closed up at 360° it is a hollow torus tube whose boundary
            // is two disjoint tori, also χ = 0.
            assert_eq!(chi, 0, "{ctx}: unexpected Euler characteristic");

            let signed = signed_mesh_volume(&mesh);
            assert!(
                signed > 0.0,
                "{ctx}: shell is wound inward (signed mesh volume {signed:.6e})"
            );

            // Pappus on the annular profile: outer sweep minus hole sweep.
            let expected = pappus_volume(&outer, angle) - pappus_volume(&inner, angle);
            let kernel = remus_operations::measure::solid_volume(&topo, solid, 0.01).unwrap();
            let kernel_err = (kernel - expected).abs() / expected;
            assert!(
                kernel_err < 1e-3,
                "{ctx}: solid_volume {kernel:.9e} vs Pappus {expected:.9e} \
                 (rel {kernel_err:.2e})"
            );
            let mesh_err = (signed - expected).abs() / expected;
            assert!(
                mesh_err < 0.02,
                "{ctx}: signed mesh volume {signed:.9e} vs Pappus {expected:.9e} \
                 (rel {mesh_err:.2e})"
            );
        }
    }
}
