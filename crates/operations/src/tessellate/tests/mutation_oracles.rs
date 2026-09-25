//! B19 tessellation mutation tranche (weekly run 36075171651): tests that
//! kill surviving mutants in the band meshers, the CDT decline gates and the
//! density sizers with oracles independent of the code under test. See
//! `docs/kernel-maturity/mutants-tessellate-2026-09-25.md` for the triage.

#![allow(clippy::unwrap_used, clippy::panic)]

use std::f64::consts::TAU;

use remus_math::mat::Mat4;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;

use super::mesh_oracles::{closed_mesh_by_face, inverted_triangles, signed_volume, vertex_and_sag};

/// Composite Simpson over `[a, b]` with `n` (even) panels.
fn simpson(f: impl Fn(f64) -> f64, a: f64, b: f64, n: usize) -> f64 {
    #[allow(clippy::cast_precision_loss)]
    let h = (b - a) / n as f64;
    let mut sum = f(a) + f(b);
    for i in 1..n {
        #[allow(clippy::cast_precision_loss)]
        let x = h.mul_add(i as f64, a);
        sum += if i % 2 == 1 { 4.0 } else { 2.0 } * f(x);
    }
    sum * h / 3.0
}

/// Every analytic face: vertices on the carrier, chords within twice the
/// deflection (the bound `test_max_sag_within_deflection` uses), and every
/// triangle facing out of the material.
fn assert_faces_follow_their_carriers(
    faces: &[super::mesh_oracles::FaceTris],
    deflection: f64,
    what: &str,
) {
    assert_faces_follow_their_carriers_at_scale(faces, deflection, 1.0, what);
}

/// [`assert_faces_follow_their_carriers`] for a model scaled by `scale`.
fn assert_faces_follow_their_carriers_at_scale(
    faces: &[super::mesh_oracles::FaceTris],
    deflection: f64,
    scale: f64,
    what: &str,
) {
    for (i, face) in faces.iter().enumerate() {
        if matches!(face.surface, FaceSurface::Nurbs(_)) {
            continue;
        }
        let (vertex, sag) = vertex_and_sag(face);
        // Marched section curves are good to ~1e-6 of the model size, so
        // the bound is the weld scale (100 × the 1e-7 linear tolerance).
        assert!(
            vertex <= 1e-5 * scale,
            "{what}: face {i} has a vertex {vertex} off its carrier"
        );
        assert!(
            sag <= 2.0 * deflection,
            "{what}: face {i} sags {sag} from its carrier at deflection {deflection}"
        );
        assert_eq!(
            inverted_triangles(face),
            0,
            "{what}: face {i} has inverted triangles"
        );
    }
}

/// A mesh whose every point lies within `2 d` of the true boundary encloses
/// a volume within `2 d` × its area of the true one (the Hausdorff slab);
/// an inscribed mesh reads low.
fn assert_volume_within_chord_bound(
    mesh: &crate::tessellate::TriangleMesh,
    exact: f64,
    deflection: f64,
    what: &str,
) {
    let area: f64 = mesh
        .indices
        .chunks_exact(3)
        .map(|t| {
            super::mesh_oracles::triangle_area(&[0, 1, 2].map(|k| mesh.positions[t[k] as usize]))
        })
        .sum();
    let v = signed_volume(mesh);
    assert!(
        v <= exact * (1.0 + 1e-9) && exact - v <= 2.0 * deflection * area,
        "{what}: volume {v} vs exact {exact} at deflection {deflection} (slab {})",
        2.0 * deflection * area
    );
}

/// Calibrates the oracles on untouched primitives: closed forms for every
/// carrier kind, no sag or orientation findings.
#[test]
fn oracles_hold_on_primitives() {
    use std::f64::consts::PI;
    let mut topo = Topology::new();
    let cases = [
        (
            crate::primitives::make_cylinder(&mut topo, 2.0, 5.0).unwrap(),
            PI * 4.0 * 5.0,
        ),
        (
            crate::primitives::make_sphere(&mut topo, 3.0, 16).unwrap(),
            4.0 / 3.0 * PI * 27.0,
        ),
        (
            crate::primitives::make_torus(&mut topo, 10.0, 3.0, 32).unwrap(),
            2.0 * PI * PI * 10.0 * 9.0,
        ),
        (
            crate::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap(),
            24.0,
        ),
    ];
    for (solid, exact) in cases {
        for deflection in [0.05, 0.01] {
            let (mesh, faces) = closed_mesh_by_face(&topo, solid, deflection);
            assert_faces_follow_their_carriers(&faces, deflection, "primitive");
            let v = signed_volume(&mesh);
            assert!(
                v > 0.0 && v <= exact * (1.0 + 1e-9) && v >= exact * 0.95,
                "volume {v} vs {exact}"
            );
        }
    }
}

/// Exact volume of the torus (R = 10, r = 3) minus the box x ∈ [6, 14],
/// |y| ≤ 4, |z| ≤ 4. A ring of radius ρ loses the arc |u| ≤ min(asin(4/ρ),
/// acos(6/ρ)); integrating over the tube section with ρ = 10 + 3 sin θ.
fn torus_notch_exact_volume() -> f64 {
    use std::f64::consts::{FRAC_PI_2, PI};
    let removed = simpson(
        |theta: f64| {
            let rho = 3.0f64.mul_add(theta.sin(), 10.0);
            let arc = (4.0 / rho).asin().min((6.0 / rho).acos());
            36.0 * rho * arc * theta.cos().powi(2)
        },
        -FRAC_PI_2,
        FRAC_PI_2,
        200_000,
    );
    2.0 * PI * PI * 10.0 * 9.0 - removed
}

/// Exact volume of the same torus minus that box after the box is turned
/// `tilt` about the x axis. At tube point (t, φ), ρ = 10 + t cos φ and
/// z = t sin φ, the ring keeps every u except the one interval near u = 0
/// where the turned box holds it: there sin u is bounded by the box's y
/// faces, its z faces, and x ≥ 6. A 2D Simpson over the tube section.
fn tilted_torus_notch_exact_volume(tilt: f64) -> f64 {
    use std::f64::consts::{PI, TAU};
    let (st, ct) = tilt.sin_cos();
    let removed = simpson(
        |t: f64| {
            t * simpson(
                |phi: f64| {
                    let (sp, cp) = phi.sin_cos();
                    let rho = t.mul_add(cp, 10.0);
                    let z = t * sp;
                    // Box frame: y' = y cos + z sin, z' = z cos − y sin.
                    let mut lo = -1.0_f64;
                    let mut hi = 1.0_f64;
                    let y_band = [(-4.0 - z * st) / ct, (4.0 - z * st) / ct];
                    lo = lo.max(y_band[0] / rho);
                    hi = hi.min(y_band[1] / rho);
                    if st != 0.0 {
                        let a = (z * ct - 4.0) / st;
                        let b = (z * ct + 4.0) / st;
                        lo = lo.max(a.min(b) / rho);
                        hi = hi.min(a.max(b) / rho);
                    }
                    if rho <= 6.0 {
                        return 0.0;
                    }
                    let reach = (1.0 - 36.0 / (rho * rho)).sqrt();
                    lo = lo.max(-reach);
                    hi = hi.min(reach);
                    if hi <= lo {
                        return 0.0;
                    }
                    rho * (hi.asin() - lo.asin())
                },
                0.0,
                TAU,
                2_000,
            )
        },
        0.0,
        3.0,
        2_000,
    );
    2.0 * PI * PI * 10.0 * 9.0 - removed
}

fn torus_box_notch(topo: &mut Topology, tilt: f64) -> remus_topology::solid::SolidId {
    let tor = crate::primitives::make_torus(topo, 10.0, 3.0, 32).unwrap();
    let bx = crate::primitives::make_box(topo, 8.0, 8.0, 8.0).unwrap();
    crate::transform::transform_solid(topo, bx, &Mat4::translation(6.0, -4.0, -4.0)).unwrap();
    crate::transform::transform_solid(topo, bx, &Mat4::rotation_x(tilt)).unwrap();
    crate::boolean::boolean(topo, crate::boolean::BooleanOp::Cut, tor, bx).unwrap()
}

/// The torus notch band's loops are spiric sections (the box walls y = ±4
/// are not planes through the axis), so their ring angle varies along the
/// tube: the band mesher's ruled rows must follow them exactly. Turning the
/// box 10° about x also breaks the loops' symmetry in z, so the ring angle
/// has a nonzero slope across the tube seam, where rows interpolate across
/// the wrap. Beyond watertightness (the existing
/// `torus_box_notch_band_tessellates_watertight`), every vertex stays
/// outside the removed box, chords stay within the deflection, triangles
/// face outward, and the volume converges on the quadrature value from below
/// within the chord slab.
#[test]
fn torus_notch_band_follows_oblique_loops() {
    let flat = tilted_torus_notch_exact_volume(0.0);
    let reference = torus_notch_exact_volume();
    assert!(
        (flat - reference).abs() <= 1e-6 * reference,
        "{flat} vs {reference}"
    );
    for tilt in [0.0, 10f64.to_radians()] {
        let exact = tilted_torus_notch_exact_volume(tilt);
        let mut topo = Topology::new();
        let solid = torus_box_notch(&mut topo, tilt);
        let (st, ct) = tilt.sin_cos();
        let mut previous = 0.0;
        for deflection in [0.1, 0.02, 0.005] {
            let (mesh, faces) = closed_mesh_by_face(&topo, solid, deflection);
            assert_faces_follow_their_carriers(&faces, deflection, "torus notch");
            for p in &mesh.positions {
                let (y, z) = (p.y() * ct + p.z() * st, p.z() * ct - p.y() * st);
                let inside_box = p.x() > 6.0 + 1e-6 && y.abs() < 4.0 - 1e-6 && z.abs() < 4.0 - 1e-6;
                assert!(
                    !inside_box,
                    "tilt {tilt}: vertex {p:?} lies inside the removed box"
                );
            }
            assert_volume_within_chord_bound(&mesh, exact, deflection, "torus notch");
            let v = signed_volume(&mesh);
            assert!(
                v > previous,
                "volume {v} does not converge upward from {previous}"
            );
            previous = v;
        }
    }
}

/// Area of the lens where circles of radii `r1`, `r2` at distance `d` overlap.
fn lens_area(r1: f64, r2: f64, d: f64) -> f64 {
    use std::f64::consts::PI;
    if d >= r1 + r2 {
        return 0.0;
    }
    if d <= (r1 - r2).abs() {
        return PI * r1.min(r2).powi(2);
    }
    let a1 = ((d * d + r1 * r1 - r2 * r2) / (2.0 * d * r1))
        .clamp(-1.0, 1.0)
        .acos();
    let a2 = ((d * d + r2 * r2 - r1 * r1) / (2.0 * d * r2))
        .clamp(-1.0, 1.0)
        .acos();
    let k = ((-d + r1 + r2) * (d + r1 - r2) * (d - r1 + r2) * (d + r1 + r2))
        .max(0.0)
        .sqrt();
    r1 * r1 * a1 + r2 * r2 * a2 - 0.5 * k
}

/// The B37 pair: a pointed cone (base r = 2 at z = 0, apex at z = 1) and a
/// unit cylinder z ∈ [0.5, 1.5] on the axis (0, 1.5). The cylinder pierces
/// the cone's side, so the cone face keeps only its base rim as outer wire,
/// closes on the apex, and carries the footprint as an inner wire.
fn pointed_cone_and_cylinder(
    topo: &mut Topology,
    k: f64,
) -> (
    remus_topology::solid::SolidId,
    remus_topology::solid::SolidId,
) {
    let cone = crate::primitives::make_cone(topo, 2.0 * k, 0.0, k).unwrap();
    let cyl = crate::primitives::make_cylinder(topo, k, k).unwrap();
    crate::transform::transform_solid(topo, cyl, &Mat4::translation(0.0, 1.5 * k, 0.5 * k))
        .unwrap();
    (cone, cyl)
}

/// Exact area of the cone's kept lateral surface in the fuse: π·2·√5 minus
/// the footprint. On the ring of radius ρ = 2(1 − z) the cylinder covers
/// sin u > (ρ² + 1.25) / (3ρ), for ρ ∈ [0.5, 1]; dA = ρ√5 du dz, dz = dρ/2.
fn pointed_cone_kept_area() -> f64 {
    use std::f64::consts::PI;
    let root5 = 5.0f64.sqrt();
    let footprint = simpson(
        |rho: f64| {
            let s = ((rho * rho + 1.25) / (3.0 * rho)).min(1.0);
            (PI - 2.0 * s.asin()) * rho * root5 / 2.0
        },
        0.5,
        1.0,
        200_000,
    );
    PI * 2.0 * root5 - footprint
}

/// Exact fuse volume: cone + cylinder − ∫ lens(ρ(z), 1, 1.5) dz.
fn pointed_cone_fuse_volume() -> f64 {
    use std::f64::consts::PI;
    let overlap = simpson(
        |z: f64| lens_area(2.0 * (1.0 - z), 1.0, 1.5),
        0.5,
        0.75,
        200_000,
    );
    4.0 * PI / 3.0 + PI - overlap
}

/// The single-rim cone chart: its seam and apex rows are synthesized, then
/// the apex row folds onto one vertex. The kept cone face must tile its
/// exact area (no overlap past it, no gap below the chord bound), every
/// vertex on the cone, every triangle outward, and the body's mesh volume
/// must sit within the chord slab of the closed form — at 1, 10 and 1e3,
/// with the deflection scaled alike, and with one triangle count at all
/// three.
///
/// Regression (B19 tranche): the chart radius came from
/// `compute_v_param_range`, whose `(-1, 1)` fallback fires for a wire whose
/// vertices share one level, so this cone was charted at `radius_at(1)` at
/// every scale: at 10× its chords sagged 2.4× the deflection (9× at the
/// finest), and at 1000× it meshed 274,571 triangles at the coarsest
/// deflection with the volume 1 % low at the finest.
#[test]
fn pointed_cone_with_a_side_hole_tiles_its_exact_area() {
    // Small-scale curved booleans are unqualified (B34/B35/B40): 1e-3
    // refuses exact, so the sweep stays at and above unit scale. The same
    // body at the same relative deflection is the same problem in other
    // units, so the cone face's triangle count must not move with `k`.
    let mut counts: Vec<Vec<usize>> = Vec::new();
    for k in [1.0, 10.0, 1e3] {
        let mut per_deflection = Vec::new();
        let area = pointed_cone_kept_area() * k * k;
        let volume = pointed_cone_fuse_volume() * k * k * k;
        let mut topo = Topology::new();
        let (cone, cyl) = pointed_cone_and_cylinder(&mut topo, k);
        let fused =
            crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Fuse, cone, cyl).unwrap();
        for deflection in [0.02 * k, 0.005 * k, 0.001 * k] {
            let what = format!("cone fuse at scale {k}");
            let (mesh, faces) = closed_mesh_by_face(&topo, fused, deflection);
            assert_faces_follow_their_carriers_at_scale(&faces, deflection, k, &what);
            let cone_faces: Vec<_> = faces
                .iter()
                .filter(|f| matches!(f.surface, FaceSurface::Cone(_)))
                .collect();
            assert_eq!(cone_faces.len(), 1);
            per_deflection.push(cone_faces[0].tris.len());
            let meshed: f64 = cone_faces[0]
                .tris
                .iter()
                .map(super::mesh_oracles::triangle_area)
                .sum();
            // Chord bound on a developable surface: a chord sagging s below
            // a ring of radius ρ is short by s / (3ρ) of its arc, so with
            // s ≤ 2d the rings lose at most (2d/3)·∫dA/ρ = (2d/3)·2π√5·k over
            // the whole lateral cone; the hole's own chords may add back 2d
            // per unit of its perimeter (at most 2πk).
            let slack = (2.0 * deflection / 3.0 * TAU * 5.0f64.sqrt() + 2.0 * deflection * TAU) * k;
            assert!(
                (meshed - area).abs() <= slack,
                "{what}, deflection {deflection}: cone face meshes {meshed} of exact {area} (slack {slack})"
            );
            // The cylinder wall is trimmed by the marched footprint (NURBS
            // rails between railed rims), so the CDT densifies its trim band.
            // Hold it to the deflection itself, as
            // `nurbs_trimmed_cylinder_keeps_chords_near_surface` does, with
            // 10 % for the edge-midpoint samples: without the densification
            // this wall sags 1.2–1.5 × the deflection.
            for wall in faces
                .iter()
                .filter(|f| matches!(f.surface, FaceSurface::Cylinder(_)))
            {
                let (_, sag) = vertex_and_sag(wall);
                assert!(
                    sag <= 1.1 * deflection,
                    "{what}: NURBS-railed wall sags {sag} at deflection {deflection}"
                );
            }
            assert_volume_within_chord_bound(&mesh, volume, deflection, &what);
        }
        counts.push(per_deflection);
    }
    assert!(
        counts.windows(2).all(|w| w[0] == w[1]),
        "cone face triangle counts move with scale: {counts:?}"
    );
}

/// Area of a convex polygon clipped to the half-planes `a·y + b·z ≤ c`.
fn clipped_area(mut poly: Vec<(f64, f64)>, planes: &[(f64, f64, f64)]) -> f64 {
    for &(a, b, c) in planes {
        let f = |p: (f64, f64)| a * p.0 + b * p.1 - c;
        let mut out = Vec::with_capacity(poly.len() + 1);
        for i in 0..poly.len() {
            let (p, q) = (poly[i], poly[(i + 1) % poly.len()]);
            let (fp, fq) = (f(p), f(q));
            if fp <= 0.0 {
                out.push(p);
            }
            if (fp < 0.0 && fq > 0.0) || (fp > 0.0 && fq < 0.0) {
                let t = fp / (fp - fq);
                out.push((t.mul_add(q.0 - p.0, p.0), t.mul_add(q.1 - p.1, p.1)));
            }
        }
        poly = out;
        if poly.len() < 3 {
            return 0.0;
        }
    }
    let n = poly.len();
    (0..n)
        .map(|i| poly[i].0 * poly[(i + 1) % n].1 - poly[(i + 1) % n].0 * poly[i].1)
        .sum::<f64>()
        .abs()
        / 2.0
}

/// `boolean::tests::cut_cylinder_by_tilted_slab_stays_exact_and_closed`: an
/// r6 h6 cylinder cut by a 6 × 1 × 6.5 slab tilted 30° about x. The slab's
/// faces meet the wall in ellipses.
fn tilted_slab_cut(topo: &mut Topology) -> remus_topology::solid::SolidId {
    use std::f64::consts::FRAC_PI_6;
    let stock = crate::primitives::make_cylinder(topo, 6.0, 6.0).unwrap();
    let tool = crate::primitives::make_box(topo, 6.0, 1.0, 6.5).unwrap();
    let placement = Mat4::translation(-4.0, -4.0, -4.0) * Mat4::rotation_x(FRAC_PI_6);
    crate::transform::transform_solid(topo, tool, &placement).unwrap();
    crate::boolean::boolean(topo, crate::boolean::BooleanOp::Cut, stock, tool).unwrap()
}

/// Exact kept volume: 216π minus, for x ∈ [−4, 2], the slab's (y, z)
/// section (its rotated 1 × 6.5 rectangle) clipped to the caps and to
/// y ≥ −√(36 − x²).
fn tilted_slab_cut_volume() -> f64 {
    use std::f64::consts::{FRAC_PI_6, PI};
    let (s, c) = FRAC_PI_6.sin_cos();
    let corner = |y: f64, z: f64| (y * c - z * s - 4.0, y * s + z * c - 4.0);
    let rect = vec![
        corner(0.0, 0.0),
        corner(1.0, 0.0),
        corner(1.0, 6.5),
        corner(0.0, 6.5),
    ];
    let removed = simpson(
        |x: f64| {
            let w = (36.0 - x * x).sqrt();
            clipped_area(
                rect.clone(),
                &[(0.0, -1.0, 0.0), (0.0, 1.0, 6.0), (-1.0, 0.0, w)],
            )
        },
        -4.0,
        2.0,
        200_000,
    );
    216.0 * PI - removed
}

/// The ellipse-trimmed wall takes the CDT path's curved-trim densification:
/// a two-row grid would bridge the trim valley with chords through the
/// solid. Chords within the deflection, vertices on the carriers, outward
/// triangles, and the exact volume within the chord slab.
///
/// Regression (B19 tranche): at 0.005, 0.003 and 0.002 the densification
/// exceeded its polygon budget and was dropped whole, so the wall kept one
/// mid-height row and meshed chords 1.6 off the cylinder; the closed,
/// manifold mesh read 643.5 against the exact 676.6. At 0.01 the budget
/// held and at 0.001 the trim run was long enough to grow rows on its own.
#[test]
fn ellipse_trimmed_cylinder_wall_stays_within_the_chord_bound() {
    let exact = tilted_slab_cut_volume();
    let mut topo = Topology::new();
    let solid = tilted_slab_cut(&mut topo);
    for deflection in [0.05, 0.01, 0.005, 0.003, 0.001] {
        let (mesh, faces) = closed_mesh_by_face(&topo, solid, deflection);
        assert!(
            faces
                .iter()
                .any(|f| matches!(f.surface, FaceSurface::Cylinder(_)))
        );
        assert_faces_follow_their_carriers(&faces, deflection, "tilted slab");
        assert_volume_within_chord_bound(&mesh, exact, deflection, "tilted slab");
    }
}

/// Filleting a cylinder's top rim leaves a torus band bounded by two
/// once-used circles and seamed by its profile arc. Its rims are densified
/// to the band mesher's own wrap density; without that the pool's sparser
/// rims stitch against denser interior rows: the ρ = 1.5 band sags past the
/// deflection and the B46-sized ρ = 0.05 band meshes open at 0.1 and 0.05.
/// So: a closed mesh, the structured band within the deflection itself (its
/// contract; it holds ≤ 0.25 ×), every other analytic face within the
/// chord bound, and the volume on Pappus — the removed ring's section is a
/// square minus a quarter disc, (1 − π/4)ρ², whose centroid sits
/// ρ(10 − 3π)/(12 − 3π) in from the corner.
#[test]
fn filleted_cap_rim_band_stays_within_the_chord_bound() {
    use std::f64::consts::PI;
    let (r, h) = (5.0, 8.0);
    for (rho, deflections) in [
        (1.5, &[0.1, 0.05, 0.02, 0.004][..]),
        (0.05, &[0.1, 0.05, 0.01][..]),
    ] {
        let mut topo = Topology::new();
        let cyl = crate::primitives::make_cylinder(&mut topo, r, h).unwrap();
        let rim = remus_topology::explorer::solid_edges(&topo, cyl)
            .unwrap()
            .into_iter()
            .find(|&e| {
                let edge = topo.edge(e).unwrap();
                edge.start() == edge.end()
                    && matches!(edge.curve(), remus_topology::edge::EdgeCurve::Circle(_))
                    && (topo.vertex(edge.start()).unwrap().point().z() - h).abs() < 1e-9
            })
            .unwrap();
        let solid = crate::blend_ops::fillet_v2(&mut topo, cyl, &[rim], rho)
            .unwrap()
            .solid;
        let centroid = rho * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
        let exact = PI * r * r * h - (1.0 - PI / 4.0) * rho * rho * 2.0 * PI * (r - centroid);
        for &deflection in deflections {
            let what = format!("rim fillet ρ = {rho}");
            let (mesh, faces) = closed_mesh_by_face(&topo, solid, deflection);
            let bands: Vec<_> = faces
                .iter()
                .filter(|f| matches!(f.surface, FaceSurface::Torus(_)))
                .collect();
            assert_eq!(bands.len(), 1, "{what}");
            let (_, sag) = vertex_and_sag(bands[0]);
            assert!(
                sag <= deflection,
                "{what}: torus band sags {sag} at deflection {deflection}"
            );
            assert_faces_follow_their_carriers(&faces, deflection, &what);
            assert_volume_within_chord_bound(&mesh, exact, deflection, &what);
        }
    }
}

/// `regress_stepped_rim_cylinder_cdt`'s body: a 30 × 18 × 24 box fused with
/// an r6 h28 cylinder at the origin. The cylinder wall's outer wire steps at
/// three levels (z = 0, 24, 28), so it takes the stepped-rim interior rows;
/// the exact volume is 30·18·24 + 792π (the full cylinder less the quarter
/// of height 24 the box swallows).
#[test]
fn stepped_rim_wall_stays_within_the_chord_bound() {
    use std::f64::consts::PI;
    let mut topo = Topology::new();
    let bx = crate::primitives::make_box(&mut topo, 30.0, 18.0, 24.0).unwrap();
    let cyl = crate::primitives::make_cylinder(&mut topo, 6.0, 28.0).unwrap();
    let solid =
        crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Fuse, bx, cyl).unwrap();
    let exact = 30.0 * 18.0 * 24.0 + 792.0 * PI;
    for deflection in [0.05, 0.01, 0.003] {
        let (mesh, faces) = closed_mesh_by_face(&topo, solid, deflection);
        assert_faces_follow_their_carriers(&faces, deflection, "stepped rim");
        assert_volume_within_chord_bound(&mesh, exact, deflection, "stepped rim");
    }
}

/// Ready-repro (B68): with the densification capped to its budget, the
/// tilted-slab wall still sags 2.2 × the deflection at 0.002 and 4.8 × at
/// 0.0015, next to the slot's straight end generator (x = 2), where the
/// capped rows leave the grid columns between the line and the trim unused
/// and the CDT fans the line's endpoints to a column ~0.5 away. Volume is
/// already within 1e-4 there. Exit: this passes.
#[test]
#[ignore = "open: B68 — budget-capped trim rows still sag past 2× deflection beside a line trim"]
fn ellipse_trimmed_wall_stays_within_the_chord_bound_at_capped_rows() {
    let exact = tilted_slab_cut_volume();
    let mut topo = Topology::new();
    let solid = tilted_slab_cut(&mut topo);
    for deflection in [0.002, 0.0015] {
        let (mesh, faces) = closed_mesh_by_face(&topo, solid, deflection);
        assert_faces_follow_their_carriers(&faces, deflection, "tilted slab");
        assert_volume_within_chord_bound(&mesh, exact, deflection, "tilted slab");
    }
}

/// Ready-repro (B69): the torus notch with the box turned 25° or 30° about x
/// (its corners now cut the tube too) is an exact, valid B-Rep whose mesh
/// is closed at 0.1 and 0.02 but opens at 0.005 (5 and 15 boundary edges),
/// while its volume converges on the quadrature value. Exit: this passes.
#[test]
#[ignore = "open: B69 — steeply turned torus notch meshes open at fine deflection"]
fn steep_torus_notch_meshes_closed_at_fine_deflection() {
    for degrees in [25.0_f64, 30.0] {
        let tilt = degrees.to_radians();
        let exact = tilted_torus_notch_exact_volume(tilt);
        let mut topo = Topology::new();
        let solid = torus_box_notch(&mut topo, tilt);
        for deflection in [0.02, 0.01, 0.005] {
            let (mesh, faces) = closed_mesh_by_face(&topo, solid, deflection);
            assert_faces_follow_their_carriers(&faces, deflection, "steep torus notch");
            assert_volume_within_chord_bound(&mesh, exact, deflection, "steep torus notch");
        }
    }
}

/// Ready-repro (B70): the cross-drilled shaft of
/// `cross_drilled_display_mesh_is_closed_and_matches_brep_volume` (shaft
/// r = 3, h = 30, bored along x at mid-height). The bore wall is bounded by
/// its two marched saddle curves, which touch the wall's v extremes at only
/// four points, so `tessellate_nonplanar_cdt` withholds the NURBS-rail
/// densification from it. At b = 2 the wall then sags 16 × the deflection
/// at 0.05 and 397 × at 0.002, and the closed mesh reads up to 0.87 % over
/// the exact volume (inside that test's 2 % band); at b = 1, 6.5 × at
/// 0.002. With the densification the same walls hold ≤ 0.63 × and the volume
/// converges from below. The exact volume: 270π minus the bore's
/// intersection with the shaft, ∫ 4·√(9 − y²)·√(b² − y²) dy over |y| ≤ b.
/// Exit: this passes.
#[test]
#[ignore = "open: B70 — a cross-drilled bore wall is meshed without its trim densification"]
fn cross_drilled_bore_wall_stays_within_the_chord_bound() {
    use std::f64::consts::PI;
    for b in [2.0_f64, 1.0] {
        let (topo, solid) = super::shaft_drilled_with(b);
        let removed = simpson(
            |y: f64| 4.0 * (9.0 - y * y).max(0.0).sqrt() * (b * b - y * y).max(0.0).sqrt(),
            -b,
            b,
            200_000,
        );
        let exact = PI * 9.0 * 30.0 - removed;
        for deflection in [0.05, 0.02, 0.006, 0.002] {
            let what = format!("bore r = {b}");
            let (mesh, faces) = closed_mesh_by_face(&topo, solid, deflection);
            // Chords only: the marched saddle curves also leave rim vertices
            // up to 1.4e-4 off the bore (b = 2), a separate observation.
            for (i, face) in faces.iter().enumerate() {
                if matches!(face.surface, FaceSurface::Cylinder(_)) {
                    let (_, sag) = vertex_and_sag(face);
                    assert!(
                        sag <= 2.0 * deflection,
                        "{what}: face {i} sags {sag} at deflection {deflection}"
                    );
                }
            }
            assert_volume_within_chord_bound(&mesh, exact, deflection, &what);
        }
    }
}
