//! A primitive sphere against a box whose walls cross the upper hemisphere
//! and whose lid sits below the pole: the upper hemisphere splits into a
//! collar (inside the walls, holed by the lid's latitude circle), the lid's
//! cap and four wall caps — the face splitter's box–sphere collar route
//! (B19 face-splitter tranche).
//!
//! Oracles: exact success, validity, the analytic face census, ray-cast
//! probes, and the volume against an independent numerical integral of the
//! sphere ∩ box column heights (cut checked by `sphere − intersect`).
//!
//! Closed defects:
//! - B64: the cut dropped the lid-cap lump (the sphere above the lid), so it
//!   returned only the four wall caps — a silent wrong volume on `main`.
//!   Root: `remove_doubled_faces` dropped the plane-disc + sphere-cap lens
//!   (one shared circle, different carriers) as coincident copies.
//! - B65: with the walls close under the lid (`a = 0.75·r`) the collar's
//!   classification sample, the lid latitude nudged a fixed amount toward
//!   the equator, landed beyond a wall, and the exact boolean was refused.
//!   The nudge now stops halfway to the first wall arc on its meridian, with
//!   demonstrated containment (|x| < a, |y| < a, strictly between lid and
//!   wall), so the sample stays on the collar.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_sphere};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// Sphere of radius `r` at the origin and the box `|x|, |y| < a`,
/// `-r - 5 < z < c`.
fn operands(r: f64, a: f64, c: f64) -> (Topology, SolidId, SolidId) {
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, r, 64).unwrap();
    let b = make_box(&mut topo, 2.0 * a, 2.0 * a, r + 5.0 + c).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        b,
        &Mat4::translation(-a, -a, -r - 5.0),
    )
    .unwrap();
    (topo, sphere, b)
}

/// `∬_{|x|,|y|<a, x²+y²<r²} (min(c, √(r²−x²−y²)) + √(r²−x²−y²)) dx dy` by the
/// midpoint rule: the volume of sphere ∩ box, independent of the kernel.
fn reference_volume(r: f64, a: f64, c: f64) -> f64 {
    let n = 1200;
    let h = 2.0 * a / f64::from(n);
    let mut sum = 0.0;
    for i in 0..n {
        let x = h.mul_add(f64::from(i) + 0.5, -a);
        for j in 0..n {
            let y = h.mul_add(f64::from(j) + 0.5, -a);
            let q = r.mul_add(r, -x.mul_add(x, y * y));
            if q > 0.0 {
                let t = q.sqrt();
                sum += t.min(c) + t;
            }
        }
    }
    sum * h * h
}

/// Box half-width `frac·r`, lid halfway between the walls' arc apex and the
/// pole (so its latitude circle, of radius < a, stays inside the walls).
fn lidded(r: f64, frac: f64) -> (f64, f64, f64) {
    let a = frac * r;
    let rho = a.mul_add(-a, r * r).sqrt();
    (a, rho, 0.5 * (rho + r))
}

/// Box half-width `frac·r`, lid at `t` of the way from the walls' arc apex
/// (`t = 0`) to the pole (`t = 1`).
fn lidded_at(r: f64, frac: f64, t: f64) -> (f64, f64, f64) {
    let a = frac * r;
    let rho = a.mul_add(-a, r * r).sqrt();
    (a, rho, rho + t * (r - rho))
}

fn check(r: f64, frac: f64, op: BooleanOp) {
    let (a, rho, c) = lidded(r, frac);
    check_at(r, a, rho, c, op);
}

fn check_at(r: f64, a: f64, rho: f64, c: f64, op: BooleanOp) {
    let ctx = format!("r={r} a={a} c={c} {op:?}");
    let want = reference_volume(r, a, c);
    let (mut topo, s, b) = operands(r, a, c);
    let result = boolean(&mut topo, op, s, b)
        .unwrap_or_else(|e| panic!("{ctx}: exact boolean refused: {e:?}"));
    assert!(
        remus_operations::validate::validate_solid(&topo, result)
            .unwrap()
            .is_valid(),
        "{ctx}: invalid result"
    );
    let faces = remus_topology::explorer::solid_faces(&topo, result).unwrap();
    let spheres = faces
        .iter()
        .filter(|&&f| topo.face(f).unwrap().surface().type_tag() == "sphere")
        .count();
    let planes = faces
        .iter()
        .filter(|&&f| topo.face(f).unwrap().surface().type_tag() == "plane")
        .count();
    // Exact carrier preservation: only sphere + plane faces, no NURBS/mesh.
    assert_eq!(
        spheres + planes,
        faces.len(),
        "{ctx}: non-analytic faces in result"
    );
    // Intersect: collar + lower hemisphere | 4 walls + lid. Cut: each wall
    // cap in two hemisphere halves + the lid cap | 4 walls + lid.
    let want_counts = match op {
        BooleanOp::Intersect => (2, 5),
        _ => (9, 5),
    };
    assert_eq!((spheres, planes), want_counts, "{ctx}: face census");
    // Disconnected outer components (cut) vs single shell (intersect).
    let solid = topo.solid(result).unwrap();
    assert!(
        solid.inner_shells().is_empty(),
        "{ctx}: unexpected cavity shells"
    );
    let outer = topo.shell(solid.outer_shell()).unwrap();
    assert_eq!(
        outer.faces().len(),
        faces.len(),
        "{ctx}: outer shell must hold every component (compatibility fold)"
    );
    let opts = ClassifyOptions::default();
    let in_box = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 0.95 * c),
        Point3::new(0.0, 0.9 * a, 0.5 * rho),
        Point3::new(0.6 * a, 0.6 * a, 0.3 * rho),
    ];
    let out_of_box = [
        Point3::new(0.0, 0.0, 0.5 * (c + r)),
        Point3::new(0.5 * (a + r), 0.0, 0.2 * rho),
    ];
    for (points, is_in_box) in [(&in_box[..], true), (&out_of_box[..], false)] {
        for &p in points {
            let inside = match op {
                BooleanOp::Intersect => is_in_box,
                _ => !is_in_box,
            };
            let want_class = if inside {
                PointClassification::Inside
            } else {
                PointClassification::Outside
            };
            assert_eq!(
                classify_point(&topo, result, p, &opts).unwrap(),
                want_class,
                "{ctx}: {p:?}"
            );
        }
    }
    // Watertight + manifold tessellation at shipping deflection.
    let mesh = tessellate_solid(&topo, result, 1e-3 * r).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0, "{ctx}: open mesh");
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "{ctx}: non-manifold mesh"
    );
    let got = remus_operations::measure::solid_volume(&topo, result, 1e-3 * r).unwrap();
    let want_op = match op {
        BooleanOp::Intersect => want,
        _ => (4.0 / 3.0 * std::f64::consts::PI).mul_add(r.powi(3), -want),
    };
    // 1e-3 relative catches a missing lump (lid cap alone is 65% of the cut
    // at the B64 width) while allowing tessellation chord error on small caps.
    assert!(
        (got - want_op).abs() < 1e-3 * want_op,
        "{ctx}: volume {got}, want {want_op}"
    );
}

/// Walls far enough out that the collar sample stays inside them: the exact
/// intersect keeps the holed collar and the lower hemisphere.
#[test]
fn lidded_box_intersects_a_sphere_exactly() {
    for r in [1.0, 50.0] {
        check(r, 0.85, BooleanOp::Intersect);
    }
}

/// B64: the cut keeps every material component, including the lid-cap lump
/// (sphere cap above the lid closed by the lid disc).
#[test]
fn b64_lidded_box_cut_keeps_the_lid_cap_lump() {
    for r in [1.0, 50.0] {
        check(r, 0.9, BooleanOp::Cut);
    }
}

/// B65: with the walls close under the lid the collar sample stays inside
/// the walls (bounded nudge), so the exact intersect succeeds.
#[test]
fn b65_lidded_box_with_walls_close_under_the_lid_intersects_exactly() {
    for r in [1.0, 50.0] {
        check(r, 0.75, BooleanOp::Intersect);
    }
}

/// Supported transitions: near-wall widths, lid heights from the wall apex
/// toward the pole, and the fuse leg. Each component is probed separately
/// (lid cap vs wall caps) as well as by aggregate volume, so a missing lump
/// cannot hide behind a duplicated one.
#[test]
fn lidded_box_family_supported_transitions() {
    // Near-wall width (walls near the silhouette).
    check(1.0, 0.95, BooleanOp::Cut);
    // Lid heights across the qualified band at the B64 width.
    for t in [0.25, 0.75] {
        let (a, rho, c) = lidded_at(1.0, 0.9, t);
        check_at(1.0, a, rho, c, BooleanOp::Cut);
    }
    // Fuse leg on the B64 operands (union closes inclusion–exclusion).
    {
        let (a, _, c) = lidded(1.0, 0.9);
        let ctx = "r=1 a=0.9 Fuse";
        let (mut topo, s, b) = operands(1.0, a, c);
        let result = boolean(&mut topo, BooleanOp::Fuse, s, b)
            .unwrap_or_else(|e| panic!("{ctx}: exact boolean refused: {e:?}"));
        assert!(
            remus_operations::validate::validate_solid(&topo, result)
                .unwrap()
                .is_valid(),
            "{ctx}: invalid"
        );
        let faces = remus_topology::explorer::solid_faces(&topo, result).unwrap();
        let spheres = faces
            .iter()
            .filter(|&&f| topo.face(f).unwrap().surface().type_tag() == "sphere")
            .count();
        assert_eq!((spheres, faces.len() - spheres), (9, 6), "{ctx}: census");
        let want_fuse = 4.0 / 3.0 * std::f64::consts::PI + (2.0 * a).powi(2) * (1.0 + 5.0 + c)
            - reference_volume(1.0, a, c);
        let got = remus_operations::measure::solid_volume(&topo, result, 1e-3).unwrap();
        assert!(
            (got - want_fuse).abs() < 5e-4 * want_fuse,
            "{ctx}: volume {got}, want {want_fuse}"
        );
    }
    // Modest rigid placement is invariant (small offset + rotation-free move).
    {
        let (a, _rho, c) = lidded(1.0, 0.9);
        let (mut topo, s, b) = operands(1.0, a, c);
        let mat = Mat4::translation(0.37, -0.21, 0.13);
        remus_operations::transform::transform_solid(&mut topo, s, &mat).unwrap();
        remus_operations::transform::transform_solid(&mut topo, b, &mat).unwrap();
        let result = boolean(&mut topo, BooleanOp::Cut, s, b).expect("placed cut exact");
        let faces = remus_topology::explorer::solid_faces(&topo, result).unwrap();
        assert_eq!(faces.len(), 14, "placed cut: face count");
    }
}

/// Neighboring disjoint/contained identities (fast paths, no collar).
#[test]
fn lidded_box_neighboring_identities() {
    // Disjoint: sphere untouched by a far box.
    {
        let mut topo = Topology::new();
        let sphere = make_sphere(&mut topo, 1.0, 64).unwrap();
        let b = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        remus_operations::transform::transform_solid(
            &mut topo,
            b,
            &Mat4::translation(10.0, 0.0, 0.0),
        )
        .unwrap();
        let cut = boolean(&mut topo, BooleanOp::Cut, sphere, b).expect("disjoint cut");
        assert_eq!(
            remus_topology::explorer::solid_faces(&topo, cut)
                .unwrap()
                .len(),
            2,
            "disjoint cut is the sphere"
        );
        let fuse = boolean(&mut topo, BooleanOp::Fuse, sphere, b).expect("disjoint fuse");
        assert_eq!(
            remus_topology::explorer::solid_faces(&topo, fuse)
                .unwrap()
                .len(),
            8,
            "disjoint fuse keeps both bodies"
        );
    }
    // Contained: small box inside the sphere fuses to the sphere.
    {
        let mut topo = Topology::new();
        let sphere = make_sphere(&mut topo, 1.0, 64).unwrap();
        let b = make_box(&mut topo, 0.5, 0.5, 0.5).unwrap();
        remus_operations::transform::transform_solid(
            &mut topo,
            b,
            &Mat4::translation(-0.25, -0.25, -0.25),
        )
        .unwrap();
        let intersect =
            boolean(&mut topo, BooleanOp::Intersect, sphere, b).expect("contained intersect");
        assert_eq!(
            remus_topology::explorer::solid_faces(&topo, intersect)
                .unwrap()
                .len(),
            6,
            "contained intersect is the box"
        );
        let fuse = boolean(&mut topo, BooleanOp::Fuse, sphere, b).expect("contained fuse");
        assert_eq!(
            remus_topology::explorer::solid_faces(&topo, fuse)
                .unwrap()
                .len(),
            2,
            "contained fuse is the sphere"
        );
    }
}

/// Unsupported/degenerate configurations refuse atomically (typed
/// `ExactOnlyUnattainable`, no partial solid, topology rolled back).
#[test]
fn lidded_box_unsupported_refuses_atomically() {
    // Narrow column below the qualified band (raw GFA fragments; the gate
    // refuses rather than returning a partial solid).
    for (r, frac) in [(1.0, 0.6)] {
        let (a, _, c) = lidded(r, frac);
        for op in [BooleanOp::Cut, BooleanOp::Intersect] {
            let (mut topo, s, b) = operands(r, a, c);
            let faces_before = remus_topology::explorer::solid_faces(&topo, s)
                .unwrap()
                .len()
                + remus_topology::explorer::solid_faces(&topo, b)
                    .unwrap()
                    .len();
            let err = boolean(&mut topo, op, s, b).expect_err("narrow column must refuse");
            assert!(
                matches!(
                    err,
                    remus_operations::OperationsError::ExactOnlyUnattainable
                ),
                "narrow r={r} frac={frac} {op:?}: typed refusal, got {err:?}"
            );
            // Rollback: operands intact.
            assert_eq!(
                remus_topology::explorer::solid_faces(&topo, s)
                    .unwrap()
                    .len()
                    + remus_topology::explorer::solid_faces(&topo, b)
                        .unwrap()
                        .len(),
                faces_before,
                "refusal must roll back"
            );
        }
    }
}
