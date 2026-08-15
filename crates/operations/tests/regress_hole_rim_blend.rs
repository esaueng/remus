//! Regression: fillets and chamfers on a closed hole rim.
//!
//! The convex standalone-cylinder rim (a primitive cylinder's end cap, bounded
//! BY the cylinder) has worked for a while. A drilled hole's rim differs in two
//! ways: the plane-side loop is an INNER wire of the cap, and the setback runs
//! the other way — it GROWS that inner circle to `r_hole + r_fillet` instead of
//! shrinking an outer boundary.
//!
//! The analytic path used to read the bore's `reversed` flag alone as "concave"
//! and round the rim inward at `r_hole − r_fillet`, describing geometry that is
//! not there; the rim assembler then declined the cap because the rim was not
//! its outer wire, and the trim path failed. Every hole-rim fillet on the
//! reported plate came back `TrimmingFailure`.
//!
//! What must NOT happen is a rim blend that reports success while running off
//! the geometry. Both failure modes here — a setback that reaches another
//! boundary of the cap, and one deeper than the bore is long — leave a closed
//! shell with no free edges and a watertight tessellation. Only measuring the
//! cap's other loops and the wall's own extent catches them, so both are
//! measured, and both refuse by radius.
//!
//! Note on validation: `validate_solid` is deliberately NOT asserted here. The
//! boolean's own output already fails its `ShellOrientationConsistent` check on
//! every drilled plate — both hole rims are traversed the same way by the cap
//! and the bore — so the fixture is red before any blend runs. That is a
//! boolean-side defect, unrelated to these blends; the checks below are the
//! ones the input actually passes.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;
use std::f64::consts::PI;

use brepkit_math::mat::Mat4;
use brepkit_operations::blend_ops;
use brepkit_operations::boolean::{BooleanOp, boolean};
use brepkit_operations::measure;
use brepkit_operations::primitives::{make_box, make_cylinder};
use brepkit_operations::tessellate::tessellate_solid_with_tolerance;
use brepkit_operations::transform::transform_solid;
use brepkit_topology::Topology;
use brepkit_topology::edge::{EdgeCurve, EdgeId};
use brepkit_topology::explorer::solid_faces;
use brepkit_topology::face::FaceSurface;
use brepkit_topology::solid::SolidId;
use brepkit_topology::validation::validate_shell_closed;

const W: f64 = 80.0;
const D: f64 = 60.0;
const T: f64 = 6.0;
const HOLE_R: f64 = 2.25;

/// Deflection for the volume comparisons.
///
/// The residual against the closed form is pure quadrature and converges
/// linearly: filleting this hole at R0.5 lands 2.57 % low at deflection 0.005,
/// 0.53 % at 0.001 and 0.10 % at 0.0002 — a clean 5x per 5x. `VOLUME_REL_BAND`
/// is set just above the worst measured value at this deflection, so a fillet
/// built on the wrong side of the rim (which misses by tens of percent) cannot
/// hide inside it.
const VOLUME_DEFLECTION: f64 = 0.0002;
const VOLUME_REL_BAND: f64 = 2.5e-3;

/// A plate with one hole drilled straight through the middle.
fn drilled(topo: &mut Topology, plate_thickness: f64, hole_r: f64) -> SolidId {
    let body = make_box(topo, W, D, plate_thickness).expect("plate blank");
    let drill = make_cylinder(topo, hole_r, plate_thickness + 4.0).expect("drill");
    transform_solid(topo, drill, &Mat4::translation(W / 2.0, D / 2.0, -2.0)).expect("place drill");
    boolean(topo, BooleanOp::Cut, body, drill).expect("drill hole")
}

/// The closed circular edges whose centre sits at height `z`.
fn rims_at(topo: &Topology, s: SolidId, z: f64) -> Vec<EdgeId> {
    let mut out = Vec::new();
    for fid in solid_faces(topo, s).unwrap() {
        let f = topo.face(fid).unwrap();
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                let e = topo.edge(oe.edge()).unwrap();
                if e.start() != e.end() {
                    continue;
                }
                if let EdgeCurve::Circle(c) = e.curve()
                    && (c.center().z() - z).abs() < 1e-9
                    && !out.contains(&oe.edge())
                {
                    out.push(oe.edge());
                }
            }
        }
    }
    out
}

/// Guard the premise: the rim really is an inner loop of a planar cap, which is
/// what distinguishes this from the standalone-cylinder rim that already works.
fn assert_rim_is_an_inner_loop(topo: &Topology, s: SolidId, rim: EdgeId) {
    let holds = solid_faces(topo, s).unwrap().into_iter().any(|fid| {
        let f = topo.face(fid).unwrap();
        matches!(f.surface(), FaceSurface::Plane { .. })
            && f.inner_wires().iter().any(|&wid| {
                topo.wire(wid)
                    .unwrap()
                    .edges()
                    .iter()
                    .any(|oe| oe.edge() == rim)
            })
    });
    assert!(
        holds,
        "the fixture must present the rim as an inner wire of a planar cap"
    );
}

fn assert_watertight(topo: &Topology, s: SolidId, what: &str) {
    let shell_id = topo.solid(s).unwrap().outer_shell();
    let shell = topo.shell(shell_id).unwrap().clone();
    validate_shell_closed(&shell, topo)
        .unwrap_or_else(|e| panic!("{what}: the result shell must be closed, got {e}"));

    let mut usage: HashMap<usize, usize> = HashMap::new();
    for fid in solid_faces(topo, s).unwrap() {
        let f = topo.face(fid).unwrap();
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                *usage.entry(oe.edge().index()).or_insert(0) += 1;
            }
        }
    }
    assert_eq!(
        (
            usage.values().filter(|&&c| c == 1).count(),
            usage.values().filter(|&&c| c >= 3).count()
        ),
        (0, 0),
        "{what}: no free or non-manifold B-rep edges"
    );

    let mesh = tessellate_solid_with_tolerance(topo, s, 0.01, 0.1).unwrap();
    let mut canon: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut remap = vec![0u32; mesh.positions.len()];
    for (i, p) in mesh.positions.iter().enumerate() {
        #[allow(clippy::cast_possible_truncation)]
        let key = (
            (p.x() * 1e6).round() as i64,
            (p.y() * 1e6).round() as i64,
            (p.z() * 1e6).round() as i64,
        );
        #[allow(clippy::cast_possible_truncation)]
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
    assert_eq!(
        (
            edges.values().filter(|&&c| c == 1).count(),
            edges.values().filter(|&&c| c >= 3).count()
        ),
        (0, 0),
        "{what}: the tessellation must be watertight"
    );
}

fn count_surface(topo: &Topology, s: SolidId, tag: &str) -> usize {
    solid_faces(topo, s)
        .unwrap()
        .into_iter()
        .filter(|&f| topo.face(f).unwrap().surface().type_tag() == tag)
        .count()
}

/// Material a hole-rim fillet removes, in closed form.
///
/// In the axial section the removed area is the corner square `r x r` less the
/// quarter disc the rolling ball leaves behind; Pappus turns that into a volume
/// about the bore axis. The square contributes `∫ρ dA = a r² + r³/2` and the
/// quarter disc `(a + r)πr²/4 − r³/3`.
fn analytic_fillet_removal(a: f64, r: f64) -> f64 {
    2.0 * PI * (a * r * r + 5.0 * r * r * r / 6.0 - PI * r * r * (a + r) / 4.0)
}

/// Material an equal-setback rim chamfer removes: a right triangle with legs
/// `d`, centroid at radius `a + d/3`, revolved.
fn analytic_chamfer_removal(a: f64, d: f64) -> f64 {
    2.0 * PI * (a + d / 3.0) * d * d / 2.0
}

fn assert_removal(got: f64, want: f64, what: &str) {
    assert!(
        (got - want).abs() < want.abs() * VOLUME_REL_BAND,
        "{what}: removed {got}, closed form says {want} ({:+.4} %)",
        (got - want) / want * 100.0
    );
}

/// A plate with a cylindrical post fused onto its top face.
fn post_on_plate(topo: &mut Topology) -> SolidId {
    let plate = make_box(topo, 80.0, 40.0, 8.0).expect("plate");
    let post = make_cylinder(topo, 10.0, 32.0).expect("post");
    transform_solid(topo, post, &Mat4::translation(40.0, 20.0, 8.0)).expect("place post");
    boolean(topo, BooleanOp::Fuse, plate, post).expect("fuse post")
}

/// Material a concave post-base fillet adds: the corner square less its
/// quarter-disc, revolved about the post axis by Pappus.
fn analytic_post_base_fill(post_r: f64, r: f64) -> f64 {
    let square_moment = r * r * (post_r + r / 2.0);
    let quarter_moment = PI * r * r / 4.0 * (post_r + r - 4.0 * r / (3.0 * PI));
    2.0 * PI * (square_moment - quarter_moment)
}

/// The headline case: the top rim of a drilled hole rounds at each radius, and
/// removes exactly the material the closed form says it should.
#[test]
fn hole_rim_fillets_at_each_radius() {
    let mut topo = Topology::new();
    let body = drilled(&mut topo, T, HOLE_R);
    let before = measure::solid_volume(&topo, body, VOLUME_DEFLECTION).unwrap();

    for r in [0.5_f64, 1.0, 2.0] {
        let mut t = topo.clone();
        let rim = rims_at(&t, body, T);
        assert_eq!(rim.len(), 1, "one top rim");
        assert_rim_is_an_inner_loop(&t, body, rim[0]);

        let result = blend_ops::fillet_v2(&mut t, body, &rim, r)
            .unwrap_or_else(|e| panic!("hole rim at r={r}: {e}"));
        assert!(result.failed.is_empty(), "{:?}", result.failed);
        assert_watertight(&t, result.solid, &format!("hole rim r={r}"));

        // The exact analytic assembler ran, not some approximating fallback.
        assert_eq!(
            count_surface(&t, result.solid, "torus"),
            1,
            "r={r}: the rim blend must be one exact torus band"
        );
        // The bore survives as a cylinder, now shorter.
        assert_eq!(count_surface(&t, result.solid, "cylinder"), 1, "r={r}");

        let after = measure::solid_volume(&t, result.solid, VOLUME_DEFLECTION).unwrap();
        assert_removal(
            before - after,
            analytic_fillet_removal(HOLE_R, r),
            &format!("hole rim r={r}"),
        );
    }
}

/// A post standing on a plate is the remaining plane-cylinder rim case: the
/// band fills the 270-degree re-entrant corner and its outward normal points
/// INTO the torus tube. The rim assembler used to apply the convex-rim sign,
/// producing a closed shell whose inside-out band made the volume gate refuse.
#[test]
fn post_base_fillet_adds_exact_material_and_reverses_band() {
    let mut topo = Topology::new();
    let body = post_on_plate(&mut topo);
    let before = measure::solid_volume(&topo, body, 0.05).unwrap();
    let rim = rims_at(&topo, body, 8.0);
    assert_eq!(rim.len(), 1, "one post-base rim");

    for r in [1.0_f64, 2.0, 5.0] {
        let mut t = topo.clone();
        let result = blend_ops::fillet_v2(&mut t, body, &rim, r)
            .unwrap_or_else(|e| panic!("post-base fillet r={r}: {e}"));
        assert!(result.failed.is_empty(), "{:?}", result.failed);
        assert_watertight(&t, result.solid, &format!("post-base fillet r={r}"));

        let bands: Vec<_> = solid_faces(&t, result.solid)
            .unwrap()
            .into_iter()
            .filter(|&face| matches!(t.face(face).unwrap().surface(), FaceSurface::Torus(_)))
            .collect();
        assert_eq!(bands.len(), 1, "r={r}: one exact torus band");
        let band_face = t.face(bands[0]).unwrap();
        let FaceSurface::Torus(band) = band_face.surface() else {
            unreachable!()
        };
        assert!(
            band_face.is_reversed(),
            "r={r}: a concave rim band must be reversed, not inside-out"
        );
        assert!((band.minor_radius() - r).abs() < 1e-9);
        assert!((band.major_radius() - (10.0 + r)).abs() < 1e-9);
        assert!((band.center().z() - (8.0 + r)).abs() < 1e-9);

        let after = measure::solid_volume(&t, result.solid, 0.05).unwrap();
        let added = after - before;
        let expected = analytic_post_base_fill(10.0, r);
        assert!(
            (added - expected).abs() < 0.5,
            "r={r}: added {added}, closed form says {expected}"
        );
    }
}

/// Both rims of the same bore in one operation: the wall is shortened from each
/// end, and the two bands are independent.
#[test]
fn both_rims_of_one_bore_fillet_together() {
    let mut topo = Topology::new();
    let body = drilled(&mut topo, T, HOLE_R);
    let before = measure::solid_volume(&topo, body, VOLUME_DEFLECTION).unwrap();

    let r = 1.0;
    let mut rim = rims_at(&topo, body, T);
    rim.extend(rims_at(&topo, body, 0.0));
    assert_eq!(rim.len(), 2, "a through hole has two rims");

    let result = blend_ops::fillet_v2(&mut topo, body, &rim, r).expect("both rims");
    assert!(result.failed.is_empty(), "{:?}", result.failed);
    assert_watertight(&topo, result.solid, "both rims");
    assert_eq!(count_surface(&topo, result.solid, "torus"), 2);

    let after = measure::solid_volume(&topo, result.solid, VOLUME_DEFLECTION).unwrap();
    assert_removal(
        before - after,
        2.0 * analytic_fillet_removal(HOLE_R, r),
        "both rims",
    );
}

/// A rim fillet deeper than the bore is long must be refused by radius.
///
/// Every topological check passes for such a result — the shell closes, no edge
/// is free, the mesh is watertight — because the contact circle simply hangs
/// below the plate. Only the wall's own axial extent tells the truth, and the
/// maximum it reports is exactly the plate thickness.
#[test]
fn rim_fillet_deeper_than_the_bore_is_refused_by_radius() {
    let mut topo = Topology::new();
    // A wide hole so the plate boundary is nowhere near: thickness is the only
    // binding constraint.
    let body = drilled(&mut topo, T, 20.0);
    let rim = rims_at(&topo, body, T);
    let before = measure::solid_volume(&topo, body, VOLUME_DEFLECTION).unwrap();

    // Just inside the thickness still works.
    {
        let mut t = topo.clone();
        let ok = blend_ops::fillet_v2(&mut t, body, &rim, T - 0.1).expect("r just under thickness");
        assert_watertight(&t, ok.solid, "r just under thickness");
    }

    let err = blend_ops::fillet_v2(&mut topo, body, &rim, T + 0.5)
        .err()
        .expect("a fillet deeper than the plate must fail");
    assert_eq!(
        blend_ops::blend_failure_code(&err),
        "radius-too-large",
        "the cause is the radius, not the topology: {err}"
    );
    assert!(
        err.to_string().contains(&format!("max={T}")),
        "the reported maximum is the bore's length: {err}"
    );
    let after = measure::solid_volume(&topo, body, VOLUME_DEFLECTION).unwrap();
    assert!(
        (after - before).abs() < 1e-9,
        "the refused fillet must leave the input untouched"
    );
}

/// Two rims on one bore cannot both eat more than their share of it.
///
/// Each radius passes on its own — the bore is 6 mm and each asks for 3.5 — but
/// together they would invert the wall. The second one is measured against what
/// the first left, and reports that as its maximum.
#[test]
fn paired_rim_fillets_cannot_both_consume_the_bore() {
    let mut topo = Topology::new();
    let body = drilled(&mut topo, T, 20.0);
    let mut rim = rims_at(&topo, body, T);
    rim.extend(rims_at(&topo, body, 0.0));

    let err = blend_ops::fillet_v2(&mut topo, body, &rim, 3.5)
        .err()
        .expect("two 3.5 mm rim fillets do not fit in a 6 mm bore");
    assert_eq!(
        blend_ops::blend_failure_code(&err),
        "radius-too-large",
        "{err}"
    );
    assert!(
        err.to_string().contains("max=2.5"),
        "the maximum is what the first fillet left: {err}"
    );

    // Half the thickness each is exactly what fits.
    let ok = blend_ops::fillet_v2(&mut topo, body, &rim, T / 2.0 - 0.1).expect("halves fit");
    assert_watertight(&topo, ok.solid, "paired rim fillets");
}

/// A rim fillet whose grown circle would reach the plate's own boundary must be
/// refused by radius, reporting the clearance it had.
#[test]
fn rim_fillet_reaching_the_plate_boundary_is_refused_by_radius() {
    let mut topo = Topology::new();
    // A 20 mm hole in a 60 mm-deep plate: the nearest edge is 10 mm from the
    // rim. The plate is thick so the bore length is not the binding constraint.
    let body = drilled(&mut topo, 30.0, 20.0);
    let rim = rims_at(&topo, body, 30.0);

    {
        let mut t = topo.clone();
        let ok = blend_ops::fillet_v2(&mut t, body, &rim, 9.0).expect("inside the clearance");
        assert_watertight(&t, ok.solid, "r inside clearance");
    }

    let err = blend_ops::fillet_v2(&mut topo, body, &rim, 11.0)
        .err()
        .expect("a rim fillet that reaches the plate edge must fail");
    assert_eq!(
        blend_ops::blend_failure_code(&err),
        "radius-too-large",
        "{err}"
    );
    assert!(
        err.to_string().contains("max=10"),
        "the maximum is the exact distance to the plate edge: {err}"
    );
}

/// The same rim, chamfered: the bore mouth widens by the setback.
#[test]
fn hole_rim_chamfers_at_the_bore_mouth() {
    let mut topo = Topology::new();
    let body = drilled(&mut topo, T, HOLE_R);
    let before = measure::solid_volume(&topo, body, VOLUME_DEFLECTION).unwrap();

    for d in [0.5_f64, 1.0, 2.0] {
        let mut t = topo.clone();
        let rim = rims_at(&t, body, T);
        let result = blend_ops::chamfer_v2(&mut t, body, &rim, d, d)
            .unwrap_or_else(|e| panic!("hole rim chamfer d={d}: {e}"));
        assert!(result.failed.is_empty(), "{:?}", result.failed);
        assert_watertight(&t, result.solid, &format!("hole rim chamfer d={d}"));
        assert_eq!(
            count_surface(&t, result.solid, "cone"),
            1,
            "d={d}: the rim chamfer must be one exact cone band"
        );

        let after = measure::solid_volume(&t, result.solid, VOLUME_DEFLECTION).unwrap();
        assert_removal(
            before - after,
            analytic_chamfer_removal(HOLE_R, d),
            &format!("hole rim chamfer d={d}"),
        );
    }
}

/// A counterbore: the wide bore's mouth blends both ways, and the seat below it
/// keeps its own hole.
#[test]
fn counterbore_mouth_blends_both_ways() {
    const CB_R: f64 = 6.0;
    const THK: f64 = 20.0;
    let mut topo = Topology::new();
    let body = {
        let blank = make_box(&mut topo, W, D, THK).unwrap();
        let cb = make_cylinder(&mut topo, CB_R, 8.0).unwrap();
        transform_solid(
            &mut topo,
            cb,
            &Mat4::translation(W / 2.0, D / 2.0, THK - 8.0),
        )
        .unwrap();
        let seated = boolean(&mut topo, BooleanOp::Cut, blank, cb).unwrap();
        let thru = make_cylinder(&mut topo, HOLE_R, THK + 10.0).unwrap();
        transform_solid(&mut topo, thru, &Mat4::translation(W / 2.0, D / 2.0, -5.0)).unwrap();
        boolean(&mut topo, BooleanOp::Cut, seated, thru).unwrap()
    };
    let before = measure::solid_volume(&topo, body, VOLUME_DEFLECTION).unwrap();

    let mouth: Vec<EdgeId> = rims_at(&topo, body, THK)
        .into_iter()
        .filter(|&e| {
            matches!(topo.edge(e).unwrap().curve(),
                EdgeCurve::Circle(c) if (c.radius() - CB_R).abs() < 1e-9)
        })
        .collect();
    assert_eq!(mouth.len(), 1, "one counterbore mouth");

    let r = 1.0;
    let mut t = topo.clone();
    let filleted = blend_ops::fillet_v2(&mut t, body, &mouth, r).expect("counterbore mouth fillet");
    assert_watertight(&t, filleted.solid, "counterbore mouth fillet");
    assert_removal(
        before - measure::solid_volume(&t, filleted.solid, VOLUME_DEFLECTION).unwrap(),
        analytic_fillet_removal(CB_R, r),
        "counterbore mouth fillet",
    );
    // The small through bore is untouched: still a cylinder, still a hole in
    // the seat.
    assert_eq!(count_surface(&t, filleted.solid, "cylinder"), 2);

    let mut t = topo.clone();
    let chamfered =
        blend_ops::chamfer_v2(&mut t, body, &mouth, r, r).expect("counterbore mouth chamfer");
    assert_watertight(&t, chamfered.solid, "counterbore mouth chamfer");
    assert_removal(
        before - measure::solid_volume(&t, chamfered.solid, VOLUME_DEFLECTION).unwrap(),
        analytic_chamfer_removal(CB_R, r),
        "counterbore mouth chamfer",
    );
    assert_eq!(count_surface(&t, chamfered.solid, "cylinder"), 2);
}

/// A bore-mouth chamfer that reaches the plate's boundary is refused by radius,
/// with the same exact clearance the fillet reports.
#[test]
fn rim_chamfer_reaching_the_plate_boundary_is_refused_by_radius() {
    let mut topo = Topology::new();
    let body = drilled(&mut topo, 30.0, 20.0);
    let rim = rims_at(&topo, body, 30.0);

    {
        let mut t = topo.clone();
        let ok = blend_ops::chamfer_v2(&mut t, body, &rim, 9.0, 9.0).expect("inside the clearance");
        assert_watertight(&t, ok.solid, "d inside clearance");
    }

    let err = blend_ops::chamfer_v2(&mut topo, body, &rim, 11.0, 11.0)
        .err()
        .expect("a rim chamfer that reaches the plate edge must fail");
    assert_eq!(
        blend_ops::blend_failure_code(&err),
        "radius-too-large",
        "{err}"
    );
    assert!(err.to_string().contains("max=10"), "{err}");
}
