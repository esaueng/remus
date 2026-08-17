//! Regression: a bore rim named alongside a plate's top perimeter.
//!
//! On an 80 x 60 x 6 plate with one R2.25 bore at (10, 10), an R2 fillet of the
//! four top perimeter edges worked, and an R2 fillet of the bore's top rim
//! worked, but naming both at once was refused — and refused at a PLATE CORNER,
//! a vertex the rim is 7.75 mm away from and shares nothing with:
//!
//! ```text
//! edges-not-blended: ... the blend engines refused the whole selection
//! (blend: unsupported vertex blend at Id(18): 2 stripes meet)
//! ```
//!
//! Nothing about the rim reaches that corner. What reached it was the engine
//! choice. `fillet_v2` picked ONE engine for the whole selection on an
//! all-or-nothing test — every edge a straight line between two planar faces —
//! and the two engines are complementary, not ranked: only the planar rebuild
//! closes a vertex blend where two rounded edges meet, and only the walking
//! builder assembles a closed rim. A circle in the selection failed the test,
//! so the four straight perimeter edges went to the walking builder too, and
//! that builder refuses every corner it is handed. The corner named in the
//! refusal was simply the first one its chain map reached.
//!
//! Edges further apart than twice the radius round into surfaces that cannot
//! meet, so such a selection is several independent features and each one now
//! goes to the engine that fits its shape.
//!
//! What must NOT happen is the loud refusal from #44 turning quiet: a selection
//! no engine can build must still fail, name the edges, and leave the input
//! exactly as it was, rather than come back as a silent subset.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;
use std::f64::consts::PI;

use remus_check::validate::{CheckId, Severity, ValidateOptions, validate_solid};
use remus_math::mat::Mat4;
use remus_operations::OperationsError;
use remus_operations::blend_ops;
use remus_operations::blend_ops::BlendError;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const W: f64 = 80.0;
const D: f64 = 60.0;
const T: f64 = 6.0;
const BORE_R: f64 = 2.25;
const BORE_X: f64 = 10.0;
const BORE_Y: f64 = 10.0;
const R: f64 = 2.0;

/// Deflection for the volume comparisons.
const VOLUME_DEFLECTION: f64 = 0.001;

/// How far below its closed form a measured volume may sit.
///
/// A body with a hole is measured off its watertight mesh, and that mesh is
/// inscribed — every curved face is chorded INSIDE the true surface — so the
/// measurement always under-reads, by an amount that shrinks with the
/// deflection: on this part 0.89 mm³ at 0.005, 0.21 at 0.001, 0.10 at 0.0005.
/// The budget is 1.8 parts in 10^5 of the body. It is deliberately far smaller
/// than the 14.54 mm³ the rim blend removes and the 241.86 mm³ the perimeter
/// blend removes, so neither feature could go missing inside it.
const VOLUME_BUDGET: f64 = 0.5;

/// The part: plate blank minus the named bores, each drilled clear through.
fn plate(topo: &mut Topology, bores: &[(f64, f64)]) -> SolidId {
    let mut body = make_box(topo, W, D, T).expect("plate blank");
    for &(x, y) in bores {
        let drill = make_cylinder(topo, BORE_R, T + 4.0).expect("drill");
        transform_solid(topo, drill, &Mat4::translation(x, y, -2.0)).expect("place drill");
        body = boolean(topo, BooleanOp::Cut, body, drill).expect("drill the bore");
    }
    body
}

/// The part the defect was reported on: one bore at (10, 10).
fn one_bore_plate(topo: &mut Topology) -> SolidId {
    plate(topo, &[(BORE_X, BORE_Y)])
}

fn solid_edge_list(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
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

/// The four straight edges of the top face.
fn top_perimeter(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    let edges: Vec<EdgeId> = solid_edge_list(topo, s)
        .into_iter()
        .filter(|&e| {
            let ed = topo.edge(e).unwrap();
            if ed.start() == ed.end() || !matches!(ed.curve(), EdgeCurve::Line) {
                return false;
            }
            let a = topo.vertex(ed.start()).unwrap().point();
            let b = topo.vertex(ed.end()).unwrap().point();
            (a.z() - T).abs() < 1e-9 && (b.z() - T).abs() < 1e-9
        })
        .collect();
    assert_eq!(edges.len(), 4, "the top face must have four straight edges");
    edges
}

/// A bore's closed circular rim on the top face — one edge, one seam vertex.
fn top_rim_at(topo: &Topology, s: SolidId, x: f64, y: f64) -> Vec<EdgeId> {
    let edges: Vec<EdgeId> = solid_edge_list(topo, s)
        .into_iter()
        .filter(|&e| {
            let ed = topo.edge(e).unwrap();
            let EdgeCurve::Circle(c) = ed.curve() else {
                return false;
            };
            ed.start() == ed.end()
                && (c.center().z() - T).abs() < 1e-9
                && (c.center().x() - x).abs() < 1e-9
                && (c.center().y() - y).abs() < 1e-9
        })
        .collect();
    assert_eq!(
        edges.len(),
        1,
        "the bore at ({x}, {y}) must have one top rim"
    );
    edges
}

/// The rim of the reported part's only bore.
fn top_rim(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    top_rim_at(topo, s, BORE_X, BORE_Y)
}

/// How many faces of each surface type the result carries.
fn surface_census(topo: &Topology, s: SolidId) -> HashMap<&'static str, usize> {
    let mut census: HashMap<&'static str, usize> = HashMap::new();
    for fid in solid_faces(topo, s).unwrap() {
        *census
            .entry(topo.face(fid).unwrap().surface().type_tag())
            .or_insert(0) += 1;
    }
    census
}

/// (free edges, non-manifold edges) in the B-rep.
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

/// (free edges, non-manifold edges) in the tessellation.
fn mesh_edge_health(topo: &Topology, s: SolidId) -> (usize, usize) {
    let mesh = tessellate_solid_with_tolerance(topo, s, 0.01, 0.1).unwrap();
    let q = 1e6;
    let mut canon: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut remap = vec![0u32; mesh.positions.len()];
    for (i, p) in mesh.positions.iter().enumerate() {
        #[allow(clippy::cast_possible_truncation)]
        let key = (
            (p.x() * q).round() as i64,
            (p.y() * q).round() as i64,
            (p.z() * q).round() as i64,
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
    (
        edges.values().filter(|&&c| c == 1).count(),
        edges.values().filter(|&&c| c >= 3).count(),
    )
}

/// Error-severity magnitudes of a validation report, keyed by check.
fn error_magnitudes(topo: &Topology, s: SolidId) -> HashMap<CheckId, f64> {
    let report = validate_solid(topo, s, &ValidateOptions::default()).unwrap();
    let mut map: HashMap<CheckId, f64> = HashMap::new();
    for issue in &report.issues {
        if issue.severity == Severity::Error {
            *map.entry(issue.check).or_insert(0.0) += issue.deviation.unwrap_or(1.0);
        }
    }
    map
}

/// The result must be a closed 2-manifold, in the B-rep, in the mesh, and by
/// `remus-check`.
///
/// `validate_solid` is compared against the INPUT rather than asserted clean:
/// the boolean's own drilled plate already fails `ShellOrientationConsistent`
/// (both rims of a bore are traversed the same way by the cap and the wall),
/// so the fixture is red before any blend runs. That is a boolean-side defect
/// and not this blend's to fix; what the blend owes is that it introduces no
/// new error and worsens none — and that `ShellClosed`, the watertightness
/// check itself, is clean outright.
fn assert_watertight(topo: &Topology, input: SolidId, s: SolidId, what: &str) {
    let shell_id = topo.solid(s).unwrap().outer_shell();
    let shell = topo.shell(shell_id).unwrap().clone();
    validate_shell_closed(&shell, topo)
        .unwrap_or_else(|e| panic!("{what}: the result shell must be closed, got {e}"));
    assert_eq!(
        brep_edge_health(topo, s),
        (0, 0),
        "{what}: no free or non-manifold B-rep edges"
    );
    assert_eq!(
        mesh_edge_health(topo, s),
        (0, 0),
        "{what}: the tessellation must be watertight"
    );

    let before = error_magnitudes(topo, input);
    let after = error_magnitudes(topo, s);
    assert!(
        !after.contains_key(&CheckId::ShellClosed),
        "{what}: the result must carry no ShellClosed error"
    );
    for (check, &magnitude) in &after {
        let baseline = before.get(check).copied().unwrap_or(0.0);
        assert!(
            magnitude <= baseline,
            "{what}: {check:?} went from {baseline} to {magnitude}"
        );
    }
}

/// Volume of the blank with its bore: `80·60·6 − π·2.25²·6`.
fn bored_plate_volume() -> f64 {
    W * D * T - PI * BORE_R * BORE_R * T
}

/// Material an R2 fillet of the four top perimeter edges removes.
///
/// Along a straight run the section removed is the square corner less the
/// quarter disc, `r²(1 − π/4)`. The four edges total 280 mm and each of the
/// four corners consumes `r` from each of the two edges meeting there, leaving
/// `280 − 8r = 264` mm of straight run.
///
/// At a corner only the two TOP edges are rounded — the vertical edge below it
/// stays sharp — and what a rolling ball leaves there is the octant of the
/// corner ball plus the planar ledge that sharp edge runs out onto, one radius
/// down. Inside the `r`-cube at the corner that keeps exactly the ball octant,
/// so the corner loses `r³ − πr³/6`.
///
/// Total: `(4 − π)·264 + 4·8·(1 − π/6)` = `1088 − 808π/3`.
fn perimeter_removal() -> f64 {
    let straight = R * R * (1.0 - PI / 4.0) * (2.0 * (W + D) - 8.0 * R);
    let corners = 4.0 * R * R * R * (1.0 - PI / 6.0);
    straight + corners
}

/// Material an R2 fillet of the bore's top rim removes.
///
/// The section is the same corner-less-quarter-disc, taken in the half plane
/// through the bore axis: the square `[a, a+r] × [T−r, T]` minus the quarter
/// disc centred on `(a+r, T−r)`. Pappus turns its first moment about the axis
/// into the volume of revolution.
///
///   square:       area `r²`, centroid at `a + r/2`      ⇒ moment `4 · 3.25`
///   quarter disc: area `πr²/4`, centroid at `a + r − 4r/(3π)`
///                                                       ⇒ moment `17π/4 − 8/3`
///   region moment `= 47/3 − 17π/4`, volume `= 2π(47/3 − 17π/4) = 94π/3 − 17π²/2`.
fn rim_removal() -> f64 {
    let square_moment = R * R * (BORE_R + R / 2.0);
    let quarter_moment = PI * R * R / 4.0 * (BORE_R + R - 4.0 * R / (3.0 * PI));
    2.0 * PI * (square_moment - quarter_moment)
}

/// Baseline: each half of the selection blends on its own, exactly as it did
/// before. If either of these ever fails, the combined case below is proving
/// nothing.
#[test]
fn each_feature_blends_on_its_own() {
    let mut topo = Topology::new();
    let body = one_bore_plate(&mut topo);
    let before = measure::solid_volume(&topo, body, VOLUME_DEFLECTION).unwrap();
    assert!(
        (before - bored_plate_volume()).abs() < 1e-9,
        "the fixture must be the exact bored plate, got {before}"
    );

    {
        let mut t = topo.clone();
        let edges = top_perimeter(&t, body);
        let result = blend_ops::fillet_v2(&mut t, body, &edges, R).expect("perimeter only");
        assert!(result.failed.is_empty(), "{:?}", result.failed);
        assert_watertight(&t, body, result.solid, "perimeter only");

        let census = surface_census(&t, result.solid);
        assert_eq!(
            census.get("sphere").copied().unwrap_or(0),
            4,
            "each of the four corners must round into a spherical patch, got {census:?}"
        );
        assert_eq!(
            census.get("torus").copied().unwrap_or(0),
            0,
            "no rim was named, so there must be no torus, got {census:?}"
        );

        let after = measure::solid_volume(&t, result.solid, VOLUME_DEFLECTION).unwrap();
        let want = bored_plate_volume() - perimeter_removal();
        assert!(
            (want - after) < VOLUME_BUDGET && (after - want) < 1e-9,
            "perimeter only measured {after}, closed form {want}; the mesh may only under-read"
        );
    }

    {
        let mut t = topo.clone();
        let edges = top_rim(&t, body);
        let result = blend_ops::fillet_v2(&mut t, body, &edges, R).expect("rim only");
        assert!(result.failed.is_empty(), "{:?}", result.failed);
        assert_watertight(&t, body, result.solid, "rim only");

        let census = surface_census(&t, result.solid);
        assert_eq!(
            census.get("torus").copied().unwrap_or(0),
            1,
            "the rim must round into one torus, got {census:?}"
        );

        // This body is still fully analytic, so its volume is exact, not
        // quadrature: hold it to the closed form outright.
        let removed = before - measure::solid_volume(&t, result.solid, VOLUME_DEFLECTION).unwrap();
        assert!(
            (removed - rim_removal()).abs() < 1e-6,
            "rim only removed {removed}, closed form {}",
            rim_removal()
        );
    }
}

/// The defect: both features named in one call must both be blended.
#[test]
fn perimeter_and_rim_blend_together() {
    let mut topo = Topology::new();
    let body = one_bore_plate(&mut topo);
    let before = measure::solid_volume(&topo, body, VOLUME_DEFLECTION).unwrap();

    let mut edges = top_perimeter(&topo, body);
    edges.extend(top_rim(&topo, body));
    assert_eq!(edges.len(), 5);

    let result = blend_ops::fillet_v2(&mut topo, body, &edges, R)
        .expect("the perimeter and the rim are 7.75 mm apart and must blend together");
    assert!(result.failed.is_empty(), "{:?}", result.failed);
    assert_eq!(
        result.succeeded.len(),
        edges.len(),
        "every named edge must be reported as blended"
    );

    let census = surface_census(&topo, result.solid);
    assert_eq!(
        census.get("torus").copied().unwrap_or(0),
        1,
        "the rim's torus must be there, got {census:?}"
    );
    assert_eq!(
        census.get("sphere").copied().unwrap_or(0),
        4,
        "all four corner patches must be there, got {census:?}"
    );
    assert_eq!(
        census.get("cylinder").copied().unwrap_or(0),
        5,
        "four perimeter blends plus the bore wall, got {census:?}"
    );

    assert_watertight(&topo, body, result.solid, "perimeter and rim");

    // The two blends are disjoint, so the body loses exactly the sum of what
    // each removes on its own.
    let after = measure::solid_volume(&topo, result.solid, VOLUME_DEFLECTION).unwrap();
    let want = bored_plate_volume() - perimeter_removal() - rim_removal();
    assert!(
        (before - after) > 0.0,
        "a convex fillet must remove material"
    );
    assert!(
        (want - after) < VOLUME_BUDGET && (after - want) < 1e-9,
        "combined measured {after}, closed form {want} \
         (= 27712 + 207.625\u{3c0} + 8.5\u{3c0}\u{b2}); the mesh may only under-read"
    );
}

/// The #44 contract, still intact on the new route: when one feature of a mixed
/// selection genuinely cannot be blended, the whole call fails, names the edge
/// that could not be blended, and leaves the input untouched. It must not come
/// back as the subset that would have worked.
///
/// Two bores 6 mm apart make the rim impossible on its own terms: an R2 rim
/// fillet grows the top contact circle to `2.25 + 2 = 4.25` mm, past the 3.75 mm
/// of clear cap between the bore's centre and its neighbour's wall, so the
/// rebuilt cap would cross its own hole. The perimeter, 27.75 mm away, rounds
/// perfectly — which is exactly what makes the silent-subset answer tempting.
#[test]
fn an_impossible_feature_still_refuses_the_whole_selection() {
    let bores = [(30.0, 30.0), (36.0, 30.0)];
    let mut topo = Topology::new();
    let body = plate(&mut topo, &bores);
    let before = measure::solid_volume(&topo, body, VOLUME_DEFLECTION).unwrap();
    let faces_before = solid_faces(&topo, body).unwrap().len();

    let rim = top_rim_at(&topo, body, bores[0].0, bores[0].1);
    let perimeter = top_perimeter(&topo, body);
    {
        // The premise: the perimeter half of this selection is fine.
        let mut t = topo.clone();
        blend_ops::fillet_v2(&mut t, body, &perimeter, R)
            .expect("the perimeter alone must still blend");
    }

    let mut edges = perimeter;
    edges.extend(rim.iter().copied());
    let error = blend_ops::fillet_v2(&mut topo, body, &edges, R)
        .err()
        .expect("a rim that cannot be built must not come back as the perimeter alone");
    match error {
        OperationsError::Blend(BlendError::RadiusTooLarge { edge, .. }) => {
            assert_eq!(edge, rim[0], "the refusal must name the rim edge");
        }
        other => panic!("expected the rim's radius refusal, got {other}"),
    }

    assert_eq!(
        solid_faces(&topo, body).unwrap().len(),
        faces_before,
        "the failed blend must leave the input's faces alone"
    );
    let after = measure::solid_volume(&topo, body, VOLUME_DEFLECTION).unwrap();
    assert!(
        (after - before).abs() < 1e-9,
        "the failed blend must leave the input's volume alone, {before} -> {after}"
    );
}
