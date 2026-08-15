//! Regression: chamfering a CLOSED circular edge (a cylinder rim).
//!
//! `chamfer_v2` refused every closed edge outright — `reject_closed_edges`
//! returned "closed-edge chamfer assembly is not yet supported" — because the
//! per-face, line-based trimmer cannot cut a face along a closed interior
//! contact loop: there are no endpoints to cut at. The v1 flat-bevel engine is
//! planar-only and fails such an edge with "cannot normalize zero vector".
//! Between them, no engine could chamfer a cylinder rim at all.
//!
//! `fillet_builder` already solved exactly this with an annular rebuild
//! (`closed_rim_info` / `assemble_closed_rim`): rebuild the disc cap bounded by
//! the plate-contact circle, shorten the wall to the wall-contact circle, and
//! emit the band between them sharing both edges. The chamfer band is the same
//! construction with a cone instead of a torus and a straight ruled seam
//! instead of a minor arc.
//!
//! This stayed invisible for a long time because the OpenZCAD flange demo
//! chamfers its rim and had been passing: the mesh-boolean fallback handed it a
//! body whose "rim" was a polyline of straight segments, which the planar v1
//! engine handles. Fixing the booleans made the blank analytic, the rim became
//! a real circle, and the chamfer started failing.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_blend::BlendError;
use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::OperationsError;
use remus_operations::blend_ops;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure;
use remus_operations::primitives;
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

const R: f64 = 45.0;
const H: f64 = 10.0;

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
    mesh_edge_health_at(topo, s, 0.01, 0.1)
}

/// Mesh health at a chosen tolerance.
///
/// Worth sweeping rather than fixing at one setting: a band and its neighbour
/// wall pick their own chord-deviation sample counts, so whether they happen to
/// agree depends on the tolerance as much as on the geometry. A bore-mouth
/// chamfer that leaks 58 boundary edges at (0.02, 0.3) reads perfectly closed
/// at (0.01, 0.1).
fn mesh_edge_health_at(
    topo: &Topology,
    s: SolidId,
    deflection: f64,
    angular: f64,
) -> (usize, usize) {
    let mesh = tessellate_solid_with_tolerance(topo, s, deflection, angular).unwrap();
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

/// Distinct edges of a solid, in discovery order: `[bottom rim, seam, top rim]`
/// for a cylinder primitive.
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

fn closed_rims(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    solid_edges(topo, s)
        .into_iter()
        .filter(|&e| {
            let ed = topo.edge(e).unwrap();
            ed.start() == ed.end() && ed.curve().type_tag() == "circle"
        })
        .collect()
}

/// Material removed by a symmetric chamfer of setback `d` on a rim of radius
/// `R`, by Pappus: the right triangle (legs `d`, `d`, area `d²/2`) revolved
/// about the axis at centroid radius `R − d/3`.
fn expected_volume(d: f64) -> f64 {
    let full = std::f64::consts::PI * R * R * H;
    full - 0.5 * d * d * std::f64::consts::TAU * (R - d / 3.0)
}

#[test]
fn closed_rim_chamfer_is_exact_and_watertight() {
    for d in [0.5_f64, 1.5, 3.0] {
        for rim_index in 0..2 {
            let mut topo = Topology::new();
            let cyl = primitives::make_cylinder(&mut topo, R, H).unwrap();
            let rims = closed_rims(&topo, cyl);
            assert_eq!(rims.len(), 2, "a cylinder has two closed rim circles");

            let r = blend_ops::chamfer_v2(&mut topo, cyl, &[rims[rim_index]], d, d)
                .unwrap_or_else(|e| panic!("d={d} rim={rim_index}: chamfer failed: {e:?}"));
            assert!(r.failed.is_empty(), "d={d} rim={rim_index}: {:?}", r.failed);

            // The band must be an exact analytic cone, not a NURBS approximation.
            let census = surface_census(&topo, r.solid);
            assert_eq!(
                census.get("cone").copied().unwrap_or(0),
                1,
                "d={d} rim={rim_index}: chamfer band must be an analytic cone: {census:?}"
            );
            assert_eq!(
                census.get("cylinder").copied().unwrap_or(0),
                1,
                "d={d} rim={rim_index}: the wall stays a cylinder: {census:?}"
            );
            assert_eq!(
                census.values().sum::<usize>(),
                4,
                "d={d} rim={rim_index}: wall + two caps + band: {census:?}"
            );

            assert_eq!(
                brep_edge_health(&topo, r.solid),
                (0, 0),
                "d={d} rim={rim_index}: B-Rep must be closed and manifold"
            );
            // A closed B-Rep can still mesh open — check the mesh separately.
            assert_eq!(
                mesh_edge_health(&topo, r.solid),
                (0, 0),
                "d={d} rim={rim_index}: tessellation must be watertight"
            );

            // Volume against the Pappus closed form. The band is exact, so this
            // is tight rather than a loose tolerance.
            let vol = measure::solid_volume(&topo, r.solid, 0.02).unwrap();
            let want = expected_volume(d);
            assert!(
                (vol - want).abs() / want < 1e-6,
                "d={d} rim={rim_index}: volume {vol} vs Pappus {want}"
            );
        }
    }
}

#[test]
fn closed_rim_chamfer_removes_material_on_the_right_side() {
    let d = 3.0;
    let mut topo = Topology::new();
    let cyl = primitives::make_cylinder(&mut topo, R, H).unwrap();
    // Chamfer the TOP rim (z = H).
    let rims = closed_rims(&topo, cyl);
    let top = rims
        .into_iter()
        .find(|&e| {
            let p = topo.vertex(topo.edge(e).unwrap().start()).unwrap().point();
            (p.z() - H).abs() < 1e-9
        })
        .expect("top rim");
    let r = blend_ops::chamfer_v2(&mut topo, cyl, &[top], d, d).expect("top rim chamfer");

    let opts = ClassifyOptions::default();
    let probe = |x: f64, z: f64| classify_point(&topo, r.solid, Point3::new(x, 0.0, z), &opts);

    // Just outside the chamfered corner — removed.
    assert_eq!(
        probe(R - 0.4, H - 0.4).unwrap(),
        PointClassification::Outside,
        "the top outer corner must be cut away"
    );
    // The mirrored point at the UNCHAMFERED bottom rim — still material.
    assert_eq!(
        probe(R - 0.4, 0.4).unwrap(),
        PointClassification::Inside,
        "the bottom rim was not chamfered and must be untouched"
    );
    // Deep interior — material.
    assert_eq!(
        probe(0.0, H / 2.0).unwrap(),
        PointClassification::Inside,
        "the core is solid"
    );
    // Well inside the top face but away from the rim — material.
    assert_eq!(
        probe(R - 10.0, H - 0.4).unwrap(),
        PointClassification::Inside,
        "only the rim corner is removed, not the whole top"
    );
}

/// The rim assembler must handle a cap that carries HOLES, not just a bare
/// disc.
///
/// The first version of this fix bailed on any cap with inner wires, because
/// the rebuild handed the new cap an empty inner-wire list — which would have
/// filled in every hole and stranded each bore wall. That restriction passed
/// the bare-cylinder repro and still failed the real case: the drilled flange's
/// rim cap is an annulus with a central opening and six bolt holes, and
/// chamfering it reported "trimming failure". The cap's holes are now carried
/// through, guarded by a check that the shrinking outer boundary still clears
/// them.
#[test]
fn rim_chamfer_preserves_cap_holes() {
    use remus_math::mat::Mat4;
    use remus_math::vec::Vec3;
    use remus_operations::boolean::{BooleanOp, boolean};
    use remus_operations::heal::unify_faces;
    use remus_operations::revolve::revolve;
    use remus_operations::transform::transform_solid;
    use remus_topology::builder::{make_planar_face_from_wire, make_polygon_wire};

    let revolved = |t: &mut Topology, ri: f64, ro: f64, z0: f64, z1: f64| {
        let pts = [
            Point3::new(ri, 0.0, z0),
            Point3::new(ro, 0.0, z0),
            Point3::new(ro, 0.0, z1),
            Point3::new(ri, 0.0, z1),
        ];
        let w = make_polygon_wire(t, &pts, 1e-7).unwrap();
        let f = make_planar_face_from_wire(t, w).unwrap();
        revolve(
            t,
            f,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            std::f64::consts::TAU,
        )
        .unwrap()
    };

    let mut topo = Topology::new();
    let rim = revolved(&mut topo, 24.0, 45.0, 0.0, 10.0);
    let hub = revolved(&mut topo, 12.0, 24.0, 0.0, 26.0);
    let blank = boolean(&mut topo, BooleanOp::Fuse, rim, hub).unwrap();
    unify_faces(&mut topo, blank).unwrap();

    let mut pattern = None;
    for i in 0..6 {
        let a = std::f64::consts::TAU * f64::from(i) / 6.0;
        let c = primitives::make_cylinder(&mut topo, 3.0, 16.0).unwrap();
        transform_solid(
            &mut topo,
            c,
            &Mat4::translation(34.0 * a.cos(), 34.0 * a.sin(), -3.0),
        )
        .unwrap();
        pattern = Some(match pattern {
            None => c,
            Some(p) => boolean(&mut topo, BooleanOp::Fuse, p, c).unwrap(),
        });
    }
    let body = boolean(&mut topo, BooleanOp::Cut, blank, pattern.unwrap()).unwrap();

    // The two r45 rims plus the r24 hub lip — the edges the flange demo picks.
    let picks: Vec<EdgeId> = solid_edges(&topo, body)
        .into_iter()
        .filter(|&e| {
            let ed = topo.edge(e).unwrap();
            if ed.start() != ed.end() {
                return false;
            }
            let p = topo.vertex(ed.start()).unwrap().point();
            let r = p.x().hypot(p.y());
            (r - 45.0).abs() < 1e-6 || ((r - 24.0).abs() < 1e-6 && p.z() >= 25.5)
        })
        .collect();
    assert_eq!(picks.len(), 3, "two r45 rims and the r24 hub lip");

    // Each cap involved really does carry holes — otherwise this test would
    // silently be re-testing the bare-disc case.
    let holed_caps = solid_faces(&topo, body)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            let face = topo.face(f).unwrap();
            face.surface().type_tag() == "plane" && !face.inner_wires().is_empty()
        })
        .count();
    assert!(holed_caps >= 2, "the flange caps must be holed annuli");

    let before = measure::solid_volume(&topo, body, 0.05).unwrap();
    let d = 1.5;
    let r = blend_ops::chamfer_v2(&mut topo, body, &picks, d, d)
        .expect("all three flange rims must chamfer");
    assert!(r.failed.is_empty(), "{:?}", r.failed);

    // Three exact conical bands; the nine cylinders (3 body + 6 bores) survive.
    let census = surface_census(&topo, r.solid);
    assert_eq!(census.get("cone").copied().unwrap_or(0), 3, "{census:?}");
    assert_eq!(
        census.get("cylinder").copied().unwrap_or(0),
        9,
        "{census:?}"
    );

    assert_eq!(
        brep_edge_health(&topo, r.solid),
        (0, 0),
        "the chamfered flange must stay closed — a dropped cap hole would \
         strand its bore wall and open the shell"
    );
    assert_eq!(
        mesh_edge_health(&topo, r.solid),
        (0, 0),
        "and it must tessellate watertight"
    );

    // Volume by Pappus, one wedge per rim.
    let wedge = |rr: f64| 0.5 * d * d * std::f64::consts::TAU * (rr - d / 3.0);
    let want = before - 2.0 * wedge(45.0) - wedge(24.0);
    let got = measure::solid_volume(&topo, r.solid, 0.05).unwrap();
    assert!(
        (got - want).abs() / want < 1e-4,
        "volume {got} vs Pappus {want}"
    );

    // The bolt holes must still be holes.
    let opts = ClassifyOptions::default();
    assert_eq!(
        classify_point(&topo, r.solid, Point3::new(34.0, 0.0, 5.0), &opts).unwrap(),
        PointClassification::Outside,
        "a bolt hole must survive the chamfer"
    );
}

/// Chamfering BOTH rims in one call must also work, and must be symmetric.
#[test]
fn both_rims_chamfered_in_one_call() {
    let d = 2.0;
    let mut topo = Topology::new();
    let cyl = primitives::make_cylinder(&mut topo, R, H).unwrap();
    let rims = closed_rims(&topo, cyl);

    let r = blend_ops::chamfer_v2(&mut topo, cyl, &rims, d, d).expect("both rims");
    assert!(r.failed.is_empty(), "{:?}", r.failed);

    let census = surface_census(&topo, r.solid);
    assert_eq!(
        census.get("cone").copied().unwrap_or(0),
        2,
        "one cone band per rim: {census:?}"
    );
    assert_eq!(
        brep_edge_health(&topo, r.solid),
        (0, 0),
        "double-chamfered solid must be closed"
    );
    assert_eq!(
        mesh_edge_health(&topo, r.solid),
        (0, 0),
        "double-chamfered solid must mesh watertight"
    );

    // Both chamfers removed, so subtract the Pappus wedge twice.
    let full = std::f64::consts::PI * R * R * H;
    let one = full - expected_volume(d);
    let want = full - 2.0 * one;
    let vol = measure::solid_volume(&topo, r.solid, 0.02).unwrap();
    assert!(
        (vol - want).abs() / want < 1e-6,
        "volume {vol} vs {want} (two chamfers)"
    );
}

// ---------------------------------------------------------------------------
// Bore mouths: the rim as an INNER wire of the cap.
//
// The rebuild above swaps the cap's OUTER wire for the plate contact, which
// covers a disc rim and an annulus rim. A bore mouth is the other way round:
// the rim bounds a hole *through* the cap, material lies outside it, and the
// chamfer widens the hole instead of shrinking the boundary. Those failed with
// `TrimmingFailure` until the rebuild learned to move whichever single wire the
// rim forms and carry the rest.
//
// They also force the mesh question. The band and the wall sample their shared
// contact circle from their own surfaces, and here the two contact radii always
// differ — the plate contact sits a full setback outside the wall — so the
// sample counts differ too. The result was closed, manifold and
// `validate_solid`-clean with a mesh full of holes.
// ---------------------------------------------------------------------------

/// Tolerances the bore-mouth meshes are checked at. One setting proves very
/// little here: the leak this pins is invisible at (0.01, 0.1) and 58 boundary
/// edges wide at (0.02, 0.3), which is close to the tessellator's own default.
const MESH_TOLERANCES: [(f64, f64); 5] = [
    (0.01, 0.1),
    (0.02, 0.3),
    (0.05, 0.3),
    (0.005, 0.2),
    (0.1, 0.5),
];

/// A 20x20x6 plate with an r=3 bore straight through it at `(x, y)`.
fn drilled_plate_at(topo: &mut Topology, x: f64, y: f64) -> SolidId {
    let plate = primitives::make_box(topo, 20.0, 20.0, 6.0).unwrap();
    let tool = primitives::make_cylinder(topo, 3.0, 20.0).unwrap();
    transform_solid(topo, tool, &Mat4::translation(x, y, -5.0)).unwrap();
    boolean(topo, BooleanOp::Cut, plate, tool).unwrap()
}

/// A 20x20x6 plate with a centred r=3 bore straight through it.
fn drilled_plate(topo: &mut Topology) -> SolidId {
    drilled_plate_at(topo, 10.0, 10.0)
}

/// Material removed by a symmetric chamfer of setback `d` at the mouth of a
/// bore of radius `r`, by Pappus: the right triangle (legs `d`, `d`, area
/// `d²/2`) revolved about the axis at centroid radius `r + d/3` — plus, not
/// minus, because the chamfer eats outward into the surrounding plate.
fn expected_bore_removal(r: f64, d: f64) -> f64 {
    0.5 * d * d * std::f64::consts::TAU * (r + d / 3.0)
}

#[test]
fn bore_mouth_chamfer_is_exact_and_watertight() {
    for (label, z) in [("bottom", 0.0), ("top", 6.0)] {
        let mut topo = Topology::new();
        let drilled = drilled_plate(&mut topo);
        let before = measure::solid_volume(&topo, drilled, 0.01).unwrap();

        let rim = closed_rims(&topo, drilled)
            .into_iter()
            .find(|&e| {
                let ed = topo.edge(e).unwrap();
                (topo.vertex(ed.start()).unwrap().point().z() - z).abs() < 1e-9
            })
            .expect("a through bore has a rim at each face");

        let d = 0.5;
        let result = blend_ops::chamfer_v2(&mut topo, drilled, &[rim], d, d).unwrap();
        assert!(!result.is_partial, "{label}: chamfer must fully succeed");

        let (free, nonmanifold) = brep_edge_health(&topo, result.solid);
        assert_eq!((free, nonmanifold), (0, 0), "{label}: B-Rep edge health");

        for (deflection, angular) in MESH_TOLERANCES {
            assert_eq!(
                mesh_edge_health_at(&topo, result.solid, deflection, angular),
                (0, 0),
                "{label}: the chamfered bore must MESH closed at \
                 (deflection {deflection}, angular {angular}) — the band and \
                 the wall sample the shared contact circle at different \
                 resolutions, and no B-Rep gate catches that"
            );
        }

        let census = surface_census(&topo, result.solid);
        assert_eq!(
            census.get("cone").copied().unwrap_or(0),
            1,
            "{label}: a rim chamfer is one cone band"
        );

        let after = measure::solid_volume(&topo, result.solid, 0.01).unwrap();
        let expected = expected_bore_removal(3.0, d);
        assert!(
            (before - after - expected).abs() < 1e-2,
            "{label}: expected {expected:.4} removed, got {:.4}",
            before - after
        );
    }
}

/// Both mouths of the same bore in one call: the second rim's assembly has to
/// find its boundary on the cap the first one already replaced, and shorten a
/// wall that has already been shortened at the other end.
#[test]
fn both_bore_mouths_chamfered_in_one_call() {
    let mut topo = Topology::new();
    let drilled = drilled_plate(&mut topo);
    let before = measure::solid_volume(&topo, drilled, 0.01).unwrap();

    let rims = closed_rims(&topo, drilled);
    assert_eq!(rims.len(), 2, "a through bore has two mouths");

    let d = 0.5;
    let result = blend_ops::chamfer_v2(&mut topo, drilled, &rims, d, d).unwrap();
    assert!(!result.is_partial);
    assert_eq!(result.succeeded.len(), 2);

    assert_eq!(brep_edge_health(&topo, result.solid), (0, 0));
    for (deflection, angular) in MESH_TOLERANCES {
        assert_eq!(
            mesh_edge_health_at(&topo, result.solid, deflection, angular),
            (0, 0),
            "must mesh closed at (deflection {deflection}, angular {angular})"
        );
    }
    assert_eq!(
        surface_census(&topo, result.solid)
            .get("cone")
            .copied()
            .unwrap_or(0),
        2,
        "one band per mouth"
    );

    let after = measure::solid_volume(&topo, result.solid, 0.01).unwrap();
    let expected = 2.0 * expected_bore_removal(3.0, d);
    assert!(
        (before - after - expected).abs() < 2e-2,
        "expected {expected:.4} removed, got {:.4}",
        before - after
    );
}

/// A setback wide enough to swallow the plate's own outer boundary has no
/// annular rebuild — the widened mouth would have to merge with the outside of
/// the part. It must be declined, not approximated.
#[test]
fn bore_mouth_chamfer_wider_than_the_plate_is_declined() {
    let mut topo = Topology::new();
    let drilled = drilled_plate(&mut topo);
    let rim = closed_rims(&topo, drilled)[0];

    // The plate's nearest wall is 10mm from the bore axis, the bore r=3, so a
    // 9mm setback puts the plate contact at r=12 — outside the part.
    let result = blend_ops::chamfer_v2(&mut topo, drilled, &[rim], 9.0, 9.0);
    assert!(
        result.is_err(),
        "a chamfer that outgrows the face must fail, not produce a cap whose \
         wires cross"
    );
}

/// The reported off-axis crossing sits exactly between two of the old fixed
/// samples on the plate's x=0 edge. The bore centre is 3.2 mm from that edge,
/// so an r=3.0 mouth has 0.2 mm of real clearance. A 0.3 mm chamfer grows the
/// contact to r=3.3 and crosses the boundary, even though samples at y=5 and
/// y=7.5 are both more than 3.4 mm from the bore axis.
#[test]
fn off_axis_bore_mouth_chamfer_crossing_is_refused() {
    const CLEARANCE: f64 = 0.2;

    let mut topo = Topology::new();
    let drilled = drilled_plate_at(&mut topo, 3.2, 6.25);
    let rim = closed_rims(&topo, drilled)
        .into_iter()
        .find(|&e| {
            let edge = topo.edge(e).unwrap();
            (topo.vertex(edge.start()).unwrap().point().z() - 6.0).abs() < 1e-9
        })
        .expect("the off-axis bore has a top mouth");

    // Guard the adversarial premise: the obsolete nine-point scan really
    // misses this crossing on the actual post-boolean cap wire.
    let cap = solid_faces(&topo, drilled)
        .unwrap()
        .into_iter()
        .find(|&face_id| {
            let face = topo.face(face_id).unwrap();
            face.surface().type_tag() == "plane"
                && face.inner_wires().iter().any(|&wire_id| {
                    topo.wire(wire_id)
                        .unwrap()
                        .edges()
                        .iter()
                        .any(|edge| edge.edge() == rim)
                })
        })
        .expect("the top cap carries the bore rim as an inner wire");
    let mut legacy_min = f64::INFINITY;
    for oriented in topo
        .wire(topo.face(cap).unwrap().outer_wire())
        .unwrap()
        .edges()
    {
        let edge = topo.edge(oriented.edge()).unwrap();
        let start = topo.vertex(edge.start()).unwrap().point();
        let end = topo.vertex(edge.end()).unwrap().point();
        let (t0, t1) = edge.curve().domain_with_endpoints(start, end);
        for k in 0..=8 {
            let t = t0 + (t1 - t0) * f64::from(k) / 8.0;
            let point = edge.curve().evaluate_with_endpoints(t, start, end);
            legacy_min = legacy_min.min((point.x() - 3.2).hypot(point.y() - 6.25));
        }
    }
    assert!(
        legacy_min > 3.3,
        "the fixture must evade the obsolete samples, got minimum {legacy_min}"
    );

    // Preserve supported blends immediately below the exact clearance.
    {
        let mut supported = topo.clone();
        let ok = blend_ops::chamfer_v2(&mut supported, drilled, &[rim], 0.19, 0.19)
            .expect("a chamfer below the exact boundary clearance must remain supported");
        assert!(!ok.is_partial);
        assert_eq!(brep_edge_health(&supported, ok.solid), (0, 0));
        assert_eq!(mesh_edge_health(&supported, ok.solid), (0, 0));
        assert_eq!(
            surface_census(&supported, ok.solid)
                .get("cone")
                .copied()
                .unwrap_or(0),
            1,
            "the supported path must still use one exact cone band"
        );
    }

    let before = measure::solid_volume(&topo, drilled, 0.01).unwrap();
    let err = blend_ops::chamfer_v2(&mut topo, drilled, &[rim], 0.3, 0.3)
        .err()
        .expect("the r=3.3 plate contact crosses the x=0 boundary");
    match err {
        OperationsError::Blend(BlendError::RadiusTooLarge { edge, max_radius }) => {
            assert_eq!(edge, rim);
            assert!(
                (max_radius - CLEARANCE).abs() < 1e-12,
                "the exact curve-to-axis minimum gives max={CLEARANCE}, got {max_radius}"
            );
        }
        other => panic!("crossing must be a typed radius refusal, got {other:?}"),
    }
    let after = measure::solid_volume(&topo, drilled, 0.01).unwrap();
    assert!(
        (after - before).abs() < 1e-9,
        "the refused chamfer must leave the input untouched"
    );
}
