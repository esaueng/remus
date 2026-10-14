//! B89: a fillet must never return a face whose outer loop crosses its own hole.
//!
//! Two defects let one through (kernel review 2026-10-14, BLEND-2 and BLEND-3):
//!
//! - **BLEND-2.** `fillet_cascade` fell back to the bare rolling-ball rebuild
//!   after `fillet_v2` refused. The fallback skipped the hole guard and the B71
//!   material guard `fillet_v2` applies, so the plate whose hole sits 1.75 from
//!   the edge — refused at r = 2 by `fillet_v2` — came back `Ok` from the
//!   cascade with the top cap's outer wire (y = 2) crossing its hole (y = 1.75).
//! - **BLEND-3.** The hole-clearance and adjacent-extent guards compared the
//!   clearance with the radius r. On a planar neighbour the band reaches the
//!   setback r·tan(β/2), β the angle between the outward face normals: r on a
//!   right-angle edge, r·√3 on a 60° wedge. A hole 2.5 clear of a 60° ridge
//!   passed the r = 2 test and was crossed by the contact line at 3.46.
//!
//! Every validator stayed silent on both bodies: a wire crossing its own inner
//! loop is still closed, 2-manifold and Euler-consistent. So each case below
//! accepts exactly two outcomes — a typed `cliff-encountered` refusal that
//! leaves the input untouched, or a result whose faces validate, whose planar
//! loops do not cross, and whose hole still classifies as empty — and checks
//! the loop geometry itself rather than trusting the validators.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_check::validate::{Severity, ValidateOptions, validate_solid};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::blend_ops::{self, BlendResult};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::extrude::extrude;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_entity_counts, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

// ── fixtures ────────────────────────────────────────────────────────────────

/// Plate of `regress_plate_hole_fillet`: 80 × 60 × 6.
const PLATE: (f64, f64, f64) = (80.0, 60.0, 6.0);
/// Its hole: r = 2.25 centred 4 from the y = 0 edge, so the rim clears that
/// edge by 1.75.
const PLATE_HOLE_R: f64 = 2.25;
const PLATE_HOLE_Y: f64 = 4.0;

/// The plate drilled straight through at `(PLATE.0 / 2, PLATE_HOLE_Y)`.
fn drilled_plate(topo: &mut Topology) -> SolidId {
    let (w, d, t) = PLATE;
    let body = make_box(topo, w, d, t).unwrap();
    let drill = make_cylinder(topo, PLATE_HOLE_R, t + 4.0).unwrap();
    transform_solid(topo, drill, &Mat4::translation(w / 2.0, PLATE_HOLE_Y, -2.0)).unwrap();
    boolean(topo, BooleanOp::Cut, body, drill).unwrap()
}

/// The plate's straight top edge along x at y = 0.
fn plate_top_front_edge(topo: &Topology, solid: SolidId) -> EdgeId {
    straight_edge(topo, solid, |a, b| {
        a.y().abs() < 1e-9
            && b.y().abs() < 1e-9
            && (a.z() - PLATE.2).abs() < 1e-9
            && (b.z() - PLATE.2).abs() < 1e-9
    })
}

/// Wedge height (extrusion length along z).
const WEDGE_H: f64 = 10.0;
/// Blind hole drilled into the wedge's y = 0 face: radius, centre x, depth.
const WEDGE_HOLE_R: f64 = 1.0;
const WEDGE_HOLE_X: f64 = 3.5;
const WEDGE_HOLE_DEPTH: f64 = 1.5;
/// The hole's clearance from the ridge: 3.5 − 1.0.
const WEDGE_CLEARANCE: f64 = WEDGE_HOLE_X - WEDGE_HOLE_R;

/// A prism over the triangle (0,0) – (`ab`, 0) – `ac`·(cos 60°, sin 60°),
/// extruded `WEDGE_H` along +z. The ridge on the z axis has a 60° interior
/// angle: its outward face normals meet at β = 120°, so a rolling ball of
/// radius r touches each face r·tan(60°) = r·√3 from the ridge.
fn wedge(topo: &mut Topology, ab: f64, ac: f64) -> SolidId {
    prism(topo, std::f64::consts::FRAC_PI_3, ab, ac)
}

/// The prism over (0,0) – (`ab`, 0) – `ac`·(cos θ, sin θ), extruded
/// `WEDGE_H` along +z: a ridge on the z axis with interior angle θ.
fn prism(topo: &mut Topology, theta: f64, ab: f64, ac: f64) -> SolidId {
    let (s, c) = theta.sin_cos();
    let pts = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(ab, 0.0, 0.0),
        Point3::new(ac * c, ac * s, 0.0),
    ];
    let wire = remus_topology::builder::make_polygon_wire(topo, &pts, 1e-7).unwrap();
    let face = remus_topology::builder::make_planar_face_from_wire(topo, wire).unwrap();
    extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), WEDGE_H).unwrap()
}

/// The 60° wedge with a blind hole in its y = 0 face, 2.5 clear of the ridge.
fn holed_wedge(topo: &mut Topology) -> SolidId {
    let body = wedge(topo, 12.0, 12.0);
    drill_blind(topo, body, WEDGE_HOLE_X, WEDGE_HOLE_R, WEDGE_HOLE_DEPTH)
}

/// Drill a blind hole of radius `r` and `depth` into the y = 0 face at
/// `(x, WEDGE_H / 2)`.
fn drill_blind(topo: &mut Topology, body: SolidId, x: f64, r: f64, depth: f64) -> SolidId {
    let drill = make_cylinder(topo, r, depth + 0.5).unwrap();
    // +z → +y, then start the drill 0.5 outside the face.
    let place =
        Mat4::translation(x, -0.5, WEDGE_H / 2.0) * Mat4::rotation_x(-std::f64::consts::FRAC_PI_2);
    transform_solid(topo, drill, &place).unwrap();
    boolean(topo, BooleanOp::Cut, body, drill).unwrap()
}

/// The wedge's ridge: the straight edge on the z axis.
fn ridge(topo: &Topology, solid: SolidId) -> EdgeId {
    straight_edge(topo, solid, |a, b| {
        a.x().abs() < 1e-9 && a.y().abs() < 1e-9 && b.x().abs() < 1e-9 && b.y().abs() < 1e-9
    })
}

fn straight_edge(topo: &Topology, solid: SolidId, want: impl Fn(Point3, Point3) -> bool) -> EdgeId {
    let mut found = Vec::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                let e = topo.edge(oe.edge()).unwrap();
                if !matches!(e.curve(), EdgeCurve::Line) || found.contains(&oe.edge()) {
                    continue;
                }
                let a = topo.vertex(e.start()).unwrap().point();
                let b = topo.vertex(e.end()).unwrap().point();
                if want(a, b) {
                    found.push(oe.edge());
                }
            }
        }
    }
    assert_eq!(found.len(), 1, "expected exactly one matching edge");
    found[0]
}

// ── oracles ─────────────────────────────────────────────────────────────────

/// Whether segments `p0p1` and `q0q1` (2D) intersect, endpoints included,
/// with `eps` slack. Collinear overlaps count as intersections.
fn segments_touch(p0: [f64; 2], p1: [f64; 2], q0: [f64; 2], q1: [f64; 2], eps: f64) -> bool {
    let orient = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]).mul_add(c[1] - a[1], -((b[1] - a[1]) * (c[0] - a[0])))
    };
    let on_segment = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        c[0] >= a[0].min(b[0]) - eps
            && c[0] <= a[0].max(b[0]) + eps
            && c[1] >= a[1].min(b[1]) - eps
            && c[1] <= a[1].max(b[1]) + eps
    };
    let d1 = orient(q0, q1, p0);
    let d2 = orient(q0, q1, p1);
    let d3 = orient(p0, p1, q0);
    let d4 = orient(p0, p1, q1);
    if ((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps))
        && ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps))
    {
        return true;
    }
    (d1.abs() <= eps && on_segment(q0, q1, p0))
        || (d2.abs() <= eps && on_segment(q0, q1, p1))
        || (d3.abs() <= eps && on_segment(p0, p1, q0))
        || (d4.abs() <= eps && on_segment(p0, p1, q1))
}

/// Even-odd point-in-polygon (2D).
fn inside_polygon(p: [f64; 2], poly: &[[f64; 2]]) -> bool {
    let mut inside = false;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[j]);
        if (a[1] > p[1]) != (b[1] > p[1]) {
            let x = (b[0] - a[0]).mul_add((p[1] - a[1]) / (b[1] - a[1]), a[0]);
            if p[0] < x {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

/// Every planar face's inner loops lie strictly inside its outer loop: no
/// segment of an inner loop touches the outer loop, and every inner sample is
/// inside it. This is the property the validators do not check.
fn assert_planar_loops_do_not_cross(topo: &Topology, solid: SolidId, what: &str) {
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        let FaceSurface::Plane { normal, .. } = face.surface() else {
            continue;
        };
        if face.inner_wires().is_empty() {
            continue;
        }
        let n = normal.normalize().unwrap();
        let seed = if n.x().abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let u = n.cross(seed).normalize().unwrap();
        let v = n.cross(u);
        let chart = |wire| -> Vec<[f64; 2]> {
            remus_check::util::wire_polygon_curve_sampled(topo, wire, 128, 32)
                .unwrap()
                .into_iter()
                .map(|p| {
                    let d = p - Point3::new(0.0, 0.0, 0.0);
                    [d.dot(u), d.dot(v)]
                })
                .collect()
        };
        let outer = chart(face.outer_wire());
        for &inner_wire in face.inner_wires() {
            let inner = chart(inner_wire);
            for i in 0..inner.len() {
                let (p0, p1) = (inner[i], inner[(i + 1) % inner.len()]);
                assert!(
                    inside_polygon(p0, &outer),
                    "{what}: an inner-loop point of face {fid:?} lies outside its outer loop"
                );
                for j in 0..outer.len() {
                    let (q0, q1) = (outer[j], outer[(j + 1) % outer.len()]);
                    assert!(
                        !segments_touch(p0, p1, q0, q1, 1e-9),
                        "{what}: the outer loop of face {fid:?} crosses its inner loop near \
                         ({:.4}, {:.4}) in the face chart",
                        p0[0],
                        p0[1],
                    );
                }
            }
        }
    }
}

/// No error-severity issue from the full validator suite, whose per-face
/// checks (surface, orientation, inner-wire winding, wire self-intersection)
/// run on every face of the result.
fn assert_validators_clean(topo: &Topology, solid: SolidId, what: &str) {
    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    let errors: Vec<_> = report
        .issues
        .iter()
        .filter(|issue| issue.severity == Severity::Error)
        .map(|issue| issue.description.clone())
        .collect();
    assert!(errors.is_empty(), "{what}: validator errors {errors:?}");
}

/// Points the result must classify as stated.
struct Probe {
    at: Point3,
    want: PointClassification,
    why: &'static str,
}

/// The two acceptable outcomes: a typed cliff refusal that left the input
/// untouched, or a result that validates, keeps its loops apart, and
/// classifies every probe as stated. Returns whether it was refused.
fn assert_refused_or_sound(
    topo: &Topology,
    input: SolidId,
    input_counts: (usize, usize, usize),
    outcome: &Result<BlendResult, OperationsError>,
    probes: &[Probe],
    what: &str,
) -> bool {
    match outcome {
        Err(err) => {
            assert_eq!(
                blend_ops::blend_failure_code(err),
                "cliff-encountered",
                "{what}: the cause is the support boundary: {err}"
            );
            assert_eq!(
                solid_entity_counts(topo, input).unwrap(),
                input_counts,
                "{what}: a refusal must leave the input untouched"
            );
            true
        }
        Ok(result) => {
            assert_validators_clean(topo, result.solid, what);
            assert_planar_loops_do_not_cross(topo, result.solid, what);
            let opts = ClassifyOptions::default();
            for probe in probes {
                assert_eq!(
                    classify_point(topo, result.solid, probe.at, &opts).unwrap(),
                    probe.want,
                    "{what}: {} at {:?}",
                    probe.why,
                    probe.at
                );
            }
            false
        }
    }
}

fn plate_probes() -> Vec<Probe> {
    let (w, _, t) = PLATE;
    vec![
        Probe {
            at: Point3::new(w / 2.0, PLATE_HOLE_Y, t / 2.0),
            want: PointClassification::Outside,
            why: "the bore stays empty",
        },
        Probe {
            // Inside the bore, under the band the bad cap would have cut.
            at: Point3::new(w / 2.0, PLATE_HOLE_Y - PLATE_HOLE_R + 0.1, t - 0.2),
            want: PointClassification::Outside,
            why: "the bore stays empty under the fillet band",
        },
        Probe {
            at: Point3::new(w / 2.0, 30.0, t / 2.0),
            want: PointClassification::Inside,
            why: "the plate away from the edge stays solid",
        },
    ]
}

fn wedge_probes() -> Vec<Probe> {
    vec![
        Probe {
            at: Point3::new(WEDGE_HOLE_X, WEDGE_HOLE_DEPTH / 2.0, WEDGE_H / 2.0),
            want: PointClassification::Outside,
            why: "the blind hole stays empty",
        },
        Probe {
            // In the hole, on the ridge side, just under the face.
            at: Point3::new(WEDGE_CLEARANCE + 0.2, 0.1, WEDGE_H / 2.0),
            want: PointClassification::Outside,
            why: "the blind hole stays empty under the fillet band",
        },
        Probe {
            at: Point3::new(7.0, 4.0, WEDGE_H / 2.0),
            want: PointClassification::Inside,
            why: "the wedge away from the ridge stays solid",
        },
    ]
}

// ── BLEND-2: the cascade fallback keeps fillet_v2's guards ──────────────────

/// `fillet_v2` refuses r = 2 at the plate hole; the cascade must not then hand
/// the same edge to an unguarded engine.
#[test]
fn cascade_keeps_the_hole_refusal_on_the_drilled_plate() {
    let mut topo = Topology::new();
    let body = drilled_plate(&mut topo);
    let edge = plate_top_front_edge(&topo, body);
    let counts = solid_entity_counts(&topo, body).unwrap();

    // Premise: the walking engine refuses at the cliff.
    let mut direct = topo.clone();
    let err = blend_ops::fillet_v2(&mut direct, body, &[edge], 2.0)
        .err()
        .expect("fillet_v2 must refuse a radius that reaches the hole");
    assert_eq!(blend_ops::blend_failure_code(&err), "cliff-encountered");

    let outcome = blend_ops::fillet_cascade(&mut topo, body, &[edge], 2.0);
    assert_refused_or_sound(
        &topo,
        body,
        counts,
        &outcome,
        &plate_probes(),
        "cascade r=2 on the drilled plate",
    );

    // Below the clearance the cascade still rounds the edge.
    let ok = blend_ops::fillet_cascade(&mut topo, body, &[edge], 1.0)
        .expect("a radius inside the clearance must still work");
    assert_ne!(solid_entity_counts(&topo, ok.solid).unwrap(), counts);
    assert!(!assert_refused_or_sound(
        &topo,
        body,
        counts,
        &Ok(ok),
        &plate_probes(),
        "cascade r=1 on the drilled plate",
    ));
}

/// Every radius between the clearance and the edge's other limits is refused
/// by the cascade, not only the reported r = 2.
#[test]
fn cascade_refuses_every_radius_that_reaches_the_plate_hole() {
    for r in [1.75, 1.8, 2.0, 2.5, 3.0] {
        let mut topo = Topology::new();
        let body = drilled_plate(&mut topo);
        let edge = plate_top_front_edge(&topo, body);
        let counts = solid_entity_counts(&topo, body).unwrap();
        let outcome = blend_ops::fillet_cascade(&mut topo, body, &[edge], r);
        assert_refused_or_sound(
            &topo,
            body,
            counts,
            &outcome,
            &plate_probes(),
            &format!("cascade r={r} on the drilled plate"),
        );
    }
}

// ── BLEND-3: the guards measure the setback, not the radius ─────────────────

/// r = 2 on the 60° ridge sets back 2·√3 = 3.46, past the hole at 2.5.
#[test]
fn ridge_fillet_reaching_a_hole_through_its_setback_is_refused() {
    let r = 2.0;
    let setback = r * std::f64::consts::FRAC_PI_3.tan();
    assert!(r < WEDGE_CLEARANCE && WEDGE_CLEARANCE < setback);

    let mut topo = Topology::new();
    let body = holed_wedge(&mut topo);
    let edge = ridge(&topo, body);
    let counts = solid_entity_counts(&topo, body).unwrap();

    for (name, run) in [
        (
            "fillet_v2",
            blend_ops::fillet_v2
                as fn(&mut Topology, SolidId, &[EdgeId], f64) -> Result<_, OperationsError>,
        ),
        ("fillet_cascade", blend_ops::fillet_cascade),
    ] {
        let mut t = topo.clone();
        let outcome = run(&mut t, body, &[edge], r);
        assert_refused_or_sound(
            &t,
            body,
            counts,
            &outcome,
            &wedge_probes(),
            &format!("{name} r={r} on the holed 60° wedge"),
        );
        if let Err(OperationsError::Blend(remus_blend::BlendError::CliffEncountered {
            available_radius,
            ..
        })) = outcome
        {
            assert!(
                available_radius < r,
                "{name}: the reported achievable radius {available_radius} must be one the \
                 ridge could actually take (setback ≤ clearance means r ≤ {:.4})",
                WEDGE_CLEARANCE / std::f64::consts::FRAC_PI_3.tan()
            );
        }
    }
}

/// A radius whose setback stays inside the clearance still rounds the ridge.
#[test]
fn ridge_fillet_inside_the_setback_clearance_still_works() {
    let r = 1.2; // setback 2.08 < 2.5
    let mut topo = Topology::new();
    let body = holed_wedge(&mut topo);
    let edge = ridge(&topo, body);
    let counts = solid_entity_counts(&topo, body).unwrap();

    for (name, run) in [
        (
            "fillet_v2",
            blend_ops::fillet_v2
                as fn(&mut Topology, SolidId, &[EdgeId], f64) -> Result<_, OperationsError>,
        ),
        ("fillet_cascade", blend_ops::fillet_cascade),
    ] {
        let mut t = topo.clone();
        let result = run(&mut t, body, &[edge], r)
            .unwrap_or_else(|e| panic!("{name}: a setback inside the clearance must work: {e}"));
        assert_ne!(solid_entity_counts(&t, result.solid).unwrap(), counts);
        assert!(!assert_refused_or_sound(
            &t,
            body,
            counts,
            &Ok(result),
            &wedge_probes(),
            &format!("{name} r={r} on the holed 60° wedge"),
        ));
    }
}

/// The adjacent-extent guard: a 60° wedge whose y = 0 face is only 3 deep.
/// r = 2 sets back 3.46 along it, off the face, though r < 3.
#[test]
fn ridge_fillet_whose_setback_overruns_the_adjacent_face_is_refused() {
    let r = 2.0;
    let mut topo = Topology::new();
    let body = wedge(&mut topo, 3.0, 12.0);
    let edge = ridge(&topo, body);
    let counts = solid_entity_counts(&topo, body).unwrap();

    for (name, run) in [
        (
            "fillet_v2",
            blend_ops::fillet_v2
                as fn(&mut Topology, SolidId, &[EdgeId], f64) -> Result<_, OperationsError>,
        ),
        ("fillet_cascade", blend_ops::fillet_cascade),
    ] {
        let mut t = topo.clone();
        let outcome = run(&mut t, body, &[edge], r);
        let refused = assert_refused_or_sound(
            &t,
            body,
            counts,
            &outcome,
            &[Probe {
                at: Point3::new(5.0, 3.0, WEDGE_H / 2.0),
                want: PointClassification::Inside,
                why: "the wedge away from the ridge stays solid",
            }],
            &format!("{name} r={r} on the shallow 60° wedge"),
        );
        assert!(refused, "{name}: the band cannot fit on a 3-deep face");
    }
}

/// The other side of the setback: on a 120° ridge an r = 2 ball touches down
/// 2/√3 = 1.155 out, so a hole 1.5 clear is inside r but outside the band.
/// The radius-based guard refused this; the setback admits it, and the result
/// keeps its hole intact.
#[test]
fn obtuse_ridge_fillet_clears_a_hole_inside_the_radius() {
    let r = 2.0;
    let theta = 2.0 * std::f64::consts::FRAC_PI_3;
    let (hole_r, clearance) = (0.5, 1.5);
    let setback = r / (theta / 2.0).tan();
    assert!(setback < clearance && clearance < r);

    let mut topo = Topology::new();
    let body = prism(&mut topo, theta, 12.0, 12.0);
    let body = drill_blind(&mut topo, body, clearance + hole_r, hole_r, 1.0);
    let edge = ridge(&topo, body);
    let counts = solid_entity_counts(&topo, body).unwrap();
    let probes = [
        Probe {
            at: Point3::new(clearance + hole_r, 0.5, WEDGE_H / 2.0),
            want: PointClassification::Outside,
            why: "the blind hole stays empty",
        },
        Probe {
            at: Point3::new(clearance + 0.1, 0.1, WEDGE_H / 2.0),
            want: PointClassification::Outside,
            why: "the hole's ridge side stays empty",
        },
        Probe {
            at: Point3::new(4.0, 3.0, WEDGE_H / 2.0),
            want: PointClassification::Inside,
            why: "the prism away from the ridge stays solid",
        },
    ];

    for (name, run) in [
        (
            "fillet_v2",
            blend_ops::fillet_v2
                as fn(&mut Topology, SolidId, &[EdgeId], f64) -> Result<_, OperationsError>,
        ),
        ("fillet_cascade", blend_ops::fillet_cascade),
    ] {
        let mut t = topo.clone();
        let result = run(&mut t, body, &[edge], r)
            .unwrap_or_else(|e| panic!("{name}: the band clears the hole: {e}"));
        assert_ne!(solid_entity_counts(&t, result.solid).unwrap(), counts);
        assert!(!assert_refused_or_sound(
            &t,
            body,
            counts,
            &Ok(result),
            &probes,
            &format!("{name} r={r} on the holed 120° ridge"),
        ));
    }
}
