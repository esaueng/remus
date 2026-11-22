//! A closed edge whose pcurve is a periodic NURBS with its seam vertex
//! mid-domain gets the full period, not a zero-length range (#1832).
//!
//! OCCT writes the cap pcurves of a swept or revolved solid as periodic
//! rational B-splines (knot multiplicities `(1,2,2,2,2,1)`) whose domain
//! ends sit half a turn away from the closed edge's vertex. The reader used
//! to project both ends of the use onto that vertex and trim the pcurve to
//! `(π, π)`; the import succeeded, but the STEP writer then refused the
//! zero-length range, so the solid could not be re-exported.
//!
//! The fixture is a hand-written r 8, h 16 cylinder in the same layout: its
//! seam and both cap-circle vertices sit at x = -8 (circle parameter π),
//! and both planar caps carry the OCCT-style periodic B-spline pcurve
//! starting at x = +8. One cap uses its circle forward, the other reversed.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::{PI, TAU};

use remus_io::step::reader::read_step;
use remus_io::step::writer::write_step;
use remus_math::curves2d::Curve2D;
use remus_math::frame::Frame3;
use remus_math::vec::{Point2, Point3};
use remus_operations::measure::solid_volume;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_boundary_authority;

const FIXTURE: &str = include_str!("data/periodic_bspline_cap_cylinder.step");
const VOLUME: f64 = PI * 64.0 * 16.0;

fn import(step: &str) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solids = read_step(step, &mut topo).unwrap();
    assert_eq!(solids.len(), 1);
    (topo, solids[0])
}

/// A pcurve point lifted through its face's chart. Planes store no frame:
/// their pcurves live in the canonical chart the STEP reader and writer
/// share.
fn on_surface(surface: &FaceSurface, uv: Point2) -> Point3 {
    match surface {
        FaceSurface::Plane { normal, d } => {
            let foot = *normal * (*d / normal.dot(*normal));
            let chart =
                Frame3::from_normal(Point3::new(foot.x(), foot.y(), foot.z()), *normal).unwrap();
            chart.origin + chart.x * uv.x() + chart.y * uv.y()
        }
        other => other.evaluate(uv.x(), uv.y()).unwrap(),
    }
}

/// `(planes, cylinders, others)` over the solid's faces.
fn census(topo: &Topology, solid: SolidId) -> (usize, usize, usize) {
    let mut counts = (0, 0, 0);
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => counts.0 += 1,
            FaceSurface::Cylinder(_) => counts.1 += 1,
            _ => counts.2 += 1,
        }
    }
    counts
}

/// Every cap pcurve covers the whole circle in the use's direction: it
/// starts and ends on the vertex, spans a full turn of its own parameter
/// domain, and at each interior fraction lands on the same side of the
/// vertex as the 3D edge use does at that fraction.
fn assert_cap_pcurves_cover_their_circles(topo: &Topology, solid: SolidId, label: &str) {
    let mut caps = 0;
    for face_id in solid_faces(topo, solid).unwrap() {
        let face = topo.face(face_id).unwrap();
        let FaceSurface::Plane { .. } = face.surface() else {
            continue;
        };
        for (edge_id, forward, pcurve) in topo.pcurves_for_face(face_id) {
            let edge = topo.edge(edge_id).unwrap();
            assert_eq!(edge.start(), edge.end(), "{label}: cap edge is closed");
            let EdgeCurve::Circle(_) = edge.curve() else {
                panic!("{label}: cap edge is not a circle");
            };
            let vertex = topo.vertex(edge.start()).unwrap().point();
            let (t0, t1) = (pcurve.t_start(), pcurve.t_end());
            assert!(
                t0.is_finite() && t1.is_finite() && (t1 - t0).abs() > 0.0,
                "{label}: pcurve range ({t0}, {t1}) is empty"
            );
            if let Curve2D::Nurbs(nurbs) = pcurve.curve() {
                let (d0, d1) = nurbs.domain();
                assert!(
                    ((t1 - t0).abs() - (d1 - d0)).abs() <= 1e-9,
                    "{label}: NURBS pcurve range ({t0}, {t1}) is not its full domain ({d0}, {d1})"
                );
            } else {
                assert!(
                    ((t1 - t0).abs() - TAU).abs() <= 1e-9,
                    "{label}: pcurve range ({t0}, {t1}) is not a full turn"
                );
            }
            let lift = |t: f64| on_surface(face.surface(), pcurve.evaluate(t));
            for t in [t0, t1] {
                assert!(
                    (lift(t) - vertex).length() <= 1e-7,
                    "{label}: pcurve end {t} misses the vertex"
                );
            }
            let (e0, e1) = edge.strict_domain().unwrap();
            let (e0, e1) = if forward { (e0, e1) } else { (e1, e0) };
            for fraction in [0.125, 0.25, 0.5, 0.75, 0.875] {
                let on_pcurve = lift((t1 - t0).mul_add(fraction, t0));
                let on_edge = edge.curve().evaluate_with_endpoints(
                    (e1 - e0).mul_add(fraction, e0),
                    vertex,
                    vertex,
                );
                assert!(
                    (on_pcurve.z() - on_edge.z()).abs() <= 1e-9
                        && (on_pcurve.x() * on_pcurve.x() + on_pcurve.y() * on_pcurve.y() - 64.0)
                            .abs()
                            <= 1e-9,
                    "{label}: pcurve sample {on_pcurve:?} is off the cap circle"
                );
                // Same half of the circle as the use at this fraction: the
                // pcurve runs the edge's way, not the opposite one.
                if (fraction - 0.5_f64).abs() > 0.1 {
                    assert!(
                        on_pcurve.y() * on_edge.y() > 0.0,
                        "{label}: at {fraction} the pcurve is at {on_pcurve:?}, the use at {on_edge:?}"
                    );
                } else {
                    assert!(
                        on_pcurve.x() > 7.9 && on_edge.x() > 7.9,
                        "{label}: the half-way point is not opposite the vertex"
                    );
                }
            }
            caps += 1;
        }
    }
    assert_eq!(caps, 2, "{label}: one pcurve per cap");
}

fn assert_cylinder(topo: &Topology, solid: SolidId, label: &str) {
    assert_eq!(census(topo, solid), (2, 1, 0), "{label}");
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {:?}", report.issues);
    validate_boundary_authority(topo).unwrap();
    let volume = solid_volume(topo, solid, 0.001).unwrap();
    assert!(
        (volume - VOLUME).abs() <= VOLUME * 1e-6,
        "{label}: volume {volume} vs {VOLUME}"
    );
}

#[test]
fn periodic_nurbs_cap_pcurve_takes_the_full_period() {
    let (topo, solid) = import(FIXTURE);
    assert_cylinder(&topo, solid, "import");
    assert_cap_pcurves_cover_their_circles(&topo, solid, "import");
}

#[test]
fn imported_periodic_nurbs_caps_round_trip_through_step() {
    let (topo, solid) = import(FIXTURE);
    let written = write_step(&topo, &[solid]).unwrap();
    let (reread, resolid) = import(&written);
    assert_cylinder(&reread, resolid, "re-import");
    assert_cap_pcurves_cover_their_circles(&reread, resolid, "re-import");
    // And once more: the re-imported solid writes and reads back the same.
    let again = write_step(&reread, &[resolid]).unwrap();
    let (third, third_solid) = import(&again);
    assert_cylinder(&third, third_solid, "second re-import");
}

/// The same defect on a periodic surface: an independent writer's cylinder
/// cut by a slanted plane carries its closed ELLIPSE's cylinder pcurve as a
/// B-spline from (0, 12) to (2π, 12), closed only modulo the period. Both
/// ends projected to the domain start and the use read as `(3π/2, 3π/2)`,
/// which the STEP writer refused. The loop now takes the whole domain and
/// the solid round-trips at the file's own volume (942.477796).
#[test]
fn closed_ellipse_loop_round_a_cylinder_round_trips_through_step() {
    const ELLIPSE: &str = include_str!("data/iges_brep/ellipse_edges.step");
    const ELLIPSE_VOLUME: f64 = 942.477_796;
    let (topo, solid) = import(ELLIPSE);
    for face_id in solid_faces(&topo, solid).unwrap() {
        for (edge_id, _, pcurve) in topo.pcurves_for_face(face_id) {
            let edge = topo.edge(edge_id).unwrap();
            if edge.start() == edge.end() {
                let (t0, t1) = (pcurve.t_start(), pcurve.t_end());
                assert!(
                    (t1 - t0).abs() > 1.0,
                    "closed-edge pcurve range ({t0}, {t1})"
                );
            }
        }
    }
    let mut topo = topo;
    let mut solid = solid;
    for label in ["re-import", "second re-import"] {
        let written = write_step(&topo, &[solid]).unwrap();
        (topo, solid) = import(&written);
        let report = validate_solid(&topo, solid).unwrap();
        assert!(report.is_valid(), "{label}: {:?}", report.issues);
        validate_boundary_authority(&topo).unwrap();
        let volume = solid_volume(&topo, solid, 0.001).unwrap();
        assert!(
            (volume - ELLIPSE_VOLUME).abs() <= ELLIPSE_VOLUME * 1e-6,
            "{label}: volume {volume}"
        );
    }
}

/// The cap pcurve's own direction decides the use's, not the coedge sense:
/// with the top cap's control net reversed (the same circle, run the other
/// way from the same start), the reader still stores the loop running the
/// way the edge's use does, and the solid still round-trips.
#[test]
fn reversed_cap_control_net_still_runs_the_edge_way() {
    let reversed = FIXTURE
        .replacen(
            "B_SPLINE_CURVE(2,(#120,#121,#122,#123,#124,#125,",
            "B_SPLINE_CURVE(2,(#126,#125,#124,#123,#122,#121,",
            1,
        )
        .replacen("#126),.UNSPECIFIED.", "#120),.UNSPECIFIED.", 1);
    assert_ne!(reversed, FIXTURE);
    let (topo, solid) = import(&reversed);
    assert_cylinder(&topo, solid, "reversed import");
    assert_cap_pcurves_cover_their_circles(&topo, solid, "reversed import");
    let written = write_step(&topo, &[solid]).unwrap();
    let (reread, resolid) = import(&written);
    assert_cylinder(&reread, resolid, "reversed re-import");
    assert_cap_pcurves_cover_their_circles(&reread, resolid, "reversed re-import");
}
