//! STEP round trip for the unbounded-conic edge curves.
//!
//! ISO 10303-42 has native `HYPERBOLA` and `PARABOLA` entities, so these
//! edges survive export/import exactly rather than degrading to a spline
//! or a chord. The checks below compare the re-imported curve against the
//! parameters the original was BUILT from, and — more strongly — against
//! sampled points on the original curve, so a placement convention that is
//! self-consistent but wrong (e.g. STEP's parabola parameter differs from
//! remus's by a factor of 2f) cannot pass.

#![allow(clippy::unwrap_used, clippy::panic)]

use remus_io::step::{read_step, write_step};
use remus_math::curves::{Hyperbola3D, Parabola3D};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

/// Minimal single-face scaffold carrying one conic edge, enough for the
/// STEP writer to emit an `EDGE_CURVE` for it.
fn one_edge_solid(topo: &mut Topology, curve: EdgeCurve, s: Point3, e: Point3) -> SolidId {
    let v0 = topo.add_vertex(Vertex::new(s, 1e-7));
    let v1 = topo.add_vertex(Vertex::new(e, 1e-7));
    let domain = match &curve {
        EdgeCurve::Hyperbola(curve) => (curve.project(s), curve.project(e)),
        EdgeCurve::Parabola(curve) => (curve.project(s), curve.project(e)),
        other => panic!("test scaffold needs an unbounded conic, got {other:?}"),
    };
    let mut edge = Edge::new(v0, v1, curve);
    edge.set_trim(Some(domain));
    let eid = topo.add_edge(edge);
    let wid = topo.add_wire(Wire::new(vec![OrientedEdge::new(eid, true)], false).unwrap());
    let fid = topo.add_face(Face::new(
        wid,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    let sid = topo.add_shell(Shell::new(vec![fid]).unwrap());
    topo.add_solid(Solid::new(sid, vec![]))
}

/// Pull the single non-line edge curve out of a re-imported solid.
fn only_conic_curve(topo: &Topology, solid: SolidId) -> EdgeCurve {
    let mut found = Vec::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                let c = topo.edge(oe.edge()).unwrap().curve().clone();
                if matches!(c, EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_)) {
                    found.push(c);
                }
            }
        }
    }
    assert_eq!(
        found.len(),
        1,
        "expected exactly one conic edge after round trip, got {}",
        found.len()
    );
    found.pop().unwrap()
}

#[test]
fn hyperbola_survives_a_step_round_trip() {
    let src = Hyperbola3D::with_axes(
        Point3::new(1.0, -2.0, 0.5),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        2.0,
        1.5,
    )
    .unwrap();
    let (t0, t1) = (-0.9, 1.1);

    let mut topo = Topology::new();
    let solid = one_edge_solid(
        &mut topo,
        EdgeCurve::Hyperbola(src.clone()),
        src.evaluate(t0),
        src.evaluate(t1),
    );
    let text = write_step(&topo, &[solid]).unwrap();
    assert!(
        text.contains("HYPERBOLA"),
        "writer emitted no HYPERBOLA entity"
    );

    let mut back = Topology::new();
    let solids = read_step(&text, &mut back).unwrap();
    match only_conic_curve(&back, solids[0]) {
        EdgeCurve::Hyperbola(got) => {
            assert!(
                (got.semi_major() - 2.0).abs() < 1e-9,
                "{}",
                got.semi_major()
            );
            assert!(
                (got.semi_minor() - 1.5).abs() < 1e-9,
                "{}",
                got.semi_minor()
            );
            // Point-set equality is the real check: sample the ORIGINAL and
            // require every sample to lie on the re-imported curve.
            for i in 0..=20 {
                let t = t0 + (t1 - t0) * f64::from(i) / 20.0;
                let p = src.evaluate(t);
                let d = (got.evaluate(got.project(p)) - p).length();
                assert!(d < 1e-9, "sample at t={t} is {d} off the re-imported curve");
            }
        }
        other => panic!("expected Hyperbola after round trip, got {other:?}"),
    }
}

#[test]
fn parabola_survives_a_step_round_trip() {
    // STEP parameterizes a parabola as `V + f*u^2*x + 2f*u*y` while remus
    // uses `V + (t^2/4f)*axis + t*u_axis`; the two differ by `t = 2f*u`.
    // Sampling the point SET below is what catches a mistake in that
    // conversion — comparing focal lengths alone would not.
    let src = Parabola3D::with_axes(
        Point3::new(-3.0, 1.0, 2.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        0.6,
    )
    .unwrap();
    let (t0, t1) = (-1.5, 2.0);

    let mut topo = Topology::new();
    let solid = one_edge_solid(
        &mut topo,
        EdgeCurve::Parabola(src.clone()),
        src.evaluate(t0),
        src.evaluate(t1),
    );
    let text = write_step(&topo, &[solid]).unwrap();
    assert!(
        text.contains("PARABOLA"),
        "writer emitted no PARABOLA entity"
    );

    let mut back = Topology::new();
    let solids = read_step(&text, &mut back).unwrap();
    match only_conic_curve(&back, solids[0]) {
        EdgeCurve::Parabola(got) => {
            assert!(
                (got.focal_length() - 0.6).abs() < 1e-9,
                "focal length {}",
                got.focal_length()
            );
            // The PLANE must survive too — it is the piece `Parabola3D::new`
            // cannot carry.
            assert!(
                got.normal().cross(src.normal()).length() < 1e-9,
                "plane normal {:?} vs {:?}",
                got.normal(),
                src.normal()
            );
            for i in 0..=20 {
                let t = t0 + (t1 - t0) * f64::from(i) / 20.0;
                let p = src.evaluate(t);
                let d = (got.evaluate(got.project(p)) - p).length();
                assert!(d < 1e-9, "sample at t={t} is {d} off the re-imported curve");
            }
        }
        other => panic!("expected Parabola after round trip, got {other:?}"),
    }
}
