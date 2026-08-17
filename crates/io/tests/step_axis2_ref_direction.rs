//! `AXIS2_PLACEMENT_3D.ref_direction` is data, not a hint.
//!
//! ISO 10303-42 defines the placement's x axis as
//! `first_proj_axis(z, ref_direction)` — `ref_direction` with its component
//! along `z` removed, then normalized. For a conic that x axis is not
//! cosmetic: it is the ellipse's MAJOR axis, the hyperbola's REAL axis, and
//! the parabola's axis of symmetry, so a reader that re-derives it, or that
//! uses it without projecting, imports a different curve than the file
//! declares.
//!
//! Each test below states the ISO answer for the placement it feeds in, so a
//! reader convention that is internally consistent but wrong cannot pass.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::step::{read_step, write_step};
use remus_math::curves::{Ellipse3D, Hyperbola3D, Parabola3D};
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
    let eid = topo.add_edge(Edge::new(v0, v1, curve));
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

/// Pull the single conic edge curve out of a re-imported solid.
fn only_conic_curve(topo: &Topology, solid: SolidId) -> EdgeCurve {
    let mut found = Vec::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                let c = topo.edge(oe.edge()).unwrap().curve().clone();
                if matches!(
                    c,
                    EdgeCurve::Ellipse(_) | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_)
                ) {
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

/// The entity ids referenced on the right-hand side of an entity line.
///
/// The `#id` that names the entity sits left of the `=`, so it is dropped.
fn rhs_refs(line: &str) -> Vec<u64> {
    let (_, rhs) = line.split_once('=').expect("entity line has no '='");
    rhs.split('#')
        .skip(1)
        .filter_map(|s| {
            let digits: String = s.chars().take_while(char::is_ascii_digit).collect();
            digits.parse().ok()
        })
        .collect()
}

fn entity_line(step: &str, id: u64) -> &str {
    let needle = format!("#{id} =");
    step.lines()
        .find(|l| l.trim_start().starts_with(&needle))
        .unwrap_or_else(|| panic!("no entity #{id} in the emitted file"))
}

/// Rewrite the `ref_direction` of the placement carried by the file's single
/// `<entity>`, leaving every other entity untouched.
///
/// The writer emits a fresh `DIRECTION` per placement — it does not share
/// them — so retargeting one placement's ref_direction cannot disturb
/// another entity's.
fn retarget_ref_direction(step: &str, entity: &str, dir: Vec3) -> String {
    let curve_line = step
        .lines()
        .find(|l| l.contains(&format!("= {entity}(")))
        .unwrap_or_else(|| panic!("writer emitted no {entity} entity"));
    let placement = rhs_refs(curve_line)[0];

    let placement_line = entity_line(step, placement);
    assert!(
        placement_line.contains("AXIS2_PLACEMENT_3D"),
        "{entity} does not reference a placement: {placement_line}"
    );
    let ref_dir_id = rhs_refs(placement_line)[2];

    let old = entity_line(step, ref_dir_id).to_string();
    assert!(old.contains("DIRECTION"), "not a DIRECTION: {old}");
    let new = format!(
        "#{ref_dir_id} = DIRECTION('', ({}, {}, {}));",
        dir.x(),
        dir.y(),
        dir.z()
    );
    step.replace(&old, &new)
}

/// A file's declared major axis is the imported ellipse's major axis.
///
/// The reader used to bind the placement's `ref_direction` to `_u_axis` and
/// call `Ellipse3D::new`, which re-derives an in-plane frame from the normal
/// through `perpendicular_pair` — `x = z × candidate`, which for a Z-up
/// normal is `(0,1,0)`, not ISO's `(1,0,0)`. Every such ellipse imported
/// rotated a quarter turn inside its own plane, swapping which physical
/// direction carried `semi_major`.
///
/// remus's own writer emits the true major axis as `ref_direction`, so
/// this was a lossy round trip of remus's own output, not only of
/// third-party files.
#[test]
fn ellipse_major_axis_follows_the_declared_ref_direction() {
    let src = Ellipse3D::new_with_ref(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        4.0,
        1.0,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    // Not a closed edge: two distinct vertices keep the scaffold's wire
    // valid and the curve's identity is what is under test.
    let (t0, t1) = (0.3, 2.1);

    let mut topo = Topology::new();
    let solid = one_edge_solid(
        &mut topo,
        EdgeCurve::Ellipse(src.clone()),
        src.evaluate(t0),
        src.evaluate(t1),
    );
    let text = write_step(&topo, &[solid]).unwrap();
    assert!(text.contains("ELLIPSE"), "writer emitted no ELLIPSE entity");

    let mut back = Topology::new();
    let solids = read_step(&text, &mut back).unwrap();
    match only_conic_curve(&back, solids[0]) {
        EdgeCurve::Ellipse(got) => {
            assert!(
                (got.semi_major() - 4.0).abs() < 1e-9,
                "semi_major {}",
                got.semi_major()
            );
            // The declared ref_direction, verbatim: it is already unit and
            // already perpendicular to the normal, so ISO's projection is
            // the identity here.
            assert!(
                (got.u_axis() - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-9,
                "major axis {:?}, expected (1,0,0)",
                got.u_axis()
            );
            assert!(
                (got.v_axis() - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-9,
                "minor axis {:?}, expected (0,1,0)",
                got.v_axis()
            );
            // Point-set equality, which a self-consistent but rotated frame
            // cannot fake: the long axis must physically point along x.
            for i in 0..=20 {
                let t = t0 + (t1 - t0) * f64::from(i) / 20.0;
                let p = src.evaluate(t);
                let d = (got.evaluate(got.project(p)) - p).length();
                assert!(d < 1e-9, "sample at t={t} is {d} off the re-imported curve");
            }
        }
        other => panic!("expected Ellipse after round trip, got {other:?}"),
    }
}

/// A ref_direction that is not already perpendicular to the normal is
/// projected, not used raw.
///
/// The reader used to hand the raw direction to `Parabola3D::with_axes` as
/// the symmetry axis. That constructor orthogonalizes its `u_axis` against
/// the axis — it never touches the axis itself — so a tilted `ref_direction`
/// tilted the whole parabola out of its declared plane. (The sibling
/// `Hyperbola3D::with_axes` takes the NORMAL in the same argument slot and
/// does project; the two signatures read alike and mean different things.)
///
/// Here the declared plane normal is `(0,0,1)` and the declared
/// `ref_direction` is `(1,0,1)`, so ISO's x is `(1,0,0)` — which is exactly
/// the parabola the file was written from.
#[test]
fn parabola_symmetry_axis_is_the_projected_ref_direction() {
    let src = Parabola3D::with_axes(
        Point3::new(-3.0, 1.0, 2.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        0.6,
    )
    .unwrap();
    assert!(
        (src.normal() - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-12,
        "fixture normal {:?}",
        src.normal()
    );
    let (t0, t1) = (-1.5, 2.0);

    let mut topo = Topology::new();
    let solid = one_edge_solid(
        &mut topo,
        EdgeCurve::Parabola(src.clone()),
        src.evaluate(t0),
        src.evaluate(t1),
    );
    let written = write_step(&topo, &[solid]).unwrap();
    let text = retarget_ref_direction(&written, "PARABOLA", Vec3::new(1.0, 0.0, 1.0));

    let mut back = Topology::new();
    let solids = read_step(&text, &mut back).unwrap();
    match only_conic_curve(&back, solids[0]) {
        EdgeCurve::Parabola(got) => {
            assert!(
                (got.axis_dir() - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-9,
                "symmetry axis {:?}, expected (1,0,0)",
                got.axis_dir()
            );
            assert!(
                (got.normal().cross(Vec3::new(0.0, 0.0, 1.0))).length() < 1e-9,
                "plane normal {:?}, expected ±(0,0,1)",
                got.normal()
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

/// The same tilt on an ELLIPSE, for the other half of the projection.
///
/// `ref_direction = (1,0,1)` against normal `(0,0,1)` must import with the
/// major axis along `(1,0,0)` — projected and renormalized, not normalized
/// as given.
#[test]
fn ellipse_ref_direction_is_projected_onto_the_plane() {
    let src = Ellipse3D::new_with_ref(
        Point3::new(2.0, -1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        5.0,
        2.0,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let (t0, t1) = (0.2, 1.9);

    let mut topo = Topology::new();
    let solid = one_edge_solid(
        &mut topo,
        EdgeCurve::Ellipse(src.clone()),
        src.evaluate(t0),
        src.evaluate(t1),
    );
    let written = write_step(&topo, &[solid]).unwrap();
    let text = retarget_ref_direction(&written, "ELLIPSE", Vec3::new(1.0, 0.0, 1.0));

    let mut back = Topology::new();
    let solids = read_step(&text, &mut back).unwrap();
    match only_conic_curve(&back, solids[0]) {
        EdgeCurve::Ellipse(got) => {
            assert!(
                (got.u_axis() - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-9,
                "major axis {:?}, expected (1,0,0)",
                got.u_axis()
            );
            for i in 0..=20 {
                let t = t0 + (t1 - t0) * f64::from(i) / 20.0;
                let p = src.evaluate(t);
                let d = (got.evaluate(got.project(p)) - p).length();
                assert!(d < 1e-9, "sample at t={t} is {d} off the re-imported curve");
            }
        }
        other => panic!("expected Ellipse after round trip, got {other:?}"),
    }
}

/// HYPERBOLA already projects, and must keep doing so.
///
/// Its reader arm passes the raw `ref_direction` into
/// `Hyperbola3D::with_axes`, which is correct only because that
/// constructor's second argument is the plane NORMAL and it orthogonalizes
/// `u_axis` against it. This pins that behaviour so the arm is not
/// "made consistent" with the parabola's by copying the wrong shape.
#[test]
fn hyperbola_real_axis_is_the_projected_ref_direction() {
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
    let written = write_step(&topo, &[solid]).unwrap();
    let text = retarget_ref_direction(&written, "HYPERBOLA", Vec3::new(1.0, 0.0, 1.0));

    let mut back = Topology::new();
    let solids = read_step(&text, &mut back).unwrap();
    match only_conic_curve(&back, solids[0]) {
        EdgeCurve::Hyperbola(got) => {
            assert!(
                (got.u_axis() - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-9,
                "real axis {:?}, expected (1,0,0)",
                got.u_axis()
            );
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

/// A `ref_direction` parallel to the axis leaves ISO's `first_proj_axis`
/// undefined, and the reader keeps rejecting such a file rather than
/// inventing a plane for it.
///
/// Stated as a test because it is the one input where the two arms
/// deliberately differ: the ellipse's arbitrary in-plane fallback is
/// harmless (its frame was already arbitrary), while a parabola with no
/// declared plane is not a parabola.
#[test]
fn parabola_rejects_a_ref_direction_parallel_to_the_normal() {
    let src = Parabola3D::with_axes(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        0.6,
    )
    .unwrap();

    let mut topo = Topology::new();
    let solid = one_edge_solid(
        &mut topo,
        EdgeCurve::Parabola(src.clone()),
        src.evaluate(-1.0),
        src.evaluate(1.0),
    );
    let written = write_step(&topo, &[solid]).unwrap();
    // The parabola's plane normal is (0,0,1); so is this ref_direction.
    let text = retarget_ref_direction(&written, "PARABOLA", Vec3::new(0.0, 0.0, 1.0));

    let mut back = Topology::new();
    let err = read_step(&text, &mut back).expect_err("degenerate placement was accepted");
    let msg = err.to_string();
    assert!(
        msg.contains("PARABOLA"),
        "error should name the offending entity, got: {msg}"
    );
}
