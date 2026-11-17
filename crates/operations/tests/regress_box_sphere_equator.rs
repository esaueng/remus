//! Regression (#1790): a box boolean with a sphere whose equator lies in a
//! box face.
//!
//! `make_sphere` builds two hemisphere faces joined along the equator. With
//! the equator in a 60×60 box face, that face meets each hemisphere along
//! the whole equator, so it received the same closed section circle twice,
//! once from each hemisphere. The face splitter cut the same hole twice and
//! the face carried two copies of one inner loop; copying it out of the
//! builder failed ("copied face … does not contain exactly one reverse use
//! of edge …"), and the cut and fuse fell to a typed exact-only refusal.
//!
//! The intersect (one hemisphere closed by the equatorial disc) built, but
//! its outward test read the hemisphere face as contributing nothing: the
//! flux of a hemisphere bounded by its equator was computed only for a
//! faceted (straight-chord) equator, and `make_sphere` bounds it by exact
//! great-circle arcs. The closing disc alone then decided the shell's sense,
//! so away from the origin (32 segments, box above the equator) the shell
//! read inward and the intersect refused.
//!
//! Every result is checked against closed forms (box ± hemisphere,
//! hemisphere) and by the ladder of the solid-verification skill: face
//! census and exact carriers, validation, B-Rep and mesh closure, ray-cast
//! probes, and volume.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::{make_box, make_sphere};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const R: f64 = 10.0;
const SIDE: f64 = 60.0;
const HEIGHT: f64 = 20.0;
/// The plane holding both the sphere's equator and one box face.
const EQUATOR_Z: f64 = 20.0;

/// Which side of the equator plane the box's material lies on.
#[derive(Clone, Copy, Debug)]
enum BoxSide {
    /// The box spans z = 0..20: its top face holds the equator (the issue's
    /// geometry).
    Below,
    /// The box spans z = 20..40: its bottom face holds the equator.
    Above,
}

impl BoxSide {
    /// Unit step from the equator plane into the box.
    const fn into_box(self) -> f64 {
        match self {
            Self::Below => -1.0,
            Self::Above => 1.0,
        }
    }

    const fn floor(self) -> f64 {
        match self {
            Self::Below => EQUATOR_Z - HEIGHT,
            Self::Above => EQUATOR_Z,
        }
    }
}

fn box_sphere(op: BooleanOp, side: BoxSide, segments: usize) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, SIDE, SIDE, HEIGHT).unwrap();
    transform_solid(&mut topo, block, &Mat4::translation(0.0, 0.0, side.floor())).unwrap();
    let sphere = make_sphere(&mut topo, R, segments).unwrap();
    transform_solid(
        &mut topo,
        sphere,
        &Mat4::translation(SIDE / 2.0, SIDE / 2.0, EQUATOR_Z),
    )
    .unwrap();
    let body = boolean(&mut topo, op, block, sphere)
        .unwrap_or_else(|e| panic!("{op:?}, box {side:?}, {segments} segments: {e}"));
    (topo, body)
}

fn assert_rel(got: f64, want: f64, what: &str) {
    assert!(
        (got - want).abs() <= 1e-9 * want.abs().max(1.0),
        "{what}: {got} vs closed form {want}"
    );
}

/// Each result is exact: the right faces on the input carriers, closed and
/// valid, material where intended, and the closed-form volume.
#[test]
fn equator_in_a_box_face_builds_every_boolean_exactly() {
    let box_volume = SIDE * SIDE * HEIGHT;
    let hemisphere = 2.0 * PI * R.powi(3) / 3.0;
    for segments in [4, 16, 32] {
        for side in [BoxSide::Below, BoxSide::Above] {
            let step = side.into_box();
            let at =
                |depth: f64| Point3::new(SIDE / 2.0, SIDE / 2.0, depth.mul_add(step, EQUATOR_Z));
            // Probes on the box axis: the hemisphere inside the box, the one
            // outside it, the box past the sphere, and outside both.
            let probes = [at(5.0), at(-5.0), at(15.0), at(-15.0)];
            for (op, volume, planes, inside) in [
                (
                    BooleanOp::Cut,
                    box_volume - hemisphere,
                    6,
                    [false, false, true, false],
                ),
                (
                    BooleanOp::Fuse,
                    box_volume + hemisphere,
                    6,
                    [true, true, true, false],
                ),
                (
                    BooleanOp::Intersect,
                    hemisphere,
                    1,
                    [true, false, false, false],
                ),
            ] {
                let what = format!("{op:?}, box {side:?}, {segments} segments");
                let (topo, body) = box_sphere(op, side, segments);

                // Census: the box's planes and ONE hemisphere on the input
                // sphere (the face lying on the box's side for a cut or
                // intersect, the other for a fuse).
                let (mut spheres, mut plane_faces) = (0, 0);
                for f in solid_faces(&topo, body).unwrap() {
                    match topo.face(f).unwrap().surface() {
                        FaceSurface::Sphere(s) => {
                            assert_rel(s.radius(), R, &format!("{what} sphere radius"));
                            let c = s.center();
                            assert_rel(c.x(), SIDE / 2.0, &format!("{what} sphere x"));
                            assert_rel(c.y(), SIDE / 2.0, &format!("{what} sphere y"));
                            assert_rel(c.z(), EQUATOR_Z, &format!("{what} sphere z"));
                            spheres += 1;
                        }
                        FaceSurface::Plane { .. } => plane_faces += 1,
                        other => panic!("{what}: unexpected {} face", other.type_tag()),
                    }
                }
                assert_eq!((spheres, plane_faces), (1, planes), "{what}: face census");

                let report = validate_solid(&topo, body).unwrap();
                assert!(report.is_valid(), "{what}: {:#?}", report.issues);
                let mesh = tessellate_solid(&topo, body, 0.05).unwrap();
                assert_eq!(boundary_edge_count(&mesh), 0, "{what}: open mesh");
                assert_eq!(
                    non_manifold_edge_count(&mesh),
                    0,
                    "{what}: non-manifold mesh"
                );

                for (p, want_inside) in probes.iter().zip(inside) {
                    let got = classify_point(&topo, body, *p, &ClassifyOptions::default()).unwrap();
                    let want = if want_inside {
                        PointClassification::Inside
                    } else {
                        PointClassification::Outside
                    };
                    assert_eq!(got, want, "{what}: probe at z = {}", p.z());
                }

                assert_rel(
                    mass_properties(&topo, body).unwrap().mass,
                    volume,
                    &format!("{what} mass_properties"),
                );
                for deflection in [1e-3, 5e-4] {
                    assert_rel(
                        solid_volume(&topo, body, deflection).unwrap(),
                        volume,
                        &format!("{what} solid_volume at {deflection}"),
                    );
                }
            }
        }
    }
}
