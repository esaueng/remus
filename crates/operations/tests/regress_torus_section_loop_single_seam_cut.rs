//! A planar cut across a revolved tube, whose section is a closed loop round
//! the tube crossing the tube's seam once.
//!
//! The torus face's chart partition cuts that loop at its one seam crossing,
//! so the torus side carries it as one piece from the crossing round to
//! itself, while the planar face keeps it as a closed loop through its own
//! start vertex. The builder's global refinement split a closed NURBS loop
//! only at two or more vertices, so the two sides never shared the edges and
//! the exact boolean refused. A closed loop now also takes a single cut
//! where another edge on the same curve ends (the conic rule's mate
//! partition), and the cut is exact.
#![allow(clippy::unwrap_used, clippy::panic, missing_docs)]

use remus_math::curves::Circle3D;
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

/// The `R = 10`, `r = 2` tube swept 200° about `z`, its profile circle's
/// seam at angle `phase` in the profile plane.
fn tube(topo: &mut Topology, phase: f64) -> SolidId {
    let normal = Vec3::new(0.0, 1.0, 0.0);
    let circle = Circle3D::new_with_ref(
        Point3::new(10.0, 0.0, 0.0),
        normal,
        2.0,
        Vec3::new(phase.cos(), 0.0, phase.sin()),
    )
    .unwrap();
    let v = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let mut e = Edge::new(v, v, EdgeCurve::Circle(circle));
    e.set_trim(Some((0.0, std::f64::consts::TAU)));
    let e = topo.add_edge(e);
    let w = topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap());
    let f = topo.add_face(Face::new(
        w,
        Vec::new(),
        FaceSurface::Plane { normal, d: 0.0 },
    ));
    remus_operations::revolve::revolve(
        topo,
        f,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        200f64.to_radians(),
    )
    .unwrap()
}

/// Cut the tube by the half-space `x ≥ x0` turned `tilt` degrees about `y`,
/// and check the result is an exact, closed, valid solid of the reference
/// volume.
fn check(phase: f64, x0: f64, tilt: f64, reference: f64) {
    let mut topo = Topology::new();
    let t = tube(&mut topo, phase);
    let b = make_box(&mut topo, 40.0, 40.0, 40.0).unwrap();
    transform_solid(&mut topo, b, &Mat4::translation(x0, -20.0, -20.0)).unwrap();
    transform_solid(&mut topo, b, &Mat4::rotation_y(tilt.to_radians())).unwrap();
    let scope = remus_algo::gfa::SectionDropScope::enter();
    let what = format!("phase {phase} x0 {x0} tilt {tilt}");
    let s = boolean(&mut topo, BooleanOp::Cut, t, b)
        .unwrap_or_else(|e| panic!("{what}: refused: {e:?}"));
    assert_eq!(scope.dropped(), 0, "{what}");
    assert_eq!(scope.uncertified_charts(), 0, "{what}");
    assert_eq!(scope.uncertified_rescues(), 0, "{what}");
    let faces = remus_topology::explorer::solid_faces(&topo, s).unwrap();
    let mut uses = std::collections::BTreeMap::<usize, usize>::new();
    let mut tori = 0;
    for &f in &faces {
        let face = topo.face(f).unwrap();
        tori += usize::from(face.surface().type_tag() == "torus");
        for w in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(w).unwrap().edges() {
                *uses.entry(oe.edge().index()).or_default() += 1;
            }
        }
    }
    assert_eq!(
        (faces.len(), tori),
        (3, 1),
        "{what}: torus, end cap, cut face"
    );
    assert!(uses.values().all(|&c| c == 2), "{what}: {uses:?}");
    let report = remus_check::validate::validate_solid(
        &topo,
        s,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(
        report
            .issues
            .iter()
            .all(|i| i.severity != remus_check::validate::Severity::Error),
        "{what}: {:?}",
        report.issues
    );
    for (point, expected) in [
        (
            Point3::new(-10.0, 0.0, 0.0),
            remus_operations::classify::PointClassification::Inside,
        ),
        (
            Point3::new(10.0 * 0.1_f64.cos(), 10.0 * 0.1_f64.sin(), 0.0),
            remus_operations::classify::PointClassification::Outside,
        ),
        (
            Point3::new(0.0, 0.0, 0.0),
            remus_operations::classify::PointClassification::Outside,
        ),
    ] {
        assert_eq!(
            remus_operations::classify::classify_point(&topo, s, point, 0.005, 1e-7).unwrap(),
            expected,
            "{what}: material at {point:?}"
        );
    }
    // Signed tetrahedron volume is meaningful only for a closed, oriented
    // mesh. Check that first, then extrapolate its linear deflection error.
    let mesh_volume = |deflection| {
        let mesh = remus_operations::tessellate::tessellate_solid(&topo, s, deflection).unwrap();
        let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
        assert!(quality.is_watertight(), "{what}: {quality:?}");
        assert_eq!(quality.euler_characteristic, 2, "{what}");
        assert_eq!(
            remus_operations::tessellate::boundary_edge_count(&mesh),
            0,
            "{what}: inconsistent triangle winding"
        );
        let corner = |i: u32| {
            let p = mesh.positions[i as usize];
            Vec3::new(p.x(), p.y(), p.z())
        };
        mesh.indices
            .chunks_exact(3)
            .map(|t| corner(t[0]).dot(corner(t[1]).cross(corner(t[2]))) / 6.0)
            .sum::<f64>()
    };
    let (coarse, fine) = (mesh_volume(1e-3), mesh_volume(1e-4));
    let volume = fine + (fine - coarse) / 9.0;
    assert!(
        (volume - reference).abs() < 2e-3,
        "{what}: volume {volume} against {reference}"
    );
}

#[test]
fn a_straight_cut_across_the_tube_is_exact() {
    // Closed form: ∫ ρ (U − acos(3/ρ)) dA over the profile disc.
    for phase in [0.0, 1.0, 2.0] {
        check(phase, 3.0, 0.0, 279.566_144);
    }
}

#[test]
fn a_tilted_cut_with_the_seam_elsewhere_is_exact() {
    // The same integral with the tilted half-space (fine quadrature).
    for phase in [0.0, 1.0, 2.0] {
        check(phase, -4.0, 15.0, 187.515_139);
    }
}
