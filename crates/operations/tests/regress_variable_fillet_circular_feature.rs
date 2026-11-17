//! #1789: a variable-radius fillet on a straight box edge refused whenever the
//! body carried a circular edge anywhere, even far from the rounded edge.
//!
//! A 60 × 60 × 20 box with a boss fused on top, a blind bore, or a through
//! bore: `fillet_variable` on the bottom-front edge failed its postcondition
//! with "shell has 2 connected components (7 of 8 faces reached); shell has
//! 66 free (boundary) edges", while the constant-radius fillet on the same
//! edge built. The plain box and a box with a box-shaped pocket built.
//!
//! The variable engine rebuilds every face from vertex positions, so a loop
//! whose boundary is one closed circle edge (one position) cannot survive
//! the rebuild: the bore or boss rim fell out of the cap that carries it and
//! the cylinder wall that is bounded by it, and the boss or bore came back
//! as a separate, open component. Faces the vocabulary cannot express are
//! now copied whole, as the rolling-ball engine does.
//!
//! Acceptance: each body takes the round, validates clean, is watertight as
//! a B-Rep and as a mesh, keeps its cylinder wall and circle rims exact, and
//! measures the closed form: the body's volume less `(1 - π/4) ∫ r² ds`
//! along the 60-long edge.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_4, PI};

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::fillet::{FilletEdgeSetback, FilletRadiusLaw, fillet_variable_with_setbacks};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid_with_tolerance,
};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const SIDE: f64 = 60.0;
const HEIGHT: f64 = 20.0;
const BORE: f64 = 8.0;

#[derive(Clone, Copy, Debug)]
enum Feature {
    Boss,
    BlindBore,
    ThroughBore,
}

impl Feature {
    /// The body's volume before the round.
    fn body_volume(self) -> f64 {
        let slab = SIDE * SIDE * HEIGHT;
        match self {
            Self::Boss => BORE.mul_add(BORE * PI * 10.0, slab),
            Self::BlindBore => BORE.mul_add(-BORE * PI * 10.0, slab),
            Self::ThroughBore => BORE.mul_add(-BORE * PI * HEIGHT, slab),
        }
    }
}

fn body(topo: &mut Topology, feature: Feature) -> SolidId {
    let slab = make_box(topo, SIDE, SIDE, HEIGHT).unwrap();
    let (height, z, op) = match feature {
        Feature::Boss => (20.0, 10.0, BooleanOp::Fuse),
        Feature::BlindBore => (20.0, 10.0, BooleanOp::Cut),
        Feature::ThroughBore => (40.0, -10.0, BooleanOp::Cut),
    };
    let cylinder = make_cylinder(topo, BORE, height).unwrap();
    transform_solid(topo, cylinder, &Mat4::translation(30.0, 30.0, z)).unwrap();
    boolean(topo, op, slab, cylinder).unwrap()
}

/// The box edge along x at y = 0, z = 0.
fn bottom_front(topo: &Topology, solid: SolidId) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&id| {
            let edge = topo.edge(id).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            matches!(edge.curve(), EdgeCurve::Line)
                && [a, b]
                    .iter()
                    .all(|p| p.y().abs() < 1e-9 && p.z().abs() < 1e-9)
        })
        .expect("bottom-front edge")
}

/// `∫₀¹ r(t)² dt` by composite Simpson; exact for these cubic-or-lower laws.
fn mean_square_radius(law: &FilletRadiusLaw) -> f64 {
    let n = 64_u32;
    let h = 1.0 / f64::from(n);
    let mut sum = 0.0;
    for i in 0..=n {
        let r = law.evaluate(f64::from(i) * h);
        let weight = if i == 0 || i == n {
            1.0
        } else if i % 2 == 1 {
            4.0
        } else {
            2.0
        };
        sum += weight * r * r;
    }
    sum * h / 3.0
}

fn curved_census(topo: &Topology, solid: SolidId) -> (usize, usize) {
    let cylinders = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .count();
    let circles = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&e| matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Circle(_)))
        .count();
    (cylinders, circles)
}

fn check(feature: Feature, law: FilletRadiusLaw) {
    let label = format!("{feature:?} {law:?}");
    let mut topo = Topology::new();
    let input = body(&mut topo, feature);
    let census_before = curved_census(&topo, input);
    assert!(
        census_before.0 >= 1 && census_before.1 >= 2,
        "{label}: {census_before:?}"
    );
    let edge = bottom_front(&topo, input);
    let result = fillet_variable_with_setbacks(
        &mut topo,
        input,
        &[FilletEdgeSetback {
            edge,
            law: law.clone(),
            start_setback: 0.0,
            end_setback: 0.0,
        }],
    )
    .unwrap_or_else(|e| panic!("{label}: {e}"));
    assert_ne!(result, input, "{label}: returned the input handle");

    let issues = validate_solid(&topo, result).unwrap();
    assert!(issues.is_valid(), "{label}: {issues:?}");
    let shell = topo
        .shell(topo.solid(result).unwrap().outer_shell())
        .unwrap();
    validate_shell_closed(shell, &topo).unwrap_or_else(|e| panic!("{label}: {e}"));

    // The circular feature survives exactly: same cylinder wall and rims.
    assert_eq!(curved_census(&topo, result), census_before, "{label}");
    assert_eq!(
        solid_faces(&topo, result).unwrap().len(),
        solid_faces(&topo, input).unwrap().len() + 1,
        "{label}: one blend face added"
    );

    let removed = (1.0 - FRAC_PI_4) * SIDE * mean_square_radius(&law);
    let expected = feature.body_volume() - removed;
    let gauss = mass_properties(&topo, result).unwrap().mass;
    assert!(
        (gauss - expected).abs() < 1e-6 * expected,
        "{label}: Gauss {gauss:.6} against the closed form {expected:.6}"
    );
    for deflection in [0.002, 0.001] {
        let mesh = tessellate_solid_with_tolerance(&topo, result, deflection, 0.1).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "{label}");
        assert_eq!(non_manifold_edge_count(&mesh), 0, "{label}");
        let volume = solid_volume(&topo, result, deflection).unwrap();
        assert!(
            (volume - expected).abs() < 1e-4 * expected,
            "{label}: mesh {volume:.6} at {deflection} against {expected:.6}"
        );
    }

    // Ray-cast probes: the rounded-off corner is gone, the feature is intact.
    let options = ClassifyOptions::default();
    let class = |p: Point3| classify_point(&topo, result, p, &options).unwrap();
    let r_mid = law.evaluate(0.5);
    let corner = Point3::new(30.0, 0.1 * r_mid, 0.1 * r_mid);
    assert_eq!(
        class(corner),
        PointClassification::Outside,
        "{label}: corner"
    );
    let kept = Point3::new(30.0, 1.5 * r_mid, 1.5 * r_mid);
    assert_eq!(class(kept), PointClassification::Inside, "{label}: kept");
    let axis = Point3::new(30.0, 30.0, 15.0);
    let in_feature = match feature {
        Feature::Boss => PointClassification::Inside,
        Feature::BlindBore | Feature::ThroughBore => PointClassification::Outside,
    };
    assert_eq!(class(axis), in_feature, "{label}: feature axis");
    let wall = Point3::new(30.0 + BORE + 1.0, 30.0, 15.0);
    let at_wall = match feature {
        Feature::Boss => PointClassification::Outside,
        Feature::BlindBore | Feature::ThroughBore => PointClassification::Inside,
    };
    let at_wall_z = if matches!(feature, Feature::Boss) {
        25.0
    } else {
        15.0
    };
    assert_eq!(
        class(Point3::new(wall.x(), wall.y(), at_wall_z)),
        at_wall,
        "{label}: beside the feature"
    );
}

fn laws() -> [FilletRadiusLaw; 2] {
    [
        FilletRadiusLaw::Linear {
            start: 1.0,
            end: 2.0,
        },
        FilletRadiusLaw::SCurve {
            start: 2.0,
            end: 1.0,
        },
    ]
}

#[test]
fn variable_fillet_on_a_box_edge_survives_a_fused_boss() {
    for law in laws() {
        check(Feature::Boss, law);
    }
}

#[test]
fn variable_fillet_on_a_box_edge_survives_a_blind_bore() {
    for law in laws() {
        check(Feature::BlindBore, law);
    }
}

#[test]
fn variable_fillet_on_a_box_edge_survives_a_through_bore() {
    // The through bore's rim lies in the bottom face, which the round trims.
    for law in laws() {
        check(Feature::ThroughBore, law);
    }
}
