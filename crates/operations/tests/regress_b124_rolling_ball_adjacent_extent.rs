//! B124: the rolling-ball adjacent-extent guard measures the setback across
//! the face's perpendicular extent.
//!
//! Phase 2b of `fillet_rolling_ball_with_origins` refused a fillet only when
//! the radius reached the length of the adjacent edge on a support face. The
//! band's contact line runs one setback from the edge, `r·tan(θ/2)` between
//! planes (θ between the outward normals), and has to land on that adjacent
//! edge, so what bounds it is how far the adjacent edge reaches away from the
//! filleted edge's line. On a 60° wedge whose face is 3 deep an r = 2 band sets
//! back 3.46, off the face, though r < 3; the old guard let it through to a
//! postcondition failure reported as `trimming-failure`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::blend_ops;
use remus_operations::extrude::extrude;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_entity_counts, solid_faces};
use remus_topology::solid::SolidId;

const HEIGHT: f64 = 10.0;

/// The prism over (0,0) – (`ab`, 0) – `ac`·(cos θ, sin θ), extruded `HEIGHT`
/// along `direction`: a ridge from the origin with interior angle θ.
fn prism(topo: &mut Topology, theta: f64, ab: f64, ac: f64, direction: Vec3) -> SolidId {
    let (s, c) = theta.sin_cos();
    let pts = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(ab, 0.0, 0.0),
        Point3::new(ac * c, ac * s, 0.0),
    ];
    let wire = remus_topology::builder::make_polygon_wire(topo, &pts, 1e-7).unwrap();
    let face = remus_topology::builder::make_planar_face_from_wire(topo, wire).unwrap();
    extrude(topo, face, direction, HEIGHT).unwrap()
}

/// The straight edge leaving the origin that is not in the z = 0 profile.
fn ridge(topo: &Topology, solid: SolidId) -> EdgeId {
    let mut found = Vec::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for oe in topo.wire(face.outer_wire()).unwrap().edges() {
            let e = topo.edge(oe.edge()).unwrap();
            if !matches!(e.curve(), EdgeCurve::Line) || found.contains(&oe.edge()) {
                continue;
            }
            let a = topo.vertex(e.start()).unwrap().point();
            let b = topo.vertex(e.end()).unwrap().point();
            let at_origin =
                |p: Point3| p.x().abs() < 1e-9 && p.y().abs() < 1e-9 && p.z().abs() < 1e-9;
            if (at_origin(a) && b.z() > 1.0) || (at_origin(b) && a.z() > 1.0) {
                found.push(oe.edge());
            }
        }
    }
    assert_eq!(found.len(), 1, "expected exactly one ridge edge");
    found[0]
}

type Fillet =
    fn(&mut Topology, SolidId, &[EdgeId], f64) -> Result<blend_ops::BlendResult, OperationsError>;

fn engines() -> [(&'static str, Fillet); 2] {
    [
        ("fillet_v2", blend_ops::fillet_v2),
        ("fillet_cascade", blend_ops::fillet_cascade),
    ]
}

/// The call is refused at the cliff, reports a radius the face can take, and
/// leaves the input untouched.
fn assert_cliff(
    topo: &Topology,
    body: SolidId,
    outcome: Result<blend_ops::BlendResult, OperationsError>,
    r: f64,
    what: &str,
) {
    let counts = solid_entity_counts(topo, body).unwrap();
    let err = outcome
        .err()
        .unwrap_or_else(|| panic!("{what}: the band cannot fit on the face"));
    assert_eq!(
        blend_ops::blend_failure_code(&err),
        "cliff-encountered",
        "{what}: the cause is the support boundary: {err}"
    );
    if let OperationsError::Blend(remus_blend::BlendError::CliffEncountered {
        available_radius,
        ..
    }) = err
    {
        assert!(
            available_radius > 0.0 && available_radius < r,
            "{what}: achievable radius {available_radius} must be positive and below {r}"
        );
    }
    assert_eq!(solid_entity_counts(topo, body).unwrap(), counts);
}

/// A 60° wedge whose y = 0 face is only 3 deep: r = 2 sets back 2·√3 = 3.46
/// along it, off the face, though r < 3.
#[test]
fn acute_ridge_whose_setback_overruns_the_adjacent_face_is_refused() {
    let r = 2.0;
    let theta = std::f64::consts::FRAC_PI_3;
    let setback = r / (theta / 2.0).tan();
    assert!(r < 3.0 && 3.0 < setback);

    let mut topo = Topology::new();
    let body = prism(&mut topo, theta, 3.0, 12.0, Vec3::new(0.0, 0.0, 1.0));
    let edge = ridge(&topo, body);
    for (name, run) in engines() {
        let mut t = topo.clone();
        let outcome = run(&mut t, body, &[edge], r);
        assert_cliff(
            &t,
            body,
            outcome,
            r,
            &format!("{name} r={r} on the shallow 60° wedge"),
        );
    }
}

/// The same wedge rounds when the setback stays on the face: r = 1.5 sets
/// back 2.6 < 3.
#[test]
fn acute_ridge_whose_setback_stays_on_the_face_still_rounds() {
    let r = 1.5;
    let mut topo = Topology::new();
    let body = prism(
        &mut topo,
        std::f64::consts::FRAC_PI_3,
        3.0,
        12.0,
        Vec3::new(0.0, 0.0, 1.0),
    );
    let edge = ridge(&topo, body);
    let counts = solid_entity_counts(&topo, body).unwrap();
    for (name, run) in engines() {
        let mut t = topo.clone();
        let result = run(&mut t, body, &[edge], r)
            .unwrap_or_else(|e| panic!("{name}: a setback inside the face must round: {e}"));
        assert_ne!(solid_entity_counts(&t, result.solid).unwrap(), counts);
        assert_eq!(
            classify_point(
                &t,
                result.solid,
                Point3::new(0.4, 0.4, HEIGHT / 2.0),
                &ClassifyOptions::default()
            )
            .unwrap(),
            PointClassification::Outside,
            "{name}: the ridge tip is rounded away"
        );
    }
}

/// The other side of the setback: on a 110° ridge an r = 2 ball touches the
/// faces 2/tan(55°) = 1.40 out, so a face 1.7 deep holds the band although it
/// is shorter than r. The rebuild classifies this ridge and uses its exact
/// contacts, so the guard must not fall back to the radius; the old
/// radius-against-length bound refused it.
#[test]
fn obtuse_ridge_whose_setback_fits_a_face_shorter_than_r_still_rounds() {
    let r = 2.0;
    let theta = 110.0_f64.to_radians();
    let depth = 1.7;
    assert!(r / (theta / 2.0).tan() < depth && depth < r);

    let mut topo = Topology::new();
    let body = prism(&mut topo, theta, depth, 12.0, Vec3::new(0.0, 0.0, 1.0));
    let edge = ridge(&topo, body);
    let counts = solid_entity_counts(&topo, body).unwrap();
    for (name, run) in engines() {
        let mut t = topo.clone();
        let result = run(&mut t, body, &[edge], r)
            .unwrap_or_else(|e| panic!("{name}: the band fits the face: {e}"));
        assert_ne!(solid_entity_counts(&t, result.solid).unwrap(), counts);
        let report = remus_check::validate::validate_solid(
            &t,
            result.solid,
            &remus_check::validate::ValidateOptions::default(),
        )
        .unwrap();
        assert_eq!(report.error_count(), 0, "{name}: {:?}", report.issues);
        assert_eq!(
            classify_point(
                &t,
                result.solid,
                Point3::new(0.1, 0.05, HEIGHT / 2.0),
                &ClassifyOptions::default()
            )
            .unwrap(),
            PointClassification::Outside,
            "{name}: the ridge tip is rounded away"
        );
    }
}
