//! Arena documents must refuse analytic carriers no constructor would build
//! (kernel review 2026-10-14, IO-7).
//!
//! The decoder fills the analytic surface and curve structs field by field,
//! and replay validated only NURBS carriers, so a zero or non-unit plane
//! normal, a negative radius or a skewed frame deserialized `Ok` and reached
//! evaluation. Each case below edits one field of a valid document and must
//! be refused with [`IoError::InvalidGeometry`] without touching the
//! destination arena.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::IoError;
use remus_io::arena_io::{deserialize_solids, serialize_solids};
use remus_operations::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
use remus_topology::Topology;
use serde_json::{Value, json};

fn document(build: impl FnOnce(&mut Topology) -> remus_topology::SolidId) -> Value {
    let mut topo = Topology::new();
    let solid = build(&mut topo);
    serde_json::from_slice(&serialize_solids(&topo, &[solid]).unwrap()).unwrap()
}

/// The first face carrier tagged `variant`, as a mutable JSON object.
fn surface<'a>(document: &'a mut Value, variant: &str) -> &'a mut Value {
    document["faces"]
        .as_array_mut()
        .unwrap()
        .iter_mut()
        .find_map(|face| face["surface"].get_mut(variant))
        .unwrap_or_else(|| panic!("document has no {variant} face"))
}

/// The first edge carrier tagged `variant`, as a mutable JSON object.
fn curve<'a>(document: &'a mut Value, variant: &str) -> &'a mut Value {
    document["edges"]
        .as_array_mut()
        .unwrap()
        .iter_mut()
        .find_map(|edge| edge["curve"].get_mut(variant))
        .unwrap_or_else(|| panic!("document has no {variant} edge"))
}

fn decode(document: &Value) -> (Result<Vec<remus_topology::SolidId>, IoError>, Topology) {
    let mut topo = Topology::new();
    let bytes = serde_json::to_vec(document).unwrap();
    let result = deserialize_solids(&bytes, &mut topo);
    (result, topo)
}

#[track_caller]
fn assert_refused(document: &Value, expected_code: &str) {
    let (result, topo) = decode(document);
    match result {
        Err(IoError::InvalidGeometry { code, reason }) => {
            assert_eq!(code, expected_code, "wrong refusal code: {reason}");
        }
        other => panic!("expected InvalidGeometry({expected_code}), got {other:?}"),
    }
    assert!(
        topo.vertices().is_empty() && topo.faces().is_empty(),
        "a refused document must not leave topology behind"
    );
}

fn boxed() -> Value {
    document(|topo| make_box(topo, 1.0, 2.0, 3.0).unwrap())
}

fn cylinder() -> Value {
    document(|topo| make_cylinder(topo, 1.5, 4.0).unwrap())
}

fn sphere() -> Value {
    document(|topo| make_sphere(topo, 2.0, 16).unwrap())
}

#[test]
fn unedited_analytic_documents_still_decode() {
    for document in [
        boxed(),
        cylinder(),
        sphere(),
        document(|topo| make_cone(topo, 2.0, 1.0, 3.0).unwrap()),
        document(|topo| make_torus(topo, 5.0, 1.0, 16).unwrap()),
    ] {
        let (result, _) = decode(&document);
        assert_eq!(result.unwrap().len(), 1);
    }
}

#[test]
fn zero_and_non_unit_plane_normals_are_refused() {
    for normal in [json!([0.0, 0.0, 0.0]), json!([0.0, 0.0, 2.0])] {
        let mut document = boxed();
        surface(&mut document, "Plane")["normal"] = normal;
        assert_refused(&document, "non-unit-direction");
    }
}

#[test]
fn non_positive_cylinder_and_sphere_radii_are_refused() {
    for radius in [-1.0, 0.0] {
        let mut document = cylinder();
        surface(&mut document, "Cylinder")["radius"] = json!(radius);
        assert_refused(&document, "non-positive-length");

        let mut document = sphere();
        surface(&mut document, "Sphere")["radius"] = json!(radius);
        assert_refused(&document, "non-positive-length");
    }
}

#[test]
fn skewed_and_scaled_surface_frames_are_refused() {
    let mut document = cylinder();
    // Unit length, but no longer perpendicular to the axis.
    let skew = std::f64::consts::FRAC_1_SQRT_2;
    surface(&mut document, "Cylinder")["x_axis"] = json!([skew, 0.0, skew]);
    assert_refused(&document, "non-orthogonal-frame");

    let mut document = sphere();
    surface(&mut document, "Sphere")["z_axis"] = json!([0.0, 0.0, 3.0]);
    assert_refused(&document, "non-unit-direction");
}

#[test]
fn cone_half_angle_outside_its_open_interval_is_refused() {
    for half_angle in [0.0, std::f64::consts::FRAC_PI_2, -0.3] {
        let mut document = document(|topo| make_cone(topo, 2.0, 1.0, 3.0).unwrap());
        surface(&mut document, "Cone")["half_angle"] = json!(half_angle);
        assert_refused(&document, "angle-out-of-range");
    }
}

#[test]
fn non_positive_torus_radii_are_refused() {
    for field in ["major_radius", "minor_radius"] {
        let mut document = document(|topo| make_torus(topo, 5.0, 1.0, 16).unwrap());
        surface(&mut document, "Torus")[field] = json!(-1.0);
        assert_refused(&document, "non-positive-length");
    }
}

#[test]
fn circle_edges_check_radius_and_frame() {
    let mut document = cylinder();
    curve(&mut document, "Circle")["radius"] = json!(-1.5);
    assert_refused(&document, "non-positive-length");

    let mut document = cylinder();
    curve(&mut document, "Circle")["normal"] = json!([0.0, 0.0, 0.0]);
    assert_refused(&document, "non-unit-direction");
}

/// The frame bound scales with the carrier: a u axis 5e-10 too long is
/// rounding on a radius-1.5 circle but puts a radius-1e12 circle's start
/// point 500 units off its declared radius.
#[test]
fn frame_error_is_bounded_by_carrier_size() {
    let stretched = |radius: f64| {
        let mut document = cylinder();
        let circle = curve(&mut document, "Circle");
        circle["radius"] = json!(radius);
        circle["u_axis"] = json!([0.0, 1.0 + 5e-10, 0.0]);
        document
    };
    assert_refused(&stretched(1e12), "non-unit-direction");
    // 5e-10 on radius 100 is a 5e-8 displacement, inside the linear tolerance.
    if let Err(IoError::InvalidGeometry { code, reason }) = decode(&stretched(100.0)).0 {
        panic!("a 5e-8 frame displacement was refused ({code}): {reason}");
    }

    // Exact unit axes on a huge carrier still pass carrier validation.
    let mut document = cylinder();
    curve(&mut document, "Circle")["radius"] = json!(1e12);
    assert!(!matches!(
        decode(&document).0,
        Err(IoError::InvalidGeometry { .. })
    ));
}

/// A cone or parabola has no size of its own: its trimmed extent multiplies
/// any frame error, so the frame must be unit to rounding.
#[test]
fn unbounded_carriers_need_a_rounding_exact_frame() {
    use remus_math::curves::Parabola3D;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    let mut document = document(|topo| make_cone(topo, 2.0, 1.0, 3.0).unwrap());
    surface(&mut document, "Cone")["axis"] = json!([0.0, 0.0, 1.0 + 5e-10]);
    assert_refused(&document, "non-unit-direction");

    // f = 1e-6 trimmed at t = ±1: a 5e-10 axis error moves the ends 1.25e-4.
    let mut topo = Topology::new();
    let parabola =
        Parabola3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 1e-6).unwrap();
    let start = topo.add_vertex(Vertex::new(parabola.evaluate(-1.0), 1e-7));
    let end = topo.add_vertex(Vertex::new(parabola.evaluate(1.0), 1e-7));
    let edge = topo.add_edge(Edge::new(start, end, EdgeCurve::Parabola(parabola)));
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], false).unwrap());
    let bytes = remus_io::arena_io::serialize_wire(&topo, wire).unwrap();
    let mut restored = Topology::new();
    assert_eq!(
        remus_io::arena_io::deserialize_wires(&bytes, &mut restored)
            .unwrap()
            .len(),
        1
    );

    let mut document: Value = serde_json::from_slice(&bytes).unwrap();
    curve(&mut document, "Parabola")["axis_dir"] = json!([1.0 + 5e-10, 0.0, 0.0]);
    let mut restored = Topology::new();
    match remus_io::arena_io::deserialize_wires(
        &serde_json::to_vec(&document).unwrap(),
        &mut restored,
    ) {
        Err(IoError::InvalidGeometry { code, .. }) => assert_eq!(code, "non-unit-direction"),
        other => panic!("expected the stretched parabola to be refused, got {other:?}"),
    }
}

#[test]
fn rotated_analytic_carriers_stay_within_the_frame_tolerance() {
    // A rigid transform's rounding must stay far inside the 1e-9 band, so
    // kernel-written carriers keep decoding.
    use remus_math::mat::Mat4;
    for build in [
        (|topo: &mut Topology| make_cylinder(topo, 1.5, 4.0).unwrap()) as fn(&mut Topology) -> _,
        |topo| make_cone(topo, 2.0, 1.0, 3.0).unwrap(),
        |topo| make_sphere(topo, 2.0, 16).unwrap(),
        |topo| make_torus(topo, 5.0, 1.0, 16).unwrap(),
    ] {
        let mut topo = Topology::new();
        let solid = build(&mut topo);
        let moved = Mat4::translation(1e3, -2e3, 5e2)
            * Mat4::rotation_x(0.731)
            * Mat4::rotation_y(-2.2)
            * Mat4::rotation_z(1.3);
        remus_operations::transform::transform_solid(&mut topo, solid, &moved).unwrap();
        let bytes = serialize_solids(&topo, &[solid]).unwrap();
        let mut restored = Topology::new();
        assert_eq!(deserialize_solids(&bytes, &mut restored).unwrap().len(), 1);
    }
}

#[test]
fn ellipse_minor_axis_longer_than_major_is_refused() {
    use remus_math::mat::Mat4;
    use remus_operations::boolean::{BooleanOp, boolean};

    // A tilted planar cut through a cylinder trims its wall with exact
    // ellipse arcs.
    let mut document = document(|topo| {
        // The exact cut of `boolean::tests::cut_cylinder_by_tilted_slab_*`.
        let cylinder = make_cylinder(topo, 6.0, 6.0).unwrap();
        let slab = make_box(topo, 6.0, 1.0, 6.5).unwrap();
        let place =
            Mat4::translation(-4.0, -4.0, -4.0) * Mat4::rotation_x(std::f64::consts::FRAC_PI_6);
        remus_operations::transform::transform_solid(topo, slab, &place).unwrap();
        boolean(topo, BooleanOp::Cut, cylinder, slab).unwrap()
    });
    let (result, _) = decode(&document);
    assert_eq!(result.unwrap().len(), 1);

    let ellipse = curve(&mut document, "Ellipse");
    let major = ellipse["semi_major"].as_f64().unwrap();
    ellipse["semi_minor"] = json!(major * 1.5);
    assert_refused(&document, "semi-axis-order");
}

/// The writer must refuse what replay refuses, so its output always replays.
/// `Circle3D::with_axes` checks only the radius, so a skewed frame can reach
/// a standalone wire.
#[test]
fn writer_refuses_carriers_its_reader_would_refuse() {
    use remus_math::curves::Circle3D;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    let circle = |u_axis: Vec3| {
        let mut topo = Topology::new();
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let circle = Circle3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            normal,
            1.0,
            u_axis,
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let vertex = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
        let edge = topo.add_edge(Edge::new(vertex, vertex, EdgeCurve::Circle(circle)));
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
        remus_io::arena_io::serialize_wire(&topo, wire)
    };

    let bytes = circle(Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let mut restored = Topology::new();
    assert_eq!(
        remus_io::arena_io::deserialize_wires(&bytes, &mut restored)
            .unwrap()
            .len(),
        1
    );

    let skew = std::f64::consts::FRAC_1_SQRT_2;
    match circle(Vec3::new(skew, 0.0, skew)) {
        Err(IoError::InvalidGeometry { code, .. }) => assert_eq!(code, "non-orthogonal-frame"),
        other => panic!("expected the writer to refuse a skewed circle, got {other:?}"),
    }
}
