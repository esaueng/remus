//! IO-7 (kernel review 2026-10-14): the arena decoder validated NURBS
//! carriers but deserialized analytic ones straight into their private fields.
//! A plane normal of `[0,0,0]` or `|n| = 2`, a cylinder of radius −1 and a
//! sphere of radius −1 all decoded Ok. Analytic carriers are now held to the
//! invariants their constructors establish (finite placement, positive finite
//! radii, a cone half-angle inside `(0, π/2)`, unit mutually perpendicular
//! axes) on both encode and decode, so serialize succeeding implies deserialize
//! succeeding.
//!
//! Every edge mutation below keeps the curve's point set, so the decoder's
//! endpoint certification cannot be what refuses it: a negated radius or semi
//! axis is paired with a negated axis, and a broken normal is not evaluated at
//! all.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::IoError;
use remus_io::arena_io::{deserialize_solids, deserialize_wire, serialize_solids, serialize_wire};
use remus_io::step::{read_step, write_step};
use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use remus_math::vec::{Point3, Vec3};
use remus_operations::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::FaceSurface;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};
use serde_json::{Value, json};

fn edit(bytes: &[u8], mutate: impl FnOnce(&mut Value)) -> Vec<u8> {
    let mut document: Value = serde_json::from_slice(bytes).unwrap();
    mutate(&mut document);
    serde_json::to_vec(&document).unwrap()
}

/// The first face surface of the given variant.
fn surface<'a>(document: &'a mut Value, variant: &str) -> &'a mut Value {
    document["faces"]
        .as_array_mut()
        .unwrap()
        .iter_mut()
        .map(|face| &mut face["surface"])
        .find(|surface| surface.get(variant).is_some())
        .and_then(|surface| surface.get_mut(variant))
        .unwrap_or_else(|| panic!("document has no {variant} face"))
}

/// The first edge curve of the given variant.
fn curve<'a>(document: &'a mut Value, variant: &str) -> &'a mut Value {
    document["edges"]
        .as_array_mut()
        .unwrap()
        .iter_mut()
        .map(|edge| &mut edge["curve"])
        .find(|curve| curve.get(variant).is_some())
        .and_then(|curve| curve.get_mut(variant))
        .unwrap_or_else(|| panic!("document has no {variant} edge"))
}

fn negate(vector: &mut Value) {
    for component in vector.as_array_mut().unwrap() {
        *component = json!(-component.as_f64().unwrap());
    }
}

fn scale(vector: &mut Value, factor: f64) {
    for component in vector.as_array_mut().unwrap() {
        *component = json!(component.as_f64().unwrap() * factor);
    }
}

fn solid_document(build: impl FnOnce(&mut Topology) -> remus_topology::solid::SolidId) -> Vec<u8> {
    let mut topo = Topology::new();
    let solid = build(&mut topo);
    let bytes = serialize_solids(&topo, &[solid]).unwrap();
    // Control: the unmutated document decodes.
    deserialize_solids(&bytes, &mut Topology::new()).unwrap();
    bytes
}

#[track_caller]
fn assert_solid_refused(bytes: &[u8], what: &str) {
    let mut topo = Topology::new();
    let error = deserialize_solids(bytes, &mut topo)
        .err()
        .unwrap_or_else(|| panic!("{what} deserialized"));
    assert!(
        error.to_string().contains("analytic"),
        "{what} was refused for another reason: {error}"
    );
    assert_eq!(
        topo.num_faces(),
        0,
        "{what}: a refused import mutated the destination"
    );
}

#[track_caller]
fn assert_wire_refused(bytes: &[u8], what: &str) {
    let error = deserialize_wire(bytes, &mut Topology::new())
        .err()
        .unwrap_or_else(|| panic!("{what} deserialized"));
    assert!(
        error.to_string().contains("analytic"),
        "{what} was refused for another reason: {error}"
    );
}

#[test]
fn plane_normal_must_be_unit() {
    let bytes = solid_document(|topo| make_box(topo, 1.0, 2.0, 3.0).unwrap());
    let zero = edit(&bytes, |doc| {
        surface(doc, "Plane")["normal"] = json!([0.0, 0.0, 0.0]);
    });
    assert_solid_refused(&zero, "plane with zero normal");
    let doubled = edit(&bytes, |doc| {
        scale(&mut surface(doc, "Plane")["normal"], 2.0);
    });
    assert_solid_refused(&doubled, "plane with |n| = 2");
}

#[test]
fn cylinder_radius_must_be_positive_and_frame_orthonormal() {
    let bytes = solid_document(|topo| make_cylinder(topo, 2.0, 5.0).unwrap());
    let negative = edit(&bytes, |doc| {
        surface(doc, "Cylinder")["radius"] = json!(-1.0);
    });
    assert_solid_refused(&negative, "cylinder with r = -1");
    let zero = edit(&bytes, |doc| {
        surface(doc, "Cylinder")["radius"] = json!(0.0);
    });
    assert_solid_refused(&zero, "cylinder with r = 0");
    let skewed = edit(&bytes, |doc| {
        let cylinder = surface(doc, "Cylinder");
        cylinder["y_axis"] = cylinder["x_axis"].clone();
    });
    assert_solid_refused(&skewed, "cylinder with x_axis = y_axis");
    let no_axis = edit(&bytes, |doc| {
        surface(doc, "Cylinder")["axis"] = json!([0.0, 0.0, 0.0]);
    });
    assert_solid_refused(&no_axis, "cylinder with a zero axis");
}

#[test]
fn sphere_radius_must_be_positive() {
    let bytes = solid_document(|topo| make_sphere(topo, 3.0, 8).unwrap());
    let negative = edit(&bytes, |doc| {
        surface(doc, "Sphere")["radius"] = json!(-1.0);
    });
    assert_solid_refused(&negative, "sphere with r = -1");
    let stretched = edit(&bytes, |doc| {
        scale(&mut surface(doc, "Sphere")["z_axis"], 2.0);
    });
    assert_solid_refused(&stretched, "sphere with |z_axis| = 2");
}

#[test]
fn cone_half_angle_must_lie_strictly_inside_a_right_angle() {
    let bytes = solid_document(|topo| make_cone(topo, 2.0, 1.0, 3.0).unwrap());
    for half_angle in [0.0, -0.5, std::f64::consts::FRAC_PI_2, 2.0] {
        let bad = edit(&bytes, |doc| {
            surface(doc, "Cone")["half_angle"] = json!(half_angle);
        });
        assert_solid_refused(&bad, &format!("cone with half_angle {half_angle}"));
    }
}

#[test]
fn torus_radii_must_be_positive() {
    let bytes = solid_document(|topo| make_torus(topo, 5.0, 1.0, 8).unwrap());
    for (field, value) in [
        ("major_radius", -5.0),
        ("minor_radius", -1.0),
        ("minor_radius", 0.0),
    ] {
        let bad = edit(&bytes, |doc| surface(doc, "Torus")[field] = json!(value));
        assert_solid_refused(&bad, &format!("torus with {field} {value}"));
    }
}

/// One open wire holding a single trimmed edge on `curve`.
fn single_edge_wire(topo: &mut Topology, curve: EdgeCurve, t0: f64, t1: f64) -> WireId {
    let start =
        curve.evaluate_with_endpoints(t0, Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0));
    let end =
        curve.evaluate_with_endpoints(t1, Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0));
    let v0 = topo.add_vertex(Vertex::new(start, 1e-7));
    let v1 = topo.add_vertex(Vertex::new(end, 1e-7));
    let mut edge = Edge::new(v0, v1, curve);
    edge.set_trim(Some((t0, t1)));
    let edge = topo.add_edge(edge);
    topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], false).unwrap())
}

fn wire_document(curve: EdgeCurve, t0: f64, t1: f64) -> Vec<u8> {
    let mut topo = Topology::new();
    let wire = single_edge_wire(&mut topo, curve, t0, t1);
    let bytes = serialize_wire(&topo, wire).unwrap();
    deserialize_wire(&bytes, &mut Topology::new()).unwrap();
    bytes
}

fn tilted_frame() -> (Point3, Vec3, Vec3) {
    let center = Point3::new(1.0, -2.0, 0.5);
    let normal = Vec3::new(1.0, 2.0, 2.0).normalize().unwrap();
    let u = Vec3::new(2.0, -1.0, 0.0).normalize().unwrap();
    (center, normal, u)
}

#[test]
fn circle_radius_and_frame_are_checked() {
    let (center, normal, u) = tilted_frame();
    let circle = Circle3D::new_with_ref(center, normal, 2.0, u).unwrap();
    let bytes = wire_document(EdgeCurve::Circle(circle), 0.25, 2.0);
    // P(t) = c + r(cos t·u + sin t·v): negating r, u and v keeps every point.
    let mirrored = edit(&bytes, |doc| {
        let circle = curve(doc, "Circle");
        circle["radius"] = json!(-2.0);
        negate(&mut circle["u_axis"]);
        negate(&mut circle["v_axis"]);
    });
    assert_wire_refused(&mirrored, "circle with r = -2");
    let no_normal = edit(&bytes, |doc| {
        curve(doc, "Circle")["normal"] = json!([0.0, 0.0, 0.0]);
    });
    assert_wire_refused(&no_normal, "circle with a zero normal");
}

#[test]
fn ellipse_and_hyperbola_semi_axes_are_checked() {
    let (center, normal, u) = tilted_frame();
    let ellipse = Ellipse3D::new_with_ref(center, normal, 3.0, 1.0, u).unwrap();
    let hyperbola = Hyperbola3D::with_axes(center, normal, u, 2.0, 1.5).unwrap();
    for (variant, curve_value) in [
        ("Ellipse", EdgeCurve::Ellipse(ellipse)),
        ("Hyperbola", EdgeCurve::Hyperbola(hyperbola)),
    ] {
        let bytes = wire_document(curve_value, -0.5, 0.75);
        // P(t) = c + a·f(t)·u + b·g(t)·v: negating a with u keeps every point.
        let mirrored = edit(&bytes, |doc| {
            let conic = curve(doc, variant);
            let semi_major = conic["semi_major"].as_f64().unwrap();
            conic["semi_major"] = json!(-semi_major);
            negate(&mut conic["u_axis"]);
        });
        assert_wire_refused(&mirrored, &format!("{variant} with a negative semi_major"));
        let stretched_normal = edit(&bytes, |doc| scale(&mut curve(doc, variant)["normal"], 2.0));
        assert_wire_refused(&stretched_normal, &format!("{variant} with |normal| = 2"));
    }
}

#[test]
fn parabola_focal_length_is_checked() {
    let (vertex, axis, u) = tilted_frame();
    let u = (u - axis * u.dot(axis)).normalize().unwrap();
    let parabola = Parabola3D::with_axes(vertex, axis, u, 0.75).unwrap();
    let bytes = wire_document(EdgeCurve::Parabola(parabola), -1.0, 2.0);
    // P(t) = V + t²/(4f)·a + t·u: negating f with a keeps every point.
    let mirrored = edit(&bytes, |doc| {
        let parabola = curve(doc, "Parabola");
        parabola["focal_length"] = json!(-0.75);
        negate(&mut parabola["axis_dir"]);
    });
    assert_wire_refused(&mirrored, "parabola with f = -0.75");
}

/// The writer applies the same checks, so a document it emits is one the
/// reader accepts: `Circle3D::with_axes` trusts its caller's frame, and a
/// circle built from a zero normal is refused at serialization instead of
/// producing a document that cannot be read back.
#[test]
fn serializer_refuses_what_the_reader_would() {
    let (center, _, u) = tilted_frame();
    let v = Vec3::new(0.0, 0.0, 1.0).cross(u).normalize().unwrap();
    let circle = Circle3D::with_axes(center, Vec3::new(0.0, 0.0, 0.0), 2.0, u, v).unwrap();
    let mut topo = Topology::new();
    let wire = single_edge_wire(&mut topo, EdgeCurve::Circle(circle), 0.0, 1.0);
    let error = serialize_wire(&topo, wire).unwrap_err();
    assert!(
        matches!(error, IoError::InvalidTopology { .. }) && error.to_string().contains("analytic"),
        "{error}"
    );
}

/// ISO 10303-42 direction ratios need not be unit. The STEP reader rescales
/// a `PLANE` normal, so a file that states its axes unnormalized still
/// crosses the arena: the WASM translator serializes every STEP import.
#[test]
fn step_plane_with_unnormalized_direction_crosses_the_arena() {
    // Ratios whose squared length overflows or underflows f64 are still
    // valid directions.
    for scale in ["2.5E0", "1.0E200", "1.0E-200"] {
        step_plane_with_scaled_directions_crosses_the_arena(scale);
    }
}

fn step_plane_with_scaled_directions_crosses_the_arena(scale: &str) {
    let mut source = Topology::new();
    let solid = make_box(&mut source, 1.0, 2.0, 3.0).unwrap();
    let text = write_step(&source, &[solid]).unwrap();
    let mut scaled = 0;
    let text: String = text
        .lines()
        .map(|line| {
            if line.contains("DIRECTION(") {
                scaled += 1;
                line.replace("1.00000000000000000E0", scale)
            } else {
                line.to_owned()
            }
        })
        .collect::<Vec<_>>()
        .join("\n");
    assert!(scaled > 0 && text.contains(scale), "{text}");

    let mut topo = Topology::new();
    let solids = read_step(&text, &mut topo).unwrap();
    let bytes = serialize_solids(&topo, &solids).unwrap();
    let restored = deserialize_solids(&bytes, &mut Topology::new()).unwrap();
    assert_eq!(restored.len(), 1);
    for face in remus_topology::explorer::solid_faces(&topo, solids[0]).unwrap() {
        let FaceSurface::Plane { normal, .. } = topo.face(face).unwrap().surface() else {
            panic!("a box face is planar");
        };
        assert!((normal.length() - 1.0).abs() < 1e-15, "{normal:?}");
    }
    let volume = remus_operations::measure::solid_volume(&topo, solids[0], 0.01).unwrap();
    assert!((volume - 6.0).abs() < 1e-9, "{volume}");
}
