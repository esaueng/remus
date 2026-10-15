//! Preserve real cone generator boundaries while removing only doubled apex seams.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
use remus_math::curves::Circle3D;
use remus_math::surfaces::ConicalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_operations::tessellate::tessellate_with_uvs_a;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

fn chart_wire(
    t: &mut Topology,
    cone: &ConicalSurface,
    uv: &[(f64, f64)],
) -> remus_topology::wire::WireId {
    let vs: Vec<_> = uv
        .iter()
        .map(|&(u, v)| t.add_vertex(Vertex::new(cone.evaluate(u, v), 1e-7)))
        .collect();
    let mut uses = vec![];
    for i in 0..uv.len() {
        let j = (i + 1) % uv.len();
        let ((u0, v0), (u1, v1)) = (uv[i], uv[j]);
        let edge = if v0.to_bits() == v1.to_bits() && v0 > 0.0 {
            let center = cone.apex() + cone.axis() * (v0 * cone.half_angle().sin());
            let circle =
                Circle3D::new_with_ref(center, cone.axis(), cone.radius_at(v0), cone.x_axis())
                    .unwrap();
            let mut e = Edge::new(vs[i], vs[j], EdgeCurve::Circle(circle));
            e.set_trim(Some((u0, u1)));
            e
        } else {
            Edge::new(vs[i], vs[j], EdgeCurve::Line)
        };
        uses.push(OrientedEdge::new(t.add_edge(edge), true));
    }
    t.add_wire(Wire::new(uses, true).unwrap())
}

#[test]
fn partial_pointed_cone_with_pocket_does_not_expand_past_its_generators() {
    let mut t = Topology::new();
    let cone = ConicalSurface::with_ref_dir(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::FRAC_PI_4,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let outer = chart_wire(
        &mut t,
        &cone,
        &[(0.0, 2.0), (std::f64::consts::PI, 2.0), (0.0, 0.0)],
    );
    let inner = chart_wire(
        &mut t,
        &cone,
        &[(0.4, 1.0), (0.6, 1.0), (0.6, 1.2), (0.4, 1.2)],
    );
    let face = t.add_face(Face::new(outer, vec![inner], FaceSurface::Cone(cone)));
    remus_topology::validation::validate_boundary_authority(&t).unwrap();
    for wire in [outer, inner] {
        remus_topology::validation::validate_wire_closed(t.wire(wire).unwrap(), &t).unwrap();
    }
    let mesh = tessellate_with_uvs_a(&t, face, 0.01, 0.2).unwrap().mesh;
    assert!(!mesh.indices.is_empty());
    let mut area = 0.0;
    let mut ymin = f64::INFINITY;
    for tri in mesh.indices.chunks_exact(3) {
        let [a, b, c] = [tri[0], tri[1], tri[2]].map(|id| mesh.positions[id as usize]);
        area += (b - a).cross(c - a).length() * 0.5;
        ymin = ymin.min(a.y()).min(b.y()).min(c.y());
    }
    assert!(area.is_finite());
    assert!(
        ymin >= -1e-7,
        "mesher expanded past the two distinct generator boundaries: ymin={ymin}"
    );
}

#[test]
fn doubled_apex_seam_keeps_the_rim_only_pocket_mesh_for_every_wire_rotation() {
    let mut t = Topology::new();
    let cone = ConicalSurface::with_ref_dir(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::FRAC_PI_4,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let rim_v = 2.0;
    let rim_vertex = t.add_vertex(Vertex::new(cone.evaluate(0.0, rim_v), 1e-7));
    let apex_vertex = t.add_vertex(Vertex::new(cone.apex(), 1e-7));
    let circle = Circle3D::new_with_ref(
        cone.apex() + cone.axis() * (rim_v * cone.half_angle().sin()),
        cone.axis(),
        cone.radius_at(rim_v),
        cone.x_axis(),
    )
    .unwrap();
    let mut rim = Edge::new(rim_vertex, rim_vertex, EdgeCurve::Circle(circle));
    rim.set_trim(Some((0.0, std::f64::consts::TAU)));
    let rim = t.add_edge(rim);
    let seam = t.add_edge(Edge::new(rim_vertex, apex_vertex, EdgeCurve::Line));
    let pocket = chart_wire(
        &mut t,
        &cone,
        &[(0.4, 1.0), (0.6, 1.0), (0.6, 1.2), (0.4, 1.2)],
    );
    let outer = t.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true).unwrap());
    let reference = t.add_face(Face::new(
        outer,
        vec![pocket],
        FaceSurface::Cone(cone.clone()),
    ));
    let reference = tessellate_with_uvs_a(&t, reference, 0.01, 0.2)
        .unwrap()
        .mesh;
    let mut uses = vec![
        OrientedEdge::new(rim, true),
        OrientedEdge::new(seam, true),
        OrientedEdge::new(seam, false),
    ];
    for _ in 0..3 {
        let outer = t.add_wire(Wire::new(uses.clone(), true).unwrap());
        let face = t.add_face(Face::new(
            outer,
            vec![pocket],
            FaceSurface::Cone(cone.clone()),
        ));
        remus_topology::validation::validate_boundary_authority(&t).unwrap();
        let mesh = tessellate_with_uvs_a(&t, face, 0.01, 0.2).unwrap().mesh;
        assert_eq!(mesh.positions, reference.positions);
        assert_eq!(mesh.normals, reference.normals);
        assert_eq!(mesh.indices, reference.indices);
        uses.rotate_left(1);
    }
}

/// A genuine trimmed band takes the sliced adaptive path. The cone flux
/// identity holds independently of whether the trimmed face reaches its apex.
#[test]
fn trimmed_cone_band_uses_stable_adaptive_apex_flux() {
    let mut topo = Topology::new();
    let apex = Point3::new(13.0, -7.0, 5.0);
    let cone = ConicalSurface::with_ref_dir(
        apex,
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::FRAC_PI_4,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let outer = chart_wire(
        &mut topo,
        &cone,
        &[(0.2, 1.0), (1.4, 1.0), (1.4, 2.0), (0.2, 2.0)],
    );
    let pocket = chart_wire(
        &mut topo,
        &cone,
        &[(0.4, 1.2), (0.6, 1.2), (0.6, 1.4), (0.4, 1.4)],
    );
    let face = topo.add_face(Face::new(outer, vec![pocket], FaceSurface::Cone(cone)));
    let options = remus_check::properties::PropertiesOptions {
        adaptive_eps: 1e-9,
        max_depth: 12,
        ..Default::default()
    };
    let contribution =
        remus_check::properties::face_integrator::integrate_face_about(&topo, face, &options, apex)
            .unwrap();
    let expected_area = (1.2 * 1.5 - 0.2 * 0.26) * std::f64::consts::FRAC_1_SQRT_2;
    assert!(
        (contribution.area - expected_area).abs() < 1e-9,
        "area {} expected {}",
        contribution.area,
        expected_area
    );
    assert_eq!(contribution.volume.abs().to_bits(), 0);
}
