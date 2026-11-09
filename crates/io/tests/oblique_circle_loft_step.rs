//! #1253: a loft between laterally offset parallel circles is an exact
//! oblique frustum: two exact circular rims, one curved ruled band and two
//! planar caps. A STEP round trip must keep all of it — the rims as
//! circles, the band as one curved surface whose every point stays on the
//! closed-form section circle, the outward orientation and the exact volume.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::step::reader::read_step;
use remus_io::step::write_step;
use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_operations::loft::{loft, loft_smooth};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const BOTTOM: (Point3, f64) = (Point3::new(0.0, 0.0, 0.0), 25.0);
const TOP: (Point3, f64) = (Point3::new(12.0, 0.0, 60.0), 10.0);
/// π·h·(R² + R·r + r²)/3 for h 60, R 25, r 10 (Cavalieri: the lateral
/// offset does not change it).
const EXACT_VOLUME: f64 = 61_261.056_745_000_96;

fn circle_section(topo: &mut Topology, (center, radius): (Point3, f64)) -> FaceId {
    let z = Vec3::new(0.0, 0.0, 1.0);
    let circle = Circle3D::new(center, z, radius).unwrap();
    let seam = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let mut edge = Edge::new(seam, seam, EdgeCurve::Circle(circle));
    edge.set_trim(Some((0.0, std::f64::consts::TAU)));
    let edge = topo.add_edge(edge);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: z,
            d: center.z(),
        },
    ))
}

fn assert_exact_oblique_frustum(topo: &Topology, solid: SolidId, label: &str) {
    let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: invalid solid {report:?}");

    let faces = solid_faces(topo, solid).unwrap();
    let planes = faces
        .iter()
        .filter(|&&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Plane { .. }))
        .count();
    let bands: Vec<FaceId> = faces
        .iter()
        .copied()
        .filter(|&f| !matches!(topo.face(f).unwrap().surface(), FaceSurface::Plane { .. }))
        .collect();
    assert_eq!(
        (faces.len(), planes, bands.len()),
        (3, 2, 1),
        "{label}: two caps and one curved band"
    );

    let mut rims: Vec<(Point3, f64)> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|e| match topo.edge(e).unwrap().curve() {
            EdgeCurve::Circle(c) => Some((c.center(), c.radius())),
            _ => None,
        })
        .collect();
    rims.sort_by(|a, b| a.0.z().total_cmp(&b.0.z()));
    assert_eq!(
        rims.len(),
        2,
        "{label}: both section boundaries stay circles"
    );
    for (rim, (center, radius)) in rims.iter().zip([BOTTOM, TOP]) {
        assert!(
            (rim.0 - center).length() < 1e-9 && (rim.1 - radius).abs() < 1e-9,
            "{label}: rim {rim:?} vs section {center:?} r {radius}"
        );
    }

    let face = topo.face(bands[0]).unwrap();
    let FaceSurface::Nurbs(surface) = face.surface() else {
        panic!("{label}: the band is a rational ruled surface");
    };
    let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
    for i in 0..=10 {
        for j in 0..40 {
            let u = u0 + (u1 - u0) * f64::from(i) / 10.0;
            let v = v0 + (v1 - v0) * (f64::from(j) + 0.5) / 40.0;
            let p = surface.evaluate(u, v);
            let t = p.z() / 60.0;
            let center = Point3::new(12.0 * t, 0.0, 60.0 * t);
            let radius = 15.0f64.mul_add(-t, 25.0);
            let offset = p - center;
            assert!(
                (offset.length() - radius).abs() < 1e-9 && offset.z().abs() < 1e-12,
                "{label}: band point {p:?} is off the section circle at height {}",
                p.z()
            );
            let mut normal = surface.normal(u, v).unwrap();
            if face.is_reversed() {
                normal = normal * -1.0;
            }
            assert!(
                normal.dot(offset) > 0.0,
                "{label}: band normal points inward"
            );
        }
    }

    let volume = remus_operations::measure::mass_properties(topo, solid)
        .unwrap()
        .mass;
    assert!(
        ((volume - EXACT_VOLUME) / EXACT_VOLUME).abs() < 1e-9,
        "{label}: volume {volume} vs exact {EXACT_VOLUME}"
    );

    let mesh = remus_operations::tessellate::tessellate_solid(topo, solid, 0.05).unwrap();
    assert!(
        remus_operations::tessellate::is_watertight(&mesh),
        "{label}: open mesh"
    );
}

#[test]
fn offset_circle_loft_round_trips_through_step_exactly() {
    type LoftFn =
        fn(&mut Topology, &[FaceId]) -> Result<SolidId, remus_operations::OperationsError>;
    for (name, method) in [("loft", loft as LoftFn), ("loft_smooth", loft_smooth)] {
        let mut topo = Topology::new();
        let a = circle_section(&mut topo, BOTTOM);
        let b = circle_section(&mut topo, TOP);
        let solid = method(&mut topo, &[a, b]).unwrap();
        assert_exact_oblique_frustum(&topo, solid, name);

        let step = write_step(&topo, &[solid]).unwrap();
        assert!(
            step.contains("B_SPLINE_SURFACE_WITH_KNOTS"),
            "{name}: band exported"
        );
        assert_eq!(
            step.matches("= CIRCLE(").count(),
            2,
            "{name}: two exact rims"
        );
        let mut reread = Topology::new();
        let solids = read_step(&step, &mut reread).unwrap();
        assert_eq!(solids.len(), 1);
        assert_exact_oblique_frustum(&reread, solids[0], &format!("{name} after STEP"));
    }
}
