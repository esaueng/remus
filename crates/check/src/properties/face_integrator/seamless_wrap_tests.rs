//! Closed-form oracles for cone and cylinder faces whose outer wire wraps
//! the axis with no seam and climbs and falls along it: a pointed cone face
//! bounded by one oblique rim, and a cylinder band between an oblique rim
//! and a circle.
//!
//! The sampled trim once read such a face as the band of its outer rim's
//! own axial extent: the pointed faces of `cone_green_tests`' sections read
//! 3.6 % low, 2.5 times and 2.2 times their area, and the cylinder band a
//! third of its area. Every number here comes from solid geometry:
//!
//! * the pointed face is the lateral face of the cone over the elliptic
//!   section, whose volume and first moments `cone_green_tests` derives;
//!   its area is the ellipse's projection onto the radial plane,
//!   `π A B cos θ`, over that plane's slope `cos a` on the cone;
//! * the band on a cylinder of radius `r` between `z = 0` and the plane
//!   `z = h + s x` has the area `2π r h` (the `s` term integrates out) and,
//!   about a point on the axis, the volume term `r A / 3`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::surfaces::{ConicalSurface, CylindricalSurface};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use crate::properties::PropertiesOptions;

use super::cone_green_tests::{Section, closed_nurbs_edge, refined, rel, sections};
use super::{integrate_face_about, integrate_face_area, integrate_face_fixed_about};

/// The face bounded by `section`'s rim alone, on `cone`: no seam, no apex
/// vertex.
fn seamless_face(topo: &mut Topology, section: &Section, cone: ConicalSurface) -> FaceId {
    let (rim, _) = closed_nurbs_edge(topo, section.rim());
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true).unwrap());
    topo.add_face(Face::new(wire, vec![], FaceSurface::Cone(cone)))
}

fn lateral_area(section: &Section) -> f64 {
    let (_, _, semi_major, semi_minor) = section.ellipse();
    PI * semi_major * semi_minor * section.theta.cos() / section.a.cos()
}

/// A pointed cone face bounded by one oblique rim reads the closed-form
/// cone on its boundary, through the area-only rule too, and to the chord
/// error of its trim on the sampled path.
#[test]
fn a_seamless_pointed_cone_face_reads_the_closed_form_cone() {
    for scale in [1e-3, 1.0, 1e3] {
        for (index, section) in sections(scale).iter().enumerate() {
            let mut topo = Topology::new();
            let face = seamless_face(&mut topo, section, section.cone());
            let reference = section.apex + Vec3::new(0.5, 0.25, -1.0) * scale;
            let (volume, moments) = section.lateral_terms(reference);
            let area = lateral_area(section);
            let v_scale = section.d.powi(3);
            let label = format!("section {index} at scale {scale}");

            let exact = integrate_face_about(&topo, face, &PropertiesOptions::default(), reference)
                .unwrap();
            assert!(
                rel(exact.area, area, area) < 1e-12,
                "{label}: area {} vs {area}",
                exact.area
            );
            assert!(
                rel(exact.volume, volume, v_scale) < 1e-12,
                "{label}: volume {} vs {volume}",
                exact.volume
            );
            let measured = [
                exact.volume_moment_x,
                exact.volume_moment_y,
                exact.volume_moment_z,
            ];
            let m_scale = v_scale * (section.d + (reference - section.apex).length());
            for (axis, (measured, exact)) in measured.into_iter().zip(moments).enumerate() {
                assert!(
                    rel(measured, exact, m_scale) < 1e-11,
                    "{label}: moment {axis} {measured} vs {exact}"
                );
            }
            let area_only = integrate_face_area(&topo, face, 5).unwrap();
            assert!(
                rel(area_only, area, area) < 1e-12,
                "{label}: area-only {area_only} vs {area}"
            );

            // The sampled trim alone: right to its chords, where it read the
            // band of the rim's extent before (3.6 % to 152 % off; its chords
            // leave 1e-4 here).
            let sampled = integrate_face_about(&topo, face, &refined(), reference).unwrap();
            assert!(
                rel(sampled.area, area, area) < 1e-3,
                "{label}: sampled area {} vs {area}",
                sampled.area
            );
            assert!(
                rel(sampled.volume, volume, v_scale) < 1e-3,
                "{label}: sampled volume {} vs {volume}",
                sampled.volume
            );
        }
    }
}

/// The same rim on the cone's other nappe (the carrier's axis reversed, so
/// the face lies at `v < 0`) is refused: there the chart is the carrier's
/// mirror image across its axis and would read the area but not the
/// moments.
#[test]
fn a_seamless_face_below_the_apex_is_refused() {
    let section = &sections(1.0)[1];
    let mut topo = Topology::new();
    let below = ConicalSurface::new(section.apex, -section.axis, section.a).unwrap();
    let face = seamless_face(&mut topo, section, below);
    let error = integrate_face_fixed_about(&topo, face, 5, section.apex).unwrap_err();
    assert!(
        error.to_string().contains("below its apex"),
        "unexpected refusal: {error}"
    );
}

/// The band on a cylinder of radius `r` between the circle `z = 0` (an
/// inner wire) and the oblique section `z = h + s x` (the outer wire), with
/// no seam: the closed-form area and volume term on its boundary and on the
/// sampled trim.
#[test]
fn a_seamless_band_under_an_oblique_cylinder_rim_reads_its_closed_form() {
    for scale in [1e-3, 1.0, 1e3] {
        let (r, h, s) = (scale, 3.0 * scale, 0.5);
        let mut topo = Topology::new();
        let ellipse = Ellipse3D::new_with_ref(
            Point3::new(0.0, 0.0, h),
            Vec3::new(-s, 0.0, 1.0),
            r * s.hypot(1.0),
            r,
            Vec3::new(1.0, 0.0, s),
        )
        .unwrap();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), r).unwrap();
        let mut wire_of = |curve: EdgeCurve, start: Point3, forward: bool| {
            let vertex = topo.add_vertex(Vertex::new(start, 1e-9 * scale));
            let mut edge = Edge::new(vertex, vertex, curve);
            edge.set_trim(Some((0.0, TAU)));
            let edge = topo.add_edge(edge);
            topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, forward)], true).unwrap())
        };
        // Counter-clockwise in `(u, v)` about the outward normal: the top
        // rim against `u`, the bottom one along it.
        let top = wire_of(
            EdgeCurve::Ellipse(ellipse.clone()),
            ellipse.evaluate(0.0),
            false,
        );
        let bottom = wire_of(
            EdgeCurve::Circle(circle.clone()),
            circle.evaluate(0.0),
            true,
        );
        let cylinder =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), r)
                .unwrap();
        let face = topo.add_face(Face::new(
            top,
            vec![bottom],
            FaceSurface::Cylinder(cylinder),
        ));
        let area = TAU * r * h;
        let volume = r * area / 3.0;
        let origin = Point3::new(0.0, 0.0, 0.0);
        for (label, reading) in [
            (
                "fixed",
                integrate_face_fixed_about(&topo, face, 5, origin).unwrap(),
            ),
            (
                "sampled",
                integrate_face_about(&topo, face, &refined(), origin).unwrap(),
            ),
        ] {
            let tolerance = if label == "fixed" { 1e-12 } else { 1e-6 };
            assert!(
                rel(reading.area, area, area) < tolerance,
                "{label} at scale {scale}: area {} vs {area}",
                reading.area
            );
            assert!(
                rel(reading.volume, volume, volume) < tolerance,
                "{label} at scale {scale}: volume {} vs {volume}",
                reading.volume
            );
        }
        let area_only = integrate_face_area(&topo, face, 5).unwrap();
        assert!(rel(area_only, area, area) < 1e-12, "{area_only} vs {area}");
    }
}
