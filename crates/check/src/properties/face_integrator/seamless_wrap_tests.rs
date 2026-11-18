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

use super::cone_green_tests::{Section, closed_nurbs_edge, refined, rel, reversed, sections};
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

/// The band on a cylinder of radius `r` between the circle `z = 0` and the
/// oblique section `z = h + s x`, with no seam, either one the outer wire:
/// the closed-form area and volume term on its boundary and on the sampled
/// trim. With the circle outer, the polar-cap path once read the band from
/// it to the section's mean height (21.99 for 18.85).
#[test]
fn a_seamless_band_under_an_oblique_cylinder_rim_reads_its_closed_form() {
    for (scale, circle_outer) in [1e-3, 1.0, 1e3]
        .into_iter()
        .flat_map(|scale| [(scale, false), (scale, true)])
    {
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
        let (outer, inner) = if circle_outer {
            (bottom, top)
        } else {
            (top, bottom)
        };
        let face = topo.add_face(Face::new(
            outer,
            vec![inner],
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
                "{label} at scale {scale} (circle outer {circle_outer}): area {} vs {area}",
                reading.area
            );
            assert!(
                rel(reading.volume, volume, volume) < tolerance,
                "{label} at scale {scale} (circle outer {circle_outer}): volume {} vs {volume}",
                reading.volume
            );
        }
        let area_only = integrate_face_area(&topo, face, 5).unwrap();
        assert!(rel(area_only, area, area) < 1e-12, "{area_only} vs {area}");
    }
}

/// The circle where the cone of `section` meets the plane normal to its axis
/// `height` above the apex, as a section at `θ = 0`.
fn latitude(section: &Section, height: f64) -> Section {
    Section {
        theta: 0.0,
        d: height,
        ..*section
    }
}

/// `latitude`'s rim as an exact circle, run in `−u` (the way a pointed
/// face's rim runs), starting at parameter `start`.
fn latitude_rim(
    topo: &mut Topology,
    section: &Section,
    start: f64,
) -> remus_topology::edge::EdgeId {
    let radius = section.d / section.a.tan();
    let circle = Circle3D::new_with_ref(
        section.apex + section.axis * section.d,
        -section.axis,
        radius,
        section.tilt_dir,
    )
    .unwrap();
    let vertex = topo.add_vertex(Vertex::new(circle.evaluate(start), 1e-9));
    let mut edge = Edge::new(vertex, vertex, EdgeCurve::Circle(circle));
    edge.set_trim(Some((start, start + TAU)));
    topo.add_edge(edge)
}

/// A pointed cone face whose seam to the apex is one edge used both ways
/// (the primitive's layout) or two coincident edges, under a latitude
/// circle starting anywhere round it: the closed-form cone in every
/// reading. Review of #1859: with two seam edges the strand reading took the
/// face and the apex, which projects to an arbitrary longitude, read it
/// 13.8 % low for a rim starting at 1.3.
#[test]
fn a_pointed_face_with_a_seam_to_the_apex_reads_the_closed_form_cone() {
    let base = &sections(1.0)[1];
    for (a, start, two_edges) in [0.6, 1.0].into_iter().flat_map(|a| {
        [0.0, 1.3, 4.0]
            .into_iter()
            .flat_map(move |start| [(a, start, false), (a, start, true)])
    }) {
        let section = latitude(&Section { a, ..*base }, 2.0);
        let mut topo = Topology::new();
        let rim = latitude_rim(&mut topo, &section, start);
        let rim_vertex = topo.edge(rim).unwrap().start();
        let apex = topo.add_vertex(Vertex::new(section.apex, 1e-9));
        let up = topo.add_edge(Edge::new(rim_vertex, apex, EdgeCurve::Line));
        let down = if two_edges {
            OrientedEdge::new(
                topo.add_edge(Edge::new(apex, rim_vertex, EdgeCurve::Line)),
                true,
            )
        } else {
            OrientedEdge::new(up, false)
        };
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(rim, true),
                    OrientedEdge::new(up, true),
                    down,
                ],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cone(section.cone())));
        let reference = section.apex + Vec3::new(0.5, 0.25, -1.0);
        let (volume, _) = section.lateral_terms(reference);
        let area = lateral_area(&section);
        let label = format!("a {a}, start {start}, two seam edges {two_edges}");
        for (what, reading) in [
            (
                "fixed",
                integrate_face_fixed_about(&topo, face, 5, reference).unwrap(),
            ),
            (
                "default",
                integrate_face_about(&topo, face, &PropertiesOptions::default(), reference)
                    .unwrap(),
            ),
            (
                "sampled",
                integrate_face_about(&topo, face, &refined(), reference).unwrap(),
            ),
        ] {
            assert!(
                rel(reading.area, area, area) < 1e-10,
                "{label} {what}: area {} vs {area}",
                reading.area
            );
            assert!(
                rel(reading.volume, volume, section.d.powi(3)) < 1e-10,
                "{label} {what}: volume {} vs {volume}",
                reading.volume
            );
        }
        let area_only = integrate_face_area(&topo, face, 5).unwrap();
        assert!(
            rel(area_only, area, area) < 1e-10,
            "{label}: area-only {area_only}"
        );
    }
}

/// The band between a latitude circle (the outer wire) and an oblique rim
/// below it (an inner wire, run either way): the latitude's cone less the
/// section's. Review of #1859: the polar-cap path once read it from the
/// latitude to the rim's mean height (41.71 for 29.88).
#[test]
fn a_band_over_an_oblique_inner_rim_reads_the_difference_of_two_cones() {
    for scale in [1e-3, 1.0, 1e3] {
        for (index, section) in sections(scale).iter().take(2).enumerate() {
            for rim_along_u in [true, false] {
                let top = latitude(section, 3.0 * section.d);
                let mut topo = Topology::new();
                let outer_rim = latitude_rim(&mut topo, &top, 0.0);
                let outer = topo
                    .add_wire(Wire::new(vec![OrientedEdge::new(outer_rim, true)], true).unwrap());
                let rim = if rim_along_u {
                    reversed(&section.rim())
                } else {
                    section.rim()
                };
                let (inner_rim, _) = closed_nurbs_edge(&mut topo, rim);
                let inner = topo
                    .add_wire(Wire::new(vec![OrientedEdge::new(inner_rim, true)], true).unwrap());
                let face = topo.add_face(Face::new(
                    outer,
                    vec![inner],
                    FaceSurface::Cone(section.cone()),
                ));
                let reference = section.apex + Vec3::new(0.5, 0.25, -1.0) * scale;
                let volume = top.lateral_terms(reference).0 - section.lateral_terms(reference).0;
                let area = lateral_area(&top) - lateral_area(section);
                let v_scale = top.d.powi(3);
                let label = format!("section {index} at scale {scale}, rim along u {rim_along_u}");
                let reading =
                    integrate_face_about(&topo, face, &PropertiesOptions::default(), reference)
                        .unwrap();
                // Wound the kernel's way (the lower rim along `u`) the boundary
                // reading takes the face exactly; wound the other way it
                // keeps the sampled trim, right to its chords.
                let tolerance = if rim_along_u { 1e-12 } else { 1e-3 };
                assert!(
                    rel(reading.area, area, area) < tolerance,
                    "{label}: area {} vs {area}",
                    reading.area
                );
                assert!(
                    rel(reading.volume, volume, v_scale) < tolerance,
                    "{label}: volume {} vs {volume}",
                    reading.volume
                );
                let sampled = integrate_face_about(&topo, face, &refined(), reference).unwrap();
                assert!(
                    rel(sampled.area, area, area) < 1e-3,
                    "{label}: sampled area {} vs {area}",
                    sampled.area
                );
            }
        }
    }
}
