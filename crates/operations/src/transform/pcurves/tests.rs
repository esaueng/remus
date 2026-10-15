#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_3, PI, TAU};

use remus_math::curves2d::{Circle2D, Curve2D, Ellipse2D, Line2D, NurbsCurve2D};
use remus_math::frame::Frame3;
use remus_math::mat::Mat4;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::vec::{Point2, Point3, Vec2, Vec3};
use remus_topology::Topology;
use remus_topology::coedge::PeriodicWinding;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::pcurve::PCurve;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use super::{ChartMap, UvAffine, derived_winding, reexpress_face_pcurves};
use crate::transform::{TransformRecorder, linear_determinant, transform_face_surface_recorded};

/// Rotation about an oblique axis, then a translation far from the origin.
fn rigid() -> Mat4 {
    Mat4::translation(37.0, -12.5, 8.0) * Mat4::rotation_z(0.7) * Mat4::rotation_x(-1.1)
}

fn matrices() -> [(&'static str, Mat4); 5] {
    [
        ("translation", Mat4::translation(10.0, 0.0, 0.0)),
        ("rotation", rigid()),
        ("mirror", rigid() * Mat4::scale(-1.0, 1.0, 1.0)),
        ("uniform scale", rigid() * Mat4::scale(2.5, 2.5, 2.5)),
        ("scaled mirror", rigid() * Mat4::scale(0.4, -0.4, 0.4)),
    ]
}

/// The canonical plane chart the pcurve consumers evaluate.
fn plane_point(normal: Vec3, d: f64, uv: Point2) -> Point3 {
    let foot = normal * (d / normal.dot(normal));
    let chart = Frame3::from_normal(Point3::new(foot.x(), foot.y(), foot.z()), normal).unwrap();
    chart.origin + chart.x * uv.x() + chart.y * uv.y()
}

fn surface_point(surface: &FaceSurface, uv: Point2) -> Point3 {
    match surface {
        FaceSurface::Plane { normal, d } => plane_point(*normal, *d, uv),
        other => other.evaluate(uv.x(), uv.y()).unwrap(),
    }
}

/// A face on `surface` bounded by a triangle through `corners`.
fn face_on(topo: &mut Topology, surface: FaceSurface, corners: [Point3; 3]) -> FaceId {
    let v = corners.map(|p| topo.add_vertex(Vertex::new(p, 1e-7)));
    let edges = [(0, 1), (1, 2), (2, 0)]
        .map(|(a, b)| topo.add_edge(Edge::new(v[a], v[b], EdgeCurve::Line)));
    let wire = topo.add_wire(
        Wire::new(
            edges.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
            true,
        )
        .unwrap(),
    );
    topo.add_face(Face::new(wire, Vec::new(), surface))
}

/// Moves the face's vertices, then its carrier, as `transform_solid` does.
fn transform_face(topo: &mut Topology, face: FaceId, matrix: &Mat4) -> ChartMap {
    let wire = topo.wire(topo.face(face).unwrap().outer_wire()).unwrap();
    let vertices: Vec<_> = wire
        .edges()
        .iter()
        .map(|oe| topo.edge(oe.edge()).unwrap().start())
        .collect();
    for vid in vertices {
        let moved = matrix.mul_point(topo.vertex(vid).unwrap().point());
        topo.vertex_mut(vid).unwrap().set_point(moved);
    }
    let normal_matrix = matrix.inverse().unwrap().transpose();
    let mut recorder = TransformRecorder::new(linear_determinant(matrix) < 0.0, true);
    transform_face_surface_recorded(topo, face, matrix, &normal_matrix, &mut recorder).unwrap()
}

fn carriers() -> Vec<(&'static str, FaceSurface, [Point3; 3])> {
    let normal = Vec3::new(0.3, -0.5, 0.8).normalize().unwrap();
    let on_plane = |p: Point3| p - normal * (normal.dot(Vec3::new(p.x(), p.y(), p.z())) - 4.0);
    vec![
        (
            "plane",
            FaceSurface::Plane { normal, d: 4.0 },
            [
                on_plane(Point3::new(1.0, 2.0, 3.0)),
                on_plane(Point3::new(5.0, 2.0, 3.0)),
                on_plane(Point3::new(1.0, 6.0, 3.0)),
            ],
        ),
        (
            "cylinder",
            FaceSurface::Cylinder(
                CylindricalSurface::with_ref_dir(
                    Point3::new(3.0, 21.0, 0.0),
                    Vec3::new(0.0, 0.0, 1.0),
                    3.0,
                    Vec3::new(0.0, 1.0, 0.0),
                )
                .unwrap(),
            ),
            [Point3::new(0.0, 0.0, 0.0); 3],
        ),
        (
            "cone",
            FaceSurface::Cone(
                ConicalSurface::with_ref_dir(
                    Point3::new(1.0, -2.0, 0.5),
                    Vec3::new(0.2, 0.1, 1.0),
                    0.6,
                    Vec3::new(0.0, 1.0, 0.0),
                )
                .unwrap(),
            ),
            [Point3::new(0.0, 0.0, 0.0); 3],
        ),
        (
            "sphere",
            FaceSurface::Sphere(
                SphericalSurface::with_frame(
                    Point3::new(-4.0, 2.0, 1.0),
                    2.0,
                    Vec3::new(0.0, 1.0, 1.0),
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .unwrap(),
            ),
            [Point3::new(0.0, 0.0, 0.0); 3],
        ),
        (
            "torus",
            FaceSurface::Torus(
                ToroidalSurface::with_axis_and_ref_dir(
                    Point3::new(2.0, 2.0, -3.0),
                    5.0,
                    1.5,
                    Vec3::new(1.0, 0.0, 1.0),
                    Vec3::new(0.0, 1.0, 0.0),
                )
                .unwrap(),
            ),
            [Point3::new(0.0, 0.0, 0.0); 3],
        ),
    ]
}

/// `new_surface(map(u, v)) = M · old_surface(u, v)` over the whole chart, for
/// every analytic carrier under every similarity: the identity that keeps a
/// re-expressed pcurve on its transformed edge.
#[test]
fn chart_maps_are_exact_images_of_the_transform() {
    for (carrier, surface, corners) in carriers() {
        for (what, matrix) in matrices() {
            let mut topo = Topology::new();
            let face = face_on(&mut topo, surface.clone(), corners);
            let chart = transform_face(&mut topo, face, &matrix);
            let moved = topo.face(face).unwrap().surface().clone();
            assert_eq!(moved.type_tag(), surface.type_tag(), "{carrier} / {what}");
            let map = match chart {
                ChartMap::Unchanged => UvAffine::diagonal(1.0, 1.0),
                ChartMap::Affine(map) => map,
                ChartMap::Replaced => panic!("{carrier} / {what}: similarity replaced the chart"),
            };
            for i in 0..=8 {
                for j in 0..=8 {
                    let uv = Point2::new(-0.5 + 1.1 * f64::from(i), -1.2 + 0.3 * f64::from(j));
                    let expected = matrix.mul_point(surface_point(&surface, uv));
                    let actual = surface_point(&moved, map.point(uv));
                    let scale = expected.0.iter().fold(1.0_f64, |m, x| m.max(x.abs()));
                    assert!(
                        (actual - expected).length() <= 1e-13 * scale,
                        "{carrier} / {what} at {uv:?}: {actual:?} != {expected:?}"
                    );
                }
            }
        }
    }
}

/// Only the pcurves move: every carrier is rebuilt exactly as before B130
/// (booleans are tuned on where those frames put the seam). Sphere and torus
/// frames are carried, so a rigid motion leaves their pcurves bit-exact.
#[test]
fn carriers_keep_their_frames_and_only_pcurves_move() {
    use crate::transform::{scaled_radius, transform_direction};
    for (what, matrix) in matrices() {
        for (carrier, surface, corners) in carriers().into_iter().skip(1) {
            let mut topo = Topology::new();
            let face = face_on(&mut topo, surface.clone(), corners);
            let chart = transform_face(&mut topo, face, &matrix);
            let moved = format!("{:?}", topo.face(face).unwrap().surface());
            // The transform's own column-norm arithmetic, for bit equality.
            let m = &matrix.0;
            let sx = (m[0][0] * m[0][0] + m[1][0] * m[1][0] + m[2][0] * m[2][0]).sqrt();
            let expected = match &surface {
                FaceSurface::Cylinder(old) => FaceSurface::Cylinder(
                    CylindricalSurface::new(
                        matrix.mul_point(old.origin()),
                        transform_direction(&matrix, old.axis()).unwrap(),
                        scaled_radius(&matrix, old.axis(), old.radius()),
                    )
                    .unwrap(),
                ),
                FaceSurface::Cone(old) => FaceSurface::Cone(
                    ConicalSurface::new(
                        matrix.mul_point(old.apex()),
                        transform_direction(&matrix, old.axis()).unwrap(),
                        old.half_angle(),
                    )
                    .unwrap(),
                ),
                FaceSurface::Sphere(old) => FaceSurface::Sphere(
                    SphericalSurface::with_frame(
                        matrix.mul_point(old.center()),
                        old.radius() * sx,
                        transform_direction(&matrix, old.z_axis()).unwrap(),
                        transform_direction(&matrix, old.x_axis()).unwrap(),
                    )
                    .unwrap(),
                ),
                FaceSurface::Torus(old) => FaceSurface::Torus(
                    ToroidalSurface::with_axis_and_ref_dir(
                        matrix.mul_point(old.center()),
                        old.major_radius() * sx,
                        old.minor_radius() * sx,
                        transform_direction(&matrix, old.z_axis()).unwrap(),
                        transform_direction(&matrix, old.x_axis()).unwrap(),
                    )
                    .unwrap(),
                ),
                FaceSurface::Plane { .. } | FaceSurface::Nurbs(_) => unreachable!(),
            };
            assert_eq!(moved, format!("{expected:?}"), "{carrier} / {what}");
            let carried_frame = matches!(surface, FaceSurface::Sphere(_) | FaceSurface::Torus(_));
            if carried_frame && linear_determinant(&matrix) > 0.0 {
                let ChartMap::Affine(map) = chart else {
                    panic!("{carrier} / {what}: {chart:?}");
                };
                assert!(map.is_identity(), "{carrier} / {what}: {map:?}");
            }
        }
    }
}

fn affines() -> [(&'static str, UvAffine); 5] {
    let (s, c) = 0.9_f64.sin_cos();
    let at = |u: Vec2, v: Vec2| UvAffine {
        origin: Point2::new(4.0, -7.5),
        u_image: u,
        v_image: v,
        conformal: u.dot(v).abs() < 1e-12 && (u.length() - v.length()).abs() < 1e-12,
    };
    [
        ("rotation", at(Vec2::new(c, s), Vec2::new(-s, c))),
        ("reflection", at(Vec2::new(c, s), Vec2::new(s, -c))),
        (
            "scaled rotation",
            at(Vec2::new(3.0 * c, 3.0 * s), Vec2::new(-3.0 * s, 3.0 * c)),
        ),
        ("shear", at(Vec2::new(1.0, 0.0), Vec2::new(1.7, 0.6))),
        (
            "anisotropic reflection",
            at(Vec2::new(0.0, 2.0), Vec2::new(0.5, 0.3)),
        ),
    ]
}

fn pcurves() -> Vec<(&'static str, PCurve)> {
    vec![
        (
            "line",
            PCurve::new(
                Curve2D::Line(Line2D::new(Point2::new(1.0, 2.0), Vec2::new(0.6, -0.8)).unwrap()),
                -1.5,
                4.0,
            ),
        ),
        (
            "circle",
            PCurve::new(
                Curve2D::Circle(Circle2D::new(Point2::new(-2.0, 1.0), 1.5).unwrap()),
                5.5,
                0.25,
            ),
        ),
        (
            "ellipse",
            PCurve::new(
                Curve2D::Ellipse(Ellipse2D::new(Point2::new(3.0, 3.0), 2.5, 0.75, 1.2).unwrap()),
                -FRAC_PI_3,
                PI + 0.4,
            ),
        ),
        (
            "rational nurbs",
            PCurve::new(
                Curve2D::Nurbs(
                    NurbsCurve2D::new(
                        2,
                        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                        vec![
                            Point2::new(0.0, 0.0),
                            Point2::new(1.0, 2.0),
                            Point2::new(3.0, 0.5),
                        ],
                        vec![1.0, 0.4, 1.0],
                    )
                    .unwrap(),
                ),
                0.1,
                0.9,
            ),
        ),
    ]
}

/// Each image is the source traced at the same normalized parameter, which
/// is what keeps a use same-parameter with its edge.
#[test]
fn pcurve_images_preserve_the_normalized_parameter() {
    for (what, map) in affines() {
        for (kind, pcurve) in pcurves() {
            let image = map.apply(&pcurve).unwrap();
            for k in 0..=32 {
                let f = f64::from(k) / 32.0;
                let source = pcurve.t_start() + f * (pcurve.t_end() - pcurve.t_start());
                let target = image.t_start() + f * (image.t_end() - image.t_start());
                let expected = map.point(pcurve.evaluate(source));
                let actual = image.evaluate(target);
                assert!(
                    (actual - expected).length() < 1e-12,
                    "{kind} / {what} at f = {f}: {actual:?} != {expected:?}"
                );
            }
        }
    }
}

/// Circles stay circles exactly when the map is conformal; a reflection
/// reverses the parameter direction.
#[test]
fn conic_images_keep_their_family_and_orientation() {
    let circle = &pcurves()[1].1;
    for (what, map) in affines() {
        let image = map.apply(circle).unwrap();
        let conformal = !what.contains("shear") && !what.contains("anisotropic");
        assert_eq!(
            matches!(image.curve(), Curve2D::Circle(_)),
            conformal,
            "{what}: {:?}",
            image.curve()
        );
        let reversing = what.contains("reflection");
        let source_sweep = circle.t_end() - circle.t_start();
        let image_sweep = image.t_end() - image.t_start();
        assert_eq!(
            image_sweep.signum() == source_sweep.signum(),
            !reversing,
            "{what}"
        );
        assert!(
            (image_sweep.abs() - source_sweep.abs()).abs() < 1e-12,
            "{what}"
        );
    }
}

/// A deliberate near-unit anisotropy is kept: the circle's image becomes the
/// exact ellipse instead of a circle with averaged radii, which would miss a
/// far vertex by the dropped stretch times the radius.
#[test]
fn near_unit_anisotropy_keeps_the_ellipse() {
    let stretch = 1.0 + 2e-15;
    let map = UvAffine {
        origin: Point2::new(0.0, 0.0),
        u_image: Vec2::new(stretch, 0.0),
        v_image: Vec2::new(0.0, 1.0),
        conformal: false,
    };
    let radius = 1e9;
    let circle = PCurve::new(
        Curve2D::Circle(Circle2D::new(Point2::new(0.0, 0.0), radius).unwrap()),
        0.0,
        1.0,
    );
    let image = map.apply(&circle).unwrap();
    assert!(
        matches!(image.curve(), Curve2D::Ellipse(_)),
        "{:?}",
        image.curve()
    );
    let expected = Point2::new(radius * stretch, 0.0);
    let actual = image.evaluate(image.t_start());
    assert!(
        (actual - expected).length() < 1e-7,
        "{actual:?} != {expected:?}"
    );
    // The exactly conformal image of a circle stays a circle.
    let identity = UvAffine::diagonal(1.0, 1.0);
    let same = identity.apply(&circle).unwrap();
    assert!(
        matches!(same.curve(), Curve2D::Circle(_)),
        "{:?}",
        same.curve()
    );
}

/// A cylinder or cone chart scales `v` by the map's axial scale, so it is
/// conformal only when that scale is exactly one: a deliberate `1 + 2e-15`
/// scale must turn a circle pcurve into the exact ellipse, while a
/// translation keeps the circle.
#[test]
fn near_unit_axial_scale_keeps_revolved_circles_exact() {
    let stretch = 1.0 + 2e-15;
    let radius = 1e9;
    let circle = PCurve::new(
        Curve2D::Circle(Circle2D::new(Point2::new(0.3, 0.2), radius).unwrap()),
        0.0,
        TAU,
    );
    for (carrier, surface, corners) in carriers() {
        if !matches!(surface, FaceSurface::Cylinder(_) | FaceSurface::Cone(_)) {
            continue;
        }
        let mut topo = Topology::new();
        let face = face_on(&mut topo, surface.clone(), corners);
        let ChartMap::Affine(map) =
            transform_face(&mut topo, face, &Mat4::scale(stretch, stretch, stretch))
        else {
            panic!("{carrier}: a near-unit similarity must map the chart affinely");
        };
        let image = map.apply(&circle).unwrap();
        for k in 0..=8 {
            let f = f64::from(k) / 8.0;
            let t = TAU * f;
            let expected = map.point(circle.evaluate(t));
            let actual = image.evaluate(image.t_start() + f * (image.t_end() - image.t_start()));
            // A few ulps at radius 1e9; the averaged-radius circle misses
            // by `radius·(stretch − 1)/2 ≈ 1e-6`.
            assert!(
                (actual - expected).length() <= 3e-7,
                "{carrier}: {:?} traces {actual:?}, not {expected:?}",
                image.curve()
            );
        }

        let mut topo = Topology::new();
        let face = face_on(&mut topo, surface, corners);
        let ChartMap::Affine(map) =
            transform_face(&mut topo, face, &Mat4::translation(10.0, -3.0, 4.0))
        else {
            panic!("{carrier}: a translation must map the chart affinely");
        };
        let image = map.apply(&circle).unwrap();
        assert!(
            matches!(image.curve(), Curve2D::Circle(_)),
            "{carrier}: {:?}",
            image.curve()
        );
    }
}

#[test]
fn derived_winding_follows_the_principal_branch() {
    let cylinder = FaceSurface::Cylinder(
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap(),
    );
    let FaceSurface::Cylinder(carrier) = &cylinder else {
        unreachable!()
    };
    let (bottom, top) = (carrier.evaluate(0.0, 0.0), carrier.evaluate(0.0, 3.0));
    let seam = |u: f64| {
        PCurve::new(
            Curve2D::Line(Line2D::new(Point2::new(u, 0.0), Vec2::new(0.0, 1.0)).unwrap()),
            0.0,
            3.0,
        )
    };
    assert_eq!(
        derived_winding(&cylinder, &seam(0.0), bottom, top),
        Some(PeriodicWinding::ZERO)
    );
    assert_eq!(
        derived_winding(&cylinder, &seam(TAU), bottom, top),
        Some(PeriodicWinding::new(1, 0))
    );
    assert_eq!(
        derived_winding(&cylinder, &seam(-2.0 * TAU), bottom, top),
        Some(PeriodicWinding::new(-2, 0))
    );
    // Not an integral lift: the use misses its vertex, so no count is derived.
    assert_eq!(derived_winding(&cylinder, &seam(1.0), bottom, top), None);
}

/// A replaced chart drops each pcurve with its lift count: a count left
/// without its pcurve fails boundary authority.
#[test]
fn replaced_chart_drops_pcurves_and_their_windings() {
    let mut topo = Topology::new();
    let surface = carriers()[1].1.clone();
    let face = face_on(
        &mut topo,
        surface,
        [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
    );
    let edge = topo
        .wire(topo.face(face).unwrap().outer_wire())
        .unwrap()
        .edges()[0]
        .edge();
    let pcurve = pcurves()[0].1.clone();
    topo.set_pcurve_oriented(edge, face, true, pcurve).unwrap();
    let coedge = topo.loops_of_face(face).unwrap()[0];
    let coedge = topo.face_loop(coedge).unwrap().coedges()[0];
    topo.set_coedge_periodic_winding(coedge, PeriodicWinding::new(1, 0))
        .unwrap();

    reexpress_face_pcurves(&mut topo, face, &ChartMap::Replaced).unwrap();

    assert!(topo.pcurves_for_face(face).is_empty());
    assert_eq!(
        topo.coedge(coedge).unwrap().periodic_winding(),
        PeriodicWinding::ZERO
    );
    remus_topology::validation::validate_boundary_authority(&topo).unwrap();
}

/// Retained carrier reconciliation must not replace an exact nonconformal
/// ellipse or adopt an unrelated edge's radius.
#[test]
fn retained_planar_circle_reconciles_only_analytic_roundoff() {
    let radius = 1e9;
    let source = PCurve::new(
        Curve2D::Circle(Circle2D::new(Point2::new(0.0, 0.0), radius).unwrap()),
        0.0,
        PI,
    );
    let near = UvAffine::diagonal(1.0, 1.0 + 3e-15);
    assert!(matches!(
        near.apply(&source).unwrap().curve(),
        Curve2D::Ellipse(_)
    ));
    let retained = near.apply_retained_planar_circle(&source, radius).unwrap();
    let Curve2D::Circle(circle) = retained.curve() else {
        panic!("locally retained circle")
    };
    assert_eq!(circle.radius().to_bits(), radius.to_bits());
    for map in [UvAffine::diagonal(2.0, 1.0), UvAffine::diagonal(1.0, 1.0)] {
        let expected = map.apply(&source).unwrap();
        let guarded = map
            .apply_retained_planar_circle(&source, 1.01 * radius)
            .unwrap();
        assert_eq!(format!("{guarded:?}"), format!("{expected:?}"));
    }
}

#[test]
fn retained_planar_ellipse_reconciles_only_analytic_roundoff() {
    let source = PCurve::new(
        Curve2D::Ellipse(Ellipse2D::new(Point2::new(1.0, -2.0), 2.0, 1.0, 0.2).unwrap()),
        0.0,
        1.0,
    );
    let map = UvAffine::diagonal(1.0, 1.0);
    let major = 2.0 * (1.0 + 4.0 * f64::EPSILON);
    let reconciled = map
        .apply_retained_planar_ellipse(&source, major, 1.0)
        .unwrap();
    let Curve2D::Ellipse(ellipse) = reconciled.curve() else {
        panic!("ellipse image")
    };
    assert_eq!(ellipse.semi_major().to_bits(), major.to_bits());
    let expected = map.apply(&source).unwrap();
    for (a, b) in [(2.1, 1.0), (1.0, 2.0), (f64::NAN, 1.0)] {
        let guarded = map.apply_retained_planar_ellipse(&source, a, b).unwrap();
        assert_eq!(format!("{guarded:?}"), format!("{expected:?}"));
    }
}

#[test]
fn seam_pcurves_keep_both_exact_images_or_drop_both_replaced_uses() {
    // The cylinder's seam is used twice by its lateral face. A uniform
    // scale stretches only the axial `v`, so the chart map is not
    // conformal: the exact mapper retains both the ellipse and line images.
    let mut topo = Topology::new();
    let solid = crate::primitives::make_cylinder(&mut topo, 1.0, 2.0).unwrap();
    let lateral = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let mut uses = Vec::new();
    for &loop_id in topo.face(lateral).unwrap().boundary_loops() {
        uses.extend_from_slice(topo.face_loop(loop_id).unwrap().coedges());
    }
    let seam: Vec<_> = uses
        .into_iter()
        .filter(|&coedge| {
            let edge = topo.coedge(coedge).unwrap().edge();
            matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Line)
        })
        .collect();
    assert_eq!(seam.len(), 2, "cylinder seam uses");
    // The line branch lies exactly on its use, so it maps and certifies
    // on its own; the exact affine mapper keeps its ellipse partner too.
    let surface = topo.face(lateral).unwrap().surface().clone();
    let data = topo.coedge(seam[0]).unwrap();
    let edge = topo.edge(data.edge()).unwrap();
    let (from, to) = if data.is_forward() {
        (edge.start(), edge.end())
    } else {
        (edge.end(), edge.start())
    };
    let from = surface
        .project_point(topo.vertex(from).unwrap().point())
        .unwrap();
    let to = surface
        .project_point(topo.vertex(to).unwrap().point())
        .unwrap();
    let line = PCurve::new(
        Curve2D::Line(
            Line2D::new(Point2::new(from.0, from.1), Vec2::new(0.0, to.1 - from.1)).unwrap(),
        ),
        0.0,
        (to.1 - from.1).abs(),
    );
    let ellipse = PCurve::new(
        Curve2D::Ellipse(Ellipse2D::new(Point2::new(0.0, 1.0), 1.0, 0.5, 0.0).unwrap()),
        0.0,
        1.0,
    );
    topo.set_coedge_pcurve(seam[0], line).unwrap();
    topo.set_coedge_pcurve(seam[1], ellipse).unwrap();

    super::super::transform_solid(&mut topo, solid, &Mat4::scale(2.0, 2.0, 2.0)).unwrap();
    let kept: Vec<bool> = seam
        .iter()
        .map(|&coedge| topo.coedge(coedge).unwrap().pcurve().is_some())
        .collect();
    assert_eq!(kept, [true, true], "both exact affine images must survive");
    remus_topology::validation::validate_boundary_authority(&topo).unwrap();
    reexpress_face_pcurves(&mut topo, lateral, &ChartMap::Replaced).unwrap();
    assert!(
        seam.iter()
            .all(|&id| topo.coedge(id).unwrap().pcurve().is_none())
    );
    remus_topology::validation::validate_boundary_authority(&topo).unwrap();
    for &coedge in &seam {
        assert_eq!(
            topo.coedge(coedge).unwrap().periodic_winding(),
            PeriodicWinding::ZERO
        );
    }
}
