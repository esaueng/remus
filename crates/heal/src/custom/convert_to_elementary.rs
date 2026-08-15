//! Convert NURBS geometry to analytic (elementary) surfaces and curves
//! where possible.

use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use remus_math::tolerance::Tolerance;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use remus_geometry::convert::{
    RecognizedCurve, RecognizedSurface, recognize_curve, recognize_surface,
};

use crate::HealError;

/// Try to recognize and replace NURBS surfaces with analytic equivalents.
///
/// Returns the number of surfaces converted.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups fail.
pub fn convert_to_elementary(
    topo: &mut Topology,
    solid_id: SolidId,
    tolerance: &Tolerance,
) -> Result<usize, HealError> {
    // Walk outer shell *and* inner (cavity) shells via the topology
    // explorer helper. Hollow solids (cavities from `shell_op` or
    // boolean cuts) hold faces in `Solid::inner_shells()`, and
    // visiting only the outer shell would silently leave those faces
    // unconverted.
    let face_ids: Vec<FaceId> = solid_faces(topo, solid_id)?;

    let mut converted = 0;

    let surfaces: Vec<(FaceId, FaceSurface)> = face_ids
        .iter()
        .map(|&fid| topo.face(fid).map(|f| (fid, f.surface().clone())))
        .collect::<Result<Vec<_>, _>>()?;

    for (fid, surface) in &surfaces {
        if let FaceSurface::Nurbs(nurbs) = surface {
            match recognize_surface(nurbs, tolerance.linear) {
                RecognizedSurface::Plane { normal, d } => {
                    let face = topo.face_mut(*fid)?;
                    face.set_surface(FaceSurface::Plane { normal, d });
                    converted += 1;
                }
                RecognizedSurface::Cylinder {
                    origin,
                    axis,
                    radius,
                } => {
                    if let Ok(cyl) =
                        remus_math::surfaces::CylindricalSurface::new(origin, axis, radius)
                    {
                        let face = topo.face_mut(*fid)?;
                        face.set_surface(FaceSurface::Cylinder(cyl));
                        converted += 1;
                    }
                }
                RecognizedSurface::Sphere { center, radius } => {
                    if let Ok(sph) = remus_math::surfaces::SphericalSurface::new(center, radius) {
                        let face = topo.face_mut(*fid)?;
                        face.set_surface(FaceSurface::Sphere(sph));
                        converted += 1;
                    }
                }
                RecognizedSurface::Cone {
                    apex,
                    axis,
                    half_angle,
                } => {
                    if let Ok(cone) =
                        remus_math::surfaces::ConicalSurface::new(apex, axis, half_angle)
                    {
                        let face = topo.face_mut(*fid)?;
                        face.set_surface(FaceSurface::Cone(cone));
                        converted += 1;
                    }
                }
                RecognizedSurface::Torus {
                    center,
                    axis,
                    major_radius,
                    minor_radius,
                } => {
                    if let Ok(torus) = remus_math::surfaces::ToroidalSurface::with_axis(
                        center,
                        major_radius,
                        minor_radius,
                        axis,
                    ) {
                        let face = topo.face_mut(*fid)?;
                        face.set_surface(FaceSurface::Torus(torus));
                        converted += 1;
                    }
                }
                RecognizedSurface::NotRecognized => {}
            }
        }
    }

    Ok(converted)
}

/// Try to recognize and replace NURBS edges with analytic curves.
///
/// Iterates every edge in the solid; if the edge has an
/// [`EdgeCurve::NurbsCurve`] that
/// `recognize_curve` identifies as a line, circle, or ellipse, replaces the edge's
/// curve with the analytic form. Returns the number of curves
/// converted.
///
/// Hyperbolic and parabolic arcs convert to
/// [`EdgeCurve::Hyperbola`] / [`EdgeCurve::Parabola`]. Both carry the
/// recognized in-plane axis explicitly (via `with_axes`), because
/// `Hyperbola3D::new` / `Parabola3D::new` derive that axis from an
/// arbitrary perpendicular and would silently rotate the curve within
/// its plane. Ellipses carry it for the same reason: `Ellipse3D::new`
/// would put `semi_major` along a frame of its own choosing, which is a
/// rotation for every ellipse that is not a circle.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups fail.
pub fn convert_edges_to_elementary(
    topo: &mut Topology,
    solid_id: SolidId,
    tolerance: &Tolerance,
) -> Result<usize, HealError> {
    let face_ids: Vec<FaceId> = solid_faces(topo, solid_id)?;

    // Collect unique edge IDs across all faces (edges may be shared
    // between faces).
    let mut edge_ids: Vec<EdgeId> = Vec::new();
    let mut seen = std::collections::HashSet::new();
    for &fid in &face_ids {
        let face = topo.face(fid)?;
        for &wid in std::iter::once(&face.outer_wire()).chain(face.inner_wires()) {
            let wire = topo.wire(wid)?;
            for oe in wire.edges() {
                let eid = oe.edge();
                if seen.insert(eid.index()) {
                    edge_ids.push(eid);
                }
            }
        }
    }

    let mut converted = 0;
    for eid in edge_ids {
        let edge = topo.edge(eid)?;
        let nurbs = match edge.curve() {
            EdgeCurve::NurbsCurve(n) => n.clone(),
            // Already analytic — nothing to convert.
            EdgeCurve::Line
            | EdgeCurve::Circle(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => continue,
        };
        match recognize_curve(&nurbs, tolerance.linear) {
            RecognizedCurve::Circle {
                center,
                normal,
                radius,
            } => {
                if let Ok(c) = Circle3D::new(center, normal, radius) {
                    let edge_mut = topo.edge_mut(eid)?;
                    edge_mut.set_curve(EdgeCurve::Circle(c));
                    converted += 1;
                }
            }
            RecognizedCurve::Ellipse {
                center,
                normal,
                u_axis,
                semi_major,
                semi_minor,
            } => {
                // `with_axes`, not `new`: `Ellipse3D::new` derives its own
                // in-plane frame from the normal alone (`Frame3::from_normal`)
                // and puts `semi_major` along it, so the recognized major-axis
                // direction is replaced by an arbitrary one. That rotates the
                // ellipse inside its plane by the angle between the two
                // frames — a different point set, published as an exact
                // analytic conversion. Only a circle is exempt, because
                // rotation about its own normal is the identity on it.
                //
                // The recognizer's `u_axis` is an eigenvector, so its sign is
                // arbitrary, but unlike the hyperbola that costs nothing
                // here: taking `v = normal × u` flips `v` with `u`, and a
                // 180° rotation maps an ellipse onto itself. Only the
                // parameter origin moves, and `domain_with_endpoints`
                // re-projects the edge's vertices onto whatever frame it is
                // given, so the trimmed arc is unaffected.
                let v_axis = normal.cross(u_axis);
                if let Ok(e) =
                    Ellipse3D::with_axes(center, normal, semi_major, semi_minor, u_axis, v_axis)
                {
                    let edge_mut = topo.edge_mut(eid)?;
                    edge_mut.set_curve(EdgeCurve::Ellipse(e));
                    converted += 1;
                }
            }
            RecognizedCurve::Line { .. } => {
                // EdgeCurve::Line stores no geometry — vertex
                // positions imply the line. Replace the NURBS with
                // the implicit Line variant.
                let edge_mut = topo.edge_mut(eid)?;
                edge_mut.set_curve(EdgeCurve::Line);
                converted += 1;
            }
            RecognizedCurve::Hyperbola {
                center,
                normal,
                u_axis,
                semi_major,
                semi_minor,
            } => {
                // `with_axes`, not `new`: `Hyperbola3D::new` picks an
                // arbitrary in-plane u-axis, which would rotate the branch
                // inside its plane and yield a different point set.
                //
                // The recognizer's u_axis is an eigenvector, so its SIGN is
                // arbitrary (`0.5·atan2(B, A−C)` always lands in the
                // half-plane `cos θ ≥ 0`). `Hyperbola3D` represents only
                // the `+u` branch, so an unflipped axis would mirror the
                // edge onto the opposite branch — the classic silent
                // wrong-geometry conversion. Pick the sign from the curve
                // itself.
                let u_axis = orient_hyperbola_axis(&nurbs, center, u_axis);
                if let Ok(h) =
                    Hyperbola3D::with_axes(center, normal, u_axis, semi_major, semi_minor)
                {
                    let edge_mut = topo.edge_mut(eid)?;
                    edge_mut.set_curve(EdgeCurve::Hyperbola(h));
                    converted += 1;
                }
            }
            RecognizedCurve::Parabola {
                vertex,
                normal,
                axis_dir,
                focal_length,
            } => {
                // The recognized `normal` fixes the parabola's plane, which
                // `Parabola3D::new` cannot represent (it takes only the
                // symmetry axis). `u_axis = normal × axis_dir` recovers the
                // in-plane direction, so the plane survives the conversion.
                let u_axis = normal.cross(axis_dir);
                if let Ok(p) = Parabola3D::with_axes(vertex, axis_dir, u_axis, focal_length) {
                    let edge_mut = topo.edge_mut(eid)?;
                    edge_mut.set_curve(EdgeCurve::Parabola(p));
                    converted += 1;
                }
            }
            RecognizedCurve::NotRecognized => {}
        }
    }

    Ok(converted)
}

/// Choose the sign of a recognized hyperbola's real axis so it points at
/// the branch the curve actually lies on.
///
/// `try_recognize_hyperbola` returns an eigenvector, which is defined only
/// up to sign, while [`Hyperbola3D`] represents just the `+u_axis` branch.
/// Sampling the source curve and taking the sign of `(P − center)·u_axis`
/// resolves it exactly: every point of one branch has the same sign of that
/// projection, and `|(P − center)·u| ≥ semi_major > 0` there, so the test
/// never sits near zero and needs no tolerance.
fn orient_hyperbola_axis(
    nurbs: &remus_math::nurbs::curve::NurbsCurve,
    center: remus_math::vec::Point3,
    u_axis: remus_math::vec::Vec3,
) -> remus_math::vec::Vec3 {
    let (t0, t1) = nurbs.domain();
    let sample = nurbs.evaluate(f64::midpoint(t0, t1));
    if (sample - center).dot(u_axis) < 0.0 {
        -u_axis
    } else {
        u_axis
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use remus_geometry::convert::curve_to_nurbs::circle_to_nurbs;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::edge::Edge;
    use remus_topology::face::Face;
    use remus_topology::shell::Shell;
    use remus_topology::solid::Solid;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    #[test]
    fn convert_edges_to_elementary_recovers_circle() {
        // Build a minimal solid with one face whose boundary contains
        // a NURBS edge that's actually a full circle. After running
        // `convert_edges_to_elementary`, the edge should be a Circle3D
        // EdgeCurve.
        let mut topo = Topology::new();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.5).unwrap();
        let nurbs = circle_to_nurbs(&circle, 0.0, std::f64::consts::TAU).unwrap();

        // Closed circle: start_vertex == end_vertex.
        let v = topo.add_vertex(Vertex::new(Point3::new(2.5, 0.0, 0.0), 1e-7));
        let edge_id = topo.add_edge(Edge::new(v, v, EdgeCurve::NurbsCurve(nurbs)));

        // Wrap in a wire / face / shell / solid scaffold so the iterator
        // in `convert_edges_to_elementary` can find the edge.
        let wire = Wire::new(vec![OrientedEdge::new(edge_id, true)], true).unwrap();
        let wid = topo.add_wire(wire);
        let face_id = topo.add_face(Face::new(
            wid,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let shell_id = topo.add_shell(Shell::new(vec![face_id]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell_id, vec![]));

        let tol = Tolerance::new();
        let n = convert_edges_to_elementary(&mut topo, solid_id, &tol).unwrap();
        assert_eq!(n, 1, "expected 1 conversion, got {n}");

        // Verify the edge is now Circle3D, not NurbsCurve.
        let edge = topo.edge(edge_id).unwrap();
        match edge.curve() {
            EdgeCurve::Circle(c) => {
                assert!(
                    (c.radius() - 2.5).abs() < 1e-6,
                    "radius {} vs 2.5",
                    c.radius()
                );
            }
            other => panic!("expected Circle, got {other:?}"),
        }
    }

    #[test]
    fn convert_edges_skips_already_analytic() {
        // An edge that's already EdgeCurve::Line should not be touched.
        let mut topo = Topology::new();
        let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let edge_id = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));

        // Build the minimum scaffold (degenerate face/shell/solid).
        let wire = Wire::new(vec![OrientedEdge::new(edge_id, true)], false).unwrap();
        let wid = topo.add_wire(wire);
        let face_id = topo.add_face(Face::new(
            wid,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let shell_id = topo.add_shell(Shell::new(vec![face_id]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell_id, vec![]));

        let tol = Tolerance::new();
        let n = convert_edges_to_elementary(&mut topo, solid_id, &tol).unwrap();
        assert_eq!(n, 0, "Line edges shouldn't be converted, got {n}");
    }

    #[test]
    fn convert_walks_inner_shells() {
        // A solid with both an outer shell and an inner (cavity) shell
        // should have faces recognized on BOTH shells. Regression for
        // the prior outer-shell-only behavior, which silently left
        // cavity faces unconverted in hollow solids.
        use crate::construct::convert_surface::sphere_to_nurbs;
        use remus_math::surfaces::SphericalSurface;

        let mut topo = Topology::new();

        // Build two scaffolds — an outer "face" (planar) and an inner
        // "face" carrying a NURBS sphere surface that should be
        // recognized back as Sphere.
        let outer_face = {
            let v = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
            let edge_id = topo.add_edge(Edge::new(v, v, EdgeCurve::Line));
            let wire = Wire::new(vec![OrientedEdge::new(edge_id, true)], true).unwrap();
            let wid = topo.add_wire(wire);
            topo.add_face(Face::new(
                wid,
                vec![],
                FaceSurface::Plane {
                    normal: Vec3::new(0.0, 0.0, 1.0),
                    d: 0.0,
                },
            ))
        };

        let sphere = SphericalSurface::new(Point3::new(5.0, 0.0, 0.0), 1.0).unwrap();
        let nurbs_sphere = sphere_to_nurbs(&sphere).unwrap();
        let inner_face = {
            let v = topo.add_vertex(Vertex::new(Point3::new(6.0, 0.0, 0.0), 1e-7));
            let edge_id = topo.add_edge(Edge::new(v, v, EdgeCurve::Line));
            let wire = Wire::new(vec![OrientedEdge::new(edge_id, true)], true).unwrap();
            let wid = topo.add_wire(wire);
            topo.add_face(Face::new(wid, vec![], FaceSurface::Nurbs(nurbs_sphere)))
        };

        let outer_shell = topo.add_shell(Shell::new(vec![outer_face]).unwrap());
        let inner_shell = topo.add_shell(Shell::new(vec![inner_face]).unwrap());
        let solid_id = topo.add_solid(Solid::new(outer_shell, vec![inner_shell]));

        let tol = Tolerance::new();
        let converted = convert_to_elementary(&mut topo, solid_id, &tol).unwrap();
        assert_eq!(
            converted, 1,
            "should have converted the cavity-shell sphere face, got {converted}"
        );

        // The outer face was already analytic; the inner-shell face
        // should now be Sphere, not NURBS.
        assert!(matches!(
            topo.face(outer_face).unwrap().surface(),
            FaceSurface::Plane { .. }
        ));
        match topo.face(inner_face).unwrap().surface() {
            FaceSurface::Sphere(s) => {
                assert!(
                    (s.radius() - 1.0).abs() < 1e-6,
                    "recovered sphere radius {} should be ~1.0",
                    s.radius()
                );
            }
            other => panic!("expected inner-shell face to be Sphere, got {other:?}"),
        }
    }

    #[test]
    fn converts_a_rotated_ellipse_without_rotating_it() {
        // The recognizer returns the major-axis direction it measured.
        // `Ellipse3D::new` ignores it and derives its own in-plane frame
        // from the normal alone, which rotates the ellipse inside its
        // plane by whatever angle separates the two frames. The result is
        // published as an exact analytic conversion with no refusal.
        //
        // This case is built deliberately OFF the axes (major axis at 37°
        // in its plane) and deliberately non-circular (a/b = 2.5). Both
        // matter: an axis-aligned ellipse passes whether or not the u_axis
        // survives, and rotation is the identity on a circle.
        //
        // Swept at 1x/1000x/0.001x because the recognizer's conic residual
        // is compared against a LINEAR tolerance.
        use remus_geometry::convert::curve_to_nurbs::ellipse_to_nurbs;

        for k in [1.0_f64, 1000.0, 0.001] {
            // Closed form, written out here rather than taken from the
            // kernel: the source frame is stated, not derived.
            let theta = 37.0_f64.to_radians();
            let normal = Vec3::new(0.0, 0.0, 1.0);
            let u_src = Vec3::new(theta.cos(), theta.sin(), 0.0);
            let v_src = Vec3::new(-theta.sin(), theta.cos(), 0.0);
            let center = Point3::new(0.4 * k, -0.7 * k, 1.1 * k);
            let (a, b) = (3.0 * k, 1.2 * k);

            let src = Ellipse3D::with_axes(center, normal, a, b, u_src, v_src).unwrap();
            let (t0, t1) = (0.3, 2.4);
            let nurbs = ellipse_to_nurbs(&src, t0, t1).unwrap();

            let mut topo = Topology::new();
            let v0 = topo.add_vertex(Vertex::new(src.evaluate(t0), 1e-7 * k));
            let v1 = topo.add_vertex(Vertex::new(src.evaluate(t1), 1e-7 * k));
            let eid = topo.add_edge(Edge::new(v0, v1, EdgeCurve::NurbsCurve(nurbs)));
            let solid_id = scaffold(&mut topo, eid);

            let mut tol = Tolerance::new();
            tol.linear *= k;
            let n = convert_edges_to_elementary(&mut topo, solid_id, &tol).unwrap();
            assert_eq!(n, 1, "expected one conversion at {k}x, got {n}");

            let EdgeCurve::Ellipse(got) = topo.edge(eid).unwrap().curve() else {
                panic!(
                    "expected Ellipse at {k}x, got {:?}",
                    topo.edge(eid).unwrap().curve()
                );
            };

            assert!(
                (got.semi_major() - a).abs() < 1e-6 * a && (got.semi_minor() - b).abs() < 1e-6 * b,
                "semi-axes {}/{} vs {a}/{b} at {k}x",
                got.semi_major(),
                got.semi_minor()
            );

            // The major axis must point along the source's, up to sign —
            // a 180° flip maps an ellipse onto itself, a 37° one does not.
            let align = got.u_axis().dot(u_src).abs();
            assert!(
                align > 1.0 - 1e-9,
                "recovered major axis {:?} is rotated off the source {u_src:?} by {:.3}° at {k}x",
                got.u_axis(),
                align.clamp(-1.0, 1.0).acos().to_degrees()
            );

            // Point-set check against a hand-derived closed form: every
            // point of the SOURCE ellipse must satisfy the recovered
            // ellipse's implicit equation (x/a)² + (y/b)² = 1 and lie in
            // its plane. This is what "the same ellipse set" would mean.
            for i in 0..24 {
                let t = f64::from(i) * std::f64::consts::TAU / 24.0;
                let p = center + u_src * (a * t.cos()) + v_src * (b * t.sin());
                let d = p - got.center();
                let x = d.dot(got.u_axis());
                let y = d.dot(got.v_axis());
                let z = d.dot(got.normal());
                assert!(
                    z.abs() < 1e-9 * k,
                    "source point is {z} out of the recovered plane at {k}x"
                );
                let resid = (x / got.semi_major()).hypot(y / got.semi_minor()) - 1.0;
                assert!(
                    resid.abs() < 1e-9,
                    "source point at t={t:.3} misses the recovered ellipse by \
                     implicit residual {resid:.3e} at {k}x — the converted curve \
                     is a different point set",
                );
            }
        }
    }

    // ── Unbounded conics ──────────────────────────────────────────────
    //
    // The reference values below are the parameters the source curve was
    // BUILT from, so the assertions compare a recovered curve against a
    // known-exact input rather than against another kernel routine.

    use crate::construct::convert_curve::{hyperbola_to_nurbs, parabola_to_nurbs};
    use remus_math::curves::{Hyperbola3D, Parabola3D};

    /// Wrap a single edge in the minimum face/shell/solid scaffold the
    /// converter's iterator needs.
    fn scaffold(topo: &mut Topology, edge_id: EdgeId) -> SolidId {
        let wire = Wire::new(vec![OrientedEdge::new(edge_id, true)], false).unwrap();
        let wid = topo.add_wire(wire);
        let face_id = topo.add_face(Face::new(
            wid,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let shell_id = topo.add_shell(Shell::new(vec![face_id]).unwrap());
        topo.add_solid(Solid::new(shell_id, vec![]))
    }

    #[test]
    fn converts_a_nurbs_parabola_back_to_an_analytic_parabola() {
        // Round trip: exact conic Bezier -> recognizer -> EdgeCurve::Parabola.
        // Checked at 1x, 1000x and 0.001x because the recognizer compares an
        // algebraic conic residual against a LINEAR tolerance, which is not
        // dimensionally consistent (see the PR notes).
        for k in [1.0_f64, 1000.0, 0.001] {
            let focal = 0.6 * k;
            let par = Parabola3D::with_axes(
                Point3::new(1.0 * k, -2.0 * k, 0.5 * k),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
                focal,
            )
            .unwrap();
            let (t0, t1) = (-1.5 * k, 2.0 * k);
            let nurbs = parabola_to_nurbs(&par, t0, t1).unwrap();

            let mut topo = Topology::new();
            let v0 = topo.add_vertex(Vertex::new(par.evaluate(t0), 1e-7 * k));
            let v1 = topo.add_vertex(Vertex::new(par.evaluate(t1), 1e-7 * k));
            let eid = topo.add_edge(Edge::new(v0, v1, EdgeCurve::NurbsCurve(nurbs)));
            let solid_id = scaffold(&mut topo, eid);

            // Recognition tolerance is expressed relative to the model, so
            // the same test is meaningful at every scale.
            let mut tol = Tolerance::new();
            tol.linear *= k;
            let n = convert_edges_to_elementary(&mut topo, solid_id, &tol).unwrap();
            assert_eq!(n, 1, "expected one conversion at {k}x, got {n}");

            match topo.edge(eid).unwrap().curve() {
                EdgeCurve::Parabola(got) => {
                    assert!(
                        (got.focal_length() - focal).abs() < 1e-6 * focal,
                        "focal length {} vs {focal} at {k}x",
                        got.focal_length()
                    );
                    // The PLANE must survive. `Parabola3D::new` would have
                    // discarded it; only `with_axes` keeps it.
                    assert!(
                        got.normal().cross(par.normal()).length() < 1e-6,
                        "plane normal {:?} vs {:?} at {k}x",
                        got.normal(),
                        par.normal()
                    );
                    // Strongest check: the recovered curve must pass through
                    // the edge's own vertices.
                    let s = topo.vertex(v0).unwrap().point();
                    let e = topo.vertex(v1).unwrap().point();
                    for p in [s, e] {
                        let d = (got.evaluate(got.project(p)) - p).length();
                        assert!(d < 1e-6 * k, "vertex off recovered parabola by {d} at {k}x");
                    }
                }
                other => panic!("expected Parabola at {k}x, got {other:?}"),
            }
        }
    }

    #[test]
    fn converts_a_nurbs_hyperbola_back_onto_the_correct_branch() {
        // The recognizer returns the real axis as an EIGENVECTOR, whose sign
        // is arbitrary (`0.5*atan2(B, A-C)` always lands in the half-plane
        // cos(theta) >= 0). `Hyperbola3D` models only the +u branch, so an
        // unflipped axis puts the edge on the MIRROR branch — geometry that
        // still looks like a hyperbola but no longer touches the vertices.
        //
        // This case is built with its real axis pointing at -x, so a missing
        // sign fix reflects it and the vertex check below fails.
        let (a, b) = (2.0, 1.5);
        let hyp = Hyperbola3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(-1.0, 0.0, 0.0),
            a,
            b,
        )
        .unwrap();
        let (t0, t1) = (-0.9, 1.1);
        let nurbs = hyperbola_to_nurbs(&hyp, t0, t1).unwrap();

        let mut topo = Topology::new();
        let v0 = topo.add_vertex(Vertex::new(hyp.evaluate(t0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(hyp.evaluate(t1), 1e-7));
        let eid = topo.add_edge(Edge::new(v0, v1, EdgeCurve::NurbsCurve(nurbs)));
        let solid_id = scaffold(&mut topo, eid);

        let tol = Tolerance::new();
        let n = convert_edges_to_elementary(&mut topo, solid_id, &tol).unwrap();
        assert_eq!(n, 1, "expected one conversion, got {n}");

        match topo.edge(eid).unwrap().curve() {
            EdgeCurve::Hyperbola(got) => {
                assert!(
                    (got.semi_major() - a).abs() < 1e-6 * a,
                    "semi_major {} vs {a}",
                    got.semi_major()
                );
                assert!(
                    (got.semi_minor() - b).abs() < 1e-6 * b,
                    "semi_minor {} vs {b}",
                    got.semi_minor()
                );
                // The branch test: the recovered real axis must point the
                // same way the source's did.
                assert!(
                    got.u_axis().dot(hyp.u_axis()) > 0.99,
                    "recovered real axis {:?} is on the mirror branch (source {:?})",
                    got.u_axis(),
                    hyp.u_axis()
                );
                // And the edge's vertices must lie on it.
                for vid in [v0, v1] {
                    let p = topo.vertex(vid).unwrap().point();
                    let d = (got.evaluate(got.project(p)) - p).length();
                    assert!(d < 1e-6, "vertex off recovered hyperbola by {d}");
                }
            }
            other => panic!("expected Hyperbola, got {other:?}"),
        }
    }

    #[test]
    fn already_analytic_conics_are_left_alone() {
        // A second pass must be a no-op: re-converting would churn the arena
        // and could drift the geometry.
        let par = Parabola3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
        )
        .unwrap();
        let mut topo = Topology::new();
        let v0 = topo.add_vertex(Vertex::new(par.evaluate(-1.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(par.evaluate(1.0), 1e-7));
        let eid = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Parabola(par)));
        let solid_id = scaffold(&mut topo, eid);

        let tol = Tolerance::new();
        assert_eq!(
            convert_edges_to_elementary(&mut topo, solid_id, &tol).unwrap(),
            0
        );
        assert!(matches!(
            topo.edge(eid).unwrap().curve(),
            EdgeCurve::Parabola(_)
        ));
    }
}
