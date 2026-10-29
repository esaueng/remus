//! The exact torus of a fillet along a circular edge between a plane and a
//! coaxial cylinder.
//!
//! When the circle lies in the plane and the cylinder shares its axis, the
//! rolling ball's centre stays one radius off the plane and a fixed distance
//! from the axis, so the blend is a torus about that axis. This is the arc an
//! earlier fillet leaves where its band meets an end face: a later fillet
//! whose tangent chain runs across that arc rounds it with this torus, whose
//! ring radius is the band's radius plus the new one beside a concave band
//! and minus it beside a convex one. When that ring radius is zero (a convex
//! band met at its own radius) the ball's centre does not move and the
//! stripe is a sphere triangle instead.

use remus_math::curves::Circle3D;
use remus_math::surfaces::ToroidalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;

/// A torus stripe and its four exact boundary arcs, each as
/// `(circle, start, end)` along the stripe's corner points.
pub struct ArcTorus {
    pub torus: ToroidalSurface,
    pub seams: [(Circle3D, Point3, Point3); 4],
}

/// The sphere patch a radius-`radius` fillet sweeps along `edge` when the
/// ball's centre stays put: the edge is the arc a convex band of the same
/// radius leaves on an end plane, so the torus [`arc_stripe_torus`] would
/// build has zero ring radius. Both plane contacts then sit at the arc's
/// centre and the stripe is the spherical triangle between that pole and the
/// two band contacts, bounded by three great circles: the band's contact
/// circle and the two end sections.
pub struct ArcSphere {
    pub centre: Point3,
    /// The plane contacts' shared point.
    pub pole: Point3,
    /// The band contacts at the stripe's start and end.
    pub band: [Point3; 2],
}

/// The plane/coaxial-cylinder stripe both exact arc carriers start from.
struct ArcStation {
    /// The arc's centre, on the plane.
    origin: Point3,
    axis: Vec3,
    /// The arc's (and its coaxial band's) radius.
    band_radius: f64,
    /// The plane contacts' distance from the arc's axis.
    ring: f64,
    /// The band contacts' circle centre, one radius off the plane.
    centre: Point3,
    plane: [Point3; 2],
    band: [Point3; 2],
    eps: f64,
}

/// Qualify `edge` as a circle in a plane beside a coaxial cylinder whose
/// stripe contacts sit where a rolling ball of `radius` puts them.
fn arc_station(
    edge: &EdgeCurve,
    surf1: &FaceSurface,
    surf2: &FaceSurface,
    corners: [Point3; 4],
    radius: f64,
    tol: Tolerance,
) -> Option<ArcStation> {
    let EdgeCurve::Circle(spine) = edge else {
        return None;
    };
    let [c1s, c2s, c2e, c1e] = corners;
    // Walker contacts land on their surfaces to the solver's tolerance, not
    // the modelling one.
    let eps = (radius * 1e-6).max(tol.linear * 10.0);
    let (o, axis, a) = (spine.center(), spine.normal(), spine.radius());

    // Only a plane beside a cylinder qualifies, in either face order.
    let ((ps, pe), (cs, ce), cylinder) =
        if let (FaceSurface::Plane { normal, d }, FaceSurface::Cylinder(cylinder)) = (surf1, surf2)
        {
            on_plane(*normal, *d, o, axis, eps)?;
            ((c1s, c1e), (c2s, c2e), cylinder)
        } else if let (FaceSurface::Cylinder(cylinder), FaceSurface::Plane { normal, d }) =
            (surf1, surf2)
        {
            on_plane(*normal, *d, o, axis, eps)?;
            ((c2s, c2e), (c1s, c1e), cylinder)
        } else {
            return None;
        };
    if cylinder.axis().cross(axis).length() > tol.angular.max(1e-9)
        || (cylinder.radius() - a).abs() > eps
        || (cylinder.origin() - o).cross(axis).length() > eps
    {
        return None;
    }

    // Plane contacts run on the ring circle, cylinder contacts on the band
    // circle one radius off the plane.
    let ring = (ps - o).length();
    let height = (cs - o).dot(axis);
    let radial = |p: Point3| {
        let offset = p - o;
        offset - axis * offset.dot(axis)
    };
    if [ps, pe].iter().any(|&p| (p - o).dot(axis).abs() > eps)
        || ((pe - o).length() - ring).abs() > eps
        || ((ce - o).dot(axis) - height).abs() > eps
        || [cs, ce]
            .iter()
            .any(|&p| (radial(p).length() - a).abs() > eps)
        || (height.abs() - radius).abs() > eps
        || ((ring - a).abs() - radius).abs() > eps
    {
        return None;
    }
    Some(ArcStation {
        origin: o,
        axis,
        band_radius: a,
        ring,
        centre: o + axis * height,
        plane: [ps, pe],
        band: [cs, ce],
        eps,
    })
}

/// The torus a radius-`radius` fillet sweeps along `edge`, or `None` when the
/// edge is not a circle in a plane beside a coaxial cylinder, the torus would
/// not be a ring torus, or the stripe's contacts do not lie on it.
///
/// `corners` are the stripe's contact points `[c1_start, c2_start, c2_end,
/// c1_end]`, the `1` contacts on `surf1`.
pub fn arc_stripe_torus(
    edge: &EdgeCurve,
    surf1: &FaceSurface,
    surf2: &FaceSurface,
    corners: [Point3; 4],
    radius: f64,
    tol: Tolerance,
) -> Option<ArcTorus> {
    let ArcStation {
        origin: o,
        axis,
        band_radius: a,
        ring,
        centre,
        plane: [ps, pe],
        band: [cs, ce],
        eps,
    } = arc_station(edge, surf1, surf2, corners, radius, tol)?;
    if ring <= radius + eps {
        return None;
    }
    let radial = |p: Point3, from: Point3| {
        let offset = p - from;
        offset - axis * offset.dot(axis)
    };
    let torus = ToroidalSurface::with_axis(centre, ring, radius, axis).ok()?;

    // Each end section is the quarter circle about that station's ball
    // centre, in the meridian plane through it.
    let section = |plane_contact: Point3, band_contact: Point3, plane_first: bool| {
        let u = radial(plane_contact, o).normalize().ok()?;
        let ball = centre + u * ring;
        if ((plane_contact - ball).length() - radius).abs() > eps
            || ((band_contact - ball).length() - radius).abs() > eps
        {
            return None;
        }
        let circle = Circle3D::new(ball, axis.cross(u), radius).ok()?;
        Some(if plane_first {
            (circle, plane_contact, band_contact)
        } else {
            (circle, band_contact, plane_contact)
        })
    };
    let plane_first = matches!(surf1, FaceSurface::Plane { .. });
    let start = section(ps, cs, plane_first)?;
    let end = section(pe, ce, plane_first)?;
    Some(ArcTorus {
        torus,
        seams: [
            (Circle3D::new(o, axis, ring).ok()?, ps, pe),
            (Circle3D::new(centre, axis, a).ok()?, cs, ce),
            start,
            end,
        ],
    })
}

/// The sphere patch of a radius-`radius` fillet along `edge`, or `None`
/// unless the stripe qualifies as for [`arc_stripe_torus`] with a ring radius
/// of zero: a convex band of radius `radius` met by an end plane.
///
/// `band_mid` is a band-side contact from an interior station; it pins the
/// contact arc to the short great-circle arc between the band contacts,
/// which is the one the sphere cap's boundary mints.
pub fn arc_stripe_sphere(
    edge: &EdgeCurve,
    surf1: &FaceSurface,
    surf2: &FaceSurface,
    corners: [Point3; 4],
    band_mid: Point3,
    radius: f64,
    tol: Tolerance,
) -> Option<ArcSphere> {
    let station = arc_station(edge, surf1, surf2, corners, radius, tol)?;
    let eps = station.eps;
    if station.ring > eps {
        return None;
    }
    let centre = station.centre;
    let [ps, pe] = station.plane;
    let [cs, ce] = station.band;
    if [ps, pe, cs, ce, band_mid]
        .iter()
        .any(|&p| ((p - centre).length() - radius).abs() > eps)
        || ((band_mid - centre).dot(station.axis)).abs() > eps
    {
        return None;
    }
    // The contact arc must be the short arc from `cs` to `ce` through
    // `band_mid`, well clear of a half turn.
    let (ds, de, dm) = (cs - centre, ce - centre, band_mid - centre);
    let normal = ds.cross(de);
    if normal.length() <= radius * radius * 1e-6
        || ds.cross(dm).dot(normal) <= 0.0
        || dm.cross(de).dot(normal) <= 0.0
    {
        return None;
    }
    Some(ArcSphere {
        centre,
        pole: ps,
        band: [cs, ce],
    })
}

fn on_plane(normal: Vec3, d: f64, o: Point3, axis: Vec3, eps: f64) -> Option<()> {
    let normal = normal.normalize().ok()?;
    let through_centre = (normal.dot(Vec3::new(o.x(), o.y(), o.z())) - d).abs() <= eps;
    (through_centre && normal.cross(axis).length() <= 1e-9).then_some(())
}

/// Exact runout from the floor ring to the inner tangency of a ring torus
/// and an axis-parallel plane. This is a rational quartic, not a fitted SSI.
///
/// In the section plane, with `d = R - r`, the implicit equation is
/// `(x² + z²)² = 4 R (r x² - d z²)`. Rationalizing its tangent-cone conic
/// gives the homogeneous polynomials below. The finite lower branch runs
/// from `s0` to -1; reflecting either coordinate gives the other branches.
/// A single degree-four rational Bézier represents the whole interval.
/// In homogeneous coordinates their exact identities are
/// `X² + Z² = 4 R r (1 - s²)² W` and
/// `r X² - d Z² = 4 R r² (1 - s²)⁴`, which imply the implicit equation
/// over the complete interval. `W >= 1` prevents poles. Positive Bézier
/// weights retain that property after restriction to the chosen branch.
///
/// Admitted only by the bounded tab constructor, which independently
/// qualifies adjoining support limits, complete history and the closed body;
/// a rollover operation.
fn inner_tangent_runout(
    torus: &ToroidalSurface,
    plane_direction: Vec3,
    opposite_radial: bool,
    upper: bool,
) -> Option<remus_math::nurbs::curve::NurbsCurve> {
    use remus_math::nurbs::curve::NurbsCurve;

    let (major, minor) = (torus.major_radius(), torus.minor_radius());
    let distance = major - minor;
    if !major.is_finite() || !minor.is_finite() || minor <= 0.0 || distance <= 0.0 {
        return None;
    }
    let axis = torus.z_axis();
    let front = plane_direction.normalize().ok()?;
    if front.dot(axis).abs() > 64.0 * f64::EPSILON {
        return None;
    }
    let origin = torus.center() + front * distance;
    let x_axis = axis.cross(front) * if opposite_radial { -1.0 } else { 1.0 };
    let z_axis = axis * if upper { -1.0 } else { 1.0 };
    // This quotient avoids subtracting two nearly equal square roots.
    let s0 = -distance.sqrt() / ((2.0 * major - minor).sqrt() + major.sqrt());
    let x_scale = 2.0 * (major * minor).sqrt();
    let z_scale = 4.0 * minor * (major / distance).sqrt();
    let to_bernstein = |power: [f64; 5]| {
        const BINOMIAL: [[f64; 5]; 5] = [
            [1.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0, 0.0, 0.0],
            [1.0, 2.0, 1.0, 0.0, 0.0],
            [1.0, 3.0, 3.0, 1.0, 0.0],
            [1.0, 4.0, 6.0, 4.0, 1.0],
        ];
        let mut restricted = [0.0; 5];
        for (k, coefficient) in power.iter().enumerate() {
            for (i, value) in restricted.iter_mut().enumerate().take(k + 1) {
                #[allow(clippy::cast_possible_wrap, clippy::cast_possible_truncation)]
                let (left, right) = ((k - i) as i32, i as i32);
                *value += coefficient * BINOMIAL[k][i] * s0.powi(left) * (-1.0 - s0).powi(right);
            }
        }
        std::array::from_fn::<_, 5, _>(|j| {
            (0..=j)
                .map(|i| restricted[i] * BINOMIAL[j][i] / BINOMIAL[4][i])
                .sum::<f64>()
        })
    };
    let x = to_bernstein([x_scale, 0.0, 0.0, 0.0, -x_scale]);
    let z = to_bernstein([0.0, z_scale, 0.0, -z_scale, 0.0]);
    let weights = to_bernstein([1.0, 0.0, 2.0 + 4.0 * minor / distance, 0.0, 1.0]);
    if weights
        .iter()
        .any(|&weight| !weight.is_finite() || weight <= 0.0)
    {
        return None;
    }
    let controls: Vec<_> = (0..5)
        .map(|i| origin + x_axis * (x[i] / weights[i]) + z_axis * (z[i] / weights[i]))
        .collect();
    if controls
        .iter()
        .any(|point| !point.x().is_finite() || !point.y().is_finite() || !point.z().is_finite())
    {
        return None;
    }
    NurbsCurve::new(
        4,
        vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        controls,
        weights.to_vec(),
    )
    .ok()
}

#[cfg(test)]
mod inner_tangent_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use remus_math::nurbs::curve::NurbsCurve;

    /// Recover power coefficients from the actual stored homogeneous
    /// control net. The oracle never calls the constructor's conversion.
    fn power(bernstein: [f64; 5]) -> Vec<f64> {
        let mut differences = bernstein;
        let mut result = vec![bernstein[0]];
        for multiplier in [4.0, 6.0, 4.0, 1.0] {
            for i in 0..4 {
                differences[i] = differences[i + 1] - differences[i];
            }
            result.push(multiplier * differences[0]);
        }
        result
    }

    fn product(a: &[f64], b: &[f64]) -> Vec<f64> {
        let mut result = vec![0.0; a.len() + b.len() - 1];
        for (i, &x) in a.iter().enumerate() {
            for (j, &y) in b.iter().enumerate() {
                result[i + j] += x * y;
            }
        }
        result
    }

    /// The entire degree-16 homogeneous implicit polynomial must vanish,
    /// including coefficients between every pair of evaluation samples.
    fn implicit_residual(curve: &NurbsCurve, torus: &ToroidalSurface, front: Vec3) -> f64 {
        let major = torus.major_radius();
        let minor = torus.minor_radius() / major;
        let distance = 1.0 - minor;
        let origin = torus.center() + front * (major * distance);
        let x_axis = torus.z_axis().cross(front);
        let coord = |axis: Vec3| {
            power(std::array::from_fn(|i| {
                (curve.control_points()[i] - origin).dot(axis) * curve.weights()[i] / major
            }))
        };
        let x = coord(x_axis);
        let z = coord(torus.z_axis());
        let w = power(std::array::from_fn(|i| curve.weights()[i]));
        let (xx, zz, ww) = (product(&x, &x), product(&z, &z), product(&w, &w));
        let sum: Vec<_> = xx.iter().zip(&zz).map(|(a, b)| a + b).collect();
        let rhs: Vec<_> = xx
            .iter()
            .zip(&zz)
            .map(|(a, b)| 4.0 * (minor * a - distance * b))
            .collect();
        product(&sum, &sum)
            .iter()
            .zip(product(&rhs, &ww))
            .map(|(left, right)| (left - right).abs() / curve.max_weight().powi(4))
            .fold(0.0_f64, f64::max)
    }

    #[test]
    fn inner_tangent_quartic_has_whole_span_identity_and_exact_contacts() {
        for scale in [0.01, 1.0, 100.0] {
            for (major, minor) in [(5.0, 3.0), (3.0, 1.0), (10.0, 8.0)] {
                for axis in [
                    Vec3::new(0.0, 0.0, 1.0),
                    Vec3::new(2.0, -3.0, 7.0).normalize().unwrap(),
                ] {
                    let torus = ToroidalSurface::with_axis(
                        Point3::new(37.0, -91.0, 13.0),
                        major * scale,
                        minor * scale,
                        axis,
                    )
                    .unwrap();
                    let front = torus.x_axis();
                    for opposite in [false, true] {
                        for upper in [false, true] {
                            let curve =
                                inner_tangent_runout(&torus, front, opposite, upper).unwrap();
                            assert_eq!(curve.degree(), 4);
                            assert_eq!(curve.control_points().len(), 5);
                            assert!(curve.weights().iter().all(|&w| w > 0.0 && w.is_finite()));
                            assert!(implicit_residual(&curve, &torus, front) < 2e-11);
                            let radial = axis.cross(front) * if opposite { -1.0 } else { 1.0 };
                            let axial = axis * if upper { -1.0 } else { 1.0 };
                            let distance = (major - minor) * scale;
                            let origin = torus.center() + front * distance;
                            let start = origin
                                + radial
                                    * ((major * major - (major - minor).powi(2)).sqrt() * scale)
                                - axial * (minor * scale);
                            assert!(
                                (curve.evaluate(0.0) - start).length() < 2e-12 * scale.max(1.0)
                            );
                            assert!(
                                (curve.evaluate(1.0) - origin).length() < 2e-12 * scale.max(1.0)
                            );
                            assert!(curve.tangent(0.0).unwrap().dot(-radial) > 1.0 - 1e-11);
                            let endpoint_tangent = (-radial
                                + axial * (minor / (major - minor)).sqrt())
                            .normalize()
                            .unwrap();
                            assert!(
                                curve.tangent(1.0).unwrap().dot(endpoint_tangent) > 1.0 - 1e-11
                            );
                            // Tube-distance checks independently select the intended
                            // real branch of the squared implicit equation.
                            for i in 0..=32 {
                                let p = curve.evaluate(f64::from(i) / 32.0);
                                let v = p - torus.center();
                                let z = v.dot(axis);
                                let rho = (v - axis * z).length();
                                assert!(
                                    ((rho - major * scale).hypot(z) - minor * scale).abs()
                                        < 2e-12 * scale.max(1.0)
                                );
                                assert!((p - origin).dot(front).abs() < 2e-12 * scale.max(1.0));
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn inner_tangent_identity_rejects_a_wrong_interior_control() {
        let torus = ToroidalSurface::with_axis(
            Point3::new(0.0, 0.0, 0.0),
            5.0,
            3.0,
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();
        let front = torus.x_axis();
        let curve = inner_tangent_runout(&torus, front, false, false).unwrap();
        let mut controls = curve.control_points().to_vec();
        controls[2] = controls[2] + torus.y_axis() * 1e-3;
        let wrong = NurbsCurve::new(
            4,
            curve.knots().to_vec(),
            controls,
            curve.weights().to_vec(),
        )
        .unwrap();
        assert!(implicit_residual(&wrong, &torus, front) > 1e-7);
        assert!(inner_tangent_runout(&torus, torus.z_axis(), false, false).is_none());
        assert!(
            inner_tangent_runout(&torus, Vec3::new(f64::NAN, 0.0, 0.0), false, false).is_none()
        );
    }

    #[test]
    fn inner_tangent_quartic_refuses_outside_its_ring_plane_family() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let z = Vec3::new(0.0, 0.0, 1.0);
        for (major, minor) in [(3.0, 3.0), (2.0, 3.0), (f64::INFINITY, 1.0)] {
            let torus = ToroidalSurface::with_axis(origin, major, minor, z).unwrap();
            assert!(inner_tangent_runout(&torus, Vec3::new(1.0, 0.0, 0.0), false, false).is_none());
        }
        let torus = ToroidalSurface::with_axis(origin, 5.0, 3.0, z).unwrap();
        for normal in [Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 1e-10)] {
            assert!(inner_tangent_runout(&torus, normal, false, false).is_none());
        }
    }
}

/// Recognition stays test-gated with the curve until the sealed constructor
/// and original-region history are complete. No general blend refusal is
/// relaxed by these read-only queries.
mod bounded_tab_supports {
    use super::{Circle3D, FaceSurface, Point3, Tolerance, ToroidalSurface, Vec3};
    use remus_math::surfaces::CylindricalSurface;
    use remus_topology::Topology;
    use remus_topology::edge::{EdgeCurve, EdgeId};
    use remus_topology::face::FaceId;
    use remus_topology::solid::SolidId;
    use remus_topology::vertex::VertexId;

    struct Support {
        id: FaceId,
        surface: FaceSurface,
        reversed: bool,
        vertices: Vec<VertexId>,
        points: Vec<Point3>,
    }

    impl Support {
        fn normal(&self) -> Option<Vec3> {
            let FaceSurface::Plane { normal, .. } = &self.surface else {
                return None;
            };
            (*normal * if self.reversed { -1.0 } else { 1.0 })
                .normalize()
                .ok()
        }

        fn contains(&self, point: Point3, eps: f64) -> bool {
            self.points.iter().any(|&p| (p - point).length() <= eps)
        }
    }

    #[derive(Clone, Copy)]
    struct Frame {
        origin: Point3,
        along: Vec3,
        back: Vec3,
        up: Vec3,
    }

    impl Frame {
        fn point(self, x: f64, y: f64, z: f64) -> Point3 {
            self.origin + self.along * x + self.back * y + self.up * z
        }

        fn local(self, point: Point3) -> Vec3 {
            let offset = point - self.origin;
            Vec3::new(
                offset.dot(self.along),
                offset.dot(self.back),
                offset.dot(self.up),
            )
        }
    }

    #[derive(Clone, Copy)]
    struct Foot {
        frame: Frame,
        old_radius: f64,
        length: f64,
        side_length: f64,
        height: f64,
        floor: FaceId,
        back_wall: FaceId,
        side_wall: FaceId,
        front_cap: FaceId,
        outer_cap: FaceId,
        back_cylinder: FaceId,
        front_cylinder: FaceId,
    }

    fn supports(topo: &Topology, solid: SolidId) -> Option<Vec<Support>> {
        let source = topo.solid(solid).ok()?;
        if !source.inner_shells().is_empty() {
            return None;
        }
        let mut result = Vec::new();
        for id in remus_topology::explorer::solid_faces(topo, solid).ok()? {
            let face = topo.face(id).ok()?;
            if !face.inner_wires().is_empty()
                || !matches!(
                    face.surface(),
                    FaceSurface::Plane { .. } | FaceSurface::Cylinder(_)
                )
            {
                return None;
            }
            let mut vertices = Vec::new();
            for oriented in topo.wire(face.outer_wire()).ok()?.edges() {
                let edge = topo.edge(oriented.edge()).ok()?;
                if !matches!(edge.curve(), EdgeCurve::Line | EdgeCurve::Circle(_)) {
                    return None;
                }
                vertices.push(if oriented.is_forward() {
                    edge.start()
                } else {
                    edge.end()
                });
            }
            let points: Option<Vec<_>> = vertices
                .iter()
                .map(|&v| {
                    topo.vertex(v)
                        .ok()
                        .map(remus_topology::vertex::Vertex::point)
                })
                .collect();
            result.push(Support {
                id,
                surface: face.surface().clone(),
                reversed: face.is_reversed(),
                vertices,
                points: points?,
            });
        }
        Some(result)
    }

    fn projected_axis_origin(cylinder: &CylindricalSurface, frame: Frame) -> Point3 {
        let height = (cylinder.origin() - frame.origin).dot(frame.up);
        cylinder.origin() - frame.up * height
    }

    #[allow(clippy::too_many_arguments)]
    fn exact_quarter_wall(
        topo: &Topology,
        face: &Support,
        frame: Frame,
        radius: f64,
        centre_y: f64,
        length: f64,
        height: f64,
        back_quarter: bool,
        tol: Tolerance,
    ) -> bool {
        let FaceSurface::Cylinder(cylinder) = &face.surface else {
            return false;
        };
        if face.reversed
            || face.vertices.len() != 4
            || cylinder.axis().cross(frame.up).length() > tol.angular
            || (cylinder.radius() - radius).abs() > tol.linear
        {
            return false;
        }
        let centre = projected_axis_origin(cylinder, frame);
        if (centre - frame.point(length, centre_y, 0.0)).length() > tol.linear {
            return false;
        }
        let sign = if back_quarter { 1.0 } else { -1.0 };
        let endpoints = [
            (length, centre_y + sign * radius),
            (length + radius, centre_y),
        ];
        for (x, y) in endpoints {
            if !face.contains(frame.point(x, y, 0.0), tol.linear)
                || !face.contains(frame.point(x, y, height), tol.linear)
            {
                return false;
            }
        }
        let Ok(data) = topo.face(face.id) else {
            return false;
        };
        let Ok(wire) = topo.wire(data.outer_wire()) else {
            return false;
        };
        let mut circles = 0;
        for oriented in wire.edges() {
            let Ok(edge) = topo.edge(oriented.edge()) else {
                return false;
            };
            match edge.curve() {
                EdgeCurve::Line => {}
                EdgeCurve::Circle(circle) => {
                    let Ok((start, end)) = edge.strict_domain() else {
                        return false;
                    };
                    let (Ok(v0), Ok(v1)) = (topo.vertex(edge.start()), topo.vertex(edge.end()))
                    else {
                        return false;
                    };
                    if separation(circle.evaluate(start), v0.point()) > tol.linear
                        || separation(circle.evaluate(end), v1.point()) > tol.linear
                    {
                        return false;
                    }
                    let axial = (circle.center() - centre).dot(frame.up);
                    let expected = centre + frame.up * axial;
                    if (circle.center() - expected).length() > tol.linear
                        || (circle.radius() - radius).abs() > tol.linear
                        || circle.normal().cross(frame.up).length() > tol.angular
                        || ((end - start).abs() - std::f64::consts::FRAC_PI_2).abs() > tol.angular
                        || axial.abs().min((axial - height).abs()) > tol.linear
                    {
                        return false;
                    }
                    circles += 1;
                }
                EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => return false,
            }
        }
        circles == 2
    }

    fn find_plane(
        faces: &[Support],
        normal: Vec3,
        corners: &[Point3],
        rectangular: bool,
        tol: Tolerance,
    ) -> Option<FaceId> {
        let matches: Vec<_> = faces
            .iter()
            .filter(|face| {
                (!rectangular || face.points.len() == 4)
                    && face
                        .normal()
                        .is_some_and(|n| (n - normal).length() <= tol.angular)
                    && corners.iter().all(|&p| face.contains(p, tol.linear))
            })
            .collect();
        (matches.len() == 1).then(|| matches[0].id)
    }

    /// Read-only structural support recognizer. Reach clearance, source-region
    /// provenance and transactional reconstruction are deliberately separate
    /// gates and must all precede any public dispatch.
    fn recognize_impl(
        topo: &Topology,
        solid: SolidId,
        seed: EdgeId,
        radius: f64,
        tol: Tolerance,
        require_full_floor: bool,
    ) -> Option<Foot> {
        if !radius.is_finite() || radius <= 0.0 {
            return None;
        }
        let faces = supports(topo, solid)?;
        let edge = topo.edge(seed).ok()?;
        if !matches!(edge.curve(), EdgeCurve::Line) {
            return None;
        }
        let endpoints = [
            topo.vertex(edge.start()).ok()?.point(),
            topo.vertex(edge.end()).ok()?.point(),
        ];
        let adjacency = topo.build_adjacency(solid).ok()?;
        let adjacent = adjacency.faces_for_edge(seed);
        if adjacent.len() != 2 {
            return None;
        }
        for floor_index in 0..2 {
            let floor = faces.iter().find(|face| face.id == adjacent[floor_index])?;
            let back_wall = faces
                .iter()
                .find(|face| face.id == adjacent[1 - floor_index])?;
            let (Some(up), Some(back)) = (floor.normal(), back_wall.normal()) else {
                continue;
            };
            if up.dot(back).abs() > tol.angular {
                continue;
            }
            for inner_index in 0..2 {
                let (inner, outer) = (endpoints[inner_index], endpoints[1 - inner_index]);
                let length = (inner - outer).length();
                let Ok(along) = (inner - outer).normalize() else {
                    continue;
                };
                if along.dot(up).abs() > tol.angular || along.dot(back).abs() > tol.angular {
                    continue;
                }
                let frame = Frame {
                    origin: outer,
                    along,
                    back,
                    up,
                };
                for back_cylinder in &faces {
                    let FaceSurface::Cylinder(cylinder) = &back_cylinder.surface else {
                        continue;
                    };
                    let old_radius = cylinder.radius();
                    if !old_radius.is_finite() || old_radius <= 0.0 {
                        continue;
                    }
                    let centre = projected_axis_origin(cylinder, frame);
                    if (centre - frame.point(length, -old_radius, 0.0)).length() > tol.linear {
                        continue;
                    }
                    let height = back_cylinder
                        .points
                        .iter()
                        .map(|&p| frame.local(p).z())
                        .fold(0.0_f64, f64::max);
                    if radius >= height - tol.linear
                        || !exact_quarter_wall(
                            topo,
                            back_cylinder,
                            frame,
                            old_radius,
                            -old_radius,
                            length,
                            height,
                            true,
                            tol,
                        )
                    {
                        continue;
                    }
                    for front_cylinder in &faces {
                        let FaceSurface::Cylinder(front) = &front_cylinder.surface else {
                            continue;
                        };
                        let front_origin = frame.local(projected_axis_origin(front, frame));
                        let side_length = -old_radius - front_origin.y();
                        if side_length <= tol.linear
                            || !exact_quarter_wall(
                                topo,
                                front_cylinder,
                                frame,
                                old_radius,
                                front_origin.y(),
                                length,
                                height,
                                false,
                                tol,
                            )
                        {
                            continue;
                        }
                        let p = frame.point(length + old_radius, -old_radius, 0.0);
                        let q = frame.point(length + old_radius, -old_radius - side_length, 0.0);
                        let c = frame.point(length, -2.0 * old_radius - side_length, 0.0);
                        let Some(side_wall) = find_plane(
                            &faces,
                            along,
                            &[p, q, p + up * height, q + up * height],
                            true,
                            tol,
                        ) else {
                            continue;
                        };
                        let Some(back_id) = find_plane(
                            &faces,
                            back,
                            &[outer, inner, outer + up * height, inner + up * height],
                            true,
                            tol,
                        ) else {
                            continue;
                        };
                        if back_id != back_wall.id
                            || ![p, q].iter().all(|&p| floor.contains(p, tol.linear))
                            || (require_full_floor && !floor.contains(c, tol.linear))
                        {
                            continue;
                        }
                        let Some(front_cap) =
                            find_plane(&faces, -back, &[c, c + up * height], false, tol)
                        else {
                            continue;
                        };
                        let Some(outer_cap) =
                            find_plane(&faces, -along, &[outer, outer + up * height], false, tol)
                        else {
                            continue;
                        };
                        return Some(Foot {
                            frame,
                            old_radius,
                            length,
                            side_length,
                            height,
                            floor: floor.id,
                            back_wall: back_id,
                            side_wall,
                            front_cap,
                            outer_cap,
                            back_cylinder: back_cylinder.id,
                            front_cylinder: front_cylinder.id,
                        });
                    }
                }
            }
        }
        None
    }

    fn recognize(
        topo: &Topology,
        solid: SolidId,
        seed: EdgeId,
        radius: f64,
        tol: Tolerance,
    ) -> Option<Foot> {
        recognize_impl(topo, solid, seed, radius, tol, true)
    }

    // Draft for insertion into the existing test-gated bounded_tab_supports module.
    // Not a public operation: reach/source-region/history gates must precede dispatch.
    use remus_math::det_hash::DetHashMap;
    use remus_topology::edge::Edge;
    use remus_topology::face::Face;
    use remus_topology::shell::Shell;
    use remus_topology::solid::Solid;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    const FO: usize = 0;
    const FB: usize = 1;
    const FP: usize = 2;
    const FQ: usize = 3;
    const FD: usize = 4;
    const WO: usize = 5;
    const WB: usize = 6;
    const WP: usize = 7;
    const WQ: usize = 8;
    const WC: usize = 9;

    struct Constructed {
        solid: SolidId,
        // Canonical support face -> its final replacement. Original source-region
        // coverage must be composed separately, before exposing BlendFaceOrigins.
        modified: DetHashMap<FaceId, FaceId>,
        bands: Vec<(FaceId, Vec<FaceId>)>,
    }

    fn pair(a: VertexId, b: VertexId) -> (usize, usize) {
        (a.index().min(b.index()), a.index().max(b.index()))
    }

    fn source_vertex(face: &Support, point: Point3, tol: Tolerance) -> Option<VertexId> {
        let matches: Vec<_> = face
            .vertices
            .iter()
            .zip(&face.points)
            .filter_map(|(&id, &p)| ((p - point).length() <= tol.linear).then_some(id))
            .collect();
        (matches.len() == 1).then(|| matches[0])
    }

    fn replace_ids(vertices: &mut [VertexId], replacements: &[(VertexId, VertexId)]) {
        for vertex in vertices {
            if let Some((_, new)) = replacements.iter().find(|(old, _)| *old == *vertex) {
                *vertex = *new;
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn native_circle(
        topo: &mut Topology,
        edges: &mut DetHashMap<(usize, usize), EdgeId>,
        a: VertexId,
        b: VertexId,
        centre: Point3,
        normal: Vec3,
        radius: f64,
        tol: Tolerance,
    ) -> Option<()> {
        let circle = Circle3D::new(centre, normal, radius).ok()?;
        let (start, end) = (topo.vertex(a).ok()?.point(), topo.vertex(b).ok()?.point());
        let t0 = circle.project(start);
        let mut delta = circle.project(end) - t0;
        if delta > std::f64::consts::PI {
            delta -= std::f64::consts::TAU;
        }
        if delta < -std::f64::consts::PI {
            delta += std::f64::consts::TAU;
        }
        if (circle.evaluate(t0) - start).length() > tol.linear
            || (circle.evaluate(t0 + delta) - end).length() > tol.linear
            || delta.abs() <= tol.angular
        {
            return None;
        }
        let mut edge = Edge::with_tolerance(a, b, EdgeCurve::Circle(circle), Some(tol.linear));
        edge.set_trim(Some((t0, t0 + delta)));
        let id = topo.add_edge(edge);
        if edges.insert(pair(a, b), id).is_some() {
            return None;
        }
        Some(())
    }

    fn mint_face(
        topo: &mut Topology,
        edges: &mut DetHashMap<(usize, usize), EdgeId>,
        vertices: &[VertexId],
        surface: FaceSurface,
        reversed: bool,
        tol: Tolerance,
    ) -> Option<FaceId> {
        let mut wire = Vec::with_capacity(vertices.len());
        for i in 0..vertices.len() {
            let (a, b) = (vertices[i], vertices[(i + 1) % vertices.len()]);
            if a == b {
                return None;
            }
            let id = if let Some(&id) = edges.get(&pair(a, b)) {
                id
            } else {
                // The sealed caller registers every curved boundary in advance.
                // In particular no torus boundary may silently become a chord.
                if matches!(surface, FaceSurface::Torus(_)) {
                    return None;
                }
                let id = topo.add_edge(Edge::with_tolerance(
                    a,
                    b,
                    EdgeCurve::Line,
                    Some(tol.linear),
                ));
                edges.insert(pair(a, b), id);
                id
            };
            wire.push(OrientedEdge::new(id, topo.edge(id).ok()?.start() == a));
        }
        let wire = topo.add_wire(Wire::new(wire, true).ok()?);
        Some(topo.add_face(if reversed {
            Face::new_reversed(wire, Vec::new(), surface)
        } else {
            Face::new(wire, Vec::new(), surface)
        }))
    }

    #[allow(clippy::too_many_arguments)]
    fn alter_floor(
        topo: &Topology,
        vertices: &mut Vec<VertexId>,
        old: [VertexId; 5],
        new: [VertexId; 5],
        frame: Frame,
        length: f64,
        depth: f64,
        radius: f64,
        tol: Tolerance,
    ) -> Option<()> {
        let threshold = frame.local(topo.vertex(new[FD]).ok()?.point()).x();
        for reverse in [false, true] {
            let mut trial = vertices.clone();
            if reverse {
                trial.reverse();
            }
            let Some(at) = trial.iter().position(|&v| v == old[0]) else {
                continue;
            };
            trial.rotate_left(at);
            if trial.len() < 5 || trial[..5] != old {
                continue;
            }
            let mut suffix = trial[5..].to_vec();
            while let Some(&id) = suffix.first() {
                let p = frame.local(topo.vertex(id).ok()?.point());
                if p.z().abs() <= tol.linear
                    && (p.y() + depth).abs() <= tol.linear
                    && p.x() >= length - tol.linear
                    && p.x() < threshold - tol.linear
                {
                    suffix.remove(0);
                } else {
                    break;
                }
            }
            let next = frame.local(topo.vertex(*suffix.first()?).ok()?.point());
            let previous = frame.local(topo.vertex(*suffix.last()?).ok()?.point());
            if (next.y() + depth).abs() > tol.linear
                || next.z().abs() > tol.linear
                || next.x() <= threshold + tol.linear
                || previous.x().abs() > tol.linear
                || previous.z().abs() > tol.linear
                || previous.y() <= radius + tol.linear
            {
                return None;
            }
            let mut result = new.to_vec();
            result.extend(suffix);
            if reverse {
                result.reverse();
            }
            *vertices = result;
            return Some(());
        }
        None
    }

    // Rotate a cap so the reflex source corner comes first and is followed by
    // its upward wall segment. The opposite preceding side is the floor edge.
    #[allow(clippy::too_many_arguments)]
    fn alter_cap(
        topo: &Topology,
        vertices: &mut Vec<VertexId>,
        old: VertexId,
        wall: VertexId,
        floor: VertexId,
        frame: Frame,
        front: bool,
        depth: f64,
        tol: Tolerance,
    ) -> Option<()> {
        let floor_point = frame.local(topo.vertex(floor).ok()?.point());
        let wall_point = frame.local(topo.vertex(wall).ok()?.point());
        for reverse in [false, true] {
            let mut trial = vertices.clone();
            if reverse {
                trial.reverse();
            }
            let Some(at) = trial.iter().position(|&v| v == old) else {
                continue;
            };
            trial.rotate_left(at);
            let next = frame.local(topo.vertex(*trial.get(1)?).ok()?.point());
            if next.z() <= wall_point.z() + tol.linear {
                continue;
            }
            if front {
                if (next.y() + depth).abs() > tol.linear
                    || (next.x() - wall_point.x()).abs() > tol.linear
                {
                    continue;
                }
                while trial.len() > 2 {
                    let p = frame.local(topo.vertex(*trial.last()?).ok()?.point());
                    if p.z().abs() <= tol.linear
                        && (p.y() + depth).abs() <= tol.linear
                        && p.x() < floor_point.x() - tol.linear
                    {
                        trial.pop();
                    } else {
                        break;
                    }
                }
                let last = frame.local(topo.vertex(*trial.last()?).ok()?.point());
                if last.z().abs() > tol.linear
                    || (last.y() + depth).abs() > tol.linear
                    || last.x() <= floor_point.x() + tol.linear
                {
                    return None;
                }
            } else {
                if next.x().abs() > tol.linear || next.y().abs() > tol.linear {
                    continue;
                }
                let last = frame.local(topo.vertex(*trial.last()?).ok()?.point());
                if last.x().abs() > tol.linear
                    || last.z().abs() > tol.linear
                    || last.y() <= floor_point.y() + tol.linear
                {
                    return None;
                }
            }
            trial[0] = wall;
            trial.push(floor);
            if reverse {
                trial.reverse();
            }
            *vertices = trial;
            return Some(());
        }
        None
    }

    #[derive(Clone, Copy)]
    struct Bounds {
        lo: [f64; 3],
        hi: [f64; 3],
    }
    impl Bounds {
        fn empty() -> Self {
            Self {
                lo: [f64::INFINITY; 3],
                hi: [f64::NEG_INFINITY; 3],
            }
        }
        fn include(&mut self, p: Vec3) -> Option<()> {
            for i in 0..3 {
                if !p.0[i].is_finite() {
                    return None;
                }
                self.lo[i] = self.lo[i].min(p.0[i]);
                self.hi[i] = self.hi[i].max(p.0[i]);
            }
            Some(())
        }
        fn intersects(self, other: Self, axes: &[usize], eps: f64) -> bool {
            axes.iter()
                .all(|&i| self.lo[i] < other.hi[i] - eps && self.hi[i] > other.lo[i] + eps)
        }
    }
    fn circle_bounds(circle: &Circle3D, frame: Frame) -> Option<Bounds> {
        let c = frame.local(circle.center());
        let mut b = Bounds::empty();
        let mut lo = c;
        let mut hi = c;
        for (i, axis) in [frame.along, frame.back, frame.up].into_iter().enumerate() {
            let extent =
                circle.radius() * circle.u_axis().dot(axis).hypot(circle.v_axis().dot(axis));
            lo.0[i] -= extent;
            hi.0[i] += extent;
        }
        b.include(lo)?;
        b.include(hi)?;
        Some(b)
    }
    fn edge_bounds(topo: &Topology, id: EdgeId, frame: Frame) -> Option<Bounds> {
        let edge = topo.edge(id).ok()?;
        match edge.curve() {
            EdgeCurve::Line => {
                let mut b = Bounds::empty();
                b.include(frame.local(topo.vertex(edge.start()).ok()?.point()))?;
                b.include(frame.local(topo.vertex(edge.end()).ok()?.point()))?;
                Some(b)
            }
            EdgeCurve::Circle(circle) => circle_bounds(circle, frame),
            EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => None,
        }
    }
    fn face_bounds(topo: &Topology, face: &Support, frame: Frame) -> Option<Bounds> {
        let mut result = Bounds::empty();
        for &p in &face.points {
            result.include(frame.local(p))?;
        }
        let wire = topo.wire(topo.face(face.id).ok()?.outer_wire()).ok()?;
        match &face.surface {
            FaceSurface::Plane { .. } => {
                for oriented in wire.edges() {
                    let b = edge_bounds(topo, oriented.edge(), frame)?;
                    result.include(Vec3::new(b.lo[0], b.lo[1], b.lo[2]))?;
                    result.include(Vec3::new(b.hi[0], b.hi[1], b.hi[2]))?;
                }
                Some(result)
            }
            FaceSurface::Cylinder(cylinder) => {
                let axis = cylinder.axis();
                let centre = cylinder.origin();
                let mut min = f64::INFINITY;
                let mut max = f64::NEG_INFINITY;
                for &p in &face.points {
                    let v = (p - centre).dot(axis);
                    min = min.min(v);
                    max = max.max(v);
                }
                for oriented in wire.edges() {
                    if let EdgeCurve::Circle(circle) = topo.edge(oriented.edge()).ok()?.curve() {
                        let v = (circle.center() - centre).dot(axis);
                        let spread = circle.radius()
                            * circle.u_axis().dot(axis).hypot(circle.v_axis().dot(axis));
                        min = min.min(v - spread);
                        max = max.max(v + spread);
                    }
                }
                let c = frame.local(centre);
                let mut lo = c;
                let mut hi = c;
                for (i, basis) in [frame.along, frame.back, frame.up].into_iter().enumerate() {
                    let slope = axis.dot(basis);
                    let spread = cylinder.radius()
                        * cylinder
                            .x_axis()
                            .dot(basis)
                            .hypot(cylinder.y_axis().dot(basis));
                    lo.0[i] += (slope * min).min(slope * max) - spread;
                    hi.0[i] += (slope * min).max(slope * max) + spread;
                }
                result.include(lo)?;
                result.include(hi)?;
                Some(result)
            }
            FaceSurface::Nurbs(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => None,
        }
    }
    fn foot_bounds(foot: &Foot, radius: f64) -> Bounds {
        Bounds {
            lo: [0.0, -2.0 * foot.old_radius - foot.side_length, 0.0],
            hi: [foot.length + foot.old_radius + radius, radius, radius],
        }
    }
    fn cap_reach_clear(
        topo: &Topology,
        face: &Support,
        foot: &Foot,
        radius: f64,
        front: bool,
        tol: Tolerance,
    ) -> Option<()> {
        let f = foot.frame;
        let reach = foot_bounds(foot, radius);
        let axes = if front { [0, 2] } else { [1, 2] };
        let wire = topo.wire(topo.face(face.id).ok()?.outer_wire()).ok()?;
        for oriented in wire.edges() {
            let edge = topo.edge(oriented.edge()).ok()?;
            let bounds = edge_bounds(topo, oriented.edge(), f)?;
            let a = f.local(topo.vertex(edge.start()).ok()?.point());
            let b = f.local(topo.vertex(edge.end()).ok()?.point());
            if matches!(edge.curve(), EdgeCurve::Line) {
                let floor_clip = a.z().abs() <= tol.linear
                    && b.z().abs() <= tol.linear
                    && if front {
                        a.x().min(b.x()) >= foot.length - tol.linear
                    } else {
                        a.y().min(b.y()) >= -tol.linear
                    };
                let side_clip = if front {
                    a.x().abs() <= tol.linear && b.x().abs() <= tol.linear
                } else {
                    (a.y() - reach.lo[1]).abs() <= tol.linear
                        && (b.y() - reach.lo[1]).abs() <= tol.linear
                };
                if floor_clip || side_clip {
                    continue;
                }
            }
            if !bounds.intersects(reach, &axes, -tol.linear) {
                continue;
            }
            if !matches!(edge.curve(), EdgeCurve::Line) {
                return None;
            }
            if front {
                if (a.x() - foot.length).abs() > tol.linear
                    || (b.x() - foot.length).abs() > tol.linear
                    || (a.y() - reach.lo[1]).abs() > tol.linear
                    || (b.y() - reach.lo[1]).abs() > tol.linear
                    || a.z().min(b.z()) < -tol.linear
                {
                    return None;
                }
            } else if a.x().abs() > tol.linear
                || b.x().abs() > tol.linear
                || a.y().abs() > tol.linear
                || b.y().abs() > tol.linear
                || a.z().min(b.z()) < -tol.linear
            {
                return None;
            }
        }
        Some(())
    }
    fn reach_is_clear(
        topo: &Topology,
        source: SolidId,
        feet: &[Foot],
        radius: f64,
        tol: Tolerance,
    ) -> Option<()> {
        if feet.is_empty() || feet.len() > 2 {
            return None;
        }
        let views = supports(topo, source)?;
        for (ordinal, foot) in feet.iter().enumerate() {
            if !radius.is_finite() || radius <= 0.0 || radius >= foot.height - tol.linear {
                return None;
            }
            let allowed = [
                foot.floor,
                foot.back_wall,
                foot.side_wall,
                foot.front_cap,
                foot.outer_cap,
                foot.back_cylinder,
                foot.front_cylinder,
            ];
            let frame = foot.frame;
            let reach = foot_bounds(foot, radius);
            let floor = views.iter().find(|view| view.id == foot.floor)?;
            let a = foot.old_radius;
            let l = foot.length;
            let h = foot.side_length;
            let expected = [
                frame.origin,
                frame.point(l, 0.0, 0.0),
                frame.point(l + a, -a, 0.0),
                frame.point(l + a, -a - h, 0.0),
                frame.point(l, -2.0 * a - h, 0.0),
            ];
            let old: Option<Vec<_>> = expected
                .into_iter()
                .map(|p| source_vertex(floor, p, tol))
                .collect();
            let old = old?;
            let pairs: Vec<_> = old.windows(2).map(|p| pair(p[0], p[1])).collect();
            let wire = topo.wire(topo.face(floor.id).ok()?.outer_wire()).ok()?;
            for oriented in wire.edges() {
                let edge = topo.edge(oriented.edge()).ok()?;
                if pairs.contains(&pair(edge.start(), edge.end())) {
                    continue;
                }
                let start = frame.local(topo.vertex(edge.start()).ok()?.point());
                let end = frame.local(topo.vertex(edge.end()).ok()?.point());
                let outer_clip = matches!(edge.curve(), EdgeCurve::Line)
                    && start.x().abs() <= tol.linear
                    && end.x().abs() <= tol.linear
                    && start.y().min(end.y()) >= -tol.linear;
                let front_clip = matches!(edge.curve(), EdgeCurve::Line)
                    && (start.y() - reach.lo[1]).abs() <= tol.linear
                    && (end.y() - reach.lo[1]).abs() <= tol.linear
                    && start.x().min(end.x()) >= l - tol.linear;
                if outer_clip || front_clip {
                    continue;
                }
                if edge_bounds(topo, oriented.edge(), frame)?.intersects(
                    reach,
                    &[0, 1],
                    -tol.linear,
                ) {
                    return None;
                }
            }
            for &id in &[foot.back_wall, foot.side_wall] {
                let view = views.iter().find(|v| v.id == id)?;
                let wire = topo.wire(topo.face(view.id).ok()?.outer_wire()).ok()?;
                for oe in wire.edges() {
                    if !matches!(topo.edge(oe.edge()).ok()?.curve(), EdgeCurve::Line) {
                        return None;
                    }
                }
            }
            cap_reach_clear(
                topo,
                views.iter().find(|v| v.id == foot.front_cap)?,
                foot,
                radius,
                true,
                tol,
            )?;
            cap_reach_clear(
                topo,
                views.iter().find(|v| v.id == foot.outer_cap)?,
                foot,
                radius,
                false,
                tol,
            )?;
            for face in &views {
                if allowed.contains(&face.id) {
                    continue;
                }
                if face_bounds(topo, face, frame)?.intersects(reach, &[0, 1, 2], -tol.linear) {
                    return None;
                }
            }
            for other in feet.iter().skip(ordinal + 1) {
                if other.floor != foot.floor {
                    return None;
                }
                let b = foot_bounds(other, radius);
                let mut in_frame = Bounds::empty();
                for x in [b.lo[0], b.hi[0]] {
                    for y in [b.lo[1], b.hi[1]] {
                        for z in [b.lo[2], b.hi[2]] {
                            in_frame.include(frame.local(other.frame.point(x, y, z)))?;
                        }
                    }
                }
                if in_frame.intersects(reach, &[0, 1, 2], -tol.linear) {
                    return None;
                }
                if allowed
                    .iter()
                    .filter(|&&face| face != foot.floor)
                    .any(|id| {
                        *id == other.back_wall
                            || *id == other.side_wall
                            || *id == other.front_cap
                            || *id == other.outer_cap
                            || *id == other.back_cylinder
                            || *id == other.front_cylinder
                    })
                {
                    return None;
                }
            }
        }
        Some(())
    }

    // Returns only appended construction. Its caller owns a topology transaction;
    // a rejected plan never commits these provisional handles.
    fn construct(
        topo: &mut Topology,
        source: SolidId,
        feet: &[Foot],
        radius: f64,
        tol: Tolerance,
    ) -> Option<Constructed> {
        reach_is_clear(topo, source, feet, radius, tol)?;
        let faces = supports(topo, source)?;
        let mut loops: DetHashMap<FaceId, Vec<VertexId>> = faces
            .iter()
            .map(|face| (face.id, face.vertices.clone()))
            .collect();
        let mut edges = DetHashMap::default();
        for id in remus_topology::explorer::solid_edges(topo, source).ok()? {
            let edge = topo.edge(id).ok()?;
            if edge.start() == edge.end()
                || edges.insert(pair(edge.start(), edge.end()), id).is_some()
            {
                return None;
            }
        }
        let mut new_specs = Vec::<(Vec<VertexId>, FaceSurface, bool, Vec<FaceId>)>::new();
        let mut changed = Vec::<FaceId>::new();
        for foot in feet {
            let f = foot.frame;
            let a = foot.old_radius;
            let l = foot.length;
            let h = foot.side_length;
            let depth = 2.0 * a + h;
            let ring = a + radius;
            let torus_front =
                ToroidalSurface::with_axis(f.point(l, -a - h, radius), ring, radius, f.up).ok()?;
            let opposite = f.up.cross(-f.back).dot(f.along) < 0.0;
            let runout = super::inner_tangent_runout(&torus_front, -f.back, opposite, false)?;
            let points = [
                f.point(0.0, radius, 0.0),
                f.point(l, radius, 0.0),
                f.point(l + ring, -a, 0.0),
                f.point(l + ring, -a - h, 0.0),
                runout.evaluate(0.0),
                f.point(0.0, 0.0, radius),
                f.point(l, 0.0, radius),
                f.point(l + a, -a, radius),
                f.point(l + a, -a - h, radius),
                runout.evaluate(1.0),
            ];
            if points.iter().any(|p| p.0.iter().any(|v| !v.is_finite())) {
                return None;
            }
            let ids = points.map(|point| topo.add_vertex(Vertex::new(point, tol.linear)));
            let view = |id| faces.iter().find(|face| face.id == id);
            let floor_view = view(foot.floor)?;
            let o = source_vertex(floor_view, f.origin, tol)?;
            let b = source_vertex(floor_view, f.point(l, 0.0, 0.0), tol)?;
            let p = source_vertex(floor_view, f.point(l + a, -a, 0.0), tol)?;
            let q = source_vertex(floor_view, f.point(l + a, -a - h, 0.0), tol)?;
            let c = source_vertex(floor_view, f.point(l, -depth, 0.0), tol)?;
            alter_floor(
                topo,
                loops.get_mut(&foot.floor)?,
                [o, b, p, q, c],
                [ids[FO], ids[FB], ids[FP], ids[FQ], ids[FD]],
                f,
                l,
                depth,
                radius,
                tol,
            )?;
            alter_cap(
                topo,
                loops.get_mut(&foot.front_cap)?,
                c,
                ids[WC],
                ids[FD],
                f,
                true,
                depth,
                tol,
            )?;
            alter_cap(
                topo,
                loops.get_mut(&foot.outer_cap)?,
                o,
                ids[WO],
                ids[FO],
                f,
                false,
                depth,
                tol,
            )?;
            replace_ids(
                loops.get_mut(&foot.back_wall)?,
                &[(o, ids[WO]), (b, ids[WB])],
            );
            replace_ids(
                loops.get_mut(&foot.side_wall)?,
                &[(p, ids[WP]), (q, ids[WQ])],
            );
            replace_ids(
                loops.get_mut(&foot.back_cylinder)?,
                &[(b, ids[WB]), (p, ids[WP])],
            );
            replace_ids(
                loops.get_mut(&foot.front_cylinder)?,
                &[(q, ids[WQ]), (c, ids[WC])],
            );
            changed.extend([
                foot.floor,
                foot.back_wall,
                foot.side_wall,
                foot.front_cap,
                foot.outer_cap,
                foot.back_cylinder,
                foot.front_cylinder,
            ]);
            for (start, end, centre, normal, r) in [
                (FO, WO, f.point(0.0, radius, radius), f.along, radius),
                (FB, WB, f.point(l, radius, radius), f.along, radius),
                (FP, WP, f.point(l + ring, -a, radius), -f.back, radius),
                (FQ, WQ, f.point(l + ring, -a - h, radius), -f.back, radius),
                (FB, FP, f.point(l, -a, 0.0), f.up, ring),
                (WB, WP, f.point(l, -a, radius), f.up, a),
                (FQ, FD, f.point(l, -a - h, 0.0), f.up, ring),
                (WQ, WC, f.point(l, -a - h, radius), f.up, a),
            ] {
                native_circle(
                    topo, &mut edges, ids[start], ids[end], centre, normal, r, tol,
                )?;
            }
            let mut edge = Edge::with_tolerance(
                ids[FD],
                ids[WC],
                EdgeCurve::NurbsCurve(runout),
                Some(tol.linear),
            );
            edge.set_trim(Some((0.0, 1.0)));
            if edges
                .insert(pair(ids[FD], ids[WC]), topo.add_edge(edge))
                .is_some()
            {
                return None;
            }
            let torus_back =
                ToroidalSurface::with_axis(f.point(l, -a, radius), ring, radius, f.up).ok()?;
            let back_band =
                CylindricalSurface::new(f.point(0.0, radius, radius), f.along, radius).ok()?;
            let side_band =
                CylindricalSurface::new(f.point(l + ring, -a, radius), -f.back, radius).ok()?;
            let mirrored = f.along.cross(f.back).dot(f.up) < 0.0;
            for (corners, surface, parents) in [
                (
                    [FO, FB, WB, WO],
                    FaceSurface::Cylinder(back_band),
                    vec![foot.floor, foot.back_wall],
                ),
                (
                    [FB, FP, WP, WB],
                    FaceSurface::Torus(torus_back),
                    vec![foot.floor, foot.back_cylinder],
                ),
                (
                    [FP, FQ, WQ, WP],
                    FaceSurface::Cylinder(side_band),
                    vec![foot.floor, foot.side_wall],
                ),
                (
                    [FQ, FD, WC, WQ],
                    FaceSurface::Torus(torus_front),
                    vec![foot.floor, foot.front_cylinder],
                ),
            ] {
                let mut vertices = corners.map(|i| ids[i]).to_vec();
                if mirrored {
                    vertices.reverse();
                }
                new_specs.push((vertices, surface, true, parents));
            }
        }
        changed.sort_unstable();
        changed.dedup();
        let mut result_faces = Vec::new();
        let mut modified = DetHashMap::default();
        for face in faces {
            if changed.binary_search(&face.id).is_err() {
                result_faces.push(face.id);
                continue;
            }
            let vertices = loops.get(&face.id)?;
            let id = mint_face(topo, &mut edges, vertices, face.surface, face.reversed, tol)?;
            if let Some(attributes) = topo.attributes().face(face.id).cloned() {
                topo.set_face_attributes(id, attributes).ok()?;
            }
            modified.insert(face.id, id);
            result_faces.push(id);
        }
        let mut bands = Vec::new();
        for (vertices, surface, reversed, parents) in new_specs {
            let id = mint_face(topo, &mut edges, &vertices, surface, reversed, tol)?;
            result_faces.push(id);
            bands.push((id, parents));
        }
        let shell = topo.add_shell(Shell::new(result_faces).ok()?);
        let solid = topo.add_solid(Solid::new(shell, Vec::new()));
        Some(Constructed {
            solid,
            modified,
            bands,
        })
    }

    struct Spandrel {
        source: FaceId,
        centre: Point3,
        radius: f64,
        points: [Point3; 3],
        hull: [Point3; 6],
        corner: Point3,
        error: f64,
    }
    fn quarter_spandrel(
        topo: &Topology,
        face: &Support,
        up: Vec3,
        tol: Tolerance,
    ) -> Option<Spandrel> {
        if face.vertices.len() != 3
            || face
                .normal()
                .is_none_or(|n| (n - up).length() > tol.angular)
        {
            return None;
        }
        let wire = topo.wire(topo.face(face.id).ok()?.outer_wire()).ok()?;
        let mut curved = None;
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge()).ok()?;
            match edge.curve() {
                EdgeCurve::Line => {}
                EdgeCurve::Circle(circle) => {
                    if curved.is_some() {
                        return None;
                    }
                    let (p0, p1) = edge.strict_domain().ok()?;
                    if ((p1 - p0).abs() - std::f64::consts::FRAC_PI_2).abs() > tol.angular
                        || circle.normal().cross(up).length() > tol.angular
                    {
                        return None;
                    }
                    curved = Some((circle, edge.start(), edge.end(), p0, p1));
                }
                EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => return None,
            }
        }
        let (circle, a, b, p0, p1) = curved?;
        let corner_id = *face.vertices.iter().find(|&&v| v != a && v != b)?;
        let a = topo.vertex(a).ok()?.point();
        let b = topo.vertex(b).ok()?.point();
        let corner = topo.vertex(corner_id).ok()?.point();
        if (corner - (a + (b - circle.center()))).length() > tol.linear {
            return None;
        }
        let scale = [a, b, corner, circle.center()]
            .into_iter()
            .flat_map(|p| p.0)
            .map(f64::abs)
            .fold(circle.radius().max(1.0), f64::max);
        let error = 512.0 * f64::EPSILON * scale
            + 4.0 * circle.radius() * ((p1 - p0).abs() - std::f64::consts::FRAC_PI_2).abs();
        if !error.is_finite() || error > tol.linear / 16.0 {
            return None;
        }
        Some(Spandrel {
            source: face.id,
            centre: circle.center(),
            radius: circle.radius(),
            points: [a, b, corner],
            hull: [
                a,
                b,
                corner,
                circle.evaluate(p0),
                circle.evaluate(p1),
                circle.evaluate(p0) + (circle.evaluate(p1) - circle.center()),
            ],
            corner,
            error,
        })
    }
    fn away_tangent(topo: &Topology, oriented: OrientedEdge, at_use_start: bool) -> Option<Vec3> {
        let edge = topo.edge(oriented.edge()).ok()?;
        let (p0, p1) = edge.strict_domain().ok()?;
        let at_native_start = if at_use_start {
            oriented.is_forward()
        } else {
            !oriented.is_forward()
        };
        let (a, b) = (
            topo.vertex(edge.start()).ok()?.point(),
            topo.vertex(edge.end()).ok()?.point(),
        );
        let t = if at_native_start { p0 } else { p1 };
        let sign = if at_native_start {
            (p1 - p0).signum()
        } else {
            -(p1 - p0).signum()
        };
        (edge.curve().tangent_with_endpoints(t, a, b) * sign)
            .normalize()
            .ok()
    }
    fn main_floor_retains_area(
        topo: &Topology,
        face: &Support,
        feet: &[Foot],
        radius: f64,
        tol: Tolerance,
    ) -> bool {
        let Ok(data) = topo.face(face.id) else {
            return false;
        };
        let Ok(wire) = topo.wire(data.outer_wire()) else {
            return false;
        };
        for (i, &point) in face.points.iter().enumerate() {
            if !feet.iter().all(|foot| {
                let p = foot.frame.local(point);
                let b = foot_bounds(foot, radius);
                p.x() < b.lo[0] - tol.linear
                    || p.x() > b.hi[0] + tol.linear
                    || p.y() < b.lo[1] - tol.linear
                    || p.y() > b.hi[1] + tol.linear
            }) {
                continue;
            }
            let n = wire.edges().len();
            let (Some(prev), Some(next)) = (
                away_tangent(topo, wire.edges()[(i + n - 1) % n], false),
                away_tangent(topo, wire.edges()[i], true),
            ) else {
                continue;
            };
            if prev.dot(next) < 1.0 - tol.angular {
                return true;
            }
        }
        false
    }
    struct OriginalRegions {
        parents: DetHashMap<FaceId, Vec<FaceId>>,
        main_floor: FaceId,
        deleted: Vec<FaceId>,
        front_spandrels: Vec<Option<FaceId>>,
    }
    fn original_regions(
        topo: &Topology,
        originals: &[Support],
        copy: &crate::copy::CopiedSolidEntities,
        union: &crate::heal::FaceUnifyHistory,
        feet: &[Foot],
        radius: f64,
        tol: Tolerance,
    ) -> Option<OriginalRegions> {
        let final_by_copy: DetHashMap<_, _> = union.modified.iter().copied().collect();
        let mut parents = DetHashMap::<FaceId, Vec<FaceId>>::default();
        for original in originals {
            let copied = copy.face_map.get(&original.id.index())?;
            let canonical = *final_by_copy.get(copied)?;
            parents.entry(canonical).or_default().push(original.id);
        }
        for ids in parents.values_mut() {
            ids.sort_unstable();
            ids.dedup();
        }
        let floor = feet.first()?.floor;
        if feet.iter().any(|foot| foot.floor != floor) {
            return None;
        }
        let up = feet[0].frame.up;
        let mut main = None;
        let mut spandrels = Vec::new();
        for id in parents.get(&floor)? {
            let view = originals.iter().find(|view| view.id == *id)?;
            let wire = topo.wire(topo.face(view.id).ok()?.outer_wire()).ok()?;
            let curved_triangle = view.vertices.len() == 3
                && wire.edges().iter().any(|oe| {
                    topo.edge(oe.edge())
                        .is_ok_and(|edge| !matches!(edge.curve(), EdgeCurve::Line))
                });
            if curved_triangle {
                // A failed spandrel certificate is not a second route to a
                // nominal main-floor classification.
                spandrels.push(quarter_spandrel(topo, view, up, tol)?);
            } else if main.replace(view).is_some() {
                return None;
            }
        }
        let main = main?;
        if !main_floor_retains_area(topo, main, feet, radius, tol) {
            return None;
        }
        for foot in feet {
            for id in [
                foot.back_wall,
                foot.side_wall,
                foot.back_cylinder,
                foot.front_cylinder,
            ] {
                if parents.get(&id)?.len() != 1 {
                    return None;
                }
            }
        }
        let mut deleted = Vec::new();
        let mut front_spandrels = vec![None; feet.len()];
        for spandrel in spandrels {
            let mut selected = None;
            for (i, foot) in feet.iter().enumerate() {
                let f = foot.frame;
                let a = foot.old_radius;
                let l = foot.length;
                let h = foot.side_length;
                let centre = f.point(l, -a - h, 0.0);
                let expected = [
                    f.point(l + a, -a - h, 0.0),
                    f.point(l, -2.0 * a - h, 0.0),
                    f.point(l + a, -2.0 * a - h, 0.0),
                ];
                if (spandrel.centre - centre).length() > tol.linear
                    || (spandrel.radius - a).abs() > tol.linear
                    || !expected.iter().all(|&p| {
                        spandrel
                            .points
                            .iter()
                            .any(|&q| (p - q).length() <= tol.linear)
                    })
                {
                    continue;
                }
                if selected.replace(i).is_some() {
                    return None;
                }
            }
            if let Some(i) = selected {
                if front_spandrels[i].replace(spandrel.source).is_some() {
                    return None;
                }
                let foot = &feet[i];
                let centre =
                    foot.frame
                        .point(foot.length, -foot.old_radius - foot.side_length, 0.0);
                let ring = foot.old_radius + radius;
                // A quarter-spandrel lies in the convex triangle of its three
                // exact vertices. The norm reaches its maximum at a vertex.
                let max = spandrel
                    .hull
                    .iter()
                    .map(|&p| (p - centre).length())
                    .fold(0.0_f64, f64::max);
                if max + spandrel.error < ring - tol.linear {
                    deleted.push(spandrel.source);
                } else if (spandrel.corner - centre).length() <= ring + tol.linear + spandrel.error
                {
                    return None;
                }
            } else {
                for foot in feet {
                    let mut b = Bounds::empty();
                    for p in spandrel.hull {
                        b.include(foot.frame.local(p))?;
                    }
                    if b.intersects(
                        foot_bounds(foot, radius),
                        &[0, 1],
                        -tol.linear - spandrel.error,
                    ) {
                        return None;
                    }
                }
            }
        }
        deleted.sort_unstable();
        Some(OriginalRegions {
            parents,
            main_floor: main.id,
            deleted,
            front_spandrels,
        })
    }
    fn face_origins(
        regions: OriginalRegions,
        feet: &[Foot],
        built: &Constructed,
    ) -> Option<remus_blend::BlendFaceOrigins> {
        let mut origins = remus_blend::BlendFaceOrigins::default();
        for (&canonical, parents) in &regions.parents {
            let result = *built.modified.get(&canonical).unwrap_or(&canonical);
            for &parent in parents {
                if !regions.deleted.contains(&parent) {
                    origins.survived.push((parent, result));
                }
            }
        }
        for (result, canonical_parents) in &built.bands {
            if canonical_parents.len() != 2 {
                return None;
            }
            let support = canonical_parents[1];
            let (i, foot) = feet.iter().enumerate().find(|(_, foot)| {
                [
                    foot.back_wall,
                    foot.side_wall,
                    foot.back_cylinder,
                    foot.front_cylinder,
                ]
                .contains(&support)
            })?;
            let original_supports = regions.parents.get(&support)?;
            if original_supports.len() != 1 {
                return None;
            }
            let mut from = vec![regions.main_floor, original_supports[0]];
            if support == foot.front_cylinder
                && let Some(spandrel) = regions.front_spandrels[i]
            {
                from.push(spandrel);
            }
            from.sort_unstable();
            from.dedup();
            origins.created.push((*result, from));
        }
        origins.survived.sort_unstable();
        origins
            .created
            .sort_unstable_by_key(|(face, _)| face.index());
        origins.deleted = regions.deleted;
        Some(origins)
    }

    fn separation(a: Point3, b: Point3) -> f64 {
        let delta = a - b;
        delta.x().hypot(delta.y()).hypot(delta.z())
    }

    fn input_edge_authority(topo: &Topology, input: SolidId) -> Option<()> {
        // Preserve the claimed vertex balls and authoritative native trims.
        // Scan only this solid's edges, without a document-wide incident-edge
        // query or any tolerance inflation/endpoint re-projection.
        for id in remus_topology::explorer::solid_edges(topo, input).ok()? {
            let edge = topo.edge(id).ok()?;
            let start = topo.vertex(edge.start()).ok()?;
            let end = topo.vertex(edge.end()).ok()?;
            let (t0, t1) = edge.strict_domain().ok()?;
            for (vertex, parameter) in [(start, t0), (end, t1)] {
                let ball = vertex.tolerance();
                if !ball.is_finite()
                    || ball < 0.0
                    || vertex.point().0.iter().any(|x| !x.is_finite())
                {
                    return None;
                }
                let on_curve =
                    edge.curve()
                        .evaluate_with_endpoints(parameter, start.point(), end.point());
                if on_curve.0.iter().any(|x| !x.is_finite())
                    || separation(on_curve, vertex.point()) > ball
                {
                    return None;
                }
            }
        }
        Some(())
    }

    fn preflight(
        topo: &Topology,
        input: SolidId,
        seeds: &[EdgeId],
        radius: f64,
        tol: Tolerance,
    ) -> Option<Vec<Support>> {
        if seeds.is_empty() || seeds.len() > 2 {
            return None;
        }
        input_edge_authority(topo, input)?;
        // This first pass is read-only. A split coplanar front spandrel may
        // be absent from the seed's own floor wire; full recognition follows
        // the construction-traced copy/union, never before an arbitrary copy.
        for &seed in seeds {
            recognize_impl(topo, input, seed, radius, tol, false)?;
        }
        if !crate::validate::validate_solid(topo, input)
            .ok()?
            .is_valid()
        {
            return None;
        }
        let originals = supports(topo, input)?;
        // Existing per-use data is input authority. An unsupported or invalid
        // stored use must not disappear inside copy/unification and become a
        // clean result merely because new coordinates can be generated.
        for face in &originals {
            let wire = topo.wire(topo.face(face.id).ok()?.outer_wire()).ok()?;
            for oe in wire.edges() {
                if topo
                    .pcurve_oriented(oe.edge(), face.id, oe.is_forward())
                    .is_some()
                {
                    remus_topology::validation::validate_same_parameter_strict(
                        topo,
                        oe.edge(),
                        face.id,
                        oe.is_forward(),
                        tol.linear,
                        16,
                    )
                    .ok()?;
                    remus_topology::validation::validate_same_range_strict(
                        topo,
                        oe.edge(),
                        face.id,
                        oe.is_forward(),
                        tol.linear,
                    )
                    .ok()?;
                }
            }
        }
        Some(originals)
    }

    fn reconstruct_with_regions(
        topo: &mut Topology,
        input: SolidId,
        seeds: &[EdgeId],
        radius: f64,
        tol: Tolerance,
        originals: Vec<Support>,
    ) -> Option<(Constructed, remus_blend::BlendFaceOrigins)> {
        let copy = crate::copy::copy_solid_with_entity_map(topo, input).ok()?;
        let source = copy.solid;
        let copied: Option<Vec<_>> = seeds
            .iter()
            .map(|seed| copy.edge_map.get(&seed.index()).copied())
            .collect();
        let copied = copied?;
        let union = crate::heal::unify_faces_with_history(topo, source).ok()?;
        let live_edges = remus_topology::explorer::solid_edges(topo, source).ok()?;
        if copied.iter().any(|seed| !live_edges.contains(seed)) {
            return None;
        }
        let feet: Option<Vec<_>> = copied
            .into_iter()
            .map(|seed| recognize(topo, source, seed, radius, tol))
            .collect();
        let feet = feet?;
        reach_is_clear(topo, source, &feet, radius, tol)?;
        let regions = original_regions(topo, &originals, &copy, &union, &feet, radius, tol)?;
        let built = construct(topo, source, &feet, radius, tol)?;
        let origins = face_origins(regions, &feet, &built)?;
        let input_faces: remus_math::det_hash::DetHashSet<_> =
            originals.iter().map(|face| face.id).collect();
        let result_faces: remus_math::det_hash::DetHashSet<_> =
            remus_topology::explorer::solid_faces(topo, built.solid)
                .ok()?
                .into_iter()
                .collect();
        let mut input_coverage = remus_math::det_hash::DetHashSet::default();
        let mut result_coverage = remus_math::det_hash::DetHashSet::default();
        for &(from, to) in &origins.survived {
            if !input_faces.contains(&from)
                || !result_faces.contains(&to)
                || !input_coverage.insert(from)
            {
                return None;
            }
            result_coverage.insert(to);
        }
        for &from in &origins.deleted {
            if !input_faces.contains(&from) || !input_coverage.insert(from) {
                return None;
            }
        }
        for (to, from) in &origins.created {
            if !result_faces.contains(to)
                || !result_coverage.insert(*to)
                || from.is_empty()
                || from.iter().any(|id| !input_faces.contains(id))
            {
                return None;
            }
        }
        if input_coverage != input_faces
            || result_coverage != result_faces
            || !origins.created_unattributed.is_empty()
        {
            return None;
        }
        Some((built, origins))
    }

    fn plane_chart(normal: Vec3, d: f64) -> Option<(Point3, Vec3, Vec3)> {
        let square = normal.dot(normal);
        if !square.is_finite() || square <= 0.0 {
            return None;
        }
        let origin = Point3::new(
            normal.x() * d / square,
            normal.y() * d / square,
            normal.z() * d / square,
        );
        let unit = normal.normalize().ok()?;
        let seed = if unit.x().abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let u = unit.cross(seed).normalize().ok()?;
        Some((origin, u, unit.cross(u)))
    }

    fn surface_uv(surface: &FaceSurface, point: Point3) -> Option<remus_math::vec::Point2> {
        use remus_math::vec::Point2;
        let uv = match surface {
            FaceSurface::Plane { normal, d } => {
                let (o, u, v) = plane_chart(*normal, *d)?;
                let q = point - o;
                Point2::new(q.dot(u), q.dot(v))
            }
            FaceSurface::Cylinder(c) => {
                let q = point - c.origin();
                Point2::new(q.dot(c.y_axis()).atan2(q.dot(c.x_axis())), q.dot(c.axis()))
            }
            FaceSurface::Torus(t) => {
                let q = point - t.center();
                let x = q.dot(t.x_axis());
                let y = q.dot(t.y_axis());
                Point2::new(
                    y.atan2(x),
                    q.dot(t.z_axis()).atan2(x.hypot(y) - t.major_radius()),
                )
            }
            FaceSurface::Cone(_) | FaceSurface::Sphere(_) | FaceSurface::Nurbs(_) => return None,
        };
        uv.0.iter().all(|x| x.is_finite()).then_some(uv)
    }

    fn periodic_coordinates(surface: &FaceSurface) -> (bool, bool) {
        match surface {
            FaceSurface::Cylinder(_) => (true, false),
            FaceSurface::Torus(_) => (true, true),
            FaceSurface::Plane { .. }
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Nurbs(_) => (false, false),
        }
    }

    fn unwrap_after(next: f64, previous: f64) -> f64 {
        next + std::f64::consts::TAU * ((previous - next) / std::f64::consts::TAU).round()
    }

    fn edge_uv_curve(
        topo: &Topology,
        id: EdgeId,
        surface: &FaceSurface,
        samples: usize,
    ) -> Option<(remus_math::curves2d::Curve2D, f64, f64)> {
        use remus_math::curves2d::{Circle2D, Curve2D, Line2D, NurbsCurve2D};
        use remus_math::vec::{Point2, Vec2};
        let edge = topo.edge(id).ok()?;
        let (a, b) = edge.strict_domain().ok()?;
        let start_point = topo.vertex(edge.start()).ok()?.point();
        let end_point = topo.vertex(edge.end()).ok()?.point();
        let point = |f: f64| {
            edge.curve()
                .evaluate_with_endpoints((b - a).mul_add(f, a), start_point, end_point)
        };
        let (period_u, period_v) = periodic_coordinates(surface);
        let mut uv = Vec::new();
        let mut parameters = Vec::new();
        for i in 0..=samples {
            let f = i as f64 / samples as f64;
            let mut p = surface_uv(surface, point(f))?;
            if let Some(previous) = uv.last().copied() {
                let previous: Point2 = previous;
                if period_u {
                    p.0[0] = unwrap_after(p.x(), previous.x());
                }
                if period_v {
                    p.0[1] = unwrap_after(p.y(), previous.y());
                }
            }
            uv.push(p);
            parameters.push(f);
        }
        let start = *uv.first()?;
        let mut end = *uv.last()?;
        if matches!(
            (surface, edge.curve()),
            (FaceSurface::Cylinder(_), EdgeCurve::Line)
        ) {
            end.0[0] = start.x();
        }
        match (surface, edge.curve()) {
            (FaceSurface::Plane { normal, d }, EdgeCurve::NurbsCurve(curve))
                if curve.degree() == 4 =>
            {
                let (o, u, v) = plane_chart(*normal, *d)?;
                let poles = curve
                    .control_points()
                    .iter()
                    .map(|&p| Point2::new((p - o).dot(u), (p - o).dot(v)))
                    .collect();
                Some((
                    Curve2D::Nurbs(
                        NurbsCurve2D::new(
                            4,
                            curve.knots().to_vec(),
                            poles,
                            curve.weights().to_vec(),
                        )
                        .ok()?,
                    ),
                    a,
                    b,
                ))
            }
            (FaceSurface::Plane { normal, d }, EdgeCurve::Circle(circle)) => {
                let (o, u, v) = plane_chart(*normal, *d)?;
                let q = circle.center() - o;
                let center = Point2::new(q.dot(u), q.dot(v));
                let start_angle = (start.y() - center.y()).atan2(start.x() - center.x());
                let sign = if circle.normal().dot(u.cross(v)) >= 0.0 {
                    1.0
                } else {
                    -1.0
                };
                Some((
                    Curve2D::Circle(Circle2D::new(center, circle.radius()).ok()?),
                    start_angle,
                    start_angle + sign * (b - a),
                ))
            }
            (FaceSurface::Torus(_), EdgeCurve::NurbsCurve(curve)) if curve.degree() == 4 => {
                let points: Vec<_> = uv
                    .into_iter()
                    .map(|p| Point3::new(p.x(), p.y(), 0.0))
                    .collect();
                let fitted =
                    remus_math::nurbs::fitting::interpolate_with_params(&points, 3, &parameters)
                        .ok()?;
                let poles = fitted
                    .control_points()
                    .iter()
                    .map(|p| Point2::new(p.x(), p.y()))
                    .collect();
                Some((
                    Curve2D::Nurbs(
                        NurbsCurve2D::new(
                            3,
                            fitted.knots().to_vec(),
                            poles,
                            fitted.weights().to_vec(),
                        )
                        .ok()?,
                    ),
                    0.0,
                    1.0,
                ))
            }
            (FaceSurface::Plane { .. }, EdgeCurve::Line)
            | (
                FaceSurface::Cylinder(_) | FaceSurface::Torus(_),
                EdgeCurve::Line | EdgeCurve::Circle(_),
            ) => {
                let delta = end - start;
                let length = delta.length();
                if !length.is_finite() || length <= 0.0 {
                    return None;
                }
                Some((
                    Curve2D::Line(Line2D::new(start, Vec2::new(delta.x(), delta.y())).ok()?),
                    0.0,
                    length,
                ))
            }
            (
                FaceSurface::Plane { .. } | FaceSurface::Cylinder(_) | FaceSurface::Torus(_),
                EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_),
            )
            | (
                FaceSurface::Cone(_) | FaceSurface::Sphere(_) | FaceSurface::Nurbs(_),
                EdgeCurve::Line
                | EdgeCurve::Circle(_)
                | EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_),
            ) => None,
        }
    }

    fn shift_uv_curve(
        curve: remus_math::curves2d::Curve2D,
        shift: remus_math::vec::Vec2,
    ) -> Option<remus_math::curves2d::Curve2D> {
        use remus_math::curves2d::{Circle2D, Curve2D, Line2D, NurbsCurve2D};
        match curve {
            Curve2D::Line(line) => Some(Curve2D::Line(
                Line2D::new(line.origin() + shift, line.direction()).ok()?,
            )),
            Curve2D::Circle(circle) => Some(Curve2D::Circle(
                Circle2D::new(circle.center() + shift, circle.radius()).ok()?,
            )),
            Curve2D::Nurbs(nurbs) => Some(Curve2D::Nurbs(
                NurbsCurve2D::new(
                    nurbs.degree(),
                    nurbs.knots().to_vec(),
                    nurbs.control_points().iter().map(|&p| p + shift).collect(),
                    nurbs.weights().to_vec(),
                )
                .ok()?,
            )),
            Curve2D::Ellipse(_) => None,
        }
    }

    // Build complete per-use coordinates. The exact native 3D curves remain
    // authoritative. Only the quartic torus UV curve is approximated, and it
    // is accepted solely by the strict whole-interval certificate.
    fn register_runout_pcurves(topo: &mut Topology, solid: SolidId, tol: Tolerance) -> Option<()> {
        use remus_math::vec::{Point2, Vec2};
        use remus_topology::pcurve::PCurve;
        let faces = remus_topology::explorer::solid_faces(topo, solid).ok()?;
        for id in faces {
            let face = topo.face(id).ok()?;
            if !face.inner_wires().is_empty() {
                return None;
            }
            let surface = face.surface().clone();
            let wire = face.outer_wire();
            let edges = topo.wire(wire).ok()?.edges().to_vec();
            // Unchanged copied faces retain their already qualified native
            // coordinates verbatim. Modified/new faces have no stored uses.
            if edges.iter().all(|oe| {
                topo.pcurve_oriented(oe.edge(), id, oe.is_forward())
                    .is_some()
            }) {
                let mut first = None;
                let mut previous: Option<Point2> = None;
                for oe in &edges {
                    let pc = topo.pcurve_oriented(oe.edge(), id, oe.is_forward())?;
                    let start = pc.evaluate(pc.t_start());
                    let end = pc.evaluate(pc.t_end());
                    if let Some(p) = previous {
                        if (start - p).length() > tol.linear {
                            return None;
                        }
                    } else {
                        first = Some(start);
                    }
                    previous = Some(end);
                    remus_topology::validation::validate_same_parameter_strict(
                        topo,
                        oe.edge(),
                        id,
                        oe.is_forward(),
                        tol.linear,
                        16,
                    )
                    .ok()?;
                    remus_topology::validation::validate_same_range_strict(
                        topo,
                        oe.edge(),
                        id,
                        oe.is_forward(),
                        tol.linear,
                    )
                    .ok()?;
                }
                if (previous? - first?).length() > tol.linear {
                    return None;
                }
                continue;
            }
            let (period_u, period_v) = periodic_coordinates(&surface);
            let mut proved = false;
            for samples in [16, 32, 64, 128, 256] {
                let mut previous: Option<Point2> = None;
                let mut first = None;
                for oe in &edges {
                    let (curve, a, b) = edge_uv_curve(topo, oe.edge(), &surface, samples)?;
                    let (p0, p1) = if oe.is_forward() { (a, b) } else { (b, a) };
                    let start = curve.evaluate(p0);
                    let shift = previous.map_or(Vec2::new(0.0, 0.0), |p| {
                        Vec2::new(
                            if period_u {
                                unwrap_after(start.x(), p.x()) - start.x()
                            } else {
                                0.0
                            },
                            if period_v {
                                unwrap_after(start.y(), p.y()) - start.y()
                            } else {
                                0.0
                            },
                        )
                    });
                    let curve = shift_uv_curve(curve, shift)?;
                    let start = curve.evaluate(p0);
                    let end = curve.evaluate(p1);
                    if let Some(p) = previous {
                        if (start - p).length() > tol.linear {
                            return None;
                        }
                    } else {
                        first = Some(start);
                    }
                    previous = Some(end);
                    topo.set_pcurve_oriented(
                        oe.edge(),
                        id,
                        oe.is_forward(),
                        PCurve::new(curve, p0, p1),
                    )
                    .ok()?;
                }
                if (previous? - first?).length() > tol.linear {
                    return None;
                }
                proved = edges.iter().all(|oe| {
                    remus_topology::validation::validate_same_parameter_strict(
                        topo,
                        oe.edge(),
                        id,
                        oe.is_forward(),
                        tol.linear,
                        16,
                    )
                    .is_ok()
                        && remus_topology::validation::validate_same_range_strict(
                            topo,
                            oe.edge(),
                            id,
                            oe.is_forward(),
                            tol.linear,
                        )
                        .is_ok()
                        && topo
                            .pcurve_oriented(oe.edge(), id, oe.is_forward())
                            .is_some()
                });
                if proved {
                    break;
                }
            }
            if !proved {
                return None;
            }
        }
        Some(())
    }

    pub(super) fn qualified_attempt(
        topo: &mut Topology,
        input: SolidId,
        seeds: &[EdgeId],
        radius: f64,
        tol: Tolerance,
    ) -> Result<remus_blend::BlendResult, crate::OperationsError> {
        // Reject unrecognized input before taking the transaction's topology
        // snapshot. Admission itself is read-only; original region authority
        // remains stable under this exclusive mutable borrow.
        let originals = preflight(topo, input, seeds, radius, tol).ok_or_else(|| {
            crate::OperationsError::Unsupported {
                operation: "fillet",
                reason: "outside the certified quarter-cylinder tab runout family".into(),
            }
        })?;
        remus_topology::transaction::run_transacted(topo, |topo| {
            let (built, origins) = reconstruct_with_regions(
                topo, input, seeds, radius, tol, originals,
            )
            .ok_or_else(|| crate::OperationsError::Unsupported {
                operation: "fillet",
                reason: "outside the certified quarter-cylinder tab runout family".into(),
            })?;
            register_runout_pcurves(topo, built.solid, tol).ok_or_else(|| {
                crate::OperationsError::Unsupported {
                    operation: "fillet",
                    reason: "quarter-cylinder runout boundary parameter proof is unavailable"
                        .into(),
                }
            })?;
            let mut baseline = crate::blend_ops::BlendBaseline::new(topo, input);
            crate::blend_ops::validate_blend_solid_against_input(
                &mut baseline,
                "fillet",
                built.solid,
            )?;
            crate::blend_ops::validate_blend_volume(
                &mut baseline,
                "fillet",
                built.solid,
                seeds,
                crate::blend_ops::BlendSize::Fillet { radius },
            )?;
            Ok(remus_blend::BlendResult {
                solid: built.solid,
                succeeded: seeds.to_vec(),
                failed: Vec::new(),
                is_partial: false,
                face_origins: Some(origins),
                engine: remus_blend::BlendEngine::RollingBall,
            })
        })
    }

    #[cfg(test)]
    fn fixture_with_seeds(topo: &mut Topology) -> Option<(SolidId, Vec<EdgeId>)> {
        let source = remus_io::arena_io::deserialize_solid(
            include_bytes!("../../tests/data/rollover_tabbed_source.arena.json"),
            topo,
        )
        .ok()?;
        let seeds: Vec<_> = remus_topology::explorer::solid_edges(topo, source)
            .ok()?
            .into_iter()
            .filter(|&id| {
                let Ok(edge) = topo.edge(id) else {
                    return false;
                };
                let (Ok(a), Ok(b)) = (topo.vertex(edge.start()), topo.vertex(edge.end())) else {
                    return false;
                };
                let (a, b) = (a.point(), b.point());
                matches!(edge.curve(), EdgeCurve::Line)
                    && (a.y() - 10.0).abs() < 1e-8
                    && (b.y() - 10.0).abs() < 1e-8
                    && (a.z() - 4.0).abs() < 1e-8
                    && (b.z() - 4.0).abs() < 1e-8
            })
            .collect();
        if seeds.len() != 2 {
            return None;
        }
        Some((source, seeds))
    }

    #[cfg(test)]
    pub(super) fn constructed_body_for_test(topo: &mut Topology, radius: f64) -> Option<SolidId> {
        let (source, seeds) = fixture_with_seeds(topo)?;
        let before = remus_io::arena_io::serialize_solid(topo, source).ok()?;
        let result = qualified_attempt(topo, source, &seeds, radius, Tolerance::new()).ok()?;
        let origins = result.face_origins.as_ref()?;
        let deleted = if radius > 2.0 * (2.0_f64.sqrt() - 1.0) {
            2
        } else {
            0
        };
        if origins.survived.len() != 32 - deleted
            || origins.deleted.len() != deleted
            || origins.created.len() != 8
        {
            return None;
        }
        if remus_io::arena_io::serialize_solid(topo, source).ok()? != before {
            return None;
        }
        Some(result.solid)
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn inner_tangent_complete_pcurves_cover_single_foot_and_reject_corrupt_input() {
        use remus_math::curves2d::{Curve2D, Line2D};
        use remus_math::vec::{Point2, Vec2};
        use remus_topology::pcurve::PCurve;
        let mut topo = Topology::new();
        let (source, seeds) = fixture_with_seeds(&mut topo).unwrap();
        let before = remus_io::arena_io::serialize_solid(&topo, source).unwrap();
        let result =
            qualified_attempt(&mut topo, source, &seeds[..1], 3.0, Tolerance::new()).unwrap();
        let faces = remus_topology::explorer::solid_faces(&topo, result.solid).unwrap();
        assert_eq!(faces.len(), 24);
        let mut uses = 0;
        let mut quartic_uses = 0;
        for face in faces {
            let wire = topo.wire(topo.face(face).unwrap().outer_wire()).unwrap();
            for oe in wire.edges() {
                assert!(
                    topo.pcurve_oriented(oe.edge(), face, oe.is_forward())
                        .is_some()
                );
                remus_topology::validation::validate_same_parameter_strict(
                    &topo,
                    oe.edge(),
                    face,
                    oe.is_forward(),
                    1e-7,
                    2,
                )
                .unwrap();
                remus_topology::validation::validate_same_range_strict(
                    &topo,
                    oe.edge(),
                    face,
                    oe.is_forward(),
                    1e-7,
                )
                .unwrap();
                uses += 1;
                if matches!(topo.edge(oe.edge()).unwrap().curve(),EdgeCurve::NurbsCurve(n) if n.degree()==4)
                {
                    quartic_uses += 1;
                }
            }
        }
        assert_eq!(quartic_uses, 2);
        assert!(uses > 100);
        // Endpoints stay identical while one interior coordinate pole moves.
        // Neither strict contract may certify that hidden parameter mismatch.
        let (face,oe)=remus_topology::explorer::solid_faces(&topo,result.solid).unwrap().into_iter().find_map(|face| {
            if !matches!(topo.face(face).ok()?.surface(),FaceSurface::Plane{..}) {return None;}
            topo.wire(topo.face(face).ok()?.outer_wire()).ok()?.edges().iter().copied()
                .find(|oe|matches!(topo.edge(oe.edge()).ok().map(remus_topology::edge::Edge::curve),Some(EdgeCurve::NurbsCurve(n)) if n.degree()==4))
                .map(|oe|(face,oe))
        }).unwrap();
        let pc = topo
            .pcurve_oriented(oe.edge(), face, oe.is_forward())
            .unwrap()
            .clone();
        let Curve2D::Nurbs(nurbs) = pc.curve() else {
            unreachable!()
        };
        let mut poles = nurbs.control_points().to_vec();
        poles[2].0[0] += 0.01;
        let wrong = remus_math::curves2d::NurbsCurve2D::new(
            4,
            nurbs.knots().to_vec(),
            poles,
            nurbs.weights().to_vec(),
        )
        .unwrap();
        topo.set_pcurve_oriented(
            oe.edge(),
            face,
            oe.is_forward(),
            PCurve::new(Curve2D::Nurbs(wrong), pc.t_start(), pc.t_end()),
        )
        .unwrap();
        assert!(
            remus_topology::validation::validate_same_parameter_strict(
                &topo,
                oe.edge(),
                face,
                oe.is_forward(),
                1e-7,
                2
            )
            .is_err()
        );
        assert!(
            remus_topology::validation::validate_same_range_strict(
                &topo,
                oe.edge(),
                face,
                oe.is_forward(),
                1e-7
            )
            .is_err()
        );
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, source).unwrap(),
            before
        );
        let source_faces = remus_topology::explorer::solid_faces(&topo, source).unwrap();
        let face = source_faces
            .into_iter()
            .find(|&id| matches!(topo.face(id).unwrap().surface(), FaceSurface::Plane { .. }))
            .unwrap();
        let oe = *topo
            .wire(topo.face(face).unwrap().outer_wire())
            .unwrap()
            .edges()
            .iter()
            .find(|oe| matches!(topo.edge(oe.edge()).unwrap().curve(), EdgeCurve::Line))
            .unwrap();
        topo.set_pcurve_oriented(
            oe.edge(),
            face,
            oe.is_forward(),
            PCurve::new(
                Curve2D::Line(Line2D::new(Point2::new(1e6, 1e6), Vec2::new(1.0, 0.0)).unwrap()),
                0.0,
                1.0,
            ),
        )
        .unwrap();
        let corrupt = remus_io::arena_io::serialize_solid(&topo, source).unwrap();
        let live = (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_faces(),
            topo.num_coedges(),
        );
        assert!(qualified_attempt(&mut topo, source, &seeds, 3.0, Tolerance::new()).is_err());
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, source).unwrap(),
            corrupt
        );
        assert_eq!(
            (
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_faces(),
                topo.num_coedges()
            ),
            live
        );
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn inner_tangent_original_regions_distinguish_buried_and_retained_floor_faces() {
        for radius in [0.5, 1.0, 3.0] {
            let mut topo = Topology::new();
            let (source, seeds) = fixture_with_seeds(&mut topo).unwrap();
            let original = remus_topology::explorer::solid_faces(&topo, source).unwrap();
            let expected_spandrels:Vec<_>=original.iter().copied().filter(|&id|{
                let face=topo.face(id).unwrap();
                matches!(face.surface(),FaceSurface::Plane{normal,d} if normal.z()>0.99 && (*d-4.0).abs()<1e-8)
                    && topo.wire(face.outer_wire()).unwrap().edges().len()==3
            }).collect();
            assert_eq!(expected_spandrels.len(), 2);
            let before = remus_io::arena_io::serialize_solid(&topo, source).unwrap();
            let result =
                qualified_attempt(&mut topo, source, &seeds, radius, Tolerance::new()).unwrap();
            assert!(!result.is_partial && result.failed.is_empty());
            let origins = result.face_origins.as_ref().unwrap();
            let expected_deleted = if radius > 0.9 { 2 } else { 0 };
            assert_eq!(origins.deleted.len(), expected_deleted);
            assert_eq!(origins.survived.len(), 32 - expected_deleted);
            assert_eq!(origins.created.len(), 8);
            assert!(origins.created_unattributed.is_empty());
            for &deleted in &origins.deleted {
                assert!(expected_spandrels.contains(&deleted));
            }
            assert_eq!(
                remus_topology::explorer::solid_faces(&topo, result.solid)
                    .unwrap()
                    .len(),
                28
            );
            assert_eq!(
                remus_io::arena_io::serialize_solid(&topo, source).unwrap(),
                before
            );
        }
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn inner_tangent_refuses_poisoned_original_trim_without_repairing_input() {
        for phase in [-1e-5, 1e-5] {
            let mut topo = Topology::new();
            let (source, seeds) = fixture_with_seeds(&mut topo).unwrap();
            let edge=remus_topology::explorer::solid_edges(&topo,source).unwrap().into_iter().find(|&id| {
                let edge=topo.edge(id).unwrap();
                matches!(edge.curve(),EdgeCurve::Circle(c) if (c.center().x()-16.0).abs()<1e-8 && (c.center().y()-8.0).abs()<1e-8)
                    && (topo.vertex(edge.start()).unwrap().point().z()-4.0).abs()<1e-8
                    && (topo.vertex(edge.end()).unwrap().point().z()-4.0).abs()<1e-8
            }).unwrap();
            let (a, b) = topo.edge(edge).unwrap().strict_domain().unwrap();
            topo.edge_mut(edge)
                .unwrap()
                .set_trim(Some((a + phase, b + phase)));
            assert!(
                crate::validate::validate_solid(&topo, source)
                    .unwrap()
                    .is_valid()
            );
            let vertex = topo.edge(edge).unwrap().start();
            assert!(
                remus_topology::validation::check_vertex_ball(&topo, vertex)
                    .unwrap()
                    .iter()
                    .any(|report| report.edge == edge && report.deviation > 1e-6)
            );
            for face in remus_topology::explorer::solid_faces(&topo, source).unwrap() {
                for oe in topo
                    .wire(topo.face(face).unwrap().outer_wire())
                    .unwrap()
                    .edges()
                {
                    assert!(
                        topo.pcurve_oriented(oe.edge(), face, oe.is_forward())
                            .is_none()
                    );
                }
            }
            let before = remus_io::arena_io::serialize_solid(&topo, source).unwrap();
            let live = [
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_wires(),
                topo.num_faces(),
                topo.num_solids(),
                topo.num_loops(),
                topo.num_coedges(),
            ];
            let allocated = topo.allocated_slot_count();
            let journal = topo.journal().entries().len();
            assert!(qualified_attempt(&mut topo, source, &seeds, 3.0, Tolerance::new()).is_err());
            assert_eq!(
                remus_io::arena_io::serialize_solid(&topo, source).unwrap(),
                before
            );
            assert_eq!(
                [
                    topo.num_vertices(),
                    topo.num_edges(),
                    topo.num_wires(),
                    topo.num_faces(),
                    topo.num_solids(),
                    topo.num_loops(),
                    topo.num_coedges()
                ],
                live
            );
            assert_eq!(topo.allocated_slot_count(), allocated);
            assert_eq!(topo.journal().entries().len(), journal);
        }
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn inner_tangent_qualified_attempt_rolls_back_unqualified_contact() {
        let mut topo = Topology::new();
        let (source, seeds) = fixture_with_seeds(&mut topo).unwrap();
        let before = remus_io::arena_io::serialize_solid(&topo, source).unwrap();
        let census = |topo: &Topology| {
            [
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_wires(),
                topo.num_faces(),
                topo.num_shells(),
                topo.num_solids(),
                topo.num_loops(),
                topo.num_coedges(),
            ]
        };
        let live = census(&topo);
        let journal = topo.journal().entries().len();
        let allocated = topo.allocated_slot_count();
        for radius in [4.0, 4.1, 2.0 * (2.0_f64.sqrt() - 1.0)] {
            assert!(
                qualified_attempt(&mut topo, source, &seeds, radius, Tolerance::new()).is_err()
            );
            assert_eq!(census(&topo), live);
            assert_eq!(topo.journal().entries().len(), journal);
            assert_eq!(
                remus_io::arena_io::serialize_solid(&topo, source).unwrap(),
                before
            );
            assert!(topo.allocated_slot_count() >= allocated);
        }
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn inner_tangent_reach_refuses_tangent_and_crossing_notch_contacts() {
        let mut topo = Topology::new();
        let source = remus_io::arena_io::deserialize_solid(
            include_bytes!("../../tests/data/rollover_tabbed_source.arena.json"),
            &mut topo,
        )
        .unwrap();
        crate::heal::unify_faces_with_history(&mut topo, source).unwrap();
        let feet: Vec<_> = remus_topology::explorer::solid_edges(&topo, source)
            .unwrap()
            .into_iter()
            .filter(|&id| {
                let e = topo.edge(id).unwrap();
                let a = topo.vertex(e.start()).unwrap().point();
                let b = topo.vertex(e.end()).unwrap().point();
                matches!(e.curve(), EdgeCurve::Line)
                    && (a.y() - 10.0).abs() < 1e-8
                    && (b.y() - 10.0).abs() < 1e-8
                    && (a.z() - 4.0).abs() < 1e-8
                    && (b.z() - 4.0).abs() < 1e-8
            })
            .map(|seed| recognize(&topo, source, seed, 3.0, Tolerance::new()).unwrap())
            .collect();
        assert_eq!(feet.len(), 2);
        for radius in [0.5, 1.0, 3.0, 3.9] {
            assert!(
                reach_is_clear(&topo, source, &feet, radius, Tolerance::new()).is_some(),
                "clear radius {radius}"
            );
        }
        for radius in [4.0, 4.1, 40.0] {
            assert!(
                reach_is_clear(&topo, source, &feet, radius, Tolerance::new()).is_none(),
                "unqualified radius {radius}"
            );
        }
        assert!(
            reach_is_clear(&topo, source, &[feet[0], feet[0]], 3.0, Tolerance::new()).is_none()
        );
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn inner_tangent_recognizes_both_tab_supports() {
        let mut topo = Topology::new();
        let source = remus_io::arena_io::deserialize_solid(
            include_bytes!("../../tests/data/rollover_tabbed_source.arena.json"),
            &mut topo,
        )
        .unwrap();
        crate::heal::unify_faces_with_history(&mut topo, source).unwrap();
        let seeds: Vec<_> = remus_topology::explorer::solid_edges(&topo, source)
            .unwrap()
            .into_iter()
            .filter(|&id| {
                let edge = topo.edge(id).unwrap();
                let a = topo.vertex(edge.start()).unwrap().point();
                let b = topo.vertex(edge.end()).unwrap().point();
                matches!(edge.curve(), EdgeCurve::Line)
                    && (a.y() - 10.0).abs() < 1e-8
                    && (b.y() - 10.0).abs() < 1e-8
                    && (a.z() - 4.0).abs() < 1e-8
                    && (b.z() - 4.0).abs() < 1e-8
            })
            .collect();
        assert_eq!(seeds.len(), 2);
        for seed in seeds {
            let foot = recognize(&topo, source, seed, 3.0, Tolerance::new()).unwrap();
            assert!((foot.old_radius - 2.0).abs() < 1e-10);
            assert!((foot.length - 16.0).abs() < 1e-10);
            assert!((foot.side_length - 6.0).abs() < 1e-10);
            assert!((foot.height - 36.0).abs() < 1e-10);
            assert!((foot.frame.up.z() - 1.0).abs() < 1e-10);
            let roles = [
                foot.floor,
                foot.back_wall,
                foot.side_wall,
                foot.front_cap,
                foot.outer_cap,
                foot.back_cylinder,
                foot.front_cylinder,
            ];
            let mut unique = roles.to_vec();
            unique.sort_unstable();
            unique.dedup();
            assert_eq!(unique.len(), roles.len());
        }
    }
}

/// Last-attempt exact runout for one or two disjoint quarter-cylinder tab
/// feet. All recognition, construction, boundary proofs and history must
/// qualify; refusal leaves the caller's existing diagnostic authoritative.
pub fn inner_tangent_tab_fillet(
    topo: &mut remus_topology::Topology,
    solid: remus_topology::solid::SolidId,
    edges: &[remus_topology::edge::EdgeId],
    radius: f64,
) -> Result<remus_blend::BlendResult, crate::OperationsError> {
    bounded_tab_supports::qualified_attempt(topo, solid, edges, radius, Tolerance::new())
}

/// Test-only bridge for raw boundary witnesses. Public fillet routes use the qualified attempt above.
#[cfg(test)]
pub fn constructed_tab_runout_for_test(
    topo: &mut remus_topology::Topology,
    radius: f64,
) -> Option<remus_topology::solid::SolidId> {
    bounded_tab_supports::constructed_body_for_test(topo, radius)
}
