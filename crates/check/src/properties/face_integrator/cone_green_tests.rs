//! Closed-form oracles for cone faces trimmed by curved NURBS sections that
//! lie on the cone (see `cone_green`).
//!
//! A plane `n · (P − O) = d` tilted by `θ` from the cone's axis cuts the
//! cone (apex `O`, half-angle `a` from the radial plane) in an ellipse. The
//! section is carried as the exact rational quadratic image of the cone's
//! direction circle under central projection, so it lies on the cone to
//! rounding and is curved in `(u, v)`: the sampled trim chords it.
//!
//! The oracles come from elementary solid geometry, not from any integrator:
//!
//! * the solid between the apex and the plane is a cone over the elliptic
//!   section, so its volume is `π A B d / 3` and its centroid lies `3/4` of
//!   the way from the apex to the ellipse's centre;
//! * the section's semi-axes are read off the two generators in the tilt
//!   plane (major axis) and the horizontal circle through the ellipse's
//!   centre (minor axis: `B² = R_M² − e²`);
//! * the planar cap's own terms are the ellipse's area moments in closed
//!   form, so the lateral face's terms are the solid's minus the cap's.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::cell::Cell;
use std::f64::consts::PI;

use remus_geometry::convert::curve_to_nurbs::circle_to_nurbs;
use remus_math::curves::Circle3D;
use remus_math::nurbs::NurbsCurve;
use remus_math::surfaces::ConicalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use crate::properties::PropertiesOptions;

use super::{
    FaceContribution, IntegrationRule, MomentRequest, analytic_face_area, cone_green,
    integrate_face_about, integrate_face_area, integrate_face_fixed_about,
};

/// One oblique section of a cone, in a frame with the cone's axis `â` and
/// the tilt direction `t̂` (unit, perpendicular to `â`).
pub(super) struct Section {
    pub(super) apex: Point3,
    pub(super) axis: Vec3,
    pub(super) tilt_dir: Vec3,
    /// The cone's half-angle from its radial plane.
    pub(super) a: f64,
    /// The plane normal's angle from the axis.
    pub(super) theta: f64,
    /// The plane's distance from the apex.
    pub(super) d: f64,
}

impl Section {
    pub(super) fn normal(&self) -> Vec3 {
        self.tilt_dir * self.theta.sin() + self.axis * self.theta.cos()
    }

    /// Generator directions in the tilt plane, toward and away from `t̂`.
    fn generators(&self) -> [Vec3; 2] {
        let (sin_a, cos_a) = self.a.sin_cos();
        [
            self.tilt_dir * cos_a + self.axis * sin_a,
            -self.tilt_dir * cos_a + self.axis * sin_a,
        ]
    }

    /// The exact section: centre, unit major direction, semi-axes.
    pub(super) fn ellipse(&self) -> (Point3, Vec3, f64, f64) {
        let n = self.normal();
        let [g1, g2] = self.generators();
        let p1 = self.apex + g1 * (self.d / n.dot(g1));
        let p2 = self.apex + g2 * (self.d / n.dot(g2));
        let centre = p1 + (p2 - p1) * 0.5;
        let major = (p2 - p1).normalize().unwrap();
        let semi_major = (p2 - p1).length() / 2.0;
        let height = (centre - self.apex).dot(self.axis);
        let radius = height / self.a.tan();
        let eccentric = (centre - self.apex) - self.axis * height;
        let semi_minor = radius.mul_add(radius, -eccentric.dot(eccentric)).sqrt();
        (centre, major, semi_major, semi_minor)
    }

    /// The section as an exact rational quadratic NURBS on the cone, run in
    /// `−u` (clockwise seen from the axis tip), the way an outer rim above a
    /// pointed cone's material runs in `(u, v)`.
    pub(super) fn rim(&self) -> NurbsCurve {
        let (sin_a, cos_a) = self.a.sin_cos();
        // The direction circle `cos a ρ(u) + sin a â` about the apex.
        let circle = Circle3D::new(self.apex + self.axis * sin_a, -self.axis, cos_a).unwrap();
        let directions = circle_to_nurbs(&circle, 0.0, std::f64::consts::TAU).unwrap();
        let n = self.normal();
        let mut points = Vec::new();
        let mut weights = Vec::new();
        for (point, &weight) in directions.control_points().iter().zip(directions.weights()) {
            let g = *point - self.apex;
            let along = n.dot(g);
            assert!(along > 0.0, "control direction behind the plane");
            points.push(self.apex + g * (self.d / along));
            weights.push(weight * along);
        }
        NurbsCurve::new(2, directions.knots().to_vec(), points, weights).unwrap()
    }

    pub(super) fn cone(&self) -> ConicalSurface {
        ConicalSurface::new(self.apex, self.axis, self.a).unwrap()
    }

    /// The lateral face's exact `(volume, first volume moments)` about
    /// `reference`: the solid cone's minus the planar cap's.
    pub(super) fn lateral_terms(&self, reference: Point3) -> (f64, [f64; 3]) {
        let n = self.normal();
        let (centre, major, semi_major, semi_minor) = self.ellipse();
        let base = PI * semi_major * semi_minor;
        let solid = base * self.d / 3.0;
        // About `reference`, the cap's volume term is `(1/3)(P − R)·n A`.
        let cap_volume = (centre - reference).dot(n) * base / 3.0;
        let centroid = self.apex + (centre - self.apex) * 0.75;
        // `∫_V (x − R_x) dV` for the solid; `∫ ½ (x − R_x)² n_x dA` for the
        // cap, a point `M + s m̂ + t ŵ` of it with `∫ s² dA = π A³ B / 4`
        // along the major axis and `∫ t² dA = π A B³ / 4` along the minor.
        let minor = n.cross(major);
        let moment = |component: fn(Vec3) -> f64| {
            let c = component(centroid - reference);
            let m = component(centre - reference);
            let (k, w) = (component(major), component(minor));
            let spread = PI * semi_major * semi_minor / 4.0
                * (k * k).mul_add(semi_major * semi_major, w * w * semi_minor * semi_minor);
            let cap = 0.5 * component(n) * (m * m).mul_add(base, spread);
            c.mul_add(solid, -cap)
        };
        (
            solid - cap_volume,
            [moment(Vec3::x), moment(Vec3::y), moment(Vec3::z)],
        )
    }
}

pub(super) fn reversed(curve: &NurbsCurve) -> NurbsCurve {
    let points: Vec<_> = curve.control_points().iter().rev().copied().collect();
    let weights: Vec<_> = curve.weights().iter().rev().copied().collect();
    let (lo, hi) = curve.domain();
    let knots: Vec<_> = curve.knots().iter().rev().map(|k| lo + hi - k).collect();
    NurbsCurve::new(curve.degree(), knots, points, weights).unwrap()
}

pub(super) fn closed_nurbs_edge(topo: &mut Topology, curve: NurbsCurve) -> (EdgeId, VertexId) {
    let start = curve.evaluate(curve.domain().0);
    let vertex = topo.add_vertex(Vertex::new(start, 1e-9));
    let domain = curve.domain();
    let mut edge = Edge::new(vertex, vertex, EdgeCurve::NurbsCurve(curve));
    edge.set_trim(Some(domain));
    (topo.add_edge(edge), vertex)
}

/// The band between two sections in the kernel's frustum layout: the near
/// rim in `+u`, the seam up, the far rim in `−u`, the seam down.
fn band_face(topo: &mut Topology, near: &Section, far: &Section) -> FaceId {
    let (near_rim, near_vertex) = closed_nurbs_edge(topo, reversed(&near.rim()));
    let (far_rim, far_vertex) = closed_nurbs_edge(topo, far.rim());
    let seam = topo.add_edge(Edge::new(near_vertex, far_vertex, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(near_rim, true),
                OrientedEdge::new(seam, true),
                OrientedEdge::new(far_rim, true),
                OrientedEdge::new(seam, false),
            ],
            true,
        )
        .unwrap(),
    );
    topo.add_face(Face::new(wire, vec![], FaceSurface::Cone(near.cone())))
}

/// The pointed face from the apex up to `section`'s rim, in the kernel's
/// cone layout: the rim in `−u`, then the seam to the apex and back.
fn pointed_face(topo: &mut Topology, section: &Section) -> FaceId {
    let (rim, rim_vertex) = closed_nurbs_edge(topo, section.rim());
    let apex = topo.add_vertex(Vertex::new(section.apex, 1e-9));
    let seam = topo.add_edge(Edge::new(rim_vertex, apex, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(rim, true),
                OrientedEdge::new(seam, true),
                OrientedEdge::new(seam, false),
            ],
            true,
        )
        .unwrap(),
    );
    topo.add_face(Face::new(wire, vec![], FaceSurface::Cone(section.cone())))
}

/// Options that request refinement, which keeps the sampled trim.
pub(super) fn refined() -> PropertiesOptions {
    PropertiesOptions {
        adaptive_eps: 1e-8,
        ..PropertiesOptions::default()
    }
}

pub(super) fn rel(measured: f64, exact: f64, scale: f64) -> f64 {
    (measured - exact).abs() / scale
}

pub(super) fn sections(scale: f64) -> Vec<Section> {
    let tilt = |phi: f64, axis: Vec3| {
        let helper = if axis.x().abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let t = helper.cross(axis).normalize().unwrap();
        let s = axis.cross(t);
        t * phi.cos() + s * phi.sin()
    };
    let axis = Vec3::new(0.2, -0.3, 1.0).normalize().unwrap();
    vec![
        Section {
            apex: Point3::new(0.0, 0.0, 0.0),
            axis: Vec3::new(0.0, 0.0, 1.0),
            tilt_dir: Vec3::new(1.0, 0.0, 0.0),
            a: 1.0,
            theta: 0.35,
            d: 2.0 * scale,
        },
        Section {
            apex: Point3::new(0.7 * scale, -1.1 * scale, 0.4 * scale),
            axis,
            tilt_dir: tilt(0.8, axis),
            a: 0.6,
            theta: 0.35,
            d: 3.0 * scale,
        },
        Section {
            apex: Point3::new(-2.0 * scale, 0.5 * scale, 1.5 * scale),
            axis: -axis,
            tilt_dir: tilt(2.3, -axis),
            a: 1.2,
            theta: 0.9,
            d: 1.5 * scale,
        },
    ]
}

/// The lateral face of an obliquely cut cone integrates to the closed-form
/// volume and first moments at every scale, about the plane and about a
/// point off it, while the sampled trim reads it a chord error off.
#[test]
fn an_oblique_section_integrates_to_the_closed_form_cone() {
    for scale in [1e-3, 1.0, 1e3] {
        for (index, section) in sections(scale).iter().enumerate() {
            let mut topo = Topology::new();
            let face = pointed_face(&mut topo, section);
            let on_plane = section.apex + section.normal() * section.d;
            let off_plane = on_plane + Vec3::new(0.3, 0.8, -0.5) * scale;
            for reference in [on_plane, off_plane] {
                let (volume, moments) = section.lateral_terms(reference);
                let contribution: FaceContribution =
                    integrate_face_about(&topo, face, &PropertiesOptions::default(), reference)
                        .unwrap();
                let label = format!("section {index} at scale {scale}");
                let v_scale = section.d.powi(3);
                assert!(
                    rel(contribution.volume, volume, v_scale) < 1e-12,
                    "{label}: volume {} vs {volume}",
                    contribution.volume
                );
                for (axis, (measured, exact)) in [
                    contribution.volume_moment_x,
                    contribution.volume_moment_y,
                    contribution.volume_moment_z,
                ]
                .into_iter()
                .zip(moments)
                .enumerate()
                {
                    let m_scale = v_scale * (section.d + (reference - section.apex).length());
                    assert!(
                        rel(measured, exact, m_scale) < 1e-11,
                        "{label}: moment {axis} {measured} vs {exact}"
                    );
                }
                // The face's area against the independent boundary
                // reduction of `analytic_face_area`.
                let area = analytic_face_area(&topo, face, 8).unwrap();
                assert!(
                    rel(contribution.area, area, area) < 1e-12,
                    "{label}: area {} vs {area}",
                    contribution.area
                );
                // The area-only rule reads the face with its moments, so on
                // the boundary too (it once kept the sampled trim).
                let area_only = integrate_face_area(&topo, face, 5).unwrap();
                assert!(
                    rel(area_only, area, area) < 1e-12,
                    "{label}: area-only {area_only} vs {area}"
                );
                // The sampled trim (explicit refinement keeps it) chords the
                // section: the oracle tells the two paths apart.
                let sampled = integrate_face_about(&topo, face, &refined(), reference)
                    .unwrap()
                    .area;
                assert!(
                    rel(sampled, area, area) > 1e-8,
                    "{label}: sampled area {sampled} already exact"
                );
            }
        }
    }
}

/// Fixed-order readings (`integrate_face`, `solid_properties` at any order)
/// take the same boundary path.
#[test]
fn fixed_order_readings_agree_with_the_closed_form() {
    let section = &sections(1.0)[1];
    let mut topo = Topology::new();
    let face = pointed_face(&mut topo, section);
    let reference = section.apex + Vec3::new(0.5, 0.25, -1.0);
    let (volume, _) = section.lateral_terms(reference);
    for order in [3, 5, 8] {
        let c = integrate_face_fixed_about(&topo, face, order, reference).unwrap();
        assert!(
            rel(c.volume, volume, section.d.powi(3)) < 1e-12,
            "order {order}: {} vs {volume}",
            c.volume
        );
    }
}

/// The face-integral cache answers a fitted cone face's full contribution
/// bit for bit as the uncached integrator does (review of #1856: its
/// flux-collecting miss path read the sampled trim), enabled or disabled,
/// cold or warm, and a volume request after it keeps its own reading.
#[test]
fn the_face_cache_matches_the_uncached_boundary_reading() {
    use crate::properties::face_cache::FaceIntegralCache;
    let section = &sections(1.0)[1];
    let mut topo = Topology::new();
    let face = pointed_face(&mut topo, section);
    let reference = section.apex + Vec3::new(0.5, 0.25, -1.0);
    let (volume, _) = section.lateral_terms(reference);
    let fresh_volume = {
        let mut cold = FaceIntegralCache::new();
        cold.integrate_face_volume_about(&topo, face, 5, reference)
            .unwrap()
    };
    let uncached = integrate_face_fixed_about(&topo, face, 5, reference).unwrap();
    assert!(rel(uncached.volume, volume, section.d.powi(3)) < 1e-12);
    for mut cache in [FaceIntegralCache::new(), FaceIntegralCache::disabled()] {
        for _ in 0..2 {
            let cached = cache
                .integrate_face_fixed_about(&topo, face, 5, reference)
                .unwrap();
            // `f64`'s Debug form round-trips, so equal text is equal bits.
            assert_eq!(format!("{cached:?}"), format!("{uncached:?}"));
        }
        let after = cache
            .integrate_face_volume_about(&topo, face, 5, reference)
            .unwrap();
        assert_eq!(after.volume.to_bits(), fresh_volume.volume.to_bits());
        // The area-only family reads the boundary too, cold and warm.
        let uncached_area = integrate_face_area(&topo, face, 5).unwrap();
        assert_eq!(uncached_area.to_bits(), uncached.area.to_bits());
        for _ in 0..2 {
            let cached = cache.integrate_face_area(&topo, face, 5).unwrap();
            assert_eq!(cached.to_bits(), uncached_area.to_bits());
        }
    }
}

/// The band between two oblique sections: the frustum's closed-form volume
/// less its two caps' terms.
#[test]
fn a_band_between_two_oblique_sections_integrates_exactly() {
    for scale in [1e-3, 1.0, 1e3] {
        let near = Section {
            apex: Point3::new(0.4 * scale, 0.1 * scale, -0.3 * scale),
            axis: Vec3::new(0.0, 0.0, 1.0),
            tilt_dir: Vec3::new(0.6, 0.8, 0.0),
            a: 0.9,
            theta: 0.3,
            d: 1.0 * scale,
        };
        let far = Section {
            tilt_dir: Vec3::new(-0.8, 0.6, 0.0),
            theta: 0.5,
            d: 2.5 * scale,
            ..near
        };
        let mut topo = Topology::new();
        let face = band_face(&mut topo, &near, &far);
        let reference = near.apex + Vec3::new(0.7, -0.2, 0.9) * scale;
        let (far_volume, far_moments) = far.lateral_terms(reference);
        let (near_volume, near_moments) = near.lateral_terms(reference);
        let c =
            integrate_face_about(&topo, face, &PropertiesOptions::default(), reference).unwrap();
        let v_scale = far.d.powi(3);
        assert!(
            rel(c.volume, far_volume - near_volume, v_scale) < 1e-12,
            "scale {scale}: {} vs {}",
            c.volume,
            far_volume - near_volume
        );
        for (axis, measured) in [c.volume_moment_x, c.volume_moment_y, c.volume_moment_z]
            .into_iter()
            .enumerate()
        {
            let exact = far_moments[axis] - near_moments[axis];
            assert!(
                rel(measured, exact, v_scale * 4.0 * far.d) < 1e-11,
                "scale {scale}: moment {axis} {measured} vs {exact}"
            );
        }
        let area = analytic_face_area(&topo, face, 8).unwrap();
        assert!(rel(c.area, area, area) < 1e-12, "{} vs {area}", c.area);
    }
}

fn rule(flux: Option<&Cell<Option<Vec3>>>) -> IntegrationRule<'_> {
    IntegrationRule {
        order: 5,
        adaptive: None,
        knots: None,
        reference: Point3::new(0.0, 0.0, 0.0),
        moments: MomentRequest::Full,
        ruled_guard: false,
        flux,
        cone_apex: None,
        boundary: None,
    }
}

/// Faces with only exact trims and area-only readings never take the
/// boundary path, so their results stay bit for bit. (An area-only request
/// on a fitted face reaches it through the moments reading instead; see
/// `integrate_face_area`.) A reading that collects the vector area takes it
/// exactly as the moments reading does, and gets the certificate of its
/// fitted rim: an exact conic on the cone, so it converges to round-off and
/// is read where it lies.
#[test]
fn the_boundary_path_is_scoped_to_fitted_moment_readings() {
    let section = &sections(1.0)[0];
    let mut topo = Topology::new();
    let fitted = pointed_face(&mut topo, section);
    let cone = section.cone();
    let face = topo.face(fitted).unwrap().clone();
    let moments = cone_green::integrate::<false>(&topo, &face, &cone, &rule(None), 1.0)
        .unwrap()
        .unwrap();
    let sink = Cell::new(None);
    let collecting = cone_green::integrate::<false>(&topo, &face, &cone, &rule(Some(&sink)), 1.0)
        .unwrap()
        .unwrap();
    assert_eq!(
        collecting.contribution.volume.to_bits(),
        moments.contribution.volume.to_bits()
    );
    assert_eq!(
        collecting.contribution.area.to_bits(),
        moments.contribution.area.to_bits()
    );
    assert_eq!(
        collecting.flux.0.map(f64::to_bits),
        moments.flux.0.map(f64::to_bits)
    );
    let fitted_reading = collecting.fitted.unwrap();
    // About the apex the volume term vanishes: compare at the area's scale.
    let area = collecting.contribution.area;
    let volume = area * area.sqrt();
    assert!(
        fitted_reading.span_disagreement <= 1e-12 * volume,
        "{fitted_reading:?}"
    );
    assert!(
        fitted_reading.closure_allowance <= 1e-12 * area,
        "{fitted_reading:?}"
    );
    assert!(
        cone_green::integrate::<true>(&topo, &face, &cone, &rule(None), 1.0)
            .unwrap()
            .is_none()
    );
    // A coaxial circle rim: exact trims keep the sampled reading.
    let height = 2.0;
    let rim = Circle3D::new(
        Point3::new(0.0, 0.0, height),
        Vec3::new(0.0, 0.0, -1.0),
        height / section.a.tan(),
    )
    .unwrap();
    let vertex = topo.add_vertex(Vertex::new(rim.evaluate(0.0), 1e-9));
    let mut edge = Edge::new(vertex, vertex, EdgeCurve::Circle(rim));
    edge.set_trim(Some((0.0, std::f64::consts::TAU)));
    let edge = topo.add_edge(edge);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let exact = topo.add_face(Face::new(wire, vec![], FaceSurface::Cone(cone.clone())));
    let face = topo.face(exact).unwrap().clone();
    assert!(
        cone_green::integrate::<false>(&topo, &face, &cone, &rule(None), 1.0)
            .unwrap()
            .is_none()
    );
}

/// A fitted rim pushed off the cone keeps the sampled reading.
#[test]
fn a_rim_off_the_cone_keeps_the_sampled_reading() {
    let section = &sections(1.0)[0];
    let rim = section.rim();
    let lifted: Vec<_> = rim
        .control_points()
        .iter()
        .map(|p| *p + Vec3::new(0.0, 0.0, 0.05))
        .collect();
    let lifted = NurbsCurve::new(2, rim.knots().to_vec(), lifted, rim.weights().to_vec()).unwrap();
    let mut topo = Topology::new();
    let (edge, _) = closed_nurbs_edge(&mut topo, lifted);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let cone = section.cone();
    let face_id = topo.add_face(Face::new(wire, vec![], FaceSurface::Cone(cone.clone())));
    let face = topo.face(face_id).unwrap().clone();
    assert!(
        cone_green::integrate::<false>(&topo, &face, &cone, &rule(None), 1.0)
            .unwrap()
            .is_none()
    );
}
