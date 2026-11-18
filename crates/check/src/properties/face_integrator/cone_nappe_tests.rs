//! Cone faces on both nappes of their carrier, against an independent
//! quadrature of the physical surface.
//!
//! A `ConicalSurface` is the double cone `P(u, v) = A + v (cos a ρ(u) +
//! sin a â)`: at `v < 0` the point at longitude `u` lies at `−ρ(u)`, half a
//! turn round the axis from where `project_point` (which answers the point's
//! own longitude) says. A face whose boundary lies below the apex therefore
//! reads its own mirror image across the axis unless the integrator charts it
//! on the reversed-axis carrier, where it lies at `v > 0`.
//!
//! The oracle never touches the carrier: it walks the band
//! `Q(φ, s) = A + s (cos a (cos φ x̂ + sin φ ŷ) + σ sin a â)` (`σ = ±1` the
//! nappe) on a Gauss–Legendre product grid, with the outward normal
//! `±∂Q/∂φ × ∂Q/∂s` (radial component positive), and sums exactly the
//! integrands the accumulator documents.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::curves::Circle3D;
use remus_math::quadrature::gauss_legendre_points;
use remus_math::surfaces::ConicalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use super::{
    FaceContribution, analytic_face_area, boundary_face_area, integrate_face_area,
    integrate_face_fixed_about, integrate_face_fixed_flux_about,
};

/// A band of the cone with apex `apex`, axis `ẑ`, half-angle `a`, between
/// generator distances `s0 < s1` and longitudes `φ0 < φ1`, on the nappe
/// `sigma` (`+1` above the apex, `−1` below).
struct Band {
    apex: Point3,
    a: f64,
    s: (f64, f64),
    phi: (f64, f64),
    sigma: f64,
}

impl Band {
    fn point(&self, phi: f64, s: f64) -> Point3 {
        let (sin_a, cos_a) = self.a.sin_cos();
        self.apex
            + Vec3::new(
                s * cos_a * phi.cos(),
                s * cos_a * phi.sin(),
                self.sigma * s * sin_a,
            )
    }

    /// The independent reading of the band about `reference`.
    fn oracle(&self, reference: Point3) -> FaceContribution {
        let (sin_a, cos_a) = self.a.sin_cos();
        let mut c = zero();
        let gauss = gauss_legendre_points(20);
        let (half_phi, mid_phi) = (
            (self.phi.1 - self.phi.0) / 2.0,
            f64::midpoint(self.phi.0, self.phi.1),
        );
        let (half_s, mid_s) = (
            (self.s.1 - self.s.0) / 2.0,
            f64::midpoint(self.s.0, self.s.1),
        );
        for gp in gauss {
            let phi = half_phi.mul_add(gp.x, mid_phi);
            for gs in gauss {
                let s = half_s.mul_add(gs.x, mid_s);
                let w = gp.w * half_phi * gs.w * half_s;
                let d_phi = Vec3::new(-s * cos_a * phi.sin(), s * cos_a * phi.cos(), 0.0);
                let d_s = Vec3::new(cos_a * phi.cos(), cos_a * phi.sin(), self.sigma * sin_a);
                let mut n = d_phi.cross(d_s);
                // Outward: away from the axis.
                if n.x() * phi.cos() + n.y() * phi.sin() < 0.0 {
                    n = -n;
                }
                let n_len = n.length();
                let p = self.point(phi, s) - reference;
                c.area += w * n_len;
                c.volume += w * p.dot(n) / 3.0;
                c.volume_moment_x += w * 0.5 * p.x() * p.x() * n.x();
                c.volume_moment_y += w * 0.5 * p.y() * p.y() * n.y();
                c.volume_moment_z += w * 0.5 * p.z() * p.z() * n.z();
                c.volume_second_x += w * p.x().powi(3) * n.x() / 3.0;
                c.volume_second_y += w * p.y().powi(3) * n.y() / 3.0;
                c.volume_second_z += w * p.z().powi(3) * n.z() / 3.0;
                c.volume_product_xy += w * 0.5 * p.x() * p.x() * p.y() * n.x();
                c.volume_product_xz += w * 0.5 * p.x() * p.x() * p.z() * n.x();
                c.volume_product_yz += w * 0.5 * p.y() * p.y() * p.z() * n.y();
                c.centroid_x += w * p.x() * n_len;
                c.centroid_y += w * p.y() * n_len;
                c.centroid_z += w * p.z() * n_len;
            }
        }
        c
    }

    /// The band as a face on `carrier`, wound counter-clockwise about the
    /// outward normal: two rim arcs and two generator lines.
    fn face(&self, topo: &mut Topology, carrier: ConicalSurface) -> FaceId {
        let (sin_a, cos_a) = self.a.sin_cos();
        let z = Vec3::new(0.0, 0.0, 1.0);
        let corners = [
            self.point(self.phi.0, self.s.0),
            self.point(self.phi.1, self.s.0),
            self.point(self.phi.1, self.s.1),
            self.point(self.phi.0, self.s.1),
        ];
        let vertices: Vec<_> = corners
            .iter()
            .map(|&p| topo.add_vertex(Vertex::new(p, 1e-9)))
            .collect();
        let rim =
            |s: f64| Circle3D::new(self.apex + z * (self.sigma * s * sin_a), z, s * cos_a).unwrap();
        let arc = |topo: &mut Topology, from: usize, to: usize, s: f64| {
            let circle = rim(s);
            let t0 = circle.project(corners[from]);
            let mut t1 = circle.project(corners[to]);
            if t1 <= t0 {
                t1 += std::f64::consts::TAU;
            }
            let mut edge = Edge::new(vertices[from], vertices[to], EdgeCurve::Circle(circle));
            edge.set_trim(Some((t0, t1)));
            topo.add_edge(edge)
        };
        let near = arc(topo, 0, 1, self.s.0);
        let far = arc(topo, 3, 2, self.s.1);
        let right = topo.add_edge(Edge::new(vertices[1], vertices[2], EdgeCurve::Line));
        let left = topo.add_edge(Edge::new(vertices[3], vertices[0], EdgeCurve::Line));
        // Counter-clockwise about the outward normal: along +φ on the rim
        // nearer the apex above it, and against +φ below it (the outward
        // normal leans the other way along the axis).
        let edges = if self.sigma > 0.0 {
            vec![
                OrientedEdge::new(near, true),
                OrientedEdge::new(right, true),
                OrientedEdge::new(far, false),
                OrientedEdge::new(left, true),
            ]
        } else {
            vec![
                OrientedEdge::new(left, false),
                OrientedEdge::new(far, true),
                OrientedEdge::new(right, false),
                OrientedEdge::new(near, false),
            ]
        };
        let wire = topo.add_wire(Wire::new(edges, true).unwrap());
        topo.add_face(Face::new(wire, vec![], FaceSurface::Cone(carrier)))
    }
}

fn zero() -> FaceContribution {
    FaceContribution {
        area: 0.0,
        volume: 0.0,
        volume_moment_x: 0.0,
        volume_moment_y: 0.0,
        volume_moment_z: 0.0,
        volume_second_x: 0.0,
        volume_second_y: 0.0,
        volume_second_z: 0.0,
        volume_product_xy: 0.0,
        volume_product_xz: 0.0,
        volume_product_yz: 0.0,
        centroid_x: 0.0,
        centroid_y: 0.0,
        centroid_z: 0.0,
    }
}

fn terms(c: &FaceContribution) -> [(&'static str, f64); 14] {
    [
        ("area", c.area),
        ("volume", c.volume),
        ("mx", c.volume_moment_x),
        ("my", c.volume_moment_y),
        ("mz", c.volume_moment_z),
        ("qxx", c.volume_second_x),
        ("qyy", c.volume_second_y),
        ("qzz", c.volume_second_z),
        ("qxy", c.volume_product_xy),
        ("qxz", c.volume_product_xz),
        ("qyz", c.volume_product_yz),
        ("cx", c.centroid_x),
        ("cy", c.centroid_y),
        ("cz", c.centroid_z),
    ]
}

/// Compare every term, each at its own dimension of `scale`.
fn assert_close(
    label: &str,
    got: &FaceContribution,
    want: &FaceContribution,
    scale: f64,
    tol: f64,
) {
    // Area ~ L², volume and centroid ~ L³, first moments ~ L⁴, second ~ L⁵.
    let dims = [2, 3, 4, 4, 4, 5, 5, 5, 5, 5, 5, 3, 3, 3];
    for ((name, g), ((_, w), dim)) in terms(got)
        .into_iter()
        .zip(terms(want).into_iter().zip(dims))
    {
        let bound = tol * scale.powi(dim);
        assert!(
            (g - w).abs() <= bound,
            "{label}: {name} reads {g}, oracle {w} (|Δ| {} > {bound})",
            (g - w).abs()
        );
    }
}

/// The carriers a face on nappe `sigma` can be written on: the one with its
/// axis along `ẑ` (the face at `v = sigma s`) and the one with its axis
/// reversed (at `v = −sigma s`).
fn carriers(band: &Band) -> [(&'static str, ConicalSurface); 2] {
    let z = Vec3::new(0.0, 0.0, 1.0);
    [
        (
            "axis +z",
            ConicalSurface::new(band.apex, z, band.a).unwrap(),
        ),
        (
            "axis -z",
            ConicalSurface::new(band.apex, -z, band.a).unwrap(),
        ),
    ]
}

/// A quarter-turn band off the origin, on either nappe, written on either
/// carrier: every term against the oracle, about the origin and about a
/// reference off the axis, and its area through every area reader.
#[test]
fn a_band_reads_its_own_moments_on_either_nappe_and_carrier() {
    for scale in [1e-3, 1.0, 1e3] {
        for sigma in [1.0, -1.0] {
            let band = Band {
                apex: Point3::new(1.0 * scale, -2.0 * scale, 3.0 * scale),
                a: 0.6,
                s: (1.0 * scale, 2.5 * scale),
                phi: (0.3, 0.3 + std::f64::consts::FRAC_PI_2),
                sigma,
            };
            let length = 6.0 * scale;
            for (name, carrier) in carriers(&band) {
                let mut topo = Topology::new();
                let face = band.face(&mut topo, carrier);
                for reference in [
                    Point3::new(0.0, 0.0, 0.0),
                    band.apex + Vec3::new(-0.7, 1.9, 0.4) * scale,
                ] {
                    let label =
                        format!("scale {scale}, nappe {sigma}, {name}, about {reference:?}");
                    let want = band.oracle(reference);
                    let got = integrate_face_fixed_about(&topo, face, 8, reference).unwrap();
                    assert_close(
                        &label,
                        &got,
                        &want,
                        length + (reference - band.apex).length(),
                        1e-9,
                    );
                    let (flux_reading, _) =
                        integrate_face_fixed_flux_about(&topo, face, 8, reference).unwrap();
                    assert_close(
                        &format!("{label} (flux)"),
                        &flux_reading,
                        &want,
                        length + (reference - band.apex).length(),
                        1e-9,
                    );
                }
                let area = band.oracle(Point3::new(0.0, 0.0, 0.0)).area;
                for (reader, got) in [
                    (
                        "integrate_face_area",
                        integrate_face_area(&topo, face, 8).unwrap(),
                    ),
                    (
                        "analytic_face_area",
                        analytic_face_area(&topo, face, 8).unwrap(),
                    ),
                ] {
                    assert!(
                        (got.abs() - area).abs() <= 1e-9 * area,
                        "scale {scale}, nappe {sigma}, {name}: {reader} {got} vs {area}"
                    );
                }
                // The exact boundary reduction reads both nappes.
                let got = boundary_face_area(&topo, face).unwrap().unwrap();
                {
                    assert!(
                        (got - area).abs() <= 1e-9 * area,
                        "scale {scale}, nappe {sigma}, {name}: boundary area {got} vs {area}"
                    );
                }
            }
        }
    }
}

/// The face-integral cache on a band below its carrier's apex: a miss and
/// a hit of identical content read the uncached integrator bit for bit
/// (full contribution, volume term, area), a rigid translation of the band
/// is answered from the entry to rounding, and both agree with the oracle.
#[test]
fn the_face_cache_reads_a_band_below_the_apex_as_the_uncached_integrator() {
    use crate::properties::face_cache::FaceIntegralCache;

    let bits = |c: &FaceContribution| terms(c).map(|(_, value)| value.to_bits());
    let band = |shift: f64| Band {
        apex: Point3::new(1.0 + shift, -2.0, 3.0),
        a: 0.6,
        s: (1.0, 2.5),
        phi: (0.3, 0.3 + std::f64::consts::FRAC_PI_2),
        sigma: -1.0,
    };
    let reference = Point3::new(0.5, -1.0, 2.0);
    let (near, far) = (band(0.0), band(1e3));
    let mut topo = Topology::new();
    let near_face = near.face(&mut topo, carriers(&near)[0].1.clone());
    let far_face = far.face(&mut topo, carriers(&far)[0].1.clone());

    let uncached = integrate_face_fixed_about(&topo, near_face, 8, reference).unwrap();
    assert_close("uncached", &uncached, &near.oracle(reference), 8.0, 1e-9);
    let uncached_area = integrate_face_area(&topo, near_face, 8).unwrap();

    let mut cache = FaceIntegralCache::new();
    for pass in ["miss", "hit"] {
        let cached = cache
            .integrate_face_fixed_about(&topo, near_face, 8, reference)
            .unwrap();
        assert_eq!(bits(&cached), bits(&uncached), "{pass}: contribution");
        let term = cache
            .integrate_face_volume_about(&topo, near_face, 8, reference)
            .unwrap();
        let (fresh, _) = integrate_face_fixed_flux_about(&topo, near_face, 8, reference).unwrap();
        assert_eq!(
            term.volume.to_bits(),
            fresh.volume.to_bits(),
            "{pass}: volume term"
        );
        assert_eq!(
            term.area.to_bits(),
            fresh.area.to_bits(),
            "{pass}: volume-term area"
        );
        let area = cache.integrate_face_area(&topo, near_face, 8).unwrap();
        assert_eq!(area.to_bits(), uncached_area.to_bits(), "{pass}: area");
    }
    assert!(cache.stats().hits >= 3, "{:?}", cache.stats());

    // The translated band, about a reference translated with it.
    let moved = reference + Vec3::new(1e3, 0.0, 0.0);
    let translated = cache
        .integrate_face_volume_about(&topo, far_face, 8, moved)
        .unwrap();
    let (fresh, _) = integrate_face_fixed_flux_about(&topo, far_face, 8, moved).unwrap();
    let want = far.oracle(moved);
    for (what, got) in [
        ("translated hit", translated.volume),
        ("fresh", fresh.volume),
    ] {
        assert!(
            (got - want.volume).abs() <= 1e-9 * 8.0_f64.powi(3),
            "{what}: volume {got} vs oracle {}",
            want.volume
        );
    }
    assert!((translated.area - want.area).abs() <= 1e-9 * want.area);
}

/// A face whose boundary runs on both nappes has no single chart that reads
/// it (above the apex the carrier's own, below it the reversed one), so
/// every reading refuses it rather than mirror half of it.
#[test]
fn a_face_on_both_nappes_is_refused() {
    let above = Band {
        apex: Point3::new(1.0, -2.0, 3.0),
        a: 0.6,
        s: (1.0, 2.5),
        phi: (0.3, 1.8),
        sigma: 1.0,
    };
    let below = Band {
        sigma: -1.0,
        ..above
    };
    let mut topo = Topology::new();
    let z = Vec3::new(0.0, 0.0, 1.0);
    let (sin_a, cos_a) = above.a.sin_cos();
    let corners = [
        above.point(above.phi.0, 2.0),
        above.point(above.phi.1, 2.0),
        below.point(below.phi.1, 2.0),
        below.point(below.phi.0, 2.0),
    ];
    let vertices: Vec<_> = corners
        .iter()
        .map(|&p| topo.add_vertex(Vertex::new(p, 1e-9)))
        .collect();
    let mut edges = Vec::new();
    for (from, to, sigma) in [(0, 1, 1.0), (2, 3, -1.0)] {
        let circle = Circle3D::new(above.apex + z * (sigma * 2.0 * sin_a), z, 2.0 * cos_a).unwrap();
        let t0 = circle.project(corners[from]);
        let mut t1 = circle.project(corners[to]);
        if t1 <= t0 {
            t1 += std::f64::consts::TAU;
        }
        let mut edge = Edge::new(vertices[from], vertices[to], EdgeCurve::Circle(circle));
        edge.set_trim(Some((t0, t1)));
        edges.push(topo.add_edge(edge));
    }
    let right = topo.add_edge(Edge::new(vertices[1], vertices[2], EdgeCurve::Line));
    let left = topo.add_edge(Edge::new(vertices[3], vertices[0], EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(edges[0], true),
                OrientedEdge::new(right, true),
                OrientedEdge::new(edges[1], true),
                OrientedEdge::new(left, true),
            ],
            true,
        )
        .unwrap(),
    );
    let carrier = ConicalSurface::new(above.apex, z, above.a).unwrap();
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cone(carrier)));
    let refusal = |error: crate::CheckError| {
        assert!(
            error.to_string().contains("both nappes"),
            "unexpected error: {error}"
        );
    };
    refusal(integrate_face_fixed_about(&topo, face, 8, above.apex).unwrap_err());
    refusal(integrate_face_area(&topo, face, 8).unwrap_err());
    assert!(boundary_face_area(&topo, face).unwrap().is_none());
}
