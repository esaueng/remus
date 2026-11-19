//! A partial revolve of a profile with a vertex on the axis bounds each
//! planar sector cap with a zero-radius rational arc: a point-sized NURBS
//! edge whose three control points coincide. Its moments are rounding noise,
//! so the planar reader's adaptive rule never converged on it about an
//! off-origin axis and the whole cap fell to the 128-chord polygon: a half
//! frustum about an axis through (0.05, 0.03, 0.01) failed closure, and an
//! off-origin half cylinder closed (its two caps' errors cancel) and read
//! 1.35e-4 of its volume low.
//!
//! Oracles: the body is `{(r, φ, z): 0 ≤ φ ≤ θ, 0 ≤ r ≤ ρ(z), 0 ≤ z ≤ h}`
//! about the axis point `A`, with `ρ` linear in `z`, so every volume moment
//! is a closed-form trigonometric factor times `∫ zᵐ ρ(z)ᵏ dz` (degree ≤ 4,
//! integrated exactly by Boole's rule); each sector cap's area is `θρ²/2`
//! and each profile cap's area the trapezoid's. About a tilted axis the same
//! moments are taken in the axis frame and rotated into the world.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI};

use super::*;
use remus_topology::builder::make_polygon_wire;
use remus_topology::face::Face;

#[derive(Clone, Copy, Debug)]
enum Profile {
    /// Bottom radius `2s`, top radius `s`, height `2s`.
    Frustum,
    /// Radius `2s`, height `2s`.
    Cylinder,
}

impl Profile {
    /// `(ρ(0), ρ(h), h)` at scale `s`.
    fn dims(self, s: f64) -> (f64, f64, f64) {
        match self {
            Self::Frustum => (2.0 * s, s, 2.0 * s),
            Self::Cylinder => (2.0 * s, 2.0 * s, 2.0 * s),
        }
    }
}

struct Case {
    profile: Profile,
    angle: f64,
    scale: f64,
    axis: Point3,
    /// Unit axis direction and the profile's radial direction, orthogonal.
    dir: Vec3,
    radial: Vec3,
}

impl Case {
    /// The frame `(radial, dir × radial, dir)` the revolve turns from its
    /// first vector toward its second.
    fn frame(&self) -> [Vec3; 3] {
        [self.radial, self.dir.cross(self.radial), self.dir]
    }
}

fn build(topo: &mut Topology, case: &Case) -> SolidId {
    let (r0, r1, h) = case.profile.dims(case.scale);
    let a = case.axis;
    let [e1, e2, d] = case.frame();
    let at = |x: f64, z: f64| a + e1 * x + d * z;
    let wire = make_polygon_wire(
        topo,
        &[at(0.0, 0.0), at(r0, 0.0), at(r1, h), at(0.0, h)],
        1e-7,
    )
    .unwrap();
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: -e2,
            d: -e2.dot(a - Point3::new(0.0, 0.0, 0.0)),
        },
    ));
    crate::revolve::revolve(topo, face, a, d, case.angle).unwrap()
}

/// `∫₀ʰ f(z) dz` by Boole's rule: exact through degree 5.
fn boole(h: f64, f: impl Fn(f64) -> f64) -> f64 {
    let q = h / 4.0;
    h / 90.0 * (7.0 * f(0.0) + 32.0 * f(q) + 12.0 * f(2.0 * q) + 32.0 * f(3.0 * q) + 7.0 * f(h))
}

/// Volume, centroid and centroidal inertia `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]`
/// (products as `+∫xy`, the `GProps` convention) in closed form.
fn closed_form(case: &Case) -> (f64, Point3, [f64; 6]) {
    let (r0, r1, h) = case.profile.dims(case.scale);
    let t = case.angle;
    let rho = |z: f64| r0 + (r1 - r0) * z / h;
    // ∫∫∫ over the body of r-power and z-power monomials, about A.
    let v = t * boole(h, |z| rho(z).powi(2) / 2.0);
    let i3 = boole(h, |z| rho(z).powi(3) / 3.0);
    let i4 = boole(h, |z| rho(z).powi(4) / 4.0);
    let mx = t.sin() * i3;
    let my = (1.0 - t.cos()) * i3;
    let mz = t * boole(h, |z| z * rho(z).powi(2) / 2.0);
    let qxx = (t / 2.0 + (2.0 * t).sin() / 4.0) * i4;
    let qyy = (t / 2.0 - (2.0 * t).sin() / 4.0) * i4;
    let qzz = t * boole(h, |z| z * z * rho(z).powi(2) / 2.0);
    let qxy = t.sin().powi(2) / 2.0 * i4;
    let iz3 = boole(h, |z| z * rho(z).powi(3) / 3.0);
    let qxz = t.sin() * iz3;
    let qyz = (1.0 - t.cos()) * iz3;
    let (cx, cy, cz) = (mx / v, my / v, mz / v);
    let (xx, yy, zz) = (qxx - v * cx * cx, qyy - v * cy * cy, qzz - v * cz * cz);
    let (pxy, pxz, pyz) = (qxy - v * cx * cy, qxz - v * cx * cz, qyz - v * cy * cz);
    // The inertia matrix in the axis frame, rotated into the world:
    // J_world = R J Rᵀ with R's columns the frame vectors.
    let local = [
        [yy + zz, -pxy, -pxz],
        [-pxy, xx + zz, -pyz],
        [-pxz, -pyz, xx + yy],
    ];
    let frame = case.frame();
    let r = |i: usize, k: usize| [frame[k].x(), frame[k].y(), frame[k].z()][i];
    let world = |i: usize, j: usize| {
        let mut sum = 0.0;
        for k in 0..3 {
            for l in 0..3 {
                sum += r(i, k) * local[k][l] * r(j, l);
            }
        }
        sum
    };
    let inertia = [
        world(0, 0),
        world(1, 1),
        world(2, 2),
        -world(0, 1),
        -world(0, 2),
        -world(1, 2),
    ];
    let [e1, e2, d] = frame;
    (v, case.axis + e1 * cx + e2 * cy + d * cz, inertia)
}

fn cases() -> Vec<Case> {
    let z = (Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0));
    // A tilted axis: no frame vector along a world axis, so rotating its
    // on-axis vertex leaves rounding in every coordinate.
    let tilted = (
        Vec3::new(1.0, 2.0, 2.0) * (1.0 / 3.0),
        Vec3::new(2.0, -1.0, 0.0) * (1.0 / 5.0_f64.sqrt()),
    );
    let mut out = Vec::new();
    for profile in [Profile::Frustum, Profile::Cylinder] {
        for angle in [PI, FRAC_PI_2] {
            for scale in [1e-2, 1.0, 1e2] {
                let off = Point3::new(5.0 * scale, 3.0 * scale, scale);
                for (axis, (dir, radial)) in [
                    (off, z),
                    (Point3::new(0.0, 0.0, 0.0), z),
                    (off, tilted),
                    (Point3::new(0.0, 0.0, 0.0), tilted),
                ] {
                    out.push(Case {
                        profile,
                        angle,
                        scale,
                        axis,
                        dir,
                        radial,
                    });
                }
            }
        }
    }
    out
}

fn label(case: &Case) -> String {
    format!(
        "{:?} θ={} s={} axis={:?} dir={:?}",
        case.profile, case.angle, case.scale, case.axis, case.dir
    )
}

/// Whether `face`'s boundary holds a NURBS edge whose control points all
/// coincide to rounding: the zero-radius arc at the on-axis vertex.
fn has_point_nurbs_edge(topo: &Topology, face: FaceId) -> bool {
    remus_topology::explorer::face_edges(topo, face)
        .unwrap()
        .into_iter()
        .any(|e| {
            let EdgeCurve::NurbsCurve(nc) = topo.edge(e).unwrap().curve() else {
                return false;
            };
            let cps = nc.control_points();
            let scale = cps
                .iter()
                .flat_map(|p| [p.x().abs(), p.y().abs(), p.z().abs()])
                .fold(f64::MIN_POSITIVE, f64::max);
            cps.iter().all(|p| (*p - cps[0]).length() <= 1e-12 * scale)
        })
}

#[test]
fn every_planar_cap_reads_exactly_and_matches_its_closed_form_area() {
    for case in cases() {
        let mut topo = Topology::new();
        let solid = build(&mut topo, &case);
        let (r0, r1, h) = case.profile.dims(case.scale);
        let a = case.axis;
        // Planar faces as (face, Some(level) for a sector cap: 0 bottom, 1 top).
        let mut planar = Vec::new();
        for face in remus_topology::explorer::solid_faces(&topo, solid).unwrap() {
            let FaceSurface::Plane { normal, .. } = *topo.face(face).unwrap().surface() else {
                continue;
            };
            if normal.normalize().unwrap().dot(case.dir).abs() < 0.5 {
                planar.push((face, None));
                continue;
            }
            // A sector cap: on the bottom (z = A.z) or the top level.
            assert!(
                has_point_nurbs_edge(&topo, face),
                "{}: the sector cap carries the on-axis point edge",
                label(&case)
            );
            let first = remus_topology::explorer::face_edges(&topo, face).unwrap()[0];
            let height = (topo
                .vertex(topo.edge(first).unwrap().start())
                .unwrap()
                .point()
                - a)
                .dot(case.dir);
            let level = usize::from(height.abs() > h / 2.0);
            planar.push((face, Some(level)));
        }
        // The revolve splits the turn into equal segments, one sector cap per
        // segment and level, so each of a level's `k` caps spans `θ / k`.
        let per_level = |level: usize| planar.iter().filter(|(_, l)| *l == Some(level)).count();
        assert!(per_level(0) >= 1 && per_level(1) >= 1, "{}", label(&case));
        let mut level_sum = [0.0_f64; 2];
        for (face, sector) in planar.iter().copied() {
            let exact =
                remus_check::properties::face_integrator::planar_face_exact_area(&topo, face)
                    .unwrap()
                    .unwrap_or_else(|| panic!("{}: planar face fell to the fan", label(&case)));
            #[allow(clippy::cast_precision_loss)]
            let expected = match sector {
                Some(level) => {
                    let rho = [r0, r1][level];
                    level_sum[level] += exact;
                    case.angle / per_level(level) as f64 * rho * rho / 2.0
                }
                None => f64::midpoint(r0, r1) * h,
            };
            assert!(
                (exact - expected).abs() <= 1e-13 * expected,
                "{}: planar face {} reads {exact}, closed form {expected}",
                label(&case),
                face.index()
            );
            let area = crate::measure::face_area(&topo, face, 1e-3 * case.scale).unwrap();
            assert!(
                (area - expected).abs() <= 1e-13 * expected,
                "{}: face_area {area}, closed form {expected}",
                label(&case)
            );
        }
        for (sum, rho) in level_sum.into_iter().zip([r0, r1]) {
            let whole = case.angle * rho * rho / 2.0;
            assert!(
                (sum - whole).abs() <= 1e-13 * whole,
                "{}: sector caps at radius {rho} sum to {sum}, closed form {whole}",
                label(&case)
            );
        }
    }
}

#[test]
fn partial_revolves_close_and_measure_their_closed_forms() {
    let order = remus_check::properties::PropertiesOptions::default().gauss_order;
    for case in cases() {
        let mut topo = Topology::new();
        let solid = build(&mut topo, &case);
        let (volume, centroid, inertia) = closed_form(&case);
        let s = case.scale;

        // Closure of the face integrals, as `qualified_shell_volume` reads it.
        let shell = topo.solid(solid).unwrap().outer_shell();
        let faces = topo.shell(shell).unwrap().faces().to_vec();
        let reference = shell_reference(&topo, &faces).unwrap().unwrap();
        let (mut flux, mut area) = (Vec3::new(0.0, 0.0, 0.0), 0.0);
        for &f in &faces {
            let term = super::super::guard_readings::integrate(&topo, f, order, reference).unwrap();
            flux += term.vector_area.unwrap();
            area += term.area;
        }
        let closure = flux.length() / area;
        assert!(
            closure <= 1e-12,
            "{}: shell closes to {closure:.3e} of its area",
            label(&case)
        );
        let (qualified, _) = qualified_shell_volume(&topo, shell, order, false)
            .unwrap()
            .unwrap_or_else(|| panic!("{}: face integrals declined", label(&case)));
        assert!(
            (qualified - volume).abs() <= 1e-12 * volume,
            "{}: face integrals {qualified}, closed form {volume}",
            label(&case)
        );

        let measured = solid_volume(&topo, solid, 1e-3 * s).unwrap();
        assert!(
            (measured - volume).abs() <= 1e-12 * volume,
            "{}: solid_volume {measured}, closed form {volume} (rel {:.3e})",
            label(&case),
            (measured - volume).abs() / volume
        );

        let props = mass_properties(&topo, solid).unwrap();
        assert!(
            (props.mass - volume).abs() <= 1e-12 * volume,
            "{}: mass {} vs {volume}",
            label(&case),
            props.mass
        );
        for (got, want) in [
            (props.center.x(), centroid.x()),
            (props.center.y(), centroid.y()),
            (props.center.z(), centroid.z()),
        ] {
            assert!(
                (got - want).abs() <= 1e-11 * s,
                "{}: centroid {:?} vs {centroid:?}",
                label(&case),
                props.center
            );
        }
        // Inertia scales as s⁵; compare against the tensor's own size.
        let size = inertia.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        for (k, (got, want)) in props.inertia.iter().zip(inertia).enumerate() {
            assert!(
                (got - want).abs() <= 1e-10 * size,
                "{}: inertia[{k}] {got} vs {want}",
                label(&case)
            );
        }
    }
}
