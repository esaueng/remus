//! Resizing a cylindrical boss whose seam with a sphere carries a
//! rolling-ball fillet: the band is removed to the exact sharp seam, the wall
//! is resized, and the same radius is rolled back onto the new seam.
//!
//! The consumer body: sphere r 73.5 at (25.5, -1.5, 0) fused with a boss r 6,
//! h 64.1 from (0, 0, 58) along +Z (and a variant tilted 20° about X and 10°
//! about Y), then an r 2 fillet on the boss/sphere seam. Off axis the band is
//! a sampled NURBS canal surface (approximate); coaxial it is an exact torus.
//!
//! Volume oracles are independent of the kernel: the closed-form sphere
//! volume, the boss sleeve `∫∫ (H − t_sphere) dA` by composite Simpson in ρ
//! and the trapezoid rule in θ, and the rolling-ball band volume integrated
//! directly from the closed-form ball-centre spine (the intersection of the
//! wall offset by r and the sphere offset by r): for every vertical line
//! beside the wall the fill runs from the sphere up to the lowest ball that
//! crosses it. The band oracle is numerical (≈1e-3 relative); each comparison
//! states its bound.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines
)]

use std::f64::consts::{PI, TAU};

use remus_math::compact_sort::CompactSort;
use remus_math::mat::Mat4;
use remus_math::vec::Vec3;
use remus_operations::OperationsError;
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_cylinder, make_sphere};
use remus_operations::push_pull::resize_cylindrical_face;
use remus_operations::resize_blend::{
    ReblendPolicy, ReblendQuality, blend_region, resize_blend, resize_cylindrical_face_reblended,
};
use remus_operations::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::{edge_to_face_map, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const SPHERE_RADIUS: f64 = 73.5;
const HEIGHT: f64 = 64.1;
const WALL: f64 = 6.0;
const BALL: f64 = 2.0;
const BASE: (f64, f64, f64) = (0.0, 0.0, 58.0);

/// Placement of the consumer body.
#[derive(Clone, Copy)]
struct Case {
    sphere_center: (f64, f64, f64),
    /// Rotation of the boss about X, then about Y, in degrees.
    tilt: (f64, f64),
}

const OFF_AXIS: Case = Case {
    sphere_center: (25.5, -1.5, 0.0),
    tilt: (0.0, 0.0),
};
const TILTED: Case = Case {
    sphere_center: (25.5, -1.5, 0.0),
    tilt: (20.0, 10.0),
};
const COAXIAL: Case = Case {
    sphere_center: (0.0, 0.0, 0.0),
    tilt: (0.0, 0.0),
};

impl Case {
    /// Boss axis from the construction rotations (independent of the kernel).
    fn axis(self) -> Vec3 {
        let (sx, cx) = self.tilt.0.to_radians().sin_cos();
        let (sy, cy) = self.tilt.1.to_radians().sin_cos();
        // R_y · R_x · ẑ
        let after_x = Vec3::new(0.0, -sx, cx);
        Vec3::new(
            after_x.x() * cy + after_x.z() * sy,
            after_x.y(),
            -after_x.x() * sy + after_x.z() * cy,
        )
    }

    /// Sphere centre `(c_x, c_y, c_t)` in a wall frame with origin at the
    /// boss base.
    fn frame(self) -> Frame {
        let axis = self.axis();
        let helper = if axis.x().abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let x = (helper - axis * helper.dot(axis)).normalize().unwrap();
        let y = axis.cross(x);
        let d = Vec3::new(
            self.sphere_center.0 - BASE.0,
            self.sphere_center.1 - BASE.1,
            self.sphere_center.2 - BASE.2,
        );
        Frame {
            cx: d.dot(x),
            cy: d.dot(y),
            ct: d.dot(axis),
        }
    }
}

#[derive(Clone, Copy)]
struct Frame {
    cx: f64,
    cy: f64,
    ct: f64,
}

impl Frame {
    /// Upper sphere branch above wall-frame `(ρ, θ)`, sphere radius `radius`.
    fn station(self, radius: f64, rho: f64, theta: f64) -> f64 {
        let dx = rho * theta.cos() - self.cx;
        let dy = rho * theta.sin() - self.cy;
        self.ct + (radius * radius - dx * dx - dy * dy).sqrt()
    }

    /// Closed-form sharp body volume with wall radius `wall`.
    fn sharp_volume(self, wall: f64) -> f64 {
        let sphere = 4.0 / 3.0 * PI * SPHERE_RADIUS.powi(3);
        sphere
            + integrate(0.0, wall, |rho, theta| {
                HEIGHT - self.station(SPHERE_RADIUS, rho, theta)
            })
    }

    /// Rolling-ball fill volume of a ball `ball` in the corner of a boss of
    /// radius `wall` on the sphere.
    fn band_volume(self, wall: f64, ball: f64) -> f64 {
        let spine = wall + ball;
        let offset = SPHERE_RADIUS + ball;
        let centre = |theta: f64| self.station(offset, spine, theta);
        // The sphere contact of the ball at spine azimuth θ' lies on the ray
        // from the sphere centre through the ball centre; tabulate its wall
        // radius by its own azimuth. The fill lies between the wall and it.
        let samples = 4096;
        let mut contact: Vec<(f64, f64)> = (0..samples)
            .map(|k| {
                let theta = TAU * f64::from(k) / f64::from(samples);
                let (sin, cos) = theta.sin_cos();
                let c = [spine * cos, spine * sin, centre(theta)];
                let s = [self.cx, self.cy, self.ct];
                let q: Vec<f64> = (0..3)
                    .map(|i| s[i] + (c[i] - s[i]) * SPHERE_RADIUS / offset)
                    .collect();
                (q[1].atan2(q[0]).rem_euclid(TAU), q[0].hypot(q[1]))
            })
            .collect();
        contact.compact_sort_by(|a, b| a.0.total_cmp(&b.0));
        let contact_radius = |theta: f64| {
            let theta = theta.rem_euclid(TAU);
            let n = contact.len();
            let (a, b) = match contact.partition_point(|entry| entry.0 < theta) {
                0 => ((contact[n - 1].0 - TAU, contact[n - 1].1), contact[0]),
                upper if upper == n => (contact[n - 1], (contact[0].0 + TAU, contact[0].1)),
                upper => (contact[upper - 1], contact[upper]),
            };
            a.1 + (b.1 - a.1) * (theta - a.0) / (b.0 - a.0)
        };
        integrate(wall, wall + 2.0 * ball, |rho, theta| {
            if rho > contact_radius(theta) {
                return 0.0;
            }
            // Azimuth window in which the ball centred on the spine crosses
            // the vertical line at (ρ, θ).
            let cos_max = (rho * rho + spine * spine - ball * ball) / (2.0 * rho * spine);
            if cos_max >= 1.0 {
                return 0.0;
            }
            let window = cos_max.max(-1.0).acos();
            let low = |delta: f64| {
                let d2 = rho * rho + spine * spine - 2.0 * rho * spine * delta.cos();
                centre(theta + delta) - (ball * ball - d2).max(0.0).sqrt()
            };
            let steps = 64;
            let mut best = (f64::INFINITY, 0.0);
            for index in 0..=steps {
                let delta = -window + 2.0 * window * f64::from(index) / f64::from(steps);
                let value = low(delta);
                if value < best.0 {
                    best = (value, delta);
                }
            }
            let step = 2.0 * window / f64::from(steps);
            let (mut lo, mut hi) = (best.1 - step, best.1 + step);
            for _ in 0..50 {
                let a = hi - 0.618_033_988_75 * (hi - lo);
                let b = lo + 0.618_033_988_75 * (hi - lo);
                if low(a) <= low(b) {
                    hi = b;
                } else {
                    lo = a;
                }
            }
            let top = low(0.5 * (lo + hi)).min(best.0);
            (top - self.station(SPHERE_RADIUS, rho, theta)).max(0.0)
        })
    }
}

/// `∫θ∫ρ f ρ dρ dθ` over `ρ ∈ [a, b]`: composite Simpson in ρ (400 panels),
/// periodic trapezoid in θ (720 steps).
fn integrate(a: f64, b: f64, f: impl Fn(f64, f64) -> f64) -> f64 {
    let panels = 400;
    let steps = 720;
    let h = (b - a) / f64::from(panels);
    let mut total = 0.0;
    for k in 0..steps {
        let theta = TAU * f64::from(k) / f64::from(steps);
        let mut radial = 0.0;
        for i in 0..=panels {
            let rho = a + h * f64::from(i);
            let weight = if i == 0 || i == panels {
                1.0
            } else if i % 2 == 1 {
                4.0
            } else {
                2.0
            };
            radial += weight * f(rho, theta) * rho;
        }
        total += radial * h / 3.0;
    }
    total * TAU / f64::from(steps)
}

fn faces_of(topo: &Topology, solid: SolidId, tag: &str) -> Vec<FaceId> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| topo.face(face).unwrap().surface().type_tag() == tag)
        .collect()
}

fn only(faces: &[FaceId], label: &str) -> FaceId {
    assert_eq!(faces.len(), 1, "{label}: {faces:?}");
    faces[0]
}

/// The sharp boss body, its wall, and the filleted body with its band.
fn build(topo: &mut Topology, case: Case) -> (SolidId, SolidId, FaceId) {
    let sphere = make_sphere(topo, SPHERE_RADIUS, 16).unwrap();
    transform_solid(
        topo,
        sphere,
        &Mat4::translation(
            case.sphere_center.0,
            case.sphere_center.1,
            case.sphere_center.2,
        ),
    )
    .unwrap();
    let boss = make_cylinder(topo, WALL, HEIGHT).unwrap();
    transform_solid(topo, boss, &Mat4::rotation_x(case.tilt.0.to_radians())).unwrap();
    transform_solid(topo, boss, &Mat4::rotation_y(case.tilt.1.to_radians())).unwrap();
    transform_solid(topo, boss, &Mat4::translation(BASE.0, BASE.1, BASE.2)).unwrap();
    let sharp = boolean(topo, BooleanOp::Fuse, sphere, boss).unwrap();
    let wall = only(&faces_of(topo, sharp, "cylinder"), "sharp wall");
    let edge_faces = edge_to_face_map(topo, sharp).unwrap();
    let seam: Vec<_> = edge_faces
        .iter()
        .filter(|(_, faces)| {
            faces.contains(&wall)
                && faces
                    .iter()
                    .any(|face| topo.face(*face).unwrap().surface().type_tag() == "sphere")
        })
        .map(|(&edge, _)| topo.edge_id_from_index(edge).unwrap())
        .collect();
    assert!(!seam.is_empty(), "boss/sphere seam");
    let filleted = fillet_cascade(topo, sharp, &seam, BALL).unwrap();
    assert!(!filleted.is_partial);
    let wall = only(&faces_of(topo, filleted.solid, "cylinder"), "filleted wall");
    (sharp, filleted.solid, wall)
}

/// Divergence-theorem volume of a closed triangle mesh.
fn mesh_volume(mesh: &remus_operations::tessellate::TriangleMesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|tri| {
            let [a, b, c] = [0, 1, 2].map(|k| {
                let p = mesh.positions[tri[k] as usize];
                Vec3::new(p.x(), p.y(), p.z())
            });
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

/// Validation always; a watertight display mesh only for an exact body. A
/// sampled walking band's display mesh does not share its neighbours'
/// boundary samples (the known display-mesh gap of approximate bands), so
/// its volume is checked through the exact face integrals instead.
fn assert_valid(topo: &Topology, solid: SolidId, label: &str, watertight: bool) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {report:?}");
    if watertight {
        let mesh = tessellate_solid_with_tolerance(topo, solid, 0.05, 0.1).unwrap();
        assert!(
            is_watertight(&mesh),
            "{label}: tessellation is not watertight"
        );
    }
}

/// Live-entity census used to prove a refusal restored the arena.
fn census(topo: &Topology) -> [usize; 5] {
    [
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
    ]
}

/// Check the oracles for one filleted body before any edit: the sharp body
/// and the filleted body against the closed form and the band integral.
fn check_source(topo: &Topology, case: Case, sharp: SolidId, filleted: SolidId) -> f64 {
    let frame = case.frame();
    let sharp_expected = frame.sharp_volume(WALL);
    let sharp_actual = solid_volume(topo, sharp, 0.01).unwrap();
    assert!(
        (sharp_actual - sharp_expected).abs() <= 1e-5 * sharp_expected,
        "sharp volume {sharp_actual} vs closed form {sharp_expected}"
    );
    let band = frame.band_volume(WALL, BALL);
    let filleted_actual = solid_volume(topo, filleted, 0.01).unwrap();
    let measured_band = filleted_actual - sharp_actual;
    // 3 %: the walking band's contacts are polylines through its sections,
    // which biases the band high by O(step²) (38.0 / 32.6 / 32.4 mm³ at 5 /
    // 1 / 0.4 % steps against this integral's 31.9 mm³).
    assert!(
        (measured_band - band).abs() <= 0.03 * band,
        "source band adds {measured_band} mm³; the rolling-ball integral is {band} mm³"
    );
    band
}

fn reblend_case(case: Case, new_wall: f64, expect_exact: bool) {
    let mut topo = Topology::new();
    let (sharp, filleted, wall) = build(&mut topo, case);
    check_source(&topo, case, sharp, filleted);
    let source_counts = solid_entity_counts(&topo, filleted).unwrap();
    let band_tag = if expect_exact { "torus" } else { "nurbs" };
    let source_band = only(&faces_of(&topo, filleted, band_tag), "source band");

    let result = resize_cylindrical_face_reblended(
        &mut topo,
        filleted,
        wall,
        new_wall,
        ReblendPolicy::AllowApproximate,
    )
    .unwrap()
    .expect("seam band recognized");
    assert!(
        (result.blend_radius - BALL).abs() <= 1e-6,
        "{}",
        result.blend_radius
    );
    assert_eq!(result.source_band, vec![source_band]);
    match (&result.quality, expect_exact) {
        (ReblendQuality::Exact, true) => {}
        (
            ReblendQuality::Approximate {
                faces,
                source_deviation,
                result_deviation,
                source_contact_angle,
                result_contact_angle,
            },
            false,
        ) => {
            assert!(
                *result_contact_angle <= 2.0 * source_contact_angle.max(1e-9),
                "contact angle {result_contact_angle} vs source {source_contact_angle}"
            );
            assert_eq!(faces.len(), 1, "one approximated band face");
            assert_eq!(
                topo.face(faces[0]).unwrap().surface().type_tag(),
                "nurbs",
                "approximated face is the rebuilt band"
            );
            assert!(*source_deviation <= 0.02 * BALL, "{source_deviation}");
            assert!(*result_deviation <= 0.02 * BALL, "{result_deviation}");
        }
        (quality, exact) => panic!("quality {quality:?}, expected exact = {exact}"),
    }
    let solid = result.solid;
    assert_valid(&topo, solid, "re-blended body", expect_exact);
    assert_eq!(solid_entity_counts(&topo, solid).unwrap(), source_counts);
    assert_eq!(
        faces_of(&topo, solid, band_tag).len(),
        1,
        "one rebuilt band"
    );
    let new_wall_face = only(&faces_of(&topo, solid, "cylinder"), "resized wall");
    let FaceSurface::Cylinder(cylinder) = topo.face(new_wall_face).unwrap().surface() else {
        unreachable!()
    };
    assert!((cylinder.radius() - new_wall).abs() <= 1e-9);

    let frame = case.frame();
    let expected = frame.sharp_volume(new_wall) + frame.band_volume(new_wall, BALL);
    let band = frame.band_volume(new_wall, BALL);
    let actual = solid_volume(&topo, solid, 0.01).unwrap();
    // Sharp part: closed form to 1e-5 relative; band part: the rolling-ball
    // integral to 3 % of the band volume (the band itself is a sampled fit,
    // see `check_source`).
    assert!(
        (actual - expected).abs() <= 1e-5 * expected + 0.03 * band,
        "re-blended volume {actual} vs oracle {expected} (band {band})"
    );
    if expect_exact {
        let mesh = tessellate_solid_with_tolerance(&topo, solid, 0.01, 0.05).unwrap();
        let mesh_volume = mesh_volume(&mesh);
        assert!(
            (mesh_volume - expected).abs() <= 1e-4 * expected + 0.05 * band,
            "tessellated volume {mesh_volume} vs oracle {expected}"
        );
    }
}

#[test]
fn off_axis_boss_grows_with_its_approximate_seam_fillet() {
    reblend_case(OFF_AXIS, 7.0, false);
}

#[test]
fn off_axis_boss_shrinks_with_its_approximate_seam_fillet() {
    reblend_case(OFF_AXIS, 5.0, false);
}

#[test]
fn tilted_boss_resizes_with_its_approximate_seam_fillet() {
    reblend_case(TILTED, 7.0, false);
    reblend_case(TILTED, 5.0, false);
}

#[test]
fn coaxial_boss_resizes_with_its_exact_torus_fillet() {
    reblend_case(COAXIAL, 7.0, true);
    reblend_case(COAXIAL, 5.0, true);
}

#[test]
fn exact_only_refuses_the_approximate_rebuild_and_restores_the_arena() {
    let mut topo = Topology::new();
    let (_, filleted, wall) = build(&mut topo, OFF_AXIS);
    let before = census(&topo);
    let volume = solid_volume(&topo, filleted, 0.01).unwrap();
    let error =
        resize_cylindrical_face_reblended(&mut topo, filleted, wall, 7.0, ReblendPolicy::ExactOnly)
            .unwrap_err();
    assert!(
        matches!(error, OperationsError::ExactOnlyUnattainable),
        "{error:?}"
    );
    assert_eq!(census(&topo), before);
    assert!((solid_volume(&topo, filleted, 0.01).unwrap() - volume).abs() <= 1e-9 * volume);

    // The exact-only legacy entry keeps its established typed refusal.
    let legacy = resize_cylindrical_face(&mut topo, filleted, wall, 7.0).unwrap_err();
    assert!(
        format!("{legacy:?}").contains("UnsupportedNeighbor"),
        "{legacy:?}"
    );
    assert_eq!(census(&topo), before);
}

#[test]
fn a_failing_rebuild_rolls_back_every_step() {
    let mut topo = Topology::new();
    let (_, filleted, wall) = build(&mut topo, OFF_AXIS);
    let before = census(&topo);
    // A wall this wide reaches past the sphere's silhouette: the sharp resize
    // refuses after the band was already removed inside the transaction.
    let error = resize_cylindrical_face_reblended(
        &mut topo,
        filleted,
        wall,
        60.0,
        ReblendPolicy::AllowApproximate,
    )
    .unwrap_err();
    assert!(
        !matches!(error, OperationsError::ExactOnlyUnattainable),
        "{error:?}"
    );
    assert_eq!(census(&topo), before);
    assert_valid(&topo, filleted, "source after refusal", false);
}

#[test]
fn coaxial_torus_band_is_removed_and_resized_by_resize_blend() {
    let mut topo = Topology::new();
    let (sharp, filleted, _) = build(&mut topo, COAXIAL);
    check_source(&topo, COAXIAL, sharp, filleted);
    let torus = only(&faces_of(&topo, filleted, "torus"), "torus band");
    let region = blend_region(&topo, filleted, torus).unwrap();
    assert!((region.radius - BALL).abs() <= 1e-12);
    let frame = COAXIAL.frame();

    let removed = resize_blend(&mut topo, filleted, torus, BALL, 0.0).unwrap();
    assert_valid(&topo, removed.solid, "sharp coaxial seam", true);
    assert_eq!(
        solid_entity_counts(&topo, removed.solid).unwrap(),
        solid_entity_counts(&topo, sharp).unwrap()
    );
    assert!(faces_of(&topo, removed.solid, "torus").is_empty());
    let volume = solid_volume(&topo, removed.solid, 0.01).unwrap();
    let expected = frame.sharp_volume(WALL);
    assert!(
        (volume - expected).abs() <= 1e-5 * expected,
        "{volume} vs {expected}"
    );

    let grown = resize_blend(&mut topo, filleted, torus, BALL, 3.0).unwrap();
    assert_valid(&topo, grown.solid, "r 3 coaxial seam", true);
    let band = only(&faces_of(&topo, grown.solid, "torus"), "rebuilt torus");
    let FaceSurface::Torus(surface) = topo.face(band).unwrap().surface() else {
        unreachable!()
    };
    assert!((surface.minor_radius() - 3.0).abs() <= 1e-12);
    let volume = solid_volume(&topo, grown.solid, 0.01).unwrap();
    let expected = frame.sharp_volume(WALL) + frame.band_volume(WALL, 3.0);
    let band_volume = frame.band_volume(WALL, 3.0);
    assert!(
        (volume - expected).abs() <= 1e-5 * expected + 0.01 * band_volume,
        "{volume} vs {expected}"
    );

    // The exact-only legacy resize carries the exact band too.
    let wall = only(&faces_of(&topo, filleted, "cylinder"), "wall");
    let resized = resize_cylindrical_face(&mut topo, filleted, wall, 7.0).unwrap();
    assert_valid(&topo, resized, "legacy coaxial resize", true);
    assert_eq!(faces_of(&topo, resized, "torus").len(), 1);
    assert!(faces_of(&topo, resized, "nurbs").is_empty());
}
