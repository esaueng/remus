//! The r 2 fillet on the seam of an r 6 boss fused into an r 73.5 sphere
//! (esaueng/remus#1561, OpenZCAD K02/W01).
//!
//! The walker's closed rim used to be joined by 20 straight chords: the
//! cylinder contact sat up to 0.11 mm off the boss while its edge recorded
//! 1e-7, the display tessellator refused the hole those chords bound, and its
//! fallback left 286 open edges on the sphere's equator. The seam's boolean
//! edges also run against their parameter (`(1, 0.987)`), which made the
//! convexity frames call the concave seam convex, so the ball rolled inside
//! the material. These tests pin the repaired result: contacts on their
//! supports within the tolerance they record, on the empty side of both
//! supports, a closed display mesh at every setting the issue measured, and
//! the material the fillet adds against an estimate that never reads the
//! B-Rep. A coaxial boss, whose fillet section is the same in every meridian,
//! is the exact control.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::f64::consts::TAU;

use remus_math::compact_sort::CompactSort;
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_cylinder, make_sphere};
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count,
    tessellate_solid_grouped_with_tolerance,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const SPHERE_RADIUS: f64 = 73.5;
const BOSS_RADIUS: f64 = 6.0;
const FILLET_RADIUS: f64 = 2.0;
/// OpenZCAD's display settings for this body (2e-4 × its ~147 mm extent),
/// then the other three rows of the issue's table.
const DISPLAY_SETTINGS: [(f64, f64); 4] =
    [(0.0294, 0.06), (0.1, 0.06), (0.01, 0.06), (0.0294, 0.5)];

#[derive(Clone, Copy, Debug)]
enum Boss {
    /// Axis along +Z through (0, 0, 58), 25.5 mm off the sphere's pole.
    OffAxis,
    /// The off-axis boss tilted 20° about X, then 10° about Y.
    Tilted,
    /// Axis through the sphere's centre: the exact control.
    Coaxial,
}

impl Boss {
    fn sphere_center(self) -> Point3 {
        match self {
            Self::OffAxis | Self::Tilted => Point3::new(25.5, -1.5, 0.0),
            Self::Coaxial => Point3::new(0.0, 0.0, 0.0),
        }
    }

    fn placement(self) -> Mat4 {
        match self {
            Self::OffAxis | Self::Coaxial => Mat4::translation(0.0, 0.0, 58.0),
            Self::Tilted => {
                Mat4::translation(0.0, 0.0, 58.0)
                    * Mat4::rotation_y(10f64.to_radians())
                    * Mat4::rotation_x(20f64.to_radians())
            }
        }
    }

    /// The boss axis direction: `placement` applied to +Z.
    fn axis(self) -> Vec3 {
        match self {
            Self::OffAxis | Self::Coaxial => Vec3::new(0.0, 0.0, 1.0),
            Self::Tilted => {
                let (a, b) = (20f64.to_radians(), 10f64.to_radians());
                Vec3::new(a.cos() * b.sin(), -a.sin(), a.cos() * b.cos())
            }
        }
    }

    /// Distance of `point` from the boss axis.
    fn axis_distance(self, point: Point3) -> f64 {
        let offset = point - Point3::new(0.0, 0.0, 58.0);
        let axis = self.axis();
        (offset - axis * offset.dot(axis)).length()
    }
}

/// The fused body and its four boss/sphere seam edges.
fn sphere_boss(topo: &mut Topology, boss: Boss) -> (SolidId, Vec<EdgeId>) {
    let sphere = make_sphere(topo, SPHERE_RADIUS, 32).unwrap();
    let center = boss.sphere_center();
    transform_solid(
        topo,
        sphere,
        &Mat4::translation(center.x(), center.y(), center.z()),
    )
    .unwrap();
    let cylinder = make_cylinder(topo, BOSS_RADIUS, 64.1).unwrap();
    transform_solid(topo, cylinder, &boss.placement()).unwrap();
    let fused = boolean(topo, BooleanOp::Fuse, sphere, cylinder).unwrap();
    let adjacency = AdjacencyIndex::build(topo, fused).unwrap();
    let mut seams: Vec<EdgeId> = adjacency
        .edge_faces_iter()
        .filter(|(_, faces)| {
            let has = |sphere: bool| {
                faces.iter().any(|&face| {
                    matches!(
                        (sphere, topo.face(face).unwrap().surface()),
                        (true, FaceSurface::Sphere(_)) | (false, FaceSurface::Cylinder(_))
                    )
                })
            };
            faces.len() == 2 && has(true) && has(false)
        })
        .map(|(edge, _)| edge)
        .collect();
    seams.compact_sort_by_key(|edge| edge.index());
    assert!(
        !seams.is_empty(),
        "{boss:?}: the fuse leaves a boss/sphere seam"
    );
    (fused, seams)
}

fn filleted(boss: Boss) -> (Topology, SolidId, SolidId) {
    let mut topo = Topology::new();
    let (fused, seams) = sphere_boss(&mut topo, boss);
    let result = fillet_cascade(&mut topo, fused, &seams, FILLET_RADIUS)
        .unwrap_or_else(|error| panic!("{boss:?}: the seam fillet builds: {error}"));
    assert!(!result.is_partial, "{boss:?}: every seam edge is filleted");
    (topo, fused, result.solid)
}

/// Every curved edge between the blend band and a sphere or cylinder support:
/// `(edge, support surface, recorded tolerance, 2001 samples)`.
fn contact_edges(topo: &Topology, solid: SolidId) -> Vec<(EdgeId, FaceSurface, f64, Vec<Point3>)> {
    let adjacency = AdjacencyIndex::build(topo, solid).unwrap();
    let mut contacts = Vec::new();
    for (edge, faces) in adjacency.edge_faces_iter() {
        let data = topo.edge(edge).unwrap();
        let surfaces: Vec<&FaceSurface> = faces
            .iter()
            .map(|&face| topo.face(face).unwrap().surface())
            .collect();
        if !surfaces
            .iter()
            .any(|s| matches!(s, FaceSurface::Nurbs(_) | FaceSurface::Torus(_)))
        {
            continue;
        }
        let Some(support) = surfaces
            .iter()
            .find(|s| matches!(s, FaceSurface::Sphere(_) | FaceSurface::Cylinder(_)))
        else {
            continue;
        };
        let start = topo.vertex(data.start()).unwrap().point();
        let end = topo.vertex(data.end()).unwrap().point();
        let (t0, t1) = data.strict_domain().unwrap();
        let samples = (0..=2000)
            .map(|i| {
                let t = t0 + (t1 - t0) * f64::from(i) / 2000.0;
                data.curve().evaluate_with_endpoints(t, start, end)
            })
            .collect();
        let tolerance = data.tolerance().unwrap_or(1e-7);
        contacts.push((edge, (*support).clone(), tolerance, samples));
    }
    contacts.compact_sort_by_key(|(edge, ..)| edge.index());
    contacts
}

/// Largest closest-point distance of `samples` from `support`.
fn support_deviation(support: &FaceSurface, samples: &[Point3]) -> f64 {
    samples
        .iter()
        .map(|&point| {
            let (u, v) = support.project_point(point).unwrap();
            (support.evaluate(u, v).unwrap() - point).length()
        })
        .fold(0.0_f64, f64::max)
}

#[test]
fn seam_fillet_contacts_lie_on_their_supports_within_recorded_tolerance() {
    for boss in [Boss::OffAxis, Boss::Tilted, Boss::Coaxial] {
        let (topo, _, solid) = filleted(boss);
        let contacts = contact_edges(&topo, solid);
        let mut sides = BTreeMap::new();
        for (edge, support, tolerance, samples) in &contacts {
            let deviation = support_deviation(support, samples);
            // Before: 9.2e-2 (0.110 tilted) off the cylinder, 3.8e-3 off the
            // sphere, both recording 1e-7.
            assert!(
                deviation <= *tolerance,
                "{boss:?}: contact {edge:?} lies {deviation:e} off its {} support, \
                 beyond the {tolerance:e} it records",
                support.type_tag()
            );
            // The ball rolls in the empty corner: the boss contact is outside
            // the sphere and the sphere contact outside the boss. Before the
            // convexity frames honoured the seam's reversed domain, both
            // contacts sat inside the material (sphere contact 3.4 mm from
            // the axis, boss contact 2.6 mm inside the sphere).
            let outside = samples
                .iter()
                .map(|&point| match support {
                    FaceSurface::Cylinder(_) => {
                        (point - boss.sphere_center()).length() - SPHERE_RADIUS
                    }
                    _ => boss.axis_distance(point) - BOSS_RADIUS,
                })
                .fold(f64::INFINITY, f64::min);
            assert!(
                outside > FILLET_RADIUS * 0.25,
                "{boss:?}: contact {edge:?} on the {} reaches {outside:.4} into the other \
                 support's material",
                support.type_tag()
            );
            *sides.entry(support.type_tag()).or_insert(0) += 1;
        }
        assert!(
            sides.get("cylinder").is_some_and(|&n| n > 0)
                && sides.get("sphere").is_some_and(|&n| n > 0),
            "{boss:?}: the band meets both supports along contact edges: {sides:?}"
        );
    }
}

/// `(open, non-manifold, inconsistently wound)` edges of `mesh`, unwelded.
fn closure(mesh: &TriangleMesh) -> (usize, usize, usize) {
    let mut uses: BTreeMap<(u32, u32), (usize, i32)> = BTreeMap::new();
    for triangle in mesh.indices.chunks_exact(3) {
        for k in 0..3 {
            let (a, b) = (triangle[k], triangle[(k + 1) % 3]);
            let entry = uses.entry((a.min(b), a.max(b))).or_default();
            entry.0 += 1;
            entry.1 += if a < b { 1 } else { -1 };
        }
    }
    let flipped = uses
        .values()
        .filter(|&&(count, balance)| count == 2 && balance != 0)
        .count();
    (
        boundary_edge_count(mesh),
        non_manifold_edge_count(mesh),
        flipped,
    )
}

#[test]
fn seam_fillet_display_mesh_is_closed_at_every_measured_setting() {
    for boss in [Boss::OffAxis, Boss::Tilted, Boss::Coaxial] {
        let (topo, _, solid) = filleted(boss);
        for (deflection, angular) in DISPLAY_SETTINGS {
            let (mesh, _) =
                tessellate_solid_grouped_with_tolerance(&topo, solid, deflection, angular).unwrap();
            // Before (off-axis): 286 / 277 / 1 439 / 409 open edges, and 962
            // and 115 inconsistently wound ones at 0.01 / 0.06 and 0.0294 / 0.5.
            assert_eq!(
                closure(&mesh),
                (0, 0, 0),
                "{boss:?} at {deflection} / {angular}: (open, non-manifold, flipped) edges"
            );
        }
    }
}

/// Signed volume of a closed mesh.
fn mesh_volume(mesh: &TriangleMesh) -> f64 {
    let apex = mesh.positions[0];
    mesh.indices
        .chunks_exact(3)
        .map(|triangle| {
            let [a, b, c] = [0, 1, 2].map(|k| mesh.positions[triangle[k] as usize] - apex);
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

/// Material the fillet adds, read off two closed meshes at one setting so the
/// tessellation error of every face the fillet left alone cancels. The
/// angular limit matters as much as the deflection here: at 0.06 rad the
/// sphere's rings shift with the hole and the difference reads 5% high.
fn added_material(topo: &Topology, input: SolidId, result: SolidId) -> f64 {
    let volume = |solid| {
        let (mesh, _) = tessellate_solid_grouped_with_tolerance(topo, solid, 0.01, 0.02).unwrap();
        assert_eq!(closure(&mesh), (0, 0, 0));
        mesh_volume(&mesh)
    };
    volume(result) - volume(input)
}

/// The sphere's centre in the boss frame (axis +Z from the boss's base).
fn sphere_center_in_boss_frame(boss: Boss) -> (f64, f64, f64) {
    let center = boss.sphere_center();
    let (x, y, z) = (center.x(), center.y(), center.z() - 58.0);
    match boss {
        Boss::OffAxis | Boss::Coaxial => (x, y, z),
        Boss::Tilted => {
            let (a, b) = (20f64.to_radians(), 10f64.to_radians());
            let (x, z) = (x * b.cos() - z * b.sin(), x * b.sin() + z * b.cos());
            let (y, z) = (y * a.cos() + z * a.sin(), -y * a.sin() + z * a.cos());
            (x, y, z)
        }
    }
}

/// `∫ r dA` over one meridian section of the fillet: the half-plane through
/// the boss axis cuts the sphere in a circle of radius `circle_radius` centred
/// at `(a, b)` (radial, axial), and a ball of radius ρ rolling in that plane
/// touches the wall `r = 6` and the circle. The section is the region outside
/// the wall, the circle and the ball, below the wall contact and inside the
/// circle contact, integrated by the midpoint rule on an `n²` grid.
fn section_moment(a: f64, b: f64, circle_radius: f64, n: u32) -> f64 {
    let rho = FILLET_RADIUS;
    let center_r = BOSS_RADIUS + rho;
    let center_z = b + ((circle_radius + rho).powi(2) - (center_r - a).powi(2)).sqrt();
    let pull = rho / (circle_radius + rho);
    let contact_r = (a - center_r).mul_add(pull, center_r);
    let contact_z = (b - center_z).mul_add(pull, center_z);
    let seam_z = b + (circle_radius.powi(2) - (BOSS_RADIUS - a).powi(2)).sqrt();
    let low = seam_z.min(contact_z);
    let (dr, dz) = (
        (contact_r - BOSS_RADIUS) / f64::from(n),
        (center_z - low) / f64::from(n),
    );
    let mut moment = 0.0;
    for i in 0..n {
        let r = (f64::from(i) + 0.5).mul_add(dr, BOSS_RADIUS);
        for j in 0..n {
            let z = (f64::from(j) + 0.5).mul_add(dz, low);
            if (r - a).hypot(z - b) > circle_radius && (r - center_r).hypot(z - center_z) > rho {
                moment += r;
            }
        }
    }
    moment * dr * dz
}

/// Fillet material from meridian sections alone, without the B-Rep:
/// `∫ dφ ∫ r dA(φ)` over 180 azimuths. Each section is exact for a ball that
/// rolls in its meridian plane; the real ball leans out of that plane where
/// the seam climbs the sphere, which this ignores. For the coaxial boss every
/// section is the same and the ball does roll in the plane, so there the
/// estimate is the exact Pappus volume up to the grid.
fn meridian_estimate(boss: Boss) -> f64 {
    let (cx, cy, cz) = sphere_center_in_boss_frame(boss);
    let steps = 180_u32;
    let mut volume = 0.0;
    for k in 0..steps {
        let (sin, cos) = (TAU * f64::from(k) / f64::from(steps)).sin_cos();
        let radial = cx.mul_add(cos, cy * sin);
        let off_plane = (-cx).mul_add(sin, cy * cos);
        let circle_radius = (SPHERE_RADIUS.powi(2) - off_plane.powi(2)).sqrt();
        volume += section_moment(radial, cz, circle_radius, 300) * TAU / f64::from(steps);
    }
    volume
}

#[test]
fn seam_fillet_adds_the_independently_estimated_material() {
    for (boss, bound) in [
        (Boss::Coaxial, 0.01),
        (Boss::OffAxis, 0.05),
        (Boss::Tilted, 0.05),
    ] {
        let (topo, input, solid) = filleted(boss);
        let added = added_material(&topo, input, solid);
        let estimate = meridian_estimate(boss);
        // Off-axis and tilted, the estimate is an approximation (see
        // `meridian_estimate`): measured +32.03 against 31.51 and +40.61
        // against 39.33. Coaxial it is exact (27.33, measured +27.45), and
        // the bound is the meshes' chord error on the band and its rims.
        assert!(
            (added - estimate).abs() <= bound * estimate,
            "{boss:?}: the fillet adds {added:+.3} mm³, the meridian estimate {estimate:.3}"
        );
    }
}

/// The closed walk's contacts are smooth: each contact edge is a single C¹
/// curve, not a chord polyline.
#[test]
fn closed_walk_contacts_are_cubic_not_chords() {
    for boss in [Boss::OffAxis, Boss::Tilted] {
        let (topo, _, solid) = filleted(boss);
        for (edge, ..) in contact_edges(&topo, solid) {
            let EdgeCurve::NurbsCurve(curve) = topo.edge(edge).unwrap().curve() else {
                panic!("{boss:?}: walked contact {edge:?} is a NURBS curve");
            };
            assert_eq!(curve.degree(), 3, "{boss:?}: contact {edge:?}");
        }
    }
}

/// The coaxial control is exact: its band is a torus and its contacts are
/// circles on both supports, so it never reaches the resampled closed walk.
#[test]
fn coaxial_seam_fillet_is_an_exact_torus() {
    let (topo, _, solid) = filleted(Boss::Coaxial);
    let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
    let kinds: Vec<&str> = faces
        .iter()
        .map(|&face| topo.face(face).unwrap().surface().type_tag())
        .collect();
    assert!(kinds.contains(&"torus"), "coaxial band surfaces: {kinds:?}");
    assert!(
        !kinds.contains(&"nurbs"),
        "coaxial band surfaces: {kinds:?}"
    );
    for (edge, support, tolerance, samples) in contact_edges(&topo, solid) {
        assert!(
            matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Circle(_)),
            "coaxial contact {edge:?} is a circle"
        );
        let deviation = support_deviation(&support, &samples);
        assert!(deviation <= 1e-9, "coaxial contact {edge:?}: {deviation:e}");
        assert!(
            tolerance <= 1e-7,
            "coaxial contact {edge:?} records {tolerance:e}"
        );
    }
}
