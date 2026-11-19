//! A cone wall written on its carrier's lower nappe (`v < 0`) measures the
//! same body as the same wall on the upper nappe.
//!
//! `ConicalSurface` is a double cone: at `v < 0` its chart puts the point of
//! longitude `u` half a turn round the axis from where `project_point` says,
//! so a wall below its carrier's apex read its own mirror image across the
//! axis: the right area, mirrored volume terms and moments. A full-turn wall
//! is its own mirror image, so the body here is half a frustum, whose
//! centroid sits off the axis.
//!
//! Oracles: the half frustum `y ≥ 0` of radius `ρ(z) = R − (R − r) z / h`
//! sliced into half discs (area `π ρ² / 2`, centroid `4 ρ / 3π` off the
//! axis, polar moment `π ρ⁴ / 4`), integrated in closed form; the whole
//! inertia tensor and the bounding box against the same body on its
//! upper-nappe carrier.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::vec::{Point3, Vec3};
use remus_operations::measure;
use remus_operations::revolve::revolve;
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;

/// Half of the frustum of bottom radius `2s`, top radius `s`, height `2s`
/// (the `y ≥ 0` side: its trapezoid profile revolved half a turn about
/// `ẑ`), and its cone walls. With `lower` the walls are written on the
/// reversed-axis carrier, where they lie below the apex.
fn half_frustum(s: f64, lower: bool) -> (Topology, SolidId, Vec<FaceId>) {
    let mut topo = Topology::new();
    let profile = make_polygon_wire(
        &mut topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0 * s, 0.0, 0.0),
            Point3::new(s, 0.0, 2.0 * s),
            Point3::new(0.0, 0.0, 2.0 * s),
        ],
        1e-7,
    )
    .unwrap();
    let face = topo.add_face(Face::new(
        profile,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, -1.0, 0.0),
            d: 0.0,
        },
    ));
    let half = revolve(
        &mut topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        PI,
    )
    .unwrap();
    let walls: Vec<_> = solid_faces(&topo, half)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cone(_)))
        .collect();
    assert!(!walls.is_empty(), "a cone wall");
    for &wall in &walls {
        let FaceSurface::Cone(cone) = topo.face(wall).unwrap().surface().clone() else {
            unreachable!()
        };
        let carrier = if lower {
            cone.with_reversed_axis()
        } else {
            cone
        };
        let wire = topo.wire(topo.face(wall).unwrap().outer_wire()).unwrap();
        for oriented in wire.edges() {
            let edge = topo.edge(oriented.edge()).unwrap();
            let (_, v) = carrier.project_point(topo.vertex(edge.start()).unwrap().point());
            assert_eq!(v < 0.0, lower, "the wall's nappe on its carrier");
        }
        topo.face_mut(wall)
            .unwrap()
            .set_surface(FaceSurface::Cone(carrier));
    }
    (topo, half, walls)
}

struct Oracle {
    volume: f64,
    centroid: [f64; 3],
    /// `∫ (x² + y²) dV` about the axis, less `V ȳ²`: `Izz` at the centroid.
    izz: f64,
    wall_area: f64,
}

fn oracle(s: f64) -> Oracle {
    let (big, small, h) = (2.0 * s, s, 2.0 * s);
    // ρ runs linearly from `big` to `small`:
    // ∫₀ʰ ρⁿ dz = h (Rⁿ⁺¹ − rⁿ⁺¹) / ((n + 1)(R − r)).
    let rho_n =
        |n: i32| h * (big.powi(n + 1) - small.powi(n + 1)) / (f64::from(n + 1) * (big - small));
    let volume = PI / 2.0 * rho_n(2);
    // Half discs of centroid 4ρ / 3π off the axis: ∫ (π ρ² / 2)(4 ρ / 3π).
    let y_bar = (2.0 / 3.0) * rho_n(3) / volume;
    // ∫ z ρ² dz with ρ = R − k z, k = (R − r) / h.
    let k = (big - small) / h;
    let z_rho2 = (k * k).mul_add(
        h.powi(4) / 4.0,
        big * big * h * h / 2.0 - 2.0 * big * k * h.powi(3) / 3.0,
    );
    let z_bar = PI / 2.0 * z_rho2 / volume;
    let izz = (PI / 4.0).mul_add(rho_n(4), -volume * y_bar * y_bar);
    let slant = h.hypot(big - small);
    Oracle {
        volume,
        centroid: [0.0, y_bar, z_bar],
        izz,
        wall_area: PI / 2.0 * (big + small) * slant,
    }
}

/// Record a failed comparison, so one run reports every term that is off.
macro_rules! check {
    ($failures:ident, $cond:expr, $($fmt:tt)+) => {
        let holds: bool = $cond;
        if !holds {
            $failures.push(format!($($fmt)+));
        }
    };
}

#[test]
fn a_half_frustum_on_either_nappe_measures_the_closed_form() {
    let mut failures = Vec::new();
    for s in [1.0, 1e2, 1e4] {
        let want = oracle(s);
        let (upper_topo, upper, _) = half_frustum(s, false);
        let upper_props = measure::mass_properties(&upper_topo, upper).unwrap();
        let upper_box = measure::solid_bounding_box(&upper_topo, upper).unwrap();
        for lower in [false, true] {
            let label = format!("scale {s}, lower nappe {lower}");
            let (topo, solid, walls) = half_frustum(s, lower);
            let volume = measure::solid_volume(&topo, solid, 1e-3 * s).unwrap();
            check!(
                failures,
                (volume - want.volume).abs() <= 1e-9 * want.volume,
                "{label}: solid_volume {volume} vs {}",
                want.volume
            );
            let props = measure::mass_properties(&topo, solid).unwrap();
            check!(
                failures,
                (props.mass - want.volume).abs() <= 1e-9 * want.volume,
                "{label}: mass {} vs {}",
                props.mass,
                want.volume
            );
            let centroid = [props.center.x(), props.center.y(), props.center.z()];
            for (axis, (got, exact)) in centroid.into_iter().zip(want.centroid).enumerate() {
                check!(
                    failures,
                    (got - exact).abs() <= 1e-9 * s,
                    "{label}: centroid {axis} {got} vs {exact}"
                );
            }
            let inertia_scale = want.volume * s * s;
            check!(
                failures,
                (props.inertia[2] - want.izz).abs() <= 1e-8 * inertia_scale,
                "{label}: Izz {} vs {}",
                props.inertia[2],
                want.izz
            );
            for (index, (got, upper)) in props
                .inertia
                .into_iter()
                .zip(upper_props.inertia)
                .enumerate()
            {
                check!(
                    failures,
                    (got - upper).abs() <= 1e-9 * inertia_scale,
                    "{label}: inertia {index} {got} vs the upper carrier's {upper}"
                );
            }
            let area: f64 = walls
                .iter()
                .map(|&wall| measure::face_area(&topo, wall, 1e-3 * s).unwrap())
                .sum();
            check!(
                failures,
                (area - want.wall_area).abs() <= 1e-9 * want.wall_area,
                "{label}: face_area {area} vs {}",
                want.wall_area
            );
            let bbox = measure::solid_bounding_box(&topo, solid).unwrap();
            for (got, upper) in [(bbox.min, upper_box.min), (bbox.max, upper_box.max)] {
                check!(
                    failures,
                    (got - upper).length() <= 1e-9 * s,
                    "{label}: bounding box corner {got:?} vs the upper carrier's {upper:?}"
                );
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
