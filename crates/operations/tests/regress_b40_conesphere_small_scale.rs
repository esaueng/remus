//! A sphere patch whose boundary loop winds once around the sphere's polar
//! axis holds a pole, and must mesh as that pole cap (B40, B26 finding 15).
//!
//! The witness fuses a frustum cone (r0 = 1, r1 = 0.5, h = 1) with a unit
//! sphere rotated a quarter turn about x (poles on ±y) and centred on the
//! cone's top face (0, 0, 1). The sphere's primitive equator now lies in the
//! plane y = 0, so each retained hemisphere face is bounded by an equator arc
//! and half of the coaxial section circle, and that loop winds once around
//! the ±y pole. The parametric CDT unwrapped the loop into an open curve over
//! a full u period and closed it with a chord, which meshes the lens between
//! the equator and the section: exactly the region the cut keeps. At a fine
//! deflection every section segment was then one-sided (1428 open edges). At
//! a coarse one the boundary weld glued the wrong lens onto the cone and
//! coincident-triangle removal erased both, which read as watertight. The
//! defect was never scale-specific: unit and 1e3 scale failed identically at
//! the harness deflection. The proptest simply drew it at 1e-3.
//!
//! The section edges are marched NURBS that sit 2.5e-6 to 2.7e-5 of the
//! radius off the sphere (scale-invariant). The stereographic hemisphere
//! filler rejects them on its 1e-8 radius on-sphere gate, and that gate is
//! kept. The pole-cap mesher charts each sample by its unit direction
//! instead.
//!
//! Two families, three scales, three operations:
//!
//! - **Witness.** The finding-15 placement. The fuse and sphere-minus-cone
//!   faces hold poles and are not reversed.
//! - **Contained pole.** A 0.6 sphere at (0, 0, 0.7) sits inside the cone
//!   except for a cap poking through the side near the top. The cut keeps the
//!   sphere's pole-holding part as REVERSED cavity-wall faces, which proves
//!   the orientation rule (winding sign selects the pole in the carrier
//!   frame, and the reversed flag is applied afterwards).
//!
//! Oracles, all independent of the kernel's own volume routing:
//!
//! - closed-form volumes by inclusion–exclusion (derivations inline);
//! - the watertight mesh's own divergence-theorem volume at the harness
//!   deflection;
//! - watertight and manifold meshes at 0.1, 0.01 and the harness deflection;
//! - both validators;
//! - ray-cast classification at intent probes;
//! - translation invariance: the mesh under the harness's absolute offset,
//!   `solid_volume` under a body-proportional one.
//!
//! Found here and fixed as B56: `solid_volume`'s whole-solid route summed
//! signed tetrahedra about the world origin (`measure/volume.rs`,
//! `signed_volume_from_mesh`), so a 1e-3 body moved 13 units away cancelled
//! catastrophically and read up to 0.7% off although its mesh was unchanged.
//! `b40_solid_volume_far_translation_small_scale` pins it.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI};

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::{solid_bounding_box, solid_volume};
use remus_operations::primitives::{make_cone, make_sphere};
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];
const OPS: [BooleanOp; 3] = [BooleanOp::Fuse, BooleanOp::Cut, BooleanOp::Intersect];
/// Relative volume slack. Mesh-backed readings at these deflections land
/// within 2e-4 of the closed forms (an inscribed mesh reads low by about
/// surface area × deflection); a mis-meshed pole cap or a lost lens moves the
/// volume by whole percent.
const VOL_REL: f64 = 1e-3;

/// Cone (r0 = 1, r1 = 0.5, h = 1) volume: `pi h (r0^2 + r0 r1 + r1^2) / 3`.
fn cone_volume() -> f64 {
    PI * (1.0 + 0.5 + 0.25) / 3.0
}

/// Volume of the slice of that cone between heights `z0` and 1, whose radius
/// is `1 - z / 2`.
fn cone_top_frustum(z0: f64) -> f64 {
    let r0 = 1.0 - z0 / 2.0;
    PI * (1.0 - z0) * (r0 * r0 + r0 * 0.5 + 0.25) / 3.0
}

/// Spherical cap of height `h` on radius `r`: `pi h^2 (3 r - h) / 3`.
fn sphere_cap(r: f64, h: f64) -> f64 {
    PI * h * h * (3.0 * r - h) / 3.0
}

struct Family {
    name: &'static str,
    sphere_radius: f64,
    sphere_z: f64,
    /// Unit-scale intersection volume, derived by hand.
    intersection: f64,
    /// Unit-scale probes: (point, expected in fuse, cut, intersect).
    probes: Vec<(Point3, [PointClassification; 3])>,
}

fn families() -> [Family; 2] {
    use PointClassification::{Inside, Outside};
    // Witness: sphere r = 1 centred on the cone axis at z = 1. The cone
    // radius 1 - z/2 meets the sphere where (1 - z/2)^2 + (z - 1)^2 = 1,
    // i.e. 1.25 z^2 - 3 z + 1 = 0, so z = 0.4 (radius 0.8). Below it the
    // sphere lies inside the cone; above it the cone lies inside the sphere.
    // Intersection = sphere cap below z = 0.4 (height 0.4) + cone slice
    // from 0.4 to 1.
    let witness = Family {
        name: "witness",
        sphere_radius: 1.0,
        sphere_z: 1.0,
        intersection: sphere_cap(1.0, 0.4) + cone_top_frustum(0.4),
        probes: vec![
            // Near the +y pole, inside the sphere only.
            (Point3::new(0.0, 0.9, 1.0), [Inside, Outside, Outside]),
            // Just past the +y pole: outside everything.
            (Point3::new(0.0, 1.1, 1.0), [Outside, Outside, Outside]),
            // Inside both (the sphere's lower cap within the cone).
            (Point3::new(0.0, 0.0, 0.3), [Inside, Outside, Inside]),
            // Cone base ring, outside the sphere.
            (Point3::new(0.85, 0.0, 0.1), [Inside, Inside, Outside]),
        ],
    };
    // Contained pole: sphere r = 0.6 at z = 0.7 (poles at (0, ±0.6, 0.7),
    // inside the cone, whose radius there is 0.65). The sphere meets the
    // cone side where (1 - z/2)^2 + (z - 0.7)^2 = 0.36, i.e.
    // 1.25 z^2 - 2.4 z + 1.13 = 0, so z1 = (2.4 - sqrt(0.11)) / 2.5 (the
    // other root lies above the top face). The cone's top disc (radius 0.5)
    // is inside the sphere's z = 1 slice (radius sqrt(0.27)).
    // Intersection = sphere below z1 (sphere minus the cap of height
    // 1.3 - z1) + cone slice from z1 to 1.
    let z1 = (2.4 - 0.11_f64.sqrt()) / 2.5;
    let contained = Family {
        name: "contained-pole",
        sphere_radius: 0.6,
        sphere_z: 0.7,
        intersection: 4.0 / 3.0 * PI * 0.216 - sphere_cap(0.6, 1.3 - z1) + cone_top_frustum(z1),
        probes: vec![
            // Near the +y pole: inside the sphere and the cone.
            (Point3::new(0.0, 0.55, 0.7), [Inside, Outside, Inside]),
            // Cone base ring, below the sphere.
            (Point3::new(0.9, 0.0, 0.05), [Inside, Inside, Outside]),
            // The sphere's cap poking out through the cone side.
            (Point3::new(0.0, 0.0, 1.25), [Inside, Outside, Outside]),
        ],
    };
    [witness, contained]
}

fn build(family: &Family, scale: f64, op: BooleanOp) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let cone = make_cone(&mut topo, scale, 0.5 * scale, scale).unwrap();
    let sphere = make_sphere(&mut topo, family.sphere_radius * scale, 8).unwrap();
    let place =
        Mat4::translation(0.0, 0.0, family.sphere_z * scale) * Mat4::rotation_x(3.0 * FRAC_PI_2);
    transform_solid(&mut topo, sphere, &place).unwrap();
    let outcome = boolean_with_context(
        &mut topo,
        op,
        cone,
        sphere,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap_or_else(|e| {
        panic!(
            "{} {op:?} at {scale}: exact boolean refused: {e:?}",
            family.name
        )
    });
    assert!(
        matches!(outcome.quality, BooleanQuality::Exact),
        "{} {op:?} at {scale}: not exact",
        family.name
    );
    (topo, outcome.solid)
}

fn expected_volume(family: &Family, op: BooleanOp, scale: f64) -> f64 {
    let a = cone_volume();
    let b = 4.0 / 3.0 * PI * family.sphere_radius.powi(3);
    let i = family.intersection;
    let unit = match op {
        BooleanOp::Fuse => a + b - i,
        BooleanOp::Cut => a - i,
        BooleanOp::Intersect => i,
    };
    unit * scale.powi(3)
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

/// The harness deflection (`prop_boolean_invariants::mesh_deflection`).
fn harness_deflection(topo: &Topology, solid: SolidId) -> f64 {
    let aabb = solid_bounding_box(topo, solid).unwrap();
    ((aabb.max - aabb.min).length() * 1e-5).max(1e-7)
}

/// Signed divergence-theorem volume and total area of a closed mesh, the
/// volume taken about its first vertex so far-from-origin placements do not
/// cancel catastrophically.
fn mesh_volume_and_area(mesh: &TriangleMesh) -> (f64, f64) {
    let origin = mesh.positions[0];
    mesh.indices
        .chunks_exact(3)
        .fold((0.0, 0.0), |(volume, area), t| {
            let a = mesh.positions[t[0] as usize] - origin;
            let b = mesh.positions[t[1] as usize] - origin;
            let c = mesh.positions[t[2] as usize] - origin;
            (
                volume + a.dot(b.cross(c)) / 6.0,
                area + (b - a).cross(c - a).length() / 2.0,
            )
        })
}

fn check_leg(family: &Family, scale: f64, op: BooleanOp) {
    let what = format!("{} {op:?} at scale {scale}", family.name);
    let (topo, solid) = build(family, scale, op);
    let expected = expected_volume(family, op, scale);

    // Census: the exact result keeps analytic carriers, in a handful of faces.
    let faces = solid_faces(&topo, solid).unwrap();
    assert!(
        !faces.is_empty() && faces.len() <= 8,
        "{what}: {} faces",
        faces.len()
    );
    for &face in &faces {
        assert!(
            !matches!(topo.face(face).unwrap().surface(), FaceSurface::Nurbs(_)),
            "{what}: a face degraded to NURBS"
        );
    }

    // Both validators.
    let report = remus_operations::validate::validate_solid(&topo, solid).unwrap();
    assert!(
        report.is_valid(),
        "{what}: ops validator: {:?}",
        report.issues
    );
    let check = remus_check::validate::validate_solid(
        &topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    let errors: Vec<_> = check
        .issues
        .iter()
        .filter(|i| i.severity == remus_check::validate::Severity::Error)
        .collect();
    assert!(errors.is_empty(), "{what}: check-crate errors: {errors:?}");

    // Watertight and manifold at coarse, medium and the harness deflection;
    // the finest mesh's own volume is an oracle independent of
    // `solid_volume`'s routing.
    let harness = harness_deflection(&topo, solid);
    let mut base_volume = f64::NAN;
    for deflection in [0.1 * scale, 0.01 * scale, harness] {
        let mesh = tessellate_solid(&topo, solid, deflection).unwrap();
        let (open, branching) = (boundary_edge_count(&mesh), non_manifold_edge_count(&mesh));
        assert!(
            open == 0 && branching == 0,
            "{what}: mesh at {deflection:e} has {open} boundary and {branching} non-manifold edges"
        );
        // Every mesh point lies within `deflection` of the true boundary,
        // so the enclosed volume can differ by at most area × deflection. A
        // wrong-region cap or a skinned hole misses by a whole lens instead.
        let (v, area) = mesh_volume_and_area(&mesh);
        assert!(
            (v - expected).abs() <= area * deflection,
            "{what}: mesh volume at {deflection:e} is {v:e}, closed form {expected:e}, \
             sag bound {:e}",
            area * deflection
        );
        base_volume = v;
    }

    // The kernel's volume against the closed form. `solid_volume` refines
    // any coarser request to its own bbox-derived clamp.
    let v0 = solid_volume(&topo, solid, 0.01 * scale).unwrap();
    assert!(
        rel(v0, expected) <= VOL_REL,
        "{what}: solid_volume {v0:e}, closed form {expected:e}"
    );

    // Translation invariance. The mesh must not depend on placement: moved
    // by the harness's absolute offset (thousands of body lengths at 1e-3
    // scale) it stays watertight with the same enclosed volume. The kernel's
    // volume must hold under a body-proportional move. (Its reading under the
    // absolute offset is the B56 measure-layer case pinned by
    // `b40_solid_volume_far_translation_small_scale`.)
    let mut moved = topo.clone();
    transform_solid(&mut moved, solid, &Mat4::translation(13.0, -7.0, 5.0)).unwrap();
    let mesh = tessellate_solid(&moved, solid, harness).unwrap();
    let (open, branching) = (boundary_edge_count(&mesh), non_manifold_edge_count(&mesh));
    assert!(
        open == 0 && branching == 0,
        "{what}: moved 13 units, mesh has {open} boundary and {branching} non-manifold edges"
    );
    let (v, _) = mesh_volume_and_area(&mesh);
    assert!(
        rel(v, base_volume) <= 1e-6,
        "{what}: moved 13 units, mesh volume {v:e} vs {base_volume:e} in place"
    );
    let mut moved = topo.clone();
    transform_solid(
        &mut moved,
        solid,
        &Mat4::translation(13.0 * scale, -7.0 * scale, 5.0 * scale),
    )
    .unwrap();
    let v1 = solid_volume(&moved, solid, 0.01 * scale).unwrap();
    assert!(
        rel(v1, expected) <= VOL_REL,
        "{what}: solid_volume {v1:e} after a body-scaled move, closed form {expected:e}"
    );

    // Intent probes with the ray-cast classifier.
    let options = ClassifyOptions {
        tolerance: 1e-6 * scale,
        ..ClassifyOptions::default()
    };
    let slot = match op {
        BooleanOp::Fuse => 0,
        BooleanOp::Cut => 1,
        BooleanOp::Intersect => 2,
    };
    for (point, expect) in &family.probes {
        let p = Point3::new(point.x() * scale, point.y() * scale, point.z() * scale);
        let got = classify_point(&topo, solid, p, &options).unwrap();
        assert_eq!(got, expect[slot], "{what}: probe {point:?}");
    }
}

fn check_family_at(index: usize, scale: f64) {
    let family = &families()[index];
    for op in OPS {
        check_leg(family, scale, op);
    }
}

// One test per family and scale so the harness-deflection legs (a million
// triangles per pole-holding fuse at unit scale) run in parallel.
#[test]
fn b40_witness_small_scale() {
    check_family_at(0, SCALES[0]);
}

#[test]
fn b40_witness_unit_scale() {
    check_family_at(0, SCALES[1]);
}

#[test]
fn b40_witness_large_scale() {
    check_family_at(0, SCALES[2]);
}

#[test]
fn b40_contained_pole_small_scale() {
    check_family_at(1, SCALES[0]);
}

#[test]
fn b40_contained_pole_unit_scale() {
    check_family_at(1, SCALES[1]);
}

#[test]
fn b40_contained_pole_large_scale() {
    check_family_at(1, SCALES[2]);
}

/// Measure-layer case found closing B40 (B56): at 1e-3 scale, the harness's
/// absolute (13, -7, 5) move leaves the mesh the same shape (see `check_leg`),
/// yet `solid_volume` drifted by up to 0.7% on every leg while its
/// whole-solid route (`signed_volume_from_mesh`) summed signed tetrahedra
/// about the world origin: terms of order `|offset|^3` cancelled to a 1e-9
/// result. The sum is now taken about the mesh's bounding-box centre.
#[test]
fn b40_solid_volume_far_translation_small_scale() {
    for family in &families() {
        for op in OPS {
            let (topo, solid) = build(family, 1e-3, op);
            let expected = expected_volume(family, op, 1e-3);
            let harness = harness_deflection(&topo, solid);
            let mut moved = topo.clone();
            transform_solid(&mut moved, solid, &Mat4::translation(13.0, -7.0, 5.0)).unwrap();
            let v = solid_volume(&moved, solid, harness).unwrap();
            assert!(
                rel(v, expected) <= VOL_REL,
                "{} {op:?}: solid_volume {v:e} 13 units away, closed form {expected:e}",
                family.name
            );
        }
    }
}
