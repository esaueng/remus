//! Regression (#1742): two parallel r20 h30 cylinders must fuse exactly for
//! every axis separation below 2r, through both the pairwise boolean and
//! `fuse_solids` (wasm `fuseAll`), with the caps coplanar or offset.
//!
//! From 39 mm apart (an overlap of 1 mm or less) each cylinder wall meets
//! the other's cap in a circle that enters the cap for only a short lens arc
//! (26° at 39 mm, 8° at 39.9 mm). Phase FF's sampled graze test saw too few
//! in-face samples on it and dropped it as a tangency, so the caps were never
//! split and the exact pipeline refused the fuse.
//!
//! Every case is checked against the closed-form union volume
//! `2·π·r²·h − lens(d)·(h − dz)`, by exact integration and by tessellation,
//! with the second axis placed along +x and off-axis (so no sample lines up
//! with the lens by symmetry).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::compound_ops::fuse_solids;
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::make_cylinder;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const R: f64 = 20.0;
const H: f64 = 30.0;
const SEPARATIONS: [f64; 4] = [38.5, 39.0, 39.5, 39.9];
const CAP_OFFSETS: [f64; 2] = [0.0, 2.0];
/// Directions of the second axis from the first, in degrees.
const DIRECTIONS: [f64; 2] = [0.0, 37.0];

/// Area common to two discs of radius `r` with centres `d` apart.
fn lens(r: f64, d: f64) -> f64 {
    2.0 * r * r * (d / (2.0 * r)).acos() - 0.5 * d * (4.0 * r * r - d * d).sqrt()
}

fn expected_volume(d: f64, dz: f64) -> f64 {
    2.0 * std::f64::consts::PI * R * R * H - lens(R, d) * (H - dz)
}

fn operands(topo: &mut Topology, d: f64, dz: f64, degrees: f64) -> (SolidId, SolidId) {
    let a = make_cylinder(topo, R, H).unwrap();
    let b = make_cylinder(topo, R, H).unwrap();
    let (s, c) = degrees.to_radians().sin_cos();
    transform_solid(topo, b, &Mat4::translation(d * c, d * s, dz)).unwrap();
    (a, b)
}

fn check(topo: &Topology, solid: SolidId, what: &str, expected: f64) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{what}: {:?}", report.issues);
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    let cylinders = faces
        .iter()
        .filter(|f| matches!(topo.face(**f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .count();
    assert!(
        cylinders >= 2 && faces.len() < 20,
        "{what}: not an analytic result ({} faces, {cylinders} cylindrical)",
        faces.len()
    );
    let exact = mass_properties(topo, solid).unwrap().mass;
    let tessellated = solid_volume(topo, solid, 1e-3).unwrap();
    for (label, value) in [("exact", exact), ("tessellated", tessellated)] {
        let rel = ((value - expected) / expected).abs();
        assert!(
            rel < 1e-6,
            "{what}: {label} volume {value}, expected {expected} (rel {rel:.2e})"
        );
    }
}

type Fuse =
    fn(&mut Topology, SolidId, SolidId) -> Result<SolidId, remus_operations::OperationsError>;

fn each_case(run: Fuse, entry: &str) {
    for degrees in DIRECTIONS {
        for d in SEPARATIONS {
            for dz in CAP_OFFSETS {
                let mut topo = Topology::new();
                let (a, b) = operands(&mut topo, d, dz, degrees);
                let what = format!("{entry} d={d} dz={dz} at {degrees}°");
                let fused = run(&mut topo, a, b).unwrap_or_else(|e| panic!("{what} refused: {e}"));
                check(&topo, fused, &what, expected_volume(d, dz));
            }
        }
    }
}

#[test]
fn pairwise_fuse_of_parallel_cylinders_with_thin_lens_is_exact() {
    each_case(|topo, a, b| boolean(topo, BooleanOp::Fuse, a, b), "fuse");
}

#[test]
fn fuse_solids_of_parallel_cylinders_with_thin_lens_is_exact() {
    each_case(|topo, a, b| fuse_solids(topo, &[a, b]), "fuse_solids");
}

/// The former mesh-fallback fixture of `fallback_policy.rs` and the batch
/// contract tests: two r5 h10 cylinders whose walls overlap by 1e-3, caps
/// flush or the second raised 3. Its lens is about 1e-6 of either volume, so
/// the result is held to the lens itself: the volume each Boolean removes
/// (or keeps, for the intersection) by exact integration against the closed
/// form, and the lens's middle classified on the right side.
#[test]
fn former_near_tangent_fallback_fixture_is_exact_and_right() {
    use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
    use remus_math::vec::Point3;
    use remus_operations::boolean::{BooleanOptions, compound_cut};

    const RADIUS: f64 = 5.0;
    const HEIGHT: f64 = 10.0;
    const D: f64 = 9.999;
    for dz in [0.0, 3.0] {
        let lens_volume = lens(RADIUS, D) * (HEIGHT - dz);
        let middle = Point3::new(D / 2.0, 0.0, dz + 0.5 * (HEIGHT - dz));
        for op in ["fuse", "cut", "intersect", "fuse_solids", "compound_cut"] {
            let what = format!("{op} dz={dz}");
            let mut topo = Topology::new();
            let a = make_cylinder(&mut topo, RADIUS, HEIGHT).unwrap();
            let b = make_cylinder(&mut topo, RADIUS, HEIGHT).unwrap();
            transform_solid(&mut topo, b, &Mat4::translation(D, 0.0, dz)).unwrap();
            let operand = mass_properties(&topo, a).unwrap().mass;
            let result = match op {
                "fuse" => boolean(&mut topo, BooleanOp::Fuse, a, b),
                "cut" => boolean(&mut topo, BooleanOp::Cut, a, b),
                "intersect" => boolean(&mut topo, BooleanOp::Intersect, a, b),
                "fuse_solids" => fuse_solids(&mut topo, &[a, b]),
                _ => compound_cut(&mut topo, a, &[b], BooleanOptions::default()),
            }
            .unwrap_or_else(|e| panic!("{what} refused: {e}"));
            let report = validate_solid(&topo, result).unwrap();
            assert!(report.is_valid(), "{what}: {:?}", report.issues);
            let volume = mass_properties(&topo, result).unwrap().mass;
            let (lens_seen, middle_inside) = match op {
                "fuse" | "fuse_solids" => (2.0 * operand - volume, true),
                "intersect" => (volume, true),
                _ => (operand - volume, false),
            };
            assert!(
                ((lens_seen - lens_volume) / lens_volume).abs() < 1e-6,
                "{what}: lens volume {lens_seen:e}, expected {lens_volume:e}"
            );
            let side = classify_point(&topo, result, middle, &ClassifyOptions::default()).unwrap();
            let wanted = if middle_inside {
                PointClassification::Inside
            } else {
                PointClassification::Outside
            };
            assert_eq!(side, wanted, "{what}: the lens's middle");
        }
    }
}
