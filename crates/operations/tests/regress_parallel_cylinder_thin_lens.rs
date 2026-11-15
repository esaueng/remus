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

/// What the exact pipeline did on `main` before the thin cap-lens retry, for
/// a case of [`sweep_subset`]. Documentation only: the test holds the branch
/// to an exact, clean, right result either way.
#[derive(Clone, Copy, Debug)]
enum Main {
    /// Exact there too: the retry never runs, so the result is main's own.
    Exact,
    /// Refused there: the retry builds it.
    Refused,
}

/// Two parallel cylinders of radius `r` (height 10 for r5, 30 for r20),
/// axes `2r - overlap` apart along x, the second raised `dz`.
fn overlap_pair(topo: &mut Topology, r: f64, dz: f64, overlap: f64) -> (SolidId, SolidId, f64) {
    let height = if r < 10.0 { 10.0 } else { 30.0 };
    let a = make_cylinder(topo, r, height).unwrap();
    let b = make_cylinder(topo, r, height).unwrap();
    transform_solid(topo, b, &Mat4::translation(2.0 * r - overlap, 0.0, dz)).unwrap();
    (a, b, height)
}

fn run_op(
    topo: &mut Topology,
    op: &str,
    a: SolidId,
    b: SolidId,
) -> Result<SolidId, remus_operations::OperationsError> {
    use remus_operations::boolean::{BooleanOptions, compound_cut};
    match op {
        "fuse" => boolean(topo, BooleanOp::Fuse, a, b),
        "cut" => boolean(topo, BooleanOp::Cut, a, b),
        "intersect" => boolean(topo, BooleanOp::Intersect, a, b),
        "fuse_solids" => fuse_solids(topo, &[a, b]),
        "compound_cut" => compound_cut(topo, a, &[b], BooleanOptions::default()),
        _ => unreachable!("{op}"),
    }
}

/// The lens volume an `op` result accounts for: removed by a fuse or a cut,
/// kept by an intersection.
fn lens_seen(op: &str, operand: f64, volume: f64) -> f64 {
    match op {
        "fuse" | "fuse_solids" => 2.0 * operand - volume,
        "intersect" => volume,
        _ => operand - volume,
    }
}

/// Review of the depth gate (#1770): with the thin cap-lens keep applied on
/// every run, r5 walls overlapping by just over its depth (1.01e-5, 2e-5),
/// the second raised 3, were refused for the Cut that `main` makes exactly.
/// The keep is now a retry after a failed run, so `main`'s result stands.
/// Their lenses are ~1e-9 of the volume, under what the volume can resolve,
/// so the result is held to exact, clean and the closed-form total.
#[test]
fn cut_just_past_the_depth_gate_stays_exact() {
    for overlap in [1.01e-5, 2e-5] {
        for op in ["cut", "compound_cut"] {
            let what = format!("{op} overlap={overlap:e}");
            let mut topo = Topology::new();
            let (a, b, height) = overlap_pair(&mut topo, 5.0, 3.0, overlap);
            let operand = mass_properties(&topo, a).unwrap().mass;
            let result = run_op(&mut topo, op, a, b).unwrap_or_else(|e| panic!("{what}: {e}"));
            let report = validate_solid(&topo, result).unwrap();
            assert!(report.issues.is_empty(), "{what}: {:?}", report.issues);
            let expected = operand - lens(5.0, 10.0 - overlap) * (height - 3.0);
            let volume = mass_properties(&topo, result).unwrap().mass;
            assert!(
                ((volume - expected) / expected).abs() < 1e-8,
                "{what}: volume {volume}, expected {expected}"
            );
        }
    }
}

/// A representative subset of the main-vs-branch sweep in PR #1770 (radius,
/// cap offset, wall overlap, operation, what `main` did): every case where
/// the branch is exact and its lens volume matches the closed form, at
/// overlaps either side of the depth gate up to 1.5 mm. Each must be exact,
/// carry no validation issue, and account for its lens to 1e-6.
fn sweep_subset() -> Vec<(f64, f64, f64, &'static str, Main)> {
    vec![
        (5.0, 0.0, 1e-4, "fuse", Main::Refused),
        (5.0, 0.0, 1e-4, "cut", Main::Refused),
        (5.0, 0.0, 1e-4, "intersect", Main::Refused),
        (5.0, 0.0, 1e-4, "fuse_solids", Main::Exact),
        (5.0, 0.0, 1e-4, "compound_cut", Main::Refused),
        (5.0, 0.0, 1e-3, "fuse", Main::Refused),
        (5.0, 0.0, 1e-3, "cut", Main::Refused),
        (5.0, 0.0, 1e-3, "intersect", Main::Refused),
        (5.0, 0.0, 1e-3, "fuse_solids", Main::Exact),
        (5.0, 0.0, 1e-3, "compound_cut", Main::Refused),
        (5.0, 0.0, grid(26), "fuse", Main::Refused),
        (5.0, 0.0, grid(26), "cut", Main::Refused),
        (5.0, 0.0, grid(26), "intersect", Main::Refused),
        (5.0, 0.0, grid(26), "fuse_solids", Main::Exact),
        (5.0, 0.0, grid(26), "compound_cut", Main::Refused),
        (5.0, 0.0, 1.0, "fuse", Main::Exact),
        (5.0, 0.0, 1.0, "cut", Main::Exact),
        (5.0, 0.0, 1.0, "intersect", Main::Exact),
        (5.0, 0.0, 1.0, "fuse_solids", Main::Exact),
        (5.0, 0.0, 1.0, "compound_cut", Main::Exact),
        (5.0, 0.0, 1.5, "fuse", Main::Exact),
        (5.0, 0.0, 1.5, "cut", Main::Exact),
        (5.0, 0.0, 1.5, "intersect", Main::Exact),
        (5.0, 0.0, 1.5, "fuse_solids", Main::Exact),
        (5.0, 0.0, 1.5, "compound_cut", Main::Exact),
        (5.0, 3.0, 1e-4, "fuse", Main::Refused),
        (5.0, 3.0, 1e-4, "intersect", Main::Refused),
        (5.0, 3.0, 1e-4, "fuse_solids", Main::Refused),
        (5.0, 3.0, 1e-3, "fuse", Main::Refused),
        (5.0, 3.0, 1e-3, "cut", Main::Refused),
        (5.0, 3.0, 1e-3, "intersect", Main::Refused),
        (5.0, 3.0, 1e-3, "fuse_solids", Main::Refused),
        (5.0, 3.0, 1e-3, "compound_cut", Main::Refused),
        (5.0, 3.0, grid(26), "fuse", Main::Refused),
        (5.0, 3.0, grid(26), "cut", Main::Refused),
        (5.0, 3.0, grid(26), "intersect", Main::Refused),
        (5.0, 3.0, grid(26), "fuse_solids", Main::Refused),
        (5.0, 3.0, grid(26), "compound_cut", Main::Refused),
        (5.0, 3.0, 1.0, "fuse", Main::Exact),
        (5.0, 3.0, 1.0, "cut", Main::Exact),
        (5.0, 3.0, 1.0, "intersect", Main::Exact),
        (5.0, 3.0, 1.0, "fuse_solids", Main::Exact),
        (5.0, 3.0, 1.0, "compound_cut", Main::Exact),
        (5.0, 3.0, 1.5, "fuse", Main::Exact),
        (5.0, 3.0, 1.5, "cut", Main::Exact),
        (5.0, 3.0, 1.5, "intersect", Main::Exact),
        (5.0, 3.0, 1.5, "fuse_solids", Main::Exact),
        (5.0, 3.0, 1.5, "compound_cut", Main::Exact),
        (20.0, 0.0, 1e-4, "fuse_solids", Main::Exact),
        (20.0, 0.0, 1e-3, "fuse", Main::Refused),
        (20.0, 0.0, 1e-3, "cut", Main::Refused),
        (20.0, 0.0, 1e-3, "intersect", Main::Refused),
        (20.0, 0.0, 1e-3, "fuse_solids", Main::Exact),
        (20.0, 0.0, 1e-3, "compound_cut", Main::Refused),
        (20.0, 0.0, grid(26), "fuse", Main::Refused),
        (20.0, 0.0, grid(26), "cut", Main::Refused),
        (20.0, 0.0, grid(26), "intersect", Main::Refused),
        (20.0, 0.0, grid(26), "fuse_solids", Main::Exact),
        (20.0, 0.0, grid(26), "compound_cut", Main::Refused),
        (20.0, 0.0, 1.0, "fuse", Main::Refused),
        (20.0, 0.0, 1.0, "cut", Main::Refused),
        (20.0, 0.0, 1.0, "intersect", Main::Refused),
        (20.0, 0.0, 1.0, "fuse_solids", Main::Exact),
        (20.0, 0.0, 1.0, "compound_cut", Main::Refused),
        (20.0, 0.0, 1.5, "fuse", Main::Exact),
        (20.0, 0.0, 1.5, "cut", Main::Exact),
        (20.0, 0.0, 1.5, "intersect", Main::Exact),
        (20.0, 0.0, 1.5, "fuse_solids", Main::Exact),
        (20.0, 0.0, 1.5, "compound_cut", Main::Exact),
        (20.0, 3.0, 1e-3, "fuse", Main::Refused),
        (20.0, 3.0, 1e-3, "intersect", Main::Refused),
        (20.0, 3.0, 1e-3, "fuse_solids", Main::Refused),
        (20.0, 3.0, grid(26), "fuse", Main::Refused),
        (20.0, 3.0, grid(26), "intersect", Main::Refused),
        (20.0, 3.0, grid(26), "fuse_solids", Main::Refused),
        (20.0, 3.0, 1.0, "fuse", Main::Refused),
        (20.0, 3.0, 1.0, "cut", Main::Refused),
        (20.0, 3.0, 1.0, "intersect", Main::Refused),
        (20.0, 3.0, 1.0, "fuse_solids", Main::Refused),
        (20.0, 3.0, 1.0, "compound_cut", Main::Refused),
        (20.0, 3.0, 1.5, "fuse", Main::Exact),
        (20.0, 3.0, 1.5, "cut", Main::Exact),
        (20.0, 3.0, 1.5, "intersect", Main::Exact),
        (20.0, 3.0, 1.5, "fuse_solids", Main::Exact),
        (20.0, 3.0, 1.5, "compound_cut", Main::Exact),
    ]
}

/// Overlap `k` of the sweep's log grid, `10^(-7 + 7k/30)` mm, bit for bit.
fn grid(k: u32) -> f64 {
    10f64.powf(-7.0 + 7.0 * f64::from(k) / 30.0)
}

#[test]
fn sweep_subset_is_exact_clean_and_right() {
    for (r, dz, overlap, op, main) in sweep_subset() {
        let what = format!("{op} r={r} dz={dz} overlap={overlap:e} (main: {main:?})");
        let mut topo = Topology::new();
        let (a, b, height) = overlap_pair(&mut topo, r, dz, overlap);
        let operand = mass_properties(&topo, a).unwrap().mass;
        let result = run_op(&mut topo, op, a, b).unwrap_or_else(|e| panic!("{what}: {e}"));
        let report = validate_solid(&topo, result).unwrap();
        assert!(report.issues.is_empty(), "{what}: {:?}", report.issues);
        let lens_volume = lens(r, 2.0 * r - overlap) * (height - dz);
        let seen = lens_seen(op, operand, mass_properties(&topo, result).unwrap().mass);
        assert!(
            ((seen - lens_volume) / lens_volume).abs() < 1e-6,
            "{what}: lens volume {seen:e}, expected {lens_volume:e}"
        );
    }
}

/// Ready repro of a defect on `main` that this change leaves as it is: an
/// exact Cut by a parallel cylinder raised 3 returns the blank untouched at
/// many overlaps up to 0.2 mm (r5) and 0.34 mm (r20), losing up to 32 mm³.
/// The sweep in PR #1770 found it; the cut is accepted on the first exact run,
/// so the thin cap-lens retry never sees it.
#[test]
#[ignore = "open: exact Cut by a raised parallel cylinder returns the blank whole (pre-existing on main)"]
fn raised_parallel_cut_removes_its_lens() {
    for (r, overlap) in [
        (5.0, 1e-4),
        (5.0, grid(18)),
        (5.0, grid(27)),
        (20.0, grid(28)),
    ] {
        let what = format!("cut r={r} overlap={overlap:e}");
        let mut topo = Topology::new();
        let (a, b, height) = overlap_pair(&mut topo, r, 3.0, overlap);
        let operand = mass_properties(&topo, a).unwrap().mass;
        let Ok(result) = boolean(&mut topo, BooleanOp::Cut, a, b) else {
            continue;
        };
        let lens_volume = lens(r, 2.0 * r - overlap) * (height - 3.0);
        let seen = operand - mass_properties(&topo, result).unwrap().mass;
        assert!(
            ((seen - lens_volume) / lens_volume).abs() < 1e-6,
            "{what}: removed {seen:e}, expected {lens_volume:e}"
        );
    }
}
