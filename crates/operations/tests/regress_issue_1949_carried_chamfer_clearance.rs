//! Issue #1949: a planar cap carries a 45° inner chamfer when it
//! translates. Two obstacles the cap's own clearance proofs exempt, but the
//! carried chamfer cone sweeps into, must refuse the move:
//!
//! * case 1: a coaxial collar (r = 9.5, z = 1.35..1.95) on the central post,
//!   cap moved +2.4 (moved cone r(z) = 9 + (z − 1.4) is below 9.5 for
//!   z < 1.9; the extended r = 9 wall reaches z = 1.4);
//! * case 2: the #1905 tab lowered to z = −0.45..−0.05, cap moved +0.4
//!   (moved cone r(z) = 9.6 + z is 9.4 at z = −0.2; the tab reaches x = 9.5).
//!
//! Before the fix both moves returned a self-overlapping body (validator
//! clean and watertight; only volume and classification showed it). The
//! controls, where the moved chamfer clears the obstacle, must still return
//! the fuse of the moved ring and the obstacle: the oracle volume is the
//! moved ring revolved independently, plus the obstacle, less their
//! closed-form intersection.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::{
    mat::Mat4,
    vec::{Point3, Vec3},
};
use remus_operations::{
    boolean::{BooleanOp, boolean},
    classify::{PointClassification, classify_point},
    measure::solid_volume,
    primitives::{make_box, make_cylinder},
    push_pull::move_faces,
    revolve::revolve,
    tessellate::{is_watertight, tessellate_solid_with_tolerance},
    transform::transform_solid,
    validate::validate_solid,
};
use remus_topology::{
    Topology,
    builder::{make_planar_face_from_wire, make_polygon_wire},
    explorer::{face_vertices, solid_faces},
    face::{FaceId, FaceSurface},
    solid::SolidId,
};

const UP: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// The #1905 ring: cap at z = `lift` (annulus r 10..20), 45° chamfer to
/// r = 9 at z = `lift` − 1, bore wall r = 9 down to z = −3, post r = 4 up to
/// z = 3, on a base slab down to z = −5.
fn ring(topo: &mut Topology, lift: f64) -> SolidId {
    let section = [
        (0., -5.),
        (20., -5.),
        (20., lift),
        (10., lift),
        (9., lift - 1.),
        (9., -3.),
        (4., -3.),
        (4., 3.),
        (0., 3.),
    ]
    .map(|(r, z)| Point3::new(r, 0., z));
    let wire = make_polygon_wire(topo, &section, 1e-7).unwrap();
    let profile = make_planar_face_from_wire(topo, wire).unwrap();
    revolve(
        topo,
        profile,
        Point3::new(0., 0., 0.),
        UP,
        std::f64::consts::TAU,
    )
    .unwrap()
}

#[derive(Clone, Copy, Debug)]
enum Obstacle {
    /// Box x 3.5..x1, y −0.3..0.3, z z0..z1 (fused to the post).
    Tab { x1: f64, z0: f64, z1: f64 },
    /// Coaxial cylinder of radius `r` on z0..z1 (fused to the post).
    Collar { r: f64, z0: f64, z1: f64 },
}

fn obstacle(topo: &mut Topology, o: Obstacle) -> SolidId {
    match o {
        Obstacle::Tab { x1, z0, z1 } => {
            let tab = make_box(topo, x1 - 3.5, 0.6, z1 - z0).unwrap();
            transform_solid(topo, tab, &Mat4::translation(3.5, -0.3, z0)).unwrap();
            tab
        }
        Obstacle::Collar { r, z0, z1 } => {
            let collar = make_cylinder(topo, r, z1 - z0).unwrap();
            transform_solid(topo, collar, &Mat4::translation(0.0, 0.0, z0)).unwrap();
            collar
        }
    }
}

fn body(topo: &mut Topology, lift: f64, o: Obstacle) -> SolidId {
    let ring = ring(topo, lift);
    let obstacle = obstacle(topo, o);
    boolean(topo, BooleanOp::Fuse, ring, obstacle).unwrap()
}

fn cap(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.z() > 0.99)
                && face_vertices(topo, face).unwrap().iter().all(|&v| {
                    let p = topo.vertex(v).unwrap().point();
                    p.z().abs() < 1e-9 && Vec3::new(p.x(), p.y(), 0.0).length() > 9.99
                })
        })
        .expect("cap annulus at z = 0")
}

/// Inner radius of the ring's material at height `z` with its cap at
/// `lift` (`None` above the cap): the bore r = 9, then the 45° chamfer.
fn ring_inner(z: f64, lift: f64) -> Option<f64> {
    if z > lift || z < -5.0 {
        None
    } else if z <= lift - 1.0 {
        Some(9.0)
    } else {
        Some(9.0 + (z - lift + 1.0))
    }
}

/// Volume of the fuse of the lifted ring and the obstacle: the ring's own
/// measured volume, plus the obstacle's, less their intersection integrated
/// by slices (the ring is the post r <= 4 for z <= 3 and r in
/// [inner(z), 20] up to the cap).
fn oracle_volume(o: Obstacle, lift: f64) -> f64 {
    let mut topo = Topology::new();
    let ring = ring(&mut topo, lift);
    let ring_volume = solid_volume(&topo, ring, 0.01).unwrap();
    let (z0, z1, obstacle_volume) = match o {
        Obstacle::Tab { x1, z0, z1 } => (z0, z1, (x1 - 3.5) * 0.6 * (z1 - z0)),
        Obstacle::Collar { r, z0, z1 } => (z0, z1, std::f64::consts::PI * r * r * (z1 - z0)),
    };
    let section = |z: f64| -> f64 {
        let inner = ring_inner(z, lift);
        match o {
            Obstacle::Collar { r, .. } => {
                let post = if z <= 3.0 { 16.0 } else { 0.0 };
                let band = inner.map_or(0.0, |i| (r * r - i * i).max(0.0));
                std::f64::consts::PI * (post + band)
            }
            Obstacle::Tab { x1, .. } => {
                // Midpoint rule across y; each x-run is closed-form.
                let n = 20_000;
                let h = 0.6 / f64::from(n);
                (0..n)
                    .map(|k| {
                        let y = -0.3 + (f64::from(k) + 0.5) * h;
                        let post = if z <= 3.0 {
                            ((16.0 - y * y).sqrt().min(x1) - 3.5).max(0.0)
                        } else {
                            0.0
                        };
                        let band = inner.map_or(0.0, |i| (x1 - (i * i - y * y).sqrt()).max(0.0));
                        (post + band) * h
                    })
                    .sum()
            }
        }
    };
    // Midpoint rule along z, the sections piecewise smooth.
    let n = 4_000;
    let h = (z1 - z0) / f64::from(n);
    let intersection: f64 = (0..n)
        .map(|k| section(z0 + (f64::from(k) + 0.5) * h) * h)
        .sum();
    ring_volume + obstacle_volume - intersection
}

#[derive(Debug)]
struct Outcome {
    /// `Err(text)` when the move refused.
    moved: Result<Report, String>,
}

#[derive(Debug)]
#[allow(dead_code)]
struct Report {
    valid: bool,
    issues: Vec<String>,
    watertight: bool,
    faces: usize,
    volume: f64,
    oracle_volume: f64,
    probe: PointClassification,
}

impl Report {
    fn correct(&self) -> bool {
        self.valid
            && self.watertight
            && (self.volume - self.oracle_volume).abs() < 2e-3
            && self.probe == PointClassification::Inside
    }
}

/// Builds the body, moves its cap by `distance`, and compares the result
/// with the oracle fuse; `probe` is a point inside both the moved ring and
/// the obstacle (or, for a control, inside the oracle body).
fn run(o: Obstacle, distance: f64, probe: Point3) -> Outcome {
    let mut topo = Topology::new();
    let source = body(&mut topo, 0.0, o);
    let report = validate_solid(&topo, source).unwrap();
    assert!(report.is_valid(), "source invalid: {:?}", report.issues);
    assert!(
        solid_faces(&topo, source)
            .unwrap()
            .iter()
            .any(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cone(_))),
        "the opening keeps its chamfer"
    );
    let cap = cap(&topo, source);
    let moved = match move_faces(&mut topo, source, &[cap], distance) {
        Ok(moved) => moved,
        Err(err) => {
            return Outcome {
                moved: Err(err.to_string()),
            };
        }
    };
    let report = validate_solid(&topo, moved).unwrap();
    let watertight = tessellate_solid_with_tolerance(&topo, moved, 0.01, 0.2)
        .map(|m| is_watertight(&m))
        .unwrap_or(false);
    let volume = solid_volume(&topo, moved, 0.01).unwrap();
    let probe_class = classify_point(&topo, moved, probe, 0.01, 1e-7).unwrap();
    let oracle_volume = oracle_volume(o, distance);
    Outcome {
        moved: Ok(Report {
            valid: report.is_valid(),
            issues: report.issues.iter().map(|i| format!("{i:?}")).collect(),
            watertight,
            faces: solid_faces(&topo, moved).unwrap().len(),
            volume,
            oracle_volume,
            probe: probe_class,
        }),
    }
}

fn expect_refused(name: &str, outcome: &Outcome) {
    match &outcome.moved {
        Err(reason) => assert!(
            reason.contains("swept face reaches nonadjacent face"),
            "{name}: refused for another reason: {reason}"
        ),
        Ok(report) => panic!("{name}: move accepted: {report:#?}"),
    }
}

fn expect_correct(name: &str, outcome: &Outcome) {
    let report = outcome
        .moved
        .as_ref()
        .unwrap_or_else(|e| panic!("{name}: refused: {e}"));
    assert!(report.correct(), "{name}: wrong body: {report:#?}");
}

const COLLAR: Obstacle = Obstacle::Collar {
    r: 9.5,
    z0: 1.35,
    z1: 1.95,
};

const LOW_TAB: Obstacle = Obstacle::Tab {
    x1: 9.5,
    z0: -0.45,
    z1: -0.05,
};

#[test]
fn case1_cap_chamfer_cannot_sweep_into_a_coaxial_collar() {
    // (9.25, 0, 1.6): inside the collar (r < 9.5) and inside the moved
    // ring (the moved cone is at r = 9.2 there).
    let outcome = run(COLLAR, 2.4, Point3::new(9.25, 0.0, 1.6));
    expect_refused("case1 collar +2.4", &outcome);
}

#[test]
fn case1_control_collar_clear_of_the_moved_chamfer() {
    // Collar raised to z = 2.5..2.9: the moved cap (z = 2.4, hole r = 10)
    // stays below it and radially outside it.
    let clear = Obstacle::Collar {
        r: 9.5,
        z0: 2.5,
        z1: 2.9,
    };
    expect_correct(
        "case1 control (collar z 2.5..2.9) +2.4",
        &run(clear, 2.4, Point3::new(9.25, 0.0, 2.7)),
    );
    // Same collar as case 1, shorter move: the cone's foot stays at z ≤ 0.
    expect_correct(
        "case1 control (collar z 1.35..1.95) +1.0",
        &run(COLLAR, 1.0, Point3::new(9.25, 0.0, 1.6)),
    );
}

#[test]
fn case2_cap_chamfer_cannot_sweep_into_a_tab_below_the_cap_box() {
    // (9.45, 0, −0.3): inside the tab and inside the moved ring (the moved
    // cone is at r = 9.3 there).
    let outcome = run(LOW_TAB, 0.4, Point3::new(9.45, 0.0, -0.3));
    expect_refused("case2 low tab +0.4", &outcome);
}

#[test]
fn case2_control_tab_clear_of_the_moved_chamfer() {
    // Tab ends at x = 9.0 (reach 9.005): the moved cone is at r ≥ 9.15 over
    // the tab's height.
    let short = Obstacle::Tab {
        x1: 9.0,
        z0: -0.45,
        z1: -0.05,
    };
    expect_correct(
        "case2 control (tab to x 9.0) +0.4",
        &run(short, 0.4, Point3::new(8.9, 0.0, -0.3)),
    );
}

#[test]
fn case0_pr1905_tab_reference() {
    // The #1905 regression fixture itself: tab at z = 1.35..1.95, +2.4.
    let tab = Obstacle::Tab {
        x1: 9.5,
        z0: 1.35,
        z1: 1.95,
    };
    let outcome = run(tab, 2.4, Point3::new(9.4, 0.0, 1.6));
    expect_refused("case0 #1905 tab +2.4", &outcome);
}
