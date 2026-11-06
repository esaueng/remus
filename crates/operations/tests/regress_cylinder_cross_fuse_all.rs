//! `fuse_solids` (`fuseAll`) of crossing cylinders (consumer K10 combination
//! matrix on Remus 2026.1.122, 2026-11-05).
//!
//! Every cylinder × cylinder union in the matrix (83 cells: a bar r 10 along
//! X crossed by a drill r 6 along Z, centred, off-centre, tilted or at equal
//! radius) was refused by the consumer's strict gate as open, non-manifold or
//! inconsistently oriented. `fuse` on the same pair was correct; `fuseAll`,
//! which the consumer uses for every union, returned six faces: the two
//! cylinders' own faces, unsplit, stitched into one shell with one
//! validation error.
//!
//! Root cause: `partition_touching` groups the solids that need a boolean.
//! For two simple cylinders it skipped every pair whose `cylinder_relation`
//! was not `Some(PositiveOverlap)`, but `cylinder_relation` answers `None`
//! for axes that are not parallel, meaning "not classified, use the
//! conservative path". Crossing cylinders were therefore read as apart and
//! shell-merged by `merge_disjoint_solids`. Only a proved separation or
//! contact keeps cylinders apart now.
//!
//! Oracle, independent of the kernel: the overlap of a bar of radius R and a
//! drill of radius r. For a drill perpendicular to the bar it is a smooth 1D
//! Gauss–Legendre integral (16r³/3 at R = r); for a skew drill, the drill
//! disc's integral of the bar's chord length along the drill axis
//! (Gauss–Legendre in the disc radius, trapezoid rule in its angle).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::compound_ops::fuse_solids;
use remus_operations::measure::mass_properties;
use remus_operations::primitives::make_cylinder;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

const LEN: f64 = 60.0;
const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];
/// The exact section of two crossing cylinders is a quartic the kernel fits
/// with NURBS; their measured overlap reads about 1.3e-5 low (the K10
/// Steinmetz accuracy item, tracked separately). This bounds the oracle
/// comparison only; `fuseAll` must equal the pairwise `fuse` to 1e-9.
const SECTION_FIT: f64 = 2e-5;

/// Gauss–Legendre nodes and weights on [-1, 1] (Newton on Pₙ).
fn gauss_legendre(n: usize) -> Vec<(f64, f64)> {
    let nf = n as f64;
    (0..n)
        .map(|i| {
            let mut x = (PI * (i as f64 + 0.75) / (nf + 0.5)).cos();
            let mut dp = 1.0;
            for _ in 0..100 {
                let (mut p0, mut p1) = (1.0, x);
                for k in 2..=n {
                    let kf = k as f64;
                    let p2 = ((2.0 * kf - 1.0) * x * p1 - (kf - 1.0) * p0) / kf;
                    p0 = p1;
                    p1 = p2;
                }
                dp = nf * (x * p1 - p0) / (x * x - 1.0);
                let step = p1 / dp;
                x -= step;
                if step.abs() < 1e-16 {
                    break;
                }
            }
            (x, 2.0 / ((1.0 - x * x) * dp * dp))
        })
        .collect()
}

/// A drill placement: offset along Y, then tilts about X and Y (radians).
#[derive(Clone, Copy)]
struct Cross {
    name: &'static str,
    bar: f64,
    drill: f64,
    offset: f64,
    tilt_x: f64,
    tilt_y: f64,
}

impl Cross {
    fn drill_frame(&self) -> Mat4 {
        Mat4::rotation_y(self.tilt_y) * Mat4::rotation_x(self.tilt_x)
    }

    /// Unit-scale overlap volume (the bar treated as infinite along X; every
    /// case keeps the overlap well inside both cylinders' lengths).
    fn overlap(&self) -> f64 {
        if self.tilt_y == 0.0 {
            // A tilt about X turns the drill about the bar's own axis, a
            // symmetry of the bar. Perpendicular drill: with y = offset +
            // r sin θ across the bar, the overlap is
            // ∫ 4 r² cos²θ √(R² − y²) dθ over (−π/2, π/2), smooth even at
            // R = r (where it is 4r³cos³θ).
            let (big, small) = (self.bar, self.drill);
            return gauss_legendre(96)
                .into_iter()
                .map(|(x, w)| {
                    let theta = 0.5 * PI * x;
                    let y = small.mul_add(theta.sin(), self.offset);
                    let c = theta.cos();
                    4.0 * small * small * c * c * big.mul_add(big, -(y * y)).sqrt() * w
                })
                .sum::<f64>()
                * 0.5
                * PI;
        }
        // The frame is a pure rotation: it maps directions as points.
        let frame = self.drill_frame();
        let map = |x: f64, y: f64, z: f64| {
            let p = frame.mul_point(Point3::new(x, y, z));
            Vec3::new(p.x(), p.y(), p.z())
        };
        let (along, e1, e2) = (map(0.0, 0.0, 1.0), map(1.0, 0.0, 0.0), map(0.0, 1.0, 0.0));
        let centre = e2 * self.offset;
        let a = along.y() * along.y() + along.z() * along.z();
        let n_phi = 1024;
        let mut total = 0.0;
        for (x, w) in gauss_legendre(96) {
            let rho = 0.5 * self.drill * (x + 1.0);
            for k in 0..n_phi {
                let phi = 2.0 * PI * f64::from(k) / f64::from(n_phi);
                let p = centre + e1 * (rho * phi.cos()) + e2 * (rho * phi.sin());
                let b = 2.0 * (p.y() * along.y() + p.z() * along.z());
                let c = p.y() * p.y() + p.z() * p.z() - self.bar * self.bar;
                let disc = b * b - 4.0 * a * c;
                if disc > 0.0 {
                    total += disc.sqrt() / a * rho * w;
                }
            }
        }
        total * 0.5 * self.drill * 2.0 * PI / f64::from(n_phi)
    }

    fn place(&self, topo: &mut Topology, scale: f64) -> (SolidId, SolidId) {
        let bar = make_cylinder(topo, self.bar * scale, LEN * scale).unwrap();
        let along_x = Mat4::translation(-LEN * scale / 2.0, 0.0, 0.0) * Mat4::rotation_y(PI / 2.0);
        transform_solid(topo, bar, &along_x).unwrap();
        let drill = make_cylinder(topo, self.drill * scale, LEN * scale).unwrap();
        let place =
            self.drill_frame() * Mat4::translation(0.0, self.offset * scale, -LEN * scale / 2.0);
        transform_solid(topo, drill, &place).unwrap();
        (bar, drill)
    }
}

const CROSSES: [Cross; 6] = [
    // The minimal repro: two r 6 cylinders on perpendicular, crossing axes.
    Cross {
        name: "r6 x r6",
        bar: 6.0,
        drill: 6.0,
        offset: 0.0,
        tilt_x: 0.0,
        tilt_y: 0.0,
    },
    // The matrix cells: aligned, off-centre, tilted (about the bar's own
    // axis, a symmetry of the bar), equal radius.
    Cross {
        name: "aligned",
        bar: 10.0,
        drill: 6.0,
        offset: 0.0,
        tilt_x: 0.0,
        tilt_y: 0.0,
    },
    Cross {
        name: "off-centre",
        bar: 10.0,
        drill: 6.0,
        offset: 3.0,
        tilt_x: 0.0,
        tilt_y: 0.0,
    },
    Cross {
        name: "tilted",
        bar: 10.0,
        drill: 6.0,
        offset: 0.0,
        tilt_x: 0.349_065_850_398_865_9,
        tilt_y: 0.0,
    },
    Cross {
        name: "equal-radius",
        bar: 10.0,
        drill: 10.0,
        offset: 0.0,
        tilt_x: 0.0,
        tilt_y: 0.0,
    },
    // Skew to the bar: the drill leans 20° toward the bar's axis.
    Cross {
        name: "skew",
        bar: 10.0,
        drill: 6.0,
        offset: 2.0,
        tilt_x: 0.0,
        tilt_y: 0.349_065_850_398_865_9,
    },
];

fn assert_solid(topo: &Topology, solid: SolidId, expected: f64, scale: f64, what: &str) -> f64 {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{what}: invalid {:?}", report.issues);
    for deflection in [0.1 * scale, 0.01 * scale] {
        let mesh = tessellate_solid(topo, solid, deflection).unwrap();
        assert_eq!(
            boundary_edge_count(&mesh),
            0,
            "{what}: open mesh at {deflection}"
        );
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "{what}: non-manifold at {deflection}"
        );
    }
    let mass = mass_properties(topo, solid).unwrap().mass;
    assert!(
        (mass - expected).abs() <= SECTION_FIT * expected,
        "{what}: volume {mass} vs oracle {expected}"
    );
    mass
}

#[test]
fn oracle_matches_the_steinmetz_closed_form() {
    let steinmetz = CROSSES[4].overlap();
    let exact = 16.0 * 1000.0 / 3.0;
    assert!(
        (steinmetz - exact).abs() <= 1e-9 * exact,
        "{steinmetz} vs {exact}"
    );
}

/// `fuseAll` of two crossing cylinders, in both orders and at three scales,
/// is the valid, watertight union, the same body the pairwise `fuse` builds
/// from the same operand order.
#[test]
fn fuse_all_unions_crossing_cylinders() {
    for cross in CROSSES {
        let overlap = cross.overlap();
        let union = PI * (cross.bar * cross.bar + cross.drill * cross.drill) * LEN - overlap;
        for scale in SCALES {
            let s3 = scale * scale * scale;
            for swap in [false, true] {
                let what = format!("{} scale {scale} swap {swap}", cross.name);
                let mut pair = Topology::new();
                let (bar, drill) = cross.place(&mut pair, scale);
                let (a, b) = if swap { (drill, bar) } else { (bar, drill) };
                let fused = boolean(&mut pair, BooleanOp::Fuse, a, b).unwrap();
                let pairwise = mass_properties(&pair, fused).unwrap().mass;
                let mut topo = Topology::new();
                let (bar, drill) = cross.place(&mut topo, scale);
                let order = if swap { [drill, bar] } else { [bar, drill] };
                let solid = fuse_solids(&mut topo, &order)
                    .unwrap_or_else(|e| panic!("{what}: fuseAll refused: {e}"));
                assert!(
                    solid_faces(&topo, solid).unwrap().len() > 6,
                    "{what}: the cylinders' faces were not split"
                );
                let mass = assert_solid(&topo, solid, union * s3, scale, &what);
                assert!(
                    (mass - pairwise).abs() <= 1e-9 * pairwise,
                    "{what}: fuseAll {mass} vs fuse {pairwise}"
                );
            }
        }
    }
}

/// A bar crossed by two drills fuses as one cluster.
#[test]
fn fuse_all_unions_a_bar_with_two_drills() {
    let cross = CROSSES[1];
    let overlap = cross.overlap();
    let mut topo = Topology::new();
    let (bar, drill) = cross.place(&mut topo, 1.0);
    let second = make_cylinder(&mut topo, cross.drill, LEN).unwrap();
    transform_solid(&mut topo, second, &Mat4::translation(18.0, 0.0, -LEN / 2.0)).unwrap();
    let union =
        PI * (cross.bar * cross.bar + 2.0 * cross.drill * cross.drill) * LEN - 2.0 * overlap;
    let solid = fuse_solids(&mut topo, &[bar, drill, second]).unwrap();
    assert_solid(&topo, solid, union, 1.0, "two drills");
}

/// Drills that miss or only touch the bar stay separate lumps: the fix
/// sends only unclassified pairs down the boolean path.
#[test]
fn fuse_all_keeps_a_clear_drill_apart() {
    let mut topo = Topology::new();
    let (bar, _) = CROSSES[1].place(&mut topo, 1.0);
    let clear = make_cylinder(&mut topo, 6.0, LEN).unwrap();
    transform_solid(&mut topo, clear, &Mat4::translation(0.0, 20.0, -LEN / 2.0)).unwrap();
    let solid = fuse_solids(&mut topo, &[bar, clear]).unwrap();
    let expected = PI * (100.0 + 36.0) * LEN;
    let mass = mass_properties(&topo, solid).unwrap().mass;
    assert!(
        (mass - expected).abs() <= 1e-9 * expected,
        "{mass} vs {expected}"
    );
    assert!(validate_solid(&topo, solid).unwrap().is_valid());
}
