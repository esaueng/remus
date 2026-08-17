//! Union robustness census: fuse every primitive pair across a placement
//! lattice that deliberately includes the degenerate poses CAD users create
//! constantly — flush faces, tangencies, corner crossings, coaxial stacks —
//! and classify every result.
//!
//! Verdicts per case:
//! - `ok`        analytic result, all material probes Inside, volume sane
//! - `FACETED`   operands had curved faces, result has none (exactness lost)
//! - `DROPPED`   a probe point inside an operand classifies Outside the union
//! - `VOL`       union volume above the operand sum or below the larger operand
//! - `ERR`       the boolean returned an error
//!
//! Run: `cargo run --release --example union_robustness_census -p remus-operations`
#![allow(
    clippy::unwrap_used,
    clippy::print_stdout,
    clippy::too_many_lines,
    missing_docs
)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::transform::transform_solid;
use remus_operations::{measure, primitives};
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

#[derive(Clone, Copy)]
enum Prim {
    Box(f64, f64, f64),
    Cyl(f64, f64),
    Sphere(f64),
    Cone(f64, f64),
}

impl Prim {
    fn build(self, topo: &mut Topology) -> SolidId {
        match self {
            Self::Box(w, d, h) => primitives::make_box(topo, w, d, h).unwrap(),
            Self::Cyl(r, h) => primitives::make_cylinder(topo, r, h).unwrap(),
            Self::Sphere(r) => primitives::make_sphere(topo, r, 24).unwrap(),
            Self::Cone(r, h) => primitives::make_cone(topo, r, 0.0, h).unwrap(),
        }
    }
    fn curved(self) -> bool {
        !matches!(self, Self::Box(..))
    }
    /// Interior probe points in the primitive's own frame — well inside.
    fn probes(self) -> Vec<Point3> {
        match self {
            Self::Box(w, d, h) => vec![
                Point3::new(w * 0.5, d * 0.5, h * 0.5),
                Point3::new(w * 0.85, d * 0.85, h * 0.85),
                Point3::new(w * 0.15, d * 0.15, h * 0.15),
            ],
            Self::Cyl(r, h) => vec![
                Point3::new(0.0, 0.0, h * 0.5),
                Point3::new(r * 0.6, 0.0, h * 0.1),
                Point3::new(-r * 0.6, 0.0, h * 0.9),
            ],
            Self::Sphere(r) => vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(r * 0.6, 0.0, 0.0),
                Point3::new(0.0, -r * 0.6, 0.0),
            ],
            Self::Cone(r, h) => vec![
                Point3::new(0.0, 0.0, h * 0.2),
                Point3::new(r * 0.4, 0.0, h * 0.1),
            ],
        }
    }
}

struct Case {
    family: &'static str,
    label: String,
    a: Prim,
    b: Prim,
    move_b: (f64, f64, f64),
}

fn apply(p: Point3, m: (f64, f64, f64)) -> Point3 {
    Point3::new(p.x() + m.0, p.y() + m.1, p.z() + m.2)
}

fn run_case(case: &Case) -> &'static str {
    let mut topo = Topology::new();
    let a = case.a.build(&mut topo);
    let b = case.b.build(&mut topo);
    let mat = Mat4::translation(case.move_b.0, case.move_b.1, case.move_b.2);
    transform_solid(&mut topo, b, &mat).unwrap();

    let vol_a = measure::solid_volume(&topo, a, 0.1).unwrap_or(-1.0);
    let vol_b = measure::solid_volume(&topo, b, 0.1).unwrap_or(-1.0);

    let result = match boolean(&mut topo, BooleanOp::Fuse, a, b) {
        Ok(r) => r,
        Err(_) => return "ERR",
    };

    let faces = remus_topology::explorer::solid_faces(&topo, result).unwrap();
    let curved = faces
        .iter()
        .filter(|f| !matches!(topo.face(**f).unwrap().surface(), FaceSurface::Plane { .. }))
        .count();
    // A curved operand fully absorbed into a planar one correctly yields an
    // all-planar result with few faces. The mesh fallback's signature is the
    // combination: exactness lost AND the face count exploded.
    if (case.a.curved() || case.b.curved()) && curved == 0 && faces.len() > 15 {
        return "FACETED";
    }

    let opts = ClassifyOptions::default();
    for p in case.a.probes() {
        if matches!(
            classify_point(&topo, result, p, &opts),
            Ok(PointClassification::Outside)
        ) {
            return "DROPPED";
        }
    }
    for p in case.b.probes() {
        let p = apply(p, case.move_b);
        if matches!(
            classify_point(&topo, result, p, &opts),
            Ok(PointClassification::Outside)
        ) {
            return "DROPPED";
        }
    }

    let vol = measure::solid_volume(&topo, result, 0.1).unwrap_or(-1.0);
    let hi = (vol_a + vol_b) * 1.01 + 1.0;
    let lo = vol_a.max(vol_b) * 0.99 - 1.0;
    if vol > hi || vol < lo {
        return "VOL";
    }
    "ok"
}

fn main() {
    let bx = Prim::Box(30.0, 18.0, 24.0);
    let mut cases: Vec<Case> = Vec::new();

    // box ∪ cylinder: flush-cap variants x placement grid (incl. corner
    // protrusion, tangency, containment) — the Twinkly Otter family.
    for (ztag, cyl, cz) in [
        ("both-flush", Prim::Cyl(6.0, 24.0), 0.0),
        ("top-flush", Prim::Cyl(6.0, 30.0), -6.0),
        ("bottom-flush", Prim::Cyl(6.0, 30.0), 0.0),
        ("no-flush", Prim::Cyl(6.0, 30.0), -3.0),
    ] {
        for cx in [-6.0, -4.0, 0.0, 4.0, 15.0, 24.0, 30.0, 36.0] {
            for cy in [0.0, 4.0, 9.0] {
                cases.push(Case {
                    family: "box∪cyl",
                    label: format!("{ztag} cx={cx} cy={cy}"),
                    a: bx,
                    b: cyl,
                    move_b: (cx, cy, cz),
                });
            }
        }
    }

    // box ∪ box: flush, edge-touch, corner-touch, contained, generic
    for (tag, m) in [
        ("generic", (15.0, 9.0, 12.0)),
        ("face-flush", (30.0, 0.0, 0.0)),
        ("face-overlap-half", (15.0, 0.0, 0.0)),
        ("edge-touch", (30.0, 18.0, 0.0)),
        ("corner-touch", (30.0, 18.0, 24.0)),
        ("contained", (5.0, 4.0, 6.0)),
        ("flush-top", (10.0, 5.0, 24.0)),
        ("cross", (-5.0, 5.0, 10.0)),
    ] {
        let small = Prim::Box(12.0, 8.0, 10.0);
        cases.push(Case {
            family: "box∪box",
            label: tag.into(),
            a: bx,
            b: if tag == "face-flush" || tag == "edge-touch" || tag == "corner-touch" {
                bx
            } else {
                small
            },
            move_b: m,
        });
    }

    // box ∪ sphere: tangent-face, straddle-face, straddle-corner, contained
    for (tag, m) in [
        ("contained", (15.0, 9.0, 12.0)),
        ("straddle-face", (0.0, 9.0, 12.0)),
        ("straddle-edge", (0.0, 0.0, 12.0)),
        ("straddle-corner", (0.0, 0.0, 24.0)),
        ("tangent-face", (-8.0, 9.0, 12.0)),
        ("half-out-top", (15.0, 9.0, 24.0)),
    ] {
        cases.push(Case {
            family: "box∪sphere",
            label: tag.into(),
            a: bx,
            b: Prim::Sphere(8.0),
            move_b: m,
        });
    }

    // cyl ∪ cyl: coaxial stacks (flush + overlapping), offset-parallel, tangent
    for (tag, b, m) in [
        (
            "coaxial-stack-flush",
            Prim::Cyl(6.0, 10.0),
            (0.0, 0.0, 24.0),
        ),
        ("coaxial-overlap", Prim::Cyl(6.0, 10.0), (0.0, 0.0, 20.0)),
        (
            "coaxial-wider-stack",
            Prim::Cyl(9.0, 10.0),
            (0.0, 0.0, 24.0),
        ),
        ("parallel-overlap", Prim::Cyl(6.0, 24.0), (8.0, 0.0, 0.0)),
        ("parallel-tangent", Prim::Cyl(6.0, 24.0), (12.0, 0.0, 0.0)),
        (
            "parallel-overlap-taller",
            Prim::Cyl(6.0, 30.0),
            (8.0, 0.0, -3.0),
        ),
        (
            "parallel-overlap-flush",
            Prim::Cyl(6.0, 24.0),
            (8.0, 0.0, 0.0),
        ),
    ] {
        cases.push(Case {
            family: "cyl∪cyl",
            label: tag.into(),
            a: Prim::Cyl(6.0, 24.0),
            b,
            move_b: m,
        });
    }

    // box ∪ cone: base-flush boss, protruding, straddling a face
    for (tag, m) in [
        ("boss-on-top", (15.0, 9.0, 24.0)),
        ("base-flush-inside", (15.0, 9.0, 0.0)),
        ("straddle-side", (0.0, 9.0, 6.0)),
        ("straddle-side-base-flush", (0.0, 9.0, 0.0)),
    ] {
        cases.push(Case {
            family: "box∪cone",
            label: tag.into(),
            a: bx,
            b: Prim::Cone(6.0, 15.0),
            move_b: m,
        });
    }

    let mut totals: std::collections::BTreeMap<&str, (usize, usize)> =
        std::collections::BTreeMap::default();
    let mut failures: Vec<String> = Vec::new();
    for case in &cases {
        let verdict = run_case(case);
        let entry = totals.entry(case.family).or_insert((0, 0));
        entry.1 += 1;
        if verdict == "ok" {
            entry.0 += 1;
        } else {
            failures.push(format!("{:10} {} [{}]", case.family, case.label, verdict));
        }
    }

    println!("=== union robustness census ===");
    let mut ok_all = 0;
    let mut n_all = 0;
    for (family, (ok, n)) in &totals {
        println!("{family:12} {ok:3}/{n:3}");
        ok_all += ok;
        n_all += n;
    }
    println!("{:12} {ok_all:3}/{n_all:3}", "TOTAL");
    println!("\n=== failures ===");
    for f in &failures {
        println!("{f}");
    }
}
