//! A differential sweep of round cutters against holed planar faces, for
//! comparing two revisions of the kernel.
//!
//! Each of the three ignored tests (one per scale) runs 1260 booleans
//! through the public `boolean` and writes one line per case, its outcome.
//! Run it on the base revision to write the dumps, and on the revision
//! under test with those dumps as its baseline:
//!
//! ```text
//! REMUS_SWEEP_DUMP=/tmp/sweep-base cargo nextest run --cargo-profile ci-test \
//!     -p remus-operations --test sweep_shelled_wall_rim_notch --run-ignored all
//! REMUS_SWEEP_BASELINE=/tmp/sweep-base REMUS_SWEEP_DUMP=/tmp/sweep-new cargo nextest run \
//!     --cargo-profile ci-test -p remus-operations --test sweep_shelled_wall_rim_notch \
//!     --run-ignored all
//! ```
//!
//! With a baseline the run is a pass/fail test: it prints how every case
//! moved between the two revisions and FAILS when a case that was exact on
//! the baseline is anything else now, or one that was refused there is
//! accepted with a defect now. Without one it only writes and tallies.
//!
//! The file uses only the public API, so it compiles unchanged on a
//! revision that predates it.
//!
//! The cases: a vertical cylinder against the shelled box (seven radii,
//! sixteen positions on and off the wall and round the cavity's corner,
//! five depths, cut and fuse: 1120), against a bored plate (seven
//! positions on and off the bore's rim, three radii, two depths: 84) and
//! against a tube (56).
//!
//! An outcome is one of:
//!
//! - `exact`: accepted, with the closed-form volume to 1e-9, a clean
//!   `validate_solid`, a watertight mesh, and only plane and cylinder
//!   faces;
//! - `defect(…)`: accepted, with the listed checks failing;
//! - `refused(…)`: an error, by its variant.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::disallowed_methods,
    clippy::print_stdout,
    missing_docs
)]

use std::f64::consts::PI;
use std::fmt::Write as _;

use remus_math::mat::Mat4;
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::shell_op::shell;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

// ── Closed forms ────────────────────────────────────────────────────────

/// Area of the disc of radius `r` about the origin inside
/// `[x0, x1] × [y0, y1]`.
fn disc_in_rect(r: f64, x0: f64, x1: f64, y0: f64, y1: f64) -> f64 {
    let (a, b) = (x0.max(-r), x1.min(r));
    if a >= b {
        return 0.0;
    }
    let mut cuts = vec![a, b];
    for y in [y0, y1] {
        if y.abs() < r {
            let x = (r * r - y * y).sqrt();
            cuts.extend([-x, x].into_iter().filter(|c| *c > a && *c < b));
        }
    }
    cuts.sort_by(f64::total_cmp);
    let half_disc = |x: f64| {
        0.5 * (x * (r * r - x * x).max(0.0).sqrt() + r * r * (x / r).clamp(-1.0, 1.0).asin())
    };
    let mut area = 0.0;
    for w in cuts.windows(2) {
        let (p, q) = (w[0], w[1]);
        let mid = 0.5 * (p + q);
        let s = (r * r - mid * mid).max(0.0).sqrt();
        let (top_on_circle, bottom_on_circle) = (s < y1, -s > y0);
        let top = if top_on_circle { s } else { y1 };
        let bottom = if bottom_on_circle { -s } else { y0 };
        if top <= bottom {
            continue;
        }
        let arc = half_disc(q) - half_disc(p);
        area += if top_on_circle { arc } else { y1 * (q - p) };
        area -= if bottom_on_circle { -arc } else { y0 * (q - p) };
    }
    area
}

/// Area common to discs of radii `r` and `big` whose centres are `d` apart.
fn lens(r: f64, big: f64, d: f64) -> f64 {
    if d >= r + big {
        return 0.0;
    }
    if d <= (big - r).abs() {
        return PI * r.min(big).powi(2);
    }
    r * r * ((d * d + r * r - big * big) / (2.0 * d * r)).acos()
        + big * big * ((d * d + big * big - r * r) / (2.0 * d * big)).acos()
        - 0.5 * ((-d + r + big) * (d + r - big) * (d - r + big) * (d + r + big)).sqrt()
}

/// Volume a vertical cylinder shares with the shelled box (unit scale): a
/// 100 × 100 × 50 box with a 2 thick floor and walls.
fn in_shell(r: f64, cx: f64, cy: f64, z0: f64, h: f64) -> f64 {
    let footprint = disc_in_rect(r, -cx, 100.0 - cx, -cy, 100.0 - cy);
    let wall = footprint - disc_in_rect(r, 2.0 - cx, 98.0 - cx, 2.0 - cy, 98.0 - cy);
    let (lo, hi) = (z0.max(0.0), (z0 + h).min(50.0));
    (hi.min(2.0) - lo).max(0.0) * footprint + (hi - lo.max(2.0)).max(0.0) * wall
}

fn expected(op: BooleanOp, body: f64, tool: f64, common: f64) -> f64 {
    match op {
        BooleanOp::Cut => body - common,
        BooleanOp::Fuse => body + tool - common,
        BooleanOp::Intersect => common,
    }
}

// ── Bodies ──────────────────────────────────────────────────────────────

fn cylinder_at(
    topo: &mut Topology,
    s: f64,
    r: f64,
    (cx, cy): (f64, f64),
    (z0, h): (f64, f64),
) -> SolidId {
    let solid = make_cylinder(topo, r * s, h * s).unwrap();
    transform_solid(topo, solid, &Mat4::translation(cx * s, cy * s, z0 * s)).unwrap();
    solid
}

fn shelled_box(topo: &mut Topology, s: f64) -> SolidId {
    let solid = make_box(topo, 100.0 * s, 100.0 * s, 50.0 * s).unwrap();
    let top = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| {
            let face = topo.face(f).unwrap();
            matches!(face.surface(), FaceSurface::Plane { normal, .. }
                if normal.z() > 0.9 && !face.is_reversed())
        })
        .unwrap();
    shell(topo, solid, 2.0 * s, &[top]).unwrap()
}

fn bored_plate(topo: &mut Topology, s: f64) -> SolidId {
    let plate = make_box(topo, 40.0 * s, 40.0 * s, 10.0 * s).unwrap();
    let bore = cylinder_at(topo, s, 8.0, (20.0, 20.0), (-10.0, 30.0));
    boolean(topo, BooleanOp::Cut, plate, bore).unwrap()
}

fn tube(topo: &mut Topology, s: f64) -> SolidId {
    let outer = make_cylinder(topo, 10.0 * s, 10.0 * s).unwrap();
    let bore = cylinder_at(topo, s, 5.0, (0.0, 0.0), (-10.0, 30.0));
    boolean(topo, BooleanOp::Cut, outer, bore).unwrap()
}

// ── Outcomes ────────────────────────────────────────────────────────────

/// What `result` is, held to the closed-form `volume` (scaled).
fn outcome(
    topo: &Topology,
    result: Result<SolidId, OperationsError>,
    volume: f64,
    s: f64,
) -> String {
    let solid = match result {
        Ok(solid) => solid,
        Err(OperationsError::ExactOnlyUnattainable) => return "refused(exact-only)".to_owned(),
        Err(other) => {
            let text = other.to_string();
            return format!("refused({})", text.split(':').next().unwrap_or(&text));
        }
    };
    let mut defects: Vec<String> = Vec::new();
    let faces = solid_faces(topo, solid).unwrap();
    let other = faces
        .iter()
        .filter(|&&f| {
            !matches!(
                topo.face(f).unwrap().surface().type_tag(),
                "plane" | "cylinder"
            )
        })
        .count();
    if other > 0 || faces.len() > 64 {
        defects.push(format!(
            "census {} faces, {other} not plane or cylinder",
            faces.len()
        ));
    }
    match validate_solid(topo, solid) {
        Ok(report) if report.is_valid() => {}
        Ok(report) => defects.push(format!("validation {} issue(s)", report.issues.len())),
        Err(error) => defects.push(format!("validation error {error}")),
    }
    match tessellate_solid(topo, solid, 0.01 * s) {
        Ok(mesh) => {
            let (open, branching) = (boundary_edge_count(&mesh), non_manifold_edge_count(&mesh));
            if open > 0 || branching > 0 {
                defects.push(format!("mesh {open} open, {branching} non-manifold edges"));
            }
        }
        Err(error) => defects.push(format!("mesh error {error}")),
    }
    match solid_volume(topo, solid, 0.01 * s) {
        Ok(measured) => {
            let off = (measured - volume).abs() / volume;
            if off > 1e-9 {
                defects.push(format!("volume off by {off:.1e}"));
            }
        }
        Err(error) => {
            let text = error.to_string();
            defects.push(format!(
                "volume error {}",
                text.split(':').next().unwrap_or(&text)
            ));
        }
    }
    if defects.is_empty() {
        "exact".to_owned()
    } else {
        format!("defect({})", defects.join("; "))
    }
}

const OPS: [BooleanOp; 2] = [BooleanOp::Cut, BooleanOp::Fuse];

/// Every case at scale `s`, one line each: its name and its outcome.
fn sweep(s: f64) -> String {
    let mut lines = String::new();
    let cube = s.powi(3);

    // The shelled box: on and off the wall at mid-side, and round the
    // cavity's corner.
    let positions = [
        (96.0, 50.0),
        (97.0, 50.0),
        (97.5, 50.0),
        (98.0, 50.0),
        (98.5, 50.0),
        (99.0, 50.0),
        (99.5, 50.0),
        (100.0, 50.0),
        (101.0, 50.0),
        (102.0, 50.0),
        (98.0, 98.0),
        (99.0, 99.0),
        (97.0, 97.0),
        (99.0, 97.0),
        (100.0, 100.0),
        (96.0, 98.0),
    ];
    let spans = [
        (-20.0, 100.0),
        (40.0, 40.0),
        (45.0, 20.0),
        (10.0, 30.0),
        (-10.0, 30.0),
    ];
    for r in [0.5, 1.0, 1.5, 2.0, 3.0, 6.0, 10.0] {
        for at in positions {
            for span in spans {
                for op in OPS {
                    let mut topo = Topology::new();
                    let body = shelled_box(&mut topo, s);
                    let tool = cylinder_at(&mut topo, s, r, at, span);
                    let common = in_shell(r, at.0, at.1, span.0, span.1);
                    let volume = expected(op, 57_632.0, PI * r * r * span.1, common) * cube;
                    let result = boolean(&mut topo, op, body, tool);
                    writeln!(
                        lines,
                        "shell {op:?} r={r} at={at:?} span={span:?} => {}",
                        outcome(&topo, result, volume, s)
                    )
                    .unwrap();
                }
            }
        }
    }

    // A bored plate: the cutter on, inside and outside the bore's rim.
    for r in [1.0, 3.0, 8.0] {
        for off in [0.0, 5.0, 7.0, 8.0, 9.0, 10.0, 11.0] {
            for span in [(-5.0, 30.0), (5.0, 30.0)] {
                for op in OPS {
                    let mut topo = Topology::new();
                    let body = bored_plate(&mut topo, s);
                    let tool = cylinder_at(&mut topo, s, r, (20.0 + off, 20.0), span);
                    let depth = (span.0 + span.1).min(10.0) - span.0.max(0.0);
                    let common = depth * (PI * r * r - lens(r, 8.0, off));
                    let body_volume = 16_000.0 - PI * 64.0 * 10.0;
                    let volume = expected(op, body_volume, PI * r * r * span.1, common) * cube;
                    let result = boolean(&mut topo, op, body, tool);
                    writeln!(
                        lines,
                        "bore {op:?} r={r} off={off} span={span:?} => {}",
                        outcome(&topo, result, volume, s)
                    )
                    .unwrap();
                }
            }
        }
    }

    // A tube: its end faces are annuli.
    for r in [1.0, 1.5, 2.0, 4.0] {
        for off in [3.0, 4.0, 5.0, 6.0, 7.0, 9.0, 10.0] {
            for op in OPS {
                let span = (-5.0, 30.0);
                let mut topo = Topology::new();
                let body = tube(&mut topo, s);
                let tool = cylinder_at(&mut topo, s, r, (off, 0.0), span);
                // The cutter's disc inside the outer circle, less the bore.
                let common = 10.0 * (lens(r, 10.0, off) - lens(r, 5.0, off));
                let body_volume = PI * 75.0 * 10.0;
                let volume = expected(op, body_volume, PI * r * r * span.1, common) * cube;
                let result = boolean(&mut topo, op, body, tool);
                writeln!(
                    lines,
                    "tube {op:?} r={r} off={off} => {}",
                    outcome(&topo, result, volume, s)
                )
                .unwrap();
            }
        }
    }
    lines
}

/// An outcome's class: `exact`, `defect` or `refused`.
fn class(line: &str) -> &'static str {
    let outcome = line.rsplit("=> ").next().unwrap_or_default();
    if outcome.starts_with("exact") {
        "exact"
    } else if outcome.starts_with("defect(") {
        "defect"
    } else {
        "refused"
    }
}

/// How every case moved from the baseline's outcome to this run's, and the
/// two moves that must not happen: `(the transitions with their counts, the
/// cases that were exact and are not, the cases that were refused and are
/// accepted with a defect)`.
fn compare(baseline: &str, lines: &str) -> (Vec<(String, usize)>, Vec<String>, Vec<String>) {
    let case = |line: &str| line.split(" => ").next().unwrap_or_default().to_owned();
    let before: std::collections::BTreeMap<String, &str> =
        baseline.lines().map(|line| (case(line), line)).collect();
    let mut moves: std::collections::BTreeMap<String, usize> = std::collections::BTreeMap::new();
    let (mut lost, mut defects) = (Vec::new(), Vec::new());
    for line in lines.lines() {
        let Some(was) = before.get(&case(line)) else {
            panic!("the baseline has no case {:?}", case(line));
        };
        let (from, to) = (class(was), class(line));
        // An accepted defect is the same only when it is the same defect.
        let key = if from == "defect" && to == "defect" && was != &line {
            "defect -> defect (changed)".to_owned()
        } else {
            format!("{from} -> {to}")
        };
        *moves.entry(key).or_default() += 1;
        if from == "exact" && to != "exact" {
            lost.push(format!("{was}\n      now {line}"));
        }
        if from == "refused" && to == "defect" {
            defects.push(format!("{was}\n      now {line}"));
        }
    }
    (moves.into_iter().collect(), lost, defects)
}

fn run(s: f64) {
    let lines = sweep(s);
    let tally = |prefix: &str| lines.lines().filter(|l| l.contains(prefix)).count();
    println!(
        "scale {s}: {} cases, {} exact, {} accepted with a defect, {} refused",
        lines.lines().count(),
        tally("=> exact"),
        tally("=> defect("),
        tally("=> refused(")
    );
    if let Ok(path) = std::env::var("REMUS_SWEEP_DUMP") {
        std::fs::write(format!("{path}.{s}"), &lines).unwrap();
    }
    if let Ok(path) = std::env::var("REMUS_SWEEP_BASELINE") {
        let baseline = std::fs::read_to_string(format!("{path}.{s}")).unwrap();
        assert_eq!(baseline.lines().count(), lines.lines().count());
        let (moves, lost, defects) = compare(&baseline, &lines);
        for (transition, count) in &moves {
            println!("scale {s}: {transition}: {count}");
        }
        assert!(
            lost.is_empty(),
            "scale {s}: {} case(s) exact on the baseline are not exact now:\n  {}",
            lost.len(),
            lost.join("\n  ")
        );
        assert!(
            defects.is_empty(),
            "scale {s}: {} case(s) refused on the baseline are accepted with a defect now:\n  {}",
            defects.len(),
            defects.join("\n  ")
        );
    }
}

/// The comparison itself: a case that stays exact, one that becomes exact,
/// and the two moves that fail the sweep.
#[test]
fn the_comparison_names_the_moves_that_must_not_happen() {
    let baseline = "a => exact\nb => refused(X)\nc => exact\nd => refused(X)\ne => defect(v)\n";
    let now = "a => exact\nb => exact\nc => refused(Y)\nd => defect(mesh)\ne => defect(w)\n";
    let (moves, lost, defects) = compare(baseline, now);
    let count = |key: &str| {
        moves
            .iter()
            .find(|(transition, _)| transition == key)
            .map_or(0, |(_, count)| *count)
    };
    assert_eq!(count("exact -> exact"), 1);
    assert_eq!(count("refused -> exact"), 1);
    assert_eq!(count("exact -> refused"), 1);
    assert_eq!(count("refused -> defect"), 1);
    assert_eq!(count("defect -> defect (changed)"), 1);
    assert_eq!(lost.len(), 1, "{lost:?}");
    assert!(lost[0].starts_with("c => exact"), "{lost:?}");
    assert_eq!(defects.len(), 1, "{defects:?}");
    assert!(defects[0].starts_with("d => refused"), "{defects:?}");
    let (_, lost, defects) = compare(baseline, baseline);
    assert!(lost.is_empty() && defects.is_empty());
}

#[test]
#[ignore = "differential sweep: run on two revisions, the second with the first as its baseline"]
fn sweep_at_scale_0_1() {
    run(0.1);
}

#[test]
#[ignore = "differential sweep: run on two revisions, the second with the first as its baseline"]
fn sweep_at_scale_1() {
    run(1.0);
}

#[test]
#[ignore = "differential sweep: run on two revisions, the second with the first as its baseline"]
fn sweep_at_scale_10() {
    run(10.0);
}
