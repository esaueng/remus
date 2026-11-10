//! `ray_torus` against an independent double-double reference, over seeded
//! line families: random chords, lines `0` to `1e-3` torus sizes either side
//! of exterior, interior, asymptotic (triple) and quadruple tangencies,
//! large thin tori (`R / r` up to 3000) with the origin a hair off the tube,
//! tori up to `1e12` from the world origin, ray origins up to `1e4` torus
//! sizes away, and directions `1e-5` to `1e4` long, at torus sizes `1e-3`,
//! `1` and `1e6`.
//!
//! On every line: no reference root is lost, no hit is a phantom, each root
//! reported on its own is within its stated accuracy, every root a window
//! holds is inside it, no crossing is labelled a touch and no touch a
//! crossing, and no two roots farther apart than `1e-12` torus sizes are
//! merged.
//!
//! That test uses only `ray_torus`, so it runs unchanged against older
//! trees; `LINE_TORUS_REPORT=1` prints the per-family table instead of
//! failing, for comparing them. `LINE_TORUS_SEEDS=n` runs the families from
//! `n` further seeds as well (a soak; CI runs the one fixed seed).
//!
//! # The contract of the caller's parameter
//!
//! `windows_keep_the_caller_parameter_contract` holds the windows of
//! `intersect_line_torus_contacts` to what they promise about `f64` values
//! of the parameter, over the families above and those that put the
//! parameter at its limit: tori `1e9` to `1e15` out with tubes a fraction
//! of a spacing of `t` to thirty spacings across, grazing lines `1e3` to
//! `1e12` away whose chords run from below one spacing of `t` to many,
//! tubes thinner than a spacing, and directions `1e-12` to `1e12` long.
//! The quartic's sign at each window end is evaluated *exactly*, in dyadic
//! big-number arithmetic, so the test reads the true sign at the very
//! `f64` the solver returned:
//!
//! - a resolved window has opposite exact signs at its ends, holds exactly
//!   one reference root, and no two windows meet;
//! - an unresolved window's label is the exact parity of its end signs
//!   (and of the reference roots inside), and it holds no two roots that
//!   are separable: more than `1e-12` torus sizes apart with two `f64`
//!   values of the parameter strictly between them.
//!
//! It needs `intersect_line_torus_contacts`, so it runs on trees from the
//! certified solve on. `window_contract_soak` (ignored; run it with
//! `--ignored`) repeats it from `LINE_TORUS_SEEDS` further seeds, twelve by
//! default.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod line_torus_reference;

use std::collections::BTreeMap;
use std::fmt::Write as _;

use line_torus_reference::exact::{Big, quartic_sign};
use line_torus_reference::{
    Case, Tally, WindowTally, caller_cases, caller_cases_seeded, cases, cases_seeded, judge,
    judge_windows, reference,
};
use remus_math::analytic_intersection::intersect_line_torus_contacts;
use remus_math::ray::ray_torus;
use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point3, Vec3};

#[test]
fn ray_torus_matches_the_reference_on_every_family() {
    let mut cases = cases();
    assert!(cases.len() >= 4000, "{} lines", cases.len());
    let extra_seeds: u64 = std::env::var("LINE_TORUS_SEEDS")
        .ok()
        .and_then(|n| n.parse().ok())
        .unwrap_or(0);
    for k in 0..extra_seeds {
        cases.extend(cases_seeded(
            0x9E37_79B9_7F4A_7C15_u64.wrapping_mul(k + 1) ^ 0x1638,
        ));
    }
    let mut tallies: BTreeMap<&'static str, Tally> = BTreeMap::new();
    let mut examples: Vec<String> = Vec::new();
    let (mut ambiguous, mut empty) = (0_usize, 0_usize);
    for case in &cases {
        let truth = reference(&case.torus, case.origin, case.dir);
        ambiguous += truth.ambiguous.len();
        empty += usize::from(truth.roots.is_empty() && truth.ambiguous.is_empty());
        let hits = ray_torus(case.origin, case.dir, &case.torus);
        let tally = tallies.entry(case.family).or_default();
        if let Some(what) = judge(&hits, &truth, tally)
            && examples.len() < 12
        {
            examples.push(format!(
                "{}: {what}\n  torus {:?}\n  origin {:?} dir {:?}\n  truth {:?} ambiguous {:?}\n  hits {hits:?}",
                case.family,
                case.torus,
                case.origin,
                case.dir,
                truth.roots.iter().map(|r| r.value()).collect::<Vec<_>>(),
                truth.ambiguous,
            ));
        }
    }
    let mut table = String::from(
        "family                 lines  roots  own   lost phantom mislabel outside inaccurate merged   unbounded\n",
    );
    let mut total = Tally::default();
    for (family, t) in &tallies {
        writeln!(
            table,
            "{family:<22} {:>5} {:>6} {:>5} {:>5} {:>7} {:>8} {:>7} {:>10} {:>6} {:>11}",
            t.lines,
            t.roots,
            t.separate,
            t.lost,
            t.phantom,
            t.mislabelled,
            t.outside_window,
            t.inaccurate,
            t.merged,
            t.unbounded
        )
        .unwrap();
        total.lines += t.lines;
        total.roots += t.roots;
        total.separate += t.separate;
        total.lost += t.lost;
        total.phantom += t.phantom;
        total.mislabelled += t.mislabelled;
        total.outside_window += t.outside_window;
        total.inaccurate += t.inaccurate;
        total.merged += t.merged;
        total.unbounded += t.unbounded;
    }
    writeln!(
        table,
        "{:<22} {:>5} {:>6} {:>5} {:>5} {:>7} {:>8} {:>7} {:>10} {:>6} {:>11}",
        "total",
        total.lines,
        total.roots,
        total.separate,
        total.lost,
        total.phantom,
        total.mislabelled,
        total.outside_window,
        total.inaccurate,
        total.merged,
        total.unbounded
    )
    .unwrap();
    writeln!(
        table,
        "{ambiguous} contacts the reference leaves ambiguous, {empty} lines with no contact"
    )
    .unwrap();
    if std::env::var_os("LINE_TORUS_REPORT").is_some() {
        #[allow(clippy::print_stdout)]
        {
            println!("{table}");
            for example in &examples {
                println!("{example}\n");
            }
        }
        return;
    }
    // The fixture covers what it claims: plenty of roots, of misses, and
    // every family populated.
    assert!(total.roots >= 6000 && empty >= 300, "{table}");
    assert!(tallies.values().all(|t| t.lines >= 60), "{table}");
    assert_eq!(total.failures(), 0, "\n{table}\n{}", examples.join("\n\n"));
    // Every root came back on its own, none in a merged window.
    assert_eq!(total.separate, total.roots, "{table}");
}

/// The exact arithmetic behind the window judge: integer identities, a sum
/// no `f64` holds, and the quartic at closed-form roots and between them.
#[test]
fn exact_arithmetic_and_quartic_signs() {
    let of = Big::of;
    assert_eq!(of(3.0).mul(&of(-7.0)).sub(&of(-21.0)).sign(), 0);
    assert_eq!(of(0.1).add(&of(0.2)).sub(&of(0.3)).sign(), 1);
    // (2^60 + 1)² − 2^120 − 2^61 = 1, far past an f64's 53 bits.
    let x = of(2.0_f64.powi(60)).add(&of(1.0));
    let rest = of(2.0_f64.powi(120)).add(&of(2.0_f64.powi(61)));
    assert_eq!(x.square().sub(&rest).sub(&of(1.0)).sign(), 0);
    assert_eq!(x.square().sub(&rest).sign(), 1);
    // (1 + 2^-52)(1 − 2^-52) − 1 = −2^-104, which an `f64` rounds to zero.
    let (up, down) = (of(1.0 + f64::EPSILON), of(1.0 - f64::EPSILON));
    assert_eq!(up.mul(&down).sub(&of(1.0)).sign(), -1);
    assert_eq!(
        up.mul(&down)
            .sub(&of(1.0))
            .add(&of(2.0_f64.powi(-104)))
            .sign(),
        0
    );
    assert_eq!(Big::sum(&[1e16, 1.0, -1e16, -1.0]).sign(), 0);
    assert_eq!(of(5e-324).mul(&of(2.0)).sub(&of(1e-323)).sign(), 0);
    assert_eq!(of(-0.0).sign(), 0);

    let torus = ToroidalSurface::new(Point3::new(1e15, 0.0, 0.0), 1.5, 0.5).unwrap();
    let (origin, dir) = (Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
    let sign = |t: f64| quartic_sign(&torus, origin, dir, &[t]);
    for root in [-2.0, -1.0, 1.0, 2.0] {
        assert_eq!(sign(1e15 + root), 0);
    }
    let between: Vec<i8> = [-2.5, -1.5, 0.0, 1.5, 2.5]
        .iter()
        .map(|x| sign(1e15 + x))
        .collect();
    assert_eq!(between, vec![1, -1, 1, -1, 1]);
    // A double-double parameter 2^-60 either side of the root at 1e15 − 1,
    // where the tube's inner wall is: inside the tube below it.
    let tiny = 2.0_f64.powi(-60);
    assert_eq!(quartic_sign(&torus, origin, dir, &[1e15 - 1.0, -tiny]), -1);
    assert_eq!(quartic_sign(&torus, origin, dir, &[1e15 - 1.0, tiny]), 1);
}

/// Every family's windows against the contract, with the first few
/// violations spelt out.
fn window_tallies(cases: &[Case]) -> (BTreeMap<&'static str, WindowTally>, Vec<String>) {
    let mut tallies: BTreeMap<&'static str, WindowTally> = BTreeMap::new();
    let mut examples = Vec::new();
    for case in cases {
        let truth = reference(&case.torus, case.origin, case.dir);
        let contacts = intersect_line_torus_contacts(&case.torus, case.origin, case.dir);
        let tally = tallies.entry(case.family).or_default();
        if let Some(what) = judge_windows(case, &contacts, &truth, tally)
            && examples.len() < 12
        {
            examples.push(format!(
                "{}: {what}\n  torus {:?}\n  origin {:?} dir {:?}\n  truth {:?} ambiguous {:?}\n  contacts {contacts:?}",
                case.family,
                case.torus,
                case.origin,
                case.dir,
                truth.roots.iter().map(|r| r.value()).collect::<Vec<_>>(),
                truth.ambiguous,
            ));
        }
    }
    (tallies, examples)
}

fn window_table(tallies: &BTreeMap<&'static str, WindowTally>) -> (String, WindowTally) {
    let mut table = String::from(
        "family                 lines  roots windows resolved fused stretch | disorder  unbnd end-root res-sign res-count unres-parity lost separable | unjudged\n",
    );
    let mut total = WindowTally::default();
    let row = |name: &str, t: &WindowTally| {
        format!(
            "{name:<22} {:>5} {:>6} {:>7} {:>8} {:>5} {:>7} | {:>8} {:>6} {:>8} {:>8} {:>9} {:>12} {:>4} {:>9} | {:>8}\n",
            t.lines,
            t.roots,
            t.contacts,
            t.resolved,
            t.fused,
            t.stretches,
            t.disorder,
            t.unbounded,
            t.root_at_end,
            t.resolved_parity,
            t.resolved_count,
            t.unresolved_parity,
            t.lost,
            t.separable,
            t.unjudged,
        )
    };
    for (family, t) in tallies {
        table.push_str(&row(family, t));
        total.add(t);
    }
    table.push_str(&row("total", &total));
    (table, total)
}

#[test]
fn windows_keep_the_caller_parameter_contract() {
    let mut all = cases();
    let limit = caller_cases();
    let limit_families: Vec<&'static str> = limit.iter().map(|c| c.family).collect();
    all.extend(limit);
    let (tallies, examples) = window_tallies(&all);
    let (table, total) = window_table(&tallies);
    if std::env::var_os("LINE_TORUS_REPORT").is_some() {
        #[allow(clippy::print_stdout)]
        {
            println!("{table}");
            for example in &examples {
                println!("{example}\n");
            }
        }
        return;
    }
    assert!(total.lines >= 7000 && total.roots >= 12_000, "{table}");
    assert!(tallies.values().all(|t| t.lines >= 60), "{table}");
    assert_eq!(
        total.violations(),
        0,
        "\n{table}\n{}",
        examples.join("\n\n")
    );
    // Hardly a resolved window sits where the reference cannot count.
    assert!(total.unjudged * 1000 <= total.resolved, "{table}");
    // The families at the parameter's limit are on both sides of it: roots
    // it resolves and roots it cannot, in every group.
    for group in ["far-centre-ulps/", "far-origin-graze/", "far-origin-thin/"] {
        let mut sum = WindowTally::default();
        for (family, t) in &tallies {
            if family.starts_with(group) {
                sum.add(t);
            }
        }
        assert!(sum.fused >= 40 && sum.resolved >= 200, "{group}\n{table}");
    }
    assert!(limit_families.iter().all(|f| tallies.contains_key(f)));
}

/// The contract over the families drawn from further seeds: twelve by
/// default, `LINE_TORUS_SEEDS` to choose. Not run in CI.
#[test]
#[ignore = "soak: minutes; run with --ignored"]
fn window_contract_soak() {
    let seeds: u64 = std::env::var("LINE_TORUS_SEEDS")
        .ok()
        .and_then(|n| n.parse().ok())
        .unwrap_or(12);
    let mut total = WindowTally::default();
    let mut tallies: BTreeMap<&'static str, WindowTally> = BTreeMap::new();
    let mut examples = Vec::new();
    for k in 0..seeds {
        let seed = 0x9E37_79B9_7F4A_7C15_u64.wrapping_mul(k + 1) ^ 0x1638_0005;
        let mut all = cases_seeded(seed);
        all.extend(caller_cases_seeded(seed ^ 0xFDA8_173A));
        let (found, mut spelt) = window_tallies(&all);
        for (family, t) in &found {
            tallies.entry(family).or_default().add(t);
            total.add(t);
        }
        examples.append(&mut spelt);
        examples.truncate(12);
    }
    let (table, _) = window_table(&tallies);
    #[allow(clippy::print_stdout)]
    {
        println!("{seeds} seeds\n{table}");
    }
    assert!(total.lines >= 9000, "{table}");
    assert_eq!(
        total.violations(),
        0,
        "\n{table}\n{}",
        examples.join("\n\n")
    );
}
