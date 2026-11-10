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
//! The file uses only `ray_torus`, so it runs unchanged against older
//! trees; `LINE_TORUS_REPORT=1` prints the per-family table instead of
//! failing, for comparing them. `LINE_TORUS_SEEDS=n` runs the families from
//! `n` further seeds as well (a soak; CI runs the one fixed seed).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod line_torus_reference;

use std::collections::BTreeMap;
use std::fmt::Write as _;

use line_torus_reference::{Tally, cases, cases_seeded, judge, reference};
use remus_math::ray::ray_torus;

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
        "family                 lines  roots  own   lost phantom mislabel outside inaccurate merged uncertified\n",
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
            t.uncertified
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
        total.uncertified += t.uncertified;
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
        total.uncertified
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
