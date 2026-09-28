//! STEP import benchmarks for PERF-I01.
//!
//! Covers the shapes the borrowed-span parser must win on: a small writer
//! round-trip, the shipped NURBS-heavy hammer-holder fixture, a synthetic
//! repeated-entity file (indexing cost), a synthetic large rational control
//! net (attribute cost), and malformed-input rejection cost.

#![allow(missing_docs, clippy::expect_used, clippy::unwrap_used)]

use std::fmt::Write as _;

use criterion::{Criterion, criterion_group, criterion_main};
use remus_io::ImportLimits;
use remus_topology::Topology;
use remus_topology::test_utils::make_unit_cube_non_manifold;

fn small_box_step() -> String {
    let mut topo = Topology::new();
    let solid = make_unit_cube_non_manifold(&mut topo);
    remus_io::step::write_step(&topo, &[solid]).expect("box writes")
}

/// 5k repeated `CARTESIAN_POINT` entities plus the minimal unit context,
/// exercising statement scanning and entity indexing without topology work.
fn repeated_entities_step(count: usize) -> String {
    let mut s = String::from(
        "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('R'),'1');\nFILE_NAME('R','',(),(), '', '', '');\nFILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\nDATA;\n",
    );
    s.push_str("#1=GLOBAL_UNIT_ASSIGNED_CONTEXT((#2,#3));\n");
    s.push_str("#2=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));\n");
    s.push_str("#3=(PLANE_ANGLE_UNIT()NAMED_UNIT(*)SI_UNIT($,.RADIAN.));\n");
    for i in 0..count {
        let id = 10 + i;
        let _ = writeln!(s, "#{id}=CARTESIAN_POINT('P{id}',({id}.0,0.0,0.0));");
    }
    s.push_str("ENDSEC;\nEND-ISO-10303-21;");
    s
}

/// One rational B-spline curve with a large control net and weight row,
/// exercising attribute splitting and typed-measure parsing.
fn large_rational_net_step(points: usize) -> String {
    let mut s = String::from(
        "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('R'),'1');\nFILE_NAME('R','',(),(), '', '', '');\nFILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\nDATA;\n",
    );
    s.push_str("#1=GLOBAL_UNIT_ASSIGNED_CONTEXT((#2,#3));\n");
    s.push_str("#2=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));\n");
    s.push_str("#3=(PLANE_ANGLE_UNIT()NAMED_UNIT(*)SI_UNIT($,.RADIAN.));\n");
    for i in 0..points {
        let id = 100 + i;
        let _ = writeln!(s, "#{id}=CARTESIAN_POINT('',({id}.0,0.0,0.0));");
    }
    let cps: Vec<String> = (0..points).map(|i| format!("#{}", 100 + i)).collect();
    let weights = vec!["1.0"; points].join(",");
    let knots = vec!["0.0"; points].join(",");
    let mults = vec!["1"; points].join(",");
    let _ = writeln!(
        s,
        "#50=(B_SPLINE_CURVE(3,({}),.UNSPECIFIED.,.F.,.F.)B_SPLINE_CURVE_WITH_KNOTS((3),({mults}),({knots}),.UNSPECIFIED.)RATIONAL_B_SPLINE_CURVE(({weights})));",
        cps.join(",")
    );
    s.push_str("ENDSEC;\nEND-ISO-10303-21;");
    s
}

fn bench_step_import(c: &mut Criterion) {
    let mut group = c.benchmark_group("step_import");

    let small = small_box_step();
    group.bench_function("small_box", |b| {
        b.iter(|| {
            let mut topo = Topology::new();
            let solids =
                remus_io::step::reader::read_step(&small, &mut topo).expect("small box imports");
            assert_eq!(solids.len(), 1);
        });
    });

    group.bench_function("hammer_holder", |b| {
        let data = include_str!("../tests/data/shapr3d_hammer_holder.step");
        b.iter(|| {
            let mut topo = Topology::new();
            let solids =
                remus_io::step::reader::read_step(data, &mut topo).expect("hammer holder imports");
            assert!(!solids.is_empty());
        });
    });

    let repeated = repeated_entities_step(5_000);
    group.bench_function("repeated_entities_5k", |b| {
        b.iter(|| {
            let mut topo = Topology::new();
            // No MANIFOLD_SOLID_BREP roots: measures scan + index + unit
            // resolution only, without topology construction.
            let solids =
                remus_io::step::reader::read_step(&repeated, &mut topo).expect("index runs");
            assert!(solids.is_empty());
        });
    });

    let rational = large_rational_net_step(2_000);
    group.bench_function("rational_net_2k", |b| {
        b.iter(|| {
            let mut topo = Topology::new();
            let solids =
                remus_io::step::reader::read_step(&rational, &mut topo).expect("index runs");
            assert!(solids.is_empty());
        });
    });

    let malformed =
        "ISO-10303-21;DATA;#1=CARTESIAN_POINT('',(1.,2.,unterminated;ENDSEC;END-ISO-10303-21;";
    group.bench_function("malformed_rejection", |b| {
        b.iter(|| {
            let mut topo = Topology::new();
            let _ = remus_io::step::reader::read_step_with_limits(
                malformed,
                &mut topo,
                ImportLimits::default(),
            );
        });
    });

    group.finish();
}

criterion_group!(benches, bench_step_import);
criterion_main!(benches);
