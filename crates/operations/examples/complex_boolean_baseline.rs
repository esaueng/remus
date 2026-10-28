//! Bounded complex-boolean worker; see `scripts/performance/complex-boolean`.
//!
//! Setup, kernel, independent qualification and destruction have separate
//! timers. Fresh topology is built for every sample; thread-local content
//! memos, when requested, intentionally survive between samples.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stdout,
    missing_docs
)]

use std::f64::consts::PI;
use std::time::Instant;

use remus_check::properties::{PropertiesOptions, solid_volume};
use remus_math::{mat::Mat4, vec::Point3};
use remus_operations::boolean::{BooleanOp, boolean, set_thread_store_local_cuts};
use remus_operations::classify::{PointClassification, classify_points};
use remus_operations::{compound_ops::fuse_solids, primitives, transform::transform_solid};
use remus_topology::{Topology, solid::SolidId};
use serde_json::{Value, json};

struct Problem {
    topo: Topology,
    operands: Vec<SolidId>,
    operation: Option<BooleanOp>,
    expected_volume: f64,
    expected_components: usize,
    probes: Vec<(Point3, PointClassification)>,
}

fn placed_box(topo: &mut Topology, dims: [f64; 3], at: [f64; 3]) -> SolidId {
    let body = primitives::make_box(topo, dims[0], dims[1], dims[2]).unwrap();
    transform_solid(topo, body, &Mat4::translation(at[0], at[1], at[2])).unwrap();
    body
}

fn prepare(scenario: &str, size: u32) -> Problem {
    let mut topo = Topology::new();
    let mut probes = vec![(
        Point3::new(-10.0, -10.0, -10.0),
        PointClassification::Outside,
    )];
    let (operands, operation, expected_volume, expected_components) = match scenario {
        "slot_cut" | "slot_fuse" | "slot_intersect" => {
            assert!(size <= 24, "hole-grid width exceeds bounded workload");
            let extent = f64::from(size.max(4)) * 12.0 + 4.0;
            let mut plate = primitives::make_box(&mut topo, extent, extent, 10.0).unwrap();
            // Fixture generation uses the normal defaults even when the timed
            // operation will request store-local execution.
            for row in 0..size {
                for col in 0..size {
                    let drill = primitives::make_cylinder(&mut topo, 2.0, 20.0).unwrap();
                    transform_solid(
                        &mut topo,
                        drill,
                        &Mat4::translation(
                            6.0 + f64::from(col) * 12.0,
                            6.0 + f64::from(row) * 12.0,
                            -5.0,
                        ),
                    )
                    .unwrap();
                    plate = boolean(&mut topo, BooleanOp::Cut, plate, drill).unwrap();
                }
            }
            let plate_volume = extent * extent * 10.0 - f64::from(size * size) * 40.0 * PI;
            let (op, height, z, expected) = match scenario {
                "slot_fuse" => (BooleanOp::Fuse, 4.0, 8.0, plate_volume + 24.0),
                "slot_intersect" => (BooleanOp::Intersect, 20.0, -5.0, 120.0),
                _ => (BooleanOp::Cut, 20.0, -5.0, plate_volume - 120.0),
            };
            let tool = placed_box(&mut topo, [2.0, 6.0, height], [11.0, 9.0, z]);
            probes.push((
                Point3::new(12.0, 12.0, 5.0),
                if op == BooleanOp::Cut {
                    PointClassification::Outside
                } else {
                    PointClassification::Inside
                },
            ));
            probes.push((
                Point3::new(2.0, 2.0, 5.0),
                if op == BooleanOp::Intersect {
                    PointClassification::Outside
                } else {
                    PointClassification::Inside
                },
            ));
            if size > 0 {
                probes.push((Point3::new(6.0, 6.0, 5.0), PointClassification::Outside));
            }
            (vec![plate, tool], Some(op), expected, 1)
        }
        "fuse_sparse" | "fuse_dense" => {
            assert!(
                (1..=256).contains(&size),
                "body count exceeds bounded workload"
            );
            let sparse = scenario == "fuse_sparse";
            let step = if sparse { 3.0 } else { 0.5 / f64::from(size) };
            let bodies = (0..size)
                .map(|i| placed_box(&mut topo, [1.0; 3], [f64::from(i) * step, 0.0, 0.0]))
                .collect();
            probes.push((Point3::new(0.25, 0.25, 0.25), PointClassification::Inside));
            for i in 1..size {
                probes.push((
                    Point3::new(f64::from(i) * step + 0.5, 0.5, 0.5),
                    PointClassification::Inside,
                ));
            }
            if sparse {
                probes.push((Point3::new(2.0, 0.25, 0.25), PointClassification::Outside));
            }
            (
                bodies,
                None,
                if sparse {
                    f64::from(size)
                } else {
                    1.0 + f64::from(size - 1) * step
                },
                if sparse {
                    usize::try_from(size).unwrap()
                } else {
                    1
                },
            )
        }
        "curved_half_cylinder" => {
            assert_eq!(size, 1);
            let cylinder = primitives::make_cylinder(&mut topo, 2.0, 10.0).unwrap();
            let tool = placed_box(&mut topo, [3.0, 6.0, 12.0], [0.0, -3.0, -1.0]);
            probes.push((Point3::new(-1.0, 0.0, 5.0), PointClassification::Inside));
            probes.push((Point3::new(1.0, 0.0, 5.0), PointClassification::Outside));
            (vec![cylinder, tool], Some(BooleanOp::Cut), 20.0 * PI, 1)
        }
        // Unknown scenarios are protocol errors, never successful samples.
        _ => std::process::exit(2),
    };
    Problem {
        topo,
        operands,
        operation,
        expected_volume,
        expected_components,
        probes,
    }
}

fn qualify(problem: &Problem, result: SolidId, operand_volumes: &[f64]) -> Value {
    let topo = &problem.topo;
    let report = remus_operations::validate::validate_solid(topo, result).unwrap();
    assert!(
        report.is_valid(),
        "strict result validation failed: {report:?}"
    );
    let actual = solid_volume(topo, result, &PropertiesOptions::default()).unwrap();
    // Independent closed forms; this is a benchmark oracle, not an operation
    // tolerance or a change to integration/validation policy.
    let bound = 1e-8 * problem.expected_volume.abs().max(1.0);
    assert!(
        (actual - problem.expected_volume).abs() <= bound,
        "volume {actual} != {} (bound {bound})",
        problem.expected_volume
    );
    let faces = remus_topology::explorer::solid_faces(topo, result).unwrap();
    let edge_map = remus_topology::explorer::edge_to_face_map(topo, result).unwrap();
    let mut remaining: std::collections::BTreeSet<_> = faces.iter().copied().collect();
    let mut component_count = 0;
    while let Some(seed) = remaining.pop_first() {
        component_count += 1;
        let mut pending = vec![seed];
        while let Some(face) = pending.pop() {
            for edge in remus_topology::explorer::face_edges(topo, face).unwrap() {
                for &adjacent in &edge_map[&edge.index()] {
                    if remaining.remove(&adjacent) {
                        pending.push(adjacent);
                    }
                }
            }
        }
    }
    assert_eq!(component_count, problem.expected_components);
    let points: Vec<_> = problem.probes.iter().map(|&(point, _)| point).collect();
    let classified = classify_points(topo, result, &points, 0.1, 1e-7).unwrap();
    for (&actual, &(_, expected)) in classified.iter().zip(&problem.probes) {
        assert_eq!(actual, expected);
    }
    for (&operand, &before) in problem.operands.iter().zip(operand_volumes) {
        assert_eq!(
            solid_volume(topo, operand, &PropertiesOptions::default())
                .unwrap()
                .to_bits(),
            before.to_bits(),
            "input operand changed"
        );
    }
    json!({"volume":actual,"expected_volume":problem.expected_volume,"volume_bound":bound,
        "edge_connected_components":component_count,"faces":faces.len(),
        "material_probes":problem.probes.len(),"input_operands_preserved":true,
        "allocated_slots":topo.allocated_slot_count()})
}

#[cfg(feature = "perf-counters")]
fn collect_work() -> Value {
    let c = remus_algo::perf::snapshot();
    json!({"pave_vertex_probes":c.pave_vertex_probes,"sd_poly_clips":c.sd_poly_clips,
        "ve_sampled_probes":c.ve_sampled_probes,"ve_projection_evals":c.ve_projection_evals,
        "ef_analytic_pair_scans":c.ef_analytic_pair_scans,"face_split_probes":c.face_split_probes,
        "store_faces_copied":c.store_faces_copied,"coplanar_section_probes":c.coplanar_section_probes,
        "ve_pairs_visited":c.ve_pairs_visited,"ee_pairs_visited":c.ee_pairs_visited,
        "ee_pairs_surviving":c.ee_pairs_surviving,"ff_pairs_visited":c.ff_pairs_visited,
        "ff_pairs_surviving":c.ff_pairs_surviving})
}

#[cfg(feature = "perf-counters")]
fn clock_ms() -> f64 {
    static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_secs_f64() * 1000.0
}

#[cfg(not(feature = "perf-counters"))]
fn collect_work() -> Value {
    Value::Null
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    assert_eq!(
        args.len(),
        7,
        "scenario size samples warmup store-local(off|on) memos(off|on) phase-clock(off|on)"
    );
    let size: u32 = args[1].parse().unwrap();
    let samples: u32 = args[2].parse().unwrap();
    let warmup: u32 = args[3].parse().unwrap();
    assert!((1..=1000).contains(&samples) && (1..=100).contains(&warmup));
    assert!((4..=6).all(|i| ["off", "on"].contains(&args[i].as_str())));
    let store_local = args[4] == "on";
    let memos = args[5] == "on";
    let phase_clock = args[6] == "on";
    assert!(!phase_clock || cfg!(feature = "perf-counters"));
    #[cfg(feature = "perf-counters")]
    remus_algo::perf::configure_phase_clock(if phase_clock { Some(clock_ms) } else { None });
    if memos {
        remus_algo::pcurve_memo::enable_thread_pcurve_memo();
        remus_algo::wire_area_memo::enable_thread_wire_area_memo();
    }
    for sample in 0..samples + warmup {
        set_thread_store_local_cuts(false);
        let started = Instant::now();
        let mut problem = prepare(&args[0], size);
        let setup_ns = started.elapsed().as_nanos();
        let started = Instant::now();
        let operand_volumes: Vec<_> = problem
            .operands
            .iter()
            .map(|&s| solid_volume(&problem.topo, s, &PropertiesOptions::default()).unwrap())
            .collect();
        let preflight_ns = started.elapsed().as_nanos();
        set_thread_store_local_cuts(store_local);
        #[cfg(feature = "perf-counters")]
        {
            remus_algo::perf::reset();
            let _ = remus_algo::perf::take_phases();
            let _ = remus_operations::performance::take_boolean_routes();
        }
        let started = Instant::now();
        let result = match problem.operation {
            Some(op) => boolean(
                &mut problem.topo,
                op,
                problem.operands[0],
                problem.operands[1],
            )
            .unwrap(),
            None => fuse_solids(&mut problem.topo, &problem.operands).unwrap(),
        };
        let kernel_ns = started.elapsed().as_nanos();
        let work = collect_work();
        #[cfg(feature = "perf-counters")]
        let (routes, phases) = {
            let r = remus_operations::performance::take_boolean_routes();
            (
                json!({"general_gfa_calls":r.general_gfa_calls,"store_local_attempts":r.store_local_attempts,
                "store_local_successes":r.store_local_successes,"local_cut_successes":r.local_cut_successes,
                "nary_calls":r.nary_calls}),
                remus_algo::perf::take_phases()
                    .into_iter()
                    .map(|p| json!({"name":p.name,"calls":p.calls,"duration_ms":p.duration_ms}))
                    .collect::<Vec<_>>(),
            )
        };
        #[cfg(not(feature = "perf-counters"))]
        let (routes, phases) = (Value::Null, Vec::<Value>::new());
        let started = Instant::now();
        let metrics = qualify(&problem, result, &operand_volumes);
        let qualification_ns = started.elapsed().as_nanos();
        let started = Instant::now();
        drop(problem);
        let teardown_ns = started.elapsed().as_nanos();
        println!(
            "{}",
            json!({"schema":"remus-complex-boolean-sample-v1","scenario":args[0],
            "size":size,"sample":sample,"warmup":sample<warmup,"validation":"passed",
            "settings":{"store_local_requested":store_local,"content_memos":memos,
                "perf_counters":cfg!(feature="perf-counters"),"phase_clock":phase_clock},
            "stages_ns":{"setup":setup_ns,"preflight":preflight_ns,"kernel":kernel_ns,
                "qualification":qualification_ns,"teardown":teardown_ns},
            "generation_ns":setup_ns+kernel_ns,"work":work,"routes":routes,"phases":phases,"metrics":metrics})
        );
    }
}
