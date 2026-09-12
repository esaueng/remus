//! Native workers for `scripts/performance/run.py`; stdout is a JSONL protocol.
#![allow(clippy::expect_used, clippy::unwrap_used, clippy::print_stdout)]

use std::{hint::black_box, time::Instant};

use remus_math::{
    nurbs::intersection::{IntersectionPoint, chain_intersection_points},
    vec::Point3,
};
use remus_wasm::kernel::BrepKernel;
use serde_json::{Value, json};

const CALLS: usize = 150;
const MATRIX: [f64; 16] = [
    1., 0., 0., 0.001, 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
];

fn near(actual: f64, expected: f64) {
    assert!(
        actual.is_finite() && (actual - expected).abs() <= 1e-8,
        "{actual} != {expected}"
    );
}

fn batch(kernel: &mut BrepKernel, input: &str, count: usize) -> Vec<Value> {
    checked_batch(&kernel.execute_batch(input), count)
}

fn checked_batch(output: &str, count: usize) -> Vec<Value> {
    let rows: Vec<Value> = serde_json::from_str(output).expect("batch JSON");
    assert_eq!(rows.len(), count);
    assert!(
        rows.iter()
            .all(|r| r.get("ok").is_some() && r.get("error").is_none()),
        "{output}"
    );
    rows
}

fn transform(size: usize, direct: bool) -> (u128, Value) {
    let mut kernel = BrepKernel::new();
    let seeds = vec![json!({"op":"makeBox", "args":{"width":1.,"height":1.,"depth":1.}}); size];
    let handles = batch(&mut kernel, &serde_json::to_string(&seeds).unwrap(), size);
    let ids: std::collections::BTreeSet<u32> = handles
        .iter()
        .map(|row| u32::try_from(row["ok"].as_u64().expect("solid handle")).unwrap())
        .collect();
    assert_eq!(ids.len(), size, "seeding must create distinct solids");
    let base = u32::try_from(handles[0]["ok"].as_u64().expect("solid handle")).unwrap();
    let untouched = handles[size - 1]["ok"].as_u64().unwrap();
    let input = serde_json::to_string(&vec![
        json!({"op":"transform","args":{"solid":base,"matrix":MATRIX}});
        CALLS
    ])
    .unwrap();
    let start = Instant::now();
    let output = if direct {
        for _ in 0..CALLS {
            kernel
                .transform_solid_binding(base, MATRIX.to_vec())
                .expect("direct transform");
        }
        None
    } else {
        Some(kernel.execute_batch(&input))
    };
    let ns = start.elapsed().as_nanos();
    if let Some(output) = output {
        checked_batch(&output, CALLS);
    }
    let query = json!([
        {"op":"boundingBox","args":{"solid":base}},
        {"op":"volume","args":{"solid":base,"deflection":0.5}},
        {"op":"boundingBox","args":{"solid":untouched}}
    ]);
    let values = batch(&mut kernel, &query.to_string(), 3);
    let moved = [0.15, 0., 0., 1.15, 1., 1.];
    let stationary = [0., 0., 0., 1., 1., 1.];
    for i in 0..6 {
        near(values[0]["ok"][i].as_f64().unwrap(), moved[i]);
        near(values[2]["ok"][i].as_f64().unwrap(), stationary[i]);
    }
    near(values[1]["ok"].as_f64().unwrap(), 1.);
    (
        ns,
        json!({"calls":CALLS,"volume":1.,"translation_x":0.15,"untouched_solid_checked":true}),
    )
}

fn chain(size: usize) -> (u128, Value) {
    let points: Vec<_> = (0..size)
        .map(|i| IntersectionPoint {
            point: Point3::new(i as f64, 0., 0.),
            param1: (i as f64, 0.),
            param2: (i as f64, 0.),
        })
        .collect();
    let start = Instant::now();
    let chains = black_box(chain_intersection_points(black_box(&points), 1.1));
    let ns = start.elapsed().as_nanos();
    assert_eq!(chains.len(), 1);
    assert_eq!(chains[0].len(), size);
    for (actual, expected) in chains[0].iter().zip(&points) {
        near((actual.point - expected.point).length(), 0.);
        near(actual.param1.0, expected.param1.0);
        near(actual.param1.1, expected.param1.1);
        near(actual.param2.0, expected.param2.0);
        near(actual.param2.1, expected.param2.1);
    }
    (
        ns,
        json!({"components":1,"points":size,"ordered_membership_checked":true}),
    )
}

#[cfg(feature = "io")]
fn nurbs(scenario: &str, samples: usize, warmup: usize) {
    use remus_operations::{measure::mass_properties, validate::validate_solid};
    use remus_topology::{Topology, explorer::solid_faces, face::FaceSurface};
    let mut topo = Topology::new();
    let solids = remus_io::step::reader::read_step(
        include_str!("../../io/tests/data/shapr3d_hammer_holder.step"),
        &mut topo,
    )
    .expect("hammer-holder import");
    assert_eq!(solids.len(), 1);
    let solid = solids[0];
    let faces = solid_faces(&topo, solid).unwrap();
    assert_eq!(faces.len(), 160);
    let nurbs_faces = faces
        .iter()
        .filter(|&&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_)))
        .count();
    assert_eq!(nurbs_faces, 42);
    for sample in 0..samples + warmup {
        let start = Instant::now();
        let (ns, metrics) = if scenario == "native_validate" {
            let report = black_box(validate_solid(&topo, solid).expect("strict validation"));
            let ns = start.elapsed().as_nanos();
            assert!(report.is_valid(), "{report:?}");
            (
                ns,
                json!({"faces":faces.len(),"nurbs_faces":nurbs_faces,"validation_errors":report.error_count()}),
            )
        } else {
            let props = black_box(mass_properties(&topo, solid).expect("mass properties"));
            let ns = start.elapsed().as_nanos();
            // Same measured reference and tolerance as regress_shapr3d_reversed_nurbs_faces.
            let reference = 50_240.482_8;
            assert!(props.mass.is_finite() && (props.mass - reference).abs() <= reference * 0.001);
            let center = [props.center.x(), props.center.y(), props.center.z()];
            assert!(
                center
                    .iter()
                    .chain(props.inertia.iter())
                    .all(|v| v.is_finite())
            );
            assert!(props.inertia[..3].iter().all(|&v| v > 0.));
            (
                ns,
                json!({"faces":faces.len(),"nurbs_faces":nurbs_faces,"mass":props.mass,"center":center,"inertia":props.inertia}),
            )
        };
        emit(scenario, 42, sample, warmup, ns, metrics);
    }
}

fn emit(scenario: &str, size: usize, sample: usize, warmup: usize, ns: u128, metrics: Value) {
    println!(
        "{}",
        json!({"schema":"remus-performance-sample-v1","scenario":scenario,"size":size,
        "sample":sample,"warmup":sample < warmup,"operation_ns":ns,"validation":"passed","metrics":metrics})
    );
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    assert_eq!(args.len(), 4, "scenario size samples warmup");
    let scenario = args[0].as_str();
    let size: usize = args[1].parse().expect("size");
    let samples: usize = args[2].parse().expect("samples");
    let warmup: usize = args[3].parse().expect("warmup");
    assert!((1..=1000).contains(&samples) && (1..=100).contains(&warmup));
    match scenario {
        "native_validate" | "native_mass_properties" => {
            assert_eq!(size, 42);
            #[cfg(feature = "io")]
            nurbs(scenario, samples, warmup);
            #[cfg(not(feature = "io"))]
            return Err("NURBS benchmarks require --features io".into());
        }
        "native_chain" | "native_transform_direct" | "native_transform_batch" => {
            assert!((2..=8192).contains(&size));
            for sample in 0..samples + warmup {
                let (ns, metrics) = if scenario == "native_chain" {
                    chain(size)
                } else {
                    transform(size, scenario == "native_transform_direct")
                };
                emit(scenario, size, sample, warmup, ns, metrics);
            }
        }
        _ => return Err(format!("unknown scenario: {scenario}").into()),
    }
    Ok(())
}
