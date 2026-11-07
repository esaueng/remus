//! Ignored native acceptance repro for the imported direct-edit workflow, A0-A8.
//!
//! Tier: native (qualification only). No geometry is synthesized in place of
//! the supplied STEP files. Set REMUS_VIDEO_FIXTURES to their directory and
//! REMUS_VIDEO_OUTPUT to an empty evidence directory. Run with:
//! cargo test -p remus-io --profile ci-test --test video_workflow_baseline -- --ignored --nocapture
//!
//! The coordinator isolates each case in a process (180 s limit), writes the
//! complete table, then ASSERTS acceptance. The report-only spike is separate.
//! Failed prerequisites are recorded; acceptance probes start at W-Fillet + R1.
//! Output includes exact errors, before/after validation, dual-deflection
//! volumes, source-body preservation, whole-arena rollback, and selection witnesses.
//! Fixtures default to tests/data. `REMUS_VIDEO_MODE=cumulative|spike|all`
//! selects the sequential A1-A7 repro, sharp-body rebuild spike, or everything.
//! Known-open rows are labelled XFAIL, but still FAIL the acceptance assertion.
//! Rows follow the corrected spec of the round-2 application run: A1 shortens
//! the rimmed cap to z26 with sampled permission (256 stations), A2 resizes the
//! convex band along x = 0 (R4 -> 1.5 -> 1.9), A3 rounds the x = 0 / y = 18
//! vertical edge after A2, and the cumulative order skips the concave R2 seam
//! (an isolated XFAIL row). Exact-only cap moves into the shoulder are typed
//! refusals by contract.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stderr,
    clippy::print_stdout
)]

use std::collections::BTreeMap;
use std::fmt::Write;
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use remus_io::{arena_io::serialize_solid, step::reader::read_step};
use remus_math::{tolerance::Tolerance, vec::Point3};
use remus_operations::{OperationsError, blend_ops, measure, push_pull, resize_blend};
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use serde_json::{Value, json};

const CASES: &[&str] = &[
    "reference",
    "sharp",
    "A0",
    "A0_cap_rim",
    "A1_short",
    "A1_tall",
    "A1_sequence",
    "A1_shoulder",
    "A1_rim_short",
    "A1_rim_tall",
    "A1_rim_shoulder",
    "A1_quality_short",
    "A1_quality_shoulder",
    "A2_convex_x",
    "A2_convex_y",
    "A2_concave_x",
    "A2_concave_y",
    "A2_sequence",
    "A3",
    "A4_fillet_1.5",
    "A4_fillet_3.7",
    "A4_chamfer_0.7",
    "A4_chamfer_5",
    "A5",
    "A6_seed",
    "A6_chain",
    "A6_loop",
    "A7_sequence",
    "A7_after_bottom",
    "A8",
];

fn output() -> PathBuf {
    std::env::var_os("REMUS_VIDEO_OUTPUT").map_or_else(
        || PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target/video-workflow-baseline"),
        PathBuf::from,
    )
}

fn save(case: &str, value: &Value) {
    std::fs::create_dir_all(output()).unwrap();
    std::fs::write(
        output().join(format!("{case}.json")),
        serde_json::to_vec_pretty(value).unwrap(),
    )
    .unwrap();
}

fn fixtures() -> PathBuf {
    std::env::var_os("REMUS_VIDEO_FIXTURES").map_or_else(
        || PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data"),
        PathBuf::from,
    )
}

struct Trace;
static FAMILIES: std::sync::Mutex<Vec<String>> = std::sync::Mutex::new(Vec::new());
impl log::Log for Trace {
    fn enabled(&self, m: &log::Metadata<'_>) -> bool {
        m.level() <= log::Level::Debug
    }
    fn log(&self, r: &log::Record<'_>) {
        if self.enabled(r.metadata()) {
            let message = format!("{}: {}", r.target(), r.args());
            if r.target().contains("ridge_chain")
                || r.target().contains("equal_radius_boss_corner")
                || r.target() == "remus_approx"
            {
                FAMILIES.lock().unwrap().push(message.clone());
            }
            eprintln!("{message}");
        }
    }
    fn flush(&self) {}
}
fn install_logger() {
    static TRACE: Trace = Trace;
    let _ = log::set_logger(&TRACE);
    log::set_max_level(log::LevelFilter::Debug);
}

fn expected_today(case: &str) -> &'static str {
    if case.starts_with("W3_") {
        "SPIKE"
    } else if case.starts_with("A2_concave") {
        // No construction of the R2 seam under the R4 shoulder (W3).
        "XFAIL (open)"
    } else if matches!(
        case,
        "A1_short" | "A1_sequence" | "A1_shoulder" | "A1_rim_short" | "A1_rim_shoulder"
    ) {
        "PASS (typed exact-only refusal)"
    } else {
        "PASS"
    }
}

fn relative(a: f64, b: f64) -> f64 {
    (a - b).abs() / a.abs().max(b.abs()).max(1e-30)
}

/// Collect every violation before the coordinator asserts, retaining a full table.
fn geometry_errors(m: &Value) -> Vec<String> {
    let mut errors = vec![];
    for k in [
        "closed",
        "operations_valid",
        "check_valid",
        "volume_consistent_1e-4",
    ] {
        if m[k] != true {
            errors.push(format!("{k} failed"));
        }
    }
    for k in ["free_edges", "nonmanifold_edges"] {
        if m[k] != 0 {
            errors.push(format!("{k}={}", m[k]));
        }
    }
    let diag = (0..3)
        .map(|i| (m["bbox_max"][i].as_f64().unwrap() - m["bbox_min"][i].as_f64().unwrap()).powi(2))
        .sum::<f64>()
        .sqrt();
    let mut mesh_volumes = vec![];
    for i in 0..2 {
        let d = m["deflections"][i].as_f64().unwrap();
        if !(d > 0.0 && d <= diag * 5e-5) {
            errors.push("deflection exceeds volume clamp".into());
        }
        let mesh = &m["meshes"][i]["Ok"];
        if mesh["boundary"] != 0 || mesh["nonmanifold"] != 0 || mesh["watertight"] != true {
            errors.push(format!(
                "mesh[{i}] boundary={} nonmanifold={}",
                mesh["boundary"], mesh["nonmanifold"]
            ));
        }
        match (
            m["volumes"][i]["Ok"].as_f64(),
            mesh["signed_volume"].as_f64(),
        ) {
            (Some(a), Some(b)) if a.is_finite() && b > 0.0 => {
                if relative(a, b) > 1e-4 {
                    errors.push(format!(
                        "API/mesh[{i}] volume mismatch {:.6}",
                        relative(a, b)
                    ));
                }
                mesh_volumes.push(b);
            }
            _ => errors.push(format!("volume[{i}] unavailable or nonpositive")),
        }
    }
    if mesh_volumes.len() == 2 && relative(mesh_volumes[0], mesh_volumes[1]) > 1e-4 {
        errors.push("mesh volume does not converge".into());
    }
    // These fixtures are small analytic solids with a bounded number of original
    // sampled corners. A planar facet fallback cannot satisfy this census.
    let c = &m["census"];
    if c["plane"].as_u64().unwrap_or(0) < 7
        || c["cylinder"].as_u64().unwrap_or(0) < 1
        || m["counts_fev"][0].as_u64().unwrap_or(999) > 40
    {
        errors.push(format!("analytic census failed: {c}"));
    }
    for (label, want) in [
        ("block_core", "Ok(Inside)"),
        ("outside_block", "Ok(Outside)"),
    ] {
        if !m["probes"]
            .as_array()
            .unwrap()
            .iter()
            .any(|p| p["label"] == label && p["classification"] == want)
        {
            errors.push(format!("classification {label} != {want}"));
        }
    }
    errors
}

fn evaluate(case: &str, value: &Value) -> Vec<String> {
    let mut errors = vec![];
    if let Some(m) = value.get("metrics") {
        errors.extend(geometry_errors(m));
        let expected = if case == "sharp" {
            json!({"plane":7,"cylinder":1})
        } else {
            json!({"plane":7,"cylinder":5,"torus":1,"nurbs":2})
        };
        if m["census"] != expected {
            errors.push("fixture analytic census changed".into());
        }
        let volume = if case == "sharp" {
            16_352.920_065_876_968
        } else {
            16_347.661_489_813_898
        };
        if m["volumes"][0]["Ok"]
            .as_f64()
            .is_none_or(|v| relative(v, volume) > 1e-8)
        {
            errors.push("fixture volume changed".into());
        }
    }
    for row in value["attempts"].as_array().into_iter().flatten() {
        let negative_radius = row["selection"]["value"].as_f64().is_some_and(|v| v < 0.0);
        if row.get("blocked").is_some() {
            errors.push(format!("blocked: {}", row["label"]));
            continue;
        }
        if row["phase"] != "complete" {
            errors.push("incomplete operation".into());
            continue;
        }
        if row["source_body_untouched"] != true {
            errors.push("source body mutated".into());
        }
        errors.extend(geometry_errors(&row["after"]));
        if row["success"] == false {
            if row["error_code"].as_str().is_none_or(str::is_empty) {
                errors.push("untyped refusal".into());
            }
            // Retired slots and mutation ticks are monotonic by contract. Live
            // entities, pcurves and the exact source document must be unchanged.
            for k in [
                "vertices", "edges", "faces", "wires", "shells", "solids", "pcurves",
            ] {
                if row["arena_before"][k] != row["arena_after"][k] {
                    errors.push(format!("refusal leaked {k}"));
                }
            }
            if !negative_radius
                && case != "A8"
                && row["allow_refusal"] != true
                && row["selection"]["preview_only"] != true
            {
                errors.push(format!("refused: {}", row["error"]));
            }
            if negative_radius && row["error_code"] != "invalid-input" {
                errors.push("negative radius must be InvalidInput".into());
            }
        } else if negative_radius {
            errors.push("negative radius accepted".into());
        }
        if row["success"] == true {
            if let Some(expected) = row["selection"]["expected_volume"].as_f64() {
                for volume in row["after"]["volumes"].as_array().unwrap() {
                    if volume["Ok"]
                        .as_f64()
                        .is_none_or(|v| relative(v, expected) > 1e-4)
                    {
                        errors.push("sharp diameter closed-form volume mismatch".into());
                    }
                }
            }
            if let Some(r) = row["selection"]["new_radius"]
                .as_f64()
                .or_else(|| row["selection"]["radius"].as_f64())
                && !row["after"]["analytic_radii"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .any(|a| a["radius"].as_f64().is_some_and(|v| near(v, r)))
            {
                errors.push(format!("analytic radius R{r} missing"));
            }
            let before_tori = row["before"]["census"]["torus"].as_u64().unwrap_or(0);
            if before_tori > 0 && row["after"]["census"]["torus"].as_u64().unwrap_or(0) == 0 {
                errors.push("analytic tori lost".into());
            }
        }
        for p in row["intent_probes"].as_array().into_iter().flatten() {
            if p["actual"] != p["expected"] {
                errors.push(format!(
                    "feature probe {} expected {} got {}",
                    p["label"], p["expected"], p["actual"]
                ));
            }
        }
        if row.get("cancel_bytes_equal").is_some() && row["cancel_bytes_equal"] != true {
            errors.push("cancel mutated document".into());
        }
    }
    if case == "cumulative" && value["completed_A7"] != true {
        errors.push("A1-A7 cumulative workflow incomplete".into());
    }
    if case == "A0" {
        let last = value["attempts"].as_array().unwrap().last().unwrap();
        if last["after"]["census"] != json!({"plane":7,"cylinder":5,"torus":2,"nurbs":2}) {
            errors.push("rebuilt A0 must match W-Fillet plus R1 analytic census".into());
        }
        if last["after"]["volumes"][0]["Ok"]
            .as_f64()
            .is_none_or(|v| relative(v, 16_339.872_376_730_229) > 1e-8)
        {
            errors.push("rebuilt A0 must match W-Fillet plus R1 volume".into());
        }
    }
    errors
}

#[test]
#[ignore = "open: A1/A2/A3/A6/A7 video acceptance; process-isolated ready-repro"]
fn video_workflow_baseline() {
    install_logger();
    if let Ok(case) = std::env::var("REMUS_VIDEO_CASE") {
        if case.starts_with("W3_") {
            spike(&case);
        } else {
            worker(&case);
        }
        if std::env::var_os("REMUS_VIDEO_CHILD").is_none() {
            let value = serde_json::from_slice(
                &std::fs::read(output().join(format!("{case}.json"))).unwrap(),
            )
            .unwrap();
            let errors = evaluate(&case, &value);
            println!(
                "{case}: {} {errors:?}",
                if errors.is_empty() { "PASS" } else { "FAIL" }
            );
            if !case.starts_with("W3_") {
                assert!(errors.is_empty(), "{case} acceptance failed: {errors:?}");
            }
        }
        return;
    }
    std::fs::create_dir_all(output()).unwrap();
    let mode = std::env::var("REMUS_VIDEO_MODE").unwrap_or_else(|_| "acceptance".into());
    let mut cases: Vec<String> = Vec::new();
    if mode == "acceptance" || mode == "all" {
        cases.extend(CASES.iter().map(|s| (*s).into()));
    }
    if mode != "spike" {
        cases.push("cumulative".into());
    }
    if mode == "spike" || mode == "all" {
        for body in [
            "base", "z23", "z26", "z30", "z61.2", "d14.6", "d17", "d21.1", "d14.7",
        ] {
            for set in [
                "all4", "round1.5", "round1.9", "seam2", "both1.5", "both1.9",
            ] {
                cases.push(format!("W3_{body}_{set}"));
                if set.starts_with("seam") || set.starts_with("both") {
                    cases.push(format!("W3_{body}_{set}_shoulderfirst"));
                }
            }
        }
    }
    let mut process_failures = Vec::new();
    let mut failures = Vec::new();
    let mut table =
        String::from("| Case | Expected today | Acceptance | Details |\n|---|---|---|---|\n");
    for batch in cases.chunks(4) {
        let results = std::thread::scope(|scope| {
            // Spawn the complete batch before joining; a lazy iterator serializes it.
            #[allow(clippy::needless_collect)]
            let jobs = batch
                .iter()
                .map(|case| scope.spawn(|| run_case(case)))
                .collect::<Vec<_>>();
            jobs.into_iter()
                .map(|j| j.join().unwrap())
                .collect::<Vec<_>>()
        });
        for (case, process_ok, errors) in results {
            if !process_ok {
                process_failures.push(case.clone());
            }
            let status = if errors.is_empty() { "PASS" } else { "FAIL" };
            writeln!(
                table,
                "| {case} | {} | {status} | {} |",
                expected_today(&case),
                errors.join("; ").replace('|', "/")
            )
            .unwrap();
            std::fs::write(output().join("ACCEPTANCE.md"), &table).unwrap();
            if !errors.is_empty() {
                failures.push(case);
            }
        }
    }
    println!("{table}");
    assert!(
        process_failures.is_empty(),
        "evidence collection failed for {process_failures:?}"
    );
    // The spike is report-only; acceptance never turns known failures into success.
    if mode != "spike" {
        assert!(
            failures.is_empty(),
            "acceptance failed: {failures:?}; see ACCEPTANCE.md"
        );
    }
}

fn run_case(case: &str) -> (String, bool, Vec<String>) {
    let mut process_ok = true;
    let log = std::fs::File::create({
        std::fs::create_dir_all(output()).unwrap();
        output().join(format!("{case}.log"))
    })
    .unwrap();
    let started = Instant::now();
    let mut child = Command::new(std::env::current_exe().unwrap())
        .args([
            "--ignored",
            "--exact",
            "video_workflow_baseline",
            "--nocapture",
        ])
        .env("REMUS_VIDEO_CASE", case)
        .env("REMUS_VIDEO_CHILD", "1")
        .stdout(Stdio::from(log.try_clone().unwrap()))
        .stderr(Stdio::from(log))
        .spawn()
        .unwrap();
    loop {
        if let Some(status) = child.try_wait().unwrap() {
            if !status.success() {
                process_ok = false;
                save(
                    &format!("{case}_process"),
                    &json!({"case":case,"process_error":status.to_string(),"elapsed_ms":started.elapsed().as_secs_f64()*1000.0}),
                );
            }
            break;
        }
        if started.elapsed() > Duration::from_secs(180) {
            process_ok = false;
            child.kill().unwrap();
            child.wait().unwrap();
            save(
                &format!("{case}_process"),
                &json!({"case":case,"timeout_seconds":180,"last_progress":"see case.json and case.log"}),
            );
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    println!(
        "{case}: evidence collected ({:.3}s)",
        started.elapsed().as_secs_f64()
    );
    let errors = if process_ok {
        let value: Value =
            serde_json::from_slice(&std::fs::read(output().join(format!("{case}.json"))).unwrap())
                .unwrap();
        evaluate(case, &value)
    } else {
        vec!["child failed or timed out; see log".into()]
    };
    (case.into(), process_ok, errors)
}

fn endpoints(t: &Topology, e: EdgeId) -> (Point3, Point3) {
    let e = t.edge(e).unwrap();
    (
        t.vertex(e.start()).unwrap().point(),
        t.vertex(e.end()).unwrap().point(),
    )
}
fn xyz(p: Point3) -> [f64; 3] {
    [p.x(), p.y(), p.z()]
}
fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-5
}

fn arena_counts(t: &Topology) -> Value {
    json!({"vertices":t.num_vertices(),"edges":t.num_edges(),"faces":t.num_faces(),
        "wires":t.num_wires(),"shells":t.num_shells(),"solids":t.num_solids(),"pcurves":t.num_pcurves(),
        "slots_including_retired":t.allocated_slot_count(),"mutation_ticks":t.mutation_ticks()})
}

fn inventory(t: &Topology, s: SolidId) -> Value {
    let adj = AdjacencyIndex::build(t, s).unwrap();
    json!({"faces": solid_faces(t,s).unwrap().iter().map(|&f| {
        let face=t.face(f).unwrap();
        json!({"id":f.index(), "reversed":face.is_reversed(), "surface":format!("{:?}",face.surface())})
    }).collect::<Vec<_>>(), "edges":solid_edges(t,s).unwrap().iter().map(|&e| {
        let (a,b)=endpoints(t,e);
        json!({"id":e.index(), "a":xyz(a), "b":xyz(b), "curve":t.edge(e).unwrap().curve().type_tag(), "faces":adj.faces_for_edge(e).iter().map(|f|f.index()).collect::<Vec<_>>()})
    }).collect::<Vec<_>>()})
}

fn metrics(t: &Topology, s: SolidId) -> Value {
    let started = Instant::now();
    let mut census = BTreeMap::new();
    let mut radii = Vec::new();
    for f in solid_faces(t, s).unwrap() {
        match t.face(f).unwrap().surface() {
            FaceSurface::Cylinder(c) => radii.push(json!({"type":"cylinder","radius":c.radius()})),
            FaceSurface::Torus(c) => radii.push(json!({"type":"torus","radius":c.minor_radius()})),
            _ => {}
        }
        *census
            .entry(t.face(f).unwrap().surface().type_tag())
            .or_insert(0usize) += 1;
    }
    let adj = AdjacencyIndex::build(t, s).unwrap();
    let report = remus_operations::validate::validate_solid(t, s).unwrap();
    let strict = remus_check::validate::validate_solid(
        t,
        s,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    let bbox = measure::solid_bounding_box(t, s).unwrap();
    // Strictly finer than solid_volume's bbox-diagonal * 5e-5 clamp.
    let diag = (bbox.max - bbox.min).length();
    let ds = [diag * 1e-5, diag * 5e-6];
    let volumes = ds.map(|d| measure::solid_volume(t, s, d).map_err(|e| e.to_string()));
    let relative = match (&volumes[0], &volumes[1]) {
        (Ok(a), Ok(b)) => Some((a - b).abs() / a.abs().max(b.abs()).max(1e-30)),
        _ => None,
    };
    let meshes=ds.map(|d| remus_operations::tessellate::tessellate_solid_with_tolerance(t,s,d,0.15).map(|mesh| {
        let q=remus_operations::tessellate::welded_mesh_quality(&mesh);
        let origin=bbox.min;
        let mesh_volume=mesh.indices.chunks_exact(3).map(|ix| {
            let a=mesh.positions[ix[0] as usize]-origin;
            let b=mesh.positions[ix[1] as usize]-origin;
            let c=mesh.positions[ix[2] as usize]-origin;
            a.dot(b.cross(c))/6.0
        }).sum::<f64>();
        json!({"triangles":q.triangle_count,"boundary":q.boundary_edges,"nonmanifold":q.non_manifold_edges,"watertight":q.is_watertight(),"signed_volume":mesh_volume})
    }).map_err(|e|e.to_string()));
    let probes=[("block_core",Point3::new(20.,9.,12.)),("boss_original_height",Point3::new(0.,0.,30.)),
        ("below_new_cap",Point3::new(0.,0.,bbox.max.z()-0.5)),("above_new_cap",Point3::new(0.,0.,bbox.max.z()+0.5)),
        ("outside_block",Point3::new(35.,9.,12.)),("free_end_top_corner",Point3::new(29.8,8.,23.8)),
        ("expanded_boss",Point3::new(6.5,-0.1,30.))].map(|(label,p)| {
        json!({"label":label,"point":xyz(p),"classification":format!("{:?}",remus_check::classify::classify_point(t,s,p,&remus_check::classify::ClassifyOptions::default()))})
    });
    json!({"counts_fev":solid_entity_counts(t,s).unwrap(),"census":census,"analytic_radii":radii,
        "closed":shell_closed(t,s),"free_edges":adj.boundary_edges().len(),"nonmanifold_edges":adj.non_manifold_edges().len(),
        "operations_valid":report.is_valid(),"operations_issues":format!("{:?}",report.issues),
        "check_valid":strict.is_valid(),"check_issues":format!("{:?}",strict.issues),
        "bbox_min":xyz(bbox.min),"bbox_max":xyz(bbox.max),"deflections":ds,"volumes":volumes,
        "volume_relative_difference":relative,"volume_consistent_1e-4":relative.is_some_and(|r|r<=1e-4),
        "meshes":meshes,"probes":probes,"verification_ms":started.elapsed().as_secs_f64()*1000.0})
}

fn shell_closed(t: &Topology, s: SolidId) -> bool {
    let data = t.solid(s).unwrap();
    std::iter::once(data.outer_shell())
        .chain(data.inner_shells().iter().copied())
        .all(|sh| {
            remus_topology::validation::validate_shell_closed(t.shell(sh).unwrap(), t).is_ok()
        })
}

// These adapters mirror wasm/helpers.rs: filter selection then the common
// fillet cascade; for chamfer, planar first, rollback, then v2 and closure gate.
fn fillet(
    t: &mut Topology,
    s: SolidId,
    edges: &[EdgeId],
    r: f64,
) -> Result<(SolidId, String), OperationsError> {
    let edges = remus_operations::query::filter_filletable_edges(t, s, edges)?;
    if edges.is_empty() {
        return Err(OperationsError::InvalidInput {
            reason:
                "no filletable edges in the selection (tangent and degenerate edges are skipped)"
                    .into(),
        });
    }
    blend_ops::fillet_cascade(t, s, &edges, r).map(|b| (b.solid, format!("{:?}", b.engine)))
}
fn chamfer(
    t: &mut Topology,
    s: SolidId,
    edges: &[EdgeId],
    d: f64,
) -> Result<(SolidId, String), OperationsError> {
    let snapshot = t.clone();
    if let Ok((result, _)) = blend_ops::planar_chamfer_with_origins(t, s, edges, d)
        && shell_closed(t, result)
    {
        return Ok((result, "PlanarBevel".into()));
    }
    t.restore_preserving_handle_slots(&snapshot);
    match blend_ops::chamfer_v2(t, s, edges, d, d) {
        Ok(b) if shell_closed(t, b.solid) => Ok((b.solid, format!("{:?}", b.engine))),
        result => {
            t.restore_preserving_handle_slots(&snapshot);
            match result {
                Err(e) => Err(e),
                Ok(_) => Err(OperationsError::InvalidInput {
                    reason: "chamfer produced an open shell".into(),
                }),
            }
        }
    }
}

fn cap(t: &Topology, s: SolidId) -> FaceId {
    solid_faces(t,s).unwrap().into_iter().filter(|&f|matches!(t.face(f).unwrap().surface(),FaceSurface::Plane {normal,..} if normal.z().abs()>0.999))
        .max_by(|&a,&b| {let z=|f| {let f=t.face(f).unwrap(); match f.surface(){FaceSurface::Plane{normal,d}=>d/normal.z(),_=>unreachable!()}}; z(a).total_cmp(&z(b))}).unwrap()
}
fn band_at(t: &Topology, s: SolidId, vertical: bool, side: bool, radius: f64) -> FaceId {
    solid_faces(t, s)
        .unwrap()
        .into_iter()
        .find(|&f| match t.face(f).unwrap().surface() {
            FaceSurface::Cylinder(c) => {
                near(c.radius(), radius)
                    && (c.axis().z().abs() > 0.99) == vertical
                    && if vertical {
                        if side {
                            c.origin().x() > 1.0
                        } else {
                            c.origin().y() > 1.0
                        }
                    } else if side {
                        c.axis().x().abs() > 0.99
                    } else {
                        c.axis().y().abs() > 0.99
                    }
            }
            _ => false,
        })
        .unwrap()
}
fn boss(t: &Topology, s: SolidId) -> FaceId {
    solid_faces(t,s).unwrap().into_iter().find(|&f|matches!(t.face(f).unwrap().surface(),FaceSurface::Cylinder(c) if c.axis().z().abs()>0.99 && near(c.origin().x(),0.0)&&near(c.origin().y(),0.0))).unwrap()
}
fn end_edge(t: &Topology, s: SolidId) -> EdgeId {
    solid_edges(t, s)
        .unwrap()
        .into_iter()
        .find(|&e| {
            let (a, b) = endpoints(t, e);
            near(a.x(), 30.0)
                && near(b.x(), 30.0)
                && near(a.z(), 24.0)
                && near(b.z(), 24.0)
                && (a.y() - b.y()).abs() > 1.0
        })
        .unwrap()
}
fn bottom(t: &Topology, s: SolidId) -> Vec<EdgeId> {
    let face=solid_faces(t,s).unwrap().into_iter().find(|&f|matches!(t.face(f).unwrap().surface(),FaceSurface::Plane{normal,d} if normal.z().abs()>0.99&&d.abs()<1e-5)).unwrap();
    t.wire(t.face(face).unwrap().outer_wire())
        .unwrap()
        .edges()
        .iter()
        .map(remus_topology::OrientedEdge::edge)
        .collect()
}

fn attempt<F>(
    case: &str,
    label: &str,
    t: &mut Topology,
    s: SolidId,
    selection: Value,
    op: F,
    rows: &mut Vec<Value>,
) -> Option<SolidId>
where
    F: FnOnce(&mut Topology) -> Result<(SolidId, String), OperationsError>,
{
    let before = metrics(t, s);
    let bytes = serialize_solid(t, s).unwrap();
    let arena = format!("{t:?}");
    let arena_before = arena_counts(t);
    rows.push(json!({"label":label,"selection":selection,"before":before,"phase":"running"}));
    save(case, &json!({"case":case,"attempts":rows}));
    let start = Instant::now();
    FAMILIES.lock().unwrap().clear();
    let result = op(t);
    let elapsed = start.elapsed().as_secs_f64() * 1000.0;
    let untouched = serialize_solid(t, s).is_ok_and(|b| b == bytes);
    let whole_arena = arena == format!("{t:?}");
    let (out, error, engine, error_code, error_typed) = match result {
        Ok((out, engine)) => (Some(out), None, Some(engine), None, None),
        Err(error) => (
            None,
            Some(format!("{error}")),
            None,
            Some(blend_ops::blend_failure_code(&error)),
            Some(format!("{error:?}")),
        ),
    };
    let after = metrics(t, out.unwrap_or(s));
    if let Some(out) = out {
        save(&format!("{case}_after_{}", rows.len()), &inventory(t, out));
    }
    let source_after = if untouched {
        before.clone()
    } else {
        metrics(t, s)
    };
    let mut intent_probes = vec![];
    if let Some(out) = out {
        let mut probe = |label: &str, p: Point3, expected: &str| {
            let actual = format!(
                "{:?}",
                remus_check::classify::classify_point(
                    t,
                    out,
                    p,
                    &remus_check::classify::ClassifyOptions::default()
                )
            );
            intent_probes
                .push(json!({"label":label,"point":xyz(p),"actual":actual,"expected":expected}));
        };
        if let Some(d) = selection["delta"].as_f64() {
            let f = cap(t, s);
            if let FaceSurface::Plane { normal, d: offset } = t.face(f).unwrap().surface() {
                let z = offset / normal.z() + d;
                probe(
                    "below_selected_cap",
                    Point3::new(0., 0., z - 0.5),
                    "Ok(Inside)",
                );
                probe(
                    "above_selected_cap",
                    Point3::new(0., 0., z + 0.5),
                    "Ok(Outside)",
                );
            }
        }
        if let Some(d) = selection["diameter"].as_f64() {
            probe(
                "inside_edited_wall",
                Point3::new(0., -d / 2. + 0.5, 30.),
                "Ok(Inside)",
            );
            probe(
                "outside_edited_wall",
                Point3::new(0., -d / 2. - 0.5, 30.),
                "Ok(Outside)",
            );
        }
        if label.starts_with("A4") {
            probe(
                "removed_free_end_corner",
                Point3::new(29.8, 8., 23.8),
                "Ok(Outside)",
            );
        }
        if case.starts_with("W3_")
            && let Some(radius) = selection["radius"].as_f64()
        {
            if label.starts_with("all ") || label.starts_with("seam") {
                let boss_face = boss(t, out);
                let boss_radius = match t.face(boss_face).unwrap().surface() {
                    FaceSurface::Cylinder(c) => c.radius(),
                    _ => unreachable!(),
                };
                probe(
                    "seam_added_material",
                    Point3::new(boss_radius + 0.2, -0.2, 12.),
                    "Ok(Inside)",
                );
                probe(
                    "seam_radius_witness",
                    Point3::new(boss_radius + 1., -0.5, 12.),
                    if radius < 3. {
                        "Ok(Outside)"
                    } else {
                        "Ok(Inside)"
                    },
                );
            }
            if label.starts_with("all ") || label.starts_with("round ") {
                probe(
                    "convex_radius_witness",
                    Point3::new(20., 1., 23.),
                    if radius < 2. {
                        "Ok(Inside)"
                    } else {
                        "Ok(Outside)"
                    },
                );
            }
        }
    }
    *rows.last_mut().unwrap() = json!({"label":label,"selection":selection,"before":before,"after":after,"source_after":source_after,
        "success":out.is_some(),"error":error,"engine":engine,"source_body_untouched":untouched,
        "error_code":error_code,"error_typed":error_typed,"intent_probes":intent_probes,
        "corner_trace":FAMILIES.lock().unwrap().clone(),
        "whole_arena_debug_equal":whole_arena,"arena_before":arena_before,"arena_after":arena_counts(t),"operation_ms":elapsed,"phase":"complete"});
    save(case, &json!({"case":case,"attempts":rows}));
    out
}

fn worker(case: &str) {
    let fixtures = fixtures();
    let name = if case == "sharp" || case == "A0" {
        "Gentle-Gecko.step"
    } else {
        "Gentle-Gecko-W-Fillet.step"
    };
    let mut t = Topology::new();
    let start = Instant::now();
    let solids = read_step(
        &std::fs::read_to_string(fixtures.join(name)).unwrap(),
        &mut t,
    )
    .unwrap();
    assert_eq!(solids.len(), 1);
    let import_ms = start.elapsed().as_secs_f64() * 1000.0;
    let mut s = solids[0];
    save(
        &format!("{case}_inventory"),
        &json!({"fixture":name,"import_ms":import_ms,"topology":inventory(&t,s)}),
    );
    let mut rows = vec![];
    // Acceptance starts at the fully rounded A0. Historical no-rim A1 rows
    // remain separately named supplementary probes for comparison.
    if case.starts_with("A2")
        || case.starts_with("A3")
        || case.starts_with("A4")
        || case == "A5"
        || case.starts_with("A6")
        || case.starts_with("A7")
        || case == "A8"
        || case == "cumulative"
        || case.starts_with("A1_quality")
    {
        s = add_rim(case, &mut t, s, &mut rows).expect("A0 R1 cap rim prerequisite");
    }
    match case {
        "cumulative" => cumulative(case, &mut t, s, &mut rows),
        "A1_quality_short" | "A1_quality_shoulder" => {
            // z26 directly, and the application's order: z61.2, then z26.
            let deltas: &[f64] = if case.ends_with("short") {
                &[-10.0]
            } else {
                &[25.2, -35.2]
            };
            let mut current = s;
            for &d in deltas {
                let f = cap(&t, current);
                let Some(out) = sampled_cap_move(case, &mut t, current, f, d, &mut rows) else {
                    break;
                };
                current = out;
            }
        }
        "reference" | "sharp" => save(
            case,
            &json!({"fixture":name,"import_ms":import_ms,"metrics":metrics(&t,s)}),
        ),
        "A0" => {
            let pairs = [
                ([6., 0., 0.], [6., 0., 24.]),
                ([6., 0., 24.], [30., 0., 24.]),
                ([0., 6., 24.], [6., 0., 24.]),
                ([0., 6., 24.], [0., 18., 24.]),
                ([0., 6., 0.], [0., 6., 24.]),
            ];
            let edges: Vec<_> = pairs
                .iter()
                .map(|(p, q)| {
                    solid_edges(&t, s)
                        .unwrap()
                        .into_iter()
                        .find(|&e| {
                            let (a, b) = endpoints(&t, e);
                            let eq = |a: Point3, p: &[f64; 3]| {
                                xyz(a).iter().zip(p).all(|(a, b)| near(*a, *b))
                            };
                            (eq(a, p) && eq(b, q)) || (eq(a, q) && eq(b, p))
                        })
                        .unwrap()
                })
                .collect();

            let rebuilt = attempt(
                case,
                "recorded five-edge R4",
                &mut t,
                s,
                json!({"edges":edges.iter().map(|e|e.index()).collect::<Vec<_>>(),"radius":4}),
                |t| fillet(t, s, &edges, 4.0),
                &mut rows,
            );
            if let Some(rebuilt) = rebuilt {
                add_rim(case, &mut t, rebuilt, &mut rows);
            }
        }
        "A0_cap_rim" => {
            let f = cap(&t, s);
            let edges = t
                .wire(t.face(f).unwrap().outer_wire())
                .unwrap()
                .edges()
                .iter()
                .map(remus_topology::OrientedEdge::edge)
                .collect::<Vec<_>>();
            attempt(
                case,
                "additional cap rim R1 (absent from fixture)",
                &mut t,
                s,
                json!({"radius":1}),
                |t| fillet(t, s, &edges, 1.0),
                &mut rows,
            );
        }
        "A1_short" | "A1_tall" | "A1_sequence" | "A1_shoulder" => {
            let ds = if case == "A1_short" {
                vec![-13.0]
            } else if case == "A1_shoulder" {
                vec![-10.0]
            } else if case == "A1_tall" {
                vec![25.2]
            } else {
                vec![-13.0, 38.2]
            };
            let mut current = s;
            for d in ds {
                let f = cap(&t, current);
                if let Some(out) = attempt(
                    case,
                    &format!("cap delta {d}"),
                    &mut t,
                    current,
                    json!({"face":f.index(),"delta":d}),
                    |t| {
                        push_pull::move_faces(t, current, &[f], d).map(|s| (s, "move_faces".into()))
                    },
                    &mut rows,
                ) {
                    current = out;
                } else {
                    // Exact-only shortening into the shoulder refuses typed;
                    // the sampled rows below build it.
                    rows.last_mut().unwrap()["allow_refusal"] = json!(d < 0.0);
                    if case == "A1_sequence" {
                        break;
                    }
                }
            }
        }
        "A1_rim_short" | "A1_rim_tall" | "A1_rim_shoulder" => {
            let face = cap(&t, s);
            let edges = t
                .wire(t.face(face).unwrap().outer_wire())
                .unwrap()
                .edges()
                .iter()
                .map(remus_topology::OrientedEdge::edge)
                .collect::<Vec<_>>();
            if let Some(rounded) = attempt(
                case,
                "prerequisite cap rim R1",
                &mut t,
                s,
                json!({"radius":1}),
                |t| fillet(t, s, &edges, 1.0),
                &mut rows,
            ) {
                let face = cap(&t, rounded);
                let d = if case.ends_with("short") {
                    -13.0
                } else if case.ends_with("shoulder") {
                    -10.0
                } else {
                    25.2
                };
                let moved = attempt(
                    case,
                    &format!("cap with R1 rim delta {d}"),
                    &mut t,
                    rounded,
                    json!({"face":face.index(),"delta":d}),
                    |t| {
                        push_pull::move_faces(t, rounded, &[face], d)
                            .map(|s| (s, "move_faces".into()))
                    },
                    &mut rows,
                );
                if moved.is_none() {
                    rows.last_mut().unwrap()["allow_refusal"] = json!(d < 0.0);
                }
            }
        }
        c if c.starts_with("A2") => {
            // `(vertical, side, from, to)`. The application's A2 drags the
            // convex band along x = 0 (axis Y) R4 -> 1.5 -> 1.9.
            let selections = if c == "A2_sequence" {
                vec![(false, false, 4.0, 1.5), (false, false, 1.5, 1.9)]
            } else {
                let vertical = c.contains("concave");
                let r = if vertical { 2.0 } else { 1.9 };
                vec![(vertical, c.ends_with('x'), 4.0, r)]
            };
            let mut current = s;
            for (vertical, side, from, r) in selections {
                let f = band_at(&t, current, vertical, side, from);
                if let Some(out) = attempt(
                    case,
                    &format!(
                        "{} side={side} R{from}->{r}",
                        if vertical { "concave" } else { "convex" }
                    ),
                    &mut t,
                    current,
                    json!({"face":f.index(),"expected_radius":from,"new_radius":r}),
                    |t| {
                        resize_blend::resize_blend(t, current, f, from, r)
                            .map(|b| (b.solid, "resize_blend".into()))
                    },
                    &mut rows,
                ) {
                    current = out;
                } else if c == "A2_sequence" {
                    break;
                }
            }
        }
        "A3" => {
            // After A2 (the x = 0 convex band at R1.9), R1 on the x = 0 /
            // y = 18 vertical edge: its chain turns through the band's end
            // arc onto the y = 18 top edge.
            let f = band_at(&t, s, false, false, 4.0);
            if let Some(a2) = attempt(
                case,
                "A2 convex x = 0 R4->1.9",
                &mut t,
                s,
                json!({"face":f.index(),"expected_radius":4,"new_radius":1.9}),
                |t| {
                    resize_blend::resize_blend(t, s, f, 4.0, 1.9)
                        .map(|b| (b.solid, "resize_blend".into()))
                },
                &mut rows,
            ) {
                let e = a3_edge(&t, a2);
                attempt(
                    case,
                    "A3 x = 0 / y = 18 vertical edge R1",
                    &mut t,
                    a2,
                    json!({"edge":e.index(),"radius":1}),
                    |t| fillet(t, a2, &[e], 1.0),
                    &mut rows,
                );
            }
        }
        c if c.starts_with("A4") || c == "A5" => {
            let e = end_edge(&t, s);
            let r = if c == "A5" {
                -1.0
            } else {
                c.rsplit('_').next().unwrap().parse::<f64>().unwrap()
            };
            attempt(
                case,
                c,
                &mut t,
                s,
                json!({"edge":e.index(),"value":r}),
                |t| {
                    if c.contains("chamfer") {
                        chamfer(t, s, &[e], r)
                    } else {
                        fillet(t, s, &[e], r)
                    }
                },
                &mut rows,
            );
        }
        c if c.starts_with("A6") || c == "A7_after_bottom" => {
            let outline = bottom(&t, s);
            let seed = outline
                .iter()
                .copied()
                .find(|&e| {
                    let (a, b) = endpoints(&t, e);
                    near(a.y(), 0.) && near(b.y(), 0.) && (a.x() - b.x()).abs() > 10.
                })
                .unwrap();
            let edges = if c == "A6_seed" {
                vec![seed]
            } else if c == "A6_chain" || c == "A7_after_bottom" {
                remus_blend::g1_chain::expand_g1_chain(&t, s, &[seed], Tolerance::new()).unwrap()
            } else {
                outline.clone()
            };
            let out = attempt(
                case,
                "bottom chamfer 0.8",
                &mut t,
                s,
                json!({"seed":seed.index(),"edges":edges.iter().map(|e|e.index()).collect::<Vec<_>>(),"bottom_outline":outline.iter().map(|e|e.index()).collect::<Vec<_>>()}),
                |t| chamfer(t, s, &edges, 0.8),
                &mut rows,
            );
            if c == "A7_after_bottom" {
                if let Some(out) = out {
                    diameters(case, &mut t, out, &mut rows);
                } else {
                    rows.push(json!({"label":"diameters after bottom","blocked":"bottom chamfer prerequisite refused"}));
                    save(case, &json!({"attempts":rows}));
                }
            }
        }
        "A7_sequence" => diameters(case, &mut t, s, &mut rows),
        "A8" => {
            let f = cap(&t, s);
            let snapshot = t.clone();
            let original = serialize_solid(&t, s).unwrap();
            let prepared_start = Instant::now();
            let mut prepared =
                remus_operations::prepared_move::PreparedFaceMove::prepare(&t, s, &[f]).unwrap();
            let preparation_ms = prepared_start.elapsed().as_secs_f64() * 1000.0;
            for d in [-13.0, 1.0, 25.2] {
                attempt(
                    case,
                    &format!("preview {d}, discard"),
                    &mut t,
                    s,
                    json!({"face":f.index(),"delta":d}),
                    |t| {
                        prepared
                            .move_faces(t, d)
                            .map(|b| (b.solid, "PreparedFaceMove::move_faces".into()))
                    },
                    &mut rows,
                );
                let start = Instant::now();
                t.restore_preserving_handle_slots(&snapshot);
                let restore_ms = start.elapsed().as_secs_f64() * 1000.0;
                rows.last_mut().unwrap()["cancel_restore_ms"] = json!(restore_ms);
                rows.last_mut().unwrap()["preparation_ms"] = json!(preparation_ms);
                rows.last_mut().unwrap()["cancel_bytes_equal"] =
                    json!(serialize_solid(&t, s).unwrap() == original);
                save(case, &json!({"attempts":rows}));
            }
        }
        _ => panic!("unknown case {case}"),
    }
}

fn diameters(case: &str, t: &mut Topology, s: SolidId, rows: &mut Vec<Value>) {
    let mut current = s;
    for diameter in [14.6, 17.0, 21.1, 14.7] {
        let face = boss(t, current);
        let snapshot = t.clone();
        let source = current;
        let source_diameter = match t.face(face).unwrap().surface() {
            FaceSurface::Cylinder(c) => 2.0 * c.radius(),
            _ => unreachable!(),
        };
        let out = attempt(
            case,
            &format!("diameter {diameter}"),
            t,
            source,
            json!({"face":face.index(),"source_diameter":source_diameter,"diameter":diameter,"preview_only":near(diameter,21.1)}),
            |t| {
                // The resizeCylindricalFace WASM binding owns this transaction.
                remus_topology::transaction::run_transacted(t, |t| {
                    push_pull::resize_cylindrical_face(t, source, face, diameter / 2.0)
                })
                .map(|s| (s, "resize_cylindrical_face".into()))
            },
            rows,
        );
        if near(diameter, 21.1) {
            t.restore_preserving_handle_slots(&snapshot);
        } else if let Some(out) = out {
            current = out;
        }
    }
}

fn add_rim(case: &str, t: &mut Topology, s: SolidId, rows: &mut Vec<Value>) -> Option<SolidId> {
    let f = cap(t, s);
    let edges = t
        .wire(t.face(f).unwrap().outer_wire())
        .unwrap()
        .edges()
        .iter()
        .map(remus_topology::OrientedEdge::edge)
        .collect::<Vec<_>>();
    attempt(
        case,
        "A0 R1 cap rim",
        t,
        s,
        json!({"radius":1}),
        |t| fillet(t, s, &edges, 1.),
        rows,
    )
}

fn cumulative(case: &str, t: &mut Topology, mut s: SolidId, rows: &mut Vec<Value>) {
    // The round-2 application order on one body, without the concave R2 seam
    // (no construction; its isolated row stays open). Commit only successful
    // candidates. Stop immediately on refusal or invalid geometry: later
    // operations must never run on a substituted fresh body.
    for step in 1..=7 {
        let out = match step {
            1 => {
                // Lengthen to z61.2, then shorten to z26 into the shoulder.
                let mut next = Some(s);
                for distance in [25.2, -35.2] {
                    let Some(current) = next else {
                        break;
                    };
                    let f = cap(t, current);
                    next = sampled_cap_move(case, t, current, f, distance, rows);
                    if rows
                        .last()
                        .is_some_and(|r| !geometry_errors(&r["after"]).is_empty())
                    {
                        next = None;
                        break;
                    }
                }
                next
            }
            2 => {
                let f = band_at(t, s, false, false, 4.0);
                attempt(
                    case,
                    "A2 convex x = 0 R4->1.9",
                    t,
                    s,
                    json!({"face":f.index(),"expected_radius":4,"new_radius":1.9}),
                    |t| {
                        resize_blend::resize_blend(t, s, f, 4., 1.9)
                            .map(|b| (b.solid, "resize_blend".into()))
                    },
                    rows,
                )
            }
            3 => {
                let e = a3_edge(t, s);
                attempt(
                    case,
                    "A3 x = 0 / y = 18 vertical edge R1",
                    t,
                    s,
                    json!({"edge":e.index(),"radius":1}),
                    |t| fillet(t, s, &[e], 1.),
                    rows,
                )
            }
            4 => {
                // The single free-end edge names its tangent chain; the chain
                // is chamfered.
                let e = end_edge(t, s);
                attempt(
                    case,
                    "A4 free-end chamfer 0.7 (single edge)",
                    t,
                    s,
                    json!({"edge":e.index(),"distance":0.7}),
                    |t| chamfer(t, s, &[e], 0.7),
                    rows,
                );
                rows.last_mut().unwrap()["allow_refusal"] = json!(true);
                let chain =
                    remus_blend::g1_chain::expand_g1_chain(t, s, &[e], Tolerance::new()).unwrap();
                attempt(
                    case,
                    "A4 free-end chamfer 0.7 (tangent chain)",
                    t,
                    s,
                    json!({"edges":chain.iter().map(|e|e.index()).collect::<Vec<_>>(),"distance":0.7}),
                    |t| chamfer(t, s, &chain, 0.7),
                    rows,
                )
            }
            5 => {
                let e = bottom(t, s)[0];
                let out = attempt(
                    case,
                    "A5 reject negative radius",
                    t,
                    s,
                    json!({"value":-1}),
                    |t| fillet(t, s, &[e], -1.),
                    rows,
                );
                rows.last_mut().unwrap()["allow_refusal"] = json!(true);
                if out.is_some() { None } else { Some(s) }
            }
            6 => {
                let outline = bottom(t, s);
                let seed = outline
                    .iter()
                    .copied()
                    .find(|&e| {
                        let (a, b) = endpoints(t, e);
                        near(a.y(), 0.) && near(b.y(), 0.) && (a.x() - b.x()).abs() > 10.
                    })
                    .unwrap();
                let edges = remus_blend::g1_chain::expand_g1_chain(t, s, &[seed], Tolerance::new())
                    .unwrap();
                attempt(
                    case,
                    "A6 bottom chamfer 0.8 (seed tangent chain)",
                    t,
                    s,
                    json!({"seed":seed.index(),"edges":edges.iter().map(|e|e.index()).collect::<Vec<_>>()}),
                    |t| chamfer(t, s, &edges, 0.8),
                    rows,
                )
            }
            7 => {
                diameters(case, t, s, rows);
                rows.last().filter(|r| r["success"] == true).map(|_| s)
            }
            _ => unreachable!(),
        };
        let valid = rows
            .last()
            .is_some_and(|r| geometry_errors(&r["after"]).is_empty());
        if out.is_none() || !valid {
            for blocked in step + 1..=7 {
                rows.push(json!({"label":format!("A{blocked}"),"blocked":format!("A{step} prerequisite failed")}));
            }
            save(case, &json!({"attempts":rows,"completed_A7":false}));
            return;
        }
        s = out.unwrap();
    }
    save(case, &json!({"attempts":rows,"completed_A7":true}));
}

/// The cap moved by `distance` with the application's sampled permission.
fn sampled_cap_move(
    case: &str,
    t: &mut Topology,
    s: SolidId,
    f: FaceId,
    distance: f64,
    rows: &mut Vec<Value>,
) -> Option<SolidId> {
    attempt(
        case,
        &format!("cap delta {distance} with sampled permission"),
        t,
        s,
        json!({"face":f.index(),"delta":distance,"stations":256}),
        |t| {
            remus_operations::face_move_quality::move_faces_with_quality(
                t,
                s,
                &[f],
                distance,
                Some(remus_operations::face_move_quality::FaceMoveApproximation { stations: 256 }),
            )
            .map(|b| {
                (
                    b.result.solid,
                    format!("move_faces_with_quality::{:?}", b.quality),
                )
            })
        },
        rows,
    )
}

/// The A3 edge: the block's x = 0 / y = 18 vertical edge.
fn a3_edge(t: &Topology, s: SolidId) -> EdgeId {
    solid_edges(t, s)
        .unwrap()
        .into_iter()
        .find(|&e| {
            let (a, b) = endpoints(t, e);
            (a.z() - b.z()).abs() > 10.0
                && near(a.x(), 0.)
                && near(b.x(), 0.)
                && near(a.y(), 18.)
                && near(b.y(), 18.)
        })
        .expect("x = 0 / y = 18 vertical edge")
}

// Re-identify surviving sharp support intersections after each earlier band.
// This avoids stale imported IDs, and works at every sharp boss radius/height.
fn network_edges(t: &Topology, s: SolidId, kind: &str) -> Vec<EdgeId> {
    let adj = AdjacencyIndex::build(t, s).unwrap();
    solid_edges(t, s)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let faces = adj.faces_for_edge(e);
            if faces.len() != 2 {
                return false;
            }
            let a = t.face(faces[0]).unwrap().surface();
            let b = t.face(faces[1]).unwrap().surface();
            let (p, q) = endpoints(t, e);
            let cylinder_plane = matches!(
                (a, b),
                (FaceSurface::Cylinder(_), FaceSurface::Plane { .. })
                    | (FaceSurface::Plane { .. }, FaceSurface::Cylinder(_))
            );
            match kind {
                "seam" => cylinder_plane && (p.z() - q.z()).abs() > 1.,
                "shoulder" => cylinder_plane && near(p.z(), 24.) && near(q.z(), 24.),
                "round" => {
                    matches!(
                        (a, b),
                        (FaceSurface::Plane { .. }, FaceSurface::Plane { .. })
                    ) && near(p.z(), 24.)
                        && near(q.z(), 24.)
                        && ((near(p.y(), 0.) && near(q.y(), 0.))
                            || (near(p.x(), 0.) && near(q.x(), 0.)))
                }
                "all" => {
                    cylinder_plane
                        && ((p.z() - q.z()).abs() > 1. || near(p.z(), 24.) && near(q.z(), 24.))
                        || matches!(
                            (a, b),
                            (FaceSurface::Plane { .. }, FaceSurface::Plane { .. })
                        ) && near(p.z(), 24.)
                            && near(q.z(), 24.)
                            && ((near(p.y(), 0.) && near(q.y(), 0.))
                                || (near(p.x(), 0.) && near(q.x(), 0.)))
                }
                _ => false,
            }
        })
        .collect()
}

fn spike(case: &str) {
    let parts = case.split('_').collect::<Vec<_>>();
    let (body, set) = (parts[1], parts[2]);
    let mut t = Topology::new();
    let mut s = read_step(
        &std::fs::read_to_string(fixtures().join("Gentle-Gecko.step")).unwrap(),
        &mut t,
    )
    .unwrap()[0];
    let mut rows = vec![];
    if body != "base" {
        let f = if body.starts_with('z') {
            cap(&t, s)
        } else {
            boss(&t, s)
        };
        let value = body[1..].parse::<f64>().unwrap();
        let out = attempt(
            case,
            "sharp exact edit",
            &mut t,
            s,
            if body.starts_with('z') {
                json!({"delta":value-36.})
            } else {
                json!({"diameter":value})
            },
            |t| {
                if body.starts_with('z') {
                    push_pull::move_faces(t, s, &[f], value - 36.)
                } else {
                    remus_topology::transaction::run_transacted(t, |t| {
                        push_pull::resize_cylindrical_face(t, s, f, value / 2.)
                    })
                }
                .map(|s| (s, "sharp exact edit".into()))
            },
            &mut rows,
        );
        if let Some(out) = out {
            s = out;
        } else if body.starts_with('d') {
            // The convenience resize dispatch only admits a quarter-wall cell.
            // W3 is a reconstruction spike, so also try the existing generic
            // exact carrier replacement. W2's A7 keeps the consumer resize API.
            rows.last_mut().unwrap()["allow_refusal"] = json!(true);
            let cylinder = match t.face(f).unwrap().surface() {
                FaceSurface::Cylinder(c) => c,
                _ => unreachable!(),
            };
            let replacement = remus_math::surfaces::CylindricalSurface::with_ref_dir(
                cylinder.origin(),
                cylinder.axis(),
                value / 2.,
                cylinder.x_axis(),
            )
            .unwrap();
            let out = attempt(
                case,
                "sharp exact carrier replacement",
                &mut t,
                s,
                json!({"diameter":value}),
                |t| {
                    remus_operations::replace_surface::replace_surface(
                        t,
                        s,
                        f,
                        FaceSurface::Cylinder(replacement),
                    )
                    .map(|b| (b.solid, "replace_surface".into()))
                },
                &mut rows,
            );
            if let Some(out) = out {
                s = out;
            } else {
                // All requested diameter variants grow the original sharp boss.
                // Fuse an exact coaxial cylinder into the actual imported solid;
                // no synthetic replacement of the source is involved.
                rows.last_mut().unwrap()["allow_refusal"] = json!(true);
                let expected = 12960. + 30. * std::f64::consts::PI * (value / 2.).powi(2);
                let out = attempt(
                    case,
                    "sharp exact boolean growth",
                    &mut t,
                    s,
                    json!({"diameter":value,"expected_volume":expected}),
                    |t| {
                        remus_topology::transaction::run_transacted(t, |t| {
                            let tool =
                                remus_operations::primitives::make_cylinder(t, value / 2., 36.)?;
                            remus_operations::boolean::boolean(
                                t,
                                remus_operations::boolean::BooleanOp::Fuse,
                                s,
                                tool,
                            )
                        })
                        .map(|s| (s, "exact boolean Fuse".into()))
                    },
                    &mut rows,
                );
                let Some(out) = out else {
                    return;
                };
                s = out;
            }
        } else {
            return;
        }
    }
    let round = if set.ends_with("1.5") {
        1.5
    } else if set.ends_with("1.9") {
        1.9
    } else {
        4.
    };
    let seam = if set.starts_with("seam") || set.starts_with("both") {
        2.
    } else {
        4.
    };
    let groups: Vec<(&str, f64)> = if set == "all4" {
        vec![("all", 4.)]
    } else if near(seam, 4.) {
        // The existing seam-band workflow: one continuous concave chain,
        // followed by the convex ridges running into that band.
        vec![("seam_shoulder", 4.), ("round", round)]
    } else if parts.get(3) == Some(&"shoulderfirst") {
        vec![("shoulder", 4.), ("seam", seam), ("round", round)]
    } else {
        vec![("seam", seam), ("shoulder", 4.), ("round", round)]
    };
    for (kind, r) in groups {
        let edges = if kind == "seam_shoulder" {
            let mut e = network_edges(&t, s, "seam");
            e.extend(network_edges(&t, s, "shoulder"));
            e
        } else {
            network_edges(&t, s, kind)
        };
        if edges.is_empty() {
            rows.push(
                json!({"label":kind,"blocked":"sharp support intersection no longer exists"}),
            );
            save(case, &json!({"attempts":rows}));
            return;
        }
        let out = attempt(
            case,
            &format!("{kind} R{r}"),
            &mut t,
            s,
            json!({"radius":r,"edges":edges.iter().map(|e|e.index()).collect::<Vec<_>>()}),
            |t| fillet(t, s, &edges, r),
            &mut rows,
        );
        let Some(out) = out else {
            return;
        };
        s = out;
    }
    add_rim(case, &mut t, s, &mut rows);
}
