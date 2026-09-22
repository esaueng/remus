//! Native qualification runner: executes the bounded case set and emits one
//! O1.2d job.
//!
//! Parent mode (default) runs every case twice, each repetition in a fresh
//! child process under a wall-clock ceiling, then maps the two evidence
//! documents to one observation per case. Worker mode (`--worker <case>`)
//! executes a single case once and prints its evidence JSON; the parent
//! re-invokes this same binary for isolation.
//!
//! Exit status mirrors the scorecard CLI: 0 means a complete job was
//! emitted (kernel refusals inside are data, not failure), 2 means usage,
//! harness-identity, or incomplete-evidence failure. A timed-out or crashed
//! child never becomes an observation: the run records the attempt and
//! refuses to emit a partial job.

use std::io::Write;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use remus_vs_bench::qualification::{
    AttemptRecord, ChildOutcome, QualificationError, assemble_job, case_specs, check_harness_sha,
    execute_case, manifest_sha256, map_to_observation, run_case_in_child, write_json,
};

fn stderr_line(message: &str) {
    let _ = writeln!(std::io::stderr().lock(), "{message}");
}

fn worker_mode(case: &str) -> i32 {
    match execute_case(case) {
        Ok(evidence) => match serde_json::to_string(&evidence) {
            Ok(json) => {
                let _ = writeln!(std::io::stdout().lock(), "{json}");
                0
            }
            Err(e) => {
                stderr_line(&format!("worker serialise: {e}"));
                2
            }
        },
        Err(e) => {
            stderr_line(&format!("worker: {e}"));
            2
        }
    }
}

fn harness_sha_from_git() -> Option<String> {
    let output = std::process::Command::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    Some(String::from_utf8_lossy(&output.stdout).trim().to_string())
}

fn unix_secs() -> Result<u64, QualificationError> {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .map_err(|e| QualificationError::Child(format!("clock: {e}")))
}

struct Args {
    worker: Option<String>,
    out: Option<String>,
    attempts: Option<String>,
    timeout_ms: u64,
    harness_sha: Option<String>,
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        worker: None,
        out: None,
        attempts: None,
        timeout_ms: 60_000,
        harness_sha: None,
    };
    let mut raw = std::env::args().skip(1);
    while let Some(flag) = raw.next() {
        match flag.as_str() {
            "--worker" => {
                args.worker = Some(raw.next().ok_or("--worker needs a case id")?);
            }
            "--out" => {
                args.out = Some(raw.next().ok_or("--out needs a path")?);
            }
            "--attempts" => {
                args.attempts = Some(raw.next().ok_or("--attempts needs a path")?);
            }
            "--timeout-ms" => {
                let value = raw.next().ok_or("--timeout-ms needs a value")?;
                args.timeout_ms = value
                    .parse::<u64>()
                    .map_err(|_| format!("invalid --timeout-ms: {value:?}"))?;
            }
            "--harness-sha" => {
                args.harness_sha = Some(raw.next().ok_or("--harness-sha needs a value")?);
            }
            other => return Err(format!("unknown argument: {other:?}")),
        }
    }
    Ok(args)
}

#[allow(clippy::too_many_lines)]
fn parent_mode(args: &Args) -> i32 {
    let harness_sha = match &args.harness_sha {
        Some(sha) => sha.clone(),
        None => {
            if let Some(sha) = harness_sha_from_git() {
                sha
            } else {
                stderr_line("no git HEAD found; pass --harness-sha explicitly");
                return 2;
            }
        }
    };
    if let Err(e) = check_harness_sha(&harness_sha) {
        stderr_line(&format!("harness identity: {e}"));
        return 2;
    }
    let manifest_sha = match manifest_sha256() {
        Ok(sha) => sha,
        Err(e) => {
            stderr_line(&format!("manifest identity: {e}"));
            return 2;
        }
    };
    let kernel = format!("remus-native@{harness_sha}");
    let short = harness_sha[..7].to_string();
    let stamp = match unix_secs() {
        Ok(secs) => secs,
        Err(e) => {
            stderr_line(&format!("clock: {e}"));
            return 2;
        }
    };
    let run_id = format!("remus-native-qualification-{short}-{stamp}");
    let exe = match std::env::current_exe() {
        Ok(path) => path.to_string_lossy().into_owned(),
        Err(e) => {
            stderr_line(&format!("current exe: {e}"));
            return 2;
        }
    };
    let timeout = Duration::from_millis(args.timeout_ms);
    let specs = case_specs();
    let mut attempts: Vec<AttemptRecord> = Vec::new();
    let mut complete = true;
    let mut observations = Vec::new();
    let mut first_reps: Vec<remus_vs_bench::qualification::CaseEvidence> = Vec::new();
    for spec in &specs {
        let mut reps = Vec::new();
        for repetition in 0..2_u32 {
            let start = std::time::Instant::now();
            let outcome = run_case_in_child(&exe, spec.id, timeout, &[]);
            let elapsed = start.elapsed().as_secs_f64();
            match outcome {
                Ok(ChildOutcome::Evidence(evidence)) => {
                    attempts.push(AttemptRecord {
                        case: spec.id.to_string(),
                        repetition,
                        outcome: "evidence".to_string(),
                        elapsed_secs: elapsed,
                        evidence: Some(evidence.clone()),
                        detail: None,
                    });
                    reps.push((evidence, elapsed));
                }
                Ok(ChildOutcome::Timeout) => {
                    attempts.push(AttemptRecord {
                        case: spec.id.to_string(),
                        repetition,
                        outcome: "timeout".to_string(),
                        elapsed_secs: elapsed,
                        evidence: None,
                        detail: Some(format!(
                            "wall-clock ceiling {}ms expired; child killed",
                            args.timeout_ms
                        )),
                    });
                    complete = false;
                }
                Ok(ChildOutcome::Crash(detail)) => {
                    attempts.push(AttemptRecord {
                        case: spec.id.to_string(),
                        repetition,
                        outcome: "crash".to_string(),
                        elapsed_secs: elapsed,
                        evidence: None,
                        detail: Some(detail),
                    });
                    complete = false;
                }
                Err(e) => {
                    stderr_line(&format!("harness failure on {}: {e}", spec.id));
                    return 2;
                }
            }
        }
        if reps.len() == 2 {
            let median = f64::midpoint(reps[0].1, reps[1].1);
            let p95 = reps[0].1.max(reps[1].1);
            match map_to_observation(spec, &reps[0].0, &reps[1].0, &kernel, median, p95, &short) {
                Ok(observation) => {
                    observations.push(observation);
                    first_reps.push(reps[0].0.clone());
                }
                Err(e) => {
                    stderr_line(&format!("observation mapping for {}: {e}", spec.id));
                    complete = false;
                }
            }
        } else {
            complete = false;
        }
    }
    let attempts_json = serde_json::to_value(&attempts).unwrap_or(serde_json::Value::Null);
    if !complete {
        if let Some(path) = &args.attempts {
            let _ = write_json(path, &attempts_json);
        }
        stderr_line("incomplete evidence: refusing to emit a partial job");
        return 2;
    }
    let pairs: Vec<(
        &remus_vs_bench::qualification::CaseSpec,
        &remus_vs_bench::qualification::CaseEvidence,
    )> = specs.iter().zip(first_reps.iter()).collect();
    match assemble_job(
        &pairs,
        observations,
        &run_id,
        &harness_sha,
        &manifest_sha,
        &kernel,
        2,
    ) {
        Ok(job) => {
            if let Some(path) = &args.out {
                if let Err(e) = write_json(path, &job) {
                    stderr_line(&format!("write job: {e}"));
                    return 2;
                }
            } else if serde_json::to_writer_pretty(std::io::stdout().lock(), &job).is_err() {
                stderr_line("write job to stdout");
                return 2;
            } else {
                let _ = writeln!(std::io::stdout().lock());
            }
            if let Some(path) = &args.attempts
                && let Err(e) = write_json(path, &attempts_json)
            {
                stderr_line(&format!("write attempts: {e}"));
                return 2;
            }
            0
        }
        Err(e) => {
            stderr_line(&format!("assemble job: {e}"));
            2
        }
    }
}

fn main() -> std::process::ExitCode {
    let args = match parse_args() {
        Ok(args) => args,
        Err(e) => {
            stderr_line(&format!("usage: {e}"));
            return std::process::ExitCode::from(2);
        }
    };
    let code = if let Some(case) = &args.worker {
        worker_mode(case)
    } else {
        parent_mode(&args)
    };
    std::process::ExitCode::from(code as u8)
}
