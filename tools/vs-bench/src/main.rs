//! Read a job from stdin; emit a JSON report and a gate-sensitive exit status.
use std::io::{Read, Write};
use std::process::ExitCode;

fn run() -> Result<ExitCode, Box<dyn std::error::Error>> {
    let mut input = String::new();
    std::io::stdin().read_to_string(&mut input)?;
    let report = remus_vs_bench::evaluate_json(&input)?;
    serde_json::to_writer_pretty(std::io::stdout().lock(), &report)?;
    writeln!(std::io::stdout().lock())?;
    Ok(if report.passed {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    })
}

fn main() -> ExitCode {
    match run() {
        Ok(code) => code,
        Err(error) => {
            let _ = writeln!(std::io::stderr().lock(), "{error}");
            ExitCode::from(2)
        }
    }
}
