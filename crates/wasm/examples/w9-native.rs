//! Native facade half of the W9 preflight workflow (one process per attempt).

use std::io::{self, Read, Write};

use remus::{IoError, Model};
use remus_io::ImportLimits;
use serde::Deserialize;
use serde_json::json;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Case {
    schema_version: u32,
    id: String,
    data: String,
    max_input_bytes: usize,
    max_entities: usize,
    expected_code: String,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut input = String::new();
    io::stdin().read_to_string(&mut input)?;
    let case: Case = serde_json::from_str(&input)?;
    if case.schema_version != 1 || case.id.is_empty() || case.expected_code.is_empty() {
        return Err("unsupported or incomplete W9 case".into());
    }
    let mut model = Model::new();
    let empty = format!("{model:?}");
    let solid = model.make_box(2.0, 3.0, 4.0)?;
    if empty == format!("{model:?}") {
        return Err("state oracle failed the geometry mutation control".into());
    }
    let valid_step = model.write_step(&[solid])?;
    if model.read_step(&valid_step)?.len() != 1 {
        return Err("valid STEP import control failed".into());
    }
    let before = format!("{model:?}");
    let result = model.read_step_with_limits(
        &case.data,
        ImportLimits {
            max_input_bytes: case.max_input_bytes,
            max_model_entities: case.max_entities,
            ..ImportLimits::default()
        },
    );
    let (outcome, code, diagnostic) = match result {
        Err(error @ IoError::LimitExceeded { .. }) => (
            "typed_refusal",
            "resource_limit_exceeded",
            error.to_string(),
        ),
        Err(error @ IoError::ParseError { .. }) => {
            ("typed_refusal", "invalid_argument", error.to_string())
        }
        Err(error) => ("untyped_error", "unexpected_error", error.to_string()),
        Ok(_) => ("invalid_success", "unexpected_success", String::new()),
    };
    writeln!(
        io::stdout().lock(),
        "{}",
        json!({
            "id": case.id,
            "outcome": outcome,
            "code": code,
            "diagnostic": diagnostic,
            "unchanged": before == format!("{model:?}"),
            "snapshot_bytes": before.len(),
        })
    )?;
    Ok(())
}
