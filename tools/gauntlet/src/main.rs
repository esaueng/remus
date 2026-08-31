//! Command-line entry point for the Remus robustness gauntlet.

use std::collections::BTreeMap;
use std::ffi::OsString;
use std::fs;
use std::io::{self, Write};
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Duration;

use remus_gauntlet::manifest::{
    ArchiveManifestConfig, FetchConfig, fetch_manifest, generate_archive_manifest, write_manifest,
};
use remus_gauntlet::{
    GauntletError, PipelineConfig, RunConfig, arg_is, process_model, run_models_isolated,
    write_outputs,
};
use remus_io::ImportLimits;

const USAGE: &str = "Usage:\n  remus-gauntlet run [--output DIR] [--timeout-ms N] [--deflection D] [--max-input-bytes N] [--max-model-entities N] MODEL.step...\n  remus-gauntlet fetch MANIFEST.json --cache DIR [--sample N --seed S] [--source-file URL PATH]... [--output-list PATH]\n  remus-gauntlet manifest-archive --archive PATH --output PATH --name NAME --url URL --license-class CLASS --id-prefix PREFIX --sample N --seed S\n\nThe run command writes models.jsonl, scoreboard.json, and scoreboard.md. Fetch verifies every byte into a content-addressed cache.\n";

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            let mut stderr = io::stderr().lock();
            let _ = writeln!(stderr, "remus-gauntlet: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), GauntletError> {
    let mut args = std::env::args_os();
    let _program = args.next();
    let Some(command) = args.next() else {
        return Err(GauntletError::message(USAGE));
    };
    let rest: Vec<_> = args.collect();
    if arg_is(&command, "run") {
        run_parent(&rest)
    } else if arg_is(&command, "fetch") {
        run_fetch(&rest)
    } else if arg_is(&command, "manifest-archive") {
        run_manifest_archive(&rest)
    } else if arg_is(&command, "worker") {
        run_worker(&rest)
    } else if arg_is(&command, "--help") || arg_is(&command, "-h") {
        io::stdout()
            .lock()
            .write_all(USAGE.as_bytes())
            .map_err(|error| GauntletError::message(error.to_string()))
    } else {
        Err(GauntletError::message(format!(
            "unknown command {}\n\n{USAGE}",
            command.to_string_lossy()
        )))
    }
}

fn run_fetch(args: &[OsString]) -> Result<(), GauntletError> {
    let mut manifest = None;
    let mut cache = None;
    let mut sample = None;
    let mut seed = 0_u64;
    let mut source_files = BTreeMap::new();
    let mut output_list = None;
    let mut index = 0;
    while index < args.len() {
        let argument = &args[index];
        if arg_is(argument, "--cache") {
            cache = Some(PathBuf::from(next_value(args, &mut index, "--cache")?));
        } else if arg_is(argument, "--sample") {
            sample = Some(parse_usize(
                next_value(args, &mut index, "--sample")?,
                "sample",
            )?);
        } else if arg_is(argument, "--seed") {
            seed = parse_u64(next_value(args, &mut index, "--seed")?, "seed")?;
        } else if arg_is(argument, "--source-file") {
            let url = parse_utf8(
                next_value(args, &mut index, "--source-file URL")?,
                "source URL",
            )?
            .to_owned();
            let path = PathBuf::from(next_value(args, &mut index, "--source-file PATH")?);
            if source_files.insert(url.clone(), path).is_some() {
                return Err(GauntletError::message(format!(
                    "duplicate --source-file URL {url}"
                )));
            }
        } else if arg_is(argument, "--output-list") {
            output_list = Some(PathBuf::from(next_value(
                args,
                &mut index,
                "--output-list",
            )?));
        } else if argument.to_string_lossy().starts_with('-') {
            return Err(GauntletError::message(format!(
                "unknown fetch option {}",
                argument.to_string_lossy()
            )));
        } else if manifest.replace(PathBuf::from(argument)).is_some() {
            return Err(GauntletError::message(
                "fetch accepts exactly one manifest path",
            ));
        }
        index += 1;
    }
    let manifest =
        manifest.ok_or_else(|| GauntletError::message("fetch requires MANIFEST.json"))?;
    let cache = cache.ok_or_else(|| GauntletError::message("fetch requires --cache DIR"))?;
    let fetched = fetch_manifest(
        &manifest,
        &FetchConfig {
            cache_dir: cache,
            sample,
            seed,
            source_files,
        },
    )?;
    if let Some(path) = output_list {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)
                .map_err(|error| GauntletError::message(error.to_string()))?;
        }
        let mut output = String::new();
        for model in &fetched {
            output.push_str(&model.path.to_string_lossy());
            output.push('\n');
        }
        fs::write(path, output).map_err(|error| GauntletError::message(error.to_string()))?;
    } else {
        let mut stdout = io::stdout().lock();
        for model in fetched {
            writeln!(stdout, "{}\t{}", model.id, model.path.display())
                .map_err(|error| GauntletError::message(error.to_string()))?;
        }
    }
    Ok(())
}

#[allow(clippy::too_many_lines)]
fn run_manifest_archive(args: &[OsString]) -> Result<(), GauntletError> {
    let mut archive = None;
    let mut output = None;
    let mut name = None;
    let mut url = None;
    let mut license_class = None;
    let mut id_prefix = None;
    let mut sample = None;
    let mut seed = None;
    let mut index = 0;
    while index < args.len() {
        let argument = &args[index];
        if arg_is(argument, "--archive") {
            archive = Some(PathBuf::from(next_value(args, &mut index, "--archive")?));
        } else if arg_is(argument, "--output") {
            output = Some(PathBuf::from(next_value(args, &mut index, "--output")?));
        } else if arg_is(argument, "--name") {
            name = Some(parse_utf8(next_value(args, &mut index, "--name")?, "name")?.to_owned());
        } else if arg_is(argument, "--url") {
            url = Some(parse_utf8(next_value(args, &mut index, "--url")?, "URL")?.to_owned());
        } else if arg_is(argument, "--license-class") {
            license_class = Some(
                parse_utf8(
                    next_value(args, &mut index, "--license-class")?,
                    "license class",
                )?
                .to_owned(),
            );
        } else if arg_is(argument, "--id-prefix") {
            id_prefix = Some(
                parse_utf8(next_value(args, &mut index, "--id-prefix")?, "id prefix")?.to_owned(),
            );
        } else if arg_is(argument, "--sample") {
            sample = Some(parse_usize(
                next_value(args, &mut index, "--sample")?,
                "sample",
            )?);
        } else if arg_is(argument, "--seed") {
            seed = Some(parse_u64(next_value(args, &mut index, "--seed")?, "seed")?);
        } else {
            return Err(GauntletError::message(format!(
                "unknown manifest-archive option {}",
                argument.to_string_lossy()
            )));
        }
        index += 1;
    }
    let archive = archive.ok_or_else(|| GauntletError::message("missing --archive"))?;
    let output = output.ok_or_else(|| GauntletError::message("missing --output"))?;
    let config = ArchiveManifestConfig {
        name: name.ok_or_else(|| GauntletError::message("missing --name"))?,
        id_prefix: id_prefix.ok_or_else(|| GauntletError::message("missing --id-prefix"))?,
        url: url.ok_or_else(|| GauntletError::message("missing --url"))?,
        license_class: license_class
            .ok_or_else(|| GauntletError::message("missing --license-class"))?,
        sample: sample.ok_or_else(|| GauntletError::message("missing --sample"))?,
        seed: seed.ok_or_else(|| GauntletError::message("missing --seed"))?,
    };
    let manifest = generate_archive_manifest(&archive, &config)?;
    write_manifest(&output, &manifest)
}

fn run_parent(args: &[OsString]) -> Result<(), GauntletError> {
    let parsed = parse_args(args, true)?;
    if parsed.models.is_empty() {
        return Err(GauntletError::message(
            "run requires at least one STEP model",
        ));
    }
    let executable =
        std::env::current_exe().map_err(|error| GauntletError::message(error.to_string()))?;
    let results = run_models_isolated(
        &executable,
        &parsed.models,
        RunConfig {
            pipeline: parsed.pipeline,
            model_timeout: parsed.timeout,
        },
    );
    write_outputs(&parsed.output, &results)
}

fn run_worker(args: &[OsString]) -> Result<(), GauntletError> {
    let parsed = parse_args(args, false)?;
    if parsed.models.len() != 1 {
        return Err(GauntletError::message(
            "worker requires exactly one STEP model",
        ));
    }
    let result = process_model(&parsed.models[0], parsed.pipeline);
    let mut stdout = io::stdout().lock();
    serde_json::to_writer(&mut stdout, &result)
        .map_err(|error| GauntletError::message(error.to_string()))?;
    stdout
        .write_all(b"\n")
        .map_err(|error| GauntletError::message(error.to_string()))
}

struct ParsedArgs {
    output: PathBuf,
    timeout: Duration,
    pipeline: PipelineConfig,
    models: Vec<PathBuf>,
}

fn parse_args(args: &[OsString], allow_parent_flags: bool) -> Result<ParsedArgs, GauntletError> {
    let mut output = PathBuf::from("gauntlet-results");
    let mut timeout_ms = 60_000_u64;
    let mut deflection = remus_gauntlet::DEFAULT_DEFLECTION;
    let mut limits = ImportLimits::default();
    let mut models = Vec::new();
    let mut index = 0;
    while index < args.len() {
        let argument = &args[index];
        if arg_is(argument, "--") {
            models.extend(args[index + 1..].iter().map(PathBuf::from));
            break;
        }
        if arg_is(argument, "--output") {
            if !allow_parent_flags {
                return Err(GauntletError::message("worker does not accept --output"));
            }
            output = PathBuf::from(next_value(args, &mut index, "--output")?);
        } else if arg_is(argument, "--timeout-ms") {
            if !allow_parent_flags {
                return Err(GauntletError::message(
                    "worker does not accept --timeout-ms",
                ));
            }
            timeout_ms = parse_u64(next_value(args, &mut index, "--timeout-ms")?, "timeout")?;
        } else if arg_is(argument, "--deflection") {
            deflection = parse_f64(next_value(args, &mut index, "--deflection")?, "deflection")?;
        } else if arg_is(argument, "--max-input-bytes") {
            limits.max_input_bytes = parse_usize(
                next_value(args, &mut index, "--max-input-bytes")?,
                "max input bytes",
            )?;
        } else if arg_is(argument, "--max-model-entities") {
            limits.max_model_entities = parse_usize(
                next_value(args, &mut index, "--max-model-entities")?,
                "max model entities",
            )?;
        } else if argument.to_string_lossy().starts_with('-') {
            return Err(GauntletError::message(format!(
                "unknown option {}",
                argument.to_string_lossy()
            )));
        } else {
            models.push(PathBuf::from(argument));
        }
        index += 1;
    }
    if !deflection.is_finite() || deflection <= 0.0 {
        return Err(GauntletError::message(
            "deflection must be finite and positive",
        ));
    }
    Ok(ParsedArgs {
        output,
        timeout: Duration::from_millis(timeout_ms),
        pipeline: PipelineConfig {
            import_limits: limits,
            deflection,
        },
        models,
    })
}

fn next_value<'a>(
    args: &'a [OsString],
    index: &mut usize,
    option: &str,
) -> Result<&'a OsString, GauntletError> {
    *index += 1;
    args.get(*index)
        .ok_or_else(|| GauntletError::message(format!("{option} requires a value")))
}

fn parse_u64(value: &OsString, name: &str) -> Result<u64, GauntletError> {
    value
        .to_str()
        .ok_or_else(|| GauntletError::message(format!("{name} must be UTF-8")))?
        .parse()
        .map_err(|_| GauntletError::message(format!("{name} must be an unsigned integer")))
}

fn parse_usize(value: &OsString, name: &str) -> Result<usize, GauntletError> {
    usize::try_from(parse_u64(value, name)?)
        .map_err(|_| GauntletError::message(format!("{name} is too large")))
}

fn parse_f64(value: &OsString, name: &str) -> Result<f64, GauntletError> {
    value
        .to_str()
        .ok_or_else(|| GauntletError::message(format!("{name} must be UTF-8")))?
        .parse()
        .map_err(|_| GauntletError::message(format!("{name} must be a number")))
}

fn parse_utf8<'a>(value: &'a OsString, name: &str) -> Result<&'a str, GauntletError> {
    value
        .to_str()
        .ok_or_else(|| GauntletError::message(format!("{name} must be UTF-8")))
}
