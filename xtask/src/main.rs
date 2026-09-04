mod wasm;

use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(name = "xtask", about = "remus build automation")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Build the kernel and translator WASM packages, merge, and validate.
    WasmBuild {
        /// Disable SIMD optimizations (simd128 is enabled by default).
        #[arg(long)]
        no_simd: bool,

        /// Bundle the file-format translators into the kernel module as well
        /// (the pre-split single-module layout). Not what CI ships.
        #[arg(long)]
        kernel_io: bool,
    },

    /// Build, validate, and publish WASM package to npm.
    WasmPublish {
        /// Run npm publish with --dry-run.
        #[arg(long)]
        dry_run: bool,

        /// Disable SIMD optimizations (simd128 is enabled by default).
        #[arg(long)]
        no_simd: bool,
    },
}

/// Build, merge, and validate every package, then run the consumer tests
/// against the pair.
fn build_packages(simd: bool, kernel_io: bool) -> anyhow::Result<()> {
    wasm::check_tools()?;
    wasm::check_versions_match()?;
    for spec in wasm::packages(kernel_io) {
        wasm::build_both_targets(&spec, simd)?;
        wasm::merge_packages(&spec)?;
        wasm::validate_output(&spec)?;
    }
    wasm::run_smoke_test()?;
    wasm::run_installed_tarball_test()
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Command::WasmBuild { no_simd, kernel_io } => {
            build_packages(!no_simd, kernel_io)?;
            println!("\n✅ WASM build and package runtime tests complete.");
        }
        Command::WasmPublish { dry_run, no_simd } => {
            build_packages(!no_simd, false)?;
            wasm::publish(dry_run)?;
        }
    }

    Ok(())
}
