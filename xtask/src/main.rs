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

        /// Hand the built packages to CI's two required runtime jobs.
        /// Local and integrity builds run both suites unless explicitly deferred.
        #[arg(long)]
        defer_package_tests: bool,
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
fn build_packages(simd: bool, kernel_io: bool, defer_package_tests: bool) -> anyhow::Result<()> {
    wasm::check_tools()?;
    wasm::check_versions_match()?;
    wasm::test_size_policy()?;
    for spec in wasm::packages(kernel_io) {
        wasm::build_both_targets(&spec, simd)?;
        wasm::merge_packages(&spec)?;
        wasm::validate_output(&spec)?;
    }
    wasm::version_packages()?;
    if defer_package_tests {
        println!(
            "Package runtime tests deferred; both CI runtime jobs must pass before publication."
        );
        Ok(())
    } else {
        wasm::run_package_tests()
    }
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Command::WasmBuild {
            no_simd,
            kernel_io,
            defer_package_tests,
        } => {
            build_packages(!no_simd, kernel_io, defer_package_tests)?;
            if defer_package_tests {
                println!("\nWASM build complete; package runtime tests are pending.");
            } else {
                println!("\n✅ WASM build and package runtime tests complete.");
            }
        }
        Command::WasmPublish { dry_run, no_simd } => {
            build_packages(!no_simd, false, false)?;
            wasm::publish(dry_run)?;
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{Cli, Command};
    use clap::Parser;

    #[test]
    fn package_tests_are_required_by_default() {
        let cli = Cli::try_parse_from(["xtask", "wasm-build"]);
        assert!(matches!(
            cli.map(|cli| cli.command),
            Ok(Command::WasmBuild {
                defer_package_tests: false,
                ..
            })
        ));
    }

    #[test]
    fn deferral_is_explicit_and_unavailable_to_publish() {
        let cli = Cli::try_parse_from(["xtask", "wasm-build", "--defer-package-tests"]);
        assert!(matches!(
            cli.map(|cli| cli.command),
            Ok(Command::WasmBuild {
                defer_package_tests: true,
                ..
            })
        ));
        assert!(Cli::try_parse_from(["xtask", "wasm-publish", "--defer-package-tests"]).is_err());
    }
}
