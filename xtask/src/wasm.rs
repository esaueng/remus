use anyhow::{bail, Context, Result};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Pinned wasm-bindgen-cli version. Must match `wasm-bindgen = "=0.2.126"` in
/// the workspace Cargo.toml.
const WASM_BINDGEN_VERSION: &str = "0.2.126";

/// Valid .wasm file size range (bytes). The upper bounds mirror the consumer
/// policy `KERNEL_WASM_POLICY` in esaueng/openzcad
/// `scripts/bundle-size-policy.mjs` (rationale in its
/// `docs/kernel-wasm-size-policy.md`): `rawReviewBytes` (9 MiB) warns and
/// requires a kernel-size review, `rawHardBytes` (10 MiB) fails. Change them
/// together with that policy, never independently.
const MIN_WASM_SIZE: u64 = 500_000;
const REVIEW_WASM_SIZE: u64 = 9 * 1024 * 1024;
const MAX_WASM_SIZE: u64 = 10 * 1024 * 1024;
const _: () = assert!(MIN_WASM_SIZE < REVIEW_WASM_SIZE && REVIEW_WASM_SIZE < MAX_WASM_SIZE);

/// Size window for one distributable `.wasm`.
#[derive(Clone, Copy, Debug)]
pub struct SizeBudget {
    min: u64,
    /// Inside the window but worth a review; `None` for lazily loaded modules.
    review: Option<u64>,
    max: u64,
}

/// One npm package produced by `cargo xtask wasm-build`.
#[derive(Clone, Copy, Debug)]
pub struct PackageSpec {
    /// Crate directory relative to the project root.
    crate_dir: &'static str,
    /// npm package name.
    name: &'static str,
    /// wasm-pack file stem (`<stem>_bg.wasm`, `<stem>.js`, ...).
    stem: &'static str,
    /// The exported class whose method count is sanity-checked.
    class: &'static str,
    min_methods: usize,
    size: SizeBudget,
    /// Extra cargo arguments passed after `--`.
    cargo_args: &'static [&'static str],
}

impl PackageSpec {
    fn pkg_dir(&self) -> Result<PathBuf> {
        Ok(project_root()?.join(self.crate_dir).join("pkg"))
    }

    fn pkg_node_dir(&self) -> Result<PathBuf> {
        Ok(project_root()?.join(self.crate_dir).join("pkg-node"))
    }

    fn node_entry(&self) -> String {
        format!("{}_node.cjs", self.stem)
    }
}

/// The kernel module as shipped: no file-format translators. They live in
/// [`TRANSLATORS`] and bodies cross between the two as exact arena documents.
pub const KERNEL: PackageSpec = PackageSpec {
    crate_dir: "crates/wasm",
    name: "remus-wasm",
    stem: "remus_wasm",
    class: "BrepKernel",
    // Based on ~185 methods in the current BrepKernel. Update when the API
    // surface changes significantly.
    min_methods: 170,
    size: SizeBudget {
        min: MIN_WASM_SIZE,
        review: Some(REVIEW_WASM_SIZE),
        max: MAX_WASM_SIZE,
    },
    cargo_args: &["--no-default-features"],
};

/// The pre-split single-module kernel with translators bundled in. An escape
/// hatch for consumers that cannot load a second module; not what CI ships.
pub const KERNEL_WITH_IO: PackageSpec = PackageSpec {
    cargo_args: &[],
    ..KERNEL
};

/// The file-format translator module, loaded only around import and export.
pub const TRANSLATORS: PackageSpec = PackageSpec {
    crate_dir: "crates/wasm-io",
    name: "remus-wasm-io",
    stem: "remus_wasm_io",
    class: "RemusIo",
    min_methods: 18,
    size: SizeBudget {
        min: MIN_WASM_SIZE,
        review: None,
        max: MAX_WASM_SIZE,
    },
    cargo_args: &[],
};

/// The packages one build produces, kernel first.
#[must_use]
pub fn packages(kernel_io: bool) -> [PackageSpec; 2] {
    [if kernel_io { KERNEL_WITH_IO } else { KERNEL }, TRANSLATORS]
}

fn project_root() -> Result<PathBuf> {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .map(Path::to_path_buf)
        .context("xtask must be located inside the project root")
}

fn run_cmd(cmd: &mut Command) -> Result<()> {
    let status = cmd
        .status()
        .with_context(|| format!("failed to run: {cmd:?}"))?;
    if !status.success() {
        bail!("command failed with {status}: {cmd:?}");
    }
    Ok(())
}

fn run_cmd_output(cmd: &mut Command) -> Result<String> {
    let output = cmd
        .output()
        .with_context(|| format!("failed to run: {cmd:?}"))?;
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        bail!("command failed with {}: {stderr}", output.status);
    }
    Ok(String::from_utf8_lossy(&output.stdout).trim().to_string())
}

fn command_exists(name: &str) -> bool {
    // Use `which` — standard on Linux/macOS where WASM builds run.
    Command::new("which")
        .arg(name)
        .output()
        .is_ok_and(|o| o.status.success())
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Verify required tools are installed and versions match.
pub fn check_tools() -> Result<()> {
    println!("Checking tools...");

    if !command_exists("wasm-pack") {
        bail!(
            "wasm-pack not found. Install with:\n  \
             cargo binstall wasm-pack --no-confirm\n  \
             or: cargo install wasm-pack --locked"
        );
    }

    // wasm-bindgen-cli version check
    if command_exists("wasm-bindgen") {
        let version = run_cmd_output(
            Command::new("wasm-bindgen").arg("--version"),
        )?;
        // Output is like "wasm-bindgen 0.2.126"
        let installed = version.split_whitespace().last().unwrap_or("");
        if installed != WASM_BINDGEN_VERSION {
            bail!(
                "wasm-bindgen-cli version mismatch: installed={installed}, \
                 required={WASM_BINDGEN_VERSION}\n  \
                 Fix: cargo binstall wasm-bindgen-cli@{WASM_BINDGEN_VERSION} --no-confirm"
            );
        }
        println!("  wasm-bindgen-cli {WASM_BINDGEN_VERSION} ok");
    } else {
        println!(
            "  warning: wasm-bindgen-cli not found (wasm-pack bundles its own, \
             but pinned version won't be verified)"
        );
    }

    Ok(())
}

/// Build one package's WASM for both bundler and nodejs targets.
pub fn build_both_targets(spec: &PackageSpec, simd: bool) -> Result<()> {
    let wasm_crate = project_root()?.join(spec.crate_dir);

    let mut rustflags = String::from("-Dwarnings");
    if simd {
        rustflags.push_str(" -C target-feature=+simd128");
    }

    println!("\nBuilding {} WASM (bundler target)...", spec.name);
    let mut bundler = Command::new("wasm-pack");
    bundler.args(["build", "--target", "bundler", "--release"]);
    bundler
        .args(["--out-dir", "pkg"])
        .current_dir(&wasm_crate)
        .env("RUSTFLAGS", &rustflags);
    if !spec.cargo_args.is_empty() {
        bundler.arg("--").args(spec.cargo_args);
    }
    run_cmd(&mut bundler).context("wasm-pack build (bundler) failed")?;

    println!("\nBuilding {} WASM (nodejs target)...", spec.name);
    let mut node = Command::new("wasm-pack");
    // Only the Node glue is merged into the distributable package; its WASM
    // is discarded in favour of the optimized bundler binary above.
    node.args([
        "build",
        "--target",
        "nodejs",
        "--release",
        "--no-opt",
    ]);
    node.args(["--out-dir", "pkg-node"])
        .current_dir(&wasm_crate)
        .env("RUSTFLAGS", &rustflags);
    if !spec.cargo_args.is_empty() {
        node.arg("--").args(spec.cargo_args);
    }
    run_cmd(&mut node).context("wasm-pack build (nodejs) failed")?;

    Ok(())
}

/// Merge nodejs target into bundler package with proper package.json exports.
pub fn merge_packages(spec: &PackageSpec) -> Result<()> {
    let root = project_root()?;
    let pkg = spec.pkg_dir()?;
    merge_at(&pkg, &spec.pkg_node_dir()?, spec)?;
    copy_package_metadata(&root, &pkg)
}

fn copy_package_metadata(root: &Path, pkg: &Path) -> Result<()> {
    let stale_mit_license = pkg.join("LICENSE-MIT");
    if stale_mit_license.exists() {
        fs::remove_file(&stale_mit_license).context("removing stale MIT package license")?;
    }
    let name = "LICENSE-APACHE";
    fs::copy(root.join(name), pkg.join(name))
        .with_context(|| format!("copying {name} into npm package"))?;
    Ok(())
}

/// Core merge logic, parameterised on directories for testability.
fn merge_at(pkg: &Path, pkg_node: &Path, spec: &PackageSpec) -> Result<()> {
    println!("\nMerging {} dual-target packages...", spec.name);

    if !pkg_node.exists() {
        bail!(
            "nodejs build output not found: {}\n  \
             Run `build_both_targets` first.",
            pkg_node.display()
        );
    }

    // Copy nodejs entry point, renamed to .cjs so Node treats it as CommonJS
    // even when package.json has "type": "module" (set by bundler target).
    let node_src = pkg_node.join(format!("{}.js", spec.stem));
    let node_dst = pkg.join(spec.node_entry());
    fs::copy(&node_src, &node_dst).with_context(|| {
        format!(
            "copying nodejs entry: {} -> {}",
            node_src.display(),
            node_dst.display()
        )
    })?;

    // Patch package.json
    let pkg_json_path = pkg.join("package.json");
    let raw = fs::read_to_string(&pkg_json_path).context("reading pkg/package.json")?;
    let mut pkg_json: serde_json::Value =
        serde_json::from_str(&raw).context("parsing pkg/package.json")?;

    patch_package_json(&mut pkg_json, spec)?;

    let output = serde_json::to_string_pretty(&pkg_json).context("serializing package.json")?;
    fs::write(&pkg_json_path, format!("{output}\n")).context("writing pkg/package.json")?;

    println!("  Merged package.json with dual-target exports");

    // Clean up pkg-node (no longer needed)
    if pkg_node.exists() {
        fs::remove_dir_all(pkg_node).context("removing pkg-node")?;
    }

    Ok(())
}

/// Apply dual-target fields to the wasm-pack-generated package.json.
fn patch_package_json(pkg_json: &mut serde_json::Value, spec: &PackageSpec) -> Result<()> {
    let obj = pkg_json
        .as_object_mut()
        .context("package.json is not an object")?;

    let node_entry = spec.node_entry();
    let module_entry = format!("{}.js", spec.stem);
    let wasm_entry = format!("{}_bg.wasm", spec.stem);
    obj.insert("name".into(), serde_json::json!(spec.name));
    obj.insert("main".into(), serde_json::json!(node_entry));
    obj.insert("module".into(), serde_json::json!(module_entry));

    obj.insert(
        "exports".into(),
        serde_json::json!({
            ".": {
                "node": format!("./{node_entry}"),
                "import": format!("./{module_entry}"),
                "default": format!("./{module_entry}")
            },
            format!("./{wasm_entry}"): format!("./{wasm_entry}")
        }),
    );

    // Ensure files array includes the node entry.
    {
        let files = obj
            .entry("files")
            .or_insert_with(|| serde_json::json!([]))
            .as_array_mut()
            .context("package.json files is not an array")?;

        files.retain(|entry| entry.as_str() != Some("LICENSE-MIT"));

        for entry in [node_entry.as_str(), "LICENSE-APACHE"] {
            let entry = serde_json::json!(entry);
            if !files.contains(&entry) {
                files.push(entry);
            }
        }
    }

    // Preserve the package's deterministic top-level layout while retaining
    // semantic insertion order inside the conditional exports object.
    let mut keys = obj.keys().cloned().collect::<Vec<_>>();
    keys.sort_unstable();
    let mut sorted = serde_json::Map::new();
    for key in keys {
        if let Some(value) = obj.remove(&key) {
            sorted.insert(key, value);
        }
    }
    *obj = sorted;

    Ok(())
}

/// Validate the output package meets all quality criteria.
pub fn validate_output(spec: &PackageSpec) -> Result<()> {
    validate_at(&spec.pkg_dir()?, spec)
}

/// Core validation logic, parameterised on the package directory for testability.
fn validate_at(pkg: &Path, spec: &PackageSpec) -> Result<()> {
    println!("\nValidating {} output...", spec.name);

    let mut errors = Vec::new();

    // 1. Required files exist
    let stem = spec.stem;
    let required_files = [
        format!("{stem}_bg.wasm"),
        format!("{stem}.js"),
        spec.node_entry(),
        format!("{stem}.d.ts"),
        "package.json".to_string(),
        "LICENSE-APACHE".to_string(),
    ];
    for file in &required_files {
        let path = pkg.join(file);
        if path.exists() {
            println!("  ok {file}");
        } else {
            errors.push(format!("missing required file: {file}"));
        }
    }
    if pkg.join("LICENSE-MIT").exists() {
        errors.push("stale LICENSE-MIT present in Apache-only package".into());
    }

    // 2. WASM binary size
    let wasm_path = pkg.join(format!("{stem}_bg.wasm"));
    if wasm_path.exists() {
        let size = fs::metadata(&wasm_path)?.len();
        if let Some(error) = wasm_size_error(size, &spec.size) {
            errors.push(error);
        } else if let Some(warning) = wasm_size_warning(size, &spec.size) {
            println!("  WARN {warning}");
        } else {
            println!("  ok .wasm size: {:.1} KB", size as f64 / 1024.0);
        }
    }

    // 3. Type completeness
    let dts_path = pkg.join(format!("{stem}.d.ts"));
    if dts_path.exists() {
        let dts = fs::read_to_string(&dts_path)?;
        let class_export = format!("export class {}", spec.class);
        if !dts.contains(&class_export) {
            errors.push(format!("d.ts missing '{class_export}'"));
        }
        let method_count = count_dts_methods(&dts);
        if method_count < spec.min_methods {
            errors.push(format!(
                "d.ts has only {method_count} methods (expected >= {})",
                spec.min_methods
            ));
        } else {
            println!("  ok d.ts methods: {method_count}");
        }
    }

    // 4. package.json checks — collect errors rather than short-circuiting
    //    so all issues are reported together.
    let pkg_json_path = pkg.join("package.json");
    if pkg_json_path.exists() {
        match fs::read_to_string(&pkg_json_path)
            .context("reading package.json")
            .and_then(|s| serde_json::from_str(&s).context("parsing package.json"))
        {
            Ok(pkg_json) => validate_package_json(&pkg_json, spec, &mut errors),
            Err(e) => errors.push(format!("package.json unreadable/invalid: {e}")),
        }
    }

    if errors.is_empty() {
        println!("\n  All validation checks passed");
        Ok(())
    } else {
        let msg = errors
            .iter()
            .map(|e| format!("  FAIL {e}"))
            .collect::<Vec<_>>()
            .join("\n");
        bail!("Validation failed:\n{msg}");
    }
}

/// Count class method declarations in a wasm-bindgen .d.ts file.
/// Matches indented lines like `  methodName(...): ReturnType;` but excludes
/// top-level `export function ...` lines which are module-level bindings, not
/// class methods.
fn count_dts_methods(dts: &str) -> usize {
    dts.lines()
        .filter(|l| {
            let trimmed = l.trim();
            !trimmed.starts_with("export ")
                && trimmed
                    .chars()
                    .next()
                    .is_some_and(|c| c.is_ascii_lowercase())
                && trimmed.contains('(')
        })
        .count()
}

fn wasm_size_error(size: u64, budget: &SizeBudget) -> Option<String> {
    if size < budget.min {
        Some(format!(".wasm too small: {size} bytes (min {})", budget.min))
    } else if size > budget.max {
        Some(format!(".wasm too large: {size} bytes (max {})", budget.max))
    } else {
        None
    }
}

/// Sizes inside the hard budget but above the consumer's review threshold.
fn wasm_size_warning(size: u64, budget: &SizeBudget) -> Option<String> {
    let review = budget.review?;
    (review < size && size <= budget.max).then(|| {
        format!(
            ".wasm size {size} bytes exceeds the review threshold ({review}); \
             hard limit {}. A kernel-size review is required before shipping",
            budget.max
        )
    })
}

/// The two packages ship in lockstep: refuse to build them at different versions.
pub fn check_versions_match() -> Result<()> {
    let root = project_root()?;
    let versions = [KERNEL, TRANSLATORS]
        .iter()
        .map(|spec| {
            let manifest = fs::read_to_string(root.join(spec.crate_dir).join("Cargo.toml"))
                .with_context(|| format!("reading {} manifest", spec.name))?;
            manifest_version(&manifest)
                .with_context(|| format!("{} manifest has no version", spec.name))
        })
        .collect::<Result<Vec<_>>>()?;
    if versions[0] != versions[1] {
        bail!(
            "package versions differ: {}={} but {}={}; bump both crates together",
            KERNEL.name,
            versions[0],
            TRANSLATORS.name,
            versions[1]
        );
    }
    println!("  ok both packages at version {}", versions[0]);
    Ok(())
}

/// The `[package] version` of a manifest. Good enough for the two manifests
/// this tool owns, where the field is a plain quoted literal.
fn manifest_version(manifest: &str) -> Option<String> {
    manifest
        .lines()
        .map(str::trim)
        .find_map(|line| line.strip_prefix("version = \"")?.strip_suffix('"'))
        .map(str::to_owned)
}

/// Validate package.json fields, pushing errors into the collector.
fn validate_package_json(
    pkg_json: &serde_json::Value,
    spec: &PackageSpec,
    errors: &mut Vec<String>,
) {
    let get_str = |key: &str| pkg_json.get(key).and_then(|v| v.as_str()).unwrap_or("");

    let name = get_str("name");
    if name != spec.name {
        errors.push(format!(
            "package.json name is '{name}', expected '{}'",
            spec.name
        ));
    } else {
        println!("  ok name: {name}");
    }

    let main = get_str("main");
    let node_entry = spec.node_entry();
    if main != node_entry {
        errors.push(format!(
            "package.json main is '{main}', expected '{node_entry}'"
        ));
    } else {
        println!("  ok main: {main}");
    }

    if let Some(exports) = pkg_json.get("exports") {
        if let Some(dot) = exports.get(".") {
            for key in &["node", "import", "default"] {
                if dot.get(key).is_none() {
                    errors.push(format!("exports[\".\"] missing key: {key}"));
                }
            }
            if let Some(conditions) = dot.as_object() {
                let order = conditions.keys().map(String::as_str).collect::<Vec<_>>();
                if order != ["node", "import", "default"] {
                    errors.push(format!(
                        "exports[\".\"] condition order is {order:?}, expected node/import/default"
                    ));
                } else {
                    println!("  ok exports[\".\"] order: node/import/default");
                }
            } else {
                errors.push("exports[\".\"] is not an object".into());
            }
        } else {
            errors.push("exports missing \".\" entry".into());
        }
    } else {
        errors.push("package.json missing exports".into());
    }

    if let Some(files) = pkg_json.get("files").and_then(|v| v.as_array()) {
        for required in [node_entry.as_str(), "LICENSE-APACHE"] {
            if !files.iter().any(|v| v.as_str() == Some(required)) {
                errors.push(format!("files array missing '{required}'"));
            } else {
                println!("  ok files includes {required}");
            }
        }
    } else {
        errors.push("package.json missing files array".into());
    }
}

/// Run the Node.js smoke test.
pub fn run_smoke_test() -> Result<()> {
    println!("\nRunning smoke test...");

    let script = project_root()?.join("scripts/test-wasm-smoke.mjs");
    if !script.exists() {
        bail!("Smoke test script not found: {}", script.display());
    }

    run_cmd(Command::new("node").arg(&script)).context("smoke test failed")?;

    println!("  Smoke test passed");
    Ok(())
}

/// Pack, install, and test the npm artifact through normal Node resolution.
pub fn run_installed_tarball_test() -> Result<()> {
    println!("\nRunning installed-tarball consumer test...");

    let script = project_root()?.join("scripts/test-wasm-tarball-consumer.mjs");
    if !script.exists() {
        bail!(
            "Tarball consumer test script not found: {}",
            script.display()
        );
    }

    run_cmd(Command::new("node").arg(&script)).context("installed-tarball consumer test failed")?;

    println!("  Installed-tarball consumer test passed");
    Ok(())
}

/// Publish the kernel WASM package to npm.
pub fn publish(dry_run: bool) -> Result<()> {
    let pkg = KERNEL.pkg_dir()?;

    let tag_name = std::env::var("TAG_NAME")
        .context("TAG_NAME env var not set — required for publish")?;
    let tag_version = tag_name.strip_prefix('v').unwrap_or(&tag_name);

    let pkg_json: serde_json::Value = serde_json::from_str(
        &fs::read_to_string(pkg.join("package.json"))
            .context("reading package.json for version check")?,
    )?;
    let pkg_version = pkg_json
        .get("version")
        .and_then(|v| v.as_str())
        .unwrap_or("");

    if pkg_version != tag_version {
        bail!("Version mismatch: package.json={pkg_version}, tag={tag_version}");
    }

    println!("\nPublishing remus-wasm@{pkg_version}...");

    let mut cmd = Command::new("npm");
    cmd.args(["publish", "--provenance", "--access", "public"]);
    if dry_run {
        cmd.arg("--dry-run");
    }
    cmd.current_dir(&pkg);

    run_cmd(&mut cmd).context("npm publish failed")?;

    if dry_run {
        println!("  Dry run complete (nothing published)");
    } else {
        println!("  Published remus-wasm@{pkg_version}");
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use serde_json::json;

    #[test]
    fn wasm_bindgen_cli_version_matches_workspace_dependency() {
        let cargo_toml = fs::read_to_string(project_root().unwrap().join("Cargo.toml")).unwrap();
        let expected = format!("wasm-bindgen = \"={WASM_BINDGEN_VERSION}\"");

        assert!(
            cargo_toml.contains(&expected),
            "workspace dependency must contain {expected}"
        );
    }

    #[test]
    fn wasm_size_budget_accepts_its_exact_boundaries() {
        assert!(wasm_size_error(MIN_WASM_SIZE, &KERNEL.size).is_none());
        assert!(wasm_size_error(MAX_WASM_SIZE, &KERNEL.size).is_none());
    }

    #[test]
    fn wasm_size_budget_rejects_binaries_outside_its_bounds() {
        assert!(
            wasm_size_error(MIN_WASM_SIZE - 1, &KERNEL.size)
                .is_some_and(|error| error.contains("too small"))
        );
        assert!(
            wasm_size_error(MAX_WASM_SIZE + 1, &KERNEL.size)
                .is_some_and(|error| error.contains("too large"))
        );
    }

    #[test]
    fn wasm_size_review_warning_covers_only_the_band_below_the_hard_limit() {
        assert!(wasm_size_warning(REVIEW_WASM_SIZE, &KERNEL.size).is_none());
        assert!(
            wasm_size_warning(REVIEW_WASM_SIZE + 1, &KERNEL.size)
                .is_some_and(|warning| warning.contains("review threshold"))
        );
        assert!(wasm_size_warning(MAX_WASM_SIZE, &KERNEL.size).is_some());
        // Above the hard limit the error path owns the message.
        assert!(wasm_size_warning(MAX_WASM_SIZE + 1, &KERNEL.size).is_none());
        assert!(wasm_size_error(MAX_WASM_SIZE + 1, &KERNEL.size).is_some());
    }

    #[test]
    fn translator_module_has_no_review_band() {
        // Loaded lazily, so it is outside the consumer's kernel-size policy.
        assert!(wasm_size_warning(REVIEW_WASM_SIZE + 1, &TRANSLATORS.size).is_none());
        assert!(wasm_size_error(MAX_WASM_SIZE + 1, &TRANSLATORS.size).is_some());
    }

    #[test]
    fn shipped_kernel_excludes_translators_but_escape_hatch_keeps_them() {
        assert_eq!(KERNEL.cargo_args, ["--no-default-features"]);
        assert!(KERNEL_WITH_IO.cargo_args.is_empty());
        assert_eq!(packages(false)[0].cargo_args, KERNEL.cargo_args);
        assert!(packages(true)[0].cargo_args.is_empty());
        assert_eq!(packages(false)[1].name, TRANSLATORS.name);
    }

    #[test]
    fn kernel_and_translator_crates_share_one_version() {
        check_versions_match().unwrap();
        assert_eq!(manifest_version("[package]\nversion = \"1.2.3\"\n").as_deref(), Some("1.2.3"));
        assert!(manifest_version("[package]\nname = \"x\"\n").is_none());
    }

    #[test]
    fn publish_workflow_refreshes_both_packages() {
        let root = project_root().unwrap();
        let workflow = fs::read_to_string(root.join(".github/workflows/publish.yml")).unwrap();
        for spec in [KERNEL, TRANSLATORS] {
            let pkg_dir = format!("{}/pkg", spec.crate_dir);
            assert!(
                workflow.contains(&pkg_dir),
                "publish.yml must archive and stage {pkg_dir}"
            );
        }
    }

    #[test]
    fn consumer_workflows_cannot_skip_wasm_optimization() {
        let root = project_root().unwrap();
        for relative in [
            ".github/workflows/ci.yml",
            ".github/workflows/publish.yml",
            ".github/workflows/openzcad-wasm-release.yml",
        ] {
            let workflow = fs::read_to_string(root.join(relative)).unwrap();
            assert!(
                workflow.contains("cargo xtask wasm-build"),
                "{relative} must use the validated package builder"
            );
            assert!(
                !workflow.contains("wasm-build --skip-opt"),
                "{relative} must not bypass distributable WASM optimization"
            );
        }
    }

    // -- patch_package_json tests -----------------------------------------

    #[test]
    fn patch_sets_all_required_fields() {
        let mut pkg = json!({
            "name": "wasm-pack-default",
            "version": "0.5.3",
            "files": ["remus_wasm_bg.wasm", "remus_wasm.js", "remus_wasm.d.ts", "LICENSE-MIT"],
            "module": "remus_wasm.js",
            "types": "remus_wasm.d.ts",
            "sideEffects": ["./snippets/*"]
        });

        patch_package_json(&mut pkg, &KERNEL).unwrap();

        assert_eq!(pkg["name"], "remus-wasm");
        assert_eq!(pkg["main"], "remus_wasm_node.cjs");
        assert_eq!(pkg["module"], "remus_wasm.js");
        assert_eq!(pkg["exports"]["."]["node"], "./remus_wasm_node.cjs");
        assert_eq!(pkg["exports"]["."]["import"], "./remus_wasm.js");
        assert_eq!(pkg["exports"]["."]["default"], "./remus_wasm.js");
        let export_order = pkg["exports"]["."]
            .as_object()
            .unwrap()
            .keys()
            .map(String::as_str)
            .collect::<Vec<_>>();
        assert_eq!(export_order, ["node", "import", "default"]);

        let files = pkg["files"].as_array().unwrap();
        assert!(files.contains(&json!("remus_wasm_node.cjs")));
        assert!(files.contains(&json!("LICENSE-APACHE")));
        assert!(!files.contains(&json!("LICENSE-MIT")));
        // Original files preserved
        assert!(files.contains(&json!("remus_wasm_bg.wasm")));
    }

    #[test]
    fn patch_names_the_translator_package_by_its_own_stem() {
        let mut pkg = json!({ "name": "wasm-pack-default", "files": ["remus_wasm_io_bg.wasm"] });
        patch_package_json(&mut pkg, &TRANSLATORS).unwrap();
        assert_eq!(pkg["name"], "remus-wasm-io");
        assert_eq!(pkg["main"], "remus_wasm_io_node.cjs");
        assert_eq!(pkg["module"], "remus_wasm_io.js");
        assert_eq!(pkg["exports"]["."]["node"], "./remus_wasm_io_node.cjs");
        assert_eq!(
            pkg["exports"]["./remus_wasm_io_bg.wasm"],
            "./remus_wasm_io_bg.wasm"
        );
        let mut errors = Vec::new();
        validate_package_json(&pkg, &TRANSLATORS, &mut errors);
        assert!(errors.is_empty(), "unexpected errors: {errors:?}");
        validate_package_json(&pkg, &KERNEL, &mut errors);
        assert!(errors.iter().any(|error| error.contains("remus-wasm-io")));
    }

    #[test]
    fn patch_does_not_duplicate_node_entry() {
        let mut pkg = json!({
            "files": ["remus_wasm_node.cjs", "other.js"]
        });

        patch_package_json(&mut pkg, &KERNEL).unwrap();

        assert_eq!(pkg["files"].as_array().unwrap().len(), 3);
    }

    #[test]
    fn patch_creates_files_array_if_missing() {
        let mut pkg = json!({ "name": "test" });

        patch_package_json(&mut pkg, &KERNEL).unwrap();

        let files = pkg["files"].as_array().unwrap();
        assert_eq!(files.len(), 2);
        assert_eq!(files[0], "remus_wasm_node.cjs");
    }

    // -- validate_package_json tests --------------------------------------

    #[test]
    fn validate_detects_wrong_name() {
        let pkg = json!({
            "name": "wrong-name",
            "main": "remus_wasm_node.cjs",
            "exports": { ".": { "node": "x", "import": "x", "default": "x" } },
            "files": ["remus_wasm_node.cjs"]
        });
        let mut errors = Vec::new();
        validate_package_json(&pkg, &KERNEL, &mut errors);

        assert!(errors.iter().any(|e| e.contains("wrong-name")));
    }

    #[test]
    fn validate_detects_missing_exports() {
        let pkg = json!({
            "name": "remus-wasm",
            "main": "remus_wasm_node.cjs",
            "files": ["remus_wasm_node.cjs"]
        });
        let mut errors = Vec::new();
        validate_package_json(&pkg, &KERNEL, &mut errors);

        assert!(errors.iter().any(|e| e.contains("missing exports")));
    }

    #[test]
    fn validate_detects_unsafe_export_condition_order() {
        let pkg = json!({
            "name": "remus-wasm",
            "main": "remus_wasm_node.cjs",
            "exports": {
                ".": {
                    "default": "./remus_wasm.js",
                    "import": "./remus_wasm.js",
                    "node": "./remus_wasm_node.cjs"
                }
            },
            "files": ["remus_wasm_node.cjs", "LICENSE-APACHE"]
        });
        let mut errors = Vec::new();
        validate_package_json(&pkg, &KERNEL, &mut errors);

        assert!(
            errors.iter().any(|error| error.contains("condition order")),
            "unexpected errors: {errors:?}"
        );
    }

    #[test]
    fn validate_passes_correct_json() {
        let mut pkg = json!({
            "name": "wasm-pack-default",
            "version": "0.5.3",
            "files": ["remus_wasm_bg.wasm"]
        });
        patch_package_json(&mut pkg, &KERNEL).unwrap();

        let mut errors = Vec::new();
        validate_package_json(&pkg, &KERNEL, &mut errors);

        assert!(errors.is_empty(), "unexpected errors: {errors:?}");
    }

    // -- count_dts_methods tests ------------------------------------------

    #[test]
    fn count_methods_in_synthetic_dts() {
        let mut dts = String::from("export class BrepKernel {\n");
        for i in 0..200 {
            dts.push_str(&format!("  method{i}(): void;\n"));
        }
        dts.push_str("}\n");

        assert_eq!(count_dts_methods(&dts), 200);
    }

    #[test]
    fn count_methods_ignores_non_method_lines() {
        let dts = "\
export class BrepKernel {
  free(): void;
  /**
   * Some documentation
   */
  makeBox(w: number, h: number, d: number): number;
  readonly positions: Float64Array;
}
";
        // free() and makeBox() are methods; readonly and doc comments are not
        assert_eq!(count_dts_methods(dts), 2);
    }

    // -- merge_at integration test ----------------------------------------

    #[test]
    fn merge_at_copies_and_patches_correctly() {
        let dir = tempfile::tempdir().unwrap();
        let pkg = dir.path().join("pkg");
        let pkg_node = dir.path().join("pkg-node");
        fs::create_dir_all(&pkg).unwrap();
        fs::create_dir_all(&pkg_node).unwrap();

        // Create mock package.json (as wasm-pack would generate)
        let initial = json!({
            "name": "remus-wasm",
            "version": "0.5.3",
            "files": ["remus_wasm_bg.wasm", "remus_wasm.js", "remus_wasm.d.ts"],
            "module": "remus_wasm.js"
        });
        fs::write(
            pkg.join("package.json"),
            serde_json::to_string_pretty(&initial).unwrap(),
        )
        .unwrap();

        // Create mock nodejs entry
        fs::write(pkg_node.join("remus_wasm.js"), "// node CJS entry").unwrap();

        merge_at(&pkg, &pkg_node, &KERNEL).unwrap();

        // .cjs file was created
        assert!(pkg.join("remus_wasm_node.cjs").exists());
        let content = fs::read_to_string(pkg.join("remus_wasm_node.cjs")).unwrap();
        assert_eq!(content, "// node CJS entry");

        // pkg-node was cleaned up
        assert!(!pkg_node.exists());

        // package.json was patched
        let result: serde_json::Value =
            serde_json::from_str(&fs::read_to_string(pkg.join("package.json")).unwrap()).unwrap();
        assert_eq!(result["name"], "remus-wasm");
        assert_eq!(result["main"], "remus_wasm_node.cjs");
        assert_eq!(result["exports"]["."]["node"], "./remus_wasm_node.cjs");
    }

    #[test]
    fn merge_at_fails_if_pkg_node_missing() {
        let dir = tempfile::tempdir().unwrap();
        let pkg = dir.path().join("pkg");
        let pkg_node = dir.path().join("pkg-node");
        fs::create_dir_all(&pkg).unwrap();
        // pkg_node intentionally not created

        let result = merge_at(&pkg, &pkg_node, &KERNEL);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("nodejs build output not found"), "got: {err}");
    }
}
