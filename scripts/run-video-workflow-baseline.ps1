# Native Windows GNU runner for crates/io/tests/video_workflow_baseline.rs.
# Uses the installed Rust linker, without a C compiler or benchmark dev-deps.
# All generated tools, manifests and evidence stay under target/.
param(
    [string]$Fixtures = (Join-Path $PSScriptRoot '../crates/io/tests/data'),
    [string]$OutputDirectory,
    [string]$Case,
    [ValidateSet('acceptance', 'cumulative', 'spike', 'all')][string]$Mode = 'acceptance',
    [switch]$CompileOnly,
    [switch]$Lint
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
Push-Location $repo
$saved = @{}
$names = @('PATH', 'RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'REMUS_BASELINE_LLD', 'REMUS_VIDEO_FIXTURES', 'REMUS_VIDEO_OUTPUT', 'REMUS_VIDEO_CASE', 'REMUS_VIDEO_MODE')
foreach ($name in $names) { $saved[$name] = [Environment]::GetEnvironmentVariable($name, 'Process') }
try {
    $cargoBin = Join-Path $env:USERPROFILE '.cargo\bin'
    $env:PATH = "$cargoBin;$env:PATH"
    $sysroot = (& rustc --print sysroot).Trim()
    $hostTriple = ((& rustc -vV | Select-String '^host: ').ToString() -replace '^host: ', '').Trim()
    if ($hostTriple -ne 'x86_64-pc-windows-gnu') { throw 'This helper requires the Windows x64 GNU Rust toolchain; otherwise use the cargo command in the test header.' }
    $lld = Join-Path $sysroot "lib\rustlib\$hostTriple\bin\rust-lld.exe"
    $tools = Join-Path $repo 'target\native-tools'
    $runner = Join-Path $repo 'target\video-runner'
    New-Item -ItemType Directory -Force $tools, $runner | Out-Null
    $env:REMUS_BASELINE_LLD = $lld
    # rustc's GNU raw-dylib import-library generator uses dlltool's small CLI.
    # Translate that CLI to the same installed LLD's COFF librarian mode.
    @'
use std::process::{Command, exit};
fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |key: &str| args.iter().position(|a| a == key).map(|i| args[i + 1].clone());
    let mut c = Command::new(std::env::var_os("REMUS_BASELINE_LLD").unwrap());
    c.args(["-flavor", "link", "/lib", "/machine:x64"]);
    let dll = value("-D").or_else(|| value("--dllname")).unwrap();
    let input = value("-d").or_else(|| value("--input-def")).unwrap();
    let output = value("-l").or_else(|| value("--output-lib")).unwrap();
    let def = format!("{output}.def");
    std::fs::write(&def, format!("LIBRARY {dll}\n{}", std::fs::read_to_string(input).unwrap())).unwrap();
    c.arg(format!("/def:{def}"));
    c.arg(format!("/out:{output}"));
    exit(c.status().unwrap().code().unwrap_or(1));
}
'@ | Set-Content -LiteralPath (Join-Path $tools 'dlltool.rs') -Encoding UTF8
    & rustc (Join-Path $tools 'dlltool.rs') -o (Join-Path $tools 'dlltool.exe') -C linker-flavor=ld.lld -C "linker=$lld"
    if ($LASTEXITCODE -ne 0) { throw 'Failed to build import-library helper' }
    $env:PATH = "$tools;$env:PATH"
    $env:RUSTFLAGS = $null
    $env:CARGO_ENCODED_RUSTFLAGS = "-C$([char]31)linker-flavor=ld.lld$([char]31)-C$([char]31)linker=$lld"
    @'
[package]
name = "video-workflow-runner"
version = "0.0.0"
edition = "2024"
[workspace]
[dependencies]
remus-io = {path = "../../crates/io"}
remus-operations = {path = "../../crates/operations"}
remus-topology = {path = "../../crates/topology"}
remus-math = {path = "../../crates/math"}
remus-check = {path = "../../crates/check"}
remus-blend = {path = "../../crates/blend"}
serde_json = "1"
log = "0.4"
[[test]]
name = "video_workflow_baseline"
path = "../../crates/io/tests/video_workflow_baseline.rs"
[profile.test]
opt-level = 1
debug = false
'@ | Set-Content -LiteralPath (Join-Path $runner 'Cargo.toml') -Encoding UTF8
    # Apply the real workspace lint policy to the test-only runner as well.
    $rootManifest = Get-Content -LiteralPath Cargo.toml -Raw
    $lintStart = $rootManifest.IndexOf('[workspace.lints.rust]')
    $lintEnd = $rootManifest.IndexOf('[profile.release]')
    if ($lintStart -lt 0 -or $lintEnd -le $lintStart) { throw 'Cannot locate workspace lint policy' }
    $rootManifest.Substring($lintStart, $lintEnd - $lintStart).Replace('[workspace.lints.', '[lints.') |
        Add-Content -LiteralPath (Join-Path $runner 'Cargo.toml') -Encoding UTF8
    Copy-Item -LiteralPath Cargo.lock -Destination (Join-Path $runner 'Cargo.lock') -Force
    $env:REMUS_VIDEO_FIXTURES = (Resolve-Path -LiteralPath $Fixtures).Path
    $env:REMUS_VIDEO_OUTPUT = if ($OutputDirectory) { [IO.Path]::GetFullPath($OutputDirectory) } else { Join-Path $repo 'target\video-workflow-baseline' }
    $env:REMUS_VIDEO_CASE = $Case
    $env:REMUS_VIDEO_MODE = $Mode
    New-Item -ItemType Directory -Force $env:REMUS_VIDEO_OUTPUT | Out-Null
    @{
        head = (& git rev-parse HEAD).Trim()
        origin_main = (& git rev-parse origin/main).Trim()
        rustc = (& rustc -vV) -join "`n"
        started_utc = [DateTime]::UtcNow.ToString('o')
        repository_lock_sha256 = (Get-FileHash -LiteralPath Cargo.lock -Algorithm SHA256).Hash
        fixtures = @('Gentle-Gecko.step', 'Gentle-Gecko-W-Fillet.step') | ForEach-Object {
            @{ name = $_; sha256 = (Get-FileHash -LiteralPath (Join-Path $Fixtures $_) -Algorithm SHA256).Hash }
        }
        profile = 'test, opt-level=1, debug=false; kernel workspace defaults'
        mode = $Mode
        case = $Case
    } | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $env:REMUS_VIDEO_OUTPUT 'run-manifest.json') -Encoding UTF8
    $arguments = @('--manifest-path', (Join-Path $runner 'Cargo.toml'), '--target-dir', (Join-Path $repo 'target'), '--test', 'video_workflow_baseline', '--offline')
    if ($Lint) { & cargo clippy @arguments -- -D warnings }
    elseif ($CompileOnly) { & cargo test @arguments --no-run }
    else { & cargo test @arguments -- --ignored --nocapture }
    if ($LASTEXITCODE -ne 0) { throw "Cargo failed with exit code $LASTEXITCODE" }
}
finally {
    foreach ($name in $names) { [Environment]::SetEnvironmentVariable($name, $saved[$name], 'Process') }
    Pop-Location
}
