#!/usr/bin/env python3
"""Run the versioned baseline in isolated worker processes; failures never become timings."""
import argparse
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = Path(__file__).with_name("workloads.json")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def command(args, env=None):
    return subprocess.check_output(args, cwd=ROOT, env=env, text=True).strip()


def cpu_model():
    cpuinfo = Path("/proc/cpuinfo")
    if cpuinfo.exists():
        for line in cpuinfo.read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    return platform.processor()


def build_environment(env):
    keys = {"RUSTFLAGS", "CARGO_ENCODED_RUSTFLAGS", "CARGO_BUILD_TARGET", "CARGO_TARGET_DIR",
            "CARGO_BUILD_JOBS", "RAYON_NUM_THREADS", "RUSTUP_TOOLCHAIN", "RUSTC", "RUSTC_WRAPPER",
            "RUSTC_WORKSPACE_WRAPPER", "CC", "CXX", "CFLAGS", "CXXFLAGS", "LDFLAGS"}
    keys.update(k for k in env if k.startswith("CARGO_PROFILE_") or
                (k.startswith("CARGO_TARGET_") and k.endswith(("_RUSTFLAGS", "_LINKER"))))
    return {k: env.get(k) for k in sorted(keys)}


def source_identity():
    paths = subprocess.check_output(
        ["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard"], cwd=ROOT
    ).decode().split("\0")
    tree = hashlib.sha256()
    for name in sorted(set(filter(None, paths))):
        path = ROOT / name
        tree.update(name.encode() + b"\0")
        if path.is_symlink():
            tree.update(b"symlink:" + os.readlink(path).encode())
        elif path.is_file():
            tree.update(bytes.fromhex(digest(path)))
        else:
            tree.update(b"missing")
    return {
        "head": command(["git", "rev-parse", "HEAD"]),
        "status": command(["git", "status", "--porcelain=v1"]),
        "working_tree_sha256": tree.hexdigest(),
    }


def validate_records(text, case, samples, warmup):
    rows = [json.loads(line) for line in text.splitlines()]
    if len(rows) != samples + warmup:
        raise ValueError("missing or extra sample records")
    for i, row in enumerate(rows):
        expected = {"schema": "remus-performance-sample-v1", "scenario": case["scenario"],
                    "size": case["size"], "sample": i, "warmup": i < warmup,
                    "validation": "passed"}
        if any(row.get(k) != v for k, v in expected.items()):
            raise ValueError(f"sample {i}: identity, warmup or validation mismatch")
        ns = row.get("operation_ns")
        if isinstance(ns, bool) or not isinstance(ns, (int, float)) or not math.isfinite(ns) or ns <= 0:
            raise ValueError(f"sample {i}: invalid duration")
        if not isinstance(row.get("metrics"), dict) or not row["metrics"]:
            raise ValueError(f"sample {i}: missing correctness metrics")
    return rows


def summarize(rows):
    retained = [r for r in rows if not r["warmup"]]
    values = [r["operation_ns"] / 1e6 for r in retained]
    if not values:
        raise ValueError("no retained samples")
    processes = sorted({r["process"] for r in retained})
    return {
        "samples": len(values), "unit": "ms per named operation", "min": min(values),
        "median": statistics.median(values), "max": max(values),
        "process_medians": [statistics.median(r["operation_ns"] / 1e6 for r in retained
                             if r["process"] == process) for process in processes],
        "tail_percentiles": None,
        "tail_note": "No p95/p99 estimate: this baseline does not qualify tail sample independence or sample count.",
    }


def capture_worker(worker, stem, env, timeout):
    stdout_path = Path(str(stem) + ".stdout.jsonl")
    stderr_path = Path(str(stem) + ".stderr.txt")
    metadata_path = Path(str(stem) + ".process.json")
    started = time.monotonic()
    metadata = {"command": worker, "status": "failed", "exit_code": None}
    try:
        with stdout_path.open("w") as stdout, stderr_path.open("w") as stderr:
            result = subprocess.run(worker, cwd=ROOT, env=env, stdout=stdout, stderr=stderr, timeout=timeout)
        metadata["exit_code"] = result.returncode
        if result.returncode != 0:
            raise ValueError(f"{stem.name} failed with exit {result.returncode}; see {stderr_path.name}")
        metadata["status"] = "exited_successfully"
        return stdout_path.read_text()
    except (OSError, subprocess.SubprocessError) as error:
        metadata["error"] = str(error)
        raise
    finally:
        metadata["wall_seconds"] = time.monotonic() - started
        write_json(metadata_path, metadata)


def positive(value):
    number = int(value)
    if not 1 <= number <= 1000:
        raise argparse.ArgumentTypeError("expected 1..1000")
    return number


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def run(args):
    manifest = json.loads(MANIFEST.read_text())
    if manifest["schema"] != "remus-performance-workloads-v1":
        raise ValueError("unknown workload manifest")
    cases = manifest["cases"]
    selected = [c for c in cases if not args.family or c["family"] in args.family]
    if not selected:
        raise ValueError("no matching workload family")
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = (args.output or ROOT / "target" / "performance" / stamp).resolve()
    if output.is_relative_to(ROOT) and not output.is_relative_to(ROOT / "target"):
        raise ValueError("use target/performance or an output directory outside the checkout")
    output.mkdir(parents=True, exist_ok=False)
    print(f"Results: {output}", flush=True)
    env = os.environ.copy()
    env["RAYON_NUM_THREADS"] = str(args.threads)
    env["CARGO_BUILD_JOBS"] = str(args.jobs)
    info = {"schema": "remus-performance-run-v1", "started_utc": stamp, "status": "running",
            "source": source_identity(), "manifest_sha256": digest(MANIFEST),
            "manifest": manifest, "selected_cases": [c["id"] for c in selected],
            "processes": args.processes, "samples_per_process": args.samples, "warmup_per_process": args.warmup,
            "threads": args.threads, "platform": platform.platform(), "machine": platform.machine(),
            "cpu": cpu_model(), "logical_cpus": os.cpu_count(),
            "python": sys.version, "rustc": command(["rustc", "-Vv"]),
            "cargo": command(["cargo", "-V"]), "environment": build_environment(env),
            "harness_sha256": {os.path.relpath(p, ROOT): digest(p) for p in
                [Path(__file__), MANIFEST, Path(__file__).with_name("wasm.cjs"), ROOT / "crates/wasm/examples/performance_baseline.rs"]}}
    write_json(output / "run.json", info)
    try:
        for fixture in manifest["fixtures"]:
            if digest(ROOT / fixture["path"]) != fixture["sha256"]:
                raise ValueError(f"fixture hash mismatch: {fixture['id']}")
        native = any(c["scenario"].startswith("native_") for c in selected)
        wasm = any(c["scenario"].startswith("wasm_") for c in selected)
        binary = None
        if native:
            build = ["cargo", "build", "--locked", "--profile", "profiling", "--no-default-features", "--features", "io",
                     "-p", "remus-wasm", "--example", "performance_baseline", "--message-format=json"]
            if args.offline:
                build.append("--offline")
            info["build_command"] = build
            write_json(output / "run.json", info)
            with (output / "build.jsonl").open("w") as stdout, (output / "build.stderr.txt").open("w") as stderr:
                subprocess.run(build, cwd=ROOT, env=env, stdout=stdout, stderr=stderr, check=True)
            for line in (output / "build.jsonl").read_text().splitlines():
                row = json.loads(line)
                if row.get("reason") == "compiler-artifact" and row["target"]["name"] == "performance_baseline" and row.get("executable"):
                    binary = Path(row["executable"])
                    info["native_profile"] = row["profile"]
                    info["native_features"] = row["features"]
            if not binary:
                raise ValueError("Cargo did not report the benchmark executable")
            info["native_binary_sha256"] = digest(binary)
        pkg = ROOT / "crates/wasm/pkg"
        if wasm:
            info["node"] = command(["node", "--version"])
            info["wasm_package"] = {
                "version": json.loads((pkg / "package.json").read_text())["version"],
                "declared_source": json.loads((pkg / "package.json").read_text()).get("remusSourceCommit"),
                "files_sha256": {p.name: digest(p) for p in sorted(pkg.iterdir()) if p.is_file()},
                "last_package_commit": command(["git", "log", "-1", "--format=%H%n%B", "--", "crates/wasm/pkg"]),
                "source_equivalence": "Not established. Committed WASM may predate native source; do not infer cross-runtime speedup.",
            }
        write_json(output / "run.json", info)
        summaries = []
        with (output / "samples.jsonl").open("w") as all_samples:
            for case in selected:
                records = []
                for process in range(args.processes):
                    worker = ([str(binary)] if case["scenario"].startswith("native_") else
                              ["node", str(Path(__file__).with_name("wasm.cjs")), str(pkg)])
                    worker += [case["scenario"], str(case["size"]), str(args.samples), str(args.warmup)]
                    stem = output / f"{case['id']}.{process}"
                    text = capture_worker(worker, stem, env, args.timeout)
                    rows = validate_records(text, case, args.samples, args.warmup)
                    for row in rows:
                        row.update(case_id=case["id"], process=process)
                        all_samples.write(json.dumps(row, allow_nan=False) + "\n")
                    all_samples.flush()
                    records.extend(rows)
                summaries.append({"case_id": case["id"], **summarize(records)})
                print(f"{case['id']}: median {summaries[-1]['median']:.3f} ms; correctness passed", flush=True)
        if source_identity() != info["source"]:
            raise ValueError("source changed during the run; baseline rejected")
        info["status"] = "passed"
        write_json(output / "summary.json", {"schema": "remus-performance-summary-v1", "cases": summaries})
        (output / "summary.md").write_text(
            "# Performance baseline\n\nAll selected correctness checks passed. Timings are diagnostic, not release gates.\n\n"
            "| Case | Samples | Median ms | Min ms | Max ms |\n| --- | ---: | ---: | ---: | ---: |\n" +
            "".join(f"| {s['case_id']} | {s['samples']} | {s['median']:.4f} | {s['min']:.4f} | {s['max']:.4f} |\n" for s in summaries))
    except Exception as error:
        info["status"] = "failed"
        info["error"] = str(error)
        raise
    finally:
        info["finished_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
        write_json(output / "run.json", info)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="new results directory; never overwrite a baseline")
    parser.add_argument("--family", action="append", choices=["nurbs", "transform", "chain"], help="repeat to select families; default: all")
    parser.add_argument("--processes", type=positive, default=3)
    parser.add_argument("--samples", type=positive, default=5)
    parser.add_argument("--warmup", type=positive, default=1)
    parser.add_argument("--threads", type=positive, default=1)
    parser.add_argument("--jobs", type=positive, default=2)
    parser.add_argument("--timeout", type=positive, default=300, help="seconds per worker process")
    parser.add_argument("--offline", action="store_true", help="require cached Cargo dependencies")
    args = parser.parse_args()
    if args.warmup > 100:
        parser.error("warmup must be at most 100")
    try:
        run(args)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"FAILED: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
