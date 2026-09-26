#!/usr/bin/env python3
"""Repeatable sketch solve/drag performance baseline (PERF-S01).

Native solve/drag timings come from current source via the profiling-release
`sketch_baseline` example; WASM timings come from the already-committed
package via `wasm.cjs` and are never compared as speedups. Failures never
become timings: any identity, dimension, outcome, or validation mismatch
rejects the run. No wall-time thresholds are asserted anywhere.
"""
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

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
MANIFEST = HERE / "workloads.json"
WASM_WORKER = HERE / "wasm.cjs"
NATIVE_EXAMPLE = ROOT / "crates/wasm/examples/sketch_baseline.rs"

# Representative packaged-WASM coverage (fresh runs, separate provenance).
WASM_COVERAGE = [
    ("independent_solved", 100),
    ("coupled_chain", 100),
    ("drag", 100),
]
MODES = ["solve", "detailed"]

# Bounded smoke: smallest cold solves plus the 100-param outcome/drag pins,
# including the packaged-WASM representative subset.
SMOKE_CASES = [
    "independent_under_10",
    "independent_solved_10",
    "coupled_chain_10",
    "independent_under_100",
    "independent_solved_100",
    "coupled_chain_100",
    "redundant_100",
    "inconsistent_100",
    "drag_100",
]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def command(args, env=None):
    return subprocess.check_output(args, cwd=ROOT, env=env, text=True).strip()


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


def validate_records(text, case, mode, samples, warmup):
    rows = [json.loads(line) for line in text.splitlines()]
    if len(rows) != samples + warmup:
        raise ValueError(f"{case['id']}/{mode}: missing or extra sample records")
    for i, row in enumerate(rows):
        if row.get("schema") != "remus-sketch-perf-sample-v1":
            raise ValueError(f"{case['id']}/{mode} sample {i}: bad schema")
        for key, want in (
            ("workload", case["workload"]),
            ("size", case["size"]),
            ("mode", mode),
            ("sample", i),
            ("warmup", i < warmup),
            ("validation", "passed"),
        ):
            if row.get(key) != want:
                raise ValueError(
                    f"{case['id']}/{mode} sample {i}: {key} {row.get(key)!r} != {want!r}"
                )
        if row.get("num_params") != case["num_params"]:
            raise ValueError(f"{case['id']}/{mode} sample {i}: num_params mismatch")
        if row.get("num_equations") != case["num_equations"]:
            raise ValueError(f"{case['id']}/{mode} sample {i}: num_equations mismatch")
        expected = case["expected"]
        if row.get("classification") != expected:
            raise ValueError(
                f"{case['id']}/{mode} sample {i}: classification "
                f"{row.get('classification')!r} != {expected!r}"
            )
        if expected == "resource_refused":
            if row.get("resource") != "refused" or row.get("operation_ns") is not None:
                raise ValueError(f"{case['id']}/{mode} sample {i}: refusal shape mismatch")
        else:
            if row.get("resource") != "solved":
                raise ValueError(f"{case['id']}/{mode} sample {i}: resource mismatch")
            ns = row.get("operation_ns")
            if isinstance(ns, bool) or not isinstance(ns, (int, float)) or not math.isfinite(ns) or ns <= 0:
                raise ValueError(f"{case['id']}/{mode} sample {i}: invalid duration")
        if not isinstance(row.get("metrics"), dict) or not row["metrics"]:
            raise ValueError(f"{case['id']}/{mode} sample {i}: missing metrics")
    return rows


def summarize(rows):
    timed = [r for r in rows if not r["warmup"] and r.get("operation_ns") is not None]
    refused = [r for r in rows if not r["warmup"] and r.get("operation_ns") is None]
    if timed:
        values = [r["operation_ns"] / 1e6 for r in timed]
        processes = sorted({r["process"] for r in timed})
        return {
            "samples": len(values),
            "refused": len(refused),
            "unit": "ms per named operation",
            "min": min(values),
            "median": statistics.median(values),
            "max": max(values),
            "process_medians": [
                statistics.median(r["operation_ns"] / 1e6 for r in timed if r["process"] == p)
                for p in processes
            ],
            "tail_percentiles": None,
            "tail_note": "No p95/p99 estimate: this baseline does not qualify tail sample independence or sample count.",
        }
    return {
        "samples": 0,
        "refused": len(refused),
        "unit": "ms per named operation",
        "min": None,
        "median": None,
        "max": None,
        "process_medians": [],
        "tail_percentiles": None,
        "tail_note": "All retained samples refused the upfront resource budget; no timing exists for this row.",
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
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")


def positive(value):
    number = int(value)
    if not 1 <= number <= 1000:
        raise argparse.ArgumentTypeError("expected 1..1000")
    return number


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def run(args):
    manifest = json.loads(MANIFEST.read_text())
    if manifest["schema"] != "remus-sketch-perf-workloads-v1":
        raise ValueError("unknown sketch workload manifest")
    cases = manifest["cases"]
    if args.smoke:
        cases = [c for c in cases if c["id"] in SMOKE_CASES]
        if not cases:
            raise ValueError("smoke filter matched no cases")
    if args.case:
        wanted = set(args.case)
        cases = [c for c in cases if c["id"] in wanted]
        if not cases:
            raise ValueError("no matching cases")
    modes = [args.mode] if args.mode else MODES
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = (args.output or ROOT / "target" / "performance-sketch" / stamp).resolve()
    if output.is_relative_to(ROOT) and not output.is_relative_to(ROOT / "target"):
        raise ValueError("use target/performance-sketch or an output directory outside the checkout")
    output.mkdir(parents=True, exist_ok=False)
    print(f"Results: {output}", flush=True)
    env = os.environ.copy()
    info = {
        "schema": "remus-sketch-perf-run-v1",
        "started_utc": stamp,
        "status": "running",
        "source": source_identity(),
        "manifest_sha256": digest(MANIFEST),
        "manifest": manifest,
        "selected_cases": [c["id"] for c in cases],
        "modes": modes,
        "smoke": bool(args.smoke),
        "processes": args.processes,
        "samples_per_process": args.samples,
        "warmup_per_process": args.warmup,
        "threads": 1,
        "platform": platform.platform(),
        "machine": platform.machine(),
        "cpu": command(["sh", "-c", "grep -m1 'model name' /proc/cpuinfo | cut -d: -f2 || uname -p"]),
        "logical_cpus": os.cpu_count(),
        "python": sys.version,
        "rustc": command(["rustc", "-Vv"]),
        "cargo": command(["cargo", "-V"]),
        "harness_sha256": {
            os.path.relpath(p, ROOT): digest(p)
            for p in [Path(__file__), MANIFEST, WASM_WORKER, NATIVE_EXAMPLE]
        },
    }
    write_json(output / "run.json", info)
    try:
        build = [
            "cargo", "build", "--locked", "--profile", "profiling",
            "-p", "remus-wasm", "--example", "sketch_baseline",
            "--message-format=json",
        ]
        if args.offline:
            build.append("--offline")
        info["build_command"] = build
        write_json(output / "run.json", info)
        binary = None
        with (output / "build.jsonl").open("w") as stdout, (output / "build.stderr.txt").open("w") as stderr:
            subprocess.run(build, cwd=ROOT, env=env, stdout=stdout, stderr=stderr, check=True)
        for line in (output / "build.jsonl").read_text().splitlines():
            row = json.loads(line)
            if (
                row.get("reason") == "compiler-artifact"
                and row["target"]["name"] == "sketch_baseline"
                and row.get("executable")
            ):
                binary = Path(row["executable"])
                info["native_profile"] = row["profile"]
        if not binary:
            raise ValueError("Cargo did not report the sketch_baseline executable")
        info["native_binary_sha256"] = digest(binary)
        pkg = ROOT / "crates/wasm/pkg"
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
        with (output / "samples.jsonl").open("w") as all_samples, (output / "wasm_samples.jsonl").open("w") as wasm_samples:
            for case in cases:
                for mode in modes:
                    records = []
                    for process in range(args.processes):
                        worker = [str(binary), case["workload"], str(case["size"]), mode, str(args.samples), str(args.warmup)]
                        stem = output / f"native.{case['id']}.{mode}.{process}"
                        text = capture_worker(worker, stem, env, args.timeout)
                        rows = validate_records(text, case, mode, args.samples, args.warmup)
                        for row in rows:
                            row.update(case_id=case["id"], runtime="native", process=process)
                            all_samples.write(json.dumps(row, allow_nan=False) + "\n")
                        all_samples.flush()
                        records.extend(rows)
                    summary = {"case_id": case["id"], "mode": mode, "runtime": "native", **summarize(records)}
                    summaries.append(summary)
                    med = summary["median"]
                    med_s = f"{med:.3f} ms" if med is not None else "refused (no timing)"
                    print(f"native {case['id']}/{mode}: median {med_s}; correctness passed", flush=True)
            # Fresh packaged-WASM runs for the representative subset only.
            wasm_cases = [c for c in cases if (c["workload"], c["size"]) in [tuple(x) for x in WASM_COVERAGE]]
            for case in wasm_cases:
                for mode in modes:
                    records = []
                    for process in range(args.wasm_processes):
                        worker = ["node", str(WASM_WORKER), str(pkg), case["workload"], str(case["size"]), mode, str(args.samples), str(args.warmup)]
                        stem = output / f"wasm.{case['id']}.{mode}.{process}"
                        text = capture_worker(worker, stem, env, args.timeout)
                        rows = validate_records(text, case, mode, args.samples, args.warmup)
                        for row in rows:
                            row.update(case_id=case["id"], runtime="wasm", process=process)
                            wasm_samples.write(json.dumps(row, allow_nan=False) + "\n")
                        wasm_samples.flush()
                        records.extend(rows)
                    summary = {"case_id": case["id"], "mode": mode, "runtime": "wasm", **summarize(records)}
                    summaries.append(summary)
                    med = summary["median"]
                    med_s = f"{med:.3f} ms" if med is not None else "refused (no timing)"
                    print(f"wasm {case['id']}/{mode}: median {med_s}; correctness passed", flush=True)
        if source_identity() != info["source"]:
            raise ValueError("source changed during the run; baseline rejected")
        info["status"] = "passed"
        write_json(output / "summary.json", {"schema": "remus-sketch-perf-summary-v1", "cases": summaries})
        lines = ["# Sketch performance baseline", "", "All selected correctness checks passed. Timings are diagnostic, not release gates.", ""]
        lines.append("| Runtime | Case | Mode | Samples | Refused | Median ms | Min ms | Max ms |")
        lines.append("| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |")
        for s in summaries:
            med = f"{s['median']:.4f}" if s["median"] is not None else "refused"
            mn = f"{s['min']:.4f}" if s["min"] is not None else "—"
            mx = f"{s['max']:.4f}" if s["max"] is not None else "—"
            lines.append(f"| {s['runtime']} | {s['case_id']} | {s['mode']} | {s['samples']} | {s['refused']} | {med} | {mn} | {mx} |")
        (output / "summary.md").write_text("\n".join(lines) + "\n")
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
    parser.add_argument("--case", action="append", help="repeat to select case IDs; default: all (or smoke set)")
    parser.add_argument("--mode", choices=MODES, help="run a single mode; default: solve and detailed")
    parser.add_argument("--smoke", action="store_true", help="bounded smoke: small cases only, 1 process x 1 sample")
    parser.add_argument("--processes", type=positive, default=3)
    parser.add_argument("--samples", type=positive, default=5)
    parser.add_argument("--warmup", type=int, default=1, choices=list(range(0, 101)))
    parser.add_argument("--wasm-processes", type=positive, default=1)
    parser.add_argument("--timeout", type=positive, default=300, help="seconds per worker process")
    parser.add_argument("--offline", action="store_true", help="require cached Cargo dependencies")
    args = parser.parse_args()
    if args.smoke:
        args.processes = 1
        args.samples = 1
        args.wasm_processes = 1
        if args.warmup is None:
            args.warmup = 0
    try:
        run(args)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"FAILED: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
