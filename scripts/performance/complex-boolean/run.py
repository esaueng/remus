#!/usr/bin/env python3
"""Capture bounded native complex booleans; failed qualification rejects timings."""
import argparse
import datetime as dt
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import shutil
import statistics
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
MANIFEST = HERE / "workloads.json"
WORKER = ROOT / "crates/operations/examples/complex_boolean_baseline.rs"
spec = importlib.util.spec_from_file_location("maintained_baseline", HERE.parent / "run.py")
common = importlib.util.module_from_spec(spec)
spec.loader.exec_module(common)
STAGES = ("setup", "preflight", "kernel", "qualification", "teardown")
ROUTES = ("general_gfa_calls", "store_local_attempts", "store_local_successes", "local_cut_successes", "nary_calls")
WORK = ("pave_vertex_probes", "sd_poly_clips", "ve_sampled_probes", "ve_projection_evals",
        "ef_analytic_pair_scans", "face_split_probes", "store_faces_copied", "coplanar_section_probes",
        "ve_pairs_visited", "ee_pairs_visited", "ee_pairs_surviving", "ff_pairs_visited", "ff_pairs_surviving")
PHASES = {"gfa.store_import", "pave.vv", "pave.ve", "pave.ee", "pave.vf", "pave.ef", "pave.ff",
          "pave.coplanar", "pave.resolve", "gfa.build", "gfa.assemble", "gfa.store_export", "gfa.build_nway"}


def duration(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value) and value > 0)


def expected_volume(case):
    scenario, size = case["scenario"], case["size"]
    if scenario.startswith("slot_"):
        extent = max(size, 4) * 12 + 4
        plate = extent * extent * 10 - size * size * 40 * math.pi
        return {"slot_cut": plate - 120, "slot_fuse": plate + 24, "slot_intersect": 120.0}[scenario]
    if scenario == "fuse_sparse":
        return float(size)
    if scenario == "fuse_dense":
        return 1 + (size - 1) * (0.5 / size)
    if scenario == "curved_half_cylinder":
        return 20 * math.pi
    raise ValueError("unknown scenario")


def validate_records(text, case, samples, warmup, store_local, memos, phase_clock=False, counters=True):
    rows = [json.loads(line) for line in text.splitlines()]
    if len(rows) != samples + warmup:
        raise ValueError("missing or extra sample records")
    for index, row in enumerate(rows):
        expected = {"schema": "remus-complex-boolean-sample-v1", "scenario": case["scenario"],
                    "size": case["size"], "sample": index, "warmup": index < warmup,
                    "validation": "passed"}
        if any(row.get(key) != value for key, value in expected.items()):
            raise ValueError(f"sample {index}: identity or qualification mismatch")
        if any(type(row.get(key)) is not int for key in ("size", "sample")) or type(row.get("warmup")) is not bool:
            raise ValueError(f"sample {index}: malformed sample identity")
        settings = row.get("settings", {})
        if (settings.get("store_local_requested") is not store_local
                or settings.get("content_memos") is not memos
                or settings.get("perf_counters") is not counters
                or settings.get("phase_clock") is not phase_clock):
            raise ValueError(f"sample {index}: execution settings mismatch")
        routes, work = row.get("routes", {}), row.get("work", {})
        if counters:
            for observed, keys in [(routes, ROUTES), (work, WORK)]:
                if not isinstance(observed, dict) or set(observed) != set(keys) or any(type(observed[k]) is not int or observed[k] < 0 for k in keys):
                    raise ValueError(f"sample {index}: missing or malformed work/route counters")
        elif routes is not None or work is not None or row.get("phases") != [] or phase_clock:
            raise ValueError(f"sample {index}: instrumented fields in a counter-free capture")
        if counters and (routes["store_local_successes"] > routes["store_local_attempts"] or (not store_local and routes["store_local_attempts"])):
            raise ValueError(f"sample {index}: inconsistent store-local routes")
        if counters and any(work[f"{kind}_pairs_surviving"] > work[f"{kind}_pairs_visited"] for kind in ("ee", "ff")):
            raise ValueError(f"sample {index}: surviving pairs exceed visits")
        phases = row.get("phases")
        if not isinstance(phases, list) or len(phases) > 13:
            raise ValueError(f"sample {index}: missing or unbounded phase inventory")
        names = set()
        for phase in phases:
            if not isinstance(phase, dict):
                raise ValueError(f"sample {index}: malformed phase attribution")
            elapsed = phase.get("duration_ms")
            if (not isinstance(phase.get("name"), str) or phase["name"] not in PHASES or phase["name"] in names
                    or type(phase.get("calls")) is not int or phase["calls"] <= 0
                    or isinstance(elapsed, bool) or not isinstance(elapsed, (int, float))
                    or not math.isfinite(elapsed) or elapsed < 0 or (not phase_clock and elapsed != 0)):
                raise ValueError(f"sample {index}: invalid phase attribution")
            names.add(phase["name"])
        stages = row.get("stages_ns", {})
        if set(stages) != set(STAGES) or not all(duration(stages[key]) for key in STAGES):
            raise ValueError(f"sample {index}: invalid stage timings")
        if not duration(row.get("generation_ns")) or row["generation_ns"] != stages["setup"] + stages["kernel"]:
            raise ValueError(f"sample {index}: invalid generation total")
        metrics = row.get("metrics", {})
        volume, expected, bound = (metrics.get(k) for k in ("volume", "expected_volume", "volume_bound"))
        if (not all(duration(v) for v in [volume, expected, bound])
                or abs(volume - expected) > bound
                or metrics.get("input_operands_preserved") is not True
                or type(metrics.get("faces")) is not int or metrics["faces"] <= 0
                or type(metrics.get("material_probes")) is not int or metrics["material_probes"] < 2):
            raise ValueError(f"sample {index}: missing or failed independent metrics")
        reference = expected_volume(case)
        if not math.isclose(expected, reference, rel_tol=2e-15, abs_tol=1e-12) or not math.isclose(bound, 1e-8 * max(abs(reference), 1), rel_tol=2e-15):
            raise ValueError(f"sample {index}: altered closed-form oracle or bound")
        components = case["size"] if case["scenario"] == "fuse_sparse" else 1
        if type(metrics.get("edge_connected_components")) is not int or metrics["edge_connected_components"] != components:
            raise ValueError(f"sample {index}: incorrect result components")
    return rows


def summarize(rows):
    retained = [row for row in rows if not row["warmup"]]
    if not retained:
        raise ValueError("no retained samples")
    result = {"samples": len(retained), "unit": "ms", "tail_percentiles": None,
              "tail_note": "No tail-confidence claim; retain raw samples and per-process medians.", "stages": {}}
    for key in (*STAGES, "generation"):
        def value(row):
            return (row["generation_ns"] if key == "generation" else row["stages_ns"][key]) / 1e6
        values = [value(row) for row in retained]
        result["stages"][key] = {"min": min(values), "median": statistics.median(values), "max": max(values),
            "process_medians": [statistics.median(value(row) for row in retained if row["process"] == process)
                                for process in sorted({row["process"] for row in retained})]}
    return result


def run(args):
    manifest = json.loads(MANIFEST.read_text())
    if manifest["schema"] != "remus-complex-boolean-workloads-v1":
        raise ValueError("unknown manifest")
    available = {case["id"] for case in manifest["cases"]}
    if args.case and set(args.case) - available:
        raise ValueError("unknown case ID")
    cases = [case for case in manifest["cases"] if case["id"] in args.case] if args.case else [
        case for case in manifest["cases"] if case.get("measurement_eligible", True)]
    output = (args.output or ROOT / "target/performance/complex-boolean" / dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")).resolve()
    if output.is_relative_to(ROOT) and not output.is_relative_to(ROOT / "target"):
        raise ValueError("use target/performance or an output directory outside the checkout")
    output.mkdir(parents=True, exist_ok=False)
    info = {"schema": "remus-complex-boolean-run-v1", "status": "initializing"}
    common.write_json(output / "run.json", info)
    try:
        env = os.environ.copy()
        env.update(RAYON_NUM_THREADS=str(args.threads), CARGO_BUILD_JOBS=str(args.jobs), CARGO_TARGET_DIR=str(ROOT / "target"))
        counters = args.counters or args.phase_timings
        build_environment = common.build_environment(env)
        build_environment.update({key: env.get(key) for key in ("CARGO_INCREMENTAL", "RUST_MIN_STACK")})
        info = {"schema": "remus-complex-boolean-run-v1", "status": "running", "source": common.source_identity(),
                "manifest": manifest, "manifest_sha256": common.digest(MANIFEST), "selected_cases": [c["id"] for c in cases],
                "known_unqualified_cases": [case for case in manifest["cases"] if not case.get("measurement_eligible", True)],
                "settings": {"store_local_requested": args.store_local == "on", "content_memos": args.memos == "on", "phase_clock": args.phase_timings, "perf_counters": counters},
                "memo_policy": "pcurve-and-wire-area-only; other caches keep native defaults",
                "timing_status": "instrumented-phase-clock" if args.phase_timings else "instrumented-clock-off" if counters else "counter-free",
                "processes": args.processes, "samples_per_process": args.samples, "warmup": args.warmup,
                "threads": args.threads, "platform": platform.platform(), "cpu": common.cpu_model(),
                "affinity": sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None,
                "capture_cpu": args.capture_cpu, "environment": build_environment,
                "rustc": common.command(["rustc", "-Vv"]), "cargo": common.command(["cargo", "-V"]),
                "counter_source_sha256": {name: common.digest(ROOT / name) for name in ["crates/algo/src/perf.rs", "crates/operations/src/performance.rs"]},
                "harness_sha256": {str(path.relative_to(ROOT)): common.digest(path) for path in [Path(__file__), MANIFEST, WORKER, HERE.parent / "run.py"]}}
        common.write_json(output / "run.json", info)
        binary = args.binary
        if binary is None:
            build = ["cargo", "build", "--locked", "--profile", "profiling", "-p", "remus-operations",
                     "--example", "complex_boolean_baseline", "--message-format=json"]
            if counters:
                build += ["--features", "perf-counters"]
            if args.offline:
                build.append("--offline")
            info["build_command"] = build
            with (output / "build.jsonl").open("w") as stdout, (output / "build.stderr.txt").open("w") as stderr:
                subprocess.run(build, cwd=ROOT, env=env, stdout=stdout, stderr=stderr, check=True)
            for line in (output / "build.jsonl").read_text().splitlines():
                row = json.loads(line)
                if row.get("reason") == "compiler-artifact" and row["target"]["name"] == "complex_boolean_baseline" and row.get("executable"):
                    binary = Path(row["executable"])
                    info["native_profile"] = row["profile"]
                    info["native_features"] = row["features"]
                    info["cargo_artifact_receipt"] = row
            if binary is None:
                raise ValueError("Cargo did not report an executable")
        binary = binary.resolve(strict=True)
        binary_hash = common.digest(binary)
        frozen = output / ("native-worker" + binary.suffix)
        shutil.copyfile(binary, frozen)
        frozen.chmod(0o700)
        if common.digest(frozen) != binary_hash or common.digest(binary) != binary_hash:
            raise ValueError("executable changed while freezing capture artifact")
        binary = frozen
        info["native_binary_sha256"] = binary_hash
        info["native_binary_capture"] = frozen.name
        info["binary_provenance"] = "Built from captured source" if args.binary is None else "Externally supplied executable; source equivalence is not established"
        common.write_json(output / "run.json", info)
        summaries = []
        with (output / "samples.jsonl").open("w") as raw:
            for case in cases:
                records = []
                for process in range(args.processes):
                    worker = [str(binary), case["scenario"], str(case["size"]), str(args.samples), str(args.warmup), args.store_local, args.memos, "on" if args.phase_timings else "off"]
                    if args.capture_cpu is not None:
                        worker = ["taskset", "-c", str(args.capture_cpu), *worker]
                    text = common.capture_worker(worker, output / f"{case['id']}.{process}", env, args.timeout)
                    rows = validate_records(text, case, args.samples, args.warmup, args.store_local == "on", args.memos == "on", args.phase_timings, counters)
                    for row in rows:
                        row.update(case_id=case["id"], process=process)
                        raw.write(json.dumps(row, allow_nan=False) + "\n")
                    raw.flush()
                    records.extend(rows)
                summaries.append({"case_id": case["id"], **summarize(records)})
                print(f"{case['id']}: kernel median {summaries[-1]['stages']['kernel']['median']:.3f} ms; qualified", flush=True)
        if common.source_identity() != info["source"]:
            raise ValueError("source changed during run; samples rejected")
        if common.digest(binary) != info["native_binary_sha256"]:
            raise ValueError("executable changed during run; samples rejected")
        common.write_json(output / "summary.json", {"schema": "remus-complex-boolean-summary-v1", "timing_status": info["timing_status"], "cases": summaries})
        info["status"] = "passed"
    except Exception as error:
        info.update(status="failed", error=str(error))
        raise
    finally:
        info["finished_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
        common.write_json(output / "run.json", info)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--case", action="append", help="repeat a workload ID; defaults to all")
    parser.add_argument("--binary", type=Path, help="reuse an executable; its source provenance remains external")
    parser.add_argument("--capture-cpu", type=int, help="pin worker processes only; build retains caller affinity")
    parser.add_argument("--store-local", choices=["off", "on"], default="off")
    parser.add_argument("--memos", choices=["off", "on"], default="on")
    parser.add_argument("--counters", action="store_true", help="opt into instrumented work/route counters; default build is counter-free")
    parser.add_argument("--phase-timings", action="store_true", help="diagnostic phase clocks; keep separate from clock-disabled captures")
    for flag, default in [("processes", 3), ("samples", 5), ("warmup", 1), ("threads", 1), ("jobs", 2), ("timeout", 300)]:
        parser.add_argument(f"--{flag}", type=common.positive, default=default)
    parser.add_argument("--offline", action="store_true")
    args = parser.parse_args()
    if args.warmup > 100 or (args.capture_cpu is not None and args.capture_cpu < 0):
        parser.error("warmup must be <=100; capture CPU must be nonnegative")
    try:
        run(args)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"FAILED: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
