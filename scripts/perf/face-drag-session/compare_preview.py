#!/usr/bin/env python3
"""Compare freshly built revisions through maintained preview and edit drivers."""

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
import tempfile


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def package_hashes(root):
    return {str(p.relative_to(root)): sha256(p)
            for package in [root / "crates/wasm/pkg", root / "crates/wasm-io/pkg"]
            for p in sorted(package.rglob("*")) if p.is_file()}


def native_binary(root):
    binaries = [p for p in (root / "target/profiling/deps").glob("nurbs_properties-*")
                if p.is_file() and os.access(p, os.X_OK)]
    if len(binaries) != 1:
        raise ValueError("Expected exactly one compiled native benchmark")
    return binaries[0]


def executable_text_sha256(binary):
    # Ignore source-location/debug metadata when checking whether executable
    # instructions changed. Keep the original input binary untouched.
    with tempfile.TemporaryDirectory(prefix="preview-native-") as temporary:
        section = Path(temporary) / "text"
        subprocess.run(["objcopy", "--dump-section", f".text={section}",
                        str(binary), str(Path(temporary) / "copy")],
                       check=True, capture_output=True)
        return sha256(section)


def validate_capture(run, ticks, warmup, kernel_sha, io_sha, harness_sha):
    meta, records, summary = run["metadata"], run["records"], run["summary"]
    if (meta["timingStatus"] != "measured" or meta["timingsExcluded"]
            or meta["kernel"]["wasmSha256"] != kernel_sha
            or meta["io"]["wasmSha256"] != io_sha
            or meta["harnessSha256"] != harness_sha):
        raise ValueError("Capture timing policy or artifact identity differs")
    if len(records) != ticks or summary["refused"] != 0:
        raise ValueError("Capture is incomplete or contains refused edits")
    for index, row in enumerate(records):
        quality = row["quality"]
        if (row["tick"] != index or not row["accepted"]
                or not quality["watertight"] or quality["boundaryEdges"] != 0
                or quality["nonManifoldEdges"] != 0 or quality["faces"] != 160
                or not quality["capAtRequestedOffset"]):
            raise ValueError("Preview geometry gate failed")
        for value in row["stagesMs"].values():
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError("Nonfinite or negative preview duration")
    if (summary["warmedSamples"] != ticks - warmup
            or summary["warmed"]["tick"]["n"] != ticks - warmup):
        raise ValueError("Retained sample count differs")
    work = validate_work_contract(run)
    return {"samples": summary["warmedSamples"], "stagesMs": summary["warmed"],
            "coldMs": summary["cold"]["stagesMs"],
            "memoryBefore": summary["memoryBefore"], "memoryAfter": summary["memoryLast"],
            "workContract": work}


def validate_work_contract(run):
    """Rescore raw slot snapshots; a claimed passing summary is not proof."""
    def slots(snapshot):
        try:
            values = tuple(snapshot[key] for key in ["allocatedSlots", "retiredSlots"])
        except (KeyError, TypeError) as error:
            raise ValueError("Missing topology-slot snapshot") from error
        if any(type(value) is not int or value < 0 or value > 2**53 - 1 for value in values):
            raise ValueError("Invalid topology-slot snapshot")
        if values[1] > values[0]:
            raise ValueError("Retired topology exceeds allocated slots")
        return values

    source = run["metadata"].get("sourceTopologySlots")
    previous = slots(run["summary"]["memoryBefore"])
    if type(source) is not int or not 0 < source <= previous[0] or len(run["records"]) < 2:
        raise ValueError("Invalid source topology inventory or missing recurring work")
    initial = previous[0]
    maximum = 0
    for index, row in enumerate(run["records"]):
        stages = row.get("memoryStages", {})
        prior = slots(stages.get("before"))
        if prior != previous:
            raise ValueError("Untimed diagnostics changed topology slots")
        for stage in ["restore", "move", "validate", "mesh", "transfer"]:
            current = slots(stages.get(stage))
            allocated, retired = (current[i] - prior[i] for i in range(2))
            budget = source + (initial if index == 0 else 0) if stage == "move" else 0
            if not 0 <= allocated <= budget or not 0 <= retired <= (source if stage == "restore" else 0):
                raise ValueError(f"Preview topology work budget exceeded at tick {index}, {stage}")
            if index > 0 and stage == "move":
                maximum = max(maximum, allocated)
            prior = current
        previous = slots(row.get("memoryAfter"))
        if previous != prior:
            raise ValueError("Untimed quality checks changed topology slots")
        if previous[0] - previous[1] > 2 * initial + source:
            raise ValueError("Live topology exceeds the checkpoint, its copy and one edited body")
    if slots(run["summary"]["memoryLast"]) != previous:
        raise ValueError("Final topology inventory differs from raw records")
    return {"sourceSlots": source, "firstMoveSlotBudget": initial + source,
            "recurringMoveSlotBudget": source, "maximumRecurringSlots": maximum,
            "liveSlotBudget": 2 * initial + source,
            "readStageSlotBudget": 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-root", type=Path, required=True)
    parser.add_argument("--candidate-root", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--cpu", default="auto")
    args = parser.parse_args()
    roots = {"baseline": args.baseline_root.resolve(), "candidate": args.candidate_root.resolve()}
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    affinity = sorted(os.sched_getaffinity(0))
    cpu = affinity[0] if args.cpu == "auto" else int(args.cpu)
    if cpu not in affinity:
        raise ValueError("Requested CPU is outside the process affinity")
    env = {**os.environ, "RAYON_NUM_THREADS": "1"}
    # Balance each variant's outer/inner positions across two fresh-process
    # blocks. Retain the original ABBA block and add its inverse BAAB.
    order = ["baseline", "candidate", "candidate", "baseline",
             "candidate", "baseline", "baseline", "candidate"]
    driver = roots["candidate"] / "scripts/perf/face-drag-session/drag_session_bench.cjs"
    fixture = roots["candidate"] / "crates/io/tests/data/shapr3d_hammer_holder.step"
    native_drivers = {v: sha256(root / "crates/io/benches/nurbs_properties.rs") for v, root in roots.items()}
    if len(set(native_drivers.values())) != 1:
        raise ValueError("Native benchmark drivers differ")
    harness_sha = sha256(driver)
    hashes = {variant: package_hashes(root) for variant, root in roots.items()}
    metadata = {"status": "running", "startedUtc": dt.datetime.now(dt.timezone.utc).isoformat(),
                "cpu": cpu, "platform": platform.platform(), "logicalCpus": os.cpu_count(),
                "node": subprocess.check_output(["node", "--version"], text=True).strip(),
                "rustc": subprocess.check_output(["rustc", "-vV"], cwd=roots["candidate"], text=True).strip(),
                "nativeDriverSha256": native_drivers["candidate"],
                "harnessSha256": harness_sha, "fixtureSha256": sha256(fixture),
                "source": {v: subprocess.check_output(["git", "-C", str(r), "rev-parse", "HEAD"],
                                                     text=True).strip() for v, r in roots.items()},
                "packages": hashes, "processOrder": order, "cases": [], "native": [],
                "limits": "Descriptive single-host process observations; no browser presentation or tail-confidence claim."}

    def save():
        (out / "comparison.json").write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n")

    def execute(command, label, extra_env=None):
        with (out / f"{label}.stdout").open("w") as stdout, (out / f"{label}.stderr").open("w") as stderr:
            subprocess.run(command, env=extra_env or env, stdout=stdout, stderr=stderr,
                           check=True, timeout=900)

    save()
    try:
        native_binaries = {v: native_binary(r) for v, r in roots.items()}
        native_hashes = {v: sha256(p) for v, p in native_binaries.items()}
        native_text = {v: executable_text_sha256(p) for v, p in native_binaries.items()}
        metadata.update(nativeBinarySha256=native_hashes,
                        nativeExecutableTextSha256=native_text,
                        nativeExecutableTextIdentical=len(set(native_text.values())) == 1)
        save()
        kernels = {v: sha256(r / "crates/wasm/pkg/remus_wasm_bg.wasm") for v, r in roots.items()}
        translators = {v: sha256(r / "crates/wasm-io/pkg/remus_wasm_io_bg.wasm") for v, r in roots.items()}
        if len(set(kernels.values())) != 2:
            raise ValueError("Comparison requires two different kernel binaries")
        for pattern, background, ticks in [("unique", 0, 60), ("repeated", 0, 60), ("unique", 200, 40)]:
            case = {"pattern": pattern, "background": background, "runs": []}
            for index, variant in enumerate(order):
                label = f"{pattern}-bg{background}-{index}-{variant}"
                root = roots[variant]
                output = out / f"{label}.json"
                execute(["taskset", "-c", str(cpu), "node", str(driver),
                         "--kernel-pkg", str(root / "crates/wasm/pkg"),
                         "--io-pkg", str(root / "crates/wasm-io/pkg"),
                         "--fixture", str(fixture), "--ticks", str(ticks), "--warmup", "5",
                         "--package-source-sha", metadata["source"][variant],
                         "--move", "prepared", "--pattern", pattern, "--background", str(background),
                         "--out", str(output)], label)
                summary = validate_capture(json.loads(output.read_text()), ticks, 5,
                                           kernels[variant], translators[variant], harness_sha)
                case["runs"].append({"variant": variant, "file": output.name, **summary})
                print(label, summary["stagesMs"]["tick"]["p50"], flush=True)
            medians = {v: statistics.median(row["stagesMs"]["tick"]["p50"]
                                           for row in case["runs"] if row["variant"] == v) for v in roots}
            case["medianOfProcessP50Ms"] = medians
            case["candidateToBaselineRatio"] = medians["candidate"] / medians["baseline"]
            with_transfer = {v: statistics.median(row["stagesMs"]["tickWithTransfer"]["p50"]
                                                  for row in case["runs"] if row["variant"] == v)
                             for v in roots}
            case["medianOfProcessP50WithTransferMs"] = with_transfer
            case["candidateToBaselineWithTransferRatio"] = with_transfer["candidate"] / with_transfer["baseline"]
            metadata["cases"].append(case)
            save()

        # Existing stronger completion qualification checks complete arena,
        # volume bits, features, mesh buffers and transactional edit outcomes.
        execute([sys.executable, str(roots["candidate"] / "scripts/perf/filleted-face-edit/run_wasm_edit_comparison.py"),
                 "--old-kernel", str(roots["baseline"] / "crates/wasm/pkg"),
                 "--old-io", str(roots["baseline"] / "crates/wasm-io/pkg"),
                 "--new-kernel", str(roots["candidate"] / "crates/wasm/pkg"),
                 "--new-io", str(roots["candidate"] / "crates/wasm-io/pkg"),
                 "--fixture", str(fixture), "--out", str(out / "completion"),
                 "--warmup", "2", "--samples", "4", "--rounds", "2", "--cpu", str(cpu)], "completion")

        for index, variant in enumerate(order):
            label = f"native-{index}-{variant}"
            binary = native_binaries[variant]
            if sha256(binary) != native_hashes[variant]:
                raise ValueError("Native benchmark binary changed during the comparison")
            criterion = out / label
            execute(["taskset", "-c", str(cpu), str(binary), "new_edit_orientation",
                     "--bench", "--noplot", "--save-baseline", label], label,
                    {**env, "CRITERION_HOME": str(criterion)})
            estimates = list(criterion.rglob("estimates.json"))
            if not estimates:
                raise ValueError("Criterion did not retain its estimator")
            metadata["native"].append({"variant": variant, "binarySha256": native_hashes[variant],
                                       "estimates": {str(p.relative_to(out)): json.loads(p.read_text()) for p in estimates}})
            save()
        if hashes != {v: package_hashes(r) for v, r in roots.items()}:
            raise ValueError("Package files changed during the comparison")
        if sha256(driver) != harness_sha or sha256(fixture) != metadata["fixtureSha256"]:
            raise ValueError("Preview driver or fixture changed during the comparison")
        if native_drivers != {v: sha256(r / "crates/io/benches/nurbs_properties.rs") for v, r in roots.items()}:
            raise ValueError("Native benchmark driver changed during the comparison")
        if native_hashes != {v: sha256(p) for v, p in native_binaries.items()}:
            raise ValueError("Native benchmark binary changed during the comparison")
        metadata["status"] = "passed"
    except Exception as error:
        metadata.update(status="failed", error=str(error))
        raise
    finally:
        metadata["finishedUtc"] = dt.datetime.now(dt.timezone.utc).isoformat()
        save()


if __name__ == "__main__":
    main()
