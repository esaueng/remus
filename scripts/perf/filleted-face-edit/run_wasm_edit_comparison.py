#!/usr/bin/env python3
"""Serial old/new Node WASM qualification and warmed edit-stage comparison."""
from __future__ import annotations

import argparse
import array
import json
import math
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import time


HERE = Path(__file__).resolve().parent
REPOSITORY = HERE.parents[2]
EPS32 = 2.0 ** -23
EPS64 = 2.0 ** -52
STAGES = ("move", "validate", "bbox", "meshKernel", "meshTransfer", "mesh", "volume",
          "recognize", "kernelPipeline", "pipelineWithTransfer", "restoredPipeline")


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--old-kernel", type=Path, required=True)
    parser.add_argument("--old-io", type=Path, required=True)
    parser.add_argument("--new-kernel", type=Path, default=REPOSITORY / "crates/wasm/pkg")
    parser.add_argument("--new-io", type=Path, default=REPOSITORY / "crates/wasm-io/pkg")
    parser.add_argument("--fixture", type=Path,
                        default=REPOSITORY / "crates/io/tests/data/shapr3d_hammer_holder.step")
    parser.add_argument("--out", type=Path,
                        default=Path.cwd() / time.strftime("wasm-comparison-%Y%m%d-%H%M%S", time.gmtime()))
    parser.add_argument("--node", default="node")
    parser.add_argument("--warmup", type=int, default=2)
    parser.add_argument("--samples", type=int, default=6)
    parser.add_argument("--rounds", type=int, default=2,
                        help="Independent process pairs, alternating old/new launch order")
    parser.add_argument("--cpu", default="auto",
                        help="Linux CPU index, auto (lowest allowed CPU), or none; inherited by Node")
    parser.add_argument("--order", choices=("both", "geometry-first", "volume-first"), default="both",
                        help="Benchmark both next-drag scheduling orders in independent processes")
    args = parser.parse_args()
    if args.warmup < 0 or args.samples < 1 or args.rounds < 1:
        parser.error("warmup must be nonnegative; samples and rounds must be positive")
    for key in ("old_kernel", "old_io", "new_kernel", "new_io", "fixture", "out"):
        setattr(args, key, getattr(args, key).resolve())
    return args


def pin_cpu(choice: str) -> int | None:
    if choice == "none":
        return None
    if not hasattr(os, "sched_getaffinity"):
        if choice == "auto":
            return None
        raise RuntimeError("Explicit CPU affinity is unsupported on this platform")
    allowed = os.sched_getaffinity(0)
    cpu = min(allowed) if choice == "auto" else int(choice)
    if cpu not in allowed:
        raise RuntimeError(f"CPU {cpu} is outside the current affinity: {sorted(allowed)}")
    os.sched_setaffinity(0, {cpu})
    return cpu


def exact_tree(a: object, b: object) -> bool:
    # Require the full feature record, including face identities and numeric values.
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(exact_tree(a[k], b[k]) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(exact_tree(x, y) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return type(a) is type(b) and a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return a == b
    return type(a) is type(b) and a == b


def compare_arena(old_file: Path, new_file: Path) -> dict:
    old_bytes, new_bytes = old_file.read_bytes(), new_file.read_bytes()
    if old_bytes == new_bytes:
        return {"byteExact": True, "semanticExact": True, "near": True,
                "numericDifferences": 0, "maxAbsoluteDifference": 0.0, "firstFailures": []}
    old, new = json.loads(old_bytes), json.loads(new_bytes)
    result = {"byteExact": False, "semanticExact": exact_tree(old, new), "near": True,
              "numericDifferences": 0, "maxAbsoluteDifference": 0.0, "firstFailures": []}

    def fail(location: str, a: object, b: object) -> None:
        result["near"] = False
        if len(result["firstFailures"]) < 6:
            result["firstFailures"].append({"path": location, "old": a, "new": b})

    def visit(a: object, b: object, location: str) -> None:
        if isinstance(a, dict) and isinstance(b, dict):
            if a.keys() != b.keys():
                fail(location, sorted(a.keys()), sorted(b.keys()))
                return
            for key in a:
                visit(a[key], b[key], f"{location}.{key}")
        elif isinstance(a, list) and isinstance(b, list):
            if len(a) != len(b):
                fail(location + ".length", len(a), len(b))
                return
            for index, (x, y) in enumerate(zip(a, b)):
                visit(x, y, f"{location}[{index}]")
        elif isinstance(a, bool) or isinstance(b, bool):
            if type(a) is not type(b) or a != b:
                fail(location, a, b)
        elif isinstance(a, (int, float)) and isinstance(b, (int, float)):
            if a != b:
                result["numericDifferences"] += 1
                difference = abs(a - b)
                result["maxAbsoluteDifference"] = max(result["maxAbsoluteDifference"], difference)
                # Integer topology fields remain exact, including IDs above 2**53.
                integer_change = isinstance(a, int) and isinstance(b, int)
                if integer_change or not math.isfinite(a) or not math.isfinite(b) or \
                        difference > 64 * EPS64 * max(1.0, abs(a), abs(b)):
                    fail(location, a, b)
        elif type(a) is not type(b) or a != b:
            fail(location, a, b)

    visit(old, new, "arena")
    return result


def float32_mesh_comparison(old_file: Path, new_file: Path) -> dict:
    old_bytes, new_bytes = old_file.read_bytes(), new_file.read_bytes()
    if len(old_bytes) != len(new_bytes):
        return {"byteExact": False, "near": False,
                "oldElements": len(old_bytes) // 4, "newElements": len(new_bytes) // 4}
    if old_bytes == new_bytes:
        return {"byteExact": True, "near": True, "elements": len(old_bytes) // 4,
                "differentBits": 0, "differentValues": 0, "maxAbsoluteDifference": 0.0,
                "maxScaledDifference": 0.0, "firstFailures": []}
    old, new = array.array("f"), array.array("f")
    old.frombytes(old_bytes)
    new.frombytes(new_bytes)
    if sys.byteorder != "little":
        old.byteswap()
        new.byteswap()
    result = {"byteExact": False, "near": True, "elements": len(old), "differentBits": 0,
              "differentValues": 0, "maxAbsoluteDifference": 0.0,
              "maxScaledDifference": 0.0, "firstFailures": []}
    for index, (a, b) in enumerate(zip(old, new)):
        if old_bytes[index * 4:index * 4 + 4] != new_bytes[index * 4:index * 4 + 4]:
            result["differentBits"] += 1
        if a != b:
            result["differentValues"] += 1
        difference = abs(a - b)
        scaled = difference / max(1.0, abs(a), abs(b))
        result["maxAbsoluteDifference"] = max(result["maxAbsoluteDifference"], difference)
        result["maxScaledDifference"] = max(result["maxScaledDifference"], scaled)
        # Four Float32 epsilon units per component. Topology and grouping must
        # still be byte-exact; arena geometry and volume have separate checks.
        if not math.isfinite(a) or not math.isfinite(b) or scaled > 4 * EPS32:
            result["near"] = False
            if len(result["firstFailures"]) < 6:
                result["firstFailures"].append({"index": index, "old": a, "new": b})
    return result


def qualify_pair(old: dict, new: dict, old_dir: Path, new_dir: Path) -> dict:
    old_artifacts, new_artifacts = old["artifacts"], new["artifacts"]
    geometry = compare_arena(old_dir / old_artifacts["arena"]["file"],
                             new_dir / new_artifacts["arena"]["file"])
    mesh = {}
    for field in ("positions", "normals", "indices", "faceOffsets"):
        old_file = old_dir / old_artifacts[field]["file"]
        new_file = new_dir / new_artifacts[field]["file"]
        if field in ("positions", "normals"):
            mesh[field] = float32_mesh_comparison(old_file, new_file)
        else:
            exact = old_file.read_bytes() == new_file.read_bytes()
            mesh[field] = {"byteExact": exact, "near": exact,
                           "oldElements": old_artifacts[field]["elements"],
                           "newElements": new_artifacts[field]["elements"]}
    volume_exact = old["volumeBits"] == new["volumeBits"]
    volume_relative = abs(old["volume"] - new["volume"]) / max(1.0, abs(old["volume"]), abs(new["volume"]))
    volume_near = volume_relative <= 64 * EPS64
    features_exact = exact_tree(old["features"], new["features"])
    mesh_near = all(comparison["near"] for comparison in mesh.values())
    discrete_exact = old["faces"] == new["faces"] and \
        old["surfaceCounts"] == new["surfaceCounts"] and \
        old["mesh"]["triangles"] == new["mesh"]["triangles"] and \
        exact_tree(old["analyticRadii"], new["analyticRadii"])
    valid = old.get("valid", True) and new.get("valid", True) and \
        old["mesh"]["watertight"] and new["mesh"]["watertight"]
    return {"qualified": geometry["near"] and mesh_near and volume_exact and
            features_exact and discrete_exact and valid,
            "geometry": geometry, "mesh": mesh, "featuresExact": features_exact,
            "featureJsonByteExact": old["featuresJsonSha256"] == new["featuresJsonSha256"],
            "volumeExact": volume_exact, "volumeNear": volume_near,
            "volumeAbsoluteDifference": abs(old["volume"] - new["volume"]),
            "volumeRelativeDifference": volume_relative, "discreteGeometryExact": discrete_exact,
            "validAndWatertight": valid}


def distribution(values: list[float]) -> dict:
    sorted_values = sorted(values)
    def quantile(q: float) -> float:
        return sorted_values[max(0, math.ceil(len(values) * q) - 1)]
    return {"n": len(values), "medianMs": statistics.median(values), "p10Ms": quantile(0.1),
            "p90Ms": quantile(0.9), "p95Ms": quantile(0.95),
            "minMs": min(values), "maxMs": max(values),
            "meanMs": statistics.mean(values),
            "stdevMs": statistics.stdev(values) if len(values) > 1 else 0.0}


def launch(args: argparse.Namespace, variant: str, round_index: int, workflow: str) -> tuple[dict, Path]:
    directory = args.out / f"{workflow}-{variant}-round-{round_index}"
    directory.mkdir(parents=True, exist_ok=False)
    command = [args.node, str(HERE / "wasm_edit_bench.cjs"),
               "--kernel-pkg", str(getattr(args, f"{variant}_kernel")),
               "--io-pkg", str(getattr(args, f"{variant}_io")),
               "--fixture", str(args.fixture), "--out", str(directory),
               "--label", directory.name, "--warmup", str(args.warmup), "--samples", str(args.samples),
               "--order", workflow]
    print("Running " + directory.name, file=sys.stderr, flush=True)
    # stderr streams progress; stdout is a complete JSON document stored by
    # the worker too. Processes run sequentially and inherit the pinned CPU.
    completed = subprocess.run(command, stdout=subprocess.PIPE, stderr=None, text=True, check=True)
    return json.loads(completed.stdout), directory


def main() -> int:
    args = arguments()
    cpu = pin_cpu(args.cpu)
    args.out.mkdir(parents=True, exist_ok=False)
    started = time.time()
    manifest = {"command": sys.argv, "python": sys.version, "platform": platform.platform(),
                "cpuPin": cpu, "startedUtc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(started)),
                "warmup": args.warmup, "samplesPerProcess": args.samples, "rounds": args.rounds,
                "workflows": ["geometry-first", "volume-first"] if args.order == "both" else [args.order],
                "comparisonPolicy": {"featureRecords": "exact recursive equality",
                    "indicesAndFaceOffsets": "byte exact",
                    "arena": "byte exact reported; near permits <=64*f64epsilon*max(1,|a|,|b|), integers exact",
                    "volume": "exact f64 bits required; near relative <=64*f64epsilon reported separately",
                    "meshFloats": "f32 bytes reported; near <=4*f32epsilon*max(1,|a|,|b|) per component"}}
    (args.out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    runs = []
    comparisons = []
    for workflow in manifest["workflows"]:
        for round_index in range(args.rounds):
            order = ("old", "new") if round_index % 2 == 0 else ("new", "old")
            paired = {variant: launch(args, variant, round_index, workflow) for variant in order}
            old, old_dir = paired["old"]
            new, new_dir = paired["new"]
            runs.append({"workflow": workflow, "round": round_index, "order": list(order), "old": old, "new": new})
            source = qualify_pair(old["source"], new["source"], old_dir, new_dir)
            records = []
            if len(old["records"]) != len(new["records"]):
                raise RuntimeError("Mismatched record counts")
            for old_record, new_record in zip(old["records"], new["records"]):
                if old_record["iteration"] != new_record["iteration"] or \
                        old_record["distance"] != new_record["distance"] or \
                        old_record["warmup"] != new_record["warmup"]:
                    raise RuntimeError("Mismatched edit sequence")
                records.append({"iteration": old_record["iteration"], "warmup": old_record["warmup"],
                                "distance": old_record["distance"],
                                **qualify_pair(old_record, new_record, old_dir, new_dir)})
            comparisons.append({"workflow": workflow, "round": round_index, "source": source, "records": records})
    qualified = all(comparison["source"]["qualified"] and
                    all(record["qualified"] for record in comparison["records"]) for comparison in comparisons)
    summaries = {}
    for workflow in manifest["workflows"]:
        summaries[workflow] = {}
        workflow_qualified = all(comparison["source"]["qualified"] and
                                 all(record["qualified"] for record in comparison["records"])
                                 for comparison in comparisons if comparison["workflow"] == workflow)
        for stage in STAGES:
            values = {variant: [record["stagesMs"][stage] for run in runs if run["workflow"] == workflow
                                for record in run[variant]["records"] if not record["warmup"]]
                      for variant in ("old", "new")}
            old_stats, new_stats = distribution(values["old"]), distribution(values["new"])
            summaries[workflow][stage] = {"old": old_stats, "new": new_stats,
                                "medianSpeedup": old_stats["medianMs"] / new_stats["medianMs"],
                                "pairedSpeedupMedian": statistics.median(
                                    old_ms / new_ms for old_ms, new_ms in zip(values["old"], values["new"])),
                                "qualifiedForComparison": workflow_qualified}
    pairs = [comparison["source"] for comparison in comparisons] + \
            [record for comparison in comparisons for record in comparison["records"]]
    qualification_summary = {
        "pairsIncludingSourceAndWarmup": len(pairs),
        "arenaByteExact": sum(pair["geometry"]["byteExact"] for pair in pairs),
        "arenaNear": sum(pair["geometry"]["near"] for pair in pairs),
        "volumeBitsExact": sum(pair["volumeExact"] for pair in pairs),
        "volumeNear": sum(pair["volumeNear"] for pair in pairs),
        "featuresExact": sum(pair["featuresExact"] for pair in pairs),
        "displayPositionsByteExact": sum(pair["mesh"]["positions"]["byteExact"] for pair in pairs),
        "displayPositionsNear": sum(pair["mesh"]["positions"]["near"] for pair in pairs),
        "displayNormalsByteExact": sum(pair["mesh"]["normals"]["byteExact"] for pair in pairs),
        "displayNormalsNear": sum(pair["mesh"]["normals"]["near"] for pair in pairs),
        "displayMaxAbsolutePositionDifference": max(pair["mesh"]["positions"].get("maxAbsoluteDifference", 0)
                                                    for pair in pairs),
    }
    result = {"manifest": manifest, "qualified": qualified, "stages": summaries,
              "qualificationSummary": qualification_summary,
              "comparisons": comparisons, "elapsedSeconds": time.time() - started,
              "runs": [{"workflow": run["workflow"], "round": run["round"], "order": run["order"],
                        "oldMetadata": run["old"]["metadata"], "newMetadata": run["new"]["metadata"]}
                       for run in runs]}
    (args.out / "comparison.json").write_text(json.dumps(result, indent=2) + "\n")
    count = qualification_summary["pairsIncludingSourceAndWarmup"]
    lines = [f"Qualified: {qualified}", "",
             f"Float64 arena byte identity: {qualification_summary['arenaByteExact']}/{count} pairs.",
             f"Float64 volume bit identity: {qualification_summary['volumeBitsExact']}/{count} pairs.",
             f"Full feature identity: {qualification_summary['featuresExact']}/{count} pairs.",
             f"Float32 display-position byte identity: {qualification_summary['displayPositionsByteExact']}/{count}; "
             f"bounded near identity: {qualification_summary['displayPositionsNear']}/{count}.",
             f"Maximum Float32 display-position difference: "
             f"{qualification_summary['displayMaxAbsolutePositionDifference']:.9g} model units."]
    if not qualified:
        lines += ["", "Qualification failed: all speedup values below are invalid for performance claims."]
    for workflow, stage_summaries in summaries.items():
        lines += ["", workflow, "", "Stage | Original median ms | New median ms | Median speedup",
                  "--- | ---: | ---: | ---:"]
        for stage, summary in stage_summaries.items():
            lines.append(f"{stage} | {summary['old']['medianMs']:.3f} | "
                         f"{summary['new']['medianMs']:.3f} | {summary['medianSpeedup']:.2f}x")
    lines += ["", "Each warmed sample starts from a restored source body. Process pairs are sequential;",
              "launch order alternates between independent rounds. Exact and near comparisons are",
              "reported separately in comparison.json. Failed qualification invalidates the speedup claim.",
              "This measures Node WASM calls and bulk transfer, excluding browser rendering and ZCAD orchestration."]
    (args.out / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    print(f"\nArtifacts: {args.out}")
    return 0 if qualified else 2


if __name__ == "__main__":
    sys.exit(main())
