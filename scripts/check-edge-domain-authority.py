#!/usr/bin/env python3
"""Enforce RFC 0002 edge-domain authority migration inventories.

Identities are pinned to an immutable baseline. Approved reader and direct
boundary-mutation identities may disappear, but unknown identities fail even
when the total count is unchanged. Required trim-preservation identities are
non-decreasing and fixed missing-writer paths add weighted requirements.
"""

from __future__ import annotations

import argparse
import hashlib
import re
import sys
from collections.abc import Iterable
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CRATES = ROOT / "crates"

BASELINE_SHA = "39c7a7b7ccbfc746ed7d9e9b8f156d54d6cfe090"
BASELINE_PRODUCTION_READERS = 131
BASELINE_DEFINITIONS = 2
BASELINE_INTERNAL_FALLBACKS = 1
BASELINE_TEST_READERS = 25
BASELINE_BOUNDARY_MUTATIONS = 30
BASELINE_PRESERVATION_WRITES = 12

DOMAIN_PRODUCTION_MANIFEST = """c4c575621dc65a20 05aba1a7403f1df4 0292d028b7b6affa 35483546ac9f4466 ed958f09037a8eba 42116742a7822855 bf2fb9808357661b 66fb58bd830bbca3 b2b1fc0fc2e8af14 50100bd8f229542b 6bcf3516af95a0aa 494798c4b443519d 636c37b3cab5ef88 024441844579149c 8294c01f06b4b166 016980917593694f 8ed0fa28945c0a59 81c10a1d9f22cf8a 8040d0ce7a23c36b af282aaaedfc6fde 083ef00495c0ff44 1b827ba05788c75b 5022445594be1644 054a7d0dfd92bc29 2df2efe33324a022 bbdfe003f54e5f1a 9fbd7025b214049c 05cf3d67cf4035ec c4fa3463da251f4a 7f73185451a14a2a 1eb8ae09ffc0854c d8e726e6ea523c72 4ea4ea94ecf56c58 42f33fde4d5959a3 18b0113ddcdaef0a 51a07ed895e13c28 51a06728883ea93c dfc1e97e3164a593 da23b812eee24822 ccdefc857be5c4d2 bb1a9f8e88e1c5bc 0076b3437beeb3d3 d21fc0fc7032c963 664dc474f66ad343 a080cd1f41760ed0 a76bf7cee511c249 6c4a3ec11003b60d 134fcfb42433b1ce 42b5fa8d2acaa080 0ed82dd07c44dfb8 b2aa88cca9dfcedd 0421af0d4824fb0e 396adfb2f5df7e21 c2dbab4c172d4184 710ab8281584feaa 64fb7cfc5aa25179 514ffe5400a8afb6 f423fe5b37d68666 edc923b10ccb381a 8d7bc3b0b5a4f551 90162e0110c9b044 15453f34f05380a5 33e420ff254e8810 ea5899202681102e 84bfbb94bc8c851d 2f3ecc164a4c518f e8f05f2cb32837d5 78aef34ca46784d5 64448a1386975aad cd816be00f89a239 1fb9f3faeb5c5efa 982b25d696aa0788 905612e254d4b339 a611022ccd77b019 16653eba4048edc8 5438c4ddc236b10e 4a55e478f988720f f0a5c1ab015175ed 839ed40a0e22ca51 d4d2004926f8bc17 72b36f3c58f53c4d ad62283f30bfcae0 3ff9506315ac7eeb 577dc1a69820d29d bff1802d7f5ee650 bf21ef47ddd98bfd deb6e1960b9298c7 43e36b9842d1a298 9c72dd7ac1bc3381 f7659d3645a72cf7 6c02c7a627f57c8c b9e837c95cea995c f1ee0d1086f5190e 1d7f191b61da1ced 5cee3848d4354177 02e67199398859da 8a886e8f6e6cd82e 084ca8540010c1de 21fc0711eb57b3b2 eb1218a73b7c94dd cfdc4915b2dcbcd0 b651adbe89e0da27 afc49641658db12c ed97f0344c07fdfe 7a3cbfbaac6e5177 27ae2c8d7500b4fb d663b10bc592974f 07e3653fed38c0fb efcf44ab60cd6e82 61c999f628cefed8 2984e71151947a87 f11ddfb1752d6d44 554d585661bbf640 efeabea9b799a74d 30629faa19427c95 044797cebbaa8338 951e43b583f0cc60 56145eeaefa520dc 9e99bc6daf3b7836 66770a47aec4f959 9c526e07992f47e8 a262856a1f7d0c0b 155de79f959de002 58f2fe469d1c5479 2e9efcd51960f8aa 764d16ba49eb8939 c8b1cb99b497696d 5dbd35208b20d823 83b839be9fe42952 6aa7f14c7cc5c61a 865615bfa35d1aa6""".split()
DOMAIN_DEFINITION_MANIFEST = """1f86ec8a22c89f06 809ed15fd0b8401a""".split()
DOMAIN_FALLBACK_MANIFEST = """91a4d2a1068bb6fb""".split()
DOMAIN_TEST_MANIFEST = """8277dadb9701777b 51c0813fea3b625a d47f8b4a5f77d05a 17591aa844a92559 30620a876040ddd5 b6bfb3c711b90ed9 c787b58a2c165c78 7417af0c8434d4f9 9e634c6a7a082a19 c88d358809169bab 7e8707feb2e86f57 3e08d1b88fade05b 40bfa30a8427db2e 97158146fbc808eb 4e7a2871ee80b050 c22a68cba58c7f28 b1bada2d5ab15cd0 7ebef0b284320e42 e87d61a1febfd731 c2b387d808cd08da 204452963e1c5e98 c55dfa9acf88f647 2ed119c302805b91 322638dd3568ec26 866c614a7cf0964e""".split()
BOUNDARY_PRODUCTION_MANIFEST = """fb55de050adbb88c 1d3d3b084411fd14 c23d815cf0a862cd 3cb826f4d02579b5 f003ef01c6bb6b5c 4d8b6fc07e8c3a9d 9e53b906967cfd5e d01b54e628597143 91e219e53e6d8d3e 4446f3cec3e34692 0875bdbd1dff21b7 56e6d89d2bea1373 c109e6e08cc65763 5736f9fd5fbf1c76 bc4653b22d2d6cd0 45aa5e7848f4c888 ff6f52ee9ee26c6a 7bd5ba190dcf648f 75fe315ba99522ed 29abde1caf2c1a69 604d510e5b233e40 a3f1a4d3efa9e063 323be0bc1cd3bf59 a4b432acb8a1243d 7af1bcf9abc3f7d1 47d5c5ce79b171a7 b1da5ab122bcc9e3 5821968b9ea57cc4 8f42785da0314cf8 fb8b14d8cb0fe464""".split()
BOUNDARY_EXCLUDED_MANIFEST = """8fe14bd9c408d729 0562c28e114de4de c0f2a1f22c2e8a8c 7cc937c159b16539 b01c467494ff98a6 68f26eecb04615d0 31b2abde73d32ea3 1c6bd024fe00c4b8""".split()
BASELINE_PRESERVATION_MANIFEST = """409e26059e7657d1 4fcd3d400dabcb8c 7eb3fdf95ecd461e 2ef78a8d560c9b51 f8dac04156521ab6 c0079d093e520988 d6672a48aeae38c1 f36ef17e09585a9d 3c435d48ac3cd69f b77ddaa8423159bb 793fa734cab8252d fe75393c672bf17d""".split()

MISSING_TRIM_CONSTRUCTION_ANCHORS = (
    "crates/algo/src/pave_filler/phase_ff.rs:876 perform_with_context raw section edge",
    "crates/algo/src/pave_filler/phase_ff.rs:976 emit_exact_arc",
    "crates/algo/src/pave_filler/phase_ff.rs:5193 emit_split_circle_arcs",
    "crates/operations/src/boolean/assembly.rs:504 mixed SphereCapFace circle",
    "crates/operations/src/boolean/assembly.rs:615 mixed CylindricalFace circle",
    "crates/operations/src/boolean/mod.rs:2359 box-sphere build_arc_edge",
    "crates/operations/src/boolean/mod.rs:4501 merge_result_vertices rebuild",
    "crates/operations/src/primitives.rs:189 make_cylinder bottom rim",
    "crates/operations/src/primitives.rs:190 make_cylinder top rim",
    "crates/operations/src/primitives.rs:347 make_cone pointed rim",
    "crates/operations/src/primitives.rs:405 make_cone frustum bottom rim",
    "crates/operations/src/primitives.rs:406 make_cone frustum top rim",
)
MISSING_TRIM_SNAPSHOT_ANCHORS = (
    "crates/operations/src/boolean/mod.rs:4330 merge_result_vertices FaceSnap omission",
)
BASELINE_MISSING_TRIM_PATHS = (
    "phase_ff::perform_with_context",
    "phase_ff::emit_exact_arc",
    "phase_ff::emit_split_circle_arcs",
    "boolean::assembly::SphereCapFace",
    "boolean::assembly::CylindricalFace",
    "boolean::box_sphere::build_arc_edge",
    "boolean::merge_result_vertices::snapshot_and_rebuild",
    "primitives::make_cylinder::rims",
    "primitives::make_cone::pointed_rim",
    "primitives::make_cone::frustum_rims",
)
MISSING_TRIM_PATH_WRITER_COUNTS = {
    "phase_ff::perform_with_context": 1,
    "phase_ff::emit_exact_arc": 1,
    "phase_ff::emit_split_circle_arcs": 1,
    "boolean::assembly::SphereCapFace": 1,
    "boolean::assembly::CylindricalFace": 1,
    "boolean::box_sphere::build_arc_edge": 1,
    "boolean::merge_result_vertices::snapshot_and_rebuild": 1,
    "primitives::make_cylinder::rims": 2,
    "primitives::make_cone::pointed_rim": 1,
    "primitives::make_cone::frustum_rims": 2,
}
# Reduce only when the path's implementation, tests, and oracles have landed.
REMAINING_MISSING_TRIM_PATHS = (
    "phase_ff::perform_with_context",
    "phase_ff::emit_exact_arc",
    "phase_ff::emit_split_circle_arcs",
)
# Removing a path above requires its exact new set_trim identities here. This
# mapping is the sole registration point for post-baseline required writers;
# each tuple length must match the pinned path weight.
FIXED_PATH_WRITER_IDENTITIES: dict[str, tuple[str, ...]] = {
    "boolean::assembly::SphereCapFace": ("b4495bb8cfd29eeb",),
    "boolean::assembly::CylindricalFace": ("1ee00d4def91bbfd",),
    "boolean::box_sphere::build_arc_edge": ("e947102e0733a63b",),
    "boolean::merge_result_vertices::snapshot_and_rebuild": ("8fa4ac4bcd5a517f",),
    "primitives::make_cylinder::rims": (
        "3be2b1df6dc3bc40",
        "8ea7c2b262f86493",
    ),
    "primitives::make_cone::pointed_rim": ("fc2617ad85a661d5",),
    "primitives::make_cone::frustum_rims": (
        "770c0c4e72efbff5",
        "0b80bd84fb7a0dfe",
    ),
}

DOMAIN_PATTERN = re.compile(r"domain_with_endpoints\s*\(")
BOUNDARY_PATTERN = re.compile(r"\.(?:wire_mut|inner_wires_mut|set_outer_wire)\s*\(")
PRESERVATION_PATTERN = re.compile(r"\.set_trim\s*\(")
FUNCTION_PATTERN = re.compile(
    r"^\s*(?:pub(?:\s*\([^)]*\))?\s+)?(?:async\s+)?"
    r"fn\s+([A-Za-z_][A-Za-z0-9_]*)"
)
SHA_PATTERN = re.compile(r"^[0-9a-f]{40}$")


def fail(message: str) -> None:
    print(f"VIOLATION: {message}", file=sys.stderr)


def identity_hash(identity: str) -> str:
    return hashlib.sha256(identity.encode()).hexdigest()[:16]


def validate_manifest(manifest: list[str], expected: int, label: str) -> bool:
    unique = set(manifest)
    if len(manifest) != expected or len(unique) != expected:
        fail(
            f"malformed {label} manifest: entries={len(manifest)} "
            f"unique={len(unique)} expected={expected}"
        )
        return False
    return True


def source_files() -> list[Path]:
    return sorted(CRATES.rglob("*.rs"), key=lambda path: path.relative_to(ROOT).as_posix())


def read_sources() -> dict[Path, list[str]]:
    sources: dict[Path, list[str]] = {}
    for path in source_files():
        try:
            sources[path] = path.read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeError) as error:
            fail(f"cannot read {path.relative_to(ROOT)}: {error}")
            raise SystemExit(1) from error
    return sources


def matching_sites(
    sources: dict[Path, list[str]], pattern: re.Pattern[str]
) -> list[tuple[Path, int, str]]:
    sites: list[tuple[Path, int, str]] = []
    for path, lines in sources.items():
        text = "\n".join(lines)
        for match in pattern.finditer(text):
            line_number = text.count("\n", 0, match.start()) + 1
            sites.append((path, line_number, lines[line_number - 1]))
    return sites


def enclosing_function(lines: list[str], line_number: int) -> str:
    scope = ""
    for line in lines[:line_number]:
        match = FUNCTION_PATTERN.search(line)
        if match:
            scope = match.group(1)
    return scope


def site_base_identity(
    path: Path, line_number: int, sources: dict[Path, list[str]]
) -> str:
    lines = sources[path]
    scope = enclosing_function(lines, line_number)
    start = max(0, line_number - 3)
    end = line_number + 2
    context = "".join("".join(lines[start:end]).split())
    context_hash = hashlib.sha256(context.encode()).hexdigest()[:16]
    relative = path.relative_to(ROOT).as_posix()
    return f"{relative}::{scope}::{context_hash}"


def classified_sites(
    sites: Iterable[tuple[Path, int, str]],
    sources: dict[Path, list[str]],
    manifest: dict[str, str],
) -> tuple[dict[str, list[str]], list[str]]:
    categories: dict[str, list[str]] = {}
    unknown: list[str] = []
    seen_bases: dict[str, int] = {}
    for path, line_number, line in sites:
        base = site_base_identity(path, line_number, sources)
        ordinal = seen_bases.get(base, 0) + 1
        seen_bases[base] = ordinal
        identity = f"{base}::{ordinal}"
        hashed = identity_hash(identity)
        relative = path.relative_to(ROOT).as_posix()
        record = f"{hashed}|{identity}|{relative}:{line_number}:{line}"
        category = manifest.get(hashed)
        if category is None:
            unknown.append(record)
        else:
            categories.setdefault(category, []).append(record)
    return categories, unknown


def preservation_sites(
    sites: Iterable[tuple[Path, int, str]], sources: dict[Path, list[str]]
) -> dict[str, str]:
    current: dict[str, str] = {}
    seen_bases: dict[str, int] = {}
    for path, line_number, line in sites:
        base = site_base_identity(path, line_number, sources)
        ordinal = seen_bases.get(base, 0) + 1
        seen_bases[base] = ordinal
        hashed = identity_hash(f"{base}::{ordinal}")
        relative = path.relative_to(ROOT).as_posix()
        current[hashed] = f"{relative}:{line_number}:{line}"
    return current


def validate_static_configuration() -> bool:
    valid = True
    valid &= validate_manifest(
        DOMAIN_PRODUCTION_MANIFEST,
        BASELINE_PRODUCTION_READERS,
        "domain-production",
    )
    valid &= validate_manifest(
        DOMAIN_DEFINITION_MANIFEST, BASELINE_DEFINITIONS, "domain-definition"
    )
    valid &= validate_manifest(
        DOMAIN_FALLBACK_MANIFEST,
        BASELINE_INTERNAL_FALLBACKS,
        "domain-fallback",
    )
    valid &= validate_manifest(
        DOMAIN_TEST_MANIFEST, BASELINE_TEST_READERS, "domain-test"
    )
    valid &= validate_manifest(
        BOUNDARY_PRODUCTION_MANIFEST,
        BASELINE_BOUNDARY_MUTATIONS,
        "boundary-production",
    )
    valid &= validate_manifest(BOUNDARY_EXCLUDED_MANIFEST, 8, "boundary-excluded")
    valid &= validate_manifest(
        BASELINE_PRESERVATION_MANIFEST,
        BASELINE_PRESERVATION_WRITES,
        "baseline-trim-preservation",
    )

    domain_manifests = (
        DOMAIN_PRODUCTION_MANIFEST,
        DOMAIN_DEFINITION_MANIFEST,
        DOMAIN_FALLBACK_MANIFEST,
        DOMAIN_TEST_MANIFEST,
    )
    domain_hashes = [value for manifest in domain_manifests for value in manifest]
    if len(domain_hashes) != len(set(domain_hashes)):
        fail("domain identity manifests overlap")
        valid = False
    boundary_hashes = BOUNDARY_PRODUCTION_MANIFEST + BOUNDARY_EXCLUDED_MANIFEST
    if len(boundary_hashes) != len(set(boundary_hashes)):
        fail("boundary identity manifests overlap")
        valid = False
    identity_hash_pattern = re.compile(r"^[0-9a-f]{16}$")
    all_hashes = (
        domain_hashes + boundary_hashes + BASELINE_PRESERVATION_MANIFEST
    )
    if any(identity_hash_pattern.fullmatch(value) is None for value in all_hashes):
        fail("identity manifests contain a malformed SHA-256 prefix")
        valid = False

    baseline_paths = set(BASELINE_MISSING_TRIM_PATHS)
    if len(baseline_paths) != len(BASELINE_MISSING_TRIM_PATHS):
        fail("baseline missing-trim path inventory contains duplicates")
        valid = False
    if set(MISSING_TRIM_PATH_WRITER_COUNTS) != baseline_paths:
        fail("writer-count weights do not match the baseline missing-path inventory")
        valid = False
    if any(weight <= 0 for weight in MISSING_TRIM_PATH_WRITER_COUNTS.values()):
        fail("writer-count weights must be positive")
        valid = False
    if not SHA_PATTERN.fullmatch(BASELINE_SHA):
        fail("baseline SHA is not a 40-character lowercase hex commit")
        valid = False

    # Deterministic stdlib self-checks also pin whitespace-tolerant UFCS and
    # non-public definition matching, which the inventory relies on.
    if identity_hash("abc") != "ba7816bf8f01cfea":
        fail("hashlib SHA-256 identity self-test failed")
        valid = False
    if not DOMAIN_PATTERN.search("EdgeCurve::domain_with_endpoints (edge)"):
        fail("UFCS domain scanner self-test failed")
        valid = False
    if not DOMAIN_PATTERN.search("fn domain_with_endpoints\n("):
        fail("whitespace-tolerant domain scanner self-test failed")
        valid = False
    return valid


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check RFC 0002 edge-domain authority identity inventories."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--list", action="store_true")
    args = parser.parse_args()

    if not validate_static_configuration():
        return 1
    failed = False
    sources = read_sources()

    domain_manifest = {
        **{value: "production" for value in DOMAIN_PRODUCTION_MANIFEST},
        **{value: "definition" for value in DOMAIN_DEFINITION_MANIFEST},
        **{value: "internal_fallback" for value in DOMAIN_FALLBACK_MANIFEST},
        **{value: "test_example" for value in DOMAIN_TEST_MANIFEST},
    }
    domain, unknown_domain = classified_sites(
        matching_sites(sources, DOMAIN_PATTERN), sources, domain_manifest
    )
    if unknown_domain:
        fail("unknown domain_with_endpoints identities:")
        for record in unknown_domain:
            print(f"  {record}", file=sys.stderr)
        failed = True

    boundary_manifest = {
        **{value: "production" for value in BOUNDARY_PRODUCTION_MANIFEST},
        **{value: "excluded_baseline" for value in BOUNDARY_EXCLUDED_MANIFEST},
    }
    boundary, unknown_boundary = classified_sites(
        matching_sites(sources, BOUNDARY_PATTERN), sources, boundary_manifest
    )
    if unknown_boundary:
        fail("unknown direct boundary-mutation identities:")
        for record in unknown_boundary:
            print(f"  {record}", file=sys.stderr)
        failed = True

    current_preservation = preservation_sites(
        matching_sites(sources, PRESERVATION_PATTERN), sources
    )
    baseline_paths = set(BASELINE_MISSING_TRIM_PATHS)
    remaining_paths = set(REMAINING_MISSING_TRIM_PATHS)
    if (
        len(remaining_paths) != len(REMAINING_MISSING_TRIM_PATHS)
        or not remaining_paths <= baseline_paths
    ):
        fail("manually reduced missing-trim path manifest is invalid")
        failed = True

    baseline_preservation = set(BASELINE_PRESERVATION_MANIFEST)
    fixed_writer_count = 0
    claimed_fixed_identities: set[str] = set()
    for path in BASELINE_MISSING_TRIM_PATHS:
        fixed_identities = FIXED_PATH_WRITER_IDENTITIES.get(path, ())
        if path in remaining_paths:
            if fixed_identities:
                fail(f"remaining missing-trim path has fixed-writer identities: {path}")
                failed = True
            continue

        expected = MISSING_TRIM_PATH_WRITER_COUNTS[path]
        unique = set(fixed_identities)
        if len(fixed_identities) != expected or len(unique) != expected:
            fail(
                f"fixed path {path} has writers={len(fixed_identities)} "
                f"unique={len(unique)} expected={expected}"
            )
            failed = True
        for hashed in fixed_identities:
            if hashed in baseline_preservation:
                fail(
                    f"fixed writer identity for {path} reuses immutable "
                    f"baseline writer: {hashed}"
                )
                failed = True
            if hashed in claimed_fixed_identities:
                fail(f"fixed writer identity is claimed by multiple paths: {hashed}")
                failed = True
            claimed_fixed_identities.add(hashed)
        fixed_writer_count += expected

    unknown_fixed_paths = set(FIXED_PATH_WRITER_IDENTITIES) - baseline_paths
    for path in sorted(unknown_fixed_paths):
        fail(f"fixed-writer manifest names unknown path: {path}")
        failed = True

    expected_preservation = BASELINE_PRESERVATION_WRITES + fixed_writer_count
    required_preservation = baseline_preservation | claimed_fixed_identities
    if not validate_manifest(
        list(required_preservation), expected_preservation, "trim-preservation"
    ):
        failed = True
    missing_preservation = sorted(required_preservation - current_preservation.keys())
    for hashed in missing_preservation:
        fail(f"required trim-preservation identity disappeared: {hashed}")
    if missing_preservation:
        failed = True

    production = domain.get("production", [])
    definitions = domain.get("definition", [])
    fallbacks = domain.get("internal_fallback", [])
    test_readers = domain.get("test_example", [])
    boundary_production = boundary.get("production", [])
    boundary_excluded = boundary.get("excluded_baseline", [])
    preservation_present = sum(
        hashed in current_preservation for hashed in required_preservation
    )

    print(f"edge-domain identity baseline: {BASELINE_SHA}")
    print(
        "domain identities: "
        f"production={len(production)}/{BASELINE_PRODUCTION_READERS} "
        f"definitions={len(definitions)}/{BASELINE_DEFINITIONS} "
        f"internal_fallback={len(fallbacks)}/{BASELINE_INTERNAL_FALLBACKS} "
        f"tests_examples={len(test_readers)}/{BASELINE_TEST_READERS} "
        f"unknown={len(unknown_domain)}"
    )
    print(
        "trim preservation identities: "
        f"present={preservation_present}/"
        f"{expected_preservation} missing={len(missing_preservation)}"
    )
    print(
        "missing-trim measured anchors: "
        f"constructions={len(MISSING_TRIM_CONSTRUCTION_ANCHORS)} "
        f"snapshot_omissions={len(MISSING_TRIM_SNAPSHOT_ANCHORS)} "
        f"remaining_paths={len(REMAINING_MISSING_TRIM_PATHS)}/"
        f"{len(BASELINE_MISSING_TRIM_PATHS)} (manual manifest)"
    )
    print(
        "boundary-mutation identities: "
        f"production={len(boundary_production)}/{BASELINE_BOUNDARY_MUTATIONS} "
        f"excluded_baseline={len(boundary_excluded)} "
        f"unknown={len(unknown_boundary)}"
    )

    if args.list:
        sections = (
            ("current production domain identities", production),
            ("current definition identities", definitions),
            ("current internal fallback identities", fallbacks),
            ("current test/example identities", test_readers),
            (
                "immutable missing-trim construction anchors",
                MISSING_TRIM_CONSTRUCTION_ANCHORS,
            ),
            (
                "immutable missing-trim snapshot anchors",
                MISSING_TRIM_SNAPSHOT_ANCHORS,
            ),
            (
                "manually reviewed remaining missing-trim paths",
                REMAINING_MISSING_TRIM_PATHS,
            ),
            (
                "current production boundary-mutation identities",
                boundary_production,
            ),
        )
        for heading, records in sections:
            print(f"\n[{heading}]")
            print("\n".join(records))

    if failed:
        return 1
    print("edge-domain identity ratchet passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
