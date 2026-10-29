# Preliminary torus raw-sample carry evidence

This directory preserves **pre-lint working-tree evidence**, not final-source or packaged qualification. The primary tested snapshot is `cfe5977992fb4ae2580844c2140a9e0f41892b4590316c9308ef7c9db130d235`. The default 47-test classifier run used its production-identical predecessor `6098b364b5c67957c811cf97c5b670f1f263b8f1854c913c5f920c5c6f80a102`; final perf-counters 47/47 and default domain differential 1/1 used `cfe597…`. The owner subsequently made test-only lint fixes. Final immutable workspace, canonical package and performance gates are separate evidence.

[qualification.json](qualification.json) and [report.md](report.md) are byte-for-byte copies of the verified preliminary receipts. Four portable test logs retain the observed results in lossless JSON text payloads; checkout prefixes were redacted before those logs were frozen. Each decoded payload retains exactly the SHA-256 listed under `portable_log_sha256`. The raw local log hashes remain in the receipt.

The baseline-equivalent uncached control passed its output and logical-order comparisons, then failed the desired work guard with 1,788/1,788 evaluations (expected exit 101). The carried candidate passed 1,023/1,788. On four actual holder torus boundaries, counts changed 45,482→26,129, 44,033→25,301, 876→612 and 876→612: 91,267→52,654 in total. Every polygon, centre and margin bit, plus every logical parameter/anchor call, matched literal original algorithms. These deterministic test work counts establish **no elapsed-time improvement or shipping instruction-count reduction**.

The reviewed and tested production prefixes are byte-identical. The preliminary receipt hashes bytes before `#[cfg(test)]` at the unit-test module, including its preceding LF: `b474b8c215c47eb91f4e4b1221d1d3db0ec8c82d369fae85726cf95b71389335`. The independent review excludes that last LF and records `e88423deb7437d268c3f9dabc982c34baa5a7c3dcd29eb6b65249494813d3314`. Both definitions compare equal across `6098…`, `cfe597…` and the owner source observed here: commit `7364ab7be5e598b7c868bd63239dc831cbceb933`, file SHA-256 `6439e3d7b658cac1f32c6339f4c1a96347d7030d251e4f3fc33ccad5842974c7`. This is a production-text comparison only; no final gate result is inferred.

[independent-oracle-review.json](independent-oracle-review.json) is retained verbatim. It now records the owner's updated source above, explicitly retains the earlier `6098…` review identity, and rechecks both original reference functions. The review removes comments as well as whitespace, reference-name differences, the two test-observer calls and optional call trailing commas. The preliminary receipt's mechanical body comparison retains comments, so their normalized body hashes use different policies. Neither normalizes expressions, constants, branches or comparisons. The independent review is source evidence, not a canonical package or performance pass.

[candidate-promoted-v2.patch.json](candidate-promoted-v2.patch.json) reconstructs tested pre-lint source `cfe597…`. [uncached-control.patch.json](uncached-control.patch.json) reconstructs control source `f1d0ed78ffbdd8cd5fe90235aa714cc68ebe525d0c92c325b4f7e77dd09e30b6`; [uncached-control-result.json](uncached-control-result.json) records its scope and expected failed work guard. Both patches apply to the baseline torus file at `c754a9b7f89971691ab8867bf7d7bafcc022ad14`, also byte-identical at preliminary base `292f084…`. The existing public holder fixture has the same bytes at that baseline. Use separate isolated checkouts for the alternatives. No new fixture, dependency, layer edge, tolerance, default, public API or resource budget is introduced.

The patch and diagnostic-log payloads are losslessly JSON-encoded to retain unified-diff context spaces and original log endings without introducing whitespace findings in evidence files. Each payload records the original patch SHA-256 and reconstructed source SHA-256. Both patches were decoded and applied to the saved baseline file in a temporary directory; the reconstructed bytes matched their recorded source hashes. The original local raw patches remain untouched.

Extract a patch payload before applying it, for example:

```sh
python3 -c 'import json,pathlib; p=pathlib.Path("candidate-promoted-v2.patch.json"); pathlib.Path("candidate.patch").write_text(json.loads(p.read_text())["patch"])'
```

Decode a log similarly with `json.loads(path.read_text())["text"]`. The decoded bytes, not the JSON wrapper bytes, match the preliminary log hashes.

A bounded reproduction uses the native classifier tests:

```sh
taskset -c 0-14,16-30 env CARGO_BUILD_JOBS=2 RUST_MIN_STACK=536870912 \
  cargo test -p remus-algo --features perf-counters --lib classifier:: -- --nocapture
```

The control's `carried_parent_probes_have_deterministic_raw_work_reduction` test is expected to fail; the control is not claimed as an old release binary. All original local preparations, source snapshots and logs remain preserved. Nothing here claims final gates passed.
