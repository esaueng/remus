#!/usr/bin/env node
/**
 * The browser kernel's size policy: the one place its limits live.
 *
 * `cargo xtask wasm-build` evaluates every kernel module it builds with this
 * script, and CI size reporting is meant to call it too, so a limit change is
 * a one-line edit here. The limits, the measurement and the evaluation mirror
 * the consumer policy `KERNEL_WASM_POLICY` in esaueng/openzcad
 * `scripts/bundle-size-policy.mjs` (rationale in its
 * `docs/kernel-wasm-size-policy.md`): raw bytes, Node `zlib.gzipSync()` with
 * default settings and Node Brotli at quality 11. Compressed sizes must come
 * from Node's zlib, not a reimplementation, or they drift from the consumer's
 * numbers. Change the limits together with that policy, never independently.
 *
 * Usage:
 *   node scripts/wasm-size-policy.mjs [--json] <kernel.wasm>
 *   node scripts/wasm-size-policy.mjs --policy
 *
 * Exit status: 0 within the hard limits (review warnings allowed), 1 when a
 * hard limit is exceeded, 2 on a usage or input error.
 */
import { readFileSync } from 'node:fs';
import { pathToFileURL } from 'node:url';
import { brotliCompressSync, constants as zlibConstants, gzipSync } from 'node:zlib';

const MIB = 1024 * 1024;
const WASM_HEADER = Uint8Array.from([0x00, 0x61, 0x73, 0x6d, 0x01, 0x00, 0x00, 0x00]);

export const KERNEL_WASM_POLICY = Object.freeze({
  rawReviewBytes: 11 * MIB,
  rawHardBytes: 12 * MIB,
  gzipHardBytes: 4.5 * MIB,
  brotliReviewBytes: 3 * MIB,
  brotliQuality: 11,
});

/** Raw, gzip and Brotli sizes exactly as the consumer measures them. */
export function measureKernelWasm(contents) {
  if (
    contents.byteLength < WASM_HEADER.byteLength ||
    WASM_HEADER.some((byte, index) => contents[index] !== byte)
  ) {
    throw new Error('Kernel size input is not a version 1 WebAssembly module');
  }
  return {
    rawBytes: contents.byteLength,
    gzipBytes: gzipSync(contents).byteLength,
    brotliBytes: brotliCompressSync(contents, {
      params: { [zlibConstants.BROTLI_PARAM_QUALITY]: KERNEL_WASM_POLICY.brotliQuality },
    }).byteLength,
  };
}

/**
 * Review findings are warnings, hard findings are failures. The comparisons
 * match the consumer's: the raw review threshold is inclusive, every other
 * limit is exceeded only above its exact byte count.
 */
export function evaluateKernelWasm(metrics, policy = KERNEL_WASM_POLICY) {
  const warnings = [];
  const failures = [];
  if (metrics.rawBytes >= policy.rawReviewBytes) {
    warnings.push({
      code: 'kernel-raw-review',
      metric: 'raw bytes',
      bytes: metrics.rawBytes,
      budgetBytes: policy.rawReviewBytes,
      reason: 'Exact geometry kernel requires a size review',
    });
  }
  if (metrics.brotliBytes > policy.brotliReviewBytes) {
    warnings.push({
      code: 'kernel-brotli-review',
      metric: `Brotli q${policy.brotliQuality} bytes`,
      bytes: metrics.brotliBytes,
      budgetBytes: policy.brotliReviewBytes,
      reason: 'Exact geometry kernel requires a compressed-size review',
    });
  }
  if (metrics.rawBytes > policy.rawHardBytes) {
    failures.push({
      code: 'kernel-raw-hard',
      metric: 'raw bytes',
      bytes: metrics.rawBytes,
      budgetBytes: policy.rawHardBytes,
      reason: 'Exact geometry kernel exceeds its raw hard limit',
    });
  }
  if (metrics.gzipBytes > policy.gzipHardBytes) {
    failures.push({
      code: 'kernel-gzip-hard',
      metric: 'gzip bytes',
      bytes: metrics.gzipBytes,
      budgetBytes: policy.gzipHardBytes,
      reason: 'Exact geometry kernel exceeds its gzip hard limit',
    });
  }
  return { warnings, failures };
}

/** Measure and evaluate one module file. */
export function checkKernelWasmFile(file) {
  const metrics = measureKernelWasm(readFileSync(file));
  return {
    file,
    policy: KERNEL_WASM_POLICY,
    toolchain: { node: process.versions.node, zlib: process.versions.zlib, brotli: process.versions.brotli ?? null },
    metrics,
    ...evaluateKernelWasm(metrics),
  };
}

const bytes = (n) => `${n.toLocaleString('en-US')} B`;

export function formatReport(report) {
  const { metrics, policy } = report;
  const lines = [
    `${report.file}`,
    `  raw    ${bytes(metrics.rawBytes)} (review ${bytes(policy.rawReviewBytes)}, hard ${bytes(policy.rawHardBytes)})`,
    `  gzip   ${bytes(metrics.gzipBytes)} (hard ${bytes(policy.gzipHardBytes)})`,
    `  brotli ${bytes(metrics.brotliBytes)} q${policy.brotliQuality} (review ${bytes(policy.brotliReviewBytes)})`,
  ];
  for (const w of report.warnings) {
    lines.push(`  WARN ${w.code}: ${w.metric} ${bytes(w.bytes)} (budget ${bytes(w.budgetBytes)}). ${w.reason}`);
  }
  for (const f of report.failures) {
    lines.push(`  FAIL ${f.code}: ${f.metric} ${bytes(f.bytes)} (budget ${bytes(f.budgetBytes)}). ${f.reason}`);
  }
  return lines.join('\n');
}

function main(argv) {
  const json = argv.includes('--json');
  const args = argv.filter((arg) => arg !== '--json');
  if (args.length === 1 && args[0] === '--policy') {
    console.log(JSON.stringify(KERNEL_WASM_POLICY));
    return 0;
  }
  if (args.length !== 1 || args[0].startsWith('--')) {
    console.error('usage: wasm-size-policy.mjs [--json] <kernel.wasm> | --policy');
    return 2;
  }
  let report;
  try {
    report = checkKernelWasmFile(args[0]);
  } catch (error) {
    console.error(`wasm-size-policy: ${error.message}`);
    return 2;
  }
  console.log(json ? JSON.stringify(report) : formatReport(report));
  return report.failures.length > 0 ? 1 : 0;
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  process.exitCode = main(process.argv.slice(2));
}
