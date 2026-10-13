import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { mkdtempSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { test } from 'node:test';
import { fileURLToPath } from 'node:url';
import { brotliCompressSync, constants as zlibConstants, gzipSync } from 'node:zlib';
import {
  KERNEL_WASM_POLICY as P,
  checkKernelWasmFile,
  evaluateKernelWasm,
  measureKernelWasm,
} from './wasm-size-policy.mjs';

const SCRIPT = fileURLToPath(new URL('./wasm-size-policy.mjs', import.meta.url));
const MIB = 1024 * 1024;
// Comfortably inside every limit, so each test moves one metric only.
const inside = { rawBytes: 1_000_000, gzipBytes: 400_000, brotliBytes: 300_000 };
const codes = (findings) => findings.map((finding) => finding.code);

function module(bytes) {
  const contents = new Uint8Array(bytes);
  contents.set([0x00, 0x61, 0x73, 0x6d, 0x01, 0x00, 0x00, 0x00]);
  for (let i = 8; i < bytes; i++) contents[i] = (i * 2654435761) >>> 24;
  return contents;
}

function tempFile(t, name, contents) {
  const dir = mkdtempSync(join(tmpdir(), 'remus-size-policy-'));
  t.after(() => rmSync(dir, { recursive: true, force: true }));
  const file = join(dir, name);
  writeFileSync(file, contents);
  return file;
}

test('limits mirror the consumer kernel policy', () => {
  assert.deepEqual(P, {
    rawReviewBytes: 9 * MIB,
    rawHardBytes: 10 * MIB,
    gzipHardBytes: 3.5 * MIB,
    brotliReviewBytes: 2.5 * MIB,
    brotliQuality: 11,
  });
  assert(Object.isFrozen(P));
});

test('a module inside every limit has no findings', () => {
  assert.deepEqual(evaluateKernelWasm(inside), { warnings: [], failures: [] });
});

test('raw review threshold is inclusive, like the consumer', () => {
  const at = (rawBytes) => evaluateKernelWasm({ ...inside, rawBytes });
  assert.deepEqual(codes(at(P.rawReviewBytes - 1).warnings), []);
  assert.deepEqual(codes(at(P.rawReviewBytes).warnings), ['kernel-raw-review']);
  assert.deepEqual(at(P.rawReviewBytes).failures, []);
});

test('raw hard limit fails only above its exact byte count', () => {
  const at = (rawBytes) => evaluateKernelWasm({ ...inside, rawBytes });
  assert.deepEqual(codes(at(P.rawHardBytes).failures), []);
  assert.deepEqual(codes(at(P.rawHardBytes).warnings), ['kernel-raw-review']);
  const over = at(P.rawHardBytes + 1);
  assert.deepEqual(codes(over.failures), ['kernel-raw-hard']);
  assert.equal(over.failures[0].bytes, P.rawHardBytes + 1);
  assert.equal(over.failures[0].budgetBytes, P.rawHardBytes);
});

test('gzip hard limit fails only above its exact byte count', () => {
  const at = (gzipBytes) => evaluateKernelWasm({ ...inside, gzipBytes });
  assert.deepEqual(at(P.gzipHardBytes), { warnings: [], failures: [] });
  const over = at(P.gzipHardBytes + 1);
  assert.deepEqual(codes(over.failures), ['kernel-gzip-hard']);
  assert.deepEqual(over.warnings, []);
});

test('Brotli review warns only above its exact byte count and never fails', () => {
  const at = (brotliBytes) => evaluateKernelWasm({ ...inside, brotliBytes });
  assert.deepEqual(at(P.brotliReviewBytes), { warnings: [], failures: [] });
  const over = at(P.brotliReviewBytes + 1);
  assert.deepEqual(codes(over.warnings), ['kernel-brotli-review']);
  assert.deepEqual(over.failures, []);
});

test('measurement uses Node gzip defaults and Brotli quality 11', () => {
  const contents = module(64 * 1024);
  assert.deepEqual(measureKernelWasm(contents), {
    rawBytes: contents.byteLength,
    gzipBytes: gzipSync(contents).byteLength,
    brotliBytes: brotliCompressSync(contents, {
      params: { [zlibConstants.BROTLI_PARAM_QUALITY]: 11 },
    }).byteLength,
  });
});

test('measurement refuses input that is not a version 1 module', () => {
  assert.throws(() => measureKernelWasm(Uint8Array.from([0x00, 0x61, 0x73])), /not a version 1/);
  const wrongVersion = module(16);
  wrongVersion[4] = 2;
  assert.throws(() => measureKernelWasm(wrongVersion), /not a version 1/);
});

test('file report carries metrics, policy and findings', (t) => {
  const file = tempFile(t, 'kernel.wasm', module(4096));
  const report = checkKernelWasmFile(file);
  assert.equal(report.metrics.rawBytes, 4096);
  assert.deepEqual(report.policy, P);
  assert.deepEqual(report.warnings, []);
  assert.deepEqual(report.failures, []);
});

test('command line: JSON report, policy and exit status', (t) => {
  const run = (...args) => spawnSync(process.execPath, [SCRIPT, ...args], { encoding: 'utf8' });
  const ok = run('--json', tempFile(t, 'kernel.wasm', module(4096)));
  assert.equal(ok.status, 0, ok.stderr);
  assert.equal(JSON.parse(ok.stdout).metrics.rawBytes, 4096);
  const text = run(tempFile(t, 'kernel.wasm', module(4096)));
  assert.equal(text.status, 0, text.stderr);
  assert.match(text.stdout, /raw +4,096 B/);
  const policy = run('--policy');
  assert.equal(policy.status, 0);
  assert.deepEqual(JSON.parse(policy.stdout), P);
  const bad = run('--json', tempFile(t, 'not.wasm', 'not wasm'));
  assert.equal(bad.status, 2);
  assert.match(bad.stderr, /not a version 1/);
  assert.equal(run().status, 2);
});

test('command line exits 1 above a hard limit', (t) => {
  // Zero padding just over the raw hard limit: compresses to almost nothing,
  // so the raw failure is the only finding and the test stays fast.
  const contents = new Uint8Array(P.rawHardBytes + 1);
  contents.set([0x00, 0x61, 0x73, 0x6d, 0x01, 0x00, 0x00, 0x00]);
  const result = spawnSync(process.execPath, [SCRIPT, '--json', tempFile(t, 'big.wasm', contents)], {
    encoding: 'utf8',
  });
  assert.equal(result.status, 1, result.stderr);
  const report = JSON.parse(result.stdout);
  assert.deepEqual(codes(report.failures), ['kernel-raw-hard']);
  assert.deepEqual(codes(report.warnings), ['kernel-raw-review']);
});
