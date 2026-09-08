import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const self = fileURLToPath(import.meta.url);
const timeout = 30_000;

export function validateCases(cases) {
  assert.ok(Array.isArray(cases) && cases.length > 0, 'empty workflow');
  const ids = new Set();
  for (const c of cases) {
    assert.deepEqual(Object.keys(c).sort(), ['data', 'expected_code', 'id', 'max_entities', 'max_input_bytes', 'schema_version']);
    assert.equal(c.schema_version, 1);
    assert.ok(typeof c.id === 'string' && c.id.length > 0 && !ids.has(c.id));
    ids.add(c.id);
    assert.equal(typeof c.data, 'string');
    for (const limit of [c.max_entities, c.max_input_bytes]) {
      assert.ok(Number.isSafeInteger(limit) && limit > 0);
    }
    assert.ok(['resource_limit_exceeded', 'invalid_argument'].includes(c.expected_code));
  }
}

export function score(c, observations) {
  const stages = [];
  for (const surface of ['native', 'wasm']) {
    const attempts = observations[surface];
    assert.equal(attempts.length, 2, 'two independent attempts required');
    const deterministic = attempts.every(a => a.outcome === attempts[0].outcome
      && a.code === attempts[0].code && a.unchanged === attempts[0].unchanged
      && a.diagnostic === attempts[0].diagnostic);
    const bounded = attempts.every(a => !['crash', 'hang_or_budget_overrun'].includes(a.outcome));
    const typed = attempts.every(a => a.id === c.id && a.outcome === 'typed_refusal'
      && a.code === c.expected_code && typeof a.diagnostic === 'string' && a.diagnostic.length > 0);
    const unchanged = attempts.every(a => a.unchanged === true
      && Number.isSafeInteger(a.snapshot_bytes) && a.snapshot_bytes > 0);
    for (const [stage, passed] of [['bounded_parser', bounded], ['typed_refusal', typed], ['session_unchanged', unchanged], ['repeat_agreement', deterministic]]) {
      stages.push({ workflow: 'W9', case: c.id, surface, stage, passed });
    }
  }
  stages.push({ workflow: 'W9', case: c.id, surface: 'native/wasm', stage: 'surface_agreement',
    passed: observations.native.every((a, i) => a.code === c.expected_code
      && a.code === observations.wasm[i].code && a.outcome === observations.wasm[i].outcome
      && a.unchanged === true && observations.wasm[i].unchanged === true) });
  return stages;
}

function wasmAttempt(c, modulePath) {
  const { BrepKernel } = createRequire(import.meta.url)(modulePath);
  const kernel = new BrepKernel();
  try {
    const empty = kernel.workflowStateSnapshot();
    const seed = JSON.parse(kernel.executeBatchV2(JSON.stringify([
      { op: 'makeBox', args: { width: 2, height: 3, depth: 4 } },
    ])));
    assert.ok(Object.hasOwn(seed[0], 'ok'), 'seed failed');
    assert.notEqual(empty, kernel.workflowStateSnapshot(), 'geometry mutation control');
    const exported = JSON.parse(kernel.executeBatchV2(JSON.stringify([
      { op: 'exportStep', args: { solid: seed[0].ok } },
    ])));
    assert.equal(typeof exported[0].ok, 'string');
    const imported = JSON.parse(kernel.executeBatchV2(JSON.stringify([
      { op: 'importStepBodies', args: { data: exported[0].ok } },
    ])));
    assert.equal(imported[0].ok?.solids?.length, 1, 'valid STEP import control');
    kernel.sketchNew();
    kernel.gcsNew();
    kernel.assemblyNew('W9 sentinel');
    kernel.checkpoint();
    const before = kernel.workflowStateSnapshot();
    const response = JSON.parse(kernel.executeBatchV2(JSON.stringify([
      { op: 'importStepBodies', args: { data: c.data, maxInputBytes: c.max_input_bytes, maxEntities: c.max_entities } },
    ])));
    assert.equal(response.length, 1);
    const error = response[0].error;
    return {
      id: c.id,
      outcome: error ? 'typed_refusal' : 'invalid_success',
      code: error?.code ?? 'unexpected_success',
      diagnostic: error?.message ?? '',
      unchanged: before === kernel.workflowStateSnapshot(),
      snapshot_bytes: Buffer.byteLength(before),
    };
  } finally {
    kernel.free();
  }
}

function attempt(command, args, c) {
  const result = spawnSync(command, args, { input: JSON.stringify(c), encoding: 'utf8', timeout, maxBuffer: 1024 * 1024 });
  if (result.error?.code === 'ETIMEDOUT') return { outcome: 'hang_or_budget_overrun' };
  if (result.error || result.status !== 0) return { outcome: 'crash', diagnostic: result.stderr || String(result.error) };
  try { return JSON.parse(result.stdout); } catch { return { outcome: 'untyped_error' }; }
}

if (process.argv[1] === self) {
  if (process.argv[2] === '--wasm-child') {
    const c = JSON.parse(readFileSync(0, 'utf8'));
    validateCases([c]);
    console.log(JSON.stringify(wasmAttempt(c, process.argv[3])));
  } else {
    const [native, wasm] = process.argv.slice(2);
    assert.ok(native && wasm, 'usage: node w9-preflight.mjs /absolute/w9-native /absolute/pkg/remus_wasm.js');
    const manifest = readFileSync(new URL('./w9-preflight.json', import.meta.url));
    const cases = JSON.parse(manifest);
    validateCases(cases);
    const observations = cases.map(c => ({
      case: c.id,
      native: Array.from({ length: 2 }, () => attempt(native, [], c)),
      wasm: Array.from({ length: 2 }, () => attempt(process.execPath, [self, '--wasm-child', wasm], c)),
    }));
    const stages = cases.flatMap((c, i) => score(c, observations[i]));
    const passed = stages.every(s => s.passed);
    console.log(JSON.stringify({
      schema_version: 1, scope: 'W9 STEP preallocation refusals; compatibility io build',
      manifest_sha256: createHash('sha256').update(manifest).digest('hex'),
      timeout_ms: timeout, repetitions: 2, passed, stages, observations,
    }, null, 2));
    process.exitCode = passed ? 0 : 1;
  }
}
