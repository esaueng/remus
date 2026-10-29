'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const test = require('node:test');
const profiler = require('./profile_holder.cjs');

test('measured timing is refused before package loading or output creation', async () => {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'holder-profile-options-'));
  const output = path.join(directory, 'must-not-exist');
  await assert.rejects(profiler.main(['--kernel-pkg', path.join(directory, 'absent'), '--out', output,
    '--timing-status', 'measured']), /diagnostic-only/);
  assert.equal(fs.existsSync(output), false);
});

test('unknown arguments, invalid source labels and sample intervals fail closed', () => {
  assert.throws(() => profiler.options(['--unknown', 'value']), /Unknown/);
  assert.throws(() => profiler.options(['--kernel-pkg', '.', '--out', 'unused',
    '--package-source-sha', 'main']), /source SHA/);
  assert.throws(() => profiler.options(['--kernel-pkg', '.', '--out', 'unused',
    '--sample-interval-us', '0']), /sample interval/);
});

test('observer separates retained geometry, nested calls and untimed qualification', () => {
  let clock = 0;
  class Kernel {
    restore(value) { return value; }
    validateSolid(value) { return value; }
    internal() { return this.validateSolid(4); }
    recognizeFeatures() { throw Error('original refusal'); }
  }
  const recipe = {
    prepare(kernel) { assert.equal(this, recipe); return kernel.internal(); },
    rebuild(kernel) { return kernel.validateSolid(5); },
    qualify(kernel) { return kernel.validateSolid(6); },
    extract(value) { return value; }
  };
  const original = Kernel.prototype.validateSolid, prepare = recipe.prepare;
  const observer = profiler.observe(recipe, Kernel, () => ++clock);
  const kernel = new Kernel();
  assert.equal(kernel.restore(123), 123);
  assert.equal(recipe.prepare(kernel), 4);
  assert.equal(recipe.rebuild(kernel), 5);
  assert.equal(recipe.qualify(kernel), 6);
  assert.throws(() => kernel.recognizeFeatures(), /original refusal/);
  assert.equal(observer.calls.find(v => v.method === 'restore').tick, 0);
  const nested = observer.calls.find(v => v.method === 'validateSolid' && v.firstArgument === 4);
  assert.equal(nested.phase, 'prefix'); assert.equal(nested.nested, true);
  assert.equal(observer.calls.find(v => v.firstArgument === 5).phase, 'rebuild');
  assert.equal(observer.calls.find(v => v.firstArgument === 6).phase, 'qualification');
  const refused = observer.calls.find(v => v.method === 'recognizeFeatures');
  assert.equal(refused.status, 'threw'); assert.equal(refused.phase, 'recognize');
  assert(observer.calls.every(v => v.endUs >= v.startUs));
  observer.restore();
  assert.equal(Kernel.prototype.validateSolid, original); assert.equal(recipe.prepare, prepare);
});

test('package mutation is detected by bytes even when file names and sizes agree', () => {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'holder-profile-inventory-'));
  for (const name of ['package.json', 'remus_wasm_bg.wasm', 'remus_wasm_node.cjs'])
    fs.writeFileSync(path.join(directory, name), 'aaaa');
  const before = profiler.inventory(directory);
  fs.writeFileSync(path.join(directory, 'remus_wasm_bg.wasm'), 'bbbb');
  assert.throws(() => profiler.unchanged(before, profiler.inventory(directory), 'Package'), /changed/);
});

test('failed package initialization preserves a failure receipt and refuses directory reuse', async () => {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'holder-profile-failure-'));
  const output = path.join(directory, 'capture');
  await assert.rejects(profiler.main(['--kernel-pkg', path.join(directory, 'absent'), '--out', output]));
  const capture = JSON.parse(fs.readFileSync(path.join(output, 'capture.json')));
  assert.equal(capture.status, 'failed'); assert.equal(capture.diagnosticOnly, true);
  assert(capture.artifacts['initial-receipt.json']);
  assert.equal(fs.existsSync(path.join(output, 'run.json')), false);
  const bytes = fs.readFileSync(path.join(output, 'capture.json'));
  await assert.rejects(profiler.main(['--kernel-pkg', directory, '--out', output]), /EEXIST/);
  assert.deepEqual(fs.readFileSync(path.join(output, 'capture.json')), bytes);
});
