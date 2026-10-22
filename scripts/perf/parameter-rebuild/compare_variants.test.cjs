'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const { compareVariants } = require('./compare_variants.cjs');
const runs = JSON.parse(fs.readFileSync(path.join(__dirname,
  '../../../docs/performance/evidence/parameter-rebuild-2026-10-20/runs.json')));
const source = runs.find(run => run.metadata.mode === 'completion' &&
  run.metadata.rebuildMode === 'full' && run.metadata.pattern === 'unique');
const pair = () => {
  const before = structuredClone(source), after = structuredClone(source);
  after.metadata.package.wasmSha256 = 'a'.repeat(64);
  return [before, after];
};

test('different binaries qualify with unchanged wrapper and package metadata', () => {
  const [before, after] = pair();
  assert.equal(before.metadata.package.wrapperSha256, after.metadata.package.wrapperSha256);
  assert.equal(before.metadata.package.packageJsonSha256, after.metadata.package.packageJsonSha256);
  assert.equal(compareVariants(before, after).records, 10);
});
test('identical captures and cloned same-package captures are rejected', () => {
  assert.throws(() => compareVariants(source, source), /different kernel WASM/);
  assert.throws(() => compareVariants(structuredClone(source), structuredClone(source)),
    /different kernel WASM/);
});
test('metadata-only differences do not establish a different kernel', () => {
  for (const change of [
    metadata => { metadata.wrapperSha256 = 'a'.repeat(64); },
    metadata => { metadata.packageJsonSha256 = 'a'.repeat(64); },
    metadata => { metadata.suppliedSourceSha = 'b'.repeat(40); },
  ]) {
    const before = structuredClone(source), after = structuredClone(source);
    change(after.metadata.package);
    assert.throws(() => compareVariants(before, after), /different kernel WASM/);
  }
});
test('changed geometry, quantity, feature and replay policy are rejected', () => {
  for (const [key, change] of [
    ['arenaSha256', run => { run.records[1].arenaSha256 = 'a'.repeat(64); }],
    ['volumes', run => { run.records[1].volumes[0] += 1e-9; }],
    ['features', run => { run.records[1].features[0] = []; }],
    ['policy', run => { run.metadata.policy.linear *= 2; }],
  ]) {
    const [before, after] = pair(); change(after);
    assert.throws(() => compareVariants(before, after), new RegExp(key));
  }
});
test('missing evidence on both sides never qualifies', () => {
  const [before, after] = pair();
  delete before.records[0].features; delete after.records[0].features;
  assert.throws(() => compareVariants(before, after));
});
