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
const pair = () => [structuredClone(source), structuredClone(source)];

test('different binaries qualify only with complete exact workflow outputs', () => {
  const [before, after] = pair();
  after.metadata.package.wasmSha256 = 'a'.repeat(64);
  assert.equal(compareVariants(before, after).records, 10);
});
test('changed geometry, quantity, feature and replay policy are rejected', () => {
  for (const change of [
    run => { run.records[1].arenaSha256 = 'a'.repeat(64); },
    run => { run.records[1].volumes[0] += 1e-9; },
    run => { run.records[1].features[0] = []; },
    run => { run.metadata.policy.linear *= 2; },
  ]) {
    const [before, after] = pair(); change(after);
    assert.throws(() => compareVariants(before, after));
  }
});
test('missing evidence on both sides never qualifies', () => {
  const [before, after] = pair();
  delete before.records[0].features; delete after.records[0].features;
  assert.throws(() => compareVariants(before, after));
});
