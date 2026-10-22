#!/usr/bin/env node
'use strict';
const fs = require('node:fs');
const assert = require('node:assert/strict');
const { checkRun } = require('./compare_runs.cjs');

// Kernel WASM hashes must differ across variants. Every measured input,
// replay policy and semantic output must still be present and exactly equal.
function compareVariants(before, after) {
  checkRun(before, 'completion'); checkRun(after, 'completion');
  assert.notEqual(before.metadata.package.wasmSha256, after.metadata.package.wasmSha256,
    'Expected different kernel WASM hashes for cross-variant comparison');
  for (const key of ['rebuildMode', 'pattern', 'ticks', 'warmup', 'harnessSha256',
    'recipeHarnessSha256', 'meshQualityHarnessSha256', 'prefixDocumentSha256',
    'fixture', 'policy', 'node', 'platform', 'architecture']) {
    assert.notEqual(before.metadata[key], undefined, `Missing baseline ${key}`);
    assert.deepEqual(before.metadata[key], after.metadata[key], `Changed ${key}`);
  }
  assert.equal(before.records.length, after.records.length);
  for (let i = 0; i < before.records.length; i++) {
    for (const key of ['index', 'width', 'cold', 'warmup', 'revisited', 'accepted',
      'validation', 'bbox', 'inventories', 'material', 'arenaSha256', 'meshSha256',
      'transferredBytes', 'checkpointCount', 'volumes', 'features', 'featureSummary']) {
      assert.notEqual(before.records[i][key], undefined, `Missing baseline ${key}`);
      assert.deepEqual(before.records[i][key], after.records[i][key], `Changed record ${i}: ${key}`);
    }
  }
  return { qualified: true, records: before.records.length,
    exactArenaMeshQuantitiesAndFeatures: true, timingsComparedAsSpeedup: false };
}
module.exports = { compareVariants };
if (require.main === module) {
  try {
    assert.equal(process.argv.length, 4, 'Expected before-completion.json after-completion.json');
    console.log(JSON.stringify(compareVariants(...process.argv.slice(2)
      .map(file => JSON.parse(fs.readFileSync(file)))), null, 2));
  } catch (error) { console.error(error.stack || error); process.exitCode = 1; }
}
