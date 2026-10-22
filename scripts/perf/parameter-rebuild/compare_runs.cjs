#!/usr/bin/env node
'use strict';
const fs = require('node:fs');
const assert = require('node:assert/strict');
const digest = value => typeof value === 'string' && /^[0-9a-f]{64}$/.test(value);

function checkRun(run, mode) {
  assert.equal(run.metadata.mode, mode);
  assert.ok(['full', 'suffix'].includes(run.metadata.rebuildMode), 'Missing replay scope');
  for (const key of ['harnessSha256', 'recipeHarnessSha256', 'meshQualityHarnessSha256',
    'prefixDocumentSha256']) assert.ok(digest(run.metadata[key]), `Missing ${key}`);
  for (const key of ['wasmSha256', 'wrapperSha256', 'packageJsonSha256'])
    assert.ok(digest(run.metadata.package[key]), `Missing package ${key}`);
  assert.ok(digest(run.metadata.fixture.recipeSha256), 'Missing canonical recipe identity');
  assert.ok(Array.isArray(run.records) && run.records.length > 0);
  assert.equal(run.records.length, run.metadata.ticks);
  for (const [index, row] of run.records.entries()) {
    assert.equal(row.index, index); assert.ok(Number.isFinite(row.width));
    assert.equal(row.checkpointCount, 1, 'Unexpected retained checkpoints');
    for (const key of ['cold', 'warmup', 'revisited']) assert.equal(typeof row[key], 'boolean');
    assert.equal(row.accepted, true); assert.ok(digest(row.arenaSha256), 'Missing exact arena digest');
    assert.deepEqual(row.validation, Array(8).fill(0));
    assert.ok(Array.isArray(row.bbox) && row.bbox.length === 6 && row.bbox.every(Number.isFinite));
    assert.ok(Array.isArray(row.inventories) && row.inventories.length === 8);
    assert.ok(Array.isArray(row.material) && row.material.length === 6 &&
      row.material.every(p => ['inside', 'outside'].includes(p.expected) && p.actual === p.expected));
    assert.ok(Array.isArray(row.meshSha256) && row.meshSha256.length === 8);
    for (const mesh of row.meshSha256) for (const key of ['positions', 'indices', 'normals', 'faceOffsets'])
      assert.ok(digest(mesh[key]), `Missing ${key} digest`);
    assert.ok(Number.isSafeInteger(row.transferredBytes) && row.transferredBytes > 0);
    if (mode === 'completion') {
      assert.ok(Array.isArray(row.volumes) && row.volumes.length === 8 &&
        row.volumes.every(v => Number.isFinite(v) && v > 0));
      assert.ok(Array.isArray(row.features) && row.features.length === 8 && row.features.every(Array.isArray));
    } else { assert.equal(row.volumes, null); assert.equal(row.features, null); }
  }
}

// Compare the entire reachable arena digest, not sampled positions or a
// surface census alone: carriers, trims, per-use boundary authority and all
// other serialized fields must agree between the two independent processes.
function compare(preview, completion) {
  checkRun(preview, 'preview'); checkRun(completion, 'completion');
  for (const key of ['rebuildMode', 'pattern', 'ticks', 'warmup', 'harnessSha256', 'recipeHarnessSha256',
    'meshQualityHarnessSha256']) assert.deepEqual(preview.metadata[key], completion.metadata[key]);
  assert.deepEqual(preview.metadata.package, completion.metadata.package);
  assert.deepEqual(preview.metadata.fixture, completion.metadata.fixture);
  assert.equal(preview.metadata.prefixDocumentSha256, completion.metadata.prefixDocumentSha256);
  assert.equal(preview.records.length, completion.records.length);
  for (let i = 0; i < preview.records.length; i++) {
    const a = preview.records[i], b = completion.records[i];
    assert.equal(a.accepted, true); assert.equal(b.accepted, true);
    for (const key of ['index', 'width', 'cold', 'warmup', 'revisited', 'validation',
      'bbox', 'inventories', 'material', 'arenaSha256', 'meshSha256', 'transferredBytes', 'checkpointCount']) {
      assert.deepEqual(a[key], b[key], `Output mismatch at record ${i}: ${key}`);
    }
    assert.equal(a.volumes, null); assert.equal(a.features, null);
    assert.equal(b.volumes.length, 8); assert.equal(b.features.length, 8);
    assert.ok(b.volumes.every(v => Number.isFinite(v) && v > 0));
  }
  return { qualified: true, records: preview.records.length, rebuildMode: preview.metadata.rebuildMode,
    pattern: preview.metadata.pattern,
    arenaAndMeshBytesMatch: true, timingsComparedAsSpeedup: false };
}
module.exports = { compare };
if (require.main === module) {
  try {
    assert.equal(process.argv.length, 4, 'Expected preview.json completion.json');
    console.log(JSON.stringify(compare(...process.argv.slice(2).map(p => JSON.parse(fs.readFileSync(p)))), null, 2));
  } catch (e) { console.error(e.stack || e); process.exitCode = 1; }
}
