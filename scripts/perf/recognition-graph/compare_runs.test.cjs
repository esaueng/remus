'use strict';
const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { test } = require('node:test');
const { compareRuns, compareReplays } = require('./compare_runs.cjs');

function pair(t) {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), 'remus-recognition-compare-'));
  t.after(() => fs.rmSync(root, { recursive: true, force: true }));
  const directories = ['baseline', 'candidate'].map(name => path.join(root, name));
  for (const directory of directories) {
    fs.mkdirSync(directory);
    const artifacts = {};
    for (const name of ['positions', 'normals', 'indices', 'faceOffsets', 'arena']) {
      const bytes = Buffer.from(name);
      fs.writeFileSync(path.join(directory, name), bytes);
      artifacts[name] = { file: name, sha256: crypto.createHash('sha256').update(bytes).digest('hex') };
    }
    const snapshot = { faces: 1, surfaceCounts: { plane: 1 }, analyticRadii: [],
      planesAtExpectedX: [], bbox: [0, 0, 0, 1, 1, 1], volume: 1,
      mesh: { watertight: true }, volumeBits: '000000000000f03f',
      features: [{ face: 7, value: 1 }], featuresJsonSha256: 'feature-digest',
      arenaSha256: artifacts.arena.sha256, artifacts, selectedFace: { index: 0 } };
    const record = { ...snapshot, iteration: 0, warmup: false, distance: 5, valid: true };
    fs.writeFileSync(path.join(directory, 'results.json'), JSON.stringify({
      workflow: 'geometry-first', warmup: 0, samples: 1, source: snapshot, records: [record],
      metadata: { policy: { measureDeflection: 0.08 }, fixture: { sha256: 'fixture' },
        kernel: { wasmSha256: path.basename(directory) } },
    }));
  }
  return directories;
}

function mutate(directory, action) {
  const filename = path.join(directory, 'results.json');
  const value = JSON.parse(fs.readFileSync(filename, 'utf8'));
  action(value);
  fs.writeFileSync(filename, JSON.stringify(value));
}

test('compares every source/edit artifact and complete feature records', t => {
  const [before, after] = pair(t);
  const result = compareRuns(before, after);
  assert.equal(result.snapshotPairs, 2);
  assert.equal(result.byteExactArtifactPairs, 10);
  assert.equal(result.retainedPairs, 1);
});

test('rejects changed bytes even when the stored digest is updated', t => {
  const [before, after] = pair(t);
  const bytes = Buffer.from('changed positions');
  fs.writeFileSync(path.join(after, 'positions'), bytes);
  mutate(after, value => {
    const digest = crypto.createHash('sha256').update(bytes).digest('hex');
    value.source.artifacts.positions.sha256 = digest;
    value.records[0].artifacts.positions.sha256 = digest;
  });
  assert.throws(() => compareRuns(before, after), /positions bytes differ/);
});

test('rejects stale digest claims and incomplete runs', t => {
  const [before, after] = pair(t);
  fs.writeFileSync(path.join(after, 'normals'), 'corrupt');
  assert.throws(() => compareRuns(before, after), /SHA256 does not match/);
  mutate(after, value => { value.records = []; });
  assert.throws(() => compareRuns(before, after), /Incomplete run/);
});

test('rejects feature, quantity-bit, and cache-policy changes', t => {
  for (const [change, message] of [
    [value => { value.records[0].features[0].face = 8; }, /features/],
    [value => { value.records[0].volumeBits = 'different'; }, /volumeBits/],
    [value => { value.metadata.policy.measureDeflection = 0.1; }, /Query\/cache policy/],
  ]) {
    const [before, after] = pair(t);
    mutate(after, change);
    assert.throws(() => compareRuns(before, after), message);
  }
});

test('fails closed when both runs omit a mandatory semantic field', t => {
  const [before, after] = pair(t);
  for (const directory of [before, after]) {
    mutate(directory, value => { delete value.records[0].bbox; });
  }
  assert.throws(() => compareRuns(before, after), /missing mandatory bbox/);
});

test('rejects source selection changes and unvalidated edits', t => {
  for (const [change, message] of [
    [value => { value.source.selectedFace.index = 1; }, /Selected source face/],
    [value => { delete value.records[0].valid; }, /edit must validate/],
  ]) {
    const [before, after] = pair(t);
    mutate(after, change);
    assert.throws(() => compareRuns(before, after), message);
  }
});

test('replay comparisons include all per-face area bits and edge relations', t => {
  const [before, after] = pair(t);
  const files = [before, after].map(directory => path.join(directory, 'replay.json'));
  const replay = { diagnosticOnly: true, timingClaimsExcluded: true,
    rows: [{ snapshot: 'source', inputArenaSha256: 'arena', outputArenaSha256: 'arena',
      areaBits: '000000000000f03f', volumeBits: '000000000000f03f', bboxBits: [],
      features: [], featuresJsonSha256: 'features',
      faceRecords: [{ face: 0, carrier: { type: 'plane' }, areaBits: '000000000000f03f' }],
      edgeRelations: [{ edge: 1, type: 'convex' }] }] };
  for (const file of files) fs.writeFileSync(file, JSON.stringify(replay));
  assert.deepEqual(compareReplays(...files), { replayPairs: 1, perFaceAreaPairs: 1 });
  replay.rows[0].edgeRelations[0].type = 'concave';
  fs.writeFileSync(files[1], JSON.stringify(replay));
  assert.throws(() => compareReplays(...files), /Complete replay/);
  replay.rows[0].edgeRelations[0].type = 'convex';
  replay.rows[0].faceRecords[0].areaBits = 'changed';
  fs.writeFileSync(files[1], JSON.stringify(replay));
  assert.throws(() => compareReplays(...files), /Complete replay/);
  for (const file of files) {
    const invalid = JSON.parse(fs.readFileSync(file, 'utf8'));
    delete invalid.rows[0].edgeRelations;
    fs.writeFileSync(file, JSON.stringify(invalid));
  }
  assert.throws(() => compareReplays(...files), /Missing replay edgeRelations/);
});
