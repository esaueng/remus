#!/usr/bin/env node
'use strict';

// Compare complete saved outputs, never rounded timings or digest claims alone.
const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');

const sha256 = bytes => crypto.createHash('sha256').update(bytes).digest('hex');
const semanticKeys = ['faces', 'surfaceCounts', 'analyticRadii', 'planesAtExpectedX',
  'bbox', 'volume', 'volumeBits', 'features', 'featuresJsonSha256', 'mesh', 'arenaSha256'];
const artifactKeys = ['positions', 'normals', 'indices', 'faceOffsets', 'arena'];

function load(directory) {
  return JSON.parse(fs.readFileSync(path.join(directory, 'results.json'), 'utf8'));
}

function artifact(directory, item, context) {
  assert.equal(typeof item.file, 'string', `${context}: missing filename`);
  assert.equal(path.basename(item.file), item.file, `${context}: artifact must be local to run`);
  const bytes = fs.readFileSync(path.join(directory, item.file));
  assert.equal(sha256(bytes), item.sha256, `${context}: stored SHA256 does not match bytes`);
  return bytes;
}

function snapshots(run) {
  assert.equal(run.records.length, run.warmup + run.samples, 'Incomplete run');
  assert.equal(Object.hasOwn(run.source, 'selectedFace'), true, 'Missing selected source face');
  const rows = [run.source, ...run.records];
  for (const [index, row] of rows.entries()) {
    for (const key of semanticKeys) {
      assert.equal(Object.hasOwn(row, key), true, `Snapshot ${index}: missing mandatory ${key}`);
    }
    if (index > 0) assert.equal(row.valid, true, `Snapshot ${index}: edit must validate`);
  }
  return rows;
}

function packageIdentity({ directory: omitted, ...identity }) { return identity; }

function compareReplays(baselineFile, candidateFile) {
  const before = JSON.parse(fs.readFileSync(baselineFile, 'utf8'));
  const after = JSON.parse(fs.readFileSync(candidateFile, 'utf8'));
  for (const replay of [before, after]) {
    assert.equal(replay.diagnosticOnly, true);
    assert.equal(replay.timingClaimsExcluded, true);
    assert.ok(Array.isArray(replay.rows) && replay.rows.length > 0, 'Missing replay rows');
    for (const row of replay.rows) {
      for (const key of ['snapshot', 'inputArenaSha256', 'faceRecords', 'areaBits', 'volumeBits',
        'bboxBits', 'features', 'featuresJsonSha256', 'edgeRelations', 'outputArenaSha256']) {
        assert.equal(Object.hasOwn(row, key), true, `Missing replay ${key}`);
      }
      assert.equal(row.outputArenaSha256, row.inputArenaSha256, 'Replay modified the input arena');
      assert.ok(Array.isArray(row.faceRecords) && row.faceRecords.length > 0);
      for (const face of row.faceRecords) {
        for (const key of ['face', 'carrier', 'areaBits']) {
          assert.equal(Object.hasOwn(face, key), true, `Missing per-face ${key}`);
        }
      }
    }
  }
  assert.deepEqual(after, before, 'Complete replay: per-face area bits, quantities, features and edge relations');
  return { replayPairs: before.rows.length,
    perFaceAreaPairs: before.rows.reduce((sum, row) => sum + row.faceRecords.length, 0) };
}

function compareRuns(baselineDirectory, candidateDirectory) {
  const baseline = load(baselineDirectory), candidate = load(candidateDirectory);
  for (const key of ['workflow', 'warmup', 'samples']) {
    assert.deepEqual(candidate[key], baseline[key], `Run ${key}`);
  }
  assert.deepEqual(candidate.metadata.policy, baseline.metadata.policy, 'Query/cache policy');
  assert.equal(candidate.metadata.fixture.sha256, baseline.metadata.fixture.sha256, 'Fixture SHA256');
  const left = snapshots(baseline), right = snapshots(candidate);
  assert.equal(left.length, right.length);
  assert.deepEqual(candidate.source.selectedFace, baseline.source.selectedFace, 'Selected source face');
  const pairs = [];
  for (let index = 0; index < left.length; index++) {
    const before = left[index], after = right[index];
    for (const key of ['iteration', 'warmup', 'distance', 'valid', ...semanticKeys]) {
      assert.deepEqual(after[key], before[key], `Snapshot ${index}: ${key}`);
    }
    assert.equal(before.mesh.watertight, true, `Snapshot ${index}: baseline closure`);
    const artifacts = {};
    for (const key of artifactKeys) {
      const a = artifact(baselineDirectory, before.artifacts[key], `baseline ${index} ${key}`);
      const b = artifact(candidateDirectory, after.artifacts[key], `candidate ${index} ${key}`);
      assert.equal(a.equals(b), true, `Snapshot ${index}: ${key} bytes differ`);
      artifacts[key] = { bytes: a.length, sha256: sha256(a) };
    }
    pairs.push({ snapshot: index === 0 ? 'source' : `iteration-${before.iteration}`,
      warmup: index === 0 ? null : before.warmup, volumeBits: before.volumeBits,
      featuresJsonSha256: before.featuresJsonSha256, artifacts });
  }
  return { schemaVersion: 1, allEquivalent: true, snapshotPairs: pairs.length,
    sourcePairs: 1, warmupPairs: baseline.warmup, retainedPairs: baseline.samples,
    byteExactArtifactPairs: pairs.length * artifactKeys.length,
    workflow: baseline.workflow,
    candidateKernel: packageIdentity(candidate.metadata.kernel), pairs,
    baselineKernel: packageIdentity(baseline.metadata.kernel) };
}

module.exports = { compareRuns, compareReplays };
if (require.main === module) {
  try {
    assert.ok([4, 6].includes(process.argv.length),
      'Usage: compare_runs.cjs BASELINE_DIR CANDIDATE_DIR [BASELINE_REPLAY CANDIDATE_REPLAY]');
    const result = compareRuns(process.argv[2], process.argv[3]);
    if (process.argv.length === 6) result.replay = compareReplays(process.argv[4], process.argv[5]);
    process.stdout.write(JSON.stringify(result, null, 2) + '\n');
  } catch (error) {
    process.stderr.write(`${error.stack || error}\n`);
    process.exitCode = 1;
  }
}
