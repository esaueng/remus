import assert from 'node:assert/strict';
import test from 'node:test';

import { expandSplitMatrix, ORACLES, scoreSplitCase } from './split-io-matrix.mjs';
import {
  describeStaleness,
  requireObservations,
  validateInstalledEntry,
  validateProvenance,
} from './provenance.mjs';

function goodSuccess(id) {
  switch (id) {
    case 'split/step-import-box':
      return {
        outcome: 'success',
        diagnosticCodes: [],
        volume: ORACLES.boxVolume,
        faceCount: 6,
        shellCount: 1,
        validationErrors: 0,
        census: { plane: 6 },
        probes: { inside: 'inside', outside: 'outside' },
      };
    case 'split/box-hollow-roundtrip':
      return {
        outcome: 'success',
        diagnosticCodes: [],
        volumes: { box: 24, hollow: 488 },
        faceCounts: { box: 6, hollow: 12 },
        shellCounts: { box: 1, hollow: 2 },
        validationErrors: 0,
        census: { plane: 12 },
        probes: { wall: 'inside', cavity: 'outside', outside: 'outside' },
      };
    case 'split/periodic-seam':
      return {
        outcome: 'success',
        diagnosticCodes: [],
        volume: ORACLES.plateVolume,
        faceCount: 10,
        validationErrors: 0,
        census: { cylinder: 4, plane: 6 },
        pcurveCount: 48,
        stepPcurveCount: 48,
      };
    case 'split/two-session-isolation':
      return {
        outcome: 'success',
        diagnosticCodes: ['limit_exceeded'],
        volumes: { sessionA: 24, sessionB: 24 },
        noPartialSolids: true,
        crossSessionStaleRefused: true,
      };
    default:
      throw new Error(`no good observation for ${id}`);
  }
}

function goodRefusal() {
  return {
    outcome: 'batch_error',
    diagnosticCodes: ['limit_exceeded'],
    volume: 24,
    faceCount: 6,
    rollbackPreserved: true,
    preservedBytesEqual: true,
  };
}

test('split matrix covers the six required import/export cells', () => {
  const cases = expandSplitMatrix();
  assert.deepEqual(
    cases.map(({ id }) => id),
    [
      'split/step-import-box',
      'split/box-hollow-roundtrip',
      'split/periodic-seam',
      'split/malformed-refusal',
      'split/arena-truncation',
      'split/two-session-isolation',
    ],
  );
  // Independent oracles are pinned, not derived from either surface.
  assert.equal(ORACLES.boxVolume, 24);
  assert.equal(ORACLES.hollowVolume, 488);
  assert.equal(ORACLES.platePcurves, 48);
  const seam = cases.find(({ id }) => id === 'split/periodic-seam');
  assert.equal(seam.expect.pcurves, 48);
  assert.deepEqual(seam.expect.census, { cylinder: 4, plane: 6 });
  const roundtrip = cases.find(({ id }) => id === 'split/box-hollow-roundtrip');
  assert.deepEqual(roundtrip.expect.shellCounts, { box: 1, hollow: 2 });
  assert.deepEqual(roundtrip.expect.probes, { wall: 'inside', cavity: 'outside', outside: 'outside' });
});

test('scorer passes identical independent-oracle observations', () => {
  for (const c of expandSplitMatrix()) {
    const observation = c.expect.outcome === 'batch_error' ? goodRefusal() : goodSuccess(c.id);
    const stages = scoreSplitCase(c, { native: observation, fresh: structuredClone(observation) });
    const failed = stages.filter(({ required, passed }) => required && !passed);
    assert.deepEqual(failed, [], `${c.id} should score green`);
  }
});

test('scorer fails a volume that disagrees with the independent oracle', () => {
  const c = expandSplitMatrix().find(({ id }) => id === 'split/step-import-box');
  const good = goodSuccess(c.id);
  const wrong = structuredClone(good);
  wrong.volume *= 1.01;
  const stages = scoreSplitCase(c, { native: good, fresh: wrong });
  assert.equal(
    stages.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'oracle_volume')?.passed,
    false,
  );
  assert.equal(
    stages.find(({ invariant }) => invariant === 'volume_agreement')?.passed,
    false,
  );
});

test('wrong installed entry fails instead of loading foreign bytes', () => {
  assert.throws(
    () => validateInstalledEntry('/tmp/foreign/remus_wasm.js', '/consumer/node_modules/remus-wasm'),
    /resolved outside install/,
  );
  assert.throws(() => validateInstalledEntry(null, '/consumer/node_modules/remus-wasm'), /missing/);
});

test('mixed package pair is flagged stale, never silently attributed', () => {
  const fresh = {
    packageVersion: '2.130.51',
    wasmSha256: 'ab'.repeat(32),
  };
  const committed = {
    packageVersion: '2.130.27',
    wasmSha256: 'cd'.repeat(32),
  };
  const verdict = describeStaleness(fresh, committed);
  assert.equal(verdict.stale, true);
  assert.equal(verdict.versionMatch, false);
  const same = describeStaleness(fresh, structuredClone(fresh));
  assert.equal(same.stale, false);
});

test('missing cell observations are refused, never vacuously green', () => {
  assert.throws(
    () => requireObservations('split/step-import-box', { native: { outcome: 'success' }, fresh: null }),
    /missing/,
  );
  assert.throws(
    () => requireObservations('split/step-import-box', { native: { volume: 24 } }),
    /no outcome/,
  );
});

test('stale or missing provenance identity fails', () => {
  assert.throws(() => validateProvenance(null), /missing/);
  assert.throws(
    () => validateProvenance({ mode: 'fresh-tarball', sourceRevision: 'abc' }),
    /missing required field/,
  );
  assert.throws(
    () => validateProvenance({
      mode: 'fresh-tarball',
      sourceRevision: 'abc',
      sourceDirty: false,
      toolchain: {},
      buildOptions: {},
      packageVersion: '1.0.0',
      tarballSha256: 'ab'.repeat(32),
      wasmSha256: 'not-a-hash',
      installedEntry: '/x',
    }),
    /SHA-256/,
  );
});

test('fabricated successful malformed import fails the typed-refusal gate', () => {
  const c = expandSplitMatrix().find(({ id }) => id === 'split/malformed-refusal');
  const fabricated = {
    outcome: 'success',
    diagnosticCodes: [],
    volume: 24,
    faceCount: 6,
    rollbackPreserved: true,
    preservedBytesEqual: true,
  };
  const stages = scoreSplitCase(c, { native: goodRefusal(), fresh: fabricated });
  assert.equal(
    stages.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'typed_refusal')?.passed,
    false,
  );
  assert.equal(
    stages.find(({ invariant }) => invariant === 'outcome_agreement')?.passed,
    false,
  );
});

test('mutated pre-existing session claimed preserved fails', () => {
  const c = expandSplitMatrix().find(({ id }) => id === 'split/arena-truncation');
  const mutated = { ...goodRefusal(), volume: 25, rollbackPreserved: false };
  const stages = scoreSplitCase(c, { native: goodRefusal(), fresh: mutated });
  assert.equal(
    stages.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'rollback_preserved')?.passed,
    false,
  );
  assert.equal(
    stages.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'oracle_volume')?.passed,
    false,
  );
});

test('census agreement is key-order and null-insensitive', () => {
  const c = expandSplitMatrix().find(({ id }) => id === 'split/periodic-seam');
  const a = goodSuccess(c.id);
  const b = structuredClone(a);
  b.census = { plane: 6, cylinder: 4 };
  const stages = scoreSplitCase(c, { native: a, fresh: b });
  assert.equal(
    stages.find(({ invariant }) => invariant === 'census_agreement')?.passed,
    true,
  );
  const iso = expandSplitMatrix().find(({ id }) => id === 'split/two-session-isolation');
  const ia = goodSuccess(iso.id);
  const ib = structuredClone(ia);
  delete ib.census;
  const stages2 = scoreSplitCase(iso, { native: ia, fresh: ib });
  assert.deepEqual(
    stages2.filter(({ required, passed }) => required && !passed),
    [],
  );
});

test('seam cell requires both per-use pcurve branches, not volume alone', () => {
  const c = expandSplitMatrix().find(({ id }) => id === 'split/periodic-seam');
  const good = goodSuccess(c.id);
  const noSeam = structuredClone(good);
  noSeam.stepPcurveCount = 0;
  noSeam.pcurveCount = 0;
  const stages = scoreSplitCase(c, { native: good, fresh: noSeam });
  assert.equal(
    stages.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'seam_pcurves')?.passed,
    false,
  );
  // Volume alone still passes; the seam gate carries the signal.
  assert.equal(
    stages.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'oracle_volume')?.passed,
    true,
  );
});
