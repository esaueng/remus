import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { mkdtempSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { resolve } from 'node:path';
import test from 'node:test';
import { fileURLToPath } from 'node:url';

import { expandContractMatrix, scoreContractCase } from './contract-matrix.mjs';
import { expandSplitMatrix, scoreSplitCase } from './split-io-matrix.mjs';
import { validateProofReport, validateProofReports } from './proof-verdict.mjs';

function provenance(mode) {
  return {
    mode,
    sourceRevision: 'ab'.repeat(20),
    sourceDirty: false,
    toolchain: { rustc: 'rustc 1.96', cargo: 'cargo 1.96', node: 'v24', wasmPack: 'wasm-pack 0.15' },
    target: mode === 'native' ? 'host' : mode === 'fresh-tarball' ? 'nodejs' : 'wasm32-unknown-unknown',
    buildOptions: mode === 'native'
      ? { profile: 'release', binary: 'remus-parity-native', facade: 'remus::Model' }
      : mode === 'fresh-tarball'
        ? { target: 'nodejs', profile: 'release', features: '--no-default-features' }
        : { origin: 'committed bytes; build options unknown' },
    packageVersion: mode === 'native' ? null : '2026.1.0',
    wasmSha256: mode === 'native' ? null : 'cd'.repeat(32),
    tarballSha256: mode === 'fresh-tarball' ? 'ef'.repeat(32) : null,
    installedEntry: '/tmp/consumer/node_modules/remus-wasm/remus_wasm.js',
    sourceRelationship: mode === 'committed-package' ? 'unverified — distributed bytes' : 'built from recorded source',
  };
}

function contractObservation(c, surface) {
  const expected = c.expect;
  if (expected.outcome === 'batch_error') {
    return {
      outcome: 'batch_error', diagnosticCodes: [expected.code],
      rollbackVolumes: expected.rollbackVolume === undefined ? [] : [expected.rollbackVolume],
      rollbackFaces: expected.rollbackFaces ?? null,
    };
  }
  const observation = {
    outcome: 'success', diagnosticCodes: [], quality: expected.quality,
    volume: expected.volume ?? 100,
    faceCount: expected.faces ?? 6,
    operandVolume: expected.volumeBelowOperand ? 200 : expected.operandVolume,
    validationErrors: expected.validationErrors,
    meshQuality: { isWatertight: true, boundaryEdges: 0, nonManifoldEdges: 0 },
  };
  if (c.checkedValidationIndex !== undefined && surface === 'native') observation.checkedValidationErrors = 0;
  if (expected.evolution) {
    observation.evolution = {
      origin: 'construction',
      modified: Object.fromEntries(Array.from({ length: expected.evolution.modifiedInputs }, (_, i) => [i, [i]])),
      generated: {},
      deleted: Array.from({ length: expected.evolution.deleted }, (_, i) => i + 100),
      unresolved: {},
    };
  }
  return observation;
}

function splitObservation(c) {
  const expected = c.expect;
  return {
    outcome: expected.outcome,
    diagnosticCodes: expected.outcome === 'batch_error' || expected.noPartialSolids ? ['limit_exceeded'] : [],
    volume: expected.volume,
    volumes: expected.volumes,
    faceCount: expected.faces,
    faceCounts: expected.faceCounts,
    shellCount: expected.shells,
    shellCounts: expected.shellCounts,
    validationErrors: expected.validationErrors,
    census: expected.census,
    probes: expected.probes,
    pcurveCount: expected.pcurves,
    stepPcurveCount: expected.pcurves,
    rollbackPreserved: expected.rollbackPreserved,
    preservedBytesEqual: expected.preservedBytesEqual,
    noPartialSolids: expected.noPartialSolids,
    crossSessionStaleRefused: expected.crossSessionStaleRefused,
  };
}

function goodReport(kind) {
  const contract = kind === 'contract';
  const cases = contract ? expandContractMatrix() : expandSplitMatrix();
  const observations = cases.map((c) => ({
    case: c.id,
    ...Object.fromEntries(['native', 'fresh', 'committed'].map((surface) => [
      surface, contract ? contractObservation(c, surface) : splitObservation(c),
    ])),
  }));
  const score = contract ? scoreContractCase : scoreSplitCase;
  const stages = cases.flatMap((c, index) => score(c, observations[index]));
  return {
    schema_version: 1, passed: true,
    matrix: contract
      ? { cases: cases.map(({ id, title }) => ({ id, title })) }
      : { cases: cases.length, ids: cases.map(({ id }) => id) },
    observations, stages,
    failed_stages: [], known_gaps: [],
    provenance: contract
      ? { native: provenance('native'), fresh: provenance('fresh-tarball'), committed: provenance('committed-package') }
      : {
        native: { mode: 'native', sourceRevision: 'ab'.repeat(20) },
        freshKernel: provenance('fresh-tarball'), freshIo: provenance('fresh-tarball'),
        committedKernel: provenance('committed-package'), committedIo: provenance('committed-package'),
      },
  };
}

test('complete proof is rescored with the existing owners and preserves minimal split native metadata', () => {
  const summary = validateProofReports(goodReport('contract'), goodReport('split-io'));
  assert.equal(summary.passed, true);
  assert.equal(summary.contract.cases, 12);
  assert.equal(summary.splitIo.cases, 6);
});

test('empty, malformed and wrong-schema evidence cannot be green', () => {
  for (const kind of ['contract', 'split-io']) {
    for (const report of [null, {}, [], { passed: true }]) {
      assert.throws(() => validateProofReport(kind, report));
    }
    const report = goodReport(kind);
    report.schema_version = 2;
    assert.throws(() => validateProofReport(kind, report), /schema/);
  }
});

test('missing, duplicate and unknown cells or matrix identities are rejected', () => {
  for (const kind of ['contract', 'split-io']) {
    for (const mutate of [
      (report) => report.observations.pop(),
      (report) => { report.observations[1] = report.observations[0]; },
      (report) => { report.observations[0].case = 'unknown'; },
      (report) => {
        if (kind === 'contract') report.matrix.cases.pop();
        else report.matrix.ids.pop();
      },
    ]) {
      const report = goodReport(kind);
      mutate(report);
      assert.throws(() => validateProofReport(kind, report), /identities|observations/);
    }
  }
});

test('every cell requires actual native, fresh and committed observations', () => {
  for (const kind of ['contract', 'split-io']) {
    for (const surface of ['native', 'fresh', 'committed']) {
      const report = goodReport(kind);
      delete report.observations[0][surface];
      assert.throws(() => validateProofReport(kind, report), new RegExp(`missing ${surface} observation`));
    }
  }
  const report = goodReport('contract');
  delete report.observations.at(-1).native.checkedValidationErrors;
  assert.throws(() => validateProofReport('contract', report), /check-crate validation/);
});

test('fabricated passing summary and stages cannot hide wrong raw geometry or rollback', () => {
  for (const kind of ['contract', 'split-io']) {
    const report = goodReport(kind);
    report.observations[0].fresh.volume *= 2;
    assert.throws(() => validateProofReport(kind, report), /raw observations fail required gates/);
  }
  const report = goodReport('contract');
  report.observations.find(({ case: id }) => id === 'contract/transactional-rollback').fresh.rollbackVolumes = [];
  assert.throws(() => validateProofReport('contract', report), /raw observations fail required gates/);
});

test('red, omitted, duplicated or demoted required stage records are rejected', () => {
  for (const kind of ['contract', 'split-io']) {
    for (const mutate of [
      (report) => { report.stages[0].passed = false; },
      (report) => report.stages.shift(),
      (report) => report.stages.push(report.stages[0]),
      (report) => { report.stages[0].required = false; },
      (report) => { report.stages[0].passed = 'true'; },
      (report) => { report.passed = false; },
      (report) => { report.passed = 'true'; },
      (report) => report.failed_stages.push(report.stages[0]),
    ]) {
      const report = goodReport(kind);
      mutate(report);
      assert.throws(() => validateProofReport(kind, report), /stage|pass|summary|required/);
    }
  }
});

test('known non-gating evolution differences remain visible without failing proof', () => {
  const report = goodReport('contract');
  const index = report.observations.findIndex(({ case: id }) => id === 'contract/evolution-fuse');
  const changed = report.observations[index].fresh.evolution;
  changed.modified = Object.fromEntries(Object.entries(changed.modified).map(([key, value]) => [`fresh-${key}`, value]));
  report.stages = expandContractMatrix().flatMap((c, i) => scoreContractCase(c, report.observations[i]));
  report.known_gaps = report.stages.filter(({ required, passed }) => !required && !passed);
  assert.equal(report.known_gaps.length, 1);
  assert.ok(validateProofReport('contract', report));
});

test('source identity, package evidence mode and mixed kernel report artifacts are rejected', () => {
  const contract = goodReport('contract');
  contract.provenance.fresh.mode = 'committed-package';
  assert.throws(() => validateProofReport('contract', contract), /evidence mode/);
  const split = goodReport('split-io');
  split.provenance.freshIo.sourceRevision = '12'.repeat(20);
  assert.throws(() => validateProofReport('split-io', split), /source identity/);
  const mixed = goodReport('split-io');
  mixed.provenance.freshKernel.wasmSha256 = '12'.repeat(32);
  assert.throws(() => validateProofReports(goodReport('contract'), mixed), /kernel wasmSha256/);
});

test('CLI rejects missing and malformed report files and emits a valid summary only for complete reports', () => {
  const dir = mkdtempSync(resolve(tmpdir(), 'remus-o15-verdict-'));
  const cli = fileURLToPath(new URL('./proof-verdict.mjs', import.meta.url));
  const contract = resolve(dir, 'contract.json');
  const split = resolve(dir, 'split.json');
  const run = (...args) => {
    const result = spawnSync(process.execPath, [cli, ...args], { encoding: 'utf8' });
    assert.ifError(result.error);
    return result;
  };
  try {
    assert.equal(run().status, 2);
    assert.equal(run(contract, split).status, 1);
    writeFileSync(contract, '');
    writeFileSync(split, JSON.stringify(goodReport('split-io')));
    assert.equal(run(contract, split).status, 1);
    writeFileSync(contract, '{broken');
    assert.equal(run(contract, split).status, 1);
    writeFileSync(contract, JSON.stringify(goodReport('contract')));
    const result = run(contract, split);
    assert.equal(result.status, 0, result.stderr);
    assert.equal(JSON.parse(result.stdout).passed, true);
    assert.equal(JSON.parse(readFileSync(contract)).observations.length, 12);
  } finally {
    rmSync(dir, { recursive: true, force: true });
  }
});
