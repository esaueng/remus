import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { mkdtempSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { resolve } from 'node:path';
import test from 'node:test';
import { fileURLToPath } from 'node:url';

import { directConsistent, expandContractMatrix, scoreContractCase } from './contract-matrix.mjs';
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

function scoreStages(kind, observations) {
  const contract = kind === 'contract';
  const cases = contract ? expandContractMatrix() : expandSplitMatrix();
  const score = contract ? scoreContractCase : scoreSplitCase;
  return cases.flatMap((c, index) => [
    ...score(c, observations[index]),
    ...(contract && !c.directOnly ? ['fresh', 'committed'].map((surface) => ({
      case: c.id,
      surface: `${surface}-direct-vs-batch`,
      invariant: 'direct_batch_consistency',
      passed: directConsistent(c, observations[index][`${surface}Batch`], observations[index][`${surface}DirectRaw`]),
      required: false,
    })) : []),
  ]);
}

function goodReport(kind) {
  const contract = kind === 'contract';
  const cases = contract ? expandContractMatrix() : expandSplitMatrix();
  const observations = cases.map((c) => {
    const observations = {
      case: c.id,
      ...Object.fromEntries(['native', 'fresh', 'committed'].map((surface) => [
        surface, contract ? contractObservation(c, surface) : splitObservation(c),
      ])),
    };
    if (contract) for (const surface of ['fresh', 'committed']) {
      observations[`${surface}Batch`] = structuredClone(observations[surface]);
      observations[`${surface}DirectRaw`] = structuredClone(observations[surface]);
    }
    return observations;
  });
  const stages = scoreStages(kind, observations);
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

function hostBoundReports() {
  const host = {
    schema_version: 1,
    sourceRevision: 'ab'.repeat(20), sourceDirty: false, checkoutSha: 'ab'.repeat(20),
    expected: { runner: 'ubuntu-24.04', platform: 'linux', arch: 'x64', rustHost: 'x86_64-unknown-linux-gnu' },
    runner: { os: 'Linux', arch: 'X64' },
    node: { platform: 'linux', arch: 'x64', version: 'v24', v8: '13.6' },
    rustc: { host: 'x86_64-unknown-linux-gnu', verbose: 'rustc 1.96\nhost: x86_64-unknown-linux-gnu' },
    unameMachine: 'x86_64', buildTarget: null,
    native: Object.fromEntries([['kernel', 'remus-parity-native'], ['splitIo', 'remus-split-io-native']].map(([key, binary]) => [key, {
      path: `/tmp/${binary}`, sha256: '12'.repeat(32),
      runtime: { schemaVersion: 1, binary, os: 'linux', arch: 'x86_64', pointerWidth: 64 },
    }])),
  };
  const contract = goodReport('contract');
  const split = goodReport('split-io');
  for (const [report, runner] of [[contract, host.native.kernel], [split, host.native.splitIo]]) {
    report.provenance.native.installedEntry = runner.path;
    report.provenance.native.binarySha256 = runner.sha256;
  }
  return [contract, split, host];
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
  report.stages = scoreStages('contract', report.observations);
  report.known_gaps = report.stages.filter(({ required, passed }) => !required && !passed);
  assert.equal(report.known_gaps.length, 1);
  assert.ok(validateProofReport('contract', report));
});

test('omitted advisory failures cannot disappear from known gaps or stage records', () => {
  const report = goodReport('contract');
  const index = report.observations.findIndex(({ case: id }) => id === 'contract/evolution-fuse');
  const changed = report.observations[index].fresh.evolution;
  changed.modified = Object.fromEntries(Object.entries(changed.modified).map(([key, value]) => [`fresh-${key}`, value]));
  report.stages = scoreStages('contract', report.observations);
  report.known_gaps = report.stages.filter(({ required, passed }) => !required && !passed);
  assert.equal(report.known_gaps.length, 1);
  const missingSummary = structuredClone(report);
  missingSummary.known_gaps = [];
  assert.throws(() => validateProofReport('contract', missingSummary), /known.gap|advisory/);
  const hiddenStages = structuredClone(report);
  hiddenStages.stages = hiddenStages.stages.filter(({ required }) => required);
  hiddenStages.known_gaps = [];
  assert.throws(() => validateProofReport('contract', hiddenStages), /stage|known.gap|advisory/);
});

test('every advisory stage is retained exactly, including passing diagnostics', () => {
  for (const kind of ['contract', 'split-io']) {
    for (const mutate of [
      (report, index) => report.stages.splice(index, 1),
      (report, index) => report.stages.push(structuredClone(report.stages[index])),
      (report, index) => { report.stages[index].required = true; },
      (report, index) => { report.stages[index].passed = !report.stages[index].passed; },
      (report, index) => { report.stages[index].invariant = 'fabricated_advisory'; },
      (report, index) => { report.stages[index].operation = 'another operation'; },
    ]) {
      const report = goodReport(kind);
      const index = report.stages.findIndex(({ required }) => !required);
      assert.ok(index >= 0, `${kind} has real advisory stages`);
      mutate(report, index);
      assert.throws(() => validateProofReport(kind, report), /stage records/);
    }
  }
});

test('direct-versus-batch advisory gaps are rescored from retained raw observations', () => {
  const report = goodReport('contract');
  report.observations[0].freshDirectRaw.diagnosticCodes = ['different_direct_diagnostic'];
  report.stages = scoreStages('contract', report.observations);
  report.known_gaps = report.stages.filter(({ required, passed }) => !required && !passed);
  assert.equal(report.known_gaps.length, 1);
  assert.equal(report.known_gaps[0].invariant, 'direct_batch_consistency');
  assert.ok(validateProofReport('contract', report));
  for (const mutate of [
    (changed) => { changed.known_gaps = []; },
    (changed) => changed.known_gaps.push(structuredClone(changed.known_gaps[0])),
    (changed) => { changed.known_gaps[0].passed = true; },
    (changed) => { changed.known_gaps[0].invariant = 'fabricated_gap'; },
    (changed) => {
      changed.stages = changed.stages.filter(({ invariant }) => invariant !== 'direct_batch_consistency');
      changed.known_gaps = [];
    },
  ]) {
    const changed = structuredClone(report);
    mutate(changed);
    assert.throws(() => validateProofReport('contract', changed), /known.gap|stage records/);
  }
  for (const field of ['freshBatch', 'freshDirectRaw', 'committedBatch', 'committedDirectRaw']) {
    const changed = goodReport('contract');
    delete changed.observations[0][field];
    assert.throws(() => validateProofReport('contract', changed), /missing .*advisory observation/);
  }
});

test('matching WASM bytes cannot hide different installed fresh tarballs', () => {
  const contract = goodReport('contract');
  const split = goodReport('split-io');
  split.provenance.freshKernel.tarballSha256 = '12'.repeat(32);
  assert.throws(() => validateProofReports(contract, split), /kernel tarballSha256/);
});

test('shared fresh packages retain the same target, build profile and toolchain', () => {
  for (const [field, mutate] of [
    ['target', (split) => {
      split.provenance.freshKernel.target = 'web';
      split.provenance.freshKernel.buildOptions.target = 'web';
    }],
    ['build profile', (split) => { split.provenance.freshKernel.buildOptions.profile = 'dev'; }],
    ['toolchain', (split) => {
      for (const entry of Object.values(split.provenance)) {
        if (entry.toolchain) entry.toolchain.rustc = 'rustc 1.97';
      }
    }],
  ]) {
    const split = goodReport('split-io');
    mutate(split);
    assert.throws(() => validateProofReports(goodReport('contract'), split), new RegExp(`kernel ${field}`));
  }
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

test('host qualification binds actual native executable hashes, source, compiler and Node to both matrices', () => {
  const reports = hostBoundReports();
  assert.equal(validateProofReports(...reports).host.arch, 'x64');
  for (const mutate of [
    ([, , host]) => { host.native.kernel.sha256 = '34'.repeat(32); },
    ([, split]) => { delete split.provenance.native.installedEntry; },
    ([, , host]) => { host.sourceRevision = '34'.repeat(20); host.checkoutSha = host.sourceRevision; },
    ([, , host]) => { host.sourceDirty = true; },
    ([, , host]) => { host.node.version = 'v26'; },
    ([, , host]) => { host.rustc.verbose = host.rustc.verbose.replace('rustc 1.96', 'rustc 1.97'); },
    ([, , host]) => { delete host.native.splitIo; },
  ]) {
    const changed = hostBoundReports();
    mutate(changed);
    assert.throws(() => validateProofReports(...changed), /host proof|splitIo/);
  }
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
