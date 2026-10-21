import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

import { expandContractMatrix, scoreContractCase } from './contract-matrix.mjs';
import { expandSplitMatrix, scoreSplitCase } from './split-io-matrix.mjs';
import { validateProvenance } from './provenance.mjs';

function object(value) {
  return value !== null && typeof value === 'object' && !Array.isArray(value);
}

function stageKey(stage) {
  return JSON.stringify([stage.case, stage.surface, stage.invariant]);
}

function validateIdentity(kind, provenance) {
  assert.ok(object(provenance), `${kind}: provenance is missing`);
  const entries = kind === 'contract'
    ? { native: 'native', fresh: 'fresh-tarball', committed: 'committed-package' }
    : {
      freshKernel: 'fresh-tarball', freshIo: 'fresh-tarball',
      committedKernel: 'committed-package', committedIo: 'committed-package',
    };
  for (const [name, mode] of Object.entries(entries)) {
    assert.ok(object(provenance[name]), `${kind}: missing ${name} provenance`);
    assert.equal(provenance[name].mode, mode, `${kind}: wrong ${name} evidence mode`);
    validateProvenance(provenance[name]);
  }
  const source = kind === 'contract' ? provenance.fresh : provenance.freshKernel;
  assert.equal(provenance.native?.mode, 'native', `${kind}: missing native provenance`);
  // The split harness historically records only the native source ID. Its
  // artifact/build metadata must not be invented by this report consumer.
  for (const [name, entry] of Object.entries(provenance)) {
    if (!entries[name] && name !== 'native') continue;
    assert.equal(entry.sourceRevision, source.sourceRevision, `${kind}: ${name} source identity differs`);
    if (name !== 'native' || kind === 'contract') {
      assert.equal(entry.sourceDirty, source.sourceDirty, `${kind}: ${name} dirty flag differs`);
      assert.deepEqual(entry.toolchain, source.toolchain, `${kind}: ${name} toolchain differs`);
    }
  }
}

/** Validate complete proof evidence by reusing the owning matrix scorers. */
export function validateProofReport(kind, report) {
  assert.ok(['contract', 'split-io'].includes(kind), `unknown proof kind: ${kind}`);
  assert.ok(object(report), `${kind}: report is missing`);
  assert.equal(report.schema_version, 1, `${kind}: unsupported report schema`);
  assert.equal(report.passed, true, `${kind}: report did not pass`);
  const cases = kind === 'contract' ? expandContractMatrix() : expandSplitMatrix();
  const ids = cases.map(({ id }) => id);
  const matrixIds = kind === 'contract'
    ? report.matrix?.cases?.map?.(({ id }) => id)
    : report.matrix?.ids;
  assert.deepEqual(matrixIds, ids, `${kind}: matrix identities are incomplete or changed`);
  if (kind === 'split-io') assert.equal(report.matrix.cases, ids.length, `${kind}: wrong matrix denominator`);
  assert.ok(Array.isArray(report.observations), `${kind}: observations are missing`);
  assert.deepEqual(report.observations.map((entry) => entry?.case), ids, `${kind}: observations are incomplete or duplicated`);
  validateIdentity(kind, report.provenance);

  const score = kind === 'contract' ? scoreContractCase : scoreSplitCase;
  const recomputed = cases.flatMap((c, index) => {
    const observations = report.observations[index];
    for (const surface of ['native', 'fresh', 'committed']) {
      const observation = observations[surface];
      assert.ok(object(observation), `${c.id}: missing ${surface} observation`);
      assert.ok(typeof observation.outcome === 'string', `${c.id}: missing ${surface} outcome`);
      assert.ok(
        Array.isArray(observation.diagnosticCodes)
          && observation.diagnosticCodes.every((code) => typeof code === 'string'),
        `${c.id}: missing ${surface} diagnostic codes`,
      );
    }
    if (kind === 'contract' && c.checkedValidationIndex !== undefined) {
      assert.ok(
        Number.isInteger(observations.native.checkedValidationErrors),
        `${c.id}: missing native check-crate validation`,
      );
    }
    return score(c, {
      native: observations.native, fresh: observations.fresh, committed: observations.committed,
    });
  });
  const required = recomputed.filter((stage) => stage.required);
  const failures = required.filter((stage) => stage.passed !== true);
  assert.equal(failures.length, 0, `${kind}: raw observations fail required gates: ${failures.map(stageKey).join(', ')}`);
  assert.ok(Array.isArray(report.failed_stages), `${kind}: failed-stage summary is missing`);
  assert.equal(report.failed_stages.length, 0, `${kind}: report records required failures`);
  assert.ok(Array.isArray(report.known_gaps), `${kind}: known-gap summary is missing`);
  assert.ok(report.known_gaps.every((stage) => stage.required === false), `${kind}: required failures cannot be known gaps`);
  assert.ok(Array.isArray(report.stages), `${kind}: stage records are missing`);
  for (const stage of report.stages) {
    assert.ok(object(stage) && ids.includes(stage.case), `${kind}: unknown stage case`);
    assert.equal(typeof stage.required, 'boolean', `${kind}: stage required flag is not boolean`);
    assert.equal(typeof stage.passed, 'boolean', `${kind}: stage passed flag is not boolean`);
  }
  const recorded = report.stages.filter((stage) => stage.required);
  assert.deepEqual(
    recorded.map(stageKey).sort(), required.map(stageKey).sort(),
    `${kind}: required stage records disagree with recomputed gates`,
  );
  assert.ok(recorded.every((stage) => stage.passed), `${kind}: required stage is red`);
  return { cases: ids.length, requiredStages: required.length };
}

/** Validate both independent slices and their shared kernel artifact identity. */
export function validateProofReports(contract, split) {
  const summary = {
    contract: validateProofReport('contract', contract),
    splitIo: validateProofReport('split-io', split),
  };
  for (const [left, right] of [
    [contract.provenance.fresh, split.provenance.freshKernel],
    [contract.provenance.committed, split.provenance.committedKernel],
  ]) {
    for (const field of ['sourceRevision', 'sourceDirty', 'packageVersion', 'wasmSha256']) {
      assert.equal(left[field], right[field], `proof slices disagree on kernel ${field}`);
    }
  }
  return { passed: true, ...summary };
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  if (process.argv.length !== 4) {
    console.error('usage: proof-verdict.mjs CONTRACT_REPORT SPLIT_IO_REPORT');
    process.exitCode = 2;
  } else {
    try {
      const reports = process.argv.slice(2).map((path) => JSON.parse(readFileSync(path, 'utf8')));
      console.log(JSON.stringify(validateProofReports(...reports)));
    } catch (error) {
      console.error(`O1.5 proof rejected: ${error.message}`);
      process.exitCode = 1;
    }
  }
}
