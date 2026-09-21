import assert from 'node:assert/strict';
import test from 'node:test';

import { expandContractMatrix, scoreContractCase } from './contract-matrix.mjs';

test('contract matrix covers the six required evidence cells', () => {
  const cases = expandContractMatrix();
  assert.deepEqual(
    cases.map(({ id }) => id),
    [
      'contract/exact-fuse',
      'contract/exact-only-refusal',
      'contract/disclosed-approximation',
      'contract/invalid-handle',
      'contract/transactional-rollback',
      'contract/cancelled-boolean',
    ],
  );
  const titles = Object.fromEntries(cases.map(({ id, title }) => [id, title]));
  assert.match(titles['contract/exact-fuse'], /exact result/);
  assert.match(titles['contract/exact-only-refusal'], /refusal/);
  assert.match(titles['contract/disclosed-approximation'], /approximation/);
  assert.match(titles['contract/invalid-handle'], /invalid handle/);
  assert.match(titles['contract/transactional-rollback'], /rollback/);
  assert.match(titles['contract/cancelled-boolean'], /cancellation/);
  const cancelled = cases.find(({ id }) => id === 'contract/cancelled-boolean');
  assert.match(cancelled.cancellationScope, /pre-cancelled/);
});

test('exact success requires quality and oracle volume on every surface', () => {
  const cases = expandContractMatrix();
  const c = cases.find(({ id }) => id === 'contract/exact-fuse');
  const good = {
    outcome: 'success', diagnosticCodes: [], quality: 'exact', volume: 15,
  };
  const stages = scoreContractCase(c, { native: good, fresh: { ...good }, committed: { ...good } });
  assert.ok(stages.filter(({ required }) => required).every(({ passed }) => passed));

  const wrongQuality = { ...good, quality: 'approximate' };
  assert.equal(
    scoreContractCase(c, { native: good, fresh: wrongQuality })
      .find(({ surface, invariant }) => surface === 'fresh' && invariant === 'disclosed_quality')?.passed,
    false,
  );
  const wrongVolume = { ...good, volume: 15 * 1.01 };
  assert.equal(
    scoreContractCase(c, { native: good, fresh: wrongVolume })
      .find(({ surface, invariant }) => surface === 'fresh' && invariant === 'oracle_volume')?.passed,
    false,
  );
});

test('refusal cells require the declared code and cross-surface diagnostics agreement', () => {
  const cases = expandContractMatrix();
  const c = cases.find(({ id }) => id === 'contract/exact-only-refusal');
  const refused = { outcome: 'batch_error', diagnosticCodes: ['operation_failed'] };
  const stages = scoreContractCase(c, { native: refused, fresh: { ...refused } });
  assert.ok(stages.filter(({ required }) => required).every(({ passed }) => passed));

  const wrongCode = { outcome: 'batch_error', diagnosticCodes: ['invalid_handle'] };
  assert.equal(
    scoreContractCase(c, { native: refused, fresh: wrongCode })
      .find(({ surface, invariant }) => surface === 'fresh' && invariant === 'refusal_code')?.passed,
    false,
  );
  assert.equal(
    scoreContractCase(c, { native: refused, fresh: wrongCode })
      .find(({ invariant }) => invariant === 'diagnostics_agreement')?.passed,
    false,
  );
});

test('rollback cells require preserved post-failure probes', () => {
  const cases = expandContractMatrix();
  const c = cases.find(({ id }) => id === 'contract/transactional-rollback');
  const preserved = {
    outcome: 'batch_error',
    diagnosticCodes: ['invalid_handle'],
    rollbackVolumes: [8],
    rollbackFaces: 6,
  };
  assert.ok(
    scoreContractCase(c, { native: preserved, fresh: { ...preserved } })
      .filter(({ required }) => required).every(({ passed }) => passed),
  );
  const lost = { ...preserved, rollbackVolumes: [7] };
  assert.equal(
    scoreContractCase(c, { native: preserved, fresh: lost })
      .find(({ surface, invariant }) => surface === 'fresh' && invariant === 'rollback_preserved')?.passed,
    false,
  );
  const missing = { outcome: 'batch_error', diagnosticCodes: ['invalid_handle'] };
  assert.equal(
    scoreContractCase(c, { native: preserved, fresh: missing })
      .find(({ surface, invariant }) => surface === 'fresh' && invariant === 'rollback_preserved')?.passed,
    false,
  );
});

test('missing observations fail instead of passing vacuously', () => {
  const cases = expandContractMatrix();
  const c = cases.find(({ id }) => id === 'contract/invalid-handle');
  assert.throws(() => scoreContractCase(c, null), /missing/);
  assert.throws(() => scoreContractCase(c, { native: null }), /missing/);
});
