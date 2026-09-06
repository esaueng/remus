import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import test from 'node:test';
import { score, validateCases } from './w9-preflight.mjs';

const cases = JSON.parse(readFileSync(new URL('./w9-preflight.json', import.meta.url)));
const c = cases[0];
const good = { id: c.id, outcome: 'typed_refusal', code: c.expected_code, diagnostic: 'limit exceeded', unchanged: true, snapshot_bytes: 100 };
const observations = () => ({ native: [{ ...good }, { ...good }], wasm: [{ ...good }, { ...good }] });

test('manifest is nonempty, versioned, unique, and closed', () => {
  validateCases(cases);
  for (const bad of [[], [c, c], [{ ...c, schema_version: 2 }], [{ ...c, max_entities: 0 }], [{ ...c, extra: true }]]) {
    assert.throws(() => validateCases(bad));
  }
});

test('every refusal requires independent diagnostic and complete snapshot evidence', () => {
  assert.ok(score(c, observations()).every(s => s.passed));
  for (const patch of [{ unchanged: false }, { snapshot_bytes: 0 }, { code: 'wrong' }, { diagnostic: '' }, { id: 'wrong' }, { outcome: 'invalid_success' }, { outcome: 'crash' }, { outcome: 'hang_or_budget_overrun' }]) {
    const obs = observations();
    Object.assign(obs.wasm[1], patch);
    assert.ok(score(c, obs).some(s => !s.passed), JSON.stringify(patch));
  }
});

test('missing repetitions and inconsistent refusals cannot pass', () => {
  const obs = observations();
  obs.native.pop();
  assert.throws(() => score(c, obs));
  const different = observations();
  different.wasm[1].diagnostic = 'different refusal';
  assert.equal(score(c, different).find(s => s.surface === 'wasm' && s.stage === 'repeat_agreement').passed, false);
});
