import assert from 'node:assert/strict';
import test from 'node:test';

import {
  FAILURE_CLASSES,
  HAMMER_HOLDER_FIXTURE,
  HAMMER_SHIFTED_INTERSECT_ID,
  classifyFailures,
  expandContractMatrix,
  resolveFixtures,
  scoreContractCase,
  summarizeEvolution,
  summarizeFailureClasses,
} from './contract-matrix.mjs';

test('contract matrix covers the twelve required evidence cells', () => {
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
      'contract/evolution-fuse',
      'contract/evolution-cut',
      'contract/invalid-primitive-input',
      'contract/empty-intersect-sentinel',
      'contract/contained-cut-refusal',
      'contract/hammer-shifted-intersect',
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
  assert.match(titles['contract/evolution-fuse'], /evolution/);
  assert.match(titles['contract/evolution-cut'], /evolution/);
  assert.match(titles['contract/invalid-primitive-input'], /invalid primitive/);
  assert.match(titles['contract/empty-intersect-sentinel'], /empty intersect/);
  assert.match(titles['contract/contained-cut-refusal'], /contained-cut/);
  assert.match(titles[HAMMER_SHIFTED_INTERSECT_ID], /hammer-holder shifted intersect/);
});

function hammerObservation(overrides = {}) {
  return {
    outcome: 'success',
    diagnosticCodes: [],
    quality: 'exact',
    volume: 5000,
    faceCount: 104,
    validationErrors: 0,
    checkedValidationErrors: null,
    meshQuality: { triangleCount: 20000, boundaryEdges: 0, nonManifoldEdges: 0, eulerCharacteristic: 2, isWatertight: true },
    operandVolume: 9299.47,
    ...overrides,
  };
}

test('hammer shifted-intersect cell replays the smoke sequence from one exact arena document, direct-only', () => {
  const cases = expandContractMatrix();
  const c = cases.find(({ id }) => id === HAMMER_SHIFTED_INTERSECT_ID);
  assert.equal(c.directOnly, true);
  assert.match(c.directOnlyScope, /deserializeSolids/);
  assert.equal(c.fixture.step, HAMMER_HOLDER_FIXTURE);
  assert.match(HAMMER_HOLDER_FIXTURE, /shapr3d_hammer_holder\.step$/);
  // Both operands enter through the same arena document; the mask is the
  // smoke's 29x53x70 box at (-18,-10,0); the shifted copy sits at -2 in x.
  assert.deepEqual(c.batch[0], { op: 'deserializeSolids', args: { fixture: HAMMER_HOLDER_FIXTURE } });
  assert.deepEqual(c.batch[5], { op: 'deserializeSolids', args: { fixture: HAMMER_HOLDER_FIXTURE } });
  assert.deepEqual(c.batch[1].args, { width: 29, height: 53, depth: 70 });
  assert.deepEqual(c.batch[2].args.matrix.slice(0, 4), [1, 0, 0, -18]);
  assert.deepEqual(c.batch[6].args.matrix.slice(0, 4), [1, 0, 0, -2]);
  assert.equal(c.batch[3].args.operation, 'cut');
  assert.equal(c.batch[4].args.operation, 'intersect');
  assert.equal(c.batch[c.booleanIndex].args.operation, 'intersect');
  assert.deepEqual(c.batch[c.booleanIndex].args.solidA, { fromOp: 4, pick: 'solid' });
  for (const index of [3, 4, c.booleanIndex]) assert.equal(c.batch[index].args.exactOnly, true);
  assert.deepEqual(c.resultHandle, { fromOp: c.booleanIndex });
  // Probes: volume at 0.01, both validators, watertightness at (0.05, 0.1),
  // the face list, and the common's volume for the strict-inside bound.
  assert.deepEqual(c.batch[c.volumeIndex].args.deflection, 0.01);
  assert.equal(c.batch[c.validationIndex].op, 'validateSolid');
  assert.equal(c.batch[c.checkedValidationIndex].op, 'validateSolidChecked');
  assert.deepEqual(c.batch[c.meshQualityIndex].args, { solid: { fromOp: 7, pick: 'solid' }, deflection: 0.05, angularTolerance: 0.1 });
  assert.equal(c.batch[c.facesIndex].op, 'getSolidFaces');
  assert.deepEqual(c.batch[c.operandVolumeIndex].args.solid, { fromOp: 4, pick: 'solid' });
  assert.deepEqual(c.expect, {
    outcome: 'success',
    quality: 'exact',
    faces: 104,
    validationErrors: 0,
    watertight: true,
    volumeBelowOperand: true,
    crossSurfaceVolumeBound: 5e-8,
  });
});

test('resolveFixtures swaps every fixture argument for the loaded arena hex, once per STEP path', () => {
  const cases = expandContractMatrix();
  const loads = [];
  const resolved = resolveFixtures(cases, (step) => {
    loads.push(step);
    return 'cafe';
  });
  assert.deepEqual(loads, [HAMMER_HOLDER_FIXTURE]);
  const c = resolved.find(({ id }) => id === HAMMER_SHIFTED_INTERSECT_ID);
  assert.deepEqual(c.batch[0], { op: 'deserializeSolids', args: { bytesHex: 'cafe' } });
  assert.deepEqual(c.batch[5], { op: 'deserializeSolids', args: { bytesHex: 'cafe' } });
  assert.equal(c.batch.some((item) => item.args?.fixture), false);
  // Untouched cells keep their batches by identity; the source list is not mutated.
  const exact = cases.find(({ id }) => id === 'contract/exact-fuse');
  assert.equal(resolved.find(({ id }) => id === 'contract/exact-fuse').batch, exact.batch);
  assert.equal(cases.find(({ id }) => id === HAMMER_SHIFTED_INTERSECT_ID).batch[0].args.fixture, HAMMER_HOLDER_FIXTURE);
  assert.throws(() => resolveFixtures(cases, () => ''), /empty arena document/);
});

test('hammer cell gates face count, both validators, watertightness, and the strict-inside volume on every surface', () => {
  const cases = expandContractMatrix();
  const c = cases.find(({ id }) => id === HAMMER_SHIFTED_INTERSECT_ID);
  const native = hammerObservation({ checkedValidationErrors: 0 });
  const stages = scoreContractCase(c, { native, fresh: hammerObservation(), committed: hammerObservation() });
  assert.ok(stages.filter(({ required }) => required).every(({ passed }) => passed));
  const invariantsOf = (surface) => new Set(stages.filter((s) => s.surface === surface).map(({ invariant }) => invariant));
  for (const invariant of ['batch_success', 'disclosed_quality', 'face_count', 'strict_validation', 'watertight_mesh', 'volume_below_operand']) {
    assert.ok(invariantsOf('native').has(invariant), `native missing ${invariant}`);
    assert.ok(invariantsOf('fresh').has(invariant), `fresh missing ${invariant}`);
  }
  // The check-crate validator has no WASM binding: it gates natively and
  // is absent, not vacuously green, on the installed surfaces.
  assert.ok(invariantsOf('native').has('checked_validation'));
  assert.equal(invariantsOf('fresh').has('checked_validation'), false);
  for (const invariant of ['face_count_agreement', 'validation_agreement', 'mesh_quality_agreement', 'volume_agreement', 'quality_agreement']) {
    assert.ok(invariantsOf('cross-surface').has(invariant), `cross-surface missing ${invariant}`);
  }
  assert.equal(stages.some(({ failureClass }) => failureClass), false, 'green stages carry no failure class');

  // The check-crate validator failing natively is a contract violation.
  const checkedRed = scoreContractCase(c, { native: hammerObservation({ checkedValidationErrors: 2 }), fresh: hammerObservation() });
  const checked = checkedRed.find(({ surface, invariant }) => surface === 'native' && invariant === 'checked_validation');
  assert.equal(checked.passed, false);
  assert.equal(checked.failureClass, FAILURE_CLASSES.contractViolation);

  // A volume outside (0, common) or a cross-surface volume drift is a finding.
  const inflated = scoreContractCase(c, { native, fresh: hammerObservation({ volume: 9400 }) });
  assert.equal(inflated.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'volume_below_operand')?.passed, false);
  assert.equal(inflated.find(({ invariant }) => invariant === 'volume_agreement')?.passed, false);
  const drifted = scoreContractCase(c, { native, fresh: hammerObservation({ volume: 5000 * (1 + 1e-7) }) });
  assert.equal(drifted.find(({ invariant }) => invariant === 'volume_agreement')?.passed, false);
  const within = scoreContractCase(c, { native, fresh: hammerObservation({ volume: 5000 * (1 + 1e-9) }) });
  assert.equal(within.find(({ invariant }) => invariant === 'volume_agreement')?.passed, true);
});

test('the B39 wasm32-only refusal and free edges report as platform_divergence, never as a contract violation', () => {
  const cases = expandContractMatrix();
  const c = cases.find(({ id }) => id === HAMMER_SHIFTED_INTERSECT_ID);
  const native = hammerObservation({ checkedValidationErrors: 0 });

  // 8539b266 shape: native closes, the fresh tarball refuses exact-only.
  const refused = { outcome: 'batch_error', diagnosticCodes: ['operation_failed'], rollbackVolumes: [], rollbackFaces: null };
  const stages = scoreContractCase(c, { native, fresh: refused, committed: hammerObservation() });
  const failed = stages.filter(({ required, passed }) => required && !passed);
  assert.ok(failed.length > 0);
  assert.ok(failed.every(({ failureClass }) => failureClass === FAILURE_CLASSES.platformDivergence), JSON.stringify(failed));
  assert.ok(failed.some(({ surface, invariant }) => surface === 'fresh' && invariant === 'batch_success'));
  assert.ok(failed.some(({ surface, invariant }) => surface === 'cross-surface' && invariant === 'outcome_agreement'));
  assert.ok(stages.filter(({ surface }) => surface === 'native').every(({ passed }) => passed));
  const classes = summarizeFailureClasses(stages);
  assert.ok(classes.platform_divergence.length >= 2);
  assert.equal(classes.contract_violation, undefined);

  // The same 104-face result with 4 free edges on the installed surface:
  // the validators and the mesh disagree while the face count agrees.
  const leaky = hammerObservation({
    validationErrors: 1,
    meshQuality: { triangleCount: 20000, boundaryEdges: 4, nonManifoldEdges: 0, eulerCharacteristic: 1, isWatertight: false },
  });
  const leakyStages = scoreContractCase(c, { native, fresh: leaky });
  for (const invariant of ['strict_validation', 'watertight_mesh']) {
    const stage = leakyStages.find((s) => s.surface === 'fresh' && s.invariant === invariant);
    assert.equal(stage.passed, false, invariant);
    assert.equal(stage.failureClass, FAILURE_CLASSES.platformDivergence, invariant);
  }
  for (const invariant of ['validation_agreement', 'mesh_quality_agreement']) {
    const stage = leakyStages.find((s) => s.invariant === invariant);
    assert.equal(stage.passed, false, invariant);
    assert.equal(stage.failureClass, FAILURE_CLASSES.platformDivergence, invariant);
  }
  assert.equal(leakyStages.find(({ invariant }) => invariant === 'face_count_agreement')?.passed, true);

  // When the native facade misses the pin too, nothing is a platform
  // divergence: every failure is a contract violation.
  const bothLeaky = scoreContractCase(c, { native: { ...leaky, checkedValidationErrors: 1 }, fresh: leaky });
  const bothFailed = bothLeaky.filter(({ required, passed }) => required && !passed);
  assert.ok(bothFailed.length > 0);
  assert.ok(bothFailed.every(({ failureClass }) => failureClass === FAILURE_CLASSES.contractViolation));
  assert.deepEqual(Object.keys(summarizeFailureClasses(bothLeaky)), [FAILURE_CLASSES.contractViolation]);
});

test('classifyFailures applies to every cell: a green native with a red installed surface is a platform divergence', () => {
  const cases = expandContractMatrix();
  const c = cases.find(({ id }) => id === 'contract/exact-fuse');
  const good = { outcome: 'success', diagnosticCodes: [], quality: 'exact', volume: 15 };
  const approx = { ...good, quality: 'approximate' };
  const stages = scoreContractCase(c, { native: good, fresh: approx });
  const quality = stages.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'disclosed_quality');
  assert.equal(quality.passed, false);
  assert.equal(quality.failureClass, FAILURE_CLASSES.platformDivergence);
  assert.equal(stages.find(({ invariant }) => invariant === 'quality_agreement')?.failureClass, FAILURE_CLASSES.platformDivergence);
  const nativeRed = scoreContractCase(c, { native: approx, fresh: good });
  assert.equal(
    nativeRed.find(({ surface, invariant }) => surface === 'native' && invariant === 'disclosed_quality')?.failureClass,
    FAILURE_CLASSES.contractViolation,
  );
  // Non-gating stages never carry a class.
  assert.ok(classifyFailures([{ surface: 'cross-surface', invariant: 'x', passed: false, required: false }])
    .every(({ failureClass }) => failureClass === undefined));
});

test('evolution cells drive the *WithEvolution batch ops and expect construction-derived buckets', () => {
  const cases = expandContractMatrix();
  const fuse = cases.find(({ id }) => id === 'contract/evolution-fuse');
  const cut = cases.find(({ id }) => id === 'contract/evolution-cut');
  assert.equal(fuse.batch[fuse.evolutionIndex].op, 'fuseWithEvolution');
  assert.equal(cut.batch[cut.evolutionIndex].op, 'cutWithEvolution');
  assert.equal(fuse.batch[fuse.facesIndex].op, 'getSolidFaces');
  assert.equal(cut.batch[cut.volumeIndex].op, 'volume');
  assert.deepEqual(fuse.expect, {
    outcome: 'success',
    quality: 'exact',
    volume: 15,
    volumeTolerance: 1e-6,
    faces: 12,
    evolution: {
      modifiedInputs: 12, modifiedOutputs: 12, generatedInputs: 0, generatedOutputs: 0,
      deleted: 0, unresolved: 0, origin: 'construction',
    },
    unresolvedBucket: {},
  });
  assert.equal(cut.expect.volume, 7);
  assert.equal(cut.expect.faces, 9);
  assert.equal(cut.expect.evolution.deleted, 3);
  assert.equal(cut.expect.evolution.modifiedInputs, 9);
});

test('summarizeEvolution counts every bucket of the wire shape', () => {
  assert.equal(summarizeEvolution(null), null);
  assert.equal(summarizeEvolution('not an object'), null);
  const summary = summarizeEvolution({
    modified: { 0: [12], 1: [17, 18] },
    generated: { 4: [30] },
    deleted: [7, 9, 11],
    unresolved: { 40: [1, 2], 41: [] },
    origin: 'construction',
  });
  assert.deepEqual(summary, {
    modifiedInputs: 2,
    modifiedOutputs: 3,
    generatedInputs: 1,
    generatedOutputs: 1,
    deleted: 3,
    unresolved: 2,
    origin: 'construction',
  });
  assert.deepEqual(summarizeEvolution({}), {
    modifiedInputs: 0, modifiedOutputs: 0, generatedInputs: 0, generatedOutputs: 0,
    deleted: 0, unresolved: 0, origin: null,
  });
});

function fuseEvolution() {
  const modified = {};
  for (let input = 0; input < 12; input++) modified[input] = [12 + input];
  return { modified, generated: {}, deleted: [], unresolved: {}, origin: 'construction' };
}

test('evolution cells gate bucket counts, the unresolved bucket, face count, and cross-surface agreement', () => {
  const cases = expandContractMatrix();
  const c = cases.find(({ id }) => id === 'contract/evolution-fuse');
  const good = {
    outcome: 'success', diagnosticCodes: [], quality: 'exact', volume: 15, faceCount: 12,
    evolution: fuseEvolution(),
  };
  const stages = scoreContractCase(c, { native: good, fresh: { ...good }, committed: { ...good } });
  assert.ok(stages.filter(({ required }) => required).every(({ passed }) => passed));
  const invariants = new Set(stages.map(({ invariant }) => invariant));
  for (const invariant of [
    'face_count', 'evolution_buckets', 'evolution_unresolved',
    'face_count_agreement', 'evolution_agreement', 'evolution_raw_agreement',
  ]) {
    assert.ok(invariants.has(invariant), `missing invariant ${invariant}`);
  }
  assert.equal(stages.find(({ invariant }) => invariant === 'evolution_raw_agreement').required, false);

  // A surface that reports one face as generated instead of modified keeps
  // the same face count but changes the bucket counts: a finding on that
  // surface and a cross-surface disagreement.
  const generatedInstead = { ...good, evolution: { ...fuseEvolution(), modified: (() => {
    const m = fuseEvolution().modified; delete m[11]; return m;
  })(), generated: { 11: [23] } } };
  const drifted = scoreContractCase(c, { native: good, fresh: generatedInstead, committed: { ...good } });
  assert.equal(
    drifted.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'evolution_buckets')?.passed,
    false,
  );
  assert.equal(drifted.find(({ invariant }) => invariant === 'evolution_agreement')?.passed, false);
  assert.equal(drifted.find(({ invariant }) => invariant === 'face_count_agreement')?.passed, true);
  assert.ok(drifted.some(({ required, passed }) => required && !passed), 'the drift must be a finding');

  // An unresolved face fails the unresolved gate even when counts match.
  const unresolved = { ...good, evolution: { ...fuseEvolution(), unresolved: { 23: [11, 5] } } };
  const scoredUnresolved = scoreContractCase(c, { native: good, fresh: unresolved });
  assert.equal(
    scoredUnresolved.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'evolution_unresolved')?.passed,
    false,
  );
  assert.equal(
    scoredUnresolved.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'evolution_buckets')?.passed,
    false,
  );

  // A face-count drift alone is caught per surface and across surfaces.
  const wrongFaces = { ...good, faceCount: 13 };
  const scoredFaces = scoreContractCase(c, { native: good, fresh: wrongFaces });
  assert.equal(
    scoredFaces.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'face_count')?.passed,
    false,
  );
  assert.equal(scoredFaces.find(({ invariant }) => invariant === 'face_count_agreement')?.passed, false);

  // Different arena handles with identical buckets pass the gates and only
  // trip the non-gating raw-agreement evidence.
  const renumbered = { ...good, evolution: { ...fuseEvolution(), modified: (() => {
    const m = {}; for (let input = 0; input < 12; input++) m[input] = [40 + input]; return m;
  })() } };
  const scoredRenumbered = scoreContractCase(c, { native: good, fresh: renumbered });
  assert.ok(scoredRenumbered.filter(({ required }) => required).every(({ passed }) => passed));
  assert.equal(scoredRenumbered.find(({ invariant }) => invariant === 'evolution_raw_agreement')?.passed, false);

  // A missing evolution report is a finding, never a vacuous pass.
  const missing = { ...good, evolution: null };
  const scoredMissing = scoreContractCase(c, { native: good, fresh: missing });
  assert.equal(
    scoredMissing.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'evolution_buckets')?.passed,
    false,
  );
  assert.equal(
    scoredMissing.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'evolution_unresolved')?.passed,
    false,
  );
});

test('failure-class cells require the declared code plus preserved rollback probes', () => {
  const cases = expandContractMatrix();
  const primitive = cases.find(({ id }) => id === 'contract/invalid-primitive-input');
  assert.equal(primitive.batch[1].op, 'makeBox');
  assert.ok(primitive.batch[1].args.width < 0);
  const refused = {
    outcome: 'batch_error', diagnosticCodes: ['invalid_argument'], rollbackVolumes: [8], rollbackFaces: 6,
  };
  assert.ok(
    scoreContractCase(primitive, { native: refused, fresh: { ...refused }, committed: { ...refused } })
      .filter(({ required }) => required).every(({ passed }) => passed),
  );
  // The stale-handle code on one surface is a wrong-vocabulary finding.
  const wrongCode = { ...refused, diagnosticCodes: ['invalid_handle'] };
  const scoredWrong = scoreContractCase(primitive, { native: refused, fresh: wrongCode });
  assert.equal(
    scoredWrong.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'refusal_code')?.passed,
    false,
  );
  assert.equal(scoredWrong.find(({ invariant }) => invariant === 'diagnostics_agreement')?.passed, false);
  // A surface that silently accepts the negative box is caught on outcome.
  const accepted = { outcome: 'success', diagnosticCodes: [], quality: 'exact', volume: -4 };
  const scoredAccepted = scoreContractCase(primitive, { native: refused, fresh: accepted });
  assert.equal(
    scoredAccepted.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'refusal_outcome')?.passed,
    false,
  );
  assert.equal(scoredAccepted.find(({ invariant }) => invariant === 'outcome_agreement')?.passed, false);

  const contained = cases.find(({ id }) => id === 'contract/contained-cut-refusal');
  assert.equal(contained.batch[contained.booleanIndex].args.operation, 'cut');
  assert.equal(contained.expect.code, 'operation_failed');
  const containedRefused = {
    outcome: 'batch_error', diagnosticCodes: ['operation_failed'], rollbackVolumes: [64], rollbackFaces: 6,
  };
  assert.ok(
    scoreContractCase(contained, { native: containedRefused, fresh: { ...containedRefused } })
      .filter(({ required }) => required).every(({ passed }) => passed),
  );
  const toolLost = { ...containedRefused, rollbackVolumes: [63] };
  assert.equal(
    scoreContractCase(contained, { native: containedRefused, fresh: toolLost })
      .find(({ surface, invariant }) => surface === 'fresh' && invariant === 'rollback_preserved')?.passed,
    false,
  );
  const facesLost = { ...containedRefused, rollbackFaces: 5 };
  assert.equal(
    scoreContractCase(contained, { native: containedRefused, fresh: facesLost })
      .find(({ surface, invariant }) => surface === 'fresh' && invariant === 'rollback_faces')?.passed,
    false,
  );
});

test('empty-intersect sentinel requires exact quality, zero volume, zero faces, and a preserved operand', () => {
  const cases = expandContractMatrix();
  const c = cases.find(({ id }) => id === 'contract/empty-intersect-sentinel');
  assert.equal(c.batch[c.booleanIndex].args.operation, 'intersect');
  assert.equal(c.batch[c.operandVolumeIndex].op, 'volume');
  assert.equal(c.batch[c.operandVolumeIndex].args.solid, 0);
  const sentinel = {
    outcome: 'success', diagnosticCodes: [], quality: 'exact', volume: 0, faceCount: 0, operandVolume: 8,
  };
  const stages = scoreContractCase(c, { native: sentinel, fresh: { ...sentinel }, committed: { ...sentinel } });
  assert.ok(stages.filter(({ required }) => required).every(({ passed }) => passed));
  assert.ok(stages.some(({ invariant }) => invariant === 'operands_preserved'));

  // A surface that refuses the disjoint intersect instead of returning the
  // sentinel disagrees on outcome and diagnostics.
  const refused = { outcome: 'batch_error', diagnosticCodes: ['operation_failed'], rollbackVolumes: [8] };
  const scoredRefused = scoreContractCase(c, { native: sentinel, fresh: refused });
  assert.equal(
    scoredRefused.find(({ surface, invariant }) => surface === 'fresh' && invariant === 'batch_success')?.passed,
    false,
  );
  assert.equal(scoredRefused.find(({ invariant }) => invariant === 'outcome_agreement')?.passed, false);
  assert.equal(scoredRefused.find(({ invariant }) => invariant === 'diagnostics_agreement')?.passed, false);

  // A stray face or a nonzero volume on the sentinel is a finding.
  const strayFace = { ...sentinel, faceCount: 1 };
  assert.equal(
    scoreContractCase(c, { native: sentinel, fresh: strayFace })
      .find(({ surface, invariant }) => surface === 'fresh' && invariant === 'face_count')?.passed,
    false,
  );
  const leaked = { ...sentinel, volume: 1e-6 };
  assert.equal(
    scoreContractCase(c, { native: sentinel, fresh: leaked })
      .find(({ surface, invariant }) => surface === 'fresh' && invariant === 'oracle_volume')?.passed,
    false,
  );
  // An operand that changed underneath the sentinel fails operand preservation.
  const operandLost = { ...sentinel, operandVolume: 7 };
  assert.equal(
    scoreContractCase(c, { native: sentinel, fresh: operandLost })
      .find(({ surface, invariant }) => surface === 'fresh' && invariant === 'operands_preserved')?.passed,
    false,
  );
  const operandMissing = { ...sentinel, operandVolume: null };
  assert.equal(
    scoreContractCase(c, { native: sentinel, fresh: operandMissing })
      .find(({ surface, invariant }) => surface === 'fresh' && invariant === 'operands_preserved')?.passed,
    false,
  );
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
