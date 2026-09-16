import assert from 'node:assert/strict';
import test from 'node:test';

import { expandMatrix, readManifest, scoreCase } from './parity-matrix.mjs';

test('extended parity matrix covers every family, scale, and placement', () => {
  const { manifest } = readManifest();
  const cases = expandMatrix(manifest);

  const booleanFamilies = manifest.families.filter(({ kind }) => kind === 'boolean');
  const modifierFamilies = manifest.families.filter(({ kind }) => kind !== 'boolean');
  assert.equal(booleanFamilies.length, 16, 'boolean family count');
  assert.equal(modifierFamilies.length, 13, 'modifier/sweep family count');

  // 16 boolean families x 3 ops x 3 scales x 2 placements = 288, plus
  // 13 modifier/sweep families x 3 scales x 2 placements = 78.
  assert.equal(cases.length, 16 * 3 * 3 * 2 + 13 * 3 * 2);
  assert.equal(new Set(cases.map(({ id }) => id)).size, cases.length);

  for (const family of booleanFamilies) {
    for (const operation of manifest.operations) {
      const cells = cases.filter((c) => c.family === family.id && c.operation === operation);
      assert.equal(cells.length, 6, `${family.id}/${operation}`);
      assert.deepEqual(new Set(cells.map(({ scale }) => scale)), new Set(manifest.scales));
      assert.deepEqual(new Set(cells.map(({ placement }) => placement)), new Set(manifest.placements));
    }
  }
  for (const family of modifierFamilies) {
    const cells = cases.filter((c) => c.family === family.id);
    assert.equal(cells.length, 6, family.id);
    assert.deepEqual(new Set(cells.map(({ scale }) => scale)), new Set(manifest.scales));
    assert.deepEqual(new Set(cells.map(({ placement }) => placement)), new Set(manifest.placements));
  }

  // Scales are exactly the 1e-3/1/1e3 decade band the task requires.
  assert.deepEqual(manifest.scales, [0.001, 1, 1000]);

  const covered = new Set(cases.flatMap((c) => c.batch.map(({ op }) => op)));
  for (const op of [
    'makeBox', 'makeCone', 'makeSphere', 'makeCylinder', 'makeTorus', 'transform',
    'booleanWithQuality', 'solidEdges', 'fillet', 'chamfer', 'shell', 'offsetSolid',
    'draft', 'makeLineEdge', 'makeWire', 'makePlanarFaceFromWire', 'extrude', 'revolve',
    'sweep', 'pipe', 'loft', 'helicalSweep',
    'volume', 'validateSolid', 'meshQuality', 'getSolidFaces',
  ]) assert.ok(covered.has(op), `missing batch operation ${op}`);
});

test('scorer requires independent geometry oracles and cross-surface identity', () => {
  const { manifest } = readManifest();
  // Use a scale-1 cell so the 1% oracle perturbation is not absorbed by the
  // max(|expected|, 1) floor in relativeError at 1e-3 scales.
  const c = expandMatrix(manifest).find(({ family, scale }) => family === 'box-box' && scale === 1);
  const observation = {
    outcome: 'success',
    diagnosticCodes: [],
    quality: 'exact',
    volume: c.expected_volume,
    validationErrors: 0,
    census: { faces: 6, edges: 12, vertices: 8 },
    surfaceTypes: { plane: 6 },
    meshQuality: {
      triangleCount: 24,
      boundaryEdges: 0,
      nonManifoldEdges: 0,
      eulerCharacteristic: 2,
      isWatertight: true,
    },
    serializedBytes: 128,
    serializedSha256: 'ab'.repeat(32),
  };
  const stages = scoreCase(c, { native: observation, wasm: structuredClone(observation) });
  assert.ok(stages.every(({ passed }) => passed));

  const wrongOracle = structuredClone(observation);
  wrongOracle.volume *= 1.01;
  const oracleStages = scoreCase(c, { native: observation, wasm: wrongOracle });
  assert.equal(
    oracleStages.find(({ surface, invariant }) => surface === 'wasm' && invariant === 'oracle_volume')?.passed,
    false,
  );
  assert.equal(
    oracleStages.find(({ invariant }) => invariant === 'volume_agreement')?.passed,
    false,
  );

  const wrongBytes = structuredClone(observation);
  wrongBytes.serializedSha256 = 'cd'.repeat(32);
  assert.equal(
    scoreCase(c, { native: observation, wasm: wrongBytes })
      .find(({ invariant }) => invariant === 'serialized_bytes_agreement')?.passed,
    false,
  );
  assert.equal(
    scoreCase(c, { native: observation, wasm: wrongBytes })
      .find(({ invariant }) => invariant === 'serialized_bytes_agreement')?.required,
    false,
  );
});

test('modifier cells assert disclosed quality and bounded oracle volume', () => {
  const { manifest } = readManifest();
  const cases = expandMatrix(manifest);
  const draft = cases.find(({ family, scale }) => family === 'draft-box-wall' && scale === 1);
  assert.ok(draft && draft.expected_volume != null, 'draft cell carries a closed-form oracle');
  const good = {
    outcome: 'success',
    diagnosticCodes: [],
    quality: 'exact',
    volume: draft.expected_volume,
    validationErrors: 0,
    census: { faces: 6, edges: 12, vertices: 8 },
    surfaceTypes: { plane: 6 },
    meshQuality: {
      triangleCount: 12,
      boundaryEdges: 0,
      nonManifoldEdges: 0,
      eulerCharacteristic: 2,
      isWatertight: true,
    },
    serializedBytes: 64,
    serializedSha256: 'ef'.repeat(32),
  };
  assert.ok(scoreCase(draft, { native: good, wasm: structuredClone(good) }).every(({ passed }) => passed));
  const undisclosed = structuredClone(good);
  undisclosed.quality = 'unknown';
  assert.equal(
    scoreCase(draft, { native: good, wasm: undisclosed })
      .find(({ invariant }) => invariant === 'quality_agreement')?.passed,
    false,
  );
});

test('mutual typed refusal asserts disclosure agreement, not geometry', () => {
  const { manifest } = readManifest();
  const cases = expandMatrix(manifest);
  const c = cases.find(({ family }) => family === 'box-cone');
  const refused = {
    outcome: 'batch_error',
    diagnosticCodes: ['invalid_argument', 'invalid_handle'],
  };
  const stages = scoreCase(c, { native: refused, wasm: structuredClone(refused) });
  assert.ok(stages.filter(({ required }) => required).every(({ passed }) => passed));
  assert.equal(
    stages.find(({ invariant }) => invariant === 'diagnostics_agreement')?.passed,
    true,
  );
  const split = structuredClone(refused);
  split.diagnosticCodes = ['operation_failed', 'invalid_handle'];
  assert.equal(
    scoreCase(c, { native: refused, wasm: split })
      .find(({ invariant }) => invariant === 'diagnostics_agreement')?.passed,
    false,
  );
});

test('rigid oracle cells carry the FP-rotation floor', () => {
  const { manifest } = readManifest();
  const cases = expandMatrix(manifest);
  for (const c of cases.filter(({ kind }) => kind !== 'boolean')) {
    const family = manifest.families.find(({ id }) => id === c.family);
    if (family.volume_oracle) {
      if (c.placement === 'rigid') {
        assert.ok(c.oracle_relative_tolerance >= 1e-7, `${c.id}: rigid oracle floor`);
      } else {
        assert.equal(c.oracle_relative_tolerance, family.volume_oracle.relative_tolerance, `${c.id}: origin oracle bound`);
      }
    }
  }
});
