import assert from 'node:assert/strict';
import test from 'node:test';

import { expandMatrix, readManifest, scoreCase } from './quadric-boolean.mjs';

test('qualified quadric matrix expands every axis exactly once', () => {
  const { manifest } = readManifest();
  const cases = expandMatrix(manifest);
  assert.equal(cases.length, 54);
  assert.equal(new Set(cases.map(({ id }) => id)).size, 54);

  for (const family of manifest.families) {
    for (const operation of manifest.operations) {
      const cells = cases.filter((c) => c.family === family.id && c.operation === operation);
      assert.equal(cells.length, 6, `${family.id}/${operation}`);
      assert.deepEqual(new Set(cells.map(({ scale }) => scale)), new Set(manifest.scales));
      assert.deepEqual(new Set(cells.map(({ placement }) => placement)), new Set(manifest.placements));
    }
  }

  const covered = new Set(cases.flatMap((c) => c.batch.map(({ op }) => op)));
  for (const op of [
    'makeCone', 'makeSphere', 'makeCylinder', 'makeTorus', 'transform',
    'booleanWithQuality', 'volume', 'validateSolid', 'meshQuality', 'getSolidFaces',
  ]) assert.ok(covered.has(op), `missing batch operation ${op}`);
});

test('scorer requires independent geometry oracles and cross-surface identity', () => {
  const { manifest } = readManifest();
  const c = expandMatrix(manifest)[0];
  const surfaceTypes = Object.fromEntries(c.required_surface_types.map((type) => [type, 1]));
  const faceCount = Object.values(surfaceTypes).reduce((sum, count) => sum + count, 0);
  const observation = {
    outcome: 'success',
    diagnosticCodes: [],
    quality: 'exact',
    volume: c.expected_volume,
    validationErrors: 0,
    census: { faces: faceCount, edges: 8, vertices: 4 },
    surfaceTypes,
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
