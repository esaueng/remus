#!/usr/bin/env node
/**
 * Smoke test for the brepkit WASM package.
 * Verifies that the built package loads and basic operations work.
 *
 * Usage: node scripts/test-wasm-smoke.mjs
 */

import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import {
  runOpenZcadAnalyticFlangeBooleanRegression,
  runOpenZcadCylindricalFaceResizeRegression,
} from './openzcad-wasm-consumer-regressions.mjs';

const __dirname = dirname(fileURLToPath(import.meta.url));
const projectRoot = resolve(__dirname, '..');

// Use createRequire to load the CJS node entry from an ESM context.
// The node entry uses CommonJS (exports.X = ...) and is renamed to .cjs
// so Node treats it correctly even with "type": "module" in package.json.
const require = createRequire(import.meta.url);
const { BrepKernel, decodeEvolutionPayload } = require(
  resolve(projectRoot, 'crates/wasm/pkg/brepkit_wasm_node.cjs'),
);

const DEFLECTION = 0.1;

const assertCompleteEvolution = (payload, label) => {
  assert.equal(payload.schemaVersion, 1, `${label}: schema version`);
  const source = new Set(payload.source.faces);
  const result = new Set(payload.result.faces);
  const accountedSources = new Set([
    ...payload.evolution.modified.map((claim) => claim.source),
    ...payload.evolution.deleted,
    ...payload.evolution.unresolvedSources,
  ]);
  const accountedResults = new Set([
    ...payload.evolution.modified.flatMap((claim) => claim.results),
    ...payload.evolution.generated.flatMap((claim) => claim.results),
    ...payload.evolution.unresolvedResults.map((claim) => claim.result),
  ]);
  assert.deepEqual(accountedSources, source, `${label}: source coverage`);
  assert.deepEqual(accountedResults, result, `${label}: result coverage`);
};

// 1. Kernel creation
const kernel = new BrepKernel();
console.log('ok - BrepKernel created');

// Stable batch-v2 errors are additive: successful envelopes match v1, while
// v1 keeps its string error and v2 exposes the same text plus code/details.
{
  const input = JSON.stringify([
    { op: 'makeBox', args: { width: 2, height: 3, depth: 4 } },
    { op: 'volume', args: { solid: 0, deflection: 0.1 } },
    { op: 'volume', args: { solid: 99, deflection: 0.1 } },
  ]);
  const legacy = JSON.parse(new BrepKernel().executeBatch(input));
  const v2 = JSON.parse(new BrepKernel().executeBatchV2(input));
  assert.deepEqual(v2.slice(0, 2), legacy.slice(0, 2));
  assert.equal(typeof legacy[2].error, 'string');
  assert.equal(v2[2].error.code, 'invalid_handle');
  assert.equal(v2[2].error.message, legacy[2].error);
  assert.deepEqual(v2[2].error.details, {
    entity: 'solid',
    index: 99,
    operation: 'volume',
    operationIndex: 2,
  });
  console.log('ok - executeBatchV2 stable error envelope');
}

// 2. Make a box
const boxId = kernel.makeBox(10, 20, 30);
assert.equal(typeof boxId, 'number', 'makeBox should return a number handle');
console.log(`ok - makeBox(10, 20, 30) -> handle ${boxId}`);

// Batch deflection validation must reject every value rejected by the matching
// direct binding. JSON.stringify serializes NaN and Infinity as null, which the
// batch API must reject rather than silently replacing with its default.
{
  const deflectionOperations = [
    {
      name: 'volume',
      direct: (deflection) => kernel.volume(boxId, deflection),
      args: { solid: boxId },
    },
    {
      name: 'surfaceArea',
      direct: (deflection) => kernel.surfaceArea(boxId, deflection),
      args: { solid: boxId },
    },
    {
      name: 'centerOfMass',
      direct: (deflection) => kernel.centerOfMass(boxId, deflection),
      args: { solid: boxId },
    },
    {
      name: 'meshQuality',
      direct: (deflection) => kernel.meshQuality(boxId, deflection),
      args: { solid: boxId },
    },
    {
      name: 'projectEdges',
      direct: (deflection) =>
        kernel.projectEdges(boxId, 0, 0, 0, 0, 0, 1, 1, 0, 0, true, deflection),
      args: {
        solid: boxId,
        originX: 0,
        originY: 0,
        originZ: 0,
        dirX: 0,
        dirY: 0,
        dirZ: 1,
        xAxisX: 1,
        xAxisY: 0,
        xAxisZ: 0,
        hiddenLines: true,
      },
    },
  ];

  for (const deflection of [NaN, Infinity, 0, -0.1]) {
    for (const operation of deflectionOperations) {
      assert.throws(
        () => operation.direct(deflection),
        undefined,
        `${operation.name} direct binding must reject deflection ${deflection}`,
      );
      const [batchResult] = JSON.parse(
        kernel.executeBatch(
          JSON.stringify([
            {
              op: operation.name,
              args: { ...operation.args, deflection },
            },
          ]),
        ),
      );
      assert.equal(
        typeof batchResult.error,
        'string',
        `${operation.name} batch binding must reject deflection ${deflection}`,
      );
    }
  }
  console.log('ok - direct/batch deflection validation parity');
}

// 3. Volume check
const vol = kernel.volume(boxId, DEFLECTION);
assert.ok(Math.abs(vol - 6000) < 1e-6, `volume=${vol}, expected ~6000`);
console.log(`ok - volume = ${vol}`);

// Detailed validation must preserve every operations-layer diagnostic while
// leaving the existing numeric validator unchanged.
const validation = JSON.parse(kernel.validateSolidDetailed(boxId));
assert.equal(validation.errorCount, kernel.validateSolid(boxId));
assert.equal(
  validation.issues.length,
  validation.errorCount + validation.warningCount,
  'detailed validation should return every counted issue',
);
for (const issue of validation.issues) {
  assert.ok(
    issue.severity === 'error' || issue.severity === 'warning',
    `unexpected validation severity ${issue.severity}`,
  );
  assert.equal(typeof issue.description, 'string');
}
const validationWithOptions = JSON.parse(kernel.validateSolidDetailedWithOptions(boxId, 10));
assert.equal(validationWithOptions.errorCount, kernel.validateSolidWithOptions(boxId, 10));
console.log(
  `ok - detailed validation: ${validation.errorCount} errors, ` +
    `${validation.warningCount} warnings`,
);

// 4. Tessellation
const mesh = kernel.tessellateSolid(boxId, DEFLECTION);
assert.ok(mesh.positions.length > 0, 'mesh should have positions');
assert.ok(mesh.indices.length > 0, 'mesh should have indices');
assert.equal(mesh.positions.length % 3, 0, 'positions should be a multiple of 3');
assert.equal(mesh.indices.length % 3, 0, 'indices should be a multiple of 3');
console.log(
  `ok - tessellation: ${mesh.positions.length / 3} verts, ${mesh.indices.length / 3} tris`,
);

// 5. Mass properties
const props = JSON.parse(kernel.massProperties(boxId));
assert.ok(
  Math.abs(props.volume - 6000) < 1e-6,
  `massProperties.volume=${props.volume}, expected ~6000`,
);
// 10x20x30 box about its CoM: Ixx = m/12 * (20^2 + 30^2) = 650000.
assert.ok(Math.abs(props.inertia[0] - 650000) < 1e-3, `Ixx=${props.inertia[0]}, expected ~650000`);
assert.equal(props.principalAxes.length, 9, 'principalAxes should have 9 entries');
console.log(`ok - massProperties: volume=${props.volume}, Ixx=${props.inertia[0]}`);

// 6. Mesh quality
const quality = JSON.parse(kernel.meshQuality(boxId, DEFLECTION));
assert.equal(quality.boundaryEdges, 0, 'box mesh should have no boundary edges');
assert.equal(quality.isWatertight, true, 'box mesh should be watertight');
console.log(`ok - meshQuality: watertight, euler=${quality.eulerCharacteristic}`);

// 7. STL export (only if io feature is compiled in)
if (typeof kernel.exportStl === 'function') {
  const stl = kernel.exportStl(boxId, DEFLECTION);
  assert.ok(stl.length > 0, 'STL export should not be empty');
  console.log(`ok - STL export: ${stl.length} bytes`);
} else {
  console.log('skip - exportStl not available (io feature not enabled)');
}

// 8. PLY round trip (only if io feature is compiled in)
if (typeof kernel.importPly === 'function') {
  const ply = kernel.exportPly(boxId, DEFLECTION);
  const reimported = kernel.importPly(ply);
  const vol2 = kernel.volume(reimported, DEFLECTION);
  assert.ok(Math.abs(vol2 - 6000) < 60, `PLY round-trip volume=${vol2}`);
  console.log(`ok - PLY round trip: volume=${vol2}`);
} else {
  console.log('skip - importPly not available (io feature not enabled)');
}

// 9. Direct face editing: push/pull a planar face.
{
  const block = kernel.makeBox(10, 10, 10);
  const faces = Array.from(kernel.getSolidFaces(block));
  let topFace = null;
  for (const f of faces) {
    if (kernel.getSurfaceType(f) !== 'plane') continue;
    const n = kernel.getFaceNormal(f);
    if (Math.abs(n[2] - 1) < 1e-6) {
      topFace = f;
      break;
    }
  }
  assert.ok(topFace !== null, 'expected a +Z planar face on the box');
  const pulled = kernel.pushPullFace(block, topFace, 5);
  const pulledVol = kernel.volume(pulled, DEFLECTION);
  assert.ok(Math.abs(pulledVol - 1500) < 1, `pushPullFace volume=${pulledVol}, expected ~1500`);
  console.log(`ok - pushPullFace(+5) -> volume ${pulledVol}`);
}

// 10. Direct face editing: resize a cylindrical bore.
{
  const block = kernel.makeBox(40, 40, 10);
  const drill = kernel.copyAndTransformSolid(
    kernel.makeCylinder(3, 10),
    [1, 0, 0, 20, 0, 1, 0, 20, 0, 0, 1, 0, 0, 0, 0, 1],
  );
  const drilled = kernel.cut(block, drill);
  const bore = Array.from(kernel.getSolidFaces(drilled)).find(
    (f) => kernel.getSurfaceType(f) === 'cylinder',
  );
  assert.ok(bore !== undefined, 'expected a cylindrical bore face');
  const widened = kernel.resizeCylindricalFace(drilled, bore, 5);
  const widenedVol = kernel.volume(widened, DEFLECTION);
  const expected = 40 * 40 * 10 - Math.PI * 25 * 10;
  assert.ok(
    Math.abs(widenedVol - expected) < 5,
    `resizeCylindricalFace volume=${widenedVol}, expected ~${expected}`,
  );
  console.log(`ok - resizeCylindricalFace(5) -> volume ${widenedVol}`);

  if (typeof kernel.exportStep === 'function') {
    const step = kernel.exportStep(widened);
    assert.ok(step.length > 0, 'STEP export of the resized bore should not be empty');
    console.log(`ok - resized bore STEP export: ${step.length} bytes`);
  }
}

// 11. Analytic preservation through an OpenZCAD-shaped flange boolean.
runOpenZcadAnalyticFlangeBooleanRegression({ BrepKernel });

// 12. Versioned fillet/chamfer evolution contract on the shipped package.
//
// Run the ordinary and evolution entry points in fresh kernels so their arena
// allocation is identical, then compare serialized B-Reps byte-for-byte. This
// pins the requirement that provenance does not change exact geometry,
// topology post-processing, tolerances, or engine selection.
for (const operation of ['fillet', 'chamfer']) {
  const plainKernel = new BrepKernel();
  const plainSource = plainKernel.makeBox(10, 10, 10);
  const plainEdge = plainKernel.getSolidEdges(plainSource)[0];
  const plainResult = plainKernel[operation](plainSource, Uint32Array.of(plainEdge), 1);

  const evolutionKernel = new BrepKernel();
  const evolutionSource = evolutionKernel.makeBox(10, 10, 10);
  const evolutionEdge = evolutionKernel.getSolidEdges(evolutionSource)[0];
  const method = `${operation}WithEvolution`;
  const payload = evolutionKernel[method](evolutionSource, Uint32Array.of(evolutionEdge), 1);

  assert.equal(typeof payload, 'object', `${method} must return a typed object`);
  assert.equal(payload.source.solid, evolutionSource, `${method}: source solid`);
  assert.equal(payload.result.solid >= 0, true, `${method}: result solid`);
  assert.equal(payload.evolution.provenance, 'construction', `${method}: box provenance`);
  assert.equal(payload.evolution.unresolvedSources.length, 0, `${method}: sources`);
  assert.equal(payload.evolution.unresolvedResults.length, 0, `${method}: results`);
  assert.ok(payload.evolution.generated.length > 0, `${method}: generated face`);
  assertCompleteEvolution(payload, method);

  const decoded = decodeEvolutionPayload(JSON.stringify(payload));
  assert.deepEqual(decoded, payload, `${method}: decoder round trip`);
  assert.deepEqual(
    evolutionKernel.serializeSolid(payload.result.solid),
    plainKernel.serializeSolid(plainResult),
    `${method}: evolution path changed the exact serialized B-Rep`,
  );

  const generatedFaces = new Set(payload.evolution.generated.flatMap((claim) => claim.results));
  const generatedSurfaceTypes = [...generatedFaces].map((face) =>
    evolutionKernel.getSurfaceType(face),
  );
  if (operation === 'fillet') {
    assert.ok(
      generatedSurfaceTypes.includes('cylinder'),
      `${method}: generated blend face must remain analytic`,
    );
  } else {
    assert.ok(
      generatedSurfaceTypes.every((surface) => surface === 'plane'),
      `${method}: generated bevel faces must remain planar`,
    );
  }

  const quality = JSON.parse(evolutionKernel.meshQuality(payload.result.solid, DEFLECTION));
  assert.equal(quality.isWatertight, true, `${method}: watertight mesh`);
  assert.ok(
    evolutionKernel.volume(payload.result.solid, DEFLECTION) > 0,
    `${method}: positive volume`,
  );
  const step = evolutionKernel.exportStep(payload.result.solid);
  assert.ok(step.length > 0, `${method}: STEP export`);
  console.log(`ok - ${method}: typed, complete, exact-geometry parity`);
}

// A stored/transported payload is untrusted input: malformed versions,
// incomplete coverage and contradictory result claims must fail closed.
{
  const source = kernel.makeBox(10, 10, 10);
  const edge = kernel.getSolidEdges(source)[0];
  const payload = kernel.filletWithEvolution(source, Uint32Array.of(edge), 1);

  const badVersion = structuredClone(payload);
  badVersion.schemaVersion = 2;
  assert.throws(
    () => decodeEvolutionPayload(JSON.stringify(badVersion)),
    /unsupported face evolution schema version/,
  );

  const contradictory = structuredClone(payload);
  contradictory.evolution.generated[0].results = [contradictory.evolution.modified[0].results[0]];
  assert.throws(
    () => decodeEvolutionPayload(JSON.stringify(contradictory)),
    /contradictory claims/,
  );

  const failureKernel = new BrepKernel();
  const failureSource = failureKernel.makeBox(10, 10, 10);
  const failureEdge = failureKernel.getSolidEdges(failureSource)[0];
  const before = failureKernel.volume(failureSource, DEFLECTION);
  assert.throws(
    () => failureKernel.filletWithEvolution(failureSource, Uint32Array.of(failureEdge), 0),
    /radius|fillet|blend/i,
  );
  assert.ok(
    Math.abs(failureKernel.volume(failureSource, DEFLECTION) - before) < 1e-9,
    'failed evolution fillet must leave the input unchanged',
  );
  console.log('ok - evolution decoder and degenerate-operation rejection');
}

// 13. OpenZCAD mounting-bracket cylindrical-face resize and STEP round trip.
runOpenZcadCylindricalFaceResizeRegression({ BrepKernel, decodeEvolutionPayload });

console.log('\nAll smoke tests passed');
