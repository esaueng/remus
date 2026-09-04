#!/usr/bin/env node
/**
 * Smoke test for the remus WASM packages: the kernel (`remus-wasm`) and the
 * file-format translators (`remus-wasm-io`). Verifies that both load and
 * that bodies cross between them as exact arena documents.
 *
 * Usage: node scripts/test-wasm-smoke.mjs
 */

import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
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
const {
  BrepKernel,
  OperationCancellationToken,
  decodeEvolutionPayload,
} = require(resolve(projectRoot, 'crates/wasm/pkg/remus_wasm_node.cjs'));
const { RemusIo } = require(resolve(projectRoot, 'crates/wasm-io/pkg/remus_wasm_io_node.cjs'));

const DEFLECTION = 0.1;

// The translator module never sees kernel handles: exports take the bytes of
// `serializeSolids`, imports return bytes for `deserializeSolids`.
const io = new RemusIo();
const bodies = (kernel, ...solids) => kernel.serializeSolids(Uint32Array.from(solids));

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
assert.equal(typeof kernel.exportStep, 'undefined', 'translators must not ship in the kernel');
assert.equal(typeof kernel.serializeSolids, 'function', 'arena codec must ship in the kernel');
console.log('ok - RemusIo created; kernel module carries the arena codec only');

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

// JS-controlled work counts must fail before they reach allocation or topology
// construction. This input was previously accepted and created 10,001 edges.
assert.throws(
  () => kernel.makeCircle(1, 10_001),
  /segments must be at most 10000/,
  'makeCircle must reject oversized topology work',
);
assert.equal(typeof kernel.makeCircle(1, 64), 'number');
assert.throws(
  () => kernel.makeSphere(1, 10_001),
  /segments must be at most 10000/,
  'makeSphere must reject oversized topology work',
);
const workCountBatch = JSON.parse(
  kernel.executeBatch(
    JSON.stringify([
      { op: 'makeSphere', args: { radius: 1, segments: 10_001 } },
      { op: 'makeSphere', args: { radius: 1, segments: 64 } },
    ]),
  ),
);
assert.match(workCountBatch[0].error, /segments must be at most 10000/);
assert.equal(typeof workCountBatch[1].ok, 'number');
console.log('ok - oversized WASM work counts fail closed in direct and batch APIs');

// All SSI work budgets are optional additions to the quality-disclosing
// boolean contract. Analytic operands do not enter SSI, so zero budgets must
// preserve the exact result while each malformed positional argument refuses
// before touching the input topology.
{
  const budgetKernel = new BrepKernel();
  const a = budgetKernel.makeBox(2, 2, 2);
  const b = budgetKernel.makeBox(1, 1, 1);
  const bounded = budgetKernel.booleanWithQuality(
    'fuse',
    a,
    b,
    true,
    0,
    0,
    0,
    0,
    0,
    0,
  );
  assert.equal(bounded.quality, 'exact');
  assert.equal(budgetKernel.volume(bounded.solid, DEFLECTION), 8);
  const cancellable = budgetKernel.booleanWithCancellation(
    'fuse',
    a,
    b,
    new OperationCancellationToken(),
    true,
    0,
    0,
    0,
    0,
    0,
    0,
  );
  assert.equal(cancellable.status, 'completed');
  assert.equal(cancellable.result.quality, 'exact');

  const fieldIndexes = new Map([
    ['newton_iterations', 4],
    ['subdivision_depth', 5],
    ['march_steps', 6],
    ['queue_size', 7],
    ['segments', 8],
    ['branches_per_direction', 9],
  ]);
  for (const [field, index] of fieldIndexes) {
    const args = ['fuse', a, b, true];
    args[index] = -1;
    assert.throws(
      () => budgetKernel.booleanWithQuality(...args),
      new RegExp(field),
      `${field} must reject before geometry work`,
    );
    assert.equal(budgetKernel.volume(a, DEFLECTION), 8);
  }
  console.log('ok - direct SSI budget controls are additive and fail closed');
}

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

const planarPairs = JSON.parse(kernel.getOpposingPlanarFacePairs(boxId));
assert.deepEqual(
  planarPairs.map((pair) => pair.distance).sort((a, b) => a - b),
  [10, 20, 30],
);
const widthPair = planarPairs.find((pair) => pair.distance === 10);
assert.ok(widthPair, 'expected a 10-unit box face pair');
assert.equal(widthPair.overlapArea, 600);
const movedBox = kernel.moveFaces(boxId, new Uint32Array([widthPair.faceA]), 2);
assert.ok(Math.abs(kernel.volume(movedBox, DEFLECTION) - 7200) < 1e-6);
const [batchPairs] = JSON.parse(
  kernel.executeBatch(
    JSON.stringify([{ op: 'getOpposingPlanarFacePairs', args: { solid: boxId } }]),
  ),
);
assert.equal(batchPairs.ok.length, 3);
console.log('ok - opposing planar pairs and moveFaces direct edit');

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
assert.ok(quality.triangleCount > 0, 'box mesh should contain triangles');
assert.equal(quality.boundaryEdges, 0, 'box mesh should have no boundary edges');
assert.equal(quality.isWatertight, true, 'box mesh should be watertight');
console.log(`ok - meshQuality: watertight, euler=${quality.eulerCharacteristic}`);

// 7. STL export through the translator module
{
  const stl = io.exportStl(bodies(kernel, boxId), DEFLECTION);
  assert.ok(stl.length > 0, 'STL export should not be empty');
  console.log(`ok - STL export: ${stl.length} bytes`);
}

// 8. PLY round trip through the translator module
{
  const ply = io.exportPly(bodies(kernel, boxId), DEFLECTION);
  const [reimported] = kernel.deserializeSolids(io.importPly(ply));
  const vol2 = kernel.volume(reimported, DEFLECTION);
  assert.ok(Math.abs(vol2 - 6000) < 60, `PLY round-trip volume=${vol2}`);
  console.log(`ok - PLY round trip: volume=${vol2}`);
}

// Successful STEP imports expose bounded, edge-local healing without changing
// the legacy handles-only import contract.
{
  const reportKernel = new BrepKernel();
  const step = readFileSync(
    resolve(projectRoot, 'crates/io/tests/data/shapr_untrimmed_nurbs_domain.step'),
  );
  const imported = io.importStepWithReport(step);
  const report = JSON.parse(imported.report);
  assert.equal(report.solidCount, 1);
  assert.equal(reportKernel.deserializeSolids(imported.solids).length, 1);
  assert.equal(report.diagnostics.length, 2);
  for (const diagnostic of report.diagnostics) {
    assert.equal(diagnostic.code, 'step_untrimmed_nurbs_domain_recovered');
    assert.equal(diagnostic.category, 'tolerance_violation');
    assert.ok(diagnostic.details.edgeCurveEntity > 0);
    assert.ok(Math.abs(diagnostic.details.startParameter - 0.1) < 1e-12);
    assert.ok(Math.abs(diagnostic.details.endParameter - 0.9) < 1e-12);
    assert.ok(diagnostic.details.endpointResidualMm > 1e-7);
    assert.ok(diagnostic.details.endpointResidualMm <= 1e-6);
    assert.equal(
      diagnostic.details.storedEdgeToleranceMm,
      diagnostic.details.endpointResidualMm,
    );
    assert.ok(Math.abs(diagnostic.details.recoveryToleranceCapMm - 1e-6) < 1e-18);
  }
  console.log('ok - STEP bounded-healing report');
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

  const step = io.exportStep(bodies(kernel, widened));
  assert.ok(step.length > 0, 'STEP export of the resized bore should not be empty');
  console.log(`ok - resized bore STEP export: ${step.length} bytes`);
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
  const step = io.exportStep(bodies(evolutionKernel, payload.result.solid));
  assert.ok(step.length > 0, `${method}: STEP export`);
  console.log(`ok - ${method}: typed, complete, exact-geometry parity`);
}

// Offset face identity is construction-derived and reaches both public WASM
// routes as a real journal evolution entry, never a barrier.
{
  const directKernel = new BrepKernel();
  const source = directKernel.makeBox(2, 2, 2);
  const direct = JSON.parse(directKernel.offsetJournaled(source, 0.5));
  assert.ok(Math.abs(directKernel.volume(direct.solid, DEFLECTION) - 27) < 1e-9);
  const directEntry = JSON.parse(directKernel.journalSummary()).at(-1);
  assert.deepEqual(
    {
      kind: directEntry.kind,
      type: directEntry.type,
      origin: directEntry.detail.origin,
      events: directEntry.detail.events,
    },
    { kind: 'offset', type: 'evolution', origin: 'construction', events: 6 },
  );

  const batchKernel = new BrepKernel();
  const batch = JSON.parse(
    batchKernel.executeBatch(
      JSON.stringify([
        { op: 'makeBox', args: { width: 2, height: 2, depth: 2 } },
        { op: 'offsetJournaled', args: { solid: 0, distance: 0.5 } },
        { op: 'journalSummary', args: {} },
      ]),
    ),
  );
  assert.deepEqual(batch[1].ok, direct);
  assert.equal(batch[2].ok.at(-1).kind, 'offset');
  assert.equal(batch[2].ok.at(-1).type, 'evolution');
  assert.equal(batch[2].ok.at(-1).detail.events, 6);
  console.log('ok - offsetJournaled direct/batch construction evolution');
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

// 13. Generic periodic STEP bounds must import as an exact analytic band, and
// an ambiguous band must fail transactionally through the shipped JS binding.
{
  const stepPath = resolve(
    projectRoot,
    'crates/io/tests/data/mambo_b12_period_winding_torus.step',
  );
  const step = readFileSync(stepPath);
  const periodicKernel = new BrepKernel();
  const imported = Array.from(periodicKernel.deserializeSolids(io.importStep(step)));
  assert.equal(imported.length, 1, 'MAMBO B12 STEP must contain one solid');
  const solid = imported[0];
  const exactVolume = 1.25 * Math.PI ** 2;
  assert.ok(Math.abs(periodicKernel.volume(solid, 0.01) - exactVolume) < 1e-9);
  const quality = JSON.parse(periodicKernel.meshQuality(solid, 0.01));
  assert.equal(quality.boundaryEdges, 0);
  assert.equal(quality.nonManifoldEdges, 0);
  assert.equal(quality.isWatertight, true);

  const roundTrip = io.exportStep(bodies(periodicKernel, solid));
  const roundKernel = new BrepKernel();
  const [roundSolid] = Array.from(roundKernel.deserializeSolids(io.importStep(roundTrip)));
  assert.ok(Math.abs(roundKernel.volume(roundSolid, 0.01) - exactVolume) < 1e-9);

  const source = step.toString('utf8');
  const malformed = source.replace(
    "#46=ADVANCED_FACE('',(#50,#51),#52,.T.);",
    "#46=ADVANCED_FACE('',(#50,#50),#52,.T.);",
  );
  assert.notEqual(malformed, source, 'periodic refusal fixture rewrite must apply');
  assert.throws(
    () => io.importStep(Buffer.from(malformed)),
    /the two circles must wind the same periodic axis in opposite directions/,
  );
  console.log('ok - periodic STEP band import, round trip, and typed refusal');
}

// 14. CAx-IF STEP validation properties through the translator module.
{
  const sourceKernel = new BrepKernel();
  const sourceSolid = sourceKernel.makeBox(2, 3, 4);
  const stepBytes = io.exportStepWithOptions(
    bodies(sourceKernel, sourceSolid),
    JSON.stringify({ validationProperties: true }),
  );
  const step = Buffer.from(stepBytes).toString('utf8');
  assert.match(
    step,
    /CAx-IF Rec\.Pracs\.---Geometric and Assembly Validation Properties---4\.6---2023-04-21/,
  );

  const directKernel = new BrepKernel();
  const validated = io.importStepWithValidation(stepBytes, undefined, undefined, undefined);
  const direct = JSON.parse(validated.report);
  assert.equal(direct.solidCount, 1);
  assert.equal(directKernel.deserializeSolids(validated.solids).length, 1);
  assert.equal(direct.diagnostics.length, 0);
  assert.equal(direct.validation.length, 1);
  assert.equal(direct.validation[0].diagnostics.length, 0);
  assert.ok(Math.abs(direct.validation[0].declared.volume - 24) < 1e-9);
  assert.ok(Math.abs(direct.validation[0].recomputed.surfaceArea - 52) < 1e-9);

  const volumeMarker = "'volume measure', VOLUME_MEASURE(";
  const firstVolume = step.indexOf(volumeMarker);
  const geometryVolume = step.indexOf(volumeMarker, firstVolume + volumeMarker.length);
  assert.ok(geometryVolume >= 0, 'geometry-level volume property');
  const volumeStart = geometryVolume + volumeMarker.length;
  const volumeEnd = step.indexOf(')', volumeStart);
  const deviating = `${step.slice(0, volumeStart)}2.4144E1${step.slice(volumeEnd)}`;
  const deviation = JSON.parse(
    io.importStepWithValidation(Buffer.from(deviating), undefined, undefined, undefined).report,
  );
  assert.equal(
    deviation.validation[0].diagnostics[0].code,
    'step_validation_volume_deviation',
  );
  assert.equal(deviation.validation[0].diagnostics[0].category, 'tolerance_violation');

  const areaMarker = "'surface area measure', AREA_MEASURE(";
  const firstArea = step.indexOf(areaMarker);
  const geometryArea = step.indexOf(areaMarker, firstArea + areaMarker.length);
  assert.ok(geometryArea >= 0, 'geometry-level area property');
  const malformed = `${step.slice(0, geometryArea)}'surface area measure', VOLUME_MEASURE(${step.slice(geometryArea + areaMarker.length)}`;
  assert.throws(
    () =>
      io.importStepWithValidation(Buffer.from(malformed), undefined, undefined, undefined),
    /step_validation_invalid_measure/,
  );

  // The kernel module no longer registers the translator batch ops.
  const batch = JSON.parse(
    new BrepKernel().executeBatchV2(
      JSON.stringify([
        { op: 'makeBox', args: { width: 1, height: 2, depth: 3 } },
        { op: 'importStepWithValidation', args: { data: malformed } },
        { op: 'volume', args: { solid: 0, deflection: 0.01 } },
      ]),
    ),
  );
  assert.equal(batch[1].error.code, 'unknown_operation');
  assert.ok(Math.abs(batch[2].ok - 6) < 1e-9);
  console.log('ok - CAx-IF STEP validation properties: translator direct, diagnostics, refusal');
}

// 15. The real Shapr3D hammer-holder contract from the connected-blend work.
// Its radius-3 regions border imported NURBS support geometry, so a nontrivial
// resize is an exact refusal today. The public direct/batch query must agree,
// and the refusal must not silently damage the imported body.
{
  const hammerKernel = new BrepKernel();
  const step = readFileSync(
    resolve(projectRoot, 'crates/io/tests/data/shapr3d_hammer_holder.step'),
  );
  const imported = Array.from(hammerKernel.deserializeSolids(io.importStep(step)));
  assert.equal(imported.length, 1, 'hammer holder STEP must contain one solid');
  const solid = imported[0];
  const faces = Array.from(hammerKernel.getSolidFaces(solid));
  const params = new Map(
    faces.map((face) => [face, JSON.parse(hammerKernel.getAnalyticSurfaceParams(face))]),
  );
  const radiusThreeCylinders = faces.filter((face) => {
    const surface = params.get(face);
    return surface.type === 'cylinder' && Math.abs(surface.radius - 3) < 1e-9;
  });
  assert.equal(radiusThreeCylinders.length, 32);
  assert.equal(faces.filter((face) => params.get(face).type === 'torus').length, 14);

  const seed = radiusThreeCylinders[0];
  assert.throws(
    () => hammerKernel.getBlendRegion(solid, seed),
    /blend band touches a freeform face/,
  );

  const [batchRegion] = JSON.parse(
    hammerKernel.executeBatch(
      JSON.stringify([{ op: 'getBlendRegion', args: { solid, face: seed } }]),
    ),
  );
  assert.match(batchRegion.error, /blend band touches a freeform face/);

  const beforeCounts = Array.from(hammerKernel.getEntityCounts(solid));
  assert.deepEqual(beforeCounts, [160, 386, 238]);
  assert.throws(() => hammerKernel.resizeBlend(solid, seed, 3, 2), /band-touches-freeform/);

  assert.deepEqual(Array.from(hammerKernel.getEntityCounts(solid)), beforeCounts);
  assert.ok(Math.abs(hammerKernel.volume(solid, 0.01) - 50_240.482_852_844_82) <= 0.01);
  const strict = JSON.parse(hammerKernel.validateSolidDetailed(solid));
  assert.equal(strict.errorCount, 0, 'refused edit must preserve strict topology');
  assert.equal(strict.warningCount, 0, 'refused edit must preserve warning-free topology');
  const quality = JSON.parse(hammerKernel.meshQuality(solid, 0.1));
  assert.equal(quality.boundaryEdges, 0);
  assert.equal(quality.nonManifoldEdges, 0);
  assert.equal(quality.isWatertight, true);
  assert.deepEqual(
    faces.map((face) => params.get(face).type).sort(),
    Array.from(hammerKernel.getSolidFaces(solid))
      .map((face) => JSON.parse(hammerKernel.getAnalyticSurfaceParams(face)).type)
      .sort(),
    'refused edit must preserve the face census',
  );
  console.log('ok - real Shapr3D connected-blend refusal is exact and transactional');
}

// 15. OpenZCAD mounting-bracket cylindrical-face resize and STEP round trip.
runOpenZcadCylindricalFaceResizeRegression({ BrepKernel, decodeEvolutionPayload, RemusIo });

console.log('\nAll smoke tests passed');
