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

// 2. Make a box
const boxId = kernel.makeBox(10, 20, 30);
assert.equal(typeof boxId, 'number', 'makeBox should return a number handle');
console.log(`ok - makeBox(10, 20, 30) -> handle ${boxId}`);

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
  "detailed validation should return every counted issue",
);
for (const issue of validation.issues) {
  assert.ok(
    issue.severity === "error" || issue.severity === "warning",
    `unexpected validation severity ${issue.severity}`,
  );
  assert.equal(typeof issue.description, "string");
}
const validationWithOptions = JSON.parse(
  kernel.validateSolidDetailedWithOptions(boxId, 10),
);
assert.equal(
  validationWithOptions.errorCount,
  kernel.validateSolidWithOptions(boxId, 10),
);
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

// 11. Analytic preservation through a boolean, checked on the SHIPPED package.
//
// Fusing two coaxial revolved annuli that share an exact cylindrical wall —
// the OpenZCAD flange demo's "Union flange blank": a rim (r24..45, t10)
// against a hub (r12..24, h26), both walls exactly r24 and overlapping in z.
// The fuse must resolve that coincident cylindrical face pair and stay
// analytic. It regressed to a ~1031-face all-plane mesh fallback, fixed in
// #21 (canonical same-domain key for closed edges).
//
// #21 already carries the native regression test, which is the primary guard.
// This is deliberately a SECOND, different guard: it runs the built, bundled
// wasm package rather than the Rust library. That gap is not theoretical —
// the committed `crates/wasm/pkg` is what OpenZCAD installs, and it shipped
// as 2.129.0 built from a pre-fix commit, so consumers hit a ~2789-face
// flange body with 873 boundary edges while `cargo test` was green. A
// package-level assertion is what catches that class.
//
// Nothing else in this smoke test asserts a boolean keeps curved surfaces,
// and face count is the only reliable signal: a mesh fallback is watertight,
// valid, and close on volume, so none of those expose it.
//
// Narrowed by varying one thing at a time:
//   shared r24 wall + in contact          -> fallback
//   shared r24 wall, tops not coplanar    -> fallback  (coplanarity is not it)
//   inner radius 23 instead of 24         -> analytic  (needs EXACT coincidence)
//   shared r24 wall but z-disjoint        -> analytic  (needs real contact)
//   primitive coaxial cylinders           -> analytic  (no coincident wall pair)
//
// Verified fail-before/pass-after by reverting only same_domain.rs and
// rebuilding: 1031 faces without the fix, 7 with it. Do NOT relax this into
// a pin — a suppressed assertion here is the green-looking blindfold that let
// the stale package ship.
{
  /** Revolve an axial rectangle (x = radius, z = height) a full turn about +Z. */
  const revolveAnnulus = (r0, r1, z0, z1) => {
    const pts = [
      [r0, 0, z0],
      [r1, 0, z0],
      [r1, 0, z1],
      [r0, 0, z1],
    ];
    const edges = pts.map((p, i) => {
      const n = pts[(i + 1) % pts.length];
      return kernel.makeLineEdge(p[0], p[1], p[2], n[0], n[1], n[2]);
    });
    const wire = kernel.makeWire(Uint32Array.from(edges), true);
    const face = kernel.makePlanarFaceFromWire(wire);
    return kernel.revolve(face, 0, 0, 0, 0, 0, 1, 360);
  };

  const rim = revolveAnnulus(24, 45, -10, 0);
  const hub = revolveAnnulus(12, 24, -26, 0);

  // Both operands must be analytic going in, or the assertion below proves
  // nothing about the boolean.
  for (const [label, solid] of [
    ['rim', rim],
    ['hub', hub],
  ]) {
    const kinds = Array.from(kernel.getSolidFaces(solid)).map((f) => kernel.getSurfaceType(f));
    assert.equal(
      kinds.filter((t) => t === 'cylinder').length,
      2,
      `${label} operand should have 2 cylindrical walls, got ${JSON.stringify(kinds)}`,
    );
  }

  const fused = kernel.fuse(rim, hub);
  const faceKinds = Array.from(kernel.getSolidFaces(fused)).map((f) => kernel.getSurfaceType(f));
  const cylinders = faceKinds.filter((t) => t === 'cylinder').length;

  // Face count is the only reliable fallback signal: the mesh fallback is
  // watertight and valid, and its volume is close, so neither exposes it.
  assert.ok(
    faceKinds.length <= 12,
    `coaxial annulus fuse mesh-fell-back: ${faceKinds.length} faces ` +
      `(expected <= 12; native gives 7)`,
  );
  assert.ok(
    cylinders >= 3,
    `coaxial annulus fuse lost its cylindrical walls: ${cylinders} cylinder ` +
      `faces of ${faceKinds.length} (expected >= 3)`,
  );

  const fusedVol = kernel.volume(fused, DEFLECTION);
  const expectedVol = Math.PI * ((45 * 45 - 24 * 24) * 10 + (24 * 24 - 12 * 12) * 26);
  assert.ok(
    Math.abs(fusedVol - expectedVol) < 50,
    `coaxial annulus fuse volume=${fusedVol}, expected ~${expectedVol}`,
  );
  console.log(
    `ok - coaxial annulus fuse stayed analytic: ${faceKinds.length} faces, ` +
      `${cylinders} cylindrical`,
  );
}

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

// 13. OpenZCAD mounting-bracket workflow across typed face evolution.
//
// The application stores the r3 mounting-bore face before rounding the four
// outside bracket corners. Consume the transported/decoded evolution payload
// to find that face on the filleted result, then exercise the two edits that
// exposed the original regression: widen r3 -> r4.8 and shrink r4.8 -> r3.8.
// This intentionally runs through the shipped WASM package and its STEP I/O;
// the native operation and I/O tests retain the matching low-level guards.
{
  const W = 80;
  const D = 40;
  const PLATE_T = 8;
  const MOUNT_X = 16;
  const MOUNT_Y = 20;
  const BRACKET_DEFLECTION = 0.05;
  const FILLETED_VOLUME = 47_360.940_056_943_74;
  const WIDE_VOLUME = 47_008.076_370_092_516;
  const FINAL_VOLUME = WIDE_VOLUME + Math.PI * (4.8 ** 2 - 3.8 ** 2) * PLATE_T;
  const EXPECTED_FINAL_RADII = [3, 3, 3, 3, 3, 3.8, 4, 10];

  const bracketKernel = new BrepKernel();
  const translated = (solid, x, y, z) =>
    bracketKernel.copyAndTransformSolid(solid, [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1]);
  const rotatedX90AndTranslated = (solid, x, y, z) =>
    bracketKernel.copyAndTransformSolid(solid, [1, 0, 0, x, 0, 0, -1, y, 0, 1, 0, z, 0, 0, 0, 1]);
  const fuseUniform = (...solids) => {
    const result = bracketKernel.fuseAll(Uint32Array.from(solids));
    bracketKernel.unifyFaces(result);
    return result;
  };
  const cutUniform = (target, tools) => {
    for (const tool of tools) target = bracketKernel.cut(target, tool);
    bracketKernel.unifyFaces(target);
    return target;
  };
  const analyticParams = (face) => JSON.parse(bracketKernel.getAnalyticSurfaceParams(face));
  const mountingWall = (solid, radius) => {
    const matches = Array.from(bracketKernel.getSolidFaces(solid)).filter((face) => {
      const surface = analyticParams(face);
      return (
        surface.type === 'cylinder' &&
        Math.abs(surface.radius - radius) < 1e-8 &&
        Math.abs(surface.origin[0] - MOUNT_X) < 1e-8 &&
        Math.abs(surface.origin[1] - MOUNT_Y) < 1e-8 &&
        Math.abs(surface.axis[2]) > 1 - 1e-10
      );
    });
    assert.equal(matches.length, 1, `expected one r${radius} mounting-bore wall`);
    return matches[0];
  };
  const assertVolumeAndClosure = (solid, expected, label) => {
    assert.equal(bracketKernel.validateSolid(solid), 0, `${label}: closed, valid shell`);
    const actual = bracketKernel.volume(solid, BRACKET_DEFLECTION);
    const tolerance = Math.max(Math.abs(expected), 1) * 1e-9;
    assert.ok(
      Math.abs(actual - expected) <= tolerance,
      `${label}: volume=${actual}, expected=${expected}, tolerance=${tolerance}`,
    );
  };
  const cylinderRadii = (activeKernel, solid) =>
    Array.from(activeKernel.getSolidFaces(solid))
      .map((face) => JSON.parse(activeKernel.getAnalyticSurfaceParams(face)))
      .filter((surface) => surface.type === 'cylinder')
      .map((surface) => surface.radius)
      .sort((a, b) => a - b);
  const assertExactCylinderRadii = (activeKernel, solid, label) => {
    const actual = cylinderRadii(activeKernel, solid);
    assert.equal(actual.length, EXPECTED_FINAL_RADII.length, `${label}: cylinder count`);
    actual.forEach((radius, index) => {
      assert.ok(
        Math.abs(radius - EXPECTED_FINAL_RADII[index]) < 1e-8,
        `${label}: analytic cylinder radii ${JSON.stringify(actual)}`,
      );
    });
  };

  const base = bracketKernel.makeBox(W, D, PLATE_T);
  const wall = translated(bracketKernel.makeBox(W, PLATE_T, 32), 0, 32, 7.5);
  const blank = fuseUniform(base, wall);

  const boss = rotatedX90AndTranslated(bracketKernel.makeCylinder(10, 12), 40, 34, 24);
  const bossed = fuseUniform(blank, boss);
  const bossBore = rotatedX90AndTranslated(bracketKernel.makeCylinder(4, 48), 40, 48, 24);
  const bored = cutUniform(bossed, [bossBore]);

  const leftMount = translated(bracketKernel.makeCylinder(3, 12), MOUNT_X, MOUNT_Y, -2);
  const rightMount = translated(bracketKernel.makeCylinder(3, 12), W - MOUNT_X, MOUNT_Y, -2);
  const drilled = cutUniform(bored, [leftMount, rightMount]);
  const sourceMountingWall = mountingWall(drilled, 3);

  const cornerEdges = Array.from(bracketKernel.getSolidEdges(drilled)).filter((edge) => {
    const [ax, ay, az, bx, by, bz] = bracketKernel.getEdgeVertices(edge);
    const atCorner = (x, y, z) =>
      (Math.abs(x) < 0.1 || Math.abs(x - W) < 0.1) &&
      (Math.abs(y) < 0.1 || Math.abs(y - D) < 0.1) &&
      z >= -0.1 &&
      z <= 8.1;
    return (
      atCorner(ax, ay, az) &&
      atCorner(bx, by, bz) &&
      Math.abs(ax - bx) <= 1.5 &&
      Math.abs(ay - by) <= 1.5 &&
      Math.abs(az - bz) >= 4
    );
  });
  assert.equal(cornerEdges.length, 4, 'mounting bracket: four outside corner edges');

  const transported = JSON.stringify(
    bracketKernel.filletWithEvolution(drilled, Uint32Array.from(cornerEdges), 3),
  );
  const evolution = decodeEvolutionPayload(transported);
  assertCompleteEvolution(evolution, 'mounting bracket fillet');
  assert.equal(evolution.evolution.provenance, 'construction');
  const descendantClaims = evolution.evolution.modified.filter(
    (claim) => claim.source === sourceMountingWall,
  );
  assert.equal(descendantClaims.length, 1, 'mounting-bore source must have one lineage claim');
  assert.equal(
    descendantClaims[0].results.length,
    1,
    'mounting-bore source must resolve to one descendant face',
  );
  const descendantMountingWall = descendantClaims[0].results[0];
  assert.ok(
    evolution.result.faces.includes(descendantMountingWall),
    'mounting-bore descendant must belong to the filleted solid',
  );
  const descendantSurface = analyticParams(descendantMountingWall);
  assert.equal(descendantSurface.type, 'cylinder', 'mounting-bore descendant stays analytic');
  assert.ok(Math.abs(descendantSurface.radius - 3) < 1e-8, 'mounting-bore descendant stays r3');
  assertVolumeAndClosure(evolution.result.solid, FILLETED_VOLUME, 'filleted bracket');

  const widened = bracketKernel.resizeCylindricalFace(
    evolution.result.solid,
    descendantMountingWall,
    4.8,
  );
  assertVolumeAndClosure(widened, WIDE_VOLUME, 'r3 -> r4.8 bracket');
  const widenedMountingWall = mountingWall(widened, 4.8);
  const narrowed = bracketKernel.resizeCylindricalFace(widened, widenedMountingWall, 3.8);
  assertVolumeAndClosure(narrowed, FINAL_VOLUME, 'r4.8 -> r3.8 bracket');
  assertExactCylinderRadii(bracketKernel, narrowed, 'resized bracket');

  const step = bracketKernel.exportStep(narrowed);
  const stepText = new TextDecoder().decode(step);
  assert.equal(
    stepText.match(/CYLINDRICAL_SURFACE/g)?.length,
    EXPECTED_FINAL_RADII.length,
    'STEP must encode every cylinder analytically',
  );
  const importedKernel = new BrepKernel();
  const imported = Array.from(importedKernel.importStep(step));
  assert.equal(imported.length, 1, 'STEP round trip must yield one bracket solid');
  assert.equal(importedKernel.validateSolid(imported[0]), 0, 'STEP bracket: closed, valid shell');
  const importedVolume = importedKernel.volume(imported[0], BRACKET_DEFLECTION);
  assert.ok(
    Math.abs(importedVolume - FINAL_VOLUME) <= Math.abs(FINAL_VOLUME) * 1e-9,
    `STEP bracket: volume=${importedVolume}, expected=${FINAL_VOLUME}`,
  );
  assertExactCylinderRadii(importedKernel, imported[0], 'STEP bracket');
  console.log('ok - OpenZCAD mounting bracket: decoded lineage, r3 -> r4.8 -> r3.8, exact STEP');
}

console.log('\nAll smoke tests passed');
