#!/usr/bin/env node
/**
 * Smoke test for the brepkit WASM package.
 * Verifies that the built package loads and basic operations work.
 *
 * Usage: node scripts/test-wasm-smoke.mjs
 */

import assert from "node:assert/strict";
import { createRequire } from "node:module";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = dirname(fileURLToPath(import.meta.url));
const projectRoot = resolve(__dirname, "..");

// Use createRequire to load the CJS node entry from an ESM context.
// The node entry uses CommonJS (exports.X = ...) and is renamed to .cjs
// so Node treats it correctly even with "type": "module" in package.json.
const require = createRequire(import.meta.url);
const { BrepKernel } = require(
  resolve(projectRoot, "crates/wasm/pkg/brepkit_wasm_node.cjs"),
);

const DEFLECTION = 0.1;

// 1. Kernel creation
const kernel = new BrepKernel();
console.log("ok - BrepKernel created");

// 2. Make a box
const boxId = kernel.makeBox(10, 20, 30);
assert.equal(typeof boxId, "number", "makeBox should return a number handle");
console.log(`ok - makeBox(10, 20, 30) -> handle ${boxId}`);

// 3. Volume check
const vol = kernel.volume(boxId, DEFLECTION);
assert.ok(Math.abs(vol - 6000) < 1e-6, `volume=${vol}, expected ~6000`);
console.log(`ok - volume = ${vol}`);

// 4. Tessellation
const mesh = kernel.tessellateSolid(boxId, DEFLECTION);
assert.ok(mesh.positions.length > 0, "mesh should have positions");
assert.ok(mesh.indices.length > 0, "mesh should have indices");
assert.equal(mesh.positions.length % 3, 0, "positions should be a multiple of 3");
assert.equal(mesh.indices.length % 3, 0, "indices should be a multiple of 3");
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
assert.ok(
  Math.abs(props.inertia[0] - 650000) < 1e-3,
  `Ixx=${props.inertia[0]}, expected ~650000`,
);
assert.equal(props.principalAxes.length, 9, "principalAxes should have 9 entries");
console.log(`ok - massProperties: volume=${props.volume}, Ixx=${props.inertia[0]}`);

// 6. Mesh quality
const quality = JSON.parse(kernel.meshQuality(boxId, DEFLECTION));
assert.equal(quality.boundaryEdges, 0, "box mesh should have no boundary edges");
assert.equal(quality.isWatertight, true, "box mesh should be watertight");
console.log(
  `ok - meshQuality: watertight, euler=${quality.eulerCharacteristic}`,
);

// 7. STL export (only if io feature is compiled in)
if (typeof kernel.exportStl === "function") {
  const stl = kernel.exportStl(boxId, DEFLECTION);
  assert.ok(stl.length > 0, "STL export should not be empty");
  console.log(`ok - STL export: ${stl.length} bytes`);
} else {
  console.log("skip - exportStl not available (io feature not enabled)");
}

// 8. PLY round trip (only if io feature is compiled in)
if (typeof kernel.importPly === "function") {
  const ply = kernel.exportPly(boxId, DEFLECTION);
  const reimported = kernel.importPly(ply);
  const vol2 = kernel.volume(reimported, DEFLECTION);
  assert.ok(Math.abs(vol2 - 6000) < 60, `PLY round-trip volume=${vol2}`);
  console.log(`ok - PLY round trip: volume=${vol2}`);
} else {
  console.log("skip - importPly not available (io feature not enabled)");

}

// 9. Direct face editing: push/pull a planar face.
{
  const block = kernel.makeBox(10, 10, 10);
  const faces = Array.from(kernel.getSolidFaces(block));
  let topFace = null;
  for (const f of faces) {
    if (kernel.getSurfaceType(f) !== "plane") continue;
    const n = kernel.getFaceNormal(f);
    if (Math.abs(n[2] - 1) < 1e-6) {
      topFace = f;
      break;
    }
  }
  assert.ok(topFace !== null, "expected a +Z planar face on the box");
  const pulled = kernel.pushPullFace(block, topFace, 5);
  const pulledVol = kernel.volume(pulled, DEFLECTION);
  assert.ok(
    Math.abs(pulledVol - 1500) < 1,
    `pushPullFace volume=${pulledVol}, expected ~1500`,
  );
  console.log(`ok - pushPullFace(+5) -> volume ${pulledVol}`);
}

// 10. Direct face editing: resize a cylindrical bore.
{
  const block = kernel.makeBox(40, 40, 10);
  const drill = kernel.copyAndTransformSolid(kernel.makeCylinder(3, 10), [
    1, 0, 0, 20, 0, 1, 0, 20, 0, 0, 1, 0, 0, 0, 0, 1,
  ]);
  const drilled = kernel.cut(block, drill);
  const bore = Array.from(kernel.getSolidFaces(drilled)).find(
    (f) => kernel.getSurfaceType(f) === "cylinder",
  );
  assert.ok(bore !== undefined, "expected a cylindrical bore face");
  const widened = kernel.resizeCylindricalFace(drilled, bore, 5);
  const widenedVol = kernel.volume(widened, DEFLECTION);
  const expected = 40 * 40 * 10 - Math.PI * 25 * 10;
  assert.ok(
    Math.abs(widenedVol - expected) < 5,
    `resizeCylindricalFace volume=${widenedVol}, expected ~${expected}`,
  );
  console.log(`ok - resizeCylindricalFace(5) -> volume ${widenedVol}`);

  if (typeof kernel.exportStep === "function") {
    const step = kernel.exportStep(widened);
    assert.ok(step.length > 0, "STEP export of the resized bore should not be empty");
    console.log(`ok - resized bore STEP export: ${step.length} bytes`);
  }
}

console.log("\nAll smoke tests passed");
