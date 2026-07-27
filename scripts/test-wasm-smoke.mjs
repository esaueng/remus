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
    ["rim", rim],
    ["hub", hub],
  ]) {
    const kinds = Array.from(kernel.getSolidFaces(solid)).map((f) =>
      kernel.getSurfaceType(f),
    );
    assert.equal(
      kinds.filter((t) => t === "cylinder").length,
      2,
      `${label} operand should have 2 cylindrical walls, got ${JSON.stringify(kinds)}`,
    );
  }

  const fused = kernel.fuse(rim, hub);
  const faceKinds = Array.from(kernel.getSolidFaces(fused)).map((f) =>
    kernel.getSurfaceType(f),
  );
  const cylinders = faceKinds.filter((t) => t === "cylinder").length;

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
  const expectedVol =
    Math.PI * ((45 * 45 - 24 * 24) * 10 + (24 * 24 - 12 * 12) * 26);
  assert.ok(
    Math.abs(fusedVol - expectedVol) < 50,
    `coaxial annulus fuse volume=${fusedVol}, expected ~${expectedVol}`,
  );
  console.log(
    `ok - coaxial annulus fuse stayed analytic: ${faceKinds.length} faces, ` +
      `${cylinders} cylindrical`,
  );
}

console.log("\nAll smoke tests passed");
