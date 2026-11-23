'use strict';

// Run from the repository root:
// node scripts/perf/tooling-plate/hole-wire-bench.cjs crates/wasm/pkg 88 3
// Compare 1, 20 and 88 bores with the same Node version, package build options
// and sample count. Each line times addHolesToFace alone on a fresh kernel.
// Pass a baseline row's sha256 as the final argument to check another build.
// In OpenZCAD, import ordinary.openzcad (20 bores) or dense.openzcad (88),
// orbit the loaded plate, then change the recess_depth parameter from 3 to 4 mm.
// The 36 × 16 mm recess removes another 576 mm³ in that edit.

const assert = require('node:assert/strict');
const { createHash } = require('node:crypto');
const { readFileSync } = require('node:fs');
const path = require('node:path');
const { performance } = require('node:perf_hooks');
const { kernelClass } = require('../../kernel-class.cjs');

const [packageDir, boresArg, samplesArg = '3', expectedDigest] = process.argv.slice(2);
const boreCount = Number(boresArg);
const samples = Number(samplesArg);
assert.ok(packageDir && [1, 20, 88].includes(boreCount) && Number.isSafeInteger(samples) && samples >= 1,
  'Expected a kernel package directory, a bore count (1, 20 or 88), and an optional positive sample count');

if (expectedDigest !== undefined) assert.match(expectedDigest, /^[a-f0-9]{64}$/);

const Kernel = kernelClass(require(path.resolve(packageDir, 'remus_wasm_node.cjs')));
const filename = boreCount === 88 ? 'dense.openzcad' : 'ordinary.openzcad';
const { document } = JSON.parse(readFileSync(path.join(__dirname, filename), 'utf8'));
const sketch = Object.values(document.nodes).find(node =>
  node.kind === 'sketch' && node.sketchId === document.sketchOrder[0]);
assert.deepEqual(sketch.planeRef, { type: 'canonical', plane: 'XY', offset: 0 });
const objects = sketch.objectIds.map(id => document.nodes[id].data);
const outline = objects.filter(obj => obj.objectKind === 'line');
const circles = objects.filter(obj => obj.objectKind === 'circle').slice(0, boreCount);
assert.equal(outline.length, 8);
assert.equal(circles.length, boreCount);

// The browser adapter winds each inner circle clockwise, keeping its original
// seam vertex. The seam contributes to the published topology fingerprint.
function holeWire(kernel, { centerX, centerY, radius }) {
  const ccw = kernel.makeCircleEdge(centerX, centerY, 0, 0, 0, 1, radius);
  const [sx, sy, sz] = kernel.evaluateEdgeCurve(ccw, 0);
  const cw = kernel.makeCircleEdgeWithRef(centerX, centerY, 0, 0, 0, -1, radius,
    sx - centerX, sy - centerY, sz);
  return kernel.makeWire(Uint32Array.of(cw), true);
}

// The stock is 160 × 112 × 10 with four 8 mm corner cuts and Ø5 mm bores.
// This benchmark measures the stock before its separate pocket feature.
const expectedVolume = (160 * 112 - 4 * 8 * 8 / 2 - boreCount * Math.PI * 2.5 ** 2) * 10;
let referenceDigest = expectedDigest;
for (let i = 0; i <= samples; i++) {
  const kernel = new Kernel();
  try {
    const edges = outline.map(({ x1, y1, x2, y2 }) => kernel.makeLineEdge(x1, y1, 0, x2, y2, 0));
    const outer = kernel.makeWire(Uint32Array.from(edges), true);
    const face = kernel.makePlanarFaceFromWire(outer);
    const holes = Uint32Array.from(circles, circle => holeWire(kernel, circle));

    const start = performance.now();
    const perforated = kernel.addHolesToFace(face, holes);
    const elapsedMs = performance.now() - start;

    assert.equal(kernel.getFaceWires(perforated).length, 1 + boreCount);
    const blank = kernel.extrude(perforated, 0, 0, 1, 10);
    assert.equal(kernel.validateSolid(blank), 0);
    const volume = kernel.volume(blank, 0.08);
    assert.ok(Math.abs(volume - expectedVolume) < 1e-6, `Stock volume ${volume} mm³`);
    const bytes = kernel.serializeSolids(Uint32Array.of(blank));
    const sha256 = createHash('sha256').update(bytes).digest('hex');
    referenceDigest ??= sha256;
    assert.equal(sha256, referenceDigest);
    console.log(JSON.stringify({ sample: i, warmup: i === 0, boreCount, elapsedMs, volume, sha256 }));
  } finally {
    kernel.free();
  }
}
