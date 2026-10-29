/** Closed-form global maximum witnesses through all three real WASM routes. */
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const routes = ['direct', 'executeBatch', 'executeBatchV2'];
const translation = (x, y, z) => [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];
function pose(scale, placed) {
  if (!placed) return translation(0, 0, 0);
  const c = Math.cos(0.37), s = Math.sin(0.37), d = Math.cos(0.41), t = Math.sin(0.41);
  return [c * d, -s, c * t, 17 * scale, s * d, c, s * t, -11 * scale,
    -t, 0, d, 9 * scale, 0, 0, 0, 1];
}
const apply = (matrix, point) => [0, 1, 2].map(i => matrix[4 * i] * point[0]
  + matrix[4 * i + 1] * point[1] + matrix[4 * i + 2] * point[2] + matrix[4 * i + 3]);
const norm = (a, b) => Math.hypot(...a.map((value, i) => value - b[i]));
const close = (actual, expected, scale) => assert.ok(Math.abs(actual - expected) <= 1e-10 * scale,
  `${actual} != ${expected}`);
const pointClose = (actual, expected, scale) => close(norm(actual, expected), 0, scale);
function query(kernel, route, a, b) {
  if (route === 'direct') return Array.from(kernel.solidToSolidMaximumDistance(a, b));
  const row = JSON.parse(kernel[route](JSON.stringify([
    { op: 'solidToSolidMaximumDistance', args: { solidA: a, solidB: b } },
  ])))[0];
  assert.ok(Object.hasOwn(row, 'ok'), JSON.stringify(row));
  return row.ok;
}
function check(result, expected, scale) {
  assert.equal(result.length, 7);
  assert.ok(result.every(Number.isFinite));
  close(result[0], expected, scale);
  close(norm(result.slice(1, 4), result.slice(4, 7)), expected, scale);
}
function state(kernel, solids) {
  return { geometry: Array.from(kernel.serializeSolids(Uint32Array.from(solids))),
    journal: kernel.journalSummary(), memory: kernel.checkpointMemoryStats() };
}
function refuse(kernel, route, a, b, kernelCode, category) {
  if (route === 'direct') {
    assert.throws(() => kernel.solidToSolidMaximumDistance(a, b), error => {
      assert.ok(!(error instanceof WebAssembly.RuntimeError));
      assert.ok(error.message.startsWith(`${kernelCode}:`), error.message);
      return true;
    });
    return;
  }
  const row = JSON.parse(kernel[route](JSON.stringify([
    { op: 'solidToSolidMaximumDistance', args: { solidA: a, solidB: b } },
  ])))[0];
  assert.ok(Object.hasOwn(row, 'error'), JSON.stringify(row));
  if (route === 'executeBatch') assert.ok(row.error.startsWith(`${kernelCode}:`), row.error);
  else {
    assert.equal(row.error.details.kernelCode, kernelCode);
    assert.equal(row.error.category, category);
  }
}

export function runMaximumDistancePackaged({ BrepKernel }) {
  let fixtures = 0;
  for (const route of routes) for (const scale of [1e-3, 1, 1e3]) for (const placed of [false, true]) {
    const matrix = pose(scale, placed);
    {
      const kernel = new BrepKernel();
      try {
        const a = kernel.makeBox(2 * scale, 3 * scale, 4 * scale);
        const b = kernel.makeBox(5 * scale, 2 * scale, scale);
        kernel.transformSolid(b, translation(8 * scale, -4 * scale, 2 * scale));
        kernel.transformSolid(a, matrix); kernel.transformSolid(b, matrix);
        kernel.checkpoint();
        const before = state(kernel, [a, b]);
        const result = query(kernel, route, a, b);
        check(result, Math.sqrt(227) * scale, scale);
        pointClose(result.slice(1, 4), apply(matrix, [0, 3 * scale, 0]), scale);
        pointClose(result.slice(4, 7), apply(matrix, [13 * scale, -4 * scale, 3 * scale]), scale);
        check(query(kernel, route, a, a), Math.sqrt(29) * scale, scale);
        const reverse = query(kernel, route, b, a);
        pointClose(reverse.slice(1, 4), result.slice(4, 7), scale);
        pointClose(reverse.slice(4, 7), result.slice(1, 4), scale);
        assert.deepEqual(state(kernel, [a, b]), before);
        fixtures += 1;
      } finally { kernel.free(); }
    }
    for (const offset of [[8, 6, 0], [4, 0, 0], [1, 0, 0], [0, 0, 0]]) {
      const kernel = new BrepKernel();
      try {
        const a = kernel.makeSphere(2 * scale, 16), b = kernel.makeSphere(3 * scale, 16);
        kernel.transformSolid(b, translation(...offset.map(x => x * scale)));
        kernel.transformSolid(a, matrix); kernel.transformSolid(b, matrix);
        const before = state(kernel, [a, b]);
        const result = query(kernel, route, a, b);
        check(result, (Math.hypot(...offset) + 5) * scale, scale);
        close(norm(result.slice(1, 4), apply(matrix, [0, 0, 0])), 2 * scale, scale);
        close(norm(result.slice(4, 7), apply(matrix, offset.map(x => x * scale))), 3 * scale, scale);
        check(query(kernel, route, a, a), 4 * scale, scale);
        assert.deepEqual(state(kernel, [a, b]), before);
        fixtures += 1;
      } finally { kernel.free(); }
    }
    {
      const kernel = new BrepKernel();
      try {
        const sphere = kernel.makeSphere(2 * scale, 16), box = kernel.makeBox(2 * scale, 3 * scale, 4 * scale);
        kernel.transformSolid(box, translation(4 * scale, -3 * scale, scale));
        kernel.transformSolid(sphere, matrix); kernel.transformSolid(box, matrix);
        const before = state(kernel, [sphere, box]);
        const result = query(kernel, route, sphere, box);
        check(result, (Math.sqrt(70) + 2) * scale, scale);
        pointClose(result.slice(4, 7), apply(matrix, [6 * scale, -3 * scale, 5 * scale]), scale);
        close(norm(result.slice(1, 4), apply(matrix, [0, 0, 0])), 2 * scale, scale);
        const reverse = query(kernel, route, box, sphere);
        pointClose(reverse.slice(1, 4), result.slice(4, 7), scale);
        pointClose(reverse.slice(4, 7), result.slice(1, 4), scale);
        assert.deepEqual(state(kernel, [sphere, box]), before);
        fixtures += 1;
      } finally { kernel.free(); }
    }
  }
  for (const route of routes) {
    for (const scale of [1e-3,1,1e3]) {
      const kernel=new BrepKernel();
      try {
        const outer=kernel.makeBox(8*scale,6*scale,4*scale),tool=kernel.makeBox(2*scale,2*scale,6*scale);
        kernel.transformSolid(tool,translation(3*scale,2*scale,-scale));
        const hollow=kernel.cut(outer,tool);
        assert.equal(kernel.validateSolid(hollow),0,'through-hole input validates');
        const before=state(kernel,[hollow]);
        check(query(kernel,route,hollow,hollow),Math.sqrt(116)*scale,scale);
        assert.deepEqual(state(kernel,[hollow]),before);
        fixtures+=1;
      } finally {kernel.free();}
    }
    const kernel = new BrepKernel();
    try {
      const box = kernel.makeBox(2, 3, 4), cylinder = kernel.makeCylinder(1, 2, 16);
      const before = state(kernel, [box, cylinder]);
      refuse(kernel, route, box, cylinder, 'maximum-distance:unsupported', 'unsupported');
      assert.deepEqual(state(kernel, [box, cylinder]), before);
      const sphere = kernel.makeSphere(2, 4097);
      const large = state(kernel, [sphere]);
      refuse(kernel, route, sphere, sphere, 'maximum-distance:resource-limit', 'resource_limit');
      assert.deepEqual(state(kernel, [sphere]), large);
      fixtures += 2;
    } finally { kernel.free(); }
  }
  console.log(`Maximum-distance packaged witnesses passed (${fixtures} fixtures; direct/legacy/V2)`);
  return fixtures;
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  assert.ok(process.argv[2], 'pass the actual built or installed remus_wasm.js path');
  runMaximumDistancePackaged(createRequire(import.meta.url)(resolve(process.argv[2])));
}
