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

// Synthetic arena documents keep shared Line edges and the exact per-use
// authority. Only carriers and their UV charts change; no fitted geometry.
function affineDocument(kernel, solid, everyFace = true, chart = [2, 8, -3, 3]) {
  const doc = JSON.parse(Buffer.from(kernel.serializeSolids(Uint32Array.of(solid))).toString());
  const authority = doc.boundary_authority;
  assert.ok(authority, 'the fixture requires authoritative arena loops/coedges');
  for (const [index, face] of doc.faces.entries()) {
    if (!everyFace && index !== 0) continue;
    const normal = face.surface.Plane.normal;
    const axis = normal.findIndex(x => x !== 0);
    let u = (axis + 1) % 3, v = (axis + 2) % 3;
    if (normal[axis] < 0) [u, v] = [v, u];
    const loops = authority.faces[index];
    const uses = [loops.outer_loop, ...loops.inner_loops]
      .flatMap(l => authority.loops[l].coedges).map(c => authority.coedges[c]);
    const points = uses.flatMap(use => {
      const edge = doc.edges[use.edge];
      return [doc.vertices[edge.start].point, doc.vertices[edge.end].point];
    });
    const low = [0, 1, 2].map(i => Math.min(...points.map(p => p[i])));
    const high = [0, 1, 2].map(i => Math.max(...points.map(p => p[i])));
    const corner = (upperU, upperV) => {
      const p = [...low]; if (upperU) p[u] = high[u]; if (upperV) p[v] = high[v]; return p;
    };
    face.surface = { Nurbs: { degree_u: 1, degree_v: 1,
      knots_u: [chart[0], chart[0], chart[1], chart[1]],
      knots_v: [chart[2], chart[2], chart[3], chart[3]],
      control_points: [[corner(false, false), corner(false, true)],
        [corner(true, false), corner(true, true)]], weights: [[3, 3], [3, 3]] } };
    const uv = p => [chart[0] + (chart[1] - chart[0]) * (p[u] - low[u]) / (high[u] - low[u]),
      chart[2] + (chart[3] - chart[2]) * (p[v] - low[v]) / (high[v] - low[v])];
    for (const use of uses) {
      const edge = doc.edges[use.edge], a = uv(doc.vertices[edge.start].point), b = uv(doc.vertices[edge.end].point);
      const delta = b.map((x, i) => x - a[i]), span = Math.hypot(...delta);
      use.pcurve = { curve: { Line: { origin: a, direction: delta.map(x => x / span) } },
        t_start: use.forward ? 0 : span, t_end: use.forward ? span : 0 };
    }
  }
  return doc;
}
const importDocument = (kernel, doc) => kernel.deserializeSolids(Buffer.from(JSON.stringify(doc)))[0];

export function runAffineMaximumDistancePackaged({ BrepKernel }) {
  let fixtures = 0;
  for (const route of routes) for (const scale of [1e-3, 1, 1e3]) for (const placed of [false, true]) {
    for (const all of [false, true]) {
      const kernel = new BrepKernel();
      try {
        const sourceA = kernel.makeBox(2 * scale, 3 * scale, 4 * scale);
        const sourceB = kernel.makeBox(5 * scale, 2 * scale, scale);
        kernel.transformSolid(sourceB, translation(8 * scale, -4 * scale, 2 * scale));
        const a = importDocument(kernel, affineDocument(kernel, sourceA, all));
        const b = importDocument(kernel, affineDocument(kernel, sourceB));
        // Signed coordinate permutations preserve the represented affine nets
        // exactly, including at decimal scales and translated placements.
        const matrix = placed ? [0, 0, 1, 17 * scale, 1, 0, 0, -11 * scale,
          0, 1, 0, 9 * scale, 0, 0, 0, 1] : translation(0, 0, 0);
        kernel.transformSolid(a, matrix); kernel.transformSolid(b, matrix);
        kernel.checkpoint(); const before = state(kernel, [a, b]);
        const r = query(kernel, route, a, b);
        check(r, Math.sqrt(227) * scale, scale);
        pointClose(r.slice(1, 4), apply(matrix, [0, 3 * scale, 0]), scale);
        pointClose(r.slice(4, 7), apply(matrix, [13 * scale, -4 * scale, 3 * scale]), scale);
        const reverse = query(kernel, route, b, a);
        pointClose(reverse.slice(1, 4), r.slice(4, 7), scale);
        pointClose(reverse.slice(4, 7), r.slice(1, 4), scale);
        check(query(kernel, route, a, a), Math.sqrt(29) * scale, scale);
        assert.deepEqual(state(kernel, [a, b]), before);
        fixtures += 1;
      } finally { kernel.free(); }
    }
  }
  for (const route of routes) for (const fault of ['near-affine', 'weights', 'missing', 'outside', 'bowed', 'chart-roundoff']) {
    const kernel = new BrepKernel();
    try {
      const source = kernel.makeBox(2, 3, 4);
      const doc = affineDocument(kernel, source, true,
        fault === 'chart-roundoff' ? [1e12, 1e12 + 6, -3, 3] : undefined);
      const surface = doc.faces[0].surface.Nurbs;
      const use = doc.boundary_authority.coedges[doc.boundary_authority.loops[0].coedges[0]];
      if (fault === 'near-affine') surface.control_points[1][1][2] += 1e-9;
      if (fault === 'weights') surface.weights[1][1] = 6;
      if (fault === 'missing') delete use.pcurve;
      if (fault === 'outside') use.pcurve.curve.Line.origin = [100, 100];
      if (fault === 'bowed') {
        const p = use.pcurve, line = p.curve.Line;
        const uv = t => line.origin.map((x, i) => x + line.direction[i] * t);
        const a = uv(p.t_start), b = uv(p.t_end);
        use.pcurve = { curve: { Nurbs: { degree: 2, knots: [0, 0, 0, 1, 1, 1],
          control_points: [a, [(a[0] + b[0]) / 2 + 0.1, (a[1] + b[1]) / 2 + 0.1], b],
          weights: [1, 1, 1] } }, t_start: 0, t_end: 1 };
      }
      const a = importDocument(kernel, doc); kernel.checkpoint();
      const before = state(kernel, [a]);
      const unresolved = fault === 'chart-roundoff';
      refuse(kernel, route, a, a, unresolved ? 'maximum-distance:unresolved' : 'maximum-distance:unsupported',
        unresolved ? 'quality_refused' : 'unsupported');
      assert.deepEqual(state(kernel, [a]), before);
      fixtures += 1;
    } finally { kernel.free(); }
  }
  for (const route of routes) for (const scale of [1e-3, 1, 1e3]) {
    const kernel = new BrepKernel();
    try {
      const source = kernel.makeBox(2 * scale, 3 * scale, 4 * scale);
      kernel.transformSolid(source, translation(4 * scale, -3 * scale, scale));
      const a = importDocument(kernel, affineDocument(kernel, source));
      const b = kernel.makeSphere(2 * scale, 16); kernel.checkpoint();
      const before = state(kernel, [a, b]), r = query(kernel, route, a, b);
      check(r, (Math.sqrt(70) + 2) * scale, scale);
      pointClose(r.slice(1, 4), [6 * scale, -3 * scale, 5 * scale], scale);
      close(Math.hypot(...r.slice(4, 7)), 2 * scale, scale);
      const reverse = query(kernel, route, b, a);
      pointClose(reverse.slice(1, 4), r.slice(4, 7), scale);
      pointClose(reverse.slice(4, 7), r.slice(1, 4), scale);
      assert.deepEqual(state(kernel, [a, b]), before); fixtures += 1;
    } finally { kernel.free(); }
  }
  for (const route of routes) for (const scale of [1 / 1024, 1, 1024]) {
    const kernel = new BrepKernel();
    try {
      const x = kernel.makeBox(2 * scale, 3 * scale, 4 * scale);
      const y = kernel.makeBox(5 * scale, 2 * scale, scale);
      kernel.transformSolid(y, translation(8 * scale, -4 * scale, 2 * scale));
      const a = importDocument(kernel, affineDocument(kernel, x));
      const b = importDocument(kernel, affineDocument(kernel, y));
      const matrix = [1, 1, 0, 0, 1, -1, 0, 0, 0, 0, -2, 0, 0, 0, 0, 1];
      kernel.transformSolid(a, matrix); kernel.transformSolid(b, matrix); kernel.checkpoint();
      const before = state(kernel, [a, b]), r = query(kernel, route, a, b);
      check(r, Math.sqrt(472) * scale, scale);
      pointClose(r.slice(1, 4), apply(matrix, [0, 3 * scale, 0]), scale);
      pointClose(r.slice(4, 7), apply(matrix, [13 * scale, -4 * scale, 3 * scale]), scale);
      assert.deepEqual(state(kernel, [a, b]), before); fixtures += 1;
    } finally { kernel.free(); }
  }
  for (const route of routes) for (const cavity of [false, true]) {
    const kernel = new BrepKernel();
    try {
      const outer = kernel.makeBox(8, 6, 4), tool = kernel.makeBox(2, 2, cavity ? 2 : 6);
      kernel.transformSolid(tool, translation(3, 2, cavity ? 1 : -1));
      const source = kernel.cut(outer, tool), doc = affineDocument(kernel, source);
      const a = importDocument(kernel, doc); kernel.checkpoint();
      const before = state(kernel, [a]);
      check(query(kernel, route, a, a), Math.sqrt(116), 1);
      assert.deepEqual(state(kernel, [a]), before); fixtures += 1;
      const solid = doc.solids[doc.solid_roots[0]];
      assert.equal(solid.inner_shells.length, Number(cavity));
      const shell = cavity ? solid.inner_shells[0] : solid.outer_shell;
      const face = doc.shells[shell].faces[0];
      const loop = doc.boundary_authority.faces[face].outer_loop;
      delete doc.boundary_authority.coedges[doc.boundary_authority.loops[loop].coedges[0]].pcurve;
      const invalid = importDocument(kernel, doc), failedBefore = state(kernel, [invalid]);
      refuse(kernel, route, invalid, invalid, 'maximum-distance:unsupported', 'unsupported');
      assert.deepEqual(state(kernel, [invalid]), failedBefore); fixtures += 1;
    } finally { kernel.free(); }
  }
  console.log(`Affine maximum-distance packaged witnesses passed (${fixtures} fixtures; direct/legacy/V2)`);
  return fixtures;
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
  runAffineMaximumDistancePackaged({ BrepKernel });
  return fixtures;
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  assert.ok(process.argv[2], 'pass the actual built or installed remus_wasm.js path');
  runMaximumDistancePackaged(createRequire(import.meta.url)(resolve(process.argv[2])));
}
