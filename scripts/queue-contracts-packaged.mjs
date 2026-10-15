/** Packaged regressions for the reviewed JSON BREP, batch, sketch, sweep and mesh-import contracts.
 * Runs over the optimized wasm32 pair, including through installed npm tarballs.
 * Usage: node scripts/queue-contracts-packaged.mjs
 */
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const bytes = (text) => new TextEncoder().encode(text);
const close = (actual, expected, label, tolerance = 1e-8) => {
  assert.ok(Number.isFinite(actual) && Math.abs(actual - expected) <= tolerance,
    `${label}: got ${actual}, expected ${expected}`);
};
const bounds = (kernel, solid, expected, label) => {
  const actual = Array.from(kernel.boundingBox(solid));
  assert.equal(actual.length, 6, `${label}: bbox dimensions`);
  actual.forEach((value, index) => close(value, expected[index], `${label}: bbox[${index}]`));
};
const state = (kernel, solids) => {
  const usage = JSON.parse(kernel.checkpointMemoryStats());
  assert.equal(usage.checkpoints, 0, 'live-count oracle requires no retained checkpoint snapshots');
  return {
    liveSlots: usage.allocatedSlots - usage.retiredSlots,
    geometry: Array.from(kernel.serializeSolids(Uint32Array.from(solids))),
    journal: kernel.journalSummary(),
  };
};
const regularError = (operation, expression, label) => assert.throws(operation,
  (error) => !(error instanceof WebAssembly.RuntimeError) && expression.test(String(error.message)), label);
const batch = (kernel, route, op, args) => JSON.parse(kernel[route](JSON.stringify([{ op, args }])))[0];
const okay = (row, label) => {
  assert.ok(Object.hasOwn(row, 'ok') && !Object.hasOwn(row, 'error'), `${label}: ${JSON.stringify(row)}`);
  return row.ok;
};
const clone = (value) => JSON.parse(JSON.stringify(value));

// Same independent unit-cube fixture as json_brep_id_tests; document IDs differ from arena IDs.
function cubeBrep() {
  const vertices = Array.from({ length: 8 }, (_, i) => ({ id: 10 + i,
    position: [i & 1, (i >> 1) & 1, (i >> 2) & 1] }));
  const sides = [
    [[0, 0, -1], 0, [0, 2, 3, 1]], [[0, 0, 1], 1, [4, 5, 7, 6]],
    [[0, -1, 0], 0, [0, 1, 5, 4]], [[0, 1, 0], 1, [2, 6, 7, 3]],
    [[-1, 0, 0], 0, [0, 4, 6, 2]], [[1, 0, 0], 1, [1, 3, 7, 5]],
  ];
  const pairs = [];
  const faces = sides.map(([normal, d, corners]) => {
    const outerWireEdges = [], outerWireOrientations = [];
    corners.forEach((a, i) => {
      const b = corners[(i + 1) % 4], key = [Math.min(a, b), Math.max(a, b)];
      let index = pairs.findIndex((pair) => pair[0] === key[0] && pair[1] === key[1]);
      if (index < 0) { index = pairs.length; pairs.push(key); }
      outerWireEdges.push(100 + index); outerWireOrientations.push(a < b);
    });
    return { outerWireEdges, outerWireOrientations, surfaceType: 'plane',
      surfaceParams: { normal, d }, reversed: false, innerWires: [] };
  });
  return { vertices, edges: pairs.map(([a, b], i) => ({ id: 100 + i,
    curveType: 'line', startVertex: 10 + a, endVertex: 10 + b })), faces };
}
function brepWitness({ BrepKernel }) {
  const kernel = new BrepKernel();
  try {
    const retained = kernel.makeBox(2, 3, 4), doc = cubeBrep();
    const control = kernel.fromBREP(JSON.stringify(doc));
    close(kernel.volume(control, 0.01), 1, 'fromBREP positive control');
    const variants = [
      ['duplicate vertex', (d) => d.vertices.push(clone(d.vertices[0])), /vertex id .*duplicate/],
      ['duplicate edge', (d) => d.edges.push(clone(d.edges[0])), /edge id .*duplicate/],
      ['overflow declaration', (d) => { d.vertices[0].id += 2 ** 32; }, /outside the u32/],
      ['overflow reference', (d) => { d.edges[0].startVertex += 2 ** 32; }, /outside the u32/],
      ['unknown inner edge', (d) => { d.faces[0].innerWires = [[999]]; }, /edge id 999 is unknown/],
      ['malformed inner wire', (d) => { d.faces[0].innerWires = ['x']; }, /inner.*wire/i],
    ];
    for (const [label, mutate, expected] of variants) {
      const bad = clone(doc); mutate(bad); const before = state(kernel, [retained, control]);
      regularError(() => kernel.fromBREP(JSON.stringify(bad)), expected, label);
      assert.deepEqual(state(kernel, [retained, control]), before, `${label}: live state atomicity`);
    }
    close(kernel.volume(retained, 0.01), 24, 'retained solid after BREP refusals');
  } finally { kernel.free(); }
}
function optionalArgumentsWitness({ BrepKernel }) {
  for (const route of ['executeBatch', 'executeBatchV2']) {
    const kernel = new BrepKernel();
    try {
      const solid = kernel.makeBox(2, 3, 4), other = kernel.makeBox(1, 1, 1);
      for (const [op, args, argument] of [
        ['mirror', { solid, nx: '0' }, 'nx'],
        ['makeSphere', { radius: 1, segments: 1.5 }, 'segments'],
        ['fuseWithOptions', { solidA: solid, solidB: other, unifyFaces: 'false' }, 'unifyFaces'],
      ]) {
        const before = state(kernel, [solid, other]); const row = batch(kernel, route, op, args);
        assert.ok(row.error, `${route} ${op}: malformed optional must refuse`);
        if (route === 'executeBatchV2') {
          assert.equal(row.error.code, 'invalid_argument');
          assert.equal(row.error.details.argument, argument);
        } else assert.equal(typeof row.error, 'string');
        assert.deepEqual(state(kernel, [solid, other]), before, `${route} ${op}: atomic`);
      }
      const omitted = okay(batch(kernel, route, 'mirror', { solid }), 'omitted default');
      const nullable = okay(batch(kernel, route, 'mirror', { solid, nx: null }), 'null default');
      bounds(kernel, omitted, [-2, 0, 0, 0, 3, 4], `${route} omitted mirror`);
      assert.deepEqual(Array.from(kernel.boundingBox(nullable)), Array.from(kernel.boundingBox(omitted)));
    } finally { kernel.free(); }
  }
}
function legacySketchWitness({ BrepKernel }) {
  const kernel = new BrepKernel();
  try {
    const sketch = kernel.sketchNew(), points = [[0, 0], [1, 0], [0, 1], [1, 1]];
    for (const [x, y] of points) kernel.sketchAddPoint(sketch, x, y, true);
    kernel.sketchAddArc(sketch, 0, 1, 2);
    const dof = kernel.sketchDof(sketch);
    for (const constraint of [
      { type: 'angle', p1: 0, p2: 1, p3: -1, p4: 3, value: 0 },
      { type: 'angle', p1: 0, p2: 1, p3: '2', p4: 3, value: 0 },
      { type: 'angle', p1: 0, p2: 1, p3: 2, value: 0 },
      { type: 'pointOnArc', point: 1, arc: 9 },
    ]) {
      regularError(() => kernel.sketchAddConstraint(sketch, JSON.stringify(constraint)),
        /invalid input|invalid.*handle|sketch arc/i, JSON.stringify(constraint));
      assert.equal(kernel.sketchDof(sketch), dof, 'refusal preserves DOF');
      assert.deepEqual(JSON.parse(kernel.sketchSolve(sketch, 20, 1e-8)).points, points);
    }
    kernel.sketchAddConstraint(sketch, JSON.stringify({ type: 'angle', p1: 0, p2: 1,
      p3: null, p4: null, value: 0 }));
    assert.equal(JSON.parse(kernel.sketchSolve(sketch, 20, 1e-8)).converged, true);
  } finally { kernel.free(); }
}
function ellipseWitness({ BrepKernel }) {
  for (const a of [5, 3]) for (const route of ['executeBatch', 'executeBatchV2']) {
    const kernel = new BrepKernel();
    try {
      const ops = [
        { op: 'gcsNew', args: {} },
        { op: 'gcsAddPoint', args: { sketch: 0, x: 0, y: 0, fixed: true } },
        { op: 'gcsAddEllipse', args: { sketch: 0, center: 0, a, b: 1, angle: 0 } },
        ...[{ type: 'ellipseAxisB', ellipse: 0, value: 1 },
          { type: 'ellipseAngle', ellipse: 0, value: 0 }].map((constraint) =>
          ({ op: 'gcsAddConstraint', args: { sketch: 0, constraint } })),
        { op: 'gcsAddPoint', args: { sketch: 0, x: 1, y: 0, fixed: true } },
        { op: 'gcsAddConstraint', args: { sketch: 0, constraint:
          { type: 'pointOnEllipse', point: 1, ellipse: 0 } } },
        { op: 'gcsSolve', args: { sketch: 0, maxIterations: 200, tolerance: 1e-10 } },
        { op: 'gcsEllipseParams', args: { sketch: 0, ellipse: 0 } },
      ];
      const result = JSON.parse(kernel[route](JSON.stringify(ops)));
      result.slice(0, 7).forEach((row, i) => okay(row, `ellipse setup ${i}`));
      const params = okay(result[8], 'ellipse readback');
      if (a === 5) {
        assert.equal(okay(result[7], 'negative-axis solve').converged, true);
        close(params[2], 1, 'negative semiaxis normalized', 1e-9);
      } else {
        assert.ok(result[7].error, 'collapsed ellipse must refuse');
        const error = result[7].error;
        if (route === 'executeBatchV2') assert.equal(error.code, 'invalid_argument');
        assert.match(typeof error === 'string' ? error : error.message, /collapsed an ellipse semiaxis/);
        assert.equal(params[2], 3, 'collapsed ellipse restores pre-solve axis');
      }
      assert.ok(params[2] > 0 && params[3] > 0, 'published semiaxes positive');
    } finally { kernel.free(); }
  }
}
function sweepWitness({ BrepKernel }) {
  const kernel = new BrepKernel();
  try {
    const retained = kernel.makeBox(1, 1, 1), a = kernel.makeRectangle(2, 2), b = kernel.makeRectangle(2, 2);
    const knots = Float64Array.of(0, 0, 1, 1), points = Float64Array.of(0, 0, 0, 0, 0, 10);
    const faces = Uint32Array.of(a, b), weights = Float64Array.of(1, 1);
    let before = state(kernel, [retained]);
    regularError(() => kernel.multiSectionSweep(faces, Float64Array.of(0.5, 0.5),
      1, knots, points, weights, true), /./, 'direct degenerate multi-section sweep');
    assert.deepEqual(state(kernel, [retained]), before, 'direct sweep live-state atomicity');
    const spine = okay(batch(kernel, 'executeBatchV2', 'makeNurbsEdge', {
      startX: 0, startY: 0, startZ: 0, endX: 0, endY: 0, endZ: 10,
      degree: 1, knots: Array.from(knots), controlPoints: Array.from(points), weights: [1, 1],
    }), 'spine fixture');
    for (const route of ['executeBatch', 'executeBatchV2']) {
      before = state(kernel, [retained]);
      const row = batch(kernel, route, 'multiSectionSweep', { faces: [a, b],
        params: [0.5, 0.5], spineEdge: spine, ruled: true });
      assert.ok(row.error, `${route}: degenerate sweep refuses`);
      if (route === 'executeBatchV2') assert.equal(typeof row.error.code, 'string');
      assert.deepEqual(state(kernel, [retained]), before, `${route} sweep live-state atomicity`);
    }
    const swept = kernel.multiSectionSweep(faces, Float64Array.of(0, 1), 1, knots, points, weights, true);
    close(kernel.volume(swept, 0.01), 40, 'valid multi-section sweep volume', 1e-6);
    bounds(kernel, swept, [-1, -1, 0, 1, 1, 10], 'valid sweep bounds');
  } finally { kernel.free(); }
}
const tetra = [[10, 20, 30], [11, 20, 30], [10, 21, 30], [10, 20, 31]];
const tetraFaces = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
function plyWitness({ BrepKernel, RemusIo }) {
  const io = new RemusIo(), kernel = new BrepKernel();
  try {
    const header = 'ply\nformat binary_little_endian 1.0\nelement vertex 4\n' +
      'property float x\nproperty float y\nproperty float z\nproperty uchar red\n' +
      'property uchar green\nproperty uchar blue\nelement face 4\n' +
      'property list uchar int vertex_indices\nend_header\n';
    const binary = Buffer.alloc(4 * 15 + 4 * 13); let offset = 0;
    for (const point of tetra) {
      for (const coordinate of point) { binary.writeFloatLE(coordinate, offset); offset += 4; }
      for (const color of [200, 100, 50]) binary[offset++] = color;
    }
    for (const face of tetraFaces) {
      binary[offset++] = 3;
      for (const index of face) { binary.writeInt32LE(index, offset); offset += 4; }
    }
    const normals = [[-1, -1, -1], [1, 0, 0], [0, 1, 0], [0, 0, 1]];
    const ascii = header.replace('binary_little_endian', 'ascii')
      .replace('element face 4', 'property float nx\nproperty float ny\nproperty float nz\nelement face 4') +
      tetra.map((point, i) => [...point, 255, 128, 7, ...normals[i]].join(' ')).join('\n') + '\n' +
      tetraFaces.map((face) => [3, ...face].join(' ')).join('\n') + '\n';
    for (const [label, data] of [['binary colors', Buffer.concat([bytes(header), binary])], ['ASCII normal columns', bytes(ascii)]]) {
      const solids = Array.from(kernel.deserializeSolids(io.importPly(data)));
      assert.equal(solids.length, 1, label); const solid = solids[0];
      close(kernel.volume(solid, 0.01), 1 / 6, `${label}: tetra volume`);
      bounds(kernel, solid, [10, 20, 30, 11, 21, 31], label);
      assert.equal(JSON.parse(kernel.validateSolidDetailed(solid)).errorCount, 0, `${label}: valid tetra`);
    }
  } finally { kernel.free(); io.free(); }
}
// Minimal one-entry stored ZIP, avoiding any new dependency for a 3MF fixture.
function modelZip(xml) {
  const name = Buffer.from('3D/3dmodel.model'), data = Buffer.from(xml);
  let crc = 0xffffffff;
  for (const byte of data) {
    crc ^= byte;
    for (let bit = 0; bit < 8; bit++) crc = (crc >>> 1) ^ ((crc & 1) ? 0xedb88320 : 0);
  }
  crc = (crc ^ 0xffffffff) >>> 0;
  const local = Buffer.alloc(30), central = Buffer.alloc(46), end = Buffer.alloc(22);
  local.writeUInt32LE(0x04034b50, 0); local.writeUInt16LE(20, 4); local.writeUInt32LE(crc, 14);
  local.writeUInt32LE(data.length, 18); local.writeUInt32LE(data.length, 22); local.writeUInt16LE(name.length, 26);
  central.writeUInt32LE(0x02014b50, 0); central.writeUInt16LE(20, 4); central.writeUInt16LE(20, 6);
  central.writeUInt32LE(crc, 16); central.writeUInt32LE(data.length, 20); central.writeUInt32LE(data.length, 24);
  central.writeUInt16LE(name.length, 28);
  end.writeUInt32LE(0x06054b50, 0); end.writeUInt16LE(1, 8); end.writeUInt16LE(1, 10);
  end.writeUInt32LE(central.length + name.length, 12); end.writeUInt32LE(local.length + name.length + data.length, 16);
  return Buffer.concat([local, name, data, central, name, end]);
}
function threemfWitness({ BrepKernel, RemusIo }) {
  const corners = [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
    [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]];
  const triangles = [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
    [2, 3, 7], [2, 7, 6], [0, 4, 7], [0, 7, 3], [1, 2, 6], [1, 6, 5]];
  const cube = (id) => `<object id="${id}" type="model"><mesh><vertices>` +
    corners.map(([x, y, z]) => `<vertex x="${x}" y="${y}" z="${z}"/>`).join('') +
    '</vertices><triangles>' + triangles.map(([v1, v2, v3]) => `<triangle v1="${v1}" v2="${v2}" v3="${v3}"/>`).join('') +
    '</triangles></mesh></object>';
  const xml = '<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:v="urn:vendor"><resources>' +
    cube(1) + cube(2) + '<object id="3" type="model"><components><component objectid="&#49;" transform="1 0 0 0 1 0 0 0 1 10 0 0"/></components></object>' +
    '</resources><build><item objectid=" 3 " transform="0 1 0 -1 0 0 0 0 1 0 0 100"/>' +
    '<item objectid="1" transform="1 0 0 0 1 0 0 0 1 5 0 0" v:transform="ignored"/>' +
    '<item objectid="1" transform="1 0 0 0 1 0 0 0 1 0 -7 0"/>' +
    '<item objectid="1" transform="-1 0 0 0 1 0 0 0 1 -5 0 0"/></build>' +
    '<v:build><v:item objectid="2"/></v:build></model>';
  const io = new RemusIo(), kernel = new BrepKernel();
  try {
    const solids = Array.from(kernel.deserializeSolids(io.import3mf(modelZip(xml))));
    assert.equal(solids.length, 4, 'one solid per core build item, multiplicity and no unused resource');
    const boxes = [[-1, 10, 100, 0, 11, 101], [5, 0, 0, 6, 1, 1],
      [0, -7, 0, 1, -6, 1], [-6, 0, 0, -5, 1, 1]];
    solids.forEach((solid, i) => {
      bounds(kernel, solid, boxes[i], `3MF build item ${i}`);
      close(kernel.volume(solid, 0.01), 1, `3MF build item ${i}: outward volume`);
      assert.equal(JSON.parse(kernel.validateSolidDetailed(solid)).errorCount, 0, `3MF item ${i}: valid`);
    });
    regularError(() => io.import3mf(modelZip(xml.replace('objectid=" 3 "', 'objectid="99"'))),
      /unknown object id 99/, 'unknown build item refuses');
  } finally { kernel.free(); io.free(); }
}
export function runPackageQueueWitness(exports) {
  const failures = [];
  for (const [name, witness] of Object.entries({ fromBREP: brepWitness, optionalArguments: optionalArgumentsWitness,
    legacySketch: legacySketchWitness, ellipsePublication: ellipseWitness, sweepRollback: sweepWitness,
    headerDrivenPLY: plyWitness, buildItem3MF: threemfWitness })) {
    try { witness(exports); console.log(`ok - queue package ${name}`); }
    catch (error) { failures.push(error); console.error(`FAIL - queue package ${name}: ${error.message}`); }
  }
  if (failures.length) throw new AggregateError(failures, `${failures.length} package queue witnesses failed`);
}
if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const consumer = process.env.REMUS_CONSUMER_DIR;
  const root = process.env.REMUS_PACKAGE_ROOT || resolve(dirname(fileURLToPath(import.meta.url)), '..');
  const require = createRequire(resolve(consumer || root, 'package-queue-witness.cjs'));
  const kernel = require(consumer ? 'remus-wasm' : resolve(root, 'crates/wasm/pkg/remus_wasm_node.cjs'));
  const io = require(consumer ? 'remus-wasm-io' : resolve(root, 'crates/wasm-io/pkg/remus_wasm_io_node.cjs'));
  runPackageQueueWitness({ ...kernel, ...io });
}
