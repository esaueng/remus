/** Exact pointed-cone cuts, exercised through built and installed packages.
 * Standalone: REMUS_PACKAGE_ROOT=/path/to/repo node scripts/pointed-cone-cut-packaged.mjs
 * Installed: REMUS_CONSUMER_DIR=/path/to/consumer node scripts/pointed-cone-cut-packaged.mjs
 */
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const radius = 0.5, cx = 1, cy = 1.5;
const fixtures = [
  { name: 'disjoint control', up: true, z0: 5 },
  { name: 'base-inside up', up: true, z0: 2 },
  { name: 'base-inside down', up: false, z0: -2 },
  { name: 'apex-inside up', up: true, z0: -2 },
  { name: 'apex-inside down', up: false, z0: 2 },
];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const sub = (a, b) => a.map((v, i) => v - b[i]);
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
function close(actual, expected, tolerance, label) {
  assert.ok(Number.isFinite(actual) && Math.abs(actual - expected) <= tolerance,
    `${label}: got ${actual}, expected ${expected}`);
}
function invoke(kernel, route, op, args, direct, label) {
  if (route === 'direct') return kernel[op === 'transform' ? 'transformSolid' : op](...direct);
  const row = JSON.parse(kernel[route](JSON.stringify([{ op, args }])))[0];
  assert.ok(Object.hasOwn(row, 'ok') && !Object.hasOwn(row, 'error'),
    `${label}: ${op}: ${JSON.stringify(row)}`);
  return row.ok;
}
function recordUse(uses, edge, forward) {
  const directions = uses.get(edge) ?? [];
  directions.push(forward);
  uses.set(edge, directions);
}
function opposedUses(uses, label) {
  assert.ok(uses.size > 0, `${label}: no edges`);
  for (const [edge, directions] of uses) {
    assert.equal(directions.length, 2, `${label}: boundary/non-manifold edge ${edge}`);
    assert.notEqual(directions[0], directions[1], `${label}: same-sense edge ${edge}`);
  }
}
function topology(kernel, solid, hasCavity, label) {
  const doc = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(solid))));
  const root = doc.solids[doc.solid_roots[0]];
  const faces = [root.outer_shell, ...root.inner_shells].flatMap(shell => doc.shells[shell].faces);
  const uses = new Map();
  let cones = 0;
  for (const index of faces) {
    const face = doc.faces[index], kind = Object.keys(face.surface)[0];
    assert.ok(kind === 'Plane' || kind === 'Cone', `${label}: unexpected ${kind} carrier`);
    if (kind === 'Cone') cones++;
    for (const index of [face.outer_wire, ...face.inner_wires]) {
      const wire = doc.wires[index];
      assert.equal(wire.closed, true, `${label}: open boundary wire`);
      for (const use of wire.edges) recordUse(uses, use.edge, use.forward !== face.reversed);
    }
  }
  assert.equal(cones > 0, hasCavity, `${label}: conical cavity presence`);
  opposedUses(uses, `${label}: topology`);
  const validation = JSON.parse(kernel.validateSolidDetailed(solid));
  assert.equal(validation.errorCount, 0, `${label}: ${JSON.stringify(validation)}`);
  return { doc, faces };
}
function checkMesh(kernel, solid, boundary, apex, up, height, expectedVolume, deflection, label) {
  const mesh = JSON.parse(kernel.tessellateSolidGrouped(solid, deflection, 0.35));
  assert.equal(mesh.positions.length % 3, 0, `${label}: position triples`);
  assert.equal(mesh.indices.length % 3, 0, `${label}: triangle triples`);
  assert.equal(mesh.faceOffsets.length, boundary.faces.length + 1, `${label}: face grouping`);
  assert.equal(mesh.faceOffsets[0], 0, `${label}: first face offset`);
  assert.equal(mesh.faceOffsets.at(-1), mesh.indices.length, `${label}: final face offset`);
  assert.ok(mesh.positions.every(Number.isFinite), `${label}: finite mesh coordinates`);
  const points = Array.from({ length: mesh.positions.length / 3 }, (_, i) => mesh.positions.slice(i * 3, i * 3 + 3));
  const uses = new Map();
  let volume = 0;
  for (let fi = 0; fi < boundary.faces.length; fi++) {
    const cone = Object.hasOwn(boundary.doc.faces[boundary.faces[fi]].surface, 'Cone');
    assert.ok(mesh.faceOffsets[fi + 1] > mesh.faceOffsets[fi], `${label}: empty face ${fi}`);
    for (let at = mesh.faceOffsets[fi]; at < mesh.faceOffsets[fi + 1]; at += 3) {
      const indices = mesh.indices.slice(at, at + 3);
      assert.ok(indices.every(i => Number.isInteger(i) && i >= 0 && i < points.length), `${label}: mesh index bounds`);
      const [a, b, c] = indices.map(i => points[i]), normal = cross(sub(b, a), sub(c, a));
      assert.ok(Math.hypot(...normal) > 1e-15, `${label}: zero-area triangle`);
      volume += dot(a, cross(b, c)) / 6;
      if (cone) {
        const center = a.map((v, i) => (v + b[i] + c[i]) / 3);
        const dx = center[0] - cx, dy = center[1] - cy;
        const outwardCone = [dx, dy, (up ? 1 : -1) * radius / height * Math.hypot(dx, dy)];
        assert.ok(dot(normal, outwardCone) < 0, `${label}: conical cavity faces into retained material`);
        for (const point of [a, b, c]) {
          assert.ok(point[2] >= -1e-7 && point[2] <= 4 + 1e-7, `${label}: cavity escaped box slab`);
          close(Math.hypot(point[0] - cx, point[1] - cy), radius * Math.abs(point[2] - apex) / height,
            1e-6, `${label}: vertex on exact cone`);
        }
      }
      for (const [a, b] of [[indices[0], indices[1]], [indices[1], indices[2]], [indices[2], indices[0]]]) {
        recordUse(uses, a < b ? `${a},${b}` : `${b},${a}`, a < b);
      }
    }
  }
  opposedUses(uses, `${label}: raw mesh`);
  // Chordal cavity facets approximate the exact conic; this is a mesh-volume
  // control, separate from the 1e-9 exact BREP-volume assertion below.
  close(volume, expectedVolume, 4 * deflection, `${label}: oriented chordal mesh volume`);
}
function fixture(RemusKernel, route, item) {
  const label = `pointed cone ${item.name}, ${route}`, kernel = new RemusKernel();
  const run = (op, args, direct) => invoke(kernel, route, op, args, direct, label);
  try {
    const height = item.height ?? 4;
    const box = run('makeBox', { width: 2, height: 3, depth: 4 }, [2, 3, 4]);
    const bottomRadius = item.up ? radius : 0, topRadius = item.up ? 0 : radius;
    const cone = run('makeCone', { bottomRadius, topRadius, height }, [bottomRadius, topRadius, height]);
    const matrix = [1, 0, 0, cx, 0, 1, 0, cy, 0, 0, 1, item.z0, 0, 0, 0, 1];
    run('transform', { solid: cone, matrix }, [cone, Float64Array.from(matrix)]);
    const apex = item.up ? item.z0 + height : item.z0, base = item.up ? item.z0 : item.z0 + height;
    const whole = Math.PI * radius ** 2 * height / 3;
    close(kernel.volume(box, 0.001), 24, 1e-9, `${label}: input box`);
    close(kernel.volume(cone, 0.001), whole, 1e-9, `${label}: input cone`);
    const operands = Uint32Array.of(box, cone), original = kernel.serializeSolids(operands);
    const outcome = run('booleanWithQuality', { operation: 'cut', solidA: box, solidB: cone, exactOnly: true },
      ['cut', box, cone, true]);
    assert.equal(outcome.quality, 'exact', `${label}: exact quality`);
    const lo = Math.min(apex, base), hi = Math.max(apex, base);
    // Integral of pi*(r/h)^2*(z-apex)^2 over the cone/box slab intersection.
    const lower = Math.max(0, lo), upper = Math.min(4, hi);
    const removed = upper <= lower ? 0
      : Math.PI * (radius / height) ** 2 * ((upper - apex) ** 3 - (lower - apex) ** 3) / 3;
    const expected = 24 - removed, solid = outcome.solid;
    close(kernel.volume(solid, 0.001), expected, 1e-9, `${label}: exact cut volume`);
    close(JSON.parse(kernel.massProperties(solid)).volume, expected, 1e-9, `${label}: exact mass volume`);
    const boundary = topology(kernel, solid, removed > 0, label);
    for (const deflection of [0.01, 0.001]) checkMesh(kernel, solid, boundary, apex, item.up, height, expected, deflection, label);
    const probes = [
      [[0.2, 0.3, 2], 'inside'], [[1.8, 2.7, 0.5], 'inside'],
      [[cx, cy, 4.5], 'outside'], [[cx, cy, -0.5], 'outside'],
    ];
    for (const z of [0.3, 1.1, 2.6, 3.7, 3.95]) {
      if (z > lo && z < hi) {
        const r = radius * Math.abs(z - apex) / height;
        probes.push([[cx + r / 2, cy, z], 'outside'], [[cx + r + 0.05, cy, z], 'inside']);
      } else probes.push([[cx, cy, z], 'inside']);
    }
    for (const [point, expected] of probes) {
      assert.equal(run('classifyPoint', { solid, x: point[0], y: point[1], z: point[2], tolerance: 1e-7 },
        [solid, ...point, 1e-7]), expected, `${label}: material at ${point}`);
    }
    assert.deepEqual(kernel.serializeSolids(operands), original, `${label}: retained operands unchanged`);
  } finally { kernel.free(); }
  console.log(`ok - ${label}: exact volume, oriented topology/mesh and material`);
}
export function runPointedConeCutPackaged({ RemusKernel }) {
  const failures = [];
  for (const route of ['direct', 'executeBatch', 'executeBatchV2']) for (const item of fixtures) {
    try { fixture(RemusKernel, route, item); }
    catch (error) { failures.push(error); console.error(`FAIL - pointed cone ${item.name}, ${route}: ${error.message}`); }
  }
  if (failures.length) throw new AggregateError(failures, `${failures.length} pointed-cone packaged cells failed`);
}
if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const consumer = process.env.REMUS_CONSUMER_DIR;
  const root = process.env.REMUS_PACKAGE_ROOT || resolve(dirname(fileURLToPath(import.meta.url)), '..');
  const require = createRequire(resolve(consumer || root, 'pointed-cone-cut-witness.cjs'));
  runPointedConeCutPackaged(require(consumer ? 'remus-wasm' : resolve(root, 'crates/wasm/pkg/remus_wasm_node.cjs')));
}
