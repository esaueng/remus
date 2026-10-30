/** Independent spherical-octant material, quantity and raw-mesh contracts. */
import assert from 'node:assert/strict';

const identity = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
const multiply = (a, b) => Array.from({ length: 16 }, (_, i) => {
  const r = Math.floor(i / 4), c = i % 4;
  return a[r * 4] * b[c] + (a[r * 4 + 1] * b[c + 4]
    + (a[r * 4 + 2] * b[c + 8] + a[r * 4 + 3] * b[c + 12]));
});
function placement(scale, placed) {
  if (!placed) return identity();
  const a = 0.37, b = 0.23, ca = Math.cos(a), sa = Math.sin(a), cb = Math.cos(b), sb = Math.sin(b);
  return multiply([1, 0, 0, 10 * scale, 0, 1, 0, -7 * scale, 0, 0, 1, 3 * scale, 0, 0, 0, 1],
    multiply([1, 0, 0, 0, 0, ca, -sa, 0, 0, sa, ca, 0, 0, 0, 0, 1],
      [cb, -sb, 0, 0, sb, cb, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]));
}
const worldPoint = (m, p) => [0, 1, 2].map(r => m[4 * r] * p[0]
  + (m[4 * r + 1] * p[1] + (m[4 * r + 2] * p[2] + m[4 * r + 3])));
const localPoint = (m, p, scale) => [0, 1, 2].map(c => (m[c] * (p[0] - m[3])
  + (m[c + 4] * (p[1] - m[7]) + m[c + 8] * (p[2] - m[11]))) / scale);
const sub = (a, b) => a.map((v, i) => v - b[i]);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);

function intersectsRemovedOctant(triangle) {
  let polygon = triangle;
  for (let axis = 0; axis < 3; axis++) {
    const clipped = [];
    for (let i = 0; i < polygon.length; i++) {
      const a = polygon[i], b = polygon[(i + 1) % polygon.length];
      const da = a[axis] - 1e-6, db = b[axis] - 1e-6;
      if (da >= 0) clipped.push(a);
      if ((da >= 0) !== (db >= 0)) clipped.push(a.map((v, j) => v + (b[j] - v) * da / (da - db)));
    }
    polygon = clipped;
  }
  return polygon.length > 0;
}

function invoke(kernel, api, op, args, direct, label) {
  if (api === 'direct') return kernel[op === 'transform' ? 'transformSolid' : op](...direct);
  const cell = JSON.parse(kernel[api](JSON.stringify([{ op, args }])))[0];
  assert.ok(Object.hasOwn(cell, 'ok'), `${label}: ${op}: ${JSON.stringify(cell)}`);
  return cell.ok;
}

function checkTopology(kernel, solid, label, minimumFaces = 4) {
  const doc = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(solid))));
  assert.ok(doc.faces.length >= minimumFaces && doc.faces.length <= 12, `${label}: analytic census`);
  const uses = new Map();
  let spherical = 0;
  for (const face of doc.faces) {
    const [kind] = Object.keys(face.surface);
    assert.ok(kind === 'Plane' || kind === 'Sphere', `${label}: unexpected ${kind} carrier`);
    if (kind === 'Sphere') spherical++;
    for (const wi of [face.outer_wire, ...face.inner_wires]) {
      const wire = doc.wires[wi];
      assert.equal(wire.closed, true, `${label}: open wire`);
      for (const use of wire.edges) {
        const directions = uses.get(use.edge) ?? [];
        directions.push(use.forward !== face.reversed);
        uses.set(use.edge, directions);
      }
    }
  }
  assert.ok(spherical > 0, `${label}: lost spherical carrier`);
  for (const [edge, directions] of uses) {
    assert.equal(directions.length, 2, `${label}: edge ${edge} incidence`);
    assert.notEqual(directions[0], directions[1], `${label}: edge ${edge} same-sense uses`);
  }
  const validation = JSON.parse(kernel.validateSolidDetailed(solid));
  assert.equal(validation.errorCount, 0, `${label}: strict validation: ${JSON.stringify(validation)}`);
  return doc;
}

function checkMesh(kernel, solid, doc, matrix, scale, deflection, label, angular = 0.35,
  sphericalTriangleAllowed = triangle => !intersectsRemovedOctant(triangle)) {
  const mesh = JSON.parse(kernel.tessellateSolidGrouped(solid, deflection * scale, angular));
  const root = doc.solids[doc.solid_roots[0]];
  const faceOrder = [root.outer_shell, ...root.inner_shells].flatMap(shell => doc.shells[shell].faces);
  assert.equal(mesh.faceOffsets.length, faceOrder.length + 1, `${label}: face grouping`);
  const points = Array.from({ length: mesh.positions.length / 3 }, (_, i) =>
    localPoint(matrix, mesh.positions.slice(i * 3, i * 3 + 3), scale));
  const uses = new Map();
  for (let fi = 0; fi < faceOrder.length; fi++) {
    const spherical = Object.hasOwn(doc.faces[faceOrder[fi]].surface, 'Sphere');
    assert.ok(mesh.faceOffsets[fi + 1] > mesh.faceOffsets[fi], `${label}: empty face ${fi}`);
    for (let at = mesh.faceOffsets[fi]; at < mesh.faceOffsets[fi + 1]; at += 3) {
      const indices = mesh.indices.slice(at, at + 3);
      const [a, b, c] = indices.map(i => points[i]);
      const normal = cross(sub(b, a), sub(c, a));
      assert.ok(Math.hypot(...normal) > 1e-15, `${label}: degenerate triangle`);
      if (spherical) {
        const center = a.map((v, i) => (v + b[i] + c[i]) / 3);
        assert.ok(dot(normal, center) > 0, `${label}: inward spherical triangle`);
        assert.ok(sphericalTriangleAllowed([a, b, c]), `${label}: triangle intersects removed material`);
        for (const p of [a, b, c]) assert.ok(Math.abs(Math.hypot(...p) - 1) < 1e-5, `${label}: off-sphere mesh vertex`);
      }
      for (const [a, b] of [[indices[0], indices[1]], [indices[1], indices[2]], [indices[2], indices[0]]]) {
        const key = a < b ? `${a},${b}` : `${b},${a}`;
        const directions = uses.get(key) ?? [];
        directions.push(a < b);
        uses.set(key, directions);
      }
    }
  }
  for (const [edge, directions] of uses) {
    assert.equal(directions.length, 2, `${label}: raw mesh boundary/non-manifold ${edge}`);
    assert.notEqual(directions[0], directions[1], `${label}: same-sense mesh edge ${edge}`);
  }
  const quality = JSON.parse(kernel.meshQuality(solid, deflection * scale, angular));
  assert.equal(quality.boundaryEdges, 0, `${label}: public mesh boundary`);
  assert.equal(quality.nonManifoldEdges, 0, `${label}: public non-manifold mesh`);
  assert.equal(quality.isWatertight, true, `${label}: public mesh closure`);
}

function checkFixture(RemusKernel, api, scale, placed, segments, side, fuse = false, reverse = false) {
  const label = `F6 ${api} scale=${scale} placed=${placed} segments=${segments} side=${side} fuse=${fuse} reverse=${reverse}`;
  const kernel = new RemusKernel();
  const run = (op, args, direct) => invoke(kernel, api, op, args, direct, label);
  try {
    const makeCube = () => run('makeBox', { width: side * scale, height: side * scale, depth: side * scale }, [side * scale, side * scale, side * scale]);
    const makeSphere = () => run('makeSphere', { radius: scale, segments }, [scale, segments]);
    let cube, sphere;
    if (fuse) { sphere = makeSphere(); cube = makeCube(); } else { cube = makeCube(); sphere = makeSphere(); }
    const matrix = placement(scale, placed);
    if (placed) for (const solid of [cube, sphere]) run('transform', { solid, matrix }, [solid, Float64Array.from(matrix)]);
    const ids = Uint32Array.of(cube, sphere), original = kernel.serializeSolids(ids);
    try {
      const operation = fuse ? 'fuse' : 'cut', solidA = reverse ? cube : sphere, solidB = reverse ? sphere : cube;
      const outcome = run('booleanWithQuality', { operation, solidA, solidB, exactOnly: true }, [operation, solidA, solidB, true]);
      assert.equal(outcome.quality, 'exact', `${label}: exact quality`);
      const solid = outcome.solid, doc = checkTopology(kernel, solid, label);
      for (const d of [0.1, 0.01, 0.001]) checkMesh(kernel, solid, doc, matrix, scale, d, `${label} deflection=${d}`);
      if (scale === 1 && !placed && segments === 16 && side === 1) {
        checkMesh(kernel, solid, doc, matrix, scale, 0.1, `${label} angular=0.1`, 0.1);
      }
      const classes = [0, 0, 0, 0];
      for (const x of [-1.15, -0.85, -0.55, -0.25, 0.25, 0.55, 0.85, 1.15]) {
        for (const y of [-1.15, -0.85, -0.55, -0.25, 0.25, 0.55, 0.85, 1.15]) {
          for (const z of [-1.15, -0.85, -0.55, -0.25, 0.25, 0.55, 0.85, 1.15]) {
            if (Math.abs(Math.hypot(x, y, z) - 1) < 0.06 || [x, y, z].some(v => Math.abs(v - side) < 0.06)) continue;
            const ball = Math.hypot(x, y, z) < 1;
            const box = [x, y, z].every(v => v > 0 && v < side);
            classes[2 * Number(ball) + Number(box)]++;
            const point = worldPoint(matrix, [x * scale, y * scale, z * scale]);
            for (const [name, id, inside] of [['sphere', sphere, ball], ['box', cube, box], ['result', solid, fuse ? ball || box : ball && !box]]) {
              const expected = inside ? 'inside' : 'outside';
              const args = { solid: id, x: point[0], y: point[1], z: point[2], tolerance: 1e-7 };
              assert.equal(run('classifyPoint', args, [id, ...point, 1e-7]), expected, `${label}: ${name} at ${[x, y, z]}`);
            }
          }
        }
      }
      assert.ok(classes.every(n => n > 0), `${label}: Cartesian coverage`);
      const unit = 1 / Math.sqrt(3);
      for (const [p, expected] of [
        [[unit, unit, unit], fuse ? 'inside' : 'outside'], [[-unit, unit, unit], 'boundary'], [[0.25, 0.25, 0], fuse ? 'inside' : 'boundary'],
      ]) {
        const point = worldPoint(matrix, p.map(v => v * scale));
        assert.equal(run('classifyPoint', { solid, x: point[0], y: point[1], z: point[2], tolerance: 1e-7 },
          [solid, ...point, 1e-7]), expected, `${label}: exact boundary membership at ${p}`);
      }
      const expectedVolume = 7 * Math.PI / 6 + (fuse ? 1 : 0);
      const expectedArea = fuse ? 11 * Math.PI / 4 + 6 : 17 * Math.PI / 4;
      for (const deflection of [0.0001, 0.00005]) {
        const volume = kernel.volume(solid, deflection * scale) / scale ** 3;
        assert.ok(Math.abs(volume - expectedVolume) < 1e-4 * expectedVolume, `${label}: V=${volume} at ${deflection}`);
      }
      const gauss = JSON.parse(kernel.massProperties(solid)).volume / scale ** 3;
      assert.ok(Math.abs(gauss - expectedVolume) < 1e-4 * expectedVolume, `${label}: Gauss V=${gauss}`);
      const area = kernel.surfaceArea(solid, 0.0001 * scale) / scale ** 2;
      assert.ok(Math.abs(area - expectedArea) < 1e-4 * expectedArea, `${label}: A=${area}`);
      for (const [p, inside] of [[[-0.55, -0.25, 0.55], true], [[0.25, 0.25, 0.25], fuse], [[0.85, 0.85, -0.85], false]]) {
        const point = worldPoint(matrix, p.map(v => v * scale));
        assert.equal(kernel.classifyPointRobust(solid, ...point, 1e-7), inside ? 'inside' : 'outside', `${label}: robust ${p}`);
      }
    } finally {
      assert.deepEqual(kernel.serializeSolids(ids), original, `${label}: unchanged retained operands`);
    }
  } finally { kernel.free(); }
  console.log(`ok - ${label}: material, quantities, strict topology and three raw mesh deflections`);
}

function checkLatitudeCut(RemusKernel, api) {
  const label = `F6 compatibility ${api}: radius-10 sphere cut above z=5`;
  const kernel = new RemusKernel();
  const run = (op, args, direct) => invoke(kernel, api, op, args, direct, label);
  try {
    const sphere = run('makeSphere', { radius: 10, segments: 32 }, [10, 32]);
    const box = run('makeBox', { width: 30, height: 30, depth: 10 }, [30, 30, 10]);
    const matrix = identity();
    matrix[3] = -15; matrix[7] = -15; matrix[11] = 5;
    run('transform', { solid: box, matrix }, [box, Float64Array.from(matrix)]);
    const ids = Uint32Array.of(sphere, box), original = kernel.serializeSolids(ids);
    const outcome = run('booleanWithQuality', { operation: 'cut', solidA: sphere, solidB: box, exactOnly: true },
      ['cut', sphere, box, true]);
    assert.equal(outcome.quality, 'exact', `${label}: exact quality`);
    const solid = outcome.solid, doc = checkTopology(kernel, solid, label, 3);
    assert.equal(doc.faces.length, 3, `${label}: three-face analytic cut`);
    const expectedVolume = 1000 * 9 * Math.PI / 8;
    assert.ok(Math.abs(kernel.volume(solid, 0.01) - expectedVolume) < 1e-8, `${label}: spherical-cap volume`);
    assert.ok(Math.abs(JSON.parse(kernel.massProperties(solid)).volume - expectedVolume) < 1e-8, `${label}: mass-properties volume`);
    assert.ok(Math.abs(kernel.surfaceArea(solid, 0.01) - 100 * 15 * Math.PI / 4) < 1e-8, `${label}: spherical-cap area`);
    for (const deflection of [0.1, 0.05, 0.01, 0.005]) {
      checkMesh(kernel, solid, doc, identity(), 10, deflection / 10, label, 0.35,
        triangle => triangle.every(point => point[2] <= 0.5 + 1e-6));
    }
    assert.deepEqual(kernel.serializeSolids(ids), original, `${label}: unchanged retained operands`);
  } finally { kernel.free(); }
  console.log(`ok - ${label}: exact quantities and four raw mesh deflections`);
}

export function runOpseqF6Packaged({ RemusKernel }) {
  for (const api of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const scale of [1, 0.001, 1000]) {
      for (const placed of [false, true]) checkFixture(RemusKernel, api, scale, placed, 16, 1);
    }
    for (const segments of [8, 32]) checkFixture(RemusKernel, api, 1, false, segments, 1);
    checkFixture(RemusKernel, api, 1, false, 16, 1.25);
    for (const reverse of [false, true]) checkFixture(RemusKernel, api, 1, false, 16, 1, true, reverse);
    checkLatitudeCut(RemusKernel, api);
  }
}
