/** Independent placed sphere/box material, quantity and raw-mesh contracts. */
import assert from 'node:assert/strict';

const identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
const multiply = (a, b) => Array.from({ length: 16 }, (_, i) => {
  const r = Math.floor(i / 4), c = i % 4;
  return a[r * 4] * b[c] + (a[r * 4 + 1] * b[c + 4]
    + (a[r * 4 + 2] * b[c + 8] + a[r * 4 + 3] * b[c + 12]));
});
const point = (m, p) => [0, 1, 2].map(r => m[r * 4] * p[0]
  + (m[r * 4 + 1] * p[1] + (m[r * 4 + 2] * p[2] + m[r * 4 + 3])));
const local = (m, p, scale) => [0, 1, 2].map(c => (m[c] * (p[0] - m[3])
  + (m[c + 4] * (p[1] - m[7]) + m[c + 8] * (p[2] - m[11]))) / scale);
const sub = (a, b) => a.map((v, i) => v - b[i]);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);

function quantities(side, h, operation) {
  const r = 3, a = side, b = side;
  const integral = (radius, length) => (length * Math.sqrt(radius * radius - length * length)
    + radius * radius * Math.asin(length / radius)) / 2;
  const k = a * Math.asin(b / Math.sqrt(r * r - a * a))
    + b * Math.asin(a / Math.sqrt(r * r - b * b))
    - r * Math.atan(a * b / (r * Math.sqrt(r * r - a * a - b * b)));
  const bite = (a * integral(Math.sqrt(r * r - a * a), b)
    + b * integral(Math.sqrt(r * r - b * b), a) + r * r * k) / 3 - h * a * b;
  const planes = a * b + integral(r, a) - h * a + integral(r, b) - h * b
    + integral(Math.sqrt(r * r - b * b), a) - h * a
    + integral(Math.sqrt(r * r - a * a), b) - h * b;
  const trimLength = r * Math.asin(a / r) + r * Math.asin(b / r)
    + Math.sqrt(r * r - a * a) * Math.asin(b / Math.sqrt(r * r - a * a))
    + Math.sqrt(r * r - b * b) * Math.asin(a / Math.sqrt(r * r - b * b));
  if (operation === 'intersect') return [bite, r * k + planes, trimLength];
  if (operation === 'fuse') return [36 * Math.PI + side * side - bite,
    36 * Math.PI - r * k + 2 * side * side + 4 * side - planes, trimLength];
  return [36 * Math.PI - bite, 36 * Math.PI - r * k + planes, trimLength];
}

function entersBox(triangle, side) {
  let polygon = triangle;
  for (const [axis, high] of [[0, side], [1, side], [2, 1]]) for (const upper of [false, true]) {
    const clipped = [], signed = p => (upper ? high - p[axis] : p[axis]) - 1e-6;
    for (let i = 0; i < polygon.length; i++) {
      const a = polygon[i], b = polygon[(i + 1) % polygon.length], da = signed(a), db = signed(b);
      if (da >= 0) clipped.push(a);
      if ((da >= 0) !== (db >= 0)) clipped.push(a.map((v, j) => v + (b[j] - v) * da / (da - db)));
    }
    polygon = clipped;
  }
  return polygon.length > 0;
}

function checkFixture(BrepKernel, api, scale, placed, side, h, theta, operation, reverse = false) {
  const label = `F13 ${api} scale=${scale} placed=${placed} side=${side} h=${h} theta=${theta} ${operation} reverse=${reverse}`;
  const kernel = new BrepKernel();
  const run = (op, args, direct) => {
    if (api === 'direct') return kernel[op === 'transform' ? 'transformSolid' : op](...direct);
    const cell = JSON.parse(kernel[api](JSON.stringify([{ op, args }])))[0];
    assert.ok(Object.hasOwn(cell, 'ok'), `${label}: ${op}: ${JSON.stringify(cell)}`);
    return cell.ok;
  };
  try {
    const box = run('makeBox', { width: side * scale, height: side * scale, depth: scale }, [side * scale, side * scale, scale]);
    const sphere = run('makeSphere', { radius: 3 * scale, segments: 16 }, [3 * scale, 16]);
    const c = Math.cos(theta), s = Math.sin(theta);
    const frame = [c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, -h * scale, 0, 0, 0, 1];
    run('transform', { solid: sphere, matrix: frame }, [sphere, Float64Array.from(frame)]);
    const ca = Math.cos(0.37), sa = Math.sin(0.37), cb = Math.cos(0.23), sb = Math.sin(0.23);
    const matrix = placed ? multiply([1, 0, 0, 10 * scale, 0, 1, 0, -7 * scale, 0, 0, 1, 3 * scale, 0, 0, 0, 1],
      multiply([1, 0, 0, 0, 0, ca, -sa, 0, 0, sa, ca, 0, 0, 0, 0, 1],
        [cb, -sb, 0, 0, sb, cb, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])) : identity;
    if (placed) for (const solid of [box, sphere]) run('transform', { solid, matrix }, [solid, Float64Array.from(matrix)]);
    const ids = Uint32Array.of(box, sphere), original = kernel.serializeSolids(ids);
    const solidA = reverse ? box : sphere, solidB = reverse ? sphere : box;
    const outcome = run('booleanWithQuality', { operation, solidA, solidB, exactOnly: true }, [operation, solidA, solidB, true]);
    assert.equal(outcome.quality, 'exact', `${label}: exact quality`);
    const solid = outcome.solid;
    assert.equal(JSON.parse(kernel.validateSolidDetailed(solid)).errorCount, 0, `${label}: strict validity`);
    const doc = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(solid))));
    assert.ok(doc.faces.length >= 4 && doc.faces.length <= 12, `${label}: analytic census`);
    const uses = new Map();
    for (const face of doc.faces) {
      assert.ok(Object.hasOwn(face.surface, 'Plane') || Object.hasOwn(face.surface, 'Sphere'), `${label}: analytic carrier`);
      for (const wi of [face.outer_wire, ...face.inner_wires]) for (const use of doc.wires[wi].edges) {
        const directions = uses.get(use.edge) ?? [];
        directions.push(use.forward !== face.reversed); uses.set(use.edge, directions);
      }
    }
    for (const directions of uses.values()) {
      assert.equal(directions.length, 2, `${label}: B-Rep edge incidence`);
      assert.notEqual(directions[0], directions[1], `${label}: B-Rep opposing coedges`);
    }
    const [expectedVolume, expectedArea, trimLength] = quantities(side, h, operation);
    const root = doc.solids[doc.solid_roots[0]], faceOrder = [root.outer_shell, ...root.inner_shells].flatMap(sh => doc.shells[sh].faces);
    for (const [d, angular] of [[0.1, 0.35], [0.01, 0], [0.003, 0.35]]) {
      const mesh = JSON.parse(kernel.tessellateSolidGrouped(solid, d * scale, angular));
      const points = Array.from({ length: mesh.positions.length / 3 }, (_, i) => local(matrix, mesh.positions.slice(i * 3, i * 3 + 3), scale));
      const meshUses = new Map(), positionUses = new Map(); let area = 0, volume = 0;
      const reference = [0, 0, -h];
      assert.equal(mesh.faceOffsets.length, faceOrder.length + 1, `${label}: face offsets`);
      for (let fi = 0; fi < faceOrder.length; fi++) {
        assert.ok(mesh.faceOffsets[fi + 1] > mesh.faceOffsets[fi], `${label}: empty face`);
        const spherical = Object.hasOwn(doc.faces[faceOrder[fi]].surface, 'Sphere');
        for (let i = mesh.faceOffsets[fi]; i < mesh.faceOffsets[fi + 1]; i += 3) {
          const indices = mesh.indices.slice(i, i + 3), [a, b, c] = indices.map(v => points[v]);
          const normal = cross(sub(b, a), sub(c, a)); area += Math.hypot(...normal) / 2;
          volume += dot(sub(a, reference), cross(sub(b, reference), sub(c, reference))) / 6;
          assert.ok(Math.hypot(...normal) > 1e-15, `${label}: degenerate triangle`);
          if (spherical) {
            assert.ok(dot(normal, sub(a.map((v, j) => (v + b[j] + c[j]) / 3), reference)) > 0, `${label}: inward sphere triangle`);
            for (const p of [a, b, c]) assert.ok(Math.abs(Math.hypot(...sub(p, reference)) - 3) < 1e-6, `${label}: off-sphere vertex`);
            if (operation !== 'intersect') assert.ok(!entersBox([a, b, c], side), `${label}: removed sphere patch`);
          }
          for (let j = 0; j < 3; j++) {
            const a = indices[j], b = indices[(j + 1) % 3], key = a < b ? `${a}:${b}` : `${b}:${a}`;
            const directions = meshUses.get(key) ?? []; directions.push(a < b); meshUses.set(key, directions);
            const ka = points[a].map(v => Math.round(v * 1e8)).join(':'), kb = points[b].map(v => Math.round(v * 1e8)).join(':');
            assert.notEqual(ka, kb, `${label}: collapsed position edge`);
            const positionKey = ka < kb ? `${ka}|${kb}` : `${kb}|${ka}`;
            const positional = positionUses.get(positionKey) ?? []; positional.push(ka < kb); positionUses.set(positionKey, positional);
          }
        }
      }
      for (const directions of meshUses.values()) {
        assert.equal(directions.length, 2, `${label}: raw mesh closure at ${d}`);
        assert.notEqual(directions[0], directions[1], `${label}: raw mesh orientation at ${d}`);
      }
      for (const directions of positionUses.values()) {
        assert.equal(directions.length, 2, `${label}: position mesh closure at ${d}`);
        assert.notEqual(directions[0], directions[1], `${label}: position mesh orientation at ${d}`);
      }
      assert.ok(Math.abs(volume - expectedVolume) < expectedArea * d * 0.75, `${label}: raw volume ${volume} != ${expectedVolume}`);
      // Each planar circular arc contributes a chord strip bounded by
      // L_arc*sagitta. Include that independent planar term alongside curvature.
      assert.ok(Math.abs(area - expectedArea) < (expectedArea * 2 / 3 + trimLength) * d, `${label}: raw area ${area} != ${expectedArea}`);
      const quality = JSON.parse(kernel.meshQuality(solid, d * scale, angular));
      assert.equal(quality.isWatertight, true, `${label}: public mesh closure`);
    }
    for (const d of [1e-4, 5e-5]) assert.ok(Math.abs(kernel.volume(solid, d * scale) / scale ** 3 - expectedVolume) < expectedVolume * 2e-4, `${label}: closed-form volume`);
    assert.ok(Math.abs(kernel.surfaceArea(solid, 1e-4 * scale) / scale ** 2 - expectedArea) < expectedArea * 3e-4, `${label}: closed-form area`);
    const coverage = [0, 0];
    for (const x of [-1.2, 0.15, 0.5, 0.85, 1.2]) for (const y of [-1.2, 0.15, 0.5, 0.85, 1.2]) for (const z of [-3, -0.2, 0.1, 0.3, 0.7, 1.2]) {
      if (Math.abs(Math.hypot(x, y, z + h) - 3) < 0.06) continue;
      const sphereInside = Math.hypot(x, y, z + h) < 3, boxInside = x > 0 && x < side && y > 0 && y < side && z > 0 && z < 1;
      const inside = operation === 'cut' ? sphereInside && !boxInside : operation === 'fuse' ? sphereInside || boxInside : sphereInside && boxInside;
      coverage[Number(inside)]++;
      const p = point(matrix, [x * scale, y * scale, z * scale]);
      assert.equal(run('classifyPoint', { solid, x: p[0], y: p[1], z: p[2], tolerance: 1e-7 }, [solid, ...p, 1e-7]), inside ? 'inside' : 'outside', `${label}: material ${[x, y, z]}`);
    }
    assert.ok(coverage.every(n => n > 0), `${label}: material coverage`);
    assert.deepEqual(kernel.serializeSolids(ids), original, `${label}: retained source documents`);
  } finally { kernel.free(); }
}

export function runOpseqF13Packaged({ BrepKernel }) {
  let cells = 0;
  for (const api of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const scale of [0.001, 1]) for (const placed of [false, true]) {
      for (const [operation, reverse] of [['cut', false], ['fuse', false], ['fuse', true], ['intersect', false]]) {
        checkFixture(BrepKernel, api, scale, placed, 1, 2.5, Math.PI / 6, operation, reverse); cells++;
      }
    }
    // h=2.45 at unit scale and the literal h=2.5 at1000x are separate
    // pre-mesh equator-orientation defects, retained as native ready-repros.
    for (const placed of [false, true]) for (const [operation, reverse] of [['cut', false], ['fuse', false], ['fuse', true], ['intersect', false]]) {
      checkFixture(BrepKernel, api, 1000, placed, 1, 2.55, Math.PI / 6, operation, reverse); cells++;
    }
    for (const [side, h, theta] of [[0.9, 2.5, Math.PI / 6], [1.05, 2.5, Math.PI / 6], [1, 2.55, Math.PI / 6], [1, 2.5, 0.47], [1, 2.5, 0.58]]) {
      for (const operation of ['cut', 'fuse']) { checkFixture(BrepKernel, api, 1, false, side, h, theta, operation); cells++; }
    }
  }
  console.log(`ok - F13 ${cells} direct/both-batch cells: material, quantities and three raw mesh deflections`);
}
