/** F17: bounded exact equal-frustum sections through all shipped JS APIs. */
import assert from 'node:assert/strict';

const refusal = 'exact-only policy: the exact boolean pipeline could not produce this result and the approximate fallback was declined';
const sub = (a, b) => a.map((v, i) => v - b[i]);
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const state = kernel => {
  const usage = JSON.parse(kernel.checkpointMemoryStats());
  return { slots: usage.allocatedSlots, retired: usage.retiredSlots, journal: kernel.journalSummary() };
};
const translation = x => [1, 0, 0, x, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
function matrix(scale, placed) {
  const a = placed ? 0.37 : 0, b = placed ? 0.23 : 0;
  const c = Math.cos(a), s = Math.sin(a), cb = Math.cos(b), sb = Math.sin(b);
  return [cb, -sb, 0, placed ? 13 * scale : 0, c * sb, c * cb, -s, placed ? -7 * scale : 0,
    s * sb, s * cb, c, placed ? 9 * scale : 0, 0, 0, 0, 1];
}
const world = (m, p) => [0, 1, 2].map(r => m[4 * r] * p[0] + m[4 * r + 1] * p[1] + m[4 * r + 2] * p[2] + m[4 * r + 3]);
const local = (m, p, scale) => [0, 1, 2].map(c => (m[c] * (p[0] - m[3])
  + m[c + 4] * (p[1] - m[7]) + m[c + 8] * (p[2] - m[11])) / scale);
function invoke(kernel, api, op, args, direct, label) {
  if (api === 'direct') return kernel[op === 'transform' ? 'transformSolid' : op](...direct);
  const row = JSON.parse(kernel[api](JSON.stringify([{ op, args }])))[0];
  assert.ok(Object.hasOwn(row, 'ok'), `${label}: ${op}: ${JSON.stringify(row)}`);
  return row.ok;
}
function quantities() {
  // Horizontal disc overlap integrated in radius, independently of the
  // radical-plane hyperbola. The native test checks this with Simpson sums.
  const r = 4.5, q = 3.5, k = 4 / 11, chord = Math.sqrt(r * r - q * q), angle = Math.acos(q / r);
  const lens = 2 * (r * r * angle - q * chord);
  const overlap = 2 / (3 * k) * (r ** 3 * angle - 2 * q * r * chord + q ** 3 * Math.acosh(r / q));
  return [2 * Math.PI * 5.5 * (r * r + r * 2.5 + 2.5 ** 2) / 3 - overlap,
    2 * (Math.PI * 7 * Math.hypot(5.5, 2) + Math.PI * (r * r + 2.5 ** 2))
      - lens * (1 + Math.hypot(1, k) / k)];
}
function fixture(BrepKernel, api, scale, placed, swapped, mirrored, operation = 'fuse') {
  const label = `F17 ${api} ${operation} scale=${scale} placed=${placed} swapped=${swapped} mirrored=${mirrored}`;
  const kernel = new BrepKernel(), run = (op, args, direct) => invoke(kernel, api, op, args, direct, label);
  try {
    const args = { bottomRadius: 4.5 * scale, topRadius: 2.5 * scale, height: 5.5 * scale };
    const a = run('makeCone', args, [args.bottomRadius, args.topRadius, args.height]);
    let b;
    if (mirrored) b = run('mirror', { solid: a, px: 3.5 * scale, py: 0, pz: 0, nx: 1, ny: 0, nz: 0 }, [a, 3.5 * scale, 0, 0, 1, 0, 0]);
    else {
      b = run('makeCone', args, [args.bottomRadius, args.topRadius, args.height]);
      const m = translation(7 * scale);
      run('transform', { solid: b, matrix: m }, [b, Float64Array.from(m)]);
    }
    const m = matrix(scale, placed);
    if (placed) for (const solid of [a, b]) run('transform', { solid, matrix: m }, [solid, Float64Array.from(m)]);
    const ids = Uint32Array.of(a, b), original = kernel.serializeSolids(ids);
    const solidA = swapped ? b : a, solidB = swapped ? a : b;
    if (placed || (!mirrored && scale === 0.001)) {
      const before = state(kernel);
      if (api === 'direct') {
        assert.throws(() => kernel.booleanWithQuality('fuse', solidA, solidB, true),
          error => !(error instanceof WebAssembly.RuntimeError) && error.message === refusal, label);
        const detailed = kernel.fuseDetailed(solidA, solidB);
        assert.equal(detailed.status, 'error', label);
        assert.equal(detailed.code, 'exact_only_unattainable', label);
        assert.equal(detailed.category, 'quality_refused', label);
      } else {
        const row = JSON.parse(kernel[api](JSON.stringify([{ op: 'booleanWithQuality',
          args: { operation: 'fuse', solidA, solidB, exactOnly: true } }])))[0];
        if (api === 'executeBatch') assert.deepEqual(row, { error: refusal }, label);
        else {
          assert.equal(row.error.code, 'operation_failed', label);
          assert.equal(row.error.category, 'quality_refused', label);
          assert.equal(row.error.details.kernelCode, 'exact_only_unattainable', label);
          assert.equal(row.error.message, refusal, label);
        }
      }
      assert.deepEqual(state(kernel), before, `${label}: refusal rollback`);
      assert.deepEqual(kernel.serializeSolids(ids), original, `${label}: refused operands`);
      return false;
    }
    const outcome = run('booleanWithQuality', { operation, solidA, solidB, exactOnly: true }, [operation, solidA, solidB, true]);
    assert.equal(outcome.quality, 'exact', label);
    const solid = outcome.solid;
    const doc = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(solid))));
    const root = doc.solids[doc.solid_roots[0]];
    const faces = [root.outer_shell, ...root.inner_shells].flatMap(shell => doc.shells[shell].faces);
    assert.ok(faces.length <= 10, `${label}: analytic census`);
    assert.equal(faces.filter(id => Object.hasOwn(doc.faces[id].surface, 'Cone')).length, 2, label);
    const uses = new Map();
    for (const id of faces) {
      const face = doc.faces[id];
      assert.ok(Object.hasOwn(face.surface, 'Cone') || Object.hasOwn(face.surface, 'Plane'), label);
      for (const wi of [face.outer_wire, ...face.inner_wires]) for (const use of doc.wires[wi].edges) {
        const directions = uses.get(use.edge) ?? [];
        directions.push(use.forward !== face.reversed); uses.set(use.edge, directions);
      }
    }
    for (const directions of uses.values()) {
      assert.equal(directions.length, 2, `${label}: B-Rep incidence`);
      assert.notEqual(directions[0], directions[1], `${label}: B-Rep winding`);
    }
    assert.equal(JSON.parse(kernel.validateSolidDetailed(solid)).errorCount, 0, label);
    const [unionVolume, unionArea] = quantities();
    const primitiveVolume = Math.PI * 5.5 * (4.5 ** 2 + 4.5 * 2.5 + 2.5 ** 2) / 3;
    const primitiveArea = Math.PI * 7 * Math.hypot(5.5, 2) + Math.PI * (4.5 ** 2 + 2.5 ** 2);
    const overlap = 2 * primitiveVolume - unionVolume;
    const lens = 2 * (4.5 ** 2 * Math.acos(3.5 / 4.5) - 3.5 * Math.sqrt(8));
    const [expectedVolume, expectedArea] = operation === 'fuse' ? [unionVolume, unionArea]
      : operation === 'cut' ? [primitiveVolume - overlap, primitiveArea - lens]
        : [overlap, 2 * primitiveArea - unionArea];
    for (const d of [0.0001, 0.00005]) {
      const volume = kernel.volume(solid, d * scale) / scale ** 3;
      const area = kernel.surfaceArea(solid, d * scale) / scale ** 2;
      assert.ok(Math.abs(volume - expectedVolume) < 1e-4 * expectedVolume, `${label}: volume ${volume}`);
      assert.ok(Math.abs(area - expectedArea) < 1e-4 * expectedArea, `${label}: area ${area}`);
    }
    const mass = JSON.parse(kernel.massProperties(solid)).volume / scale ** 3;
    assert.ok(Math.abs(mass - expectedVolume) < 1e-4 * expectedVolume, `${label}: mass ${mass}`);
    for (const z of [0.5, 1.5, 3, 4.5]) for (const x of [-3, 0, 3.5, 7, 10]) for (const y of [-2, 0, 2]) {
      const radius = 4.5 - 4 * z / 11, inA = Math.hypot(x, y) < radius, inB = Math.hypot(x - 7, y) < radius;
      const point = world(m, [x * scale, y * scale, z * scale]);
      const [left, right] = swapped ? [inB, inA] : [inA, inB];
      const resultInside = operation === 'fuse' ? left || right : operation === 'cut' ? left && !right : left && right;
      for (const [id, inside] of [[a, inA], [b, inB], [solid, resultInside]]) {
        assert.equal(run('classifyPoint', { solid: id, x: point[0], y: point[1], z: point[2], tolerance: 1e-7 }, [id, ...point, 1e-7]),
          inside ? 'inside' : 'outside', `${label}: material ${x},${y},${z}`);
      }
    }
    for (const d of [0.1, 0.01]) {
      const mesh = JSON.parse(kernel.tessellateSolidGrouped(solid, d * scale, 0.35));
      const points = Array.from({ length: mesh.positions.length / 3 }, (_, i) => local(m, mesh.positions.slice(3 * i, 3 * i + 3), scale));
      const meshUses = new Map(), origin = points[mesh.indices[0]];
      let signed = 0;
      assert.equal(mesh.faceOffsets.length, faces.length + 1, label);
      for (let fi = 0; fi < faces.length; fi++) {
        assert.ok(mesh.faceOffsets[fi + 1] > mesh.faceOffsets[fi], `${label}: empty face`);
        for (let at = mesh.faceOffsets[fi]; at < mesh.faceOffsets[fi + 1]; at += 3) {
          const tri = mesh.indices.slice(at, at + 3), [a, b, c] = tri.map(i => sub(points[i], origin));
          signed += dot(a, cross(b, c)) / 6;
          for (const [a, b] of [[tri[0], tri[1]], [tri[1], tri[2]], [tri[2], tri[0]]]) {
            const key = a < b ? `${a},${b}` : `${b},${a}`, directions = meshUses.get(key) ?? [];
            directions.push(a < b); meshUses.set(key, directions);
          }
        }
      }
      for (const directions of meshUses.values()) {
        assert.equal(directions.length, 2, `${label}: raw mesh incidence`);
        assert.notEqual(directions[0], directions[1], `${label}: raw mesh winding`);
      }
      assert.ok(signed > 0, `${label}: inward raw mesh`);
      if (operation === 'fuse') assert.ok(Math.abs(signed - expectedVolume) < 0.01 * expectedVolume,
        `${label}: signed raw mesh volume ${signed}`);
    }
    assert.deepEqual(kernel.serializeSolids(ids), original, `${label}: unchanged operands`);
    return true;
  } finally { kernel.free(); }
}
export function runOpseqF17Packaged({ BrepKernel }) {
  const failures = [];
  let accepted = 0, refused = 0;
  for (const api of ['direct', 'executeBatch', 'executeBatchV2']) for (const mirrored of [false, true]) {
    for (const scale of [0.001, 1, 1000]) for (const placed of [false, true]) for (const swapped of [false, true]) {
      try { if (fixture(BrepKernel, api, scale, placed, swapped, mirrored)) accepted++; else refused++; }
      catch (error) { failures.push(error); }
    }
  }
  for (const api of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const [operation, mirrored] of [['cut', false], ['intersect', false], ['intersect', true]]) {
      try { fixture(BrepKernel, api, 1, false, false, mirrored, operation); }
      catch (error) { failures.push(error); }
    }
  }
  if (failures.length) throw new AggregateError(failures, `F17: ${accepted} accepted, ${refused} refused of 72 fuse cells plus 9 operator controls`);
  assert.equal(accepted, 30); assert.equal(refused, 42);
  console.log('ok - F17 complete equal-frustum fuse: 30 exact, 42 atomic refusals, 9 complementary operator controls across direct/both batch APIs');
}
