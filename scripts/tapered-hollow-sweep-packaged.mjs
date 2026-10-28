/** Independent square-frustum material and volume oracles for P-Class 7.1. */
import assert from 'node:assert/strict';

function invoke(kernel, route, args) {
  if (route === 'direct') return kernel.sweepWithOptions(
    args.profile, args.pathEdge, 'fixed', Float64Array.from(args.scaleValues),
    args.segments, 'smooth',
  );
  const row = JSON.parse(kernel[route](JSON.stringify([{ op: 'sweepWithOptions', args }])))[0];
  assert.ok(Object.hasOwn(row, 'ok'), JSON.stringify(row));
  return row.ok;
}

function closedMesh(kernel, solid, scale) {
  const mesh = JSON.parse(kernel.tessellateSolidGrouped(solid, 0.01 * scale, 0.1));
  assert.ok(mesh.indices.length > 0);
  assert.ok(mesh.positions.every(Number.isFinite));
  const remap = [], vertices = new Map();
  for (let i = 0; i < mesh.positions.length; i += 3) {
    const key = mesh.positions.slice(i, i + 3).map(value => Math.round(value / scale * 1e7)).join(',');
    if (!vertices.has(key)) vertices.set(key, vertices.size);
    remap.push(vertices.get(key));
  }
  const uses = new Map();
  for (let i = 0; i < mesh.indices.length; i += 3) {
    const t = mesh.indices.slice(i, i + 3).map(index => remap[index]);
    for (let j = 0; j < 3; j++) {
      const a = t[j], b = t[(j + 1) % 3];
      assert.notEqual(a, b, 'degenerate welded triangle');
      const key = a < b ? `${a},${b}` : `${b},${a}`;
      const value = uses.get(key) ?? [0, 0];
      value[0]++;
      value[1] += a < b ? 1 : -1;
      uses.set(key, value);
    }
  }
  for (const [edge, [count, balance]] of uses) {
    assert.equal(count, 2, `boundary/non-manifold edge ${edge}`);
    assert.equal(balance, 0, `same-sense mesh edge ${edge}`);
  }
}

export function runTaperedHollowSweepPackaged({ BrepKernel }) {
  let cells = 0;
  for (const route of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const [scale, placed] of [0.001, 1, 1000].flatMap(scale => [false, true].map(placed => [scale, placed]))) {
      for (const scaleValues of [[0, 1, 1, 2], [0, 2, 1, 0.5], [0, 1, 0.5, 2, 1, 1],
        [0, 1 + 5e-8, 1, 1 + 5e-8]]) {
        const kernel = new BrepKernel();
        try {
          const outer = kernel.makePolygonWire(Float64Array.from(
            [-2, -2, 0, 2, -2, 0, 2, 2, 0, -2, 2, 0].map(value => value * scale),
          ));
          const inner = kernel.makePolygonWire(Float64Array.from(
            [-1, -1, 0, -1, 1, 0, 1, 1, 0, 1, -1, 0].map(value => value * scale),
          ));
          const profile = kernel.makeFaceFromWires(outer, Uint32Array.of(inner));
          const origin = placed ? [3 * scale, -5 * scale, 7 * scale] : [0, 0, 0];
          const pathEdge = kernel.makeLineEdge(...origin,
            origin[0], origin[1], origin[2] + 6 * scale);
          const args = { profile, pathEdge, contactMode: 'fixed', scaleValues, segments: 4 };
          const solid = invoke(kernel, route, args);
          let integral = 0;
          for (let i = 0; i + 3 < scaleValues.length; i += 2) {
            const [t0, s0, t1, s1] = scaleValues.slice(i, i + 4);
            integral += (t1 - t0) * (s0 * s0 + s0 * s1 + s1 * s1) / 3;
          }
          const expected = 12 * 6 * integral;
          const volume = kernel.volume(solid, scale * 0.01) / scale ** 3;
          assert.ok(Math.abs(volume - expected) < expected * 1e-8,
            `${route} scale=${scale} placed=${placed}: volume ${volume} vs ${expected}`);
          assert.equal(kernel.validateSolid(solid), 0);
          for (const t of [0.125, 0.375, 0.625, 0.875]) {
            let sectionScale;
            for (let i = 0; i + 3 < scaleValues.length; i += 2) {
              const [t0, s0, t1, s1] = scaleValues.slice(i, i + 4);
              if (t >= t0 && t <= t1) sectionScale = s0 + (s1 - s0) * (t - t0) / (t1 - t0);
            }
            for (const [x, expectedState] of [[0.5, 'outside'], [1.5, 'inside'], [2.5, 'outside']]) {
              assert.equal(kernel.classifyPoint(solid, origin[0] + x * sectionScale * scale,
                origin[1], origin[2] + 6 * scale * t, 1e-7), expectedState);
            }
          }
          closedMesh(kernel, solid, scale);
          const state = () => ({
            source: Array.from(kernel.serializeSolids(Uint32Array.of(solid))),
            history: kernel.journalSummary(),
            memory: JSON.parse(kernel.checkpointMemoryStats()),
          });
          for (const malformed of [[0, 1], [0, 1, 1], [0, 1, 0, 2], [1, 1, 0, 2], [0, 1, 1, 0],
            [-1e308, 1, 1e308, 2]]) {
            const before = state();
            if (route === 'direct') {
              assert.throws(() => invoke(kernel, route, { ...args, scaleValues: malformed }), /scaleValues/);
            } else {
              const row = JSON.parse(kernel[route](JSON.stringify([
                { op: 'sweepWithOptions', args: { ...args, scaleValues: malformed } },
              ])))[0];
              assert.ok(Object.hasOwn(row, 'error'), JSON.stringify(row));
              if (route === 'executeBatchV2') {
                assert.equal(row.error.code, 'invalid_argument');
                assert.equal(row.error.category, 'invalid_input');
              }
            }
            assert.deepEqual(state(), before);
          }
          cells++;
        } finally { kernel.free(); }
      }
    }
  }
  console.log(`ok - tapered hollow sweeps: ${cells} direct/batch positive cells and malformed-law rollback`);
}
